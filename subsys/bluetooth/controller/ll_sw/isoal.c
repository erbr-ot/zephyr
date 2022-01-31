/*
 * Copyright (c) 2021 Demant
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include <zephyr/types.h>
#include <sys/types.h>
#include <toolchain.h>
#include <sys/util.h>

#include <zephyr.h>

#include <bluetooth/bluetooth.h>

#include "util/memq.h"
#include "pdu.h"


#include "ll.h"
#include "lll.h"
#include "lll_conn_iso.h"
#include "isoal.h"
#include "ull_iso_types.h"

#define LOG_MODULE_NAME bt_ctlr_isoal
#include "common/log.h"
#include "hal/debug.h"

/* TODO this must be taken from a Kconfig */
#define ISOAL_SINKS_MAX   (4)
#define ISOAL_SOURCES_MAX (4)

/** Allocation state */
typedef uint8_t isoal_alloc_state_t;
#define ISOAL_ALLOC_STATE_FREE            ((isoal_alloc_state_t) 0x00)
#define ISOAL_ALLOC_STATE_TAKEN           ((isoal_alloc_state_t) 0x01)

struct
{
	isoal_alloc_state_t  sink_allocated[ISOAL_SINKS_MAX];
	isoal_alloc_state_t  source_allocated[ISOAL_SOURCES_MAX];
	struct isoal_sink    sink_state[ISOAL_SINKS_MAX];
	struct isoal_source  source_state[ISOAL_SOURCES_MAX];
} isoal_global;


/**
 * @brief Internal reset
 * Zero-init entire ISO-AL state
 */
static isoal_status_t isoal_init_reset(void)
{
	memset(&isoal_global, 0, sizeof(isoal_global));
	return ISOAL_STATUS_OK;
}

/**
 * @brief  Initialize ISO-AL
 */
isoal_status_t isoal_init(void)
{
	isoal_status_t err = ISOAL_STATUS_OK;

	err = isoal_init_reset();

	return err;
}

/** Clean up and reinitialize */
isoal_status_t isoal_reset(void)
{
	isoal_status_t err = ISOAL_STATUS_OK;

	err = isoal_init_reset();

	return err;
}

/**
 * @brief Find free sink from statically-sized pool and allocate it
 * @details Implemented as linear search since pool is very small
 *
 * @param hdl[out]  Handle to sink
 * @return ISOAL_STATUS_OK if we could allocate; otherwise ISOAL_STATUS_ERR_SINK_ALLOC
 */
static isoal_status_t isoal_sink_allocate(isoal_sink_handle_t *hdl)
{
	isoal_sink_handle_t i;

	/* Very small linear search to find first free */
	for (i = 0; i < ISOAL_SINKS_MAX; i++) {
		if (isoal_global.sink_allocated[i] == ISOAL_ALLOC_STATE_FREE) {
			isoal_global.sink_allocated[i] = ISOAL_ALLOC_STATE_TAKEN;
			*hdl = i;
			return ISOAL_STATUS_OK;
		}
	}

	return ISOAL_STATUS_ERR_SINK_ALLOC; /* All entries were taken */
}

/**
 * @brief Mark a sink as being free to allocate again
 * @param hdl[in]  Handle to sink
 */
static void isoal_sink_deallocate(isoal_sink_handle_t hdl)
{
	isoal_global.sink_allocated[hdl] = ISOAL_ALLOC_STATE_FREE;
}

/**
 * @brief Create a new sink
 *
 * @param handle[in]            Connection handle
 * @param role[in]              Peripheral, Central or Broadcast
 * @param burst_number[in]      Burst Number
 * @param flush_timeout[in]     Flush timeout
 * @param sdu_interval[in]      SDU interval
 * @param iso_interval[in]      ISO interval
 * @param stream_sync_delay[in] CIS sync delay
 * @param group_sync_delay[in]  CIG sync delay
 * @param sdu_alloc[in]         Callback of SDU allocator
 * @param sdu_emit[in]          Callback of SDU emitter
 * @param sdu_write[in]         Callback of SDU byte writer
 * @param hdl[out]              Handle to new sink
 *
 * @return ISOAL_STATUS_OK if we could create a new sink; otherwise ISOAL_STATUS_ERR_SINK_ALLOC
 */
isoal_status_t isoal_sink_create(
	uint16_t                 handle,
	uint8_t                  role,
	uint8_t                  burst_number,
	uint8_t                  flush_timeout,
	uint32_t                 sdu_interval,
	uint16_t                 iso_interval,
	uint32_t                 stream_sync_delay,
	uint32_t                 group_sync_delay,
	isoal_sink_sdu_alloc_cb  sdu_alloc,
	isoal_sink_sdu_emit_cb   sdu_emit,
	isoal_sink_sdu_write_cb  sdu_write,
	isoal_sink_handle_t      *hdl)
{
	isoal_status_t err;

	/* Allocate a new sink */
	err = isoal_sink_allocate(hdl);
	if (err) {
		return err;
	}

	struct isoal_sink_session *session = &isoal_global.sink_state[*hdl].session;

	session->handle = handle;

	/* Todo: Next section computing various constants, should potentially be a
	 * function in itself as a number of the dependencies could be changed while
	 * a connection is active.
	 */

	/* Note: sdu_interval unit is uS, iso_interval is a multiple of 1.25mS */
	session->pdus_per_sdu = burst_number * (sdu_interval / (iso_interval * 1250));

	/* Computation of transport latency (constant part)
	 *
	 * Unframed case:
	 *
	 * C->P: SDU_Synchronization_Reference =
	 *   CIS reference anchor point + CIS_Sync_Delay + (FT_C_To_P - 1) * ISO_Interval
	 *
	 * P->C: SDU_Synchronization_Reference =
	 *   CIS reference anchor point + CIS_Sync_Delay - CIG_Sync_Delay -
	 *   ((ISO_Interval / SDU interval)-1) * SDU interval
	 *
	 * BIS: SDU_Synchronization_Reference =
	 *    BIG reference anchor point + BIG_Sync_Delay
	 *
	 * Framed case:
	 *
	 * C->P: SDU_Synchronization_Reference =
	 *   CIS Reference Anchor point +
	 *   CIS_Sync_Delay + SDU_Interval_C_To_P + FT_C_To_P * ISO_Interval -
	 *   Time_Offset
	 *
	 * P->C: synchronization reference SDU = CIS reference anchor point +
	 *   CIS_Sync_Delay - CIG_Sync_Delay - Time_Offset
	 *
	 * BIS: SDU_Synchronization_Reference =
	 *   BIG reference anchor point +
	 *   BIG_Sync_Delay + SDU_interval + ISO_Interval - Time_Offset.
	 */
	if (role == BT_CONN_ROLE_PERIPHERAL) {
		isoal_global.sink_state[*hdl].session.latency_unframed =
			stream_sync_delay + ((flush_timeout - 1) * iso_interval);

		isoal_global.sink_state[*hdl].session.latency_framed =
			stream_sync_delay + sdu_interval + (flush_timeout * iso_interval);
	} else if (role == BT_CONN_ROLE_CENTRAL) {
		isoal_global.sink_state[*hdl].session.latency_unframed =
			stream_sync_delay - group_sync_delay -
			(((iso_interval / sdu_interval) - 1) * iso_interval);

		isoal_global.sink_state[*hdl].session.latency_framed =
			stream_sync_delay - group_sync_delay;
	} else if (role == BT_ROLE_BROADCAST) {
		isoal_global.sink_state[*hdl].session.latency_unframed =
			group_sync_delay;

		isoal_global.sink_state[*hdl].session.latency_framed =
			group_sync_delay + sdu_interval + iso_interval;

	} else {
		LL_ASSERT(0);
	}

	/* Remember the platform-specific callbacks */
	session->sdu_alloc = sdu_alloc;
	session->sdu_emit  = sdu_emit;
	session->sdu_write = sdu_write;

	/* Initialize running seq number to zero */
	session->seqn = 0;

	return err;
}

/**
 * @brief Find free source from statically-sized pool and allocate it
 * @details Implemented as linear search since pool is very small
 *
 * @param hdl[out]  Handle to source
 * @return ISOAL_STATUS_OK if we could allocate; otherwise ISOAL_STATUS_ERR_SOURCE_ALLOC
 */
static isoal_status_t isoal_source_allocate(isoal_source_handle_t *hdl)
{
	isoal_source_handle_t i;

	/* Very small linear search to find first free */
	for (i = 0; i < ISOAL_SOURCES_MAX; i++) {
		if (isoal_global.source_allocated[i] == ISOAL_ALLOC_STATE_FREE) {
			isoal_global.source_allocated[i] = ISOAL_ALLOC_STATE_TAKEN;
			*hdl = i;
			return ISOAL_STATUS_OK;
		}
	}

	return ISOAL_STATUS_ERR_SOURCE_ALLOC; /* All entries were taken */
}

/**
 * @brief Mark a source as being free to allocate again
 * @param hdl[in]  Handle to source
 */
static void isoal_source_deallocate(isoal_source_handle_t hdl)
{
	isoal_global.source_allocated[hdl] = ISOAL_ALLOC_STATE_FREE;
}



/**
 * @brief Create a new source
 *
 * @param handle[in]            Connection handle
 * @param role[in]              Peripheral, Central or Broadcast
 * @param burst_number[in]      Burst Number
 * @param flush_timeout[in]     Flush timeout
 * @param max_octets[in]        Maximum PDU size (Max_PDU_C_To_P / Max_PDU_P_To_C)
 * @param sdu_interval[in]      SDU interval
 * @param iso_interval[in]      ISO interval
 * @param stream_sync_delay[in] CIS sync delay
 * @param group_sync_delay[in]  CIG sync delay
 * @param hdl[out]              Handle to new source
 *
 * @return ISOAL_STATUS_OK if we could create a new sink; otherwise ISOAL_STATUS_ERR_SOURCE_ALLOC
 */
isoal_status_t isoal_source_create(
	uint16_t                  handle,
	uint8_t                   role,
	uint8_t                   burst_number,
	uint8_t                   flush_timeout,
	uint8_t                   max_octets,
	uint32_t                  sdu_interval,
	uint16_t                  iso_interval,
	uint32_t                  stream_sync_delay,
	uint32_t                  group_sync_delay,
	isoal_source_handle_t     *hdl)
{
	isoal_status_t err;

	/* Allocate a new source */
	err = isoal_source_allocate(hdl);
	if (err) {
		return err;
	}

	struct isoal_source_session *session = &isoal_global.source_state[*hdl].session;

	session->handle = handle;

	/* Todo: Next section computing various constants, should potentially be a
	 * function in itself as a number of the dependencies could be changed while
	 * a connection is active.
	 */

	/* Note: sdu_interval unit is uS, iso_interval is a multiple of 1.25mS */
	session->pdus_per_sdu = burst_number * (sdu_interval / (iso_interval * 1250));
	/* Set maximum PDU size */
	session->max_pdu_size = max_octets;


	/* TODO: Constant need to be updated */

	/* Initialize running seq number to zero */
	session->seqn = 0;

	return err;
}

/**
 * @brief Get reference to configuration struct
 *
 * @param hdl[in]   Handle to new sink
 * @return Reference to parameter struct, to be configured by caller
 */
struct isoal_sink_config *isoal_get_sink_param_ref(isoal_sink_handle_t hdl)
{
	LL_ASSERT(isoal_global.sink_allocated[hdl] == ISOAL_ALLOC_STATE_TAKEN);

	return &isoal_global.sink_state[hdl].session.param;
}

/**
 * @brief Atomically enable latch-in of packets and SDU production
 * @param hdl[in]  Handle of existing instance
 */
void isoal_sink_enable(isoal_sink_handle_t hdl)
{
	/* Reset bookkeeping state */
	memset(&isoal_global.sink_state[hdl].sdu_production, 0,
	       sizeof(isoal_global.sink_state[hdl].sdu_production));

	/* Atomically enable */
	isoal_global.sink_state[hdl].sdu_production.mode = ISOAL_PRODUCTION_MODE_ENABLED;
}

/**
 * @brief Atomically disable latch-in of packets and SDU production
 * @param hdl[in]  Handle of existing instance
 */
void isoal_sink_disable(isoal_sink_handle_t hdl)
{
	/* Atomically disable */
	isoal_global.sink_state[hdl].sdu_production.mode = ISOAL_PRODUCTION_MODE_DISABLED;
}

/**
 * @brief Disable and deallocate existing sink
 * @param hdl[in]  Handle of existing instance
 */
void isoal_sink_destroy(isoal_sink_handle_t hdl)
{
	/* Atomic disable */
	isoal_sink_disable(hdl);

	/* Permit allocation anew */
	isoal_sink_deallocate(hdl);
}

/**
 * @brief Get reference to configuration struct
 *
 * @param hdl[in]   Handle to new source
 * @return Reference to parameter struct, to be configured by caller
 */
struct isoal_source_config *isoal_get_source_param_ref(isoal_source_handle_t hdl)
{
	LL_ASSERT(isoal_global.source_allocated[hdl] == ISOAL_ALLOC_STATE_TAKEN);

	return &isoal_global.source_state[hdl].session.param;
}

/**
 * @brief Atomically enable latch-in of packets and PDU production
 * @param hdl[in]  Handle of existing instance
 */
void isoal_source_enable(isoal_source_handle_t hdl)
{
	/* Reset bookkeeping state */
	memset(&isoal_global.source_state[hdl].pdu_production, 0,
	       sizeof(isoal_global.source_state[hdl].pdu_production));

	/* Atomically enable */
	isoal_global.source_state[hdl].pdu_production.mode = ISOAL_PRODUCTION_MODE_ENABLED;
}

/**
 * @brief Atomically disable latch-in of packets and PDU production
 * @param hdl[in]  Handle of existing instance
 */
void isoal_source_disable(isoal_source_handle_t hdl)
{
	/* Atomically disable */
	isoal_global.source_state[hdl].pdu_production.mode = ISOAL_PRODUCTION_MODE_DISABLED;
}

/**
 * @brief Disable and deallocate existing source
 * @param hdl[in]  Handle of existing instance
 */
void isoal_source_destroy(isoal_source_handle_t hdl)
{
	/* Atomic disable */
	isoal_source_disable(hdl);

	/* Permit allocation anew */
	isoal_source_deallocate(hdl);
}

/* Obtain destination SDU */
static isoal_status_t isoal_rx_allocate_sdu(struct isoal_sink *sink,
					    const struct isoal_pdu_rx *pdu_meta)
{
	struct isoal_sink_session *session;
	struct isoal_sdu_production *sp;
	struct isoal_sdu_produced *sdu;
	isoal_status_t err;

	err = ISOAL_STATUS_OK;
	session = &sink->session;
	sp = &sink->sdu_production;
	sdu = &sp->sdu;

	/* Allocate a SDU if the previous was filled (thus sent) */
	const bool sdu_complete = (sp->sdu_available == 0);

	if (sdu_complete) {
		/* Allocate new clean SDU buffer */
		err = session->sdu_alloc(
			sink,
			pdu_meta,      /* [in]  PDU origin may determine buffer */
			&sdu->contents  /* [out] Updated with pointer and size */
		);

		/* Nothing has been written into buffer yet */
		sp->sdu_written   = 0;
		sp->sdu_available = sdu->contents.size;
		LL_ASSERT(sdu->contents.size > 0);

		/* Remember meta data */
		sdu->status = pdu_meta->meta->status;
		sdu->timestamp = pdu_meta->meta->timestamp;
		/* Get seq number from session counter */
		sdu->seqn = session->seqn;
	}

	return err;
}

static isoal_status_t isoal_rx_try_emit_sdu(struct isoal_sink *sink, bool end_of_sdu)
{
	struct isoal_sdu_production *sp;
	struct isoal_sdu_produced *sdu;
	isoal_status_t err;

	err = ISOAL_STATUS_OK;
	sp = &sink->sdu_production;
	sdu = &sp->sdu;

	/* Emit a SDU */
	const bool sdu_complete = (sp->sdu_available == 0) || end_of_sdu;

	if (end_of_sdu) {
		sp->sdu_available = 0;
	}

	if (sdu_complete) {
		uint8_t next_state = BT_ISO_START;

		switch (sp->sdu_state) {
		case BT_ISO_START:
			if (end_of_sdu) {
				sp->sdu_state = BT_ISO_SINGLE;
				next_state = BT_ISO_START;
			} else {
				sp->sdu_state = BT_ISO_START;
				next_state = BT_ISO_CONT;
			}
			break;
		case BT_ISO_CONT:
			if (end_of_sdu) {
				sp->sdu_state  = BT_ISO_END;
				next_state = BT_ISO_START;
			} else {
				sp->sdu_state  = BT_ISO_CONT;
				next_state = BT_ISO_CONT;
			}
			break;
		}
		sdu->status = sp->sdu_status;
		struct isoal_sink_session *session = &sink->session;

		err = session->sdu_emit(sink, sdu);

		/* update next state */
		sink->sdu_production.sdu_state = next_state;
	}

	return err;
}

static isoal_status_t isoal_rx_append_to_sdu(struct isoal_sink *sink,
					     const struct isoal_pdu_rx *pdu_meta,
					     uint8_t offset,
					     uint8_t length,
					     bool is_end_fragment)
{
	isoal_pdu_len_t packet_available;
	const uint8_t *pdu_payload;
	bool handle_error_case;
	isoal_status_t err;

	/* Might get an empty packed due to errors, we will need to terminate
	 * and send something up anyhow
	 */
	packet_available = length;
	handle_error_case = (is_end_fragment && (packet_available == 0));

	pdu_payload = pdu_meta->pdu->payload + offset;
	LL_ASSERT(pdu_payload);

	/* While there is something left of the packet to consume */
	err = ISOAL_STATUS_OK;
	while ((packet_available > 0) || handle_error_case) {
		const isoal_status_t err_alloc = isoal_rx_allocate_sdu(sink, pdu_meta);
		struct isoal_sdu_production *sp = &sink->sdu_production;
		struct isoal_sdu_produced *sdu = &sp->sdu;

		err |= err_alloc;

		/*
		 * For this SDU we can only consume of packet, bounded by:
		 *   - What can fit in the destination SDU.
		 *   - What remains of the packet.
		 */
		const size_t consume_len = MIN(
			packet_available,
			sp->sdu_available
		);

		if (consume_len > 0) {
			if (pdu_meta->meta->status == ISOAL_PDU_STATUS_VALID) {
				struct isoal_sink_session *session = &sink->session;

				err |= session->sdu_write(sdu->contents.dbuf,
							  pdu_payload,
							  consume_len);
			}
			pdu_payload += consume_len;
			sp->sdu_written   += consume_len;
			sp->sdu_available -= consume_len;
			packet_available  -= consume_len;
		}
		bool end_of_sdu = (packet_available == 0) && is_end_fragment;

		const isoal_status_t err_emit = isoal_rx_try_emit_sdu(sink, end_of_sdu);

		handle_error_case = false;
		err |= err_emit;
	}

	return err;
}


/**
 * @brief Consume an unframed PDU: Copy contents into SDU(s) and emit to a sink
 * @details Destination sink may have an already partially built SDU
 *
 * @param sink[in,out]    Destination sink with bookkeeping state
 * @param pdu_meta[out]  PDU with meta information (origin, timing, status)
 *
 * @return Status
 */
static isoal_status_t isoal_rx_unframed_consume(struct isoal_sink *sink,
						const struct isoal_pdu_rx *pdu_meta)
{
	struct isoal_sink_session *session;
	struct isoal_sdu_production *sp;
	struct node_rx_iso_meta *meta;
	struct pdu_iso *pdu;
	bool end_of_packet;
	uint8_t next_state;
	isoal_status_t err;
	bool pdu_padding;
	uint8_t length;
	bool last_pdu;
	bool pdu_err;
	bool seq_err;
	uint8_t llid;

	sp = &sink->sdu_production;
	session = &sink->session;
	meta = pdu_meta->meta;
	pdu = pdu_meta->pdu;

	err = ISOAL_STATUS_OK;
	next_state = ISOAL_START;

	/* If status is not ISOAL_PDU_STATUS_VALID, length and LLID cannot be trusted */
	llid = pdu_meta->pdu->ll_id;
	pdu_err = (pdu_meta->meta->status != ISOAL_PDU_STATUS_VALID);
	length = pdu_err ? 0 : pdu_meta->pdu->length;
	pdu_padding = (length == 0) && (llid == PDU_BIS_LLID_START_CONTINUE) && !pdu_err;

	if (sp->fsm == ISOAL_START) {
		struct isoal_sdu_produced *sdu;
		uint32_t anchorpoint;
		uint32_t latency;

		sp->sdu_status = ISOAL_SDU_STATUS_VALID;
		sp->sdu_state = BT_ISO_START;
		sp->pdu_cnt = 1;
		session->seqn++;
		seq_err = false;

		/* Todo: anchorpoint must be reference anchor point, should be fixed in LL */
		anchorpoint = meta->timestamp;
		latency = session->latency_unframed;
		sdu = &sp->sdu;

		sdu->timestamp = anchorpoint + latency;
	} else {
		sp->pdu_cnt++;
		seq_err = (meta->payload_number != (sp->prev_pdu_id+1));
	}

	last_pdu = (sp->pdu_cnt == session->pdus_per_sdu);
	end_of_packet = (llid == PDU_BIS_LLID_COMPLETE_END) || last_pdu || pdu_err;

	switch (sp->fsm) {
	case ISOAL_START:
	case ISOAL_CONTINUE:
		if (pdu_err || seq_err) {
			/* PDU contains errors */
			if (last_pdu) {
				/* Last PDU all done */
				next_state = ISOAL_START;
			} else {
				next_state = ISOAL_ERR_SPOOL;
			}
		} else if (llid == PDU_BIS_LLID_START_CONTINUE) {
			/* PDU contains a continuation (neither start of end) fragment of SDU */
			if (last_pdu) {
				/* last pdu in sdu, but end fragment not seen, emit with error */
				next_state = ISOAL_START;
			} else {
				next_state = ISOAL_CONTINUE;
			}
		} else if (llid == PDU_BIS_LLID_COMPLETE_END) {
			/* PDU contains end fragment of a fragmented SDU */
			if (last_pdu) {
				/* Last PDU all done */
				next_state = ISOAL_START;
			} else {
				/* Padding after end fragment to follow */
				next_state = ISOAL_ERR_SPOOL;
			}
		} else  {
			/* Unsupported case */
			err = ISOAL_STATUS_ERR_UNSPECIFIED;
		}
		break;

	case ISOAL_ERR_SPOOL:
		/* State assumes that at end fragment or err has been seen,
		 * now just consume the rest
		 */
		if (last_pdu) {
			/* Last padding seen, restart */
			next_state = ISOAL_START;
		} else {
			next_state = ISOAL_ERR_SPOOL;
		}
		break;

	}

	/* Update error state */
	if (pdu_err && !pdu_padding) {
		sp->sdu_status |= meta->status;
	} else if (last_pdu && (llid != PDU_BIS_LLID_COMPLETE_END) &&
				(sp->fsm  != ISOAL_ERR_SPOOL)) {
		/* END fragment never seen */
		sp->sdu_status |= ISOAL_SDU_STATUS_ERRORS;
	} else if (seq_err) {
		sp->sdu_status |= ISOAL_SDU_STATUS_LOST_DATA;
	}

	/* Append valid PDU to SDU */
	if (!pdu_padding) {
		err |= isoal_rx_append_to_sdu(sink, pdu_meta, 0,
					      length, end_of_packet);
	}

	/* Update next state */
	sp->fsm = next_state;
	sp->prev_pdu_id = meta->payload_number;

	return err;
}

/**
 * @brief Consume a framed PDU: Copy contents into SDU(s) and emit to a sink
 * @details Destination sink may have an already partially built SDU
 *
 * @param sink[in,out]   Destination sink with bookkeeping state
 * @param pdu_meta[out]  PDU with meta information (origin, timing, status)
 *
 * @return Status
 */
static isoal_status_t isoal_rx_framed_consume(struct isoal_sink *sink,
					      const struct isoal_pdu_rx *pdu_meta)
{
	struct isoal_sink_session *session;
	struct isoal_sdu_production *sp;
	struct isoal_sdu_produced *sdu;
	struct pdu_iso_sdu_sh *seg_hdr;
	struct node_rx_iso_meta *meta;
	uint32_t anchorpoint;
	uint8_t *end_of_pdu;
	uint32_t timeoffset;
	isoal_status_t err;
	uint8_t next_state;
	uint32_t timestamp;
	uint32_t latency;
	bool pdu_padding;
	bool pdu_err;
	bool seq_err;

	sp = &sink->sdu_production;
	session = &sink->session;
	meta = pdu_meta->meta;
	sdu = &sp->sdu;

	err = ISOAL_STATUS_OK;
	next_state = ISOAL_START;
	pdu_err = (pdu_meta->meta->status != ISOAL_PDU_STATUS_VALID);
	pdu_padding = (pdu_meta->pdu->length == 0);

	if (sp->fsm == ISOAL_START) {
		seq_err = false;
	} else {
		seq_err = (meta->payload_number != (sp->prev_pdu_id + 1));
	}

	end_of_pdu = ((uint8_t *) pdu_meta->pdu->payload) + pdu_meta->pdu->length - 1;
	seg_hdr = (struct pdu_iso_sdu_sh *) pdu_meta->pdu->payload;

	if (pdu_err || seq_err) {
		/* When one or more ISO Data PDUs are not received, the receiving device may
		 * discard all SDUs affected by the missing PDUs. Any partially received SDU
		 * may also be discarded.
		 */
		next_state = ISOAL_ERR_SPOOL;
		/* Update next state */
		sink->sdu_production.fsm = next_state;

		if (pdu_err) {
			sp->sdu_status |= meta->status;
		} else if (seq_err) {
			sp->sdu_status |= ISOAL_SDU_STATUS_LOST_DATA;
		}

		/* Flush current SDU with error if any */
		err |= isoal_rx_append_to_sdu(sink, pdu_meta, 0, 0, true);

		/* Skip searching this PDU */
		seg_hdr = NULL;
	}

	if (pdu_padding) {
		/* Skip searching this PDU */
		seg_hdr = NULL;
	}

	while (seg_hdr) {
		bool append = true;
		const uint8_t sc    = seg_hdr->sc;
		const uint8_t cmplt = seg_hdr->cmplt;

		if (sp->fsm == ISOAL_START) {
			sp->sdu_status = ISOAL_SDU_STATUS_VALID;
			sp->sdu_state  = BT_ISO_START;
			session->seqn++;
		}

		switch (sp->fsm) {
		case ISOAL_START:
			timeoffset = seg_hdr->timeoffset;
			anchorpoint = meta->timestamp;
			latency = session->latency_framed;
			timestamp = anchorpoint + latency - timeoffset;

			if (!sc && !cmplt) {
				/* The start of a new SDU, where not all SDU data is included in
				 * the current PDU, and additional PDUs are required to complete
				 * the SDU.
				 */
				sdu->timestamp = timestamp;
				next_state = ISOAL_CONTINUE;
			} else if (!sc && cmplt) {
				/* The start of a new SDU that contains the full SDU data in the
				 * current PDU.
				 */
				sdu->timestamp = timestamp;
				next_state = ISOAL_START;
			} else {
				/* Unsupported case */
				err = ISOAL_STATUS_ERR_UNSPECIFIED;
			}
			break;

		case ISOAL_CONTINUE:
			if (sc && !cmplt) {
				/* The continuation of a previous SDU. The SDU payload is appended
				 * to the previous data and additional PDUs are required to
				 * complete the SDU.
				 */
				next_state = ISOAL_CONTINUE;
			} else if (sc && cmplt) {
				/* The continuation of a previous SDU.
				 * Frame data is appended to previously received SDU data and
				 * completes in the current PDU.
				 */
				next_state = ISOAL_START;
			} else {
				/* Unsupported case */
				err = ISOAL_STATUS_ERR_UNSPECIFIED;
			}
			break;

		case ISOAL_ERR_SPOOL:
			/* In error state, search for valid next start of SDU */
			timeoffset = seg_hdr->timeoffset;
			anchorpoint = meta->timestamp;
			latency = session->latency_framed;
			timestamp = anchorpoint + latency - timeoffset;

			if (!sc && !cmplt) {
				/* The start of a new SDU, where not all SDU data is included in
				 * the current PDU, and additional PDUs are required to complete
				 * the SDU.
				 */
				sdu->timestamp = timestamp;
				next_state = ISOAL_CONTINUE;
			} else if (!sc && cmplt) {
				/* The start of a new SDU that contains the full SDU data in the
				 * current PDU.
				 */
				sdu->timestamp = timestamp;
				next_state = ISOAL_START;
			} else {
				/* Start not found yet, stay in Error state */
				append = false;
				next_state = ISOAL_ERR_SPOOL;
			}
			break;
		}

		if (append) {
			/* Calculate offset of first payload byte from SDU based on assumption
			 * of No time_offset in header
			 */
			uint8_t offset = ((uint8_t *) seg_hdr) + PDU_ISO_SEG_HDR_SIZE -
					 pdu_meta->pdu->payload;
			uint8_t length = seg_hdr->length;

			if (!sc) {
				/* time_offset included in header, don't copy offset field to SDU */
				offset = offset + PDU_ISO_SEG_TIMEOFFSET_SIZE;
				length = length - PDU_ISO_SEG_TIMEOFFSET_SIZE;
			}

			/* Todo: check if effective len=0 what happens then?
			 * We should possibly be able to send empty packets with only time stamp
			 */

			err |= isoal_rx_append_to_sdu(sink, pdu_meta, offset, length, cmplt);
		}

		/* Update next state */
		sp->fsm = next_state;

		/* Find next segment header, set to null if past end of PDU */
		seg_hdr = (struct pdu_iso_sdu_sh *) (((uint8_t *) seg_hdr) +
						     seg_hdr->length + PDU_ISO_SEG_HDR_SIZE);

		if (((uint8_t *) seg_hdr) > end_of_pdu) {
			seg_hdr = NULL;
		}
	}

	sp->prev_pdu_id = meta->payload_number;

	return err;
}

/**
 * @brief Deep copy a PDU, recombine into SDU(s)
 * @details Recombination will occur individually for every enabled sink
 *
 * @param sink_hdl[in] Handle of destination sink
 * @param pdu_meta[in] PDU along with meta information (origin, timing, status)
 * @return Status
 */
isoal_status_t isoal_rx_pdu_recombine(isoal_sink_handle_t sink_hdl,
				      const struct isoal_pdu_rx *pdu_meta)
{
	struct isoal_sink *sink = &isoal_global.sink_state[sink_hdl];
	isoal_status_t err = ISOAL_STATUS_ERR_SDU_ALLOC;

	if (sink->sdu_production.mode != ISOAL_PRODUCTION_MODE_DISABLED) {
		bool pdu_framed = (pdu_meta->pdu->ll_id == PDU_BIS_LLID_FRAMED);

		if (pdu_framed) {
			err = isoal_rx_framed_consume(sink, pdu_meta);
		} else {
			err = isoal_rx_unframed_consume(sink, pdu_meta);
		}
	}

	return err;
}


/**
 * Allocate a PDU from the LL and store the details in the given buffer. Allocation
 * is not expected to fail as there must always be sufficient PDU buffers. Any
 * failure will trigger the assert.
 * @param[in]  pdu_buffer Buffer to store PDU details in
 * @return     Error status of operation
 */
static isoal_status_t isoal_tx_pdu_alloc(struct isoal_pdu_buffer *pdu_buffer)
{
	struct node_tx_iso *node_tx;

	node_tx = ll_iso_tx_mem_acquire();
	if (!node_tx) {
		BT_ERR("Tx Buffer Overflow");
		LL_ASSERT(0);
		return ISOAL_STATUS_ERR_PDU_ALLOC;
	}

	/* node_tx handle will be required to emit the PDU later */
	pdu_buffer->handle = (void *)node_tx;
	pdu_buffer->pdu    = (void *)node_tx->pdu;
	/* Use TX buffer size as the limit here. Actual size will be decided in
	 * the ISOAL based on the minimum of the buffer size and the respective
	 * Max_PDU_C_To_P or Max_PDU_P_To_C.
	 */
	pdu_buffer->size   = ISO_TX_BUFFER_SIZE;

	return ISOAL_STATUS_OK;
}

/**
 * Queue the PDU in production in the relevant LL transmit queue. If the
 * attmept to release the PDU fails, the buffer linked to the PDU will be released
 * and it will not be possible to retry the emit operation on the same PDU.
 * @param[in]  source_ctx   ISO-AL source reference for this CIS / BIS
 * @param[in]  produced_pdu PDU in production
 * @param[in]  pdu_ll_id    LLID to be set indicating the type of fragment
 * @param[in]  payload_size Length of the data written to the PDU
 * @return     Error status of the operation
 */
static isoal_status_t isoal_tx_pdu_emit(const struct isoal_source       *source_ctx,
			       const struct isoal_pdu_produced *produced_pdu,
			       const uint8_t                   pdu_ll_id,
			       const isoal_pdu_len_t           payload_size)
{
	struct node_tx_iso *node_tx;
	uint16_t handle;

	/* Retreive CIS / BIS handle */
	handle = bt_iso_handle(source_ctx->session.handle);

	/* Retrieve Node handle */
	node_tx = produced_pdu->contents.handle;
	/* Set PDU LLID */
	produced_pdu->contents.pdu->ll_id = pdu_ll_id;
	/* Set PDU length */
	produced_pdu->contents.pdu->length = (uint8_t)payload_size;

	/* Attempt to enqueue the node towards the LL */
	if (ll_iso_tx_mem_enqueue(handle, node_tx)) {
		/* If it fails, the node will be released and no further attempt
		 * will be possible
		 */
		BT_ERR("Failed to release node (%p)", node_tx);
		ll_iso_tx_mem_release(node_tx);
		return ISOAL_STATUS_ERR_PDU_EMIT;
	}

	return ISOAL_STATUS_OK;
}

/**
 * Write the given SDU payload to the target PDU buffer at the given offset. *
 * @param[in,out]  pdu_buffer  Target PDU buffer
 * @param[in]      pdu_offset  Offset / current write position within PDU
 * @param[in]      sdu_payload Location of source data
 * @param[in]      consume_len Length of data to copy
 * @return         Error status of write operation
 */
static isoal_status_t isoal_tx_pdu_write(struct isoal_pdu_buffer *pdu_buffer,
				       const size_t  pdu_offset,
				       const uint8_t *sdu_payload,
				       const size_t  consume_len)
{
	LL_ASSERT(pdu_buffer);
	LL_ASSERT(pdu_buffer->pdu);
	LL_ASSERT(sdu_payload);


	if ((pdu_offset + consume_len) > pdu_buffer->size) {
		/* Exceeded PDU buffer */
		return ISOAL_STATUS_ERR_UNSPECIFIED;
	}

	/* Copy source to destination at given offset */
	memcpy(&pdu_buffer->pdu->payload[pdu_offset], sdu_payload, consume_len);

	return ISOAL_STATUS_OK;
}

/* Allocates a new PDU only if the previous PDU was emitted */
static isoal_status_t isoal_tx_allocate_pdu(struct isoal_source *source,
					    const struct isoal_sdu_tx *tx_sdu)
{
	struct isoal_source_session *session;
	struct isoal_pdu_production *pp;
	struct isoal_pdu_produced   *pdu;
	isoal_status_t err;

	err = ISOAL_STATUS_OK;
	session = &source->session;
	pp = &source->pdu_production;
	pdu = &pp->pdu;

	/* Allocate a PDU if the previous was filled (thus sent) */
	const bool pdu_complete = (pp->pdu_available == 0);

	if (pdu_complete) {
		/* Allocate new PDU buffer */
		err = isoal_tx_pdu_alloc(
			&pdu->contents  /* [out] Updated with pointer and size */
		);

		if (err){
			pdu->contents.handle = NULL;
			pdu->contents.pdu    = NULL;
			pdu->contents.size   = 0;
		}

		/* Get maximum buffer available */
		const size_t available_len = MIN(
			session->max_pdu_size,
			pdu->contents.size
		);

		/* Nothing has been written into buffer yet */
		pp->pdu_written   = 0;
		pp->pdu_available = available_len;
		LL_ASSERT(available_len > 0);

		pp->pdu_cnt++;
	}

	return err;
}

/**
 * Attempt to emit the PDU in production if it is complete.
 * @param[in]  source     ISO-AL source reference
 * @param[in]  end_of_sdu SDU end has been reached
 * @param[in]  pdu_ll_id  LLID / PDU fragment type as Start, Cont, End, Single (Unframed) or Framed
 * @return     Error status of operation
 */
static isoal_status_t isoal_tx_try_emit_pdu(struct isoal_source *source, bool end_of_sdu, uint8_t pdu_ll_id)
{
	struct isoal_pdu_production *pp;
	struct isoal_pdu_produced   *pdu;
	isoal_status_t err;

	err = ISOAL_STATUS_OK;
	pp = &source->pdu_production;
	pdu = &pp->pdu;

	/* Emit a PDU */
	const bool pdu_complete = (pp->pdu_available == 0) || end_of_sdu;

	if (end_of_sdu) {
		pp->pdu_available = 0;
	}

	if (pdu_complete) {
		err = isoal_tx_pdu_emit(source, pdu, pdu_ll_id, pp->pdu_written);
	}

	return err;
}


/**
 * @brief Fragment received SDU and produce unframed PDUs
 * @details Destination source may have an already partially built PDU
 *
 * @param source[in,out] Destination source with bookkeeping state
 * @param tx_sdu[in]     SDU with packet boundary information
 *
 * @return Status
 */
static isoal_status_t isoal_tx_unframed_produce(struct isoal_source *source,
						const struct isoal_sdu_tx *tx_sdu)
{
	struct isoal_source_session *session;
	struct isoal_pdu_production *pp;
	isoal_sdu_len_t packet_available;
	const uint8_t   *sdu_payload;
	isoal_status_t  err;
	bool zero_length_sdu;
	bool padding_pdu;
	uint8_t ll_id;

	session     = &source->session;
	pp          = &source->pdu_production;
	padding_pdu = false;
	err         = ISOAL_STATUS_OK;

	packet_available = tx_sdu->size;
	sdu_payload = tx_sdu->dbuf;
	LL_ASSERT(sdu_payload);

	zero_length_sdu = (packet_available == 0 &&
		tx_sdu->sdu_state == BT_ISO_SINGLE);

	if (tx_sdu->sdu_state == BT_ISO_START ||
		tx_sdu->sdu_state == BT_ISO_SINGLE) {
		/* Start of a new SDU */

		/* Update sequence number for received SDU
		 *
		 * BT Core V5.3 : Vol 6 Low Energy Controller : Part G IS0-AL:
		 * 2 ISOAL Features :
		 * SDUs received by the ISOAL from the upper layer shall be given a sequence number
		 * which is initialized to 0 when the CIS or BIS is created.
		 *
		 * NOTE: The upper layer may synchronize its sequence number with the sequence number
		 * in the ISOAL once the Datapath is configured and the link is established.
		 */
		session->seqn++;

		/* Reset PDU fragmentation count for this SDU */
		pp->pdu_cnt = 0;
	}

	/* PDUs should be created until the SDU fragment has been fragmented or if
	 * this is the last fragment of the SDU, until the required padding PDU(s)
	 * are sent.
	 */
	while ((err == ISOAL_STATUS_OK) &&
		((packet_available > 0) || padding_pdu || zero_length_sdu)) {
		const isoal_status_t err_alloc = isoal_tx_allocate_pdu(source, tx_sdu);
		struct isoal_pdu_produced  *pdu = &pp->pdu;
		err |= err_alloc;

		/*
		 * For this PDU we can only consume of packet, bounded by:
		 *   - What can fit in the destination PDU.
		 *   - What remains of the packet.
		 */
		const size_t consume_len = MIN(
			packet_available,
			pp->pdu_available
		);

		if (consume_len > 0) {
			err |= isoal_tx_pdu_write(&pdu->contents,
						  pp->pdu_written,
						  sdu_payload,
						  consume_len);
			sdu_payload       += consume_len;
			pp->pdu_written   += consume_len;
			pp->pdu_available -= consume_len;
			packet_available  -= consume_len;
		}

		/* End of the SDU is reached at the end of the last SDU fragment
		 * or if this is a single fragment SDU
		 */
		bool end_of_sdu = (packet_available == 0) &&
				((tx_sdu->sdu_state == BT_ISO_SINGLE) ||
					(tx_sdu->sdu_state == BT_ISO_END));

		/* Decide PDU type
		 *
		 * BT Core V5.3 : Vol 6 Low Energy Controller : Part G IS0-AL:
		 * 2.1 Unframed PDU :
		 * LLID 0b00 PDU_BIS_LLID_COMPLETE_END:
		 * (1) When the payload of the ISO Data PDU contains the end fragment of an SDU.
		 * (2) When the payload of the ISO Data PDU contains a complete SDU.
		 * (3) When an SDU contains zero length data, the corresponding PDU shall be of zero length and
		 *     the LLID field shall be set to 0b00.
		 *
		 * LLID 0b01 PDU_BIS_LLID_COMPLETE_END:
		 * (1) When the payload of the ISO Data PDU contains a start or a continuation fragment of an SDU.
		 * (2) When the ISO Data PDU is used as padding.
		 */
		ll_id = PDU_BIS_LLID_COMPLETE_END;
		if (!end_of_sdu || padding_pdu) {
			ll_id = PDU_BIS_LLID_START_CONTINUE;
		}

		const isoal_status_t err_emit = isoal_tx_try_emit_pdu(source, end_of_sdu, ll_id);

		err |= err_emit;

		/* Send padding PDU(s) if required
		 *
		 * BT Core V5.3 : Vol 6 Low Energy Controller : Part G IS0-AL:
		 * 2.1 Unframed PDU :
		 * Each SDU shall generate BN ÷ (ISO_Interval ÷ SDU_Interval) fragments.
		 * If an SDU generates less than this number of fragments,
		 * empty payloads shall be used to make up the number.
		 */
		padding_pdu = (end_of_sdu && (pp->pdu_cnt < session->pdus_per_sdu));
		zero_length_sdu = false;
	}

	return err;
}


/**
 * @brief Deep copy a SDU, fragment into PDU(s)
 * @details Fragmentation will occur individually for every enabled source
 *
 * @param source_hdl[in] Handle of destination source
 * @param tx_sdu[in]     SDU along with packet boudary state
 * @return Status
 */
isoal_status_t isoal_tx_sdu_fragment(isoal_source_handle_t source_hdl,
				      const struct isoal_sdu_tx *tx_sdu)
{
	struct isoal_source *source = &isoal_global.source_state[source_hdl];
	isoal_status_t err = ISOAL_STATUS_ERR_PDU_ALLOC;

	if (source->pdu_production.mode != ISOAL_PRODUCTION_MODE_DISABLED) {
		/* TODO: consider how to separate framed and unframed production
		 * BT Core V5.3 : Vol 6 Low Energy Controller : Part G IS0-AL:
		 * 2 ISOAL Features :
		 * (1) Unframed PDUs shall only be used when the ISO_Interval is equal to or an
		 * integer multiple of the SDU_Interval and a constant time offset alignment is
		 * maintained between the SDU generation and the timing in the isochronous
		 * transport.
		 * (2) When the Host requests the use of framed PDUs, the Controller shall use framed PDUs.
		 */
		bool pdu_framed = false;

		if (pdu_framed) {
			/* TODO: add framed handling */
		} else {
			err = isoal_tx_unframed_produce(source, tx_sdu);
		}
	}

	return err;
}
