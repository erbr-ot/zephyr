/*
 * Copyright (c) 2020 Demant
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <soc.h>

#include "hal/cpu.h"
#include "hal/ccm.h"

#include "util/util.h"
#include "util/memq.h"
#include "util/mem.h"
#include "util/mfifo.h"
#include "util/mayfly.h"

#include "pdu.h"

#include "lll.h"
#include "lll_sync.h"
#include "lll_sync_iso.h"
#include "lll_conn.h"
#include "lll_conn_iso.h"

#include "isoal.h"

#include "ull_sync_types.h"
#include "ull_iso_types.h"
#include "ull_conn_iso_types.h"
#include "ull_internal.h"
#include "ull_iso_internal.h"

#include "ull_conn_internal.h"
#include "ull_sync_iso_internal.h"
#include "ull_conn_iso_internal.h"
#include "ull_conn_types.h"

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_DEBUG_HCI_DRIVER)
#define LOG_MODULE_NAME bt_ctlr_ull_iso
#include "common/log.h"
#include "hal/debug.h"

#if defined(CONFIG_BT_CTLR_CONN_ISO_STREAMS)
#define BT_CTLR_CONN_ISO_STREAMS CONFIG_BT_CTLR_CONN_ISO_STREAMS
#else /* !CONFIG_BT_CTLR_CONN_ISO_STREAMS */
#define BT_CTLR_CONN_ISO_STREAMS 0
#endif /* !CONFIG_BT_CTLR_CONN_ISO_STREAMS */

#if defined(CONFIG_BT_CTLR_SYNC_ISO_STREAM_COUNT)
#define BT_CTLR_SYNC_ISO_STREAMS (CONFIG_BT_CTLR_SYNC_ISO_STREAM_COUNT)
#else /* !CONFIG_BT_CTLR_SYNC_ISO_STREAM_COUNT */
#define BT_CTLR_SYNC_ISO_STREAMS 0
#endif /* CONFIG_BT_CTLR_SYNC_ISO_STREAM_COUNT */

static int init_reset(void);

static isoal_status_t ll_iso_pdu_alloc(struct isoal_pdu_buffer *pdu_buffer);
static isoal_status_t ll_iso_pdu_write(struct isoal_pdu_buffer *pdu_buffer,
				       const size_t   offset,
				       const uint8_t *sdu_payload,
				       const size_t   consume_len);
static isoal_status_t ll_iso_pdu_release(struct node_tx_iso *node_tx,
					 const uint16_t handle,
					 const isoal_status_t status);

/* Allocate data path pools for RX/TX directions for each stream */
#define BT_CTLR_ISO_STREAMS ((2 * (BT_CTLR_CONN_ISO_STREAMS)) + \
			     BT_CTLR_SYNC_ISO_STREAMS)
#if BT_CTLR_ISO_STREAMS
static struct ll_iso_datapath datapath_pool[BT_CTLR_ISO_STREAMS];
#endif

static void *datapath_free;

#if defined(CONFIG_BT_CTLR_SYNC_ISO) || defined(CONFIG_BT_CTLR_CONN_ISO)
#define NODE_RX_HEADER_SIZE (offsetof(struct node_rx_pdu, pdu))
/* ISO LL conformance tests require a PDU size of maximum 251 bytes + header */
#define ISO_RX_BUFFER_SIZE (2 + 251)

/* Declare the ISO rx node RXFIFO. This is a composite pool-backed MFIFO for
 * rx_nodes. The declaration constructs the following data structures:
 * - mfifo_iso_rx:    FIFO with pointers to PDU buffers
 * - mem_iso_rx:      Backing data pool for PDU buffer elements
 * - mem_link_iso_rx: Pool of memq_link_t elements
 *
 * Two extra links are reserved for use by the ll_iso_rx and ull_iso_rx memq.
 */
static RXFIFO_DEFINE(iso_rx, NODE_RX_HEADER_SIZE + ISO_RX_BUFFER_SIZE,
			     CONFIG_BT_CTLR_ISO_RX_BUFFERS, 2);

static MEMQ_DECLARE(ll_iso_rx);
#if defined(CONFIG_BT_CTLR_ISO_VENDOR_DATA_PATH)
static MEMQ_DECLARE(ull_iso_rx);
static void iso_rx_demux(void *param);
static void ull_iso_link_tx_release(memq_link_t *link);
#endif /* CONFIG_BT_CTLR_ISO_VENDOR_DATA_PATH */
#endif /* CONFIG_BT_CTLR_SYNC_ISO) || CONFIG_BT_CTLR_CONN_ISO */

#if defined(CONFIG_BT_CTLR_ADV_ISO) || defined(CONFIG_BT_CTLR_CONN_ISO)
#define NODE_TX_BUFFER_SIZE MROUND(offsetof(struct node_tx_iso, pdu) + \
				   offsetof(struct pdu_iso, payload) + \
				   ISO_TX_BUFFER_SIZE)

static MFIFO_DEFINE(iso_tx, sizeof(struct lll_tx),
		    CONFIG_BT_CTLR_ISO_TX_BUFFERS);

static struct {
	void *free;
	uint8_t pool[NODE_TX_BUFFER_SIZE * CONFIG_BT_CTLR_ISO_TX_BUFFERS];
} mem_iso_tx;

static struct {
	void *free;
	uint8_t pool[sizeof(memq_link_t) * (CONFIG_BT_CTLR_ISO_TX_BUFFERS + \
					    CONFIG_BT_CTLR_ISO_TX_VENDOR_BUFFERS)];
} mem_link_iso_tx;

static MFIFO_DEFINE(iso_tx_ack, sizeof(struct lll_tx),
		    CONFIG_BT_CTLR_ISO_TX_BUFFERS);
#endif /* CONFIG_BT_CTLR_ADV_ISO || CONFIG_BT_CTLR_CONN_ISO */


/* Must be implemented by vendor */
__weak bool ll_data_path_configured(uint8_t data_path_dir,
				    uint8_t data_path_id)
{
	ARG_UNUSED(data_path_dir);
	ARG_UNUSED(data_path_id);

	return false;
}

/* Contains vendor specific argument, function to be implemented by vendors */
__weak uint8_t ll_configure_data_path(uint8_t data_path_dir,
				      uint8_t data_path_id,
				      uint8_t vs_config_len,
				      uint8_t *vs_config)
{
	ARG_UNUSED(data_path_dir);
	ARG_UNUSED(data_path_id);
	ARG_UNUSED(vs_config_len);
	ARG_UNUSED(vs_config);

	return BT_HCI_ERR_CMD_DISALLOWED;
}

uint8_t ll_read_iso_tx_sync(uint16_t handle, uint16_t *seq,
			    uint32_t *timestamp, uint32_t *offset)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(seq);
	ARG_UNUSED(timestamp);
	ARG_UNUSED(offset);

	return BT_HCI_ERR_CMD_DISALLOWED;
}

/* Must be implemented by vendor */
__weak bool ll_data_path_sink_create(struct ll_iso_datapath *datapath,
				     isoal_sink_sdu_alloc_cb *sdu_alloc,
				     isoal_sink_sdu_emit_cb *sdu_emit,
				     isoal_sink_sdu_write_cb *sdu_write)
{
	ARG_UNUSED(datapath);

	*sdu_alloc = NULL;
	*sdu_emit  = NULL;
	*sdu_write = NULL;

	return false;
}

/* Could be implemented by vendor */
__weak bool ll_data_path_source_create(struct ll_iso_datapath *datapath,
				       isoal_source_pdu_alloc_cb *pdu_alloc,
				       isoal_source_pdu_write_cb *pdu_write,
				       isoal_source_pdu_release_cb *pdu_release)
{
	ARG_UNUSED(datapath);
	ARG_UNUSED(pdu_alloc);
	ARG_UNUSED(pdu_write);
	ARG_UNUSED(pdu_release);

	return false;
}

static inline bool path_is_vendor_specific(uint8_t path_id)
{
	return (path_id >= BT_HCI_DATAPATH_ID_VS &&
		path_id <= BT_HCI_DATAPATH_ID_VS_END);
}

uint8_t ll_setup_iso_path(uint16_t handle, uint8_t path_dir, uint8_t path_id,
			  uint8_t coding_format, uint16_t company_id,
			  uint16_t vs_codec_id, uint32_t controller_delay,
			  uint8_t codec_config_len, uint8_t *codec_config)
{
	ARG_UNUSED(controller_delay);
	ARG_UNUSED(codec_config_len);
	ARG_UNUSED(codec_config);

	isoal_source_handle_t source_handle;
	isoal_sink_handle_t sink_handle;
	uint32_t stream_sync_delay;
	uint32_t group_sync_delay;
	uint8_t flush_timeout;
	uint16_t iso_interval;
	uint32_t sdu_interval;
	uint8_t burst_number;
	uint8_t max_octets;
	isoal_status_t err;
	uint8_t role;

	if (path_id == BT_HCI_DATAPATH_ID_DISABLED) {
		return 0;
	}

#if defined(CONFIG_BT_CTLR_CONN_ISO)
	struct ll_iso_datapath *dp_in = NULL;
	struct ll_iso_datapath *dp_out = NULL;

	struct ll_conn_iso_stream *cis = NULL;
	struct ll_conn_iso_group *cig = NULL;

	if (IS_CIS_HANDLE(handle)) {
		struct ll_conn *conn;

		cis = ll_conn_iso_stream_get(handle);
		if (!cis->group) {
			/* CIS does not belong to a CIG */
			return BT_HCI_ERR_UNKNOWN_CONN_ID;
		}

		conn = ll_connected_get(cis->lll.acl_handle);
		if (conn) {
			/* If we're still waiting for accept/response from
			 * host, path setup is premature and we must return
			 * disallowed status.
			 */
			if (conn->llcp_cis.state == LLCP_CIS_STATE_RSP_WAIT) {
				return BT_HCI_ERR_CMD_DISALLOWED;
			}
		}

		cig = cis->group;
		dp_in = cis->hdr.datapath_in;
		dp_out = cis->hdr.datapath_out;
	}

	/* If the Host attempts to set a data path with a Connection Handle
	 * that does not exist or that is not for a CIS or a BIS, the Controller
	 * shall return the error code Unknown Connection Identifier (0x02)
	 */
	if (!cis) {
		return BT_HCI_ERR_UNKNOWN_CONN_ID;
	}

	if ((path_dir == BT_HCI_DATAPATH_DIR_HOST_TO_CTLR  && dp_in) ||
	    (path_dir == BT_HCI_DATAPATH_DIR_CTLR_TO_HOST && dp_out)) {
		/* Data path has been set up, can only do setup once */
		return BT_HCI_ERR_CMD_DISALLOWED;
	}

	if (path_is_vendor_specific(path_id) &&
	    !ll_data_path_configured(path_dir, path_id)) {
		/* Data path must be configured prior to setup */
		return BT_HCI_ERR_CMD_DISALLOWED;
	}

	/* If Codec_Configuration_Length non-zero and Codec_ID set to
	 * transparent air mode, the Controller shall return the error code
	 * Invalid HCI Command Parameters (0x12).
	 */
	if (codec_config_len && vs_codec_id == BT_HCI_CODING_FORMAT_TRANSPARENT) {
		return BT_HCI_ERR_INVALID_PARAM;
	}
#endif /* CONFIG_BT_CTLR_CONN_ISO */

#if defined(CONFIG_BT_CTLR_SYNC_ISO)
	struct lll_sync_iso_stream *stream;
	uint16_t stream_handle;

	if (handle < BT_CTLR_SYNC_ISO_STREAM_HANDLE_BASE) {
		return BT_HCI_ERR_CMD_DISALLOWED;
	}
	stream_handle = handle - BT_CTLR_SYNC_ISO_STREAM_HANDLE_BASE;

	stream = ull_sync_iso_stream_get(stream_handle);
	if (!stream || stream->dp) {
		return BT_HCI_ERR_CMD_DISALLOWED;
	}
#endif /* CONFIG_BT_CTLR_CONN_ISO */

	/* Allocate and configure datapath */
	struct ll_iso_datapath *dp = mem_acquire(&datapath_free);
	if (!dp) {
		return BT_HCI_ERR_CMD_DISALLOWED;
	}

	dp->path_dir = path_dir;
	dp->path_id = path_id;
	dp->coding_format = coding_format;
	dp->company_id = company_id;

	/* TODO dp->sync_delay    = controller_delay; ?*/

	flush_timeout = 0;
	stream_sync_delay = 0;
	group_sync_delay = 0;

#if defined(CONFIG_BT_CTLR_CONN_ISO)
	role = cig->lll.role;
	iso_interval = cig->iso_interval;
	stream_sync_delay = cis->sync_delay;
	group_sync_delay = cig->sync_delay;

	if (path_dir == BT_HCI_DATAPATH_DIR_CTLR_TO_HOST) {
		/* Create sink for RX data path */
		burst_number  = cis->lll.rx.burst_number;
		flush_timeout = cis->lll.rx.flush_timeout;
		max_octets    = cis->lll.rx.max_octets;

		if (role) {
			/* peripheral */
			sdu_interval = cig->c_sdu_interval;
		} else {
			/* central */
			sdu_interval = cig->p_sdu_interval;
		}

		cis->hdr.datapath_out = dp;

		if (path_id == BT_HCI_DATAPATH_ID_HCI) {
			/* Not vendor specific, thus alloc and emit functions known */
			err = isoal_sink_create(handle, role,
						burst_number, flush_timeout,
						sdu_interval, iso_interval,
						stream_sync_delay, group_sync_delay,
						sink_sdu_alloc_hci, sink_sdu_emit_hci,
						sink_sdu_write_hci, &sink_handle);
		} else {
			/* Set up vendor specific data path */
			isoal_sink_sdu_alloc_cb sdu_alloc;
			isoal_sink_sdu_emit_cb  sdu_emit;
			isoal_sink_sdu_write_cb sdu_write;

			/* Request vendor sink callbacks for path */
			if (ll_data_path_sink_create(dp, &sdu_alloc, &sdu_emit, &sdu_write)) {
				err = isoal_sink_create(handle, role,
							burst_number, flush_timeout,
							sdu_interval, iso_interval,
							stream_sync_delay, group_sync_delay,
							sdu_alloc, sdu_emit, sdu_write,
							&sink_handle);
			} else {
				return BT_HCI_ERR_CMD_DISALLOWED;
			}
		}

		if (!err) {
			dp->sink_hdl = sink_handle;
			isoal_sink_enable(sink_handle);
		} else {
			return BT_HCI_ERR_CMD_DISALLOWED;
		}
	} else  {
		burst_number  = cis->lll.tx.burst_number;
		flush_timeout = cis->lll.tx.flush_timeout;
		max_octets    = cis->lll.tx.max_octets;

		if (role) {
			/* peripheral */
			sdu_interval = cig->p_sdu_interval;
		} else {
			/* central */
			sdu_interval = cig->c_sdu_interval;
		}

		cis->hdr.datapath_in = dp;

		/* Create source for TX data path */
		isoal_source_pdu_alloc_cb   pdu_alloc;
		isoal_source_pdu_write_cb   pdu_write;
		isoal_source_pdu_release_cb pdu_release;

		/* Set default callbacks assuming not vendor specific
		 * or that the vendor specific path is the same.
		 */
		pdu_alloc   = ll_iso_pdu_alloc;
		pdu_write   = ll_iso_pdu_write;
		pdu_release = ll_iso_pdu_release;

		if (path_is_vendor_specific(path_id)) {
			if (!ll_data_path_source_create(dp, &pdu_alloc, &pdu_write,
							&pdu_release)) {
				return BT_HCI_ERR_CMD_DISALLOWED;
			}
		}

		err = isoal_source_create(handle, role,
					  burst_number, flush_timeout, max_octets,
					  sdu_interval, iso_interval,
					  stream_sync_delay, group_sync_delay,
					  pdu_alloc, pdu_write, pdu_release,
					  &source_handle);

		if (!err) {
			dp->source_hdl = source_handle;
			isoal_source_enable(source_handle);
		} else {
			return BT_HCI_ERR_CMD_DISALLOWED;
		}
	}
#endif /* CONFIG_BT_CTLR_CONN_ISO */

#if defined(CONFIG_BT_CTLR_SYNC_ISO)
	struct ll_sync_iso_set *sync_iso;
	struct lll_sync_iso *lll_iso;

	sync_iso = ull_sync_iso_by_stream_get(stream_handle);
	lll_iso = &sync_iso->lll;

	role = 1U; /* FIXME: Set role from LLL struct */
	burst_number = lll_iso->bn;
	sdu_interval = lll_iso->sdu_interval;
	iso_interval = lll_iso->iso_interval;
#endif /* CONFIG_BT_CTLR_SYNC_ISO */

	return 0;
}

uint8_t ll_remove_iso_path(uint16_t handle, uint8_t path_dir)
{
	struct ll_iso_datapath *dp;

#if defined(CONFIG_BT_CTLR_CONN_ISO)
	struct ll_conn_iso_stream *cis = NULL;
	struct ll_iso_stream_hdr *hdr = NULL;

	if (IS_CIS_HANDLE(handle)) {
		cis = ll_conn_iso_stream_get(handle);
		hdr = &cis->hdr;
	}

	/* If the Host issues this command with a Connection_Handle that does not exist
	 * or is not for a CIS or a BIS, the Controller shall return the error code Unknown
	 * Connection Identifier (0x02).
	 */
	if (!cis) {
		return BT_HCI_ERR_UNKNOWN_CONN_ID;
	}

	if (path_dir == BT_HCI_DATAPATH_DIR_HOST_TO_CTLR) {
		dp = hdr->datapath_in;
		if (dp) {
			isoal_source_destroy(dp->source_hdl);

			hdr->datapath_in = NULL;
			ull_iso_datapath_release(dp);
		}
	} else if (path_dir == BT_HCI_DATAPATH_DIR_CTLR_TO_HOST) {
		dp = hdr->datapath_out;
		if (dp) {
			isoal_sink_destroy(dp->sink_hdl);

			hdr->datapath_out = NULL;
			ull_iso_datapath_release(dp);
		}
	} else {
		/* Reserved for future use */
		return BT_HCI_ERR_CMD_DISALLOWED;
	}
#endif /* CONFIG_BT_CTLR_CONN_ISO */

#if defined(CONFIG_BT_CTLR_SYNC_ISO)
	struct lll_sync_iso_stream *stream;
	uint16_t stream_handle;

	if (path_dir != BT_HCI_DATAPATH_DIR_CTLR_TO_HOST) {
		return BT_HCI_ERR_CMD_DISALLOWED;
	}

	if (handle < BT_CTLR_SYNC_ISO_STREAM_HANDLE_BASE) {
		return BT_HCI_ERR_CMD_DISALLOWED;
	}
	stream_handle = handle - BT_CTLR_SYNC_ISO_STREAM_HANDLE_BASE;

	stream = ull_sync_iso_stream_get(stream_handle);
	if (!stream) {
		return BT_HCI_ERR_CMD_DISALLOWED;
	}

	dp = stream->dp;
	if (dp) {
		isoal_sink_destroy(dp->sink_hdl);

		stream->dp = NULL;
		ull_iso_datapath_release(dp);
	}
#endif /* CONFIG_BT_CTLR_SYNC_ISO */

	if (!dp) {
		/* Datapath was not previously set up */
		return BT_HCI_ERR_CMD_DISALLOWED;
	}

	return 0;
}

#if defined(CONFIG_BT_CTLR_SYNC_ISO) || defined(CONFIG_BT_CTLR_CONN_ISO)
uint8_t ll_iso_receive_test(uint16_t handle, uint8_t payload_type)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(payload_type);

	return BT_HCI_ERR_CMD_DISALLOWED;
}

uint8_t ll_iso_read_test_counters(uint16_t handle, uint32_t *received_cnt,
				  uint32_t *missed_cnt,
				  uint32_t *failed_cnt)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(received_cnt);
	ARG_UNUSED(missed_cnt);
	ARG_UNUSED(failed_cnt);

	return BT_HCI_ERR_CMD_DISALLOWED;
}

#if defined(CONFIG_BT_CTLR_READ_ISO_LINK_QUALITY)
uint8_t ll_read_iso_link_quality(uint16_t  handle,
				 uint32_t *tx_unacked_packets,
				 uint32_t *tx_flushed_packets,
				 uint32_t *tx_last_subevent_packets,
				 uint32_t *retransmitted_packets,
				 uint32_t *crc_error_packets,
				 uint32_t *rx_unreceived_packets,
				 uint32_t *duplicate_packets)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(tx_unacked_packets);
	ARG_UNUSED(tx_flushed_packets);
	ARG_UNUSED(tx_last_subevent_packets);
	ARG_UNUSED(retransmitted_packets);
	ARG_UNUSED(crc_error_packets);
	ARG_UNUSED(rx_unreceived_packets);
	ARG_UNUSED(duplicate_packets);

	return BT_HCI_ERR_CMD_DISALLOWED;
}
#endif /* CONFIG_BT_CTLR_READ_ISO_LINK_QUALITY */

#endif /* CONFIG_BT_CTLR_SYNC_ISO || CONFIG_BT_CTLR_CONN_ISO */

#if defined(CONFIG_BT_CTLR_ADV_ISO) || defined(CONFIG_BT_CTLR_CONN_ISO)
uint8_t ll_iso_transmit_test(uint16_t handle, uint8_t payload_type)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(payload_type);

	return BT_HCI_ERR_CMD_DISALLOWED;
}
#endif /* CONFIG_BT_CTLR_ADV_ISO || CONFIG_BT_CTLR_CONN_ISO */

uint8_t ll_iso_test_end(uint16_t handle, uint32_t *received_cnt,
			uint32_t *missed_cnt, uint32_t *failed_cnt)
{
	ARG_UNUSED(handle);
	ARG_UNUSED(received_cnt);
	ARG_UNUSED(missed_cnt);
	ARG_UNUSED(failed_cnt);

	return BT_HCI_ERR_CMD_DISALLOWED;
}

#if defined(CONFIG_BT_CTLR_ADV_ISO) || defined(CONFIG_BT_CTLR_CONN_ISO)
void *ll_iso_tx_mem_acquire(void)
{
	return mem_acquire(&mem_iso_tx.free);
}

void ll_iso_tx_mem_release(void *node_tx)
{
	mem_release(node_tx, &mem_iso_tx.free);
}
#endif /* CONFIG_BT_CTLR_ADV_ISO || CONFIG_BT_CTLR_CONN_ISO */

int ull_iso_init(void)
{
	int err;

	err = init_reset();
	if (err) {
		return err;
	}

	return 0;
}

int ull_iso_reset(void)
{
	int err;

#if defined(CONFIG_BT_CTLR_ADV_ISO) || defined(CONFIG_BT_CTLR_CONN_ISO)
	/* Re-initialize the Tx mfifo */
	MFIFO_INIT(iso_tx);
	/* Re-initialize the Tx Ack mfifo */
	MFIFO_INIT(iso_tx_ack);
#endif /* CONFIG_BT_CTLR_ADV_ISO || CONFIG_BT_CTLR_CONN_ISO */

	err = init_reset();
	if (err) {
		return err;
	}

	return 0;
}

#if defined(CONFIG_BT_CTLR_SYNC_ISO) || defined(CONFIG_BT_CTLR_CONN_ISO)
void *ull_iso_pdu_rx_alloc_peek(uint8_t count)
{
	if (count > MFIFO_AVAIL_COUNT_GET(iso_rx)) {
		return NULL;
	}

	return MFIFO_DEQUEUE_PEEK(iso_rx);
}

void *ull_iso_pdu_rx_alloc(void)
{
	return MFIFO_DEQUEUE(iso_rx);
}

#if defined(CONFIG_BT_CTLR_ISO_VENDOR_DATA_PATH)
void ull_iso_rx_put(memq_link_t *link, void *rx)
{
	/* Enqueue the Rx object */
	memq_enqueue(link, rx, &memq_ull_iso_rx.tail);
}

void ull_iso_rx_sched(void)
{
	static memq_link_t link;
	static struct mayfly mfy = {0, 0, &link, NULL, iso_rx_demux};

	/* Kick the ULL (using the mayfly, tailchain it) */
	mayfly_enqueue(TICKER_USER_ID_LLL, TICKER_USER_ID_ULL_HIGH, 1, &mfy);
}

void *ull_iso_tx_ack_dequeue(void)
{
	return MFIFO_DEQUEUE(iso_tx_ack);
}

static  memq_link_t *ull_iso_ack_peek(uint16_t *handle, struct node_tx_iso **tx)
{
	struct lll_tx *lll_tx;

	lll_tx = MFIFO_DEQUEUE_GET(iso_tx_ack);
	if (!lll_tx) {
		return NULL;
	}

	*handle = lll_tx->handle;
	*tx = lll_tx->node;

	return (*tx)->link;
}

void ull_iso_lll_ack_enqueue(uint16_t handle, struct node_tx_iso *tx)
{
	struct lll_tx *lll_tx;
	uint8_t idx;

	idx = MFIFO_ENQUEUE_GET(iso_tx_ack, (void **)&lll_tx);
	LL_ASSERT(lll_tx);

	lll_tx->handle = handle;
	lll_tx->node = tx;

	MFIFO_ENQUEUE(iso_tx_ack, idx);
}

static void iso_rx_demux_tx_ack(void)
{
	struct node_tx_iso *node_tx;
	struct ll_conn_iso_stream *cis;
	struct ll_iso_datapath *dp;
	memq_link_t *link;
	uint16_t handle;

	link = ull_iso_ack_peek(&handle, &node_tx);
	while (link) {
		cis = ll_conn_iso_stream_get(handle);
		dp  = cis->hdr.datapath_in;
		if (dp) {
			isoal_tx_pdu_release(dp->source_hdl, node_tx);
		}

		/* Release link mem */
		ull_iso_link_tx_release(link);

		/* Dequeue node */
		ull_iso_tx_ack_dequeue();

		link = ull_iso_ack_peek(&handle, &node_tx);
	};

	/* Trigger thread to call ll_rx_get() */
	ll_rx_sched();
}

static void iso_rx_demux(void *param)
{
	struct ll_conn_iso_stream *cis;
	struct ll_iso_datapath *dp;
	struct node_rx_pdu *rx_pdu;
	struct node_rx_hdr *rx;
	memq_link_t *link;

	do {
		link = memq_peek(memq_ull_iso_rx.head, memq_ull_iso_rx.tail,
				 (void **)&rx);
		if (link) {
			/* Demux Rx objects */
			switch (rx->type) {
			case NODE_RX_TYPE_RELEASE:
				(void)memq_dequeue(memq_ull_iso_rx.tail,
						   &memq_ull_iso_rx.head, NULL);
				ll_iso_rx_put(link, rx);
				ll_rx_sched();
				break;

			case NODE_RX_TYPE_ISO_PDU:
				/* Remove from receive-queue; ULL has received this now */
				(void)memq_dequeue(memq_ull_iso_rx.tail, &memq_ull_iso_rx.head,
						   NULL);

#if defined(CONFIG_BT_CTLR_CONN_ISO)
				rx_pdu = (struct node_rx_pdu *)rx;
				cis = ll_conn_iso_stream_get(rx_pdu->hdr.handle);
				dp  = cis->hdr.datapath_out;

				if (dp && dp->path_id != BT_HCI_DATAPATH_ID_HCI) {
					/* If vendor specific datapath pass to ISO AL here,
					 * in case of HCI destination it will be passed in
					 * HCI context.
					 */
					struct isoal_pdu_rx pckt_meta = {
						.meta = &rx_pdu->hdr.rx_iso_meta,
						.pdu  = (struct pdu_iso *)&rx_pdu->pdu[0]
					};

					/* Pass the ISO PDU through ISO-AL */
					const isoal_status_t err =
						isoal_rx_pdu_recombine(dp->sink_hdl, &pckt_meta);

					LL_ASSERT(err == ISOAL_STATUS_OK); /* TODO handle err */
				}
#endif /* CONFIG_BT_CTLR_CONN_ISO */

				/* Let ISO PDU start its long journey upwards */
				ll_iso_rx_put(link, rx);
				ll_rx_sched();
				break;

			default:
				LL_ASSERT(0);
				break;
			}
		}

		/* Process TX acks */
		iso_rx_demux_tx_ack();
	} while (link);
}
#endif /* CONFIG_BT_CTLR_ISO_VENDOR_DATA_PATH */

void ll_iso_rx_put(memq_link_t *link, void *rx)
{
	/* Enqueue the Rx object */
	memq_enqueue(link, rx, &memq_ll_iso_rx.tail);
}

void *ll_iso_rx_get(void)
{
	struct node_rx_hdr *rx;
	memq_link_t *link;

	link = memq_peek(memq_ll_iso_rx.head, memq_ll_iso_rx.tail, (void **)&rx);
	while (link) {
		/* Do not send up buffers to Host thread that are
		 * marked for release
		 */
		if (rx->type == NODE_RX_TYPE_RELEASE) {
			(void)memq_dequeue(memq_ll_iso_rx.tail,
						&memq_ll_iso_rx.head, NULL);
			mem_release(link, &mem_link_iso_rx.free);
			mem_release(rx, &mem_iso_rx.free);
			RXFIFO_ALLOC(iso_rx, 1);

			link = memq_peek(memq_ll_iso_rx.head, memq_ll_iso_rx.tail, (void **)&rx);
			continue;
		}
		return rx;
	}

	return NULL;
}

void ll_iso_rx_dequeue(void)
{
	struct node_rx_hdr *rx = NULL;
	memq_link_t *link;

	link = memq_dequeue(memq_ll_iso_rx.tail, &memq_ll_iso_rx.head,
			    (void **)&rx);
	LL_ASSERT(link);

	mem_release(link, &mem_link_iso_rx.free);

	/* Handle object specific clean up */
	switch (rx->type) {
	case NODE_RX_TYPE_ISO_PDU:
		break;
	default:
		LL_ASSERT(0);
		break;
	}
}

void ll_iso_rx_mem_release(void **node_rx)
{
	struct node_rx_hdr *rx;

	rx = *node_rx;
	while (rx) {
		struct node_rx_hdr *rx_free;

		rx_free = rx;
		rx = rx->next;

		switch (rx_free->type) {
		case NODE_RX_TYPE_ISO_PDU:
			mem_release(rx_free, &mem_iso_rx.free);
			break;
		default:
			/* Ignore other types as node may have been initialized due to
			 * race with HCI reset.
			 */
			break;
		}
	}

	*node_rx = rx;

	RXFIFO_ALLOC(iso_rx, UINT8_MAX);
}
#endif /* CONFIG_BT_CTLR_SYNC_ISO) || CONFIG_BT_CTLR_CONN_ISO */

void ull_iso_datapath_release(struct ll_iso_datapath *dp)
{
	mem_release(dp, &datapath_free);
}

#if defined (CONFIG_BT_CTLR_ADV_ISO) || defined(CONFIG_BT_CTLR_CONN_ISO)
static void ull_iso_link_tx_release(memq_link_t *link)
{
	mem_release(link, &mem_link_iso_tx.free);
}

/**
 * @brief De-multiplex a number of waiting TX nodes
 *
 * De-multiplex a number of TX nodes in the shared FIFO to the linked node 
 * lists of individual CISes.
 * 
 * @param count Number of TX nodes to demux
 */
static void ull_iso_tx_demux(uint8_t count)
{
	do {
		struct ll_conn_iso_stream *cis;
		struct lll_tx *lll_tx;

		lll_tx = MFIFO_DEQUEUE_GET(iso_tx);
		if (!lll_tx) {
			break;
		}

		cis = ll_iso_stream_connected_get(lll_tx->handle);
		if (cis) {
			struct node_tx *tx = lll_tx->node;

			tx->next = NULL;
			if (!cis->tx_head) {
				cis->tx_head = tx;
				cis->tx_tail = NULL;
			}

			if (cis->tx_tail) {
				cis->tx_tail->next = tx;
			}

			cis->tx_tail = tx;
		} else {
			/* Not a valid handle - create instant ACK with error */
			struct node_tx *tx = lll_tx->node;
			struct pdu_data *pdu = (void *)tx->pdu;

			pdu->ll_id = PDU_DATA_LLID_RESV;
			ll_tx_ack_put(LLL_HANDLE_INVALID, tx);
		}

		MFIFO_DEQUEUE(iso_tx);
	} while (--count);
}

/**
 * @brief Encode a number of TX nodes to LLL
 * 
 * @param cis   Connected ISO stream
 * @param count Max. number of nodes to encode
 */
static void ull_iso_tx_lll_enqueue(struct ll_conn_iso_stream *cis, uint8_t count)
{
	while (cis->tx_head && count--) {
		struct pdu_data *pdu_tx;
		struct node_tx *tx;
		memq_link_t *link;

		tx = cis->tx_head;
		cis->tx_head = cis->tx_head->next;

		pdu_tx = (void *)tx->pdu;

		link = mem_acquire(&mem_link_iso_tx.free);
		LL_ASSERT(link);

		memq_enqueue(link, tx, &cis->lll.memq_tx.tail);
	}
}

static void iso_tx_demux(void *param)
{
	ull_iso_tx_demux(1);
	ull_iso_tx_lll_enqueue(param, 1);
}

int ll_iso_tx_mem_enqueue(uint16_t handle, void *tx)
{
	static memq_link_t link;
	static struct mayfly mfy = {0, 0, &link, NULL, iso_tx_demux};

	struct ll_conn_iso_stream *cis;
	struct lll_tx *lll_tx;
	uint8_t idx;

	if (!IS_CIS_HANDLE(handle)) {
		/* FIXME: We only support CIS for now */
		return -EINVAL;
	}

	cis = ll_iso_stream_connected_get(handle);
	if (!cis) {
		return -EINVAL;
	}

	idx = MFIFO_ENQUEUE_GET(iso_tx, (void **) &lll_tx);
	if (!lll_tx) {
		return -ENOBUFS;
	}

	lll_tx->handle = handle;
	lll_tx->node = tx;


	MFIFO_ENQUEUE(iso_tx, idx);

	mfy.param = cis;

	mayfly_enqueue(TICKER_USER_ID_THREAD, TICKER_USER_ID_ULL_HIGH,
		       0, &mfy);

	return 0;
}
#endif /* CONFIG_BT_CTLR_ADV_ISO || CONFIG_BT_CTLR_CONN_ISO */

#if defined(CONFIG_BT_CTLR_ISO)
/**
 * Allocate a PDU from the LL and store the details in the given buffer. Allocation
 * is not expected to fail as there must always be sufficient PDU buffers. Any
 * failure will trigger the assert.
 * @param[in]  pdu_buffer Buffer to store PDU details in
 * @return     Error status of operation
 */
static isoal_status_t ll_iso_pdu_alloc(struct isoal_pdu_buffer *pdu_buffer)
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
	pdu_buffer->size = ISO_TX_BUFFER_SIZE;

	return ISOAL_STATUS_OK;
}

/**
 * Write the given SDU payload to the target PDU buffer at the given offset.
 * @param[in,out]  pdu_buffer  Target PDU buffer
 * @param[in]      pdu_offset  Offset / current write position within PDU
 * @param[in]      sdu_payload Location of source data
 * @param[in]      consume_len Length of data to copy
 * @return         Error status of write operation
 */
static isoal_status_t ll_iso_pdu_write(struct isoal_pdu_buffer *pdu_buffer,
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

/**
 * Release the given payload back to the memory pool.
 * @param node_tx TX node to release or forward
 * @param handle  CIS/BIS handle
 * @param status  Reason for release
 * @return        Error status of write operation
 */
static isoal_status_t ll_iso_pdu_release(struct node_tx_iso *node_tx,
					 const uint16_t handle,
					 const isoal_status_t status)
{
	if (status == ISOAL_STATUS_OK) {
		/* Process as TX ack */
		ll_tx_ack_put(handle, (void *)node_tx);
	} else {
		/* Release back to memory pool */
		ll_iso_tx_mem_release(node_tx);
	}

	return ISOAL_STATUS_OK;
}
#endif /* CONFIG_BT_CTLR_ISO */

static int init_reset(void)
{
#if defined(CONFIG_BT_CTLR_SYNC_ISO) || defined(CONFIG_BT_CTLR_CONN_ISO)
	memq_link_t *link;

	RXFIFO_INIT(iso_rx);

	/* Acquire a link to initialize ull rx memq */
	link = mem_acquire(&mem_link_iso_rx.free);
	LL_ASSERT(link);

#if defined(CONFIG_BT_CTLR_ISO_VENDOR_DATA_PATH)
	/* Initialize ull rx memq */
	MEMQ_INIT(ull_iso_rx, link);
#endif /* CONFIG_BT_CTLR_ISO_VENDOR_DATA_PATH */

	/* Acquire a link to initialize ll_iso_rx memq */
	link = mem_acquire(&mem_link_iso_rx.free);
	LL_ASSERT(link);

	/* Initialize ll_iso_rx memq */
	MEMQ_INIT(ll_iso_rx, link);

	RXFIFO_ALLOC(iso_rx, UINT8_MAX);
#endif /* CONFIG_BT_CTLR_SYNC_ISO) || CONFIG_BT_CTLR_CONN_ISO */

#if defined(CONFIG_BT_CTLR_ADV_ISO) || defined(CONFIG_BT_CTLR_CONN_ISO)
	/* Initialize tx pool. */
	mem_init(mem_iso_tx.pool, NODE_TX_BUFFER_SIZE,
		 CONFIG_BT_CTLR_ISO_TX_BUFFERS, &mem_iso_tx.free);

	/* Initialize tx link pool. */
	mem_init(mem_link_iso_tx.pool, sizeof(memq_link_t),
		 CONFIG_BT_CTLR_ISO_TX_BUFFERS + CONFIG_BT_CTLR_ISO_TX_VENDOR_BUFFERS,
		 &mem_link_iso_tx.free);
#endif /* CONFIG_BT_CTLR_ADV_ISO || CONFIG_BT_CTLR_CONN_ISO */

#if BT_CTLR_ISO_STREAMS
	/* Initialize ISO Datapath pool */
	mem_init(datapath_pool, sizeof(struct ll_iso_datapath),
		 sizeof(datapath_pool) / sizeof(struct ll_iso_datapath), &datapath_free);
#endif

	return 0;
}
