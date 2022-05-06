/*
 * Copyright (c) 2020 Demant
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 *  Run this test from zephyr directory as:
 *
 *     ./scripts/twister --coverage -p native_posix -v -T tests/bluetooth/ctrl_isoal/
 *
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/types.h>
#include <ztest.h>
#include <ztest_error_hook.h>

#include <sys/types.h>
#include <toolchain.h>
#include <sys/util.h>

#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/hci.h>

#include "util/memq.h"
#include "pdu.h"

#include "ll.h"
#include "lll.h"
#include "lll_conn_iso.h"
#include "lll_iso_tx.h"
#include "isoal.h"
#include "ull_iso_types.h"

#include "isoal_test_common.h"
#include "isoal_test_debug.h"


#if defined(DEBUG_TEST)
/**
 * Print contents of a RX PDU
 * @param[in] pdu_meta Pointer to PDU structure including meta information
 */
void isoal_test_debug_print_rx_pdu(struct isoal_pdu_rx *pdu_meta)
{
	zassert_not_null(pdu_meta, "");

	PRINT("PDU %04u (%10u) | %12s [%10s] %03u: ",
		(uint32_t) pdu_meta->meta->payload_number,
		(uint32_t) pdu_meta->meta->timestamp,
		LLID_TO_STR(pdu_meta->pdu->ll_id),
		DU_ERR_TO_STR(pdu_meta->meta->status),
		pdu_meta->pdu->length);

	for (int i = 0; i < pdu_meta->pdu->length; i++) {
		PRINT("%02x ", pdu_meta->pdu->payload[i]);
	}
	PRINT("\n");
	PRINT("\n");
}

/**
 * Print contents of RX SDU
 * @param[in] sink_ctx Sink context provided when SDU is emitted
 * @param[in] buf      SDU data buffer pointer
 */
void isoal_test_debug_print_rx_sdu(const struct isoal_sink *sink_ctx, uint8_t *buf)
{
	zassert_not_null(sink_ctx, "");
	zassert_not_null(buf, "");

	uint16_t len = sink_ctx->sdu_production.sdu_written;

	PRINT("\n");
	PRINT("SDU %04d (%10d) | %12s [%10s] %03d: ",
		sink_ctx->sdu_production.sdu.seqn,
		sink_ctx->sdu_production.sdu.timestamp,
		STATE_TO_STR(sink_ctx->sdu_production.sdu_state),
		DU_ERR_TO_STR(sink_ctx->sdu_production.sdu.status),
		len);
	for (int i = 0; i < len; i++) {
		PRINT("%02x ", buf[i]);
	}
	PRINT("\n");
	PRINT("\n");
}

/**
 * Print contents of a TX PDU
 * @param[in] node_tx Pointer to PDU structure including meta information
 */
void isoal_test_debug_print_tx_pdu(struct node_tx_iso *node_tx)
{
	struct pdu_iso *pdu;
	uint8_t seg_length;

	zassert_not_null(node_tx, "");

	pdu = (struct pdu_iso *)node_tx->pdu;
	seg_length = 0;

	zassert_not_null(pdu, "");

	PRINT("\n");
	PRINT("PDU %04u (    %02u    ) | %12s | %03u: ",
		(uint32_t) node_tx->payload_number,
		 node_tx->sdu_fragments,
		LLID_TO_STR(pdu->ll_id),
		pdu->length);

	for (int i = 0; i < pdu->length; i++) {
		if (seg_length == 0 && pdu->ll_id == PDU_BIS_LLID_FRAMED) {
			seg_length = pdu->payload[i+1];
			PRINT("[%s %s %03u]",
				pdu->payload[i] & BIT(0) ? "C" : "S",
				pdu->payload[i] & BIT(1) ? "C" : "-",
				pdu->payload[i+1]);
			if ((pdu->payload[i] & BIT(0)) == 0) {
				PRINT("(%8uus)",
					((uint32_t)pdu->payload[i+2] +
					((uint32_t)pdu->payload[i+3] << 8) +
					((uint32_t)pdu->payload[i+4] << 16)));
			}
			PRINT(" / ");
			PRINT("[%02x %02x]",
				pdu->payload[i],
				pdu->payload[i+1]);
			if ((pdu->payload[i] & BIT(0)) == 0) {
				PRINT("(%02x %02x %02x)",
					(uint32_t)pdu->payload[i+4],
					(uint32_t)pdu->payload[i+3],
					(uint32_t)pdu->payload[i+2]);
			}
			PRINT(" : ");
			seg_length -= pdu->payload[i] & BIT(0) ? 0 : PDU_ISO_SEG_TIMEOFFSET_SIZE;
			i = i + PDU_ISO_SEG_HDR_SIZE +
				(pdu->payload[i] & BIT(0) ? 0 : PDU_ISO_SEG_TIMEOFFSET_SIZE);
		}
		PRINT("%02x ", pdu->payload[i]);
		seg_length--;
		if (seg_length == 0 && pdu->ll_id == PDU_BIS_LLID_FRAMED) {
			PRINT("\n%44s", "");
		}
	}
	PRINT("\n");
	PRINT("\n");
}

/**
 * Print contents of TX SDU
 * @param[in] sink_ctx Sink context provided when SDU is emitted
 * @param[in] buf      SDU data buffer pointer
 */
void isoal_test_debug_print_tx_sdu(struct isoal_sdu_tx *tx_sdu)
{
	uint8_t *buf;
	uint16_t len;

	zassert_not_null(tx_sdu, "");

	buf = tx_sdu->dbuf;
	len = tx_sdu->size;

	PRINT("\n");
	PRINT("SDU %04u (%10u) | %12s | %03u: ",
		tx_sdu->packet_sn,
		tx_sdu->time_stamp,
		STATE_TO_STR(tx_sdu->sdu_state),
		len);
	for (int i = 0; i < len; i++) {
		PRINT("%02x ", buf[i]);
	}
	PRINT("\n");
	PRINT("    Ref. <%10u>\n", tx_sdu->cig_ref_point);
	PRINT("   Event <%10u>\n", (uint32_t)tx_sdu->target_event);
	PRINT("\n");
}
#else /* DEBUG_TEST */
void isoal_test_debug_print_rx_pdu(struct isoal_pdu_rx *pdu_meta) {}
void isoal_test_debug_print_rx_sdu(const struct isoal_sink *sink_ctx, uint8_t *buf) {}
void isoal_test_debug_print_tx_pdu(struct node_tx_iso *node_tx) {}
void isoal_test_debug_print_tx_sdu(struct isoal_sdu_tx *tx_sdu) {}
#endif /* DEBUG_TEST */

#if defined(DEBUG_TRACE)
/**
 * Print function trace
 * @param func   Function name
 * @param status Status
 */
void isoal_test_debug_trace_func_call(const uint8_t *func, const uint8_t *status)
{
	PRINT("%s :: %s\n", func, status);
}
#else /* DEBUG_TRACE */
void isoal_test_debug_trace_func_call(const uint8_t *func, const uint8_t *status) {}
#endif /* DEBUG_TRACE */
