/*
 * Copyright (c) 2020 Demant
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <ztest.h>
#include "kconfig.h"

#include <bluetooth/hci.h>
#include <sys/byteorder.h>
#include <sys/slist.h>
#include <sys/util.h>
#include "hal/ccm.h"

#include "util/util.h"
#include "util/mem.h"
#include "util/memq.h"
#include "util/dbuf.h"

#include "pdu.h"
#include "ll.h"
#include "ll_settings.h"

#include "lll.h"
#include "lll_df_types.h"
#include "lll_conn.h"
#include "lll_conn_iso.h"

#include "ull_tx_queue.h"

#include "isoal.h"
#include "ull_iso_types.h"
#include "ull_conn_iso_types.h"
#include "ull_conn_types.h"
#include "ull_llcp.h"
#include "ull_conn_internal.h"
#include "ull_llcp_internal.h"

#include "helper_pdu.h"
#include "helper_util.h"

struct ll_conn conn;

static void setup(void)
{
	test_setup(&conn);
}

static void test_cis_terminate_rem(uint8_t role)
{
	struct pdu_data_llctrl_cis_terminate_ind remote_cis_terminate_ind = {
		.cig_id = 0x01,
		.cis_id = 0x02,
		.error_code = 0x05,
	};

	/* Role */
	test_set_role(&conn, role);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx */
	lt_tx(LL_CIS_TERMINATE_IND, &conn, &remote_cis_terminate_ind);

	/* Done */
	event_done(&conn);

	/* There should be no host notification */
	ut_rx_q_is_empty();

	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

void test_cis_terminate_mas_rem(void)
{
	test_cis_terminate_rem(BT_HCI_ROLE_CENTRAL);
}

void test_cis_terminate_sla_rem(void)
{
	test_cis_terminate_rem(BT_HCI_ROLE_PERIPHERAL);
}

void test_cis_terminate_loc(uint8_t role)
{
	uint8_t err;
	struct node_tx *tx;
	struct ll_conn_iso_stream cis = { 0 };
	struct ll_conn_iso_group group = { 0 };

	struct pdu_data_llctrl_cis_terminate_ind local_cis_terminate_ind = {
		.cig_id = 0x03,
		.cis_id = 0x04,
		.error_code = 0x06,
	};

	/* Role */
	test_set_role(&conn, role);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Mock CIS/ACL */
	cis.lll.acl_handle = conn.lll.handle;
	group.cig_id = 0x03;
	cis.cis_id = 0x04;
	cis.group = &group;

	/* Initiate an LE Ping Procedure */
	err = ull_cp_cis_terminate(&conn, &cis, 0x06);
	zassert_equal(err, BT_HCI_ERR_SUCCESS, NULL);

	/* Prepare */
	event_prepare(&conn);

	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* 'Signal' CIS terminated */
	conn.llcp.cis.terminate_ack = 1;

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should now have one LL Control PDU */
	lt_rx(LL_CIS_TERMINATE_IND, &conn, &tx, &local_cis_terminate_ind);
	lt_rx_q_is_empty(&conn);

	/* RX Ack */
	event_tx_ack(&conn, tx);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* There should be no host notification */
	ut_rx_q_is_empty();

	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

void test_cis_terminate_mas_loc(void)
{
	test_cis_terminate_loc(BT_HCI_ROLE_CENTRAL);
}

void test_cis_terminate_sla_loc(void)
{
	test_cis_terminate_loc(BT_HCI_ROLE_PERIPHERAL);
}

void test_main(void)
{
	ztest_test_suite(
		cis_term,
		ztest_unit_test_setup_teardown(test_cis_terminate_mas_rem, setup, unit_test_noop),
		ztest_unit_test_setup_teardown(test_cis_terminate_sla_rem, setup, unit_test_noop),
		ztest_unit_test_setup_teardown(test_cis_terminate_mas_loc, setup, unit_test_noop),
		ztest_unit_test_setup_teardown(test_cis_terminate_sla_loc, setup, unit_test_noop));

	ztest_run_test_suite(cis_term);
}
