/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>

#include "host/hci_core.h"
#include "utils.h"
#include "bstests.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dut_uatt, LOG_LEVEL_DBG);

DEFINE_FLAG(is_connected);
DEFINE_FLAG(flag_data_length_updated);
DEFINE_FLAG(flag_mtu_exchanged);
DEFINE_FLAG(notified);
DEFINE_FLAG(written);

struct bt_conn *g_conn;

#define TEST_SERVICE_UUID BT_UUID_DECLARE_128( \
	0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, \
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12)

#define TEST_CHAR_UUID BT_UUID_DECLARE_128( \
	0xf2, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, \
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12)

BT_GATT_SERVICE_DEFINE(test_svc, BT_GATT_PRIMARY_SERVICE(TEST_SERVICE_UUID),
		       BT_GATT_CHARACTERISTIC(TEST_CHAR_UUID,
					      BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
					      BT_GATT_PERM_READ, NULL, NULL, NULL),
		       BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

static void notification_sent(struct bt_conn *conn, void *user_data)
{
	LOG_INF("Sent notification");
	SET_FLAG(notified);
}

/* 251 = LL data length
 * overhead: 2 att-handle 1 att-opcode 2 l2-len 2 l2-cid
 */
#define CHAR_LEN (CONFIG_BT_L2CAP_TX_MTU - 7)

static void write_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
	if (err != BT_ATT_ERR_SUCCESS) {
		/* TODO: assert */
		LOG_ERR("Write failed: 0x%02X", err);
	}

	SET_FLAG(written);
}

static void gatt_write(void)
{
	int err;
	static uint8_t data[CHAR_LEN];
	for (int i=0; i<sizeof(data); i++) data[i] = 0xea;
	data[sizeof(data) - 1] = 0xbb;

	LOG_ERR("write: %d", CHAR_LEN);

	static struct bt_gatt_write_params params = {
		.handle = 0x1337,
		.func = write_cb,
		.offset = 0,
		.data = data,
		.length = sizeof(data),
	};

#if defined(CONFIG_BT_EATT)
	/* Always send over UATT */
	params.chan_opt = BT_ATT_CHAN_OPT_UNENHANCED_ONLY;
#endif

	err = bt_gatt_write(g_conn, &params);
	ASSERT(!err, "couldn't notify: err %d\n", err);

	/* TODO: tester: implement RSP */
	/* WAIT_FOR_FLAG(written); */
}

static void notify(void)
{
	int err;
	static uint8_t data[CHAR_LEN];
	for (int i=0; i<sizeof(data); i++) data[i] = 0xea;
	data[sizeof(data) - 1] = 0xbb;

	LOG_ERR("notify: %d", CHAR_LEN);

	static struct bt_gatt_notify_params params = {
		.attr = &test_svc.attrs[1],
		.data = data,
		.len = CHAR_LEN,
		.func = notification_sent,
		.uuid = NULL,
	};

#if defined(CONFIG_BT_EATT)
	/* Always send over UATT */
	params.chan_opt = BT_ATT_CHAN_OPT_UNENHANCED_ONLY;
#endif

	err = bt_gatt_notify_cb(g_conn, &params);
	ASSERT(!err, "couldn't notify: err %d\n", err);

	WAIT_FOR_FLAG(notified);
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		FAIL("Failed to connect to %s (%u)", addr, conn_err);
		return;
	}

	LOG_DBG("%s", addr);

	g_conn = bt_conn_ref(conn);
	SET_FLAG(is_connected);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_conn_unref(g_conn);
	g_conn = NULL;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_DBG("%p %s (reason 0x%02x)", conn, addr, reason);

	UNSET_FLAG(is_connected);
}

static void data_len_updated(struct bt_conn *conn,
			     struct bt_conn_le_data_len_info *info)
{
	LOG_DBG("Data length updated: TX %d RX %d",
		info->tx_max_len,
		info->rx_max_len);
	SET_FLAG(flag_data_length_updated);
}

static void do_dlu(struct bt_conn *conn, void *data)
{
	int err;

	struct bt_conn_le_data_len_param param;

	param.tx_max_len = CONFIG_BT_CTLR_DATA_LENGTH_MAX;
	param.tx_max_time = 2500;

	err = bt_conn_le_data_len_update(conn, &param);
	ASSERT(err == 0, "Can't update data length (err %d)\n", err);

	WAIT_FOR_FLAG(flag_data_length_updated);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_data_len_updated = data_len_updated,
};

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	struct bt_le_conn_param *param;
	struct bt_conn *conn;
	int err;

	err = bt_le_scan_stop();
	if (err) {
		FAIL("Stop LE scan failed (err %d)", err);
		return;
	}

	char str[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, str, sizeof(str));

	LOG_DBG("Connecting to %s", str);

	param = BT_LE_CONN_PARAM_DEFAULT;
	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param, &conn);
	if (err) {
		FAIL("Create conn failed (err %d)", err);
		return;
	}
}

static void connect(void)
{
	struct bt_le_scan_param scan_param = {
		.type = BT_LE_SCAN_TYPE_ACTIVE,
		.options = BT_LE_SCAN_OPT_NONE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_WINDOW,
	};

	UNSET_FLAG(is_connected);

	int err = bt_le_scan_start(&scan_param, device_found);

	ASSERT(!err, "Scanning failed to start (err %d)\n", err);

	LOG_DBG("Central initiating connection...");
	WAIT_FOR_FLAG(is_connected);
}

static void disconnect_device(struct bt_conn *conn, void *data)
{
	int err;

	SET_FLAG(is_connected);

	err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	ASSERT(!err, "Failed to initate disconnect (err %d)", err);

	LOG_DBG("Waiting for disconnection...");
	WAIT_FOR_FLAG_UNSET(is_connected);
}

static void att_mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	LOG_DBG("tx %d rx %d", tx, rx);

	if (tx > 23) {
		SET_FLAG(flag_mtu_exchanged);
	}
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = att_mtu_updated,
};

void test_procedure_0(void)
{
	LOG_DBG("Procedure 0 started");
	int err;

	err = bt_enable(NULL);
	ASSERT(err == 0, "Can't enable Bluetooth (err %d)\n", err);
	LOG_DBG("Bluetooth initialized.");

	bt_gatt_cb_register(&gatt_callbacks);

	/* Goal:
	 * Test on-air PDU length for a given GATT notification
	 *
	 * Test procedure:
	 * - connect (dut is central for simplicity)
	 * - DLU to fit 100 byte notification
	 * - [DUT] send max length L2CAP notification
	 * - [tester] measure and store notification size
	 * - [DUT] disconnect & die
	 *
	 * - connect (dut2 is central)
	 * - same procedure
	 *
	 * - [tester] assert both notification sizes ==
	 */

	connect();

	bt_conn_foreach(BT_CONN_TYPE_LE, do_dlu, NULL);
	WAIT_FOR_FLAG(flag_mtu_exchanged);

	notify();
	k_msleep(1000);		/* allow other side to process it */
	gatt_write();
	k_msleep(1000);		/* allow other side to process it */

	LOG_INF("disconnecting");
	bt_conn_foreach(BT_CONN_TYPE_LE, disconnect_device, NULL);

	PASS("DUT done\n");
}

void test_tick(bs_time_t HW_device_time)
{
	bs_trace_debug_time(0, "Simulation ends now.\n");
	if (bst_result != Passed) {
		bst_result = Failed;
		bs_trace_error("Test did not pass before simulation ended.\n");
	}
}

void test_init(void)
{
	bst_ticker_set_next_tick_absolute(TEST_TIMEOUT_SIMULATED);
	bst_result = In_progress;
}

static const struct bst_test_instance test_to_add[] = {
	{
		.test_id = "test_0",
		.test_pre_init_f = test_init,
		.test_tick_f = test_tick,
		.test_main_f = test_procedure_0,
	},
	BSTEST_END_MARKER,
};

static struct bst_test_list *install(struct bst_test_list *tests)
{
	return bst_add_tests(tests, test_to_add);
};

bst_test_install_t test_installers[] = {install, NULL};

int main(void)
{
	bst_main();

	return 0;
}
