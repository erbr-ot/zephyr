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
#include <zephyr/types.h>
#include <ztest.h>
#include <ztest_error_hook.h>

#include <stdio.h>
#include <stdlib.h>


/* Include the DUT */
#include "ll_sw/isoal.c"

#include "isoal_test_common.h"
#include "isoal_test_debug.h"

/* Include Test Subsets */
#include "isoal_test_rx.c"
#include "isoal_test_tx.c"


ZTEST_SUITE(test_rx_basics, NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(test_rx_unframed, NULL, NULL, NULL, NULL, NULL);
ZTEST_SUITE(test_rx_framed, NULL, NULL, NULL, NULL, NULL);
