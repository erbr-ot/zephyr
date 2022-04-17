/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * Copyright (c) 2021 Demant
 *
 * SPDX-License-Identifier: Apache-2.0
 */

struct node_tx_iso {
	union {
		void        *next;
		memq_link_t *link;
	};

	uint64_t payload_number : 39; /* cisPayloadNumber */
	uint64_t sdu_fragments : 8;
	uint8_t  pdu[];
};
