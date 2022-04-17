/*
 * Copyright (c) 2017-2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 *  User CPR Interval
 */
#if !defined(CONFIG_BT_CTLR_USER_CPR_INTERVAL_MIN)
/* Bluetooth defined CPR Interval Minimum (7.5ms) */
#define CONN_INTERVAL_MIN(x) (6)
#else /* CONFIG_BT_CTLR_USER_CPR_INTERVAL_MIN */
/* Proprietary user defined CPR Interval Minimum */
struct ll_conn;
extern uint16_t ull_conn_interval_min_get(struct ll_conn *conn);
#define CONN_INTERVAL_MIN(x) (MAX(ull_conn_interval_min_get(x), 1))
#endif /* CONFIG_BT_CTLR_USER_CPR_INTERVAL_MIN */

/* Macro to convert time in us to connection interval units */
#define RADIO_CONN_EVENTS(x, y) ((uint16_t)(((x) + (y) - 1) / (y)))

/* Macro to convert time in us to periodic advertising interval units */
#define RADIO_SYNC_EVENTS(x, y) ((uint16_t)(((x) + (y) - 1) / (y)))

static inline uint8_t ull_ref_get(struct ull_hdr *hdr)
{
	return hdr->ref;
}

static inline uint8_t ull_ref_inc(struct ull_hdr *hdr)
{
	return ++hdr->ref;
}

static inline uint8_t ull_ref_dec(struct ull_hdr *hdr)
{
	return hdr->ref--;
}

static inline void ull_hdr_init(struct ull_hdr *hdr)
{
	hdr->ref = 0U;
	hdr->disabled_cb = hdr->disabled_param = NULL;
}

void *ll_rx_link_alloc(void);
void ll_rx_link_release(memq_link_t *link);
void *ll_rx_alloc(void);
void ll_rx_release(void *node_rx);
void *ll_pdu_rx_alloc_peek(uint8_t count);
void *ll_pdu_rx_alloc(void);
void ll_rx_put(memq_link_t *link, void *rx);
void ll_rx_sched(void);
void ull_ticker_status_give(uint32_t status, void *param);
uint32_t ull_ticker_status_take(uint32_t ret, uint32_t volatile *ret_cb);
void *ull_disable_mark(void *param);
void *ull_disable_unmark(void *param);
void *ull_disable_mark_get(void);
int ull_ticker_stop_with_mark(uint8_t ticker_handle, void *param,
			      void *lll_disable);
void *ull_update_mark(void *param);
void *ull_update_unmark(void *param);
void *ull_update_mark_get(void);
int ull_disable(void *param);
void ull_drift_ticks_get(struct node_rx_event_done *done,
			 uint32_t *ticks_drift_plus,
			 uint32_t *ticks_drift_minus);

/**
 * @brief   RX FIFO macro frontend
 * @details The RXFIFO data composite consists of an MFIFO with pointers to
 *          data elements backed by a memory pool and memq link elements.
 *          Link memq elements have a separate pool of (_count + _extra_links)
 *          elements. Extra links may be used for initializing one or more
 *          external memq instances. The following data structures are created
 *          with RXFIFO_DEFINE():
 *          - mfifo_<_name>:    FIFO with pointers to RX node elements.
 *          - mem_<_name>:      Backing data pool of <_count> RX node elements
 *                              of size <_size>.
 *          - mem_link_<_name>: Pool of <_count + _extra_links> memq_link_t
 *                              elements.
 */
#define RXFIFO_DEFINE(_name, _size, _count, _extra_links) \
	MFIFO_DEFINE(_name, sizeof(void *), _count); \
	\
	static struct { \
		void *free; \
		uint16_t size; \
		uint8_t count; \
		uint8_t extra_links; \
		uint8_t pool[MROUND(_size) * (_count)]; \
	} mem_##_name = { .size = MROUND(_size), .count = _count, \
			  .extra_links = _extra_links }; \
	\
	static struct { \
		void *free; \
		uint8_t pool[sizeof(memq_link_t) * \
		     (_count + _extra_links)]; \
	} mem_link_##_name

/**
 * @brief   Initializes MFIFO and pools
 * @details This makes the MFIFO empty and will subsequently need
 *          RXFIFO_ALLOC. Memory pools are initialized.
 */
#define RXFIFO_INIT(_name) \
	MFIFO_INIT(_name); \
	mem_init(mem_##_name.pool, mem_##_name.size, \
		 mem_##_name.count, &mem_##_name.free); \
	\
	mem_init(mem_link_##_name.pool, sizeof(memq_link_t), mem_##_name.count + \
		 mem_##_name.extra_links, &mem_link_##_name.free)

/**
 * @brief   Allocate FIFO elements with backing
 * @details This function allocates up to <_count> number of MFIFO elements by
 *          enqueuing pointers to memory elements with associated memq links.
 */
#define RXFIFO_ALLOC(_name, _count) \
	ull_rxfifo_alloc(mfifo_##_name.s, mfifo_##_name.n, mfifo_##_name.f, \
			 &mfifo_##_name.l, mfifo_##_name.m, &mem_##_name.free, \
			 &mem_link_##_name.free, _count)

/**
 * @brief Initialize and allocate MFIFO and pools
 */
#define RXFIFO_INIT_ALLOC(_name) \
	RXFIFO_INIT(_name); \
	RXFIFO_ALLOC(_name, mem_##_name.count)

/**
 * @brief   Release RX node
 * @details Enqueues an RX node back into the FIFO.
 */
#define RXFIFO_RELEASE(_name, _link, _rx) \
	ull_rxfifo_release(mfifo_##_name.s, mfifo_##_name.n, mfifo_##_name.f, \
			   &mfifo_##_name.l, mfifo_##_name.m, _link, \
			   (struct node_rx_hdr *)_rx)

void ull_rxfifo_alloc(uint8_t s, uint8_t n, uint8_t f, uint8_t *l, uint8_t *m,
		      void *mem_free, void *link_free, uint8_t max);
void *ull_rxfifo_release(uint8_t s, uint8_t n, uint8_t f, uint8_t *l, uint8_t *m,
			 memq_link_t *link, struct node_rx_hdr *rx);
