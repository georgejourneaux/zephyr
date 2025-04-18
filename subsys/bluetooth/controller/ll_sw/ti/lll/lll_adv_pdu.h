#ifndef BT_CTLR_LLL_LLL_ADV_PDU_H_
#define BT_CTLR_LLL_LLL_ADV_PDU_H_

#include <soc.h>
// #include "ll_sw/lll_adv.h"
#include "lll/lll_adv_types.h"

#if defined(CONFIG_BT_CTLR_ADV_PDU_LINK)
#define PDU_ADV_MEM_SIZE MROUND(PDU_AC_LL_HEADER_SIZE + PDU_AC_PAYLOAD_SIZE_MAX + sizeof(uintptr_t))

#define PDU_ADV_NEXT_PTR(p)                                                                        \
	*(struct pdu_adv **)((uint8_t *)(p) + PDU_ADV_MEM_SIZE - sizeof(uintptr_t))
#else
#define PDU_ADV_MEM_SIZE MROUND(PDU_AC_LL_HEADER_SIZE + PDU_AC_PAYLOAD_SIZE_MAX)
#endif

int lll_adv_data_init(struct lll_adv_pdu *pdu);
int lll_adv_data_reset(struct lll_adv_pdu *pdu);
int lll_adv_data_dequeue(struct lll_adv_pdu *pdu);
int lll_adv_data_release(struct lll_adv_pdu *pdu);
void lll_adv_data_enqueue(struct lll_adv *lll, uint8_t idx);

struct pdu_adv *lll_adv_data_alloc(struct lll_adv *lll, uint8_t *idx);
struct pdu_adv *lll_adv_data_peek(struct lll_adv *lll);
struct pdu_adv *lll_adv_data_latest_peek(const struct lll_adv *const lll);

struct pdu_adv *lll_adv_pdu_alloc_pdu_adv(void);

void lll_adv_scan_rsp_enqueue(struct lll_adv *lll, uint8_t idx);
struct pdu_adv *lll_adv_scan_rsp_alloc(struct lll_adv *lll, uint8_t *idx);
struct pdu_adv *lll_adv_scan_rsp_peek(const struct lll_adv *lll);

#endif /* BT_CTLR_LLL_LLL_ADV_PDU_H_ */
