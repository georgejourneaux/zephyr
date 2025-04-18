#ifndef BT_CTLR_LLL_LLL_ADV_TYPES_H_
#define BT_CTLR_LLL_LLL_ADV_TYPES_H_

// #include "util/util.h"

struct lll_adv_pdu {
	uint8_t volatile first;
	uint8_t last;
	uint8_t *pdu[DOUBLE_BUFFER_SIZE];
};

#endif /* BT_CTLR_LLL_LLL_ADV_TYPES_H_ */
