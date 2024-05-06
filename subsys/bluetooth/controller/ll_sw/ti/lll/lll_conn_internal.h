#ifndef LLL_CONN_INTERNAL_H
#define LLL_CONN_INTERNAL_H

#include "hal/cc13xx_cc26xx/radio/rf_queue.h"

dataQueue_t *lll_conn_get_rf_rx_queue(void);
dataQueue_t *lll_conn_get_rf_tx_queue(void);

#endif /* LLL_CONN_INTERNAL_H */