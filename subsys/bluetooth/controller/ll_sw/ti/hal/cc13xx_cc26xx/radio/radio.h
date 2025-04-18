#ifndef BT_CTLR_HAL_RADIO_H_
#define BT_CTLR_HAL_RADIO_H_

#include <ti/drivers/rf/RF.h>

#define RADIO_TXP_DEFAULT (0)

#define RADIO_RF_EVENT_MASK_TX_DONE (RF_EventTxDone)

#define RADIO_RF_EVENT_MASK_RX_DONE                                                                \
	(RF_EventRxOk | RF_EventRxEmpty | RF_EventRxCtrl | RF_EventRxCtrlAck | RF_EventRxEntryDone)

#define RADIO_RF_EVENT_MASK_CMD_DONE (RF_EventCmdDone | RF_EventLastCmdDone)

#define RADIO_RF_EVENT_MASK_CMD_STOPPED                                                            \
	(RF_EventCmdCancelled | RF_EventCmdAborted | RF_EventCmdStopped)

typedef RF_Callback radio_isr_cb_t;

int hal_radio_init(void);
void radio_disable(radio_isr_cb_t callback);

uint32_t radio_rx_ready_delay_get(uint8_t phy, uint8_t flags);
uint32_t radio_tx_ready_delay_get(uint8_t phy, uint8_t flags);

#endif /* BT_CTLR_HAL_RADIO_H_ */