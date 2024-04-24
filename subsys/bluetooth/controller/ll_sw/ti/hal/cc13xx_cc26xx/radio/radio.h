#ifndef BT_CTLR_RADIO_H
#define BT_CTLR_RADIO_H

#include <ti/drivers/rf/RF.h>

#define RADIO_RX_CONFIG_AUTO_FLUSH_IGNORED (1)
#define RADIO_RX_CONFIG_AUTO_FLUSH_CRC_ERR (1)
#define RADIO_RX_CONFIG_AUTO_FLUSH_EMPTY   (0)
#define RADIO_RX_CONFIG_INCLUDE_LEN_BYTE   (1)
#define RADIO_RX_CONFIG_INCLUDE_CRC        (1)
#define RADIO_RX_CONFIG_APPEND_RSSI        (0)
#define RADIO_RX_CONFIG_APPEND_STATUS      (0)
#define RADIO_RX_CONFIG_APPEND_TIMESTAMP   (0)

#define RF_RX_ADDITIONAL_DATA_BYTES                                                                \
	(RADIO_RX_CONFIG_INCLUDE_LEN_BYTE + RADIO_RX_CONFIG_INCLUDE_CRC +                          \
	 RADIO_RX_CONFIG_APPEND_RSSI + RADIO_RX_CONFIG_APPEND_STATUS +                             \
	 RADIO_RX_CONFIG_APPEND_TIMESTAMP)

#define RADIO_RF_EVENT_MASK_TX_DONE (RF_EventTxDone)

#define RADIO_RF_EVENT_MASK_RX_DONE                                                                \
	(RF_EventRxOk | RF_EventRxEmpty | RF_EventRxCtrl | RF_EventRxCtrlAck | RF_EventRxEntryDone)

#define RADIO_RF_EVENT_MASK_CMD_DONE (RF_EventCmdDone | RF_EventLastCmdDone)

#define RADIO_RF_EVENT_MASK_CMD_STOPPED                                                            \
	(RF_EventCmdCancelled | RF_EventCmdAborted | RF_EventCmdStopped)

#define RADIO_TXP_DEFAULT 0

typedef RF_Callback radio_isr_cb_t;

void radio_setup(void);
void radio_reset(void);
void radio_stop(void);
void radio_disable(radio_isr_cb_t callback);

uint32_t radio_rf_op_start_now(RF_Op *rf_op, radio_isr_cb_t callback);
uint32_t radio_rf_op_start_delayed(RF_Op *rf_op, uint32_t delay_ticks, radio_isr_cb_t callback);
uint32_t radio_rf_op_start_tick(RF_Op *rf_op, uint32_t start_tick, uint32_t remainder,
				radio_isr_cb_t callback);

void radio_isr(RF_Handle rf_handle, RF_CmdHandle command_handle, RF_EventMask event_mask);

void radio_tx_power_set(int8_t power);
void radio_tx_power_max_set(void);

uint32_t radio_rx_ready_delay_get(uint8_t phy, uint8_t flags);
uint32_t radio_rx_chain_delay_get(uint8_t phy, uint8_t flags);

uint32_t radio_tx_ready_delay_get(uint8_t phy, uint8_t flags);
uint32_t radio_tx_chain_delay_get(uint8_t phy, uint8_t flags);

#endif /* BT_CTLR_RADIO_H */
