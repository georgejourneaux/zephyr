#include <stdint.h>
#include <errno.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/power/PowerCC26X2.h>

#include <driverlib/rfc.h>
#include <driverlib/rf_mailbox.h>
#include <driverlib/rf_ble_mailbox.h>

#include <rf_patches/rf_patch_cpe_multi_protocol.h>

#include "hal/cntr.h"
#include "hal/radio.h"
#include "hal/swi.h"
#include "hal/ticker.h"

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_ti_radio);

#define RF_EVENT_DONE_MASK                                                                         \
	(RADIO_RF_EVENT_MASK_TX_DONE | RADIO_RF_EVENT_MASK_RX_DONE |                               \
	 RADIO_RF_EVENT_MASK_CMD_DONE | RADIO_RF_EVENT_MASK_CMD_STOPPED)

#define RF_EVENT_ERROR_MASK                                                                        \
	(RF_EventRxNOk | RF_EventRxBufFull | RF_EventRxAborted | RF_EventRxCollisionDetected |     \
	 RF_EventInternalError)

#define RF_EVENT_ISR_MASK (RF_EVENT_DONE_MASK | RF_EVENT_ERROR_MASK)

typedef enum HAL_RADIO_RF_PHY {
	HAL_RADIO_RF_PHY_1MBS = 0,
	HAL_RADIO_RF_PHY_2MBS,
	HAL_RADIO_RF_PHY_CODED,
} hal_radio_rf_phy_t;

typedef struct HAL_RADIO_RF {
	RF_Handle handle;
	RF_Object object;
	RF_Params params;
	RF_Mode mode;
	struct HAL_RADIO_RF_CMD {
		rfc_CMD_NOP_t nop;
		rfc_CMD_FS_t fs;
		rfc_CMD_CLEAR_RX_t clear_rx;
		rfc_CMD_BLE5_RADIO_SETUP_t ble5_radio_setup;
	} cmd;
} hal_radio_rf_t;

typedef struct HAL_RADIO_ISR {
	uint32_t latency;
	bool latency_done;
} hal_radio_isr_t;

typedef struct HAL_RADIO_DATA {
	hal_radio_isr_t isr;
	hal_radio_rf_t rf;
} hal_radio_data_t;

#if !(defined(CONFIG_PM) || defined(CONFIG_PM_DEVICE) || defined(CONFIG_POWEROFF))
const PowerCC26X2_Config PowerCC26X2_config = {
	.policyInitFxn = NULL,
	.policyFxn = &PowerCC26XX_doWFI,
	.calibrateFxn = &PowerCC26XX_calibrate,
	.enablePolicy = true,
	.calibrateRCOSC_LF = true,
	.calibrateRCOSC_HF = true,
};
#endif

static uint32_t override_ble_common[] = {
	// DC/DC regulator: In Tx, use DCDCCTL5[3:0]=0x3 (DITHER_EN=0 and IPEAK=3).
	(uint32_t)0x00F388D3,
	// Bluetooth 5: Set pilot tone length to 20 us Common
	HW_REG_OVERRIDE(0x6024, 0x2E20),
	// Bluetooth 5: Compensate for reduced pilot tone length
	(uint32_t)0x01280263,
	// Bluetooth 5: Default to no CTE.
	HW_REG_OVERRIDE(0x5328, 0x0000),
	// Synth: Increase mid code calibration time to 5 us
	(uint32_t)0x00058683,
	// Synth: Increase mid code calibration time to 5 us
	HW32_ARRAY_OVERRIDE(0x4004, 1),
	// Synth: Increase mid code calibration time to 5 us
	(uint32_t)0x38183C30,
	// Bluetooth 5: Move synth start code
	HW_REG_OVERRIDE(0x4064, 0x3C),
	// Bluetooth 5: Set DTX gain -5% for 1 Mbps
	(uint32_t)0x00E787E3,
	// Bluetooth 5: Set DTX threshold 1 Mbps
	(uint32_t)0x00950803,
	// Bluetooth 5: Set DTX gain -2.5% for 2 Mbps
	(uint32_t)0x00F487F3,
	// Bluetooth 5: Set DTX threshold 2 Mbps
	(uint32_t)0x012A0823,
	// Bluetooth 5: Set synth fine code calibration interval
	HW32_ARRAY_OVERRIDE(0x4020, 1),
	// Bluetooth 5: Set synth fine code calibration interval
	(uint32_t)0x41005F00,
	// Bluetooth 5: Adapt to synth fine code calibration interval
	(uint32_t)0xC0040141,
	// Bluetooth 5: Adapt to synth fine code calibration interval
	(uint32_t)0x0007DD44,
	// Bluetooth 5: Set enhanced TX shape
	(uint32_t)0x000D8C73, (uint32_t)0xFFFFFFFF};

uint32_t override_ble_1Mbps[] = {
	// Bluetooth 5: Set pilot tone length to 20 us
	HW_REG_OVERRIDE(0x5320, 0x03C0),
	// Bluetooth 5: Compensate syncTimeadjust
	(uint32_t)0x015302A3,
	// Symbol tracking: timing correction
	HW_REG_OVERRIDE(0x50D4, 0x00F9),
	// Symbol tracking: reduce sample delay
	HW_REG_OVERRIDE(0x50E0, 0x0087),
	// Symbol tracking: demodulation order
	HW_REG_OVERRIDE(0x50F8, 0x0014), (uint32_t)0xFFFFFFFF};

uint32_t override_ble_2Mbps[] = {
	// Bluetooth 5: Set pilot tone length to 20 us
	HW_REG_OVERRIDE(0x5320, 0x03C0),
	// Bluetooth 5: Compensate syncTimeAdjust
	(uint32_t)0x00F102A3,
	// Bluetooth 5: increase low gain AGC delay for 2 Mbps
	HW_REG_OVERRIDE(0x60A4, 0x7D00),
	// Symbol tracking: timing correction
	HW_REG_OVERRIDE(0x50D4, 0x00F9),
	// Symbol tracking: reduce sample delay
	HW_REG_OVERRIDE(0x50E0, 0x0087),
	// Symbol tracking: demodulation order
	HW_REG_OVERRIDE(0x50F8, 0x0014), (uint32_t)0xFFFFFFFF};

uint32_t override_ble_coded[] = {
	// Bluetooth 5: Set pilot tone length to 20 us
	HW_REG_OVERRIDE(0x5320, 0x03C0),
	// Bluetooth 5: Compensate syncTimeadjust
	(uint32_t)0x07A902A3,
	// Rx: Set AGC reference level to 0x21 (default: 0x2E)
	HW_REG_OVERRIDE(0x609C, 0x0021), (uint32_t)0xFFFFFFFF};

static hal_radio_data_t hal_radio_data = {
	.isr =
		{
			.latency = 0,
			.latency_done = false,
		},
	.rf.mode =
		{
			.rfMode = RF_MODE_AUTO,
			.cpePatchFxn = &rf_patch_cpe_multi_protocol,
			.mcePatchFxn = 0,
			.rfePatchFxn = 0,
		},
	.rf.cmd =
		{
			.fs = {.commandNo = CMD_FS,
			       .status = IDLE,
			       .pNextOp = NULL,
			       .startTime = 0,
			       .startTrigger.triggerType = TRIG_NOW,
			       .startTrigger.bEnaCmd = 0,
			       .startTrigger.triggerNo = 0,
			       .startTrigger.pastTrig = 0,
			       .condition.rule = COND_NEVER,
			       .condition.nSkip = COND_ALWAYS,
			       .frequency = 0,
			       .fractFreq = 0,
			       .synthConf.bTxMode = 0,
			       .synthConf.refFreq = 0,
			       .__dummy0 = 0,
			       .__dummy1 = 0,
			       .__dummy2 = 0,
			       .__dummy3 = 0},

			.nop =
				{
					.commandNo = CMD_NOP,
					.status = IDLE,
					.pNextOp = NULL,
					.startTime = 0,
					.startTrigger.triggerType = TRIG_NOW,
					.startTrigger.bEnaCmd = 0,
					.startTrigger.triggerNo = 0,
					.startTrigger.pastTrig = 0,
					.condition.rule = COND_NEVER,
					.condition.nSkip = COND_ALWAYS,
				},

			.clear_rx =
				{
					.commandNo = CMD_CLEAR_RX,
					.__dummy0 = 0,
					.pQueue = NULL,
				},

			.ble5_radio_setup =
				{
					.commandNo = CMD_BLE5_RADIO_SETUP,
					.status = IDLE,
					.pNextOp = NULL,
					.startTime = 0,
					.startTrigger.triggerType = TRIG_NOW,
					.startTrigger.bEnaCmd = 0,
					.startTrigger.triggerNo = 0,
					.startTrigger.pastTrig = 0,
					.condition.rule = COND_NEVER,
					.condition.nSkip = COND_ALWAYS,
					.defaultPhy.mainMode = HAL_RADIO_RF_PHY_1MBS,
					.defaultPhy.coding = 0,
					.loDivider = 0,
					.config.frontEndMode = 0,
					.config.biasMode = 1,
					.config.analogCfgMode = 0,
					.config.bNoFsPowerUp = 0,
					.config.bSynthNarrowBand = 0,
					.txPower = 0x7217,
					.pRegOverrideCommon = override_ble_common,
					.pRegOverride1Mbps = override_ble_1Mbps,
					.pRegOverride2Mbps = override_ble_2Mbps,
					.pRegOverrideCoded = override_ble_coded,
				},
		},
};
static hal_radio_data_t *driver_data = &hal_radio_data;

static void isr_latency_callback(RF_Handle rf_handle, RF_CmdHandle command_handle,
				 RF_EventMask event_mask);
static int measure_radio_isr_latency(void);

int hal_radio_init(void)
{
	RF_Params_init(&driver_data->rf.params);

	driver_data->rf.handle = RF_open(&driver_data->rf.object, &driver_data->rf.mode,
					 (RF_RadioSetup *)&driver_data->rf.cmd.ble5_radio_setup,
					 &driver_data->rf.params);
	if (driver_data->rf.handle == NULL) {
		LOG_ERR("RF_open failed");
		return -EIO;
	}

	return measure_radio_isr_latency();

	return 0;
}

void hal_radio_disable(radio_isr_cb_t callback)
{
	LOG_DBG("cntr %u (%uus)", cntr_cnt_get(), HAL_TICKER_TICKS_TO_US(cntr_cnt_get()));

	RF_runDirectCmd(driver_data->rf.handle, CMD_STOP);
	RF_runImmediateCmd(driver_data->rf.handle, (uint32_t *)&driver_data->rf.cmd.clear_rx);
	RF_postCmd(driver_data->rf.handle, (RF_Op *)&driver_data->rf.cmd.nop, RF_PriorityNormal,
		   callback, RF_EVENT_ISR_MASK);
}

uint32_t radio_rx_ready_delay_get(uint8_t phy, uint8_t flags)
{
#warning "TODO"
	return 0;
}

uint32_t radio_tx_ready_delay_get(uint8_t phy, uint8_t flags)
{
#warning "TODO"
	return 0;
}

static void isr_latency_callback(RF_Handle rf_handle, RF_CmdHandle command_handle,
				 RF_EventMask event_mask)
{
	if (driver_data->isr.latency == 0) {
		driver_data->isr.latency = HAL_TICKER_TICKS_TO_US(cntr_cnt_get());
	}

	if (event_mask & RF_EVENT_DONE_MASK) {
		driver_data->isr.latency_done = true;
	}
}

static int measure_radio_isr_latency(void)
{
	irq_enable(HAL_RADIO_IRQ);

	driver_data->isr.latency = 0;
	driver_data->isr.latency_done = false;

	hal_radio_disable(isr_latency_callback);

	uint32_t timeout_ticks =
		HAL_TICKER_TICKS_TO_US(cntr_cnt_get()) + HAL_TICKER_US_TO_TICKS(1000000);
	while ((!driver_data->isr.latency_done) && (timeout_ticks > cntr_cnt_get())) {
	}

	if (!driver_data->isr.latency_done) {
		LOG_ERR("measure_radio_isr_latency failed");
		return -EIO;
	}

	LOG_DBG("latency-start: %uus", driver_data->isr.latency);
	driver_data->isr.latency =
		HAL_TICKER_TICKS_TO_US(cntr_cnt_get()) - driver_data->isr.latency;
	LOG_DBG("latency: %uus", driver_data->isr.latency);

	irq_disable(HAL_RADIO_IRQ);

	return 0;
}