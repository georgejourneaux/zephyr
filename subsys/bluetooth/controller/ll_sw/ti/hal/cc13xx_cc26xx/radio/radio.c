#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/power/PowerCC26X2.h>

#include <driverlib/rfc.h>
#include <driverlib/rf_mailbox.h>
#include <driverlib/rf_ble_mailbox.h>

#include <rf_patches/rf_patch_cpe_multi_protocol.h>

#include "hal/radio.h"
#include "hal/cc13xx_cc26xx/ll_irqs.h"

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_ti_radio);

#define ISR_LATENCY_DONE (0xfedcba98)

#define RF_EVENT_DONE_MASK                                                                         \
	(RADIO_RF_EVENT_MASK_TX_DONE | RADIO_RF_EVENT_MASK_RX_DONE |                               \
	 RADIO_RF_EVENT_MASK_CMD_DONE | RADIO_RF_EVENT_MASK_CMD_STOPPED)

#define RF_EVENT_ERROR_MASK                                                                        \
	(RF_EventRxNOk | RF_EventRxBufFull | RF_EventRxAborted | RF_EventRxCollisionDetected |     \
	 RF_EventInternalError)

#define RF_EVENT_ISR_MASK (RF_EVENT_DONE_MASK | RF_EVENT_ERROR_MASK)

typedef enum RF_TX_POWER {
	RADIO_TX_POWER_m20 = 0,
	RADIO_TX_POWER_m18,
	RADIO_TX_POWER_m15,
	RADIO_TX_POWER_m12,
	RADIO_TX_POWER_m10,
	RADIO_TX_POWER_m9,
	RADIO_TX_POWER_m6,
	RADIO_TX_POWER_m5,
	RADIO_TX_POWER_m3,
	RADIO_TX_POWER_0,
	RADIO_TX_POWER_1,
	RADIO_TX_POWER_2,
	RADIO_TX_POWER_3,
	RADIO_TX_POWER_4,
	RADIO_TX_POWER_5,

	RADIO_TX_POWER_TABLE_END,
	RADIO_TX_POWER_TABLE_SIZE,
} rf_tx_power_t;

typedef enum RF_CHANNEL {
	RF_CHANNEL_0 = 0,
	RF_CHANNEL_1,
	RF_CHANNEL_2,
	RF_CHANNEL_3,
	RF_CHANNEL_4,
	RF_CHANNEL_5,
	RF_CHANNEL_6,
	RF_CHANNEL_7,
	RF_CHANNEL_8,
	RF_CHANNEL_9,
	RF_CHANNEL_10,
	RF_CHANNEL_11,
	RF_CHANNEL_12,
	RF_CHANNEL_13,
	RF_CHANNEL_14,
	RF_CHANNEL_15,
	RF_CHANNEL_16,
	RF_CHANNEL_17,
	RF_CHANNEL_18,
	RF_CHANNEL_19,
	RF_CHANNEL_20,
	RF_CHANNEL_21,
	RF_CHANNEL_22,
	RF_CHANNEL_23,
	RF_CHANNEL_24,
	RF_CHANNEL_25,
	RF_CHANNEL_26,
	RF_CHANNEL_27,
	RF_CHANNEL_28,
	RF_CHANNEL_29,
	RF_CHANNEL_30,
	RF_CHANNEL_31,
	RF_CHANNEL_32,
	RF_CHANNEL_33,
	RF_CHANNEL_34,
	RF_CHANNEL_35,
	RF_CHANNEL_36,
	RF_CHANNEL_37,
	RF_CHANNEL_38,
	RF_CHANNEL_39,

	RF_CHANNEL_COUNT,
} rf_channel_t;

typedef enum RF_CHANNEL_FREQUENCY {
	RF_CHANNEL_FREQUENCY_2402 = RF_CHANNEL_37,
	RF_CHANNEL_FREQUENCY_2404 = RF_CHANNEL_0,
	RF_CHANNEL_FREQUENCY_2406 = RF_CHANNEL_1,
	RF_CHANNEL_FREQUENCY_2408 = RF_CHANNEL_2,
	RF_CHANNEL_FREQUENCY_2410 = RF_CHANNEL_3,
	RF_CHANNEL_FREQUENCY_2412 = RF_CHANNEL_4,
	RF_CHANNEL_FREQUENCY_2414 = RF_CHANNEL_5,
	RF_CHANNEL_FREQUENCY_2416 = RF_CHANNEL_6,
	RF_CHANNEL_FREQUENCY_2418 = RF_CHANNEL_7,
	RF_CHANNEL_FREQUENCY_2420 = RF_CHANNEL_8,
	RF_CHANNEL_FREQUENCY_2422 = RF_CHANNEL_9,
	RF_CHANNEL_FREQUENCY_2424 = RF_CHANNEL_10,
	RF_CHANNEL_FREQUENCY_2426 = RF_CHANNEL_38,
	RF_CHANNEL_FREQUENCY_2428 = RF_CHANNEL_11,
	RF_CHANNEL_FREQUENCY_2430 = RF_CHANNEL_12,
	RF_CHANNEL_FREQUENCY_2432 = RF_CHANNEL_13,
	RF_CHANNEL_FREQUENCY_2434 = RF_CHANNEL_14,
	RF_CHANNEL_FREQUENCY_2436 = RF_CHANNEL_15,
	RF_CHANNEL_FREQUENCY_2438 = RF_CHANNEL_16,
	RF_CHANNEL_FREQUENCY_2440 = RF_CHANNEL_17,
	RF_CHANNEL_FREQUENCY_2442 = RF_CHANNEL_18,
	RF_CHANNEL_FREQUENCY_2444 = RF_CHANNEL_19,
	RF_CHANNEL_FREQUENCY_2446 = RF_CHANNEL_20,
	RF_CHANNEL_FREQUENCY_2448 = RF_CHANNEL_21,
	RF_CHANNEL_FREQUENCY_2450 = RF_CHANNEL_22,
	RF_CHANNEL_FREQUENCY_2452 = RF_CHANNEL_23,
	RF_CHANNEL_FREQUENCY_2454 = RF_CHANNEL_24,
	RF_CHANNEL_FREQUENCY_2456 = RF_CHANNEL_25,
	RF_CHANNEL_FREQUENCY_2458 = RF_CHANNEL_26,
	RF_CHANNEL_FREQUENCY_2460 = RF_CHANNEL_27,
	RF_CHANNEL_FREQUENCY_2462 = RF_CHANNEL_28,
	RF_CHANNEL_FREQUENCY_2464 = RF_CHANNEL_29,
	RF_CHANNEL_FREQUENCY_2466 = RF_CHANNEL_30,
	RF_CHANNEL_FREQUENCY_2468 = RF_CHANNEL_31,
	RF_CHANNEL_FREQUENCY_2470 = RF_CHANNEL_32,
	RF_CHANNEL_FREQUENCY_2472 = RF_CHANNEL_33,
	RF_CHANNEL_FREQUENCY_2474 = RF_CHANNEL_34,
	RF_CHANNEL_FREQUENCY_2476 = RF_CHANNEL_35,
	RF_CHANNEL_FREQUENCY_2478 = RF_CHANNEL_36,
	RF_CHANNEL_FREQUENCY_2480 = RF_CHANNEL_39,

	RF_CHANNEL_FREQUENCY_COUNT,
} rf_channel_frequency_t;

typedef enum RF_PHY {
	RF_PHY_1MBS = 0,
	RF_PHY_2MBS,
	RF_PHY_CODED,
} rf_phy_t;

typedef struct rf_frequency_table_entry {
	const uint16_t frequency;
	const uint8_t whitening;
} rf_frequency_table_entry_t;

typedef struct BLE_CC13XX_CC26XX_RF {
	RF_Handle handle;
	RF_Object object;
	RF_Params params;
	RF_Mode mode;
	struct BLE_CC13XX_CC26XX_RF {
		rfc_CMD_NOP_t nop;
		rfc_CMD_FS_t fs;
		rfc_CMD_CLEAR_RX_t clear_rx;
		rfc_CMD_BLE5_RADIO_SETUP_t ble5_radio_setup;
	} cmd;
} ble_cc13xx_cc26xx_rf_t;

typedef struct BLE_CC13XX_CC26XX_DATA {
	ble_cc13xx_cc26xx_rf_t rf;
} ble_cc13xx_cc26xx_data_t;

static const rf_frequency_table_entry_t channel_frequency_table[RF_CHANNEL_FREQUENCY_COUNT] = {
	[RF_CHANNEL_FREQUENCY_2402] = {.frequency = 2402, .whitening = 0xE5},
	[RF_CHANNEL_FREQUENCY_2404] = {.frequency = 2404, .whitening = 0xC0},
	[RF_CHANNEL_FREQUENCY_2406] = {.frequency = 2406, .whitening = 0xC1},
	[RF_CHANNEL_FREQUENCY_2408] = {.frequency = 2408, .whitening = 0xC2},
	[RF_CHANNEL_FREQUENCY_2410] = {.frequency = 2410, .whitening = 0xC3},
	[RF_CHANNEL_FREQUENCY_2412] = {.frequency = 2412, .whitening = 0xC4},
	[RF_CHANNEL_FREQUENCY_2414] = {.frequency = 2414, .whitening = 0xC5},
	[RF_CHANNEL_FREQUENCY_2416] = {.frequency = 2416, .whitening = 0xC6},
	[RF_CHANNEL_FREQUENCY_2418] = {.frequency = 2418, .whitening = 0xC7},
	[RF_CHANNEL_FREQUENCY_2420] = {.frequency = 2420, .whitening = 0xC8},
	[RF_CHANNEL_FREQUENCY_2422] = {.frequency = 2422, .whitening = 0xC9},
	[RF_CHANNEL_FREQUENCY_2424] = {.frequency = 2424, .whitening = 0xCA},
	[RF_CHANNEL_FREQUENCY_2426] = {.frequency = 2426, .whitening = 0xE6},
	[RF_CHANNEL_FREQUENCY_2428] = {.frequency = 2428, .whitening = 0xCB},
	[RF_CHANNEL_FREQUENCY_2430] = {.frequency = 2430, .whitening = 0xCC},
	[RF_CHANNEL_FREQUENCY_2432] = {.frequency = 2432, .whitening = 0xCD},
	[RF_CHANNEL_FREQUENCY_2434] = {.frequency = 2434, .whitening = 0xCE},
	[RF_CHANNEL_FREQUENCY_2436] = {.frequency = 2436, .whitening = 0xCF},
	[RF_CHANNEL_FREQUENCY_2438] = {.frequency = 2438, .whitening = 0xD0},
	[RF_CHANNEL_FREQUENCY_2440] = {.frequency = 2440, .whitening = 0xD1},
	[RF_CHANNEL_FREQUENCY_2442] = {.frequency = 2442, .whitening = 0xD2},
	[RF_CHANNEL_FREQUENCY_2444] = {.frequency = 2444, .whitening = 0xD3},
	[RF_CHANNEL_FREQUENCY_2446] = {.frequency = 2446, .whitening = 0xD4},
	[RF_CHANNEL_FREQUENCY_2448] = {.frequency = 2448, .whitening = 0xD5},
	[RF_CHANNEL_FREQUENCY_2450] = {.frequency = 2450, .whitening = 0xD6},
	[RF_CHANNEL_FREQUENCY_2452] = {.frequency = 2452, .whitening = 0xD7},
	[RF_CHANNEL_FREQUENCY_2454] = {.frequency = 2454, .whitening = 0xD8},
	[RF_CHANNEL_FREQUENCY_2456] = {.frequency = 2456, .whitening = 0xD9},
	[RF_CHANNEL_FREQUENCY_2458] = {.frequency = 2458, .whitening = 0xDA},
	[RF_CHANNEL_FREQUENCY_2460] = {.frequency = 2460, .whitening = 0xDB},
	[RF_CHANNEL_FREQUENCY_2462] = {.frequency = 2462, .whitening = 0xDC},
	[RF_CHANNEL_FREQUENCY_2464] = {.frequency = 2464, .whitening = 0xDD},
	[RF_CHANNEL_FREQUENCY_2466] = {.frequency = 2466, .whitening = 0xDE},
	[RF_CHANNEL_FREQUENCY_2468] = {.frequency = 2468, .whitening = 0xDF},
	[RF_CHANNEL_FREQUENCY_2470] = {.frequency = 2470, .whitening = 0xE0},
	[RF_CHANNEL_FREQUENCY_2472] = {.frequency = 2472, .whitening = 0xE1},
	[RF_CHANNEL_FREQUENCY_2474] = {.frequency = 2474, .whitening = 0xE2},
	[RF_CHANNEL_FREQUENCY_2476] = {.frequency = 2476, .whitening = 0xE3},
	[RF_CHANNEL_FREQUENCY_2478] = {.frequency = 2478, .whitening = 0xE4},
	[RF_CHANNEL_FREQUENCY_2480] = {.frequency = 2480, .whitening = 0xE7}};

static const RF_TxPowerTable_Entry RF_BLE_txPowerTable[RADIO_TX_POWER_TABLE_SIZE] = {
	[RADIO_TX_POWER_m20] =
		{
			.power = -20,
			.value = RF_TxPowerTable_DEFAULT_PA_ENTRY(6, 3, 0, 2),
		}, /* 0x04C6 */
	[RADIO_TX_POWER_m18] =
		{
			.power = -18,
			.value = RF_TxPowerTable_DEFAULT_PA_ENTRY(8, 3, 0, 3),
		}, /* 0x06C8 */
	[RADIO_TX_POWER_m15] =
		{
			.power = -15,
			.value = RF_TxPowerTable_DEFAULT_PA_ENTRY(10, 3, 0, 3),
		}, /* 0x06CA */
	[RADIO_TX_POWER_m12] =
		{
			.power = -12,
			.value = RF_TxPowerTable_DEFAULT_PA_ENTRY(12, 3, 0, 5),
		}, /* 0x0ACC */
	[RADIO_TX_POWER_m10] =
		{
			.power = -10,
			.value = RF_TxPowerTable_DEFAULT_PA_ENTRY(15, 3, 0, 5),
		}, /* 0x0ACF */
	[RADIO_TX_POWER_m9] =
		{
			.power = -9,
			.value = RF_TxPowerTable_DEFAULT_PA_ENTRY(16, 3, 0, 5),
		}, /* 0x0AD0 */
	[RADIO_TX_POWER_m6] =
		{
			.power = -6,
			.value = RF_TxPowerTable_DEFAULT_PA_ENTRY(20, 3, 0, 8),
		}, /* 0x10D4 */
	[RADIO_TX_POWER_m5] =
		{
			.power = -5,
			.value = RF_TxPowerTable_DEFAULT_PA_ENTRY(22, 3, 0, 9),
		}, /* 0x12D6 */
	[RADIO_TX_POWER_m3] =
		{
			.power = -3,
			.value = RF_TxPowerTable_DEFAULT_PA_ENTRY(19, 2, 0, 12),
		}, /* 0x1893 */
	[RADIO_TX_POWER_0] =
		{
			.power = 0,
			.value = RF_TxPowerTable_DEFAULT_PA_ENTRY(19, 1, 0, 20),
		}, /* 0x2853 */
	[RADIO_TX_POWER_1] =
		{
			.power = 1,
			.value = RF_TxPowerTable_DEFAULT_PA_ENTRY(22, 1, 0, 20),
		}, /* 0x2856 */
	[RADIO_TX_POWER_2] =
		{
			.power = 2,
			.value = RF_TxPowerTable_DEFAULT_PA_ENTRY(25, 1, 0, 25),
		}, /* 0x3259 */
	[RADIO_TX_POWER_3] =
		{
			.power = 3,
			.value = RF_TxPowerTable_DEFAULT_PA_ENTRY(29, 1, 0, 28),
		}, /* 0x385D */
	[RADIO_TX_POWER_4] =
		{
			.power = 4,
			.value = RF_TxPowerTable_DEFAULT_PA_ENTRY(35, 1, 0, 39),
		}, /* 0x4E63 */
	[RADIO_TX_POWER_5] =
		{
			.power = 5,
			.value = RF_TxPowerTable_DEFAULT_PA_ENTRY(23, 0, 0, 57),
		}, /* 0x7217 */
	[RADIO_TX_POWER_TABLE_END] = RF_TxPowerTable_TERMINATION_ENTRY};

// Overrides for CMD_BLE5_RADIO_SETUP
static uint32_t overrides_ble_common[] = {
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

// Overrides for CMD_BLE5_RADIO_SETUP
uint32_t overrides_ble_1Mbps[] = {
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

// Overrides for CMD_BLE5_RADIO_SETUP
uint32_t overrides_ble_2Mbps[] = {
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

// Overrides for CMD_BLE5_RADIO_SETUP
uint32_t overrides_ble_coded[] = {
	// Bluetooth 5: Set pilot tone length to 20 us
	HW_REG_OVERRIDE(0x5320, 0x03C0),
	// Bluetooth 5: Compensate syncTimeadjust
	(uint32_t)0x07A902A3,
	// Rx: Set AGC reference level to 0x21 (default: 0x2E)
	HW_REG_OVERRIDE(0x609C, 0x0021), (uint32_t)0xFFFFFFFF};

static ble_cc13xx_cc26xx_data_t ble_cc13xx_cc26xx_data = {
	.rf.handle = NULL,
	.rf.object = {0},
	.rf.params = {0},
	.rf.mode =
		{
			.rfMode = RF_MODE_AUTO,
			.cpePatchFxn = &rf_patch_cpe_multi_protocol,
			.mcePatchFxn = 0,
			.rfePatchFxn = 0,
		},
	.rf.cmd = {
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

		.ble5_radio_setup = {.commandNo = CMD_BLE5_RADIO_SETUP,
				     .status = IDLE,
				     .pNextOp = NULL,
				     .startTime = 0,
				     .startTrigger.triggerType = TRIG_NOW,
				     .startTrigger.bEnaCmd = 0,
				     .startTrigger.triggerNo = 0,
				     .startTrigger.pastTrig = 0,
				     .condition.rule = COND_NEVER,
				     .condition.nSkip = COND_ALWAYS,
				     .defaultPhy.mainMode = RF_PHY_1MBS,
				     .defaultPhy.coding = 0,
				     .loDivider = 0,
				     .config.frontEndMode = 0,
				     .config.biasMode = 1,
				     .config.analogCfgMode = 0,
				     .config.bNoFsPowerUp = 0,
				     .config.bSynthNarrowBand = 0,
				     .txPower = 0x7217,
				     .pRegOverrideCommon = overrides_ble_common,
				     .pRegOverride1Mbps = overrides_ble_1Mbps,
				     .pRegOverride2Mbps = overrides_ble_2Mbps,
				     .pRegOverrideCoded = overrides_ble_coded},
	}};
static ble_cc13xx_cc26xx_data_t *driver_data = &ble_cc13xx_cc26xx_data;

static uint32_t isr_latency_status = 0;
static uint32_t isr_latency = 0;

#if !(defined(CONFIG_PM) || defined(CONFIG_PM_DEVICE) || defined(CONFIG_POWEROFF))
static const PowerCC26X2_Config PowerCC26X2_config = {
	.policyInitFxn = NULL,
	.policyFxn = &PowerCC26XX_doWFI,
	.calibrateFxn = &PowerCC26XX_calibrate,
	.enablePolicy = true,
	.calibrateRCOSC_LF = true,
	.calibrateRCOSC_HF = true,
};
#endif

static void isr_latency_callback(RF_Handle rf_handle, RF_CmdHandle command_handle,
				 RF_EventMask event_mask);
static void get_isr_latency(void);

static const char *const rf_get_command_string(RF_CmdHandle commandNo);
static const char *const rf_get_status_string(uint16_t status);
static void rf_log_dbg_event_mask(RF_EventMask event);

void radio_setup(void)
{
	RF_Params_init(&driver_data->rf.params);

	driver_data->rf.handle = RF_open(&driver_data->rf.object, &driver_data->rf.mode,
					 (RF_RadioSetup *)&driver_data->rf.cmd.ble5_radio_setup,
					 &driver_data->rf.params);
	LL_ASSERT(driver_data->rf.handle);

	driver_data->rf.cmd.fs.frequency = channel_frequency_table[RF_CHANNEL_37].frequency;
	RF_runCmd(driver_data->rf.handle, (RF_Op *)&driver_data->rf.cmd.fs, RF_PriorityNormal, NULL,
		  RF_EventLastCmdDone);

	get_isr_latency();
}

void radio_reset(void)
{
	LOG_DBG("cntr %u (%uus)", cntr_cnt_get(), HAL_TICKER_TICKS_TO_US(cntr_cnt_get()));
	irq_disable(LL_RADIO_IRQn);
}

void radio_stop(void)
{
	LOG_DBG("cntr %u (%uus)", cntr_cnt_get(), HAL_TICKER_TICKS_TO_US(cntr_cnt_get()));
}

void radio_disable(void)
{
	LOG_DBG("cntr %u (%uus)", cntr_cnt_get(), HAL_TICKER_TICKS_TO_US(cntr_cnt_get()));

	RF_runDirectCmd(driver_data->rf.handle, CMD_STOP);
	RF_runImmediateCmd(driver_data->rf.handle, (uint32_t *)&driver_data->rf.cmd.clear_rx);
}

uint32_t radio_rf_op_start_now(RF_Op *rf_op, uint32_t timeout_ticks, radio_isr_cb_t callback)
{
	return radio_rf_op_start_tick(rf_op, 0, 1, timeout_ticks, callback);
}

uint32_t radio_rf_op_start_delayed(RF_Op *rf_op, uint32_t delay_ticks, uint32_t timeout_ticks,
				   radio_isr_cb_t callback)
{
	return radio_rf_op_start_tick(rf_op, (cntr_cnt_get() + delay_ticks), 1, timeout_ticks,
				      callback);
}

uint32_t radio_rf_op_start_tick(RF_Op *rf_op, uint32_t start_tick, uint32_t remainder,
				uint32_t timeout_ticks, radio_isr_cb_t callback)
{
	uint32_t now = cntr_cnt_get();

	/* Save it for later */
	// rtc_start = start_ticks;

#warning "TODO: Check if start time has already passed"
	rfc_ble5RadioOp_t *ble_radio_op = (rfc_ble5RadioOp_t *)rf_op;
	ble_radio_op->startTime = start_tick + remainder;
	ble_radio_op->startTrigger.triggerType = TRIG_ABSTIME;
	ble_radio_op->startTrigger.pastTrig = true;

	LOG_DBG("%s (0x%04X) ch %u starts in %u", rf_get_command_string(ble_radio_op->commandNo),
		ble_radio_op->commandNo, ble_radio_op->channel, (ble_radio_op->startTime - now));

	RF_postCmd(driver_data->rf.handle, rf_op, RF_PriorityNormal, callback, RF_EVENT_ISR_MASK);

	return (ble_radio_op->startTime - now);
}

void radio_isr(RF_Handle rf_handle, RF_CmdHandle command_handle, RF_EventMask event_mask)
{
	RF_Op *rf_op = RF_getCmdOp(rf_handle, command_handle);
	LOG_DBG("%s (0x%04X) %s (0x%04X)", rf_get_command_string(rf_op->commandNo),
		rf_op->commandNo, rf_get_status_string(rf_op->status), rf_op->status);
	rf_log_dbg_event_mask(event_mask);
}

void radio_tx_power_set(int8_t power)
{
	LOG_DBG("%u", power);
	RF_setTxPower(
		driver_data->rf.handle,
		RF_TxPowerTable_findValue((RF_TxPowerTable_Entry *)RF_BLE_txPowerTable, power));
}

void radio_tx_power_max_set(void)
{
	radio_tx_power_set(RF_TxPowerTable_MAX_DBM);
}

uint32_t radio_rx_ready_delay_get(uint8_t phy, uint8_t flags)
{
	return 0;
}

uint32_t radio_rx_chain_delay_get(uint8_t phy, uint8_t flags)
{
#warning "TODO: Figure this out"
	return 16 + 2 * 32 + 8 + 32 + isr_latency;
}

uint32_t radio_tx_ready_delay_get(uint8_t phy, uint8_t flags)
{
	return 0;
}

uint32_t radio_tx_chain_delay_get(uint8_t phy, uint8_t flags)
{
	return 0;
}

static void isr_latency_callback(RF_Handle rf_handle, RF_CmdHandle command_handle,
				 RF_EventMask event_mask)
{
	isr_latency = HAL_TICKER_TICKS_TO_US(cntr_cnt_get());
	isr_latency_status = ISR_LATENCY_DONE;
}

static void get_isr_latency(void)
{
	isr_latency_status = 0;

	radio_disable();
	RF_postCmd(driver_data->rf.handle, &driver_data->rf.cmd.nop, RF_PriorityNormal,
		   isr_latency_callback, RF_EVENT_ISR_MASK);

	while (isr_latency_status != ISR_LATENCY_DONE) {
	}
	isr_latency -= HAL_TICKER_TICKS_TO_US(cntr_cnt_get());

	irq_disable(LL_RADIO_IRQn);
}

static const char *const rf_get_command_string(RF_CmdHandle commandNo)
{
	switch (commandNo) {
	case CMD_FS:
		return "CMD_FS";
	case CMD_NOP:
		return "CMD_NOP";
	case CMD_CLEAR_RX:
		return "CMD_CLEAR_RX";

	case CMD_BLE5_RADIO_SETUP:
		return "CMD_BLE5_RADIO_SETUP";
	case CMD_BLE5_ADV:
		return "CMD_BLE5_ADV";
	case CMD_BLE_ADV_PAYLOAD:
		return "CMD_BLE_ADV_PAYLOAD";
	case CMD_BLE5_GENERIC_RX:
		return "CMD_BLE5_GENERIC_RX";
	case CMD_BLE5_SLAVE:
		return "CMD_BLE5_SLAVE";

	default:
		break;
	}

	return "unknown";
}

static const char *const rf_get_status_string(uint16_t status)
{
	switch (status) {
	case IDLE:
		return "IDLE";
	case PENDING:
		return "PENDING";
	case ACTIVE:
		return "ACTIVE";
	case SKIPPED:
		return "SKIPPED";

	case DONE_OK:
		return "DONE_OK";
	case DONE_COUNTDOWN:
		return "DONE_COUNTDOWN";
	case DONE_RXERR:
		return "DONE_RXERR";
	case DONE_TIMEOUT:
		return "DONE_TIMEOUT";
	case DONE_STOPPED:
		return "DONE_STOPPED";
	case DONE_ABORT:
		return "DONE_ABORT";
	case DONE_FAILED:
		return "DONE_FAILED";

	case ERROR_PAST_START:
		return "ERROR_PAST_START";
	case ERROR_START_TRIG:
		return "ERROR_START_TRIG";
	case ERROR_CONDITION:
		return "ERROR_CONDITION";
	case ERROR_PAR:
		return "ERROR_PAR";
	case ERROR_POINTER:
		return "ERROR_POINTER";
	case ERROR_CMDID:
		return "ERROR_CMDID";
	case ERROR_WRONG_BG:
		return "ERROR_WRONG_BG";
	case ERROR_NO_SETUP:
		return "ERROR_NO_SETUP";
	case ERROR_NO_FS:
		return "ERROR_NO_FS";
	case ERROR_SYNTH_PROG:
		return "ERROR_SYNTH_PROG";
	case ERROR_TXUNF:
		return "ERROR_TXUNF";
	case ERROR_RXOVF:
		return "ERROR_RXOVF";
	case ERROR_NO_RX:
		return "ERROR_NO_RX";
	case ERROR_PENDING:
		return "ERROR_PENDING";

	case BLE_DONE_OK:
		return "BLE_DONE_OK";
	case BLE_DONE_RXTIMEOUT:
		return "BLE_DONE_RXTIMEOUT";
	case BLE_DONE_NOSYNC:
		return "BLE_DONE_NOSYNC";
	case BLE_DONE_RXERR:
		return "BLE_DONE_RXERR";
	case BLE_DONE_CONNECT:
		return "BLE_DONE_CONNECT";
	case BLE_DONE_MAXNACK:
		return "BLE_DONE_MAXNACK";
	case BLE_DONE_ENDED:
		return "BLE_DONE_ENDED";
	case BLE_DONE_ABORT:
		return "BLE_DONE_ABORT";
	case BLE_DONE_STOPPED:
		return "BLE_DONE_STOPPED";
	case BLE_DONE_AUX:
		return "BLE_DONE_AUX";
	case BLE_DONE_CONNECT_CHSEL0:
		return "BLE_DONE_CONNECT_CHSEL0";
	case BLE_DONE_SCAN_RSP:
		return "BLE_DONE_SCAN_RSP";
	case BLE_ERROR_PAR:
		return "BLE_ERROR_PAR";
	case BLE_ERROR_RXBUF:
		return "BLE_ERROR_RXBUF";
	case BLE_ERROR_NO_SETUP:
		return "BLE_ERROR_NO_SETUP";
	case BLE_ERROR_NO_FS:
		return "BLE_ERROR_NO_FS";
	case BLE_ERROR_SYNTH_PROG:
		return "BLE_ERROR_SYNTH_PROG";
	case BLE_ERROR_RXOVF:
		return "BLE_ERROR_RXOVF";
	case BLE_ERROR_TXUNF:
		return "BLE_ERROR_TXUNF";
	case BLE_ERROR_AUX:
		return "BLE_ERROR_AUX";

	default:
		break;
	}

	return "unknown";
}

static void rf_log_dbg_event_mask(RF_EventMask event)
{
	if (event & RF_EventCmdDone) {
		LOG_DBG("Command done");
	}
	if (event & RF_EventLastCmdDone) {
		LOG_DBG("Last command done");
	}
	if (event & RF_EventFGCmdDone) {
		LOG_DBG("IEEE command done");
	}
	if (event & RF_EventLastFGCmdDone) {
		LOG_DBG("Last IEEE command done");
	}
	if (event & RF_EventTxDone) {
		LOG_DBG("Tx sent");
	}
	if (event & RF_EventTXAck) {
		LOG_DBG("Tx ACK sent");
	}
	if (event & RF_EventTxCtrl) {
		LOG_DBG("Tx control sent");
	}
	if (event & RF_EventTxCtrlAck) {
		LOG_DBG("Tx control ack received");
	}
	if (event & RF_EventTxCtrlAckAck) {
		LOG_DBG("Tx control ack ack sent");
	}
	if (event & RF_EventTxRetrans) {
		LOG_DBG("Tx retransmit sent");
	}
	if (event & RF_EventTxEntryDone) {
		LOG_DBG("Tx queue data entry state = finished");
	}
	if (event & RF_EventTxBufferChange) {
		LOG_DBG("Tx buffer changed");
	}
	if (event & RF_EventPaChanged) {
		LOG_DBG("PA changed");
	}
	if (event & RF_EventRxOk) {
		LOG_DBG("Rx ok");
	}
	if (event & RF_EventRxNOk) {
		LOG_DBG("Rx CRC error");
	}
	if (event & RF_EventRxIgnored) {
		LOG_DBG("Rx ok - ignore");
	}
	if (event & RF_EventRxEmpty) {
		LOG_DBG("Rx ok - no payload");
	}
	if (event & RF_EventRxCtrl) {
		LOG_DBG("Rx control ok");
	}
	if (event & RF_EventRxCtrlAck) {
		LOG_DBG("Rx control ok - ACK sent");
	}
	if (event & RF_EventRxBufFull) {
		LOG_DBG("Rx too big for queue");
	}
	if (event & RF_EventRxEntryDone) {
		LOG_DBG("Rx queue data entry state = finished");
	}
	if (event & RF_EventDataWritten) {
		LOG_DBG("Data written to partial read Rx buffer");
	}
	if (event & RF_EventNDataWritten) {
		LOG_DBG("Specified number of bytes written to partial read Rx buffer");
	}
	if (event & RF_EventRxAborted) {
		LOG_DBG("Rx aborted");
	}
	if (event & RF_EventRxCollisionDetected) {
		LOG_DBG("Rx collision");
	}
	if (event & RF_EventModulesUnlocked) {
		LOG_DBG("CM0 boot - RF core modules and memories unlocked");
	}
	if (event & RF_EventInternalError) {
		LOG_DBG("Internal error");
	}
	if (event & RF_EventMdmSoft) {
		LOG_DBG("Synchronization word detected (MDMSOFT interrupt flag)");
	}
	if (event & RF_EventCmdCancelled) {
		LOG_DBG("Command canceled");
	}
	if (event & RF_EventCmdAborted) {
		LOG_DBG("Command aborted");
	}
	if (event & RF_EventCmdStopped) {
		LOG_DBG("Command stopped");
	}
	if (event & RF_EventRatCh) {
		LOG_DBG("User RAT channel event");
	}
	if (event & RF_EventError) {
		LOG_DBG("Event error - see RF_Params::pErrCb");
	}
	if (event & RF_EventCmdPreempted) {
		LOG_DBG("Command preempted");
	}
}