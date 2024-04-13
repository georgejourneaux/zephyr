/*
 * Copyright (c) 2016 - 2020 Nordic Semiconductor ASA
 * Copyright (c) 2016 Vinayak Kariappa Chettimada
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>
#include <zephyr/dt-bindings/gpio/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <soc.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/power/PowerCC26X2.h>
#include <driverlib/rf_data_entry.h>
#include <driverlib/aon_rtc.h>
#include <driverlib/osc.h>
#include <driverlib/prcm.h>
#include <driverlib/rf_mailbox.h>

#include <driverlib/rf_ble_mailbox.h>
#include <driverlib/rfc.h>
#include <rf_patches/rf_patch_cpe_multi_protocol.h>
#include <inc/hw_ccfg.h>
#include <inc/hw_fcfg1.h>

#include "util/mem.h"

#include "hal/cpu.h"
#include "hal/ccm.h"
#include "hal/cntr.h"
#include "hal/debug.h"
#include "hal/radio.h"
#include "hal/radio_df.h"
#include "hal/swi.h"
#include "hal/ticker.h"
#include "hal/cc13xx_cc26xx/ll_irqs.h"

#include "lll/pdu_vendor.h"

#include "ll_sw/pdu_df.h"
#include "ll_sw/pdu.h"

#include "radio_internal.h"
#include "RFQueue.h"

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_ti_radio);

#define RF_RX_ADDITIONAL_DATA_BYTES                                                                \
	(RADIO_RX_CONFIG_INCLUDE_LEN_BYTE + RADIO_RX_CONFIG_INCLUDE_CRC +                          \
	 RADIO_RX_CONFIG_APPEND_RSSI + RADIO_RX_CONFIG_APPEND_STATUS +                             \
	 RADIO_RX_CONFIG_APPEND_TIMESTAMP)

#define RF_RX_ENTRY_BUFFER_SIZE (2)
#define RF_RX_BUFFER_SIZE       (HAL_RADIO_PDU_LEN_MAX + RF_RX_ADDITIONAL_DATA_BYTES)

#define RF_TX_ENTRY_BUFFER_SIZE (2)
#define RF_TX_BUFFER_SIZE       HAL_RADIO_PDU_LEN_MAX

#define RF_EVENT_ISR_MASK                                                                          \
	(RADIO_RF_EVENT_MASK_CMD_DONE | RADIO_RF_EVENT_MASK_TX_DONE | RADIO_RF_EVENT_MASK_RX_DONE)

#define TX_MARGIN    (0)
#define RX_MARGIN    (8)
#define Rx_OVHD      (32)  /* Rx overhead, depends on PHY type */
#define MIN_CMD_TIME (400) /* Minimum interval for a delayed radio cmd */

typedef enum BLE_CHANNEL_FREQUENCY {
	BLE_CHANNEL_FREQUENCY_2402 = 37,
	BLE_CHANNEL_FREQUENCY_2404 = 0,
	BLE_CHANNEL_FREQUENCY_2406 = 1,
	BLE_CHANNEL_FREQUENCY_2408 = 2,
	BLE_CHANNEL_FREQUENCY_2410 = 3,
	BLE_CHANNEL_FREQUENCY_2412 = 4,
	BLE_CHANNEL_FREQUENCY_2414 = 5,
	BLE_CHANNEL_FREQUENCY_2416 = 6,
	BLE_CHANNEL_FREQUENCY_2418 = 7,
	BLE_CHANNEL_FREQUENCY_2420 = 8,
	BLE_CHANNEL_FREQUENCY_2422 = 9,
	BLE_CHANNEL_FREQUENCY_2424 = 10,
	BLE_CHANNEL_FREQUENCY_2426 = 38,
	BLE_CHANNEL_FREQUENCY_2428 = 11,
	BLE_CHANNEL_FREQUENCY_2430 = 12,
	BLE_CHANNEL_FREQUENCY_2432 = 13,
	BLE_CHANNEL_FREQUENCY_2434 = 14,
	BLE_CHANNEL_FREQUENCY_2436 = 15,
	BLE_CHANNEL_FREQUENCY_2438 = 16,
	BLE_CHANNEL_FREQUENCY_2440 = 17,
	BLE_CHANNEL_FREQUENCY_2442 = 18,
	BLE_CHANNEL_FREQUENCY_2444 = 19,
	BLE_CHANNEL_FREQUENCY_2446 = 20,
	BLE_CHANNEL_FREQUENCY_2448 = 21,
	BLE_CHANNEL_FREQUENCY_2450 = 22,
	BLE_CHANNEL_FREQUENCY_2452 = 23,
	BLE_CHANNEL_FREQUENCY_2454 = 24,
	BLE_CHANNEL_FREQUENCY_2456 = 25,
	BLE_CHANNEL_FREQUENCY_2458 = 26,
	BLE_CHANNEL_FREQUENCY_2460 = 27,
	BLE_CHANNEL_FREQUENCY_2462 = 28,
	BLE_CHANNEL_FREQUENCY_2464 = 29,
	BLE_CHANNEL_FREQUENCY_2466 = 30,
	BLE_CHANNEL_FREQUENCY_2468 = 31,
	BLE_CHANNEL_FREQUENCY_2470 = 32,
	BLE_CHANNEL_FREQUENCY_2472 = 33,
	BLE_CHANNEL_FREQUENCY_2474 = 34,
	BLE_CHANNEL_FREQUENCY_2476 = 35,
	BLE_CHANNEL_FREQUENCY_2478 = 36,
	BLE_CHANNEL_FREQUENCY_2480 = 39,

	BLE_FREQUENCY_TABLE_SIZE,
} ble_frequency_table_entry_t;

typedef struct RF_RX_DATA {
	dataQueue_t queue;
	uint8_t buffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(RF_RX_ENTRY_BUFFER_SIZE, RF_RX_BUFFER_SIZE)]
		__attribute__((aligned(4)));
} rf_rx_data_t;
typedef struct rf_tx_data {
	dataQueue_t queue;
	uint8_t buffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(RF_TX_ENTRY_BUFFER_SIZE, RF_TX_BUFFER_SIZE)]
		__attribute__((aligned(4)));
} rf_tx_data_t;

typedef struct ble_cc13xx_cc26xx_rf_rat {
	RF_RatConfigCompare hcto_compare;
	RF_RatHandle hcto_handle;
} ble_cc13xx_cc26xx_rf_rat_t;

typedef struct ble_cc13xx_cc26xx_rf_cmd {
	RF_CmdHandle active_handle;

	rfc_CMD_NOP_t nop;
	rfc_CMD_FS_t fs;
	rfc_CMD_CLEAR_RX_t clear_rx;

	rfc_CMD_BLE5_RADIO_SETUP_t ble5_radio_setup;

	rfc_bleGenericRxPar_t _ble_generic_rx_param;
	rfc_bleGenericRxOutput_t _ble_generic_rx_output;
	rfc_CMD_BLE5_GENERIC_RX_t ble5_generic_rx;

	rfc_ble5SlavePar_t _ble5_slave_param;
	rfc_bleMasterSlaveOutput_t _ble_slave_output;
	rfc_CMD_BLE5_SLAVE_t ble5_slave;
} ble_cc13xx_cc26xx_rf_cmd_t;

typedef struct ble_cc13xx_cc26xx_rf {
	RF_Op *op;
	rf_rx_data_t rx;
	rf_tx_data_t tx;
	ble_cc13xx_cc26xx_rf_rat_t rat;
	ble_cc13xx_cc26xx_rf_cmd_t cmd;
} ble_cc13xx_cc26xx_rf_t;

typedef struct ble_cc13xx_cc26xx_data {
	uint32_t access_address;
	uint32_t polynomial;
	uint32_t iv;
	uint16_t channel;

	uint8_t *lll_rx_pdu;

	ble_cc13xx_cc26xx_rf_t rf;
} ble_cc13xx_cc26xx_data_t;

typedef struct cc13xx_cc26xx_frequency_table_entry {
	const uint16_t frequency;
	const uint8_t whitening;
} cc13xx_cc26xx_frequency_table_entry_t;

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

static RF_Object rfObject;
static RF_Handle rfBleHandle;
static RF_Mode RF_modeBle = {
	.rfMode = RF_MODE_AUTO,
	.cpePatchFxn = &rf_patch_cpe_multi_protocol,
	.mcePatchFxn = 0,
	.rfePatchFxn = 0,
};

// Overrides for CMD_BLE5_RADIO_SETUP
uint32_t pOverrides_bleCommon[] = {
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
uint32_t pOverrides_ble1Mbps[] = {
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
uint32_t pOverrides_ble2Mbps[] = {
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
uint32_t pOverrides_bleCoded[] = {
	// Bluetooth 5: Set pilot tone length to 20 us
	HW_REG_OVERRIDE(0x5320, 0x03C0),
	// Bluetooth 5: Compensate syncTimeadjust
	(uint32_t)0x07A902A3,
	// Rx: Set AGC reference level to 0x21 (default: 0x2E)
	HW_REG_OVERRIDE(0x609C, 0x0021), (uint32_t)0xFFFFFFFF};

static const cc13xx_cc26xx_frequency_table_entry_t frequency_table[BLE_FREQUENCY_TABLE_SIZE] = {
	[BLE_CHANNEL_FREQUENCY_2402] = {.frequency = 2402, .whitening = 0xE5},
	[BLE_CHANNEL_FREQUENCY_2404] = {.frequency = 2404, .whitening = 0xC0},
	[BLE_CHANNEL_FREQUENCY_2406] = {.frequency = 2406, .whitening = 0xC1},
	[BLE_CHANNEL_FREQUENCY_2408] = {.frequency = 2408, .whitening = 0xC2},
	[BLE_CHANNEL_FREQUENCY_2410] = {.frequency = 2410, .whitening = 0xC3},
	[BLE_CHANNEL_FREQUENCY_2412] = {.frequency = 2412, .whitening = 0xC4},
	[BLE_CHANNEL_FREQUENCY_2414] = {.frequency = 2414, .whitening = 0xC5},
	[BLE_CHANNEL_FREQUENCY_2416] = {.frequency = 2416, .whitening = 0xC6},
	[BLE_CHANNEL_FREQUENCY_2418] = {.frequency = 2418, .whitening = 0xC7},
	[BLE_CHANNEL_FREQUENCY_2420] = {.frequency = 2420, .whitening = 0xC8},
	[BLE_CHANNEL_FREQUENCY_2422] = {.frequency = 2422, .whitening = 0xC9},
	[BLE_CHANNEL_FREQUENCY_2424] = {.frequency = 2424, .whitening = 0xCA},
	[BLE_CHANNEL_FREQUENCY_2426] = {.frequency = 2426, .whitening = 0xE6},
	[BLE_CHANNEL_FREQUENCY_2428] = {.frequency = 2428, .whitening = 0xCB},
	[BLE_CHANNEL_FREQUENCY_2430] = {.frequency = 2430, .whitening = 0xCC},
	[BLE_CHANNEL_FREQUENCY_2432] = {.frequency = 2432, .whitening = 0xCD},
	[BLE_CHANNEL_FREQUENCY_2434] = {.frequency = 2434, .whitening = 0xCE},
	[BLE_CHANNEL_FREQUENCY_2436] = {.frequency = 2436, .whitening = 0xCF},
	[BLE_CHANNEL_FREQUENCY_2438] = {.frequency = 2438, .whitening = 0xD0},
	[BLE_CHANNEL_FREQUENCY_2440] = {.frequency = 2440, .whitening = 0xD1},
	[BLE_CHANNEL_FREQUENCY_2442] = {.frequency = 2442, .whitening = 0xD2},
	[BLE_CHANNEL_FREQUENCY_2444] = {.frequency = 2444, .whitening = 0xD3},
	[BLE_CHANNEL_FREQUENCY_2446] = {.frequency = 2446, .whitening = 0xD4},
	[BLE_CHANNEL_FREQUENCY_2448] = {.frequency = 2448, .whitening = 0xD5},
	[BLE_CHANNEL_FREQUENCY_2450] = {.frequency = 2450, .whitening = 0xD6},
	[BLE_CHANNEL_FREQUENCY_2452] = {.frequency = 2452, .whitening = 0xD7},
	[BLE_CHANNEL_FREQUENCY_2454] = {.frequency = 2454, .whitening = 0xD8},
	[BLE_CHANNEL_FREQUENCY_2456] = {.frequency = 2456, .whitening = 0xD9},
	[BLE_CHANNEL_FREQUENCY_2458] = {.frequency = 2458, .whitening = 0xDA},
	[BLE_CHANNEL_FREQUENCY_2460] = {.frequency = 2460, .whitening = 0xDB},
	[BLE_CHANNEL_FREQUENCY_2462] = {.frequency = 2462, .whitening = 0xDC},
	[BLE_CHANNEL_FREQUENCY_2464] = {.frequency = 2464, .whitening = 0xDD},
	[BLE_CHANNEL_FREQUENCY_2466] = {.frequency = 2466, .whitening = 0xDE},
	[BLE_CHANNEL_FREQUENCY_2468] = {.frequency = 2468, .whitening = 0xDF},
	[BLE_CHANNEL_FREQUENCY_2470] = {.frequency = 2470, .whitening = 0xE0},
	[BLE_CHANNEL_FREQUENCY_2472] = {.frequency = 2472, .whitening = 0xE1},
	[BLE_CHANNEL_FREQUENCY_2474] = {.frequency = 2474, .whitening = 0xE2},
	[BLE_CHANNEL_FREQUENCY_2476] = {.frequency = 2476, .whitening = 0xE3},
	[BLE_CHANNEL_FREQUENCY_2478] = {.frequency = 2478, .whitening = 0xE4},
	[BLE_CHANNEL_FREQUENCY_2480] = {.frequency = 2480, .whitening = 0xE7}};

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

static uint32_t timer_aa = 0;       /* AA (Access Address) timestamp saved value */
static uint32_t timer_aa_save = 0;  /* save AA timestamp */
static uint32_t timer_ready = 0;    /* radio ready for Tx/Rx timestamp */
static uint32_t timer_end = 0;      /* Tx/Rx end timestamp saved value */
static uint32_t timer_end_save = 0; /* save Tx/Rx end timestamp */
static uint32_t timer_tifs = 0;

static uint32_t rtc_start;
static uint32_t rtc_diff_start_us;
static uint32_t skip_hcto;

static uint32_t isr_timer_aa = 0;
static uint32_t isr_timer_end = 0;
static uint32_t isr_latency = 0;

static int8_t rssi;

static uint32_t rx_warmup = 0;
static uint32_t tx_warmup = 0;
static uint32_t next_warmup;

static uint32_t radio_trx = 0;

static radio_isr_cb_t radio_isr_cb;
static void *radio_isr_cb_param;

static bool crc_valid = true;

static uint8_t MALIGN(4) _pkt_empty[PDU_EM_LL_SIZE_MAX];
static uint8_t MALIGN(4) _pkt_scratch[MAX((HAL_RADIO_PDU_LEN_MAX + 3), PDU_AC_LL_SIZE_MAX)];

static ble_cc13xx_cc26xx_data_t ble_cc13xx_cc26xx_data = {
	.access_address = 0xFFFFFFFF,
	.polynomial = 0,
	.iv = 0,
	.channel = 0,

	.rf.rx = {.queue = {NULL}, .buffer = {0}},
	.rf.tx = {.queue = {NULL}, .buffer = {0}},
	.rf.rat =
		{
			.hcto_compare = {0},
			.hcto_handle = 0,
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
				     .defaultPhy.mainMode = 0,
				     .defaultPhy.coding = 0,
				     .loDivider = 0,
				     .config.frontEndMode = 0,
				     .config.biasMode = 1,
				     .config.analogCfgMode = 0,
				     .config.bNoFsPowerUp = 0,
				     .config.bSynthNarrowBand = 0,
				     .txPower = 0x7217,
				     .pRegOverrideCommon = pOverrides_bleCommon,
				     .pRegOverride1Mbps = pOverrides_ble1Mbps,
				     .pRegOverride2Mbps = pOverrides_ble2Mbps,
				     .pRegOverrideCoded = pOverrides_bleCoded},

		.ble5_generic_rx =
			{
				.commandNo = CMD_BLE5_GENERIC_RX,
				.status = IDLE,
				.pNextOp = NULL,
				.startTime = 0,
				.startTrigger.triggerType = TRIG_NOW,
				.startTrigger.bEnaCmd = 0,
				.startTrigger.triggerNo = 0,
				.startTrigger.pastTrig = 0,
				.condition.rule = COND_NEVER,
				.condition.nSkip = COND_ALWAYS,
				.channel = 0,
				.whitening.init = 0,
				.whitening.bOverride = 0,
				.phyMode.mainMode = 0,
				.phyMode.coding = 0,
				.rangeDelay = 0,
				.txPower = 0,
				.pParams = &ble_cc13xx_cc26xx_data.rf.cmd._ble_generic_rx_param,
				.pOutput = &ble_cc13xx_cc26xx_data.rf.cmd._ble_generic_rx_output,
				.tx20Power = 0,
			},
		._ble_generic_rx_param =
			{
				.pRxQ = NULL,
				.rxConfig.bAutoFlushIgnored = RADIO_RX_CONFIG_AUTO_FLUSH_IGNORED,
				.rxConfig.bAutoFlushCrcErr = RADIO_RX_CONFIG_AUTO_FLUSH_CRC_ERR,
				.rxConfig.bAutoFlushEmpty = RADIO_RX_CONFIG_AUTO_FLUSH_EMPTY,
				.rxConfig.bIncludeLenByte = RADIO_RX_CONFIG_INCLUDE_LEN_BYTE,
				.rxConfig.bIncludeCrc = RADIO_RX_CONFIG_INCLUDE_CRC,
				.rxConfig.bAppendRssi = RADIO_RX_CONFIG_APPEND_RSSI,
				.rxConfig.bAppendStatus = RADIO_RX_CONFIG_APPEND_STATUS,
				.rxConfig.bAppendTimestamp = RADIO_RX_CONFIG_APPEND_TIMESTAMP,
				.bRepeat = 0,
				.__dummy0 = 0,
				.accessAddress = 0,
				.crcInit0 = 0,
				.crcInit1 = 0,
				.crcInit2 = 0,
				.endTrigger.triggerType = TRIG_NEVER,
				.endTrigger.bEnaCmd = 0,
				.endTrigger.triggerNo = 0,
				.endTrigger.pastTrig = 0,
				.endTime = 0,
			},
		._ble_generic_rx_output = {0},

		.ble5_slave =
			{
				.commandNo = CMD_BLE5_SLAVE,
				.status = IDLE,
				.pNextOp = NULL,
				.startTime = 0,
				.startTrigger.triggerType = TRIG_NOW,
				.startTrigger.bEnaCmd = 0,
				.startTrigger.triggerNo = 0,
				.startTrigger.pastTrig = 0,
				.condition.rule = COND_NEVER,
				.condition.nSkip = COND_ALWAYS,
				.channel = 0,
				.whitening.init = 0,
				.whitening.bOverride = 0,
				.phyMode.mainMode = 0,
				.phyMode.coding = 0,
				.rangeDelay = 0,
				.txPower = 0,
				.pParams = &ble_cc13xx_cc26xx_data.rf.cmd._ble5_slave_param,
				.pOutput = &ble_cc13xx_cc26xx_data.rf.cmd._ble_slave_output,
				.tx20Power = 0,
			},
		._ble5_slave_param =
			{
				.pRxQ = NULL,
				.pTxQ = &ble_cc13xx_cc26xx_data.rf.tx.queue,
				.rxConfig.bAutoFlushIgnored = RADIO_RX_CONFIG_AUTO_FLUSH_IGNORED,
				.rxConfig.bAutoFlushCrcErr = RADIO_RX_CONFIG_AUTO_FLUSH_CRC_ERR,
				.rxConfig.bAutoFlushEmpty = RADIO_RX_CONFIG_AUTO_FLUSH_EMPTY,
				.rxConfig.bIncludeLenByte = RADIO_RX_CONFIG_INCLUDE_LEN_BYTE,
				.rxConfig.bIncludeCrc = RADIO_RX_CONFIG_INCLUDE_CRC,
				.rxConfig.bAppendRssi = RADIO_RX_CONFIG_APPEND_RSSI,
				.rxConfig.bAppendStatus = RADIO_RX_CONFIG_APPEND_STATUS,
				.rxConfig.bAppendTimestamp = RADIO_RX_CONFIG_APPEND_TIMESTAMP,
				.seqStat.lastRxSn = 0,
				.seqStat.lastTxSn = 0,
				.seqStat.nextTxSn = 0,
				.seqStat.bFirstPkt = 1,
				.seqStat.bAutoEmpty = 0,
				.seqStat.bLlCtrlTx = 0,
				.seqStat.bLlCtrlAckRx = 0,
				.seqStat.bLlCtrlAckPending = 0,
				.maxNack = 0,
				.maxPkt = 0,
				.accessAddress = 0,
				.crcInit0 = 0,
				.crcInit1 = 0,
				.crcInit2 = 0,
				.timeoutTrigger.triggerType = TRIG_REL_START,
				.timeoutTrigger.bEnaCmd = 0,
				.timeoutTrigger.triggerNo = 0,
				.timeoutTrigger.pastTrig = 0,
				.timeoutTime = 0,
				.maxRxPktLen = 0,
				.maxLenLowRate = 0,
				.__dummy0 = 0,
				.endTrigger.triggerType = TRIG_NEVER,
				.endTrigger.bEnaCmd = 0,
				.endTrigger.triggerNo = 0,
				.endTrigger.pastTrig = 0,
			},
		._ble_slave_output = {0},
	}};
static ble_cc13xx_cc26xx_data_t *driver_data = &ble_cc13xx_cc26xx_data;

static const char *const commandNo_to_string(RF_CmdHandle commandNo)
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

static const char *const ble_status_to_string(uint16_t status)
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

static void dbg_event(RF_EventMask event)
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

static inline bool pdu_is_adv(uint32_t access_address)
{
	return (access_address == PDU_AC_ACCESS_ADDR);
}

static void isr_radio(RF_Handle handle, RF_CmdHandle command_handle, RF_EventMask event_mask)
{
	DEBUG_RADIO_ISR(1);

	RF_Op *rf_op = RF_getCmdOp(rfBleHandle, command_handle);
	LOG_DBG("%s (0x%04X) %s (0x%04X) event %llu", commandNo_to_string(rf_op->commandNo),
		rf_op->commandNo, ble_status_to_string(rf_op->status), rf_op->status, event_mask);
	dbg_event(event_mask);

	if (event_mask & RADIO_RF_EVENT_MASK_TX_DONE) {
		if (timer_end_save) {
			timer_end = isr_timer_end;
		}
	}

	if (event_mask & RADIO_RF_EVENT_MASK_RX_DONE) {
		/* Disable Rx timeout */
		RF_ratDisableChannel(rfBleHandle, driver_data->rf.rat.hcto_handle);

		rfc_dataEntryPointer_t *rx_entry =
			(rfc_dataEntryPointer_t *)driver_data->rf.rx.queue.pCurrEntry;

		size_t offs = rx_entry->pData[0];
		uint8_t *data = &rx_entry->pData[1];

		ratmr_t timestamp = 0;
		timestamp |= data[--offs] << 24;
		timestamp |= data[--offs] << 16;
		timestamp |= data[--offs] << 8;
		timestamp |= data[--offs] << 0;

		rssi = (int8_t)data[--offs];

		uint32_t crc = 0;
		crc |= data[--offs] << 16;
		crc |= data[--offs] << 8;
		crc |= data[--offs] << 0;
		crc_valid = true;

		size_t len = offs + 1;

		rtc_start = timestamp;

		/* Add to AA time, PDU + CRC time */
		isr_timer_end = rtc_start + HAL_TICKER_US_TO_TICKS(len + sizeof(crc - 1));

		if (timer_aa_save) {
			timer_aa = isr_timer_aa;
		}

		if (timer_end_save) {
			timer_end = isr_timer_end; /* from pkt_rx() */
		}
	}

	if (event_mask & RADIO_RF_EVENT_MASK_CMD_DONE) {
		/* Disable both comparators */
		RF_ratDisableChannel(rfBleHandle, driver_data->rf.rat.hcto_handle);
	}

	radio_isr_cb_rf_param_t rf_param = {
		.handle = handle,
		.command_handle = command_handle,
		.event_mask = event_mask,
	};
	radio_isr_cb(radio_isr_cb_param, rf_param);

	DEBUG_RADIO_ISR(0);
}

void radio_isr_set(radio_isr_cb_t cb, void *param)
{
	irq_disable(LL_RADIO_IRQn);

	radio_isr_cb_param = param;
	radio_isr_cb = cb;

	/* Clear pending interrupts */
	ClearPendingIRQ(LL_RADIO_IRQn);

	irq_enable(LL_RADIO_IRQn);
}

static void rat_deferred_hcto_callback(RF_Handle h, RF_RatHandle rh, RF_EventMask e,
				       uint32_t compareCaptureTime)
{
	LOG_DBG("cntr %u (%uus)", cntr_cnt_get(), HAL_TICKER_TICKS_TO_US(cntr_cnt_get()));
	RF_cancelCmd(rfBleHandle, driver_data->rf.cmd.active_handle, RF_ABORT_GRACEFULLY);
	driver_data->rf.cmd.active_handle = -1;
}

static void ble_cc13xx_cc26xx_data_init(void)
{
	RFQueue_defineQueue(&driver_data->rf.rx.queue, driver_data->rf.rx.buffer,
			    sizeof(driver_data->rf.rx.buffer), RF_RX_ENTRY_BUFFER_SIZE,
			    RF_RX_BUFFER_SIZE);

	RFQueue_defineQueue(&driver_data->rf.tx.queue, driver_data->rf.tx.buffer,
			    sizeof(driver_data->rf.tx.buffer), RF_TX_ENTRY_BUFFER_SIZE,
			    RF_TX_BUFFER_SIZE);

	RF_RatConfigCompare_init((RF_RatConfigCompare *)&driver_data->rf.rat.hcto_compare);
	driver_data->rf.rat.hcto_compare.callback = rat_deferred_hcto_callback;
}

/*
 * A (high) arbitrarily-chosen 32-bit number that the RAT will (likely) not
 * encounter soon after reset
 */
#define ISR_LATENCY_MAGIC 0xfedcba98
static void isr_latency_cb(void *param, radio_isr_cb_rf_param_t rf_param)
{
	ARG_UNUSED(rf_param);

	uint32_t t1 = *(uint32_t *)param;
	uint32_t t2 = HAL_TICKER_TICKS_TO_US(cntr_cnt_get());

	isr_latency = t2 - t1;
	/* Mark as done */
	*(uint32_t *)param = ISR_LATENCY_MAGIC;
}

static void get_isr_latency(void)
{
	volatile uint32_t isr_latency_param;

	radio_isr_set(isr_latency_cb, (void *)&isr_latency_param);

	/* Reset TMR to zero */
	/* (not necessary using the ISR_LATENCY_MAGIC approach) */

	isr_latency_param = HAL_TICKER_TICKS_TO_US(cntr_cnt_get());

	radio_disable();

	while (isr_latency_param != ISR_LATENCY_MAGIC) {
	}

	irq_disable(LL_RADIO_IRQn);
}

void radio_setup(void)
{
	RF_Params rfBleParams;
	RF_Params_init(&rfBleParams);

	ble_cc13xx_cc26xx_data_init();

	rfBleHandle = RF_open(&rfObject, &RF_modeBle,
			      (RF_RadioSetup *)&driver_data->rf.cmd.ble5_radio_setup, &rfBleParams);
	LL_ASSERT(rfBleHandle);

	driver_data->rf.cmd.fs.frequency =
		frequency_table[ble_cc13xx_cc26xx_data.channel].frequency;
	RF_runCmd(rfBleHandle, (RF_Op *)&driver_data->rf.cmd.fs, RF_PriorityNormal, NULL,
		  RF_EventLastCmdDone);

	/* Get warmpup times. They are used in TIFS calculations */
	tx_warmup = 0; /* us */
	rx_warmup = 0; /* us */
	get_isr_latency();
}

void radio_reset(void)
{
	irq_disable(LL_RADIO_IRQn);

#if defined(CONFIG_BT_CTLR_DF) && !defined(CONFIG_ZTEST)
	radio_df_reset();
#endif /* CONFIG_BT_CTLR_DF && !CONFIG_ZTEST */

	// hal_radio_reset();
}

void radio_stop(void)
{
	// hal_radio_stop();
}

void radio_phy_set(uint8_t phy, uint8_t flags)
{
	ARG_UNUSED(phy);
	ARG_UNUSED(flags);

	/* This function should set one of three modes:
	 * - BLE 1 Mbps
	 * - BLE 2 Mbps
	 * - Coded BLE
	 * We set this on radio_setup() function. There radio is
	 * setup for DataRate of 1 Mbps.
	 * For now this function does nothing. In the future it
	 * may have to reset the radio
	 * to the 2 Mbps (the only other mode supported by Vega radio).
	 */
}

void radio_tx_power_set(int8_t power)
{
	LOG_DBG("%u", power);
	RF_setTxPower(rfBleHandle, RF_TxPowerTable_findValue(
					   (RF_TxPowerTable_Entry *)RF_BLE_txPowerTable, power));
}

void radio_tx_power_max_set(void)
{
	radio_tx_power_set(RF_TxPowerTable_MAX_DBM);
}

void radio_freq_chan_set(uint32_t chan)
{
	LOG_DBG("%u", chan);
	LL_ASSERT(chan < BLE_FREQUENCY_TABLE_SIZE);

	driver_data->channel = chan;
}

void radio_whiten_iv_set(uint32_t iv)
{
	/* do nothing */
}

void radio_aa_set(const uint8_t *aa)
{
	driver_data->access_address = *(uint32_t *)aa;
}

void radio_pkt_configure(uint8_t bits_len, uint8_t max_len, uint8_t flags)
{
}

void radio_pkt_rx_set(void *rx_packet)
{
	driver_data->lll_rx_pdu = (uint8_t *)rx_packet;
}

void radio_pkt_tx_set(RF_Op *tx_packet)
{
	driver_data->rf.op = tx_packet;
}

uint32_t radio_tx_ready_delay_get(uint8_t phy, uint8_t flags)
{
	return tx_warmup;
}

uint32_t radio_tx_chain_delay_get(uint8_t phy, uint8_t flags)
{
	return 0;
}

uint32_t radio_rx_ready_delay_get(uint8_t phy, uint8_t flags)
{
	return rx_warmup;
}

uint32_t radio_rx_chain_delay_get(uint8_t phy, uint8_t flags)
{
	return 16 + 2 * Rx_OVHD + RX_MARGIN + isr_latency + Rx_OVHD;
}

void radio_rx_enable(void)
{
	radio_tmr_start_now(RADIO_TRX_RX);
}

void radio_tx_enable(void)
{
	radio_tmr_start_now(RADIO_TRX_TX);
}

void radio_disable(void)
{
	LOG_DBG("cntr %u (%uus)", cntr_cnt_get(), HAL_TICKER_TICKS_TO_US(cntr_cnt_get()));

	RF_runDirectCmd(rfBleHandle, CMD_ABORT);
	RF_runImmediateCmd(rfBleHandle, (uint32_t *)&driver_data->rf.cmd.clear_rx);

	/* generate interrupt to get into isr_radio */
	RF_postCmd(rfBleHandle, (RF_Op *)&driver_data->rf.cmd.nop, RF_PriorityNormal, isr_radio,
		   RF_EventLastCmdDone);

	driver_data->rf.op = NULL;
}

void radio_status_reset(void)
{
	radio_trx = 0;
}

uint32_t radio_is_ready(void)
{
	/* Always false. LLL expects the radio not to be in idle/Tx/Rx state */
	return 0;
}

uint32_t radio_is_done(void)
{
	return radio_trx;
}

uint32_t radio_has_disabled(void)
{
	/* Not used */
	return 0;
}

uint32_t radio_is_idle(void)
{
	return 1;
}

void radio_crc_configure(uint32_t polynomial, uint32_t iv)
{
	driver_data->polynomial = polynomial;
	driver_data->iv = iv;
}

uint32_t radio_crc_is_valid(void)
{
	bool r = crc_valid;

	/* only valid for first call */
	crc_valid = false;

	return r;
}

void *radio_pkt_empty_get(void)
{
	return _pkt_empty;
}

void *radio_pkt_scratch_get(void)
{
	return _pkt_scratch;
}

#if defined(CONFIG_BT_CTLR_LE_ENC) && defined(HAL_RADIO_PDU_LEN_MAX) &&                            \
	(!defined(CONFIG_BT_CTLR_DATA_LENGTH_MAX) ||                                               \
	 (CONFIG_BT_CTLR_DATA_LENGTH_MAX < (HAL_RADIO_PDU_LEN_MAX - 4)))
static uint8_t MALIGN(4) _pkt_decrypt[MAX((HAL_RADIO_PDU_LEN_MAX + 3), PDU_AC_LL_SIZE_MAX)];

void *radio_pkt_decrypt_get(void)
{
	return _pkt_decrypt;
}
#elif !defined(HAL_RADIO_PDU_LEN_MAX)
#error "Undefined HAL_RADIO_PDU_LEN_MAX."
#endif

#if defined(CONFIG_BT_CTLR_ADV_ISO) || defined(CONFIG_BT_CTLR_SYNC_ISO)
/* Dedicated Rx PDU Buffer for Control PDU independent of node_rx with BIS Data
 * PDU buffer. Note this buffer will be used to store whole PDUs, not just the BIG control payload.
 */
static uint8_t pkt_big_ctrl[offsetof(struct pdu_bis, payload) + sizeof(struct pdu_big_ctrl)];

void *radio_pkt_big_ctrl_get(void)
{
	return pkt_big_ctrl;
}
#endif /* CONFIG_BT_CTLR_ADV_ISO || CONFIG_BT_CTLR_SYNC_ISO */

void radio_switch_complete_and_rx(uint8_t phy_rx)
{
	/* the margin is used to account for any overhead in radio switching */
	next_warmup = rx_warmup + RX_MARGIN;
}

void radio_switch_complete_and_tx(uint8_t phy_rx, uint8_t flags_rx, uint8_t phy_tx,
				  uint8_t flags_tx)
{
	/* the margin is used to account for any overhead in radio switching */
	next_warmup = tx_warmup + TX_MARGIN;
}

void radio_switch_complete_and_disable(void)
{
	RF_ratDisableChannel(rfBleHandle, driver_data->rf.rat.hcto_handle);
	driver_data->rf.op = NULL;
}

void radio_rssi_measure(void)
{
	rssi = RF_getRssi(rfBleHandle);
}

uint32_t radio_rssi_get(void)
{
	return (uint32_t)-rssi;
}

void radio_rssi_status_reset(void)
{
	rssi = RF_GET_RSSI_ERROR_VAL;
}

uint32_t radio_rssi_is_ready(void)
{
	return (rssi != RF_GET_RSSI_ERROR_VAL);
}

void radio_filter_configure(uint8_t bitmask_enable, uint8_t bitmask_addr_type, uint8_t *bdaddr)
{
}

void radio_filter_disable(void)
{
}

void radio_filter_status_reset(void)
{
}

uint32_t radio_filter_has_match(void)
{
	return true;
}

uint32_t radio_filter_match_get(void)
{
	return 0;
}

void radio_bc_configure(uint32_t n)
{
}

void radio_bc_status_reset(void)
{
}

uint32_t radio_bc_has_match(void)
{
	return 0;
}

void radio_tmr_status_reset(void)
{
	timer_aa_save = 0;
	timer_end_save = 0;
}

void radio_tmr_tx_status_reset(void)
{
}

void radio_tmr_rx_status_reset(void)
{
}

void radio_tmr_tx_enable(void)
{
}

void radio_tmr_rx_enable(void)
{
}

void radio_tmr_tx_disable(void)
{
}

void radio_tmr_rx_disable(void)
{
}

void radio_tmr_tifs_set(uint32_t tifs)
{
	timer_tifs = tifs;
}

/* Start the radio after ticks_start (ticks) + remainder (us) time */
static uint32_t radio_tmr_start_hlp(uint8_t trx, uint32_t ticks_start, uint32_t remainder)
{
	uint32_t now = cntr_cnt_get();

	/* Save it for later */
	// rtc_start = ticks_start;

	/* Convert ticks to us and use just EVENT_TMR */
	rtc_diff_start_us = HAL_TICKER_TICKS_TO_US(rtc_start - now);

	skip_hcto = 0;
	if (rtc_diff_start_us > 0x80000000) {
		LOG_DBG("dropped command");
		/* ticks_start already passed. Don't start the radio */
		rtc_diff_start_us = 0;

		/* Ignore time out as well */
		skip_hcto = 1;
		remainder = 0;
		return remainder;
	}

	if (trx == RADIO_TRX_TX) {
		if (remainder <= MIN_CMD_TIME) {
			remainder = 0;
		}

		timer_ready = remainder + tx_warmup;

	} else {
		if (remainder <= MIN_CMD_TIME) {
			remainder = 0;
		}
		timer_ready = remainder + rx_warmup;
	}

	if (driver_data->rf.op == NULL) {
		LOG_DBG("no cmd");
		return remainder;
	}

	/* Add function to check op type - allow any RF_Op? */
	rfc_ble5RadioOp_t *ble_radio_op = (rfc_ble5RadioOp_t *)driver_data->rf.op;
	ble_radio_op->channel = driver_data->channel;
	ble_radio_op->startTime = now + remainder;
	ble_radio_op->startTrigger.triggerType = TRIG_ABSTIME;
	ble_radio_op->startTrigger.pastTrig = true;

	LOG_DBG("%s (0x%04X) in %u ch %u", commandNo_to_string(ble_radio_op->commandNo),
		ble_radio_op->commandNo, (ble_radio_op->startTime - now), ble_radio_op->channel);

	driver_data->rf.cmd.active_handle = RF_postCmd(
		rfBleHandle, driver_data->rf.op, RF_PriorityNormal, isr_radio, RF_EVENT_ISR_MASK);

	return remainder;
}

uint32_t radio_tmr_start(uint8_t trx, uint32_t ticks_start, uint32_t remainder)
{
	if ((!(remainder / 1000000UL)) || (remainder & 0x80000000)) {
		ticks_start--;
		remainder += 30517578UL;
	}
	remainder /= 1000000UL;

	return radio_tmr_start_hlp(trx, ticks_start, remainder);
}

uint32_t radio_tmr_start_tick(uint8_t trx, uint32_t tick)
{
	/* Setup compare event with min. 1 us offset */
	uint32_t remainder_us = 1;

	return radio_tmr_start_hlp(trx, tick, remainder_us);
}

uint32_t radio_tmr_start_us(uint8_t trx, uint32_t start_us)
{
	/* Setup compare event with min. 1 us offset */
	uint32_t remainder_us = 1;

	return radio_tmr_start_hlp(trx, HAL_TICKER_US_TO_TICKS(start_us), remainder_us);
}

uint32_t radio_tmr_start_now(uint8_t trx)
{
	return radio_tmr_start(trx, cntr_cnt_get(), 0);
}

uint32_t radio_tmr_start_get(void)
{
	return rtc_start;
}

void radio_tmr_stop(void)
{
	/* Deep Sleep Mode (DSM)? */
}

void radio_tmr_hcto_configure(uint32_t hcto)
{
	if (skip_hcto) {
		skip_hcto = 0;
		return;
	}

	driver_data->rf.rat.hcto_compare.timeout = hcto;

	/* 0b1001..RX Stop @ T2 Timer Compare Match (EVENT_TMR = T2_CMP) */
	driver_data->rf.rat.hcto_handle =
		RF_ratCompare(rfBleHandle, &driver_data->rf.rat.hcto_compare, NULL);
}

void radio_tmr_aa_capture(void)
{
	timer_aa_save = 1;
}

uint32_t radio_tmr_aa_get(void)
{
	LOG_DBG("return tmr_aa (%u) - rtc_diff_start_us (%u) = %u", timer_aa, rtc_diff_start_us,
		timer_aa - rtc_diff_start_us);
	return timer_aa - rtc_diff_start_us;
}

static uint32_t radio_timer_aa;

void radio_tmr_aa_save(uint32_t aa)
{
	LOG_DBG("set radio_timer_aa = %u", aa);
	radio_timer_aa = aa;
}

uint32_t radio_tmr_aa_restore(void)
{
	LOG_DBG("return radio_timer_aa = %u", radio_timer_aa);
	return radio_timer_aa;
}

uint32_t radio_tmr_ready_get(void)
{
	LOG_DBG("return tmr_ready (%u) - rtc_diff_start_us (%u) = %u", timer_ready,
		rtc_diff_start_us, timer_ready - rtc_diff_start_us);
	return timer_ready - rtc_diff_start_us;
}

static uint32_t radio_timer_ready;

void radio_tmr_ready_save(uint32_t ready)
{
	radio_timer_ready = ready;
}

uint32_t radio_tmr_ready_restore(void)
{
	return radio_timer_ready;
}

void radio_tmr_end_capture(void)
{
	timer_end_save = 1;
}

uint32_t radio_tmr_end_get(void)
{
	return timer_end - rtc_start;
}

uint32_t radio_tmr_tifs_base_get(void)
{
	return radio_tmr_end_get();
}

#if defined(CONFIG_BT_CTLR_SW_SWITCH_SINGLE_TIMER)
static uint32_t timer_sample_val;
#endif /* CONFIG_BT_CTLR_SW_SWITCH_SINGLE_TIMER */

void radio_tmr_sample(void)
{
#if defined(CONFIG_BT_CTLR_SW_SWITCH_SINGLE_TIMER)

#else /* !CONFIG_BT_CTLR_SW_SWITCH_SINGLE_TIMER */

#endif /* !CONFIG_BT_CTLR_SW_SWITCH_SINGLE_TIMER */
}

uint32_t radio_tmr_sample_get(void)
{
#if defined(CONFIG_BT_CTLR_SW_SWITCH_SINGLE_TIMER)
	return timer_sample_val;
#else  /* !CONFIG_BT_CTLR_SW_SWITCH_SINGLE_TIMER */
	return 0;
#endif /* !CONFIG_BT_CTLR_SW_SWITCH_SINGLE_TIMER */
}

static void *radio_ccm_ext_rx_pkt_set(struct ccm *cnf, uint8_t phy, uint8_t pdu_type, void *pkt)
{
	return _pkt_scratch;
}

void *radio_ccm_rx_pkt_set(struct ccm *cnf, uint8_t phy, void *pkt)
{
	return radio_ccm_ext_rx_pkt_set(cnf, phy, RADIO_PKT_CONF_PDU_TYPE_DC, pkt);
}

void *radio_ccm_iso_rx_pkt_set(struct ccm *cnf, uint8_t phy, uint8_t pdu_type, void *pkt)
{
	return radio_ccm_ext_rx_pkt_set(cnf, phy, pdu_type, pkt);
}

static void *radio_ccm_ext_tx_pkt_set(struct ccm *cnf, uint8_t pdu_type, void *pkt)
{
	return _pkt_scratch;
}

void *radio_ccm_tx_pkt_set(struct ccm *cnf, void *pkt)
{
	return radio_ccm_ext_tx_pkt_set(cnf, RADIO_PKT_CONF_PDU_TYPE_DC, pkt);
}

void *radio_ccm_iso_tx_pkt_set(struct ccm *cnf, uint8_t pdu_type, void *pkt)
{
	return radio_ccm_ext_tx_pkt_set(cnf, pdu_type, pkt);
}

uint32_t radio_ccm_is_done(void)
{
	return 0;
}

uint32_t radio_ccm_mic_is_valid(void)
{
	return 0;
}

#if defined(CONFIG_BT_CTLR_PRIVACY)
void radio_ar_configure(uint32_t nirk, void *irk, uint8_t flags)
{
}

uint32_t radio_ar_match_get(void)
{
	return 0;
}

void radio_ar_status_reset(void)
{
}

uint32_t radio_ar_has_match(void)
{
	return 0;
}

uint8_t radio_ar_resolve(const uint8_t *addr)
{
	return 0;
}
#endif /* CONFIG_BT_CTLR_PRIVACY */
