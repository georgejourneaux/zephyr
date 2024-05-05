/*
 * Copyright (c) 2018-2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

int lll_prepare_done(void *param);
int lll_done(void *param);
bool lll_is_done(void *param, bool *is_resume);
int lll_is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb);
void lll_abort_cb(struct lll_prepare_param *prepare_param, void *param);

uint32_t lll_event_offset_get(struct ull_hdr *ull);
uint32_t lll_preempt_calc(struct ull_hdr *ull, uint8_t ticker_id, uint32_t ticks_at_event);

void lll_isr_abort(RF_Handle rf_handle, RF_CmdHandle command_handle, RF_EventMask event_mask);
void lll_isr_done(RF_Handle rf_handle, RF_CmdHandle command_handle, RF_EventMask event_mask);
void lll_isr_cleanup(void *param);
void lll_isr_early_abort(RF_Handle rf_handle, RF_CmdHandle command_handle, RF_EventMask event_mask);

void lll_isr_peripheral(RF_Handle rf_handle, RF_CmdHandle command_handle, RF_EventMask event_mask);

dataQueue_t *lll_conn_get_rf_rx_queue(void);
dataQueue_t *lll_conn_get_rf_tx_queue(void);
