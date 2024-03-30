/* Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/hci_vs.h>

#include <soc.h>

#include <inc/hw_ccfg.h>
#include <inc/hw_fcfg1.h>
#include <inc/hw_memmap.h>

uint8_t hci_vendor_read_static_addr(struct bt_hci_vs_static_addr addrs[],
				 uint8_t size)
{
	/* only one supported */
	ARG_UNUSED(size);

	/* Read address from cc13xx_cc26xx-specific storage */
	uint32_t* mac;

    if (IS_ENABLED(CONFIG_HWINFO_CC13XX_CC26XX_ALWAYS_USE_FACTORY_DEFAULT) ||
        sys_read32(CCFG_BASE + CCFG_O_IEEE_BLE_0) == 0xFFFFFFFF ||
        sys_read32(CCFG_BASE + CCFG_O_IEEE_BLE_1) == 0xFFFFFFFF) {
        mac = (uint32_t*)(FCFG1_BASE + FCFG1_O_MAC_BLE_0);
    } else {
        mac = (uint32_t*)(CCFG_BASE + CCFG_O_IEEE_BLE_0);
    }

	sys_put_le32(mac[0], &addrs[0].bdaddr.val[0]);
	sys_put_le16((uint16_t)mac[1], &addrs[0].bdaddr.val[4]);

	BT_ADDR_SET_STATIC(&addrs[0].bdaddr);

	/* Mark IR as invalid */
	(void)memset(addrs[0].ir, 0x00, sizeof(addrs[0].ir));

	return 1;
}
