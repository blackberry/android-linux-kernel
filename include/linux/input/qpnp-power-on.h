/* Copyright (c) 2012-2015, 2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef QPNP_PON_H
#define QPNP_PON_H

#include <linux/errno.h>

/**
 * enum pon_trigger_source: List of PON trigger sources
 * %PON_SMPL:		PON triggered by SMPL - Sudden Momentary Power Loss
 * %PON_RTC:		PON triggered by RTC alarm
 * %PON_DC_CHG:		PON triggered by insertion of DC charger
 * %PON_USB_CHG:	PON triggered by insertion of USB
 * %PON_PON1:		PON triggered by other PMIC (multi-PMIC option)
 * %PON_CBLPWR_N:	PON triggered by power-cable insertion
 * %PON_KPDPWR_N:	PON triggered by long press of the power-key
 */
enum pon_trigger_source {
	PON_SMPL = 1,
	PON_RTC,
	PON_DC_CHG,
	PON_USB_CHG,
	PON_PON1,
	PON_CBLPWR_N,
	PON_KPDPWR_N,
};

/**
 * enum pon_power_off_type: Possible power off actions to perform
 * %PON_POWER_OFF_RESERVED:          Reserved, not used
 * %PON_POWER_OFF_WARM_RESET:        Reset the MSM but not all PMIC peripherals
 * %PON_POWER_OFF_SHUTDOWN:          Shutdown the MSM and PMIC completely
 * %PON_POWER_OFF_HARD_RESET:        Reset the MSM and all PMIC peripherals
 */
enum pon_power_off_type {
	PON_POWER_OFF_RESERVED		= 0x00,
	PON_POWER_OFF_WARM_RESET	= 0x01,
	PON_POWER_OFF_SHUTDOWN		= 0x04,
	PON_POWER_OFF_HARD_RESET	= 0x07,
	PON_POWER_OFF_MAX_TYPE		= 0x10,
};


#ifdef CONFIG_OLD_PON_IMPL

enum pon_restart_reason {
	PON_RESTART_REASON_UNKNOWN	= 0x00,
	PON_RESTART_REASON_OTHER	= 0x01,
	PON_RESTART_REASON_UNKNOWN_OEM	= 0x02,
	PON_RESTART_REASON_RESET_BUTTON_COMBO	= 0x03,
	PON_RESTART_REASON_HLOS_POWER_OFF    	= 0x04,
	PON_RESTART_REASON_HLOS_WDOG_BARK	= 0x05,
	PON_RESTART_REASON_DM_VERITY_CORRUPT	= 0x06,
	PON_RESTART_REASON_KERNEL_RESTART_UNKNOWN_ARG	= 0x07,
	PON_RESTART_REASON_HLOS_RESTART = PON_RESTART_REASON_KERNEL_RESTART_UNKNOWN_ARG,

/* !!! DO NOT CHANGE PON_RESTART_REASON_RECOVERY value, it must stay as is
 * for backward compatibility reasons
 */
	PON_RESTART_REASON_RECOVERY	= 0x08, /* 1 << 3 */

/* 0x09 - 0x0F are reserved for the bootchain */
	PON_RESTART_REASON_SBL_CHARGER	= 0x09,
	PON_RESTART_REASON_LK_OTHER	= 0x0A,
	PON_RESTART_REASON_LK_POWER_OFF	= 0x0B,

/* !!! DO NOT CHANGE PON_RESTART_REASON_BOOTLOADER value, it must stay as is
 * for backward compatibility reasons
 */
	PON_RESTART_REASON_BOOTLOADER	= 0x10, /* 2 << 3 */

	PON_RESTART_REASON_PWR_BUTTON_FORCED_12S   = 0x11,
	PON_RESTART_REASON_PWR_BUTTON_FORCED_24S   = 0x12,
	PON_RESTART_REASON_PWR_BUTTON_FORCED_32S   = 0x13,
	PON_RESTART_REASON_PWR_BUTTON_FORCED       = PON_RESTART_REASON_PWR_BUTTON_FORCED_12S,
	PON_RESTART_REASON_OVER_TEMP               = 0x14,
	PON_RESTART_REASON_UVLO                    = 0x15,
	PON_RESTART_REASON_PMI_INTERNAL_FAIL       = 0x16,
	PON_RESTART_REASON_POWERLOSS               = 0x17,

/* !!! DO NOT CHANGE PON_RESTART_REASON_RTC value, it must stay as is
 * for backward compatibility reasons
 */
	PON_RESTART_REASON_RTC		= 0x18, /* 3 << 3 */
	PON_RESTART_REASON_MODEM    = 0x19,
	PON_RESTART_REASON_VENUS    = 0x1A,
	PON_RESTART_REASON_CDSP     = 0x1B,
	PON_RESTART_REASON_ADSP     = 0x1C,
	PON_RESTART_REASON_WCNSS    = 0x1D,
	PON_RESTART_REASON_A512_ZAP = 0x1E,
	PON_RESTART_REASON_OVLO     = PON_RESTART_REASON_UNKNOWN,
	PON_RESTART_REASON_HW_FAULT = PON_RESTART_REASON_UNKNOWN,

/* 0x19 - 0x1F are free to use */
	PON_RESTART_REASON_OEM_WARM	= 0x20, /* Map to 0x00 - 0x0F in the OEM enum above */
	PON_RESTART_REASON_OEM_HARD	= 0x30, /* Map to 0x80 - 0x8F in the OEM enum above */
	PON_RESTART_REASON_MASK		= 0x3F,

	PON_RESTART_REASON_EDLOAD   = 0xFF
};
#else
enum pon_restart_reason {

	PON_RESTART_REASON_UNKNOWN              = 0x00,
	PON_RESTART_REASON_RECOVERY             = 0x01,
	PON_RESTART_REASON_BOOTLOADER           = 0x02,
	PON_RESTART_REASON_RTC                  = 0x03,
	PON_RESTART_REASON_DMVERITY_CORRUPTED   = 0x04,
	PON_RESTART_REASON_DMVERITY_ENFORCE     = 0x05,
	PON_RESTART_REASON_KEYS_CLEAR           = 0x06,
	PON_RESTART_REASON_KERNEL_RESTART_UNKNOWN_ARG       = 0x07,

	PON_RESTART_REASON_OTHER                = 0x08,
	PON_RESTART_REASON_SBL_CHARGER          = 0x09, //maintain this on all products!
	PON_RESTART_REASON_UNKNOWN_OEM          = 0x0A,
	PON_RESTART_REASON_LK_OTHER             = 0x0B,
	PON_RESTART_REASON_LK_POWER_OFF         = 0x0C,
	PON_RESTART_REASON_DM_VERITY_CORRUPT    = PON_RESTART_REASON_DMVERITY_CORRUPTED,

	PON_RESTART_REASON_RESET_BUTTON_COMBO   = 0x10,
	PON_RESTART_REASON_HLOS_POWER_OFF       = 0x11,
	PON_RESTART_REASON_HLOS_WDOG_BARK       = 0x12,
	PON_RESTART_REASON_SIM_SWAP             = 0x13,
	PON_RESTART_REASON_SDRAM                = 0x14,

/* Qualcomm specific */
	PON_RESTART_REASON_EDLOAD               = 0xFF,
	PON_RESTART_REASON_MODEM                = 0x15,
	PON_RESTART_REASON_VENUS                = 0x16,
	PON_RESTART_REASON_CDSP                 = 0x17,
	PON_RESTART_REASON_ADSP                 = 0x18,
	PON_RESTART_REASON_GPU                  = 0x19,
	PON_RESTART_REASON_WCNSS                = 0x1A,
	PON_RESTART_REASON_HEALTHD_CHG_DISABLED = 0x1B,

/* !!! Android Framework starting with 0x60 */
	PON_RESTART_FRAMEWORK_MASK              = 0x60,
	PON_RESTART_ADBD                        = 0x61,
	PON_RESTART_CHARGER_DISABLED            = 0x62, //comes from system/core
	PON_RESTART_REASON_SETUPFS_RESTART      = 0x63,
	PON_RESTART_REASON_UI_USER_REQUESTED    = 0x64,
	PON_RESTART_REASON_SAFEMODE             = 0x65,
	PON_RESTART_REASON_HLOS_RESTART         = 0x66,
	PON_RESTART_REASON_DEVADMIN             = 0x67,

/* OEM commands */
	PON_RESTART_REASON_OEM_WARM     = 0x20, /* Map to 0x00 - 0x0F in the OEM enum above */
	PON_KERNEL_PANIC_RESET          = 0x21,
	PON_RESTART_REASON_OEM_HARD     = 0x30, /* Map to 0x80 - 0x8F in the OEM enum above */
	PON_RESTART_REASON_MASK         = 0x3F,

/* !!! hardware failure. derived from pmic pon registers. Not saved in spare register */
	PON_SHUTDOWN_REASON_HW_MASK              = 0x40,
	PON_SHUTDOWN_REASON_POWER_BUTTON         = 0x41,
	PON_RESTART_REASON_POWER_BUTTON          = 0x42,
	PON_RESTART_REASON_PWR_BUTTON_FORCED_12S = PON_RESTART_REASON_POWER_BUTTON,
	PON_RESTART_REASON_PWR_BUTTON_FORCED_24S = PON_RESTART_REASON_POWER_BUTTON,
	PON_RESTART_REASON_PWR_BUTTON_FORCED_32S = PON_RESTART_REASON_POWER_BUTTON,
	PON_RESTART_REASON_OVER_TEMP             = 0x43,
	PON_RESTART_REASON_UVLO                  = 0x44,
	PON_RESTART_REASON_PMI_INTERNAL_FAIL     = 0x45,
	PON_RESTART_REASON_POWERLOSS             = 0x46,
	PON_RESTART_REASON_OVLO                  = 0x47,
	PON_RESTART_REASON_HW_FAULT              = 0x48
};
#endif

#ifdef CONFIG_BBRY
#define FIRST_OEM_WARM_RESET 0x00
#define LAST_OEM_WARM_RESET  0x0F
#define FIRST_OEM_HARD_RESET 0x80
#define LAST_OEM_HARD_RESET  0x8F
#define OEM_WARM_COUNT (LAST_OEM_WARM_RESET - FIRST_OEM_WARM_RESET + 1)
#define OEM_HARD_COUNT (LAST_OEM_HARD_RESET - FIRST_OEM_HARD_RESET + 1)

#define OEM_CODE_TO_PON_RESTART_REASON(o) \
({\
	__typeof__(o) _oem = o;\
	enum pon_restart_reason _pon = PON_RESTART_REASON_UNKNOWN_OEM; \
	if (_oem <= LAST_OEM_WARM_RESET) \
		_pon = PON_RESTART_REASON_OEM_WARM + \
			(_oem - FIRST_OEM_WARM_RESET); \
	else if ((_oem >= FIRST_OEM_HARD_RESET) && \
			(_oem <= LAST_OEM_HARD_RESET)) \
		_pon = PON_RESTART_REASON_OEM_HARD + \
			(_oem - FIRST_OEM_HARD_RESET); \
	_pon; \
})

#endif

#ifdef CONFIG_INPUT_QPNP_POWER_ON
int qpnp_pon_system_pwr_off(enum pon_power_off_type type);
int qpnp_pon_is_warm_reset(void);
int qpnp_pon_trigger_config(enum pon_trigger_source pon_src, bool enable);
int qpnp_pon_wd_config(bool enable);
int qpnp_pon_set_restart_reason(enum pon_restart_reason reason);
bool qpnp_pon_check_hard_reset_stored(void);

#ifdef CONFIG_BBRY
enum pon_power_off_type pwr_off_default_type(void);
#endif

#else
static int qpnp_pon_system_pwr_off(enum pon_power_off_type type)
{
	return -ENODEV;
}
static inline int qpnp_pon_is_warm_reset(void) { return -ENODEV; }
static inline int qpnp_pon_trigger_config(enum pon_trigger_source pon_src,
							bool enable)
{
	return -ENODEV;
}
int qpnp_pon_wd_config(bool enable)
{
	return -ENODEV;
}
static inline int qpnp_pon_set_restart_reason(enum pon_restart_reason reason)
{
	return -ENODEV;
}
static inline bool qpnp_pon_check_hard_reset_stored(void)
{
	return false;
}
#ifdef CONFIG_BBRY
enum pon_power_off_type pwr_off_default_type(void);
#endif

#endif

#endif
