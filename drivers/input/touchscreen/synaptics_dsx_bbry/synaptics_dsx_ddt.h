/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2015 BlackBerry Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _SYNAPTICS_DSX_DDT_H_
#define _SYNAPTICS_DSX_DDT_H_

#include "synaptics_dsx_core.h"
#include "synaptics_dsx_wakeup.h"

/* DDT event types */
#define UNEXPECTED_RESET		1
#define EXCEEDED_COMM_RETRIES		2
#define CONTROLLER_LOCKUP		3
#define FW_UPGRADE_EXCEEDED_RETRIES	4
#define MTOUCH_RW_ERROR			5
#define MTOUCH_PROXI_NOT_READY		6
#define MTOUCH_DEBUG			7

typedef enum {
	SYNAPTICS_WAKEUP_PASSED,
	SYNAPTICS_WAKEUP_WRONG_DIRECTION,
	SYNAPTICS_WAKEUP_OVER_X,
	SYNAPTICS_WAKEUP_TOO_SLOW,
	SYNAPTICS_WAKEUP_TOO_FAST,
	SYNAPTICS_WAKEUP_UNDER_Z,
	SYNAPTICS_WAKEUP_OVER_Z,
	SYNAPTICS_WAKEUP_INVALID_NUM_FINGERS,
	SYNAPTICS_WAKEUP_LARGE_OBJECT,
	SYNAPTICS_WAKEUP_IN_POCKET,
	SYNAPTICS_WAKEUP_BLANKING_DURATION,
	SYNAPTICS_WAKEUP_TOUCH_EVENT,
	SYNAPTICS_WAKEUP_TOO_BIG,
	SYNAPTICS_WAKEUP_INVALID_FINGER_ID,
	SYNAPTICS_WAKEUP_SWIPE_LEFT_EDGE,
	SYNAPTICS_WAKEUP_SWIPE_RIGHT_EDGE,
	SYNAPTICS_WAKEUP_STARTED_TOO_HIGH,
} synaptics_wakeup_swipe_reason_e;

enum MTOUCH_DBG_SUBTYPE {
	MTOUCH_DBG_DFLT_RPT = 0,
	MTOUCH_DBG_DT_WAKEUP,
	MTOUCH_DBG_GLASS_ON,
	MTOUCH_DBG_PROXI_IO_ERROR
};

typedef void synaptics_quip_t;

void synaptics_dsx_ddt_send_handler(struct work_struct *work);

int synaptics_dsx_ddt_send(struct synaptics_rmi4_data *rmi4_data, uint32_t etype, uint32_t details_param);

/*
 * synaptics_quip_report_wakeup_swipe
 *
 * Creates and injects a list counter update into synaptics's QUIP processing thread
 *
 * @param quip - Pointer to quip structure. This is created when calling synaptics_init_quip(...)
 * @param reason - Reason for the wakeup
 * @param wakeup_criteria - Values that hold the criteria for deciding on valid swipe or not
 * @param wakeup_data - Values that hold the wakeup swipe information
 */
static inline void synaptics_report_wakeup_swipe(
	synaptics_quip_t *quip,
	synaptics_wakeup_swipe_reason_e reason,
	const void *wakeup_criteria,
	void *wakeup_data)
{
	pr_err("%s:  reason=%d\n", __func__, reason);
	pr_err("%s:  not fully implemented", __func__);
};
#endif
