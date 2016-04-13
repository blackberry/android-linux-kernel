/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2014 BlackBerry Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include "synaptics_dsx_core.h"
#include "synaptics_dsx_ddt.h"

#include <misc/ddt.h>
#include <misc/lw_event_types.h>

#define MAX_DDT_RETRY_CNT                     (3)
#define DDT_RETRY_TIMEOUT_MS                  (500)

static bool  ddt_enabled = false;

/************************************************************
 * Check the if the ddt_token is enabled.
 * DDT is enalbed on:
 *    - debug builds
 *    - ddt token
 *************************************************************/
static int __init early_ddt_check(char *p)
{
#ifdef CONFIG_BBRY_DEBUG
	ddt_enabled = true;
#else
	if (memcmp(p, "1", 1) == 0)
		ddt_enabled = true;
	else
		ddt_enabled = false;
#endif
	return 0;
}
early_param("androidboot.debug_extra", early_ddt_check);

static inline int synaptics_dsx_ddt_event_send(struct synaptics_rmi4_data *rmi4_data, uint32_t etype)
{
	int ddt_etype, ret;
	struct logworthy_event_details_t details;
	char *device_name = (char *)rmi4_data->input_dev->name;

	/* convert etype to DDT event */
	switch (etype) {
	case FW_UPGRADE_EXCEEDED_RETRIES:
		ddt_etype = LW_EVENT_MTOUCH_0000059a;
		break;
	case EXCEEDED_COMM_RETRIES:
		ddt_etype = LW_EVENT_MTOUCH_0000059b;
		break;
	case UNEXPECTED_RESET:
		ddt_etype = LW_EVENT_MTOUCH_0000059c;
		break;
	case CONTROLLER_LOCKUP:
		ddt_etype = LW_EVENT_MTOUCH_000005aa;
		break;
	case MTOUCH_PROXI_NOT_READY:
		ddt_etype = LW_EVENT_MTOUCH_00002521;
		break;
	case MTOUCH_RW_ERROR:
		ddt_etype = LW_EVENT_MTOUCH_00002522;
		break;
	default:
		return -EINVAL;
	}

	details.d1 = 1;
	details.d2 = 2;
	details.d3 = 3;
	details.d4 = 4;
	details.creator_id = device_name;

	/* send to DDT */
	ret = ddt_send(ddt_etype, &details,
			"['logcat://main',"
			"'exec:///system/bin/dmesg']");
	return ret;
}

static char *synaptics_dsx_ddt_event_name(uint32_t etype)
{
	switch (etype) {
	case FW_UPGRADE_EXCEEDED_RETRIES:
		return "F/W UPGRADE EXCEEDED RETRIES";
	case EXCEEDED_COMM_RETRIES:
		return "EXCEEDED COMM. RETRIES";
	case UNEXPECTED_RESET:
		return "UNEXPECTED RESET";
	case CONTROLLER_LOCKUP:
		return "CONTROLLER LOCKUP";
	case MTOUCH_PROXI_NOT_READY:
		return "PROXI NOT READY";
	case MTOUCH_RW_ERROR:
		return "MTOUCH_RW_ERROR";
	default:
		break;
	}
	return NULL;
}

void synaptics_dsx_ddt_send_handler(struct work_struct *work)
{
	int ret;
	char *event_name;
	uint32_t  etype;
	struct delayed_work *dw = container_of(
			work, struct delayed_work, work);
	struct synaptics_rmi4_data *rmi4_data =
		container_of(dw, struct synaptics_rmi4_data, ddt_work);

	/* get event_name */
	etype = rmi4_data->ddt_evt;
	event_name = synaptics_dsx_ddt_event_name(etype);
	if (event_name == NULL) {
		dev_err(rmi4_data->pdev->dev.parent, "%s: invalid etype %u",
				__func__, etype);
		return;
	}
	/* print locally */
	dev_err(rmi4_data->pdev->dev.parent, "DDT - %s", event_name);

	/* send to DDT */
	ret = synaptics_dsx_ddt_event_send(rmi4_data, etype);

	if ((ret < 0) && (rmi4_data->ddt_retry < MAX_DDT_RETRY_CNT)) {
		rmi4_data->ddt_retry++;
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: Failed to send ddt event (err=%d, retry=%d)\n",
			__func__, ret, rmi4_data->ddt_retry);
		queue_delayed_work(rmi4_data->workqueue,
			&rmi4_data->ddt_work, msecs_to_jiffies(DDT_RETRY_TIMEOUT_MS));
	}
	return;
}

int synaptics_dsx_ddt_send(struct synaptics_rmi4_data *rmi4_data, uint32_t etype)
{
	int ret = 0;

	if (ddt_enabled != true) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: DDT is not enabled. Not sending ddt event", __func__);
		return 0;
	}

	rmi4_data->ddt_evt = etype;
	rmi4_data->ddt_retry = 0;
	queue_delayed_work(rmi4_data->workqueue,
		&rmi4_data->ddt_work, msecs_to_jiffies(DDT_RETRY_TIMEOUT_MS));
	return ret;
}
