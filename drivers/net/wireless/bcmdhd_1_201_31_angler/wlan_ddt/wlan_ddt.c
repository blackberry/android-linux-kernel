/*
 * WLAN driver DDT manager
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/version.h>
#include <misc/ddt.h>
#include "wlan_ddt.h"

/* QUIP event creator name string */
#define CREATOR_NAME    "wlan_driver"

/**
 * QUIP event details lookup table
 *
 * Legend for temp investigation events:
 *   d1 & d2 - unique identifier for WLAN, must be 87 and 73
 *   d3      - suggest to be 1 by now (the value 0 is used by WifiService in Java
 *             though it's for a different quip type, LW_EVENT_TEMP_INVESTIGATION.)
 *   d4      - an incremental value used to identify the particular temp event
 **/
struct logworthy_event_details_t wlan_drv_ddt_event_details[] = {
	[WLAN_DRV_EVENT_TEST] = {87, 73, 1, 99, CREATOR_NAME},
	[WLAN_DRV_EVENT_IO_CONT_BUSY] = {87, 73, 1, 1, CREATOR_NAME},
	[WLAN_DRV_EVENT_P2P_NETDEV_NULL] = {87, 73, 1, 2, CREATOR_NAME},
	[WLAN_DRV_EVENT_PCIE_LINKDOWN] = {1, 0, 0, 0, CREATOR_NAME},
	[WLAN_DRV_EVENT_MAX] = {0, 0, 0, 0, CREATOR_NAME}, /* Always last */
};

static char *wlan_ddt_event_name(wlan_drv_event_t event_type)
{
	switch (event_type) {
	case WLAN_DRV_EVENT_TEST:
		return "WLAN DRIVER TEST EVENT";
	case WLAN_DRV_EVENT_IO_CONT_BUSY:
		return "WLAN IO CONTINUOUSLY BUSY";
	case WLAN_DRV_EVENT_P2P_NETDEV_NULL:
		return "WLAN P2P NETDEV NULL POINTER";
	case WLAN_DRV_EVENT_PCIE_LINKDOWN:
		return "PCIE LINKDOWN EVENT";
	default:
		break;
	}
	return NULL;
}

int wlan_ddt_event_send(wlan_drv_event_t event_type)
{
	struct logworthy_event_details_t *details;
	int ddt_event;
	int ret;
	char *event_name;

	/* convert WLAN event type to DDT event */
	switch (event_type) {
	case WLAN_DRV_EVENT_TEST:
	case WLAN_DRV_EVENT_IO_CONT_BUSY:
	case WLAN_DRV_EVENT_P2P_NETDEV_NULL:
		ddt_event = LW_EVENT_TEST_00000103;
		break;
	case WLAN_DRV_EVENT_PCIE_LINKDOWN:
		ddt_event = LW_EVENT_WLAN_00000900;
		break;
	default:
		return -EINVAL;
	}

	/* Get event_name */
	event_name = wlan_ddt_event_name(event_type);
	if (event_name == NULL) {
		pr_err("%s: Invalid WLAN driver event\n", __func__);
		return -EINVAL;
	}

	/* Assign the event details */
	details = &wlan_drv_ddt_event_details[event_type];

	/* Send to DDT */
	ret = ddt_send(ddt_event, details,
			"['logcat://main','dumpsys://wifi','kerneldump://']");
	if (ret < 0) {
		pr_err("%s: Failed to create \"%s\" event (err::%d)\n",
			__func__, event_name, ret);
	} else {
		pr_err("%s: Successfully created \"%s\" event\n",
			__func__, event_name);
	}

	return ret;
}

