/*
 * WLAN Driver DDT manager
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

#ifndef _WLAN_DDT_H_
#define _WLAN_DDT_H_

/* WLAN QUIP event types
 * https://wikis.rim.net/display/WLAN/WLAN+QUIP+events
 */
typedef enum {
	WLAN_DRV_EVENT_TEST = 0,
	WLAN_DRV_EVENT_PCIE_LINKDOWN,
	WLAN_DRV_EVENT_MAX /* Always last */
} wlan_drv_event_t;

/**
 * Invoke DDT to send QUIP event asynchronously
 *
 * @param event_type WLAN driver event type
 * @return > 0 if success, < 0 otherwise
 */
int wlan_ddt_event_send(wlan_drv_event_t event_type);

#endif /* _WLAN_DDT_H_ */
