/*
 * Copyright (C) 2016 BlackBerry Limited
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _DDT_H_
#define _DDT_H_

#include <linux/kernel.h>
#include <misc/lw_event_types.h>

/**
 * Logworthy Event Details
 */
struct logworthy_event_details_t {
	uint32_t d1;      /* Details Field 1 */
	uint32_t d2;      /* Details Field 2 */
	uint32_t d3;      /* Details Field 3 */
	uint32_t d4;      /* Details Field 4 */
	char *creator_id; /* Creator id Field */
};

int ddt_send(int etype, const struct logworthy_event_details_t *d,
		const char *cmds);

#endif
