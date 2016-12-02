/*
 * Copyright (C) 2015 BlackBerry Limited
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

#ifndef _LINUX_PATHTRUST_NETLINK_H
#define _LINUX_PATHTRUST_NETLINK_H

#include <linux/types.h>

/* Message types. */
#define PTNL_MSG_BASE 0x10
enum {
	PTNL_MSG_ENFORCE_TOGGLE = PTNL_MSG_BASE,
	PTNL_MSG_MAX
};

/* Multicast groups */
enum pathtrust_nlgroups {
	PTNLGRP_NONE,
#define PTNLGRP_NONE	PTNLGRP_NONE
	PTNLGRP_ALL,
#define PTNLGRP_ALL		PTNLGRP_ALL
	__PTNLGRP_MAX
};
#define PTNLGRP_MAX		(__PTNLGRP_MAX - 1)

/* Message structures */
struct ptnl_msg_enforce_toggle {
	__s32		enforce;
};

#endif /* _LINUX_PATHTRUST_NETLINK_H */
