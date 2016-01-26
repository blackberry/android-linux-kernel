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

#ifndef _LINUX_QSEECOM_NETLINK_H
#define _LINUX_QSEECOM_NETLINK_H

#include <linux/types.h>

/* Message types. */
#define QSCNL_MSG_BASE 0x10
enum {
	QSCNL_MSG_INIT_DONE = QSCNL_MSG_BASE,
	QSCNL_MSG_MAX
};

/* Multicast groups */
enum qscnl_groups {
	QSCNLGRP_NONE,
#define QSCNLGRP_NONE	QSCNLGRP_NONE
	QSCNLGRP_ALL,
#define QSCNLGRP_ALL		QSCNLGRP_ALL
	__QSCNLGRP_MAX
};
#define QSCNLGRP_MAX		(__QSCNLGRP_MAX - 1)

#endif /* _LINUX_QSEECOM_NETLINK_H */

