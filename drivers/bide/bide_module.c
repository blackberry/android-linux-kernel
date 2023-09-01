/*
 * Copyright (C) 2018 BlackBerry Limited
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

/* Kernel Includes */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"
#include "bide_proc.h"

/*************************************************************************/

/*
 * Handler function for when the module is being unloaded. Clean up the
 * various subsections of this module.
 */
static void __exit bide_exit(void)
{
	debugfs_exit();
	thread_exit();
	dev_exit();
	report_exit();
	bide_netlink_exit();
	secop_exit();
	vma_exit();
	tz_exit();
	auth_exit();
	proc_exit();
	ctl_exit();
	perm_exit();
}

/*************************************************************************/

/*
 * Order matters for init calls.
 * The order is: early, core, postcore, arch, subsys, fs, device, late.
 * Two special types of init calls are: console, and security.
 */
#ifdef BID_USER_DEBUG
core_initcall(stat_init);
#endif
core_initcall(ctl_init);
core_initcall(auth_init);
core_initcall(perm_init);
core_initcall(proc_debug_init);
security_initcall(proc_init);
security_initcall(secop_init);
subsys_initcall(tz_init);
subsys_initcall(bide_netlink_init);
device_initcall(dev_init);
device_initcall(report_init);
late_initcall(audit_init);
late_initcall(vma_init);
late_initcall(thread_init);
late_initcall(debugfs_init);
module_exit(bide_exit);

/*************************************************************************/

MODULE_AUTHOR("BlackBerry Limited, All rights reserved.");
MODULE_LICENSE("GPL");
