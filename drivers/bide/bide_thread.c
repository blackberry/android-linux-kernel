/*
 * Copyright (C) 2014 BlackBerry Limited
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
#include <linux/kthread.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/rbtree.h>
#include <linux/mm_types.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/kallsyms.h>
#include <asm/sections.h>

/* QSEECOM Includes */
#include <qseecom_kernel.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"

/*************************************************************************/

struct _thread_globals {
	struct task_struct *task;	/* A handle to the thread */
	wait_queue_head_t wq;		/* Wait queue event handle */
};

static struct _thread_globals ctx = {};

/*************************************************************************/

/* Time in between VMA scans */
#define WAIT_TIME_FOUR_HOURS	(4 * 60 * 60 * 1000)

/*************************************************************************/

/*
 * This is the main bide thread that runs a periodic check on kernel memory.
 *
 * @param   arg             User argument pointer (unused).
 *
 * @return  0               No Error.
 */
static int thread_proc(void *arg)
{
	void *kern_beg = (void *) (unsigned long) _stext;
	void *kern_end = (void *) (unsigned long) _etext;
	void *phys_kern_beg = (void *) virt_to_phys(kern_beg);
	int rc = 0;

	logInfo("BIDE thread started.");

	/* Add the kernel's text section to TZ */
	logError("tz_add_section(0x%p, 0x%x).", phys_kern_beg, (unsigned)(kern_end - kern_beg));
	rc = tz_add_section(phys_kern_beg, kern_end - kern_beg);
	if (rc)
		logError("Failed on tz_add_section(). rc=%d.", -rc);

	while (!kthread_should_stop()) {
		const int timeout_ms = WAIT_TIME_FOUR_HOURS;

		/* Wait for unblock signal */
		wait_event_interruptible_timeout(ctx.wq,
			kthread_should_stop(),
			msecs_to_jiffies(timeout_ms));

		/* Check if thread needs to exit */
		if (kthread_should_stop())
			break;

		vma_scan_processes();
	}

	logInfo("BIDE thread exiting.");

	return 0;
}

/*************************************************************************/

/*
 * Entry point into the module during the module init phase. This function
 * creates a thread that will periodically call into trustzone and initiate a
 * check of the kernel.
 *
 * @return  0               No Error.
 *          -ENOMEM         Out of memory for thread.
 */
int __init thread_init(void)
{
	/* Create the main thread of the driver */
	ctx.task = kthread_run(&thread_proc, NULL, "bide_thread");
	if (IS_ERR(ctx.task)) {
		int rc = (int) PTR_ERR(ctx.task);

		logError("Failed on kthread_run(). rc=%d.", -rc);
		return rc;
	}

	/* Initialize wait event */
	init_waitqueue_head(&ctx.wq);

	logInfo("Thread Initialized.");

	return 0;
}

/*************************************************************************/

/*
 * Thread clean up. This call is blocking while the thread exits.
 */
int __exit thread_exit(void)
{
	/* Signal the thread to stop and wait for it to exit */
	int rc = kthread_stop(ctx.task);
	if (rc) {
		logError("Failed on kthread_stop(). rc=%d.", -rc);
		return rc;
	}

	return 0;
}
