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

/* Kernel Includes */
#include <linux/kernel.h>
#include <linux/mm_types.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/mount.h>
#include <linux/version.h>
#include <linux/list.h>
#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"

/*************************************************************************/

struct process_list {
	struct list_head list;	/* kernel's list structure */
	char *path;				/* Path to an executable */
	int pid;				/* PID of the executable */
	int caps;				/* the capabilities for the executable */
};

struct _caps_globals {
	struct process_list	proclist;	/* list of {exeName/pid/caps} */
	struct mutex 			lock;		/* Lock for context */
};

static struct _caps_globals ctx = {};


/*************************************************************************/

/*
 * Entry point for initializing context.
 *
 * @return  0                No Error (Always).
 */
int __init caps_init(void)
{
	mutex_init(&ctx.lock);
	INIT_LIST_HEAD(&ctx.proclist.list);

	return 0;
}

/*************************************************************************/

/*
 * De-initialization of context data.
 *
 * @return  0                No Error (Always).
 */
int __exit caps_exit(void)
{
	caps_clean_list(1);

	mutex_destroy(&ctx.lock);

	return 0;
}


/*************************************************************************/

/*
 * A function to add a process to the list.
 *
 * If the process already exists by name then the PID is replaced.
 *
 * @param   path            The path to the executable to add.
 * @param   path_sz         Size path.
 * @param   pid             The PID of the executable
 *
 * @return  0               No Error.
 *          -ENOMEM         Memory allocation error.
 *          -EINVAL         Invalid parameters.
 */
int caps_add_process(const char *path,
				  unsigned path_sz,
				  int pid)
{
	struct process_list *proc = NULL;
	struct list_head *pos;
	int rc = 0;

	if (!path)
		return -EINVAL;

	/* Lock mutex or if interrupted by a signal, return immediately */
	if (mutex_lock_interruptible(&ctx.lock))
		return -ERESTARTSYS;

	/* Find the process in the list.  If found then update current pid */
	list_for_each(pos, &ctx.proclist.list) {
		proc = list_entry(pos, struct process_list, list);
		if (0 == strncmp(proc->path, path, BIDE_MAX_FILE_PATH)) {
			logInfo("Updating pid(%d) for process [%s]. (%d, %s)",
					proc->pid, proc->path, pid, path);
			/* update the pid */
			proc->pid = pid;
			rc = 0;
			goto cleanup;
		}
	}

	/* Only allow creation of new items before snapshot is taken. */
	if (ctl_snapshot_complete()) {
		rc = 0;
		goto cleanup;
	}

	/* Before snapshot and unknown process */
	proc = kzalloc(sizeof(struct process_list), GFP_KERNEL);
	if (!proc) {
		logError("Failed to allocate process_list.");
		rc = -ENOMEM;
		goto cleanup;
	}

	proc->path = kzalloc(path_sz + 1, GFP_KERNEL);
	if (!proc->path) {
		logError("Failed to allocate process_list->path.");
		kfree(proc);
		rc = -ENOMEM;
		goto cleanup;
	}

	memcpy(proc->path, path, path_sz);
	/* At this point we know the PID and the exe name, but not the caps
	   it may or may not request.  Set caps to 0 until we learn what
	   caps it requires. */
	proc->caps = 0;
	proc->pid = pid;

	/* Add the process to the list */
	list_add_tail(&(proc->list), &(ctx.proclist.list));

cleanup:

	mutex_unlock(&ctx.lock);
	return rc;
}

/*************************************************************************/

/*
 * A function to update the caps of a process in our list
 *
 * This should only be called prior to a snapshot being taken.
 *
 * @param   pid             The PID of the executable
 * @param   caps            The Capabilities that the PID is granted
 *
 * @return  0               No Error.
 *			-ENOENT			If the process is not found
 */
int caps_update_process(int pid,
				int caps)
{
	struct process_list *proc = NULL;
	struct list_head *pos;
	int found = 0;
	int rc = 0;

	if (ctl_snapshot_complete())
		return 0;

	/* Lock mutex or if interrupted by a signal, return immediately */
	if (mutex_lock_interruptible(&ctx.lock))
		return -ERESTARTSYS;

	list_for_each(pos, &ctx.proclist.list) {
		proc = list_entry(pos, struct process_list, list);
		if (proc->pid == pid) {
			proc->caps = proc->caps | caps;

			found = 1;

			logInfo("Updating caps to 0x%x of pid(%d)[%s].", proc->caps, pid, proc->path);
			/* update the pid */
			proc->pid = pid;
			break;
		}
	}

	if (!found) {
		rc = -ENOENT;
	}

	mutex_unlock(&ctx.lock);
	return rc;
}

/*************************************************************************/

/*
 * A function cleans the list of processes with no caps allowances.
 *
 * This should be called right after snapshot is taken to speed up searching
 * the list.  All the items with caps == 0 will be removed from the list.
 *
 * @param   all             if set then process list is deleted
 */
void caps_clean_list(int all)
{
	struct process_list *proc = NULL;
	struct list_head *pos, *next;

	/* Lock mutex or if interrupted by a signal, return immediately */
	if (mutex_lock_interruptible(&ctx.lock))
		return;

	list_for_each_safe(pos, next, &ctx.proclist.list){
		proc = list_entry(pos, struct process_list, list);
		if (all || proc->caps == 0) {
			list_del(pos);
			kfree(proc->path);
			kfree(proc);
		}
	}

	mutex_unlock(&ctx.lock);
}


/*************************************************************************/

/*
 * A function looks up the capability allowance of the process by pid.
 *
 * This should be called only before snapshot is taken.
 *
 * @param   pid             The PID of the process we are interested in
 *
 * @return  0               if process is not found or error happened
 *			#				The capability allowance of the pid
 */
int caps_get_caps_for_process(int pid)
{
	struct process_list *proc = NULL;
	struct list_head *pos;
	int rc = 0;

	/* Lock mutex or if interrupted by a signal, return immediately */
	if (mutex_lock_interruptible(&ctx.lock))
		return -ERESTARTSYS;

	list_for_each(pos, &ctx.proclist.list) {
		proc = list_entry(pos, struct process_list, list);
		if (proc->pid == pid) {
			rc = proc->caps;
			goto cleanup;
		}
	}

cleanup:
	mutex_unlock(&ctx.lock);
	return rc;
}

/*************************************************************************/

/*
 * A function is used to print the current state of the process list.
 *
 * This should be only used for debugging purposes.
 */
void caps_print(void)
{
	struct process_list *proc = NULL;
	struct list_head *pos;
	int i = 0;

	/* Lock mutex or if interrupted by a signal, return immediately */
	if (mutex_lock_interruptible(&ctx.lock))
		return;

	logInfo(" ---- Start process_list ------");

	list_for_each(pos, &ctx.proclist.list) {
		proc = list_entry(pos, struct process_list, list);
		logInfo(" process_list[%d]{ pid %d[%s], caps 0x%x}", i, proc->pid, proc->path, proc->caps);
		i++;
	}
	logInfo(" ---- End process_list ------");

	mutex_unlock(&ctx.lock);

}


