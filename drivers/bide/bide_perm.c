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
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/err.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"

struct bide_perm_list {
	struct list_head list;  /* kernel's list structure */
	int pid;                /* PID of the executable */
	int perm;               /* permission assigned to the PID */
};

struct _perm_globals {
	struct bide_perm_list perm_list;
};

static struct _perm_globals ctx = {};

/*****************************************************************************/

/*
 * Search for the permission assigned to a specific PID.
 *
 * @param root    the root of the RedBlack tree to look in
 * @param pid     the pid of the process being queried
 *
 * @returns the permission associated with the process, 0 if the process
 *          does not have any paermissions assigned to it
 *
 */
int bide_perm_search(int pid)
{
	struct list_head *pos;
	struct bide_perm_list *proc;
	int count = 0;

	/* make sure the PID is not in the list yet */
	list_for_each(pos, &ctx.perm_list.list) {
		proc = list_entry(pos, struct bide_perm_list, list);
		if (!proc) {
			logError("NULL process found in perm_list_search at item %d.", count);
			return 0;
		}
		if (proc->pid == pid)
			return proc->perm;
		count++;
	}
	/* process not found, return 0 */
	return 0;
}

/*****************************************************************************/

/*
 * Add a permission for a PID.
 *
 * If we do not know if this PID yet then add it to our list, otherview update
 * the PIDs permission with the permission passed in
 *
 * @param pid    the PID of the process we want to remember a permission for
 * @param perm   the permission we want to assign the PID
 *
 * @return 0         if successful
 *         -ENOMEM   if we are out of memory
 */
int bide_perm_insert(int pid, int perm)
{
	struct list_head *pos;
	struct bide_perm_list *proc;
	int count = 0;

	/* If the PID is in the list then update the permission */
	list_for_each(pos, &ctx.perm_list.list) {
		proc = list_entry(pos, struct bide_perm_list, list);
		if (!proc) {
			logError("NULL process found in perm_list_insert at item %d.", count);
			return 0;
		}
		if (proc->pid == pid) {
			proc->perm |= perm;
			return 0;
		}
		count++;
	}

	/* process not found, let's add it to the list */
	proc = kzalloc(sizeof(struct bide_perm_list), GFP_KERNEL);
	if (!proc) {
		logError("Failed to allocate bide_perm_list(%d, %d).", pid, perm);
		return -ENOMEM;
	}

	proc->pid = pid;
	proc->perm = perm;

	/* Add the process to the list */
	/* TODO, we should sort this list to make it quicker for lookup */
	list_add_tail(&(proc->list), &(ctx.perm_list.list));
	return 0;
}

/*****************************************************************************/

/*
 * Drop a permission for a PID.
 *
 * @param pid    the PID of the process we want to drop the permission for
 * @param perm   the permission we want to drop
 *
 * @return 0         if successful
 *         -ENOENT   if the PID permissions do not exist
 */
int bide_perm_drop(int pid, int perm)
{
	struct list_head *pos;
	struct bide_perm_list *proc;
	int count = 0;

	/* If the PID is in the list then update the permission */
	list_for_each(pos, &ctx.perm_list.list) {
		proc = list_entry(pos, struct bide_perm_list, list);
		if (!proc) {
			logError("NULL process found in perm_list_drop at item %d.", count);
			return 0;
		}
		if (proc->pid == pid) {
			proc->perm &= ~perm;
			return 0;
		}
		count++;
	}

	return -ENOENT;
}


/*****************************************************************************/

/*
 * Remove the permission of a specific PID.
 *
 *
 * @param pid    the PID of the process we want to forget about
 *
 * @returns the permission assigned to the PId that was removed
 */
int bide_perm_remove(int pid)
{
	struct bide_perm_list *proc;
	struct list_head *pos, *next;
	int perm = 0;
	int count = 0;

	/* Process all the PIDs in the debug PID list */
	list_for_each_safe(pos, next, &ctx.perm_list.list) {
		proc = list_entry(pos, struct bide_perm_list, list);
		if (!proc) {
			logError("NULL process found in perm_list_remove at item %d.", count);
			return 0;
		}

		if (pid == proc->pid) {
			perm = proc->perm;
			list_del(&proc->list);
			kfree(proc);
			return perm;
		}
		count++;
	}
	return 0;
}

/*************************************************************************/

/*
 * Entry point for initializing context.
 *
 * @return  0                No Error (Always).
 */
int __init perm_init(void)
{
	INIT_LIST_HEAD(&ctx.perm_list.list);
	return 0;
}

/*************************************************************************/

/*
 * Deinitialization of context data.
 *
 * @return  0                No Error (Always).
 */
int __exit perm_exit(void)
{
	return 0;
}
