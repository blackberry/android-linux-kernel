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

#ifndef __BIDE_PROC_H__
#define __BIDE_PROC_H__

#include <linux/kernel.h>
#include <linux/list.h>

/************************************************************************
 * Types
 ************************************************************************/
#define BIDE_PROC_FLAG_INITIALIZING	0x00000001
#define BIDE_PROC_FLAG_INIT_SPAWNED	0x00000002
#define BIDE_PROC_FLAG_ZYGOTE_SPAWNED	0x00000004
#define BIDE_PROC_FLAG_PRE_SNAPSHOT	0x00000008
#define BIDE_PROC_FLAG_DEAD		0x00000010
#define BIDE_PROC_FREE_NAME		0x00000020

/************************************************************************
 * Types
 ************************************************************************/
struct pinfo_node;
struct process_info {
	struct pinfo_node *node;

	struct rcu_head rcu;

	int pid;
	char *name;
	uint32_t flags;
};

/************************************************************************
 * Private API - Don't call these directly
 ************************************************************************/
const struct process_info* __proc_get_info(int pid);
const struct process_info* __proc_create_info(int pid, int ppid);

/************************************************************************
 * Public API
 ************************************************************************/
int proc_debug_init(void);
int proc_init(void);
int proc_exit(void);

/*
 * A function to get, and create if necessary, a process_info struct
 * for a given pid
 *
 * If a mapping from pid->process_info does not yet exist
 * one is created by cloning the parent's.
 *
 * On success, this API implicitly starts an RCU read-side critical section.
 * To complete/exit the section call proc_put_info.
 *
 * The returned process_info struct should be considered RO and
 * MUST NOT be modified by the caller. To get a modifiable version
 * of the struct call proc_start_update.
 *
 * @param   pid             The PID of the process we are interested in
 *
 * @return  A pointer to the process_info struct or NULL if one can't
 *          be found and created
 */
static inline const struct process_info* proc_get_info(int pid) {
	const struct process_info *result;

	rcu_read_lock();
	result = __proc_get_info(pid);
	if (result == NULL)
		rcu_read_unlock();

	return result;
}

/*
 * A function to create a process_info struct
 * for a given pid
 *
 * If a mapping from pid->process_info does not yet exist
 * one is created by cloning the parent's.
 *
 * On success, this API implicitly starts an RCU read-side critical section.
 * To complete/exit the section call proc_put_info.
 *
 * The returned process_info struct should be considered RO and
 * MUST NOT be modified by the caller. To get a modifiable version
 * of the struct call proc_start_update.
 *
 * @param   pid             The PID of the process we are interested in
 *
 * @param   ppid             The PID of the parent process we are interested in
 *
 * @return  A pointer to the process_info struct or NULL if one can't
 *          be found and created
 */
static inline const struct process_info* proc_create_info(int pid, int ppid) {
	const struct process_info *result;

	rcu_read_lock();
	result = __proc_create_info(pid, ppid);
	if (result == NULL)
		rcu_read_unlock();

	return result;
}

/*
 * A function to indicate that the caller is done reading
 * the specified pinfo.
 *
 * This will end the read-side critical section
 *
 * @param pinfo The process_info that is no longer needed
 */
static inline void proc_put_info(const struct process_info *pinfo)
{
	if (pinfo != NULL)
		rcu_read_unlock();
}

/*
 * Helper function that retrieves the name of a given process
 * into the indicated buffer.
 *
 * This API is guaranteed to produce a NULL-terminated string
 * in buffer even if it is unable to find the process.
 *
 * @param pid           The PID of the process
 * @param buffer        Buffer to store the processes name
 * @param size          The size of the buffer in bytes.
 */
void proc_task_get_name(int pid, char *buffer, size_t size);

/*
 * Helper function that indicates if a given process is
 * in the 'initializing' state.
 *
 * A process is in the initializing state if it is being
 * forked from either init or zygote and the jump into
 * the new program's code hasn't happened yet.
 *
 * @param pid   The pid of the process
 *
 * @return      Non-zero if the process is in the initializing state,
 *              zero otherwise. If the process isn't found it is
 *              considered to NOT be in the initializing state.
 */
int proc_is_pid_initializing(int pid);

/*
 * Returns the process_info for the specified process's parent.
 *
 * The information is for the processes original parent, the
 * one that spawned/forked it. Unlike the normal process hierarchy
 * a process is not re-parented if its parent dies. Instead we
 * remember the last state of the parent to be returned by
 * this API.
 *
 * Remember to call proc_put_info() passing the process_info
 * returned from this function when you are done with it.
 *
 * @note The parent process may no longer be present in the system,
 *       nor is its PID guaranteed to be unique. It may have been
 *       reused.
 *
 * @param pinfo         The process_info, probably retrieved by
 *                      proc_get_info, whos parent is being requested.
 *
 * @returns             The parent's process_info, or NULL if the process
 *                      has no parent. This should only happen for the /init
 *                      process.
 */
const struct process_info *proc_get_parent(const struct process_info *pinfo);

/*
 * A function to indicate a process has 'exec' into
 * a new program.
 *
 * @param pid   The PID of the process being exec'ed
 * @param name  The name of the binary/process being exec'ed into
 */
void proc_task_exec(int pid, const char* name);

/*
 * A function indicating that a processes' initialization
 * phase has completed.
 *
 * That means control is being passed to the application's
 * code and should no longer be truested.
 *
 * This API should only be called for processes spawned
 * from either init or zygote. It should also only be
 * called once the process is in the right security
 * domain as any pending security checks are completed
 * during this function call.
 *
 * @param pid   The PID of the process
 * @param sid   The SID of the processes' security domain
 */
void proc_task_init_complete(int pid, u32 sid);

/*
 * A function to indicate a task has died.
 *
 * This API is safe to call from an interrupt context
 *
 * @param pid The PID of the process that died.
 */
void proc_task_free(int pid);

#endif
