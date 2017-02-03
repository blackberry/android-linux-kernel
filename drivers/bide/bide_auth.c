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
#include <linux/hardirq.h>
#include <linux/syscalls.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/types.h>
#include <linux/version.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"

/*************************************************************************/

struct proc_auth {
	int pid;           /* Task's PID */
	int permission;    /* Allowed permission (AUTH_PERM_*) */
};

struct debug_process_list {
        struct list_head list;  /* kernel's list structure */
        int pid;                /* PID of the executable */
};

struct _auth_globals {
	struct rb_root pids;
	struct debug_process_list process_list;
	int token_present;   /* -1 still undetermined, 0 not present, 1 present */
};

static struct _auth_globals ctx = {};

static DEFINE_SPINLOCK(auth_lock);

/*************************************************************************/

#define DNSMASQ_PATH     "/system/bin/dnsmasq"
#define MAX_XML_LENGTH   200
#define MAX_PATH         256

/*************************************************************************/

/*
 * This function checks if the current PID is allowed to have extra
 * capabilities.
 *
 * @param   uid        The UID of the process getting extra capabilities.
 * @param   caps       The capabilities being assigned to this process.
 *
 * @return  0          No Error.
 *          -EPERM     Permission denied.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
int auth_check_capabilities(kuid_t uid, int caps)
#else
int auth_check_capabilities(int uid, int caps)
#endif
{
	int allowed = 0;

	/* No capabilities, no issues */
	if (caps == 0)
		return 0;

	if (util_uid_eq(uid, NOBODY_UID)) {
		char path[MAX_PATH] = { 0 };
		int rc = 0;

		/* Nobody is a special case, need to use paths to resolve caps */
		rc = util_get_task_path(current->mm, path, sizeof(path));
		if (rc < 0)
			return rc;

		if (strncmp(path, DNSMASQ_PATH, sizeof(DNSMASQ_PATH)) == 0) {
			allowed = CAP_TO_MASK(CAP_NET_BROADCAST) |
				  CAP_TO_MASK(CAP_NET_RAW) |
				  CAP_TO_MASK(CAP_IPC_LOCK);
		}
	} else if (util_uid_eq(uid, CLAT_UID)) {
		allowed = CAP_TO_MASK(CAP_NET_ADMIN);
	} else if (util_uid_eq(uid, BLUETOOTH_UID)) {
		allowed = CAP_TO_MASK(CAP_FOWNER);
	} else if (util_uid_eq(uid, WIFI_UID)) {
		allowed = CAP_TO_MASK(CAP_NET_ADMIN) |
			  CAP_TO_MASK(CAP_NET_RAW);
	} else if (util_uid_eq(uid, DHCP_UID)) {
		allowed = CAP_TO_MASK(CAP_NET_ADMIN) |
			  CAP_TO_MASK(CAP_NET_RAW) |
			  CAP_TO_MASK(CAP_NET_BROADCAST) |
			  CAP_TO_MASK(CAP_NET_BIND_SERVICE);

	} else if (util_uid_eq(uid, INSTALLER_UID)) {
		allowed = CAP_TO_MASK(CAP_CHOWN) |
			  CAP_TO_MASK(CAP_DAC_OVERRIDE) |
			  CAP_TO_MASK(CAP_FOWNER) |
			  CAP_TO_MASK(CAP_SETGID) |
			  CAP_TO_MASK(CAP_SETUID);
	} else if (util_uid_eq(uid, SHELL_UID)) {
		allowed = CAP_TO_MASK(CAP_DAC_READ_SEARCH);
	} else if (util_uid_eq(uid, PHONE_UID)) {
		allowed = CAP_TO_MASK(CAP_NET_ADMIN) |
			  CAP_TO_MASK(CAP_NET_RAW) |
			  CAP_TO_MASK(CAP_FSETID);
	} else if (util_uid_eq(uid, GPS_UID)) {
		allowed = CAP_TO_MASK(CAP_SETGID) |
			  CAP_TO_MASK(CAP_SETUID);
	} else if (util_uid_eq(uid, ROOT_UID)) {
		/* Don't interfere with root processes */
		return 0;
	}

	/* system_server also has allowed capabilities. */
	if ((0 == allowed) &&
	    (0 == auth_check_permission(current->pid, AUTH_PERM_PRIVILEGED_ZYGOTE))) {
		logWarn("SystemServerPID=%d", current->pid);
		allowed = CAP_TO_MASK(CAP_BLOCK_SUSPEND) |
			  CAP_TO_MASK(CAP_KILL) |
			  CAP_TO_MASK(CAP_NET_ADMIN) |
			  CAP_TO_MASK(CAP_NET_BIND_SERVICE) |
			  CAP_TO_MASK(CAP_NET_BROADCAST) |
			  CAP_TO_MASK(CAP_NET_RAW) |
			  CAP_TO_MASK(CAP_SYS_MODULE) |
			  CAP_TO_MASK(CAP_SYS_NICE) |
			  CAP_TO_MASK(CAP_SYS_RESOURCE) |
			  CAP_TO_MASK(CAP_SYS_TIME) |
			  CAP_TO_MASK(CAP_SYS_TTY_CONFIG);
	}

	/* bugreporter has allowed capabilities. */
	if ((0 == auth_check_parents_permission(current, AUTH_PERM_BUGREPORT)) ||
            (0 == auth_check_permission(current->pid, AUTH_PERM_BUGREPORT))) {
		logInfo("BugReportPID=%d", current->pid);
		allowed |= CAP_TO_MASK(CAP_DAC_READ_SEARCH) |
			  CAP_TO_MASK(CAP_SETUID);
	}

	/* Allow adbd to have setuid and setgid capabilities */
	/* This lineage check is intended for run-as */
	if (0 == auth_check_parents_permission(current, AUTH_PERM_ADBD)) {
		logInfo("RunasPID=%d", current->pid);
		allowed |= CAP_TO_MASK(CAP_SETUID) |
			   CAP_TO_MASK(CAP_SETGID);
	}

	/* Check capability subset */
	if (caps & allowed)
		return 0;

	logWarn("Invalid capabilities detected:");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
	logWarn("  - UID=%d, PID=%d", (unsigned int)uid.val, current->pid);
#else
	logWarn("  - UID=%d, PID=%d", uid, current->pid);
#endif
	logWarn("  - Caps=%X, Allowed=%X", caps, allowed);

	return -EPERM;
}

/*************************************************************************/

/*
 * This function checks if the capabilities it has are banned.
 *
 * @param   task         The task being checked.
 *
 * @return  0            Task does not have banned caps.
 *          -EACCES      Task has banned caps.
 *          -EINVAL      Invalid arguments
 */
int auth_check_banned_caps(struct task_struct *task)
{
	const int CAPS[] = { CAP_CHOWN,
			     CAP_DAC_OVERRIDE,
			     CAP_DAC_READ_SEARCH,
			     CAP_FOWNER,
			     CAP_MAC_ADMIN,
			     CAP_MAC_OVERRIDE,
			     CAP_MKNOD,
			     CAP_SETGID,
			     CAP_SETUID,
			     CAP_SYS_ADMIN,
			     CAP_SYS_MODULE,
			     CAP_SYS_PTRACE,
			     CAP_SYS_RAWIO };
	struct user_namespace *ns = NULL;
	int i = 0;

	if (!task)
		return -EINVAL;

	/* Get the task's name space */
	ns = task_cred_xxx(task, user_ns);

	for (i = 0; i < COUNT_OF(CAPS); ++i) {
		if (ns_capable(ns, CAPS[i]))
			return -EACCES;
	}

	return 0;
}
/*************************************************************************/

/*
 * This function checks if the task and it's parents have a specific
 * whitelist permission.
 *
 * @param   task            The task being checked.
 *
 * @return  0               Current process is allowed to run.
 *          -EINVAL         Invalid parameters.
 *          -ENOENT         PID not found.
 */
int auth_check_parents_permission(struct task_struct *task,
				  int perm)
{
	int rc = -ENOENT;

	if (!task)
		return -EINVAL;


	while (task != &init_task) {
		int pid = task_tgid_vnr(task);

		/* Check if the parent chain is whitelisted */
		rc = auth_check_permission(pid, perm);
		if (!rc)
			break;

		/* Get parent task */
		rcu_read_lock();
		task = rcu_dereference(task->real_parent);
		rcu_read_unlock();
	}


	return rc;
}

/*************************************************************************/

/*
 * This function processes a set property call. If the property is not
 * allowed then a bide report is raised.
 *
 * @param   p                A pointer to a bide_set_property_cmd_t struct.
 *
 * @return  0                No Error.
 *          -EINVAL          Invalid parameter passed.
 *          -ve              Internal error while creating BID report
 */
int auth_check_property(const char *name,
			const char *value)
{
	struct property_value_pair_t {
		const char *name;
		const char *value;
	};

	static const struct property_value_pair_t BANNED[] = {
		{"ro.debuggable",    "1" },
		{"service.adb.root", "1" },
		{"ro.secure",        "0" },
		{"ro.kernel.qemu",   "1" }
        };


	int i = 0;
	int rc = 0;

	if (!name || !value)
		return -EINVAL;

	/* Check for disallowed property settings */
	for (i = 0; i < COUNT_OF(BANNED); i++) {
		char msg[MAX_XML_LENGTH] = { 0 };

		if (strcmp(BANNED[i].name, name))
			continue;

		if (strcmp(BANNED[i].value, value))
			continue;

		/* Compile a property message */
		rc = xml_property_msg(msg, sizeof(msg), name, value);
		if (rc < 0) {
			logError("Failed on xml_property_msg(). rc=%d.", -rc);
			return rc;
		}

		/* Send an incident report to JBIDE */
		rc = report_incident(SEVERITY_CRITICAL,
				     SN_INVALID_PROPERTY,
				     msg,
				     NULL);
		if (rc) {
			logError("Failed on report_incident(SN_INVALID_PROPERTY). rc=%d.", -rc);
			return rc;
		}
	}

	return rc;
}

/*************************************************************************/

/*
 * This function checks whether or not a PID has a given permission.
 *
 * @param   pid             The PID of the process.
 * @param   perm            The permission to check.
 *
 * @return  0               No Error.
 *          -ENOENT         PID not found.
 *          -EPERM          Permission not allowed.
 */
int auth_check_permission(int pid,
			  int perm)
{
	int rc = 0;
	int perm_found = 0;
	unsigned long flags = 0;

	spin_lock_irqsave(&auth_lock, flags);

	perm_found = bide_perm_search(pid);

	/* Check that the permissions exist */
	if (perm_found == 0)
		rc = -ENOENT;
	/* Check that the permissions are allowed */
	else if (perm != (perm_found & perm))
		rc = -EPERM;

	spin_unlock_irqrestore(&auth_lock, flags);

	return rc;
}

/*************************************************************************/

/*
 * This function removes a PID from the pending debug PID list.
 *
 * This method is not thread safe and should only be called internally while
 * holding the ctx.lock.
 *
 * @param   pid             The PID of the process.
 */
static void auth_remove_debug_pid(int pid)
{
	struct debug_process_list *proc;
	struct list_head *pos, *next;

	/* Process all the PIDs in the debug PID list */
	list_for_each_safe(pos, next, &ctx.process_list.list) {
		proc = list_entry(pos, struct debug_process_list, list);

		if (pid == proc->pid) {
			list_del(&proc->list);
			kfree(proc);
			return;
		}
	}
}
/*************************************************************************/

/*
 * This function removes a PID from the authorized PID list.
 *
 * @param   pid             The PID of the process.
 * @param   perm            Permission assigned to the process(out).
 *
 * @return  0               No Error.
 *          -ENOENT         PID not found.
 */
int auth_remove_pid(int pid,
		    int *perm)
{
	unsigned long flags = 0;
	int perm_found = 0;

	/* Kernel threads may have PID 0, this is never handed out by the kernel */
	if (pid == 0)
		return -ENOENT;

	spin_lock_irqsave(&auth_lock, flags);

	/* remove the PID from the debug list */
	auth_remove_debug_pid(pid);
	perm_found = bide_perm_remove(pid);

	if (perm)
		*perm = perm_found;

	spin_unlock_irqrestore(&auth_lock, flags);

	return 0;
}

/*************************************************************************/

/*
 * This function removes a a given permission for a PID.
 *
 * @param   pid             The PID of the process.
 * @param   perm            The permission to remove.
 *
 * @return  0               No Error.
 *          -ENOENT         PID not found.
 */
int auth_drop_pid_permission(int pid,
			     int perm)
{
	int rc = 0;
	unsigned long flags = 0;

	spin_lock_irqsave(&auth_lock, flags);

	/* If found, drop the permission */
	rc = bide_perm_drop(pid, perm);

	spin_unlock_irqrestore(&auth_lock, flags);

	return rc;
}

/*************************************************************************/

/*
 * This function adds a PID to the authorized PID list. If the PID already
 * exists within the table, the permissions will be merged.
 *
 * @param   pid             The PID of the process.
 * @param   perm            The whitelisted permission of the process.
 * @param   locked          1 if ctx.lock is already held by called.
 *
 * @return  0               No Error.
 *          -ENOMEM         Memory allocation error.
 */
static int auth_add_pid_lock(int pid,
		 int perm,
		 int locked)
{
	unsigned long flags = 0;
	int rc = 0;

	if (!locked)
		spin_lock_irqsave(&auth_lock, flags);

	rc = bide_perm_insert(pid, perm);

	if (!locked)
		spin_unlock_irqrestore(&auth_lock, flags);

	return rc;
}

/*************************************************************************/

/*
 * This function adds a PID to the authorized PID list. If the PID already
 * exists within the table, the permissions will be merged.
 *
 * @param   pid             The PID of the process.
 * @param   perm            The whitelisted permission of the process.
 *
 * @return  0               No Error.
 *          -ENOMEM         Memory allocation error.
 */
int auth_add_pid(int pid,
		 int perm)
{
	return auth_add_pid_lock(pid, perm, 0);
}

/*************************************************************************/

/*
 * This function processes the  debug token list based on the presence of
 * the TCL_DebugToken.
 *
 * This method is not thread safe and should only be called internally while
 * holding the ctx.lock.
 *
 * @param   token_present   0, if the debug token is not present
 *
 * @return  0               No Error.
 *          -ENOMEM         Memory allocation error.
 */
static int auth_internal_process_debug_pids(int token_present)
{
	struct debug_process_list *proc;
	struct list_head *pos, *next;
	int rc = 0;
	int current_rc = 0;

	/* Process all the PIDs in the debug PID list */
	list_for_each_safe(pos, next, &ctx.process_list.list) {
		proc = list_entry(pos, struct debug_process_list, list);

		logInfo("Registering %d based on debug token %s.", proc->pid, (token_present == 0)? "missing":"present");
		current_rc = auth_add_pid_lock(proc->pid,
			(token_present == 1)? AUTH_PERM_PRIVILEGED_LINEAGE:BIDE_IOCTL_WHITELIST_PRIVILEGED_CHILD,
			1 );

		if (current_rc)
			logError("Failed registering PID(%d):%d.", proc->pid, current_rc);

		list_del(&proc->list);
		kfree(proc);

		rc |= current_rc;
	}
	return rc;
}

/*************************************************************************/

/*
 * This function adds a PID of a debug app to the system.
 *
 * If the tcl_token is available then the PID will have lineage permission.
 * If the tcl_token is missing then the PID will have privilege permission.
 * If the token server is not available to determine then we hold onto the
 * PID until we can determine what access it should have.  At the time of
 * the snapshot we traverse the PID list and add them accordingly.
 *
 * @param   pid             The PID of the process.
 *
 * @return  0               No Error.
 *          -ENOMEM         Memory allocation error.
 */
int auth_add_debug_pid(int pid)
{
	struct list_head *pos;
	unsigned long flags = 0;
	struct debug_process_list *proc;
	int rc = 0;

	spin_lock_irqsave(&auth_lock, flags);

	if( ctx.token_present < 0 ) {
		logInfo("Token Service not yet available.  Adding PID(%d) to list.",
			pid);

		/* make sure the PID is not in the list yet */
		list_for_each(pos, &ctx.process_list.list) {
			proc = list_entry(pos, struct debug_process_list, list);
			/* if the PID is already in the list then let's ignore it */
			if (proc->pid == pid)
				goto cleanup;
		}

		/* The token service is not available.  Let's add this PID
		   to our list and reconcile it later at snapshot time */
		proc = kzalloc(sizeof(struct debug_process_list), GFP_KERNEL);
		if (!proc) {
			logError("Failed to allocate debug_process_list(%d).", pid);
			rc = -ENOMEM;
			goto cleanup;
		}
		proc->pid = pid;
		/* Add the process to the list */
		list_add_tail(&(proc->list), &(ctx.process_list.list));

		rc = 0;
	} else {
		if (rc)
			logError("Failed registering list of PIDs:%d.", rc);
		/* add the PID currently being */
		rc = auth_add_pid_lock(pid,
			ctx.token_present? AUTH_PERM_PRIVILEGED_LINEAGE:
			BIDE_IOCTL_WHITELIST_PRIVILEGED_CHILD,
			1 );

		if (rc)
			logError("Failed registering PID(%d):%d.", pid, rc);
	}

cleanup:
	spin_unlock_irqrestore(&auth_lock, flags);

	return rc;
}

/*************************************************************************/

/*
 * This function processes the debug token list.
 *
 * This method should be called from a point where we believe that the token
 * service is already available.
 *
 * @param   token_present   0 if the debug token is missing
 *
 * @return  0               No Error.
 *          -EAGAIN         Token Service not yet available
 */
int auth_process_debug_pids(int token_present)
{
	unsigned long flags = 0;
	int rc = 0;

	spin_lock_irqsave(&auth_lock, flags);

	ctx.token_present = token_present;

	rc = auth_internal_process_debug_pids(ctx.token_present);

	spin_unlock_irqrestore(&auth_lock, flags);

	return rc;
}

/*************************************************************************/

/*
 * Entry point for initializing context.
 *
 * @return  0                No Error (Always).
 */
int __init auth_init(void)
{
	INIT_LIST_HEAD(&ctx.process_list.list);
	ctx.token_present = -1;

	return 0;
}

/*************************************************************************/

/*
 * Deinitialization of context data.
 *
 * @return  0                No Error (Always).
 */
int __exit auth_exit(void)
{
	return 0;
}
