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
#include "bide_proc.h"

/*************************************************************************/

struct proc_auth {
	int pid;           /* Task's PID */
	int permission;    /* Allowed permission (AUTH_PERM_*) */
};

struct debug_process_list {
	struct list_head list;  /* kernel's list structure */
	int pid;                /* PID of the executable */
	bool allow_children;    /* Allowed to have children) */
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

#define SET_CAP(var, c)  var.cap[CAP_TO_INDEX(c)] |= CAP_TO_MASK(c)

/*************************************************************************/

/*
 * This function checks if the current PID is allowed to have extra
 * capabilities.
 *
 * @param   creds      The credentials of the process getting extra capabilities.
 * @param   caps       The capabilities being assigned to this process.
 *
 * @return  0          No Error.
 *          -EPERM     Permission denied.
 */
int auth_check_capabilities(const struct cred *creds, const kernel_cap_t *caps)
{
	int i;
	kernel_cap_t allowed = {{0}};
	char buffer[CAP2HEXSTR_BUFFER_SIZE];
	char buffer2[CAP2HEXSTR_BUFFER_SIZE];
	int orig_uid = CRED_UID(creds, uid);
	int uid;

	STAT_ADD(stat_auth_check_capabilities);

	/* Convert uid to the primary user's uid space */
	uid = orig_uid % USER_UID;

	/* No capabilities, no issues */
	for (i = 0; i < _KERNEL_CAPABILITY_U32S; ++i) {
		if (caps->cap[i] != 0)
			break;
	}
	if (i == _KERNEL_CAPABILITY_U32S)
		return 0;

	if (NOBODY_UID == uid) {
		char path[BIDE_MAX_FILE_PATH] = { 0 };
		int rc = 0;

		/* Nobody is a special case, need to use paths to resolve caps */
		rc = util_get_task_path(current->mm, path, sizeof(path));
		if (rc < 0)
			return rc;

		if (strncmp(path, DNSMASQ_PATH, sizeof(DNSMASQ_PATH)) == 0) {
			SET_CAP(allowed, CAP_NET_BIND_SERVICE);
			SET_CAP(allowed, CAP_NET_ADMIN);
			SET_CAP(allowed, CAP_NET_RAW);
			SET_CAP(allowed, CAP_SETGID);
			SET_CAP(allowed, CAP_SETUID);
		}
	} else if (CLAT_UID == uid) {
		SET_CAP(allowed, CAP_NET_ADMIN);
	} else if (BLUETOOTH_UID == uid) {
		SET_CAP(allowed, CAP_NET_BIND_SERVICE);
		SET_CAP(allowed, CAP_NET_RAW);
		SET_CAP(allowed, CAP_WAKE_ALARM);
		SET_CAP(allowed, CAP_BLOCK_SUSPEND);
	} else if (WIFI_UID == uid) {
		SET_CAP(allowed, CAP_NET_ADMIN);
		SET_CAP(allowed, CAP_NET_RAW);
	} else if (DHCP_UID == uid) {
		SET_CAP(allowed, CAP_NET_ADMIN);
		SET_CAP(allowed, CAP_NET_RAW);
		SET_CAP(allowed, CAP_NET_BROADCAST);
		SET_CAP(allowed, CAP_NET_BIND_SERVICE);
	} else if (INSTALLER_UID == uid) {
		SET_CAP(allowed, CAP_CHOWN);
		SET_CAP(allowed, CAP_DAC_OVERRIDE);
		SET_CAP(allowed, CAP_FOWNER);
		SET_CAP(allowed, CAP_SETGID);
		SET_CAP(allowed, CAP_SETUID);
	} else if (SHELL_UID == uid) {
		/* BugReport runs under shell uid and requests this capability.
		   A better way to fix this in the long term is reading the
		   SELinux context that the app is running in instead of the UID
		   it's running under. */
		SET_CAP(allowed, CAP_BLOCK_SUSPEND);
	} else if (PHONE_UID == uid) {
		SET_CAP(allowed, CAP_NET_ADMIN);
		SET_CAP(allowed, CAP_NET_RAW);
		SET_CAP(allowed, CAP_FSETID);
	} else if (GPS_UID == uid) {
		SET_CAP(allowed, CAP_SETGID);
		SET_CAP(allowed, CAP_SETUID);
	} else if (RESET_CAUSE_UID == uid) {
		SET_CAP(allowed, CAP_SYS_BOOT);
		SET_CAP(allowed, CAP_SYSLOG);
		SET_CAP(allowed, CAP_SETUID);
	} else if (ROOT_UID == uid) {
		/* Don't interfere with root processes */
		return 0;
	}

	/* system_server also has allowed capabilities. */
	if (0 == auth_check_permission(current->pid, AUTH_PERM_PRIVILEGED_ZYGOTE)) {
		logInfo("SystemServerPID=%d", current->pid);
		SET_CAP(allowed, CAP_BLOCK_SUSPEND);
		SET_CAP(allowed, CAP_KILL);
		SET_CAP(allowed, CAP_NET_ADMIN);
		SET_CAP(allowed, CAP_NET_BIND_SERVICE);
		SET_CAP(allowed, CAP_NET_BROADCAST);
		SET_CAP(allowed, CAP_NET_RAW);
		SET_CAP(allowed, CAP_SYS_BOOT);
		SET_CAP(allowed, CAP_IPC_LOCK);
		SET_CAP(allowed, CAP_SYS_MODULE);
		SET_CAP(allowed, CAP_SYS_NICE);
		SET_CAP(allowed, CAP_SYS_PTRACE);
		SET_CAP(allowed, CAP_SYS_TIME);
		SET_CAP(allowed, CAP_SYS_TTY_CONFIG);
	}

	/* bugreporter has allowed capabilities. */
	if ((0 == auth_check_parents_permission(current, AUTH_PERM_BUGREPORT)) ||
            (0 == auth_check_permission(current->pid, AUTH_PERM_BUGREPORT))) {
		logInfo("BugReportPID=%d", current->pid);
		SET_CAP(allowed, CAP_SYSLOG);
	}

	/* dumpstate has allowed capabilities. */
	if ((0 == auth_check_parents_permission(current, AUTH_PERM_DUMPSTATE)) ||
            (0 == auth_check_permission(current->pid, AUTH_PERM_DUMPSTATE))) {
		logInfo("DumpstatePID=%d", current->pid);
		SET_CAP(allowed, CAP_SETUID);
		SET_CAP(allowed, CAP_SETGID);
		SET_CAP(allowed, CAP_SYS_RESOURCE);
		SET_CAP(allowed, CAP_KILL);
		SET_CAP(allowed, CAP_NET_RAW);
		SET_CAP(allowed, CAP_NET_ADMIN);
		SET_CAP(allowed, CAP_DAC_OVERRIDE);
		SET_CAP(allowed, CAP_CHOWN);
		SET_CAP(allowed, CAP_FOWNER);
		SET_CAP(allowed, CAP_FSETID);
		SET_CAP(allowed, CAP_SYSLOG);
		SET_CAP(allowed, CAP_SYS_PTRACE);
	}

	/* Allow adbd to have setuid and setgid capabilities */
	/* This lineage check is intended for run-as */
	if (0 == auth_check_parents_permission(current, AUTH_PERM_ADBD)) {
		logInfo("RunasPID=%d", current->pid);
		SET_CAP(allowed, CAP_SETUID);
		SET_CAP(allowed, CAP_SETGID);
	}

	/* Check capability subset */
	for (i = 0; i < _KERNEL_CAPABILITY_U32S; ++i) {
		if (caps->cap[i] & ~allowed.cap[i])
			break;
	}
	if (i == _KERNEL_CAPABILITY_U32S)
		return 0;

	logWarn("Invalid capabilities detected:");
	logWarn("  - UID=%d, PID=%d", orig_uid, current->pid);
	logWarn("  - Caps=%s, Allowed=%s",
		util_cap2hexstr(caps, buffer, sizeof(buffer)),
		util_cap2hexstr(&allowed, buffer2, sizeof(buffer2)));

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

	STAT_ADD(stat_auth_check_banned_caps);

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

	STAT_ADD(stat_auth_check_parents_permission);

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
 * @return  0                No error.
 *          -ENOENT          No such file or directory.
 *          -ENOMEM          Out of memory.
 *          -EINVAL          Invalid argument.
 *          -ENAMETOOLONG    File name too long.
 *          -EBADE           Invalid exchange.
 *          -ENODATA         No data available.
 *          -EMSGSIZE        Message too long.
 */
int auth_check_property(const char *name,
			const char *value)
{
	struct property_value_pair_t {
		const char *name;
		const char *value;
	};

	static const struct property_value_pair_t BANNED[] = {
#ifndef BID_USER_DEBUG
		{"ro.debuggable",    "1" },
#endif
		{"service.adb.root", "1" },
		{"ro.secure",        "0" },
		{"ro.kernel.qemu",   "1" }
	};


	int i = 0;
	int rc = 0;

	STAT_ADD(stat_auth_check_property);

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
			logError("Failed on report_incident(SN_INVALID_PROPERTY)"
				"[%s:%s]. rc=%d.", name, value, -rc);
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

	STAT_ADD(stat_auth_check_permission);

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
 * This function verifies that we have a specific permission
 *
 * @param   pid             The PID of the process.
 * @param   perm            The permission to check.
 *
 * @return 1                The PID has the permission
 *         0                The PID does not have the permission
 */
int auth_has_permission(int pid,
	int perm)
{
	if (auth_check_permission(pid, perm) == 0)
		return 1;
	else
		return 0;
}

/*************************************************************************/

/*
 * This function verifies that a process does not have a specific permission
 *
 * @param   pid             The PID of the process.
 * @param   perm            The permission to check.
 *
 * @return 1                The PID does not have the permission
 *         0                The PID has the permission
 */
int auth_does_not_have_permission(int pid,
	int perm)
{
	if (auth_check_permission(pid, perm) == 0)
		return 0;
	else
		return 1;
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

	STAT_ADD(stat_auth_remove_debug_pid);

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

	STAT_ADD(stat_auth_remove_pid);

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

	STAT_ADD(stat_auth_drop_pid_permission);

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

	STAT_ADD(stat_auth_add_pid_lock);

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
	int permissions = AUTH_PERM_PRIVILEGED;

	STAT_ADD(stat_auth_internal_process_debug_pids);

	/* Process all the PIDs in the debug PID list */
	list_for_each_safe(pos, next, &ctx.process_list.list) {
		proc = list_entry(pos, struct debug_process_list, list);

		logInfo("Registering %d based on debug token %s.", proc->pid, (token_present == 0)? "missing":"present");
		if (proc->allow_children == true)
			permissions = AUTH_PERM_PRIVILEGED_CHILDREN;

		current_rc = auth_add_pid_lock(proc->pid,
			(token_present == 1) ? AUTH_PERM_PRIVILEGED_LINEAGE : permissions,
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
 * @param   allow_children  If the process can have privileged children.
 *
 * @return  0               No Error.
 *          -ENOMEM         Memory allocation error.
 */
int auth_add_debug_pid(int pid, bool allow_children)
{
	struct list_head *pos;
	struct debug_process_list *proc;
	int rc = 0;

	STAT_ADD(stat_auth_add_debug_pid);

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
		proc = kzalloc(sizeof(struct debug_process_list), GFP_ATOMIC);
		if (!proc) {
			logError("Failed to allocate debug_process_list(%d).", pid);
			rc = -ENOMEM;
			goto cleanup;
		}
		proc->pid = pid;
		proc->allow_children = allow_children;
		/* Add the process to the list */
		list_add_tail(&(proc->list), &(ctx.process_list.list));

		rc = 0;
	} else {
		int permissions = AUTH_PERM_PRIVILEGED;
		if (rc)
			logError("Failed registering list of PIDs:%d.", rc);

		if (allow_children)
			permissions = AUTH_PERM_PRIVILEGED_CHILDREN;
		/* add the PID currently being */
		rc = auth_add_pid_lock(pid,
			ctx.token_present? AUTH_PERM_PRIVILEGED_LINEAGE:
			permissions,
			1 );

		if (rc)
			logError("Failed registering PID(%d):%d.", pid, rc);
	}

cleanup:
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
	int rc = 0;

	STAT_ADD(stat_auth_process_debug_pids);

	ctx.token_present = token_present;

	rc = auth_internal_process_debug_pids(ctx.token_present);

	return rc;
}

/*************************************************************************/

/*
 * This function display the current length and content of the auth and
 * permission list.
 */
#ifdef BID_USER_DEBUG
void auth_dump_list(void)
{
	struct debug_process_list *proc;
	struct list_head *pos, *next;
	unsigned long flags = 0;
	int count = 0;

	logError("---- Auth list ----");

	spin_lock_irqsave(&auth_lock, flags);

	list_for_each_safe(pos, next, &ctx.process_list.list) {
		proc = list_entry(pos, struct debug_process_list, list);
		logError("PID: %d", proc->pid);
		count ++;
	}

	logError("auth_list length = %d", count);
	logError("---- End auth list ----");

	bide_perm_dump_list();

	/* Unlock only after dumping bide_perm_list: Bide_perm_list
	 * is protected by auth_lock. */
	spin_unlock_irqrestore(&auth_lock, flags);
}
#endif

/*************************************************************************/

/*
 * This is the "show" function that supplies to the single_open() when
 * single_open() is being called.
 *
 * This function writes all the PIDs of the debug application that are
 * logged by Bide
 *
 * @param    sfp    Pointer the sequential file which is being opened.
 *                  This pointer is supplied by single_open().
 *
 *           p      Neither used by this function nor single_open().
 *                  It is there just to conform with seq_file API.
 *
 * @return   0      No errors.
 *           -1     Argument size > seq_file size. Nothing will be written
 *                  to the seq_file. Boundary check is done at seq_vprintf()
 *                  and -1 is returned.
 *
 */
static int auth_dbfs_dump_show(struct seq_file *sfp, void *p)
{
	struct debug_process_list *proc;
	struct list_head *pos, *next;
	unsigned long flags = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
	int count = 0;
#else
	int count = 0, rc = 0;
#endif

	/* Main logging Code */
	seq_printf(sfp, "==========================================\n");
	seq_printf(sfp, "Auth Debug Pid List\n");

	spin_lock_irqsave(&auth_lock, flags);

	list_for_each_safe(pos, next, &ctx.process_list.list) {
		proc = list_entry(pos, struct debug_process_list, list);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
		seq_printf(sfp, "PID: %d\n", proc->pid);
#else
		rc = seq_printf(sfp, "PID: %d\n", proc->pid);

		if (rc == -1) {
			spin_unlock_irqrestore(&auth_lock, flags);
			logError("auth_dbfs_dump_show(): seq_printf overflow");
			seq_printf(sfp,
                                "==========================================\n");
			return rc;
		}

#endif

		count ++;
	}

	spin_unlock_irqrestore(&auth_lock, flags);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
	seq_printf(sfp, "\nAuth Debug Pid List Length = %d\n", count);
#else
	rc = seq_printf(sfp, "\nAuth Debug Pid List Length = %d\n", count);
#endif
	seq_printf(sfp, "==========================================\n");

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
	return 0;
#else
	return ((rc == -1) ? -1 : 0);
#endif
}

/*************************************************************************/

/*
 * This is the .open function that is supplied to the debugfs file "open"
 * operation.
 *
 * This function calls single_open() function from seq_file API.
 *
 * @param    inode      Pointer to inode structure regarding to the file
 *                      that is being opened. This paramenter is supplied
 *                      by fs API.
 *
 *           file       Pointer to the file structure regarding to the file
 *                      that is being opened. This paramenter is supplied
 *                      by fs API.
 *
 * @return    0         No errors.
 *           -ENOMEM    Kzalloc() from single_open() or Kmalloc() from
 *                      seq_open() return.
 *
 */
int auth_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, auth_dbfs_dump_show, inode->i_private);
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
