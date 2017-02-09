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

/* Kernel Includes */
#include <linux/kernel.h>
#include <linux/hardirq.h>
#include <linux/syscalls.h>
#include <linux/err.h>
#include <linux/types.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"

/*************************************************************************/

struct proc_auth {
	int pid;		/* Task's PID */
	int permission;		/* Allowed permission (AUTH_PERM_*) */
};

struct _auth_globals {
	struct rb_root pids;
	struct mutex lock;
};

static struct _auth_globals ctx = {};

/*************************************************************************/

#define DNSMASQ_PATH		"/system/bin/dnsmasq"
#define MAX_XML_LENGTH		200
#define MAX_PATH		256
#define KEY_STR_SZ		20

/*************************************************************************/

/*
 * This function checks if the current PID is allowed to have extra
 * capabilities.
 *
 * @param   uid        The UID of the process getting extra capabilities.
 * @param   caps       The capabilities being assinged to this process.
 *
 * @return  0          No Error.
 *          -EPERM     Permission denied.
 */
int auth_check_capabilities(int uid,
			    int caps)
{
	int allowed = 0;

	/* No capabilities, no issues */
	if (caps == 0)
		return 0;

	switch (uid) {
	case NOBODY_UID:
	{
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

		break;
	}

	case CLAT_UID:
		allowed = CAP_TO_MASK(CAP_NET_ADMIN);
		break;

	case BLUETOOTH_UID:
		allowed = CAP_TO_MASK(CAP_FOWNER);
		break;

	case WIFI_UID:
		allowed = CAP_TO_MASK(CAP_NET_ADMIN) |
			  CAP_TO_MASK(CAP_NET_RAW);
		break;

	case DHCP_UID:
		allowed = CAP_TO_MASK(CAP_NET_ADMIN) |
			  CAP_TO_MASK(CAP_NET_RAW) |
			  CAP_TO_MASK(CAP_NET_BROADCAST) |
			  CAP_TO_MASK(CAP_NET_BIND_SERVICE);
		break;

	case INSTALLER_UID:
		allowed = CAP_TO_MASK(CAP_CHOWN) |
			  CAP_TO_MASK(CAP_DAC_OVERRIDE) |
			  CAP_TO_MASK(CAP_FOWNER) |
			  CAP_TO_MASK(CAP_SETGID) |
			  CAP_TO_MASK(CAP_SETUID);
		break;

	case SHELL_UID:
		allowed = CAP_TO_MASK(CAP_DAC_READ_SEARCH);
		break;

	case PHONE_UID:
		allowed = CAP_TO_MASK(CAP_NET_ADMIN) |
			  CAP_TO_MASK(CAP_NET_RAW) |
			  CAP_TO_MASK(CAP_FSETID);
		break;
	case GPS_UID:
		allowed = CAP_TO_MASK(CAP_SETGID) |
				  CAP_TO_MASK(CAP_SETUID);
		break;
	case ROOT_UID:
		/* Don't interfere with root processes */
		return 0;
	}

	/* system_server also has allowed capabilities. */
	if ((0 == allowed) &&
	    (0 == auth_check_permission(current->pid, AUTH_PERM_PRIVILEGED_ZYGOTE))) {
		logWarn("SystemServerPid=%d", current->pid);
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
	/* Check capability subset */
	if (caps & allowed)
		return 0;

	logWarn("Invalid capabilities detected:");
	logWarn("  - UID=%d, PID=%d", uid, current->pid);
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
 *          -EAGAIN          Operation disallowed due to snapshot not taken.
 *          -EINVAL          Invalid parameter passed.
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
 *          -EINVAL         Invalid parameters.
 *          -ENOENT         PID not found.
 *          -EPERM          Permission not allowed.
 */
int auth_check_permission(int pid,
			  int perm)
{
	struct proc_auth *proc = NULL;
	char key[KEY_STR_SZ] = { 0 };
	int sz = 0;
	int rc = 0;

	/* Key for hash table is the PID in string form */
	sz = snprintf(key, sizeof(key), "%d", pid);
	if (sz < 0) {
		logError("Failed on snprintf(). rc=%d.", -sz);
		return sz;
	}

	if (mutex_lock_interruptible(&ctx.lock))
		return -ERESTARTSYS;

	rc = hash_search(&ctx.pids, key, sz, (void **) &proc);
	if (rc) {
		if (rc != -ENOENT)
			logError("Failed on hash_search(). rc=%d.", -rc);

		rc = -EPERM;
		goto err;
	}

	/* Check that the permissions are allowed */
	if (perm != (proc->permission & perm))
		rc = -EPERM;

err:
	mutex_unlock(&ctx.lock);

	return rc;
}

/*************************************************************************/

/*
 * This function removes a PID from the authorized PID list.
 *
 * @param   pid             The PID of the process.
 * @param   perm            The pid's permission (out).
 *
 * @return  0               No Error.
 *          -EINVAL         Invalid parameters.
 *          -ENOENT         PID not found.
 */
int auth_remove_pid(int pid,
		    int *perm)
{
	struct proc_auth *proc = NULL;
	unsigned long flags = 0;
	char key[KEY_STR_SZ] = { 0 };
	int spin = in_atomic();
	int sz = 0;
	int rc = 0;

	/* Key for hash table is the PID in string form */
	sz = snprintf(key, sizeof(key), "%d", pid);
	if (sz < 0) {
		logError("Failed on snprintf(). rc=%d.", -sz);
		return sz;
	}

	if (spin)
		spin_lock_irqsave(&ctx.lock.wait_lock, flags);
	else
		if (mutex_lock_interruptible(&ctx.lock))
			return -ERESTARTSYS;

	rc = hash_remove(&ctx.pids, key, sz, (void **) &proc);
	if (rc) {
		if (rc != -ENOENT)
			logError("Failed on hash_remove(). rc=%d.", -rc);

		goto err;
	}

	if (perm)
		*perm = proc->permission;

	kfree(proc);

err:
	if (spin)
		spin_unlock_irqrestore(&ctx.lock.wait_lock, flags);
	else
		mutex_unlock(&ctx.lock);

	return rc;
}

/*************************************************************************/

/*
 * This function removes a a given permission for a PID.
 *
 * @param   pid             The PID of the process.
 * @param   perm            The permission to remove.
 *
 * @return  0               No Error.
 *          -EINVAL         Invalid parameters.
 *          -ENOENT         PID not found.
 */
int auth_drop_pid_permission(int pid,
			     int perm)
{
	struct proc_auth *proc = NULL;
	char key[KEY_STR_SZ] = { 0 };
	int sz = 0;
	int rc = 0;

	/* Key for hash table is the PID in string form */
	sz = snprintf(key, sizeof(key), "%d", pid);
	if (sz < 0) {
		logError("Failed on snprintf(). rc=%d.", -sz);
		return sz;
	}

	if (mutex_lock_interruptible(&ctx.lock))
		return -ERESTARTSYS;

	/* If found, drop the permission */
	rc = hash_search(&ctx.pids, key, sz, (void **) &proc);
	if (rc == 0)
		proc->permission &= ~perm;

	mutex_unlock(&ctx.lock);

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
 *          -EINVAL         Invalid parameters.
 *          -ENOMEM         Memory allocation error.
 */
int auth_add_pid(int pid,
		 int perm)
{
	struct proc_auth *proc = NULL;
	unsigned long flags = 0;
	char key[KEY_STR_SZ] = { 0 };
	int spin = in_atomic();
	int sz = 0;
	int rc = 0;

	/* Key for hash table is the PID in string form */
	sz = snprintf(key, sizeof(key), "%d", pid);
	if (sz < 0) {
		logError("Failed on snprintf(). rc=%d.", -sz);
		return sz;
	}

	if (spin)
		spin_lock_irqsave(&ctx.lock.wait_lock, flags);
	else
		if (mutex_lock_interruptible(&ctx.lock))
			return -ERESTARTSYS;

	rc = hash_search(&ctx.pids, key, sz, (void **) &proc);
	if (rc == -ENOENT) {
		int flag = spin ? GFP_ATOMIC : GFP_KERNEL;

		/* PID is not in table, add it */
		proc = kzalloc(sizeof(struct proc_auth), flag);
		if (!proc) {
			logError("Failed to allocate proc_alloc.");

			rc = -ENOMEM;
			goto err;
		}

		proc->pid = pid;
		proc->permission = perm;

		/* Add PID to hash table */
		rc = hash_insert(&ctx.pids, key, sz, proc);
		if (rc) {
			logError("Failed on hash_insert(). rc=%d.", -rc);
			kfree(proc);
		}
	} else if (rc == 0) {
		/* Duplicate PID detected, merge permissions */
		proc->permission |= perm;
	} else
		logError("Failed on hash_search(). rc=%d.", -rc);

err:
	if (spin)
		spin_unlock_irqrestore(&ctx.lock.wait_lock, flags);
	else
		mutex_unlock(&ctx.lock);

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
	mutex_init(&ctx.lock);

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
	mutex_destroy(&ctx.lock);

	return 0;
}
