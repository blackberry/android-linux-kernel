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
#include <linux/security.h>
#include <linux/err.h>
#include <linux/fs_struct.h>
#include <linux/syscalls.h>
#include <linux/cred.h>
#include <linux/list.h>
#include <linux/mman.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <asm-generic/siginfo.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
#include <linux/lsm_hooks.h>
#endif

/* SELinux internals */
#include <security.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"
#include "bide_proc.h"

/*************************************************************************/

#define EXTRA_XML_MESSAGE_LEN		256

#define INTERNAL_STORAGE		"/mnt/shell/emulated/"
#define EXTERNAL_STORAGE		"/storage/emulated/"
#define DEV_BLOCK_DM			"/dev/block/dm-"
#define AD_SECURE_STORAGE		"/mnt/media_rw/sdcard1/.android_secure"
#define EXFAT_STORAGE			"/dev/block/loop_exfat"

 /* External Mounts introduced in Android M */
#define EXTERNAL_WRITE			"/mnt/runtime/write"
#define EXTERNAL_READ			"/mnt/runtime/read"
#define EXTERNAL_DEFAULT		"/mnt/runtime/default"
#define EXTERNAL_SD			"/mnt/runtime/allstar"
/* User Mount introduced in Android M*/
#define USER_MOUNT_PATH			"/mnt/user/"

/*************************************************************************/

static const kgid_t PRIVILEGED_GROUPS[] = {
	ROOT_KGID,
	SYSTEM_KGID
};

struct gid_whitelist_t {
	const char *name;
	//gids is a bitfield marking which PRIVILEGED_GROUPS
	//the app is whitelisted for
	//Take
	unsigned int gids;
};
#define PRIVILEGED_GID_IDX_TO_MASK(idx) (1L << (idx))
static const struct gid_whitelist_t GID_WHITELIST[] = {
	{"/system/bin/loc_launcher", PRIVILEGED_GID_IDX_TO_MASK(1)}
};

static int is_app_whitelisted_for_gid(const char *name, int priv_gid_idx) {
	int i;

	for (i = 0; i < sizeof(GID_WHITELIST)/sizeof(GID_WHITELIST[0]); ++i) {
		if (!strcmp(GID_WHITELIST[i].name, name)) {
			return (GID_WHITELIST[i].gids &
				PRIVILEGED_GID_IDX_TO_MASK(priv_gid_idx)) != 0;
		}
	}

	return 0;
}

/*************************************************************************/

/*
 * A security operation callback when a new task is created. Currently we are in
 * the context of the parent process.
 *
 * @param   task            The child task that is being created
 *
 * @param   flags           Determines what should be shared. See clone()
 *                          man page for definitations of the flags.
 */
static void secop_task_created_notify(struct task_struct *task, unsigned long flags)
{
	const struct process_info *pinfo;

	STAT_ADD(stat_secop_task_created_notify);

	if (!task)
		return;

	//dont care about user threads
	if (flags & CLONE_THREAD)
		return;

	if (util_get_tgid(task->real_parent) == KTHREAD_PID)
		return;

	pinfo = proc_create_info(util_get_tgid(task), util_get_tgid(task->real_parent));
	if (!pinfo) {
		// the process info was already created.  No locking was
		// neccessary so we can just return. */
		return;
	}
	// We created a new process info.  We need to release the lock so it may
	// be used
	proc_put_info(pinfo);
}

/************************************************************************/

/*
 * A security operation callback when a task ends. This will be called
 * upon the final call to put_task_struct(), essentially removing the last
 * reference from that task.
 *
 * This function gets called while in an IRQ interrupt context. Any mutex
 * locking attempt will cause undefined behaviour.
 *
 * @param   task                The task that is exiting.
 */
static void secop_task_free(struct task_struct *task)
{
	struct list_head *p = NULL;
	int perm = 0;
	int rc = 0;

	STAT_ADD(stat_secop_task_free);

	if (!task)
		return;

	/* This callback is called for any thread that terminates for
	   the process.  Only remove the PID if it's the group leader
	   as we only add pid for the group leader. */
	if (util_get_tgid(task) != task->pid)
		return;

	proc_task_free(util_get_tgid(task));

	rc = auth_remove_pid(util_get_tgid(task), &perm);
	if (rc) {
		if (rc != -ENOENT)
			logError("Failed on auth_remove_pid(). rc=%d.", -rc);

		return;
	}

	/* Disallow propagation of the following whitelist privilages */
	perm &= ~(AUTH_PERM_ZYGOTEMGR | AUTH_PERM_ZYGOTEMGR_CHILD);

	if (!perm || !task->children.next)
		return;

	/* Loop through all children and white list them */
	list_for_each(p, &task->children) {
		struct task_struct *t = NULL;

		if (!p)
			continue;

		t = list_entry(p, struct task_struct, sibling);
		if (!t)
			continue;

		/* Give child processes the same permission as parent */
		rc = auth_add_pid(util_get_tgid(t), perm);
		if (rc)
			logError("Failed on auth_add_pid(). rc=%d.", -rc);
	}
}

/************************************************************************/

/*
 * Save security information in the bprm->security field, typically based
 * on information about the bprm->file, for later use by the apply_creds
 * hook.  This hook may also optionally check permissions (e.g. for
 * transitions between security domains).
 *
 * This hook may be called multiple times during a single execve, e.g. for
 * interpreters.  The hook can tell whether it has already been called by
 * checking to see if @bprm->security is non-NULL.  If so, then the hook
 * may decide either to retain the security information saved earlier or
 * to replace it.
 *
 * @param   bprm                Contains the linux_binprm structure.
 *
 * @return  0                   No error.
 *          -EPERM              Operation not permitted
 *          -ENOENT             No such file or directory.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -ENAMETOOLONG       File name too long.
 *          -EBADE              Invalid exchange.
 *          -ENODATA            No data available.
 *          -EMSGSIZE           Message too long.
 */
static int secop_bprm_set_creds(struct linux_binprm *bprm)
{
	struct task_struct *parent = NULL;
	int rc = 0;
	int is_parent_kthread = 0;
	unsigned long ppid = 0;
	int current_pid = util_get_tgid(current);

	STAT_ADD(stat_secop_bprm_set_creds);

	/* Determine if the task needs to be checked against whitelist */
	if (!util_is_task_root(current) &&
	    !auth_check_banned_caps(current))
		return 0;

	/* Ignore kernel and init direct descendants */
	if (current_pid == KTHREAD_PID ||
	    current_pid == INIT_PID)
		return 0;

	/* Ignore whitelisted processes */
	if (!auth_check_permission(current_pid, AUTH_PERM_ZYGOTEMGR) ||
	    !auth_check_permission(current_pid, AUTH_PERM_PRIVILEGED_ZYGOTE))
	    return 0;

	/* learn about our parent */
	rcu_read_lock();
	parent = rcu_dereference(current->real_parent);
	ppid = task_tgid_vnr(parent);
	if (!parent->mm)
		is_parent_kthread = 1;
	/* the parent task structure is no longer needed */
	parent = NULL;
	rcu_read_unlock();

	/* Ignore whitelisted process */
	if (auth_has_permission(current_pid, AUTH_PERM_PRIVILEGED_GID) ||
		auth_has_permission(current_pid, AUTH_PERM_PRIVILEGED))
		return 0;

	/* If Parent can have privileged children, then promote this child */
	if (auth_has_permission(ppid, AUTH_PERM_PRIVELEGED_CHILDREN)) {
		rc = auth_add_pid(current_pid, AUTH_PERM_PRIVILEGED);
		if (rc)
			logError("Failed on auth_add_pid(%d, %d). rc=%d.",
			current_pid, AUTH_PERM_PRIVILEGED, -rc);

		return 0;
	}

	/* If Parent can have privileged lineage, then promote this child to that level */
	if (auth_has_permission(ppid, AUTH_PERM_PRIVILEGED_LINEAGE)) {
		rc = auth_add_pid(current_pid, AUTH_PERM_PRIVILEGED_LINEAGE |
					       AUTH_PERM_PRIVILEGED);
		if (rc)
			logError("Failed on auth_add_pid(%d, %d). rc=%d.",
			current_pid, AUTH_PERM_PRIVILEGED_LINEAGE, -rc);

		return 0;
	}


	/* Allow system_server to launch child processes */
	if (!auth_check_permission(ppid, AUTH_PERM_PRIVILEGED_ZYGOTE))
		return 0;

	/* Check if Parent is the Kernel worker thread (DDT launched) */
	if (is_parent_kthread == 1) {
		rc = auth_add_pid(current_pid, AUTH_PERM_PRIVILEGED);
		if (rc)
			logError("Failed on auth_add_pid() from secop_bprm_set_creds(). rc=%d.", -rc);

		return 0;
	}

	/* Promote root process to privilaged status before snapshot or
	 * if the parent is init. */
	if (!ctl_snapshot_complete() || ppid == INIT_PID) {
		rc = auth_add_pid(current_pid, AUTH_PERM_PRIVILEGED);
		if (rc)
			logError("Failed on auth_add_pid(). rc=%d.", -rc);
	} else {
		/* We still don't have a real way of knowing what should run as root.  We
		   use to keep a whitelist in the libinit_oem code that got registeres but
		   we kept playing catchup.  We need to see if we can get a better list
		   from the rc files, or some other way.  for now let's log these as sensors
		   in develop so we could take some metrics of waht's going on and make a
		   decision afterwards of what we should do.*/
		rc = report_incident(SEVERITY_DEVELOP,
				     SN_ROOT_PROCESS_DETECTOR,
				     NULL,
				     current);
		if (rc)
			logDebug("Failed on report_incident() from secop_bprm_set_creds(). rc=%d", -rc);
	}

	/* Allow */
	return 0;
}

/*
 * This function checks if the current process is a whitelisted system process
 * or its parent is a whitelisted system process other then the Zygote Manager
 * or one of the Zygote spawning processes
 *
 * @param    curent_pid  The PID of the current process
 *           ppid        the pid of the current process' parent
 *
 * @return   0 if the process is not suppose to run with system privileges
 *           1 if the process is allowed to run with system privileges
 */
static int secop_is_allowed_system_process(int current_pid, int ppid)
{
	/* The design of this method to allow the following lineage of process:
	 *  - init->PrA
	 *  - [anything]->PrA, where PrA is a system process
	 *  - zygote->PrA->PrB, where PrA is a system process
	 *  - zygoteman->PrA->PrB, where PrA is a system process
	 * The design is trying to flag the following process as not allowed:
	 *  - zygote->PrA, where PrA is not whitelisted as system process
	 *  - zygoteman->PrA, where PrA is not whitelisted as ssytem process
	 */
	if ((0 == auth_check_permission(current_pid, AUTH_PERM_SYSTEM_UID)) ||
		(ppid == INIT_PID)) {
		return 1;
	} else if ((auth_does_not_have_permission(ppid, AUTH_PERM_ZYGOTEMGR_CHILD)) &&
				(auth_does_not_have_permission(ppid, AUTH_PERM_ZYGOTEMGR)) &&
				(auth_has_permission(ppid, AUTH_PERM_SYSTEM_UID))){
		return 1;
	}
	return 0;

}
/*************************************************************************/

/*
 * This function is when the module's state is updated, after setting one or
 * more of the user identity attributes of the current process.
 *
 * @param   new                 The set of credentials that will be installed.
 * @param   old                 The set of credentials that are being replaced.
 * @param   flags               Indicates which of the set*uid system calls invoked
 *                              this hook contains one of the LSM_SETID_* values.
 *
 * @return  0                   No error.
 *          -EPERM              Operation not permitted.
 *          -ENOENT             No such file or directory.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -ENAMETOOLONG       File name too long.
 *          -EBADE              Invalid exchange.
 *          -ENODATA            No data available.
 *          -EMSGSIZE           Message too long.
 */
static int secop_task_fix_setuid(struct cred *new,
				 const struct cred *old,
				 int flags)
{
	int perm = 0;
	int rc = 0;
	unsigned long ppid = 0;
	const int ROOT_UID_ISSUE = 1;
	const int SYSTEM_UID_ISSUE = 2;
	int issue_found = 0;
	int current_pid = util_get_tgid(current);
	int old_uid = CRED_UID(old, uid);
	int new_uid = CRED_UID(new, uid);
	int new_euid = CRED_UID(new, euid);
	int new_base_uid = new_uid % USER_UID;

	STAT_ADD(stat_secop_task_fix_setuid);

	/* Track downgrade in permission */
	if ((SYSTEM_UID == old_uid) && (SYSTEM_UID != new_base_uid))
		perm |= AUTH_PERM_SYSTEM_UID;
	if (perm)
		auth_drop_pid_permission(current_pid, perm);

	ppid = util_get_real_parent_pid();
	if (!auth_check_permission(ppid, AUTH_PERM_INSTALLD) ||
	   (!auth_check_permission(current_pid, AUTH_PERM_INSTALLD))) {
		rc = auth_add_pid(current_pid, AUTH_PERM_SYSTEM_UID);
		if (rc)
			logError("Failed on auth_add_pid(AUTH_PERM_SYSTEM_UID). rc=%d.", -rc);
	}

	/* If Parent can have privileged children, then promote this child */
	if (auth_has_permission(ppid, AUTH_PERM_PRIVELEGED_CHILDREN)) {
		rc = auth_add_pid(current_pid, AUTH_PERM_PRIVILEGED);
		if (rc)
			logError("Failed on auth_add_pid(AUTH_PERM_PRIVILEGED). rc=%d.", -rc);
	}

	/* If Parent can have privileged lineage, then promote this child to that level */
	if (auth_has_permission(ppid, AUTH_PERM_PRIVILEGED_LINEAGE)) {
		rc = auth_add_pid(current_pid, AUTH_PERM_PRIVILEGED_LINEAGE |
					       AUTH_PERM_PRIVILEGED);
		if (rc)
			logError("Failed on auth_add_pid(%d, %d). rc=%d.",
			current_pid, AUTH_PERM_PRIVILEGED_LINEAGE, -rc);

		return 0;
	}

	/* Don't report if snapshot was not taken */
	if (!ctl_snapshot_complete())
		return 0;

	/* Check the process was whitelisted if it
	 * wants a privileged uid
	 */
	if ((ROOT_UID == new_uid) || (ROOT_UID == new_euid)) {
		/* Check root permission was whitelisted */
		if (auth_does_not_have_permission(current_pid, AUTH_PERM_PRIVILEGED))
			issue_found |= ROOT_UID_ISSUE;
	}

	if (SYSTEM_UID == new_base_uid) {
		/* Allow a root process to downgrade to System */
		if (ROOT_UID == old_uid) {
			rc = auth_add_pid(current_pid, AUTH_PERM_SYSTEM_UID);
		} else {
			/* Check that System permissions were whitelisted */
			rc = secop_is_allowed_system_process(current_pid, ppid);
			if (!rc)
				issue_found |= SYSTEM_UID_ISSUE;
		}
	}

	if (issue_found & ROOT_UID_ISSUE) {
		rc = report_incident(SEVERITY_CRITICAL,
				     SN_ESCALATED_UID,
				     NULL,
				     current);
		if (rc)
			logError("Failed on report_incident(SN_ESCALATED_UID). rc=%d.", -rc);
	}

	if (issue_found & SYSTEM_UID_ISSUE) {
		rc = report_incident(SEVERITY_CRITICAL,
				     SN_SYSTEM_UID,
				     NULL,
				     current);
		if (rc)
			logError("Failed on report_incident(SN_SYSTEM_UID). rc=%d.", -rc);

	}
	/* Allow */
	return 0;
}

/*************************************************************************/

/*
 * This function is called when the module's state is updated, after setting
 * one or more of the group identity attributes of the current process.
 *
 * @param   new                 The set of credentials that will be installed.
 * @param   old                 The set of credentials that are being replaced.
 * @param   flags               Indicates which of the set*uid system calls invoked
 *                              this hook contains one of the LSM_SETID_* values.
 *
 * @return  0                   No error.
 *          -EPERM              Operation not permitted.
 *          -ENOENT             No such file or directory.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -ENAMETOOLONG       File name too long.
 *          -EBADE              Invalid exchange.
 *          -ENODATA            No data available.
 *          -EMSGSIZE           Message too long.
 */
static int secop_task_fix_setgid(struct cred *new,
				 const struct cred *old,
				 int flags)
{
	int i;
	int rc = 0;
	int current_pid = util_get_tgid(current);
	char name[BIDE_MAX_FILE_PATH];
	char msg[EXTRA_XML_MESSAGE_LEN] = {0};
	char *p = msg;
	gid_info_t acquiring[2];
	int num_acquired = 0;
	struct group_info *gi;

	STAT_ADD(stat_secop_task_fix_setgid);

	/* Track changes in permission.
	 * A process that no longer has the ROOT gid is
	 * no longer authorized for ROOT gids
	 */
	if (gid_eq(old->gid, ROOT_KGID) || gid_eq(old->egid, ROOT_KGID))
		if (!gid_eq(new->gid, ROOT_KGID) && !gid_eq(new->egid, ROOT_KGID))
			auth_drop_pid_permission(current_pid,
						 AUTH_PERM_PRIVILEGED_GID);

	/* A process that is re-acquiring ROOT gid from its sgid
	 * is once again authorized
	 */
	if (!gid_eq(old->gid, ROOT_KGID) && !gid_eq(old->egid, ROOT_KGID))
		if (gid_eq(new->gid, ROOT_KGID) || gid_eq(new->egid, ROOT_KGID))
			if (gid_eq(old->sgid, ROOT_KGID))
				auth_add_pid(current_pid,
					     AUTH_PERM_PRIVILEGED_GID);


	/*
	 * If a process is initializing (by either init or zygote)
	 * we trust that the system knows what it is doing, and allow
	 * any gids to be set.
	 */
	if (proc_is_pid_initializing(current_pid))
		return 0;

	/* Also if a process is root, it can do as it pleases */
	if (uid_eq(old->uid, ROOT_KUID) || uid_eq(old->euid, ROOT_KUID))
		return 0;

	proc_task_get_name(current_pid, name, sizeof(name));
	gi = get_group_info(old->group_info);
	/* Figure out if the process is trying to gain a privileged GID.
	 * We allow this if the sgid is the privileged gid that is being gained,
	 * otherwise we complain
	 */
	for (i = 0; i < ARRAY_SIZE(PRIVILEGED_GROUPS); ++i) {
		unsigned types = 0;

		/*
		 * The saved GID is the current privileged group.
		 * If the process is trying to change into
		 * that privileged group it is just restoring
		 * its saved gid which we allow.
		 */
		if (gid_eq(old->sgid, PRIVILEGED_GROUPS[i]))
			continue;

		/*
		 * If the app is whitelisted for this GID we don't
		 * need to do any further checking
		 */
		if (is_app_whitelisted_for_gid(name, i))
			continue;

		if (!gid_eq(old->gid, new->gid)) {
			/* gid is changing... */
			if (gid_eq(new->gid, PRIVILEGED_GROUPS[i])) {
				/*...into a privileged group...*/
				if (!gid_eq(new->gid, old->egid) &&
				    (gi == NULL || !groups_search(gi, new->gid)) &&
				    CRED_UID(old, uid) != CRED_GID(new, gid) &&
				    CRED_UID(old, euid) != CRED_GID(new, gid)) {
					/*... that isn't the existing egid OR
					  one of its supplementary gids OR
					  the proc's UID */
					types |= GID_REAL;
				}
			}
		}

		if (!gid_eq(old->egid, new->egid)) {
			/* egid is changing... */
			if (gid_eq(new->egid, PRIVILEGED_GROUPS[i])) {
				/*...into a privileged group...*/
				if (!gid_eq(new->egid, old->gid) &&
				    (gi == NULL || !groups_search(gi, new->egid)) &&
				    CRED_UID(old, uid) != CRED_GID(new, egid) &&
				    CRED_UID(old, euid) != CRED_GID(new, egid)) {
					/*... that isn't the existing gid OR
					  one of its supplementary gids OR
					  the proc's UID */
					types |= GID_EFFECTIVE;
				}
			}
		}

		if (types) {
			acquiring[num_acquired].gid_types = types;
			acquiring[num_acquired].gid = PRIVILEGED_GROUPS[i];
			num_acquired++;
		}
	}
	put_group_info(gi);

	if (num_acquired) {
		logError("PID %d(%s) gaining privileged gid(s)", current_pid, name);
		for (i = 0; i < num_acquired; ++i) {
			if (acquiring[i].gid_types & GID_REAL) {
				logError("--- real gid (old/new) %d/%d",
					 CRED_GID(old, gid), CRED_GID(new, gid));
			}
			if (acquiring[i].gid_types & GID_EFFECTIVE) {
				logError("--- effective gid (old/new) %d/%d",
					 CRED_GID(old, egid), CRED_GID(new, egid));
			}
		}

		/* Don't report if snapshot was not taken */
		if (!ctl_snapshot_complete())
			return 0;

		rc = xml_escalated_gid_msg(p, sizeof(msg),
					   acquiring, num_acquired);
		if (rc >= 0) {
			p = msg;
		} else {
			p = NULL;
		}
		rc = report_incident(SEVERITY_CRITICAL,
				     SN_ESCALATED_GID,
				     p,
				     current);
		if (rc)
			logError("Failed on report_incident(SN_ESCALATED_GID). rc=%d.", -rc);
	}

	return 0;
}

/*************************************************************************/

/*
 * This function is called when a task's set of supplementary groups
 * is being changed.
 *
 * @param   old                 The task's current set of supplementary groups
 * @param   new                 The task's new set of supplementary groups.
 *
 * @return  0                   No error.
 *          -EPERM              Operation not permitted.
 *          -ENOENT             No such file or directory.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -ENAMETOOLONG       File name too long.
 *          -EBADE              Invalid exchange.
 *          -ENODATA            No data available.
 *          -EMSGSIZE           Message too long.
 */
static int secop_task_set_groups(struct group_info *old,
				 struct group_info *new)
{
	/*
	 * If the process is trying to get into a privileged group
	 * verify it is allowed to do so.
	 *
	 * We assume that the init process and zygotes know what
	 * they are doing while they are initializing things.
	 *
	 * We also assume that is the only time that something can
	 * enter a privileged group. Any other times should
	 * be reported.
	 */
	int i;
	gid_info_t acquiring[ARRAY_SIZE(PRIVILEGED_GROUPS)];
	int num_acquired = 0;
	const struct cred *cred = current_cred();
	int current_pid = util_get_tgid(current);
	char name[BIDE_MAX_FILE_PATH];

	STAT_ADD(stat_secop_task_set_groups);

	/* If a process is initializing (by either init or zygote)
	 * we trust that the system knows what it is doing, and allow
	 * any gids to be set.
	 */
	if (proc_is_pid_initializing(current_pid))
		return 0;

	/* Also if a process is root, it can do as it pleases */
	if (uid_eq(cred->uid, ROOT_KUID) || uid_eq(cred->euid, ROOT_KUID))
		return 0;


	proc_task_get_name(current_pid, name, sizeof(name));
	for (i = 0; i < ARRAY_SIZE(PRIVILEGED_GROUPS); ++i) {
		if (groups_search(new, PRIVILEGED_GROUPS[i]) &&
		    (!groups_search(old, PRIVILEGED_GROUPS[i]) &&
		     !gid_eq(cred->gid, PRIVILEGED_GROUPS[i]) &&
		     !gid_eq(cred->egid, PRIVILEGED_GROUPS[i]) &&
		     !is_app_whitelisted_for_gid(name, i)))
		{
			/* The task is trying to gain a privileged gid */
			acquiring[num_acquired].gid_types = GID_SUPPLEMENTARY;
			acquiring[num_acquired].gid = PRIVILEGED_GROUPS[i];
			++num_acquired;
		}
	}

	if (num_acquired) {
		char msg[EXTRA_XML_MESSAGE_LEN] = {0};
		char *p = msg;
		int rc;

		logError("PID %d(%s) acquiring %d privileged supplementary group(s)",
			 current_pid, name, num_acquired);
		for (i = 0; i < num_acquired; ++i) {
			logError("--- Group %d == %d",
				 i+1, from_kgid(cred->user_ns, acquiring[i].gid));
		}

		/* Don't report if snapshot isn't complete */
		if (!ctl_snapshot_complete())
			return 0;

		rc = xml_escalated_gid_msg(p, sizeof(msg),
					   acquiring,
					   num_acquired);
		p = (rc >= 0) ? msg : NULL;
		rc = report_incident(SEVERITY_CRITICAL,
				     SN_ESCALATED_GID,
				     p,
				     current);
		if (rc)
			logError("Failed on report_incident(SN_ESCALATED_GID). rc=%d.", -rc);
	}

	return 0;
}

/*
 * Tidy up after the installation of the new security attributes of a
 * process being transformed by an execve operation.  The new credentials
 * have, by this point, been set to @current->cred.  @bprm points to the
 * linux_binprm structure.  This hook is a good place to perform state
 * changes on the process such as clearing out non-inheritable signal
 * state.  This is called immediately after commit_creds().
 *
 * @param   bprm                Contains the linux_binprm structure.
 */
static void secop_bprm_committed_creds(struct linux_binprm *bprm)
{
	int pid = util_get_tgid(current);

	proc_task_exec(pid, bprm->interp);
	proc_task_init_complete(pid, util_get_current_sid());
}

/*************************************************************************/

/*
 * This function gets called when a mount operation is triggered.
 *
 * @param   dev_name            Name of the object being mounted.
 * @param   path                The mount point path.
 * @param   type                File system type.
 * @param   flags               Flags used during mount operation.
 * @param   data                File system specific data.
 *
 * @return  0                   No error.
 *          -EPERM              Operation not permitted.
 *          -ENOENT             No such file or directory.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -ENAMETOOLONG       File name too long.
 *          -EBADE              Invalid exchange.
 *          -ENODATA            No data available.
 *          -EMSGSIZE           Message too long.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
static int secop_sb_mount(const char *dev_name,
			  struct path *path,
			  const char *type,
			  unsigned long flags,
			  void *data)
#else
static int secop_sb_mount(char *dev_name,
			  struct path *path,
			  char *type,
			  unsigned long flags,
			  void *data)
#endif
{
	struct paths_struct {
		char *p;
		unsigned sz;
	};

	/* Paths contain a trailing '/' and NULL character. The following
	 * size calculation strips the NULL (-1) and trailing '/' (-2) because
	 * what follows is a numerical identifier that denotes a user id.
	 *
	 * ie. /storage/emulated/10
	 */
	static const struct paths_struct PATHS[] = {
		{ INTERNAL_STORAGE, sizeof(INTERNAL_STORAGE) - 1 },
		{ EXTERNAL_STORAGE, sizeof(EXTERNAL_STORAGE) - 1 },
		{ INTERNAL_STORAGE, sizeof(INTERNAL_STORAGE) - 2 },
		{ EXTERNAL_STORAGE, sizeof(EXTERNAL_STORAGE) - 2 },
		{ AD_SECURE_STORAGE, sizeof(AD_SECURE_STORAGE) - 1 },
		{ EXTERNAL_WRITE, sizeof(EXTERNAL_WRITE) - 1 },
		{ EXTERNAL_READ, sizeof(EXTERNAL_READ) - 1 },
		{ EXTERNAL_DEFAULT, sizeof(EXTERNAL_DEFAULT) - 1 },
		{ EXTERNAL_SD, sizeof(EXTERNAL_SD) - 1 },
		{ USER_MOUNT_PATH, sizeof(USER_MOUNT_PATH) - 2 },
		{ EXFAT_STORAGE, sizeof(EXFAT_STORAGE) - 1 },
		{ DEV_BLOCK_DM, sizeof(DEV_BLOCK_DM) - 1 } };
	char msg[EXTRA_XML_MESSAGE_LEN] = { 0 };
	char *p = msg;
	int rc = 0;
	int i = 0;

	STAT_ADD(stat_secop_sb_mount);

	/* mount called with no dev_name cannot be evaluated */
	if (NULL == dev_name)
		return 0;

	/* Allow until snapshot is taken */
	if (!ctl_snapshot_complete())
		return 0;

	/* Only do a check if NOSUID or NODEV is set */
	if ((flags & MS_NOSUID) &&
	    (flags & MS_NODEV))
		return 0;

	/* Allow the parent zygote processes to mount */
	if (auth_check_permission(util_get_tgid(current), AUTH_PERM_ZYGOTEMGR_CHILD) == 0)
		return 0;


	/* Check device name against known good paths */
	for (i = 0; i < COUNT_OF(PATHS); ++i) {
		if (strncmp(PATHS[i].p, dev_name, PATHS[i].sz) == 0)
			return 0;
	}

	rc = xml_mount_path_msg(msg, sizeof(msg), dev_name);
	if (rc < 0) {
		logError("Failed on xml_mount_path_msg(). rc=%d.", -rc);
		p = NULL;
	}


	/* Create a report on this incident */
	if(!(flags & MS_NODEV)) {
		rc = report_incident(SEVERITY_CRITICAL,
				SN_NODEV,
				p,
				current);

		if (rc)
			logError("Failed on report_incident(MS_NODEV). rc=%d.", -rc);
	}

	if(!(flags & MS_NOSUID)) {
		rc = report_incident(SEVERITY_CRITICAL,
				SN_NOSUID,
				p,
				current);

		if (rc)
			logError("Failed on report_incident(SN_NOSUID). rc=%d.", -rc);
	}

	/* Allow */
	return 0;
}

/*************************************************************************/

/*
 * This callback function is called when a client process attempts to use
 * mmap() on a file or memory.
 *
 * @param   file                File structure for map.
 * @param   reqprot             Protection options requested.
 * @param   prot                Protection options that will be applied.
 * @param   flags               Operational flags.
 * @param   addr                The virtual address that will be used.
 * @param   addr_only           A boolean denoting if file-backed VMA or not.
 *
 * @return  0                   No error.
 *          -EPERM              Operation not permitted.
 *          -ENOENT             No such file or directory.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -ENAMETOOLONG       File name too long.
 *          -EBADE              Invalid exchange.
 *          -ENODATA            No data available.
 *          -EMSGSIZE           Message too long.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
static int secop_mmap_file(struct file *file,
			   unsigned long reqprot,
			   unsigned long prot,
			   unsigned long flags)
#else
static int secop_file_mmap(struct file *file,
			   unsigned long reqprot,
			   unsigned long prot,
			   unsigned long flags,
			   unsigned long addr,
			   unsigned long addr_only)
#endif
{
	int rc = 0;

	STAT_ADD(stat_secop_file_mmap);

	/* Check that the minimum mmap address has not changed */
	if (mmap_min_addr < CONFIG_DEFAULT_MMAP_MIN_ADDR) {
		/* Create a report on this incident */
		rc = report_incident(SEVERITY_CRITICAL,
				     SN_LOW_MMAP_ADDR,
				     NULL,
				     current);
		if (rc)
			logError("Failed on report_incident(SN_LOW_MMAP_ADDR). rc=%d.", -rc);
	}
	return 0;
}

/*************************************************************************/

/*
 * This function is called when a processes capabilities are changed.
 *
 * @param   new                 The new credentials for the current process.
 * @param   old                 The previous credentials for the current process.
 * @param   effective           The effective capabilities being set.
 * @param   inheritable         The inheritable capabilities being set.
 * @param   permitted           The permitted capabilities being set.
 *
 * @return  0                   Permission allowed.
 *          -ENOMEM             Out of memory.
 *          -EPERM              Permission denied.
 */
static int secop_capset(struct cred *new,
			const struct cred *old,
			const kernel_cap_t *effective,
			const kernel_cap_t *inheritable,
			const kernel_cap_t *permitted)
{
	const struct process_info *pinfo;
	int pid;
	int initializing;
	kernel_cap_t denied;
	kernel_cap_t new_inh;
	kernel_cap_t new_per;
	int i;

	STAT_ADD(stat_secop_capset);

	pid = util_get_tgid(current);
	pinfo = proc_get_info(pid);
	if (!pinfo)
		return 0;
	initializing = (pinfo->flags & BIDE_PROC_FLAG_INITIALIZING) != 0;
	proc_put_info(pinfo);

	/*
	 * If the process is still initializing don't check capabilities
	 * as it may not be in the right security domain.
	 *
	 * Capabilities will automatically be checked once initialization
	 * is complete.
	 */
	if (initializing)
		return 0;

	/* We only care about capabilities that are being set in the
	 * inheritable and permitted blocks. For effective, we check everything
	 */
	for (i = 0; i < _KERNEL_CAPABILITY_U32S; ++i) {
		new_inh.cap[i] = inheritable->cap[i] & ~old->cap_inheritable.cap[i];
		new_per.cap[i] = permitted->cap[i] & ~old->cap_permitted.cap[i];
	}

	if (!caps_verify(pid, util_get_current_sid(),
			 effective, &new_inh, &new_per, &denied))
		caps_gen_report(pid, effective, inheritable, permitted, &denied);

	return 0;
}

/*************************************************************************/

/*
 * This function is a hook for the mprotect() function.
 *
 * @param   vma                 The virtual memory segment being modified.
 * @param   reqprot             The protection level requested.
 * @param   prot                The current protection level.
 *
 * @return  0                   No error.
 *          -EPERM              Operation not permitted.
 *          -ENOENT             No such file or directory.
 *          -EAGAIN             Another hook has been installed already.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -ENAMETOOLONG       File name too long.
 *          -EBADE              Invalid exchange.
 *          -ENODATA            No data available.
 *          -EMSGSIZE           Message too long.
 */
static int secop_file_mprotect(struct vm_area_struct *vma,
			       unsigned long reqprot,
			       unsigned long prot)
{
	struct prot_flag {
		int prot;
		int vm_flag;
	};

	static struct prot_flag FLAGS[] = { { PROT_READ, MAY_READ },
					    { PROT_WRITE, MAY_WRITE },
					    { PROT_EXEC, MAY_EXEC } };
	int rep = 0;
	int i = 0;
	int rc = 0;

	STAT_ADD(stat_secop_file_mprotect);

	if (!ctl_snapshot_complete())
		return 0;

	if (vma->vm_flags & VM_SHARED ||
	    vma->vm_mm == current->mm)
		return 0;

	/* Check if the permission being added is allowed */
	for (i = 0; i < COUNT_OF(FLAGS); ++i) {
		rep |= (FLAGS[i].prot & reqprot &&
			FLAGS[i].prot & ~prot &&
			FLAGS[i].vm_flag & ~vma->vm_flags);
	}

	if (rep) {
		/* Send an incident report to JBIDE */
		rc = report_incident(SEVERITY_CRITICAL,
				     SN_MPROTECT,
				     NULL,
				     current);
		if (rc)
			logDebug("Failed on report_incident(SN_MPROTECT). rc=%d.", rc);
	}

	/* Allow */
	return 0;
}

/*************************************************************************/

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
#ifdef AVEN_44469_FIXED
/*
 * This function scans a section of contiguous virtual memory for the first
 * fragment in physical memory. The result is a pointer to physical memory
 * and the size of the physical memory section found. It is likely that
 * the virtual memory section is not contiguous in physical memory and the
 * returned physical memory section is smaller in size.
 *
 * @param   from                The start of the virtual memory section.
 * @param   to                  The first address beyond the end of the section.
 * @param   out                 The physical address of the 'from' variable.
 *
 * @return  0+                  Size of the physical memory section.
 *          -EINVAL             Invalid parameters.
 */
static int secop_module_phys_mem(void *from,
				 void *to,
				 void **out)
{
	unsigned inc = PAGE_SIZE;
	void *start = NULL;
	void *i = NULL;
	int size = 0;

	STAT_ADD(stat_secop_module_phys_mem);

	if (!from || !to || !out)
		return -EINVAL;

	if (from >= to)
		return 0;

	/* Loop while contiguous physical memory is found */
	for (i = from; i < to; i += inc) {
		struct page *page = vmalloc_to_page(i);
		void *phys = (void *) page_to_phys(page) + offset_in_page(i);

		inc = PAGE_SIZE - offset_in_page(i);
		if (i + inc > to)
			inc = to - i;

		/* Begin a new section */
		if (!start) {
			start = phys;
			size = inc;
			continue;
		}

		/* Check if contiguous and include the next page */
		if (phys == start + size) {
			size += inc;
			continue;
		}

		/* Section ended */
		break;
	}

	*out = start;

	return size;
};

/*************************************************************************/

/*
 * This function is called after a kernel module has been loaded into
 * memory and before it is initialized.
 *
 * @param   mod                 The module being loaded.
 *
 * @return  0                   Permission allowed.
 *          -EPERM              Permission denied.
 */
static int secop_kernel_module_init(struct module *mod)
{
	void *start = mod->module_core_rx;
	void *end = mod->module_core_rx + mod->core_size_rx;

	STAT_ADD(stat_secop_kernel_module_init);

	logDebug("Scanning module: %s", mod->name);

	/*
	 * Module memory is virtually contiguous but may be fragmented in
	 * physical memory. This loop will add each contiguous physical
	 * segment into trustzone.
	 */
	while (start < end) {
		void *phys = NULL;
		int rc = 0;

		int sz = secop_module_phys_mem(start, end, &phys);
		if (sz <= 0)
			break;

		rc = tz_add_section(phys, (unsigned) sz);
		if (rc) {
			logError("Failed on secop_kernel_module_init:tz_add_section(). rc=%d.", -rc);
			break;
		}

		start += sz;
	}

	return 0;
}

/*************************************************************************/

/*
 * This function is called before a kernel module is removed.
 *
 * @param   mod                 The module being unloaded.
 */
static void secop_kernel_module_free(struct module *mod)
{
	void *start = mod->module_core_rx;
	void *end = mod->module_core_rx + mod->core_size_rx;

	STAT_ADD(stat_secop_kernel_module_free);

	logDebug("Removing module: %s", mod->name);

/* Qualcomm issue open with TZ's inability to look at some regions of memory */
	/* Remove each physical, contiguous section of memory */
	while (start < end) {
		void *phys = NULL;
		int rc = 0;

		int sz = secop_module_phys_mem(start, end, &phys);
		if (sz <= 0)
			break;

		rc = tz_remove_section(phys, (unsigned) sz);
		if (rc) {
			logError("Failed on secop_kernel_module_free:tz_remove_section(). rc=%d.", -rc);
			break;
		}

		start += sz;
	}
}
#endif /* AVEN_44469_FIXED */
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */

/*************************************************************************/
static int secop_setprocattr(struct task_struct *p, char *name, void *value,
			      size_t size)
{
	char *str = value;
	int rc;
	u32 sid;

	if (current != p) {
		/* Ignore cross-process operations */
		return 0;
	}

	if (strcmp(name, "current")) {
		/* Ignore everything but the 'current' attribute */
		return 0;
	}

	if (size && str[1] && str[1] != '\n') {
		if (str[size-1] == '\n') {
			str[size-1] = '\0';
			--size;
		}

		/*
		 * There is a decent chance that the task's current credentials
		 * already have the right security context. It depends on
		 * whether selinux's hook for this callback got called first
		 * or not.
		 *
		 * Since I can't guarantee that it has I manually look up
		 * the security domain again to get the correct sid.
		 */
		if ((rc = security_context_to_sid(str, size, &sid, GFP_KERNEL))) {
			logError("Unable to find sid for security context %s",
				 str);
			return 0;
		}

		proc_task_init_complete(p->pid, sid);
	}

	return 0;
}

/*************************************************************************/

/*
 * Entry point during the security phase of initialization. This function
 * sets up hooks into the kernel to guard against privilage elevation or
 * unwanted processes.
 *
 * @return  0                   No Error.
 *          -EINVAL             Bad sec_ops structure.
 *          -EAGAIN             Another hook has been installed already.
 */
int __init secop_init(void)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	static struct security_hook_list hooks[] = {
		LSM_HOOK_INIT(task_free, 		secop_task_free),
		LSM_HOOK_INIT(task_created_notify,	secop_task_created_notify),
		LSM_HOOK_INIT(task_fix_setuid, 		secop_task_fix_setuid),
		LSM_HOOK_INIT(task_fix_setgid, 		secop_task_fix_setgid),
		LSM_HOOK_INIT(task_set_groups,		secop_task_set_groups),
		LSM_HOOK_INIT(sb_mount, 		secop_sb_mount),
		LSM_HOOK_INIT(mmap_file, 		secop_mmap_file),
		LSM_HOOK_INIT(capset, 			secop_capset),
		LSM_HOOK_INIT(bprm_set_creds, 		secop_bprm_set_creds),
		LSM_HOOK_INIT(bprm_committed_creds,     secop_bprm_committed_creds),
		LSM_HOOK_INIT(setprocattr,		secop_setprocattr),
#ifdef AVEN_44469_FIXED
		LSM_HOOK_INIT(kernel_module_init,	secop_kernel_module_init),
		LSM_HOOK_INIT(kernel_module_free,	secop_kernel_module_free),
#endif
		LSM_HOOK_INIT(file_mprotect,		secop_file_mprotect)
	};
#else
	static struct security_operations sec_ops = {
		.task_free		= secop_task_free,
		.task_fix_setuid	= secop_task_fix_setuid,
		.task_fix_setgid	= secop_task_fix_setgid,
		.task_set_groups	= secop_task_set_groups,
		.sb_mount		= secop_sb_mount,
		.file_mmap		= secop_file_mmap,
		.capset			= secop_capset,
		.bprm_set_creds		= secop_bprm_set_creds,
		.bprm_committed_creds   = secop_bprm_committed_creds,
	        .setprocattr		= secop_setprocattr
	};
#endif

	int rc = 0;

	/* Register ourselves with the security framework */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	security_add_hooks(hooks, ARRAY_SIZE(hooks));
#else
	rc = register_security(&sec_ops);
	if (rc) {
		logError("Failed on register_security(). rc=%d.", -rc);
		return -EINVAL;
	}
#endif

	logInfo("Security hooks installed.");

	return rc;
}

/*************************************************************************/

/*
 * Entry point for the module.
 *
 * @return  0                   No Error.
 */
int __exit secop_exit(void)
{
	return 0;
}
