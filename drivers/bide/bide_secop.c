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

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"

/*************************************************************************/

#define MAX_PATH			256
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
#define EXTERNAL_SD				"/mnt/runtime/allstar"
/* User Mount introduced in Android M*/
#define USER_MOUNT_PATH			"/mnt/user/"

/*************************************************************************/

/*
 * A security operation callback when a new task is created. This will
 * check permission before creating a child process.  Currently we are in
 * the context of the parent process and need to make decisions if we should
 * raise any alarms when a process is trying to fork a process.
 *
 * @param   flags           Determines what should be shared. See clone()
 *                          man page for definitations of the flags.
 *
 * @return  0               Permission allowed.
 *          -EPERM          Permission denied.
 */
static int secop_task_create(unsigned long flags)
{
	/* A fork is just a copy of the current program, which we already
	 * stated that we trusted.  We don't need to report on it, even
	 * if it's a root process */

	/* Allow */
	return 0;
}

/************************************************************************/

/*
 * A security operation callback when a task ends. This will be called
 * upon the final call to put_task_struct(), essentially removing the last
 * reference from that task.
 *
 * This funciton gets called while in an IRQ interrupt context. Any mutex
 * locking attempt will cause undefined behaviour.
 *
 * @param   task            The task that is exiting.
 */
static void secop_task_free(struct task_struct *task)
{
	struct list_head *p = NULL;
	int perm = 0;
	int rc = 0;

	/* Ignore kernel threads */
	if (!task || !task->mm)
		return;

	logInfo("secop_task_free() called.  task->exit_state=%d.", task->exit_state);
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
 * @param   bprm            Contains the linux_binprm structure.
 *
 * @return  0               if the hook is successful and permission is granted.
 *          -EPERM          Permission denied.
 */
static int secop_bprm_set_creds(struct linux_binprm *bprm)
{
	struct task_struct *parent = NULL;
	int rc = 0;
	int is_parent_kthread = 0;
	unsigned long ppid = 0;
	int current_pid = util_get_tgid(current);

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
	if (!auth_check_permission(current_pid, AUTH_PERM_PRIVILEGED_GID))
		return 0;

	/* If Parent can have privileged children, then promote this child */
	if (!auth_check_permission(ppid, AUTH_PERM_PRIVELEGED_CHILDREN) ||
	    !auth_check_parents_permission(current, AUTH_PERM_PRIVILEGED_LINEAGE)) {
		rc = auth_add_pid(current_pid, AUTH_PERM_PRIVILEGED_GID);
		if (rc)
			logError("Failed on auth_add_pid(). rc=%d.", -rc);

		return 0;
	}

	/* Allow system_server to launch child processes */
	if (!auth_check_permission(ppid, AUTH_PERM_PRIVILEGED_ZYGOTE))
		return 0;

	/* Check if Parent is the Kernel worker thread (DDT launched) */
	if (is_parent_kthread == 1) {
		rc = auth_add_pid(current_pid, AUTH_PERM_PRIVILEGED_GID);
		if (rc)
			logError("Failed on auth_add_pid() from secop_bprm_set_creds(). rc=%d.", -rc);

		return 0;
	}

	/* Promote root process to privilaged status before snapshot or
	 * if the parent is init. */
	if (!ctl_snapshot_complete() || ppid == INIT_PID) {
		rc = auth_add_pid(current_pid, AUTH_PERM_PRIVILEGED_GID);
		if (rc)
			logError("Failed on auth_add_pid(). rc=%d.", -rc);
	} else {
		rc = report_incident(SEVERITY_CRITICAL,
				     SN_ROOT_PROCESS_DETECTOR,
				     NULL,
				     current);
		if (rc)
			logDebug("Failed on report_incident() from secop_bprm_set_creds(). rc=%d", -rc);
	}

	/* Allow */
	return 0;
}

/*************************************************************************/

/*
 * This funciton is when the module's state is updated, after setting one or
 * more of the user identity attributes of the current process.
 *
 * @param   new             The set of credentials that will be installed.
 * @param   old             The set of credentials that are being replaced.
 * @param   flags           Indicates which of the set*uid system calls invoked
 *                          this hook contains one of the LSM_SETID_* values.
 *
 * @return  0               Permission allowed.
 *          -EPERM          Permission denied.
 *          -ENOMEM         Out of memory, permission denied.
 */
static int secop_task_fix_setuid(struct cred *new,
				 const struct cred *old,
				 int flags)
{
	int perm = 0;
	int rc = 0;
	unsigned long ppid = 0;
	const int ESCALATED_GID_ISSUE = 1;
	const int ROOT_UID_ISSUE = 2;
	const int SYSTEM_UID_ISSUE = 4;
	int issue_found = 0;
	int current_pid = util_get_tgid(current);

	if (old->uid % USER_UID == SYSTEM_UID &&
	    new->uid % USER_UID != SYSTEM_UID)
		perm |= AUTH_PERM_SYSTEM_UID;

	if (old->gid % USER_UID == ROOT_UID &&
	    new->gid % USER_UID != ROOT_UID)
		perm |= AUTH_PERM_PRIVILEGED_GID;

	/* Track downgrade in permission */
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
	if (!auth_check_permission(ppid, AUTH_PERM_PRIVELEGED_CHILDREN) ||
	    !auth_check_parents_permission(current, AUTH_PERM_PRIVILEGED_LINEAGE)) {
		rc = auth_add_pid(current_pid, AUTH_PERM_PRIVILEGED_GID);
		if (rc)
			logError("Failed on auth_add_pid(AUTH_PERM_PRIVILEGED_GID). rc=%d.", -rc);
	}

	/* Don't report if snapshot was not taken */
	if (!ctl_snapshot_complete())
		return 0;

	if (groups_search(new->group_info, ROOT_UID) ||
	    groups_search(new->group_info, SYSTEM_UID)){
		/* Check root permission was whitelisted */
		rc = auth_check_permission(current_pid, AUTH_PERM_PRIVILEGED_GID);
		if (rc)
			issue_found |= ESCALATED_GID_ISSUE;
	}

	if (new->uid % USER_UID == ROOT_UID ||
	    new->euid % USER_UID == ROOT_UID) {

		/* Check root permission was whitelisted */
		rc = auth_check_permission(current_pid, AUTH_PERM_PRIVILEGED_GID);
		if (rc)
			issue_found |= ROOT_UID_ISSUE;
	}

	if (new->uid % USER_UID == SYSTEM_UID) {
		/* Allow a root process to downgrade to System */
		if (old->uid % USER_UID == ROOT_UID) {
			rc = auth_add_pid(current_pid, AUTH_PERM_SYSTEM_UID);
		} else {
			/* Check that System permissions were whitelisted */
			rc = auth_check_permission(current_pid, AUTH_PERM_SYSTEM_UID);
			if (rc)
				issue_found |= SYSTEM_UID_ISSUE;
		}
	}

	if (issue_found & ESCALATED_GID_ISSUE) {
		rc = report_incident(SEVERITY_CRITICAL,
				     SN_ESCALATED_GID,
				     NULL,
				     current);
		if (rc)
			logError("Failed on report_incident(SN_ESCALATED_GID). rc=%d.", -rc);
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
 * This function gets called when a mount operation is triggered.
 *
 * @param   dev_name        Name of the object being mounted.
 * @param   path            The mount point path.
 * @param   type            File system type.
 * @param   flags           Flags used during mount operation.
 * @param   data            File system specific data.
 *
 * @return  0               Permission allowed.
 *          -EPERM          Permission denied.
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

	/* Allow until snapshot is taken */
	if (!ctl_snapshot_complete())
		return 0;

	/* Only do a check if NOSUID of NODEV is set */
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
 * @param   file            File structure for map.
 * @param   reqprot         Protection options requested.
 * @param   prot            Protection options that will be applied.
 * @param   flags           Operational flags.
 * @param   addr            The virtual address that will be used.
 * @param   addr_only       A boolean denoting if file-backed VMA or not.
 *
 * @return  0               Permission allowed.
 *          -EPERM          Permission denied.
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
 * This funciton is called when a processes capabilities are changed.
 *
 * @param   new             The new credentials for the current process.
 * @param   old             The previous credentials for the current process.
 * @param   effective       The effective capabilities being set.
 * @param   inheritable     The inheritable capabilities being set.
 * @param   permitted       The permitted capabilities being set.
 *
 * @return  0               Permission allowed.
 *          -ENOMEM         Out of memory.
 *          -EPERM          Permission denied.
 */
static int secop_capset(struct cred *new,
			const struct cred *old,
			const kernel_cap_t *effective,
			const kernel_cap_t *inheritable,
			const kernel_cap_t *permitted)
{
	int rc = 0;
	int i = 0;
	int allowed_caps = 0;

	/* Ignore all capsets before the snapshot is taken */
	if (!ctl_snapshot_complete()) {
		for (i = 0; i < _KERNEL_CAPABILITY_U32S && !rc; ++i) {
			rc |= effective->cap[i] |
				  inheritable->cap[i] |
				  permitted->cap[i];
		}

		if(rc != 0) {
			caps_update_process(util_get_tgid(current), rc);
		}

		return 0;
	}

	allowed_caps = caps_get_caps_for_process(util_get_tgid(current));

	/* Allow if all the capabilities are being set to 0 */
	for (i = 0; i < _KERNEL_CAPABILITY_U32S && !rc; ++i) {
		rc |= effective->cap[i] |
		      inheritable->cap[i] |
		      permitted->cap[i];
	}

	/* Check to see if the capabilites are allowed, ignore the capabilites that
	   were allowed based on bootup */
	if ((((~allowed_caps) & rc) != 0) &&
	    (auth_check_capabilities(new->uid, rc))) {
		char msg[EXTRA_XML_MESSAGE_LEN] = { 0 };
		char *p = msg;

		/* Compile a capset message */
		rc = xml_capset_msg(msg,
				    EXTRA_XML_MESSAGE_LEN,
				    effective,
				    inheritable,
				    permitted,
				    CAP_NEW);
		if (rc < 0)
			p = NULL;

		/* Send an incident report to JBIDE */
		rc = report_incident(SEVERITY_CRITICAL,
				     SN_CAPSET,
				     p,
				     current);
		if (rc)
			logDebug("Failed on report_incident(SN_CAPSET). rc=%d.", rc);
	}

	/* Allow */
	return 0;
}

/*************************************************************************/

/*
 * This function is a hook for the mprotect() function.
 *
 * @param   vma             The virtual memory segment being modified.
 * @param   reqprot         The protection level requested.
 * @param   prot            The current protection level.
 *
 * @return  0               No Error.
 *          -EINVAL         Bad sec_ops structure.
 *          -EAGAIN         Another hook has been installed already.
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
 * @param   from            The start of the virtual memory section.
 * @param   to              The first address beyond the end of the section.
 * @param   out             The physical address of the 'from' variable.
 *
 * @return  0+              Size of the physical memory section.
 *          -EINVAL         Invalid parameters.
 */
static int secop_module_phys_mem(void *from,
				 void *to,
				 void **out)
{
	unsigned inc = PAGE_SIZE;
	void *start = NULL;
	void *i = NULL;
	int size = 0;

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
 * @param   mod             The module being loaded.
 *
 * @return  0               Permission allowed.
 *          -EPERM          Permission denied.
 */
static int secop_kernel_module_init(struct module *mod)
{
	void *start = mod->module_core_rx;
	void *end = mod->module_core_rx + mod->core_size_rx;

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
			logError("Failed on tz_add_section(). rc=%d.", -rc);
			break;
		}

		start += sz;
	}

	return 0;
}

/*************************************************************************/

/*
 * This funciton is called before a kernel module is removed.
 *
 * @param   mod             The module being unloaded.
 */
static void secop_kernel_module_free(struct module *mod)
{
	void *start = mod->module_core_rx;
	void *end = mod->module_core_rx + mod->core_size_rx;

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
			logError("Failed on tz_add_section(). rc=%d.", -rc);
			break;
		}

		start += sz;
	}
}
#endif /* AVEN_44469_FIXED */
#endif /* LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0) */

/*************************************************************************/

/*
 * Entry point during the security phase of initialization. This function
 * sets up hooks into the kernel to guard against privilage elevation or
 * unwanted processes.
 *
 * @return  0               No Error.
 *          -EINVAL         Bad sec_ops structure.
 *          -EAGAIN         Another hook has been installed already.
 */
int __init secop_init(void)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	static struct security_hook_list hooks[] = {
		LSM_HOOK_INIT(task_create, 		secop_task_create),
		LSM_HOOK_INIT(task_free, 		secop_task_free),
		LSM_HOOK_INIT(task_fix_setuid, 		secop_task_fix_setuid),
		LSM_HOOK_INIT(sb_mount, 		secop_sb_mount),
		LSM_HOOK_INIT(mmap_file, 		secop_mmap_file),
		LSM_HOOK_INIT(capset, 			secop_capset),
		LSM_HOOK_INIT(bprm_set_creds, 		secop_bprm_set_creds),
#ifdef AVEN_44469_FIXED
		LSM_HOOK_INIT(kernel_module_init,	secop_kernel_module_init),
		LSM_HOOK_INIT(kernel_module_free,	secop_kernel_module_free),
#endif
		LSM_HOOK_INIT(file_mprotect,		secop_file_mprotect)
	};
#else
	static struct security_operations sec_ops = {
		.task_create		= secop_task_create,
		.task_free		= secop_task_free,
		.task_fix_setuid	= secop_task_fix_setuid,
		.sb_mount		= secop_sb_mount,
		.file_mmap		= secop_file_mmap,
		.capset			= secop_capset,
		.bprm_set_creds		= secop_bprm_set_creds
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
 * @return  0               No Error.
 *          -EINVAL         Security struct was not registered prior.
 */
int __exit secop_exit(void)
{
	return 0;
}
