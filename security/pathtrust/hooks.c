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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/audit.h>
#include <linux/binfmts.h>
#include <linux/errno.h>
#include <linux/genhd.h>
#include <linux/glob.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <linux/mman.h>
#include <linux/module.h>
#include <linux/mount.h>
#include <linux/pathtrust.h>
#include <linux/personality.h>
#include <linux/security.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/lsm_hooks.h>

/* Access to task_security_struct selinux internal */
#include <objsec.h>
/* Access to security_sid_to_context selinux internal */
#include <security.h>

#include "pathtrust_private.h"

static bool pathtrust_path_match(const char *pathname);

/*
 * Pathtrust forbidden capabilities for euid|egid!=0
 * returns true if the current user has any of the banned capabilities
 */
bool has_banned_caps(void)
{
	return (capable(CAP_CHOWN) ||
			capable(CAP_DAC_OVERRIDE) ||
			capable(CAP_DAC_READ_SEARCH) ||
			capable(CAP_FOWNER) ||
			capable(CAP_MAC_ADMIN) ||
			capable(CAP_MAC_OVERRIDE) ||
			capable(CAP_MKNOD) ||
			capable(CAP_SETGID) ||
			capable(CAP_SETUID) ||
			capable(CAP_SYS_ADMIN) ||
			capable(CAP_SYS_MODULE) ||
			capable(CAP_SYS_PTRACE) ||
			capable(CAP_SYS_RAWIO));
}

/*
 * Dumps the type of pathtrust security op triggered
 */
static void pathtrust_dump_audit_type(struct audit_buffer *ab,
		struct common_audit_data *ad)
{
	if (!ab || !ad)
		return;

	switch (ad->pathtrust_audit_data->type) {

	case PATHTRUST_AUDIT_TYPE_EXEC:
		audit_log_format(ab, "execution ");
		break;
	case PATHTRUST_AUDIT_TYPE_MMAP:
		audit_log_format(ab, "exec mmap ");
		break;
	case PATHTRUST_AUDIT_TYPE_KMOD:
		audit_log_format(ab, "module loading ");
		break;
	case PATHTRUST_AUDIT_TYPE_KMODA:
		audit_log_format(ab, "anonymous module loading ");
		break;
	case PATHTRUST_AUDIT_TYPE_MOUNT:
		audit_log_format(ab, "'trusted' mount ");
		break;
	case PATHTRUST_AUDIT_TYPE_WHITELIST:
		audit_log_format(ab, "whitelist 'trusted' device ");
		break;
	case PATHTRUST_AUDIT_TYPE_IOCTL:
		audit_log_format(ab, "ioctl ");
		break;
	case PATHTRUST_AUDIT_TYPE_FIRM:
		audit_log_format(ab, "firmware loading ");
		break;
	default:
		break;
	}
}

/*
 * Dumps the block type trigger on the security op
 */
static void pathtrust_dump_audit_block_type(struct audit_buffer *ab,
		struct common_audit_data *ad)
{
	if (!ab || !ad)
		return;

	switch (ad->pathtrust_audit_data->btype) {

	case PATHTRUST_AUDIT_BTYPE_NONE:
		break;
	case PATHTRUST_AUDIT_BTYPE_ROOT:
		audit_log_format(ab, "(root)");
		break;
	case PATHTRUST_AUDIT_BTYPE_CAPS:
		audit_log_format(ab, "(caps)");
		break;
	case PATHTRUST_AUDIT_BTYPE_SELCTX:
		audit_log_format(ab, "(selctx)");
		break;
	default:
		break;
	}

	if (ad->pathtrust_audit_data->extra)
			audit_log_format(ab, "[%s]", ad->pathtrust_audit_data->extra);
}

/*
 * Dumps the block type trigger on the security op
 */
static void pathtrust_dump_devt(struct audit_buffer *ab,
		struct common_audit_data *ad)
{
	dev_t dev;

	if (!ab || !ad)
		return;

	dev = ad->pathtrust_audit_data->dev;

	if (dev)
		audit_log_format(ab, "devt(%u,%u)", MAJOR(dev), MINOR(dev));
}

/**
 * pathtrust_audit_pre_callback - Pathtrust specific information
 * will be called by generic audit code
 * @ab: the audit buffer
 * @a: audit_data
 */
static void pathtrust_audit_pre_callback(struct audit_buffer *ab, void *a)
{
	struct common_audit_data *ad = a;

	if (!ab || !a)
		return;

	if (!pathtrust_enforce)
		audit_log_format(ab, "pathtrust: %s ",
			(ad->pathtrust_audit_data->allowed ? "allowed" : "denied (non-enforced)"));
	else
		audit_log_format(ab, "pathtrust: %s ",
					(ad->pathtrust_audit_data->allowed ? "allowed" : "denied"));
	pathtrust_dump_audit_type(ab, ad);
	pathtrust_dump_audit_block_type(ab, ad);
	pathtrust_dump_devt(ab, ad);
}

/**
 * pathtrust_audit_post_callback - Pathtrust specific information
 * will be called by generic audit code
 * @ab: the audit buffer
 * @a: audit_data
 */
static void pathtrust_audit_post_callback(struct audit_buffer *ab, void *a)
{
	return;
}

/*
 * Pathtrust common LSM audit print function
 * @cad: audit data
 */
void pathtrust_lsm_audit(struct common_audit_data *cad) {
	common_lsm_audit(cad, pathtrust_audit_pre_callback,
		pathtrust_audit_post_callback);
}
/*
 * Mount security operation
 * If user requested the mount as MS_TRUSTED,
 * only allow mount if device is whitelisted in
 * pathtrust
 */
static int pathtrust_sb_mount(const char *dev_name,
			 struct path *path,
			 const char *type,
			 unsigned long flags,
			 void *data)
{
	int res = 0;
	struct block_device *bdev;
	struct common_audit_data cad;
	struct pathtrust_audit_data pt_data = {
			.type = PATHTRUST_AUDIT_TYPE_MOUNT,
			.btype = PATHTRUST_AUDIT_BTYPE_NONE,
			.allowed = true };

	memset(&cad, 0, sizeof(cad));
	cad.type = LSM_AUDIT_DATA_PATH;
	cad.pathtrust_audit_data = &pt_data;
	cad.u.path = *path;

	if (flags & MS_TRUSTED) {

		bdev = lookup_bdev(dev_name);
		if (IS_ERR(bdev))
			return -ENOENT;

		pt_data.dev = bdev->bd_dev;
		if (!pathtrust_dev_trusted(bdev->bd_dev)) {
			res = -EPERM;
			pt_data.allowed = false;
		}
		pathtrust_lsm_audit(&cad);
	}

	return res;
}

/*
 * Kernel Mount security operation
 * If user requested the mount as MS_TRUSTED,
 * only allow mount if device is whitelisted in
 * pathtrust
 *
 * This is an extra check to make sure no one altered
 * the block device between the do_mount and kern_mount
 */
static int pathtrust_sb_kern_mount(struct super_block *sb,
		int flags, void *data)
{
	int res = 0;
	struct common_audit_data cad;
	struct pathtrust_audit_data pt_data = {
			.type = PATHTRUST_AUDIT_TYPE_MOUNT,
			.btype = PATHTRUST_AUDIT_BTYPE_NONE,
			.allowed = true };

	memset(&cad, 0, sizeof(cad));
	cad.type = LSM_AUDIT_DATA_DENTRY;
	cad.pathtrust_audit_data = &pt_data;
	cad.u.dentry = sb->s_root;

	if (flags & MS_TRUSTED) {

		pt_data.dev = sb->s_dev;

		if (!pathtrust_dev_trusted(sb->s_dev)) {
			res = -EPERM;
			pt_data.allowed = false;
		}

		pathtrust_lsm_audit(&cad);
	}
	return res;
}

/*
 * Returns true if user is considered root
 */
bool is_root(const struct cred *cred)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0)
	if (uid_eq(cred->euid, GLOBAL_ROOT_UID) ||
		gid_eq(cred->egid, GLOBAL_ROOT_GID))
#else
	if (cred->euid == 0 || cred->egid == 0)
#endif
		return true;

	return false;
}

/*
 * Execute security operation
 * Block execution if file comes from a non trusted filesystem and:
 * - The user is euid == 0 | egid == 0; or
 * - The user is not euid == 0 & egid == 0 and has one or more
 *   of the banned capabilities
 * - The user is not executing with a forbidden selinux domain context
 */
static int pathtrust_bprm_set_creds(struct linux_binprm *bprm)
{
	struct common_audit_data cad;
	struct pathtrust_audit_data pt_data = { .allowed = false, .dev = 0 };
	const struct task_security_struct *tsec = bprm->cred->security;
	bool fsid;
	char *fcontext = NULL;
	int fcontext_len;

	fsid = is_forbidden_sid(tsec->sid);

	/* If executed as root user */
	if (!(bprm->file->f_path.mnt->mnt_flags & MNT_TRUSTED) &&
			(is_root(bprm->cred) ||
			 has_banned_caps() ||
			 fsid)) {

		memset(&cad, 0, sizeof(cad));
		cad.type = LSM_AUDIT_DATA_PATH;
		cad.pathtrust_audit_data = &pt_data;
		pt_data.type = PATHTRUST_AUDIT_TYPE_EXEC;
		if (is_root(bprm->cred)) {
			pt_data.btype = PATHTRUST_AUDIT_BTYPE_ROOT;
		}
		else if (fsid) {
			pt_data.btype = PATHTRUST_AUDIT_BTYPE_SELCTX;
			if (!security_sid_to_context(tsec->sid, &fcontext, &fcontext_len)) {
				pt_data.extra = fcontext;
			}
		}
		else {
			pt_data.btype = PATHTRUST_AUDIT_BTYPE_CAPS;
		}

		cad.u.path = bprm->file->f_path;
		pathtrust_lsm_audit(&cad);

		if (fsid)
			kfree(fcontext);

		if (!pathtrust_enforce)
			return 0;

		return -EPERM;
	}

	return 0;
}

/*
 * Mmap security operation
 * Block PROT_EXEC mmap if file comes from a non trusted filesystem and:
 * - The user is euid == 0 | egid == 0; or
 * - The user is not euid == 0 & egid == 0 and has one or more
 *   of the banned capabilities
 * - The user is not executing with a forbidden selinux domain context
 */
static int pathtrust_mmap_file(struct file *file, unsigned long reqprot,
				unsigned long prot, unsigned long flags)
{
	const struct cred *cred = current_cred();
	const struct task_security_struct *tsec = cred->security;
	struct common_audit_data cad;
	struct pathtrust_audit_data pt_data = { .allowed = false, .dev = 0 };
	char *filename = NULL;
	char *pathname;
	int res = 0;
	bool fsid;
	char *fcontext = NULL;
	int fcontext_len;

	if (file && ((prot & PROT_EXEC) ||
			((prot & PROT_READ) &&
			(current->personality
				& READ_IMPLIES_EXEC)))) {

		fsid = is_forbidden_sid(tsec->sid);
		if (!(file->f_path.mnt->mnt_flags & MNT_TRUSTED) &&
				(is_root(cred) || has_banned_caps() || fsid)) {
#if 0
			/* No process should have a different mount namespace
			 * than init
			 */
			if (init_task.nsproxy->mnt_ns !=
					current->nsproxy->mnt_ns) {
				memset(&cad, 0, sizeof(cad));
				cad.type = LSM_AUDIT_DATA_PATH;
				cad.pathtrust_audit_data = &pt_data;
				pt_data.type = PATHTRUST_AUDIT_TYPE_MMAP;
				pt_data.btype = PATHTRUST_AUDIT_BTYPE_MNTNS;
				cad.u.path = file->f_path;
				pathtrust_lsm_audit(&cad);
				res = -EPERM;
				goto out;
			}
#endif

			/* Get pathname of file */
			filename = kmalloc(PATH_MAX+11, GFP_KERNEL);

			if (!filename) {
				res = -ENOMEM;
				goto out;
			}

			pathname = d_path(&file->f_path, filename, PATH_MAX+11);

			if (IS_ERR(pathname)) {
				res = -EINVAL;
				goto clean;
			}

			if (pathtrust_debug)
				pr_err("mmap non-trusted file '%s'", pathname);

			/* Pathname pattern match means it's allowed */
			if (pathtrust_path_match(pathname))
				goto clean;

			memset(&cad, 0, sizeof(cad));
			cad.type = LSM_AUDIT_DATA_PATH;
			cad.pathtrust_audit_data = &pt_data;
			pt_data.type = PATHTRUST_AUDIT_TYPE_MMAP;
			if (is_root(cred)) {
				pt_data.btype = PATHTRUST_AUDIT_BTYPE_ROOT;
			}
			else if (fsid) {
				pt_data.btype = PATHTRUST_AUDIT_BTYPE_SELCTX;
				if (!security_sid_to_context(tsec->sid, &fcontext, &fcontext_len)) {
					pt_data.extra = fcontext;
				}
			}
			else {
				pt_data.btype = PATHTRUST_AUDIT_BTYPE_CAPS;
			}

			cad.u.path = file->f_path;
			pathtrust_lsm_audit(&cad);

			if (fsid)
				kfree(fcontext);

			if (!pathtrust_enforce) {
				res = 0;
				goto clean;
			}

			res = -EPERM;
		}
	}

clean:
	kfree(filename);

out:
	return res;
}

/*
 * Kernel firmware loading through request_firmware
 * Only allow firmware loading from trusted filesystems
 */
int pathtrust_kernel_fw_from_file(struct file *file, char *buf, size_t size)
{
#ifdef CONFIG_SECURITY_PATHTRUST_FIRMWARE
	struct common_audit_data cad;
	struct pathtrust_audit_data pt_data = { .allowed = false, .dev = 0 };

	if (file && !(file->f_path.mnt->mnt_flags & MNT_TRUSTED)) {

		memset(&cad, 0, sizeof(cad));
		cad.type = LSM_AUDIT_DATA_PATH;
		cad.pathtrust_audit_data = &pt_data;
		pt_data.type = PATHTRUST_AUDIT_TYPE_FIRM;
		pt_data.btype = PATHTRUST_AUDIT_BTYPE_NONE;
		cad.u.path = file->f_path;
		pathtrust_lsm_audit(&cad);

		// don't move following, put here to get dmesg feedback
		if (!pathtrust_enforce)
			return 0;

		return -EPERM;
	}
#endif
	return 0;
}

/*
 * Kernel module loading through finit_module system call
 * Only allow kernel module loading from trusted filesystems
 * Disallow anonymous kernel module loading (init_module syscall)
 * since we can't tell from where the module comes from
 */
int pathtrust_kernel_module_from_file(struct file *file)
{
	struct common_audit_data cad;
	struct pathtrust_audit_data pt_data = { .allowed = false, .dev = 0 };

	if (!file) {

		memset(&cad, 0, sizeof(cad));
		cad.type = LSM_AUDIT_DATA_TASK;
		cad.u.tsk = current;
		cad.pathtrust_audit_data = &pt_data;
		pt_data.type = PATHTRUST_AUDIT_TYPE_KMODA;
		pt_data.btype = PATHTRUST_AUDIT_BTYPE_NONE;
		pathtrust_lsm_audit(&cad);

		// don't move following, put here to get dmesg feedback
		if (!pathtrust_enforce)
			return 0;

		return -EPERM;

	} else if (!(file->f_path.mnt->mnt_flags & MNT_TRUSTED)) {

		memset(&cad, 0, sizeof(cad));
		cad.type = LSM_AUDIT_DATA_PATH;
		cad.pathtrust_audit_data = &pt_data;
		pt_data.type = PATHTRUST_AUDIT_TYPE_KMOD;
		pt_data.btype = PATHTRUST_AUDIT_BTYPE_NONE;
		cad.u.path = file->f_path;
		pathtrust_lsm_audit(&cad);

		// don't move following, put here to get dmesg feedback
		if (!pathtrust_enforce)
			return 0;

		return -EPERM;
	}

	return 0;
}

/*
 * Pathtrust security operations
 */
static struct security_hook_list pathtrust_hooks[] = {
	LSM_HOOK_INIT(sb_mount, pathtrust_sb_mount),
	LSM_HOOK_INIT(sb_kern_mount, pathtrust_sb_kern_mount),
	LSM_HOOK_INIT(bprm_set_creds, pathtrust_bprm_set_creds),
	LSM_HOOK_INIT(mmap_file, pathtrust_mmap_file),
	LSM_HOOK_INIT(kernel_fw_from_file, pathtrust_kernel_fw_from_file),
	LSM_HOOK_INIT(kernel_module_from_file, pathtrust_kernel_module_from_file),
};

static LIST_HEAD(pathtrust_devnode_list);
static DEFINE_SPINLOCK(pathtrust_devnode_list_lock);

/*
 * Add a new whitelisted device
 */
int pathtrust_add_dev(dev_t dev)
{
	int res = 0;
	struct pathtrust_devnode *newnode;
	struct common_audit_data cad;
	struct pathtrust_audit_data pt_data = {
			.type = PATHTRUST_AUDIT_TYPE_WHITELIST,
			.btype = PATHTRUST_AUDIT_BTYPE_NONE,
			.dev = dev };
	cad.type = LSM_AUDIT_DATA_NONE;
	cad.pathtrust_audit_data = &pt_data;

	if (pathtrust_dev_trusted(dev))
		return 0;

	newnode = kzalloc(sizeof(*newnode), GFP_KERNEL);

	if (unlikely(!newnode)) {
		res = -ENOMEM;
		pr_err("no memory for dev node\n");
		goto out;
	}

	newnode->dev = dev;

	spin_lock(&pathtrust_devnode_list_lock);
	INIT_LIST_HEAD(&newnode->lhead);
	list_add_tail(&newnode->lhead, &pathtrust_devnode_list);
	spin_unlock(&pathtrust_devnode_list_lock);
out:

	pt_data.allowed = (res) ? false : true;
	pathtrust_lsm_audit(&cad);

	return res;
}

EXPORT_SYMBOL_GPL(pathtrust_add_dev);

/*
 * Check if a device is whitelisted or not
 */
bool pathtrust_dev_trusted(dev_t dev)
{
	struct pathtrust_devnode *node;
	bool res = false;

#ifdef CONFIG_SECURITY_PATHTRUST_WHITELIST_IMPLICIT
	return true;
#endif

	spin_lock(&pathtrust_devnode_list_lock);
	list_for_each_entry(node, &pathtrust_devnode_list, lhead) {
		if (node->dev == dev) {
			pr_debug("found existing device (%u,%u)",
					MAJOR(dev), MINOR(dev));
			res = true;
			goto out;
		}
	}

out:
	spin_unlock(&pathtrust_devnode_list_lock);

	return res;
}

EXPORT_SYMBOL_GPL(pathtrust_dev_trusted);


static LIST_HEAD(pathtrust_pathnode_list);

/*
 * Add whitelisted pathnames
 * Note: No list protection required, this is only done once at init time
 */
static int pathtrust_add_path(const char *pathname)
{
	int res = 0;
	struct pathtrust_pathnode *newnode;

	if (!pathname)
		return -EINVAL;

	newnode = kzalloc(sizeof(*newnode), GFP_KERNEL);

	if (unlikely(!newnode)) {
		res = -ENOMEM;
		pr_err("no memory for path node\n");
		goto out;
	}

	newnode->pathname = kstrdup(pathname, GFP_KERNEL);

	if (unlikely(!(newnode->pathname))) {
		res = -ENOMEM;
		pr_err("no memory pathname kdupstr\n");
		goto error;
	}

	INIT_LIST_HEAD(&newnode->lhead);
	list_add_tail(&newnode->lhead, &pathtrust_pathnode_list);
	goto out;

error:
	kfree(newnode);

out:
	pr_info("added trusted pathname: %s\n", pathname);

	return res;
}

/*
 * Pathtrust whitelisted file pattern matching
 * Returns true if pathname matches
 */
static bool pathtrust_path_match(const char *pathname)
{
	struct pathtrust_pathnode *node;

	if (!pathname)
		return false;

	list_for_each_entry(node, &pathtrust_pathnode_list, lhead) {
		if (glob_match(node->pathname, pathname)) {
			pr_debug("pathtrust: matched '%s' with %s", pathname, node->pathname);
			return true;
		}
	}

	return false;
}

/*
 * Pathtrust initialization
 */
static int __init pathtrust_init(void){

	char *item;
	char default_trusted_pathnames[] = CONFIG_SECURITY_PATHTRUST_PATHNAMES;
	char *default_pathnames = default_trusted_pathnames;

	security_add_hooks(pathtrust_hooks, ARRAY_SIZE(pathtrust_hooks));

	pr_info("starting in '%s' (%d) mode\n", pathtrust_enforce ? "enforced" : "non-enforced", pathtrust_enforce);

	/* Empty string check */
	if (default_trusted_pathnames[0]) {

		/* Load configured whitelisted pathnames */
		while ((item = strsep(&default_pathnames, ";")) != NULL) {

			if (!*item)
				continue;

			/* If we have an error, stop the loop*/
			if (pathtrust_add_path(item))
				break;
		}
	}

	return 0;
}

module_init(pathtrust_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Pathtrust Security Module");
MODULE_AUTHOR("BlackBerry Limited");
