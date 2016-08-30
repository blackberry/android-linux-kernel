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

#include <linux/device.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/module.h>
#include <linux/mount.h>
#include "pathtrust_private.h"

/* Access to task_security_struct selinux internal */
#include <objsec.h>

#define IOC_MAGIC 'p'
#define IOCTL_TRUST_FILE	_IOR(IOC_MAGIC, 0, int)

/*
 * Pathtrust ioctl call so that external
 * clients can query the state of pathtrusted
 * files like the shell
 */
static long pathtrust_cdev_ioctl(struct file *fp, unsigned cmd, unsigned long value)
{
	struct file *file;
	struct common_audit_data cad;
	struct pathtrust_audit_data pt_data = { .allowed = false, .dev = 0 };

	switch(cmd)
	{
	case IOCTL_TRUST_FILE:
	{
		const struct cred *cred = current_cred();
		const struct task_security_struct *tsec = cred->security;
		bool fsid;

		if (!pathtrust_enforce)
			return 0;

		file = fget(value);
		fsid = is_forbidden_sid(tsec->sid);

		if (file && !(file->f_path.mnt->mnt_flags & MNT_TRUSTED) &&
				(is_root(cred) || has_banned_caps() || fsid)) {

			memset(&cad, 0, sizeof(cad));
			cad.type = LSM_AUDIT_DATA_PATH;
			cad.pathtrust_audit_data = &pt_data;
			pt_data.type = PATHTRUST_AUDIT_TYPE_IOCTL;
			if (is_root(cred))
				pt_data.btype = PATHTRUST_AUDIT_BTYPE_ROOT;
			else if (fsid)
				pt_data.btype = PATHTRUST_AUDIT_BTYPE_SELCTX;
			else
				pt_data.btype = PATHTRUST_AUDIT_BTYPE_CAPS;

			pt_data.extra = "IOCTL_TRUST_FILE";
			cad.u.path = file->f_path;
			pathtrust_lsm_audit(&cad);

			return -EPERM;
		}

		return 0;
		break;
	}

	default:
		return -EINVAL;
		break;
	}

	return -EINVAL;
}

static struct file_operations pathtrust_cdev_fops =
	{
		.owner = THIS_MODULE,
		.unlocked_ioctl = pathtrust_cdev_ioctl,
	};

/*
 * Creates /dev/pathtrust device
 */
static int __init pathtrust_dev_init(void)
{
	int ret;
	int major;
	struct class *cl;
	struct device * dev;

	major = register_chrdev(0, KBUILD_MODNAME, &pathtrust_cdev_fops);
	if (major < 0) {
		pr_err("unable to register chrdev '%s'", KBUILD_MODNAME);
		return -EIO;
	}

	cl = class_create(THIS_MODULE, "pathtrust");
	if (!cl) {
		pr_err("failed to create class");
		ret = -EIO;
		goto fail_class;
	}

	dev = device_create(cl, NULL, MKDEV(major,0),
			NULL, "%s", KBUILD_MODNAME);
	if (!dev) {
		pr_err("failed to create device");
		ret = -EIO;
		goto fail_create;
	}

	return 0;

fail_create:
	class_destroy(cl);

fail_class:
	unregister_chrdev(major, KBUILD_MODNAME);

	return ret;
}

late_initcall(pathtrust_dev_init);
