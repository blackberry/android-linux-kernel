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
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/random.h>
#include <asm/uaccess.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"

/*************************************************************************/

struct dev_read_buffer {
	char			*buf;
	unsigned		sz;
	unsigned		off;
};

struct dev_path {
	struct file_operations	fops;	/* Operation handlers */
	struct class 		*cl;	/* Device class */
	struct device 		*dev;	/* Device descriptor for /dev/bide */
	int			major;	/* Major version number */
};

struct _dev_globals {
	struct mutex		iolock;	/* Mutex for ioctl callback */
	struct mutex		rlock;	/* Mutex for dev_read callback */
	struct dev_read_buffer	read;	/* Buffer for dev_read() calls */
	struct dev_path		dev;	/* Device path info */
};

static struct _dev_globals ctx = {};

/*************************************************************************/

/*
 * This function checks whether the current process is allowed to access
 * this module via ioctl().
 *
 * @return  0                Access is allowed.
 *          -EPERM           Disallowed.
 */
static int dev_check_access(void)
{
	const struct cred *cred = NULL;
	struct task_struct *task = NULL;
	int uid = 0;

	/* Call is from kernel, allow by default */
	if (!current)
		return 0;

	/* Aquire the UID of the current process */
	rcu_read_lock();
	task = pid_task(find_vpid(current->pid), PIDTYPE_PID);
	cred = __task_cred(task);
	uid = cred->uid;
	rcu_read_unlock();

	/* Limit access to the JBIDE pid only */
	if (uid != BIDE_UID &&
	    uid != ROOT_UID &&
	    uid != SYSTEM_UID) {
		logError("Access from unregistered UID %d is denied.", uid);
		return -EPERM;
	}

	return 0;
}

/*************************************************************************/

/*
 * This function is a wrapper for ioctl command handling functions. When
 * called, the data provided is copied into kernel memory and passed into
 * the provided ioctl handling function. Once the funciton pointer returns
 * a success, the data is copied back into user space.
 *
 * @param   ptr                 A pointer to a user space buffer.
 * @param   sz                  Size of the buffer.
 * @param   fn                  The function to call after the user space
 *                              data has been copied to kernel space.
 *
 * @return  0                   Access is allowed.
 *          -EINVAL             Invalid parameters.
 *          -ENOMEM             Failed to allocated kernel memory.
 *          -EFAULT             Failed on copy to/from userland.
 */
static int dev_wrap_call(void __user *ptr,
			 unsigned sz,
			 int (*fn)(void *))
{
	void *local = NULL;
	int rc = 0;

	if (!ptr || !sz || !fn)
		return -EINVAL;

	local = kzalloc(sz, GFP_KERNEL);
	if (!local) {
		logError("Failed to allocate local buffer.");
		return -ENOMEM;
	}

	/* Copy the command into kernel space */
	rc = copy_from_user(local, ptr, sz);
	if (rc) {
		logError("Failed on copy_from_user(). rc=%d.", rc);

		rc = -EFAULT;
		goto cleanup;
	}

	/* Call the custom function with a copy of the user data */
	rc = fn(local);
	if (rc) {
		logError("Failed on fn(). rc=%d.", -rc);
		goto cleanup;
	}

	/* Copy all data back to the user */
	rc = copy_to_user(ptr, local, sz);
	if (rc) {
		logError("Failed on copy_to_user(). rc=%d.", rc);

		rc = -EFAULT;
		goto cleanup;
	}

cleanup:
	kfree(local);

	return rc;
}

/*************************************************************************/

/*
 * This function is called whenever a process tries to do an ioctl on the BIDE
 * device file. If the ioctl is write or read/write (meaning output is returned
 * to the calling process), the ioctl call returns the output of this function.
 *
 * @param   filep               Handle to the device file.
 * @param   cmd                 The control number being sent.
 * @param   arg                 A pointer to user data being passed in.
 *
 * @return  0                   No Error.
 *          -EINVAL             Invalid parameters.
 *          -ERESTARTSYS        The mutex was interrupted by a signal.
 *          -EPERM              Access to ioctl disallowed.
 */
static long dev_ioctl(struct file *filep,
		      unsigned int cmd,
		      unsigned long arg)
{
	void *data = (void *) arg;
	int rc = -EINVAL;

	/* Lock mutex or if interrupted by a signal, return immediately */
	if (mutex_lock_interruptible(&ctx.iolock))
		return -ERESTARTSYS;

	/* Limit access to the JBIDE pid only */
	rc = dev_check_access();
	if (rc) {
		rc = -EPERM;
		goto cleanup;
	}

	switch (cmd) {
	case BIDE_IOCTL_FIRST_BOOT_CHECK:
		rc = dev_wrap_call(data,
				   sizeof(bide_is_first_boot_check_cmd_t),
				   &ctl_first_boot_check);
		break;

	case BIDE_IOCTL_TAKE_SNAPSHOT:
		if (!ctl_snapshot_complete()) {
			logDebug("Taking system startup snapshot");
			rc = dev_wrap_call(data,
					   sizeof(bide_snapshot_cmd_t),
					   &ctl_snapshot_initialize);
		}
		break;

	case BIDE_IOCTL_SET_PROPERTY:
		rc = dev_wrap_call(data,
				   sizeof(bide_set_property_cmd_t),
				   &ctl_set_property_check);
		break;

	case BIDE_IOCTL_REGISTER_INSTALLD:
                        /* Register this pid tp be privileged and allowed to run System UID */
			auth_add_pid(current->pid, AUTH_PERM_PRIVELEGED_CHILDREN);
			rc = auth_add_pid(current->pid, AUTH_PERM_INSTALLD);
		break;

	case BIDE_IOCTL_GET_SYSTEM_SERVER_PID:
                rc = dev_wrap_call(data,
				   sizeof(int),
				   &ctl_get_system_server_pid);
		break;

	case BIDE_IOCTL_WHITELIST_SYSTEM_SERVER:
		rc = ctl_add_system_server_pid();
		break;

	case BIDE_IOCTL_FOR_CAPS:
		rc = dev_wrap_call(data,
			sizeof(bide_prog_name_cmd_t),
			&ctl_add_process);

		break;
	case BIDE_IOCTL_WHITELIST_ZYGOTEMGR:
		rc = auth_add_pid(current->pid, AUTH_PERM_ZYGOTEMGR);
		break;

	case BIDE_IOCTL_WHITELIST_PRIVILEGED_PROCESS:
		rc = auth_add_pid(current->pid, AUTH_PERM_PRIVILEGED_GID);
		break;

	case BIDE_IOCTL_WHITELIST_PRIVILEGED_CHILD:
		rc = auth_add_pid(current->pid, AUTH_PERM_PRIVELEGED_CHILDREN);
		break;

	case BIDE_IOCTL_WHITELIST_SYSTEM_PROCESS:
		rc = auth_add_pid(current->pid, AUTH_PERM_SYSTEM_UID);
		break;

	case BIDE_IOCTL_WHITELIST_PRIVILEGED_LINEAGE:
		rc = auth_add_pid(current->pid, AUTH_PERM_PRIVILEGED_LINEAGE);
		break;

	case BIDE_IOCTL_WHITELIST_PRIVILEGED_ZYGOTE:
		rc = auth_add_pid(current->pid, AUTH_PERM_PRIVILEGED_ZYGOTE);
		break;

	case BIDE_IOCTL_CREATE_BIDE_REPORT:
		if (ctl_snapshot_complete()) {
			logDebug("Creating bide report.");
			rc = dev_wrap_call(data,
					   sizeof(bide_create_report_cmd_t),
					   &ctl_generate_report);
		}
		break;

	case BIDE_IOCTL_CREATE_NEW_KEYS:
		if (ctl_snapshot_complete()) {
			logDebug("New keypair is being requested.");
			rc = dev_wrap_call(data,
					   sizeof(bide_request_keys_cmd_t),
					   &ctl_generate_keys);
		}
		break;

	case BIDE_IOCTL_KICK_PROC_SCAN:
		if (ctl_snapshot_complete()) {
			logDebug("Kicking process scan.");
			rc = dev_wrap_call(data,
					   sizeof(bide_kick_process_scan_cmd_t),
					   &ctl_kick_process_scan);
		}
		break;

	case BIDE_IOCTL_REGISTER_BUGREPORT_FOR_CAPS:
		rc = auth_add_pid(current->pid, AUTH_PERM_BUGREPORT);
		break;

	case BIDE_IOCTL_WHITELIST_ZYGOTEMGR_CHILD:
		rc = auth_add_pid(current->pid, AUTH_PERM_ZYGOTEMGR_CHILD);
		rc = auth_add_pid(current->pid, AUTH_PERM_PRIVILEGED_GID);
		break;

	default:
		logError("Unknown Bide command: %d", cmd);
		rc = -EINVAL;
	}

cleanup:
	mutex_unlock(&ctx.iolock);

	if (rc)
		logWarn("Failure on dev_ioctl(). Returning rc=%d", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function is called when a process attempts to read from /dev/bide. This
 * is a blocking call that will unblock when new data is available. The data
 * returned will be an XML based report of system state.
 *
 * @param   filep               Handle to the device file.
 * @param   buf                 The user-land buffer to write to.
 * @param   sz                  Size of buffer.
 * @param   offset              Offset at which to put data in buffer.
 *
 * @return  0+                  The amount of bytes written.
 *          -EFAULT             Buffer size is not large neough for data.
 *          -EPERM              Client does not have permission to read.
 *          -ERESTARTSYS        The mutex was interrupted by a signal.
 */
static ssize_t dev_read(struct file *filep,
			char __user *buf,
			size_t sz,
			loff_t *offset)
{
	unsigned copy_amt = 0;
	unsigned temp = 0;
	int rc = 0;

	/* Check that the client is allowed to read */
	rc = dev_check_access();
	if (rc)
		return -EPERM;

	/* Lock mutex or if interrupted by a signal, return immediately */
	if (mutex_lock_interruptible(&ctx.rlock))
		return -ERESTARTSYS;

	if (!ctx.read.buf) {
		/* Grab the next report (or wait if non exists) */
		rc = report_dequeue(&ctx.read.buf, &ctx.read.sz);
		if (rc) {
			logError("Failed on report_dequeue(). rc=%d.", -rc);
			goto cleanup;
		}
	}

	/* Derive the amount needed to copy */
	temp = ctx.read.sz - ctx.read.off;
	copy_amt = sz < temp ? sz : temp;

	/* Copy the TLV to JBIDE's buffer */
	rc = copy_to_user(buf, ctx.read.buf + ctx.read.off, copy_amt);
	if (rc) {
		logError("Failed on copy_to_user(). rc=%d.", -rc);

		rc = -EFAULT;
		goto cleanup;
	}

	ctx.read.off += copy_amt;

	/* Check if this buffer is done */
	if (ctx.read.off >= ctx.read.sz) {
		kfree(ctx.read.buf);

		ctx.read.buf = NULL;
		ctx.read.off = 0;
		ctx.read.sz = 0;
	}

cleanup:
	mutex_unlock(&ctx.rlock);

	return rc < 0 ? rc : (ssize_t) copy_amt;
}

/*************************************************************************/

/*
 * This callback function is invoked when another process writes data to the
 * BIDE device file. Nothing will be done at this point.
 *
 * @param   filep               Handle to the device file.
 * @param   buf                 The user-land buffer to read from.
 * @param   sz                  Size of buffer.
 * @param   offset              Offset at which to read data from buffer.
 *
 * @return  -ENOSYS             Function not supported.
 */
static ssize_t dev_write(struct file *filep,
			 const char __user *buf,
			 size_t sz,
			 loff_t *offset)
{
	return -ENOSYS;
}

/*************************************************************************/

/*
 * This function is invoked when a user process opens the BIDE device path.
 *
 * @param   inode               The inode of the device file.
 * @param   filep               Handle to the device file.
 *
 * @return  0                   No Error.
 *          -EPERM              Permission denied.
 */
static int dev_open(struct inode *inode,
		    struct file *filep)
{
	/* Check if the JBIDE process is the one opening the bide path */
	int rc = dev_check_access();
	if (rc) {
		logError("Permission denied for PID (%d).", current->pid);
		return -EPERM;
	}

	return 0;
}

/*************************************************************************/

/*
 * This callback function is called when the user process closes the BIDE
 * device file.
 *
 * @param   inode               The inode of the device file.
 * @param   filep               Handle to the device file.
 *
 * @return  0                   No Error.
*/
static int dev_release(struct inode *inode,
		       struct file *filep)
{
	return 0;
}

/*************************************************************************/

/*
 * Entry point into the module during the device phase of the initialization
 * chain. The purpose of this funciton is to set up /dev/bide.
 *
 * @return  0                   No Error.
 *          -EIO                Error during device creaiton.
 */
int __init dev_init(void)
{
	struct class *cl = NULL;
	struct device *dev = NULL;
	int major = 0;

	logInfo("Initializing Device");

	/* Initialize Mutex */
	mutex_init(&ctx.iolock);
	mutex_init(&ctx.rlock);

	/* Set up IO handlers */
	ctx.dev.fops.owner		= THIS_MODULE;
	ctx.dev.fops.unlocked_ioctl	= dev_ioctl;
	ctx.dev.fops.compat_ioctl	= dev_ioctl;
	ctx.dev.fops.read		= dev_read;
	ctx.dev.fops.write		= dev_write;
	ctx.dev.fops.open		= dev_open;
	ctx.dev.fops.release		= dev_release;

	/* Register the character driver */
	major = register_chrdev(0, BIDE_DEVICE_NAME, &ctx.dev.fops);
	if (major < 0) {
		logError("Failed on register_chrdev(). rc=%d", -major);
		return -EIO;
	}

	cl = class_create(THIS_MODULE, BIDE_DEVICE_NAME);
	if (!cl) {
		logError("Failed on class_create().");
		goto fail;
	}

	dev = device_create(cl,
			    NULL,
			    MKDEV(major, 0),
			    "%s",
			    BIDE_DEVICE_NAME);
	if (!dev) {
		logError("Failed on device_create().");
		goto fail;
	}

	ctx.dev.cl = cl;
	ctx.dev.dev = dev;
	ctx.dev.major = major;

	return 0;

fail:
	class_destroy(cl);
	unregister_chrdev(major, BIDE_DEVICE_NAME);

	return -EIO;
}

/*************************************************************************/

/*
 * Device clean up function.
 *
 * @return  0                   No Error.
 */
int __exit dev_exit(void)
{
	/* Remove the bide device */
	device_destroy(ctx.dev.cl, MKDEV(ctx.dev.major, 0));
	class_destroy(ctx.dev.cl);
	unregister_chrdev(ctx.dev.major, BIDE_DEVICE_NAME);

	kfree(ctx.read.buf);

	mutex_destroy(&ctx.iolock);
	mutex_destroy(&ctx.rlock);

	return 0;
}
