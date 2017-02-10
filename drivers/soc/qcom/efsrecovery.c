/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/elf.h>
#include <linux/wait.h>
#include <soc/qcom/efsrecovery.h>


#define EFSREC_WAIT_MSECS	120000

static int enable_efsrec;
module_param(enable_efsrec, int, S_IRUGO | S_IWUSR);

struct efsrecovery_device {
	char name[256];

	unsigned int data_ready;
	unsigned int consumer_present;
	int efsrecovery_status;

	struct completion efsrecovery_complete;
	struct miscdevice device;

	wait_queue_head_t rec_wait_q;
};
static void *efsdo_ramdump_dev;
static void *efscnt_ramdump_dev;
static int erase_efs_now;

static int efsrecovery_open(struct inode *inode, struct file *filep)
{
	struct efsrecovery_device *rd_dev = container_of(filep->private_data,
				struct efsrecovery_device, device);
	rd_dev->consumer_present = 1;
	rd_dev->efsrecovery_status = 0;
	return 0;
}

static int efsrecovery_release(struct inode *inode, struct file *filep)
{
	struct efsrecovery_device *rd_dev = container_of(filep->private_data,
				struct efsrecovery_device, device);
	rd_dev->consumer_present = 0;
	rd_dev->data_ready = 0;
	complete(&rd_dev->efsrecovery_complete);
	return 0;
}

static ssize_t efsrecovery_read(struct file *filep, char __user *buf, size_t count,
			loff_t *pos)
{
	struct efsrecovery_device *rd_dev = container_of(filep->private_data,
				struct efsrecovery_device, device);

	int ret = 0;
	if ((filep->f_flags & O_NONBLOCK) && !rd_dev->data_ready)
		return -EAGAIN;

	ret = wait_event_interruptible(rd_dev->rec_wait_q, rd_dev->data_ready);
	pr_debug("efsrecovery_read  wait_event_interruptible ret=%d", ret);
	if (ret)
		return ret;

	rd_dev->efsrecovery_status = 0;
	rd_dev->data_ready = 0;
	*pos = 0;
	
	complete(&rd_dev->efsrecovery_complete);
	return ret;
}

static unsigned int efsrecovery_poll(struct file *filep,
					struct poll_table_struct *wait)
{
	struct efsrecovery_device *rd_dev = container_of(filep->private_data,
				struct efsrecovery_device, device);
	unsigned int mask = 0;

	if (rd_dev->data_ready)
		mask |= (POLLIN | POLLRDNORM);

	poll_wait(filep, &rd_dev->rec_wait_q, wait);
	return mask;
}

static const struct file_operations efsrecovery_file_ops = {
	.open = efsrecovery_open,
	.release = efsrecovery_release,
	.read = efsrecovery_read,
	.poll = efsrecovery_poll
};

static void *create_efsrecovery_device(const char *dev_name, struct device *parent)
{
	int ret;
	struct efsrecovery_device *rd_dev;

	if (!dev_name) {
		pr_err("%s: Invalid device name.\n", __func__);
		return NULL;
	}

	rd_dev = kzalloc(sizeof(struct efsrecovery_device), GFP_KERNEL);

	if (!rd_dev) {
		pr_err("%s: Couldn't alloc space for efsrecovery device!",
			__func__);
		return NULL;
	}

	snprintf(rd_dev->name, ARRAY_SIZE(rd_dev->name), "efsrec_%s",
		 dev_name);

	init_completion(&rd_dev->efsrecovery_complete);

	rd_dev->device.minor = MISC_DYNAMIC_MINOR;
	rd_dev->device.name = rd_dev->name;
	rd_dev->device.fops = &efsrecovery_file_ops;
	rd_dev->device.parent = parent;

	init_waitqueue_head(&rd_dev->rec_wait_q);

	ret = misc_register(&rd_dev->device);

	if (ret) {
		pr_err("%s: misc_register failed for %s (%d)", __func__,
				dev_name, ret);
		kfree(rd_dev);
		return NULL;
	}

	return (void *)rd_dev;
}

void destroy_efsrecovery_device(void *dev)
{
	struct efsrecovery_device *rd_dev = dev;

	if (IS_ERR_OR_NULL(rd_dev))
		return;

	misc_deregister(&rd_dev->device);
	kfree(rd_dev);
}
EXPORT_SYMBOL(destroy_efsrecovery_device);

static int _do_efsrecovery(void *handle)
{
	int ret;
	struct efsrecovery_device *rd_dev = (struct efsrecovery_device *)handle;

	if (!rd_dev->consumer_present) {
		pr_err("efsrecovery(%s): No consumers. Aborting..\n", rd_dev->name);
		return -EPIPE;
	}

	rd_dev->data_ready = 1;
	rd_dev->efsrecovery_status = -1;

	INIT_COMPLETION(rd_dev->efsrecovery_complete);

	/* Tell userspace that the data is ready */
	wake_up(&rd_dev->rec_wait_q);

	/* Wait (with a timeout) to let the efsrecovery complete */
	ret = wait_for_completion_timeout(&rd_dev->efsrecovery_complete,
			msecs_to_jiffies(EFSREC_WAIT_MSECS));

	if (!ret) {
		pr_err("efsrecovery(%s): Timed out waiting for userspace.\n",
			rd_dev->name);
		ret = -EPIPE;
	} else
		ret = (rd_dev->efsrecovery_status == 0) ? 0 : -EPIPE;

	rd_dev->data_ready = 0;
	return ret;
}

int do_efsrecovery(void)
{
	int ret;

	if(enable_efsrec == 1)
	{
		if(erase_efs_now){
			if(efsdo_ramdump_dev==NULL)
				return 0;
			ret = _do_efsrecovery(efsdo_ramdump_dev);
			if (ret < 0)
				pr_err("unable to dump efsdo %d\n", ret);
		}else{
			if(efscnt_ramdump_dev==NULL)
				return 0;
			ret = _do_efsrecovery(efscnt_ramdump_dev);
			if (ret < 0)
				pr_err("unable to dump efscnt %d\n", ret);
		}
	}
	return ret;
}
EXPORT_SYMBOL(do_efsrecovery);

static __init int efsrecovery_init(void)
{
	erase_efs_now=0;

	efsdo_ramdump_dev = create_efsrecovery_device("do", NULL);

	if (IS_ERR_OR_NULL(efsdo_ramdump_dev)) {
		pr_err("Unable to create efsdo ramdump device.\n");
		efsdo_ramdump_dev = NULL;
	}

	efscnt_ramdump_dev = create_efsrecovery_device("cnt", NULL);

	if (IS_ERR_OR_NULL(efscnt_ramdump_dev)) {
		pr_err("Unable to create efscnt ramdump device.\n");
		efscnt_ramdump_dev = NULL;
	}

	return 0;
}
late_initcall(efsrecovery_init);

void efsrecovery_check(char * reason)
{
	if(strstr(reason, "fs_") != NULL){
		erase_efs_now=1;
		pr_debug("erase efs partition now.\n");

	}else if(strstr(reason, "sfs_err") != NULL){
		erase_efs_now=1;
		pr_debug("erase sfs partition now.\n");

	}else{
		erase_efs_now=0;
		pr_debug("erase efs partition for count.\n");
	}
}
EXPORT_SYMBOL(efsrecovery_check);

