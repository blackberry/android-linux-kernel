/*
 *  Copyright (c) 2015 BlackBerry
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <soc/qcom/smem.h>

static struct kobject *export_bsi_kobj;

static struct bin_attribute bsi_entry_bin;

static ssize_t read_bin_file(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count)
{

	return memory_read_from_buffer(buf, count, &off, bin_attr->private,
				       bin_attr->size);
}

static int create_bin_file_entry(struct platform_device *pdev,
				 phys_addr_t start,
				 phys_addr_t size)
{
	const char *name = "bsi_data";
	int rc;

	/* direct memory approach */
	bsi_entry_bin.private = (void *) start;

	sysfs_bin_attr_init(bsi_entry_bin);
	bsi_entry_bin.attr.name = kstrdup(name, GFP_KERNEL);

	bsi_entry_bin.attr.mode = S_IRUSR;
	bsi_entry_bin.size = size;  /* entry_size; */
	bsi_entry_bin.read = read_bin_file;

	rc = sysfs_create_bin_file(export_bsi_kobj, &bsi_entry_bin);
	if (!rc)
		pr_info("Info: sysfs_create_bin_file returned %d, success!", rc);
	else
		pr_err("Error: sysfs_create_bin_file returned %d", rc);

	return rc;
}

static int export_bsi_probe(struct platform_device *pdev)
{
	void *bsi_data;
	unsigned bsi_size;
	int rc;

	bsi_data = smem_get_entry(SMEM_ID_VENDOR1, &bsi_size, 0, SMEM_ANY_HOST_FLAG);
	if (bsi_data == NULL) {
		pr_err("%s: Failed to get smem VENDOR1\n", __func__);
		return -EINVAL;
	}
	else
		pr_info("Info:  bsi_data is 0x%p, bsi_size is %u", bsi_data, bsi_size);

	if (bsi_size == 0) {
		pr_err("Error:  bsi_size == 0 ?!?!\n");
		return -EINVAL;
	}

	export_bsi_kobj = kobject_create_and_add("bsi", NULL);
	if (!export_bsi_kobj) {
		dev_err(&pdev->dev, "unable to create kobject\n");
		return -ENOMEM;
	}

	rc = create_bin_file_entry(pdev, (phys_addr_t)bsi_data,
				   bsi_size);
	if (rc < 0) {
		kobject_put(export_bsi_kobj);
		return -EINVAL;
	}

	return 0;
}
static int export_bsi_remove(struct platform_device *pdev)
{
	sysfs_remove_bin_file(export_bsi_kobj, &bsi_entry_bin);
	kobject_put(export_bsi_kobj);

	return 0;
}

static struct of_device_id export_bsi_match_table[] = {
	{ .compatible = "oem,export_bsi_region",},
	{ },
};


static struct platform_driver export_bsi_driver = {
	.probe = export_bsi_probe,
	.remove = export_bsi_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "export_bsi",
		.of_match_table = export_bsi_match_table,
	},
};

module_platform_driver(export_bsi_driver);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Blackberry Limited");
MODULE_DESCRIPTION("Provide limited access to BSI binary originating from bootloader");
