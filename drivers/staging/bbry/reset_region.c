/*
 *  Copyright (c) 2014 BlackBerry
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>

static LIST_HEAD(reset_region_head);
static struct kobject *reset_region_kobj;

struct reset_region_entry {
	struct list_head list;
	struct bin_attribute bin;
};

static ssize_t read_bin_file(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count)
{
	return memory_read_from_buffer(buf, count, &off, bin_attr->private,
					bin_attr->size);
}

static ssize_t write_bin_file(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count)
{
	if (off < 0)
		return -EINVAL;
	if (off >= bin_attr->size)
		return 0;
	if (count > (bin_attr->size - off))
		count = (bin_attr->size - off);
	memcpy(bin_attr->private + off, buf, count);
	return count;
}

static int create_bin_file_entry(struct platform_device *pdev,
				struct device_node *node, phys_addr_t start,
				phys_addr_t size)
{
	const char *name;
	u32 entry_size;
	struct reset_region_entry *entry;
	int rc;

	if (of_property_read_string(node, "oem,preserved_region_name",
					&name))
		return -EINVAL;
	if (of_property_read_u32(node, "oem,preserved_region_size",
					&entry_size))
		return -EINVAL;

	if (entry_size > size)
		return -EINVAL;

	entry = devm_kzalloc(&pdev->dev, sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->bin.private = devm_ioremap_nocache(&pdev->dev,
							start, entry_size);
	if (!entry->bin.private)
		return -ENOMEM;

	sysfs_bin_attr_init(&entry->bin);
	entry->bin.attr.name = name;
	entry->bin.attr.mode = S_IRUSR | S_IWUSR;
	entry->bin.size = entry_size;
	entry->bin.read = read_bin_file;
	entry->bin.write = write_bin_file;

	rc = sysfs_create_bin_file(reset_region_kobj, &entry->bin);
	if (!rc) {
		list_add(&entry->list, &reset_region_head);
		rc = entry_size;
	}
	return rc;
}

static int __devinit reset_region_probe(struct platform_device *pdev)
{
	struct reset_region_entry *entry;
	struct device_node *child;
	u32 data[2];
	phys_addr_t addr, size;
	int rc;

	if (of_property_read_u32_array(pdev->dev.of_node,
					"qcom,memblock-remove",
					&data[0], sizeof(data)/sizeof(data[0])))
		return -EINVAL;

	addr = data[0];
	size = data[1];
	if (size == 0)
		return -EINVAL;

	reset_region_kobj = kobject_create_and_add("reset_region", NULL);
	if (!reset_region_kobj) {
		dev_err(&pdev->dev, "unable to create kobject\n");
		return -ENOMEM;
	}

	for_each_child_of_node(pdev->dev.of_node, child) {
		rc = create_bin_file_entry(pdev, child, addr, size);
		of_node_put(child);
		if (rc < 0)
			goto cleanup;
		addr += rc;
		size -= rc;
	}
	return 0;

cleanup:
	list_for_each_entry(entry, &reset_region_head, list)
		sysfs_remove_bin_file(reset_region_kobj, &entry->bin);
	kobject_put(reset_region_kobj);
	return -EINVAL;
}

static int __devexit reset_region_remove(struct platform_device *pdev)
{
	struct reset_region_entry *entry;

	list_for_each_entry(entry, &reset_region_head, list)
		sysfs_remove_bin_file(reset_region_kobj, &entry->bin);
	kobject_put(reset_region_kobj);

	return 0;
}

static struct of_device_id reset_region_match_table[] = {
	{ .compatible = "oem,preserved_reset_region",},
	{ },
};

static struct platform_driver reset_region_driver = {
	.probe = reset_region_probe,
	.remove = __devexit_p(reset_region_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "reset_region",
		.of_match_table = reset_region_match_table,
	},
};
EXPORT_COMPAT("oem,preserved_reset_region");

module_platform_driver(reset_region_driver);
MODULE_LICENSE("GPL v2");
