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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>

#ifdef CONFIG_PSTORE_CONSOLE
#include <linux/pstore_ram.h>
static struct ramoops_platform_data ramoops_data = {
	.dump_oops = 1,
	.ecc_info.ecc_size = 1,
};

static struct platform_device ramoops_dev = {
	.name = "ramoops",
	.dev = {
		.platform_data = &ramoops_data,
	},
};
#endif

struct reset_region_data {
	int count;
	struct kobject *kobj;
	struct bin_attribute bin[0];
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
				struct device_node *pnode, int index)
{
	struct reset_region_data *pdata = platform_get_drvdata(pdev);
	const char *name;
	u64 size;
	const u32 *addrp;
	u64 addr;
	int rc;
	void __iomem *private;
	struct bin_attribute *bin;

	if (of_property_read_string(pnode, "label", &name)) {
		dev_err(&pdev->dev, "unable to read label\n");
		return -EINVAL;
	}

	addrp = of_get_address(pnode, 0, &size, NULL);
	if (!addrp) {
		dev_err(&pdev->dev, "unable to get address for %s\n", name);
		return -EINVAL;
	}

	addr = of_read_ulong(addrp, 2);
	if (!addr || !size) {
		dev_info(&pdev->dev, "size and/or address is 0 for %s\n", name);
		return -EINVAL;
	}

	private = devm_ioremap_nocache(&pdev->dev, addr, size);
	if (!private) {
		dev_err(&pdev->dev, "failed to remap %p of size %llu\n",
					(void *) addr, size);
		return -EINVAL;
	}

	bin = &pdata->bin[index];

	sysfs_bin_attr_init(bin);
	bin->private = private;
	bin->attr.name = name;
	bin->attr.mode = S_IRUSR;
	bin->size = size;
	bin->read = read_bin_file;
	if (of_property_read_bool(pnode, "oem,writeable")) {
		bin->write = write_bin_file;
		bin->attr.mode |= S_IWUSR;
	}

	rc = sysfs_create_bin_file(pdata->kobj, bin);
	if (!rc) {
#ifdef CONFIG_PSTORE_CONSOLE
		if (!strcmp(name, "reset_log")) {
			ramoops_data.mem_address = addr;
			ramoops_data.mem_size = size;
			ramoops_data.console_size = size/2;
			ramoops_data.record_size = size/2;
			dev_info(&pdev->dev, "mem_addr=0x%llx, console_size=%d\n", addr, (int)ramoops_data.console_size);
			return platform_device_register(&ramoops_dev);
		}
#endif
		return 0;
	}

	dev_err(&pdev->dev, "sysfs_create_bin_file for %s failed\n", name);
	devm_iounmap(&pdev->dev, private);
	bin->private = NULL;
	return -EINVAL;
}

static int reset_region_probe(struct platform_device *pdev)
{
	struct reset_region_data *pdata;
	struct device_node *pnode;
	int i, found, count, rc;

	count = of_count_phandle_with_args(pdev->dev.of_node, "regions", "reg");
	if (count <= 0) {
		dev_err(&pdev->dev, "unable to get phandle\n");
		return -EINVAL;
	}

	pdata = devm_kzalloc(&pdev->dev,
		sizeof(*pdata) + (count * sizeof(pdata->bin[0])), GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev, "alloc failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, pdata);

	pdata->kobj = kobject_create_and_add("reset_region", NULL);
	if (!pdata->kobj) {
		dev_err(&pdev->dev, "unable to create kobject\n");
		rc = -ENOMEM;
		goto kobj_failed;
	}

	for (i = 0, found = 0; i < count; i++) {
		pnode = of_parse_phandle(pdev->dev.of_node, "regions", i);
		if (pnode) {
			if (!create_bin_file_entry(pdev, pnode, i))
				found++;
			of_node_put(pnode);
		}
	}

	if (found != 0)
		return 0;

	rc = -EINVAL;
	kobject_put(pdata->kobj);

kobj_failed:
	devm_kfree(&pdev->dev, pdata);
	return rc;
}

static int reset_region_remove(struct platform_device *pdev)
{
	struct reset_region_data *pdata = platform_get_drvdata(pdev);
	int i;

	if (pdata) {
		for (i = 0; i < pdata->count; i++)
			sysfs_remove_bin_file(pdata->kobj, &pdata->bin[i]);
		kobject_put(pdata->kobj);
	}
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct of_device_id reset_region_match_table[] = {
	{ .compatible = "oem,preserved_reset_region",},
	{ },
};

static struct platform_driver reset_region_driver = {
	.probe = reset_region_probe,
	.remove = reset_region_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "reset_region",
		.of_match_table = reset_region_match_table,
	},
};

module_platform_driver(reset_region_driver);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("BlackBerry Limited");
