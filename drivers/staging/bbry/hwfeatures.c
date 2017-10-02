/*
 *  drivers/staging/bbry/hwfeatures.c
 *
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
#include <linux/version.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/slab.h>

struct hwfeatures_entry {
	const char *propvalue;
	struct device_attribute dattr;
	char name[0];
};

static ssize_t hwfeatures_attr_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct hwfeatures_entry *descr =
			container_of(attr, struct hwfeatures_entry, dattr);
	return scnprintf(buf, PAGE_SIZE, "%s", descr->propvalue);
}

static struct hwfeatures_entry * __init add_entry(const char *name,
						struct device_node *np,
						struct property *prop)
{
	const char *propvalue;
	struct hwfeatures_entry *entry;
	int len = 0;

	/* There is a prop named 'name' on each node, just skip it */
	if (!strcmp(prop->name, "name"))
		return NULL;

	/* Also skip the compatible prop - we don't want it */
	if (!strcmp(prop->name, "compatible"))
		return NULL;

	if (of_property_read_string(np, prop->name, &propvalue))
		return NULL;

	if (name)
		len += strlen(name);
	len += strlen(prop->name) + 1;
	entry = kmalloc(sizeof(*entry) + len, GFP_KERNEL);
	if (!entry)
		return NULL;

	entry->propvalue = propvalue;
	snprintf(entry->name, len, "%s%s", name ? name : "", prop->name);

	sysfs_attr_init(&entry->dattr.attr);
	entry->dattr.attr.name = entry->name;
	entry->dattr.attr.mode = 0444;
	entry->dattr.show = hwfeatures_attr_show;

	return entry;
}

static struct hwfeatures_entry * attr_to_entry(struct attribute *a)
{
	struct device_attribute *devattr;
	devattr = container_of(a, struct device_attribute, attr);
	return container_of(devattr, struct hwfeatures_entry, dattr);
}

static void hwfeatures_cleanup(struct device *dev)
{
	struct attribute_group *group = dev_get_platdata(dev);
	int i;

	if (group->attrs == NULL)
		return;

	for (i = 0; group->attrs[i] != NULL; i++)
		kfree(attr_to_entry(group->attrs[i]));
	kfree(group->attrs);
	group->attrs = NULL;
}

static int __init scan_all_node(struct device_node *node,
				struct attribute_group *group,
				int *count, int *max_count, char *name)
{
	struct attribute **attrs;
	struct hwfeatures_entry *entry;
	struct device_node *np;
	struct property *prop;
	char *new_name;
	int baselen = 0;
	int rc, i;

	if (name)
		baselen = strlen(name);

	for (prop = node->properties; prop != NULL; prop = prop->next) {
		entry = add_entry(name, node, prop);
		if (!entry)
			continue;

		/* Entry must be unique, always keep the first one 
		 * added as its the most specific one
		*/
		for(i=0; i < *count; i++) {
			if (strcmp(entry->name, group->attrs[i]->name))
				continue;
			kfree(entry);
			entry = NULL;
			break;
		}

		if(!entry)
			continue;

		if (*count == *max_count) {
			*max_count += 16;
			/* +1 to keep room for NULL terminating the
			 * attribute group
			 */
			attrs = krealloc(group->attrs,
					(*max_count + 1) * sizeof(struct attribute *), GFP_KERNEL);
			if (!attrs) {
				kfree(entry);
				return -EINVAL;
			}
			group->attrs = attrs;
		}
		group->attrs[*count] =  &entry->dattr.attr;
		*count += 1;
	}

	/* The max prop length is currently set to 32 so this
	 * should be plenty enough
	 */
	new_name = kzalloc(512, GFP_KERNEL);
	rc = 0;
	for_each_child_of_node(node, np) {
		snprintf(new_name, 512, "%s%s.", name ? name : "", np->name);
		rc = scan_all_node(np, group, count, max_count, new_name);
		if (rc)
			break;
	}
	kfree(new_name);
	return rc;
}

static u32 dtsid = UINT_MAX;
static int __init set_board_dtsid(char *arg)
{
	return kstrtou32(arg, 0, &dtsid);
}
__setup("androidboot.binfo.dtsid=", set_board_dtsid);

static int __init hwfeatures_create_group(struct device *dev)
{
	char namebuf[32];
	struct attribute_group *group = dev_get_platdata(dev);
	struct device_node *node;
	int max_count = 32;
	int count = 0;
	int rc = -ENOMEM;

	/* +1 to keep room for NULL terminating the attribute group */
	group->attrs =
		kcalloc(max_count + 1, sizeof(struct attribute *), GFP_KERNEL);
	if (!group->attrs)
		return -ENOMEM;

	/* Add the most specific one first */
	sprintf(namebuf,"hwfeatures,%08x", dtsid);
	node = of_find_compatible_node(NULL, NULL, namebuf);
	if (node)
		(void)scan_all_node(node, group, &count, &max_count, NULL);

	/* Now add the generic one */
	node = of_find_compatible_node(NULL, NULL, "hwfeatures");
	if (node)
		(void)scan_all_node(node, group, &count, &max_count, NULL);

	if (count > 0) {
		group->attrs[count] = NULL;
		rc = sysfs_create_group(&dev->kobj, group);
		if (!rc)
			return 0;
	}
	hwfeatures_cleanup(dev);
	return rc;
}

static struct attribute_group hwfeatures_group = {
	.name = "hwfeatures0",
};
static struct device hwfeatures_dev = {
	.init_name = "hwfeatures",
	.release = hwfeatures_cleanup,
	.platform_data = (void *)&hwfeatures_group,
};

static int __init hwfeatures_init(void)
{
	int err;

	err = device_register(&hwfeatures_dev);
	if (err) {
		pr_err("%s : Error registering\n", hwfeatures_dev.init_name);
		return err;
	}

	err = hwfeatures_create_group(&hwfeatures_dev);
	if (err) {
		device_unregister(&hwfeatures_dev);
		return err;
	}
	return 0;
}
module_init(hwfeatures_init);

static void __exit hwfeatures_exit(void)
{
	struct attribute_group *group = dev_get_platdata(&hwfeatures_dev);
	sysfs_remove_group(&hwfeatures_dev.kobj, group);
	device_unregister(&hwfeatures_dev);
}
module_exit(hwfeatures_exit);
MODULE_LICENSE("GPL v2");
