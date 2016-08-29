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

#include "sensor_extended.h"
#include <linux/sensors.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/of.h>
#include <linux/gpio.h>

static const char * x_name = "orientation-x";
static const char * y_name = "orientation-y";
static const char * z_name = "orientation-z";
static const char * userspace_name = "userspace";

static const char * get_char_attr(struct device * dev, const char * attr)
{
	struct device_node *np = dev->of_node;
	int rc;
	const char * attr_out;
	rc = of_property_read_string(np, attr, &attr_out);
	if (rc < 0) {
		dev_err(dev, "failed to read property %s\n", attr);
		return NULL;
	}
	return attr_out;
}

static int char_attr_exists(struct device * dev, const char * attr)
{
	struct device_node *np = dev->of_node;
	int err;
	const char * attr_out;
	if (np == NULL)
		return 0;
	err = of_property_read_string(np, attr, &attr_out);
	if (err < 0) {
		return 0;
	}
	return 1;
}

static ssize_t show_char_prop(struct device * dev, struct device_attribute * attr, char * buf, const char * prop)
{
	struct device_node *np = dev->of_node;
	int err;
	const char * attr_name;
	if (np == NULL)
		return 0;
	if (NULL == (attr_name = get_char_attr(dev, prop)))
		return -1;
	err = scnprintf(buf, PAGE_SIZE, "%s\n", attr_name);
	return err;
}

static ssize_t show_u32_prop(struct device * dev, struct device_attribute * attr, char * buf, const char * prop)
{
	struct device_node *np = dev->of_node;
	int err;
	u32 val;
	if (np == NULL)
		return 0;
	if (0 > (err = of_property_read_u32(np, prop, &val)))
		return err;
	return scnprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t show_x_orientation(struct device * dev, struct device_attribute * attr, char * buf)
{
	return show_char_prop(dev, attr, buf, x_name);
}

static ssize_t show_y_orientation(struct device * dev, struct device_attribute * attr, char * buf)
{
	return show_char_prop(dev, attr, buf, y_name);
}

static ssize_t show_z_orientation(struct device * dev, struct device_attribute * attr, char * buf)
{
	return show_char_prop(dev, attr, buf, z_name);
}

static ssize_t show_userspace(struct device * dev, struct device_attribute * attr, char * buf)
{
	return show_u32_prop(dev, attr, buf, userspace_name);
}

static DEVICE_ATTR(x_orientation, 0444, show_x_orientation, NULL);
static DEVICE_ATTR(y_orientation, 0444, show_y_orientation, NULL);
static DEVICE_ATTR(z_orientation, 0444, show_z_orientation, NULL);
static DEVICE_ATTR(userspace, 0444, show_userspace, NULL);

int sensor_create_extension(struct device * dev)
{
	if (char_attr_exists(dev, x_name))
		device_create_file(dev, &dev_attr_x_orientation);
	if (char_attr_exists(dev, y_name))
		device_create_file(dev, &dev_attr_y_orientation);
	if (char_attr_exists(dev, z_name))
		device_create_file(dev, &dev_attr_z_orientation);
	if (char_attr_exists(dev, userspace_name))
		device_create_file(dev, &dev_attr_userspace);
	return 0;
}

static int expose_pin(struct device * dev, int gpio, const char * name)
{
	int err;
	if (0 == gpio_is_valid(gpio)) {
		pr_err("The requested GPIO [%d] is not available\n", gpio);
		err = -EINVAL;
		goto fail0;
	}

	if (0 > (err = gpio_request(gpio, name))) {
		pr_err("Unable to request GPIO [%d]: %d\n", gpio, err);
		goto fail0;
	}

	if (0 != (err = gpio_export(gpio, false))) {
		pr_err("Failed to export GPIO [%d]: %d", gpio, err);
		goto fail1;
	}
	return 0;
fail1:
	gpio_free(gpio);
fail0:
	return err;
}

int expose_interrupt(struct device * dev, int gpio, const char * name)
{
	int err;
	if (0 != (err = expose_pin(dev, gpio, name))) {
		dev_err(dev, "failed to expose gpio %d\n", gpio);
		goto fail0;
	}
	/*
	 * Configure pin as an input pin, and expose to userspace.
	 * Prevent userspace from reconfiguring as an output pin.
	 */
	if (0 != (err = gpio_direction_input(gpio))) {
		pr_err("Failed to set direction for GPIO [%d]", gpio);
		goto fail1;
	}
	if (0 != (err = gpio_export_link(dev, "interrupt", gpio))) {
		pr_err("Failed to create symlink for GPIO [%d]", gpio);
		goto fail1;
	}
	return 0;
fail1:
	gpio_unexport(gpio);
	gpio_free(gpio);
fail0:
	return err;
}

int expose_reset(struct device * dev, int gpio, const char * name, const int
		default_val)
{
	int err;
	if (0 != (err = expose_pin(dev, gpio, name))) {
		dev_err(dev, "failed to expose gpio %d\n", gpio);
		goto fail0;
	}
	if (0 != (err = gpio_direction_output(gpio, default_val))) {
		pr_err("Failed to set direction for GPIO [%d]", gpio);
		goto fail1;
	}
	if (0 != (err = gpio_export_link(dev, "reset", gpio))) {
		pr_err("Failed to create symlink for GPIO [%d]", gpio);
		goto fail1;
	}
	return 0;

fail1:
	gpio_unexport(gpio);
	gpio_free(gpio);
fail0:
	return err;
}

int expose_output(struct device * dev, int gpio, const char * name, const
				int default_val, const char * entry_name) {
	int err;
	if (0 != (err = expose_pin(dev, gpio, name))) {
		dev_err(dev, "failed to expose gpio %d\n", gpio);
		goto fail0;
	}
	if (0 != (err = gpio_direction_output(gpio, default_val))) {
		pr_err("Failed to set direction for GPIO [%d]", gpio);
		goto fail1;
	}
	if (0 != (err = gpio_export_link(dev, entry_name, gpio))) {
		pr_err("Failed to create symlink for GPIO [%d]", gpio);
		goto fail1;
	}
	return 0;

fail1:
	gpio_unexport(gpio);
	gpio_free(gpio);
fail0:
	return err;
}

int get_u32_property(struct device * dev, u32 * val, const char * name)
{
	struct device_node *np = dev->of_node;
	int rc;
	if (np == NULL)
		return -1;
	rc = of_property_read_u32(np, name, val);
	if (rc < 0)
		return rc;
	return 0;
}

