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
#ifndef __SENSOR_EXTENDED__
#define __SENSOR_EXTENDED__

#include <linux/device.h>
int sensor_create_extension(struct device * dev);
int expose_interrupt(struct device * dev, int gpio, const char * name);
int expose_reset(struct device * dev, int gpio, const char * name, const int
		default_val);
int set_input(struct device * dev, int val);
int expose_output(struct device * dev, int gpio, const char * name, const int
		default_val, const char * entry_name);
int get_u32_property(struct device * dev, u32 * val, const char * name);
#endif
