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

#ifndef __BBRY_FLASH_DRIVERS_H__
#define __BBRY_FLASH_DRIVERS_H__

#include <linux/of.h>
#include "bbry_flash.h"

extern int32_t pmi8994_flash_probe(struct bbry_flash_ctrl_t *fctrl,
				struct device_node *flash_of_node,
				struct device_node *torch_of_node,
				struct bbry_flash_device_t *flash);

extern int32_t max77387_flash_probe(struct bbry_flash_ctrl_t *fctrl,
				struct device_node *flash_of_node,
				struct device_node *torch_of_node,
				struct bbry_flash_device_t *flash);

static struct bbry_flash_driver_t supported_drivers[] = {
	{
		.type = PMI8994_FLASH,
		.name = "PMI8994",
		.probe = &pmi8994_flash_probe
	},
	{
		.type = MAX77387_FLASH,
		.name = "MAX77387",
		.probe = &max77387_flash_probe
	},
};

#endif /* __BBRY_FLASH_DRIVERS_H__ */

