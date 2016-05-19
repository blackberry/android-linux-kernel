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

#ifndef __BBRY_FLASH_GPIO_H__
#define __BBRY_FLASH_GPIO_H__

#include "bbry_flash.h"

extern int32_t bbry_flash_gpio_get_gpio_dt_data(struct device_node *of_node,
					struct bbry_flash_ctrl_t *fctrl);
extern void bbry_flash_gpio_init(struct bbry_flash_ctrl_t *fctrl);
extern void bbry_flash_gpio_release(struct bbry_flash_ctrl_t *fctrl);
extern void bbry_flash_gpio_trigger_low(struct bbry_flash_ctrl_t *fctrl);
extern void bbry_flash_gpio_trigger_high(struct bbry_flash_ctrl_t *fctrl);

#endif /* __BBRY_FLASH_GPIO_H__ */
