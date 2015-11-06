/*
 * Copyright (C) 2014 BlackBerry Limited
 * Copyright(c) 2012, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef SLIMPORT_DEVICE
#define SLIMPORT_DEVICE

struct anx7808_platform_data {
	int gpio_p_dwn;
	int gpio_reset;
	int gpio_int;
	int gpio_cbl_det;
	int gpio_v10_ctrl;
#ifdef CONFIG_BBRY
	int gpio_v33_ctrl;
	int external_ldo_control;
	struct regulator *avdd_10;
	struct regulator *dvdd_10;
	struct platform_device *hdmi_pdev;
	struct switch_dev hdcp_cap_sdev;
	struct switch_dev online_sdev;
#endif
};

#endif /* SLIMPORT_DEVICE */
