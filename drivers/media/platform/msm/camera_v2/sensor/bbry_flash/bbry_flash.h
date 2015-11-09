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

#ifndef __BBRY_FLASH_H__
#define __BBRY_FLASH_H__

#include <linux/platform_device.h>
#include <media/msm_cam_sensor.h>
#include <linux/leds.h>
#include <soc/qcom/camera2.h>
#include "msm_sd.h"

#define MAX_DRIVER_NAME_LENGTH	32

struct bbry_flash_device_t;
struct bbry_flash_ctrl_t;

enum bbry_flash_device_type_t {
	PMI8994_FLASH,
	MAX77387_FLASH,
	INVALID_FLASH
};

struct bbry_flash_cmd_t {
	enum msm_flash_cfg_type_t cfg_type;
	uint32_t requested_current;
	uint32_t limited_current;
	uint32_t applied_current;
	int32_t flash_duration;
};

struct bbry_flash_driver_t {
	enum bbry_flash_device_type_t type;
	char name[MAX_DRIVER_NAME_LENGTH];
	int32_t (*probe)(struct bbry_flash_ctrl_t *, struct device_node *,
			struct device_node *, struct bbry_flash_device_t *);
};

struct bbry_flash_func_t {
	int32_t (*flash_init)(struct bbry_flash_device_t *,
				struct bbry_flash_cmd_t *);
	int32_t (*flash_release)(struct bbry_flash_device_t *,
				struct bbry_flash_cmd_t *);
	int32_t (*flash_off)(struct bbry_flash_device_t *,
				struct bbry_flash_cmd_t *);
	int32_t (*torch_on)(struct bbry_flash_device_t *,
				struct bbry_flash_cmd_t *);
	int32_t (*flash_low)(struct bbry_flash_device_t *,
				struct bbry_flash_cmd_t *);
	int32_t (*flash_high)(struct bbry_flash_device_t *,
				struct bbry_flash_cmd_t *);
#ifdef CONFIG_BBRY_MFG
	int32_t (*flash_status)(struct bbry_flash_device_t *,
				struct bbry_flash_cmd_t *);
#endif /*ifdef CONFIG_BBRY_MFG */
};

struct bbry_flash_mode_t {
	const char *trigger_name;
	struct led_trigger *trigger;
	uint32_t max_current;
	uint32_t default_current;
};

struct bbry_flash_mitigation_t {
	uint32_t *levels;
	uint32_t num_levels;
	uint32_t limit;
};

struct bbry_flux_curve_t {
	uint32_t *mA_values;
	uint32_t *rel_flux_values;
	uint32_t num_points;
};

struct bbry_flash_device_t {
	enum bbry_flash_device_type_t type;
	struct bbry_flash_func_t func_tbl;

	struct bbry_flash_mode_t flash;
	struct bbry_flash_mode_t torch;

	struct bbry_flash_cmd_t state;
};

struct bbry_flash_ctrl_t {
	struct platform_device *pdev;
	struct msm_sd_subdev msm_sd;
	struct msm_camera_power_ctrl_t power_info;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_default;
	struct pinctrl_state *gpio_state_suspend;

	uint32_t high_max_current;	/* max current in flash_high mode */
	uint32_t low_max_current;	/* max current in flash_low mode */
	uint32_t torch_max_current;	/* max current in torch_on mode */
	bool use_fixed_ratio;
	uint32_t preflash_divisor;
	struct bbry_flash_mitigation_t flash_mitigation;
	struct bbry_flash_mitigation_t torch_mitigation;
	struct bbry_flux_curve_t flux_curve;

	struct msm_flash_hw_data_t hw_data;
	struct bbry_flash_device_t flash[MAX_LED_TRIGGERS];
	struct regulator *vdd;
	bool vdd_enabled;
	enum msm_flash_cfg_type_t state;
};

#endif /* __BBRY_FLASH_H__ */
