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

#include <linux/of.h>
#include <linux/of_gpio.h>
#include "msm_camera_io_util.h"
#include "msm_camera_dt_util.h"
#include "bbry_flash_gpio.h"

#undef pr_fmt
#define pr_fmt(fmt)	"Flash: " fmt

int32_t bbry_flash_gpio_get_gpio_dt_data(struct device_node *of_node,
				struct bbry_flash_ctrl_t *fctrl)
{
	int32_t rc = 0, i = 0;
	uint16_t *gpio_array = NULL;
	int16_t gpio_array_size = 0;
	struct msm_camera_gpio_conf *gconf = NULL;

	if (NULL != fctrl->power_info.gpio_conf) {
		kfree(fctrl->power_info.gpio_conf);
		fctrl->power_info.gpio_conf = NULL;
	}

	gpio_array_size = of_gpio_count(of_node);
	pr_debug("%s gpio count %d\n", __func__, gpio_array_size);

	if (gpio_array_size > 0) {
		fctrl->power_info.gpio_conf =
			 kzalloc(sizeof(struct msm_camera_gpio_conf),
				 GFP_KERNEL);
		if (!fctrl->power_info.gpio_conf) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			rc = -ENOMEM;
			return rc;
		}
		gconf = fctrl->power_info.gpio_conf;

		gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size,
			GFP_KERNEL);
		if (!gpio_array) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			rc = -ENOMEM;
			goto free_gpio_conf;
		}
		for (i = 0; i < gpio_array_size; i++) {
			gpio_array[i] = of_get_gpio(of_node, i);
			if (((int16_t)gpio_array[i]) < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				rc = -EINVAL;
				goto free_gpio_array;
			}
			pr_debug("%s gpio_array[%d] = %d\n", __func__, i,
				gpio_array[i]);
		}

		rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto free_gpio_array;
		}

		rc = msm_camera_get_dt_gpio_set_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto free_cam_gpio_req_tbl;
		}

		rc = msm_camera_init_gpio_pin_tbl(of_node, gconf,
			gpio_array, gpio_array_size);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto free_cam_gpio_set_tbl;
		}

		kfree(gpio_array);
	}

	return 0;

free_cam_gpio_set_tbl:
	kfree(gconf->cam_gpio_set_tbl);
free_cam_gpio_req_tbl:
	kfree(gconf->cam_gpio_req_tbl);
free_gpio_array:
	kfree(gpio_array);
free_gpio_conf:
	kfree(fctrl->power_info.gpio_conf);
	return rc;
}

void bbry_flash_gpio_init(struct bbry_flash_ctrl_t *fctrl)
{
	if (fctrl->power_info.gpio_conf) {
		int rc = 0;

		fctrl->pinctrl = devm_pinctrl_get(&fctrl->pdev->dev);
		if (IS_ERR(fctrl->pinctrl)) {
			pr_err("%s: failed to get pinctrl\n", __func__);
			fctrl->pinctrl = NULL;
		}

		if (fctrl->pinctrl) {
			fctrl->gpio_state_default = pinctrl_lookup_state(
							fctrl->pinctrl,
							"cam_flash_default");
			if (IS_ERR(fctrl->gpio_state_default)) {
				pr_err("%s: can not get active pinstate\n",
					__func__);
				fctrl->gpio_state_default = NULL;
			}
		}

		if (fctrl->pinctrl && fctrl->gpio_state_default) {
			rc = pinctrl_select_state(fctrl->pinctrl,
				fctrl->gpio_state_default);
			if (rc)
				pr_err("%s:set state failed!\n", __func__);
		}

		if (fctrl->pinctrl) {
			fctrl->gpio_state_suspend = pinctrl_lookup_state(
					fctrl->pinctrl, "cam_flash_suspend");
			if (IS_ERR(fctrl->gpio_state_suspend)) {
				pr_err("%s: can not get suspend pinstate\n",
					__func__);
				fctrl->gpio_state_suspend = NULL;
			}
		}

		rc = msm_camera_request_gpio_table(
			fctrl->power_info.gpio_conf->cam_gpio_req_tbl,
			fctrl->power_info.gpio_conf->cam_gpio_req_tbl_size, 1);
		if (rc < 0)
			pr_err("%s: request gpio failed\n", __func__);

		if (fctrl->power_info.gpio_conf->gpio_num_info->
				valid[SENSOR_GPIO_FL_NOW] == 1) {
			gpio_set_value_cansleep(
				fctrl->power_info.gpio_conf->gpio_num_info->
					gpio_num[SENSOR_GPIO_FL_NOW],
					GPIO_OUT_LOW);
		} else {
			pr_err("%s: bad gpio requested?\n", __func__);
		}
	}
}

void bbry_flash_gpio_release(struct bbry_flash_ctrl_t *fctrl)
{
	int rc = 0;

	if (fctrl->power_info.gpio_conf) {
		if (fctrl->power_info.gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_NOW] == 1) {
			gpio_set_value_cansleep(
			fctrl->power_info.gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_LOW);
		}

		rc = msm_camera_request_gpio_table(
			fctrl->power_info.gpio_conf->cam_gpio_req_tbl,
			fctrl->power_info.gpio_conf->cam_gpio_req_tbl_size, 0);
		if (rc < 0)
			pr_err("%s: request gpio failed\n", __func__);
	}

	if (fctrl->pinctrl && fctrl->gpio_state_suspend) {
		rc = pinctrl_select_state(fctrl->pinctrl,
			fctrl->gpio_state_suspend);
		if (rc)
			pr_err("%s:%d cannot set pin to suspend state\n",
				__func__, __LINE__);
	}

	if (fctrl->pinctrl)
		devm_pinctrl_put(fctrl->pinctrl);
}

void bbry_flash_gpio_trigger_low(struct bbry_flash_ctrl_t *fctrl)
{
	if (fctrl->power_info.gpio_conf) {
		if (fctrl->power_info.gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_NOW] == 1) {
			gpio_set_value_cansleep(
				fctrl->power_info.gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_NOW],
				GPIO_OUT_LOW);
		}
	}
}

void bbry_flash_gpio_trigger_high(struct bbry_flash_ctrl_t *fctrl)
{
	if (fctrl->power_info.gpio_conf) {
		if (fctrl->power_info.gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_NOW] == 1) {
			gpio_set_value_cansleep(
				fctrl->power_info.gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_NOW],
				GPIO_OUT_HIGH);
		}
	}
}
