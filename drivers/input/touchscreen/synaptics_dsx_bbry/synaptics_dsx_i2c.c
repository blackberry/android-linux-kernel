/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2014 BlackBerry Limited
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <synaptics_dsx.h>
#include "synaptics_dsx_core.h"
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
#include "synaptics_dsx_ddt.h"
#endif
#include "synaptics_dsx_wakeup.h"

#define SYN_I2C_RETRY_TIMES 10
#define LOCKUP_POLLING_COUNT 3
#if defined(CONFIG_BBRY_MFG) || defined(CONFIG_BBRY_DEBUG)
#define BIST_MAX 5000
#endif /*CONFIG_BBRY_MFG || CONFIG_BBRY_DEBUG*/



static int synaptics_rmi4_i2c_set_page(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr)
{
	int retval;
	unsigned char retry;
	unsigned char buf[PAGE_SELECT_LEN];
	unsigned char page;
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);

	page = ((addr >> 8) & MASK_8BIT);
	if (page != rmi4_data->current_page) {
		buf[0] = MASK_8BIT;
		buf[1] = page;
		for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
			retval = i2c_master_send(i2c, buf, PAGE_SELECT_LEN);
			if (retval != PAGE_SELECT_LEN) {
				dev_err(rmi4_data->pdev->dev.parent,
						"%s: I2C retry %d\n",
						__func__, retry + 1);
				if (retry == 0)
					dump_stack();
				msleep(20);
			} else {
				rmi4_data->current_page = page;
				break;
			}
		}
	} else {
		retval = PAGE_SELECT_LEN;
	}

	return retval;
}

static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf;
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);
	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};

	buf = addr & MASK_8BIT;

	if (rmi4_data->sm_err.i2c) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Simulated I2C read error\n",
				__func__);
		rmi4_data->sm_err.i2c--;
		retval = -EIO;
		goto exit;
	}

	mutex_lock(&rmi4_data->rmi4_io_ctrl_mutex);

	retval = synaptics_rmi4_i2c_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN) {
		retval = -EIO;
		goto exit;
	}

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(i2c->adapter, msg, 2) == 2) {
			retval = length;
			break;
		}
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		if (retry == 0)
			dump_stack();
		msleep(20);
	}


	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C read over retry limit\n",
				__func__);
		retval = -EIO;
	}

exit:
	if ((retry > 1) || (retval < 0)) {
		rmi4_data->mtouch_counter.i2c_rw_error++;
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
		synaptics_dsx_ddt_send(rmi4_data, MTOUCH_RW_ERROR, 0);
#endif
	}
	mutex_unlock(&rmi4_data->rmi4_io_ctrl_mutex);

	return retval;
}

static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf[length + 1];
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);
	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	if (rmi4_data->sm_err.i2c) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Simulated I2C read error\n",
				__func__);
		rmi4_data->sm_err.i2c--;
		retval = -EIO;
		goto exit;
	}
	mutex_lock(&rmi4_data->rmi4_io_ctrl_mutex);

	retval = synaptics_rmi4_i2c_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN) {
		retval = -EIO;
		goto exit;
	}

	buf[0] = addr & MASK_8BIT;
	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(i2c->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		if (retry == 0)
			dump_stack();
		msleep(20);
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C write over retry limit\n",
				__func__);
		retval = -EIO;
	}

	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: I2C write. Reg=0x%04x, Len=%d, Data[0]=0x%02x\n",
		__func__, addr, length, data[0]);

exit:
	if ((retry > 1) || (retval < 0)) {
		rmi4_data->mtouch_counter.i2c_rw_error++;
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DDT
		synaptics_dsx_ddt_send(rmi4_data, MTOUCH_RW_ERROR, 0);
#endif
	}

	mutex_unlock(&rmi4_data->rmi4_io_ctrl_mutex);

	return retval;
}

static struct synaptics_dsx_bus_access bus_access = {
	.type = BUS_I2C,
	.read = synaptics_rmi4_i2c_read,
	.write = synaptics_rmi4_i2c_write,
};


static void synaptics_rmi4_i2c_dev_release(struct device *dev)
{
	return;
}

#ifdef CONFIG_OF
static int synaptics_rmi4_parse_dt(struct device *dev,
				struct synaptics_dsx_board_data *bdata)
{
	int retval;
	u32 value;
	struct property *prop;
	struct device_node *np = dev->of_node;
	u32 tmp = 0;

	bdata->irq_gpio = of_get_named_gpio_flags(np,
			"synaptics,irq-gpio", 0, NULL);

	retval = of_property_read_u32(np, "synaptics,irq-on-state",
			&value);
	if (retval < 0)
		bdata->irq_on_state = 0;
	else
		bdata->irq_on_state = value;

	bdata->irq_flags = IRQF_TRIGGER_LOW | IRQF_ONESHOT;


	if (of_find_property(np, "synaptics,power-gpio", NULL)) {
		bdata->power_gpio = of_get_named_gpio_flags(np,
				"synaptics,power-gpio", 0, NULL);
		if (!gpio_is_valid(bdata->power_gpio)) {
			dev_err(dev, "%s: power GPIO %d, is invalid\n",
				__func__, bdata->power_gpio);
			return -EPROBE_DEFER;
		}

		retval = of_property_read_u32(np, "synaptics,power-on-status",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->power_on_state = value;
	} else {
		bdata->power_gpio = -1;
	}

	if (of_find_property(np, "synaptics,main-power-gpio", NULL)) {
		bdata->main_power_gpio = of_get_named_gpio_flags(np,
				"synaptics,main-power-gpio", 0, NULL);
		if (!gpio_is_valid(bdata->main_power_gpio)) {
			dev_err(dev, "%s: main power GPIO %d is invalid\n",
				__func__, bdata->main_power_gpio);
			return -EPROBE_DEFER;
		}

		retval = of_property_read_u32(np, "synaptics,main-power-on-status",
				&value);
		if (retval < 0)
			return retval;
		else
			bdata->main_power_on_state = value;
	} else {
		bdata->main_power_gpio = -1;
	}

	tmp = 0;
	retval = of_property_read_u32(np, "synaptics,power-delay-ms", &tmp);
	if (!retval)
		bdata->power_delay_ms = tmp;
	else
		bdata->power_delay_ms = 0;

	if (of_find_property(np, "synaptics,reset-gpio", NULL)) {
		bdata->reset_gpio = of_get_named_gpio_flags(np,
				"synaptics,reset-gpio", 0, NULL);
		retval = of_property_read_u32(np, "synaptics,reset-on-status",
				&value);
		if (retval < 0) {
			dev_err(dev,
			"%s: failed to read synaptics,reset-on-status\n",
			__func__);
			return retval;
		} else
			bdata->reset_on_state = value;

		retval = of_property_read_u32(np, "synaptics,reset-active-ms",
				&value);
		if (retval < 0) {
			dev_err(dev,
			"%s: failed to read synaptics,reset-active-ms\n",
			__func__);
			return retval;
		} else
			bdata->reset_active_ms = value;
	} else {
		bdata->reset_gpio = -1;
	}

	tmp = 0;
	retval = of_property_read_u32(np, "synaptics,reset-delay-ms", &tmp);
	if (!retval)
		bdata->reset_delay_ms = tmp;
	else
		bdata->reset_delay_ms = 0;

	if (of_property_read_u32(np, "synaptics,resolution_x",
			&bdata->resolution_x))
		bdata->resolution_x = 0;

	if (of_property_read_u32(np, "synaptics,resolution_y",
			&bdata->resolution_y))
		bdata->resolution_y = 0;

	bdata->swap_axes = of_property_read_bool(np, "synaptics,swap-axes");
	dev_dbg(dev,
			"%s: synaptics,swap-axes = (%d)\n",
			__func__,
			bdata->swap_axes);

	/* regulator info */
	bdata->reg_en = of_property_read_bool(np, "synaptics,reg-en");
	dev_dbg(dev,
			"%s: synaptics,reg-en = (%d)\n",
			__func__,
			bdata->reg_en);

	bdata->i2c_pull_up = of_property_read_bool(np, "synaptics,i2c-pull-up");
	dev_dbg(dev,
			"%s: synaptics,i2c-pull-up = (%d)\n",
			__func__,
			bdata->i2c_pull_up);

	tmp = 0;
	retval = of_property_read_u32(np, "synaptics,x-flip", &tmp);
	if (!retval)
		bdata->x_flip = tmp;
	else
		bdata->x_flip = 0;

	tmp = 0;
	retval = of_property_read_u32(np, "synaptics,y-flip", &tmp);
	if (!retval)
		bdata->y_flip = tmp;
	else
		bdata->y_flip = 0;

	prop = of_find_property(np, "synaptics,button-map", NULL);
	if (prop) {
		bdata->cap_button_map->nbuttons = prop->length / sizeof(tmp);
		retval = of_property_read_u32_array(np,
			"synaptics,button-map",
			(unsigned int *)bdata->cap_button_map->map,
			bdata->cap_button_map->nbuttons);
		if (retval) {
			dev_err(dev, "Unable to read key codes\n");
			bdata->cap_button_map->map = NULL;
		}
	}

	if (bdata->reg_en) {
		bdata->vcc = regulator_get(dev, "vcc");
		if (IS_ERR(bdata->vcc)) {
			retval = PTR_ERR(bdata->vcc);
			dev_err(dev,
				"Regulator get failed vcc rc=%d\n", retval);
			bdata->reg_en = false;
		}
	}

	if (bdata->i2c_pull_up) {
		bdata->vcc_i2c = regulator_get(dev, "vcc_i2c");
		if (IS_ERR(bdata->vcc_i2c)) {
			retval = PTR_ERR(bdata->vcc_i2c);
			dev_err(dev,
				"Regulator get failed vcc_i2c rc=%d\n", retval);
			bdata->i2c_pull_up = false;
		}
	}

	tmp = 0;
	retval = of_property_read_u32(np, "synaptics,watchdog-timeout-s", &tmp);
	if  (!retval)
		bdata->watchdog_timeout_s = tmp;
	else
		bdata->watchdog_timeout_s = 0;

	/* Selectively apply power management */
	bdata->pm_disabled = of_property_read_bool(np, "synaptics,pm-disabled");

	/* Touch controls power to the DDIC */
	bdata->ddic_power_control = of_property_read_bool(np,
					"synaptics,ddic-power-control");

	/* Wakeup gesture support */
	bdata->wg_enabled = of_property_read_bool(np,
					"synaptics,wakeup-gesture");

	/* Wakeup gesture without criteria */
	bdata->wg_no_ct = of_property_read_bool(np,
					"synaptics,wakeup-gesture-no-criteria");

	if (bdata->wg_enabled) {
		int fw_wake_zone_width;

		memcpy(&bdata->wakeup_criteria, &default_wakeup_criteria,
			sizeof(struct synaptics_wakeup_criteria_t));
		bdata->fw_wake_swipe_distance = SYNAPTICS_FIRMWARE_MIN_WAKE_DISTANCE;
		bdata->fw_wake_zone_top_left_x = SYNAPTICS_FIRMWARE_ZONE_TOP_LEFT_X;
		bdata->fw_wake_zone_top_left_y = SYNAPTICS_FIRMWARE_ZONE_TOP_LEFT_Y;
		bdata->fw_wake_zone_bottom_right_x = SYNAPTICS_FIRMWARE_ZONE_BOTTOM_RIGHT_X;
		bdata->fw_wake_zone_bottom_right_y = SYNAPTICS_FIRMWARE_ZONE_BOTTOM_RIGHT_Y;
		bdata->fw_wake_swipe_min_speed = SYNAPTICS_FIRMWARE_MIN_WAKE_SPEED;
		bdata->fw_wake_swipe_max_speed = SYNAPTICS_WAKEUP_MAX_WAKE_SPEED;

		fw_wake_zone_width = ((bdata->fw_wake_zone_bottom_right_x - bdata->fw_wake_zone_top_left_x)*30)/100;
		bdata->fw_wake_zone_top_left_x1 = bdata->fw_wake_zone_top_left_x + fw_wake_zone_width;
		bdata->fw_wake_zone_bottom_right_x1 = bdata->fw_wake_zone_bottom_right_x - fw_wake_zone_width;

		bdata->wakeup_criteria.max_speed =
			(bdata->fw_wake_swipe_distance + bdata->fw_wake_swipe_max_speed) *
			(bdata->fw_wake_swipe_distance + bdata->fw_wake_swipe_max_speed);


	}
	/* Restart system after firmware upgrade */
	bdata->restart_sys_fw_upgrade = of_property_read_bool(np,
					"synaptics,restart-sys-after-fw-upgrade");

	/* Lockup check */
	tmp = 0;
	retval = of_property_read_u32(np,
			"synaptics,lockup-poll-interval-ms", &tmp);
	if  (!retval)
		bdata->lockup_poll_interval_ms = tmp;
	else
		bdata->lockup_poll_interval_ms = 0;

	if (bdata->lockup_poll_interval_ms) {
		tmp = 0;
		retval = of_property_read_u32(np,
				"synaptics,lockup-poll-count", &tmp);
		if  (!retval)
			bdata->lockup_poll_count = tmp;
		else
			bdata->lockup_poll_count = LOCKUP_POLLING_COUNT;
	}

	/* disable while in holster */
	bdata->dis_in_holster =
		of_property_read_bool(np, "disable-in-holster");

	/* ignore the touches while sliding */
	bdata->dis_while_sliding =
			of_property_read_bool(np, "disable-while-sliding");

	/* disable while slider is closed */
	bdata->dis_in_slider =
			of_property_read_bool(np, "disable-in-closed-slider");

	/* enable_abs_cancel event support */
	bdata->enable_abs_cancel =
			of_property_read_bool(np, "enable-touch-cancel");

	/* enable_abs_edge event support */
	bdata->enable_abs_edge =
			of_property_read_bool(np, "enable-edge-touches");

	tmp = 0;
	retval = of_property_read_u32(np, "number-of-slider-hall-sensors", &tmp);
	if  (!retval)
		bdata->num_of_slider_hall_sensors = tmp;
	else
		bdata->num_of_slider_hall_sensors = 2;  /* default value */

#if defined(CONFIG_BBRY_MFG) || defined(CONFIG_BBRY_DEBUG)
	tmp = 0;
	retval = of_property_read_u32(np, "synaptics,bist-min", &tmp);
	if  (!retval)
		bdata->bist_min = tmp;
	else
		bdata->bist_min = 0;

	tmp = 0;
	retval = of_property_read_u32(np, "synaptics,bist-max", &tmp);
	if  (!retval)
		bdata->bist_max = tmp;
	else
		bdata->bist_max = BIST_MAX;

	tmp = 0;
	retval = of_property_read_u32(np, "synaptics,report-type", &tmp);
	if  (!retval)
		bdata->report_type = tmp;
	else
		bdata->report_type = -1;

	tmp = 0;
	retval = of_property_read_u32(np, "synaptics,disable-baseline-compensation-in-BIST", &tmp);
	if  (!retval)
		bdata->disable_bc_bist = tmp;
	else
		bdata->disable_bc_bist = 1;

#endif /*CONFIG_BBRY_MFG || CONFIG_BBRY_DEBUG*/


	prop = of_find_property(np, "synaptics,product-id-major", NULL);
	if (prop) {
		of_property_read_string(np,
				"synaptics,product-id-major",
				&bdata->product_id_major);
	} else {
		bdata->product_id_major = NULL;
	}

	dev_info(dev,
		"%s: synaptics,product-id-major = %s\n",
		__func__,
		bdata->product_id_major);

	prop = of_find_property(np, "synaptics,blacklist-product-id", NULL);
	if (prop) {
		of_property_read_string(np,
				"synaptics,blacklist-product-id",
				&bdata->bl_product_id);

		dev_info(dev,
			"%s: synaptics,blacklist-product-id = %s\n",
			__func__,
			bdata->bl_product_id);
	} else {
		bdata->bl_product_id = NULL;
	}

	tmp = 0;
	retval = of_property_read_u32(np, "tap-status-addr", &tmp);
	if  (!retval)
		bdata->tap_status_addr = tmp;
	else
		bdata->tap_status_addr = -1;

	tmp = 0;
	retval = of_property_read_u32(np, "position-buffer-addr", &tmp);
	if  (!retval)
		bdata->pos_buf_addr = tmp;
	else
		bdata->pos_buf_addr = -1;

	of_property_read_string(np,
				"synaptics,input-device-name",
				&bdata->input_dev_name);

	return 0;
}
#else
static int synaptics_rmi4_parse_dt(struct device *dev,
				struct synaptics_dsx_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static int synaptics_rmi4_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	int retval;
	struct synaptics_dsx_hw_interface *hw_if;
	struct platform_device *synaptics_dsx_i2c_device;
	static int device_id;

	dev_dbg(&client->dev,
		"%s: ENTRY\n",
		__func__);

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"%s: SMBus byte data commands not supported by host\n",
				__func__);
		return -EIO;
	}

	synaptics_dsx_i2c_device = kzalloc(
			sizeof(struct platform_device),
			GFP_KERNEL);
	if (!synaptics_dsx_i2c_device) {
		dev_err(&client->dev,
				"%s: Failed to allocate memory for synaptics_dsx_i2c_device\n",
				__func__);
		return -ENOMEM;
	}

	hw_if = kzalloc(
		sizeof(struct synaptics_dsx_hw_interface), GFP_KERNEL);
	if (!hw_if) {
		dev_err(&client->dev,
			"%s: Failed to allocate memory for hw_if\n",
			__func__);
		return -ENOMEM;
	}

#ifdef CONFIG_OF
	if (client->dev.of_node) {
		hw_if->board_data = kzalloc(
			sizeof(struct synaptics_dsx_board_data), GFP_KERNEL);
		if (!hw_if->board_data) {
			dev_err(&client->dev,
				"%s: Failed to allocate memory for pdata\n",
				__func__);
			return -ENOMEM;
		}

		retval = synaptics_rmi4_parse_dt(&client->dev, hw_if->board_data);
		if (retval < 0) {
			dev_err(&client->dev,
				"%s: Failed to parse device tree %d\n",
				__func__, retval);
			return retval;
		}
	}
#else
	hw_if->board_data = client->dev.platform_data;
#endif
	hw_if->bus_access = &bus_access;

	synaptics_dsx_i2c_device->name = PLATFORM_DRIVER_NAME;
	synaptics_dsx_i2c_device->id = device_id++;
	synaptics_dsx_i2c_device->num_resources = 0;
	synaptics_dsx_i2c_device->dev.parent = &client->dev;
	synaptics_dsx_i2c_device->dev.platform_data = hw_if;
	synaptics_dsx_i2c_device->dev.release = synaptics_rmi4_i2c_dev_release;
	dev_set_drvdata(&client->dev, synaptics_dsx_i2c_device);

	retval = platform_device_register(synaptics_dsx_i2c_device);
	if (retval) {
		dev_err(&client->dev,
				"%s: Failed to register platform device\n",
				__func__);
		return -ENODEV;
	}

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_enable(&client->dev);
#endif

	dev_dbg(&client->dev,
		"%s: EXIT\n",
		__func__);

	return 0;
}

static int synaptics_rmi4_i2c_remove(struct i2c_client *client)
{
	return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int synaptics_rmi4_i2c_runtime_resume(struct device *dev)
{
	struct platform_device *platform_dev = dev_get_drvdata(dev);

	return synaptics_rmi4_runtime_resume(&platform_dev->dev);
}

static int synaptics_rmi4_i2c_runtime_idle(struct device *dev)
{
	return 0;
}

static int synaptics_rmi4_i2c_runtime_suspend(struct device *dev)
{
	struct platform_device *platform_dev = dev_get_drvdata(dev);

	return synaptics_rmi4_runtime_suspend(&platform_dev->dev);
}

#endif

static const struct i2c_device_id synaptics_rmi4_id_table[] = {
	{I2C_DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, synaptics_rmi4_id_table);

#ifdef CONFIG_OF
static struct of_device_id synaptics_match_table[] = {
	{ .compatible = "synaptics,dsx",},
	{ },
};
#else
#define synaptics_match_table NULL
#endif

#ifdef CONFIG_PM_SLEEP
int synaptics_rmi4_i2c_suspend(struct device *dev)
{
	struct platform_device *platform_dev = dev_get_drvdata(dev);

	return synaptics_rmi4_suspend(&platform_dev->dev);

}

int synaptics_rmi4_i2c_resume(struct device *dev)
{
	struct platform_device *platform_dev = dev_get_drvdata(dev);

	return synaptics_rmi4_resume(&platform_dev->dev);

}
#endif

static const struct dev_pm_ops synaptics_rmi4_i2c_pm_ops = {
#ifdef CONFIG_PM_SLEEP
	SET_SYSTEM_SLEEP_PM_OPS(synaptics_rmi4_i2c_suspend,
					synaptics_rmi4_i2c_resume)
#endif
#ifdef CONFIG_PM_RUNTIME
	SET_RUNTIME_PM_OPS(synaptics_rmi4_i2c_runtime_suspend,
					   synaptics_rmi4_i2c_runtime_resume,
					   synaptics_rmi4_i2c_runtime_idle)
#endif
};

static struct i2c_driver synaptics_rmi4_i2c_driver = {
	.driver = {
		.name = I2C_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = synaptics_match_table,
#ifdef CONFIG_PM
		.pm = &synaptics_rmi4_i2c_pm_ops,
#endif
	},
	.probe = synaptics_rmi4_i2c_probe,
	.remove = synaptics_rmi4_i2c_remove,
	.id_table = synaptics_rmi4_id_table,
};

int synaptics_rmi4_bus_init(void)
{
	return i2c_add_driver(&synaptics_rmi4_i2c_driver);
}
EXPORT_SYMBOL(synaptics_rmi4_bus_init);

void synaptics_rmi4_bus_exit(void)
{
	i2c_del_driver(&synaptics_rmi4_i2c_driver);

	return;
}
EXPORT_SYMBOL(synaptics_rmi4_bus_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX I2C Bus Support Module");
MODULE_LICENSE("GPL v2");
