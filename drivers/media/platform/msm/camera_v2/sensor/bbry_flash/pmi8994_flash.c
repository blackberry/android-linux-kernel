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
#include <linux/leds.h>
#include "bbry_flash.h"

#undef pr_fmt
#define pr_fmt(fmt)	"PMI8994: " fmt

static struct bbry_flash_ctrl_t *flash_ctrl;

int32_t pmi8994_flash_init(struct bbry_flash_device_t *flash,
			struct bbry_flash_cmd_t *cmd)
{
	pr_debug("%s Enter\n", __func__);

	led_trigger_event_not_critical(flash->torch.trigger, 0);
	led_trigger_event_not_critical(flash->flash.trigger, 0);
	return 0;
}

int32_t pmi8994_flash_release(struct bbry_flash_device_t *flash,
			struct bbry_flash_cmd_t *cmd)
{
	pr_debug("%s Enter\n", __func__);

	led_trigger_event_not_critical(flash->torch.trigger, 0);
	led_trigger_event_not_critical(flash->flash.trigger, 0);
	return 0;
}

int32_t pmi8994_flash_off(struct bbry_flash_device_t *flash,
			struct bbry_flash_cmd_t *cmd)
{
	pr_debug("%s Enter\n", __func__);

	led_trigger_event_not_critical(flash->torch.trigger, 0);
	led_trigger_event_not_critical(flash->flash.trigger, 0);

	cmd->applied_current = 0;

	return 0;
}

int32_t pmi8994_flash_low(struct bbry_flash_device_t *flash,
			struct bbry_flash_cmd_t *cmd)
{
	pr_debug("%s Enter\n", __func__);

	led_trigger_event_not_critical(flash->flash.trigger, 0);
	led_trigger_event_not_critical(flash->torch.trigger, cmd->limited_current);

	cmd->applied_current = cmd->limited_current;

	return 0;
}

int32_t pmi8994_flash_high(struct bbry_flash_device_t *flash,
			struct bbry_flash_cmd_t *cmd)
{
	pr_debug("%s Enter\n", __func__);


	led_trigger_event_not_critical(flash->torch.trigger, 0);
	led_trigger_event_not_critical(flash->flash.trigger, cmd->limited_current);

	cmd->applied_current = cmd->limited_current;

	return 0;
}

#ifdef CONFIG_BBRY_MFG
int32_t pmi8994_flash_status(struct bbry_flash_device_t *flash,
			struct bbry_flash_cmd_t *cmd)
{
	int32_t rc = 0;
	uint16_t curr = 0;
	u8 reg_value = 0;

	pr_debug("%s Enter\n", __func__);

	rc = led_trigger_get_status(flash->flash.trigger, &reg_value, &curr);
	if (rc < 0) {
		pr_err("Flash get status failed rc = %d\n", rc);
		return rc;
	}
	pr_info("Flash status reg 0x%x, current %d mA\n", reg_value, curr);
	cmd->applied_current = curr;
	cmd->flash_duration = reg_value;
	curr = 0;
	rc = led_trigger_get_status(flash->torch.trigger, &reg_value, &curr);
	if (rc < 0) {
		pr_err("Torch get status failed rc = %d\n", rc);
		return rc;
	}
	pr_info("Torch status reg 0x%x, current %d mA\n", reg_value, curr);
	cmd->applied_current += curr;

	/* use flash_duration to hold FAULT STATUS register reading */
	cmd->flash_duration |= reg_value;

	return rc;
}
#endif /* CONFIG_BBRY_MFG */

int32_t pmi8994_parse_device_tree(struct device_node *flash_of_node,
				struct device_node *torch_of_node,
				struct bbry_flash_device_t *flash)
{
	int32_t rc;

	/* Read the flash information from the device tree */
	rc = of_property_read_string(flash_of_node, "linux,default-trigger",
				&flash->flash.trigger_name);
	if (rc < 0) {
		rc = of_property_read_string(flash_of_node,
				"qcom,default-led-trigger",
				&flash->flash.trigger_name);
		if (rc < 0) {
			pr_err("Missing flash trigger\n");
			return -EINVAL;
		}
	}
	pr_info("Default flash trigger %s\n", flash->flash.trigger_name);

	rc = of_property_read_u32(flash_of_node, "qcom,max-current",
				&flash->flash.max_current);
	if (rc < 0) {
		pr_err("Missing flash max current\n");
		return -EINVAL;
	}
	pr_info("Flash max current %d mA\n", flash->flash.max_current);

	rc = of_property_read_u32(flash_of_node, "qcom,current",
				&flash->flash.default_current);
	if (rc < 0) {
		pr_err("Missing default flash current\n");
		return -EINVAL;
	}
	if (flash->flash.default_current > flash->flash.max_current) {
		pr_err(
			"Default flash current %d mA greater than flash max current %d mA\n",
			flash->flash.default_current, flash->flash.max_current);
		return -EINVAL;
	}
	pr_info("Flash default current %d mA\n", flash->flash.default_current);

	/* Read the torch information from the device tree */
	rc = of_property_read_string(torch_of_node, "linux,default-trigger",
				&flash->torch.trigger_name);
	if (rc < 0) {
		rc = of_property_read_string(torch_of_node,
					"qcom,default-led-trigger",
					&flash->torch.trigger_name);
		if (rc < 0) {
			pr_err("Missing torch trigger\n");
			return -EINVAL;
		}
	}
	pr_info("Default torch trigger %s\n", flash->torch.trigger_name);

	rc = of_property_read_u32(torch_of_node, "qcom,max-current",
				&flash->torch.max_current);
	if (rc < 0) {
		pr_err("Missing torch max current\n");
		return -EINVAL;
	}
	pr_info("Torch max current %d mA\n", flash->torch.max_current);

	rc = of_property_read_u32(torch_of_node, "qcom,current",
				&flash->torch.default_current);
	if (rc < 0) {
		pr_err("Missing torch default current\n");
		return -EINVAL;
	}
	if (flash->torch.default_current > flash->torch.max_current) {
		pr_err(
			"Default torch current %d mA greater than torch max current %d mA\n",
			flash->torch.default_current, flash->torch.max_current);
		return -EINVAL;
	}
	pr_info("Torch default current %d mA\n", flash->torch.default_current);

	return 0;
}

int32_t pmi8994_flash_probe(struct bbry_flash_ctrl_t *fctrl,
			struct device_node *flash_of_node,
			struct device_node *torch_of_node,
			struct bbry_flash_device_t *flash)
{
	int rc;

	pr_debug("%s Enter\n", __func__);

	rc = pmi8994_parse_device_tree(flash_of_node, torch_of_node, flash);
	if (rc < 0)
		return -EINVAL;

	flash->flash.trigger = NULL;
	led_trigger_register_simple(flash->flash.trigger_name,
				&flash->flash.trigger);
	if (flash->flash.trigger == NULL) {
		pr_err("Failed to register flash trigger\n");
		return -EINVAL;
	}

	flash->torch.trigger = NULL;
	led_trigger_register_simple(flash->torch.trigger_name,
				&flash->torch.trigger);
	if (flash->torch.trigger == NULL) {
		pr_err("Failed to register torch trigger\n");
		return -EINVAL;
	}

	flash->type = PMI8994_FLASH;
	flash_ctrl = fctrl;
	flash->func_tbl.flash_init = &pmi8994_flash_init;
	flash->func_tbl.flash_release = &pmi8994_flash_release;
	flash->func_tbl.flash_off = &pmi8994_flash_off;
	/* Currently torch_on and flash_low use same event handler */
	flash->func_tbl.torch_on = &pmi8994_flash_low;
	flash->func_tbl.flash_low = &pmi8994_flash_low;
	flash->func_tbl.flash_high = &pmi8994_flash_high;
#ifdef CONFIG_BBRY_MFG
	flash->func_tbl.flash_status = &pmi8994_flash_status;
#endif /* CONFIG_BBRY_MFG */

	return 0;
}
