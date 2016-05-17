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
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <media/msm_cam_sensor.h>
#include "msm_camera_i2c.h"
#include "bbry_flash.h"

#undef pr_fmt
#define pr_fmt(fmt)	"MAX77387: " fmt

#define FLASH_NAME "bbry-camera-flash"

enum {
	REG_CHIP_ID1		= 0x00,
	REG_CHIP_ID2		= 0x01,
	REG_STATUS1		= 0x02,
	REG_STATUS2		= 0x03,
	REG_I_FLASH1		= 0x04,
	REG_I_FLASH2		= 0x05,
	REG_I_TORCH1		= 0x06,
	REG_I_TORCH2		= 0x07,
	REG_MODE_SEL		= 0x08,
	REG_TX1_MASK		= 0x09,
	REG_TX2_MASK		= 0x0A,
	REG_FLASH_RAMP_SEL	= 0x0B,
	REG_TORCH_RAMP_SEL	= 0x0C,
	REG_FLASH_TMR_CNTL	= 0x0D,
	REG_TORCH_TMR_CNTL	= 0x0E,
	REG_MAXFLASH1		= 0x10,
	REG_MAXFLASH2		= 0x11,
	REG_MAXFLASH3		= 0x12,
	REG_MAXFLASH4		= 0x13,
	REG_NTC			= 0x14,
	REG_DCDC_CNTL1		= 0x15,
	REG_DCDC_CNTL2		= 0x16,
	REG_DCDC_ILIM		= 0x17,
	REG_DCDC_OUT		= 0x18,
	REG_DCDC_OUT_MAX	= 0x19,
};

#define MAX77387_CHIP_ID	0x91

/* TODO: Make names more descriptive of value */
#define MASK_TORCH_EN_PD	0x80
#define MASK_FLASH_STB_PD	0x40
#define MASK_TX1_MASK_EN	0x80
#define MASK_TX1_MASK_PD	0x40
#define MASK_FLASH_TMR_CNTL	0x80
#define MASK_TORCH_TMR_CNTL	0x80
#define MASK_TORCH_MODE		0x38
#define MASK_FLASH_MODE		0x07
#define MASK_TORCH1_EN		0x80
#define MASK_TORCH1_CURRENT	0x7E
#define MASK_FLASH1_EN		0x80
#define MASK_FLASH2_EN		0x80
#define MASK_FLASH_CURRENT	0x3F

#define TORCH_MODE_ENABLE	0x38

#define DCDC_CNTL1_DEFAULT	0xC0
#define DCDC_CNTL2_DEFAULT	0x8C
#define DCDC_ILIM_DEFAULT	0x52

/* maximum flash time */

#define FLASH_TIMEOUT_DEFAULT	0x66 /* 494 ms */

/* max current per channel during GSM call */
#define GSM_PA_CURRENT		0x02 /* 46.875 mA per channel */

/* Droop the output if VSYS drops below this threshold */
#define MAXFLASH_THRESHOLD	0x13 /* 3.0 volts */

/* Droop hysteresis - VSYS must rise to the threshold plus this value before the
 * droop is removed.
 */
#define MAXFLASH_HYSTERESIS	0x20 /* 50 mV */

/* Flash is triggered by FLASH_STB pin */
#define FLASH_MODE_STB_TRIGGER	0x2

#define TORCH_MAX_CURRENT_UA	250000	/* 250 mA */
#define TORCH_MAX_STEP		0x3F
#define TORCH_STEP_SIZE_UA	3910	/* 3.91 mA */

#define FLASH_MAX_CURRENT_UA	1000000	/* 1000 mA */
#define FLASH_MAX_STEP		0x3F
#define FLASH_STEP_SIZE_UA	15625	/* 15.625 mA */

#define REGULATOR_VOLTAGE_UV	1800000	/* 1.8 volts */

static void __exit max77387_i2c_remove(void);
static int max77387_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id);

static struct bbry_flash_ctrl_t *flash_ctrl;
static struct i2c_driver max77387_i2c_driver;
static struct msm_camera_i2c_client max77387_i2c_client;

static const struct of_device_id max77387_dt_match[] = {
	{.compatible = "maxim,max77387"},
	{ }
};

static const struct i2c_device_id max77387_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&flash_ctrl},
	{ }
};

static struct i2c_driver max77387_i2c_driver = {
	.id_table = max77387_i2c_id,
	.probe  = max77387_i2c_probe,
	.remove = __exit_p(max77387_i2c_remove),
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = max77387_dt_match,
	},
};

__always_inline int32_t max77387_read_register(uint32_t reg_addr)
{
	uint16_t reg_value = 0;
	int rc;

	rc = msm_camera_qup_i2c_read(&max77387_i2c_client, reg_addr,
			&reg_value, MSM_CAMERA_I2C_BYTE_DATA);

	return (rc < 0) ? rc : reg_value;
}

__always_inline int32_t max77387_write_register(uint32_t reg_addr,
						uint8_t reg_value)
{
	return msm_camera_qup_i2c_write(&max77387_i2c_client, reg_addr,
			reg_value, MSM_CAMERA_I2C_BYTE_DATA);
}

static int32_t max77387_power_on(int microvolts)
{
	int rc;
	pr_debug("%s Enter\n", __func__);

	if (IS_ERR(flash_ctrl->vdd)) {
		pr_err("Invalid regulator pointer\n");
		return -EINVAL;
	}

	rc = regulator_set_voltage(flash_ctrl->vdd, microvolts, microvolts);
	if (rc < 0) {
		pr_err("Setting vdd voltage to %d uV failed\n",
			microvolts);
		return -EINVAL;
	}

	if (!flash_ctrl->vdd_enabled) {
		rc = regulator_enable(flash_ctrl->vdd);
		if (rc < 0) {
			pr_err("Enable vdd failed\n");
			return -EINVAL;
		}
		flash_ctrl->vdd_enabled = true;
	}

	return 0;
}

static int32_t max77387_power_off(void)
{
	int rc;
	pr_debug("%s Enter\n", __func__);

	if (IS_ERR(flash_ctrl->vdd)) {
		pr_err("Invalid regulator pointer\n");
		return -EINVAL;
	}

	if (flash_ctrl->vdd_enabled) {
		rc = regulator_disable(flash_ctrl->vdd);
		if (rc < 0) {
			pr_err("failed to disable vdd\n");
			return -EINVAL;
		}
		flash_ctrl->vdd_enabled = false;
	}

	return 0;
}

int32_t max77387_clear_faults(void)
{
	int32_t reg_value;

	/* Reading the status registers clears existing faults */
	reg_value = max77387_read_register(REG_STATUS2);
	if (reg_value < 0) {
		pr_err("Failed to read register STATUS2 (0x%02X)\n",
			REG_STATUS2);
	}

	reg_value = max77387_read_register(REG_STATUS1);
	if (reg_value < 0) {
		pr_err("Failed to read register STATUS1 (0x%02X)\n",
			REG_STATUS1);
	} else if (reg_value > 0) {
		pr_err("Register STATUS1 (0x%02X) had faults 0x%02X\n",
			REG_STATUS1, reg_value);
	}

	return reg_value;
}

__always_inline uint32_t max77387_mA_to_uA(uint32_t current_mA)
{
	return current_mA * 1000;
}

__always_inline uint32_t max77387_uA_to_mA(uint32_t current_uA)
{
	return current_uA / 1000;
}

__always_inline int32_t max77387_torch_round_current(int32_t current_uA)
{
	/* Round down the current to the next multiple of TORCH_STEP_SIZE_UA */
	return (current_uA / TORCH_STEP_SIZE_UA) * TORCH_STEP_SIZE_UA;
}

__always_inline int32_t max77387_flash_round_current(int32_t current_uA)
{
	/* Round down the current to the next multiple of FLASH_STEP_SIZE_UA */
	return (current_uA / FLASH_STEP_SIZE_UA) * FLASH_STEP_SIZE_UA;
}

int32_t max77387_uA_to_torch_steps(int32_t current_uA)
{
	int32_t register_value;

	if (current_uA < TORCH_STEP_SIZE_UA)
		register_value = -1;
	else if (current_uA >= TORCH_MAX_CURRENT_UA)
		register_value = TORCH_MAX_STEP;
	else
		register_value = (current_uA / TORCH_STEP_SIZE_UA) - 1;

	return register_value;
}

int32_t max77387_uA_to_flash_steps(int32_t current_uA)
{
	int32_t register_value;

	if (current_uA < FLASH_STEP_SIZE_UA)
		register_value = -1;
	else if (current_uA >= FLASH_MAX_CURRENT_UA)
		register_value = FLASH_MAX_STEP;
	else
		register_value = (current_uA / FLASH_STEP_SIZE_UA) - 1;

	return register_value;
}

int32_t max77387_torch_steps_to_uA(int32_t torch_steps)
{
	int32_t current_uA;

	if (torch_steps < 0)
		current_uA = 0;
	else if (torch_steps >= TORCH_MAX_STEP)
		current_uA = TORCH_MAX_CURRENT_UA;
	else
		current_uA = (torch_steps + 1) * TORCH_STEP_SIZE_UA;

	return current_uA;
}

int32_t max77387_flash_steps_to_uA(int32_t flash_steps)
{
	int32_t current_uA;

	if (flash_steps < 0)
		current_uA = 0;
	else if (flash_steps >= FLASH_MAX_STEP)
		current_uA = FLASH_MAX_CURRENT_UA;
	else
		current_uA = (flash_steps + 1) * FLASH_STEP_SIZE_UA;

	return current_uA;
}

int32_t max77387_flash_init(struct bbry_flash_device_t *flash,
			struct bbry_flash_cmd_t *cmd)
{
	pr_debug("%s Enter\n", __func__);

	max77387_power_on(REGULATOR_VOLTAGE_UV);

	max77387_clear_faults();

	max77387_write_register(REG_MODE_SEL,
			MASK_TORCH_EN_PD | MASK_FLASH_STB_PD);
	max77387_write_register(REG_I_FLASH1, 0);
	max77387_write_register(REG_I_TORCH1, 0);
	max77387_write_register(REG_I_FLASH2, 0);
	max77387_write_register(REG_I_TORCH2, 0);
	max77387_write_register(REG_TX2_MASK, 0);
	max77387_write_register(REG_FLASH_RAMP_SEL, 0);
	max77387_write_register(REG_TORCH_RAMP_SEL, 0);
	max77387_write_register(REG_TX1_MASK,
			MASK_TX1_MASK_EN | MASK_TX1_MASK_PD | GSM_PA_CURRENT);
	max77387_write_register(REG_FLASH_TMR_CNTL,
			MASK_FLASH_TMR_CNTL | FLASH_TIMEOUT_DEFAULT);
	max77387_write_register(REG_TORCH_TMR_CNTL, MASK_TORCH_TMR_CNTL);
	max77387_write_register(REG_MAXFLASH1,
			MAXFLASH_HYSTERESIS | MAXFLASH_THRESHOLD);
	max77387_write_register(REG_DCDC_CNTL1, DCDC_CNTL1_DEFAULT);
	max77387_write_register(REG_DCDC_CNTL2, DCDC_CNTL2_DEFAULT);
	max77387_write_register(REG_DCDC_ILIM, DCDC_ILIM_DEFAULT);

	return 0;
}

int32_t max77387_flash_off(struct bbry_flash_device_t *flash,
			struct bbry_flash_cmd_t *cmd)
{
	pr_debug("%s Enter\n", __func__);

	/* Disable flash and torch modes */
	max77387_write_register(REG_MODE_SEL,
			MASK_TORCH_EN_PD | MASK_FLASH_STB_PD);

	/* Disable flashes and torch and set the current to the minimum */
	max77387_write_register(REG_I_FLASH1, 0);
	max77387_write_register(REG_I_FLASH2, 0);
	max77387_write_register(REG_I_TORCH1, 0);

	cmd->applied_current = 0;

	return 0;
}

int32_t max77387_flash_release(struct bbry_flash_device_t *flash,
			struct bbry_flash_cmd_t *cmd)
{
	pr_debug("%s Enter\n", __func__);

	max77387_flash_off(flash, cmd);
	max77387_power_off();

	return 0;
}

int32_t max77387_flash_low(struct bbry_flash_device_t *flash,
			struct bbry_flash_cmd_t *cmd)
{
	int32_t torch_current_mA = cmd->limited_current;
	int32_t torch_current_uA;
	int32_t current_steps;
	int32_t mode;

	pr_debug("%s Enter\n", __func__);

	max77387_clear_faults();

	mode = max77387_read_register(REG_MODE_SEL);
	if (mode < 0) {
		pr_err("Unable to read register MODE_SEL (0x%02X)\n",
			REG_MODE_SEL);
		return -EINVAL;
	}

	torch_current_uA = max77387_mA_to_uA(torch_current_mA);
	current_steps = max77387_uA_to_torch_steps(torch_current_uA);
	if (current_steps < 0) {
		max77387_write_register(REG_MODE_SEL, mode & ~MASK_TORCH_MODE);
		max77387_write_register(REG_I_TORCH1, 0);
	} else {
		current_steps <<= 1;

		/* If the torch has already been triggered you must set the
		 * torch mode to 000 before setting it to 111 to activate the
		 * trigger again.
		 */
		if ((mode & MASK_TORCH_MODE) != 0) {
			max77387_write_register(REG_MODE_SEL,
						mode & ~MASK_TORCH_MODE);
		}

		max77387_write_register(REG_I_TORCH1,
					MASK_TORCH1_EN | current_steps);
		max77387_write_register(REG_MODE_SEL,
					(mode &
					~(MASK_TORCH_MODE | MASK_FLASH_MODE)) |
					TORCH_MODE_ENABLE);
	}

	/* Pass back the current we actually programmed */
	torch_current_uA = max77387_torch_round_current(torch_current_uA);
	cmd->applied_current = max77387_uA_to_mA(torch_current_uA);

	return 0;
}

int32_t max77387_flash_high(struct bbry_flash_device_t *flash,
			struct bbry_flash_cmd_t *cmd)
{
	int32_t flash_current_mA = cmd->limited_current;
	int32_t flash_current_uA;
	int32_t flash1_current_uA;
	int32_t flash2_current_uA;
	int32_t current_steps;
	int32_t mode;

	pr_debug("%s Enter\n", __func__);

	max77387_clear_faults();

	/* Split the current as equally as possible between the two outputs */
	flash_current_uA = max77387_mA_to_uA(flash_current_mA);
	flash1_current_uA = max77387_flash_round_current(flash_current_uA / 2);
	flash2_current_uA = flash_current_uA - flash1_current_uA;

	mode = max77387_read_register(REG_MODE_SEL);
	if (mode < 0) {
		pr_err("Unable to read register MODE_SEL (0x%02X)\n",
			REG_MODE_SEL);
		return -EINVAL;
	}

	/* If the flash has already been triggered you must set the flash mode
	 * to 000 before setting it to 010 to activate the trigger again.
	 */
	if ((mode & MASK_FLASH_MODE) != 0)
		max77387_write_register(REG_MODE_SEL, mode & ~MASK_FLASH_MODE);

	/* Program the first flash line's current */
	current_steps = max77387_uA_to_flash_steps(flash1_current_uA);
	if (current_steps < 0) {
		max77387_write_register(REG_I_FLASH1, 0);
	} else {
		max77387_write_register(REG_I_FLASH1,
					MASK_FLASH1_EN | current_steps);
	}

	/* Program the second flash line's current.
	 * If the total current is too small to be split it will be applied to
	 * the second line only.
	 */
	current_steps = max77387_uA_to_flash_steps(flash2_current_uA);
	if (current_steps < 0) {
		max77387_write_register(REG_I_FLASH2, 0);
	} else {
		max77387_write_register(REG_I_FLASH2,
					MASK_FLASH2_EN | current_steps);
		max77387_write_register(REG_MODE_SEL,
					(mode &
					~(MASK_TORCH_MODE | MASK_FLASH_MODE)) |
					FLASH_MODE_STB_TRIGGER);
	}

	/* Pass back the current we actually programmed */
	flash_current_uA = flash1_current_uA +
				max77387_flash_round_current(flash2_current_uA);
	cmd->applied_current = max77387_uA_to_mA(flash_current_uA);

	return 0;
}

#ifdef CONFIG_BBRY_MFG
int32_t max77387_flash_status(struct bbry_flash_device_t *flash,
			struct bbry_flash_cmd_t *cmd)
{
	int32_t reg_value;
	int32_t mode;
	int32_t curr;

	pr_debug("%s Enter\n", __func__);

	mode = max77387_read_register(REG_MODE_SEL);
	if (mode < 0) {
		pr_err("Flash mode failed rc = %d\n", mode);
		return mode;
	}
	reg_value = max77387_read_register(REG_DCDC_OUT);
	if (reg_value < 0) {
		pr_err("Flash get DCDC OUT failed rc = %d\n", reg_value);
		return reg_value;
	}
	pr_info("Flash REG_DCDC_OUT status reg 0x%x\n",
	reg_value);
	cmd->flash_duration = reg_value << 16;
	reg_value = max77387_read_register(REG_STATUS1);
	if (reg_value < 0) {
		pr_err("Flash get status1 failed rc = %d\n", reg_value);
		return reg_value;
	}
	pr_info("Flash REG_STATUS1 status reg 0x%x\n",
	reg_value);
	/* use flash_duration to hold FAULT STATUS register reading */
	cmd->flash_duration |= reg_value << 8;
	reg_value = max77387_read_register(REG_STATUS2);
	if (reg_value < 0) {
		pr_err("Flash get status2 failed rc = %d\n", reg_value);
		return reg_value;
	}
	pr_info("Flash REG_STATUS2 status reg 0x%x\n",
	reg_value);
	cmd->flash_duration |= reg_value;
	if ((mode & MASK_TORCH_MODE) != 0) {
		/* if torch mode is enabled read the torch current */
		reg_value = max77387_read_register(REG_I_TORCH1);
		if (reg_value < 0) {
			pr_err("Torch current failed rc =%d\n", reg_value);
			return reg_value;
		}

		if ((reg_value & MASK_TORCH1_EN) == 0) {
			cmd->applied_current = 0;
		} else {
			curr = (reg_value & MASK_TORCH1_CURRENT) >> 1;
			curr = max77387_torch_steps_to_uA(curr);
			cmd->applied_current = max77387_uA_to_mA(curr);
		}
		pr_info("Torch status reg 0x%x, current %d mA\n",
			cmd->flash_duration, cmd->applied_current);
	} else if ((mode & MASK_FLASH_MODE) != 0) {
		/* if flash mode is enabled read the flash current */
		reg_value = max77387_read_register(REG_I_FLASH1);
		if (reg_value < 0) {
			pr_err("Flash1 current failed rc =%d\n", reg_value);
			return reg_value;
		}

		if ((reg_value & MASK_FLASH1_EN) == 0) {
			cmd->applied_current = 0;
		} else {
			curr = reg_value & MASK_FLASH_CURRENT;
			cmd->applied_current = max77387_flash_steps_to_uA(curr);
		}

		reg_value = max77387_read_register(REG_I_FLASH2);
		if (reg_value < 0) {
			pr_err("Flash2 current failed rc =%d\n", reg_value);
			return reg_value;
		}

		if ((reg_value & MASK_FLASH2_EN) != 0) {
			curr = reg_value & MASK_FLASH_CURRENT;
			cmd->applied_current +=
					max77387_flash_steps_to_uA(curr);
		}
		cmd->applied_current = max77387_uA_to_mA(cmd->applied_current);
		pr_info("Flash status reg 0x%x, current %d mA\n",
			cmd->flash_duration, cmd->applied_current);
	} else {
		cmd->applied_current = 0;
		pr_info("Flash status reg 0x%x, current %d mA\n",
			cmd->flash_duration, cmd->applied_current);
	}

	return 0;
}
#endif /* CONFIG_BBRY_MFG */

int32_t max77387_parse_device_tree(struct device_node *flash_of_node,
				struct device_node *torch_of_node,
				struct bbry_flash_device_t *flash)
{
	int32_t rc;

	/* Read the flash information from the device tree */
	rc = of_property_read_u32(flash_of_node, "max-current",
				&flash->flash.max_current);
	if (rc < 0) {
		pr_err("Flash max-current failed\n");
		return -EINVAL;
	} else if (flash->flash.max_current >
			max77387_uA_to_mA(FLASH_MAX_CURRENT_UA * 2)) {
		pr_err("Flash max current larger than MAX77387 supports\n");
		return -EINVAL;
	}
	pr_info("Flash max current %d mA\n", flash->flash.max_current);

	rc = of_property_read_u32(flash_of_node, "default-current",
				&flash->flash.default_current);
	if (rc < 0) {
		pr_err("Flash default-current failed\n");
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
	rc = of_property_read_u32(torch_of_node, "max-current",
				&flash->torch.max_current);
	if (rc < 0) {
		pr_err("Torch max-current failed\n");
		return -EINVAL;
	} else if (flash->torch.max_current >
			max77387_uA_to_mA(TORCH_MAX_CURRENT_UA)) {
		pr_err("Torch max current larger than MAX77387 supports\n");
		return -EINVAL;
	}
	pr_info("Torch max current %d mA\n", flash->torch.max_current);

	rc = of_property_read_u32(torch_of_node, "default-current",
				&flash->torch.default_current);
	if (rc < 0) {
		pr_err("Torch default-current failed\n");
		return -EINVAL;
	}
	if (flash->torch.default_current > flash->torch.max_current) {
		pr_err(
			"Torch default current %d mA greater than torch max current %d mA\n",
			flash->torch.default_current, flash->torch.max_current);
		return -EINVAL;
	}
	pr_info("Torch default current %d mA\n", flash->torch.default_current);

	flash_ctrl->vdd_enabled = false;
	flash_ctrl->vdd = regulator_get(&flash_ctrl->pdev->dev, "vdd");
	if (IS_ERR(flash_ctrl->vdd)) {
		pr_err("Regulator get for vdd failed\n");
		return -EINVAL;
	}
	return 0;
}

int32_t max77387_flash_probe(struct bbry_flash_ctrl_t *fctrl,
			struct device_node *flash_of_node,
			struct device_node *torch_of_node,
			struct bbry_flash_device_t *flash)
{
	int rc;

	pr_debug("%s Enter\n", __func__);

	flash_ctrl = fctrl;
	rc = max77387_parse_device_tree(flash_of_node, torch_of_node, flash);
	if (rc < 0)
		return -EINVAL;

	rc = max77387_power_on(REGULATOR_VOLTAGE_UV);
	if (rc < 0) {
		pr_err("Failed to power on\n");
		return -EINVAL;
	}

	flash->type = MAX77387_FLASH;
	flash->func_tbl.flash_init = &max77387_flash_init;
	flash->func_tbl.flash_release = &max77387_flash_release;
	flash->func_tbl.flash_off = &max77387_flash_off;
	/* Currently torch_on and flash_low use same event handler */
	flash->func_tbl.torch_on = &max77387_flash_low;
	flash->func_tbl.flash_low = &max77387_flash_low;
	flash->func_tbl.flash_high = &max77387_flash_high;
#ifdef CONFIG_BBRY_MFG
	flash->func_tbl.flash_status = &max77387_flash_status;
#endif /* CONFIG_BBRY_MFG */

	return 0;
}

static int max77387_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int reg_value;
	int rc = 0;

	pr_debug("%s Enter\n", __func__);

	if (id == NULL)
		id = max77387_i2c_id;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		return -EINVAL;
	}

	/* msm_camera_qup_i2c_rxdata expects 8 bits slave address */
	client->addr <<= 1;
	max77387_i2c_client.client = client;
	max77387_i2c_client.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;

	max77387_power_on(REGULATOR_VOLTAGE_UV);
	reg_value = max77387_read_register(REG_CHIP_ID1);
	if (reg_value < 0) {
		pr_err("i2c read CHIP_ID1 (0x%02X) failed with error %d\n",
			REG_CHIP_ID1, reg_value);
		rc = -EINVAL;
	} else if (reg_value != MAX77387_CHIP_ID) {
		pr_err(
			"Register CHIP_ID1 (0x%02X) returned 0x%02X (expected 0x%02X)\n",
			REG_CHIP_ID1, reg_value, MAX77387_CHIP_ID);
		rc = -EINVAL;
	}
	/* TODO: Disable flash if ID doesn't match */
	max77387_power_off();
	return 0;
}

static void __exit max77387_i2c_remove(void)
{
	max77387_power_off();
	if (!IS_ERR(flash_ctrl->vdd))
		regulator_put(flash_ctrl->vdd);
	return;
}

static int __init max77387_init_module(void)
{
	int32_t rc = 0;

	pr_debug("%s Enter\n", __func__);

	rc = i2c_add_driver(&max77387_i2c_driver);
	if (rc != 0)
		pr_err("i2c_add_driver failure\n");

	return rc;
}

static void __exit max77387_exit_module(void)
{
	i2c_del_driver(&max77387_i2c_driver);
}

MODULE_DEVICE_TABLE(of, max77387_dt_match);

module_init(max77387_init_module);
module_exit(max77387_exit_module);

MODULE_AUTHOR("BlackBerry Limited");
MODULE_DESCRIPTION("Maxim MAX77387 camera flash I2C driver");
MODULE_LICENSE("GPL v2");
