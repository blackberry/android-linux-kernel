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


#include <linux/module.h>
#include <linux/of.h>
#include <asm/div64.h>
#include "bbry_flash.h"
#include "bbry_flash_drivers.h"
#include "bbry_flash_gpio.h"

#define FLASH_NAME "bbry-camera-flash"

#undef pr_fmt
#define pr_fmt(fmt)	"Flash: " fmt

static int32_t bbry_flash_probe(struct platform_device *pdev);
static int32_t bbry_flash_config(struct bbry_flash_ctrl_t *fctrl, void *data);
static int bbry_flash_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh);
static int bbry_flash_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh);

static struct bbry_flash_ctrl_t flash_ctrl;
static struct v4l2_file_operations bbry_flash_v4l2_subdev_fops;

static const struct of_device_id bbry_flash_dt_match[] = {
	{.compatible = "camera-flash"},
	{}
};

static struct platform_driver bbry_flash_driver = {
	.probe = bbry_flash_probe,
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = bbry_flash_dt_match,
	},
};

static const struct v4l2_subdev_internal_ops bbry_flash_internal_ops = {
	.open = bbry_flash_open,
	.close = bbry_flash_close,
};

static int32_t bbry_flash_get_subdev_id(struct bbry_flash_ctrl_t *fctrl,
				void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;

	pr_debug("%s Enter\n", __func__);

	if (!subdev_id) {
		pr_err("%s failed\n", __func__);
		return -EINVAL;
	}

	*subdev_id = fctrl->pdev->id;
	pr_debug("Flash subdev_id %d\n", *subdev_id);

	return 0;
}

static void bbry_flash_set_mitigation_level(struct bbry_flash_ctrl_t *fctrl,
					struct msm_flash_cfg_data_t *cfg)
{
	cfg->flash_mitigation_level = min(cfg->flash_mitigation_level,
					fctrl->flash_mitigation.num_levels - 1);
	fctrl->flash_mitigation.limit =
		fctrl->flash_mitigation.levels[cfg->flash_mitigation_level];

	cfg->torch_mitigation_level = min(cfg->torch_mitigation_level,
					fctrl->torch_mitigation.num_levels - 1);
	fctrl->torch_mitigation.limit =
		fctrl->torch_mitigation.levels[cfg->torch_mitigation_level];

	pr_info(
		"Mitigation: flash level=%d limit=%d mA, torch level=%d limit=%d mA\n",
		cfg->flash_mitigation_level, fctrl->flash_mitigation.limit,
		cfg->torch_mitigation_level, fctrl->torch_mitigation.limit);
}

static void bbry_flash_apply_mitigation(struct bbry_flash_ctrl_t *fctrl,
					struct msm_flash_cfg_data_t *cfg)
{
	int i;
	struct msm_flash_cfg_data_t replay_cfg;

	if ((fctrl->state == CFG_TORCH_ON) ||
		(fctrl->state == CFG_FLASH_LOW) ||
		(fctrl->state == CFG_FLASH_HIGH)) {

		/* Re-create the previous request and apply it again */
		replay_cfg.cfg_type = fctrl->state;
		for (i = 0; i < fctrl->hw_data.flash_count; i++) {
			if (fctrl->flash[i].type == INVALID_FLASH)
				continue;

			replay_cfg.flash_current[i] =
					fctrl->flash[i].state.requested_current;
			replay_cfg.flash_duration[i] =
					fctrl->flash[i].state.flash_duration;
		}
		bbry_flash_config(fctrl, &replay_cfg);

		for (i = 0; i < fctrl->hw_data.flash_count; i++) {
			cfg->flash_current[i] = replay_cfg.flash_current[i];
			cfg->flash_duration[i] = replay_cfg.flash_duration[i];
		}

	}
}

static uint32_t bbry_flash_get_current_limit(struct bbry_flash_ctrl_t *fctrl,
					enum msm_flash_cfg_type_t cfg_type)
{
	uint32_t current_limit;
	uint32_t preflash_ratio_limit;

	/* There are three possible current limits in the system.  Cap the
	 * current at the lowest of the applicable limits.
	 * 1. The system limit based on what the flash mode is.
	 * 2. The mitigation limit based on the mitigation level.
	 * 3. The preflash ratio, which is meant to keep a constant ratio
	 *    between the preflash and main flash.
	 */
	if (cfg_type == CFG_TORCH_ON) {
		current_limit = min(fctrl->torch_max_current,
				fctrl->torch_mitigation.limit);
	} else if (cfg_type == CFG_FLASH_LOW) {
		if (fctrl->use_fixed_ratio && fctrl->preflash_divisor != 0) {
			preflash_ratio_limit = bbry_flash_get_current_limit(
						fctrl, CFG_FLASH_HIGH) /
						fctrl->preflash_divisor;
			current_limit = min(fctrl->low_max_current,
						preflash_ratio_limit);
		} else {
			current_limit = min(fctrl->low_max_current,
						fctrl->flash_mitigation.limit);
		}
	} else {
		current_limit = min(fctrl->high_max_current,
				fctrl->flash_mitigation.limit);
	}

	return current_limit;
}

static __always_inline int32_t bbry_flash_sanitize_current(int32_t curr,
					int32_t max_curr, int32_t default_curr)
{
	return ((curr < 0) || (curr > max_curr)) ? default_curr : curr;
}

static void bbry_flash_cmds_init(struct bbry_flash_ctrl_t *fctrl,
				struct msm_flash_cfg_data_t *cfg,
				struct bbry_flash_cmd_t *cmds)
{
	int i;

	for (i = 0; i < fctrl->hw_data.flash_count; i++) {
		cmds[i].cfg_type = cfg->cfg_type;
		cmds[i].requested_current = cfg->flash_current[i];
		cmds[i].limited_current = 0;
		cmds[i].applied_current = 0;
		cmds[i].flash_duration = cfg->flash_duration[i];
	}
}

static void bbry_flash_cmds_set_limited_current(struct bbry_flash_ctrl_t *fctrl,
						struct bbry_flash_cmd_t *cmds,
						uint32_t system_max_current)
{
	int i;
	int32_t total_current = 0;
	int32_t curr;

	for (i = 0; i < fctrl->hw_data.flash_count; i++) {
		if (fctrl->flash[i].type == INVALID_FLASH)
			continue;

		total_current += cmds[i].requested_current;
	}

	if (total_current > system_max_current) {
		for (i = 0; i < fctrl->hw_data.flash_count; i++) {
			if (fctrl->flash[i].type == INVALID_FLASH)
				continue;

			curr = cmds[i].requested_current;
			cmds[i].limited_current = (cmds[i].requested_current *
						system_max_current) /
						total_current;
			pr_info("Flash %d current limited from %d to %d mA\n",
				i, cmds[i].requested_current,
				cmds[i].limited_current);
		}
	} else {
		for (i = 0; i < fctrl->hw_data.flash_count; i++) {
			if (fctrl->flash[i].type == INVALID_FLASH)
				continue;

			cmds[i].limited_current = cmds[i].requested_current;
		}
	}
}

static void bbry_flash_cmds_set_requested_current(
					struct bbry_flash_ctrl_t *fctrl,
					struct bbry_flash_cmd_t *cmds)
{
	int i;

	for (i = 0; i < fctrl->hw_data.flash_count; i++) {
		if (fctrl->flash[i].type == INVALID_FLASH)
			continue;

		/* if the requested current is invalid, use the dtsi's value */
		if ((cmds[i].cfg_type == CFG_TORCH_ON) ||
			(cmds[i].cfg_type == CFG_FLASH_LOW)) {
			cmds[i].requested_current = bbry_flash_sanitize_current(
					cmds[i].requested_current,
					fctrl->flash[i].torch.max_current,
					fctrl->flash[i].torch.default_current);
		} else {
			cmds[i].requested_current = bbry_flash_sanitize_current(
					cmds[i].requested_current,
					fctrl->flash[i].flash.max_current,
					fctrl->flash[i].flash.default_current);
		}
	}
}

static void bbry_flash_cmds_calc_currents(struct bbry_flash_ctrl_t *fctrl,
					struct bbry_flash_cmd_t *cmds,
					enum msm_flash_cfg_type_t cfg_type)
{
	uint32_t current_limit;

	bbry_flash_cmds_set_requested_current(fctrl, cmds);
	current_limit = bbry_flash_get_current_limit(fctrl, cfg_type);
	bbry_flash_cmds_set_limited_current(fctrl, cmds, current_limit);
}

/* Divide a 16.16 fixed point by another and return a 16.16 fixed point */
static __always_inline uint32_t bbry_flash_fp_divide(uint32_t fp_numerator,
						uint32_t fp_denominator)
{
	uint64_t numerator = ((uint64_t) fp_numerator) * 0x10000;

	do_div(numerator, fp_denominator);
	return numerator;
}

static uint32_t bbry_flash_interpolate_flux(struct bbry_flux_curve_t curve,
					uint32_t current_mA)
{
	int lower_index = 0;
	int upper_index = curve.num_points - 1;

	if (current_mA >= curve.mA_values[upper_index]) {
		return curve.rel_flux_values[upper_index];
	} else if (current_mA <= curve.mA_values[0]) {
		return curve.rel_flux_values[0];
	} else {
		int test_index;
		uint32_t x_n;
		uint32_t x_1;
		uint32_t y_1;
		uint32_t x_2;
		uint32_t y_2;

		/* binary search for the indexes above and below current_mA */
		while ((upper_index - lower_index) > 1) {
			test_index = lower_index +
					((upper_index - lower_index) / 2);

			if (current_mA == curve.mA_values[test_index]) {
				/* no interpolation needed, found a match */
				return curve.rel_flux_values[test_index];
			} else if (current_mA < curve.mA_values[test_index]) {
				upper_index = test_index;
			} else {
				lower_index = test_index;
			}
		}

		/* Linear interpolation: y_n = slope * (x_n - x_1) + y_1 */
		x_n = current_mA;
		x_1 = curve.mA_values[lower_index];
		y_1 = curve.rel_flux_values[lower_index];
		x_2 = curve.mA_values[upper_index];
		y_2 = curve.rel_flux_values[upper_index];

		return (((y_2 - y_1) * (x_n - x_1)) / (x_2 - x_1)) + y_1;
	}
}

static void bbry_flash_calc_relative_flux(struct bbry_flash_ctrl_t *fctrl,
					struct msm_flash_cfg_data_t *cfg)
{
	cfg->preflash_limits.max_current =
		bbry_flash_get_current_limit(fctrl, CFG_FLASH_LOW);
	cfg->flash_limits.max_current =
		bbry_flash_get_current_limit(fctrl, CFG_FLASH_HIGH);

	if (fctrl->flux_curve.num_points != 0) {
		uint32_t preflash_relative_flux;
		uint32_t full_preflash_relative_flux;

		cfg->flash_limits.relative_flux = bbry_flash_interpolate_flux(
						fctrl->flux_curve,
						cfg->flash_limits.max_current);

		/* preflash flux is relative to the full preflash value */
		full_preflash_relative_flux = bbry_flash_interpolate_flux(
						fctrl->flux_curve,
						fctrl->low_max_current);
		if (full_preflash_relative_flux != 0) {
			preflash_relative_flux = bbry_flash_interpolate_flux(
					fctrl->flux_curve,
					cfg->preflash_limits.max_current);
			cfg->preflash_limits.relative_flux =
						bbry_flash_fp_divide(
						preflash_relative_flux,
						full_preflash_relative_flux);
		} else {
			cfg->preflash_limits.relative_flux = 0;
		}
	} else {
		/* If no curve is provided assume flux is linear */
		cfg->flash_limits.relative_flux =
					bbry_flash_fp_divide(
					cfg->flash_limits.max_current,
					fctrl->high_max_current);
		cfg->preflash_limits.relative_flux =
					bbry_flash_fp_divide(
					cfg->preflash_limits.max_current,
					fctrl->low_max_current);
	}
}

static int32_t bbry_flash_pre_process_event(struct bbry_flash_ctrl_t *fctrl,
					struct msm_flash_cfg_data_t *cfg,
					struct bbry_flash_cmd_t *cmds)
{
	switch (cfg->cfg_type) {
	case CFG_FLASH_INIT:
		fctrl->use_fixed_ratio = cfg->use_fixed_ratio;
		bbry_flash_gpio_init(fctrl);
		bbry_flash_calc_relative_flux(fctrl, cfg);
		break;
	case CFG_FLASH_RELEASE:
		bbry_flash_gpio_release(fctrl);
		break;
	case CFG_FLASH_OFF:
		bbry_flash_gpio_trigger_low(fctrl);
		break;
	case CFG_TORCH_ON:
		bbry_flash_cmds_calc_currents(fctrl, cmds, cfg->cfg_type);
		break;
	case CFG_FLASH_LOW:
		bbry_flash_cmds_calc_currents(fctrl, cmds, cfg->cfg_type);
		break;
	case CFG_FLASH_HIGH:
		bbry_flash_cmds_calc_currents(fctrl, cmds, cfg->cfg_type);
		break;
	case CFG_FLASH_MITIGATION_LEVELS:
		bbry_flash_set_mitigation_level(fctrl, cfg);
		bbry_flash_calc_relative_flux(fctrl, cfg);
		break;
	case CFG_FLASH_HW_DATA:
		cfg->hw_data = fctrl->hw_data;
		break;
	case CFG_FLASH_STATE:
		cfg->state = fctrl->state;
		break;
	case CFG_FLASH_KEEP_ALIVE:
		fctrl->keep_alive = true;
		pr_info("Keep alive\n");
		break;
	default:
		break;
	}

	return 0;
}

static int32_t bbry_flash_post_process_event(struct bbry_flash_ctrl_t *fctrl,
					struct msm_flash_cfg_data_t *cfg,
					struct bbry_flash_cmd_t *cmds)
{
	int i;
	int update_flash_state;

	switch (cfg->cfg_type) {
	case CFG_FLASH_INIT:
		update_flash_state = 1;
		break;
	case CFG_FLASH_RELEASE:
		update_flash_state = 1;
		break;
	case CFG_FLASH_OFF:
		update_flash_state = 1;
		break;
	case CFG_TORCH_ON:
		update_flash_state = 1;
		break;
	case CFG_FLASH_LOW:
		update_flash_state = 1;
		break;
	case CFG_FLASH_HIGH:
		update_flash_state = 1;
		bbry_flash_gpio_trigger_high(fctrl);
		break;
	case CFG_FLASH_MITIGATION_LEVELS:
		update_flash_state = 0;
		bbry_flash_apply_mitigation(fctrl, cfg);
		break;
	default:
		update_flash_state = 0;
		break;
	}

	if (update_flash_state) {
		fctrl->state = cfg->cfg_type;
		for (i = 0; i < fctrl->hw_data.flash_count; i++) {
			if (fctrl->flash[i].type == INVALID_FLASH)
				continue;

			fctrl->flash[i].state = cmds[i];
		}
	}

	for (i = 0; i < MAX_LED_TRIGGERS; i++) {
		if ((i >= fctrl->hw_data.flash_count) ||
			(fctrl->flash[i].type == INVALID_FLASH)) {
			cfg->flash_current[i] = 0;
			cfg->flash_duration[i] = 0;
		}
	}
	return 0;
}

static __always_inline bool bbry_flash_is_cmd_nop(
					struct bbry_flash_device_t *flash,
					struct bbry_flash_cmd_t *cmd)
{
	return (flash->state.cfg_type == cmd->cfg_type) &&
		(flash->state.limited_current == cmd->limited_current);
}

static bool bbry_flash_is_cmd_allowed(struct bbry_flash_ctrl_t *fctrl,
					struct msm_flash_cfg_data_t *cfg)
{
	if (fctrl->state == CFG_FLASH_RELEASE) {
		if ((cfg->cfg_type != CFG_FLASH_INIT) &&
			(cfg->cfg_type != CFG_FLASH_MITIGATION_LEVELS) &&
			(cfg->cfg_type != CFG_FLASH_HW_DATA) &&
			(cfg->cfg_type != CFG_FLASH_STATE) &&
			(cfg->cfg_type != CFG_FLASH_KEEP_ALIVE)) {
			pr_err(
				"cfg_type %d not allowed until flash has been initialized\n",
				cfg->cfg_type);
			return false;
		}
	}

	return true;
}

static int32_t bbry_flash_config(struct bbry_flash_ctrl_t *fctrl, void *data)
{
	int rc = 0;
	int overall_rc = 0;
	int i;
	struct msm_flash_cfg_data_t *cfg = (struct msm_flash_cfg_data_t *)data;
	struct bbry_flash_device_t *flash;
	struct bbry_flash_cmd_t cmds[MAX_LED_TRIGGERS];

	pr_debug("%s Enter\n", __func__);

	if (!bbry_flash_is_cmd_allowed(fctrl, cfg))
		return -EINVAL;

	bbry_flash_cmds_init(fctrl, cfg, cmds);
	bbry_flash_pre_process_event(fctrl, cfg, cmds);

	for (i = 0; i < fctrl->hw_data.flash_count; i++) {
		if (fctrl->flash[i].type == INVALID_FLASH)
			continue;

		flash = &fctrl->flash[i];

		/* if the flash is already in the requested state, do nothing */
		if (bbry_flash_is_cmd_nop(flash, &cmds[i])) {
			cmds[i].applied_current = flash->state.applied_current;
			cfg->flash_current[i] = cmds[i].applied_current;
			cfg->flash_duration[i] = cmds[i].flash_duration;
			pr_info("Flash %d nop\n", i);
			continue;
		}

		switch (cmds[i].cfg_type) {
		case CFG_FLASH_INIT:
			pr_info("Flash %d request: init\n", i);
			rc = flash->func_tbl.flash_init(flash, &cmds[i]);
			break;
		case CFG_FLASH_RELEASE:
			pr_info("Flash %d request: release\n", i);
			rc = flash->func_tbl.flash_release(flash, &cmds[i]);
			break;
		case CFG_FLASH_OFF:
			pr_info("Flash %d request: off\n", i);
			rc = flash->func_tbl.flash_off(flash, &cmds[i]);
			if (rc == 0)
				pr_info("Flash %d set: 0 mA\n", i);
			break;
		case CFG_TORCH_ON:
			pr_info("Flash %d request: torch_on %d mA\n",
				i, cmds[i].limited_current);
			rc = flash->func_tbl.torch_on(flash, &cmds[i]);
			if (rc == 0) {
				pr_info("Flash %d set: %d mA\n",
					i, cmds[i].applied_current);
			}
			break;
		case CFG_FLASH_LOW:
			pr_info("Flash %d request: flash_low %d mA\n",
				i, cmds[i].limited_current);
			rc = flash->func_tbl.flash_low(flash, &cmds[i]);
			if (rc == 0) {
				pr_info("Flash %d set: %d mA\n",
					i, cmds[i].applied_current);
			}
			break;
		case CFG_FLASH_HIGH:
			pr_info("Flash %d request: flash_high %d mA\n",
				i, cmds[i].limited_current);
			rc = flash->func_tbl.flash_high(flash, &cmds[i]);
			if (rc == 0) {
				pr_info("Flash %d set: %d mA\n",
					i, cmds[i].applied_current);
			}
			break;
#ifdef CONFIG_BBRY_MFG
		case CFG_FLASH_STATUS:
			pr_info("Flash %d request: flash_status\n", i);
			rc = flash->func_tbl.flash_status(flash, &cmds[i]);
			break;
#endif
		case CFG_FLASH_MITIGATION_LEVELS:
		case CFG_FLASH_HW_DATA:
		case CFG_FLASH_STATE:
		case CFG_FLASH_KEEP_ALIVE:
			/* Nothing to be done by each flash subdriver */
			rc = 0;
			break;
		default:
			rc = -EFAULT;
			break;
		}

		if (rc != 0) {
			overall_rc = rc;
		} else {
			cfg->flash_current[i] = cmds[i].applied_current;
			cfg->flash_duration[i] = cmds[i].flash_duration;
		}
	}

	bbry_flash_post_process_event(fctrl, cfg, cmds);

	pr_debug("%s: return %d\n", __func__, overall_rc);

	return overall_rc;
}

static long bbry_flash_subdev_ioctl(struct v4l2_subdev *sd,
				unsigned int cmd, void *arg)
{
	struct bbry_flash_ctrl_t *fctrl = NULL;
	void __user *argp = (void __user *)arg;

	pr_debug("%s Enter\n", __func__);

	if (!sd) {
		pr_err("%s: Bad param: sd=%p\n", __func__, sd);
		return -EINVAL;
	}

	if (!arg && (cmd != MSM_SD_SHUTDOWN)) {
		pr_err("%s: Bad param: cmd=%d, arg=NULL\n", __func__, cmd);
		return -EINVAL;
	}

	fctrl = v4l2_get_subdevdata(sd);
	if (!fctrl) {
		pr_err("%s: fctrl NULL\n", __func__);
		return -EINVAL;
	}

#ifdef CONFIG_COMPAT
	/* Support 32 bit clients */
	if (is_compat_task()) {
		if (cmd == VIDIOC_MSM_FLASH_CFG32)
			cmd = VIDIOC_MSM_FLASH_CFG;
	}
#endif /* CONFIG_COMPAT */

	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return bbry_flash_get_subdev_id(fctrl, argp);
	case VIDIOC_MSM_FLASH_CFG:
		return bbry_flash_config(fctrl, argp);
	case MSM_SD_SHUTDOWN:
		if (fctrl->state != CFG_FLASH_RELEASE) {
			struct msm_flash_cfg_data_t cfg;

			cfg.cfg_type = CFG_FLASH_RELEASE;
			return bbry_flash_config(fctrl, &cfg);
		}
		break;
	default:
		pr_err("Invalid cmd %d\n", cmd);
		return -ENOIOCTLCMD;
	}

	return 0;
}

static int bbry_flash_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct bbry_flash_ctrl_t *fctrl;

#ifndef CONFIG_BBRY_MFG
	if (1 != v4l2_fh_is_singular(&fh->vfh)) {
		pr_info("Subdev open rejected: only one client allowed\n");
		return -EBUSY;
	}
#endif /* CONFIG_BBRY_MFG */

	fctrl = v4l2_get_subdevdata(sd);
	if (fctrl != NULL)
		fctrl->keep_alive = false;

	return 0;
}

static int bbry_flash_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	if (1 == v4l2_fh_is_singular(&fh->vfh)) {
		struct bbry_flash_ctrl_t *fctrl;

		fctrl = v4l2_get_subdevdata(sd);
		if ((fctrl != NULL) && (fctrl->keep_alive))
			return 0;

		/* When the last client disconnects, shut down the hardware */
		return bbry_flash_subdev_ioctl(sd, MSM_SD_SHUTDOWN, NULL);
	}
	return 0;
}

static struct v4l2_subdev_core_ops bbry_flash_subdev_core_ops = {
	.ioctl = bbry_flash_subdev_ioctl,
};

static struct v4l2_subdev_ops bbry_flash_subdev_ops = {
	.core = &bbry_flash_subdev_core_ops,
};

static int32_t bbry_flash_create_v4lsubdev(struct platform_device *pdev,
				struct bbry_flash_ctrl_t *fctrl)
{
	pr_debug("%s Enter\n", __func__);

	if (!fctrl) {
		pr_err("fctrl NULL\n");
		return -EINVAL;
	}

	/* Initialize sub device */
	v4l2_subdev_init(&fctrl->msm_sd.sd, &bbry_flash_subdev_ops);
	v4l2_set_subdevdata(&fctrl->msm_sd.sd, fctrl);

	fctrl->pdev = pdev;
	fctrl->msm_sd.sd.internal_ops = &bbry_flash_internal_ops;
	fctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(fctrl->msm_sd.sd.name, ARRAY_SIZE(fctrl->msm_sd.sd.name),
		"bbry_flash");
	media_entity_init(&fctrl->msm_sd.sd.entity, 0, NULL, 0);
	fctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	fctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_FLASH;
	fctrl->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x1;
	msm_sd_register(&fctrl->msm_sd);

	bbry_flash_v4l2_subdev_fops = v4l2_subdev_fops;
#ifdef CONFIG_COMPAT
	bbry_flash_v4l2_subdev_fops.compat_ioctl32 =
		bbry_flash_v4l2_subdev_fops.unlocked_ioctl;
#endif
	fctrl->msm_sd.sd.devnode->fops = &bbry_flash_v4l2_subdev_fops;

	pr_debug("Flash probe success\n");
	return 0;
}

static int32_t bbry_flash_dt_load_array(struct device_node *of_node,
					const char *property_name,
					uint32_t **array)
{
	int32_t rc;
	int32_t array_size;
	int32_t num_entries;

	array_size = 0;
	if (NULL == of_get_property(of_node, property_name, &array_size)) {
		pr_err("%s missing\n", property_name);
		return -EINVAL;
	}

	if (array_size == 0) {
		pr_err("%s empty\n", property_name);
		return -EINVAL;
	}

	*array = kmalloc(array_size, GFP_KERNEL);
	if (*array == NULL) {
		pr_err("Error allocating %s array\n", property_name);
		return -ENOMEM;
	}

	num_entries = array_size / sizeof(uint32_t);
	rc = of_property_read_u32_array(of_node, property_name, *array,
					num_entries);
	if (rc < 0) {
		pr_err("Reading %s array failed\n", property_name);
		kfree(*array);
		*array = NULL;
		return rc;
	}

	return num_entries;
}

static int32_t bbry_flash_dt_parse_mitigation_levels(
				struct device_node *of_node,
				const char *property_name,
				struct bbry_flash_mitigation_t *mitigation)
{
	mitigation->num_levels = bbry_flash_dt_load_array(of_node,
					property_name, &mitigation->levels);

	if (mitigation->num_levels <= 0) {
		mitigation->num_levels = 0;
		return -EINVAL;
	}

	mitigation->limit = mitigation->levels[0];
	return 0;
}

static int32_t bbry_flash_dt_parse_flux_curve(struct device_node *of_node,
					struct bbry_flux_curve_t *flux_curve)
{
	int i;
	int num_array_entries;

	flux_curve->mA_values = NULL;
	flux_curve->rel_flux_values = NULL;
	num_array_entries = bbry_flash_dt_load_array(of_node, "curve_mA_values",
						&flux_curve->mA_values);
	if (num_array_entries <= 0)
		goto error;

	for (i = 0; i < num_array_entries - 1; i++) {
		if (flux_curve->mA_values[i] >= flux_curve->mA_values[i + 1]) {
			pr_err("flux curve is not sorted in ascending order\n");
			goto error;
		}
	}

	if (num_array_entries != bbry_flash_dt_load_array(of_node,
						"curve_rel_flux_values",
						&flux_curve->rel_flux_values)) {
		pr_err("flux curve array sizes do not match\n");
		goto error;
	}

	flux_curve->num_points = num_array_entries;
	return 0;

error:
	if (flux_curve->rel_flux_values != NULL) {
		kfree(flux_curve->rel_flux_values);
		flux_curve->rel_flux_values = NULL;
	}

	if (flux_curve->mA_values != NULL) {
		kfree(flux_curve->mA_values);
		flux_curve->mA_values = NULL;
	}

	flux_curve->num_points = 0;
	return -EINVAL;
}

static int32_t bbry_flash_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct device_node *of_node = pdev->dev.of_node;
	struct device_node *flash_src_node = NULL;
	struct device_node *torch_src_node = NULL;
	int flash_count;
	int torch_count;
	int i;
	int j;
	const char *flash_type;
	const char *torch_type;
	int num_valid_flashes = 0;

	pr_debug("%s Enter\n", __func__);

	/* Ensure the driver name strings are null terminated */
	for (i = 0; i < ARRAY_SIZE(supported_drivers); i++)
		supported_drivers[i].name[MAX_DRIVER_NAME_LENGTH - 1] = '\0';

	flash_ctrl.pdev = pdev;

	rc = of_property_read_u32(of_node, "cell-index", &pdev->id);
	if (rc < 0) {
		pr_err("cell-index missing\n");
		return -EINVAL;
	}
	pr_debug("pdev id %d\n", pdev->id);

	rc = of_property_read_u32(of_node, "high-max-current",
				&flash_ctrl.high_max_current);
	if ((rc < 0) || (flash_ctrl.high_max_current == 0)) {
		pr_err("high-max-current missing or 0\n");
		return -EINVAL;
	}
	pr_info("flash_high max current %d mA\n", flash_ctrl.high_max_current);

	rc = of_property_read_u32(of_node, "low-max-current",
				&flash_ctrl.low_max_current);
	if ((rc < 0) || (flash_ctrl.low_max_current == 0)) {
		pr_err("low-max-current missing or 0\n");
		return -EINVAL;
	}
	pr_info("flash_low max current %d mA\n", flash_ctrl.low_max_current);

	rc = of_property_read_u32(of_node, "torch-max-current",
				&flash_ctrl.torch_max_current);
	if ((rc < 0) || (flash_ctrl.torch_max_current == 0)) {
		pr_err("torch-max-current missing or 0\n");
		return -EINVAL;
	}
	pr_info("torch_on max current %d mA\n", flash_ctrl.torch_max_current);

	rc = of_property_read_u32(of_node, "preflash-divisor",
				&flash_ctrl.preflash_divisor);
	if ((rc < 0) || (flash_ctrl.preflash_divisor == 0)) {
		flash_ctrl.preflash_divisor = 1;
	}
	pr_info("preflash-divisor %d\n", flash_ctrl.preflash_divisor);

	/* Read the gpio information from device tree */
	rc = bbry_flash_gpio_get_gpio_dt_data(of_node, &flash_ctrl);
	if (rc < 0) {
		pr_err("%s:%d bbry_flash_get_gpio_dt_data failed rc %d\n",
			__func__, __LINE__, rc);
		return rc;
	}

	if (NULL == of_get_property(of_node, "flash-source",
				&flash_count)) {
		pr_err("flash-source missing\n");
		return -EINVAL;
	}

	flash_count /= sizeof(uint32_t);
	pr_debug("flash count %d\n", flash_count);

	if (NULL == of_get_property(of_node, "torch-source",
				&torch_count)) {
		pr_err("torch-source missing\n");
		return -EINVAL;
	}

	torch_count /= sizeof(uint32_t);
	pr_debug("torch count %d\n", torch_count);

	if (flash_count != torch_count) {
		pr_err("Flash and torch counts do not match\n");
		return -EINVAL;
	}

	if (flash_count > MAX_LED_TRIGGERS) {
		pr_err("flash-source failed: too many sources\n");
		return -EINVAL;
	}

	flash_ctrl.hw_data.flash_count = (uint32_t) flash_count;
	for (i = 0; i < flash_count; i++) {
		flash_ctrl.flash[i].type = INVALID_FLASH;
		flash_ctrl.flash[i].state.cfg_type = CFG_FLASH_RELEASE;

		flash_src_node = of_parse_phandle(of_node, "flash-source",
							i);
		if (NULL == flash_src_node) {
			pr_err("flash-source %d NULL\n", i);
			continue;
		}

		rc = of_property_read_string(flash_src_node, "flash-type",
				&flash_type);
		if (rc < 0) {
			pr_err("flash-type failed\n");
			of_node_put(flash_src_node);
			continue;
		}

		torch_src_node = of_parse_phandle(of_node, "torch-source",
							i);
		if (NULL == torch_src_node) {
			pr_err("torch-source %d NULL\n", i);
			of_node_put(flash_src_node);
			continue;
		}

		rc = of_property_read_string(torch_src_node, "flash-type",
				&torch_type);
		if (rc < 0) {
			pr_err("flash-type failed\n");
			of_node_put(flash_src_node);
			of_node_put(torch_src_node);
			continue;
		}

		if (0 != strcmp(flash_type, torch_type)) {
			pr_err("Flash and torch types do not match\n");
			of_node_put(flash_src_node);
			of_node_put(torch_src_node);
			continue;
		}

		/* Search the list of supported flash drivers for a match and
		 * call that driver's probe function
		 */
		for (j = 0; j < ARRAY_SIZE(supported_drivers); j++) {
			if (0 == strcmp(flash_type,
					supported_drivers[j].name)) {
				if (0 == supported_drivers[j].probe(&flash_ctrl,
							flash_src_node,
							torch_src_node,
							&flash_ctrl.flash[i])) {
					num_valid_flashes++;
				}
				break;
			}
		}

		if (j >= ARRAY_SIZE(supported_drivers)) {
			pr_err("flash-type %s not supported\n", flash_type);
			of_node_put(flash_src_node);
			of_node_put(torch_src_node);
			continue;
		}

		if (of_property_read_bool(flash_src_node, "use-for-videolight"))
			flash_ctrl.hw_data.videolight_flashes |= (1 << i);

		if (of_property_read_bool(flash_src_node,
					"use-for-capture-timer"))
			flash_ctrl.hw_data.capture_timer_flashes |= (1 << i);

		of_node_put(flash_src_node);
		of_node_put(torch_src_node);
	}

	pr_info("Reported %d flashes, found %d valid flashes\n",
		flash_count, num_valid_flashes);

	if (num_valid_flashes == 0) {
		pr_err("No valid flashes found\n");
		return -EINVAL;
	}

	pr_info("Videolight flashes = 0x%08X\n",
		flash_ctrl.hw_data.videolight_flashes);

	pr_info("Capture timer flashes = 0x%08X\n",
		flash_ctrl.hw_data.capture_timer_flashes);

	rc = bbry_flash_dt_parse_mitigation_levels(of_node,
					"flash-mitigation-levels",
					&flash_ctrl.flash_mitigation);
	if (rc < 0)
		return rc;

	pr_info("Flash mitigation levels %d\n",
		flash_ctrl.flash_mitigation.num_levels);

	rc = bbry_flash_dt_parse_mitigation_levels(of_node,
					"torch-mitigation-levels",
					&flash_ctrl.torch_mitigation);
	if (rc < 0) {
		kfree(flash_ctrl.flash_mitigation.levels);
		return rc;
	}
	pr_info("Torch mitigation levels %d\n",
		flash_ctrl.torch_mitigation.num_levels);

	rc = bbry_flash_dt_parse_flux_curve(of_node, &flash_ctrl.flux_curve);
	if (rc < 0) {
		pr_info("Valid flux curve not found\n");
	} else {
		pr_info("Flux curve points %d\n",
			flash_ctrl.flux_curve.num_points);
	}

	flash_ctrl.state = CFG_FLASH_RELEASE;

	rc = bbry_flash_create_v4lsubdev(pdev, &flash_ctrl);
	if (rc < 0) {
		pr_err("Creating V4L2 flash subdev failed\n");
		kfree(flash_ctrl.torch_mitigation.levels);
		kfree(flash_ctrl.flash_mitigation.levels);
	}

	return rc;
}

static int __init bbry_flash_add_driver(void)
{
	pr_debug("%s Enter\n", __func__);
	return platform_driver_register(&bbry_flash_driver);
}

MODULE_DEVICE_TABLE(of, bbry_flash_dt_match);

module_init(bbry_flash_add_driver);
MODULE_AUTHOR("BlackBerry Limited");
MODULE_DESCRIPTION("Multi-flash camera flash driver");
MODULE_LICENSE("GPL v2");
