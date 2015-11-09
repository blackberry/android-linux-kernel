/* Copyright (c) 2015 BlackBerry Limited
 * Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/qpnp/pwm.h>
#include <linux/err.h>
#include <linux/string.h>

#include "mdss_dsi.h"
#ifdef CONFIG_BBRY
#include "mdss_mdp.h"
/* For reading nvram file */
#include <linux/fs.h>
#include <asm/sections.h>
#include <asm/uaccess.h>

/* For sending ddt report */
#include <linux/version.h>
#include <misc/ddt.h>
#include <misc/lw_event_types.h>

void mdss_dsi_read_serial_id(struct mdss_panel_info *pinfo);
#endif

#define DT_CMD_HDR 6
#define MIN_REFRESH_RATE 30
#define DEFAULT_MDP_TRANSFER_TIME 14000

DEFINE_LED_TRIGGER(bl_led_trigger);

void mdss_dsi_panel_pwm_cfg(struct mdss_dsi_ctrl_pdata *ctrl)
{
	if (ctrl->pwm_pmi)
		return;

	ctrl->pwm_bl = pwm_request(ctrl->pwm_lpg_chan, "lcd-bklt");
	if (ctrl->pwm_bl == NULL || IS_ERR(ctrl->pwm_bl)) {
		pr_err("%s: Error: lpg_chan=%d pwm request failed",
				__func__, ctrl->pwm_lpg_chan);
	}
	ctrl->pwm_enabled = 0;
}

static void mdss_dsi_panel_bklt_pwm(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	int ret;
	u32 duty;
	u32 period_ns;

	if (ctrl->pwm_bl == NULL) {
		pr_err("%s: no PWM\n", __func__);
		return;
	}

	if (level == 0) {
		if (ctrl->pwm_enabled) {
			ret = pwm_config_us(ctrl->pwm_bl, level,
					ctrl->pwm_period);
			if (ret)
				pr_err("%s: pwm_config_us() failed err=%d.\n",
						__func__, ret);
			pwm_disable(ctrl->pwm_bl);
		}
		ctrl->pwm_enabled = 0;
		return;
	}

	duty = level * ctrl->pwm_period;
	duty /= ctrl->bklt_max;

	pr_debug("%s: bklt_ctrl=%d pwm_period=%d pwm_gpio=%d pwm_lpg_chan=%d\n",
			__func__, ctrl->bklt_ctrl, ctrl->pwm_period,
				ctrl->pwm_pmic_gpio, ctrl->pwm_lpg_chan);

	pr_debug("%s: ndx=%d level=%d duty=%d\n", __func__,
					ctrl->ndx, level, duty);

	if (ctrl->pwm_period >= USEC_PER_SEC) {
		ret = pwm_config_us(ctrl->pwm_bl, duty, ctrl->pwm_period);
		if (ret) {
			pr_err("%s: pwm_config_us() failed err=%d.\n",
					__func__, ret);
			return;
		}
	} else {
		period_ns = ctrl->pwm_period * NSEC_PER_USEC;
		ret = pwm_config(ctrl->pwm_bl,
				level * period_ns / ctrl->bklt_max,
				period_ns);
		if (ret) {
			pr_err("%s: pwm_config() failed err=%d.\n",
					__func__, ret);
			return;
		}
	}

	if (!ctrl->pwm_enabled) {
		ret = pwm_enable(ctrl->pwm_bl);
		if (ret)
			pr_err("%s: pwm_enable() failed err=%d\n", __func__,
				ret);
		ctrl->pwm_enabled = 1;
	}
}

static char dcs_cmd[2] = {0x54, 0x00}; /* DTYPE_DCS_READ */
static struct dsi_cmd_desc dcs_read_cmd = {
	{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(dcs_cmd)},
	dcs_cmd
};

u32 mdss_dsi_panel_cmd_read(struct mdss_dsi_ctrl_pdata *ctrl, char cmd0,
		char cmd1, void (*fxn)(int), char *rbuf, int len)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo;

	pinfo = &(ctrl->panel_data.panel_info);
	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			return -EINVAL;
	}

	dcs_cmd[0] = cmd0;
	dcs_cmd[1] = cmd1;
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dcs_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = len;
	cmdreq.rbuf = rbuf;
	cmdreq.cb = fxn; /* call back */
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	/*
	 * blocked here, until call back called
	 */

	return 0;
}

#ifndef CONFIG_BBRY
static void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds, u32 flags)
#else
static int mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds, u32 flags)
#endif
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo;

	pinfo = &(ctrl->panel_data.panel_info);
	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
#ifndef CONFIG_BBRY
			return;
#else
			return -EINVAL;
#endif
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = flags;

	/*Panel ON/Off commands should be sent in DSI Low Power Mode*/
	if (pcmds->link_state == DSI_LP_MODE)
		cmdreq.flags  |= CMD_REQ_LP_MODE;
	else if (pcmds->link_state == DSI_HS_MODE)
		cmdreq.flags |= CMD_REQ_HS_MODE;

	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

#ifndef CONFIG_BBRY
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
#else
	return mdss_dsi_cmdlist_put(ctrl, &cmdreq);
#endif
}

static char led_pwm1[2] = {0x51, 0x0};	/* DTYPE_DCS_WRITE1 */
static struct dsi_cmd_desc backlight_cmd = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1, sizeof(led_pwm1)},
	led_pwm1
};
#ifdef CONFIG_BBRY
static char alpm_brightness_bb_values[6][4] = {
	{0xBB, 0x4D, 0x42, 0x5C},       /* 10 nits */
	{0xBB, 0x5B, 0x4F, 0x6B},
	{0xBB, 0x64, 0x57, 0x74},
	{0xBB, 0x6A, 0x5D, 0x7B},
	{0xBB, 0x70, 0x62, 0x81},
	{0xBB, 0x75, 0x67, 0x87},       /* 60 nits */
};
#define NUM_ALPM_BRIGHTNESS_BB_VALUES (sizeof(alpm_brightness_bb_values)/sizeof(alpm_brightness_bb_values[0]))

static struct dcs_cmd_req alpm_brightness_cmd_req = {
	.cmds = (struct dsi_cmd_desc []) {
		{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 3}, (char []) {0xf0, 0x5a, 0x5a}},
		{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 2}, (char []) {0xb0, 0x05}},
		{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 4}, NULL}, /* payload will be set to one of alpm_brightness_bb_values[] */
		{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, 2}, (char []) {0xf7, 0x03}},
		{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 2}, (char []) {0xf7, 0x00}},
		{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, 3}, (char []) {0xf0, 0xa5, 0xa5}}
	},
	.cmds_cnt = 6,
	.flags = CMD_REQ_COMMIT | CMD_REQ_HS_MODE
};
#endif

#ifdef CONFIG_BBRY
static void oled_panel_set_brightness(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	return;
}
#endif

static void mdss_dsi_panel_bklt_dcs(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo;

#ifdef CONFIG_BBRY
	/* BBRY Platforms with OLED panels will go through the BBRY
	*  oled-simulation code to change the brightness of the OLED panel.
	*/
	oled_panel_set_brightness(ctrl, level);
	return;
#endif

	pinfo = &(ctrl->panel_data.panel_info);
	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			return;
	}

	pr_debug("%s: level=%d\n", __func__, level);

#ifdef CONFIG_BBRY
	if (pinfo->blank_state == MDSS_PANEL_BLANK_LOW_POWER) {
		int i;

		/* Logan ALPM mode has a different backlighting mechanism. Choose one of 6 discrete brightnesses */
		for (i = 0; i < NUM_ALPM_BRIGHTNESS_BB_VALUES-1; i++) {
			int level_nits = level * 600 / 255;
			if (level_nits < (15 + 10 * i))
				break;
		}

		pr_info("%s: alpm brightness level=%d nits\n", __func__, (i+1)*10);
		alpm_brightness_cmd_req.cmds[2].payload = alpm_brightness_bb_values[i];
		mdss_dsi_cmdlist_put(ctrl, &alpm_brightness_cmd_req);
		return;
	}

	/* Read the Serial ID before attempting to set the backlight. We want to catch
	 * The case when we have an NV-corrupt Logan, so we can use recovery values
	 * for the DSI registers which the gamma calculator reads.
	*/
	if (pinfo->read_serial_id_bytes && pinfo->serial_id_length && !pinfo->serial_id)
		mdss_dsi_read_serial_id(&(ctrl->panel_data.panel_info));

	/* BBRY Platforms with OLED panels will go through the BBRY
	*  oled-simulation code to change the brightness of the OLED panel.
	*/
	oled_panel_set_brightness(ctrl, level);
	return;
#endif

	led_pwm1[1] = (unsigned char)level;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &backlight_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static int mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;

	if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		rc = gpio_request(ctrl_pdata->disp_en_gpio,
						"disp_enable");
		if (rc) {
			pr_err("request disp_en gpio failed, rc=%d\n",
				       rc);
			goto disp_en_gpio_err;
		}
	}
	rc = gpio_request(ctrl_pdata->rst_gpio, "disp_rst_n");
	if (rc) {
		pr_err("request reset gpio failed, rc=%d\n",
			rc);
		goto rst_gpio_err;
	}
	if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
		rc = gpio_request(ctrl_pdata->bklt_en_gpio,
						"bklt_enable");
		if (rc) {
			pr_err("request bklt gpio failed, rc=%d\n",
				       rc);
			goto bklt_en_gpio_err;
		}
	}
	if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
		rc = gpio_request(ctrl_pdata->mode_gpio, "panel_mode");
		if (rc) {
			pr_err("request panel mode gpio failed,rc=%d\n",
								rc);
			goto mode_gpio_err;
		}
	}
	return rc;

mode_gpio_err:
	if (gpio_is_valid(ctrl_pdata->bklt_en_gpio))
		gpio_free(ctrl_pdata->bklt_en_gpio);
bklt_en_gpio_err:
	gpio_free(ctrl_pdata->rst_gpio);
rst_gpio_err:
	if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
		gpio_free(ctrl_pdata->disp_en_gpio);
disp_en_gpio_err:
	return rc;
}

int mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int i, rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}

	pr_debug("%s: enable = %d\n", __func__, enable);
	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (enable) {
		rc = mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}
		if (!pinfo->cont_splash_enabled) {
			if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
				gpio_set_value((ctrl_pdata->disp_en_gpio), 1);

			for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
				gpio_set_value((ctrl_pdata->rst_gpio),
					pdata->panel_info.rst_seq[i]);
				if (pdata->panel_info.rst_seq[++i])
					usleep(pinfo->rst_seq[i] * 1000);
			}

			if (gpio_is_valid(ctrl_pdata->bklt_en_gpio))
				gpio_set_value((ctrl_pdata->bklt_en_gpio), 1);
		}

		if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
			if (pinfo->mode_gpio_state == MODE_GPIO_HIGH)
				gpio_set_value((ctrl_pdata->mode_gpio), 1);
			else if (pinfo->mode_gpio_state == MODE_GPIO_LOW)
				gpio_set_value((ctrl_pdata->mode_gpio), 0);
		}
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
			gpio_set_value((ctrl_pdata->bklt_en_gpio), 0);
			gpio_free(ctrl_pdata->bklt_en_gpio);
		}
		if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
			gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
			gpio_free(ctrl_pdata->disp_en_gpio);
		}
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
		gpio_free(ctrl_pdata->rst_gpio);
		if (gpio_is_valid(ctrl_pdata->mode_gpio))
			gpio_free(ctrl_pdata->mode_gpio);
	}
	return rc;
}

/**
 * mdss_dsi_roi_merge() -  merge two roi into single roi
 *
 * Function used by partial update with only one dsi intf take 2A/2B
 * (column/page) dcs commands.
 */
static int mdss_dsi_roi_merge(struct mdss_dsi_ctrl_pdata *ctrl,
					struct mdss_rect *roi)
{
	struct mdss_panel_info *l_pinfo;
	struct mdss_rect *l_roi;
	struct mdss_rect *r_roi;
	struct mdss_dsi_ctrl_pdata *other = NULL;
	int ans = 0;

	if (ctrl->ndx == DSI_CTRL_LEFT) {
		other = mdss_dsi_get_ctrl_by_index(DSI_CTRL_RIGHT);
		if (!other)
			return ans;
		l_pinfo = &(ctrl->panel_data.panel_info);
		l_roi = &(ctrl->panel_data.panel_info.roi);
		r_roi = &(other->panel_data.panel_info.roi);
	} else  {
		other = mdss_dsi_get_ctrl_by_index(DSI_CTRL_LEFT);
		if (!other)
			return ans;
		l_pinfo = &(other->panel_data.panel_info);
		l_roi = &(other->panel_data.panel_info.roi);
		r_roi = &(ctrl->panel_data.panel_info.roi);
	}

	if (l_roi->w == 0 && l_roi->h == 0) {
		/* right only */
		*roi = *r_roi;
		roi->x += l_pinfo->xres;/* add left full width to x-offset */
	} else {
		/* left only and left+righ */
		*roi = *l_roi;
		roi->w +=  r_roi->w; /* add right width */
		ans = 1;
	}

	return ans;
}

static char caset[] = {0x2a, 0x00, 0x00, 0x03, 0x00};	/* DTYPE_DCS_LWRITE */
static char paset[] = {0x2b, 0x00, 0x00, 0x05, 0x00};	/* DTYPE_DCS_LWRITE */

/* pack into one frame before sent */
static struct dsi_cmd_desc set_col_page_addr_cmd[] = {
	{{DTYPE_DCS_LWRITE, 0, 0, 0, 1, sizeof(caset)}, caset},	/* packed */
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(paset)}, paset},
};

static void mdss_dsi_send_col_page_addr(struct mdss_dsi_ctrl_pdata *ctrl,
				struct mdss_rect *roi, int unicast)
{
	struct dcs_cmd_req cmdreq;

	caset[1] = (((roi->x) & 0xFF00) >> 8);
	caset[2] = (((roi->x) & 0xFF));
	caset[3] = (((roi->x - 1 + roi->w) & 0xFF00) >> 8);
	caset[4] = (((roi->x - 1 + roi->w) & 0xFF));
	set_col_page_addr_cmd[0].payload = caset;

	paset[1] = (((roi->y) & 0xFF00) >> 8);
	paset[2] = (((roi->y) & 0xFF));
	paset[3] = (((roi->y - 1 + roi->h) & 0xFF00) >> 8);
	paset[4] = (((roi->y - 1 + roi->h) & 0xFF));
	set_col_page_addr_cmd[1].payload = paset;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds_cnt = 2;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	if (unicast)
		cmdreq.flags |= CMD_REQ_UNICAST;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	cmdreq.cmds = set_col_page_addr_cmd;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static int mdss_dsi_set_col_page_addr(struct mdss_panel_data *pdata,
		bool force_send)
{
	struct mdss_panel_info *pinfo;
	struct mdss_rect roi = {0};
	struct mdss_rect *p_roi;
	struct mdss_rect *c_roi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_dsi_ctrl_pdata *other = NULL;
	int left_or_both = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pinfo = &pdata->panel_info;
	p_roi = &pinfo->roi;

	/*
	 * to avoid keep sending same col_page info to panel,
	 * if roi_merge enabled, the roi of left ctrl is used
	 * to compare against new merged roi and saved new
	 * merged roi to it after comparing.
	 * if roi_merge disabled, then the calling ctrl's roi
	 * and pinfo's roi are used to compare.
	 */
	if (pinfo->partial_update_roi_merge) {
		left_or_both = mdss_dsi_roi_merge(ctrl, &roi);
		other = mdss_dsi_get_ctrl_by_index(DSI_CTRL_LEFT);
		c_roi = &other->roi;
	} else {
		c_roi = &ctrl->roi;
		roi = *p_roi;
	}

	/* roi had changed, do col_page update */
	if (force_send || !mdss_rect_cmp(c_roi, &roi)) {
		pr_debug("%s: ndx=%d x=%d y=%d w=%d h=%d\n",
				__func__, ctrl->ndx, p_roi->x,
				p_roi->y, p_roi->w, p_roi->h);

		*c_roi = roi; /* keep to ctrl */
		if (c_roi->w == 0 || c_roi->h == 0) {
			/* no new frame update */
			pr_debug("%s: ctrl=%d, no partial roi set\n",
						__func__, ctrl->ndx);
			return 0;
		}

		if (pinfo->dcs_cmd_by_left) {
			if (left_or_both && ctrl->ndx == DSI_CTRL_RIGHT) {
				/* 2A/2B sent by left already */
				return 0;
			}
		}

		if (!mdss_dsi_sync_wait_enable(ctrl)) {
			if (pinfo->dcs_cmd_by_left)
				ctrl = mdss_dsi_get_ctrl_by_index(
							DSI_CTRL_LEFT);
			mdss_dsi_send_col_page_addr(ctrl, &roi, 0);
		} else {
			/*
			 * when sync_wait_broadcast enabled,
			 * need trigger at right ctrl to
			 * start both dcs cmd transmission
			 */
			other = mdss_dsi_get_other_ctrl(ctrl);
			if (!other)
				goto end;

			if (mdss_dsi_is_left_ctrl(ctrl)) {
				if (pinfo->partial_update_roi_merge) {
					/*
					 * roi is the one after merged
					 * to dsi-1 only
					 */
					mdss_dsi_send_col_page_addr(other,
							&roi, 0);
				} else {
					mdss_dsi_send_col_page_addr(ctrl,
							&ctrl->roi, 1);
					mdss_dsi_send_col_page_addr(other,
							&other->roi, 1);
				}
			} else {
				if (pinfo->partial_update_roi_merge) {
					/*
					 * roi is the one after merged
					 * to dsi-1 only
					 */
					mdss_dsi_send_col_page_addr(ctrl,
							&roi, 0);
				} else {
					mdss_dsi_send_col_page_addr(other,
							&other->roi, 1);
					mdss_dsi_send_col_page_addr(ctrl,
							&ctrl->roi, 1);
				}
			}
		}
	}

end:
	return 0;
}

static void mdss_dsi_panel_switch_mode(struct mdss_panel_data *pdata,
							int mode)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mipi_panel_info *mipi;
	struct dsi_panel_cmds *pcmds;
	u32 flags = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	mipi  = &pdata->panel_info.mipi;

	if (!mipi->dms_mode)
		return;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (mipi->dms_mode != DYNAMIC_MODE_RESOLUTION_SWITCH_IMMEDIATE) {
		if (mode == SWITCH_TO_CMD_MODE)
			pcmds = &ctrl_pdata->video2cmd;
		else
			pcmds = &ctrl_pdata->cmd2video;
	} else if ((mipi->dms_mode ==
				DYNAMIC_MODE_RESOLUTION_SWITCH_IMMEDIATE)
			&& pdata->current_timing
			&& !list_empty(&pdata->timings_list)) {
		struct dsi_panel_timing *pt;

		pt = container_of(pdata->current_timing,
				struct dsi_panel_timing, timing);

		pr_debug("%s: sending switch commands\n", __func__);
		pcmds = &pt->switch_cmds;
		flags |= CMD_REQ_DMA_TPG;
	} else {
		pr_warn("%s: Invalid mode switch attempted\n", __func__);
		return;
	}

	mdss_dsi_panel_cmds_send(ctrl_pdata, pcmds, flags);
}

static void mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata,
							u32 bl_level)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_dsi_ctrl_pdata *sctrl = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	/*
	 * Some backlight controllers specify a minimum duty cycle
	 * for the backlight brightness. If the brightness is less
	 * than it, the controller can malfunction.
	 */

	if ((bl_level < pdata->panel_info.bl_min) && (bl_level != 0))
		bl_level = pdata->panel_info.bl_min;

	switch (ctrl_pdata->bklt_ctrl) {
	case BL_WLED:
		led_trigger_event(bl_led_trigger, bl_level);
		break;
	case BL_PWM:
		mdss_dsi_panel_bklt_pwm(ctrl_pdata, bl_level);
		break;
	case BL_DCS_CMD:
		if (!mdss_dsi_sync_wait_enable(ctrl_pdata)) {
			mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);
			break;
		}
		/*
		 * DCS commands to update backlight are usually sent at
		 * the same time to both the controllers. However, if
		 * sync_wait is enabled, we need to ensure that the
		 * dcs commands are first sent to the non-trigger
		 * controller so that when the commands are triggered,
		 * both controllers receive it at the same time.
		 */
		sctrl = mdss_dsi_get_other_ctrl(ctrl_pdata);
		if (mdss_dsi_sync_wait_trigger(ctrl_pdata)) {
			if (sctrl)
				mdss_dsi_panel_bklt_dcs(sctrl, bl_level);
			mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);
		} else {
			mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);
			if (sctrl)
				mdss_dsi_panel_bklt_dcs(sctrl, bl_level);
		}
		break;
	default:
		pr_err("%s: Unknown bl_ctrl configuration\n",
			__func__);
		break;
	}
}

#ifdef CONFIG_BBRY
/* Apply Panel Colour Correction - send the register settings
 * to the MDP to be written to hardware
 */
static void mdss_apply_pcc(struct mdss_panel_data *pdata)
{
	struct	msmfb_mdp_pp pp;
	int		copy;
	int		ret;

	if (pdata->panel_info.pcc_enabled) {
		memset(&pp, 0, sizeof(pp));
		pp.op = mdp_op_pcc_cfg;
		pp.data.pcc_cfg_data.block = MDP_LOGICAL_BLOCK_DISP_0;
		pp.data.pcc_cfg_data.ops = MDP_PP_OPS_ENABLE | MDP_PP_OPS_WRITE;

		memcpy(&pp.data.pcc_cfg_data.r, &pdata->panel_info.pcc_data[0],
			   sizeof(uint32_t) * MDSS_PCC_DATA_TABLE_SIZE);

		ret = mdss_mdp_pcc_config(&pp.data.pcc_cfg_data, &copy);
		if (ret)
			pr_err("%s: failed to set PCC!\n", __func__);
		else
			pr_notice("%s: set PCC successfully.\n", __func__);
	} else
		pr_notice("%s: no pcc in panel_info\n", __func__);
}

/* Apply Gamut Map - send the register settings
 * to the MDP to be written to hardware
 */
static void mdss_apply_gamut_map(struct mdss_panel_data *pdata)
{
	struct mdp_gamut_cfg_data  gm_data = {0};
	int gamut_data_table_size[MDP_GAMUT_TABLE_NUM] =
	{125, 100, 80, 100, 100, 80, 64, 80}; /* it isn't clear where these
											 values come from */
	int ret;
	int i;
	int cur_offset = 0;
	bool alloc_failed = false;

	if (pdata->panel_info.gm_flags & MDSS_CC_FLAGS_GAMUT_DIRTY) {
		gm_data.block = MDP_LOGICAL_BLOCK_DISP_0;
		gm_data.flags = MDP_PP_OPS_ENABLE | MDP_PP_OPS_WRITE;
		gm_data.gamut_first = 0;

		for (i=0; i < MDP_GAMUT_TABLE_NUM; i++) {
			gm_data.tbl_size[i] = gamut_data_table_size[i];
			gm_data.r_tbl[i] = kzalloc(sizeof(uint16_t) * gm_data.tbl_size[i],
									   GFP_KERNEL);
			if (!gm_data.r_tbl[i]) {
				pr_err("%s: Unable to allocate r_tbl[%d]!\n", __func__, i);
				alloc_failed = true;
				break;
			}
			gm_data.g_tbl[i] = kzalloc(sizeof(uint16_t) * gm_data.tbl_size[i],
									   GFP_KERNEL);
			if (!gm_data.g_tbl[i]) {
				pr_err("%s: Unable to allocate g_tbl[%d]!\n", __func__, i);
				alloc_failed = true;
				break;
			}
			gm_data.b_tbl[i] = kzalloc(sizeof(uint16_t) * gm_data.tbl_size[i],
									   GFP_KERNEL);
			if (!gm_data.b_tbl[i]) {
				pr_err("%s: Unable to allocate b_tbl[%d]!\n", __func__, i);
				alloc_failed = true;
				break;
			}

			memcpy(gm_data.r_tbl[i], pdata->panel_info.gm_data[0] + cur_offset,
				   sizeof(uint16_t) * gm_data.tbl_size[i]);
			memcpy(gm_data.g_tbl[i], pdata->panel_info.gm_data[1] + cur_offset,
				   sizeof(uint16_t) * gm_data.tbl_size[i]);
			memcpy(gm_data.b_tbl[i], pdata->panel_info.gm_data[2] + cur_offset,
				   sizeof(uint16_t) * gm_data.tbl_size[i]);
			cur_offset += gm_data.tbl_size[i];
		}

		if (!alloc_failed) {
			ret = mdss_mdp_gamut_config(&gm_data, NULL);
			if (ret)
				pr_err("%s: failed to set gamut map, ret = %d!\n", __func__,
						ret);
			else {
				pr_notice("%s: set gamut map successfully.\n", __func__);
				pdata->panel_info.gm_flags &= ~MDSS_CC_FLAGS_GAMUT_DIRTY;
			}
		}

		for (i=0; i < MDP_GAMUT_TABLE_NUM; i++) {
			kfree(gm_data.r_tbl[i]);
			kfree(gm_data.g_tbl[i]);
			kfree(gm_data.b_tbl[i]);
		}
	} else
		pr_notice("%s: gamut map does not require updating.\n", __func__);
}

static int read_file(char *fname, void *buf,	unsigned sz)
{
	struct file *fp = NULL;
	mm_segment_t fs;
	int rc = 0;

	if (!fname || !buf)
		return -EINVAL;

	/* Save the segment descriptor and associate it with the kernel */
	fs = get_fs();
	set_fs(KERNEL_DS);

	/* Open the dev node for reading */
	fp = filp_open(fname, O_NOFOLLOW | O_RDONLY, 0);
	if (!IS_ERR(fp)) {
		rc = fp->f_op->read(fp, buf, sz, &fp->f_pos);
		if (rc < 0)
			pr_err("Failed on read(%s). rc=%d.\n", fname, -rc);

		filp_close(fp, NULL);
	} else {
		rc = (int) PTR_ERR(fp);
		pr_err("Failed on filp_open(%s). rc=%d.\n", fname, -rc);
	}

	/* Restore segment descriptor and close */
	set_fs(fs);

	return (rc < 0 ? rc : 0);
}

void print_buf(const char *str, uint8_t *buf, int length)
{
	int i, off;
	size_t alloc_length = strlen(str) + length * 3 + 1;
	char *print_buf = kzalloc(alloc_length, GFP_KERNEL);
	if (!print_buf) {
		pr_info("%s: alloc failed\n", __func__);
		return;
	}

	memcpy(print_buf, str, strlen(str));
	for (i = 0; i < length; i++) {
		off = strlen(str) + (i*3);
		snprintf(print_buf + off, alloc_length - off, " %02x", buf[i]);
	}
	pr_info("%s\n", print_buf);
	kfree(print_buf);
}

void mdss_dsi_read_serial_id(struct mdss_panel_info *pinfo)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_data *pdata = NULL;
	bool nv_corruption;
	struct logworthy_event_details_t details;

	if (pinfo == NULL) {
		pr_err("%s: Invalid arg\n", __func__);
		return;
	}
	pdata = container_of(pinfo, struct mdss_panel_data, panel_info);
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	if (pinfo->read_serial_id_bytes && pinfo->serial_id_length) {
		int i, ret;
		u8 *rbuf;
		u8 read_length = 0;
		for (i = 0; i < pinfo->serial_id_length; i++)
			if (read_length < pinfo->read_serial_id_bytes[i] + 1)
					read_length = pinfo->read_serial_id_bytes[i] + 1;

		pr_info("%s: serial ID read length %d\n", __func__, read_length);
		rbuf = kzalloc(sizeof(u8) * read_length, GFP_KERNEL);
		if (!rbuf) {
			pr_err("%s: failed to alloc serial ID read buffer", __func__);
			return;
		}

		for (i = 0; i < ctrl->read_serial_cmds.cmd_cnt; i++) {
			struct dcs_cmd_req cmdreq;
			memset(&cmdreq, 0, sizeof(cmdreq));
			cmdreq.cmds = ctrl->read_serial_cmds.cmds + i;
			cmdreq.cmds_cnt = 1;
			cmdreq.flags = CMD_REQ_COMMIT | CMD_REQ_LP_MODE;
			if ((ctrl->read_serial_cmds.cmds[i].dchdr.dtype == DTYPE_DCS_READ) ||
				(ctrl->read_serial_cmds.cmds[i].dchdr.dtype == DTYPE_GEN_READ) ||
				(ctrl->read_serial_cmds.cmds[i].dchdr.dtype == DTYPE_GEN_READ1) ||
				(ctrl->read_serial_cmds.cmds[i].dchdr.dtype == DTYPE_GEN_READ2)) {

				cmdreq.flags |= CMD_REQ_RX;
				cmdreq.rlen = read_length;
				cmdreq.rbuf = rbuf;
			}
			ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);
			if (ret <= 0) {
				pr_err("%s Reading serial ID failed with %d\n",
					__func__, ret);
				kzfree(rbuf);
				return;
			}
		}

		if (!pinfo->serial_id)
			pinfo->serial_id = kzalloc(sizeof(char) * pinfo->serial_id_length, GFP_KERNEL);

		if (pinfo->serial_id) {
			for (i = 0; i < pinfo->serial_id_length; i++)
				pinfo->serial_id[i] = rbuf[pinfo->read_serial_id_bytes[i]];

			print_buf("SID:", pinfo->serial_id, pinfo->serial_id_length);

			/* logan specific */
			nv_corruption =
					(pinfo->serial_id[5] == 0) ||
					(pinfo->serial_id[7] < 0x6F) || (pinfo->serial_id[7] > 0xD2) ||
					(pinfo->serial_id[8] < 0x31) || (pinfo->serial_id[8] > 0x39);


			/* logan specific */
			if (nv_corruption && ctrl->logan_rec_data == NULL) {
				pr_err("%s: Serial ID was corrupt. Enable NV recovery mode.\n", __func__);
				details.d1 = 0xE5D;
				details.d2 = 0;
				details.d3 = 0;
				details.d4 = 0;
				details.creator_id = "display";

				ret = ddt_send(LW_EVENT_DISPLAY_00002530, &details,	"['kerneldump://']");
				pr_info("%s: Send LW_EVENT_DISPLAY_00002530 ret %d", __func__, ret);

				ctrl->logan_rec_data = kzalloc(sizeof(struct logan_recovery_data), GFP_KERNEL);
				if (!ctrl->logan_rec_data) {
					pr_err("%s: alloc failed\n", __func__);
				} else {
					ret = read_file("/nvram/boardid/display_recovery_data", (void *)ctrl->logan_rec_data, sizeof(struct logan_recovery_data));
					if (!ret) {
						print_buf("display_recovery_data:", (uint8_t *)ctrl->logan_rec_data, sizeof(struct logan_recovery_data));

						/* use the recovery register values which we read from nvram
						 * to populate the recovery command sequence we read from the dt */
						 for (i = 0; i < ctrl->on_recovery_cmds.cmd_cnt; i++) {
							if (ctrl->on_recovery_cmds.cmds[i].payload[0] == 0xB4)
								/* for B4, skip the first byte. we want this to always be 0, regardless of the backed-up NV value */
								memcpy(ctrl->on_recovery_cmds.cmds[i].payload+2, ctrl->logan_rec_data->reg_B4+1, sizeof(ctrl->logan_rec_data->reg_B4)-1);
							else if (ctrl->on_recovery_cmds.cmds[i].payload[0] == 0xB6)
								memcpy(ctrl->on_recovery_cmds.cmds[i].payload+1, ctrl->logan_rec_data->reg_B6, sizeof(ctrl->logan_rec_data->reg_B6));
							else if (ctrl->on_recovery_cmds.cmds[i].payload[0] == 0xC8)
								memcpy(ctrl->on_recovery_cmds.cmds[i].payload+1, ctrl->logan_rec_data->reg_C8, sizeof(ctrl->logan_rec_data->reg_C8));
						}
					} else {
						pr_err("%s: Failed to read_file(). rc=%d.\n", __func__, -ret);
						/* Plan B: populate logan_rec_data with the default dtsi values for b4 an c8, so that these can be used by the gamma calculator */
						for (i = 0; i < ctrl->on_recovery_cmds.cmd_cnt; i++) {
							if (ctrl->on_recovery_cmds.cmds[i].payload[0] == 0xB4)
								memcpy(ctrl->logan_rec_data->reg_B4, ctrl->on_recovery_cmds.cmds[i].payload+1, sizeof(ctrl->logan_rec_data->reg_B4));
							else if (ctrl->on_recovery_cmds.cmds[i].payload[0] == 0xC8)
								memcpy(ctrl->logan_rec_data->reg_C8, ctrl->on_recovery_cmds.cmds[i].payload+1, sizeof(ctrl->logan_rec_data->reg_C8));
						}
					}
				}
			}
		} else {
			pr_err("%s:serial id alloc failed\n", __func__);
		}
		kzfree(rbuf);
	}
}

void mdss_dsi_set_partial_window(struct mdss_panel_info *pinfo, int sr, int er)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_data *pdata = NULL;
	int i;

	if (!pinfo || sr < 0 || sr >= pinfo->yres || er <= 0 || er > pinfo->yres || er <= sr) {
		pr_err("%s: Invalid arg pinfo = %p, sr = %d, er = %d\n", __func__, pinfo, sr, er);
		return;
	}

	pdata = container_of(pinfo, struct mdss_panel_data, panel_info);
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	if (sr == 0 && er == pinfo->yres) {
		pr_info("%s: Disable\n", __func__);
		if (ctrl->partial_window_dis_cmds.cmd_cnt)
			mdss_dsi_panel_cmds_send(ctrl, &ctrl->partial_window_dis_cmds, CMD_REQ_COMMIT);

	} else if (ctrl->partial_window_en_cmds.cmd_cnt &&
			ctrl->partial_window_sr_bytes &&
			ctrl->partial_window_er_bytes) {

		pr_info("%s: Enable %d -> %d\n", __func__, sr, er);

		/* Populate the commands with the sr and er values */
		for (i = 0; i < ctrl->partial_window_sr_bytes_length; i++) {
			int idx = ctrl->partial_window_sr_bytes[i];
			/* grab the ith byte of sr and put it into the index specified by partial_window_sr_bytes */
			ctrl->partial_window_en_cmds.buf[idx] = (sr >> (8 * i)) & 0xff;
		}
		for (i = 0; i < ctrl->partial_window_er_bytes_length; i++) {
			int idx = ctrl->partial_window_er_bytes[i];
			ctrl->partial_window_en_cmds.buf[idx] = (er >> (8 * i)) & 0xff;
		}

		mdss_dsi_panel_cmds_send(ctrl, &ctrl->partial_window_en_cmds, CMD_REQ_COMMIT);
	}
}
#endif

static int mdss_dsi_panel_on(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;
	struct dsi_panel_cmds *on_cmds;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

#ifdef CONFIG_BBRY
	mdss_apply_pcc(pdata);
	mdss_apply_gamut_map(pdata);
#endif

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			goto end;
	}

	on_cmds = &ctrl->on_cmds;

	if ((pinfo->mipi.dms_mode == DYNAMIC_MODE_SWITCH_IMMEDIATE) &&
			(pinfo->mipi.boot_mode != pinfo->mipi.mode))
		on_cmds = &ctrl->post_dms_on_cmds;

#ifdef CONFIG_BBRY
	if (ctrl->logan_rec_data) {
		pr_info("%s: Using NV recovery command set.\n", __func__);
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->on_recovery_cmds, CMD_REQ_COMMIT);
	}
#endif

	if (on_cmds->cmd_cnt)

#ifndef CONFIG_BBRY
		mdss_dsi_panel_cmds_send(ctrl, on_cmds, CMD_REQ_COMMIT);
#else
		if (mdss_dsi_panel_cmds_send(ctrl, on_cmds, CMD_REQ_COMMIT) <= 0)
			return -EIO;
#endif

end:
	pinfo->blank_state = MDSS_PANEL_BLANK_UNBLANK;
	pr_debug("%s:-\n", __func__);
	return 0;
}

static int mdss_dsi_panel_off(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			goto end;
	}

	if (ctrl->off_cmds.cmd_cnt)
		mdss_dsi_panel_cmds_send(ctrl, &ctrl->off_cmds, CMD_REQ_COMMIT);

#ifdef CONFIG_BBRY
	mdss_dsi_panel_reset(pdata, 0);
	mdss_dsi_panel_reset(pdata, 1);

	mdss_dsi_read_serial_id(pinfo); /* To check for the NV corruption case (Logan) */
	pdata->panel_info.panel_glass_on = 0;
#endif

end:
	pinfo->blank_state = MDSS_PANEL_BLANK_BLANK;
	pr_debug("%s:-\n", __func__);
	return 0;
}

#ifdef CONFIG_BBRY
static int mdss_dsi_glass_on(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;
	int ret;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: ctrl=%p ndx=%d\n", __func__, ctrl, ctrl->ndx);

	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			goto end;
	}

	if (ctrl->glass_on_cmds.cmd_cnt) {
		struct dcs_cmd_req cmdreq;

		memset(&cmdreq, 0, sizeof(cmdreq));
		cmdreq.cmds = ctrl->glass_on_cmds.cmds;
		cmdreq.cmds_cnt = ctrl->glass_on_cmds.cmd_cnt;
		cmdreq.flags = CMD_REQ_COMMIT;

		if (ctrl->glass_on_cmds.link_state == DSI_HS_MODE)
			cmdreq.flags |= CMD_REQ_HS_MODE;
		else
			cmdreq.flags |= CMD_REQ_LP_MODE;

		cmdreq.rlen = 0;
		cmdreq.cb = NULL;

		ret = mdss_dsi_cmdlist_put(ctrl, &cmdreq);
		if (ret <= 0) {
			pr_err("%s Sending disp_on failed with %d\n",
					__func__, ret);
			return ret;
		}
	}

	pdata->panel_info.panel_glass_on = 1;
	pdata->panel_info.panel_recovery = false;

end:
	/* Print this all the time so we know how long the wakeup took */
	pr_info("%s:-\n", __func__);
	return 0;
}
#endif

static int mdss_dsi_panel_low_power_config(struct mdss_panel_data *pdata,
	int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

#ifdef CONFIG_BBRY
	pr_notice("%s: ctrl=%p ndx=%d enable=%d\n", __func__, ctrl, ctrl->ndx,
		enable);
	if (enable) {
		pinfo->blank_state = MDSS_PANEL_BLANK_LOW_POWER;
		if(ctrl->on_to_lpm_cmds.cmd_cnt)
			mdss_dsi_panel_cmds_send(ctrl, &ctrl->on_to_lpm_cmds, CMD_REQ_COMMIT);
	} else {
		pinfo->blank_state = MDSS_PANEL_BLANK_UNBLANK;
		if(ctrl->lpm_to_on_cmds.cmd_cnt)
			mdss_dsi_panel_cmds_send(ctrl, &ctrl->lpm_to_on_cmds, CMD_REQ_COMMIT);
	}
#else
	pr_debug("%s: ctrl=%p ndx=%d enable=%d\n", __func__, ctrl, ctrl->ndx,
		enable);	

	/* Any panel specific low power commands/config */
	if (enable)
		pinfo->blank_state = MDSS_PANEL_BLANK_LOW_POWER;
	else
		pinfo->blank_state = MDSS_PANEL_BLANK_UNBLANK;
#endif

	pr_debug("%s:-\n", __func__);
	return 0;
}

static void mdss_dsi_parse_lane_swap(struct device_node *np, char *dlane_swap)
{
	const char *data;

	*dlane_swap = DSI_LANE_MAP_0123;
	data = of_get_property(np, "qcom,mdss-dsi-lane-map", NULL);
	if (data) {
		if (!strcmp(data, "lane_map_3012"))
			*dlane_swap = DSI_LANE_MAP_3012;
		else if (!strcmp(data, "lane_map_2301"))
			*dlane_swap = DSI_LANE_MAP_2301;
		else if (!strcmp(data, "lane_map_1230"))
			*dlane_swap = DSI_LANE_MAP_1230;
		else if (!strcmp(data, "lane_map_0321"))
			*dlane_swap = DSI_LANE_MAP_0321;
		else if (!strcmp(data, "lane_map_1032"))
			*dlane_swap = DSI_LANE_MAP_1032;
		else if (!strcmp(data, "lane_map_2103"))
			*dlane_swap = DSI_LANE_MAP_2103;
		else if (!strcmp(data, "lane_map_3210"))
			*dlane_swap = DSI_LANE_MAP_3210;
	}
}

static void mdss_dsi_parse_trigger(struct device_node *np, char *trigger,
		char *trigger_key)
{
	const char *data;

	*trigger = DSI_CMD_TRIGGER_SW;
	data = of_get_property(np, trigger_key, NULL);
	if (data) {
		if (!strcmp(data, "none"))
			*trigger = DSI_CMD_TRIGGER_NONE;
		else if (!strcmp(data, "trigger_te"))
			*trigger = DSI_CMD_TRIGGER_TE;
		else if (!strcmp(data, "trigger_sw_seof"))
			*trigger = DSI_CMD_TRIGGER_SW_SEOF;
		else if (!strcmp(data, "trigger_sw_te"))
			*trigger = DSI_CMD_TRIGGER_SW_TE;
	}
}


static int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key)
{
	const char *data;
	int blen = 0, len;
	char *buf, *bp;
	struct dsi_ctrl_hdr *dchdr;
	int i, cnt;

	data = of_get_property(np, cmd_key, &blen);
	if (!data) {
		pr_err("%s: failed, key=%s\n", __func__, cmd_key);
		return -ENOMEM;
	}

	buf = kzalloc(sizeof(char) * blen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, data, blen);

	/* scan dcs commands */
	bp = buf;
	len = blen;
	cnt = 0;
	while (len >= sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen > len) {
			pr_err("%s: dtsi cmd=%x error, len=%d",
				__func__, dchdr->dtype, dchdr->dlen);
			goto exit_free;
		}
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		cnt++;
	}

	if (len != 0) {
		pr_err("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		goto exit_free;
	}

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc),
						GFP_KERNEL);
	if (!pcmds->cmds)
		goto exit_free;

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < cnt; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		pcmds->cmds[i].dchdr = *dchdr;
		pcmds->cmds[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
	}

	/*Set default link state to LP Mode*/
	pcmds->link_state = DSI_LP_MODE;

	if (link_key) {
		data = of_get_property(np, link_key, NULL);
		if (data && !strcmp(data, "dsi_hs_mode"))
			pcmds->link_state = DSI_HS_MODE;
		else
			pcmds->link_state = DSI_LP_MODE;
	}

	pr_debug("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt, pcmds->link_state);

	return 0;

exit_free:
	kfree(buf);
	return -ENOMEM;
}


int mdss_panel_get_dst_fmt(u32 bpp, char mipi_mode, u32 pixel_packing,
				char *dst_format)
{
	int rc = 0;
	switch (bpp) {
	case 3:
		*dst_format = DSI_CMD_DST_FORMAT_RGB111;
		break;
	case 8:
		*dst_format = DSI_CMD_DST_FORMAT_RGB332;
		break;
	case 12:
		*dst_format = DSI_CMD_DST_FORMAT_RGB444;
		break;
	case 16:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB565;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		}
		break;
	case 18:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB666;
			break;
		default:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		}
		break;
	case 24:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB888;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int mdss_dsi_parse_fbc_params(struct device_node *np,
				struct fbc_panel_info *fbc)
{
	int rc, fbc_enabled = 0;
	u32 tmp;

	fbc_enabled = of_property_read_bool(np,	"qcom,mdss-dsi-fbc-enable");
	if (fbc_enabled) {
		pr_debug("%s:%d FBC panel enabled.\n", __func__, __LINE__);
		fbc->enabled = 1;
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bpp", &tmp);
		fbc->target_bpp =	(!rc ? tmp : 24);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-packing",
				&tmp);
		fbc->comp_mode = (!rc ? tmp : 0);
		fbc->qerr_enable = of_property_read_bool(np,
			"qcom,mdss-dsi-fbc-quant-error");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bias", &tmp);
		fbc->cd_bias = (!rc ? tmp : 0);
		fbc->pat_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-pat-mode");
		fbc->vlc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-vlc-mode");
		fbc->bflc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-bflc-mode");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-h-line-budget",
				&tmp);
		fbc->line_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-budget-ctrl",
				&tmp);
		fbc->block_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-block-budget",
				&tmp);
		fbc->block_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossless-threshold", &tmp);
		fbc->lossless_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-threshold", &tmp);
		fbc->lossy_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-rgb-threshold",
				&tmp);
		fbc->lossy_rgb_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-mode-idx", &tmp);
		fbc->lossy_mode_idx = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-slice-height", &tmp);
		fbc->slice_height = (!rc ? tmp : 0);
		fbc->pred_mode = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-2d-pred-mode");
		fbc->enc_mode = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-ver2-mode");
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-max-pred-err", &tmp);
		fbc->max_pred_err = (!rc ? tmp : 0);
	} else {
		pr_debug("%s:%d Panel does not support FBC.\n",
				__func__, __LINE__);
		fbc->enabled = 0;
		fbc->target_bpp = 24;
	}
	return 0;
}

static void mdss_panel_parse_te_params(struct device_node *np,
				       struct mdss_panel_timing *timing)
{
	struct mdss_mdp_pp_tear_check *te = &timing->te;
	u32 tmp;
	int rc = 0;
	/*
	 * TE default: dsi byte clock calculated base on 70 fps;
	 * around 14 ms to complete a kickoff cycle if te disabled;
	 * vclk_line base on 60 fps; write is faster than read;
	 * init == start == rdptr;
	 */
	te->tear_check_en =
		!of_property_read_bool(np, "qcom,mdss-tear-check-disable");
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-cfg-height", &tmp);
	te->sync_cfg_height = (!rc ? tmp : 0xfff0);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-init-val", &tmp);
	te->vsync_init_val = (!rc ? tmp : timing->yres);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-threshold-start", &tmp);
	te->sync_threshold_start = (!rc ? tmp : 4);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-threshold-continue", &tmp);
	te->sync_threshold_continue = (!rc ? tmp : 4);
	rc = of_property_read_u32(np, "qcom,mdss-tear-check-start-pos", &tmp);
	te->start_pos = (!rc ? tmp : te->vsync_init_val);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-rd-ptr-trigger-intr", &tmp);
	te->rd_ptr_irq = (!rc ? tmp : te->vsync_init_val + 1);
	rc = of_property_read_u32(np, "qcom,mdss-tear-check-frame-rate", &tmp);
	te->refx100 = (!rc ? tmp : 6000);
}


static int mdss_dsi_parse_reset_seq(struct device_node *np,
		u32 rst_seq[MDSS_DSI_RST_SEQ_LEN], u32 *rst_len,
		const char *name)
{
	int num = 0, i;
	int rc;
	struct property *data;
	u32 tmp[MDSS_DSI_RST_SEQ_LEN];
	*rst_len = 0;
	data = of_find_property(np, name, &num);
	num /= sizeof(u32);
	if (!data || !num || num > MDSS_DSI_RST_SEQ_LEN || num % 2) {
		pr_debug("%s:%d, error reading %s, length found = %d\n",
			__func__, __LINE__, name, num);
	} else {
		rc = of_property_read_u32_array(np, name, tmp, num);
		if (rc)
			pr_debug("%s:%d, error reading %s, rc = %d\n",
				__func__, __LINE__, name, rc);
		else {
			for (i = 0; i < num; ++i)
				rst_seq[i] = tmp[i];
			*rst_len = num;
		}
	}
	return 0;
}

static int mdss_dsi_gen_read_status(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	if (!mdss_dsi_cmp_panel_reg(ctrl_pdata->status_buf,
		ctrl_pdata->status_value, 0)) {
		pr_err("%s: Read back value from panel is incorrect\n",
							__func__);
		return -EINVAL;
	} else {
		return 1;
	}
}

static int mdss_dsi_nt35596_read_status(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	if (!mdss_dsi_cmp_panel_reg(ctrl_pdata->status_buf,
		ctrl_pdata->status_value, 0)) {
		ctrl_pdata->status_error_count = 0;
		pr_err("%s: Read back value from panel is incorrect\n",
							__func__);
		return -EINVAL;
	} else {
		if (!mdss_dsi_cmp_panel_reg(ctrl_pdata->status_buf,
			ctrl_pdata->status_value, 3)) {
			ctrl_pdata->status_error_count = 0;
		} else {
			if (mdss_dsi_cmp_panel_reg(ctrl_pdata->status_buf,
				ctrl_pdata->status_value, 4) ||
				mdss_dsi_cmp_panel_reg(ctrl_pdata->status_buf,
				ctrl_pdata->status_value, 5))
				ctrl_pdata->status_error_count = 0;
			else
				ctrl_pdata->status_error_count++;
			if (ctrl_pdata->status_error_count >=
					ctrl_pdata->max_status_error_count) {
				ctrl_pdata->status_error_count = 0;
				pr_err("%s: Read value bad. Error_cnt = %i\n",
					 __func__,
					ctrl_pdata->status_error_count);
				return -EINVAL;
			}
		}
		return 1;
	}
}

static void mdss_dsi_parse_roi_alignment(struct device_node *np,
		struct mdss_panel_info *pinfo)
{
	int len = 0;
	u32 value[6];
	struct property *data;
	data = of_find_property(np, "qcom,panel-roi-alignment", &len);
	len /= sizeof(u32);
	if (!data || (len != 6)) {
		pr_debug("%s: Panel roi alignment not found", __func__);
	} else {
		int rc = of_property_read_u32_array(np,
				"qcom,panel-roi-alignment", value, len);
		if (rc)
			pr_debug("%s: Error reading panel roi alignment values",
					__func__);
		else {
			pinfo->xstart_pix_align = value[0];
			pinfo->width_pix_align = value[1];
			pinfo->ystart_pix_align = value[2];
			pinfo->height_pix_align = value[3];
			pinfo->min_width = value[4];
			pinfo->min_height = value[5];
		}

		pr_debug("%s: ROI alignment: [%d, %d, %d, %d, %d, %d]",
				__func__, pinfo->xstart_pix_align,
				pinfo->width_pix_align, pinfo->ystart_pix_align,
				pinfo->height_pix_align, pinfo->min_width,
				pinfo->min_height);
	}
}

static void mdss_dsi_parse_dms_config(struct device_node *np,
	struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct mdss_panel_info *pinfo = &ctrl->panel_data.panel_info;
	const char *data;
	bool dms_enabled;

	dms_enabled = of_property_read_bool(np,
		"qcom,dynamic-mode-switch-enabled");

	if (!dms_enabled) {
		pinfo->mipi.dms_mode = DYNAMIC_MODE_SWITCH_DISABLED;
		goto exit;
	}

	/* default mode is suspend_resume */
	pinfo->mipi.dms_mode = DYNAMIC_MODE_SWITCH_SUSPEND_RESUME;
	data = of_get_property(np, "qcom,dynamic-mode-switch-type", NULL);
	if (data && !strcmp(data, "dynamic-resolution-switch-immediate")) {
		if (!list_empty(&ctrl->panel_data.timings_list))
			pinfo->mipi.dms_mode =
				DYNAMIC_MODE_RESOLUTION_SWITCH_IMMEDIATE;
		else
			pinfo->mipi.dms_mode =
				DYNAMIC_MODE_SWITCH_DISABLED;

		goto exit;
	}

	if (data && !strcmp(data, "dynamic-switch-immediate"))
		pinfo->mipi.dms_mode = DYNAMIC_MODE_SWITCH_IMMEDIATE;
	else
		pr_debug("%s: default dms suspend/resume\n", __func__);

	mdss_dsi_parse_dcs_cmds(np, &ctrl->video2cmd,
		"qcom,video-to-cmd-mode-switch-commands", NULL);

	mdss_dsi_parse_dcs_cmds(np, &ctrl->cmd2video,
		"qcom,cmd-to-video-mode-switch-commands", NULL);

	mdss_dsi_parse_dcs_cmds(np, &ctrl->post_dms_on_cmds,
		"qcom,mdss-dsi-post-mode-switch-on-command",
		"qcom,mdss-dsi-post-mode-switch-on-command-state");

	if (pinfo->mipi.dms_mode == DYNAMIC_MODE_SWITCH_IMMEDIATE &&
		!ctrl->post_dms_on_cmds.cmd_cnt) {
		pr_warn("%s: No post dms on cmd specified\n", __func__);
		pinfo->mipi.dms_mode = DYNAMIC_MODE_SWITCH_DISABLED;
	}

	if (!ctrl->video2cmd.cmd_cnt || !ctrl->cmd2video.cmd_cnt) {
		pr_warn("%s: No commands specified for dynamic switch\n",
			__func__);
		pinfo->mipi.dms_mode = DYNAMIC_MODE_SWITCH_DISABLED;
	}
exit:
	pr_info("%s: dynamic switch feature enabled: %d\n", __func__,
		pinfo->mipi.dms_mode);
	return;
}

static void mdss_dsi_parse_esd_params(struct device_node *np,
	struct mdss_dsi_ctrl_pdata *ctrl)
{
	u32 tmp;
	int rc;
	struct property *data;
	const char *string;
	struct mdss_panel_info *pinfo = &ctrl->panel_data.panel_info;

	pinfo->esd_check_enabled = of_property_read_bool(np,
		"qcom,esd-check-enabled");

	if (!pinfo->esd_check_enabled)
		return;

	mdss_dsi_parse_dcs_cmds(np, &ctrl->status_cmds,
			"qcom,mdss-dsi-panel-status-command",
				"qcom,mdss-dsi-panel-status-command-state");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-status-read-length",
		&tmp);
	ctrl->status_cmds_rlen = (!rc ? tmp : 1);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-max-error-count",
		&tmp);
	ctrl->max_status_error_count = (!rc ? tmp : 0);

	ctrl->status_value = kzalloc(sizeof(u32) * ctrl->status_cmds_rlen,
				GFP_KERNEL);
	if (!ctrl->status_value) {
		pr_err("%s: Error allocating memory for status buffer\n",
			__func__);
		pinfo->esd_check_enabled = false;
		return;
	}

	data = of_find_property(np, "qcom,mdss-dsi-panel-status-value", &tmp);
	tmp /= sizeof(u32);
	if (!data || (tmp != ctrl->status_cmds_rlen)) {
		pr_debug("%s: Panel status values not found\n", __func__);
		memset(ctrl->status_value, 0, ctrl->status_cmds_rlen);
	} else {
		rc = of_property_read_u32_array(np,
			"qcom,mdss-dsi-panel-status-value",
			ctrl->status_value, tmp);
		if (rc) {
			pr_debug("%s: Error reading panel status values\n",
					__func__);
			memset(ctrl->status_value, 0, ctrl->status_cmds_rlen);
		}
	}

	ctrl->status_mode = ESD_MAX;
	rc = of_property_read_string(np,
			"qcom,mdss-dsi-panel-status-check-mode", &string);
	if (!rc) {
		if (!strcmp(string, "bta_check")) {
			ctrl->status_mode = ESD_BTA;
		} else if (!strcmp(string, "reg_read")) {
			ctrl->status_mode = ESD_REG;
			ctrl->check_read_status =
				mdss_dsi_gen_read_status;
		} else if (!strcmp(string, "reg_read_nt35596")) {
			ctrl->status_mode = ESD_REG_NT35596;
			ctrl->status_error_count = 0;
			ctrl->check_read_status =
				mdss_dsi_nt35596_read_status;
		} else if (!strcmp(string, "te_signal_check")) {
			if (pinfo->mipi.mode == DSI_CMD_MODE) {
				ctrl->status_mode = ESD_TE;
			} else {
				pr_err("TE-ESD not valid for video mode\n");
				goto error;
			}
		} else {
			pr_err("No valid panel-status-check-mode string\n");
			goto error;
		}
	}
	return;

error:
	kfree(ctrl->status_value);
	pinfo->esd_check_enabled = false;
}

static int mdss_dsi_parse_panel_features(struct device_node *np,
	struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct mdss_panel_info *pinfo;

	if (!np || !ctrl) {
		pr_err("%s: Invalid arguments\n", __func__);
		return -ENODEV;
	}

	pinfo = &ctrl->panel_data.panel_info;

	pinfo->cont_splash_enabled = of_property_read_bool(np,
		"qcom,cont-splash-enabled");

	pinfo->partial_update_supported = of_property_read_bool(np,
		"qcom,partial-update-enabled");
	if (pinfo->mipi.mode == DSI_CMD_MODE) {
		pinfo->partial_update_enabled = pinfo->partial_update_supported;
		pr_info("%s: partial_update_enabled=%d\n", __func__,
					pinfo->partial_update_enabled);
		ctrl->set_col_page_addr = mdss_dsi_set_col_page_addr;
		if (pinfo->partial_update_enabled) {
			pinfo->partial_update_roi_merge =
					of_property_read_bool(np,
					"qcom,partial-update-roi-merge");
		}

		pinfo->dcs_cmd_by_left = of_property_read_bool(np,
						"qcom,dcs-cmd-by-left");
	}

	pinfo->ulps_feature_enabled = of_property_read_bool(np,
		"qcom,ulps-enabled");
	pr_info("%s: ulps feature %s\n", __func__,
		(pinfo->ulps_feature_enabled ? "enabled" : "disabled"));

	pinfo->ulps_suspend_enabled = of_property_read_bool(np,
		"qcom,suspend-ulps-enabled");
	pr_info("%s: ulps during suspend feature %s", __func__,
		(pinfo->ulps_suspend_enabled ? "enabled" : "disabled"));

	mdss_dsi_parse_dms_config(np, ctrl);

	pinfo->panel_ack_disabled = of_property_read_bool(np,
				"qcom,panel-ack-disabled");

	mdss_dsi_parse_esd_params(np, ctrl);

	if (pinfo->panel_ack_disabled && pinfo->esd_check_enabled) {
		pr_warn("ESD should not be enabled if panel ACK is disabled\n");
		pinfo->esd_check_enabled = false;
	}

	if (ctrl->disp_en_gpio <= 0) {
		ctrl->disp_en_gpio = of_get_named_gpio(
			np,
			"qcom,5v-boost-gpio", 0);

		if (!gpio_is_valid(ctrl->disp_en_gpio))
			pr_err("%s:%d, Disp_en gpio not specified\n",
					__func__, __LINE__);
	}

	return 0;
}

#ifdef CONFIG_BBRY
static int mdss_panel_parse_bbry_lcd_id(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc;
	u32 mfg, class;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

	rc = of_property_read_u32(np, "oem,mdss-dsi-bbry-lcd-id-class", &class);
	if (rc) {
		pr_notice("%s:%d, bbry-lcd-id-class not specified in dt\n",	__func__, __LINE__);
		class = 0;
	}

	rc = of_property_read_u32(np, "oem,mdss-dsi-bbry-lcd-id-mfg", &mfg);
	if (rc) {
		pr_notice("%s:%d, bbry-lcd-id-mfg not specified in dt\n",	__func__, __LINE__);
		mfg = 0;
	}

	pinfo->bbry_lcd_id = (mfg & 0xff) | ((class & 0xfff) << 8);

	/*TODO Handle cases where there are different revs of panels.
	 * Handle cases where we must determine class/mfg by reading some panel register. */
	return 0;
}

static int mdss_panel_parse_panel_color_points(struct device_node *np,
			struct mdss_panel_info *pinfo)
{
	int num = 0;
	int rc = 0;
	int i;
	int j;
	struct property *data;
	u32 tmp[MDSS_COLOR_POINT_TABLE_SIZE];

	data = of_find_property(np, "oem,mdss-dsi-bbry-color-points", &num);
	num /= sizeof(u32);
	if (!data || !num || num > MDSS_COLOR_POINT_TABLE_SIZE) {
		pr_debug("%s:%d, not able to read oem,mdss-dsi-bbry-color-points, "
				 "length found = %d, not enabling color correction\n",
				 __func__, __LINE__, num);
	} else {
		memset( pinfo->color_points, 0, MDSS_COLOR_POINT_TABLE_SIZE);
		if (num < MDSS_COLOR_POINT_TABLE_SIZE)
			pr_err("%s: Not enough color point data in dtsi\n", __func__);
		else {
			rc = of_property_read_u32_array(np, "oem,mdss-dsi-bbry-color-points",
					tmp, num);
			if (rc)
				pr_debug("%s:%d, error reading oem,mdss-dsi-bbry-color-points, rc = %d\n",
						__func__, __LINE__, rc);
			else {
				for (i = 0, j = 0;
						(i < num) && (j < MDSS_MAX_COLOR_POINTS); j++ ) {

					pinfo->color_points[j].rgb[0] = tmp[i++];
					pinfo->color_points[j].rgb[1] = tmp[i++];
					pinfo->color_points[j].rgb[2] = tmp[i++];

					pinfo->color_points[j].xyY[0] = tmp[i++];
					pinfo->color_points[j].xyY[1] = tmp[i++];
					pinfo->color_points[j].xyY[2] = tmp[i++];
				}
			}
		}
	}

    return 0;
}

static int mdss_panel_parse_panel_pcc(struct device_node *np,
			struct mdss_panel_info *pinfo)
{
	int num = 0;
	int rc;
	struct property *data;
	u32 tmp[MDSS_PCC_DATA_TABLE_SIZE];

	data = of_find_property(np, "qcom,mdss-dsi-bbry-pcc-data", &num);
	num /= sizeof(u32);
	if (!data || !num || num > MDSS_PCC_DATA_TABLE_SIZE) {
		pr_debug("%s:%d, not able to read qcom,mdss-dsi-bbry-pcc-data, "
				 "length found = %d, not enabling color correction\n",
				 __func__, __LINE__, num);
		pinfo->pcc_enabled = false;
	} else {
		rc = of_property_read_u32_array(np, "qcom,mdss-dsi-bbry-pcc-data", tmp, num);
		if (rc)
			pr_debug("%s:%d, error reading qcom,mdss-dsi-bbry-pcc-data, rc = %d\n",
				      __func__, __LINE__, rc);
		else {
			memcpy(&pinfo->pcc_data[0].c, &tmp[0],
					sizeof(u32)*MDSS_PCC_DATA_TABLE_SIZE/3);
			memcpy(&pinfo->pcc_data[1].c, &tmp[12],
					sizeof(u32)*MDSS_PCC_DATA_TABLE_SIZE/3);
			memcpy(&pinfo->pcc_data[2].c, &tmp[24],
					sizeof(u32)*MDSS_PCC_DATA_TABLE_SIZE/3);
			pinfo->pcc_enabled = true;
		}
	}
	return 0;
}

#define MDSS_GAMUT_MAP_SIZE 729
static int mdss_panel_parse_gamut_map(struct device_node *np,
			struct mdss_panel_info *pinfo)
{
	int num = 0;
	int rc;
	int	i;
	struct property *data;
	uint32_t *tmp;
	uint16_t *r_dst;
	uint16_t *g_dst;
	uint16_t *b_dst;

	data = of_find_property(np, "oem,mdss-dsi-panel-gamut-map", &num);
	num /= sizeof(uint32_t);
	if (!data || !num || num > MDSS_GAMUT_MAP_SIZE * 3) {
		pr_debug("%s:%d, not able to read oem,mdss-dsi-panel-gamut-map, "
				 "length found = %d, not enabling color correction\n",
				 __func__, __LINE__, num);
		pinfo->gm_flags &= ~MDSS_CC_FLAGS_GAMUT_DIRTY;
	} else {
		tmp = kzalloc(sizeof(uint32_t) * MDSS_GAMUT_MAP_SIZE * 3, GFP_KERNEL);
		if (!tmp) {
			pr_err("%s:%d, allocation failed\n", __func__, __LINE__);
			return -ENOMEM;
		}
		rc = of_property_read_u32_array(np, "oem,mdss-dsi-panel-gamut-map", tmp, num);
		if (rc)
			pr_debug("%s:%d, error reading oem,mdss-dsi-panel-gamut-map, rc = %d\n",
				      __func__, __LINE__, rc);
		else {
			pinfo->gm_data[0] = kzalloc(sizeof(uint32_t) * MDSS_GAMUT_MAP_SIZE,
										GFP_KERNEL);
			if (!pinfo->gm_data[0]) {
				pr_err("%s:%d, allocation failed", __func__, __LINE__);
				kfree(tmp);
				return -ENOMEM;
			}
			pinfo->gm_data[1] = kzalloc(sizeof(uint32_t) * MDSS_GAMUT_MAP_SIZE,
										GFP_KERNEL);
			if (!pinfo->gm_data[1]) {
				pr_err("%s:%d, allocation failed", __func__, __LINE__);
				kfree(tmp);
				kfree(pinfo->gm_data[0]);
				return -ENOMEM;
			}
			pinfo->gm_data[2] = kzalloc(sizeof(uint32_t) * MDSS_GAMUT_MAP_SIZE,
										GFP_KERNEL);
			if (!pinfo->gm_data[2]) {
				pr_err("%s:%d, allocation failed", __func__, __LINE__);
				kfree(tmp);
				kfree(pinfo->gm_data[0]);
				kfree(pinfo->gm_data[1]);
				return -ENOMEM;
			}
			/* Copy the gamut map from the 32 bits read out of the dtsi
			 * to the 16 bits required by QC API.  Note that this
			 * contradicts the sample code in the Display Post
			 * Processing Features document but is actually required to
			 * get valid results once the hardware is programmed.
			 */
			r_dst = pinfo->gm_data[0];
			g_dst = pinfo->gm_data[1];
			b_dst = pinfo->gm_data[2];
			for(i=0; i < MDSS_GAMUT_MAP_SIZE; i++) {
				*r_dst++ = (uint16_t)tmp[i];
				*g_dst++ = (uint16_t)tmp[i+MDSS_GAMUT_MAP_SIZE];
				*b_dst++ = (uint16_t)tmp[i+MDSS_GAMUT_MAP_SIZE*2];
			}
			pinfo->gm_flags = MDSS_CC_FLAGS_GAMUT_DIRTY;
			kfree(tmp);
		}
	}
	return 0;
}
#endif

#ifdef CONFIG_BBRY_MFG
static char unlock_seq[][3] = {
	{0xf0, 0x5a, 0x5a},
	{0xf1, 0x5a, 0x5a}
};
static char lock_seq[][3] = {
	{0xf1, 0xa5, 0xa5},
	{0xf0, 0xa5, 0xa5},
};

static struct dsi_cmd_desc unlock_seq_cmd[] = {
	{{DTYPE_DCS_LWRITE, 0, 0, 0, 1, 3}, unlock_seq[0]},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, 3}, unlock_seq[1]},
};
static struct dsi_cmd_desc lock_seq_cmd[] = {
	{{DTYPE_DCS_LWRITE, 0, 0, 0, 1, 3}, lock_seq[0]},
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, 3}, lock_seq[1]},
};

int logan_read_recovery_data(struct mdss_panel_info *pinfo, int set, uint8_t *buf, int length)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_data *pdata = NULL;
	struct dcs_cmd_req cmdreq;
	int i;

	/* Each row corresponds to a set of registers which we will read.
	 * We select a row based on the "set" argument */
	const uint8_t read_reg_sets[][13] = {
		{0xC9, 0xB1, 0xB2, 0xB4, 0xB5, 0xB6, 0xB8, 0xBB, 0xC3, 0xC7, 0xC8, 0xCA, 0xCB},
		{0xC9,    0,    0, 0xB4,    0, 0xB6,    0, 0xBB,    0,    0, 0xC8,    0,    0}
	};

	const int read_offsets[] = {
			offsetof(struct logan_recovery_data, reg_C9),
			offsetof(struct logan_recovery_data, reg_B1),
			offsetof(struct logan_recovery_data, reg_B2),
			offsetof(struct logan_recovery_data, reg_B4),
			offsetof(struct logan_recovery_data, reg_B5),
			offsetof(struct logan_recovery_data, reg_B6),
			offsetof(struct logan_recovery_data, reg_B8),
			offsetof(struct logan_recovery_data, reg_BB),
			offsetof(struct logan_recovery_data, reg_C3),
			offsetof(struct logan_recovery_data, reg_C7),
			offsetof(struct logan_recovery_data, reg_C8),
			offsetof(struct logan_recovery_data, reg_CA),
			offsetof(struct logan_recovery_data, reg_CB),
			sizeof(struct logan_recovery_data)
	};

	if (!pinfo || !buf || length < sizeof(struct logan_recovery_data) ||
		set >= sizeof(read_reg_sets) / sizeof(read_reg_sets[0]))
	{
		pr_err("%s: Invalid arg\n", __func__);
		return 0;
	}
	pdata = container_of(pinfo, struct mdss_panel_data, panel_info);
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds_cnt = 2;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.cmds = unlock_seq_cmd;
	if (mdss_dsi_cmdlist_put(ctrl, &cmdreq) <= 0) {
		pr_err("%s: Failed to unlock\n", __func__);
		return 0;
	}

	for (i = 0; i < sizeof(read_reg_sets[0]); i++) {
		if (read_reg_sets[set][i]) {
			memset(&cmdreq, 0, sizeof(cmdreq));
			cmdreq.cmds_cnt = 1;
			cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
			dcs_cmd[0] = read_reg_sets[set][i];
			dcs_cmd[1] = 0;
			cmdreq.cmds = &dcs_read_cmd;
			cmdreq.rlen = read_offsets[i+1] - read_offsets[i];
			cmdreq.rbuf = buf + read_offsets[i];
			if (mdss_dsi_cmdlist_put(ctrl, &cmdreq) <= 0) {
				pr_err("%s: Failed to read %02x\n", __func__, read_reg_sets[set][i]);
				return 0;
			} else {
				pr_info("%s: Read reg %02x to offset %d: %02x %02x...\n", __func__, read_reg_sets[set][i], read_offsets[i], cmdreq.rbuf[0], cmdreq.rbuf[1]);
			}
		}
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds_cnt = 2;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.cmds = lock_seq_cmd;
	if (mdss_dsi_cmdlist_put(ctrl, &cmdreq) <= 0)
		pr_err("%s: Failed to lock\n", __func__);

	return sizeof(struct logan_recovery_data);
}
#endif

static void mdss_dsi_parse_panel_horizintal_line_idle(struct device_node *np,
	struct mdss_dsi_ctrl_pdata *ctrl)
{
	const u32 *src;
	int i, len, cnt;
	struct panel_horizontal_idle *kp;

	if (!np || !ctrl) {
		pr_err("%s: Invalid arguments\n", __func__);
		return;
	}

	src = of_get_property(np, "qcom,mdss-dsi-hor-line-idle", &len);
	if (!src || len == 0)
		return;

	cnt = len % 3; /* 3 fields per entry */
	if (cnt) {
		pr_err("%s: invalid horizontal idle len=%d\n", __func__, len);
		return;
	}

	cnt = len / sizeof(u32);

	kp = kzalloc(sizeof(*kp) * (cnt / 3), GFP_KERNEL);
	if (kp == NULL) {
		pr_err("%s: No memory\n", __func__);
		return;
	}

	ctrl->line_idle = kp;
	for (i = 0; i < cnt; i += 3) {
		kp->min = be32_to_cpu(src[i]);
		kp->max = be32_to_cpu(src[i+1]);
		kp->idle = be32_to_cpu(src[i+2]);
		kp++;
		ctrl->horizontal_idle_cnt++;
	}

	pr_debug("%s: horizontal_idle_cnt=%d\n", __func__,
				ctrl->horizontal_idle_cnt);
}

static int mdss_dsi_set_refresh_rate_range(struct device_node *pan_node,
		struct mdss_panel_info *pinfo)
{
	int rc = 0;
	rc = of_property_read_u32(pan_node,
			"qcom,mdss-dsi-min-refresh-rate",
			&pinfo->min_fps);
	if (rc) {
		pr_warn("%s:%d, Unable to read min refresh rate\n",
				__func__, __LINE__);

		/*
		 * Since min refresh rate is not specified when dynamic
		 * fps is enabled, using minimum as 30
		 */
		pinfo->min_fps = MIN_REFRESH_RATE;
		rc = 0;
	}

	rc = of_property_read_u32(pan_node,
			"qcom,mdss-dsi-max-refresh-rate",
			&pinfo->max_fps);
	if (rc) {
		pr_warn("%s:%d, Unable to read max refresh rate\n",
				__func__, __LINE__);

		/*
		 * Since max refresh rate was not specified when dynamic
		 * fps is enabled, using the default panel refresh rate
		 * as max refresh rate supported.
		 */
		pinfo->max_fps = pinfo->mipi.frame_rate;
		rc = 0;
	}

	pr_info("dyn_fps: min = %d, max = %d\n",
			pinfo->min_fps, pinfo->max_fps);
	return rc;
}

static void mdss_dsi_parse_dfps_config(struct device_node *pan_node,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	const char *data;
	bool dynamic_fps;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

	dynamic_fps = of_property_read_bool(pan_node,
			"qcom,mdss-dsi-pan-enable-dynamic-fps");

	if (!dynamic_fps)
		return;

	pinfo->dynamic_fps = true;
	data = of_get_property(pan_node, "qcom,mdss-dsi-pan-fps-update", NULL);
	if (data) {
		if (!strcmp(data, "dfps_suspend_resume_mode")) {
			pinfo->dfps_update = DFPS_SUSPEND_RESUME_MODE;
			pr_debug("dfps mode: suspend/resume\n");
		} else if (!strcmp(data, "dfps_immediate_clk_mode")) {
			pinfo->dfps_update = DFPS_IMMEDIATE_CLK_UPDATE_MODE;
			pr_debug("dfps mode: Immediate clk\n");
		} else if (!strcmp(data, "dfps_immediate_porch_mode_hfp")) {
			pinfo->dfps_update =
				DFPS_IMMEDIATE_PORCH_UPDATE_MODE_HFP;
			pr_debug("dfps mode: Immediate porch HFP\n");
		} else if (!strcmp(data, "dfps_immediate_porch_mode_vfp")) {
			pinfo->dfps_update =
				DFPS_IMMEDIATE_PORCH_UPDATE_MODE_VFP;
			pr_debug("dfps mode: Immediate porch VFP\n");
		} else {
			pinfo->dfps_update = DFPS_SUSPEND_RESUME_MODE;
			pr_debug("default dfps mode: suspend/resume\n");
		}
		mdss_dsi_set_refresh_rate_range(pan_node, pinfo);
	} else {
		pinfo->dynamic_fps = false;
		pr_debug("dfps update mode not configured: disable\n");
	}
	pinfo->new_fps = pinfo->mipi.frame_rate;

	return;
}

int mdss_dsi_panel_timing_switch(struct mdss_dsi_ctrl_pdata *ctrl,
			struct mdss_panel_timing *timing)
{
	struct dsi_panel_timing *pt;
	struct mdss_panel_info *pinfo = &ctrl->panel_data.panel_info;
	int i;

	if (!timing)
		return -EINVAL;

	if (timing == ctrl->panel_data.current_timing) {
		pr_warn("%s: panel timing \"%s\" already set\n", __func__,
				timing->name);
		return 0; /* nothing to do */
	}

	pr_debug("%s: ndx=%d switching to panel timing \"%s\"\n", __func__,
			ctrl->ndx, timing->name);

	mdss_panel_info_from_timing(timing, pinfo);

	pt = container_of(timing, struct dsi_panel_timing, timing);
	pinfo->mipi.t_clk_pre = pt->t_clk_pre;
	pinfo->mipi.t_clk_post = pt->t_clk_post;

	for (i = 0; i < ARRAY_SIZE(pt->phy_timing); i++)
		pinfo->mipi.dsi_phy_db.timing[i] = pt->phy_timing[i];

	ctrl->on_cmds = pt->on_cmds;

	ctrl->panel_data.current_timing = timing;
	if (!timing->clk_rate)
		ctrl->refresh_clk_rate = true;
	mdss_dsi_clk_refresh(&ctrl->panel_data);

	return 0;
}

static int mdss_dsi_panel_timing_from_dt(struct device_node *np,
	struct dsi_panel_timing *pt)
{
	u32 tmp;
	int rc, i, len;
	const char *data;

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-width", &tmp);
	if (rc) {
		pr_err("%s:%d, panel width not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	pt->timing.xres = tmp;

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-height", &tmp);
	if (rc) {
		pr_err("%s:%d, panel height not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	pt->timing.yres = tmp;

	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-front-porch", &tmp);
	pt->timing.h_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-back-porch", &tmp);
	pt->timing.h_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-pulse-width", &tmp);
	pt->timing.h_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-sync-skew", &tmp);
	pt->timing.hsync_skew = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-back-porch", &tmp);
	pt->timing.v_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-front-porch", &tmp);
	pt->timing.v_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-pulse-width", &tmp);
	pt->timing.v_pulse_width = (!rc ? tmp : 2);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-left-border", &tmp);
	pt->timing.border_left = !rc ? tmp : 0;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-right-border", &tmp);
	pt->timing.border_right = !rc ? tmp : 0;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-top-border", &tmp);
	pt->timing.border_top = !rc ? tmp : 0;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-bottom-border", &tmp);
	pt->timing.border_bottom = !rc ? tmp : 0;

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-framerate", &tmp);
	pt->timing.frame_rate = !rc ? tmp : DEFAULT_FRAME_RATE;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-clockrate", &tmp);
	pt->timing.clk_rate = !rc ? tmp : 0;

	data = of_get_property(np, "qcom,mdss-dsi-panel-timings", &len);
	if ((!data) || (len != 12)) {
		pr_err("%s:%d, Unable to read Phy timing settings",
		       __func__, __LINE__);
		return -EINVAL;
	}
	for (i = 0; i < len; i++)
		pt->phy_timing[i] = data[i];

	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-pre", &tmp);
	pt->t_clk_pre = (!rc ? tmp : 0x24);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-post", &tmp);
	pt->t_clk_post = (!rc ? tmp : 0x03);

	if (np->name) {
		pt->timing.name = kstrdup(np->name, GFP_KERNEL);
		pr_info("%s: found new timing \"%s\" (%p)\n", __func__,
				np->name, &pt->timing);
	}

	return 0;
}

static void  mdss_dsi_panel_config_res_properties(struct device_node *np,
		struct dsi_panel_timing *pt)
{
	mdss_dsi_parse_dcs_cmds(np, &pt->on_cmds,
			"qcom,mdss-dsi-on-command",
			"qcom,mdss-dsi-on-command-state");
	mdss_dsi_parse_dcs_cmds(np, &pt->switch_cmds,
			"qcom,mdss-dsi-timing-switch-command",
			"qcom,mdss-dsi-timing-switch-command-state");
	mdss_dsi_parse_fbc_params(np, &pt->timing.fbc);
	mdss_panel_parse_te_params(np, &pt->timing);
}

static int mdss_dsi_panel_parse_display_timings(struct device_node *np,
		struct mdss_panel_data *panel_data)
{
	struct mdss_dsi_ctrl_pdata *ctrl;
	struct dsi_panel_timing *modedb;
	struct device_node *timings_np;
	struct device_node *entry;
	int num_timings, rc;
	int i = 0, active_ndx = 0;

	ctrl = container_of(panel_data, struct mdss_dsi_ctrl_pdata, panel_data);

	INIT_LIST_HEAD(&panel_data->timings_list);

	timings_np = of_get_child_by_name(np, "qcom,mdss-dsi-display-timings");
	if (!timings_np) {
		struct dsi_panel_timing pt;
		memset(&pt, 0, sizeof(struct dsi_panel_timing));

		/*
		 * display timings node is not available, fallback to reading
		 * timings directly from root node instead
		 */
		pr_debug("reading display-timings from panel node\n");
		rc = mdss_dsi_panel_timing_from_dt(np, &pt);
		if (!rc) {
			mdss_dsi_panel_config_res_properties(np, &pt);
			rc = mdss_dsi_panel_timing_switch(ctrl, &pt.timing);
		}
		return rc;
	}

	num_timings = of_get_child_count(timings_np);
	if (num_timings == 0) {
		pr_err("no timings found within display-timings\n");
		rc = -EINVAL;
		goto exit;
	}

	modedb = kzalloc(num_timings * sizeof(*modedb), GFP_KERNEL);
	if (!modedb) {
		pr_err("unable to allocate modedb\n");
		rc = -ENOMEM;
		goto exit;
	}

	for_each_child_of_node(timings_np, entry) {
		rc = mdss_dsi_panel_timing_from_dt(entry, modedb + i);
		if (rc) {
			kfree(modedb);
			goto exit;
		}

		mdss_dsi_panel_config_res_properties(entry, (modedb + i));

		/* if default is set, use it otherwise use first as default */
		if (of_property_read_bool(entry,
				"qcom,mdss-dsi-timing-default"))
			active_ndx = i;

		list_add(&modedb[i].timing.list,
				&panel_data->timings_list);
		i++;
	}

	/* Configure default timing settings */
	rc = mdss_dsi_panel_timing_switch(ctrl, &modedb[active_ndx].timing);
	if (rc)
		pr_err("unable to configure default timing settings\n");

exit:
	of_node_put(timings_np);

	return rc;
}

static int mdss_panel_parse_dt(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	u32 tmp;
	int rc;
	const char *data;
	static const char *pdest;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

#ifdef CONFIG_BBRY
	mdss_panel_parse_bbry_lcd_id(np, ctrl_pdata);
#endif

	rc = mdss_dsi_panel_parse_display_timings(np,
					&ctrl_pdata->panel_data);
	if (rc)
		return rc;
	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-width-dimension", &tmp);
	pinfo->physical_width = (!rc ? tmp : 0);
	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-height-dimension", &tmp);
	pinfo->physical_height = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-bpp", &tmp);
	if (rc) {
		pr_err("%s:%d, bpp not specified\n", __func__, __LINE__);
		return -EINVAL;
	}
	pinfo->bpp = (!rc ? tmp : 24);
	pinfo->mipi.mode = DSI_VIDEO_MODE;
	data = of_get_property(np, "qcom,mdss-dsi-panel-type", NULL);
	if (data && !strncmp(data, "dsi_cmd_mode", 12))
		pinfo->mipi.mode = DSI_CMD_MODE;
	pinfo->mipi.boot_mode = pinfo->mipi.mode;
	tmp = 0;
	data = of_get_property(np, "qcom,mdss-dsi-pixel-packing", NULL);
	if (data && !strcmp(data, "loose"))
		pinfo->mipi.pixel_packing = 1;
	else
		pinfo->mipi.pixel_packing = 0;
	rc = mdss_panel_get_dst_fmt(pinfo->bpp,
		pinfo->mipi.mode, pinfo->mipi.pixel_packing,
		&(pinfo->mipi.dst_format));
	if (rc) {
		pr_debug("%s: problem determining dst format. Set Default\n",
			__func__);
		pinfo->mipi.dst_format =
			DSI_VIDEO_DST_FORMAT_RGB888;
	}
	pdest = of_get_property(np,
		"qcom,mdss-dsi-panel-destination", NULL);

	if (pdest) {
		if (strlen(pdest) != 9) {
			pr_err("%s: Unknown pdest specified\n", __func__);
			return -EINVAL;
		}
		if (!strcmp(pdest, "display_1"))
			pinfo->pdest = DISPLAY_1;
		else if (!strcmp(pdest, "display_2"))
			pinfo->pdest = DISPLAY_2;
		else {
			pr_debug("%s: incorrect pdest. Set Default\n",
				__func__);
			pinfo->pdest = DISPLAY_1;
		}
	} else {
		pr_debug("%s: pdest not specified. Set Default\n",
				__func__);
		pinfo->pdest = DISPLAY_1;
	}
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-underflow-color", &tmp);
	pinfo->lcdc.underflow_clr = (!rc ? tmp : 0xff);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-border-color", &tmp);
	pinfo->lcdc.border_clr = (!rc ? tmp : 0);
	data = of_get_property(np, "qcom,mdss-dsi-panel-orientation", NULL);
	if (data) {
		pr_debug("panel orientation is %s\n", data);
		if (!strcmp(data, "180"))
			pinfo->panel_orientation = MDP_ROT_180;
		else if (!strcmp(data, "hflip"))
			pinfo->panel_orientation = MDP_FLIP_LR;
		else if (!strcmp(data, "vflip"))
			pinfo->panel_orientation = MDP_FLIP_UD;
	}

	ctrl_pdata->bklt_ctrl = UNKNOWN_CTRL;
	data = of_get_property(np, "qcom,mdss-dsi-bl-pmic-control-type", NULL);
	if (data) {
		if (!strncmp(data, "bl_ctrl_wled", 12)) {
			led_trigger_register_simple("bkl-trigger",
				&bl_led_trigger);
			pr_debug("%s: SUCCESS-> WLED TRIGGER register\n",
				__func__);
			ctrl_pdata->bklt_ctrl = BL_WLED;
		} else if (!strncmp(data, "bl_ctrl_pwm", 11)) {
			ctrl_pdata->bklt_ctrl = BL_PWM;
			ctrl_pdata->pwm_pmi = of_property_read_bool(np,
					"qcom,mdss-dsi-bl-pwm-pmi");
			rc = of_property_read_u32(np,
				"qcom,mdss-dsi-bl-pmic-pwm-frequency", &tmp);
			if (rc) {
				pr_err("%s:%d, Error, panel pwm_period\n",
						__func__, __LINE__);
				return -EINVAL;
			}
			ctrl_pdata->pwm_period = tmp;
			if (ctrl_pdata->pwm_pmi) {
				ctrl_pdata->pwm_bl = of_pwm_get(np, NULL);
				if (IS_ERR(ctrl_pdata->pwm_bl)) {
					pr_err("%s: Error, pwm device\n",
								__func__);
					ctrl_pdata->pwm_bl = NULL;
					return -EINVAL;
				}
			} else {
				rc = of_property_read_u32(np,
					"qcom,mdss-dsi-bl-pmic-bank-select",
								 &tmp);
				if (rc) {
					pr_err("%s:%d, Error, lpg channel\n",
							__func__, __LINE__);
					return -EINVAL;
				}
				ctrl_pdata->pwm_lpg_chan = tmp;
				tmp = of_get_named_gpio(np,
					"qcom,mdss-dsi-pwm-gpio", 0);
				ctrl_pdata->pwm_pmic_gpio = tmp;
				pr_debug("%s: Configured PWM bklt ctrl\n",
								 __func__);
			}
		} else if (!strncmp(data, "bl_ctrl_dcs", 11)) {
			ctrl_pdata->bklt_ctrl = BL_DCS_CMD;
			pr_debug("%s: Configured DCS_CMD bklt ctrl\n",
								__func__);
		}
	}
	rc = of_property_read_u32(np, "qcom,mdss-brightness-max-level", &tmp);
	pinfo->brightness_max = (!rc ? tmp : MDSS_MAX_BL_BRIGHTNESS);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-min-level", &tmp);
	pinfo->bl_min = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-max-level", &tmp);
	pinfo->bl_max = (!rc ? tmp : 255);
	ctrl_pdata->bklt_max = pinfo->bl_max;

	rc = of_property_read_u32(np, "qcom,mdss-dsi-interleave-mode", &tmp);
	pinfo->mipi.interleave_mode = (!rc ? tmp : 0);

	pinfo->mipi.vsync_enable = of_property_read_bool(np,
		"qcom,mdss-dsi-te-check-enable");
	pinfo->mipi.hw_vsync_mode = of_property_read_bool(np,
		"qcom,mdss-dsi-te-using-te-pin");

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-h-sync-pulse", &tmp);
	pinfo->mipi.pulse_mode_hsa_he = (!rc ? tmp : false);

	pinfo->mipi.hfp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hfp-power-mode");
	pinfo->mipi.hsa_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hsa-power-mode");
	pinfo->mipi.hbp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hbp-power-mode");
	pinfo->mipi.last_line_interleave_en = of_property_read_bool(np,
		"qcom,mdss-dsi-last-line-interleave");
	pinfo->mipi.bllp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-bllp-power-mode");
	pinfo->mipi.eof_bllp_power_stop = of_property_read_bool(
		np, "qcom,mdss-dsi-bllp-eof-power-mode");
	pinfo->mipi.traffic_mode = DSI_NON_BURST_SYNCH_PULSE;
	data = of_get_property(np, "qcom,mdss-dsi-traffic-mode", NULL);
	if (data) {
		if (!strcmp(data, "non_burst_sync_event"))
			pinfo->mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;
		else if (!strcmp(data, "burst_mode"))
			pinfo->mipi.traffic_mode = DSI_BURST_MODE;
	}
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-dcs-command", &tmp);
	pinfo->mipi.insert_dcs_cmd =
			(!rc ? tmp : 1);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-wr-mem-continue", &tmp);
	pinfo->mipi.wr_mem_continue =
			(!rc ? tmp : 0x3c);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-wr-mem-start", &tmp);
	pinfo->mipi.wr_mem_start =
			(!rc ? tmp : 0x2c);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-pin-select", &tmp);
	pinfo->mipi.te_sel =
			(!rc ? tmp : 1);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-virtual-channel-id", &tmp);
	pinfo->mipi.vc = (!rc ? tmp : 0);
	pinfo->mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	data = of_get_property(np, "qcom,mdss-dsi-color-order", NULL);
	if (data) {
		if (!strcmp(data, "rgb_swap_rbg"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_RBG;
		else if (!strcmp(data, "rgb_swap_bgr"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_BGR;
		else if (!strcmp(data, "rgb_swap_brg"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_BRG;
		else if (!strcmp(data, "rgb_swap_grb"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_GRB;
		else if (!strcmp(data, "rgb_swap_gbr"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_GBR;
	}
	pinfo->mipi.data_lane0 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-0-state");
	pinfo->mipi.data_lane1 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-1-state");
	pinfo->mipi.data_lane2 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-2-state");
	pinfo->mipi.data_lane3 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-3-state");

	pinfo->mipi.rx_eot_ignore = of_property_read_bool(np,
		"qcom,mdss-dsi-rx-eot-ignore");
	pinfo->mipi.tx_eot_append = of_property_read_bool(np,
		"qcom,mdss-dsi-tx-eot-append");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-stream", &tmp);
	pinfo->mipi.stream = (!rc ? tmp : 0);

	data = of_get_property(np, "qcom,mdss-dsi-panel-mode-gpio-state", NULL);
	if (data) {
		if (!strcmp(data, "high"))
			pinfo->mode_gpio_state = MODE_GPIO_HIGH;
		else if (!strcmp(data, "low"))
			pinfo->mode_gpio_state = MODE_GPIO_LOW;
	} else {
		pinfo->mode_gpio_state = MODE_GPIO_NOT_VALID;
	}

	rc = of_property_read_u32(np, "qcom,mdss-mdp-transfer-time-us", &tmp);
	pinfo->mdp_transfer_time_us = (!rc ? tmp : DEFAULT_MDP_TRANSFER_TIME);

	pinfo->mipi.lp11_init = of_property_read_bool(np,
					"qcom,mdss-dsi-lp11-init");
	rc = of_property_read_u32(np, "qcom,mdss-dsi-init-delay-us", &tmp);
	pinfo->mipi.init_delay = (!rc ? tmp : 0);

	mdss_dsi_parse_roi_alignment(np, pinfo);

	mdss_dsi_parse_trigger(np, &(pinfo->mipi.mdp_trigger),
		"qcom,mdss-dsi-mdp-trigger");

	mdss_dsi_parse_trigger(np, &(pinfo->mipi.dma_trigger),
		"qcom,mdss-dsi-dma-trigger");

	mdss_dsi_parse_lane_swap(np, &(pinfo->mipi.dlane_swap));

	mdss_dsi_parse_reset_seq(np, pinfo->rst_seq, &(pinfo->rst_seq_len),
		"qcom,mdss-dsi-reset-sequence");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->off_cmds,
		"qcom,mdss-dsi-off-command", "qcom,mdss-dsi-off-command-state");

	pinfo->mipi.force_clk_lane_hs = of_property_read_bool(np,
		"qcom,mdss-dsi-force-clock-lane-hs");

#ifdef CONFIG_BBRY
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->on_recovery_cmds,
		"oem,mdss-dsi-on-recovery-command", "oem,mdss-dsi-on-recovery-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->glass_on_cmds,
		"oem,mdss-dsi-glass-on-command",
		"oem,mdss-dsi-glass-on-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->on_to_lpm_cmds,
		"oem,mdss-dsi-on-to-lpm-command",
		"oem,mdss-dsi-on-to-lpm-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lpm_to_on_cmds,
		"oem,mdss-dsi-lpm-to-on-command",
		"oem,mdss-dsi-lpm-to-on-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->read_serial_cmds,
		"oem,mdss-dsi-read-serial-command",
		NULL);
	pinfo->read_serial_id_bytes = of_get_property(np, "oem,mdss-dsi-read-serial-id-bytes", &pinfo->serial_id_length);

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->partial_window_en_cmds,
		"oem,mdss-dsi-partial-window-en-command",
		"oem,mdss-dsi-partial-window-en-command-state");
	ctrl_pdata->partial_window_sr_bytes = of_get_property(np, "oem,mdss-dsi-partial-window-sr-bytes", &ctrl_pdata->partial_window_sr_bytes_length);
	ctrl_pdata->partial_window_er_bytes = of_get_property(np, "oem,mdss-dsi-partial-window-er-bytes", &ctrl_pdata->partial_window_er_bytes_length);
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->partial_window_dis_cmds,
		"oem,mdss-dsi-partial-window-dis-command",
		"oem,mdss-dsi-partial-window-dis-command-state");

	mdss_panel_parse_panel_pcc(np, pinfo);
	mdss_panel_parse_gamut_map(np, pinfo);
	mdss_panel_parse_panel_color_points(np, pinfo);
#endif

	rc = mdss_dsi_parse_panel_features(np, ctrl_pdata);
	if (rc) {
		pr_err("%s: failed to parse panel features\n", __func__);
		goto error;
	}

	mdss_dsi_parse_panel_horizintal_line_idle(np, ctrl_pdata);

	mdss_dsi_parse_dfps_config(np, ctrl_pdata);

	return 0;

error:
	return -EINVAL;
}

int mdss_dsi_panel_init(struct device_node *node,
	struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	bool cmd_cfg_cont_splash)
{
	int rc = 0;
	static const char *panel_name;
	struct mdss_panel_info *pinfo;

	if (!node || !ctrl_pdata) {
		pr_err("%s: Invalid arguments\n", __func__);
		return -ENODEV;
	}

	pinfo = &ctrl_pdata->panel_data.panel_info;

	pr_debug("%s:%d\n", __func__, __LINE__);
	pinfo->panel_name[0] = '\0';
	panel_name = of_get_property(node, "qcom,mdss-dsi-panel-name", NULL);
	if (!panel_name) {
		pr_info("%s:%d, Panel name not specified\n",
						__func__, __LINE__);
	} else {
		pr_info("%s: Panel Name = %s\n", __func__, panel_name);
		strlcpy(&pinfo->panel_name[0], panel_name, MDSS_MAX_PANEL_LEN);
	}
	rc = mdss_panel_parse_dt(node, ctrl_pdata);
	if (rc) {
		pr_err("%s:%d panel dt parse failed\n", __func__, __LINE__);
		return rc;
	}

	if (!cmd_cfg_cont_splash)
		pinfo->cont_splash_enabled = false;
	pr_info("%s: Continuous splash %s\n", __func__,
		pinfo->cont_splash_enabled ? "enabled" : "disabled");

	pinfo->dynamic_switch_pending = false;
	pinfo->is_lpm_mode = false;
	pinfo->esd_rdy = false;

	ctrl_pdata->on = mdss_dsi_panel_on;
	ctrl_pdata->off = mdss_dsi_panel_off;
#ifdef CONFIG_BBRY
	ctrl_pdata->glass_on = mdss_dsi_glass_on;
	ctrl_pdata->panel_data.panel_info.set_partial_window = mdss_dsi_set_partial_window;
	ctrl_pdata->panel_data.panel_info.read_serial_id = mdss_dsi_read_serial_id;
#endif

#ifdef CONFIG_BBRY_MFG
	if (!strncmp(panel_name, "Logan", strlen("Logan"))) {
		ctrl_pdata->panel_data.panel_info.read_recovery_data = logan_read_recovery_data;
	}
#endif
	ctrl_pdata->low_power_config = mdss_dsi_panel_low_power_config;
	ctrl_pdata->panel_data.set_backlight = mdss_dsi_panel_bl_ctrl;
	ctrl_pdata->switch_mode = mdss_dsi_panel_switch_mode;

	return 0;
}
