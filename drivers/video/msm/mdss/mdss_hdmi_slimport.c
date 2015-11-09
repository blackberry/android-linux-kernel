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

#include "mdss_panel.h"
#include "mdss_hdmi_tx.h"
#include "mdss_hdmi_slimport.h"


static int hdmi_tx_set_slimport_hpd(struct platform_device *pdev, uint8_t on)
{
	int rc = 0;
	struct hdmi_tx_ctrl *hdmi_ctrl = NULL;

	hdmi_ctrl = platform_get_drvdata(pdev);

	if (!hdmi_ctrl) {
		DEV_ERR("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	/* slimport status should override */
	hdmi_ctrl->mhl_hpd_on = on;

	if (!on && hdmi_ctrl->hpd_feature_on) {
		rc = hdmi_tx_slimport_enable_hpd(pdev, false);
	} else if (on && !hdmi_ctrl->hpd_feature_on) {
		rc = hdmi_tx_slimport_enable_hpd(pdev, true);
	} else {
		DEV_DBG("%s: hpd is already '%s'. return\n", __func__,
			hdmi_ctrl->hpd_feature_on ? "enabled" : "disabled");
		return rc;
	}

	if (!rc) {
		hdmi_ctrl->hpd_feature_on =
			(~hdmi_ctrl->hpd_feature_on) & BIT(0);
		DEV_DBG("%s: '%d'\n", __func__, hdmi_ctrl->hpd_feature_on);
	} else {
		DEV_ERR("%s: failed to '%s' hpd. rc = %d\n", __func__,
			on ? "enable" : "disable", rc);
	}

	return rc;

}

static int hdmi_tx_set_slimport_max_pclk(struct platform_device *pdev,
		u32 max_val)
{
	struct hdmi_tx_ctrl *hdmi_ctrl = NULL;

	hdmi_ctrl = platform_get_drvdata(pdev);

	if (!hdmi_ctrl) {
		DEV_ERR("%s: invalid input\n", __func__);
		return -ENODEV;
	}

	if (!max_val) {
		DEV_ERR("%s: invalid max pclk val\n", __func__);
		return -EINVAL;
	}

	DEV_INFO("%s: max pclk set to [%u]\n",
		__func__, max_val);
	/* Check that this correctly removes modes from the edid, including if this is called multiple times */
	hdmi_ctrl->ds_data.ds_max_clk = max_val;
	hdmi_ctrl->ds_data.ds_registered = true;

	return 0;
} /* hdmi_tx_set_slimport_max_pclk */

int msm_hdmi_register_slimport(struct platform_device *pdev,
			  struct msm_hdmi_slimport_ops *ops, void *data)
{
	struct hdmi_tx_ctrl *hdmi_ctrl = platform_get_drvdata(pdev);

	if (!hdmi_ctrl) {
		DEV_ERR("%s: invalid pdev\n", __func__);
		return -ENODEV;
	}

	if (hdmi_ctrl->pdev != pdev) {
		DEV_ERR("%s: invalid drvdata: %p != %p",
				__func__, hdmi_ctrl->pdev, pdev);
		return -ENODEV;
	}

	if (!ops) {
		DEV_ERR("%s: invalid ops\n", __func__);
		return -EINVAL;
	}

	ops->out_set_slimport_max_pclk = hdmi_tx_set_slimport_max_pclk;
	ops->out_set_upstream_hpd = hdmi_tx_set_slimport_hpd;

	hdmi_ctrl->ds_read_edid_block = ops->in_read_edid_block;
	hdmi_ctrl->ds_registered = true;

	return 0;
}
