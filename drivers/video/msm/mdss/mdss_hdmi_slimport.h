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

#ifndef __MDSS_HDMI_SLIMPORT_H__
#define __MDSS_HDMI_SLIMPORT_H__

#include <linux/platform_device.h>

struct msm_hdmi_slimport_ops {
	u8 (*out_tmds_enabled)(struct platform_device *pdev);
	int (*out_set_slimport_max_pclk)(struct platform_device *pdev, u32 max_val);
	int (*out_set_upstream_hpd)(struct platform_device *pdev, uint8_t on);
	int (*in_read_edid_block)(int block, uint8_t *edid_buf);
};

int msm_hdmi_register_slimport(struct platform_device *pdev,
			  struct msm_hdmi_slimport_ops *ops, void *data);

int hdmi_tx_slimport_enable_hpd(struct platform_device *pdev, int on);

#endif /* __MDSS_HDMI_SLIMPORT_H__*/
