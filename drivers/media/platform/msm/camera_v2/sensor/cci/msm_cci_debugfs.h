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

#ifndef _MSM_CCI_DEBUGFS_H
#define _MSM_CCI_DEBUGFS_H

#include <linux/debugfs.h>

#ifdef CONFIG_DEBUG_FS
int msm_cci_enable_debugfs(struct cci_device *dev,
	int32_t (*msm_cci_config)(struct v4l2_subdev *sd,
		struct msm_camera_cci_ctrl *cci_ctrl));
#else
inline int msm_cci_enable_debugfs(struct cci_device *dev,
	int32_t (*msm_cci_config)(struct v4l2_subdev *sd,
		struct msm_camera_cci_ctrl *cci_ctrl))
{
	return 0;
}
#endif /* CONFIG_DEBUG_FS */
#endif /* _MSM_CCI_DEBUGFS_H */
