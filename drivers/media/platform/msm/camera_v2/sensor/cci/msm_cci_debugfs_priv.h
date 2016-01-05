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

#include <linux/debugfs.h>

#include "msm_cci.h"

/* Log buffer */
struct msm_cci_log_buffer {
	size_t rpos;
	size_t wpos;
	size_t len;
	char data[0];
};

/* MSM CCI device specific data */
struct msm_cci_dev_data {
	u8 slave;
	u8 addr_type;
	u16 reg;
	u32 count;
	struct dentry *dir;
	struct list_head node;
	enum cci_i2c_master_t master;
};

struct msm_cci_trans {
	/* Files within the hiearchy */
	u8 slave;
	u8 addr_type;
	u16 reg;
	u32 count;
	u32 offset;

	/* bool to determine if we are looking for formatted or raw output*/
	bool is_raw;

	/* master id */
	enum cci_i2c_master_t master;

	/* log to hold data received from read */
	struct msm_cci_log_buffer *log;

	/* CCI controller */
	struct msm_camera_cci_client cci_client;
	struct msm_camera_cci_ctrl cci_ctrl;
};

struct msm_cci_dbgfs {
	struct dentry *root;
	struct v4l2_subdev *sd;
	struct mutex lock;
	struct list_head dev_node; /* List of msm_cci_dev_data nodes */
	struct debugfs_blob_wrapper help_msg;

	/*
	 * Function pointer to CCI driver config function. This function will
	 * handle the calling of the correct function (init/read/write/release)
	 */
	int32_t (*config)(struct v4l2_subdev *sd,
		struct msm_camera_cci_ctrl *cci_ctrl);
};
