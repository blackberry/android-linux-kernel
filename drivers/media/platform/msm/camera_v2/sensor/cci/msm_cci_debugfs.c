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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/ctype.h>

#include "msm_cci.h"
#include "msm_cci_debugfs.h"
#include "msm_cci_debugfs_priv.h"

#define SLAVE_LEN 3 /* 1 byte address + 1 space character */
#define REG_LEN 5 /* 2 byte address + 1 space character */
#define CHARS_PER_ITEM 3 /* Format is 'XX ' */
#define ITEMS_PER_LINE 16 /* 16 data items per line */
#define MAX_LINE_LENGTH (SLAVE_LEN + REG_LEN + \
			(ITEMS_PER_LINE * CHARS_PER_ITEM) + 1)

static const char *DFS_ROOT_NAME = "msm_cci";
static const mode_t DFS_MODE = S_IRUSR | S_IWUSR;

static struct msm_cci_dbgfs dbgfs_data = {
	.lock = __MUTEX_INITIALIZER(dbgfs_data.lock),
	.dev_node = LIST_HEAD_INIT(dbgfs_data.dev_node),
	.help_msg = {
		.data =
		"MSM CCI Debug-FS support\n"
		"\n"
		"Folder Hierarchy\n"
		"/sys/kernel/debug/msm_cci\n"
		"    /help          -- Static help text\n"
		"    /msm_cci-N     -- Directory for MSM CCI N\n"
		"        /count     -- Number of registers to read\n"
		"        /data      -- Initiates the CCI read (formatted)\n"
		"        /data_raw  -- Initiates the CCI raw read or write\n"
		"        /addr_type -- Address type to read or write\n"
		"        /slave     -- Slave address for reads and writes\n"
		"        /register  -- Register address to read and write\n"
		"\n"
		"To perform CCI read or write transactions, you need to write the\n"
		"slave address to 'slave' file, the register address to 'register'\n"
		"file, and the address type to 'addr_type'. When reading, the number\n"
		"bytes to be read must be written to 'count' file.\n"
		"\n"
		"Reading from the 'data' file will initiate a CCI read transaction\n"
		"starting from slave register 'register' for 'count' number of bytes.\n"
		"\n"
		"Writing to the 'data' file will initiate a CCI write transaction\n"
		"starting from slave register 'register'. The number of registers\n"
		"written to will match the number of bytes written to 'data' file.\n"
		"\n"
		"Example: Read 8 bytes starting at register address 0xF8 for SID 0x48\n"
		"This will print the HW and FW version information.\n"
		"\n"
		"cd msm_cci-1\n"
		"echo 0x48 > slave\n"
		"echo 0xF8 > register\n"
		"echo 2 > addr_type\n"
		"echo 8 > count\n"
		"cat data\n"
		"\n"
		"Example: Write 2 bytes starting at register address 0x00 for SID 0x18\n"
		"This will set the AF position\n"
		"\n"
		"cd msm_cci-1\n"
		"echo 0x18 > slave\n"
		"echo 0x00 > register\n"
		"echo 1 > addr_type\n"
		"echo 0x3E80 > data\n"
		"\n",
	},
};

static int
msm_cci_dfs_open(struct msm_cci_dev_data *dev_data, struct file *file)
{
	struct msm_cci_log_buffer *log;
	struct msm_cci_trans *trans;
	struct msm_camera_cci_client *cci_client;
	struct msm_camera_cci_ctrl *cci_ctrl;

	size_t logbufsize = SZ_4K;

	if (!dev_data) {
		pr_err("No CCI device data\n");
		return -EINVAL;
	}

	trans = kzalloc(sizeof(*trans), GFP_KERNEL);
	if (!trans) {
		pr_err("Unable to allocate memory for transaction\n");
		return -ENOMEM;
	}

	/* allocate the log buffer */
	log = kzalloc(logbufsize, GFP_KERNEL);
	if (!log) {
		kfree(trans);
		pr_err("Unable to allocate memory for log buffer\n");
		return -ENOMEM;
	}

	log->rpos = 0;
	log->wpos = 0;
	log->len = logbufsize - sizeof(*log);

	/* setup the CCI controller and client object */
	cci_client = &(trans->cci_client);
	cci_client->cci_i2c_master = dev_data->master;
	cci_client->id_map = 0;
	cci_client->retries = 3;
	cci_client->sid = (dev_data->slave >> 1);

	cci_ctrl = &(trans->cci_ctrl);
	cci_ctrl->cmd = MSM_CCI_INIT;
	cci_ctrl->cci_info = cci_client;

	/* initialize CCI */
	dbgfs_data.config(dbgfs_data.sd, cci_ctrl);

	trans->log = log;
	trans->count = dev_data->count;
	trans->master = dev_data->master;
	trans->addr_type = dev_data->addr_type;
	trans->slave = dev_data->slave;
	trans->reg = dev_data->reg;
	trans->offset = dev_data->reg;

	file->private_data = trans;
	return 0;
}

static int
msm_cci_open_data(struct inode *inode, struct file *file)
{
	struct msm_cci_dev_data *dev_data = inode->i_private;
	return msm_cci_dfs_open(dev_data, file);
}

static int
msm_cci_open_data_raw(struct inode *inode, struct file *file)
{
	int rc;
	struct msm_cci_trans *trans;
	struct msm_cci_dev_data *dev_data = inode->i_private;

	rc = msm_cci_dfs_open(dev_data, file);
	trans = file->private_data;
	trans->is_raw = true;
	return rc;
}

static int
msm_cci_dfs_close(struct inode *inode, struct file *file)
{
	struct msm_cci_trans *trans = file->private_data;
	struct msm_camera_cci_ctrl *cci_ctrl = &(trans->cci_ctrl);

	/* release cci */
	cci_ctrl->cmd = MSM_CCI_RELEASE;
	dbgfs_data.config(dbgfs_data.sd, cci_ctrl);

	/* cleanup */
	if (trans && trans->log) {
		file->private_data = NULL;
		kfree(trans->log);
		kfree(trans);
	}

	return 0;
}

static int
msm_cci_write_data(struct msm_camera_cci_ctrl *cci_ctrl, uint8_t *buf,
	uint8_t slave, uint32_t reg, uint8_t addr_type, int num_byte)
{
	int i, ret = 0;
	struct msm_camera_i2c_reg_array reg_conf_tbl[num_byte];

	/* configure write message */
	memset(reg_conf_tbl, 0,
			num_byte * sizeof(struct msm_camera_i2c_reg_array));
	reg_conf_tbl[0].reg_addr = reg;
	for (i = 0; i < num_byte; ++i) {
		reg_conf_tbl[i].reg_data = buf[i];
		reg_conf_tbl[i].delay = 0;
	}
	cci_ctrl->cfg.cci_i2c_write_cfg.reg_setting = reg_conf_tbl;
	cci_ctrl->cfg.cci_i2c_write_cfg.data_type = MSM_CAMERA_I2C_BYTE_DATA;
	cci_ctrl->cfg.cci_i2c_write_cfg.addr_type = addr_type;
	cci_ctrl->cfg.cci_i2c_write_cfg.size = num_byte;

	cci_ctrl->cmd = MSM_CCI_I2C_WRITE_SEQ;

	/* send write command */
	ret = dbgfs_data.config(dbgfs_data.sd, cci_ctrl);
	if (ret < 0) {
		pr_err("CCI write failed, sid=0x%2.2X; reg=0x%4.4X; err = %d\n",
			slave, reg, ret);
		goto end;
	}

end:
	return ret;
}

static ssize_t
msm_cci_dfs_write(struct file *file, const char __user *buf,
	size_t count, loff_t *ppos)
{
	int bytes_read;
	int data;
	int pos = 0;
	int num_bytes = 0;
	u8 *values;
	char *kbuf;
	size_t ret = 0;

	struct msm_cci_trans *trans = file->private_data;
	struct msm_camera_cci_ctrl *cci_ctrl = &(trans->cci_ctrl);
	u8 slave = trans->slave;
	u8 type = trans->addr_type;
	u32 reg = trans->reg;

	kbuf = kmalloc(count + 1, GFP_KERNEL);

	if (!kbuf)
		return -ENOMEM;

	ret = copy_from_user(kbuf, buf, count);
	if (ret == count) {
		pr_err("failed to copy data from user\n");
		ret = -EFAULT;
		goto free_buf;
	}

	count -= ret;
	*ppos += count;
	kbuf[count] = '\0';

	values = kbuf;

	while (sscanf(kbuf + pos, "%i%n", &data, &bytes_read) == 1) {
		pos += bytes_read;
		values[num_bytes++] = data & 0xff;
	}

	if (!num_bytes)
		goto free_buf;


	ret = msm_cci_write_data(cci_ctrl, values, slave, reg, type, num_bytes);

	if (ret)
		pr_err("CCI write failed, err = %zu\n", ret);
	else {
		ret = count;
		trans->offset += num_bytes;
	}

free_buf:
	kfree(kbuf);
	return ret;
}

static int
msm_cci_read_data(struct msm_camera_cci_ctrl *cci_ctrl , uint8_t *buf,
	uint8_t slave, uint32_t reg, uint8_t addr_type, int num_byte)
{
	int ret = 0;

	/* Setup the read call */
	cci_ctrl->cfg.cci_i2c_read_cfg.data = buf;
	cci_ctrl->cfg.cci_i2c_read_cfg.num_byte = num_byte;
	cci_ctrl->cfg.cci_i2c_read_cfg.addr = reg;
	cci_ctrl->cfg.cci_i2c_read_cfg.addr_type = addr_type;

	cci_ctrl->cmd = MSM_CCI_I2C_READ;

	/* send read command */
	ret = dbgfs_data.config(dbgfs_data.sd, cci_ctrl);
	if (ret < 0)
		pr_err("CCI read failed, sid=0x%2.2X; reg=0x%4.4X; err = %d\n",
			slave, reg, ret);

	return ret;
}

static int
print_to_log(struct msm_cci_log_buffer *log, const char *fmt, ...)
{
	va_list args;
	int count;
	char *buf = &log->data[log->wpos];
	size_t size = log->len - log->wpos;

	va_start(args, fmt);
	count = vscnprintf(buf, size, fmt, args);
	va_end(args);

	log->wpos += count;
	return count;
}

static int
write_to_log_raw(struct msm_cci_trans *trans, int offset, size_t *pcount)
{
	u8 data[ITEMS_PER_LINE];
	int i;
	int count = 0;
	int items_to_read = min(ARRAY_SIZE(data), *pcount);

	u8 slave = trans->slave;
	u8 type = trans->addr_type;
	struct msm_camera_cci_ctrl *cci_ctrl = &(trans->cci_ctrl);
	struct msm_cci_log_buffer *log = trans->log;

	if ((log->len - log->wpos) < MAX_LINE_LENGTH)
		goto end;

	/* Read the desired number of 'items' */
	if (msm_cci_read_data(cci_ctrl, data, slave, offset, type, items_to_read))
		goto end;

	*pcount -= items_to_read;

	for (i = 0; i < items_to_read; ++i) {
		count = print_to_log(log, "0x%2.2X ", data[i]);
		if (count == 0)
			goto end;
	}

	if (log->wpos > 0 && log->data[log->wpos - 1] == ' ')
		log->data[log->wpos - 1] = '\n';

end:
	return count;
}

static int
write_to_log_format(struct msm_cci_trans *trans, int offset, size_t *pcount)
{
	int i, j;
	u8 data[ITEMS_PER_LINE];
	struct msm_cci_log_buffer *log = trans->log;

	int count = 0;
	int padding = offset % ITEMS_PER_LINE;
	int items_to_read = min(ARRAY_SIZE(data) - padding, *pcount);
	int items_to_log = min(ITEMS_PER_LINE, padding + items_to_read);

	u8 slave = trans->slave;
	u8 type = trans->addr_type;
	struct msm_camera_cci_ctrl *cci_ctrl = &(trans->cci_ctrl);

	if ((log->len - log->wpos) < MAX_LINE_LENGTH)
		goto end;


	/* Read the desired number of 'items' */
	if (msm_cci_read_data(cci_ctrl, data, slave, offset, type, items_to_read))
		goto end;

	*pcount -= items_to_read;

	/* print out slave address */
	count = print_to_log(log, "%2.2X ", slave & 0xff);
	if (count == 0)
		goto end;

	/* print base register address */
	count = print_to_log(log, "%4.4X ", offset & 0xfff0);
	if (count == 0)
		goto end;

	/* print required padding */
	for (i = 0; i < padding; ++i) {
		count = print_to_log(log, "-- ");
		if (count == 0)
			goto end;
	}

	/* print values read */
	for (j = 0; i < items_to_log; ++i, ++j) {
		count = print_to_log(log, "%2.2X ", data[j]);
		if (count == 0)
			goto end;
	}

	/* add a newline if last character is a space */
	if (log->wpos > 0 && log->data[log->wpos - 1] == ' ')
		log->data[log->wpos - 1] = '\n';

end:
	return count;
}

static int
get_log_data(struct msm_cci_trans *trans)
{
	int count;
	int last_count;
	int items_read;
	int total_items_read = 0;
	u32 offset = trans->offset;
	size_t item_count = trans->count;
	struct msm_cci_log_buffer *log = trans->log;
	int (*write_to_log)(struct msm_cci_trans *, int, size_t *);

	if (item_count == 0)
		return 0;

	/* determine which log function to use */
	if (trans->is_raw)
		write_to_log = write_to_log_raw;
	else
		write_to_log = write_to_log_format;


	log->wpos = log->rpos = 0;

	do {
		last_count = item_count;
		count = write_to_log(trans, offset, &item_count);
		items_read = last_count - item_count;
		offset += items_read;
		total_items_read += items_read;
	} while (count && item_count > 0);

	trans->count = item_count;
	trans->offset += total_items_read;

	return total_items_read;
}

static ssize_t
msm_cci_dfs_read(struct file *file, char __user *buf,
	size_t count, loff_t *ppos)
{
	struct msm_cci_trans *trans = file->private_data;
	struct msm_cci_log_buffer *log = trans->log;
	size_t ret;
	size_t len;

	if (log->rpos >= log->wpos)
		if (get_log_data(trans) <= 0)
			return 0;

	len = min(count, log->wpos - log->rpos);
	ret = copy_to_user(buf, &log->data[log->rpos], len);
	if (ret == len)
		pr_err("error copying CCI register values to user\n");

	len -= ret;
	*ppos += len;
	log->rpos = len;

	return len;
}

static const struct file_operations msm_cci_dfs_fops = {
	.open    = msm_cci_open_data,
	.release = msm_cci_dfs_close,
	.read    = msm_cci_dfs_read,
	.write   = msm_cci_dfs_write,
};

static const struct file_operations msm_cci_dfs_raw_fops = {
	.open    = msm_cci_open_data_raw,
	.release = msm_cci_dfs_close,
	.read    = msm_cci_dfs_read,
	.write   = msm_cci_dfs_write,
};

static struct dentry *
msm_cci_dfs_create_fs(void)
{
	struct dentry *root, *file;

	pr_debug("Creating CCI debugfs file-system\n");
	root = debugfs_create_dir(DFS_ROOT_NAME, NULL);
	if (IS_ERR_OR_NULL(root)) {
		pr_err("Error creating top level directory err:%ld",
			(long)root);
		if (PTR_ERR(root) == -ENODEV)
			pr_err("debugfs is not enabled in the kernel");

		return NULL;
	}

	/* create a debugfs blob for the help message */
	dbgfs_data.help_msg.size = strlen(dbgfs_data.help_msg.data);
	file = debugfs_create_blob("help", S_IRUGO, root, &dbgfs_data.help_msg);
	if (!file) {
		pr_err("error creating help entry\n");
		goto err_remove_fs;
	}
	return root;

err_remove_fs:
	debugfs_remove_recursive(root);
	return NULL;
}

static struct dentry *
msm_cci_get_root(void)
{
	if (dbgfs_data.root)
		return dbgfs_data.root;

	if (mutex_lock_interruptible(&dbgfs_data.lock) < 0)
		return NULL;

	if (!dbgfs_data.root) /* double checking idiom */
		dbgfs_data.root = msm_cci_dfs_create_fs();

	mutex_unlock(&dbgfs_data.lock);
	return dbgfs_data.root;
}

static struct msm_cci_dev_data *
msm_cci_add_device(struct cci_device *dev, int master)
{
	char line[32];
	struct msm_cci_dev_data *dev_data;
	struct dentry *dir;
	struct dentry *file;

	dev_data = kzalloc(sizeof(*dev_data), GFP_KERNEL);
	if (!dev_data)
		goto err_exit;

	snprintf(line, sizeof(line), "msm_cci-%d", master);
	dir = debugfs_create_dir(line, dbgfs_data.root);
	if (!dir)
		goto err_create_dir_failed;

	dev_data->count = 1;
	dev_data->dir = dir;
	dev_data->master = master;

	file = debugfs_create_u8("addr_type", DFS_MODE, dir, &dev_data->addr_type);
	if (!file) {
		pr_err("error creating 'addr_type' entry\n");
		goto err_remove_fs;
	}

	file = debugfs_create_u32("count", DFS_MODE, dir, &dev_data->count);
	if (!file) {
		pr_err("error creating 'count' entry\n");
		goto err_remove_fs;
	}

	file = debugfs_create_x8("slave", DFS_MODE, dir, &dev_data->slave);
	if (!file) {
		pr_err("error creating 'slave' entry\n");
		goto err_remove_fs;
	}

	file = debugfs_create_x16("register", DFS_MODE, dir, &dev_data->reg);
	if (!file) {
		pr_err("error creating 'register' entry\n");
		goto err_remove_fs;
	}

	file = debugfs_create_file("data", DFS_MODE, dir, dev_data,
		&msm_cci_dfs_fops);
	if (!file) {
		pr_err("error creating 'data' entry\n");
		goto err_remove_fs;
	}

	file = debugfs_create_file("data_raw", DFS_MODE, dir, dev_data,
		&msm_cci_dfs_raw_fops);
	if (!file) {
		pr_err("error creating 'data_raw' entry\n");
		goto err_remove_fs;
	}
	return dev_data;

err_remove_fs:
	debugfs_remove_recursive(dir);
err_create_dir_failed:
	kfree(dev_data);
err_exit:
	return NULL;
}

int
msm_cci_enable_debugfs(struct cci_device *cci_dev,
	int32_t (*msm_cci_config)(struct v4l2_subdev *sd,
			struct msm_camera_cci_ctrl *cci_ctrl))
{
	struct dentry *root;
	struct msm_cci_dev_data *dev_data;
	int count;

	pr_debug("Enabling debugfs for cci_device %p\n", cci_dev);
	root = msm_cci_get_root();
	if (!root)
		return -ENOENT;

	for (count = 0; count < MASTER_MAX; count++) {
		dev_data = msm_cci_add_device(cci_dev, count);
		if (!dev_data)
			return -ENOMEM;

		list_add(&dev_data->node, &dbgfs_data.dev_node);
	}

	dbgfs_data.sd = &cci_dev->msm_sd.sd;
	dbgfs_data.config = msm_cci_config;

	return 0;
}

static void __exit
msm_cci_del_all_devices(struct list_head *head)
{
	struct list_head *pos, *tmp;

	list_for_each_safe(pos, tmp, head) {
		struct msm_cci_dev_data *dev_data;

		dev_data = list_entry(pos, struct msm_cci_dev_data, node);
		list_del(pos);
		kfree(dev_data);
	}
}

static void __exit
msm_cci_dfs_destroy(void)
{
	pr_debug("destroying cci debugfs ...");
	if (mutex_lock_interruptible(&dbgfs_data.lock) < 0)
		return;

	if (dbgfs_data.root) {
		debugfs_remove_recursive(dbgfs_data.root);
		dbgfs_data.root = NULL;
		dbgfs_data.config = NULL;
		msm_cci_del_all_devices(&dbgfs_data.dev_node);
	}
	mutex_unlock(&dbgfs_data.lock);
}

module_exit(msm_cci_dfs_destroy);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:msm_cci_debug_fs");
