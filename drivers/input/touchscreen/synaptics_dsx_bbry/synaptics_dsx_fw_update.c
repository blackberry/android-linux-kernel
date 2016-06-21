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
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <synaptics_dsx.h>
#include "synaptics_dsx_core.h"

#define FW_IMAGE_FOLDER "synaptics/"
#define FW_IMAGE_NAME "%s.img"

#define LOCKDOWN_FW_PRODUCT_FOLDER "%s/"

#define STARTUP_FW_UPDATE_DELAY_MS 1000 /* ms */
#define RESTART_DELAY_MS 1000
#define FORCE_UPDATE false
#define DO_LOCKDOWN false

#define MAX_IMAGE_NAME_LEN 256
#define MAX_FIRMWARE_ID_LEN 10

#define LOCKDOWN_OFFSET 0xb0
#define IMAGE_AREA_OFFSET 0x100

#define BOOTLOADER_ID_OFFSET 0
#define BLOCK_NUMBER_OFFSET 0

#define V5_PROPERTIES_OFFSET 2
#define V5_BLOCK_SIZE_OFFSET 3
#define V5_BLOCK_COUNT_OFFSET 5
#define V5_BLOCK_DATA_OFFSET 2

#define V6_PROPERTIES_OFFSET 1
#define V6_BLOCK_SIZE_OFFSET 2
#define V6_BLOCK_COUNT_OFFSET 3
#define V6_BLOCK_DATA_OFFSET 1
#define V6_FLASH_COMMAND_OFFSET 2
#define V6_FLASH_STATUS_OFFSET 3

#define LOCKDOWN_BLOCK_COUNT 5

#define REG_MAP (1 << 0)
#define UNLOCKED (1 << 1)
#define HAS_CONFIG_ID (1 << 2)
#define HAS_PERM_CONFIG (1 << 3)
#define HAS_BL_CONFIG (1 << 4)
#define HAS_DISP_CONFIG (1 << 5)
#define HAS_CTRL1 (1 << 6)

#define UI_CONFIG_AREA 0x00
#define PERM_CONFIG_AREA 0x01
#define BL_CONFIG_AREA 0x02
#define DISP_CONFIG_AREA 0x03

#define CMD_WRITE_FW_BLOCK 0x2
#define CMD_ERASE_ALL 0x3
#define CMD_WRITE_LOCKDOWN_BLOCK 0x4
#define CMD_READ_CONFIG_BLOCK 0x5
#define CMD_WRITE_CONFIG_BLOCK 0x6
#define CMD_ERASE_CONFIG 0x7
#define CMD_ERASE_BL_CONFIG 0x9
#define CMD_ERASE_DISP_CONFIG 0xa
#define CMD_ENABLE_FLASH_PROG 0xf

#define SLEEP_MODE_NORMAL (0x00)
#define SLEEP_MODE_SENSOR_SLEEP (0x01)
#define SLEEP_MODE_RESERVED0 (0x02)
#define SLEEP_MODE_RESERVED1 (0x03)

#define ENABLE_WAIT_MS (1 * 1000)
#define WRITE_WAIT_MS (3 * 1000)
#define ERASE_WAIT_MS (5 * 1000)

#define IDLE_WAIT_POLL_TIME_MS (100)
#define MIN_SLEEP_TIME_US 50
#define MAX_SLEEP_TIME_US 100

#define MAX_FW_UPGRADE_TRIES  (5)

enum _synaptics_fw_upgrade_state {
	fw_idle,
	fw_do_not_upgrade,
	fw_upgrade_complete,
	fw_upgrade_failed
};

static ssize_t fwu_sysfs_show_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t fwu_sysfs_store_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t fwu_sysfs_do_reflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

#ifdef CONFIG_BBRY_DEBUG
static ssize_t fwu_sysfs_write_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_read_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_config_area_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_image_name_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_image_size_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);
#endif /*CONFIG_BBRY_DEBUG*/

static ssize_t fwu_sysfs_block_size_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_firmware_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_configuration_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_perm_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_bl_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_disp_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

enum bl_version {
	V5 = 5,
	V6 = 6,
};

enum flash_area {
	AREA_ERR = -1,
	NONE = 0,
	UI_FIRMWARE = 1,
	CONFIG_AREA = 2,
};

enum update_mode {
	NORMAL = 1,
	FORCE = 2,
	LOCKDOWN = 8,
};

struct image_header {
	/* 0x00 - 0x0f */
	unsigned char checksum[4];
	unsigned char reserved_04;
	unsigned char reserved_05;
	unsigned char options_firmware_id:1;
	unsigned char options_bootloader:1;
	unsigned char options_reserved:6;
	unsigned char bootloader_version;
	unsigned char firmware_size[4];
	unsigned char config_size[4];
	/* 0x10 - 0x1f */
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE];
	unsigned char package_id[2];
	unsigned char package_id_revision[2];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
	/* 0x20 - 0x2f */
	unsigned char bootloader_addr[4];
	unsigned char bootloader_size[4];
	unsigned char ui_addr[4];
	unsigned char ui_size[4];
	/* 0x30 - 0x3f */
	unsigned char ds_id[16];
	/* 0x40 - 0x4f */
	unsigned char disp_config_addr[4];
	unsigned char disp_config_size[4];
	unsigned char reserved_48_4f[8];
	/* 0x50 - 0x53 */
	unsigned char firmware_id[4];
};

struct image_header_data {
	bool contains_firmware_id;
	bool contains_bootloader;
	bool contains_disp_config;
	unsigned int firmware_id;
	unsigned int checksum;
	unsigned int firmware_size;
	unsigned int config_size;
	unsigned int bootloader_size;
	unsigned int disp_config_offset;
	unsigned int disp_config_size;
	unsigned char bootloader_version;
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
};

struct pdt_properties {
	union {
		struct {
			unsigned char reserved_1:6;
			unsigned char has_bsr:1;
			unsigned char reserved_2:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f01_device_status {
	union {
		struct {
			unsigned char status_code:4;
			unsigned char reserved:2;
			unsigned char flash_prog:1;
			unsigned char unconfigured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f01_device_control {
	union {
		struct {
			unsigned char sleep_mode:2;
			unsigned char nosleep:1;
			unsigned char reserved:2;
			unsigned char charger_connected:1;
			unsigned char report_rate:1;
			unsigned char configured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_fwu_handle {
	enum bl_version bl_version;
	bool initialized;
	bool program_enabled;
	bool has_perm_config;
	bool has_bl_config;
	bool has_disp_config;
	bool force_update;
	bool in_flash_prog_mode;
	bool do_lockdown;
	bool update_on_suspend;
	enum _synaptics_fw_upgrade_state state;
	unsigned short fw_upgrade_cnt;
	unsigned int data_pos;
	unsigned int image_size;
	unsigned char *image_name;
	unsigned char *ext_data_source;
	unsigned char *read_config_buf;
	unsigned char intr_mask;
	unsigned char command;
	unsigned char bootloader_id[2];
	unsigned char flash_properties;
	unsigned char flash_status;
	unsigned char productinfo1;
	unsigned char productinfo2;
	unsigned char properties_off;
	unsigned char blk_size_off;
	unsigned char blk_count_off;
	unsigned char blk_data_off;
	unsigned char flash_cmd_off;
	unsigned char flash_status_off;
	unsigned short block_size;
	unsigned short fw_block_count;
	unsigned short config_block_count;
	unsigned short lockdown_block_count;
	unsigned short perm_config_block_count;
	unsigned short bl_config_block_count;
	unsigned short disp_config_block_count;
	unsigned short config_size;
	unsigned short config_area;
	char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	const unsigned char *firmware_data;
	const unsigned char *config_data;
	const unsigned char *disp_config_data;
	const unsigned char *lockdown_data;
	struct workqueue_struct *fwu_workqueue;
	struct delayed_work fwu_work;
	struct delayed_work restart_work;
	struct synaptics_rmi4_fn_desc f34_fd;
	struct synaptics_rmi4_data *rmi4_data;
	struct mutex mutex;
};

static struct bin_attribute dev_attr_data = {
	.attr = {
		.name = "data",
#ifdef CONFIG_BBRY_DEBUG
		.mode = (S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP),
#else
		.mode = S_IRUSR | S_IRGRP,
#endif
	},
	.size = 0,
	.read = fwu_sysfs_show_image,
	.write = fwu_sysfs_store_image,
};

static struct device_attribute attrs[] = {
	__ATTR(doreflash, (S_IWUSR | S_IWGRP),
			synaptics_rmi4_show_error,
			fwu_sysfs_do_reflash_store),
#ifdef CONFIG_BBRY_DEBUG
	__ATTR(writeconfig, (S_IWUSR | S_IWGRP),
			synaptics_rmi4_show_error,
			fwu_sysfs_write_config_store),
	__ATTR(readconfig, (S_IWUSR | S_IWGRP),
			synaptics_rmi4_show_error,
			fwu_sysfs_read_config_store),
	__ATTR(configarea, (S_IWUSR | S_IWGRP),
			synaptics_rmi4_show_error,
			fwu_sysfs_config_area_store),
	__ATTR(imagename, (S_IWUSR | S_IWGRP),
			synaptics_rmi4_show_error,
			fwu_sysfs_image_name_store),
	__ATTR(imagesize, (S_IWUSR | S_IWGRP),
			synaptics_rmi4_show_error,
			fwu_sysfs_image_size_store),
#endif /*CONFIG_BBRY_DEBUG*/
	__ATTR(blocksize, (S_IRUSR | S_IRGRP),
			fwu_sysfs_block_size_show,
			synaptics_rmi4_store_error),
	__ATTR(fwblockcount, (S_IRUSR | S_IRGRP),
			fwu_sysfs_firmware_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(configblockcount, (S_IRUSR | S_IRGRP),
			fwu_sysfs_configuration_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(permconfigblockcount, (S_IRUSR | S_IRGRP),
			fwu_sysfs_perm_config_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(blconfigblockcount, (S_IRUSR | S_IRGRP),
			fwu_sysfs_bl_config_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(dispconfigblockcount, (S_IRUSR | S_IRGRP),
			fwu_sysfs_disp_config_block_count_show,
			synaptics_rmi4_store_error),
};

DECLARE_COMPLETION(fwu_remove_complete);

static int fwu_do_reflash(struct synaptics_rmi4_fwu_handle *fwu);
static int fwu_do_write_config(struct synaptics_rmi4_fwu_handle *fwu);

static unsigned int le_to_uint(const unsigned char *ptr)
{
	return (unsigned int)ptr[0] +
			(unsigned int)ptr[1] * 0x100 +
			(unsigned int)ptr[2] * 0x10000 +
			(unsigned int)ptr[3] * 0x1000000;
}

static unsigned int be_to_uint(const unsigned char *ptr)
{
	return (unsigned int)ptr[3] +
			(unsigned int)ptr[2] * 0x100 +
			(unsigned int)ptr[1] * 0x10000 +
			(unsigned int)ptr[0] * 0x1000000;
}

static void parse_header(struct image_header_data *header,
		const unsigned char *fw_image)
{
	struct image_header *data = (struct image_header *)fw_image;

	header->checksum = le_to_uint(data->checksum);

	header->bootloader_version = data->bootloader_version;

	header->firmware_size = le_to_uint(data->firmware_size);

	header->config_size = le_to_uint(data->config_size);

	memcpy(header->product_id, data->product_id, sizeof(data->product_id));
	header->product_id[sizeof(data->product_id)] = 0;

	memcpy(header->product_info, data->product_info,
			sizeof(data->product_info));

	header->contains_firmware_id = data->options_firmware_id;
	if (header->contains_firmware_id)
		header->firmware_id = le_to_uint(data->firmware_id);

	header->contains_bootloader = data->options_bootloader;
	if (header->contains_bootloader)
		header->bootloader_size = le_to_uint(data->bootloader_size);

	if ((header->bootloader_version == V5) && header->contains_bootloader) {
		header->contains_disp_config = true;
		header->disp_config_offset = le_to_uint(data->disp_config_addr);
		header->disp_config_size = le_to_uint(data->disp_config_size);
	} else {
		header->contains_disp_config = false;
	}

	return;
}

static int fwu_read_f01_device_status(struct synaptics_rmi4_fwu_handle *fwu,
			struct f01_device_status *status)
{
	int retval, retry = 0;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	do {
		if (retry++) {
			usleep(1000 * SYNAPTICS_CRC_DELAY_MS);
			dev_info(rmi4_data->pdev->dev.parent,
					"%s: Wait for CRC to finish\n",
					__func__);
		}
		retval = synaptics_rmi4_reg_read(rmi4_data,
				rmi4_data->f01_data_base_addr,
				status->data,
				sizeof(status->data));
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read F01 device status\n",
					__func__);
			return retval;
		}
	} while (status->status_code == SYNAPTICS_F01_CRC_IN_PROGRESS
		&& retry < 2);

	return 0;
}

static int fwu_validate_fw_status(struct synaptics_rmi4_fwu_handle *fwu,
			struct f01_device_status *status)
{
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	dev_dbg(rmi4_data->pdev->dev.parent,
		"Validate device status: 0x%x", status->status_code);

	/* check status code */
	switch (status->status_code) {
	case SYNAPTICS_F01_INV_CONFIG:
		dev_err(rmi4_data->pdev->dev.parent,
			"Invalid configuration: forcing firmware upgrade");
		goto fail;
		break;
	case SYNAPTICS_F01_CONFIG_CRC_FAIL:
		dev_err(rmi4_data->pdev->dev.parent,
			"Configuration CRC Failure: forcing firmware upgrade");
		goto fail;
		break;
	case SYNAPTICS_F01_FIRMWARE_CRC_FAIL:
		dev_err(rmi4_data->pdev->dev.parent,
			"Firmware CRC Failure: forcing firmware upgrade");
		goto fail;
		break;
	case SYNAPTICS_F01_CRC_IN_PROGRESS:
		dev_info(rmi4_data->pdev->dev.parent,
			"CRC in progress...");
		break;
	default:
		break;
	}

	return 0;

fail:
	return -ENODEV;
}

static int fwu_read_f34_queries(struct synaptics_rmi4_fwu_handle *fwu)
{
	int retval;
	unsigned char count;
	unsigned char buf[10];
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fwu->f34_fd.query_base_addr + BOOTLOADER_ID_OFFSET,
			fwu->bootloader_id,
			sizeof(fwu->bootloader_id));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read bootloader ID\n",
				__func__);
		return retval;
	}

	if (fwu->bootloader_id[1] == '5') {
		fwu->bl_version = V5;
	} else if (fwu->bootloader_id[1] == '6') {
		fwu->bl_version = V6;
	} else {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Unrecognized bootloader version\n",
				__func__);
		return -EINVAL;
	}

	if (fwu->bl_version == V5) {
		fwu->properties_off = V5_PROPERTIES_OFFSET;
		fwu->blk_size_off = V5_BLOCK_SIZE_OFFSET;
		fwu->blk_count_off = V5_BLOCK_COUNT_OFFSET;
		fwu->blk_data_off = V5_BLOCK_DATA_OFFSET;
	} else if (fwu->bl_version == V6) {
		fwu->properties_off = V6_PROPERTIES_OFFSET;
		fwu->blk_size_off = V6_BLOCK_SIZE_OFFSET;
		fwu->blk_count_off = V6_BLOCK_COUNT_OFFSET;
		fwu->blk_data_off = V6_BLOCK_DATA_OFFSET;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fwu->f34_fd.query_base_addr + fwu->properties_off,
			&fwu->flash_properties,
			sizeof(fwu->flash_properties));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read flash properties\n",
				__func__);
		return retval;
	}

	count = 4;

	if (fwu->flash_properties & HAS_PERM_CONFIG) {
		fwu->has_perm_config = 1;
		count += 2;
	}

	if (fwu->flash_properties & HAS_BL_CONFIG) {
		fwu->has_bl_config = 1;
		count += 2;
	}

	if (fwu->flash_properties & HAS_DISP_CONFIG) {
		fwu->has_disp_config = 1;
		count += 2;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fwu->f34_fd.query_base_addr + fwu->blk_size_off,
			buf,
			2);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read block size info\n",
				__func__);
		return retval;
	}

	batohs(&fwu->block_size, &(buf[0]));

	if (fwu->bl_version == V5) {
		fwu->flash_cmd_off = fwu->blk_data_off + fwu->block_size;
		fwu->flash_status_off = fwu->flash_cmd_off;
	} else if (fwu->bl_version == V6) {
		fwu->flash_cmd_off = V6_FLASH_COMMAND_OFFSET;
		fwu->flash_status_off = V6_FLASH_STATUS_OFFSET;
	}

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fwu->f34_fd.query_base_addr + fwu->blk_count_off,
			buf,
			count);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read block count info\n",
				__func__);
		return retval;
	}

	batohs(&fwu->fw_block_count, &(buf[0]));
	batohs(&fwu->config_block_count, &(buf[2]));

	count = 4;

	if (fwu->has_perm_config) {
		batohs(&fwu->perm_config_block_count, &(buf[count]));
		count += 2;
	}

	if (fwu->has_bl_config) {
		batohs(&fwu->bl_config_block_count, &(buf[count]));
		count += 2;
	}

	if (fwu->has_disp_config)
		batohs(&fwu->disp_config_block_count, &(buf[count]));

	return 0;
}

static int fwu_read_f34_flash_status(struct synaptics_rmi4_fwu_handle *fwu)
{
	int retval;
	unsigned char status;
	unsigned char command;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fwu->f34_fd.data_base_addr + fwu->flash_status_off,
			&status,
			sizeof(status));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read flash status\n",
				__func__);
		return retval;
	}

	fwu->program_enabled = status >> 7;

	if (fwu->bl_version == V5)
		fwu->flash_status = (status >> 4) & MASK_3BIT;
	else if (fwu->bl_version == V6)
		fwu->flash_status = status & MASK_3BIT;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fwu->f34_fd.data_base_addr + fwu->flash_cmd_off,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read flash command\n",
				__func__);
		return retval;
	}

	fwu->command = command & MASK_4BIT;

	return 0;
}

static int fwu_write_f34_command(struct synaptics_rmi4_fwu_handle *fwu,
				unsigned char cmd)
{
	int retval;
	unsigned char command = cmd & MASK_4BIT;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	fwu->command = cmd;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			fwu->f34_fd.data_base_addr + fwu->flash_cmd_off,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write command 0x%02x\n",
				__func__, command);
		return retval;
	}

	return 0;
}

/*
 * This routine check for the flash_status which will be set by the
 * interrupt handler. In case it is not set, it will poll after the
 * sleep.
 */
static int fwu_wait_for_idle(struct synaptics_rmi4_fwu_handle *fwu,
			int timeout_ms)
{
	int count = 0;
	int timeout_count = ((timeout_ms * 1000) / MAX_SLEEP_TIME_US) + 1;
	int poll_count = ((IDLE_WAIT_POLL_TIME_MS * 1000) / MAX_SLEEP_TIME_US) + 1;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	do {
		count++;

		if (((fwu->command == 0x00) && (fwu->flash_status == 0x00)) ||
		    (count >= timeout_count)) {
			/* idle or exceed the timeout. exit from the loop */
			break;
		}

		/* if we didn't get the interrupt, let's poll periodically */
		if ((count % poll_count) == 0)
			fwu_read_f34_flash_status(fwu);

		/* Since it is not done, wait for a minimum time */
		usleep_range(MIN_SLEEP_TIME_US, MAX_SLEEP_TIME_US);

	} while (count < timeout_count);

	if (count >= timeout_count) {
		dev_err(rmi4_data->pdev->dev.parent,
			"%s: Timed out waiting for idle status\n",
			__func__);
		return -ETIMEDOUT;
	} else
		return 0;

}

static enum flash_area fwu_go_nogo(struct synaptics_rmi4_fwu_handle *fwu,
				struct image_header_data *header)
{
	int retval;
	enum flash_area flash_area = NONE;
	unsigned char index = 0;
	unsigned int device_config_id;
	unsigned int image_config_id;
	unsigned int device_fw_id;
	unsigned long image_fw_id;
	char *strptr;
	char *firmware_id;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;
	unsigned char *config_id = rmi4_data->config_id;

	if (fwu->force_update) {
		flash_area = UI_FIRMWARE;
		goto exit;
	}

	/* Update both UI and config if device is in bootloader mode */
	if (fwu->in_flash_prog_mode) {
		flash_area = UI_FIRMWARE;
		goto exit;
	}

	/* Get device firmware ID */
	device_fw_id = rmi4_data->firmware_id;
	dev_info(rmi4_data->pdev->dev.parent,
			"%s: Device firmware ID = %d\n",
			__func__, device_fw_id);

	/* Get image firmware ID */
	if (header->contains_firmware_id) {
		image_fw_id = header->firmware_id;
	} else {
		strptr = strnstr(fwu->image_name, "PR", strlen("PR"));
		if (!strptr) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: No valid PR number (PRxxxxxxx) "
					"found in image file name (%s)\n",
					__func__, fwu->image_name);
			flash_area = AREA_ERR;
			goto exit;
		}

		strptr += 2;
		firmware_id = kzalloc(MAX_FIRMWARE_ID_LEN, GFP_KERNEL);
		while (strptr[index] >= '0' && strptr[index] <= '9') {
			firmware_id[index] = strptr[index];
			index++;
		}

		retval = sstrtoul(firmware_id, 10, &image_fw_id);
		kfree(firmware_id);
		if (retval) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to obtain image firmware ID\n",
					__func__);
			flash_area = NONE;
			goto exit;
		}
	}
	dev_info(rmi4_data->pdev->dev.parent,
			"%s: Image firmware ID = %d\n",
			__func__, (unsigned int)image_fw_id);

	if (image_fw_id != device_fw_id) {
		flash_area = UI_FIRMWARE;
		goto exit;
	}

	/* Get device config ID */
	retval = synaptics_rmi4_reg_read(rmi4_data,
				fwu->f34_fd.ctrl_base_addr,
				config_id,
				sizeof(rmi4_data->config_id));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read device config ID\n",
				__func__);
		flash_area = AREA_ERR;
		goto exit;
	}
	device_config_id = be_to_uint(config_id);
	dev_info(rmi4_data->pdev->dev.parent,
			"%s: Device config ID = 0x%02x 0x%02x 0x%02x 0x%02x\n",
			__func__,
			config_id[0],
			config_id[1],
			config_id[2],
			config_id[3]);

	/* Get image config ID */
	image_config_id = be_to_uint(fwu->config_data);
	dev_info(rmi4_data->pdev->dev.parent,
			"%s: Image config ID = 0x%02x 0x%02x 0x%02x 0x%02x\n",
			__func__,
			fwu->config_data[0],
			fwu->config_data[1],
			fwu->config_data[2],
			fwu->config_data[3]);

	if (image_config_id != device_config_id) {
		flash_area = CONFIG_AREA;
		goto exit;
	}

	flash_area = NONE;

exit:
	if (flash_area == AREA_ERR) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Abort reflash\n",
				__func__);
	} else if (flash_area == NONE) {
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: No need to do reflash\n",
				__func__);
		fwu->state = fw_do_not_upgrade;
	} else {
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: Need to update %s\n",
				__func__,
				flash_area == UI_FIRMWARE ?
				"UI firmware" :
				"config only");
	}

	return flash_area;
}

static int fwu_scan_pdt(struct synaptics_rmi4_fwu_handle *fwu)
{
	int retval;
	unsigned char ii;
	unsigned char intr_count = 0;
	unsigned char intr_off;
	unsigned char intr_src;
	unsigned short addr;
	bool f01found = false;
	bool f34found = false;
	struct synaptics_rmi4_fn_desc rmi_fd;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	for (addr = PDT_START; addr > PDT_END; addr -= PDT_ENTRY_SIZE) {
		retval = synaptics_rmi4_reg_read(rmi4_data,
				addr,
				(unsigned char *)&rmi_fd,
				sizeof(rmi_fd));
		if (retval < 0)
			return retval;

		if (rmi_fd.fn_number) {
			dev_dbg(rmi4_data->pdev->dev.parent,
					"%s: Found F%02x\n",
					__func__, rmi_fd.fn_number);
			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F01:
				f01found = true;

				rmi4_data->f01_query_base_addr =
						rmi_fd.query_base_addr;
				rmi4_data->f01_ctrl_base_addr =
						rmi_fd.ctrl_base_addr;
				rmi4_data->f01_data_base_addr =
						rmi_fd.data_base_addr;
				rmi4_data->f01_cmd_base_addr =
						rmi_fd.cmd_base_addr;
				break;
			case SYNAPTICS_RMI4_F34:
				f34found = true;
				fwu->f34_fd.query_base_addr =
						rmi_fd.query_base_addr;
				fwu->f34_fd.ctrl_base_addr =
						rmi_fd.ctrl_base_addr;
				fwu->f34_fd.data_base_addr =
						rmi_fd.data_base_addr;

				fwu->intr_mask = 0;
				intr_src = rmi_fd.intr_src_count;
				intr_off = intr_count % 8;
				for (ii = intr_off;
						ii < ((intr_src & MASK_3BIT) +
						intr_off);
						ii++) {
					fwu->intr_mask |= 1 << ii;
				}
				break;
			}
		} else {
			break;
		}

		intr_count += (rmi_fd.intr_src_count & MASK_3BIT);
	}

	if (!f01found || !f34found) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to find both F01 and F34\n",
				__func__);
		return -EINVAL;
	}

	return 0;
}

static int fwu_write_blocks(struct synaptics_rmi4_fwu_handle *fwu,
		unsigned char *block_ptr,
		unsigned short block_cnt,
		unsigned char command)
{
	int retval;
	unsigned char block_offset[] = {0, 0};
	unsigned short block_num;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	block_offset[1] |= (fwu->config_area << 5);

	retval = synaptics_rmi4_reg_write(rmi4_data,
			fwu->f34_fd.data_base_addr + BLOCK_NUMBER_OFFSET,
			block_offset,
			sizeof(block_offset));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write to block number registers\n",
				__func__);
		return retval;
	}

	for (block_num = 0; block_num < block_cnt; block_num++) {
		retval = synaptics_rmi4_reg_write(rmi4_data,
				fwu->f34_fd.data_base_addr + fwu->blk_data_off,
				block_ptr,
				fwu->block_size);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to write block data (block %d)\n",
					__func__, block_num);
			return retval;
		}

		retval = fwu_write_f34_command(fwu, command);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to write command for block %d\n",
					__func__, block_num);
			return retval;
		}

		retval = fwu_wait_for_idle(fwu, WRITE_WAIT_MS);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to wait for idle status (block %d)\n",
					__func__, block_num);
			return retval;
		}

		block_ptr += fwu->block_size;
	}

	return 0;
}

static int fwu_write_firmware(struct synaptics_rmi4_fwu_handle *fwu)
{
	return fwu_write_blocks(fwu, (unsigned char *)fwu->firmware_data,
		fwu->fw_block_count, CMD_WRITE_FW_BLOCK);
}

static int fwu_write_configuration(struct synaptics_rmi4_fwu_handle *fwu)
{
	return fwu_write_blocks(fwu, (unsigned char *)fwu->config_data,
		fwu->config_block_count, CMD_WRITE_CONFIG_BLOCK);
}

static int fwu_write_disp_configuration(struct synaptics_rmi4_fwu_handle *fwu)
{
	fwu->config_area = DISP_CONFIG_AREA;
	fwu->config_data = fwu->disp_config_data;
	fwu->config_block_count = fwu->disp_config_block_count;

	return fwu_do_write_config(fwu);
}

static int fwu_write_lockdown(struct synaptics_rmi4_fwu_handle *fwu)
{
	return fwu_write_blocks(fwu, (unsigned char *)fwu->lockdown_data,
		fwu->lockdown_block_count, CMD_WRITE_LOCKDOWN_BLOCK);
}

static int fwu_write_bootloader_id(struct synaptics_rmi4_fwu_handle *fwu)
{
	int retval;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			fwu->f34_fd.data_base_addr + fwu->blk_data_off,
			fwu->bootloader_id,
			sizeof(fwu->bootloader_id));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write bootloader ID\n",
				__func__);
		return retval;
	}

	return 0;
}

static int fwu_enter_flash_prog(struct synaptics_rmi4_fwu_handle *fwu)
{
	int retval;
	struct f01_device_status f01_device_status;
	struct f01_device_control f01_device_control;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	retval = fwu_write_bootloader_id(fwu);
	if (retval < 0)
		return retval;

	retval = fwu_write_f34_command(fwu, CMD_ENABLE_FLASH_PROG);
	if (retval < 0)
		return retval;

	retval = fwu_wait_for_idle(fwu, ENABLE_WAIT_MS);
	if (retval < 0)
		return retval;

	if (!fwu->program_enabled) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Program enabled bit not set\n",
				__func__);
		return -EINVAL;
	}

	retval = fwu_scan_pdt(fwu);
	if (retval < 0)
		return retval;

	retval = fwu_read_f01_device_status(fwu, &f01_device_status);
	if (retval < 0)
		return retval;

	if (!f01_device_status.flash_prog) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Not in flash prog mode\n",
				__func__);
		return -EINVAL;
	}

	retval = fwu_validate_fw_status(fwu, &f01_device_status);

	if (retval < 0) {
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: Invalid firmware, force upgrade\n",
				__func__);
		fwu->force_update = true;
	}

	retval = fwu_read_f34_queries(fwu);
	if (retval < 0)
		return retval;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			f01_device_control.data,
			sizeof(f01_device_control.data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read F01 device control\n",
				__func__);
		return retval;
	}

	f01_device_control.nosleep = true;
	f01_device_control.sleep_mode = SLEEP_MODE_NORMAL;

	retval = synaptics_rmi4_reg_write(rmi4_data,
			rmi4_data->f01_ctrl_base_addr,
			f01_device_control.data,
			sizeof(f01_device_control.data));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write F01 device control\n",
				__func__);
		return retval;
	}

	return retval;
}

static int fwu_do_reflash(struct synaptics_rmi4_fwu_handle *fwu)
{
	int retval;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	retval = fwu_write_bootloader_id(fwu);
	if (retval < 0)
		return retval;

	dev_info(rmi4_data->pdev->dev.parent,
			"%s: Bootloader ID written\n",
			__func__);

	retval = fwu_write_f34_command(fwu, CMD_ERASE_ALL);
	if (retval < 0)
		return retval;

	dev_info(rmi4_data->pdev->dev.parent,
			"%s: Erase all command written\n",
			__func__);

	retval = fwu_wait_for_idle(fwu, ERASE_WAIT_MS);
	if (retval < 0)
		return retval;

	dev_info(rmi4_data->pdev->dev.parent,
			"%s: Idle status detected\n",
			__func__);

	if (fwu->firmware_data) {
		retval = fwu_write_firmware(fwu);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Write firmware failed %d\n",
				__func__, retval);
			return retval;
		}
		dev_info(rmi4_data->pdev->dev.parent,
			"%s: Firmware programmed\n",
			__func__);
	}

	if (fwu->config_data) {
		retval = fwu_write_configuration(fwu);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Write configuration failed %d\n",
				__func__, retval);
			return retval;
		}
		dev_info(rmi4_data->pdev->dev.parent,
			"%s: Configuration programmed\n",
			__func__);
	}

	if (fwu->disp_config_data) {
		retval = fwu_write_disp_configuration(fwu);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Write disp configuration failed %d\n",
				__func__, retval);
			return retval;
		}
		dev_info(rmi4_data->pdev->dev.parent,
			"%s: Display configuration programmed\n",
			__func__);
	}

	return retval;
}

static int fwu_do_write_config(struct synaptics_rmi4_fwu_handle *fwu)
{
	int retval;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	if (fwu->config_area == PERM_CONFIG_AREA) {
		fwu->config_block_count = fwu->perm_config_block_count;
		goto write_config;
	}

	retval = fwu_write_bootloader_id(fwu);
	if (retval < 0)
		return retval;

	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Bootloader ID written\n",
			__func__);

	switch (fwu->config_area) {
	case UI_CONFIG_AREA:
		retval = fwu_write_f34_command(fwu, CMD_ERASE_CONFIG);
		break;
	case BL_CONFIG_AREA:
		retval = fwu_write_f34_command(fwu, CMD_ERASE_BL_CONFIG);
		fwu->config_block_count = fwu->bl_config_block_count;
		break;
	case DISP_CONFIG_AREA:
		retval = fwu_write_f34_command(fwu, CMD_ERASE_DISP_CONFIG);
		fwu->config_block_count = fwu->disp_config_block_count;
		break;
	}
	if (retval < 0)
		return retval;

	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Erase command written\n",
			__func__);

	retval = fwu_wait_for_idle(fwu, ERASE_WAIT_MS);
	if (retval < 0)
		return retval;

	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Idle status detected\n",
			__func__);

write_config:
	retval = fwu_write_configuration(fwu);
	if (retval < 0)
		return retval;

	dev_info(rmi4_data->pdev->dev.parent,
		"%s: Config written\n", __func__);

	return retval;
}
#ifdef CONFIG_BBRY_DEBUG
static int fwu_start_write_config(struct synaptics_rmi4_fwu_handle *fwu)
{
	int retval;
	unsigned short block_count;
	struct image_header_data header;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	switch (fwu->config_area) {
	case UI_CONFIG_AREA:
		block_count = fwu->config_block_count;
		break;
	case PERM_CONFIG_AREA:
		if (!fwu->has_perm_config)
			return -EINVAL;
		block_count = fwu->perm_config_block_count;
		break;
	case BL_CONFIG_AREA:
		if (!fwu->has_bl_config)
			return -EINVAL;
		block_count = fwu->bl_config_block_count;
		break;
	case DISP_CONFIG_AREA:
		if (!fwu->has_disp_config)
			return -EINVAL;
		block_count = fwu->disp_config_block_count;
		break;
	default:
		return -EINVAL;
	}

	if (fwu->ext_data_source)
		fwu->config_data = fwu->ext_data_source;
	else
		return -EINVAL;

	fwu->config_size = fwu->block_size * block_count;

	/* Jump to the config area if given a packrat image */
	if ((fwu->config_area == UI_CONFIG_AREA) &&
			(fwu->config_size != fwu->image_size)) {
		parse_header(&header, fwu->ext_data_source);

		if (header.config_size) {
			fwu->config_data = fwu->ext_data_source +
					IMAGE_AREA_OFFSET +
					header.firmware_size;
			if (header.contains_bootloader)
				fwu->config_data += header.bootloader_size;
		} else {
			return -EINVAL;
		}
	}

	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: Start of write config process\n", __func__);

	retval = fwu_enter_flash_prog(fwu);
	if (retval < 0)
		goto exit;

	retval = fwu_do_write_config(fwu);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write config\n",
				__func__);
	}

exit:
	dev_err(rmi4_data->pdev->dev.parent,
				"%s: Reset controller\n",
				__func__);
	rmi4_data->reset_device(rmi4_data, false);

	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: End of write config process\n", __func__);

	return retval;
}

static int fwu_do_read_config(struct synaptics_rmi4_fwu_handle *fwu)
{
	int retval;
	unsigned char block_offset[] = {0, 0};
	unsigned short block_num;
	unsigned short block_count;
	unsigned short index = 0;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	retval = fwu_enter_flash_prog(fwu);
	if (retval < 0)
		goto exit;

	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: Entered flash prog mode\n",
			__func__);

	switch (fwu->config_area) {
	case UI_CONFIG_AREA:
		block_count = fwu->config_block_count;
		break;
	case PERM_CONFIG_AREA:
		if (!fwu->has_perm_config) {
			retval = -EINVAL;
			goto exit;
		}
		block_count = fwu->perm_config_block_count;
		break;
	case BL_CONFIG_AREA:
		if (!fwu->has_bl_config) {
			retval = -EINVAL;
			goto exit;
		}
		block_count = fwu->bl_config_block_count;
		break;
	case DISP_CONFIG_AREA:
		if (!fwu->has_disp_config) {
			retval = -EINVAL;
			goto exit;
		}
		block_count = fwu->disp_config_block_count;
		break;
	default:
		retval = -EINVAL;
		goto exit;
	}

	fwu->config_size = fwu->block_size * block_count;

	kfree(fwu->read_config_buf);
	fwu->read_config_buf = kzalloc(fwu->config_size, GFP_KERNEL);

	block_offset[1] |= (fwu->config_area << 5);

	retval = synaptics_rmi4_reg_write(rmi4_data,
			fwu->f34_fd.data_base_addr + BLOCK_NUMBER_OFFSET,
			block_offset,
			sizeof(block_offset));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write to block number registers\n",
				__func__);
		goto exit;
	}

	for (block_num = 0; block_num < block_count; block_num++) {
		retval = fwu_write_f34_command(fwu, CMD_READ_CONFIG_BLOCK);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to write read config command\n",
					__func__);
			goto exit;
		}

		retval = fwu_wait_for_idle(fwu, WRITE_WAIT_MS);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to wait for idle status\n",
					__func__);
			goto exit;
		}

		retval = synaptics_rmi4_reg_read(rmi4_data,
				fwu->f34_fd.data_base_addr + fwu->blk_data_off,
				&fwu->read_config_buf[index],
				fwu->block_size);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to read block data (block %d)\n",
					__func__, block_num);
			goto exit;
		}

		index += fwu->block_size;
	}

exit:
	dev_err(rmi4_data->pdev->dev.parent,
		"%s: Reset controller\n",
		__func__);
	rmi4_data->reset_device(rmi4_data, false);

	return retval;
}
#endif /*CONFIG_BBRY_DEBUG*/
static int fwu_do_lockdown(struct synaptics_rmi4_fwu_handle *fwu)
{
	int retval;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	retval = fwu_enter_flash_prog(fwu);
	if (retval < 0)
		return retval;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			fwu->f34_fd.query_base_addr + fwu->properties_off,
			&fwu->flash_properties,
			sizeof(fwu->flash_properties));
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read flash properties\n",
				__func__);
		return retval;
	}

	if ((fwu->flash_properties & UNLOCKED) == 0) {
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: Device already locked down\n",
				__func__);
		return retval;
	}

	retval = fwu_write_lockdown(fwu);
	if (retval < 0)
		return retval;

	dev_info(rmi4_data->pdev->dev.parent,
		"%s: Lockdown programmed\n", __func__);

	return retval;
}

static int fwu_start_reflash(struct synaptics_rmi4_fwu_handle *fwu)
{
	int retval = 0;
	bool upgrade_complete = false;
	bool fw_was_invalid = false;
	enum flash_area flash_area;
	struct image_header_data header;
	struct f01_device_status f01_device_status;
	const unsigned char *fw_image;
	const struct firmware *fw_entry = NULL;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;
	const struct synaptics_dsx_board_data *bdata =
			rmi4_data->hw_if->board_data;
	bool fw_lockdown = false;

	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: Start of reflash process\n",
		__func__);

	if (fwu->ext_data_source) {
		fw_image = fwu->ext_data_source;
	} else {
		if (bdata->product_id_major &&
			(strnstr(rmi4_data->rmi4_mod_info.product_id_string,
				bdata->product_id_major, sizeof(rmi4_data->rmi4_mod_info.product_id_string)) == NULL)) {
			snprintf(fwu->image_name,
			MAX_IMAGE_NAME_LEN,
			FW_IMAGE_FOLDER LOCKDOWN_FW_PRODUCT_FOLDER FW_IMAGE_NAME,
			rmi4_data->rmi4_mod_info.product_id_string, bdata->product_id_major);
			fw_lockdown = true;
		} else {
			snprintf(fwu->image_name,
				MAX_IMAGE_NAME_LEN,
				FW_IMAGE_FOLDER FW_IMAGE_NAME,
				rmi4_data->rmi4_mod_info.product_id_string);
		}

		dev_info(rmi4_data->pdev->dev.parent,
				"%s: Requesting firmware image %s\n",
				__func__, fwu->image_name);

		retval = request_firmware(&fw_entry, fwu->image_name,
				rmi4_data->pdev->dev.parent);
		if (retval != 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Firmware image %s not available\n",
					__func__, fwu->image_name);
			fwu->state = fw_upgrade_failed;
			retval = -ENODEV;
			fw_lockdown = false;
			goto exit;
		}

		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: Firmware image size = %d\n",
				__func__, (int)fw_entry->size);

		if (fw_lockdown) {
			fwu->do_lockdown = true;
			dev_info(rmi4_data->pdev->dev.parent,
				"%s: Upgrade firmware with lockdown\n",
				__func__);
		}

		fw_image = fw_entry->data;
	}

	parse_header(&header, fw_image);

	if (fwu->bl_version != header.bootloader_version) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Bootloader version mismatch\n",
				__func__);
		retval = -EINVAL;
		goto exit;
	}

	rmi4_data->stay_awake = true;
	if (!rmi4_data->suspend)
		synaptics_rmi4_monitor_timer_stop(rmi4_data);

	retval = fwu_read_f01_device_status(fwu, &f01_device_status);
	if (retval < 0)
		goto exit;

	if (f01_device_status.flash_prog) {
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: In flash prog mode\n",
				__func__);
		fwu->in_flash_prog_mode = true;
	} else {
		fwu->in_flash_prog_mode = false;
	}

	if (fwu->do_lockdown) {
		switch (fwu->bl_version) {
		case V5:
		case V6:
			fwu->lockdown_data = fw_image + LOCKDOWN_OFFSET;
			fwu->lockdown_block_count = LOCKDOWN_BLOCK_COUNT;
			retval = fwu_do_lockdown(fwu);
			if (retval < 0) {
				dev_err(rmi4_data->pdev->dev.parent,
						"%s: Failed to do lockdown\n",
						__func__);
			}
			break;
		default:
			break;
		}
	}

	if (header.firmware_size)
		fwu->firmware_data = fw_image + IMAGE_AREA_OFFSET;
	else
		fwu->firmware_data = NULL;

	if (header.config_size)
		fwu->config_data = fw_image + IMAGE_AREA_OFFSET +
				header.firmware_size;
	else
		fwu->config_data = NULL;

	if (header.contains_bootloader) {
		if (header.firmware_size)
			fwu->firmware_data += header.bootloader_size;
		if (header.config_size)
			fwu->config_data += header.bootloader_size;
	}

	if (header.contains_disp_config)
		fwu->disp_config_data = fw_image + header.disp_config_offset;
	else
		fwu->disp_config_data = NULL;

	retval = fwu_validate_fw_status(fwu, &f01_device_status);

	if (retval < 0) {
		dev_info(rmi4_data->pdev->dev.parent,
				"%s: Invalid firmware, force upgrade\n",
				__func__);
		fwu->force_update = true;
		fw_was_invalid = true;
	}

	flash_area = fwu_go_nogo(fwu, &header);

	if (flash_area == AREA_ERR) {
		retval = -EINVAL;
		goto exit;
	}

	if (flash_area != NONE) {
		if (fwu->update_on_suspend && !rmi4_data->suspend &&
				!fw_was_invalid) {
			dev_info(rmi4_data->pdev->dev.parent,
				"%s: Waiting for suspend to perform upgrade\n",
				__func__);
			rmi4_data->reset_device(rmi4_data, true);
			goto exit;
		}
		retval = fwu_enter_flash_prog(fwu);
		if (retval < 0)
			goto exit;
	}

	dev_dbg(rmi4_data->pdev->dev.parent,
		"%s: Start Firmware upgrade, flash_area=%d, size=%d\n",
		__func__, flash_area, header.firmware_size);

	switch (flash_area) {
	case UI_FIRMWARE:
		retval = fwu_do_reflash(fwu);
		upgrade_complete = true;
		break;
	case CONFIG_AREA:
		retval = fwu_do_write_config(fwu);
		upgrade_complete = true;
		break;
	case NONE:
	default:
		dev_info(rmi4_data->pdev->dev.parent,
			"%s: No Firmware upgrade required.\n",
				__func__);
		goto exit;
	}

	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to do reflash\n",
				__func__);
		fwu->state = fw_upgrade_failed;
	} else
		fwu->state = fw_upgrade_complete;

exit:
	if (upgrade_complete) {
		kfree(fwu->ext_data_source);
		fwu->ext_data_source = NULL;
		fwu->do_lockdown = DO_LOCKDOWN;
		fwu->force_update = FORCE_UPDATE;

		if (retval >= 0) {
			if (fwu->update_on_suspend && fw_was_invalid) {
				dev_info(rmi4_data->pdev->dev.parent,
						"%s: System started with invalid firmware; "
						"reboot to recover display\n",
						__func__);
				queue_delayed_work(fwu->fwu_workqueue,
						&fwu->restart_work,
						msecs_to_jiffies(RESTART_DELAY_MS));
			} else {
				dev_info(rmi4_data->pdev->dev.parent,
					"%s: Controller reset after touch firmware upgrade\n",
					__func__);
				rmi4_data->hw_reset = false;
				rmi4_data->pre_suspend_reset = true;
				queue_work(rmi4_data->workqueue,
					&rmi4_data->reset_work);
				rmi4_data->pre_suspend_reset = false;
				flush_workqueue(rmi4_data->workqueue);
			}
		}
	} else if (!rmi4_data->suspend)
		synaptics_rmi4_monitor_timer_start(rmi4_data);

	if (fw_entry)
		release_firmware(fw_entry);

	rmi4_data->stay_awake = false;
	queue_work(rmi4_data->workqueue, &rmi4_data->fwu_done);
	dev_info(rmi4_data->pdev->dev.parent,
		"%s: End of reflash process\n", __func__);

	return retval;
}

int synaptics_fw_updater(struct synaptics_rmi4_fwu_handle *fwu,
			unsigned char *fw_data)
{
	int retval;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	if (!fwu)
		return -ENODEV;

	if (!fwu->initialized)
		return -ENODEV;

	mutex_lock(&fwu->mutex);

	fwu->ext_data_source = fw_data;
	fwu->config_area = UI_CONFIG_AREA;

	rmi4_data->state_changed = false;
	if (rmi4_data->suspend && !rmi4_data->wakeup_gesture.data) {
		retval = rmi4_data->power_enable(rmi4_data, true);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to enable power, unable to update firmware\n",
					__func__);
			goto exit;
		}
	}

	/* Enable the IRQ regardless the suspend state */
	retval = rmi4_data->irq_enable(rmi4_data, true, false);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to enable interrupt, unable to update firmware\n",
				__func__);
		rmi4_data->power_enable(rmi4_data, false);
		goto exit;
	}

	retval = fwu_start_reflash(fwu);

	if (retval < 0) {
		if (retval != -ENODEV) {
			dev_err(rmi4_data->pdev->dev.parent,
				"%s: Firmware update failed in state %d, reset controller\n",
				__func__, fwu->state);
			rmi4_data->reset_device(rmi4_data, true);
		}
	}

	if (rmi4_data->suspend && !rmi4_data->wakeup_gesture.data
		/* When firmware upgrade completed, device is reset back to suspend, */
		/* no need to redo it*/
		&& fwu->state != fw_upgrade_complete) {
		retval = rmi4_data->irq_enable(rmi4_data, false, false);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to enable interrupt, unable to update firmware\n",
					__func__);
			rmi4_data->power_enable(rmi4_data, false);
			goto exit;
		}

		retval = rmi4_data->power_enable(rmi4_data, false);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to disable power after fw upgrade\n",
					__func__);
			goto exit;
		}
	}

	if (rmi4_data->state_changed)
		queue_work(rmi4_data->workqueue, &rmi4_data->power_state_work);

exit:
	mutex_unlock(&fwu->mutex);
	return retval;
}
EXPORT_SYMBOL(synaptics_fw_updater);

static void fwu_restart_work(struct work_struct *work)
{
	kernel_restart("oem-0x04");
}

static void fwu_startup_fw_update_work(struct work_struct *work)
{
	struct synaptics_rmi4_fwu_handle *fwu =
		container_of((struct delayed_work *) work,
		struct synaptics_rmi4_fwu_handle,
		fwu_work);
	struct synaptics_rmi4_data  *rmi4_data = fwu->rmi4_data;

	synaptics_fw_updater(fwu, NULL);
	if ((fw_upgrade_failed == fwu->state) &&
	    (fwu->fw_upgrade_cnt < MAX_FW_UPGRADE_TRIES)) {
		fwu->fw_upgrade_cnt++;
		dev_err(rmi4_data->pdev->dev.parent,
			"fw upgrade failed. retry count=%d\n",
					fwu->fw_upgrade_cnt);
		queue_delayed_work(fwu->fwu_workqueue,
			&fwu->fwu_work,
			msecs_to_jiffies(STARTUP_FW_UPDATE_DELAY_MS));
	} else {
		rmi4_data->touch_ready = true;
	}

	return;
}

static ssize_t fwu_sysfs_show_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_fwu_handle *fwu =
		(struct synaptics_rmi4_fwu_handle *) rmi4_data->fwu;

	if (count < fwu->config_size) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Not enough space (%d bytes) in buffer\n",
				__func__, (int)count);
		return -EINVAL;
	}

	memcpy(buf, fwu->read_config_buf, fwu->config_size);

	return fwu->config_size;
}

static ssize_t fwu_sysfs_store_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_fwu_handle *fwu =
		(struct synaptics_rmi4_fwu_handle *) rmi4_data->fwu;

	memcpy((void *)(&fwu->ext_data_source[fwu->data_pos]),
			(const void *)buf,
			count);

	fwu->data_pos += count;

	return count;
}

static ssize_t fwu_sysfs_do_reflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_fwu_handle *fwu =
		(struct synaptics_rmi4_fwu_handle *) rmi4_data->fwu;

	if (sscanf(buf, "%u", &input) != 1) {
		retval = -EINVAL;
		goto exit;
	}

	if (input & LOCKDOWN) {
		fwu->do_lockdown = true;
		input &= ~LOCKDOWN;
	}

	if ((input != NORMAL) && (input != FORCE)) {
		retval = -EINVAL;
		goto exit;
	}

	if (input == FORCE) {
		fwu->force_update = true;
		fwu->state = fw_idle;
	}

	dev_info(rmi4_data->pdev->dev.parent,
				"%s: Start reflash with mode %d\n",
				__func__, input);

	queue_delayed_work(fwu->fwu_workqueue,
			&fwu->fwu_work, 0);
	retval = count;

exit:
	return retval;
}

#ifdef CONFIG_BBRY_DEBUG
static ssize_t fwu_sysfs_write_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_fwu_handle *fwu =
		(struct synaptics_rmi4_fwu_handle *) rmi4_data->fwu;

	if (sscanf(buf, "%u", &input) != 1) {
		retval = -EINVAL;
		goto exit;
	}

	if (input != 1) {
		retval = -EINVAL;
		goto exit;
	}

	retval = fwu_start_write_config(fwu);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to write config\n",
				__func__);
		goto exit;
	}

	retval = count;

exit:
	kfree(fwu->ext_data_source);
	fwu->ext_data_source = NULL;
	return retval;
}

static ssize_t fwu_sysfs_read_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_fwu_handle *fwu =
		(struct synaptics_rmi4_fwu_handle *) rmi4_data->fwu;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input != 1)
		return -EINVAL;

	retval = fwu_do_read_config(fwu);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to read config\n",
				__func__);
		return retval;
	}

	return count;
}

static ssize_t fwu_sysfs_config_area_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned long config_area;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_fwu_handle *fwu =
		(struct synaptics_rmi4_fwu_handle *) rmi4_data->fwu;

	retval = sstrtoul(buf, 10, &config_area);
	if (retval)
		return retval;

	fwu->config_area = config_area;

	return count;
}

static ssize_t fwu_sysfs_image_name_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_fwu_handle *fwu =
		(struct synaptics_rmi4_fwu_handle *) rmi4_data->fwu;

	memcpy(fwu->image_name, buf, count);

	return count;
}

static ssize_t fwu_sysfs_image_size_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned long size;
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_fwu_handle *fwu =
		(struct synaptics_rmi4_fwu_handle *) rmi4_data->fwu;

	retval = sstrtoul(buf, 10, &size);
	if (retval)
		return retval;

	fwu->image_size = size;
	fwu->data_pos = 0;

	kfree(fwu->ext_data_source);
	fwu->ext_data_source = kzalloc(fwu->image_size, GFP_KERNEL);
	if (!fwu->ext_data_source) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for image data\n",
				__func__);
		return -ENOMEM;
	}

	return count;
}
#endif /*CONFIG_BBRY_DEBUG*/

static ssize_t fwu_sysfs_block_size_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_fwu_handle *fwu =
		(struct synaptics_rmi4_fwu_handle *) rmi4_data->fwu;

	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->block_size);
}

static ssize_t fwu_sysfs_firmware_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_fwu_handle *fwu =
		(struct synaptics_rmi4_fwu_handle *) rmi4_data->fwu;

	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->fw_block_count);
}

static ssize_t fwu_sysfs_configuration_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_fwu_handle *fwu =
		(struct synaptics_rmi4_fwu_handle *) rmi4_data->fwu;

	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->config_block_count);
}

static ssize_t fwu_sysfs_perm_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_fwu_handle *fwu =
		(struct synaptics_rmi4_fwu_handle *) rmi4_data->fwu;

	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->perm_config_block_count);
}

static ssize_t fwu_sysfs_bl_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_fwu_handle *fwu =
		(struct synaptics_rmi4_fwu_handle *) rmi4_data->fwu;

	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->bl_config_block_count);
}

static ssize_t fwu_sysfs_disp_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
	struct synaptics_rmi4_fwu_handle *fwu =
		(struct synaptics_rmi4_fwu_handle *) rmi4_data->fwu;

	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->disp_config_block_count);
}

static int synaptics_rmi4_fwu_attn(struct synaptics_rmi4_data *rmi4_data,
		unsigned char intr_mask)
{
	struct synaptics_rmi4_fwu_handle *fwu =
		(struct synaptics_rmi4_fwu_handle *) rmi4_data->fwu;

	if (!fwu)
		return 0;

	if (fwu->intr_mask & intr_mask)
		fwu_read_f34_flash_status(fwu);

	return 0;
}

static int synaptics_rmi4_fwu_init(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char attr_count;
	struct pdt_properties pdt_props;
	struct synaptics_rmi4_fwu_handle *fwu;

	fwu = kzalloc(sizeof(*fwu), GFP_KERNEL);
	if (!fwu) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for fwu\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}

	memset(fwu, 0x00, sizeof(*fwu));
	rmi4_data->fwu = fwu;
	fwu->state = fw_idle;

	fwu->image_name = kzalloc(MAX_IMAGE_NAME_LEN, GFP_KERNEL);
	if (!fwu->image_name) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to alloc mem for image name\n",
				__func__);
		retval = -ENOMEM;
		goto exit_free_fwu;
	}

	fwu->rmi4_data = rmi4_data;

	retval = synaptics_rmi4_reg_read(rmi4_data,
			PDT_PROPS,
			pdt_props.data,
			sizeof(pdt_props.data));
	if (retval < 0) {
		dev_dbg(rmi4_data->pdev->dev.parent,
				"%s: Failed to read PDT properties, assuming 0x00\n",
				__func__);
	} else if (pdt_props.has_bsr) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Reflash for LTS not currently supported\n",
				__func__);
		retval = -ENODEV;
		goto exit_free_mem;
	}

	retval = fwu_scan_pdt(fwu);
	if (retval < 0)
		goto exit_free_mem;

	fwu->productinfo1 = rmi4_data->rmi4_mod_info.product_info[0];
	fwu->productinfo2 = rmi4_data->rmi4_mod_info.product_info[1];
	memcpy(fwu->product_id, rmi4_data->rmi4_mod_info.product_id_string,
			SYNAPTICS_RMI4_PRODUCT_ID_SIZE);
	fwu->product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE] = 0;

	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: F01 product info: 0x%04x 0x%04x\n",
			__func__, fwu->productinfo1, fwu->productinfo2);
	dev_dbg(rmi4_data->pdev->dev.parent,
			"%s: F01 product ID: %s\n",
			__func__, fwu->product_id);

	retval = fwu_read_f34_queries(fwu);
	if (retval < 0)
		goto exit_free_mem;

	fwu->force_update = FORCE_UPDATE;
	fwu->do_lockdown = DO_LOCKDOWN;
	fwu->initialized = true;

	retval = sysfs_create_bin_file(&rmi4_data->input_dev->dev.kobj,
			&dev_attr_data);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to create sysfs bin file\n",
				__func__);
		goto exit_free_mem;
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = sysfs_create_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
		if (retval < 0) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to create sysfs attributes\n",
					__func__);
			retval = -ENODEV;
			goto exit_remove_attrs;
		}
	}

	if (rmi4_data->hw_if->board_data->ddic_power_control)
		fwu->update_on_suspend = true;

	mutex_init(&fwu->mutex);
	INIT_DELAYED_WORK(&fwu->restart_work, fwu_restart_work);

	fwu->fwu_workqueue = create_singlethread_workqueue("fwu_workqueue");
	INIT_DELAYED_WORK(&fwu->fwu_work, fwu_startup_fw_update_work);
#ifdef DO_STARTUP_FW_UPDATE
	queue_delayed_work(fwu->fwu_workqueue,
			&fwu->fwu_work,
			msecs_to_jiffies(STARTUP_FW_UPDATE_DELAY_MS));
#endif

	return 0;

exit_remove_attrs:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

	sysfs_remove_bin_file(&rmi4_data->input_dev->dev.kobj, &dev_attr_data);

exit_free_mem:
	kfree(fwu->image_name);

exit_free_fwu:
	kfree(fwu);
	fwu = NULL;
	rmi4_data->fwu = NULL;

exit:
	return retval;
}

static void synaptics_rmi4_fwu_reset(struct synaptics_rmi4_data *rmi4_data)
{
	struct synaptics_rmi4_fwu_handle *fwu = rmi4_data->fwu;

	if (fwu && fw_idle == fwu->state)
		queue_delayed_work(fwu->fwu_workqueue,
			&fwu->fwu_work,
			msecs_to_jiffies(STARTUP_FW_UPDATE_DELAY_MS));
}

static void synaptics_rmi4_fwu_suspend(struct synaptics_rmi4_data *rmi4_data)
{
	struct synaptics_rmi4_fwu_handle *fwu = rmi4_data->fwu;

	if (fwu && fwu->update_on_suspend && fw_idle == fwu->state) {
		dev_info(rmi4_data->pdev->dev.parent,
						"%s: Got suspend - do blocking FW upgrade\n",
						__func__);
		mutex_unlock(&rmi4_data->exp_data.mutex);
		synaptics_fw_updater(fwu, NULL);
		mutex_lock(&rmi4_data->exp_data.mutex);
	}
}

static struct synaptics_rmi4_exp_fn fwu_module = {
	.fn_type = RMI_FW_UPDATER,
	.init = synaptics_rmi4_fwu_init,
	.remove = NULL,
	.reset = synaptics_rmi4_fwu_reset,
	.reinit = NULL,
	.suspend = synaptics_rmi4_fwu_suspend,
	.resume = NULL,
	.late_resume = NULL,
	.attn = synaptics_rmi4_fwu_attn,
};

int synaptics_rmi4_fw_update_module_init(
		struct synaptics_rmi4_data *rmi4_data)
{
	synaptics_rmi4_new_function(rmi4_data, &fwu_module, true);

	return 0;
}

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX FW Update Module");
MODULE_LICENSE("GPL v2");
