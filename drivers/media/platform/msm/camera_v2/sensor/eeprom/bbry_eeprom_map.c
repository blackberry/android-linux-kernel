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
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include "msm_sd.h"
#include "msm_cci.h"
#include "msm_eeprom.h"
#include "bbry_eeprom_map.h"

#undef CDBG
#ifdef BBRY_EEPROM_MAP_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

#define I2C_COMPARE_MISMATCH 1
#define A2030_OTP_BANK_START_ADDR 0x3800
#define A2030_OTP_BANK_SELECT_ADDR 0x304C
#define A2030_OTP_BANK_READ_MODE_ADDR 0x304A

/* For imx230_s
 * Full OTP from ISP is 3072 bytes (12 pages)
 */
static struct msm_eeprom_memory_map_t imx230_otp_map_s[] = {
	/*set read size to 64 words(256 bytes)*/
	{{2, 0x000F, 2, 0x40, 1, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0}},
	/*  page offset                        read command                  poll                   read mem             saddr*/
	{{2, 0x0010, 2, 0x0000, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}},
	{{2, 0x0010, 2, 0x0001, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}},
	{{2, 0x0010, 2, 0x0002, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}},
	{{2, 0x0010, 2, 0x0003, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}},
	{{2, 0x0010, 2, 0x0004, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}},
	{{2, 0x0010, 2, 0x0005, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}},
	{{2, 0x0010, 2, 0x0006, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}},
	{{2, 0x0010, 2, 0x0007, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}},
	{{2, 0x0010, 2, 0x0008, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}},
	{{2, 0x0010, 2, 0x0009, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}},
	{{2, 0x0010, 2, 0x000A, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}},
	{{2, 0x0010, 2, 0x000B, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}}
};

/* For imx135_s
 * Full OTP from ISP is 2048 bytes
 */
static struct msm_eeprom_memory_map_t imx135_otp_map_s[] = {
	/*set read size to 64 words(256 bytes)*/
	{{2, 0x000F, 2, 0x40, 1, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0}, {0}},
	/*  set offset                        read command                  poll                   read mem             saddr*/
	{{2, 0x0010, 2, 0x0000, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}},
	{{2, 0x0010, 2, 0x0001, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}},
	{{2, 0x0010, 2, 0x0002, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}},
	{{2, 0x0010, 2, 0x0003, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}},
	{{2, 0x0010, 2, 0x0004, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}},
	{{2, 0x0010, 2, 0x0005, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}},
	{{2, 0x0010, 2, 0x0006, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}},
	{{2, 0x0010, 2, 0x0007, 2, 0}, {1, 0x000E, 2, 0x04, 1, 0}, {1, 0x000E, 2, 0x14, 1, 0}, {256, 0x0100, 2, 0, 1, 0}, {0}}
};

/* For a2030 type L
 * Full OTP from ISP is 298 bytes
 */
static struct msm_eeprom_memory_map_t a2030_otp_map_l[] = {
	/*     bank offset                    read command               poll                  read mem          saddr*/
	{{2, 0x304C, 2, 0x5000, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, {23, 0x3800, 2, 0, 1, 0}, {0}},
	{{2, 0x304C, 2, 0x5100, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, {11, 0x3800, 2, 0, 1, 0}, {0}},
	{{2, 0x304C, 2, 0x5200, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, {97, 0x3800, 2, 0, 1, 0}, {0}},
	{{2, 0x304C, 2, 0x5300, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, {17, 0x3800, 2, 0, 1, 0}, {0}},
	{{2, 0x304C, 2, 0x5400, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, { 5, 0x3800, 2, 0, 1, 0}, {0}},
	{{2, 0x304C, 2, 0x5500, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, { 6, 0x3800, 2, 0, 1, 0}, {0}},
	{{2, 0x304C, 2, 0x5600, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, { 6, 0x3800, 2, 0, 1, 0}, {0}},
	{{2, 0x304C, 2, 0x5700, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, {97, 0x3800, 2, 0, 1, 0}, {0}},
	{{2, 0x304C, 2, 0x5800, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, {17, 0x3800, 2, 0, 1, 0}, {0}},
	{{2, 0x304C, 2, 0x5900, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, { 5, 0x3800, 2, 0, 1, 0}, {0}},
	{{2, 0x304C, 2, 0x5A00, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, { 6, 0x3800, 2, 0, 1, 0}, {0}},
	{{2, 0x304C, 2, 0x5B00, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, { 6, 0x3800, 2, 0, 1, 0}, {0}},
	{{2, 0x304C, 2, 0x6200, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, { 2, 0x3800, 2, 0, 1, 0}, {0}}
};

/* For a2030 type S
 * Full OTP from ISP is 255 bytes
 */
static struct msm_eeprom_memory_map_t a2030_otp_map_s[] = {
	/*     bank offset                    read command                poll                  read mem          saddr*/
	{{2, 0x304C, 2, 0x3300, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, {  8, 0x3800, 2, 0, 1, 0}, {0}},
	{{2, 0x304C, 2, 0x5000, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, { 11, 0x3800, 2, 0, 1, 0}, {0}},
	{{2, 0x304C, 2, 0x5200, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, {  7, 0x3800, 2, 0, 1, 0}, {0}},
	{{2, 0x304C, 2, 0x5300, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, {  7, 0x3800, 2, 0, 1, 0}, {0}},
	{{2, 0x304C, 2, 0x5400, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, {  7, 0x3800, 2, 0, 1, 0}, {0}},
	{{2, 0x304C, 2, 0x5500, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, {  7, 0x3800, 2, 0, 1, 0}, {0}},
	{{2, 0x304C, 2, 0x5800, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, {118, 0x3800, 2, 0, 1, 0}, {0}},
	{{2, 0x304C, 2, 0x5900, 2, 0}, {2, 0x304A, 2, 0x0010, 2, 0}, {0, 0, 0, 0, 0, 0}, {118, 0x3800, 2, 0, 1, 0}, {0}}
};

static int bbry_a2030_eeprom_create_memory_map(struct msm_eeprom_ctrl_t *e_ctrl,
					struct msm_eeprom_memory_block_t *data)
{
	int i, rc = 0;
	struct msm_eeprom_memory_map_t *map;
	uint8_t read_value;

	e_ctrl->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	//select product info region
	rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write(
		&(e_ctrl->i2c_client), A2030_OTP_BANK_SELECT_ADDR,
		0x5000, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s:line %d cci write failed\n", __func__, __LINE__);
		return rc;
	}

	//trigger auto read
	rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write(
		&(e_ctrl->i2c_client), A2030_OTP_BANK_READ_MODE_ADDR,
		0x0010, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s:line %d cci write failed\n", __func__, __LINE__);
		return rc;
	}

	rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
				&(e_ctrl->i2c_client), A2030_OTP_BANK_START_ADDR,
				&read_value, 1);
	if (rc < 0) {
		pr_err("%s: cci read failed\n", __func__);
		return rc;
	}
	e_ctrl->module_id = read_value;
	if (e_ctrl->module_id == 0x01 || e_ctrl->module_id == 0x05) {
		//type S
		data->num_map = sizeof(a2030_otp_map_s)/sizeof(struct msm_eeprom_memory_map_t);
		map = kzalloc(sizeof(a2030_otp_map_s), GFP_KERNEL);
		if (!map) {
			pr_err("%s memory allocation failed line %d\n", __func__, __LINE__);
			return -ENOMEM;
		}
		memcpy(map, a2030_otp_map_s, sizeof(a2030_otp_map_s));
	}
	else if (e_ctrl->module_id == 0x02 || e_ctrl->module_id == 0x06) {
		//type L
		data->num_map = sizeof(a2030_otp_map_l)/sizeof(struct msm_eeprom_memory_map_t);
		map = kzalloc(sizeof(a2030_otp_map_l), GFP_KERNEL);
		if (!map) {
			pr_err("%s memory allocation failed line %d\n", __func__, __LINE__);
			return -ENOMEM;
		}
		memcpy(map, a2030_otp_map_l, sizeof(a2030_otp_map_l));
	}
	else {
		pr_err("%s:line%d invalid a2030 module_id=%d \n", __func__, __LINE__, e_ctrl->module_id);
		return -EINVAL;
	}
	data->map = map;
	for (i = 0; i < data->num_map; i++) {
		data->num_data += map[i].mem.valid_size;
	}
	CDBG("%s:num_map=%d  num_bytes=%d module_id=%d\n", __func__, data->num_map,  data->num_data, e_ctrl->module_id);

	data->mapdata = kzalloc(data->num_data, GFP_KERNEL);
	if (!data->mapdata) {
		pr_err("%s mapdata memory allocation failed line %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR;
	}
	return rc;
ERROR:
	kfree(map);
	memset(data, 0, sizeof(*data));
	return rc;
}

static int bbry_imx135_eeprom_create_memory_map(struct msm_eeprom_ctrl_t *e_ctrl,
					struct msm_eeprom_memory_block_t *data)
{
	int i, rc = 0;
	struct msm_eeprom_memory_map_t *map;

	data->num_map = sizeof(imx135_otp_map_s)/sizeof(struct msm_eeprom_memory_map_t);
	map = kzalloc(sizeof(imx135_otp_map_s), GFP_KERNEL);
	if (!map) {
		pr_err("%s memory allocation failed line %d\n", __func__, __LINE__);
		return -ENOMEM;
	}
	memcpy(map, imx135_otp_map_s, sizeof(imx135_otp_map_s));
	data->map = map;
	for (i = 0; i < data->num_map; i++) {
		data->num_data += map[i].mem.valid_size;
	}
	CDBG("%s:num_map=%d  num_bytes=%d \n", __func__, data->num_map, data->num_data);
	data->mapdata = kzalloc(data->num_data, GFP_KERNEL);
	if (!data->mapdata) {
		pr_err("%s mapdata memory allocation failed line %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR;
	}
	return rc;
ERROR:
	kfree(map);
	memset(data, 0, sizeof(*data));
	return rc;
}

static int bbry_imx230_eeprom_create_memory_map(struct msm_eeprom_ctrl_t *e_ctrl,
					struct msm_eeprom_memory_block_t *data)
{
	int i, rc = 0;
	struct msm_eeprom_memory_map_t *map;

	data->num_map =
		sizeof(imx230_otp_map_s)/sizeof(struct msm_eeprom_memory_map_t);
	map = kzalloc(sizeof(imx230_otp_map_s), GFP_KERNEL);
	if (!map) {
		pr_err("%s memory allocation failed line %d\n", __func__, __LINE__);
		return -ENOMEM;
	}
	memcpy(map, imx230_otp_map_s, sizeof(imx230_otp_map_s));
	data->map = map;
	for (i = 0; i < data->num_map; i++) {
		data->num_data += map[i].mem.valid_size;
	}
	CDBG("%s:num_map=%d  num_bytes=%d \n", __func__,
		data->num_map, data->num_data);

	data->mapdata = kzalloc(data->num_data, GFP_KERNEL);
	if (!data->mapdata) {
		pr_err("%s mapdata memory allocation failed line %d\n",
			__func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR;
	}
	return rc;
ERROR:
	kfree(map);
	memset(data, 0, sizeof(*data));
	return rc;
}

int bbry_read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl,
					struct msm_eeprom_memory_block_t *block)
{
	int rc = 0;
	int j;
	struct msm_eeprom_memory_map_t *emap = block->map;
	uint8_t *memptr = block->mapdata;
	uint8_t retry;
	struct msm_eeprom_board_info *eb_info = NULL;

	if (!e_ctrl) {
		pr_err("%s e_ctrl is NULL", __func__);
		return -EINVAL;
	}
	eb_info = e_ctrl->eboard_info;
	for (j = 0; j < block->num_map; j++) {
		if (emap[j].page.valid_size) {
			e_ctrl->i2c_client.addr_type = emap[j].page.addr_t;
			rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write(
				&(e_ctrl->i2c_client), emap[j].page.addr,
				emap[j].page.data, emap[j].page.data_t);
			if (rc < 0) {
				pr_err("%s: page write failed\n", __func__);
				return rc;
			}
			if (emap[j].page.delay) {
				msleep(emap[j].page.delay);
			}
		}
		if (emap[j].pageen.valid_size) {
			e_ctrl->i2c_client.addr_type = emap[j].pageen.addr_t;
			rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_write(
				&(e_ctrl->i2c_client), emap[j].pageen.addr,
				emap[j].pageen.data, emap[j].pageen.data_t);
			if (rc < 0) {
				pr_err("%s: page enable failed\n", __func__);
				return rc;
			}
			if (emap[j].pageen.delay) {
				msleep(emap[j].pageen.delay);
			}
		}
		if (emap[j].poll.valid_size) {
			e_ctrl->i2c_client.addr_type = emap[j].poll.addr_t;
			retry = 50;
			do {
				rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_poll(
						&(e_ctrl->i2c_client), emap[j].poll.addr,
						emap[j].poll.data, emap[j].poll.data_t);
				if (rc == I2C_COMPARE_MISMATCH) {
					pr_err("%s: sleep 10ms\n", __func__);
					msleep(10);
				} else if (rc < 0) {
					pr_err("%s: poll failed\n", __func__);
					return rc;
				}
			} while ((rc == I2C_COMPARE_MISMATCH) && (retry--));
			if (emap[j].poll.delay) {
				msleep(emap[j].poll.delay);
			}
		}
		if (emap[j].mem.valid_size) {
		    e_ctrl->i2c_client.addr_type = emap[j].mem.addr_t;
		    rc = e_ctrl->i2c_client.i2c_func_tbl->i2c_read_seq(
		        &(e_ctrl->i2c_client), emap[j].mem.addr,
		        memptr, emap[j].mem.valid_size);
		    if (rc < 0) {
		        pr_err("%s: read failed\n", __func__);
		        return rc;
		    }
		    memptr += emap[j].mem.valid_size;
		}
	}
	return rc;
}

/**
  * bbry_eeprom_parse_memory_map() - create a cci read map
  * @e_ctrl:	eeprom control struct
  * @data:	memory block for output
  *
  * This functions parses hardcoded map to fill @data.  It allocates map itself,
  * calculate total data length, and allocates required buffer.
  * It only fills the map, but does not perform actual reading.
  */
int bbry_eeprom_parse_memory_map(struct msm_eeprom_ctrl_t *e_ctrl,
					struct msm_eeprom_memory_block_t *data)
{
	if (!e_ctrl || !data) {
		pr_err("%s Invalid data pointer %d\n", __func__, __LINE__);
		return -EINVAL;
	}
	if (strcmp(e_ctrl->eboard_info->eeprom_name, "bbryimx135") == 0) {
		return bbry_imx135_eeprom_create_memory_map(e_ctrl, data);
	}
	else if (strcmp(e_ctrl->eboard_info->eeprom_name, "bbrya2030") == 0) {
		return bbry_a2030_eeprom_create_memory_map(e_ctrl, data);
	}
	else if (strcmp(e_ctrl->eboard_info->eeprom_name, "bbryimx230") == 0) {
		return bbry_imx230_eeprom_create_memory_map(e_ctrl, data);
	}
	else {
		pr_err("%s: %d Invalid eeprom module %s\n", __func__, __LINE__, e_ctrl->eboard_info->eeprom_name);
		return -EINVAL;
	}
}
