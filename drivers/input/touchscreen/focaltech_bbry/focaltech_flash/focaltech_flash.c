/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2016, FocalTech Systems, Ltd., all rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

 /************************************************************************
*
* File Name: focaltech_apk_node.c
*
* Author:	   Xu YF & ZR, Software Department, FocalTech
*
* Created: 2016-04-01
*
* Modify:
*
* Abstract: the entry of upgrade/download fw
*
************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "../focaltech_comm.h"
#include "focaltech_flash.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FOCALTECH_FLASH_INFO  "File Version of  focaltech_flash.c:  V1.0.0 2016-03-22"


#define FTS_FLASH_AUTO_UPGRADE_FW_EN    1 /*disable: 0, enable: 1*/

#define FTS_MAX_POINTS_2                              2
#define FTS_MAX_POINTS_5                              5
#define FTS_MAX_POINTS_10                            10
#define AUTO_CLB_NEED                                 1
#define AUTO_CLB_NONEED                               0

/*Set defualt IC Name*/
#define FTS_IC_NAME                "FT8707"

/*Set ini file path*/
#define FTS_INI_FILE_PATH             "/sdcard/"

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/

/*******************************************************************************
* Static variables
*******************************************************************************/

struct fts_Upgrade_Info fts_updateinfo[] =
{
	{0x55,0x00,FTS_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000}, 	//"FT5x06"
	{0x08,0x00,FTS_MAX_POINTS_5,AUTO_CLB_NEED,50, 10, 0x79, 0x06, 100, 2000}, 	//"FT5606"
	{0x0a,0x00,FTS_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x07, 10, 1500}, 	//"FT5x16"
	{0x06,0x00,FTS_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x08, 10, 2000}, 	//"FT6x06"
	{0x36,0x00,FTS_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x18, 10, 2000}, 	//"FT6x36"
	{0x55,0x00,FTS_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000}, 	//"FT5x06i"
	{0x14,0x00,FTS_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000}, 	//"FT5336"
	{0x13,0x00,FTS_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000}, 	//"FT3316"
	{0x12,0x00,FTS_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000}, 	//"FT5436i"
	{0x11,0x00,FTS_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000}, 	//"FT5336i"
	{0x54,0x00,FTS_MAX_POINTS_5,AUTO_CLB_NONEED,2, 2, 0x54, 0x2c, 20, 2000}, 	//"FT5x46"
	{0x58,0x00,FTS_MAX_POINTS_5,AUTO_CLB_NONEED,2, 2, 0x58, 0x2c, 20, 2000},	//"FT5822",
	{0x59,0x00,FTS_MAX_POINTS_10,AUTO_CLB_NONEED,30, 50, 0x79, 0x10, 1, 2000},	//"FT5x26",
	{0x86,0x06,FTS_MAX_POINTS_10,AUTO_CLB_NONEED,2, 2, 0x86, 0xA6, 20, 2000},	//"FT8606"
	{0x86,0x07,FTS_MAX_POINTS_10,AUTO_CLB_NONEED,2, 2, 0x86, 0xA7, 20, 2000},	//"FT8607",
	{0x87,0x07,FTS_MAX_POINTS_10,AUTO_CLB_NONEED,2, 2, 0x87, 0xA7, 20, 2000},	//"FT8707"
	{0x87,0x16,FTS_MAX_POINTS_10,AUTO_CLB_NONEED,2, 2, 0x87, 0xA6, 20, 2000},	//"FT8716"
	{0x87,0x36,FTS_MAX_POINTS_10,AUTO_CLB_NONEED,2, 2, 0x87, 0xC6, 20, 2000},	//"FT8736"
	{0xE7,0x16,FTS_MAX_POINTS_10,AUTO_CLB_NONEED,2, 2, 0xE7, 0xA6, 20, 2000},	//"FTE716"
	{0xE7,0x36,FTS_MAX_POINTS_10,AUTO_CLB_NONEED,2, 2, 0xE7, 0xC6, 20, 2000},	//"FTE736"
};
/******************************************************************************* Global variable or extern global variabls/functions
*******************************************************************************/

struct fts_Upgrade_Info fts_upgrade_info_curr;

#if 1
 //#define  FTS_PRAM_BOOT_FILE  "pramboot/FT8716_Pramboot_V0.2_20160127.i"
#define FTS_PRAM_BOOT_FILE  "pramboot/FT8707_Pramboot_V0.4_20160126.i"
unsigned char BUFFER_PRAM_BOOT_FILE[] 	= {
	#include FTS_PRAM_BOOT_FILE
};

#define FTS_BUFFER_I_FILE_1  "i_file1/BOE_4.5INCH_FT8707_0xDA_P01_V0xA2_D0x05_20180301_all.i"
unsigned char BUFFER_I_FILE_1[] 				= {
	#include FTS_BUFFER_I_FILE_1
};
#define FTS_BUFFER_I_FILE_2  "i_file2/sample_file.i"
unsigned char BUFFER_I_FILE_2[] 				= {
	#include FTS_BUFFER_I_FILE_2
};
#define FTS_BUFFER_I_FILE_3  "i_file3/sample_file.i"
unsigned char BUFFER_I_FILE_3[] 				= {
	#include FTS_BUFFER_I_FILE_3
};
#else
unsigned char BUFFER_PRAM_BOOT_FILE[] 	= {
	#include "pramboot/sample_file.i"
};
unsigned char BUFFER_I_FILE_1[] 				= {
	#include "i_file1/FT5822_Ref_V01_D01_20160323_app.i"
};

unsigned char BUFFER_I_FILE_2[] 				= {
	#include "i_file2/sample_file.i"
};
unsigned char BUFFER_I_FILE_3[] 				= {
	#include "i_file3/sample_file.i"
};
#endif
unsigned int g_len_pram_boot_file;
unsigned int g_len_i_file_1;
unsigned int g_len_i_file_2;
unsigned int g_len_i_file_3;

/*******************************************************************************
* Static function prototypes
*******************************************************************************/
#if 0
static int fts_flash_get_fw_file_size(char *firmware_name);
static int fts_flash_read_fw_file(char *firmware_name,unsigned char *firmware_buf);
#endif
static int fts_flash_get_chip_id_in_upgrade_mode( u8 *ucPChipID);
static void fts_flash_get_upgrade_info(void);
static int fts_flash_compare_chip_id(unsigned char chip_id, unsigned char chip_id2);
static int fts_flash_auto_upgrade_fw(void);

/*******************************************************************************
* functions body
*******************************************************************************/
int fts_flash_init(struct i2c_client *client)
{
	int i_ret = 0;

	FTS_COMMON_DBG("[focal] %s ",  FOCALTECH_FLASH_INFO);	//show version
	FTS_COMMON_DBG("");//default print: current function name and line number

	g_len_pram_boot_file = FILE_LEN(BUFFER_PRAM_BOOT_FILE);
	g_len_i_file_1 = FILE_LEN(BUFFER_I_FILE_1);
	g_len_i_file_2 = FILE_LEN(BUFFER_I_FILE_2);
	g_len_i_file_3 = FILE_LEN(BUFFER_I_FILE_3);

	fts_irq_enable(false);

	msleep(10);

	/*get chip id, and init fts_Upgrade_Info*/
	fts_flash_get_upgrade_info();

	/*check if need to upgrade fw*/
	i_ret = fts_flash_auto_upgrade_fw();

	fts_irq_enable(true);

	return 0;
}

int fts_flash_exit(void)
{
	return 0;
}

/************************************************************************
*   Name: fts_flash_get_upgrade_info
* Brief: decide which ic
* Input: no
* Output: get ic info in fts_upgrade_info_curr
* Return: no
***********************************************************************/
static void fts_flash_get_upgrade_info(void)
{
	u8 chip_id, chip_id2;
	int ret = 0;
	int index = 0;
	unsigned int ic_code = 0;

	fts_upgrade_info_curr.chip_id = 0;//set init value

	fts_protocol_windows_to_android();
	//i2c_smbus_read_i2c_block_data(i2c_FTS_REG_CHIP_ID,1,&chip_id);
	ret = fts_read_reg(FTS_REG_CHIP_ID,&chip_id);
	if (ret<0)
	{
		FTS_COMMON_DBG("[Focal][Touch] read value fail");
		//return ret;
	}
	FTS_COMMON_DBG("%s chip_id = %x",  __func__, chip_id);

	ret = fts_read_reg(FTS_REG_CHIP_ID2, &chip_id2);
	if (ret<0)
	{
		FTS_COMMON_DBG("[Focal][Touch] read value fail");
		//return ret;
	}
	FTS_COMMON_DBG("%s chip_id2 = %x",  __func__, chip_id2);

	index = fts_flash_compare_chip_id(chip_id, chip_id2);
	if(index >= 0)
	{
		memcpy(&fts_upgrade_info_curr, &fts_updateinfo[index], sizeof(struct fts_Upgrade_Info));
		FTS_COMMON_DBG("Init upgrade info1, chip_id = 0x%02x, chip_id2 = 0x%02x, index=%d, \
			fts_upgrade_info_curr.chip_id = 0x%02x, fts_upgrade_info_curr.chip_id2 = 0x%02x",
			chip_id, chip_id2, index, fts_upgrade_info_curr.chip_id, fts_upgrade_info_curr.chip_id2);
	}
	else
	{
		ic_code = fts_ic_table_get_ic_code_from_ic_name(FTS_IC_NAME);
		fts_ic_table_get_chip_id_from_ic_code(ic_code, &chip_id, &chip_id2);
		FTS_COMMON_DBG("Get Chip ID form FTS_IC_NAME, FTS_IC_NAME = %s",  FTS_IC_NAME);
		index = fts_flash_compare_chip_id(chip_id, chip_id2);
		if(index >= 0)
		{
			memcpy(&fts_upgrade_info_curr, &fts_updateinfo[index], sizeof(struct fts_Upgrade_Info));
			FTS_COMMON_DBG("Init upgrade info1, chip_id = 0x%02x, chip_id2 = 0x%02x, index=%d, \
				fts_upgrade_info_curr.chip_id = 0x%02x, fts_upgrade_info_curr.chip_id2 = 0x%02x",
				chip_id, chip_id2, index, fts_upgrade_info_curr.chip_id, fts_upgrade_info_curr.chip_id2);
		}
		else
		{
			FTS_COMMON_DBG("Failed to init upgrade info, chip_id = 0x%02x, chip_id2 = 0x%02x",  chip_id, chip_id2);
		}
	}

	return;
}
/************************************************************************
*   Name: fts_flash_compare_chip_id
* Brief: compare chip id with upgrade info, return the index of the same id
* Input: no
* Output: get ic info in fts_upgrade_info_curr
* Return: return value <0: no the same id; return value>=0: found the same id
***********************************************************************/
static int fts_flash_compare_chip_id(unsigned char chip_id, unsigned char chip_id2)
{
	int i = 0;
	int index = -1;
	if(fts_ic_table_need_chip_id2(chip_id) < 0)
	{
		FTS_COMMON_DBG("No need chip_id2. , chip_id = 0x%02x, chip_id2 = 0x%02x",  chip_id, chip_id2);
		for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct fts_Upgrade_Info);i++)
		{
			if(chip_id==fts_updateinfo[i].chip_id)
			{
				index = i;
				break;
			}
		}
	}
	else
	{
		FTS_COMMON_DBG("Need chip_id2. , chip_id = 0x%02x, chip_id2 = 0x%02x",  chip_id, chip_id2);
		for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct fts_Upgrade_Info);i++)
		{
			if(chip_id==fts_updateinfo[i].chip_id
				&& chip_id2 ==fts_updateinfo[i].chip_id2)
			{
				index = i;
				break;
			}
		}
	}
	FTS_COMMON_DBG("index = %d",  index);
	return index;
}

#if 0
/*
*	note:the firmware default path is sdcard.
	if you want to change the dir, please modify by yourself.
*/
/************************************************************************
* Name: fts_flash_get_fw_file_size
* Brief:  get file size
* Input: file name
* Output: no
* Return: file size
***********************************************************************/
static int fts_flash_get_fw_file_size(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];

	memset(filepath, 0, sizeof(filepath));
       sprintf(filepath, "%s%s", FTS_INI_FILE_PATH, firmware_name);
	if (NULL == pfile)
	{
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if (IS_ERR(pfile))
	{
		sprintf(filepath, "%s",  firmware_name);
		if (NULL == pfile)
		{
			pfile = filp_open(filepath, O_RDONLY, 0);
		}
		if (IS_ERR(pfile))
		{
			FTS_COMMON_DBG("error occured while opening file %s.",  filepath);
			return -EIO;
		}
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

/************************************************************************
* Name: fts_flash_read_fw_file
* Brief:  read firmware buf for .bin file.
* Input: file name, data buf
* Output: data buf
* Return: 0
***********************************************************************/
/*
	note:the firmware default path is sdcard.
	if you want to change the dir, please modify by yourself.
*/
static int fts_flash_read_fw_file(char *firmware_name,unsigned char *firmware_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FTS_INI_FILE_PATH, firmware_name);
	if (NULL == pfile)
	{
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if (IS_ERR(pfile))
	{
		sprintf(filepath, "%s",  firmware_name);
		if (NULL == pfile)
		{
			pfile = filp_open(filepath, O_RDONLY, 0);
		}
		if (IS_ERR(pfile))
		{
			FTS_COMMON_DBG("error occured while opening file %s.",  filepath);
			return -EIO;
		}
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, firmware_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}
#endif

/************************************************************************
* Name: fts_flash_upgrade_with_bin_file
* Brief:  upgrade with *_app.bin file
* Input: i2c info, file name
* Output: no
* Return: success =0
***********************************************************************/
int fts_flash_upgrade_with_bin_file( char *firmware_name)
{
	u8 *pbt_buf = NULL;
	int i_ret=0;
	unsigned int ic_code = 0;
#if 0
	int fwsize = fts_flash_get_fw_file_size(firmware_name);

	FTS_COMMON_DBG("\n FTS zax ID=%d size=%d ", fts_upgrade_info_curr.chip_id,fwsize);
	if (fwsize <= 0)
	{
		FTS_COMMON_DBG( "%s ERROR:Get firmware size failed", __func__);
		return -EIO;
	}

	/*========= FW upgrade ========================*/
	pbt_buf = (unsigned char *)kmalloc(fwsize + 1, GFP_ATOMIC);
	if (fts_flash_read_fw_file(firmware_name, pbt_buf))
	{
		FTS_COMMON_DBG( "%s() - ERROR: request_firmware failed", __func__);
		kfree(pbt_buf);
		return -EIO;
	}
#else
	const struct firmware *fw_entry = NULL;
	int fwsize;

	i_ret = request_firmware(&fw_entry, firmware_name, &fts_i2c_client->dev);
	if (i_ret != 0) {
		FTS_COMMON_DBG( "%s: Firmware image %s not available, err=%d",
			__func__, firmware_name, i_ret);
		return -EIO;
	}
	FTS_COMMON_DBG( "%s: Firmware image size = %d", __func__, (int)fw_entry->size);
	fwsize = fw_entry->size;
	pbt_buf = (u8 *)fw_entry->data;

	if ((pbt_buf == NULL) || (fwsize == 0)) {
		FTS_COMMON_DBG( "%s: Invliad FW image. buff=%p, size=%d",
			 __func__, pbt_buf, (int)fw_entry->size);
		return -EIO;
	}
#endif
	/* call the upgrade function */

	ic_code = fts_ic_table_get_ic_code_from_chip_id(fts_upgrade_info_curr.chip_id, fts_upgrade_info_curr.chip_id2);
	switch(ic_code >> 4)
	{
		case (IC_FT5306 >> 4):
		case (IC_FT5606 >> 4):
		case (IC_FT5X16 >> 4):
			i_ret = fts_ft5x06_upgrade_with_bin_file(pbt_buf, fwsize);
			break;
		case (IC_FT5X36>> 4):
			i_ret = fts_ft5x36_upgrade_with_bin_file(pbt_buf, fwsize);
			break;
		case (IC_FT6X06>> 4):
			i_ret = fts_ft6x06_upgrade_with_bin_file(pbt_buf, fwsize);
			break;
		case (IC_FT6X36>> 4):
			i_ret = fts_ft6x36_upgrade_with_bin_file(pbt_buf, fwsize);
			break;
		case (IC_FT5X46>> 4):
			i_ret = fts_ft5x46_upgrade_with_bin_file(pbt_buf, fwsize);
			break;
		case (IC_FT5822>> 4):
			i_ret = fts_ft5822_upgrade_with_bin_file(pbt_buf, fwsize);
			break;
		case (IC_FT8606 >> 4):
			i_ret = fts_ft8606_upgrade_with_bin_file(pbt_buf, fwsize);
			break;
		case (IC_FT8607 >> 4):
			i_ret = fts_ft8607_upgrade_with_bin_file(pbt_buf, fwsize);
			break;
		case (IC_FT8707 >> 4):
			i_ret = fts_ft8707_upgrade_with_bin_file(pbt_buf, fwsize);
			break;
		case (IC_FT8716 >> 4):
			i_ret = fts_ft8716_upgrade_with_bin_file(pbt_buf, fwsize);
			break;
		case (IC_FT8736 >> 4):
			i_ret = fts_ft8736_upgrade_with_bin_file(pbt_buf, fwsize);
			break;
		case (IC_FTE716 >> 4):
			i_ret = fts_ftE716_upgrade_with_bin_file(pbt_buf, fwsize);
			break;
		case (IC_FTE736 >> 4):
			i_ret = fts_ftE736_upgrade_with_bin_file(pbt_buf, fwsize);
			break;
		default:
			FTS_COMMON_DBG("Failed to upgrade fw, can't find the ic series by ic code. ic_code = %d", ic_code);
			return -1;
			break;
	}

	if (i_ret != 0)
		FTS_COMMON_DBG( "%s() - ERROR:[FTS] Failed to upgrade ..",
					__func__);

	fts_enter_work_mode();
#if 0
	kfree(pbt_buf);
#endif
	return i_ret;
}
/************************************************************************
* Name: fts_flash_get_i_file_version
* Brief:  get .i file version
* Input: no
* Output: no
* Return: fw version
***********************************************************************/
int fts_flash_get_i_file_version(void)
{
	#if 1
	int i_ret=0;
	unsigned int ic_code = 0;

	ic_code = fts_ic_table_get_ic_code_from_chip_id(fts_upgrade_info_curr.chip_id, fts_upgrade_info_curr.chip_id2);
	switch(ic_code >> 4)
	{
		case (IC_FT5306 >> 4):
		case (IC_FT5606 >> 4):
		case (IC_FT5X16 >> 4):
			i_ret = fts_ft5x06_get_i_file_version();
			break;
		case (IC_FT5X36>> 4):
			i_ret = fts_ft5x36_get_i_file_version();
			break;
		case (IC_FT6X06>> 4):
			i_ret = fts_ft6x06_get_i_file_version();
			break;
		case (IC_FT6X36>> 4):
			i_ret = fts_ft6x36_get_i_file_version();
			break;
		case (IC_FT5X46>> 4):
			i_ret = fts_ft5x46_get_i_file_version();
			break;
		case (IC_FT5822>> 4):
			i_ret = fts_ft5822_get_i_file_version();
			break;
		case (IC_FT8606 >> 4):
			i_ret = fts_ft8606_get_i_file_version();
			break;
		case (IC_FT8607 >> 4):
			i_ret = fts_ft8607_get_i_file_version();
			break;
		case (IC_FT8707 >> 4):
			i_ret = fts_ft8707_get_i_file_version();
			break;
		case (IC_FT8716 >> 4):
			i_ret = fts_ft8716_get_i_file_version();
			break;
		case (IC_FT8736 >> 4):
			i_ret = fts_ft8736_get_i_file_version();
			break;
		case (IC_FTE716 >> 4):
			i_ret = fts_ftE716_get_i_file_version();
			break;
		case (IC_FTE736 >> 4):
			i_ret = fts_ftE736_get_i_file_version();
			break;
		default:
			FTS_COMMON_DBG("Failed to get fw version, can't find the ic series by ic code. ic_code = %d", ic_code);
			return -1;
			break;
	}
	return i_ret;
	#else

	u16 ui_sz;
	ui_sz = sizeof(CTPM_FW);
	if (ui_sz > 2)
	{
	    if(fts_upgrade_info_curr.chip_id==0x36)
                return CTPM_FW[0x10A];
	    if(fts_upgrade_info_curr.chip_id==0x86  || fts_upgrade_info_curr.chip_id==0x87)
                return CTPM_FW[0x10E];
	    else if(fts_upgrade_info_curr.chip_id==0x58)
                return CTPM_FW[0x1D0A];	//0x010A + 0x1C00
	    else
		return CTPM_FW[ui_sz - 2];
	}

	return 0x00;						/*default value */
	#endif
}

/************************************************************************
* Name: fts_flash_upgrade_with_i_file
* Brief:  upgrade with *.i file
* Input: i2c info
* Output: no
* Return: fail <0
***********************************************************************/
int fts_flash_upgrade_with_i_file(void)
{
	int i_ret=0;
	unsigned int ic_code = 0;

	ic_code = fts_ic_table_get_ic_code_from_chip_id(fts_upgrade_info_curr.chip_id, fts_upgrade_info_curr.chip_id2);
	FTS_COMMON_DBG("fts_flash_upgrade_with_i_file. chip_id = 0x%02x, chip_id2 = 0x%02x, ic_code = %d",
		fts_upgrade_info_curr.chip_id, fts_upgrade_info_curr.chip_id2, ic_code);
	switch(ic_code >> 4)
	{
		case (IC_FT5306 >> 4):
		case (IC_FT5606 >> 4):
		case (IC_FT5X16 >> 4):
			i_ret = fts_ft5x06_upgrade_with_i_file();
			break;
		case (IC_FT5X36>> 4):
			i_ret = fts_ft5x36_upgrade_with_i_file();
			break;
		case (IC_FT6X06>> 4):
			i_ret = fts_ft6x06_upgrade_with_i_file();
			break;
		case (IC_FT6X36>> 4):
			i_ret = fts_ft6x36_upgrade_with_i_file();
			break;
		case (IC_FT5X46>> 4):
			i_ret = fts_ft5x46_upgrade_with_i_file();
			break;
		case (IC_FT5822>> 4):
			i_ret = fts_ft5822_upgrade_with_i_file();
			break;
		case (IC_FT8606 >> 4):
			i_ret = fts_ft8606_upgrade_with_i_file();
			break;
		case (IC_FT8607 >> 4):
			i_ret = fts_ft8607_upgrade_with_i_file();
			break;
		case (IC_FT8707 >> 4):
			i_ret = fts_ft8707_upgrade_with_i_file();
			break;
		case (IC_FT8716 >> 4):
			i_ret = fts_ft8716_upgrade_with_i_file();
			break;
		case (IC_FT8736 >> 4):
			i_ret = fts_ft8736_upgrade_with_i_file();
			break;
		case (IC_FTE716 >> 4):
			i_ret = fts_ftE716_upgrade_with_i_file();
			break;
		case (IC_FTE736 >> 4):
			i_ret = fts_ftE736_upgrade_with_i_file();
			break;
		default:
			FTS_COMMON_DBG("Failed to upgrade fw, can't find the ic series by ic code. chip_id = %d, ic_code = %d",
				fts_upgrade_info_curr.chip_id, ic_code);
			return -1;
			break;
	}
	return i_ret;
}


/************************************************************************
* Name: fts_flash_get_chip_id_in_upgrade_mode
* Brief:  read chip ID
* Input: i2c info
* Output: chip ID
* Return: fail <0
***********************************************************************/
static int fts_flash_get_chip_id_in_upgrade_mode( u8 *ucPChipID)
{
	u8 reg_val[4] = {0};
	u32 i = 0;
	u8 auc_i2c_write_buf[10];
	int i_ret;
	int index = 0;

	*ucPChipID = 0;
	i_ret = fts_protocol_windows_to_android();
	if (i_ret == 0)
	{
		FTS_COMMON_DBG("hidi2c change to stdi2c fail ! ");
	}

	for (i = 0; i < FTS_UPGRADE_LOOP; i++)
	{
		/*********Step 1:Reset  CTPM *****/
		fts_write_reg( 0xfc, FTS_UPGRADE_AA);
		msleep(fts_upgrade_info_curr.delay_aa);
		fts_write_reg( 0xfc, FTS_UPGRADE_55);
		msleep(200);
		/*********Step 2:Enter upgrade mode *****/
		i_ret = fts_protocol_windows_to_android();
		if (i_ret == 0)
		{
			FTS_COMMON_DBG("hidi2c change to stdi2c fail ! ");
		}
		msleep(10);
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		i_ret = fts_i2c_write( auc_i2c_write_buf, 2);
		if (i_ret < 0) {
			FTS_COMMON_DBG("failed writing  0x55 and 0xaa ! ");
			continue;
		}
		/*********Step 3:check READ-ID***********************/
		msleep(10);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read( auc_i2c_write_buf, 4, reg_val, 2);

		index = fts_flash_compare_chip_id(reg_val[0], reg_val[1]);
		if(index >= 0)
		{
			memcpy(&fts_upgrade_info_curr, &fts_updateinfo[index], sizeof(struct fts_Upgrade_Info));
			*ucPChipID=reg_val[0];
			FTS_COMMON_DBG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x, curr:0x%x",  reg_val[0], reg_val[1], fts_upgrade_info_curr.upgrade_id_1);
			break;
		}
		else
		{
			FTS_COMMON_DBG("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x",  reg_val[0], reg_val[1]);
			continue;
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;

	msleep(10);

	/*********Step 5: reset the new FW***********************/
	FTS_COMMON_DBG("Step 4: reset the new FW");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write( auc_i2c_write_buf, 1);
	msleep(200);
	i_ret = fts_protocol_windows_to_android();
	if (i_ret == 0)
	{
		FTS_COMMON_DBG("hidi2c change to stdi2c fail ! ");
	}
	msleep(10);
	return 0;
}

/************************************************************************
* Name: fts_flash_auto_upgrade_fw
* Brief:  auto upgrade
* Input: i2c info
* Output: no
* Return: 0
***********************************************************************/
static int fts_flash_auto_upgrade_fw(void)
{
	u8 uc_host_fm_ver = 0;
	u8 uc_tp_fm_ver;
	int i_ret;
	u8 uc_chip_id=0;

	if (!FTS_FLASH_AUTO_UPGRADE_FW_EN)
	{
		FTS_COMMON_DBG("Auto upgrade FW: Disable");
		return 0;
	}

	fts_read_reg(FTS_REG_CHIP_ID, &uc_chip_id);

	if (uc_chip_id == fts_upgrade_info_curr.chip_id)	/*call fts_flash_get_upgrade_info in probe function firstly.*/
	{
		fts_read_reg( FTS_REG_FW_VER, &uc_tp_fm_ver);

		uc_host_fm_ver = fts_flash_get_i_file_version();
		FTS_COMMON_DBG("[out] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x", uc_tp_fm_ver, uc_host_fm_ver);

		if (uc_tp_fm_ver < uc_host_fm_ver ) {
			msleep(100);
			FTS_COMMON_DBG("[in] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x", uc_tp_fm_ver, uc_host_fm_ver);

			i_ret = fts_flash_upgrade_with_i_file();
			fts_enter_work_mode();
			if (i_ret == 0)
			{
				msleep(300);
				//uc_host_fm_ver = fts_flash_get_i_file_version();
				fts_read_reg( FTS_REG_FW_VER, &uc_tp_fm_ver);
				FTS_COMMON_DBG("[FTS] upgrade to new version 0x%x", uc_tp_fm_ver);
			}
			else
			{
				FTS_COMMON_DBG("[FTS] upgrade failed ret=%d.",  i_ret);
				return -EIO;
			}
		}
	}
	else
	{
		i_ret = fts_flash_get_chip_id_in_upgrade_mode( &uc_chip_id);
		FTS_COMMON_DBG("[FTS] read chip id from boot is 0x%x", uc_chip_id);

		i_ret = fts_flash_upgrade_with_i_file();
		fts_enter_work_mode();
		if (i_ret == 0)
		{
			msleep(300);
			//uc_host_fm_ver = fts_flash_get_i_file_version();
			fts_read_reg( FTS_REG_FW_VER, &uc_tp_fm_ver);
			FTS_COMMON_DBG("[FTS] upgrade to new version 0x%x", uc_tp_fm_ver);
		}
		else
		{
			FTS_COMMON_DBG("[FTS] upgrade failed ret=%d.",  i_ret);
			return -EIO;
		}

	}

	return 0;
}
