/*
 *
 * FocalTech TouchScreen driver.
 * 
 * Copyright (c) 2010-2015, Focaltech Ltd. All rights reserved.
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

 /*******************************************************************************
*
* File Name: Focaltech_flash.c
*
* Author: Xu YongFeng
*
* Created: 2015-01-29
*   
* Modify by mshl on 2015-03-20
*
* Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "focaltech_comm.h"
#include "focaltech_core.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FTS_AUTO_UPGRADE_INFO  "Auto_Upgrade_Version  V1.1.0 2016-02-24"

#define FTS_REG_FW_MAJ_VER									0xB1
#define FTS_REG_FW_MIN_VER									0xB2
#define FTS_REG_FW_SUB_MIN_VER								0xB3
#define FTS_FW_MIN_SIZE										8
#define FTS_FW_MAX_SIZE										(54 * 1024)
/* Firmware file is not supporting minor and sub minor so use 0 */
#define FTS_FW_FILE_MAJ_VER(x)									((x)->data[(x)->size - 2])
#define FTS_FW_FILE_MIN_VER(x)									0
#define FTS_FW_FILE_SUB_MIN_VER(x) 							0
#define FTS_FW_FILE_VENDOR_ID(x)								((x)->data[(x)->size - 1])
#define FTS_FW_FILE_MAJ_VER_FT6X36(x)							((x)->data[0x10a])
#define FTS_FW_FILE_VENDOR_ID_FT6X36(x)						((x)->data[0x108])
#define FTS_MAX_TRIES											5
#define FTS_RETRY_DLY											20
#define FTS_MAX_WR_BUF										10
#define FTS_MAX_RD_BUF											2
#define FTS_FW_PKT_META_LEN									6
#define FTS_FW_PKT_DLY_MS										20
#define FTS_FW_LAST_PKT										0x6ffa
#define FTS_ERASE_DLY_MS										100
#define FTS_55_AA_DLY_NS										5000
#define FTS_CAL_START											0x04
#define FTS_CAL_FIN												0x00
#define FTS_CAL_STORE											0x05
#define FTS_CAL_RETRY											100
#define FTS_REG_CAL												0x00
#define FTS_CAL_MASK											0x70
#define FTS_BLOADER_SIZE_OFF									12
#define FTS_BLOADER_NEW_SIZE									30
#define FTS_DATA_LEN_OFF_OLD_FW								8
#define FTS_DATA_LEN_OFF_NEW_FW								14
#define FTS_FINISHING_PKT_LEN_OLD_FW							6
#define FTS_FINISHING_PKT_LEN_NEW_FW							12
#define FTS_MAGIC_BLOADER_Z7									0x7bfa
#define FTS_MAGIC_BLOADER_LZ4									0x6ffa
#define FTS_MAGIC_BLOADER_GZF_30								0x7ff4
#define FTS_MAGIC_BLOADER_GZF									0x7bf4
#define FTS_REG_ECC												0xCC
#define FTS_RST_CMD_REG2										0xBC
#define FTS_READ_ID_REG										0x90
#define FTS_ERASE_APP_REG										0x61
#define FTS_ERASE_PARAMS_CMD									0x63
#define FTS_FW_WRITE_CMD										0xBF
#define FTS_REG_RESET_FW										0x07
#define FTS_RST_CMD_REG1										0xFC
#define FTS_FACTORYMODE_VALUE									0x40
#define FTS_WORKMODE_VALUE									0x00
#define FTS_APP_INFO_ADDR										0xd7f8
#define BL_VERSION_LZ4        										0
#define BL_VERSION_Z7        										1
#define BL_VERSION_GZF       						 				2
#define FTS_REG_FW_VENDOR_ID 									0xA8
//#define MAX_R_FLASH_SIZE										120
//#define FTS_SETTING_BUF_LEN      								120
#define ERROR_CODE_OK 											0
#define FTS_UPGRADE_LOOP										30
#define FTS_MAX_POINTS_2                        							2
#define FTS_MAX_POINTS_5                        							5
#define FTS_MAX_POINTS_10                        						10
#define AUTO_CLB_NEED                           							1
#define AUTO_CLB_NONEED                         							0
#define FTS_UPGRADE_AA											0xAA
#define FTS_UPGRADE_55											0x55
#define HIDTOI2C_DISABLE										0
#define FTXXXX_INI_FILEPATH_CONFIG 							"/sdcard/"

#define LEN_FLASH_ECC_MAX 					0xFFFE

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/
extern int apk_debug_flag;
/*******************************************************************************
* Static variables
*******************************************************************************/
#if FT_TP
u8 ucPVendorID=0;
#endif
static unsigned char aucFW_PRAM_BOOT[] 	= {
	#include "FT8736_Pramboot_V0.2_20151111.i"
};
static unsigned char CTPM_FW[] 				= {
	#include "FT8607_CPT_V01_D00_20160119_app.i"
};
#if FT_TP
static unsigned char CTPM_FW_1[] 				= {
	#include "FT_Upgrade_App_1.i"
};
static unsigned char CTPM_FW_2[] 				= {
	#include "FT_Upgrade_App_2.i"
};
#endif
 struct fts_Upgrade_Info fts_updateinfo[] =
{
       {0x55,FTS_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000}, 		//"FT5x06"
       {0x08,FTS_MAX_POINTS_5,AUTO_CLB_NEED,50, 10, 0x79, 0x06, 100, 2000}, 	//"FT5606"
	{0x0a,FTS_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x07, 10, 1500}, 		//"FT5x16"
	{0x06,FTS_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x08, 10, 2000}, 	//"FT6x06"
	{0x36,FTS_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x18, 10, 2000}, 	//"FT6x36"
	{0x55,FTS_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000}, 		//"FT5x06i"
	{0x14,FTS_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000}, 	//"FT5336"
	{0x13,FTS_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000}, 	//"FT3316"
	{0x12,FTS_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000}, 	//"FT5436i"
	{0x11,FTS_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000}, 	//"FT5336i"
	{0x54,FTS_MAX_POINTS_5,AUTO_CLB_NONEED,2, 2, 0x54, 0x2c, 20, 2000}, 	//"FT5x46"
       {0x58,FTS_MAX_POINTS_5,AUTO_CLB_NONEED,2, 2, 0x58, 0x2c, 20, 2000},		//"FT5822",
	{0x59,FTS_MAX_POINTS_10,AUTO_CLB_NONEED,30, 50, 0x79, 0x10, 1, 2000},	//"FT5x26",
	{0x86,FTS_MAX_POINTS_10,AUTO_CLB_NONEED,2, 2, 0x86, 0xA6, 20, 2000},	//"FT8606" "FT8607",
	{0x87,FTS_MAX_POINTS_10,AUTO_CLB_NONEED,2, 2, 0x87, 0xA6, 20, 2000},	//"FT8716" "FT8736",
	//{0x87,FTS_MAX_POINTS_10,AUTO_CLB_NONEED,2, 2, 0x87, 0xC6, 20, 2000},	//"FT8736",
};
/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
struct fts_Upgrade_Info fts_updateinfo_curr;
/*******************************************************************************
* Static function prototypes
*******************************************************************************/
int fts_6x36_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth);
int fts_6x06_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth);
int fts_5x36_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth);
int fts_5x06_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth);
int fts_5x46_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth);
int fts_5822_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth);
int fts_5x26_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth);
int fts_8606_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth);
int fts_8606_ctpm_fw_write_pram(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth);
int fts_8716_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth);
int fts_8716_ctpm_fw_write_pram(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth);
int hidi2c_to_stdi2c(struct i2c_client * client);

/************************************************************************
*   Name: hidi2c_to_stdi2c
* Brief:  HID to I2C
* Input: i2c info
* Output: no
* Return: fail =0
***********************************************************************/
int hidi2c_to_stdi2c(struct i2c_client * client)
{
	u8 auc_i2c_write_buf[5] = {0};
	int bRet = 0;
	#if HIDTOI2C_DISABLE
	
		return 2;

	#endif
	auc_i2c_write_buf[0] = 0xeb;
	auc_i2c_write_buf[1] = 0xaa;
	auc_i2c_write_buf[2] = 0x09;
	bRet =fts_i2c_write(client, auc_i2c_write_buf, 3);
	msleep(10);
	auc_i2c_write_buf[0] = auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = 0;
	fts_i2c_read(client, auc_i2c_write_buf, 0, auc_i2c_write_buf, 3);
//FTS_COMMON_DBG("Change to STDI2cValue,REG1 = 0x%x,REG2 = 0x%x,REG3 = 0x%x, iRet=%d\n", reg_val[0], reg_val[1], reg_val[2], iRet);
	if(0xeb==auc_i2c_write_buf[0] && 0xaa==auc_i2c_write_buf[1] && 0x08==auc_i2c_write_buf[2])
	{
		FTS_COMMON_DBG("hidi2c_to_stdi2c successful.\n");
		bRet = 1;		
	}
	else 
	{
		FTS_COMMON_DBG("hidi2c_to_stdi2c error.\n");
		bRet = 0;
	}

	return bRet;
}

/*******************************************************************************
*   Name: fts_update_fw_vendor_id
*  Brief:
*  Input:
* Output: None
* Return: None
*******************************************************************************/
void fts_update_fw_vendor_id(struct fts_ts_data *data)
{
	/*struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FTS_REG_FW_VENDOR_ID;
	err = fts_i2c_read(client, &reg_addr, 1, &data->fw_vendor_id, 1);
	if (err < 0)
		FTS_COMMON_DBG( "fw vendor id read failed");*/
}

/*******************************************************************************
*   Name: fts_update_fw_ver
*  Brief:
*  Input:
* Output: None
* Return: None
*******************************************************************************/
void fts_update_fw_ver(struct fts_ts_data *data)
{
	/*struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FTS_REG_FW_VER;
	err = fts_i2c_read(client, &reg_addr, 1, &data->fw_ver[0], 1);
	if (err < 0)
		FTS_COMMON_DBG( "fw major version read failed");

	reg_addr = FTS_REG_FW_MIN_VER;
	err = fts_i2c_read(client, &reg_addr, 1, &data->fw_ver[1], 1);
	if (err < 0)
		FTS_COMMON_DBG( "fw minor version read failed");

	reg_addr = FTS_REG_FW_SUB_MIN_VER;
	err = fts_i2c_read(client, &reg_addr, 1, &data->fw_ver[2], 1);
	if (err < 0)
		FTS_COMMON_DBG( "fw sub minor version read failed");

	FTS_COMMON_DBG( "Firmware version = %d.%d.%d\n",
		data->fw_ver[0], data->fw_ver[1], data->fw_ver[2]);*/
}



/************************************************************************
* Name: fts_ctpm_fw_upgrade_ReadChipID_Array
* Brief:  read chip ID
* Input: i2c info
* Output: chip ID
* Return: fail <0
***********************************************************************/
int fts_ctpm_fw_upgrade_ReadChipID_Array(struct i2c_client *client, u8 *ucPChipID)
{
	u8 reg_val[4] = {0};
	u32 i = 0;
	u32 j = 0;
	u8 auc_i2c_write_buf[10]; 
	int i_ret;

	*ucPChipID = 0;
	i_ret = hidi2c_to_stdi2c(client);
	if (i_ret == 0) 
	{
		FTS_COMMON_DBG("hidi2c change to stdi2c fail ! \n");
	}
	
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) 
	{
		/*********Step 1:Reset  CTPM *****/
		fts_write_reg(client, 0xfc, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);
		fts_write_reg(client, 0xfc, FTS_UPGRADE_55);
		msleep(200);
		/*********Step 2:Enter upgrade mode *****/
		i_ret = hidi2c_to_stdi2c(client);
		if (i_ret == 0) 
		{
			FTS_COMMON_DBG("hidi2c change to stdi2c fail ! \n");
		}
		msleep(10);
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
		if (i_ret < 0) {
			FTS_COMMON_DBG("failed writing  0x55 and 0xaa ! \n");
			continue;
		}
		/*********Step 3:check READ-ID***********************/
		msleep(10);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

		for(j=0;j<sizeof(fts_updateinfo)/sizeof(struct fts_Upgrade_Info);j++)
		{
			if(reg_val[0]==fts_updateinfo[j].CHIP_ID)
			{
				memcpy(&fts_updateinfo_curr, &fts_updateinfo[j], sizeof(struct fts_Upgrade_Info));
				break;
			}
		}

		if(j< sizeof(fts_updateinfo)/sizeof(struct fts_Upgrade_Info))
		{
			//memcpy(&fts_updateinfo_curr, &fts_updateinfo[], sizeof(struct fts_Upgrade_Info));		
			*ucPChipID=reg_val[0];
			FTS_COMMON_DBG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x, curr:0x%x\n", reg_val[0], reg_val[1], fts_updateinfo_curr.upgrade_id_1);
			break;
		} 
		else 
		{
			FTS_COMMON_DBG("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
			continue;
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;
	
	msleep(10);
	
	/*********Step 5: reset the new FW***********************/
	FTS_COMMON_DBG("Step 4: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(200);	
	i_ret = hidi2c_to_stdi2c(client);	
	if (i_ret == 0) 
	{
		FTS_COMMON_DBG("hidi2c change to stdi2c fail ! \n");
	}
	msleep(10);
	return 0;
}


/************************************************************************
* Name: fts_ctpm_fw_upgrade_ReadVendorID
* Brief:  read vendor ID
* Input: i2c info, vendor ID
* Output: no
* Return: fail <0
***********************************************************************/
int fts_ctpm_fw_upgrade_ReadVendorID(struct i2c_client *client, u8 *ucPVendorID)
{
	u8 reg_val[4] = {0};
	u32 i = 0;
	u8 auc_i2c_write_buf[10];
	int i_ret;

	*ucPVendorID = 0;
	i_ret = hidi2c_to_stdi2c(client);
	if (i_ret == 0) 
	{
		FTS_COMMON_DBG("HidI2c change to StdI2c fail ! \n");
	}
	
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) 
	{
		/********* Step 1:Reset  CTPM *****/
		fts_write_reg(client, 0xfc, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);
		fts_write_reg(client, 0xfc, FTS_UPGRADE_55);
		msleep(200);
		/********* Step 2:Enter upgrade mode *****/
		i_ret = hidi2c_to_stdi2c(client);
		if (i_ret == 0) 
		{
			FTS_COMMON_DBG("HidI2c change to StdI2c fail ! \n");
			/*continue;*/
		}
		msleep(10);
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
		if (i_ret < 0) {
			FTS_COMMON_DBG("failed writing  0x55 and 0xaa ! \n");
			continue;
		}
		/********* Step 3:check READ-ID ***********************/
		msleep(10);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1 /*&& reg_val[1] == fts_updateinfo_curr.upgrade_id_2*/) {
			FTS_COMMON_DBG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
			break;
		} 
		else 
		{
			FTS_COMMON_DBG( "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x, curr:0x%x\n", reg_val[0], reg_val[1], fts_updateinfo_curr.upgrade_id_1);
			continue;
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;
	/********* Step 4: read vendor id from app param area ***********************/
	msleep(10);
	auc_i2c_write_buf[0] = 0x03;
	auc_i2c_write_buf[1] = 0x00;
	auc_i2c_write_buf[2] = 0xd7;
	auc_i2c_write_buf[3] = 0x84;
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) 
	{
		fts_i2c_write(client, auc_i2c_write_buf, 4);		/* send param addr */
		msleep(5);
		reg_val[0] = reg_val[1] = 0x00;
		i_ret = fts_i2c_read(client, auc_i2c_write_buf, 0, reg_val, 2);
		if (0 == reg_val[0]) 
		{
			*ucPVendorID = 0;
			FTS_COMMON_DBG("In upgrade Vendor ID Mismatch, REG1 = 0x%x, REG2 = 0x%x, Definition:0x%x, i_ret=%d\n", reg_val[0], reg_val[1], 0, i_ret);
		} 
		else 
		{
			*ucPVendorID = reg_val[0];
			FTS_COMMON_DBG("In upgrade Vendor ID, REG1 = 0x%x, REG2 = 0x%x\n", reg_val[0], reg_val[1]);
			break;
		}
	}
	msleep(50);
	/********* Step 5: reset the new FW ***********************/
	FTS_COMMON_DBG("Step 5: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(200);	/*make sure CTP startup normally */
	i_ret = hidi2c_to_stdi2c(client);	/*Android to Std i2c.*/
	if (i_ret == 0) 
	{
		FTS_COMMON_DBG("HidI2c change to StdI2c fail ! \n");
	}
	msleep(10);
	return 0;
}

/************************************************************************
* Name: fts_ctpm_fw_upgrade_ReadProjectCode
* Brief:  read project code
* Input: i2c info, project code
* Output: no
* Return: fail <0
***********************************************************************/
int fts_ctpm_fw_upgrade_ReadProjectCode(struct i2c_client *client, char *pProjectCode)
{
	u8 reg_val[4] = {0};
	u32 i = 0;
	u8 j = 0;
	u8 auc_i2c_write_buf[10];
	int i_ret;
	u32 temp;
	i_ret = hidi2c_to_stdi2c(client);
	if (i_ret == 0) 
	{
		FTS_COMMON_DBG("HidI2c change to StdI2c fail ! \n");
	}	
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) 
	{
		/********* Step 1:Reset  CTPM *****/
		fts_write_reg(client, 0xfc, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);
		fts_write_reg(client, 0xfc, FTS_UPGRADE_55);
		msleep(200);
		/********* Step 2:Enter upgrade mode *****/
		i_ret = hidi2c_to_stdi2c(client);
		if (i_ret == 0) 
		{
			FTS_COMMON_DBG("HidI2c change to StdI2c fail ! \n");
			/*continue;*/
		}
		msleep(10);
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
		if (i_ret < 0) 
		{
			FTS_COMMON_DBG("failed writing  0x55 and 0xaa ! \n");
			continue;
		}
		/********* Step 3:check READ-ID ***********************/
		msleep(10);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1 && reg_val[1] == fts_updateinfo_curr.upgrade_id_2) 
		{
			FTS_COMMON_DBG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
			break;
		}
		else
		{
			FTS_COMMON_DBG( "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
			continue;
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;
	/********* Step 4: read vendor id from app param area ***********************/
	msleep(10);
	/*auc_i2c_write_buf[0] = 0x03;
	auc_i2c_write_buf[1] = 0x00;
	auc_i2c_write_buf[2] = 0xd7;
	auc_i2c_write_buf[3] = 0x84;
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		fts_i2c_write(client, auc_i2c_write_buf, 4);     //send param addr
		msleep(5);
		reg_val[0] = reg_val[1] = 0x00;
		i_ret = fts_i2c_read(client, auc_i2c_write_buf, 0, reg_val, 2);
		if (0 != reg_val[0]) {
			*pProjectCode=0;
			FTS_COMMON_DBG("In upgrade Vendor ID Mismatch, REG1 = 0x%x, REG2 = 0x%x, Definition:0x%x, i_ret=%d\n", reg_val[0], reg_val[1], 0, i_ret);
		} else {
			*pProjectCode=reg_val[0];
			FTS_COMMON_DBG("In upgrade Vendor ID, REG1 = 0x%x, REG2 = 0x%x\n", reg_val[0], reg_val[1]);
			break;
		}
	}
	*/
	/* read project code */
	auc_i2c_write_buf[0] = 0x03;
	auc_i2c_write_buf[1] = 0x00;
	for (j = 0; j < 33; j++) 
	{
		/*
		//if (is_5336_new_bootloader == BL_VERSION_Z7 || is_5336_new_bootloader == BL_VERSION_GZF)
			//temp = 0x07d0 + j;
		//else
		*/
		temp = 0xD7A0 + j;
		auc_i2c_write_buf[2] = (u8)(temp>>8);
		auc_i2c_write_buf[3] = (u8)temp;
		fts_i2c_read(client, auc_i2c_write_buf, 4, pProjectCode+j, 1);
		if (*(pProjectCode+j) == '\0')
			break;
	}
	FTS_COMMON_DBG("project code = %s \n", pProjectCode);
	msleep(50);
	/********* Step 5: reset the new FW ***********************/
	FTS_COMMON_DBG("Step 5: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(200);						/* make sure CTP startup normally */
	i_ret = hidi2c_to_stdi2c(client);	/* Android to Std i2c. */
	if (i_ret == 0) 
	{
		FTS_COMMON_DBG("HidI2c change to StdI2c fail ! \n");
	}
	msleep(10);
	return 0;
}

/************************************************************************
*   Name: fts_get_upgrade_array
* Brief: decide which ic
* Input: no
* Output: get ic info in fts_updateinfo_curr
* Return: no
***********************************************************************/
void fts_get_upgrade_array(void)
{
	u8 chip_id;
	u32 i;
	int ret = 0;

	hidi2c_to_stdi2c(fts_i2c_client);
	//i2c_smbus_read_i2c_block_data(i2c_client,FTS_REG_CHIP_ID,1,&chip_id);
	ret = fts_read_reg(fts_i2c_client, FTS_REG_CHIP_ID,&chip_id);
	if (ret<0) 
	{
		FTS_COMMON_DBG("[Focal][Touch] read value fail");
		//return ret;
	}
	FTS_COMMON_DBG("%s chip_id = %x\n", __func__, chip_id);

	for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct fts_Upgrade_Info);i++)
	{
		if(chip_id==fts_updateinfo[i].CHIP_ID)	// 0x54==fts_updateinfo[i].CHIP_ID
		{
			memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct fts_Upgrade_Info));
			break;
		}
	}

	if(i >= sizeof(fts_updateinfo)/sizeof(struct fts_Upgrade_Info))
	{
		memcpy(&fts_updateinfo_curr, &fts_updateinfo[0], sizeof(struct fts_Upgrade_Info));
	}
}

/************************************************************************
* Name: fts_ctpm_auto_clb
* Brief:  auto calibration
* Input: i2c info
* Output: no
* Return: 0
***********************************************************************/
int fts_ctpm_auto_clb(struct i2c_client *client)
{
	unsigned char uc_temp = 0x00;
	unsigned char i = 0;

	/* start auto CLB */
	msleep(200);

	fts_write_reg(client, 0, FTS_FACTORYMODE_VALUE);
	/* make sure already enter factory mode */
	msleep(100);
	/* write command to start calibration */
	fts_write_reg(client, 2, 0x4);
	msleep(300);
	if ((fts_updateinfo_curr.CHIP_ID==0x11) ||(fts_updateinfo_curr.CHIP_ID==0x12) ||(fts_updateinfo_curr.CHIP_ID==0x13) ||(fts_updateinfo_curr.CHIP_ID==0x14)) //5x36,5x36i
	{
		for(i=0;i<100;i++)
		{
			fts_read_reg(client, 0x02, &uc_temp);
			if (0x02 == uc_temp ||
				0xFF == uc_temp)
			{
				/* if 0x02, then auto clb ok, else 0xff, auto clb failure */
			    	break;
			}
			msleep(20);	    
		}
	} 
	else 
	{
		for(i=0;i<100;i++)
		{
			fts_read_reg(client, 0, &uc_temp);
			if (0x0 == ((uc_temp&0x70)>>4)) 
			{
				/* return to normal mode, calibration finish, auto/test mode auto switch */
			    	break;
			}
			msleep(20);	    
		}
	}
	/* calibration OK */
	fts_write_reg(client, 0, 0x40);  					/* goto factory mode for store */
	msleep(200);   									/* make sure already enter factory mode*/
	fts_write_reg(client, 2, 0x5); 	 					/* store CLB result, write 0x05 to 0x04 */
	msleep(300);
	fts_write_reg(client, 0, FTS_WORKMODE_VALUE);		/* return to normal mode */
	msleep(300);

	/* store CLB result OK */
	return 0;
}

#if 0
/************************************************************************
* Name: delay_qt_ms
* Brief:  delay ms time
* Input: ms time 
* Output: no
* Return: 0
***********************************************************************/
static void delay_qt_ms(unsigned long  w_ms)
{
	unsigned long i;
	unsigned long j;

	for (i = 0; i < w_ms; i++)
	{
		for (j = 0; j < 1000; j++)
		{
			 udelay(1);
		}
	}
}
#endif

/************************************************************************
*   Name: fts_6x36_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_6x36_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u32 fw_length;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;

	if(pbt_buf[0] != 0x02)
	{
		FTS_COMMON_DBG("[FTS] FW first byte is not 0x02. so it is invalid \n");
		return -1;
	}

	if(dw_lenth > 0x11f)
	{
		fw_length = ((u32)pbt_buf[0x100]<<8) + pbt_buf[0x101];
		if(dw_lenth < fw_length)
		{
			FTS_COMMON_DBG("[FTS] Fw length is invalid \n");
			return -1;
		}
	}
	else
	{
		FTS_COMMON_DBG("[FTS] Fw length is invalid \n");
		return -1;
	}
	
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) 
	{
		/********* Step 1:Reset  CTPM *****/
		/* write 0xaa to register FTS_RST_CMD_REG2 */
		fts_write_reg(client, FTS_RST_CMD_REG2, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);
		/* write 0x55 to register FTS_RST_CMD_REG2 */
		fts_write_reg(client, FTS_RST_CMD_REG2, FTS_UPGRADE_55);
		msleep(fts_updateinfo_curr.delay_55);
		/********* Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		fts_i2c_write(client, auc_i2c_write_buf, 1);
		auc_i2c_write_buf[0] = FTS_UPGRADE_AA;
		fts_i2c_write(client, auc_i2c_write_buf, 1);
		msleep(fts_updateinfo_curr.delay_readid);
		/********* Step 3:check READ-ID ***********************/		
		auc_i2c_write_buf[0] = FTS_READ_ID_REG;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =0x00;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);


		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
			&& reg_val[1] == fts_updateinfo_curr.upgrade_id_2) 
		{
			FTS_COMMON_DBG("[FTS] Step 3: GET CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			break;
		} 
		else 
		{
			FTS_COMMON_DBG( "[FTS] Step 3: GET CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;

	auc_i2c_write_buf[0] = FTS_READ_ID_REG;
	auc_i2c_write_buf[1] = 0x00;
	auc_i2c_write_buf[2] = 0x00;
	auc_i2c_write_buf[3] = 0x00;
	auc_i2c_write_buf[4] = 0x00;
	fts_i2c_write(client, auc_i2c_write_buf, 5);
	
	//auc_i2c_write_buf[0] = 0xcd;
	//fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);


	/* Step 4:erase app and panel paramenter area */
	FTS_COMMON_DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
	fts_i2c_write(client, auc_i2c_write_buf, 1);	/* erase app area */
	msleep(fts_updateinfo_curr.delay_erase_flash);

	for(i = 0;i < 200;i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		auc_i2c_write_buf[1] = 0x00;
		auc_i2c_write_buf[2] = 0x00;
		auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = 0x00;
		reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if(0xb0 == reg_val[0] && 0x02 == reg_val[1])
		{
			FTS_COMMON_DBG("[FTS] erase app finished \n");
			break;
		}
		msleep(50);
	}

	/********* Step 5:write firmware(FW) to ctpm flash *********/
	bt_ecc = 0;
	FTS_COMMON_DBG("Step 5:write firmware(FW) to ctpm flash\n");

	dw_lenth = fw_length;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = FTS_FW_WRITE_CMD;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) 
	{
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;

		for (i = 0; i < FTS_PACKET_LENGTH; i++) 
		{
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		
		fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		
		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			auc_i2c_write_buf[1] = 0x00;
			auc_i2c_write_buf[2] = 0x00;
			auc_i2c_write_buf[3] = 0x00;
			reg_val[0] = 0x00;
			reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
			if(0xb0 == (reg_val[0] & 0xf0) && (0x03 + (j % 0x0ffd)) == (((reg_val[0] & 0x0f) << 8) |reg_val[1]))
			{
				FTS_COMMON_DBG("[FTS] write a block data finished \n");
				break;
			}
			msleep(1);
		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) 
	{
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) 
		{
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		fts_i2c_write(client, packet_buf, temp + 6);

		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			auc_i2c_write_buf[1] = 0x00;
			auc_i2c_write_buf[2] = 0x00;
			auc_i2c_write_buf[3] = 0x00;
			reg_val[0] = 0x00;
			reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
			if(0xb0 == (reg_val[0] & 0xf0) && (0x03 + (j % 0x0ffd)) == (((reg_val[0] & 0x0f) << 8) |reg_val[1]))
			{
				FTS_COMMON_DBG("[FTS] write a block data finished \n");
				break;
			}
			msleep(1);
		}
	}


	/********* Step 6: read out checksum ***********************/
	/* send the opration head */
	FTS_COMMON_DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = FTS_REG_ECC;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) 
	{
		FTS_COMMON_DBG( "[FTS]--ecc error! fw_ecc=%02x flash_ecc=%02x\n",
					reg_val[0],
					bt_ecc);
		return -EIO;
	}

	/********* Step 7: reset the new FW ***********************/
	FTS_COMMON_DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);	/* make sure CTP startup normally */

	return 0;
}
/************************************************************************
*   Name: fts_6x06_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_6x06_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;

	
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) 
	{
		/********* Step 1:Reset  CTPM *****/
		/* write 0xaa to register FTS_RST_CMD_REG2 */
		
		fts_write_reg(client, FTS_RST_CMD_REG2, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);

		/* write 0x55 to register FTS_RST_CMD_REG2 */
		fts_write_reg(client, FTS_RST_CMD_REG2, FTS_UPGRADE_55);

		msleep(fts_updateinfo_curr.delay_55);

		/********* Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		do 
		{
			i++;
			i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
			msleep(5);
		} while (i_ret <= 0 && i < 5);


		/********* Step 3:check READ-ID ***********************/
		msleep(fts_updateinfo_curr.delay_readid);
		auc_i2c_write_buf[0] = FTS_READ_ID_REG;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);


		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
			&& reg_val[1] == fts_updateinfo_curr.upgrade_id_2) 
		{
			FTS_COMMON_DBG("[FTS] Step 3: CTPM ID OK ,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
			break;
		} 
		else 
		{
			FTS_COMMON_DBG( "[FTS] Step 3: CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;
	auc_i2c_write_buf[0] = 0xcd;

	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);


	/* Step 4:erase app and panel paramenter area */
	FTS_COMMON_DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
	fts_i2c_write(client, auc_i2c_write_buf, 1);	/* erase app area */
	msleep(fts_updateinfo_curr.delay_erase_flash);
	/* erase panel parameter area */
	auc_i2c_write_buf[0] = FTS_ERASE_PARAMS_CMD;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(100);

	/********* Step 5:write firmware(FW) to ctpm flash *********/
	bt_ecc = 0;
	FTS_COMMON_DBG("Step 5:write firmware(FW) to ctpm flash\n");

	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = FTS_FW_WRITE_CMD;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) 
	{
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;

		for (i = 0; i < FTS_PACKET_LENGTH; i++) 
	{
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		
		fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		msleep(FTS_PACKET_LENGTH / 6 + 1);
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) 
	{
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) 
	{
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		fts_i2c_write(client, packet_buf, temp + 6);
		msleep(20);
	}


	/* send the last six byte */
	for (i = 0; i < 6; i++) 
	{
		temp = 0x6ffa + i;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = 1;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;
		packet_buf[6] = pbt_buf[dw_lenth + i];
		bt_ecc ^= packet_buf[6];
		fts_i2c_write(client, packet_buf, 7);
		msleep(20);
	}


	/********* Step 6: read out checksum ***********************/
	/* send the opration head */
	FTS_COMMON_DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = FTS_REG_ECC;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) 
	{
		FTS_COMMON_DBG( "[FTS]--ecc error! fw_ecc=%02x flash_ecc=%02x\n",reg_val[0],bt_ecc);
		return -EIO;
	}

	/********* Step 7: reset the new FW ***********************/
	FTS_COMMON_DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = FTS_REG_RESET_FW;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);	/* make sure CTP startup normally */

	return 0;
}
/************************************************************************
* Name: fts_5x26_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_5x26_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
	u8 reg_val[4] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	//u8 bt_ecc_check;
	int i_ret=0;
	//int x=0;

	i_ret = hidi2c_to_stdi2c(client);
	if (i_ret == 0) 
	{
		FTS_COMMON_DBG("HidI2c change to StdI2c fail ! \n");
	}


	for (i = 0; i < FTS_UPGRADE_LOOP; i++) 
	{

		/********* Step 1:Reset  CTPM *****/
		fts_write_reg(client, 0xfc, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);
		fts_write_reg(client, 0xfc, FTS_UPGRADE_55);
		msleep(fts_updateinfo_curr.delay_55);

		/********* Step 2:Enter upgrade mode and switch protocol *****/
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
		/* write 0xAA to 0x55 */
		if (i_ret < 0) 
		{
			FTS_COMMON_DBG("failed writing  0x55 and 0xaa ! \n");
			continue;
		}
		
		i_ret = hidi2c_to_stdi2c(client);
		if(i_ret < 0)
		{
			FTS_COMMON_DBG("failed to Switch HidtoI2c Protocol ! \n");
			continue;
		}

		/********* Step 3:check READ-ID ***********************/
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1 && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)
		{	
			/* relate on bootloader FW */
			FTS_COMMON_DBG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
			break;
		} 
		else 
		{
			FTS_COMMON_DBG( "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
			continue;
		}
	}

	if (i >= FTS_UPGRADE_LOOP) return -EIO;
	/* Step 4:erase app and panel paramenter area */
	FTS_COMMON_DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	fts_i2c_write(client, auc_i2c_write_buf, 1);		
	/* erase app area */	
	auc_i2c_write_buf[0] = 0x63;
	fts_i2c_write(client, auc_i2c_write_buf, 1);		
	/* erase panel paramenter area */	
	auc_i2c_write_buf[0] = 0x04;
    	fts_i2c_write(client, auc_i2c_write_buf, 1);     
	/* erase panel paramenter area */
	 msleep(fts_updateinfo_curr.delay_erase_flash);
	/********* Step 5:write firmware(FW) to ctpm flash *********/
	bt_ecc = 0;
	FTS_COMMON_DBG("Step 5:write firmware(FW) to ctpm flash\n");
	temp = 0;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	//FTS_COMMON_DBG("FT5x26 Upgrade Func - dw_length:%d FTS_PACKET_LENGTH:%d\n,",dw_lenth,FTS_PACKET_LENGTH);
	//FTS_COMMON_DBG("FT5x26 Upgrade Func - Packet Number:%d\n",packet_number);
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++)
	{
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;
		
		for (i = 0; i < FTS_PACKET_LENGTH; i++)
		{
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		//FTS_COMMON_DBG("[FTS][%s] bt_ecc = %x \n", __func__, bt_ecc);
		fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		msleep(FTS_PACKET_LENGTH / 6 + 1);	
	}
	
	if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
	{
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;
		
		for (i = 0; i < temp; i++) 
		{
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, temp+6);
		msleep(20);
	}
	/********* Step 6: read out checksum ***********************/
	/* send the opration head */
	FTS_COMMON_DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0xcc;
	reg_val[0] = reg_val[1] = 0x00;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	FTS_COMMON_DBG(KERN_WARNING "Checksum FT5X26:%X %X \n", reg_val[0], bt_ecc);
	if (reg_val[0] != bt_ecc) 
	{
		FTS_COMMON_DBG( "[FTS]--ecc error! fw_ecc=%02x flash_ecc=%02x\n",reg_val[0],bt_ecc);
		return -EIO;
	}

	/********* Step 7: reset the new FW ***********************/
	FTS_COMMON_DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);

	/******** Step 8 Disable Write Flash *****/
	FTS_COMMON_DBG("Step 8: Disable Write Flash\n");
    	auc_i2c_write_buf[0] = 0x04;
    	fts_i2c_write(client, auc_i2c_write_buf, 1);

	msleep(300);	
	/* make sure CTP startup normally */
	auc_i2c_write_buf[0] =auc_i2c_write_buf[1]= 0x00;
	fts_i2c_write(client,auc_i2c_write_buf,2);

	return 0;
}

/************************************************************************
*   Name: fts_5x36_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_5x36_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u8 is_5336_new_bootloader = 0;
	u8 is_5336_fwsize_30 = 0;
	u32  packet_number;
	u32  j;
	u32  temp;
	u32  lenght;
	u8 	packet_buf[FTS_PACKET_LENGTH + 6];
	u8  	auc_i2c_write_buf[10];
	u8  	bt_ecc;
	int	i_ret;
	int	fw_filenth = sizeof(CTPM_FW);

	if(CTPM_FW[fw_filenth-12] == 30)
	{
		is_5336_fwsize_30 = 1;
	}
	else 
	{
		is_5336_fwsize_30 = 0;
	}

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) 
{
    		/********* Step 1:Reset  CTPM *****/
    		/* write 0xaa to register FTS_RST_CMD_REG1 */
	   	fts_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);
		
		 /* write 0x55 to register FTS_RST_CMD_REG1 */
		fts_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_55);   
		msleep(fts_updateinfo_curr.delay_55);   


		/********* Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		
	    i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
	  
	    /********* Step 3:check READ-ID ***********************/   
		msleep(fts_updateinfo_curr.delay_readid);
	   	auc_i2c_write_buf[0] = FTS_READ_ID_REG; 
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
              fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);           
              if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1 
                              && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)
              {
                      FTS_COMMON_DBG( "[FTS] Step 3: CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]); 
                      break; 
              }
              else
              {
                      FTS_COMMON_DBG( "[FTS] Step 3: CTPM ID FAILD,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
                      continue;
              }

          }

          if (i >= FTS_UPGRADE_LOOP)
                          return -EIO;
		  
         auc_i2c_write_buf[0] = 0xcd;
         fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);	
	if (reg_val[0] <= 4)
	{
		is_5336_new_bootloader = BL_VERSION_LZ4 ;
	}
	else if(reg_val[0] == 7)
	{
		is_5336_new_bootloader = BL_VERSION_Z7 ;
	}
	else if(reg_val[0] >= 0x0f)
	{
		is_5336_new_bootloader = BL_VERSION_GZF ;
	}

     /********* Step 4:erase app and panel paramenter area ********************/
	if(is_5336_fwsize_30)
	{
		auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
		fts_i2c_write(client, auc_i2c_write_buf, 1); 	/* erase app area */	
   		 msleep(fts_updateinfo_curr.delay_erase_flash); 

		 auc_i2c_write_buf[0] = FTS_ERASE_PARAMS_CMD;
		fts_i2c_write(client, auc_i2c_write_buf, 1); 	/* erase config area */	
   		 msleep(50);
	}
	else
	{
		auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
		fts_i2c_write(client, auc_i2c_write_buf, 1); 	/* erase app area */	
   		msleep(fts_updateinfo_curr.delay_erase_flash); 
	}

	/********* Step 5:write firmware(FW) to ctpm flash *********/
	bt_ecc = 0;

	if(is_5336_new_bootloader == BL_VERSION_LZ4 || is_5336_new_bootloader == BL_VERSION_Z7 )
	{
		dw_lenth = dw_lenth - 8;
	}
	else if(is_5336_new_bootloader == BL_VERSION_GZF) 
	{
		dw_lenth = dw_lenth - 14;
	}
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = FTS_FW_WRITE_CMD;
	packet_buf[1] = 0x00;
	for (j=0;j<packet_number;j++)
	{
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8)(temp>>8);
		packet_buf[3] = (u8)temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8)(lenght>>8);
		packet_buf[5] = (u8)lenght;

		for (i=0;i<FTS_PACKET_LENGTH;i++)
		{
		    packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
		    bt_ecc ^= packet_buf[6+i];
		}

		fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH+6);
		msleep(FTS_PACKET_LENGTH/6 + 1);
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
	{
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8)(temp>>8);
		packet_buf[3] = (u8)temp;

		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8)(temp>>8);
		packet_buf[5] = (u8)temp;

		for (i=0;i<temp;i++)
		{
		    packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
		    bt_ecc ^= packet_buf[6+i];
		}
  
		fts_i2c_write(client, packet_buf, temp+6);
		msleep(20);
	}
        /* send the last six byte */
        if(is_5336_new_bootloader == BL_VERSION_LZ4 || is_5336_new_bootloader == BL_VERSION_Z7 )
        {
                for (i = 0; i<6; i++)
                {
                                if (is_5336_new_bootloader  == BL_VERSION_Z7 /*&& DEVICE_IC_TYPE==IC_FT5x36*/) 
                                {
                                                temp = 0x7bfa + i;
                                }
                                else if(is_5336_new_bootloader == BL_VERSION_LZ4)
                                {
                                                temp = 0x6ffa + i;
                                }
                                packet_buf[2] = (u8)(temp>>8);
                                packet_buf[3] = (u8)temp;
                                temp =1;
                                packet_buf[4] = (u8)(temp>>8);
                                packet_buf[5] = (u8)temp;
                                packet_buf[6] = pbt_buf[ dw_lenth + i]; 
                                bt_ecc ^= packet_buf[6];
                                fts_i2c_write(client, packet_buf, 7);
                                msleep(10);
                }
        }
        else if(is_5336_new_bootloader == BL_VERSION_GZF)
        {
		for (i = 0; i<12; i++)
		{
		        if (is_5336_fwsize_30 /*&& DEVICE_IC_TYPE==IC_FT5x36*/) 
		        {
		                        temp = 0x7ff4 + i;
		        }
		        else if (1/*DEVICE_IC_TYPE==IC_FT5x36*/) 
		        {
		                        temp = 0x7bf4 + i;
		        }
		        packet_buf[2] = (u8)(temp>>8);
		        packet_buf[3] = (u8)temp;
		        temp =1;
		        packet_buf[4] = (u8)(temp>>8);
		        packet_buf[5] = (u8)temp;
		        packet_buf[6] = pbt_buf[ dw_lenth + i]; 
		        bt_ecc ^= packet_buf[6];
			fts_i2c_write(client, packet_buf, 7);
			msleep(10);
		}
	}

	/********* Step 6: read out checksum ***********************/
	/* send the opration head */
	auc_i2c_write_buf[0] = FTS_REG_ECC;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1); 
	if(reg_val[0] != bt_ecc)
	{
		FTS_COMMON_DBG( "[FTS]--ecc error! fw_ecc=%02x flash_ecc=%02x\n", reg_val[0], bt_ecc);
	    	return -EIO;
	}
	/********* Step 7: reset the new FW ***********************/
	auc_i2c_write_buf[0] = FTS_REG_RESET_FW;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);  /*make sure CTP startup normally*/

	return 0;
}
/************************************************************************
* Name: fts_5822_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_5822_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
	u8 reg_val[4] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	u8 bt_ecc_check;
	int i_ret;

	i_ret = hidi2c_to_stdi2c(client);
	if (i_ret == 0) 
	{
		FTS_COMMON_DBG("HidI2c change to StdI2c fail ! \n");
	}
	
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) 
	{
		/********* Step 1:Reset  CTPM *****/
		fts_write_reg(client, 0xfc, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);
		fts_write_reg(client, 0xfc, FTS_UPGRADE_55);
		msleep(200);
		/********* Step 2:Enter upgrade mode *****/
		i_ret = hidi2c_to_stdi2c(client);
		if (i_ret == 0) 
		{
			FTS_COMMON_DBG("HidI2c change to StdI2c fail ! \n");
			continue;
		}
		msleep(5);
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);	
		if (i_ret < 0) 
		{
			FTS_COMMON_DBG("failed writing  0x55 and 0xaa ! \n");
			continue;
		}
		/*********Step 3:check READ-ID***********************/
		msleep(1);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);
		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1 && reg_val[1] == fts_updateinfo_curr.upgrade_id_2) 
		{	
			/* relate on bootloader FW*/
			FTS_COMMON_DBG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
			break;
		} 
		else 
		{
			FTS_COMMON_DBG( "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n", reg_val[0], reg_val[1]);
			continue;
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;
	/* Step 4:erase app and panel paramenter area */
	FTS_COMMON_DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	fts_i2c_write(client, auc_i2c_write_buf, 1);	/*erase app area, trigger erase command */
	msleep(1350);
	for (i = 0; i < 15; i++) 
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
		if (0xF0 == reg_val[0] && 0xAA == reg_val[1]) 
		{
			break;
		}
		msleep(50);
	}
	FTS_COMMON_DBG("[FTS][%s] erase app area reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
	/* write bin file length to FW bootloader. */
	auc_i2c_write_buf[0] = 0xB0;
	auc_i2c_write_buf[1] = (u8) ((dw_lenth >> 16) & 0xFF);
	auc_i2c_write_buf[2] = (u8) ((dw_lenth >> 8) & 0xFF);
	auc_i2c_write_buf[3] = (u8) (dw_lenth & 0xFF);
	fts_i2c_write(client, auc_i2c_write_buf, 4);
	/********* Step 5:write firmware(FW) to ctpm flash *********/
	bt_ecc = 0;
	bt_ecc_check = 0;
	FTS_COMMON_DBG("Step 5:write firmware(FW) to ctpm flash\n");
	/* dw_lenth = dw_lenth - 8; */
	temp = 0;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	for (j = 0; j < packet_number; j++) 
	{
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;
		for (i = 0; i < FTS_PACKET_LENGTH; i++) 
		{
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc_check ^= pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		FTS_COMMON_DBG("[FTS][%s] bt_ecc = %x \n", __func__, bt_ecc);
		if (bt_ecc != bt_ecc_check)
			FTS_COMMON_DBG("[FTS][%s] Host checksum error bt_ecc_check = %x \n", __func__, bt_ecc_check);
		fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		/*msleep(10);*/
		for (i = 0; i < 30; i++) 
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1])) {
				break;
			}
			FTS_COMMON_DBG("[FTS][%s] reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
			msleep(1);
		}
	}
	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) 
	{
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;
		for (i = 0; i < temp; i++) 
		{
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc_check ^= pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, temp + 6);
		FTS_COMMON_DBG("[FTS][%s] bt_ecc = %x \n", __func__, bt_ecc);
		if (bt_ecc != bt_ecc_check)
			FTS_COMMON_DBG("[FTS][%s] Host checksum error bt_ecc_check = %x \n", __func__, bt_ecc_check);
		for (i = 0; i < 30; i++) 
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
			FTS_COMMON_DBG("[FTS][%s] reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1])) 
			{
				break;
			}
			FTS_COMMON_DBG("[FTS][%s] reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
			msleep(1);
		}
	}
	msleep(50);
	/********* Step 6: read out checksum ***********************/
	/* send the opration head */
	FTS_COMMON_DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0x64;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);
	temp = 0;
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = (u8)(temp >> 16);
	auc_i2c_write_buf[2] = (u8)(temp >> 8);
	auc_i2c_write_buf[3] = (u8)(temp);
	temp = dw_lenth;
	auc_i2c_write_buf[4] = (u8)(temp >> 8);
	auc_i2c_write_buf[5] = (u8)(temp);
	i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6);
	msleep(dw_lenth/256);
	for (i = 0; i < 100; i++) 
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
		FTS_COMMON_DBG( "[FTS]--reg_val[0]=%02x reg_val[0]=%02x\n", reg_val[0], reg_val[1]);
		if (0xF0 == reg_val[0] && 0x55 == reg_val[1]) 
		{
			FTS_COMMON_DBG( "[FTS]--reg_val[0]=%02x reg_val[0]=%02x\n", reg_val[0], reg_val[1]);
			break;
		}
		msleep(1);
	}
	auc_i2c_write_buf[0] = 0x66;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) 
	{
		FTS_COMMON_DBG( "[FTS]--ecc error! fw_ecc=%02x flash_ecc=%02x\n", reg_val[0], bt_ecc);
		return -EIO;
	}
	FTS_COMMON_DBG(KERN_WARNING "checksum fw_ecc=%X flash_ecc=%X \n", reg_val[0], bt_ecc);
	/********* Step 7: reset the new FW ***********************/
	FTS_COMMON_DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(200);						/* make sure CTP startup normally */
	i_ret = hidi2c_to_stdi2c(client);	/* Android to Std i2c. */
	if (i_ret == 0) 
	{
		FTS_COMMON_DBG("HidI2c change to StdI2c fail ! \n");
	}
	return 0;
}

/************************************************************************
*   Name: fts_5x06_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int fts_5x06_ctpm_fw_upgrade(struct i2c_client *client, u8 *pbt_buf, u32 dw_lenth)
{
	u8 reg_val[2] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;
	
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) 
	{
		/********* Step 1:Reset  CTPM *****/
		/* write 0xaa to register FTS_RST_CMD_REG1 */
		fts_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);

		/* write 0x55 to register FTS_RST_CMD_REG1 */
		fts_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_55);
		msleep(fts_updateinfo_curr.delay_55);
		/********* Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		do 
		{
			i++;
			i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
			msleep(5);
		} while (i_ret <= 0 && i < 5);


		/********* Step 3:check READ-ID ***********************/
		msleep(fts_updateinfo_curr.delay_readid);
		auc_i2c_write_buf[0] = FTS_READ_ID_REG;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
			&& reg_val[1] == fts_updateinfo_curr.upgrade_id_2) 
		{
			FTS_COMMON_DBG("[FTS] Step 3: CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
			break;
		} 
		else 
		{
			FTS_COMMON_DBG( "[FTS] Step 3: CTPM ID FAIL,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
		return -EIO;
	/* Step 4:erase app and panel paramenter area */
	FTS_COMMON_DBG("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
	fts_i2c_write(client, auc_i2c_write_buf, 1);	/* erase app area */
	msleep(fts_updateinfo_curr.delay_erase_flash);
	/* erase panel parameter area */
	auc_i2c_write_buf[0] = FTS_ERASE_PARAMS_CMD;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(100);

	/********* Step 5:write firmware(FW) to ctpm flash *********/
	bt_ecc = 0;
	FTS_COMMON_DBG("Step 5:write firmware(FW) to ctpm flash\n");
	dw_lenth = dw_lenth - 8;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = FTS_FW_WRITE_CMD;
	packet_buf[1] = 0x00;
	for (j = 0; j < packet_number; j++) 
	{
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;
		for (i = 0; i < FTS_PACKET_LENGTH; i++) 
		{
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		msleep(FTS_PACKET_LENGTH / 6 + 1);
	}
	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) 
	{
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;
		for (i = 0; i < temp; i++) 
		{
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}

		fts_i2c_write(client, packet_buf, temp + 6);
		msleep(20);
	}
	/* send the last six byte */
	for (i = 0; i < 6; i++) 
	{
		temp = 0x6ffa + i;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = 1;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;
		packet_buf[6] = pbt_buf[dw_lenth + i];
		bt_ecc ^= packet_buf[6];
		fts_i2c_write(client, packet_buf, 7);
		msleep(20);
	}
	/********* Step 6: read out checksum ***********************/
	/*send the opration head */
	FTS_COMMON_DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = FTS_REG_ECC;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) 
	{
		FTS_COMMON_DBG( "[FTS]--ecc error! fw_ecc=%02x flash_ecc=%02x\n",
					reg_val[0],
					bt_ecc);
		return -EIO;
	}		
	/********* Step 7: reset the new FW ***********************/
	FTS_COMMON_DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = FTS_REG_RESET_FW;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(300);	/* make sure CTP startup normally */
	return 0;
}

/************************************************************************
*   Name: fts_5x46_ctpm_fw_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
int  fts_5x46_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{	
	u8 reg_val[4] = {0};
//	u8 reg_val_i2c[2] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j,k=0;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;
	for (k= 0; k< 1/*FTS_UPGRADE_LOOP*/; k++) 
	{
		i_ret = hidi2c_to_stdi2c(client);
		if(i_ret == 0)
		{
			FTS_COMMON_DBG("[FTS] hid change to i2c fail ! \n");
		}

		for (i = 0; i < FTS_UPGRADE_LOOP; i++) 
		{
			/********* Step 1:Reset  CTPM *****/
			/* write 0xaa to register FTS_RST_CMD_REG1 */
			fts_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_AA);
			msleep(fts_updateinfo_curr.delay_aa);

			// write 0x55 to register FTS_RST_CMD_REG1 
			fts_write_reg(client, FTS_RST_CMD_REG1, FTS_UPGRADE_55);
			msleep(200);
			/********* Step 2:Enter upgrade mode *****/
			i_ret = hidi2c_to_stdi2c(client);

			if(i_ret == 0)
			{
				FTS_COMMON_DBG("[FTS] hid change to i2c fail ! \n");
				continue;
			}
			msleep(10);
			auc_i2c_write_buf[0] = FTS_UPGRADE_55;
			auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
			i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);	
			if(i_ret < 0)
			{
				FTS_COMMON_DBG("[FTS] failed writing  0x55 and 0xaa ! \n");
				continue;
			}
			/********* Step 3:check READ-ID ***********************/
			msleep(1);
			auc_i2c_write_buf[0] = FTS_READ_ID_REG;
			auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =0x00;		
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

			if (reg_val[0] == fts_updateinfo_curr.upgrade_id_1
				&& reg_val[1] == fts_updateinfo_curr.upgrade_id_2) 
			{
				/* relate on bootloader FW */
				FTS_COMMON_DBG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
				break;
			} 
			else 
			{
				FTS_COMMON_DBG( "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
				
				continue;
			}
		}
		if (i >= FTS_UPGRADE_LOOP )
			return -EIO;
		/* Step 4:erase app and panel paramenter area */
		FTS_COMMON_DBG("Step 4:erase app and panel paramenter area\n");
		auc_i2c_write_buf[0] = FTS_ERASE_APP_REG;
		fts_i2c_write(client, auc_i2c_write_buf, 1);	//erase app area 
		msleep(1350);
		for(i = 0;i < 15;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
			if(0xF0==reg_val[0] && 0xAA==reg_val[1])
			{
				break;
			}
			msleep(50);		
		}
		FTS_COMMON_DBG("[FTS][%s] erase app area reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
		/* write bin file length to FW bootloader. */
		auc_i2c_write_buf[0] = 0xB0;
		auc_i2c_write_buf[1] = (u8) ((dw_lenth >> 16) & 0xFF);
		auc_i2c_write_buf[2] = (u8) ((dw_lenth >> 8) & 0xFF);
		auc_i2c_write_buf[3] = (u8) (dw_lenth & 0xFF);
		fts_i2c_write(client, auc_i2c_write_buf, 4);
		/********* Step 5:write firmware(FW) to ctpm flash *********/
		bt_ecc = 0;
		FTS_COMMON_DBG("Step 5:write firmware(FW) to ctpm flash\n");
		/*dw_lenth = dw_lenth - 8;*/
		temp = 0;
		packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
		packet_buf[0] = FTS_FW_WRITE_CMD;
		packet_buf[1] = 0x00;

		for (j = 0; j < packet_number; j++) 
		{
			temp = j * FTS_PACKET_LENGTH;
			packet_buf[2] = (u8) (temp >> 8);
			packet_buf[3] = (u8) temp;
			lenght = FTS_PACKET_LENGTH;
			packet_buf[4] = (u8) (lenght >> 8);
			packet_buf[5] = (u8) lenght;
			for (i = 0; i < FTS_PACKET_LENGTH; i++) 
		{
				packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
				bt_ecc ^= packet_buf[6 + i];
			}
			fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
			/*msleep(10);*/
			//FTS_COMMON_DBG("enter");
			for(i = 0;i < 30;i++)
			{
				auc_i2c_write_buf[0] = 0x6a;
				reg_val[0] = reg_val[1] = 0x00;
				fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
				if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
				{
					//FTS_COMMON_DBG("\n [FTS][%s] zax i=%d,j=%d, val[0] = %x val[1] = %x \n", __func__, i,j,reg_val[0], reg_val[1]);
					break;
				}
				FTS_COMMON_DBG("[FTS][%s] reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
				msleep(1);			
			}
			//FTS_COMMON_DBG("leave");
		}
		if ((dw_lenth) % FTS_PACKET_LENGTH > 0) 
		{
			temp = packet_number * FTS_PACKET_LENGTH;
			packet_buf[2] = (u8) (temp >> 8);
			packet_buf[3] = (u8) temp;
			temp = (dw_lenth) % FTS_PACKET_LENGTH;
			packet_buf[4] = (u8) (temp >> 8);
			packet_buf[5] = (u8) temp;
			for (i = 0; i < temp; i++) 
		{
				packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
				bt_ecc ^= packet_buf[6 + i];
			}	
			fts_i2c_write(client, packet_buf, temp + 6);
			for(i = 0;i < 30;i++)
			{
				auc_i2c_write_buf[0] = 0x6a;
				reg_val[0] = reg_val[1] = 0x00;
				fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
				FTS_COMMON_DBG("[FTS][%s] reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
				if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
				{
					break;
				}
				FTS_COMMON_DBG("[FTS][%s] reg_val[0] = %x reg_val[1] = %x \n", __func__, reg_val[0], reg_val[1]);
				mdelay(1);
				
			}
		}

		msleep(50);
		
		/********* Step 6: read out checksum ***********************/
		/* send the opration head */
		FTS_COMMON_DBG("Step 6: read out checksum\n");
		//fts_i2c_read(client, 0xcc, 1, reg_val_i2c, 1);
		auc_i2c_write_buf[0] = 0x64;
		fts_i2c_write(client, auc_i2c_write_buf, 1); 
		msleep(300);

		temp = 0;
		auc_i2c_write_buf[0] = 0x65;
		auc_i2c_write_buf[1] = (u8)(temp >> 16);
		auc_i2c_write_buf[2] = (u8)(temp >> 8);
		auc_i2c_write_buf[3] = (u8)(temp);
		temp = dw_lenth;
		auc_i2c_write_buf[4] = (u8)(temp >> 8);
		auc_i2c_write_buf[5] = (u8)(temp);
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6); 
		msleep(dw_lenth/256);

		for(i = 0;i < 100;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);
			FTS_COMMON_DBG( "[FTS]--reg_val[0]=%02x reg_val[0]=%02x\n", reg_val[0], reg_val[1]);
			if (0xF0==reg_val[0] && 0x55==reg_val[1])
			{
				FTS_COMMON_DBG( "[FTS]--reg_val[0]=%02x reg_val[0]=%02x\n", reg_val[0], reg_val[1]);
				break;
			}
			msleep(1);
				
		}
		auc_i2c_write_buf[0] = 0x66;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
		if (reg_val[0] != bt_ecc) 
		{
			FTS_COMMON_DBG( "[FTS]--k=%d, ecc error! fw_ecc=%02x flash_ecc=%02x\n",
						k,reg_val[0],
						bt_ecc);//,reg_val_i2c[0]);
					
			continue;//return -EIO;
		}
		else
		{
			
			FTS_COMMON_DBG(KERN_WARNING "k=%d,checksum fw_ecc=%X flash_ecc=%X i2c_ecc \n",k,reg_val[0],bt_ecc);//,reg_val_i2c[0]);	
			break;
		}
	}
	if (k > 1/*FTS_UPGRADE_LOOP*/ )
		return -EIO;
	/********* Step 7: reset the new FW ***********************/
	FTS_COMMON_DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = FTS_REG_RESET_FW;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(200);						// make sure CTP startup normally 
	i_ret = hidi2c_to_stdi2c(client);	/* Android to Std i2c. */
	if (i_ret == 0) 
	{
		FTS_COMMON_DBG("HidI2c change to StdI2c fail ! \n");
	}
	return 0;
}
bool Upgrade_ReadPram(struct i2c_client * client,unsigned int Addr, unsigned char * pData, unsigned short Datalen)
{
	bool ReCode;
	int ret=-1;
	unsigned char pDataSend[16];
	//if (iCommMode == HY_I2C_INTERFACE)
	{
		pDataSend[0] = 0x85;
		pDataSend[1] = 0x00;
		pDataSend[2] = Addr>>8;
		pDataSend[3] = Addr;
		//HY_IIC_IO(hDevice, pDataSend, 4, NULL, 0);
		fts_i2c_write(client, pDataSend, 4);
		//HY_IIC_IO(hDevice, NULL, 0, pData, Datalen) == ERROR_CODE_OK ? ReCode = true : ReCode = false;

		

		ret =fts_i2c_read(client, NULL, 0, pData, Datalen);  
		if (ret < 0) 
		{        
			FTS_COMMON_DBG("[FTS] failed Upgrade_ReadPram \n");     
			return ret;    
		} 
		
	}
	
	

	return ReCode;
}
int  fts_8606_ctpm_fw_write_pram(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{

	u8 reg_val[4] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp,nowAddress=0,StartFlashAddr=0,FlashAddr=0;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 *pCheckBuffer = NULL;
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret,ReCode=-1;

	//fts_get_upgrade_info(&upgradeinfo);
	FTS_COMMON_DBG("8606 dw_lenth= %d",dw_lenth);
	if(dw_lenth > 0x10000 || dw_lenth ==0)
	{		
		return -EIO;
	}
	pCheckBuffer=kmalloc(dw_lenth+1,GFP_ATOMIC);
	for (i = 0; i < 20; i++) 
	{
		/********* Step 1:Reset  CTPM *****/
		/* write 0xaa to register 0xfc */
		//ftxxxx_reset_tp(0);
		//msleep(10);
		//ftxxxx_reset_tp(1);
		
		//msleep(10);	//time (5~20ms)


		fts_write_reg(client, 0xfc, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);
		fts_write_reg(client, 0xfc, FTS_UPGRADE_55);
		msleep(200);
		/********* Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 1);
		if(i_ret < 0)
		{
			FTS_COMMON_DBG("[FTS] failed writing  0x55 ! \n");
			continue;
		}
		
		/*
		auc_i2c_write_buf[0] = FT_UPGRADE_AA;
		i_ret = ftxxxx_i2c_Write(client, auc_i2c_write_buf, 1);
		if(i_ret < 0)
		{
			DBG("[FTS] failed writing  0xaa ! \n");
			continue;
		}
		*/	
		/********* Step 3:check READ-ID ***********************/
		msleep(1);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;
		reg_val[0] = reg_val[1] = 0x00;
		
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if ((reg_val[0] == 0x86 && reg_val[1] == 0x06) 
			|| (reg_val[0] == 0x86 && reg_val[1] == 0x07)) 
		{			
			msleep(50);
			break;
		} 
		else 
		{
			FTS_COMMON_DBG( "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			
			continue;
		}
	}

	if (i >= FTS_UPGRADE_LOOP )
		return -EIO;

	/********* Step 4:write firmware(FW) to ctpm flash *********/
	bt_ecc = 0;
	FTS_COMMON_DBG("Step 5:write firmware(FW) to ctpm flash\n");

	//dw_lenth = dw_lenth - 8;
	
	temp = 0;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xae;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) 
	{
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;

		for (i = 0; i < FTS_PACKET_LENGTH; i++) 
		{
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		nowAddress=nowAddress+FTS_PACKET_LENGTH;
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) 
	{
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) 
		{
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}		
		fts_i2c_write(client, packet_buf, temp + 6);
		nowAddress=nowAddress+temp;
	}
	/*
	temp = FT_APP_INFO_ADDR;
	packet_buf[2] = (u8) (temp >> 8);
	packet_buf[3] = (u8) temp;
	temp = 8;
	packet_buf[4] = (u8) (temp >> 8);
	packet_buf[5] = (u8) temp;
	for (i = 0; i < 8; i++) 
	{
		packet_buf[6+i] = pbt_buf[dw_lenth + i];
		bt_ecc ^= packet_buf[6+i];
	}	
	ftxxxx_i2c_Write(client, packet_buf, 6+8);
	*/
	/********* Step 5: read out checksum ***********************/
	/* send the opration head */
	FTS_COMMON_DBG("Step 6: read out checksum\n");
	/*auc_i2c_write_buf[0] = 0xcc;
	//msleep(2);
	ftxxxx_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) 
	{
		FTS_COMMON_DBG( "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",reg_val[0],bt_ecc);	
		return -EIO;
	}
	FTS_COMMON_DBG(KERN_WARNING "checksum %X %X \n",reg_val[0],bt_ecc);
	DBG("Read flash and compare\n");
		*/
	msleep(100);
	//-----------------------------------------------------------------------------------------------------
	FTS_COMMON_DBG( "[FTS]--nowAddress=%02x dw_lenth=%02x\n",nowAddress,dw_lenth);	
	if(nowAddress == dw_lenth)
	{

	FlashAddr=0;
	while(1)
	{
		StartFlashAddr = FlashAddr;
		if(FlashAddr == dw_lenth)
		{
			break;
		}
		else if(FlashAddr+FTS_PACKET_LENGTH > dw_lenth)
		{			
			if(!Upgrade_ReadPram(client,StartFlashAddr, pCheckBuffer+FlashAddr, dw_lenth-FlashAddr))
			{
				FTS_COMMON_DBG("read out checksum error\n");
				return -EIO;
				//break;
			}
			ReCode = ERROR_CODE_OK;
			FlashAddr = dw_lenth;
		}
		else
		{
			if(!Upgrade_ReadPram(client,StartFlashAddr, pCheckBuffer+FlashAddr, FTS_PACKET_LENGTH))
			{
				FTS_COMMON_DBG("read out checksum error\n");
				return -EIO;
				
				//break;
			}
			FlashAddr += FTS_PACKET_LENGTH;
			ReCode = ERROR_CODE_OK;
		}

		if(ReCode != ERROR_CODE_OK){
			FTS_COMMON_DBG("read out checksum error\n");
				return -EIO;
			//break;
		}
		
	}
	FTS_COMMON_DBG( "[FTS]--FlashAddr=%02x dw_lenth=%02x\n",FlashAddr,dw_lenth);
	if(FlashAddr == dw_lenth)
	{
		
		FTS_COMMON_DBG("Checking data...\n");
		for(i=0; i<dw_lenth; i++)
		{
			if(pCheckBuffer[i] != pbt_buf[i])
			{
				FTS_COMMON_DBG("read out checksum error\n");
				if(pCheckBuffer)
					kfree(pCheckBuffer);

					pCheckBuffer = NULL;
				return -EIO;
			}
		}
		if(pCheckBuffer)
					kfree(pCheckBuffer);

					pCheckBuffer = NULL;
		//COMM_FLASH_FT5422_Upgrade_StartApp(bOldProtocol, iCommMode);		//Reset
		FTS_COMMON_DBG("read out checksum successful\n");
		
	}
	else
	{
		FTS_COMMON_DBG("read out checksum error\n");
	}	

	}
	else
	{
		FTS_COMMON_DBG("read out checksum error\n");
	}	

	/********* Step 6: start app ***********************/
	FTS_COMMON_DBG("Step 6: start app\n");
	auc_i2c_write_buf[0] = 0x08;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(20);

	return 0;
}
int  fts_8606_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{
	u8 reg_val[4] = {0};
	u8 reg_val_id[4] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;
	unsigned char cmd[20];
	unsigned char Checksum = 0;
	u32 uCheckStart = 0x1000;
	u32 uCheckOff = 0x20;
	u32 uStartAddr = 0x00;
	
	//memcpy(m_DataBuffer, pFileData, m_FileLen);
	//pbt_buf=pFileData;
	auc_i2c_write_buf[0] = 0x05;
	reg_val_id[0] = 0x00;
		
	i_ret =fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val_id, 1);
	if(dw_lenth == 0)
	{
		return -EIO;
	}
	if(0x81 == (int)reg_val_id[0]) 
	{
			if(dw_lenth > 1024*60) 
			{
				return -EIO;
			}
	}
	else if(0x80 == (int)reg_val_id[0]) 
	{
			if(dw_lenth > 1024*64) 
			{
				return -EIO;
			}
	}


	/*if(dw_lenth > 1024*64)
	{
		return -EIO;
	}*/

	//fts_get_upgrade_info(&upgradeinfo);

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/********* Step 1:Reset  CTPM *****/
		//ftxxxx_write_reg(client, 0xfc, FT_UPGRADE_AA);
		//msleep(upgradeinfo.delay_aa);
		//ftxxxx_write_reg(client, 0xfc, FT_UPGRADE_55);
		//msleep(200);
		/********* Step 2:Enter upgrade mode *****/
		//i_ret = hidi2c_to_stdi2c(client);

		//if(i_ret == 0)
		//{
		//	DBG("HidI2c change to StdI2c fail ! \n");
		//	continue;
		//}
		msleep(10);
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
		if(i_ret < 0)
		{
			FTS_COMMON_DBG("failed writing  0x55 and 0xaa ! \n");
			continue;
		}

		/********* Step 3:check READ-ID ***********************/
		msleep(1);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;

		reg_val[0] = reg_val[1] = 0x00;

		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if ((reg_val[0] == fts_updateinfo_curr.upgrade_id_1 && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)) {
				FTS_COMMON_DBG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
					reg_val[0], reg_val[1]);
				break;
		} 
		else if ((reg_val[0] == 0x86 && reg_val[1] == 0xA7)) {
				FTS_COMMON_DBG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
					reg_val[0], reg_val[1]);

				//uCheckStart = 0x1020;
				break;
		}
		else {
			FTS_COMMON_DBG( "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);

			continue;
		}
	}	
	if (i >= FTS_UPGRADE_LOOP )
		return -EIO;	
	/* Step 4:erase app and panel paramenter area */
	FTS_COMMON_DBG("Step 4:erase app and panel paramenter area\n");
		
	{		
		cmd[0] = 0x05;
		cmd[1] = reg_val_id[0];//0x80;
		cmd[2] = 0x00;
		//ReCode = HY_IIC_IO(hDevice, cmd, 2, NULL, 0);	
		fts_i2c_write(client, cmd, 3);
	}
	
	// Set pramboot download mode
	//COMM_FLASH_FT5422_Upgrade_SetFlashMode(0x0B, iCommMode);
	{
		cmd[0] = 0x09;
		cmd[1] = 0x0B;
		//HY_IIC_IO(hDevice, cmd, 2, NULL, 0);
		fts_i2c_write(client, cmd, 2);
	}	
	for(i=0; i<dw_lenth ; i++)
	{		
		Checksum ^= pbt_buf[i];
	}
	msleep(50);

	// erase app area 
	
	auc_i2c_write_buf[0] = 0x61;
	fts_i2c_write(client, auc_i2c_write_buf, 1);     
	msleep(1350);

	for(i = 0;i < 15;i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

		if(0xF0==reg_val[0] && 0xAA==reg_val[1])
		{
			break;
		}
		msleep(50);
	}
	
	/********* Step 5:write firmware(FW) to ctpm flash *********/
	bt_ecc = 0;
	FTS_COMMON_DBG("Step 5:write firmware(FW) to ctpm flash\n");
	
	//dw_lenth = dw_lenth - 8;
	temp = 0;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	//packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) {
		temp = uCheckStart+j * FTS_PACKET_LENGTH;
		packet_buf[1] = (u8) (temp >> 16);
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		uStartAddr=temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;
		
		uCheckOff = uStartAddr/lenght;

		for (i = 0; i < FTS_PACKET_LENGTH; i++) 
		{
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		//msleep(10);

		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((uCheckOff +uCheckStart) == (((reg_val[0]) << 8) | reg_val[1]))
			{
				break;
			}
			msleep(1);
		}

		temp = (((reg_val[0]) << 8) | reg_val[1]);
		if( i == 30) FTS_COMMON_DBG("Query 6a reg time out value: 0x%x, driver set value:0x%x\n",  temp, uCheckOff + uCheckStart);
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = uCheckStart+packet_number * FTS_PACKET_LENGTH;
		packet_buf[1] = (u8) (temp >> 16);
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		uStartAddr=temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		uCheckOff=uStartAddr/temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}        
		fts_i2c_write(client, packet_buf, temp + 6);	

		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((uCheckOff + uCheckStart) == (((reg_val[0]) << 8) | reg_val[1]))
			{
				break;
			}
			
			msleep(1);

		}

		temp = (((reg_val[0]) << 8) | reg_val[1]);
		if( i == 30) FTS_COMMON_DBG("Query 6a reg time out value: 0x%x, driver set value:0x%x\n",  temp,  uCheckOff+ uCheckStart);
	}

	msleep(50);

	/********* Step 6: read out checksum ***********************/
	/*send the opration head */
	FTS_COMMON_DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0x64;
	fts_i2c_write(client, auc_i2c_write_buf, 1); 
	msleep(300);
	
	temp = uCheckStart+0;
	
	
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = (u8)(temp >> 16);
	auc_i2c_write_buf[2] = (u8)(temp >> 8);
	auc_i2c_write_buf[3] = (u8)(temp);
	
	if (dw_lenth > LEN_FLASH_ECC_MAX)
	{
		temp = LEN_FLASH_ECC_MAX;
	}
	else
	{
		temp = dw_lenth;
		FTS_COMMON_DBG("Step 6_1: read out checksum\n");
	}
	auc_i2c_write_buf[4] = (u8)(temp >> 8);
	auc_i2c_write_buf[5] = (u8)(temp);
	i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6); 
	msleep(dw_lenth/256);

	for(i = 0;i < 100;i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

		if (0xF0==reg_val[0] && 0x55==reg_val[1])
		{
			break;
		}
		msleep(1);

	}
	//----------------------------------------------------------------------
	if (dw_lenth > LEN_FLASH_ECC_MAX)
	{
		temp = LEN_FLASH_ECC_MAX;
		auc_i2c_write_buf[0] = 0x65;
		auc_i2c_write_buf[1] = (u8)(temp >> 16);
		auc_i2c_write_buf[2] = (u8)(temp >> 8);
		auc_i2c_write_buf[3] = (u8)(temp);
		temp = dw_lenth-LEN_FLASH_ECC_MAX;
		auc_i2c_write_buf[4] = (u8)(temp >> 8);
		auc_i2c_write_buf[5] = (u8)(temp);
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6); 
	

	
		msleep(dw_lenth/256);

		for(i = 0;i < 100;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if (0xF0==reg_val[0] && 0x55==reg_val[1])
			{
				break;
			}
			msleep(1);

		}
	}
	auc_i2c_write_buf[0] = 0x66;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) 
	{
		FTS_COMMON_DBG( "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
			reg_val[0],
			bt_ecc);

		return -EIO;
	}
	FTS_COMMON_DBG(KERN_WARNING "checksum %X %X \n",reg_val[0],bt_ecc);       
	/********* Step 7: reset the new FW ***********************/
	FTS_COMMON_DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(200);  

	return 0;
}
int  fts_8716_ctpm_fw_write_pram(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{

	u8 reg_val[4] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp,nowAddress=0,StartFlashAddr=0,FlashAddr=0;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 *pCheckBuffer = NULL;
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret,ReCode=-1;

	//fts_get_upgrade_info(&upgradeinfo);
	FTS_COMMON_DBG("8606 dw_lenth= %d",dw_lenth);
	if(dw_lenth > 0x10000 || dw_lenth ==0)
	{		
		return -EIO;
	}
	pCheckBuffer=kmalloc(dw_lenth+1,GFP_ATOMIC);
	for (i = 0; i < 20; i++) 
	{
		/********* Step 1:Reset  CTPM *****/
		/* write 0xaa to register 0xfc */
		//ftxxxx_reset_tp(0);
		//msleep(10);
		//ftxxxx_reset_tp(1);
		
		//msleep(10);	//time (5~20ms)


		fts_write_reg(client, 0xfc, FTS_UPGRADE_AA);
		msleep(fts_updateinfo_curr.delay_aa);
		fts_write_reg(client, 0xfc, FTS_UPGRADE_55);
		msleep(200);
		/********* Step 2:Enter upgrade mode *****/
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 1);
		if(i_ret < 0)
		{
			FTS_COMMON_DBG("[FTS] failed writing  0x55 ! \n");
			continue;
		}
		
		/*
		auc_i2c_write_buf[0] = FT_UPGRADE_AA;
		i_ret = ftxxxx_i2c_Write(client, auc_i2c_write_buf, 1);
		if(i_ret < 0)
		{
			DBG("[FTS] failed writing  0xaa ! \n");
			continue;
		}
		*/	
		/********* Step 3:check READ-ID ***********************/
		msleep(1);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;
		reg_val[0] = reg_val[1] = 0x00;
		
		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if ((reg_val[0] == 0x87 && reg_val[1] == 0x16)
			||(reg_val[0] == 0x87 && reg_val[1] == 0x36)) 
		{
			/*
			i_ret = 0x00;
			ftxxxx_read_reg(client, 0xd0, &i_ret);

			if(i_ret == 0)
			{
				DBG("[FTS] Step 3: READ State fail \n");
				continue;
			}

			DBG("[FTS] Step 3: i_ret = %d \n", i_ret);
			
			
			DBG("[FTS] Step 3: READ CTPM ID OK,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			*/
			msleep(50);
			break;
		} 
		else 
		{
			FTS_COMMON_DBG( "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);
			
			continue;
		}
	}

	if (i >= FTS_UPGRADE_LOOP )
		return -EIO;

	/********* Step 4:write firmware(FW) to ctpm flash *********/
	bt_ecc = 0;
	FTS_COMMON_DBG("Step 5:write firmware(FW) to ctpm flash\n");

	//dw_lenth = dw_lenth - 8;
	
	temp = 0;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xae;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) 
	{
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;

		for (i = 0; i < FTS_PACKET_LENGTH; i++) 
		{
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		nowAddress=nowAddress+FTS_PACKET_LENGTH;
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) 
	{
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) 
		{
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}		
		fts_i2c_write(client, packet_buf, temp + 6);
		nowAddress=nowAddress+temp;
	}
	/*
	temp = FT_APP_INFO_ADDR;
	packet_buf[2] = (u8) (temp >> 8);
	packet_buf[3] = (u8) temp;
	temp = 8;
	packet_buf[4] = (u8) (temp >> 8);
	packet_buf[5] = (u8) temp;
	for (i = 0; i < 8; i++) 
	{
		packet_buf[6+i] = pbt_buf[dw_lenth + i];
		bt_ecc ^= packet_buf[6+i];
	}	
	ftxxxx_i2c_Write(client, packet_buf, 6+8);
	*/
	/********* Step 5: read out checksum ***********************/
	/* send the opration head */
	FTS_COMMON_DBG("Step 6: read out checksum\n");
	/*auc_i2c_write_buf[0] = 0xcc;
	//msleep(2);
	ftxxxx_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) 
	{
		FTS_COMMON_DBG( "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",reg_val[0],bt_ecc);	
		return -EIO;
	}
	FTS_COMMON_DBG(KERN_WARNING "checksum %X %X \n",reg_val[0],bt_ecc);
	DBG("Read flash and compare\n");
		*/
	msleep(100);
	//-----------------------------------------------------------------------------------------------------
	FTS_COMMON_DBG( "[FTS]--nowAddress=%02x dw_lenth=%02x\n",nowAddress,dw_lenth);	
	if(nowAddress == dw_lenth)
	{

	FlashAddr=0;
	while(1)
	{
		StartFlashAddr = FlashAddr;
		if(FlashAddr == dw_lenth)
		{
			break;
		}
		else if(FlashAddr+FTS_PACKET_LENGTH > dw_lenth)
		{			
			if(!Upgrade_ReadPram(client,StartFlashAddr, pCheckBuffer+FlashAddr, dw_lenth-FlashAddr))
			{
				FTS_COMMON_DBG("read out checksum error\n");
				return -EIO;
				//break;
			}
			ReCode = ERROR_CODE_OK;
			FlashAddr = dw_lenth;
		}
		else
		{
			if(!Upgrade_ReadPram(client,StartFlashAddr, pCheckBuffer+FlashAddr, FTS_PACKET_LENGTH))
			{
				FTS_COMMON_DBG("read out checksum error\n");
				return -EIO;
				
				//break;
			}
			FlashAddr += FTS_PACKET_LENGTH;
			ReCode = ERROR_CODE_OK;
		}

		if(ReCode != ERROR_CODE_OK){
			FTS_COMMON_DBG("read out checksum error\n");
				return -EIO;
			//break;
		}
		
	}
	FTS_COMMON_DBG( "[FTS]--FlashAddr=%02x dw_lenth=%02x\n",FlashAddr,dw_lenth);
	if(FlashAddr == dw_lenth)
	{
		
		FTS_COMMON_DBG("Checking data...\n");
		for(i=0; i<dw_lenth; i++)
		{
			if(pCheckBuffer[i] != pbt_buf[i])
			{
				FTS_COMMON_DBG("read out checksum error\n");
				if(pCheckBuffer)
					kfree(pCheckBuffer);

					pCheckBuffer = NULL;
				return -EIO;
			}
		}
		if(pCheckBuffer)
					kfree(pCheckBuffer);

					pCheckBuffer = NULL;
		//COMM_FLASH_FT5422_Upgrade_StartApp(bOldProtocol, iCommMode);		//Reset
		FTS_COMMON_DBG("read out checksum successful\n");
		
	}
	else
	{
		FTS_COMMON_DBG("read out checksum error\n");
	}	

	}
	else
	{
		FTS_COMMON_DBG("read out checksum error\n");
	}	

	/********* Step 6: start app ***********************/
	FTS_COMMON_DBG("Step 6: start app\n");
	auc_i2c_write_buf[0] = 0x08;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(20);

	return 0;
}

int  fts_8716_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{
	u8 reg_val[4] = {0};
	u8 reg_val_id[4] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;
	unsigned char cmd[20];
	unsigned char Checksum = 0;
	u32 uCheckStart = 0x1000;
	u32 uCheckOff = 0x20;
	u32 uStartAddr = 0x00;
	
	//memcpy(m_DataBuffer, pFileData, m_FileLen);
	//pbt_buf=pFileData;
	auc_i2c_write_buf[0] = 0x05;
	reg_val_id[0] = 0x00;
		
	i_ret =fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val_id, 1);
	if(dw_lenth == 0)
	{
		return -EIO;
	}
	if(0x81 == (int)reg_val_id[0]) 
	{
			if(dw_lenth > 1024*60) 
			{
				return -EIO;
			}
	}
	else if(0x80 == (int)reg_val_id[0]) 
	{
			if(dw_lenth > 1024*64) 
			{
				return -EIO;
			}
	}


	/*if(dw_lenth > 1024*64)
	{
		return -EIO;
	}*/

	//fts_get_upgrade_info(&upgradeinfo);

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/********* Step 1:Reset  CTPM *****/
		//ftxxxx_write_reg(client, 0xfc, FT_UPGRADE_AA);
		//msleep(upgradeinfo.delay_aa);
		//ftxxxx_write_reg(client, 0xfc, FT_UPGRADE_55);
		//msleep(200);
		/********* Step 2:Enter upgrade mode *****/
		//i_ret = hidi2c_to_stdi2c(client);

		//if(i_ret == 0)
		//{
		//	DBG("HidI2c change to StdI2c fail ! \n");
		//	continue;
		//}
		msleep(10);
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
		if(i_ret < 0)
		{
			FTS_COMMON_DBG("failed writing  0x55 and 0xaa ! \n");
			continue;
		}

		/********* Step 3:check READ-ID ***********************/
		msleep(1);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;

		reg_val[0] = reg_val[1] = 0x00;

		fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if ((reg_val[0] == fts_updateinfo_curr.upgrade_id_1 && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)) {
				FTS_COMMON_DBG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
					reg_val[0], reg_val[1]);
				break;
		} 
		else if ((reg_val[0] == 0x87 && reg_val[1] == 0xA7)) {
				FTS_COMMON_DBG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
					reg_val[0], reg_val[1]);

				//uCheckStart = 0x1020;
				break;
		}
		else if ((reg_val[0] == 0x87 && reg_val[1] == 0xC6)) {
				FTS_COMMON_DBG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
					reg_val[0], reg_val[1]);

				//uCheckStart = 0x1020;
				break;
		}
		else {
			FTS_COMMON_DBG( "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);

			continue;
		}
	}	
	if (i >= FTS_UPGRADE_LOOP )
		return -EIO;	
	/* Step 4:erase app and panel paramenter area */
	FTS_COMMON_DBG("Step 4:erase app and panel paramenter area\n");
		
	{		
		cmd[0] = 0x05;
		cmd[1] = reg_val_id[0];//0x80;
		cmd[2] = 0x00;
		//ReCode = HY_IIC_IO(hDevice, cmd, 2, NULL, 0);	
		fts_i2c_write(client, cmd, 3);
	}
	
	// Set pramboot download mode
	//COMM_FLASH_FT5422_Upgrade_SetFlashMode(0x0B, iCommMode);
	{
		cmd[0] = 0x09;
		cmd[1] = 0x0B;
		//HY_IIC_IO(hDevice, cmd, 2, NULL, 0);
		fts_i2c_write(client, cmd, 2);
	}	
	for(i=0; i<dw_lenth ; i++)
	{		
		Checksum ^= pbt_buf[i];
	}
	msleep(50);

	// erase app area 
	
	auc_i2c_write_buf[0] = 0x61;
	fts_i2c_write(client, auc_i2c_write_buf, 1);     
	msleep(1350);

	for(i = 0;i < 15;i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

		if(0xF0==reg_val[0] && 0xAA==reg_val[1])
		{
			break;
		}
		msleep(50);
	}
	
	/********* Step 5:write firmware(FW) to ctpm flash *********/
	bt_ecc = 0;
	FTS_COMMON_DBG("Step 5:write firmware(FW) to ctpm flash\n");
	
	//dw_lenth = dw_lenth - 8;
	temp = 0;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	//packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) {
		temp = uCheckStart+j * FTS_PACKET_LENGTH;
		packet_buf[1] = (u8) (temp >> 16);
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		uStartAddr=temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;
		
		uCheckOff = uStartAddr/lenght;

		for (i = 0; i < FTS_PACKET_LENGTH; i++) 
		{
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		//msleep(10);

		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((uCheckOff +uCheckStart) == (((reg_val[0]) << 8) | reg_val[1]))
			{
				break;
			}
			msleep(1);
		}

		temp = (((reg_val[0]) << 8) | reg_val[1]);
		if( i == 30) FTS_COMMON_DBG("Query 6a reg time out value: 0x%x, driver set value:0x%x\n",  temp, uCheckOff + uCheckStart);
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = uCheckStart+packet_number * FTS_PACKET_LENGTH;
		packet_buf[1] = (u8) (temp >> 16);
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		uStartAddr=temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		uCheckOff=uStartAddr/temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}        
		fts_i2c_write(client, packet_buf, temp + 6);	

		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((uCheckOff + uCheckStart) == (((reg_val[0]) << 8) | reg_val[1]))
			{
				break;
			}
			
			msleep(1);

		}

		temp = (((reg_val[0]) << 8) | reg_val[1]);
		if( i == 30) FTS_COMMON_DBG("Query 6a reg time out value: 0x%x, driver set value:0x%x\n",  temp,  uCheckOff+ uCheckStart);
	}

	msleep(50);

	/********* Step 6: read out checksum ***********************/
	/*send the opration head */
	FTS_COMMON_DBG("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0x64;
	fts_i2c_write(client, auc_i2c_write_buf, 1); 
	msleep(300);
	
	temp = uCheckStart+0;
	
	
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = (u8)(temp >> 16);
	auc_i2c_write_buf[2] = (u8)(temp >> 8);
	auc_i2c_write_buf[3] = (u8)(temp);
	
	if (dw_lenth > LEN_FLASH_ECC_MAX)
	{
		temp = LEN_FLASH_ECC_MAX;
	}
	else
	{
		temp = dw_lenth;
		FTS_COMMON_DBG("Step 6_1: read out checksum\n");
	}
	auc_i2c_write_buf[4] = (u8)(temp >> 8);
	auc_i2c_write_buf[5] = (u8)(temp);
	i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6); 
	msleep(dw_lenth/256);

	for(i = 0;i < 100;i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

		if (0xF0==reg_val[0] && 0x55==reg_val[1])
		{
			break;
		}
		msleep(1);

	}
	//----------------------------------------------------------------------
	if (dw_lenth > LEN_FLASH_ECC_MAX)
	{
		temp = LEN_FLASH_ECC_MAX;
		auc_i2c_write_buf[0] = 0x65;
		auc_i2c_write_buf[1] = (u8)(temp >> 16);
		auc_i2c_write_buf[2] = (u8)(temp >> 8);
		auc_i2c_write_buf[3] = (u8)(temp);
		temp = dw_lenth-LEN_FLASH_ECC_MAX;
		auc_i2c_write_buf[4] = (u8)(temp >> 8);
		auc_i2c_write_buf[5] = (u8)(temp);
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 6); 
	

	
		msleep(dw_lenth/256);

		for(i = 0;i < 100;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if (0xF0==reg_val[0] && 0x55==reg_val[1])
			{
				break;
			}
			msleep(1);

		}
	}
	auc_i2c_write_buf[0] = 0x66;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) 
	{
		FTS_COMMON_DBG( "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
			reg_val[0],
			bt_ecc);

		return -EIO;
	}
	FTS_COMMON_DBG(KERN_WARNING "checksum %X %X \n",reg_val[0],bt_ecc);       
	/********* Step 7: reset the new FW ***********************/
	FTS_COMMON_DBG("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(200);  

	return 0;
}


/*
*	note:the firmware default path is sdcard.
	if you want to change the dir, please modify by yourself.
*/
/************************************************************************
* Name: fts_GetFirmwareSize
* Brief:  get file size
* Input: file name
* Output: no
* Return: file size
***********************************************************************/
static int fts_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];

	memset(filepath, 0, sizeof(filepath)); 
       sprintf(filepath, "%s%s", FTXXXX_INI_FILEPATH_CONFIG, firmware_name);
	if (NULL == pfile)
	{
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if (IS_ERR(pfile)) 
	{
		FTS_COMMON_DBG("error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

/************************************************************************
* Name: fts_ReadFirmware
* Brief:  read firmware buf for .bin file.
* Input: file name, data buf
* Output: data buf
* Return: 0
***********************************************************************/
/*
	note:the firmware default path is sdcard.
	if you want to change the dir, please modify by yourself.
*/
static int fts_ReadFirmware(char *firmware_name,unsigned char *firmware_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FTXXXX_INI_FILEPATH_CONFIG, firmware_name);
	if (NULL == pfile)
	{
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if (IS_ERR(pfile)) 
	{
		FTS_COMMON_DBG("error occured while opening file %s.\n", filepath);
		return -EIO;
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

/************************************************************************
* Name: fts_ctpm_fw_upgrade_with_app_file
* Brief:  upgrade with *.bin file
* Input: i2c info, file name
* Output: no
* Return: success =0
***********************************************************************/
int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client *client, char *firmware_name)
{
	u8 *pbt_buf = NULL;
	int i_ret=0;
	int fwsize = fts_GetFirmwareSize(firmware_name);
	
	FTS_COMMON_DBG("\n FTS zax ID=%d size=%d \n",fts_updateinfo_curr.CHIP_ID,fwsize);
	if (fwsize <= 0) 
	{
		FTS_COMMON_DBG( "%s ERROR:Get firmware size failed\n",__func__);
		return -EIO;
	}
	if ((fts_updateinfo_curr.CHIP_ID!=0x86) && (fts_updateinfo_curr.CHIP_ID!=0x87))
	{
		if (fwsize < 8 || fwsize > 54 * 1024) 
		{
			FTS_COMMON_DBG( "FW length error\n");
			return -EIO;
		}
	}
	/*========= FW upgrade ========================*/
	pbt_buf = (unsigned char *)kmalloc(fwsize + 1, GFP_ATOMIC);
	if (fts_ReadFirmware(firmware_name, pbt_buf)) 
	{
		FTS_COMMON_DBG( "%s() - ERROR: request_firmware failed\n",__func__);
		kfree(pbt_buf);
		return -EIO;
	}	
	/* call the upgrade function */
	if ((fts_updateinfo_curr.CHIP_ID==0x55) ||(fts_updateinfo_curr.CHIP_ID==0x08) ||(fts_updateinfo_curr.CHIP_ID==0x0a))
	{
		i_ret = fts_5x06_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	}
	else if ((fts_updateinfo_curr.CHIP_ID==0x11) ||(fts_updateinfo_curr.CHIP_ID==0x12) ||(fts_updateinfo_curr.CHIP_ID==0x13) ||(fts_updateinfo_curr.CHIP_ID==0x14))
	{
		i_ret = fts_5x36_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	}
	else if ((fts_updateinfo_curr.CHIP_ID==0x06))
	{
		i_ret = fts_6x06_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	}
	else if ((fts_updateinfo_curr.CHIP_ID==0x36))
	{
		i_ret = fts_6x36_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	}
	else if ((fts_updateinfo_curr.CHIP_ID==0x54))
	{
		i_ret = fts_5x46_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	}
	else if ((fts_updateinfo_curr.CHIP_ID==0x58))
	{
		i_ret =  fts_5822_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	}
	else if ((fts_updateinfo_curr.CHIP_ID==0x59))
	{
		i_ret =  fts_5x26_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	}
	else if ((fts_updateinfo_curr.CHIP_ID==0x86))
	{
	   	
    		i_ret = fts_8606_ctpm_fw_write_pram(client, aucFW_PRAM_BOOT, sizeof(aucFW_PRAM_BOOT));
	
		if (i_ret != 0)
		{
			FTS_COMMON_DBG( "%s:WritePram failed. err.\n",__func__);
			return -EIO;
		}

	
		
		i_ret =  fts_8606_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	}
	else if ((fts_updateinfo_curr.CHIP_ID==0x87))
	{
	   	
    		i_ret = fts_8716_ctpm_fw_write_pram(client, aucFW_PRAM_BOOT, sizeof(aucFW_PRAM_BOOT));
	
		if (i_ret != 0)
		{
			FTS_COMMON_DBG( "%s:WritePram failed. err.\n",__func__);
			return -EIO;
		}

	
		
		i_ret =  fts_8716_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	}
	if (i_ret != 0)
		FTS_COMMON_DBG( "%s() - ERROR:[FTS] upgrade failed..\n",
					__func__);
	else if (fts_updateinfo_curr.AUTO_CLB==AUTO_CLB_NEED)
	{
		fts_ctpm_auto_clb(client);
	}
	
	kfree(pbt_buf);
	FTS_COMMON_DBG("[FTS] zax end=%d \n",i_ret );
	return i_ret;
}
/************************************************************************
* Name: fts_ctpm_get_i_file_ver
* Brief:  get .i file version
* Input: no
* Output: no
* Return: fw version
***********************************************************************/
int fts_ctpm_get_i_file_ver(void)
{
	u16 ui_sz;
	ui_sz = sizeof(CTPM_FW);
	if (ui_sz > 2)
	{
	    if(fts_updateinfo_curr.CHIP_ID==0x36)
                return CTPM_FW[0x10A];
	    if(fts_updateinfo_curr.CHIP_ID==0x86  || fts_updateinfo_curr.CHIP_ID==0x87)
                return CTPM_FW[0x10E];
	    else if(fts_updateinfo_curr.CHIP_ID==0x58)
                return CTPM_FW[0x1D0A];	//0x010A + 0x1C00
	    else
		return CTPM_FW[ui_sz - 2];
	}

	return 0x00;						/*default value */
}
/************************************************************************
* Name: fts_ctpm_update_project_setting
* Brief:  update project setting, only update these settings for COB project, or for some special case
* Input: i2c info
* Output: no
* Return: fail <0
***********************************************************************/
int fts_ctpm_update_project_setting(struct i2c_client *client)
{
	u8 uc_i2c_addr;				/* I2C slave address (7 bit address) */
	u8 uc_io_voltage;			/* IO Voltage 0---3.3v;1----1.8v */
	u8 uc_panel_factory_id;		/* TP panel factory ID */
	u8 buf[FTS_PACKET_LENGTH];//zax original:128
	u8 reg_val[2] = {0};
	u8 auc_i2c_write_buf[10] = {0};
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u32 i = 0;
	int i_ret;

	uc_i2c_addr = client->addr;
	uc_io_voltage = 0x0;
	uc_panel_factory_id = 0x5a;


	/* Step 1:Reset  CTPM
	*   write 0xaa to register 0xfc
	*/
	if(fts_updateinfo_curr.CHIP_ID==0x06 || fts_updateinfo_curr.CHIP_ID==0x36)
	{
		fts_write_reg(client, 0xbc, 0xaa);
	}
	else 
	{
		fts_write_reg(client, 0xfc, 0xaa);
	}
	msleep(50);

	/* write 0x55 to register 0xfc */
	if(fts_updateinfo_curr.CHIP_ID==0x06 || fts_updateinfo_curr.CHIP_ID==0x36)
	{
		fts_write_reg(client, 0xbc, 0x55);
	}
	else
	{
		fts_write_reg(client, 0xfc, 0x55);
	}
	msleep(30);

	/********* Step 2:Enter upgrade mode *****/
	auc_i2c_write_buf[0] = 0x55;
	auc_i2c_write_buf[1] = 0xaa;
	do 
	{
		i++;
		i_ret = fts_i2c_write(client, auc_i2c_write_buf, 2);
		msleep(5);
	} while (i_ret <= 0 && i < 5);


	/********* Step 3:check READ-ID ***********************/
	auc_i2c_write_buf[0] = 0x90;
	auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] =
			0x00;

	fts_i2c_read(client, auc_i2c_write_buf, 4, reg_val, 2);

	if(reg_val[0] == fts_updateinfo_curr.upgrade_id_1 && reg_val[1] == fts_updateinfo_curr.upgrade_id_2)
		FTS_COMMON_DBG( "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0], reg_val[1]);
	else
		return -EIO;

	auc_i2c_write_buf[0] = 0xcd;
	fts_i2c_read(client, auc_i2c_write_buf, 1, reg_val, 1);
	FTS_COMMON_DBG( "bootloader version = 0x%x\n", reg_val[0]);

	/*--------- read current project setting  ---------- */
	/* set read start address */
	buf[0] = 0x3;
	buf[1] = 0x0;
	buf[2] = 0x78;
	buf[3] = 0x0;

	fts_i2c_read(client, buf, 4, buf, FTS_PACKET_LENGTH);
	FTS_COMMON_DBG( "[FTS] old setting: uc_i2c_addr = 0x%x,\
			uc_io_voltage = %d, uc_panel_factory_id = 0x%x\n",
			buf[0], buf[2], buf[4]);

	 /*--------- Step 4:erase project setting --------------*/
	auc_i2c_write_buf[0] = 0x63;
	fts_i2c_write(client, auc_i2c_write_buf, 1);
	msleep(100);

	/*----------  Set new settings ---------------*/
	buf[0] = uc_i2c_addr;
	buf[1] = ~uc_i2c_addr;
	buf[2] = uc_io_voltage;
	buf[3] = ~uc_io_voltage;
	buf[4] = uc_panel_factory_id;
	buf[5] = ~uc_panel_factory_id;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;
	packet_buf[2] = 0x78;
	packet_buf[3] = 0x0;
	packet_buf[4] = 0;
	packet_buf[5] = FTS_PACKET_LENGTH;

	for (i = 0; i < FTS_PACKET_LENGTH; i++)
		packet_buf[6 + i] = buf[i];

	fts_i2c_write(client, packet_buf, FTS_PACKET_LENGTH + 6);
	msleep(100);

	/********* reset the new FW ***********************/
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write(client, auc_i2c_write_buf, 1);

	msleep(200);
	return 0;
}
/************************************************************************
* Name: fts_ctpm_fw_upgrade_with_i_file
* Brief:  upgrade with *.i file
* Input: i2c info
* Output: no
* Return: fail <0
***********************************************************************/
int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client)
{
	u8 *pbt_buf = NULL;
	int i_ret=0;
	int fw_len = sizeof(CTPM_FW);

	/*  judge the fw that will be upgraded
	*    if illegal, then stop upgrade and return.
	*/
	FTS_COMMON_DBG("\n FTS zax ID=%d fw_len=%d \n",fts_updateinfo_curr.CHIP_ID,fw_len);
	if ((fts_updateinfo_curr.CHIP_ID==0x11) ||(fts_updateinfo_curr.CHIP_ID==0x12) ||(fts_updateinfo_curr.CHIP_ID==0x13) ||(fts_updateinfo_curr.CHIP_ID==0x14)
		||(fts_updateinfo_curr.CHIP_ID==0x55) ||(fts_updateinfo_curr.CHIP_ID==0x06) ||(fts_updateinfo_curr.CHIP_ID==0x0a) ||(fts_updateinfo_curr.CHIP_ID==0x08))
	{
		if (fw_len < 8 || fw_len > 32 * 1024) 
		{
			FTS_COMMON_DBG( "%s:FW length error\n", __func__);
			return -EIO;
		}

		if ((CTPM_FW[fw_len - 8] ^ CTPM_FW[fw_len - 6]) == 0xFF
			&& (CTPM_FW[fw_len - 7] ^ CTPM_FW[fw_len - 5]) == 0xFF
			&& (CTPM_FW[fw_len - 3] ^ CTPM_FW[fw_len - 4]) == 0xFF) 
		{
			/* FW upgrade */
			pbt_buf = CTPM_FW;
			/* call the upgrade function */
			if ((fts_updateinfo_curr.CHIP_ID==0x55) ||(fts_updateinfo_curr.CHIP_ID==0x08) ||(fts_updateinfo_curr.CHIP_ID==0x0a))
			{
				i_ret = fts_5x06_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
			}
			else if ((fts_updateinfo_curr.CHIP_ID==0x11) ||(fts_updateinfo_curr.CHIP_ID==0x12) ||(fts_updateinfo_curr.CHIP_ID==0x13) ||(fts_updateinfo_curr.CHIP_ID==0x14))
			{
				i_ret = fts_5x36_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
			}
			else if ((fts_updateinfo_curr.CHIP_ID==0x06))
			{
				i_ret = fts_6x06_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
			}
			if (i_ret != 0)
				FTS_COMMON_DBG( "%s:upgrade failed. err.\n",__func__);
			else if(fts_updateinfo_curr.AUTO_CLB==AUTO_CLB_NEED)
			{
				fts_ctpm_auto_clb(client);
			}
		} 
		else 
		{
			FTS_COMMON_DBG( "%s:FW format error\n", __func__);
			return -EBADFD;
		}
	}
	else if ((fts_updateinfo_curr.CHIP_ID==0x36))
	{            
		if (fw_len < 8 || fw_len > 32 * 1024) 
		{
			FTS_COMMON_DBG( "%s:FW length error\n", __func__);
			return -EIO;
		}
		pbt_buf = CTPM_FW;
		i_ret = fts_6x36_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
		if (i_ret != 0)
			FTS_COMMON_DBG( "%s:upgrade failed. err.\n",__func__);
	}
	else if ((fts_updateinfo_curr.CHIP_ID==0x54))
	{
              #if FT_TP
			if(0x51==ucPVendorID)
			{
				pbt_buf = CTPM_FW_1;
				fw_len = sizeof(CTPM_FW_1);
			}
			else if(0xa0==ucPVendorID)
			{
				pbt_buf = CTPM_FW_2;
				fw_len = sizeof(CTPM_FW_2);
			}
		#else
			pbt_buf = CTPM_FW;
			fw_len=sizeof(CTPM_FW);
		#endif
		if (fw_len < 8 || fw_len > 54 * 1024) 
		{
			FTS_COMMON_DBG("FW length error\n");
			return -EIO;
		}
		/* FW upgrade */
		
		/* call the upgrade function */
		i_ret = fts_5x46_ctpm_fw_upgrade(client, pbt_buf, fw_len);
		if (i_ret != 0)
		{
			FTS_COMMON_DBG( "[FTS] upgrade failed. err=%d.\n", i_ret);
		}
		else 
		{
			#ifdef AUTO_CLB
				fts_ctpm_auto_clb(client);  /* start auto CLB */
			#endif
		}
	}	
	else if ((fts_updateinfo_curr.CHIP_ID==0x58))
	{
		if (fw_len < 8 || fw_len > 54*1024) 
		{
			FTS_COMMON_DBG("FW length error\n");
			return -EIO;
		}

		/* FW upgrade */
		pbt_buf = CTPM_FW;
		/* call the upgrade function */
		i_ret = fts_5822_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
		if (i_ret != 0) 
		{
			FTS_COMMON_DBG( "[FTS] upgrade failed. err=%d.\n", i_ret);
		} 
		else 
		{
			#ifdef AUTO_CLB
				fts_ctpm_auto_clb(client);  /* start auto CLB */
			#endif
		}		
	}
	else if ((fts_updateinfo_curr.CHIP_ID==0x59))
	{
	    	if (fw_len < 8 || fw_len > 54*1024) 
		{
		    	FTS_COMMON_DBG("FW length error\n");
	    		return -EIO;
	    	}

	    	/* FW upgrade */
    		pbt_buf = CTPM_FW;
	   	 /* call the upgrade function */
    		i_ret = fts_5x26_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
    		if (i_ret != 0) 
		{
    			FTS_COMMON_DBG( "[FTS] upgrade failed. err=%d.\n", i_ret);
    		} 
		else 
		{
			#ifdef AUTO_CLB
    				fts_ctpm_auto_clb(client);  /* start auto CLB */
			#endif
    		}
	}
	else if ((fts_updateinfo_curr.CHIP_ID==0x86))
	{
	    	

	    	/* FW upgrade */
    		pbt_buf = CTPM_FW;
	   	 /* call the upgrade function */
    		i_ret = fts_8606_ctpm_fw_write_pram(client, aucFW_PRAM_BOOT, sizeof(aucFW_PRAM_BOOT));
	
		if (i_ret != 0)
		{
			FTS_COMMON_DBG( "%s:WritePram failed. err.\n",__func__);
			return -EIO;
		}

	
		
		i_ret =  fts_8606_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
	

    		if (i_ret != 0) 
		{
    			FTS_COMMON_DBG( "[FTS] upgrade failed. err=%d.\n", i_ret);
    		} 
		else 
		{
			#ifdef AUTO_CLB
    				fts_ctpm_auto_clb(client);  /* start auto CLB */
			#endif
    		}
	}
	else if ((fts_updateinfo_curr.CHIP_ID==0x87))
	{
	    	

	    	/* FW upgrade */
    		pbt_buf = CTPM_FW;
	   	 /* call the upgrade function */
    		i_ret = fts_8716_ctpm_fw_write_pram(client, aucFW_PRAM_BOOT, sizeof(aucFW_PRAM_BOOT));
	
		if (i_ret != 0)
		{
			FTS_COMMON_DBG( "%s:WritePram failed. err.\n",__func__);
			return -EIO;
		}

	
		
		i_ret =  fts_8716_ctpm_fw_upgrade(client, pbt_buf, sizeof(CTPM_FW));
	

    		if (i_ret != 0) 
		{
    			FTS_COMMON_DBG( "[FTS] upgrade failed. err=%d.\n", i_ret);
    		} 
		else 
		{
			#ifdef AUTO_CLB
    				fts_ctpm_auto_clb(client);  /* start auto CLB */
			#endif
    		}
	}
	
	FTS_COMMON_DBG("[FTS] zax end=%d \n",i_ret );
	
	return i_ret;
}

/************************************************************************
* Name: fts_ctpm_auto_upgrade
* Brief:  auto upgrade
* Input: i2c info
* Output: no
* Return: 0
***********************************************************************/
int fts_ctpm_auto_upgrade(struct i2c_client *client)
{
	
	u8 uc_host_fm_ver = FTS_REG_FW_VER;
	u8 uc_tp_fm_ver;
	int i_ret;
	u8 uc_chip_id=0;
	u8 uc_vender_id=0;
	
	FTS_COMMON_DBG("[focal] %s \n", FTS_AUTO_UPGRADE_INFO);	//show version
	fts_read_reg(client, FTS_REG_CHIP_ID, &uc_chip_id);	

	if(uc_chip_id == fts_updateinfo_curr.CHIP_ID)//call fts_get_upgrade_array in probe function firstly.
	{	
		fts_read_reg(client, FTS_REG_FW_VER, &uc_tp_fm_ver);
		
		uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		FTS_COMMON_DBG("[out] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",uc_tp_fm_ver, uc_host_fm_ver);
		
		if (uc_tp_fm_ver == FTS_REG_FW_VER ||	uc_tp_fm_ver < uc_host_fm_ver ) 
		{
			fts_read_reg(client, FTS_REG_FW_VENDOR_ID, &uc_vender_id);

			if(uc_vender_id)//select fw according uc_vender_id by yourself
			{
				memcpy(CTPM_FW, CTPM_FW, sizeof(CTPM_FW));
			}
			else
			{
				memcpy(CTPM_FW, CTPM_FW, sizeof(CTPM_FW));
			}
			
			msleep(100);
			FTS_COMMON_DBG("[in] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",uc_tp_fm_ver, uc_host_fm_ver);
			#if FT_ESD_PROTECT
			esd_switch(0);apk_debug_flag = 1;
			#endif
			i_ret = fts_ctpm_fw_upgrade_with_i_file(client);

			#if FT_ESD_PROTECT
				esd_switch(1);apk_debug_flag = 0;
			#endif
			
			if (i_ret == 0)	
			{
				msleep(300);
				//uc_host_fm_ver = fts_ctpm_get_i_file_ver();
				fts_read_reg(client, FTS_REG_FW_VER, &uc_tp_fm_ver);
				FTS_COMMON_DBG("[FTS] upgrade to new version 0x%x\n",uc_tp_fm_ver);
			} 
			else
			{
				FTS_COMMON_DBG("[FTS] upgrade failed ret=%d.\n", i_ret);
				return -EIO;
			}
		}
	}
	else
	{
		#if FT_ESD_PROTECT
			esd_switch(0);apk_debug_flag = 1;
		#endif
		i_ret = fts_ctpm_fw_upgrade_ReadChipID_Array(client, &uc_chip_id);
		FTS_COMMON_DBG("[FTS] read chip id from boot is 0x%x\n",uc_chip_id);

		if (i_ret == 0)	 i_ret = fts_ctpm_fw_upgrade_ReadVendorID(client, &uc_vender_id);
		FTS_COMMON_DBG("[FTS] read vendor id from boot is 0x%x\n",uc_vender_id);
		
		if (i_ret == 0)
		{
			if(uc_vender_id)//select fw according uc_vender_id by yourself
			{
				memcpy(CTPM_FW, CTPM_FW, sizeof(CTPM_FW));
			}
			else
			{
				memcpy(CTPM_FW, CTPM_FW, sizeof(CTPM_FW));
			}
		}
			
		if (i_ret == 0)	 i_ret = fts_ctpm_fw_upgrade_with_i_file(client);

		#if FT_ESD_PROTECT
			esd_switch(1);apk_debug_flag = 0;
		#endif
		
		if (i_ret == 0)	
		{
			msleep(300);
			//uc_host_fm_ver = fts_ctpm_get_i_file_ver();
			fts_read_reg(client, FTS_REG_FW_VER, &uc_tp_fm_ver);
			FTS_COMMON_DBG("[FTS] upgrade to new version 0x%x\n",uc_tp_fm_ver);
		} 
		else
		{
			FTS_COMMON_DBG("[FTS] upgrade failed ret=%d.\n", i_ret);
			return -EIO;
		}		
	}
	
	return 0;
}
