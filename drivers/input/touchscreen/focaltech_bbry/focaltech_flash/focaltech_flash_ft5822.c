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

 /*******************************************************************************
*
* File Name: focaltech_core.c
*
*  Author: Xu YF & ZR, Software Department, FocalTech
*
* Created: 2016-03-21
*
*  Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "../focaltech_comm.h"
#include "focaltech_flash.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FOCALTECH_FLASH_FT5822_INFO  "File Version of  focaltech_flash_ft5822.c:  V1.0.0 2016-03-21"

#define FTS_REG_FT5822_VENDOR_ID 		0xA8


/************************************************************************
*   Name: fts_ic_program_ft5822_upgrade
* Brief:  fw upgrade
* Input: i2c info, file buf, file len
* Output: no
* Return: fail <0
***********************************************************************/
static int fts_ic_program_ft5822_upgrade( u8 *pbt_buf, u32 dw_lenth)
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

	i_ret = fts_protocol_windows_to_android();
	if (i_ret == 0) 
	{
		FTS_COMMON_DBG("HidI2c change to StdI2c fail ! ");
	}
	
	for (i = 0; i < FTS_UPGRADE_LOOP; i++) 
	{
		/********* Step 1:Reset  CTPM *****/
		fts_write_reg( 0xfc, FTS_UPGRADE_AA);
		msleep(fts_upgrade_info_curr.delay_aa);
		fts_write_reg( 0xfc, FTS_UPGRADE_55);
		msleep(200);
		/********* Step 2:Enter upgrade mode *****/
		i_ret = fts_protocol_windows_to_android();
		if (i_ret == 0) 
		{
			FTS_COMMON_DBG("HidI2c change to StdI2c fail ! ");
			continue;
		}
		msleep(5);
		auc_i2c_write_buf[0] = FTS_UPGRADE_55;
		auc_i2c_write_buf[1] = FTS_UPGRADE_AA;
		i_ret = fts_i2c_write( auc_i2c_write_buf, 2);	
		if (i_ret < 0) 
		{
			FTS_COMMON_DBG("failed writing  0x55 and 0xaa ! ");
			continue;
		}
		/*********Step 3:check READ-ID***********************/
		msleep(1);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read( auc_i2c_write_buf, 4, reg_val, 2);
		if (reg_val[0] == fts_upgrade_info_curr.upgrade_id_1 && reg_val[1] == fts_upgrade_info_curr.upgrade_id_2) 
		{	
			/* relate on bootloader FW*/
			FTS_COMMON_DBG("[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x",  reg_val[0], reg_val[1]);
			break;
		} 
		else 
		{
			FTS_COMMON_DBG( "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x",  reg_val[0], reg_val[1]);
			continue;
		}
	}
	if (i >= FTS_UPGRADE_LOOP)
	{
		//��������λ���ɹ�������Ӳ����λ
		if(0 != fts_upgrade_hardware_reset(200, fts_upgrade_info_curr.upgrade_id_1, fts_upgrade_info_curr.upgrade_id_2))
			return -EIO;
	}
	
	/* Step 4:erase app and panel paramenter area */
	FTS_COMMON_DBG("Step 4:erase app and panel paramenter area");
	auc_i2c_write_buf[0] = 0x61;
	fts_i2c_write( auc_i2c_write_buf, 1);	/*erase app area, trigger erase command */
	msleep(1350);
	for (i = 0; i < 15; i++) 
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read( auc_i2c_write_buf, 1, reg_val, 2);
		if (0xF0 == reg_val[0] && 0xAA == reg_val[1]) 
		{
			break;
		}
		msleep(50);
	}
	FTS_COMMON_DBG("[FTS][%s] erase app area reg_val[0] = %x reg_val[1] = %x ",  __func__, reg_val[0], reg_val[1]);
	/* write bin file length to FW bootloader. */
	auc_i2c_write_buf[0] = 0xB0;
	auc_i2c_write_buf[1] = (u8) ((dw_lenth >> 16) & 0xFF);
	auc_i2c_write_buf[2] = (u8) ((dw_lenth >> 8) & 0xFF);
	auc_i2c_write_buf[3] = (u8) (dw_lenth & 0xFF);
	fts_i2c_write( auc_i2c_write_buf, 4);
	/********* Step 5:write firmware(FW) to ctpm flash *********/
	bt_ecc = 0;
	bt_ecc_check = 0;
	FTS_COMMON_DBG("Step 5:write firmware(FW) to ctpm flash");
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
		FTS_COMMON_DBG("[FTS][%s] bt_ecc = %x ",  __func__, bt_ecc);
		if (bt_ecc != bt_ecc_check)
			FTS_COMMON_DBG("[FTS][%s] Host checksum error bt_ecc_check = %x ",  __func__, bt_ecc_check);
		fts_i2c_write( packet_buf, FTS_PACKET_LENGTH + 6);
		/*msleep(10);*/
		for (i = 0; i < 30; i++) 
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read( auc_i2c_write_buf, 1, reg_val, 2);
			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1])) {
				break;
			}
			FTS_COMMON_DBG("[FTS][%s] reg_val[0] = %x reg_val[1] = %x ",  __func__, reg_val[0], reg_val[1]);
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
		fts_i2c_write( packet_buf, temp + 6);
		FTS_COMMON_DBG("[FTS][%s] bt_ecc = %x ",  __func__, bt_ecc);
		if (bt_ecc != bt_ecc_check)
			FTS_COMMON_DBG("[FTS][%s] Host checksum error bt_ecc_check = %x ",  __func__, bt_ecc_check);
		for (i = 0; i < 30; i++) 
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			fts_i2c_read( auc_i2c_write_buf, 1, reg_val, 2);
			FTS_COMMON_DBG("[FTS][%s] reg_val[0] = %x reg_val[1] = %x ",  __func__, reg_val[0], reg_val[1]);
			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1])) 
			{
				break;
			}
			FTS_COMMON_DBG("[FTS][%s] reg_val[0] = %x reg_val[1] = %x ",  __func__, reg_val[0], reg_val[1]);
			msleep(1);
		}
	}
	msleep(50);
	/********* Step 6: read out checksum ***********************/
	/* send the opration head */
	FTS_COMMON_DBG("Step 6: read out checksum");
	auc_i2c_write_buf[0] = 0x64;
	fts_i2c_write( auc_i2c_write_buf, 1);
	msleep(300);
	temp = 0;
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = (u8)(temp >> 16);
	auc_i2c_write_buf[2] = (u8)(temp >> 8);
	auc_i2c_write_buf[3] = (u8)(temp);
	temp = dw_lenth;
	auc_i2c_write_buf[4] = (u8)(temp >> 8);
	auc_i2c_write_buf[5] = (u8)(temp);
	i_ret = fts_i2c_write( auc_i2c_write_buf, 6);
	msleep(dw_lenth/256);
	for (i = 0; i < 100; i++) 
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		fts_i2c_read( auc_i2c_write_buf, 1, reg_val, 2);
		FTS_COMMON_DBG( "[FTS]--reg_val[0]=%02x reg_val[0]=%02x",  reg_val[0], reg_val[1]);
		if (0xF0 == reg_val[0] && 0x55 == reg_val[1]) 
		{
			FTS_COMMON_DBG( "[FTS]--reg_val[0]=%02x reg_val[0]=%02x",  reg_val[0], reg_val[1]);
			break;
		}
		msleep(1);
	}
	auc_i2c_write_buf[0] = 0x66;
	fts_i2c_read( auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) 
	{
		FTS_COMMON_DBG( "[FTS]--ecc error! fw_ecc=%02x flash_ecc=%02x",  reg_val[0], bt_ecc);
		return -EIO;
	}
	FTS_COMMON_DBG(KERN_WARNING "checksum fw_ecc=%X flash_ecc=%X ",  reg_val[0], bt_ecc);
	/********* Step 7: reset the new FW ***********************/
	FTS_COMMON_DBG("Step 7: reset the new FW");
	auc_i2c_write_buf[0] = 0x07;
	fts_i2c_write( auc_i2c_write_buf, 1);
	msleep(200);						/* make sure CTP startup normally */
	i_ret = fts_protocol_windows_to_android();	/* Android to Std i2c. */
	if (i_ret == 0) 
	{
		FTS_COMMON_DBG("HidI2c change to StdI2c fail ! ");
	}
	return 0;
}

static int fts_ft5822_get_i_file(u8 **file_buff, int *file_len)
{
	int i_ret = 0;
	u8 uc_vender_id= 0;
	
	if(USE_I_FILES_NUMBER > 1)
	{
		i_ret = fts_read_reg(FTS_REG_FT5822_VENDOR_ID, &uc_vender_id);
		if(i_ret < 0)
		{
			i_ret = fts_upgrade_read_vendor_id( &uc_vender_id);
			if(i_ret < 0)
			{
				FTS_COMMON_DBG("Failed to get Vendor ID! error code = %d", i_ret);
				return i_ret;
			}
		}
		FTS_COMMON_DBG("[FTS] read vendor id from boot is 0x%x", uc_vender_id);		
		if(USE_I_FILES_NUMBER == 2)
		{
			switch(uc_vender_id)
			{
				case USE_I_FILE_1_BY_WENDOR_ID_1:
					*file_buff = BUFFER_I_FILE_1;
					*file_len = g_len_i_file_1;	
					break;
				case USE_I_FILE_2_BY_WENDOR_ID_2:
					*file_buff = BUFFER_I_FILE_2;
					*file_len = g_len_i_file_2;	
					break;			
				default:
				{
					FTS_COMMON_DBG("Failed to get i file! Invalid wendor id, wendor id = %d", uc_vender_id);
					return -1;
				}
						
			}			
		}
		else if(USE_I_FILES_NUMBER == 3)
		{
			switch(uc_vender_id)
			{
				case USE_I_FILE_1_BY_WENDOR_ID_1:
					*file_buff = BUFFER_I_FILE_1;
					*file_len = g_len_i_file_1;	
					break;
				case USE_I_FILE_2_BY_WENDOR_ID_2:
					*file_buff = BUFFER_I_FILE_2;
					*file_len = g_len_i_file_2;	
					break;
				case USE_I_FILE_3_BY_WENDOR_ID_3:
					*file_buff = BUFFER_I_FILE_3;
					*file_len = g_len_i_file_3;	
					break;				
				default:
				{
					FTS_COMMON_DBG("Failed to get i file! Invalid wendor id, wendor id = %d", uc_vender_id);
					return -1;
				}
						
			}
		}
		else
		{
			FTS_COMMON_DBG("Failed to get i file, this is too many files, file num = %d", USE_I_FILES_NUMBER);
			return -1;
		}
		
	}
	else
	{
		*file_buff = BUFFER_I_FILE_1;
		*file_len = g_len_i_file_1;		
	}
	
	return 0;
}
int fts_ft5822_upgrade_with_i_file(void)
{
	u8 *pbt_buf = NULL;
	int i_ret=0;
	int fw_len = 0;

	/*get i file*/
	i_ret = fts_ft5822_get_i_file(&pbt_buf, &fw_len);
	if(i_ret < 0)
	{
		return -1;
	}
	else
	{
		FTS_COMMON_DBG("the file length of i file: %d",  fw_len);
	}
	if(NULL == pbt_buf)
	{
		FTS_COMMON_DBG("the file buff is NULL.");
		return -1;
	}

	if (fw_len < 8 || fw_len > 54 * 1024) 
	{
		FTS_COMMON_DBG("FW length error");
		return -EIO;
	}

	/*call upgrade function*/
	i_ret = fts_ic_program_ft5822_upgrade( pbt_buf, fw_len);

	#ifdef AUTO_CLB
		fts_ctpm_auto_clb();  /* start auto CLB */
	#endif
	
	return i_ret;
}
int fts_ft5822_get_i_file_version(void)
{
	u8 *pbt_buf = NULL;
	int i_ret=0;
	int fw_len = 0;

	/*get i file*/
	i_ret = fts_ft5822_get_i_file(&pbt_buf, &fw_len);
	if(i_ret < 0)
	{
		return 0;
	}
	if(NULL == pbt_buf)
	{
		FTS_COMMON_DBG("the file buff is NULL.");
		return -1;
	}
	
	if (fw_len > 2)
	{
              return pbt_buf[0x10A];	//0x010A + 0x1C00
	}	
	
	return 0x00;						/*default value */
}
int fts_ft5822_upgrade_with_bin_file(u8 *file_buff, int file_len)
{
	u8 *pbt_buf = NULL;
	int i_ret=0;
	int fw_len = 0;

	pbt_buf = file_buff;
	fw_len = file_len;

	if (fw_len < 8 || fw_len > 54 * 1024) 
	{
		FTS_COMMON_DBG("FW length error");
		return -EIO;
	}

	/*call upgrade function*/
	i_ret = fts_ic_program_ft5822_upgrade( pbt_buf, fw_len);

	#ifdef AUTO_CLB
		fts_ctpm_auto_clb();  /* start auto CLB */
	#endif
	
	return i_ret;
}
