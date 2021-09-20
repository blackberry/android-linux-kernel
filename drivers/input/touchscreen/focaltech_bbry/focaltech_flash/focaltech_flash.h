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

#ifndef __FOCALTECH_FLASH_H__
#define __FOCALTECH_FLASH_H__
 /*******************************************************************************
*
* File Name: focaltech_flash.h
*
*     Author: Xu YF & ZR, Software Department, FocalTech
*
*   Created: 2016-03-16
*
*  Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/

#include "../focaltech_global/focaltech_global.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/

/*for .i file*/
#define USE_I_FILES_NUMBER              1//Support wendor number 
#define USE_I_FILE_1_BY_WENDOR_ID_1        0x55//must change to actual value 
#define USE_I_FILE_2_BY_WENDOR_ID_2        0x02//must change to actual value 
#define USE_I_FILE_3_BY_WENDOR_ID_3        0x03//must change to actual value 

/*common global define of all ic*/
#define FTS_PACKET_LENGTH              32 //120

#define FTS_RST_CMD_REG1              0xFC
#define FTS_READ_ID_REG              0x90
#define FTS_ERASE_APP_REG              0x61
#define FTS_ERASE_PARAMS_CMD            0x63
#define FTS_FW_WRITE_CMD              0xBF
#define FTS_REG_RESET_FW              0x07
#define FTS_REG_ECC                  0xCC
#define FTS_RST_CMD_REG2              0xBC

#ifndef ERROR_CODE_OK
  #define ERROR_CODE_OK               0x00
#endif
#define FTS_UPGRADE_LOOP              30
#define FTS_UPGRADE_AA                0xAA
#define FTS_UPGRADE_55                0x55

#define FTS_REG_CHIP_ID                             0xA3    //chip ID
#define FTS_REG_CHIP_ID2                             0x9F    //chip ID2
#define FTS_REG_FW_VER                0xA6

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/
struct fts_Upgrade_Info 
{
        u8 chip_id;							/*from reg 0xa8*/
	 u8 chip_id2;							/*from reg 0x9f*/
        u8 TPD_MAX_POINTS;
        u8 AUTO_CLB;
	 u16 delay_aa;						/*delay of write FT_UPGRADE_AA */
	 u16 delay_55;						/*delay of write FT_UPGRADE_55 */
	 u8 upgrade_id_1;					/*upgrade id 1 */
	 u8 upgrade_id_2;					/*upgrade id 2 */
	 u16 delay_readid;					/*delay of read id */
	 u16 delay_erase_flash; 				/*delay of earse flash*/
};

/*******************************************************************************
* Global variable 
*******************************************************************************/

/*for buffer of .i file*/
#define FILE_LEN(arr)   (sizeof(arr)/sizeof(arr[0]))

extern unsigned char BUFFER_PRAM_BOOT_FILE[];
extern unsigned char BUFFER_I_FILE_1[];
extern unsigned char BUFFER_I_FILE_2[];
extern unsigned char BUFFER_I_FILE_3[];

extern unsigned int g_len_pram_boot_file;
extern unsigned int g_len_i_file_1;
extern unsigned int g_len_i_file_2;
extern unsigned int g_len_i_file_3;

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
extern int fts_protocol_windows_to_android(void);

extern int fts_upgrade_read_vendor_id( u8 *ucPVendorID);
extern bool fts_upgrade_read_pram(unsigned int Addr, unsigned char * pData, unsigned short Datalen);

extern int fts_upgrade_hardware_reset(int delay_time, unsigned char chip_id_1, unsigned char chip_id_2);
extern void fts_enter_work_mode(void);
//extern int fts_flash_read_project_code( char *pProjectCode);
/*export function for External call*/


/* export function of ft5x06*/
extern int fts_ft5x06_upgrade_with_bin_file(u8 *file_buff, int file_len);
extern int fts_ft5x06_upgrade_with_i_file(void);
extern int fts_ft5x06_get_i_file_version(void);

/* export function of ft5x36*/
extern int fts_ft5x36_upgrade_with_bin_file(u8 *file_buff, int file_len);
extern int fts_ft5x36_upgrade_with_i_file(void);
extern int fts_ft5x36_get_i_file_version(void);

/* export function of ft6x06*/
extern int fts_ft6x06_upgrade_with_bin_file(u8 *file_buff, int file_len);
extern int fts_ft6x06_upgrade_with_i_file(void);
extern int fts_ft6x06_get_i_file_version(void);

/* export function of ft6x36*/
extern int fts_ft6x36_upgrade_with_bin_file(u8 *file_buff, int file_len);
extern int fts_ft6x36_upgrade_with_i_file(void);
extern int fts_ft6x36_get_i_file_version(void);

/* export function of ft5x46*/
extern int fts_ft5x46_upgrade_with_bin_file(u8 *file_buff, int file_len);
extern int fts_ft5x46_upgrade_with_i_file(void);
extern int fts_ft5x46_get_i_file_version(void);

/* export function of ft5822*/
extern int fts_ft5822_upgrade_with_bin_file(u8 *file_buff, int file_len);
extern int fts_ft5822_upgrade_with_i_file(void);
extern int fts_ft5822_get_i_file_version(void);

/* export function of ft8606*/
extern int fts_ft8606_upgrade_with_bin_file(u8 *file_buff, int file_len);
extern int fts_ft8606_upgrade_with_i_file(void);
extern int fts_ft8606_get_i_file_version(void);

/* export function of ft8607*/
extern int fts_ft8607_upgrade_with_bin_file(u8 *file_buff, int file_len);
extern int fts_ft8607_upgrade_with_i_file(void);
extern int fts_ft8607_get_i_file_version(void);

/* export function of ft8707*/
extern int fts_ft8707_upgrade_with_bin_file(u8 *file_buff, int file_len);
extern int fts_ft8707_upgrade_with_i_file(void);
extern int fts_ft8707_get_i_file_version(void);

/* export function of ft8716*/
extern int fts_ft8716_upgrade_with_bin_file(u8 *file_buff, int file_len);
extern int fts_ft8716_upgrade_with_i_file(void);
extern int fts_ft8716_get_i_file_version(void);

/* export function of ft8736*/
extern int fts_ft8736_upgrade_with_bin_file(u8 *file_buff, int file_len);
extern int fts_ft8736_upgrade_with_i_file(void);
extern int fts_ft8736_get_i_file_version(void);

/* export function of ftE716*/
extern int fts_ftE716_upgrade_with_bin_file(u8 *file_buff, int file_len);
extern int fts_ftE716_upgrade_with_i_file(void);
extern int fts_ftE716_get_i_file_version(void);

/* export function of ftE736*/
extern int fts_ftE736_upgrade_with_bin_file(u8 *file_buff, int file_len);
extern int fts_ftE736_upgrade_with_i_file(void);
extern int fts_ftE736_get_i_file_version(void);

#endif

