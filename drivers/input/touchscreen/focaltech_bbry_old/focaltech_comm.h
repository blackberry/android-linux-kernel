/*
 *
 * FocalTech ftxxxx TouchScreen driver.
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

#ifndef __FOCALTECH_COMM_H__
#define __FOCALTECH_COMM_H__
 /*******************************************************************************
*
* File Name: focaltech_comm.h
*
*    Author: OS Team
*
*   Created: 2015-08-19
*
*  Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
//#define CONFIG_REGULATOR //for demo compile, release version need comment
//#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#include <linux/version.h>
#include <linux/cdev.h>

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38))
#define KERNEL_ABOVE_2_6_38
#endif

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FTS_COMMON_LIB_INFO  "Common_Lib_Version  V1.0.0 2016-02-24"

#define FTS_PACKET_LENGTH							120
#define MTK_EN 0
#define INTEL_EN 1
#define FTS_REG_CHIP_ID                   			0xA3    //chip ID
#define FTS_REG_FW_VER						0xA6
#define FTS_REG_VENDOR_ID					0xA8
#define FTS_REG_POINT_RATE					0x88
#define TPD_MAX_POINTS_5                        		5

/* These are not defined. Defined to 0 to avoid the warning */
#define FT_ESD_PROTECT 						0
#define FT_TFP 							0
#define FT_TP 							0
#define FTS_DBG_EN   						1

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/

#if 0
struct fts_Upgrade_Info 
{
        u8 CHIP_ID;
        u8 TPD_MAX_POINTS;
        u8 AUTO_CLB;
	 u16 delay_aa;						/*delay of write FT_UPGRADE_AA */
	 u16 delay_55;						/*delay of write FT_UPGRADE_55 */
	 u8 upgrade_id_1;					/*upgrade id 1 */
	 u8 upgrade_id_2;					/*upgrade id 2 */
	 u16 delay_readid;					/*delay of read id */
	 u16 delay_earse_flash; 				/*delay of earse flash*/
};

#endif

/*******************************************************************************
* Static variables
*******************************************************************************/


/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
// Function Switchs: define to open,  comment to close
#define FTS_GESTRUE_EN 							0
//#define FTS_APK_DEBUG

//define and initiate in focaltech_core.c
extern struct i2c_client *fts_i2c_client;
extern struct input_dev *fts_input_dev;
//define and implement in focaltech_core.c
extern int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue);
extern int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue);
extern int fts_i2c_read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen);
extern int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen);
extern void fts_reset_tp(int HighOrLow);

//define and initiate in focaltech_flash.c
extern struct fts_Upgrade_Info fts_updateinfo_curr;
//define and implement in focaltech_flash.c
extern void fts_get_upgrade_array(void);
//extern int fts_hidi2c_to_stdi2c(struct i2c_client * client);
extern int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client *client, char *firmware_name);
extern int fts_ctpm_auto_clb(struct i2c_client *client);
extern int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client);
extern int fts_ctpm_get_i_file_ver(void);
extern int fts_ctpm_auto_upgrade(struct i2c_client *client);

//define and implement in focaltech_ctl.c
extern int fts_rw_iic_drv_init(struct i2c_client *client);
extern void  fts_rw_iic_drv_exit(void);

//define and implement in focaltech_gesture.c
#if FTS_GESTRUE_EN		
extern int fts_Gesture_init(struct input_dev *input_dev);		
extern int fts_read_Gestruedata(void);
#endif

//define and implement in focaltech_ex_fun.c
extern int fts_create_sysfs(struct i2c_client *client);// Apk and ADB functions
extern int fts_remove_sysfs(struct i2c_client *client);

extern int fts_create_apk_debug_channel(struct i2c_client * client);
extern void fts_release_apk_debug_channel(void);


//define and implement in focaltech_monitor.c
extern int fts_monitor_init(struct input_dev *dev,  int bUseProtocolB);
extern int fts_monitor_exit(void);
extern int fts_monitor_record(char *readbuf, unsigned char readlen);

#define FTS_DBG_EN   						1

/******************************************************************************** Static function prototypes*******************************************************************************/
#ifdef FTS_DBG_EN	
#define FTS_COMMON_DBG(fmt, arg...)	do {printk("[FTS] %s. line: %d.  "fmt"\n", __FUNCTION__, __LINE__, ##arg);} while (0)
//#define FTS_COMMON_DBG(fmt, arg...)	printk("[FTS] %s. line: %d.  "fmt"\n", __FUNCTION__, __LINE__, ##arg)
#else
#define FTS_COMMON_DBG(fmt, args...) do{}while(0)
//#define FTS_COMMON_DBG(fmt, arg...) (void)(0)
#endif

#define ERROR_COMMON_FTS(fmt, arg...)	printk(KERN_ERR"[FTS] ERROR. %s. line: %d.  "fmt"  !!!\n", __FUNCTION__, __LINE__, ##arg)


#endif
