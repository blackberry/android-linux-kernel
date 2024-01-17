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

#ifndef __FOCALTECH_COMM_H__
#define __FOCALTECH_COMM_H__
 /*******************************************************************************
*
* File Name: focaltech_comm.h
*
*     Author: Xu YF & ZR, Software Department, FocalTech
*
*   Created: 2016-03-16
*
*  Abstract: the implement of global function and structure variable
*
* Reference:
*
*******************************************************************************/

/* temporarily define this here until we have this porperly defined for the entire kernel */
#define CONFIG_BBRY 1

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

#include <linux/wakelock.h> // MODIFIED by Haojun Chen, 2016-12-26,BUG-3841357

#include <linux/version.h>
#include <linux/cdev.h>

#include <linux/dma-mapping.h>

#include "focaltech_flash/focaltech_flash.h"

#ifdef CONFIG_BBRY
#include <linux/regulator/consumer.h>
#endif

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FTS_COMMON_LIB_INFO  "Common_Lib_Version  V1.0.0 2016-02-24"

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38))
#define KERNEL_ABOVE_2_6_38
#endif

#ifdef KERNEL_ABOVE_2_6_38
#define sstrtoul(...) kstrtoul(__VA_ARGS__)
#else
#define sstrtoul(...) strict_strtoul(__VA_ARGS__)
#endif

//#define MTK_EN 0
//#define INTEL_EN 1

//#define FT_ESD_PROTECT 0
/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/
#ifdef CONFIG_BBRY
enum {
	FTS_STATE_ON = 0,
	FTS_STATE_GESTURE,
	FTS_STATE_OFF,
	FTS_STATE_IGNORE_TOUCH
};
#endif

// platform data that can be configured by extern
struct fts_ts_platform_data {
	u32 x_resolution_max;
	u32 y_resolution_max;
	u32 pressure_max;

	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio;
	u32 reset_gpio_flags;
#ifdef CONFIG_BBRY
	/* power rails and GPIOs shared with display */
	struct regulator *disp_1p8;
	struct regulator *disp_p5v, *disp_n5v;
	u32 disp_1p8_en_gpio;
	u32 disp_rst_n_gpio;
	/* MODIFIED-BEGIN by Haojun Chen, 2016-11-08,BUG-3403049*/
	u32 disp_p5v_en_gpio;
	u32 disp_n5v_en_gpio;
	/* MODIFIED-END by Haojun Chen,BUG-3403049*/

	bool tap_to_wake_enabled; //TODO: tie this in to the Settings App
	int state;

	struct notifier_block fb_notif;
#endif
	unsigned int fts_irq_enabled; // MODIFIED by Haojun Chen, 2016-12-08,BUG-3684946
};



/*******************************************************************************
* Static variables
*******************************************************************************/


/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
// Function Switchs: define to open,  comment to close


//define and initiate in focaltech_core.c
extern struct i2c_client *fts_i2c_client;
extern struct input_dev *fts_input_dev;
extern struct fts_ts_platform_data *fts_platform_data;
//define and implement in focaltech_core.c
extern int fts_i2c_read(unsigned char *writebuf, int writelen, unsigned char *readbuf, int readlen);
extern int fts_i2c_write(unsigned char *writebuf, int writelen);
extern int fts_read_reg(unsigned char addr, unsigned char *val);
extern int fts_write_reg(unsigned char addr, const unsigned char val);
//for esd
extern int fts_i2c_read_for_esd(unsigned char *writebuf, int writelen, unsigned char *readbuf, int readlen);
extern int fts_i2c_write_for_esd(unsigned char *writebuf, int writelen);
extern int fts_read_reg_for_esd(unsigned char addr, unsigned char *val);
extern int fts_write_reg_for_esd(unsigned char addr, const unsigned char val);

//define and implement in focaltech_i2c_communication.c
extern int fts_i2c_communication_init(void);
extern int fts_i2c_communication_exit(void);
extern int fts_i2c_read_universal(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);
extern int fts_i2c_write_universal(struct i2c_client *client, char *writebuf, int writelen);
extern int fts_i2c_read_dma(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);
extern int fts_i2c_write_dma(struct i2c_client *client, char *writebuf, int writelen);

//define and initiate in focaltech_flash.c
extern struct fts_Upgrade_Info fts_upgrade_info_curr;
//define and implement in focaltech_flash.c
extern int fts_flash_init(struct i2c_client *client);
extern int fts_flash_exit(void);
extern int fts_flash_upgrade_with_bin_file( char *firmware_name);
extern int fts_flash_upgrade_with_i_file(void);
extern int fts_flash_get_i_file_version(void);
extern int fts_ctpm_auto_clb(void);

//define and implement in focaltech_monitor.c
extern int fts_monitor_init(struct input_dev *dev,  int bUseProtocolB);
extern int fts_monitor_exit(void);
extern int fts_monitor_record(char *readbuf, unsigned char readlen);

//define and implement in focaltech_platform_data.c
extern int fts_platform_data_init(struct i2c_client *client);
extern int fts_platform_data_exit(void);
extern int fts_hardware_reset(bool level);
extern void fts_wakeup_gesture_set(bool enabled);
extern bool fts_wakeup_gesture_read(void);
#ifdef CONFIG_BBRY
extern void enable_regulator(struct device *dev, struct regulator *vreg, const char* vreg_name, bool enable);
#endif

//define and implement in focaltech_report.c
extern int fts_report_init(struct i2c_client *client, void *platform_data);
extern int fts_report_exit(struct i2c_client *client, void *platform_data);
extern int fts_report_resume(void);
extern int fts_report_suspend(void);

//define and implement in focaltech_apk_node.c
extern int fts_apk_node_init(void);
extern int fts_apk_node_exit(void);

//define and implement in focaltech_sysfs.c
extern int fts_sysfs_init(struct i2c_client *client);
extern int fts_sysfs_exit(struct i2c_client *client);

//define and implement in focaltech_gesture.c
extern int fts_gesture_init(struct input_dev *input_dev);
extern int fts_gesture_exit(void);
extern int fts_gesture_handle(void);

//define and implement in focaltech_proximity.c
extern int fts_touch_proximity_init(void);
extern int fts_touch_proximity_exit(void);

//define and implement in focaltech_auto_reset.c
extern int fts_auto_reset_init(void);
extern int fts_auto_reset_exit(void);
extern int fts_auto_reset_record_time(void);
extern int  fts_auto_reset_suspend(void);
extern int  fts_auto_reset_resume(void);

//define and implement in focaltech_test.c
extern int fts_test_init(struct i2c_client *client);
extern int fts_test_exit(struct i2c_client *client);

extern void fts_irq_enable(bool enable); // MODIFIED by Haojun Chen, 2016-12-08,BUG-3684946

/************************************ Static function prototypes ************************************/
#define FTS_COMMON_DBG_EN
#ifdef FTS_COMMON_DBG_EN
#define FTS_COMMON_DBG(fmt, args...)	do {printk("[FTS] %s. line: %d.  "fmt"\n", __FUNCTION__, __LINE__, ##args);} while (0)
//#define FTS_COMMON_DBG(fmt, arg...)	printk("[FTS] %s. line: %d.  "fmt"\n", __FUNCTION__, __LINE__, ##arg)
#else
#define FTS_COMMON_DBG(fmt, args...) do{}while(0)
//#define FTS_COMMON_DBG(fmt, arg...) (void)(0)
#endif

#ifdef CONFIG_BBRY
#define INFO_COMMON_FTS(fmt, args...)	do {pr_info("[FTS] %s. "fmt"\n", __FUNCTION__, ##args);} while (0)
#endif
#define ERROR_COMMON_FTS(fmt, arg...)	printk(KERN_ERR"[FTS] ERROR. %s. line: %d.  "fmt"  !!!\n", __FUNCTION__, __LINE__, ##arg)


#endif
