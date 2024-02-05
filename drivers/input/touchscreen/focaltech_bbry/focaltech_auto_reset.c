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
* Created: 2016-03-18
*   
* Modify:
*
* Abstract: 
*
* Start ESD protection. Check whether need to restart TP after some time.
* But it shouldn't start ESD protection when other modules use I2C communication that avoid of some errors created by synchronization.
* 
************************************************************************/

/*******************************************************************************
* Included header files
*******************************************************************************/
#include <linux/kthread.h>
#include "focaltech_comm.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FOCALTECH_AUTO_RESET_INFO  "File Version of  focaltech_auto_reset.c:  V1.0.0 2016-03-18"

#define FTS_AUTO_RESET_EN      0//0:disable, 1:enable 

#define AUTO_RESET_WAIT_TIME     2000//ms 

 //#define HAS_ESD_PROTECT_FUNCTION      0 //0:not define, 1: defined

#define FTS_ESD_USE_REG_ADDR        0xa3
#define FTS_ESD_USE_REG_VALUE        0x54


/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/
#if 0
	static void force_reset_guitar(void);
	void gtp_esd_check_func(void);
#endif


/*******************************************************************************
* Static variables
*******************************************************************************/
static struct timeval g_last_comm_time;//the Communication time of i2c RW 	
static struct task_struct *thread_auto_reset = NULL;
static int g_thread_stop_flag = 0;//

static DECLARE_WAIT_QUEUE_HEAD(auto_reset_waiter);

static int g_start_auto_reset = 0;	//	whether can start ESD protection. 0:can't start: 1: can start.
static int g_auto_reset_use_i2c = 0;//
//static int g_auto_reset_checking = 0;//

static unsigned char g_esd_used_reg_addr = FTS_ESD_USE_REG_ADDR;//chip id address
static unsigned char g_esd_used_reg_value = FTS_ESD_USE_REG_VALUE;//chip id value

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/

/*******************************************************************************
* Static function prototypes
*******************************************************************************/
static int fts_auto_reset_thread(void *unused);
static int fts_auto_reset_check(void);

static void fts_esd_hardware_reset(void);
static void fts_esd_check_func(void);

/*******************************************************************************
* functions body
*******************************************************************************/
int fts_auto_reset_init(void)
{
	int err = 0;

	if(0 == FTS_AUTO_RESET_EN)
		return 0;
	
	FTS_COMMON_DBG("[focal] %s", FOCALTECH_AUTO_RESET_INFO);	//show version

	g_thread_stop_flag = 0;
	
	g_start_auto_reset = 1;
	g_auto_reset_use_i2c = 0;
	//g_auto_reset_checking = 0;
	
	do_gettimeofday(&g_last_comm_time);
	
	thread_auto_reset = kthread_run(fts_auto_reset_thread, 0, "focal_auto_reset");
	if (IS_ERR(thread_auto_reset))
	{
		err = PTR_ERR(thread_auto_reset);
		FTS_COMMON_DBG("failed to create kernel thread: %d.", err);
	}

	//init esd used reg
	if(fts_upgrade_info_curr.chip_id > 0x00 && fts_upgrade_info_curr.chip_id < 0xff)
		g_esd_used_reg_value = fts_upgrade_info_curr.chip_id;
	FTS_COMMON_DBG("esd used reg, addr = 0x%02x, data = 0x%02x", 
		g_esd_used_reg_addr, g_esd_used_reg_value);
	
	return 0;	
}
int fts_auto_reset_exit(void)
{
	if(0 == FTS_AUTO_RESET_EN)
		return 0;

	g_thread_stop_flag = 1;
	wake_up_interruptible(&auto_reset_waiter);
	msleep(AUTO_RESET_WAIT_TIME);
	//kthread_stop(thread_auto_reset);
	return 0;
}
static int fts_auto_reset_thread(void *unused)
{
	unsigned int iDeltaTime=0;
	unsigned long uljiffies=0;
	struct timeval tv;

	struct sched_param param = { .sched_priority = 5 };
	sched_setscheduler(current, SCHED_RR, &param); 
	uljiffies=msecs_to_jiffies(AUTO_RESET_WAIT_TIME+20);
		
	do
	{
		// set timeout of waitting
		wait_event_interruptible_timeout(auto_reset_waiter, 1==g_thread_stop_flag, uljiffies);

		if(1==g_thread_stop_flag) break;
		
		// check whether can start ESD protection
		if(0 == g_start_auto_reset)
			continue;

		// recode current time
		do_gettimeofday(&tv);
		//	get time of current time - time of other modules use I2C 
		iDeltaTime = (tv.tv_sec - g_last_comm_time.tv_sec)*MSEC_PER_SEC + (tv.tv_usec - g_last_comm_time.tv_usec)/1000;

		// test whether need to start ESD protection checking.
		if(AUTO_RESET_WAIT_TIME < iDeltaTime)
		{
			FTS_COMMON_DBG("enter fts_auto_reset_check(): iDeltaTime(ms) %d .\n", iDeltaTime);
			
			fts_auto_reset_check();								
		}
	}while(!kthread_should_stop());

	return 0;
}

/* 
*   record time of other modules use I2C
*
*/
int fts_auto_reset_record_time(void)
{
	int i = 0;

	if(0 == FTS_AUTO_RESET_EN)
		return 0;

	// return if ESD protection is use I2C
	if(1 == g_auto_reset_use_i2c)
	{	
		for(i = 0; i < 10; i++)
		{
			if(0 == g_auto_reset_use_i2c)
				break;
			msleep(2);
			FTS_COMMON_DBG("1 == g_auto_reset_use_i2c");
		}
		if(i == 10)
		{
			FTS_COMMON_DBG("Failed to read/write i2c,  the i2c communication has been accounted for ESD PROTECTION.\n");
			return -1;
		}
	}
	
	//if(0 == g_start_auto_reset)
	//	g_start_auto_reset = 1;
	
	// record time of other modules use I2C
	do_gettimeofday(&g_last_comm_time);
		
	return 0;
}

static int fts_auto_reset_check(void)
{
	g_auto_reset_use_i2c = 1;

	//FTS_COMMON_DBG("plese implement ESD PROTECTION function.\n");

	// call ESD protection check function
	fts_esd_check_func();

	g_auto_reset_use_i2c = 0;
	return 0;
}

int  fts_auto_reset_suspend(void)
{
	g_start_auto_reset = 0;
	return 0;
}

int  fts_auto_reset_resume(void)
{
	g_start_auto_reset = 1;
	return 0;
}

static void fts_esd_hardware_reset(void)
{
	FTS_COMMON_DBG("[FTS] Hardware Reset ! ");
	fts_hardware_reset(0);
	msleep(20);
	fts_hardware_reset(1);
	msleep(200);	

	return;
}
 /************************************************************************
 * Name: fts_esd_check_func
 * Brief: esd check function
 * Input: struct work_struct
 * Output: no
 * Return: 0
 ***********************************************************************/
 static void fts_esd_check_func(void)
 {
	 int i=0;
	 int ret = -1;
	 u8 uc_data=0;

	 for (i = 0; i < 3; i++) 
	 {
		 ret = fts_read_reg_for_esd(g_esd_used_reg_addr,&uc_data);
		 if (ret<0) 
		 {
			 FTS_COMMON_DBG("[Focal][Touch] read value fail.");
			 msleep(5);
		 }
		 if (ret >= 0 && g_esd_used_reg_value ==uc_data) 
		 {
			 break;
		 }
	 }
 
	 if (i >= 3) 
	 {
		 FTS_COMMON_DBG("I2C error in ESD check.  ret = %d,  Used_Reg_Addr = 0x%02x, Used_Reg_Value = 0x%02x, Get_Reg_Value = 0x%02x", 
		 	ret, g_esd_used_reg_addr, g_esd_used_reg_value, uc_data);
		 
		 goto FTS_HARDWARE_RESET;
	 }
	 else
	 {
		 FTS_COMMON_DBG("IIC communication is correct in ESD check.  ret = %d,  Used_Reg_Addr = 0x%02x, Used_Reg_Value = 0x%02x, Get_Reg_Value = 0x%02x", 
		 	ret, g_esd_used_reg_addr, g_esd_used_reg_value, uc_data);
	 }

	 return;
 
 FTS_HARDWARE_RESET:

	fts_esd_hardware_reset();
	 
	//check again
	msleep(20);
	ret = fts_read_reg_for_esd(0x00, &uc_data);
	if(ret < 0)
	{
		FTS_COMMON_DBG("[FTS] Enter Work Mode Failed! ");
		return;
	}
	else
	{

		if(0 == uc_data)
		{
			FTS_COMMON_DBG("[FTS] Enter Work Mode Succeeded! ");
			return;
		}
		else
		{
			FTS_COMMON_DBG("[FTS] Enter Work Mode Failed! ");
			return;
		}
	}	

	 return;
 }



#if 0
 /************************************************************************
 * Name: force_reset_guitar
 * Brief: reset
 * Input: no
 * Output: no
 * Return: 0
 ***********************************************************************/
 static void force_reset_guitar(void)
 {
	 s32 i;
	 s32 ret;
	 
	 mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM); //disable interrupt 
	 mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	 mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	 mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);	
	 msleep(10);
	 //TPD_DMESG("force_reset_guitar\n");
 
	 hwPowerDown(TPD_POWER_SOURCE_CUSTOM,  "TP");
	 msleep(200);
	 hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
	 //msleep(5);
 
	 msleep(10);
	 //TPD_DMESG(" fts ic reset\n");
	 mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	 mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	 mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
 
	 mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	 mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	 mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	 mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
 
	 msleep(300);
	 
	 mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);//enable interrupt
 }
 

#define A3_REG_VALUE          0x54
#define RESET_91_REGVALUE_SAMECOUNT   5
static u8 g_old_91_Reg_Value = 0x00;
static u8 g_first_read_91 = 0x01;
static u8 g_91value_same_count = 0;
 /************************************************************************
 * Name: gtp_esd_check_func
 * Brief: esd check function
 * Input: struct work_struct
 * Output: no
 * Return: 0
 ***********************************************************************/
 void gtp_esd_check_func(void)
 {
	 int i;
	 int ret = -1;
	 u8 data, data_old;
	 u8 flag_error = 0;
	 int reset_flag = 0;
	 u8 check_91_reg_flag = 0;

	 if(apk_debug_flag) 
	 {
		 //queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, esd_check_circle);
		 return;
	 }
 
	 run_check_91_register = 0;
	 for (i = 0; i < 3; i++) 
	 {
		 //ret = fts_i2c_smbus_read_i2c_block_data(i2c_client, 0xA3, 1, &data);
		 ret = fts_read_reg(i2c_client, 0xA3,&data);
		 if (ret<0) 
		 {
			 printk("[Focal][Touch] read value fail");
			 //return ret;
		 }
		 if (ret==1 && A3_REG_VALUE==data) 
		 {
			 break;
		 }
	 }
 
	 if (i >= 3) 
	 {
		 force_reset_guitar();
		 printk("focal--tpd reset. i >= 3  ret = %d  A3_Reg_Value = 0x%02x\n ", ret, data);
		 reset_flag = 1;
		 goto FOCAL_RESET_A3_REGISTER;
	 }
 
 FOCAL_RESET_INT:
 FOCAL_RESET_A3_REGISTER:
	 count_irq=0;
	 data=0;
	 //fts_i2c_smbus_write_i2c_block_data(i2c_client, 0x8F, 1, &data);
	 ret = fts_write_reg(i2c_client, 0x8F,data);
	 if (ret<0) 
	 {
		 printk("[Focal][Touch] write value fail");
		 //return ret;
	 }
	 if(0 == run_check_91_register)
	 {
		 g_91value_same_count = 0;
	 }
	 //end esd check for count 
	

	 return;
 }
#endif




