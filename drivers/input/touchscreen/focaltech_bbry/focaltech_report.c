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
* File Name: focaltech_report.c
*
*  Author: Xu YF & ZR, Software Department, FocalTech
*
* Created: 2016-02-17
*
* Modified:
*
*  Abstract:
*
* report touch data to input sub system
*
* Reference:
*
*******************************************************************************/
/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include <linux/kthread.h>
#include "focaltech_comm.h"
#include "focaltech_core.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
/*File Version*/
#define FOCALTECH_REPORT_INFO  "File Version of  focaltech_report.c:  V1.0.0 2016-03-09"

// report protocol: A or B
#define REPORT_PROTOCOL_OF_INPUT_DEVICE    1//0:A protocol, 1:B protocol
enum enum_Report_Protocol_Type
{
	A_PROTOCOL = 0,
	B_PROTOCOL = 1,
};

// mode of queue handle touch event
#define QUEUE_MODE_OF_TOUCH_HANDLE      0//0: wait queue, 1: work queue
enum enum_Queue_Mode
{
	WAIT_QUEUE = 0,
	WORK_QUEUE = 1,
};

/*REPORT_PRESSURE*/
#define FTS_REPORT_PRESSURE_EN      0//0:disable, 1:enable

/*Force Touch*/
#define FTS_FORCE_TOUCH_EN      0//0:disable, 1:enable

/*Debug Enable*/
#define REPORT_TOUCH_DEBUG_EN        0 //0:disable, 1:enable

/* for touch data*/
#define FTS_MAX_TOUCH_POINTS        10

#define FTS_MAX_POINT_ID          0x0F
#define EVERY_POINT_LEN          6
#define START_ADDR_OF_TOUCH_DATA      0x00 //the start addr of get touch data
#define ADDR_X_H_POS            (3 - START_ADDR_OF_TOUCH_DATA)
#define ADDR_X_L_POS            (4 - START_ADDR_OF_TOUCH_DATA)
#define ADDR_Y_H_POS            (5 - START_ADDR_OF_TOUCH_DATA)
#define ADDR_Y_L_POS            (6 - START_ADDR_OF_TOUCH_DATA)
#define ADDR_EVENT_POS            (3 - START_ADDR_OF_TOUCH_DATA)
#define ADDR_ID_POS              (5 - START_ADDR_OF_TOUCH_DATA)
#define ADDR_POINT_NUM            (2 - START_ADDR_OF_TOUCH_DATA)
#define ADDR_XY_POS            (7 - START_ADDR_OF_TOUCH_DATA)
#define ADDR_MISC              (8 - START_ADDR_OF_TOUCH_DATA)
#define FTS_TOUCH_DATA_LEN            (3 - START_ADDR_OF_TOUCH_DATA + EVERY_POINT_LEN * FTS_MAX_TOUCH_POINTS)

/*for key*/
#define FTS_KEY_EN            0//0:disable, 1:enable
#define FTS_KEY_1               KEY_BACK//From USB HUT 1.12
#define FTS_KEY_2               KEY_HOMEPAGE//From USB HUT 1.12
#define FTS_KEY_3               KEY_APPSELECT//From USB HUT 1.12
#define FTS_KEY_1_X_POS           180// set actual value
#define FTS_KEY_2_X_POS           360// set actual value
#define FTS_KEY_3_X_POS           540// set actual value
#define FTS_KEY_ALL_Y_POS           1660// set actual value

#define DOWN_EVENT              0
#define UP_EVENT              1
#define CONTACT_EVENT            2
#define NO_EVENT              3


#if defined(CONFIG_TCT_SDM660_COMMON)
/* Scale the keys x pos to the finale driver's scale */
#define BBRY_SCALE_KEY_X_POS(x,fw_maxx,drv_maxx)   (((x) * (drv_maxx)) / (fw_maxx))
#endif

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/
/*touch event info*/
struct ts_event
{
	u16 au16_x[FTS_MAX_TOUCH_POINTS];				/* x coordinate */
	u16 au16_y[FTS_MAX_TOUCH_POINTS];				/* y coordinate */
	u8 au8_touch_event[FTS_MAX_TOUCH_POINTS];		/* touch event: 0 -- down; 1-- up; 2 -- contact */
	u8 au8_finger_id[FTS_MAX_TOUCH_POINTS];			/* touch ID */
	u16 pressure[FTS_MAX_TOUCH_POINTS];
	u16 area[FTS_MAX_TOUCH_POINTS];
	u8 touch_point;
	int touchs;
	u8 touch_point_num;
};
/*******************************************************************************
* Static variables
*******************************************************************************/
// declare work queue
static struct workqueue_struct *g_workqueue=NULL;
static struct work_struct   g_work_touch_event;

// declare wait queue
static int g_waitqueue_flag = 0;
static DECLARE_WAIT_QUEUE_HEAD(g_waitqueue_touch_event);
static struct task_struct *g_tsk_touch_event;

// wherher enter suspend mode
static int g_is_suspend_mode = 0;//0: no, 1: yes

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
//	declare input device
struct input_dev *g_input_dev = NULL;
struct input_dev *fts_input_dev = NULL;

static int fts_irq_init(struct i2c_client *client, void *platform_data);
static int fts_irq_exit(struct i2c_client *client, void *platform_data);
static irqreturn_t fts_irq_handle(int irq, void *dev_id);
static int fts_input_dev_init(struct i2c_client *client, void *platform_data);
static int fts_input_dev_exit(void);
static int fts_input_dev_A_init(void *platform_data);
static int fts_input_dev_B_init(void *platform_data);
static int fts_input_dev_report(struct ts_event *data);
static int fts_input_dev_report_A(struct ts_event *data);
static int fts_input_dev_report_B(struct ts_event *data);
static int fts_input_dev_report_key_event(struct ts_event *data);
static int fts_work_queue_init(void);
static int fts_work_queue_exit(void);
static int fts_work_queue_handle(int irq, void *dev_id);
static void fts_work_queue_touch_event(struct work_struct *work);
static int fts_wait_queue_init(void);
static int fts_wait_queue_exit(void);
static int fts_wait_queue_handle(void);
static int fts_wait_queue_touch_event(void *unused);
static int fts_read_touch_data(struct ts_event *data);

#ifdef CONFIG_CKB_MASK_KEY
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_BBRY
extern bool btn_mask_on;
static bool btn_mask_hold = false;
#endif

static struct workqueue_struct *fts_btn_wq = NULL;
static struct delayed_work nav_key_report_work;

extern int get_stmpe_keypad_status(void);
static int nav_key_position_x;
static int nav_key_position_y;
static int nav_key_position_id;
extern char nav_key_is_reporting;
extern char nav_key_need_report;
static int nav_key_is_pressed;

static void fts_nav_key_report_work(struct work_struct *work)
{
	if (nav_key_need_report) {
		printk("[haojun] %s report nav key down x = %d, y = %d key_num = %d \n", 
				__func__, nav_key_position_x, nav_key_position_y, nav_key_position_id);
		input_mt_slot(g_input_dev, nav_key_position_id);
		input_report_abs(g_input_dev, ABS_MT_PRESSURE,0x3f/*data->pressure[i]*/);
		input_report_abs(g_input_dev, ABS_MT_TOUCH_MAJOR,0x3f/*data->area[i]*/);
		input_report_abs(g_input_dev, ABS_MT_POSITION_X,nav_key_position_x);
		input_report_abs(g_input_dev, ABS_MT_POSITION_Y,nav_key_position_y);
		input_report_key(g_input_dev, BTN_TOUCH, 1);
		input_sync(g_input_dev);
	} else {
		printk("[haojun] %s stmpe_keypad pressed, ignore nav key\n", __func__);
	}
	nav_key_is_reporting = 0;
}

static void fts_nav_key_report_down_up(void)
{
	printk("%s \n", __func__);
	input_mt_slot(g_input_dev, nav_key_position_id);
	input_report_abs(g_input_dev, ABS_MT_PRESSURE,0x3f/*data->pressure[i]*/);
	input_report_abs(g_input_dev, ABS_MT_TOUCH_MAJOR,0x3f/*data->area[i]*/);
	input_report_abs(g_input_dev, ABS_MT_POSITION_X,nav_key_position_x);
	input_report_abs(g_input_dev, ABS_MT_POSITION_Y,nav_key_position_y);
	input_report_key(g_input_dev, BTN_TOUCH, 1);
	input_sync(g_input_dev);
	msleep(20);
	input_mt_slot(g_input_dev, nav_key_position_id);
	input_mt_report_slot_state(g_input_dev, MT_TOOL_FINGER,false);
	input_report_abs(g_input_dev, ABS_MT_PRESSURE, 0);
	input_report_key(g_input_dev, BTN_TOUCH, 0);
	input_sync(g_input_dev);
}
static int btn_mask_function_init(void)
{
	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);

	fts_btn_wq = create_workqueue("FT_Btn_Workqueue");
	if (!fts_btn_wq)
		return -1;

	INIT_DELAYED_WORK(&nav_key_report_work, fts_nav_key_report_work);

	return 0;
}
#endif

/*******************************************************************************
* functions body
*******************************************************************************/

int fts_report_init(struct i2c_client *client, void *platform_data)
{
	int err = 0;

	FTS_COMMON_DBG("[focal] %s ",  FOCALTECH_REPORT_INFO);	//show version
	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);

	//Init IRQ
	err = fts_irq_init(client, platform_data);
	if(err < 0)
		return err;

	//Init Input Device
	err = fts_input_dev_init(client, platform_data);
	if(err < 0)
		return err;

	if(QUEUE_MODE_OF_TOUCH_HANDLE == WAIT_QUEUE)
	{
		//Init wait queue for touch event
		err = fts_wait_queue_init();
		if(err < 0)
			return err;
	}
	else
	{
		//Init work queue
		err = fts_work_queue_init();
		if(err < 0)
			return err;
	}

	//Init gesture
	err = fts_gesture_init(g_input_dev);
	if(err < 0)
		return err;
#ifdef CONFIG_CKB_MASK_KEY
	err = btn_mask_function_init();
	if (err<0)
		return err;
#endif
	//init suspend mode
	g_is_suspend_mode = 0;

	return 0;
}
int fts_report_exit(struct i2c_client *client, void *platform_data)
{
	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);

	//exit IRQ
	fts_irq_exit(client, platform_data);

	//exit Input Device
	fts_input_dev_exit();

	if(QUEUE_MODE_OF_TOUCH_HANDLE == WAIT_QUEUE)
	{
		//exit wait queue for touch event
		fts_wait_queue_exit();
	}
	else
	{
		//exit work queue
		fts_work_queue_exit();
	}

	//exit gesture
	fts_gesture_exit();

	return 0;
}
int fts_report_resume(void)
{
	g_is_suspend_mode = 0;
	return 0;
}

int fts_report_suspend(void)
{
	g_is_suspend_mode = 1;
	return 0;
}
static int fts_irq_init(struct i2c_client *client, void *platform_data)
{
	int err = 0;
	struct fts_ts_platform_data *data;

	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);

	data = platform_data;

	err = request_threaded_irq(client->irq, NULL, fts_irq_handle,
				IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
				client->dev.driver->name, data);
	if (err<0) {
		dev_err(&client->dev, "request irq failed");
		FTS_COMMON_DBG("failed to request irq.");
		return -1;
	}
	else
	{
		FTS_COMMON_DBG("successful in requesting irq. client->irq = %d ", client->irq);
	}

	return 0;
}
static int fts_irq_exit(struct i2c_client *client, void *platform_data)
{
	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);

	free_irq(client->irq, platform_data);

	return 0;
}

extern bool fts_ignore_irq;

static irqreturn_t fts_irq_handle(int irq, void *dev_id)
{
	//FTS_COMMON_DBG("ALON [focal] Enter %s ",  __func__);

	if (fts_ignore_irq)
		return IRQ_HANDLED;

	if(QUEUE_MODE_OF_TOUCH_HANDLE == WAIT_QUEUE)
	{
		fts_wait_queue_handle();
	}
	else
	{
		fts_work_queue_handle(irq, dev_id);
	}

	return IRQ_HANDLED;
}

static int fts_input_dev_init(struct i2c_client *client, void *platform_data)
{
	int ret=0;
	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);

	if(REPORT_PROTOCOL_OF_INPUT_DEVICE == A_PROTOCOL)
	{
		ret=fts_input_dev_A_init(platform_data);
	}
	else
	{
		ret=fts_input_dev_B_init(platform_data);
	}
	if(ret < 0)
		return ret;

	/*init monitor module*/
	if(REPORT_PROTOCOL_OF_INPUT_DEVICE == A_PROTOCOL)
	{
		ret=fts_monitor_init(g_input_dev, A_PROTOCOL);
	}
	else
	{
		ret=fts_monitor_init(g_input_dev, B_PROTOCOL);
	}
	return ret;

}
static int fts_input_dev_exit(void)
{
	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);

	/*must add before input_unregister_device*/
	fts_monitor_exit();

	input_unregister_device(g_input_dev);

	return 0;
}
static int fts_input_dev_A_init(void *platform_data)
{
	int ret=0;
	struct fts_ts_platform_data *data;

	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);

	data = platform_data;
	// allocate a input_dev struct
	g_input_dev = input_allocate_device();
	if (!g_input_dev) {
		ret = -ENOMEM;
		printk("failed to allocate input device");
		goto Exit;
	}
	fts_input_dev = g_input_dev;
	// set device property
	g_input_dev->name = "fts_input_device_A";
	g_input_dev->id.bustype = BUS_I2C;
	//g_input_dev->id.vendor = 0x12FA;
	//g_input_dev->id.product = 0x2143;
	//g_input_dev->id.version = 0x0100;

	/* 2. set */
   	 /* 2.1 set event of input device.*/
	set_bit(EV_SYN, g_input_dev->evbit);
	set_bit(EV_ABS, g_input_dev->evbit);
	set_bit(EV_KEY, g_input_dev->evbit);

	/* 2.2 set property	 */

	/*	  set INPUT_PROP_DIRECT */
	set_bit(INPUT_PROP_DIRECT, g_input_dev->propbit);
	/* set  BTN_TOUCH */
	set_bit(BTN_TOUCH, g_input_dev->keybit);

	/*for VA area*/
	input_set_abs_params(g_input_dev, ABS_MT_POSITION_X, 0, data->x_resolution_max, 0, 0);
	input_set_abs_params(g_input_dev, ABS_MT_POSITION_Y, 0, data->y_resolution_max, 0, 0);
	input_set_abs_params(g_input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	if(FTS_REPORT_PRESSURE_EN)
		input_set_abs_params(g_input_dev, ABS_MT_PRESSURE, 0, data->pressure_max, 0, 0);

	input_set_abs_params(g_input_dev, ABS_MT_TRACKING_ID, 0, FTS_MAX_TOUCH_POINTS, 0, 0);

	/*for Key*/

	if(1 == FTS_KEY_EN)
	{
		set_bit(FTS_KEY_1, g_input_dev->keybit);
		set_bit(FTS_KEY_2, g_input_dev->keybit);
		set_bit(FTS_KEY_3, g_input_dev->keybit);
	}

	/* 3. register */
	ret = input_register_device(g_input_dev);
	if (ret) {
		input_free_device(g_input_dev);
		printk("failed to register input device %s, %d",  g_input_dev->name, ret);
		goto Exit;
	}
	else
	{
		printk("successful in registering input device, name:  %s",  g_input_dev->name);
	}

	Exit:

	return ret;
}

static int fts_input_dev_B_init(void *platform_data)
{
	int ret=0;
	struct fts_ts_platform_data *data;

	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);

	data = platform_data;

	/* 1. allocate a input_dev struct */
	g_input_dev = input_allocate_device();
	if (!g_input_dev) {
		ret = -ENOMEM;
		printk("failed to allocate input device");
		goto Exit;
	}
	fts_input_dev = g_input_dev;

	/* set device property */
	g_input_dev->name = "fts_input_device_B";
	g_input_dev->id.bustype = BUS_I2C;
	//g_input_dev->id.vendor = 0x12FA;
	//g_input_dev->id.product = 0x2143;
	//g_input_dev->id.version = 0x0100;

	/* 2. set */
   	 /* 2.1 set event property  */
	set_bit(EV_SYN, g_input_dev->evbit);
	set_bit(EV_ABS, g_input_dev->evbit);
	set_bit(EV_KEY, g_input_dev->evbit);

	/* 2.2 set*/

	set_bit(INPUT_PROP_DIRECT, g_input_dev->propbit);

	set_bit(BTN_TOUCH, g_input_dev->keybit);

	/*for VA area*/

	/*	init slots of multi touch.*/
	#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0))
		input_mt_init_slots(g_input_dev, FTS_MAX_TOUCH_POINTS);
	#else
		input_mt_init_slots(g_input_dev, FTS_MAX_TOUCH_POINTS, 2);
	#endif

	input_set_abs_params(g_input_dev, ABS_MT_POSITION_X, 0, data->x_resolution_max, 0, 0);
	input_set_abs_params(g_input_dev, ABS_MT_POSITION_Y, 0, data->y_resolution_max, 0, 0);
	input_set_abs_params(g_input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	if(FTS_REPORT_PRESSURE_EN)
		input_set_abs_params(g_input_dev, ABS_MT_PRESSURE, 0, data->pressure_max, 0, 0);

	/*for Key*/
	if(1 == FTS_KEY_EN)
	{
		set_bit(FTS_KEY_1, g_input_dev->keybit);
		set_bit(FTS_KEY_2, g_input_dev->keybit);
		set_bit(FTS_KEY_3, g_input_dev->keybit);
	}

	/* 3. register  */
	ret = input_register_device(g_input_dev);
	if (ret) {
		input_free_device(g_input_dev);
		printk("failed to register input device %s, %d",  g_input_dev->name, ret);
		goto Exit;
	}
	else
	{
		printk("successful in registering input device, name:  %s",  g_input_dev->name);
	}

	Exit:

	return ret;
}

 /************************************************************************
* Name: fts_input_dev_report_A
* Brief: report the point information
* Input: event info
* Output: no
* Return: success is zero
***********************************************************************/
static int fts_input_dev_report(struct ts_event *data)
 {

	if(REPORT_PROTOCOL_OF_INPUT_DEVICE == A_PROTOCOL)
	{
		fts_input_dev_report_A(data);
	}
	else
	{
		fts_input_dev_report_B(data);
	}
 	return 0;
 }

 /************************************************************************
* Name: fts_input_dev_report_A
* Brief: report the point information
* Input: event info
* Output: no
* Return: success is zero
***********************************************************************/
static int fts_input_dev_report_A(struct ts_event *data)
 {
	int i = 0;
	int up_point = 0;
 	int touchs = 0;
	static int last_touchs_a = 0;
	//int touchs_count = 0;

	/*is Key Event?*/
	if(0 == fts_input_dev_report_key_event(data))
	 	return 0;

	for (i = 0; i < data->touch_point; i++)
	{
		if (data->au8_touch_event[i]== DOWN_EVENT
			||data->au8_touch_event[i] == CONTACT_EVENT )//down or contact
		{
			 input_report_abs(g_input_dev, ABS_MT_TRACKING_ID, data->au8_finger_id[i]);
			 if(FTS_REPORT_PRESSURE_EN)
			 {
			 	if(1 == FTS_FORCE_TOUCH_EN)
					 input_report_abs(g_input_dev, ABS_MT_PRESSURE, data->pressure[i]);
			 	else
			 		input_report_abs(g_input_dev, ABS_MT_PRESSURE,0x3f/*data->pressure[i]*/);
			 }
			 input_report_abs(g_input_dev, ABS_MT_TOUCH_MAJOR,0x3f/*data->area[i]*/);
			 input_report_abs(g_input_dev, ABS_MT_POSITION_X,data->au16_x[i]);
			 input_report_abs(g_input_dev, ABS_MT_POSITION_Y,data->au16_y[i]);

			 input_mt_sync(g_input_dev);

			 touchs |= BIT(data->au8_finger_id[i]);
   			 last_touchs_a |= BIT(data->au8_finger_id[i]);
			 //#if REPORT_TOUCH_DEBUG_EN
			//	printk("\n [fts] touch down (id=%d ,x=%d, y=%d, pres=%d, area=%d) ",  data->au8_finger_id[i],data->au16_x[i],data->au16_y[i],data->pressure[i],data->area[i]);
			 //#endif
			 //
		}
		else if (data->au8_touch_event[i]== UP_EVENT)//up
		{
			 up_point++;
			 last_touchs_a &= ~BIT(data->au8_finger_id[i]);
		}
		else
		{
			printk("\n [fts] no point event,  id=%d ",  data->au8_finger_id[i]);
		}

	}

	if(last_touchs_a ^ touchs){//if last_touchs != touchs, lost some up event, so add up event by manual.
		for(i = 0; i < FTS_MAX_TOUCH_POINTS; i++){//check if lost up frame
			if(BIT(i) & (last_touchs_a ^ touchs)){

				//#if REPORT_TOUCH_DEBUG_EN
					printk("\n [fts] touch manual up,  id=%d ",  i);
				//#endif
				last_touchs_a &= ~BIT(i);
			}
		}
	}

	#if REPORT_TOUCH_DEBUG_EN
		printk("[fts] touch 1 current-touchs=%d, last-touchs=%d, touch_point=%d, up_point=%d \n",
			touchs,last_touchs_a,data->touch_point, up_point);
	#endif
	last_touchs_a = touchs;

	if(data->touch_point == up_point)
		{
		 input_report_key(g_input_dev, BTN_TOUCH, 0);
		 //printk("\n [fts] touch normal_up all.");
		 input_mt_sync(g_input_dev);
		}
	else
		 input_report_key(g_input_dev, BTN_TOUCH, 1);

	input_sync(g_input_dev);

	return 0;
 }



 /************************************************************************
* Name: fts_input_dev_report_B
* Brief: report the point information
* Input: event info
* Output: no
* Return: success is zero
***********************************************************************/
static int fts_input_dev_report_B(struct ts_event *data)
 {
	int i = 0;
	int up_point = 0;
 	int touchs = 0;
	static int last_touchs = 0;
	//int touchs_count = 0;

	/*is Key Event?*/
	if(0 == fts_input_dev_report_key_event(data))
	 	return 0;

	for (i = 0; i < data->touch_point; i++)
	{
		 input_mt_slot(g_input_dev, data->au8_finger_id[i]);

		if ((data->au8_touch_event[i]== DOWN_EVENT)
			||(data->au8_touch_event[i] == CONTACT_EVENT))//down or contact
		{
			 input_mt_report_slot_state(g_input_dev, MT_TOOL_FINGER,true);
			 if(FTS_REPORT_PRESSURE_EN)
			 {
			 	if(1 == FTS_FORCE_TOUCH_EN)
				 	input_report_abs(g_input_dev, ABS_MT_PRESSURE, data->pressure[i]);
			 	else
			 		input_report_abs(g_input_dev, ABS_MT_PRESSURE,0x3f/*data->pressure[i]*/);
			 }
			 input_report_abs(g_input_dev, ABS_MT_TOUCH_MAJOR,0x3f/*data->area[i]*/);
			 input_report_abs(g_input_dev, ABS_MT_POSITION_X,data->au16_x[i]);
			 input_report_abs(g_input_dev, ABS_MT_POSITION_Y,data->au16_y[i]);
			 touchs |= BIT(data->au8_finger_id[i]);
   			 last_touchs |= BIT(data->au8_finger_id[i]);
			 #if REPORT_TOUCH_DEBUG_EN
				printk("\n [fts] touch down (id=%d ,x=%d, y=%d, pres=%d, area=%d) ",  
					data->au8_finger_id[i],data->au16_x[i],data->au16_y[i],data->pressure[i],data->area[i]);
			 #endif
		}
		else  if (data->au8_touch_event[i]== UP_EVENT)//up
		{
			 up_point++;
			 input_mt_report_slot_state(g_input_dev, MT_TOOL_FINGER,false);
			 if(!FTS_REPORT_PRESSURE_EN)
			 	input_report_abs(g_input_dev, ABS_MT_PRESSURE, 0);
			 last_touchs &= ~BIT(data->au8_finger_id[i]);
		}
		else
		{
			printk("\n [fts] no point event,  id=%d ",  data->au8_finger_id[i]);
		}

	}

	//if last_touchs != touchs, lost some up event, so add up event by manual.
	if(last_touchs ^ touchs)
	{
		for(i = 0; i < FTS_MAX_TOUCH_POINTS; i++)//check if lost up frame
		{
			if(BIT(i) & (last_touchs ^ touchs))
			{
				//#if REPORT_TOUCH_DEBUG_EN
					printk("\n [fts] touch manual up,  id=%d ",  i);
				//#endif
				last_touchs &= ~BIT(i);
				input_mt_slot(g_input_dev, i);
				input_mt_report_slot_state(g_input_dev, MT_TOOL_FINGER, false);
			}
		}
	}

	#if REPORT_TOUCH_DEBUG_EN
		printk("[fts] touch 1 current-touchs=%d, last-touchs=%d, touch_point=%d, up_point=%d \n ",
			touchs,last_touchs,data->touch_point, up_point);
	#endif
	last_touchs = touchs;

	if(data->touch_point == up_point)
		{
		 input_report_key(g_input_dev, BTN_TOUCH, 0);
		 //printk("\n [fts] touch normal_up all.");
		}
	else
		 input_report_key(g_input_dev, BTN_TOUCH, 1);

	input_sync(g_input_dev);

	return 0;
 }


/* MODIFIED-BEGIN by Haojun Chen, 2016-12-06,BUG-3668081*/
/* release all touches */
void fts_release_all_touch(void)
{
	int i;

	for (i = 0; i < FTS_MAX_TOUCH_POINTS; i++) {
		input_mt_slot(g_input_dev, i);
		input_mt_report_slot_state(g_input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(g_input_dev, false);
	input_sync(g_input_dev);
}
/* MODIFIED-END by Haojun Chen,BUG-3668081*/

  /************************************************************************
* Name: fts_input_dev_report_key_event
* Brief: report key event
* Input: event info
* Output: no
* Return: 0: is key event, -1: isn't key event
***********************************************************************/
static int fts_input_dev_report_key_event(struct ts_event *data)
 {
	int i = 0;
	int bUsedKeyEvent = 1;//0:not used, 1: used

#if !defined(CONFIG_TCT_SDM660_COMMON)
	/*is enable?*/
	if(1 != FTS_KEY_EN)
		return -1;
#endif

	/* because touch point number is not 1, so it's not key event.  */
	if(1 != data->touch_point)
		return -1;

	/* check whether data's y value equal one of all key's y values.  */
	for (i = 0; i < FTS_MAX_TOUCH_POINTS; i++)
	{
#if defined(CONFIG_TCT_SDM660_COMMON)
		if (FTS_KEY_ALL_Y_POS == data->au16_y[i]) {
			data->au16_x[i] = BBRY_SCALE_KEY_X_POS(data->au16_x[i], 720, 1080);
			break;
		}
#else
		if (FTS_KEY_ALL_Y_POS == data->au16_y[i])
			break;
#endif
 	}
	if(FTS_MAX_TOUCH_POINTS == i)
		return -1;

#ifdef CONFIG_CKB_MASK_KEY
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_BBRY
	if (btn_mask_on || btn_mask_hold) {
		if (data->au8_touch_event[i]== DOWN_EVENT || data->au8_touch_event[i] == CONTACT_EVENT)
			btn_mask_hold = true;
		else
			btn_mask_hold = false;
		if (FTS_KEY_ALL_Y_POS == data->au16_y[i]) {
			FTS_COMMON_DBG("CKB swipe occured, ignore Navigation button");
			return 0;
		} else {
			return -1;
		}
	}
#endif
	if (FTS_KEY_ALL_Y_POS == data->au16_y[i] && 
		(data->au8_touch_event[i] == DOWN_EVENT || data->au8_touch_event[i] == CONTACT_EVENT)) {
		if (nav_key_is_pressed == 1)
			return 0;
		nav_key_is_pressed = 1;
		if (get_stmpe_keypad_status() == 1)
			return 0;

		nav_key_position_x = data->au16_x[i];
		nav_key_position_y = data->au16_y[i];
		nav_key_position_id = i;
		queue_delayed_work(fts_btn_wq, &nav_key_report_work, msecs_to_jiffies(150));

		nav_key_is_reporting = 1;
		nav_key_need_report = 1;
		return 0;
	}
	if (FTS_KEY_ALL_Y_POS == data->au16_y[i] && data->au8_touch_event[i] == UP_EVENT) {
		printk("%s nav_key up x=%d y=%d touch_num=%d nav_key_is_reporting=%d, nav_key_need_report=%d\n",
				__func__, data->au16_x[i], data->au16_y[i], i, nav_key_is_reporting, nav_key_need_report);
		nav_key_is_pressed = 0;
		if (nav_key_is_reporting == 1 && nav_key_need_report == 0) {
			cancel_delayed_work_sync(&nav_key_report_work);
			return 0;
		} else if (nav_key_is_reporting == 1 && nav_key_need_report == 1) {
			cancel_delayed_work_sync(&nav_key_report_work);
			fts_nav_key_report_down_up();
			return 0;
		} else if (nav_key_is_reporting == 0) {
			return -1;
		}
	}
#endif

#if defined(CONFIG_TCT_SDM660_COMMON)
	/*is enable?*/
	if(1 != FTS_KEY_EN)
		return -1;
#endif

	bUsedKeyEvent = 1;//0:not used, 1: used
	if(data->au8_touch_event[i]== DOWN_EVENT || data->au8_touch_event[i] == CONTACT_EVENT)
	{
		switch(data->au16_x[i])
		{
			case FTS_KEY_1_X_POS:
				input_report_key(g_input_dev, FTS_KEY_1, 1);
				break;
			case FTS_KEY_2_X_POS:
				input_report_key(g_input_dev, FTS_KEY_2, 1);
				break;
			case FTS_KEY_3_X_POS:
				input_report_key(g_input_dev, FTS_KEY_3, 1);
				break;
			default:
				bUsedKeyEvent = 0;//not used
				break;
		}
	}
	else
	{
		switch(data->au16_x[i])
		{
			case FTS_KEY_1_X_POS:
				input_report_key(g_input_dev, FTS_KEY_1, 0);
				break;
			case FTS_KEY_2_X_POS:
				input_report_key(g_input_dev, FTS_KEY_2, 0);
				break;
			case FTS_KEY_3_X_POS:
				input_report_key(g_input_dev, FTS_KEY_3, 0);
				break;
			default:
				bUsedKeyEvent = 0;//not used
				break;
		}		//uppoint++;
	}
	if(1 == bUsedKeyEvent)
		input_sync(g_input_dev);
	else
		return -1;

	return 0;

 }

static int fts_work_queue_init(void)
{
	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);

	INIT_WORK(&g_work_touch_event, fts_work_queue_touch_event);
	g_workqueue = create_workqueue("FT_Touch_Workqueue");
	if (!g_workqueue)
	{
		return -1;
	}

	return 0;
}
static int fts_work_queue_exit(void)
{
	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);

	destroy_workqueue(g_workqueue);

	return 0;
}
  /************************************************************************
* Name: fts_work_queue_handle
* Brief: interrupt event from TP, and use work queue
* Input: no use
* Output: no
* Return: 0
***********************************************************************/
 static int fts_work_queue_handle(int irq, void *dev_id)
 {
	//FTS_COMMON_DBG("ALON disable IRQ %s ",  __func__);
	disable_irq_nosync(irq);
	queue_work(g_workqueue, &g_work_touch_event);
	return 0;
 }
 /************************************************************************
* Name: fts_work_queue_touch_event
* Brief: interrupt event from TP, and read/report data to Android system
* Input: no use
* Output: no
* Return: 0
***********************************************************************/
 static void fts_work_queue_touch_event(struct work_struct *work)
 {
	struct ts_event pevent;
	int ret = 0;
	/*is it gesture event?*/
	if(1 == g_is_suspend_mode)
	{
		fts_gesture_handle();
		return;
	}

	// read data
	ret = fts_read_touch_data(&pevent);

	// enable IRQ
	//FTS_COMMON_DBG("ALON enable IRQ %s ",  __func__);
	/* MODIFIED-BEGIN by Haojun Chen, 2016-12-08,BUG-3684946*/
	//enable_irq(fts_i2c_client->irq);
	fts_irq_enable(true);
	/* MODIFIED-END by Haojun Chen,BUG-3684946*/
	if (ret == 0)
		fts_input_dev_report(&pevent);	// report

	return;
 }

static int fts_wait_queue_init(void)
{
	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);

	g_tsk_touch_event = kthread_run(fts_wait_queue_touch_event, 0, "fts touch event");
    	if (IS_ERR(g_tsk_touch_event)) {
       	 printk(KERN_INFO "create kthread failed!");
    	}
    	else {
        	printk(KERN_INFO "create ktrhead ok!");
    	}

	return 0;
}
static int fts_wait_queue_exit(void)
{
	FTS_COMMON_DBG("[focal] Enter %s ",  __func__);

	kthread_stop(g_tsk_touch_event);
	return 0;
}
  /************************************************************************
* Name: fts_wait_queue_handle
* Brief: interrupt event from TP, and use wait queue
* Input: no use
* Output: no
* Return: 0
***********************************************************************/
 static int fts_wait_queue_handle(void)
 {
	g_waitqueue_flag = 1;
	wake_up_interruptible(&g_waitqueue_touch_event);
	return 0;
 }
 /************************************************************************
* Name: fts_wait_queue_touch_event
* Brief: interrupt event from TP, and read/report data to Android system
* Input: no use
* Output: no
* Return: 0
***********************************************************************/
 static int fts_wait_queue_touch_event(void *unused)
 {
	struct ts_event pevent;

	int ret = 0;

	struct sched_param param = { .sched_priority = 2 };
	sched_setscheduler(current, SCHED_RR, &param);

	do
	{
		 wait_event_interruptible(g_waitqueue_touch_event, g_waitqueue_flag!=0);

		 g_waitqueue_flag = 0;

		 set_current_state(TASK_RUNNING);

		 /*is it gesture event?*/
		if(1 == g_is_suspend_mode)
		{
			fts_gesture_handle();
			continue;
		}

		{
                    ret = fts_read_touch_data(&pevent);
			if (ret == 0)
				fts_input_dev_report(&pevent);

		}

 	}while(!kthread_should_stop());

	return 0;
 }
 /************************************************************************
* Name: fts_read_touch_data
* Brief: report the point information
* Input: event info
* Output: get touch data in pinfo
* Return: success is zero
***********************************************************************/

static int fts_read_touch_data(struct ts_event *data)
{
       u8 buf[FTS_TOUCH_DATA_LEN] = { 0 };//0xFF
	int ret = -1;
	int i = 0;
	u8 pointid = FTS_MAX_POINT_ID;

	//FTS_COMMON_DBG("[focal] Enter %s ",  __func__);

	buf[0] = START_ADDR_OF_TOUCH_DATA;
	ret = fts_i2c_read(buf, 1, buf, FTS_TOUCH_DATA_LEN);
	if (ret < 0)
	{
		printk("%s read touchdata failed.", __func__);

		return ret;
	}

	/*add fts_monitor_record of monitor module*/
	fts_monitor_record(buf, FTS_TOUCH_DATA_LEN);

	memset(data, 0, sizeof(struct ts_event));
	data->touch_point = 0;
	data->touch_point_num=buf[ADDR_POINT_NUM] & 0x0F;

	for (i = 0; i < FTS_MAX_TOUCH_POINTS; i++)
	{
		pointid = (buf[ADDR_ID_POS + EVERY_POINT_LEN * i]) >> 4;
		if (pointid >= FTS_MAX_POINT_ID)
			break;
		else
			data->touch_point++;
		data->au16_x[i] =
		    (s16) (buf[ADDR_X_H_POS + EVERY_POINT_LEN * i] & 0x0F) <<
		    8 | (s16) buf[ADDR_X_L_POS + EVERY_POINT_LEN * i];
		data->au16_y[i] =
		    (s16) (buf[ADDR_Y_H_POS + EVERY_POINT_LEN * i] & 0x0F) <<
		    8 | (s16) buf[ADDR_Y_L_POS + EVERY_POINT_LEN * i];
		data->au8_touch_event[i] =
		    buf[ADDR_EVENT_POS + EVERY_POINT_LEN * i] >> 6;
		data->au8_finger_id[i] =
		    (buf[ADDR_ID_POS + EVERY_POINT_LEN * i]) >> 4;

		data->pressure[i] =
			(buf[ADDR_XY_POS + EVERY_POINT_LEN * i]);//cannot constant value
		data->area[i] =
			(buf[ADDR_MISC + EVERY_POINT_LEN * i]) >> 4;

		if((data->au8_touch_event[i]==0 || data->au8_touch_event[i]==2)&&((data->touch_point_num==0)))
		{
			//printk("\n [fts] abnormal touch data from fw. ");
			return 1;
		}
		#if REPORT_TOUCH_DEBUG_EN
			printk("\n [haojun][fts] touch data  2 (id= %d ,x=(0x%02x),y= (0x%02x)),point_num=%d, event=%d \n ",
				data->au8_finger_id[i],
				data->au16_x[i],
				data->au16_y[i],
				data->touch_point,
				data->au8_touch_event[i]);
		#endif

	}

	return 0;
}


