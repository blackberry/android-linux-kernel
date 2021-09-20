/*
 *
 * FocalTech TouchScreen driver.
 * 
 * Copyright (c) 2010-2016, Focaltech Ltd. All rights reserved.
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
* File Name: focaltech_monitor.c
*
*  Author: Xu YF & ZR, Software Department, FocalTech
*
* Created: 2016-02-01
*
* Modified:
*
*  Abstract:
*	Monitor real-time touch data that report to input subsystem and check it's validaty.
*	Add UP deal when there're some exceptional situation.
*
*This module real-time monitoring the touch data to report to the Input subsystem, to judge its effectiveness, 
*to do UP action to repair the abnormal situation.
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include <linux/time.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include "focaltech_comm.h"
/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FTS_MONITOR_INFO  "File Version of  focaltech_monitor.c:  V1.0.0 2016-03-16"

/*monitor enable and wait time*/
#define FTS_MONITOR_EN      0//0:disable, 1: enable 
#define FTS_MONITOR_WAIT_TIME       200//ms interval time of monitor 

#define FTS_MONITOR_DEBUG_EN      0//0:disable, 1: enable 

/*jugde distance*/
#define FTS_JUGDE_DISTANCE_EN           0    //0=close, 1=open, default 0
#define FTS_TWO_POINTS_DISTANCE    100 // minimum distance of two points

/*point event*/
#define DOWN_EVENT      0
#define UP_EVENT      1
#define CONTACT_EVENT    2
#define NO_EVENT      3

#define NO_TOUCH      0xffff

/*touch data*/
#define EVERY_POINT_LEN        6 //the bytes of every point
#define FTS_START_ADDR          0x00 //the start addr of get touch data
#define ADDR_X_H_POS          (3 - FTS_START_ADDR)
#define ADDR_X_L_POS          (4 - FTS_START_ADDR)
#define ADDR_Y_H_POS          (5 - FTS_START_ADDR)
#define ADDR_Y_L_POS          (6 - FTS_START_ADDR)
#define ADDR_EVENT_POS          (3 - FTS_START_ADDR)
#define ADDR_ID_POS            (5 - FTS_START_ADDR)
#define ADDR_POINT_NUM          (2 - FTS_START_ADDR)
#define FTS_MAX_POINT_ID        0x0F

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/
struct struct_touch_data
{
	unsigned char point_id[FTS_MAX_POINT_ID];
	unsigned char event[FTS_MAX_POINT_ID];	
	unsigned int X[FTS_MAX_POINT_ID];
	unsigned int Y[FTS_MAX_POINT_ID];
	struct timeval record_time;	
};
struct struct_monitor
{
	int iThread_stop_flag;
	int bReviewTouchData;
	int current_report_num;
	int total_report_num;
	struct timeval iLastRecordTime;
	struct input_dev *dev;
	bool bUseProtoclB;
	struct struct_touch_data last_data;
	struct struct_touch_data current_data;
	struct struct_touch_data report_data;	
};

/*******************************************************************************
* Static variables
*******************************************************************************/
static struct timeval tv;	
static struct struct_monitor st_monitor;
static struct task_struct *thread_monitor = NULL;

static DECLARE_WAIT_QUEUE_HEAD(monitorwaiter);
static int monitor_waiter_flag=0;

static int g_drv_up_event_count = 0;
static int g_drv_other_event_count = 0;

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
//static int fts_monitor_init(void);
//static int fts_monitor_exit(void);
static int fts_monitor_thread(void *unused);
//static int fts_monitor_record(char *readbuf, unsigned char readlen);
static int fts_monitor_compare(void);
static int fts_monitor_review(void);
static int fts_parse_touch_data(char *readbuf, unsigned char readlen);
static int clear_last_data(void);
static int clear_current_data(void);
static int clear_report_data(void);
static int fts_report_added(void);
static int fts_report_a_protocol(void);
static int fts_report_b_protocol(void);
static int print_current_data(void);

/*calculate distance*/
static int focal_abs(int value);
static unsigned int SqrtNew(unsigned int n) ;
unsigned int DistanceToPoint(unsigned int point_x1, unsigned int point_y1,unsigned int point_x2,unsigned int point_y2);
static int fts_monitor_distance(void);

/*touch event count*/
static int fts_touch_count_init(void);
static int fts_touch_count_exit(void);	
static ssize_t fts_touch_count_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t fts_touch_count_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

/*******************************************************************************
* functions
*******************************************************************************/
int fts_monitor_init(struct input_dev *dev,  int bUseProtocolB)
{
	int err = 0;

	if(!FTS_MONITOR_EN)
		return 0;
	FTS_COMMON_DBG("[focal] %s ",  FTS_MONITOR_INFO);	//show version
	
	st_monitor.iThread_stop_flag = 0;
	st_monitor.bReviewTouchData = 1;
	st_monitor.total_report_num = 0;
	st_monitor.dev = dev;
	st_monitor.bUseProtoclB = bUseProtocolB;
	clear_current_data();
	clear_last_data();
	clear_report_data();
	do_gettimeofday(&st_monitor.iLastRecordTime);

	fts_touch_count_init();
	
	thread_monitor = kthread_run(fts_monitor_thread, 0, "focal-monitor");
	if (IS_ERR(thread_monitor))
	{
		err = PTR_ERR(thread_monitor);
		FTS_COMMON_DBG("failed to create kernel thread: %d",  err);
	}

	return 0;	
}
int fts_monitor_exit(void)
{
	if(!FTS_MONITOR_EN)
		return 0;
	
	FTS_COMMON_DBG("");

	fts_touch_count_exit();
	
	st_monitor.iThread_stop_flag = 1;
	monitor_waiter_flag=1;
	wake_up_interruptible(&monitorwaiter);

	msleep(2);
	st_monitor.bReviewTouchData=0;
	wake_up_interruptible(&monitorwaiter);

	msleep(FTS_MONITOR_WAIT_TIME);		
	//kthread_stop(thread_monitor);

	
	return 0;
}
static int fts_monitor_thread(void *unused)
{
	unsigned int iDeltaTime=0;
//	int ret = 0;
	unsigned long uljiffies=0;

	struct sched_param param = { .sched_priority = 5 };
	sched_setscheduler(current, SCHED_RR, &param); 
	uljiffies=msecs_to_jiffies(FTS_MONITOR_WAIT_TIME+20);
	FTS_COMMON_DBG("");
	
	do
	{		
		wait_event_interruptible(monitorwaiter, 0 != monitor_waiter_flag);
		monitor_waiter_flag=0;
		FTS_COMMON_DBG("begin monitor: monitor_waiter_flag reset to %d .",  monitor_waiter_flag);
		do_gettimeofday(&st_monitor.iLastRecordTime);

		st_monitor.bReviewTouchData=1;//first

		if(1== st_monitor.iThread_stop_flag) break;
		
		do
		{
			wait_event_interruptible_timeout(monitorwaiter,0 == st_monitor.bReviewTouchData, uljiffies);

			if(1== st_monitor.iThread_stop_flag) break;
			/*if(1 == st_monitor.bReviewTouchData)
			{
				msleep(FTS_MONITOR_WAIT_TIME);
				do_gettimeofday(&st_monitor.iLastRecordTime);
				continue;
			}*/
			do_gettimeofday(&tv);
			iDeltaTime = (tv.tv_sec - st_monitor.iLastRecordTime.tv_sec)*MSEC_PER_SEC + (tv.tv_usec - st_monitor.iLastRecordTime.tv_usec)/1000;
			//st_monitor.iLastRecordTime.tv_sec=tv.tv_sec;
			//st_monitor.iLastRecordTime.tv_usec=tv.tv_usec;
					
			if(FTS_MONITOR_WAIT_TIME < iDeltaTime)
			{
				FTS_COMMON_DBG("enter fts_monitor_review(): iDeltaTime(ms) %d .",  iDeltaTime);

				if(0 == st_monitor.bReviewTouchData)
				{
					fts_monitor_review();
				}

				if(FTS_JUGDE_DISTANCE_EN)
				{
					fts_monitor_distance();
				}

				monitor_waiter_flag=0;
				
				break;
			}

			//FTS_COMMON_DBG("monitor waiter loop: iDeltaTime(ms) %d .",  iDeltaTime);
			st_monitor.bReviewTouchData=1;

			//msleep(FTS_MONITOR_WAIT_TIME);
		}while(!kthread_should_stop());
	}while(!kthread_should_stop());

	return 0;
}

int fts_monitor_record(char *readbuf, unsigned char readlen)
{
	int iret = 0;
	int i = 0;

	if(!FTS_MONITOR_EN)
		return 0;

	if(FTS_MONITOR_DEBUG_EN)
	{
		/* print data packet of touch */
		printk("Touch Data: ");
		for(i = 0; i < readlen; i++)
			printk("%02x ", readbuf[i]);
		printk("\n");
	}
	
	do_gettimeofday(&st_monitor.iLastRecordTime);
	//st_monitor.iLastRecordTime = tv.tv_usec;	
	st_monitor.bReviewTouchData = 0;
	iret = fts_parse_touch_data(readbuf, readlen);
	if(iret >= 0)
	{
		st_monitor.current_data.record_time = st_monitor.iLastRecordTime;
		
		#if FTS_JUGDE_DISTANCE_EN
		fts_monitor_distance();
		#endif
		
		fts_monitor_compare();
	}

	monitor_waiter_flag=1;
	wake_up_interruptible(&monitorwaiter);
	
	return 0;
}

static int fts_monitor_compare(void)
{

	int i = 0;
	
	clear_report_data();	
	for(i=0; i<FTS_MAX_POINT_ID; i++)
	{
		switch(st_monitor.last_data.event[i])
		{
			case NO_EVENT:
			case UP_EVENT:
			{	
				break;
			}
			case CONTACT_EVENT:
			case DOWN_EVENT:
			{
				if((DOWN_EVENT == st_monitor.current_data.event[i])
					||(NO_EVENT == st_monitor.current_data.event[i])
					)
				{
					st_monitor.report_data.point_id[st_monitor.current_report_num] = i;
					st_monitor.report_data.event[st_monitor.current_report_num] = UP_EVENT;
					st_monitor.report_data.X[st_monitor.current_report_num] = st_monitor.last_data.X[i];
					st_monitor.report_data.Y[st_monitor.current_report_num] = st_monitor.last_data.Y[i];					
					st_monitor.current_report_num++;	
					FTS_COMMON_DBG("Lost Up Event by fts_monitor_compare() ! Point_id = %d, X = %d, Y = %d", 
						i,  st_monitor.last_data.X[i],  st_monitor.last_data.Y[i]);
				}					
				break;
			}
			default:
				break;
		}
		st_monitor.last_data.event[i] = st_monitor.current_data.event[i];
		st_monitor.last_data.X[i] = st_monitor.current_data.X[i];
		st_monitor.last_data.Y[i] = st_monitor.current_data.Y[i];	
		
	}
	st_monitor.last_data.record_time = st_monitor.current_data.record_time;
	
	if(st_monitor.current_report_num > 0)
	{
		fts_report_added();
	}
	return 0;
}

static int fts_monitor_review(void)
{
	int i = 0;	

	st_monitor.bReviewTouchData = 1;

	clear_report_data();
	for(i=0; i<FTS_MAX_POINT_ID; i++)
	{
		switch(st_monitor.last_data.event[i])
		{
			case CONTACT_EVENT:
			case DOWN_EVENT:
			{
				st_monitor.report_data.point_id[st_monitor.current_report_num] = i;
				st_monitor.report_data.event[st_monitor.current_report_num] = UP_EVENT;
				st_monitor.report_data.X[st_monitor.current_report_num] = st_monitor.last_data.X[i];
				st_monitor.report_data.Y[st_monitor.current_report_num] = st_monitor.last_data.Y[i];					
				st_monitor.current_report_num++;		
				FTS_COMMON_DBG("Lost Up Event by fts_monitor_review() ! Point_id = %d, X = %d, Y = %d", 
					i,  st_monitor.last_data.X[i],  st_monitor.last_data.Y[i]);
				break;
			}
			default:
				break;
		}
	}

	if(st_monitor.current_report_num > 0)
	{
		fts_report_added();
	}
	return 0;
}

static int clear_last_data(void)
{
	int i = 0;
	for(i=0; i<FTS_MAX_POINT_ID; i++)
	{
		st_monitor.last_data.event[i] = NO_EVENT;
		st_monitor.last_data.point_id[i] = i;
		st_monitor.last_data.X[i] = NO_TOUCH;
		st_monitor.last_data.Y[i] = NO_TOUCH;
	}
	do_gettimeofday(&st_monitor.last_data.record_time);

	return 0;
}

static int clear_current_data(void)
{
	int i = 0;
	for(i=0; i<FTS_MAX_POINT_ID; i++)
	{
		st_monitor.current_data.event[i] = NO_EVENT;
		st_monitor.current_data.point_id[i] = i;
		st_monitor.current_data.X [i]= NO_TOUCH;
		st_monitor.current_data.Y[i] = NO_TOUCH;
	}
	do_gettimeofday(&st_monitor.current_data.record_time);

	return 0;
}
static int clear_report_data(void)
{
	int i = 0;
	
	for(i=0; i<FTS_MAX_POINT_ID; i++)
	{
		st_monitor.report_data.event[i] = NO_EVENT;
		st_monitor.report_data.point_id[i] = i;
		st_monitor.report_data.X[i] = NO_TOUCH;
		st_monitor.report_data.Y[i] = NO_TOUCH;
	}
	do_gettimeofday(&st_monitor.report_data.record_time);
	st_monitor.current_report_num = 0;
	
	return 0;
}
static int print_current_data(void)
{
	int i = 0;
	for(i=0; i<FTS_MAX_POINT_ID; i++)
	{
		if(st_monitor.current_data.event[i] != NO_EVENT)
		{
			FTS_COMMON_DBG("point_id = %d, event = %d, X = %d, Y = %d",
				i, 
				st_monitor.current_data.event[i],
				st_monitor.current_data.X [i],
				st_monitor.current_data.Y[i] 
				);
		}
	}
	return 0;
}
static int fts_parse_touch_data(char *readbuf, unsigned char readlen)
{
	int i=0;
	unsigned char id=0;
	unsigned char point_num = 0;

	point_num = readbuf[ADDR_POINT_NUM]&0x0f;
	if(point_num >= FTS_MAX_POINT_ID)
	{
		FTS_COMMON_DBG("Error Touch Data ! ");
		return -1;
	}
	else
	{
		clear_current_data();

		for(i=0; i<FTS_MAX_POINT_ID &&((3-FTS_START_ADDR+(i+1)*EVERY_POINT_LEN) <= readlen); i++)
		{
			id = ((readbuf[ADDR_ID_POS+i*EVERY_POINT_LEN])>>4); //y\D6\E1\B8\DF\D7ֽڵ\DA 1-5bit
			//st_monitor.current_data.point_id[id] = id;
			if(id < FTS_MAX_POINT_ID)
			{
				st_monitor.current_data.event[id] = ((readbuf[ADDR_EVENT_POS+i*EVERY_POINT_LEN])>>6);  //x\D6\E1\B8\DF\D7ֽ\DA2bit, event
				if(NO_EVENT !=st_monitor.current_data.event[id])
				{
					st_monitor.current_data.X[id] = ((readbuf[ADDR_X_H_POS +i*EVERY_POINT_LEN]&0x0f)<<8)
						+readbuf[ADDR_X_L_POS+i*EVERY_POINT_LEN];
					st_monitor.current_data.Y[id] = ((readbuf[ADDR_Y_H_POS+i*EVERY_POINT_LEN]&0x0f)<<8)
						+readbuf[ADDR_Y_L_POS+i*EVERY_POINT_LEN];
				}
				if(UP_EVENT == st_monitor.current_data.event[id])
					g_drv_up_event_count++;
				if(DOWN_EVENT == st_monitor.current_data.event[id] || CONTACT_EVENT == st_monitor.current_data.event[id])
					g_drv_other_event_count++;				
			}	
		}
		if(FTS_MONITOR_DEBUG_EN)
			print_current_data();
	}
	
	return 0;
}
static int fts_report_added(void)
{
	st_monitor.total_report_num += st_monitor.current_report_num;

	FTS_COMMON_DBG("Add Up Event ! Total Added Number = %d",  st_monitor.total_report_num);
	if(st_monitor.bUseProtoclB)
		fts_report_b_protocol();
	else
		fts_report_a_protocol();	
	
	return 0;
}
static int fts_report_a_protocol(void)
{
	input_report_key(st_monitor.dev, BTN_TOUCH, 0);
	input_mt_sync(st_monitor.dev);	
	input_sync(st_monitor.dev);
	return 0;
}
static int fts_report_b_protocol(void)
{
	int i = 0;

	for(i = 0; i < st_monitor.current_report_num; i++)
	{
		input_mt_slot(st_monitor.dev, st_monitor.report_data.point_id[i]);
		input_mt_report_slot_state(st_monitor.dev, MT_TOOL_FINGER, false);
	}
	input_report_key(st_monitor.dev, BTN_TOUCH, 0);
	input_sync(st_monitor.dev);
	return 0;
}

static int focal_abs(int value)
{
	if(value < 0)
		value = 0 - value;

	return value;
}

/************************************************************************
* Name: SqrtNew
* Brief:  calculate sqrt of input.
* Input: unsigned int n
* Output: none
* Return: sqrt of n.
***********************************************************************/
static unsigned int SqrtNew(unsigned int n) 
{        
    unsigned int  val = 0, last = 0; 
    unsigned char i = 0;;
    
    if (n < 6)
    {
        if (n < 2)
        {
            return n;
        }
        return n/2;
    }   
    val = n;
    i = 0;
    while (val > 1)
    {
        val >>= 1;
        i++;
    }
    val <<= (i >> 1);
    val = (val + val + val) >> 1;
    do
    {
      last = val;
      val = ((val + n/val) >> 1);
    }while(focal_abs(val-last) > 1);
    return val; 
}

//////////////////////////////////////////////////////////////////////////
////	calculate distance of two points
//////////////////////////////////////////////////////////////////////////
unsigned int DistanceToPoint(unsigned int point_x1, unsigned int point_y1,unsigned int point_x2,unsigned int point_y2)
{
	unsigned int dDistance = 0;
	if ((point_x1 == point_x2)&&(point_y1 == point_y2))
		return 0;

	dDistance = SqrtNew(((point_y1-point_y2)*(point_y1-point_y2)) + ((point_x2 -point_x1)*(point_x2 -point_x1)));
	return dDistance;
}

static int fts_monitor_distance(void)
{

	int i = 0;
	unsigned int iDistance = 0;
	unsigned int iDeltaTime = 0;	
	
	clear_report_data();	
	for(i=0; i<FTS_MAX_POINT_ID; i++)
	{
		switch(st_monitor.last_data.event[i])
		{
			case NO_EVENT:
			case UP_EVENT:
			{	
				break;
			}
			case CONTACT_EVENT:
			case DOWN_EVENT:
			{
				if((CONTACT_EVENT == st_monitor.current_data.event[i])
					//||(NO_EVENT == st_monitor.current_data.event[i])
					)
				{					
					iDistance = DistanceToPoint(
						st_monitor.current_data.X[i],
						st_monitor.current_data.Y[i],
						st_monitor.last_data.X[i],
						st_monitor.last_data.Y[i]
						);
					if(iDistance > FTS_TWO_POINTS_DISTANCE)
					{
						/*	calculate interval time of two points */						
						iDeltaTime = (st_monitor.current_data.record_time.tv_sec - st_monitor.last_data.record_time.tv_sec)*MSEC_PER_SEC 
							+ (st_monitor.current_data.record_time.tv_usec - st_monitor.last_data.record_time.tv_usec)/1000;
	
					FTS_COMMON_DBG("Abnormal distance: %d ! point_id = %d, last_point = (%d, %d), current_point = (%d, %d), delta_time = %d ms", 
						iDistance, i,  st_monitor.last_data.X[i],  st_monitor.last_data.Y[i],
						st_monitor.current_data.X[i],  st_monitor.current_data.Y[i],
						iDeltaTime
						);	
					
					st_monitor.report_data.point_id[st_monitor.current_report_num] = i;
					st_monitor.report_data.event[st_monitor.current_report_num] = UP_EVENT;
					st_monitor.report_data.X[st_monitor.current_report_num] = st_monitor.last_data.X[i];
					st_monitor.report_data.Y[st_monitor.current_report_num] = st_monitor.last_data.Y[i];					
					st_monitor.current_report_num++;	
					FTS_COMMON_DBG("Lost Up Event by fts_monitor_distance() ! Point_id = %d, X = %d, Y = %d", 
						i,  st_monitor.last_data.X[i],  st_monitor.last_data.Y[i]);
					}
				}					
				break;
			}
			default:
				break;
		}	
	}
	
	if(st_monitor.current_report_num > 0)
	{
		fts_report_added();
	}
	return 0;
}


static ssize_t fts_touch_count_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return snprintf(buf, PAGE_SIZE, "%d", 0);
}

static ssize_t fts_touch_count_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int buff_data = 0;
	int ret =0;
	unsigned char fw_up_event_h = 0, fw_up_event_l = 0;
	unsigned char fw_other_event_h = 0, fw_other_event_l = 0;

	if('0' == buf[0])
		buff_data = 0;
	else if ('1' == buf[0])
		buff_data = 1;	
	else
		buff_data = 2;		
	
	FTS_COMMON_DBG("buff_data = %d, buf[0] = %d", buff_data, buf[0]);
	
	switch(buff_data)
		{
		case 0:
			ret = fts_write_reg(0x90, 0x00);
			if(ret < 0)goto WRITE_ERR;
			msleep(10);
			ret = fts_write_reg(0x91, 0x00);
			if(ret < 0)goto WRITE_ERR;
			msleep(10);
			ret = fts_write_reg(0x92, 0x00);
			if(ret < 0)goto WRITE_ERR;
			msleep(10);
			ret = fts_write_reg(0x93, 0x00);
			if(ret < 0)goto WRITE_ERR;
			msleep(10);
			ret = fts_read_reg(0x90, &fw_up_event_l);
			if(ret < 0)goto READ_ERR;	
			msleep(10);
			ret = fts_read_reg(0x91, &fw_up_event_h);
			if(ret < 0)goto READ_ERR;
			msleep(10);
			ret = fts_read_reg(0x92, &fw_other_event_l);
			if(ret < 0)goto READ_ERR;
			msleep(10);
			ret = fts_read_reg(0x93, &fw_other_event_h);
			if(ret < 0)goto READ_ERR;	
			
			g_drv_up_event_count = 0;
			g_drv_other_event_count = 0;
			
			FTS_COMMON_DBG("Clear touch count. From FW: up = %d, down&contact = %d. From Driver: up = %d, down&contact = %d.",
				fw_up_event_l + (fw_up_event_h<<8),
				fw_other_event_l + (fw_other_event_h<<8),
				g_drv_up_event_count,
				g_drv_other_event_count
				);
			break;
		case 1:
			ret = fts_read_reg(0x90, &fw_up_event_l);
			if(ret < 0)goto READ_ERR;
			msleep(10);
			ret = fts_read_reg(0x91, &fw_up_event_h);
			if(ret < 0)goto READ_ERR;
			msleep(10);
			ret = fts_read_reg(0x92, &fw_other_event_l);
			if(ret < 0)goto READ_ERR;
			msleep(10);
			ret = fts_read_reg(0x93, &fw_other_event_h);
			if(ret < 0)goto READ_ERR;
			FTS_COMMON_DBG("Get touch count. From FW: up = %d, down&contact = %d. From Driver: up = %d, down&contact = %d.",
				fw_up_event_l + (fw_up_event_h<<8),
				fw_other_event_l + (fw_other_event_h<<8),
				g_drv_up_event_count,
				g_drv_other_event_count
				);			
			break;
		default:
			FTS_COMMON_DBG("Invalid param of fts_touch_count.");
			break;
		}

	return count;
	
WRITE_ERR:
	FTS_COMMON_DBG("Write Reg Error.");	
	return -1;
	
READ_ERR:
	FTS_COMMON_DBG("Read Reg Error.");
	return -1;	
}
/*  fts_touch_count
*    example:
	clear count, cmd:  echo 0 > fts_touch_count
	get count, cmd:  echo 1 > fts_touch_count	
*/
static DEVICE_ATTR(fts_touch_count, S_IRUGO|S_IWUSR, fts_touch_count_show, fts_touch_count_store);

/* add your attr in here*/
static struct attribute *fts_touch_count_attributes[] = {
	&dev_attr_fts_touch_count.attr,
	NULL
};

static struct attribute_group fts_touch_count_attribute_group = {
	.attrs = fts_touch_count_attributes
};

static int fts_touch_count_init(void)
{
	int err=0;
	
	FTS_COMMON_DBG("");//default print: current function name and line number
	
	err = sysfs_create_group(&fts_i2c_client->dev.kobj, &fts_touch_count_attribute_group);
	if (0 != err) 
	{
		FTS_COMMON_DBG( "[focal] %s() - ERROR: sysfs_create_group() failed.",  __func__);
		sysfs_remove_group(&fts_i2c_client->dev.kobj, &fts_touch_count_attribute_group);
		return -EIO;
	} 
	else 
	{
		FTS_COMMON_DBG("[focal] %s() - sysfs_create_group() succeeded.", __func__);
	}
	//fts_protocol_windows_to_android(client);

	g_drv_up_event_count = 0;
	g_drv_other_event_count = 0;
	
	return err;
}
static int fts_touch_count_exit(void)
{	
	FTS_COMMON_DBG("");//default print: current function name and line number
	sysfs_remove_group(&fts_i2c_client->dev.kobj, &fts_touch_count_attribute_group);
	
	return 0;
}

