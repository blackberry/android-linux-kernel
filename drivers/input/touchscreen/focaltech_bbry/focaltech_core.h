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

#ifndef __FOCALTECH_CORE_H__
#define __FOCALTECH_CORE_H__
 /*******************************************************************************
*
* File Name: focaltech_core.h
*
*  Author: Xu YF & ZR, Software Department, FocalTech
*
* Created: 2016-01-20
*
*  Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* Included header files
*******************************************************************************/
#include "focaltech_comm.h"

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/

#define FTS_DBG_EN 1
#if FTS_DBG_EN
#define FTS_DBG(fmt, args...) 				printk("[FTS]" fmt, ## args)
#else
#define FTS_DBG(fmt, args...) 				do{}while(0)
#endif


// driver inside used data which should not modify by extern.
/*struct fts_ts_inside_data {
	//struct i2c_client *client;
	//struct input_dev *input_dev;
	struct ts_event event;
	const struct ts_platform_data *pdata;
	struct work_struct 	touch_event_work;
	struct workqueue_struct *ts_workqueue;
};*/

/////////////////////////////////////////////
/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
//extern struct i2c_client *fts_i2c_client;
//extern struct input_dev *fts_input_dev;
//extern struct fts_ts_platform_data *fts_platform_data;

//Base functions
//extern int fts_i2c_read(/*struct i2c_client *client,*/unsigned char *writebuf, int writelen, unsigned char *readbuf, int readlen);
//extern int fts_i2c_write(/*struct i2c_client *client,*/unsigned char *writebuf, int writelen);
//extern int fts_read_reg(/*struct i2c_client *client,*/unsigned char addr, unsigned char *val);
//extern int fts_write_reg(/*struct i2c_client *client,*/ unsigned char addr, const unsigned char val);


#endif
