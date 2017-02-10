/* Copyright (C) 2016 Tcl Corporation Limited */
/* drivers/input/touchscreen/gt1x.h
 *
 * 2010 - 2013 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Version: 1.4
 * Release Date:  2015/07/10
 */

#ifndef _GOODIX_GT1X_H_
#define _GOODIX_GT1X_H_
#include "gt1x_generic.h"
#include <linux/gpio.h>
#ifdef GTP_CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#endif
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
/*[PLATFORM]-Add-BEGIN by TCTSH.xuefei.wang, Task-2344941, 2016/06/14, add vr function */
extern int gt5688_init_ok;
extern bool charger_in;
#define  CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
#if defined(VR_GLASS)
extern struct work_struct gt5688_work_vr;
extern struct workqueue_struct *gt5688_wq_vr;
#endif
extern struct  gt1x_version_info gt1x_version;
extern void gt5688_change_vr_switch(struct work_struct *work);

#define IIC_MAX_TRANSFER_SIZE       250
extern int virtual_psensor_input_register3(struct i2c_client *pClient);
/*[PLATFORM]-Add-END by TCTSH.xuefei.wang, Task-2344941, 2016/06/14, add vr function */
/* Customize your I/O ports & I/O operations */
#ifdef GTP_CONFIG_OF
extern int gt1x_rst_gpio;
extern int gt1x_int_gpio;
#define GTP_RST_PORT gt1x_rst_gpio
#define GTP_INT_PORT gt1x_int_gpio
#else
#define GTP_RST_PORT    64	//S5PV210_GPJ3(6)
#define GTP_INT_PORT    65	//S5PV210_GPH1(3)
#endif
#define GTP_INT_IRQ     gpio_to_irq(GTP_INT_PORT)
//#define GTP_INT_CFG     S3C_GPIO_SFN(0xF)

#define GTP_GPIO_AS_INPUT(pin)          do{\
                                            gpio_direction_input(pin);\
                                        }while(0)
#define GTP_GPIO_AS_INT(pin)            do{\
                                            GTP_GPIO_AS_INPUT(pin);\
                                        }while(0)
#define GTP_GPIO_GET_VALUE(pin)         gpio_get_value(pin)
#define GTP_GPIO_OUTPUT(pin,level)      gpio_direction_output(pin,level)
#define GTP_IRQ_TAB                     {IRQ_TYPE_EDGE_RISING, IRQ_TYPE_EDGE_FALLING, IRQ_TYPE_LEVEL_LOW, IRQ_TYPE_LEVEL_HIGH}

#endif /* _GOODIX_GT1X_H_ */
