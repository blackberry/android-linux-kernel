/*
 * Copyright (C) 2015 BlackBerry Limited
 * Copyright(c) 2014, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _SLIMPORT_H
#define _SLIMPORT_H
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/async.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#ifndef CONFIG_BBRY
#include <video/anx_slimport.h>
#else
#include <linux/switch.h>
#include <mdss_hdmi_slimport.h>
#endif

#define SSC_EN

#define LOG_TAG "SlimPort Colorado3"

#define AUX_ERR 1
#define AUX_OK 0

#ifdef CONFIG_BBRY
#ifdef DEBUG
#define DEV_DBG(args...) pr_info(LOG_TAG args)
#else
#define DEV_DBG(args...) (void)0
#endif

#define DEV_NOTICE(args...) pr_notice(LOG_TAG args)
#define DEV_ERR(args...) pr_err(LOG_TAG args)

#define NEW_HDCP_CONTROL_LOGIC
#endif

#if defined(CONFIG_BBRY_DEBUG) || defined(CONFIG_BBRY_MFG)
extern int have_custom_sw_pre;
extern int custom_sw_pre;
#endif

#ifdef CONFIG_BBRY
#define NUM_TUNING_VALS (20)
extern int phy_tuning[NUM_TUNING_VALS];
bool is_phy_tuning_set(void);
#endif

int sp_read_reg(uint8_t slave_addr, uint8_t offset, uint8_t *buf);
int sp_write_reg(uint8_t slave_addr, uint8_t offset, uint8_t value);
void sp_tx_hardware_poweron(void);
void sp_tx_hardware_powerdown(void);

#ifdef CONFIG_BBRY
void handle_chg_query(void);
bool slimport_dongle_is_connected(void);
void slimport_publish_hdcp_cap(unchar hdcp_cap);
void slimport_publish_online(int online);
#endif
#endif
