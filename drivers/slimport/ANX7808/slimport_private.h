/*
 * Copyright (C) 2015 BlackBerry Limited
 * Copyright(c) 2012, Analogix Semiconductor. All rights reserved.
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

/* This is a renamed copy of slimport.h */
#ifndef _SLIMPORT_PRIAVTE_H
#define _SLIMPORT_PRIVATE_H

#define DEBUG
#define DEV_TAG "slimport: "


#ifdef DEBUG
#define DEV_DBG(args...) pr_info(DEV_TAG args)
#else
#define DEV_DBG(args...) (void)0
#endif

#define DEV_NOTICE(args...) pr_notice(DEV_TAG args)
#define DEV_ERR(args...) pr_err(DEV_TAG args)


#define SSC_EN

#if 0
#define SSC_1
#define EYE_TEST
#define EDID_DEBUG_PRINT
#endif

#define AUX_ERR 1
#define AUX_OK 0

extern bool sp_tx_hw_lt_done;
extern bool sp_tx_hw_lt_enable;
extern bool	sp_tx_link_config_done ;
extern enum SP_TX_System_State sp_tx_system_state;
extern enum RX_CBL_TYPE sp_tx_rx_type;
extern enum RX_CBL_TYPE sp_tx_rx_type_backup;
extern unchar sp_tx_pd_mode;
extern enum SP_TX_System_State sp_tx_system_state_bak;

#ifdef CONFIG_SLIMPORT_FAST_CHARGE
/*for ccharging status*/
extern enum CHARGING_STATUS downstream_charging_status;
#endif


extern unchar bedid_break;
extern unchar sp_tx_hw_hdcp_en;
/*extern struct i2c_client *anx7808_client;*/

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
int slimport_read_edid_block(int block, uint8_t *edid_buf);
enum SP_LINK_BW slimport_get_link_bw(void);
enum RX_CBL_TYPE sp_get_ds_cable_type(void);
#ifdef CONFIG_SLIMPORT_FAST_CHARGE
enum CHARGING_STATUS sp_get_ds_charge_type(void);
#endif
#endif
