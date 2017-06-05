/*
 * Copyright(c) 2015, BlackBerry Limited
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

#ifndef CONFIG_BBRY
#include "slimport.h"
#else
#include "slimport_private.h"
#endif
#include "slimport_tx_drv.h"
#ifdef CEC_ENABLE
#include "slimport_tx_cec.h"
#endif

/*#define CO2_CABLE_DET_MODE*/
/*//#define CHANGE_TX_P0_ADDR*/

#ifdef SP_REGISTER_SET_TEST
#define sp_tx_link_phy_initialization() do { \
	sp_write_reg(TX_P2, SP_TX_ANALOG_CTRL0, 0x02); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG0, val_SP_TX_LT_CTRL_REG0); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG10, 0x00); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG1, val_SP_TX_LT_CTRL_REG1); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG11, 0x00); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG2, val_SP_TX_LT_CTRL_REG2); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG12, 0x00); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG3, 0x7f); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG13, 0x00); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG4, 0x71); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG14, 0x0c); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG5, val_SP_TX_LT_CTRL_REG5); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG15, val_SP_TX_LT_CTRL_REG15); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG6, val_SP_TX_LT_CTRL_REG6); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG16, val_SP_TX_LT_CTRL_REG16); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG7, 0x73); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG17, 0x3e); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG8, val_SP_TX_LT_CTRL_REG8); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG18, val_SP_TX_LT_CTRL_REG18); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG9, 0x7F); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG19, 0x7e); \
	} while (0)

#elif defined(CONFIG_BBRY)
#define	sp_tx_link_phy_initialization() do { \
	if (is_phy_tuning_set()) { \
		DEV_NOTICE("PHY init with custom tuning"); \
		sp_write_reg(TX_P2, SP_TX_ANALOG_CTRL0, 0x02); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG0, phy_tuning[0]); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG1, phy_tuning[1]); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG2, phy_tuning[2]); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG3, phy_tuning[3]); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG4, phy_tuning[4]); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG5, phy_tuning[5]); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG6, phy_tuning[6]); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG7, phy_tuning[7]); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG8, phy_tuning[8]); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG9, phy_tuning[9]); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG10, phy_tuning[10]); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG11, phy_tuning[11]); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG12, phy_tuning[12]); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG13, phy_tuning[13]); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG14, phy_tuning[14]); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG15, phy_tuning[15]); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG16, phy_tuning[16]); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG17, phy_tuning[17]); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG18, phy_tuning[18]); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG19, phy_tuning[19]); \
	} else { \
		sp_write_reg(TX_P2, SP_TX_ANALOG_CTRL0, 0x02); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG0, 0x01); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG10, 0x00); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG1, 0x03); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG11, 0x00); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG2, 0x07); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG12, 0x00); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG3, 0x7f); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG13, 0x00); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG4, 0x71); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG14, 0x0c); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG5, 0x6b); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG15, 0x42); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG6, 0x7f); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG16, 0x1e); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG7, 0x73); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG17, 0x3e); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG8, 0x7f); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG18, 0x72); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG9, 0x7F); \
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG19, 0x7e); \
	} \
	} while (0)
#else
#define sp_tx_link_phy_initialization() do { \
	sp_write_reg(TX_P2, SP_TX_ANALOG_CTRL0, 0x02); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG0, 0x01); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG10, 0x00); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG1, 0x03); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG11, 0x00); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG2, 0x07); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG12, 0x00); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG3, 0x7f); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG13, 0x00); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG4, 0x71); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG14, 0x0c); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG5, 0x6b); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG15, 0x42); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG6, 0x7f); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG16, 0x1e); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG7, 0x73); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG17, 0x3e); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG8, 0x7f); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG18, 0x72); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG9, 0x7F); \
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG19, 0x7e); \
	} while (0)

#endif

#ifdef CHANGE_TX_P0_ADDR
#ifdef TX_P0
	#undef TX_P0
	#define TX_P0 0x78
#endif
#endif
