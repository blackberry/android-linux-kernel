/*
 * Copyright (C) 2015 BlackBerry Limited
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/types.h>
#include <linux/major.h>

extern int sdmmc_assd_ioctl(struct mmc_host *host, unsigned int cmd, unsigned long arg);

#define CMD_OFFSET	2
/* APDU send/ receive from given buffer pointer */
#define SDMMC_ASSD_APDU			_IOWR(MMC_BLOCK_MAJOR, (0+CMD_OFFSET), __u8 *)
/* Initilize ASSD module for given security system index */
#define SDMMC_ASSD_INIT			_IOW(MMC_BLOCK_MAJOR, (1+CMD_OFFSET), __u32)
/* Read ASSD card properties */
#define SDMMC_ASSD_CARD_PROPERTIES	_IOR(MMC_BLOCK_MAJOR, (2+CMD_OFFSET), __u8 *)
/* Read ASSD card status */
#define SDMMC_ASSD_CARD_STATUS		_IOR(MMC_BLOCK_MAJOR, (3+CMD_OFFSET), __u8 *)
/* Read ASSD card info */
#define SDMMC_ASSD_GET_CARD_INFO	_IOR(MMC_BLOCK_MAJOR, (4+CMD_OFFSET), __u8 *)
/* Set power state as per the enum below */
#define SDMMC_ASSD_CARD_POWER		_IOW(MMC_BLOCK_MAJOR, (5+CMD_OFFSET), __u32)

enum {
	ASSD_POWER_HOLD,
	ASSD_POWER_RELEASE,
	ASSD_POWER_RESET,
};
