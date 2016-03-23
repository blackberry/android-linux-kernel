/*
 * Copyright (C) 2015 BlackBerry Limited. All rights reserved.
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
#ifdef CONFIG_BBRY_DEBUG
#ifndef __BBRY_MMC_HEALTH_H__
#define __BBRY_MMC_HEALTH_H__

extern ssize_t get_emmc_health(struct mmc_host *host, char *buf);

#define eMMC_HEALTH_BUFFER_SZ		1024
#define DISKPARTITION_DEVICE		0x80
#define MMC_BOOT_PART_MAX		2

#define SDMMC_CMD_TIME_DEFAULT		1000

#define CMD56_GEN_CMD			56
#define MMC_SAMSUNG_VUNIQUE		62
#define SAMSUNG_ENTER_VENDOR_MODE	0xefac62ec
#define SAMSUNG_ENABLE_SMART_REPORT	0x0001ccee
#define SAMSUNG_DISABLE_SMART_REPORT	0x0002ccee
#define SAMSUNG_SMART_REPORT_ENTER	0x0000ccee
#define SAMSUNG_SMART_REPORT_EXIT	0x00deccee
#define SAMSUNG_SMART_SIZE		512

#define TOSHIBA_DEVICE_HEALTH		5
#define TOSHIBA_DEVICE_HEALTH_PWD	0x00175063

#define TOSHIBA_WRITE_ERASE_COUNT	7
#define TOSHIBA_WRITE_ERASE_COUNT_PWD	0xE0662D6E

#define TOSHIBA_STATUS_OK		0xA0	/* byte offset 3 */
#define TOSHIBA_STATUS_NG		0xE0

#define ECSD_SANDISK_NV_HEALTH		87
#define ECSD_SANDISK_EUA_HEALTH		88
#define ECSD_SANDISK_MLC_HEALTH		94
#define ECSD_SANDISK_LIFETIME		96

#define MID_MMC_SANDISK			0x02
#define MID_MMC_SANDISK_2		0x45
#define MID_MMC_TOSHIBA			0x11
#define MID_MMC_MICRON			0x13
#define MID_MMC_SAMSUNG			0x15
#define MID_MMC_HYNIX			0x90
#define MID_MMC_NUMONYX			0xFE

typedef struct _sandisk_health {
	unsigned int		mid;				/* Manufacture ID */
	unsigned char		lifetime;
	unsigned char		nv_avg_pe;			/* NV Cache avg P/E cycle */
	unsigned char		eua_avg_pe;			/* Enhanced User Area avg P/E cycle */
	unsigned char		mlc_avg_pe;			/* MLC avg P/E cycle */
} SANDISK_HEALTH;

typedef struct _samsung_health {
	unsigned int		mid;				/* Manufacture ID */
	unsigned int		bank0_rsvd_blocks;
	unsigned int		bank1_rsvd_blocks;
	unsigned int		bank2_rsvd_blocks;
	unsigned int		bank3_rsvd_blocks;
	unsigned int		init_bad_blocks;
	unsigned int		runtime_bad_blocks;
	unsigned int		slc_max_ec;			/* SLC Maximum Erase Count */
	unsigned int		slc_min_ec;			/* SLC Minimum Erase Count */
	unsigned int		slc_avg_ec;			/* SLC Average Erase Count */
	unsigned int		mlc_max_ec;			/* MLC Maximum Erase Count */
	unsigned int		mlc_min_ec;			/* MLC Minimum Erase Count */
	unsigned int		mlc_avg_ec;			/* MLC Average Erase Count */
	unsigned int		max_ec;				/* Overall Maximum Erase Count */
	unsigned int		min_ec;				/* Overall Minimum Erase Count */
	unsigned int		avg_ec;				/* Overall Average Erase Count */
	unsigned int		read_reclaim;
	unsigned int		num_banks;
	unsigned int		bank4_rsvd_blocks;
	unsigned int		bank5_rsvd_blocks;
	unsigned int		bank6_rsvd_blocks;
	unsigned int		bank7_rsvd_blocks;
} SAMSUNG_HEALTH;

typedef struct _toshiba_health {				/* supported from v4.41 onwards */
	unsigned int		mid;				/* Manufacture ID */
	unsigned int		lifetime_total;
	unsigned int		lifetime_rsvd_blk;
	unsigned int		lifetime_avg_pe;
	unsigned int		mlc_max_pe;			/* MLC Maximum P/E cycle range */
	unsigned int		mlc_avg_pe;			/* MLC Average P/E cycle range */
	unsigned int		slc_max_pe;			/* SLC Maximum P/E cycle range */
	unsigned int		slc_avg_pe;			/* SLC Average P/E cycle range */
} TOSHIBA_HEALTH;

typedef union _sdmmc_device_health {
	unsigned int		mid;				/* Manufacture ID */
	SANDISK_HEALTH		sandisk;
	SAMSUNG_HEALTH		samsung;
	TOSHIBA_HEALTH		toshiba;
	unsigned char		bytes[512];
} SDMMC_DEVICE_HEALTH;

#endif
#endif