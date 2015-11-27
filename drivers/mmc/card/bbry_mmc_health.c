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
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/scatterlist.h>
#include <linux/delay.h>
#include <linux/mmc/ioctl.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include "bbry_mmc_health.h"

#define le32toh(x) (*((unsigned int *)(x)))
#define be_to_le(x)	((((x) >> 24) & 0xff) | \
			(((x) >> 8) & 0xff00) | \
			(((x) & 0xff00) << 8) | \
			(((x) & 0xff) << 24))

int samsung_vendor_command(struct mmc_host *host, unsigned int vcmd)
{
	s32			err = 0;
	struct mmc_command	cmd = {0};

	cmd.opcode = MMC_SAMSUNG_VUNIQUE;
	cmd.arg    = SAMSUNG_ENTER_VENDOR_MODE;
	cmd.flags  = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
	err = mmc_wait_for_cmd(host, &cmd, 5);
	if (err)
		pr_debug("%s returns error (%d)\n", __func__, err);

	memset((struct mmc_command *)&cmd, 0, sizeof(struct mmc_command));
	udelay(20); /* delay based on BB10 logic */

	cmd.opcode = MMC_SAMSUNG_VUNIQUE;
	cmd.arg    = vcmd;
	cmd.flags  = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
	err = mmc_wait_for_cmd(host, &cmd, 5);
	if (err)
		pr_debug("%s returns error (%d)\n", __func__, err);

	return err;
}

int samsung_smart_report(struct mmc_host *host, unsigned char *sr)
{
	s32			err = 0;
	struct mmc_request	req;
	struct mmc_command	cmd;
	struct mmc_data		dat;
	struct scatterlist	sg;

	memset(&req, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&dat, 0, sizeof(struct mmc_data));

	if ((err = samsung_vendor_command(host, SAMSUNG_ENABLE_SMART_REPORT)) == 0) {
		if ((err = samsung_vendor_command(host, SAMSUNG_SMART_REPORT_ENTER)) == 0) {
			udelay(10); /* delay based on BB10 logic */
			req.cmd    = &cmd;
			req.data   = &dat;
			cmd.opcode = MMC_READ_SINGLE_BLOCK;
			cmd.arg    = 0x0;
			cmd.flags  = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;
			dat.blksz  = 512;
			dat.blocks = 1;
			dat.flags  = MMC_DATA_READ;
			dat.sg     = &sg;
			dat.sg_len = 1;
			sg_init_one(&sg, sr, 512);

			mmc_set_data_timeout(&dat, host->card);
			mmc_wait_for_req(host, &req);

			if (cmd.error) {
				pr_debug("%s returns cmd error (%d)\n", __func__, cmd.error);
				return cmd.error;
			}
			if (dat.error) {
				pr_debug("%s returns data error (%d)\n", __func__, dat.error);
				return dat.error;
			}
		} else
			pr_err("%s Error: SAMSUNG_SMART_REPORT_ENTER failed %d\n", __func__, err);

		samsung_vendor_command(host, SAMSUNG_DISABLE_SMART_REPORT);
		samsung_vendor_command(host, SAMSUNG_SMART_REPORT_EXIT);
	} else
		pr_err("%s Error: SAMSUNG_ENABLE_SMART_REPORT failed %d\n", __func__, err);

	return err;
}

int toshiba_gen_cmd(struct mmc_host *host, unsigned int scmd, unsigned int pwd, unsigned char *rdata)
{
	struct mmc_request	req;
	struct mmc_command	cmd;
	struct mmc_data		dat;
	struct scatterlist	sg;

	rdata[0] = scmd;
	rdata[4] = pwd & 0xff;
	rdata[5] = ( pwd >> 8 ) & 0xff;
	rdata[6] = ( pwd >> 16 ) & 0xff;
	rdata[7] = ( pwd >> 24 ) & 0xff;

	memset(&req, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&dat, 0, sizeof(struct mmc_data));
	req.cmd    = &cmd;
	req.data   = &dat;
	cmd.opcode = MMC_GEN_CMD;
	cmd.arg    = 0x0;
	cmd.flags  = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;
	dat.blksz  = 512;
	dat.blocks = 1;
	dat.flags  = MMC_DATA_WRITE;
	dat.sg     = &sg;
	dat.sg_len = 1;
	sg_init_one(&sg, rdata, 512);

	mmc_set_data_timeout(&dat, host->card);
	mmc_wait_for_req(host, &req);

	if (cmd.error) {
		pr_debug("%s returns cmd error (%d)\n", __func__, cmd.error);
		return cmd.error;
	}
	if (dat.error) {
		pr_debug("%s returns data error (%d)\n", __func__, dat.error);
		return dat.error;
	}

	memset(&req, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&dat, 0, sizeof(struct mmc_data));
	req.cmd    = &cmd;
	req.data   = &dat;
	cmd.opcode = MMC_GEN_CMD;
	cmd.arg    = 0x1;
	cmd.flags  = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;
	dat.blksz  = 512;
	dat.blocks = 1;
	dat.flags  = MMC_DATA_READ;
	dat.sg     = &sg;
	dat.sg_len = 1;
	sg_init_one(&sg, rdata, 512);

	mmc_set_data_timeout(&dat, host->card);
	mmc_wait_for_req(host, &req);

	if (cmd.error) {
		pr_debug("%s returns cmd error (%d)\n", __func__, cmd.error);
		return cmd.error;
	}
	if (dat.error) {
		pr_debug("%s returns data error (%d)\n", __func__, dat.error);
		return dat.error;
	}

	if (rdata[3] != TOSHIBA_STATUS_OK) {
		pr_err("%s Error: TOSHIBA STATUS NOT OKAY %d\n", __func__, rdata[3]);
		return -1;
	}

	return 0;
}

ssize_t get_emmc_health(struct mmc_host *host, char *buf)
{
	SDMMC_DEVICE_HEALTH	dh;
	unsigned char		*dat;
	ssize_t  size = 0;

	if (host == NULL) {
		pr_err("%s host is NULL", __func__);
		return -EINVAL;
	}

	dat = kzalloc(eMMC_HEALTH_BUFFER_SZ, GFP_KERNEL);

	if (dat == NULL) {
		pr_err("%s unable to get memory for buffer", __func__);
		return -ENOMEM;
	}

	memset(dat, 0, eMMC_HEALTH_BUFFER_SZ);

	switch( host->card->cid.manfid) {
		case MID_MMC_TOSHIBA:
			size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"TOSHIBA HEALTH:\n");
			if ((toshiba_gen_cmd(host, TOSHIBA_DEVICE_HEALTH, TOSHIBA_DEVICE_HEALTH_PWD, dat)) == 0) {
				dh.toshiba.lifetime_total	=	dat[0x04];
				dh.toshiba.lifetime_rsvd_blk	=	dat[0x05];
				dh.toshiba.lifetime_avg_pe	=	dat[0x06];

				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"LIFE TIME TOTAL %d\n", dh.toshiba.lifetime_total);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"RESERVED BLOCKS %d\n", dh.toshiba.lifetime_rsvd_blk);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"AVERAGE PE %d\n", dh.toshiba.lifetime_avg_pe);
			}

			if ((toshiba_gen_cmd(host, TOSHIBA_WRITE_ERASE_COUNT, TOSHIBA_WRITE_ERASE_COUNT_PWD, dat)) == 0) {
				dh.toshiba.slc_max_pe		=	be_to_le(le32toh( &dat[0x0c] ));
				dh.toshiba.slc_avg_pe		=	be_to_le(le32toh( &dat[0x10] ));
				dh.toshiba.mlc_max_pe		=	be_to_le(le32toh( &dat[0x04] ));
				dh.toshiba.mlc_avg_pe		=	be_to_le(le32toh( &dat[0x08] ));

				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"SLC MAX PE %d\n", dh.toshiba.slc_max_pe);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"SLC AVERAGE PE %d\n", dh.toshiba.slc_avg_pe);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"MLC MAX PE %d\n", dh.toshiba.mlc_max_pe);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"MLC AVERAGE PE %d\n", dh.toshiba.mlc_avg_pe);
			}
			break;

		case MID_MMC_SANDISK:
		case MID_MMC_SANDISK_2:
			size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"SANDISK HEALTH:\n");
			if (mmc_send_ext_csd(host->card, dat) == 0)
			{
				dh.sandisk.lifetime		=	dat[ECSD_SANDISK_LIFETIME];
				dh.sandisk.nv_avg_pe		=	dat[ECSD_SANDISK_NV_HEALTH];
				dh.sandisk.eua_avg_pe		=	dat[ECSD_SANDISK_EUA_HEALTH];
				dh.sandisk.mlc_avg_pe		=	dat[ECSD_SANDISK_MLC_HEALTH];

				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"LIFE TIME %d\n", dh.sandisk.lifetime);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"NV AVERAGE PE %d\n", dh.sandisk.nv_avg_pe);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"EUA AVERAGE PE %d\n", dh.sandisk.eua_avg_pe);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"MLC AVERAGE PE %d\n", dh.sandisk.mlc_avg_pe);
			}
			break;

		case MID_MMC_SAMSUNG:
			size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"SAMSUNG:\n");
			if ((samsung_smart_report(host, dat)) == 0) {
				dh.samsung.num_banks		=	le32toh( &dat[16] );
				dh.samsung.bank0_rsvd_blocks	=	le32toh( &dat[28] );
				dh.samsung.bank1_rsvd_blocks	=	le32toh( &dat[40] );
				dh.samsung.bank2_rsvd_blocks	=	le32toh( &dat[52] );
				dh.samsung.bank3_rsvd_blocks	=	le32toh( &dat[64] );

				dh.samsung.init_bad_blocks	=	le32toh( &dat[20] ) +
									le32toh( &dat[32] ) +
									le32toh( &dat[44] ) +
									le32toh( &dat[56] );

				dh.samsung.runtime_bad_blocks	=	le32toh( &dat[24] ) +
									le32toh( &dat[36] ) +
									le32toh( &dat[48] ) +
									le32toh( &dat[60] );

				dh.samsung.slc_max_ec		=	le32toh( &dat[120] );
				dh.samsung.slc_min_ec		=	le32toh( &dat[124] );
				dh.samsung.slc_avg_ec		=	le32toh( &dat[128] );
				dh.samsung.mlc_max_ec		=	le32toh( &dat[132] );
				dh.samsung.mlc_min_ec		=	le32toh( &dat[136] );
				dh.samsung.mlc_avg_ec		=	le32toh( &dat[140] );
				dh.samsung.max_ec		=	le32toh( &dat[68] );
				dh.samsung.min_ec		=	le32toh( &dat[72] );
				dh.samsung.avg_ec		=	le32toh( &dat[76] );
				dh.samsung.read_reclaim		=	le32toh( &dat[80] );

				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"NUMBER OF BANKS %d\n", dh.samsung.num_banks);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"BANK0 RESERVED BLOCKS %d\n", dh.samsung.bank0_rsvd_blocks);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"BANK1 RESERVED BLOCKS %d\n", dh.samsung.bank1_rsvd_blocks);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"BANK2 RESERVED BLOCKS %d\n", dh.samsung.bank2_rsvd_blocks);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"BANK3 RESERVED BLOCKS %d\n", dh.samsung.bank3_rsvd_blocks);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"INIT BAD BLOCKS %d\n", dh.samsung.init_bad_blocks);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"RUNTIME BAD BLOCKS %d\n", dh.samsung.runtime_bad_blocks);

				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"SLC MAX EC %d\n", dh.samsung.slc_max_ec);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"SLC MIN EC %d\n", dh.samsung.slc_min_ec);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"SLC AVERAGE EC %d\n", dh.samsung.slc_avg_ec);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"MLC MAX EC %d\n", dh.samsung.mlc_max_ec);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"MLC MIN EC %d\n", dh.samsung.mlc_min_ec);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"MLC AVERAGE EC %d\n", dh.samsung.mlc_avg_ec);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"MAX EC %d\n", dh.samsung.max_ec);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"MIN EC %d\n", dh.samsung.min_ec);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"AVERAGE EC %d\n", dh.samsung.avg_ec);
				size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"READ RECLAIM %d\n", dh.samsung.read_reclaim);

				if (dh.samsung.num_banks > 4) {
					dh.samsung.bank4_rsvd_blocks	=	le32toh( &dat[360] );
					dh.samsung.bank5_rsvd_blocks	=	le32toh( &dat[372] );
					dh.samsung.bank6_rsvd_blocks	=	le32toh( &dat[384] );
					dh.samsung.bank7_rsvd_blocks	=	le32toh( &dat[396] );
					dh.samsung.init_bad_blocks	+=	le32toh( &dat[352] ) +
										le32toh( &dat[364] ) +
										le32toh( &dat[376] ) +
										le32toh( &dat[388] );

					dh.samsung.runtime_bad_blocks	+=	le32toh( &dat[356] ) +
										le32toh( &dat[368] ) +
										le32toh( &dat[380] ) +
										le32toh( &dat[392] );

					size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"BANK4 RESERVED BLOCKS %d\n", dh.samsung.bank4_rsvd_blocks);
					size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"BANK5 RESERVED BLOCKS %d\n", dh.samsung.bank5_rsvd_blocks);
					size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"BANK6 RESERVED BLOCKS %d\n", dh.samsung.bank6_rsvd_blocks);
					size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"BANK7 RESERVED BLOCKS %d\n", dh.samsung.bank7_rsvd_blocks);
					size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"INIT BAD BLOCKS %d\n", dh.samsung.init_bad_blocks);
					size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"RUNTIME BAD BLOCKS %d\n", dh.samsung.runtime_bad_blocks);
				}
			}
			break;

		default:
			size += snprintf(buf+strlen(buf), PAGE_SIZE - strlen(buf),"un-supported eMMC\n");
			break;
	}

	if (dat != NULL)
		kfree(dat);
	return size;
}
EXPORT_SYMBOL(get_emmc_health);
#endif