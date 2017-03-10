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
#include "sdmmc_assd.h"

#define SDMMC_READ_SEC_CMD			34
#define SDMMC_WRITE_SEC_CMD			35
#define SDMMC_SEND_PSI				36
	#define SDMMC_PSI_ASSD_SR		0	/* ASSD Status Register*/
	#define SDMMC_PSI_ASSD_PR		4	/* ASSD Properties Register*/
	#define SDMMC_PSI_ASSD_RNR		6	/* ASSD Random Number Register*/
	#define SDMMC_PSI_SIZE			32

#define SDMMC_CONTROL_ASSD_SYSTEM		37
#define SDMMC_DIRECT_SECURE_READ		50
#define SDMMC_DIRECT_SECURE_WRITE		57

#define SDMMC_SF_MODE_SET			0x1
#define SDMMC_SF_MODE_CHECK			0x0
#define SDMMC_SF_GRP_DFLT			0x0
#define SDMMC_SF_GRP_BUS_SPD			0x0		/* Bus Speed*/
#define SDMMC_SF_GRP_CMD_EXT			0x1		/* Command System Extension*/
#define SDMMC_SF_GRP_DRV_STR			0x2		/* Driver Strength*/
#define SDMMC_SF_GRP_CUR_LMT			0x3		/* Current Limit*/
#define SDMMC_SF_CUR_FCN			0xF
#define SDMMC_SF_STATUS_SIZE			64

#define SDMMC_CMD_SYS_EC			(1 << 1)	/* eCommerce*/
#define SDMMC_CMD_SYS_OTP			(1 << 3)
#define SDMMC_CMD_SYS_ASSD			(1 << 4)

#define SDMMC_MIN_BUFFER			512
#define SDMMC_MAX_BUFFER			(64*1024)

typedef struct _sdmmc_assd_status {
#define ASSD_STATE_IDL				0
#define ASSD_STATE_SCP				1		/* Secure Command in Progress */
#define ASSD_STATE_SCC				2		/* Secure Command Complete */
#define ASSD_STATE_SCA				3		/* Secure Command Aborted */
	__u8		assd_state;

#define ASSD_ERR_STATE_NE			0		/* No Error */
#define ASSD_ERR_STATE_AE			1		/* Auth Error */
#define ASSD_ERR_STATE_ANF			2		/* Area Not Found */
#define ASSD_ERR_STATE_RO			3		/* Range Over */
#define ASSD_ERR_STATE_CE			4		/* Condition Error */
	__u8		assd_err_state;
	__u8		assd_sec_sys_err;
	__u8		pmem_state;
	__u8		auth_alg;
	__u8		enc_alg;
	__u8		active_sec_system;
	__u8		sec_token_prot;
	__u16		read_block_count;
	__u16		suspended_sec_sys;
	__u32		rsvd[6];
} SDMMC_ASSD_STATUS;

typedef struct _sdmmc_assd_properties {
	__u8		assd_version;
	__u8		assd_sec_sys_vendor_id;
	__u16		assd_sec_sys;

	__u16		suspendible_sec_sys;
	__u16		sup_auth_alg;
	__u16		sup_enc_alg;
	__u16		cl_support;

	__u8		sec_read_latency;		/* 250ms units */
	__u8		sec_write_latency;		/* 250ms units */
	__u8		wr_sec_bus_busy;		/* 250ms units */
	__u8		ctrl_sys_bus_busy;		/* 250ms units */

	__u8		pmem_support;
	__u8		pmem_rd_time;			/* 100ms units */
	__u8		pmem_wr_time;			/* 250ms units */

	__u8		rsvd[17];
} SDMMC_ASSD_PROPERTIES;

#define APDU_READ				0
#define APDU_WRITE				1
#define ASSD_TIME_DEFAULT			200
#define ASSD_RETRY_COUNT			5
#define ASSD_SWITCH_RETRY			200
#define ASSD_APDU_STL_SIZE			2
#define ASSD_APDU_MIN_DLEN			4
#define ASSD_APDU_BLK_SZ			512

#define ASSD_STATUS_SIZE			32
#define ASSD_STATE_IDX				0
#define ASSD_ERROR_IDX				1
#define ASSD_SEC_SYS_ERROR_IDX			2
	#define ASSD_SEC_SYS_ERR		0x80

#define ASSD_SPEC_A1_V1				0
#define ASSD_SPEC_A1_V1_1			1
#define ASSD_SPEC_A1_V2				2
#define ASSD_SPEC_A1_V3				3

#define ASSD_SS_MCEX				0

static struct semaphore				sdmmc_assd_sem;
static s32					sec_sys_idx = -1;
static u32					sd_assd_init;
static SDMMC_ASSD_PROPERTIES			prop;
static SDMMC_ASSD_STATUS			as;

extern s32 mmc_sd_switch(struct mmc_card *card, s32 mode, s32 group, u8 value,
			 u8 *resp);
/* There is no return or error check for set block as it might fail but data read/ write might still work */
static void sdmmc_assd_mmc_set_blocklen(struct mmc_card *card, u32 blocklen)
{
	s32			err = 0;
	struct mmc_command	cmd = {0};

	cmd.opcode = MMC_SET_BLOCKLEN;
	cmd.arg = blocklen;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_AC;
	err = mmc_wait_for_cmd(card->host, &cmd, 5);
	if (err)
		pr_debug("%s to %d failed (%d)\n", __func__, blocklen, err);
	return;
}

static s32 sdmmc_assd_control_system(struct mmc_host *host, u32 srcid, u32 ssi, u32 op)
{
	s32			err = 0;
	struct mmc_command	cmd = {0};

	cmd.opcode = SDMMC_CONTROL_ASSD_SYSTEM;
	cmd.arg    = (srcid << 12) | (ssi << 8) | op;
	cmd.flags  = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
	err = mmc_wait_for_cmd(host, &cmd, 5);
	if (err)
		pr_debug("%s returns error (%d)\n", __func__, err);
	return err;
}

static s32 sdmmc_assd_setup_mrq(struct mmc_card *card,
	struct mmc_request *mrq, struct scatterlist *sg, u16 sg_len,
	u32 blocks, u32 blksz, u32 write)
{
	if (!sg || !mrq || !mrq->cmd || !mrq->data)
		return -EINVAL;

	mrq->cmd->opcode = write ?
			SDMMC_WRITE_SEC_CMD : SDMMC_READ_SEC_CMD;
	mrq->cmd->arg = (blocks & 0x0000FFFF);

	mrq->cmd->flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	if (mrq->stop != NULL) {
		mrq->stop->opcode = MMC_STOP_TRANSMISSION;
		mrq->stop->arg = 0;
		mrq->stop->flags = MMC_RSP_SPI_R1B | MMC_RSP_R1B | MMC_CMD_AC;
	}
	sdmmc_assd_mmc_set_blocklen(card, blksz);
	mrq->data->blksz = blksz;
	mrq->data->blocks = blocks;
	mrq->data->flags = write ? MMC_DATA_WRITE : MMC_DATA_READ;
	mrq->data->sg = sg;
	mrq->data->sg_len = sg_len;

	if ((write ? prop.sec_write_latency : prop.sec_read_latency)) {
		mrq->data->timeout_ns   = (write ? prop.sec_write_latency : prop.sec_read_latency) * 250 * 1000 * 1000;/* properties are in 250ms unit */
		mrq->data->timeout_clks = 0;
	} else
		mmc_set_data_timeout(mrq->data, card);
	return 0;
}

static s32 sdmmc_assd_sec_read_write(struct mmc_card *card,
	u8 *buffer, u32 blocks, u32 blksz, u32 write)
{
	s32			err = 0;
	struct mmc_request	mrq = {0};
	struct mmc_command	cmd = {0};
	struct mmc_command	stop = {0};
	struct mmc_data		data = {0};
	struct scatterlist	sg;
	if (buffer == NULL)
		return -EINVAL;

	mrq.cmd = &cmd;
	mrq.data = &data;
	if (blocks > 1)
		mrq.stop = &stop;
	sg_init_one(&sg, buffer, blksz);

	err = sdmmc_assd_setup_mrq(card, &mrq, &sg, 1, blocks, blksz, write);
	if (err)
		goto out;
	mmc_wait_for_req(card->host, &mrq);

	if (cmd.error) {
		err = cmd.error;
		goto out;
	}

	if (data.error) {
		err = data.error;
		goto out;
	}

	if (write)
		err = mmc_card_is_prog_state(card);
out:
	if (err)
		pr_err("%s returns error (%d) RW %d\n", __func__, err, write);
	return err;
}

static s32 sdmmc_assd_send_psi(struct mmc_host *host, u32 rid, u8 *psi)
{
	struct mmc_request	req;
	struct mmc_command	cmd;
	struct mmc_data		dat;
	struct scatterlist	sg;

	sdmmc_assd_mmc_set_blocklen(host->card, SDMMC_PSI_SIZE);

	memset(&req, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&dat, 0, sizeof(struct mmc_data));
	req.cmd    = &cmd;
	req.data   = &dat;
	cmd.opcode = SDMMC_SEND_PSI;
	cmd.arg    = rid;
	cmd.flags  = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;
	dat.blksz  = SDMMC_PSI_SIZE;
	dat.blocks = 1;
	dat.flags  = MMC_DATA_READ;
	dat.sg     = &sg;
	dat.sg_len = 1;
	sg_init_one(&sg, psi, SDMMC_PSI_SIZE);

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
	return 0;
}

static s32 sdmmc_assd_status(struct mmc_host *host, SDMMC_ASSD_STATUS *as, u8 __user *buf)
{
	s32		err = 0;
	u8		*psi;

	psi = kzalloc(max(SDMMC_MIN_BUFFER, ASSD_STATUS_SIZE), GFP_KERNEL);
	if (psi == NULL)
		return -ENOMEM;

	err = sdmmc_assd_send_psi(host, SDMMC_PSI_ASSD_SR, psi);
	if (!err) {
		as->assd_state		= psi[0];
		as->assd_err_state	= psi[1];
		as->assd_sec_sys_err	= psi[2] >> 7;
		as->pmem_state		= psi[3];
		as->auth_alg		= psi[4];
		as->enc_alg		= psi[5];
		as->active_sec_system	= psi[6];
		as->sec_token_prot	= psi[7];
		as->read_block_count	= (psi[8]  << 8) | psi[9];
		as->suspended_sec_sys	= (psi[10] << 8) | psi[11];
		if (buf !=  NULL)
			err = copy_to_user(buf, psi, ASSD_STATUS_SIZE);/* copy if ioctl requested */
	}
	kfree(psi);
	if (err)
		pr_debug("%s returns error (%d)\n", __func__, err);
	return err;
}

static s32 sdmmc_assd_properties(struct mmc_host *host, SDMMC_ASSD_PROPERTIES *prop, u8 __user *buf)
{
	s32		err = 0;
	u8		*psi;

	psi = kzalloc(max(SDMMC_MIN_BUFFER, ASSD_STATUS_SIZE), GFP_KERNEL);
	if (psi == NULL)
		return -ENOMEM;

	err = sdmmc_assd_send_psi(host, SDMMC_PSI_ASSD_PR, psi);
	if (!err) {
		prop->sec_read_latency		= psi[0];
		prop->sec_write_latency		= psi[1];
		prop->assd_version		= psi[2];
		prop->cl_support		= (psi[3] << 7) | (psi[4] >> 1);
		prop->pmem_support		= psi[4] & 0x1;
		prop->pmem_rd_time		= psi[5];
		prop->pmem_wr_time		= psi[6];
		prop->wr_sec_bus_busy		= psi[7];
		prop->sup_auth_alg		= (psi[8]  << 8) | psi[9];
		prop->sup_enc_alg		= (psi[10] << 8) | psi[11];
		prop->assd_sec_sys		= (psi[12] << 8) | psi[13];
		prop->assd_sec_sys_vendor_id	= psi[14];
		prop->ctrl_sys_bus_busy		= psi[15];
		prop->suspendible_sec_sys	= (psi[16] << 8) | psi[17];
		if (buf !=  NULL)
			err = copy_to_user(buf, psi, ASSD_STATUS_SIZE);/* copy if ioctl requested*/
	}
	kfree(psi);
	if (err)
		pr_debug("%s returns error (%d)\n", __func__, err);
	return err;
}

static s32 sdmmc_assd_wait_scc(struct mmc_host *host, u32 state)
{
	s32		err = 0;
	u32		msec;
	u8		*psi;

	psi = kzalloc(max(SDMMC_MIN_BUFFER, ASSD_STATUS_SIZE), GFP_KERNEL);
	if (psi == NULL)
		return -ENOMEM;

	for (msec = 0; msec < ASSD_TIME_DEFAULT; ++msec) {
		msleep_interruptible(10);
		err = sdmmc_assd_send_psi(host, SDMMC_PSI_ASSD_SR, psi);
		if (err)
			continue;

		if (state == ASSD_STATE_SCC) {
			if (psi[ASSD_STATE_IDX] == ASSD_STATE_SCA) {
				err = -EIO; break;
			}

			if ((psi[ASSD_SEC_SYS_ERROR_IDX] & ASSD_SEC_SYS_ERR)) {
				err = -EIO; break;
			}
		}

		if ((psi[ASSD_STATE_IDX] == state) || (psi[ASSD_STATE_IDX] == ASSD_STATE_IDL)) {/* if card is idle or asked for state then exit */
			err = 0;
			break;
		} else if (msec > (ASSD_TIME_DEFAULT - 10)) {/* 10 times reset and try to recover*/
			err = sdmmc_assd_control_system(host, 0, ((prop.assd_version > 1) ? sec_sys_idx : 0), 0x1);
			state = ASSD_STATE_IDL;
		}
	}

	if (msec >= ASSD_TIME_DEFAULT)
		err = -ETIMEDOUT;
	if (err) {
		sdmmc_assd_control_system(host, 0, ((prop.assd_version > 1) ? sec_sys_idx : 0), 0x1);
		pr_err("%s: (%d), msec %d, state %d, error %d, sec sys err %d\n",
			__func__, err, msec, psi[ASSD_STATE_IDX],
			psi[ASSD_ERROR_IDX], psi[ASSD_SEC_SYS_ERROR_IDX]);
	}
	kfree(psi);
	return err;
}

static s32 sdmmc_assd_cmd_sys(struct mmc_host *host)
{
	s32		err = 0;
	u32		idx = 0;
	u32		retry = 0;
	u8		*ss;

	ss = kzalloc(max(SDMMC_MIN_BUFFER, SDMMC_SF_STATUS_SIZE), GFP_KERNEL);
	if (ss == NULL)
		return -ENOMEM;
	do {
		err = mmc_sd_switch(host->card, SDMMC_SF_MODE_CHECK, SDMMC_SF_GRP_CMD_EXT, SDMMC_SF_CUR_FCN, ss);
		if (err)
			continue;

		if (!((ss[11] & SDMMC_CMD_SYS_EC) || (ss[11] & SDMMC_CMD_SYS_ASSD))) {
			err = -ENODEV;
			continue;
		}

		idx = ffs(ss[11] & (SDMMC_CMD_SYS_EC | SDMMC_CMD_SYS_ASSD)) - 1;

		err = mmc_sd_switch(host->card, SDMMC_SF_MODE_SET, SDMMC_SF_GRP_CMD_EXT, idx, ss);
		if (!err) {
			if (((ss[16] >> 4) & 0x0F) != idx) {
				err = -EIO;
				continue;
			}
			if (((ss[17] == 0x01) && (ss[27] & (1 << idx)))) {
				err = -EBUSY;
				continue;
			}
			break;
		}
		msleep_interruptible(10);
	} while (++retry < ASSD_SWITCH_RETRY);/* try switching to security system multiple times */

	kfree(ss);
	if (err)
		pr_err("%s returns error (%d)\n", __func__, err);
	return err;
}

static s32 sdmmc_assd_state(struct mmc_host *host)
{
	s32			err = 0;
	u32			retry = ASSD_RETRY_COUNT;
	SDMMC_ASSD_STATUS	as;

	do {
		err = sdmmc_assd_cmd_sys(host);
		if (err) {
			pr_debug("%s Unable to switch (%d)\n", __func__, err);
			continue;
		}
		err = sdmmc_assd_properties(host, &prop, NULL);
		if (err) {
			pr_debug("%s Unable to read properties (%d)\n", __func__, err);
			continue;
		}

		/* If we are version 1.1 or lower, there is no need to
		   send CMD37 as there is only a single security system. */
		if (prop.assd_version <= ASSD_SPEC_A1_V1_1)
			break;

		err = sdmmc_assd_control_system(host, 0, ((prop.assd_version > 1) ? sec_sys_idx : 0), 0x1);
		if (!err) {
			err = sdmmc_assd_status(host, &as, NULL);
			if (!err) {
				if ((sec_sys_idx != as.active_sec_system) && (prop.assd_version > 1)) {/* check if desired security system is active */
					err = -EIO;
					sec_sys_idx = -1;
				} else
					break;
			}
		}
	} while (--retry);

	if (err)
		pr_err("%s returns error (%d) on ssi %d active ss %d assd version %d sec sys %d\n",
			__func__, err, sec_sys_idx, as.active_sec_system, prop.assd_version, prop.assd_sec_sys);
	return err;
}

static s32 sdmmc_assd_get_card_info(struct mmc_host *host, u8 __user *buf)
{
	s32			err = 0;
	u32			*raw_cid;
	u32			raw_cid_element, i, j;
	u8			cid[4 * sizeof(u32)];

	if (!buf)
		return -EINVAL;
	raw_cid = host->card->raw_cid;
	for (i = 0; i < 4; i++) {
		raw_cid_element = raw_cid[i];
		for (j = 0; j < sizeof(u32); j++)
			cid[i * sizeof(u32) + j] = 0xFF & (raw_cid_element >> (24 - (8 * j)));
	}
	err = copy_to_user(buf, cid, sizeof(cid));
	if (err)
		pr_err("%s returns error (%d)\n", __func__, err);
	return err;
}

int sdmmc_assd_ioctl(struct mmc_host *host, unsigned int cmd, unsigned long arg)
{
	s32				err = -EINVAL;
	u32				len, blk_alloc, blocks;
	u8				*data = NULL;
	u8				apdu_sz[2];

	if ((host == NULL) || (host->card == NULL))
		return -EINVAL;
	if ((!sd_assd_init) && (cmd != SDMMC_ASSD_INIT))
		return -ENODEV;
	if (!sd_assd_init)
		sema_init(&sdmmc_assd_sem, 1);
	if (down_trylock(&sdmmc_assd_sem))
		return -EBUSY;

	switch (cmd) {

	case SDMMC_ASSD_INIT:
		err = copy_from_user((u8 *)&sec_sys_idx, (u8 __user *)arg, sizeof(sec_sys_idx));
		if (err)
			break;
		err = sdmmc_assd_state(host);
		if (err)
			break;
		sd_assd_init = 1;
		break;
	case SDMMC_ASSD_CARD_STATUS:
		err = sdmmc_assd_state(host);
		if (!err)
			err = sdmmc_assd_status(host, &as, (u8 *)arg);
		break;
	case SDMMC_ASSD_CARD_PROPERTIES:
		err = sdmmc_assd_state(host);
		if (!err)
			err = sdmmc_assd_properties(host, &prop, (u8 *)arg);
		break;
	case SDMMC_ASSD_GET_CARD_INFO:
		err = sdmmc_assd_get_card_info(host, (u8 *)arg);
		break;
	case SDMMC_ASSD_APDU:
		err = copy_from_user(apdu_sz, (u8 *)arg, 2);
		if (err)
			break;
		len = (u32)((apdu_sz[0] << 8) | apdu_sz[1]);/* read APDU length from header first and then allocate memory */
		if ((len < ASSD_APDU_MIN_DLEN) || (len > SDMMC_MAX_BUFFER)) {
			err = -EMSGSIZE;
			break;
		}
		if (len % ASSD_APDU_BLK_SZ)/* make transfer multiple of blocks */
			blk_alloc = len + (ASSD_APDU_BLK_SZ - (len & (ASSD_APDU_BLK_SZ - 1)));
		else
			blk_alloc = len;

		blocks = (blk_alloc / ASSD_APDU_BLK_SZ);
		data = kzalloc(blk_alloc, GFP_KERNEL);
		if (data == NULL) {
			err = -ENOMEM;
			break;
		}
		err = copy_from_user((u8 *)data, (u8 *)arg, len);
		if (err)
			break;
		err = sdmmc_assd_wait_scc(host, ASSD_STATE_IDL);/* wait for idle state before writing data */
		if (err)
			break;
		err = sdmmc_assd_sec_read_write(host->card, data, blocks, ASSD_APDU_BLK_SZ, APDU_WRITE);
		if (err)
			break;
		err = sdmmc_assd_wait_scc(host, ASSD_STATE_SCC);/* wait for write command to complete before reading */
		if (err)
			break;
		err = sdmmc_assd_sec_read_write(host->card, data, blocks, ASSD_APDU_BLK_SZ, APDU_READ);
		if (err)
			break;
		len = min((u32)((data[0] << 8) | data[1]), (u32)ASSD_APDU_BLK_SZ);
		err = copy_to_user((u8 *)arg, (u8 *)data, len);/* copy minimum of what was read or size of buffer that was sent from user space */
		break;
	default:
		err = -EINVAL;
		break;
	}

	if (data != NULL)
		kfree(data);
	up(&sdmmc_assd_sem);
	if (err)
		pr_err("%s CMD 0x%x ERR (%d)\n", __func__, cmd, err);
	return err;
}
EXPORT_SYMBOL(sdmmc_assd_ioctl);
