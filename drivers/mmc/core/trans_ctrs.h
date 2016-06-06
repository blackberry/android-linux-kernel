/*
 * Copyright (C) 2016 BlackBerry Limited. All rights reserved.
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
#ifndef __TRANS_CTRS_H__
#define __TRANS_CTRS_H__

#include <linux/mmc/host.h>

void mmc_init_trans_ctrs(struct mmc_host *host);
int mmc_get_trans_ctrs(struct mmc_host *host, char *buf, int max_buf_len);

#endif /* __TRANS_CTRS_H__ */
