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

#include "trans_ctrs.h"
#include <linux/atomic.h>
#include <linux/mmc/host.h>

void mmc_init_trans_ctrs(struct mmc_host *host)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(host->part_ctrs); i++) {
		struct ctrs_part *part = &host->part_ctrs[i];

		atomic_long_set(&part->trim, 0);
		atomic_long_set(&part->discard, 0);
		atomic_long_set(&part->erase, 0);
		atomic_long_set(&part->write, 0);
		atomic_long_set(&part->read, 0);
	}

	atomic_long_set(&host->ctrs.pwr_irq_timeout, 0);
	atomic_long_set(&host->ctrs.cmd_request_timeout, 0);
	atomic_long_set(&host->ctrs.data_request_timeout, 0);
}

int mmc_get_trans_ctrs(struct mmc_host *host, char *buf, int max_buf_len)
{
	int i;
	int rc;
	int buf_i = 0;
	const char *part_names[] = {"USER", "BOOT0", "BOOT1", "RPMB"};
	int part_count;

	part_count = ARRAY_SIZE(part_names);

#define cat_long_to_buf(fmt, ...) { \
	rc = snprintf(buf+buf_i, max_buf_len, fmt ": %ld\n", ## __VA_ARGS__); \
	if (rc <= 0 || rc >= max_buf_len) { \
		return -ENOMEM; \
	} else { \
		buf_i += rc; \
		max_buf_len -= rc; \
	} \
}

#define cat_part_ctr(label, name, field) { \
	long temp_v = atomic_long_xchg(field, 0); \
	if (temp_v) \
		cat_long_to_buf("[%s]\t" label, name, temp_v); \
} \

#define cat_ctr(label, field) { \
	long temp_v = atomic_long_xchg(field, 0); \
	if (temp_v) \
		cat_long_to_buf(label, temp_v); \
} \


	for (i = 0; i < part_count; i++) {
		struct ctrs_part *ctrs;

		if (i >= ARRAY_SIZE(host->part_ctrs))
			break;

		ctrs = &host->part_ctrs[i];

		cat_part_ctr("trim", part_names[i], &ctrs->trim);
		cat_part_ctr("discard", part_names[i], &ctrs->discard);
		cat_part_ctr("erase", part_names[i], &ctrs->erase);
		cat_part_ctr("write", part_names[i], &ctrs->write);
		cat_part_ctr("read", part_names[i], &ctrs->read);
	}

	cat_ctr("pwr_irq_timeout", &host->ctrs.pwr_irq_timeout);
	cat_ctr("command_request_timeout", &host->ctrs.cmd_request_timeout);
	cat_ctr("data_request_timeout", &host->ctrs.data_request_timeout);

	if (!buf_i)
		buf_i = snprintf(buf, max_buf_len, "All counters zero\n");

	return buf_i;
}

