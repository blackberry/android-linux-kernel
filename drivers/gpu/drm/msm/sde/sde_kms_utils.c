/* Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	"sde-kms_utils:[%s] " fmt, __func__

#include "sde_kms.h"

void sde_kms_info_reset(struct sde_kms_info *info)
{
	if (info) {
		info->len = 0;
		info->staged_len = 0;
	}
}

void sde_kms_info_add_keyint(struct sde_kms_info *info,
		const char *key,
		int32_t value)
{
	uint32_t len;

	if (info && key) {
		len = snprintf(info->data + info->len,
				SDE_KMS_INFO_MAX_SIZE - info->len,
				"%s=%d\n",
				key,
				value);

		/* check if snprintf truncated the string */
		if ((info->len + len) < SDE_KMS_INFO_MAX_SIZE)
			info->len += len;
	}
}

void sde_kms_info_add_keystr(struct sde_kms_info *info,
		const char *key,
		const char *value)
{
	uint32_t len;

	if (info && key && value) {
		len = snprintf(info->data + info->len,
				SDE_KMS_INFO_MAX_SIZE - info->len,
				"%s=%s\n",
				key,
				value);

		/* check if snprintf truncated the string */
		if ((info->len + len) < SDE_KMS_INFO_MAX_SIZE)
			info->len += len;
	}
}

void sde_kms_info_start(struct sde_kms_info *info,
		const char *key)
{
	uint32_t len;

	if (info && key) {
		len = snprintf(info->data + info->len,
				SDE_KMS_INFO_MAX_SIZE - info->len,
				"%s=",
				key);

		info->start = true;

		/* check if snprintf truncated the string */
		if ((info->len + len) < SDE_KMS_INFO_MAX_SIZE)
			info->staged_len = info->len + len;
	}
}

void sde_kms_info_append(struct sde_kms_info *info,
		const char *str)
{
	uint32_t len;

	if (info) {
		len = snprintf(info->data + info->staged_len,
				SDE_KMS_INFO_MAX_SIZE - info->staged_len,
				"%s",
				str);

		/* check if snprintf truncated the string */
		if ((info->staged_len + len) < SDE_KMS_INFO_MAX_SIZE) {
			info->staged_len += len;
			info->start = false;
		}
	}
}

void sde_kms_info_append_format(struct sde_kms_info *info,
		uint32_t pixel_format,
		uint64_t modifier)
{
	uint32_t len;

	if (!info)
		return;

	if (modifier) {
		len = snprintf(info->data + info->staged_len,
				SDE_KMS_INFO_MAX_SIZE - info->staged_len,
				info->start ?
				"%c%c%c%c/%llX/%llX" : " %c%c%c%c/%llX/%llX",
				(pixel_format >> 0) & 0xFF,
				(pixel_format >> 8) & 0xFF,
				(pixel_format >> 16) & 0xFF,
				(pixel_format >> 24) & 0xFF,
				(modifier >> 56) & 0xFF,
				modifier & ((1ULL << 56) - 1));
	} else {
		len = snprintf(info->data + info->staged_len,
				SDE_KMS_INFO_MAX_SIZE - info->staged_len,
				info->start ?
				"%c%c%c%c" : " %c%c%c%c",
				(pixel_format >> 0) & 0xFF,
				(pixel_format >> 8) & 0xFF,
				(pixel_format >> 16) & 0xFF,
				(pixel_format >> 24) & 0xFF);
	}

	/* check if snprintf truncated the string */
	if ((info->staged_len + len) < SDE_KMS_INFO_MAX_SIZE) {
		info->staged_len += len;
		info->start = false;
	}
}

void sde_kms_info_stop(struct sde_kms_info *info)
{
	uint32_t len;

	if (info) {
		/* insert final delimiter */
		len = snprintf(info->data + info->staged_len,
				SDE_KMS_INFO_MAX_SIZE - info->staged_len,
				"\n");

		/* check if snprintf truncated the string */
		if ((info->staged_len + len) < SDE_KMS_INFO_MAX_SIZE)
			info->len = info->staged_len + len;
	}
}
