/*
 * Copyright (C) 2016 BlackBerry Limited
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

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/io.h>
#include "devices_bbry.h"

static struct _bbry_board {
	u32 hwid;
	u32 rev;
	char product[64];
} bbry_board = {
	.hwid = 0xFFFFFFFF,
	.product = "Unknown\n",
};

static int __init bbry_board_data(const char *arg, int size, void *field)
{
	switch (size) {
	case 1:
		return kstrtou8(arg, 0, (u8 *)field);
	case 4:
		return kstrtou32(arg, 0, (u32 *)field);
	case 8:
		return kstrtou64(arg, 0, (u64 *)field);
	default:
		break;
	}
	return -EINVAL;
}

#define BBRY_CMDLINE_DATA(_str, _type, _field) \
static int __init bbry_board_##_field(char *arg) \
{ \
	return bbry_board_data(arg, sizeof(bbry_board._field),\
				&bbry_board._field); \
} \
__setup(_str, bbry_board_##_field); \
_type bbry_get_board_##_field(void) \
{ \
	return bbry_board._field; \
}

#define BBRY_CMDLINE_DATA_STRING(_str, _field) \
static int __init bbry_board_##_field(char *arg) \
{ \
	if (strlcpy(bbry_board._field, arg, sizeof(bbry_board._field)) \
			>= sizeof(bbry_board._field)) \
		return -EINVAL; \
	return 0; \
} \
__setup(_str, bbry_board_##_field); \
const char * bbry_get_board_##_field(void) \
{ \
	return bbry_board._field; \
}

BBRY_CMDLINE_DATA("androidboot.binfo.rev=", u32,  rev);
BBRY_CMDLINE_DATA("androidboot.binfo.hwid=", u32, hwid);
BBRY_CMDLINE_DATA_STRING("androidboot.binfo.product=", product);
