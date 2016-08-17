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

#include <linux/ctype.h>
#include <osl.h>
#include <dhd_dbg.h>
#include <logger.h>

/* max number of bytes to dump per line */
#define HEXDUMP_MAX_HEX_PER_LINE 16
/*max length of line in hex dump*/
#define HEXDUMP_MAX_LINE_LEN ((HEXDUMP_MAX_HEX_PER_LINE * 3) + 4)
/* max length of line in ascii hex dump */
#define HEXDUMP_ASCII_MAX_LINE_LEN ((HEXDUMP_MAX_LINE_LEN + HEXDUMP_MAX_HEX_PER_LINE) + 5)

int wlan_hexdump(char *buf, int buf_len)
{
	int line_count = 0;
	int line_pos = 0;
	int buf_pos = 0;
	char *hex_buf = 0;
	int hex_buf_len = 0;

	if (buf == 0) {
		DHD_ERROR(("wlan - %s: Invalid buffer. buf=NULL\n", __func__));
		return -EINVAL;
	}

	if (buf_len <= 0) {
		DHD_ERROR(("wlan - %s: Invalid buffer length. len=%d\n",
			__func__, buf_len));
		return -EINVAL;
	}

	hex_buf_len = (buf_len * 3) + ((buf_len / 16) + 2)  * 16;
	hex_buf = kmalloc(hex_buf_len, GFP_KERNEL);

	if (hex_buf == 0) {
		DHD_ERROR(("wlan - kmalloc() for hex_buf failed.\n"));
		return -ENOMEM;
	}

	/* Create output buffer in hex dump format */
	/* 0000: FF FF FF FF FF FF FF FF - FF FF FF FF FF FF FF FF */
	/* 0001: FF FF FF FF FF FF FF FF - FF FF FF FF FF FF FF FF */
	while (buf_pos < buf_len) {
		if (buf_pos % 16 == 0) {
			snprintf(&hex_buf[line_pos], (hex_buf_len - line_pos),
				"\nwlan - %04X: ", line_count);
			line_pos += 14;
			line_count += 16;
		} else if (buf_pos % 8 == 0) {
			snprintf(&hex_buf[line_pos], (hex_buf_len - line_pos), "- ");
			line_pos += 2;
		}
		snprintf(&hex_buf[line_pos], (hex_buf_len - line_pos),
			"%02hhX ", buf[buf_pos++]);
		line_pos += 3;
	}

	DHD_ERROR(("wlan - %s: %s\n", __func__, hex_buf));
	kfree(hex_buf);
	return 0;
}

int wlan_hexdump_ascii(char *buf, int buf_len, int log_level)
{
	int i = 0;
	int line_pos = 0;
	int buf_pos = 0;
	int num_out = 0;
	char hex_buf[HEXDUMP_ASCII_MAX_LINE_LEN] = {0};

	if (buf == 0) {
		if (log_level == DHD_INFO_VAL)
			DHD_INFO(("wlan - %s: Invalid buffer. buf=NULL\n", __func__));
		else
			DHD_ERROR(("wlan - %s: Invalid buffer. buf=NULL\n", __func__));
		return -EINVAL;
	}

	if (buf_len <= 0) {
		if (log_level == DHD_INFO_VAL)
			DHD_INFO(("wlan - %s: Invalid buffer length. len=%d\n", __func__, buf_len));
		else
			DHD_ERROR(("wlan - %s: Invalid buffer length. len=%d\n", __func__, buf_len));
		return -EINVAL;
	}

	/* Create output buffer in hex dump ascii format */
	/* FF FF FF FF FF FF FF FF - FF FF FF FF FF FF FF FF  --  AAAAAAAAAAAAAAA */
	while (buf_pos < buf_len) {
		line_pos = 0;
		num_out = 0;

		/* Output each byte in hex (zero pad until line is full) */
		for (i = 0; i < HEXDUMP_MAX_HEX_PER_LINE; ++i) {
			if (i == (HEXDUMP_MAX_HEX_PER_LINE / 2)) {
				hex_buf[line_pos++] = '-';
				hex_buf[line_pos++] = ' ';
			}

			if (buf_pos >= buf_len) {
				snprintf(&hex_buf[line_pos],
					(HEXDUMP_MAX_LINE_LEN - line_pos), "00 ");
			} else {
				snprintf(&hex_buf[line_pos], (HEXDUMP_MAX_LINE_LEN - line_pos),
					"%02hhX ", buf[buf_pos++]);
				++num_out;
			}

			line_pos += 3;
		}
	}

	snprintf(&hex_buf[line_pos], (HEXDUMP_ASCII_MAX_LINE_LEN - line_pos), " --  ");
	line_pos += 5;
	buf_pos -= num_out;

	/* output ASCII representation after hex_dump (non-printable ASCII char='_') */
	for (i = 0; i < HEXDUMP_MAX_HEX_PER_LINE; ++i) {
		if (buf_pos >= buf_len) {
			snprintf(&hex_buf[line_pos],
				(HEXDUMP_ASCII_MAX_LINE_LEN - line_pos), "_");
		} else {
			if (isprint(buf[buf_pos]))
				snprintf(&hex_buf[line_pos],
					(HEXDUMP_ASCII_MAX_LINE_LEN - line_pos),
					"%c ", buf[buf_pos]);
			else
				snprintf(&hex_buf[line_pos],
					(HEXDUMP_ASCII_MAX_LINE_LEN - line_pos), "_");

			++buf_pos;
		}
		++line_pos;
	}

	if (log_level == DHD_INFO_VAL)
		DHD_INFO(("wlan - %s: %s\n", __func__, hex_buf));
	else
		DHD_ERROR(("wlan - %s: %s\n", __func__, hex_buf));
	return 0;
}

