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

#ifndef __LOGGER_H__
#define __LOGGER_H__

#define WLAN_HEXDUMP_ASCII_INFO(buf, len)     wlan_hexdump_ascii(buf, len, DHD_INFO_VAL)
#define WLAN_HEXDUMP_ASCII_ERROR(buf, len)     wlan_hexdump_ascii(buf, len, DHD_ERROR_VAL)

int wlan_hexdump(char *buf, int len);
int wlan_hexdump_ascii(char *buf, int len, int log_level);


#endif /* __LOGGER_H__ */
