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

#ifndef MSM_CSID_BBRY_H
#define MSM_CSID_BBRY_H


typedef struct {
	uint16_t  offset;  /* Offset from base address of CSID hardware */
	uint32_t  value;   /* Value to read (or write)                  */
} csid_reg_t;

#define VIDIOC_MSM_CSID_READ_REGISTER \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 31, csid_reg_t)


#endif  // MSM_CSID_BBRY_H
