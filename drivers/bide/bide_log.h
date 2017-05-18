/*
 * Copyright (C) 2014 BlackBerry Limited
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

#ifndef __LOG_H__
#define __LOG_H__

#define logError(x, ...) printk(KERN_ERR     "[BIDE] ERR: " x "\n", ##__VA_ARGS__)
#define logWarn(x, ...)  printk(KERN_WARNING "[BIDE] WRN: " x "\n", ##__VA_ARGS__)
#define logInfo(x, ...)  printk(KERN_INFO    "[BIDE] INF: " x "\n", ##__VA_ARGS__)
#define logDebug(x, ...) printk(KERN_DEBUG   "[BIDE] DBG: " x "\n", ##__VA_ARGS__)

#endif
