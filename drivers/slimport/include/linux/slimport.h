/*
 * Copyright (C) 2015 BlackBerry Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _SLIMPORT_H
#define _SLIMPORT_H

#if defined(CONFIG_SLIMPORT_ANX7808) && defined(CONFIG_BBRY)
bool slimport_is_connected(void);
uint32_t slimport_get_chg_current(void);
#else
#if defined(CONFIG_SLIMPORT_COLORADO3) && defined(CONFIG_BBRY)
bool slimport_dongle_is_connected(void);
uint32_t slimport_get_chg_current(void);
#else
static inline bool slimport_is_connected(void) { return false; }
static inline uint32_t slimport_get_chg_current(void) { return 0; }
#endif
#endif
#endif
