/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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

#ifndef _EFSRECOVERY_HEADER
#define _EFSRECOVERY_HEADER

struct device;

#ifdef CONFIG_TCT_EFS_RECOVERY
extern void destroy_efsrecovery_device(void *dev);
extern int do_efsrecovery(void);
extern void efsrecovery_check(char * reason);
#else
static inline void destroy_efsrecovery_device(void *dev)
{
}

static inline int do_efsrecovery(void)
{
	return -ENODEV;
}

static inline void efsrecovery_check(char * reason)
{

}
#endif /* CONFIG_TCT_EFS_RECOVERY */

#endif
