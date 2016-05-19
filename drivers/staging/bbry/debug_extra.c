/*
 *  drivers/staging/bbry/debug_extra.c
 *
 *  Copyright (c) 2016 BlackBerry
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/kernel.h>

static int debug_extra;
static int __init early_debug_extra(char *p)
{
#ifdef CONFIG_BBRY_DEBUG
	debug_extra = 1;
#else
	int tmp;
	if (get_option(&p, &tmp))
		debug_extra = !!tmp;
#endif
	return 0;
}
early_param("androidboot.debug_extra", early_debug_extra);

int bbry_get_debug_extra(void)
{
	return debug_extra;
}
