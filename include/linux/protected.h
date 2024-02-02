/*
 * protected.h - Functions for dealing with protected kernel variables
 *
 * Copyright (C) 2017 Glenn Wurster <gwurster@blackberry.com>
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#ifndef _LINUX_PROTECTED_H
#define _LINUX_PROTECTED_H

#ifndef __ASSEMBLY__

#ifdef CONFIG_PROTECTED_VARS

#ifdef CONFIG_PROTECTED_VARS_CONST
#define __ro_protected const __attribute__((__section__(".protected")))
#else
#define __ro_protected __attribute__((__section__(".protected")))
#endif

#define update_protected(target, source)						\
	({										\
		typeof(target) tmp = source;						\
		__update_protected( &target, &tmp, sizeof(target) );			\
	})

	int __update_protected(void *target, const void *src, unsigned length);

#else /* CONFIG_PROTECTED_VARS */

#define __ro_protected
#define update_protected(x, y) x = y

#endif /* CONFIG_PROTECTED_VARS */

#endif /* __ASSEMBLY__ */

#endif /* _LINUX_PROTECTED_H */
