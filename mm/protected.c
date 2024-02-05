/*
 * protected.c - Functions for dealing with protected kernel variables
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

#include <linux/spinlock.h>
#include <linux/protected.h>
#include <linux/vmalloc.h>
#include <linux/mmzone.h>
#include <linux/module.h>

#include <asm/pgtable.h>
#include <asm/cpufeature.h>
#include <asm/sections.h>

int __update_protected(void *target, const void *src, unsigned length)
{
	/* First step - make sure that the variable we are being asked to
	 * update really is a protected kernel variable. */
	if( target >= (void *)__end_protected )
		return -EINVAL;
	if( target < (void *)__start_protected )
		return -EINVAL;
	if( target + length > (void *)__end_protected )
		return -EINVAL;

	while( length ) {
		unsigned pgoffset;
		unsigned pglen;

		struct page *tgt = virt_to_page(target);
		void * writable;

		pgoffset = (unsigned long)target & ~PAGE_MASK;
		pglen = min((unsigned)(PAGE_SIZE - pgoffset), length);

		if( !(writable = vmap( &tgt, 1, VM_MAP, PAGE_KERNEL )) )
			return -EIO;

		/* Attempt to make the write atomic if we can. */
		__write_once_size( writable + pgoffset, src, pglen );

		vunmap( writable );

		target += pglen;
		src += pglen;
		length -= pglen;
	}
	return 0;
}
#ifdef CONFIG_BBRY_DEBUG
EXPORT_SYMBOL(__update_protected);
#endif
