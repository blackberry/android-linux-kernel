/*
 * Based on arch/arm/kernel/io.c
 *
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/export.h>
#include <linux/types.h>
#include <linux/io.h>

#define IO_CHECK_ALIGN(v, a) ((((unsigned long)(v)) & ((a) - 1)) == 0)

/*
 * Copy data from IO memory space to "real" memory space.
 */
void __memcpy_fromio(void *to, const volatile void __iomem *from, size_t count)
{
	while (count && (!IO_CHECK_ALIGN(from, 8) || !IO_CHECK_ALIGN(to, 8))) {
		*(u8 *)to = readb_relaxed_no_log(from);
		from++;
		to++;
		count--;
	}

	while (count >= 8) {
		*(u64 *)to = readq_relaxed_no_log(from);
		from += 8;
		to += 8;
		count -= 8;
	}

	while (count) {
		*(u8 *)to = readb_relaxed_no_log(from);
		from++;
		to++;
		count--;
	}
	__iormb();
}
EXPORT_SYMBOL(__memcpy_fromio);

/*
 * Copy data from "real" memory space to IO memory space.
 */
void __memcpy_toio(volatile void __iomem *to, const void *from, size_t count)
{
	void *p = (void __force *)to;

	__iowmb();
	while (count && (!IO_CHECK_ALIGN(p, 8) || !IO_CHECK_ALIGN(from, 8))) {
		writeb_relaxed_no_log(*(volatile u8 *)from, p);
		from++;
		p++;
		count--;
	}

	while (count >= 8) {
		writeq_relaxed_no_log(*(volatile u64 *)from, p);
		from += 8;
		p += 8;
		count -= 8;
	}

	while (count) {
		writeb_relaxed_no_log(*(volatile u8 *)from, p);
		from++;
		p++;
		count--;
	}
}
EXPORT_SYMBOL(__memcpy_toio);

/*
 * "memset" on IO memory space.
 */
void __memset_io(volatile void __iomem *dst, int c, size_t count)
{
	void *p = (void __force *)dst;
	u64 qc = c;

	qc |= qc << 8;
	qc |= qc << 16;
	qc |= qc << 32;

	__iowmb();
	while (count && !IO_CHECK_ALIGN(p, 8)) {
		writeb_relaxed_no_log(c, p);
		p++;
		count--;
	}

	while (count >= 8) {
		writeq_relaxed_no_log(qc, p);
		p += 8;
		count -= 8;
	}

	while (count) {
		writeb_relaxed_no_log(c, p);
		p++;
		count--;
	}
}
EXPORT_SYMBOL(__memset_io);
