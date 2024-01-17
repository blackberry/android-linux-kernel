/*
 * Based on arch/arm/include/asm/atomic.h
 *
 * Copyright (C) 1996 Russell King.
 * Copyright (C) 2002 Deep Blue Solutions Ltd.
 * Copyright (C) 2012 ARM Ltd.
 * Copyright (C) 2016 BlackBerry Limited
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
#ifndef __ASM_ATOMIC_H
#define __ASM_ATOMIC_H

#include <linux/compiler.h>
#include <linux/types.h>

#include <asm/barrier.h>
#include <asm/cmpxchg.h>
#include <asm/debug-monitors.h>


#define ATOMIC_INIT(i)	{ (i) }

#ifdef __KERNEL__

#define _STR(x) #x
#define STR(x) _STR(x)
#define REFCOUNT_TRAP_INSN "brk        " STR(REFCOUNT_BRK_IMM)

#define _ASM_EXTABLE(from, to)         \
"      .pushsection __ex_table,\"a\"\n"\
"      .align  3\n"                    \
"      .quad   " #from ", " #to"\n"    \
"      .popsection"


/*
 * On ARM, ordinary assignment (str instruction) doesn't clear the local
 * strex/ldrex monitor on some implementations. The reason we can use it for
 * atomic_set() is the clrex or dummy strex done on every exception return.
 */
#define atomic_read(v)	ACCESS_ONCE((v)->counter)
static inline int atomic_read_unchecked(const atomic_unchecked_t *v)
{
       return *(const volatile int *)&v->counter;
}

#define atomic_set(v,i)	(((v)->counter) = (i))

static inline void atomic_set_unchecked(atomic_unchecked_t *v, int i)
{
       v->counter = i;
}

#ifdef CONFIG_PAX_REFCOUNT
#define __OVERFLOW_POST			\
	"	b.vc	3f\n"		\
	"2:	" REFCOUNT_TRAP_INSN "\n"\
	"3:\n"
#define __OVERFLOW_EXTABLE		\
	"\n4:\n"			\
	_ASM_EXTABLE(2b, 4b)
#else
#define __OVERFLOW_POST
#define __OVERFLOW_EXTABLE
#endif

/*
 * AArch64 UP and SMP safe atomic ops.  We use load exclusive and
 * store exclusive to ensure that these are atomic.  We may loop
 * to ensure that the update happens.
 */

#define ATOMIC_OP(op, asm_op)						\
static inline void atomic_##op(int i, atomic_t *v)			\
{									\
	unsigned long tmp;						\
	int result;							\
									\
	asm volatile("// atomic_" #op "\n"				\
"1:	ldxr	%w0, %2\n"						\
"	" #asm_op "	%w0, %w0, %w3\n"				\
	__OVERFLOW_POST							\
"	stxr	%w1, %w0, %2\n"						\
"	cbnz	%w1, 1b"						\
	__OVERFLOW_EXTABLE						\
	: "=&r" (result), "=&r" (tmp), "+Q" (v->counter)		\
	: "Ir" (i));							\
}									\

#define ATOMIC_OP_RETURN(op, asm_op)					\
static inline int atomic_##op##_return(int i, atomic_t *v)		\
{									\
	unsigned long tmp;						\
	int result;							\
									\
	asm volatile("// atomic_" #op "_return\n"			\
"1:	ldxr	%w0, %2\n"						\
"	" #asm_op "	%w0, %w0, %w3\n"				\
	__OVERFLOW_POST							\
"	stlxr	%w1, %w0, %2\n"						\
"	cbnz	%w1, 1b"						\
	__OVERFLOW_EXTABLE                                              \
	: "=&r" (result), "=&r" (tmp), "+Q" (v->counter)		\
	: "Ir" (i)							\
	: "memory");							\
									\
	smp_mb();							\
	return result;							\
}

#define ATOMIC_OPS(op, asm_op)						\
	ATOMIC_OP(op, asm_op)						\
	ATOMIC_OP_RETURN(op, asm_op)

#ifdef CONFIG_PAX_REFCOUNT
ATOMIC_OPS(add, adds)
ATOMIC_OPS(sub, subs)
# else
ATOMIC_OPS(add, add)
ATOMIC_OPS(sub, sub)
#endif

#undef ATOMIC_OPS
#undef ATOMIC_OP_RETURN
#undef ATOMIC_OP

static inline void atomic_add_unchecked(int i, atomic_unchecked_t *v)
{
        unsigned long tmp;
        int result;

        asm volatile("// atomic_add_unchecked\n"
"1:     ldxr    %w0, %2\n"
"       add     %w0, %w0, %w3\n"
"       stxr    %w1, %w0, %2\n"
"       cbnz    %w1, 1b"
        : "=&r" (result), "=&r" (tmp), "+Q" (v->counter)
        : "Ir" (i));
}

static inline int atomic_add_return_unchecked(int i, atomic_unchecked_t *v)
{
        unsigned long tmp;
        int result;

        asm volatile("// atomic_add_return_unchecked\n"
"1:     ldxr    %w0, %2\n"
"       add     %w0, %w0, %w3\n"
"       stlxr   %w1, %w0, %2\n"
"       cbnz    %w1, 1b"
        : "=&r" (result), "=&r" (tmp), "+Q" (v->counter)
        : "Ir" (i)
        : "memory");

        smp_mb();
        return result;
}

static inline void atomic_sub_unchecked(int i, atomic_unchecked_t *v)
{
        unsigned long tmp;
        int result;

        asm volatile("// atomic_sub_unchecked\n"
"1:     ldxr    %w0, %2\n"
"       sub     %w0, %w0, %w3\n"
"       stxr    %w1, %w0, %2\n"
"       cbnz    %w1, 1b"
        : "=&r" (result), "=&r" (tmp), "+Q" (v->counter)
        : "Ir" (i));
}

static inline int atomic_sub_return_unchecked(int i, atomic_unchecked_t *v)
{
        unsigned long tmp;
        int result;

        asm volatile("// atomic_sub_return_unchecked\n"
"1:     ldxr    %w0, %2\n"
"       sub     %w0, %w0, %w3\n"
"       stlxr   %w1, %w0, %2\n"
"       cbnz    %w1, 1b"
        : "=&r" (result), "=&r" (tmp), "+Q" (v->counter)
        : "Ir" (i)
        : "memory");

        smp_mb();
        return result;
}

static inline int atomic_cmpxchg(atomic_t *ptr, int old, int new)
{
	unsigned long tmp;
	int oldval;

	smp_mb();

	asm volatile("// atomic_cmpxchg\n"
"1:	ldxr	%w1, %2\n"
"	cmp	%w1, %w3\n"
"	b.ne	2f\n"
"	stxr	%w0, %w4, %2\n"
"	cbnz	%w0, 1b\n"
"2:"
	: "=&r" (tmp), "=&r" (oldval), "+Q" (ptr->counter)
	: "Ir" (old), "r" (new)
	: "cc");

	smp_mb();
	return oldval;
}

static inline int atomic_cmpxchg_unchecked(atomic_unchecked_t *ptr, int old, int new)
{
        unsigned long tmp;
        int oldval;

        smp_mb();

        asm volatile("// atomic_cmpxchg_unchecked\n"
"1:     ldxr    %w1, %2\n"
"       cmp     %w1, %w3\n"
"       b.ne    2f\n"
"       stxr    %w0, %w4, %2\n"
"       cbnz    %w0, 1b\n"
"2:"
        : "=&r" (tmp), "=&r" (oldval), "+Q" (ptr->counter)
        : "Ir" (old), "r" (new)
        : "cc");

        smp_mb();
        return oldval;
}


#define atomic_xchg(v, new) (xchg(&((v)->counter), new))
static inline int atomic_xchg_unchecked(atomic_unchecked_t *v, int new)
{
        return xchg(&v->counter, new);
}

static inline int __atomic_add_unless(atomic_t *v, int a, int u)
{
	int c, old;

	c = atomic_read(v);
	while (c != u && (old = atomic_cmpxchg((v), c, c + a)) != c)
		c = old;
	return c;
}

#define atomic_inc(v)		atomic_add(1, v)
static inline void atomic_inc_unchecked(atomic_unchecked_t *v)
{
        atomic_add_unchecked(1, v);
}
#define atomic_dec(v)		atomic_sub(1, v)
static inline void atomic_dec_unchecked(atomic_unchecked_t *v)
{
        atomic_sub_unchecked(1, v);
}
#define atomic_inc_and_test(v)	(atomic_add_return(1, v) == 0)
static inline int atomic_inc_and_test_unchecked(atomic_unchecked_t *v)
{
        return atomic_add_return_unchecked(1, v) == 0;
}
#define atomic_dec_and_test(v)	(atomic_sub_return(1, v) == 0)
#define atomic_inc_return(v)    (atomic_add_return(1, v))
static inline int atomic_inc_return_unchecked(atomic_unchecked_t *v)
{
        return atomic_add_return_unchecked(1, v);
}
#define atomic_dec_return(v)    (atomic_sub_return(1, v))
#define atomic_sub_and_test(i, v) (atomic_sub_return(i, v) == 0)

#define atomic_add_negative(i,v) (atomic_add_return(i, v) < 0)


#ifndef CONFIG_GENERIC_ATOMIC64
/*
 * 64-bit atomic operations.
 */
#define ATOMIC64_INIT(i) { (i) }

#define atomic64_read(v)	ACCESS_ONCE((v)->counter)
static inline int atomic64_read_unchecked(const atomic64_unchecked_t *v)
{
        return *(const volatile long *)&v->counter;
}
#define atomic64_set(v,i)	(((v)->counter) = (i))
static inline void atomic64_set_unchecked(atomic64_unchecked_t *v, long i)
{
        v->counter = i;
}

#define ATOMIC64_OP(op, asm_op)						\
static inline void atomic64_##op(long i, atomic64_t *v)			\
{									\
	long result;							\
	unsigned long tmp;						\
									\
	asm volatile("// atomic64_" #op "\n"				\
"1:	ldxr	%0, %2\n"						\
"	" #asm_op "	%0, %0, %3\n"					\
	 __OVERFLOW_POST                                                \
"	stxr	%w1, %0, %2\n"						\
"	cbnz	%w1, 1b"						\
	__OVERFLOW_EXTABLE                                              \
	: "=&r" (result), "=&r" (tmp), "+Q" (v->counter)		\
	: "Ir" (i));							\
}									\

#define ATOMIC64_OP_RETURN(op, asm_op)					\
static inline long atomic64_##op##_return(long i, atomic64_t *v)	\
{									\
	long result;							\
	unsigned long tmp;						\
									\
	asm volatile("// atomic64_" #op "_return\n"			\
"1:	ldxr	%0, %2\n"						\
"	" #asm_op "	%0, %0, %3\n"					\
	 __OVERFLOW_POST 						\
"	stlxr	%w1, %0, %2\n"						\
"	cbnz	%w1, 1b"						\
	__OVERFLOW_EXTABLE						\
	: "=&r" (result), "=&r" (tmp), "+Q" (v->counter)		\
	: "Ir" (i)							\
	: "memory");							\
									\
	smp_mb();							\
	return result;							\
}

#define ATOMIC64_OPS(op, asm_op)					\
	ATOMIC64_OP(op, asm_op)						\
	ATOMIC64_OP_RETURN(op, asm_op)

#ifdef CONFIG_PAX_REFCOUNT
ATOMIC64_OPS(add, adds)
ATOMIC64_OPS(sub, subs)
#else
ATOMIC64_OPS(add, add)
ATOMIC64_OPS(sub, sub)
#endif

#undef ATOMIC64_OPS
#undef ATOMIC64_OP_RETURN
#undef ATOMIC64_OP

static inline void atomic64_add_unchecked(u64 i, atomic64_unchecked_t *v)
{
        long result;
        unsigned long tmp;

        asm volatile("// atomic64_add_unchecked\n"
"1:     ldxr    %0, %2\n"
"       add     %0, %0, %3\n"
"       stxr    %w1, %0, %2\n"
"       cbnz    %w1, 1b"
        : "=&r" (result), "=&r" (tmp), "+Q" (v->counter)
        : "Ir" (i));
}

static inline long atomic64_add_return_unchecked(long i, atomic64_unchecked_t *v)
{
        long result;
        unsigned long tmp;

        asm volatile("// atomic64_add_return_unchecked\n"
"1:     ldxr    %0, %2\n"
"       add     %0, %0, %3\n"
"       stlxr   %w1, %0, %2\n"
"       cbnz    %w1, 1b"
        : "=&r" (result), "=&r" (tmp), "+Q" (v->counter)
        : "Ir" (i)
        : "memory");

        smp_mb();
        return result;
}

static inline void atomic64_sub_unchecked(u64 i, atomic64_unchecked_t *v)
{
        long result;
        unsigned long tmp;

        asm volatile("// atomic64_sub_unchecked\n"
"1:     ldxr    %0, %2\n"
"       sub     %0, %0, %3\n"
"       stxr    %w1, %0, %2\n"
"       cbnz    %w1, 1b"
        : "=&r" (result), "=&r" (tmp), "+Q" (v->counter)
        : "Ir" (i));
}


static inline long atomic64_cmpxchg(atomic64_t *ptr, long old, long new)
{
	long oldval;
	unsigned long res;

	smp_mb();

	asm volatile("// atomic64_cmpxchg\n"
"1:	ldxr	%1, %2\n"
"	cmp	%1, %3\n"
"	b.ne	2f\n"
"	stxr	%w0, %4, %2\n"
"	cbnz	%w0, 1b\n"
"2:"
	: "=&r" (res), "=&r" (oldval), "+Q" (ptr->counter)
	: "Ir" (old), "r" (new)
	: "cc");

	smp_mb();
	return oldval;
}

static inline long atomic64_cmpxchg_unchecked(atomic64_unchecked_t *ptr, long old, long new)
{
        long oldval;
        unsigned long res;

        smp_mb();

        asm volatile("// atomic64_cmpxchg\n"
"1:     ldxr    %1, %2\n"
"       cmp     %1, %3\n"
"       b.ne    2f\n"
"       stxr    %w0, %4, %2\n"
"       cbnz    %w0, 1b\n"
"2:"
        : "=&r" (res), "=&r" (oldval), "+Q" (ptr->counter)
        : "Ir" (old), "r" (new)
        : "cc");

        smp_mb();
        return oldval;
}

#define atomic64_xchg(v, new) (xchg(&((v)->counter), new))
static inline int atomic64_xchg_unchecked(atomic64_unchecked_t *v, long new)
{
        return xchg(&v->counter, new);
}

static inline long atomic64_dec_if_positive(atomic64_t *v)
{
	long result;
	unsigned long tmp;

	asm volatile("// atomic64_dec_if_positive\n"
"1:	ldxr	%0, %2\n"
"	subs	%0, %0, #1\n"
"	b.mi	2f\n"
"	stlxr	%w1, %0, %2\n"
"	cbnz	%w1, 1b\n"
"	dmb	ish\n"
"2:"
	: "=&r" (result), "=&r" (tmp), "+Q" (v->counter)
	:
	: "cc", "memory");

	return result;
}

static inline int atomic64_add_unless(atomic64_t *v, long a, long u)
{
	long c, old;

	c = atomic64_read(v);
	while (c != u && (old = atomic64_cmpxchg((v), c, c + a)) != c)
		c = old;

	return c != u;
}

#define atomic64_add_negative(a, v)	(atomic64_add_return((a), (v)) < 0)
#define atomic64_inc(v)			atomic64_add(1LL, (v))
static inline void atomic64_inc_unchecked(atomic64_unchecked_t *v)
{
        atomic64_add_unchecked(1, v);
}
#define atomic64_inc_return(v)		atomic64_add_return(1LL, (v))
static inline int atomic64_inc_return_unchecked(atomic64_unchecked_t *v)
{
        return atomic64_add_return_unchecked(1, v);
}
#define atomic64_inc_and_test(v)	(atomic64_inc_return(v) == 0)
static inline int atomic64_inc_and_test_unchecked(atomic64_unchecked_t *v)
{
        return atomic64_add_return_unchecked(1, v) == 0;
}
#define atomic64_sub_and_test(a, v)	(atomic64_sub_return((a), (v)) == 0)
#define atomic64_dec(v)			atomic64_sub(1LL, (v))
static inline void atomic64_dec_unchecked(atomic64_unchecked_t *v)
{
        atomic64_sub_unchecked(1, v);
}
#define atomic64_dec_return(v)		atomic64_sub_return(1LL, (v))
#define atomic64_dec_and_test(v)	(atomic64_dec_return((v)) == 0)
#define atomic64_inc_not_zero(v)	atomic64_add_unless((v), 1LL, 0LL)

#undef STR
#undef _STR

#endif /*!CONFIG_GENERIC_ATOMIC64*/
#endif
#endif
