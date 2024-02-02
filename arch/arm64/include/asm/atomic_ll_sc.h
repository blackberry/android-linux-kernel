/*
 * Based on arch/arm/include/asm/atomic.h
 *
 * Copyright (C) 1996 Russell King.
 * Copyright (C) 2002 Deep Blue Solutions Ltd.
 * Copyright (C) 2012 ARM Ltd.
 * Copyright (C) 2017 BlackBerry Limited
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

#ifndef __ASM_ATOMIC_LL_SC_H
#define __ASM_ATOMIC_LL_SC_H

#ifndef __ARM64_IN_ATOMIC_IMPL
#error "please don't include this file directly"
#endif

#ifdef CONFIG_PAX_REFCOUNT
#define BUG_BRK_IMM		0x800  /* defined in arm64/include/asm/brk-imm.h */
#define REFCOUNT_TRAP_INSN "brk	0x800"

#define _ASM_EXTABLE(from, to)					\
"	.pushsection	__ex_table, \"a\"\n"			\
"	.align		3\n"					\
"	.long		(" #from " - .), (" #to " - .)\n"	\
"	.popsection\n"


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
 *
 * NOTE: these functions do *not* follow the PCS and must explicitly
 * save any clobbered registers other than x0 (regardless of return
 * value).  This is achieved through -fcall-saved-* compiler flags for
 * this file, which unfortunately don't work on a per-function basis
 * (the optimize attribute silently ignores these options).
 */

#define ATOMIC_OP(op, asm_op)						\
__LL_SC_INLINE void							\
__LL_SC_PREFIX(atomic_##op(int i, atomic_t *v))				\
{									\
	unsigned long tmp;						\
	int result;							\
									\
	asm volatile("// atomic_" #op "\n"				\
"	prfm	pstl1strm, %2\n"					\
"1:	ldxr	%w0, %2\n"						\
"	" #asm_op "	%w0, %w0, %w3\n"				\
	__OVERFLOW_POST							\
"	stxr	%w1, %w0, %2\n"						\
"	cbnz	%w1, 1b"						\
	__OVERFLOW_EXTABLE						\
	: "=&r" (result), "=&r" (tmp), "+Q" (v->counter)		\
	: "Ir" (i));							\
}									\
__LL_SC_EXPORT(atomic_##op);

#define ATOMIC_OP_RETURN(name, mb, acq, rel, cl, op, asm_op)		\
__LL_SC_INLINE int							\
__LL_SC_PREFIX(atomic_##op##_return##name(int i, atomic_t *v))		\
{									\
	unsigned long tmp;						\
	int result;							\
									\
	asm volatile("// atomic_" #op "_return" #name "\n"		\
"	prfm	pstl1strm, %2\n"					\
"1:	ld" #acq "xr	%w0, %2\n"					\
"	" #asm_op "	%w0, %w0, %w3\n"				\
	__OVERFLOW_POST							\
"	st" #rel "xr	%w1, %w0, %2\n"					\
"	cbnz	%w1, 1b\n"						\
	__OVERFLOW_EXTABLE						\
"	" #mb								\
	: "=&r" (result), "=&r" (tmp), "+Q" (v->counter)		\
	: "Ir" (i)							\
	: cl);								\
									\
	return result;							\
}									\
__LL_SC_EXPORT(atomic_##op##_return##name);

#define ATOMIC_OPS(...)							\
	ATOMIC_OP(__VA_ARGS__)						\
	ATOMIC_OP_RETURN(        , dmb ish,  , l, "memory", __VA_ARGS__)

#define ATOMIC_OPS_RLX(...)						\
	ATOMIC_OPS(__VA_ARGS__)						\
	ATOMIC_OP_RETURN(_relaxed,        ,  ,  ,         , __VA_ARGS__)\
	ATOMIC_OP_RETURN(_acquire,        , a,  , "memory", __VA_ARGS__)\
	ATOMIC_OP_RETURN(_release,        ,  , l, "memory", __VA_ARGS__)

#ifdef CONFIG_PAX_REFCOUNT
ATOMIC_OPS_RLX(add, adds)
ATOMIC_OPS_RLX(sub, subs)
#else
ATOMIC_OPS_RLX(add, add)
ATOMIC_OPS_RLX(sub, sub)
#endif

ATOMIC_OP(and, and)
ATOMIC_OP(andnot, bic)
ATOMIC_OP(or, orr)
ATOMIC_OP(xor, eor)

#undef ATOMIC_OPS_RLX
#undef ATOMIC_OPS
#undef ATOMIC_OP_RETURN
#undef ATOMIC_OP

#define ATOMIC64_OP(op, asm_op)						\
__LL_SC_INLINE void							\
__LL_SC_PREFIX(atomic64_##op(long i, atomic64_t *v))			\
{									\
	long result;							\
	unsigned long tmp;						\
									\
	asm volatile("// atomic64_" #op "\n"				\
"	prfm	pstl1strm, %2\n"					\
"1:	ldxr	%0, %2\n"						\
"	" #asm_op "	%0, %0, %3\n"					\
	 __OVERFLOW_POST                                                \
"	stxr	%w1, %0, %2\n"						\
"	cbnz	%w1, 1b"						\
	__OVERFLOW_EXTABLE                                              \
	: "=&r" (result), "=&r" (tmp), "+Q" (v->counter)		\
	: "Ir" (i));							\
}									\
__LL_SC_EXPORT(atomic64_##op);

#define ATOMIC64_OP_RETURN(name, mb, acq, rel, cl, op, asm_op)		\
__LL_SC_INLINE long							\
__LL_SC_PREFIX(atomic64_##op##_return##name(long i, atomic64_t *v))	\
{									\
	long result;							\
	unsigned long tmp;						\
									\
	asm volatile("// atomic64_" #op "_return" #name "\n"		\
"	prfm	pstl1strm, %2\n"					\
"1:	ld" #acq "xr	%0, %2\n"					\
"	" #asm_op "	%0, %0, %3\n"					\
	 __OVERFLOW_POST 						\
"	st" #rel "xr	%w1, %0, %2\n"					\
"	cbnz	%w1, 1b\n"						\
	__OVERFLOW_EXTABLE						\
"	" #mb								\
	: "=&r" (result), "=&r" (tmp), "+Q" (v->counter)		\
	: "Ir" (i)							\
	: cl);								\
									\
	return result;							\
}									\
__LL_SC_EXPORT(atomic64_##op##_return##name);

#define ATOMIC64_OPS(...)						\
	ATOMIC64_OP(__VA_ARGS__)					\
	ATOMIC64_OP_RETURN(, dmb ish,  , l, "memory", __VA_ARGS__)

#define ATOMIC64_OPS_RLX(...)						\
	ATOMIC64_OPS(__VA_ARGS__)					\
	ATOMIC64_OP_RETURN(_relaxed,,  ,  ,         , __VA_ARGS__)	\
	ATOMIC64_OP_RETURN(_acquire,, a,  , "memory", __VA_ARGS__)	\
	ATOMIC64_OP_RETURN(_release,,  , l, "memory", __VA_ARGS__)

#ifdef CONFIG_PAX_REFCOUNT
ATOMIC64_OPS_RLX(add, adds)
ATOMIC64_OPS_RLX(sub, subs)
#else
ATOMIC64_OPS_RLX(add, add)
ATOMIC64_OPS_RLX(sub, sub)
#endif

ATOMIC64_OP(and, and)
ATOMIC64_OP(andnot, bic)
ATOMIC64_OP(or, orr)
ATOMIC64_OP(xor, eor)

#undef ATOMIC64_OPS_RLX
#undef ATOMIC64_OPS
#undef ATOMIC64_OP_RETURN
#undef ATOMIC64_OP

__LL_SC_INLINE long
__LL_SC_PREFIX(atomic64_dec_if_positive(atomic64_t *v))
{
	long result;
	unsigned long tmp;

	asm volatile("// atomic64_dec_if_positive\n"
"	prfm	pstl1strm, %2\n"
"1:	ldxr	%0, %2\n"
"	subs	%0, %0, #1\n"
"	b.lt	2f\n"
"	stlxr	%w1, %0, %2\n"
"	cbnz	%w1, 1b\n"
"	dmb	ish\n"
"2:"
	: "=&r" (result), "=&r" (tmp), "+Q" (v->counter)
	:
	: "cc", "memory");

	return result;
}
__LL_SC_EXPORT(atomic64_dec_if_positive);

#define __CMPXCHG_CASE(w, sz, name, mb, acq, rel, cl)			\
__LL_SC_INLINE unsigned long						\
__LL_SC_PREFIX(__cmpxchg_case_##name(volatile void *ptr,		\
				     unsigned long old,			\
				     unsigned long new))		\
{									\
	unsigned long tmp, oldval;					\
									\
	asm volatile(							\
	"	prfm	pstl1strm, %[v]\n"				\
	"1:	ld" #acq "xr" #sz "\t%" #w "[oldval], %[v]\n"		\
	"	eor	%" #w "[tmp], %" #w "[oldval], %" #w "[old]\n"	\
	"	cbnz	%" #w "[tmp], 2f\n"				\
	"	st" #rel "xr" #sz "\t%w[tmp], %" #w "[new], %[v]\n"	\
	"	cbnz	%w[tmp], 1b\n"					\
	"	" #mb "\n"						\
	"	mov	%" #w "[oldval], %" #w "[old]\n"		\
	"2:"								\
	: [tmp] "=&r" (tmp), [oldval] "=&r" (oldval),			\
	  [v] "+Q" (*(unsigned long *)ptr)				\
	: [old] "Lr" (old), [new] "r" (new)				\
	: cl);								\
									\
	return oldval;							\
}									\
__LL_SC_EXPORT(__cmpxchg_case_##name);

__CMPXCHG_CASE(w, b,     1,        ,  ,  ,         )
__CMPXCHG_CASE(w, h,     2,        ,  ,  ,         )
__CMPXCHG_CASE(w,  ,     4,        ,  ,  ,         )
__CMPXCHG_CASE( ,  ,     8,        ,  ,  ,         )
__CMPXCHG_CASE(w, b, acq_1,        , a,  , "memory")
__CMPXCHG_CASE(w, h, acq_2,        , a,  , "memory")
__CMPXCHG_CASE(w,  , acq_4,        , a,  , "memory")
__CMPXCHG_CASE( ,  , acq_8,        , a,  , "memory")
__CMPXCHG_CASE(w, b, rel_1,        ,  , l, "memory")
__CMPXCHG_CASE(w, h, rel_2,        ,  , l, "memory")
__CMPXCHG_CASE(w,  , rel_4,        ,  , l, "memory")
__CMPXCHG_CASE( ,  , rel_8,        ,  , l, "memory")
__CMPXCHG_CASE(w, b,  mb_1, dmb ish,  , l, "memory")
__CMPXCHG_CASE(w, h,  mb_2, dmb ish,  , l, "memory")
__CMPXCHG_CASE(w,  ,  mb_4, dmb ish,  , l, "memory")
__CMPXCHG_CASE( ,  ,  mb_8, dmb ish,  , l, "memory")

#undef __CMPXCHG_CASE

#define __CMPXCHG_DBL(name, mb, rel, cl)				\
__LL_SC_INLINE long							\
__LL_SC_PREFIX(__cmpxchg_double##name(unsigned long old1,		\
				      unsigned long old2,		\
				      unsigned long new1,		\
				      unsigned long new2,		\
				      volatile void *ptr))		\
{									\
	unsigned long tmp, ret;						\
									\
	asm volatile("// __cmpxchg_double" #name "\n"			\
	"	prfm	pstl1strm, %2\n"				\
	"1:	ldxp	%0, %1, %2\n"					\
	"	eor	%0, %0, %3\n"					\
	"	eor	%1, %1, %4\n"					\
	"	orr	%1, %0, %1\n"					\
	"	cbnz	%1, 2f\n"					\
	"	st" #rel "xp	%w0, %5, %6, %2\n"			\
	"	cbnz	%w0, 1b\n"					\
	"	" #mb "\n"						\
	"2:"								\
	: "=&r" (tmp), "=&r" (ret), "+Q" (*(unsigned long *)ptr)	\
	: "r" (old1), "r" (old2), "r" (new1), "r" (new2)		\
	: cl);								\
									\
	return ret;							\
}									\
__LL_SC_EXPORT(__cmpxchg_double##name);

__CMPXCHG_DBL(   ,        ,  ,         )
__CMPXCHG_DBL(_mb, dmb ish, l, "memory")

#undef __CMPXCHG_DBL

/*  PAX unchecked implementations */

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

static inline long atomic64_sub_return_unchecked(u64 i, atomic64_unchecked_t *v)
{
        long result = 0;
        unsigned long tmp;

        asm volatile("// atomic64_sub_unchecked\n"
"1:     ldxr    %0, %2\n"
"       sub     %0, %0, %3\n"
"       stxr    %w1, %0, %2\n"
"       cbnz    %w1, 1b"
        : "=&r" (result), "=&r" (tmp), "+Q" (v->counter)
        : "Ir" (i)
	: "memory");
	smp_mb();

        return result;
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


#endif	/* __ASM_ATOMIC_LL_SC_H */
