/*
 * linux/fs/binfmt_elf.c
 *
 * These are the functions used to load ELF format executables as used
 * on SVr4 machines.  Information on the format may be found in the book
 * "UNIX SYSTEM V RELEASE 4 Programmers Guide: Ansi C and Programming Support
 * Tools".
 *
 * Copyright 1993, 1994: Eric Youngdale (ericy@cais.com).
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/binfmts.h>
#include <linux/string.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/personality.h>
#include <linux/elfcore.h>
#include <linux/init.h>
#include <linux/highuid.h>
#include <linux/compiler.h>
#include <linux/highmem.h>
#include <linux/pagemap.h>
#include <linux/vmalloc.h>
#include <linux/security.h>
#include <linux/random.h>
#include <linux/elf.h>
#include <linux/elf-randomize.h>
#include <linux/utsname.h>
#include <linux/coredump.h>
#include <linux/sched.h>
#include <linux/xattr.h>
#include <asm/uaccess.h>
#include <asm/param.h>
#include <asm/page.h>

#ifndef user_long_t
#define user_long_t long
#endif
#ifndef user_siginfo_t
#define user_siginfo_t siginfo_t
#endif

static int load_elf_binary(struct linux_binprm *bprm);
static unsigned long elf_map(struct file *, unsigned long, struct elf_phdr *,
				int, int, unsigned long) __intentional_overflow(-1);

#ifdef CONFIG_USELIB
static int load_elf_library(struct file *);
#else
#define load_elf_library NULL
#endif

/*
 * If we don't support core dumping, then supply a NULL so we
 * don't even try.
 */
#ifdef CONFIG_ELF_CORE
static int elf_core_dump(struct coredump_params *cprm);
#else
#define elf_core_dump	NULL
#endif

#ifdef CONFIG_PAX_MPROTECT
static void elf_handle_mprotect(struct vm_area_struct *vma, unsigned long newflags);
#endif

#ifdef CONFIG_GRKERNSEC_RWXMAP_LOG
static void elf_handle_mmap(struct file *file);
#endif

#if ELF_EXEC_PAGESIZE > PAGE_SIZE
#define ELF_MIN_ALIGN	ELF_EXEC_PAGESIZE
#else
#define ELF_MIN_ALIGN	PAGE_SIZE
#endif

#ifndef ELF_CORE_EFLAGS
#define ELF_CORE_EFLAGS	0
#endif

#define ELF_PAGESTART(_v) ((_v) & ~(unsigned long)(ELF_MIN_ALIGN-1))
#define ELF_PAGEOFFSET(_v) ((_v) & (ELF_MIN_ALIGN-1))
#define ELF_PAGEALIGN(_v) (((_v) + ELF_MIN_ALIGN - 1) & ~(ELF_MIN_ALIGN - 1))

static struct linux_binfmt elf_format = {
	.module		= THIS_MODULE,
	.load_binary	= load_elf_binary,
	.load_shlib	= load_elf_library,
	.core_dump	= elf_core_dump,

#ifdef CONFIG_PAX_MPROTECT
	.handle_mprotect= elf_handle_mprotect,
#endif

#ifdef CONFIG_GRKERNSEC_RWXMAP_LOG
	.handle_mmap	= elf_handle_mmap,
#endif

	.min_coredump	= ELF_EXEC_PAGESIZE,
};

#define BAD_ADDR(x) ((unsigned long)(x) >= TASK_SIZE)

static int set_brk(unsigned long start, unsigned long end)
{
	unsigned long e = end;

	start = ELF_PAGEALIGN(start);
	end = ELF_PAGEALIGN(end);
	if (end > start) {
		unsigned long addr;
		addr = vm_brk(start, end - start);
		if (BAD_ADDR(addr))
			return addr;
	}
	current->mm->start_brk = current->mm->brk = e;
	return 0;
}

/* We need to explicitly zero any fractional pages
   after the data section (i.e. bss).  This would
   contain the junk from the file that should not
   be in memory
 */
static int padzero(unsigned long elf_bss)
{
	unsigned long nbyte;

	nbyte = ELF_PAGEOFFSET(elf_bss);
	if (nbyte) {
		nbyte = ELF_MIN_ALIGN - nbyte;
		if (clear_user((void __user *) elf_bss, nbyte))
			return -EFAULT;
	}
	return 0;
}

/* Let's use some macros to make this stack manipulation a little clearer */
#ifdef CONFIG_STACK_GROWSUP
#define STACK_ADD(sp, items) ((elf_addr_t __user *)(sp) + (items))
#define STACK_ROUND(sp, items) \
	((15 + (unsigned long) ((sp) + (items))) &~ 15UL)
#define STACK_ALLOC(sp, len) ({ \
	elf_addr_t __user *old_sp = (elf_addr_t __user *)sp; sp += len; \
	old_sp; })
#else
#define STACK_ADD(sp, items) ((elf_addr_t __user *)(sp) - (items))
#define STACK_ROUND(sp, items) \
	(((unsigned long) (sp - items)) &~ 15UL)
#define STACK_ALLOC(sp, len) ({ sp -= len ; sp; })
#endif

#ifndef ELF_BASE_PLATFORM
/*
 * AT_BASE_PLATFORM indicates the "real" hardware/microarchitecture.
 * If the arch defines ELF_BASE_PLATFORM (in asm/elf.h), the value
 * will be copied to the user stack in the same manner as AT_PLATFORM.
 */
#define ELF_BASE_PLATFORM NULL
#endif

static int
create_elf_tables(struct linux_binprm *bprm, struct elfhdr *exec,
		unsigned long load_addr, unsigned long interp_load_addr)
{
	unsigned long p = bprm->p;
	int argc = bprm->argc;
	int envc = bprm->envc;
	elf_addr_t __user *argv;
	elf_addr_t __user *envp;
	elf_addr_t __user *sp;
	elf_addr_t __user *u_platform;
	elf_addr_t __user *u_base_platform;
	elf_addr_t __user *u_rand_bytes;
	const char *k_platform = ELF_PLATFORM;
	const char *k_base_platform = ELF_BASE_PLATFORM;
	u32 k_rand_bytes[4];
	int items;
	elf_addr_t *elf_info;
	int ei_index = 0;
	const struct cred *cred = current_cred();
	struct vm_area_struct *vma;
	unsigned long saved_auxv[AT_VECTOR_SIZE];

	/*
	 * In some cases (e.g. Hyper-Threading), we want to avoid L1
	 * evictions by the processes running on the same package. One
	 * thing we can do is to shuffle the initial stack for them.
	 */

	p = arch_align_stack(p);

	/*
	 * If this architecture has a platform capability string, copy it
	 * to userspace.  In some cases (Sparc), this info is impossible
	 * for userspace to get any other way, in others (i386) it is
	 * merely difficult.
	 */
	u_platform = NULL;
	if (k_platform) {
		size_t len = strlen(k_platform) + 1;

		u_platform = (elf_addr_t __user *)STACK_ALLOC(p, len);
		if (__copy_to_user(u_platform, k_platform, len))
			return -EFAULT;
	}

	/*
	 * If this architecture has a "base" platform capability
	 * string, copy it to userspace.
	 */
	u_base_platform = NULL;
	if (k_base_platform) {
		size_t len = strlen(k_base_platform) + 1;

		u_base_platform = (elf_addr_t __user *)STACK_ALLOC(p, len);
		if (__copy_to_user(u_base_platform, k_base_platform, len))
			return -EFAULT;
	}

	/*
	 * Generate 16 random bytes for userspace PRNG seeding.
	 */
	get_random_bytes(k_rand_bytes, sizeof(k_rand_bytes));
	prandom_seed(k_rand_bytes[0] ^ prandom_u32());
	prandom_seed(k_rand_bytes[1] ^ prandom_u32());
	prandom_seed(k_rand_bytes[2] ^ prandom_u32());
	prandom_seed(k_rand_bytes[3] ^ prandom_u32());
	p = STACK_ROUND(p, sizeof(k_rand_bytes));
	u_rand_bytes = (elf_addr_t __user *) p;
	if (__copy_to_user(u_rand_bytes, k_rand_bytes, sizeof(k_rand_bytes)))
		return -EFAULT;

	/* Create the ELF interpreter info */
	elf_info = (elf_addr_t *)current->mm->saved_auxv;
	/* update AT_VECTOR_SIZE_BASE if the number of NEW_AUX_ENT() changes */
#define NEW_AUX_ENT(id, val) \
	do { \
		elf_info[ei_index++] = id; \
		elf_info[ei_index++] = val; \
	} while (0)

#ifdef ARCH_DLINFO
	/* 
	 * ARCH_DLINFO must come first so PPC can do its special alignment of
	 * AUXV.
	 * update AT_VECTOR_SIZE_ARCH if the number of NEW_AUX_ENT() in
	 * ARCH_DLINFO changes
	 */
	ARCH_DLINFO;
#endif
	NEW_AUX_ENT(AT_HWCAP, ELF_HWCAP);
	NEW_AUX_ENT(AT_PAGESZ, ELF_EXEC_PAGESIZE);
	NEW_AUX_ENT(AT_CLKTCK, CLOCKS_PER_SEC);
	NEW_AUX_ENT(AT_PHDR, load_addr + exec->e_phoff);
	NEW_AUX_ENT(AT_PHENT, sizeof(struct elf_phdr));
	NEW_AUX_ENT(AT_PHNUM, exec->e_phnum);
	NEW_AUX_ENT(AT_BASE, interp_load_addr);
	NEW_AUX_ENT(AT_FLAGS, 0);
	NEW_AUX_ENT(AT_ENTRY, exec->e_entry);
	NEW_AUX_ENT(AT_UID, from_kuid_munged(cred->user_ns, cred->uid));
	NEW_AUX_ENT(AT_EUID, from_kuid_munged(cred->user_ns, cred->euid));
	NEW_AUX_ENT(AT_GID, from_kgid_munged(cred->user_ns, cred->gid));
	NEW_AUX_ENT(AT_EGID, from_kgid_munged(cred->user_ns, cred->egid));
 	NEW_AUX_ENT(AT_SECURE, security_bprm_secureexec(bprm));
	NEW_AUX_ENT(AT_RANDOM, (elf_addr_t)(unsigned long)u_rand_bytes);
#ifdef ELF_HWCAP2
	NEW_AUX_ENT(AT_HWCAP2, ELF_HWCAP2);
#endif
	NEW_AUX_ENT(AT_EXECFN, bprm->exec);
	if (k_platform) {
		NEW_AUX_ENT(AT_PLATFORM,
			    (elf_addr_t)(unsigned long)u_platform);
	}
	if (k_base_platform) {
		NEW_AUX_ENT(AT_BASE_PLATFORM,
			    (elf_addr_t)(unsigned long)u_base_platform);
	}
	if (bprm->interp_flags & BINPRM_FLAGS_EXECFD) {
		NEW_AUX_ENT(AT_EXECFD, bprm->interp_data);
	}
#undef NEW_AUX_ENT
	/* AT_NULL is zero; clear the rest too */
	memset(&elf_info[ei_index], 0,
	       sizeof current->mm->saved_auxv - ei_index * sizeof elf_info[0]);

	/* And advance past the AT_NULL entry.  */
	ei_index += 2;

	sp = STACK_ADD(p, ei_index);

	items = (argc + 1) + (envc + 1) + 1;
	bprm->p = STACK_ROUND(sp, items);

	/* Point sp at the lowest address on the stack */
#ifdef CONFIG_STACK_GROWSUP
	sp = (elf_addr_t __user *)bprm->p - items - ei_index;
	bprm->exec = (unsigned long)sp; /* XXX: PARISC HACK */
#else
	sp = (elf_addr_t __user *)bprm->p;
#endif


	/*
	 * Grow the stack manually; some architectures have a limit on how
	 * far ahead a user-space access may be in order to grow the stack.
	 */
	vma = find_extend_vma(current->mm, bprm->p);
	if (!vma)
		return -EFAULT;

	/* Now, let's put argc (and argv, envp if appropriate) on the stack */
	if (__put_user(argc, sp++))
		return -EFAULT;
	argv = sp;
	envp = argv + argc + 1;

	/* Populate argv and envp */
	p = current->mm->arg_end = current->mm->arg_start;
	while (argc-- > 0) {
		size_t len;
		if (__put_user((elf_addr_t)p, argv++))
			return -EFAULT;
		len = strnlen_user((void __user *)p, MAX_ARG_STRLEN);
		if (!len || len > MAX_ARG_STRLEN)
			return -EINVAL;
		p += len;
	}
	if (__put_user(0, argv))
		return -EFAULT;
	current->mm->arg_end = current->mm->env_start = p;
	while (envc-- > 0) {
		size_t len;
		if (__put_user((elf_addr_t)p, envp++))
			return -EFAULT;
		len = strnlen_user((void __user *)p, MAX_ARG_STRLEN);
		if (!len || len > MAX_ARG_STRLEN)
			return -EINVAL;
		p += len;
	}
	if (__put_user(0, envp))
		return -EFAULT;
	current->mm->env_end = p;

	memcpy(saved_auxv, elf_info, ei_index * sizeof(elf_addr_t));

	/* Put the elf_info on the stack in the right place.  */
	sp = (elf_addr_t __user *)envp + 1;
	if (copy_to_user(sp, saved_auxv, ei_index * sizeof(elf_addr_t)))
		return -EFAULT;
	return 0;
}

#ifndef elf_map

static unsigned long elf_map(struct file *filep, unsigned long addr,
		struct elf_phdr *eppnt, int prot, int type,
		unsigned long total_size)
{
	unsigned long map_addr;
	unsigned long size = eppnt->p_filesz + ELF_PAGEOFFSET(eppnt->p_vaddr);
	unsigned long off = eppnt->p_offset - ELF_PAGEOFFSET(eppnt->p_vaddr);
	addr = ELF_PAGESTART(addr);
	size = ELF_PAGEALIGN(size);

	/* mmap() will return -EINVAL if given a zero size, but a
	 * segment with zero filesize is perfectly valid */
	if (!size)
		return addr;

	/*
	* total_size is the size of the ELF (interpreter) image.
	* The _first_ mmap needs to know the full size, otherwise
	* randomization might put this image into an overlapping
	* position with the ELF binary image. (since size < total_size)
	* So we first map the 'big' image - and unmap the remainder at
	* the end. (which unmap is needed for ELF images with holes.)
	*/
	if (total_size) {
		total_size = ELF_PAGEALIGN(total_size);
		map_addr = vm_mmap(filep, addr, total_size, prot, type, off);
		if (!BAD_ADDR(map_addr))
			vm_munmap(map_addr+size, total_size-size);
	} else
		map_addr = vm_mmap(filep, addr, size, prot, type, off);

	return(map_addr);
}

#endif /* !elf_map */

static unsigned long total_mapping_size(struct elf_phdr *cmds, int nr)
{
	int i, first_idx = -1, last_idx = -1;

	for (i = 0; i < nr; i++) {
		if (cmds[i].p_type == PT_LOAD) {
			last_idx = i;
			if (first_idx == -1)
				first_idx = i;
		}
	}
	if (first_idx == -1)
		return 0;

	return cmds[last_idx].p_vaddr + cmds[last_idx].p_memsz -
				ELF_PAGESTART(cmds[first_idx].p_vaddr);
}


/* This is much more generalized than the library routine read function,
   so we keep this separate.  Technically the library read function
   is only provided so that we can read a.out libraries that have
   an ELF header */

static unsigned long load_elf_interp(struct elfhdr *interp_elf_ex,
		struct file *interpreter, unsigned long no_base)
{
	struct elf_phdr *elf_phdata;
	struct elf_phdr *eppnt;
	unsigned long load_addr = 0, pax_task_size = TASK_SIZE;
	int load_addr_set = 0;
	unsigned long last_bss = 0, elf_bss = 0;
	unsigned long error = -EINVAL;
	unsigned long total_size;
	int retval, i, size;

	/* First of all, some simple consistency checks */
	if (interp_elf_ex->e_type != ET_EXEC &&
	    interp_elf_ex->e_type != ET_DYN)
		goto out;
	if (!elf_check_arch(interp_elf_ex))
		goto out;
	if (!interpreter->f_op->mmap)
		goto out;

	/*
	 * If the size of this structure has changed, then punt, since
	 * we will be doing the wrong thing.
	 */
	if (interp_elf_ex->e_phentsize != sizeof(struct elf_phdr))
		goto out;
	if (interp_elf_ex->e_phnum < 1 ||
		interp_elf_ex->e_phnum > 65536U / sizeof(struct elf_phdr))
		goto out;

	/* Now read in all of the header information */
	size = sizeof(struct elf_phdr) * interp_elf_ex->e_phnum;
	if (size > ELF_MIN_ALIGN)
		goto out;
	elf_phdata = kmalloc(size, GFP_KERNEL);
	if (!elf_phdata)
		goto out;

	retval = kernel_read(interpreter, interp_elf_ex->e_phoff,
			     (char *)elf_phdata, size);
	error = -EIO;
	if (retval != size) {
		if (retval < 0)
			error = retval;	
		goto out_close;
	}

	total_size = total_mapping_size(elf_phdata, interp_elf_ex->e_phnum);
	if (!total_size) {
		error = -EINVAL;
		goto out_close;
	}

#ifdef CONFIG_PAX_SEGMEXEC
	if (current->mm->pax_flags & MF_PAX_SEGMEXEC)
		pax_task_size = SEGMEXEC_TASK_SIZE;
#endif

	eppnt = elf_phdata;
	for (i = 0; i < interp_elf_ex->e_phnum; i++, eppnt++) {
		if (eppnt->p_type == PT_LOAD) {
			int elf_type = MAP_PRIVATE | MAP_DENYWRITE;
			int elf_prot = 0;
			unsigned long vaddr = 0;
			unsigned long k, map_addr;

			if (eppnt->p_flags & PF_R)
		    		elf_prot = PROT_READ;
			if (eppnt->p_flags & PF_W)
				elf_prot |= PROT_WRITE;
			if (eppnt->p_flags & PF_X)
				elf_prot |= PROT_EXEC;
			vaddr = eppnt->p_vaddr;
			if (interp_elf_ex->e_type == ET_EXEC || load_addr_set)
				elf_type |= MAP_FIXED;
			else if (no_base && interp_elf_ex->e_type == ET_DYN)
				load_addr = -vaddr;

			map_addr = elf_map(interpreter, load_addr + vaddr,
					eppnt, elf_prot, elf_type, total_size);
			total_size = 0;
			error = map_addr;
			if (BAD_ADDR(map_addr))
				goto out_close;

			if (!load_addr_set &&
			    interp_elf_ex->e_type == ET_DYN) {
				load_addr = map_addr - ELF_PAGESTART(vaddr);
				load_addr_set = 1;
			}

			/*
			 * Check to see if the section's size will overflow the
			 * allowed task size. Note that p_filesz must always be
			 * <= p_memsize so it's only necessary to check p_memsz.
			 */
			k = load_addr + eppnt->p_vaddr;
			if (BAD_ADDR(k) ||
			    eppnt->p_filesz > eppnt->p_memsz ||
			    eppnt->p_memsz > pax_task_size ||
			    pax_task_size - eppnt->p_memsz < k) {
				error = -ENOMEM;
				goto out_close;
			}

			/*
			 * Find the end of the file mapping for this phdr, and
			 * keep track of the largest address we see for this.
			 */
			k = load_addr + eppnt->p_vaddr + eppnt->p_filesz;
			if (k > elf_bss)
				elf_bss = k;

			/*
			 * Do the same thing for the memory mapping - between
			 * elf_bss and last_bss is the bss section.
			 */
			k = load_addr + eppnt->p_memsz + eppnt->p_vaddr;
			if (k > last_bss)
				last_bss = k;
		}
	}

	if (last_bss > elf_bss) {
		/*
		 * Now fill out the bss section.  First pad the last page up
		 * to the page boundary, and then perform a mmap to make sure
		 * that there are zero-mapped pages up to and including the
		 * last bss page.
		 */
		if (padzero(elf_bss)) {
			error = -EFAULT;
			goto out_close;
		}

		/* What we have mapped so far */
		elf_bss = ELF_PAGESTART(elf_bss + ELF_MIN_ALIGN - 1);

		/* Map the last of the bss segment */
		if (last_bss > elf_bss) {
			error = vm_brk(elf_bss, last_bss - elf_bss);
			if (BAD_ADDR(error))
				goto out_close;
		}
	}

	error = load_addr;

out_close:
	kfree(elf_phdata);
out:
	return error;
}

#ifdef CONFIG_PAX_PT_PAX_FLAGS
#ifdef CONFIG_PAX_SOFTMODE
static unsigned long pax_parse_pt_pax_softmode(const struct elf_phdr * const elf_phdata)
{
	unsigned long pax_flags = 0UL;

#ifdef CONFIG_PAX_PAGEEXEC
	if (elf_phdata->p_flags & PF_PAGEEXEC)
		pax_flags |= MF_PAX_PAGEEXEC;
#endif

#ifdef CONFIG_PAX_SEGMEXEC
	if (elf_phdata->p_flags & PF_SEGMEXEC)
		pax_flags |= MF_PAX_SEGMEXEC;
#endif

#ifdef CONFIG_PAX_EMUTRAMP
	if ((elf_phdata->p_flags & PF_EMUTRAMP) && (pax_flags & (MF_PAX_PAGEEXEC | MF_PAX_SEGMEXEC)))
		pax_flags |= MF_PAX_EMUTRAMP;
#endif

#ifdef CONFIG_PAX_MPROTECT
	if (elf_phdata->p_flags & PF_MPROTECT)
		pax_flags |= MF_PAX_MPROTECT;
#endif

#if defined(CONFIG_PAX_RANDMMAP) || defined(CONFIG_PAX_RANDUSTACK)
	if (randomize_va_space && (elf_phdata->p_flags & PF_RANDMMAP))
		pax_flags |= MF_PAX_RANDMMAP;
#endif

	return pax_flags;
}
#endif

static unsigned long pax_parse_pt_pax_hardmode(const struct elf_phdr * const elf_phdata)
{
	unsigned long pax_flags = 0UL;

#ifdef CONFIG_PAX_PAGEEXEC
	if (!(elf_phdata->p_flags & PF_NOPAGEEXEC))
		pax_flags |= MF_PAX_PAGEEXEC;
#endif

#ifdef CONFIG_PAX_SEGMEXEC
	if (!(elf_phdata->p_flags & PF_NOSEGMEXEC))
		pax_flags |= MF_PAX_SEGMEXEC;
#endif

#ifdef CONFIG_PAX_EMUTRAMP
	if (!(elf_phdata->p_flags & PF_NOEMUTRAMP))
		pax_flags |= MF_PAX_EMUTRAMP;
#endif

#ifdef CONFIG_PAX_MPROTECT
	if (!(elf_phdata->p_flags & PF_NOMPROTECT))
		pax_flags |= MF_PAX_MPROTECT;
#endif

#if defined(CONFIG_PAX_RANDMMAP) || defined(CONFIG_PAX_RANDUSTACK)
	if (randomize_va_space && !(elf_phdata->p_flags & PF_NORANDMMAP))
		pax_flags |= MF_PAX_RANDMMAP;
#endif

	return pax_flags;
}
#endif

#ifdef CONFIG_PAX_XATTR_PAX_FLAGS
#ifdef CONFIG_PAX_SOFTMODE
static unsigned long pax_parse_xattr_pax_softmode(unsigned long pax_flags_softmode)
{
	unsigned long pax_flags = 0UL;

#ifdef CONFIG_PAX_PAGEEXEC
	if (pax_flags_softmode & MF_PAX_PAGEEXEC)
		pax_flags |= MF_PAX_PAGEEXEC;
#endif

#ifdef CONFIG_PAX_SEGMEXEC
	if (pax_flags_softmode & MF_PAX_SEGMEXEC)
		pax_flags |= MF_PAX_SEGMEXEC;
#endif

#ifdef CONFIG_PAX_EMUTRAMP
	if ((pax_flags_softmode & MF_PAX_EMUTRAMP) && (pax_flags & (MF_PAX_PAGEEXEC | MF_PAX_SEGMEXEC)))
		pax_flags |= MF_PAX_EMUTRAMP;
#endif

#ifdef CONFIG_PAX_MPROTECT
	if (pax_flags_softmode & MF_PAX_MPROTECT)
		pax_flags |= MF_PAX_MPROTECT;
#endif

#if defined(CONFIG_PAX_RANDMMAP) || defined(CONFIG_PAX_RANDUSTACK)
	if (randomize_va_space && (pax_flags_softmode & MF_PAX_RANDMMAP))
		pax_flags |= MF_PAX_RANDMMAP;
#endif

	return pax_flags;
}
#endif

static unsigned long pax_parse_xattr_pax_hardmode(unsigned long pax_flags_hardmode)
{
	unsigned long pax_flags = 0UL;

#ifdef CONFIG_PAX_PAGEEXEC
	if (!(pax_flags_hardmode & MF_PAX_PAGEEXEC))
		pax_flags |= MF_PAX_PAGEEXEC;
#endif

#ifdef CONFIG_PAX_SEGMEXEC
	if (!(pax_flags_hardmode & MF_PAX_SEGMEXEC))
		pax_flags |= MF_PAX_SEGMEXEC;
#endif

#ifdef CONFIG_PAX_EMUTRAMP
	if (!(pax_flags_hardmode & MF_PAX_EMUTRAMP))
		pax_flags |= MF_PAX_EMUTRAMP;
#endif

#ifdef CONFIG_PAX_MPROTECT
	if (!(pax_flags_hardmode & MF_PAX_MPROTECT))
		pax_flags |= MF_PAX_MPROTECT;
#endif

#if defined(CONFIG_PAX_RANDMMAP) || defined(CONFIG_PAX_RANDUSTACK)
	if (randomize_va_space && !(pax_flags_hardmode & MF_PAX_RANDMMAP))
		pax_flags |= MF_PAX_RANDMMAP;
#endif

	return pax_flags;
}
#endif

#if defined(CONFIG_PAX_NOEXEC) || defined(CONFIG_PAX_ASLR)
static unsigned long pax_parse_defaults(void)
{
	unsigned long pax_flags = 0UL;

#ifdef CONFIG_PAX_SOFTMODE
	if (pax_softmode)
		return pax_flags;
#endif

#ifdef CONFIG_PAX_PAGEEXEC
	pax_flags |= MF_PAX_PAGEEXEC;
#endif

#ifdef CONFIG_PAX_SEGMEXEC
	pax_flags |= MF_PAX_SEGMEXEC;
#endif

#ifdef CONFIG_PAX_MPROTECT
	pax_flags |= MF_PAX_MPROTECT;
#endif

#ifdef CONFIG_PAX_RANDMMAP
	if (randomize_va_space)
		pax_flags |= MF_PAX_RANDMMAP;
#endif

	return pax_flags;
}

static unsigned long pax_parse_ei_pax(const struct elfhdr * const elf_ex)
{
	unsigned long pax_flags = PAX_PARSE_FLAGS_FALLBACK;

#ifdef CONFIG_PAX_EI_PAX

#ifdef CONFIG_PAX_SOFTMODE
	if (pax_softmode)
		return pax_flags;
#endif

	pax_flags = 0UL;

#ifdef CONFIG_PAX_PAGEEXEC
	if (!(elf_ex->e_ident[EI_PAX] & EF_PAX_PAGEEXEC))
		pax_flags |= MF_PAX_PAGEEXEC;
#endif

#ifdef CONFIG_PAX_SEGMEXEC
	if (!(elf_ex->e_ident[EI_PAX] & EF_PAX_SEGMEXEC))
		pax_flags |= MF_PAX_SEGMEXEC;
#endif

#ifdef CONFIG_PAX_EMUTRAMP
	if ((pax_flags & (MF_PAX_PAGEEXEC | MF_PAX_SEGMEXEC)) && (elf_ex->e_ident[EI_PAX] & EF_PAX_EMUTRAMP))
		pax_flags |= MF_PAX_EMUTRAMP;
#endif

#ifdef CONFIG_PAX_MPROTECT
	if ((pax_flags & (MF_PAX_PAGEEXEC | MF_PAX_SEGMEXEC)) && !(elf_ex->e_ident[EI_PAX] & EF_PAX_MPROTECT))
		pax_flags |= MF_PAX_MPROTECT;
#endif

#ifdef CONFIG_PAX_ASLR
	if (randomize_va_space && !(elf_ex->e_ident[EI_PAX] & EF_PAX_RANDMMAP))
		pax_flags |= MF_PAX_RANDMMAP;
#endif

#endif

	return pax_flags;

}

static unsigned long pax_parse_pt_pax(const struct elfhdr * const elf_ex, const struct elf_phdr * const elf_phdata)
{

#ifdef CONFIG_PAX_PT_PAX_FLAGS
	unsigned long i;

	for (i = 0UL; i < elf_ex->e_phnum; i++)
		if (elf_phdata[i].p_type == PT_PAX_FLAGS) {
			if (((elf_phdata[i].p_flags & PF_PAGEEXEC) && (elf_phdata[i].p_flags & PF_NOPAGEEXEC)) ||
			    ((elf_phdata[i].p_flags & PF_SEGMEXEC) && (elf_phdata[i].p_flags & PF_NOSEGMEXEC)) ||
			    ((elf_phdata[i].p_flags & PF_EMUTRAMP) && (elf_phdata[i].p_flags & PF_NOEMUTRAMP)) ||
			    ((elf_phdata[i].p_flags & PF_MPROTECT) && (elf_phdata[i].p_flags & PF_NOMPROTECT)) ||
			    ((elf_phdata[i].p_flags & PF_RANDMMAP) && (elf_phdata[i].p_flags & PF_NORANDMMAP)))
				return PAX_PARSE_FLAGS_FALLBACK;

#ifdef CONFIG_PAX_SOFTMODE
			if (pax_softmode)
				return pax_parse_pt_pax_softmode(&elf_phdata[i]);
			else
#endif

				return pax_parse_pt_pax_hardmode(&elf_phdata[i]);
			break;
		}
#endif

	return PAX_PARSE_FLAGS_FALLBACK;
}

static unsigned long pax_parse_xattr_pax(struct file * const file)
{

#ifdef CONFIG_PAX_XATTR_PAX_FLAGS
	ssize_t xattr_size, i;
	unsigned char xattr_value[sizeof("pemrs") - 1];
	unsigned long pax_flags_hardmode = 0UL, pax_flags_softmode = 0UL;

	xattr_size = pax_getxattr(file->f_path.dentry, xattr_value, sizeof xattr_value);
	if (xattr_size < 0 || xattr_size > sizeof xattr_value)
		return PAX_PARSE_FLAGS_FALLBACK;

	for (i = 0; i < xattr_size; i++)
		switch (xattr_value[i]) {
		default:
			return PAX_PARSE_FLAGS_FALLBACK;

#define parse_flag(option1, option2, flag)			\
		case option1:					\
			if (pax_flags_hardmode & MF_PAX_##flag)	\
				return PAX_PARSE_FLAGS_FALLBACK;\
			pax_flags_hardmode |= MF_PAX_##flag;	\
			break;					\
		case option2:					\
			if (pax_flags_softmode & MF_PAX_##flag)	\
				return PAX_PARSE_FLAGS_FALLBACK;\
			pax_flags_softmode |= MF_PAX_##flag;	\
			break;

		parse_flag('p', 'P', PAGEEXEC);
		parse_flag('e', 'E', EMUTRAMP);
		parse_flag('m', 'M', MPROTECT);
		parse_flag('r', 'R', RANDMMAP);
		parse_flag('s', 'S', SEGMEXEC);

#undef parse_flag
		}

	if (pax_flags_hardmode & pax_flags_softmode)
		return PAX_PARSE_FLAGS_FALLBACK;

#ifdef CONFIG_PAX_SOFTMODE
	if (pax_softmode)
		return pax_parse_xattr_pax_softmode(pax_flags_softmode);
	else
#endif

		return pax_parse_xattr_pax_hardmode(pax_flags_hardmode);
#else
	return PAX_PARSE_FLAGS_FALLBACK;
#endif

}

static long pax_parse_pax_flags(const struct elfhdr * const elf_ex, const struct elf_phdr * const elf_phdata, struct file * const file)
{
	unsigned long pax_flags, ei_pax_flags,  pt_pax_flags, xattr_pax_flags;

	pax_flags = pax_parse_defaults();
	ei_pax_flags = pax_parse_ei_pax(elf_ex);
	pt_pax_flags = pax_parse_pt_pax(elf_ex, elf_phdata);
	xattr_pax_flags = pax_parse_xattr_pax(file);

	if (pt_pax_flags != PAX_PARSE_FLAGS_FALLBACK &&
	    xattr_pax_flags != PAX_PARSE_FLAGS_FALLBACK &&
	    pt_pax_flags != xattr_pax_flags)
		return -EINVAL;
	if (xattr_pax_flags != PAX_PARSE_FLAGS_FALLBACK)
		pax_flags = xattr_pax_flags;
	else if (pt_pax_flags != PAX_PARSE_FLAGS_FALLBACK)
		pax_flags = pt_pax_flags;
	else if (ei_pax_flags != PAX_PARSE_FLAGS_FALLBACK)
		pax_flags = ei_pax_flags;

#if defined(CONFIG_PAX_PAGEEXEC) && defined(CONFIG_PAX_SEGMEXEC)
	if ((pax_flags & (MF_PAX_PAGEEXEC | MF_PAX_SEGMEXEC)) == (MF_PAX_PAGEEXEC | MF_PAX_SEGMEXEC)) {
		if ((__supported_pte_mask & _PAGE_NX))
			pax_flags &= ~MF_PAX_SEGMEXEC;
		else
			pax_flags &= ~MF_PAX_PAGEEXEC;
	}
#endif

	if (0 > pax_check_flags(&pax_flags))
		return -EINVAL;

	current->mm->pax_flags = pax_flags;
	return 0;
}
#endif

/*
 * These are the functions used to load ELF style executables and shared
 * libraries.  There is no binary dependent code anywhere else.
 */

#ifndef STACK_RND_MASK
#define STACK_RND_MASK (0x7ff >> (PAGE_SHIFT - 12))	/* 8MB of VA */
#endif

static unsigned long randomize_stack_top(unsigned long stack_top)
{
	unsigned long random_variable = 0;

#ifdef CONFIG_PAX_RANDUSTACK
	if (current->mm->pax_flags & MF_PAX_RANDMMAP)
		return stack_top - current->mm->delta_stack;
#endif

	if ((current->flags & PF_RANDOMIZE) &&
		!(current->personality & ADDR_NO_RANDOMIZE)) {
		random_variable = (unsigned long) get_random_int();
		random_variable &= STACK_RND_MASK;
		random_variable <<= PAGE_SHIFT;
	}
#ifdef CONFIG_STACK_GROWSUP
	return PAGE_ALIGN(stack_top) + random_variable;
#else
	return PAGE_ALIGN(stack_top) - random_variable;
#endif
}

static int load_elf_binary(struct linux_binprm *bprm)
{
	struct file *interpreter = NULL; /* to shut gcc up */
 	unsigned long load_addr = 0, load_bias = 0;
	int load_addr_set = 0;
	char * elf_interpreter = NULL;
	unsigned long error = 0;
	struct elf_phdr *elf_ppnt, *elf_phdata;
	unsigned long elf_bss, elf_brk;
	int retval, i;
	unsigned int size;
	unsigned long elf_entry;
	unsigned long interp_load_addr = 0;
	unsigned long start_code, end_code, start_data, end_data;
	unsigned long reloc_func_desc __maybe_unused = 0;
	int executable_stack = EXSTACK_DEFAULT;
	struct pt_regs *regs = current_pt_regs();
	struct {
		struct elfhdr elf_ex;
		struct elfhdr interp_elf_ex;
	} *loc;
	unsigned long pax_task_size;

	loc = kmalloc(sizeof(*loc), GFP_KERNEL);
	if (!loc) {
		retval = -ENOMEM;
		goto out_ret;
	}
	
	/* Get the exec-header */
	loc->elf_ex = *((struct elfhdr *)bprm->buf);

	retval = -ENOEXEC;
	/* First of all, some simple consistency checks */
	if (memcmp(loc->elf_ex.e_ident, ELFMAG, SELFMAG) != 0)
		goto out;

	if (loc->elf_ex.e_type != ET_EXEC && loc->elf_ex.e_type != ET_DYN)
		goto out;
	if (!elf_check_arch(&loc->elf_ex))
		goto out;
	if (!bprm->file->f_op->mmap)
		goto out;

	/* Now read in all of the header information */
	if (loc->elf_ex.e_phentsize != sizeof(struct elf_phdr))
		goto out;
	if (loc->elf_ex.e_phnum < 1 ||
	 	loc->elf_ex.e_phnum > 65536U / sizeof(struct elf_phdr))
		goto out;
	size = loc->elf_ex.e_phnum * sizeof(struct elf_phdr);
	retval = -ENOMEM;
	elf_phdata = kmalloc(size, GFP_KERNEL);
	if (!elf_phdata)
		goto out;

	retval = kernel_read(bprm->file, loc->elf_ex.e_phoff,
			     (char *)elf_phdata, size);
	if (retval != size) {
		if (retval >= 0)
			retval = -EIO;
		goto out_free_ph;
	}

	elf_ppnt = elf_phdata;
	elf_bss = 0;
	elf_brk = 0;

	start_code = ~0UL;
	end_code = 0;
	start_data = 0;
	end_data = 0;

	for (i = 0; i < loc->elf_ex.e_phnum; i++) {
		if (elf_ppnt->p_type == PT_INTERP) {
			/* This is the program interpreter used for
			 * shared libraries - for now assume that this
			 * is an a.out format binary
			 */
			retval = -ENOEXEC;
			if (elf_ppnt->p_filesz > PATH_MAX || 
			    elf_ppnt->p_filesz < 2)
				goto out_free_ph;

			retval = -ENOMEM;
			elf_interpreter = kmalloc(elf_ppnt->p_filesz,
						  GFP_KERNEL);
			if (!elf_interpreter)
				goto out_free_ph;

			retval = kernel_read(bprm->file, elf_ppnt->p_offset,
					     elf_interpreter,
					     elf_ppnt->p_filesz);
			if (retval != elf_ppnt->p_filesz) {
				if (retval >= 0)
					retval = -EIO;
				goto out_free_interp;
			}
			/* make sure path is NULL terminated */
			retval = -ENOEXEC;
			if (elf_interpreter[elf_ppnt->p_filesz - 1] != '\0')
				goto out_free_interp;

			interpreter = open_exec(elf_interpreter);
			retval = PTR_ERR(interpreter);
			if (IS_ERR(interpreter))
				goto out_free_interp;

			/*
			 * If the binary is not readable then enforce
			 * mm->dumpable = 0 regardless of the interpreter's
			 * permissions.
			 */
			would_dump(bprm, interpreter);

			retval = kernel_read(interpreter, 0, bprm->buf,
					     BINPRM_BUF_SIZE);
			if (retval != BINPRM_BUF_SIZE) {
				if (retval >= 0)
					retval = -EIO;
				goto out_free_dentry;
			}

			/* Get the exec headers */
			loc->interp_elf_ex = *((struct elfhdr *)bprm->buf);
			break;
		}
		elf_ppnt++;
	}

	elf_ppnt = elf_phdata;
	for (i = 0; i < loc->elf_ex.e_phnum; i++, elf_ppnt++)
		if (elf_ppnt->p_type == PT_GNU_STACK) {
			if (elf_ppnt->p_flags & PF_X)
				executable_stack = EXSTACK_ENABLE_X;
			else
				executable_stack = EXSTACK_DISABLE_X;
			break;
		}

	/* Some simple consistency checks for the interpreter */
	if (elf_interpreter) {
		retval = -ELIBBAD;
		/* Not an ELF interpreter */
		if (memcmp(loc->interp_elf_ex.e_ident, ELFMAG, SELFMAG) != 0)
			goto out_free_dentry;
		/* Verify the interpreter has a valid arch */
		if (!elf_check_arch(&loc->interp_elf_ex))
			goto out_free_dentry;
	}

	/* Flush all traces of the currently running executable */
	retval = flush_old_exec(bprm);
	if (retval)
		goto out_free_dentry;

	/* Do this immediately, since STACK_TOP as used in setup_arg_pages
	   may depend on the personality.  */
	SET_PERSONALITY(loc->elf_ex);

#if defined(CONFIG_PAX_NOEXEC) || defined(CONFIG_PAX_ASLR)
	current->mm->pax_flags = 0UL;
#endif

#ifdef CONFIG_PAX_DLRESOLVE
	current->mm->call_dl_resolve = 0UL;
#endif

#if defined(CONFIG_PPC32) && defined(CONFIG_PAX_EMUSIGRT)
	current->mm->call_syscall = 0UL;
#endif

#ifdef CONFIG_PAX_ASLR
	current->mm->delta_mmap = 0UL;
	current->mm->delta_stack = 0UL;
#endif

#if defined(CONFIG_PAX_NOEXEC) || defined(CONFIG_PAX_ASLR)
	if (0 > pax_parse_pax_flags(&loc->elf_ex, elf_phdata, bprm->file)) {
		send_sig(SIGKILL, current, 0);
		goto out_free_dentry;
	}
#endif

#ifdef CONFIG_PAX_HAVE_ACL_FLAGS
	pax_set_initial_flags(bprm);
#elif defined(CONFIG_PAX_HOOK_ACL_FLAGS)
	if (pax_set_initial_flags_func)
		(pax_set_initial_flags_func)(bprm);
#endif

#ifdef CONFIG_ARCH_TRACK_EXEC_LIMIT
	if ((current->mm->pax_flags & MF_PAX_PAGEEXEC) && !(__supported_pte_mask & _PAGE_NX)) {
		current->mm->context.user_cs_limit = PAGE_SIZE;
		current->mm->def_flags |= VM_PAGEEXEC | VM_NOHUGEPAGE;
	}
#endif

#ifdef CONFIG_PAX_SEGMEXEC
	if (current->mm->pax_flags & MF_PAX_SEGMEXEC) {
		current->mm->context.user_cs_base = SEGMEXEC_TASK_SIZE;
		current->mm->context.user_cs_limit = TASK_SIZE-SEGMEXEC_TASK_SIZE;
		pax_task_size = SEGMEXEC_TASK_SIZE;
		current->mm->def_flags |= VM_NOHUGEPAGE;
	} else
#endif

	pax_task_size = TASK_SIZE;

#if defined(CONFIG_ARCH_TRACK_EXEC_LIMIT) || defined(CONFIG_PAX_SEGMEXEC)
	if (current->mm->pax_flags & (MF_PAX_PAGEEXEC | MF_PAX_SEGMEXEC)) {
		set_user_cs(current->mm->context.user_cs_base, current->mm->context.user_cs_limit, get_cpu());
		put_cpu();
	}
#endif

#ifdef CONFIG_PAX_ASLR
	if (current->mm->pax_flags & MF_PAX_RANDMMAP) {
		current->mm->delta_mmap = (pax_get_random_long() & ((1UL << PAX_DELTA_MMAP_LEN)-1)) << PAGE_SHIFT;
		current->mm->delta_stack = (pax_get_random_long() & ((1UL << PAX_DELTA_STACK_LEN)-1)) << PAGE_SHIFT;
	}
#endif

#if defined(CONFIG_PAX_PAGEEXEC) || defined(CONFIG_PAX_SEGMEXEC)
	if (current->mm->pax_flags & (MF_PAX_PAGEEXEC | MF_PAX_SEGMEXEC)) {
		executable_stack = EXSTACK_DISABLE_X;
		current->personality &= ~READ_IMPLIES_EXEC;
	} else
#endif

	if (elf_read_implies_exec(loc->elf_ex, executable_stack))
		current->personality |= READ_IMPLIES_EXEC;

	if (!(current->personality & ADDR_NO_RANDOMIZE) && randomize_va_space)
		current->flags |= PF_RANDOMIZE;

	setup_new_exec(bprm);

	/* Do this so that we can load the interpreter, if need be.  We will
	   change some of these later */
	retval = setup_arg_pages(bprm, randomize_stack_top(STACK_TOP),
				 executable_stack);
	if (retval < 0)
		goto out_free_dentry;
	
	current->mm->start_stack = bprm->p;

	/* Now we do a little grungy work by mmapping the ELF image into
	   the correct location in memory. */
	for(i = 0, elf_ppnt = elf_phdata;
	    i < loc->elf_ex.e_phnum; i++, elf_ppnt++) {
		int elf_prot = 0, elf_flags;
		unsigned long k, vaddr;
		unsigned long total_size = 0;

		if (elf_ppnt->p_type != PT_LOAD)
			continue;

		if (unlikely (elf_brk > elf_bss)) {
			unsigned long nbyte;
	            
			/* There was a PT_LOAD segment with p_memsz > p_filesz
			   before this one. Map anonymous pages, if needed,
			   and clear the area.  */
			retval = set_brk(elf_bss + load_bias,
					 elf_brk + load_bias);
			if (retval)
				goto out_free_dentry;
			nbyte = ELF_PAGEOFFSET(elf_bss);
			if (nbyte) {
				nbyte = ELF_MIN_ALIGN - nbyte;
				if (nbyte > elf_brk - elf_bss)
					nbyte = elf_brk - elf_bss;
				if (clear_user((void __user *)elf_bss +
							load_bias, nbyte)) {
					/*
					 * This bss-zeroing can fail if the ELF
					 * file specifies odd protections. So
					 * we don't check the return value
					 */
				}
			}
		}

		if (elf_ppnt->p_flags & PF_R)
			elf_prot |= PROT_READ;
		if (elf_ppnt->p_flags & PF_W)
			elf_prot |= PROT_WRITE;
		if (elf_ppnt->p_flags & PF_X)
			elf_prot |= PROT_EXEC;

		elf_flags = MAP_PRIVATE | MAP_DENYWRITE | MAP_EXECUTABLE;

		vaddr = elf_ppnt->p_vaddr;
		if (loc->elf_ex.e_type == ET_EXEC || load_addr_set) {
			elf_flags |= MAP_FIXED;
		} else if (loc->elf_ex.e_type == ET_DYN) {
			/* Try and get dynamic programs out of the way of the
			 * default mmap base, as well as whatever program they
			 * might try to exec.  This is because the brk will
			 * follow the loader, and is not movable.  */
			load_bias = ELF_ET_DYN_BASE - vaddr;
			if (current->flags & PF_RANDOMIZE)
				load_bias += arch_mmap_rnd();
			load_bias = ELF_PAGESTART(load_bias);

#ifdef CONFIG_PAX_RANDMMAP
			/* PaX: randomize base address at the default exe base if requested */
			if ((current->mm->pax_flags & MF_PAX_RANDMMAP) && elf_interpreter) {
#ifdef CONFIG_SPARC64
				load_bias = (pax_get_random_long() & ((1UL << PAX_DELTA_MMAP_LEN) - 1)) << (PAGE_SHIFT+1);
#else
				load_bias = (pax_get_random_long() & ((1UL << PAX_DELTA_MMAP_LEN) - 1)) << PAGE_SHIFT;
#endif
				load_bias = ELF_PAGESTART(PAX_ELF_ET_DYN_BASE - vaddr + load_bias);
				elf_flags |= MAP_FIXED;
			}
#endif

			total_size = total_mapping_size(elf_phdata,
							loc->elf_ex.e_phnum);
			if (!total_size) {
				retval = -EINVAL;
				goto out_free_dentry;
			}
		}

		error = elf_map(bprm->file, load_bias + vaddr, elf_ppnt,
				elf_prot, elf_flags, total_size);
		if (BAD_ADDR(error)) {
			retval = IS_ERR((void *)error) ?
				PTR_ERR((void*)error) : -EINVAL;
			goto out_free_dentry;
		}

		if (!load_addr_set) {
			load_addr_set = 1;
			load_addr = (elf_ppnt->p_vaddr - elf_ppnt->p_offset);
			if (loc->elf_ex.e_type == ET_DYN) {
				load_bias += error -
				             ELF_PAGESTART(load_bias + vaddr);
				load_addr += load_bias;
				reloc_func_desc = load_bias;
			}
		}
		k = elf_ppnt->p_vaddr;
		if (k < start_code)
			start_code = k;
		if (start_data < k)
			start_data = k;

		/*
		 * Check to see if the section's size will overflow the
		 * allowed task size. Note that p_filesz must always be
		 * <= p_memsz so it is only necessary to check p_memsz.
		 */
		if (k >= pax_task_size || elf_ppnt->p_filesz > elf_ppnt->p_memsz ||
		    elf_ppnt->p_memsz > pax_task_size ||
		    pax_task_size - elf_ppnt->p_memsz < k) {
			/* set_brk can never work. Avoid overflows. */
			retval = -EINVAL;
			goto out_free_dentry;
		}

		k = elf_ppnt->p_vaddr + elf_ppnt->p_filesz;

		if (k > elf_bss)
			elf_bss = k;
		if ((elf_ppnt->p_flags & PF_X) && end_code < k)
			end_code = k;
		if (end_data < k)
			end_data = k;
		k = elf_ppnt->p_vaddr + elf_ppnt->p_memsz;
		if (k > elf_brk)
			elf_brk = k;
	}

	loc->elf_ex.e_entry += load_bias;
	elf_bss += load_bias;
	elf_brk += load_bias;
	start_code += load_bias;
	end_code += load_bias;
	start_data += load_bias;
	end_data += load_bias;

	/* Calling set_brk effectively mmaps the pages that we need
	 * for the bss and break sections.  We must do this before
	 * mapping in the interpreter, to make sure it doesn't wind
	 * up getting placed where the bss needs to go.
	 */
	retval = set_brk(elf_bss, elf_brk);
	if (retval)
		goto out_free_dentry;
	if (likely(elf_bss != elf_brk) && unlikely(padzero(elf_bss))) {
		/*
		 * This bss-zeroing can fail if the ELF
		 * file specifies odd protections. So
		 * we don't check the return value
		 */
	}

#ifdef CONFIG_PAX_RANDMMAP
	if (current->mm->pax_flags & MF_PAX_RANDMMAP) {
		unsigned long start, size, flags;
		vm_flags_t vm_flags;

		start = ELF_PAGEALIGN(elf_brk);
		size = PAGE_SIZE + ((pax_get_random_long() & ((1UL << 22) - 1UL)) << 4);
		flags = MAP_FIXED | MAP_PRIVATE;
		vm_flags = VM_DONTEXPAND | VM_DONTDUMP;

		down_write(&current->mm->mmap_sem);
		start = get_unmapped_area(NULL, start, PAGE_ALIGN(size), 0, flags);
		retval = -ENOMEM;
		if (!IS_ERR_VALUE(start) && !find_vma_intersection(current->mm, start, start + size + PAGE_SIZE)) {
//			if (current->personality & ADDR_NO_RANDOMIZE)
//				vm_flags |= VM_READ | VM_MAYREAD;
			start = mmap_region(NULL, start, PAGE_ALIGN(size), vm_flags, 0);
			retval = IS_ERR_VALUE(start) ? start : 0;
		}
		up_write(&current->mm->mmap_sem);
		if (retval == 0)
			retval = set_brk(start + size, PAGE_ALIGN(start + size));
		if (retval < 0) {
			send_sig(SIGKILL, current, 0);
			goto out_free_dentry;
		}
	}
#endif

	if (elf_interpreter) {
		elf_entry = load_elf_interp(&loc->interp_elf_ex,
					    interpreter,
					    load_bias);
		if (!IS_ERR((void *)elf_entry)) {
			/*
			 * load_elf_interp() returns relocation
			 * adjustment
			 */
			interp_load_addr = elf_entry;
			elf_entry += loc->interp_elf_ex.e_entry;
		}
		if (BAD_ADDR(elf_entry)) {
			retval = IS_ERR((void *)elf_entry) ?
					(int)elf_entry : -EINVAL;
			goto out_free_dentry;
		}
		reloc_func_desc = interp_load_addr;

		allow_write_access(interpreter);
		fput(interpreter);
		kfree(elf_interpreter);
	} else {
		elf_entry = loc->elf_ex.e_entry;
		if (BAD_ADDR(elf_entry)) {
			retval = -EINVAL;
			goto out_free_dentry;
		}
	}

	kfree(elf_phdata);

	set_binfmt(&elf_format);

#ifdef ARCH_HAS_SETUP_ADDITIONAL_PAGES
	retval = arch_setup_additional_pages(bprm, !!elf_interpreter);
	if (retval < 0)
		goto out;
#endif /* ARCH_HAS_SETUP_ADDITIONAL_PAGES */

	install_exec_creds(bprm);
	retval = create_elf_tables(bprm, &loc->elf_ex,
			  load_addr, interp_load_addr);
	if (retval < 0)
		goto out;
	/* N.B. passed_fileno might not be initialized? */
	current->mm->end_code = end_code;
	current->mm->start_code = start_code;
	current->mm->start_data = start_data;
	current->mm->end_data = end_data;
	current->mm->start_stack = bprm->p;

#ifdef arch_randomize_brk
	if ((current->flags & PF_RANDOMIZE) && (randomize_va_space > 1)) {
		current->mm->brk = current->mm->start_brk =
			arch_randomize_brk(current->mm);
#ifdef CONFIG_COMPAT_BRK
		current->brk_randomized = 1;
#endif
	}
#endif

	if (current->personality & MMAP_PAGE_ZERO) {
		/* Why this, you ask???  Well SVr4 maps page 0 as read-only,
		   and some applications "depend" upon this behavior.
		   Since we do not have the power to recompile these, we
		   emulate the SVr4 behavior. Sigh. */
		error = vm_mmap(NULL, 0, PAGE_SIZE, PROT_READ | PROT_EXEC,
				MAP_FIXED | MAP_PRIVATE, 0);
	}

#ifdef ELF_PLAT_INIT
	/*
	 * The ABI may specify that certain registers be set up in special
	 * ways (on i386 %edx is the address of a DT_FINI function, for
	 * example.  In addition, it may also specify (eg, PowerPC64 ELF)
	 * that the e_entry field is the address of the function descriptor
	 * for the startup routine, rather than the address of the startup
	 * routine itself.  This macro performs whatever initialization to
	 * the regs structure is required as well as any relocations to the
	 * function descriptor entries when executing dynamically links apps.
	 */
	ELF_PLAT_INIT(regs, reloc_func_desc);
#endif

	start_thread(regs, elf_entry, bprm->p);
	retval = 0;
out:
	kfree(loc);
out_ret:
	return retval;

	/* error cleanup */
out_free_dentry:
	allow_write_access(interpreter);
	if (interpreter)
		fput(interpreter);
out_free_interp:
	kfree(elf_interpreter);
out_free_ph:
	kfree(elf_phdata);
	goto out;
}

#ifdef CONFIG_USELIB
/* This is really simpleminded and specialized - we are loading an
   a.out library that is given an ELF header. */
static int load_elf_library(struct file *file)
{
	struct elf_phdr *elf_phdata;
	struct elf_phdr *eppnt;
	unsigned long elf_bss, bss, len;
	int retval, error, i, j;
	struct elfhdr elf_ex;

	error = -ENOEXEC;
	retval = kernel_read(file, 0, (char *)&elf_ex, sizeof(elf_ex));
	if (retval != sizeof(elf_ex))
		goto out;

	if (memcmp(elf_ex.e_ident, ELFMAG, SELFMAG) != 0)
		goto out;

	/* First of all, some simple consistency checks */
	if (elf_ex.e_type != ET_EXEC || elf_ex.e_phnum > 2 ||
	    !elf_check_arch(&elf_ex) || !file->f_op->mmap)
		goto out;

	/* Now read in all of the header information */

	j = sizeof(struct elf_phdr) * elf_ex.e_phnum;
	/* j < ELF_MIN_ALIGN because elf_ex.e_phnum <= 2 */

	error = -ENOMEM;
	elf_phdata = kmalloc(j, GFP_KERNEL);
	if (!elf_phdata)
		goto out;

	eppnt = elf_phdata;
	error = -ENOEXEC;
	retval = kernel_read(file, elf_ex.e_phoff, (char *)eppnt, j);
	if (retval != j)
		goto out_free_ph;

	for (j = 0, i = 0; i<elf_ex.e_phnum; i++)
		if ((eppnt + i)->p_type == PT_LOAD)
			j++;
	if (j != 1)
		goto out_free_ph;

	while (eppnt->p_type != PT_LOAD)
		eppnt++;

	/* Now use mmap to map the library into memory. */
	error = vm_mmap(file,
			ELF_PAGESTART(eppnt->p_vaddr),
			(eppnt->p_filesz +
			 ELF_PAGEOFFSET(eppnt->p_vaddr)),
			PROT_READ | PROT_WRITE | PROT_EXEC,
			MAP_FIXED | MAP_PRIVATE | MAP_DENYWRITE,
			(eppnt->p_offset -
			 ELF_PAGEOFFSET(eppnt->p_vaddr)));
	if (error != ELF_PAGESTART(eppnt->p_vaddr))
		goto out_free_ph;

	elf_bss = eppnt->p_vaddr + eppnt->p_filesz;
	if (padzero(elf_bss)) {
		error = -EFAULT;
		goto out_free_ph;
	}

	len = ELF_PAGESTART(eppnt->p_filesz + eppnt->p_vaddr +
			    ELF_MIN_ALIGN - 1);
	bss = eppnt->p_memsz + eppnt->p_vaddr;
	if (bss > len)
		vm_brk(len, bss - len);
	error = 0;

out_free_ph:
	kfree(elf_phdata);
out:
	return error;
}
#endif /* #ifdef CONFIG_USELIB */

#ifdef CONFIG_ELF_CORE
/*
 * ELF core dumper
 *
 * Modelled on fs/exec.c:aout_core_dump()
 * Jeremy Fitzhardinge <jeremy@sw.oz.au>
 */

/*
 * The purpose of always_dump_vma() is to make sure that special kernel mappings
 * that are useful for post-mortem analysis are included in every core dump.
 * In that way we ensure that the core dump is fully interpretable later
 * without matching up the same kernel and hardware config to see what PC values
 * meant. These special mappings include - vDSO, vsyscall, and other
 * architecture specific mappings
 */
static bool always_dump_vma(struct vm_area_struct *vma)
{
	/* Any vsyscall mappings? */
	if (vma == get_gate_vma(vma->vm_mm))
		return true;

	/*
	 * Assume that all vmas with a .name op should always be dumped.
	 * If this changes, a new vm_ops field can easily be added.
	 */
	if (vma->vm_ops && vma->vm_ops->name && vma->vm_ops->name(vma))
		return true;

	/*
	 * arch_vma_name() returns non-NULL for special architecture mappings,
	 * such as vDSO sections.
	 */
	if (arch_vma_name(vma))
		return true;

	return false;
}

/*
 * Decide what to dump of a segment, part, all or none.
 */
static unsigned long vma_dump_size(struct vm_area_struct *vma,
				   unsigned long mm_flags, long signr)
{
#define FILTER(type)	(mm_flags & (1UL << MMF_DUMP_##type))

	/* always dump the vdso and vsyscall sections */
	if (always_dump_vma(vma))
		goto whole;

	if (vma->vm_flags & VM_DONTDUMP)
		return 0;

	/* Hugetlb memory check */
	if (vma->vm_flags & VM_HUGETLB) {
		if ((vma->vm_flags & VM_SHARED) && FILTER(HUGETLB_SHARED))
			goto whole;
		if (!(vma->vm_flags & VM_SHARED) && FILTER(HUGETLB_PRIVATE))
			goto whole;
		return 0;
	}

	/* Do not dump I/O mapped devices or special mappings */
	if (vma->vm_flags & VM_IO)
		return 0;

	/* By default, dump shared memory if mapped from an anonymous file. */
	if (vma->vm_flags & VM_SHARED) {
		if (file_inode(vma->vm_file)->i_nlink == 0 ?
		    FILTER(ANON_SHARED) : FILTER(MAPPED_SHARED))
			goto whole;
		return 0;
	}

	/* Dump segments that have been written to.  */
	if (vma->anon_vma && FILTER(ANON_PRIVATE))
		goto whole;
	if (vma->vm_file == NULL)
		return 0;

	if (signr == SIGKILL || FILTER(MAPPED_PRIVATE))
		goto whole;

	/*
	 * If this looks like the beginning of a DSO or executable mapping,
	 * check for an ELF header.  If we find one, dump the first page to
	 * aid in determining what was mapped here.
	 */
	if (FILTER(ELF_HEADERS) &&
	    vma->vm_pgoff == 0 && (vma->vm_flags & VM_READ)) {
		u32 __user *header = (u32 __user *) vma->vm_start;
		u32 word;
		mm_segment_t fs = get_fs();
		/*
		 * Doing it this way gets the constant folded by GCC.
		 */
		union {
			u32 cmp;
			char elfmag[SELFMAG];
		} magic;
		BUILD_BUG_ON(SELFMAG != sizeof word);
		magic.elfmag[EI_MAG0] = ELFMAG0;
		magic.elfmag[EI_MAG1] = ELFMAG1;
		magic.elfmag[EI_MAG2] = ELFMAG2;
		magic.elfmag[EI_MAG3] = ELFMAG3;
		/*
		 * Switch to the user "segment" for get_user(),
		 * then put back what elf_core_dump() had in place.
		 */
		set_fs(USER_DS);
		if (unlikely(get_user(word, header)))
			word = 0;
		set_fs(fs);
		if (word == magic.cmp)
			return PAGE_SIZE;
	}

#undef	FILTER

	return 0;

whole:
	return vma->vm_end - vma->vm_start;
}

/* An ELF note in memory */
struct memelfnote
{
	const char *name;
	int type;
	unsigned int datasz;
	void *data;
};

static int notesize(struct memelfnote *en)
{
	int sz;

	sz = sizeof(struct elf_note);
	sz += roundup(strlen(en->name) + 1, 4);
	sz += roundup(en->datasz, 4);

	return sz;
}

static int writenote(struct memelfnote *men, struct coredump_params *cprm)
{
	struct elf_note en;
	en.n_namesz = strlen(men->name) + 1;
	en.n_descsz = men->datasz;
	en.n_type = men->type;

	return dump_emit(cprm, &en, sizeof(en)) &&
	    dump_emit(cprm, men->name, en.n_namesz) && dump_align(cprm, 4) &&
	    dump_emit(cprm, men->data, men->datasz) && dump_align(cprm, 4);
}

static void fill_elf_header(struct elfhdr *elf, int segs,
			    u16 machine, u32 flags)
{
	memset(elf, 0, sizeof(*elf));

	memcpy(elf->e_ident, ELFMAG, SELFMAG);
	elf->e_ident[EI_CLASS] = ELF_CLASS;
	elf->e_ident[EI_DATA] = ELF_DATA;
	elf->e_ident[EI_VERSION] = EV_CURRENT;
	elf->e_ident[EI_OSABI] = ELF_OSABI;

	elf->e_type = ET_CORE;
	elf->e_machine = machine;
	elf->e_version = EV_CURRENT;
	elf->e_phoff = sizeof(struct elfhdr);
	elf->e_flags = flags;
	elf->e_ehsize = sizeof(struct elfhdr);
	elf->e_phentsize = sizeof(struct elf_phdr);
	elf->e_phnum = segs;

	return;
}

static void fill_elf_note_phdr(struct elf_phdr *phdr, int sz, loff_t offset)
{
	phdr->p_type = PT_NOTE;
	phdr->p_offset = offset;
	phdr->p_vaddr = 0;
	phdr->p_paddr = 0;
	phdr->p_filesz = sz;
	phdr->p_memsz = 0;
	phdr->p_flags = 0;
	phdr->p_align = 0;
	return;
}

static void fill_note(struct memelfnote *note, const char *name, int type, 
		unsigned int sz, void *data)
{
	note->name = name;
	note->type = type;
	note->datasz = sz;
	note->data = data;
	return;
}

/*
 * fill up all the fields in prstatus from the given task struct, except
 * registers which need to be filled up separately.
 */
static void fill_prstatus(struct elf_prstatus *prstatus,
		struct task_struct *p, long signr)
{
	prstatus->pr_info.si_signo = prstatus->pr_cursig = signr;
	prstatus->pr_sigpend = p->pending.signal.sig[0];
	prstatus->pr_sighold = p->blocked.sig[0];
	rcu_read_lock();
	prstatus->pr_ppid = task_pid_vnr(rcu_dereference(p->real_parent));
	rcu_read_unlock();
	prstatus->pr_pid = task_pid_vnr(p);
	prstatus->pr_pgrp = task_pgrp_vnr(p);
	prstatus->pr_sid = task_session_vnr(p);
	if (thread_group_leader(p)) {
		struct task_cputime cputime;

		/*
		 * This is the record for the group leader.  It shows the
		 * group-wide total, not its individual thread total.
		 */
		thread_group_cputime(p, &cputime);
		cputime_to_timeval(cputime.utime, &prstatus->pr_utime);
		cputime_to_timeval(cputime.stime, &prstatus->pr_stime);
	} else {
		cputime_t utime, stime;

		task_cputime(p, &utime, &stime);
		cputime_to_timeval(utime, &prstatus->pr_utime);
		cputime_to_timeval(stime, &prstatus->pr_stime);
	}
	cputime_to_timeval(p->signal->cutime, &prstatus->pr_cutime);
	cputime_to_timeval(p->signal->cstime, &prstatus->pr_cstime);
}

static int fill_psinfo(struct elf_prpsinfo *psinfo, struct task_struct *p,
		       struct mm_struct *mm)
{
	const struct cred *cred;
	unsigned int i, len;
	
	/* first copy the parameters from user space */
	memset(psinfo, 0, sizeof(struct elf_prpsinfo));

	len = mm->arg_end - mm->arg_start;
	if (len >= ELF_PRARGSZ)
		len = ELF_PRARGSZ-1;
	if (copy_from_user(&psinfo->pr_psargs,
		           (const char __user *)mm->arg_start, len))
		return -EFAULT;
	for(i = 0; i < len; i++)
		if (psinfo->pr_psargs[i] == 0)
			psinfo->pr_psargs[i] = ' ';
	psinfo->pr_psargs[len] = 0;

	rcu_read_lock();
	psinfo->pr_ppid = task_pid_vnr(rcu_dereference(p->real_parent));
	rcu_read_unlock();
	psinfo->pr_pid = task_pid_vnr(p);
	psinfo->pr_pgrp = task_pgrp_vnr(p);
	psinfo->pr_sid = task_session_vnr(p);

	i = p->state ? ffz(~p->state) + 1 : 0;
	psinfo->pr_state = i;
	psinfo->pr_sname = (i > 5) ? '.' : "RSDTZW"[i];
	psinfo->pr_zomb = psinfo->pr_sname == 'Z';
	psinfo->pr_nice = task_nice(p);
	psinfo->pr_flag = p->flags;
	rcu_read_lock();
	cred = __task_cred(p);
	SET_UID(psinfo->pr_uid, from_kuid_munged(cred->user_ns, cred->uid));
	SET_GID(psinfo->pr_gid, from_kgid_munged(cred->user_ns, cred->gid));
	rcu_read_unlock();
	strncpy(psinfo->pr_fname, p->comm, sizeof(psinfo->pr_fname));
	
	return 0;
}

static void fill_auxv_note(struct memelfnote *note, struct mm_struct *mm)
{
	elf_addr_t *auxv = (elf_addr_t *) mm->saved_auxv;
	int i = 0;
	do {
		i += 2;
	} while (auxv[i - 2] != AT_NULL);
	fill_note(note, "CORE", NT_AUXV, i * sizeof(elf_addr_t), auxv);
}

static void fill_siginfo_note(struct memelfnote *note, user_siginfo_t *csigdata,
		const siginfo_t *siginfo)
{
	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);
	copy_siginfo_to_user((user_siginfo_t __force_user *) csigdata, siginfo);
	set_fs(old_fs);
	fill_note(note, "CORE", NT_SIGINFO, sizeof(*csigdata), csigdata);
}

#define MAX_FILE_NOTE_SIZE (4*1024*1024)
/*
 * Format of NT_FILE note:
 *
 * long count     -- how many files are mapped
 * long page_size -- units for file_ofs
 * array of [COUNT] elements of
 *   long start
 *   long end
 *   long file_ofs
 * followed by COUNT filenames in ASCII: "FILE1" NUL "FILE2" NUL...
 */
static int fill_files_note(struct memelfnote *note)
{
	struct vm_area_struct *vma;
	unsigned count, size, names_ofs, remaining, n;
	user_long_t *data;
	user_long_t *start_end_ofs;
	char *name_base, *name_curpos;

	/* *Estimated* file count and total data size needed */
	count = current->mm->map_count;
	size = count * 64;

	names_ofs = (2 + 3 * count) * sizeof(data[0]);
 alloc:
	if (size >= MAX_FILE_NOTE_SIZE) /* paranoia check */
		return -EINVAL;
	size = round_up(size, PAGE_SIZE);
	data = vmalloc(size);
	if (!data)
		return -ENOMEM;

	start_end_ofs = data + 2;
	name_base = name_curpos = ((char *)data) + names_ofs;
	remaining = size - names_ofs;
	count = 0;
	for (vma = current->mm->mmap; vma != NULL; vma = vma->vm_next) {
		struct file *file;
		const char *filename;

		file = vma->vm_file;
		if (!file)
			continue;
		filename = d_path(&file->f_path, name_curpos, remaining);
		if (IS_ERR(filename)) {
			if (PTR_ERR(filename) == -ENAMETOOLONG) {
				vfree(data);
				size = size * 5 / 4;
				goto alloc;
			}
			continue;
		}

		/* d_path() fills at the end, move name down */
		/* n = strlen(filename) + 1: */
		n = (name_curpos + remaining) - filename;
		remaining = filename - name_curpos;
		memmove(name_curpos, filename, n);
		name_curpos += n;

		*start_end_ofs++ = vma->vm_start;
		*start_end_ofs++ = vma->vm_end;
		*start_end_ofs++ = vma->vm_pgoff;
		count++;
	}

	/* Now we know exact count of files, can store it */
	data[0] = count;
	data[1] = PAGE_SIZE;
	/*
	 * Count usually is less than current->mm->map_count,
	 * we need to move filenames down.
	 */
	n = current->mm->map_count - count;
	if (n != 0) {
		unsigned shift_bytes = n * 3 * sizeof(data[0]);
		memmove(name_base - shift_bytes, name_base,
			name_curpos - name_base);
		name_curpos -= shift_bytes;
	}

	size = name_curpos - (char *)data;
	fill_note(note, "CORE", NT_FILE, size, data);
	return 0;
}

#ifdef CORE_DUMP_USE_REGSET
#include <linux/regset.h>

struct elf_thread_core_info {
	struct elf_thread_core_info *next;
	struct task_struct *task;
	struct elf_prstatus prstatus;
	struct memelfnote notes[0];
};

struct elf_note_info {
	struct elf_thread_core_info *thread;
	struct memelfnote psinfo;
	struct memelfnote signote;
	struct memelfnote auxv;
	struct memelfnote files;
	user_siginfo_t csigdata;
	size_t size;
	int thread_notes;
};

/*
 * When a regset has a writeback hook, we call it on each thread before
 * dumping user memory.  On register window machines, this makes sure the
 * user memory backing the register data is up to date before we read it.
 */
static void do_thread_regset_writeback(struct task_struct *task,
				       const struct user_regset *regset)
{
	if (regset->writeback)
		regset->writeback(task, regset, 1);
}

#ifndef PR_REG_SIZE
#define PR_REG_SIZE(S) sizeof(S)
#endif

#ifndef PRSTATUS_SIZE
#define PRSTATUS_SIZE(S) sizeof(S)
#endif

#ifndef PR_REG_PTR
#define PR_REG_PTR(S) (&((S)->pr_reg))
#endif

#ifndef SET_PR_FPVALID
#define SET_PR_FPVALID(S, V) ((S)->pr_fpvalid = (V))
#endif

static int fill_thread_core_info(struct elf_thread_core_info *t,
				 const struct user_regset_view *view,
				 long signr, size_t *total)
{
	unsigned int i;

	/*
	 * NT_PRSTATUS is the one special case, because the regset data
	 * goes into the pr_reg field inside the note contents, rather
	 * than being the whole note contents.  We fill the reset in here.
	 * We assume that regset 0 is NT_PRSTATUS.
	 */
	fill_prstatus(&t->prstatus, t->task, signr);
	(void) view->regsets[0].get(t->task, &view->regsets[0],
				    0, PR_REG_SIZE(t->prstatus.pr_reg),
				    PR_REG_PTR(&t->prstatus), NULL);

	fill_note(&t->notes[0], "CORE", NT_PRSTATUS,
		  PRSTATUS_SIZE(t->prstatus), &t->prstatus);
	*total += notesize(&t->notes[0]);

	do_thread_regset_writeback(t->task, &view->regsets[0]);

	/*
	 * Each other regset might generate a note too.  For each regset
	 * that has no core_note_type or is inactive, we leave t->notes[i]
	 * all zero and we'll know to skip writing it later.
	 */
	for (i = 1; i < view->n; ++i) {
		const struct user_regset *regset = &view->regsets[i];
		do_thread_regset_writeback(t->task, regset);
		if (regset->core_note_type && regset->get &&
		    (!regset->active || regset->active(t->task, regset))) {
			int ret;
			size_t size = regset->n * regset->size;
			void *data = kmalloc(size, GFP_KERNEL);
			if (unlikely(!data))
				return 0;
			ret = regset->get(t->task, regset,
					  0, size, data, NULL);
			if (unlikely(ret))
				kfree(data);
			else {
				if (regset->core_note_type != NT_PRFPREG)
					fill_note(&t->notes[i], "LINUX",
						  regset->core_note_type,
						  size, data);
				else {
					SET_PR_FPVALID(&t->prstatus, 1);
					fill_note(&t->notes[i], "CORE",
						  NT_PRFPREG, size, data);
				}
				*total += notesize(&t->notes[i]);
			}
		}
	}

	return 1;
}

static int fill_note_info(struct elfhdr *elf, int phdrs,
			  struct elf_note_info *info,
			  const siginfo_t *siginfo, struct pt_regs *regs)
{
	struct task_struct *dump_task = current;
	const struct user_regset_view *view = task_user_regset_view(dump_task);
	struct elf_thread_core_info *t;
	struct elf_prpsinfo *psinfo;
	struct core_thread *ct;
	unsigned int i;

	info->size = 0;
	info->thread = NULL;

	psinfo = kmalloc(sizeof(*psinfo), GFP_KERNEL);
	if (psinfo == NULL) {
		info->psinfo.data = NULL; /* So we don't free this wrongly */
		return 0;
	}

	fill_note(&info->psinfo, "CORE", NT_PRPSINFO, sizeof(*psinfo), psinfo);

	/*
	 * Figure out how many notes we're going to need for each thread.
	 */
	info->thread_notes = 0;
	for (i = 0; i < view->n; ++i)
		if (view->regsets[i].core_note_type != 0)
			++info->thread_notes;

	/*
	 * Sanity check.  We rely on regset 0 being in NT_PRSTATUS,
	 * since it is our one special case.
	 */
	if (unlikely(info->thread_notes == 0) ||
	    unlikely(view->regsets[0].core_note_type != NT_PRSTATUS)) {
		WARN_ON(1);
		return 0;
	}

	/*
	 * Initialize the ELF file header.
	 */
	fill_elf_header(elf, phdrs,
			view->e_machine, view->e_flags);

	/*
	 * Allocate a structure for each thread.
	 */
	for (ct = &dump_task->mm->core_state->dumper; ct; ct = ct->next) {
		t = kzalloc(offsetof(struct elf_thread_core_info,
				     notes[info->thread_notes]),
			    GFP_KERNEL);
		if (unlikely(!t))
			return 0;

		t->task = ct->task;
		if (ct->task == dump_task || !info->thread) {
			t->next = info->thread;
			info->thread = t;
		} else {
			/*
			 * Make sure to keep the original task at
			 * the head of the list.
			 */
			t->next = info->thread->next;
			info->thread->next = t;
		}
	}

	/*
	 * Now fill in each thread's information.
	 */
	for (t = info->thread; t != NULL; t = t->next)
		if (!fill_thread_core_info(t, view, siginfo->si_signo, &info->size))
			return 0;

	/*
	 * Fill in the two process-wide notes.
	 */
	fill_psinfo(psinfo, dump_task->group_leader, dump_task->mm);
	info->size += notesize(&info->psinfo);

	fill_siginfo_note(&info->signote, &info->csigdata, siginfo);
	info->size += notesize(&info->signote);

	fill_auxv_note(&info->auxv, current->mm);
	info->size += notesize(&info->auxv);

	if (fill_files_note(&info->files) == 0)
		info->size += notesize(&info->files);

	return 1;
}

static size_t get_note_info_size(struct elf_note_info *info)
{
	return info->size;
}

/*
 * Write all the notes for each thread.  When writing the first thread, the
 * process-wide notes are interleaved after the first thread-specific note.
 */
static int write_note_info(struct elf_note_info *info,
			   struct coredump_params *cprm)
{
	bool first = true;
	struct elf_thread_core_info *t = info->thread;

	do {
		int i;

		if (!writenote(&t->notes[0], cprm))
			return 0;

		if (first && !writenote(&info->psinfo, cprm))
			return 0;
		if (first && !writenote(&info->signote, cprm))
			return 0;
		if (first && !writenote(&info->auxv, cprm))
			return 0;
		if (first && info->files.data &&
				!writenote(&info->files, cprm))
			return 0;

		for (i = 1; i < info->thread_notes; ++i)
			if (t->notes[i].data &&
			    !writenote(&t->notes[i], cprm))
				return 0;

		first = false;
		t = t->next;
	} while (t);

	return 1;
}

static void free_note_info(struct elf_note_info *info)
{
	struct elf_thread_core_info *threads = info->thread;
	while (threads) {
		unsigned int i;
		struct elf_thread_core_info *t = threads;
		threads = t->next;
		WARN_ON(t->notes[0].data && t->notes[0].data != &t->prstatus);
		for (i = 1; i < info->thread_notes; ++i)
			kfree(t->notes[i].data);
		kfree(t);
	}
	kfree(info->psinfo.data);
	vfree(info->files.data);
}

#else

/* Here is the structure in which status of each thread is captured. */
struct elf_thread_status
{
	struct list_head list;
	struct elf_prstatus prstatus;	/* NT_PRSTATUS */
	elf_fpregset_t fpu;		/* NT_PRFPREG */
	struct task_struct *thread;
#ifdef ELF_CORE_COPY_XFPREGS
	elf_fpxregset_t xfpu;		/* ELF_CORE_XFPREG_TYPE */
#endif
	struct memelfnote notes[3];
	int num_notes;
};

/*
 * In order to add the specific thread information for the elf file format,
 * we need to keep a linked list of every threads pr_status and then create
 * a single section for them in the final core file.
 */
static int elf_dump_thread_status(long signr, struct elf_thread_status *t)
{
	int sz = 0;
	struct task_struct *p = t->thread;
	t->num_notes = 0;

	fill_prstatus(&t->prstatus, p, signr);
	elf_core_copy_task_regs(p, &t->prstatus.pr_reg);	
	
	fill_note(&t->notes[0], "CORE", NT_PRSTATUS, sizeof(t->prstatus),
		  &(t->prstatus));
	t->num_notes++;
	sz += notesize(&t->notes[0]);

	if ((t->prstatus.pr_fpvalid = elf_core_copy_task_fpregs(p, NULL,
								&t->fpu))) {
		fill_note(&t->notes[1], "CORE", NT_PRFPREG, sizeof(t->fpu),
			  &(t->fpu));
		t->num_notes++;
		sz += notesize(&t->notes[1]);
	}

#ifdef ELF_CORE_COPY_XFPREGS
	if (elf_core_copy_task_xfpregs(p, &t->xfpu)) {
		fill_note(&t->notes[2], "LINUX", ELF_CORE_XFPREG_TYPE,
			  sizeof(t->xfpu), &t->xfpu);
		t->num_notes++;
		sz += notesize(&t->notes[2]);
	}
#endif	
	return sz;
}

struct elf_note_info {
	struct memelfnote *notes;
	struct memelfnote *notes_files;
	struct elf_prstatus *prstatus;	/* NT_PRSTATUS */
	struct elf_prpsinfo *psinfo;	/* NT_PRPSINFO */
	struct list_head thread_list;
	elf_fpregset_t *fpu;
#ifdef ELF_CORE_COPY_XFPREGS
	elf_fpxregset_t *xfpu;
#endif
	user_siginfo_t csigdata;
	int thread_status_size;
	int numnote;
};

static int elf_note_info_init(struct elf_note_info *info)
{
	memset(info, 0, sizeof(*info));
	INIT_LIST_HEAD(&info->thread_list);

	/* Allocate space for ELF notes */
	info->notes = kmalloc(8 * sizeof(struct memelfnote), GFP_KERNEL);
	if (!info->notes)
		return 0;
	info->psinfo = kmalloc(sizeof(*info->psinfo), GFP_KERNEL);
	if (!info->psinfo)
		return 0;
	info->prstatus = kmalloc(sizeof(*info->prstatus), GFP_KERNEL);
	if (!info->prstatus)
		return 0;
	info->fpu = kmalloc(sizeof(*info->fpu), GFP_KERNEL);
	if (!info->fpu)
		return 0;
#ifdef ELF_CORE_COPY_XFPREGS
	info->xfpu = kmalloc(sizeof(*info->xfpu), GFP_KERNEL);
	if (!info->xfpu)
		return 0;
#endif
	return 1;
}

static int fill_note_info(struct elfhdr *elf, int phdrs,
			  struct elf_note_info *info,
			  const siginfo_t *siginfo, struct pt_regs *regs)
{
	struct list_head *t;
	struct core_thread *ct;
	struct elf_thread_status *ets;

	if (!elf_note_info_init(info))
		return 0;

	for (ct = current->mm->core_state->dumper.next;
					ct; ct = ct->next) {
		ets = kzalloc(sizeof(*ets), GFP_KERNEL);
		if (!ets)
			return 0;

		ets->thread = ct->task;
		list_add(&ets->list, &info->thread_list);
	}

	list_for_each(t, &info->thread_list) {
		int sz;

		ets = list_entry(t, struct elf_thread_status, list);
		sz = elf_dump_thread_status(siginfo->si_signo, ets);
		info->thread_status_size += sz;
	}
	/* now collect the dump for the current */
	memset(info->prstatus, 0, sizeof(*info->prstatus));
	fill_prstatus(info->prstatus, current, siginfo->si_signo);
	elf_core_copy_regs(&info->prstatus->pr_reg, regs);

	/* Set up header */
	fill_elf_header(elf, phdrs, ELF_ARCH, ELF_CORE_EFLAGS);

	/*
	 * Set up the notes in similar form to SVR4 core dumps made
	 * with info from their /proc.
	 */

	fill_note(info->notes + 0, "CORE", NT_PRSTATUS,
		  sizeof(*info->prstatus), info->prstatus);
	fill_psinfo(info->psinfo, current->group_leader, current->mm);
	fill_note(info->notes + 1, "CORE", NT_PRPSINFO,
		  sizeof(*info->psinfo), info->psinfo);

	fill_siginfo_note(info->notes + 2, &info->csigdata, siginfo);
	fill_auxv_note(info->notes + 3, current->mm);
	info->numnote = 4;

	if (fill_files_note(info->notes + info->numnote) == 0) {
		info->notes_files = info->notes + info->numnote;
		info->numnote++;
	}

	/* Try to dump the FPU. */
	info->prstatus->pr_fpvalid = elf_core_copy_task_fpregs(current, regs,
							       info->fpu);
	if (info->prstatus->pr_fpvalid)
		fill_note(info->notes + info->numnote++,
			  "CORE", NT_PRFPREG, sizeof(*info->fpu), info->fpu);
#ifdef ELF_CORE_COPY_XFPREGS
	if (elf_core_copy_task_xfpregs(current, info->xfpu))
		fill_note(info->notes + info->numnote++,
			  "LINUX", ELF_CORE_XFPREG_TYPE,
			  sizeof(*info->xfpu), info->xfpu);
#endif

	return 1;
}

static size_t get_note_info_size(struct elf_note_info *info)
{
	int sz = 0;
	int i;

	for (i = 0; i < info->numnote; i++)
		sz += notesize(info->notes + i);

	sz += info->thread_status_size;

	return sz;
}

static int write_note_info(struct elf_note_info *info,
			   struct coredump_params *cprm)
{
	int i;
	struct list_head *t;

	for (i = 0; i < info->numnote; i++)
		if (!writenote(info->notes + i, cprm))
			return 0;

	/* write out the thread status notes section */
	list_for_each(t, &info->thread_list) {
		struct elf_thread_status *tmp =
				list_entry(t, struct elf_thread_status, list);

		for (i = 0; i < tmp->num_notes; i++)
			if (!writenote(&tmp->notes[i], cprm))
				return 0;
	}

	return 1;
}

static void free_note_info(struct elf_note_info *info)
{
	while (!list_empty(&info->thread_list)) {
		struct list_head *tmp = info->thread_list.next;
		list_del(tmp);
		kfree(list_entry(tmp, struct elf_thread_status, list));
	}

	/* Free data possibly allocated by fill_files_note(): */
	if (info->notes_files)
		vfree(info->notes_files->data);

	kfree(info->prstatus);
	kfree(info->psinfo);
	kfree(info->notes);
	kfree(info->fpu);
#ifdef ELF_CORE_COPY_XFPREGS
	kfree(info->xfpu);
#endif
}

#endif

static struct vm_area_struct *first_vma(struct task_struct *tsk,
					struct vm_area_struct *gate_vma)
{
	struct vm_area_struct *ret = tsk->mm->mmap;

	if (ret)
		return ret;
	return gate_vma;
}
/*
 * Helper function for iterating across a vma list.  It ensures that the caller
 * will visit `gate_vma' prior to terminating the search.
 */
static struct vm_area_struct *next_vma(struct vm_area_struct *this_vma,
					struct vm_area_struct *gate_vma)
{
	struct vm_area_struct *ret;

	ret = this_vma->vm_next;
	if (ret)
		return ret;
	if (this_vma == gate_vma)
		return NULL;
	return gate_vma;
}

static void fill_extnum_info(struct elfhdr *elf, struct elf_shdr *shdr4extnum,
			     elf_addr_t e_shoff, int segs)
{
	elf->e_shoff = e_shoff;
	elf->e_shentsize = sizeof(*shdr4extnum);
	elf->e_shnum = 1;
	elf->e_shstrndx = SHN_UNDEF;

	memset(shdr4extnum, 0, sizeof(*shdr4extnum));

	shdr4extnum->sh_type = SHT_NULL;
	shdr4extnum->sh_size = elf->e_shnum;
	shdr4extnum->sh_link = elf->e_shstrndx;
	shdr4extnum->sh_info = segs;
}

static size_t elf_core_vma_data_size(struct vm_area_struct *gate_vma,
				     struct coredump_params *cprm)
{
	struct vm_area_struct *vma;
	size_t size = 0;

	for (vma = first_vma(current, gate_vma); vma != NULL;
	     vma = next_vma(vma, gate_vma))
		size += vma_dump_size(vma, cprm->mm_flags, cprm->siginfo->si_signo);
	return size;
}

/*
 * Actual dumper
 *
 * This is a two-pass process; first we find the offsets of the bits,
 * and then they are actually written out.  If we run out of core limit
 * we just truncate.
 */
static int elf_core_dump(struct coredump_params *cprm)
{
	int has_dumped = 0;
	mm_segment_t fs;
	int segs;
	struct vm_area_struct *vma, *gate_vma;
	struct elfhdr *elf = NULL;
	loff_t offset = 0, dataoff;
	struct elf_note_info info = { };
	struct elf_phdr *phdr4note = NULL;
	struct elf_shdr *shdr4extnum = NULL;
	Elf_Half e_phnum;
	elf_addr_t e_shoff;

	/*
	 * We no longer stop all VM operations.
	 * 
	 * This is because those proceses that could possibly change map_count
	 * or the mmap / vma pages are now blocked in do_exit on current
	 * finishing this core dump.
	 *
	 * Only ptrace can touch these memory addresses, but it doesn't change
	 * the map_count or the pages allocated. So no possibility of crashing
	 * exists while dumping the mm->vm_next areas to the core file.
	 */
  
	/* alloc memory for large data structures: too large to be on stack */
	elf = kmalloc(sizeof(*elf), GFP_KERNEL);
	if (!elf)
		goto out;
	/*
	 * The number of segs are recored into ELF header as 16bit value.
	 * Please check DEFAULT_MAX_MAP_COUNT definition when you modify here.
	 */
	segs = current->mm->map_count;
	segs += elf_core_extra_phdrs();

	gate_vma = get_gate_vma(current->mm);
	if (gate_vma != NULL)
		segs++;

	/* for notes section */
	segs++;

	/* If segs > PN_XNUM(0xffff), then e_phnum overflows. To avoid
	 * this, kernel supports extended numbering. Have a look at
	 * include/linux/elf.h for further information. */
	e_phnum = segs > PN_XNUM ? PN_XNUM : segs;

	/*
	 * Collect all the non-memory information about the process for the
	 * notes.  This also sets up the file header.
	 */
	if (!fill_note_info(elf, e_phnum, &info, cprm->siginfo, cprm->regs))
		goto cleanup;

	has_dumped = 1;

	fs = get_fs();
	set_fs(KERNEL_DS);

	offset += sizeof(*elf);				/* Elf header */
	offset += segs * sizeof(struct elf_phdr);	/* Program headers */

	/* Write notes phdr entry */
	{
		size_t sz = get_note_info_size(&info);

		sz += elf_coredump_extra_notes_size();

		phdr4note = kmalloc(sizeof(*phdr4note), GFP_KERNEL);
		if (!phdr4note)
			goto end_coredump;

		fill_elf_note_phdr(phdr4note, sz, offset);
		offset += sz;
	}

	dataoff = offset = roundup(offset, ELF_EXEC_PAGESIZE);

	offset += elf_core_vma_data_size(gate_vma, cprm);
	offset += elf_core_extra_data_size();
	e_shoff = offset;

	if (e_phnum == PN_XNUM) {
		shdr4extnum = kmalloc(sizeof(*shdr4extnum), GFP_KERNEL);
		if (!shdr4extnum)
			goto end_coredump;
		fill_extnum_info(elf, shdr4extnum, e_shoff, segs);
	}

	offset = dataoff;

	if (!dump_emit(cprm, elf, sizeof(*elf)))
		goto end_coredump;

	if (!dump_emit(cprm, phdr4note, sizeof(*phdr4note)))
		goto end_coredump;

	/* Write program headers for segments dump */
	for (vma = first_vma(current, gate_vma); vma != NULL;
			vma = next_vma(vma, gate_vma)) {
		struct elf_phdr phdr;

		phdr.p_type = PT_LOAD;
		phdr.p_offset = offset;
		phdr.p_vaddr = vma->vm_start;
		phdr.p_paddr = 0;
		phdr.p_filesz = vma_dump_size(vma, cprm->mm_flags, cprm->siginfo->si_signo);
		phdr.p_memsz = vma->vm_end - vma->vm_start;
		offset += phdr.p_filesz;
		phdr.p_flags = vma->vm_flags & VM_READ ? PF_R : 0;
		if (vma->vm_flags & VM_WRITE)
			phdr.p_flags |= PF_W;
		if (vma->vm_flags & VM_EXEC)
			phdr.p_flags |= PF_X;
		phdr.p_align = ELF_EXEC_PAGESIZE;

		if (!dump_emit(cprm, &phdr, sizeof(phdr)))
			goto end_coredump;
	}

	if (!elf_core_write_extra_phdrs(cprm, offset))
		goto end_coredump;

 	/* write out the notes section */
	if (!write_note_info(&info, cprm))
		goto end_coredump;

	if (elf_coredump_extra_notes_write(cprm))
		goto end_coredump;

	/* Align to page */
	if (!dump_skip(cprm, dataoff - cprm->written))
		goto end_coredump;

	for (vma = first_vma(current, gate_vma); vma != NULL;
			vma = next_vma(vma, gate_vma)) {
		unsigned long addr;
		unsigned long end;

		end = vma->vm_start + vma_dump_size(vma, cprm->mm_flags, cprm->siginfo->si_signo);

		for (addr = vma->vm_start; addr < end; addr += PAGE_SIZE) {
			struct page *page;
			int stop;

			page = get_dump_page(addr);
			if (page) {
				void *kaddr = kmap(page);
				stop = !dump_emit(cprm, kaddr, PAGE_SIZE);
				kunmap(page);
				page_cache_release(page);
			} else
				stop = !dump_skip(cprm, PAGE_SIZE);
			if (stop)
				goto end_coredump;
		}
	}

	if (!elf_core_write_extra_data(cprm))
		goto end_coredump;

	if (e_phnum == PN_XNUM) {
		if (!dump_emit(cprm, shdr4extnum, sizeof(*shdr4extnum)))
			goto end_coredump;
	}

end_coredump:
	set_fs(fs);

cleanup:
	free_note_info(&info);
	kfree(shdr4extnum);
	kfree(phdr4note);
	kfree(elf);
out:
	return has_dumped;
}

#endif		/* CONFIG_ELF_CORE */

#ifdef CONFIG_PAX_MPROTECT
/* PaX: non-PIC ELF libraries need relocations on their executable segments
 * therefore we'll grant them VM_MAYWRITE once during their life. Similarly
 * we'll remove VM_MAYWRITE for good on RELRO segments.
 *
 * The checks favour ld-linux.so behaviour which operates on a per ELF segment
 * basis because we want to allow the common case and not the special ones.
 */
static void elf_handle_mprotect(struct vm_area_struct *vma, unsigned long newflags)
{
	struct elfhdr elf_h;
	struct elf_phdr elf_p;
	unsigned long i;
	unsigned long oldflags;
	bool is_textrel_rw, is_textrel_rx, is_relro;

	if (!(vma->vm_mm->pax_flags & MF_PAX_MPROTECT) || !vma->vm_file)
		return;

	oldflags = vma->vm_flags & (VM_MAYEXEC | VM_MAYWRITE | VM_MAYREAD | VM_EXEC | VM_WRITE | VM_READ);
	newflags &= VM_MAYEXEC | VM_MAYWRITE | VM_MAYREAD | VM_EXEC | VM_WRITE | VM_READ;

#ifdef CONFIG_PAX_ELFRELOCS
	/* possible TEXTREL */
	is_textrel_rw = !vma->anon_vma && oldflags == (VM_MAYEXEC | VM_MAYREAD | VM_EXEC | VM_READ) && newflags == (VM_WRITE | VM_READ);
	is_textrel_rx = vma->anon_vma && oldflags == (VM_MAYEXEC | VM_MAYWRITE | VM_MAYREAD | VM_WRITE | VM_READ) && newflags == (VM_EXEC | VM_READ);
#else
	is_textrel_rw = false;
	is_textrel_rx = false;
#endif

	/* possible RELRO */
	is_relro = vma->anon_vma && oldflags == (VM_MAYWRITE | VM_MAYREAD | VM_READ) && newflags == (VM_MAYWRITE | VM_MAYREAD | VM_READ);

	if (!is_textrel_rw && !is_textrel_rx && !is_relro)
		return;

	if (sizeof(elf_h) != kernel_read(vma->vm_file, 0UL, (char *)&elf_h, sizeof(elf_h)) ||
	    memcmp(elf_h.e_ident, ELFMAG, SELFMAG) ||

#ifdef CONFIG_PAX_ETEXECRELOCS
	    ((is_textrel_rw || is_textrel_rx) && (elf_h.e_type != ET_DYN && elf_h.e_type != ET_EXEC)) ||
#else
	    ((is_textrel_rw || is_textrel_rx) && elf_h.e_type != ET_DYN) ||
#endif

	    (is_relro && (elf_h.e_type != ET_DYN && elf_h.e_type != ET_EXEC)) ||
	    !elf_check_arch(&elf_h) ||
	    elf_h.e_phentsize != sizeof(struct elf_phdr) ||
	    elf_h.e_phnum > 65536UL / sizeof(struct elf_phdr))
		return;

	for (i = 0UL; i < elf_h.e_phnum; i++) {
		if (sizeof(elf_p) != kernel_read(vma->vm_file, elf_h.e_phoff + i*sizeof(elf_p), (char *)&elf_p, sizeof(elf_p)))
			return;
		switch (elf_p.p_type) {
		case PT_DYNAMIC:
			if (!is_textrel_rw && !is_textrel_rx)
				continue;
			i = 0UL;
			while ((i+1) * sizeof(elf_dyn) <= elf_p.p_filesz) {
				elf_dyn dyn;

				if (sizeof(dyn) != kernel_read(vma->vm_file, elf_p.p_offset + i*sizeof(dyn), (char *)&dyn, sizeof(dyn)))
					break;
				if (dyn.d_tag == DT_NULL)
					break;
				if (dyn.d_tag == DT_TEXTREL || (dyn.d_tag == DT_FLAGS && (dyn.d_un.d_val & DF_TEXTREL))) {
					gr_log_textrel(vma);
					if (is_textrel_rw)
						vma->vm_flags |= VM_MAYWRITE;
					else
						/* PaX: disallow write access after relocs are done, hopefully noone else needs it... */
						vma->vm_flags &= ~VM_MAYWRITE;
					break;
				}
				i++;
			}
			is_textrel_rw = false;
			is_textrel_rx = false;
			continue;

		case PT_GNU_RELRO:
			if (!is_relro)
				continue;
			if ((elf_p.p_offset >> PAGE_SHIFT) == vma->vm_pgoff && ELF_PAGEALIGN(elf_p.p_memsz) == vma->vm_end - vma->vm_start)
				vma->vm_flags &= ~VM_MAYWRITE;
			is_relro = false;
			continue;

#ifdef CONFIG_PAX_PT_PAX_FLAGS
		case PT_PAX_FLAGS: {
			const char *msg_mprotect = "", *msg_emutramp = "";
			char *buffer_lib, *buffer_exe;

			if (elf_p.p_flags & PF_NOMPROTECT)
				msg_mprotect = "MPROTECT disabled";

#ifdef CONFIG_PAX_EMUTRAMP
			if (!(vma->vm_mm->pax_flags & MF_PAX_EMUTRAMP) && !(elf_p.p_flags & PF_NOEMUTRAMP))
				msg_emutramp = "EMUTRAMP enabled";
#endif

			if (!msg_mprotect[0] && !msg_emutramp[0])
				continue;

			if (!printk_ratelimit())
				continue;

			buffer_lib = (char *)__get_free_page(GFP_KERNEL);
			buffer_exe = (char *)__get_free_page(GFP_KERNEL);
			if (buffer_lib && buffer_exe) {
				char *path_lib, *path_exe;

				path_lib = pax_get_path(&vma->vm_file->f_path, buffer_lib, PAGE_SIZE);
				path_exe = pax_get_path(&vma->vm_mm->exe_file->f_path, buffer_exe, PAGE_SIZE);

				pr_info("PAX: %s wants %s%s%s on %s\n", path_lib, msg_mprotect,
					(msg_mprotect[0] && msg_emutramp[0] ? " and " : ""), msg_emutramp, path_exe);

			}
			free_page((unsigned long)buffer_exe);
			free_page((unsigned long)buffer_lib);
			continue;
		}
#endif

		}
	}
}
#endif

#ifdef CONFIG_GRKERNSEC_RWXMAP_LOG

extern int grsec_enable_log_rwxmaps;

static void elf_handle_mmap(struct file *file)
{
	struct elfhdr elf_h;
	struct elf_phdr elf_p;
	unsigned long i;

	if (!grsec_enable_log_rwxmaps)
		return;

	if (sizeof(elf_h) != kernel_read(file, 0UL, (char *)&elf_h, sizeof(elf_h)) ||
	    memcmp(elf_h.e_ident, ELFMAG, SELFMAG) ||
	    (elf_h.e_type != ET_DYN && elf_h.e_type != ET_EXEC) || !elf_check_arch(&elf_h) ||
	    elf_h.e_phentsize != sizeof(struct elf_phdr) ||
	    elf_h.e_phnum > 65536UL / sizeof(struct elf_phdr))
		return;

	for (i = 0UL; i < elf_h.e_phnum; i++) {
		if (sizeof(elf_p) != kernel_read(file, elf_h.e_phoff + i*sizeof(elf_p), (char *)&elf_p, sizeof(elf_p)))
			return;
		if (elf_p.p_type == PT_GNU_STACK && (elf_p.p_flags & PF_X))
			gr_log_ptgnustack(file);
	}
}
#endif

static int __init init_elf_binfmt(void)
{
	register_binfmt(&elf_format);
	return 0;
}

static void __exit exit_elf_binfmt(void)
{
	/* Remove the COFF and ELF loaders. */
	unregister_binfmt(&elf_format);
}

core_initcall(init_elf_binfmt);
module_exit(exit_elf_binfmt);
MODULE_LICENSE("GPL");
