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

/* Kernel Includes */
#include <linux/kernel.h>
#include <linux/mm_types.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/mount.h>
#include <linux/version.h>
#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <asm/pgalloc.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"

/*************************************************************************/

struct task_queue_node {
	struct task_struct *task;
	struct list_head queue;
};

struct vma_section {
	char *path;			/* Section binary path */
	uint8_t hash[HASH_SHA256_SIZE];	/* Hash of section */
};

struct _vma_globals {
	struct rb_root sections;
	struct mutex lock;		/* Lock for context */
};

static struct _vma_globals ctx = {};

/*************************************************************************/

#define VM_READEXEC		(VM_READ | VM_EXEC)
#define MAX_PATH		256
#define MAX_FAULT_ATTEMPT	3
#define BAD_HASH_MESSAGE_LEN	200

/*************************************************************************/

/*
 * This function composes a list of all running processes. Each process has
 * their reference count increased. The processes returned must have their
 * reference count decreased after this call.
 *
 * @param   queue           A pointer to a list.
 *
 * @return  0               No Error.
 *          -EINVAL         Invalid parameters.
 *          -ENOMEM         Memory allocation error.
 */
static int vma_get_process_list(struct list_head *queue)
{
	struct task_struct *p = NULL;
	int rc = 0;

	if (!queue)
		return -EINVAL;

	read_lock(&tasklist_lock);

	for_each_process(p) {
		struct task_queue_node *node = NULL;
		int root = 0;

		/* Ignore kernel pid */
		if (unlikely(!p->pid))
			continue;

		/* Ignore non-root processes */
		root = util_is_task_root(p);
		if (root <= 0)
			continue;

		/* Allocate a node for the list of tasks */
		node = kzalloc(sizeof(struct task_queue_node), GFP_ATOMIC);
		if (!node) {
			logError("Failed to allocate report queue node.");

			rc = -ENOMEM;
			break;
		}

		/* Increase the reference count on the task */
		get_task_struct(p);
		node->task = p;

		/* Add the node to the list */
		INIT_LIST_HEAD(&node->queue);
		list_add_tail(&node->queue, queue);
	}

	read_unlock(&tasklist_lock);

	return rc;
}


/*************************************************************************/

/*
 * This function resolves any pages that have not been loaded from disk.
 * The mmap_sem semaphore should be held prior to calling this function.
 *
 * @param   task            The process itself.
 * @param   addr            A pointer within a page.
 *
 * @return  0               No Error.
 *          -EINVAL         Bad input parameters.
 *          -ENODATA        Task does not have mm or vma pointers.
 *          -EFAULT         Memory fault was not resolved.
 */
static int vma_handle_fault(struct task_struct *task,
			    struct vm_area_struct *vma,
			    unsigned long addr)
{
	struct mm_struct *mm = NULL;
	int flags = FAULT_FLAG_ALLOW_RETRY | FAULT_FLAG_RETRY_NOWAIT;
	int rc = 0;

	if (!task || !vma)
		return -EINVAL;

	mm = task->mm;
	if (!mm)
		return -ENODATA;

	do {
		/* Resolve any page faults prior to page access */
		rc = handle_mm_fault(mm, vma, addr, flags);
		if (rc & VM_FAULT_ERROR) {
			logError("Failed on handle_mm_fault(). rc=%X.", rc);
			return -EFAULT;
		}

		if (rc & VM_FAULT_MAJOR)
			task->maj_flt++;
		else
			task->min_flt++;

		if (rc & VM_FAULT_RETRY) {
			cond_resched();

			/* Retry only once */
			flags &= ~FAULT_FLAG_ALLOW_RETRY;
		}
	} while (rc & VM_FAULT_RETRY);

	return 0;
}

/*************************************************************************/

/*
 * This function walks the page table structures to find the page table
 * entry that contains the address specified. On success, the function
 * increases the reference count on the page and returns a pointer to it.
 *
 * @param   mm              The task's memory region.
 * @param   addr            A pointer to a userspace memory chunk.
 * @param   out             The resulting page.
 *
 * @return  0               No Error.
 *          -EFAULT         Memory fault encountered.
 *          -EINVAL         Invalid parameters.
 *          -ENOMEM         Unable to obtain page.
 */
static int vma_find_page(struct mm_struct *mm,
			 unsigned long addr,
			 struct page **out)
{
	struct page *pg = NULL;
	pgd_t *pgd = NULL;
	pud_t *pud = NULL;
	pmd_t *pmd = NULL;
	pte_t *pte = NULL;

	if (!mm || !addr || !out)
		return -EINVAL;

	/* Walk the global page directory */
	pgd = pgd_offset(mm, addr);
	if (pgd_none(*pgd) || pgd_bad(*pgd))
		return -EFAULT;

	pud = pud_offset(pgd, addr);
	if (pud_none(*pud) || pud_bad(*pud))
		return -EFAULT;

	pmd = pmd_offset(pud, addr);
	if (pmd_none(*pmd) || pmd_bad(*pmd))
		return -EFAULT;

	pte = pte_offset_map(pmd, addr);
	if (!pte || !pte_val(*pte) || !pte_present(*pte))
		return -EFAULT;

	/* Once the PTE was resolved, obtain the Page and return */
	pg = pte_page(*pte);
	if (pg) {
		get_page(pg);
		*out = pg;
	}

	pte_unmap(pte);

	return (pg) ? 0 : -ENOMEM;
}

/*************************************************************************/

/*
 * This function creates a hash of a chunk of memory that resides in
 * user-space. The memory chunk is mapped to a kernel address prior to
 * being hashed.
 *
 * @param   vma             Virtual memory struct of the user process.
 * @param   addr            A pointer to a userspace memory chunk.
 * @param   sz              Size of the memory chunk.
 * @param   hash            A pointer to the hash output buffer.
 * @param   hash_sz         Size of hash output buffer.
 *
 * @return  0               No Error.
 *          -ENOMEM         Memory allocation error.
 *          -EINVAL         Invalid parameters.
 */
static int vma_hash_userspace(struct task_struct *task,
			      struct vm_area_struct *vma,
			      uint8_t *hash,
			      unsigned hash_sz)
{
	struct crypto_context ctx = {};
	struct mm_struct *mm = NULL;
	unsigned long i = 0;
	int last_rc = 0;
	int rc = 0;

	if (!task || !vma || !hash)
		return -EINVAL;

	mm = task->mm;
	if (!mm) {
		logError("Task does not contain valid mm.");
		return -ENODATA;
	}

	/* Initialize sha256 */
	rc = crypto_begin(HASH_ALG_SHA256, &ctx);
	if (rc) {
		logError("Failed on crypto_begin(). rc=%d.", -rc);
		return rc;
	}

	spin_lock(&mm->page_table_lock);

	/* Loop through all pages, hashing each one */
	for (i = vma->vm_start;
	     i < vma->vm_end && !rc;
	     i += PAGE_SIZE) {
		unsigned long page_addr = i & PAGE_MASK;
		struct page *page = NULL;

		/* Find the page the pointer points to */
		rc = vma_find_page(mm, page_addr, &page);
		if (rc == -EFAULT) {
			/* Page fault encountered, resolve it */
			spin_unlock(&mm->page_table_lock);
			rc = vma_handle_fault(task, vma, i);
			spin_lock(&mm->page_table_lock);

			if (rc) {
				logError("Failed on vma_handle_fault(). rc=%d", -rc);
				break;
			}

			/* Retry page look up after fault resolution */
			rc = vma_find_page(mm, page_addr, &page);
		}

		if (rc) {
			logError("Failed on vma_find_page(). rc=%d.", -rc);
			break;
		}

		if (page) {
			unsigned offset = (unsigned) (i & (PAGE_SIZE - 1));
			unsigned ksz = min_t(unsigned, PAGE_SIZE - offset, vma->vm_end - i);

			/* Hash the page's contents */
			rc = crypto_update_page(&ctx, page, ksz, offset);
			if (rc)
				logError("Failed on crypto_update(). rc=%d.", -rc);

			/* Decrement page reference count */
			put_page(page);
		} else
			logError("Could not resolve page.");

		/* Realign to page boundaries */
		i = page_addr;
	}

	spin_unlock(&mm->page_table_lock);
	last_rc = rc;

	/* Finalize the hashing and generate a hash */
	rc = crypto_end(&ctx, hash, hash_sz);
	if (rc) {
		logError("Failed on crypto_end(). rc=%d.", -rc);
		return rc;
	}

	return last_rc ? last_rc : rc;
}

/*************************************************************************/

/*
 * A function to verify and add read-only and executable memory sections
 * of a given process.
 *
 * @param   path            The path where the section was loaded from.
 * @param   path_sz         Size path.
 * @param   hash            The hash of the memory section.
 * @param   hash_sz         Size fo the hash (Always SHA256_DIGEST_SIZE).
 * @param   flags           Flags.
 *
 * @return  0               No Error.
 *          -ENOMEM         Memory allocation error.
 *          -EINVAL         Invalid parameters.
 *          -EFAULT         Hash of section does not match.
 */
static int vma_add_verify_section(const char *path,
				  unsigned path_sz,
				  const uint8_t *hash,
				  unsigned hash_sz,
				  int flags)
{
	struct vma_section *sec = NULL;
	int rc = 0;

	if (!path || !hash || hash_sz != HASH_SHA256_SIZE)
		return -EINVAL;

	/* Do a lookup to see if the hash already exists */
	rc = hash_search(&ctx.sections, path, path_sz, (void **) &sec);
	if (!rc) {
		/* Verify that the hash is valid */
		rc = memcmp(sec->hash, hash, HASH_SHA256_SIZE);
		if (!rc)
			return 0;

		logError("Hashes for [%s] do not Match!", path);
		return -EFAULT;

	} else if (rc == -ENOENT) {
		/* Check if new sections are allowed */
		if (flags & VMA_FLAG_DISALLOW_NEW_SECTIONS)
			return rc;

		sec = kzalloc(sizeof(struct vma_section), GFP_KERNEL);
		if (!sec) {
			logError("Failed to allocate vma_section.");
			return -ENOMEM;
		}

		sec->path = kzalloc(path_sz + 1, GFP_KERNEL);
		if (!sec->path) {
			logError("Failed to allocate vma_section->path.");

			kfree(sec);
			return -ENOMEM;
		}

		memcpy(sec->path, path, path_sz);
		memcpy(sec->hash, hash, HASH_SHA256_SIZE);

		/* Add the section to the hash map */
		rc = hash_insert(&ctx.sections, path, path_sz, sec);
		if (rc) {
			logError("Failed on hash_insert(). rc=%d", -rc);

			kfree(sec->path);
			kfree(sec);
		}
	} else
		logError("Failed on hash_search(). rc=%d.", -rc);

	return rc;
}

/*************************************************************************/

/*
 * This function creates hashes for each read only and executable page of
 * the provided task. The hashes are checked against known values, if they
 * exist, and a report is generated if the hashes do not match.
 *
 * @param   task            The task data.
 * @param   flags           Operational flags.
 *
 * @return  0               No Error.
 *          -EINVAL         Bad input parameters.
 *          -ENODATA        Task is a kernel task and does not have mmap.
 */
int vma_scan_task(struct task_struct *task,
		  int flags)
{
	struct mm_struct *mm = NULL;
	struct vm_area_struct *vma = NULL;
	char name[MAX_PATH] = { 0 };
	unsigned nsz = 0;
	int rc = 0;

	if (!task)
		return -EINVAL;

	/* Increment reference on task */
	get_task_struct(task);

	/* Kernel tasks don't have an mm, ignore them */
	mm = get_task_mm(task);
	if (!mm) {
		rc = -ENODATA;
		goto cleanup;
	}

	/* Get the name of the task; used as the key in hash table */
	nsz = util_get_task_path(task->mm, name, sizeof(name));
	if (nsz < 0) {
		logError("Failed on util_get_task_path(). rc=%d.", -nsz);

		rc = (int) nsz;
		goto cleanup;
	}

	logDebug("Hashing Process: %s.", name);

	/* Write lock is necessary for page table fault handling */
	down_write(&mm->mmap_sem);

	/* Loop through all VMAs, hashing each one */
	for (vma = mm->mmap; vma; vma = vma->vm_next) {
		uint8_t hash[HASH_SHA256_SIZE] = { 0 };
		char tmp[MAX_PATH + 1] = { 0 };
		char *p = NULL;

		/* Don't hash anonymous sections */
		if (!vma->vm_file)
			continue;

		/* Limit the hashing to executable code */
		if ((vma->vm_flags & VM_READEXEC) != VM_READEXEC)
			continue;

		/* Extract the path name of the binary in this section */
		p = d_path(&vma->vm_file->f_path, tmp, MAX_PATH);
		if (IS_ERR(p)) {
			rc = (int) -PTR_ERR(p);

			logError("Failed on d_path(). rc=%d", -rc);
			break;
		}

		/* Hash the process' page */
		rc = vma_hash_userspace(task, vma, hash, sizeof(hash));
		if (rc) {
			logError("Failed on vma_hash_userspace(). rc=%d.", -rc);
			break;
		}

		/* Add or verify the hash against preexisting data */
		rc = vma_add_verify_section(p,
					    strnlen(p, MAX_PATH - (p - tmp)),
					    hash,
					    sizeof(hash),
					    flags);
		if (rc == -EFAULT) {
			char msg[BAD_HASH_MESSAGE_LEN] = { 0 };

			/* Compile XML with offending process name and section */
			rc = xml_vma_msg(msg, sizeof(msg), name, p);
			if (rc < 0) {
				logError("Failed on xml_vma_msg(). rc=%d.", -rc);
				break;
			}

			/* Send an incident report to JBIDE */
			rc = report_incident(SEVERITY_CRITICAL,
					     SN_MISMATCHED_PROC_HASH,
					     msg,
					     task);
			if (rc)
				logError("Failed on report_incident() from vma_scan_task(). rc=%d.", -rc);

			break;
		} else if (rc) {
			logError("Failed on vma_add_verify_section(). rc=%d.", -rc);
			break;
		}
	}

	up_write(&mm->mmap_sem);

cleanup:

	if (mm)
		mmput(mm);

	/* Dereference task */
	put_task_struct(task);

	return rc;
}

/*************************************************************************/

/*
 * This function loops through all running processes and creates a hash
 * of their VMAs. Any values that do not match up will cause an error.
 *
 * @return  0               No Error.
 *          -EINVAL         Bad input parameters.
 *          -ENODATA        Task is a kernel task and does not have mmap.
 */
int vma_scan_processes(void)
{
	struct list_head queue = { 0 };
	int rc = 0;

	logDebug("Starting process scan.");

	INIT_LIST_HEAD(&queue);

	rc = vma_get_process_list(&queue);
	if (rc)
		logError("Failed on vma_get_process_list(). rc=%d.", -rc);

	/* Process each task and deallocate */
	while (!list_empty(&queue)) {
		struct task_queue_node *node = NULL;
		int vm_rc = 0;

		node = list_first_entry(&queue, struct task_queue_node, queue);

		BUG_ON(!node);

		vm_rc = vma_scan_task(node->task, 0);
		if (vm_rc && vm_rc != -ENODATA) {
			logError("Failed on vma_hash_task(). rc=%d", -vm_rc);
			rc = vm_rc;
		}

		list_del(&node->queue);

		/* Decrement reference count on task */
		put_task_struct(node->task);
		kfree(node);
	}

	logDebug("Done scanning processes. rc=%d.", -rc);

	return rc;
}

/*************************************************************************/

/*
 * Entry point for initializaiton.
 *
 * @return  0               No Error.
 */
int __init vma_init(void)
{
	mutex_init(&ctx.lock);

	return 0;
}

/*************************************************************************/

/*
 * Context clean up.
 *
 * @return  0               No Error.
 */
int __exit vma_exit(void)
{
	mutex_destroy(&ctx.lock);

	return 0;
}
