/*
 * Copyright (C) 2018 BlackBerry Limited
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
	uint8_t hash[SHA256_SIZE];	/* Hash of section */
	uint8_t last_page_hash[SHA256_SIZE];	/* Hash of section */
	uintptr_t sect_addr; /* The address of the section */
	unsigned long sect_sz; /* The size of the section */
	unsigned long last_page_sz; /* The size of the last section */
	unsigned page_fault_status; /* Information about page faults when scanning*/
	unsigned long vm_flags; /* The vm flags set on the section when it was scanned */
};

struct _vma_globals {
	struct rb_root sections;
	struct mutex lock;		/* Lock for context */
};

static struct _vma_globals ctx = {};

/*************************************************************************/

#define VM_READEXEC		(VM_READ | VM_EXEC)
#define BAD_HASH_MESSAGE_LEN	200
#define PAGE_FAULT_LAST_SECTION 1
#define PAGE_FAULT_ANY_SECTION  2

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
 * @param   vma             Virtual memory struct of the user process.
 * @param   addr            A pointer within a page.
 *
 * @return  0               No Error.
 *          -EINVAL         Bad input parameters.
 *          -ENODATA        Task does not have mm or vma pointers.
 *          -EFAULT         Memory fault was not resolved.
 */
static int vma_handle_fault(struct task_struct *task,
			    struct vm_area_struct *vma,
			    uintptr_t addr)
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
			 uintptr_t addr,
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
 * @param   task            The task data.
 * @param   vma             Virtual memory struct of the user process.
 * @param   hash            A pointer to the hash output buffer.
 * @param   hash_sz         Size of hash output buffer.
 * @param   status          Information if a page fault happened on the
 *                          sections of this call.
 *
 * @return  0               No Error.
 *          -ENOMEM         Memory allocation error.
 *          -EINVAL         Invalid parameters.
 *          -ENODATA        Invalid task passed in.
 */
static int vma_hash_userspace(struct task_struct *task,
			      struct vm_area_struct *vma,
			      uint8_t *hash,
			      unsigned hash_sz,
			      uint8_t *last_page_hash,
			      unsigned long *last_page_sz,
			      unsigned last_page_hash_sz,
			      unsigned *status)
{
	struct crypto_context ctx = {};
	struct crypto_context last_ctx = {};
	struct mm_struct *mm = NULL;
	uintptr_t i = 0;
	int last_rc = 0;
	int rc = 0;

	if (!task || !vma || !hash || !status || !last_page_hash
		|| !last_page_sz)
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

	rc = crypto_begin(HASH_ALG_SHA256, &last_ctx);
	if (rc) {
		logError("Failed on crypto_begin(last). rc=%d.", -rc);
		return rc;
	}

	spin_lock(&mm->page_table_lock);

	/* Loop through all pages, hashing each one */
	for (i = vma->vm_start;
	     i < vma->vm_end && !rc;
	     i += PAGE_SIZE) {
		uintptr_t page_addr = i & PAGE_MASK;
		struct page *page = NULL;

		/* Find the page the pointer points to */
		rc = vma_find_page(mm, page_addr, &page);
		if (rc == -EFAULT) {
			if (i + PAGE_SIZE >= vma->vm_end)
				*status |= PAGE_FAULT_LAST_SECTION;
			else
				*status |= PAGE_FAULT_ANY_SECTION;

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

			/* hash the last page to check if this is what causes the
			   hash missmatch error */
			if (i + PAGE_SIZE >= vma->vm_end) {
				*last_page_sz = ksz;
				rc = crypto_update_page(&last_ctx, page, ksz, offset);
				if (rc)
					logError("Failed on crypto_update(last). rc=%d.", -rc);
			}

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

	rc = crypto_end(&last_ctx, last_page_hash, last_page_hash_sz);
	if (rc) {
		logError("Failed on crypto_end(last). rc=%d.", -rc);
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
 * @param   sect_addr       Address of the section being hashed.
 * @param   sect_sz         Size of the section being hashed.
 * @param   vm_flags        The flags set on the virtual memory address.
 * @param   flags           Flags.
 *
 * @return  0               No Error.
 *          -ENOMEM         Memory allocation error.
 *          -EINVAL         Invalid parameters.
 *          -EFAULT         Hash of section does not match.
 *          -ERESTARTSYS    The call was interrupted by a signal
 */
static int vma_add_verify_section(const char *path,
				  unsigned path_sz,
				  unsigned long vm_pgoff,
				  const uint8_t *hash,
				  unsigned hash_sz,
				  const uint8_t *last_page_hash,
				  unsigned last_page_hash_sz,
				  uintptr_t sect_addr,
				  unsigned long sect_sz,
				  unsigned long last_page_sz,
				  unsigned long vm_flags,
				  int flags,
				  unsigned page_fault_status)
{
	struct vma_section *sec = NULL;
	int rc = 0;
	int last_page_hash_cmp;
	/* The maximum number of digits in an unsigned long is 20,
	   plus 1 for space seperator to avoid name collisions */
	char path_with_offset[BIDE_MAX_FILE_PATH + 21] = {0};
	char *path_to_use = path_with_offset;
	unsigned path_to_use_sz;

	if (!path || !hash || hash_sz != SHA256_SIZE
		|| !last_page_hash || last_page_hash_sz != SHA256_SIZE )
		return -EINVAL;

	rc = snprintf(path_with_offset, sizeof(path_with_offset), "%s/%lu", path, vm_pgoff);
	if (rc < 0) {
		logError("Failed to add vm_pgoff to path. rc=%d.", rc);
		return -ENOMEM;
	} else {
		path_to_use_sz = (unsigned)rc;
	}

	if (mutex_lock_interruptible(&ctx.lock))
		return -ERESTARTSYS;

	/* Do a lookup to see if the hash already exists */
	rc = hash_search(&ctx.sections, path_to_use, path_to_use_sz, (void **) &sec);
	if (!rc) {
		/* Verify that the hash is valid */
		rc = memcmp(sec->hash, hash, SHA256_SIZE);
		if (!rc) {
			rc = 0;
			goto cleanup;
		}
		last_page_hash_cmp = memcmp(sec->last_page_hash, last_page_hash, SHA256_SIZE);

		logError("Hashes for section [%s] do not Match! Old:[0x%lx, %lu, %lu]"
			", New:[0x%lx, %lu, %lu][%d, %d][%d, %d][0x%lx, 0x%lx]",
			path_to_use,
			sec->sect_addr, sec->sect_sz, sec->last_page_sz,
			sect_addr, sect_sz, last_page_sz,
			rc, last_page_hash_cmp,
			sec->page_fault_status, page_fault_status,
			sec->vm_flags, vm_flags);

		rc = -EFAULT;
		goto cleanup;

	} else if (rc == -ENOENT) {

		logDebug("New section being hashed [%s][0x%lx, %lu, %lu][%d][0x%lx]",
			path_to_use, sect_addr, sect_sz, last_page_sz, page_fault_status, vm_flags);

		/* Check if new sections are allowed */
		if (flags & VMA_FLAG_DISALLOW_NEW_SECTIONS)
			goto cleanup;

		sec = kzalloc(sizeof(struct vma_section), GFP_ATOMIC);
		if (!sec) {
			logError("Failed to allocate vma_section.");
			rc = -ENOMEM;
			goto cleanup;
		}

		sec->path = kzalloc(path_to_use_sz + 1, GFP_ATOMIC);
		if (!sec->path) {
			logError("Failed to allocate vma_section->path.");

			kfree(sec);
			rc = -ENOMEM;
			goto cleanup;
		}

		memcpy(sec->path, path_to_use, path_to_use_sz);
		memcpy(sec->hash, hash, SHA256_SIZE);
		memcpy(sec->last_page_hash, last_page_hash, SHA256_SIZE);

		sec->sect_addr = sect_addr;
		sec->sect_sz = sect_sz;
		sec->last_page_sz = last_page_sz;
		sec->page_fault_status = page_fault_status;
		sec->vm_flags = vm_flags;

		/* Add the section to the hash map */
		rc = hash_insert(&ctx.sections, path_to_use, path_to_use_sz, sec);
		if (rc) {
			logError("Failed on hash_insert(). rc=%d", -rc);

			kfree(sec->path);
			kfree(sec);
		}
	} else
		logError("Failed on hash_search(). rc=%d.", -rc);

cleanup:
	mutex_unlock(&ctx.lock);
	return rc;
}

/*************************************************************************/

/*
 * This function checks to see if the given file should be scanned.
 *
 * Certain files are JIT compiled and do not come from the read only
 * filesystem.  These files are known to be updated over the lifetime of
 * the process and cannot be scanned and validated for malicious code
 * changes as they inherently change.
 *
 * @param name The name of the file
 * @return 1 of the file may be skipped; 0 otherwise
 *
 */
int skipScanning(const char *name)
{
	if(!name)
		return 1;

	if (0 == strcmp("/dev/ashmem/dalvik-jit-code-cache (deleted)", name))
		return 1;
	else if (0 == strncmp("/data/dalvik-cache/",
	                      name,
	                      sizeof("/data/dalvik-cache/") - 1))
		return 1;

	return 0;
}

/*************************************************************************/

/*
 * This function creates hashes for each read only and executable page of
 * the provided task. The hashes are checked against known values, if they
 * exist, and a report is generated if the hashes do not match.
 *
 * @param   task             The task data.
 * @param   flags            Operational flags.
 *
 * @return  0                No error.
 *          -ENOENT          No such file or directory.
 *          -ENOMEM          Out of memory.
 *          -EINVAL          Invalid argument.
 *          -ENAMETOOLONG    File name too long.
 *          -EBADE           Invalid exchange.
 *          -ENODATA         No data available.
 *          -EMSGSIZE        Message too long.
 */
int vma_scan_task(struct task_struct *task,
		  int flags)
{
	struct mm_struct *mm = NULL;
	struct vm_area_struct *vma = NULL;
	char name[BIDE_MAX_FILE_PATH] = { 0 };
	unsigned nsz = 0;
	int rc = 0;
	int hash_mismatch = 0;
	char *p;
	unsigned page_fault_status;

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

	logDebug("Hashing Process: %s(%d, %d).", name, (unsigned int)task->pid,
		(unsigned int)task->tgid);

	/* Write lock is necessary for page table fault handling */
	down_write(&mm->mmap_sem);

	/* Loop through all VMAs, hashing each one */
	for (vma = mm->mmap; vma; vma = vma->vm_next) {
		uint8_t hash[SHA256_SIZE] = { 0 };
		uint8_t hash_last_page[SHA256_SIZE] = { 0 };
		unsigned long last_page_sz = 0;
		char tmp[BIDE_MAX_FILE_PATH + 1] = { 0 };
		p = NULL;

		/* Don't hash anonymous sections */
		if (!vma->vm_file)
			continue;

		/* Limit the hashing to executable code */
		if ((vma->vm_flags & VM_READEXEC) != VM_READEXEC)
			continue;

		/* Extract the path name of the binary in this section */
		p = d_path(&vma->vm_file->f_path, tmp, BIDE_MAX_FILE_PATH);
		if (IS_ERR(p)) {
			rc = (int) -PTR_ERR(p);

			logError("Failed on d_path(). rc=%d", -rc);
			break;
		}

		/* Don't scan certain file-paths which are non-static */
		if (skipScanning(p)) {
			logInfo("Skipping the scan of %s:%s", name, p);
			continue;
		}

		page_fault_status = 0;
		/* Hash the process' page */
		rc = vma_hash_userspace(task, vma, hash, sizeof(hash),
			hash_last_page, &last_page_sz, sizeof(hash_last_page), &page_fault_status);
		if (rc) {
			logError("Failed on vma_hash_userspace(). rc=%d.", -rc);
			break;
		}

		/* Add or verify the hash against preexisting data */
		rc = vma_add_verify_section(p,
					    strnlen(p, BIDE_MAX_FILE_PATH - (p - tmp)),
					    vma->vm_pgoff,
					    hash,
					    sizeof(hash),
					    hash_last_page,
					    sizeof(hash_last_page),
					    vma->vm_start,
					    vma->vm_end - vma->vm_start,
					    last_page_sz,
					    vma->vm_flags,
					    flags,
					    page_fault_status);

		if (rc == -EFAULT) {
			hash_mismatch = 1;
			break;
		} else if (rc) {
			logError("Failed on vma_add_verify_section(). rc=%d.", -rc);
			break;
		}
	}

	up_write(&mm->mmap_sem);

	if(hash_mismatch == 1) {
		char msg[BAD_HASH_MESSAGE_LEN] = { 0 };

		/* Compile XML with offending process name and section */
		rc = xml_vma_msg(msg, sizeof(msg), name, p);
		if (rc < 0) {
			/* log this error but still coninue with reporting the incident */
			logError("Failed on xml_vma_msg(). rc=%d.", -rc);
			memset(msg, 0, BAD_HASH_MESSAGE_LEN);
		}

		/* Send an incident report to JBIDE */
		rc = report_incident(SEVERITY_WARNING,
				     SN_MISMATCHED_PROC_HASH,
				     msg,
				     task);
		if (rc)
			logError("Failed on report_incident() from vma_scan_task()."
				" rc=%d.", -rc);
	}
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
 * Print the content of the section tree. Here we assume that no string
 * will be greater than  100 character in length.
 */
#ifdef BID_USER_DEBUG
#define LINE_MAX_LEN	100
void vma_print_section_tree(void)
{
	int rc = 0;
	int count = 0;
	int cur_str_len = 0;
	int already_printed = 0;
	struct vma_section *sec = NULL;
	int size_left = LINE_MAX_LEN - 1;
	char line_buffer[LINE_MAX_LEN] = {0};
	struct rb_node *node;

	logError("---- Section tree ----");

	if (mutex_lock_interruptible(&ctx.lock))
		return;

	node = rb_first(&ctx.sections);

	while (node) {
		rc = hash_get(node, (void **)&sec);
		if (rc) {
			mutex_unlock(&ctx.lock);
			logError("Failed on hash_get(). rc=%d.", -rc);
			return;
		}
		if (sec == NULL) {
			logError("Failed on hash_get(). Pointer returned is NULL.");
			node = rb_next(node);
			continue;
		}

		cur_str_len = strlen(sec->path);

		/* Check if we have to print on two lines */
		if (size_left < cur_str_len) {
			already_printed = size_left;
			strncat(line_buffer, sec->path, size_left);

			/* Flush */
			logError("%s", line_buffer);
			size_left = LINE_MAX_LEN - 1;
			/* Clear buffer */
			memset(line_buffer, 0, LINE_MAX_LEN);

			/* Print what's remaining */
			strncat(line_buffer, sec->path + already_printed, size_left);
			size_left -= (cur_str_len-already_printed);
		} else {
			/* Copy the whole string and update size_left */
			strncat(line_buffer, sec->path, size_left);
			size_left -= cur_str_len;
		}

		/* Add a space or a line break depending on size_left*/
		if (size_left < 2) {
			logError("%s", line_buffer);
			size_left = LINE_MAX_LEN - 1;
			/* Clear buffer */
			memset(line_buffer, 0, LINE_MAX_LEN);
		} else {
			strncat(line_buffer, " ", 1);
			size_left--;
		}
		count++;
		node = rb_next(node);
	}
	mutex_unlock(&ctx.lock);

	if (size_left != LINE_MAX_LEN - 1)
		logError("%s", line_buffer);

	logError("Tree size: %d", count);
	logError("---- End section tree ----");
}
#endif

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
