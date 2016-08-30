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
#include <linux/dcache.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/highmem.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"

/*************************************************************************/

static const char BASE64_VALS[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
				  "abcdefghijklmnopqrstuvwxyz"
				  "0123456789+/??????????????";

#define BASE64_BLOCK_SIZE 			4
#define NULL_TERMINATOR_CHAR_SIZE		sizeof('\0')
#define MAX_PATH				256

/*************************************************************************/

/*
 * Base 64 encodes a buffer. The output size must be large enough to
 * accomodate the encoded value and a null terminator.
 *
 * @param   src            The buffer to convert.
 * @param   src_sz         The length of the source buffer.
 * @param   dst            The output buffer to receive the encoded blob.
 * @param   dst_sz         The length of the output buffer.
 *
 * @return  0+             The amount of bytes written to dst.
 *          -EMSGSIZE      Output buffer is too small.
 */
int util_base64_encode(uint8_t *src,
		       unsigned src_sz,
		       char *dst,
		       unsigned dst_sz)
{
	uint8_t *src_end = src + src_sz;
	char *dst_end = dst + dst_sz;

	if (!src || !dst)
		return -EINVAL;

	while (src < src_end) {
		uint8_t a = *src++;

		/* Check that the destination has enough buffer space */
		if (dst + BASE64_BLOCK_SIZE >= dst_end + NULL_TERMINATOR_CHAR_SIZE)
			return -EMSGSIZE;

		*dst++ = BASE64_VALS[a >> 2];

		if (src < src_end) {
			uint8_t b = *src++;
			*dst++ = BASE64_VALS[((a & 3) << 4) | (b >> 4)];

			if (src < src_end) {
				uint8_t c = *src++;

				*dst++ = BASE64_VALS[((b & 15) << 2) | (c >> 6)];
				*dst++ = BASE64_VALS[c & 63];
			} else {
				*dst++ = BASE64_VALS[(b & 15) << 2];
				*dst++ = '=';
			}
		} else {
			*dst++ = BASE64_VALS[((a & 3) << 4)];
			*dst++ = '=';
			*dst++ = '=';
		}
	}

	*dst = '\0';

	/* Return the amount of bytes written */
	return dst_sz - (dst_end - dst);
}

/*************************************************************************/

/* Get current time as a string.
 *
 * @param   buf            The buffer to write the string into.
 * @param   buf_sz         The length of the buf.
 *
 * @return  0+             The amount of bytes written.
 *          -EMSGSIZE      Buffer is too small.
 */
int util_get_time_string(char *buf,
			 unsigned buf_sz)
{
	struct timespec tv = {};
	int rc = 0;

	/* Get current time in seconds since epoc */
	getnstimeofday(&tv);

	/* Put time into a string */
	rc = snprintf(buf, buf_sz, "%ld", tv.tv_sec);
	if (rc <= 0) {
		logError("Failed on snprintf(). rc=%d.", -rc);
		return -EMSGSIZE;
	}

	return rc;
}

/*************************************************************************/

/*
 * Gets the executable path of the current task.
 *
 * @param   mm             The task to query.
 * @param   path           The buffer to write the path into (out).
 * @param   path_sz        The size of the buffer.
 *
 * @return  0+             Number of bytes written.
 *          -EINVAL        Invalid input parameters.
 *          -ENOMEM        Memory allocation failiure.
 *          -ENAMETOOLONG  Buffer is too small.
 */
int util_get_task_path(struct mm_struct *mm,
		       char *path,
		       unsigned path_sz)
{
	struct path *exe_path = NULL;
	int rc = -ENOENT;

	if (!mm || !path || !path_sz)
		return -EINVAL;

	down_read(&mm->mmap_sem);

	if (mm->exe_file) {
		path_get(&mm->exe_file->f_path);
		exe_path = &mm->exe_file->f_path;
	}

	up_read(&mm->mmap_sem);

	if (exe_path) {
		char *p = NULL;
		char *tmp = NULL;

		tmp = kzalloc(PAGE_SIZE, GFP_KERNEL);
		if (!tmp) {
			logError("Failed to allocate memory for task name");

			path_put(exe_path);
			return -ENOMEM;
		}

		/* Get the executable's path */
		p = d_path(exe_path, tmp, PAGE_SIZE);
		if (!IS_ERR(p)) {
			unsigned p_sz = strnlen(p, tmp + PAGE_SIZE - p);

			if (p_sz <= path_sz)
				rc = strlcpy(path, p, path_sz);
			else
				rc = -ENAMETOOLONG;
		} else {
			rc = (int) -PTR_ERR(p);

			logError("Failed on d_path(). rc=%d.", -rc);
		}

		kfree(tmp);
		path_put(exe_path);
	}

	return rc;
}

/*************************************************************************/

/*
 * This function retrieves the command line parameters used when the task
 * began execution.
 *
 * @param   task           The task to query.
 * @param   cmdline        A buffer for the command line parameters.
 * @param   sz             The size of the buffer.
 *
 * @return  0+             Number of bytes written.
 *          -EINVAL        Invalid input parameters.
 *          -ENODATA       Task does not have mm_struct.
 *          -EMSGSIZE      Buffer is too small.
 */
int util_get_task_cmdline(struct task_struct *task,
			  char *cmdline,
			  unsigned sz)
{
	struct mm_struct *mm = NULL;
	unsigned long addr = 0;
	char *p = cmdline;
	int len = 0;
	int rc = 0;

	if (!task || !cmdline)
		return -EINVAL;

	/* Kernel tasks don't have an mm, ignore them */
	mm = get_task_mm(task);
	if (!mm)
		return -ENODATA;

	addr = mm->arg_start;
	len = mm->arg_end - mm->arg_start;

	/* The command line should not be longer then the MAX_PATH, if it is let's not this and skip the command line */
	if( len > MAX_PATH) {
		logError("Failed in util_get_task_cmdline(pid:%d): len:%d, sz:%d.", current->pid, len, sz);
		rc = 0;
		goto cleanup;
	}

	if (len > sz) {
		logError("Failed in util_get_task_cmdline(): %d > %d.", len, sz);
		rc = -EMSGSIZE;
		goto cleanup;
	}

	down_read(&mm->mmap_sem);

	while (len > 0) {
		struct vm_area_struct *vma = NULL;
		struct page *page = NULL;
		void *maddr = NULL;
		int bytes = len;
		int offset = addr & (PAGE_SIZE - 1);

		if (get_user_pages(task, mm, addr, 1, 0, 1, &page, &vma) <= 0)
			break;

		maddr = kmap(page);
		bytes = len;

		if (bytes > PAGE_SIZE - offset)
			bytes = PAGE_SIZE - offset;

		memcpy(cmdline, maddr + offset, bytes);

		kunmap(page);
		put_page(page);

		len -= bytes;
		cmdline += bytes;
		addr += bytes;
	}

	up_read(&mm->mmap_sem);

	/* Convert NULL-separators to spaces */
	for (len = 0; len < sz; ++len) {
		if (p[len] == '\0' && p[len + 1] != '\0')
			p[len] = ' ';
		else if (p[len] == '\0' && p[len + 1] == '\0')
			break;
	}

	rc = len;

cleanup:

	if (mm)
		mmput(mm);

	return rc;
}

/*************************************************************************/

/*
 * This function determines if a task is a root process.
 *
 * @param   task           The task to query.
 *
 * @return  1              Task is a root process.
 *          0              Task is not root.
 *          -EINVAL        Invalid parameter.
 */
int util_is_task_root(struct task_struct *task)
{
	const struct cred *cred = NULL;
	int rc = 0;

	if (!task)
		return -EINVAL;

	/* Obtaining the task's cred is a RCU dereference, requires lock */
	rcu_read_lock();

	cred = __task_cred(task);
	if (cred)
		rc = (cred->euid == 0 || cred->egid == 0) ? 1 : 0;

	rcu_read_unlock();

	return rc;
}

/*************************************************************************/

/*
 * This function retrieves the PID of the real parent.
 *
 * @return  the pid of the real parent of the process
 */
unsigned long util_get_real_parent_pid()
{
	unsigned long ppid = 0;
	struct task_struct *parent = NULL;

	rcu_read_lock();

	/* learn about our parent */
	parent = rcu_dereference(current->real_parent);
	ppid = task_tgid_vnr(parent);

	rcu_read_unlock();

	return ppid;
}

/*************************************************************************/

/*
 * This function retrieves the PID of the current process group leader.
 *
 * Note, despite the name, this returns the tgid not the pid.  The tgid and
 * the pid are identical unless CLONE_THREAD was specified on clone() in
 * which case the tgid is the same in all threads of the same group.
 *
 * @param   task           The task to query.
 *
 * @return  the pid of the process group leader
 */
int util_get_tgid(struct task_struct *task)
{
	return task_tgid_vnr(task);
}
