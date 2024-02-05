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
#include <linux/dcache.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/highmem.h>
#include <linux/version.h>
#include <linux/capability.h>
#include <linux/uidgid.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"

/*************************************************************************/

static const char BASE64_VALS[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
				  "abcdefghijklmnopqrstuvwxyz"
				  "0123456789+/??????????????";

#define BASE64_BLOCK_SIZE           4
#define NULL_TERMINATOR_CHAR_SIZE   sizeof('\0')
#define BID_PATH_SIZE               128
#define ZZZ999_AS_NUMBER            865903913

static char g_build_fingerprint[PROP_VALUE_MAX] = "notyetset";

/*************************************************************************/

/*
 * Base 64 encodes a buffer. The output size must be large enough to
 * accommodate the encoded value and a null terminator.
 *
 * @param   src            The buffer to convert.
 * @param   src_sz         The length of the source buffer.
 * @param   dst            The output buffer to receive the encoded blob.
 * @param   dst_sz         The length of the output buffer.
 *
 * @return  0+             The amount of bytes written to dst.
 *          -EINVAL        Invalid parameters.
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
 *          -ENOMEM        Memory allocation failure.
 *          -ENAMETOOLONG  Buffer is too small.
 *          -ENOENT        Unable to find the exe path.
 */
int util_get_task_path(struct mm_struct *mm,
		       char *path,
		       unsigned path_sz)
{
	struct path *exe_path = NULL;
	int rc = -ENOENT;
	/*
	 * We only care about the first part of the path.  The path string
	 * is only used for analysis Of sensor reports.  Moreover, a longer path
	 * string would not fit into our sensor report.
	 */
	char tmp[BID_PATH_SIZE] = { 0 };

	if (!mm || !path || !path_sz)
		return -EINVAL;

	if (mm->exe_file) {
		path_get(&mm->exe_file->f_path);
		exe_path = &mm->exe_file->f_path;
	}

	if (exe_path) {
		char *p = NULL;

		/* Get the executable's path */
		p = d_path(exe_path, tmp, BID_PATH_SIZE);
		if (!IS_ERR(p)) {
			unsigned p_sz = strnlen(p, tmp + BID_PATH_SIZE - p);

			if (p_sz <= path_sz)
				rc = strlcpy(path, p, path_sz);
			else
				rc = -ENAMETOOLONG;
		} else {
			rc = (int) -PTR_ERR(p);

			logError("Failed on d_path(). rc=%d.", -rc);
		}

		path_put(exe_path);
	}

	return rc;
}

/*************************************************************************/

struct xml_char_mapping_str
{
	char from;
	const char *to;
	unsigned to_sz;
} xml_char_mapping_str_t;

static unsigned int XML_MAP_SZ = 5;

struct xml_char_mapping_str XML_MAP[] =
{
	{'<', "&lt;", 4},
	{'&', "&amp;", 5},
	{'>', "&gt;", 4},
	{'\"', "&quot;", 6},
	{'\'', "&apos;", 6}
};


/*
 * This method sanitizes the data that would be added to xml.
 *
 * There are five special characters in XML data which need to be sanitized:
 *    Character           Sanitized value
 *        <                  &lt;
 *        &                  &amp;
 *        >                  &gt;
 *        "                  &quot;
 *        '                  &apos;
 * If the input contains one of the above characters then it is replaced
 * by the sanitized value before returned.
 *
 * @param in_data[in]         The data to sanitize
 * @param in_data_sz[in]      The size of in_data
 * @param out_data[out]       The resulting sanitized data
 * @param out_data_sz[in/out] The initial and resulting size of the out_data
 *
 * @returns 0             Success
 *          -EINVAL       Invalid argument.
 *          -EMSGSIZE     Result buffer too short.
 */
int util_sanitize_xml_data(const char *in_data,
			   unsigned in_data_sz,
			   char *out_data,
			   unsigned *out_data_sz)
{
	char *cur = out_data;
	unsigned cur_out_data_sz = 0;
	bool replaced = false;
	unsigned i, j;

	if (in_data == NULL || out_data == NULL || out_data_sz == NULL)
		return -EINVAL;

	for (i = 0; i < in_data_sz; ++i) {
		replaced = false;
		for (j = 0; j < XML_MAP_SZ; ++j) {
			if (in_data[i] == XML_MAP[j].from) {
				if ((*out_data_sz - 1) >= cur_out_data_sz + XML_MAP[j].to_sz) {
					memcpy(cur, XML_MAP[j].to, XML_MAP[j].to_sz);
					cur += XML_MAP[j].to_sz;
					cur_out_data_sz += XML_MAP[j].to_sz;
					/* let's look at the next character in the input string */
					replaced = true;
					break;
				} else {
					return -EMSGSIZE;
				}
			}
		}
		if (!replaced) {
			if ((*out_data_sz - 1) >= cur_out_data_sz + 1) {
				*cur = in_data[i];
				cur++;
				cur_out_data_sz++;
			} else {
				return -EMSGSIZE;
			}
		}
	}

	*out_data_sz = cur_out_data_sz;
	return 0;
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
	char tmp_cmdline[BIDE_MAX_FILE_PATH] = {0};
	char* tmp_cmdline_ptr = tmp_cmdline;
	unsigned cmdline_sanitized_len;

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

	/* The command line should not be longer then the BIDE_MAX_FILE_PATH,
	   if it is let's not this and skip the command line */
	if( len > BIDE_MAX_FILE_PATH) {
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

	/* Copy the unsanitized command line into the temporary string */
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

		memcpy(tmp_cmdline_ptr, maddr + offset, bytes);

		kunmap(page);
		put_page(page);

		len -= bytes;
		tmp_cmdline_ptr += bytes;
		addr += bytes;
	}

	up_read(&mm->mmap_sem);

	cmdline_sanitized_len = sz;
	if ((rc = util_sanitize_xml_data(tmp_cmdline,
			strnlen(tmp_cmdline, BIDE_MAX_FILE_PATH - 1),
			cmdline,
			&cmdline_sanitized_len)) != 0)
		logError("Failed in util_sanitize_xml_data(): %d ", rc);

	/* Convert NULL-separators to spaces */
	for (len = 0; len < sz - 1; ++len) {
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
	int euid;
	int egid;

	if (!task)
		return -EINVAL;

	/* Obtaining the task's cred is a RCU dereference, requires lock */
	rcu_read_lock();

	cred = __task_cred(task);
	if (cred) {
		euid = CRED_UID(cred, euid);
		egid = CRED_GID(cred, egid);

		if ((ROOT_UID == euid) || (ROOT_GID == egid))
			rc = 1;
	}

	rcu_read_unlock();

	return rc;
}

/*************************************************************************/

/*
 * This function retrieves the PID of the real parent.
 *
 * @return  the pid of the real parent of the process
 */
unsigned long util_get_real_parent_pid(void)
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
	return (task != NULL)? task->tgid: -1;
}

/**************************************************************************/

/*
 * This function saves the build fingerprint.
 *
 * This method should be called at the time of the snapshot.
 *
 * @param fp the build fingerprint
 */
 void util_save_build_fingerprint(const char* fp)
 {
	if (fp == NULL)
		return;

	/* clear the currently set value */
	memset(g_build_fingerprint, 0, PROP_VALUE_MAX);
	strncpy(g_build_fingerprint, fp, PROP_VALUE_MAX);
 }

/**************************************************************************/

/*
 * Return the currently remembered fingerprint.
 *
 * @returns the currently set fingerprint
 */
 const char* util_get_build_fingerprint(void)
 {
	return g_build_fingerprint;
 }

/**************************************************************************/

/*
 * Formats the capabilities into a printable hex string
 *
 * The buffer should be at least CAP2HEXSTR_BUFFER_SIZE
 * bytes in size
 *
 * @param caps      The capabilities to format
 * @param buffer    The buffer to put the string
 * @param sz        The size of the buffer
 */
char* util_cap2hexstr(const kernel_cap_t *caps,
		      char *buffer,
		      size_t sz) {

	char *ptr = buffer;
	int rc;
	int i;

	if (sz < 3) {
		/* Not enough room, so just
		 * return an empty string if possible
		 */
		if (sz > 0)
			ptr[0] = '\0';
		return buffer;
	}
	*ptr++ = '0';
	*ptr++ = 'x';
	sz -= 2;

	for (i = _KERNEL_CAPABILITY_U32S-1; i >= 0; --i) {
		rc = snprintf(ptr, sz, "%08x", caps->cap[i]);
		if (rc < 0) {
			/* Error just abort */
			ptr[0] = '\0';
			break;
		} else if (rc == sz) {
			/* Truncated */
			break;
		}
		ptr += rc;
		sz -= rc;
	}

	return buffer;
}

/*
 * Converts a sensor name to a sensor id.
 *
 * @param        sensor    The specific KBIDE sensor to look up
 *
 * @returns      +#        Id of the sensor
 *               -ENOENT   Unknown sensor
 */
int util_sensor_name_to_integer(const char* sensor)
{
	int i;
	for (i = 0; i < SN_ARRAY_SIZE; i++) {
		if (strcmp(sensor, sensor_names[i]) == 0) {
			return i;
		}
	}
	logError("Invalid sensor name: %s", sensor);
	return -ENOENT;
}

/*
 * Converts a sensor id to the associated sensor name.
 *
 * @returns      string    Name of the sensor
 *               ""        Invalid id
 */
const char* util_integer_to_sensor_name(int sensor_id)
{
	if (sensor_id < 0 || sensor_id >= SN_ARRAY_SIZE) {
		logError("Invalid sensor id: %d", sensor_id);
		return "";
	}
	return sensor_names[sensor_id];
}


/*
 * Convert an OS version number from build format "AAA000" to a uint32_t to
 * be stored in RPMB. It is up to the caller to make sure uppercase letters
 * are used.
 * Format of number: | 00 | A-Z (5 bits) | A-Z (5 bits) | A-Z (5 bits) |
 *                               0-9 (5 bits) | 0-9 (5 bits) | 0-9 (5 bits) |
 *
 * @param    build    build number (must be in AAA000 format)
 * @param    number   converted build as a uint32_t number to be saved in the
 *                    database  (out)
 *
 * @return   0        Success
 *           -EINVAL  Invalid parameters
 */
int utils_bide_storage_build_to_number(const char *build,
				       uint32_t *number)
{
	/* check input, no need to worry about TOCTOU */
	if ((NULL == build) ||
		(OS_VERSION_BUILD_SIZE != strlen(build)) ||
		(NULL == number)) {
		logError("Invalid parameters");
		return -EINVAL;
	}
	if ((build[0] < 'A') || ('Z' < build[0]) ||
		(build[1] < 'A') || ('Z' < build[1]) ||
		(build[2] < 'A') || ('Z' < build[2]) ||
		(build[3] < '0') || ('9' < build[3]) ||
		(build[4] < '0') || ('9' < build[4]) ||
		(build[5] < '0') || ('9' < build[5])) {
		logError("Invalid build format");
		return -EINVAL;
	}

	/* perform conversion */
	*number =  (build[5] - '0');
	*number += (build[4] - '0') << 5;
	*number += (build[3] - '0') << 10;
	*number += (build[2] - 'A') << 15;
	*number += (build[1] - 'A') << 20;
	*number += (build[0] - 'A') << 25;

	return 0;
}


/*
 * Convert a number from a RPMB storage format to the more readable build
 * format "AAA000".
 * Format of number: | 00 | A-Z (5 bits) | A-Z (5 bits) | A-Z (5 bits) |
 *                          0-9 (5 bits) | 0-9 (5 bits) | 0-9 (5 bits) |
 *
 * @param    number   build version as a uint32_t number saved in the database
 * @param    build    build version string in AAA000 format (out)
 *
 * @returns  0        Success
 *           -EINVAL  Invalid parameters
 */
int utils_bide_storage_number_to_build(uint32_t number,
				       char build[BUILD_MAX_LEN])
{
	int32_t index;
	uint32_t offset;

	/* check input, no need to worry about TOCTOU */
	if ((ZZZ999_AS_NUMBER < number) || (NULL == build)) {
		logError("Invalid parameters, %d, %s, %d", number, build, (int) sizeof(build));
		return -EINVAL;
	}

	/* pre-clear the output buffer */
	memset(build, 0, BUILD_MAX_LEN);

	/* perform conversion */
	index = OS_VERSION_BUILD_SIZE - 1;

	/* For index positions 5,4,3 */
	while (index >= 3) {
		offset = (number & 0x1F);
		if (offset > 9)
			return -EINVAL;
		build[index] = '0' + offset;
		number >>= 5;
		index--;
	}

	while (index >= 0) {
		offset = (number & 0x1F);
		if ('A' + offset > 'Z')
			return -EINVAL;
		build[index] = 'A' + offset;
		number >>= 5;
		/* make sure we don't decrement this uint below zero */
		index--;
	}

	return 0;
}
