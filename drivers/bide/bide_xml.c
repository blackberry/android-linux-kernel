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
#include <linux/string.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/version.h>
#include <linux/cred.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"
#include "bide_proc.h"

/*************************************************************************/

static const char CAPABILITY_TEMPLATE[] =
	"<cap_%s>"
	  "<effective>%s</effective>"
	  "<inheritable>%s</inheritable>"
	  "<permitted>%s</permitted>"
	"</cap_%s>";

static const char DENIED_CAPS_TEMPLATE[] =
	"<denied_caps>%s</denied_caps>";

static const char VMA_TEMPLATE[] =
	"<process nm='%s'>"
	  "<section>%s</section>"
	"</process>";

static const char TIME_TAGS[]		= "<time>%ld</time>";
static const char DEV_NAME_TAGS[]	= "<devnm>%s</devnm>";
static const char PID_TAGS[]		= "<pid>%d</pid>";
static const char NAME_TAGS[]		= "<nm>%s</nm>";
static const char VERSION_TAGS[]	= "<version>1</version>";
static const char JBIDE_HASH_TAG[]	= "<jbidehash>%s</jbidehash>";
static const char CMD_LINE_START_TAG[]	= "<cmdline>";
static const char CMD_LINE_END_TAG[]	= "</cmdline>";
static const char REPORT_START_TAG[]	= "<kbide>";
static const char REPORT_END_TAG[]	= "</kbide>";
static const char PROPERTY_TAG[] 	= "<property nm='%s' value='%s' />";
static const char INCIDENT_START_TAG[]	= "<failure sensor='%s' severity='%d'>";
static const char INCIDENT_END_TAG[]	= "</failure>";
static const char ECRED_TAG[]		= "<ecred euid='%d' egid='%d' />";
static const char RCRED_TAG[]		= "<cred uid='%d' gid='%d' />";
static const char PROCESS_SINGLE_TAG[]	= "<process pid='%d' ppid='%d' procnm='%s' />";
static const char PROCESS_START_TAG[]	= "<process pid='%d'>";
static const char PROCESS_END_TAG[]	= "</process>";
static const char GROUPS_START_TAG[]	= "<groups>";
static const char GROUPS_END_TAG[]	= "</groups>";
static const char LINEAGE_START_TAG[]	= "<lineage>";
static const char LINEAGE_END_TAG[]	= "</lineage>";
static const char FINGERPRINT_TAG[]	= "<fp>%s</fp>";
static const char TZ_ERROR_TAG[]	= "<tzerr>%d</tzerr>";
static const char JBIDE_SENSORS_TAG[]	= "<jsens>%s</jsens>";
static const char KBIDE_SENSORS_TAG[]	= "<ksens>%s</ksens>";
static const char PRIVILEGED_GID_TAG[]  = "<privileged_gid%s%s%s>%d</privileged_gid>";

/*************************************************************************/

struct xml_ctx {
	char *buf;
	unsigned sz;
	unsigned off;
};

/* Context Helper Macros */
#define XML_INIT(b, s)	{ .buf = b, .sz  = s, .off = b ? strnlen(b, s) : 0 }

/*************************************************************************/

/*
 * This function is a wrapper around vsnprintf that checks the boundaries
 * of the buffer being written to and updates offsets.
 *
 * @param   ctx                 The xml context buffer.
 * @param   fmt                 The tag format of the string.
 * @param   ...                 Various other arguments.
 *
 * @return  0+                  The amount of bytes written.
 *          -EINVAL             Invalid argument.
 *          -EMSGSIZE           Message too long.
 */
static __printf(2, 3) int xml_write_tag(struct xml_ctx *ctx,
					const char *fmt,
					...)
{
	va_list args;
	int rc = 0;

	if (!ctx || !fmt)
		return -EINVAL;

	if (ctx->off >= ctx->sz) {
		ctx->buf[ctx->sz - 1] = '\0';
		logError("xml_write_tag()[ctx->off=%d, ctx->sz=%d, ctx->buf.l=%d, "
		         "ctx->buf='%s'", (int)ctx->off, (int)ctx->sz,
		         (int)strlen(ctx->buf), ctx->buf);
		return -EMSGSIZE;
	}

	va_start(args, fmt);
	rc = vsnprintf(ctx->buf + ctx->off, ctx->sz - ctx->off, fmt, args);
	va_end(args);

	if (rc < 0 || rc + ctx->off >= ctx->sz) {
		ctx->buf[ctx->sz - 1] = '\0';
		logError("xml_write_tag(%d) ctx->off=%d, ctx->sz=%d, ctx->buf.l=%d, "
		         "ctx->buf='%s'", rc, (int)ctx->off, (int)ctx->sz,
		         (int)strlen(ctx->buf), ctx->buf);
		return -EMSGSIZE;
	}

	/* Update offset with size of buffer written */
	ctx->off += rc;

	return rc;
}

/*************************************************************************/

/*
 * This function creates a message for capabilities.
 *
 * @param   buf                 The buffer that will be written to (out).
 * @param   sz                  Size of the buffer.
 * @param   eff                 The effective capabilities being set.
 * @param   inh                 The inheritable capabilities being set.
 * @param   per                 The permitted capabilities being set.
 * @param   state               Current or new capability.
 *
 * @return  0+                  The amount of bytes written.
 *          -EINVAL             Invalid argument.
 *          -EMSGSIZE           Message too long.
 */
int xml_capset_msg(char *buf,
		   unsigned sz,
		   const kernel_cap_t *eff,
		   const kernel_cap_t *inh,
		   const kernel_cap_t *per,
		   CAP_STATE state)
{
	struct xml_ctx ctx = XML_INIT(buf, sz);
	char buf1[CAP2HEXSTR_BUFFER_SIZE];
	char buf2[CAP2HEXSTR_BUFFER_SIZE];
	char buf3[CAP2HEXSTR_BUFFER_SIZE];
	int rc = 0;

	if (!buf || !sz || !eff || !inh || !per)
		return -EINVAL;

	/* Generate the XML message */
	rc = xml_write_tag(&ctx,
			   CAPABILITY_TEMPLATE,
			   state == CAP_NEW ? "new" : "current",
			   util_cap2hexstr(eff, buf1, sizeof(buf1)),
			   util_cap2hexstr(inh, buf2, sizeof(buf2)),
			   util_cap2hexstr(per, buf3, sizeof(buf3)),
			   state == CAP_NEW ? "new" : "current");
	if (rc < 0)
		logError("Failed on xml_write_tag(). rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function creates a message for escalated gids
 *
 * @param   buf                 The buffer that will be written to (out).
 * @param   sz                  Size of the buffer.
 * @param   infos               Information about the privileged gids that
 *                              are being acquired
 * @param   num_infos           The number of gids being acquired
 *
 * @return  0+                  The amount of bytes written.
 *          -EINVAL             Invalid argument.
 *          -EMSGSIZE           Message too long.
 */
int xml_escalated_gid_msg(char *buf,
			  unsigned sz,
			  const gid_info_t *infos,
			  int num_infos)
{
	struct xml_ctx ctx = XML_INIT(buf, sz);
	int rc = 0;
	int i;
	const struct cred *cred = current_cred();
	int num = 0;

	if (!buf || !sz || !infos || num_infos <= 0)
		return -EINVAL;

	for (i = 0; i < num_infos; ++i) {
		rc = xml_write_tag(&ctx, PRIVILEGED_GID_TAG,
				   (infos[i].gid_types & GID_REAL) ? " r=1" : "",
				   (infos[i].gid_types & GID_EFFECTIVE) ? " e=1" : "",
				   (infos[i].gid_types & GID_SUPPLEMENTARY) ? " s=1" : "",
				   from_kgid(cred->user_ns, infos[i].gid));
		if (rc < 0) {
			logError("Failed on xml_write_tag(). rc=%d.", rc);
			return rc;
		}
		num += rc;
	}

	return num;
}

/*************************************************************************/

/*
 * This function creates a message for denied capabilities
 *
 * @param   buf                 The buffer that will be written to (out).
 * @param   sz                  Size of the buffer.
 * @param   denied              The capabilities that were denied.
 *
 * @return  0+                  The amount of bytes written.
 *          -EINVAL             Invalid argument.
 *          -EMSGSIZE           Message too long.
 */
int xml_denied_caps_msg(char *buf,
			unsigned sz,
			const kernel_cap_t *denied)
{
	struct xml_ctx ctx = XML_INIT(buf, sz);
	char strbuf[CAP2HEXSTR_BUFFER_SIZE];
	int rc = 0;

	if (!buf || !sz || !denied)
		return -EINVAL;

	/* Generate the XML message */
	rc = xml_write_tag(&ctx,
			   DENIED_CAPS_TEMPLATE,
			   util_cap2hexstr(denied, strbuf, sizeof(strbuf)));
	if (rc < 0)
		logError("Failed on xml_write_tag(DENIED_CAPS_TEMPLATE). rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function generates a message for a process that has a mismatched
 * hash.
 *
 * @param   buf                 The buffer that will be written to (out).
 * @param   sz                  Size of the buffer.
 * @param   proc                The name of the process.
 * @param   section             The section that failed the hash check.
 *
 * @return  0+                  The amount of bytes written.
 *          -EINVAL             Invalid argument.
 *          -EMSGSIZE           Message too long.
 */
int xml_vma_msg(char *buf,
		unsigned sz,
		const char *proc,
		const char *section)
{
	struct xml_ctx ctx = XML_INIT(buf, sz);
	int rc = 0;

	if (!buf || !sz || !proc || !section)
		return -EINVAL;

	/* Generate process info XML */
	rc = xml_write_tag(&ctx, VMA_TEMPLATE, proc, section);
	if (rc < 0)
		logError("Failed on xml_write_tag(). rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function creates a message for properties.
 *
 * @param   buf                 The buffer that will be written to (out).
 * @param   sz                  Size of the buffer.
 * @param   name                The name of the property.
 * @param   value               The value of the property.
 *
 * @return  0+                  The amount of bytes written.
 *          -EINVAL             Invalid argument.
 *          -EMSGSIZE           Message too long.
 */
int xml_property_msg(char *buf,
		     unsigned sz,
		     const char *name,
		     const char *value)
{
	struct xml_ctx ctx = XML_INIT(buf, sz);
	int rc = 0;

	if (!buf || !sz || !name || !value )
		return -EINVAL;

	/* Generate the XML message */
	rc = xml_write_tag(&ctx, PROPERTY_TAG, name, value);
	if (rc < 0)
		logError("Failed on xml_write_tag(). rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function creates a message displaying the error code
 *
 * @param   buf                 The buffer that will be written to (out).
 * @param   sz                  Size of the buffer.
 * @param   error_code          Error code to print
 *
 * @return  0+                  The amount of bytes written.
 *          -EINVAL             Invalid argument.
 *          -EMSGSIZE           Message too long.
 */
int xml_nonce_mismatch_msg(char *buf,
			   unsigned sz,
			   int error_code)
{
	struct xml_ctx ctx = XML_INIT(buf, sz);
	int rc = 0;

	if (!buf || !sz || !error_code )
		return -EINVAL;

	/* Generate the XML message */
	rc = xml_write_tag(&ctx, TZ_ERROR_TAG, error_code);
	if (rc < 0)
		logError("Failed on xml_write_tag(). rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function creates a message for a mount path.
 *
 * @param   buf                 The buffer that will be written to (out).
 * @param   sz                  Size of the buffer.
 * @param   path                The path of the mount.
 *
 * @return  0+                  The amount of bytes written.
 *          -EINVAL             Invalid argument.
 *          -EMSGSIZE           Message too long.
 */
int xml_mount_path_msg(char *buf,
		       unsigned sz,
		       const char *path)
{
	struct xml_ctx ctx = XML_INIT(buf, sz);
	int rc = 0;

	if (!buf || !sz || !path)
		return -EINVAL;

	/* Generate the XML message */
	rc = xml_write_tag(&ctx, DEV_NAME_TAGS, path);
	if (rc < 0)
		logError("Failed on xml_write_tag(). rc=%d.", rc);

	return rc;
}

/*************************************************************************/
/********************************* KBIDE *********************************/
/*************************************************************************/

/*
 * This function is called when the kbide report buffer is currently on
 * the heap, and it increases the buffer size.
 *
 * @param   rep                 The report structure.
 *
 * @return  0                   No error.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 */
static int xml_resize_heap(bide_reports_t *rep)
{
	uint8_t *temp = NULL;

	if (!rep)
		return -EINVAL;

	temp = krealloc(rep->kbide, rep->kbide_size, GFP_ATOMIC);
	if (!temp) {
		kfree(rep->kbide);
		rep->kbide = NULL;
		logError("xml_resize_heap(): krealloc() failed.");
		return -ENOMEM;
	}
	rep->kbide = temp;
	/* Set the newly allocated space to null. */
	memset(rep->kbide + rep->kbide_offset, 0,
	       rep->kbide_size - rep->kbide_offset);
	return 0;
}

/*************************************************************************/

/*
 * This function is called when the kbide report buffer is currently on
 * the stack. It allocates a bigger buffer on the heap and copies the
 * contents of the stack buffer to the heap buffer.
 *
 * @param   rep                 The report structure.
 *
 * @return  0                   No error.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 */
static int xml_change_stack_to_heap(bide_reports_t *rep)
{
	if (!rep)
		return -EINVAL;

	rep->kbide = kzalloc(rep->kbide_size, GFP_ATOMIC);
	if (!rep->kbide) {
		logError("xml_change_stack_to_heap(): kzalloc() failed.");
		return -ENOMEM;
	}
	strncpy(rep->kbide, rep->kbide_stack, KBIDE_START_SIZE);
	rep->kbide_stack[0] = '\0';
	rep->is_kbide_on_heap = 1;
	return 0;
}

/*************************************************************************/

/*
 * This function manages the kbide heap memory.
 *
 * @param   rep                 The report structure.
 * @param   len                 Amount of bytes to add to buffer.
 *
 * @return  0                   No error.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 */
static int xml_manage_heap(bide_reports_t *rep, int len)
{
	int new_size = 0;

	if (!rep || len < 0)
		return -EINVAL;

	/* Get new size, most likely won't go into while loop. */
	new_size = rep->kbide_size + KBIDE_STEP_SIZE;
	while (len + rep->kbide_offset >= new_size) {
		new_size += KBIDE_STEP_SIZE;
	}
	rep->kbide_size = new_size;
	if (rep->is_kbide_on_heap) {
		return xml_resize_heap(rep);
	}
	return xml_change_stack_to_heap(rep);
}

/*************************************************************************/

/*
 * This function is a wrapper around vsnprintf that checks the boundaries
 * of the buffer being written to and updates offsets. If the buffer runs
 * out of space, the buffer space will increase as long as the buffer is
 * allowed to be stored on heap.
 *
 * @param   rep                 The report structure.
 * @param   fmt                 The tag format of the string.
 * @param   ...                 Various other arguments.
 *
 * @return  0                   No error.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -EBADE              Invalid exchange.
 *          -EMSGSIZE           Message too long.
 */
static __printf(2, 3) int xml_write_kbide(bide_reports_t *rep,
					  const char *fmt,
					  ...)
{
	va_list args;
	int len = 0;

	/* Check input and kbide size. */
	if (!rep || !fmt)
		return -EINVAL;
	if (rep->kbide_size == 0) {
		logError("xml_write_kbide(): kbide size is 0, "
			 "likely not initialized.");
		return -EBADE;
	}

	/* Check size it would take. */
	va_start(args, fmt);
	len = vsnprintf(0, 0, fmt, args);
	if (len < 0) {
		logError("xml_write_kbide(): vsnprintf() encoding error "
			 "when checking size it would take.");
		va_end(args);
		return len;
	}

	/* Determine if buffer size increase is needed. */
	if (len + rep->kbide_offset >= rep->kbide_size) {
		int rc = 0;
		if (!rep->can_kbide_be_on_heap) {
			rep->kbide[rep->kbide_size - 1] = '\0';
			logError("xml_write_kbide(): cannot resize the buffer as the "
				 "buffer cannot be on heap if originated from JNI.");
			va_end(args);
			return -EMSGSIZE;
		}
		rc = xml_manage_heap(rep, len);
		if (rc) {
			logError("xml_write_kbide(): failed on xml_manage_heap(). rc=%d.",
				 rc);
			va_end(args);
			return rc;
		}
	}

	/* Write to buffer. */
	len = vsnprintf(rep->kbide + rep->kbide_offset,
		       rep->kbide_size - rep->kbide_offset,
		       fmt,
		       args);
	va_end(args);
	if (len < 0) {
		logError("xml_write_kbide(): vsnprintf() encoding error "
			 "when actually writing to buffer.");
		return len;
	}

	/* Update offset with size of buffer written. */
	rep->kbide_offset += len;
	return 0;
}

/*************************************************************************/

/*
 * This function concatenates a message to the kbide report buffer.
 *
 * @param   rep                 The report structure.
 * @param   msg                 The message.
 *
 * @return  0                   No error.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -EBADE              Invalid exchange.
 *          -EMSGSIZE           Message too long.
 */
int xml_cat(bide_reports_t *rep, const char *msg)
{
	int rc = 0;

	if (!rep || !msg)
		return -EINVAL;

	rc = xml_write_kbide(rep, "%s", msg);
	if (rc)
		logError("xml_cat(): failed on xml_write_kbide(). rc=%d.",
			 rc);

	return rc;
}

/*************************************************************************/

/*
 * This function adds the JBIDE hash into the message buffer.
 *
 * @param   rep                 The report structure.
 * @param   jhash               The base64 encoded hash of the JBIDE report.
 * @param   hash_sz             The size of the encoded hash.
 *
 * @return  0                   No error.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -EBADE              Invalid exchange.
 *          -EMSGSIZE           Message too long.
 */
int xml_jbide_hash_msg(bide_reports_t *rep,
		       const char *jhash,
		       unsigned hash_sz)
{
	int rc = 0;

	if (!rep || !jhash || hash_sz != BASE64_SHA256_SIZE)
		return -EINVAL;

	/* Generate the XML message */
	rc = xml_write_kbide(rep, JBIDE_HASH_TAG, jhash);
	if (rc)
		logError("xml_jbide_hash_msg(): failed on "
			 "xml_write_kbide(). rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function adds the list of supplementary groups for the process to
 * the xml report.
 *
 * @param   rep                 The report structure.
 * @param   cred                The process's cred struct.
 *
 * @return  0                   No error.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -EBADE              Invalid exchange.
 *          -EMSGSIZE           Message too long.
 */
static int xml_process_groups_msg(bide_reports_t *rep,
				  const struct cred *cred)
{
	gid_t gid = 0;
	int rc = 0;
	int i = 0;
	struct group_info *gi;

	if (!rep || !cred)
		return -EINVAL;

	/* Start <groups> tag. */
	rc = xml_write_kbide(rep, GROUPS_START_TAG);
	if (rc) {
		logError("xml_process_groups_msg(): failed on "
			 "xml_write_kbide() for start tag. rc=%d.", rc);
		return rc;
	}

	/* Write groups. */
	gi = get_group_info(cred->group_info);
	for (i = 0; i < cred->group_info->ngroups; i++) {
		gid = __kgid_val(GROUP_AT(cred->group_info, i));
		rc = xml_write_kbide(rep, (i != 0) ? ",%d" : "%d", gid);
		if (rc) {
			logError("xml_process_groups_msg(): failed on "
				 "xml_write_kbide(). rc=%d.", rc);
			put_group_info(gi);
			return rc;
		}
	}
	put_group_info(gi);

	/* End </groups> tag. */
	rc = xml_write_kbide(rep, GROUPS_END_TAG);
	if (rc)
		logError("xml_process_groups_msg(): failed on "
			 "xml_write_kbide() for end tag. rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function writes out the command line parameters that were used
 * when the task was executed. The first parameter is always the name of
 * the executable.
 *
 * @param   rep                 The report structure.
 * @param   task                The process' task struct.
 *
 * @return  0                   No error.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -EBADE              Invalid exchange.
 *          -ENODATA            No data available.
 *          -EMSGSIZE           Message too long.
 */
static int xml_process_cmdline_msg(bide_reports_t *rep,
				   struct task_struct *task)
{
	int rc = 0;
	char cmdline[BIDE_MAX_FILE_PATH + 1] = { 0 };

	if (!rep || !task)
		return -EINVAL;

	rc = xml_write_kbide(rep, CMD_LINE_START_TAG);
	if (rc) {
		logError("xml_process_cmdline_msg(): failed on "
			 "xml_write_kbide() for start tag. rc=%d.", rc);
		return rc;
	}

	rc = util_get_task_cmdline(task, cmdline, sizeof(cmdline));
	if (rc < 0) {
		logError("xml_process_cmdline_msg(): failed on "
			 "util_get_task_cmdline(). rc=%d.", rc);
		return rc;
	}

	rc = xml_write_kbide(rep, cmdline);
	if (rc) {
		logError("xml_process_cmdline_msg(): failed on "
			 "xml_write_kbide() for cmdline. rc=%d.", rc);
		return rc;
	}

	rc = xml_write_kbide(rep, CMD_LINE_END_TAG);
	if (rc)
		logError("xml_process_cmdline_msg(): failed on "
			 "xml_write_kbide() for end tag. rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function creates a message for capabilities.
 *
 * @param   rep                 The report structure.
 * @param   eff                 The effective capabilities being set.
 * @param   inh                 The inheritable capabilities being set.
 * @param   per                 The permitted capabilities being set.
 * @param   state               Current or new capability.
 *
 * @return  0                   No error.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -EBADE              Invalid exchange.
 *          -EMSGSIZE           Message too long.
 */
static int xml_process_capset_msg(bide_reports_t *rep,
				  const kernel_cap_t *eff,
				  const kernel_cap_t *inh,
				  const kernel_cap_t *per,
				  CAP_STATE state)
{
	char buf1[CAP2HEXSTR_BUFFER_SIZE];
	char buf2[CAP2HEXSTR_BUFFER_SIZE];
	char buf3[CAP2HEXSTR_BUFFER_SIZE];
	int rc = 0;

	if (!rep || !eff || !inh || !per)
		return -EINVAL;

	/* Generate the XML message */
	rc = xml_write_kbide(rep,
			     CAPABILITY_TEMPLATE,
			     state == CAP_NEW ? "new" : "current",
			     util_cap2hexstr(eff, buf1, sizeof(buf1)),
			     util_cap2hexstr(inh, buf2, sizeof(buf2)),
			     util_cap2hexstr(per, buf3, sizeof(buf3)),
			     state == CAP_NEW ? "new" : "current");
	if (rc)
		logError("xml_process_capset_msg(): failed on "
			 "xml_write_kbide(). rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function creates a message for root processes.
 *
 * @param   rep                 The report structure.
 * @param   pinfo               Process info
 *
 * @return  0                   No error.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -EBADE              Invalid exchange.
 *          -EMSGSIZE           Message too long.
 */
static int xml_process_msg_pinfo(bide_reports_t *rep,
				 const struct process_info *pinfo)
{
	int ppid = -1;
	int rc = 0;
	const struct process_info *ppinfo;

	if (!rep || !pinfo)
		return -EINVAL;

	/* Get parent task's pid */
	ppinfo = proc_get_parent(pinfo);
	if (ppinfo != NULL)
		ppid = ppinfo->pid;
	proc_put_info(ppinfo);

	/* Generate the XML message */
	rc = xml_write_kbide(rep,
			     PROCESS_SINGLE_TAG,
			     pinfo->pid,
			     ppid,
			     pinfo->name);
	if (rc)
		logError("xml_process_msg_pinfo(): failed on "
			 "xml_write_kbide(). rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function creates a message for root processes.
 *
 * Note: caller must ensure rcu is locked for read
 *
 * @param   rep                 The report structure.
 * @param   task                The process' task struct.
 *
 * @return  0                   No error.
 *          -ENOENT             No such file or directory.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -ENAMETOOLONG       File name too long.
 *          -EBADE              Invalid exchange.
 *          -EMSGSIZE           Message too long.
 */
static int xml_process_msg(bide_reports_t *rep, struct task_struct *task)
{
	char name[BIDE_MAX_FILE_PATH] = { 0 };
	int ppid = 0;
	int pid = 0;
	int rc = 0;

	if (!rep || !task)
		return -EINVAL;

	pid = task_tgid_vnr(task);

	rc = util_get_task_path(task->mm, name, sizeof(name));
	if (rc < 0) {
		logError("xml_process_msg(): failed on "
			 "util_get_task_path(). rc=%d.", rc);
		return rc;
	}

	/* Get parent task - rcu already locked by caller */
	task = rcu_dereference(task->real_parent);

	if (task)
		ppid = task_tgid_vnr(task);

	/* Generate the XML message */
	rc = xml_write_kbide(rep, PROCESS_SINGLE_TAG, pid, ppid, name);
	if (rc)
		logError("xml_process_msg(): failed on "
			 "xml_write_kbide(). rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function lists the parent tree of a process in XML format.
 *
 * @param   rep                 The report structure.
 * @param   task                The process' task struct.
 *
 * @return  0                   No error.
 *          -ENOENT             No such file or directory.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -ENAMETOOLONG       File name too long.
 *          -EBADE              Invalid exchange.
 *          -EMSGSIZE           Message too long.
 */
static int xml_process_lineage_msg(bide_reports_t *rep,
				   struct task_struct *task)
{
	int rc = 0;
	const struct process_info *pinfo;
	const struct process_info *ppinfo;

	if (!rep || !task)
		return -EINVAL;

	/* Start <lineage> tag */
	rc = xml_write_kbide(rep, LINEAGE_START_TAG);
	if (rc) {
		logError("xml_process_lineage_msg(): failed on "
			 "xml_write_kbide() for start tag. rc=%d.", rc);
		return rc;
	}

	pinfo = proc_get_info(util_get_tgid(task));
	if (pinfo) {
		while (pinfo) {
			/* Generate XML about the process */
			rc = xml_process_msg_pinfo(rep, pinfo);
			if (rc) {
				proc_put_info(pinfo);
				logError("xml_process_lineage_msg(): failed on "
					 "xml_process_msg_pinfo(). rc=%d.", rc);
				return rc;
			}
			/* Get next parent task */
			ppinfo = proc_get_parent(pinfo);
			proc_put_info(pinfo);
			pinfo = ppinfo;
		}

	} else {
		/*
		 * We don't have information about this process.
		 * Use the old style lineage
		 */
		rcu_read_lock();
		while (task && task->mm) {
			/* Generate XML about the process */
			rc = xml_process_msg(rep, task);
			if (rc) {
				rcu_read_unlock();
				logError("xml_process_lineage_msg(): failed on "
					 "xml_process_msg(). rc=%d.", rc);
				return rc;
			}
			/* Get next parent task */
			task = rcu_dereference(task->real_parent);
		}
		rcu_read_unlock();
	}

	/* Finish </lineage> tag */
	rc = xml_write_kbide(rep, LINEAGE_END_TAG);
	if (rc)
		logError("xml_process_lineage_msg(): failed on "
			 "xml_write_kbide() for end tag. rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function dumps relevant process information into XML format.
 *
 * @param   rep                 The report structure.
 * @param   task                The process' task struct.
 *
 * @return  0                   No error.
 *          -ENOENT             No such file or directory.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -ENAMETOOLONG       File name too long.
 *          -EBADE              Invalid exchange.
 *          -ENODATA            No data available.
 *          -EMSGSIZE           Message too long.
 */
int xml_process_dump_msg(bide_reports_t *rep, struct task_struct *task)
{
	const struct cred *cred = NULL;
	int rc = 0;

	if (!rep || !task)
		return -EINVAL;

	/* Obtaining the task's cred is a RCU dereference, requires lock */
	rcu_read_lock();
	cred = __task_cred(task);

	rc = xml_write_kbide(rep, PROCESS_START_TAG, task->pid) |
	     xml_write_kbide(rep, ECRED_TAG, CRED_UID(cred, euid),
			     CRED_GID(cred, egid));
	     xml_write_kbide(rep, RCRED_TAG, CRED_UID(cred, uid),
			     CRED_GID(cred, gid));
	if (rc) {
		logError("xml_process_dump_msg(): failed on xml_write_kbide() "
			 "for start tag | cred. rc=%d.", rc);
		rcu_read_unlock();
		return rc;
	}

	rc = xml_process_groups_msg(rep, cred);
	rcu_read_unlock();
	if (rc) {
		logError("xml_process_dump_msg(): failed on "
			 "xml_process_groups_msg(). rc=%d.", rc);
		return rc;
	}

	/* Command line parameters */
	rc = xml_process_cmdline_msg(rep, task);
	if (rc) {
		logError("xml_process_dump_msg(): failed on "
			 "xml_process_cmdline_msg(). rc=%d.", rc);
		return rc;
	}

	/* Obtaining the task's cred is a RCU dereference, requires lock */
	rcu_read_lock();
	cred = __task_cred(task);

	/* Capabilities */
	rc = xml_process_capset_msg(rep,
				    &cred->cap_effective,
				    &cred->cap_inheritable,
				    &cred->cap_permitted,
				    CAP_CURRENT);

	rcu_read_unlock();

	if (rc) {
		logError("xml_process_dump_msg(): failed on "
			 "xml_process_capset_msg(). rc=%d.", rc);
		return rc;
	}

	rc = xml_process_lineage_msg(rep, task);
	if (rc) {
		logError("xml_process_dump_msg(): failed on "
			 "xml_process_lineage_msg(). rc=%d.", rc);
		return rc;
	}

	rc = xml_write_kbide(rep, PROCESS_END_TAG);
	if (rc)
		logError("xml_process_dump_msg(): failed on "
			 "xml_write_kbide() for end tag. rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function generates a message for the jbide sensors triggered.
 *
 * @param   rep                 The report structure.
 * @param   msg                 List of the jbide sensors triggered
 *
 * @return  0                   No error.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -EBADE              Invalid exchange.
 *          -EMSGSIZE           Message too long.
 */
int xml_jbide_sensor_msg(bide_reports_t *rep, char *msg)
{
	int rc = 0;

	if (!rep || !msg)
		return -EINVAL;

	/* Generate the XML message */
	rc = xml_write_kbide(rep, JBIDE_SENSORS_TAG, msg);
	if (rc)
		logError("xml_jbide_sensor_msg(): failed on "
			 "xml_write_kbide(). rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function generates a message for the kbide sensors triggered.
 *
 * @param   rep                 The report structure.
 * @param   msg                 List of the kbide sensors triggered
 *
 * @return  0                   No error.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -EBADE              Invalid exchange.
 *          -EMSGSIZE           Message too long.
 */
int xml_kbide_sensor_msg(bide_reports_t *rep, char *msg)
{
	int rc = 0;

	if (!rep || !msg)
		return -EINVAL;

	/* Generate the XML message */
	rc = xml_write_kbide(rep, KBIDE_SENSORS_TAG, msg);
	if (rc)
		logError("xml_kbide_sensor_msg(): failed on "
			 "xml_write_kbide(). rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function begins an xml section to the report for the specific
 * failure. The buffer provided must be initialized and must be large
 * enough to hold the report.
 *
 * @param   rep                 The report structure.
 * @param   severity            The severity (See: SEVERITY_* macros).
 * @param   sensor              The failed sensor (See: SN_* macros).
 *
 * @return  0                   No error.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -EBADE              Invalid exchange.
 *          -EMSGSIZE           Message too long.
 */
int xml_begin_incident_report(bide_reports_t *rep,
			      int severity,
			      const char *sensor)
{
	int rc = 0;

	if (!rep)
		return -EINVAL;

	/* Add the beginning of the report */
	rc = xml_write_kbide(rep, INCIDENT_START_TAG, sensor, severity);
	if (rc)
		logError("xml_begin_incident_report(): failed on "
			 "xml_write_kbide(). rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function finalizes the incident report. Buffer must be large
 * enough to concatenate the ending tag.
 *
 * @param   rep                 The report structure.
 *
 * @return  0                   No error.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -EBADE              Invalid exchange.
 *          -EMSGSIZE           Message too long.
 */
int xml_end_incident_report(bide_reports_t *rep)
{
	int rc = 0;

	if (!rep)
		return -EINVAL;

	/* Finish the incident report */
	rc = xml_write_kbide(rep, INCIDENT_END_TAG);
	if (rc)
		logError("xml_end_incident_report(): failed on "
			 "xml_write_kbide(). rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function begins a KBIDE report xml. The buffer must be at least
 * big enough to accomodate the starting tags.
 *
 * @param   rep                 The report structure.
 *
 * @return  0                   No error.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -EBADE              Invalid exchange.
 *          -EMSGSIZE           Message too long.
 */
int xml_begin_report(bide_reports_t *rep)
{
	struct timespec tv = {};
	int rc = 0;

	if (!rep)
		return -EINVAL;

	/* Get current time in seconds since epoch */
	getnstimeofday(&tv);

	/* Start the <kbide> report */
	rc = xml_write_kbide(rep, REPORT_START_TAG) |
	     xml_write_kbide(rep, VERSION_TAGS) |
	     xml_write_kbide(rep, TIME_TAGS, tv.tv_sec) |
	     xml_write_kbide(rep, FINGERPRINT_TAG, util_get_build_fingerprint());
	if (rc)
		logError("xml_begin_report(): failed on "
			 "xml_write_kbide(). rc=%d.", rc);

	return rc;
}

/*************************************************************************/

/*
 * This function writes the ending tag to finish the report xml structure.
 * The ending tag is concatenated to the report.
 *
 * @param   rep                 The report structure.
 *
 * @return  0                   No error.
 *          -ENOMEM             Out of memory.
 *          -EINVAL             Invalid argument.
 *          -EBADE              Invalid exchange.
 *          -EMSGSIZE           Message too long.
 */
int xml_end_report(bide_reports_t *rep)
{
	int rc = 0;

	if (!rep)
		return -EINVAL;

	/* Write the ending </kbide> tag */
	rc = xml_write_kbide(rep, REPORT_END_TAG);
	if (rc)
		logError("xml_end_report(): failed on "
			 "xml_write_kbide(). rc=%d.", rc);

	return rc;
}
