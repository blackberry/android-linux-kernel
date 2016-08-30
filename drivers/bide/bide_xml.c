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
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/time.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"

/*************************************************************************/

static const char CAPABILITY_TEMPLATE[] =
	"<cap_%s>"
	  "<effective>0x%08x</effective>"
	  "<inheritable>0x%08x</inheritable>"
	  "<permitted>0x%08x</permitted>"
	"</cap_%s>";


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
static const char LINEAGE_START_TAG[]	= "<lineage>";
static const char LINEAGE_END_TAG[]	= "</lineage>";

/* Space for 64 bit long with a null terminator */
#define MAX_TIMESTAMP_LENGTH 	21
#define MAX_PATH 		256

/*************************************************************************/

struct xml_ctx {
	char *buf;
	unsigned sz;
	unsigned off;
};

/* Context Helper Macros */
#define XML_INIT(b, s)	{ .buf = b, .sz  = s, .off = b ? strnlen(b, s) : 0 }
#define XML_SZ(ctx)	((ctx).off >= (ctx).sz ? 0 : (ctx).sz - (ctx).off)
#define XML_BUF(ctx)	((ctx).buf + (ctx).off)

/*************************************************************************/

/*
 * This function is a wrapper around snprintf that checks the boundaries
 * of the buffer being written to and updates offsets.
 *
 * @param   ctx           The xml context buffer.
 * @param   fmt           The tag format of the string.
 * @param   ...           Various other parameters
 *
 * @return  0+            Number of bytes written.
 *          -EINVAL       Invalid parameters.
 *          -EMSGSIZE     Message is too large for buffer.
 */
static __printf(2, 3) int xml_write_tag(struct xml_ctx *ctx,
					const char *fmt,
					...)
{
	va_list args;
	int rc = 0;

	if (!ctx || !fmt)
		return -EINVAL;

	if (ctx->off >= ctx->sz)
		return -EMSGSIZE;

	va_start(args, fmt);
	rc = vsnprintf(ctx->buf + ctx->off, ctx->sz - ctx->off, fmt, args);
	va_end(args);

	if (rc < 0 || rc + ctx->off >= ctx->sz)
		return -EMSGSIZE;

	/* Update offset with size of buffer written */
	ctx->off += rc;

	return rc;
}

/*************************************************************************/

/*
 * This function adds the JBIDE hash into the message buffer.
 *
 * @param   buf           The buffer that will be written to (out).
 * @param   sz            Size of the buffer.
 * @param   jhash         The base64 encoded hash of the JBIDE report.
 *
 * @return  0+            The amount of bytes written.
 *          -EINVAL       Invalid input parameters
 *          -EMSGSIZE     Message is too large for buffer.
 */
int xml_jbide_hash_msg(char *buf,
		       unsigned sz,
		       const char *jhash,
		       unsigned hash_sz)
{
	struct xml_ctx ctx = XML_INIT(buf, sz);
	int rc = 0;

	if (!buf || !sz || !jhash || hash_sz != BASE64_SHA256_SIZE)
		return -EINVAL;

	/* Generate the XML message */
	rc = xml_write_tag(&ctx, JBIDE_HASH_TAG, jhash);
	if (rc < 0)
		logError("Failed on xml_write_tag(). rc=%d.", -rc);

	return rc;
}

/*************************************************************************/

/*
 * This function creates a message for capabilities.
 *
 * @param   buf           The buffer that will be written to (out).
 * @param   buf_sz        Size of the buffer.
 * @param   eff           The effective capabilities being set.
 * @param   inh           The inheritable capabilities being set.
 * @param   per           The permitted capabilities being set.
 * @param   pid           The process ID of the offending process.
 * @param   ppid          The offending process's parent's PID
 * @param   uid           The UID of the process
 *
 * @return  0+            The amount of bytes written.
 *          -EINVAL       Invalid input parameters
 *          -EMSGSIZE     Message is too large for buffer.
 */
int xml_capset_msg(char *buf,
		unsigned sz,
		const kernel_cap_t *eff,
		const kernel_cap_t *inh,
		const kernel_cap_t *per,
		CAP_STATE state)
{
	struct xml_ctx ctx = XML_INIT(buf, sz);
	int caps[3] = { 0 };
	int rc = 0;
	int i = 0;

	if (!buf || !sz || !eff || !inh || !per)
		return -EINVAL;

	/* Compile which is the offending capability */
	for (i = 0; i < _KERNEL_CAPABILITY_U32S; ++i) {
		caps[0] |= eff->cap[i];
		caps[1] |= inh->cap[i];
		caps[2] |= per->cap[i];
	}

	/* Generate the XML message */
	rc = xml_write_tag(&ctx,
			   CAPABILITY_TEMPLATE,
			   state == CAP_NEW ? "new" : "current",
			   caps[0],
			   caps[1],
			   caps[2],
			   state == CAP_NEW ? "new" : "current");
	if (rc < 0)
		logError("Failed on xml_write_tag(). rc=%d.", -rc);

	return rc;
}

/*************************************************************************/


/*
 * This function generates a message for a process that has a mismatched
 * hash.
 *
 * @param   buf           The buffer that will be written to (out).
 * @param   sz            Size of the buffer.
 * @param   proc          The name of the process.
 * @param   seciton       The section that failed the hash check.
 *
 * @return  0+            The amount of bytes written.
 *          -EINVAL       Invalid input parameters
 *          -EMSGSIZE     Message is too large for buffer.
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
		logError("Failed on xml_write_tag(). rc=%d.", -rc);

	return rc;
}

/*************************************************************************/

/*
 * This function creates a message for properties.
 *
 * @param   buf           The buffer that will be written to (out).
 * @param   sz            Size of the buffer.
 * @param   name          The name of the property.
 * @param   value         The value of the property.
 *
 * @return  0+            The amount of bytes written.
 *          -EINVAL       Invalid input parameters
 *          -EMSGSIZE     Message is too large for buffer.
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
		logError("Failed on xml_write_tag(). rc=%d.", -rc);

	return rc;
}

/*************************************************************************/

/*
 * This function creates a message for a mount path.
 *
 * @param   buf           The buffer that will be written to (out).
 * @param   sz            Size of the buffer.
 * @param   path          The path of the mount.
 *
 * @return  0+            The amount of bytes written.
 *          -EINVAL       Invalid input parameters
 *          -EMSGSIZE     Message is too large for buffer.
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
		logError("Failed on xml_write_tag(). rc=%d.", -rc);

	return rc;
}

/*************************************************************************/

/*
 * This function creates a message for root processes.
 *
 * Note: caller must ensure rcu is locked for read
 *
 * @param   buf           The buffer that will be written to (out).
 * @param   sz            Size of the buffer.
 * @param   name          The name of root process.
 * @param   pid           The pid of the root process.
 * @param   ppid          The pid of the parent process.
 *
 * @return  0+            The amount of bytes written.
 *          -EINVAL       Invalid input parameters
 *          -EMSGSIZE     Message is too large for buffer.
 */
static int xml_process_msg(char *buf,
		    unsigned sz,
		    struct task_struct *task)
{
	struct xml_ctx ctx = XML_INIT(buf, sz);
	char name[MAX_PATH] = { 0 };
	int off = ctx.off;
	int ppid = 0;
	int pid = 0;
	int rc = 0;

	if (!buf || !sz || !task)
		return -EINVAL;

	pid = task_tgid_vnr(task);

	rc = util_get_task_path(task->mm, name, sizeof(name));
	if (rc < 0) {
		logError("Failed on util_get_task_path(). rc=%d.", -rc);
		return rc;
	}

	/* Get parent task - rcu already locked by caller */
	task = rcu_dereference(task->real_parent);

	if (task)
		ppid = task_tgid_vnr(task);

	/* Generate the XML message */
	rc = xml_write_tag(&ctx, PROCESS_SINGLE_TAG, pid, ppid, name);
	if (rc < 0)
		logError("Failed on xml_write_tag(). rc=%d.", -rc);

	return rc < 0 ? rc : ctx.off - off;
}

/*************************************************************************/

/*
 * This function lists the parent tree of a process in XML format.
 *
 * @param   buf           A buffer that will receive the XML report.
 * @param   sz            The size of the buffer.
 * @param   task          The process' task struct.
 *
 * @return  0+            The amount of bytes written.
 *          -EINVAL       Invalid parameters.
 *          -EMSGSIZE     Message is too large for buffer.
 */
int xml_process_lineage_msg(char *buf,
			    unsigned sz,
			    struct task_struct *task)
{
	struct xml_ctx ctx = XML_INIT(buf, sz);
	int off = ctx.off;
	int rc = 0;

	if (!buf || !task)
		return -EINVAL;

	/* Start <lineage> tag */
	rc = xml_write_tag(&ctx, LINEAGE_START_TAG);
	if (rc < 0) {
		logError("Failed on xml_write_tag(). rc=%d.", -rc);
		return rc;
	}

	rcu_read_lock();
	while (task && task->mm) {
		/* Generate XML about the process */
		rc = xml_process_msg(XML_BUF(ctx), XML_SZ(ctx), task);
		if (rc < 0) {
			rcu_read_unlock();
			logError("Failed on xml_process_msg(). rc=%d.", -rc);
			return rc;
		}

		ctx.off += rc;

		/* Get next parent task */
		task = rcu_dereference(task->real_parent);
	}
	rcu_read_unlock();

	/* Finish </lineage> tag */
	rc = xml_write_tag(&ctx, LINEAGE_END_TAG);
	if (rc < 0)
		logError("Failed on xml_write_tag(). rc=%d.", -rc);

	return rc < 0 ? rc : ctx.off - off;
}

/*************************************************************************/

/*
 * This function writes out the command line parameters that were used
 * when the task was executed. The first parameter is always the name of
 * the executable.
 *
 * @param   buf           A buffer that will receive the XML report.
 * @param   sz            The size of the buffer.
 * @param   task          The process' task struct.
 *
 * @return  0+            The amount of bytes written.
 *          -EINVAL       Invalid parameters.
 *          -EMSGSIZE     Message is too large for buffer.
 */
int xml_process_cmdline_msg(char *buf,
			    unsigned sz,
			    struct task_struct *task)
{
	struct xml_ctx ctx = XML_INIT(buf, sz);
	int off = ctx.off;
	int rc = 0;

	if (!buf || !task)
		return -EINVAL;

	rc = xml_write_tag(&ctx, CMD_LINE_START_TAG);
	if (rc < 0) {
		logError("Failed on xml_write_tag(). rc=%d.", -rc);
		return rc;
	}

	rc = util_get_task_cmdline(task, XML_BUF(ctx), XML_SZ(ctx));
	if (rc < 0) {
		logError("Failed on util_get_task_cmdline(). rc=%d.", -rc);
		return rc;
	}
	ctx.off += rc;

	rc = xml_write_tag(&ctx, CMD_LINE_END_TAG);
	if (rc < 0)
		logError("Failed on xml_write_tag(). rc=%d.", -rc);

	return rc < 0 ? rc : ctx.off - off;
};

/*************************************************************************/

/*
 * This funciton dumps relevant process information into XML format.
 *
 * @param   buf           A buffer that will receive the XML report.
 * @param   sz            The size of the buffer.
 * @param   task          The process' task struct.
 *
 * @return  0+            The amount of bytes written.
 *          -EINVAL       Invalid parameters.
 *          -EMSGSIZE     Message is too large for buffer.
 */
int xml_process_dump_msg(char *buf,
			 unsigned sz,
			 struct task_struct *task)
{
	struct xml_ctx ctx = XML_INIT(buf, sz);
	const struct cred *cred = NULL;
	int off = ctx.off;
	int rc = 0;

	if (!buf || !task)
		return -EINVAL;

	/* Obtaining the task's cred is a RCU dereference, requires lock */
	rcu_read_lock();
	cred = __task_cred(task);
	rcu_read_unlock();

	rc = xml_write_tag(&ctx, PROCESS_START_TAG, task->pid) |
	     xml_write_tag(&ctx, ECRED_TAG, cred->euid, cred->egid) |
	     xml_write_tag(&ctx, RCRED_TAG, cred->uid, cred->gid);
	if (rc < 0) {
		logError("Failed on xml_write_tag(). rc=%d.", -rc);
		return rc;
	}

	/* Command line parameters */
	rc = xml_process_cmdline_msg(XML_BUF(ctx), XML_SZ(ctx), task);
	if (rc < 0) {
		logError("Failed on xml_process_cmdline_msg(). rc=%d.", -rc);
		return rc;
	}

	ctx.off += rc;

	/* Capabilities */
	rc = xml_capset_msg(XML_BUF(ctx),
			    XML_SZ(ctx),
			    &cred->cap_effective,
			    &cred->cap_inheritable,
			    &cred->cap_permitted,
			    CAP_CURRENT);
	if (rc < 0) {
		logError("Failed on xml_capset_msg(). rc=%d.", -rc);
		return rc;
	}

	ctx.off += rc;

	rc = xml_process_lineage_msg(XML_BUF(ctx),
				     XML_SZ(ctx),
				     task);
	if (rc < 0) {
		logError("Failed on xml_process_lineage_msg(). rc=%d.", -rc);
		return rc;
	}

	ctx.off += rc;

	rc = xml_write_tag(&ctx, PROCESS_END_TAG);
	if (rc < 0)
		logError("Failed on xml_write_tag(). rc=%d.", -rc);

	return rc < 0 ? rc : ctx.off - off;
}

/*************************************************************************/

/*
 * This function begins an xml section to the report for the specific
 * failure. The buffer provided must be initialized and must be large
 * enough to hold the report.
 *
 * @param   buf           The buffer that will be written to (out).
 * @param   sz            Size of the buffer.
 * @param   severity      The severity (See: SEVERITY_* macros).
 * @param   sensor        The failed sensor (See: SN_* macros).
 *
 * @return  0+            The amount of bytes written.
 *          -EINVAL       Invalid input parameters
 *          -EMSGSIZE     Message is too large for buffer.
 */
int xml_begin_incident_report(char *buf,
			      unsigned sz,
			      int severity,
			      const char *sensor)
{
	struct xml_ctx ctx = XML_INIT(buf, sz);
	int rc = 0;

	if (!buf)
		return -EINVAL;

	/* Add the beginning of the report */
	rc = xml_write_tag(&ctx, INCIDENT_START_TAG, sensor, severity);
	if (rc < 0)
		logError("Failed on xml_write_tag(). rc=%d.", -rc);

	/* Return the amount of bytes written */
	return rc;
}

/*************************************************************************/

/*
 * This function finalizes the incident report. Buffer must be large
 * enough to concatenate the ending tag.
 *
 * @param   buf           The buffer that will be written to (out).
 * @param   sz            Size of the buffer.
 *
 * @return  0+            The amount of bytes written.
 *          -EINVAL       Invalid input parameters
 *          -EMSGSIZE     Message is too large for buffer.
 */
int xml_end_incident_report(char *buf,
			    unsigned sz)
{
	struct xml_ctx ctx = XML_INIT(buf, sz);
	int rc = 0;

	if (!buf)
		return -EINVAL;

	/* Finish the incident report */
	rc = xml_write_tag(&ctx, INCIDENT_END_TAG);
	if (rc < 0)
		logError("Failed on xml_write_tag(). rc=%d.", -rc);

	return rc;
}

/*************************************************************************/

/*
 * This function begins a KBIDE report xml. The buffer must be at least
 * big enough to accomodate the starting tags.
 *
 * @param   buf           The buffer that will be written to (out).
 * @param   sz            Size of the buffer.
 *
 * @return  0+            The amount of bytes written.
 *          -EINVAL       Invalid input parameters
 *          -EMSGSIZE     Message is too large for buffer.
 */
int xml_begin_report(char *buf,
		     unsigned sz)
{
	struct xml_ctx ctx = XML_INIT(buf, sz);
	struct timespec tv = {};
	int off = ctx.off;
	int rc = 0;

	if (!buf)
		return -EINVAL;

	/* Get current time in seconds since epoc */
	getnstimeofday(&tv);

	/* Start the <kbide> report */
	rc = xml_write_tag(&ctx, REPORT_START_TAG) |
	     xml_write_tag(&ctx, VERSION_TAGS) |
	     xml_write_tag(&ctx, TIME_TAGS, tv.tv_sec);
	if (rc < 0)
		logError("Failed on xml_write_tag(). rc=%d.", -rc);

	return rc < 0 ? rc : ctx.off - off;
}

/*************************************************************************/

/*
 * This function writes the ending tag to finish the report xml structure.
 * The ending tag is concatenated to the report.
 *
 * @param   buf           The buffer that will be written to (out).
 * @param   sz            Size of the buffer.
 *
 * @return  0+            The amount of bytes written.
 *          -EINVAL       Invalid input parameters
 *          -EMSGSIZE     Message is too large for buffer.
 */
int xml_end_report(char *buf,
		   unsigned sz)
{
	struct xml_ctx ctx = XML_INIT(buf, sz);
	int rc = 0;

	if (!buf)
		return -EINVAL;

	ctx.off = strnlen(buf, sz);

	/* Write the ending </kbide> tag */
	rc = xml_write_tag(&ctx, REPORT_END_TAG);
	if (rc < 0)
		logError("Failed on xml_write_tag(). rc=%d.", -rc);

	return rc;
}
