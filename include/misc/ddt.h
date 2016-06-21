/*
 * Copyright (C) 2015 BlackBerry Limited
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

#include <linux/kmod.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <misc/lw_event_types.h>

/**
 * Logworthy Event Details
 */
struct logworthy_event_details_t {
	uint32_t d1;      /* Details Field 1 */
	uint32_t d2;      /* Details Field 2 */
	uint32_t d3;      /* Details Field 3 */
	uint32_t d4;      /* Details Field 4 */
	char *creator_id; /* Creator id Field */
};

#define BUFFER_SIZE 512
static const char * const tool = "/system/bin/ds_launcher send";

static void argv_cleanup(struct subprocess_info *info)
{
	argv_free(info->argv);
}

static int ddt_send(int etype, struct logworthy_event_details_t *d,
		const char *cmds)
{
	struct subprocess_info *info;
	char **argv = NULL;
	char *tmp = NULL;
	char cmd_buffer[BUFFER_SIZE];
	static char *envp[] = {
		"HOME=/",
		"TERM=linux",
		"PATH=/system/bin",
		NULL
	};
	int ret = -ENOMEM;
	int argc = 0;
	int len = 0;
	int cmd_len = strlen(cmds);

	/*
	 * The idea is to pass a tmp non-white space cmd string of
	 * equal length to input cmds, since argv_split white space
	 * delimits, and then replace the tmp string with our actual cmd
	 * string. This will allow parameters to be passed in the cmd.
	 */

	tmp = kzalloc(cmd_len + 1, GFP_KERNEL);
	if (tmp == NULL) {
		pr_err("%s: failed to allocate memory\n", __func__);
		goto out;
	}
	memset(tmp, 'b', cmd_len);

	len = snprintf(cmd_buffer, BUFFER_SIZE, "%s %d %d %d %d %d %s %s",
		tool, etype, d->d1, d->d2, d->d3, d->d4, d->creator_id, tmp);
	if (len == 0) {
		pr_err("%s: failed to create command line\n", __func__);
		goto out;
	} else if (len >= BUFFER_SIZE)
		pr_warn("%s: command line truncated!\n", __func__);

	argv = argv_split(GFP_KERNEL, cmd_buffer, &argc);
	if (argv == NULL) {
		pr_err("%s: failed to parse command line\n", __func__);
		goto out;
	}

	strlcpy(argv[argc - 1], cmds, cmd_len + 1);

	pr_debug("%s: log gen args added: %s\n", __func__, argv[argc - 1]);
	pr_debug("%s: parsed command line: argc %d\n", __func__, argc);

	info = call_usermodehelper_setup(argv[0], argv, envp, GFP_KERNEL, NULL,
		argv_cleanup, NULL);

	if (info == NULL) {
		argv_free(argv);
		goto out;
	}

	pr_debug("%s: Start helper\n", __func__);
	ret = call_usermodehelper_exec(info, UMH_NO_WAIT);
	pr_debug("%s: helper finished %d\n", __func__, ret);

out:
	kfree(tmp);
	return ret;
}
