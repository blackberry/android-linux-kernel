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
#include <linux/debugfs.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"

/*************************************************************************/

static struct dentry *bide_dbfs_dentry, *bide_auth_dentry;

static const struct file_operations auth_debug_op = {
        .open = auth_debugfs_open,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = single_release,
};

/*************************************************************************/

/*
 * Entry point for initializing bide debugfs directory and files
 *
 * @return  0                No Error (Always).
 */
int __init debugfs_init(void)
{
        bide_dbfs_dentry = debugfs_create_dir("bide", NULL);

	/* debugfs_create_dir either return pointers, NULL or -%ENODEV */
	if(IS_ERR_OR_NULL(bide_dbfs_dentry)) {
		/* Not a major concern to BIDE if debugfs is not available */
		return 0;
        }

	/* Creating READONLY debugfs files */
	bide_auth_dentry = debugfs_create_file("auth", 0400, bide_dbfs_dentry,
					       NULL, &auth_debug_op);

	if(IS_ERR_OR_NULL(bide_auth_dentry)) {
		logError("Failed to create debugfs file");
		return 0;
	}

	/* Hold the mutex just to be safe */
	mutex_lock(&bide_auth_dentry->d_inode->i_mutex);

	/* Update Owner and Group ID so Jbide can read the file */
	bide_auth_dentry->d_inode->i_uid.val = BIDE_UID;
	bide_auth_dentry->d_inode->i_gid.val = BIDE_UID;

	mutex_unlock(&bide_auth_dentry->d_inode->i_mutex);

        return 0;
}

/*************************************************************************/

/*
 * Deinitialization of Bide Debugfs directory and files
 *
 * @return  0                No Error (Always).
 */
int __exit debugfs_exit(void)
{
        debugfs_remove(bide_auth_dentry);
        debugfs_remove(bide_dbfs_dentry);
        return 0;
}
