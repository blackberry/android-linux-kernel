/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "fuse_shortcircuit.h"

#include <linux/aio.h>
#include <linux/fs_stack.h>
#include <linux/fsnotify.h>// MODIFIED by wu.yan, 2016-03-23,BUG-1488328

void fuse_setup_shortcircuit(struct fuse_conn *fc, struct fuse_req *req)
{
	int daemon_fd;
	struct file *rw_lower_file = NULL;
	struct fuse_open_out *open_out;
	int open_out_index;

	req->private_lower_rw_file = NULL;

	if (!(fc->shortcircuit_io))
		return;

	if ((req->in.h.opcode != FUSE_OPEN) &&
	    /* MODIFIED-BEGIN by wu.yan, 2016-03-23,BUG-1488328 */
	    (req->in.h.opcode != FUSE_CREATE) &&
	    (req->in.h.opcode != FUSE_OPENDIR))
	    /* MODIFIED-END by wu.yan,BUG-1488328 */
		return;

	open_out_index = req->in.numargs - 1;

	BUG_ON(open_out_index != 0 && open_out_index != 1);
	BUG_ON(req->out.args[open_out_index].size != sizeof(*open_out));

	open_out = req->out.args[open_out_index].value;

	daemon_fd = (int)open_out->lower_fd;
	if (daemon_fd < 0)
		return;

	rw_lower_file = fget_raw(daemon_fd);
	if (!rw_lower_file)
		return;
	req->private_lower_rw_file = rw_lower_file;
}

static ssize_t fuse_shortcircuit_read_write_iter(struct kiocb *iocb,
						 struct iov_iter *iter,
						 int do_write)
{
	ssize_t ret_val;
	struct fuse_file *ff;
	struct file *fuse_file, *lower_file;
	struct inode *fuse_inode, *lower_inode;

	ff = iocb->ki_filp->private_data;
	fuse_file = iocb->ki_filp;
	lower_file = ff->rw_lower_file;

	/* lock lower file to prevent it from being released */
	get_file(lower_file);
	iocb->ki_filp = lower_file;
	fuse_inode = fuse_file->f_path.dentry->d_inode;
	lower_inode = file_inode(lower_file);

	if (do_write) {
		if (!lower_file->f_op->write_iter)
			return -EIO;
		ret_val = lower_file->f_op->write_iter(iocb, iter);

		if (ret_val >= 0 || ret_val == -EIOCBQUEUED) {
			fsstack_copy_inode_size(fuse_inode, lower_inode);
			fsstack_copy_attr_times(fuse_inode, lower_inode);
		}
	} else {
		if (!lower_file->f_op->read_iter)
			return -EIO;
		ret_val = lower_file->f_op->read_iter(iocb, iter);
		if (ret_val >= 0 || ret_val == -EIOCBQUEUED)
			fsstack_copy_attr_atime(fuse_inode, lower_inode);
	}

	iocb->ki_filp = fuse_file;
	fput(lower_file);
	/* unlock lower file */

	return ret_val;
}

ssize_t fuse_shortcircuit_read_iter(struct kiocb *iocb, struct iov_iter *to)
{
	return fuse_shortcircuit_read_write_iter(iocb, to, 0);
}

ssize_t fuse_shortcircuit_write_iter(struct kiocb *iocb, struct iov_iter *from)
{
	return fuse_shortcircuit_read_write_iter(iocb, from, 1);
}

/* MODIFIED-BEGIN by wu.yan, 2016-03-23,BUG-1488328 */
/* same as iterate_dir except skipping security check */
static int fuse_iterate_dir(struct file *file, struct dir_context *ctx)
{
	struct inode *inode = file_inode(file);
	int res = -ENOTDIR;
	if (!file->f_op->iterate)
		goto out;

	res = mutex_lock_killable(&inode->i_mutex);
	if (res)
		goto out;

	res = -ENOENT;
	if (!IS_DEADDIR(inode)) {
		ctx->pos = file->f_pos;
		res = file->f_op->iterate(file, ctx);
		file->f_pos = ctx->pos;
		fsnotify_access(file);
		file_accessed(file);
	}
	mutex_unlock(&inode->i_mutex);
out:
	return res;
}

int fuse_shortcircuit_readdir(struct file *file, struct dir_context *ctx)
{
	int err;
	struct fuse_file *ff = file->private_data;
	struct file *lower_file = ff->rw_lower_file;
	struct dentry *dentry = file->f_path.dentry;

	err = fuse_iterate_dir(lower_file, ctx);
	file->f_pos = lower_file->f_pos;
	if (err > 0)
		fsstack_copy_attr_atime(dentry->d_inode,
				file_inode(lower_file));
	return err;
}
/* MODIFIED-END by wu.yan,BUG-1488328 */

void fuse_shortcircuit_release(struct fuse_file *ff)
{
	if (!(ff->rw_lower_file))
		return;

	/* Release the lower file. */
	fput(ff->rw_lower_file);
	ff->rw_lower_file = NULL;
}
