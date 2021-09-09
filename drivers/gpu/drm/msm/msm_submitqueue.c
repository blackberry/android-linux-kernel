/* Copyright (c) 2017 The Linux Foundation. All rights reserved.
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

#include <linux/kref.h>
#include "msm_gpu.h"

void msm_submitqueue_destroy(struct kref *kref)
{
	struct msm_gpu_submitqueue *queue = container_of(kref,
		struct msm_gpu_submitqueue, ref);

	kfree(queue);
}

struct msm_gpu_submitqueue *msm_submitqueue_get(struct msm_file_private *ctx,
		u32 id)
{
	struct msm_gpu_submitqueue *entry;

	if (!ctx)
		return NULL;

	read_lock(&ctx->queuelock);

	list_for_each_entry(entry, &ctx->submitqueues, node) {
		if (entry->id == id) {
			kref_get(&entry->ref);
			read_unlock(&ctx->queuelock);

			return entry;
		}
	}

	read_unlock(&ctx->queuelock);
	return NULL;
}

void msm_submitqueue_close(struct msm_file_private *ctx)
{
	struct msm_gpu_submitqueue *entry, *tmp;

	/*
	 * No lock needed in close and there won't
	 * be any more user ioctls coming our way
	 */

	list_for_each_entry_safe(entry, tmp, &ctx->submitqueues, node)
		msm_submitqueue_put(entry);
}

int msm_submitqueue_create(struct msm_file_private *ctx, u32 prio, u32 flags,
		u32 *id)
{
	struct msm_gpu_submitqueue *queue = kzalloc(sizeof(*queue), GFP_KERNEL);

	if (!queue)
		return -ENOMEM;

	kref_init(&queue->ref);
	queue->flags = flags;
	queue->prio = prio;

	write_lock(&ctx->queuelock);

	queue->id = ctx->queueid++;

	if (id)
		*id = queue->id;

	list_add_tail(&queue->node, &ctx->submitqueues);

	write_unlock(&ctx->queuelock);

	return 0;
}

int msm_submitqueue_init(struct msm_file_private *ctx)
{
	INIT_LIST_HEAD(&ctx->submitqueues);

	rwlock_init(&ctx->queuelock);

	/*
	 * Add the "default" submitqueue with id 0
	 * "low" priority (2) and no flags
	 */

	return msm_submitqueue_create(ctx, 2, 0, NULL);
}

int msm_submitqueue_query(struct msm_file_private *ctx, u32 id, u32 param,
		void __user *data, u32 len)
{
	struct msm_gpu_submitqueue *queue = msm_submitqueue_get(ctx, id);
	int ret = 0;

	if (!queue)
		return -ENOENT;

	if (param == MSM_SUBMITQUEUE_PARAM_FAULTS) {
		u32 size = min_t(u32, len, sizeof(queue->faults));

		if (copy_to_user(data, &queue->faults, size))
			ret = -EFAULT;
	} else {
		ret = -EINVAL;
	}

	msm_submitqueue_put(queue);

	return ret;
}

int msm_submitqueue_remove(struct msm_file_private *ctx, u32 id)
{
	struct msm_gpu_submitqueue *entry;

	/*
	 * id 0 is the "default" queue and can't be destroyed
	 * by the user
	 */

	if (!id)
		return -ENOENT;

	write_lock(&ctx->queuelock);

	list_for_each_entry(entry, &ctx->submitqueues, node) {
		if (entry->id == id) {
			list_del(&entry->node);
			write_unlock(&ctx->queuelock);

			msm_submitqueue_put(entry);
			return 0;
		}
	}

	write_unlock(&ctx->queuelock);
	return -ENOENT;
}

