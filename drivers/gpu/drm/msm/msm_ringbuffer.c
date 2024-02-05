/*
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "msm_ringbuffer.h"
#include "msm_gpu.h"

struct msm_ringbuffer *msm_ringbuffer_new(struct msm_gpu *gpu, int id,
		struct msm_memptrs *memptrs, uint64_t memptrs_iova)
{
	struct msm_ringbuffer *ring;
	int ret;

	/* We assume everwhere that MSM_GPU_RINGBUFFER_SZ is a power of 2 */
	BUILD_BUG_ON(!is_power_of_2(MSM_GPU_RINGBUFFER_SZ));

	ring = kzalloc(sizeof(*ring), GFP_KERNEL);
	if (!ring) {
		ret = -ENOMEM;
		goto fail;
	}

	ring->gpu = gpu;
	ring->id = id;
	ring->bo = msm_gem_new(gpu->dev, MSM_GPU_RINGBUFFER_SZ,
			MSM_BO_WC);
	if (IS_ERR(ring->bo)) {
		ret = PTR_ERR(ring->bo);
		ring->bo = NULL;
		goto fail;
	}

	ring->memptrs = memptrs;
	ring->memptrs_iova = memptrs_iova;


	ring->start = msm_gem_vaddr(ring->bo);
	ring->end   = ring->start + (MSM_GPU_RINGBUFFER_SZ >> 2);
	ring->next  = ring->start;
	ring->cur   = ring->start;

	INIT_LIST_HEAD(&ring->submits);
	spin_lock_init(&ring->lock);

	return ring;

fail:
	if (ring)
		msm_ringbuffer_destroy(ring);
	return ERR_PTR(ret);
}

void msm_ringbuffer_destroy(struct msm_ringbuffer *ring)
{
	if (ring && ring->bo) {
		msm_gem_put_iova(ring->bo, ring->gpu->aspace);
		drm_gem_object_unreference_unlocked(ring->bo);
	}

	kfree(ring);
}
