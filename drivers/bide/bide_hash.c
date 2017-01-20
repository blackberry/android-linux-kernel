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
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/rbtree.h>
#include <linux/err.h>
#include <linux/hardirq.h> /* for in_atomic() */

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"

/*************************************************************************/

struct hash_node {
	struct rb_node node;		/* Node within the RB tree */
	uint8_t *hash;			/* Pointer to node's hash value */
	void *data;			/* User allocated data */
};

/*************************************************************************/

/*
 * Using BIDE Wrapper function of Linux SHA-256 hash.
 *
 * @param   data            Blob of data to hash.
 * @param   sz              The size of the blob.
 * @param   hash            A given pointer to mem size of SHA-256 digest
 *
 * @return  0               No Error.
 *          -EINVAL         Invalid input, bad hash, fail hash.
 */
static int hash_func(const char *data,
		     unsigned sz,
		     uint8_t *hash)
{
	int rc = -EINVAL;
	if (sz && data && hash) {
		rc = crypto_once(HASH_ALG_SHA256, data, sz, hash, HASH_SHA256_SIZE);

		if (rc)
			logError("Failed on crypto_once(). rc=%d", rc);

		return rc;
	}
	return rc;
}

/*************************************************************************/

/*
 * RB tree traversal and lookup based on key hash.
 *
 * @param   n               A pointer to a root node.
 * @param   hash            A pointer to a hashed blob to search for.
 * @param   parent          Pointer to the parent node, if found.
 *
 * @return        A pointer to user data
 *                NULL if not found.
 */
static struct hash_node *find_node(struct rb_node *n,
				   const uint8_t *hash,
				   struct rb_node **parent)
{
	struct hash_node *t = NULL;
	int ret;

	/* Set the parent to NULL in case the top node is chosen */
	if (parent)
		*parent = NULL;

	while (n) {
		/* Determine offset to the start of the hash_node struct */
		t = rb_entry((void *) n, struct hash_node, node);

		/* Traverse the RB tree, matching hash values */
		ret = memcmp(hash, t->hash, HASH_SHA256_SIZE);

		if (ret < 0) {
			if (parent)
				*parent = n;
			n = n->rb_left;
		} else if (ret > 0) {
			if (parent)
				*parent = n;
			n = n->rb_right;
		} else {
			return t;
		}

	}

	/* Node was not found */
	return NULL;
}

/*************************************************************************/

/*
 * A search mechanism based on a hash of the key blob.
 *
 * @param   root            A pointer to the root of the RB tree.
 * @param   key             A pointer to the blob that represents the key.
 * @param   key_sz          Size of the key blob.
 * @param   out             Destination of data once found (optional).
 *
 * @return  0               No Error, value was found.
 *          -EINVAL         Bad inputs or bad hash.
 *          -ENOENT         Key was not found.
 */
int hash_search(struct rb_root *root,
		const char *key,
		unsigned key_sz,
		void **out)
{
	struct hash_node *node = NULL;
	uint8_t key_hash[HASH_SHA256_SIZE];

	if (!root || !key) {
		return -EINVAL;
	}

	/* Obtain a hash of the key */
	if (hash_func(key, key_sz, key_hash)) {
		return -EINVAL;
	}

	/* Find the node and return the data value */
	node = find_node(root->rb_node, key_hash, NULL);
	if (node) {
		if (out)
			*out = node->data;

		return 0;
	}

	/* Node not found */
	return -ENOENT;
}

/*************************************************************************/

/*
 * A function to insert a key-value pair into the hash map.
 *
 * @param   root            A pointer to the root of the RB tree.
 * @param   key             A pointer to the blob that represents the key.
 * @param   key_sz          Size of the key blob.
 * @param   data            The value of the key-value pair.
 *
 * @return  0               No Error.
 *          -EINVAL         Invalid parameters, bad hash or hash fail.
 *          -EEXIST         Key already exists.
 *          -ENOMEM         Memory allocation error.
 */
int hash_insert(struct rb_root *root,
		const char *key,
		unsigned key_sz,
		void *data)
{
	struct rb_node *parent = NULL;
	struct rb_node **dest = NULL;
	struct hash_node *node = NULL;
	struct hash_node *pnode = NULL;
	uint8_t *key_hash;
	int ret;
	int flag = in_atomic() ? GFP_ATOMIC : GFP_KERNEL;

	/* Check that the root node and data is valid */
	if ((!root) && (!data))
		return -EINVAL;

	key_hash = kzalloc(sizeof(uint8_t) * HASH_SHA256_SIZE, flag);
	if (!key_hash) {
		logError("Failed to allocate kmem for hash");
		return -EINVAL;
	}

	/* Validate that the hash is valid */
	if (hash_func(key, key_sz, key_hash)) {
		kfree(key_hash);
		return -EINVAL;
	}

	/* Attempt to look up the key */
	node = find_node(root->rb_node, key_hash, &parent);
	if (node) {
		kfree(key_hash);
		return -EEXIST;
	}

	/* Node does not exist with current hash, create a new node */
	node = (struct hash_node *) kzalloc(sizeof(struct hash_node), flag);
	if (!node) {
		logError("Failed to allocate hash_node.");
		kfree(key_hash);
		return -ENOMEM;
	}

	node->hash = key_hash;
	node->data = data;

	/* Find the destination pointer for the new hash node */
	if (!parent)
		dest = &root->rb_node;
	else {
		pnode = rb_entry((void *) parent, struct hash_node, node);

		ret = memcmp(pnode->hash, key_hash, HASH_SHA256_SIZE);
		dest = ret < 0 ? &parent->rb_right : &parent->rb_left;
	}

	rb_link_node(&node->node, parent, dest);
	rb_insert_color(&node->node, root);

	return 0;
}

/*************************************************************************/

/*
 * This function removes a key-value pair from the hash map. The data
 * value is returned through the out parameter and must be deallocated
 * by the caller.
 *
 * @param   root            A pointer to the root of the RB tree.
 * @param   key             A pointer to the blob that represents the key.
 * @param   key_sz          Size of the key blob.
 * @param   out             The old data to be deallocated by caller.
 *
 * @return  0               No Error.
 *          -EINVAL         Invalid parameters.
 *          -ENOENT         Key not found.
 */
int hash_remove(struct rb_root *root,
		const char *key,
		unsigned key_sz,
		void **out)
{
	struct hash_node *node = NULL;
	uint8_t key_hash[HASH_SHA256_SIZE];

	/* Sanity check on inputs */
	if (!root || !key || !out)
		return -EINVAL;

	/* Derive a hash from the key */
	if (hash_func(key, key_sz, key_hash)) {
		return -EINVAL;
	}

	/* Attempt to look up the key */
	node = find_node(root->rb_node, key_hash, NULL);
	if (!node) {
		return -ENOENT;
	}

	/* Return the data to caller for deallocation */
	*out = node->data;

	rb_erase(&node->node, root);
	kfree(node->hash);
	kfree(node);

	return 0;
}

/*************************************************************************/

/*
 * A function for obtain the value of a given key-value pair.
 *
 * @param   root            A pointer to the root of the RB tree.
 * @param   out             The output parameter to receive the data pointer.
 *
 * @return  0               No Error.
 *          -EINVAL         Invalid parameters.
 */
int hash_get(struct rb_node *n,
	     void **out)
{
	struct hash_node *t = NULL;

	if (!n || !out)
		return -EINVAL;

	/* Determine offset to the hash_node struct and assign return value */
	t = rb_entry((void *) n, struct hash_node, node);
	*out = t->data;

	return 0;
}
