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

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"

/*************************************************************************/

struct hash_node {
	struct rb_node node;		/* Node within the RB tree */
	unsigned hash;			/* This node's hash value */
	void *data;			/* User allocated data */
};

/*************************************************************************/

/*
 * Pseudo-randomized transform: http://www.isthe.com/chongo/tech/comp/fnv/
 */
#define HASH_RANDOM_TRANSFORM 2166136261U

/*************************************************************************/

/*
 * A fast hashing mechanism for key creation based on the Jenkins One-at-
 * a-time algorithm. Source: http://www.burtleburtle.net/bob/hash/doobs.html
 *
 * @param   data            Blob of data to hash.
 * @param   sz              The size of the blob.
 *
 * @return                  The resulting hash of the blob.
 */
static unsigned hash_func(const char *data,
			  unsigned sz)
{
	if (sz && data) {
		uint32_t hash = HASH_RANDOM_TRANSFORM;
		uint32_t i = 0;

		for (i = 0; i < sz; ++i) {
			hash += (unsigned) data[i];
			hash += (hash << 10);
			hash ^= (hash >> 6);
		}

		hash += (hash << 3);
		hash ^= (hash >> 11);
		hash += (hash << 15);

		return hash;
	}

	return 0;
}

/*************************************************************************/

/*
 * RB tree traversal and lookup based on key hash.
 *
 * @param   n               A pointer to a root node.
 * @param   hash            A hashed value of the key to search for.
 * @param   parent          Pointer to the parent node, if found.
 *
 * @return                  A pointer to user data, or NULL if not found.
 */
static struct hash_node *find_node(struct rb_node *n,
				   unsigned hash,
				   struct rb_node **parent)
{
	struct hash_node *t = NULL;

	/* Set the parent to NULL in case the top node is chosen */
	if (parent)
		*parent = NULL;

	while (n) {
		/* Determine offset to the start of the hash_node struct */
		t = rb_entry((void *) n, struct hash_node, node);

		/* Traverse the RB tree, matching hash values */
		if (hash < t->hash) {
			if (parent)
				*parent = n;
			n = n->rb_left;
		} else if (hash > t->hash) {
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
	unsigned key_hash = 0;

	if (!root)
		return -EINVAL;

	/* Obtain a hash of the key */
	key_hash = hash_func(key, key_sz);
	if (!key_hash)
		return -EINVAL;

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
 *          -EINVAL         Invalid parameters.
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
	unsigned key_hash = 0;

	/* Check that the root node is valid */
	if (!root)
		return -EINVAL;

	/* Validate that the hash is valid */
	key_hash = hash_func(key, key_sz);
	if (!key_hash)
		return -EINVAL;

	/* Attempt to look up the key */
	node = find_node(root->rb_node, key_hash, &parent);
	if (node)
		return -EEXIST;

	/* Node does not exist with current hash, create a new node */
	node = (struct hash_node *) kzalloc(sizeof(struct hash_node), GFP_KERNEL);
	if (!node) {
		logError("Failed to allocate hash_node.");
		return -ENOMEM;
	}

	node->hash = key_hash;
	node->data = data;

	/* Find the destination pointer for the new hash node */
	if (!parent)
		dest = &root->rb_node;
	else {
		pnode = rb_entry((void *) parent, struct hash_node, node);
		dest = pnode->hash < key_hash ? &parent->rb_right : &parent->rb_left;
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
	unsigned key_hash = 0;

	/* Sanity check on inputs */
	if (!root || !key || !out)
		return -EINVAL;

	/* Derive a hash from the key */
	key_hash = hash_func(key, key_sz);
	if (!key_hash)
		return -EINVAL;

	/* Attempt to look up the key */
	node = find_node(root->rb_node, key_hash, NULL);
	if (!node)
		return -ENOENT;

	/* Return the data to caller for deallocation */
	*out = node->data;

	rb_erase(&node->node, root);
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

