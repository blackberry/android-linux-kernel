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
#include <linux/uaccess.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"

/*************************************************************************/

#define TAG_SIZE	sizeof(uint32_t)
#define LENGTH_SIZE	sizeof(uint32_t)
#define DATA_OFFSET	TAG_SIZE + LENGTH_SIZE

/*************************************************************************/

/*
 * This function adds the specified unit32 value to the TLV in Big Endian
 * format.
 *
 * @param   buf              The buffer to write to.
 * @param   sz               Size of the buffer.
 * @param   x                The value to write.
 *
 * @return  0+               The amount of bytes written.
 *          -EINVAL          Invalid parameters given.
 *          -EMSGSIZE        Not enough buffer space.
 */
static int tlv_write_uint32(uint8_t *buf,
			    unsigned sz,
			    uint32_t x)
{
	if (!buf)
		return -EINVAL;

	if (sz < sizeof(uint32_t))
		return -EMSGSIZE;

	buf[0] = (uint8_t)(x >> 24);
	buf[1] = (uint8_t)(x >> 16);
	buf[2] = (uint8_t)(x >> 8);
	buf[3] = (uint8_t)(x);

	return sizeof(uint32_t);
}

/*************************************************************************/

/*
 * This function returns the total size of a TLV whose value is the
 * specified size.
 *
 * @param   value_sz            Size of the value part of the TLV.
 *
 * @return  +                   Total size of TLV.
 */
unsigned tlv_size(unsigned value_sz)
{
	return TAG_SIZE + LENGTH_SIZE + value_sz;
}

/*************************************************************************/

/*
 * This function adds a TLV to the queue with the specified type, length,
 * and value.
 *
 * @param   buf                 The buffer to write the TLV to.
 * @param   sz                  The size of the buffer.
 * @param   tag                 The TLV tag.
 * @param   value               Pointer to the value bytes.
 * @param   val_sz              The size of the value in bytes.
 *
 * @return  0+                  The number of bytes written.
 *          -EINVAL             Invalid parameters.
 *          -EMSGSIZE           TLV is too big for the provided buffer.
 */
int tlv_write_value(void *buf,
		    unsigned sz,
		    uint32_t tag,
		    const uint8_t *value,
		    unsigned val_sz)
{
	int rc = 0;

	if (!buf || !value)
		return -EINVAL;

	if (sz < tlv_size(val_sz))
		return -EMSGSIZE;

	/* Write the tag */
	rc = tlv_write_uint32(buf, sz - rc, tag);
	if (rc < 0)
		return -rc;

	/* Write the length */
	rc = tlv_write_uint32(buf + TAG_SIZE, sz - rc, val_sz);
	if (rc < 0)
		return -rc;

	if (val_sz && value != buf + DATA_OFFSET) {
		/* Write in the value */
		memcpy(buf + DATA_OFFSET, value, val_sz);
	}

	return tlv_size(val_sz);
}
