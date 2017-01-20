/*
 * Copyright (C) 2016 BlackBerry Limited
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

#ifdef BID_USER_DEBUG

/* Kernel Includes */
#include <linux/atomic.h>

/* Local Includes */
#include "bide.h"
#include "bide_internal.h"
#include "bide_log.h"

/*************************************************************************/

static atomic_t stat_fct_call_count[stat_last_element];

/*************************************************************************/

/*
 * This function is called to increment the count. This function should
 * be called everytime the function monitored is called.
 *
 * @param   fct_id              Id of the function defined in bide_internal.h
 */
void stat_increment(int fct_id)
{
	atomic_inc(&stat_fct_call_count[fct_id]);
}

/*************************************************************************/

/*
 * This function is called display the current count of all the functions
 * monitored.
 */
void stat_display(void)
{
	int i;
	logInfo("---- Function Count ----");
	for(i = 0; i < stat_last_element; i++) {
		logInfo("Function \"%s\" was called %d times", stat_fct_name[i], atomic_read(&stat_fct_call_count[i]));
	}
	logInfo("------------------------");
}

/*************************************************************************/

/*
 * Entry point into the module. Initialize the count to 0 for all the
 * functions monitored.
 *
 * @return  0                   Always
 */
int __init stat_init(void)
{
	int i;
	for(i = 0; i < stat_last_element; i++) {
		atomic_set(&stat_fct_call_count[i], 0);
	}
	return 0;
}

#endif
