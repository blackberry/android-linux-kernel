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

#ifndef BBRY_POLICY_H
#define BBRY_POLICY_H

#include "sdcardfs.h"

#ifdef CONFIG_BBSECURE_SDAFW
/* Max users to which these policies affect, usually personal + work */
#define MAX_USERS_POLICY 2
#endif /* CONFIG_BBSECURE_SDAFW */

int bbry_policy_init(void);
void bbry_policy_exit(void);
int get_managed_profile(userid_t *);

#ifdef CONFIG_BBSECURE_SDAFW
int get_mediacard_disabled(userid_t *, size_t);
int get_usbotg_disabled(userid_t *, size_t);

enum storage_t {INVALID_STORE, EMU_STORE, SD_STORE, OTG_STORE};
#endif /* CONFIG_BBSECURE_SDAFW */

#ifdef CONFIG_BBSECURE_ADBAFW
int get_adb_disabled(userid_t);
#endif

#endif /* BBRY_POLICY_H */
