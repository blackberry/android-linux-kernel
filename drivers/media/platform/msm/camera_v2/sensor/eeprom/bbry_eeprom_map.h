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
#ifndef BBRY_EEPROM_MAP_H
#define BBRY_EEPROM_MAP_H

int bbry_read_eeprom_memory(struct msm_eeprom_ctrl_t *e_ctrl,
					struct msm_eeprom_memory_block_t *block);

int bbry_eeprom_parse_memory_map(struct msm_eeprom_ctrl_t *e_ctrl,
					struct msm_eeprom_memory_block_t *data);

#endif
