/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
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
#ifndef _SENSOR_CFG_H_
#define _SENSOR_CFG_H_

#include "sensor_drv_u.h"

#if 0
struct sensor_drv_cfg {
	struct list_head list;
	uint32_t sensor_pos;
	const char *sensor_name;
	SENSOR_INFO_T *driver_info;
};
int dcam_register_sensor_drv(struct sensor_drv_cfg *cfg);
struct list_head *Sensor_GetList(SENSOR_ID_E sensor_id);

#endif

SENSOR_INFO_T ** Sensor_GetInforTab(SENSOR_ID_E sensor_id);
uint32_t Sensor_GetInforTabLenght(SENSOR_ID_E sensor_id);


#endif
