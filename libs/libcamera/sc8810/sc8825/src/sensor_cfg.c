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
//#include <linux/i2c.h>
//#include <linux/gpio.h>
//#include <linux/delay.h>
//#include <mach/hardware.h>
//#include <asm/io.h>
//#include <linux/list.h>
#include "sensor_drv_u.h"
#include "sensor_cfg.h"

/**---------------------------------------------------------------------------*
 **                         extend Variables and function                     *
 **---------------------------------------------------------------------------*/
extern SENSOR_INFO_T g_OV7675_yuv_info;
extern SENSOR_INFO_T g_OV7670_yuv_info;
extern SENSOR_INFO_T g_OV9655_yuv_info;
extern SENSOR_INFO_T g_OV2640_yuv_info;
extern SENSOR_INFO_T g_OV2655_yuv_info;
extern SENSOR_INFO_T g_GC0306_yuv_info;
extern SENSOR_INFO_T g_SIV100A_yuv_info;
extern SENSOR_INFO_T g_SIV100B_yuv_info;
extern SENSOR_INFO_T g_OV3640_yuv_info;
extern SENSOR_INFO_T g_mt9m112_yuv_info;
extern SENSOR_INFO_T g_OV9660_yuv_info;
extern SENSOR_INFO_T g_OV7690_yuv_info;
extern SENSOR_INFO_T g_OV7675_yuv_info;
extern SENSOR_INFO_T g_GT2005_yuv_info;
extern SENSOR_INFO_T g_GC0309_yuv_info;
extern SENSOR_INFO_T g_ov5640_yuv_info;
extern SENSOR_INFO_T g_ov5640_raw_info;
extern SENSOR_INFO_T g_OV7660_yuv_info;
extern SENSOR_INFO_T g_nmi600_yuv_info;//atv:nmi bonnie
extern SENSOR_INFO_T g_ov5640_mipi_yuv_info;
extern SENSOR_INFO_T g_ov5640_mipi_raw_info;
extern SENSOR_INFO_T g_s5k5ccgx_yuv_info_mipi;
extern SENSOR_INFO_T g_hi351_mipi_yuv_info;

/**---------------------------------------------------------------------------*
 **                         Constant Variables                                *
 **---------------------------------------------------------------------------*/
const SENSOR_INFO_T* main_sensor_infor_tab[]=
{
	&g_ov5640_mipi_yuv_info,
	&g_s5k5ccgx_yuv_info_mipi,
	&g_hi351_mipi_yuv_info,
	//&g_ov5640_mipi_raw_info,
	//&g_ov5640_yuv_info,
	//&g_OV7675_yuv_info,
	//&g_OV2655_yuv_info,
	//&g_OV7675_yuv_info,
	//&g_OV2640_yuv_info,
	PNULL
};

const SENSOR_INFO_T* sub_sensor_infor_tab[]=
{
	&g_OV7675_yuv_info,
	//&g_GC0309_yuv_info,
	//g_OV7690_yuv_info,
	PNULL
};


const SENSOR_INFO_T* atv_infor_tab[]=
{
    //&g_nmi600_yuv_info, //&g_tlg1120_yuv_info,  bonnie
	PNULL
};

/*****************************************************************************/
//  Description:    This function is used to get sensor information table    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
SENSOR_INFO_T ** Sensor_GetInforTab(SENSOR_ID_E sensor_id)
{
	SENSOR_INFO_T * sensor_infor_tab_ptr=NULL;

	switch(sensor_id)
	{
		case SENSOR_MAIN:
		{
			sensor_infor_tab_ptr=(SENSOR_INFO_T*)&main_sensor_infor_tab;
			break;
		}
		case SENSOR_SUB:
		{
			sensor_infor_tab_ptr=(SENSOR_INFO_T*)&sub_sensor_infor_tab;
			break;
		}
		case SENSOR_ATV: 
		{
			sensor_infor_tab_ptr=(SENSOR_INFO_T*)&atv_infor_tab;
			break;
		}
		default:
			break;
	}

	return (SENSOR_INFO_T **)sensor_infor_tab_ptr;
}

/*****************************************************************************/
//  Description:    This function is used to get sensor information table    
//  Author:         Liangwen.Zhen
//  Note:           
/*****************************************************************************/
uint32_t Sensor_GetInforTabLenght(SENSOR_ID_E sensor_id)
{
	uint32_t tab_lenght = 0;

	switch(sensor_id)
	{
		case SENSOR_MAIN:
		{
			tab_lenght=(sizeof(main_sensor_infor_tab)/sizeof(SENSOR_INFO_T*));
			break;
		}
		case SENSOR_SUB:
		{
			tab_lenght=(sizeof(sub_sensor_infor_tab)/sizeof(SENSOR_INFO_T*));
			break;
		}
		case SENSOR_ATV: 
		{
			tab_lenght=(sizeof(atv_infor_tab)/sizeof(SENSOR_INFO_T*));
			break;
		}
		default:
			break;
	}

	return tab_lenght;
}


#if 0
static LIST_HEAD(main_sensor_info_list);	/*for back camera */
static LIST_HEAD(sub_sensor_info_list);	/*for front camera */
static LIST_HEAD(atv_info_list);	/*for atv */
static DEFINE_MUTEX(sensor_mutex);


int dcam_register_sensor_drv(struct sensor_drv_cfg *cfg)
{
	//printk(KERN_INFO "Sensor driver is %s.\n", cfg->sensor_name);
	mutex_lock(&sensor_mutex);
	if (cfg->sensor_pos == 1) {
		list_add_tail(&cfg->list, &main_sensor_info_list);
	} else if (cfg->sensor_pos == 2) {
		list_add_tail(&cfg->list, &sub_sensor_info_list);
	} else if (cfg->sensor_pos == 3) {
		list_add_tail(&cfg->list, &main_sensor_info_list);
		list_add_tail(&cfg->list, &sub_sensor_info_list);
	} else {
		list_add_tail(&cfg->list, &atv_info_list);
	}
	mutex_unlock(&sensor_mutex);

	return 0;
}

struct list_head *Sensor_GetList(SENSOR_ID_E sensor_id)
{
	struct list_head *sensor_list = 0;

	pr_debug("sensor cfg:Sensor_GetList,id=%d.\n", sensor_id);
	switch (sensor_id) {
	case SENSOR_MAIN:
		sensor_list = &main_sensor_info_list;
		break;
	case SENSOR_SUB:
		sensor_list = &sub_sensor_info_list;
		break;
	case SENSOR_ATV:
		sensor_list = &atv_info_list;
		break;
	default:
		printk("sensor cfg:Sensor_GetList fail!\n");
		break;
	}

	return sensor_list;
}
#endif
