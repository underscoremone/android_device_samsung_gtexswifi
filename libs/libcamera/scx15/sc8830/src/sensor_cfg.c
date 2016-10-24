/*
 * Copyright (C) 2012 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "sensor_drv_u.h"
#include "sensor_cfg.h"

#if 0
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
extern SENSOR_INFO_T g_nmi600_yuv_info;
extern SENSOR_INFO_T g_ov5640_mipi_raw_info;
extern SENSOR_INFO_T g_ov5647_mipi_raw_info;
extern SENSOR_INFO_T g_ov5648_mipi_raw_info;
extern SENSOR_INFO_T g_s5k5ccgx_yuv_info_mipi;
extern SENSOR_INFO_T g_s5k4e1ga_mipi_raw_info;
extern SENSOR_INFO_T g_hi351_mipi_yuv_info;
extern SENSOR_INFO_T g_ov8830_mipi_raw_infoextern;
#endif
SENSOR_INFO_T g_GT2005_yuv_info;
SENSOR_INFO_T g_GC0308_yuv_info;
SENSOR_INFO_T g_GC2035_yuv_info;
extern SENSOR_INFO_T g_ov5640_mipi_yuv_info;
extern SENSOR_INFO_T g_ov8825_mipi_raw_info;
extern SENSOR_INFO_T g_imx179_mipi_raw_info;
extern SENSOR_INFO_T g_ov8865_mipi_raw_info;
extern SENSOR_INFO_T g_ov13850_mipi_raw_info;
extern SENSOR_INFO_T g_s5k4ec_mipi_yuv_info;
extern SENSOR_INFO_T g_HI702_yuv_info;
extern SENSOR_INFO_T g_ov5640_yuv_info;
extern SENSOR_INFO_T g_OV7675_yuv_info;
extern SENSOR_INFO_T g_hi253_yuv_info;
#if defined(CONFIG_CAMERA_X3542)
SENSOR_INFO_T g_GC2155_yuv_info;
SENSOR_INFO_T g_GC0311_yuv_info;
#endif
extern SENSOR_INFO_T g_s5k4ec_yuv_info;
extern SENSOR_INFO_T g_autotest_ov8825_mipi_raw_info;
extern SENSOR_INFO_T g_autotest_ov5640_mipi_yuv_info;
extern SENSOR_INFO_T g_at_ov5640_ccir_yuv_info;
extern SENSOR_INFO_T g_autotest_yuv_info;
extern SENSOR_INFO_T g_sr352_yuv_info;
extern SENSOR_INFO_T g_sr030pc50_yuv_info;

#define AUTO_TEST_CAMERA 1
/**---------------------------------------------------------------------------*
 **                         Constant Variables                                *
 **---------------------------------------------------------------------------*/
const SENSOR_INFO_T* main_sensor_infor_tab[]=
{
#ifdef CONFIG_BACK_CAMERA_MIPI
	&g_s5k4ec_mipi_yuv_info,
	&g_ov8825_mipi_raw_info,
	//&g_ov8830_mipi_raw_info,
	&g_ov5640_mipi_yuv_info,
	&g_imx179_mipi_raw_info,
	&g_ov8865_mipi_raw_info,
	&g_ov13850_mipi_raw_info,
	//&g_ov5640_mipi_raw_info,
	//&g_s5k5ccgx_yuv_info_mipi,
	//&g_s5k4e1ga_mipi_raw_info,
	//&g_hi351_mipi_yuv_info,
	//&g_ov5640_mipi_raw_info,
	//&g_ov5647_mipi_raw_info,
	//&g_ov5648_mipi_raw_info,
#endif
#ifdef CONFIG_BACK_CAMERA_CCIR
	&g_ov5640_yuv_info,
	&g_hi253_yuv_info,
	&g_GT2005_yuv_info,
	&g_s5k4ec_yuv_info,
	&g_sr352_yuv_info,
	#if defined(CONFIG_CAMERA_X3542)
	&g_GC2035_yuv_info,
	&g_GC2155_yuv_info,
	#endif
#endif
	PNULL
};

const SENSOR_INFO_T* sub_sensor_infor_tab[]=
{
#ifdef CONFIG_FRONT_CAMERA_CCIR
	&g_OV7675_yuv_info,
	&g_GC0308_yuv_info,
	&g_GC2035_yuv_info,
	&g_HI702_yuv_info,
	#if defined(CONFIG_CAMERA_X3542)
	&g_GC0311_yuv_info,
	#endif
	&g_sr030pc50_yuv_info,
#endif
	PNULL
};


const SENSOR_INFO_T* atv_infor_tab[]=
{
	//&g_nmi600_yuv_info, //&g_tlg1120_yuv_info,
	PNULL
};

/*
*add for auto test for main and sub camera (raw yuv)
* 2014-02-07 freed wang  begin
*/
const SENSOR_INFO_T* at_main_sensor_infor_tab[]=
{
#ifdef CONFIG_BACK_CAMERA_MIPI
	&g_autotest_ov8825_mipi_raw_info,
	&g_autotest_ov5640_mipi_yuv_info,
#endif
#ifdef CONFIG_BACK_CAMERA_CCIR
	&g_at_ov5640_ccir_yuv_info,
	&g_hi253_yuv_info,
	//&g_GT2005_yuv_info,
	//&g_s5k4ec_yuv_info,
	&g_autotest_yuv_info,
#endif
PNULL

};

const SENSOR_INFO_T* at_sub_sensor_infor_tab[]=
{
#ifdef CONFIG_FRONT_CAMERA_CCIR
	&g_GC0308_yuv_info,
	&g_GC2035_yuv_info,
	//&g_HI702_yuv_info,
	&g_OV7675_yuv_info,
	//&g_autotest_yuv_info,
#endif
PNULL

};

const SENSOR_INFO_T* at_atv_infor_tab[]=
{
	//&g_nmi600_yuv_info, //&g_tlg1120_yuv_info,  bonnie
	PNULL
};
/*
* add for auto test for main and sub camera (raw yuv)
* 2014-02-07 freed wang end
*/
SENSOR_INFO_T ** Sensor_GetInforTab(SENSOR_ID_E sensor_id)
{
	SENSOR_INFO_T * sensor_infor_tab_ptr=NULL;

	int at_flag=Sensor_GetAutoTest();
	CMR_LOGE("%s autotest_camera_flag= %d  line=  %d ",__func__,at_flag,__LINE__);

	if (AUTO_TEST_CAMERA == at_flag) {
		switch ( sensor_id) {
			case SENSOR_MAIN:
					sensor_infor_tab_ptr=(SENSOR_INFO_T*)&at_main_sensor_infor_tab;
					break;
			case SENSOR_SUB:
					sensor_infor_tab_ptr=(SENSOR_INFO_T*)&at_sub_sensor_infor_tab;
					break;
			case SENSOR_ATV:
					sensor_infor_tab_ptr=(SENSOR_INFO_T*)&at_atv_infor_tab;
					break;
			default:
				break;
				CMR_LOGE("%s s_autotest_flag= %d  line= %d ",__func__,at_flag,__LINE__);
		}
		return (SENSOR_INFO_T **)sensor_infor_tab_ptr;

	}
	switch (sensor_id) {
		case SENSOR_MAIN:
			sensor_infor_tab_ptr=(SENSOR_INFO_T*)&main_sensor_infor_tab;
			break;

		case SENSOR_SUB:
			sensor_infor_tab_ptr=(SENSOR_INFO_T*)&sub_sensor_infor_tab;
			break;

		case SENSOR_ATV:
			sensor_infor_tab_ptr=(SENSOR_INFO_T*)&atv_infor_tab;
			break;

		default:
			break;
	}

	return (SENSOR_INFO_T **)sensor_infor_tab_ptr;
}

uint32_t Sensor_GetInforTabLenght(SENSOR_ID_E sensor_id)
{
	uint32_t tab_lenght = 0;

	switch (sensor_id) {
		case SENSOR_MAIN:
			tab_lenght=(sizeof(main_sensor_infor_tab)/sizeof(SENSOR_INFO_T*));
			break;

		case SENSOR_SUB:
			tab_lenght=(sizeof(sub_sensor_infor_tab)/sizeof(SENSOR_INFO_T*));
			break;

		case SENSOR_ATV:
			tab_lenght=(sizeof(atv_infor_tab)/sizeof(SENSOR_INFO_T*));
			break;

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
