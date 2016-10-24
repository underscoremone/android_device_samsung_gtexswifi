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
#include <utils/Log.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include "sensor_drv_u.h"
#include "dc_cfg.h"
#include "dc_product_cfg.h"

EXIF_GPS_INFO_T s_dc_gps_info;
EXIF_SPEC_DATE_TIME_T image_date_time;
EXIF_SPECIFIC_INFO_T s_dc_specific_info;
JINF_EXIF_INFO_T s_dc_exif_info;
EXIF_SPEC_OTHER_T s_dc_spec_other;
const JINF_EXIF_INFO_T* s_dc_exif_info_ptr=&s_dc_exif_info;

static EXIF_PRI_DATA_STRUCT_T exif_prim_data = {
	{1, 0}, /*just Orientation valid*/
	1, /*The 0th row is at the visual top of the image, and the 0th column is the visual left-hand side*/
	0
};

static void _DC_SetExifSpecificBasicParameter(void)
{
	EXIF_SPECIFIC_INFO_T* dc_specific_info_ptr = &s_dc_specific_info;

	dc_specific_info_ptr->basic.ColorSpace=1;
	dc_specific_info_ptr->basic.ComponentsConfiguration[0]=1;
	dc_specific_info_ptr->basic.ComponentsConfiguration[1]=2;
	dc_specific_info_ptr->basic.ComponentsConfiguration[2]=3;
	dc_specific_info_ptr->basic.ComponentsConfiguration[3]=0;
}

static void _DC_SetExifSpecificUserParameter(void)
{
	DC_PRODUCT_CFG_FUNC_TAB_T_PTR exif_ptr = DC_GetDcProductCfgFun();
	EXIF_SPECIFIC_INFO_T* dc_specific_info_ptr=&s_dc_specific_info;

	if (PNULL!=exif_ptr->get_exifspecuser) {
		dc_specific_info_ptr->user_ptr=(EXIF_SPEC_USER_T*)exif_ptr->get_exifspecuser(0x00);
	} else {
		dc_specific_info_ptr->user_ptr=NULL;
	}
}

void _DC_SetExifSpecificPicTakingParameter(void)
{
	EXIF_SPEC_PIC_TAKING_COND_T* img_sensor_exif_ptr=Sensor_GetSensorExifInfo();
	EXIF_SPECIFIC_INFO_T* dc_specific_info_ptr=&s_dc_specific_info;

	dc_specific_info_ptr->pic_taking_cond_ptr = img_sensor_exif_ptr;
}

static void _DC_SetExifPrimaryParameter(void)
{
	JINF_EXIF_INFO_T* dc_exif_info_ptr=(JINF_EXIF_INFO_T* )s_dc_exif_info_ptr;
	DC_PRODUCT_CFG_FUNC_TAB_T_PTR exif_ptr=DC_GetDcProductCfgFun();

	dc_exif_info_ptr->primary.basic.YCbCrPositioning=1;
	dc_exif_info_ptr->primary.basic.XResolution.numerator=72;
	dc_exif_info_ptr->primary.basic.XResolution.denominator=1;
	dc_exif_info_ptr->primary.basic.YResolution.numerator=72;
	dc_exif_info_ptr->primary.basic.YResolution.denominator=1;
	dc_exif_info_ptr->primary.basic.ResolutionUnit=2;
	exif_prim_data.valid.Orientation = 1;
	exif_prim_data.Orientation = 1;
	dc_exif_info_ptr->primary.data_struct_ptr = &exif_prim_data;

	if (PNULL!=exif_ptr->get_exifprimarypridesc) {
		dc_exif_info_ptr->primary.img_desc_ptr=(EXIF_PRI_DESC_T*)exif_ptr->get_exifprimarypridesc(0x00);
	} else {
		dc_exif_info_ptr->primary.img_desc_ptr=NULL;
	}

}

static void _DC_SetExifSpecificParameter(void)
{
	JINF_EXIF_INFO_T* dc_exif_info_ptr=(JINF_EXIF_INFO_T* )s_dc_exif_info_ptr;
#if 0
	SCI_DATE_T       cur_date = {0};
	SCI_TIME_T       cur_time = {0};

	TM_GetSysDate(&cur_date);
	TM_GetSysTime(&cur_time);
#endif
	time_t timep;
	struct tm *p;
	time(&timep);
	p=gmtime(&timep);

	dc_exif_info_ptr->spec_ptr=&s_dc_specific_info;

	_DC_SetExifSpecificBasicParameter();
	_DC_SetExifSpecificUserParameter();
	_DC_SetExifSpecificPicTakingParameter();
	s_dc_spec_other.valid.ImageUniqueID = 1;

	sprintf((char *)s_dc_spec_other.ImageUniqueID,
		"%04d:%02d:%02d %02d:%02d:%02d",
		(1900+p->tm_year),
		(1+p->tm_mon),
		p->tm_mday,
		p->tm_hour,
		p->tm_min,
		p->tm_sec);

	CMR_LOGD("_DC_SetExifSpecificParameter %s",s_dc_spec_other.ImageUniqueID);
	dc_exif_info_ptr->spec_ptr->other_ptr = &s_dc_spec_other;
}

static void _DC_SetExifGpsParameter(void)
{
	JINF_EXIF_INFO_T* dc_exif_info_ptr=(JINF_EXIF_INFO_T* )s_dc_exif_info_ptr;
#if 0
	s_dc_gps_info.GPSVersionID[0] = 2;
	s_dc_gps_info.GPSVersionID[1] = 2;
	s_dc_gps_info.GPSVersionID[2] = 0;
	s_dc_gps_info.GPSVersionID[3] = 0;
	*(uint32*)&s_dc_gps_info.valid = (uint32)0x7F;
	s_dc_gps_info.GPSLatitudeRef[0] = 'N';
	s_dc_gps_info.GPSLatitude[0].numerator = 31;
	s_dc_gps_info.GPSLatitude[0].denominator = 1;
	s_dc_gps_info.GPSLatitude[1].numerator = 2;
	s_dc_gps_info.GPSLatitude[1].denominator = 1;
	s_dc_gps_info.GPSLatitude[2].numerator = 0;
	s_dc_gps_info.GPSLatitude[2].denominator = 1;

	s_dc_gps_info.GPSLongitudeRef[0] = 'E';
	s_dc_gps_info.GPSLongitude[0].numerator = 121;
	s_dc_gps_info.GPSLongitude[0].denominator = 1;
	s_dc_gps_info.GPSLongitude[1].numerator = 4;
	s_dc_gps_info.GPSLongitude[1].denominator = 1;
	s_dc_gps_info.GPSLongitude[2].numerator = 0;
	s_dc_gps_info.GPSLongitude[2].denominator = 1;

	s_dc_gps_info.GPSAltitudeRef = 0;
	s_dc_gps_info.GPSAltitude.numerator = 4;
	s_dc_gps_info.GPSAltitude.denominator = 1;
#endif
	dc_exif_info_ptr->gps_ptr = &s_dc_gps_info;

}

void DC_SetExifImagePixel(uint32_t width, uint32_t height)
{
	EXIF_SPECIFIC_INFO_T* dc_specific_info_ptr = &s_dc_specific_info;

	dc_specific_info_ptr->basic.PixelXDimension = width;
	dc_specific_info_ptr->basic.PixelYDimension = height;
}

void DC_SetExifImageDataTime(void)
{
	EXIF_SPECIFIC_INFO_T* dc_specific_info_ptr = &s_dc_specific_info;
#if 0
	SCI_DATE_T                   cur_date = {0};
	SCI_TIME_T                    cur_time = {0};

	TM_GetSysDate(&cur_date);
	TM_GetSysTime(&cur_time);
#endif
	time_t timep;
	struct tm *p;
	time(&timep);
	p=gmtime(&timep);

	sprintf((char *)image_date_time.DateTimeOriginal,
		"%04d:%02d:%02d %02d:%02d:%02d",
		(1900+p->tm_year),
		(1+p->tm_mon),
		p->tm_mday,
		p->tm_hour,
		p->tm_min,
		p->tm_sec);
	strcpy((char*)image_date_time.DateTimeDigitized,(char*)image_date_time.DateTimeOriginal);
	image_date_time.valid.DateTimeOriginal = 1;
	image_date_time.valid.DateTimeDigitized = 1;

	CMR_LOGD("DC_SetExifImageDataTime %s",image_date_time.DateTimeOriginal);

	dc_specific_info_ptr->date_time_ptr = &image_date_time;

}

JINF_EXIF_INFO_T* DC_GetExifParameter(void)
{
	_DC_SetExifPrimaryParameter();
	_DC_SetExifSpecificParameter();
	_DC_SetExifGpsParameter();
	DC_SetExifImageDataTime();

	CMR_LOGD("DC_GetExifParameter");
	return (JINF_EXIF_INFO_T* )s_dc_exif_info_ptr;
}

DC_PRODUCT_CFG_T_PTR DC_GeProductCfgPtr(void)
{
	DC_PRODUCT_CFG_FUNC_TAB_T_PTR dc_productcfgfunptr = DC_GetDcProductCfgFun();
	DC_PRODUCT_CFG_T_PTR dc_productcfgptr=NULL;

	if (PNULL!=dc_productcfgfunptr->get_productcfg) {
		dc_productcfgptr=(DC_PRODUCT_CFG_T_PTR)dc_productcfgfunptr->get_productcfg(0x00);
	}

	return dc_productcfgptr;
}

