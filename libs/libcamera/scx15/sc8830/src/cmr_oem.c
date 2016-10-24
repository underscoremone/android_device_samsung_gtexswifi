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
#include <stdlib.h>
#include <math.h>
#include <cutils/properties.h>
#include "SprdOEMCamera.h"
#include "cmr_oem.h"
#include "dc_cfg.h"
#include "sensor_drv_u.h"

#define SENSOR_PARAM_NUM        8
#define SENSOR_PARA             "/data/misc/sensors/sensor.file"
#define EXIF_DEF_MAKER          "Spreadtrum"
#define EXIF_DEF_MODEL          "spxxxx"


static camera_position_type s_position;
const char image_desc[] = "Exif_JPEG_420";
/* const char image_make[] = "Spreadtrum"; */ /*the value will be read from system property*/
const char copyright[] = "Copyright,Spreadtrum,2011";
/* const char model[] = "SmartPhone"; */ /*the value will be read from system property*/

uint32_t camera_get_rot_angle(uint32_t degree)
{
	uint32_t                 angle = IMG_ROT_0;

	switch (degree) {
	case 90:
		angle = IMG_ROT_90;
		break;

	case 180:
		angle = IMG_ROT_180;
		break;

	case 270:
		angle = IMG_ROT_270;
		break;

	default:
		angle = IMG_ROT_0;
		break;
	}

	return angle;
}
uint32_t camera_get_img_type(uint32_t format_mode)
{
	int                      ret = CAMERA_SUCCESS;
	uint32_t                 ret_4cc;

	switch(format_mode) {
	case 0:
		ret_4cc = IMG_DATA_TYPE_YUV422;
		break;

	case 1:
		ret_4cc = IMG_DATA_TYPE_YUV420;
		break;

	case 2:
		ret_4cc = IMG_DATA_TYPE_RGB565;
		break;

	case 3:
		ret_4cc = IMG_DATA_TYPE_JPEG;
		break;

	case 4:
		ret_4cc = IMG_DATA_TYPE_RAW;
		break;

	default:
		ret_4cc = IMG_DATA_TYPE_YUV420;
		break;
	}

	return ret_4cc;
}

int camera_get_trim_rect(struct img_rect *src_trim_rect, uint32_t zoom_level, struct img_size *dst_size)
{
	uint32_t                 trim_width;
	uint32_t                 trim_height;
	uint32_t                 zoom_step_w = 0, zoom_step_h = 0;

	if (NULL == src_trim_rect || NULL == dst_size) {
		CMR_LOGE("0x%x, 0x%x", (uint32_t)src_trim_rect, (uint32_t)dst_size);
		return -CAMERA_INVALID_PARM;
	}

	trim_width = src_trim_rect->width;
	trim_height = src_trim_rect->height;

	if (0 == dst_size->width || 0 == dst_size->height) {
		CMR_LOGE("0x%x, 0x%x", dst_size->width, dst_size->height);
		return -CAMERA_INVALID_PARM;
	}

	if (dst_size->width * src_trim_rect->height < dst_size->height * src_trim_rect->width) {
		trim_width = dst_size->width * src_trim_rect->height / dst_size->height;
	} else {
		trim_height = dst_size->height * src_trim_rect->width / dst_size->width;
	}

	zoom_step_w = ZOOM_STEP(trim_width);
	zoom_step_w &= ~1;
	zoom_step_w *= zoom_level;

 	zoom_step_h = ZOOM_STEP(trim_height);
	zoom_step_h &= ~1;
	zoom_step_h *= zoom_level;
	trim_width = trim_width - zoom_step_w;
	trim_height = trim_height - zoom_step_h;

	src_trim_rect->start_x += (src_trim_rect->width - trim_width) >> 1;
	src_trim_rect->start_y += (src_trim_rect->height - trim_height) >> 1;
	src_trim_rect->start_x = CAMERA_WIDTH(src_trim_rect->start_x);
	src_trim_rect->start_y = CAMERA_HEIGHT(src_trim_rect->start_y);
	src_trim_rect->width = CAMERA_WIDTH(trim_width);
	src_trim_rect->height = CAMERA_HEIGHT(trim_height);
	CMR_LOGI("zoom_level %d, trim rect, %d %d %d %d",
		zoom_level,
		src_trim_rect->start_x,
		src_trim_rect->start_y,
		src_trim_rect->width,
		src_trim_rect->height);

	return CAMERA_SUCCESS;
}

int camera_get_trim_rect2(struct img_rect *src_trim_rect, float zoomRatio, float dstRatio,uint32_t SensorW, uint32_t SensorH, uint8_t Rot)//for hal2.0 calculate crop again
{
	uint32_t                 trim_width;
	uint32_t                 trim_height;
	float    minOutputRatio;
	float    zoomWidth,zoomHeight,SensorRatio;

	if (NULL == src_trim_rect) {
		CMR_LOGE("NULL");
		return -CAMERA_INVALID_PARM;
	} else if (src_trim_rect->width == 0 || src_trim_rect->height == 0) {
		CMR_LOGE("0x%x w=%d h=%d", (uint32_t)src_trim_rect,src_trim_rect->width,src_trim_rect->height);
		return -CAMERA_INVALID_PARM;
	}
	minOutputRatio = dstRatio;
	SensorRatio = (float)SensorW /SensorH;
	if (Rot != IMG_ROT_0) {
		minOutputRatio = 1 / minOutputRatio;
	}
	if (minOutputRatio > SensorRatio) {
		zoomWidth = SensorW / zoomRatio;
		zoomHeight = zoomWidth / minOutputRatio;
	} else {
		zoomHeight = SensorH / zoomRatio;
		zoomWidth = zoomHeight * minOutputRatio;
	}
	trim_width = zoomWidth;
	trim_height = zoomHeight;
	src_trim_rect->start_x += (src_trim_rect->width - trim_width) >> 1;
	src_trim_rect->start_y += (src_trim_rect->height - trim_height) >> 1;
	src_trim_rect->start_x = CAMERA_WIDTH(src_trim_rect->start_x);
	src_trim_rect->start_y = CAMERA_HEIGHT(src_trim_rect->start_y);
	src_trim_rect->width = CAMERA_WIDTH(trim_width);
	src_trim_rect->height = CAMERA_HEIGHT(trim_height);
	CMR_LOGI("zoom_level %f, trim rect, %d %d %d %d",
		zoomRatio,
		src_trim_rect->start_x,
		src_trim_rect->start_y,
		src_trim_rect->width,
		src_trim_rect->height);

	return CAMERA_SUCCESS;
}

uint32_t getOrientationFromRotationDegrees(int degrees)
{
	uint32_t orientation = 1;/*ExifInterface.ORIENTATION_NORMAL;*/
	degrees %= 360;
	if (degrees < 0) degrees += 360;
	if (degrees < 45) {
		orientation = 1;/*ExifInterface.ORIENTATION_NORMAL;*/
	} else if (degrees < 135) {
		orientation = 6;/*ExifInterface.ORIENTATION_ROTATE_90;*/
	} else if (degrees < 225) {
		orientation = 3;/*ExifInterface.ORIENTATION_ROTATE_180;*/
	} else {
		orientation = 8;/*ExifInterface.ORIENTATION_ROTATE_270;*/
	}
	CMR_LOGD("rotation degrees: %d, orientation: %d.", degrees, orientation);
	return orientation;
}

static void getSecondsFromDouble(double d, uint32_t *numerator, uint32_t *denominator)
{
	d = fabs(d);
	double degrees = (int)d;
	double remainder = d - degrees;
	double minutes = (int)(remainder * 60.0);
	double seconds = (((remainder * 60.0) - minutes) * 60.0);
	uint32_t i = 0;
	uint32_t j = 0;
	uint32_t num = 0;
	char str[20];
	double value=1.0;

	sprintf(str,"%f",seconds);
	while(str[i++]!='.')
		;
	j = strlen(str)-1;
	while(str[j] == '0')
		--j;
	num = j - i + 1;
	CMR_LOGI("%s, i=%d, j=%d, num=%d \n", str, i, j, num);

	for (i=0; i<num; i++)
		value *=10.0;

	*numerator = seconds*value;
	*denominator = value;

	CMR_LOGI("data=%f, num=%d, denom=%d \n", seconds, *numerator, *denominator);
}

static uint32_t getDataFromDouble(double d, uint32_t type)/*0: dd, 1: mm, 2: ss*/
{
	d = fabs(d);
	double degrees = (int)d;
	double remainder = d - degrees;
	double minutes = (int)(remainder * 60.0);
	double seconds = (int)(((remainder * 60.0) - minutes) * 60.0);
	uint32_t retVal = 0;
	if (0 == type) {
		retVal = (int)degrees;
	} else if (1 == type) {
		retVal = (int)minutes;
	} else if (2 == type) {
		retVal = (int)seconds;
	}
	CMR_LOGI("GPS: type: %d, ret: 0x%x.", type, retVal);
	return retVal;
}

int camera_set_pos_type(camera_position_type *position)
{
	s_position.timestamp = position->timestamp;
	s_position.altitude = position->altitude;
	s_position.latitude = position->latitude;
	s_position.longitude = position->longitude;
	s_position.process_method = position->process_method;
	/*for test*/
	/*s_position.timestamp = 1199145600;
	s_position.altitude = 21;
	s_position.latitude = 37.736071;
	s_position.longitude = -122.441983;
	s_position.process_method = "GPS NETWORK HYBRID ARE ALL FINE.";*/

	CMR_LOGI("timestamp %ld, latitude : %f, longitude : %f,altitude: %f. ",
		position->timestamp,
		(double)position->latitude,
		(double)position->longitude,
		(double)position->altitude);
	return 0;
}

JINF_EXIF_INFO_T* camera_get_exif(struct camera_context *p_cxt)
{
	JINF_EXIF_INFO_T *p_exif_info = DC_GetExifParameter();
	uint32_t latitude_dd_numerator = getDataFromDouble(s_position.latitude, 0);
	uint32_t latitude_dd_denominator = 1;
	uint32_t latitude_mm_numerator = getDataFromDouble(s_position.latitude, 1);
	uint32_t latitude_mm_denominator = 1;
	uint32_t latitude_ss_numerator = 0;
	uint32_t latitude_ss_denominator = 0;
	uint32_t latitude_ref = 0;
	uint32_t longitude_ref = 0;
	uint32_t longitude_dd_numerator = 0;
	uint32_t longitude_dd_denominator = 0;
	uint32_t longitude_mm_numerator = 0;
	uint32_t longitude_mm_denominator = 0;
	uint32_t longitude_ss_numerator = 0;
	uint32_t longitude_ss_denominator = 0;
	char datetime_buf[20];
	char gps_date_buf[12];
	time_t timep;
	struct tm *p;
	char *datetime;
	char *gps_date;
	const char *gps_process_method;
	uint32_t gps_hour;
	uint32_t gps_minuter;
	uint32_t gps_second;
	uint32_t focal_length_numerator;
	uint32_t focal_length_denominator;
	char property[PROPERTY_VALUE_MAX];

	getSecondsFromDouble(s_position.latitude, &latitude_ss_numerator, &latitude_ss_denominator);

	if (s_position.latitude < 0.0) {
		latitude_ref = 1;
	} else {
		latitude_ref = 0;
	}
	longitude_dd_numerator = getDataFromDouble(s_position.longitude, 0);
	longitude_dd_denominator = 1;
	longitude_mm_numerator = getDataFromDouble(s_position.longitude, 1);
	longitude_mm_denominator = 1;
	getSecondsFromDouble(s_position.longitude, &longitude_ss_numerator, &longitude_ss_denominator);
	if (s_position.longitude < 0.0) {
		longitude_ref = 1;
	} else {
		longitude_ref = 0;
	}

	time(&timep);
	p = localtime(&timep);
	sprintf(datetime_buf,
		"%4d:%02d:%02d %02d:%02d:%02d",
		(1900+p->tm_year),
		(1+p->tm_mon),
		p->tm_mday,
		p->tm_hour,
		p->tm_min,
		p->tm_sec);
	datetime_buf[19] = '\0';
	datetime = datetime_buf;

	CMR_LOGD("datetime %s",datetime);

	if (0 == s_position.timestamp)
		time(&s_position.timestamp);
	p = gmtime(&s_position.timestamp);
	sprintf(gps_date_buf, "%4d:%02d:%02d", (1900+p->tm_year), (1+p->tm_mon),p->tm_mday);
	gps_date_buf[10] = '\0';
	gps_date = gps_date_buf;
	gps_hour = p->tm_hour;
	gps_minuter = p->tm_min;
	gps_second = p->tm_sec;
	CMR_LOGD("gps_data 2 = %s, %d:%d:%d \n", gps_date, gps_hour, gps_minuter, gps_second);

	gps_process_method = s_position.process_method;
	focal_length_numerator = p_cxt->cmr_set.focal_len;
	focal_length_denominator = 1000;
	/* Some info is not get from the kernel */
	if (NULL != p_exif_info->spec_ptr) {
		p_exif_info->spec_ptr->basic.PixelXDimension = p_cxt->picture_size.width;
		p_exif_info->spec_ptr->basic.PixelYDimension = p_cxt->picture_size.height;
	}
	p_exif_info->primary.basic.ImageWidth = p_cxt->actual_picture_size.width;
	p_exif_info->primary.basic.ImageLength = p_cxt->actual_picture_size.height;
	CMR_LOGD("EXIF width=%d, height=%d \n",
			p_exif_info->primary.basic.ImageWidth,
			p_exif_info->primary.basic.ImageLength);

	if (NULL != p_exif_info->primary.data_struct_ptr) {
		p_exif_info->primary.data_struct_ptr->valid.Orientation = 1;
		p_exif_info->primary.data_struct_ptr->Orientation =
			getOrientationFromRotationDegrees(p_cxt->jpeg_cxt.set_encode_rotation);
	}

	if (NULL != p_exif_info->primary.img_desc_ptr) {
		strcpy((char *)p_exif_info->primary.img_desc_ptr->ImageDescription, (char *)image_desc);
		memset(property,'\0',sizeof(property));
		property_get("ro.product.manufacturer", property, EXIF_DEF_MAKER);
		strcpy((char *)p_exif_info->primary.img_desc_ptr->Make, (char *)property);

		memset(property,'\0',sizeof(property));
		property_get("ro.product.model", property, EXIF_DEF_MODEL);
		strcpy((char *)p_exif_info->primary.img_desc_ptr->Model, (char *)property);
		strcpy((char *)p_exif_info->primary.img_desc_ptr->Copyright, (char *)copyright);
	}

	if (NULL != p_exif_info->gps_ptr) {
		if ((0 == latitude_dd_numerator) && (0 == latitude_mm_numerator) && (0 == latitude_ss_numerator)
			&& (0 == longitude_dd_numerator) && (0 == longitude_mm_numerator) && (0 == longitude_ss_numerator)) {
			/* if no Latitude and Longitude, do not write GPS to EXIF */
			CMR_LOGD("GPS: Latitude and Longitude is 0, do not write to EXIF: valid=%d \n",
					*(uint32_t*)&p_exif_info->gps_ptr->valid);
			memset(&p_exif_info->gps_ptr->valid,0,sizeof(EXIF_GPS_VALID_T));
		} else {
			p_exif_info->gps_ptr->valid.GPSLatitudeRef = 1;
			p_exif_info->gps_ptr->GPSLatitudeRef[0] = (0 == latitude_ref) ? 'N' : 'S';
			p_exif_info->gps_ptr->valid.GPSLongitudeRef = 1;
			p_exif_info->gps_ptr->GPSLongitudeRef[0] = (0 == longitude_ref) ? 'E' : 'W';

			p_exif_info->gps_ptr->valid.GPSLatitude = 1;
			p_exif_info->gps_ptr->GPSLatitude[0].numerator = latitude_dd_numerator;
			p_exif_info->gps_ptr->GPSLatitude[0].denominator = latitude_dd_denominator;
			p_exif_info->gps_ptr->GPSLatitude[1].numerator = latitude_mm_numerator;
			p_exif_info->gps_ptr->GPSLatitude[1].denominator = latitude_mm_denominator;
			p_exif_info->gps_ptr->GPSLatitude[2].numerator = latitude_ss_numerator;
			p_exif_info->gps_ptr->GPSLatitude[2].denominator = latitude_ss_denominator;

			p_exif_info->gps_ptr->valid.GPSLongitude = 1;
			p_exif_info->gps_ptr->GPSLongitude[0].numerator = longitude_dd_numerator;
			p_exif_info->gps_ptr->GPSLongitude[0].denominator = longitude_dd_denominator;
			p_exif_info->gps_ptr->GPSLongitude[1].numerator = longitude_mm_numerator;
			p_exif_info->gps_ptr->GPSLongitude[1].denominator = longitude_mm_denominator;
			p_exif_info->gps_ptr->GPSLongitude[2].numerator = longitude_ss_numerator;
			p_exif_info->gps_ptr->GPSLongitude[2].denominator = longitude_ss_denominator;

			p_exif_info->gps_ptr->valid.GPSAltitude = 1;
			p_exif_info->gps_ptr->GPSAltitude.numerator = s_position.altitude;
			CMR_LOGD("gps_ptr->GPSAltitude.numerator: %d.", p_exif_info->gps_ptr->GPSAltitude.numerator);
			p_exif_info->gps_ptr->GPSAltitude.denominator = 1;
			p_exif_info->gps_ptr->valid.GPSAltitudeRef = 1;

			if(NULL != gps_process_method) {
				const char ascii[] = {0x41, 0x53, 0x43, 0x49, 0x49, 0, 0, 0};
				p_exif_info->gps_ptr->valid.GPSProcessingMethod = 1;
				p_exif_info->gps_ptr->GPSProcessingMethod.count = strlen(gps_process_method)+sizeof(ascii) + 1;
				memcpy((char *)p_exif_info->gps_ptr->GPSProcessingMethod.ptr, ascii, sizeof(ascii));
				strcpy((char *)p_exif_info->gps_ptr->GPSProcessingMethod.ptr+sizeof(ascii), (char *)gps_process_method);
				/*add "ASCII\0\0\0" for cts test by lyh*/
			}

			p_exif_info->gps_ptr->valid.GPSTimeStamp = 1;
			p_exif_info->gps_ptr->GPSTimeStamp[0].numerator = gps_hour;
			p_exif_info->gps_ptr->GPSTimeStamp[1].numerator = gps_minuter;
			p_exif_info->gps_ptr->GPSTimeStamp[2].numerator = gps_second;

			p_exif_info->gps_ptr->GPSTimeStamp[0].denominator = 1;
			p_exif_info->gps_ptr->GPSTimeStamp[1].denominator = 1;
			p_exif_info->gps_ptr->GPSTimeStamp[2].denominator = 1;
			p_exif_info->gps_ptr->valid.GPSDateStamp = 1;
			strcpy((char *)p_exif_info->gps_ptr->GPSDateStamp, (char *)gps_date);
			CMR_LOGD("GPS: valid=%d \n", *(uint32_t*)&p_exif_info->gps_ptr->valid);
		}
	}

	if ((NULL != p_exif_info->spec_ptr) && (NULL != p_exif_info->spec_ptr->pic_taking_cond_ptr)) {
		p_exif_info->spec_ptr->pic_taking_cond_ptr->valid.FocalLength = 1;
		p_exif_info->spec_ptr->pic_taking_cond_ptr->FocalLength.numerator = focal_length_numerator;
		p_exif_info->spec_ptr->pic_taking_cond_ptr->FocalLength.denominator = focal_length_denominator;
	}

	/* TODO: data time is get from user space now */
	if (NULL != p_exif_info->primary.img_desc_ptr) {
		CMR_LOGD("set DateTime.");
		strcpy((char *)p_exif_info->primary.img_desc_ptr->DateTime, (char *)datetime);
	}

	if (NULL != p_exif_info->spec_ptr) {
		if(NULL != p_exif_info->spec_ptr->other_ptr) {
			CMR_LOGD("set ImageUniqueID.");
			memset(p_exif_info->spec_ptr->other_ptr->ImageUniqueID, 0, sizeof(p_exif_info->spec_ptr->other_ptr->ImageUniqueID));
			sprintf((char *)p_exif_info->spec_ptr->other_ptr->ImageUniqueID,"IMAGE %s", datetime);
		}

		if(NULL != p_exif_info->spec_ptr->date_time_ptr) {
			strcpy((char *)p_exif_info->spec_ptr->date_time_ptr->DateTimeOriginal, (char *)datetime);
			strcpy((char *)p_exif_info->spec_ptr->date_time_ptr->DateTimeDigitized, (char *)datetime);
			CMR_LOGD("set DateTimeOriginal.");
		}
	}
	memset(&s_position, 0, sizeof(camera_position_type));
	return p_exif_info;
}

int camera_sync_var_init(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;

	sem_init(&p_cxt->init_sem, 0, 0);
	sem_init(&p_cxt->exit_sem, 0, 0);
	sem_init(&p_cxt->start_sem, 0, 0);
	sem_init(&p_cxt->stop_sem, 0, 0);
	sem_init(&p_cxt->set_sem, 0, 0);
	sem_init(&p_cxt->cap_path_sem, 0, 0);
	sem_init(&p_cxt->scale_path_sem, 0, 0);
	sem_init(&p_cxt->thum_sem, 0, 0);
	sem_init(&p_cxt->takepicdone_sem, 0, 0);
	sem_init(&p_cxt->takepic_callback_sem, 0, 0);
	sem_init(&p_cxt->sync_scale_sem, 0, 1);
	sem_init(&p_cxt->sync_rotate_sem, 0, 1);
	sem_init(&p_cxt->jpeg_specify_cxt.jpeg_specify_cap_sem, 0, 0);

	return ret;
}

int camera_sync_var_deinit(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;

	sem_destroy(&p_cxt->init_sem);
	sem_destroy(&p_cxt->exit_sem);
	sem_destroy(&p_cxt->start_sem);
	sem_destroy(&p_cxt->stop_sem);
	sem_destroy(&p_cxt->set_sem);
	sem_destroy(&p_cxt->cap_path_sem);
	sem_destroy(&p_cxt->scale_path_sem);
	sem_destroy(&p_cxt->thum_sem);
	sem_destroy(&p_cxt->takepicdone_sem);
	sem_destroy(&p_cxt->takepic_callback_sem);
	sem_destroy(&p_cxt->sync_scale_sem);
	sem_destroy(&p_cxt->sync_rotate_sem);
	sem_destroy(&p_cxt->jpeg_specify_cxt.jpeg_specify_cap_sem);

	return ret;
}

int camera_wait_start(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();

	sem_wait(&p_cxt->start_sem);
	ret = cxt->err_code;
	return ret;
}

int camera_start_done(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;

	sem_post(&p_cxt->start_sem);

	return ret;
}

int camera_wait_set(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();

	sem_wait(&p_cxt->set_sem);
	ret = cxt->err_code;
	return ret;
}

int camera_set_done(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;

	sem_post(&p_cxt->set_sem);

	return ret;
}

int camera_wait_cap_path(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;
	int                      tmpval = 0;
	struct camera_context    *cxt = camera_get_cxt();

	sem_wait(&p_cxt->cap_path_sem);
	sem_getvalue(&p_cxt->cap_path_sem, &tmpval);
	CMR_LOGI("got cap path sem, val = %d", tmpval);
	ret = cxt->err_code;
	return ret;
}

int camera_cap_path_done(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;
	int                      tmpval = 0;

	sem_post(&p_cxt->cap_path_sem);
	sem_getvalue(&p_cxt->cap_path_sem, &tmpval);
	CMR_LOGI("post cap path sem, val = %d", tmpval);
	return ret;
}

int camera_wait_scale_path(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();

	sem_wait(&p_cxt->scale_path_sem);
	ret = cxt->err_code;
	return ret;
}

int camera_scale_path_done(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;

	sem_post(&p_cxt->scale_path_sem);
	return ret;
}

int camera_wait_convert_thum(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();

	sem_wait(&p_cxt->thum_sem);
	ret = cxt->err_code;
	return ret;
}

int camera_convert_thum_done(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;

	sem_post(&p_cxt->thum_sem);
	return ret;
}

int camera_wait_stop(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();

	sem_wait(&p_cxt->stop_sem);
	ret = cxt->err_code;

	return ret;
}

int camera_stop_done(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;

	sem_post(&p_cxt->stop_sem);

	return ret;
}

int camera_wait_init(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();

	sem_wait(&p_cxt->init_sem);
	ret = cxt->err_code;

	return ret;

}
int camera_init_done(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;

	sem_post(&p_cxt->init_sem);
	return ret;
}

int camera_wait_exit(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();

	sem_wait(&p_cxt->exit_sem);
	ret = cxt->err_code;
	return ret;

}

int camera_exit_done(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;

	sem_post(&p_cxt->exit_sem);

	return ret;
}

int camera_wait_takepicdone(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();

	sem_wait(&p_cxt->takepicdone_sem);
	ret = cxt->err_code;

	return ret;
}

int camera_takepic_done(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();
	sem_post(&p_cxt->takepicdone_sem);
	ret = cxt->err_code;

	return ret;
}

int camera_wait_takepic_callback(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();

	sem_wait(&p_cxt->takepic_callback_sem);
	ret = cxt->err_code;

	return ret;
}

int camera_takepic_callback_done(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();

	sem_post(&p_cxt->takepic_callback_sem);
	ret = cxt->err_code;

	return ret;
}

int camera_sync_scale_start(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;

	sem_wait(&p_cxt->sync_scale_sem);

	return ret;
}

int camera_sync_scale_done(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;

	sem_post(&p_cxt->sync_scale_sem);

	return ret;
}

int camera_sync_rotate_start(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;

	sem_wait(&p_cxt->sync_rotate_sem);

	return ret;
}

int camera_sync_rotate_done(struct camera_context *p_cxt)
{
	int                      ret = CAMERA_SUCCESS;

	sem_post(&p_cxt->sync_rotate_sem);

	return ret;
}

int camera_save_to_file(uint32_t index, uint32_t img_fmt,
	uint32_t width, uint32_t height, struct img_addr *addr)
{
	int                      ret = CAMERA_SUCCESS;
	char                     file_name[40];
	char                     tmp_str[10];
	FILE                     *fp = NULL;

	CMR_LOGI("index %d, format %d, width %d, heght %d",
		index, img_fmt, width, height);

	bzero(file_name, 40);
	strcpy(file_name, "/data/misc/media/");
	sprintf(tmp_str, "%d", width);
	strcat(file_name, tmp_str);
	strcat(file_name, "X");
	sprintf(tmp_str, "%d", height);
	strcat(file_name, tmp_str);

	if (IMG_DATA_TYPE_YUV420 == img_fmt ||
		IMG_DATA_TYPE_YUV422 == img_fmt) {
		strcat(file_name, "_y_");
		sprintf(tmp_str, "%d", index);
		strcat(file_name, tmp_str);
		strcat(file_name, ".raw");
		CMR_LOGI("file name %s", file_name);
		fp = fopen(file_name, "wb");

		if (NULL == fp) {
			CMR_LOGI("can not open file: %s \n", file_name);
			return 0;
		}

		fwrite((void*)addr->addr_y, 1, width * height, fp);
		fclose(fp);

		bzero(file_name, 40);
		strcpy(file_name, "/data/misc/media/");
		sprintf(tmp_str, "%d", width);
		strcat(file_name, tmp_str);
		strcat(file_name, "X");
		sprintf(tmp_str, "%d", height);
		strcat(file_name, tmp_str);
		strcat(file_name, "_uv_");
		sprintf(tmp_str, "%d", index);
		strcat(file_name, tmp_str);
		strcat(file_name, ".raw");
		CMR_LOGI("file name %s", file_name);
		fp = fopen(file_name, "wb");
		if (NULL == fp) {
			CMR_LOGI("can not open file: %s \n", file_name);
			return 0;
		}

		if (IMG_DATA_TYPE_YUV420 == img_fmt) {
			fwrite((void*)addr->addr_u, 1, width * height / 2, fp);
		} else {
			fwrite((void*)addr->addr_u, 1, width * height, fp);
		}
		fclose(fp);
	} else if (IMG_DATA_TYPE_JPEG == img_fmt) {
		strcat(file_name, "_");
		sprintf(tmp_str, "%d", index);
		strcat(file_name, tmp_str);
		strcat(file_name, ".jpg");
		CMR_LOGI("file name %s", file_name);

		fp = fopen(file_name, "wb");
		if (NULL == fp) {
			CMR_LOGI("can not open file: %s \n", file_name);
			return 0;
		}

		fwrite((void*)addr->addr_y, 1, width * height*2, fp);
		fclose(fp);
	} else if (IMG_DATA_TYPE_RAW == img_fmt) {
		strcat(file_name, "_");
		sprintf(tmp_str, "%d", index);
		strcat(file_name, tmp_str);
		strcat(file_name, ".raw");
		CMR_LOGI("file name %s", file_name);

		fp = fopen(file_name, "wb");
		if(NULL == fp){
			CMR_LOGI("can not open file: %s \n", file_name);
			return 0;
		}

		fwrite((void*)addr->addr_y, 1, (uint32_t)(width * height * 5 / 4), fp);
		fclose(fp);
	}
	return 0;
}

void camera_sensor_inf(struct sensor_context *sensor_cxt)
{
	if (SENSOR_INTERFACE_TYPE_CSI2 == sensor_cxt->sensor_info->sensor_interface.type) {
		sensor_cxt->sn_if.if_type = 1;
		sensor_cxt->sn_if.if_spec.mipi.lane_num = sensor_cxt->sensor_info->sensor_interface.bus_width;
		sensor_cxt->sn_if.if_spec.mipi.bits_per_pxl = sensor_cxt->sensor_info->sensor_interface.pixel_width;
		sensor_cxt->sn_if.if_spec.mipi.is_loose = sensor_cxt->sensor_info->sensor_interface.is_loose;
		sensor_cxt->sn_if.if_spec.mipi.use_href = 0;
		CMR_LOGI("lane_num %d, bits_per_pxl %d, is_loose %d",
			sensor_cxt->sn_if.if_spec.mipi.lane_num,
			sensor_cxt->sn_if.if_spec.mipi.bits_per_pxl,
			sensor_cxt->sn_if.if_spec.mipi.is_loose);
	} else {
		sensor_cxt->sn_if.if_type = 0;
		sensor_cxt->sn_if.if_spec.ccir.v_sync_pol = sensor_cxt->sensor_info->vsync_polarity;
		sensor_cxt->sn_if.if_spec.ccir.h_sync_pol = sensor_cxt->sensor_info->hsync_polarity;
		sensor_cxt->sn_if.if_spec.ccir.pclk_pol = sensor_cxt->sensor_info->pclk_polarity;
	}
	sensor_cxt->sn_if.frm_deci = sensor_cxt->sensor_info->preview_deci_num;
	sensor_cxt->sn_if.img_ptn = sensor_cxt->sensor_info->image_pattern;
}

int camera_set_sensormark(void)
{
	FILE                     *fp;
	uint32_t                 i = 0;
	uint32_t                 len = 0;
	uint8_t                  sensor_param[SENSOR_PARAM_NUM];
	int                      ret = CAMERA_SUCCESS;

	bzero(&sensor_param[0], sizeof(sensor_param));
	fp = fopen(SENSOR_PARA, "rb+");
	if (NULL == fp) {
		fp = fopen(SENSOR_PARA,"wb+");
		if (NULL == fp) {
			CMR_LOGE("open file error: %s",SENSOR_PARA);
		}
		memset(&sensor_param[0], 0xFF, SENSOR_PARAM_NUM);
	}

	if (NULL != fp) {
		len = fread(sensor_param, 1, SENSOR_PARAM_NUM, fp);
		CMR_LOGI(":param len %d", len);
		CMR_LOGI("sensor param: %d, %d, %d, %d, %d, %d, %d, %d",
			sensor_param[0], sensor_param[1], sensor_param[2], sensor_param[3],
			sensor_param[4], sensor_param[5], sensor_param[6], sensor_param[7]);
		fclose(fp);
	}

	Sensor_SetMark(sensor_param);

	return CAMERA_SUCCESS;
}

int camera_save_sensormark(void)
{
	FILE                     *fp;
	uint32_t                 i = 0;
	uint8_t                  is_saving= 0;
	uint8_t                  sensor_param[SENSOR_PARAM_NUM];
	int                      ret = CAMERA_SUCCESS;

	memset(&sensor_param[0], 0, SENSOR_PARAM_NUM);

	Sensor_GetMark(sensor_param, &is_saving);

	if (is_saving) {
		fp = fopen(SENSOR_PARA,"wb+");
		if (NULL != fp) {
			fwrite(sensor_param, 1, SENSOR_PARAM_NUM, fp);
			CMR_LOGI("sensor param: %d, %d, %d, %d, %d, %d, %d, %d",
				sensor_param[0], sensor_param[1], sensor_param[2], sensor_param[3],
				sensor_param[4], sensor_param[5], sensor_param[6], sensor_param[7]);
			fclose(fp);

		} else {
			CMR_LOGW("can not create SENSOR_PARA");
		}
	}

	return ret;
}
