/*
 * Copyright (C) 2008 The Android Open Source Project
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
 
#include "SprdOEMCamera.h"
#include "cmr_oem.h"


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
	uint32_t trim_width = src_trim_rect->width;
	uint32_t trim_height = src_trim_rect->height;
	uint32_t zoom_step_w = 0, zoom_step_h = 0;

	if (NULL == src_trim_rect || NULL == dst_size) {
		return -CAMERA_INVALID_PARM;
	}

	if (0 == dst_size->width || 0 == dst_size->height) {
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
	trim_width  = trim_width - zoom_step_w;
	trim_height = trim_height - zoom_step_h;

	src_trim_rect->start_x += (src_trim_rect->width - trim_width) >> 1;
	src_trim_rect->start_y += (src_trim_rect->height - trim_height) >> 1;
	src_trim_rect->start_x = CAMERA_WIDTH(src_trim_rect->start_x);
	src_trim_rect->start_y = CAMERA_HEIGHT(src_trim_rect->start_y);
	src_trim_rect->width  = CAMERA_WIDTH(trim_width);
	src_trim_rect->height = CAMERA_HEIGHT(trim_height);
	CMR_LOGV("zoom_level %d, trim rect, %d %d %d %d",
		zoom_level,
		src_trim_rect->start_x,
		src_trim_rect->start_y,
		src_trim_rect->width,
		src_trim_rect->height);
	return CAMERA_SUCCESS;	
}


