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

#include "cmr_set.h"
#include "sensor_drv_u.h"
#include "cmr_v4l2.h"

#define DV_FLASH_ON_DV_WITH_PREVIEW 1
#define IS_NEED_FLASH(x,y) ((x)&&((CAMERA_NORMAL_MODE == (y)) || \
		(CAMERA_NORMAL_CONTINUE_SHOT_MODE == (y))))
#define IS_NEED_FLASH_HILIGHT(x,y) ((x)&&((CAMERA_NORMAL_CONTINUE_SHOT_MODE == (y))))
#define ISP_PROCESS_SEC_TIMEOUT (2)
#define ISP_PROCESS_NSEC_TIMEOUT (800000000)
#define SUPPORT_CAMERA_SUM 2

static uint32_t s_cam_orientation[SUPPORT_CAMERA_SUM];

static uint32_t camera_flash_mode_to_status(enum cmr_flash_mode f_mode);
static int camera_set_brightness(uint32_t brightness, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_contrast(uint32_t contrast, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_saturation(uint32_t saturation, uint32_t *skip_mode, uint32_t *skip_num);
int camera_set_sharpness(uint32_t sharpness, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_effect(uint32_t effect, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_ev(uint32_t expo_compen, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_wb(uint32_t wb_mode, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_scene(uint32_t scene_mode, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_night(uint32_t night_mode, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_flicker(uint32_t flicker_mode, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_iso(uint32_t iso, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_flash(uint32_t flash_mode, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_video_mode(uint32_t mode, uint32_t frame_rate, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_get_video_mode(uint32_t frame_rate, uint32_t *video_mode);
static int camera_set_preview_env(uint32_t mode, uint32_t frame_rate,uint32_t *skip_mode, uint32_t *skip_num);
static int camera_init_af_mode(uint32_t af_mode, uint32_t *skip_mode, uint32_t *skip_num);

static int camera_param_to_isp(uint32_t cmd, uint32_t in_param, uint32_t *ptr_out_param)
{
	int ret = CAMERA_SUCCESS;

	switch (cmd) {
	case ISP_CTRL_SATURATION:
	case ISP_CTRL_BRIGHTNESS:
	case ISP_CTRL_CONTRAST:
	case ISP_CTRL_EV:
		*ptr_out_param = in_param;
		break;

	case ISP_CTRL_SPECIAL_EFFECT:
		/*the effect parameters need to be confirm, isp effect is very different with sensor effect*/
		*ptr_out_param = in_param;
		break;

	case ISP_CTRL_AE_MODE:
	{
		switch (in_param)
		{
			case 0:
			case 5:
				*ptr_out_param = ISP_AUTO;
				break;

			case 2:
				*ptr_out_param = ISP_SPORT;
				break;

			case 1:
				*ptr_out_param = ISP_NIGHT;
				break;

			case 3:
				*ptr_out_param = ISP_PORTRAIT;
				break;

			case 4:
				*ptr_out_param = ISP_LANDSCAPE;
				break;
			default :
				break;
		}
		break;
	}

	case ISP_CTRL_AWB_MODE:
	{
		switch (in_param)
		{
			case 0:
				*ptr_out_param = ISP_AWB_AUTO;
				break;

			case 1:
				*ptr_out_param = ISP_AWB_INDEX1;
				break;

			case 4:
				*ptr_out_param = ISP_AWB_INDEX4;
				break;

			case 5:
				*ptr_out_param = ISP_AWB_INDEX5;
				break;

			case 6:
				*ptr_out_param = ISP_AWB_INDEX6;
				break;
			default :
				break;
		}
		break;
	}

    case ISP_CTRL_AF_MODE:
	{
		switch (in_param)
		{
			case CAMERA_FOCUS_MODE_AUTO:
				*ptr_out_param = ISP_FOCUS_TRIG;
				break;

			case CAMERA_FOCUS_MODE_AUTO_MULTI:
				*ptr_out_param = ISP_FOCUS_MULTI_ZONE;
				break;

			case CAMERA_FOCUS_MODE_MACRO:
				*ptr_out_param = ISP_FOCUS_MACRO;
				break;

			case CAMERA_FOCUS_MODE_CAF:
				*ptr_out_param = ISP_FOCUS_CONTINUE;
				break;

			default :
				*ptr_out_param = ISP_FOCUS_TRIG;
				break;
		}
		break;
	}

	case ISP_CTRL_FLICKER:
		*ptr_out_param = in_param;
		break;

	case ISP_CTRL_ISO:
		*ptr_out_param = in_param;
		break;

	default :
		break;

	}

	return ret;
}

uint32_t camera_flash_mode_to_status(enum cmr_flash_mode f_mode)
{
	struct camera_context    *cxt = camera_get_cxt();
	uint32_t                 status = FLASH_STATUS_MAX;
	int                      ret = CAMERA_SUCCESS;
	uint32_t                 autoflash = 0;

	switch (f_mode) {
	case CAMERA_FLASH_MODE_OFF:
		status = FLASH_CLOSE;
		break;

	case CAMERA_FLASH_MODE_ON:
		status = FLASH_OPEN;
		break;

	case CAMERA_FLASH_MODE_TORCH:
		#ifdef DV_FLASH_ON_DV_WITH_PREVIEW
		status = FLASH_TORCH;
		#else
		status = FLASH_OPEN_ON_RECORDING;
		#endif
		break;

	case CAMERA_FLASH_MODE_AUTO:
		if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
			ret = isp_capability(ISP_LOW_LUX_EB, (void *)&autoflash);
			CMR_LOGI("isp auto flash value is %d", autoflash);
		} else {
			ret = Sensor_Ioctl(SENSOR_IOCTL_FLASH, (uint32_t)&autoflash);
			CMR_LOGI("yuv auto flash value is %d", autoflash);
		}
		if (ret) {
			autoflash = 1;
			CMR_LOGW("Failed to read auto flash mode, %d", ret);
		}
		if (autoflash) {
			status = FLASH_OPEN;
		} else {
			status = FLASH_CLOSE;
		}
		cxt->cmr_set.auto_flash_status = autoflash;
		break;

	default:
		break;
	}

	return status;
}

int camera_set_ae(uint32_t ae_mode, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGD ("ae mode %d\n", ae_mode);

	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		ret = isp_ioctl(ISP_CTRL_AE_MODE,(void *)&ae_mode);
	} else {
		CMR_LOGW ("set ae: sensor not support\n");
		ret = CAMERA_NOT_SUPPORTED;
	}

	return ret;
}

int camera_set_ae_measure_lum(uint32_t meas_lum_mode, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGD ("ae measure lum mode %d\n", meas_lum_mode);

	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		ret = isp_ioctl(ISP_CTRL_AE_MEASURE_LUM, (void *)&meas_lum_mode);
	} else {
		CMR_LOGW ("set ae measure lum: sensor not support\n");
		ret = CAMERA_NOT_SUPPORTED;
	}

	return ret;
}

int camera_set_ae_metering_area(uint32_t *win_ptr)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;
	uint32_t                 zone_cnt = *win_ptr;
	struct isp_trim_size     trim_size;

	CMR_LOGI("zone_cnt %d, x y w h, %d %d %d %d", zone_cnt, win_ptr[1], win_ptr[2], win_ptr[3], win_ptr[4]);

	if (0 == zone_cnt) {
		CMR_LOGE ("zone_cnt = 0, no metering area \n");
		return ret;
	}

	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		trim_size.x = win_ptr[1];
		trim_size.y = win_ptr[2];
		trim_size.w = win_ptr[3];
		trim_size.h = win_ptr[4];
		ret = isp_ioctl(ISP_CTRL_AE_TOUCH, (void *)&trim_size);
	} else {
		CMR_LOGW ("camera_set_ae_metering_area: sensor not support\n");
		ret = CAMERA_NOT_SUPPORTED;
	}

	return ret;
}

/*ae work mode: normal mode, fase mode, disable(bypass)*/
int camera_set_alg(uint32_t ae_work_mode, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGD ("ae alg mode %d\n", ae_work_mode);

	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		ret = isp_ioctl(ISP_CTRL_AE_MEASURE_LUM, (void *)&ae_work_mode);
	} else {
		CMR_LOGW ("set alg:sensor not support\n");
		ret = CAMERA_NOT_SUPPORTED;
	}

	return ret;
}

/*HDR mode: low/high/disable(bypass)*/
int camera_set_hdr(uint32_t hdr_mode, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGD ("hdr mode %d", hdr_mode);

	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		ret = isp_ioctl(ISP_CTRL_HDR, (void *)&hdr_mode);
	} else {
		CMR_LOGW ("set hdr:sensor not support\n");
		ret = CAMERA_NOT_SUPPORTED;
	}
	return ret;
}

int camera_set_brightness(uint32_t brightness, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;
	uint32_t isp_param = 0;

	CMR_LOGD ("brightness %d", brightness);
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		camera_param_to_isp(ISP_CTRL_BRIGHTNESS, brightness, &isp_param);
		ret = isp_ioctl(ISP_CTRL_BRIGHTNESS, (void *)&isp_param);
	} else {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		ret = Sensor_Ioctl(SENSOR_IOCTL_BRIGHTNESS, brightness);
	}
	Sensor_SetSensorExifInfo(SENSOR_EXIF_CTRL_BRIGHTNESSVALUE, brightness);

	return ret;
}

int camera_set_contrast(uint32_t contrast, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;
	uint32_t isp_param = 0;

	CMR_LOGD ("contrast %d", contrast);
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		camera_param_to_isp(ISP_CTRL_CONTRAST, contrast, &isp_param);
		ret = isp_ioctl(ISP_CTRL_CONTRAST, (void *)&isp_param);
	} else {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		ret = Sensor_Ioctl(SENSOR_IOCTL_CONTRAST, contrast);
	}
	Sensor_SetSensorExifInfo(SENSOR_EXIF_CTRL_CONTRAST, contrast);

	return ret;
}

int camera_set_saturation(uint32_t saturation, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;
	uint32_t                 isp_param = 0;

	CMR_LOGD ("saturation %d", saturation);
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		camera_param_to_isp(ISP_CTRL_SATURATION, saturation, &isp_param);
		ret = isp_ioctl(ISP_CTRL_SATURATION, (void *)&isp_param);
	} else {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		ret = Sensor_Ioctl(SENSOR_IOCTL_SATURATION, saturation);
	}
	Sensor_SetSensorExifInfo(SENSOR_EXIF_CTRL_SATURATION, saturation);

	CMR_LOGD ("skip_mode=%d, skip_num=%d", *skip_mode, *skip_num);

	return ret;
}

int camera_set_sharpness(uint32_t sharpness, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;
	uint32_t                 isp_param = 0;

	CMR_LOGD ("sharpness %d", sharpness);
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		/*camera_param_to_isp(ISP_CTRL_SHARPNESS, sharpness, &isp_param);*/
		ret = isp_ioctl(ISP_CTRL_SHARPNESS, (void *)&sharpness);
	} else {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		ret = Sensor_Ioctl(SENSOR_IOCTL_SHARPNESS, sharpness);
	}
	Sensor_SetSensorExifInfo(SENSOR_EXIF_CTRL_SHARPNESS, sharpness);

	return ret;
}

int camera_set_effect(uint32_t effect, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;
	uint32_t isp_param = 0;

	CMR_LOGD ("effect %d", effect);
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		camera_param_to_isp(ISP_CTRL_SPECIAL_EFFECT, effect, &isp_param);
		ret = isp_ioctl(ISP_CTRL_SPECIAL_EFFECT, (void *)&isp_param);
	} else {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		ret = Sensor_Ioctl(SENSOR_IOCTL_IMAGE_EFFECT, effect);
	}

	return ret;
}

int camera_set_ev(uint32_t expo_compen, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;
	uint32_t isp_param = 0;

	CMR_LOGD ("expo_compen %d", expo_compen);
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		camera_param_to_isp(ISP_CTRL_EV, expo_compen, &isp_param);
		ret = isp_ioctl(ISP_CTRL_EV, (void *)&isp_param);
	} else {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		ret = Sensor_Ioctl(SENSOR_IOCTL_EXPOSURE_COMPENSATION, expo_compen);
	}

	return ret;
}

int camera_set_wb(uint32_t wb_mode, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;
	uint32_t isp_param = 0;

	CMR_LOGD ("wb_mode %d", wb_mode);
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		camera_param_to_isp(ISP_CTRL_AWB_MODE, wb_mode, &isp_param);
		ret = isp_ioctl(ISP_CTRL_AWB_MODE, (void *)&isp_param);
	} else {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		ret = Sensor_Ioctl(SENSOR_IOCTL_SET_WB_MODE, wb_mode);
	}
	Sensor_SetSensorExifInfo(SENSOR_EXIF_CTRL_LIGHTSOURCE, wb_mode);
	Sensor_SetSensorExifInfo(SENSOR_EXIF_CTRL_WHITEBALANCE, wb_mode);

	return ret;
}

int camera_set_scene(uint32_t scene_mode, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;
	uint32_t                 isp_param = 0;

	CMR_LOGD("scene_mode %d", scene_mode);
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = 0;
		camera_param_to_isp(ISP_CTRL_AE_MODE, scene_mode, &isp_param);
		ret = isp_ioctl(ISP_CTRL_AE_MODE,(void *)&isp_param);
	} else {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		ret = Sensor_Ioctl(SENSOR_IOCTL_PREVIEWMODE, scene_mode);
	}
	Sensor_SetSensorExifInfo(SENSOR_EXIF_CTRL_SCENECAPTURETYPE,scene_mode);

	return ret;
}

int camera_set_night(uint32_t night_mode, uint32_t *skip_mode, uint32_t *skip_num)
{
	/*struct camera_context    *cxt = camera_get_cxt();*/
	int                      ret = CAMERA_SUCCESS;

	return ret;
}

int camera_set_flicker(uint32_t flicker_mode, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;
	uint32_t isp_param = 0;

	CMR_LOGD("flicker_mode %d", flicker_mode);
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		camera_param_to_isp(ISP_CTRL_FLICKER, flicker_mode, &isp_param);
		ret = isp_ioctl(ISP_CTRL_FLICKER, (void *)&isp_param);
	} else {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		ret = Sensor_Ioctl(SENSOR_IOCTL_ANTI_BANDING_FLICKER, flicker_mode);
	}

	return ret;
}

uint32_t camera_convert_iso(uint32_t iso)
{
	struct camera_context    *cxt = camera_get_cxt();
	uint32_t isp_param = 0;
	uint32_t convert_iso = 100;

	switch(iso) {
	case 0:
		if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
			convert_iso = isp_capability(ISP_CUR_ISO,(void *)&isp_param);
		}
		break;

	case 2:
		convert_iso = 200;
		break;

	case 3:
		convert_iso = 400;
		break;

	case 4:
		convert_iso = 800;
		break;

	case 5:
		convert_iso = 1600;
		break;

	default:
		break;
	}
	return convert_iso;
}

int camera_set_iso(uint32_t iso, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;
	uint32_t isp_param = 0;

	CMR_LOGD("iso %d", iso);
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = 0;
		camera_param_to_isp(ISP_CTRL_ISO, iso, &isp_param);
		ret = isp_ioctl(ISP_CTRL_ISO, (void *)&isp_param);
	} else {
		*skip_mode = IMG_SKIP_SW;
		*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		ret = Sensor_Ioctl(SENSOR_IOCTL_ISO, iso);
	}
	Sensor_SetSensorExifInfo(SENSOR_EXIF_CTRL_ISOSPEEDRATINGS,camera_convert_iso(iso));
	return ret;
}

int camera_set_flash(uint32_t flash_mode, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	uint32_t                 status = FLASH_STATUS_MAX;
	int                      ret = CAMERA_SUCCESS;

	(void)skip_mode; (void)skip_num;
	status = camera_flash_mode_to_status(flash_mode);
	if (status != cxt->cmr_set.flash) {
		if (FLASH_CLOSE == status || FLASH_TORCH == status)
		ret = camera_set_flashdevice(status);
		cxt->cmr_set.flash = status;
	}

	CMR_LOGI("ret %d, flash %d, flash_mode %d", ret, cxt->cmr_set.flash, flash_mode);
	return ret;
}

int camera_set_video_mode(uint32_t mode, uint32_t frame_rate,uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;
	uint32_t                 isp_param = 0;

	CMR_LOGD("preview mode %d", mode);
	*skip_mode = IMG_SKIP_SW;
	*skip_num = 0;

	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		isp_param = frame_rate;
		*skip_num = 0;
		CMR_LOGD("frame rate:%d.",isp_param);
		ret = isp_ioctl(ISP_CTRL_VIDEO_MODE, (void *)&isp_param);
		if ((0x00 == frame_rate) && (1 != cxt->is_dv_mode)) {
			CMR_LOGD("frame rate is auto mode, need not set!");
		} else {
			camera_param_to_isp(ISP_CTRL_ISO, 5, &isp_param);
			ret = isp_ioctl(ISP_CTRL_ISO, (void *)&isp_param);
		}
	}

	*skip_num = cxt->sn_cxt.sensor_info->change_setting_skip_num;
	ret = Sensor_Ioctl(SENSOR_IOCTL_VIDEO_MODE, mode);
	cxt->cmr_set.sensor_mode = cxt->sn_cxt.preview_mode;

	return ret;
}

int camera_set_preview_env(uint32_t mode, uint32_t frame_rate,uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;
	uint32_t                 isp_param = 0;

	CMR_LOGD("preview env %d", mode);
	*skip_mode = IMG_SKIP_SW;
	*skip_num = 0;
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		isp_param = mode;
		*skip_num = 0;
		CMR_LOGD("frame rate:%d.",isp_param);
		ret = isp_ioctl(ISP_CTRL_VIDEO_MODE, (void *)&isp_param);
		camera_param_to_isp(ISP_CTRL_ISO, 5, &isp_param);
		ret = isp_ioctl(ISP_CTRL_ISO, (void *)&isp_param);
	}
	return ret;
}

int camera_flash_process(uint32_t on_off)
{
	struct camera_context    *cxt = camera_get_cxt();
	uint32_t                 status = on_off ? FLASH_TORCH : FLASH_CLOSE_AFTER_OPEN;

	CMR_LOGD("status %d,flash %d.", status, cxt->cmr_set.flash);

	if ((FLASH_OPEN_ON_RECORDING == cxt->cmr_set.flash)
		&& (CAMERA_PREVIEW_MODE_MOVIE == cxt->cmr_set.video_mode)){
		camera_set_flashdevice(status);
	} else if (FLASH_TORCH == cxt->cmr_set.flash) {
		camera_set_flashdevice(status);
	}

	return CAMERA_SUCCESS;
}

int camera_setting_init(void)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	memset((void*)&cxt->cmr_set, INVALID_SET_BYTE,
		offsetof(struct camera_settings, set_end));

	CMR_LOGD("0x%x 0x%x 0x%x", cxt->cmr_set.video_mode,
		cxt->cmr_set.set_end,
		cxt->cmr_set.af_cancelled);

	cxt->cmr_set.isp_alg_timeout = 0;
	cxt->cmr_set.isp_ae_stab_timeout = 0;
	cxt->cmr_set.isp_af_timeout = 0;
	pthread_mutex_init (&cxt->cmr_set.isp_af_mutex, NULL);
	pthread_mutex_init (&cxt->cmr_set.set_mutex, NULL);
	pthread_mutex_init (&cxt->cmr_set.isp_alg_mutex, NULL);
	pthread_mutex_init (&cxt->cmr_set.isp_ae_stab_mutex, NULL);
	sem_init(&cxt->cmr_set.isp_af_sem, 0, 0);
	sem_init(&cxt->cmr_set.isp_alg_sem, 0, 0);
	sem_init(&cxt->cmr_set.isp_ae_stab_sem, 0, 0);

	return ret;
}

int camera_setting_deinit(void)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	cxt->cmr_set.isp_alg_timeout = 0;
	cxt->cmr_set.isp_ae_stab_timeout = 0;
	cxt->cmr_set.isp_af_timeout = 0;
	sem_destroy(&cxt->cmr_set.isp_af_sem);
	sem_destroy(&cxt->cmr_set.isp_alg_sem);
	sem_destroy(&cxt->cmr_set.isp_ae_stab_sem);
	pthread_mutex_destroy(&cxt->cmr_set.set_mutex);
	pthread_mutex_destroy(&cxt->cmr_set.isp_alg_mutex);
	pthread_mutex_destroy(&cxt->cmr_set.isp_ae_stab_mutex);
	pthread_mutex_destroy(&cxt->cmr_set.isp_af_mutex);

	return ret;
}

int camera_preview_start_set(void)
{
	struct camera_context    *cxt = camera_get_cxt();
	struct camera_settings   *set = &cxt->cmr_set;
	uint32_t                 skip, skip_num;
	int                      ret = CAMERA_SUCCESS;
	uint32_t                 sn_mode;
	uint32_t                 video_mode;
	uint32_t                 skip_mode, skip_number;

	if ((CAMERA_ZSL_MODE != cxt->cap_mode)
		&& (CAMERA_ZSL_CONTINUE_SHOT_MODE != cxt->cap_mode)
		&& (CAMERA_ANDROID_ZSL_MODE != cxt->cap_mode)) {
		sn_mode = cxt->sn_cxt.preview_mode;
	} else {
		sn_mode = cxt->sn_cxt.capture_mode;
	}
	CMR_LOGI("Sensor workmode %d", sn_mode);
	ret = Sensor_SetMode(sn_mode);
	if (ret) {
		CMR_LOGW("Sensor can't work at this mode %d", sn_mode);
		goto exit;
	}

	ret = camera_ae_enable(1);
	if (ret) {
		CMR_LOGW("ae enable fail");
		goto exit;
	}

	camera_init_af_mode(cxt->cmr_set.af_mode, &skip_mode, &skip_number);

	/*ret = Sensor_StreamOff();//wait for set mode done*/
	ret = Sensor_SetMode_WaitDone();
	if (ret) {
		CMR_LOGE("Fail to switch off the sensor stream");
		goto exit;
	}

	if (INVALID_SET_WORD != set->brightness) {
		ret = camera_set_brightness(set->brightness, &skip, &skip_num);
		CMR_RTN_IF_ERR(ret);
	}

	if (INVALID_SET_WORD != set->contrast) {
		ret = camera_set_contrast(set->contrast, &skip, &skip_num);
		CMR_RTN_IF_ERR(ret);
	}

	if (INVALID_SET_WORD != set->effect) {
		ret = camera_set_effect(set->effect, &skip, &skip_num);
		CMR_RTN_IF_ERR(ret);
	}

	if (INVALID_SET_WORD != set->expo_compen) {
		ret = camera_set_ev(set->expo_compen, &skip, &skip_num);
		CMR_RTN_IF_ERR(ret);
	}

	if (INVALID_SET_WORD != set->wb_mode) {
		ret = camera_set_wb(set->wb_mode, &skip, &skip_num);
		CMR_RTN_IF_ERR(ret);
	}

	if (INVALID_SET_WORD != set->flicker_mode) {
		ret = camera_set_flicker(set->flicker_mode, &skip, &skip_num);
		CMR_RTN_IF_ERR(ret);
	}

	if (INVALID_SET_WORD != set->iso) {
		ret = camera_set_iso(set->iso, &skip, &skip_num);
		CMR_RTN_IF_ERR(ret);
	}

	/*the flicker should be set after iso, because some mode will set the iso level*/
	if (INVALID_SET_WORD != set->scene_mode) {
		ret = camera_set_scene(set->scene_mode, &skip, &skip_num);
		CMR_RTN_IF_ERR(ret);
	}

	if (INVALID_SET_WORD != set->video_mode) {
		ret = camera_get_video_mode(set->frame_rate,&video_mode);
		CMR_RTN_IF_ERR(ret);
		ret = camera_set_video_mode(video_mode, set->frame_rate, &skip, &skip_num);
		CMR_RTN_IF_ERR(ret);
	}

	ret = camera_flash_process(1);
exit:
	return ret;
}

int camera_preview_stop_set(void)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();
	/*Todo something if necessary after preview stopped*/
	CMR_LOGD("flash process %d.",cxt->is_dv_mode);
	if (1 != cxt->is_dv_mode) {
		ret = camera_flash_process(0);
	}
	return ret;
}

int camera_set_flash_hightlight(uint32_t flash_mode)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();

	if (IS_NEED_FLASH_HILIGHT(cxt->cmr_set.flash,cxt->cap_mode)) {
		/*open flash*/
		camera_set_flashdevice((uint32_t)flash_mode);
	}
	return ret;
}

int camera_snapshot_start_set(void)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();

	/*if (V4L2_SENSOR_FORMAT_RAWRGB != cxt->sn_cxt.sn_if.img_fmt) {
		if (CAMERA_FLASH_MODE_AUTO == cxt->cmr_set.flash_mode) {
			uint32_t skip_mode = 0;
			uint32_t skip_number = 0;
			ret = camera_set_flash(cxt->cmr_set.flash_mode, &skip_mode, &skip_number);
		}
	}*/

	if (IS_NEED_FLASH(cxt->cmr_set.flash,cxt->cap_mode)) {
		/*open flash*/
		if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
			ret = isp_ioctl(ISP_CTRL_FLASH_EG,0);
			if (CAMERA_SUCCESS != ret) {
				CMR_LOGE("ISP_CTRL_FLASH_EG error.");
			}
			camera_set_flashdevice((uint32_t)FLASH_HIGH_LIGHT);
		} else {
			camera_set_flashdevice((uint32_t)FLASH_HIGH_LIGHT);
		}
	} else if ((CAMERA_FLASH_MODE_AUTO == cxt->cmr_set.flash_mode)
		&& ((uint32_t)FLASH_OPEN == cxt->cmr_set.auto_flash_status)) {
		camera_set_flashdevice((uint32_t)FLASH_HIGH_LIGHT);
	}

	if (camera_get_is_nonzsl()) {
		ret = Sensor_Ioctl(SENSOR_IOCTL_BEFORE_SNAPSHOT, (cxt->sn_cxt.capture_mode | (cxt->sn_cxt.preview_mode<<CAP_MODE_BITS)));
		if (ret) {
			CMR_LOGW("Sensor can't work at this mode %d", cxt->sn_cxt.capture_mode);
		}
		if (CAMERA_HDR_MODE == cxt->cap_mode) {
			switch (cxt->cap_cnt) {
			case 0:
				ret = camera_set_hdr_ev(SENSOR_HDR_EV_LEVE_0);
				break;

			case 1:
				ret = camera_set_hdr_ev(SENSOR_HDR_EV_LEVE_1);
				break;

			case 2:
				ret = camera_set_hdr_ev(SENSOR_HDR_EV_LEVE_2);
				break;

			default:
				ret = camera_set_hdr_ev(SENSOR_HDR_EV_LEVE_0);
				break;
			}
		}
	}
	return ret;
}

int camera_snapshot_stop_set(void)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();

	if (IS_NEED_FLASH(cxt->cmr_set.flash,cxt->cap_mode)) {
		/*open flash*/
		if ((uint32_t)CAMERA_FLASH_MODE_TORCH != cxt->cmr_set.flash) {
			camera_set_flashdevice((uint32_t)FLASH_CLOSE_AFTER_OPEN);
		}
	} else if (((uint32_t)CAMERA_FLASH_MODE_AUTO == cxt->cmr_set.flash_mode)
		&& ((uint32_t)FLASH_OPEN == cxt->cmr_set.auto_flash_status)) {
		camera_set_flashdevice((uint32_t)FLASH_CLOSE_AFTER_OPEN);
		cxt->cmr_set.auto_flash_status = FLASH_CLOSE;
	}
	if ((CAMERA_NORMAL_MODE == cxt->cap_mode) || (CAMERA_HDR_MODE == cxt->cap_mode)
		|| (CAMERA_NORMAL_CONTINUE_SHOT_MODE == cxt->cap_mode)) {
		ret = Sensor_Ioctl(SENSOR_IOCTL_AFTER_SNAPSHOT, cxt->sn_cxt.preview_mode);
		if (ret) {
			CMR_LOGW("Sensor can't work at this mode %d", cxt->sn_cxt.preview_mode);
		}
		if (CAMERA_HDR_MODE == cxt->cap_mode) {
			camera_set_hdr_ev(SENSOR_HDR_EV_LEVE_1);
			cxt->cap_cnt_for_err = 0;
		}
	}
	return ret;
}

int camera_autofocus_init(void)
{
	int                      ret = CAMERA_SUCCESS;

	ret = Sensor_AutoFocusInit();

	return ret;
}

void camera_set_rot_angle(uint32_t *angle)
{
	struct camera_context *cxt = camera_get_cxt();
	uint32_t temp_angle = *angle;
	uint32_t camera_id = cxt->sn_cxt.cur_id;

#ifdef CONFIG_FRONT_CAMERA_ROTATION
	s_cam_orientation[1] = 1;/*need to rotate*/
#else
	s_cam_orientation[1] = 0;
#endif

#ifdef CONFIG_BACK_CAMERA_ROTATION
	s_cam_orientation[0] = 1;/*need to rotate*/
#else
	s_cam_orientation[0] = 0;
#endif

	CMR_LOGD("front cam orientation %d,back cam orientation %d.orientation %d.",
		s_cam_orientation[1],s_cam_orientation[0],cxt->orientation);

	if (camera_id >= SUPPORT_CAMERA_SUM) {
		CMR_LOGE("dont support.");
		return;
	}
	if ((0 == s_cam_orientation[camera_id])&&(0 == cxt->orientation)) {
		return;
	}
	if (s_cam_orientation[camera_id]) {
		switch(temp_angle){
		case 0:
			*angle = IMG_ROT_90;
			break;

		case 90:
			*angle = IMG_ROT_180;
			break;

		case 180:
			*angle = IMG_ROT_270;
			break;

		case 270:
			*angle = 0;
			break;

		default:
			break;
		}
	} else {
		switch(temp_angle){
		case 0:
			*angle = IMG_ROT_0;
			break;

		case 90:
			*angle = IMG_ROT_90;
			break;

		case 180:
			*angle = IMG_ROT_180;
			break;

		case 270:
			*angle = IMG_ROT_270;
			break;

		default:
			break;
		}
	}
	CMR_LOGD("angle=%d.\n",*angle);
}

int camera_set_ctrl(camera_parm_type id,
			uint32_t parm,
			cmr_before_set_cb before_set,
			cmr_after_set_cb after_set)
{
	struct camera_context    *cxt = camera_get_cxt();
	uint32_t                 skip_mode, skip_number;
	int                      ret = CAMERA_SUCCESS;

	if((CAMERA_PARM_ZOOM != id)
		&& (CAMERA_PARM_FOCUS_RECT != id)
		&& (CAMERA_PARM_FLASH != id)
		&& (CAMERA_PARM_AF_MODE != id)
		&& (CAMERA_PARM_ENCODE_ROTATION != id)
		&& (CAMERA_PARM_BRIGHTNESS != id)
		&& (CAMERA_PARM_CONTRAST != id)
		&& (CAMERA_PARM_EFFECT != id)
		&& (CAMERA_PARM_SCENE_MODE != id)
		&& (CAMERA_PARM_ANTIBANDING != id)
		&& (CAMERA_PARM_WB != id)
		&& (CAMERA_PARM_EXPOSURE_COMPENSATION != id)
		&& (CAMERA_PARM_FOCAL_LENGTH != id)
		&& (CAMERA_PARM_ISO != id)
		&& (CAMERA_PARM_SENSOR_ROTATION != id)
		&& (CAMERA_PARM_ORIENTATION != id)
		&& (CAMERA_PARM_PREVIEW_MODE != id)
		&& (CAMERA_PARM_THUMBCOMP != id)
		&& (CAMERA_PARM_JPEGCOMP != id)
		&& (CAMERA_PARM_DCDV_MODE != id)
		&& (CAMERA_PARM_SHOT_NUM != id)
		&& (CAMERA_PARAM_SLOWMOTION != id)
		&& (CAMERA_PARM_SATURATION != id)
		&& (CAMERA_PARM_AUTO_EXPOSURE_MODE != id)
		&& (CAMERA_PARM_EXPOSURE_METERING != id)
		&& (CAMERA_PARM_SHARPNESS != id)
		&& (CAMERA_PARAM_ROTATION_CAPTURE != id)
		&& (CAMERA_PARM_PREVIEW_ENV != id)
		&& (CAMERA_PARM_ZOOM_RECT != id)) {
		return ret;
	}

	CMR_LOGD("ID %d or parm %d . camera preview_status %d, capture_status=%d",
		id, parm, cxt->preview_status, cxt->capture_status);

	if (id >= CAMERA_PARM_MAX || INVALID_SET_WORD == parm) {
		return CAMERA_INVALID_PARM;
	}

	switch (id) {
	case CAMERA_PARM_PREVIEW_ENV:
		cxt->cmr_set.preview_env = parm;
		CMR_LOGD("preview env %d.",parm);
		if ((CMR_PREVIEW == cxt->preview_status) &&
			(0 != cxt->cmr_set.preview_env)) {
			if (before_set) {
				ret = (*before_set)(RESTART_LIGHTLY);
				CMR_RTN_IF_ERR(ret);
			}
			ret = camera_set_preview_env(parm, cxt->cmr_set.frame_rate, &skip_mode, &skip_number);
			CMR_RTN_IF_ERR(ret);
			if (after_set) {
				ret = (*after_set)(RESTART_LIGHTLY, skip_mode, skip_number);
				CMR_RTN_IF_ERR(ret);
			}
		}
		break;

	case CAMERA_PARAM_ROTATION_CAPTURE:
		cxt->is_cfg_rot_cap = parm;
		CMR_LOGD("is_cfg_rot_cap:%d.",parm);
		break;

	case CAMERA_PARM_SHARPNESS:
		if (parm != cxt->cmr_set.sharpness) {
			if (CMR_PREVIEW == cxt->preview_status) {
				if (before_set) {
					ret = (*before_set)(RESTART_LIGHTLY);
					CMR_RTN_IF_ERR(ret);
				}
				ret = camera_set_sharpness(parm, &skip_mode, &skip_number);
				CMR_RTN_IF_ERR(ret);
				if (after_set) {
					ret = (*after_set)(RESTART_LIGHTLY, skip_mode, skip_number);
					CMR_RTN_IF_ERR(ret);
				}
			}
			cxt->cmr_set.sharpness = parm;
		}
		break;

	case CAMERA_PARM_SATURATION:
		if (parm != cxt->cmr_set.saturation) {
			if (CMR_PREVIEW == cxt->preview_status) {
				if (before_set) {
					ret = (*before_set)(RESTART_LIGHTLY);
					CMR_RTN_IF_ERR(ret);
				}
				ret = camera_set_saturation(parm, &skip_mode, &skip_number);
				CMR_RTN_IF_ERR(ret);
				if (after_set) {
					ret = (*after_set)(RESTART_LIGHTLY, skip_mode, skip_number);
					CMR_RTN_IF_ERR(ret);
				}
			}
			cxt->cmr_set.saturation = parm;
		}
		break;

	case CAMERA_PARAM_SLOWMOTION:
		cxt->cmr_set.slow_motion_mode = parm;
		CMR_LOGD("slow motion:%d.",parm);
		break;

	case CAMERA_PARM_SHOT_NUM:
		if ((0 == parm) || (9 < parm)) {
			CMR_LOGI("uncommon cap number %d.",parm);
			parm = 1;
		}
		cxt->total_capture_num = parm;
		CMR_LOGD("capture num is %d.",parm);
		break;

	case CAMERA_PARM_DCDV_MODE:
		cxt->is_dv_mode = parm;
		CMR_LOGD("camera mode %d.",parm);
		break;

	case CAMERA_PARM_EXPOSURE_COMPENSATION:
		if (parm != cxt->cmr_set.expo_compen) {
			if (CMR_PREVIEW == cxt->preview_status) {
				if (before_set) {
					ret = (*before_set)(RESTART_LIGHTLY);
					CMR_RTN_IF_ERR(ret);
				}
				ret = camera_set_ev(parm, &skip_mode, &skip_number);
				CMR_RTN_IF_ERR(ret);
				if (after_set) {
					ret = (*after_set)(RESTART_LIGHTLY, skip_mode, skip_number);
					CMR_RTN_IF_ERR(ret);
				}
			}
			cxt->cmr_set.expo_compen = parm;
		}
		break;

	case CAMERA_PARM_ENCODE_ROTATION: /* 0, 90, 180, 270 degrees */
		if (cxt->is_cfg_rot_cap) {
			uint32_t orientation;
			cxt->jpeg_cxt.set_encode_rotation = 0;
			orientation = getOrientationFromRotationDegrees(parm);
			switch (orientation) {
			case 1:
				cxt->cfg_cap_rot = IMG_ROT_0;
				break;

			case 3:
				cxt->cfg_cap_rot = IMG_ROT_180;
				break;

			case 6:
				cxt->cfg_cap_rot = IMG_ROT_90;
				break;

			case 8:
				cxt->cfg_cap_rot = IMG_ROT_270;
				break;

			default:
				cxt->cfg_cap_rot = IMG_ROT_0;
				break;
			}
		} else {
			cxt->jpeg_cxt.set_encode_rotation = parm;
			cxt->cfg_cap_rot = IMG_ROT_0;
		}
		CMR_LOGD("is_cfg_rot_cap :%d,rot:%d.",cxt->is_cfg_rot_cap,cxt->cfg_cap_rot);
		break;

	case CAMERA_PARM_SENSOR_ROTATION: /* 0, 90, 180, 270 degrees */
		if (CMR_PREVIEW == cxt->preview_status) {
			uint32_t tmp_rot = parm;
			camera_set_rot_angle(&tmp_rot);
			if (tmp_rot == cxt->prev_rot){
				CMR_LOGD("same rot setting, not changed!\n");
			} else {
				if (before_set) {
					ret = (*before_set)(RESTART_LIGHTLY);
					CMR_RTN_IF_ERR(ret);
				}
				skip_mode = IMG_SKIP_SW;
				if (SCENE_MODE_NIGHT == cxt->cmr_set.scene_mode) {
					skip_number = 3;
				} else {
					skip_number = 0;
				}
				CMR_RTN_IF_ERR(ret);
				cxt->prev_rot = parm;
				camera_set_rot_angle(&cxt->prev_rot);
				cxt->cap_rot = cxt->prev_rot;
				if (after_set) {
					ret = (*after_set)(RESTART_LIGHTLY, skip_mode, skip_number);
					CMR_RTN_IF_ERR(ret);
				}
			}
		}else {
			cxt->prev_rot = parm;
			camera_set_rot_angle(&cxt->prev_rot);
			cxt->cap_rot = cxt->prev_rot;
			cxt->cap_rot_backup = cxt->cap_rot;
		}
		break;

	case CAMERA_PARM_FOCAL_LENGTH:
		cxt->cmr_set.focal_len = (uint32_t)parm;
		break;

	case CAMERA_PARM_CONTRAST:/* contrast */
		if (parm != cxt->cmr_set.contrast) {
			if (CMR_PREVIEW == cxt->preview_status) {
				if (before_set) {
					ret = (*before_set)(RESTART_LIGHTLY);
					CMR_RTN_IF_ERR(ret);
				}
				ret = camera_set_contrast(parm, &skip_mode, &skip_number);
				CMR_RTN_IF_ERR(ret);
				if (after_set) {
					ret = (*after_set)(RESTART_LIGHTLY, skip_mode, skip_number);
					CMR_RTN_IF_ERR(ret);
				}
			}
			cxt->cmr_set.contrast = parm;
		}
		break;

	case CAMERA_PARM_BRIGHTNESS:/* brightness */
		if (parm != cxt->cmr_set.brightness) {
			if (CMR_PREVIEW == cxt->preview_status) {
				if (before_set) {
					ret = (*before_set)(RESTART_LIGHTLY);
					CMR_RTN_IF_ERR(ret);
				}
				ret = camera_set_brightness(parm, &skip_mode, &skip_number);
				CMR_RTN_IF_ERR(ret);
				if (after_set) {
					ret = (*after_set)(RESTART_LIGHTLY, skip_mode, skip_number);
					CMR_RTN_IF_ERR(ret);
				}
			}
			cxt->cmr_set.brightness = parm;
		}
		break;

	case CAMERA_PARM_WB:/* use camera_wb_type */
		if (parm != cxt->cmr_set.wb_mode) {
			if (CMR_PREVIEW == cxt->preview_status) {
				if (before_set) {
					ret = (*before_set)(RESTART_LIGHTLY);
					CMR_RTN_IF_ERR(ret);
				}
				ret = camera_set_wb(parm, &skip_mode, &skip_number);
				CMR_RTN_IF_ERR(ret);
				if (after_set) {
					ret = (*after_set)(RESTART_LIGHTLY, skip_mode, skip_number);
					CMR_RTN_IF_ERR(ret);
				}
			}
			cxt->cmr_set.wb_mode = parm;
		}
		break;

	case CAMERA_PARM_EFFECT:/* use camera_effect_type */
		if (parm != cxt->cmr_set.effect) {
			if (CMR_PREVIEW == cxt->preview_status) {
				if (before_set) {
					ret = (*before_set)(RESTART_LIGHTLY);
					CMR_RTN_IF_ERR(ret);
				}
				ret = camera_set_effect(parm, &skip_mode, &skip_number);
				CMR_RTN_IF_ERR(ret);
				if (after_set) {
					ret = (*after_set)(RESTART_LIGHTLY, skip_mode, skip_number);
					CMR_RTN_IF_ERR(ret);
				}
			}
			cxt->cmr_set.effect = parm;
		}
		break;

	case CAMERA_PARM_SCENE_MODE:/* use camera_scene_mode_type */
		if (parm != cxt->cmr_set.scene_mode) {
			if (CMR_PREVIEW == cxt->preview_status) {
				if (before_set) {
					ret = (*before_set)(RESTART_LIGHTLY);
					CMR_RTN_IF_ERR(ret);
				}
				ret = camera_set_scene(parm, &skip_mode, &skip_number);
				CMR_RTN_IF_ERR(ret);
				if (after_set) {
					ret = (*after_set)(RESTART_LIGHTLY, skip_mode, skip_number);
					CMR_RTN_IF_ERR(ret);
				}
			}
			cxt->cmr_set.scene_mode = parm;
		}
		break;

	case CAMERA_PARM_ZOOM:
		cxt->zoom_mode = (uint32_t)CAMERA_PARM_ZOOM;
		if (parm != cxt->zoom_level) {
			CMR_LOGI("Set zoom level %d", parm);
			cxt->zoom_level = parm;
			if (CMR_PREVIEW == cxt->preview_status) {
				if (before_set) {
					ret = (*before_set)(RESTART_ZOOM);
					CMR_RTN_IF_ERR(ret);
				}
				skip_mode = IMG_SKIP_HW;
				if (SCENE_MODE_NIGHT == cxt->cmr_set.scene_mode) {
					skip_number = 3;
				} else {
					skip_number = 0;
				}
				CMR_RTN_IF_ERR(ret);
				if (after_set) {
					ret = (*after_set)(RESTART_ZOOM, skip_mode, skip_number);
					CMR_RTN_IF_ERR(ret);
				}
			}
		}
		break;

	case CAMERA_PARM_ZOOM_RECT:
		{
			struct img_rect *rect_ptr = (struct img_rect *)parm;
			cxt->zoom_mode = (uint32_t)CAMERA_PARM_ZOOM_RECT;
			if (PNULL == rect_ptr) {
				CMR_LOGE("set zoom rect is null parameter.");
				break;
			}
			if ((rect_ptr->start_x != cxt->zoom_rect.start_x)
				|| (rect_ptr->start_y != cxt->zoom_rect.start_y)
				|| (rect_ptr->width != cxt->zoom_rect.width)
				|| (rect_ptr->height != cxt->zoom_rect.height)) {
				memcpy(&cxt->zoom_rect, rect_ptr, sizeof(struct img_rect));
				if ((rect_ptr->start_x != 0) || (rect_ptr->start_y != 0)) {
					cxt->zoom_level = 1;
				}
				if (CMR_PREVIEW == cxt->preview_status) {
					if (before_set) {
						ret = (*before_set)(RESTART_ZOOM_RECT);
						CMR_RTN_IF_ERR(ret);
					}
					skip_mode = IMG_SKIP_HW;
					if(SCENE_MODE_NIGHT == cxt->cmr_set.scene_mode){
						skip_number = 3;
					} else {
						skip_number = 0;
					}
					CMR_RTN_IF_ERR(ret);
					if (after_set) {
						ret = (*after_set)(RESTART_ZOOM_RECT, skip_mode, skip_number);
						CMR_RTN_IF_ERR(ret);
					}
				}
			}
		}
		break;

	case CAMERA_PARM_JPEGCOMP:
		cxt->jpeg_cxt.quality = parm;
		break;

	case CAMERA_PARM_THUMBCOMP:
		cxt->jpeg_cxt.thumb_quality = parm;
		break;

	case CAMERA_PARM_ORIENTATION:
		cxt->orientation = parm;
		CMR_LOGD("set orientation %d.",cxt->orientation);
		break;

	case CAMERA_PARM_FLASH:/* Flash control, see camera_flash_type */
		if (CAMERA_FLASH_MODE_AUTO == parm) {
			cxt->cmr_set.flash_mode = parm;
		} else {
			cxt->cmr_set.flash_mode = CAMERA_FLASH_MODE_MAX;
			ret = camera_set_flash(parm, &skip_mode, &skip_number);
		}
		break;

	case CAMERA_PARM_NIGHTSHOT_MODE:/* Night shot mode, snapshot at reduced FPS */
		cxt->cmr_set.night_mode = parm;
		break;

	case CAMERA_PARM_ANTIBANDING:/* Use camera_anti_banding_type */
		if (parm != cxt->cmr_set.flicker_mode) {
			if (CMR_PREVIEW == cxt->preview_status) {
				if (before_set) {
					ret = (*before_set)(RESTART_LIGHTLY);
					CMR_RTN_IF_ERR(ret);
				}
				ret = camera_set_flicker(parm, &skip_mode, &skip_number);
				CMR_RTN_IF_ERR(ret);
				if (after_set) {
					ret = (*after_set)(RESTART_LIGHTLY, skip_mode, skip_number);
					CMR_RTN_IF_ERR(ret);
				}
			}
			cxt->cmr_set.flicker_mode = parm;
		}
		break;

	case CAMERA_PARM_FOCUS_RECT:
		memcpy((void*)&cxt->cmr_set.focus_zone_param, (void*)parm, CAMERA_FOCUS_RECT_PARAM_LEN);
		break;

	case CAMERA_PARM_AF_MODE:
		CMR_LOGI("Set AF Mode %d", parm);
		camera_init_af_mode(parm, &skip_mode, &skip_number);
		if (CAMERA_FOCUS_MODE_CAF_VIDEO != parm) {
			cxt->cmr_set.af_mode = (uint32_t)parm;
		} else {
			CMR_LOGI("force Set AF Mode Auto");
			cxt->cmr_set.af_mode = CAMERA_FOCUS_MODE_AUTO; /*to do*/
		}
		break;

	case CAMERA_PARM_ISO:
		if (parm != cxt->cmr_set.iso) {
			if (CMR_PREVIEW == cxt->preview_status) {
				if (before_set) {
					ret = (*before_set)(RESTART_LIGHTLY);
					CMR_RTN_IF_ERR(ret);
				}
				ret = camera_set_iso(parm, &skip_mode, &skip_number);
				CMR_RTN_IF_ERR(ret);
				if (after_set) {
					ret = (*after_set)(RESTART_LIGHTLY, skip_mode, skip_number);
					CMR_RTN_IF_ERR(ret);
				}
			}
			cxt->cmr_set.iso = parm;
		}
		break;

	case CAMERA_PARM_PREVIEW_MODE:/* Use camera_preview_mode_type */
		CMR_LOGI("parm=%d, cxt->cmr_set.video_mode = %d \n", parm, cxt->cmr_set.video_mode);
		{
			uint32_t video_mode = 0;
			ret = camera_get_video_mode(parm,&video_mode);
			CMR_RTN_IF_ERR(ret);
			cxt->cmr_set.frame_rate = parm;
			CMR_LOGI("cxt->preview_status = %d \n", cxt->preview_status);
			if (CMR_PREVIEW == cxt->preview_status) {
				if (before_set) {
					ret = (*before_set)(RESTART_LIGHTLY);
					CMR_RTN_IF_ERR(ret);
				}
				ret = camera_set_video_mode(video_mode, cxt->cmr_set.frame_rate, &skip_mode, &skip_number);
				CMR_RTN_IF_ERR(ret);
				if (after_set) {
					ret = (*after_set)(RESTART_LIGHTLY, skip_mode, skip_number);
					CMR_RTN_IF_ERR(ret);
				}
			}
			cxt->cmr_set.video_mode = video_mode;
		}
		break;

	case CAMERA_PARM_AUTO_EXPOSURE_MODE:
		CMR_LOGI("CAMERA_PARM_AUTO_EXPOSURE_MODE = %d \n", parm);
		if (parm != cxt->cmr_set.auto_exposure_mode) {
			ret = camera_set_ae_measure_lum(parm, &skip_mode, &skip_number);
			cxt->cmr_set.auto_exposure_mode = parm;
		}
		break;

	case CAMERA_PARM_EXPOSURE_METERING:
		ret = camera_set_ae_metering_area((uint32_t*)parm);
		break;

	default:
		break;
	}

exit:
	return ret;
}

int camera_set_hdr_ev(int ev_level)
{
	int                      ret = CAMERA_SUCCESS;
	SENSOR_EXT_FUN_PARAM_T ev_param;

	ev_param.cmd = SENSOR_EXT_EV;
	ev_param.param = ev_level;
	CMR_LOGD("level %d.",ev_param.param);
	ret = Sensor_Ioctl(SENSOR_IOCTL_FOCUS, (uint32_t) & ev_param);
	CMR_LOGD("done %d.",ret);
	return ret;
}

int camera_preflash(void)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();

#ifdef CONFIG_CAMERA_FLASH_CTRL
	CMR_LOGD("start.");
	if (CAMERA_FLASH_MODE_AUTO == cxt->cmr_set.flash_mode) {
		uint32_t skip_mode = 0;
		uint32_t skip_number = 0;
		ret = camera_set_flash(cxt->cmr_set.flash_mode, &skip_mode, &skip_number);
	}

	if (IS_NEED_FLASH(cxt->cmr_set.flash,cxt->cap_mode)) {
		if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
			SENSOR_FLASH_LEVEL_T flash_level;
			if (Sensor_GetFlashLevel(&flash_level)) {
				CMR_LOGE("get flash level error.");
			}

			camera_isp_awb_bypass(ISP_AWB_BYPASS);
			camera_isp_ae_bypass(ISP_AE_BYPASS);

			camera_isp_alg_wait();
			camera_set_flashdevice((uint32_t)FLASH_OPEN);
			camera_isp_flash_ratio(&flash_level);
		}
	}
	if (IS_NEED_FLASH(cxt->cmr_set.flash,cxt->cap_mode)) {
		if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
			camera_isp_alg_wait();
		}
		camera_set_flashdevice((uint32_t)FLASH_CLOSE_AFTER_OPEN);
	}
#endif
	return ret;
}

int camera_caf_preflash(void)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();
	uint32_t                 af_cancel_is_ext = 0;

#ifndef CONFIG_CAMERA_FLASH_CTRL
	if (CAMERA_FLASH_MODE_AUTO == cxt->cmr_set.flash_mode) {
		uint32_t skip_mode = 0;
		uint32_t skip_number = 0;
		ret = camera_set_flash(cxt->cmr_set.flash_mode, &skip_mode, &skip_number);
	}

	if (IS_NEED_FLASH(cxt->cmr_set.flash,cxt->cap_mode)) {
		CMR_LOGI("@xin open flash");
		camera_set_flashdevice((uint32_t)FLASH_OPEN);
	}
#endif

	usleep(200*1000);

	if (IS_NEED_FLASH(cxt->cmr_set.flash,cxt->cap_mode)) {
		camera_set_flashdevice((uint32_t)FLASH_CLOSE_AFTER_AUTOFOCUS);
	}

exit:
	return ret;
}


static int camera_check_autofocus_area(SENSOR_RECT_T *rect_ptr,uint32_t rect_num)
{
	uint32_t                 ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();
	uint32_t                 sn_work_mode = 0;
	SENSOR_MODE_INFO_T       *sensor_mode;
	uint32_t                 i = 0;

	if (camera_get_is_nonzsl()) {
		sn_work_mode = cxt->sn_cxt.preview_mode;
	} else {
		sn_work_mode = cxt->sn_cxt.capture_mode;
	}
	sensor_mode = &cxt->sn_cxt.sensor_info->sensor_mode_info[sn_work_mode];

	for ( i=0 ; i<rect_num ; i++) {
		if ((0 == rect_ptr[i].w) || (0 == rect_ptr[i].h)
			|| (rect_ptr->w > sensor_mode->trim_width)
			|| (rect_ptr->h > sensor_mode->trim_height)
			|| ((rect_ptr->x+rect_ptr->w) > (sensor_mode->trim_start_x + sensor_mode->trim_width))
			|| ((rect_ptr->y+rect_ptr->h) > (sensor_mode->trim_start_y + sensor_mode->trim_height))) {
			CMR_LOGW("auto focus use auto mode.");
			ret = CAMERA_FAILED;
			break;
		}
	}
	return ret;
}

int camera_autofocus_start(void)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();
	uint32_t                 *ptr = (uint32_t*)&cxt->cmr_set.focus_zone_param[0];
	uint32_t                 i = 0;
	uint32_t                 zone_cnt = *ptr++;
	uint32_t                 af_cancel_is_ext = 0;
	struct timespec          ts;
	SENSOR_EXT_FUN_PARAM_T   af_param;
	memset(&af_param,0,sizeof(af_param));

	CMR_LOGI("zone_cnt %d, x y w h, %d %d %d %d", zone_cnt, ptr[0], ptr[1], ptr[2], ptr[3]);

	CMR_PRINT_TIME;
	if (camera_autofocus_need_exit(&af_cancel_is_ext)) {
		ret = CAMERA_INVALID_STATE;
		CMR_RTN_IF_ERR(ret);
	}

	if (0 == cxt->pre_frm_cnt) {
		CMR_LOGE("not preview frame output yet, autofocus fail");
		ret = CAMERA_FAILED;
		CMR_RTN_IF_ERR(ret);
	}
#ifndef CONFIG_CAMERA_FLASH_CTRL
	if (CAMERA_FLASH_MODE_AUTO == cxt->cmr_set.flash_mode) {
		uint32_t skip_mode = 0;
		uint32_t skip_number = 0;
		ret = camera_set_flash(cxt->cmr_set.flash_mode, &skip_mode, &skip_number);
	}

	if (IS_NEED_FLASH(cxt->cmr_set.flash,cxt->cap_mode)) {
		if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
			SENSOR_FLASH_LEVEL_T flash_level;
			if (Sensor_GetFlashLevel(&flash_level)) {
				CMR_LOGW("get flash level error.");
			}

			camera_isp_awb_bypass(ISP_AWB_BYPASS);

			camera_isp_ae_bypass(ISP_AE_BYPASS);

			if (CAMERA_SUCCESS != camera_isp_alg_wait()) {
				if (camera_autofocus_need_exit(&af_cancel_is_ext)) {
					ret = CAMERA_INVALID_STATE;
					CMR_RTN_IF_ERR(ret);
				}
				usleep(10*1000);
			}

			camera_set_flashdevice((uint32_t)FLASH_OPEN);
			camera_isp_flash_ratio(&flash_level);
		} else {
			camera_set_flashdevice((uint32_t)FLASH_OPEN);
		}
		if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
			if (CAMERA_SUCCESS != camera_isp_alg_wait()) {
				if (camera_autofocus_need_exit(&af_cancel_is_ext)) {
					ret = CAMERA_INVALID_STATE;
					camera_set_flashdevice((uint32_t)FLASH_CLOSE_AFTER_OPEN);
					CMR_RTN_IF_ERR(ret);
				}
				usleep(10*1000);
			}
		}
	}
#endif
	if (CAMERA_FOCUS_MODE_MACRO == cxt->cmr_set.af_mode) {
		af_param.cmd = SENSOR_EXT_FOCUS_START;
		af_param.param = SENSOR_EXT_FOCUS_MACRO;
		af_param.zone_cnt = zone_cnt;
		CMR_LOGI("SPRD OEM: camera_start_focus macro");
		af_param.zone_cnt = 1;
		af_param.zone[0].x = *ptr++;
		af_param.zone[0].y = *ptr++;
		af_param.zone[0].w = *ptr++;
		af_param.zone[0].h = *ptr++;
		if (CAMERA_SUCCESS != camera_check_autofocus_area(&af_param.zone[0],1)) {
			af_param.zone_cnt = 0;
		}
	} else {
		if(CAMERA_FOCUS_MODE_CAF == cxt->cmr_set.af_mode){
			af_param.cmd = SENSOR_EXT_FOCUS_START;
			af_param.param = SENSOR_EXT_FOCUS_CAF;
			af_param.zone_cnt = 0;
		} else if ((0 == zone_cnt) || (CAMERA_FOCUS_MODE_AUTO == cxt->cmr_set.af_mode)) {
			af_param.cmd = SENSOR_EXT_FOCUS_START;
			af_param.param = SENSOR_EXT_FOCUS_TRIG;
			af_param.zone_cnt = 0;
			if (zone_cnt) {
				af_param.zone_cnt = 1;
				af_param.zone[0].x = *ptr++;
				af_param.zone[0].y = *ptr++;
				af_param.zone[0].w = *ptr++;
				af_param.zone[0].h = *ptr++;
				if (CAMERA_SUCCESS != camera_check_autofocus_area(&af_param.zone[0],1)) {
					af_param.zone_cnt = 0;
				}
			}
		} else if (1 == zone_cnt) {
			af_param.cmd = SENSOR_EXT_FOCUS_START;
			af_param.param = SENSOR_EXT_FOCUS_ZONE;
			af_param.zone_cnt = 1;
			af_param.zone[0].x = *ptr++;
			af_param.zone[0].y = *ptr++;
			af_param.zone[0].w = *ptr++;
			af_param.zone[0].h = *ptr++;
			if (CAMERA_SUCCESS != camera_check_autofocus_area(&af_param.zone[0],1)) {
				af_param.cmd = SENSOR_EXT_FOCUS_START;
				af_param.param = SENSOR_EXT_FOCUS_TRIG;
				af_param.zone_cnt = 0;
			}

		} else if (zone_cnt <= FOCUS_ZONE_CNT_MAX) {
			af_param.cmd = SENSOR_EXT_FOCUS_START;
			af_param.param = SENSOR_EXT_FOCUS_MULTI_ZONE;
			af_param.zone_cnt = zone_cnt;
			for (i = 0; i < zone_cnt; i++) {
				af_param.zone[i].x = *ptr++;
				af_param.zone[i].y = *ptr++;
				af_param.zone[i].w = *ptr++;
				af_param.zone[i].h = *ptr++;
			}
			if (CAMERA_SUCCESS != camera_check_autofocus_area(&af_param.zone[0],zone_cnt)) {
				af_param.cmd = SENSOR_EXT_FOCUS_START;
				af_param.param = SENSOR_EXT_FOCUS_TRIG;
				af_param.zone_cnt = 0;
			}
		} else {
			CMR_LOGE("Unsupported zone count %d", zone_cnt);
			ret = CAMERA_NOT_SUPPORTED;
		}
	}
	CMR_LOGD("cnt %d",af_param.zone_cnt);
	if (CAMERA_SUCCESS == ret) {
		cxt->af_busy = 1;
		if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
			struct isp_af_win isp_af_param;

			memset(&isp_af_param, 0, sizeof(struct isp_af_win));
			isp_af_param.mode = af_param.param;
			isp_af_param.valid_win = af_param.zone_cnt;

			for (i = 0; i < af_param.zone_cnt; i++) {
				isp_af_param.win[i].start_x = af_param.zone[i].x;
				isp_af_param.win[i].start_y = af_param.zone[i].y;
				isp_af_param.win[i].end_x = af_param.zone[i].x + af_param.zone[i].w - 1;
				isp_af_param.win[i].end_y = af_param.zone[i].y + af_param.zone[i].h - 1;

				CMR_LOGI("af_win num:%d, x:%d y:%d e_x:%d e_y:%d",
					zone_cnt,
					isp_af_param.win[i].start_x,
					isp_af_param.win[i].start_y,
					isp_af_param.win[i].end_x,
					isp_af_param.win[i].end_y);
			}
			cxt->cmr_set.isp_af_timeout = 0;
			ret = isp_ioctl(ISP_CTRL_AF, &isp_af_param);
			if (clock_gettime(CLOCK_REALTIME, &ts)) {
				ret = -1;
				CMR_LOGW("get time failed.");
			} else {
				ts.tv_sec += ISP_PROCESS_SEC_TIMEOUT;
				if (ts.tv_nsec + ISP_PROCESS_NSEC_TIMEOUT >= 1000000000) {
					ts.tv_nsec = ts.tv_nsec + ISP_PROCESS_NSEC_TIMEOUT - 1000000000;
					ts.tv_sec ++;
				} else {
					ts.tv_nsec += ISP_PROCESS_NSEC_TIMEOUT;
				}
				ret = sem_timedwait(&cxt->cmr_set.isp_af_sem, &ts);
				if (ret) {
					pthread_mutex_lock(&cxt->cmr_set.isp_af_mutex);
					CMR_LOGW("isp af timeout");
					cxt->cmr_set.isp_af_timeout = 1;
					pthread_mutex_unlock(&cxt->cmr_set.isp_af_mutex);
					camera_autofocus_stop(0);
				} else {
					if (0 == cxt->cmr_set.isp_af_win_val) {
						ret = -1;
					}
				}
			}
		} else {
			ret = Sensor_Ioctl(SENSOR_IOCTL_FOCUS, (uint32_t) & af_param);
		}
		cxt->af_busy = 0;
	}

#ifndef CONFIG_CAMERA_FLASH_CTRL

	if (IS_NEED_FLASH(cxt->cmr_set.flash,cxt->cap_mode)) {
		if (FLASH_TORCH != cxt->cmr_set.flash) {
			camera_set_flashdevice((uint32_t)FLASH_CLOSE_AFTER_AUTOFOCUS);
		}
	}
#endif
	CMR_PRINT_TIME;
	CMR_LOGI("End. %d", ret);

	if (ret) {
		ret = CAMERA_FAILED;
	}

exit:
	return ret;
}

int camera_autofocus_start_light(void)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();
	SENSOR_EXT_FUN_PARAM_T af_param;
	memset(&af_param,0,sizeof(af_param));

	CMR_LOGI(" caf focus move start");
	if (V4L2_SENSOR_FORMAT_RAWRGB != cxt->sn_cxt.sn_if.img_fmt) {
		af_param.cmd = SENSOR_EXT_FOCUS_START;
		af_param.param = SENSOR_EXT_FOCUS_TRIG;
		af_param.zone_cnt = 0;
		ret = Sensor_Ioctl(SENSOR_IOCTL_FOCUS, (uint32_t) &af_param);
	}

	if (ret) {
		ret = CAMERA_FAILED;
	}

	return ret;
}


int camera_autofocus_quit(void)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();
	uint32_t                 skip_mode, skip_number;

	if ((CAMERA_FOCUS_MODE_CAF == cxt->cmr_set.af_mode || CAMERA_FOCUS_MODE_CAF_VIDEO == cxt->cmr_set.af_mode)
		&& (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt))
	{
		/*disable isp caf*/
		camera_init_af_mode(CAMERA_FOCUS_MODE_AUTO, &skip_mode, &skip_number);
	}

	if (!(cxt->af_busy)) {
		CMR_LOGI("autofocus is IDLE direct return!");
		return ret;
	}

	CMR_LOGI("set autofocus quit!");
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		struct isp_af_win isp_af_param;
		memset(&isp_af_param, 0, sizeof(struct isp_af_win));
		ret = isp_ioctl(ISP_CTRL_AF_STOP, &isp_af_param);
	} else {
		SENSOR_EXT_FUN_PARAM_T af_param;
		memset(&af_param,0,sizeof(af_param));
		af_param.cmd = SENSOR_EXT_FOCUS_QUIT;
		ret = Sensor_Ioctl(SENSOR_IOCTL_FOCUS, (uint32_t) &af_param);
	}

	return ret;
}

int camera_autofocus(void)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	pthread_mutex_lock(&cxt->cmr_set.set_mutex);
	cxt->cmr_set.af_cancelled = 0x00;
	pthread_mutex_unlock(&cxt->cmr_set.set_mutex);

	return ret;
}

int camera_autofocus_stop(uint32_t is_external)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;


	pthread_mutex_lock(&cxt->cmr_set.set_mutex);
	cxt->cmr_set.af_cancelled |= 0x01;
	cxt->cmr_set.af_cancelled |= (is_external & 0x1) << 1;
	cxt->cmr_set.bflash = 1;
	CMR_LOGI("af_cancelled 0x%x", cxt->cmr_set.af_cancelled);
	pthread_mutex_unlock(&cxt->cmr_set.set_mutex);

	camera_autofocus_quit();
	CMR_LOGI("quit.");
	return ret;
}

int camera_autofocus_need_exit(uint32_t *is_external)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = 0;

	pthread_mutex_lock(&cxt->cmr_set.set_mutex);
	ret = (cxt->cmr_set.af_cancelled & 0x01) == 1 ? 1 : 0;
	*is_external = (cxt->cmr_set.af_cancelled >> 1) & 0x1;
	pthread_mutex_unlock(&cxt->cmr_set.set_mutex);

	if (ret) {
		CMR_LOGI("af_cancelled val: 0x%x, 0x%x", cxt->cmr_set.af_cancelled, *is_external);
	}
	return ret;
}

int camera_init_af_mode(uint32_t af_mode, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGD ("af mode %d\n", af_mode);

	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		*skip_mode = IMG_SKIP_SW;
		*skip_num  = cxt->sn_cxt.sensor_info->change_setting_skip_num;
		camera_param_to_isp(ISP_CTRL_AF_MODE,af_mode,&af_mode);
		ret = isp_ioctl(ISP_CTRL_AF_MODE,(void *)&af_mode);
	} else {
		CMR_LOGW ("set af: sensor not support\n");
	}

	return ret;
}

int camera_set_focusmove_flag(uint32_t is_done)
{
	struct camera_context    *cxt = camera_get_cxt();

	pthread_mutex_lock(&cxt->cmr_set.set_mutex);
	cxt->cmr_set.caf_move_done= is_done;
	pthread_mutex_unlock(&cxt->cmr_set.set_mutex);

	return CAMERA_SUCCESS;
}

uint32_t camera_is_focusmove_done(void)
{
	struct camera_context    *cxt = camera_get_cxt();
	uint32_t is_move_done;

	pthread_mutex_lock(&cxt->cmr_set.set_mutex);
	is_move_done = cxt->cmr_set.caf_move_done;
	pthread_mutex_unlock(&cxt->cmr_set.set_mutex);

	return is_move_done;
}

int camera_isp_ctrl_done(uint32_t cmd, void* data)
{
	/*struct camera_context    *cxt = camera_get_cxt();*/
	int                      ret = 0;

	if (cmd >= ISP_CTRL_MAX) {
		CMR_LOGE("Wrong cmd %d", cmd);
		return CAMERA_FAILED;
	}

	CMR_LOGI("cmd, 0x%x, ret %d", cmd, ret);
	return ret;
}

int camera_isp_af_done(void *data)
{
	struct camera_context    *cxt = camera_get_cxt();
	struct isp_af_notice     *isp_af = (struct isp_af_notice*)data;


	if (NULL == isp_af) {
		CMR_LOGE("fail data is NULL");
		return -1;
	}
	pthread_mutex_lock(&cxt->cmr_set.isp_af_mutex);
	CMR_LOGI("AF done, valid_win 0x%x, isp_af_timeout %d",
		isp_af->valid_win, cxt->cmr_set.isp_af_timeout);
	cxt->cmr_set.isp_af_win_val = isp_af->valid_win;
	if (cxt->cmr_set.isp_af_timeout == 0) {
		sem_post(&cxt->cmr_set.isp_af_sem);
	}
	pthread_mutex_unlock(&cxt->cmr_set.isp_af_mutex);

	return 0;
}

int camera_isp_alg_done(void *data)
{
	struct camera_context    *cxt = camera_get_cxt();

	pthread_mutex_lock(&cxt->cmr_set.isp_alg_mutex);
	CMR_LOGI("isp ALG done.");
	if (0 == cxt->cmr_set.isp_alg_timeout) {
		sem_post(&cxt->cmr_set.isp_alg_sem);
	}
	pthread_mutex_unlock(&cxt->cmr_set.isp_alg_mutex);
	return 0;
}

int camera_isp_af_stat(void* data)
{
	struct camera_context    *cxt = camera_get_cxt();

	CMR_LOGI("isp get af stat.\n");

	return 0;
}

int camera_isp_ae_stab(void* data)
{
	struct camera_context    *cxt = camera_get_cxt();
	int tmpVal = 0;

	pthread_mutex_lock(&cxt->cmr_set.isp_ae_stab_mutex);
	CMR_LOGI("callback return.now eb: %d timeout: %d", cxt->is_isp_ae_stab_eb ,cxt->cmr_set.isp_ae_stab_timeout);
	sem_getvalue(&cxt->cmr_set.isp_ae_stab_sem, &tmpVal);
	while (0 < tmpVal) {
		sem_wait(&cxt->cmr_set.isp_ae_stab_sem);
		sem_getvalue(&cxt->cmr_set.isp_ae_stab_sem, &tmpVal);
	}

	if (cxt->is_isp_ae_stab_eb && (0 == cxt->cmr_set.isp_ae_stab_timeout)) {
		sem_post(&cxt->cmr_set.isp_ae_stab_sem);
	}
	cxt->is_isp_ae_stab_eb = 0;
	pthread_mutex_unlock(&cxt->cmr_set.isp_ae_stab_mutex);
	return 0;
}

int camera_isp_alg_wait(void)
{
	int rtn = CAMERA_SUCCESS;
	struct timespec ts;
	struct camera_context    *cxt = camera_get_cxt();

	pthread_mutex_lock(&cxt->cmr_set.isp_alg_mutex);
	if (clock_gettime(CLOCK_REALTIME, &ts)) {
		rtn = -1;
		CMR_LOGE("get time failed.");
	} else {
		ts.tv_sec += ISP_ALG_TIMEOUT;
		pthread_mutex_unlock(&cxt->cmr_set.isp_alg_mutex);
		if (sem_timedwait((&cxt->cmr_set.isp_alg_sem), &ts)) {
			pthread_mutex_lock(&cxt->cmr_set.isp_alg_mutex);
			rtn = -1;
			cxt->cmr_set.isp_alg_timeout = 1;
			CMR_LOGW("timeout.");
		} else {
			pthread_mutex_lock(&cxt->cmr_set.isp_alg_mutex);
			cxt->cmr_set.isp_alg_timeout = 0;
			CMR_LOGI("done.");
		}
	}
	pthread_mutex_unlock(&cxt->cmr_set.isp_alg_mutex);
	return rtn;
}

int camera_set_flashdevice(uint32_t param)
{
	int                      ret = 0;
	struct camera_context    *cxt = camera_get_cxt();

	CMR_LOGD("test flash:0x%x.",param);

	if (0 != cxt->camera_id) {
		CMR_LOGW("don't support flash.");
		return ret;
	}

	ret = Sensor_SetFlash(param);

	if (0 != ret) {
		CMR_LOGE("error:%d.",ret);
	}
	return ret;
}
int camera_get_video_mode(uint32_t frame_rate, uint32_t *video_mode)
{
	int                      ret = 0;
	int sensor_mode = 0;
	uint32_t                 i = 0;
	struct camera_context    *cxt = camera_get_cxt();
	SENSOR_AE_INFO_T         *sensor_ae_info;

	if (PNULL == video_mode) {
		CMR_LOGD("null pointer.");
		return CAMERA_FAILED;
	}
	*video_mode = 0;
	sensor_mode = cxt->sn_cxt.preview_mode;
	sensor_ae_info = (SENSOR_AE_INFO_T*)&cxt->sn_cxt.sensor_info->sensor_video_info[sensor_mode];
	CMR_LOGI("sensor_mode %d.",sensor_mode);
	for (i=0 ; i<SENSOR_VIDEO_MODE_MAX ; i++) {
		if (frame_rate <= sensor_ae_info[i].max_frate) {
			*video_mode = i;
			break;
		}
	}
	if (SENSOR_VIDEO_MODE_MAX == i) {
		CMR_LOGD("use default video mode");
	}
	CMR_LOGD("video mode:%d.",*video_mode);
	return ret;
}

int camera_get_af_mode(void)
{
	struct camera_context    *cxt = camera_get_cxt();

	return cxt->cmr_set.af_mode;
}

int camera_ae_enable(uint32_t param)
{
	int	ret = CAMERA_SUCCESS;

	CMR_LOGI("ae enable %d.", param);
	ret = Sensor_Ioctl(SENSOR_IOCTL_AE_ENABLE, param);
	if (ret) {
		CMR_LOGE("ae enable err.");
	}

	return ret;
}
