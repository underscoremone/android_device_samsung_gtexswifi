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

#include "cmr_set.h"
#include "sensor_drv_u.h"

enum cmr_focus_mode {
	CAMERA_FOCUS_MODE_AUTO = 0,
	CAMERA_FOCUS_MODE_AUTO_MULTI = 1,
	CAMERA_FOCUS_MODE_MACRO = 2,
	CAMERA_FOCUS_MODE_MAX
};

enum cmr_flash_mode {
	CAMERA_FLASH_MODE_OFF = 0,
	CAMERA_FLASH_MODE_ON = 1,
	CAMERA_FLASH_MODE_TORCH = 2,
	CAMERA_FLASH_MODE_MAX
};

enum cmr_flash_status {
	FLASH_CLOSE = 0x0,
	FLASH_OPEN = 0x1,
	FLASH_TORCH = 0x2,	/*user only set flash to close/open/torch state */
	FLASH_CLOSE_AFTER_OPEN = 0x10,	/* following is set to sensor */
	FLASH_HIGH_LIGHT = 0x11,
	FLASH_OPEN_ON_RECORDING = 0x22,
	FLASH_STATUS_MAX
};

static int camera_autofocus_need_exit(void);
static uint32_t camera_flash_mode_to_status(enum cmr_flash_mode f_mode);
static int camera_set_brightness(uint32_t brightness, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_contrast(uint32_t contrast, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_effect(uint32_t effect, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_ev(uint32_t expo_compen, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_wb(uint32_t wb_mode, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_scene(uint32_t scene_mode, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_night(uint32_t night_mode, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_flicker(uint32_t flicker_mode, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_iso(uint32_t iso, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_flash(uint32_t flash_mode, uint32_t *skip_mode, uint32_t *skip_num);
static int camera_set_video_mode(uint32_t mode, uint32_t *skip_mode, uint32_t *skip_num);


uint32_t camera_flash_mode_to_status(enum cmr_flash_mode f_mode)
{
	uint32_t                 status = FLASH_STATUS_MAX;

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
	default:
		break;
	}

	return status;
}

int camera_set_brightness(uint32_t brightness, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGI("brightness %d", brightness);
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		// TODO
		*skip_mode = IMG_SKIP_SW;
		*skip_num  = 0;
	} else {
		*skip_mode = IMG_SKIP_HW;
		*skip_num  = cxt->sn_cxt.sensor_info->preview_skip_num;
		ret = Sensor_Ioctl(SENSOR_IOCTL_BRIGHTNESS, brightness);
	}

	return ret;
}

int camera_set_contrast(uint32_t contrast, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGI("contrast %d", contrast);
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		// TODO
		*skip_mode = IMG_SKIP_SW;
		*skip_num  = 0;
	} else {
		*skip_mode = IMG_SKIP_HW;
		*skip_num  = cxt->sn_cxt.sensor_info->preview_skip_num;
		ret = Sensor_Ioctl(SENSOR_IOCTL_CONTRAST, contrast);
	}

	return ret;
}

int camera_set_effect(uint32_t effect, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGI("effect %d", effect);
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		// TODO
		*skip_mode = IMG_SKIP_SW;
		*skip_num  = 0;
	} else {
		*skip_mode = IMG_SKIP_HW;
		*skip_num  = cxt->sn_cxt.sensor_info->preview_skip_num;
		ret = Sensor_Ioctl(SENSOR_IOCTL_IMAGE_EFFECT, effect);
	}

	return ret;
}

int camera_set_ev(uint32_t expo_compen, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGI("expo_compen %d", expo_compen);
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		// TODO
		*skip_mode = IMG_SKIP_SW;
		*skip_num  = 0;
	} else {
		*skip_mode = IMG_SKIP_HW;
		*skip_num  = cxt->sn_cxt.sensor_info->preview_skip_num;
		ret = Sensor_Ioctl(SENSOR_IOCTL_EXPOSURE_COMPENSATION, expo_compen);
	}

	return ret;
}

int camera_set_wb(uint32_t wb_mode, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGI("wb_mode %d", wb_mode);
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		// TODO
		*skip_mode = IMG_SKIP_SW;
		*skip_num  = 0;
	} else {
		*skip_mode = IMG_SKIP_HW;
		*skip_num  = cxt->sn_cxt.sensor_info->preview_skip_num;
		ret = Sensor_Ioctl(SENSOR_IOCTL_SET_WB_MODE, wb_mode);
	}

	return ret;
}

int camera_set_scene(uint32_t scene_mode, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGI("scene_mode %d", scene_mode);
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		// TODO
		*skip_mode = IMG_SKIP_SW;
		*skip_num  = 0;
	} else {
		*skip_mode = IMG_SKIP_HW;
		*skip_num  = cxt->sn_cxt.sensor_info->preview_skip_num;
		ret = Sensor_Ioctl(SENSOR_IOCTL_PREVIEWMODE, scene_mode);
	}

	return ret;
}

int camera_set_night(uint32_t night_mode, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	return ret;
}

int camera_set_flicker(uint32_t flicker_mode, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGI("flicker_mode %d", flicker_mode);
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		// TODO
		*skip_mode = IMG_SKIP_SW;
		*skip_num  = 0;
	} else {
		*skip_mode = IMG_SKIP_HW;
		*skip_num  = cxt->sn_cxt.sensor_info->preview_skip_num;
		ret = Sensor_Ioctl(SENSOR_IOCTL_ANTI_BANDING_FLICKER, flicker_mode);
	}

	return ret;
}

int camera_set_iso(uint32_t iso, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGI("iso %d", iso);
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		// TODO
		*skip_mode = IMG_SKIP_SW;
		*skip_num  = 0;
	} else {
		*skip_mode = IMG_SKIP_HW;
		*skip_num  = cxt->sn_cxt.sensor_info->preview_skip_num;
		ret = Sensor_Ioctl(SENSOR_IOCTL_ISO, iso);
	}

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
		ret = Sensor_Ioctl(SENSOR_IOCTL_FLASH, status);
		cxt->cmr_set.flash = status;
	}
	return ret;
}

int camera_set_video_mode(uint32_t mode, uint32_t *skip_mode, uint32_t *skip_num)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGI("preview mode %d", mode);
	if (V4L2_SENSOR_FORMAT_RAWRGB == cxt->sn_cxt.sn_if.img_fmt) {
		// TODO
		*skip_mode = IMG_SKIP_SW;
		*skip_num  = 0;
	} else {
		*skip_mode = IMG_SKIP_HW;
		*skip_num  = cxt->sn_cxt.sensor_info->preview_skip_num;
		ret = Sensor_Ioctl(SENSOR_IOCTL_VIDEO_MODE, mode);
	}

	return ret;
}

int camera_flash_process(uint32_t on_off)
{
	struct camera_context    *cxt = camera_get_cxt();
	uint32_t                 status = on_off ? FLASH_TORCH : FLASH_CLOSE_AFTER_OPEN;

	if (FLASH_OPEN_ON_RECORDING == cxt->cmr_set.flash) {
		Sensor_Ioctl(SENSOR_IOCTL_FLASH, status);
	}
	
	return CAMERA_SUCCESS;
}

int camera_setting_init(void)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	memset((void*)&cxt->cmr_set, INVALID_SET, 
		sizeof(struct camera_settings) - sizeof(pthread_mutex_t));
	
	pthread_mutex_init (&cxt->cmr_set.set_mutex, NULL);

	return ret;
}

int camera_setting_deinit(void)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	pthread_mutex_destroy(&cxt->cmr_set.set_mutex);

	return ret;	
}

int camera_preview_start_set(void)
{
	struct camera_context    *cxt = camera_get_cxt();
	struct camera_settings   *set = &cxt->cmr_set;
	uint32_t                 skip, skip_num;
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("Sensor workmode %d", cxt->sn_cxt.preview_mode);

	ret = Sensor_SetMode(cxt->sn_cxt.preview_mode);
	if (ret) {
		CMR_LOGE("Sensor can't work at this mode %d", cxt->sn_cxt.preview_mode);
		goto exit;
	}

	if (INVALID_SET != set->brightness) {
		ret = camera_set_brightness(set->brightness, &skip, &skip_num);
		CMR_RTN_IF_ERR(ret);
	}

	if (INVALID_SET != set->contrast) {
		ret = camera_set_contrast(set->contrast, &skip, &skip_num);
		CMR_RTN_IF_ERR(ret);
	}

	if (INVALID_SET != set->effect) {
		ret = camera_set_effect(set->effect, &skip, &skip_num);
		CMR_RTN_IF_ERR(ret);
	}

	if (INVALID_SET != set->expo_compen) {
		ret = camera_set_ev(set->expo_compen, &skip, &skip_num);
		CMR_RTN_IF_ERR(ret);
	}

	if (INVALID_SET != set->wb_mode) {
		ret = camera_set_wb(set->wb_mode, &skip, &skip_num);
		CMR_RTN_IF_ERR(ret);
	}

	if (INVALID_SET != set->scene_mode) {
		ret = camera_set_scene(set->scene_mode, &skip, &skip_num);
		CMR_RTN_IF_ERR(ret);
	}

	if (INVALID_SET != set->flicker_mode) {
		ret = camera_set_flicker(set->flicker_mode, &skip, &skip_num);
		CMR_RTN_IF_ERR(ret);
	}

	if (INVALID_SET != set->iso) {
		ret = camera_set_iso(set->iso, &skip, &skip_num);
		CMR_RTN_IF_ERR(ret);
	}

	if (INVALID_SET != set->video_mode) {
		ret = camera_set_video_mode(set->video_mode, &skip, &skip_num);
		CMR_RTN_IF_ERR(ret);
	}

	ret = camera_flash_process(1);
exit:
	return ret;
}

int camera_preview_stop_set(void)
{
	int                      ret = CAMERA_SUCCESS;

	/*Todo something if necessary after preview stopped*/
	ret = camera_flash_process(0);

	return ret;
}

int camera_snapshot_start_set(void)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();

	ret = Sensor_Ioctl(SENSOR_IOCTL_BEFORE_SNAPSHOT, cxt->sn_cxt.capture_mode);
	if (ret) {
		CMR_LOGE("Sensor can't work at this mode %d", cxt->sn_cxt.capture_mode);
	} else {
	
	}

	return ret;
}

int camera_snapshot_stop_set(void)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();

	ret = Sensor_Ioctl(SENSOR_IOCTL_AFTER_SNAPSHOT, cxt->sn_cxt.preview_mode);
	if (ret) {
		CMR_LOGE("Sensor can't work at this mode %d", cxt->sn_cxt.preview_mode);
	} else {
	
	}

	return ret;
}

int camera_autofocus_init(void)
{
	struct camera_context    *cxt = camera_get_cxt();
	SENSOR_EXT_FUN_PARAM_T   af_param;
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("inited, %d", cxt->cmr_set.af_inited);
	if (1 != cxt->cmr_set.af_inited) {
		af_param.cmd = SENSOR_EXT_FUNC_INIT;
		af_param.param = SENSOR_EXT_FOCUS_TRIG;
		ret = Sensor_Ioctl(SENSOR_IOCTL_FOCUS, (uint32_t)&af_param);
		if (ret) {
			CMR_LOGE("Failed to init AF");
		} else {
			cxt->cmr_set.af_inited = 1;
		}
	}

	return ret;
}

int camera_set_ctrl(camera_parm_type id,
			uint32_t          parm,
			cmr_before_set_cb before_set,
			cmr_after_set_cb  after_set)
{
	struct camera_context    *cxt = camera_get_cxt();
	uint32_t                 skip_mode, skip_number;
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGE("ID %d or parm %d . camera status %d", id, parm, cxt->camera_status);

	if (id >= CAMERA_PARM_MAX || INVALID_SET == parm) {
		return CAMERA_INVALID_PARM;
	}

	switch (id) {
	case CAMERA_PARM_EXPOSURE_COMPENSATION:
		if (parm != cxt->cmr_set.expo_compen) {
			if (CMR_PREVIEW == cxt->camera_status) {
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
		if (CMR_CAPTURE == cxt->camera_status || CMR_CAPTURE_SLICE == cxt->camera_status) {
			
		}
		cxt->cap_rot = (uint32_t)camera_get_rot_angle(parm);
		break;
		
	case CAMERA_PARM_SENSOR_ROTATION: /* 0, 90, 180, 270 degrees */
		cxt->prev_rot = (uint32_t)camera_get_rot_angle(parm);
		break;
		
	case CAMERA_PARM_FOCAL_LENGTH: 
		cxt->cmr_set.focal_len = (uint32_t)parm;
		break;	
		
	case CAMERA_PARM_CONTRAST:    /* contrast */
		if (parm != cxt->cmr_set.contrast) {
			if (CMR_PREVIEW == cxt->camera_status) {
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
			if (CMR_PREVIEW == cxt->camera_status) {
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
		
	case CAMERA_PARM_WB:              /* use camera_wb_type */
		if (parm != cxt->cmr_set.wb_mode) {
			if (CMR_PREVIEW == cxt->camera_status) {
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

	case CAMERA_PARM_EFFECT:          /* use camera_effect_type */
		if (parm != cxt->cmr_set.effect) {
			if (CMR_PREVIEW == cxt->camera_status) {
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

	case CAMERA_PARM_SCENE_MODE:          /* use camera_scene_mode_type */
		if (parm != cxt->cmr_set.scene_mode) {
			if (CMR_PREVIEW == cxt->camera_status) {
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
		if (parm != cxt->zoom_level) {
			CMR_LOGV("Set zoom level %d", parm);
			cxt->zoom_level = parm;
			if (CMR_PREVIEW == cxt->camera_status) {
				if (before_set) {
					ret = (*before_set)(RESTART_MIDDLE);
					CMR_RTN_IF_ERR(ret);
				}
				skip_mode = IMG_SKIP_HW;
				skip_number = 1;
				CMR_RTN_IF_ERR(ret);
				if (after_set) {
					ret = (*after_set)(RESTART_MIDDLE, skip_mode, skip_number);
					CMR_RTN_IF_ERR(ret);
				}
			}
		}
		break;

	case CAMERA_PARM_JPEGCOMP:
		cxt->jpeg_cxt.quality = parm;
		break;

	case CAMERA_PARM_ORIENTATION:

		break;

	case CAMERA_PARM_FLASH:         /* Flash control, see camera_flash_type */
		ret = camera_set_flash(parm, &skip_mode, &skip_number);
		break;
		
	case CAMERA_PARM_NIGHTSHOT_MODE:  /* Night shot mode, snapshot at reduced FPS */
		if (CMR_PREVIEW == cxt->camera_status) {
			
		} 

		cxt->cmr_set.night_mode = parm;
		break;
		
	case CAMERA_PARM_ANTIBANDING:   /* Use camera_anti_banding_type */
		if (parm != cxt->cmr_set.flicker_mode) {
			if (CMR_PREVIEW == cxt->camera_status) {
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
		CMR_LOGV("Set AF Mode %d", parm);
		if (CMR_PREVIEW == cxt->camera_status) {
			
		} 
		cxt->cmr_set.af_mode = (uint32_t)parm;
		break;

	case CAMERA_PARM_ISO:
		if (parm != cxt->cmr_set.iso) {
			if (CMR_PREVIEW == cxt->camera_status) {
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

	case CAMERA_PARM_PREVIEW_MODE:   /* Use camera_preview_mode_type */
		if (parm != cxt->cmr_set.video_mode) {
			if (CMR_PREVIEW == cxt->camera_status) {
				if (before_set) {
					ret = (*before_set)(RESTART_LIGHTLY);
					CMR_RTN_IF_ERR(ret);
				}
				ret = camera_set_video_mode(parm, &skip_mode, &skip_number);
				CMR_RTN_IF_ERR(ret);
				if (after_set) {
					ret = (*after_set)(RESTART_LIGHTLY, skip_mode, skip_number);
					CMR_RTN_IF_ERR(ret);
				}
				
			} 
			cxt->cmr_set.video_mode = parm;
		}
		break;

	default:
		break;  
	}	

exit:
	return ret;
}

int camera_autofocus_start(void)
{
	int                      ret = CAMERA_SUCCESS;
	struct camera_context    *cxt = camera_get_cxt();
	uint32_t                 *ptr = (uint32_t*)&cxt->cmr_set.focus_zone_param[0];
	uint32_t                 i = 0;
	uint32_t                 zone_cnt = *ptr++;
	SENSOR_EXT_FUN_PARAM_T   af_param;

	CMR_LOGV("zone_cnt %d, x y w h, %d %d %d %d", zone_cnt, ptr[0], ptr[1], ptr[2], ptr[3]);

	if (camera_autofocus_need_exit()) {
		ret = CAMERA_INVALID_STATE;
		CMR_RTN_IF_ERR(ret);
	}
	
	ret = camera_autofocus_init();
	CMR_RTN_IF_ERR(ret);

	if (camera_autofocus_need_exit()) {
		ret = CAMERA_INVALID_STATE;
		CMR_RTN_IF_ERR(ret);
	}
	
	if (cxt->cmr_set.flash) {
		Sensor_Ioctl(SENSOR_IOCTL_FLASH, FLASH_OPEN);	/*open flash*/
	}

	if (CAMERA_FOCUS_MODE_MACRO == cxt->cmr_set.af_mode) {
		af_param.cmd = SENSOR_EXT_FOCUS_START;
		af_param.param = SENSOR_EXT_FOCUS_MACRO;
		CMR_LOGV("SPRD OEM: camera_start_focus macro");
	} else {
		if (0 == zone_cnt) {
			af_param.cmd = SENSOR_EXT_FOCUS_START;
			af_param.param = SENSOR_EXT_FOCUS_TRIG;
		} else if (1 == zone_cnt) {
			af_param.cmd = SENSOR_EXT_FOCUS_START;
			af_param.param = SENSOR_EXT_FOCUS_ZONE;
			af_param.zone_cnt = 1;
			af_param.zone[0].x = *ptr++;
			af_param.zone[0].y = *ptr++;
			af_param.zone[0].w = *ptr++;
			af_param.zone[0].h = *ptr++;

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
		} else {
			CMR_LOGE("Unsupported zone count %d", zone_cnt);
			ret = CAMERA_NOT_SUPPORTED;
		}
	}

	if (CAMERA_SUCCESS == ret) {
		ret = Sensor_Ioctl(SENSOR_IOCTL_FOCUS, (uint32_t) & af_param);
	}


	if (cxt->cmr_set.flash) {
		Sensor_Ioctl(SENSOR_IOCTL_FLASH, FLASH_CLOSE_AFTER_OPEN);
	}
	
	CMR_LOGV("End. %d", ret);
	
	if (ret) {
		ret = CAMERA_FAILED;
	}

exit:
	return ret;
}

int camera_autofocus_stop(void)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = CAMERA_SUCCESS;

	pthread_mutex_lock(&cxt->cmr_set.set_mutex);
	cxt->cmr_set.af_cancelled = 1;
	pthread_mutex_unlock(&cxt->cmr_set.set_mutex);

	return ret;
}

int camera_autofocus_need_exit(void)
{
	struct camera_context    *cxt = camera_get_cxt();
	int                      ret = 0;

	pthread_mutex_lock(&cxt->cmr_set.set_mutex);
	ret = cxt->cmr_set.af_cancelled == 1 ? 1 : 0;
	pthread_mutex_unlock(&cxt->cmr_set.set_mutex);

	return ret;
}

int camera_af_ctrl(uint32_t step)
{
	int                      ret = CAMERA_SUCCESS;

	return ret;
	
}

int camera_ae_get_gain(uint32_t * val)
{
	int                      ret = CAMERA_SUCCESS;

	return ret;

}

int camera_ae_set_gain(uint32_t val)
{
	int                      ret = CAMERA_SUCCESS;

	return ret;

}

int camera_skip_frame_cb(uint32_t rtn,void *param_ptr)
{
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("skip frame cb.");

	return ret;

}

int camera_proc_start_cb(uint32_t rtn,void *param_ptr)
{
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("proc start cb.");

	return ret;

}