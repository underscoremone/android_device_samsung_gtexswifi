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
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <math.h>
#include "SprdOEMCamera.h"
#include "cmr_oem.h"
#include "sprd_rot_k.h"

struct camera_context        cmr_cxt;
struct camera_context        *g_cxt = &cmr_cxt;
#define IS_PREVIEW           (CMR_PREVIEW == g_cxt->camera_status)
#define IS_CAPTURE           (CMR_CAPTURE == g_cxt->camera_status || CMR_CAPTURE_SLICE == g_cxt->camera_status)

static struct img_addr       addi_addr;


static void camera_sensor_evt_cb(int evt, void* data);
static void camera_isp_evt_cb(int evt, void* data);
static void camera_jpeg_evt_cb(int evt, void* data);
static void camera_v4l2_evt_cb(int evt, void* data);
static void camera_rot_evt_cb(int evt, void* data);
static void camera_scaler_evt_cb(int evt, void* data);
static int  camera_create_main_thread(void);
static int  camera_destroy_main_thread(void);
static int  camera_get_sensor_preview_mode(struct img_size* target_size, uint32_t *work_mode);
static int  camera_get_sensor_capture_mode(struct img_size* target_size, uint32_t *work_mode);
static int  camera_preview_init(int format_mode);
static void camera_set_client_data(void* user_data);
static void *camera_get_client_data(void);
static void camera_set_hal_cb(camera_cb_f_type cmr_cb);
static void camera_call_cb(camera_cb_type cb, const void *client_data,
		camera_func_type func, int32_t parm4);
static int  camera_capture_init(int format_mode);
static void *camera_main_routine(void *client_data);
static int  camera_v4l2_handle(uint32_t evt_type, struct frm_info *data);
static int  camera_isp_handle(uint32_t evt_type, struct frm_info *data);
static int  camera_jpeg_codec_handle(uint32_t evt_type, void *data);
static int  camera_scale_handle(uint32_t evt_type, struct img_frm *data);
static int  camera_rotation_handle(uint32_t evt_type, struct img_frm *data);
static int  camera_sensor_handle(uint32_t evt_type, struct frm_info *data);
static int  camera_img_cvt_handle(uint32_t evt_type, struct img_frm *data);
static int  camera_af_handle(uint32_t evt_type, struct img_frm *data);
static int  camera_v4l2_preview_handle(struct frm_info *data);
static int  camera_v4l2_capture_handle(struct frm_info *data);
static int  camera_alloc_preview_buf(struct buffer_cfg *buffer, uint32_t format);
static int  camera_capture_ability(SENSOR_MODE_INFO_T *sn_mode, 
			struct img_frm_cap *img_cap, 
			struct img_size *cap_size);
static int  camera_alloc_capture_buf0(struct buffer_cfg *buffer, uint32_t cap_index);
static int  camera_alloc_capture_buf1(struct buffer_cfg *buffer, uint32_t cap_index);
static int  camera_start_isp_process(struct frm_info *data);
static int  camera_start_jpeg_decode(struct frm_info *data);
static int  camera_start_jpeg_encode(struct frm_info *data);
static int  camera_jpeg_encode_next(struct frm_info *data);
static int  camera_start_scale(struct frm_info *data);
static int  camera_scale_next(struct frm_info *data);
static int  camera_scale_done(struct frm_info *data);
static int  camera_start_rotate(struct frm_info *data);
static int  camera_start_preview_internal(void);
static int  camera_stop_preview_internal(void);
static int  camera_before_set(enum restart_mode re_mode);
static int  camera_after_set(enum restart_mode re_mode,
			enum img_skip_mode skip_mode,
			uint32_t skip_number);
static int camera_wait_for_preview_done(void);
static int camera_preview_done(void);
static int camera_jpeg_encode_done(struct frm_info *data);
static int camera_jpeg_encode_handle(JPEG_ENC_CB_PARAM_T *data);
static int camera_jpeg_decode_handle(JPEG_DEC_CB_PARAM_T *data);
static int camera_set_frame_type(camera_frame_type *frame_type, struct frm_info* info);

static int camera_save_to_file(uint32_t index, uint32_t img_fmt,
	uint32_t width, uint32_t height, struct img_addr *addr);


camera_ret_code_type camera_encode_picture(camera_frame_type *frame,
					camera_handle_type *handle,
					camera_cb_f_type callback,
					void *client_data)
{
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("w h %d %d", frame->dx, frame->dy);

	return ret;
}

int camera_sensor_init(int32_t camera_id)
{
	struct camera_ctrl       *ctrl = &g_cxt->control;
	struct sensor_context    *sensor_cxt = &g_cxt->sn_cxt;
	uint32_t                 sensor_num, sensor_ret;
	int                      ret = CAMERA_SUCCESS;

	if (ctrl->sensor_inited) {
		CMR_LOGI("sensor intialized before");
		goto exit;
	}

	sensor_num = Sensor_Init();
	if (0 == sensor_num) {
		CMR_LOGE("No sensor %d", sensor_num);
		ret = -CAMERA_NO_SENSOR;
		goto exit;
	} else {
		if ((uint32_t)camera_id >= sensor_num) {
			CMR_LOGE("No sensor %d", sensor_num);
			ret = -CAMERA_NO_SENSOR;
			goto exit;
		}
		sensor_ret = Sensor_Open(camera_id);
		if (sensor_ret) {
			CMR_LOGE("Failed to open %d sensor", camera_id);
			ret = -CAMERA_NO_SENSOR;
			goto exit;
		}
		sensor_cxt->cur_id = camera_id;
		sensor_cxt->sensor_info = Sensor_GetInfo();
		if (NULL == sensor_cxt->sensor_info) {
			CMR_LOGE("Failed to Get sensor info");
			ret = -CAMERA_NOT_SUPPORTED;
			goto sensor_exit;
		}
		if (SENSOR_IMAGE_FORMAT_RAW == sensor_cxt->sensor_info->image_format) {
			sensor_cxt->sn_if.img_fmt = V4L2_SENSOR_FORMAT_RAWRGB;
			ret = Sensor_GetRawSettings(&sensor_cxt->raw_settings, &sensor_cxt->setting_length);
			if (ret) {
				CMR_LOGE("Failed to Get sensor raw settings");
				ret = -CAMERA_NOT_SUPPORTED;
				goto sensor_exit;
			}
		} else {
			sensor_cxt->sn_if.img_fmt = V4L2_SENSOR_FORMAT_YUV;
		}

		if (SENSOR_INTERFACE_CSI2 == sensor_cxt->sensor_info->sensor_interface) {
			sensor_cxt->sn_if.if_type = 1;
			sensor_cxt->sn_if.if_spec.mipi.bits_per_pxl = 10;
			sensor_cxt->sn_if.if_spec.mipi.is_packet    = 0;
			sensor_cxt->sn_if.if_spec.mipi.lane_num     = 2;
			sensor_cxt->sn_if.if_spec.mipi.use_href     = 0;
		} else {
			sensor_cxt->sn_if.if_type = 0;
			sensor_cxt->sn_if.if_spec.ccir.v_sync_pol = sensor_cxt->sensor_info->vsync_polarity;
			sensor_cxt->sn_if.if_spec.ccir.h_sync_pol = sensor_cxt->sensor_info->hsync_polarity;
			sensor_cxt->sn_if.if_spec.ccir.pclk_pol   = sensor_cxt->sensor_info->pclk_polarity;
		}
		sensor_cxt->sn_if.frm_deci    = sensor_cxt->sensor_info->preview_deci_num;
		sensor_cxt->sn_if.img_ptn     = sensor_cxt->sensor_info->image_pattern;
		sensor_cxt->sn_if.size.width  = sensor_cxt->sensor_info->source_width_max;
		sensor_cxt->sn_if.size.height = sensor_cxt->sensor_info->source_height_max;

		Sensor_EventReg(camera_sensor_evt_cb);
		ctrl->sensor_inited = 1;
		goto exit;
	}

sensor_exit:
	Sensor_Close();
exit:
	return ret;
}

int camera_sensor_deinit(void)
{
	struct camera_ctrl       *ctrl = &g_cxt->control;
	struct sensor_context    *sensor_cxt = &g_cxt->sn_cxt;
	uint32_t                 sensor_ret;
	int                      ret = CAMERA_SUCCESS;

	if (0 == ctrl->sensor_inited) {
		CMR_LOGI("sensor has been de-intialized");
		goto exit;
	}
	Sensor_EventReg(NULL);
	ret = Sensor_Close();
	bzero(sensor_cxt, sizeof(*sensor_cxt));
	ctrl->sensor_inited = 0;

exit:
	return ret;
}

int camera_v4l2_init(void)
{
	struct camera_ctrl       *ctrl = &g_cxt->control;
	struct v4l2_context      *cxt  = &g_cxt->v4l2_cxt;
	int                      ret   = CAMERA_SUCCESS;

	if (0 == ctrl->v4l2_inited) {
		ret = cmr_v4l2_init();
		if (ret) {
			CMR_LOGE("Failed to init v4l2 %d", ret);
			ret = -CAMERA_NOT_SUPPORTED;
			goto exit;
		}
		ret = cmr_v4l2_scale_capability(&cxt->sc_capability);
		if (ret) {
			CMR_LOGE("Failed to get v4l2 frame scaling capability %d", ret);
			ret = -CAMERA_NOT_SUPPORTED;
			goto exit;
		}
		cmr_v4l2_evt_reg(camera_v4l2_evt_cb);
		cxt->v4l2_state = V4L2_IDLE;
		ctrl->v4l2_inited = 1;
	}

exit:
	return ret;
}

int camera_v4l2_deinit(void)
{
	struct camera_ctrl       *ctrl = &g_cxt->control;
	struct v4l2_context      *cxt  = &g_cxt->v4l2_cxt;
	int                      ret   = CAMERA_SUCCESS;

	if (0 == ctrl->v4l2_inited) {
		CMR_LOGI("V4L2 has been de-intialized");
		goto exit;
	}

	ret = cmr_v4l2_deinit();
	if (ret) {
		CMR_LOGE("Failed to init v4l2 %d", ret);
		ret = -CAMERA_NOT_SUPPORTED;
		goto exit;
	}
	bzero(cxt, sizeof(*cxt));
	ctrl->v4l2_inited = 0;

exit:
	return ret;
}

int camera_isp_init(void)
{
	struct camera_ctrl       *ctrl = &g_cxt->control;
	struct isp_context       *cxt = &g_cxt->isp_cxt;
/*	struct isp_init_param    isp_param;*/
	int                      ret = CAMERA_SUCCESS;
/*	struct isp_video_limit   isp_limit;*/

	if (0 == ctrl->sensor_inited || V4L2_SENSOR_FORMAT_RAWRGB != g_cxt->sn_cxt.sn_if.img_fmt) {
		CMR_LOGI("No need to init ISP %d %d", ctrl->sensor_inited, g_cxt->sn_cxt.sn_if.img_fmt);
		goto exit;
	}

	if (0 == ctrl->isp_inited) {
/*		isp_param.setting_param_ptr = g_cxt->sn_cxt.raw_settings;
		isp_param.size = g_cxt->sn_cxt.setting_length;
		isp_param.ctrl_func.af_notice_cb = camera_af_ctrl;
		isp_param.ctrl_func.skip_frame_cb = camera_skip_frame_cb;*/
/*		isp_param.ctrl_func.ae_get_gain_cb = camera_ae_get_gain;
		isp_param.ctrl_func.ae_set_gain_cb = camera_ae_set_gain;
*/
/*		ret = isp_init(&isp_param);*/
		if (ret) {
			CMR_LOGE("Failed to init ISP %d", ret);
			ret = -CAMERA_NOT_SUPPORTED;
			goto exit;
		}
/*		ret = isp_capbility(ISP_VIDEO_SIZE, &isp_limit);*/
		if (ret) {
			CMR_LOGE("Failed to get the limitation of ISP %d", ret);
			ret = -CAMERA_NOT_SUPPORTED;
			goto exit;
		}
/*		cmr_isp_evt_reg(camera_isp_evt_cb);*//*wjp*/
//		cxt->width_limit = isp_limit.width;
		cxt->isp_state = ISP_IDLE;
		ctrl->isp_inited = 1;

	}

exit:
	return ret;
}

int camera_isp_deinit(void)
{
	struct camera_ctrl       *ctrl = &g_cxt->control;
	struct isp_context       *isp_cxt = &g_cxt->isp_cxt;
	int                      ret = CAMERA_SUCCESS;

	if (0 == ctrl->isp_inited) {
		CMR_LOGI("V4L2 has been de-intialized");
		goto exit;
	}
	/*cmr_isp_evt_reg(NULL);*/
/*	ret = isp_deinit();*/
	if (ret) {
		CMR_LOGE("Failed to de-init ISP %d", ret);
		ret = -CAMERA_NOT_SUPPORTED;
		goto exit;
	}
	bzero(isp_cxt, sizeof(*isp_cxt));
	ctrl->isp_inited = 0;

exit:
	return ret;
}

int camera_jpeg_init(void)
{
	struct camera_ctrl       *ctrl = &g_cxt->control;
	struct jpeg_context      *cxt  = &g_cxt->jpeg_cxt;
	int                      ret   = CAMERA_SUCCESS;

	if (0 == ctrl->isp_inited) {
		ret = jpeg_init();
		if (CAMERA_SUCCESS == ret) {
			jpeg_evt_reg(camera_jpeg_evt_cb);
			cxt->jpeg_state = JPEG_IDLE;
			ctrl->jpeg_inited = 1;
		} else {
			CMR_LOGE("Failed to init JPEG Codec %d", ret);
			ret = -CAMERA_NOT_SUPPORTED;
		}
	}

	return ret;
}

int camera_jpeg_deinit(void)
{
	struct camera_ctrl       *ctrl = &g_cxt->control;
	struct jpeg_context      *cxt  = &g_cxt->jpeg_cxt;
	int                      ret   = CAMERA_SUCCESS;

	if (0 == ctrl->jpeg_inited) {
		CMR_LOGI("JPEG Codec has been de-intialized");
		goto exit;
	}
	jpeg_evt_reg(NULL);
	ret = jpeg_deinit();
	if (ret) {
		CMR_LOGE("Failed to de-init JPEG Codec %d", ret);
		ret = -CAMERA_NOT_SUPPORTED;
		goto exit;
	}
	bzero(cxt, sizeof(*cxt));
	ctrl->jpeg_inited = 0;

exit:
	return ret;
}

int camera_rotation_init(void)
{
	struct camera_ctrl       *ctrl = &g_cxt->control;
	struct rotation_context  *cxt  = &g_cxt->rot_cxt;
	int                      ret   = CAMERA_SUCCESS;

	if (1 == ctrl->rot_inited) {
		CMR_LOGI("Rot has been intialized");
		goto exit;
	}

	ret = cmr_rot_init();
	if (ret) {
		CMR_LOGE("Failed to init Rot %d", ret);
		ret = -CAMERA_NOT_SUPPORTED;
	} else {
		cmr_rot_evt_reg(camera_rot_evt_cb);
		cxt->rot_state = IMG_CVT_IDLE;
		sem_init(&cxt->cmr_rot_sem, 0, 1);
		ctrl->rot_inited = 1;
	}

exit:
	return ret;
}

int camera_rotation_deinit(void)
{
	struct camera_ctrl       *ctrl = &g_cxt->control;
	struct rotation_context  *cxt  = &g_cxt->rot_cxt;
	int                      ret   = CAMERA_SUCCESS;

	if (0 == ctrl->rot_inited) {
		CMR_LOGI("Rot has been de-intialized");
		goto exit;
	}

	ret = cmr_rot_deinit();
	if (ret) {
		CMR_LOGE("Failed to de-init ROT %d", ret);
		ret = -CAMERA_NOT_SUPPORTED;
		goto exit;
	}
	sem_destroy(&cxt->cmr_rot_sem);
	bzero(cxt, sizeof(*cxt));
	ctrl->rot_inited = 0;

exit:
	return ret;
}

int camera_scaler_init(void)
{
	struct camera_ctrl       *ctrl = &g_cxt->control;
	struct scaler_context    *cxt  = &g_cxt->scaler_cxt;
	int                      ret   = CAMERA_SUCCESS;

	if (1 == ctrl->scaler_inited) {
		CMR_LOGI("scaler has been intialized");
		goto exit;
	}

	ret = cmr_scale_init();
	if (ret) {
		CMR_LOGE("Failed to init scaler %d", ret);
		ret = -CAMERA_NOT_SUPPORTED;
	} else {
		ret = cmr_scale_capability(&cxt->sc_capability);
		if (ret) {
			CMR_LOGE("Failed to get frame scaling capability %d", ret);
			ret = -CAMERA_NOT_SUPPORTED;
			goto exit;
		}
		cmr_scale_evt_reg(camera_scaler_evt_cb);
		cxt->scale_state = IMG_CVT_IDLE;
		ctrl->scaler_inited = 1;
	}

exit:
	return ret;
}

int camera_scaler_deinit(void)
{
	struct camera_ctrl       *ctrl = &g_cxt->control;
	struct scaler_context    *cxt  = &g_cxt->scaler_cxt;
	int                      ret   = CAMERA_SUCCESS;

	if (0 == ctrl->scaler_inited) {
		CMR_LOGI("scaler has been de-intialized");
		goto exit;
	}
	
	ret = cmr_scale_deinit();
	if (ret) {
		CMR_LOGE("Failed to de-init scaler %d", ret);
		ret = -CAMERA_NOT_SUPPORTED;
		goto exit;
	}
	bzero(cxt, sizeof(*cxt));
	ctrl->scaler_inited = 0;

exit:
	return ret;
}

int camera_local_init(void)
{	
	int                      ret = CAMERA_SUCCESS;

	ret = cmr_msg_queue_create(CAMERA_OEM_MSG_QUEUE_SIZE, &g_cxt->msg_queue_handle);
	if (ret) {
		CMR_LOGE("NO Memory, Frailed to create message queue");
	}

	g_cxt->camera_status = CMR_IDLE;

	pthread_mutex_init(&g_cxt->cb_mutex, NULL);
	pthread_mutex_init(&g_cxt->data_mutex, NULL);
	sem_init(&g_cxt->prev_sem, 0, 0);
	sem_init(&g_cxt->picture_sem, 0, 0);

	return ret;
}

int camera_local_deinit(void)
{
	int                      ret = CAMERA_SUCCESS;

	g_cxt->camera_status = CMR_IDLE;

	sem_destroy(&g_cxt->picture_sem);

	sem_destroy(&g_cxt->prev_sem);

	pthread_mutex_destroy (&g_cxt->data_mutex);

	pthread_mutex_destroy (&g_cxt->cb_mutex);

	cmr_msg_queue_destroy(g_cxt->msg_queue_handle);

	g_cxt->msg_queue_handle = 0;
	return ret;
}

camera_ret_code_type camera_init(int32_t camera_id)
{
	struct camera_ctrl       *ctrl = &g_cxt->control;
	int                      ret = CAMERA_SUCCESS;
	
	CMR_LOGV("%d %d", ctrl->sensor_inited, ctrl->v4l2_inited);

	bzero(g_cxt, sizeof(*g_cxt));

	/* 
		the whole initialize process can not be re-scheduled and 
		it should follow the beolow steps.

		sensor-->V4L2 manager-->ISP if necessary-->JPEG-->image convert-->Main thread;
	*/

	ret = camera_sensor_init(camera_id);
	if (ret) {
		CMR_LOGE("Failed to init sensor %d", ret);
		goto exit;
	}

	ret = camera_v4l2_init();
	if (ret) {
		CMR_LOGE("Failed to init V4L2 manager %d", ret);
		goto sensor_deinit;
	}

	ret = camera_isp_init();
	if (ret) {
		CMR_LOGE("Failed to init ISP driver %d", ret);
		goto v4l2_deinit;
	}

	ret = camera_jpeg_init();
	if (ret) {
		CMR_LOGE("Failed to init jpeg codec %d", ret);
		goto isp_deinit;
	}

	ret = camera_rotation_init();
	if (ret) {
		CMR_LOGE("Fail to init Rot device.");
		goto jpeg_deinit;
	}

	ret = camera_setting_init();
	if (ret) {
		CMR_LOGE("Fail to init Setting sub-module");
		goto rot_deinit;
	}

	ret = camera_local_init();
	if (ret) {
		CMR_LOGE("Failed to init local variables %d", ret);
		goto setting_deinit;
	}

	ret = camera_create_main_thread();
	if (CAMERA_SUCCESS == ret)
		goto exit;

local_deinit:
	camera_local_deinit();
setting_deinit:
	camera_setting_deinit();
rot_deinit:
	camera_rotation_deinit();
jpeg_deinit:
	camera_jpeg_deinit();
isp_deinit:
	camera_isp_deinit();
v4l2_deinit:
	camera_v4l2_deinit();
sensor_deinit:
	camera_sensor_deinit();
exit:
	return ret;
}

camera_ret_code_type camera_stop( camera_cb_f_type callback, void *client_data)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;

	camera_setting_deinit();

	camera_rotation_deinit();

	camera_jpeg_deinit();

	camera_isp_deinit();

	camera_v4l2_deinit();

	camera_sensor_deinit();

	camera_destroy_main_thread();

	camera_local_deinit();

	callback(CAMERA_EXIT_CB_DONE, client_data, CAMERA_FUNC_STOP, 0);

	return ret_type;
}

camera_ret_code_type camera_release_frame(uint32_t index)
{
	int                      ret = CAMERA_SUCCESS;

	if (!IS_PREVIEW || g_cxt->v4l2_cxt.v4l2_state != V4L2_PREVIEW) {
		CMR_LOGE("Not in Preview, cmr %d, v4l2 %d", g_cxt->camera_status, g_cxt->v4l2_cxt.v4l2_state);
		return CAMERA_SUCCESS;
	}

	/* 
	   only the frame whose rotation angle is zero should be released by app, 
	   otherwise, it will be released after rotation done;
	 */
	if (IMG_ROT_0 == g_cxt->prev_rot) {
		if (index >= CAMERA_OEM_PREV_FRM_ID_BASE && 
			index < CAMERA_OEM_PREV_FRM_ID_BASE + CAMERA_OEM_PREV_FRM_CNT) {
			ret = cmr_v4l2_free_frame(0, index);
			CMR_LOGV("release the frame whose index is 0x%x, rot %d, ret %d",
				index,
				g_cxt->prev_rot,
				ret);
		} else {
			CMR_LOGE("wrong index, 0x%x ", index);
		}
		
	}

	return CAMERA_SUCCESS;
}

int camera_preview_sensor_mode(void)
{
	int                      ret = CAMERA_SUCCESS;

	if (IMG_ROT_90 == g_cxt->prev_rot || IMG_ROT_270 == g_cxt->prev_rot) {
		g_cxt->preview_size.width = g_cxt->display_size.height;
		g_cxt->preview_size.height = g_cxt->display_size.width;
	} else {
		g_cxt->preview_size.width  = g_cxt->display_size.width;
		g_cxt->preview_size.height = g_cxt->display_size.height;
	}

	ret = camera_get_sensor_preview_mode(&g_cxt->preview_size, &g_cxt->sn_cxt.preview_mode);
	if (ret) {
		CMR_LOGE("Unsupported display size");
		ret = -CAMERA_NOT_SUPPORTED;
	}

	return ret;
}

int camera_capture_sensor_mode(void)
{
	int                      ret = CAMERA_SUCCESS;
	SENSOR_MODE_INFO_T       *sensor_mode;

	CMR_LOGV("cap_rot %d, capture size %d %d",
		g_cxt->cap_rot,
		g_cxt->picture_size.width,
		g_cxt->picture_size.height);

	if (IMG_ROT_90 == g_cxt->cap_rot || IMG_ROT_270 == g_cxt->cap_rot) {
		g_cxt->capture_size.width  = g_cxt->picture_size.height;
		g_cxt->capture_size.height = g_cxt->picture_size.width;
	} else {
		g_cxt->capture_size.width  = g_cxt->picture_size.width;
		g_cxt->capture_size.height = g_cxt->picture_size.height;
	}

	ret = camera_get_sensor_capture_mode(&g_cxt->capture_size, &g_cxt->sn_cxt.capture_mode);
	if (ret) {
		CMR_LOGE("Unsupported picture size");
		ret = -CAMERA_NOT_SUPPORTED;
	}

	sensor_mode = &g_cxt->sn_cxt.sensor_info->sensor_mode_info[g_cxt->sn_cxt.capture_mode];

	g_cxt->max_size.width  = MAX(g_cxt->picture_size.width,  sensor_mode->width);
	g_cxt->max_size.height = MAX(g_cxt->picture_size.height, sensor_mode->height);

	return ret;
}

camera_ret_code_type camera_set_dimensions(uint16_t picture_width,
					uint16_t picture_height,
					uint16_t display_width,
					uint16_t display_height,
					camera_cb_f_type callback,
					void *client_data)
{
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("picture %d %d, display %d %d",
		picture_width, 
		picture_height, 
		display_width, 
		display_height);

	// g_cxt->prev_rot = IMG_ROT_90;
	if (picture_width && picture_height && display_width && display_height) {
		g_cxt->display_size.width  = display_width;
		g_cxt->display_size.height = display_height;
		ret = camera_preview_sensor_mode();

		g_cxt->picture_size.width  = picture_width;
		g_cxt->picture_size.height = picture_height;
	} else {
		ret = -CAMERA_INVALID_PARM;
	}

	return ret;
}

camera_ret_code_type camera_set_encode_properties(camera_encode_properties_type *encode_properties)
{
	int                      ret = CAMERA_SUCCESS;

	if (NULL == encode_properties) {
		return CAMERA_INVALID_PARM;
	}

	CMR_LOGV("Take photo format %d",encode_properties->format);
	
	switch (encode_properties->format) {
	case CAMERA_RAW:
		g_cxt->cap_target_fmt = V4L2_SENSOR_FORMAT_RAWRGB;
		break;
	case CAMERA_JPEG:
		g_cxt->cap_target_fmt = V4L2_SENSOR_FORMAT_JPEG;
		break;
	case CAMERA_YCBCR_ENCODE:
		g_cxt->cap_target_fmt = V4L2_SENSOR_FORMAT_YUV;
		break;
	default:
		ret = CAMERA_INVALID_FORMAT;
		break;
	}

	return ret;
}

camera_ret_code_type camera_set_parm(camera_parm_type id,
				uint32_t         parm,
				camera_cb_f_type callback,
				void            *client_data)
{
	return camera_set_ctrl(id, parm, camera_before_set, camera_after_set);
}


camera_ret_code_type camera_set_position(camera_position_type *position,
					camera_cb_f_type      callback,
					void                 *client_data)
{
	int                      ret = CAMERA_SUCCESS;

	return ret;

}

camera_ret_code_type camera_set_thumbnail_properties(uint32_t width,
						uint32_t height,
						uint32_t quality)
{
	(void) quality;

	if (0 == width || 0 == height)
		return CAMERA_INVALID_PARM;

	g_cxt->thum_size.width  = width;
	g_cxt->thum_size.height = height;

	return CAMERA_SUCCESS;
}

camera_ret_code_type camera_start(camera_cb_f_type callback,
				void *client_data,
				int  display_height,
				int  display_width)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;

	CMR_LOGV("OK to init_device.");
	//change the status from INIT to IDLE.
	callback(CAMERA_STATUS_CB, client_data, CAMERA_FUNC_START, 0);
	CMR_LOGV("OK to change the status from INIT to IDLE.");

	return ret_type;
}

int camera_before_set(enum restart_mode re_mode)
{
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("restart mode %d", re_mode);
	if (re_mode >= RESTART_MAX) {
		CMR_LOGE("Wrong restart mode");
		return CAMERA_INVALID_PARM;
	}

	switch (re_mode) {
	case RESTART_HEAVY:
	case RESTART_MIDDLE:
		ret = camera_stop_preview_internal();
		break;
	case RESTART_LIGHTLY:
		ret = cmr_v4l2_cap_pause();
		break;
	default:
		break;
	}
	return ret;
}

int camera_after_set(enum restart_mode re_mode,
			enum img_skip_mode skip_mode,
			uint32_t skip_number)
{
	int                      ret = CAMERA_SUCCESS;
	uint32_t                 skip_number_l = 0;

	CMR_LOGV("after set %d, skip mode %d, skip number %d",
		re_mode,
		skip_mode,
		skip_number);

	if (skip_mode > IMG_SKIP_SW) {
		CMR_LOGE("Wrong skip mode");
		return -CAMERA_FAILED;
	}

	g_cxt->skip_mode = skip_mode;
	g_cxt->skip_num  = skip_number;
	if (IMG_SKIP_HW == g_cxt->skip_mode) {
		skip_number_l = g_cxt->skip_num;
	}

	switch (re_mode) {
	case RESTART_HEAVY:
		ret = camera_preview_sensor_mode();
		if (ret) {
			CMR_LOGE("Failed to get preview mode");
			return -CAMERA_FAILED;
		}
		ret  = camera_start_preview_internal();
		break;
	case RESTART_MIDDLE:
		ret = camera_preview_init(g_cxt->preview_fmt);
		if (ret) {
			CMR_LOGE("Failed to restart preview");
			return -CAMERA_FAILED;
		}
		ret = cmr_v4l2_cap_start(skip_number);
		if (ret) {
			CMR_LOGE("Failed to restart preview");
			return -CAMERA_FAILED;
		} else {
			g_cxt->v4l2_cxt.v4l2_state = V4L2_PREVIEW;
			g_cxt->camera_status = CMR_PREVIEW;
		}
		break;
	case RESTART_LIGHTLY:
		ret = cmr_v4l2_cap_resume(skip_mode, skip_number);
		break;
	default:
		CMR_LOGE("Wrong re-start mode");
		ret = -CAMERA_INVALID_PARM;
		break;
	}

	if (CMR_PREVIEW == g_cxt->camera_status) {
		ret = camera_wait_for_preview_done();
	}

	return ret;
}

int camera_start_preview_internal(void)
{
	uint32_t                 skip_number = 0;
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("preview format is %d .\n", g_cxt->preview_fmt);
	ret = camera_preview_start_set();
	if (ret) {
		CMR_LOGE("Fail to set sensor preview mode.");
		return -CAMERA_FAILED;
	}

	ret = camera_preview_init(g_cxt->preview_fmt);
	if (ret) {
		CMR_LOGE("Fail to init preview mode.");
		return -CAMERA_FAILED;
	}

	CMR_LOGV("OK  to camera_preview_init.\n");

	if (IMG_SKIP_HW == g_cxt->skip_mode) {
		skip_number = g_cxt->skip_num;
	}
	ret = cmr_v4l2_cap_start(skip_number);
	if (ret) {
		CMR_LOGE("Fail to start V4L2 Capture");
		return -CAMERA_FAILED;
	}
	g_cxt->v4l2_cxt.v4l2_state = V4L2_PREVIEW;
	g_cxt->camera_status = CMR_PREVIEW;

	return ret;
}
camera_ret_code_type camera_start_preview(camera_cb_f_type callback,
					void *client_data)
{
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("start preview");

	camera_set_client_data(client_data);
	camera_set_hal_cb(callback);

	ret = camera_start_preview_internal();

	camera_call_cb(CAMERA_RSP_CB_SUCCESS, client_data, CAMERA_FUNC_START_PREVIEW, 0);

	CMR_LOGV("start preview finished... %d", ret);
	return ret;
}

int camera_stop_preview_internal(void)
{
	int                      ret = CAMERA_SUCCESS;

	if (!IS_PREVIEW) {
		return ret;
	}

	g_cxt->pre_frm_cnt = 0;
	if (V4L2_PREVIEW == g_cxt->v4l2_cxt.v4l2_state) {
		ret = cmr_v4l2_cap_stop();
		g_cxt->v4l2_cxt.v4l2_state = V4L2_IDLE;
		if (ret) {
			CMR_LOGE("Failed to stop V4L2 capture, %d", ret);
		}
	}

	if (ISP_COWORK == g_cxt->isp_cxt.isp_state) {
/*		ret = isp_video_stop();*/
		g_cxt->isp_cxt.isp_state = ISP_IDLE;
		if (ret) {
			CMR_LOGE("Failed to stop ISP video mode, %d", ret);
		}
	}

	ret = camera_preview_stop_set();
	g_cxt->camera_status = CMR_IDLE;

	return ret;
}
camera_ret_code_type camera_stop_preview(void)
{
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("stop preview");
	ret = camera_stop_preview_internal();
	camera_call_cb(CAMERA_RSP_CB_SUCCESS,
			camera_get_client_data(),
			CAMERA_FUNC_STOP_PREVIEW,
			0);
	CMR_LOGV("stop preview... %d", ret);
exit:
	return ret;
}

int camera_take_picture_done(struct frm_info *data)
{
	camera_frame_type        frame_type;
	int                      ret = CAMERA_SUCCESS;

	ret = camera_set_frame_type(&frame_type, data);
	if (CAMERA_SUCCESS == ret) {
		camera_call_cb(CAMERA_EVT_CB_SNAPSHOT_DONE,
				camera_get_client_data(),
				CAMERA_FUNC_TAKE_PICTURE,
				(uint32_t)&frame_type);
		camera_call_cb(CAMERA_EXIT_CB_DONE,
				camera_get_client_data(),
				CAMERA_FUNC_TAKE_PICTURE,
				(uint32_t)&frame_type);
	} else {
		camera_call_cb(CAMERA_EXIT_CB_FAILED,
			camera_get_client_data(),
			CAMERA_FUNC_TAKE_PICTURE,
			0);
	}

	return ret;
}

int camera_take_picture_internal(void)
{
	int                      preview_format;
	uint32_t                 skip_number = 0;
	int                      ret = CAMERA_SUCCESS;

	ret = camera_capture_init(g_cxt->preview_fmt);
	if (ret) {
		CMR_LOGE("Fail to init preview mode.");
		return -CAMERA_FAILED;
	}
	
	CMR_LOGV("OK  to camera_capture_init.\n");

	if (0 == g_cxt->skip_mode) {
		skip_number = g_cxt->skip_num;
	}
	ret = cmr_v4l2_cap_start(skip_number);
	if (ret) {
		CMR_LOGE("Fail to start V4L2 Capture");
		return -CAMERA_FAILED;
	}
	g_cxt->v4l2_cxt.v4l2_state = V4L2_CAPTURE;
	g_cxt->camera_status = CMR_CAPTURE;

	return ret;
}

camera_ret_code_type camera_take_picture(camera_cb_f_type    callback,
					void                 *client_data)
{
	int                      ret = CAMERA_SUCCESS;

	camera_set_client_data(client_data);
	camera_set_hal_cb(callback);

	ret = camera_take_picture_internal();

	return ret;
}
camera_ret_code_type camera_stop_capture(void)
{
	int                      ret = CAMERA_SUCCESS;

	return ret;
}

camera_ret_code_type camera_cancel_autofocus(void)
{
	int                      ret = CAMERA_SUCCESS;

	ret = camera_autofocus_stop();
	return ret;
}

uint32_t camera_get_size_align_page(uint32_t size)
{
	uint32_t buffer_size, page_size;

	page_size = getpagesize();
	buffer_size = size;
	buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

	return buffer_size;
}

int camera_start_autofocus(camera_focus_e_type focus,
			camera_cb_f_type callback,
			void *client_data)
{
	CMR_MSG_INIT(message);
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("focus %d", focus);

	camera_set_client_data(client_data);
	camera_set_hal_cb(callback);

	message.msg_type = CMR_EVT_AF;
	ret = cmr_msg_post(g_cxt->msg_queue_handle, &message);

	if (ret) {
		CMR_LOGE("Fail to send message to camera main thread");
		return ret;
	}

	return ret;
}

int camera_set_frame_type(camera_frame_type *frame_type, struct frm_info* info)
{
	uint32_t                 frm_id;
	int                      ret = CAMERA_SUCCESS;

	if (NULL == frame_type || NULL == info ||
	info->frame_id < CAMERA_OEM_PREV_FRM_ID_BASE)
		return -CAMERA_INVALID_PARM;

	if (IS_PREVIEW) {
		if (g_cxt->prev_rot) {
			frm_id = g_cxt->prev_rot_index % CAMERA_OEM_PREV_ROT_FRM_CNT;
			frame_type->buf_id = CAMERA_OEM_PREV_FRM_ID_BASE + CAMERA_OEM_PREV_FRM_CNT + frm_id; // more than CAMERA_OEM_PREV_FRM_CNT
			frame_type->buf_Virt_Addr = (uint32_t*)g_cxt->prev_rot_frm[frm_id].addr_vir.addr_y;
			frame_type->buffer_phy_addr = g_cxt->prev_rot_frm[frm_id].addr_phy.addr_y;
		} else {
			frm_id = info->frame_id - CAMERA_OEM_PREV_FRM_ID_BASE;
			frame_type->buf_id = info->frame_id;
			frame_type->buf_Virt_Addr = (uint32_t*)g_cxt->prev_frm[frm_id].addr_vir.addr_y;
			frame_type->buffer_phy_addr = g_cxt->prev_frm[frm_id].addr_phy.addr_y;

		}
		frame_type->dx = g_cxt->display_size.width;
		frame_type->dy = g_cxt->display_size.height;
#if 0
		if (10 == g_cxt->pre_frm_cnt) {
			camera_save_to_file(0,
					IMG_DATA_TYPE_YUV420,
					frame_type->dx,
					frame_type->dy,
					&g_cxt->prev_frm[frm_id].addr_vir);
		}
#endif

	} else if (IS_CAPTURE) {
		frm_id = info->frame_id - CAMERA_OEM_CAP0_FRM_ID_BASE;
		frame_type->buf_Virt_Addr = (uint32_t*)g_cxt->cap_mem[frm_id].target_yuv.addr_vir.addr_y;
		frame_type->buffer_phy_addr = g_cxt->cap_mem[frm_id].target_yuv.addr_phy.addr_y;
		frame_type->Y_Addr = (uint8_t *)g_cxt->cap_mem[frm_id].target_yuv.addr_vir.addr_y;
		frame_type->CbCr_Addr = (uint8_t *)g_cxt->cap_mem[frm_id].target_yuv.addr_vir.addr_u;
		frame_type->dx = g_cxt->picture_size.width;
		frame_type->dy = g_cxt->picture_size.height;
		frame_type->captured_dx = g_cxt->picture_size.width;
		frame_type->captured_dy = g_cxt->picture_size.height;
		frame_type->rotation = 0;
		frame_type->header_size = 0;
	}
	frame_type->format = CAMERA_YCBCR_4_2_0;
        frame_type->timestamp = info->sec * 1000000000LL + info->usec * 1000;

	CMR_LOGV("index 0x%x, addr 0x%x 0x%x, w h %d %d, format %d, sec %d usec %d",
		info->frame_id,
		(uint32_t)frame_type->buf_Virt_Addr,
		frame_type->buffer_phy_addr,
		frame_type->dx,
		frame_type->dy,
		frame_type->format,
		info->sec,
		info->usec);
	
	return ret;
}

int camera_create_main_thread(void)
{
	int                      ret = CAMERA_SUCCESS;
	pthread_attr_t           attr;
	CMR_MSG_INIT(message);

	pthread_attr_init (&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

	ret = pthread_create(&g_cxt->camera_main_thr, &attr, camera_main_routine, NULL);

	if (ret) {
		CMR_LOGE("Fail to careate main thread");
		return ret;
	}

	message.msg_type = CMR_EVT_START;
	ret = cmr_msg_post(g_cxt->msg_queue_handle, &message);

	if (ret) {
		CMR_LOGE("Fail to send message to camera main thread");
		return ret;
	}

	while (0 == g_cxt->is_working) {
		CMR_LOGE("wait for the main thread to startup");
		usleep(CMR_MSG_WAIT_TIME);
	}

	return ret;
}

int camera_destroy_main_thread(void)
{
	CMR_MSG_INIT(message);
	int                      ret = CAMERA_SUCCESS;

	message.msg_type = CMR_EVT_STOP;
	ret = cmr_msg_post(g_cxt->msg_queue_handle, &message);

	while (1 == g_cxt->is_working) {
		CMR_LOGE("wait for the main thread to exit");
		usleep(CMR_MSG_WAIT_TIME);
	}

	return ret;
}

void *camera_main_routine(void *client_data)
{
	CMR_MSG_INIT(message);
	uint32_t                 evt;
	int                      ret = CAMERA_SUCCESS;

	while (1) {
		ret = cmr_msg_get(g_cxt->msg_queue_handle, &message);
		if (ret) {
			CMR_LOGE("Message queue destroied");
			break;
		}

		CMR_LOGE("message.msg_type 0x%x", message.msg_type);
		evt = (uint32_t)(message.msg_type & CMR_EVT_MASK_BITS);

		switch (evt) {

		case CMR_EVT_START:
			g_cxt->is_working = 1;
			break;

		case CMR_EVT_STOP:
			g_cxt->is_working = 0;
			break;

		case CMR_EVT_V4L2_BASE:
			ret = camera_v4l2_handle(message.msg_type, (struct frm_info *)message.data);
			break;

		case CMR_EVT_ISP_BASE:
			ret = camera_isp_handle(message.msg_type, (struct frm_info *)message.data);
			break;

		case CMR_EVT_SENSOR_BASE:
			ret = camera_sensor_handle(message.msg_type, (struct frm_info *)message.data);
			break;

		case CMR_EVT_CVT_BASE:
			ret = camera_img_cvt_handle(message.msg_type, (struct img_frm *)message.data);
			break;

		case CMR_EVT_JPEG_BASE:
			ret = camera_jpeg_codec_handle(message.msg_type, (void *)message.data);
			break;

		case CMR_EVT_AF:
			ret = camera_af_handle(message.msg_type, NULL);
			break;
		default:
			break;
		}

		if (message.alloc_flag) {
			free(message.data);
		}

		if (0 == g_cxt->is_working) {
			CMR_LOGV("Camera main thread Exit!");
			break;
		}
	}

	return NULL;

}

void camera_sensor_evt_cb(int evt, void* data)
{
	CMR_MSG_INIT(message);
	int                      ret = CAMERA_SUCCESS;

	if (NULL == data || CMR_EVT_SENSOR_BASE != (CMR_EVT_SENSOR_BASE & evt)) {
		CMR_LOGE("Error param, 0x%x 0x%x", (uint32_t)data, evt);
		return;
	}
	
	message.msg_type = evt;
	ret = cmr_msg_post(g_cxt->msg_queue_handle, &message);
	if (ret) {
		free(message.data);
		CMR_LOGE("Faile to send one msg to camera main thread");
	}

	return;
}

void camera_v4l2_evt_cb(int evt, void* data)
{
	CMR_MSG_INIT(message);
	int                      ret = CAMERA_SUCCESS;

	if (NULL == data || CMR_EVT_V4L2_BASE != (CMR_EVT_V4L2_BASE & evt)) {
		CMR_LOGE("Error param, 0x%x 0x%x", (uint32_t)data, evt);
		return;
	}

	message.data = malloc(sizeof(struct frm_info));
	if (NULL == message.data) {
		CMR_LOGE("NO mem, Faile to alloc memory for one msg");
		return;
	}
	message.msg_type = evt;
	message.alloc_flag = 1;
	memcpy(message.data, data, sizeof(struct frm_info));
	ret = cmr_msg_post(g_cxt->msg_queue_handle, &message);
	if (ret) {
		free(message.data);
		CMR_LOGE("Faile to send one msg to camera main thread");
	}

	return;
}

void camera_isp_evt_cb(int evt, void* data)
{
	CMR_MSG_INIT(message);
	int                      ret = CAMERA_SUCCESS;

	if (NULL == data || CMR_EVT_ISP_BASE != (CMR_EVT_ISP_BASE & evt)) {
		CMR_LOGE("Error param, 0x%x 0x%x", (uint32_t)data, evt);
		return;
	}

	/* TBD
	
	message.data = malloc(sizeof(struct frm_info));
	if (NULL == message.data) {
		CMR_LOGE("NO mem, Faile to alloc memory for one msg");
		return;
	}
	message.msg_type = evt;
	message.alloc_flag = 1;
	bcopy(message.data, data, sizeof(struct frm_info));
	*/
	ret = cmr_msg_post(g_cxt->msg_queue_handle, &message);
	if (ret) {
		free(message.data);
		CMR_LOGE("Faile to send one msg to camera main thread");
	}

	return;
}

void camera_jpeg_evt_cb(int evt, void* data)
{
	CMR_MSG_INIT(message);
	int                      ret = CAMERA_SUCCESS;

	if (NULL == data || CMR_EVT_JPEG_BASE != (CMR_EVT_JPEG_BASE & evt)) {
		CMR_LOGE("Error param, 0x%x 0x%x", (uint32_t)data, evt);
		return;
	}

	message.data = malloc(sizeof(JPEG_ENC_CB_PARAM_T));
	if (NULL == message.data) {
		CMR_LOGE("NO mem, Faile to alloc memory for one msg");
		return;
	}
	message.msg_type = evt;
	message.alloc_flag = 1;
	memcpy(message.data, data, sizeof(JPEG_ENC_CB_PARAM_T));
	CMR_LOGV("evt 0x%x", evt);
	ret = cmr_msg_post(g_cxt->msg_queue_handle, &message);
	if (ret) {
		free(message.data);
		CMR_LOGE("Faile to send one msg to camera main thread");
	}

	return;
}

void camera_rot_evt_cb(int evt, void* data)
{
	CMR_MSG_INIT(message);
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("0x%x", evt);
	if ((NULL == data) || (CMR_IMG_CVT_ROT_DONE != evt)) {
		CMR_LOGE("Error param, 0x%x 0x%x", (uint32_t)data, evt);
		return;
	}

	message.data = malloc(sizeof(struct img_frm));
	if (NULL == message.data) {
		CMR_LOGE("NO mem, Faile to alloc memory for one msg");
		return;
	}
	message.msg_type = evt;
	message.alloc_flag = 1;
	memcpy(message.data, data, sizeof(struct img_frm));
	ret = cmr_msg_post(g_cxt->msg_queue_handle, &message);
	if (ret) {
		free(message.data);
		CMR_LOGE("Faile to send one msg to camera main thread");
	}

	return;
}

void camera_scaler_evt_cb(int evt, void* data)
{
	CMR_MSG_INIT(message);
	struct img_frm           frame;
	int                      ret = CAMERA_SUCCESS;

	if (NULL == data || CMR_EVT_CVT_BASE != (CMR_EVT_CVT_BASE & evt)) {
		CMR_LOGE("Error param, 0x%x 0x%x", (uint32_t)data, evt);
		return;
	}

	message.data = malloc(sizeof(struct img_frm));
	if (NULL == message.data) {
		CMR_LOGE("NO mem, Faile to alloc memory for one msg");
		return;
	}
	message.msg_type = evt;
	message.alloc_flag = 1;
	memcpy(message.data, data, sizeof(struct img_frm));
	ret = cmr_msg_post(g_cxt->msg_queue_handle, &message);
	if (ret) {
		free(message.data);
		CMR_LOGE("Faile to send one msg to camera main thread");
	}

	return;
	
}

int camera_v4l2_handle(uint32_t evt_type, struct frm_info *data)
{
	int                      ret = CAMERA_SUCCESS;

	if (NULL == data) {
		CMR_LOGE("Parameter error");
		return -CAMERA_INVALID_PARM;
	}

	if (CMR_IDLE == g_cxt->camera_status || CMR_ERR == g_cxt->camera_status) {
		CMR_LOGE("Status error");
		return -CAMERA_INVALID_STATE;
	}

	switch (evt_type) {
	case CMR_V4L2_TX_DONE:
		if (IS_PREVIEW) {
			ret = camera_v4l2_preview_handle(data);
		} else if (IS_CAPTURE) {
			ret = camera_v4l2_capture_handle(data);
		}
		break;

	case CMR_V4L2_TX_NO_MEM:
		break;
	default:
		break;

	}

	return ret;
}

int camera_isp_handle(uint32_t evt_type, struct frm_info *data)
{
	int                      ret = CAMERA_SUCCESS;
	
	return ret;

}

int camera_jpeg_encode_handle(JPEG_ENC_CB_PARAM_T *data)
{
	JPEG_ENC_NXT_PARAM       enc_nxt_param;
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("slice hegith %d index %d, stream size %d addr 0x%x",
		data->slice_height,
		g_cxt->jpeg_cxt.index,
		data->stream_size,
		data->stream_buf_vir);

	CMR_LOGV("stream buf 0x%x size 0x%x",
		g_cxt->cap_mem[g_cxt->jpeg_cxt.index].target_jpeg.addr_vir.addr_y,
		data->stream_size);

	if (NULL == data || 0 == data->slice_height) {
		return CAMERA_INVALID_PARM;
	}
	g_cxt->jpeg_cxt.proc_status.slice_height_out += data->slice_height;
	if (g_cxt->jpeg_cxt.proc_status.slice_height_out == g_cxt->picture_size.height) {
		g_cxt->cap_mem[g_cxt->jpeg_cxt.index].target_jpeg.addr_vir.addr_u = data->stream_size;
		ret = camera_save_to_file(2,
			IMG_DATA_TYPE_JPEG,
			g_cxt->picture_size.width,
			g_cxt->picture_size.height,
			&g_cxt->cap_mem[g_cxt->jpeg_cxt.index].target_jpeg.addr_vir);
		ret = camera_jpeg_encode_done(NULL);
		ret = jpeg_stop(g_cxt->jpeg_cxt.handle);
		g_cxt->jpeg_cxt.jpeg_state = JPEG_IDLE;

	} else {
		bzero(&enc_nxt_param, sizeof(JPEG_ENC_NXT_PARAM));
		enc_nxt_param.handle       = g_cxt->jpeg_cxt.handle;
		enc_nxt_param.slice_height = g_cxt->jpeg_cxt.proc_status.slice_height_in;
		ret = jpeg_enc_next(&enc_nxt_param);
	}
	return ret;

}

int camera_jpeg_decode_handle(JPEG_DEC_CB_PARAM_T *data)
{
	int                      ret = CAMERA_SUCCESS;

	return ret;

}
int camera_jpeg_codec_handle(uint32_t evt_type, void *data)
{
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("status %d, evt 0x%x, data 0x%x",
		g_cxt->jpeg_cxt.jpeg_state, evt_type, (uint32_t)data);

	if (JPEG_ENCODE == g_cxt->jpeg_cxt.jpeg_state) {
		ret = camera_jpeg_encode_handle((JPEG_ENC_CB_PARAM_T*)data);
	} else {
		ret = camera_jpeg_decode_handle((JPEG_DEC_CB_PARAM_T*)data);
	}

	return ret;
}

int camera_scale_handle(uint32_t evt_type, struct img_frm *data)
{
	int                      ret = CAMERA_SUCCESS;
	struct scaler_context    *cxt = &g_cxt->scaler_cxt;

	CMR_LOGV("channel id %d, slice %d", 
		cxt->proc_status.frame_info.channel_id,
		data->size.height);
	
	if (IMG_CVT_SCALING != g_cxt->scaler_cxt.scale_state) {
		CMR_LOGE("Error state %d", g_cxt->scaler_cxt.scale_state);
		return CAMERA_INVALID_STATE;
	}

	if (0 == cxt->proc_status.frame_info.channel_id) {
		cxt->proc_status.slice_height_out += data->size.height;
		if (cxt->proc_status.slice_height_out < g_cxt->capture_size.height) {
			CMR_LOGV("Need more slice");
			ret = camera_scale_next(&cxt->proc_status.frame_info);
		} else {
			CMR_LOGV("Scaling done");
			ret = camera_scale_done(&cxt->proc_status.frame_info);
		}
	} else {
		ret = camera_scale_done(&cxt->proc_status.frame_info);
	}

	return ret;

}


int camera_rotation_handle(uint32_t evt_type, struct img_frm *data)
{
	camera_frame_type        frame_type;
	struct rotation_context  *cxt = &g_cxt->rot_cxt;
	struct frm_info          *info = &cxt->proc_status.frame_info;
	uint32_t                 frm_id;
	uint32_t                 tmp;
	int                      ret = CAMERA_SUCCESS;

	if (IMG_CVT_ROTATING != g_cxt->rot_cxt.rot_state) {
		CMR_LOGE("Error state %d", g_cxt->rot_cxt.rot_state);
		return CAMERA_INVALID_STATE;
	}

	if (IS_PREVIEW) {
		/* the source frame can be freed here*/
		CMR_LOGV("Rot Done");
		ret = cmr_v4l2_free_frame(0, info->frame_id);
		if (ret) {
			CMR_LOGE("Failed to free frame, %d, %d", info->frame_id, info->channel_id);
			goto exit;
		}
		ret = camera_set_frame_type(&frame_type, info);
		if (ret) {
			CMR_LOGE("Failed to set frame type, %d, %d", info->frame_id, info->channel_id);
			goto exit;
		}

		g_cxt->prev_rot_index ++;
		g_cxt->rot_cxt.rot_state = IMG_CVT_ROT_DONE;
		camera_call_cb(CAMERA_EVT_CB_FRAME,
				camera_get_client_data(),
				CAMERA_FUNC_START_PREVIEW,
				(uint32_t)&frame_type);
		
	}else {
		tmp = g_cxt->cap_orig_size.width;
		g_cxt->cap_orig_size.width = g_cxt->cap_orig_size.height;
		g_cxt->cap_orig_size.height = tmp;
		/*
		camera_save_to_file(1, IMG_DATA_TYPE_YUV420,
					g_cxt->cap_orig_size.width,
					g_cxt->cap_orig_size.height,
					&g_cxt->cap_mem[0].target_yuv.addr_vir);
		*/
		if (g_cxt->cap_orig_size.width == g_cxt->picture_size.width &&
			g_cxt->cap_orig_size.height == g_cxt->picture_size.height) {
			ret = camera_start_jpeg_encode(info);
			if (ret) {
				CMR_LOGE("Failed to start jpeg encode %d", ret);
				return -CAMERA_FAILED;
			}

			if (0 == info->channel_id) {
				frm_id = info->frame_id - CAMERA_OEM_CAP0_FRM_ID_BASE;
				ret = camera_take_picture_done(info);
				if (ret) {
					CMR_LOGE("Failed to set take_picture done %d", ret);
					return -CAMERA_FAILED;
				}
			} else {
				frm_id = info->frame_id - CAMERA_OEM_CAP1_FRM_ID_BASE;
			}
		} else {
			ret = camera_start_scale(info);
		}
	}

exit:
	return ret;
	
}

int camera_sensor_handle(uint32_t evt_type, struct frm_info *data)
{
	int                      ret = CAMERA_SUCCESS;
	
	return ret;

}

int camera_img_cvt_handle(uint32_t evt_type, struct img_frm *data)
{
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("evt 0x%x", evt_type);
	if (CMR_IMG_CVT_SC_DONE == evt_type) {
		ret = camera_scale_handle(evt_type, data);
	} else {
		ret = camera_rotation_handle(evt_type, data);
	}
	return ret;
}

int camera_af_handle(uint32_t evt_type, struct img_frm *data)
{
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("evt 0x%x", evt_type);

	// ret = camera_autofocus_start();
	if (CAMERA_INVALID_STATE == ret) {
		camera_call_cb(CAMERA_EXIT_CB_ABORT,
			camera_get_client_data(),
			CAMERA_FUNC_START_FOCUS,
			0);
	} else if (CAMERA_FAILED == ret) {
		camera_call_cb(CAMERA_EXIT_CB_FAILED,
			camera_get_client_data(),
			CAMERA_FUNC_START_FOCUS,
			0);

	} else {
		camera_call_cb(CAMERA_EXIT_CB_DONE,
			camera_get_client_data(),
			CAMERA_FUNC_START_FOCUS,
			0);
	}

	return ret;
}


void camera_set_hal_cb(camera_cb_f_type cmr_cb)
{
	pthread_mutex_lock(&g_cxt->cb_mutex);
	g_cxt->camera_cb = cmr_cb;
	pthread_mutex_unlock(&g_cxt->cb_mutex);
	return;
}

void camera_call_cb(camera_cb_type cb,
                 const void *client_data,
                 camera_func_type func,
                 int32_t parm4)
{
	pthread_mutex_lock(&g_cxt->cb_mutex);
	if (g_cxt->camera_cb) {
		(*g_cxt->camera_cb)(cb, client_data, func, parm4);
	}
	pthread_mutex_unlock(&g_cxt->cb_mutex);
	return;
}

void camera_set_client_data(void* user_data)
{
	pthread_mutex_lock(&g_cxt->data_mutex);
	g_cxt->client_data = user_data;
	pthread_mutex_unlock(&g_cxt->data_mutex);
	return;
}

void *camera_get_client_data(void)
{
	void                     *user_data = NULL;
	
	pthread_mutex_lock(&g_cxt->data_mutex);
	user_data = g_cxt->client_data;
	pthread_mutex_unlock(&g_cxt->data_mutex);
	return user_data;
}

int camera_preview_init(int format_mode)
{
	int                      ret = CAMERA_SUCCESS;
	struct cap_cfg           v4l2_cfg;
	SENSOR_MODE_INFO_T       *sensor_mode;
	struct buffer_cfg        buffer_info;
/*	struct isp_video_start   isp_param;*/

	sensor_mode = &g_cxt->sn_cxt.sensor_info->sensor_mode_info[g_cxt->sn_cxt.preview_mode];

	g_cxt->prev_rot_index = 0;
	g_cxt->skip_mode = IMG_SKIP_HW;
	g_cxt->skip_num  = g_cxt->sn_cxt.sensor_info->preview_skip_num;
	g_cxt->pre_frm_cnt  = 0;
	v4l2_cfg.cfg0.need_isp = 0;
	if (SENSOR_IMAGE_FORMAT_YUV422 == sensor_mode->image_format) {
		g_cxt->sn_cxt.sn_if.img_fmt = V4L2_SENSOR_FORMAT_YUV;
	} else if (SENSOR_IMAGE_FORMAT_RAW == sensor_mode->image_format) {
		g_cxt->sn_cxt.sn_if.img_fmt = V4L2_SENSOR_FORMAT_RAWRGB;
		g_cxt->skip_mode = IMG_SKIP_SW;
		v4l2_cfg.cfg0.need_isp = 1;
	} else {
		CMR_LOGE("Unsupported sensor format %d for preview", sensor_mode->image_format);
		ret = -CAMERA_INVALID_FORMAT;
		goto exit;
	}

	CMR_LOGI("sensor output, width, hegiht %d %d", sensor_mode->width, sensor_mode->height);
	g_cxt->sn_cxt.sn_if.size.width  = sensor_mode->width;
	g_cxt->sn_cxt.sn_if.size.height = sensor_mode->height;
	ret = cmr_v4l2_if_cfg(&g_cxt->sn_cxt.sn_if);
	if (ret) {
		CMR_LOGE("the sensor interface is unsupported by V4L2");
		goto exit;
	}
	v4l2_cfg.channel_num = 1;
	v4l2_cfg.frm_num = -1;
	v4l2_cfg.cfg0.dst_img_size.width   = g_cxt->preview_size.width;
	v4l2_cfg.cfg0.dst_img_size.height  = g_cxt->preview_size.height;
	v4l2_cfg.cfg0.notice_slice_height  = v4l2_cfg.cfg0.dst_img_size.height;
	v4l2_cfg.cfg0.src_img_rect.start_x = sensor_mode->trim_start_x;
	v4l2_cfg.cfg0.src_img_rect.start_y = sensor_mode->trim_start_y;
	v4l2_cfg.cfg0.src_img_rect.width   = sensor_mode->trim_width;
	v4l2_cfg.cfg0.src_img_rect.height  = sensor_mode->trim_height;
	ret = camera_get_trim_rect(&v4l2_cfg.cfg0.src_img_rect, g_cxt->zoom_level, &v4l2_cfg.cfg0.dst_img_size);
	if (ret) {
		CMR_LOGE("Failed to get trimming window for %d zoom level ", g_cxt->zoom_level);
		goto exit;
	}

	v4l2_cfg.cfg0.dst_img_fmt = camera_get_img_type(format_mode);
	ret = cmr_v4l2_cap_cfg(&v4l2_cfg);
	if (ret) {
		CMR_LOGE("Can't support this capture configuration");
		goto exit;
	}

	ret = camera_alloc_preview_buf(&buffer_info, v4l2_cfg.cfg0.dst_img_fmt);
	if (ret) {
		CMR_LOGE("Failed to alloc preview buffer");
		goto exit;
	}

	ret = cmr_v4l2_buff_cfg(&buffer_info);
	if (ret) {
		CMR_LOGE("Failed to Q preview buffer");
		goto exit;
	}

	if (v4l2_cfg.cfg0.need_isp) {
/*		isp_param.size.w = sensor_mode->width;
		isp_param.size.h = sensor_mode->height;*/
		/*isp_param.format =  wjp*/
/*		ret = isp_video_start(&isp_param);*/
		if (CAMERA_SUCCESS == ret) {
			g_cxt->isp_cxt.isp_state = ISP_COWORK;
		}
	}
	
exit:
	return ret;
}

int camera_capture_init(int format_mode)
{
	struct img_size          capture_size;
	int                      ret = CAMERA_SUCCESS;
	struct cap_cfg           v4l2_cfg;
	SENSOR_MODE_INFO_T       *sensor_mode;
	struct buffer_cfg        buffer_info;
/*	struct isp_video_start   isp_video_param;*/

	CMR_LOGV("capture size, %d %d, cap_rot %d",
		g_cxt->capture_size.width,
		g_cxt->capture_size.height,
		g_cxt->cap_rot);

	g_cxt->total_cap_num = 1;
	g_cxt->cap_cnt = 0;
	g_cxt->thum_ready = 0;
	ret = camera_snapshot_start_set();
	if (ret) {
		CMR_LOGE("Failed to snapshot");
		goto exit;
	}

	sensor_mode = &g_cxt->sn_cxt.sensor_info->sensor_mode_info[g_cxt->sn_cxt.capture_mode];
	g_cxt->sn_cxt.sn_if.size.width  = sensor_mode->width;
	g_cxt->sn_cxt.sn_if.size.height = sensor_mode->height;
	ret = cmr_v4l2_if_cfg(&g_cxt->sn_cxt.sn_if);
	if (ret) {
		CMR_LOGE("the sensor interface is unsupported by V4L2");
		goto exit;
	}	

	v4l2_cfg.channel_num = 1;
	v4l2_cfg.frm_num = 1; 
	/* It can be more than 1*/

	ret = camera_capture_ability(sensor_mode, &v4l2_cfg.cfg0, &g_cxt->capture_size);
/*
	if (IMG_DATA_TYPE_YUV420 == g_cxt->cap_original_fmt ||
		IMG_DATA_TYPE_RAW == g_cxt->cap_original_fmt) {
		CMR_LOGV("YUV420 or RAW can be output from channel 0, maybe it support YUV420 in channel 1");
		v4l2_cfg.cfg1.dst_img_fmt = IMG_DATA_TYPE_YUV420;
		v4l2_cfg.cfg1.dst_img_size.width  = g_cxt->thum_size.width;
		v4l2_cfg.cfg1.dst_img_size.height = g_cxt->thum_size.height;
		v4l2_cfg.cfg1.src_img_rect.start_x = sensor_mode->trim_start_x;
		v4l2_cfg.cfg1.src_img_rect.start_y = sensor_mode->trim_start_y;
		v4l2_cfg.cfg1.src_img_rect.width   = sensor_mode->trim_width;
		v4l2_cfg.cfg1.src_img_rect.height  = sensor_mode->trim_height;
		ret = camera_get_trim_rect(&v4l2_cfg.cfg1.src_img_rect, g_cxt->zoom_level, &g_cxt->thum_size);
		if (0 == ret) {
			v4l2_cfg.channel_num += 1;
		}
	}
*/
	g_cxt->total_cap_ch_num = v4l2_cfg.channel_num;
	g_cxt->cap_ch_cnt = 0;
	ret = cmr_v4l2_cap_cfg(&v4l2_cfg);
	if (ret) {
		CMR_LOGE("Can't support this capture configuration");
		goto exit;
	} else {
		if (1 == v4l2_cfg.channel_num) {
			CMR_LOGV("Only one channel supported");
			g_cxt->thum_from = THUM_FROM_SCALER;
		} else {
			g_cxt->thum_from = THUM_FROM_CAP;
		}
	}

	ret = camera_alloc_capture_buf0(&buffer_info, g_cxt->cap_cnt);
	if (ret) {
		CMR_LOGE("Failed to alloc capture buffer");
		goto exit;
	}

	ret = cmr_v4l2_buff_cfg(&buffer_info);
	if (ret) {
		CMR_LOGE("Failed to Q capture buffer");
		goto exit;
	}

	if (THUM_FROM_CAP == g_cxt->thum_from) {
		bzero(&buffer_info, sizeof(buffer_info));
		ret = camera_alloc_capture_buf1(&buffer_info, g_cxt->cap_cnt);
		if (ret) {
			CMR_LOGE("Failed to alloc capture buffer");
			goto exit;
		}

		ret = cmr_v4l2_buff_cfg(&buffer_info);
		if (ret) {
			CMR_LOGE("Failed to Q preview buffer");
			goto exit;
		}
	}

	if (v4l2_cfg.cfg0.need_isp) {
/*		isp_video_param.size.w = sensor_mode->width;/*in capture mode, if raw rgb , it can't use binning;*/
//		isp_video_param.size.h = sensor_mode->height;*/
/*		isp_video_param.format wjp*/
/*		ret = isp_video_start(&isp_video_param);*/
		if (CAMERA_SUCCESS == ret) {
			g_cxt->isp_cxt.isp_state = ISP_COWORK;
		}
	}

exit:
	return ret;
}

int camera_get_sensor_capture_mode(struct img_size* target_size, uint32_t *work_mode)
{
	uint32_t                 width = 0, theight = 0, i;
	uint32_t                 search_width = target_size->width;
	uint32_t                 target_mode = SENSOR_MODE_MAX;
	SENSOR_EXP_INFO_T        *sn_info = g_cxt->sn_cxt.sensor_info;
	int                      ret = -CAMERA_FAILED;

	if (NULL == target_size || NULL == g_cxt->sn_cxt.sensor_info)
		return ret;
		
	CMR_LOGV("search_width = %d", search_width);
	for (i = SENSOR_MODE_PREVIEW_ONE; i < SENSOR_MODE_MAX; i++) {
		if (SENSOR_MODE_MAX != sn_info->sensor_mode_info[i].mode) {
			width = sn_info->sensor_mode_info[i].width;
			CMR_LOGV("width = %d", width);
			if (search_width <= width) {
				target_mode = i;
				ret = CAMERA_SUCCESS;
				break;
			}
		}
	}

	if (i == SENSOR_MODE_MAX) {
		CMR_LOGV("can't find the right mode");
	}

	*work_mode = target_mode;
	CMR_LOGV("mode %d, width %d height %d",
		target_mode,
		sn_info->sensor_mode_info[i].width,
		sn_info->sensor_mode_info[i].height);

	return ret;
}

int camera_get_sensor_preview_mode(struct img_size* target_size, uint32_t *work_mode)
{
	uint32_t                 width = 0, theight = 0, i;
	uint32_t                 search_width = target_size->width;
	uint32_t                 target_mode = SENSOR_MODE_MAX;
	SENSOR_EXP_INFO_T        *sn_info = g_cxt->sn_cxt.sensor_info;
	int                      ret = -CAMERA_FAILED;

	if (NULL == target_size || NULL == g_cxt->sn_cxt.sensor_info || NULL == work_mode)
		return ret;
		
	CMR_LOGV("search_width = %d", search_width);
	for (i = SENSOR_MODE_PREVIEW_ONE; i < SENSOR_MODE_MAX; i++) {
		if (SENSOR_MODE_MAX != sn_info->sensor_mode_info[i].mode) {
			width = sn_info->sensor_mode_info[i].width;
			CMR_LOGV("width = %d", width);
			if (search_width <= width) {
				if(sn_info->sensor_mode_info[i].mode < SENSOR_MODE_PREVIEW_TWO) {
					target_mode = SENSOR_MODE_PREVIEW_ONE;
				} else {
					target_mode = SENSOR_MODE_PREVIEW_TWO;
				}
				ret = CAMERA_SUCCESS;
				break;
			}
			
		}
		
	}

	if (i == SENSOR_MODE_MAX) {
		CMR_LOGV("can't find the right mode");
	}

	*work_mode = target_mode;
	return ret;
}

int camera_alloc_preview_buf(struct buffer_cfg *buffer, uint32_t format)
{
	uint32_t                 size, total_size, buffer_size, frame_size, i, base_addr;
	int                      ret = CAMERA_SUCCESS;

	if (NULL == buffer)
		return -CAMERA_INVALID_PARM;

	buffer_size = g_cxt->display_size.width * g_cxt->display_size.height;
	if (IMG_DATA_TYPE_YUV420 == format) {
		frame_size = buffer_size * 3 / 2;
	} else if (IMG_DATA_TYPE_YUV422 == format) {
		frame_size = buffer_size * 2;
	} else {
		CMR_LOGE("Unsupported format %d", format);
		return -CAMERA_INVALID_PARM;
	}

	size = frame_size * CAMERA_OEM_PREV_FRM_CNT;
	
	if (NULL == g_cxt->prev_virt_addr || 0 == g_cxt->prev_phys_addr) {
		CMR_LOGE("Fail to malloc preview memory.");
		ret = -CAMERA_FAILED;
	}

	total_size = CAMERA_OEM_PREV_FRM_CNT;
	if (g_cxt->prev_rot) {
		total_size += CAMERA_OEM_PREV_ROT_FRM_CNT;
	}
	total_size = total_size * frame_size;

	if (total_size > g_cxt->prev_mem_szie) {
		return -CAMERA_NO_MEMORY;
	}

	CMR_LOGV("preview addr, vir 0x%x phy 0x%x", (uint32_t)g_cxt->prev_virt_addr, g_cxt->prev_phys_addr);
	buffer->channel_id = 0;
	buffer->base_id    = CAMERA_OEM_PREV_FRM_ID_BASE;
	buffer->count      = CAMERA_OEM_PREV_FRM_CNT;
	buffer->length     = frame_size;
	bzero((void*)&buffer->addr[0], (uint32_t)(V4L2_BUF_MAX*sizeof(struct img_addr)));
	for (i = 0; i < buffer->count; i++) {
		g_cxt->prev_frm[i].addr_vir.addr_y = (uint32_t)g_cxt->prev_virt_addr + (uint32_t)(i * frame_size);
		g_cxt->prev_frm[i].addr_vir.addr_u = g_cxt->prev_frm[i].addr_vir.addr_y + buffer_size;
		
		g_cxt->prev_frm[i].addr_phy.addr_y = (uint32_t)g_cxt->prev_phys_addr + (uint32_t)(i * frame_size);
		g_cxt->prev_frm[i].addr_phy.addr_u = g_cxt->prev_frm[i].addr_phy.addr_y + buffer_size;
		g_cxt->prev_frm[i].fmt             = format;
		g_cxt->prev_frm[i].size.width      = g_cxt->preview_size.width;
		g_cxt->prev_frm[i].size.height     = g_cxt->preview_size.height;
		buffer->addr[i].addr_y = g_cxt->prev_frm[i].addr_phy.addr_y;
		buffer->addr[i].addr_u = g_cxt->prev_frm[i].addr_phy.addr_u;
	}

	if (g_cxt->prev_rot) {
		for (i = 0; i < CAMERA_OEM_PREV_ROT_FRM_CNT; i++) {
			g_cxt->prev_rot_frm[i].addr_vir.addr_y =
				(uint32_t)g_cxt->prev_virt_addr + (uint32_t)((i + buffer->count) * frame_size);
			g_cxt->prev_rot_frm[i].addr_vir.addr_u = g_cxt->prev_rot_frm[i].addr_vir.addr_y + buffer_size;

			g_cxt->prev_rot_frm[i].addr_phy.addr_y =
				(uint32_t)g_cxt->prev_phys_addr + (uint32_t)((i + buffer->count) * frame_size);
			g_cxt->prev_rot_frm[i].addr_phy.addr_u = g_cxt->prev_rot_frm[i].addr_phy.addr_y + buffer_size;
			g_cxt->prev_rot_frm[i].fmt             = format;
			g_cxt->prev_frm[i].size.width      = g_cxt->display_size.width;
			g_cxt->prev_frm[i].size.height     = g_cxt->display_size.height;

		}
	}
	
	return ret;
}

int camera_capture_ability(SENSOR_MODE_INFO_T *sn_mode,
					struct img_frm_cap *img_cap,
					struct img_size *cap_size)
{
	int                      ret = CAMERA_SUCCESS;

	img_cap->need_isp = 0;
	if (SENSOR_IMAGE_FORMAT_YUV422 == sn_mode->image_format) {
		g_cxt->sn_cxt.sn_if.img_fmt = V4L2_SENSOR_FORMAT_YUV;
		g_cxt->cap_original_fmt = IMG_DATA_TYPE_YUV420;
		g_cxt->cap_zoom_mode = ZOOM_BY_CAP;
	} else if (SENSOR_IMAGE_FORMAT_RAW == sn_mode->image_format) {
	
		if (IMG_DATA_TYPE_RAW == img_cap->dst_img_fmt) {
			CMR_LOGV("Get RawData From RawRGB senosr");
		} else {
			if (cap_size->width < g_cxt->isp_cxt.width_limit) {
				CMR_LOGV("Need ISP to work at video mode");
				img_cap->need_isp = 1;
				g_cxt->cap_original_fmt = IMG_DATA_TYPE_YUV420;
				g_cxt->cap_zoom_mode = ZOOM_BY_CAP;

			} else {
				img_cap->need_isp = 0;
				g_cxt->cap_original_fmt = IMG_DATA_TYPE_RAW;
				g_cxt->cap_zoom_mode = ZOOM_POST_PROCESS;
			}
		}
		g_cxt->sn_cxt.sn_if.img_fmt = V4L2_SENSOR_FORMAT_RAWRGB;
	} else if (SENSOR_IMAGE_FORMAT_JPEG == sn_mode->image_format) {
		g_cxt->sn_cxt.sn_if.img_fmt = V4L2_SENSOR_FORMAT_JPEG;
		g_cxt->cap_original_fmt = IMG_DATA_TYPE_JPEG;
		g_cxt->cap_zoom_mode = ZOOM_POST_PROCESS;
	} else {
		CMR_LOGE("Unsupported sensor format %d for capture", sn_mode->image_format);
		ret = -CAMERA_INVALID_FORMAT;
		goto exit;
	}

	img_cap->dst_img_fmt = g_cxt->cap_original_fmt;
	if (ZOOM_POST_PROCESS == g_cxt->cap_zoom_mode) {
		img_cap->dst_img_size.width   = sn_mode->width;
		img_cap->dst_img_size.height  = sn_mode->height;
		img_cap->notice_slice_height  = sn_mode->height;
		img_cap->src_img_rect.start_x = 0;
		img_cap->src_img_rect.start_y = 0;
		img_cap->src_img_rect.width   = sn_mode->width;
		img_cap->src_img_rect.height  = sn_mode->height;
		g_cxt->cap_orig_size.width  = img_cap->src_img_rect.width;
		g_cxt->cap_orig_size.height = img_cap->src_img_rect.height;
		CMR_LOGV("zoomed by post process, sample the image in original size, %d %d",
			g_cxt->cap_orig_size.width,
			g_cxt->cap_orig_size.height);
	} else {
		/* zoomed by CAP sub-module of DCAM */
		img_cap->notice_slice_height  = sn_mode->trim_height;
		img_cap->src_img_rect.start_x = sn_mode->trim_start_x;
		img_cap->src_img_rect.start_y = sn_mode->trim_start_y;
		img_cap->src_img_rect.width   = sn_mode->trim_width;
		img_cap->src_img_rect.height  = sn_mode->trim_height;
		ret = camera_get_trim_rect(&img_cap->src_img_rect, g_cxt->zoom_level, cap_size);
		if (ret) {
			CMR_LOGE("Failed to get trimming window for %d zoom level ", g_cxt->zoom_level);
			goto exit;
		}
		if (g_cxt->capture_size.width <= g_cxt->v4l2_cxt.sc_capability) {
			img_cap->dst_img_size.width   = g_cxt->capture_size.width;
			img_cap->dst_img_size.height  = g_cxt->capture_size.height;
		} else {
			img_cap->dst_img_size.width   = img_cap->src_img_rect.width;
			img_cap->dst_img_size.height  = img_cap->src_img_rect.height;
		}
		g_cxt->cap_orig_size.width    = img_cap->dst_img_size.width;
		g_cxt->cap_orig_size.height   = img_cap->dst_img_size.height;
		CMR_LOGV("zoomed by CAP sub-module of DCAM, original size %d %d",
			g_cxt->cap_orig_size.width,
			g_cxt->cap_orig_size.height);
	}

exit:
	return ret;
}

int camera_alloc_capture_buf0(struct buffer_cfg *buffer, uint32_t cap_index)
{
	uint32_t                 mem_size, buffer_size, frame_size, y_addr, u_addr;
	int                      ret = CAMERA_SUCCESS;

	if (NULL == buffer)
		return -CAMERA_INVALID_PARM;

	if (IMG_DATA_TYPE_RAW == g_cxt->cap_original_fmt) {
		mem_size = g_cxt->cap_mem[cap_index].cap_raw.buf_size;
		y_addr   = g_cxt->cap_mem[cap_index].cap_raw.addr_phy.addr_y;
		u_addr   = y_addr;
	} else if (IMG_DATA_TYPE_JPEG == g_cxt->cap_original_fmt) {
		mem_size = g_cxt->cap_mem[cap_index].target_jpeg.buf_size;
		y_addr   = g_cxt->cap_mem[cap_index].target_jpeg.addr_phy.addr_y;
		u_addr   = y_addr;
	} else if (IMG_DATA_TYPE_YUV420 == g_cxt->cap_original_fmt) {
		if (IMG_ROT_0 == g_cxt->cap_rot) {
			if (g_cxt->cap_orig_size.width == g_cxt->capture_size.width &&
				g_cxt->cap_orig_size.height == g_cxt->capture_size.height) {
				/* It means no need to scale up before JPEG encode */
				mem_size = g_cxt->cap_mem[cap_index].target_yuv.buf_size;
				y_addr   = g_cxt->cap_mem[cap_index].target_yuv.addr_phy.addr_y;
				u_addr   = g_cxt->cap_mem[cap_index].target_yuv.addr_phy.addr_u;
			} else {
				mem_size = g_cxt->cap_mem[cap_index].cap_yuv.buf_size;
				y_addr   = g_cxt->cap_mem[cap_index].cap_yuv.addr_phy.addr_y;
				u_addr   = g_cxt->cap_mem[cap_index].cap_yuv.addr_phy.addr_u;
			}
		} else {
			mem_size = g_cxt->cap_mem[cap_index].cap_yuv_rot.buf_size;
			y_addr   = g_cxt->cap_mem[cap_index].cap_yuv_rot.addr_phy.addr_y;
			u_addr   = g_cxt->cap_mem[cap_index].cap_yuv_rot.addr_phy.addr_u;
		}
	} else {
		CMR_LOGE("Unsupported capture format!");
		return -CAMERA_NOT_SUPPORTED;
	}
	
	buffer_size = g_cxt->capture_size.width * g_cxt->capture_size.height;
	frame_size = buffer_size * 3 / 2;

	if (0 == y_addr || 0 == u_addr || frame_size > mem_size) {
		CMR_LOGE("Fail to malloc capture memory.");
		return -CAMERA_NO_MEMORY;
	}

	CMR_LOGV("capture addr, y 0x%x uv 0x%x", y_addr, u_addr);
	buffer->channel_id = 0;
	buffer->base_id    = CAMERA_OEM_CAP0_FRM_ID_BASE + cap_index;
	buffer->count      = 1;	
	buffer->length     = frame_size;
	buffer->addr[0].addr_y = y_addr;
	buffer->addr[0].addr_u = u_addr;

	return ret;
}

int camera_alloc_capture_buf1(struct buffer_cfg *buffer, uint32_t cap_index)
{
	uint32_t                 mem_size, buffer_size, frame_size, y_addr, u_addr;
	int                      ret = CAMERA_SUCCESS;

	if (NULL == buffer)
		return -CAMERA_INVALID_PARM;

	if (IMG_DATA_TYPE_YUV420 == g_cxt->cap_original_fmt) {
		mem_size = g_cxt->cap_mem[cap_index].thum_yuv.buf_size;
		y_addr   = g_cxt->cap_mem[cap_index].thum_yuv.addr_phy.addr_y;
		u_addr   = g_cxt->cap_mem[cap_index].thum_yuv.addr_phy.addr_u;
	} else {
		CMR_LOGE("Unsupported capture format for channel 1!");
		return -CAMERA_NOT_SUPPORTED;
	}
	
	buffer_size = g_cxt->thum_size.width * g_cxt->thum_size.height;
	frame_size = buffer_size * 3 / 2;

	if (0 == y_addr || 0 == u_addr || frame_size > mem_size) {
		CMR_LOGE("Fail to malloc capture memory.");
		return -CAMERA_NO_MEMORY;
	}
		
	CMR_LOGV("capture addr, y 0x%x uv 0x%x", y_addr, u_addr);
	buffer->channel_id = 1;
	buffer->base_id    = CAMERA_OEM_CAP1_FRM_ID_BASE + cap_index;
	buffer->count      = 1;	
	buffer->length     = frame_size;
	buffer->addr[0].addr_y = y_addr;
	buffer->addr[0].addr_u = u_addr;
	
	return ret;	
}

int camera_capture_max_img_size(uint32_t *max_width, uint32_t *max_height)
{
	int                      ret = CAMERA_SUCCESS;

	if (NULL == max_width || NULL == max_height) {
		return -CAMERA_INVALID_PARM;
	}

	ret = camera_capture_sensor_mode();
	if (ret) {
		CMR_LOGE("Failed to get sensor mode");
	}

	CMR_LOGV("%d %d", g_cxt->max_size.width, g_cxt->max_size.height);
	*max_width  = g_cxt->max_size.width;
	*max_height = g_cxt->max_size.height;

	return 0;
}

int camera_capture_get_buffer_size(uint32_t width,
						uint32_t height, 
						uint32_t *size0, 
						uint32_t *size1)
{
	struct img_size          local_size;
	int                      ret = CAMERA_SUCCESS;

	if (0 == width || 0 == height) {
		return -CAMERA_INVALID_PARM;
	}

	local_size.width  = width;
	local_size.height = height;

	ret = camera_capture_buf_size(&local_size, size0, size1);

	return ret;
}

int camerea_set_preview_format(uint32_t pre_format)
{
	if (IS_PREVIEW)	{
		CMR_LOGE("Invalid camera status, 0x%x", pre_format);
		return -CAMERA_INVALID_STATE;
	}

	g_cxt->preview_fmt = pre_format;

	return CAMERA_SUCCESS;
}
int camera_set_preview_mem(uint32_t phy_addr, uint32_t vir_addr, uint32_t mem_size)
{
	if (0 == phy_addr || 0 == vir_addr || 0 == mem_size)
		return -1;

	CMR_LOGV("phy_addr, 0x%x, vir_addr, 0x%x, mem_size 0x%x", phy_addr, vir_addr, mem_size);
	g_cxt->prev_phys_addr = phy_addr;
	g_cxt->prev_virt_addr = (uint32_t*)vir_addr;
	g_cxt->prev_mem_szie  = mem_size;

	return 0;
}

int camera_set_capture_mem(uint32_t     cap_index,
						uint32_t phy_addr0,
						uint32_t vir_addr0,
						uint32_t mem_size0,
						uint32_t phy_addr1,
						uint32_t vir_addr1,
						uint32_t mem_size1)
{
	struct cmr_cap_2_frm     cap_2_mems;
	struct img_size          max_size;
	struct img_frm           *rot_frm;
	int                      ret = CAMERA_SUCCESS;

	if (cap_index > CAMERA_OEM_CAP_FRM_CNT) {
		CMR_LOGE("Invalid cap_index %d", cap_index);
		return -CAMERA_NO_MEMORY;
	}
	if (0 == phy_addr0 || 0 == vir_addr0 || 0 == mem_size0) {
		CMR_LOGE("Invalid parameter 0x%x 0x%x 0x%x", phy_addr0, vir_addr0, mem_size0);
		return -CAMERA_NO_MEMORY;
	}
	
	cap_2_mems.major_frm.buf_size = mem_size0;
	cap_2_mems.major_frm.addr_phy.addr_y = phy_addr0;
	cap_2_mems.major_frm.addr_vir.addr_y = vir_addr0;

	cap_2_mems.minor_frm.buf_size = mem_size1;
	cap_2_mems.minor_frm.addr_phy.addr_y = phy_addr1;
	cap_2_mems.minor_frm.addr_vir.addr_y = vir_addr1;

	ret = camera_arrange_capture_buf(&cap_2_mems,
					&g_cxt->max_size,
					&g_cxt->thum_size,
					&g_cxt->cap_mem[cap_index]);

	if (0 == ret) {
		if (IMG_ROT_0 != g_cxt->cap_rot) {
			rot_frm = &g_cxt->cap_mem[cap_index].cap_yuv_rot;

			if (0 == rot_frm->addr_phy.addr_y ||
				0 == rot_frm->addr_phy.addr_u ||
				0 == rot_frm->buf_size) {
				CMR_LOGE("No rotation buffer, %d", g_cxt->cap_rot);
				return -CAMERA_NO_MEMORY;
			}
		}
	}

	return ret;
}

int camera_v4l2_preview_handle(struct frm_info *data)
{
	camera_frame_type        frame_type;
	int                      ret = CAMERA_SUCCESS;

	if (IMG_SKIP_SW == g_cxt->skip_mode) {
		if (g_cxt->pre_frm_cnt < g_cxt->skip_num) {
			CMR_LOGV("Ignore this frame, preview cnt %d, total skip num %d",
				g_cxt->pre_frm_cnt, g_cxt->skip_num);
			return ret;
		}
	}
	g_cxt->pre_frm_cnt++;

	camera_preview_done();
	if (IMG_ROT_0 == g_cxt->prev_rot) {
		ret = camera_set_frame_type(&frame_type, data);
		camera_call_cb(CAMERA_EVT_CB_FRAME, 
				camera_get_client_data(), 
				CAMERA_FUNC_START_PREVIEW, 
				(uint32_t)&frame_type);
	} else {
		CMR_LOGV("Need rotate");
		ret = camera_start_rotate(data);
	}

	return ret;
}

int camera_v4l2_capture_handle(struct frm_info *data)
{
	camera_frame_type        frame_type;
	int                      ret = CAMERA_SUCCESS;

	if (NULL == data) {
		CMR_LOGE("Invalid parameter, 0x%x", (uint32_t)data);
		return -CAMERA_INVALID_PARM;
	}

	g_cxt->cap_ch_cnt ++;
	if (g_cxt->cap_ch_cnt == g_cxt->total_cap_ch_num) {
		g_cxt->cap_process_id = g_cxt->cap_cnt;
		g_cxt->cap_cnt ++;
		ret = cmr_v4l2_cap_stop();
		if (ret) {
			CMR_LOGE("Failed to stop v4l2 capture, %d", ret);
			return -CAMERA_FAILED;
		}

		if (g_cxt->cap_cnt == g_cxt->total_cap_num) {
			ret = camera_snapshot_stop_set();
			if (ret) {
				CMR_LOGE("Failed to exit snapshot %d", ret);
				return -CAMERA_FAILED;
			}

		}
	}


	if (0 == data->channel_id) {
		CMR_LOGV("channel 0 capture done, cap_original_fmt %d, cap_zoom_mode %d, rot %d",
			g_cxt->cap_original_fmt,
			g_cxt->cap_zoom_mode,
			g_cxt->cap_rot);
		camera_call_cb(CAMERA_RSP_CB_SUCCESS,
			camera_get_client_data(),
			CAMERA_FUNC_TAKE_PICTURE,
			0);
		if (IMG_DATA_TYPE_RAW == g_cxt->cap_original_fmt) {
			ret = camera_start_isp_process(data);
		} else if (IMG_DATA_TYPE_JPEG == g_cxt->cap_original_fmt) {
			ret = camera_start_jpeg_decode(data);
		} else if (IMG_DATA_TYPE_YUV420 == g_cxt->cap_original_fmt) {
			if (ZOOM_BY_CAP == g_cxt->cap_zoom_mode) {
				if (IMG_ROT_0 == g_cxt->cap_rot) {
					if (g_cxt->cap_orig_size.width == g_cxt->picture_size.width &&
						g_cxt->cap_orig_size.height == g_cxt->picture_size.height) {
						ret = camera_start_jpeg_encode(data);
						if (ret) {
							CMR_LOGE("Failed to start jpeg encode %d", ret);
							return -CAMERA_FAILED;
						}
						ret = camera_take_picture_done(data);
						if (ret) {
							CMR_LOGE("Failed to set take_picture done %d", ret);
							return -CAMERA_FAILED;
						}
					} else {
						ret = camera_start_scale(data);
					}
				} else {
				/*
					camera_save_to_file(0, IMG_DATA_TYPE_YUV420,
						g_cxt->cap_orig_size.width,
						g_cxt->cap_orig_size.height,
							&g_cxt->cap_mem[0].cap_yuv_rot.addr_vir);
				*/
					ret = camera_start_rotate(data);
				}
			} else {
				// it should not be in this branch.
				CMR_LOGE("Something error happened here, original format %d, cap_zoom_mode %d",
				g_cxt->cap_original_fmt,
				g_cxt->cap_zoom_mode);
				//ret = camera_start_scale(data); //just for test
			}
		}
	} else {

		/* It's thumbnail, call jpeg encode */
		CMR_LOGV("channel 1 capture done");
		g_cxt->thum_ready = 1;
#if 0
		camera_save_to_file(0, IMG_DATA_TYPE_YUV420, g_cxt->thum_size.width, g_cxt->thum_size.height,
			&g_cxt->cap_mem[0].thum_yuv.addr_vir);


		if (JPEG_IDLE == g_cxt->jpeg_cxt.jpeg_state) {
			ret = camera_start_jpeg_encode(data);
			if (ret) {
				CMR_LOGE("Failed to start jpeg encode %d", ret);
				return -CAMERA_FAILED;
			}
		} else {
			CMR_LOGV("JPEG is busy, just ");
		}
#endif

	}

	return ret;
}


int camera_start_isp_process(struct frm_info *data)
{
//	struct ips_in_param      ips_in;
	uint32_t                 frm_id;
	int                      ret = CAMERA_SUCCESS;
//	struct ips_out_param			 ips_out;

	if (NULL == data) {
		return -CAMERA_INVALID_PARM;
	}
	frm_id = data->frame_id - CAMERA_OEM_CAP0_FRM_ID_BASE;
/*	
	ips_in.src_frame.img_addr_phy.chn0 = g_cxt->cap_mem[frm_id].cap_raw.addr_phy.addr_y;
	ips_in.src_frame.img_size.w = g_cxt->cap_mem[frm_id].cap_raw.size.width;
	ips_in.src_frame.img_size.h = g_cxt->cap_mem[frm_id].cap_raw.size.height;
	ips_in.dst_frame.img_addr_phy.chn0 = g_cxt->cap_mem[frm_id].cap_yuv.addr_phy.addr_y;
	ips_in.dst_frame.img_size.w = g_cxt->cap_mem[frm_id].cap_yuv.size.width;
	ips_in.dst_frame.img_size.h = g_cxt->cap_mem[frm_id].cap_yuv.size.height;
	ips_in.src_avail_height = g_cxt->picture_size.height;
	ips_in.src_slice_height = CMR_SLICE_HEIGHT;
	ips_in.dst_slice_height = CMR_SLICE_HEIGHT;
	ips_in.callback = camera_proc_start_cb;
	ret = isp_proc_start(&ips_in, &ips_out);*/
	if (0 == ret) {
		CMR_LOGV("ISP post-process started");
		g_cxt->isp_cxt.isp_state = ISP_POST_PROCESS;
		g_cxt->isp_cxt.proc_status.slice_height_in = CMR_SLICE_HEIGHT;
		g_cxt->isp_cxt.proc_status.slice_height_out = 0;
	} else {
		CMR_LOGV("Failed to start ISP, %d", ret);
	}
	return ret;
}

int camera_isp_process_next(struct frm_info *data)
{
	int                      ret = CAMERA_SUCCESS;
	
	return ret;
}

int camera_start_jpeg_decode(struct frm_info *data)
{
	JPEG_DEC_IN_PARAM        dec_in;
	JPEG_DEC_OUT_PARAM       dec_out;
	uint32_t                 frm_id;
	struct img_frm           *frm;
	int                      ret = CAMERA_SUCCESS;

	if (NULL == data || data->frame_id < CAMERA_OEM_CAP0_FRM_ID_BASE) {
		CMR_LOGE("Parameter error, data 0x%x, frame id 0x%x",
			(uint32_t)data, data->frame_id);
		return -CAMERA_INVALID_PARM;
	}

	frm_id = data->frame_id - CAMERA_OEM_CAP0_FRM_ID_BASE;
	if (frm_id > CAMERA_OEM_CAP_FRM_CNT) {
		CMR_LOGE("Wrong Frame id, 0x%x", data->frame_id);
		return -CAMERA_INVALID_PARM;
	}

	frm = &g_cxt->cap_mem[frm_id].target_jpeg;
	dec_in.stream_buf_phy       = frm->addr_phy.addr_y;
	dec_in.stream_buf_vir       = frm->addr_vir.addr_y;
	dec_in.stream_buf_size      = data->length;
	dec_in.size.width           = g_cxt->capture_size.width;
	dec_in.size.height          = g_cxt->capture_size.height;
	dec_in.dst_endian.y_endian  = 1;//TBD
	dec_in.dst_endian.uv_endian = 2;//TBD
	dec_in.slice_height         = CMR_SLICE_HEIGHT;
	if (IMG_ROT_0 == g_cxt->cap_rot) {
		frm = &g_cxt->cap_mem[frm_id].cap_yuv;
	} else {
		frm = &g_cxt->cap_mem[frm_id].cap_yuv_rot;
	}
	dec_in.dst_addr_phy.addr_y  = frm->addr_phy.addr_y;
	dec_in.dst_addr_phy.addr_u  = frm->addr_phy.addr_u;
	dec_in.dst_fmt              = IMG_DATA_TYPE_YUV420;
	dec_in.temp_buf_phy         = g_cxt->cap_mem[frm_id].jpeg_tmp.addr_phy.addr_y;
	dec_in.temp_buf_vir         = g_cxt->cap_mem[frm_id].jpeg_tmp.addr_vir.addr_y;
	dec_in.temp_buf_size        = g_cxt->cap_mem[frm_id].jpeg_tmp.buf_size;
	ret = jpeg_dec_start(&dec_in, &dec_out);
	if (0 == ret) {
		CMR_LOGV("OK, handle 0x%x", dec_out.handle);
		g_cxt->jpeg_cxt.handle = dec_out.handle;
		g_cxt->jpeg_cxt.proc_status.slice_height_out = 0;
		g_cxt->jpeg_cxt.index = g_cxt->cap_cnt;
		g_cxt->jpeg_cxt.jpeg_state = JPEG_DECODE;
	} else {
		CMR_LOGV("Failed, 0x%x", ret);
		g_cxt->jpeg_cxt.jpeg_state = JPEG_ERR;
	}
	return ret;

}

int camera_jpeg_decode_next(struct frm_info *data)
{

	uint32_t                 frm_id;
	int                      ret = CAMERA_SUCCESS;
/*
	if (NULL == data) {
		return -CAMERA_INVALID_PARM;
	}
	ret = jpeg_dec_start(&ips_in, NULL, NULL);

*/	return ret;

}

int camera_jpeg_encode_done(struct frm_info *data)
{
	JPEGENC_CBrtnType        encoder_param;
	camera_encode_mem_type   encoder_type;
	struct img_frm           *jpg_frm;
	int                      ret = CAMERA_SUCCESS;

	(void)data;
	jpg_frm = &g_cxt->cap_mem[g_cxt->jpeg_cxt.index].target_jpeg;

	CMR_LOGV("index %d, bitstream size %d", g_cxt->jpeg_cxt.index, jpg_frm->addr_vir.addr_u);

	bzero(&encoder_param, sizeof(JPEGENC_CBrtnType));
	encoder_param.header_size = 0;
	encoder_param.mode        = JPEGENC_MEM;
	encoder_type.buffer       = (uint8_t *)jpg_frm->addr_vir.addr_y;
	encoder_param.size        = jpg_frm->addr_vir.addr_u;
	encoder_param.outPtr      = &encoder_type;
	encoder_param.status      = JPEGENC_IMG_DONE;

	camera_call_cb(CAMERA_EXIT_CB_DONE,
			camera_get_client_data(),
			CAMERA_FUNC_ENCODE_PICTURE,
			(uint32_t)&encoder_param);

	return ret;
}

int camera_start_jpeg_encode(struct frm_info *data)
{
	uint32_t                 frm_id;
	struct img_frm           *src_frm;
	struct img_frm           *target_frm;
	struct img_frm           *tmp_frm;
	JPEG_ENC_INPUT_PARAM     in_parm;
	JPEG_ENC_OUTPUT_PARAM    out_parm;
	int                      ret = CAMERA_SUCCESS;

	if (NULL == data || JPEG_ENCODE == g_cxt->jpeg_cxt.jpeg_state) {
		CMR_LOGV("wrong parameter 0x%x or status %d",
			(uint32_t)data,
			g_cxt->jpeg_cxt.jpeg_state);
		return -CAMERA_INVALID_PARM;
	}

	CMR_LOGV("channel_id %d, frame_id 0x%x", data->channel_id, data->frame_id);

	if (0 == data->channel_id) {
		frm_id = data->frame_id - CAMERA_OEM_CAP0_FRM_ID_BASE;
		if (frm_id > CAMERA_OEM_CAP_FRM_CNT) {
			CMR_LOGE("Wrong Frame id, 0x%x", data->frame_id);
			return -CAMERA_INVALID_PARM;
		}
		src_frm    = &g_cxt->cap_mem[frm_id].target_yuv;
		target_frm = &g_cxt->cap_mem[frm_id].target_jpeg;
	} else {
		frm_id = data->frame_id - CAMERA_OEM_CAP1_FRM_ID_BASE;
		if (frm_id > CAMERA_OEM_CAP_FRM_CNT) {
			CMR_LOGE("Wrong Frame id, 0x%x", data->frame_id);
			return -CAMERA_INVALID_PARM;
		}
		src_frm    = &g_cxt->cap_mem[frm_id].thum_yuv;
		target_frm = &g_cxt->cap_mem[frm_id].thum_jpeg;
	}
	tmp_frm = &g_cxt->cap_mem[frm_id].jpeg_tmp;

	in_parm.quality_level        = g_cxt->jpeg_cxt.quality;
	in_parm.slice_mod            = JPEG_YUV_SLICE_ONE_BUF;
	in_parm.size.width           = src_frm->size.width;
	in_parm.size.height          = src_frm->size.height;
	in_parm.src_addr_phy.addr_y  = src_frm->addr_phy.addr_y;
	in_parm.src_addr_phy.addr_u  = src_frm->addr_phy.addr_u;
	in_parm.src_addr_vir.addr_y  = src_frm->addr_vir.addr_y;
	in_parm.src_addr_vir.addr_u  = src_frm->addr_vir.addr_u;
	in_parm.slice_height         = in_parm.size.height;
	in_parm.src_endian.y_endian  = data->data_endian.y_endian;
	in_parm.src_endian.uv_endian = data->data_endian.uv_endian;
	in_parm.stream_buf_phy       = target_frm->addr_phy.addr_y;
	in_parm.stream_buf_vir       = target_frm->addr_vir.addr_y;
	in_parm.stream_buf_size      = target_frm->buf_size;
	in_parm.temp_buf_phy         = tmp_frm->addr_phy.addr_y;
	in_parm.temp_buf_vir         = tmp_frm->addr_vir.addr_y;
	in_parm.temp_buf_size        = tmp_frm->buf_size;

	CMR_LOGE("w h, %d %d, quality level %d", in_parm.size.width, in_parm.size.height,
		in_parm.quality_level);
	CMR_LOGE("slice height, %d, slice mode %d", in_parm.slice_height, in_parm.slice_mod);
	CMR_LOGE("phy addr 0x%x 0x%x, vir addr 0x%x 0x%x",
		in_parm.src_addr_phy.addr_y, in_parm.src_addr_phy.addr_u,
		in_parm.src_addr_vir.addr_y, in_parm.src_addr_vir.addr_u);

	CMR_LOGE("endian %d %d", in_parm.src_endian.y_endian, in_parm.src_endian.uv_endian);
	CMR_LOGE("stream phy 0x%x vir 0x%x, size 0x%x",
		in_parm.stream_buf_phy,
		in_parm.stream_buf_vir,
		in_parm.stream_buf_size);
	CMR_LOGE("temp_buf phy 0x%x vir 0x%x, size 0x%x",
		in_parm.temp_buf_phy,
		in_parm.temp_buf_vir,
		in_parm.temp_buf_size);
	ret = jpeg_enc_start(&in_parm, &out_parm);
	if (0 == ret) {
		CMR_LOGV("OK, handle 0x%x", out_parm.handle);
		g_cxt->jpeg_cxt.handle = out_parm.handle;
		g_cxt->jpeg_cxt.proc_status.slice_height_in  = in_parm.size.height;
		g_cxt->jpeg_cxt.proc_status.slice_height_out = 0;
		g_cxt->jpeg_cxt.index = frm_id;
		g_cxt->jpeg_cxt.jpeg_state = JPEG_ENCODE;
	} else {
		CMR_LOGV("Failed, 0x%x", ret);
		g_cxt->jpeg_cxt.jpeg_state = JPEG_ERR;
	}

	return ret;
}

int camera_jpeg_encode_next(struct frm_info *data)
{
	uint32_t                 frm_id;
	int                      ret = CAMERA_SUCCESS;

	return ret;
}

int camera_start_scale(struct frm_info *data)
{
	SENSOR_MODE_INFO_T       *sensor_mode;
	uint32_t                 frm_id;
	struct img_rect          rect;
	struct img_frm           src_frame, dst_frame;
	uint32_t                 slice_h = 0;
	int                      ret = CAMERA_SUCCESS;

	ret = camera_scaler_init();
	if (ret) {
		CMR_LOGE("Failed to init scaler, %d", ret);
		return ret;
	}

	if (0 == data->channel_id) {

		frm_id = data->frame_id - CAMERA_OEM_CAP0_FRM_ID_BASE;

		if (g_cxt->cap_zoom_mode == ZOOM_BY_CAP) {
			rect.start_x = 0;
			rect.start_y = 0;
			rect.width   = g_cxt->cap_orig_size.width;
			rect.height  = g_cxt->cap_orig_size.height;
		} else {
			sensor_mode = &g_cxt->sn_cxt.sensor_info->sensor_mode_info[g_cxt->sn_cxt.capture_mode];
			rect.start_x = sensor_mode->trim_start_x;
			rect.start_y = sensor_mode->trim_start_y;
			rect.width   = sensor_mode->trim_width;
			rect.height  = sensor_mode->trim_height;
			ret = camera_get_trim_rect(&rect, g_cxt->zoom_level, &g_cxt->picture_size);
			if (ret) {
				CMR_LOGE("Failed to calculate scaling window, %d", ret);
				return ret;
			}
		}

		if (rect.width == g_cxt->picture_size.width &&
			rect.height == g_cxt->picture_size.height) {
			CMR_LOGV("No need to to scaling");
			return ret;
		}
		src_frame.size.width      = g_cxt->cap_orig_size.width;
		src_frame.size.height     = g_cxt->cap_orig_size.height;
		src_frame.addr_phy.addr_y = g_cxt->cap_mem[frm_id].cap_yuv.addr_phy.addr_y;
		src_frame.addr_phy.addr_u = g_cxt->cap_mem[frm_id].cap_yuv.addr_phy.addr_u;
		src_frame.addr_phy.addr_v = g_cxt->cap_mem[frm_id].cap_yuv.addr_phy.addr_v;

		dst_frame.size.width      = g_cxt->picture_size.width;
		dst_frame.size.height     = g_cxt->picture_size.height;
		dst_frame.addr_phy.addr_y = g_cxt->cap_mem[frm_id].target_yuv.addr_phy.addr_y;
		dst_frame.addr_phy.addr_u = g_cxt->cap_mem[frm_id].target_yuv.addr_phy.addr_u;
		dst_frame.addr_phy.addr_v = g_cxt->cap_mem[frm_id].target_yuv.addr_phy.addr_v;

		if (dst_frame.size.width <= g_cxt->scaler_cxt.sc_capability) {
			slice_h = dst_frame.size.height;
			dst_frame.fmt = IMG_DATA_TYPE_YUV420;
		} else {
			slice_h = CMR_SLICE_HEIGHT;
			dst_frame.fmt = IMG_DATA_TYPE_YUV422;
		}
		g_cxt->scaler_cxt.proc_status.slice_height_out = 0;
		g_cxt->scaler_cxt.out_fmt = dst_frame.fmt;
	} else {
	
		rect.start_x = 0;
		rect.start_y = 0;
		rect.width   = g_cxt->picture_size.width;
		rect.height  = g_cxt->picture_size.height;

		frm_id = data->frame_id - CAMERA_OEM_CAP0_FRM_ID_BASE;
		src_frame.size.width      = g_cxt->picture_size.width;
		src_frame.size.height     = g_cxt->picture_size.height;
		src_frame.addr_phy.addr_y = g_cxt->cap_mem[frm_id].target_yuv.addr_phy.addr_y;
		src_frame.addr_phy.addr_u = g_cxt->cap_mem[frm_id].target_yuv.addr_phy.addr_u;
		src_frame.addr_phy.addr_v = g_cxt->cap_mem[frm_id].target_yuv.addr_phy.addr_v;

		dst_frame.size.width      = g_cxt->thum_size.width;
		dst_frame.size.height     = g_cxt->thum_size.height;
		dst_frame.addr_phy.addr_y = g_cxt->cap_mem[frm_id].thum_yuv.addr_phy.addr_y;
		dst_frame.addr_phy.addr_u = g_cxt->cap_mem[frm_id].thum_yuv.addr_phy.addr_u;
		dst_frame.addr_phy.addr_v = g_cxt->cap_mem[frm_id].thum_yuv.addr_phy.addr_v;

		slice_h = src_frame.size.height;
		dst_frame.fmt = IMG_DATA_TYPE_YUV420;
	}

	src_frame.fmt = IMG_DATA_TYPE_YUV420;
	src_frame.data_end = data->data_endian;
	dst_frame.data_end = data->data_endian;
	CMR_LOGV("Data endian y, uv %d %d", data->data_endian.y_endian, data->data_endian.uv_endian);

	ret = cmr_scale_start(slice_h, 
			&src_frame,
			&rect,
			&dst_frame,
			NULL,
			NULL);
	if (ret) {
		CMR_LOGE("Failed to start scaler, %d", ret);
		camera_scaler_deinit();
		return ret;
	}

	g_cxt->scaler_cxt.proc_status.frame_info = *data;
	g_cxt->scaler_cxt.proc_status.slice_height_in = slice_h;
	
	g_cxt->scaler_cxt.scale_state = IMG_CVT_SCALING;
	
	return ret;
}

int camera_scale_next(struct frm_info *data)
{
	struct scaler_context    *cxt = &g_cxt->scaler_cxt;
	int                      ret = CAMERA_SUCCESS;

	if (IMG_DATA_TYPE_YUV422 == cxt->out_fmt) {

	}
	ret = cmr_scale_next(0, NULL, NULL, NULL);
	
	return ret;
}

int camera_scale_done(struct frm_info *data)
{
	uint32_t                 frm_id;
	int                      ret = CAMERA_SUCCESS;

	ret = camera_start_jpeg_encode(data);
	if (ret) {
		CMR_LOGE("Failed to start jpeg encode %d", ret);
		return -CAMERA_FAILED;
	}

	if (0 == data->channel_id) {
		frm_id = data->frame_id - CAMERA_OEM_CAP0_FRM_ID_BASE;
		ret = camera_take_picture_done(data);
		if (ret) {
			CMR_LOGE("Failed to set take_picture done %d", ret);
			return -CAMERA_FAILED;
		}
	} else {
		frm_id = data->frame_id - CAMERA_OEM_CAP1_FRM_ID_BASE;
	}

	CMR_LOGV("channel id %d, frame id %d, height %d",
		data->channel_id,
		frm_id,
		g_cxt->capture_size.height);

#if 0
	camera_save_to_file(0, IMG_DATA_TYPE_YUV420, g_cxt->capture_size.width, g_cxt->capture_size.height,
		&g_cxt->cap_mem[frm_id].target_yuv.addr_vir);
#endif

	ret = camera_scaler_deinit();
	if (ret) {
		CMR_LOGV("Failed to deinit scaler, %d", ret);
		return ret;
	}

	return ret;
}

int camera_start_rotate(struct frm_info *data)
{
	uint32_t                 frm_id, rot_frm_id;
	struct img_rect          rect;
	int                      ret = CAMERA_SUCCESS;
	
	if (IS_PREVIEW) {
		CMR_LOGE("Call Rotation in preview");
		if (IMG_CVT_ROTATING == g_cxt->rot_cxt.rot_state) {
			CMR_LOGE("Last rotate not finished yet, drop this frame");
			ret = cmr_v4l2_free_frame(0, data->frame_id);
			return ret;
		} else {
			frm_id = data->frame_id - CAMERA_OEM_PREV_FRM_ID_BASE;
			rot_frm_id  = g_cxt->prev_rot_index % CAMERA_OEM_PREV_ROT_FRM_CNT;
			rect.start_x = 0;
			rect.start_y = 0;
			rect.width  = g_cxt->preview_size.width;
			rect.height = g_cxt->preview_size.height;
			ret = cmr_rot(g_cxt->prev_rot,
				&g_cxt->prev_frm[frm_id],
				&rect,
				&g_cxt->prev_rot_frm[rot_frm_id],
				NULL);
			if (ret) {
				CMR_LOGE("Rot error");
			}
			g_cxt->rot_cxt.proc_status.frame_info = *data;
			g_cxt->rot_cxt.proc_status.slice_height_in = rect.height;
			g_cxt->rot_cxt.rot_state = IMG_CVT_ROTATING;
		}

	} else {
		CMR_LOGE("Call Rotation after capture for channel %d, orig size %d %d",
			data->channel_id,
			g_cxt->cap_orig_size.width,
			g_cxt->cap_orig_size.height);
		frm_id = g_cxt->cap_process_id;
		rect.start_x = 0;
		rect.start_y = 0;
		rect.width  = g_cxt->cap_orig_size.width;
		rect.height = g_cxt->cap_orig_size.height;
		if (g_cxt->cap_orig_size.height == g_cxt->picture_size.width &&
			g_cxt->cap_orig_size.width == g_cxt->picture_size.height) {
			ret = cmr_rot(g_cxt->cap_rot,
				&g_cxt->cap_mem[frm_id].cap_yuv_rot,
				&rect,
				&g_cxt->cap_mem[frm_id].target_yuv,
				NULL);
		} else {
			ret = cmr_rot(g_cxt->cap_rot,
				&g_cxt->cap_mem[frm_id].cap_yuv_rot,
				&rect,
				&g_cxt->cap_mem[frm_id].cap_yuv,
				NULL);
		}
		if (ret) {
			CMR_LOGE("Rot error");
		}
		g_cxt->rot_cxt.proc_status.frame_info = *data;
		g_cxt->rot_cxt.proc_status.slice_height_in = rect.height;
		g_cxt->rot_cxt.rot_state = IMG_CVT_ROTATING;
	}
	return ret;
}

int camera_rotation_copy_data(uint32_t width, uint32_t height, uint32_t in_addr, uint32_t out_addr)
{

	struct img_frm           src_frame, dst_frame;
	uint32_t                 img_len = (uint32_t)(width * height);
	struct img_rect          rect;
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("(w,h)%d %d; addr src, 0x%x dst, 0x%x", 
		width, height, in_addr, out_addr);

	if (!IS_PREVIEW) {
		CMR_LOGE("Preview Stoped");
		return -CAMERA_INVALID_STATE;
	}

#if 0
	src_frame.fmt = ROT_YUV420;
	src_frame.addr_phy.addr_y = in_addr;
	src_frame.addr_phy.addr_u = in_addr + (uint32_t)(width * height);
	src_frame.size.width      = width;
	src_frame.size.height     = height;

	dst_frame.addr_phy.addr_y = out_addr;
	dst_frame.addr_phy.addr_u = out_addr + (uint32_t)(width * height);
	dst_frame.size.width      = width;
	dst_frame.size.height     = height;

	return cmr_rot_cpy(&src_frm, &dst_frm);
#else
	ret = camera_scaler_init();
	if (ret) {
		CMR_LOGE("Failed to init scaler %d", ret);
		ret = -CAMERA_NOT_SUPPORTED;
	} else {
		cmr_scale_evt_reg(NULL);
	}

	src_frame.addr_phy.addr_y = in_addr;
	src_frame.addr_phy.addr_u = in_addr + img_len;
	src_frame.size.width      = width;
	src_frame.size.height     = height;
	src_frame.fmt             = IMG_DATA_TYPE_YUV420;
	src_frame.data_end.y_endian = 1;
	src_frame.data_end.uv_endian = 2;
	dst_frame.addr_phy.addr_y = out_addr;
	dst_frame.addr_phy.addr_u = out_addr + img_len;
	dst_frame.size.width      = width;
	dst_frame.size.height     = height;
	dst_frame.fmt             = IMG_DATA_TYPE_YUV420;
	dst_frame.data_end.y_endian = 1;
	dst_frame.data_end.uv_endian = 2;
	rect.start_x              = 0;
	rect.start_y              = 0;
	rect.width                = width;
	rect.height               = height;
	ret = cmr_scale_start(height,
			&src_frame,
			&rect,
			&dst_frame,
			NULL,
			NULL);

	CMR_LOGV("Done, %d", ret);
	ret = camera_scaler_deinit();
	if (ret) {
		CMR_LOGE("Failed to deinit scaler, %d", ret);
	}
	return ret;
#endif
}


int camera_get_data_redisplay(int output_addr,
				int output_width,
				int output_height,
				int input_addr,
				int input_width,
				int input_height)
{
	return 0;
}

struct camera_context *camera_get_cxt(void)
{
	return g_cxt;
}

int camera_wait_for_preview_done(void)
{
	int                      ret = CAMERA_SUCCESS;

	g_cxt->is_waiting = 1;
	sem_wait(&g_cxt->prev_sem);

	return ret;
}

int camera_preview_done()
{
	int                      ret = CAMERA_SUCCESS;

	if (g_cxt->is_waiting) {
		sem_post(&g_cxt->prev_sem);
		g_cxt->is_waiting = 0;
	}

	return ret;
}

int camera_save_to_file(uint32_t index, uint32_t img_fmt,
	uint32_t width, uint32_t height, struct img_addr *addr)
{
	int                      ret = CAMERA_SUCCESS;
	char                     file_name[40];
	char                     tmp_str[10];
	FILE                     *fp = NULL;

	CMR_LOGV("index %d, format %d, width %d, heght %d",
		index, img_fmt, width, height);

	if (IMG_DATA_TYPE_YUV420 == img_fmt ||
		IMG_DATA_TYPE_YUV422 == img_fmt) {
		bzero(file_name, 40);
		strcpy(file_name, "/data/");
		sprintf(tmp_str, "%d", width);
		strcat(file_name, tmp_str);
		strcat(file_name, "X");
		sprintf(tmp_str, "%d", height);
		strcat(file_name, tmp_str);
		strcat(file_name, "_y_");
		sprintf(tmp_str, "%d", index);
		strcat(file_name, tmp_str);
		strcat(file_name, ".raw");
		CMR_LOGV("file name %s", file_name);
		fp = fopen(file_name, "wb");
		fwrite((void*)addr->addr_y, 1, width * height, fp);
	        fclose(fp);

		bzero(file_name, 40);
		strcpy(file_name, "/data/");
		sprintf(tmp_str, "%d", width);
		strcat(file_name, tmp_str);
		strcat(file_name, "X");
		sprintf(tmp_str, "%d", height);
		strcat(file_name, tmp_str);
		strcat(file_name, "_uv_");
		sprintf(tmp_str, "%d", index);
		strcat(file_name, tmp_str);
		strcat(file_name, ".raw");
		CMR_LOGV("file name %s", file_name);
		fp = fopen(file_name, "wb");
		if (IMG_DATA_TYPE_YUV420 == img_fmt) {
			fwrite((void*)addr->addr_u, 1, width * height / 2, fp);
		} else {
			fwrite((void*)addr->addr_u, 1, width * height, fp);
		}
	        fclose(fp);
	} else if (IMG_DATA_TYPE_JPEG == img_fmt) {
		bzero(file_name, 40);
		strcpy(file_name, "/data/");
		sprintf(tmp_str, "%d", width);
		strcat(file_name, tmp_str);
		strcat(file_name, "X");
		sprintf(tmp_str, "%d", height);
		strcat(file_name, tmp_str);
		strcat(file_name, "_");
		sprintf(tmp_str, "%d", index);
		strcat(file_name, tmp_str);
		strcat(file_name, ".jpg");
		CMR_LOGV("file name %s", file_name);

		fp = fopen(file_name, "wb");
		fwrite((void*)addr->addr_y, 1, addr->addr_u, fp);
	        fclose(fp);
	}

	return 0;
}
