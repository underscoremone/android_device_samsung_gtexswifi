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
#define IS_PREV_FRM(id)      ((id & CAMERA_PREV_ID_BASE) == CAMERA_PREV_ID_BASE)
#define IS_CAP0_FRM(id)      ((id & CAMERA_CAP0_ID_BASE) == CAMERA_CAP0_ID_BASE)
#define IS_CAP1_FRM(id)      ((id & CAMERA_CAP1_ID_BASE) == CAMERA_CAP1_ID_BASE)
#define IS_CAP_FRM(id)       (IS_CAP0_FRM(id) || IS_CAP1_FRM(id))
#define YUV_NO_SCALING       ((ZOOM_BY_CAP == g_cxt->cap_zoom_mode) && \
				(g_cxt->cap_orig_size.width == g_cxt->picture_size.width) && \
				(g_cxt->cap_orig_size.height == g_cxt->picture_size.height))
#define RAW_NO_SCALING       ((ZOOM_POST_PROCESS == g_cxt->cap_zoom_mode) && \
				0 == g_cxt->zoom_level && \
				(g_cxt->cap_orig_size.width == g_cxt->picture_size.width) && \
				(g_cxt->cap_orig_size.height == g_cxt->picture_size.height))
#define NO_SCALING           (YUV_NO_SCALING || RAW_NO_SCALING)
#define IMAGE_FORMAT		 "YVU420_SEMIPLANAR"

static void camera_sensor_evt_cb(int evt, void* data);
static int  camera_isp_evt_cb(int evt, void* data);
static void camera_jpeg_evt_cb(int evt, void* data);
static void camera_v4l2_evt_cb(int evt, void* data);
static void camera_rot_evt_cb(int evt, void* data);
static void camera_scaler_evt_cb(int evt, void* data);
static int  camera_create_main_thread(int32_t camera_id);
static int  camera_destroy_main_thread(void);
static int  camera_get_sensor_preview_mode(struct img_size* target_size, uint32_t *work_mode);
static int  camera_get_sensor_capture_mode(struct img_size* target_size, uint32_t *work_mode);
static int  camera_preview_init(int format_mode);
static void camera_set_client_data(void* user_data);
/*static void *camera_get_client_data(void);*/
static void camera_set_hal_cb(camera_cb_f_type cmr_cb);
/*static void camera_call_cb(camera_cb_type cb,
		const void *client_data,
		camera_func_type func,
		int32_t parm4);*/
static void camera_set_af_cb(camera_cb_f_type cmr_cb);
static void camera_call_af_cb(camera_cb_type cb,
                 const void *client_data,
                 camera_func_type func,
                 int32_t parm4);
static int  camera_capture_init(int format_mode,takepicture_mode cap_mode);
static void *camera_main_routine(void *client_data);
static int  camera_internal_handle(uint32_t evt_type, uint32_t sub_type, struct frm_info *data);
static int  camera_v4l2_handle(uint32_t evt_type, uint32_t sub_type, struct frm_info *data);
static int  camera_isp_handle(uint32_t evt_type, uint32_t sub_type, void *data);
static int  camera_jpeg_codec_handle(uint32_t evt_type, uint32_t sub_type, void *data);
static int  camera_scale_handle(uint32_t evt_type, uint32_t sub_type, struct img_frm *data);
static int  camera_rotation_handle(uint32_t evt_type, uint32_t sub_type, struct img_frm *data);
static int  camera_sensor_handle(uint32_t evt_type, uint32_t sub_type, struct frm_info *data);
static int  camera_img_cvt_handle(uint32_t evt_type, uint32_t sub_type, struct img_frm *data);
static void *camera_af_thread_proc(void *data);
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
static int camera_jpeg_decode_next(struct frm_info *data);
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
static int camera_jpeg_encode_done(uint32_t thumb_stream_size);
static int camera_jpeg_encode_handle(JPEG_ENC_CB_PARAM_T *data);
static int camera_jpeg_decode_handle(JPEG_DEC_CB_PARAM_T *data);
static int camera_set_frame_type(camera_frame_type *frame_type, struct frm_info* info);
static int camera_capture_yuv_process(struct frm_info *data);
static int camera_init_internal(uint32_t camera_id);
static int camera_stop_internal(void);
static int camera_flush_msg_queue(void);
static int camera_preview_err_handle(uint32_t evt_type);
static int camera_capture_err_handle(uint32_t evt_type);
static int camera_jpeg_encode_thumb(uint32_t *stream_size_ptr);
static int camera_convert_to_thumb(void);
static int camera_isp_skip_frame_handle(struct isp_skip_num *skip_number);
static int camera_isp_proc_handle(struct ips_out_param *isp_out);
static int camera_af_init(void);
static int camera_af_deinit(void);
static int camera_uv422_to_uv420(uint32_t dst, uint32_t src, uint32_t width, uint32_t height);

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
	uint32_t                 sensor_num;
	int                      ret = CAMERA_SUCCESS;
	int sensor_ret = SENSOR_FAIL;

	if (ctrl->sensor_inited) {
		CMR_LOGI("sensor intialized before");
		goto exit;
	}

//	camera_set_sensormark();
	sensor_ret = Sensor_Init(camera_id, &sensor_num);
	if (SENSOR_SUCCESS != sensor_ret) {
		CMR_LOGE("No sensor %d", sensor_num);
		ret = -CAMERA_NO_SENSOR;
		goto exit;
	} else {
		if ((uint32_t)camera_id >= sensor_num) {
			CMR_LOGE("No sensor %d", sensor_num);
			ret = -CAMERA_NO_SENSOR;
			goto exit;
		}
		/*
		sensor_ret = Sensor_Open(camera_id);
		if (sensor_ret) {
			CMR_LOGE("Failed to open %d sensor", camera_id);
			ret = -CAMERA_NO_SENSOR;
			goto exit;
		}
		*/
		sensor_cxt->cur_id = camera_id;
		sensor_cxt->sensor_info = Sensor_GetInfo();
		if (NULL == sensor_cxt->sensor_info) {
			CMR_LOGE("Failed to Get sensor info");
			ret = -CAMERA_NOT_SUPPORTED;
			goto sensor_exit;
		}
		if (SENSOR_IMAGE_FORMAT_RAW == sensor_cxt->sensor_info->image_format) {
			sensor_cxt->sn_if.img_fmt = V4L2_SENSOR_FORMAT_RAWRGB;
			CMR_LOGV("It's RawRGB Sensor, %d", sensor_cxt->sn_if.img_fmt);
			ret = Sensor_GetRawSettings(&sensor_cxt->raw_settings, &sensor_cxt->setting_length);
			if (ret) {
				CMR_LOGE("Failed to Get sensor raw settings");
				ret = -CAMERA_NOT_SUPPORTED;
				goto sensor_exit;
			}
		} else {
			sensor_cxt->sn_if.img_fmt = V4L2_SENSOR_FORMAT_YUV;
			CMR_LOGV("It's YUV Sensor, %d", sensor_cxt->sn_if.img_fmt);
		}

		camera_sensor_inf(&sensor_cxt->sn_if, &sensor_cxt->sensor_info->sensor_interface);

		if (0 == sensor_cxt->sn_if.if_type) {
			sensor_cxt->sn_if.if_spec.ccir.v_sync_pol = sensor_cxt->sensor_info->vsync_polarity;
			sensor_cxt->sn_if.if_spec.ccir.h_sync_pol = sensor_cxt->sensor_info->hsync_polarity;
			sensor_cxt->sn_if.if_spec.ccir.pclk_pol   = sensor_cxt->sensor_info->pclk_polarity;
			sensor_cxt->sn_if.frm_deci    = sensor_cxt->sensor_info->preview_deci_num;
			sensor_cxt->sn_if.img_ptn     = sensor_cxt->sensor_info->image_pattern;
		}

		Sensor_EventReg(camera_sensor_evt_cb);
		ctrl->sensor_inited = 1;
		goto exit;
	}

sensor_exit:
	Sensor_Close();
exit:

	//camera_save_sensormark();
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

		ret = cmr_v4l2_scale_capability(&cxt->sc_capability, &cxt->sc_factor);
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
	struct isp_init_param    isp_param;
	int                      ret = CAMERA_SUCCESS;
	struct isp_video_limit   isp_limit;
	SENSOR_EXP_INFO_T		 *sensor_info_ptr;

	if (0 == ctrl->sensor_inited || V4L2_SENSOR_FORMAT_RAWRGB != g_cxt->sn_cxt.sn_if.img_fmt) {
		CMR_LOGI("No need to init ISP %d %d", ctrl->sensor_inited, g_cxt->sn_cxt.sn_if.img_fmt);
		goto exit;
	}

	if (0 == ctrl->isp_inited) {
		sensor_info_ptr = g_cxt->sn_cxt.sensor_info;
		isp_param.setting_param_ptr = sensor_info_ptr;
		if(0 != sensor_info_ptr->sensor_mode_info[SENSOR_MODE_COMMON_INIT].width) {
			isp_param.size.w = sensor_info_ptr->sensor_mode_info[SENSOR_MODE_COMMON_INIT].width;
			isp_param.size.h = sensor_info_ptr->sensor_mode_info[SENSOR_MODE_COMMON_INIT].height;
		} else {
			isp_param.size.w = sensor_info_ptr->sensor_mode_info[SENSOR_MODE_PREVIEW_ONE].width;
			isp_param.size.h = sensor_info_ptr->sensor_mode_info[SENSOR_MODE_PREVIEW_ONE].height;
		}
		isp_param.ctrl_callback = camera_isp_evt_cb;
		ret = isp_init(&isp_param);
		if (ret) {
			CMR_LOGE("Failed to init ISP %d", ret);
			ret = -CAMERA_NOT_SUPPORTED;
			goto exit;
		}
		ret = isp_capbility(ISP_VIDEO_SIZE, &isp_limit);
		if (ret) {
			CMR_LOGE("Failed to get the limitation of ISP %d", ret);
			ret = -CAMERA_NOT_SUPPORTED;
			goto exit;
		}
		cxt->width_limit = isp_limit.width;
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
	ret = isp_deinit();
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

	if (0 == ctrl->jpeg_inited) {
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
		ret = cmr_scale_capability(&cxt->sc_capability, &cxt->sc_factor);
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
	pthread_mutex_init(&g_cxt->prev_mutex, NULL);
	pthread_mutex_init(&g_cxt->af_cb_mutex, NULL);
	pthread_mutex_init(&g_cxt->cancel_mutex, NULL);
	ret = camera_sync_var_init(g_cxt);

	return ret;
}

int camera_local_deinit(void)
{
	int                      ret = CAMERA_SUCCESS;

	g_cxt->camera_status = CMR_IDLE;

	pthread_mutex_destroy (&g_cxt->af_cb_mutex);
	pthread_mutex_destroy (&g_cxt->prev_mutex);
	pthread_mutex_destroy (&g_cxt->data_mutex);
	pthread_mutex_destroy (&g_cxt->cb_mutex);
	pthread_mutex_destroy (&g_cxt->cancel_mutex);
	cmr_msg_queue_destroy(g_cxt->msg_queue_handle);
	g_cxt->msg_queue_handle = 0;

	ret = camera_sync_var_deinit(g_cxt);

	return ret;
}

int camera_init_internal(uint32_t camera_id)
{
	struct camera_ctrl       *ctrl = &g_cxt->control;
	int                      ret = CAMERA_SUCCESS;

	CMR_PRINT_TIME;
	ret = camera_sensor_init(camera_id);
	if (ret) {
		CMR_LOGE("Failed to init sensor %d", ret);
		goto exit;
	}
	CMR_PRINT_TIME;
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
		CMR_LOGE("Fail to init Rot device %d", ret);
		goto jpeg_deinit;
	}

	ret = camera_setting_init();
	if (ret) {
		CMR_LOGE("Fail to init Setting sub-module");
	} else {
		goto exit;
	}

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
camera_ret_code_type camera_init(int32_t camera_id)
{
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("%d", camera_id);

	bzero(g_cxt, sizeof(*g_cxt));
	ret = camera_local_init();
	if (ret) {
		CMR_LOGE("Failed to init local variables %d", ret);
		return ret;
	}

	ret = camera_create_main_thread(camera_id);
	if (ret) {
		CMR_LOGE("Fail to send message to camera main thread");
		return ret;
	}

	return ret;
}

int camera_stop_internal(void)
{
	camera_af_deinit();

	arithmetic_fd_deinit();

	camera_setting_deinit();

	camera_rotation_deinit();

	camera_jpeg_deinit();

	camera_isp_deinit();

	camera_v4l2_deinit();

	camera_sensor_deinit();

	camera_call_cb(CAMERA_EXIT_CB_DONE,
			camera_get_client_data(),
			CAMERA_FUNC_STOP,
			0);

	return CAMERA_SUCCESS;
}

camera_ret_code_type camera_stop( camera_cb_f_type callback, void *client_data)
{
	CMR_MSG_INIT(message);
	int                      ret = CAMERA_SUCCESS;

	camera_set_client_data(client_data);
	camera_set_hal_cb(callback);

	camera_destroy_main_thread();

	camera_local_deinit();

	return ret;
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
	index += CAMERA_PREV_ID_BASE;
	if (IMG_ROT_0 == g_cxt->prev_rot) {
		if (index >= CAMERA_PREV_ID_BASE &&
			index < CAMERA_PREV_ID_BASE + CAMERA_PREV_FRM_CNT) {
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
	SENSOR_EXP_INFO_T        *sn_info = NULL;
	SENSOR_MODE_INFO_T       *sn_mode = NULL;
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
	sn_info = g_cxt->sn_cxt.sensor_info;
	if (NULL == sn_info) {
		CMR_LOGE("NO Sensor.");
		return -CAMERA_NOT_SUPPORTED;
	}
	sn_mode = &sn_info->sensor_mode_info[g_cxt->sn_cxt.preview_mode];
	if (SENSOR_IMAGE_FORMAT_YUV422 != sn_mode->image_format &&
		SENSOR_IMAGE_FORMAT_RAW != sn_mode->image_format) {
		CMR_LOGE("Wrong sensor formast %d", sn_mode->image_format);
		ret = -CAMERA_NOT_SUPPORTED;
	} else {
		if (SENSOR_IMAGE_FORMAT_YUV422 == sn_mode->image_format) {
			g_cxt->sn_cxt.sn_if.img_fmt = V4L2_SENSOR_FORMAT_YUV;
		} else {
			g_cxt->sn_cxt.sn_if.img_fmt = V4L2_SENSOR_FORMAT_RAWRGB;
		}
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

	if (SENSOR_IMAGE_FORMAT_YUV422 == sensor_mode->image_format) {
		g_cxt->sn_cxt.sn_if.img_fmt = V4L2_SENSOR_FORMAT_YUV;
	} else if (SENSOR_IMAGE_FORMAT_RAW == sensor_mode->image_format) {
		g_cxt->sn_cxt.sn_if.img_fmt = V4L2_SENSOR_FORMAT_RAWRGB;
	} else if (SENSOR_IMAGE_FORMAT_JPEG == sensor_mode->image_format) {
		g_cxt->sn_cxt.sn_if.img_fmt = V4L2_SENSOR_FORMAT_JPEG;
	} else {
		CMR_LOGE("Wrong sensor formast %d", sensor_mode->image_format);
		ret = -CAMERA_NOT_SUPPORTED;
	}

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

	CMR_LOGV("picture %d %d, display %d %d,rot %d",
		picture_width,
		picture_height,
		display_width,
		display_height,
		g_cxt->prev_rot);

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

	ret = camera_set_pos_type(position);

	return ret;
}

camera_ret_code_type camera_set_thumbnail_properties(uint32_t width,
						uint32_t height,
						uint32_t quality)
{
	(void) quality;

	CMR_LOGV("%d,%d.",g_cxt->thum_size.width,g_cxt->thum_size.height);
/*	if (0 == width || 0 == height)
		return CAMERA_INVALID_PARM;
*/
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

int camera_before_set_internal(enum restart_mode re_mode)
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
		if (RESTART_HEAVY == re_mode) {
			Sensor_Close();
		}
		break;
	case RESTART_LIGHTLY:
		ret = cmr_v4l2_cap_pause();
		break;
	default:
		break;
	}
	return ret;
}

int camera_before_set(enum restart_mode re_mode)
{
	CMR_MSG_INIT(message);
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("before_set");

	message.msg_type = CMR_EVT_BEFORE_SET;
	message.sub_msg_type = re_mode;
	ret = cmr_msg_post(g_cxt->msg_queue_handle, &message);

	if (ret) {
		CMR_LOGE("Fail to send message to camera main thread");
		return ret;
	}

	ret = camera_wait_stop(g_cxt);

	return ret;
}

int camera_after_set_internal(enum restart_mode re_mode)
{
	int                      ret = CAMERA_SUCCESS;
	uint32_t                 skip_number_l = 0, sensor_num = 0;

	CMR_LOGV("after set %d, skip mode %d, skip number %d",
		re_mode,
		g_cxt->skip_mode,
		g_cxt->skip_num);

	if (IMG_SKIP_HW == g_cxt->skip_mode) {
		skip_number_l = g_cxt->skip_num;
	}

	switch (re_mode) {
	case RESTART_HEAVY:
		ret = Sensor_Init(g_cxt->sn_cxt.cur_id, &sensor_num);
		if (ret) {
			CMR_LOGE("Failed to init sensor");
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
		ret = cmr_v4l2_cap_start(skip_number_l);
		if (ret) {
			CMR_LOGE("Failed to restart preview");
			return -CAMERA_FAILED;
		} else {
			ret = Sensor_StreamOn();
			if (ret) {
				CMR_LOGE("Fail to switch on the sensor stream");
				return -CAMERA_FAILED;
			}
			g_cxt->v4l2_cxt.v4l2_state = V4L2_PREVIEW;
			g_cxt->camera_status = CMR_PREVIEW;
		}
		break;
	case RESTART_LIGHTLY:
		ret = cmr_v4l2_cap_resume(skip_number_l);
		break;
	default:
		CMR_LOGE("Wrong re-start mode");
		ret = -CAMERA_INVALID_PARM;
		break;
	}

	CMR_LOGV("Exit");

	return ret;
}

int camera_after_set(enum restart_mode re_mode,
			enum img_skip_mode skip_mode,
			uint32_t skip_number)
{
	CMR_MSG_INIT(message);
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("after_set");
	if (skip_mode > IMG_SKIP_SW) {
		CMR_LOGE("Wrong skip mode");
		return -CAMERA_FAILED;
	}

	g_cxt->skip_mode = skip_mode;
	g_cxt->skip_num  = skip_number;

	message.msg_type = CMR_EVT_AFTER_SET;
	message.sub_msg_type = re_mode;
	ret = cmr_msg_post(g_cxt->msg_queue_handle, &message);

	if (ret) {
		CMR_LOGE("Fail to send message to camera main thread");
		return ret;
	}

	ret = camera_wait_start(g_cxt);
	return ret;
}

int camera_start_preview_internal(void)
{
	uint32_t                 skip_number = 0;
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("preview format is %d", g_cxt->preview_fmt);

	ret = cmr_v4l2_if_cfg(&g_cxt->sn_cxt.sn_if);
	if (ret) {
		CMR_LOGE("the sensor interface is unsupported by V4L2");
		return -CAMERA_FAILED;
	}
	CMR_PRINT_TIME;
	ret = camera_preview_start_set();
	if (ret) {
		CMR_LOGE("Failed to set sensor preview mode.");
		return -CAMERA_FAILED;
	}
	CMR_PRINT_TIME;
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
	ret = Sensor_StreamOn();
	if (ret) {
		CMR_LOGE("Fail to switch on the sensor stream");
		return -CAMERA_FAILED;
	}
	g_cxt->v4l2_cxt.v4l2_state = V4L2_PREVIEW;
	g_cxt->camera_status = CMR_PREVIEW;

	ret = camera_af_init();
	if (ret) {
		CMR_LOGE("Fail to initialize AF");
	}
	ret = arithmetic_fd_init();
	if (ret) {
		CMR_LOGE("Fail to init arithmetic %d", ret);
	}

	return 0;
}

camera_ret_code_type camera_start_preview(camera_cb_f_type callback,
					void *client_data)
{
	CMR_MSG_INIT(message);
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("start preview");
	CMR_PRINT_TIME;
	camera_set_client_data(client_data);
	camera_set_hal_cb(callback);

	g_cxt->err_code = 0;
	g_cxt->recover_status = NO_RECOVERY;
	message.msg_type = CMR_EVT_START;
	message.sub_msg_type = CMR_PREVIEW;
	ret = cmr_msg_post(g_cxt->msg_queue_handle, &message);

	if (ret) {
		CMR_LOGE("Fail to send message to camera main thread");
		return ret;
	}

	ret = camera_wait_start(g_cxt);
	if (CAMERA_SUCCESS == ret) {
		ret = g_cxt->err_code;
	}
	CMR_LOGV("start preview finished... %d", ret);
	return ret;
}

int camera_stop_preview_internal(void)
{
	int                      ret = CAMERA_SUCCESS;

	if (!IS_PREVIEW) {
		CMR_LOGE("Not in preview, %d", g_cxt->camera_status);
		return ret;
	}

	pthread_mutex_lock(&g_cxt->prev_mutex);
	CMR_PRINT_TIME;

	g_cxt->camera_status = CMR_IDLE;
	camera_scaler_deinit();
	CMR_PRINT_TIME;

	g_cxt->pre_frm_cnt = 0;
	if (V4L2_PREVIEW == g_cxt->v4l2_cxt.v4l2_state) {
		ret = cmr_v4l2_cap_stop();
		g_cxt->v4l2_cxt.v4l2_state = V4L2_IDLE;
		if (ret) {
			CMR_LOGE("Failed to stop V4L2 capture, %d", ret);
		}
	}
	CMR_PRINT_TIME;

	ret = Sensor_StreamOff();
	if (ret) {
		CMR_LOGE("Failed to switch off the sensor stream, %d", ret);
	}

	pthread_mutex_unlock(&g_cxt->prev_mutex);
	CMR_PRINT_TIME;

	if (ISP_COWORK == g_cxt->isp_cxt.isp_state) {
		ret = isp_video_stop();
		g_cxt->isp_cxt.isp_state = ISP_IDLE;
		if (ret) {
			CMR_LOGE("Failed to stop ISP video mode, %d", ret);
		}
	}

	cmr_rot_wait_done();
	return ret;
}

int camera_flush_msg_queue(void)
{
	CMR_MSG_INIT(message);
	int                      ret = CAMERA_SUCCESS;
	int                      cnt = 0;

	while (CMR_MSG_SUCCESS == ret) {
		ret = cmr_msg_peak(g_cxt->msg_queue_handle, &message);
		if (ret) {
			CMR_LOGV("no more msg");
			break;
		}

		if (message.alloc_flag) {
			free(message.data);
		}
		cnt ++;
	}

	CMR_LOGV("release count %d", cnt);
	return ret;
}

camera_ret_code_type camera_stop_preview(void)
{
	CMR_MSG_INIT(message);
	int                      ret = CAMERA_SUCCESS;

	pthread_mutex_lock(&g_cxt->cb_mutex);
	pthread_mutex_unlock(&g_cxt->cb_mutex);
	CMR_PRINT_TIME;
/*	camera_flush_msg_queue();*/
	message.msg_type = CMR_EVT_STOP;
	message.sub_msg_type = CMR_PREVIEW;
	ret = cmr_msg_post(g_cxt->msg_queue_handle, &message);

	if (ret) {
		CMR_LOGE("Fail to send message to camera main thread");
		return ret;
	}

	camera_wait_stop(g_cxt);
	CMR_PRINT_TIME;
	CMR_LOGV("stop preview... %d", ret);

	return ret;
}

int camera_take_picture_done(struct frm_info *data)
{
	camera_frame_type        frame_type;
	int                      ret = CAMERA_SUCCESS;

	if (camera_capture_need_exit()) {
		ret = CAMERA_INVALID_STATE;
		CMR_LOGE("exit.");
		return ret;
	}
	ret = camera_set_frame_type(&frame_type, data);
	if (CAMERA_SUCCESS == ret) {
		camera_call_cb(CAMERA_EVT_CB_SNAPSHOT_DONE,
				camera_get_client_data(),
				CAMERA_FUNC_TAKE_PICTURE,
				(uint32_t)&frame_type);
		CMR_LOGE("CAMERA_EVT_CB_SNAPSHOT_DONE.");

		camera_call_cb(CAMERA_EXIT_CB_DONE,
				camera_get_client_data(),
				CAMERA_FUNC_TAKE_PICTURE,
				(uint32_t)&frame_type);
		CMR_LOGE("CAMERA_EXIT_CB_DONE.");

	} else {
		camera_call_cb(CAMERA_EXIT_CB_FAILED,
			camera_get_client_data(),
			CAMERA_FUNC_TAKE_PICTURE,
			0);
	}

	return ret;
}

int camera_take_picture_internal(takepicture_mode cap_mode)
{
	int                      preview_format;
	uint32_t                 skip_number = 0;
	int                      ret = CAMERA_SUCCESS;

	g_cxt->cap_mode = cap_mode;
	ret = camera_capture_init(g_cxt->preview_fmt,cap_mode);
	if (ret) {
		CMR_LOGE("Failed to init capture mode.");
		return -CAMERA_FAILED;
	}

	CMR_LOGV("OK  to camera_capture_init,mode %d.\n",cap_mode);

	if (IMG_SKIP_HW == g_cxt->skip_mode) {
		skip_number = g_cxt->sn_cxt.sensor_info->capture_skip_num;
	}
	CMR_PRINT_TIME;
	ret = cmr_v4l2_cap_start(skip_number);
	if (ret) {
		CMR_LOGE("Fail to start V4L2 Capture");
		return -CAMERA_FAILED;
	}
	g_cxt->v4l2_cxt.v4l2_state = V4L2_CAPTURE;
	CMR_PRINT_TIME;
	ret = Sensor_StreamOn();
	if (ret) {
		CMR_LOGE("Fail to switch on the sensor stream");
		return -CAMERA_FAILED;
	}
	g_cxt->camera_status = CMR_CAPTURE;
	CMR_PRINT_TIME;
	return ret;
}

camera_ret_code_type camera_take_picture(camera_cb_f_type    callback,
					void                 *client_data,takepicture_mode cap_mode)
{
	CMR_MSG_INIT(message);
	int                      ret = CAMERA_SUCCESS;

	camera_set_client_data(client_data);
	camera_set_hal_cb(callback);

	g_cxt->err_code = 0;
	message.msg_type = CMR_EVT_START;
	message.sub_msg_type = CMR_CAPTURE;
	message.data = (void*)cap_mode;
	ret = cmr_msg_post(g_cxt->msg_queue_handle, &message);

	if (ret) {
		CMR_LOGE("Fail to send message to camera main thread");
		return ret;
	}

	ret = camera_wait_start(g_cxt);
	if (CAMERA_SUCCESS == ret) {
		ret = g_cxt->err_code;
	}
	return ret;
}

int camera_take_picture_hdr(void)
{
	int                      preview_format;
	uint32_t                 skip_number = 0;
	int                      ret = CAMERA_SUCCESS;
	struct buffer_cfg        buffer_info;

	CMR_LOGI("start.");
	g_cxt->cap_ch_cnt = 0;
	ret = camera_alloc_capture_buf0(&buffer_info, 0);//g_cxt->cap_cnt);
	if (ret) {
		CMR_LOGE("Failed to alloc capture buffer");
		return -CAMERA_NO_MEMORY;
	}

	ret = cmr_v4l2_buff_cfg(&buffer_info);
	if (ret) {
		CMR_LOGE("Failed to Q capture buffer");
		return -CAMERA_NO_MEMORY;
	}

	ret = cmr_v4l2_cap_start(0);
	if (ret) {
		CMR_LOGE("Fail to start V4L2 Capture");
		return -CAMERA_FAILED;
	}
	g_cxt->v4l2_cxt.v4l2_state = V4L2_CAPTURE;
	if ((HDR_CAP_NUM-2) == g_cxt->cap_cnt) {
		camera_set_hdr_ev(SENSOR_HDR_EV_LEVE_1);
	} else {
		camera_set_hdr_ev(SENSOR_HDR_EV_LEVE_2);
	}
	CMR_PRINT_TIME;
	ret = Sensor_StreamOn();
	if (ret) {
		CMR_LOGE("Fail to switch on the sensor stream");
		return -CAMERA_FAILED;
	}
	g_cxt->camera_status = CMR_CAPTURE;
	CMR_PRINT_TIME;
	return ret;
}
int camera_stop_capture_internal(void)
{
	int                      ret = CAMERA_SUCCESS;

	camera_scaler_deinit();

	ret = cmr_v4l2_cap_stop();
	if (ret) {
		CMR_LOGE("Failed to stop v4l2 capture, %d", ret);
		return -CAMERA_FAILED;
	}
	CMR_PRINT_TIME;
	ret = Sensor_StreamOff();
	if (ret) {
		CMR_LOGE("Failed to switch off the sensor stream, %d", ret);
	}
	if((JPEG_ENCODE == g_cxt->jpeg_cxt.jpeg_state)
		|| JPEG_DECODE == g_cxt->jpeg_cxt.jpeg_state) {
		jpeg_stop(g_cxt->jpeg_cxt.handle);
	}
	arithmetic_hdr_deinit();
	return ret;
}

camera_ret_code_type camera_stop_capture(void)
{
	CMR_MSG_INIT(message);
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("stop taking photo");
	camera_cancel_capture();
	message.msg_type = CMR_EVT_STOP;
	message.sub_msg_type = CMR_CAPTURE;
	ret = cmr_msg_post(g_cxt->msg_queue_handle, &message);

	if (ret) {
		CMR_LOGE("Fail to send message to camera main thread");
		return ret;
	}

	camera_wait_stop(g_cxt);

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

	CMR_LOGV("focus %d, client_data 0x%x", focus, (uint32_t)client_data);
	CMR_PRINT_TIME;
	camera_set_af_cb(callback);

	camera_autofocus();

	message.msg_type = CMR_EVT_AF_START;
	message.data     = client_data;
	ret = cmr_msg_post(g_cxt->af_msg_que_handle, &message);
	if (ret) {
		CMR_LOGE("Faile to send one msg to camera main thread");
	}
	CMR_PRINT_TIME;

	return ret;
}

int camera_af_init(void)
{
	CMR_MSG_INIT(message);
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("inited, %d", g_cxt->af_inited);

	CMR_PRINT_TIME;
	if (!g_cxt->af_inited) {
		ret = cmr_msg_queue_create(CAMERA_AF_MSG_QUEUE_SIZE, &g_cxt->af_msg_que_handle);
		if (ret) {
			CMR_LOGE("NO Memory, Frailed to create message queue");
		}
		sem_init(&g_cxt->af_sync_sem, 0, 0);
		ret = pthread_create(&g_cxt->af_thread, NULL, camera_af_thread_proc, NULL);
		sem_wait(&g_cxt->af_sync_sem);
		g_cxt->af_inited = 1;
		message.msg_type = CMR_EVT_AF_INIT;
		message.data     = 0;
		ret = cmr_msg_post(g_cxt->af_msg_que_handle, &message);
		if (ret) {
			CMR_LOGE("Faile to send one msg to camera af thread");
		}

	}
	return ret;
}

int camera_af_deinit(void)
{
	CMR_MSG_INIT(message);
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("inited, %d", g_cxt->af_inited);

	if (g_cxt->af_inited) {
		message.msg_type = CMR_EVT_AF_EXIT;
		ret = cmr_msg_post(g_cxt->af_msg_que_handle, &message);
		if (ret) {
			free(message.data);
			CMR_LOGE("Faile to send one msg to camera main thread");
		}
		sem_wait(&g_cxt->af_sync_sem);
		sem_destroy(&g_cxt->af_sync_sem);
		cmr_msg_queue_destroy(g_cxt->af_msg_que_handle);
		g_cxt->af_msg_que_handle = 0;
		g_cxt->af_inited = 0;
	}
	return ret ;
}

int camera_set_frame_type(camera_frame_type *frame_type, struct frm_info* info)
{
	uint32_t                 frm_id;
	int                      ret = CAMERA_SUCCESS;

	if (NULL == frame_type || NULL == info) {
		CMR_LOGE("Wrong param, frame_type 0x%x, info 0x%x",
			(uint32_t)frame_type,
			(uint32_t)info);
		return -CAMERA_INVALID_PARM;
	}

	if (IS_PREVIEW) {
		if (g_cxt->prev_rot) {
			frm_id = g_cxt->prev_rot_index % CAMERA_PREV_ROT_FRM_CNT;
			frame_type->buf_id = frm_id; // more than CAMERA_PREV_FRM_CNT
			frame_type->buf_Virt_Addr = (uint32_t*)g_cxt->prev_rot_frm[frm_id].addr_vir.addr_y;
			frame_type->buffer_phy_addr = g_cxt->prev_rot_frm[frm_id].addr_phy.addr_y;
		} else {
			frm_id = info->frame_id - CAMERA_PREV_ID_BASE;
			frame_type->buf_id = frm_id;
			frame_type->buf_Virt_Addr = (uint32_t*)g_cxt->prev_frm[frm_id].addr_vir.addr_y;
			frame_type->buffer_phy_addr = g_cxt->prev_frm[frm_id].addr_phy.addr_y;

		}
		frame_type->dx = g_cxt->display_size.width;
		frame_type->dy = g_cxt->display_size.height;
#if 0
		if (1 == g_cxt->pre_frm_cnt) {
			camera_save_to_file(11,
					IMG_DATA_TYPE_YUV420,
					frame_type->dx,
					frame_type->dy,
					&g_cxt->prev_frm[frm_id].addr_vir);
		}
#endif
		if ((1 == g_cxt->arithmetic_cxt.fd_flag) && (1 == g_cxt->arithmetic_cxt.fd_inited)) {
			CMR_LOGI("face detect start.");
			arithmetic_fd_start((void*)frame_type->buf_Virt_Addr);
		}
	} else if (IS_CAPTURE) {
		frm_id = info->frame_id - CAMERA_CAP0_ID_BASE;
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
		frame_type->buffer_uv_phy_addr = g_cxt->cap_mem[frm_id].target_yuv.addr_phy.addr_u;
		CMR_LOGI("cap yuv addr 0x%x.",(uint32_t)frame_type->buf_Virt_Addr);
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

int camera_create_main_thread(int32_t camera_id)
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
	message.msg_type = CMR_EVT_INIT;
	message.sub_msg_type = camera_id;
	CMR_PRINT_TIME;
	ret = cmr_msg_post(g_cxt->msg_queue_handle, &message);
	if (ret) {
		CMR_LOGE("Fail to send message to camera main thread");
		return ret;
	}

	ret = camera_wait_init(g_cxt);
	CMR_PRINT_TIME;
	return ret;
}

int camera_destroy_main_thread(void)
{
	CMR_MSG_INIT(message);
	int                      ret = CAMERA_SUCCESS;

	message.msg_type = CMR_EVT_EXIT;
	ret = cmr_msg_post(g_cxt->msg_queue_handle, &message);

	ret = camera_wait_exit(g_cxt);
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

		CMR_LOGE("message.msg_type 0x%x, sub-type 0x%x",
			message.msg_type,
			message.sub_msg_type);
		evt = (uint32_t)(message.msg_type & CMR_EVT_MASK_BITS);

		switch (evt) {
		case CMR_EVT_OEM_BASE:
			ret = camera_internal_handle(message.msg_type,
					message.sub_msg_type,
					(struct frm_info*)message.data);
			break;

		case CMR_EVT_V4L2_BASE:
			ret = camera_v4l2_handle(message.msg_type,
					message.sub_msg_type,
					(struct frm_info *)message.data);
			break;

		case CMR_EVT_ISP_BASE:
			ret = camera_isp_handle(message.msg_type,
					message.sub_msg_type,
					(void *)message.data);
			break;

		case CMR_EVT_SENSOR_BASE:
			ret = camera_sensor_handle(message.msg_type,
					message.sub_msg_type,
					(struct frm_info *)message.data);
			break;

		case CMR_EVT_CVT_BASE:
			ret = camera_img_cvt_handle(message.msg_type,
					message.sub_msg_type,
					(struct img_frm *)message.data);
			break;

		case CMR_EVT_JPEG_BASE:
			ret = camera_jpeg_codec_handle(message.msg_type,
					message.sub_msg_type,
					(void *)message.data);
			break;

		default:
			CMR_LOGE("Unsupported MSG");
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

	if (CMR_EVT_SENSOR_BASE != (CMR_EVT_SENSOR_BASE & evt)) {
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
	struct frm_info          *info = (struct frm_info*)data;

	if (NULL == data || info->frame_id < CAMERA_PREV_ID_BASE ||
		CMR_EVT_V4L2_BASE != (CMR_EVT_V4L2_BASE & evt)) {
		CMR_LOGE("Error param, 0x%x 0x%x 0x%x", (uint32_t)data, evt, info->frame_id);
		return;
	}

	if (V4L2_IDLE == g_cxt->v4l2_cxt.v4l2_state) {
		ret = cmr_v4l2_free_frame(info->channel_id, info->frame_id);
		CMR_LOGE("Wrong status, %d", g_cxt->v4l2_cxt.v4l2_state);
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

int32_t camera_isp_evt_cb(int32_t evt, void* data)
{
	CMR_MSG_INIT(message);
	uint32_t                 cmd;
	int                      ret = CAMERA_SUCCESS;

	if (CMR_EVT_ISP_BASE != (CMR_EVT_ISP_BASE & evt)) {
		CMR_LOGE("Error param, 0x%x", evt);
		return -1;
	}

	CMR_LOGV("evt, 0x%x", evt);

	message.sub_msg_type = (~CMR_EVT_ISP_BASE) & evt;
	CMR_LOGV("message.sub_msg_type, 0x%x", message.sub_msg_type);
	cmd = evt & 0xFF;
	if ((message.sub_msg_type & ISP_EVT_MASK) == 0) {
		ret = camera_isp_ctrl_done(cmd, data);
		CMR_LOGV("ret, %d", ret);
		return 0;
	}

	if (data) {
		message.data = malloc(sizeof(struct frm_info));
		if (NULL == message.data) {
			CMR_LOGE("NO mem, Faile to alloc memory for one msg");
			return -1;
		}
		message.alloc_flag = 1;
		memcpy(message.data, data, sizeof(struct frm_info));
	}
	message.msg_type = evt;
	ret = cmr_msg_post(g_cxt->msg_queue_handle, &message);

	if (ret) {
		if (message.data) {
			free(message.data);
		}
		CMR_LOGE("Faile to send one msg to camera main thread");
	}

	return 0;
}

void camera_jpeg_evt_cb(int evt, void* data)
{
	CMR_MSG_INIT(message);
	int                      ret = CAMERA_SUCCESS;

	if (NULL == data || CMR_EVT_JPEG_BASE != (CMR_EVT_JPEG_BASE & evt)) {
		CMR_LOGE("Error param, 0x%x 0x%x", (uint32_t)data, evt);
		return;
	}
	if(CMR_JPEG_DEC_DONE == evt) {
		message.data = malloc(sizeof(JPEG_DEC_CB_PARAM_T));
	} else {
		message.data = malloc(sizeof(JPEG_ENC_CB_PARAM_T));
	}

	if (NULL == message.data) {
		CMR_LOGE("NO mem, Faile to alloc memory for one msg");
		return;
	}
	message.msg_type = evt;
	message.alloc_flag = 1;
	if(CMR_JPEG_DEC_DONE == evt) {
		memcpy(message.data, data, sizeof(JPEG_DEC_CB_PARAM_T));
	} else {
		memcpy(message.data, data, sizeof(JPEG_ENC_CB_PARAM_T));
	}
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

int camera_internal_handle(uint32_t evt_type, uint32_t sub_type, struct frm_info *data)
{
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("evt_type 0x%x, sub_type %d", evt_type, sub_type);

	switch (evt_type) {
	case CMR_EVT_INIT:
		ret = camera_init_internal(sub_type);
		g_cxt->err_code = ret;
		camera_init_done(g_cxt);
		g_cxt->is_working = 1;
		break;

	case CMR_EVT_EXIT:
		if (CMR_PREVIEW == g_cxt->camera_status) {
			CMR_LOGV("OEM is still in Preview, stop it");
			camera_stop_preview_internal();
		} else if (CMR_CAPTURE == g_cxt->camera_status) {
			CMR_LOGV("OEM is still in TakePicture, stop it");
			camera_stop_capture_internal();
		}
		ret = camera_stop_internal();
		g_cxt->err_code = ret;
		camera_exit_done(g_cxt);
		g_cxt->is_working = 0;
		break;

	case CMR_EVT_START:
		if (CMR_PREVIEW == sub_type) {
			ret = camera_start_preview_internal();
			if (CAMERA_SUCCESS == ret) {
				camera_call_cb(CAMERA_RSP_CB_SUCCESS,
						camera_get_client_data(),
						CAMERA_FUNC_START_PREVIEW,
						0);
			}
		} else if (CMR_CAPTURE == sub_type) {
			ret = camera_take_picture_internal((uint32_t)data);
		} else {
			CMR_LOGV("No this sub-type");
		}
		g_cxt->err_code = ret;
		camera_start_done(g_cxt);

		break;

	case CMR_EVT_STOP:
		if (CMR_PREVIEW == sub_type) {
			ret = camera_stop_preview_internal();
			camera_preview_stop_set();
			camera_call_cb(CAMERA_RSP_CB_SUCCESS,
					camera_get_client_data(),
					CAMERA_FUNC_STOP_PREVIEW,
					0);
		} else if (CMR_CAPTURE == sub_type) {
			ret = camera_stop_capture_internal();
			camera_call_cb(CAMERA_RSP_CB_SUCCESS,
				    camera_get_client_data(),
				    CAMERA_FUNC_RELEASE_PICTURE,
				    0);
		} else {
			CMR_LOGV("No this sub-type");
		}
		g_cxt->err_code = ret;
		camera_stop_done(g_cxt);
		break;

	case CMR_EVT_BEFORE_SET:
		ret = camera_before_set_internal(sub_type);
		g_cxt->err_code = ret;

		camera_stop_done(g_cxt);

		break;

	case CMR_EVT_AFTER_SET:
		ret = camera_after_set_internal(sub_type);
		g_cxt->err_code = ret;
		camera_start_done(g_cxt);

		break;

	default:
		break;

	}

	return ret;
}

int camera_v4l2_handle(uint32_t evt_type, uint32_t sub_type, struct frm_info *data)
{
	int                      ret = CAMERA_SUCCESS;

	if (NULL == data) {
		CMR_LOGE("Parameter error");
		return -CAMERA_INVALID_PARM;
	}

	if (CMR_IDLE == g_cxt->camera_status || CMR_ERR == g_cxt->camera_status) {
		CMR_LOGE("Status error, %d", g_cxt->camera_status);
		return -CAMERA_INVALID_STATE;
	}

	(void)sub_type;
	switch (evt_type) {
	case CMR_V4L2_TX_DONE:
		if (IS_PREVIEW) {
			if (!IS_PREV_FRM(data->frame_id)) {
				CMR_LOGE("Wrong frame id %d, drop this frame", data->frame_id);
				return CAMERA_SUCCESS;
			}
			ret = camera_v4l2_preview_handle(data);
			if (ret) {
				CMR_LOGE("preview failed %d", ret);
				camera_call_cb(CAMERA_EXIT_CB_FAILED,
						camera_get_client_data(),
						CAMERA_FUNC_START_PREVIEW,
						(uint32_t)NULL);
			}
		} else if (IS_CAPTURE) {
			if (!IS_CAP_FRM(data->frame_id)) {
				CMR_LOGE("Wrong frame id %d, drop this frame", data->frame_id);
				return CAMERA_SUCCESS;
			}
			ret = camera_v4l2_capture_handle(data);
			if (ret) {
				CMR_LOGE("capture failed %d", ret);
				camera_call_cb(CAMERA_EXIT_CB_FAILED,
						camera_get_client_data(),
						CAMERA_FUNC_TAKE_PICTURE,
						(uint32_t)NULL);
			}
		}
		break;

	case CMR_V4L2_TX_NO_MEM:
		if (IS_CAPTURE && IMG_DATA_TYPE_JPEG == g_cxt->cap_original_fmt) {
			CMR_LOGE("Need more memory for JPEG capture");
		} else {
			CMR_LOGE("CMR_V4L2_TX_NO_MEM, something wrong");
		}
		break;

	case CMR_V4L2_TX_ERROR:
	case CMR_V4L2_CSI2_ERR:

		CMR_LOGV("Error type 0x%x", evt_type);
		if (IS_PREVIEW) {
			ret = camera_preview_err_handle(evt_type);
			if (ret) {
				CMR_LOGE("Call cb to notice the upper layer something error blocked preview");
				camera_call_cb(CAMERA_EXIT_CB_FAILED,
						camera_get_client_data(),
						CAMERA_FUNC_START_PREVIEW,
						(uint32_t)NULL);

			}
		} else if (IS_CAPTURE) {
			CMR_LOGV("Capture Error.");
			ret = camera_capture_err_handle(evt_type);
			if (ret) {
				camera_call_cb(CAMERA_EXIT_CB_FAILED,
							camera_get_client_data(),
							CAMERA_FUNC_TAKE_PICTURE,
							(uint32_t)NULL);
			}
		}
		CMR_LOGV("Errorhandle done.");
		break;
	default:
		break;

	}

	return ret;
}

int camera_isp_handle(uint32_t evt_type, uint32_t sub_type, void *data)
{
	int                      ret = CAMERA_SUCCESS;
	uint32_t                 cmd;

	CMR_LOGV("sub_type 0x%x, evt_type 0x%x", sub_type, evt_type);

	switch (sub_type & ISP_EVT_MASK) {
	case ISP_PROC_CALLBACK:
		ret = camera_isp_proc_handle((struct ips_out_param*)data);
		break;
	case ISP_AF_NOTICE_CALLBACK:
		ret = camera_isp_af_done(data);
		break;
	default:
		break;
	}
	return ret;

}

int camera_jpeg_encode_handle(JPEG_ENC_CB_PARAM_T *data)
{
	uint32_t                 thumb_size = 0;
	int                      thumb_exist = 1;
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("slice height %d index %d, stream size %d addr 0x%x total height %d",
		data->slice_height,
		g_cxt->jpeg_cxt.index,
		data->stream_size,
		data->stream_buf_vir,
		data->total_height);

	CMR_LOGV("stream buf 0x%x size 0x%x",
		g_cxt->cap_mem[g_cxt->jpeg_cxt.index].target_jpeg.addr_vir.addr_y,
		data->stream_size);

	CMR_PRINT_TIME;
	if (NULL == data || 0 == data->slice_height) {
		return CAMERA_INVALID_PARM;
	}

	if( (((0 != g_cxt->zoom_level) && (ZOOM_POST_PROCESS == g_cxt->cap_zoom_mode))
		|| ((0 != g_cxt->cap_rot)&&(g_cxt->picture_size.width>=1280)))
		&& (data->total_height != g_cxt->picture_size.height)) {
		CMR_LOGI("Dont Need to handle.");
		return ret;
	}
	g_cxt->jpeg_cxt.proc_status.slice_height_out = data->total_height;
	if (g_cxt->jpeg_cxt.proc_status.slice_height_out == g_cxt->picture_size.height) {
		g_cxt->cap_mem[g_cxt->jpeg_cxt.index].target_jpeg.addr_vir.addr_u = data->stream_size;
		CMR_LOGV("Encode done");
#if 0
		ret = camera_save_to_file(990,
			IMG_DATA_TYPE_JPEG,
			g_cxt->picture_size.width,
			g_cxt->picture_size.height,
			&g_cxt->cap_mem[g_cxt->jpeg_cxt.index].target_jpeg.addr_vir);
#endif
		ret = jpeg_stop(g_cxt->jpeg_cxt.handle);
		if (ret) {
			CMR_LOGE("Failed to stop jpeg, %d", ret);
		}

		if (THUM_FROM_CAP != g_cxt->thum_from) {
			if ((0 != g_cxt->thum_size.width) && (0 != g_cxt->thum_size.height)) {
				ret = camera_convert_to_thumb();
				if (ret) {
					thumb_exist = 0;
					CMR_LOGE("Failed to get thumbnail, %d", ret);
				}
			} else {
				thumb_exist = 0;
				CMR_LOGI("dont need thumbnail, %d", ret);
			}
		}
		CMR_PRINT_TIME;

		if (thumb_exist) {
			ret = camera_jpeg_encode_thumb(&thumb_size);
			if (ret) {
				CMR_LOGE("Failed to enc thumbnail, %d", ret);
				thumb_size = 0;
			}
		} else {
			thumb_size = 0;
		}
		ret = camera_jpeg_encode_done(thumb_size);
		g_cxt->jpeg_cxt.jpeg_state = JPEG_IDLE;

	} else {
		CMR_LOGV("Do nothing");
	}
	return ret;

}

int camera_jpeg_decode_handle(JPEG_DEC_CB_PARAM_T *data)
{
	int                      ret = CAMERA_SUCCESS;
	struct frm_info          capture_data;
	if (camera_capture_need_exit()) {
		ret = CAMERA_INVALID_STATE;
		CMR_LOGE("exit.");
		return ret;
	}
	CMR_LOGI("dec total height %d.",data->total_height);
	g_cxt->jpeg_cxt.proc_status.slice_height_out += data->slice_height;
	if (data->total_height >= g_cxt->cap_orig_size.height) {
		jpeg_stop(g_cxt->jpeg_cxt.handle);
		g_cxt->jpeg_cxt.handle = 0;
		g_cxt->jpeg_cxt.jpeg_state = JPEG_IDLE;
		g_cxt->cap_original_fmt = IMG_DATA_TYPE_YUV420;
		ret = camera_capture_yuv_process(&g_cxt->jpeg_cxt.proc_status.frame_info);
	} else {
		ret = camera_jpeg_decode_next(&capture_data);
	}
	return ret;

}
int camera_jpeg_codec_handle(uint32_t evt_type, uint32_t sub_type, void *data)
{
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("status %d, evt 0x%x, data 0x%x",
		g_cxt->jpeg_cxt.jpeg_state, evt_type, (uint32_t)data);

	(void)sub_type;

	if (camera_capture_need_exit()) {
		ret = CAMERA_INVALID_STATE;
		CMR_LOGE("exit.");
		return ret;
	}

	switch (evt_type) {
	case CMR_JPEG_ENC_DONE:
		ret = camera_jpeg_encode_handle((JPEG_ENC_CB_PARAM_T*)data);
		break;
	case CMR_JPEG_DEC_DONE:
		ret = camera_jpeg_decode_handle((JPEG_DEC_CB_PARAM_T*)data);
		break;
	case CMR_JPEG_WEXIF_DONE:
		break;
	case CMR_JPEG_ERR:
		CMR_LOGE("jpeg codec error.");
		break;
	}

	return ret;
}

int camera_scale_handle(uint32_t evt_type, uint32_t sub_type, struct img_frm *data)
{
	int                         ret = CAMERA_SUCCESS;
	struct scaler_context       *cxt = &g_cxt->scaler_cxt;
	uint32_t                    is_start_enc = 0;
	struct jpeg_enc_next_param  enc_nxt_param;

	CMR_LOGV("channel id %d, slice %d",
		cxt->proc_status.frame_info.channel_id,
		data->size.height);

	(void)sub_type;

	if (IMG_CVT_SCALING != g_cxt->scaler_cxt.scale_state) {
		CMR_LOGE("Error state %d", g_cxt->scaler_cxt.scale_state);
		return CAMERA_INVALID_STATE;
	}

	if (0 == cxt->proc_status.frame_info.channel_id) {
		cxt->proc_status.slice_height_out += data->size.height;
		if (cxt->proc_status.slice_height_out < g_cxt->picture_size.height) {
			CMR_LOGV("Scaling need more slice");
			CMR_PRINT_TIME;
			if (IS_CAPTURE) {
				if(cxt->proc_status.slice_height_out >= CMR_SLICE_HEIGHT) {
					if (0 == cxt->proc_status.is_encoding) {
						ret = camera_start_jpeg_encode(&cxt->proc_status.frame_info);
						if (ret) {
							CMR_LOGE("Failed to start jpeg encode %d", ret);
							return -CAMERA_FAILED;
						}
						cxt->proc_status.is_encoding = 1;
					} else {
						bzero(&enc_nxt_param, sizeof(struct jpeg_enc_next_param));
						enc_nxt_param.handle       = g_cxt->jpeg_cxt.handle;
						enc_nxt_param.slice_height = g_cxt->jpeg_cxt.proc_status.slice_height_in;
						enc_nxt_param.ready_line_num = cxt->proc_status.slice_height_out;
						CMR_LOGV("Jpeg need more slice, %d %d",
						g_cxt->jpeg_cxt.proc_status.slice_height_out,
						g_cxt->picture_size.height);
						ret = jpeg_enc_next(&enc_nxt_param);
						if (ret) {
							CMR_LOGE("Failed to next jpeg encode %d", ret);
							return -CAMERA_FAILED;
						}
					}
				} else {
					CMR_LOGV("Slice out height is too short to start endcode %d",
						cxt->proc_status.slice_height_out);
				}
			}
			if (IMG_DATA_TYPE_RAW != g_cxt->cap_original_fmt) {
				ret = camera_scale_next(&cxt->proc_status.frame_info);
				if (CVT_RET_LAST == ret)
					ret = 0;
			}
		} else {
			CMR_LOGV("Scaling done");
			bzero(&enc_nxt_param, sizeof(struct jpeg_enc_next_param));
			enc_nxt_param.handle       = g_cxt->jpeg_cxt.handle;
			enc_nxt_param.slice_height = g_cxt->jpeg_cxt.proc_status.slice_height_in;
			enc_nxt_param.ready_line_num = cxt->proc_status.slice_height_out;
			CMR_LOGV("Jpeg need more slice, %d %d",
			g_cxt->jpeg_cxt.proc_status.slice_height_out,
			g_cxt->picture_size.height);
			ret = jpeg_enc_next(&enc_nxt_param);
			if (ret) {
				CMR_LOGE("Failed to next jpeg encode %d", ret);
				return -CAMERA_FAILED;
			}
			ret = camera_scale_done(&cxt->proc_status.frame_info);
		}
	} else {
		ret = camera_scale_done(&cxt->proc_status.frame_info);
	}

	return ret;

}


int camera_rotation_handle(uint32_t evt_type, uint32_t sub_type, struct img_frm *data)
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

	(void)sub_type;

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
		camera_call_cb(CAMERA_EVT_CB_FRAME,
				camera_get_client_data(),
				CAMERA_FUNC_START_PREVIEW,
				(uint32_t)&frame_type);

	 }else if (IS_CAPTURE){
		if (IMG_ROT_90 == g_cxt->cap_rot ||
			IMG_ROT_270 == g_cxt->cap_rot) {
			tmp = g_cxt->cap_orig_size.width;
			g_cxt->cap_orig_size.width = g_cxt->cap_orig_size.height;
			g_cxt->cap_orig_size.height = tmp;
		}

		CMR_LOGV("orig size %d %d, %d %d",
			g_cxt->cap_orig_size.width, g_cxt->cap_orig_size.height,
			g_cxt->picture_size.width, g_cxt->picture_size.height);

#if 0
		camera_save_to_file(1, IMG_DATA_TYPE_YUV420,
					g_cxt->cap_orig_size.width,
					g_cxt->cap_orig_size.height,
					&g_cxt->cap_mem[0].cap_yuv.addr_vir);
#endif

		if (NO_SCALING) {
			ret = camera_start_jpeg_encode(info);
			if (ret) {
				CMR_LOGE("Failed to start jpeg encode %d", ret);
				return -CAMERA_FAILED;
			}

			if (0 == info->channel_id) {
				frm_id = info->frame_id - CAMERA_CAP0_ID_BASE;
				ret = camera_take_picture_done(info);
				if (ret) {
					CMR_LOGE("Failed to set take_picture done %d", ret);
					return -CAMERA_FAILED;
				}
			} else {
				frm_id = info->frame_id - CAMERA_CAP1_ID_BASE;
			}
		} else {
			ret = camera_start_scale(info);
		}
	} else {
		CMR_LOGV("Wrong status for ROT event");
	}
	g_cxt->rot_cxt.rot_state = IMG_CVT_ROT_DONE;

exit:
	return ret;

}

int camera_sensor_handle(uint32_t evt_type, uint32_t sub_type, struct frm_info *data)
{
	int                      ret = CAMERA_SUCCESS;

	(void)sub_type;

	CMR_LOGE("evt_type %d", evt_type);

	if (CMR_SENSOR_ERROR == evt_type) {
		if (IS_PREVIEW) {
			ret = camera_preview_err_handle(evt_type);
			if (ret) {
				CMR_LOGE("something error blocked preview");
				camera_call_cb(CAMERA_EXIT_CB_FAILED,
						camera_get_client_data(),
						CAMERA_FUNC_START_PREVIEW,
						(uint32_t)NULL);

			}
		}

	}
	return ret;
}

int camera_img_cvt_handle(uint32_t evt_type, uint32_t sub_type, struct img_frm *data)
{
	int                      ret = CAMERA_SUCCESS;

	(void)sub_type;

	CMR_LOGV("evt 0x%x", evt_type);
	if (CMR_IMG_CVT_SC_DONE == evt_type) {
		ret = camera_scale_handle(evt_type, sub_type, data);
	} else {
		ret = camera_rotation_handle(evt_type, sub_type, data);
	}
	return ret;
}

void *camera_af_thread_proc(void *data)
{
	CMR_MSG_INIT(message);
	int ret = CAMERA_SUCCESS;
	int	af_exit_flag = 0;

	CMR_PRINT_TIME;
	sem_post(&g_cxt->af_sync_sem);
	CMR_PRINT_TIME;
	while (1) {
		ret = cmr_msg_get(g_cxt->af_msg_que_handle, &message);
		if (ret) {
			CMR_LOGE("Message queue destroied");
			break;
		}

		CMR_LOGE("message.msg_type 0x%x, data 0x%x", message.msg_type, message.data);

		switch (message.msg_type) {
		case CMR_EVT_AF_INIT:
			CMR_PRINT_TIME;
			ret = camera_autofocus_init();
			if (ret) {
				CMR_LOGE("Failed, %d", ret);
			}
			CMR_PRINT_TIME;
			break;
		case CMR_EVT_AF_START:
			CMR_PRINT_TIME;
			ret = camera_autofocus_start();
			if (CAMERA_INVALID_STATE == ret) {
				camera_call_af_cb(CAMERA_EXIT_CB_ABORT,
					message.data,
					CAMERA_FUNC_START_FOCUS,
					0);
			} else if (CAMERA_FAILED == ret) {
				camera_call_af_cb(CAMERA_EXIT_CB_FAILED,
					message.data,
					CAMERA_FUNC_START_FOCUS,
					0);

			} else {
				camera_call_af_cb(CAMERA_EXIT_CB_DONE,
					message.data,
					CAMERA_FUNC_START_FOCUS,
					0);
			}
			CMR_PRINT_TIME;
			break;
		case CMR_EVT_AF_EXIT:
			CMR_LOGV("AF exit");
			af_exit_flag = 1;
			sem_post(&g_cxt->af_sync_sem);
			CMR_PRINT_TIME;
			break;

		default:
			break;
		}
		if(af_exit_flag) {
			CMR_LOGI("AF proc exit.");
			break;
		}
	}

	CMR_LOGI("exit.");

	return NULL;
}

void camera_set_af_cb(camera_cb_f_type cmr_cb)
{
	pthread_mutex_lock(&g_cxt->af_cb_mutex);
	g_cxt->camera_af_cb = cmr_cb;
	pthread_mutex_unlock(&g_cxt->af_cb_mutex);
	return;
}
void camera_call_af_cb(camera_cb_type cb,
                 const void *client_data,
                 camera_func_type func,
                 int32_t parm4)
{
	pthread_mutex_lock(&g_cxt->af_cb_mutex);
	if (g_cxt->camera_af_cb) {
		(*g_cxt->camera_af_cb)(cb, client_data, func, parm4);
	}
	pthread_mutex_unlock(&g_cxt->af_cb_mutex);
	return;
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
	struct isp_video_start   isp_param;

	sensor_mode = &g_cxt->sn_cxt.sensor_info->sensor_mode_info[g_cxt->sn_cxt.preview_mode];

	g_cxt->prev_rot_index = 0;
	g_cxt->skip_mode = IMG_SKIP_HW;
	g_cxt->skip_num  = g_cxt->sn_cxt.sensor_info->preview_skip_num;
	g_cxt->pre_frm_cnt  = 0;
	v4l2_cfg.cfg0.need_isp = 0;
	v4l2_cfg.cfg0.need_binning = 0;
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
	v4l2_cfg.sn_size.width  = sensor_mode->width;
	v4l2_cfg.sn_size.height = sensor_mode->height;
	v4l2_cfg.channel_num    = 1;
	v4l2_cfg.frm_num        = -1;
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

	g_cxt->preview_rect.start_x	= v4l2_cfg.cfg0.src_img_rect.start_x;
	g_cxt->preview_rect.start_y = v4l2_cfg.cfg0.src_img_rect.start_y;
	g_cxt->preview_rect.width 	= v4l2_cfg.cfg0.src_img_rect.width;
	g_cxt->preview_rect.height 	= v4l2_cfg.cfg0.src_img_rect.height;
	
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
		isp_param.size.w = sensor_mode->width;
		if (v4l2_cfg.cfg0.need_binning) {
			isp_param.size.w = (isp_param.size.w >> 1);
		}
		isp_param.size.h = sensor_mode->height;
		isp_param.format = ISP_DATA_NORMAL_RAW10;
		CMR_LOGV("isp w h, %d %d", isp_param.size.w, isp_param.size.h);
		ret = isp_video_start(&isp_param);
		if (CAMERA_SUCCESS == ret) {
			g_cxt->isp_cxt.isp_state = ISP_COWORK;
		}
	}

exit:
	return ret;
}

int camera_capture_init(int format_mode,takepicture_mode cap_mode)
{
	struct img_size          capture_size;
	int                      ret = CAMERA_SUCCESS;
	struct cap_cfg           v4l2_cfg;
	SENSOR_MODE_INFO_T       *sensor_mode;
	struct buffer_cfg        buffer_info;
	struct isp_video_start   isp_video_param;

	CMR_LOGV("capture size, %d %d, cap_rot %d, cap mode %d",
		g_cxt->capture_size.width,
		g_cxt->capture_size.height,
		g_cxt->cap_rot,
		cap_mode);

	if (CAMERA_HDR_MODE == cap_mode) {
		g_cxt->total_cap_num = HDR_CAP_NUM;
		ret = arithmetic_hdr_init(g_cxt->capture_size.width,g_cxt->capture_size.height);
		if (ret) {
			CMR_LOGE("malloc fail for hdr.");
			goto exit;
		}
	} else {
		g_cxt->total_cap_num = 1;
	}
	g_cxt->cap_cnt = 0;
	g_cxt->thum_ready = 0;

	bzero(&v4l2_cfg, sizeof(struct cap_cfg));
	ret = cmr_v4l2_if_cfg(&g_cxt->sn_cxt.sn_if);
	if (ret) {
		CMR_LOGE("the sensor interface is unsupported by V4L2");
		goto exit;
	}

	ret = camera_snapshot_start_set();
	if (ret) {
		CMR_LOGE("Failed to snapshot");
		goto exit;
	}

	sensor_mode = &g_cxt->sn_cxt.sensor_info->sensor_mode_info[g_cxt->sn_cxt.capture_mode];

	v4l2_cfg.channel_num = 1;
	v4l2_cfg.frm_num = 1;
	/* It can be more than 1*/
	v4l2_cfg.sn_size.width  = sensor_mode->width;
	v4l2_cfg.sn_size.height = sensor_mode->height;

	ret = camera_capture_ability(sensor_mode, &v4l2_cfg.cfg0, &g_cxt->capture_size);
	if (ret) {
		CMR_LOGE("Failed to camera_capture_ability, %d", ret);
		goto exit;
	}

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

	g_cxt->total_cap_ch_num = v4l2_cfg.channel_num;
	g_cxt->cap_ch_cnt = 0;
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
		isp_video_param.size.w = sensor_mode->width;
		isp_video_param.size.h = sensor_mode->height;
		ret = isp_video_start(&isp_video_param);
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
	uint32_t                 search_height = target_size->height;
	uint32_t                 target_mode = SENSOR_MODE_MAX;
	SENSOR_EXP_INFO_T        *sn_info = g_cxt->sn_cxt.sensor_info;
	uint32_t                 last_mode = SENSOR_MODE_PREVIEW_ONE;
	int                      ret = -CAMERA_FAILED;

	if (NULL == target_size || NULL == g_cxt->sn_cxt.sensor_info)
		return ret;

	CMR_LOGV("search_width = %d", search_width);
	for (i = SENSOR_MODE_PREVIEW_ONE; i < SENSOR_MODE_MAX; i++) {
		if (SENSOR_MODE_MAX != sn_info->sensor_mode_info[i].mode) {
			last_mode = i;
			width = sn_info->sensor_mode_info[i].width;
			theight = sn_info->sensor_mode_info[i].height;
			CMR_LOGV("width = %d", width);
			if (search_width <= width && search_height <= theight ) {
				target_mode = i;
				ret = CAMERA_SUCCESS;
				break;
			}
		}
	}

	if (i == SENSOR_MODE_MAX) {
		CMR_LOGV("can't find the right mode, use last available mode %d", last_mode);
		i = last_mode;
		target_mode = last_mode;
		ret = CAMERA_SUCCESS;
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
	uint32_t                 width = 0, height = 0, i, last_one = 0;
	uint32_t                 search_height = target_size->height;
	uint32_t                 target_mode = SENSOR_MODE_MAX;
	SENSOR_EXP_INFO_T        *sn_info = g_cxt->sn_cxt.sensor_info;
	int                      ret = -CAMERA_FAILED;

	if (NULL == target_size || NULL == g_cxt->sn_cxt.sensor_info || NULL == work_mode)
		return ret;

	CMR_LOGV("search_height = %d", search_height);
	for (i = SENSOR_MODE_PREVIEW_ONE; i < SENSOR_MODE_MAX; i++) {
		if (SENSOR_MODE_MAX != sn_info->sensor_mode_info[i].mode) {
			height = sn_info->sensor_mode_info[i].height;
			CMR_LOGV("height = %d", height);
			if (search_height <= height) {
				target_mode = i;
				ret = CAMERA_SUCCESS;
				break;
			} else {
				last_one = i;
			}

		}
	}

	if (i == SENSOR_MODE_MAX) {
		CMR_LOGV("can't find the right mode, %d, use the last one %d", i, last_one);
		target_mode = last_one;
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
		CMR_LOGI("420.");
	} else if (IMG_DATA_TYPE_YUV422 == format) {
		frame_size = buffer_size * 2;
	} else {
		CMR_LOGE("Unsupported format %d", format);
		return -CAMERA_INVALID_PARM;
	}
    frame_size = camera_get_size_align_page(frame_size);
	size = frame_size * CAMERA_PREV_FRM_CNT;

	if (NULL == g_cxt->prev_virt_addr || 0 == g_cxt->prev_phys_addr) {
		CMR_LOGE("Fail to malloc preview memory.");
		ret = -CAMERA_FAILED;
	}

	total_size = CAMERA_PREV_FRM_CNT;
	if (g_cxt->prev_rot) {
		total_size += CAMERA_PREV_ROT_FRM_CNT;
	}
	total_size = total_size * frame_size;

	if (total_size > g_cxt->prev_mem_szie) {
		CMR_LOGE("prev_mem_szie 0x%x, total_size 0x%x", g_cxt->prev_mem_szie, total_size);
		return -CAMERA_NO_MEMORY;
	}

	CMR_LOGV("preview addr, vir 0x%x phy 0x%x", (uint32_t)g_cxt->prev_virt_addr, g_cxt->prev_phys_addr);
	buffer->channel_id = 0;
	buffer->base_id    = CAMERA_PREV_ID_BASE;
	buffer->count      = CAMERA_PREV_FRM_CNT;
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
		for (i = 0; i < CAMERA_PREV_ROT_FRM_CNT; i++) {
			g_cxt->prev_rot_frm[i].addr_vir.addr_y =
				(uint32_t)g_cxt->prev_virt_addr + (uint32_t)((i + buffer->count) * frame_size);
			g_cxt->prev_rot_frm[i].addr_vir.addr_u = g_cxt->prev_rot_frm[i].addr_vir.addr_y + buffer_size;

			g_cxt->prev_rot_frm[i].addr_phy.addr_y =
				(uint32_t)g_cxt->prev_phys_addr + (uint32_t)((i + buffer->count) * frame_size);
			g_cxt->prev_rot_frm[i].addr_phy.addr_u = g_cxt->prev_rot_frm[i].addr_phy.addr_y + buffer_size;
			g_cxt->prev_rot_frm[i].fmt             = format;
			g_cxt->prev_rot_frm[i].size.width      = g_cxt->display_size.width;
			g_cxt->prev_rot_frm[i].size.height     = g_cxt->display_size.height;

		}
	}

	return ret;
}

int camera_capture_ability(SENSOR_MODE_INFO_T *sn_mode,
					struct img_frm_cap *img_cap,
					struct img_size *cap_size)
{
	int                      ret = CAMERA_SUCCESS;
	uint32_t                 tmp_width = 0;
	struct img_frm           *rot_frm;
	struct img_size          sensor_size;
	struct img_rect          sn_trim_rect;

	img_cap->need_isp = 0;
	if (SENSOR_IMAGE_FORMAT_YUV422 == sn_mode->image_format) {
		g_cxt->sn_cxt.sn_if.img_fmt = V4L2_SENSOR_FORMAT_YUV;
		g_cxt->cap_original_fmt = IMG_DATA_TYPE_YUV420;
		g_cxt->cap_zoom_mode = ZOOM_BY_CAP;
	} else if (SENSOR_IMAGE_FORMAT_RAW == sn_mode->image_format) {
/*
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
*/
		img_cap->need_isp = 0;
		g_cxt->cap_original_fmt = IMG_DATA_TYPE_RAW;
		g_cxt->cap_zoom_mode = ZOOM_POST_PROCESS;
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
	sn_trim_rect.start_x = img_cap->src_img_rect.start_x;
	sn_trim_rect.start_y = img_cap->src_img_rect.start_y;
	sn_trim_rect.width   = img_cap->src_img_rect.width;
	sn_trim_rect.height  = img_cap->src_img_rect.height;
	if (ZOOM_POST_PROCESS == g_cxt->cap_zoom_mode) {
		img_cap->src_img_rect.start_x = 0;
		img_cap->src_img_rect.start_y = 0;
		img_cap->src_img_rect.width   = sn_mode->width;
		img_cap->src_img_rect.height  = sn_mode->height;
		img_cap->dst_img_size.width   = sn_mode->width;
		img_cap->dst_img_size.height  = sn_mode->height;
		g_cxt->isp_cxt.drop_slice_num = sn_trim_rect.start_y / CMR_SLICE_HEIGHT;
		g_cxt->isp_cxt.drop_slice_cnt = 0;
		CMR_LOGI("drop cnt %d, drop num %d",
			g_cxt->isp_cxt.drop_slice_cnt,
			g_cxt->isp_cxt.drop_slice_num);
	} else {
		tmp_width = (uint32_t)(g_cxt->v4l2_cxt.sc_factor * img_cap->src_img_rect.width);
		if (g_cxt->capture_size.width <= g_cxt->v4l2_cxt.sc_capability) {
			/* if the capture size is smaller than the scale capability,
				just set the output as capture_size*/
			if ( tmp_width > g_cxt->capture_size.width) {
				img_cap->dst_img_size.width  = g_cxt->capture_size.width;
				img_cap->dst_img_size.height = g_cxt->capture_size.height;
			} else {
				img_cap->dst_img_size.width = tmp_width;
				img_cap->dst_img_size.height =
				(uint32_t)((img_cap->src_img_rect.height * img_cap->dst_img_size.width) /
					img_cap->src_img_rect.width);

			}
		} else {
			/* if the capture size is larger than the scale capability */
			if (g_cxt->capture_size.width > img_cap->src_img_rect.width &&
				g_cxt->v4l2_cxt.sc_capability > img_cap->src_img_rect.width) {
				/* if the scale capability is large than src_img_rect, maybe the output can be sc_capability
				 or g_cxt->v4l2_cxt.sc_factor * img_cap->dst_img_size.width */
				if ( tmp_width > g_cxt->v4l2_cxt.sc_capability) {
					img_cap->dst_img_size.width = g_cxt->v4l2_cxt.sc_capability;
				} else {
					img_cap->dst_img_size.width = tmp_width;
				}
				img_cap->dst_img_size.height =
					(uint32_t)((img_cap->src_img_rect.height * img_cap->dst_img_size.width) /
					img_cap->src_img_rect.width);
				img_cap->dst_img_size.height = CAMERA_HEIGHT(img_cap->dst_img_size.height);
			} else {
				img_cap->dst_img_size.width   = img_cap->src_img_rect.width;
				img_cap->dst_img_size.height  = img_cap->src_img_rect.height;
			}
		}
	}

	g_cxt->cap_orig_size.width    = img_cap->dst_img_size.width;
	g_cxt->cap_orig_size.height   = img_cap->dst_img_size.height;
	CMR_LOGV("cap_orig_size %d %d", g_cxt->cap_orig_size.width, g_cxt->cap_orig_size.height);

	sensor_size.width  = sn_mode->width;
	sensor_size.height = sn_mode->height;
	ret = camera_arrange_capture_buf(&g_cxt->cap_2_mems,
					&sensor_size,
					&sn_trim_rect,
					&g_cxt->max_size,
					g_cxt->cap_original_fmt,
					&g_cxt->cap_orig_size,
					&g_cxt->thum_size,
					g_cxt->cap_mem,
					(IMG_ROT_0 != g_cxt->cap_rot),
					1);
					/*g_cxt->total_cap_num);*/

	if (0 == ret) {
		if (IMG_ROT_0 != g_cxt->cap_rot) {
			rot_frm = &g_cxt->cap_mem[g_cxt->cap_cnt].cap_yuv_rot;

			if (0 == rot_frm->addr_phy.addr_y ||
				0 == rot_frm->addr_phy.addr_u ||
				0 == rot_frm->buf_size) {
				CMR_LOGE("No rotation buffer, %d", g_cxt->cap_rot);
				return -CAMERA_NO_MEMORY;
			}
		}
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

	buffer_size = g_cxt->capture_size.width * g_cxt->capture_size.height;

	if (IMG_DATA_TYPE_RAW == g_cxt->cap_original_fmt) {
		mem_size = g_cxt->cap_mem[cap_index].cap_raw.buf_size;
		y_addr   = g_cxt->cap_mem[cap_index].cap_raw.addr_phy.addr_y;
		u_addr   = y_addr;
		frame_size = buffer_size * RAWRGB_BIT_WIDTH / 8;
	} else if (IMG_DATA_TYPE_JPEG == g_cxt->cap_original_fmt) {
		mem_size = g_cxt->cap_mem[cap_index].target_jpeg.buf_size;
		y_addr   = g_cxt->cap_mem[cap_index].target_jpeg.addr_phy.addr_y;
		u_addr   = y_addr;
		frame_size = CMR_JPEG_SZIE(g_cxt->capture_size.width, g_cxt->capture_size.height);
	} else if (IMG_DATA_TYPE_YUV420 == g_cxt->cap_original_fmt) {
		if (IMG_ROT_0 == g_cxt->cap_rot) {
			if (NO_SCALING) {
				/* It means no need to scale up before JPEG encode */
				mem_size = g_cxt->cap_mem[cap_index].target_yuv.buf_size;
				y_addr   = g_cxt->cap_mem[cap_index].target_yuv.addr_phy.addr_y;
				u_addr   = g_cxt->cap_mem[cap_index].target_yuv.addr_phy.addr_u;
/*				CMR_LOGI("wjp:0 0x%x,0x%x.",y_addr,g_cxt->cap_mem[cap_index].target_yuv.addr_vir.addr_y);*/
			} else {
				mem_size = g_cxt->cap_mem[cap_index].cap_yuv.buf_size;
				y_addr   = g_cxt->cap_mem[cap_index].cap_yuv.addr_phy.addr_y;
				u_addr   = g_cxt->cap_mem[cap_index].cap_yuv.addr_phy.addr_u;
/*				CMR_LOGI("wjp:1 0x%x,0x%x.",y_addr,g_cxt->cap_mem[cap_index].cap_yuv.addr_vir.addr_y);*/
			}
			frame_size = buffer_size * 3 / 2;
		} else {
			mem_size = g_cxt->cap_mem[cap_index].cap_yuv_rot.buf_size;
			y_addr   = g_cxt->cap_mem[cap_index].cap_yuv_rot.addr_phy.addr_y;
			u_addr   = g_cxt->cap_mem[cap_index].cap_yuv_rot.addr_phy.addr_u;
			frame_size = g_cxt->cap_orig_size.width*g_cxt->cap_orig_size.height*3/2;
/*			CMR_LOGI("wjp:2 0x%x,0x%x.",y_addr,g_cxt->cap_mem[cap_index].cap_yuv_rot.addr_vir.addr_y);*/
		}
	} else {
		CMR_LOGE("Unsupported capture format!");
		return -CAMERA_NOT_SUPPORTED;
	}

	if (0 == y_addr || 0 == u_addr || frame_size > mem_size) {
		CMR_LOGE("Fail to malloc capture memory. 0x%x 0x%x 0x%x 0x%x",
			y_addr, u_addr, frame_size, mem_size);
		return -CAMERA_NO_MEMORY;
	}

	CMR_LOGV("capture addr, y 0x%x uv 0x%x", y_addr, u_addr);
	buffer->channel_id = 0;
	buffer->base_id    = CAMERA_CAP0_ID_BASE + cap_index;
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
	buffer->base_id    = CAMERA_CAP1_ID_BASE + cap_index;
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

int camera_capture_get_buffer_size(uint32_t camera_id,
						uint32_t width,
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

	ret = camera_capture_buf_size(camera_id,
					g_cxt->sn_cxt.sensor_info->image_format,
					&local_size,
					size0,
					size1);

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
	struct img_size          max_size;
	int                      ret = CAMERA_SUCCESS;

	if (cap_index > CAMERA_CAP_FRM_CNT) {
		CMR_LOGE("Invalid cap_index %d", cap_index);
		return -CAMERA_NO_MEMORY;
	}
	if (0 == phy_addr0 || 0 == vir_addr0 || 0 == mem_size0) {
		CMR_LOGE("Invalid parameter 0x%x 0x%x 0x%x", phy_addr0, vir_addr0, mem_size0);
		return -CAMERA_NO_MEMORY;
	}

	g_cxt->cap_2_mems.major_frm.buf_size = mem_size0;
	g_cxt->cap_2_mems.major_frm.addr_phy.addr_y = phy_addr0;
	g_cxt->cap_2_mems.major_frm.addr_vir.addr_y = vir_addr0;

	g_cxt->cap_2_mems.minor_frm.buf_size = mem_size1;
	g_cxt->cap_2_mems.minor_frm.addr_phy.addr_y = phy_addr1;
	g_cxt->cap_2_mems.minor_frm.addr_vir.addr_y = vir_addr1;

	return ret;
}

int camera_v4l2_preview_handle(struct frm_info *data)
{
	camera_frame_type        frame_type;
	int                      ret = CAMERA_SUCCESS;

	CMR_PRINT_TIME;

	if (V4L2_IDLE == g_cxt->v4l2_cxt.v4l2_state) {
		CMR_LOGV("V4L2 Cap Stopped, skip this frtame");
		return ret;
	}

	g_cxt->pre_frm_cnt++;
	if (IMG_SKIP_SW == g_cxt->skip_mode) {
		if (g_cxt->pre_frm_cnt <= g_cxt->skip_num) {
			CMR_LOGV("Ignore this frame, preview cnt %d, total skip num %d",
				g_cxt->pre_frm_cnt, g_cxt->skip_num);
			ret = cmr_v4l2_free_frame(data->channel_id, data->frame_id);
			return ret;
		}
	}

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

	if (g_cxt->recover_status) {
		CMR_LOGV("Reset the recover status");
		g_cxt->recover_status = NO_RECOVERY;
	}
	return ret;
}

int camera_preview_err_handle(uint32_t evt_type)
{
	uint32_t                 rs_mode = RESTART_MAX;
	int                      ret = CAMERA_SUCCESS;

	if (RESTART == g_cxt->recover_status) {
		CMR_LOGE("No way to recover");
		return CAMERA_FAILED;
	}

	switch (evt_type) {

	case CMR_V4L2_CSI2_ERR:
	case CMR_V4L2_TX_ERROR:

		if (RECOVERING == g_cxt->recover_status) {
			/* when in recovering */
			g_cxt->recover_cnt --;
			CMR_LOGI("recover_cnt, %d", g_cxt->recover_cnt);
			if (g_cxt->recover_cnt) {
			/* try once more */
				rs_mode = RESTART_MIDDLE;
			} else {
			/* tried three times, it hasn't recovered yet, restart */
				rs_mode = RESTART_HEAVY;
				g_cxt->recover_status = RESTART;
			}
		} else {
			/* not in recovering, start to recover by three times */
			rs_mode = RESTART_MIDDLE;
			g_cxt->recover_status = RECOVERING;
			g_cxt->recover_cnt = CAMERA_RECOVER_CNT;
			CMR_LOGI("Need recover, recover_cnt, %d", g_cxt->recover_cnt);
		}
		break;

	case CMR_SENSOR_ERROR:
		rs_mode = RESTART_HEAVY;
		g_cxt->camera_status = RESTART;
		CMR_LOGI("Sensor error, restart preview");
		break;

	}

	CMR_LOGV("rs_mode %d, recover_status %d", rs_mode, g_cxt->recover_status);

	ret = camera_before_set_internal(rs_mode);
	if (ret) {
		CMR_LOGV("Failed to stop preview %d", ret);
		return CAMERA_FAILED;
	}
	ret = camera_after_set_internal(rs_mode);
	if (ret) {
		CMR_LOGV("Failed to start preview %d", ret);
	}

	return ret;
}

int camera_capture_err_handle(uint32_t evt_type)
{
	uint32_t                 rs_mode = RESTART_MAX;
	int                      ret = CAMERA_SUCCESS;

	if (RESTART == g_cxt->recover_status) {
		CMR_LOGE("No way to recover");
		return CAMERA_FAILED;
	}
	ret = cmr_v4l2_cap_stop();
	if (ret) {
		CMR_LOGE("Failed to stop v4l2 capture, %d", ret);
		return -CAMERA_FAILED;
	}
	CMR_PRINT_TIME;
	ret = Sensor_StreamOff();
	if (ret) {
		CMR_LOGE("Failed to switch off the sensor stream, %d", ret);
	}
	if (RECOVERING == g_cxt->recover_status) {
		g_cxt->recover_cnt --;
		CMR_LOGI("recover_cnt, %d", g_cxt->recover_cnt);
		if (0 == g_cxt->recover_cnt) {
			CMR_LOGE("restart fail.");
			ret = -CAMERA_FAILED;
		}
	} else {
		g_cxt->recover_status = RECOVERING;
		g_cxt->recover_cnt = CAMERA_RECOVER_CNT;
		CMR_LOGI("Need recover, recover_cnt, %d", g_cxt->recover_cnt);
	}
	if (CAMERA_SUCCESS == ret) {
		ret = camera_take_picture_internal(g_cxt->cap_mode);
		if (ret) {
			CMR_LOGE("restart fail.");
		}
	}

	CMR_LOGV("restart ret %d.",ret);
	return ret;
}
int camera_capture_yuv_process(struct frm_info *data)
{
	int                      ret = CAMERA_SUCCESS;

	if (camera_capture_need_exit()) {
		ret = CAMERA_INVALID_STATE;
		CMR_LOGE("exit.");
		return ret;
	}
	CMR_PRINT_TIME;
	if (IMG_ROT_0 == g_cxt->cap_rot) {
		if (NO_SCALING) {
			ret = camera_start_jpeg_encode(data);
			if (ret) {
				CMR_LOGE("Failed to start jpeg encode %d", ret);
			} else {
				if (IMG_DATA_TYPE_RAW != g_cxt->cap_original_fmt) {
					ret = camera_take_picture_done(data);
					if (ret) {
						CMR_LOGE("Failed to set take_picture done %d", ret);
					}
				}
			}
		} else {
			ret = camera_start_scale(data);
		}
	} else {
#if 0
		camera_save_to_file(0, IMG_DATA_TYPE_YUV420,
			g_cxt->cap_orig_size.width,
			g_cxt->cap_orig_size.height,
				&g_cxt->cap_mem[0].cap_yuv_rot.addr_vir);
#endif
		ret = camera_start_rotate(data);
	}

	return ret;

}

void camera_capture_hdr_data(struct frm_info *data)
{
	unsigned char *addr = NULL;
	uint32_t      size = g_cxt->capture_size.width*g_cxt->capture_size.height*3/2;
	uint32_t      frm_id = 0;//data->frame_id - CAMERA_CAP0_ID_BASE;
	CMR_LOGI(" s.");
	if (IMG_ROT_0 == g_cxt->cap_rot) {
		if (NO_SCALING) {
			addr = (unsigned char*)g_cxt->cap_mem[frm_id].target_yuv.addr_vir.addr_y;
		} else {
			addr = (unsigned char*)g_cxt->cap_mem[frm_id].cap_yuv.addr_vir.addr_y;
		}
	} else {
		addr = (unsigned char*)g_cxt->cap_mem[frm_id].cap_yuv_rot.addr_vir.addr_y;
	}
	arithmetic_hdr_data(addr, size,g_cxt->cap_cnt);
	CMR_LOGI(" e.");
}

int camera_v4l2_capture_handle(struct frm_info *data)
{
	camera_frame_type        frame_type;
	int                      ret = CAMERA_SUCCESS;

	if (NULL == data) {
		CMR_LOGE("Invalid parameter, 0x%x", (uint32_t)data);
		return -CAMERA_INVALID_PARM;
	}

	CMR_PRINT_TIME;
	g_cxt->cap_ch_cnt ++;
	CMR_LOGV("cap_ch_cnt %d, total_cap_ch_num %d",
		g_cxt->cap_ch_cnt, g_cxt->total_cap_ch_num);
	g_cxt->cap_cnt ++;
	if (g_cxt->cap_ch_cnt == g_cxt->total_cap_ch_num) {
		if (g_cxt->cap_cnt <= g_cxt->total_cap_num) {
			if (HDR_CAP_NUM == g_cxt->total_cap_num) {
				g_cxt->cap_process_id = 0;
			}
			ret = cmr_v4l2_cap_stop();
			if (ret) {
				CMR_LOGE("Failed to stop v4l2 capture, %d", ret);
				return -CAMERA_FAILED;
			}
			CMR_PRINT_TIME;
			ret = Sensor_StreamOff();
			if (ret) {
				CMR_LOGE("Failed to switch off the sensor stream, %d", ret);
			}
			CMR_PRINT_TIME;
			if (HDR_CAP_NUM == g_cxt->total_cap_num) {
				camera_capture_hdr_data	(data);
			}
			if (g_cxt->cap_cnt == g_cxt->total_cap_num) {
				ret = camera_snapshot_stop_set();
				if (ret) {
					CMR_LOGE("Failed to exit snapshot %d", ret);
					return -CAMERA_FAILED;
				}

			} else {
				ret = camera_take_picture_hdr();
				if (ret) {
					CMR_LOGE("Failed to camera_take_picture_hdr %d.", ret);
					return -CAMERA_FAILED;
				} else {
					CMR_LOGI("exit.");
					return ret;
				}
			}
		}
	}
	if (camera_capture_need_exit()) {
		ret = CAMERA_INVALID_STATE;
		CMR_LOGE("exit.");
		return ret;
	}
	CMR_PRINT_TIME;
	if (HDR_CAP_NUM == g_cxt->cap_cnt) {
		if(0 != arithmetic_hdr((unsigned char*)g_cxt->cap_mem[0].cap_yuv.addr_vir.addr_y,
								g_cxt->capture_size.width,g_cxt->capture_size.height)) {
			CMR_LOGE("hdr error.");
		}
		arithmetic_hdr_deinit();
	}
	CMR_PRINT_TIME;
	if (0 == data->channel_id) {
		CMR_LOGV("channel 0 capture done, cap_original_fmt %d, cap_zoom_mode %d, rot %d",
			g_cxt->cap_original_fmt,
			g_cxt->cap_zoom_mode,
			g_cxt->cap_rot);
		camera_call_cb(CAMERA_RSP_CB_SUCCESS,
			camera_get_client_data(),
			CAMERA_FUNC_TAKE_PICTURE,
			0);
		data->frame_id = CAMERA_CAP0_ID_BASE;
		switch (g_cxt->cap_original_fmt) {
		case IMG_DATA_TYPE_RAW:
			ret = camera_start_isp_process(data);
			break;
		case IMG_DATA_TYPE_JPEG:
			ret = camera_start_jpeg_decode(data);
			break;
		case IMG_DATA_TYPE_YUV420:
			ret = camera_capture_yuv_process(data);
			break;
		default:
			break;
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
	struct ips_in_param      ips_in;
	uint32_t                 frm_id;
	int                      ret = CAMERA_SUCCESS;
	struct ips_out_param     ips_out;

	if (camera_capture_need_exit()) {
		ret = CAMERA_INVALID_STATE;
		CMR_LOGE("exit.");
		return ret;
	}

	if (NULL == data) {
		return -CAMERA_INVALID_PARM;
	}

	frm_id = data->frame_id - CAMERA_CAP0_ID_BASE;

	if (g_cxt->sn_cxt.sn_if.if_type) {
		ips_in.src_frame.img_fmt = ISP_DATA_CSI2_RAW10;
	} else {
		ips_in.src_frame.img_fmt = ISP_DATA_NORMAL_RAW10;
	}
	ips_in.src_frame.img_addr_phy.chn0 = g_cxt->cap_mem[frm_id].cap_raw.addr_phy.addr_y;
	ips_in.src_frame.img_size.w = g_cxt->cap_mem[frm_id].cap_raw.size.width;
	ips_in.src_frame.img_size.h = g_cxt->cap_mem[frm_id].cap_raw.size.height;
	ips_in.dst_frame.img_addr_phy.chn0 = g_cxt->cap_mem[frm_id].cap_yuv.addr_phy.addr_y;
	g_cxt->isp_cxt.cur_dst.addr_y = g_cxt->cap_mem[frm_id].cap_yuv.addr_vir.addr_y;
#if CMR_ISP_YUV422
	ips_in.dst_frame.img_fmt = ISP_DATA_YVU422_2FRAME;
	ips_in.dst_frame.img_addr_phy.chn1 = g_cxt->cap_mem[frm_id].isp_tmp.addr_phy.addr_y;
	g_cxt->isp_cxt.cur_dst.addr_u = g_cxt->cap_mem[frm_id].isp_tmp.addr_vir.addr_y;
#else
	ips_in.dst_frame.img_fmt = ISP_DATA_YVU420_2FRAME;
	ips_in.dst_frame.img_addr_phy.chn1 = g_cxt->cap_mem[frm_id].cap_yuv.addr_phy.addr_u;
	g_cxt->isp_cxt.cur_dst.addr_u = g_cxt->cap_mem[frm_id].cap_yuv.addr_vir.addr_u;
#endif
	ips_in.dst_frame.img_size.w = g_cxt->cap_mem[frm_id].cap_yuv.size.width;
	ips_in.dst_frame.img_size.h = g_cxt->cap_mem[frm_id].cap_yuv.size.height;
	ips_in.src_avail_height = g_cxt->cap_mem[frm_id].cap_raw.size.height;

	ips_in.src_slice_height = CMR_SLICE_HEIGHT;
	ips_in.dst_slice_height = CMR_SLICE_HEIGHT;

#if 0
	camera_save_to_file(110,
			IMG_DATA_TYPE_RAW,
			g_cxt->cap_mem[frm_id].cap_raw.size.width,
			g_cxt->cap_mem[frm_id].cap_raw.size.height,
			&g_cxt->cap_mem[frm_id].cap_raw.addr_vir);
#endif

	ret = isp_proc_start(&ips_in, &ips_out);
	if (0 == ret) {
		CMR_LOGV("ISP post-process started");
		g_cxt->isp_cxt.isp_state = ISP_POST_PROCESS;
		g_cxt->isp_cxt.proc_status.slice_height_in = CMR_SLICE_HEIGHT;
		g_cxt->isp_cxt.proc_status.slice_height_out = 0;
		g_cxt->isp_cxt.is_first_slice = 1;
		memcpy((void*)&g_cxt->isp_cxt.proc_status.frame_info,
			(void*)data,
			sizeof(struct frm_info));
		g_cxt->isp_cxt.proc_status.frame_info.data_endian.y_endian = 1;
		g_cxt->isp_cxt.proc_status.frame_info.data_endian.uv_endian = 2;
	} else {
		CMR_LOGV("Failed to start ISP, %d", ret);
	}
	return ret;
}

int camera_start_jpeg_decode(struct frm_info *data)
{
	struct jpeg_dec_in_param        dec_in;
	struct jpeg_dec_out_param       dec_out;
	uint32_t                 frm_id;
	struct img_frm           *frm;
	int                      ret = CAMERA_SUCCESS;

	if (camera_capture_need_exit()) {
		ret = CAMERA_INVALID_STATE;
		CMR_LOGE("exit.");
		return ret;
	}
	if (NULL == data || !IS_CAP_FRM(data->frame_id)) {
		CMR_LOGE("Parameter error, data 0x%x, frame id 0x%x",
			(uint32_t)data, data->frame_id);
		return -CAMERA_INVALID_PARM;
	}

	frm_id = data->frame_id - CAMERA_CAP0_ID_BASE;
	if (frm_id > CAMERA_CAP_FRM_CNT) {
		CMR_LOGE("Wrong Frame id, 0x%x", data->frame_id);
		return -CAMERA_INVALID_PARM;
	}
	frm = &g_cxt->cap_mem[frm_id].target_jpeg;
	dec_in.stream_buf_phy       = frm->addr_phy.addr_y;
	dec_in.stream_buf_vir       = frm->addr_vir.addr_y;
	dec_in.stream_buf_size      = data->length;
	dec_in.size.width           = g_cxt->cap_orig_size.width;
	dec_in.size.height          = g_cxt->cap_orig_size.height;
	dec_in.dst_endian.y_endian  = 1;//TBD
	dec_in.dst_endian.uv_endian = 2;//TBD
	dec_in.slice_height         = dec_in.size.height;//CMR_SLICE_HEIGHT;

	if (IMG_ROT_0 == g_cxt->cap_rot) {
		if (0 == g_cxt->zoom_level) {
			frm = &g_cxt->cap_mem[frm_id].target_yuv;
		} else {
			frm = &g_cxt->cap_mem[frm_id].cap_yuv;
		}
	} else {
		frm = &g_cxt->cap_mem[frm_id].cap_yuv_rot;
	}

	dec_in.dst_addr_phy.addr_y  = frm->addr_phy.addr_y;
	dec_in.dst_addr_phy.addr_u  = frm->addr_phy.addr_u;
	dec_in.dst_addr_vir.addr_y  = frm->addr_vir.addr_y;
	dec_in.dst_addr_vir.addr_u  = frm->addr_vir.addr_u;
	dec_in.dst_fmt              = IMG_DATA_TYPE_YUV420;
	dec_in.temp_buf_phy         = g_cxt->cap_mem[frm_id].jpeg_tmp.addr_phy.addr_y;
	dec_in.temp_buf_vir         = g_cxt->cap_mem[frm_id].jpeg_tmp.addr_vir.addr_y;
	dec_in.temp_buf_size        = g_cxt->cap_mem[frm_id].jpeg_tmp.buf_size;
	dec_in.slice_mod 			= JPEG_YUV_SLICE_ONE_BUF;
	ret = jpeg_dec_start(&dec_in, &dec_out);
	if (0 == ret) {
		CMR_LOGV("OK, handle 0x%x", dec_out.handle);
		g_cxt->jpeg_cxt.handle = dec_out.handle;
		g_cxt->jpeg_cxt.proc_status.slice_height_out = 0;
		g_cxt->jpeg_cxt.index = g_cxt->cap_cnt;
		memcpy((void*)&g_cxt->jpeg_cxt.proc_status.frame_info, data, sizeof(struct frm_info));
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
	struct jpeg_dec_next_param       dec_in;

	if (NULL == data) {
		return -CAMERA_INVALID_PARM;
	}
	bzero(&dec_in,sizeof(struct jpeg_dec_next_param));
	dec_in.handle = g_cxt->jpeg_cxt.handle;
	ret = jpeg_dec_next(&dec_in);
	return ret;

}

int camera_jpeg_encode_done(uint32_t thumb_stream_size)
{
	JPEGENC_CBrtnType        encoder_param;
	camera_encode_mem_type   encoder_type;
	struct img_frm           *jpg_frm;
	JINF_EXIF_INFO_T         *exif_ptr;
	struct jpeg_enc_exif_param      wexif_param;
	struct jpeg_wexif_cb_param    wexif_output;
	int                      ret = CAMERA_SUCCESS;

	jpg_frm = &g_cxt->cap_mem[g_cxt->jpeg_cxt.index].target_jpeg;

	CMR_LOGV("index %d, bitstream size %d", g_cxt->jpeg_cxt.index, jpg_frm->addr_vir.addr_u);
	//camera_set_position(NULL,0,0);
	exif_ptr = camera_get_exif(g_cxt);
	bzero(&encoder_param, sizeof(JPEGENC_CBrtnType));
	encoder_param.header_size = 0;
	encoder_param.mode        = JPEGENC_MEM;
	encoder_type.buffer       = (uint8_t *)jpg_frm->addr_vir.addr_y;
	encoder_param.size        = jpg_frm->addr_vir.addr_u;
	encoder_param.outPtr      = &encoder_type;
	encoder_param.status      = JPEGENC_IMG_DONE;

	wexif_param.exif_ptr = exif_ptr;
	wexif_param.src_jpeg_addr_virt = (uint32_t)encoder_type.buffer;
	wexif_param.src_jpeg_size = encoder_param.size;
	wexif_param.target_addr_virt = wexif_param.src_jpeg_addr_virt - JPEG_EXIF_SIZE;
	wexif_param.target_size = JPEG_EXIF_SIZE + wexif_param.src_jpeg_size;
	jpg_frm = &g_cxt->cap_mem[g_cxt->jpeg_cxt.index].thum_jpeg;
	if ((0 != g_cxt->thum_size.width) && (0 != g_cxt->thum_size.height)) {
		wexif_param.thumbnail_addr_virt = jpg_frm->addr_vir.addr_y;
		wexif_param.thumbnail_size = thumb_stream_size;
	} else {
		wexif_param.thumbnail_addr_virt = 0;
		wexif_param.thumbnail_size = 0;
	}

	ret = jpeg_enc_add_eixf(&wexif_param,&wexif_output);

	if (0 == ret) {
		encoder_type.buffer = (uint8_t *)wexif_output.output_buf_virt_addr;
		encoder_param.size  = wexif_output.output_buf_size;
		camera_call_cb(CAMERA_EXIT_CB_DONE,
				camera_get_client_data(),
				CAMERA_FUNC_ENCODE_PICTURE,
				(uint32_t)&encoder_param);
	}
	return ret;
}

int camera_start_jpeg_encode(struct frm_info *data)
{
	uint32_t                 frm_id;
	struct img_frm           *src_frm;
	struct img_frm           *target_frm;
	struct img_frm           *tmp_frm;
	int                      ret = CAMERA_SUCCESS;
	struct jpeg_enc_in_param  in_parm;
	struct jpeg_enc_out_param    out_parm;

	if (camera_capture_need_exit()) {
		ret = CAMERA_INVALID_STATE;
		CMR_LOGE("exit.");
		return ret;
	}

	if (NULL == data || JPEG_ENCODE == g_cxt->jpeg_cxt.jpeg_state) {
		CMR_LOGV("wrong parameter 0x%x or status %d",
			(uint32_t)data,
			g_cxt->jpeg_cxt.jpeg_state);
		return -CAMERA_INVALID_PARM;
	}

	CMR_LOGV("channel_id %d, frame_id 0x%x", data->channel_id, data->frame_id);

	if (0 == data->channel_id) {
		frm_id = data->frame_id - CAMERA_CAP0_ID_BASE;
		if (frm_id > CAMERA_CAP_FRM_CNT) {
			CMR_LOGE("Wrong Frame id, 0x%x", data->frame_id);
			return -CAMERA_INVALID_PARM;
		}
		src_frm    = &g_cxt->cap_mem[frm_id].target_yuv;
		target_frm = &g_cxt->cap_mem[frm_id].target_jpeg;
	} else {
		frm_id = data->frame_id - CAMERA_CAP1_ID_BASE;
		if (frm_id > CAMERA_CAP_FRM_CNT) {
			CMR_LOGE("Wrong Frame id, 0x%x", data->frame_id);
			return -CAMERA_INVALID_PARM;
		}
		src_frm    = &g_cxt->cap_mem[frm_id].thum_yuv;
		target_frm = &g_cxt->cap_mem[frm_id].thum_jpeg;
	}
	tmp_frm = &g_cxt->cap_mem[frm_id].jpeg_tmp;

	in_parm.quality_level        = g_cxt->jpeg_cxt.quality;
	in_parm.slice_mod            = JPEG_YUV_SLICE_ONE_BUF;
	in_parm.size.width           = g_cxt->picture_size.width;
	in_parm.size.height          = g_cxt->picture_size.height;
	in_parm.src_addr_phy.addr_y  = src_frm->addr_phy.addr_y;
	in_parm.src_addr_phy.addr_u  = src_frm->addr_phy.addr_u;
	in_parm.src_addr_vir.addr_y  = src_frm->addr_vir.addr_y;
	in_parm.src_addr_vir.addr_u  = src_frm->addr_vir.addr_u;

	if (NO_SCALING && IMG_DATA_TYPE_RAW != g_cxt->cap_original_fmt) {
		in_parm.slice_height = in_parm.size.height;
	} else {
		in_parm.slice_height = CMR_SLICE_HEIGHT;
	}
	in_parm.src_endian.y_endian  = data->data_endian.y_endian;
	in_parm.src_endian.uv_endian = data->data_endian.uv_endian;
	in_parm.stream_buf_phy       = target_frm->addr_phy.addr_y;
	in_parm.stream_buf_vir       = target_frm->addr_vir.addr_y;
	in_parm.stream_buf_size      = target_frm->buf_size;
	in_parm.temp_buf_phy         = tmp_frm->addr_phy.addr_y;
	in_parm.temp_buf_vir         = tmp_frm->addr_vir.addr_y;
	in_parm.temp_buf_size        = tmp_frm->buf_size;

	CMR_LOGI("w h, %d %d, quality level %d", in_parm.size.width, in_parm.size.height,
		in_parm.quality_level);
	CMR_LOGI("slice height, %d, slice mode %d", in_parm.slice_height, in_parm.slice_mod);
	CMR_LOGI("phy addr 0x%x 0x%x, vir addr 0x%x 0x%x",
		in_parm.src_addr_phy.addr_y, in_parm.src_addr_phy.addr_u,
		in_parm.src_addr_vir.addr_y, in_parm.src_addr_vir.addr_u);

	CMR_LOGI("endian %d %d", in_parm.src_endian.y_endian, in_parm.src_endian.uv_endian);
	CMR_LOGI("stream phy 0x%x vir 0x%x, size 0x%x",
		in_parm.stream_buf_phy,
		in_parm.stream_buf_vir,
		in_parm.stream_buf_size);
	CMR_LOGI("temp_buf phy 0x%x vir 0x%x, size 0x%x",
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
	uint32_t                 offset = 0;
	int                      ret = CAMERA_SUCCESS;
	struct scaler_context    *cxt = &g_cxt->scaler_cxt;

	if (camera_capture_need_exit()) {
		ret = CAMERA_INVALID_STATE;
		CMR_LOGE("exit.");
		return ret;
	}
	ret = camera_scaler_init();
	if (ret) {
		CMR_LOGE("Failed to init scaler, %d", ret);
		return ret;
	}

	if (0 == data->channel_id) {

		frm_id = data->frame_id - CAMERA_CAP0_ID_BASE;

		if (g_cxt->cap_zoom_mode == ZOOM_BY_CAP) {
			rect.start_x = 0;
			rect.start_y = 0;
			rect.width   = g_cxt->cap_orig_size.width;
			rect.height  = g_cxt->cap_orig_size.height;
		} else {
			if (IMG_ROT_0 == g_cxt->cap_rot) {
				sensor_mode = &g_cxt->sn_cxt.sensor_info->sensor_mode_info[g_cxt->sn_cxt.capture_mode];
				rect.start_x = sensor_mode->trim_start_x;
				rect.start_y = sensor_mode->trim_start_y;
				rect.width   = sensor_mode->trim_width;
				rect.height  = sensor_mode->trim_height;
			} else {
				rect.start_x = 0;
				rect.start_y = 0;
				rect.width   = g_cxt->cap_orig_size.width;
				rect.height  = g_cxt->cap_orig_size.height;
			}
			ret = camera_get_trim_rect(&rect, g_cxt->zoom_level, &g_cxt->picture_size);
			if (ret) {
				CMR_LOGE("Failed to calculate scaling window, %d", ret);
				return ret;
			}
			g_cxt->isp_cxt.drop_slice_cnt ++;
			CMR_LOGV("drop slice cnt %d, drop total num %d, rect.start_y %d",
				g_cxt->isp_cxt.drop_slice_cnt,
				g_cxt->isp_cxt.drop_slice_num,
				rect.start_y);
			if (g_cxt->isp_cxt.drop_slice_cnt > g_cxt->isp_cxt.drop_slice_num) {
				rect.start_y -= (uint32_t)(g_cxt->isp_cxt.drop_slice_num * CMR_SLICE_HEIGHT);
				offset = (uint32_t)(g_cxt->isp_cxt.drop_slice_num *
						CMR_SLICE_HEIGHT * g_cxt->cap_orig_size.width);
				CMR_LOGV("New start_y %d, width %d, offset 0x%x",
					rect.start_y,
					g_cxt->cap_orig_size.width,
					offset);
			} else {
				CMR_LOGV("drop this slice");
				return ret;
			}
		}

		if (rect.width == g_cxt->picture_size.width &&
			rect.height == g_cxt->picture_size.height) {
			CMR_LOGV("No need to to scaling");
			ret = camera_scale_done(data);
			return ret;
		}
		src_frame.size.width      = g_cxt->cap_orig_size.width;
		src_frame.size.height     = g_cxt->cap_orig_size.height;
		src_frame.addr_phy.addr_y = g_cxt->cap_mem[frm_id].cap_yuv.addr_phy.addr_y + offset;
		src_frame.addr_phy.addr_u = g_cxt->cap_mem[frm_id].cap_yuv.addr_phy.addr_u + (offset >> 1);
		src_frame.addr_phy.addr_v = g_cxt->cap_mem[frm_id].cap_yuv.addr_phy.addr_v + (offset >> 1);

		dst_frame.size.width      = g_cxt->picture_size.width;
		dst_frame.size.height     = g_cxt->picture_size.height;
		memcpy((void*)&dst_frame.addr_phy,
			&g_cxt->cap_mem[frm_id].target_yuv.addr_phy,
			sizeof(struct img_addr));
		memcpy((void*)&dst_frame.addr_vir,
			&g_cxt->cap_mem[frm_id].target_yuv.addr_vir,
			sizeof(struct img_addr));
/*
		if (dst_frame.size.width <= g_cxt->scaler_cxt.sc_capability) {
			slice_h = dst_frame.size.height;
		} else {
			slice_h = CMR_SLICE_HEIGHT;
		}
*/
		slice_h = CMR_SLICE_HEIGHT;

		dst_frame.fmt = IMG_DATA_TYPE_YUV420;
		cxt->proc_status.slice_height_out = 0;
		cxt->out_fmt = dst_frame.fmt;
	} else {

		rect.start_x = 0;
		rect.start_y = 0;
		rect.width   = g_cxt->picture_size.width;
		rect.height  = g_cxt->picture_size.height;

		frm_id = data->frame_id - CAMERA_CAP0_ID_BASE;
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
		memcpy((void*)&dst_frame.addr_phy,
			(void*)&g_cxt->cap_mem[frm_id].thum_yuv.addr_phy,
			sizeof(struct img_addr));
		memcpy((void*)&dst_frame.addr_vir,
			(void*)&g_cxt->cap_mem[frm_id].thum_yuv.addr_vir,
			sizeof(struct img_addr));
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
			&g_cxt->cap_mem[frm_id].scale_tmp,
			NULL);
	if (ret) {
		CMR_LOGE("Failed to start scaler, %d", ret);
		camera_scaler_deinit();
		return ret;
	}

	cxt->proc_status.frame_info = *data;
	cxt->proc_status.slice_height_in = slice_h;
	cxt->proc_status.is_encoding = 0;
	g_cxt->scaler_cxt.scale_state = IMG_CVT_SCALING;

	return ret;
}

int camera_scale_next(struct frm_info *data)
{
	struct scaler_context    *cxt = &g_cxt->scaler_cxt;
	int                      ret = CAMERA_SUCCESS;

	if (camera_capture_need_exit()) {
		ret = CAMERA_INVALID_STATE;
		CMR_LOGE("exit.");
		return ret;
	}
	if (IMG_DATA_TYPE_YUV422 == cxt->out_fmt) {

	}
	ret = cmr_scale_next(0, NULL, NULL, NULL);

	return ret;
}

int camera_scale_done(struct frm_info *data)
{
	uint32_t                 frm_id;
	int                      ret = CAMERA_SUCCESS;
	struct frm_info          frm;

	memcpy((void*)&frm, (void*)data, sizeof(struct frm_info));
#if 0
	if(g_cxt->cap_zoom_mode == ZOOM_BY_CAP) {
		ret = camera_start_jpeg_encode(data);
		if (ret) {
			CMR_LOGE("Failed to start jpeg encode %d", ret);
			return -CAMERA_FAILED;
		}
	}

	camera_save_to_file(0, IMG_DATA_TYPE_YUV420, g_cxt->capture_size.width, g_cxt->capture_size.height,
		&g_cxt->cap_mem[frm_id].target_yuv.addr_vir);
#endif

	CMR_PRINT_TIME;
	ret = camera_scaler_deinit();
	if (ret) {
		CMR_LOGV("Failed to deinit scaler, %d", ret);
		return ret;
	}

	if (0 == data->channel_id) {
		frm_id = frm.frame_id - CAMERA_CAP0_ID_BASE;
		ret = camera_take_picture_done(&frm);
		if (ret) {
			CMR_LOGE("Failed to set take_picture done %d", ret);
			return -CAMERA_FAILED;
		}
	} else {
		frm_id = frm.frame_id - CAMERA_CAP1_ID_BASE;
	}

	CMR_LOGV("channel id %d, frame id %d, height %d",
		frm.channel_id,
		frm_id,
		g_cxt->capture_size.height);

	return ret;
}

int camera_start_rotate(struct frm_info *data)
{
	uint32_t                 frm_id, rot_frm_id;
	struct img_rect          rect;
	int                      ret = CAMERA_SUCCESS;

	if (camera_capture_need_exit()) {
		ret = CAMERA_INVALID_STATE;
		CMR_LOGE("exit.");
		return ret;
	}
	if (IS_PREVIEW) {
		CMR_LOGE("Call Rotation in preview");
		if (IMG_CVT_ROTATING == g_cxt->rot_cxt.rot_state) {
			CMR_LOGE("Last rotate not finished yet, drop this frame");
			ret = cmr_v4l2_free_frame(0, data->frame_id);
			return ret;
		} else {
			frm_id = data->frame_id - CAMERA_PREV_ID_BASE;
			rot_frm_id  = g_cxt->prev_rot_index % CAMERA_PREV_ROT_FRM_CNT;
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
		g_cxt->cap_mem[frm_id].cap_yuv_rot.size.width = rect.width;
		g_cxt->cap_mem[frm_id].cap_yuv_rot.size.height = rect.height;
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

int camera_copy_data_virtual(uint32_t width, uint32_t height, uint32_t in_addr, uint32_t out_virtual_addr)
{

	struct img_frm           src_frame, dst_frame;
	uint32_t                 img_len = (uint32_t)(width * height);
	struct img_rect          rect;
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("(w,h)%d %d; addr src, 0x%x dst virtual, 0x%x",
		width, height, in_addr, out_virtual_addr);

#if 0
	if (!IS_PREVIEW) {
		CMR_LOGE("Preview Stoped");
		return ret;
	}
#endif

	src_frame.fmt = ROT_YUV420;
	src_frame.addr_phy.addr_y = in_addr;
	src_frame.addr_phy.addr_u = in_addr + (uint32_t)(width * height);
	src_frame.size.width      = width;
	src_frame.size.height     = height;

	dst_frame.addr_phy.addr_y = out_virtual_addr;
	dst_frame.addr_phy.addr_u = out_virtual_addr + (uint32_t)(width * height);
	dst_frame.size.width      = width;
	dst_frame.size.height     = height;

	return cmr_rot_cpy_to_virtual(&src_frame, &dst_frame);
}

int camera_copy_data(uint32_t width, uint32_t height, uint32_t in_addr, uint32_t out_addr)
{

	struct img_frm           src_frame, dst_frame;
	uint32_t                 img_len = (uint32_t)(width * height);
	struct img_rect          rect;
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("(w,h)%d %d; addr src, 0x%x dst, 0x%x",
		width, height, in_addr, out_addr);

	if (!IS_PREVIEW) {
		CMR_LOGE("Preview Stoped");
		return ret;
	}

#if 1
	src_frame.fmt = ROT_YUV420;
	src_frame.addr_phy.addr_y = in_addr;
	src_frame.addr_phy.addr_u = in_addr + (uint32_t)(width * height);
	src_frame.size.width      = width;
	src_frame.size.height     = height;

	dst_frame.addr_phy.addr_y = out_addr;
	dst_frame.addr_phy.addr_u = out_addr + (uint32_t)(width * height);
	dst_frame.size.width      = width;
	dst_frame.size.height     = height;

	return cmr_rot_cpy(&src_frame, &dst_frame);
#else
	pthread_mutex_lock(&g_cxt->prev_mutex);

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

	pthread_mutex_unlock(&g_cxt->prev_mutex);
	return ret;
#endif
}


int camera_get_data_redisplay(int output_addr,
				int output_width,
				int output_height,
				int input_addr_y,
				int input_addr_uv,
				int input_width,
				int input_height)
{
	struct img_frm           src_frame, dst_frame;
	uint32_t                 img_len = (uint32_t)(output_width * output_height);
	struct img_rect          rect;
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("(w,h)%d %d; addr src, 0x%x dst, (w,h)%d %d,0x%x",
		input_width, input_height, input_addr_y, output_width,output_height,output_addr);

	if (!IS_CAPTURE) {
		CMR_LOGE("Capture Stoped");
		return ret;
	}
	ret = camera_scaler_init();
	if (ret) {
		CMR_LOGE("Failed to init scaler %d", ret);
		ret = -CAMERA_NOT_SUPPORTED;
	} else {
		cmr_scale_evt_reg(NULL);
	}

	src_frame.addr_phy.addr_y = input_addr_y;
	src_frame.addr_phy.addr_u = input_addr_uv;
	src_frame.size.width      = input_width;
	src_frame.size.height     = input_height;
	src_frame.fmt             = IMG_DATA_TYPE_YUV420;
	src_frame.data_end.y_endian = 1;
	src_frame.data_end.uv_endian = 2;
	dst_frame.addr_phy.addr_y = output_addr;
	dst_frame.addr_phy.addr_u = output_addr + img_len;
	dst_frame.size.width      = output_width;
	dst_frame.size.height     = output_height;
	dst_frame.fmt             = IMG_DATA_TYPE_YUV420;
	dst_frame.data_end.y_endian = 1;
	dst_frame.data_end.uv_endian = 2;
	rect.start_x              = 0;
	rect.start_y              = 0;
	rect.width                = input_width;
	rect.height               = input_height;
	ret = cmr_scale_start(input_height,
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
	return 0;
}

struct camera_context *camera_get_cxt(void)
{
	return g_cxt;
}


static int camera_jpeg_encode_thumb(uint32_t *stream_size_ptr)
{
	struct img_frm           *src_frm;
	struct img_frm           *target_frm;
	struct jpeg_enc_in_param     in_parm;
	uint32_t                 sream_size;
	uint32_t                 frm_id = g_cxt->jpeg_cxt.index;
	int                      ret = CAMERA_SUCCESS;

	src_frm    = &g_cxt->cap_mem[frm_id].thum_yuv;
	target_frm = &g_cxt->cap_mem[frm_id].thum_jpeg;

	in_parm.src_addr_phy = src_frm->addr_phy;
	in_parm.src_addr_vir = src_frm->addr_vir;
	in_parm.stream_buf_phy = target_frm->addr_phy.addr_y;
	in_parm.stream_buf_vir = target_frm->addr_vir.addr_y;
	in_parm.size.width = g_cxt->thum_size.width;
	in_parm.size.height = g_cxt->thum_size.height;
	in_parm.slice_height = g_cxt->thum_size.height;
	in_parm.quality_level = g_cxt->jpeg_cxt.thumb_quality;
	in_parm.slice_mod = JPEG_YUV_SLICE_ONE_BUF;
	in_parm.size.width = g_cxt->thum_size.width;
	in_parm.size.height = g_cxt->thum_size.height;
	in_parm.stream_buf_size = target_frm->buf_size;
	ret = jpeg_enc_thumbnail(&in_parm, &sream_size);
	*stream_size_ptr = sream_size;
	CMR_LOGI("encode thumbnail return %d,stream size %d.",ret,sream_size);
	return ret;
}

static int camera_convert_to_thumb(void)
{
	struct img_frm           src_frame, dst_frame;
	uint32_t                 frm_id = g_cxt->jpeg_cxt.index;
	struct img_rect          rect;
	int                      ret = CAMERA_SUCCESS;

	ret = camera_scaler_init();
	if (ret) {
		CMR_LOGE("Failed to init scaler %d", ret);
		ret = -CAMERA_NOT_SUPPORTED;
	} else {
		cmr_scale_evt_reg(NULL);
	}

	src_frame.addr_phy.addr_y = g_cxt->cap_mem[frm_id].target_yuv.addr_phy.addr_y;
	src_frame.addr_phy.addr_u = g_cxt->cap_mem[frm_id].target_yuv.addr_phy.addr_u;
	src_frame.size.width      = g_cxt->picture_size.width;
	src_frame.size.height     = g_cxt->picture_size.height;
	src_frame.fmt             = IMG_DATA_TYPE_YUV420;
	src_frame.data_end.y_endian = 1;
	src_frame.data_end.uv_endian = 2;
	dst_frame.addr_phy.addr_y = g_cxt->cap_mem[frm_id].thum_yuv.addr_phy.addr_y;
	dst_frame.addr_phy.addr_u = g_cxt->cap_mem[frm_id].thum_yuv.addr_phy.addr_u;
	dst_frame.size.width      = g_cxt->thum_size.width;
	dst_frame.size.height     = g_cxt->thum_size.height;
	dst_frame.fmt             = IMG_DATA_TYPE_YUV420;
	dst_frame.data_end.y_endian = 1;
	dst_frame.data_end.uv_endian = 2;
	rect.start_x              = 0;
	rect.start_y              = 0;
	rect.width                = src_frame.size.width;
	rect.height               = src_frame.size.height;
	ret = cmr_scale_start(src_frame.size.height,
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

	CMR_PRINT_TIME;
	return ret;
}

uint32_t camera_get_rot_set(void)
{
    CMR_LOGI("rot set %d.",g_cxt->prev_rot);
    return g_cxt->prev_rot;
}

int camera_isp_skip_frame_handle(struct isp_skip_num *skip_number)
{
	int                      ret = CAMERA_SUCCESS;

	CMR_LOGV("skip_number, %d", skip_number->skip_num);

	return ret;

}

int camera_uv422_to_uv420(uint32_t dst, uint32_t src, uint32_t width, uint32_t height)
{
	int                      ret = CAMERA_SUCCESS;
	uint32_t                 i;

	CMR_LOGV("dst 0x%x, src 0x%x, w h %d %d", dst, src, width, height);
	for (i = 0; i < (height >> 1); i++) {
		memcpy((void*)dst, (void*)src, width);
		dst += width;
		src += (width << 1);
	}

	return ret;
}

int camera_isp_proc_handle(struct ips_out_param *isp_out)
{
	struct ipn_in_param        in_param;
	struct ips_out_param       out_param;
	struct process_status      *process = &g_cxt->isp_cxt.proc_status;
	uint32_t                   offset = 0;
	uint32_t                   frm_id = 0;
	uint32_t                   no_need = 0;
	struct jpeg_enc_next_param enc_nxt_param;
	int                        ret = CAMERA_SUCCESS;

	CMR_LOGV("total processed height %d", process->slice_height_out);

#if 0
	camera_save_to_file((uint32_t)(process->slice_height_out/CMR_SLICE_HEIGHT),
			IMG_DATA_TYPE_YUV422,
			g_cxt->cap_orig_size.width,
			CMR_SLICE_HEIGHT,
			&g_cxt->isp_cxt.cur_dst);

#endif

#if CMR_ISP_YUV422
	offset = (uint32_t)(process->slice_height_out * g_cxt->cap_mem[frm_id].cap_raw.size.width);
	offset = g_cxt->cap_mem[frm_id].cap_yuv.addr_vir.addr_u + (offset >> 1);
	camera_uv422_to_uv420(offset,
				g_cxt->cap_mem[frm_id].isp_tmp.addr_vir.addr_y,
				g_cxt->cap_mem[frm_id].cap_raw.size.width,
				isp_out->output_height);
#endif
	process->slice_height_out += isp_out->output_height;

	if (g_cxt->isp_cxt.is_first_slice) {
		ret = camera_capture_yuv_process(&process->frame_info);
		g_cxt->isp_cxt.is_first_slice = 0;
	} else {
		if (IMG_ROT_0 == g_cxt->cap_rot) {
			if (NO_SCALING) {
				bzero(&enc_nxt_param, sizeof(struct jpeg_enc_next_param));
				enc_nxt_param.handle       = g_cxt->jpeg_cxt.handle;
				enc_nxt_param.slice_height = g_cxt->jpeg_cxt.proc_status.slice_height_in;
				enc_nxt_param.ready_line_num = process->slice_height_out;
				CMR_LOGV("Jpeg need more slice, %d %d",
					enc_nxt_param.slice_height,
					enc_nxt_param.ready_line_num);
				ret = jpeg_enc_next(&enc_nxt_param);
				if (ret) {
					CMR_LOGE("Failed to next jpeg encode %d", ret);
					return -CAMERA_FAILED;
				}
			} else {
				if (g_cxt->scaler_cxt.scale_state != IMG_CVT_SCALING) {
					ret = camera_start_scale(&process->frame_info);
				} else {
					ret = camera_scale_next(&process->frame_info);
					if (CVT_RET_LAST == ret) {
						CMR_LOGE("No need to process next slice");
						return 0;
					}
				}
			}
		}
	}

	if (process->slice_height_out == g_cxt->cap_orig_size.height) {
		if (NO_SCALING) {
			return camera_take_picture_done(&process->frame_info);
		}
	} else if (process->slice_height_out + process->slice_height_in <
		g_cxt->cap_orig_size.height) {
		in_param.src_slice_height = process->slice_height_in;
	} else {
		in_param.src_slice_height = g_cxt->cap_orig_size.height - process->slice_height_out;
		CMR_LOGV("last slice, %d", in_param.src_slice_height);
	}

	frm_id = process->frame_info.frame_id - CAMERA_CAP0_ID_BASE;
	offset = (uint32_t)(process->slice_height_out * g_cxt->cap_mem[frm_id].cap_raw.size.width);
	in_param.dst_addr_phy.chn0 = g_cxt->cap_mem[frm_id].cap_yuv.addr_phy.addr_y + offset;
	g_cxt->isp_cxt.cur_dst.addr_y = g_cxt->cap_mem[frm_id].cap_yuv.addr_vir.addr_y + offset;

#if CMR_ISP_YUV422
	in_param.dst_addr_phy.chn1 = g_cxt->cap_mem[frm_id].isp_tmp.addr_phy.addr_y;
#else
	in_param.dst_addr_phy.chn1 = g_cxt->cap_mem[frm_id].cap_yuv.addr_phy.addr_u + (offset >> 1);
	g_cxt->isp_cxt.cur_dst.addr_u = g_cxt->cap_mem[frm_id].cap_yuv.addr_vir.addr_u + (offset >> 1);
#endif
	offset = (uint32_t)((offset * RAWRGB_BIT_WIDTH) >> 3);
	in_param.src_addr_phy.chn0 = g_cxt->cap_mem[frm_id].cap_raw.addr_phy.addr_y + offset;
	in_param.src_avail_height = g_cxt->cap_mem[frm_id].cap_raw.size.height;

	CMR_LOGV("next, src 0x%x, dst 0x%x 0x%x",
		in_param.src_addr_phy.chn0,
		in_param.dst_addr_phy.chn0,
		in_param.dst_addr_phy.chn1);

	ret = isp_proc_next(&in_param, &out_param);

	return ret;

}
void camera_set_start_facedetect(uint32_t param)
{
	g_cxt->arithmetic_cxt.fd_flag = param;
    CMR_LOGV("param %d.",param);
}

int camera_set_fd_mem(uint32_t phy_addr, uint32_t vir_addr, uint32_t mem_size)
{
	CMR_LOGV("phy_addr, 0x%x, vir_addr, 0x%x, mem_size 0x%x", phy_addr, vir_addr, mem_size);
	arithmetic_set_mem(phy_addr,vir_addr,mem_size);

	return 0;
}

int camera_set_change_size(uint32_t cap_width,uint32_t cap_height,uint32_t preview_width,uint32_t preview_height)
{
	int ret = 0;

	CMR_LOGI("start.");
	if (CMR_PREVIEW == g_cxt->camera_status) {
		if ((g_cxt->display_size.width != preview_width)
			|| (g_cxt->display_size.height != preview_height)) {
			CMR_LOGI("need to change size.");
			ret = 1;
		}
	}
	CMR_LOGI("done,%d.",ret);
	return ret;
}

int camera_cancel_capture(void)
{
	int        ret = CAMERA_SUCCESS;

	pthread_mutex_lock(&g_cxt->cancel_mutex);
	g_cxt->cap_canceled = 1;
	pthread_mutex_unlock(&g_cxt->cancel_mutex);

	CMR_LOGV("cap_canceled %d", g_cxt->cap_canceled);
	return ret;
}

int camera_capture_need_exit(void)
{
	int        ret = 0;

	pthread_mutex_lock(&g_cxt->cancel_mutex);
	ret = g_cxt->cap_canceled == 1 ? 1 : 0;
	pthread_mutex_unlock(&g_cxt->cancel_mutex);

	return ret;
}

int camera_get_preview_rect(int *rect_x, int *rect_y, int *rect_width, int *rect_height)
{
	int ret = 0;
	SENSOR_MODE_INFO_T       *sensor_mode;

	sensor_mode = &g_cxt->sn_cxt.sensor_info->sensor_mode_info[g_cxt->sn_cxt.preview_mode];

	if((g_cxt->preview_rect.width > sensor_mode->width) || (g_cxt->preview_rect.height > sensor_mode->height)){
		CMR_LOGE("camera_get_preview_rect error: preview rect: %d, %d is greate than sensor output: %d %d \n",
			g_cxt->preview_rect.width, g_cxt->preview_rect.height, sensor_mode->width, sensor_mode->height);

		return -1;
	}

	*rect_x 	 = g_cxt->preview_rect.start_x;
	*rect_y 	 = g_cxt->preview_rect.start_y;
	*rect_width	 = g_cxt->preview_rect.width;
	*rect_height = g_cxt->preview_rect.height;

	CMR_LOGE("camera_get_preview_rect: x=%d, y=%d, w=%d, h=%d \n", *rect_x, *rect_y, *rect_width, *rect_height);

	return ret;
}
