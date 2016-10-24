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
#ifndef _CMR_OEM_H_
#define _CMR_OEM_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "cmr_common.h"
#include "cmr_mem.h"
#include "cmr_msg.h"
#include "cmr_set.h"
#include "cmr_v4l2.h"
#include "sensor_drv_u.h"
//#include "isp_app.h"
#include "cmr_cvt.h"
#include "jpeg_codec.h"

#define CAMERA_OEM_MSG_QUEUE_SIZE                    50
#define CAMERA_OEM_PREV_FRM_ID_BASE                  0x1000
#define CAMERA_OEM_CAP0_FRM_ID_BASE                  0x2000
#define CAMERA_OEM_CAP1_FRM_ID_BASE                  0x3000
#define CAMERA_OEM_PREV_FRM_CNT                      V4L2_BUF_MAX
#define CAMERA_OEM_PREV_ROT_FRM_CNT                  2
#define CAMERA_OEM_CAP_FRM_CNT                       2
#define CAMERA_ZOOM_LEVEL_MAX                        8
#define ZOOM_STEP(x)                                 ((x - x / 4) / CAMERA_ZOOM_LEVEL_MAX)
#define CAMERA_PIXEL_ALIGNED                         4 
#define CAMERA_WIDTH(w)                              ((w)& ~(CAMERA_PIXEL_ALIGNED - 1))
#define CAMERA_HEIGHT(h)                             ((h)& ~(CAMERA_PIXEL_ALIGNED - 1))
#define CAMERA_FOCUS_RECT_PARAM_LEN                  200

enum zoom_mode {
	ZOOM_BY_CAP = 0,
	ZOOM_POST_PROCESS				
};

enum thum_img_from {
	THUM_FROM_CAP = 0, // same with the large size image
	THUM_FROM_SCALER, // convert large size image to thumbnail image after capture
};

enum {
	CMR_IDLE = 0x00,
	CMR_PREVIEW,
	CMR_CAPTURE,
	CMR_CAPTURE_SLICE,
	CMR_ERR,
	
	V4L2_IDLE = 0x10,
	V4L2_PREVIEW,
	V4L2_CAPTURE,
	V4L2_ERR,

	SENSOR_IDLE = 0x20,
	SENSOR_STREAMON,
	SENSOR_ERR,

	ISP_IDLE = 0x30,
	ISP_COWORK,
	ISP_POST_PROCESS,
	ISP_ERR,

	JPEG_IDLE = 0x40,
	JPEG_ENCODE,
	JPEG_DECODE,
	JPEG_ERR,

	IMG_CVT_IDLE = 0x50,
	IMG_CVT_ROTATING,
	IMG_CVT_ROT_DONE,
	IMG_CVT_SCALING
};

	
struct process_status {
	uint32_t                 slice_height_in;
	uint32_t                 slice_height_out;
	struct frm_info          frame_info;
};

struct camera_ctrl {
	uint32_t                 v4l2_inited;
	uint32_t                 sensor_inited;
	uint32_t                 isp_inited;
	uint32_t                 jpeg_inited;
	uint32_t                 scaler_inited;
	uint32_t                 rot_inited;
};


struct v4l2_context {
	uint32_t                 v4l2_state; // 0 for preview, 1 for snapshot;
	struct process_status    proc_status;
	uint32_t                 sc_capability;
};

struct sensor_context {
	uint32_t                 cur_id;
	SENSOR_EXP_INFO_T        *sensor_info;
	void                     *raw_settings;
	struct  sensor_if        sn_if;
	uint32_t                 setting_length;
	uint32_t                 preview_mode;
	uint32_t                 capture_mode;
};


struct isp_context {
	uint32_t                 isp_state; // 0 for preview, 1 for post process;
	uint32_t                 width_limit;
	struct process_status    proc_status;
};

struct jpeg_context {
	uint32_t                 jpeg_state;
	struct process_status    proc_status;
	uint32_t                 quality;
	uint32_t                 index;
        uint32_t                 handle;
};

struct scaler_context {
	uint32_t                 scale_state;
	struct process_status    proc_status;
	uint32_t                 sc_capability;
	uint32_t                 out_fmt;
};

struct rotation_context {
	uint32_t                 rot_state;
	struct process_status    proc_status;
	sem_t                    cmr_rot_sem;
};

struct camera_settings {
	uint32_t                 focal_len;
	uint32_t                 brightness;
	uint32_t                 contrast;
	uint32_t                 effect;
	uint32_t                 expo_compen;
	uint32_t                 wb_mode;
	uint32_t                 scene_mode;
	uint32_t                 flash;
	uint32_t                 night_mode;
	uint32_t                 flicker_mode;
	uint32_t                 focus_rect;
	uint32_t                 af_mode;
	uint32_t                 iso;
	uint32_t                 luma_adapt;
	uint32_t                 video_mode;
	uint32_t                 af_inited;
	uint32_t                 af_cancelled;
	uint8_t                  focus_zone_param[CAMERA_FOCUS_RECT_PARAM_LEN];
	pthread_mutex_t          set_mutex;
};

struct camera_context {
	/*for the device OEM layer owned*/
	struct camera_ctrl       control;
	struct v4l2_context      v4l2_cxt;
	struct sensor_context    sn_cxt;
	struct isp_context       isp_cxt;
	struct jpeg_context      jpeg_cxt;
	struct scaler_context    scaler_cxt;
	struct rotation_context  rot_cxt;

	/*for the workflow management*/
	pthread_t                camera_main_thr;
	uint32_t                 msg_queue_handle;
	volatile uint32_t        is_working;
	uint32_t                 camera_status;
	pthread_mutex_t          cb_mutex;
	camera_cb_f_type         camera_cb;
	pthread_mutex_t          data_mutex;
	void*                    client_data;
	uint32_t                 zoom_level;
	uint32_t                 skip_mode;
	uint32_t                 skip_num;
	uint32_t                 pre_frm_cnt;
	
	/*for preview*/
	struct img_size          display_size;
	struct img_size          preview_size;
	uint32_t                 preview_fmt;
	uint32_t                 prev_rot;
	uint32_t                 prev_rot_index;
	struct img_frm           prev_frm[CAMERA_OEM_PREV_FRM_CNT];
	struct img_frm           prev_rot_frm[CAMERA_OEM_PREV_ROT_FRM_CNT];
	uint32_t                 prev_phys_addr;
	uint32_t                 *prev_virt_addr;
	uint32_t                 prev_mem_szie;
	uint32_t                 is_waiting;
	sem_t                    prev_sem;


	/*for capture*/
	struct img_size          picture_size;
	struct img_size          capture_size;
	struct img_size          cap_orig_size;
	struct img_size          max_size;
	uint32_t                 total_cap_num;
	uint32_t                 cap_cnt;
	uint32_t                 cap_process_id;
	uint32_t                 total_cap_ch_num;
	uint32_t                 cap_ch_cnt;
	uint32_t                 cap_rot;
	uint32_t                 cap_rot_index;
	uint32_t                 cap_original_fmt;
	uint32_t                 cap_target_fmt;
	uint32_t                 cap_zoom_mode;
	uint32_t                 thum_from;
	uint32_t                 thum_ready;
	struct img_size          thum_size;
	struct cmr_cap_mem       cap_mem[CAMERA_OEM_CAP_FRM_CNT];
	sem_t                    picture_sem;


	/*for setting*/
	struct camera_settings   cmr_set;
	
};

uint32_t camera_get_rot_angle(uint32_t degree);
uint32_t camera_get_img_type(uint32_t format_mode);
int camera_get_trim_rect(struct img_rect *src_trim_rect, uint32_t zoom_level, struct img_size *dst_size);
struct camera_context *camera_get_cxt(void);


#ifdef __cplusplus
}
#endif

#endif
