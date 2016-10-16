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
#include "isp_app.h"
#include "cmr_cvt.h"
#include "jpeg_codec.h"
#include "jpeg_exif_header.h"
#include "cmr_arith.h"


#define CMR_EVT_INIT                                (CMR_EVT_OEM_BASE)
#define CMR_EVT_START                               (CMR_EVT_OEM_BASE + 1)
#define CMR_EVT_STOP                                (CMR_EVT_OEM_BASE + 2)
#define CMR_EVT_EXIT                                (CMR_EVT_OEM_BASE + 3)
#define CMR_EVT_BEFORE_SET                          (CMR_EVT_OEM_BASE + 4)
#define CMR_EVT_AFTER_SET                           (CMR_EVT_OEM_BASE + 5)
#define CMR_EVT_AF_START                            (CMR_EVT_OEM_BASE + 10)
#define CMR_EVT_AF_EXIT                             (CMR_EVT_OEM_BASE + 11)
#define CMR_EVT_AF_INIT                             (CMR_EVT_OEM_BASE + 12)
#define CAMERA_OEM_MSG_QUEUE_SIZE                    50
#define CAMERA_AF_MSG_QUEUE_SIZE                     5
#define CAMERA_PREV_ID_BASE                          0x1000
#define CAMERA_CAP0_ID_BASE                          0x2000
#define CAMERA_CAP1_ID_BASE                          0x4000
#define CAMERA_PREV_FRM_CNT                          V4L2_BUF_MAX
#define CAMERA_PREV_ROT_FRM_CNT                      4
#define CAMERA_CAP_FRM_CNT                           CMR_IMG_CNT_MAX
#define CAMERA_ZOOM_LEVEL_MAX                        8
#define ZOOM_STEP(x)                                 ((x - x / CMR_ZOOM_FACTOR) / CAMERA_ZOOM_LEVEL_MAX)
#define CAMERA_PIXEL_ALIGNED                         4
#define CAMERA_WIDTH(w)                              ((w)& ~(CAMERA_PIXEL_ALIGNED - 1))
#define CAMERA_HEIGHT(h)                             ((h)& ~(CAMERA_PIXEL_ALIGNED - 1))
#define CAMERA_FOCUS_RECT_PARAM_LEN                  200
#define CAMERA_RECOVER_CNT                           3

enum zoom_mode {
	ZOOM_BY_CAP = 0,
	ZOOM_POST_PROCESS
};

enum RECOVER_STATUS {
	NO_RECOVERY = 0,
	RECOVERING,
	RESTART
};

enum thum_img_from {
	THUM_FROM_CAP = 0,// same with the large size image
	THUM_FROM_SCALER,// convert large size image to thumbnail image after capture
};

enum {
	CMR_IDLE = 0x00,
	CMR_PREVIEW,
	CMR_PREVIEW_RECOVER,
	CMR_CAPTURE,
	CMR_CAPTURE_SLICE,
	CMR_CAPTURE_RECOVER,
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
	uint32_t                 is_encoding;
	struct frm_info          frame_info;
};

struct camera_ctrl {
	uint32_t                 v4l2_inited;
	uint32_t                 sensor_inited;
	uint32_t                 isp_inited;
	uint32_t                 jpeg_inited;
	uint32_t                 scaler_inited;
	uint32_t                 rot_inited;
	uint32_t                 arithmetic_inited;
};


struct v4l2_context {
	uint32_t                 v4l2_state; // 0 for preview, 1 for snapshot;
	struct process_status    proc_status;
	uint32_t                 sc_capability;
	uint32_t                 sc_factor;
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
	uint32_t                 is_first_slice;
	uint32_t                 drop_slice_num;
	uint32_t                 drop_slice_cnt;
	struct img_addr          cur_dst;
	struct process_status    proc_status;
};

struct jpeg_context {
	uint32_t                 jpeg_state;
	struct process_status    proc_status;
	uint32_t                 quality;
	uint32_t                 thumb_quality;
	uint32_t                 index;
	uint32_t                 handle;
	uint32_t                 set_encode_rotation;
};

struct scaler_context {
	uint32_t                 scale_state;
	struct process_status    proc_status;
	uint32_t                 sc_capability;
	uint32_t                 sc_factor;
	uint32_t                 out_fmt;
};

struct rotation_context {
	uint32_t                 rot_state;
	struct process_status    proc_status;
	sem_t                    cmr_rot_sem;
};

struct arithmetic_context {
	uint32_t                 fd_inited;
	uint32_t                 fd_flag;
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
	/*all the above value will be set as 0xFFFFFFFF after inited*/
	uint32_t                 set_end;

	uint32_t                 af_cancelled;
	uint8_t                  focus_zone_param[CAMERA_FOCUS_RECT_PARAM_LEN];
	pthread_mutex_t          set_mutex;
	sem_t                    isp_af_sem;
	uint32_t                 isp_af_win_val;
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
	struct arithmetic_context arithmetic_cxt;

	/*for the workflow management*/
	pthread_t                camera_main_thr;
	pthread_t                af_thread;
	uint32_t                 msg_queue_handle;
	uint32_t                 af_msg_que_handle;
	volatile uint32_t        is_working;
	uint32_t                 camera_status;
	uint32_t                 recover_status;
	uint32_t                 recover_cnt;
	pthread_mutex_t          cb_mutex;
	camera_cb_f_type         camera_cb;
	pthread_mutex_t          data_mutex;
	void*                    client_data;
	uint32_t                 af_inited;
	pthread_mutex_t          af_cb_mutex;
	camera_cb_f_type         camera_af_cb;
	uint32_t                 zoom_level;
	uint32_t                 skip_mode;
	uint32_t                 skip_num;
	uint32_t                 pre_frm_cnt;
	pthread_mutex_t          prev_mutex;
	sem_t                    af_sync_sem;
	sem_t                    init_sem;
	sem_t                    exit_sem;
	sem_t                    start_sem;
	sem_t                    stop_sem;
	uint32_t                 err_code;

	/*for preview*/
	struct img_size          display_size;
	struct img_size          preview_size;
	struct img_rect          preview_rect;
	uint32_t                 preview_fmt;
	uint32_t                 prev_rot;
	uint32_t                 prev_rot_index;
	struct img_frm           prev_frm[CAMERA_PREV_FRM_CNT];
	struct img_frm           prev_rot_frm[CAMERA_PREV_ROT_FRM_CNT];
	uint32_t                 prev_phys_addr;
	uint32_t                 *prev_virt_addr;
	uint32_t                 prev_mem_szie;
	uint32_t                 wait_for_start;
	uint32_t                 wait_for_stop;

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
	struct cmr_cap_mem       cap_mem[CAMERA_CAP_FRM_CNT];
	struct cmr_cap_2_frm     cap_2_mems;
	pthread_mutex_t          cancel_mutex;
	uint32_t                 cap_canceled;
	takepicture_mode         cap_mode;
	/*for setting*/
	struct camera_settings   cmr_set;
	uint32_t                 orientation;
};

uint32_t camera_get_rot_angle(uint32_t degree);
uint32_t camera_get_img_type(uint32_t format_mode);
int camera_get_trim_rect(struct img_rect *src_trim_rect, uint32_t zoom_level, struct img_size *dst_size);
struct camera_context *camera_get_cxt(void);
int camera_set_pos_type(camera_position_type *position);
JINF_EXIF_INFO_T* camera_get_exif(struct camera_context *p_cxt);
int camera_sync_var_init(struct camera_context *p_cxt);
int camera_sync_var_deinit(struct camera_context *p_cxt);
int camera_wait_start(struct camera_context *p_cxt);
int camera_start_done(struct camera_context *p_cxt);
int camera_wait_stop(struct camera_context *p_cxt);
int camera_stop_done(struct camera_context *p_cxt);
int camera_wait_init(struct camera_context *p_cxt);
int camera_init_done(struct camera_context *p_cxt);
int camera_wait_exit(struct camera_context *p_cxt);
int camera_exit_done(struct camera_context *p_cxt);
void camera_sensor_inf(struct sensor_if *cam_inf_ptr, SENSOR_INF_T *inf_ptr);
int camera_set_sensormark(void);
int camera_save_sensormark(void);

int camera_save_to_file(uint32_t index, uint32_t img_fmt,
	uint32_t width, uint32_t height, struct img_addr *addr);

#ifdef __cplusplus
}
#endif

#endif
