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


#define CMR_EVT_INIT                                 (CMR_EVT_OEM_BASE)
#define CMR_EVT_START                                (CMR_EVT_OEM_BASE + 1)
#define CMR_EVT_STOP                                 (CMR_EVT_OEM_BASE + 2)
#define CMR_EVT_EXIT                                 (CMR_EVT_OEM_BASE + 3)
#define CMR_EVT_BEFORE_SET                           (CMR_EVT_OEM_BASE + 4)
#define CMR_EVT_AFTER_SET                            (CMR_EVT_OEM_BASE + 5)
#define CMR_EVT_BEFORE_CAPTURE                       (CMR_EVT_OEM_BASE + 6)
#define CMR_EVT_AFTER_CAPTURE                        (CMR_EVT_OEM_BASE + 7)
#define CMR_EVT_CONVERT_THUM                         (CMR_EVT_OEM_BASE + 8)
#define CMR_EVT_AF_START                             (CMR_EVT_OEM_BASE + 10)
#define CMR_EVT_AF_EXIT                              (CMR_EVT_OEM_BASE + 11)
#define CMR_EVT_AF_INIT                              (CMR_EVT_OEM_BASE + 12)
#define CMR_EVT_CAF_MOVE_START                       (CMR_EVT_OEM_BASE + 13)
#define CMR_EVT_CAF_MOVE_STOP                        (CMR_EVT_OEM_BASE + 14)

#define CMR_EVT_PREV_BASE                            (CMR_EVT_OEM_BASE + 0x100)
#define CMR_EVT_PREV_INIT                            (CMR_EVT_OEM_BASE + 0x0)
#define CMR_EVT_PREV_EXIT                            (CMR_EVT_OEM_BASE + 0xF)
#define CMR_EVT_PREV_V4L2_BASE                       (CMR_EVT_PREV_BASE + 0x10)
#define CMR_EVT_PREV_V4L2_TX_DONE                    (CMR_EVT_PREV_V4L2_BASE + CMR_V4L2_TX_DONE - CMR_EVT_V4L2_BASE)
#define CMR_EVT_PREV_V4L2_TX_NO_MEM                  (CMR_EVT_PREV_V4L2_BASE +  CMR_V4L2_TX_NO_MEM - CMR_EVT_V4L2_BASE)
#define CMR_EVT_PREV_V4L2_TX_ERR                     (CMR_EVT_PREV_V4L2_BASE + CMR_V4L2_TX_ERROR - CMR_EVT_V4L2_BASE)
#define CMR_EVT_PREV_V4L2_CSI2_ERR                   (CMR_EVT_PREV_V4L2_BASE + CMR_V4L2_CSI2_ERR - CMR_EVT_V4L2_BASE)
#define CMR_EVT_PREV_CVT_BASE                        (CMR_EVT_PREV_BASE + 0x20)
#define CMR_EVT_PREV_CVT_ROT_DONE                    (CMR_EVT_PREV_CVT_BASE + CMR_IMG_CVT_ROT_DONE - CMR_EVT_CVT_BASE)
#define CMR_EVT_PREV_STOP                            (CMR_EVT_PREV_BASE + 0x30)


#define CMR_EVT_CAP_BASE                             (CMR_EVT_OEM_BASE + 0x200)
#define CMR_EVT_CAP_INIT                             (CMR_EVT_CAP_BASE + 0)
#define CMR_EVT_CAP_EXIT                             (CMR_EVT_CAP_BASE + 1)
#define CMR_EVT_CAP_TX_DONE                          (CMR_EVT_CAP_BASE + 2)
#define CMR_EVT_CAP_START_CAP                        (CMR_EVT_CAP_BASE + 3)
#define CMR_EVT_CAP_RAW_TX_DONE                      (CMR_EVT_CAP_BASE + 4)
#define CMR_EVT_CAP_FRAME_DONE                       (CMR_EVT_CAP_BASE + 5)
#define CMR_EVT_CAP_COMPLETE_DONE                    (CMR_EVT_CAP_BASE + 6)

#define CMR_EVT_CB_BASE                              (CMR_EVT_OEM_BASE + 0x800)
#define CMR_EVT_CB_INIT                              (CMR_EVT_CB_BASE + 0)
#define CMR_EVT_CB_EXIT                              (CMR_EVT_CB_BASE + 1)
#define CMR_EVT_CB_HANDLE                            (CMR_EVT_CB_BASE + 2)

#define CMR_EVT_SW_MON_BASE                          (CMR_EVT_OEM_BASE + 0x900)
#define CMR_EVT_SW_MON_INIT                          (CMR_EVT_SW_MON_BASE + 0)
#define CMR_EVT_SW_MON_EXIT                          (CMR_EVT_SW_MON_BASE + 1)
#define CMR_EVT_SW_MON_SET_PARA                      (CMR_EVT_SW_MON_BASE + 2)

#define CAMERA_OEM_MSG_QUEUE_SIZE                    50
#define CAMERA_AF_MSG_QUEUE_SIZE                     5
#define CAMERA_PREV_MSG_QUEUE_SIZE                   20
#define CAMERA_CAP_MSG_QUEUE_SIZE                    20
#define CAMERA_CB_MSG_QUEUE_SIZE                     40
#define CAMERA_PREV_ID_BASE                          0x1000
#define CAMERA_CAP0_ID_BASE                          0x2000
#define CAMERA_CAP1_ID_BASE                          0x4000
#define CAMERA_PREV_FRM_CNT                          V4L2_BUF_MAX
#define CAMERA_PREV_ROT_FRM_CNT                      4
#define CAMERA_CAP_FRM_CNT                           4 //CMR_IMG_CNT_MAX
#define CAMERA_NORMAL_CAP_FRM_CNT                    1
#define CAMERA_ZOOM_LEVEL_MAX                        8
#define ZOOM_STEP(x)                                 (((x) - (x) / CMR_ZOOM_FACTOR) / CAMERA_ZOOM_LEVEL_MAX)
#define CAMERA_PIXEL_ALIGNED                         4
#define CAMERA_WIDTH(w)                              ((w)& ~(CAMERA_PIXEL_ALIGNED - 1))
#define CAMERA_HEIGHT(h)                             ((h)& ~(CAMERA_PIXEL_ALIGNED - 1))
#define CAMERA_ALIGNED_16(w)                         ((((w) + 16 -1) >> 4) << 4)
#define CAMERA_SAFE_SCALE_DOWN(w)                    (uint32_t)((w)*11/10)
#define CAMERA_FOCUS_RECT_PARAM_LEN                  200
#define CAMERA_RECOVER_CNT                           3

enum zoom_mode {
	ZOOM_BY_CAP = 0,
	ZOOM_POST_PROCESS
};

enum {
	TAKE_PICTURE_NO = 0,
	TAKE_PICTURE_NEEDED
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
	CMR_CAPTURE_PREPARE,
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
	IMG_CVT_SCALING,

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
	uint32_t                 dma_copy_inited;
};


struct v4l2_context {
	uint32_t                 v4l2_state; // 0 for preview, 1 for snapshot;
	struct process_status    proc_status;
	uint32_t                 sc_capability;
	uint32_t                 sc_factor;
	uint32_t                 chn_status[CHN_MAX];
	uint32_t                 chn_frm_deci[CHN_MAX];
	uint32_t                 waiting_cap_frame;
};

struct sensor_context {
	uint32_t                 cur_id;
	SENSOR_EXP_INFO_T        *sensor_info;
	void                     *raw_settings;
	struct  sensor_if        sn_if;
	uint32_t                 setting_length;
	uint32_t                 preview_mode;
	uint32_t                 capture_mode;
	uint32_t                 previous_sensor_mode;
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
	int                      fd;
	uint32_t                 rot_state;
	struct process_status    proc_status;
	sem_t                    cmr_rot_sem;
	struct img_frm           frm_data;
};

struct arithmetic_context {
	uint32_t                 fd_inited;
	uint32_t                 fd_flag;
	uint32_t                 fd_num;
};

struct jpeg_specify_context {
	uint32_t                jpeg_specify_state;
	pthread_mutex_t         jpeg_specify_mutex;
	sem_t                   jpeg_specify_cap_sem;
};

struct camera_settings {
	uint32_t                 focal_len;
	uint32_t                 brightness;
	uint32_t                 contrast;
	uint32_t                 effect;
	uint32_t                 expo_compen;
	uint32_t                 wb_mode;
	uint32_t                 saturation;
	uint32_t                 sharpness;
	uint32_t                 scene_mode;
	uint32_t                 flash;
	uint32_t                 flash_mode;
	uint32_t                 auto_flash_status;
	uint32_t                 night_mode;
	uint32_t                 flicker_mode;
	uint32_t                 focus_rect;
	uint32_t                 af_mode;
	uint32_t                 iso;
	uint32_t                 luma_adapt;
	uint32_t                 video_mode;
	uint32_t                 frame_rate;
	uint32_t                 sensor_mode;
	uint32_t                 auto_exposure_mode;
	uint32_t                 preview_env;
	/*all the above value will be set as 0xFFFFFFFF after inited*/
	uint32_t                 set_end;

	uint32_t                 af_cancelled;
	uint32_t                 caf_move_done;
	uint8_t                  focus_zone_param[CAMERA_FOCUS_RECT_PARAM_LEN];
	pthread_mutex_t          set_mutex;
	sem_t                    isp_af_sem;
	uint32_t                 isp_af_win_val;
	uint32_t                 auto_flash;
	uint8_t                  bflash;
	uint32_t                 slow_motion_mode;
	uint32_t                 isp_alg_timeout;
	uint32_t                 isp_af_timeout;
	sem_t                    isp_alg_sem;
	pthread_mutex_t          isp_alg_mutex;
	uint32_t                 isp_ae_stab_timeout;
	sem_t                    isp_ae_stab_sem;
	pthread_mutex_t          isp_ae_stab_mutex;
	pthread_mutex_t          isp_af_mutex;
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
	struct jpeg_specify_context jpeg_specify_cxt;

	/*for the workflow management*/
	pthread_t                camera_main_thr;
	pthread_t                af_thread;
	uint32_t                 msg_queue_handle;
	uint32_t                 af_msg_que_handle;
	volatile uint32_t        is_working;
	uint32_t                 preview_status;
	uint32_t                 capture_status;
	uint32_t                 capture_raw_status;
	uint32_t                 is_take_picture;
	uint32_t                 is_support_fd;
	pthread_mutex_t          take_mutex;
	pthread_mutex_t          take_raw_mutex;
	pthread_mutex_t          main_prev_mutex;
	uint32_t                 chn_0_status;
	uint32_t                 chn_1_status;
	uint32_t                 chn_2_status;
	uint32_t                 recover_status;
	uint32_t                 recover_cnt;
	pthread_mutex_t          recover_mutex;

	pthread_mutex_t          prev_cb_mutex;
	pthread_mutex_t          cap_cb_mutex;
	camera_cb_f_type         camera_cb;
	pthread_mutex_t          data_mutex;
	void*                    client_data;
	uint32_t                 af_inited;
	pthread_mutex_t          af_cb_mutex;
	uint32_t                 af_busy;

	camera_cb_f_type         camera_af_cb;
	uint32_t                 zoom_level;
	uint32_t                 zoom_mode;
	struct img_rect          zoom_rect;
	uint32_t                 skip_mode;
	uint32_t                 skip_num;
	uint32_t                 pre_frm_cnt;
	int64_t                  restart_timestamp;
	uint32_t                 restart_skip_cnt;
	uint32_t                 restart_skip_en;
	pthread_mutex_t          prev_mutex;
	sem_t                    af_sync_sem;
	sem_t                    init_sem;
	sem_t                    exit_sem;
	sem_t                    start_sem;
	sem_t                    stop_sem;
	sem_t                    takepicdone_sem;
	sem_t                    takepic_callback_sem;
	sem_t                    sync_scale_sem;
	sem_t                    sync_rotate_sem;
	uint32_t                 err_code;
	uint32_t                 camera_id;
	pthread_t                prev_thread;
	uint32_t                 prev_msg_que_handle;
	uint32_t                 prev_inited;
	sem_t                    prev_sync_sem;
	/* capture thread */
	pthread_t                cap_thread;
	pthread_t                cap_sub_thread;
	uint32_t                 cap_msg_que_handle;
	uint32_t                 cap_sub_msg_que_handle;
	uint32_t                 cap_inited;
	uint32_t                 cap_sub_inited;
	sem_t                    cap_sync_sem;
	sem_t                    cap_sub_sync_sem;

	int32_t                  set_flag;
	sem_t                    set_sem;


	pthread_t                cap_sub2_thread;
	uint32_t                 cap_sub2_inited;
	uint32_t                 cap_sub2_msg_queue_handle;
	sem_t                    cap_sub2_sync_sem;
	uint32_t                 is_cap_trace;

	/*for preview*/
	struct img_size          display_size;
	struct img_size          preview_size;
	struct img_rect          preview_rect;
	uint32_t                 preview_fmt;
	uint32_t                 prev_rot;
	uint32_t                 prev_rot_index;
	uint32_t                 prev_rot_frm_is_lock[CAMERA_PREV_ROT_FRM_CNT];
	struct img_frm           prev_frm[CAMERA_PREV_FRM_CNT];
	struct img_frm           prev_rot_frm[CAMERA_PREV_ROT_FRM_CNT];

	uint32_t*                 prev_phys_addr_array;
	uint32_t*                 prev_virt_addr_array;
	uint32_t                 prev_mem_size;
	uint32_t                 prev_mem_num;

	uint32_t                 wait_for_start;
	uint32_t                 wait_for_stop;
	uint32_t                 is_dv_mode;
	uint32_t                 prev_buf_id;
	uint32_t                 prev_self_restart;

	/*for capture*/
	struct img_size          picture_size;
	struct img_size          actual_picture_size;
	struct img_size          actual_picture_size_backup;
	struct img_size          picture_size_backup;
	struct img_size          capture_size;
	struct img_size          cap_orig_size;
	struct img_size          cap_orig_size_backup;
	struct img_size          max_size;
	uint32_t                 total_cap_num;
	uint32_t                 total_capture_num;
	uint32_t                 cap_cnt;
	int64_t                  cap_time_stamp;
	uint32_t                 last_cap_cnt;
	uint32_t                 cap_cnt_for_err;
	uint32_t                 cap_process_id;
	uint32_t                 total_cap_ch_num;
	uint32_t                 cap_ch_cnt;
	uint32_t                 cap_rot;
	uint32_t                 cap_rot_backup;
	uint32_t                 cap_rot_index;
	uint32_t                 cap_original_fmt;
	uint32_t                 cap_target_fmt;
	uint32_t                 cap_zoom_mode;
	sem_t                    cap_path_sem;
	sem_t                    scale_path_sem;

	sem_t                    thum_sem;
	uint32_t                 thum_from;
	uint32_t                 thum_ready;
	struct img_size          thum_size;
	struct img_size          thum_size_backup;
	struct cmr_cap_mem       cap_mem[CAMERA_CAP_FRM_CNT];
	struct cmr_cap_2_frm     cap_2_mems;
	pthread_mutex_t          cancel_mutex;
	uint32_t                 cap_canceled;
	takepicture_mode         cap_mode;

	uint32_t                 is_reset_if_cfg;
	/* for capture ZSL */
	struct frm_info          cap_frm_info[CAMERA_CAP_FRM_CNT];
	struct img_frm           cap_frm[CAMERA_CAP_FRM_CNT];
	uint32_t                 cap_phys_addr;
	uint32_t                 *cap_virt_addr;
	uint32_t                 cap_mem_szie;

	/*callback thread*/
	pthread_t                cb_thread;
	uint32_t                 cb_msg_que_handle;
	uint32_t                 cb_inited;
	sem_t                    cb_sync_sem;

	/*for setting*/
	struct camera_settings   cmr_set;
	uint32_t                 orientation;
	uint32_t                 hdr_cnt;

	uint32_t                 is_cfg_rot_cap;/*0:normal capture,1:rotation capture*/
	uint32_t                 cfg_cap_rot;

	uint32_t                 is_isp_ae_stab_eb;

	SENSOR_SOCID_T cpu_id;
	uint32_t		    which_cpu;
};

uint32_t camera_get_rot_angle(uint32_t degree);
uint32_t camera_get_img_type(uint32_t format_mode);
int camera_get_trim_rect(struct img_rect *src_trim_rect, uint32_t zoom_level, struct img_size *dst_size);
int camera_get_trim_rect2(struct img_rect *src_trim_rect, float zoomRatio, float dstRatio,uint32_t SensorW, uint32_t SensorH, uint8_t Rot);//for hal2.0
struct camera_context *camera_get_cxt(void);
int camera_set_pos_type(camera_position_type *position);
JINF_EXIF_INFO_T* camera_get_exif(struct camera_context *p_cxt);
int camera_sync_var_init(struct camera_context *p_cxt);
int camera_sync_var_deinit(struct camera_context *p_cxt);
int camera_wait_start(struct camera_context *p_cxt);
int camera_start_done(struct camera_context *p_cxt);
int camera_wait_set(struct camera_context *p_cxt);
int camera_set_done(struct camera_context *p_cxt);
int camera_wait_cap_path(struct camera_context *p_cxt);
int camera_cap_path_done(struct camera_context *p_cxt);
int camera_wait_scale_path(struct camera_context *p_cxt);
int camera_scale_path_done(struct camera_context *p_cxt);
int camera_wait_convert_thum(struct camera_context *p_cxt);
int camera_convert_thum_done(struct camera_context *p_cxt);
int camera_wait_stop(struct camera_context *p_cxt);
int camera_stop_done(struct camera_context *p_cxt);
int camera_wait_init(struct camera_context *p_cxt);
int camera_init_done(struct camera_context *p_cxt);
int camera_wait_exit(struct camera_context *p_cxt);
int camera_exit_done(struct camera_context *p_cxt);
int camera_wait_takepicdone(struct camera_context *p_cxt);
int camera_takepic_done(struct camera_context *p_cxt);
void camera_sensor_inf(struct sensor_context *sensor_cxt);
int camera_set_sensormark(void);
int camera_save_sensormark(void);
int camera_takepic_callback_done(struct camera_context *p_cxt);
int camera_wait_takepic_callback(struct camera_context *p_cxt);
int camera_sync_scale_start(struct camera_context *p_cxt);
int camera_sync_scale_done(struct camera_context *p_cxt);
int camera_sync_rotate_start(struct camera_context *p_cxt);
int camera_sync_rotate_done(struct camera_context *p_cxt);
int camera_save_to_file(uint32_t index, uint32_t img_fmt,
	uint32_t width, uint32_t height, struct img_addr *addr);
uint32_t getOrientationFromRotationDegrees(int degrees);

#ifdef __cplusplus
}
#endif

#endif
