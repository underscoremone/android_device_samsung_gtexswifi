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
#ifndef ANDROID_HARDWARE_SPRD_OEM_CAMERA_H
#define ANDROID_HARDWARE_SPRD_OEM_CAMERA_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <sys/types.h>
#include "../../arithmetic/sc8830/inc/FaceSolid.h"
#include "../../arithmetic/sc8830/inc/HDR2.h"
#include "sprd_dma_copy_k.h"
#include "cmr_v4l2.h"
#include "isp_app.h"
#include "sensor_drv_u.h"

#define FACE_DETECT_NUM     10
#define FACE_SMILE_LIMIT    10
#define HDR_CAP_NUM         3

typedef enum {
	CAMERA_NORMAL_MODE = 0,
	CAMERA_HDR_MODE,
	CAMERA_ZSL_MODE,/*oem zsl mode*/
	CAMERA_RAW_MODE,
	CAMERA_TOOL_RAW_MODE,
	CAMERA_ZSL_CONTINUE_SHOT_MODE,
	CAMERA_NORMAL_CONTINUE_SHOT_MODE,
	CAMERA_ANDROID_ZSL_MODE,/*hal2.0 zsl mode*/
	CAMERA_MODE_MAX
}takepicture_mode;

typedef enum {
	CAMERA_SUCCESS = 0,
	CAMERA_INVALID_STATE,
	CAMERA_INVALID_PARM,
	CAMERA_INVALID_FORMAT,
	CAMERA_NO_SENSOR,
	CAMERA_NO_MEMORY,
	CAMERA_NOT_SUPPORTED,
	CAMERA_FAILED,
	CAMERA_INVALID_STAND_ALONE_FORMAT,
	CAMERA_MALLOC_FAILED_STAND_ALONE,
	CAMERA_EXIT,
	CAMERA_JPEG_SPECIFY_FAILED,
	CAMERA_RET_CODE_MAX
} camera_ret_code_type;

typedef enum {
	/* YCbCr, each pixel is two bytes. Two pixels form a unit.
	* MSB is Y, LSB is CB for the first pixel and CR for the second pixel. */
	CAMERA_YCBCR,
	CAMERA_YCBCR_4_2_0,
	CAMERA_YCBCR_4_2_2,
	CAMERA_H1V1,
	CAMERA_H2V1,
	CAMERA_H1V2,
	CAMERA_H2V2,
	CAMERA_BAYER_8BIT,
	CAMERA_BAYER_10BIT,

	/* RGB565, each pixel is two bytes.
	* MS 5-bit is red, the next 6-bit is green. LS 5-bit is blue. */
	CAMERA_RGB565,
	/* RGB666, each pixel is four bytes.
	* MS 14 bits are zeros, the next 6-bit is red, then 6-bit of green.
	* LS 5-bit is blue. */
	CAMERA_RGB666,
	/* RGB444, each pixel is 2 bytes. The MS 4 bits are zeros, the next
	* 4 bits are red, the next 4 bits are green. The LS 4 bits are blue. */
	CAMERA_RGB444,
	/* Bayer, each pixel is 1 bytes. 2x2 pixels form a unit.
	* First line: first byte is blue, second byte is green.
	* Second line: first byte is green, second byte is red. */
	CAMERA_BAYER_BGGR,
	/* Bayer, each pixel is 1 bytes. 2x2 pixels form a unit.
	* First line: first byte is green, second byte is blue.
	* Second line: first byte is red, second byte is green. */
	CAMERA_BAYER_GBRG,
	/* Bayer, each pixel is 1 bytes. 2x2 pixels form a unit.
	* First line: first byte is green, second byte is red.
	* Second line: first byte is blue, second byte is green. */
	CAMERA_BAYER_GRBG,
	/* Bayer, each pixel is 1 bytes. 2x2 pixels form a unit.
	* First line: first byte is red, second byte is green.
	* Second line: first byte is green, second byte is blue. */
	CAMERA_BAYER_RGGB,
	/* RGB888, each pixel is 3 bytes. R is 8 bits, G is 8 bits,
	* B is 8 bits*/
	CAMERA_RGB888
} camera_format_type;

/* Enum Type for different ISO Mode supported */
typedef enum {
	CAMERA_PREVIEW_MODE_SNAPSHOT,
	CAMERA_PREVIEW_MODE_MOVIE,
	CAMERA_PREVIEW_MODE_SLOWMOTION,
	CAMERA_MAX_PREVIEW_MODE
} camera_preview_mode_type;

typedef struct {
	/* Format of the frame */
	camera_format_type      format;
	/* For pre-V7, Width and height of the picture.
	* For V7:
	*   Snapshot:     thumbnail dimension
	*   Raw Snapshot: not applicable
	*   Preview:      not applicable
	*/
	uint16_t                 dx;
	uint16_t                 dy;
	/* For pre_V7: For BAYER format, RAW data before scaling.
	* For V7:
	*   Snapshot:     Main image dimension
	*   Raw snapshot: raw image dimension
	*   Preview:      preview image dimension
	*/
	uint16_t                 captured_dx;
	uint16_t                 captured_dy;
	/* it indicates the degree of clockwise rotation that should be
	* applied to obtain the exact view of the captured image. */
	uint16_t                 rotation;
	/* Preview:      not applicable
	* Raw shapshot: not applicable
	* Snapshot:     thumbnail image buffer
	*/
	uint8_t                  *thumbnail_image;
	/* For pre-V7:
	*   Image buffer ptr
	* For V7:
	*   Preview: preview image buffer ptr
	*   Raw snapshot: Raw image buffer ptr
	*   Shapshot:     Main image buffer ptr
	*/
	uint8_t                  *buffer;
	uint8_t                  *Y_Addr;
	uint8_t                  *CbCr_Addr;
	uint32_t                 *buf_Virt_Addr;
	uint32_t                 header_size;
	/* For JPEG encoding*/
	uint32_t                 buffer_phy_addr;
	uint32_t                 thumbnail_phy_addr;
	uint32_t                 pmem_id;

	uint32_t                 buf_id; /*logical buffer id*/
	uint32_t                 order_buf_id;/*serial buffer id*/
	int64_t                  timestamp;
	uint32_t                 buffer_uv_phy_addr;
	/*face detect*/
	uint32_t face_num;
	morpho_FaceRect *face_ptr;
} camera_frame_type;

typedef struct {
	uint32_t                 *buf_Virt_Addr;
	uint32_t                 *buf_Virt_Uaddr;
	uint32_t                 width;
	uint32_t                 height;
	int64_t                  timestamp;
	struct frm_info          cap_info;
} camera_cap_frm_info;

typedef enum
{
	JPEGENC_MEM
} JPEGENC_outputType;

typedef enum
{
	JPEGENC_DSP_FAIL,
	JPEGENC_DSP_SUCCESS,
	JPEGENC_DSP_BAD_CMD,
	JPEGENC_IMG_DONE,
	JPEGENC_IMG_ABORT,
	JPEGENC_IMG_FAIL,
	JPEGENC_FILE_SIZE_FAIL,
	JPEGENC_FILLED_BUFFER
} JPEGENC_msgType;

typedef struct
{
	int32_t                  clientId;
	JPEGENC_msgType          status;
	uint32_t                 dcBitCnt;
	uint32_t                 header_size;
	JPEGENC_outputType       mode;
	void                     *outPtr;
	uint32_t                 size;
	uint32_t                 need_free;
} JPEGENC_CBrtnType;

typedef enum {
	CAMERA_DEVICE_MEM,
	CAMERA_DEVICE_EFS,
	CAMERA_DEVICE_MAX
} camera_device_type;

typedef enum {
	CAMERA_RAW,
	CAMERA_JPEG,
	CAMERA_PNG,
	CAMERA_YCBCR_ENCODE,
	CAMERA_ENCODE_TYPE_MAX
} camera_encode_type;

typedef struct {
	uint32_t                 buf_len;/* Length of each buffer */
	uint32_t                 used_len;
	uint8_t                  *buffer;
} camera_encode_mem_type;

#define MAX_JPEG_ENCODE_BUF_NUM 1
typedef struct {
	camera_device_type       device;
	int32_t                  quality;
	camera_encode_type       format;
	int32_t                  encBuf_num;
	camera_encode_mem_type   encBuf[MAX_JPEG_ENCODE_BUF_NUM];
} camera_handle_mem_type;

typedef union {
	camera_device_type       device;
	camera_handle_mem_type   mem;
} camera_handle_type;

typedef enum {
	CAMERA_RSP_CB_SUCCESS,    /* Function is accepted         */
	CAMERA_EXIT_CB_DONE,      /* Function is executed         */
	CAMERA_EXIT_CB_FAILED,    /* Execution failed or rejected */
	CAMERA_EXIT_CB_DSP_IDLE,  /* DSP is in idle state         */
	CAMERA_EXIT_CB_DSP_ABORT, /* Abort due to DSP failure     */
	CAMERA_EXIT_CB_ABORT,     /* Function aborted             */
	CAMERA_EXIT_CB_ERROR,     /* Failed due to resource       */
	CAMERA_EVT_CB_FRAME,      /* Preview or video frame ready */
	CAMERA_EVT_CB_PICTURE,    /* Picture frame ready for multi-shot */
	CAMERA_STATUS_CB,         /* Status updated               */
	CAMERA_EXIT_CB_FILE_SIZE_EXCEEDED, /* Specified file size not achieved, ecoded file written & returned anyway */
	CAMERA_EXIT_CB_BUFFER,    /* A buffer is returned         */
	CAMERA_EVT_CB_CAPTURE_FRAME_DONE,/*after captur frame, notify shutter*/
	CAMERA_EVT_CB_SNAPSHOT_DONE,/*  Snapshot updated               */
	CAMERA_EVT_CB_SNAPSHOT_JPEG_DONE,
	CAMERA_EVT_CB_FD,
	CAMERA_EVT_CB_FOCUS_MOVE,
	CAMERA_EVT_CB_FLUSH,
	CAMERA_EVT_CB_HAL2_ZSL_NEW_FRM,
	CAMERA_EVT_CB_HAL2_FLUSH_ZSL_BUF,
	CAMERA_CB_MAX
} camera_cb_type;

typedef enum {
	CAMERA_FUNC_START,
	CAMERA_FUNC_STOP,
	CAMERA_FUNC_SET_DIMENSIONS,
	CAMERA_FUNC_START_PREVIEW,
	CAMERA_FUNC_TAKE_PICTURE,
	CAMERA_FUNC_ENCODE_PICTURE,
	CAMERA_FUNC_COLOR_CONVERT,
	CAMERA_FUNC_START_RECORD,
	CAMERA_FUNC_START_FOCUS,
	CAMERA_FUNC_SET_OVERLAY,
	CAMERA_FUNC_CLR_OVERLAY,
	CAMERA_FUNC_SET_ICON_ARRAY,
	CAMERA_FUNC_CLR_ICON_ARRAY,
	CAMERA_FUNC_SET_POSITION,
	CAMERA_FUNC_SET_EXIF_TAG,
	CAMERA_FUNC_SET_PARM,
	CAMERA_FUNC_RELEASE_ENCODE_BUFFER,
	CAMERA_FUNC_MAX,
	/*==========================================================================
	* The followings are for internal use only
	==========================================================================*/
	CAMERA_FUNC_STOP_PREVIEW,
	CAMERA_FUNC_RELEASE_PICTURE,
	CAMERA_FUNC_PAUSE_RECORD,
	CAMERA_FUNC_RESUME_RECORD,
	CAMERA_FUNC_STOP_RECORD,
	CAMERA_FUNC_STOP_FOCUS,
	CAMERA_FUNC_ENABLE_FRAME_CALLBACK,
	CAMERA_FUNC_DISABLE_FRAME_CALLBACK,
	CAMERA_FUNC_RELEASE_FRAME,
	CAMERA_FUNC_BLT,
	CAMERA_FUNC_GET_INFO,
	CAMERA_FUNC_GET_PARM,
	CAMERA_FUNC_SET_REFLECT,
	CAMERA_FUNC_INIT_RECORD,
	CAMERA_FUNC_OFFLINE_SNAPSHOT,
	CAMERA_FUNC_TAKE_MULTIPLE_PICTURES,
	CAMERA_FUNC_PRVW_HISTOGRAM,
	CAMERA_FUNC_SET_ZOOM,
	CAMERA_FUNC_MAX1
} camera_func_type;

typedef void (*camera_cb_f_type)(camera_cb_type cb,
				const void *client_data,
				camera_func_type func,
				int32_t parm4);

typedef struct{
	camera_func_type       cb_func;
	camera_cb_type         cb_type;
	void                 * cb_data;
	uint32_t               cb_data_length;
	void                 * refer_data;
	uint32_t               refer_data_length;
}camera_cb_info;

typedef struct {
	int32_t                  quality;
	camera_encode_type       format;
	int32_t                  file_size;
} camera_encode_properties_type;

typedef struct
{
	long                     timestamp;  /* seconds since 1/6/1980          */
	double                   latitude;   /* degrees, WGS ellipsoid */
	double                   longitude;  /* degrees                */
	double                   altitude;   /* meters                          */
	const char               *process_method;
} camera_position_type;

typedef struct {
	uint32_t flash;
} camera_sensor_exif_info;

typedef enum {
	/* read only operation states: camera_state_type */
	CAMERA_PARM_STATE,
	/* read only active command in execution: camera_func_type */
	CAMERA_PARM_ACTIVE_CMD,
	/* zoom */
	CAMERA_PARM_ZOOM,
	/* This affects only when encoding. It has to be set only in
	* preview mode */
	CAMERA_PARM_ENCODE_ROTATION, /* 0, 90, 180, 270 degrees */
	/* Sensor can be rotated from forward direction to reversed direction or
	* vise versa. When in normal position, line 1 is on the top. When in
	* reverse position, line 1 is now at the bottom, not on the top, so the image
	* need to be reversed, 0 = normal, 1 = reverse */
	CAMERA_PARM_SENSOR_POSITION, /* use camera_sp_type */
	/* contrast */
	CAMERA_PARM_CONTRAST,
	/* brightness */
	CAMERA_PARM_BRIGHTNESS,
	/* sharpness */
	CAMERA_PARM_SHARPNESS,
	CAMERA_PARM_EXPOSURE,        /* use camera_exposure_type */
	CAMERA_PARM_WB,              /* use camera_wb_type */
	CAMERA_PARM_EFFECT,          /* use camera_effect_type */
	CAMERA_PARM_AUDIO_FMT,       /* use video_fmt_stream_audio_type */
	CAMERA_PARM_FPS,             /* frames per second, unsigned integer number */
	CAMERA_PARM_FLASH,           /* Flash control, see camera_flash_type */
	CAMERA_PARM_RED_EYE_REDUCTION, /* boolean */
	CAMERA_PARM_NIGHTSHOT_MODE,  /* Night shot mode, snapshot at reduced FPS */
	CAMERA_PARM_REFLECT,         /* Use camera_reflect_type */
	CAMERA_PARM_PREVIEW_MODE,    /* Use camera_preview_mode_type */
	CAMERA_PARM_ANTIBANDING,     /* Use camera_anti_banding_type */
	///  CAMERA_PARM_THUMBNAIL_WIDTH, /* Width of thumbnail */
	///  CAMERA_PARM_THUMBNAIL_HEIGHT, /* Height of thumbnail */
	///  CAMERA_PARM_THUMBNAIL_QUALITY, /* Quality of thumbnail */
	CAMERA_PARM_FOCUS_STEP,
	CAMERA_PARM_FOCUS_RECT, /* Suresh Gara & Saikumar*/
	CAMERA_PARM_AF_MODE,
	/* Name change to CAMERA_PARM_EXPOSURE_METERING, remove this later */
	CAMERA_PARM_AUTO_EXPOSURE_MODE, /* Use camera_auto_exposure_mode_type */
	CAMERA_PARM_ISO,
	/* Use to control the exposure compensation */
	CAMERA_PARM_EXPOSURE_COMPENSATION,
	CAMERA_PARM_PREVIEW_FPS,
	CAMERA_PARM_EXPOSURE_METERING,
	CAMERA_PARM_APERTURE,
	CAMERA_PARM_SHUTTER_SPEED,
	CAMERA_PARM_FLASH_STATE,
	CAMERA_PARM_HUE,
	CAMERA_PARM_SATURATION,
	CAMERA_PARM_LUMA_ADAPTATION,
	CAMERA_PARM_HISTOGRAM,
	CAMERA_PARM_SCENE_MODE,          /* normal, night, snow, action */
	CAMERA_PARM_CAMERA_ID,
	CAMERA_PARM_JPEGCOMP,
	CAMERA_PARM_THUMBCOMP,
	CAMERA_PARM_ORIENTATION,//wxz20110815: for the phone orientation: landscape and portrait
	CAMERA_PARM_FOCAL_LENGTH,
	CAMERA_PARM_SENSOR_ROTATION, /* 0, 90, 180, 270 degrees */
	CAMERA_PARM_FOCUS_MODE,
	CAMERA_PARM_DCDV_MODE,
	CAMERA_PARM_SHOT_NUM,
	CAMERA_PARAM_SLOWMOTION,
	CAMERA_PARAM_ROTATION_CAPTURE,
	CAMERA_PARM_PREVIEW_ENV,
	CAMERA_PARM_ZOOM_RECT,//for hal2.0
	CAMERA_PARM_MAX
} camera_parm_type;

typedef enum {
	CAMERA_AUTO_FOCUS,
	CAMERA_MANUAL_FOCUS
} camera_focus_e_type;

typedef enum {
	CAMERA_TOOL_CAP_RAW_NONE,
	CAMERA_TOOL_CAP_RAW_ENABLE
} camera_tool_cap_raw_mode;

typedef enum preview_buffer_usage{
	PREVIEW_BUFFER_USAGE_DCAM = 0,/*camera allocate buffer and copy to grapics buffer*/
	PREVIEW_BUFFER_USAGE_GRAPHICS/*camera use the grapics buffer directly*/
} preview_buffer_usage_mode;

typedef struct _cropZoom{
        uint32_t crop_x;
		uint32_t crop_y;
		uint32_t crop_w;
		uint32_t crop_h;
} cropZoom;

void camera_zsl_pic_cb_done();

camera_ret_code_type camera_encode_picture(camera_frame_type *frame,
					camera_handle_type *handle,
					camera_cb_f_type callback,
					void *client_data);

camera_ret_code_type camera_init(int32_t camera_id);

camera_ret_code_type camera_release_frame(uint32_t index);

camera_ret_code_type camera_set_dimensions(uint16_t picture_width,
					uint16_t picture_height,
					uint16_t display_width,
					uint16_t display_height,
					camera_cb_f_type callback,
					void *client_data,
					uint32_t can_resize);

camera_ret_code_type camera_set_encode_properties(camera_encode_properties_type *encode_properties);

camera_ret_code_type camera_set_parm(camera_parm_type id,
				uint32_t         parm,
				camera_cb_f_type callback,
				void            *client_data);

camera_ret_code_type camera_set_position(camera_position_type *position,
					camera_cb_f_type      callback,
					void                 *client_data);

camera_ret_code_type camera_set_thumbnail_properties(uint32_t width,
						uint32_t height,
						uint32_t quality);

camera_ret_code_type camera_start(camera_cb_f_type callback,
				void *client_data,
				int  display_height,
				int  display_width);

camera_ret_code_type camera_start_preview(camera_cb_f_type callback,
					void *client_data,takepicture_mode mode);

camera_ret_code_type camera_start_focus(void *client_data);

camera_ret_code_type camera_cancel_autofocus(void);

camera_ret_code_type camera_stop(camera_cb_f_type callback,
				void *client_data);

camera_ret_code_type camera_stop_preview(void);

camera_ret_code_type camera_stop_capture(void);

camera_ret_code_type camera_take_picture(camera_cb_f_type    callback,
					void                 *client_data,takepicture_mode cap_mode);

uint32_t camera_get_size_align_page(uint32_t size);

int camera_start_autofocus(camera_focus_e_type focus,
			camera_cb_f_type callback,
			void *client_data);

int camera_capture_max_img_size(uint32_t *max_width, uint32_t *max_height);
int camera_get_sensor_mode_trim(uint32_t mode, cropZoom *sensor_trim, uint16_t *width, uint16_t *height);

int camerea_set_preview_format(uint32_t pre_format);

int camera_set_preview_mem(uint32_t phy_addr, uint32_t vir_addr, uint32_t mem_size, uint32_t mem_num);

int camera_capture_get_buffer_size(uint32_t camera_id,
						uint32_t width,
						uint32_t height,
						uint32_t *size);
int camera_set_capture_mem(uint32_t     cap_index,
						uint32_t phy_addr,
						uint32_t vir_addr,
						uint32_t mem_size,
						uint32_t alloc_mem,
						uint32_t free_mem,
						uint32_t handle);

int camera_get_data_redisplay(int output_addr,
					int output_width,
					int output_height,
					int input_addr_y,
					int input_addr_uv,
					int input_width,
					int input_height);

uint32_t camera_get_rot_set(void);
void camera_set_facedetect_support(uint32_t is_support);
void camera_set_start_facedetect(uint32_t param, uint32_t mem_size);
void camera_call_cb(camera_cb_type cb,
                 const void *client_data,
                 camera_func_type func,
                 int32_t parm4);
void camera_direct_call_cb(camera_cb_type cb,
                 const void *client_data,
                 camera_func_type func,
                 int32_t parm4);
void camera_callback_start(camera_cb_info *cb_info);
void *camera_get_client_data(void);
int camera_set_change_size(uint32_t cap_width,uint32_t cap_height,uint32_t preview_width,uint32_t preview_height);
int camera_get_preview_rect(int *rect_x, int *rect_y, int *rect_width, int *rect_height);
camera_ret_code_type camera_take_picture_raw(camera_cb_f_type    callback,
					void                 *client_data,takepicture_mode cap_mode);
int camera_is_need_stop_preview(void);
int camera_get_is_scale(void);
int camera_dma_copy_data(struct _dma_copy_cfg_tag dma_copy_cfg);
camera_ret_code_type camera_cfg_rot_cap_param_reset(void);
int camera_get_is_nonzsl(void);
int camera_is_sensor_support_zsl(void);
int camera_isp_start(uint32_t work_mode,uint32_t need_binning,SENSOR_MODE_INFO_T *sensor_mode);
int camera_isp_ae_info(SENSOR_AE_INFO_T *sensor_aec_info);
int camera_isp_wb_trim(struct img_frm_cap *frm_cfg);
int camera_isp_awb_bypass(enum isp_alg_mode awb_mode);
int camera_isp_ae_bypass(enum isp_alg_mode ae_mode);
int camera_isp_flash_ratio(SENSOR_FLASH_LEVEL_T *flash_level);
void camera_isp_ae_stab_set (uint32_t is_ae_stab_eb);
inline uint32_t camera_get_prev_stat();
int camera_capture_is_idle(void);
int camera_set_cancel_capture(int set_val); /*for hal can invoke*/
void camera_config_exif_info(camera_sensor_exif_info * exif_info);
void camera_set_preview_trace(uint32_t is_trace);
void camera_set_capture_trace(uint32_t is_trace);

/*following functions are for HAL2.0*/
int camera_zsl_substream_process(struct frm_info *data,uint32_t *srcPhy,uint32_t *srcVirt,uint32_t width,uint32_t height);

#ifdef __cplusplus
}
#endif

#endif //ANDROID_HARDWARE_SPRD_OEM_CAMERA_H

