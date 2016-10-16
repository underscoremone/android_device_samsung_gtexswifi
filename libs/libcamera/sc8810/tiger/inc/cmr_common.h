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
#ifndef _CMR_COMMON_H_
#define _CMR_COMMON_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <sys/types.h>
#include <utils/Log.h>

//#define DEBUG_STR     "%s(L %d), %s: "
//#define DEBUG_ARGS    __FILE__,__LINE__,__FUNCTION__
#define DEBUG_STR     "L %d, %s: "
#define DEBUG_ARGS    __LINE__,__FUNCTION__

#define CMR_LOGV(format,...) LOGV(DEBUG_STR format, DEBUG_ARGS, ##__VA_ARGS__)

#define CMR_LOGE(format,...) LOGE(DEBUG_STR format, DEBUG_ARGS, ##__VA_ARGS__)

#define CMR_LOGI(format,...) LOGI(DEBUG_STR format, DEBUG_ARGS, ##__VA_ARGS__)

#define CMR_LOGW(format,...) LOGW(DEBUG_STR format, DEBUG_ARGS, ##__VA_ARGS__)

#define CMR_EVT_V4L2_BASE                  (1 << 16)
#define CMR_EVT_CVT_BASE                   (1 << 17)
#define CMR_EVT_ISP_BASE                   (1 << 18)
#define CMR_EVT_SENSOR_BASE                (1 << 19)
#define CMR_EVT_JPEG_BASE                  (1 << 20)
#define CMR_EVT_START                      (1 << 21)
#define CMR_EVT_STOP                       (1 << 22)
#define CMR_EVT_AF                         (1 << 23)
#define CMR_SLICE_HEIGHT                   128
#define CMR_JPEG_COMPRESS_FACTOR           2
#define CMR_JPEG_SZIE(w,h)                 (uint32_t)((w)*(h)/CMR_JPEG_COMPRESS_FACTOR)
#define CMR_EVT_MASK_BITS                  (uint32_t)(CMR_EVT_V4L2_BASE | CMR_EVT_CVT_BASE | \
					CMR_EVT_ISP_BASE | CMR_EVT_SENSOR_BASE | \
					CMR_EVT_JPEG_BASE | CMR_EVT_START | \
					CMR_EVT_AF | CMR_EVT_STOP)

#define CMR_RTN_IF_ERR(n)                                              \
		do {                                                   \
			if (n) {                                       \
				CMR_LOGE("ret %d", n);                 \
				goto exit;                             \
			}                                              \
		} while(0)

#ifndef MIN
#define MIN(x,y) (((x)<(y))?(x):(y))
#endif

#ifndef MAX
#define MAX(x,y) (((x)>(y))?(x):(y))
#endif
			
enum img_rot_angle {
	IMG_ROT_0 = 0,
	IMG_ROT_90,
	IMG_ROT_270,
	IMG_ROT_180,
	IMG_ROT_MIRROR,
	IMG_ROT_MAX
};

enum img_data_type_e {
	IMG_DATA_TYPE_JPEG = 0,
	IMG_DATA_TYPE_YUV422,
	IMG_DATA_TYPE_YUV420,
	IMG_DATA_TYPE_RGB565,
	IMG_DATA_TYPE_RGB888,
	IMG_DATA_TYPE_RAW,
	IMG_DATA_TYPE_MAX
};

enum img_skip_mode {
	IMG_SKIP_HW = 0,
	IMG_SKIP_SW				
};

enum restart_mode {
	RESTART_LIGHTLY,
	RESTART_MIDDLE,
	RESTART_HEAVY,
	RESTART_MAX
};

struct img_size {
	uint32_t                            width;	
	uint32_t                            height;
};

struct img_rect {
	uint32_t                            start_x;
	uint32_t                            start_y;
	uint32_t                            width;
	uint32_t                            height;
};

struct img_addr {
	uint32_t                            addr_y;
	uint32_t                            addr_u;
	uint32_t                            addr_v;
};

struct img_data_end {
	uint8_t                             y_endian;
	uint8_t                             uv_endian;
	uint8_t                             reserved0;
	uint8_t                             reserved1;
};

struct img_frm {
	uint32_t                            fmt;
	uint32_t                            buf_size;
	struct img_size                     size;
	struct img_addr                     addr_phy;
	struct img_addr                     addr_vir;
	struct img_data_end                 data_end;
	void*                               reserved;
};

struct ccir_if {
	uint8_t                             v_sync_pol;
	uint8_t                             h_sync_pol;
	uint8_t                             pclk_pol;
	uint8_t                             res1;
};

struct mipi_if {
	uint8_t                             use_href; 
	uint8_t                             bits_per_pxl; 
	uint8_t                             is_packet; 
	uint8_t                             lane_num;
};

struct  sensor_if {
	uint8_t                             if_type; 
	uint8_t                             img_fmt; 
	uint8_t                             img_ptn; 
	uint8_t                             frm_deci;
	uint8_t                             res[4];
	struct img_size                     size; 
	union  {
		struct ccir_if              ccir;
		struct mipi_if              mipi;
	}if_spec;
};

typedef void (*cmr_evt_cb)(int evt, void* data);
typedef int  (*cmr_before_set_cb)(enum restart_mode re_mode);
typedef int  (*cmr_after_set_cb)(enum restart_mode re_mode, enum img_skip_mode skip_mode, uint32_t skip_number);

#ifdef __cplusplus
}
#endif

#endif //for _CMR_COMMON_H_

