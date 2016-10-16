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
#ifndef _ISP_APP_H_
#define _ISP_APP_H_
/*----------------------------------------------------------------------------*
**				Dependencies					*
**---------------------------------------------------------------------------*/
//#include <linux/types.h>
#include <sys/types.h>

/**---------------------------------------------------------------------------*
**				Micro Define					*
**----------------------------------------------------------------------------*/

typedef int32_t ( *isp_callback)(uint32_t rtn, void* param_ptr);
typedef int32_t ( *proc_callback)(int32_t mode, void* param_ptr);
typedef int32_t ( *isp_ctrl_fun)(uint32_t param);

#define ISP_EVT_MASK	0x00000F00
/**---------------------------------------------------------------------------*
**				Data Prototype					*
**----------------------------------------------------------------------------*/
//enum
enum isp_callback_evt{
	ISP_CALLBACK_EVT=0x00040000,
	ISP_CALLBACK_MAX
};

enum isp_callback_cmd{
	ISP_CTRL_CALLBAC=0x00000000,
	ISP_PROC_CALLBACK=0x00000100,
	ISP_AF_NOTICE_CALLBACK=0x00000200,
	ISP_SKIP_FRAME_CALLBACK=0x00000300,
	ISP_CALLBACK_CMD_MAX=0xffffffff
};

enum isp_focus_mode{
	ISP_FOCUS_NONE=0x00,
	ISP_FOCUS_TRIG,
	ISP_FOCUS_ZONE,
	ISP_FOCUS_MULTI_ZONE,
	ISP_FOCUS_MACRO,
	ISP_FOCUS_MAX
};

enum isp_hdr_level{
	ISP_HDR_LOW=0x00,
	ISP_HDR_HIGH,
	ISP_HDR_BYPASS,
	ISP_HDR_LEVEL_MAX
};

enum isp_special_effect_mode{
	ISP_EFFECT_NORMAL=0x00,
	ISP_EFFECT_GRAY,
	ISP_EFFECT_WARM,
	ISP_EFFECT_GREEN,
	ISP_EFFECT_COOL,
	ISP_EFFECT_ORANGE,
	ISP_EFFECT_NEGTIVE,
	ISP_EFFECT_OLD,
	ISP_EFFECT_EMBOSS,
	ISP_EFFECT_MAX
};

enum isp_tune_level{
	ISP_LEVEL_0=0x00,
	ISP_LEVEL_1,
	ISP_LEVEL_2,
	ISP_LEVEL_3,
	ISP_LEVEL_4,
	ISP_LEVEL_5,
	ISP_LEVEL_6,
	ISP_LEVEL_7,
	ISP_LEVEL_8,
	ISP_LEVEL_9,
	ISP_LEVEL_10,
	ISP_LEVEL_11,
	ISP_LEVEL_12,
	ISP_LEVEL_13,
	ISP_LEVEL_14,
	ISP_LEVEL_15,
	ISP_TUNE_MAX
};

enum isp_ae_frame_mode{
	ISP_AE_AUTO=0x00,
	ISP_AE_FIX,
	ISP_AE_FRAME_MODE_MAX
};

enum isp_alg_mode{
	ISP_ALG_NORMAL=0x00,
	ISP_ALG_FAST,
	ISP_AE_BYPASS,
	ISP_AWB_BYPASS,
	ISP_ALG_MAX
};

enum isp_ae_wditht{
	ISP_AE_WDITHT_AVG=0x00,
	ISP_AE_WDITHT_CENTER,
	ISP_AE_WDITHT_CUSTOMER,
	ISP_AE_WDITHT_MAX
};

enum isp_flicker_mode{
	ISP_FLICKER_50HZ=0x00,
	ISP_FLICKER_60HZ,
	ISP_FLICKER_MAX
};

enum isp_ae_mode{
	ISP_AUTO=0x00,
	ISP_NIGHT,
	ISP_SPORT,
	ISP_PORTRAIT,
	ISP_LANDSCAPE,
	ISP_AE_MODE_MAX
};

enum isp_iso{
	ISP_ISO_100=0x00,
	ISP_ISO_200,
	ISP_ISO_400,
	ISP_ISO_800,
	ISP_ISO_1600,
	ISP_ISO_MAX
};

enum isp_awb_mode{
	ISP_AWB_INDEX0=0x00,
	ISP_AWB_INDEX1,
	ISP_AWB_INDEX2,
	ISP_AWB_INDEX3,
	ISP_AWB_INDEX4,
	ISP_AWB_INDEX5,
	ISP_AWB_INDEX6,
	ISP_AWB_INDEX7,
	ISP_AWB_INDEX8,
	ISP_AWB_AUTO,
	ISP_AWB_MAX
};

enum isp_format{
	ISP_DATA_YUV422_3FRAME=0x00,
	ISP_DATA_YUV422_2FRAME,
	ISP_DATA_YVU422_2FRAME,
	ISP_DATA_YUYV,
	ISP_DATA_UYVY,
	ISP_DATA_YVYU,
	ISP_DATA_VYUY,
	ISP_DATA_YUV420_2FRAME,
	ISP_DATA_YVU420_2FRAME,
	ISP_DATA_YUV420_3_FRAME,
	ISP_DATA_NORMAL_RAW10,
	ISP_DATA_CSI2_RAW10,
	ISP_DATA_FORMAT_MAX
};

enum isp_ctrl_cmd{
	ISP_CTRL_AWB_MODE,
	ISP_CTRL_AE_MODE,
	ISP_CTRL_AE_MEASURE_LUM,
	ISP_CTRL_EV,
	ISP_CTRL_FLICKER,
	ISP_CTRL_ALG,
	ISP_CTRL_SPECIAL_EFFECT,
	ISP_CTRL_BRIGHTNESS,
	ISP_CTRL_CONTRAST,
	ISP_CTRL_HIST,
	ISP_CTRL_AUTO_CONTRAST,
	ISP_CTRL_SATURATION,
	ISP_CTRL_AF,
	ISP_CTRL_AF_MODE,
	ISP_CTRL_CSS,
	ISP_CTRL_HDR,
	ISP_CTRL_GLOBAL_GAIN,
	ISP_CTRL_CHN_GAIN,
	ISP_CTRL_EXIF,
	ISP_CTRL_ISO,
	ISP_CTRL_WB_TRIM,
	ISP_CTRL_MAX
};

enum isp_capbility_cmd{
	ISP_VIDEO_SIZE,
	ISP_CAPTURE_SIZE,
	ISP_CAPBILITY_MAX
};

struct isp_addr{
	uint32_t chn0;
	uint32_t chn1;
	uint32_t chn2;
	uint32_t chn0_len;
	uint32_t chn1_len;
	uint32_t chn2_len;
};

struct isp_pos{
	uint32_t x;
	uint32_t y;
};

struct isp_size{
	uint32_t w;
	uint32_t h;
};

struct isp_trim_size{
	uint32_t x;
	uint32_t y;
	uint32_t w;
	uint32_t h;
};

struct isp_pos_rect{
	uint32_t start_x;
	uint32_t start_y;
	uint32_t end_x;
	uint32_t end_y;
};

struct isp_skip_num{
	uint32_t skip_num;
};

struct isp_af_notice{
	uint32_t valid_win;
};

struct isp_af_win{
	enum isp_focus_mode mode;
	struct isp_pos_rect win[9];
	uint32_t valid_win;
};

struct isp_img_frm{
	enum isp_format img_fmt;
	struct isp_size img_size;
	struct isp_addr img_addr_phy;
	struct isp_addr img_addr_vir;
};

struct isp_init_param{
	void* setting_param_ptr;
	struct isp_size size;
	proc_callback ctrl_callback;
};

struct isp_video_limit{
	uint16_t width;
	uint16_t height;
	uint32_t res;
};

struct isp_video_start{
	struct isp_size size;
	enum isp_format format;
};

struct ips_in_param{
	struct isp_img_frm src_frame;
	uint32_t src_avail_height;
	uint32_t src_slice_height;
	struct isp_img_frm dst_frame;
	uint32_t dst_slice_height;
};

struct ips_out_param{
	uint32_t output_height;
};

struct ipn_in_param{
	uint32_t src_avail_height;
	uint32_t src_slice_height;
	struct isp_addr img_addr_phy;
	struct isp_addr src_addr_phy;
	struct isp_addr dst_addr_phy;
};


int isp_init(struct isp_init_param* ptr);
int isp_deinit(void);
int isp_capbility(enum isp_capbility_cmd cmd, void* param_ptr);
int isp_ioctl(enum isp_ctrl_cmd cmd, void* param_ptr);
int isp_video_start(struct isp_video_start* param_ptr);
int isp_video_stop(void);
int isp_proc_start(struct ips_in_param* in_param_ptr, struct ips_out_param* out_param_ptr);
int isp_proc_next(struct ipn_in_param* in_ptr, struct ips_out_param *out_ptr);

/**---------------------------------------------------------------------------*/
#endif
// End

