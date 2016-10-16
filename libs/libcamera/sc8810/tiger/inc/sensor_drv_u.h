/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _SENSOR_DRV_U_H_
#define _SENSOR_DRV_U_H_
//#include <linux/i2c.h>
//#include <linux/gpio.h>
//#include <linux/delay.h>
//#include <mach/hardware.h>
//#include <asm/io.h>
//#include <linux/list.h>
//#include <linux/mutex.h>
#include "jpeg_exif_header.h"
#include "cmr_common.h"
#include <sys/types.h>

#define SENSOR_SUCCESS 0
#define SENSOR_FAIL 1
#define SENSOR_FALSE 0
#define SENSOR_TRUE 1

#define SENSOR_Sleep(ms) 		usleep(ms*1000)
#define SENSOR_MEMSET 			memset

#define BOOLEAN char
#define PNULL  ((void *)0)
#define LOCAL static

#define DEBUG_SENSOR_DRV 1
#ifdef DEBUG_SENSOR_DRV
#define SENSOR_PRINT   			CMR_LOGV
#else
#define SENSOR_PRINT(...)
#endif

#define SENSOR_PRINT_ERR   		CMR_LOGE
#define SENSOR_PRINT_HIGH 		CMR_LOGV
#define SENSOR_TRACE   			SENSOR_PRINT


#define NUMBER_OF_ARRAY(a)    				(sizeof(a)/sizeof(a[0]))
#define ADDR_AND_LEN_OF_ARRAY(a) 			(SENSOR_REG_T*)a, NUMBER_OF_ARRAY(a)

#define SENSOR_DEFALUT_MCLK					24	// MHZ
#define SENSOR_DISABLE_MCLK					0	// MHZ
#define SENSOR_LOW_PULSE_RESET				0x00
#define SENSOR_HIGH_PULSE_RESET				0x01

#define SENSOR_RESET_PULSE_WIDTH_DEFAULT	20
#define SENSOR_RESET_PULSE_WIDTH_MAX		200

#define SENSOR_LOW_LEVEL_PWDN				0x00
#define SENSOR_HIGH_LEVEL_PWDN				0x01


#define SENSOR_IDENTIFY_CODE_COUNT			0x02

/* bit define */
#define S_BIT_0               0x00000001
#define S_BIT_1               0x00000002
#define S_BIT_2               0x00000004
#define S_BIT_3               0x00000008
#define S_BIT_4               0x00000010
#define S_BIT_5               0x00000020
#define S_BIT_6               0x00000040
#define S_BIT_7               0x00000080
#define S_BIT_8               0x00000100

/*Image effect*/
#define SENSOR_IMAGE_EFFECT_NORMAL			(0x01 << 0)
#define SENSOR_IMAGE_EFFECT_BLACKWHITE		(0x01 << 1)
#define SENSOR_IMAGE_EFFECT_RED				(0x01 << 2)
#define SENSOR_IMAGE_EFFECT_GREEN			(0x01 << 3)
#define SENSOR_IMAGE_EFFECT_BLUE				(0x01 << 4)
#define SENSOR_IMAGE_EFFECT_YELLOW			(0x01 << 5)
#define SENSOR_IMAGE_EFFECT_NEGATIVE			(0x01 << 6)
#define SENSOR_IMAGE_EFFECT_CANVAS			(0x01 << 7)
#define SENSOR_IMAGE_EFFECT_RELIEVOS			(0x01 << 8)

/*While balance mode*/
#define SENSOR_WB_MODE_AUTO 				(0x01 << 0)
#define SENSOR_WB_MODE_INCANDESCENCE		(0x01 << 1)
#define SENSOR_WB_MODE_U30					(0x01 << 2)
#define SENSOR_WB_MODE_CWF					(0x01 << 3)
#define SENSOR_WB_MODE_FLUORESCENT		(0x01 << 4)
#define SENSOR_WB_MODE_SUN					(0x01 << 5)
#define SENSOR_WB_MODE_CLOUD				(0x01 << 6)

/*Preview mode*/
#define SENSOR_ENVIROMENT_NORMAL			(0x01 << 0)
#define SENSOR_ENVIROMENT_NIGHT			(0x01 << 1)
#define SENSOR_ENVIROMENT_SUNNY			(0x01 << 2)
#define SENSOR_ENVIROMENT_SPORTS			(0x01 << 3)
#define SENSOR_ENVIROMENT_LANDSCAPE		(0x01 << 4)
#define SENSOR_ENVIROMENT_PORTRAIT			(0x01 << 5)
#define SENSOR_ENVIROMENT_PORTRAIT_NIGHT	(0x01 << 6)
#define SENSOR_ENVIROMENT_BACKLIGHT		(0x01 << 7)
#define SENSOR_ENVIROMENT_MARCO			(0x01 << 8)

#define SENSOR_ENVIROMENT_MANUAL			(0x01 << 30)
#define SENSOR_ENVIROMENT_AUTO				(0x01 << 31)

/*YUV PATTERN*/
#define SENSOR_IMAGE_PATTERN_YUV422_YUYV	0x00
#define SENSOR_IMAGE_PATTERN_YUV422_YVYU	0x01
#define SENSOR_IMAGE_PATTERN_YUV422_UYVY	0x02
#define SENSOR_IMAGE_PATTERN_YUV422_VYUY	0x03
/*RAW RGB BAYER*/
#define SENSOR_IMAGE_PATTERN_RAWRGB_GR	0x00
#define SENSOR_IMAGE_PATTERN_RAWRGB_R		0x01
#define SENSOR_IMAGE_PATTERN_RAWRGB_B		0x02
#define SENSOR_IMAGE_PATTERN_RAWRGB_GB	0x03

/*I2C REG/VAL BIT count*/
#define SENSOR_I2C_VAL_8BIT			0x00
#define SENSOR_I2C_VAL_16BIT			0x01
#define SENSOR_I2C_REG_8BIT			(0x00 << 1)
#define SENSOR_I2C_REG_16BIT			(0x01 << 1)
#define SENSOR_I2C_CUSTOM 			(0x01 << 2)

/*I2C ACK/STOP BIT count*/
#define SNESOR_I2C_ACK_BIT (0x00 << 3)
#define SNESOR_I2C_NOACK_BIT (0x00 << 3)
#define SNESOR_I2C_STOP_BIT (0x00 << 3)
#define SNESOR_I2C_NOSTOP_BIT (0x00 << 3)

/*I2C FEEQ BIT count*/
#define SENSOR_I2C_CLOCK_MASK      (7<<5)
#define SENSOR_I2C_FREQ_20	(0x01 << 5)
#define SENSOR_I2C_FREQ_50 	(0x02 << 6)
#define SENSOR_I2C_FREQ_100 	(0x00 << 5)
#define SENSOR_I2C_FREQ_200 	(0x03 << 5)
#define SENSOR_I2C_FREQ_400  	(0x04 << 5)

#define SENSOR_I2C_ID			1

/*Hardward signal polarity*/
#define SENSOR_HW_SIGNAL_PCLK_N				0x00
#define SENSOR_HW_SIGNAL_PCLK_P				0x01
#define SENSOR_HW_SIGNAL_VSYNC_N			(0x00 << 2)
#define SENSOR_HW_SIGNAL_VSYNC_P			(0x01 << 2)
#define SENSOR_HW_SIGNAL_HSYNC_N			(0x00 << 4)
#define SENSOR_HW_SIGNAL_HSYNC_P			(0x01 << 4)

#define SENSOR_WRITE_DELAY					0xffff

#define SENSOR_IOCTL_FUNC_NOT_REGISTER		0xffffffff

/*sensor focus mode*/
#define SENSOR_FOCUS_TRIG 0x01
#define SENSOR_FOCUS_ZONE (0x01<<1)

/*sensor exposure mode*/
#define SENSOR_EXPOSURE_AUTO 0x01
#define SENSOR_EXPOSURE_ZONE (0x01<<1)

#define FOCUS_ZONE_CNT_MAX   6

/*  isp param for raw */
#define SENSOR_AE_TAB_NUM 0x04
#define SENSOR_CMC_NUM 0x09
#define SENSOR_AWB_NUM 0x09
#define SENSOR_MAP_NUM 0x08

#define SENSOR_RAW_VERSION_ID 0x0000 /* tiger-00xx, soft-xx00*/
/*  isp param for raw  end */

enum sensor_evt {
	CMR_SENSOR_ERROR = CMR_EVT_SENSOR_BASE
};

typedef enum {
	SENSOR_OP_SUCCESS = SENSOR_SUCCESS,
	SENSOR_OP_PARAM_ERR,
	SENSOR_OP_STATUS_ERR,
	SENSOR_OP_ERR,
	SENSOR_OP_MAX = 0xFFFF
} ERR_SENSOR_E;

typedef enum {
	SENSOR_MAIN = 0,
	SENSOR_SUB,
	SENSOR_ATV = 5,
	SENSOR_ID_MAX
} SENSOR_ID_E;

typedef enum {
	SENSOR_TYPE_NONE = 0x00,
	SENSOR_TYPE_IMG_SENSOR,
	SENSOR_TYPE_ATV,
	SENSOR_TYPE_MAX
} SENSOR_TYPE_E;

typedef enum {
	SENSOR_AVDD_3800MV = 0,
	SENSOR_AVDD_3000MV,
	SENSOR_AVDD_2800MV,
	SENSOR_AVDD_2500MV,
	SENSOR_AVDD_1800MV,
	SENSOR_AVDD_1500MV,
	SENSOR_AVDD_1300MV,
	SENSOR_AVDD_1200MV,
	SENSOR_AVDD_CLOSED,
	SENSOR_AVDD_UNUSED
} SENSOR_AVDD_VAL_E;

typedef enum {
	SENSOR_MCLK_12M = 12,
	SENSOR_MCLK_13M = 13,
	SENSOR_MCLK_24M = 24,
	SENSOR_MCLK_26M = 26,
	SENSOR_MCLK_MAX
} SENSOR_M_CLK_E;

typedef enum {
	SENSOR_INTERFACE_CCIR601_8BITS = 0,
	SENSOR_INTERFACE_CCIR601_4BITS,
	SENSOR_INTERFACE_CCIR601_2BITS,
	SENSOR_INTERFACE_CCIR601_1BITS,
	SENSOR_INTERFACE_CCIR656_8BITS,
	SENSOR_INTERFACE_CCIR656_4BITS,
	SENSOR_INTERFACE_CCIR656_2BITS,
	SENSOR_INTERFACE_CCIR656_1BITS,
	SENSOR_INTERFACE_SPI_8BITS,
	SENSOR_INTERFACE_SPI_4BITS,
	SENSOR_INTERFACE_SPI_4BITS_BE,
	SENSOR_INTERFACE_SPI_2BITS,
	SENSOR_INTERFACE_SPI_2BITS_BE,
	SENSOR_INTERFACE_SPI_1BITS,
	SENSOR_INTERFACE_SPI_1BITS_BE,
	SENSOR_INTERFACE_CSI2,
	SENSOR_INTERFACE_MAX
} SENSOR_INTERFACE_E;
typedef enum {
	SENSOR_MODE_COMMON_INIT = 0,
	SENSOR_MODE_PREVIEW_ONE,
	SENSOR_MODE_SNAPSHOT_ONE_FIRST,
	SENSOR_MODE_SNAPSHOT_ONE_SECOND,
	SENSOR_MODE_SNAPSHOT_ONE_THIRD,
	SENSOR_MODE_PREVIEW_TWO,
	SENSOR_MODE_SNAPSHOT_TWO_FIRST,
	SENSOR_MODE_SNAPSHOT_TWO_SECOND,
	SENSOR_MODE_SNAPSHOT_TWO_THIRD,
	SENSOR_MODE_MAX
} SENSOR_MODE_E;

typedef enum {
	SENSOR_IMAGE_FORMAT_YUV422 = 0,
	SENSOR_IMAGE_FORMAT_YUV420,
	SENSOR_IMAGE_FORMAT_RAW,
	SENSOR_IMAGE_FORMAT_RGB565,
	SENSOR_IMAGE_FORMAT_RGB666,
	SENSOR_IMAGE_FORMAT_RGB888,
	SENSOR_IMAGE_FORMAT_CCIR656,
	SENSOR_IMAGE_FORMAT_JPEG,
	SENSOR_IMAGE_FORMAT_MAX
} SENSOR_IMAGE_FORMAT;

typedef enum {
	SENSOR_IOCTL_RESET = 0,
	SENSOR_IOCTL_POWER,
	SENSOR_IOCTL_ENTER_SLEEP,
	SENSOR_IOCTL_IDENTIFY,
	SENSOR_IOCTL_WRITE_REG,
	SENSOR_IOCTL_READ_REG,
	SENSOR_IOCTL_CUS_FUNC_1,
	SENSOR_IOCTL_CUS_FUNC_2,
	SENSOR_IOCTL_AE_ENABLE,
	SENSOR_IOCTL_HMIRROR_ENABLE,
	SENSOR_IOCTL_VMIRROR_ENABLE,
	SENSOR_IOCTL_BRIGHTNESS,
	SENSOR_IOCTL_CONTRAST,
	SENSOR_IOCTL_SHARPNESS,
	SENSOR_IOCTL_SATURATION,
	SENSOR_IOCTL_PREVIEWMODE,
	SENSOR_IOCTL_IMAGE_EFFECT,
	SENSOR_IOCTL_BEFORE_SNAPSHOT,
	SENSOR_IOCTL_AFTER_SNAPSHOT,
	SENSOR_IOCTL_FLASH,
	SENSOR_IOCTL_READ_EV,
	SENSOR_IOCTL_WRITE_EV,
	SENSOR_IOCTL_READ_GAIN,
	SENSOR_IOCTL_WRITE_GAIN,
	SENSOR_IOCTL_READ_GAIN_SCALE,
	SENSOR_IOCTL_SET_FRAME_RATE,
	SENSOR_IOCTL_AF_ENABLE,
	SENSOR_IOCTL_AF_GET_STATUS,
	SENSOR_IOCTL_SET_WB_MODE,
	SENSOR_IOCTL_GET_SKIP_FRAME,
	SENSOR_IOCTL_ISO,
	SENSOR_IOCTL_EXPOSURE_COMPENSATION,
	SENSOR_IOCTL_CHECK_IMAGE_FORMAT_SUPPORT,
	SENSOR_IOCTL_CHANGE_IMAGE_FORMAT,
	SENSOR_IOCTL_ZOOM,
	SENSOR_IOCTL_CUS_FUNC_3,
	SENSOR_IOCTL_FOCUS,
	SENSOR_IOCTL_ANTI_BANDING_FLICKER,
	SENSOR_IOCTL_VIDEO_MODE,
	SENSOR_IOCTL_PICK_JPEG_STREAM,
	SENSOR_IOCTL_MAX
} SENSOR_IOCTL_CMD_E;

typedef enum {
	SENSOR_EXT_FOCUS_NONE = 0x00,
	SENSOR_EXT_FOCUS_TRIG,
	SENSOR_EXT_FOCUS_ZONE,
	SENSOR_EXT_FOCUS_MULTI_ZONE,
	SENSOR_EXT_FOCUS_MACRO,
	SENSOR_EXT_FOCUS_MAX
} SENSOR_EXT_FOCUS_CMD_E;

typedef enum {
	SENSOR_EXT_EXPOSURE_NONE = 0x00,
	SENSOR_EXT_EXPOSURE_AUTO,
	SENSOR_EXT_EXPOSURE_ZONE,
	SENSOR_EXT_EXPOSURE_MAX
} SENSOR_EXT_EXPOSURE_CMD_E;

typedef enum {
	SENSOR_EXT_FUNC_NONE = 0x00,
	SENSOR_EXT_FUNC_INIT,
	SENSOR_EXT_FOCUS_START,
	SENSOR_EXT_EXPOSURE_START,
	SENSOR_EXT_FUNC_MAX
} SENSOR_EXT_FUNC_CMD_E;

typedef enum {
	SENSOR_EXIF_CTRL_EXPOSURETIME = 0x00,
	SENSOR_EXIF_CTRL_FNUMBER,
	SENSOR_EXIF_CTRL_EXPOSUREPROGRAM,
	SENSOR_EXIF_CTRL_SPECTRALSENSITIVITY,
	SENSOR_EXIF_CTRL_ISOSPEEDRATINGS,
	SENSOR_EXIF_CTRL_OECF,
	SENSOR_EXIF_CTRL_SHUTTERSPEEDVALUE,
	SENSOR_EXIF_CTRL_APERTUREVALUE,
	SENSOR_EXIF_CTRL_BRIGHTNESSVALUE,
	SENSOR_EXIF_CTRL_EXPOSUREBIASVALUE,
	SENSOR_EXIF_CTRL_MAXAPERTUREVALUE,
	SENSOR_EXIF_CTRL_SUBJECTDISTANCE,
	SENSOR_EXIF_CTRL_METERINGMODE,
	SENSOR_EXIF_CTRL_LIGHTSOURCE,
	SENSOR_EXIF_CTRL_FLASH,
	SENSOR_EXIF_CTRL_FOCALLENGTH,
	SENSOR_EXIF_CTRL_SUBJECTAREA,
	SENSOR_EXIF_CTRL_FLASHENERGY,
	SENSOR_EXIF_CTRL_SPATIALFREQUENCYRESPONSE,
	SENSOR_EXIF_CTRL_FOCALPLANEXRESOLUTION,
	SENSOR_EXIF_CTRL_FOCALPLANEYRESOLUTION,
	SENSOR_EXIF_CTRL_FOCALPLANERESOLUTIONUNIT,
	SENSOR_EXIF_CTRL_SUBJECTLOCATION,
	SENSOR_EXIF_CTRL_EXPOSUREINDEX,
	SENSOR_EXIF_CTRL_SENSINGMETHOD,
	SENSOR_EXIF_CTRL_FILESOURCE,
	SENSOR_EXIF_CTRL_SCENETYPE,
	SENSOR_EXIF_CTRL_CFAPATTERN,
	SENSOR_EXIF_CTRL_CUSTOMRENDERED,
	SENSOR_EXIF_CTRL_EXPOSUREMODE,
	SENSOR_EXIF_CTRL_WHITEBALANCE,
	SENSOR_EXIF_CTRL_DIGITALZOOMRATIO,
	SENSOR_EXIF_CTRL_FOCALLENGTHIN35MMFILM,
	SENSOR_EXIF_CTRL_SCENECAPTURETYPE,
	SENSOR_EXIF_CTRL_GAINCONTROL,
	SENSOR_EXIF_CTRL_CONTRAST,
	SENSOR_EXIF_CTRL_SATURATION,
	SENSOR_EXIF_CTRL_SHARPNESS,
	SENSOR_EXIF_CTRL_DEVICESETTINGDESCRIPTION,
	SENSOR_EXIF_CTRL_SUBJECTDISTANCERANGE,
	SENSOR_EXIF_CTRL_MAX,
} SENSOR_EXIF_CTRL_E;

typedef enum {
	DCAMERA_ENVIRONMENT_NORMAL = 0x00,
	DCAMERA_ENVIRONMENT_NIGHT,
	DCAMERA_ENVIRONMENT_SUNNY,
	DCAMERA_ENVIRONMENT_SPORTS,
	DCAMERA_ENVIRONMENT_LANDSCAPE,
	DCAMERA_ENVIRONMENT_PORTRAIT,
	DCAMERA_ENVIRONMENT_PORTRAIT_NIGHT,
	DCAMERA_ENVIRONMENT_BACKLIGHT,
	DCAMERA_ENVIRONMENT_MACRO,
	DCAMERA_ENVIRONMENT_MANUAL = 30,
	DCAMERA_ENVIRONMENT_AUTO = 31,
	DCAMERA_ENVIRONMENT_MAX
} DCAMERA_PARAM_ENVIRONMENT_E;

typedef enum {
	DCAMERA_WB_MODE_AUTO = 0x00,
	DCAMERA_WB_MODE_INCANDESCENCE,
	DCAMERA_WB_MODE_U30,
	DCAMERA_WB_MODE_CWF,
	DCAMERA_WB_MODE_FLUORESCENT,
	DCAMERA_WB_MODE_SUN,
	DCAMERA_WB_MODE_CLOUD,
	DCAMERA_WB_MODE_MAX
} DCAMERA_PARAM_WB_MODE_E;

typedef enum {
	DCAMERA_EFFECT_NORMAL = 0x00,
	DCAMERA_EFFECT_BLACKWHITE,
	DCAMERA_EFFECT_RED,
	DCAMERA_EFFECT_GREEN,
	DCAMERA_EFFECT_BLUE,
	DCAMERA_EFFECT_YELLOW,
	DCAMERA_EFFECT_NEGATIVE,
	DCAMERA_EFFECT_CANVAS,
	DCAMERA_EFFECT_RELIEVOS,
	DCAMERA_EFFECT_MAX
} DCAMERA_PARAM_EFFECT_E;

struct sensor_pos{
	uint16_t x;
	uint16_t y;
};

struct sensor_size{
	uint16_t w;
	uint16_t h;
};

struct sensor_trim_size{
	uint16_t x;
	uint16_t y;
	uint16_t w;
	uint16_t h;
};

struct sensor_pos_rect{
	uint16_t start_x;
	uint16_t start_y;
	uint16_t end_x;
	uint16_t end_y;
};

struct sensor_rgb_gain{
	uint16_t r_gain;
	uint16_t g_gain;
	uint16_t b_gain;
};

struct sensor_blc_param{
	uint8_t mode;
	uint8_t reserved2;
	uint8_t reserved1;
	uint8_t reserved0;
	uint16_t r;
	uint16_t gr;
	uint16_t gb;
	uint16_t b;
};

struct sensor_nlc_param{
	uint16_t r_node[29];
	uint16_t g_node[29];
	uint16_t b_node[29];
	uint16_t l_node[29];
};

struct sensor_lnc_map_addr{
	uint32_t grid;
	uint32_t* param_addr;
	uint32_t max_index;
};

struct sensor_lnc_map{
	struct sensor_lnc_map_addr map[SENSOR_MAP_NUM][SENSOR_AWB_NUM];
};

struct sensor_ae_tab_info{
	uint16_t* e_ptr;
	uint16_t* g_ptr;
	uint16_t start_index;
	uint16_t max_index;
};

struct sensor_ae_param{
	uint8_t fix_fps;
	uint8_t target_lum;
	uint8_t target_zone;
	int8_t ev[16];
};

struct sensor_ae_tab{
	uint8_t* weight_tab;
	struct sensor_ae_tab_info tab[SENSOR_AE_TAB_NUM];
};

struct sensor_rgb{
	uint8_t r;
	uint8_t g;
	uint8_t b;
};

struct sensor_awb_param{
	struct sensor_pos win_start;
	struct sensor_size win_size;
	struct sensor_rgb cali[SENSOR_AWB_NUM];
	uint8_t cali_num;
	uint8_t reserved3;
	uint8_t reserved2;
	uint8_t reserved1;
	uint16_t r_gain[SENSOR_AWB_NUM];
	uint16_t g_gain[SENSOR_AWB_NUM];
	uint16_t b_gain[SENSOR_AWB_NUM];
	uint16_t reserved0;
	uint32_t matrix_zone;
	uint32_t cal_zone;
	uint32_t target_zone;
};

struct sensor_bpc_param{
	uint16_t flat_thr;
	uint16_t std_thr;
	uint16_t texture_thr;
	uint16_t reserved;
};

struct sensor_denoise_param{
	uint8_t write_back;
	uint8_t reserved4;
	uint8_t reserved3;
	uint8_t reserved2;
	uint16_t r_thr;
	uint16_t g_thr;
	uint16_t b_thr;
	uint16_t reserved;
	uint8_t diswei[19];
	uint8_t ranwei[31];
	uint8_t reserved1;
	uint8_t reserved0;
};

struct sensor_grgb_param{
	uint16_t edge_thr;
	uint16_t diff_thr;
};

struct sensor_cfa_param{
	uint16_t edge_thr;
	uint16_t diff_thr;
};

struct sensor_cmc_param{
	uint16_t matrix[SENSOR_CMC_NUM][9];
	uint16_t reserved;
};

struct sensor_gamma_param{
	uint16_t axis[26][2];
};

struct sensor_cce_uvdiv{
	uint8_t thrd[7];
	uint8_t reserved;
};

struct sensor_pref_param{
	uint8_t write_back;
	uint8_t y_thr;
	uint8_t u_thr;
	uint8_t v_thr;
};

struct sensor_bright_param{
	uint8_t factor[16];
};

struct sensor_contrast_param{
	uint8_t factor[16];
};

struct sensor_hist_param
{
	uint16_t low_ratio;
	uint16_t high_ratio;
	uint8_t in_min;
	uint8_t in_max;
	uint8_t out_min;
	uint8_t out_max;
	uint8_t mode;
	uint8_t reserved2;
	uint8_t reserved1;
	uint8_t reserved0;
};

struct sensor_auto_contrast_param{
	uint8_t mode;
	uint8_t reserved2;
	uint8_t reserved1;
	uint8_t reserved0;
};

struct sensor_saturation_param{
	uint8_t factor[16];
};

struct sensor_af_param{
	struct sensor_pos_rect win[9];
};

struct sensor_emboss_param{
	uint8_t step;
};

struct sensor_edge_param{
	uint8_t detail_thr;
	uint8_t smooth_thr;
	uint8_t strength;
	uint8_t reserved;
};

struct sensor_global_gain_param{
	uint32_t gain;
};

struct sensor_chn_gain_param{
	uint8_t r_gain;
	uint8_t g_gain;
	uint8_t b_gain;
	uint8_t reserved0;
	uint16_t r_offset;
	uint16_t g_offset;
	uint16_t b_offset;
	uint16_t reserved1;
};

struct sensor_css_param{
	uint8_t low_thr[7];
	uint8_t lum_thr;
	uint8_t low_sum_thr[7];
	uint8_t chr_thr;
};

struct sensor_version_info{
	uint32_t version_id;
	uint32_t srtuct_size;
	uint32_t reserve;
};

struct sensor_raw_tune_info{
	uint32_t blc_bypass;
	uint32_t nlc_bypass;
	uint32_t lnc_bypass;
	uint32_t ae_bypass;
	uint32_t awb_bypass;
	uint32_t bpc_bypass;
	uint32_t denoise_bypass;
	uint32_t grgb_bypass;
	uint32_t cmc_bypass;
	uint32_t gamma_bypass;
	uint32_t uvdiv_bypass;
	uint32_t pref_bypass;
	uint32_t bright_bypass;
	uint32_t contrast_bypass;
	uint32_t hist_bypass;
	uint32_t auto_contrast_bypass;
	uint32_t af_bypass;
	uint32_t edge_bypass;
	uint32_t fcs_bypass;
	uint32_t css_bypass;
	uint32_t saturation_bypass;
	uint32_t hdr_bypass;
	uint32_t glb_gain_bypass;
	uint32_t chn_gain_bypass;

	struct sensor_blc_param blc;
	struct sensor_nlc_param nlc;
	struct sensor_ae_param ae;
	struct sensor_awb_param awb;
	struct sensor_bpc_param bpc;
	struct sensor_denoise_param denoise;
	struct sensor_grgb_param grgb;
	struct sensor_cfa_param cfa;
	struct sensor_cmc_param cmc;
	struct sensor_gamma_param gamma;
	struct sensor_cce_uvdiv uv_div;
	struct sensor_pref_param pref;
	struct sensor_bright_param bright;
	struct sensor_contrast_param contrast;
	struct sensor_hist_param hist;
	struct sensor_auto_contrast_param auto_contrast;
	struct sensor_saturation_param saturation;
	struct sensor_css_param css;
	struct sensor_af_param af;
	struct sensor_edge_param edge;
	struct sensor_emboss_param emboss;
	struct sensor_global_gain_param global;
	struct sensor_chn_gain_param chn;
};

struct sensor_raw_fix_info{
	struct sensor_ae_tab ae;
	struct sensor_lnc_map lnc;
};

struct sensor_raw_info{
	struct sensor_version_info* version_info;
	struct sensor_raw_tune_info* tune_ptr;
	struct sensor_raw_fix_info* fix_ptr;
};

typedef uint32_t(*SENSOR_IOCTL_FUNC_PTR) (uint32_t param);

typedef struct sensor_ioctl_func_tab_tag {
	/*1: Internal IOCTL function */
	uint32_t(*reset) (uint32_t param);
	uint32_t(*power) (uint32_t param);
	uint32_t(*enter_sleep) (uint32_t param);
	uint32_t(*identify) (uint32_t param);
	uint32_t(*write_reg) (uint32_t param);
	uint32_t(*read_reg) (uint32_t param);
	/*Custom function */
	uint32_t(*cus_func_1) (uint32_t param);
	uint32_t(*get_trim) (uint32_t param);
	/*External IOCTL function */
	uint32_t(*ae_enable) (uint32_t param);
	uint32_t(*hmirror_enable) (uint32_t param);
	uint32_t(*vmirror_enable) (uint32_t param);

	uint32_t(*set_brightness) (uint32_t param);
	uint32_t(*set_contrast) (uint32_t param);
	uint32_t(*set_sharpness) (uint32_t param);
	uint32_t(*set_saturation) (uint32_t param);
	uint32_t(*set_preview_mode) (uint32_t param);

	uint32_t(*set_image_effect) (uint32_t param);
	uint32_t(*before_snapshort) (uint32_t param);
	uint32_t(*after_snapshort) (uint32_t param);
	uint32_t(*flash) (uint32_t param);
	uint32_t(*read_ae_value) (uint32_t param);
	uint32_t(*write_ae_value) (uint32_t param);
	uint32_t(*read_gain_value) (uint32_t param);
	uint32_t(*write_gain_value) (uint32_t param);
	uint32_t(*read_gain_scale) (uint32_t param);
	uint32_t(*set_frame_rate) (uint32_t param);
	uint32_t(*af_enable) (uint32_t param);
	uint32_t(*af_get_status) (uint32_t param);
	uint32_t(*set_wb_mode) (uint32_t param);
	uint32_t(*get_skip_frame) (uint32_t param);
	uint32_t(*set_iso) (uint32_t param);
	uint32_t(*set_exposure_compensation) (uint32_t param);
	uint32_t(*check_image_format_support) (uint32_t param);
	uint32_t(*change_image_format) (uint32_t param);
	uint32_t(*set_zoom) (uint32_t param);
	/*CUSTOMER FUNCTION */
	uint32_t(*get_exif) (uint32_t param);
	uint32_t(*set_focus) (uint32_t param);
	uint32_t(*set_anti_banding_flicker) (uint32_t param);
	uint32_t(*set_video_mode) (uint32_t param);
	uint32_t(*pick_jpeg_stream) (uint32_t param);
	uint32_t(*set_meter_mode) (uint32_t param);
} SENSOR_IOCTL_FUNC_TAB_T, *SENSOR_IOCTL_FUNC_TAB_T_PTR;

typedef struct sensor_reg_tag {
	uint16_t reg_addr;
	uint16_t reg_value;
} SENSOR_REG_T, *SENSOR_REG_T_PTR;

typedef struct sensor_reg_bits_tag {
	uint16_t reg_addr;
	uint16_t reg_value;
	uint32_t reg_bits;
} SENSOR_REG_BITS_T, *SENSOR_REG_BITS_T_PTR;
typedef struct sensor_trim_tag {
	uint16_t trim_start_x;
	uint16_t trim_start_y;
	uint16_t trim_width;
	uint16_t trim_height;
	uint32_t line_time;
	uint32_t pclk;
} SENSOR_TRIM_T, *SENSOR_TRIM_T_PTR;

typedef struct _sensor_rect_tag {
	uint16_t x;
	uint16_t y;
	uint16_t w;
	uint16_t h;
} SENSOR_RECT_T, *SENSOR_RECT_T_PTR;

typedef struct _sensor_ext_fun_tag {
	uint32_t cmd;
	uint32_t param;
	SENSOR_RECT_T zone;
} SENSOR_EXT_FUN_T, *SENSOR_EXT_FUN_T_PTR;
typedef struct _sensor_ext_fun_param_tag {
	uint8_t cmd;
	uint8_t param;
	uint16_t zone_cnt;
	SENSOR_RECT_T zone[FOCUS_ZONE_CNT_MAX];
} SENSOR_EXT_FUN_PARAM_T, *SENSOR_EXT_FUN_PARAM_T_PTR;

typedef struct sensor_reg_tab_info_tag {
	SENSOR_REG_T_PTR sensor_reg_tab_ptr;
	uint32_t reg_count;
	uint16_t width;
	uint16_t height;
	uint32_t xclk_to_sensor;
	SENSOR_IMAGE_FORMAT image_format;

} SENSOR_REG_TAB_INFO_T, *SENSOR_REG_TAB_INFO_T_PTR;

typedef struct sensor_mode_info_tag {
	SENSOR_MODE_E mode;
	uint16_t width;
	uint16_t height;
	uint16_t trim_start_x;
	uint16_t trim_start_y;
	uint16_t trim_width;
	uint16_t trim_height;
	uint32_t line_time;
	SENSOR_IMAGE_FORMAT image_format;
} SENSOR_MODE_INFO_T, *SENSOR_MODE_INFO_T_PTR;

typedef struct sensor_extend_info_tag {
	uint32_t focus_mode;
	uint32_t exposure_mode;
} SENSOR_EXTEND_INFO_T, *SENSOR_EXTEND_INFO_T_PTR;

typedef struct sensor_register_tag {
	uint32_t img_sensor_num;
	uint8_t cur_id;
	uint8_t is_register[SENSOR_ID_MAX];
} SENSOR_REGISTER_INFO_T, *SENSOR_REGISTER_INFO_T_PTR;

typedef struct sensor_exp_info_tag {
	SENSOR_IMAGE_FORMAT image_format;
	uint32_t image_pattern;
	uint8_t pclk_polarity;
	uint8_t vsync_polarity;
	uint8_t hsync_polarity;
	uint8_t pclk_delay;
	uint16_t source_width_max;
	uint16_t source_height_max;
	uint32_t environment_mode;
	uint32_t image_effect;
	uint32_t wb_mode;
	uint32_t step_count;	/*bit[0:7]: count of step in brightness, contrast, sharpness, saturation
				   bit[8:15] count of step in ISO
				   bit[16:23] count of step in exposure compensation
				   bit[24:31] reseved */
	SENSOR_MODE_INFO_T sensor_mode_info[SENSOR_MODE_MAX];
	SENSOR_IOCTL_FUNC_TAB_T_PTR ioctl_func_ptr;
	struct sensor_raw_info* raw_info_ptr;
	SENSOR_EXTEND_INFO_T_PTR ext_info_ptr;
	uint32_t preview_skip_num;
	uint32_t capture_skip_num;
	uint32_t preview_deci_num;
	uint32_t video_preview_deci_num;
	uint16_t threshold_eb;
	uint16_t threshold_mode;
	uint16_t threshold_start;
	uint16_t threshold_end;
	SENSOR_INTERFACE_E sensor_interface;
} SENSOR_EXP_INFO_T, *SENSOR_EXP_INFO_T_PTR;

typedef struct sensor_info_tag {
	uint8_t salve_i2c_addr_w;
	uint8_t salve_i2c_addr_r;
	uint8_t reg_addr_value_bits;
	uint8_t hw_signal_polarity;
	uint32_t environment_mode;
	uint32_t image_effect;
	uint32_t wb_mode;
	uint32_t step_count;
	uint16_t reset_pulse_level;
	uint16_t reset_pulse_width;
	uint32_t power_down_level;
	uint32_t identify_count;
	SENSOR_REG_T identify_code[SENSOR_IDENTIFY_CODE_COUNT];
	SENSOR_AVDD_VAL_E avdd_val;
	uint16_t source_width_max;
	uint16_t source_height_max;
	const char *name;
	SENSOR_IMAGE_FORMAT image_format;
	uint32_t image_pattern;
	SENSOR_REG_TAB_INFO_T_PTR resolution_tab_info_ptr;
	SENSOR_IOCTL_FUNC_TAB_T_PTR ioctl_func_tab_ptr;
	struct sensor_raw_info*  raw_info_ptr;
	SENSOR_EXTEND_INFO_T_PTR ext_info_ptr;
	SENSOR_AVDD_VAL_E iovdd_val;
	SENSOR_AVDD_VAL_E dvdd_val;
	uint32_t preview_skip_num;
	uint32_t capture_skip_num;
	uint32_t preview_deci_num;
	uint32_t video_preview_deci_num;
	uint16_t threshold_eb;
	uint16_t threshold_mode;
	uint16_t threshold_start;
	uint16_t threshold_end;
	int32_t i2c_dev_handler;
	SENSOR_INTERFACE_E sensor_interface;
} SENSOR_INFO_T;

typedef enum {
	SENSOR_PARAM_WB_MODE_AUTO = 0x00,
	SENSOR_PARAM_WB_MODE_INCANDESCENCE,
	SENSOR_PARAM_WB_MODE_U30,
	SENSOR_PARAM_WB_MODE_CWF,
	SENSOR_PARAM_WB_MODE_FLUORESCENT,
	SENSOR_PARAM_WB_MODE_SUN,
	SENSOR_PARAM_WB_MODE_CLOUD,
	SENSOR_PARAM_WB_MODE_MAX
} SENSOR_PARAM_WB_MODE_E;

typedef enum {
	SENSOR_PARAM_ENVIRONMENT_NORMAL = 0x00,
	SENSOR_PARAM_ENVIRONMENT_NIGHT,
	SENSOR_PARAM_ENVIRONMENT_SUNNY,
	SENSOR_PARAM_ENVIRONMENT_SPORTS,
	SENSOR_PARAM_ENVIRONMENT_LANDSCAPE,
	SENSOR_PARAM_ENVIRONMENT_PORTRAIT,
	SENSOR_PARAM_ENVIRONMENT_PORTRAIT_NIGHT,
	SENSOR_PARAM_ENVIRONMENT_BACKLIGHT,
	SENSOR_PARAM_ENVIRONMENT_MACRO,
	SENSOR_PARAM_ENVIRONMENT_MANUAL = 30,
	SENSOR_PARAM_ENVIRONMENT_AUTO = 31,
	SENSOR_PARAM_ENVIRONMENT_MAX
} SENSOR_PARAM_ENVIRONMENT_E;

typedef enum {
	SENSOR_PARAM_EFFECT_NORMAL = 0x00,
	SENSOR_PARAM_EFFECT_BLACKWHITE,
	SENSOR_PARAM_EFFECT_RED,
	SENSOR_PARAM_EFFECT_GREEN,
	SENSOR_PARAM_EFFECT_BLUE,
	SENSOR_PARAM_EFFECT_YELLOW,
	SENSOR_PARAM_EFFECT_NEGATIVE,
	SENSOR_PARAM_EFFECT_CANVAS,
	SENSOR_PARAM_EFFECT_RELIEVOS,
	SENSOR_PARAM_EFFECT_MAX
} SENSOR_PARAM_EFFECT_E;


int Sensor_Init(void);
int Sensor_WriteRegs(uint8_t *regPtr, uint32_t length);
int Sensor_Open(uint32_t sensor_id);
SENSOR_EXP_INFO_T *Sensor_GetInfo(void);
int Sensor_SetMode(uint32_t mode);
int32_t Sensor_WriteReg(uint16_t subaddr, uint16_t data);
uint16_t Sensor_ReadReg(uint16_t subaddr);
int Sensor_SetMCLK(uint32_t mclk);
int Sensor_SetVoltage(SENSOR_AVDD_VAL_E dvdd_val,
		       SENSOR_AVDD_VAL_E avdd_val, SENSOR_AVDD_VAL_E iodd_val);  // todo splite 3 fun

BOOLEAN Sensor_PowerDown(BOOLEAN power_down);
BOOLEAN Sensor_SetResetLevel(BOOLEAN plus_level);
void Sensor_Reset(uint32_t level);
uint32_t Sensor_Ioctl(uint32_t cmd, uint32_t arg);
uint32_t Sensor_SetSensorExifInfo(SENSOR_EXIF_CTRL_E cmd, uint32_t param);
EXIF_SPEC_PIC_TAKING_COND_T *Sensor_GetSensorExifInfo(void);
int Sensor_EventReg(cmr_evt_cb  event_cb);




int32_t Sensor_WriteReg_8bits(uint16_t reg_addr, uint8_t value);
int32_t Sensor_ReadReg_8bits(uint8_t reg_addr, uint8_t * reg_val);
ERR_SENSOR_E Sensor_SendRegTabToSensor(SENSOR_REG_TAB_INFO_T *
				       sensor_reg_tab_info_ptr);
BOOLEAN Sensor_IsInit(void);


ERR_SENSOR_E Sensor_Close(void);


BOOLEAN Sensor_IsOpen(void);
uint32_t Sensor_SetCurId(SENSOR_ID_E sensor_id);
SENSOR_ID_E Sensor_GetCurId(void);
SENSOR_REGISTER_INFO_T_PTR Sensor_GetRegisterInfo(void);
uint32_t Sensor_SetSensorType(SENSOR_TYPE_E sensor_type);
void ImgSensor_DeleteMutex(void);
void ImgSensor_GetMutex(void);
void ImgSensor_GetMutex(void);
void ImgSensor_PutMutex(void);
uint32_t Sensor_SetFlash(uint32_t is_open);
struct i2c_client *Sensor_GetI2CClien(void);
int Sensor_SetSensorParam(uint8_t *buf);
int Sensor_GetSensorParam(uint8_t *buf,uint8_t *is_saved_ptr);
int Sensor_GetRawSettings(void **raw_setting, uint32_t *length);


#endif
