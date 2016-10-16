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
#ifndef _SENSOR_RAW_H_
#define _SENSOR_RAW_H_

//#include "jpeg_exif_header.h"
//#include "cmr_common.h"
#include <sys/types.h>
//#include "sensor_drv_u.h"

#define MAX_AWB_MOD 8
#define MAX_EV_LEVEL 15
#define MAX_BRIGHTNESS_LEVEL 15
#define MAX_CONTRAST_LEVEL 15
#define MAX_HUE_LEVEL 15

/*
//normal: conversion/emboss/gamma
//grey: color conversion;
//sepia: color conversion;
//emboss: emboss;
//negative: color conversion;
//over exposure: gamma;
//red: color conversion;
//green: color conversion
//blue: color conversion;
//yellow: color conversion;
*/
/* Incandescent/U30/CWF/Fluorescent/Sun/Cloudy*/
#define SENSOR_ISO_NUM 0x05
#define SENSOR_AE_NUM 0x500
#define SENSOR_AE_TAB_NUM 0x04
#define SENSOR_CMC_NUM 0x09
#define SENSOR_AWB_NUM 0x14
#define SENSOR_MAP_NUM 0x08

#define SENSOR_RAW_VERSION_ID 0x0000 /* tiger-00xx, soft-xx00*/

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
	uint16_t* param_addr;
	uint32_t len;
};

struct sensor_lnc_map{
	struct sensor_lnc_map_addr map[SENSOR_MAP_NUM][9];
};

struct sensor_ae_index{
	uint16_t start;
	uint16_t max;
};

struct sensor_ae_tab_info{
	uint16_t* e_ptr;
	uint16_t* g_ptr;
	struct sensor_ae_index index[SENSOR_ISO_NUM];
};

struct sensor_ae_param{
	uint8_t skip_frame;
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
	uint16_t r;
	uint16_t g;
	uint16_t b;
};

struct sensor_awb_coord{
	uint16_t x;
	uint16_t yb;
	uint16_t yt;
};

struct sensor_awb_param{
	struct sensor_pos win_start;
	struct sensor_size win_size;
	struct sensor_awb_coord win[SENSOR_AWB_NUM];
	struct sensor_awb_coord cali[SENSOR_CMC_NUM];
	uint32_t cali_num;
	uint16_t r_gain[SENSOR_AWB_NUM];
	uint16_t g_gain[SENSOR_AWB_NUM];
	uint16_t b_gain[SENSOR_AWB_NUM];
	uint16_t reserved0;
	uint32_t cali_zone;
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

struct sensor_cce_parm{
	uint16_t matrix[9];
	uint16_t y_shift;
	uint16_t u_shift;
	uint16_t v_shift;
};

struct sensor_gamma_param{
	uint16_t axis[2][26];
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
	uint32_t max_step;
	uint32_t stab_period;
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
	struct sensor_cce_parm cce;
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




#endif
