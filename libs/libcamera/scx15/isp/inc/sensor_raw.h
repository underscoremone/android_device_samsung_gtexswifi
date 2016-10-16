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
#define SENSOR_ISO_NUM 0x06
#define SENSOR_AE_NUM 0x500
#define SENSOR_AE_TAB_NUM 0x04
#define SENSOR_AWB_CALI_NUM 0x09
#define SENSOR_AWB_NUM 0x14
#define SENSOR_MAP_NUM 0x08
#define SENSOR_AWB_G_ESTIMATE_NUM 0x6
#define SENSOR_AWB_GAIN_ADJUST_NUM 0x9
#define SENSOR_AWB_LIGHT_NUM 0x10

#define RAW_INFO_END_ID 0x71717567

#define SENSOR_RAW_VERSION_ID 0x00000000 /* tiger-0000xxxx, soft-xxxx0000*/
#define SENSOR_RAW_V0001_VERSION_ID 0x00010000 /* tiger-0000xxxx, soft-xxxx0000*/

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

struct sensor_blc_offset{
	uint16_t r;
	uint16_t gr;
	uint16_t gb;
	uint16_t b;
};

struct sensor_blc_param{
	uint8_t mode;
	uint8_t reserved2;
	uint8_t reserved1;
	uint8_t reserved0;
	struct sensor_blc_offset offset[8];
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
	struct sensor_lnc_map_addr map[SENSOR_MAP_NUM][SENSOR_AWB_CALI_NUM];
};

struct sensor_awb_map{
	uint16_t *addr;
	uint32_t len;		//by bytes
};

struct sensor_ae_index{
	uint16_t start;
	uint16_t max;
};

struct sensor_ae_tab_info{
	uint32_t* e_ptr;
	uint16_t* g_ptr;
	struct sensor_ae_index index[SENSOR_ISO_NUM];
};

struct sensor_ae_histogram_segment {
	uint8_t		min;
	uint8_t		max;
	uint16_t	weight;
};

struct sensor_ae_change_speed {
	uint16_t	delta_lum;
	uint16_t	delta_ev;
};

struct sensor_ae_param{
	uint8_t skip_frame;
	uint8_t normal_fix_fps;
	uint8_t night_fix_fps;
	uint8_t video_fps;
	uint8_t target_lum;
	uint8_t target_zone;
	uint8_t smart;
	uint8_t smart_rotio;
	uint8_t quick_mode;
	uint8_t min_exposure;
	int8_t ev[16];
	uint16_t smart_base_gain;
	uint16_t smart_wave_min;
	uint16_t smart_wave_max;
	uint8_t smart_pref_min;
	uint8_t smart_pref_max;
	uint8_t smart_denoise_min_index;
	uint8_t smart_denoise_max_index;
	uint8_t smart_edge_min_index;
	uint8_t smart_edge_max_index;
	uint8_t smart_mode;
	uint8_t smart_sta_low_thr;
	uint8_t smart_sta_ratio1;
	uint8_t smart_sta_ratio;
	uint16_t smart_sta_start_index;
	uint8_t again_skip;
	uint8_t dgain_skip;
	uint8_t gamma_start;
	uint8_t gamma_num;
	uint8_t gamma_zone;
	uint8_t gamma_thr[5];
	uint16_t lux_500_index;
	uint32_t reserved[33];
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

struct sensor_cali_info {
	uint32_t r_sum;
	uint32_t gr_sum;
	uint32_t gb_sum;
	uint32_t b_sum;
};

struct sensor_awb_g_estimate_param
{
	uint16_t t_thr[SENSOR_AWB_G_ESTIMATE_NUM];
	uint16_t g_thr[SENSOR_AWB_G_ESTIMATE_NUM][2];
	uint16_t w_thr[SENSOR_AWB_G_ESTIMATE_NUM][2];
	uint32_t num;
};

struct sensor_awb_linear_func
{
	int32_t a;
	int32_t b;
	uint32_t shift;
};

struct sensor_awb_wp_count_range
{
	uint16_t min_proportion;//min_proportion / 256
	uint16_t max_proportion;//max_proportion / 256
};

struct sensor_awb_gain_adjust
{
	uint16_t t_thr[SENSOR_AWB_GAIN_ADJUST_NUM];
	uint16_t w_thr[SENSOR_AWB_GAIN_ADJUST_NUM];
	uint32_t num;
};

struct sensor_awb_light_weight
{
	uint16_t t_thr[SENSOR_AWB_LIGHT_NUM];
	uint16_t w_thr[SENSOR_AWB_LIGHT_NUM];
	uint16_t num;
};

struct sensor_awb_param{
	struct sensor_pos win_start;
	struct sensor_size win_size;
	struct sensor_rgb gain_convert[8];
	struct sensor_awb_coord win[SENSOR_AWB_NUM];
	struct sensor_awb_light_weight light;
	uint32_t steady_speed;
	uint16_t r_gain[SENSOR_AWB_NUM];
	uint16_t g_gain[SENSOR_AWB_NUM];
	uint16_t b_gain[SENSOR_AWB_NUM];
	uint8_t gain_index;
	uint8_t alg_id;
	uint32_t target_zone;
	uint32_t smart;
	uint32_t quick_mode;
	struct sensor_awb_wp_count_range wp_count_range;
	struct sensor_awb_g_estimate_param g_estimate;
	struct sensor_awb_linear_func t_func;
	struct sensor_awb_gain_adjust gain_adjust;
	uint8_t debug_level;
	uint8_t smart_index;
	uint8_t skip_num;
	uint8_t reserved0;
	uint32_t reserved[9];
};

struct sensor_bpc_param{
	uint16_t flat_thr;
	uint16_t std_thr;
	uint16_t texture_thr;
	uint16_t reserved;
};

struct sensor_denoise_tab {
	uint8_t diswei[19];
	uint8_t reserved1;
	uint8_t ranwei[31];
	uint8_t reserved0;
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
	struct sensor_denoise_tab tab[1];
	uint32_t reserved5[57];
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
	uint16_t matrix[SENSOR_AWB_CALI_NUM][9];
	uint16_t reserved;
};

struct sensor_cce_parm{
	uint16_t matrix[9];
	uint16_t y_shift;
	uint16_t u_shift;
	uint16_t v_shift;
};

struct sensor_gamma_tab{
	uint16_t axis[2][26];
};

struct sensor_gamma_param{
	uint16_t axis[2][26];
	struct sensor_gamma_tab tab[5];
};

struct sensor_cce_uvdiv{
	uint8_t thrd[7];
	uint8_t t[2];
	uint8_t m[3];
	uint32_t reserved1[19];
};

struct sensor_pref_param{
	uint8_t write_back;
	uint8_t y_thr;
	uint8_t u_thr;
	uint8_t v_thr;
	uint32_t reserved[20];
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
	uint16_t min_step;
	uint16_t max_tune_step;
	uint32_t stab_period;
	uint16_t af_rough_step[32];
	uint16_t af_fine_step[32];
	uint8_t rough_count;
	uint8_t fine_count;
	uint8_t alg_id;
	uint8_t debug;
	uint16_t default_step_len;
	uint8_t peak_thr_0;
	uint8_t peak_thr_1;
	uint8_t peak_thr_2;
	uint8_t detect_thr;
	uint8_t detect_step_mum;
	uint8_t start_area_range;
	uint8_t end_area_range;
	uint8_t noise_thr;
	uint16_t reserved0;
	uint32_t reserved[4];
};

struct sensor_emboss_param{
	uint8_t step;
	uint8_t reserved2;
	uint8_t reserved1;
	uint8_t reserved0;
};

struct sensor_edge_param{
	uint8_t detail_thr;
	uint8_t smooth_thr;
	uint8_t strength;
	uint8_t reserved;
};

struct sensor_edge_info{
	struct sensor_edge_param info[16];
};

struct sensor_global_gain_param{
	uint32_t gain;
	uint32_t reserved[20];
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
	uint32_t reserved2[50];
};

struct sensor_flash_cali_param{
	uint16_t lum_ratio;
	uint16_t r_ratio;
	uint16_t g_ratio;
	uint16_t b_ratio;
	uint32_t reserved[50];
};

struct sensor_css_param{
	uint8_t low_thr[7];
	uint8_t lum_thr;
	uint8_t low_sum_thr[7];
	uint8_t chr_thr;
	uint32_t reserved[70];
};

struct sensor_version_info{
	uint32_t version_id;
	uint32_t srtuct_size;
	uint32_t reserve;
};

struct sensor_raw_tune_info{
	uint32_t version_id;
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
	uint32_t reserve9;
	uint32_t reserve8;
	uint32_t reserve7;
	uint32_t reserve6;
	uint32_t reserve5;
	uint32_t reserve4;
	uint32_t reserve3;
	uint32_t reserve2;
	uint32_t reserve1;
	uint32_t reserve0;

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
	struct sensor_edge_info edge;
	struct sensor_emboss_param emboss;
	struct sensor_global_gain_param global;
	struct sensor_chn_gain_param chn;
	struct sensor_flash_cali_param flash;
	struct sensor_cce_parm special_effect[16];
	uint32_t reserved[256];
};

struct sensor_raw_fix_info{
	struct sensor_ae_tab ae;
	struct sensor_lnc_map lnc;
	struct sensor_awb_map awb;
};

struct sensor_raw_awb_cali{
	struct sensor_cali_info cali_info;
	struct sensor_cali_info golden_cali_info;
};

struct sensor_raw_flashlight_cali{
	struct sensor_cali_info cali_info;
	struct sensor_cali_info golden_cali_info;
};


struct sensor_raw_ae_cali{
	struct sensor_ae_change_speed *speed_dark_to_bright;
	uint32_t			step_dark_to_bright;
	struct sensor_ae_change_speed *speed_bright_to_dark;
	uint32_t			step_bright_to_dark;
	struct sensor_ae_histogram_segment *histogram_segment;
	uint32_t			histogram_segment_num;
};

struct sensor_raw_cali_info{
	struct sensor_raw_ae_cali ae;
	struct sensor_raw_awb_cali awb;
	struct sensor_raw_flashlight_cali flashlight;
};

struct sensor_raw_resolution_info {
	uint16_t start_x;
	uint16_t start_y;
	uint16_t width;
	uint16_t height;
	uint16_t line_time;
	uint16_t frame_line;
};

struct sensor_raw_resolution_info_tab {
	uint32_t image_pattern;
	struct sensor_raw_resolution_info tab[10];
};

struct sensor_raw_ioctrl {
	uint32_t(*set_focus) (uint32_t param);
	uint32_t(*set_exposure) (uint32_t param);
	uint32_t(*set_gain) (uint32_t param);
};

struct sensor_raw_info {
	struct sensor_version_info* version_info;
	struct sensor_raw_tune_info* tune_ptr;
	struct sensor_raw_fix_info* fix_ptr;
	struct sensor_raw_cali_info* cali_ptr;
	struct sensor_raw_resolution_info_tab* resolution_info_ptr;
	struct sensor_raw_ioctrl* ioctrl_ptr;
};

struct raw_param_info_tab {
	uint32_t param_id;
	struct sensor_raw_info* info_ptr;
	uint32_t(*identify_otp) (void* param_ptr);
	uint32_t(*cfg_otp) (void* param_ptr);
};

#endif
