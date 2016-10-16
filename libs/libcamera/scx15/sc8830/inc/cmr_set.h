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

#ifndef CMR_SET_H
#define CMR_SET_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "SprdOEMCamera.h"
#include "cmr_oem.h"

#define INVALID_SET_BYTE                              0xFF
#define INVALID_SET_WORD                              0xFFFFFFFF
#define SCENE_MODE_NIGHT                              1
#define ISP_AE_STAB_TIMEOUT                           5 /*sec*/
#define ISP_ALG_TIMEOUT                               5 /*sec*/
#define EN_WAIT_AE_STAB                               1

enum cmr_focus_mode {
	CAMERA_FOCUS_MODE_AUTO = 0,
	CAMERA_FOCUS_MODE_AUTO_MULTI = 1,
	CAMERA_FOCUS_MODE_MACRO = 2,
	CAMERA_FOCUS_MODE_INFINITY = 3,
	CAMERA_FOCUS_MODE_CAF = 4,
	CAMERA_FOCUS_MODE_CAF_VIDEO = 5,
	CAMERA_FOCUS_MODE_MAX
};

enum cmr_flash_mode {
	CAMERA_FLASH_MODE_OFF = 0,
	CAMERA_FLASH_MODE_ON = 1,
	CAMERA_FLASH_MODE_TORCH = 2,
	CAMERA_FLASH_MODE_AUTO = 3,
	CAMERA_FLASH_MODE_MAX
};

enum cmr_flash_status {
	FLASH_CLOSE = 0x0,
	FLASH_OPEN = 0x1,
	FLASH_TORCH = 0x2,/*user only set flash to close/open/torch state */
	FLASH_AUTO = 0x3,
	FLASH_CLOSE_AFTER_OPEN = 0x10,/* following is set to sensor */
	FLASH_HIGH_LIGHT = 0x11,
	FLASH_OPEN_ON_RECORDING = 0x22,
	FLASH_CLOSE_AFTER_AUTOFOCUS = 0x30,
	FLASH_STATUS_MAX
};

int camera_setting_init(void);
int camera_setting_deinit(void);
int camera_skip_frame_cb(uint32_t rtn,void *param_ptr);
int camera_proc_start_cb(uint32_t rtn,void *param_ptr);
int camera_preview_start_set(void);
int camera_preview_stop_set(void);
int camera_snapshot_start_set(void);
int camera_snapshot_stop_set(void);
int camera_autofocus_init(void);
int camera_autofocus(void);
int camera_autofocus_start(void);
int camera_autofocus_stop(uint32_t is_external);
int camera_autofocus_start_light(void);
int camera_set_ctrl(camera_parm_type id,
			uint32_t parm,
			cmr_before_set_cb before_set,
			cmr_after_set_cb  after_set);
int camera_isp_ctrl_done(uint32_t cmd, void* data);
int camera_isp_af_done(void *data);
int camera_set_hdr_ev(int ev_level);
int camera_set_flashdevice(uint32_t param);
int camera_preflash(void);
int camera_caf_preflash(void);
int camera_ae_enable(uint32_t param);
int camera_isp_alg_done(void *data);
int camera_isp_af_stat(void* data);
int camera_isp_ae_stab(void* data);
int camera_autofocus_quit(void);
int camera_autofocus_need_exit(uint32_t *is_external);
int camera_set_focusmove_flag(uint32_t is_done);
uint32_t camera_is_focusmove_done(void);
int camera_isp_alg_wait(void);
int camera_get_af_mode(void);
int camera_set_flash_hightlight(uint32_t flash_mode);
#ifdef __cplusplus
}
#endif

#endif
