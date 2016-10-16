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
int camera_autofocus_stop(void);
int camera_set_ctrl(camera_parm_type id,
			uint32_t          parm,
			cmr_before_set_cb before_set,
			cmr_after_set_cb  after_set);
int camera_isp_ctrl_done(uint32_t cmd, void* data);
int camera_isp_af_done(void *data);
int camera_set_hdr_ev(int ev_level);



#ifdef __cplusplus
}
#endif

#endif
