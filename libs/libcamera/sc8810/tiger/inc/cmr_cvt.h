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
#ifndef _CMR_CVT_H_
#define _CMR_CVT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "cmr_common.h"

enum cmr_img_cvt_evt {
	CMR_IMG_CVT_ROT_DONE = CMR_EVT_CVT_BASE,
	CMR_IMG_CVT_SC_DONE,
};

//#define ROT_IO_IS_DONE             0xEE000000
#define SCALER_IS_DONE             0xFF000000

int cmr_rot_init(void);

int cmr_rot_evt_reg(cmr_evt_cb  rot_event_cb);

int cmr_rot(enum img_rot_angle  angle,
		struct img_frm  *src_img,
		struct img_rect *trim,
		struct img_frm  *dst_img,
		void            *user_data);

int cmr_rot_cpy(struct img_frm  *src_img,
			struct img_frm  *dst_img);

int cmr_rot_deinit(void);

int cmr_scale_init(void);

int cmr_scale_evt_reg(cmr_evt_cb  scale_event_cb);

int  cmr_scale_start(uint32_t slice_height,
		struct img_frm *src_img,
		struct img_rect *rect,
		struct img_frm *dst_img,
		void           *tmp_buf,
		void           *user_data);

int  cmr_scale_next(uint32_t     slice_height,
		struct img_addr  *src_addr, 
		struct img_rect  *rect,
		struct img_addr  *dst_addr);

int cmr_scale_deinit(void);
int cmr_scale_capability(uint32_t *width);

#ifdef __cplusplus
}
#endif

#endif
