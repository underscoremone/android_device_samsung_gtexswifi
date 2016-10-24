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
#ifndef _CMR_MEM_H_
#define _CMR_MEM_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "cmr_common.h"


struct cmr_cap_2_frm {
	struct img_frm  major_frm;
	struct img_frm  minor_frm;
};

struct cmr_cap_mem {
	struct img_frm  cap_raw;
	struct img_frm  cap_yuv;
	struct img_frm  target_yuv;
	struct img_frm  target_jpeg;
	struct img_frm  thum_yuv;
	struct img_frm  thum_jpeg;
	struct img_frm  jpeg_tmp;
	struct img_frm  scale_tmp;
	struct img_frm  cap_yuv_rot;
};

int camera_capture_buf_size(struct img_size *image_size, 
								uint32_t *size_major, 
								uint32_t *size_minor);
int camera_arrange_capture_buf(struct cmr_cap_2_frm *cap_2_frm,
								struct img_size *image_size, 
								struct img_size *thum_size, 
								struct cmr_cap_mem *capture_mem);

#ifdef __cplusplus
}
#endif

#endif
