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
#ifndef _DC_CFG_H_
#define _DC_CFG_H_

#define DCAM_CFG_DEBUG 1
#ifdef DCAM_CFG_DEBUG
#define DCAM_CFG_PRINT ALOGV
#else
#define DCAM_CFG_PRINT(...)
#endif
#define DCAM_CFG_ERR ALOGE

#include <linux/types.h>
#include "jpeg_exif_header.h"
JINF_EXIF_INFO_T *DC_InitExifParameter(JINF_EXIF_INFO_T * exif_info_ptr,
				       uint32_t size);
JINF_EXIF_INFO_T *DC_GetExifParameter(void);
void DC_GetExifParameter_Post(void);
#endif
