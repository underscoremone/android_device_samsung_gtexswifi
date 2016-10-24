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
#ifndef _DC_CFG_H_
#define _DC_CFG_H_

#define DCAM_CFG_DEBUG 1
#ifdef DCAM_CFG_DEBUG
#define DCAM_CFG_PRINT LOGV
#else
#define DCAM_CFG_PRINT(...)
#endif
#define DCAM_CFG_ERR LOGE

#include <linux/types.h>
#include "jpeg_exif_header.h"
JINF_EXIF_INFO_T *DC_InitExifParameter(JINF_EXIF_INFO_T * exif_info_ptr,
				       uint32_t size);
JINF_EXIF_INFO_T *DC_GetExifParameter(void);
void DC_GetExifParameter_Post(void);
#endif
