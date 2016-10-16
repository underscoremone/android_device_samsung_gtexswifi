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
#ifndef _DC_PRODUCT_CFG_H_
#define _DC_PRODUCT_CFG_H_
#include <linux/types.h>
#include "sensor_drv_u.h"

#define DC_PRO_CFG_DEBUG 1
#ifdef DC_PRO_CFG_DEBUG
#define DC_PRO_CFG_PRINT ALOGV
#else
#define DC_PRO_CFG_PRINT(...)
#endif
#define DC_PRO_CFG_ERR ALOGE

typedef enum {
	DC_MAX_VIDEO_MODE_QCIF = 0x01,
	DC_MAX_VIDEO_MODE_QVGA,
	DC_MAX_VIDEO_MODE_CIF,
	DC_MAX_VIDEO_MODE_VGA,
	DC_MAX_VIDEO_MODE_MAX
} DC_MAX_VIDEO_MODE_E;

typedef enum {
	DC_PRODUCT_FLASH_TYPE_DISABLE = 0x00,
	DC_PRODUCT_FLASH_TYPE_LED,
	DC_PRODUCT_FLASH_TYPE_XENON,
	DC_PRODUCT_FLASH_TYPE_MAX
} DC_PRODUCT_FLASH_TYPE_TYPE_E;

typedef enum {
	DC_PRODUCT_MEM_TYPE_NAND = 0x00,
	DC_PRODUCT_MEM_TYPE_NOR,
	DC_PRODUCT_MEM_TYPE_MAX
} DC_PRODUCT_MEM_TYPE_E;

typedef enum {
	DC_PRODUCT_IOCTL_CMD_MAX,
} DC_PRODUCT_IOCTL_CMD_E;

typedef struct _dc_product_mem_cfg_tag {
	BOOLEAN exif_eb;
	BOOLEAN thumbnail_eb;
	BOOLEAN reverse1;
	BOOLEAN reverse0;
	uint32_t dc_mode_mem;
	uint32_t vt_mode_mem;
	DC_MAX_VIDEO_MODE_E max_video_mode;
	DC_PRODUCT_FLASH_TYPE_TYPE_E flash_type;
	DC_PRODUCT_MEM_TYPE_E mem_type;
} DC_PRODUCT_CFG_T, *DC_PRODUCT_CFG_T_PTR;

typedef struct _dc_product_cfg_func_tab_tag {
	uint32_t(*get_productcfg) (uint32_t param);
	uint32_t(*get_exifprimarypridesc) (uint32_t param);
	uint32_t(*get_exifspecuser) (uint32_t param);
	uint32_t(*get_product_ioctl) (DC_PRODUCT_IOCTL_CMD_E ctl_cmd,
				      void *param);
} DC_PRODUCT_CFG_FUNC_TAB_T, *DC_PRODUCT_CFG_FUNC_TAB_T_PTR;

DC_PRODUCT_CFG_FUNC_TAB_T_PTR DC_GetDcProductCfgFun(void);
#endif
