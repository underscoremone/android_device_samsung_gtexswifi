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
#ifndef _SPRD_CAMERA_HARDWARE_CONFIG_H_
#define _SPRD_CAMERA_HARDWARE_CONFIG_H_

#define CAMERA_DIGITAL_ZOOM_MAX     2
#define JPEG_THUMBNAIL_WIDTH        320
#define JPEG_THUMBNAIL_HEIGHT       240

#define BACK_SENSOR_ORIG_WIDTH 2592
#define BACK_SENSOR_ORIG_HEIGHT 1952

#define FRONT_SENSOR_ORIG_WIDTH 720 //select according to max jpg resolution
#define FRONT_SENSOR_ORIG_HEIGHT 480

#define JPEG_MAX_SIZE               ((BACK_SENSOR_ORIG_WIDTH * BACK_SENSOR_ORIG_HEIGHT + (2*1024-1)) & ~(2*1024-1))


enum {
	CAMERA_SCENE_MODE_AUTO = 0,
	CAMERA_SCENE_MODE_NIGHT,
	CAMERA_SCENE_MODE_ACTION, //not support
	CAMERA_SCENE_MODE_PORTRAIT, //not support
	CAMERA_SCENE_MODE_LANDSCAPE, //not support
	CAMERA_SCENE_MODE_NORMAL,
	CAMERA_SCENE_MODE_MAX
};

enum {
	CAMERA_FOCUS_MODE_AUTO = 0,
	CAMERA_FOCUS_MODE_AUTO_MULTI,
	CAMERA_FOCUS_MODE_MACRO,
	CAMERA_FOCUS_MODE_INFINITY,
	CAMERA_FOCUS_MODE_CAF = 4,
	CAMERA_FOCUS_MODE_CAF_VIDEO = 5,
	CAMERA_FOCUS_MODE_MAX
};

enum {
	CAMERA_FLASH_MODE_OFF = 0,
	CAMERA_FLASH_MODE_ON,
	CAMERA_FLASH_MODE_TORCH,
	CAMERA_FLASH_MODE_AUTO,
	CAMERA_FLASH_MODE_MAX
};

enum {
	CAMERA_WB_AUTO = 0,
	CAMERA_WB_INCANDESCENT,
	CAMERA_WB_FLUORESCENT = 4, //id 2 and 3 not used
	CAMERA_WB_DAYLIGHT,
	CAMERA_WB_CLOUDY_DAYLIGHT,
	CAMERA_WB_MAX
};

enum {
	CAMERA_AE_FRAME_AVG = 0,
	CAMERA_AE_CENTER_WEIGHTED,
	CAMERA_AE_SPOT_METERING,
	CAMERA_AE_MODE_MAX
};

typedef enum _ae_state {
	AE_STATE_INACTIVE = 1,
	AE_STATE_SEARCHING,
	AE_STATE_CONVERGED,
	AE_STATE_LOCKED,
	AE_STATE_FLASH_REQUIRED,
	AE_STATE_PRECAPTURE
}ae_state;

typedef enum _awb_lock {
	AWB_LOCK_OFF,
	AWB_LOCK_ON
}awb_lock;

typedef enum _ae_lock {
	AE_LOCK_OFF,
	AE_LOCK_ON
}ae_lock;

struct str_map {
	const char *const desc;
	int val;
};
const struct str_map focus_mode_map[] = {
	{"auto",               CAMERA_FOCUS_MODE_AUTO},
	{"auto-multi",         CAMERA_FOCUS_MODE_AUTO_MULTI},
	{"macro",              CAMERA_FOCUS_MODE_MACRO},
	{"infinity",           CAMERA_FOCUS_MODE_INFINITY},
	{"continuous-picture", CAMERA_FOCUS_MODE_CAF},
	{"continuous-video",   CAMERA_FOCUS_MODE_CAF_VIDEO},
	{NULL,                 0}
};
#endif //_SPRD_CAMERA_HARDWARE_CONFIG_H_
