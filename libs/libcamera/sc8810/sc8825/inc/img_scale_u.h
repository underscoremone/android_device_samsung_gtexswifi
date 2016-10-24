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
#ifndef _IMAGE_SCALE_USER_H_
#define _IMAGE_SCALE_USER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <linux/types.h>
#include <asm/ioctl.h>

enum scale_cfg_id {
	SCALE_INPUT_SIZE = 0,
	SCALE_INPUT_RECT,
	SCALE_INPUT_FORMAT,
	SCALE_INPUT_ADDR,
	SCALE_INPUT_ENDIAN,
	SCALE_OUTPUT_SIZE,
	SCALE_OUTPUT_FORMAT,
	SCALE_OUTPUT_ADDR,
	SCALE_OUTPUT_ENDIAN,
	SCALE_TEMP_BUFF,
	SCALE_SCALE_MODE,
	SCALE_SLICE_SCALE_HEIGHT,
	SCALE_START,
	SCALE_CONTINUE,
	SCALE_IS_DONE,
	SCALE_STOP,
	SCALE_CFG_ID_E_MAX
};

enum scale_fmt {
	SCALE_YUV422 = 0,
	SCALE_YUV420,
	SCALE_YUV400,
	SCALE_YUV420_3FRAME,
	SCALE_RGB565,
	SCALE_RGB888,
	SCALE_FTM_MAX
};
enum scale_data_endian {
	SCALE_ENDIAN_BIG = 0,
	SCALE_ENDIAN_LITTLE,
	SCALE_ENDIAN_HALFBIG,
	SCALE_ENDIAN_HALFLITTLE,
	SCALE_ENDIAN_MAX        
};

enum scle_mode {
	SCALE_MODE_NORMAL = 0,
	SCALE_MODE_SLICE,
	SCALE_MODE_MAX
};
enum scale_rotate
{
	SCALE_ROTATION_0 = 0,
	SCALE_ROTATION_90,
	SCALE_ROTATION_180,
	SCALE_ROTATION_270,
	SCALE_ROTATION_MIRROR,
	SCALE_ROTATION_MAX
};

struct scale_size {
	uint32_t               w;
	uint32_t               h;
};

struct scale_rect {
	uint32_t               x;
	uint32_t               y;
	uint32_t               w;
	uint32_t               h;
};

struct scale_addr {
	uint32_t               yaddr;
	uint32_t               uaddr;
	uint32_t               vaddr;
};

struct scale_endian_sel {
	uint8_t               y_endian;
	uint8_t               uv_endian;
	uint8_t               reserved0;
	uint8_t               reserved1;
};

struct scale_frame {
	uint32_t                type;
	uint32_t                lock;
	uint32_t                flags;
	uint32_t                fid;
	uint32_t                width;
	uint32_t                height;
	uint32_t                yaddr;
	uint32_t                uaddr;
	uint32_t                vaddr;
	struct scale_endian_sel endian;
};

#define SCALE_IO_MAGIC                             'S'
#define SCALE_IO_INPUT_SIZE                        _IOW(SCALE_IO_MAGIC, SCALE_INPUT_SIZE,         struct scale_size)
#define SCALE_IO_INPUT_RECT                        _IOW(SCALE_IO_MAGIC, SCALE_INPUT_RECT,         struct scale_rect)
#define SCALE_IO_INPUT_FORMAT                      _IOW(SCALE_IO_MAGIC, SCALE_INPUT_FORMAT,       enum scale_fmt)
#define SCALE_IO_INPUT_ADDR                        _IOW(SCALE_IO_MAGIC, SCALE_INPUT_ADDR,         struct scale_addr)
#define SCALE_IO_INPUT_ENDIAN                      _IOW(SCALE_IO_MAGIC, SCALE_INPUT_ENDIAN,       struct scale_endian_sel)
#define SCALE_IO_OUTPUT_SIZE                       _IOW(SCALE_IO_MAGIC, SCALE_OUTPUT_SIZE,        struct scale_size)
#define SCALE_IO_OUTPUT_FORMAT                     _IOW(SCALE_IO_MAGIC, SCALE_OUTPUT_FORMAT,      enum scale_fmt)
#define SCALE_IO_OUTPUT_ADDR                       _IOW(SCALE_IO_MAGIC, SCALE_OUTPUT_ADDR,        struct scale_addr)
#define SCALE_IO_OUTPUT_ENDIAN                     _IOW(SCALE_IO_MAGIC, SCALE_OUTPUT_ENDIAN,      struct scale_endian_sel)
#define SCALE_IO_TEMP_BUFF                         _IOW(SCALE_IO_MAGIC, SCALE_TEMP_BUFF,          struct scale_addr)
#define SCALE_IO_SCALE_MODE                        _IOW(SCALE_IO_MAGIC, SCALE_SCALE_MODE,         enum scle_mode)
#define SCALE_IO_SLICE_SCALE_HEIGHT                _IOW(SCALE_IO_MAGIC, SCALE_SLICE_SCALE_HEIGHT, uint32_t)
#define SCALE_IO_START                             _IO(SCALE_IO_MAGIC,  SCALE_START)
#define SCALE_IO_CONTINUE                          _IO(SCALE_IO_MAGIC,  SCALE_CONTINUE)
#define SCALE_IO_STOP                              _IO(SCALE_IO_MAGIC,  SCALE_STOP)
#define SCALE_IO_IS_DONE                           _IOR(SCALE_IO_MAGIC, SCALE_IS_DONE,            struct scale_frame)

#ifdef __cplusplus
}
#endif

#endif

