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
#include "cmr_mem.h"
#include "cmr_oem.h"
#include <unistd.h>
/*
          to add more.........
     8M-----------16M-------------8M
     5M-----------16M-------------0M
     3M-----------8M--------------2M
     2M-----------4M--------------2M
     1.3M---------4M--------------1M
*/

#define PIXEL_1P3_MEGA            0x180000 //actually 1.5 *1024*1024
#define PIXEL_2P0_MEGA            0x200000 //actually 2.0 *1024*1024
#define PIXEL_3P0_MEGA            0x300000 //actually 3.0 *1024*1024
#define PIXEL_5P0_MEGA            0x500000 //5.0 *1024*1024
#define PIXEL_8P0_MEGA            0x800000 //8.0 *1024*1024
#define ISP_YUV_TO_RAW_GAP        CMR_SLICE_HEIGHT
#define BACK_CAMERA_ID            0
#define FRONT_CAMERA_ID           1
#define ADDR_BY_WORD(a)           (((a) + 3 ) & (~3))
#define CMR_NO_MEM(a, b)                                                                  \
	do {                                                                              \
		if ((a) > (b)) {                                                          \
			CMR_LOGE("No memory, 0x%x 0x%x", (a), (b));                       \
			return -1;                                                        \
		}                                                                         \
	} while(0)

enum {
	IMG_1P3_MEGA = 0,
	IMG_2P0_MEGA,
	IMG_3P0_MEGA,
	IMG_5P0_MEGA,
	IMG_8P0_MEGA,
	IMG_SIZE_NUM
};

enum {
	JPEG_TARGET = 0,
	THUM_YUV,
	THUM_JPEG,
	JPEG_TMP,
	SCALER_TMP,
#if CMR_ISP_YUV422
	ISP_TMP,
#endif
	BUF_TYPE_NUM
};


typedef uint32_t (*cmr_get_size)(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height);

struct cap_size_to_mem {
	uint32_t    pixel_num;
	uint32_t    major_size;
	uint32_t    minor_size;
};
#ifdef CONFIG_BACK_CAMERA_ROTATION
static const struct cap_size_to_mem back_cam_mem_size_tab[IMG_SIZE_NUM] = {
	{PIXEL_1P3_MEGA, (6 << 20),  (0 << 20)},
	{PIXEL_2P0_MEGA, (8 << 20),  (0 << 20)},
	{PIXEL_3P0_MEGA, (11 << 20),  (0 << 20)},
	{PIXEL_5P0_MEGA, (16 << 20), (8 << 20)},
	{PIXEL_8P0_MEGA, (16 << 20), (16 << 20)},
};
static const struct cap_size_to_mem back_cam_raw_mem_size_tab[IMG_SIZE_NUM] = {
	{PIXEL_1P3_MEGA, (8 << 20),  (0 << 20)},
	{PIXEL_2P0_MEGA, (8 << 20),  (1 << 20)},
	{PIXEL_3P0_MEGA, (7 << 20),  (5 << 20)},
	{PIXEL_5P0_MEGA, (16 << 20), (8 << 20)},
	{PIXEL_8P0_MEGA, (24 << 20), (16 << 20)},
};
#else
static const struct cap_size_to_mem back_cam_mem_size_tab[IMG_SIZE_NUM] = {
	{PIXEL_1P3_MEGA, (4 << 20),  (0 << 20)},
	{PIXEL_2P0_MEGA, (5 << 20),  (0 << 20)},
	{PIXEL_3P0_MEGA, (7 << 20),  (0 << 20)},
	{PIXEL_5P0_MEGA, (16 << 20), (0 << 20)},
	{PIXEL_8P0_MEGA, (16 << 20), (8 << 20)},
};
static const struct cap_size_to_mem back_cam_raw_mem_size_tab[IMG_SIZE_NUM] = {
	{PIXEL_1P3_MEGA, (6 << 20),  (0 << 20)},
	{PIXEL_2P0_MEGA, (7 << 20),  (0 << 20)},
	{PIXEL_3P0_MEGA, (8 << 20),  (0 << 20)},
	{PIXEL_5P0_MEGA, (16 << 20), (0 << 20)},
	{PIXEL_8P0_MEGA, (24 << 20), (4 << 20)},
};
#endif

#ifdef CONFIG_FRONT_CAMERA_ROTATION
static const struct cap_size_to_mem front_cam_mem_size_tab[IMG_SIZE_NUM] = {
	{PIXEL_1P3_MEGA, (6 << 20),  (0 << 20)},
	{PIXEL_2P0_MEGA, (8 << 20),  (0 << 20)},
	{PIXEL_3P0_MEGA, (11 << 20),  (0 << 20)},
	{PIXEL_5P0_MEGA, (16 << 20), (8 << 20)},
	{PIXEL_8P0_MEGA, (16 << 20), (16 << 20)},
};
static const struct cap_size_to_mem front_cam_raw_mem_size_tab[IMG_SIZE_NUM] = {
	{PIXEL_1P3_MEGA, (8 << 20),  (0 << 20)},
	{PIXEL_2P0_MEGA, (8 << 20),  (1 << 20)},
	{PIXEL_3P0_MEGA, (7 << 20),  (5 << 20)},
	{PIXEL_5P0_MEGA, (16 << 20), (8 << 20)},
	{PIXEL_8P0_MEGA, (24 << 20), (16 << 20)},
};
#else
static const struct cap_size_to_mem front_cam_mem_size_tab[IMG_SIZE_NUM] = {
	{PIXEL_1P3_MEGA, (4 << 20),  (0 << 20)},
	{PIXEL_2P0_MEGA, (5 << 20),  (0 << 20)},
	{PIXEL_3P0_MEGA, (7 << 20),  (0 << 20)},
	{PIXEL_5P0_MEGA, (16 << 20), (0 << 20)},
	{PIXEL_8P0_MEGA, (16 << 20), (8 << 20)},
};
static const struct cap_size_to_mem front_cam_raw_mem_size_tab[IMG_SIZE_NUM] = {
	{PIXEL_1P3_MEGA, (6 << 20),  (0 << 20)},
	{PIXEL_2P0_MEGA, (7 << 20),  (0 << 20)},
	{PIXEL_3P0_MEGA, (8 << 20),  (0 << 20)},
	{PIXEL_5P0_MEGA, (16 << 20), (0 << 20)},
	{PIXEL_8P0_MEGA, (24 << 20), (4 << 20)},
};
#endif
/*for ATV*/
static const struct cap_size_to_mem mem_size_tab[IMG_SIZE_NUM] = {
	{PIXEL_1P3_MEGA, (4 << 20),  (4 << 20)},
	{PIXEL_2P0_MEGA, (8 << 20),  (8 << 20)},
	{PIXEL_3P0_MEGA, (16 << 20),  (2 << 20)},
	{PIXEL_5P0_MEGA, (16 << 20), (0 << 20)},
	{PIXEL_8P0_MEGA, (16 << 20), (8 << 20)},
};
static const struct cap_size_to_mem raw_mem_size_tab[IMG_SIZE_NUM] = {
	{PIXEL_1P3_MEGA, (4 << 20),  (4 << 20)},
	{PIXEL_2P0_MEGA, (8 << 20),  (8 << 20)},
	{PIXEL_3P0_MEGA, (16 << 20),  (2 << 20)},
	{PIXEL_5P0_MEGA, (16 << 20), (0 << 20)},
	{PIXEL_8P0_MEGA, (16 << 20), (8 << 20)},
};

static uint32_t get_jpeg_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height);
static uint32_t get_thum_yuv_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height);
static uint32_t get_thum_jpeg_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height);
static uint32_t get_jpg_tmp_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height);
static uint32_t get_scaler_tmp_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height);
static uint32_t get_isp_tmp_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height);

static const cmr_get_size get_size[BUF_TYPE_NUM] = {
	get_jpeg_size,
	get_thum_yuv_size,
	get_thum_jpeg_size,
	get_jpg_tmp_size,
	get_scaler_tmp_size,
#if CMR_ISP_YUV422
	get_isp_tmp_size
#endif
};

int camera_capture_buf_size(uint32_t     camera_id,
					uint32_t        sn_fmt,
					struct img_size *image_size,
					uint32_t        *size_major,
					uint32_t        *size_minor)
{
	uint32_t               size_pixel = (uint32_t)(image_size->width * image_size->height);
	int                    i;
	struct cap_size_to_mem *mem_tab_ptr = NULL;

	if (NULL == image_size ||
		NULL == size_major ||
		NULL == size_minor) {
		CMR_LOGE("Parameter error 0x%x 0x%x 0x%x",
			(uint32_t)image_size,
			(uint32_t)size_major,
			(uint32_t)size_minor);
		return -1;
	}

	if (SENSOR_IMAGE_FORMAT_RAW == sn_fmt) {
		if (BACK_CAMERA_ID == camera_id) {
			mem_tab_ptr = (struct cap_size_to_mem*)&back_cam_raw_mem_size_tab[0];
		} else if(FRONT_CAMERA_ID == camera_id) {
			mem_tab_ptr = (struct cap_size_to_mem*)&front_cam_raw_mem_size_tab[0];
		} else {
			mem_tab_ptr = (struct cap_size_to_mem*)&raw_mem_size_tab[0];
		}
	} else {
		if (BACK_CAMERA_ID == camera_id) {
			mem_tab_ptr = (struct cap_size_to_mem*)&back_cam_mem_size_tab[0];
		} else if(FRONT_CAMERA_ID == camera_id) {
			mem_tab_ptr = (struct cap_size_to_mem*)&front_cam_mem_size_tab[0];
		} else {
			mem_tab_ptr = (struct cap_size_to_mem*)&raw_mem_size_tab[0];
		}
	}

	for (i = IMG_1P3_MEGA; i < IMG_SIZE_NUM; i++) {
		if (size_pixel <= mem_tab_ptr[i].pixel_num)
			break;
	}

	if (i == IMG_SIZE_NUM) {
		CMR_LOGE("No matched size for this image, 0x%x", size_pixel);
		return -1;
	} else {
		CMR_LOGV("image size num, %d, major mem 0x%x, minor mem 0x%x",
			i,
			mem_tab_ptr[i].major_size,
			mem_tab_ptr[i].minor_size);
	}

	*size_major = mem_tab_ptr[i].major_size;
	*size_minor = mem_tab_ptr[i].minor_size;

	return 0;
}

int camera_arrange_capture_buf(struct cmr_cap_2_frm *cap_2_frm,
						struct img_size *sn_size,
						struct img_rect *sn_trim,
						struct img_size *image_size,
						uint32_t orig_fmt,
						struct img_size *cap_size,
						struct img_size *thum_size,
						struct cmr_cap_mem *capture_mem,
						uint32_t need_rot,
						uint32_t image_cnt)
{
	uint32_t       channel_size = (uint32_t)(image_size->width * image_size->height);
	uint32_t       size_pixel = channel_size;
	uint32_t       major_res = cap_2_frm->major_frm.buf_size;
	uint32_t       minor_res = cap_2_frm->minor_frm.buf_size;
	uint32_t       major_end = 0, minor_end = 0;
	uint32_t       i, yuv_raw_size = 0, img_cnt = 0;
	uint32_t       offset = 0, offset_1;
	uint32_t       y_to_raw = 0, yy_to_y = 0, tmp = 0, raw_size = 0;
	uint32_t       uv_size = 0, useless_raw = 0;
	struct cmr_cap_mem *cap_mem = &capture_mem[0];
	struct img_frm img_frame[BUF_TYPE_NUM];

	if (NULL == cap_2_frm ||
		NULL == image_size ||
		NULL == thum_size ||
		NULL == capture_mem ||
		NULL == sn_size ||
		NULL == sn_trim) {
		CMR_LOGE("Parameter error 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
			(uint32_t)cap_2_frm,
			(uint32_t)image_size,
			(uint32_t)thum_size,
			(uint32_t)capture_mem,
			(uint32_t)sn_size,
			(uint32_t)sn_trim);
		return -1;
	}

	CMR_LOGV("Major frame, 0x%x 0x%x, 0x%x",
		cap_2_frm->major_frm.addr_phy.addr_y,
		cap_2_frm->major_frm.addr_vir.addr_y,
		cap_2_frm->major_frm.buf_size);

	CMR_LOGV("Minor frame, 0x%x 0x%x, 0x%x",
		cap_2_frm->minor_frm.addr_phy.addr_y,
		cap_2_frm->minor_frm.addr_vir.addr_y,
		cap_2_frm->minor_frm.buf_size);

	CMR_LOGV("channel_size, 0x%x, image_cnt %d, rot %d",
		channel_size,
		image_cnt,
		need_rot);

	size_pixel = (size_pixel << 1);

	memset((void*)capture_mem, 0, sizeof(struct cmr_cap_mem));
	cap_mem->target_jpeg.buf_size = get_jpeg_size(image_size->width,
					image_size->height,
					thum_size->width,
					thum_size->height);

	if (IMG_DATA_TYPE_RAW == orig_fmt) {
		channel_size = (uint32_t)(sn_size->width * sn_size->height);
		raw_size = (uint32_t)(channel_size * RAWRGB_BIT_WIDTH / 8);
		y_to_raw = (uint32_t)(ISP_YUV_TO_RAW_GAP * sn_size->width);
		uv_size = (channel_size >> 1);
		if (image_size->width != sn_size->width ||
			image_size->height != sn_size->height) {
			yy_to_y = (uint32_t)(ISP_YUV_TO_RAW_GAP * sn_size->width);
			uv_size = uv_size + (yy_to_y >> 1);
			tmp = (sn_size->height - image_size->height) >> 1;
			CMR_LOGV("Need scaling down, Recovered height, %d", tmp);
			useless_raw = (uint32_t)(yy_to_y * RAWRGB_BIT_WIDTH / 8);
		} else if (sn_trim && sn_trim->start_y) {
			tmp = sn_size->height - sn_trim->height - sn_trim->start_y;
			CMR_LOGV("Recovered height, %d", tmp);
			yy_to_y = (uint32_t)(tmp * sn_size->width);
			uv_size = uv_size + (yy_to_y >> 1);
			useless_raw = (uint32_t)(yy_to_y * RAWRGB_BIT_WIDTH / 8);
		}
		cap_mem->target_yuv.addr_phy.addr_y = cap_2_frm->major_frm.addr_phy.addr_y;
		cap_mem->target_yuv.addr_vir.addr_y = cap_2_frm->major_frm.addr_vir.addr_y;
		cap_mem->cap_yuv.addr_phy.addr_y = cap_mem->target_yuv.addr_phy.addr_y + yy_to_y;
		cap_mem->cap_yuv.addr_vir.addr_y = cap_mem->target_yuv.addr_vir.addr_y + yy_to_y;
		cap_mem->cap_raw.addr_phy.addr_y = cap_mem->cap_yuv.addr_phy.addr_y + y_to_raw;
		cap_mem->cap_raw.addr_vir.addr_y = cap_mem->cap_yuv.addr_vir.addr_y + y_to_raw;
		cap_mem->cap_raw.buf_size        = raw_size;
		CMR_LOGI("y_to_raw 0x%x, yy_to_y 0x%x, raw size 0x%x useless_raw 0x%x",
			y_to_raw, yy_to_y, raw_size, useless_raw);

		yuv_raw_size = raw_size + y_to_raw + yy_to_y + (channel_size >> 1);
		offset = raw_size + y_to_raw + yy_to_y - useless_raw;// the end of RawRGB
		CMR_NO_MEM(offset, major_res);
		major_end += offset;
		major_res -= offset;

		CMR_NO_MEM(capture_mem->target_jpeg.buf_size, major_res);
		cap_mem->target_jpeg.addr_phy.addr_y = cap_2_frm->major_frm.addr_phy.addr_y + major_end;
		cap_mem->target_jpeg.addr_vir.addr_y = cap_2_frm->major_frm.addr_vir.addr_y + major_end;
		major_end += cap_mem->target_jpeg.buf_size;
		major_res -= cap_mem->target_jpeg.buf_size;

		if (uv_size < major_res) {
			//if Y and UV can be located at Major frame
			cap_mem->target_yuv.addr_phy.addr_u = cap_2_frm->major_frm.addr_phy.addr_y + major_end;
			cap_mem->target_yuv.addr_vir.addr_u = cap_2_frm->major_frm.addr_vir.addr_y + major_end;
			major_end += uv_size;
			major_res -= uv_size;
		} else {
			CMR_NO_MEM(uv_size, minor_res);
			cap_mem->target_yuv.addr_phy.addr_u = cap_2_frm->minor_frm.addr_phy.addr_y;
			cap_mem->target_yuv.addr_vir.addr_u = cap_2_frm->minor_frm.addr_vir.addr_y;
			minor_end += uv_size;
			minor_res -= uv_size;
		}
		cap_mem->cap_yuv.addr_phy.addr_u = cap_mem->target_yuv.addr_phy.addr_u + (yy_to_y >> 1);
		cap_mem->cap_yuv.addr_vir.addr_u = cap_mem->target_yuv.addr_vir.addr_u + (yy_to_y >> 1);

		cap_mem->cap_yuv.buf_size        = (channel_size * 3) >> 1;
		cap_mem->cap_yuv.size.width      = sn_size->width;
		cap_mem->cap_yuv.size.height     = sn_size->height;
		cap_mem->cap_yuv.fmt             = IMG_DATA_TYPE_YUV420;

	} else {
		channel_size = (uint32_t)(image_size->width * image_size->height);
		offset = (channel_size * 3) >> 1;
		CMR_NO_MEM(offset, major_res);
		tmp = (uint32_t)(cap_size->width * cap_size->height);
		yy_to_y = channel_size - tmp;
		cap_mem->target_yuv.addr_phy.addr_y = cap_2_frm->major_frm.addr_phy.addr_y;
		cap_mem->target_yuv.addr_vir.addr_y = cap_2_frm->major_frm.addr_vir.addr_y;
		cap_mem->target_yuv.addr_phy.addr_u = cap_mem->target_yuv.addr_phy.addr_y + channel_size;
		cap_mem->target_yuv.addr_vir.addr_u = cap_mem->target_yuv.addr_vir.addr_y + channel_size;

		cap_mem->cap_yuv.addr_phy.addr_y = cap_mem->target_yuv.addr_phy.addr_y + yy_to_y;
		cap_mem->cap_yuv.addr_vir.addr_y = cap_mem->target_yuv.addr_vir.addr_y + yy_to_y;
		cap_mem->cap_yuv.addr_phy.addr_u = cap_mem->target_yuv.addr_phy.addr_u + (yy_to_y >> 1);
		cap_mem->cap_yuv.addr_vir.addr_u = cap_mem->target_yuv.addr_vir.addr_u + (yy_to_y >> 1);
		yuv_raw_size = offset;
		major_end = offset;
		major_res = major_res - major_end;
		CMR_NO_MEM(cap_mem->target_jpeg.buf_size, major_res);
		cap_mem->target_jpeg.addr_phy.addr_y = cap_2_frm->major_frm.addr_phy.addr_y + major_end;
		cap_mem->target_jpeg.addr_vir.addr_y = cap_2_frm->major_frm.addr_vir.addr_y + major_end;
		major_end += cap_mem->target_jpeg.buf_size;
		major_res -= cap_mem->target_jpeg.buf_size;

		/* Get the capture Y buffer, same to RawRGB buffer */
		cap_mem->cap_yuv.buf_size        = (channel_size * 3) >> 1;
		cap_mem->cap_yuv.size.width      = cap_size->width;
		cap_mem->cap_yuv.size.height     = cap_size->height;
		cap_mem->cap_yuv.fmt             = IMG_DATA_TYPE_YUV420;

	}

	/* Get the capture UV buffer */
	cap_mem->cap_raw.size.width      = sn_size->width;
	cap_mem->cap_raw.size.height     = sn_size->height;
	cap_mem->cap_raw.fmt             = IMG_DATA_TYPE_RAW;
	CMR_LOGI("cap_raw, phy 0x%x 0x%x, vir 0x%x 0x%x, size 0x%x",
		cap_mem->cap_raw.addr_phy.addr_y,
		cap_mem->cap_raw.addr_phy.addr_u,
		cap_mem->cap_raw.addr_vir.addr_y,
		cap_mem->cap_raw.addr_vir.addr_u,
		cap_mem->cap_raw.buf_size);


	/* Get the Target Y/UV buffer */
	cap_mem->target_yuv.buf_size     = (channel_size * 3) >> 1;
	cap_mem->target_yuv.size.width   = image_size->width;
	cap_mem->target_yuv.size.height  = image_size->height;
	cap_mem->target_yuv.fmt          = IMG_DATA_TYPE_YUV420;
	CMR_LOGI("target_yuv, phy 0x%x 0x%x, vir 0x%x 0x%x, size 0x%x",
		cap_mem->target_yuv.addr_phy.addr_y,
		cap_mem->target_yuv.addr_phy.addr_u,
		cap_mem->target_yuv.addr_vir.addr_y,
		cap_mem->target_yuv.addr_vir.addr_u,
		cap_mem->target_yuv.buf_size);

	CMR_LOGI("cap_yuv, phy 0x%x 0x%x, vir 0x%x 0x%x, size 0x%x",
		cap_mem->cap_yuv.addr_phy.addr_y,
		cap_mem->cap_yuv.addr_phy.addr_u,
		cap_mem->cap_yuv.addr_vir.addr_y,
		cap_mem->cap_yuv.addr_vir.addr_u,
		cap_mem->cap_yuv.buf_size);

	CMR_LOGI("target_jpeg, phy 0x%x, vir 0x%x, size 0x%x",
		cap_mem->target_jpeg.addr_phy.addr_y,
		cap_mem->target_jpeg.addr_phy.addr_y,
		cap_mem->target_jpeg.buf_size);
	/* re-calculate the currend end of major buffer */

	CMR_LOGV("Now to alloc misc buffers");
	for(i = THUM_YUV; i < BUF_TYPE_NUM; i++) {
		/* calculate the address of target_jpeg, start */
		size_pixel = get_size[i](image_size->width, image_size->height, thum_size->width, thum_size->height);
		if (major_res >= size_pixel) {
			img_frame[i].buf_size = size_pixel;
			img_frame[i].addr_phy.addr_y = cap_2_frm->major_frm.addr_phy.addr_y + major_end;
			img_frame[i].addr_vir.addr_y = cap_2_frm->major_frm.addr_vir.addr_y + major_end;
			img_frame[i].addr_phy.addr_u = img_frame[i].addr_phy.addr_y + size_pixel * 2 / 3;
			img_frame[i].addr_vir.addr_u = img_frame[i].addr_vir.addr_y + size_pixel * 2 / 3;
			/* re-calculate the currend end of major buffer */
			major_res -= size_pixel;
			major_end += size_pixel;
		} else {
			break;
		}
	}

	if (i != BUF_TYPE_NUM) {
		CMR_LOGV("No more memory reseved in Major buffer, to alloc misc buffers from Minor buffer");
		/* Not all the misc buffer have been alloc-ed yet, get memory from minor_frm */
		for (; i < BUF_TYPE_NUM; i++) {
			/* calculate the address of target_jpeg, start */
			size_pixel = get_size[i](image_size->width, image_size->height, thum_size->width, thum_size->height);
			if (minor_res >= size_pixel) {
				img_frame[i].buf_size = size_pixel;
				img_frame[i].addr_phy.addr_y = cap_2_frm->minor_frm.addr_phy.addr_y + minor_end;
				img_frame[i].addr_vir.addr_y = cap_2_frm->minor_frm.addr_vir.addr_y + minor_end;;
				img_frame[i].addr_phy.addr_u = img_frame[i].addr_phy.addr_y + size_pixel * 2 / 3;
				img_frame[i].addr_vir.addr_u = img_frame[i].addr_vir.addr_y + size_pixel * 2 / 3;
				/* re-calculate the currend end of major buffer */
				minor_res -= size_pixel;
				minor_end += size_pixel;
			} else {
				break;
			}

		}
	}

	if (i != BUF_TYPE_NUM) {
		CMR_LOGE("Failed to alloc all the buffers used in capture");
		return -1;
	}

	CMR_LOGV("major_end, major_res, minor_end, minor_res: 0x%x 0x%x 0x%x 0x%x",
		major_end, major_res, minor_end, minor_res);

	/*Alloc Rotation buffer if necessary, Start*/
	if (need_rot) {
		size_pixel = (channel_size * 3) >> 1;
		CMR_LOGV("Rot channel size 0x%x, buf size 0x%X", channel_size, size_pixel);
		if (major_res > size_pixel) {
			CMR_LOGV("Rot buffer located at Major frame");
			tmp = 1;
			offset = cap_2_frm->major_frm.addr_phy.addr_y + major_end;
			offset_1 = cap_2_frm->major_frm.addr_vir.addr_y + major_end;
			major_res -= size_pixel;
			major_end += size_pixel;
		} else if (minor_res > size_pixel) {
			CMR_LOGV("Rot buffer located at Minor frame");
			tmp = 1;
			offset = cap_2_frm->minor_frm.addr_phy.addr_y + minor_end;
			offset_1 = cap_2_frm->minor_frm.addr_vir.addr_y + minor_end;
			minor_res -= size_pixel;
			minor_end += size_pixel;
		} else {
			CMR_LOGV("No Rot buffer");
			tmp = 0;
		}

		if (tmp) {
			cap_mem->cap_yuv_rot.addr_phy.addr_y = offset;
			cap_mem->cap_yuv_rot.addr_vir.addr_y = offset_1;
			cap_mem->cap_yuv_rot.addr_phy.addr_u = offset + channel_size;
			cap_mem->cap_yuv_rot.addr_vir.addr_u = offset_1 + channel_size;
			cap_mem->cap_yuv_rot.size.width      = image_size->height;
			cap_mem->cap_yuv_rot.size.height     = image_size->width;
			cap_mem->cap_yuv_rot.buf_size        = size_pixel;
			cap_mem->cap_yuv_rot.fmt             = IMG_DATA_TYPE_YUV420;
		}
		CMR_LOGI("cap_yuv_rot, phy 0x%x 0x%x, vir 0x%x 0x%x, size 0x%x",
			cap_mem->cap_yuv_rot.addr_phy.addr_y,
			cap_mem->cap_yuv_rot.addr_phy.addr_u,
			cap_mem->cap_yuv_rot.addr_vir.addr_y,
			cap_mem->cap_yuv_rot.addr_vir.addr_u,
			cap_mem->cap_yuv_rot.buf_size);
	}
	/*Alloc Rotation buffer End*/

	cap_mem->thum_yuv.buf_size = img_frame[THUM_YUV].buf_size;
	cap_mem->thum_yuv.addr_phy.addr_y = img_frame[THUM_YUV].addr_phy.addr_y;
	cap_mem->thum_yuv.addr_vir.addr_y = img_frame[THUM_YUV].addr_vir.addr_y;
	cap_mem->thum_yuv.addr_phy.addr_u = img_frame[THUM_YUV].addr_phy.addr_u;
	cap_mem->thum_yuv.addr_vir.addr_u = img_frame[THUM_YUV].addr_vir.addr_u;
	cap_mem->thum_yuv.size.width      = thum_size->width;
	cap_mem->thum_yuv.size.height     = thum_size->height;
	CMR_LOGI("thum_yuv, phy 0x%x 0x%x, vir 0x%x 0x%x, size 0x%x",
		img_frame[THUM_YUV].addr_phy.addr_y,
		img_frame[THUM_YUV].addr_phy.addr_u,
		img_frame[THUM_YUV].addr_vir.addr_y,
		img_frame[THUM_YUV].addr_vir.addr_u,
		img_frame[THUM_YUV].buf_size);

	cap_mem->thum_jpeg.buf_size = img_frame[THUM_JPEG].buf_size;
	cap_mem->thum_jpeg.addr_phy.addr_y = img_frame[THUM_JPEG].addr_phy.addr_y;
	cap_mem->thum_jpeg.addr_vir.addr_y = img_frame[THUM_JPEG].addr_vir.addr_y;
	CMR_LOGI("thum_jpeg, phy 0x%x, vir 0x%x, size 0x%x",
		img_frame[THUM_JPEG].addr_phy.addr_y,
		img_frame[THUM_JPEG].addr_vir.addr_y,
		img_frame[THUM_JPEG].buf_size);

	cap_mem->jpeg_tmp.buf_size = img_frame[JPEG_TMP].buf_size;
	cap_mem->jpeg_tmp.addr_phy.addr_y = img_frame[JPEG_TMP].addr_phy.addr_y;
	cap_mem->jpeg_tmp.addr_vir.addr_y = img_frame[JPEG_TMP].addr_vir.addr_y;
	CMR_LOGI("jpeg_tmp, phy 0x%x, vir 0x%x, size 0x%x",
		img_frame[JPEG_TMP].addr_phy.addr_y,
		img_frame[JPEG_TMP].addr_vir.addr_y,
		img_frame[JPEG_TMP].buf_size);

	cap_mem->scale_tmp.buf_size = img_frame[SCALER_TMP].buf_size;
	cap_mem->scale_tmp.addr_phy.addr_y = img_frame[SCALER_TMP].addr_phy.addr_y;
	cap_mem->scale_tmp.addr_vir.addr_y = img_frame[SCALER_TMP].addr_vir.addr_y;
	CMR_LOGI("scale_tmp, phy 0x%x, vir 0x%x, size 0x%x",
		img_frame[SCALER_TMP].addr_phy.addr_y,
		img_frame[SCALER_TMP].addr_vir.addr_y,
		img_frame[SCALER_TMP].buf_size);

#if CMR_ISP_YUV422
	cap_mem->isp_tmp.buf_size = img_frame[ISP_TMP].buf_size;
	cap_mem->isp_tmp.addr_phy.addr_y = img_frame[ISP_TMP].addr_phy.addr_y;
	cap_mem->isp_tmp.addr_vir.addr_y = img_frame[ISP_TMP].addr_vir.addr_y;
	CMR_LOGI("isp_tmp, phy 0x%x, vir 0x%x, size 0x%x",
		img_frame[ISP_TMP].addr_phy.addr_y,
		img_frame[ISP_TMP].addr_vir.addr_y,
		img_frame[ISP_TMP].buf_size);
#endif
	img_cnt ++;
	cap_mem ++;

	/*Alloc other image buffer include RAW and CAP YUV , Start*/
	if (img_cnt < image_cnt) {
		while (major_res >= yuv_raw_size) {
			if (IMG_DATA_TYPE_RAW == orig_fmt) {
				uv_size = channel_size >> 1;
				cap_mem->target_yuv.addr_phy.addr_y = cap_2_frm->major_frm.addr_phy.addr_y + major_end;
				cap_mem->target_yuv.addr_vir.addr_y = cap_2_frm->major_frm.addr_vir.addr_y + major_end;
				cap_mem->cap_yuv.addr_phy.addr_y = cap_mem->target_yuv.addr_phy.addr_y + yy_to_y;
				cap_mem->cap_yuv.addr_vir.addr_y = cap_mem->target_yuv.addr_vir.addr_y + yy_to_y;
				cap_mem->cap_raw.addr_phy.addr_y = cap_mem->cap_yuv.addr_phy.addr_y + y_to_raw;
				cap_mem->cap_raw.addr_vir.addr_y = cap_mem->cap_yuv.addr_vir.addr_y + y_to_raw;
				cap_mem->cap_raw.buf_size        = raw_size;
				cap_mem->cap_yuv.addr_phy.addr_u = cap_mem->cap_raw.addr_phy.addr_y + raw_size;
				cap_mem->cap_yuv.addr_vir.addr_u = cap_mem->cap_raw.addr_vir.addr_y + raw_size;
				cap_mem->target_yuv.addr_phy.addr_u = cap_mem->cap_yuv.addr_phy.addr_u - (uv_size >> 1);
				cap_mem->target_yuv.addr_vir.addr_u = cap_mem->cap_yuv.addr_vir.addr_u - (uv_size >> 1);
				cap_mem->cap_yuv.buf_size        = (channel_size * 3) >> 1;
				cap_mem->cap_yuv.size.width      = sn_size->width;
				cap_mem->cap_yuv.size.height     = sn_size->height;
				cap_mem->cap_yuv.fmt             = IMG_DATA_TYPE_YUV420;
				cap_mem->cap_raw.size.width      = sn_size->width;
				cap_mem->cap_raw.size.height     = sn_size->height;
				cap_mem->cap_raw.fmt             = IMG_DATA_TYPE_RAW;

			} else {
				cap_mem->target_yuv.addr_phy.addr_y = cap_2_frm->major_frm.addr_phy.addr_y + major_end;
				cap_mem->target_yuv.addr_vir.addr_y = cap_2_frm->major_frm.addr_vir.addr_y + major_end;
				cap_mem->target_yuv.addr_phy.addr_u = cap_mem->target_yuv.addr_phy.addr_y + channel_size;
				cap_mem->target_yuv.addr_vir.addr_u = cap_mem->target_yuv.addr_vir.addr_y + channel_size;
				cap_mem->cap_yuv.addr_phy.addr_y = cap_mem->target_yuv.addr_phy.addr_y + yy_to_y;
				cap_mem->cap_yuv.addr_vir.addr_y = cap_mem->target_yuv.addr_vir.addr_y + yy_to_y;
				cap_mem->cap_yuv.addr_phy.addr_u = cap_mem->target_yuv.addr_phy.addr_u + (yy_to_y >> 1);
				cap_mem->cap_yuv.addr_vir.addr_u = cap_mem->target_yuv.addr_vir.addr_u + (yy_to_y >> 1);
				cap_mem->cap_raw.addr_phy.addr_y = cap_mem->cap_yuv.addr_phy.addr_y;
				cap_mem->cap_raw.addr_vir.addr_y = cap_mem->cap_yuv.addr_vir.addr_y;
				cap_mem->cap_raw.buf_size        = raw_size;
				cap_mem->cap_yuv.buf_size        = (channel_size * 3) >> 1;
				cap_mem->cap_yuv.size.width      = cap_size->width;
				cap_mem->cap_yuv.size.height     = cap_size->height;
				cap_mem->cap_yuv.fmt             = IMG_DATA_TYPE_YUV420;
			}

			cap_mem->target_yuv.buf_size     = (channel_size * 3) >> 1;
			cap_mem->target_yuv.size.width   = image_size->width;
			cap_mem->target_yuv.size.height  = image_size->height;
			cap_mem->target_yuv.fmt          = IMG_DATA_TYPE_YUV420;

			img_cnt++;
			if (img_cnt >= image_cnt) {
				break;
			}
			cap_mem ++;
			major_res -= yuv_raw_size;
			major_end += yuv_raw_size;
		}
	}

	if (img_cnt < image_cnt) {
		while (minor_res >= yuv_raw_size) {
			if (IMG_DATA_TYPE_RAW == orig_fmt) {
				uv_size = channel_size >> 1;
				cap_mem->target_yuv.addr_phy.addr_y = cap_2_frm->minor_frm.addr_phy.addr_y + minor_end;
				cap_mem->target_yuv.addr_vir.addr_y = cap_2_frm->minor_frm.addr_vir.addr_y + minor_end;
				cap_mem->cap_yuv.addr_phy.addr_y = cap_mem->target_yuv.addr_phy.addr_y + yy_to_y;
				cap_mem->cap_yuv.addr_vir.addr_y = cap_mem->target_yuv.addr_vir.addr_y + yy_to_y;
				cap_mem->cap_raw.addr_phy.addr_y = cap_mem->cap_yuv.addr_phy.addr_y + y_to_raw;
				cap_mem->cap_raw.addr_vir.addr_y = cap_mem->cap_yuv.addr_vir.addr_y + y_to_raw;
				cap_mem->cap_raw.buf_size        = raw_size;
				cap_mem->cap_yuv.addr_phy.addr_u = cap_mem->cap_raw.addr_phy.addr_y + raw_size;
				cap_mem->cap_yuv.addr_vir.addr_u = cap_mem->cap_raw.addr_vir.addr_y + raw_size;
				cap_mem->target_yuv.addr_phy.addr_u = cap_mem->cap_yuv.addr_phy.addr_u - (uv_size >> 1);
				cap_mem->target_yuv.addr_vir.addr_u = cap_mem->cap_yuv.addr_vir.addr_u - (uv_size >> 1);
				cap_mem->cap_yuv.buf_size        = (channel_size * 3) >> 1;
				cap_mem->cap_yuv.size.width      = sn_size->width;
				cap_mem->cap_yuv.size.height     = sn_size->height;
				cap_mem->cap_yuv.fmt             = IMG_DATA_TYPE_YUV420;
				cap_mem->cap_raw.size.width      = sn_size->width;
				cap_mem->cap_raw.size.height     = sn_size->height;
				cap_mem->cap_raw.fmt             = IMG_DATA_TYPE_RAW;

			} else {
				cap_mem->target_yuv.addr_phy.addr_y = cap_2_frm->minor_frm.addr_phy.addr_y + minor_end;
				cap_mem->target_yuv.addr_vir.addr_y = cap_2_frm->minor_frm.addr_vir.addr_y + minor_end;
				cap_mem->target_yuv.addr_phy.addr_u = cap_mem->target_yuv.addr_phy.addr_y + channel_size;
				cap_mem->target_yuv.addr_vir.addr_u = cap_mem->target_yuv.addr_vir.addr_y + channel_size;
				cap_mem->cap_yuv.addr_phy.addr_y = cap_mem->target_yuv.addr_phy.addr_y + yy_to_y;
				cap_mem->cap_yuv.addr_vir.addr_y = cap_mem->target_yuv.addr_vir.addr_y + yy_to_y;
				cap_mem->cap_yuv.addr_phy.addr_u = cap_mem->target_yuv.addr_phy.addr_u + (yy_to_y >> 1);
				cap_mem->cap_yuv.addr_vir.addr_u = cap_mem->target_yuv.addr_vir.addr_u + (yy_to_y >> 1);
				cap_mem->cap_raw.addr_phy.addr_y = cap_mem->cap_yuv.addr_phy.addr_y;
				cap_mem->cap_raw.addr_vir.addr_y = cap_mem->cap_yuv.addr_vir.addr_y;
				cap_mem->cap_raw.buf_size        = raw_size;
				cap_mem->cap_yuv.buf_size        = (channel_size * 3) >> 1;
				cap_mem->cap_yuv.size.width      = cap_size->width;
				cap_mem->cap_yuv.size.height     = cap_size->height;
				cap_mem->cap_yuv.fmt             = IMG_DATA_TYPE_YUV420;
			}

			img_cnt++;
			if (img_cnt >= image_cnt) {
				break;
			}
			cap_mem ++;
			minor_res -= yuv_raw_size;
			minor_end += yuv_raw_size;
		}
	}

	if (img_cnt < image_cnt) {
		CMR_LOGE("Not enough memory, %d", img_cnt);
		return -1;
	}

	for (i = 1; i < img_cnt; i++) {
		CMR_LOGI("Image ID %d", i);
		CMR_LOGI("cap_raw, phy 0x%x 0x%x, vir 0x%x 0x%x, size 0x%x",
			capture_mem[i].cap_raw.addr_phy.addr_y,
			capture_mem[i].cap_raw.addr_phy.addr_u,
			capture_mem[i].cap_raw.addr_vir.addr_y,
			capture_mem[i].cap_raw.addr_vir.addr_u,
			capture_mem[i].cap_raw.buf_size);

		CMR_LOGI("target_yuv, phy 0x%x 0x%x, vir 0x%x 0x%x, size 0x%x",
			capture_mem[i].target_yuv.addr_phy.addr_y,
			capture_mem[i].target_yuv.addr_phy.addr_u,
			capture_mem[i].target_yuv.addr_vir.addr_y,
			capture_mem[i].target_yuv.addr_vir.addr_u,
			capture_mem[i].target_yuv.buf_size);

		CMR_LOGI("cap_yuv, phy 0x%x 0x%x, vir 0x%x 0x%x, size 0x%x",
			capture_mem[i].cap_yuv.addr_phy.addr_y,
			capture_mem[i].cap_yuv.addr_phy.addr_u,
			capture_mem[i].cap_yuv.addr_vir.addr_y,
			capture_mem[i].cap_yuv.addr_vir.addr_u,
			capture_mem[i].cap_yuv.buf_size);	
	}
	/*Alloc other image buffer include RAW and CAP YUV , End*/

	return 0;
}


uint32_t get_jpeg_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height)
{
	uint32_t       size;
	(void)thum_width; (void)thum_height;

	size = CMR_JPEG_SZIE(width, height)+JPEG_EXIF_SIZE;

	return ADDR_BY_WORD(size);
}
uint32_t get_thum_yuv_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height)
{
	(void)width; (void)height;
	return (uint32_t)(thum_width * thum_height * 3 / 2);
}
uint32_t get_thum_jpeg_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height)
{
	uint32_t       size;

	(void)width; (void)height;
	size = CMR_JPEG_SZIE(thum_width, thum_height);
	return ADDR_BY_WORD(size);
}

uint32_t get_jpg_tmp_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height)
{
	(void)thum_width; (void)thum_height;
	return (uint32_t)(width * CMR_SLICE_HEIGHT * 2); // TBD
}
uint32_t get_scaler_tmp_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height)
{
	uint32_t   slice_buffer;

	(void)thum_width; (void)thum_height;

	slice_buffer = (uint32_t)(width * CMR_SLICE_HEIGHT * CMR_ZOOM_FACTOR);//only UV422 need dwon sample to UV420

	return slice_buffer;
}

uint32_t get_isp_tmp_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height)
{
	uint32_t   slice_buffer;

	(void)thum_width; (void)thum_height;

	slice_buffer = (uint32_t)(width * CMR_SLICE_HEIGHT);//only UV422 need dwon sample to UV420

	return slice_buffer;
}


