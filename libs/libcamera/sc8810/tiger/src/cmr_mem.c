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
#include <unistd.h>
/*
          to add more.........
     8M-----------16M-------------8M
     5M-----------16M-------------0M
     3M-----------8M--------------2M
     2M-----------4M--------------2M
     1.3M---------4M--------------1M
*/

#define RAGRGB_BIT_WIDTH          10
#define PIXEL_1P3_MEGA            0x180000 //actually 1.5 *1024*1024
#define PIXEL_2P0_MEGA            0x200000 //actually 2.0 *1024*1024
#define PIXEL_3P0_MEGA            0x300000 //actually 3.0 *1024*1024
#define PIXEL_5P0_MEGA            0x500000 //5.0 *1024*1024
#define PIXEL_8P0_MEGA            0x800000 //8.0 *1024*1024

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
	BUF_TYPE_NUM
};


typedef uint32_t (*cmr_get_size)(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height);
	
struct cap_size_to_mem {
	uint32_t    pixel_num;
	uint32_t    major_size;
	uint32_t    minor_size;
};

static const struct cap_size_to_mem mem_size_tab[IMG_SIZE_NUM] = {
	{PIXEL_1P3_MEGA, (4 << 20),  (1 << 20)},
	{PIXEL_2P0_MEGA, (4 << 20),  (2 << 20)},
	{PIXEL_3P0_MEGA, (8 << 20),  (2 << 20)},
	{PIXEL_5P0_MEGA, (16 << 20), (0 << 20)},
	{PIXEL_8P0_MEGA, (16 << 20), (8 << 20)},
};

static uint32_t get_jpeg_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height);
static uint32_t get_thum_yuv_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height);
static uint32_t get_thum_jpeg_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height);
static uint32_t get_jpg_tmp_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height);
static uint32_t get_scaler_tmp_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height);

static const cmr_get_size get_size[BUF_TYPE_NUM] = {
	get_jpeg_size,
	get_thum_yuv_size,
	get_thum_jpeg_size,
	get_jpg_tmp_size,
	get_scaler_tmp_size
};

int camera_capture_buf_size(struct img_size *image_size, 
									uint32_t *size_major, 
									uint32_t *size_minor)
{
	uint32_t     size_pixel = (uint32_t)(image_size->width * image_size->height);
	int          i;
	
	if (NULL == image_size || 
		NULL == size_major || 
		NULL == size_minor) {
		CMR_LOGE("Parameter error 0x%x 0x%x 0x%x", (uint32_t)image_size, (uint32_t)size_major, (uint32_t)size_minor);
		return -1;
	}

	for (i = IMG_1P3_MEGA; i < IMG_SIZE_NUM; i++) {
		if (size_pixel <= mem_size_tab[i].pixel_num)
			break;
	}

	if (i == IMG_SIZE_NUM) {
		CMR_LOGE("No matched size for this image, 0x%x", size_pixel);
		return -1;
	} else {
		CMR_LOGV("image size num, %d, major mem 0x%x, minor mem 0x%x",
			i,
			mem_size_tab[i].major_size,
			mem_size_tab[i].minor_size);
	}

	*size_major = mem_size_tab[i].major_size;
	*size_minor = mem_size_tab[i].minor_size;

	return 0;
}

int camera_arrange_capture_buf(struct cmr_cap_2_frm *cap_2_frm,
								struct img_size *image_size, 
								struct img_size *thum_size, 
								struct cmr_cap_mem *capture_mem)
{
	uint32_t       channel_size = (uint32_t)(image_size->width * image_size->height);
	uint32_t       size_pixel = channel_size;
	uint32_t       res_size = cap_2_frm->major_frm.buf_size;
	uint32_t       addr_tmp_phy, addr_tmp_vir, i;
	struct img_frm img_frame[BUF_TYPE_NUM];

	if (NULL == cap_2_frm ||
		NULL == image_size ||
		NULL == thum_size ||
		NULL == capture_mem) {
		CMR_LOGE("Parameter error 0x%x 0x%x 0x%x 0x%x", 
			(uint32_t)cap_2_frm, 
			(uint32_t)image_size, 
			(uint32_t)thum_size, 
			(uint32_t)capture_mem);
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

	CMR_LOGV("channel_size, 0x%x", channel_size);

	size_pixel = (size_pixel << 1);
	
	if (res_size < size_pixel) {
		CMR_LOGE("Unsupported image size %d %d", image_size->width, image_size->height);
		return -1;
	}

	/*set the img_length as image_size->width * image_size->height, then the RawRGB 10Bit will be 1.25 * img_length,

	the UV channel will be 0.5 * img_length.

	|------------------------|                 |------------------------|                 |------------------------|       
	|     0.25*img_length    |                 |     0.25*img_length    |                 |************************| 
	|------------------------|                 |------------------------|                 |************************| 
	|************************|                 |************************|                 |****** Y Channel *******| 
	|************************|                 |************************|                 |************************| 
	|******** Raw RGB *******|           \     |****** Y Channel *******|           \     |**** 1.00*img_length ***| 
	|************************|    --------\    |************************|    --------\    |************************| 
	|************************|    ---------    |************************|    ---------    |************************| 
	|**** 1.25*img_length ***|    --------/    |**** 1.00*img_length ***|    --------/    |************************| 
	|************************|           /     |************************|           /     |------------------------| 
	|************************|	           |************************|                 |    0.25*img_length     | 
	|************************|                 |------------------------|                 |------------------------| 
	|************************|                 |    0.25*img_length     |                 |****** UV Channel ******| 
	|------------------------|                 |------------------------|                 |***** 0.5*img_length ***| 
	|                        |                 |****** UV Channel ******|                 |************************| 
	|     0.5*img_length     |                 |***** 0.5*img_length ***|                 |------------------------| 
	|                        |                 |************************|                 |    0.25*img_length     | 
	|------------------------|                 |------------------------|                 |------------------------| 

	
	*/

	/* calculate the address of capture image, start */
	/* first, seek to the end of the major buffer*/
	addr_tmp_phy = cap_2_frm->major_frm.addr_phy.addr_y + size_pixel;
	addr_tmp_vir = cap_2_frm->major_frm.addr_vir.addr_y + size_pixel;

	/* Get the capture UV buffer */
	capture_mem->cap_yuv.buf_size = (channel_size * 3) >> 1;
	capture_mem->cap_yuv.addr_phy.addr_u = addr_tmp_phy - (channel_size >> 1);
	capture_mem->cap_yuv.addr_vir.addr_u = addr_tmp_vir - (channel_size >> 1);

	/* Get the capture RagRGB buffer */
	capture_mem->cap_raw.buf_size = (uint32_t)(channel_size * RAGRGB_BIT_WIDTH / 8);
	capture_mem->cap_raw.addr_phy.addr_y = capture_mem->cap_yuv.addr_phy.addr_u - capture_mem->cap_raw.buf_size;
	capture_mem->cap_raw.addr_vir.addr_y = capture_mem->cap_yuv.addr_vir.addr_u - capture_mem->cap_raw.buf_size;
	capture_mem->cap_raw.size.width      = image_size->width;
	capture_mem->cap_raw.size.height     = image_size->height;
	capture_mem->cap_raw.fmt             = IMG_DATA_TYPE_RAW;

	/* Get the capture Y buffer, same to RawRGB buffer */
	capture_mem->cap_yuv.addr_phy.addr_y = capture_mem->cap_raw.addr_phy.addr_y;
	capture_mem->cap_yuv.addr_vir.addr_y = capture_mem->cap_raw.addr_vir.addr_y;
	capture_mem->cap_yuv.size.width      = image_size->width;
	capture_mem->cap_yuv.size.height     = image_size->height;
	capture_mem->cap_yuv.fmt             = IMG_DATA_TYPE_YUV420;
	/* Get the Target Y/UV buffer */
	capture_mem->target_yuv.buf_size = (channel_size * 3) >> 1;
	capture_mem->target_yuv.addr_phy.addr_u = capture_mem->cap_yuv.addr_phy.addr_y + channel_size;
	capture_mem->target_yuv.addr_vir.addr_u = capture_mem->cap_yuv.addr_vir.addr_y + channel_size;
	capture_mem->target_yuv.addr_phy.addr_y = cap_2_frm->major_frm.addr_phy.addr_y;
	capture_mem->target_yuv.addr_vir.addr_y = cap_2_frm->major_frm.addr_vir.addr_y;
	capture_mem->target_yuv.size.width      = image_size->width;
	capture_mem->target_yuv.size.height     = image_size->height;
	capture_mem->target_yuv.fmt             = IMG_DATA_TYPE_YUV420;

	/* re-calculate the currend end of major buffer */
	res_size = res_size - size_pixel;

	CMR_LOGV("Now to alloc misc buffers");
	for(i = JPEG_TARGET; i < BUF_TYPE_NUM; i++) {
		/* calculate the address of target_jpeg, start */
		size_pixel = get_size[i](image_size->width, image_size->height, thum_size->width, thum_size->height);
		if (res_size >= size_pixel) {
			img_frame[i].buf_size = size_pixel;
			img_frame[i].addr_phy.addr_y = addr_tmp_phy;
			img_frame[i].addr_vir.addr_y = addr_tmp_vir;
			img_frame[i].addr_phy.addr_u = addr_tmp_phy + size_pixel * 2 / 3;
			img_frame[i].addr_vir.addr_u = addr_tmp_vir + size_pixel * 2 / 3;
			
			/* re-calculate the currend end of major buffer */
			res_size -= size_pixel;
			addr_tmp_phy += size_pixel;
			addr_tmp_vir += size_pixel;
		} else {
			break;
		}
	}

	if (i != BUF_TYPE_NUM) {
		CMR_LOGV("No more memory reseved in Major buffer, to alloc misc buffers from Minor buffer");
		/* Not all the misc buffer have been alloc-ed yet, get memory from minor_frm */
		addr_tmp_phy = cap_2_frm->minor_frm.addr_phy.addr_y;
		addr_tmp_vir = cap_2_frm->minor_frm.addr_vir.addr_y;
		res_size     = cap_2_frm->minor_frm.buf_size;
		for (; i < BUF_TYPE_NUM; i++) {
			/* calculate the address of target_jpeg, start */
			size_pixel = get_size[i](image_size->width, image_size->height, thum_size->width, thum_size->height);
			if (res_size >= size_pixel) {

				img_frame[i].buf_size = size_pixel;
				img_frame[i].addr_phy.addr_y = addr_tmp_phy;
				img_frame[i].addr_vir.addr_y = addr_tmp_vir;
				img_frame[i].addr_phy.addr_u = addr_tmp_phy + size_pixel * 2 / 3;
				img_frame[i].addr_vir.addr_u = addr_tmp_vir + size_pixel * 2 / 3;
				
				/* re-calculate the currend end of major buffer */
				res_size -= size_pixel;
				addr_tmp_phy += size_pixel;
				addr_tmp_vir += size_pixel;
			} else {
				break;
			}

		}
	}

	if (i != BUF_TYPE_NUM) {
		CMR_LOGE("Failed to alloc all the buffers used in capture");
	}

	channel_size = (uint32_t)(image_size->width * image_size->height);
	size_pixel = (channel_size * 3) >> 1;

	if (res_size >= size_pixel) {
		CMR_LOGE("The left memory can be used as Rot buffer, 0x%x", res_size);
		capture_mem->cap_yuv_rot.addr_phy.addr_y = addr_tmp_phy;
		capture_mem->cap_yuv_rot.addr_vir.addr_y = addr_tmp_vir;
		capture_mem->cap_yuv_rot.addr_phy.addr_u = addr_tmp_phy + channel_size;
		capture_mem->cap_yuv_rot.addr_vir.addr_u = addr_tmp_vir + channel_size;
		capture_mem->cap_yuv_rot.size.width      = image_size->height;
		capture_mem->cap_yuv_rot.size.height     = image_size->width;
		capture_mem->cap_yuv_rot.buf_size        = size_pixel;
		capture_mem->cap_yuv_rot.fmt             = IMG_DATA_TYPE_YUV420;
	} else {
		memset(&capture_mem->cap_yuv_rot, 0, sizeof(struct img_frm));
	}

	capture_mem->target_jpeg.buf_size = img_frame[JPEG_TARGET].buf_size;
	capture_mem->target_jpeg.addr_phy.addr_y = img_frame[JPEG_TARGET].addr_phy.addr_y;
	capture_mem->target_jpeg.addr_vir.addr_y = img_frame[JPEG_TARGET].addr_vir.addr_y;

	capture_mem->thum_yuv.buf_size = img_frame[THUM_YUV].buf_size;
	capture_mem->thum_yuv.addr_phy.addr_y = img_frame[THUM_YUV].addr_phy.addr_y;
	capture_mem->thum_yuv.addr_vir.addr_y = img_frame[THUM_YUV].addr_vir.addr_y;
	capture_mem->thum_yuv.addr_phy.addr_u = img_frame[THUM_YUV].addr_phy.addr_u;
	capture_mem->thum_yuv.addr_vir.addr_u = img_frame[THUM_YUV].addr_vir.addr_u;
	capture_mem->thum_yuv.size.width      = thum_size->width;
	capture_mem->thum_yuv.size.height     = thum_size->height;

	capture_mem->thum_jpeg.buf_size = img_frame[THUM_JPEG].buf_size;
	capture_mem->thum_jpeg.addr_phy.addr_y = img_frame[THUM_JPEG].addr_phy.addr_y;
	capture_mem->thum_jpeg.addr_vir.addr_y = img_frame[THUM_JPEG].addr_vir.addr_y;

	capture_mem->jpeg_tmp.buf_size = img_frame[JPEG_TMP].buf_size;
	capture_mem->jpeg_tmp.addr_phy.addr_y = img_frame[JPEG_TMP].addr_phy.addr_y;
	capture_mem->jpeg_tmp.addr_vir.addr_y = img_frame[JPEG_TMP].addr_vir.addr_y;

	capture_mem->scale_tmp.buf_size = img_frame[SCALER_TMP].buf_size;
	capture_mem->scale_tmp.addr_phy.addr_y = img_frame[SCALER_TMP].addr_phy.addr_y;
	capture_mem->scale_tmp.addr_vir.addr_y = img_frame[SCALER_TMP].addr_vir.addr_y;

	return 0;
}


uint32_t get_jpeg_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height)
{
	(void)thum_width; (void)thum_height;
	return CMR_JPEG_SZIE(width, height);
}
uint32_t get_thum_yuv_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height)
{
	(void)width; (void)height;
	return (uint32_t)(thum_width * thum_height * 3 / 2);
}
uint32_t get_thum_jpeg_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height)
{
	(void)width; (void)height;
	return CMR_JPEG_SZIE(thum_width, thum_height);
}

uint32_t get_jpg_tmp_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height)
{
	(void)thum_width; (void)thum_height;
	return (uint32_t)(width * 128); // TBD 
}
uint32_t get_scaler_tmp_size(uint32_t width, uint32_t height, uint32_t thum_width, uint32_t thum_height)
{
	uint32_t   line_buffer, swap_buffer;

	(void)thum_width; (void)thum_height;

	line_buffer = (uint32_t)(width * 8);
	swap_buffer = (uint32_t)(width * CMR_SLICE_HEIGHT * 2);
	
	return (line_buffer + swap_buffer);
}


