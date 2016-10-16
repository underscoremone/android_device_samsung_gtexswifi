/*
* hardware/sprd/hsdroid/libcamera/sprdoemcamera.h
 * Dcam HAL based on sc8800g2
 *
 * Copyright (C) 2011 Spreadtrum 
 * 
 * Author: Xiaozhe wang <xiaozhe.wang@spreadtrum.com>
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
#ifndef ANDROID_HARDWARE_SPRD_OEM_CAMERA_H
#define ANDROID_HARDWARE_SPRD_OEM_CAMERA_H

#include <sys/types.h>
#include <utils/Log.h>


#include "SprdCameraIfc.h"
#include "SprdCameraHardwareInterface.h"

namespace android {

#define JPEG_OUT_ALIGN_W(x)	((((x)+15)/16)*16)
#define JPEG_OUT_ALIGN_H(x)	((((x)+7)/8)*8)

#define CAM_OUT_YUV420_UV 1 //wxz20120420: 0: the YUV420 planer of the camera output is Y_VU; 1: the YUV420 planer of the camera output is Y_UV
typedef enum
{
	SENSOR_IMAGE_FORMAT_YUV422 = 0,
	SENSOR_IMAGE_FORMAT_YUV420,
	SENSOR_IMAGE_FORMAT_RAW,
	SENSOR_IMAGE_FORMAT_RGB565,
	SENSOR_IMAGE_FORMAT_RGB666,
	SENSOR_IMAGE_FORMAT_RGB888,
	SENSOR_IMAGE_FORMAT_CCIR656,
	SENSOR_IMAGE_FORMAT_JPEG,
	
	SENSOR_IMAGE_FORMAT_MAX
}SENSOR_IMAGE_FORMAT_E;

typedef struct
{
	uint16_t mode;
	uint16_t width;
	uint16_t height;
	uint16_t trim_start_x;
	uint16_t trim_start_y;
	uint16_t trim_width;
	uint16_t trim_height;
	SENSOR_IMAGE_FORMAT_E image_format;
}SENSOR_MODE_INFO_T;

void camera_assoc_pmem(qdsp_module_type module,
                                  int pmem_fd, 
                                  void *addr,
                                  uint32_t length,
                                  int external);
void clear_module_pmem(qdsp_module_type module);
int camera_release_pmem(qdsp_module_type module,
                                   void *addr,
                                   uint32_t size,
                                   uint32_t force);
camera_ret_code_type camera_encode_picture(
        camera_frame_type *frame,
        camera_handle_type *handle,
        camera_cb_f_type callback,
        void *client_data);

camera_ret_code_type camera_init(int32_t camera_id);
void camera_af_init(void);
camera_ret_code_type camera_release_frame(uint32_t index);
camera_ret_code_type camera_set_dimensions (
        uint16_t picture_width,
        uint16_t picture_height,
        uint16_t display_width,
#ifdef FEATURE_CAMERA_V7
        uint16_t display_height,
#endif
        camera_cb_f_type callback,
        void *client_data);
camera_ret_code_type camera_set_encode_properties(
        camera_encode_properties_type *encode_properties);
camera_ret_code_type camera_set_parm(
        camera_parm_type id,
        int32_t          parm,
        camera_cb_f_type callback,
        void            *client_data);
camera_ret_code_type camera_set_position(
        camera_position_type *position,
        camera_cb_f_type      callback,
        void                 *client_data);
camera_ret_code_type camera_set_thumbnail_properties (
                              uint32_t width,
                              uint32_t height,
                              uint32_t quality);
camera_ret_code_type camera_start (
        camera_cb_f_type callback,
        void *client_data
#ifdef FEATURE_NATIVELINUX
        ,int  display_height,
        int  display_width
#endif // FEATURE_NATIVELINUX 
        );
camera_ret_code_type camera_start_preview (
        camera_cb_f_type callback,
        void *client_data);
void* camera_start_focus (void *client_data);

camera_ret_code_type camera_cancel_autofocus (void);
camera_ret_code_type camera_stop(
        camera_cb_f_type callback,
        void *client_data);
camera_ret_code_type camera_stop_preview (void);
camera_ret_code_type camera_stop_capture (void);
camera_ret_code_type camera_take_picture (
        camera_cb_f_type    callback,
        void               *client_data
#if !defined FEATURE_CAMERA_ENCODE_PROPERTIES && defined FEATURE_CAMERA_V7
        ,camera_raw_type camera_raw_mode
#endif // nFEATURE_CAMERA_ENCODE_PROPERTIES && FEATURE_CAMERA_V7 
        );
void rex_start();
void rex_shutdown();
uint32_t camera_get_size_align_page(uint32_t size);
uint32_t camera_get_frame_size(uint32_t width, uint32_t height, uint32_t type);
uint32_t camera_get_swap_buf_size(uint32_t tar_width);
void camera_alloc_zoom_buffer(uint32_t phy_addr, uint8_t *vir_addr,uint32_t size);
void camera_alloc_swap_buffer(uint32_t phy_addr, uint32_t buf_size);
void camera_get_sensor_max_size(uint32_t *width_ptr,uint32_t *height_ptr);
void camera_get_sensor_mode(void);
SENSOR_IMAGE_FORMAT_E camera_get_capture_format(uint32_t width);
int camera_start_af_thread(camera_focus_e_type focus,
        									      camera_cb_f_type callback,
       									       void *client_data);
void camera_encoder_start_flash(void);
int camera_get_data_redisplay(int output_addr, int output_width, int output_height, int input_addr, int input_width, int input_height);
int camera_rotation_copy_data(uint32_t width, uint32_t height, uint32_t in_addr, uint32_t out_addr);
#ifndef USE_ION_MEM
int camera_rotation_copy_data_virtual(uint32_t width, uint32_t height, uint32_t in_addr, uint32_t out_addr);
#endif
int camera_convert_420_UV_VU(int src_addr,int dst_addr,int width,int height);
void camera_set_preview_mode(int mode);
};
#endif //ANDROID_HARDWARE_SPRD_OEM_CAMERA_H
