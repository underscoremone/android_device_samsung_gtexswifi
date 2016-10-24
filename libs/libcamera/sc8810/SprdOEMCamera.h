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
camera_ret_code_type camera_start_focus (
        camera_focus_e_type focus,
        camera_cb_f_type callback,
        void *client_data);
camera_ret_code_type camera_stop_focus (void);
camera_ret_code_type camera_stop(
        camera_cb_f_type callback,
        void *client_data);
camera_ret_code_type camera_stop_preview (void);
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
void camera_alloc_zoom_buffer(uint32_t phy_addr, uint8_t *vir_addr,uint32_t size);
void camera_alloc_swap_buffer(uint32_t phy_addr);
};
#endif //ANDROID_HARDWARE_SPRD_OEM_CAMERA_H
