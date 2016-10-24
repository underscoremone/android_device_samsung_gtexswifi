/*
* hardware/sprd/hsdroid/libcamera/sprdOEMcamera.cpp
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
#define LOG_NDEBUG 0
#define LOG_TAG "SprdCameraOEM"
//#include <linux/delay.h>
#include <stdlib.h>
#include <fcntl.h>              /* low-level i/o */
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <unistd.h>
#include <math.h>

#include "../sc8810/SprdOEMCamera.h"
#include "jpeg_exif_header.h"
#include "sprd_scale.h"
#include "sprd_rotation.h"
#include "jpegenc_api.h"
#include "SprdCameraHardwareCapability.h"
#include "jpegdec_api.h"


namespace android{

#define PREVIEW_BUF_NUM 8
#define CAPTURE_BUF_NUM 1
#define JPEGENC_BUF_NUM 1
#define JPEGDECENC_Y_UV_OFFSET_MAX  0x400000

#define PIXEL_ALIGNED 16
#define W_H_ALIGNED(x) (((x) + PIXEL_ALIGNED - 1) & ~(PIXEL_ALIGNED - 1))

#define SCALE_SLICE_HEIGHT 128
#define SCALE_OUT_WIDTH_MAX 960

#define PREVIEW_ENDIAN_H 1
#define JPEG_DIV                       4

typedef struct zoom_trim_rect{
	uint32_t x;
	uint32_t y;
	uint32_t w;
	uint32_t h;
}ZOOM_TRIM_RECT_T;

typedef struct pmem_info
{
	qdsp_module_type module;
       	 int pmem_fd; //pmem driver handle
        void *addr; //ALOGIcal address
        uint32_t length; //buffer size
        int external;
}PMEM_INFO_T;

typedef struct dcam_params
{
	uint32_t preview_mode;
	uint32_t orientation; //wxz20110815: 0: default, landscape; 1: portrait
	uint32_t orientation_parm;
	uint32_t is_2M_to_3M; //wxz20110825: 0: not need 2M to 3M; 1: need 2M to 3M in capture mode.
	uint32_t focus_mode;
	uint32_t flash_en;
}CAM_PARAMS_T;

typedef struct thumbnail_properties
{
	uint32_t width;
        uint32_t height;
        uint32_t quality;
}THUMBNAIL_PROPERTIES_T;

typedef struct encode_properties{
   	 int32_t quality;
    	camera_encode_type format;
   	 int32_t file_size;
} ENCODE_PROPERTIES_T;
typedef struct dcam_dimensions
{
	uint16_t picture_width;
        uint16_t picture_height;
        uint16_t display_width;
        uint16_t display_height;
}DCAM_DIMENSIONS_T;
struct buffer {
    uint32_t *virt_addr;
    uint32_t phys_addr;
    uint32_t *u_virt_addr;
    uint32_t u_phys_addr;
    uint32_t *v_virt_addr;
    uint32_t v_phys_addr;
    uint32_t length;//buffer's length is different from cap_image_size
};
typedef enum
{
	DCAM_DATA_YUV422 = 0,
	DCAM_DATA_YUV420,
	DCAM_DATA_RGB,
	DCAM_DATA_MAX
}DCAM_DATA_FORMAT_E;

typedef struct
{
	uint32_t cap_mem_size;
	uint32_t dcam_out_width;
	uint32_t dcam_out_height;
	uint32_t is_need_rotation;
	uint32_t rot_angle;
	uint32_t  is_interpolation;
	uint32_t is_zoom;
	uint32_t jpg_len;
	uint32_t alloc_jpg_mem_size;
	uint32_t jpg_out_align_w;
	uint32_t jpg_out_align_h;
	uint32_t jpg_out_y_phy_addr;
	uint32_t *jpg_out_y_virt_addr;
	uint32_t jpg_out_uv_phy_addr;
	uint32_t *jpg_out_uv_virt_addr;
	uint32_t jpg_y_temp_phy_addr; //y ping-poong buffer of jpeg decoder and jpeg encoder
	uint32_t *jpg_y_temp_virt_addr;
	uint32_t jpg_uv_temp_phy_addr;//uv ping-poong buffer of jpeg decoder and jpeg encoder
	uint32_t *jpg_uv_temp_virt_addr;
	uint32_t jpg_enc_y_temp_phy_addr;
	uint32_t* jpg_enc_y_temp_virt_addr;
	uint32_t jpg_enc_uv_temp_phy_addr;
	uint32_t *jpg_enc_uv_temp_virt_addr;
	uint32_t jpeg_codec_slice_height;
	uint32_t jpeg_buf_setting_flag;
	uint32_t jpg_record_copy_height;
	uint32_t set_encode_rotation;
	uint32_t temp_y_phy_addr;
	uint32_t temp_uv_phy_addr;
	uint32_t *temp_y_virt_addr;
	uint32_t *temp_uv_virt_addr;
	DCAM_DATA_FORMAT_E out_format;
	SENSOR_IMAGE_FORMAT_E sensor_out_format;
	char datetime_buf[20];
	char gps_date_buf[12];
	uint32_t focal_length;
	SENSOR_MODE_INFO_T sensor_mode_info[10];
	camera_frame_type frame_info;
}SPRD_OEMCAMERA_INFO_T;

#define G_PREVIEW_BUF_OFFSET 0 // 2 //the freview buffer from this number
#define G_CAPTURE_BUF_OFFSET 0 //the capture buffer from this number
struct buffer * g_buffers = NULL;   //the global buffers for preview and capture; the buffer 0 is the output buffer.
static uint32_t n_buffers = 0; //restore the number of buffers


#define CLEAR(x) memset (&(x), 0, sizeof (x))
#define PMEM_INFO_NUM 45
//store the pmem info: file handle, ALOGIcal address of buffer, buffer size.
PMEM_INFO_T g_pmem_info[PMEM_INFO_NUM];
CAM_PARAMS_T g_cam_params;
THUMBNAIL_PROPERTIES_T g_thumbnail_properties;
camera_position_type g_position;
ENCODE_PROPERTIES_T g_encode_properties;
DCAM_DIMENSIONS_T g_dcam_dimensions;
camera_cb_f_type g_callback;
JPEGENC_CBrtnType g_encoder_param;
pthread_t g_preview_thr; //the thread pointer for preview processor.
pthread_t g_capture_thr; //the thread pointer for capture processor.
pthread_t g_encoder_thr; //the thread pointer for encoder processor.
pthread_t g_restart_thr;  //the thread pointer for preview restart processor.
pthread_t g_af_thr; //the thread pointer for autofocus processor.
uint32_t g_preview_buffer_num = PREVIEW_BUF_NUM; //the number of preview buffers
uint32_t g_capture_buffer_num = CAPTURE_BUF_NUM; //the number of capture buffers
uint32_t g_preview_phys_addr; //the start physical address of preview buffers
uint32_t *g_preview_virt_addr = NULL; //the start ALOGIcal address of preview buffers
uint32_t g_capture_phys_addr; //the start physical address of capture buffers
uint32_t *g_capture_virt_addr = NULL;//the start ALOGIcal address of capture buffers
uint32_t g_capture_size; //store the size of capture picture, such as YUV420
volatile static uint32_t g_stop_preview_flag = 0; //0: will not stop the preview thread; 1: will stop the preview thread
volatile static uint32_t g_preview_stop = 0; //0: the preview thread is not stop; 1: the preview thread is stop.
volatile static uint32_t g_stop_capture_flag = 0;//0: will not stop the capture thread; 1: will stop the capture thread
volatile static uint32_t g_capture_stop = 0;//0: the capture thread is not stop; 1: the capture thread is stop.
SprdCameraHardware *g_dcam_obj = NULL;//store the SprdCameraHardware object
uint32_t g_buf_flag[3];
int32_t g_camera_id;
static uint32_t g_releasebuff_index = 0; //store the rellease buffer index.
camera_encode_mem_type g_encoder_type;
static char dev_name[50] = "/dev/video0";
static int fd = -1;
void *g_client_data = NULL; //fro jpeg encoder
JPEGENC_PARAMS_T g_jpegenc_params;
uint32_t g_jpeg_stream_size = 0; //store the size of jpeg picture by jpeg encoder.
uint32_t g_hal_zoom_level = 0;//zoom level: 0, 1, 2., 3
uint32_t g_cap_zoom_buf_phy_addr = 0;
uint8_t *g_cap_zoom_buf_vir_addr = NULL;
uint32_t g_cap_zoom_buf_size = 0;
uint32_t g_slice_swap_buf = 0;
uint32_t g_slice_swap_buf_size = 0;
JPEGENC_QUALITY_E g_jpeg_quality_level = JPEGENC_QUALITY_HIGH;//jpeg quality level for jpeg encoder.
uint32_t g_rotation = 0; //wxz20110725: 0, 90, 180, 270, store the rotation agree for the preview and capture.
uint32_t g_rotation_parm = 0; //wxz20110725: 0, 90, 180, 270, store the rotation agree for the preview and capture.
static SPRD_OEMCAMERA_INFO_T  s_camera_info;
#define FOCUS_RECT_PARAM_LEN  200
static uint8_t s_focus_zone_param[FOCUS_RECT_PARAM_LEN];
static uint32_t s_af_is_stop = 1;
static uint32_t s_af_is_cancel = 0;
static uint32_t g_encoder_is_end = 1;

/* exif  info*/
LOCAL JINF_EXIF_INFO_T 			g_dc_exif_info;

LOCAL EXIF_GPS_INFO_T 			g_dc_gps_info;
LOCAL EXIF_SPEC_DATE_TIME_T 	g_image_date_time;
LOCAL EXIF_SPECIFIC_INFO_T 		g_dc_specific_info;
LOCAL EXIF_SPEC_OTHER_T 		g_dc_spec_other;
LOCAL EXIF_PRI_DATA_STRUCT_T 	g_exif_prim_data;

/* get from dc_product_cfg */
LOCAL EXIF_SPEC_USER_T                		*g_dc_spec_user_ptr;
LOCAL EXIF_PRI_DESC_T                 		*g_dc_primary_img_desc_ptr;
/* get from sensor_drv */
LOCAL EXIF_SPEC_PIC_TAKING_COND_T     	*g_dc_spec_pic_taking_cond_ptr;


static void close_device(void);
static int camera_copy(SCALE_DATA_FORMAT_E output_fmt, uint32_t output_width, uint32_t output_height, uint32_t output_y_addr,
	                                                        uint32_t output_uv_addr,ZOOM_TRIM_RECT_T *trim_rect, uint32_t input_yaddr, uint32_t input_uvaddr,
	                                                        SCALE_DATA_FORMAT_E input_fmt);
static int camera_scale_functions(SCALE_DATA_FORMAT_E output_fmt, uint32_t output_width, uint32_t output_height,
	                                                      uint32_t output_yaddr,uint32_t output_uvaddr,
	                                                      ISP_ENDIAN_T output_endian,
	                                                      ZOOM_TRIM_RECT_T *trim_rect,
	                                                      uint32_t input_width,uint32_t input_height,uint32_t input_yaddr,
	                                                      uint32_t intput_uvaddr, SCALE_DATA_FORMAT_E input_fmt,
	                                                      ISP_ENDIAN_T input_endian);
static void errno_exit(const char * s)
{
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}
static int xioctl(int fd, int request, void * arg)
{
	int r;
	r = ioctl(fd, request, arg);
	return r;
}

static int device_write(int fd, uint8_t *buf, uint32_t count)
{
        int r;
        r = write(fd, buf, count);
        return r;
}

uint32_t camera_get_size_align_page(uint32_t size)
{
	uint32_t buffer_size, page_size;
	page_size = getpagesize();
	buffer_size = size;
	buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

	return buffer_size;
}
uint32_t camera_get_frame_size(uint32_t width, uint32_t height, uint32_t type)
{
	uint32_t frame_size = 0;
	switch(type)
	{
	case V4L2_PIX_FMT_RGB32:
		frame_size = width * height * 4;
		break;
	case V4L2_PIX_FMT_RGB565X:
		frame_size = width * height * 2;
		break;
	case V4L2_PIX_FMT_YUYV:
		frame_size = width * height * 2;
		break;
	case V4L2_PIX_FMT_YUV420:
		frame_size = width * height * 3 / 2;
		break;
	default:
		frame_size = width * height * 2;
		break;
	}
	return frame_size;
}

void camera_assoc_pmem(qdsp_module_type module,
                                  int pmem_fd,
                                  void *addr,
                                  uint32_t length,
                                  int external)
{
	return;
}

void clear_module_pmem(qdsp_module_type module)
{
	uint32_t i;

	if(module >= QDSP_MODULE_MAX)
	{
		ALOGE("Fail to clear_module_pmem: the module is invalid.");
		return;
	}
	for(i = 0; i < PMEM_INFO_NUM; i++)
	{
		if(i == (uint32_t)module)
		{
			memset(g_pmem_info[i].addr, 0, g_pmem_info[i].length);
			break;
		}
	}
	return;
}
int camera_release_pmem(qdsp_module_type module,
                                   void *addr,
                                   uint32_t size,
                                   uint32_t force)
{
	return 0;
}

#ifdef CONFIG_CAMERA_ROTATION
static uint32_t s_front_cam_orientation = 1;/*need to rotate*/
#else
static uint32_t s_front_cam_orientation = 0;
#endif

void camera_set_rot_angle(uint32_t *is_set_orientation,uint32_t *angle)
{
	uint32_t temp = *angle;
	ALOGV("wjp:s_front_cam_orientation=%d.\n",s_front_cam_orientation);
	if((1 != g_camera_id) || (0 == s_front_cam_orientation))
		return;
	switch(temp){
	case 0:
			*angle = 90;
			*is_set_orientation = 1;
			break;
	case 90:
			*angle = 0;
			*is_set_orientation = 0;
			break;
	case 180:
			*angle = 270;
			*is_set_orientation = 1;
			break;
	case 270:
			*angle = 180;
			*is_set_orientation = 1;
			break;
	default:
			break;
	}
	//ALOGV("SPRD OEM:camera_set_rot_angle,angle=%d,is_set_orientation=%d.\n",*angle,*is_set_orientation);
}

void camera_encoder_callback(uint32_t buf_id, uint32_t stream_size, uint32_t is_last_slice)
{
	ALOGV("camera_encoder_callback: buf_id,  stream_size,  is_last_slice:%d, %d, %d. ", buf_id, stream_size, is_last_slice);
	g_encoder_param.header_size = 0;
	g_encoder_param.mode = JPEGENC_MEM;
	g_encoder_type.buffer = (uint8_t *)g_jpegenc_params.stream_virt_buf[buf_id];
	g_encoder_param.outPtr = &g_encoder_type;
	g_encoder_param.status = JPEGENC_IMG_DONE;
	ALOGV("outPtr: 0x%x, size: 0x%x.", (uint32_t)g_encoder_param.outPtr, stream_size);

	if(1 != is_last_slice){
		g_encoder_param.size = stream_size;
		g_callback(CAMERA_EXIT_CB_BUFFER, g_client_data, CAMERA_FUNC_ENCODE_PICTURE, (uint32_t)&g_encoder_param);
		g_jpeg_stream_size += stream_size;
	}
	else{
		g_encoder_param.size = stream_size - g_jpeg_stream_size;
		g_jpeg_stream_size = 0;
		g_callback(CAMERA_EXIT_CB_DONE, g_client_data, CAMERA_FUNC_ENCODE_PICTURE, (uint32_t)&g_encoder_param);
	}
}

static uint32_t getDataFromDouble(double d, uint32_t type) //0: dd, 1: mm, 2: ss
{
	d = fabs(d);
	double degrees = (int)d;
	double remainder = d - degrees;
	double minutes = (int)(remainder * 60.0);
	double seconds = (int)(((remainder * 60.0) - minutes) * 60.0);
	uint32_t retVal = 0;
	if(0 == type)
	{
		retVal = (int)degrees;
	}
	else if(1 == type)
	{
		retVal = (int)minutes;
	}
	else if(2 == type)
	{
		retVal = (int)seconds;
	}
	ALOGE("SPRD OEM:getDataFromDouble,GPS: type: %d, ret: 0x%x.", type, retVal);
	return retVal;
}

static void getSecondsFromDouble(double d, uint32_t *numerator, uint32_t *denominator)
{
	d = fabs(d);
	double degrees = (int)d;
	double remainder = d - degrees;
	double minutes = (int)(remainder * 60.0);
	double seconds = (((remainder * 60.0) - minutes) * 60.0);
	uint32_t i = 0;
	uint32_t j = 0;
	uint32_t num = 0;
	char str[20];
	double value=1.0;

	sprintf(str,"%f",seconds);
	while(str[i++]!='.')
		;
	j = strlen(str)-1;
	while(str[j] == '0')
		--j;
	num = j - i + 1;
	ALOGE("SPRD OEM:getSecondsFromDouble: %s, i=%d, j=%d, num=%d \n", str, i, j, num);

	for(i=0; i<num; i++)
		value *=10.0;

	*numerator = seconds*value;
	*denominator = value;

	ALOGE("SPRD OEM:getSecondsFromDouble, data=%f, num=%d, denom=%d \n", seconds, *numerator, *denominator);
}

static uint32_t getOrientationFromRotationDegrees(int degrees)
{
	uint32_t orientation = 1;//ExifInterface.ORIENTATION_NORMAL;
	degrees %= 360;
	if (degrees < 0) degrees += 360;
	if(degrees < 45)
	{
		orientation = 1;//ExifInterface.ORIENTATION_NORMAL;
	}
	else if(degrees < 135)
	{
		orientation = 6;//ExifInterface.ORIENTATION_ROTATE_90;
	}
	else if(degrees < 225)
	{
		orientation = 3;//ExifInterface.ORIENTATION_ROTATE_180;
	}
	else
	{
		orientation = 8;//ExifInterface.ORIENTATION_ROTATE_270;
	}
	ALOGV("SPRD OEM:getOrientationFromRotationDegrees, rotation degrees: %d, orientation: %d.", degrees, orientation);
	return orientation;
}

void camera_jpegenc_write_ping_buffer(uint32_t y_ping_addr,uint32_t uv_ping_addr)
{
	ZOOM_TRIM_RECT_T trim_rect;
	uint32_t i=0;
	uint32_t size = 0;
	uint32_t slice_height = 1024;
	uint32_t *src_y_ptr = g_capture_virt_addr;
	uint32_t *src_uv_ptr = g_capture_virt_addr+g_dcam_dimensions.picture_width*g_dcam_dimensions.picture_height/4;
	uint32_t *dst_y_ptr = s_camera_info.jpg_enc_y_temp_virt_addr;
	uint32_t *dst_uv_ptr = s_camera_info.jpg_enc_uv_temp_virt_addr;

	if(0 == s_camera_info.jpeg_codec_slice_height)
	{
		ALOGV("SPRD OEM:camera_jpegenc_write_ping_buffer,don't need write ping buffer!");
		return;
	}

	if(0 != s_camera_info.jpeg_codec_slice_height)
	{
		slice_height = s_camera_info.jpeg_codec_slice_height;
	}
	size = g_dcam_dimensions.picture_width*slice_height;
	memcpy(dst_y_ptr,src_y_ptr,size);
	size = g_dcam_dimensions.picture_width;

	ALOGV("camera_jpegenc_write_ping_buffer slice_height=%d.",slice_height);

	memcpy(dst_uv_ptr,src_uv_ptr,size*slice_height/2);

	s_camera_info.jpg_record_copy_height += slice_height;
}

void camera_jpegenc_update_data_callback(uint32_t dst_y_virt_addr, uint32_t dst_uv_virt_addr, uint32_t dst_height)
{
	uint32_t i=0;
	uint32_t size=0;
	uint32_t slice_height=1024;
	uint32_t *src_y_ptr;
	uint32_t *dst_y_ptr = s_camera_info.jpg_enc_y_temp_virt_addr;
	uint32_t *src_uv_ptr;
	uint32_t *dst_uv_ptr = s_camera_info.jpg_enc_uv_temp_virt_addr;
	uint32_t src_y_phy_ptr;
	uint32_t src_uv_phy_ptr;
	uint32_t dst_y_phy_ptr = s_camera_info.jpg_enc_y_temp_phy_addr;
	uint32_t dst_uv_phy_ptr = s_camera_info.jpg_enc_uv_temp_phy_addr;
	ZOOM_TRIM_RECT_T trim_rect;

	if(0 != s_camera_info.jpeg_codec_slice_height)
	{
		slice_height = s_camera_info.jpeg_codec_slice_height;
	}

	size = g_dcam_dimensions.picture_width*s_camera_info.jpg_record_copy_height;
	src_y_ptr = g_capture_virt_addr+size/4;
	src_y_phy_ptr =  g_capture_phys_addr+size;
	size = size/2 + g_dcam_dimensions.picture_width*g_dcam_dimensions.picture_height;
	src_uv_ptr = g_capture_virt_addr+size/4;
         src_uv_phy_ptr = g_capture_phys_addr+size;
	if((s_camera_info.jpg_record_copy_height+slice_height)>g_dcam_dimensions.picture_height)
	{
		slice_height = g_dcam_dimensions.picture_height-s_camera_info.jpg_record_copy_height;
		if(0==slice_height)
		{
			ALOGV("SPRD OEM:camera_jpegenc_update_data_callback:copy fail!");
			return;
		}
	}
	ALOGV("SPRD OEM:camera_jpegenc_update_data_callback,slice_height=%d,copy height=%d .\n",slice_height,s_camera_info.jpg_record_copy_height);

	size = g_dcam_dimensions.picture_width*slice_height;

	memcpy(dst_y_ptr,src_y_ptr,size);
	memcpy(dst_uv_ptr,src_uv_ptr,size/2);

       trim_rect.x = 0;
	trim_rect.y = 0;
	trim_rect.w = g_dcam_dimensions.picture_width;
	trim_rect.h = slice_height;

	//camera_copy(SCALE_DATA_YUV420,trim_rect.w,trim_rect.h,dst_y_phy_ptr,dst_uv_phy_ptr,
	//	                   &trim_rect,src_y_phy_ptr,src_uv_phy_ptr,SCALE_DATA_YUV420);

	s_camera_info.jpg_record_copy_height += slice_height;
	ALOGV("SPRD OEM:camera_jpegenc_update_data_callback,jpg_record_copy_height=%d .\n",s_camera_info.jpg_record_copy_height);
}
static int camera_conver_jpegquailty(uint32_t quality)
{
	int ret;
	ALOGV("camera_conver_jpegquailty E.quality = %d.", quality);

	if(quality > 85){
		ret = JPEGENC_QUALITY_HIGH;
	}
	else if(quality > 75){
		ret = JPEGENC_QUALITY_MIDDLE_HIGH;
	}
	else { // if(quality > 70)
		ret = JPEGENC_QUALITY_MIDDLE;
	}

	ALOGV("camera_conver_jpegquailty X.");

	return ret;
}

void camera_exif_convert_ptr(JINF_EXIF_INFO_T *dc_exif_info_ptr)
{
	JINF_EXIF_INFO_T* g_exif_info_start_ptr=(JINF_EXIF_INFO_T* )dc_exif_info_ptr;

#if 0
	ALOGV("SPRD OEM: before: (%x), %x, %x, %x %x \n",
					g_exif_info_start_ptr, dc_exif_info_ptr->gps_ptr, dc_exif_info_ptr->primary.data_struct_ptr,
					dc_exif_info_ptr->spec_ptr, dc_exif_info_ptr->spec_ptr->other_ptr);
	ALOGV("SPRD OEM: before: (%x), %x, %x, %x %x \n",
					g_exif_info_start_ptr, dc_exif_info_ptr->primary.img_desc_ptr, dc_exif_info_ptr->spec_ptr->date_time_ptr,
					dc_exif_info_ptr->spec_ptr->user_ptr, dc_exif_info_ptr->spec_ptr->pic_taking_cond_ptr);
#endif

	g_jpegenc_params.dc_exif_info_ptr 		= &g_dc_exif_info;


	memcpy(&g_dc_exif_info, dc_exif_info_ptr, sizeof(JINF_EXIF_INFO_T));


	g_dc_spec_user_ptr 				= NULL;
	g_dc_primary_img_desc_ptr 		= NULL;
	g_dc_spec_pic_taking_cond_ptr		= NULL;

	if(NULL != dc_exif_info_ptr->gps_ptr)
	{
		dc_exif_info_ptr->gps_ptr 		= (EXIF_GPS_INFO_T *)((uint32_t)dc_exif_info_ptr->gps_ptr  + (uint32_t)g_exif_info_start_ptr);
		memcpy(&g_dc_gps_info, dc_exif_info_ptr->gps_ptr, sizeof(EXIF_GPS_INFO_T));
		g_dc_exif_info.gps_ptr 				= &g_dc_gps_info;
	}

	if(NULL != dc_exif_info_ptr->primary.data_struct_ptr)
	{
		dc_exif_info_ptr->primary.data_struct_ptr		= (EXIF_PRI_DATA_STRUCT_T *)((uint32_t)dc_exif_info_ptr->primary.data_struct_ptr + (uint32_t)g_exif_info_start_ptr);
		memcpy(&g_exif_prim_data, dc_exif_info_ptr->primary.data_struct_ptr, sizeof(EXIF_PRI_DATA_STRUCT_T));
		g_dc_exif_info.primary.data_struct_ptr 	= &g_exif_prim_data;
	}

	if(NULL != dc_exif_info_ptr->primary.img_desc_ptr)
	{
		dc_exif_info_ptr->primary.img_desc_ptr		= (EXIF_PRI_DESC_T *)((uint32_t)dc_exif_info_ptr->primary.img_desc_ptr + (uint32_t)g_exif_info_start_ptr);

		g_dc_primary_img_desc_ptr = (EXIF_PRI_DESC_T *)malloc(sizeof(EXIF_PRI_DESC_T));
		memcpy(g_dc_primary_img_desc_ptr, dc_exif_info_ptr->primary.img_desc_ptr , sizeof(EXIF_PRI_DESC_T));
		g_dc_exif_info.primary.img_desc_ptr = g_dc_primary_img_desc_ptr;
		ALOGV("g_dc_primary_img_desc_ptr = %x, img_desc_ptr=%x \n", (uint32)g_dc_primary_img_desc_ptr, (uint32)g_dc_exif_info.primary.img_desc_ptr);
	}

	if(NULL != dc_exif_info_ptr->spec_ptr)
	{
		/* change offset to real address */
		dc_exif_info_ptr->spec_ptr 		= (EXIF_SPECIFIC_INFO_T *)((uint32_t)dc_exif_info_ptr->spec_ptr + (uint32_t)g_exif_info_start_ptr);
		memcpy(&g_dc_specific_info, dc_exif_info_ptr->spec_ptr , sizeof(EXIF_SPECIFIC_INFO_T));
		g_dc_exif_info.spec_ptr				= &g_dc_specific_info;
		ALOGV("spec_ptr:  (%x), %x, %x, %x  \n",
					(uint32_t)&g_dc_specific_info,  (uint32_t)g_dc_specific_info.date_time_ptr,
					(uint32_t)g_dc_specific_info.user_ptr, (uint32_t)g_dc_specific_info.pic_taking_cond_ptr);

		if(NULL != dc_exif_info_ptr->spec_ptr->other_ptr)
		{
			dc_exif_info_ptr->spec_ptr->other_ptr 		= (EXIF_SPEC_OTHER_T *)((uint32_t)dc_exif_info_ptr->spec_ptr->other_ptr + (uint32_t)g_exif_info_start_ptr);
			memcpy(&g_dc_spec_other, dc_exif_info_ptr->spec_ptr->other_ptr , sizeof(EXIF_SPEC_OTHER_T));
			g_dc_exif_info.spec_ptr->other_ptr 		= &g_dc_spec_other;
		}

		if(NULL != dc_exif_info_ptr->spec_ptr->date_time_ptr)
		{
			dc_exif_info_ptr->spec_ptr->date_time_ptr 		= (EXIF_SPEC_DATE_TIME_T *)((uint32_t)dc_exif_info_ptr->spec_ptr->date_time_ptr + (uint32_t)g_exif_info_start_ptr);
			memcpy(&g_image_date_time, dc_exif_info_ptr->spec_ptr->date_time_ptr , sizeof(EXIF_SPEC_DATE_TIME_T));
			g_dc_exif_info.spec_ptr->date_time_ptr 	= &g_image_date_time;
			ALOGV("g_image_date_time = %x, date_time_ptr=%x \n", (uint32)&g_image_date_time, (uint32)g_dc_exif_info.spec_ptr->date_time_ptr);
		}

		if(NULL != dc_exif_info_ptr->spec_ptr->user_ptr)
		{
			dc_exif_info_ptr->spec_ptr->user_ptr 		= (EXIF_SPEC_USER_T *)((uint32_t)dc_exif_info_ptr->spec_ptr->user_ptr + (uint32_t)g_exif_info_start_ptr);
			g_dc_spec_user_ptr = (EXIF_SPEC_USER_T *)malloc(sizeof(EXIF_SPEC_USER_T));
			memcpy(g_dc_spec_user_ptr, dc_exif_info_ptr->spec_ptr->user_ptr , sizeof(EXIF_SPEC_USER_T));
			g_dc_specific_info.user_ptr = g_dc_spec_user_ptr;
			ALOGV("g_dc_spec_user_ptr = %x, user_ptr=%x \n", (uint32)g_dc_spec_user_ptr, (uint32)g_dc_specific_info.user_ptr);

		}

		if(NULL != dc_exif_info_ptr->spec_ptr->pic_taking_cond_ptr)
		{
			dc_exif_info_ptr->spec_ptr->pic_taking_cond_ptr 		= (EXIF_SPEC_PIC_TAKING_COND_T *)((uint32_t)dc_exif_info_ptr->spec_ptr->pic_taking_cond_ptr + (uint32_t)g_exif_info_start_ptr);
			g_dc_spec_pic_taking_cond_ptr = (EXIF_SPEC_PIC_TAKING_COND_T *)malloc(sizeof(EXIF_SPEC_PIC_TAKING_COND_T));
			memcpy(g_dc_spec_pic_taking_cond_ptr, dc_exif_info_ptr->spec_ptr->pic_taking_cond_ptr , sizeof(EXIF_SPEC_PIC_TAKING_COND_T));
			g_dc_specific_info.pic_taking_cond_ptr = g_dc_spec_pic_taking_cond_ptr;
			ALOGV("g_dc_spec_pic_taking_cond_ptr = %x, pic_taking_cond_ptr=%x \n", (uint32)g_dc_spec_pic_taking_cond_ptr, (uint32)g_dc_specific_info.pic_taking_cond_ptr);
		}


	}

#if 1
	/* Some info is not get from the kernel */
	g_dc_specific_info.basic.PixelXDimension = g_dcam_dimensions.picture_width;
	g_dc_specific_info.basic.PixelYDimension = g_dcam_dimensions.picture_height;
	g_dc_exif_info.primary.basic.ImageWidth = g_dcam_dimensions.picture_width;
	g_dc_exif_info.primary.basic.ImageLength = g_dcam_dimensions.picture_height;
	ALOGV("SPRD OEM:EXIF width=%d, height=%d \n", g_dc_exif_info.primary.basic.ImageWidth, g_dc_exif_info.primary.basic.ImageLength);

	if(NULL != g_dc_exif_info.primary.data_struct_ptr)
	{
		g_dc_exif_info.primary.data_struct_ptr->valid.Orientation = 1;
		g_dc_exif_info.primary.data_struct_ptr->Orientation = g_jpegenc_params.orientation;
	}

	if(NULL != g_dc_exif_info.primary.img_desc_ptr)
	{
		strcpy((char *)g_dc_exif_info.primary.img_desc_ptr->ImageDescription, (char *)g_jpegenc_params.image_description);
		strcpy((char *)g_dc_exif_info.primary.img_desc_ptr->Make, 			 (char *)g_jpegenc_params.make);
		strcpy((char *)g_dc_exif_info.primary.img_desc_ptr->Model, 			 (char *)g_jpegenc_params.model);
		strcpy((char *)g_dc_exif_info.primary.img_desc_ptr->Copyright, 		 (char *)g_jpegenc_params.copyright);
	}

	if (NULL != g_dc_exif_info.gps_ptr)
	{
		if((0 == g_jpegenc_params.Latitude_dd.numerator) && (0 == g_jpegenc_params.Latitude_mm.numerator) && (0 == g_jpegenc_params.Latitude_ss.numerator)
			&& (0 == g_jpegenc_params.Longitude_dd.numerator) && (0 == g_jpegenc_params.Longitude_mm.numerator) && (0 == g_jpegenc_params.Longitude_ss.numerator) )
		{
			/* if no Latitude and Longitude, do not write GPS to EXIF */
			ALOGV("SPRD OEM: GPS: Latitude and Longitude is 0, do not write to EXIF: valid=%d \n", *(uint32*)&g_dc_exif_info.gps_ptr->valid);
		}
		else
		{
			g_dc_exif_info.gps_ptr->valid.GPSLatitudeRef			= 1;
			g_dc_exif_info.gps_ptr->GPSLatitudeRef[0] 			= (0 == g_jpegenc_params.Latitude_ref) ? 'N' : 'S';
			g_dc_exif_info.gps_ptr->valid.GPSLongitudeRef		= 1;
			g_dc_exif_info.gps_ptr->GPSLongitudeRef[0] 			= (0 == g_jpegenc_params.Longitude_ref) ? 'E' : 'W';

			g_dc_exif_info.gps_ptr->valid.GPSLatitude 			= 1;
			g_dc_exif_info.gps_ptr->GPSLatitude[0].numerator 	= g_jpegenc_params.Latitude_dd.numerator;
			g_dc_exif_info.gps_ptr->GPSLatitude[0].denominator	= g_jpegenc_params.Latitude_dd.denominator;
			g_dc_exif_info.gps_ptr->GPSLatitude[1].numerator 	= g_jpegenc_params.Latitude_mm.numerator;
			g_dc_exif_info.gps_ptr->GPSLatitude[1].denominator	= g_jpegenc_params.Latitude_mm.denominator;
			g_dc_exif_info.gps_ptr->GPSLatitude[2].numerator 	= g_jpegenc_params.Latitude_ss.numerator;
			g_dc_exif_info.gps_ptr->GPSLatitude[2].denominator	= g_jpegenc_params.Latitude_ss.denominator;

			g_dc_exif_info.gps_ptr->valid.GPSLongitude 			= 1;
			g_dc_exif_info.gps_ptr->GPSLongitude[0].numerator 	= g_jpegenc_params.Longitude_dd.numerator;
			g_dc_exif_info.gps_ptr->GPSLongitude[0].denominator	= g_jpegenc_params.Longitude_dd.denominator;
			g_dc_exif_info.gps_ptr->GPSLongitude[1].numerator 	= g_jpegenc_params.Longitude_mm.numerator;
			g_dc_exif_info.gps_ptr->GPSLongitude[1].denominator	= g_jpegenc_params.Longitude_mm.denominator;
			g_dc_exif_info.gps_ptr->GPSLongitude[2].numerator 	= g_jpegenc_params.Longitude_ss.numerator;
			g_dc_exif_info.gps_ptr->GPSLongitude[2].denominator	= g_jpegenc_params.Longitude_ss.denominator;

			g_dc_exif_info.gps_ptr->valid.GPSAltitude 			= 1;
			g_dc_exif_info.gps_ptr->GPSAltitude.numerator		= g_position.altitude;
			ALOGE("wxz###: g_dc_exif_info.gps_ptr->GPSAltitude.numerator: %d.", g_dc_exif_info.gps_ptr->GPSAltitude.numerator);
			g_dc_exif_info.gps_ptr->GPSAltitude.denominator		= 1;
			g_dc_exif_info.gps_ptr->valid.GPSAltitudeRef  = 1;


			if(NULL != g_jpegenc_params.gps_process_method)
			{
			    const char ascii[] = {0x41, 0x53, 0x43, 0x49, 0x49, 0, 0, 0};
				g_dc_exif_info.gps_ptr->valid.GPSProcessingMethod			= 1;
				g_dc_exif_info.gps_ptr->GPSProcessingMethod.count			= strlen(g_jpegenc_params.gps_process_method)+sizeof(ascii) + 1;
				memcpy((char *)g_dc_exif_info.gps_ptr->GPSProcessingMethod.ptr, ascii, sizeof(ascii));
				strcpy((char *)g_dc_exif_info.gps_ptr->GPSProcessingMethod.ptr+sizeof(ascii), (char *)g_jpegenc_params.gps_process_method);
			    //add "ASCII\0\0\0" for cts test by lyh
            }

			g_dc_exif_info.gps_ptr->valid.GPSTimeStamp 			= 1;
			g_dc_exif_info.gps_ptr->GPSTimeStamp[0].numerator	= g_jpegenc_params.gps_hour;
			g_dc_exif_info.gps_ptr->GPSTimeStamp[1].numerator	= g_jpegenc_params.gps_minuter;
			g_dc_exif_info.gps_ptr->GPSTimeStamp[2].numerator	= g_jpegenc_params.gps_second;

			g_dc_exif_info.gps_ptr->GPSTimeStamp[0].denominator	= 1;
			g_dc_exif_info.gps_ptr->GPSTimeStamp[1].denominator	= 1;
			g_dc_exif_info.gps_ptr->GPSTimeStamp[2].denominator	= 1;

			g_dc_exif_info.gps_ptr->valid.GPSDateStamp 			= 1;
			strcpy((char *)g_dc_exif_info.gps_ptr->GPSDateStamp,  (char *)g_jpegenc_params.gps_date);

			ALOGV("SPRD OEM: GPS: valid=%d \n", *(uint32*)&g_dc_exif_info.gps_ptr->valid);
		}

	}

	if(NULL != g_dc_specific_info.pic_taking_cond_ptr)
	{
		g_dc_specific_info.pic_taking_cond_ptr->valid.FocalLength 			= 1;
		g_dc_specific_info.pic_taking_cond_ptr->FocalLength.numerator		= g_jpegenc_params.focal_length.numerator;
		g_dc_specific_info.pic_taking_cond_ptr->FocalLength.denominator		= g_jpegenc_params.focal_length.denominator;

	}


	/* TODO: data time is get from user space now */
	if(NULL != g_dc_exif_info.primary.img_desc_ptr)
	{
		strcpy((char *)g_dc_exif_info.primary.img_desc_ptr->DateTime, (char *)g_jpegenc_params.datetime);

	}

	if( NULL != g_dc_exif_info.spec_ptr)
	{
		if(NULL != g_dc_exif_info.spec_ptr->other_ptr)
		{
			memset(g_dc_exif_info.spec_ptr->other_ptr->ImageUniqueID, 0, sizeof(g_dc_exif_info.spec_ptr->other_ptr->ImageUniqueID));
			sprintf((char *)g_dc_exif_info.spec_ptr->other_ptr->ImageUniqueID,
			            "IMAGE %s", g_jpegenc_params.datetime);

		}

		if(NULL != g_dc_exif_info.spec_ptr->date_time_ptr)
		{
			strcpy((char *)g_dc_exif_info.spec_ptr->date_time_ptr->DateTimeOriginal,  (char *)g_jpegenc_params.datetime);
			strcpy((char *)g_dc_exif_info.spec_ptr->date_time_ptr->DateTimeDigitized, (char *)g_jpegenc_params.datetime);
		}
	}
#endif

	ALOGV("SPRD OEM: after: (%x), %x, %x, %x %x \n",
					(uint32_t)g_exif_info_start_ptr, (uint32_t)dc_exif_info_ptr->gps_ptr, (uint32_t)dc_exif_info_ptr->primary.data_struct_ptr,
					(uint32_t)dc_exif_info_ptr->spec_ptr, (uint32_t)dc_exif_info_ptr->spec_ptr->other_ptr);
	ALOGV("SPRD OEM: after: (%x), %x, %x, %x %x \n",
					(uint32_t)g_exif_info_start_ptr, (uint32_t)dc_exif_info_ptr->primary.img_desc_ptr, (uint32_t)dc_exif_info_ptr->spec_ptr->date_time_ptr,
					(uint32_t)dc_exif_info_ptr->spec_ptr->user_ptr, (uint32_t)dc_exif_info_ptr->spec_ptr->pic_taking_cond_ptr);

	ALOGV("SPRD OEM: after copy: (%x), %x, %x, %x %x \n",
					(uint32_t)&g_dc_exif_info, (uint32_t)g_dc_exif_info.gps_ptr, (uint32_t)g_dc_exif_info.primary.data_struct_ptr,
					(uint32_t)g_dc_exif_info.spec_ptr, (uint32_t)g_dc_exif_info.spec_ptr->other_ptr);
	ALOGV("SPRD OEM: after copy: (%x), %x, %x, %x %x \n",
					(uint32_t)&g_dc_exif_info, (uint32_t)g_dc_exif_info.primary.img_desc_ptr, (uint32_t)g_dc_exif_info.spec_ptr->date_time_ptr,
					(uint32_t)g_dc_exif_info.spec_ptr->user_ptr, (uint32_t)g_dc_exif_info.spec_ptr->pic_taking_cond_ptr);
}

void camera_exif_info_release(void)
{
	if(NULL != g_dc_primary_img_desc_ptr)
	{
		free(g_dc_primary_img_desc_ptr);
		g_dc_primary_img_desc_ptr = NULL;
		g_dc_exif_info.primary.img_desc_ptr = NULL;
	}

	if(NULL != g_dc_spec_user_ptr)
	{
		free(g_dc_spec_user_ptr);
		g_dc_spec_user_ptr = NULL;
		g_dc_specific_info.user_ptr = NULL;
	}
	if(NULL != g_dc_spec_pic_taking_cond_ptr)
	{
		free(g_dc_spec_pic_taking_cond_ptr);
		g_dc_spec_pic_taking_cond_ptr = NULL;
		g_dc_specific_info.pic_taking_cond_ptr = NULL;
	}
}

void camera_set_exif_info(void)
{
	int ret = -1;
	struct v4l2_querymenu queryparm;
	uint32 dc_exif_info_ptr_phy;
	JINF_EXIF_INFO_T *dc_exif_info_ptr_virt;

	ALOGV("camera_set_exif_info start: \n");

	dc_exif_info_ptr_phy = (g_jpegenc_params.stream_phy_buf[JPEG_ENC_HW_BUF_NUM-1]);
	dc_exif_info_ptr_virt = (JINF_EXIF_INFO_T *)(g_jpegenc_params.stream_virt_buf[JPEG_ENC_HW_BUF_NUM-1]);

	memset(dc_exif_info_ptr_virt, 0, sizeof(JINF_EXIF_INFO_T));

	ALOGV("camera_set_exif_info: value = %x,  ptr = %x, virt=%x \n",
			g_jpegenc_params.stream_phy_buf[1],  (uint32)dc_exif_info_ptr_phy, (uint32)dc_exif_info_ptr_virt);

	g_jpegenc_params.dc_exif_info_ptr 						= NULL;

	ALOGV("camera_set_exif_info: virtual get value = %x, ptr=%x \n",
		dc_exif_info_ptr_virt->primary.basic.ResolutionUnit , dc_exif_info_ptr_virt->primary.basic.XResolution.numerator);

 	queryparm.id 		= (uint32)dc_exif_info_ptr_phy;
	queryparm.index 	= g_jpegenc_params.stream_buf_len;

        ret =  xioctl(fd, VIDIOC_QUERYMENU, &queryparm);
	if(-1 == ret)
	{
   		ALOGE("camera_set_exif_info: VIDIOC_QUERYMENU error 2, ret=%x \n",ret);
	}
	else
	{
		ALOGV("camera_set_exif_info: VIDIOC_QUERYMENU set index = %x, phy_ptr=%x, virt_ptr=%x \n", queryparm.index, (uint32)dc_exif_info_ptr_virt, (uint32)dc_exif_info_ptr_virt);
		ALOGV("camera_set_exif_info: VIDIOC_QUERYMENU get value = %x, ptr=%x \n",
			dc_exif_info_ptr_virt->primary.basic.ResolutionUnit , dc_exif_info_ptr_virt->primary.basic.XResolution.numerator);

		ALOGV("test ptr: %x, %x\n",
			(uint32_t)dc_exif_info_ptr_virt->spec_ptr,
			(uint32_t)dc_exif_info_ptr_virt->gps_ptr
			);
		camera_exif_convert_ptr(dc_exif_info_ptr_virt);


		//g_jpegenc_params.dc_exif_info_ptr 						= dc_exif_info_ptr_virt;
#if 0
		ALOGV("test value: %x, %x, %x, %s \n",
			dc_exif_info_ptr_virt->gps_ptr->GPSLatitude[0].numerator,
			dc_exif_info_ptr_virt->spec_ptr->basic.ColorSpace,
			dc_exif_info_ptr_virt->spec_ptr->basic.ComponentsConfiguration[2],
			dc_exif_info_ptr_virt->primary.img_desc_ptr->Model
			);

		ALOGV("test value get dc info: %x, %x, %x, %s \n",
			g_jpegenc_params.dc_exif_info_ptr->spec_ptr->pic_taking_cond_ptr->FocalLength.numerator,
			g_jpegenc_params.dc_exif_info_ptr->spec_ptr->pic_taking_cond_ptr->ExposureProgram,
			g_jpegenc_params.dc_exif_info_ptr->spec_ptr->basic.ComponentsConfiguration[2],
			g_jpegenc_params.dc_exif_info_ptr->primary.img_desc_ptr->Model
			);
#endif


	}
}

void *camera_encoder_thread(void *client_data)
{
	uint32_t jpeg_enc_buf_phys_addr;
	uint32_t* jpeg_enc_buf_virt_addr;
	uint32_t jpeg_enc_buf_len;
	uint32_t i;
	g_encoder_is_end = 0;
	ALOGV("camera_encoder_thread S. 0x%x 0x%x 0x%x 0x%x",
		  g_buffers[g_releasebuff_index].virt_addr,g_buffers[g_releasebuff_index].phys_addr,
		  s_camera_info.jpg_enc_y_temp_virt_addr,s_camera_info.jpg_enc_y_temp_phy_addr);

	g_client_data = client_data;
	g_jpegenc_params.format = JPEGENC_YUV_420;
	g_jpegenc_params.quality = g_jpeg_quality_level;
	g_jpegenc_params.width = g_dcam_dimensions.picture_width;
	g_jpegenc_params.height = g_dcam_dimensions.picture_height;
	if(SENSOR_IMAGE_FORMAT_JPEG != s_camera_info.sensor_out_format)
	{
		g_jpegenc_params.yuv_virt_buf =  g_buffers[g_releasebuff_index].virt_addr;
		g_jpegenc_params.yuv_phy_buf = g_buffers[g_releasebuff_index].phys_addr;
		g_jpegenc_params.thumb_src_yuv_virt_buf = g_jpegenc_params.yuv_virt_buf ;
		g_jpegenc_params.thumb_src_yuv_phy_buf = g_jpegenc_params.yuv_phy_buf;
		if(0 == s_camera_info.jpeg_buf_setting_flag)
		{
			g_jpegenc_params.yuv_virt_buf = g_capture_virt_addr;
			g_jpegenc_params.yuv_phy_buf = g_capture_phys_addr;
		}
		else
		{
			g_jpegenc_params.yuv_virt_buf = s_camera_info.jpg_enc_y_temp_virt_addr;
			g_jpegenc_params.yuv_phy_buf = s_camera_info.jpg_enc_y_temp_phy_addr;
		}

	}
	else
	{
		g_jpegenc_params.thumb_src_yuv_virt_buf = g_capture_virt_addr;
		g_jpegenc_params.thumb_src_yuv_phy_buf = g_capture_phys_addr;
		if(0 == s_camera_info.jpeg_buf_setting_flag)
		{
			g_jpegenc_params.yuv_virt_buf = g_capture_virt_addr;
			g_jpegenc_params.yuv_phy_buf = g_capture_phys_addr;
		}
		else
		{
			g_jpegenc_params.yuv_virt_buf = s_camera_info.jpg_enc_y_temp_virt_addr;
			g_jpegenc_params.yuv_phy_buf = s_camera_info.jpg_enc_y_temp_phy_addr;
		}
	}

	ALOGV("camera_encoder_thread ,jpg tmp 0x%x 0x%x",g_jpegenc_params.yuv_virt_buf,g_jpegenc_params.yuv_phy_buf);

	g_jpegenc_params.set_slice_height = s_camera_info.jpeg_codec_slice_height;

	jpeg_enc_buf_len = JPEG_ENC_HW_PMEM;
	jpeg_enc_buf_len = camera_get_size_align_page(jpeg_enc_buf_len);
	jpeg_enc_buf_virt_addr = (uint32_t *)g_dcam_obj->get_jpeg_encoder_mem_by_HW(&jpeg_enc_buf_phys_addr);
         if(NULL == jpeg_enc_buf_virt_addr)
         {
		camera_exif_info_release();
		ALOGE("Fail to encode jpeg picture by SC8800G2 HW.");
		g_encoder_param.size = 0;
		g_jpeg_stream_size = 0;
		g_callback(CAMERA_EXIT_CB_FAILED, g_client_data, CAMERA_FUNC_ENCODE_PICTURE, (uint32_t)&g_encoder_param);
        g_encoder_is_end = 1;
		return NULL;
         }
    	for (i = 0; i < JPEG_ENC_HW_BUF_NUM; i++)
	{
		g_jpegenc_params.stream_virt_buf[i] = jpeg_enc_buf_virt_addr + i * jpeg_enc_buf_len / 4;
		g_jpegenc_params.stream_phy_buf[i] = jpeg_enc_buf_phys_addr + i * jpeg_enc_buf_len;
		ALOGV("encoder: jpegenc_params[%d]: virt: %x, phys: %x.",i,(uint32_t)g_jpegenc_params.stream_virt_buf[i],g_jpegenc_params.stream_phy_buf[i]);
        }
	g_jpegenc_params.stream_buf_len = jpeg_enc_buf_len;
	g_jpegenc_params.stream_size = 0;
	s_camera_info.jpg_record_copy_height = 0;

	if(0 != s_camera_info.jpeg_codec_slice_height)
	{
		g_jpegenc_params.read_callback = camera_jpegenc_update_data_callback;
		camera_jpegenc_write_ping_buffer(0,0);
	}
	else
	{
		g_jpegenc_params.read_callback = 0;
	}

	g_jpegenc_params.thumb_width = g_thumbnail_properties.width;
	g_jpegenc_params.thumb_height = g_thumbnail_properties.height;

	ALOGV("g_thumbnail_properties.quality=%d.",g_thumbnail_properties.quality);
	g_jpegenc_params.thumb_quality = camera_conver_jpegquailty(g_thumbnail_properties.quality);
	ALOGV("g_jpegenc_params.thumb_quality=%d.",g_jpegenc_params.thumb_quality);
	g_jpegenc_params.Latitude_dd.numerator = getDataFromDouble(g_position.latitude, 0);
	g_jpegenc_params.Latitude_dd.denominator = 1;
	g_jpegenc_params.Latitude_mm.numerator = getDataFromDouble(g_position.latitude, 1);
	g_jpegenc_params.Latitude_mm.denominator = 1;
	//g_jpegenc_params.Latitude_ss.numerator = getDataFromDouble(g_position.latitude, 2);
	//g_jpegenc_params.Latitude_ss.denominator = 1;
	getSecondsFromDouble(g_position.latitude, &g_jpegenc_params.Latitude_ss.numerator, &g_jpegenc_params.Latitude_ss.denominator);

	 if(g_position.latitude < 0.0){
	 	g_jpegenc_params.Latitude_ref = 1;
	 }
	 else{
	 	g_jpegenc_params.Latitude_ref = 0;
	 }
	g_jpegenc_params.Longitude_dd.numerator = getDataFromDouble(g_position.longitude, 0);
	g_jpegenc_params.Longitude_dd.denominator = 1;
	g_jpegenc_params.Longitude_mm.numerator = getDataFromDouble(g_position.longitude, 1);
	g_jpegenc_params.Longitude_mm.denominator = 1;
	//g_jpegenc_params.Longitude_ss.numerator = getDataFromDouble(g_position.longitude, 2);
	//g_jpegenc_params.Longitude_ss.denominator = 1;
	getSecondsFromDouble(g_position.longitude, &g_jpegenc_params.Longitude_ss.numerator, &g_jpegenc_params.Longitude_ss.denominator);
	 if(g_position.longitude < 0.0){
	 	g_jpegenc_params.Longitude_ref = 1;
	 }
	 else{
	 	g_jpegenc_params.Longitude_ref = 0;
	 }
	g_jpegenc_params.image_description = get_common_camera_capability("image_description");
	g_jpegenc_params.make = get_common_camera_capability("make");
	g_jpegenc_params.model= get_common_camera_capability("model");
	g_jpegenc_params.copyright= get_common_camera_capability("copyright");
	g_jpegenc_params.orientation = getOrientationFromRotationDegrees(s_camera_info.set_encode_rotation);
	 {
	 	time_t timep;
		struct tm *p;
		time(&timep);
		p=gmtime(&timep);
		sprintf(s_camera_info.datetime_buf, "%4d:%2d:%2d %2d:%2d:%2d", (1900+p->tm_year), (1+p->tm_mon),p->tm_mday,
			p->tm_hour, p->tm_min, p->tm_sec);
		s_camera_info.datetime_buf[19] = '\0';
	 	g_jpegenc_params.datetime = s_camera_info.datetime_buf;

		if(0 == g_position.timestamp)
		time(&g_position.timestamp);
		p=gmtime(&g_position.timestamp);
		sprintf(s_camera_info.gps_date_buf, "%4d:%2d:%2d", (1900+p->tm_year), (1+p->tm_mon),p->tm_mday);
		s_camera_info.gps_date_buf[10] = '\0';
		g_jpegenc_params.gps_date = s_camera_info.gps_date_buf;
		g_jpegenc_params.gps_hour = p->tm_hour;
		g_jpegenc_params.gps_minuter = p->tm_min;
		g_jpegenc_params.gps_second = p->tm_sec;
		//ALOGV("gps_data 1 = %s, %d:%d:%d \n", s_camera_info.gps_date_buf, p->tm_hour, p->tm_min, p->tm_sec);
		ALOGV("gps_data 2 = %s, %d:%d:%d \n", g_jpegenc_params.gps_date, g_jpegenc_params.gps_hour, g_jpegenc_params.gps_minuter, g_jpegenc_params.gps_second);
	 }
	g_jpegenc_params.gps_process_method = g_position.process_method;
	g_jpegenc_params.focal_length.numerator = s_camera_info.focal_length;
	g_jpegenc_params.focal_length.denominator = 1000;

	g_jpegenc_params.dc_exif_info_ptr 						= NULL;
	camera_set_exif_info();

	//wxz20120229: clear the value of g_position for the CTS test.
	memset(&g_position, 0, sizeof(camera_position_type));

  	//encode the jpeg picture by HW.
	if(0 != JPEGENC_encode_one_pic(&g_jpegenc_params, camera_encoder_callback))
	{
		camera_exif_info_release();
		ALOGE("Fail to encode jpeg picture by SC8800G2 HW.");
		g_encoder_param.size = 0;
		g_jpeg_stream_size = 0;
		g_callback(CAMERA_EXIT_CB_FAILED, g_client_data, CAMERA_FUNC_ENCODE_PICTURE, (uint32_t)&g_encoder_param);
                  g_encoder_is_end = 1;
		return NULL;
	}

	camera_exif_info_release();

	ALOGV("camera_encoder_thread E.");
        g_encoder_is_end = 1;
	return NULL;
}
camera_ret_code_type camera_encode_picture(
        camera_frame_type *frame,
        camera_handle_type *handle,
        camera_cb_f_type callback,
        void *client_data)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;
	pthread_attr_t attr;
	g_callback = callback;

#if !CAM_OUT_YUV420_UV	//wxz20120316: convrt VU to UV
{
	uint8_t *dst = (uint8_t *)frame->buf_Virt_Addr + frame->dx * frame->dy;
	uint8_t *src = dst;
	for(int i = 0; i < frame->dy / 2; i++){
		for(int j = 0; j < frame->dx / 2; j++){
			uint8_t tmp = *src++;
			*dst++ = *src++;
			*dst++ = tmp;
		}
	}
	dst = NULL;
	src = NULL;
}
#endif
	//create the thread for encoder
	pthread_attr_init (&attr);
         pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	if(0 !=  pthread_create(&g_encoder_thr, &attr, camera_encoder_thread, client_data))	
	{
		ALOGE("Fail to careate thread in encoder mode.");
		return CAMERA_FAILED;
	}
	else
	{
		ALOGV("OK to create thread in encoder mode.");
	}

	return ret_type;
}

static int open_device(void)
{
	ALOGV("Start to open_device.");

    fd = open("/dev/video0", O_RDWR /* required */, 0);
    if (fd<0) {
	ALOGE("Fail to open dcam device.errno : %d", errno);
        fprintf(stderr, "Cannot open '%s': %d, %s\n", dev_name, errno,  strerror(errno));
        exit(EXIT_FAILURE);
        return -1;
    }
    else
      ALOGV("#####DCAM: OK to open device.");
   return 0;
}
#define SENSOR_PARAM_NUM  8
#define SENSOR_PARA	"/data/misc/sensors/sensor.file"
camera_ret_code_type camera_init(int32_t camera_id)
{
	struct v4l2_streamparm streamparm;
         FILE *fp;
         uint32_t i = 0;
         uint32_t len = 0;
         uint8_t sensor_param[SENSOR_PARAM_NUM];

         memset(&sensor_param[0],0,SENSOR_PARAM_NUM);
         fp = fopen(SENSOR_PARA,"rb+");
	if(NULL == fp){
                  fp = fopen(SENSOR_PARA,"wb+");
                  if(NULL == fp){
                        ALOGE("camera_init:file %s open error:%s\n",SENSOR_PARA,strerror(errno));
                        memset(&sensor_param[0],0xFF,SENSOR_PARAM_NUM);
                  }
	}
        if(NULL != fp) {
                len = fread(sensor_param, 1, SENSOR_PARAM_NUM, fp);
                ALOGV("camera_init:read sensor param len is %d.\n",len);
                ALOGV("camera_init:read sensor param  is %d,%d,%d,%d,%d,%d,%d,%d.\n",sensor_param[0],
                    sensor_param[1],sensor_param[2],sensor_param[3],sensor_param[4],sensor_param[5],
                    sensor_param[6],sensor_param[7]);
         }

	g_camera_id = camera_id;
	ALOGV("SPRD OEM:camera_init,id=%d.",g_camera_id);
	if(0 != open_device())
	{
                ALOGE("camera_init: Fail to open device.");
		return CAMERA_FAILED;
	}

	CLEAR (streamparm);
	streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	streamparm.parm.capture.capturemode = 0;
	if(-1 == g_camera_id)
	{
		streamparm.parm.raw_data[199] = 0;
	}
	else
	{
		streamparm.parm.raw_data[199] = 1;
		streamparm.parm.raw_data[198] = g_camera_id;
	}
	if((1 == g_cam_params.orientation) && ((90 == g_rotation) || (270 == g_rotation))){
		streamparm.parm.raw_data[197] = 1;
	}
	else{
		streamparm.parm.raw_data[197] = 0;
	}
         for(i=188;i<(188+SENSOR_PARAM_NUM);i++)
         {
                streamparm.parm.raw_data[i] = sensor_param[i-188];
         }
	if (-1 == xioctl(fd, VIDIOC_S_PARM, &streamparm))
	{
		ALOGE("preview: Fail to VIDIOC_S_PARM.");
		return CAMERA_FAILED;
	}
        if(1 == streamparm.parm.raw_data[196])/*need to save index of sensor*/
        {
            for(i=188;i<(188+SENSOR_PARAM_NUM);i++)
             {
                    sensor_param[i-188] = streamparm.parm.raw_data[i];
             }
             if(NULL != fp)
             {
                    fseek(fp,0,SEEK_SET);
                    fwrite(sensor_param, 1, SENSOR_PARAM_NUM, fp);
                    fclose(fp);
             }
              ALOGV("camera_init:read sensor param  is %d,%d,%d,%d,%d,%d,%d,%d.\n",sensor_param[0],
                    sensor_param[1],sensor_param[2],sensor_param[3],sensor_param[4],sensor_param[5],
                    sensor_param[6],sensor_param[7]);
        }
        else
        {
            if(NULL != fp)
             {
                    fclose(fp);
             }
        }

	return CAMERA_SUCCESS;
}
void camera_af_init(void)
{
	return;
}
camera_ret_code_type camera_preview_qbuf(uint32_t index)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;
	struct v4l2_buffer buf;

	if((0 == g_preview_stop) && (0 == g_stop_preview_flag))
	{
		ALOGE("SPRD OEM:VIDIOC_QBUF in camera_release_frame.buff_index: %d", index);
		CLEAR (buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_USERPTR;
		buf.m.userptr = (unsigned long) g_buffers[index].phys_addr;
		buf.length = (unsigned long) g_buffers[index].length;
		buf.index = index;
		buf.reserved = 0;
		if(0 != g_buffers[index].u_phys_addr)
		{
			buf.reserved = g_buffers[index].u_phys_addr;
		}

	       	if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
		{
			ALOGE("SPRD OEM:Fail to VIDIOC_QBUF in camera_release_frame.buff_index: %d", index);
			return CAMERA_FAILED;
		}
	}
	return ret_type;
}
camera_ret_code_type camera_release_frame(uint32_t index)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;
	ret_type = camera_preview_qbuf(index);
	return ret_type;
}

camera_ret_code_type camera_set_dimensions (
        uint16_t picture_width,
        uint16_t picture_height,
        uint16_t display_width,
#ifdef FEATURE_CAMERA_V7
        uint16_t display_height,
#endif
        camera_cb_f_type callback,
        void *client_data)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;

	//set encoder and display size
	//callback and client_data are NULL.
	g_dcam_dimensions.picture_width = picture_width;
	g_dcam_dimensions.picture_height = picture_height;
	g_dcam_dimensions.display_width = display_width;
	g_dcam_dimensions.display_height = display_height;

	return ret_type;
}

camera_ret_code_type camera_set_encode_properties(
        camera_encode_properties_type *encode_properties)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;

	//set encoder propertie: format, quality.
	//there the size is 0 and format is CAMERA_JPEG.
	g_encode_properties.format = encode_properties->format;
	g_encode_properties.quality = encode_properties->quality;
	g_encode_properties.file_size = encode_properties->file_size;

	return ret_type;
}

static int camera_set_jpegcomp(uint32_t quality){
	ALOGV("camera_set_jpegcomp E.quality = %d.", quality);

	if(quality > 85){
		g_jpeg_quality_level = JPEGENC_QUALITY_HIGH;
	}
	else if(quality > 75){
		g_jpeg_quality_level = JPEGENC_QUALITY_MIDDLE_HIGH;
	}
	else { // if(quality > 70)
		g_jpeg_quality_level = JPEGENC_QUALITY_MIDDLE;
	}

	ALOGV("camera_set_jpegcomp X.");

	return 0;
}

static int camera_set_ctrl(uint32_t key, int32_t value)
{
	struct v4l2_control ctrl;

	ALOGV("camera_set_ctrl E, key: 0x%x, value: %d.", key, value);

	ctrl.id = key;
	ctrl.value = value;

	if (-1 == xioctl(fd, VIDIOC_S_CTRL, &ctrl))
	{
		ALOGE("Fail to VIDIOC_S_CTRL.");
		return -1;
	}

	ALOGV("camera_set_ctrl X.");

	return 0;
}
camera_ret_code_type camera_set_parm(
        camera_parm_type id,
        int32_t          parm,
        camera_cb_f_type callback,
        void            *client_data)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;

	//set camera parameters
	//callback and client_data are NULL.
	if(id >= CAMERA_PARM_MAX)
	{
		ALOGE("Fail to camera_set_parm: the id: %d is invalid.", id);
		return CAMERA_INVALID_PARM;
	}

	switch(id)
	{
		// CAMERA_PARM_STATE:/* read only operation states: camera_state_type */

	    	// CAMERA_PARM_ACTIVE_CMD/* read only active command in execution: camera_func_type */:

		// CAMERA_PARM_ZOOM:/* zoom */
		case CAMERA_PARM_EXPOSURE_COMPENSATION:
			camera_set_ctrl(V4L2_CID_EXPOSURE,parm);
			break;
		case CAMERA_PARM_FOCUS_MODE:
			if(0 != camera_set_ctrl(V4L2_CID_FOCUS_AUTO, parm))
			{
				ret_type = CAMERA_FAILED;
			}
			break;
                  /* This affects only when encoding. It has to be set only in preview mode */
		case CAMERA_PARM_ENCODE_ROTATION: /* 0, 90, 180, 270 degrees */
			s_camera_info.set_encode_rotation = (uint32_t)parm;
			break;
		case CAMERA_PARM_SENSOR_ROTATION: /* 0, 90, 180, 270 degrees */
			g_rotation_parm = (uint32_t)parm;
			break;
		case CAMERA_PARM_FOCAL_LENGTH:
			s_camera_info.focal_length = (uint32_t)parm;
			break;
    /* Sensor can be rotated from forward direction to reversed direction or
     * vise versa. When in normal position, line 1 is on the top. When in
     * reverse position, line 1 is now at the bottom, not on the top, so the image
     * need to be reversed, 0 = normal, 1 = reverse */
		// CAMERA_PARM_SENSOR_POSITION: /* use camera_sp_type */

		case CAMERA_PARM_CONTRAST:    /* contrast */
			camera_set_ctrl(V4L2_CID_CONTRAST, parm);
			break;

    		case CAMERA_PARM_BRIGHTNESS:/* brightness */
			camera_set_ctrl(V4L2_CID_BRIGHTNESS, parm);
			break;
		// CAMERA_PARM_SHARPNESS:/* sharpness */

		// CAMERA_PARM_EXPOSURE:        /* use camera_exposure_type */

		case CAMERA_PARM_WB:              /* use camera_wb_type */
			camera_set_ctrl(V4L2_CID_DO_WHITE_BALANCE, parm);
			break;
		case CAMERA_PARM_EFFECT:          /* use camera_effect_type */
			camera_set_ctrl(V4L2_CID_COLORFX, parm);
			break;
		case CAMERA_PARM_SCENE_MODE:          /* use camera_scene_mode_type */
			camera_set_ctrl(V4L2_CID_COLOR_KILLER, parm);
			break;
		case CAMERA_PARM_ZOOM:
			g_hal_zoom_level = (uint32_t)parm;
			camera_set_ctrl(V4L2_CID_ZOOM_ABSOLUTE, parm);
			break;
		case CAMERA_PARM_JPEGCOMP:
			camera_set_jpegcomp(parm);
			break;
		case CAMERA_PARM_ORIENTATION:
			g_cam_params.orientation_parm = (uint32_t)parm;
			ALOGE("SPRD OEM: set_parm orientation: %d.", g_cam_params.orientation_parm);
			break;
		// CAMERA_PARM_AUDIO_FMT:      /* use video_fmt_stream_audio_type */

		// CAMERA_PARM_FPS:            /* frames per second, unsigned integer number */

		case CAMERA_PARM_FLASH:         /* Flash control, see camera_flash_type */
			g_cam_params.flash_en = (uint32_t)parm;
			camera_set_ctrl(V4L2_CID_GAMMA, g_cam_params.flash_en);
			ALOGE("SPRD OEM: set_parm flash_en: %d \n", g_cam_params.flash_en);
			break;

		// CAMERA_PARM_RED_EYE_REDUCTION: /* boolean */

		case CAMERA_PARM_NIGHTSHOT_MODE:  /* Night shot mode, snapshot at reduced FPS */
			//TODO
			break;
		// CAMERA_PARM_REFLECT:        /* Use camera_reflect_type */

		case CAMERA_PARM_PREVIEW_MODE:   /* Use camera_preview_mode_type */
			g_cam_params.preview_mode = parm;
			break;
		case CAMERA_PARM_ANTIBANDING:   /* Use camera_anti_banding_type */
			camera_set_ctrl(V4L2_CID_POWER_LINE_FREQUENCY,parm);
			//TODO
			break;
		// CAMERA_PARM_FOCUS_STEP
		case CAMERA_PARM_FOCUS_RECT:/* Suresh Gara & Saikumar*/
			memcpy(s_focus_zone_param,(uint32_t*)parm,FOCUS_RECT_PARAM_LEN);
			break;

		case CAMERA_PARM_AF_MODE:
			g_cam_params.focus_mode= (uint32_t)parm;
			ALOGE("SPRD OEM: set_parm focus_mode: %d \n", g_cam_params.focus_mode);
			break;

#ifdef FEATURE_CAMERA_V7
    /* Name change to CAMERA_PARM_EXPOSURE_METERING, remove this later */
		// CAMERA_PARM_AUTO_EXPOSURE_MODE: /* Use camera_auto_exposure_mode_type */
#endif /* FEATURE_CAMERA_V7 */
#ifdef FEATURE_CAMERA_INCALL
		// CAMERA_PARM_INCALL         /* In call and vocoder type */
#endif /* FEATURE_CAMERA_INCALL */
#ifdef FEATURE_VIDEO_ENCODE_FADING
		// CAMERA_PARM_FADING
#endif /* FEATURE_VIDEO_ENCODE_FADING */
		case CAMERA_PARM_ISO:
			//TODO
			break;
#ifdef FEATURE_CAMERA_V7
    /* Use to control the exposure compensation */
		// CAMERA_PARM_EXPOSURE_COMPENSATION
		// CAMERA_PARM_PREVIEW_FPS
		// CAMERA_PARM_EXPOSURE_METERING
		// CAMERA_PARM_APERTURE
		// CAMERA_PARM_SHUTTER_SPEED
		//CAMERA_PARM_FLASH_STATE
#endif /* FEATURE_CAMERA_V7 */
   // CAMERA_PARM_HUE,
    //CAMERA_PARM_SATURATION,
		case CAMERA_PARM_LUMA_ADAPTATION:
			//TODO
			break;
#ifdef FEATURE_VIDENC_TRANSITION_EFFECTS
    //CAMERA_PARM_TRANSITION,
    //CAMERA_PARM_TRANSITION_ALPHA,
    //CAMERA_PARM_TRANSITION_CURTAIN,
    //CAMERA_PARM_TRANSITION_OFF,
#endif /* FEATURE_VIDENC_TRANSITION_EFFECTS */
#ifdef FEATURE_CAMERA_V770
    //CAMERA_PARM_FRAME_TIMESTAMP,
    //CAMERA_PARM_STROBE_FLASH,
#endif //FEATURE_CAMERA_V770
    //CAMERA_PARM_HISTOGRAM,
#ifdef FEATURE_CAMERA_BESTSHOT_MODE
    //CAMERA_PARM_BESTSHOT_MODE,
#endif /* FEATURE_CAMERA_BESTSHOT_MODE */
#ifdef FEATURE_VIDEO_ENCODE
    //CAMERA_PARM_SPACE_LIMIT,
#ifdef FEATURE_CAMCORDER_DIS
    //CAMERA_PARM_DIS,
#endif /* FEATURE_CAMCORDER_DIS */
#endif
#ifdef FEATURE_CAMERA_V7
    //CAMERA_PARM_FPS_LIST,
#endif
		default:
			break;
	}

	return ret_type;
}
camera_ret_code_type camera_set_position(
        camera_position_type *position,
        camera_cb_f_type      callback,
        void                 *client_data)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;

	g_position.timestamp = position->timestamp;
	g_position.altitude = position->altitude;
	g_position.latitude = position->latitude;
	g_position.longitude = position->longitude;
	g_position.process_method = position->process_method;
	//for test
	/*g_position.timestamp = 1199145600;
	g_position.altitude = 21;
	g_position.latitude = 37.736071;
	g_position.longitude = -122.441983;
	g_position.process_method = "GPS NETWORK HYBRID ARE ALL FINE.";
	*/

    	ALOGV("timestamp %ld, latitude : %f, longitude : %f,altitude: %d. ", position->timestamp, position->latitude, position->longitude,position->altitude);
	return ret_type;
}

camera_ret_code_type camera_set_thumbnail_properties (
                              uint32_t width,
                              uint32_t height,
                              uint32_t quality)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;

	g_thumbnail_properties.width = width;
	g_thumbnail_properties.height = height;
	g_thumbnail_properties.quality = quality;

	return ret_type;
}

static void init_device(void)
{
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;
	unsigned int min;
	ALOGV("#####DCAM: VIDIOC_QUERYCAP start.\n");
	memset(&s_camera_info.sensor_mode_info[0],0,10*sizeof(SENSOR_MODE_INFO_T));
	s_camera_info.cap_mem_size = 0;
	s_camera_info.dcam_out_width = 0;
	s_camera_info.dcam_out_height = 0;
	s_camera_info.is_need_rotation = 0;
	s_camera_info.rot_angle = 0;
	s_camera_info.is_interpolation = 0;
	s_camera_info.out_format = DCAM_DATA_MAX;

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
        if (EINVAL == errno) {
            fprintf(stderr, "%s is no V4L2 device\n", dev_name);
            exit(EXIT_FAILURE);
        } else {
            errno_exit("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        fprintf(stderr, "%s is no video capture device\n", dev_name);
        exit(EXIT_FAILURE);
    }
     if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
            fprintf(stderr, "%s does not support streaming i/o\n", dev_name);
            exit(EXIT_FAILURE);
      }
    //////not all capture support crop!!!!!!!
    /* Select video input, video standard and tune here. */
    ALOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");
    CLEAR (cropcap);
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
#ifndef CROP_BY_JACK
        crop.c = cropcap.defrect; /* reset to default */
#else
        crop.c.left = cropcap.defrect.left;
        crop.c.top = cropcap.defrect.top;
        crop.c.width = 640;
        crop.c.height = 480;
#endif
        ALOGV("----->has ability to crop!!\n");
        ALOGV("cropcap.defrect = (%d, %d, %d, %d)\n", cropcap.defrect.left,
                cropcap.defrect.top, cropcap.defrect.width,
                cropcap.defrect.height);
        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
            switch (errno) {
            case EINVAL:
                /* Cropping not supported. */
                break;
            default:
                /* Errors ignored. */
                break;
            }
            ALOGV("-----!!but crop to (%d, %d, %d, %d) Failed!!\n",
                    crop.c.left, crop.c.top, crop.c.width, crop.c.height);
        } else {
            ALOGV("----->sussess crop to (%d, %d, %d, %d)\n", crop.c.left,
                    crop.c.top, crop.c.width, crop.c.height);
        }
    } else {
        /* Errors ignored. */
        ALOGV("!! has no ability to crop!!\n");
    }
    ALOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");
    ALOGV("\n");
    ////////////crop finished!
}

camera_ret_code_type camera_start (
        camera_cb_f_type callback,
        void *client_data
#ifdef FEATURE_NATIVELINUX
        ,int  display_height,
        int  display_width
#endif // FEATURE_NATIVELINUX
        )
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;

	init_device();
	ALOGV("OK to init_device.");
	//change the status from INIT to IDLE.
	callback(CAMERA_STATUS_CB, client_data, CAMERA_FUNC_START, 0);
	ALOGV("OK to change the status from INIT to IDLE.");

	g_dcam_dimensions.display_width = display_width;
	g_dcam_dimensions.display_height = display_height;

	return ret_type;
}
//static int convert_format_by_DMA(uint32_t width, uint32_t height, uint32_t address)
static int convert_format_by_DMA(uint32_t width, uint32_t height,  uint32_t address, uint32_t in_height)
{
	SCALE_YUV422_YUV420_T yuv_config;
	int fd = -1;
         int ret = 0;

	fd = open("/dev/sprd_scale", O_RDONLY);
	if (-1 == fd)
	{
		ALOGE("Fail to open scale device for DMA.");
        	return -1;
   	 }
    	else
      		ALOGV("OK to open scale device for DMA.");

	yuv_config.width = width;
	yuv_config.height = height;
	yuv_config.src_addr = address + width * in_height;
	yuv_config.src_addr += width;
	yuv_config.dst_addr = address + width * height;

	if (-1 == xioctl(fd, SCALE_IOC_YUV422_YUV420, &yuv_config))
	{
		ALOGE("Fail to SCALE_IOC_YUV422_YUV420.");
		ret = -1;
	}

	if(-1 == close(fd))
	{
		ALOGE("Fail to close scale device for DMA.");
        	return -1;
   	 }
    	fd = -1;
	return ret;	
}

uint32_t camera_get_swap_buf_size(uint32_t tar_width)
{
    uint32_t      buf_size = 0;
    uint32_t      line_size = 0;

    if(!tar_width)
        return 0;

    line_size = (tar_width + SCALE_OUT_WIDTH_MAX - 1) / SCALE_OUT_WIDTH_MAX * SCALE_OUT_WIDTH_MAX;
    line_size = line_size << 3;

    buf_size = tar_width * SCALE_SLICE_HEIGHT * 2;
    buf_size += line_size + 3 * getpagesize();

    return buf_size;
}
void camera_alloc_swap_buffer(uint32_t phy_addr, uint32_t buf_size)
{
	g_slice_swap_buf = phy_addr;
         g_slice_swap_buf_size = buf_size;
	ALOGV("INTERPOLATION:g_slice_swap_buf=0x%x. size 0x%x",g_slice_swap_buf,g_slice_swap_buf_size);
}
static int camera_cap_zoom_colorformat(SCALE_DATA_FORMAT_E output_fmt, uint32_t output_width, uint32_t output_height, uint32_t output_addr, ZOOM_TRIM_RECT_T *trim_rect, uint32_t input_addr, SCALE_DATA_FORMAT_E input_fmt)
{
	static int fd = -1;
	SCALE_CONFIG_T scale_config;
	SCALE_SIZE_T scale_size;
	SCALE_RECT_T scale_rect;
	SCALE_ADDRESS_T scale_address;
	SCALE_DATA_FORMAT_E data_format;
	//uint32_t sub_sample_en;
	SCALE_MODE_E scale_mode;
	uint32_t enable = 0, mode;
	uint32_t slice_height = 0;
	ISP_ENDIAN_T in_endian;
	ISP_ENDIAN_T out_endian;
         int ret = 0;

	fd = open("/dev/sprd_scale", O_RDONLY);//O_RDWR /* required */, 0);
	if (-1 == fd)
	{
		ALOGE("Fail to open scale device.");
        	return -1;
   	 }

	//set mode
	scale_config.id = SCALE_PATH_MODE;
	scale_mode = SCALE_MODE_SCALE;
	scale_config.param = &scale_mode;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAP_ZOOM_COLORFORM_END;
	}
	//set input data format
	scale_config.id = SCALE_PATH_INPUT_FORMAT;
	data_format = input_fmt;
	scale_config.param = &data_format;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret =  -1;
                  goto CAP_ZOOM_COLORFORM_END;
	}
	//set output data format
	scale_config.id = SCALE_PATH_OUTPUT_FORMAT;
	data_format = output_fmt;
	scale_config.param = &data_format;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret =  -1;
                  goto CAP_ZOOM_COLORFORM_END;
	}
	//set input size
	scale_config.id = SCALE_PATH_INPUT_SIZE;
	scale_size.w = trim_rect->w;
	scale_size.h = trim_rect->h;
	scale_config.param = &scale_size;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAP_ZOOM_COLORFORM_END;
	}
	//set output size
	scale_config.id = SCALE_PATH_OUTPUT_SIZE;
	scale_size.w = output_width;
	scale_size.h = output_height;
	scale_config.param = &scale_size;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAP_ZOOM_COLORFORM_END;
	}	
	//set input size
	scale_config.id = SCALE_PATH_INPUT_RECT;
	scale_rect.x = 0;//trim_rect->x;
	scale_rect.y = 0;//trim_rect->y;
	scale_rect.w = trim_rect->w;
	scale_rect.h = trim_rect->h;
	scale_config.param = &scale_rect;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret =  -1;
                  goto CAP_ZOOM_COLORFORM_END;
	}
	//set input address
	scale_config.id = SCALE_PATH_INPUT_ADDR;
	scale_address.yaddr = input_addr;
	scale_address.uaddr = input_addr + trim_rect->w * trim_rect->h;
	scale_address.vaddr = scale_address.uaddr;
	scale_config.param = &scale_address;
	ALOGV("INTERPOLATION:scale input y addr:0x%x,uv addr:0x%x.",scale_address.yaddr,scale_address.uaddr);
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAP_ZOOM_COLORFORM_END;
	}
	//set output address
	scale_config.id = SCALE_PATH_OUTPUT_ADDR;
	scale_address.yaddr = output_addr;
	scale_address.uaddr = output_addr + output_width * output_height;
	scale_address.vaddr = scale_address.uaddr;
	scale_config.param = &scale_address;
	ALOGV("INTERPOLATION:scale out  y addr:0x%x,uv addr:0x%x.",scale_address.yaddr,scale_address.uaddr);;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAP_ZOOM_COLORFORM_END;
	}
	if(output_width > 640){
		//set slice mode
		scale_config.id = SCALE_PATH_SLICE_SCALE_EN;
		enable = 1;
		scale_config.param = &enable;
		if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
		{
			ALOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
			ret =  -1;
                           goto CAP_ZOOM_COLORFORM_END;
		}
		//set slice height
		scale_config.id = SCALE_PATH_SLICE_SCALE_HEIGHT;
		slice_height = SCALE_SLICE_HEIGHT;
		scale_config.param = &slice_height;
		if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
		{
			ALOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
			ret =  -1;
                           goto CAP_ZOOM_COLORFORM_END;
		}
		//set swap and line buffer address
		scale_config.id = SCALE_PATH_SWAP_BUFF;
		scale_address.yaddr = g_slice_swap_buf; // 384k bytes  //output_w * slice_h  the buff size is for 5M.
		scale_address.uaddr = scale_address.yaddr + (output_width * SCALE_SLICE_HEIGHT); // 384k bytes
		scale_address.vaddr = scale_address.uaddr + (output_width * SCALE_SLICE_HEIGHT);  // 256k bytes //output_w * 4
		scale_config.param = &scale_address;
		ALOGV("INTERPOLATION:set scale config g_slice_swap_buf=0x%x,scale_address.uaddr=0x%x,scale_address.vaddr=0x%x.",
			    g_slice_swap_buf,scale_address.uaddr,scale_address.vaddr);
		if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
		{
			ALOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
			ret = -1;
                            goto CAP_ZOOM_COLORFORM_END;
		}		
	}
	//set input endian
	scale_config.id = SCALE_PATH_INPUT_ENDIAN;
	in_endian.endian_y = 1;
	in_endian.endian_uv = 1;
	scale_config.param = &in_endian;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAP_ZOOM_COLORFORM_END;
	}
	//set output endian
	scale_config.id = SCALE_PATH_OUTPUT_ENDIAN;
	out_endian.endian_y = 1;
	out_endian.endian_uv = 1;
	scale_config.param = &out_endian;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAP_ZOOM_COLORFORM_END;
	}

	//done
	if (-1 == xioctl(fd, SCALE_IOC_DONE, 0))
	{
		ALOGE("Fail to SCALE_IOC_DONE");
		ret = -1;                  
	}
CAP_ZOOM_COLORFORM_END:
	if(-1 == close(fd))   
	{   
		ALOGE("Fail to close scale device.");
         	return -1;   
   	 } 
    	fd = -1;
	return ret;
}

static int camera_copy(SCALE_DATA_FORMAT_E output_fmt, uint32_t output_width, uint32_t output_height, uint32_t output_y_addr,
	                                                        uint32_t output_uv_addr,ZOOM_TRIM_RECT_T *trim_rect, uint32_t input_yaddr, uint32_t input_uvaddr,
	                                                        SCALE_DATA_FORMAT_E input_fmt)
{
	static int fd = -1;
	SCALE_CONFIG_T scale_config;
	SCALE_SIZE_T scale_size;
	SCALE_RECT_T scale_rect;
	SCALE_ADDRESS_T scale_address;
	SCALE_DATA_FORMAT_E data_format;
	SCALE_MODE_E scale_mode;
	uint32_t enable = 0, mode;
	uint32_t slice_height = 0;
	ISP_ENDIAN_T in_endian;
	ISP_ENDIAN_T out_endian;
         int ret = 0;

	//ALOGE("[SPRD OEM]:camera_copy %d %d %d %d \n",trim_rect->w ,output_width,trim_rect->h,output_height);

	if((trim_rect->w != output_width) || (trim_rect->h != output_height))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert ,input size isn't same with output size ! \n");
		return -1;
	}

	fd = open("/dev/sprd_scale", O_RDONLY);
	if (-1 == fd)
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to open scale device.");
        		return -1;
   	 }

	//set mode
	scale_config.id = SCALE_PATH_MODE;
	scale_mode = SCALE_MODE_SCALE;
	scale_config.param = &scale_mode;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_COPY_END;
	}
	//set input data format
	scale_config.id = SCALE_PATH_INPUT_FORMAT;
	data_format = input_fmt;
	scale_config.param = &data_format;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_COPY_END;
	}
	//set output data format
	scale_config.id = SCALE_PATH_OUTPUT_FORMAT;
	data_format = output_fmt;
	scale_config.param = &data_format;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_COPY_END;
	}
	//set input size
	scale_config.id = SCALE_PATH_INPUT_SIZE;
	scale_size.w = trim_rect->w;
	scale_size.h = trim_rect->h;
	scale_config.param = &scale_size;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_COPY_END;
	}
	//set output size
	scale_config.id = SCALE_PATH_OUTPUT_SIZE;
	scale_size.w = output_width;
	scale_size.h = output_height;
	scale_config.param = &scale_size;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_COPY_END;
	}	
	//set input size
	scale_config.id = SCALE_PATH_INPUT_RECT;
	scale_rect.x = trim_rect->x;
	scale_rect.y = trim_rect->y;
	scale_rect.w = trim_rect->w;
	scale_rect.h = trim_rect->h;
	scale_config.param = &scale_rect;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_COPY_END;
	}
	//set input address
	scale_config.id = SCALE_PATH_INPUT_ADDR;
	scale_address.yaddr = input_yaddr;
	if(0 != input_uvaddr)
	{
		scale_address.uaddr = input_uvaddr;
	}
	else
	{
		scale_address.uaddr = input_yaddr + trim_rect->w*trim_rect->h;
	}
	scale_address.vaddr = scale_address.uaddr;
	scale_config.param = &scale_address;
	//ALOGV("[SPRD OEM]:camera_copy,scale input y addr:0x%x,uv addr:0x%x.",scale_address.yaddr,scale_address.uaddr);
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret =  -1;
                  goto CAMERA_COPY_END;
	}
	//set output address
	scale_config.id = SCALE_PATH_OUTPUT_ADDR;
	scale_address.yaddr = output_y_addr;
	scale_address.uaddr = output_uv_addr;//output_addr + output_width * output_height;
	scale_address.vaddr = scale_address.uaddr;
	scale_config.param = &scale_address;
	//ALOGV("[SPRD OEM]:camera_format_convert,scale out  y addr:0x%x,uv addr:0x%x.",scale_address.yaddr,scale_address.uaddr);;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_COPY_END;
	}

	//set input endian
	scale_config.id = SCALE_PATH_INPUT_ENDIAN;
	in_endian.endian_y = 1;
	in_endian.endian_uv = 1;
	scale_config.param = &in_endian;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret =  -1;
                  goto CAMERA_COPY_END;
	}
	//set output endian
	scale_config.id = SCALE_PATH_OUTPUT_ENDIAN;
	out_endian.endian_y = 1;
	out_endian.endian_uv = 1;
	scale_config.param = &out_endian;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_COPY_END;
	}

	//done
	if (-1 == xioctl(fd, SCALE_IOC_DONE, 0))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_DONE");
		ret = -1;
	}	
CAMERA_COPY_END:	
	if(-1 == close(fd))   
	{   
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to close scale device.");
        		return -1;   
   	 } 
    	fd = -1;
	return ret;
}

static int camera_format_convert(SCALE_DATA_FORMAT_E output_fmt, uint32_t output_width, uint32_t output_height, uint32_t output_addr, ZOOM_TRIM_RECT_T *trim_rect, uint32_t input_yaddr, uint32_t input_uvaddr, SCALE_DATA_FORMAT_E input_fmt)
{
	static int fd = -1;
	SCALE_CONFIG_T scale_config;
	SCALE_SIZE_T scale_size;
	SCALE_RECT_T scale_rect;
	SCALE_ADDRESS_T scale_address;
	SCALE_DATA_FORMAT_E data_format;
	SCALE_MODE_E scale_mode;
	uint32_t enable = 0, mode;
	uint32_t slice_height = 0;
	ISP_ENDIAN_T in_endian;
	ISP_ENDIAN_T out_endian;
         int ret = 0;

	if((trim_rect->w != output_width) || (trim_rect->h != output_height))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert ,input size isn't same with output size ! \n");
		return -1;
	}

	fd = open("/dev/sprd_scale", O_RDONLY);
	if (-1 == fd)
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to open scale device.");
        		return -1;
   	 }

	//set mode
	scale_config.id = SCALE_PATH_MODE;
	scale_mode = SCALE_MODE_SCALE;
	scale_config.param = &scale_mode;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_FORMAT_CONVERT_END;
	}
	//set input data format
	scale_config.id = SCALE_PATH_INPUT_FORMAT;
	data_format = input_fmt;
	scale_config.param = &data_format;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_FORMAT_CONVERT_END;
	}
	//set output data format
	scale_config.id = SCALE_PATH_OUTPUT_FORMAT;
	data_format = output_fmt;
	scale_config.param = &data_format;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_FORMAT_CONVERT_END;
	}
	//set input size
	scale_config.id = SCALE_PATH_INPUT_SIZE;
	scale_size.w = trim_rect->w;
	scale_size.h = trim_rect->h;
	scale_config.param = &scale_size;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_FORMAT_CONVERT_END;
	}
	//set output size
	scale_config.id = SCALE_PATH_OUTPUT_SIZE;
	scale_size.w = output_width;
	scale_size.h = output_height;
	scale_config.param = &scale_size;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_FORMAT_CONVERT_END;
	}	
	//set input size
	scale_config.id = SCALE_PATH_INPUT_RECT;
	scale_rect.x = trim_rect->x;
	scale_rect.y = trim_rect->y;
	scale_rect.w = trim_rect->w;
	scale_rect.h = trim_rect->h;
	scale_config.param = &scale_rect;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_FORMAT_CONVERT_END;
	}
	//set input address
	scale_config.id = SCALE_PATH_INPUT_ADDR;
	scale_address.yaddr = input_yaddr;
	if(0 != input_uvaddr)
	{
		scale_address.uaddr = input_uvaddr;
	}
	else
	{
		scale_address.uaddr = input_yaddr + trim_rect->w*trim_rect->h;
	}
	scale_address.vaddr = scale_address.uaddr;
	scale_config.param = &scale_address;
	ALOGV("[SPRD OEM]:camera_format_convert,scale input y addr:0x%x,uv addr:0x%x.",scale_address.yaddr,scale_address.uaddr);
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_FORMAT_CONVERT_END;
	}
	//set output address
	scale_config.id = SCALE_PATH_OUTPUT_ADDR;
	scale_address.yaddr = output_addr;
	scale_address.uaddr = output_addr + output_width * output_height;
	scale_address.vaddr = scale_address.uaddr;
	scale_config.param = &scale_address;
	ALOGV("[SPRD OEM]:camera_format_convert,scale out  y addr:0x%x,uv addr:0x%x.",scale_address.yaddr,scale_address.uaddr);;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_FORMAT_CONVERT_END;
	}

	//set input endian
	scale_config.id = SCALE_PATH_INPUT_ENDIAN;
	in_endian.endian_y = 1;
	in_endian.endian_uv = 1;
	scale_config.param = &in_endian;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_FORMAT_CONVERT_END;
	}
	//set output endian
	scale_config.id = SCALE_PATH_OUTPUT_ENDIAN;
	out_endian.endian_y = 1;
	out_endian.endian_uv = 1;
	scale_config.param = &out_endian;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret =  -1;
                  goto CAMERA_FORMAT_CONVERT_END;
	}

	//done
	if (-1 == xioctl(fd, SCALE_IOC_DONE, 0))
	{
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to SCALE_IOC_DONE");
		ret = -1;
	}
CAMERA_FORMAT_CONVERT_END:
	if(-1 == close(fd))   
	{   
		ALOGE("[SPRD OEM ERR]:camera_format_convert,Fail to close scale device.");
        		return -1;   
   	 } 
    	fd = -1;
	return ret;
}

static int camera_yuv_rearrange(uint32_t yuv_422,
                                                                                                 uint32_t y_vir,
                                                                                                 uint32_t uv_vir,
                                                                                                 uint32_t  width,
                                                                                                 uint32_t  height,
                                                                                                 struct zoom_trim_rect *p_trim_rect,
                                                                                                 uint32_t *p_offset_y,
                                                                                                 uint32_t *p_offset_uv)
{
    uint32_t     i = 0;
    uint32_t     src_addr = 0, dst_addr = 0;
    uint32_t        offset = (uint32_t)((p_trim_rect->h + p_trim_rect->y)*width);
    uint32_t        total_cycles =  p_trim_rect->h;

    src_addr = y_vir + offset + p_trim_rect->x;
    dst_addr = y_vir + (uint32_t)(width*height) -p_trim_rect->w;
    for(i = 0; i < total_cycles;  i++)
    {
        memcpy((void*)dst_addr, (void*)src_addr,p_trim_rect->w);
        dst_addr -= p_trim_rect->w;
        src_addr -= width;
    }
    *p_offset_y = dst_addr + p_trim_rect->w - y_vir;

    if(yuv_422)
    {
        src_addr = uv_vir+ offset + p_trim_rect->x;
        dst_addr = uv_vir+ (uint32_t)(width*height) -p_trim_rect->w;
    }
    else
    {
        src_addr = uv_vir + (offset>>1) + p_trim_rect->x;
        dst_addr = uv_vir+ (uint32_t)(width*height/2) -p_trim_rect->w;
        total_cycles = total_cycles >> 1;
    }

    for(i = 0; i < total_cycles;  i++)
    {
        memcpy((void*)dst_addr, (void*)src_addr,p_trim_rect->w);
        dst_addr -= p_trim_rect->w;
        src_addr -= width;
    }
    *p_offset_uv = dst_addr + p_trim_rect->w - uv_vir;

    return 0;
}

static int camera_scale_functions(SCALE_DATA_FORMAT_E output_fmt, uint32_t output_width, uint32_t output_height,
	                                                      uint32_t output_yaddr,uint32_t output_uvaddr,
	                                                      ISP_ENDIAN_T output_endian,
	                                                      ZOOM_TRIM_RECT_T *trim_rect,
	                                                      uint32_t input_width,uint32_t input_height,uint32_t input_yaddr,
	                                                      uint32_t intput_uvaddr, SCALE_DATA_FORMAT_E input_fmt,
	                                                      ISP_ENDIAN_T input_endian)
{
	static int fd = -1;
	SCALE_CONFIG_T scale_config;
	SCALE_SIZE_T scale_size;
	SCALE_RECT_T scale_rect;
	SCALE_ADDRESS_T scale_address;
	SCALE_DATA_FORMAT_E data_format;
	SCALE_MODE_E scale_mode;
	uint32_t enable = 0, mode;
	uint32_t slice_height = 0;
	ISP_ENDIAN_T in_endian;
	ISP_ENDIAN_T out_endian;
         int ret = 0;

	fd = open("/dev/sprd_scale", O_RDONLY);
	if (-1 == fd)
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to open scale device.\n");
        	return -1;
   	 }

	//set mode
	scale_config.id = SCALE_PATH_MODE;
	scale_mode = SCALE_MODE_SCALE;
	scale_config.param = &scale_mode;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_SCALE_FUNCTIONS_END;
	}
	//set input data format
	scale_config.id = SCALE_PATH_INPUT_FORMAT;
	data_format = input_fmt;
	scale_config.param = &data_format;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret =  -1;
                  goto CAMERA_SCALE_FUNCTIONS_END;
	}
	//set output data format
	scale_config.id = SCALE_PATH_OUTPUT_FORMAT;
	data_format = output_fmt;
	scale_config.param = &data_format;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret =  -1;
                  goto CAMERA_SCALE_FUNCTIONS_END;
	}
	//set input size
	scale_config.id = SCALE_PATH_INPUT_SIZE;
	scale_size.w = input_width;// trim_rect->w;
	scale_size.h = input_height;//trim_rect->h;
	scale_config.param = &scale_size;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                   goto CAMERA_SCALE_FUNCTIONS_END;
	}
	//set output size
	scale_config.id = SCALE_PATH_OUTPUT_SIZE;
	scale_size.w = output_width;
	scale_size.h = output_height;
	scale_config.param = &scale_size;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_SCALE_FUNCTIONS_END;
	}	
	//set input size
	scale_config.id = SCALE_PATH_INPUT_RECT;
	scale_rect.x = trim_rect->x;
	scale_rect.y = trim_rect->y;
	scale_rect.w = trim_rect->w;
	scale_rect.h = trim_rect->h;
	scale_config.param = &scale_rect;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_SCALE_FUNCTIONS_END;
	}
	//set input address
	scale_config.id = SCALE_PATH_INPUT_ADDR;
	scale_address.yaddr = input_yaddr;
	scale_address.uaddr = intput_uvaddr;
	scale_address.vaddr = scale_address.uaddr;
	scale_config.param = &scale_address;
	ALOGV("[SPRD OEM]:camera_interpolation,scale input y addr:0x%x,uv addr:0x%x.",scale_address.yaddr,scale_address.uaddr);
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_SCALE_FUNCTIONS_END;
	}
	//set output address
	scale_config.id = SCALE_PATH_OUTPUT_ADDR;
	scale_address.yaddr = output_yaddr;
	scale_address.uaddr = output_uvaddr;//output_addr + output_width * output_height;
	scale_address.vaddr = scale_address.uaddr;
	scale_config.param = &scale_address;
	ALOGV("[SPRD OEM]:camera_interpolation,scale out  y addr:0x%x,uv addr:0x%x.",scale_address.yaddr,scale_address.uaddr);;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_SCALE_FUNCTIONS_END;
	}
	if(output_width > SCALE_OUT_WIDTH_MAX){
		//set slice mode
		scale_config.id = SCALE_PATH_SLICE_SCALE_EN;
		enable = 1;
		scale_config.param = &enable;
		if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
		{
			ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
			ret = -1;
                           goto CAMERA_SCALE_FUNCTIONS_END;
		}
		//set slice height
		scale_config.id = SCALE_PATH_SLICE_SCALE_HEIGHT;
		slice_height = SCALE_SLICE_HEIGHT;
		scale_config.param = &slice_height;
		if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
		{
			ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
			ret = -1;
                           goto CAMERA_SCALE_FUNCTIONS_END;
		}
		//set swap and line buffer address
		scale_config.id = SCALE_PATH_SWAP_BUFF;
		scale_address.yaddr = g_slice_swap_buf; // 384k bytes  //output_w * slice_h  the buff size is for 5M.
		scale_address.uaddr = scale_address.yaddr + (output_width*SCALE_SLICE_HEIGHT); // 384k bytes
		scale_address.vaddr = scale_address.uaddr +  (output_width*SCALE_SLICE_HEIGHT);  // 256k bytes //output_w * 4
		scale_config.param = &scale_address;
		ALOGV("[SPRD OEM]:camera_interpolation,set scale config g_slice_swap_buf=0x%x,scale_address.uaddr=0x%x,scale_address.vaddr=0x%x.",
			    g_slice_swap_buf,scale_address.uaddr,scale_address.vaddr);
		if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
		{
			ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
			ret = -1;
                           goto CAMERA_SCALE_FUNCTIONS_END;
		}		
	}
	//set input endian
	scale_config.id = SCALE_PATH_INPUT_ENDIAN;
	memcpy(&in_endian, &input_endian, sizeof(ISP_ENDIAN_T));
	scale_config.param = &in_endian;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_SCALE_FUNCTIONS_END;
	}
	//set output endian
	scale_config.id = SCALE_PATH_OUTPUT_ENDIAN;
	memcpy(&out_endian, &output_endian, sizeof(ISP_ENDIAN_T));
	scale_config.param = &out_endian;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		ret = -1;
                  goto CAMERA_SCALE_FUNCTIONS_END;
	}

	//done
	if (-1 == xioctl(fd, SCALE_IOC_DONE, 0))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_DONE");
		ret = -1;
	}
CAMERA_SCALE_FUNCTIONS_END:    
	if(-1 == close(fd))   
	{   
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to close scale device.");
        		return -1;   
   	 } 
    	fd = -1;
	return ret;
}

static int camera_crop_interpolation(SCALE_DATA_FORMAT_E output_fmt, uint32_t output_width, uint32_t output_height,
	                                                                                                   uint32_t output_yaddr,uint32_t output_uvaddr, ZOOM_TRIM_RECT_T *trim_rect,
	                                                                                                   uint32_t input_width,uint32_t input_height,uint32_t input_yaddr,
	                                                                                                   uint32_t intput_uvaddr, SCALE_DATA_FORMAT_E input_fmt)
{
	static int fd = -1;
	SCALE_CONFIG_T scale_config;
	SCALE_SIZE_T scale_size;
	SCALE_RECT_T scale_rect;
	SCALE_ADDRESS_T scale_address;
	SCALE_DATA_FORMAT_E data_format;
	SCALE_MODE_E scale_mode;
	uint32_t enable = 0, mode;
	uint32_t slice_height = 0;
	ISP_ENDIAN_T in_endian;
	ISP_ENDIAN_T out_endian;
         int ret = 0;

	fd = open("/dev/sprd_scale", O_RDONLY);
	if (-1 == fd)
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to open scale device.\n");
        	return -1;
   	 }

	//set mode
	scale_config.id = SCALE_PATH_MODE;
	scale_mode = SCALE_MODE_SCALE;
	scale_config.param = &scale_mode;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
                  ret = -1;
		goto CROP_INTERPOLATION_END;
	}
	//set input data format
	scale_config.id = SCALE_PATH_INPUT_FORMAT;
	data_format = input_fmt;
	scale_config.param = &data_format;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
                  ret = -1;        
		goto CROP_INTERPOLATION_END;
	}
	//set output data format
	scale_config.id = SCALE_PATH_OUTPUT_FORMAT;
	data_format = output_fmt;
	scale_config.param = &data_format;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
                  ret = -1;        
		goto CROP_INTERPOLATION_END;
	}
	//set input size
	scale_config.id = SCALE_PATH_INPUT_SIZE;
	scale_size.w = input_width;// trim_rect->w;
	scale_size.h = input_height;//trim_rect->h;
	scale_config.param = &scale_size;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
                  ret = -1;        
		goto CROP_INTERPOLATION_END;
	}
	//set output size
	scale_config.id = SCALE_PATH_OUTPUT_SIZE;
	scale_size.w = output_width;
	scale_size.h = output_height;
	scale_config.param = &scale_size;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
                  ret = -1;        
		goto CROP_INTERPOLATION_END;
	}
	//set input size
	scale_config.id = SCALE_PATH_INPUT_RECT;
	scale_rect.x = trim_rect->x;
	scale_rect.y = trim_rect->y;
	scale_rect.w = trim_rect->w;
	scale_rect.h = trim_rect->h;
	scale_config.param = &scale_rect;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
                  ret = -1;        
		goto CROP_INTERPOLATION_END;
	}
	//set input address
	scale_config.id = SCALE_PATH_INPUT_ADDR;
	scale_address.yaddr = input_yaddr;
	scale_address.uaddr = intput_uvaddr;
	scale_address.vaddr = scale_address.uaddr;
	scale_config.param = &scale_address;
	ALOGV("[SPRD OEM]:camera_interpolation,scale input y addr:0x%x,uv addr:0x%x.",scale_address.yaddr,scale_address.uaddr);
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
                  ret = -1;        
		goto CROP_INTERPOLATION_END;
	}
	//set output address
	scale_config.id = SCALE_PATH_OUTPUT_ADDR;
	scale_address.yaddr = output_yaddr;
	scale_address.uaddr = output_uvaddr;//output_addr + output_width * output_height;
	scale_address.vaddr = scale_address.uaddr;
	scale_config.param = &scale_address;
	ALOGV("[SPRD OEM]:camera_interpolation,scale out  y addr:0x%x,uv addr:0x%x.",scale_address.yaddr,scale_address.uaddr);;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
                  ret = -1;        
		goto CROP_INTERPOLATION_END;
	}
	if(output_width > SCALE_OUT_WIDTH_MAX){
		//set slice mode
		scale_config.id = SCALE_PATH_SLICE_SCALE_EN;
		enable = 1;
		scale_config.param = &enable;
		if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
		{
			ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
                           ret = -1;            
			goto CROP_INTERPOLATION_END;
		}
		//set slice height
		scale_config.id = SCALE_PATH_SLICE_SCALE_HEIGHT;
		slice_height = SCALE_SLICE_HEIGHT;
		scale_config.param = &slice_height;
		if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
		{
			ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
                           ret = -1;
			goto CROP_INTERPOLATION_END;
		}
		//set swap and line buffer address
		scale_config.id = SCALE_PATH_SWAP_BUFF;
		scale_address.yaddr = g_slice_swap_buf; // 384k bytes  //output_w * slice_h  the buff size is for 5M.
		scale_address.uaddr = scale_address.yaddr + (output_width * SCALE_SLICE_HEIGHT); // 384k bytes
		scale_address.vaddr = scale_address.uaddr + (output_width * SCALE_SLICE_HEIGHT);  // 256k bytes //output_w * 4
		scale_config.param = &scale_address;
		ALOGV("[SPRD OEM]:camera_interpolation,set scale config g_slice_swap_buf=0x%x,scale_address.uaddr=0x%x,scale_address.vaddr=0x%x.",
			    g_slice_swap_buf,scale_address.uaddr,scale_address.vaddr);
		if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
		{
			ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
                           ret = -1;
			goto CROP_INTERPOLATION_END;
		}
	}
	//set input endian
	scale_config.id = SCALE_PATH_INPUT_ENDIAN;
	in_endian.endian_y = 1;
	in_endian.endian_uv = 1;
	scale_config.param = &in_endian;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
                  ret = -1;
		goto CROP_INTERPOLATION_END;
	}
	//set output endian
	scale_config.id = SCALE_PATH_OUTPUT_ENDIAN;
	out_endian.endian_y = 1;
	out_endian.endian_uv = 1;
	scale_config.param = &out_endian;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
                  ret = -1;        
		goto CROP_INTERPOLATION_END;
	}

	//done
	if (-1 == xioctl(fd, SCALE_IOC_DONE, 0))
	{
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to SCALE_IOC_DONE");
                  ret = -1;
	}
CROP_INTERPOLATION_END:
	if(-1 == close(fd))   
	{   
		ALOGE("[SPRD OEM ERR]:camera_interpolation,Fail to close scale device.");
        		return -1;   
   	 } 
    	fd = -1;
	return ret;
}

int camera_rotation(uint32_t agree, uint32_t width, uint32_t height, uint32_t in_addr, uint32_t out_addr)
{
	int fd = -1;
         int ret = 0;
	ROTATION_PARAM_T rot_params;

	rot_params.data_format = ROTATION_YUV420;
	switch(agree){
		case 90:
			rot_params.rotation_dir = ROTATION_90;
			break;
		case 180:
			rot_params.rotation_dir = ROTATION_180;
			break;
		case 270:
			rot_params.rotation_dir = ROTATION_270;
			break;
		default:
			rot_params.rotation_dir = ROTATION_DIR_MAX;
			break;
	}
	rot_params.img_size.w = width;
	rot_params.img_size.h = height;
	rot_params.src_addr.y_addr = in_addr;
	rot_params.src_addr.uv_addr = rot_params.src_addr.y_addr + rot_params.img_size.w * rot_params.img_size.h;
	rot_params.src_addr.v_addr = rot_params.src_addr.uv_addr;
	rot_params.dst_addr.y_addr = out_addr;
	rot_params.dst_addr.uv_addr = rot_params.dst_addr.y_addr + rot_params.img_size.w * rot_params.img_size.h;
	rot_params.dst_addr.v_addr = rot_params.dst_addr.uv_addr;

	fd = open("/dev/sprd_rotation", O_RDWR /* required */, 0);
	if (-1 == fd)
	{
		ALOGE("Fail to open rotation device.");
        	return -1;
   	}

	//done
	if (-1 == xioctl(fd, SPRD_ROTATION_DONE, &rot_params))
	{
		ALOGE("Fail to SC8800G_ROTATION_DONE");
		ret = -1;
	}

	if(-1 == close(fd))
	{
		ALOGE("Fail to close rotation device.");
        		return -1;
   	 }
    	fd = -1;
	return ret;
}
#if 1
int camera_rotation_copy_data(uint32_t width, uint32_t height, uint32_t in_addr, uint32_t out_addr)
{
	int fd = -1;
	ROTATION_PARAM_T rot_params;

	rot_params.data_format = ROTATION_YUV420;
	rot_params.img_size.w = width;
	rot_params.img_size.h = height;
	rot_params.src_addr.y_addr = in_addr;
	rot_params.src_addr.uv_addr = rot_params.src_addr.y_addr + rot_params.img_size.w * rot_params.img_size.h;
	rot_params.src_addr.v_addr = rot_params.src_addr.uv_addr;
	rot_params.dst_addr.y_addr = out_addr;
	rot_params.dst_addr.uv_addr = rot_params.dst_addr.y_addr + rot_params.img_size.w * rot_params.img_size.h;
	rot_params.dst_addr.v_addr = rot_params.dst_addr.uv_addr;

	fd = open("/dev/sprd_rotation", O_RDWR /* required */, 0);
	if (-1 == fd)
	{
		ALOGE("Fail to open rotation device.");
        	return -1;
   	}

	//done
	if (-1 == xioctl(fd, SPRD_ROTATION_DATA_COPY, &rot_params))
	{
		ALOGE("Fail to SC8800G_ROTATION_DATA_COPY");
		return -1;
	}

	if(-1 == close(fd))
	{
		ALOGE("Fail to close rotation device.");
        		return -1;
   	 }
    	fd = -1;
	return 0;
}
#endif

#ifndef USE_ION_MEM
/* Copy data from to address */
int camera_rotation_copy_data_virtual(uint32_t width, uint32_t height, uint32_t in_addr, uint32_t out_virtual_addr)
{
	int fd = -1;
	ROTATION_PARAM_T rot_params;

	rot_params.data_format = ROTATION_YUV420;
	rot_params.img_size.w = width;
	rot_params.img_size.h = height;
	rot_params.src_addr.y_addr = in_addr;
	rot_params.src_addr.uv_addr = rot_params.src_addr.y_addr + rot_params.img_size.w * rot_params.img_size.h;
	rot_params.src_addr.v_addr = rot_params.src_addr.uv_addr;
	rot_params.dst_addr.y_addr = out_virtual_addr;
	rot_params.dst_addr.uv_addr = rot_params.dst_addr.y_addr + rot_params.img_size.w * rot_params.img_size.h;
	rot_params.dst_addr.v_addr = rot_params.dst_addr.uv_addr;

	fd = open("/dev/sprd_rotation", O_RDWR /* required */, 0);
	if (-1 == fd)
	{
		ALOGE("Fail to open rotation device.");
		return -1;
	}

	//done
	if (-1 == xioctl(fd, SPRD_ROTATION_DATA_COPY_VIRTUAL, &rot_params))
	{
		ALOGE("Fail to SC8800G_ROTATION_DATA_COPY");
		return -1;
	}

	if(-1 == close(fd))
	{
		ALOGE("Fail to close rotation device.");
		return -1;
	}
	fd = -1;
	return 0;
}
#endif

uint32_t get_stop_flag(void)
{
	if(1 ==  g_stop_preview_flag)
		return 1;
	else
		return 0;
}
uint32_t check_stop(void)
{
	if(1 == get_stop_flag())
	{
		g_preview_stop = 1;
		ALOGV("Stop preview for g_stop_preview_flag.");
		return 1;
	}
	else
	{
		//ALOGV("Stop preview: g_stop_preview_flag is 0.");
		return 0;
	}
}

//convert the endian of the preview from half word endian to little endian by DMA.
//make the scaling output endian is half word endian; it's the same as dcam's.
static int convert_preview_endian_by_DMA(uint32_t width, uint32_t height, uint32_t out_address,  uint32_t in_address)
{
	SCALE_YUV420_ENDIAN_T yuv_config;
	int fd = -1;
         int ret = 0;

	fd = open("/dev/sprd_scale", O_RDONLY);
	if (-1 == fd)
	{
		ALOGE("Fail to open scale device for preview DMA.");
		return -1;
   	 }
	yuv_config.width = width;
	yuv_config.height = height;
	yuv_config.src_addr = in_address;
	yuv_config.dst_addr = out_address;

	if (-1 == xioctl(fd, SCALE_IOC_YUV420_ENDIAN, &yuv_config))
	{
		ALOGE("Fail to SCALE_IOC_YUV420_ENDIAN.");
		ret = -1;
	}

	if(-1 == close(fd))
	{
		ALOGE("Fail to close scale device for preview DMA.");
        		return -1;
   	 }
    	fd = -1;
	return ret;	
}

camera_ret_code_type camera_start_preview_for_restart(camera_cb_f_type callback, void *client_data);
camera_ret_code_type camera_stop_preview_for_restart(void);
void *camera_restart_thread(void *client_data)
{
	g_preview_stop = 1;
	camera_stop_preview_for_restart();
	close_device();
	camera_start_preview_for_restart(g_callback, client_data);
	return NULL;
}

void *camera_preview_thread(void *client_data)
{
	struct v4l2_buffer buf;
	uint32_t i, ret = 0;
	camera_frame_type frame_type;
	static uint32_t is_odd = 0;
	uint32_t output_addr;
	struct zoom_trim_rect trim_rect;
	nsecs_t timestamp;
	nsecs_t timestamp_old, timestamp_new;
	int frame_delay = 45000000;
	int sleep_time = 0;
	uint32_t frame_num = 0;
          uint32_t first = 0;

	ALOGV("===============Start preview thread=====================.");
	while(1)
	{
		CLEAR (buf);
	        	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	        	buf.memory = V4L2_MEMORY_USERPTR;
	        	if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
		{
			if(1 == check_stop()){
				return NULL;
			}
			ALOGE("Fail to VIDIOC_DQBUF.");
			g_callback(CAMERA_EXIT_CB_FAILED, client_data, CAMERA_FUNC_START_PREVIEW, NULL);
			return NULL;
		}
		else
		{
			if(frame_num == 0)
			{
				ALOGV("OK to VIDIOC_DQBUF.buf.index: %d. userptr: %lx", buf.index, buf.m.userptr);
			}
			else
			{
				frame_num++;
				frame_num = frame_num%3;
			}
		}
    		for (i = G_PREVIEW_BUF_OFFSET; i < n_buffers; ++i)
     		{
  		     	if (buf.m.userptr == (unsigned long) g_buffers[i].phys_addr)
			{
				g_releasebuff_index = i;
                			break;
			}
    		 }
			#if 0
                   {
				FILE *fp = NULL;

				{
				ALOGE("preview yuv420: width: %d, hei: %d.", g_dcam_dimensions.display_width, g_dcam_dimensions.display_height);
				fp = fopen("/data/out_pre_yuv420_dma.raw", "wb");
				fwrite(g_buffers[i].virt_addr, 1, g_dcam_dimensions.display_width * g_dcam_dimensions.display_height * 3 / 2, fp);
				fclose(fp);
				}

		}
			#endif
   	     	if(i < n_buffers)
		{
   	         #if 0
			{
				FILE *fp = NULL;
				static uint32_t first = 1;
				if(10 == first)
				{
				ALOGE("preview yuv420: width: %d, hei: %d, addrs: 0x%x.", g_dcam_dimensions.display_width, g_dcam_dimensions.display_height, (uint32_t)frame_type.buf_Virt_Addr);
				fp = fopen("/data/out_pre_yuv420_dma.yuv", "wb");
				fwrite(frame_type.buf_Virt_Addr, 1, g_dcam_dimensions.display_width * g_dcam_dimensions.display_height * 3 / 2, fp);
				fclose(fp);
				}
				first++;
			}
		 #endif

			//if((1 == s_camera_info.is_need_rotation)&&((90 ==  s_camera_info.rot_angle)||(270 ==  s_camera_info.rot_angle)))
                        if(1 == s_camera_info.is_need_rotation)
			{
				ZOOM_TRIM_RECT_T trim_rect = {0,0,0,0};
				uint32_t dst_y_phy_addr, dst_uv_phy_addr, src_y_phy_addr, src_uv_phy_addr;

				timestamp_old = systemTime();
				ret = camera_rotation(g_rotation, s_camera_info.dcam_out_width, s_camera_info.dcam_out_height, g_buffers[g_releasebuff_index].phys_addr,g_buffers[g_preview_buffer_num ].phys_addr);
				if(0 != ret)
				{
					ALOGE("SPRD OEM:Fail to preview because camera_rotation");
					g_callback(CAMERA_EXIT_CB_FAILED, client_data, CAMERA_FUNC_START_PREVIEW, NULL);
					camera_release_frame(g_releasebuff_index);
					if(1 == check_stop())
						return NULL;
					continue;
				}
				timestamp_new = systemTime();
				//ALOGV("SPRD OEM:rotation %lld, %lld, time = %lld us \n",timestamp_old, timestamp_new, (timestamp_new-timestamp_old)/1000);

				trim_rect.x = 0;
				trim_rect.y = 0;
				trim_rect.w = s_camera_info.dcam_out_height;
				trim_rect.h = s_camera_info.dcam_out_width;
				dst_y_phy_addr = g_buffers[g_releasebuff_index].phys_addr;
				dst_uv_phy_addr = dst_y_phy_addr + trim_rect.w*trim_rect.h;
				src_y_phy_addr = g_buffers[g_preview_buffer_num].phys_addr;
				src_uv_phy_addr = src_y_phy_addr + trim_rect.w*trim_rect.h;

				timestamp_old = systemTime();
				ret = camera_copy(SCALE_DATA_YUV420,trim_rect.w,trim_rect.h,dst_y_phy_addr,dst_uv_phy_addr,
		                   		&trim_rect,src_y_phy_addr,src_uv_phy_addr,SCALE_DATA_YUV420);
				if(0 != ret)
				{
					ALOGE("SPRD OEM:Fail to preview because camera_rotation camera_copy");
				}
				timestamp_new = systemTime();
				//ALOGV("SPRD OEM:copy     %lld, %lld, time = %lld us \n",timestamp_old, timestamp_new, (timestamp_new-timestamp_old)/1000);

#if 0
				trim_rect.x = 0;
				trim_rect.y = 0;
				trim_rect.w = s_camera_info.dcam_out_height;
				trim_rect.h = s_camera_info.dcam_out_width;
				ret = camera_cap_zoom_colorformat(SCALE_DATA_YUV420,s_camera_info.dcam_out_height,s_camera_info.dcam_out_width,
					                                                               g_buffers[g_releasebuff_index].phys_addr,&trim_rect,g_buffers[g_preview_buffer_num ].phys_addr,SCALE_DATA_YUV420);
				if(0 != ret)
				{
					ALOGE("SPRD OEM:Fail to preview because camera_rotation");
					g_callback(CAMERA_EXIT_CB_FAILED, client_data, CAMERA_FUNC_START_PREVIEW, NULL);
					camera_release_frame(g_releasebuff_index);
					if(1 == check_stop())
						return NULL;
					continue;
				}
#endif
			}
			frame_type.buf_Virt_Addr  = g_buffers[g_releasebuff_index].virt_addr;
			frame_type.buffer_phy_addr  = g_buffers[g_releasebuff_index].phys_addr;
			frame_type.buf_id = g_releasebuff_index;
			frame_type.dx = g_dcam_dimensions.display_width;
			frame_type.dy = g_dcam_dimensions.display_height;
			frame_type.format = CAMERA_YCBCR_4_2_0;
                           frame_type.timestamp = buf.timestamp.tv_sec * 1000000000LL + buf.timestamp.tv_usec * 1000;
			g_callback(CAMERA_EVT_CB_FRAME, client_data, CAMERA_FUNC_START_PREVIEW, (uint32_t)&frame_type);
   	     	}
		else
		{
			ALOGE("SPRD OEM:Fail to DQBUF:userptr: %lx, len: %x.i = %d.", buf.m.userptr, buf.length,i);
		}
		if(1 == check_stop())
		{
			return NULL;
		}
	}

	ALOGV("===============Stop preview thread=====================.");
	return NULL;
}

int camera_preview_init(int preview_mode)
{
	struct v4l2_format fmt;
	uint32_t min;
	struct v4l2_requestbuffers req;
	uint32_t buffer_size, page_size, frame_size;
	struct v4l2_streamparm streamparm;
         uint32_t orientation_parm = g_cam_params.orientation_parm;
	uint32_t rotation_parm = g_rotation_parm;
	uint32_t buff_cnt = g_preview_buffer_num;

	s_camera_info.is_need_rotation = 0;
	s_camera_info.rot_angle = 0;
	s_camera_info.out_format = DCAM_DATA_YUV420;

	CLEAR (fmt);
    	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width  = g_dcam_dimensions.display_width;
    	fmt.fmt.pix.height = g_dcam_dimensions.display_height;
	switch(preview_mode)
	{
		case 0:
			fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV422P;
			break;
		case 1:
			fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
			break;
		case 2:
			fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
			break;
		default:
			fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
			break;

	}
    	fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

        	ALOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");
        	ALOGV("=====will set fmt to (%d, %d)--\n", fmt.fmt.pix.width,fmt.fmt.pix.height);
	if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
	{
		ALOGV("V4L2_PIX_FMT_YUYV\n");
	}else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420)
	{
		ALOGV("V4L2_PIX_FMT_YUV420\n");
	}else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV422P)
	{
		ALOGV("V4L2_PIX_FMT_YUV422P\n");
	}else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB32)
	{
		ALOGV("V4L2_PIX_FMT_RGB32\n");
	}else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB565X)
	{
		ALOGV("V4L2_PIX_FMT_RGB565X\n");
	}
    camera_set_rot_angle(&orientation_parm,&rotation_parm);
	if(1 == orientation_parm)
	{

		s_camera_info.is_need_rotation = 1;
		s_camera_info.rot_angle = rotation_parm;
		buff_cnt += 1;

		fmt.fmt.raw_data[199] =1;

		if( 90 == rotation_parm)
		{
			fmt.fmt.pix.width  = g_dcam_dimensions.display_height;
		    	fmt.fmt.pix.height = g_dcam_dimensions.display_width;
			fmt.fmt.raw_data[198] = 1;
		}
		else if(270 == rotation_parm)
		{
			fmt.fmt.pix.width  = g_dcam_dimensions.display_height;
		    	fmt.fmt.pix.height = g_dcam_dimensions.display_width;
			fmt.fmt.raw_data[198] = 2;
		}
		else if (180 == rotation_parm)
		{
			fmt.fmt.pix.width  = g_dcam_dimensions.display_width;
		    	fmt.fmt.pix.height = g_dcam_dimensions.display_height;
			fmt.fmt.raw_data[198] = 3;
		}
	}

    	if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
	{
		ALOGE("Fail to VIDIOC_S_FMT.");
		return -1;
	}

	ALOGV("=====after set fmt=======\n");
	ALOGV("     g_dcam_dimensions.display_width=%d.g_dcam_dimensions.display_height=%d .\n",
		           g_dcam_dimensions.display_width,g_dcam_dimensions.display_height);
	ALOGV("   orientation=%d,rotation_parm=%d, g_rotation=%d .\n",orientation_parm,rotation_parm, g_rotation);	
	ALOGV("    fmt.fmt.pix.width = %d\n", fmt.fmt.pix.width);
	ALOGV("    fmt.fmt.pix.height = %d\n", fmt.fmt.pix.height);
	ALOGV("    fmt.fmt.pix.sizeimage = %d\n", fmt.fmt.pix.sizeimage);
	ALOGV("    fmt.fmt.pix.bytesperline = %d\n", fmt.fmt.pix.bytesperline);
	ALOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");

    /* Note VIDIOC_S_FMT may change width and height. */
   	 ALOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");
    /* Buggy driver paranoia. */
    	min = fmt.fmt.pix.width * 2;
    	if (fmt.fmt.pix.bytesperline < min)
        	fmt.fmt.pix.bytesperline = min;
    	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    	if (fmt.fmt.pix.sizeimage < min)
        	fmt.fmt.pix.sizeimage = min;
	ALOGV("After Buggy driver paranoia\n");
	ALOGV("    >>fmt.fmt.pix.sizeimage = %d\n", fmt.fmt.pix.sizeimage);
	ALOGV("    >>fmt.fmt.pix.bytesperline = %d\n", fmt.fmt.pix.bytesperline);
	ALOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");

	CLEAR (req);
	req.count = g_preview_buffer_num;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    	req.memory = V4L2_MEMORY_USERPTR;
    	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
	{
		ALOGE("Fail to VIDIOC_REQBUFS.");
		return -1;
	}

	g_buffers = (buffer *)calloc(buff_cnt, sizeof(*g_buffers));
	if (!g_buffers)
	{
		ALOGE("Fail to malloc preview buffer struct.");
		return -1;
	}

	frame_size = g_dcam_dimensions.display_width * g_dcam_dimensions.display_height * 3 / 2;
	//buffer_size = frame_size;
	buffer_size = (frame_size + 256 - 1) & ~(256 - 1);
    	for (n_buffers = 0; n_buffers < buff_cnt; ++n_buffers)
	{
		g_buffers[n_buffers].length = buffer_size;
		g_buffers[n_buffers].virt_addr = g_preview_virt_addr + n_buffers * buffer_size / 4;
		g_buffers[n_buffers].phys_addr = g_preview_phys_addr + n_buffers * buffer_size;
		g_buffers[n_buffers].u_virt_addr = 0;
		g_buffers[n_buffers].u_phys_addr = 0;
		g_buffers[n_buffers].v_virt_addr = 0;
		g_buffers[n_buffers].v_phys_addr = 0;
		ALOGV("preview: g_buffer[%d]: virt: %x, phys: %x, len: %x.\n",
			 n_buffers,(uint32_t)g_buffers[n_buffers].virt_addr,g_buffers[n_buffers].phys_addr,g_buffers[n_buffers].length);
        }

	CLEAR (streamparm);
	streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	streamparm.parm.capture.capturemode = 0;
	if(-1 == g_camera_id)
	{
		streamparm.parm.raw_data[199] = 0;
	}
	else
	{
		streamparm.parm.raw_data[199] = 1;
		streamparm.parm.raw_data[198] = g_camera_id;
	}
	g_rotation = rotation_parm;
	g_cam_params.orientation = orientation_parm;

	s_camera_info.dcam_out_width = fmt.fmt.pix.width;
	s_camera_info.dcam_out_height = fmt.fmt.pix.height;
	if(1 == g_cam_params.orientation)
	{
			streamparm.parm.raw_data[197] = 0;//rotation flag
			s_camera_info.is_need_rotation = 1;
			s_camera_info.rot_angle = g_rotation;
	}
	else
	{
		streamparm.parm.raw_data[197] = 0;
	}

	if (-1 == xioctl(fd, VIDIOC_S_PARM, &streamparm))
	{
		ALOGE("preview: Fail to VIDIOC_S_PARM.\n");
		return -1;
	}
	ALOGV("SPRD OEM:preview init OK!");

	g_preview_stop = 0;
#if 0
	if(1 == g_camera_id)
	{ //for front camera
		uint32_t enable = 0;
		if((90 == s_camera_info.set_encode_rotation) || (270 == s_camera_info.set_encode_rotation))
		{ //for portrait
			camera_set_ctrl(V4L2_CID_VFLIP, enable);
		}
		else{ //for lanscape
			camera_set_ctrl(V4L2_CID_HFLIP, enable);
		}
	}
#endif
	return 0;
}

int camera_preview_qbuffers(void)
{
	uint32_t i;

	for (i = G_PREVIEW_BUF_OFFSET; i < g_preview_buffer_num ; ++i)
	{
		struct v4l2_buffer buf;
		CLEAR (buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_USERPTR;
		buf.index = i;
		buf.m.userptr = g_buffers[i].phys_addr;
		buf.length = g_buffers[i].length;
		buf.reserved = 0;
		if(0 != g_buffers[i].u_phys_addr)
		{
			buf.reserved = g_buffers[i].u_phys_addr;
		}

		ALOGV("####preview QBuf: buffers[%d].start: %lx, %x\n", buf.index, buf.m.userptr,buf.length);
		if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
		{
			ALOGE("Fail to VIDIOC_QBUF from camera_preview_qbuffers.");
			return -1;
		}
          }
	return 0;
}

int camera_preview_streamon(void)
{
	enum v4l2_buf_type type;
	g_buf_flag[0] = 0;
	g_buf_flag[1] = 0;
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
	{
		ALOGE("Fail to VIDIOC_STREAMON.");
		return -1;
	}
	return 0;
}

camera_ret_code_type camera_start_preview (camera_cb_f_type callback, void *client_data)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;
	g_dcam_obj = (SprdCameraHardware *)client_data;
	int ret;
	uint32_t size, page_size, buffer_size,frame_size;
	int preview_format;
         pthread_attr_t attr;

	//record the callback function
	g_callback = callback;

	//get the preview memory
	//frame_size = g_dcam_dimensions.display_width * g_dcam_dimensions.display_height * 2; //for YUV422
	//buffer_size = camera_get_size_align_page(frame_size);
	frame_size = g_dcam_dimensions.display_width * g_dcam_dimensions.display_height *3 / 2; //for YUV422
	buffer_size = frame_size;
	size = buffer_size * g_preview_buffer_num;
	if(NULL == (g_preview_virt_addr = (uint32_t *)g_dcam_obj->get_preview_mem(size,&g_preview_phys_addr,0)))
	{
		ALOGE("Fail to malloc preview memory.");
		return CAMERA_FAILED;
	}
	g_dcam_obj->getPreviewFormat(&preview_format);
	ALOGV("camera_start_preview:preview format is %d .\n",preview_format);
	if(0 != camera_preview_init(preview_format))
	{
		ALOGE("Fail to init preview mode.");
		return CAMERA_FAILED;
	}
	ALOGV("OK  to camera_preview_init.\n");

	if(0 != camera_preview_qbuffers())
	{
		ALOGE("Fail to qbuffers for preview.");
		return CAMERA_FAILED;
	}
	ALOGV("OK  to camera_preview_qbuffers.\n");

	if(0 != camera_preview_streamon())
	{
		ALOGE("Fail to stream on for preview.");
		return CAMERA_FAILED;
	}
	ALOGV("OK  to camera_preview_streamon.");

	//update the status
	callback(CAMERA_RSP_CB_SUCCESS, g_dcam_obj, CAMERA_FUNC_START_PREVIEW, 0);
	ALOGV("OK to CAMERA_FUNC_START_PREVIEW.");

	//create the thread for preview
	pthread_attr_init (&attr);
         pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	if(0 != (ret = pthread_create(&g_preview_thr, &attr, camera_preview_thread, client_data)))
	{
		ALOGE("Fail to careate thread in preview mode.");
		return CAMERA_FAILED;
	}
	else
	{
		//pthread_join(g_preview_thr, NULL);
		ALOGV("OK to careate thread in preview mode.");
	}
	return ret_type;
}

camera_ret_code_type camera_start_preview_for_restart(camera_cb_f_type callback, void *client_data)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;
	g_dcam_obj = (SprdCameraHardware *)client_data;
	int ret;
	uint32_t size, page_size, buffer_size,frame_size;
	int preview_format;

	ALOGV("camera_start_preview_for_restart S.");

	if(CAMERA_SUCCESS != camera_init( g_camera_id))
	{
		g_callback(CAMERA_EXIT_CB_FAILED, client_data, CAMERA_FUNC_START_PREVIEW, NULL);
		ALOGV("SPRD OEM:camera_init fail.");
		return CAMERA_FAILED;
	}
          //get the preview memory
	frame_size = g_dcam_dimensions.display_width * g_dcam_dimensions.display_height * 2; //for YUV422
	buffer_size = camera_get_size_align_page(frame_size);
	size = buffer_size * g_preview_buffer_num;
	if(NULL == (g_preview_virt_addr = (uint32_t *)g_dcam_obj->get_preview_mem(size,&g_preview_phys_addr,0)))
	{
		ALOGE("SPRD OEM:Fail to malloc preview memory.");
		return CAMERA_FAILED;
	}
	g_dcam_obj->getPreviewFormat(&preview_format);
	ALOGV("SPRD OEM:camera_start_preview_for_restart 1.");

	if(0 != camera_preview_init(preview_format))
	{
		ALOGE("SPRD OEM:Fail to init preview mode.");
		return CAMERA_FAILED;
	}
	ALOGV("SPRD OEM:OK  to camera_preview_init.");

	if(0 != camera_preview_qbuffers())
	{
		ALOGE("SPRD OEM:Fail to qbuffers for preview.");
		return CAMERA_FAILED;
	}
	ALOGV("SPRD OEM:OK  to camera_preview_qbuffers.");

	if(0 != camera_preview_streamon())
	{
		ALOGE("SPRD OEM:Fail to stream on for preview.");
		return CAMERA_FAILED;
	}
	ALOGV("SPRD OEM:OK  to camera_preview_streamon.");

	//create the thread for preview
	if(0 != (ret = pthread_create(&g_preview_thr, NULL, camera_preview_thread, client_data)))
	{
		ALOGE("SPRD OEM:Fail to careate thread in preview mode.");
		return CAMERA_FAILED;
	}
	else
	{
		//pthread_join(g_preview_thr, NULL);
		ALOGV("SPRD OEM:OK to careate thread in preview mode.");
	}

	return ret_type;
}
static camera_cb_f_type s_af_callback;
static camera_focus_e_type s_af_type;
void* camera_start_focus (void *client_data)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;
	uint16_t focus_param[26] = {0};
	uint32_t *ptr = (uint32_t*)&s_focus_zone_param[0];
	uint32_t i=0;
	uint32_t size_w=ptr[1],size_h=ptr[2];
	uint32_t zone_cnt = ptr[0];
	uint16_t *dst_ptr = &focus_param[2];
	camera_focus_e_type focus = s_af_type;
	camera_cb_f_type callback = s_af_callback;

	ALOGV("SPRD OEM:camera_start_focus S,size_w=%d,size_h=%d,s_af_is_stop=%d.",size_w,size_h,s_af_is_stop);
	//s_af_is_stop = 0;

         if(0 == callback)
         	{
		ALOGV("SPRD OEM:camera_start_focus fail,callbcak is NULL!");
		goto CAMERA_AF_END;
         	}

	//if(CAMERA_FOCUS_MODE_MACRO == g_cam_params.focus_mode)
	if(2 == g_cam_params.focus_mode)
	{
		focus_param[0] = 4;
		focus_param[1] = 0;
		ALOGV("SPRD OEM: camera_start_focus macro \n");

	}
	else
	{

	if(0 == zone_cnt)
	{
		//start focus
		focus_param[0] = 1;
		focus_param[1] = 0;
	}
	else
	{
		focus_param[0] = (1 == zone_cnt) ? 2:3;
		focus_param[1] = zone_cnt;
            		ptr += 1;
		for(i=0;i<zone_cnt;i++)
		{
			*dst_ptr++ = *ptr++;//x
			*dst_ptr++ = *ptr++;//y
            			*dst_ptr++ = *ptr++;//w
            			*dst_ptr++ = *ptr++;//h
		}
	}
	}
         if(s_af_is_cancel)
         {
                ALOGV("SPRD OEM:camera_start_focus ,canceled..0");
		callback(CAMERA_EXIT_CB_ABORT, client_data, CAMERA_FUNC_START_FOCUS, 0);
                s_af_is_stop = 1;
                return 0;
         }
	//if(0 != camera_set_parm(CAMERA_PARM_FOCUS_MODE,(int32_t)&focus_param[0],NULL,NULL))
	if(0 == device_write(fd,(uint8_t*)&focus_param[0],26))
	{
		ret_type = CAMERA_FAILED;
		ALOGE("SPRD OEM:camera_start_focus fail.");
	}

	if(!s_af_is_cancel) {
		//TODO
		if(CAMERA_AUTO_FOCUS == focus)
		{
		      if(s_af_is_stop){
			  	callback(CAMERA_EXIT_CB_ABORT, client_data, CAMERA_FUNC_START_FOCUS, 0);
		      	}
			else if(CAMERA_SUCCESS == ret_type)
			{				
				callback(CAMERA_EXIT_CB_DONE, client_data, CAMERA_FUNC_START_FOCUS, 0);		
			}
			else
			{
				callback(CAMERA_EXIT_CB_FAILED, client_data, CAMERA_FUNC_START_FOCUS, 0);	
			}
		}
		else if(CAMERA_MANUAL_FOCUS == focus)
		{
		}
		else
		{
			ret_type = CAMERA_INVALID_PARM;
			callback(CAMERA_EXIT_CB_FAILED, client_data, CAMERA_FUNC_START_FOCUS, 0);	
		}
	}
	else{
		ALOGV("SPRD OEM:camera_start_focus ,canceled..1");
		callback(CAMERA_EXIT_CB_ABORT, client_data, CAMERA_FUNC_START_FOCUS, 0);
	}
CAMERA_AF_END:
	s_af_is_stop = 1;
	ALOGV("SPRD OEM:camera_start_focus E.");

	return 0;
}

int DcamColorConverter(int src_addr,int dst_addr,int width,int height)
{
	//ALOGI("src_addr=%x,dst_addr=%x,width=%d,height=%d",src_addr,dst_addr,width,height);

	ZOOM_TRIM_RECT_T rect;
	ISP_ENDIAN_T endian_in, endian_out;
	rect.x = 0;
	rect.y = 0;
	rect.w = width;
	rect.h = height;
	endian_in.endian_y = 1;
	endian_in.endian_uv = 1;
	endian_out.endian_y = 1;
	endian_out.endian_uv = 2;
	return  camera_scale_functions(SCALE_DATA_YUV420, width, height,
	                                                          dst_addr,dst_addr+width*height,
	                                                          endian_out,
	                                                          &rect,
	                                                          width,height,src_addr,
	                                                          src_addr+width*height, SCALE_DATA_YUV420,
	                                                          endian_in);
	//0:ok,1:fail
}

int camera_convert_420_UV_VU(int src_addr,int dst_addr,int width,int height)
{
	//ALOGI("src_addr=%x,dst_addr=%x,width=%d,height=%d",src_addr,dst_addr,width,height);

	ZOOM_TRIM_RECT_T rect;
	ISP_ENDIAN_T endian_in, endian_out;
	rect.x = 0;
	rect.y = 0;
	rect.w = width;
	rect.h = height;
	endian_in.endian_y = 1;
	endian_in.endian_uv = 1;
	endian_out.endian_y = 1;
	endian_out.endian_uv = 2;
	return  camera_scale_functions(SCALE_DATA_YUV420, width, height,
	                                                          dst_addr,dst_addr+width*height,
	                                                          endian_out,
	                                                          &rect,
	                                                          width,height,src_addr,
	                                                          src_addr+width*height, SCALE_DATA_YUV420,
	                                                          endian_in);
	//0:ok,1:fail
}

int camera_get_data_redisplay(int output_addr, int output_width, int output_height, int input_addr, int input_width, int input_height)
{
	ALOGI("output_addr=%x,output_width=%x,output_height=%d,input_addr=%x,input_width=%x,input_height=%d",
		 output_addr, output_width, output_height, input_addr, input_width, input_height);

	ZOOM_TRIM_RECT_T rect;
	ISP_ENDIAN_T endian_in, endian_out;
	int src_addr = input_addr;
	int in_width = input_width;
	int in_height = input_height;
	int dst_addr = output_addr;
	int out_width = output_width;
	int out_height = output_height;
	rect.x = 0;
	rect.y = 0;
	rect.w = in_width;
	rect.h = in_height;

	endian_in.endian_y = 1;
	endian_in.endian_uv = 1;
	endian_out.endian_y = 1;
	endian_out.endian_uv = 1;
	return camera_scale_functions(SCALE_DATA_YUV420, out_width, out_height,
	                                                          dst_addr,dst_addr+out_width*out_height,
	                                                          endian_out,
	                                                          &rect,
	                                                          in_width,in_height,src_addr,
	                                                          src_addr+in_width*in_height, SCALE_DATA_YUV420,
	                                                          endian_in);
	//0:ok,1:fail
}

int camera_start_af_thread(camera_focus_e_type focus,
        									      camera_cb_f_type callback,
       									       void *client_data)
{
         pthread_attr_t attr;
	s_af_callback = callback;
	s_af_type = focus;
	int ret = 0;
         if(0 == s_af_is_stop)
         {
                ALOGE("SPRD OEM:previous auto focus don't finish.");
                return CAMERA_FAILED;
         }
         if(s_af_is_cancel)
         {
                ALOGV("SPRD OEM:camera_start_af_thread ,canceled..");
                return 0;
         }
	s_af_is_stop = 0;
         pthread_attr_init (&attr);
         pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	if(0 != (ret = pthread_create(&g_af_thr, &attr, camera_start_focus, client_data)))
	{
		ALOGE("SPRD OEM:camera_start_af_thread faile.");
		s_af_is_stop = 1;
		return CAMERA_FAILED;
	}
	ALOGV("SPRD OEM:camera_start_af_thread OK.");
	return 0;
}
camera_ret_code_type camera_stop_focus (void)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;

	return ret_type;
}


camera_ret_code_type camera_cancel_autofocus (void)
{
        s_af_is_cancel = 1;
         ALOGV("SPRD OEM:camera_cancel_autofocus!s_af_is_stop=%d.", s_af_is_stop);
         while(!s_af_is_stop) {
                  ALOGV("SPRD OEM:camera_cancel_autofocus,wait af finish!");
		usleep(50000);
	}
         s_af_is_cancel = 0;
         ALOGV("SPRD OEM:camera_cancel_autofocus!s_af_is_stop=%d, s_af_is_cancel=%d.", s_af_is_stop, s_af_is_cancel);
	return CAMERA_SUCCESS;
}
static void close_device(void)
{
	if (-1 == close(fd))
		errno_exit("close");
	fd = -1;
	ALOGV("#####DCAM: close device.\n");
}
camera_ret_code_type camera_stop( camera_cb_f_type callback, void *client_data)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;
	close_device();
	//stop camera
	callback(CAMERA_EXIT_CB_DONE, client_data, CAMERA_FUNC_STOP, 0);
	return ret_type;
}

int camera_preview_streamoff(void)
{
	enum v4l2_buf_type type;

	g_rotation = 0;
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
	{
		ALOGE("Fail to VIDIOC_STREAMOFF.");
		return -1;
	}
	return 0;
}
camera_ret_code_type camera_stop_preview(void)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;
	uint32_t num = 0;
	ALOGV("camera_stop_preview: Start to stop preview thread.");

	//stop camera preview thread
	g_stop_preview_flag = 1;
	s_af_is_cancel = 1;
	while(!s_af_is_stop)
	{
		usleep(10000);
		num++;
		ALOGV("waiting the autofocus stop....s_af_is_stop=%d.", s_af_is_stop);
		if(num >= 100){
			//pthread_kill(g_af_thr, 9);
			ALOGE("Fail: Force stop the autofocus!");
			break;
		}
	} //wait preview thread stop
	s_af_is_cancel = 0;	
         if(0 != camera_preview_streamoff())
	{
		ALOGE("Fail to stream off for preview.");
		return CAMERA_FAILED;
	}
	ALOGV("OK to stream off for preview.");
         g_callback(CAMERA_RSP_CB_SUCCESS, g_dcam_obj, CAMERA_FUNC_STOP_PREVIEW, 0);
	ALOGV("OK to stream off for CAMERA_FUNC_STOP_PREVIEW.");
         num = 0;
	while(!g_preview_stop)
	{
		usleep(10000);
		num++;
		ALOGV("waiting the preview stop....");
		if(num >= 100){
			pthread_kill(g_preview_thr, 9);
			ALOGE("Force stop the preview!");
			break;
		}
	} //wait preview thread stop
	g_stop_preview_flag = 0;

	ALOGV("camera_stop_preview: OK to stop preview thread.");	
	return ret_type;
}

camera_ret_code_type camera_stop_preview_for_restart(void)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;
	uint32_t num = 0;
	ALOGV("camera_stop_preview: Start to stop preview thread.");

	//stop camera preview thread
	g_stop_preview_flag = 1;
	if(0 != camera_preview_streamoff())
	{
		ALOGE("Fail to stream off for preview.");
		return CAMERA_FAILED;
	}
	ALOGV("OK to stream off for preview.");
	ALOGV("OK to stream off for CAMERA_FUNC_STOP_PREVIEW.");
	while(!g_preview_stop)
	{
		usleep(10000);
		num++;
		ALOGV("waiting the preview stop....");
		if(num >= 100){
			pthread_kill(g_preview_thr, 9);
			ALOGE("Force stop the preview!");
			break;
		}
	} //wait preview thread stop
	g_stop_preview_flag = 0;
	ALOGV("camera_stop_preview: OK to stop preview thread.");
	return ret_type;
}

int camera_capture_streamoff(void)
{
	enum v4l2_buf_type type;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
	{
		ALOGE("Fail to VIDIOC_STREAMOFF.");
		return -1;
	}
	return 0;
}

void camera_capture_yuv422to420(uint32_t output_width, uint32_t output_height,  uint32_t *output_addr,
												   uint32_t src_width,uint32_t src_height, uint32_t *input_y_addr,uint32_t * input_uv_addr)
{
	uint32_t out_y_size = output_width*output_height;
	uint32_t src_y_size = src_width*src_height;
	uint8_t *src_y_addr = (uint8_t*)input_y_addr;
	uint8_t *src_uv_addr  = (uint8_t*)input_uv_addr;;
	uint8_t *out_y_addr = (uint8_t*)output_addr;
	uint8_t *out_uv_addr = out_y_addr+out_y_size;
	uint32_t i = 0;
	uint32_t row_num = output_height>>1;

	if((src_width != output_width)||(src_y_addr != out_y_addr))
	{
		memcpy(out_y_addr,src_y_addr,out_y_size);
	}

	for( i=0 ; i<row_num; i++)
	{
		memcpy(out_uv_addr,src_uv_addr,output_width);
		out_uv_addr += output_width;
		src_uv_addr += src_width<<1;
	}
}
void camera_capture_uv422to420(uint32_t output_width, uint32_t output_height,  uint32_t *output_uv_addr,
												   uint32_t src_width,uint32_t src_height,uint32_t * input_uv_addr)
{

	uint8_t *src_uv_addr  = (uint8_t*)input_uv_addr;
	uint8_t *out_uv_addr = (uint8_t*)output_uv_addr;
	uint32_t i = 0;
	uint32_t row_num = output_height>>1;

	ALOGV("SPRD OEM:camera_capture_uv422to420 , output_width,output_height, %d %d, output_uv_addr, input_uv_addr 0x%x 0x%x",
                       output_width, output_height,output_uv_addr,input_uv_addr);

	if(s_camera_info.is_interpolation ||  s_camera_info.is_zoom)
	{
                  ALOGV("SPRD OEM:camera_capture_uv422to420 , end to top ");

		src_uv_addr  = (uint8_t*)input_uv_addr+src_width*(src_height-1);
		out_uv_addr = src_uv_addr;
		for( i=0 ; i<row_num; i++)
		{
			memcpy(out_uv_addr,src_uv_addr,output_width);
			out_uv_addr -= output_width;
			src_uv_addr -= src_width<<1;
		}
	}
	else
	{
		for( i=0 ; i<row_num; i++)
		{
			memcpy(out_uv_addr,src_uv_addr,output_width);
			out_uv_addr += output_width;
			src_uv_addr += src_width<<1;
		}
	}
}
#define DCAMERA_PREVIEW_ZOOM_LEVEL_MAX			    0x08
#define ZOOM_STEP(x) 	                            ((x * 2)/4 / DCAMERA_PREVIEW_ZOOM_LEVEL_MAX)
#define DCAMERA_PIXEL_ALIGNED               4
#define DCAMERA_WIDTH(w)                    ((w)& ~(DCAMERA_PIXEL_ALIGNED - 1))
#define DCAMERA_HEIGHT(h)                   ((h)& ~(DCAMERA_PIXEL_ALIGNED - 1))
void camera_get_zoom_trim(struct zoom_trim_rect *trim_rect,uint32_t zoom_level)
{
	uint32_t zoom_step_w,zoom_step_h;
	uint32_t trim_width=trim_rect->w,trim_height=trim_rect->h;

	if(0 == zoom_level)
		return;
	ALOGV("[SPRD OEM]:camera_get_zoom_trim S,level =%d,x=%d,y=%d,w=%d,h=%d .\n",
		    zoom_level,trim_rect->x,trim_rect->y,trim_rect->w,trim_rect->h);
	zoom_step_w = ZOOM_STEP(trim_width);
	zoom_step_w &= ~1;
	zoom_step_w *= zoom_level;

	zoom_step_h = ZOOM_STEP(trim_height);
	zoom_step_h &= ~1;
	zoom_step_h *= zoom_level;
	trim_width = trim_width-zoom_step_w;
	trim_height = trim_height-zoom_step_h;

	trim_rect->x = ( trim_rect->w-trim_width)/2 ;
	trim_rect->x &= ~1;
	trim_rect->y = (trim_rect->h -trim_height)/2;
	trim_rect->y &= ~1;

	trim_rect->w = DCAMERA_WIDTH(trim_width);
	trim_rect->h = DCAMERA_HEIGHT(trim_height);
	ALOGV("[SPRD OEM]:camera_get_zoom_trim E,x=%d,y=%d,w=%d,h=%d .\n",trim_rect->x,trim_rect->y,trim_rect->w,trim_rect->h);
}
void camera_capture_change_memory(uint32_t output_width, uint32_t output_height,  uint32_t output_addr, uint32_t input_addr)
{
	uint32_t size = output_width*output_height*3/2;
	uint8_t *src_addr = (uint8_t*)input_addr;
	uint8_t *dst_addr = (uint8_t*)output_addr;
	uint32_t row_num = output_height;

	memcpy(dst_addr,src_addr,size);
}

int camera_decode_one_pic(JPEGDEC_INPUT_PARAMS_T  *input_param)
{
	ALOGV("SPRD OEM:camera_decode_one_pic S.");
	long 	pos;
	uint32_t 	jpeg_dec_buf_len;
	int 		fd;
	int             ret =0;
	JPEGDEC_PARAMS_T  jpegdec_params_8810;

	jpegdec_params_8810.format = input_param->format;
	jpegdec_params_8810.width =   input_param->width;
	jpegdec_params_8810.height = input_param->height;
	jpegdec_params_8810.stream_size = input_param->src_size;
	jpegdec_params_8810.src_buf = input_param->src_virt_buf;
	jpegdec_params_8810.src_phy_buf = input_param->src_phy_buf;
         jpegdec_params_8810.target_buf_Y= input_param->target_virt_buf_Y;
	jpegdec_params_8810.target_phy_buf_Y= input_param->target_phy_buf_Y;
	jpegdec_params_8810.target_buf_UV = input_param->target_virt_buf_UV;
	jpegdec_params_8810.target_phy_buf_UV = input_param->target_phy_buf_UV;

	jpeg_dec_buf_len = jpegdec_params_8810.width*jpegdec_params_8810.height;
	jpegdec_params_8810.temp_buf_len = input_param->src_size + 256;

	jpegdec_params_8810.fw_decode_buf = malloc( 20*1024);
	jpegdec_params_8810.fw_decode_buf_size  = 20*1024;

	jpegdec_params_8810.stream_virt_buf[0] = jpegdec_params_8810.src_buf;
	jpegdec_params_8810.stream_phy_buf[0]  = jpegdec_params_8810.src_phy_buf;

	jpegdec_params_8810.stream_virt_buf[1] =  0;
	jpegdec_params_8810.stream_buf_len = jpegdec_params_8810.stream_size;
	jpegdec_params_8810.yuv_virt_buf      = jpegdec_params_8810.target_buf_Y;
	jpegdec_params_8810.yuv_phy_buf    = jpegdec_params_8810.target_phy_buf_Y;
	jpegdec_params_8810.set_slice_height = input_param->slice_height;
	ALOGV("camera_decode_one_pic:set_slice_height=%d.",jpegdec_params_8810.set_slice_height);

	ret = JPEGDEC_decode_one_pic(&jpegdec_params_8810,input_param->write_yuv_callback);
	free(jpegdec_params_8810.fw_decode_buf);
	ALOGV(" SPRD OEM:camera_decode_one_pic E,ret = %d .",ret);
	return ret;
}

void camera_jpegdec_callback(uint32_t src_y_phy_addr, uint32_t src_uv_phy_addr, uint32_t src_height)
{
	ZOOM_TRIM_RECT_T trim_rect;
	uint32_t size = 0;
	uint32_t dst_y_phy_addr = s_camera_info.jpg_out_y_phy_addr+s_camera_info.dcam_out_width*s_camera_info.jpg_record_copy_height;
	uint32_t dst_uv_phy_addr = s_camera_info.jpg_out_uv_phy_addr+s_camera_info.dcam_out_width*s_camera_info.jpg_record_copy_height;
	uint32_t *dst_y_virt_ptr = s_camera_info.jpg_out_y_virt_addr+s_camera_info.dcam_out_width*s_camera_info.jpg_record_copy_height/4;
	uint32_t *dst_uv_virt_ptr = s_camera_info.jpg_out_uv_virt_addr+s_camera_info.dcam_out_width*s_camera_info.jpg_record_copy_height/4;

	ALOGV("SPRD OEM:camera_jpegdec_callback S, dst base, 0x%x 0x%x, dst_phy_addr 0x%x 0x%x",
                     s_camera_info.jpg_out_y_phy_addr,
                     s_camera_info.jpg_out_uv_phy_addr,
                     dst_y_phy_addr,
                    dst_uv_phy_addr );

	src_height = s_camera_info.jpeg_codec_slice_height ? s_camera_info.jpeg_codec_slice_height : 1024;

         if((s_camera_info.jpg_record_copy_height+src_height) >g_dcam_dimensions.picture_height)
         	{
         		src_height = g_dcam_dimensions.picture_height-s_camera_info.jpg_record_copy_height;
		if(0 == src_height)
		{
			ALOGV("camera_jpegdec_callback copy fail!");
			return;
		}
         	}

	trim_rect.x = 0;
	trim_rect.y = 0;
	trim_rect.w = s_camera_info.dcam_out_width;
	trim_rect.h = src_height;
	camera_copy(SCALE_DATA_YUV422,trim_rect.w,trim_rect.h,dst_y_phy_addr,dst_uv_phy_addr,
		                   &trim_rect,s_camera_info.jpg_y_temp_phy_addr,s_camera_info.jpg_uv_temp_phy_addr,SCALE_DATA_YUV422);
//	memcpy(dst_y_virt_ptr,s_camera_info.jpg_y_temp_virt_addr,g_dcam_dimensions.picture_width*src_height);
//         memcpy(dst_uv_virt_ptr,s_camera_info.jpg_uv_temp_virt_addr,g_dcam_dimensions.picture_width*src_height);

	s_camera_info.jpg_record_copy_height += src_height;

//	ALOGV("SPRD OEM:camera_jpegdec_callback copy height=%d.",s_camera_info.jpg_record_copy_height);
}

int camera_process_jpg(void *client_data)
{
	camera_frame_type frame_type;
	JPEGENC_CBrtnType frame;
	struct zoom_trim_rect trim_rect;
	JPEGDEC_INPUT_PARAMS_T input_param;
	int ret = 0;
	uint32_t frame_size = 0;
         uint32_t height = 0;

	 ALOGV("SPRD OEM:camera_process_jpg 0,svirt=0x%x,sphy=0x%x,width=%d,height=%d,tvirt=0x%x,tphy=0x%x.",
	               g_buffers[0].virt_addr,g_buffers[0].phys_addr,g_dcam_dimensions.picture_width,
	               g_dcam_dimensions.picture_height,g_capture_virt_addr,g_capture_phys_addr);


	frame_size = s_camera_info.jpg_out_align_w*s_camera_info.jpg_out_align_h;
	input_param.format = JPEGDEC_YUV_422;
	input_param.width = s_camera_info.dcam_out_width;
	input_param.height = s_camera_info.dcam_out_height;
	input_param.crop_x = 0;
	input_param.crop_y = 0;
	input_param.crop_width = input_param.width;
	input_param.crop_height = input_param.height;
	input_param.src_size = s_camera_info.jpg_len;
	input_param.src_virt_buf = (void*)g_buffers[0].virt_addr;
	input_param.src_phy_buf = g_buffers[0].phys_addr;

	s_camera_info.jpg_record_copy_height = 0;

	if(0 == s_camera_info.jpeg_buf_setting_flag)
	{
		input_param.target_virt_buf_Y = (void*)s_camera_info.jpg_out_y_virt_addr;
		input_param.target_phy_buf_Y = s_camera_info.jpg_out_y_phy_addr;
		input_param.target_virt_buf_UV = (void*)s_camera_info.jpg_out_uv_virt_addr;
		input_param.target_phy_buf_UV = s_camera_info.jpg_out_uv_phy_addr;
		input_param.write_yuv_callback = NULL;
		input_param.slice_height = 0;
	}
	else
	{
		input_param.target_virt_buf_Y = (void*)s_camera_info.jpg_y_temp_virt_addr;
		input_param.target_phy_buf_Y = s_camera_info.jpg_y_temp_phy_addr;
		input_param.target_virt_buf_UV = (void*)s_camera_info.jpg_uv_temp_virt_addr;
		input_param.target_phy_buf_UV = s_camera_info.jpg_uv_temp_phy_addr;
		input_param.write_yuv_callback = camera_jpegdec_callback;
		input_param.slice_height = s_camera_info.jpeg_codec_slice_height;
	}
	ALOGV("SPRD OEM:camera_process_jpg,jpeg input ;y addr:0x%x,uvaddr:0x%x.",input_param.target_phy_buf_Y,input_param.target_phy_buf_UV);
	ALOGV("SPRD OEM:camera_process_jpg,jpeg dec,slice_height = %d.",input_param.slice_height);

	 ret = camera_decode_one_pic(&input_param);
	 if(0 != ret)
	{
		FILE *fp = NULL;
		ALOGV("SPRD OEM:camera_decode_one_pic fail!");

		ALOGE("save jpeg of sensor,s_camera_info.jpg_len=%d!.",s_camera_info.jpg_len);
		fp = fopen("/data/sensor.jpg", "wb");
		fwrite(g_buffers[0].virt_addr, 1, s_camera_info.jpg_len, fp);
		fclose(fp);
		goto TAKE_JPEG_END;
	}

 #if 0
    {
		FILE *fp = NULL;
		ALOGE(" s_camera_info.jpg_out_y_virt_addr,=0x%x. 0x%x",
                                s_camera_info.jpg_out_y_virt_addr, s_camera_info.jpg_out_uv_virt_addr) ;
		fp = fopen("/data/sensor_y.raw", "wb");
		fwrite(s_camera_info.jpg_out_y_virt_addr, 1,frame_size ,fp);
		fclose(fp);
		fp = fopen("/data/sensor_uv.raw", "wb");
		fwrite(s_camera_info.jpg_out_uv_virt_addr, 1, frame_size, fp);
		fclose(fp);
    }

        {
		FILE *fp = NULL;
		ALOGV("SPRD OEM:camera_decode_one_pic fail!");

	//	ALOGE("save jpeg of sensor,s_camera_info.jpg_len=%d!.",s_camera_info.jpg_len);
		fp = fopen("/data/y.raw", "wb");
		fwrite(s_camera_info.jpg_out_y_virt_addr, 1, s_camera_info.dcam_out_width*s_camera_info.dcam_out_height, fp);
		fclose(fp);
                fp = fopen("/data/uv.raw", "wb");
		fwrite(s_camera_info.jpg_out_uv_virt_addr, 1, s_camera_info.dcam_out_width*s_camera_info.dcam_out_height, fp);
		fclose(fp);
	}
#endif
	 ALOGV("SPRD OEM: camera_process_jpg 1, 0x%x 0x%x",
                       s_camera_info.jpg_out_y_phy_addr,
                       s_camera_info.jpg_out_uv_phy_addr);



        if(s_camera_info.is_zoom || s_camera_info.is_interpolation)
        {
            s_camera_info.out_format = DCAM_DATA_YUV422;
            height = s_camera_info.dcam_out_height;
        }
        else
        {
            camera_capture_uv422to420(s_camera_info.jpg_out_align_w,s_camera_info.jpg_out_align_h,s_camera_info.jpg_out_uv_virt_addr,
            	 					   s_camera_info.jpg_out_align_w,s_camera_info.jpg_out_align_h,s_camera_info.jpg_out_uv_virt_addr);
            s_camera_info.out_format = DCAM_DATA_YUV420;
            height = s_camera_info.dcam_out_height/2;
        }
#if !CAM_OUT_YUV420_UV
    	// convrt UV to VU
        {
        	uint8_t *dst = (uint8_t *)s_camera_info.jpg_out_uv_virt_addr;
        	uint8_t *src = dst;
        	for(int i = 0; i < height; i++){
        		for(int j = 0; j < s_camera_info.dcam_out_width/ 2; j++){
        			uint8_t tmp = *src++;
        			*dst++ = *src++;
        			*dst++ = tmp;
        		}
        	}
        	dst = NULL;
        	src = NULL;
        }
#endif
        g_buffers[0].phys_addr = s_camera_info.jpg_out_y_phy_addr;
        g_buffers[0].virt_addr = s_camera_info.jpg_out_y_virt_addr;
        g_buffers[0].u_phys_addr = s_camera_info.jpg_out_uv_phy_addr;
        g_buffers[0].u_virt_addr = s_camera_info.jpg_out_uv_virt_addr;

	ALOGV("SPRD OEM:camera_process_jpg end., g_buffers[0]. 0x%x 0x%x ", g_buffers[0].phys_addr, g_buffers[0].u_phys_addr);

	return 0;
TAKE_JPEG_END:
	//g_callback(CAMERA_EXIT_CB_FAILED, client_data, CAMERA_FUNC_TAKE_PICTURE, NULL);
	return -1;
}
camera_ret_code_type camera_stop_capture(void)
{
    camera_ret_code_type ret_type = CAMERA_SUCCESS;
    uint32_t num = 0;
    ALOGV("camera_stop_capture: Start to stop capture thread.");

    g_stop_capture_flag = 1;

    while(!g_capture_stop)
    {
        usleep(100000);
        num++;
        ALOGV("waiting the capture stop.... g_stop_capture_flag=%d, g_capture_stop=%d",g_stop_capture_flag, g_capture_stop);
        if(num >= 100) {
            pthread_kill(g_capture_thr, 9);
            ALOGE("Force stop the capture!");
            break;
        }
    } //wait capture thread stop
    num = 0;
    while(!g_encoder_is_end)
    {
        usleep(100000);
        num++;
        ALOGV("waiting the capture encode stop.... g_encoder_is_end=%d,",g_encoder_is_end);
        if(num >= 100) {
            pthread_kill(g_encoder_thr, 9);
            ALOGE("Force stop the capture!");
            break;
        }
    } //wait capture thread stop

    g_callback(CAMERA_RSP_CB_SUCCESS, g_dcam_obj, CAMERA_FUNC_RELEASE_PICTURE, 0);

    g_stop_capture_flag = 0;
    ALOGV("camera_stop_capture: OK to stop capture thread.");

    return ret_type;
}
uint32_t get_capture_stop_flag()
{
    ALOGV("get_capture_stop_flag: %d", g_stop_capture_flag);

    return g_stop_capture_flag;
}

uint32_t check_capture_stop(void)
{
    ALOGV("check_capture_stop: %d", g_capture_stop);
    if(1 == get_capture_stop_flag()) {
        g_capture_stop = 1;
        return 1;
    }
    return 0;
}
void *camera_capture_thread(void *client_data)
{
	struct v4l2_buffer buf;
	uint32_t i, ret = 0;
	camera_frame_type frame_type;
	struct zoom_trim_rect trim_rect;
	uint32_t jpg_len=0;
	uint32_t size;
	uint32_t picture_width = g_dcam_dimensions.picture_width;
	uint32_t picture_height= g_dcam_dimensions.picture_height;

	ALOGV("OK to camera_capture_thread.");
	CLEAR (buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_USERPTR;

	s_camera_info.out_format = DCAM_DATA_YUV420;

        if (0 != xioctl(fd, VIDIOC_DQBUF, &buf))
	{
		ALOGE("SPRD OEM:Fail to VIDIOC_DQBUF.");
		return NULL;
	}
	else
	{
		ALOGV("SPRD OEM:capture OK to VIDIOC_DQBUF.");
		if(0 != camera_capture_streamoff())
		{
			ALOGE("SPRD OEM:Fail to stream off for capture.");
			return NULL;
		}
                if (1 == check_capture_stop())
                {
                    ALOGV("capture stop exit 2");
                    return NULL;
                }
		g_callback(CAMERA_RSP_CB_SUCCESS, client_data, CAMERA_FUNC_TAKE_PICTURE, 0);
		ALOGV("SPRD OEM:OK to capture VIDIOC_DQBUF.buf.index: %d. userptr: %lx", buf.index, buf.m.userptr);
	}

	if(SENSOR_IMAGE_FORMAT_JPEG ==s_camera_info.sensor_out_format)
	{
		if (0 != xioctl(fd, VIDIOC_G_OUTPUT, &jpg_len))
		{
			ALOGE("SPRD OEM:camera_capture_thread,Fail to VIDIOC_G_OUTPUT.");
		}
		s_camera_info.jpg_len = jpg_len;
		ALOGV("SPRD OEM:camera_capture_thread,jpg_len=%d .\n",jpg_len);
	         ret = camera_process_jpg(client_data);
		if(0 != ret)
		{
			ALOGV("PRD OEM:camera_process_jpg fail.");
			goto TAKE_PIC_FAIL;
		}
	}

	g_releasebuff_index = 0;

	frame_type.buf_Virt_Addr = g_capture_virt_addr;
         frame_type.buffer_phy_addr = g_capture_phys_addr;
         if((90 == g_rotation)||(270 == g_rotation))
	{
		picture_width = g_dcam_dimensions.picture_height;
		picture_height= g_dcam_dimensions.picture_width;
	}
	if(s_camera_info.is_zoom || s_camera_info.is_interpolation)
	{
		ALOGV("SPRD OEM:camera_capture_thread,dcam out:w=%d,h=%d.picture size %d %d",
                                s_camera_info.dcam_out_width,
                                s_camera_info.dcam_out_height,
                                g_dcam_dimensions.picture_width,
                                g_dcam_dimensions.picture_height);

		trim_rect.x = 0;
		trim_rect.y = 0;
		trim_rect.w = s_camera_info.dcam_out_width;
		trim_rect.h = s_camera_info.dcam_out_height;
		camera_get_zoom_trim(&trim_rect,g_hal_zoom_level);
		ALOGV("SPRD OEM:camera_capture_thread,yuv with zoom,trim:x=%d,y=%d,w=%d,h=%d. dst %d %D" ,
                                trim_rect.x,trim_rect.y,trim_rect.w,trim_rect.h,
                                g_dcam_dimensions.picture_width,g_dcam_dimensions.picture_height);

                   if((g_hal_zoom_level >= 4))//&&(SENSOR_IMAGE_FORMAT_JPEG ==s_camera_info.sensor_out_format))
                    {
                        uint32_t  y_addr = (uint32_t)g_buffers[0].virt_addr;
                        uint32_t  u_addr = (uint32_t)g_buffers[0].u_virt_addr;
                        uint32_t     y_offset = 0, u_offset = 0;
                        ret = camera_yuv_rearrange(s_camera_info.out_format  == DCAM_DATA_YUV422,
                                                                               y_addr,u_addr,
                                                                               s_camera_info.dcam_out_width,s_camera_info.dcam_out_height,&trim_rect,
                                                                               &y_offset,&u_offset);
                        if(0 != ret)
                        {
                            ALOGV("SPRD OEM:camera_crop_interpolation interpolation error!");
                            goto TAKE_PIC_FAIL;
                        }
                        g_buffers[0].virt_addr = (uint32_t*)(y_addr + y_offset) ;
                        g_buffers[0].u_virt_addr = (uint32_t*)(u_addr +u_offset); ;
                        g_buffers[0].phys_addr = g_buffers[0].phys_addr + y_offset;
                        g_buffers[0].u_phys_addr = g_buffers[0].u_phys_addr + u_offset;
                        s_camera_info.dcam_out_width = trim_rect.w;
                        s_camera_info.dcam_out_height = trim_rect.h;
                        trim_rect.x = 0;
                        trim_rect.y = 0;
                        trim_rect.w = s_camera_info.dcam_out_width;
                        trim_rect.h = s_camera_info.dcam_out_height;
                        ALOGV("SPRD OEM:after camera_yuv_rearrange, phy 0x%x 0x%x, vir 0x%x 0x%x, s_camera_info.out_format  %d, width height %d %d ",
                                    g_buffers[0].phys_addr,
                                    g_buffers[0].u_phys_addr,
                                    g_buffers[0].virt_addr,
                                    g_buffers[0].u_virt_addr,
                                    s_camera_info.out_format,
                                    s_camera_info.dcam_out_width,
                                    s_camera_info.dcam_out_height);

 #if 0
    {
		FILE *fp = NULL;
		ALOGE(" g_buffers[0].virt_addr,=0x%x.  g_buffers[0].u_virt_addr 0x%x, %d %d ",
                                g_buffers[0].virt_addr, g_buffers[0].u_virt_addr,
                                s_camera_info.dcam_out_width,
                                s_camera_info.dcam_out_height);
		fp = fopen("/data/sensor_yy.raw", "wb");
		fwrite(g_buffers[0].virt_addr, 1, s_camera_info.dcam_out_width*s_camera_info.dcam_out_height, fp);
                   fclose(fp);
		fp = fopen("/data/sensor_uv_uv.raw", "wb");
		fwrite( g_buffers[0].u_virt_addr, 1, s_camera_info.dcam_out_width*s_camera_info.dcam_out_height, fp);
		fclose(fp);
   }
#endif

                    }
		ret = camera_crop_interpolation(SCALE_DATA_YUV422,
                                                                     picture_width,//            g_dcam_dimensions.picture_width,
                                                                         picture_height,//        g_dcam_dimensions.picture_height,
								      s_camera_info.temp_y_phy_addr,
								      s_camera_info.temp_uv_phy_addr,
								      &trim_rect,
								      s_camera_info.dcam_out_width,
								      s_camera_info.dcam_out_height,
								      g_buffers[0].phys_addr,
								      g_buffers[0].u_phys_addr,
								      s_camera_info.out_format  == DCAM_DATA_YUV422 ? SCALE_DATA_YUV422 : SCALE_DATA_YUV420);
		if(0 != ret)
		{
			ALOGV("SPRD OEM:camera_crop_interpolation interpolation error!");
			goto TAKE_PIC_FAIL;
		}

 #if 0

		FILE *fp = NULL;
		ALOGE(" s_camera_info.temp_y_phy_addr,=0x%x.", s_camera_info.temp_y_phy_addr);
		fp = fopen("/data/sensor_y.raw", "wb");
		fwrite(s_camera_info.temp_y_virt_addr, 1, g_dcam_dimensions.picture_width*g_dcam_dimensions.picture_height, fp);
		fp = fopen("/data/sensor_uv.raw", "wb");
		fwrite(s_camera_info.temp_uv_virt_addr, 1, g_dcam_dimensions.picture_width*g_dcam_dimensions.picture_height, fp);
		fclose(fp);
#endif
		camera_capture_yuv422to420(picture_width,
                                   picture_height,
                                                                             g_capture_virt_addr,
								  picture_width,// g_dcam_dimensions.picture_width,
								  picture_height,// g_dcam_dimensions.picture_height,
								   s_camera_info.temp_y_virt_addr,
								   s_camera_info.temp_uv_virt_addr);
		g_buffers[g_releasebuff_index].virt_addr =g_capture_virt_addr ;
		g_buffers[g_releasebuff_index].phys_addr = g_capture_phys_addr;
		s_camera_info.dcam_out_width = picture_width;
		s_camera_info.dcam_out_height = picture_height;
	}


          if(1==s_camera_info.is_need_rotation)
	{
		uint32_t size = s_camera_info.dcam_out_width * s_camera_info.dcam_out_width* 2;
		uint32_t phys_addr;
		uint32_t *virt_addr;
		if(NULL == (virt_addr = (uint32_t *)g_dcam_obj->get_temp_mem_by_HW(size, 1, &phys_addr)))
		{
			ALOGE("SPRD OEM:Fail to malloc capture temp memory, size: 0x%x.", size);
			goto TAKE_PIC_FAIL;
		}
		camera_capture_change_memory(s_camera_info.dcam_out_width,s_camera_info.dcam_out_height, (uint32_t)virt_addr, (uint32_t)g_buffers[g_releasebuff_index].virt_addr);
		ret = camera_rotation(g_rotation, s_camera_info.dcam_out_width, s_camera_info.dcam_out_height, phys_addr, g_buffers[g_releasebuff_index].phys_addr);
		if(0 != ret)
		{
			g_dcam_obj->free_temp_mem_by_HW();
			ALOGE("SPRD OEM:Fail to take picture because the camera_rotation.");
			goto TAKE_PIC_FAIL;
		}
		ALOGV("SPRD OEM:camera_capture_thread,rotation: w: %d, h: %d.", g_dcam_dimensions.picture_width, g_dcam_dimensions.picture_height);
		g_dcam_obj->free_temp_mem_by_HW();
		g_rotation = 0;
		g_cam_params.orientation = 0;
		frame_type.buf_Virt_Addr = g_buffers[g_releasebuff_index].virt_addr;
		frame_type.buffer_phy_addr = g_buffers[g_releasebuff_index].phys_addr;
	}

	ALOGV("SPRD OEM:after convert_format.");
	frame_type.dx = g_dcam_dimensions.picture_width;
	frame_type.dy = g_dcam_dimensions.picture_height;
	frame_type.captured_dx = g_dcam_dimensions.picture_width;
	frame_type.captured_dy = g_dcam_dimensions.picture_height;
	frame_type.format = CAMERA_YCBCR_4_2_0;
	frame_type.rotation = 0;
	frame_type.header_size = 0;
	frame_type.Y_Addr = (uint8_t *)frame_type.buf_Virt_Addr;
	frame_type.CbCr_Addr = frame_type.Y_Addr + frame_type.dx * frame_type.dy;
        if (1 == check_capture_stop())
        {
            ALOGV("capture stop exit 3");
            return NULL;
        }
	g_callback(CAMERA_EVT_CB_SNAPSHOT_DONE, client_data, CAMERA_FUNC_TAKE_PICTURE, (uint32_t)&frame_type);
	ALOGV("SPRD OEM:after CAMERA_EVT_CB_SNAPSHOT_DONE.");
        if (1 == check_capture_stop())
        {
            ALOGV("capture stop exit 4");
            return NULL;
        }
	g_callback(CAMERA_EXIT_CB_DONE, client_data, CAMERA_FUNC_TAKE_PICTURE, (uint32_t)&frame_type);
        g_capture_stop = 1;
	return NULL;

TAKE_PIC_FAIL:
        g_capture_stop = 1;
	g_callback(CAMERA_EXIT_CB_FAILED, client_data, CAMERA_FUNC_TAKE_PICTURE, NULL);
	return NULL;
}

int camera_capture_mem_alloc(uint32_t dcam_out_width,uint32_t dcam_out_height)
{
    uint32_t phys_addr = 0;
    uint32_t *virt_addr = 0;
    uint32_t offset = 0;
    uint32_t frame_size = 0;
    uint32_t buffer_size = 0;
    uint32_t jpeg_y_frame_size = 0;
    uint32_t mem_offset = 0;
    uint32_t slice_temp_size = 0;
    uint32_t mem_size = s_camera_info.cap_mem_size;
    uint32_t    jpg_cap_size = 0;
    uint32_t    page_size = 0;
    int32_t ret = 0;

    s_camera_info.jpeg_buf_setting_flag = 0;
    s_camera_info.jpeg_codec_slice_height = 0;

    ALOGV("SPRD OEM camera_capture_mem_alloc: mem_size = %d,mem_base ,phy 0x%x, vir 0x%x ",
		mem_size,
                g_capture_phys_addr,
                g_capture_virt_addr);

    page_size = getpagesize();

    if(SENSOR_IMAGE_FORMAT_JPEG == s_camera_info.sensor_out_format)
    {
        jpg_cap_size =   s_camera_info.jpg_out_align_w*s_camera_info.jpg_out_align_h/JPEG_DIV;
        jpg_cap_size = camera_get_size_align_page(jpg_cap_size);
        ALOGV("SPRD OEM:camera_capture_mem_alloc,jpg capture width height {%d %d}, jpg buffer size  %d",
                    s_camera_info.jpg_out_align_w,s_camera_info.jpg_out_align_h,jpg_cap_size);

    }

    if(g_dcam_dimensions.picture_width*g_dcam_dimensions.picture_height >= JPEGDECENC_Y_UV_OFFSET_MAX ||
       ((mem_size>>1) >= JPEGDECENC_Y_UV_OFFSET_MAX && (s_camera_info.is_interpolation ||  s_camera_info.is_zoom)&&(SENSOR_IMAGE_FORMAT_JPEG == s_camera_info.sensor_out_format) ))
    {
        frame_size = g_dcam_dimensions.picture_width *g_dcam_dimensions.picture_height * 2;
        buffer_size = camera_get_size_align_page(frame_size);
        s_camera_info.jpeg_buf_setting_flag = 1;
        s_camera_info.jpeg_codec_slice_height = 128;
        mem_offset = buffer_size ;
        frame_size = g_dcam_dimensions.picture_width*s_camera_info.jpeg_codec_slice_height;

        if((mem_offset + (frame_size << 1)+jpg_cap_size ) >  mem_size)
        {
            if(NULL == (virt_addr = (uint32_t *)g_dcam_obj->get_temp_mem_by_jpegslice((frame_size<<1) + jpg_cap_size, 1, &phys_addr)))
            {
                ALOGV("SPRD OEM: Error,No mem for  jpg tmp buffer");
                ret = -1;
            }
            s_camera_info.jpg_y_temp_phy_addr = phys_addr;
            s_camera_info.jpg_y_temp_virt_addr = virt_addr;
            mem_offset = g_dcam_dimensions.picture_width*s_camera_info.jpeg_codec_slice_height;
            s_camera_info.jpg_uv_temp_phy_addr = s_camera_info.jpg_y_temp_phy_addr+frame_size;
            s_camera_info.jpg_uv_temp_virt_addr = s_camera_info.jpg_y_temp_virt_addr+frame_size/4;
            s_camera_info.jpg_enc_y_temp_phy_addr = s_camera_info.jpg_y_temp_phy_addr;
            s_camera_info.jpg_enc_y_temp_virt_addr = s_camera_info.jpg_y_temp_virt_addr;
            s_camera_info.jpg_enc_uv_temp_phy_addr = s_camera_info.jpg_uv_temp_phy_addr;
            s_camera_info.jpg_enc_uv_temp_virt_addr = s_camera_info.jpg_uv_temp_virt_addr;

            if(jpg_cap_size)
            {
                phys_addr = phys_addr + (frame_size<<1);
                virt_addr = virt_addr +  (frame_size>>1);
                ALOGV("SPRD OEM:camera_capture_mem_alloc,jpeg capture buffer, phy:0x%x, virt:0x%x",
                phys_addr, virt_addr);

                phys_addr  =  phys_addr &  ~(page_size - 1);
                virt_addr = (uint32_t*)(( (uint32_t)virt_addr)&~(page_size - 1));
            }
            slice_temp_size = 0;
        }
        else
        {
            if(jpg_cap_size)
            {
                mem_offset = mem_size - jpg_cap_size;
                phys_addr = g_capture_phys_addr+mem_offset;
                virt_addr =g_capture_virt_addr+mem_offset/4;
            }
            mem_offset = frame_size*2;
            mem_offset = mem_size - mem_offset-jpg_cap_size;
            s_camera_info.jpg_y_temp_phy_addr = g_capture_phys_addr+mem_offset;
            s_camera_info.jpg_y_temp_virt_addr = g_capture_virt_addr+mem_offset/4;
            mem_offset = g_dcam_dimensions.picture_width*s_camera_info.jpeg_codec_slice_height;
            s_camera_info.jpg_uv_temp_phy_addr = s_camera_info.jpg_y_temp_phy_addr+mem_offset;
            s_camera_info.jpg_uv_temp_virt_addr = s_camera_info.jpg_y_temp_virt_addr+mem_offset/4;
            s_camera_info.jpg_enc_y_temp_phy_addr = s_camera_info.jpg_y_temp_phy_addr;
            s_camera_info.jpg_enc_y_temp_virt_addr = s_camera_info.jpg_y_temp_virt_addr;
            s_camera_info.jpg_enc_uv_temp_phy_addr = s_camera_info.jpg_uv_temp_phy_addr;
            s_camera_info.jpg_enc_uv_temp_virt_addr = s_camera_info.jpg_uv_temp_virt_addr;
            slice_temp_size = frame_size*2+jpg_cap_size;
        }

        ALOGV("SPRD OEM:camera_capture_mem_alloc,jpeg codec temp buffer, phy:0x%x,0x%x,virt:0x%x,0x%x.",
                        s_camera_info.jpg_y_temp_phy_addr,s_camera_info.jpg_uv_temp_phy_addr,
                        s_camera_info.jpg_y_temp_virt_addr,s_camera_info.jpg_uv_temp_virt_addr);
    }
    else
    {
        if(SENSOR_IMAGE_FORMAT_JPEG == s_camera_info.sensor_out_format)
        {
            frame_size = g_dcam_dimensions.picture_width *g_dcam_dimensions.picture_height * 2;
            buffer_size = camera_get_size_align_page(frame_size);
            mem_offset = buffer_size ;
            if((mem_offset + jpg_cap_size ) >  mem_size)
            {
                if(NULL == (virt_addr = (uint32_t *)g_dcam_obj->get_temp_mem_by_jpegslice(jpg_cap_size, 1, &phys_addr)))
                {
                    ALOGV("SPRD OEM: Error,No mem for  jpg tmp buffer");
                    ret = -1;
                }
                slice_temp_size = 0;
            }
            else
            {
                mem_offset = mem_size - jpg_cap_size;
                phys_addr = g_capture_phys_addr+mem_offset;
                virt_addr =g_capture_virt_addr+mem_offset/4;
                slice_temp_size = jpg_cap_size;
            }

        }
    }
    /*if slice encode needed, alloc jpg_enc_y_temp_phy_addr*/

    mem_size -= slice_temp_size;

    if(SENSOR_IMAGE_FORMAT_JPEG != s_camera_info.sensor_out_format)
    {
        frame_size = g_dcam_dimensions.picture_width *g_dcam_dimensions.picture_height * 2;
        buffer_size = camera_get_size_align_page(frame_size);
        if(s_camera_info.is_interpolation ||  s_camera_info.is_zoom)
        {
            // whatever it needs interpolation or zoom,  the capture YUV buffer and the destination buffer
            // to save the scaling up YUV should be alloced
           //    offset = (mem_size-buffer_size)/2;
            g_buffers[0].length = mem_size;
            frame_size = (uint32_t)(dcam_out_width * dcam_out_height);
            g_buffers[0].virt_addr = g_capture_virt_addr + (mem_size/2 -frame_size )/4;
            g_buffers[0].phys_addr = g_capture_phys_addr+mem_size/2 -frame_size ;
            g_buffers[0].u_virt_addr = g_capture_virt_addr+(mem_size  - frame_size/2)/4;
            g_buffers[0].u_phys_addr = g_capture_phys_addr + mem_size  - frame_size/2;

            g_buffers[0].v_virt_addr = 0;
            g_buffers[0].v_phys_addr = 0;
            ALOGV("SPRD OEM camera_capture_mem_alloc: YUV with zoom,g_buffer[%d]: virt: 0x%x 0x%x, phys: 0x%x 0x%x, len: %x.",
                        0,(uint32_t)g_buffers[0].virt_addr,g_buffers[0].u_virt_addr,g_buffers[0].phys_addr,g_buffers[0].u_phys_addr ,
                        g_buffers[0].length);
            /*first capture buffer*/

            s_camera_info.temp_y_phy_addr = g_capture_phys_addr;
            s_camera_info.temp_y_virt_addr = g_capture_virt_addr;
            s_camera_info.temp_uv_phy_addr = g_capture_phys_addr+mem_size/2;
            s_camera_info.temp_uv_virt_addr = g_capture_virt_addr+mem_size/8;
            /*then scaling up buffer*/
        }
        else
        {
               //no interpolation or zoom, just alloc capture buffer;
               g_buffers[0].length = buffer_size;
                g_buffers[0].virt_addr = g_capture_virt_addr;
                g_buffers[0].phys_addr = g_capture_phys_addr;
                g_buffers[0].u_virt_addr = g_capture_virt_addr + (frame_size >> 3) ;
                g_buffers[0].u_phys_addr = g_capture_phys_addr + (frame_size >> 1) ;
                g_buffers[0].v_virt_addr = 0;
                g_buffers[0].v_phys_addr = 0;
                ALOGV("SPRD OEM camera_capture_mem_alloc: g_buffer[%d]: virt: %x, phys: %x, len: %x.",
                    0,(uint32_t)g_buffers[0].virt_addr,g_buffers[0].phys_addr,
                    g_buffers[0].length);

		if( (frame_size >> 1)  < JPEGDECENC_Y_UV_OFFSET_MAX)
		{
			   s_camera_info.jpeg_buf_setting_flag = 0;
		}
        }

    }
    else
    {
        mem_offset = mem_size;
        jpeg_y_frame_size = s_camera_info.jpg_out_align_w*s_camera_info.jpg_out_align_h;

        if(0 == phys_addr)
        {
            if(NULL == (virt_addr = (uint32_t *)g_dcam_obj->get_temp_mem_by_jpegslice(jpg_cap_size , 1, &phys_addr)))
            {
                ALOGV("SPRD OEM: Error,No mem for  jpg tmp buffer");
                ret = -1;
            }
            ALOGV("SPRD OEM:camera_capture_mem_alloc,jpeg capture buffer, phy:0x%x virt:0x%x",
                        phys_addr,virt_addr);

        }
        g_buffers[0].virt_addr =virt_addr;
        g_buffers[0].phys_addr = phys_addr;
        g_buffers[0].u_virt_addr =0;
        g_buffers[0].u_phys_addr = 0;
        g_buffers[0].v_virt_addr = 0;
        g_buffers[0].v_phys_addr = 0;
        g_buffers[0].length = jpg_cap_size;
        /*the captured JPG bitstream will be saved in g_buffers[0] */

        if(s_camera_info.is_interpolation ||  s_camera_info.is_zoom)
        {
            mem_offset = mem_size/2-jpeg_y_frame_size;
            s_camera_info.jpg_out_y_phy_addr = g_capture_phys_addr+mem_offset;
            s_camera_info.jpg_out_y_virt_addr = g_capture_virt_addr+mem_offset/4;
            mem_offset = mem_size-jpeg_y_frame_size;
            s_camera_info.jpg_out_uv_phy_addr =   g_capture_phys_addr+mem_offset;
            s_camera_info.jpg_out_uv_virt_addr =  g_capture_virt_addr+mem_offset/4;
            s_camera_info.alloc_jpg_mem_size = mem_size;
	   s_camera_info.temp_y_phy_addr = g_capture_phys_addr;
            s_camera_info.temp_y_virt_addr = g_capture_virt_addr;
            s_camera_info.temp_uv_phy_addr = g_capture_phys_addr+(mem_size>>1);
            s_camera_info.temp_uv_virt_addr = g_capture_virt_addr+(mem_size>>3);
            /*then scaling up buffer*/
	   ALOGV("SPRD OEM:jpg out phy addr:y_addr=0x%x,uv_addr=0x%x.",s_camera_info.jpg_out_y_phy_addr,s_camera_info.jpg_out_uv_phy_addr);
  	   ALOGV("SPRD OEM:tem phy:y_addr=0x%x,uv_addr=0x%x.",s_camera_info.temp_y_phy_addr,s_camera_info.temp_uv_phy_addr);
        }
        else
        {
            mem_offset = s_camera_info.jpg_out_align_w*s_camera_info.jpg_out_align_h;
            s_camera_info.jpg_out_y_phy_addr = g_capture_phys_addr;
            s_camera_info.jpg_out_y_virt_addr = g_capture_virt_addr;
            s_camera_info.jpg_out_uv_phy_addr =  s_camera_info.jpg_out_y_phy_addr+mem_offset;
            s_camera_info.jpg_out_uv_virt_addr =  s_camera_info.jpg_out_y_virt_addr+mem_offset/4;
        }

        /*in jpeg decode , jpg_out_y_phy_addr is to save the uncompressed YUV data*/

        ALOGV("SPRD OEM camera_capture_mem_alloc: jpg target 0x%x 0x%x, jpg decode phy  0x%x 0x%x, vir 0x%x 0x%x ",
                   g_buffers[0].phys_addr ,
                   g_buffers[0].virt_addr,
                   s_camera_info.jpg_out_y_phy_addr,
                   s_camera_info.jpg_out_uv_phy_addr,
                   s_camera_info.jpg_out_y_virt_addr,
                   s_camera_info.jpg_out_uv_virt_addr);

    }

    return ret;
}

int camera_capture_init(uint32_t mem_size,int32_t capture_fmat)
{
	struct v4l2_format fmt;
	uint32_t min;
	struct v4l2_requestbuffers req;
	uint32_t buffer_size, page_size, frame_size;
	struct v4l2_streamparm streamparm;
	uint32_t ret = 0;

	s_camera_info.rot_angle = 0;
	s_camera_info.is_interpolation = 0;
	s_camera_info.is_need_rotation = 0;
	s_camera_info.dcam_out_width = 0;
	s_camera_info.dcam_out_height = 0;
	s_camera_info.is_zoom = 0;

	CLEAR (fmt);
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;    
         camera_set_rot_angle(&g_cam_params.orientation_parm,&g_rotation_parm);
	if(1 == g_cam_params.orientation_parm)
	{
		ALOGV("### camera_capture_init, g_rotation = %d, g_rotation_parm = %d \n", g_rotation, g_rotation_parm);

		fmt.fmt.raw_data[199] =1;

		if( 90 == g_rotation_parm)
		{
			fmt.fmt.pix.width = g_dcam_dimensions.picture_height;
		        fmt.fmt.pix.height = g_dcam_dimensions.picture_width;
			fmt.fmt.raw_data[198] = 1;
		}
		else if(270 == g_rotation_parm)
		{
			fmt.fmt.pix.width = g_dcam_dimensions.picture_height;
		        fmt.fmt.pix.height = g_dcam_dimensions.picture_width;
			fmt.fmt.raw_data[198] = 2;
		}
		else if (180 == g_rotation_parm)
		{
			fmt.fmt.raw_data[198] = 3;
			fmt.fmt.pix.width = g_dcam_dimensions.picture_width;
		        fmt.fmt.pix.height = g_dcam_dimensions.picture_height;
		}
	}
	else
	{
                /* 0 */
		fmt.fmt.pix.width = g_dcam_dimensions.picture_width;
	        fmt.fmt.pix.height = g_dcam_dimensions.picture_height;
	}

	if(SENSOR_IMAGE_FORMAT_JPEG != s_camera_info.sensor_out_format)
	{
	/*
		if(g_dcam_dimensions.picture_width <=SCALE_OUT_WIDTH_MAX)
		{
			fmt.fmt.raw_data[197] = (uint8_t)(g_hal_zoom_level&0xff);
		}
		else
		{
			if(g_hal_zoom_level)
			{
				s_camera_info.is_zoom = 1;
			}
		}
            */
            capture_fmat = 1;

	}
        else
        {

            capture_fmat = 3;
        }

        if(g_hal_zoom_level)
        {
            s_camera_info.is_zoom = 1;
        }
/*
	if(s_camera_info.is_zoom == 1)
	{
		capture_fmat = 0;
	}
*/
	ALOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");
	ALOGV("=====will set fmt to (%d, %d)--", fmt.fmt.pix.width,fmt.fmt.pix.height);

	switch(capture_fmat)
	{
		case 0:
			fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV422P;
			ALOGV("V4L2_PIX_FMT_YUV422P\n");
			break;
		case 2:
			fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
			ALOGV("V4L2_PIX_FMT_RGB565\n");
			break;
		case 3:
			fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_JPEG;
			ALOGV("V4L2_PIX_FMT_JPEG\n");
			break;
		case 1:
		default:
			fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
			ALOGV("V4L2_PIX_FMT_YUV420\n");
			break;

	}
	fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;


    	if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
	{
		ALOGE("Fail to VIDIOC_S_FMT.");
		return -1;
	}
         if((1 == g_cam_params.orientation_parm)&&((90 == g_rotation_parm)||(270 == g_rotation_parm)))
         {
                    if((fmt.fmt.pix.width<g_dcam_dimensions.picture_height) || (fmt.fmt.pix.height<g_dcam_dimensions.picture_width))
                    {
                                ALOGV(" camera_capture_init:need interpolation,pic size:%d,%d\n",
                                g_dcam_dimensions.picture_width,g_dcam_dimensions.picture_height);
                                s_camera_info.is_interpolation = 1;
                    }
        }
        else
        {
	if((fmt.fmt.pix.width<g_dcam_dimensions.picture_width) || (fmt.fmt.pix.height<g_dcam_dimensions.picture_height))
	{
		ALOGV(" camera_capture_init:need interpolation,pic size:%d,%d\n",
		g_dcam_dimensions.picture_width,g_dcam_dimensions.picture_height);
		s_camera_info.is_interpolation = 1;
	}
         }
	ALOGV("=====after set fmt\n");
	ALOGV("    fmt.fmt.pix.width = %d\n", fmt.fmt.pix.width);
	ALOGV("    fmt.fmt.pix.height = %d\n", fmt.fmt.pix.height);
	ALOGV("    fmt.fmt.pix.sizeimage = %d\n", fmt.fmt.pix.sizeimage);
	g_capture_size = fmt.fmt.pix.sizeimage;
	ALOGV("    fmt.fmt.pix.bytesperline = %d\n", fmt.fmt.pix.bytesperline);
	ALOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");
	ALOGV("\n");

	g_capture_size = fmt.fmt.pix.sizeimage;
    /* Note VIDIOC_S_FMT may change width and height. */
   	 ALOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");
    /* Buggy driver paranoia. */
    	min = fmt.fmt.pix.width * 2;
    	if (fmt.fmt.pix.bytesperline < min)
        		fmt.fmt.pix.bytesperline = min;
    	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    	if (fmt.fmt.pix.sizeimage < min)
        		fmt.fmt.pix.sizeimage = min;
	ALOGV("After Buggy driver paranoia\n");
	ALOGV("    >>fmt.fmt.pix.sizeimage = %d\n", fmt.fmt.pix.sizeimage);
	ALOGV("    >>fmt.fmt.pix.bytesperline = %d\n", fmt.fmt.pix.bytesperline);
	ALOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");
	ALOGV("\n");
	CLEAR (req);
	req.count = g_capture_buffer_num;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    	req.memory = V4L2_MEMORY_USERPTR;
    	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
	{
		ALOGE("Fail to VIDIOC_REQBUFS.");
		return -1;
	}
	s_camera_info.dcam_out_width = fmt.fmt.pix.width;
	s_camera_info.dcam_out_height = fmt.fmt.pix.height;
	s_camera_info.cap_mem_size = mem_size;

	ALOGV("camera_capture_init:dcam output,w=%d,h=%d .\n",s_camera_info.dcam_out_width,s_camera_info.dcam_out_height);

	g_buffers = (buffer *)calloc(g_capture_buffer_num, sizeof(*g_buffers));
	if (!g_buffers)
	{
		ALOGE("Fail to malloc capture buffer struct.");
		return -1;
	}

	if(SENSOR_IMAGE_FORMAT_JPEG == s_camera_info.sensor_out_format)
	{
		s_camera_info.jpg_out_align_w = JPEG_OUT_ALIGN_W(fmt.fmt.pix.width);
		s_camera_info.jpg_out_align_h = JPEG_OUT_ALIGN_H(fmt.fmt.pix.height);
	}

	if((90==g_rotation_parm) || (270==g_rotation_parm))
	{
		if((s_camera_info.dcam_out_width<  g_dcam_dimensions.picture_height)
		||(s_camera_info.dcam_out_height<g_dcam_dimensions.picture_width))
		{
			s_camera_info.is_interpolation = 1;
		}
		else
        {
			s_camera_info.is_interpolation = 0;
		}
	}
	else
	{
	if((s_camera_info.dcam_out_width<  g_dcam_dimensions.picture_width)
		||(s_camera_info.dcam_out_height<g_dcam_dimensions.picture_height))
	{
		s_camera_info.is_interpolation = 1;
		ALOGV("SPRD OEM:camera_capture_init,nedd interpolation,dcam out:%d,%d",
			    s_camera_info.dcam_out_width,s_camera_info.dcam_out_height);
	}
	}
	ret = camera_capture_mem_alloc(fmt.fmt.pix.width,fmt.fmt.pix.height);

	if( 0 != ret)
	{
		ALOGE("SPRD OEM:camera_capture_init,alloc memory error,error = %d.",ret);
		return ret;
	}

	CLEAR (streamparm);
	streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	streamparm.parm.capture.capturemode = 1;
	if(-1 == g_camera_id)
	{
		streamparm.parm.raw_data[199] = 0;
	}
	else
	{
		streamparm.parm.raw_data[199] = 1;
		streamparm.parm.raw_data[198] = g_camera_id;
	}
	g_rotation = g_rotation_parm;
	g_cam_params.orientation = g_cam_params.orientation_parm;

	ALOGV("camera_capture_init:cap mem size is %d .\n",mem_size);
	if(1 == g_cam_params.orientation)
	{
		streamparm.parm.raw_data[197] = 0;//rotation flag
		s_camera_info.is_need_rotation = 1;
		s_camera_info.rot_angle = g_rotation;
	}
	else
	{
		streamparm.parm.raw_data[197] = 0;
	}

	if (-1 == xioctl(fd, VIDIOC_S_PARM, &streamparm))
	{
		ALOGE("capture: Fail to VIDIOC_S_PARM.");
		return -1;
	}
        ALOGV("set g_capture_stop to 0");
        g_capture_stop = 0;
#if 0
	if(1 == g_camera_id)
	{ //for front camera
		uint32_t enable = 1;
		if((90 == s_camera_info.set_encode_rotation) || (270 == s_camera_info.set_encode_rotation))
		{ //for portrait
			camera_set_ctrl(V4L2_CID_VFLIP, enable);
		}
		else{ //for lanscape
			camera_set_ctrl(V4L2_CID_HFLIP, enable);
		}
	}
#endif
	return 0;
}

int camera_capture_qbuffers(void)
{
	uint32_t i;

	for (i = G_CAPTURE_BUF_OFFSET; i < g_capture_buffer_num; ++i)
	{
		struct v4l2_buffer buf;
		CLEAR (buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_USERPTR;
		buf.index = i;
		buf.m.userptr = g_buffers[i].phys_addr;
		buf.length = g_buffers[i].length;
		buf.reserved = 0;
		if(0 != g_buffers[i].u_phys_addr)
		{
			buf.reserved = g_buffers[i].u_phys_addr;
		}
		ALOGE("####capture QBuf: buffers[%d].start: %lx, %x,yaddr 0x%x,uaddr 0x%x.\n", buf.index, buf.m.userptr,buf.length,g_buffers[i].phys_addr,g_buffers[i].u_phys_addr);
		if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
		{
			ALOGE("Fail to VIDIOC_QBUF from camera_capture_qbuffers.");
			return -1;
		}
	}

	return 0;
}

int camera_capture_streamon(void)
{
	enum v4l2_buf_type type;

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
	 {
		ALOGE("Fail to VIDIOC_STREAMON.");
		return -1;
	 }
	return 0;
}

void camera_alloc_zoom_buffer(uint32_t phy_addr, uint8_t *vir_addr,uint32_t size)
{
	g_cap_zoom_buf_phy_addr = phy_addr;
	g_cap_zoom_buf_vir_addr = vir_addr;
	g_cap_zoom_buf_size = size;
	ALOGV("INTERPOLATION:camera_alloc_zoom_buffer,phy_addr=0x%x,virt_addr=0x%x,size=%d.",
		     g_cap_zoom_buf_phy_addr,(uint32_t)g_cap_zoom_buf_vir_addr,size);
}
camera_ret_code_type camera_take_picture (
        camera_cb_f_type    callback,
        void               *client_data
#if !defined FEATURE_CAMERA_ENCODE_PROPERTIES && defined FEATURE_CAMERA_V7
        ,camera_raw_type camera_raw_mode
#endif // nFEATURE_CAMERA_ENCODE_PROPERTIES && FEATURE_CAMERA_V7
        )
{
	ALOGV("camera_take_picture E.");
	camera_ret_code_type ret_type = CAMERA_SUCCESS;
	g_dcam_obj = (SprdCameraHardware *)client_data;
	int ret;
	uint32_t size;
	uint32_t real_size = 0;
	int32_t capture_format;
         pthread_attr_t attr;

         g_stop_capture_flag = 0;
	//record the callback function
	g_callback = callback;
	camera_get_sensor_mode();
	s_camera_info.sensor_out_format = camera_get_capture_format(g_dcam_dimensions.picture_width);

	//get the take picture memory
	size = (g_dcam_dimensions.picture_width * g_dcam_dimensions.picture_height * 2) * g_capture_buffer_num;
	if(NULL == (g_capture_virt_addr = (uint32_t *)g_dcam_obj->get_raw_mem(size,&g_capture_phys_addr,0,&real_size)))
	{
		ALOGE("Fail to malloc capture memory, size: 0x%x.", size);
		return CAMERA_FAILED;
	}
	g_dcam_obj->getPictureFormat(&capture_format);
	if(0 != camera_capture_init(real_size,capture_format))
	{
		ALOGE("Fail to init capture mode.");
		return CAMERA_FAILED;
	}
	if(0 != camera_capture_qbuffers())
	{
		ALOGE("Fail to qbuffers for capture.");
		return CAMERA_FAILED;
	}
	ALOGV("OK  to camera_capture_qbuffers.");
	if(0 != camera_capture_streamon())
	{
		ALOGE("Fail to stream on for capture.");
		return CAMERA_FAILED;
	}

	//create the thread for preview	
	pthread_attr_init (&attr);
         pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	if(0 != (ret = pthread_create(&g_capture_thr, &attr, camera_capture_thread, client_data)))
	{
		ALOGE("Fail to careate thread in preview mode.");
		return CAMERA_FAILED;
	}
	else
	{
		//pthread_join(g_capture_thr, NULL);
		ALOGE("OK to create thread in capture mode.");
	}

	ALOGV("camera_take_picture X.");

	return ret_type;
}

void camera_get_sensor_max_size(uint32_t *width_ptr,uint32_t *height_ptr)
{
	struct v4l2_crop crop;

	crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	*width_ptr = 0;
	*height_ptr = 0;

	if(-1 == xioctl(fd, VIDIOC_G_CROP, &crop))
	{
		ALOGE("CAMERA OEM:camera_get_sensor_max_size:get crop error!");
		return;
	}

	*width_ptr = crop.c.width;
	*height_ptr = crop.c.height;
	ALOGV("CAMERA OEM:get sensor max size:w=%d,h=%d.\n",*width_ptr,*height_ptr);

}

void camera_get_sensor_mode(void)
{
	uint32_t i = 0;
	struct v4l2_streamparm streamparm;
	uint8_t *data_ptr = NULL;
	uint16_t temp;
	SENSOR_MODE_INFO_T *info_ptr = NULL;
         streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(-1 == xioctl(fd, VIDIOC_G_PARM,&streamparm))
	{
		ALOGE("CAMERA OEM: vidioc_g_param error! \n");
		return;
	}

	if(streamparm.parm.raw_data[2]>10)
	{
		ALOGE("CAMERA OEM:get sensor mode error,,sensor mode info array is small! \n");
		return;
	}
	memset(&s_camera_info.sensor_mode_info[0],0,10*sizeof(SENSOR_MODE_INFO_T));
	data_ptr = &streamparm.parm.raw_data[3];
	for(i=0 ; i<streamparm.parm.raw_data[2];i++)
	{
		info_ptr = &s_camera_info.sensor_mode_info[i];
		info_ptr->mode = *data_ptr++;
		temp = *data_ptr++;
		temp |= ((*data_ptr++)<<8);
		info_ptr->width = temp;
		temp = *data_ptr++;
		temp |= ((*data_ptr++)<<8);
		info_ptr->height =  temp;
		temp = *data_ptr++;
		temp |= ((*data_ptr++)<<8);
		info_ptr->trim_start_x =  temp;
		temp = *data_ptr++;
		temp |= ((*data_ptr++)<<8);
		info_ptr->trim_start_y =  temp;
		temp = *data_ptr++;
		temp |= ((*data_ptr++)<<8);
		info_ptr->trim_width =  temp;
		temp = *data_ptr++;
		temp |= ((*data_ptr++)<<8);
		info_ptr->trim_height =  temp;
		info_ptr->image_format = (SENSOR_IMAGE_FORMAT_E)*data_ptr++;

		ALOGV("CAMERA OEM:get sensor mode:\n");
		ALOGV("mode=%d,widht=%d,height=%d,format=%d.\n",info_ptr->mode,info_ptr->width,info_ptr->height,
			  info_ptr->image_format);
	}

}

SENSOR_IMAGE_FORMAT_E camera_get_capture_format(uint32_t width)
{
	SENSOR_IMAGE_FORMAT_E image_format = SENSOR_IMAGE_FORMAT_MAX;
	uint32_t i;
	uint32_t sensor_width;

	for(i = 0; i < 10; i++)
	{
		sensor_width = s_camera_info.sensor_mode_info[i].width;
		if(width<=sensor_width)
		{
			image_format = s_camera_info.sensor_mode_info[i].image_format;
			break;
		}
                  else if(width<=SCALE_OUT_WIDTH_MAX)
                  {
                           image_format = s_camera_info.sensor_mode_info[i].image_format;
			break;
                  }
	}
	if(SENSOR_IMAGE_FORMAT_MAX == image_format)
	{
		for(i = 9; i >0; i--)
		{
			if(0 != s_camera_info.sensor_mode_info[i].width)
			{
				image_format = s_camera_info.sensor_mode_info[i].image_format;
				break;
			}
		}
	}
	s_camera_info.sensor_out_format = image_format;
	ALOGV("CAMERA OEM:get sensor format = %d .\n",(uint32_t)image_format);
	return image_format;
}
void rex_start()
{
	return;
}
void rex_shutdown()
{
	return;
}

void camera_encoder_start_flash(void)
{
	camera_set_ctrl(V4L2_CID_GAMMA, 0x22);
	ALOGV("SPRD OEM: camera_encoder_start_flash  \n");
}
void camera_set_preview_mode(int mode)
{
	ALOGV("SPRD OEM:camera_set_preview_mode,mode = %d.\n",mode);
	camera_set_ctrl(V4L2_CID_BLACK_LEVEL,mode);
}
};
