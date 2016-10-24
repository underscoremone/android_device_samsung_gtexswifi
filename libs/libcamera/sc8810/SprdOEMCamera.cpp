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

#include "SprdOEMCamera.h"
#include "scale_sc8800g2.h"
#include "rotation_sc8800g2.h"
#include "jpegenc_api.h"
#include "SprdCameraHardwareCapability.h"
extern "C" {
#include "sqlite/eng_sqlite.h"
}

namespace android{

#define USE_TWO_KIND_BUF 1 //wxz20111108: 0: use the buffers for display and HW; 1: use the buffers for HW, another buffers for display.
#define HAL_SET_SENSOR_ID 0 //wxz20111022: 0: camera driver will find the right sensor id; 1: hal will set the right sensor id.
#define FRONT_CAMERA_MIRROR 1//wxz20111112: 0: not support the mirror in capture mode for front camera; 1: support the function.

#define PREVIEW_BUF_NUM 6//5
#define CAPTURE_BUF_NUM 1
#define JPEGENC_BUF_NUM 1

#if USE_TWO_KIND_BUF
#define PREVIEW_BUF_NUM_FOR_DISPLAY 4
#endif

#define PIXEL_ALIGNED 16
#define W_H_ALIGNED(x) (((x) + PIXEL_ALIGNED - 1) & ~(PIXEL_ALIGNED - 1))

#define SCALE_SLICE_HEIGHT 128

#define PREVIEW_ENDIAN_H 1

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
        void *addr; //logical address
        uint32_t length; //buffer size
        int external;
}PMEM_INFO_T;

typedef struct dcam_params
{
	uint32_t preview_mode;
	uint32_t orientation; //wxz20110815: 0: default, landscape; 1: portrait
	uint32_t orientation_parm; 
	uint32_t is_2M_to_3M; //wxz20110825: 0: not need 2M to 3M; 1: need 2M to 3M in capture mode.
	int32_t rotation_degree;
	char datetime_buf[20];
	char gps_date_buf[11];	
	uint32_t focal_length; //wxz20111008: the value is numerator, the denominator is 1000.
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
        uint16_t display_height,;
}DCAM_DIMENSIONS_T;
struct buffer {   
    uint32_t *virt_addr;	
    uint32_t phys_addr;
    uint32_t length;//buffer's length is different from cap_image_size   
}; 

#define G_PREVIEW_BUF_OFFSET 0 // 2 //the freview buffer from this number
#define G_CAPTURE_BUF_OFFSET 0 //the capture buffer from this number
struct buffer * g_buffers = NULL;   //the global buffers for preview and capture; the buffer 0 is the output buffer.
#if USE_TWO_KIND_BUF
struct buffer * g_buffers_for_display = NULL;   //the global buffers in preview mode for display.
#endif
static uint32_t n_buffers = 0; //restore the number of buffers


#define CLEAR(x) memset (&(x), 0, sizeof (x))   
#define PMEM_INFO_NUM 45

//wxz20111021: the strings for camera number in sqlite database.
#define BACK_CAMERA_NUM "sprdbackcameranum"
#define FRONT_CAMERA_NUM "sprdfrontcameranum"

#define BACK_CAMERA_ID 0
#define FRONT_CAMERA_ID 1
//store the pmem info: file handle, logical address of buffer, buffer size.
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
pthread_t g_restart_thr; 
uint32_t g_preview_buffer_num = PREVIEW_BUF_NUM; //the number of preview buffers
#if USE_TWO_KIND_BUF
uint32_t g_preview_buffer_num_for_display = PREVIEW_BUF_NUM_FOR_DISPLAY; //the number of preview buffers for display
#endif
uint32_t g_capture_buffer_num = CAPTURE_BUF_NUM; //the number of capture buffers
uint32_t g_preview_phys_addr; //the start physical address of preview buffers
uint32_t *g_preview_virt_addr = NULL; //the start logical address of preview buffers
#if USE_TWO_KIND_BUF
uint32_t g_preview_phys_addr_for_display; //the start physical address of preview buffers
uint32_t *g_preview_virt_addr_for_display = NULL; //the start logical address of preview buffers
#endif
uint32_t g_capture_phys_addr; //the start physical address of capture buffers
uint32_t *g_capture_virt_addr = NULL;//the start logical address of capture buffers
uint32_t g_preview_size; //store the size of preview picture, such as ARGB888
uint32_t g_capture_size; //store the size of capture picture, such as YUV420
volatile static uint32_t g_stop_preview_flag = 0; //0: will not stop the preview thread; 1: will stop the preview thread 
volatile static uint32_t g_preview_stop = 0; //0: the preview thread is not stop; 1: the preview thread is stop.
volatile static uint32_t g_stop_capture_flag = 0;//0: will not stop the capture thread; 1: will stop the capture thread 
volatile static uint32_t g_capture_stop = 0;//0: the capture thread is not stop; 1: the capture thread is stop.
SprdCameraHardware *g_dcam_obj = NULL;//store the SprdCameraHardware object
uint32_t g_buf_flag[3];
int32_t g_camera_id;
//uint32_t g_dqbuf_index = 0; //store the buffer index by DQBuf.
//static uint32_t g_dqbuff_index = 0; //store the DQ buffer index.
static uint32_t g_releasebuff_index = 0; //store the rellease buffer index.
camera_encode_mem_type g_encoder_type;
static char dev_name[50] = "/dev/video0";
static int fd = -1;
//#define  DEV_NAME "../../dev/video0"
void *g_client_data = NULL; //fro jpeg encoder
JPEGENC_PARAMS_T g_jpegenc_params;
uint32_t g_jpeg_stream_size = 0; //store the size of jpeg picture by jpeg encoder.
uint32_t g_hal_zoom_level = 0;//zoom level: 0, 1, 2., 3
uint32_t g_cap_zoom_buf_phy_addr = 0;
uint8_t *g_cap_zoom_buf_vir_addr = NULL;
uint32_t g_cap_zoom_buf_size = 0;
uint32_t g_slice_swap_buf = 0;
JPEGENC_QUALITY_E g_jpeg_quality_level = JPEGENC_QUALITY_HIGH;//jpeg quality level for jpeg encoder.
uint32_t g_rotation = 0; //wxz20110725: 0, 90, 180, 270, store the rotation agree for the preview and capture.
uint32_t g_rotation_parm = 0; //wxz20110725: 0, 90, 180, 270, store the rotation agree for the preview and capture.
uint32_t g_max_w = atoi(get_back_camera_capability("max_width"));//wxz20111024: get the max width of the back camera.
uint32_t g_max_h = atoi(get_back_camera_capability("max_height"));


static void errno_exit(const char * s) {   
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));   
    exit(EXIT_FAILURE);   
}
static int xioctl(int fd, int request, void * arg) {   
    int r;   
    //do   
        r = ioctl(fd, request, arg);   
    //while (-1 == r && EINTR == errno);   
    return r;   
} 

static void close_device(void) {   
    if (-1 == close(fd))   
        errno_exit("close");   
    fd = -1;   
    LOGV("#####DCAM: close device.\n");   
}

int32_t camera_get_sensor_number(uint32_t sensor_id)
{	
	int32_t status = -1;

	if(BACK_CAMERA_ID == sensor_id){
		status = eng_sql_string2int_get((char *)BACK_CAMERA_NUM);
	}
	else if(FRONT_CAMERA_ID == sensor_id){
		status = eng_sql_string2int_get((char *)FRONT_CAMERA_NUM);
	}
	else{
		LOGE("fail to camera_get_sensor_number, sensor_id: %d.", sensor_id);
		return -1;
	}

	if(ENG_SQLSTR2INT_ERR == status){ //not set the sensor number.
		//set the sensor number by
		uint32_t back_num = atoi(get_back_camera_capability("sensor_num"));
		uint32_t front_num = atoi(get_front_camera_capability("sensor_num"));
		if(0 != eng_sqlite_create()){
			LOGE("Fail to eng_sqlite_create.");
			return -1;
		}
		LOGV("set sensor number, back_sensor_num: %d, front_sensor_num: %d.", back_num, front_num);
		if(0 != eng_sql_string2int_set((char *)BACK_CAMERA_NUM, back_num)){
			LOGE("Fail to set sensor number, back_sensor_num: %d.", back_num);
			return -1;
		}
		else{
			LOGV("ok to set sensor number, back_sensor_num: %d.", back_num);
		}
		if(0 != eng_sql_string2int_set((char *)FRONT_CAMERA_NUM, front_num)){
			LOGE("Fail to set sensor number, front_sensor_num: %d.", front_num);
			return -1;
		}
		else{
			LOGV("ok to set sensor number, front_sensor_num: %d.", front_num);
		}
		status = sensor_id == BACK_CAMERA_ID ? back_num : front_num;		
	}
	else{
		LOGV("ok to get sensor number, sensor_id: %d, sensor_num: %d.", sensor_id, status);
	}
	
	return status;
}
int32_t camera_set_sensor_number(uint32_t sensor_id, uint32_t sensor_num)
{
	LOGV("set sensor number, sensor id: %d, num: %d.", sensor_id, sensor_num);
	if(BACK_CAMERA_ID == sensor_id){
		eng_sql_string2int_set((char *)BACK_CAMERA_NUM, sensor_num);		
	}
	else if(FRONT_CAMERA_ID == sensor_id){
		eng_sql_string2int_set((char *)FRONT_CAMERA_NUM, sensor_num);		
	}
	else{
		LOGE("fail to camera_set_sensor_number, sensor_id: %d, sensor_num: %d.", sensor_id, sensor_num);
		return -1;
	}	
	
	return 0;
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
#if 0 //wxz:???
	uint32_t i;
	
	if(module >= QDSP_MODULE_MAX)
	{
		LOGE("Fail to camera_assoc_pmem: the module is invalid.");
		return;
	}
	for(i = 0; i < PMEM_INFO_NUM; i++)
	{
		if(i == (uint32_t)module)
		{
			g_pmem_info[i].module = module;
			g_pmem_info[i].pmem_fd = pmem_fd;
			g_pmem_info[i].addr = addr;
			g_pmem_info[i].length = length;
			g_pmem_info[i].external = external;			
			break;
		}
	}
#endif
	return;
}

void clear_module_pmem(qdsp_module_type module)
{
	uint32_t i;
	
	if(module >= QDSP_MODULE_MAX)
	{
		LOGE("Fail to clear_module_pmem: the module is invalid.");
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
#if 0 //wxz:???
	uint32_t i;
	
	if(module >= QDSP_MODULE_MAX)
	{
		LOGE("Fail to camera_release_pmem: the module is invalid.");
		return -1;
	}
	if(NULL == addr)
	{
		LOGE("Fail to camera_release_pmem: the address is invalid.");
		return -1;
	}		
	for(i = 0; i < PMEM_INFO_NUM; i++)
	{
		if(i == (uint32_t)module)
		{
			if(true == force)
			{
				//release the memory
				struct pmem_region region;
				region.len = size;
				region.offset = 0;
				::ioctl(g_pmem_info[i].pmem_fd,PMEM_UNMAP,&region);
			}
			else
			{
				//what to do?
			}			
			break;
		}
	}
#endif
	return 0;
}

void camera_encoder_callback(uint32_t buf_id, uint32_t stream_size, uint32_t is_last_slice)
{
	LOGV("camera_encoder_callback: buf_id,  stream_size,  is_last_slice:%d, %d, %d. ", buf_id, stream_size, is_last_slice);
	g_encoder_param.header_size = 0;
	g_encoder_param.mode = JPEGENC_MEM;	
	g_encoder_type.buffer = (uint8_t *)g_jpegenc_params.stream_virt_buf[buf_id];	
	g_encoder_param.outPtr = &g_encoder_type;
	g_encoder_param.status = JPEGENC_IMG_DONE;	
	LOGV("outPtr: 0x%x, size: 0x%x.", (uint32_t)g_encoder_param.outPtr, stream_size);
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
	if(0 == type){
		retVal = (int)degrees;
	}
	else if(1 == type){
		retVal = (int)minutes;
	}
	else if(2 == type){
		retVal = (int)seconds;
	}
	
	LOGE("wxz: GPS: type: %d, ret: 0x%x.", type, retVal);
	
        return retVal;
    }

    static uint32_t getOrientationFromRotationDegrees(int degrees) {
	uint32_t orientation = 1;//ExifInterface.ORIENTATION_NORMAL;
	   degrees %= 360;
           if (degrees < 0) degrees += 360;            
           if(degrees < 45){
                    orientation = 1;//ExifInterface.ORIENTATION_NORMAL;
           }
	    else if(degrees < 135){
			orientation = 6;//ExifInterface.ORIENTATION_ROTATE_90;
	    }
	    else if(degrees < 225){
                    orientation = 3;//ExifInterface.ORIENTATION_ROTATE_180;
	    }
	    else{
                    orientation = 8;//ExifInterface.ORIENTATION_ROTATE_270;                    
            }
       	LOGV("wxz: rotation degrees: %d, orientation: %d.", degrees, orientation);
	return orientation;
    }

void *camera_encoder_thread(void *client_data)
{
	//JPEGENC_PARAMS_T jpegenc_params;
	uint32_t jpeg_enc_buf_phys_addr;
	uint32_t* jpeg_enc_buf_virt_addr;
	uint32_t jpeg_enc_buf_len;
	uint32_t i;
	
	LOGV("camera_encoder_thread E.");	
	g_client_data = client_data;
	g_jpegenc_params.format = JPEGENC_YUV_420;
	g_jpegenc_params.quality = g_jpeg_quality_level;
	g_jpegenc_params.width = g_dcam_dimensions.picture_width;
	g_jpegenc_params.height = g_dcam_dimensions.picture_height;
	g_jpegenc_params.yuv_virt_buf =  g_buffers[g_releasebuff_index].virt_addr;
	g_jpegenc_params.yuv_phy_buf = g_buffers[g_releasebuff_index].phys_addr;
   	#if 0
			{
				FILE *fp = NULL;
				static uint32_t first = 1;
				//if(1 == first)
				{
				LOGE("enc yuv420: width: %d, hei: %d.", g_jpegenc_params.width, g_jpegenc_params.height);
				//LOGE("cap yuv422: width: %d, hei: %d.", trim_rect.w, trim_rect.h);
				fp = fopen("/data/out_enc_yuv420_before.yuv", "wb");
				fwrite(g_jpegenc_params.yuv_virt_buf, 1, g_jpegenc_params.width * g_jpegenc_params.height * 3 / 2, fp);
				//fwrite(g_buffers[g_releasebuff_index].virt_addr, 1, trim_rect.w*trim_rect.h * 2, fp);
				fclose(fp);
				first = 0;
				}
			}		
	#endif
	jpeg_enc_buf_len = JPEG_ENC_HW_PMEM;
	jpeg_enc_buf_len = camera_get_size_align_page(jpeg_enc_buf_len);
	jpeg_enc_buf_virt_addr = (uint32_t *)g_dcam_obj->get_jpeg_encoder_mem_by_HW(&jpeg_enc_buf_phys_addr);
    	for (i = 0; i < JPEG_ENC_HW_BUF_NUM; i++) 
	{
		g_jpegenc_params.stream_virt_buf[i] = jpeg_enc_buf_virt_addr + i * jpeg_enc_buf_len / 4;
		g_jpegenc_params.stream_phy_buf[i] = jpeg_enc_buf_phys_addr + i * jpeg_enc_buf_len;		
		//memset(g_jpegenc_params.stream_virt_buf[i], 0xFF, jpeg_enc_buf_len);
		LOGV("encoder: jpegenc_params[%d]: virt: %x, phys: %x.",i,(uint32_t)g_jpegenc_params.stream_virt_buf[i],g_jpegenc_params.stream_phy_buf[i]);
        }
	g_jpegenc_params.stream_buf_len = jpeg_enc_buf_len;
	g_jpegenc_params.stream_size = 0;

	g_jpegenc_params.thumb_width = g_thumbnail_properties.width;
	g_jpegenc_params.thumb_height = g_thumbnail_properties.height;
	g_jpegenc_params.thumb_quality = g_thumbnail_properties.quality;
	g_jpegenc_params.Latitude_dd.numerator = getDataFromDouble(g_position.latitude, 0);
	g_jpegenc_params.Latitude_dd.denominator = 1;
	g_jpegenc_params.Latitude_mm.numerator = getDataFromDouble(g_position.latitude, 1);
	g_jpegenc_params.Latitude_mm.denominator = 1;
	g_jpegenc_params.Latitude_ss.numerator = getDataFromDouble(g_position.latitude, 2);
	g_jpegenc_params.Latitude_ss.denominator = 1;
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
	g_jpegenc_params.Longitude_ss.numerator = getDataFromDouble(g_position.longitude, 2);
	g_jpegenc_params.Longitude_ss.denominator = 1;
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
	g_jpegenc_params.orientation = getOrientationFromRotationDegrees(g_cam_params.rotation_degree);
	 {
	 	time_t timep;
		struct tm *p;
		//char time_buf[19];
		//char date_buf[10];		
		time(&timep);
		p=gmtime(&timep);
		sprintf(g_cam_params.datetime_buf, "%4d:%2d:%2d %2d:%2d:%2d", (1900+p->tm_year), (1+p->tm_mon),p->tm_mday,
			p->tm_hour, p->tm_min, p->tm_sec);
		g_cam_params.datetime_buf[19] = '\0';
	 	g_jpegenc_params.datetime = g_cam_params.datetime_buf;

		p=gmtime(&g_position.timestamp);
		//p=gmtime(&timep); //for test
		sprintf(g_cam_params.gps_date_buf, "%4d:%2d:%2d", (1900+p->tm_year), (1+p->tm_mon),p->tm_mday);
		g_cam_params.gps_date_buf[10] = '\0';
		g_jpegenc_params.gps_date = g_cam_params.gps_date_buf;
		g_jpegenc_params.gps_hour = p->tm_hour;
		g_jpegenc_params.gps_minuter = p->tm_min;
		g_jpegenc_params.gps_second = p->tm_sec;		
	 }
	 g_jpegenc_params.gps_process_method = g_position.process_method;
	g_jpegenc_params.focal_length.numerator = g_cam_params.focal_length;
	g_jpegenc_params.focal_length.denominator = 1000;
	
  	//encode the jpeg picture by HW.	
	if(0 != JPEGENC_encode_one_pic(&g_jpegenc_params, camera_encoder_callback))
	{
		LOGE("Fail to encode jpeg picture by SC8800G2 HW.");
		return NULL;
	}
	/*
	g_encoder_param.header_size = 0;
	g_encoder_param.mode = JPEGENC_MEM;	
	g_encoder_type.buffer = (uint8_t *)jpegenc_params.stream_virt_buf[0];	
	g_encoder_param.outPtr = &g_encoder_type;
	g_encoder_param.status = JPEGENC_IMG_DONE;
	g_encoder_param.size = jpegenc_params.stream_size;
	LOGV("outPtr: 0x%x, size: 0x%x.", (uint32_t)g_encoder_param.outPtr, g_encoder_param.size);
	g_callback(CAMERA_EXIT_CB_DONE, client_data, CAMERA_FUNC_ENCODE_PICTURE, (uint32_t)&g_encoder_param);
	*/
	LOGV("camera_encoder_thread X.");

	return NULL;
}
camera_ret_code_type camera_encode_picture(
        camera_frame_type *frame,
        camera_handle_type *handle,
        camera_cb_f_type callback,
        void *client_data)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;
	
	g_callback = callback;

	/*g_encoder_param.header_size = frame->header_size;
	g_encoder_param.mode = JPEGENC_MEM;	
	g_encoder_type.buffer = (uint8_t *)frame->buf_Virt_Addr;	
	g_encoder_param.outPtr = &g_encoder_type;	
	g_encoder_param.status = JPEGENC_IMG_DONE;
	g_encoder_param.size = frame->dx * frame->dy * 2;*/

	//create the thread for encoder	
	if(0 !=  pthread_create(&g_encoder_thr, NULL, camera_encoder_thread, client_data))
	{
		LOGE("Fail to careate thread in encoder mode.");
		return CAMERA_FAILED;
	}
	else
	{		
		LOGV("OK to create thread in encoder mode.");
	}
	
	return ret_type;
}


static void open_device(void)
{  
	LOGV("Start to open_device.");
    
    fd = open("/dev/video0", O_RDWR /* required */, 0);      
    if (-1 == fd) {   
	LOGE("Fail to open dcam device.errno : %d", errno);
        fprintf(stderr, "Cannot open '%s': %d, %s\n", dev_name, errno,  strerror(errno));   
        exit(EXIT_FAILURE);   
    }
    else
      LOGV("#####DCAM: OK to open device.");       
}
camera_ret_code_type camera_init(int32_t camera_id)
{
	struct v4l2_streamparm streamparm;	
	
	g_camera_id = camera_id;
	open_device();

        //wxz20111021: raw_data
        //BYTE0: 0: HAL doesn't set sensor_id; 1: HAL sets sensor_id;
        //BYTE1: 0: back camera if BYTE0 is 1; 1: front camera if BYTE0 is 1;
        //BYTE2: 0: no crop and rotation; 1: crop and rotation;
        //BYTE3: 0: HAL doesn't set sensor number; 1: HAL sets sensor number; 2: need to update the sensor number;
	//BYTE4: back sensor number if BYTE3 is 1; If the number is right, use it; not right, check all sensors and change the number; 0xFF is bad sensor;
	//BYTE5: front sensor number if BYTE3 is 1; If the number is right, use it; not right, check all sensors and change the number; 0xFF is bad sensor;
	CLEAR (streamparm);   
	streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	//streamparm.parm.capture.capturemode = 0;
	streamparm.parm.raw_data[6] = 0;
	if(-1 == g_camera_id)
	{
		streamparm.parm.raw_data[0] = 0;
	}
	else
	{
		streamparm.parm.raw_data[0] = 1;
		streamparm.parm.raw_data[1] = g_camera_id;
	}
	g_rotation = g_rotation_parm; 
	g_cam_params.orientation = g_cam_params.orientation_parm;
	if((1 == g_cam_params.orientation) && ((90 == g_rotation) || (270 == g_rotation))){
		streamparm.parm.raw_data[2] = 1;
	}
	else{
		streamparm.parm.raw_data[2] = 0;
	}
#if HAL_SET_SENSOR_ID
	//wxz20111021: set sensor number
	streamparm.parm.raw_data[3] = 1;
	streamparm.parm.raw_data[4] = camera_get_sensor_number(BACK_CAMERA_ID);//back camera
	streamparm.parm.raw_data[5] = camera_get_sensor_number(FRONT_CAMERA_ID);//front camera
#endif
	if (-1 == xioctl(fd, VIDIOC_S_PARM, &streamparm)) 
	{
		LOGE("preview: Fail to VIDIOC_S_PARM.");
		return CAMERA_FAILED;
	}
#if HAL_SET_SENSOR_ID
	//need to update the sensor number.
	LOGV("data1: %d, data3: %d, data4: %d, data5: %d.", streamparm.parm.raw_data[1], streamparm.parm.raw_data[3], streamparm.parm.raw_data[4], streamparm.parm.raw_data[5]);
	if(2 == streamparm.parm.raw_data[3]){
		if(BACK_CAMERA_ID == streamparm.parm.raw_data[1]){
			camera_set_sensor_number(BACK_CAMERA_ID, streamparm.parm.raw_data[4]);
		}
		else if(FRONT_CAMERA_ID == streamparm.parm.raw_data[1]){
			camera_set_sensor_number(FRONT_CAMERA_ID, streamparm.parm.raw_data[5]);
		}
	}
#endif
	
	return CAMERA_SUCCESS;
}
void camera_af_init(void)
{
	//NO USED now.
	
	//init the camera auto focus
	//TODO:
	return;
}
camera_ret_code_type camera_preview_qbuf(uint32_t index)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;
	struct v4l2_buffer buf;	
	//wxz20110907: if it want to stop, not need to qbuf; otherwise, if streamoff is ok, but qbuf will be done. The stream list will not empty int v4l2 driver.
	if((0 == g_preview_stop) && (0 == g_stop_preview_flag))
	//if(0 == g_preview_stop)//wxz20110823: if 1==g_preview_stop, the preview has stop. else it not. so need to qbuf.
	{
		CLEAR (buf);   
        	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;   
        	buf.memory = V4L2_MEMORY_USERPTR;
		//buf.m.userptr = (unsigned long) g_buffers[g_releasebuff_index].phys_addr;
		//buf.length = (unsigned long) g_buffers[g_releasebuff_index].length;
		//buf.index = g_releasebuff_index;		
		buf.m.userptr = (unsigned long) g_buffers[index].phys_addr;
		buf.length = (unsigned long) g_buffers[index].length;
		buf.index = index;		
		//LOGV("camera_preview_qbuf E index: %d.", index);
	       	if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))   
		{
			//LOGE("Fail to VIDIOC_QBUF in camera_release_frame.buff_index: %d", g_releasebuff_index);
			LOGE("Fail to VIDIOC_QBUF in camera_release_frame.buff_index: %d", index);
			return CAMERA_FAILED;
		}
	}
	return ret_type;
}
camera_ret_code_type camera_release_frame(uint32_t index)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;	
	LOGV("camera_recording_release_frame E index: %d.", index);
#ifdef PREVIEW_BUF_3_2	
	//LOGV("camera_recording_release_frame E,flag: %d, %d, %d.", g_buf_flag[0], g_buf_flag[1], g_buf_flag[2]);	
	if((1 == g_buf_flag[0]) && (1 == g_buf_flag[1]))
	{
		if(0 == g_buf_flag[2])
			g_buf_flag[0] = 0;
		else
			g_buf_flag[1] = 0;
	}
	else if(1 == g_buf_flag[0])
		g_buf_flag[0] = 0;
	else
		g_buf_flag[1] = 0;
#endif	
	ret_type = camera_preview_qbuf(index);
	//LOGV("camera_recording_release_frame X.");	
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
	LOGV("camera_set_jpegcomp E.quality = %d.", quality);
	
	if(quality > 90){
		g_jpeg_quality_level = JPEGENC_QUALITY_HIGH;
	}
	else if(quality > 80){
		g_jpeg_quality_level = JPEGENC_QUALITY_MIDDLE;
	}
	else { // if(quality > 70)
		g_jpeg_quality_level = JPEGENC_QUALITY_LOW;
	}
	
	LOGV("camera_set_jpegcomp X.");

	return 0;
}
	
static int camera_set_ctrl(uint32_t key, int32_t value)
{
	struct v4l2_control ctrl;

	LOGV("camera_set_ctrl E, key: 0x%x, value: %d.", key, value);

	ctrl.id = key; 
	ctrl.value = value;
	
	if (-1 == xioctl(fd, VIDIOC_S_CTRL, &ctrl)) 
	{
		LOGE("Fail to VIDIOC_S_CTRL.");
		return -1;
	}

	LOGV("camera_set_ctrl X.");

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
	if(id >= CAMERA_PARM_MAX_SPRD)
	{
		LOGE("Fail to camera_set_parm: the id: %d is invalid.", id);
		return CAMERA_INVALID_PARM;
	}

	switch(id)
	{	    
		// CAMERA_PARM_STATE:/* read only operation states: camera_state_type */
		
	    	// CAMERA_PARM_ACTIVE_CMD/* read only active command in execution: camera_func_type */:
			
		// CAMERA_PARM_ZOOM:/* zoom */
			
    /* This affects only when encoding. It has to be set only in preview mode */
		case CAMERA_PARM_ENCODE_ROTATION: /* 0, 90, 180, 270 degrees */
			//g_rotation_parm = (uint32_t)parm;
			g_cam_params.rotation_degree = parm;
			break;
		case CAMERA_PARM_SENSOR_ROTATION: /* 0, 90, 180, 270 degrees */
			g_rotation_parm = (uint32_t)parm;
			break;
		case CAMERA_PARM_FOCAL_LENGTH: 
			g_cam_params.focal_length = (uint32_t)parm;
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
			LOGV("wxz: set_parm orientation: %d.", g_cam_params.orientation_parm);
			break;
		// CAMERA_PARM_AUDIO_FMT:      /* use video_fmt_stream_audio_type */
			
		// CAMERA_PARM_FPS:            /* frames per second, unsigned integer number */
					
		// CAMERA_PARM_FLASH:         /* Flash control, see camera_flash_type */
			
		// CAMERA_PARM_RED_EYE_REDUCTION: /* boolean */
			
		case CAMERA_PARM_NIGHTSHOT_MODE:  /* Night shot mode, snapshot at reduced FPS */
			//TODO
			break;
		// CAMERA_PARM_REFLECT:        /* Use camera_reflect_type */
			
		case CAMERA_PARM_PREVIEW_MODE:   /* Use camera_preview_mode_type */
			g_cam_params.preview_mode = parm;			
			break;
		case CAMERA_PARM_ANTIBANDING:   /* Use camera_anti_banding_type */
			//TODO
			break;
		// CAMERA_PARM_FOCUS_STEP
		// CAMERA_PARM_FOCUS_RECT:/* Suresh Gara & Saikumar*/			
		// CAMERA_PARM_AF_MODE
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

    	LOGV("timestamp %ld, latitude : %f, longitude : %f,altitude: %d. ", position->timestamp, position->latitude, position->longitude,position->altitude);
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

static void init_device(void) {   
    struct v4l2_capability cap;   
    struct v4l2_cropcap cropcap;   
    struct v4l2_crop crop;   
    struct v4l2_format fmt;   
    unsigned int min;  
    LOGV("#####DCAM: VIDIOC_QUERYCAP start.\n"); 
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
    LOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");   
    CLEAR (cropcap);   
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;   
    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {   
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;   
#ifndef CROP_BY_JACK   
        crop.c = cropcap.defrect; /* reset to default */   
#else   
        crop.c.left = cropcap.defrect.left;   
        crop.c.top = cropcap.defrect.top;   
        crop.c.width = 640;   //wxz: 320x480 ???
        crop.c.height = 480;   
#endif   
        LOGV("----->has ability to crop!!\n");   
        LOGV("cropcap.defrect = (%d, %d, %d, %d)\n", cropcap.defrect.left,   
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
            LOGV("-----!!but crop to (%d, %d, %d, %d) Failed!!\n",   
                    crop.c.left, crop.c.top, crop.c.width, crop.c.height);   
        } else {   
            LOGV("----->sussess crop to (%d, %d, %d, %d)\n", crop.c.left,   
                    crop.c.top, crop.c.width, crop.c.height);   
        }   
    } else {   
        /* Errors ignored. */   
        LOGV("!! has no ability to crop!!\n");   
    }   
    LOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");   
    LOGV("\n");   
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
	LOGV("OK to init_device.");
	//change the status from INIT to IDLE.
	callback(CAMERA_STATUS_CB, client_data, CAMERA_FUNC_START, 0);	
	LOGV("OK to change the status from INIT to IDLE.");
	
	g_dcam_dimensions.display_width = display_width;
	g_dcam_dimensions.display_height = display_height;	
	
	return ret_type;
}
//static int convert_format_by_DMA(uint32_t width, uint32_t height, uint32_t address)
static int convert_format_by_DMA(uint32_t width, uint32_t height,  uint32_t address, uint32_t in_height)
{
	SCALE_YUV422_YUV420_T yuv_config;
	int fd = -1; 
	
	fd = open("/dev/sc8800g_scale", O_RDONLY);
	if (-1 == fd) 
	{   
		LOGE("Fail to open scale device for DMA.");
        	return -1;   
   	 }
    	else
      		LOGV("OK to open scale device for DMA.");

	yuv_config.width = width;
	yuv_config.height = height;
	//yuv_config.src_addr = address + width * height;
	yuv_config.src_addr = address + width * in_height;
	yuv_config.src_addr += width;
	yuv_config.dst_addr = address + width * height;
	
	if (-1 == xioctl(fd, SCALE_IOC_YUV422_YUV420, &yuv_config))   
	{
		LOGE("Fail to SCALE_IOC_YUV422_YUV420.");
		return -1;
	}	

	if(-1 == close(fd))   
	{   
		LOGE("Fail to close scale device for DMA.");
        	return -1;   
   	 } 
    	fd = -1;   

	return 0;	
}
void camera_alloc_swap_buffer(uint32_t phy_addr)
{
	g_slice_swap_buf = phy_addr;
	LOGV("INTERPOLATION:g_slice_swap_buf=0x%x.",g_slice_swap_buf);
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

	fd = open("/dev/sc8800g_scale", O_RDONLY);//O_RDWR /* required */, 0);  
	if (-1 == fd) 
	{   
		LOGE("Fail to open scale device.");
        	return -1;   
   	 }
    	
	//set mode
	scale_config.id = SCALE_PATH_MODE;	
	scale_mode = SCALE_MODE_SCALE;
	scale_config.param = &scale_mode;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set input data format
	scale_config.id = SCALE_PATH_INPUT_FORMAT;
	//data_format = SCALE_DATA_YUV422;
	data_format = input_fmt;
	scale_config.param = &data_format;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set output data format
	scale_config.id = SCALE_PATH_OUTPUT_FORMAT;
	data_format = output_fmt;
	scale_config.param = &data_format;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set input size
	scale_config.id = SCALE_PATH_INPUT_SIZE;
	scale_size.w = trim_rect->w;
	scale_size.h = trim_rect->h;
	//scale_size.w = output_width;
	//scale_size.h = output_height;
	scale_config.param = &scale_size;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set output size
	scale_config.id = SCALE_PATH_OUTPUT_SIZE;
	scale_size.w = output_width;
	scale_size.h = output_height;
	scale_config.param = &scale_size;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
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
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set input address
	scale_config.id = SCALE_PATH_INPUT_ADDR;
	//scale_address.yaddr = input_addr; //wxz:??? YUV422 or YUYV
	scale_address.yaddr = input_addr; 
	scale_address.uaddr = input_addr + trim_rect->w * trim_rect->h;
	//scale_address.uaddr = input_addr + output_width * output_height;//wxz:???
	scale_address.vaddr = scale_address.uaddr;
	scale_config.param = &scale_address;	 
	LOGV("INTERPOLATION:scale input y addr:0x%x,uv addr:0x%x.",scale_address.yaddr,scale_address.uaddr);
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set output address
	scale_config.id = SCALE_PATH_OUTPUT_ADDR;
	scale_address.yaddr = output_addr; 
	scale_address.uaddr = output_addr + output_width * output_height;
	scale_address.vaddr = scale_address.uaddr;//output_addr + output_width * output_height * 3 / 2;	
	scale_config.param = &scale_address;	 
	LOGV("INTERPOLATION:scale out  y addr:0x%x,uv addr:0x%x.",scale_address.yaddr,scale_address.uaddr);;
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	if(output_width > 640){
		//set slice mode
		scale_config.id = SCALE_PATH_SLICE_SCALE_EN;
		enable = 1;	
		scale_config.param = &enable;	 
		if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
		{
			LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
			return -1;
		}
		//set slice height
		scale_config.id = SCALE_PATH_SLICE_SCALE_HEIGHT;
		slice_height = SCALE_SLICE_HEIGHT;	
		scale_config.param = &slice_height;	 
		if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
		{
			LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
			return -1;
		}
		//set swap and line buffer address
		scale_config.id = SCALE_PATH_SWAP_BUFF;
		scale_address.yaddr = g_slice_swap_buf; // 384k bytes  //output_w * slice_h  the buff size is for 5M.
		scale_address.uaddr = scale_address.yaddr + 1024 * 384; // 384k bytes
		scale_address.vaddr = scale_address.uaddr + 1024 * 384;  // 256k bytes //output_w * 4
		scale_config.param = &scale_address;	 
		LOGV("INTERPOLATION:set scale config g_slice_swap_buf=0x%x,scale_address.uaddr=0x%x,scale_address.vaddr=0x%x.",
			    g_slice_swap_buf,scale_address.uaddr,scale_address.vaddr);
		if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
		{
			LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
			return -1;
		}		
	}
	//set input endian
	scale_config.id = SCALE_PATH_INPUT_ENDIAN;
	mode = 1;
	scale_config.param = &mode;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set output endian
	scale_config.id = SCALE_PATH_OUTPUT_ENDIAN;
	mode = 1;
	scale_config.param = &mode;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}	
	
	//done	 
	if (-1 == xioctl(fd, SCALE_IOC_DONE, 0))   
	{
		LOGE("Fail to SCALE_IOC_DONE");
		return -1;
	}

	if(-1 == close(fd))   
	{   
		LOGE("Fail to close scale device.");
        	return -1;   
   	 } 
    	fd = -1;   

	return 0;
}

static int camera_preview_zoom_colorformat(SCALE_DATA_FORMAT_E output_fmt, uint32_t output_width, uint32_t output_height, uint32_t output_addr, ZOOM_TRIM_RECT_T *trim_rect, uint32_t input_addr)
{
	static int fd = -1; 	
	SCALE_CONFIG_T scale_config;
	SCALE_SIZE_T scale_size;
	SCALE_RECT_T scale_rect;
	SCALE_ADDRESS_T scale_address;
	SCALE_DATA_FORMAT_E data_format;
	//uint32_t sub_sample_en;
	SCALE_MODE_E scale_mode;
	uint32_t mode;
	
	fd = open("/dev/sc8800g_scale", O_RDONLY);//O_RDWR /* required */, 0);  
	if (-1 == fd) 
	{   
		LOGE("Fail to open scale device.");
        	return -1;   
   	 }
    	
	//set mode
	scale_config.id = SCALE_PATH_MODE;	
	scale_mode = SCALE_MODE_SCALE;
	scale_config.param = &scale_mode;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set input data format
	scale_config.id = SCALE_PATH_INPUT_FORMAT;
	data_format = SCALE_DATA_YUV422;
	scale_config.param = &data_format;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set output data format
	scale_config.id = SCALE_PATH_OUTPUT_FORMAT;
	data_format = output_fmt;
	scale_config.param = &data_format;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set input size
	scale_config.id = SCALE_PATH_INPUT_SIZE;
	//scale_size.w = trim_rect->w;
	//scale_size.h = trim_rect->h;
	scale_size.w = output_width;
	scale_size.h = output_height;
	scale_config.param = &scale_size;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set output size
	scale_config.id = SCALE_PATH_OUTPUT_SIZE;
	scale_size.w = output_width;
	scale_size.h = output_height;
	scale_config.param = &scale_size;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
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
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set input address
	scale_config.id = SCALE_PATH_INPUT_ADDR;
	//scale_address.yaddr = input_addr; //wxz:??? YUV422 or YUYV
	scale_address.yaddr = input_addr; 
	//scale_address.uaddr = input_addr + trim_rect->w * trim_rect->h;
	scale_address.uaddr = input_addr + output_width * output_height;//wxz:???
	scale_address.vaddr = scale_address.uaddr;
	scale_config.param = &scale_address;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set output address
	scale_config.id = SCALE_PATH_OUTPUT_ADDR;
	scale_address.yaddr = output_addr; 
	scale_address.uaddr = output_addr + output_width * output_height;
	scale_address.vaddr = output_addr + output_width * output_height * 3 / 2;	
	scale_config.param = &scale_address;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set input endian
	scale_config.id = SCALE_PATH_INPUT_ENDIAN;
	mode = 1;
	scale_config.param = &mode;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
#if PREVIEW_ENDIAN_H	
	//set output endian
	scale_config.id = SCALE_PATH_OUTPUT_ENDIAN;
	mode = 1;
	scale_config.param = &mode;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}	
#endif 		
	//done	 
	if (-1 == xioctl(fd, SCALE_IOC_DONE, 0))   
	{
		LOGE("Fail to SCALE_IOC_DONE");
		return -1;
	}

	if(-1 == close(fd))   
	{   
		LOGE("Fail to close scale device.");
        	return -1;   
   	 } 
    	fd = -1;   

	return 0;
}
static int camera_rotation(uint32_t agree, uint32_t width, uint32_t height, uint32_t in_addr, uint32_t out_addr)
{
	int fd = -1; 
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

	
	
	fd = open("/dev/sc8800g_rotation", O_RDWR /* required */, 0); 
	if (-1 == fd) 
	{   
		LOGE("Fail to open rotation device.");
        	return -1;   
   	}
    //	else
    //  		LOGV("OK to open rotation device.");
	
	//done	 
	if (-1 == xioctl(fd, SC8800G_ROTATION_DONE, &rot_params))   
	{
		LOGE("Fail to SC8800G_ROTATION_DONE");
		return -1;
	}
	//else
	//	LOGE("OK to SC8800G_ROTATION_DONE.");

	if(-1 == close(fd))   
	{   
		LOGE("Fail to close rotation device.");
        	return -1;   
   	 } 
    	fd = -1;  

	return 0;
}

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
		LOGV("Stop preview for g_stop_preview_flag.");
		return 1;
	}
	else
	{
		//LOGV("Stop preview: g_stop_preview_flag is 0.");
		return 0;
	}	
}

void camera_zoom_picture_size(uint32_t in_w, uint32_t in_h, ZOOM_TRIM_RECT_T *trim_rect, uint32_t zoom_level)
{
	uint32_t trim_w, trim_h;

	switch(zoom_level)
	{
		case 0:
			trim_w = 0;
			trim_h = 0;
			break;
		case 1:
			trim_w = in_w >> 2;  // 1/4
			trim_h = in_h >> 2; // 1/4
			break;
		case 2:
			trim_w = in_w / 3; // 1/3
			trim_h = in_h / 3; // 1/3
			break;			
		case 3:
			trim_w = in_w * 3 >> 3;  // 3/8
			trim_h = in_h * 3 >> 3; // 3/8
			break;
		default:
			trim_w = 0;
			trim_h = 0;
			break;			
	}

	trim_rect->x = (trim_w + 3) & ~3;
	trim_rect->y = (trim_h + 3) & ~3;	
	trim_rect->w = W_H_ALIGNED(in_w - (trim_rect->x << 1));
	trim_rect->h = W_H_ALIGNED(in_h - (trim_rect->y << 1));
	//wxz20110728: the sccale range must be in [4, 1/4]. There need to handle the scale factor when the zoom level is 3.
	if(3 == zoom_level){
		if(trim_rect->w < (in_w >> 2)){
			trim_rect->w = W_H_ALIGNED(in_w >> 2);
		}
		if(trim_rect->h < (in_h >> 2)){
			trim_rect->h = W_H_ALIGNED(in_h >> 2);
		}
	}
	//LOGV("trim_rect{x,y,w,h} --{%d, %d, %d, %d}, in_w: %d, in_h: %d, zoom_level: %d.", trim_rect->x, trim_rect->y, trim_rect->w, trim_rect->h,in_w, in_h, zoom_level);
}

//convert the endian of the preview from half word endian to little endian by DMA.
//make the scaling output endian is half word endian; it's the same as dcam's.
static int convert_preview_endian_by_DMA(uint32_t width, uint32_t height, uint32_t out_address,  uint32_t in_address)
{
	SCALE_YUV420_ENDIAN_T yuv_config;
	int fd = -1; 
	
	fd = open("/dev/sc8800g_scale", O_RDONLY);
	if (-1 == fd) 
	{   
		LOGE("Fail to open scale device for preview DMA.");
        	return -1;   
   	 }
    	//else
      	//	LOGV("OK to open scale device for preview DMA.");

	yuv_config.width = width;
	yuv_config.height = height;	
	yuv_config.src_addr = in_address;
	yuv_config.dst_addr = out_address;
	
	if (-1 == xioctl(fd, SCALE_IOC_YUV420_ENDIAN, &yuv_config))   
	{
		LOGE("Fail to SCALE_IOC_YUV420_ENDIAN.");
		return -1;
	}	

	if(-1 == close(fd))   
	{   
		LOGE("Fail to close scale device for preview DMA.");
        	return -1;   
   	 } 
    	fd = -1;   

	return 0;	
}

camera_ret_code_type camera_start_preview_for_restart(camera_cb_f_type callback, void *client_data);
camera_ret_code_type camera_stop_preview_for_restart(void);
void *camera_restart_thread(void *client_data)
{
	g_preview_stop = 1;
	camera_stop_preview_for_restart();
	close_device();
	usleep(10000);
	open_device();
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
	nsecs_t timestamp;//_old, timestamp_new;
	nsecs_t timestamp_old, timestamp_new;
	int frame_delay = 45000000;
	int sleep_time = 0;
	uint32_t old_w = 0, old_h = 0;
	if((1 == g_cam_params.orientation) && ((90 == g_rotation) || (270 == g_rotation) || (180 == g_rotation))){
		old_w = g_dcam_dimensions.display_width;
		old_h = g_dcam_dimensions.display_height;
	}
	LOGV("===============Start preview thread=====================.");
	while(1){
		//timestamp_old = systemTime(); 
		//timestamp = systemTime(); 
		//LOGE("###wxz: before dqbuf: %lld.", timestamp_old);
		if(1 == g_cam_params.orientation){
			if((90 == g_rotation) || (270 == g_rotation)){
				g_dcam_dimensions.display_width = old_h;
				g_dcam_dimensions.display_height = old_w;
			}
			else if(180 == g_rotation){
				g_dcam_dimensions.display_width = old_w;
				g_dcam_dimensions.display_height = old_h;
			}
		}
		CLEAR (buf);   
        	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;   
        	buf.memory = V4L2_MEMORY_USERPTR;
        	if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) 
		{
			if(1 == check_stop()){
				return NULL;
			}
			LOGE("Fail to VIDIOC_DQBUF.");			
			//create the thread for restart
			if(0 != (ret = pthread_create(&g_restart_thr, NULL, camera_restart_thread, client_data)))
			{
				LOGE("Fail to careate restart thread in preview mode.");
				return NULL;
			}	
			else
			{				
				LOGV("OK to careate restart thread in preview mode.");
			}				
			LOGV("===============Stop preview thread because the dqbuf failed.=====================.");
			return NULL;	
			//usleep(10000);
			continue;
		} 
		else
		{			
			LOGV("OK to VIDIOC_DQBUF.buf.index: %d. userptr: %lx", buf.index, buf.m.userptr);
		}
    		for (i = G_PREVIEW_BUF_OFFSET; i < n_buffers; ++i)   
     		{
  		     	if (buf.m.userptr == (unsigned long) g_buffers[i].phys_addr)
			{	
				g_releasebuff_index = i;
				//LOGV("dqbuf g_releasebuff_index: %d.buf.index: %d.userptr: %lx", g_releasebuff_index, buf.index,buf.m.userptr);
				//LOGV("OK to VIDIOC_DQBUF.buf.index: %d. userptr: %lx", buf.index, buf.m.userptr);
                		break;   
			}
    		 }
   	     	if(i < n_buffers)
		{ 
#ifdef PREVIEW_BUF_3_2		
get_buff:		
			if(0 == g_buf_flag[0])
			{
				output_addr = g_buffers[0].phys_addr;
				frame_type.buf_Virt_Addr = g_buffers[0].virt_addr;
				g_buf_flag[0] = 1;
				if(1 == g_buf_flag[1])
					g_buf_flag[2] = 1;
			}
			else if(0 == g_buf_flag[1])
			{
				output_addr = g_buffers[1].phys_addr;
				frame_type.buf_Virt_Addr = g_buffers[1].virt_addr;
				g_buf_flag[1] = 1;
				if(1 == g_buf_flag[0])
					g_buf_flag[2] = 0;				
			}	
			else
			{
				usleep(5000);
				LOGV("wait buff. flag: %d, %d, %d.", g_buf_flag[0], g_buf_flag[1], g_buf_flag[2]  );
				if(1 == check_stop())
					return NULL;
				goto get_buff;
			}	
#endif //PREVIEW_BUF_3_2
			camera_zoom_picture_size(g_dcam_dimensions.display_width, g_dcam_dimensions.display_height, &trim_rect, g_hal_zoom_level);
   	     #if 0 
			{
				FILE *fp = NULL;
				static uint32_t first = 1;
				if(1 == first)
				{
				//LOGE("preview yuv422: width: %d, hei: %d.", g_dcam_dimensions.display_width, g_dcam_dimensions.display_height);
				LOGE("preview yuv422: width: %d, hei: %d.", trim_rect.w, trim_rect.h);
				fp = fopen("/data/out_pre_yuv422.yuv", "wb");
				//fwrite(g_buffers[g_releasebuff_index].virt_addr, 1, g_dcam_dimensions.display_width * g_dcam_dimensions.display_height * 2, fp);
				fwrite(g_buffers[g_releasebuff_index].virt_addr, 1, trim_rect.w*trim_rect.h * 2, fp);
				fclose(fp);
				first = 0;
				}
			}		
		 #endif			
			{
				//convert color space from yuv422 to yuv420 and do scaling if need.
#ifdef PREVIEW_BUF_3_2			
				ret = camera_preview_zoom_colorformat(SCALE_DATA_YUV420, g_dcam_dimensions.display_width, g_dcam_dimensions.display_height, output_addr, 
					&trim_rect, g_buffers[g_releasebuff_index].phys_addr);	
#else				
				ret = camera_preview_zoom_colorformat(SCALE_DATA_YUV420, g_dcam_dimensions.display_width, g_dcam_dimensions.display_height, g_buffers[g_preview_buffer_num - 1].phys_addr, 				
					&trim_rect, g_buffers[g_releasebuff_index].phys_addr);
#endif //PREVIEW_BUF_3_2
				if(0 != ret){
					LOGE("Fail to preview because camera_preview_zoom_colorformat");
					g_callback(CAMERA_EXIT_CB_FAILED, client_data, CAMERA_FUNC_START_PREVIEW, NULL);
					camera_release_frame(g_releasebuff_index);
					if(1 == check_stop())
						return NULL;						
					continue;
				}
			
   	     #if 0
			{
				FILE *fp = NULL;
				static uint32_t first = 1;
				if(10 == first)
				{
				LOGE("preview yuv420: width: %d, hei: %d, addrs: 0x%x.", g_dcam_dimensions.display_width, g_dcam_dimensions.display_height, (uint32_t)frame_type.buf_Virt_Addr);
				fp = fopen("/data/out_pre_yuv420_dma.yuv", "wb");
				fwrite(frame_type.buf_Virt_Addr, 1, g_dcam_dimensions.display_width * g_dcam_dimensions.display_height * 3 / 2, fp);
				fclose(fp);				
				}
				first++;
			}		
		 #endif		
#if  PREVIEW_ENDIAN_H		 
				//convert the yuv420 from half word endian to little endian by DMA							
				if((1 == g_cam_params.orientation) && ((90 == g_rotation) || (270 == g_rotation) || (180 == g_rotation))){
					ret = convert_preview_endian_by_DMA(g_dcam_dimensions.display_width, g_dcam_dimensions.display_height,g_buffers[g_preview_buffer_num -2].phys_addr, g_buffers[g_preview_buffer_num - 1].phys_addr);
				}
				else{
#if USE_TWO_KIND_BUF
					ret = convert_preview_endian_by_DMA(g_dcam_dimensions.display_width, g_dcam_dimensions.display_height,g_buffers_for_display[g_releasebuff_index].phys_addr, g_buffers[g_preview_buffer_num - 1].phys_addr);
#else
					ret = convert_preview_endian_by_DMA(g_dcam_dimensions.display_width, g_dcam_dimensions.display_height,g_buffers[g_releasebuff_index].phys_addr, g_buffers[g_preview_buffer_num - 1].phys_addr);
#endif
				}
				//frame_type.buf_id = g_releasebuff_index;
				if(0 != ret){
					LOGE("Fail to preview because convert_preview_endian_by_DMA");
					g_callback(CAMERA_EXIT_CB_FAILED, client_data, CAMERA_FUNC_START_PREVIEW, NULL);
					camera_release_frame(g_releasebuff_index);
					if(1 == check_stop())
						return NULL;						
					continue;
				}
				//wxz20110725: add the rotation function.
				if((1 == g_cam_params.orientation) && ((90 == g_rotation) || (270 == g_rotation) || (180 == g_rotation))){
#if USE_TWO_KIND_BUF
					ret = camera_rotation(g_rotation, g_dcam_dimensions.display_width, g_dcam_dimensions.display_height, g_buffers[g_preview_buffer_num -2].phys_addr, g_buffers_for_display[g_releasebuff_index].phys_addr);

#else
					ret = camera_rotation(g_rotation, g_dcam_dimensions.display_width, g_dcam_dimensions.display_height, g_buffers[g_preview_buffer_num -2].phys_addr, g_buffers[g_releasebuff_index].phys_addr);
#endif
					if(0 != ret){
						LOGE("Fail to preview because camera_rotation");
						g_callback(CAMERA_EXIT_CB_FAILED, client_data, CAMERA_FUNC_START_PREVIEW, NULL);
						camera_release_frame(g_releasebuff_index);
						if(1 == check_stop())
							return NULL;						
						continue;
					}
					g_dcam_dimensions.display_width = old_w;
					g_dcam_dimensions.display_height = old_h;
				}
#if 0
				{
					uint32_t *in_buf = NULL;
					uint32_t *out_buf = NULL;
					uint32_t i, temp;
					in_buf = g_buffers[g_preview_buffer_num - 1].virt_addr;
					out_buf = g_buffers[g_releasebuff_index].virt_addr;
					for(i = 0; i < g_dcam_dimensions.display_width * g_dcam_dimensions.display_height * 3 / 2 / 4; i++){
						temp = *in_buf++;
						temp = ((temp & 0x000000FF) << 8) | ((temp & 0x000FF00) >> 8) | ((temp & 0x00FF0000) << 8) | ((temp & 0xFF000000) >> 8);
						*out_buf++ = temp;
					}	
				}
#endif				
#endif
		 	}
#if USE_TWO_KIND_BUF
			frame_type.buf_Virt_Addr  = g_buffers_for_display[g_releasebuff_index].virt_addr;
#else
			frame_type.buf_Virt_Addr  = g_buffers[g_releasebuff_index].virt_addr;
#endif
   	     #if 0 
			{
				FILE *fp = NULL;
				static uint32_t first = 1;
				if(10 == first)
				{
				LOGE("preview yuv420: width: %d, hei: %d, addrs: 0x%x.", g_dcam_dimensions.display_width, g_dcam_dimensions.display_height, (uint32_t)frame_type.buf_Virt_Addr);
				fp = fopen("/data/out_pre_yuv420.yuv", "wb");
				fwrite(frame_type.buf_Virt_Addr, 1, g_dcam_dimensions.display_width * g_dcam_dimensions.display_height * 3 / 2, fp);
				fclose(fp);				
				}
				first++;
			}		
		 #endif		 	
			//wxz: send the buffer to surface for display.		
			frame_type.buf_id = g_releasebuff_index;
			frame_type.dx = g_dcam_dimensions.display_width;
			frame_type.dy = g_dcam_dimensions.display_height;	
			frame_type.format = CAMERA_YCBCR_4_2_0;//CAMERA_YCBCR_4_2_2;//CAMERA_RGB565;				
			g_callback(CAMERA_EVT_CB_FRAME, client_data, CAMERA_FUNC_START_PREVIEW, (uint32_t)&frame_type);			
         		//LOGV("OK to send one frame for preview.g_releasebuff_index: %d, frame_type.vaddr: 0x%x.", g_releasebuff_index, (uint32_t)frame_type.buf_Virt_Addr);
   	     	}
		else
			LOGE("Fail to DQBUF:userptr: %lx, len: %x.i = %d.", buf.m.userptr, buf.length,i);
		if(1 == check_stop())
			return NULL;
		/*timestamp_new = systemTime(); 
		if((timestamp_old + frame_delay) > timestamp_new){
			sleep_time = (timestamp_old + frame_delay - timestamp_new) / 1000;
			//usleep(sleep_time);	
			LOGV("wait sleep: %ld.", sleep_time);
		}*/
	}

	LOGV("===============Stop preview thread=====================.");
	return NULL;
}

int camera_preview_init(void)
{
	struct v4l2_format fmt;
	uint32_t min;
	struct v4l2_requestbuffers req;
	uint32_t buffer_size, page_size, frame_size;
	struct v4l2_streamparm streamparm;	
	
	CLEAR (fmt);   
    	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;   
	fmt.fmt.pix.width = g_dcam_dimensions.display_width;   
    	fmt.fmt.pix.height = g_dcam_dimensions.display_height;   
    //V4L2_PIX_FMT_YVU420, V4L2_PIX_FMT_YUV420 ！ Planar formats with 1/2 horizontal and vertical chroma resolution, also known as YUV 4:2:0   
    //V4L2_PIX_FMT_YUYV ！ Packed format with 1/2 horizontal chroma resolution, also known as YUV 4:2:2   
    //wxz: pixelformat support: V4L2_PIX_FMT_YUV420 and V4L2_PIX_FMT_RGB24;
    	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV422P; //V4L2_PIX_FMT_RGB32;//V4L2_PIX_FMT_RGB565X, //V4L2_PIX_FMT_YUYV; //wxz:TOOD
    	fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;   
    	{   
        	LOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");   
        	LOGV("=====will set fmt to (%d, %d)--", fmt.fmt.pix.width,fmt.fmt.pix.height);   
	        if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) {   
        	    LOGV("V4L2_PIX_FMT_YUYV\n");   
	        } else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420) {   
        	    LOGV("V4L2_PIX_FMT_YUV420\n");   
	        } else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV422P) {   
        	    LOGV("V4L2_PIX_FMT_YUV422P\n");   
	        }  else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB32) {   
        	    LOGV("V4L2_PIX_FMT_RGB32\n");           	
	        }  else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB565X) {   
        	    LOGV("V4L2_PIX_FMT_RGB565X\n");   
        	}
    	}   
    	if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))   
	{
		LOGE("Fail to VIDIOC_S_FMT.");
		return -1;
	}
    	{   
        	LOGV("=====after set fmt\n");   
	        LOGV("    fmt.fmt.pix.width = %d\n", fmt.fmt.pix.width);   
	        LOGV("    fmt.fmt.pix.height = %d\n", fmt.fmt.pix.height);   
	        LOGV("    fmt.fmt.pix.sizeimage = %d\n", fmt.fmt.pix.sizeimage);   
	        g_preview_size = fmt.fmt.pix.sizeimage;   
	        LOGV("    fmt.fmt.pix.bytesperline = %d\n", fmt.fmt.pix.bytesperline);   
        	LOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");   
	        LOGV("\n");   
	}   
	g_preview_size = fmt.fmt.pix.sizeimage;   
    /* Note VIDIOC_S_FMT may change width and height. */   
   	 LOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");   
    /* Buggy driver paranoia. */   
    	min = fmt.fmt.pix.width * 2;   
    	if (fmt.fmt.pix.bytesperline < min)   
        	fmt.fmt.pix.bytesperline = min;   
    	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;   
    	if (fmt.fmt.pix.sizeimage < min)   
        	fmt.fmt.pix.sizeimage = min;   
    LOGV("After Buggy driver paranoia\n");   
    LOGV("    >>fmt.fmt.pix.sizeimage = %d\n", fmt.fmt.pix.sizeimage);   
    LOGV("    >>fmt.fmt.pix.bytesperline = %d\n", fmt.fmt.pix.bytesperline);   
    LOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");   
    LOGV("\n");   
	CLEAR (req);   
	req.count = g_preview_buffer_num;   
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;   
    	req.memory = V4L2_MEMORY_USERPTR;   
    	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) 
	{
		LOGE("Fail to VIDIOC_REQBUFS.");
		return -1;
	}
	
	g_buffers = (buffer *)calloc(g_preview_buffer_num, sizeof(*g_buffers));   
	if (!g_buffers)    
	{
		LOGE("Fail to malloc preview buffer struct.");
		return -1;
	}
	frame_size = g_dcam_dimensions.display_width * g_dcam_dimensions.display_height * 2; //for YUYV
	buffer_size = camera_get_size_align_page(frame_size);
    	for (n_buffers = 0; n_buffers < g_preview_buffer_num; ++n_buffers) 
	{
        	g_buffers[n_buffers].length = buffer_size; 
		g_buffers[n_buffers].virt_addr = g_preview_virt_addr + n_buffers * buffer_size / 4;			
		g_buffers[n_buffers].phys_addr = g_preview_phys_addr + n_buffers * buffer_size;
		//memset(g_buffers[n_buffers].virt_addr, 0x0, buffer_size);//wxz:???
		LOGV("preview: g_buffer[%d]: virt: %x, phys: %x, len: %x.",n_buffers,(uint32_t)g_buffers[n_buffers].virt_addr,g_buffers[n_buffers].phys_addr,g_buffers[n_buffers].length);
        }  
#if USE_TWO_KIND_BUF
	g_buffers_for_display = (buffer *)calloc(g_preview_buffer_num_for_display, sizeof(*g_buffers_for_display));   
	if (!g_buffers_for_display)    
	{
		LOGE("Fail to malloc preview buffer struct for display.");
		return -1;
	}
    	for (n_buffers = 0; n_buffers < g_preview_buffer_num_for_display; ++n_buffers) 
	{
        	g_buffers_for_display[n_buffers].length = buffer_size; 
		g_buffers_for_display[n_buffers].virt_addr = g_preview_virt_addr_for_display + n_buffers * buffer_size / 4;			
		g_buffers_for_display[n_buffers].phys_addr = g_preview_phys_addr_for_display + n_buffers * buffer_size;		
		LOGV("preview_for_display: g_buffer_for_display[%d]: virt: %x, phys: %x, len: %x.",n_buffers,(uint32_t)g_buffers_for_display[n_buffers].virt_addr,g_buffers_for_display[n_buffers].phys_addr,g_buffers_for_display[n_buffers].length);
        } 
#endif 
        //wxz20111021: raw_data
        //BYTE0: 0: HAL doesn't set sensor_id; 1: HAL sets sensor_id;
        //BYTE1: 0: back camera if BYTE0 is 1; 1: front camera if BYTE0 is 1;
        //BYTE2: 0: no crop and rotation; 1: crop and rotation;
        //BYTE3: 0: HAL doesn't set sensor number; 1: HAL sets sensor number; 2: need to update the sensor number;
	//BYTE4: back sensor number if BYTE3 is 1; If the number is right, use it; not right, check all sensors and change the number; 0xFF is bad sensor;
	//BYTE5: front sensor number if BYTE3 is 1; If the number is right, use it; not right, check all sensors and change the number; 0xFF is bad sensor;
	CLEAR (streamparm);   
	streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	//streamparm.parm.capture.capturemode = 0;
	streamparm.parm.raw_data[6] = 0;
	if(-1 == g_camera_id)
	{
		streamparm.parm.raw_data[0] = 0;
	}
	else
	{
		streamparm.parm.raw_data[0] = 1;
		streamparm.parm.raw_data[1] = g_camera_id;
	}
	g_rotation = g_rotation_parm; 
	g_cam_params.orientation = g_cam_params.orientation_parm;
	if((1 == g_cam_params.orientation) && ((90 == g_rotation) || (270 == g_rotation))){
		streamparm.parm.raw_data[2] = 1;
	}
	else{
		streamparm.parm.raw_data[2] = 0;
	}
#if HAL_SET_SENSOR_ID
	//wxz20111021: set sensor number
	streamparm.parm.raw_data[3] = 1;
	streamparm.parm.raw_data[4] = camera_get_sensor_number(BACK_CAMERA_ID);//back camera
	streamparm.parm.raw_data[5] = camera_get_sensor_number(FRONT_CAMERA_ID);//front camera
#endif
	if (-1 == xioctl(fd, VIDIOC_S_PARM, &streamparm)) 
	{
		LOGE("preview: Fail to VIDIOC_S_PARM.");
		return -1;
	}
#if HAL_SET_SENSOR_ID
	//need to update the sensor number.
	LOGV("camera_preview_init: data1: %d, data3: %d, data4: %d, data5: %d.", streamparm.parm.raw_data[1], streamparm.parm.raw_data[3], streamparm.parm.raw_data[4], streamparm.parm.raw_data[5]);
	if(2 == streamparm.parm.raw_data[3]){
		if(BACK_CAMERA_ID == streamparm.parm.raw_data[1]){
			camera_set_sensor_number(BACK_CAMERA_ID, streamparm.parm.raw_data[4]);
		}
		else if(FRONT_CAMERA_ID == streamparm.parm.raw_data[1]){
			camera_set_sensor_number(FRONT_CAMERA_ID, streamparm.parm.raw_data[5]);
		}
	}
#endif
	g_preview_stop = 0;
#if FRONT_CAMERA_MIRROR
	//wxz20111112: disable the front camera mirror funciton in preview mode
	if(1 == g_camera_id){ //for front camera
		uint32_t enable = 0;
		if((90 == g_cam_params.rotation_degree) || (270 == g_cam_params.rotation_degree)){ //for portrait
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
	
	//for (i = 0; i < g_preview_buffer_num; ++i) 
	//for (i = G_PREVIEW_BUF_OFFSET; i < g_preview_buffer_num; ++i) 
	for (i = G_PREVIEW_BUF_OFFSET; i < g_preview_buffer_num - 2; ++i) 		
	{   
            struct v4l2_buffer buf;   
            CLEAR (buf);   
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;   
            buf.memory = V4L2_MEMORY_USERPTR;   
            buf.index = i;   
            buf.m.userptr = g_buffers[i].phys_addr;               
            buf.length = g_buffers[i].length;   	
	    LOGV("####preview QBuf: buffers[%d].start: %lx, %x\n", buf.index, buf.m.userptr,buf.length);	
            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))   
	    {
		LOGE("Fail to VIDIOC_QBUF from camera_preview_qbuffers.");
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
		LOGE("Fail to VIDIOC_STREAMON.");
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


	//record the callback function
	g_callback = callback;

	//get the preview memory
	frame_size = g_dcam_dimensions.display_width * g_dcam_dimensions.display_height * 2; //for YUV422
	buffer_size = camera_get_size_align_page(frame_size);
	size = buffer_size * g_preview_buffer_num;
#if USE_TWO_KIND_BUF
	if(NULL == (g_preview_virt_addr = (uint32_t *)g_dcam_obj->get_preview_mem_for_HW(size,&g_preview_phys_addr,0)))
	{
		LOGE("Fail to malloc preview memory.");
		return CAMERA_FAILED;
	}
	size = buffer_size * g_preview_buffer_num_for_display;
	if(NULL == (g_preview_virt_addr_for_display = (uint32_t *)g_dcam_obj->get_preview_mem(size,&g_preview_phys_addr_for_display,0)))
	{
		LOGE("Fail to get preview memory for display.");
		return CAMERA_FAILED;
	}
#else
	if(NULL == (g_preview_virt_addr = (uint32_t *)g_dcam_obj->get_preview_mem(size,&g_preview_phys_addr,0)))
	{
		LOGE("Fail to malloc preview memory.");
		return CAMERA_FAILED;
	}
#endif
	if(0 != camera_preview_init())
	{
		LOGE("Fail to init preview mode.");
		return CAMERA_FAILED;
	}   
	LOGV("OK  to camera_preview_init.");

	if(0 != camera_preview_qbuffers())
	{
		LOGE("Fail to qbuffers for preview.");
		return CAMERA_FAILED;
	}
	LOGV("OK  to camera_preview_qbuffers.");

	if(0 != camera_preview_streamon())
	{
		LOGE("Fail to stream on for preview.");
		return CAMERA_FAILED;
	}
	LOGV("OK  to camera_preview_streamon.");

	//update the status
	callback(CAMERA_RSP_CB_SUCCESS, g_dcam_obj, CAMERA_FUNC_START_PREVIEW, 0);
	LOGV("OK to CAMERA_FUNC_START_PREVIEW.");

	//create the thread for preview
	if(0 != (ret = pthread_create(&g_preview_thr, NULL, camera_preview_thread, client_data)))
	{
		LOGE("Fail to careate thread in preview mode.");
		return CAMERA_FAILED;
	}	
	else
	{
		//pthread_join(g_preview_thr, NULL);
		LOGV("OK to careate thread in preview mode.");
	}

	return ret_type;
}

camera_ret_code_type camera_start_preview_for_restart(camera_cb_f_type callback, void *client_data)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;
	g_dcam_obj = (SprdCameraHardware *)client_data;
	int ret;	
	uint32_t size, page_size, buffer_size,frame_size;


	//record the callback function
	//g_callback = callback;

	//get the preview memory
	frame_size = g_dcam_dimensions.display_width * g_dcam_dimensions.display_height * 2; //for YUV422
	buffer_size = camera_get_size_align_page(frame_size);
	size = buffer_size * g_preview_buffer_num;
#if USE_TWO_KIND_BUF
	if(NULL == (g_preview_virt_addr = (uint32_t *)g_dcam_obj->get_preview_mem_for_HW(size,&g_preview_phys_addr,0)))
	{
		LOGE("Fail to malloc preview memory.");
		return CAMERA_FAILED;
	}
	size = buffer_size * g_preview_buffer_num_for_display;
	if(NULL == (g_preview_virt_addr_for_display = (uint32_t *)g_dcam_obj->get_preview_mem(size,&g_preview_phys_addr_for_display,0)))
	{
		LOGE("Fail to get preview memory for display.");
		return CAMERA_FAILED;
	}
#else
	if(NULL == (g_preview_virt_addr = (uint32_t *)g_dcam_obj->get_preview_mem(size,&g_preview_phys_addr,0)))
	{
		LOGE("Fail to malloc preview memory.");
		return CAMERA_FAILED;
	}
#endif
	if(0 != camera_preview_init())
	{
		LOGE("Fail to init preview mode.");
		return CAMERA_FAILED;
	}   
	LOGV("OK  to camera_preview_init.");

	if(0 != camera_preview_qbuffers())
	{
		LOGE("Fail to qbuffers for preview.");
		return CAMERA_FAILED;
	}
	LOGV("OK  to camera_preview_qbuffers.");

	if(0 != camera_preview_streamon())
	{
		LOGE("Fail to stream on for preview.");
		return CAMERA_FAILED;
	}
	LOGV("OK  to camera_preview_streamon.");

	//update the status
	//callback(CAMERA_RSP_CB_SUCCESS, g_dcam_obj, CAMERA_FUNC_START_PREVIEW, 0);
	LOGV("OK to CAMERA_FUNC_START_PREVIEW.");

	//create the thread for preview
	if(0 != (ret = pthread_create(&g_preview_thr, NULL, camera_preview_thread, client_data)))
	{
		LOGE("Fail to careate thread in preview mode.");
		return CAMERA_FAILED;
	}	
	else
	{
		//pthread_join(g_preview_thr, NULL);
		LOGV("OK to careate thread in preview mode.");
	}

	return ret_type;
}

camera_ret_code_type camera_start_focus (
        camera_focus_e_type focus,
        camera_cb_f_type callback,
        void *client_data)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;

	//start focus
	//TODO
	if(CAMERA_AUTO_FOCUS == focus)
	{
		callback(CAMERA_EXIT_CB_DONE, client_data, CAMERA_FUNC_START_FOCUS, 0);		
	}
	else if(CAMERA_MANUAL_FOCUS == focus)
	{
	}
	else
		ret_type = CAMERA_INVALID_PARM;
	
	return ret_type;
}
camera_ret_code_type camera_stop_focus (void)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;

	//stop focus
	//TODO	
	
	return ret_type;
}

camera_ret_code_type camera_stop(
        camera_cb_f_type callback,
        void *client_data)
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
		LOGE("Fail to VIDIOC_STREAMOFF.");
		return -1;
	 }

	return 0;
}
camera_ret_code_type camera_stop_preview(void)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;
	uint32_t num = 0;
	LOGV("camera_stop_preview: Start to stop preview thread.");
	//wxz:???
	//get lock	
	//stop camera preview thread
	g_stop_preview_flag = 1;	
	if(0 != camera_preview_streamoff())
	{
		LOGE("Fail to stream off for preview.");
		return CAMERA_FAILED;
	}
	LOGV("OK to stream off for preview.");	
	g_callback(CAMERA_RSP_CB_SUCCESS, g_dcam_obj, CAMERA_FUNC_STOP_PREVIEW, 0);
	LOGV("OK to stream off for CAMERA_FUNC_STOP_PREVIEW.");
	while(!g_preview_stop)
	{
		//usleep(100000);
		usleep(10000);
		num++;
		LOGV("waiting the preview stop....");
		if(num >= 100){
			pthread_kill(g_preview_thr, 9);
			LOGE("Force stop the preview!");
			break;
		}
	} //wait preview thread stop
	g_stop_preview_flag = 0;	
#if USE_TWO_KIND_BUF
	g_dcam_obj->free_preview_mem_for_HW();
#endif
	LOGV("camera_stop_preview: OK to stop preview thread.");
	//udpate the camera statsu
	//g_callback(CAMERA_RSP_CB_SUCCESS, g_dcam_obj, CAMERA_FUNC_STOP_PREVIEW, 0);
	//LOGV("OK to stream off for CAMERA_FUNC_STOP_PREVIEW.");
	/*if(0 != camera_preview_streamoff())
	{
		LOGE("Fail to stream off for preview.");
		return CAMERA_FAILED;
	}
	LOGV("OK to stream off for preview.");	
	g_callback(CAMERA_RSP_CB_SUCCESS, g_dcam_obj, CAMERA_FUNC_STOP_PREVIEW, 0);
	LOGV("OK to stream off for CAMERA_FUNC_STOP_PREVIEW.");
	*/
	//wxz:???
	//release lock	
	
	return ret_type;
}
camera_ret_code_type camera_stop_preview_for_restart(void)
{
	camera_ret_code_type ret_type = CAMERA_SUCCESS;
	uint32_t num = 0;
	LOGV("camera_stop_preview: Start to stop preview thread.");
	
	//stop camera preview thread
	g_stop_preview_flag = 1;	
	if(0 != camera_preview_streamoff())
	{
		LOGE("Fail to stream off for preview.");
		return CAMERA_FAILED;
	}
	LOGV("OK to stream off for preview.");	
	//g_callback(CAMERA_RSP_CB_SUCCESS, g_dcam_obj, CAMERA_FUNC_STOP_PREVIEW, 0);
	LOGV("OK to stream off for CAMERA_FUNC_STOP_PREVIEW.");
	while(!g_preview_stop)
	{
		//usleep(100000);
		usleep(10000);
		num++;
		LOGV("waiting the preview stop....");
		if(num >= 100){
			pthread_kill(g_preview_thr, 9);
			LOGE("Force stop the preview!");
			break;
		}
	} //wait preview thread stop
	g_stop_preview_flag = 0;
#if USE_TWO_KIND_BUF
	g_dcam_obj->free_preview_mem_for_HW();
#endif
	LOGV("camera_stop_preview: OK to stop preview thread.");
	
	return ret_type;
}


int camera_capture_streamoff(void)
{
	enum v4l2_buf_type type;
	
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;   
        if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))   
	 {
		LOGE("Fail to VIDIOC_STREAMOFF.");
		return -1;
	 }

	return 0;
}

void camera_capture_yuv422to420(uint32_t output_width, uint32_t output_height,  uint32_t output_addr, uint32_t input_addr)
{
	uint32_t size = output_width*output_height;
	uint8_t *src_y_addr = (uint8_t*)input_addr;
	uint8_t *src_uv_addr  = src_y_addr+size;
	uint8_t *dst_y_addr = (uint8_t*)output_addr;
	uint8_t *dst_uv_addr = (uint8_t*)dst_y_addr+size;
	uint32_t i = 0;
	uint32_t row_num = output_height>>1;
	
	memcpy(dst_y_addr,src_y_addr,size);

	src_uv_addr += output_width<<1;
	for( i=0 ; i<row_num; i++)
	{
		memcpy(dst_uv_addr,src_uv_addr,output_width);
		dst_uv_addr += output_width;
		src_uv_addr += output_width<<1;
	}
	
}
void *camera_capture_thread(void *client_data)
{
	struct v4l2_buffer buf;
	uint32_t i, ret = 0;
	camera_frame_type frame_type;
	struct zoom_trim_rect trim_rect;
	uint32_t old_w = 0, old_h = 0;
	
	if((1 == g_cam_params.orientation) && ((90 == g_rotation) || (270 == g_rotation)))
	{
		old_w = g_dcam_dimensions.picture_width;
		old_h = g_dcam_dimensions.picture_height;
		g_dcam_dimensions.picture_width = old_h;
		g_dcam_dimensions.picture_height = old_w;
	}
	if(1 == g_cam_params.is_2M_to_3M)
	{//wxz20110825: when 2M_to_3M, will not do rotation. so we can use the old_w and old_h.
		old_w = g_dcam_dimensions.picture_width;
		old_h = g_dcam_dimensions.picture_height;
		//g_dcam_dimensions.picture_width = DCAM_MAX_W;
		//g_dcam_dimensions.picture_height = DCAM_MAX_H;
		g_dcam_dimensions.picture_width = g_max_w;
		g_dcam_dimensions.picture_height = g_max_h;
	}
	LOGV("OK to camera_capture_thread.");
	CLEAR (buf);   
       	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;   
       	buf.memory = V4L2_MEMORY_USERPTR;   
	
       	if (0 != xioctl(fd, VIDIOC_DQBUF, &buf)) 
	{
		LOGE("Fail to VIDIOC_DQBUF.");	
		if(0 != camera_capture_streamoff())
		{
			LOGE("Fail to stream off for capture.");
			return NULL;
		}		
		g_callback(CAMERA_EXIT_CB_BAD_BUFF, client_data, CAMERA_FUNC_TAKE_PICTURE, NULL);		
		return NULL;
	}  	
	else    
	{	
		if(0 != camera_capture_streamoff())
		{
			LOGE("Fail to stream off for capture.");
			return NULL;
		}
		g_callback(CAMERA_RSP_CB_SUCCESS, client_data, CAMERA_FUNC_TAKE_PICTURE, 0);
		LOGV("OK to capture VIDIOC_DQBUF.buf.index: %d. userptr: %lx", buf.index, buf.m.userptr);
	}
	g_releasebuff_index = 0;
	if(0 != g_hal_zoom_level)
	{	
		if((1 == g_cam_params.orientation) && ((90 == g_rotation) || (270 == g_rotation)))
		{
			uint32_t temp, tmp_w;
			tmp_w = (((old_h * 3 >> 2) + 3) >> 2) << 2; //wxz20110815: the w = h*3/4;
			camera_zoom_picture_size(tmp_w, old_h, &trim_rect, g_hal_zoom_level);			
		}
		else
		{
			camera_zoom_picture_size(g_dcam_dimensions.picture_width, g_dcam_dimensions.picture_height, &trim_rect, g_hal_zoom_level);			
		}
	   		
		LOGV("before camera_cap_zoom_colorformat.");
		//scale up the picture to the encode size.
		if((1 == g_cam_params.orientation) && ((90 == g_rotation) || (270 == g_rotation)) && (3 == g_hal_zoom_level))
		{
			uint32_t tmp_w = (((old_h * 3 >> 2) + 3) >> 2) << 2;	
			uint32_t size = tmp_w * old_h * 2;
			uint32_t phys_addr;
			uint32_t *virt_addr;
			struct zoom_trim_rect trim_rect_new;
			if(NULL == (virt_addr = (uint32_t *)g_dcam_obj->get_temp_mem_by_HW(size, 1, &phys_addr)))
			{
				LOGE("Fail to malloc capture temp memory for zoom, size: 0x%x.", size);
				goto TAKE_PIC_FAIL;
			}
		         ret = camera_cap_zoom_colorformat(SCALE_DATA_YUV422, tmp_w, old_h, phys_addr,&trim_rect, g_cap_zoom_buf_phy_addr, SCALE_DATA_YUV422);
			if(0 != ret)
			{
				g_dcam_obj->free_temp_mem_by_HW();
				LOGE("Fail to take picture because the camera_cap_zoom_colorformat.");
				goto TAKE_PIC_FAIL;
			}
			trim_rect_new.x = 0;
			trim_rect_new.y = 0;
			trim_rect_new.w = tmp_w;
			trim_rect_new.h = old_h;
			ret = camera_cap_zoom_colorformat(SCALE_DATA_YUV422, g_dcam_dimensions.picture_width, g_dcam_dimensions.picture_height, g_buffers[g_releasebuff_index].phys_addr,&trim_rect_new, phys_addr, SCALE_DATA_YUV422);
                            if(0 != ret)
			{
				g_dcam_obj->free_temp_mem_by_HW();
				LOGE("Fail to take picture because the camera_cap_zoom_colorformat.");
				goto TAKE_PIC_FAIL;
			}
			g_dcam_obj->free_temp_mem_by_HW();				
		}
		else
		{
#if 0
			{
			FILE *fp = NULL;
			LOGE("INTERPOLATION:dcam output data! .");
			fp = fopen("/data/dcam_yuv422_1M.yuv", "wb");
			fwrite((uint32_t*)g_cap_zoom_buf_vir_addr, 1,1600*1200 * 2, fp);
			fclose(fp);  
			}
#endif			
			ret = camera_cap_zoom_colorformat(SCALE_DATA_YUV422, g_dcam_dimensions.picture_width, g_dcam_dimensions.picture_height, g_buffers[g_releasebuff_index].phys_addr,&trim_rect, g_cap_zoom_buf_phy_addr, SCALE_DATA_YUV422);
			if(0 != ret)
			{
				LOGE("Fail to take picture because the camera_cap_zoom_colorformat.");
				goto TAKE_PIC_FAIL;
			}			
		}			
		LOGV("after camera_cap_zoom_colorformat.");
	}  	     
			   	   
	frame_type.buf_Virt_Addr = g_buffers[g_releasebuff_index].virt_addr;
	frame_type.buffer_phy_addr = g_buffers[g_releasebuff_index].phys_addr;
         
	//LOGV("INTERPOLATION:g_hal_zoom_level=%d,trim_rect:%d,%d,%d,%d,g_cap_zoom_buf_vir_addr=0x%x,g_buffers[g_releasebuff_index].virt_addr=0x%x .",
	//	     g_hal_zoom_level,trim_rect.x,trim_rect.y,trim_rect.w,trim_rect.h,g_cap_zoom_buf_vir_addr,g_buffers[g_releasebuff_index].virt_addr);

	//interpolation start
	LOGV("INTERPOLATION thread,is_2M_to_3M=%d",g_cam_params.is_2M_to_3M);	
	if(1 == g_cam_params.is_2M_to_3M)
	{
		uint32_t addr = 0;
		LOGV("INTERPOLATION thread,old_w=%d,old_h=%d",old_w,old_h);	
		struct zoom_trim_rect trim_rect_new;
		
		trim_rect_new.x = 0; 
		trim_rect_new.y = 0; 
		//trim_rect_new.w = DCAM_MAX_W;
		//trim_rect_new.h = DCAM_MAX_H;	
		trim_rect_new.w = g_max_w;
		trim_rect_new.h = g_max_h;	

		#if 0
                   {
                        FILE *fp = NULL;
                        LOGE("INTERPOLATION00:cap 2M ,g_releasebuff_index=%d,phys_addr=0x%x.", g_releasebuff_index,g_buffers[g_releasebuff_index].phys_addr);
                        fp = fopen("/data/out_cap_yuv422_2M.yuv", "wb");
                        fwrite((uint32_t*)g_capture_virt_addr, 1,trim_rect_new.w * trim_rect_new.h * 2, fp);
                        fclose(fp);
                  }
                 #endif

		addr = (uint32_t)g_capture_virt_addr + 8*1024*1024-trim_rect_new.w*trim_rect_new.h*3/2;
		LOGV("INTERPOLATION00:yuv420 address:0x%x",addr);
		camera_capture_yuv422to420(trim_rect_new.w,trim_rect_new.h,addr,(uint32_t)g_capture_virt_addr);
	         
		#if 0
                   {
                        FILE *fp = NULL;
                        LOGE("INTERPOLATION:cap 2M ,g_releasebuff_index=%d,phys_addr=0x%x.", g_releasebuff_index,g_buffers[g_releasebuff_index].phys_addr);
                        fp = fopen("/data/out_cap_yuv420_2M.yuv", "wb"); 
                        fwrite((uint32_t*)addr, 1,trim_rect_new.w * trim_rect_new.h * 2, fp);
                        fclose(fp);
                  }
                 #endif
		addr = g_capture_phys_addr + 8*1024*1024-trim_rect_new.w*trim_rect_new.h*3/2;				 
		ret = camera_cap_zoom_colorformat(SCALE_DATA_YUV422, old_w, old_h, g_capture_phys_addr, &trim_rect_new, addr, SCALE_DATA_YUV420);
		//g_dcam_obj->free_temp_mem_by_HW();
	          if(0 != ret)
		{
			LOGE("INTERPOLATION:Fail to take picture because the camera_cap_zoom_colorformat in 2M_to_3M.");
			goto TAKE_PIC_FAIL;
		}			
		g_dcam_dimensions.picture_width = old_w;
	    	g_dcam_dimensions.picture_height = old_h;	
		#if 0
                   {
                        FILE *fp = NULL;
                        LOGE("INTERPOLATION:cap 2M to 3M yuv420: width: %d, hei: %d.", g_dcam_dimensions.picture_width, g_dcam_dimensions.picture_height);
                        fp = fopen("/data/new_2M_3M.yuv", "wb");
                        fwrite(g_capture_virt_addr, 1, g_dcam_dimensions.picture_width * g_dcam_dimensions.picture_height * 2, fp);
                        fclose(fp);
                  }
                 #endif	

		//convert data format from YUV422 to YUV420  
		LOGV("INTERPOLATION:before convert_format.");		
		ret = convert_format_by_DMA(g_dcam_dimensions.picture_width, g_dcam_dimensions.picture_height,g_capture_phys_addr, g_dcam_dimensions.picture_height);
			
		if(0 != ret)
		{
			LOGE("INTERPOLATION:Fail to take picture because the convert_format_by_DMA.");
			goto TAKE_PIC_FAIL;
		}	

		#if 0 
                   {
                        FILE *fp = NULL;
                        LOGE("INTERPOLATION:cap 2M to 3M yuv420: width: %d, hei: %d.", g_dcam_dimensions.picture_width, g_dcam_dimensions.picture_height);
                        fp = fopen("/data/out_cap_yuv420_2M_3M.yuv", "wb");
                        fwrite(g_capture_virt_addr, 1, g_dcam_dimensions.picture_width * g_dcam_dimensions.picture_height * 2, fp);
                        fclose(fp);
                  }
                  #endif
		LOGV("INTERPOLATION:before conver endian,g_capture_phys_addr=0x%x!.",g_capture_phys_addr);
		ret = convert_preview_endian_by_DMA(g_dcam_dimensions.picture_width, g_dcam_dimensions.picture_height,g_capture_phys_addr, g_capture_phys_addr);  
		if(0 != ret)
		{
			LOGE("INTERPOLATION:Fail to take picture because the convert_preview_endian_by_DMA.");
			goto TAKE_PIC_FAIL;
		}
		frame_type.buf_Virt_Addr = g_capture_virt_addr;
		frame_type.buffer_phy_addr = g_capture_phys_addr;
		//LOGV("INTERPOLATION:3M data addr,virt_addr = 0x%x,phy_addr = 0x%x!.",frame_type.buf_Virt_Addr,frame_type.buffer_phy_addr );
		//interpolation end
	}
	else
	{
		//convert data format from YUV422 to YUV420  
		LOGV("before convert_format.");		
		ret = convert_format_by_DMA(g_dcam_dimensions.picture_width, g_dcam_dimensions.picture_height,g_buffers[g_releasebuff_index].phys_addr, g_dcam_dimensions.picture_height);
			
		if(0 != ret)
		{
			LOGE("Fail to take picture because the convert_format_by_DMA.");
			goto TAKE_PIC_FAIL;
		}
	}
  
	if((1 == g_cam_params.orientation) && ((90 == g_rotation) || (270 == g_rotation)))
	{
		//get the take picture memory
		uint32_t size = g_dcam_dimensions.picture_width * g_dcam_dimensions.picture_height * 2;
		uint32_t phys_addr;
		uint32_t *virt_addr;
		if(NULL == (virt_addr = (uint32_t *)g_dcam_obj->get_temp_mem_by_HW(size, 1, &phys_addr)))
		{
			LOGE("Fail to malloc capture temp memory, size: 0x%x.", size);
			goto TAKE_PIC_FAIL;
		}
		ret = convert_preview_endian_by_DMA(g_dcam_dimensions.picture_width, g_dcam_dimensions.picture_height, phys_addr, g_buffers[g_releasebuff_index].phys_addr);  
		if(0 != ret)
		{
			g_dcam_obj->free_temp_mem_by_HW();
			LOGE("Fail to take picture because the convert_preview_endian_by_DMA.");
			goto TAKE_PIC_FAIL;
		}	     
		ret = camera_rotation(g_rotation, g_dcam_dimensions.picture_width, g_dcam_dimensions.picture_height, phys_addr, g_buffers[g_releasebuff_index].phys_addr);
		if(0 != ret)
		{
			g_dcam_obj->free_temp_mem_by_HW();
			LOGE("Fail to take picture because the camera_rotation.");
			goto TAKE_PIC_FAIL;
		}
		LOGV("wxz: capture rotation: w: %d, h: %d.", g_dcam_dimensions.picture_width, g_dcam_dimensions.picture_height);	     
		g_dcam_obj->free_temp_mem_by_HW();
		g_dcam_dimensions.picture_width = old_w;
		g_dcam_dimensions.picture_height = old_h;
		g_rotation = 0;
		g_cam_params.orientation = 0;
		if(1 == g_cam_params.is_2M_to_3M)
		{
			LOGE("----INTERPOLATION:flow error-----.");
		}
	}
	else
	{			
	         LOGV("INTERPOLATION:last endian covert, g_cam_params.is_2M_to_3M=%d.",g_cam_params.is_2M_to_3M);
		if(0==g_cam_params.is_2M_to_3M)		
		{				
			ret = convert_preview_endian_by_DMA(g_dcam_dimensions.picture_width, g_dcam_dimensions.picture_height,g_buffers[g_releasebuff_index].phys_addr, g_buffers[g_releasebuff_index].phys_addr);  
			if(0 != ret)
			{
				LOGE("Fail to take picture because the convert_preview_endian_by_DMA.");
				goto TAKE_PIC_FAIL;
			}
		}	     		
	}
   	
	LOGV("after convert_format.");
	frame_type.dx = g_dcam_dimensions.picture_width;
	frame_type.dy = g_dcam_dimensions.picture_height;	
	frame_type.captured_dx = g_dcam_dimensions.picture_width;
	frame_type.captured_dy = g_dcam_dimensions.picture_height;			
	frame_type.format = CAMERA_YCBCR_4_2_0;
	frame_type.rotation = 0;
	frame_type.header_size = 0;
	frame_type.Y_Addr = (uint8_t *)frame_type.buf_Virt_Addr;
	frame_type.CbCr_Addr = frame_type.Y_Addr + frame_type.dx * frame_type.dy;
	
	//LOGV("INTERPOLATION:app get virt_addr=0x%x,width=%d,height=%d.",
	//	   frame_type.Y_Addr,g_dcam_dimensions.picture_width,g_dcam_dimensions.picture_height);			
	#if 0 
                   {
                        FILE *fp = NULL;
                        LOGE("INTERPOLATION:cap 3M yuv420: width: %d, hei: %d.", g_dcam_dimensions.picture_width, g_dcam_dimensions.picture_height);
                        fp = fopen("/data/yuv420_3M.yuv", "wb");
                        fwrite(frame_type.Y_Addr, 1, g_dcam_dimensions.picture_width * g_dcam_dimensions.picture_height * 3/2, fp);
                        fclose(fp);
                  }
                 #endif	

	if(1==g_cam_params.is_2M_to_3M)
	{
 		g_buffers[g_releasebuff_index].virt_addr = g_capture_virt_addr;
	         g_buffers[g_releasebuff_index].phys_addr = g_capture_phys_addr;
	}
				 
	g_callback(CAMERA_EVT_CB_SNAPSHOT_DONE, client_data, CAMERA_FUNC_TAKE_PICTURE, (uint32_t)&frame_type);
	LOGV("after CAMERA_EVT_CB_SNAPSHOT_DONE.");
	g_callback(CAMERA_EXIT_CB_DONE, client_data, CAMERA_FUNC_TAKE_PICTURE, (uint32_t)&frame_type);
		
         return NULL;

TAKE_PIC_FAIL:
	//wxz20110615: handle the error if the dma fail.	
	g_callback(CAMERA_EXIT_CB_FAILED, client_data, CAMERA_FUNC_TAKE_PICTURE, NULL);		
	return NULL;	
}

int camera_capture_init(void)
{
	struct v4l2_format fmt;
	uint32_t min;
	struct v4l2_requestbuffers req;
	uint32_t buffer_size, page_size, frame_size;
	struct v4l2_streamparm streamparm;
	
	CLEAR (fmt);   
    	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;   
	fmt.fmt.pix.width = g_dcam_dimensions.picture_width;   
    	fmt.fmt.pix.height = g_dcam_dimensions.picture_height;   
    //V4L2_PIX_FMT_YVU420, V4L2_PIX_FMT_YUV420 ！ Planar formats with 1/2 horizontal and vertical chroma resolution, also known as YUV 4:2:0   
    //V4L2_PIX_FMT_YUYV ！ Packed format with 1/2 horizontal chroma resolution, also known as YUV 4:2:2   
    //wxz: pixelformat support: V4L2_PIX_FMT_YUV420 and V4L2_PIX_FMT_RGB24;
    	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV422P;//V4L2_PIX_FMT_YUYV; //wxz:TOOD
    	fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;   
    	{   
        	LOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");   
        	LOGV("=====will set fmt to (%d, %d)--", fmt.fmt.pix.width,fmt.fmt.pix.height);   
	        if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) {   
        	    LOGV("V4L2_PIX_FMT_YUYV\n");   
	        } else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420) {   
        	    LOGV("V4L2_PIX_FMT_YUV420\n");   
	        } else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV422P) {   
        	    LOGV("V4L2_PIX_FMT_YUV422P\n");   
	        }  else if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB32) {   
        	    LOGV("V4L2_PIX_FMT_RGB32\n");   
        	}	
    	}   
    	if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))   
	{
		LOGE("Fail to VIDIOC_S_FMT.");
		return -1;
	}
    	{   
        	LOGV("=====after set fmt\n");   
	        LOGV("    fmt.fmt.pix.width = %d\n", fmt.fmt.pix.width);   
	        LOGV("    fmt.fmt.pix.height = %d\n", fmt.fmt.pix.height);   
	        LOGV("    fmt.fmt.pix.sizeimage = %d\n", fmt.fmt.pix.sizeimage);   
	        g_capture_size = fmt.fmt.pix.sizeimage;   
	        LOGV("    fmt.fmt.pix.bytesperline = %d\n", fmt.fmt.pix.bytesperline);   
        	LOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");   
	        LOGV("\n");   
	}   
	g_capture_size = fmt.fmt.pix.sizeimage;   
    /* Note VIDIOC_S_FMT may change width and height. */   
   	 LOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");   
    /* Buggy driver paranoia. */   
    	min = fmt.fmt.pix.width * 2;   
    	if (fmt.fmt.pix.bytesperline < min)   
        	fmt.fmt.pix.bytesperline = min;   
    	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;   
    	if (fmt.fmt.pix.sizeimage < min)   
        	fmt.fmt.pix.sizeimage = min;   
    LOGV("After Buggy driver paranoia\n");   
    LOGV("    >>fmt.fmt.pix.sizeimage = %d\n", fmt.fmt.pix.sizeimage);   
    LOGV("    >>fmt.fmt.pix.bytesperline = %d\n", fmt.fmt.pix.bytesperline);   
    LOGV("-#-#-#-#-#-#-#-#-#-#-#-#-#-\n");   
    LOGV("\n");   
	CLEAR (req);   
	req.count = g_capture_buffer_num;   
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;   
    	req.memory = V4L2_MEMORY_USERPTR;   
    	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) 
	{
		LOGE("Fail to VIDIOC_REQBUFS.");
		return -1;
	}
	
	g_buffers = (buffer *)calloc(g_capture_buffer_num, sizeof(*g_buffers));   
	if (!g_buffers)    
	{
		LOGE("Fail to malloc capture buffer struct.");
		return -1;
	} 
	if(1 == g_cam_params.is_2M_to_3M){//wxz20110825: suppor the 2M to 3M, alloc a 2M buffer for dcam.
                uint32_t phys_addr;
              	uint32_t *virt_addr;
		uint32_t offset = 0;		
		frame_size = fmt.fmt.pix.width * fmt.fmt.pix.height * 2; //for YUYV
		buffer_size = camera_get_size_align_page(frame_size);	      
		if(g_hal_zoom_level>0)
		{			
			//buffer_size = DCAM_MAX_W*DCAM_MAX_H*2;
			buffer_size = g_max_w * g_max_h * 2;
			offset = 6*1024*1024;
			g_cap_zoom_buf_phy_addr = g_capture_phys_addr+offset;
			g_cap_zoom_buf_vir_addr = (uint8_t*)g_capture_virt_addr+offset;
			g_cap_zoom_buf_size = buffer_size;
			g_buffers[0].length = buffer_size; 
			g_buffers[0].virt_addr =  g_capture_virt_addr;		
			g_buffers[0].phys_addr = g_capture_phys_addr;
			LOGV("INTERPOLATION: g_cap_zoom_buf_phy_addr: 0x%x, g_cap_zoom_buf_vir_addr: 0x%x,g_capture_phys_addr:0x%x.",
			 	      g_cap_zoom_buf_phy_addr,(uint32_t)g_cap_zoom_buf_vir_addr,g_capture_phys_addr);
		}
		else
		{
			//buffer_size = DCAM_MAX_W*DCAM_MAX_H*2;
			buffer_size = g_max_w * g_max_h * 2;
			g_buffers[0].length = buffer_size; 
			g_buffers[0].virt_addr =  g_capture_virt_addr;			
			g_buffers[0].phys_addr = g_capture_phys_addr;
			
			LOGV("INTERPOLATION: g_buffer[0]: virt: %x, phys: %x, len: %x,g_capture_phys_addr:0x%x.",
				(uint32_t)g_buffers[0].virt_addr,g_buffers[0].phys_addr,g_buffers[0].length,g_capture_phys_addr);
		}
		
	}
	else{ 
		frame_size = g_dcam_dimensions.picture_width * g_dcam_dimensions.picture_height * 2; //for YUYV
		buffer_size = camera_get_size_align_page(frame_size);
	    	for (n_buffers = 0; n_buffers < g_capture_buffer_num; ++n_buffers) 
		{
	        	g_buffers[n_buffers].length = buffer_size; 
			g_buffers[n_buffers].virt_addr = g_capture_virt_addr + n_buffers * buffer_size / 4;			
			g_buffers[n_buffers].phys_addr = g_capture_phys_addr + n_buffers * buffer_size;
			//memset(g_buffers[n_buffers].virt_addr, 0xFF, buffer_size);
			LOGV("capture: g_buffer[%d]: virt: %x, phys: %x, len: %x.",n_buffers,(uint32_t)g_buffers[n_buffers].virt_addr,g_buffers[n_buffers].phys_addr,g_buffers[n_buffers].length);
	        }
                if(2048 == fmt.fmt.pix.width){
                        uint32_t offset = 6 * 1024 * 1024;
                        g_cap_zoom_buf_phy_addr = g_capture_phys_addr+offset;
                        g_cap_zoom_buf_vir_addr = (uint8_t*)g_capture_virt_addr+offset;
                        g_cap_zoom_buf_size = buffer_size;
                }	 
        } 
        //wxz20111021: raw_data
        //BYTE0: 0: HAL doesn't set sensor_id; 1: HAL sets sensor_id;
        //BYTE1: 0: back camera if BYTE0 is 1; 1: front camera if BYTE0 is 1;
        //BYTE2: 0: no crop and rotation; 1: crop and rotation;
        //BYTE3: 0: HAL doesn't set sensor number; 1: HAL sets sensor number; 2: need to update the sensor number;
	//BYTE4: back sensor number if BYTE3 is 1; If the number is right, use it; not right, check all sensors and change the number; 0xFF is bad sensor;
	//BYTE5: front sensor number if BYTE3 is 1; If the number is right, use it; not right, check all sensors and change the number; 0xFF is bad sensor;
	CLEAR (streamparm);   
	streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	//streamparm.parm.capture.capturemode = 1;
	streamparm.parm.raw_data[6] = 1;
	if(-1 == g_camera_id)
	{
		streamparm.parm.raw_data[0] = 0;
	}
	else
	{
		streamparm.parm.raw_data[0] = 1;
		streamparm.parm.raw_data[1] = g_camera_id;
	}
	g_rotation = g_rotation_parm; 
	g_cam_params.orientation = g_cam_params.orientation_parm;
	if((1 == g_cam_params.orientation) && ((90 == g_rotation) || (270 == g_rotation))){
		streamparm.parm.raw_data[2] = 1;
	}
	else{
		streamparm.parm.raw_data[2] = 0;
	}
#if HAL_SET_SENSOR_ID
	//wxz20111021: set sensor number
	streamparm.parm.raw_data[3] = 1;
	streamparm.parm.raw_data[4] = camera_get_sensor_number(BACK_CAMERA_ID);//back camera
	streamparm.parm.raw_data[5] = camera_get_sensor_number(FRONT_CAMERA_ID);//front camera
#endif
	if (-1 == xioctl(fd, VIDIOC_S_PARM, &streamparm)) 
	{
		LOGE("preview: Fail to VIDIOC_S_PARM.");
		return -1;
	}
#if HAL_SET_SENSOR_ID
	//need to update the sensor number.
	LOGV("camera_capture_init: data1: %d, data3: %d, data4: %d, data5: %d.", streamparm.parm.raw_data[1], streamparm.parm.raw_data[3], streamparm.parm.raw_data[4], streamparm.parm.raw_data[5]);
	if(2 == streamparm.parm.raw_data[3]){
		if(BACK_CAMERA_ID == streamparm.parm.raw_data[1]){
			camera_set_sensor_number(BACK_CAMERA_ID, streamparm.parm.raw_data[4]);
		}
		else if(FRONT_CAMERA_ID == streamparm.parm.raw_data[1]){
			camera_set_sensor_number(FRONT_CAMERA_ID, streamparm.parm.raw_data[5]);
		}
	}
#endif
#if FRONT_CAMERA_MIRROR
	//wxz20111112: enable the front camera mirror funciton in caputre mode
	if(1 == g_camera_id){ //for front camera
		uint32_t enable = 1;
		if((90 == g_cam_params.rotation_degree) || (270 == g_cam_params.rotation_degree)){ //for portrait
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
	
	if(0 == g_hal_zoom_level){
		for (i = G_CAPTURE_BUF_OFFSET; i < g_capture_buffer_num; ++i) 
		{   
	            struct v4l2_buffer buf;   
	            CLEAR (buf);   
	            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;   
	            buf.memory = V4L2_MEMORY_USERPTR;   
	            buf.index = i;   
	            buf.m.userptr = g_buffers[i].phys_addr;               
	            buf.length = g_buffers[i].length;   	
		    LOGE("####capture QBuf: buffers[%d].start: %lx, %x\n", buf.index, buf.m.userptr,buf.length);	
	            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))   
		    {
			LOGE("Fail to VIDIOC_QBUF from camera_capture_qbuffers.");
			return -1;
		    } 
	        }
	}
	else{
		//qbuf the other buffer.
		if(0 != g_cap_zoom_buf_phy_addr){
			    struct v4l2_buffer buf;   
		            CLEAR (buf);   
		            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;   
		            buf.memory = V4L2_MEMORY_USERPTR;   
		            buf.index = 0;   
		            buf.m.userptr = g_cap_zoom_buf_phy_addr;               
		            buf.length = g_cap_zoom_buf_size;   	
			    LOGE("####capture QBuf: buffers[%d].start: %lx, %x\n", buf.index, buf.m.userptr,buf.length);	
		            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))   
			    {
				LOGE("Fail to VIDIOC_QBUF from camera_capture_qbuffers.");
				return -1;
			    } 
		}
		else
		{
			if(1 == g_cam_params.is_2M_to_3M )
			{
				struct v4l2_buffer buf;   
				CLEAR (buf);   
				buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;   
				buf.memory = V4L2_MEMORY_USERPTR;   
				buf.index = 0;   
				buf.m.userptr = g_capture_phys_addr ;//g_cap_zoom_buf_phy_addr;               
				//buf.length = DCAM_MAX_W*DCAM_MAX_H*2;   	
				buf.length = g_max_w * g_max_h * 2;   	
				LOGE("INTERPOLATION:capture QBuf: buffers[%d].start: %lx, %x\n", buf.index, buf.m.userptr,buf.length);	
				if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))   
				{
				LOGE("Fail to VIDIOC_QBUF from camera_capture_qbuffers.");
				return -1;
				} 
			}
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
		LOGE("Fail to VIDIOC_STREAMON.");
		return -1;
	 }

	return 0;
}

void camera_alloc_zoom_buffer(uint32_t phy_addr, uint8_t *vir_addr,uint32_t size)
{
	g_cap_zoom_buf_phy_addr = phy_addr;
	g_cap_zoom_buf_vir_addr = vir_addr;
	g_cap_zoom_buf_size = size;
	//LOGV("INTERPOLATION:camera_alloc_zoom_buffer,phy_addr=0x%x,virt_addr=0x%x,size=%d.",
	//	     g_cap_zoom_buf_phy_addr,g_cap_zoom_buf_vir_addr,size);
}
camera_ret_code_type camera_take_picture (
        camera_cb_f_type    callback,
        void               *client_data
#if !defined FEATURE_CAMERA_ENCODE_PROPERTIES && defined FEATURE_CAMERA_V7
        ,camera_raw_type camera_raw_mode
#endif // nFEATURE_CAMERA_ENCODE_PROPERTIES && FEATURE_CAMERA_V7 
        )
{
	LOGV("camera_take_picture E.");		
	camera_ret_code_type ret_type = CAMERA_SUCCESS;
	g_dcam_obj = (SprdCameraHardware *)client_data;
	int ret;	
	uint32_t size;

	//record the callback function
	g_callback = callback;

	//callback(CAMERA_RSP_CB_SUCCESS, g_dcam_obj, CAMERA_FUNC_TAKE_PICTURE, 0);

	if((0 == g_camera_id) && (2048 == g_dcam_dimensions.picture_width) 
               && (1600 >= atoi(get_back_camera_capability("max_width")))){
		g_cam_params.is_2M_to_3M = 1;
	}
	else{
		g_cam_params.is_2M_to_3M = 0;
	}

	//get the take picture memory
	size = (g_dcam_dimensions.picture_width * g_dcam_dimensions.picture_height * 2) * g_capture_buffer_num;
	if(NULL == (g_capture_virt_addr = (uint32_t *)g_dcam_obj->get_raw_mem(size,&g_capture_phys_addr,0)))
	{
		LOGE("Fail to malloc capture memory, size: 0x%x.", size);
		return CAMERA_FAILED;
	}
	if(0 != camera_capture_init())
	{
		LOGE("Fail to init capture mode.");
		return CAMERA_FAILED;
	} 
	if(0 != camera_capture_qbuffers())
	{
		LOGE("Fail to qbuffers for capture.");
		return CAMERA_FAILED;
	}
	LOGV("OK  to camera_capture_qbuffers.");
	if(0 != camera_capture_streamon())
	{
		LOGE("Fail to stream on for capture.");
		return CAMERA_FAILED;
	}

	//update the status //wxz:???20110119
	/*callback(CAMERA_RSP_CB_SUCCESS, g_dcam_obj, CAMERA_FUNC_TAKE_PICTURE, 0);*/

	//create the thread for preview	
	if(0 != (ret = pthread_create(&g_capture_thr, NULL, camera_capture_thread, client_data)))
	{
		LOGE("Fail to careate thread in preview mode.");
		return CAMERA_FAILED;
	}
	else
	{
		//pthread_join(g_capture_thr, NULL);
		LOGE("OK to create thread in capture mode.");
	}
	
	LOGV("camera_take_picture X.");
	
	return ret_type;
}
void rex_start()
{
	return;
}
void rex_shutdown()
{
	return;
}	
};
