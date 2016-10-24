/******************************************************************************
 ** File Name:      jpegenc_api.c                                            *
 ** Author:         xiaozhe.wang													  *
 ** DATE:           02/15/2011                                                *
 ** Copyright:      2011 Spreadtrum, Incoporated. All Rights Reserved.        *
 ** Description:    The interface functions for jpeg encoder.									  *
 ** Note:           None                                                      *
******************************************************************************/
/******************************************************************************
 **                        Edit History                                       *
 ** ------------------------------------------------------------------------- *
 ** DATE           NAME             DESCRIPTION                               *
 ** 02/15/2011     xiaozhe.wang	         Create.                                  *
******************************************************************************/
/*----------------------------------------------------------------------------*
**                        Dependencies                                        *
**---------------------------------------------------------------------------*/
#include "sc8810_video_header.h"
#include "jpegcodec_bufmgr.h"
#include "jpegenc_api.h"
#include <sys/ioctl.h>
#include <errno.h>
#include <sys/mman.h>
#include <fcntl.h>
#include "sprd_scale.h"
/**---------------------------------------------------------------------------*
**                        Compiler Flag                                       *
**---------------------------------------------------------------------------*/
#ifdef   __cplusplus
    extern   "C" 
    {
#endif
		
#if defined(JPEG_ENC)

#define LOG_NDEBUG 0
#include "sprd_vsp.h"

#define SPRD_VSP_DRIVER "/dev/sprd_vsp"

#define SLICE_HEIGHT 1024 //the height for slice mode

typedef struct scale_param
{
	SCALE_DATA_FORMAT_E in_fmt;
	SCALE_SIZE_T in_size;
	SCALE_RECT_T in_rect;
	SCALE_ADDRESS_T in_addr;
	SCALE_DATA_FORMAT_E out_fmt;
	SCALE_SIZE_T out_size;	
	SCALE_ADDRESS_T out_addr;	
}SCALE_PARAM_T;
static uint32_t g_stream_buf_id = 1;  //record the bsm buf id for switch buf. the init buf is 0.
uint32_t g_stream_buf_size = 0;

uint32_t JPEGENC_Poll_VLC_BSM(uint32_t time, uint32_t buf_len,  jpegenc_callback callback);
PUBLIC JPEG_RET_E JPEGENC_stop_encode(JPEGENC_PARAMS_T *jpegenc_params);
//////////////////////////////////////////////////////////////////////////
int VSP_reset_cb(int fd)
{
//	SCI_TRACE_LOW("VSP_reset_cb\n");
	ioctl(fd,VSP_RESET,NULL);
	return 0;
}

LOCAL void JPEGENC_init_fw_param(JPEGENC_PARAMS_T *jpegenc_params, 
									JPEG_ENC_INPUT_PARA_T *enc_fw_info_ptr)
{
	uint32_t slice_height;
	
	enc_fw_info_ptr->is_first_slice = TRUE;
	if(jpegenc_params->height > SLICE_HEIGHT){
		enc_fw_info_ptr->is_last_slice = FALSE;
		slice_height = SLICE_HEIGHT;
	}
	else{
		enc_fw_info_ptr->is_last_slice = TRUE;
		slice_height = jpegenc_params->height;
	}	

	if((0 != jpegenc_params->set_slice_height)&&(jpegenc_params->height > SLICE_HEIGHT))
	{
		slice_height = jpegenc_params->set_slice_height;
	}
	SCI_TRACE_LOW("[JPEG_EncInitFwParam] jpeg fw info.slice_height=%d.",slice_height);
	
	enc_fw_info_ptr->width = jpegenc_params->width;
	enc_fw_info_ptr->height = jpegenc_params->height;
	enc_fw_info_ptr->quant_level = (JPEG_QUALITY_E)((jpegenc_params->quality >= JPEG_QUALITY_MAX) ? JPEG_QUALITY_HIGH 
								: jpegenc_params->quality);	
	
	enc_fw_info_ptr->yuv_0_info.input_mcu_info 	= jpegenc_params->format;
	enc_fw_info_ptr->yuv_1_info.input_mcu_info 	= jpegenc_params->format;
	enc_fw_info_ptr->yuv_0_info.y_data_ptr	= (uint8_t *)jpegenc_params->yuv_phy_buf;	
	if(SLICE_HEIGHT == slice_height)
	{
		enc_fw_info_ptr->yuv_0_info.u_data_ptr	= enc_fw_info_ptr->yuv_0_info.y_data_ptr + jpegenc_params->width * jpegenc_params->height;
	}
	else
	{
		enc_fw_info_ptr->yuv_0_info.u_data_ptr	= enc_fw_info_ptr->yuv_0_info.y_data_ptr + jpegenc_params->width * slice_height;
	}
	enc_fw_info_ptr->yuv_0_info.v_data_ptr	= PNULL;
	enc_fw_info_ptr->yuv_0_info.out_mcu_info = jpegenc_params->format;
	enc_fw_info_ptr->yuv_0_info.input_endian = 1;
	if(jpegenc_params->height > SLICE_HEIGHT){
		if(0 == jpegenc_params->set_slice_height)
		{
			enc_fw_info_ptr->yuv_1_info.y_data_ptr	= (uint8_t *)jpegenc_params->yuv_phy_buf + jpegenc_params->width * slice_height; //fro YUV420
			enc_fw_info_ptr->yuv_1_info.u_data_ptr	= (uint8_t *)jpegenc_params->yuv_phy_buf + jpegenc_params->width * jpegenc_params->height + jpegenc_params->width * slice_height / 2;
		}
		else
		{
			enc_fw_info_ptr->yuv_1_info.y_data_ptr = enc_fw_info_ptr->yuv_0_info.y_data_ptr;
			enc_fw_info_ptr->yuv_1_info.u_data_ptr = enc_fw_info_ptr->yuv_0_info.u_data_ptr;
		}
		enc_fw_info_ptr->yuv_1_info.v_data_ptr	= PNULL;	
		enc_fw_info_ptr->yuv_1_info.out_mcu_info = jpegenc_params->format;
		enc_fw_info_ptr->yuv_1_info.input_endian = 1;		
	}
	else{
		enc_fw_info_ptr->yuv_1_info.y_data_ptr	= PNULL;
		enc_fw_info_ptr->yuv_1_info.u_data_ptr	= PNULL;
		enc_fw_info_ptr->yuv_1_info.v_data_ptr	= PNULL;	
		enc_fw_info_ptr->yuv_1_info.out_mcu_info = jpegenc_params->format;
		enc_fw_info_ptr->yuv_1_info.input_endian = 1;
	}
	
	enc_fw_info_ptr->bitstream_buf_len = jpegenc_params->stream_buf_len;
	g_stream_buf_size = enc_fw_info_ptr->bitstream_buf_len;
	enc_fw_info_ptr->stream_buf0 = (uint8_t *)jpegenc_params->stream_phy_buf[0];
	enc_fw_info_ptr->stream_buf1 = (uint8_t *)jpegenc_params->stream_phy_buf[1];

	enc_fw_info_ptr->enc_buf.buf_size = 0;
	enc_fw_info_ptr->enc_buf.buf_ptr = NULL;

	enc_fw_info_ptr->mea_bfr0_valid = TRUE;
	enc_fw_info_ptr->mea_bfr1_valid = FALSE;
	enc_fw_info_ptr->stream_buf0_valid = TRUE;
	enc_fw_info_ptr->stream_buf1_valid = FALSE;
	enc_fw_info_ptr->work_mode = ALONE_MODE;
	enc_fw_info_ptr->restart_interval = 0;

	SCI_TRACE_LOW("[JPEG_EncInitFwParam] jpeg fw info, len = %d, ping addr = 0x%x, pong addr = 0x%x, encode size = %d,  encode addr = 0x%x", 
					enc_fw_info_ptr->bitstream_buf_len, (uint32_t)enc_fw_info_ptr->stream_buf0, (uint32_t)enc_fw_info_ptr->stream_buf1, 
					(uint32_t)enc_fw_info_ptr->enc_buf.buf_ptr, enc_fw_info_ptr->enc_buf.buf_size);		
	
	SCI_TRACE_LOW("[JPEG_EncInitFwParam] ping mcu info = %d, pong mcu info = %d, width = %d, height = %d", 
					enc_fw_info_ptr->yuv_0_info.input_mcu_info, enc_fw_info_ptr->yuv_1_info.input_mcu_info, enc_fw_info_ptr->width, 
					enc_fw_info_ptr->height);
}

LOCAL int xioctl(int fd, int request, void * arg) {   
    int r;   
    do   
        r = ioctl(fd, request, arg);   
    while (-1 == r && EINTR == errno);   
    return r;   
} 
LOCAL JPEG_RET_E JPEGENC_Scale_For_Thumbnail(SCALE_PARAM_T *scale_param)
{
	static int fd = -1; 	
	SCALE_CONFIG_T scale_config;	
	SCALE_MODE_E scale_mode;
	uint32_t enable = 0;
	ISP_ENDIAN_T endian_mode;

	fd = open("/dev/sprd_scale", O_RDONLY);
	if (-1 == fd) 
	{   
		SCI_TRACE_LOW("Fail to open scale device.");
        	return JPEG_FAILED;   
   	 }
    	
	//set mode
	scale_config.id = SCALE_PATH_MODE;	
	scale_mode = SCALE_MODE_SCALE;
	scale_config.param = &scale_mode;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		SCI_TRACE_LOW("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return JPEG_FAILED;
	}
	//set input data format
	scale_config.id = SCALE_PATH_INPUT_FORMAT;	
	scale_config.param = &scale_param->in_fmt;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		SCI_TRACE_LOW("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return JPEG_FAILED;
	}
	//set output data format
	scale_config.id = SCALE_PATH_OUTPUT_FORMAT;	
	scale_config.param = &scale_param->out_fmt;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		SCI_TRACE_LOW("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return JPEG_FAILED;
	}
	//set input size
	scale_config.id = SCALE_PATH_INPUT_SIZE;	
	scale_config.param = &scale_param->in_size;	 
	SCI_TRACE_LOW("test scale:input size:%d,%d.",scale_param->in_size.w,scale_param->in_size.h);
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		SCI_TRACE_LOW("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return JPEG_FAILED;
	}
	//set output size
	scale_config.id = SCALE_PATH_OUTPUT_SIZE;	
	scale_config.param = &scale_param->out_size;	 
	SCI_TRACE_LOW("test scale:out size:%d,%d.",scale_param->out_size.w,scale_param->out_size.h);
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		SCI_TRACE_LOW("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return JPEG_FAILED;
	}	
	//set input size
	scale_config.id = SCALE_PATH_INPUT_RECT;
	scale_config.param = &scale_param->in_rect;	 
	SCI_TRACE_LOW("test scale:input rect:%d,%d,%d,%d .",scale_param->in_rect.x,scale_param->in_rect.y,scale_param->in_rect.w,scale_param->in_rect.h);
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		SCI_TRACE_LOW("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return JPEG_FAILED;
	}
	//set input address
	scale_config.id = SCALE_PATH_INPUT_ADDR;	
	scale_config.param = &scale_param->in_addr;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		SCI_TRACE_LOW("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return JPEG_FAILED;
	}
	//set output address
	scale_config.id = SCALE_PATH_OUTPUT_ADDR;	
	scale_config.param = &scale_param->out_addr;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		SCI_TRACE_LOW("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return JPEG_FAILED;
	}	
	
	//set input endian
	scale_config.id = SCALE_PATH_INPUT_ENDIAN;
	//endian_mode = 1;
	endian_mode.endian_y = 1;
	endian_mode.endian_uv = 1;

	scale_config.param = &endian_mode;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		SCI_TRACE_LOW("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return JPEG_FAILED;
	}
	//set output endian
	scale_config.id = SCALE_PATH_OUTPUT_ENDIAN;
	//endian_mode = 1;
	endian_mode.endian_y = 1;
	endian_mode.endian_uv = 1;

	scale_config.param = &endian_mode;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		SCI_TRACE_LOW("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return JPEG_FAILED;
	}	
	
	//done	 
	if (-1 == xioctl(fd, SCALE_IOC_DONE, 0))   
	{
		SCI_TRACE_LOW("Fail to SCALE_IOC_DONE");
		return JPEG_FAILED;
	}

	if(-1 == close(fd))   
	{   
		SCI_TRACE_LOW("Fail to close scale device.");
        	return JPEG_FAILED;   
   	 } 
    	fd = -1;   

	return JPEG_SUCCESS;
}

LOCAL JPEG_RET_E JPEGENC_start_encode_thumbnail(JPEGENC_PARAMS_T *jpegenc_params)
{
	JPEG_RET_E ret_value = JPEG_FAILED;
	JPEG_ENC_OUTPUT_PARA_T jpeg_enc_out_param;
	JPEG_ENC_INPUT_PARA_T 	jpeg_enc_fw_info;
	SCALE_PARAM_T scale_param;
	JPEGENC_PARAMS_T jpegenc_params_tmp;

	SCI_PASSERT(jpegenc_params, ("[JPEG_6600L_StartEncodeThumbnail], context_ptr is NULL"));

	//scale down the original yuv data to 176x144 size.
	SCI_MEMSET(&scale_param, 0, sizeof(SCALE_PARAM_T));
	scale_param.in_addr.yaddr = jpegenc_params->thumb_src_yuv_phy_buf;
	scale_param.in_addr.uaddr = scale_param.in_addr.yaddr + jpegenc_params->width * jpegenc_params->height;
	scale_param.in_addr.vaddr = 0;
	scale_param.in_fmt = SCALE_DATA_YUV420;//jpegenc_params->format;
	scale_param.in_size.w = jpegenc_params->width;
	scale_param.in_size.h = jpegenc_params->height;
	scale_param.in_rect.x = 0;
	scale_param.in_rect.y = 0;
	scale_param.in_rect.w = jpegenc_params->width;
	scale_param.in_rect.h = jpegenc_params->height;
	scale_param.out_size.w = jpegenc_params->thumb_width;
	scale_param.out_size.h = jpegenc_params->thumb_height;
	//use the second stream bufffer as the scale output buffer.
	scale_param.out_addr.yaddr = jpegenc_params->stream_phy_buf[0];
	scale_param.out_addr.uaddr = scale_param.out_addr.yaddr + scale_param.out_size.w * scale_param.out_size.h;
	scale_param.out_addr.vaddr = 0;
	scale_param.out_fmt = SCALE_DATA_YUV420;//jpegenc_params->format;

	ret_value = JPEGENC_Scale_For_Thumbnail(&scale_param);
	if(JPEG_SUCCESS != ret_value)
	{
		SCI_TRACE_LOW("JPEGENC_Scale_For_Thumbnail = %d", ret_value);
		return ret_value;
	}

		#if 0
			{
				FILE *fp = NULL;				
				SCI_TRACE_LOW("cap yuv420: width: %d, hei: %d.", scale_param.out_size.w, scale_param.out_size.h);				
				fp = fopen("/data/out_enc_yuv420.yuv", "wb");
				fwrite(jpegenc_params->stream_virt_buf[0], 1, scale_param.out_size.w * scale_param.out_size.h * 3 / 2, fp);
				fclose(fp);								
			}
		#endif

	SCI_MEMSET(&jpeg_enc_out_param, 0, sizeof(JPEG_ENC_OUTPUT_PARA_T));
	SCI_MEMSET(&jpeg_enc_fw_info, 0, sizeof(JPEG_ENC_INPUT_PARA_T));
	SCI_MEMCPY(&jpegenc_params_tmp, jpegenc_params, sizeof(JPEGENC_PARAMS_T));

	//update the jpegenc_params_tmp
	jpegenc_params_tmp.width = scale_param.out_size.w;
	jpegenc_params_tmp.height = scale_param.out_size.h;
	jpegenc_params_tmp.yuv_phy_buf = scale_param.out_addr.yaddr;
	jpegenc_params_tmp.stream_phy_buf[0] = jpegenc_params->stream_phy_buf[1];
	jpegenc_params_tmp.stream_virt_buf[0] = jpegenc_params->stream_virt_buf[1];
	jpegenc_params_tmp.quality = jpegenc_params->thumb_quality;	

	JPEGENC_init_fw_param(&jpegenc_params_tmp, &jpeg_enc_fw_info);

	ret_value = JPEG_HWEncInit(&jpeg_enc_fw_info);
	if(JPEG_SUCCESS != ret_value)
	{
		SCI_TRACE_LOW("JPEG_HWEncInit failed for thubmnail  = %d", ret_value);
		return ret_value;
	}
	
	ret_value = JPEG_HWWriteHeadForThumbnail();
	if(JPEG_SUCCESS != ret_value)
	{
		SCI_TRACE_LOW("JPEG_HWWriteHead for thumbnail failed = %d", ret_value);
		return ret_value;
	}

	/*the input width must be mcu aligned width*/	
	ret_value = JPEG_HWEncStart(jpegenc_params_tmp.width, jpegenc_params_tmp.height, &jpeg_enc_out_param);

	SCI_TRACE_LOW("[JPEG_6600L_StartEncode for thumbnail] start enc, src_aligned_width = %d, slice height = %d", 
										jpegenc_params_tmp.width, jpegenc_params_tmp.height);

	//poll the end of jpeg encoder
	JPEGENC_Poll_VLC_BSM(0xFFF, jpegenc_params_tmp.stream_buf_len, NULL);	

	ret_value = JPEG_HWWriteTail();
	if(JPEG_SUCCESS != ret_value)
	{
		SCI_TRACE_LOW("JPEG_HWWriteTail for thumbnail failed = %d", ret_value);
		return ret_value;
	}
	jpegenc_params->stream_size = JPEG_HWGetSize();

	#if 0
	{
	        FILE *fp = NULL;
	        SCI_TRACE_LOW("thumb size head: %d.", jpegenc_params->stream_size);
	        fp = fopen("/data/out_thumb_head.jpg", "wb");
	        fwrite(jpegenc_params->stream_virt_buf[1], 1, jpegenc_params->stream_size, fp);
	        fclose(fp);
	}
#endif
	
	return ret_value;
}
LOCAL JPEG_RET_E JPEGENC_start_encode(JPEGENC_PARAMS_T *jpegenc_params)
{
	JPEG_RET_E 				ret_value = JPEG_FAILED;
	JPEG_ENC_OUTPUT_PARA_T jpeg_enc_out_param;
	JPEG_ENC_INPUT_PARA_T 	jpeg_enc_fw_info;
	uint32_t slice_height = SLICE_HEIGHT;
	APP1_T app1_param;	

	if(0 != jpegenc_params->set_slice_height)
	{
		slice_height = jpegenc_params->set_slice_height;
	}
		SCI_MEMSET(&app1_param, 0, sizeof(APP1_T));
	if((0 == jpegenc_params->thumb_width) && (0 == jpegenc_params->thumb_height)){
		jpegenc_params->stream_size = 0;
	}
	else{
		if(JPEG_SUCCESS != JPEGENC_start_encode_thumbnail(jpegenc_params))		
		{
			SCI_TRACE_LOW("JPEGENC fail to JPEGENC_start_encode_thumbnail.");
			return ret_value;
		}
	}
	app1_param.thumb_width = jpegenc_params->thumb_width;
	app1_param.thumb_height = jpegenc_params->thumb_height;
	app1_param.thumbnail_virt_addr = jpegenc_params->stream_virt_buf[1];
	app1_param.thumbnail_len = jpegenc_params->stream_size;
	app1_param.stream_buf_len = jpegenc_params->stream_buf_len;
	app1_param.Latitude_dd.numerator = jpegenc_params->Latitude_dd.numerator;
	app1_param.Latitude_dd.denominator = jpegenc_params->Latitude_dd.denominator;
	app1_param.Latitude_mm.numerator = jpegenc_params->Latitude_mm.numerator;
	app1_param.Latitude_mm.denominator = jpegenc_params->Latitude_mm.denominator;
	app1_param.Latitude_ss.numerator = jpegenc_params->Latitude_ss.numerator;
	app1_param.Latitude_ss.denominator = jpegenc_params->Latitude_ss.denominator;
	app1_param.Latitude_ref = jpegenc_params->Latitude_ref;	
	app1_param.Longitude_dd.numerator = jpegenc_params->Longitude_dd.numerator;
	app1_param.Longitude_dd.denominator = jpegenc_params->Longitude_dd.denominator;	
	app1_param.Longitude_mm.numerator = jpegenc_params->Longitude_mm.numerator;
	app1_param.Longitude_mm.denominator = jpegenc_params->Longitude_mm.denominator;	
	app1_param.Longitude_ss.numerator = jpegenc_params->Longitude_ss.numerator;
	app1_param.Longitude_ss.denominator = jpegenc_params->Longitude_ss.denominator;
	app1_param.Longitude_ref = jpegenc_params->Longitude_ref;
	app1_param.image_description = jpegenc_params->image_description;
	app1_param.make = jpegenc_params->make;
	app1_param.model = jpegenc_params->model;
	app1_param.copyright = jpegenc_params->copyright;
	app1_param.orientation = jpegenc_params->orientation;
	app1_param.datetime = jpegenc_params->datetime;
	app1_param.gps_date = jpegenc_params->gps_date;
	app1_param.gps_process_method = jpegenc_params->gps_process_method;
	app1_param.gps_hour = jpegenc_params->gps_hour;	
	app1_param.gps_minuter = jpegenc_params->gps_minuter;
	app1_param.gps_second = jpegenc_params->gps_second;	
	app1_param.image_width = jpegenc_params->width;
	app1_param.image_height = jpegenc_params->height;
	app1_param.focal_length.numerator = jpegenc_params->focal_length.numerator;
	app1_param.focal_length.denominator = jpegenc_params->focal_length.denominator;

	app1_param.dc_exif_info_ptr 	= jpegenc_params->dc_exif_info_ptr;

	jpegenc_params->stream_size = 0;
	SCI_TRACE_LOW("JPEGENC ok to JPEGENC_start_encode_thumbnail.addr: 0x%x, len: 0x%x.", (uint32_t)app1_param.thumbnail_virt_addr, app1_param.thumbnail_len);


	SCI_PASSERT(jpegenc_params, ("[JPEG_6600L_StartEncode], context_ptr is NULL"));

	SCI_MEMSET(&jpeg_enc_out_param, 0, sizeof(JPEG_ENC_OUTPUT_PARA_T));
	SCI_MEMSET(&jpeg_enc_fw_info, 0, sizeof(JPEG_ENC_INPUT_PARA_T));

	JPEGENC_init_fw_param(jpegenc_params, &jpeg_enc_fw_info);

	ret_value = JPEG_HWEncInit(&jpeg_enc_fw_info);
	if(JPEG_SUCCESS != ret_value)
	{
		SCI_TRACE_LOW("JPEG_HWEncInit failed = %d", ret_value);
		return ret_value;
	}

	ret_value = JPEG_HWWriteHead(&app1_param);
	//ret_value = JPEG_HWWriteHead();
	if(JPEG_SUCCESS != ret_value)
	{
		SCI_TRACE_LOW("JPEG_HWWriteHead failed = %d", ret_value);
		return ret_value;
	}	

	SCI_TRACE_LOW("[JPEG_6600L_StartEncode] hardware write head done");

	/*the input width must be mcu aligned width*/
	if(jpegenc_params->height > SLICE_HEIGHT){
		ret_value = JPEG_HWEncStart(jpegenc_params->width, slice_height, &jpeg_enc_out_param);
	}
	else{
		ret_value = JPEG_HWEncStart(jpegenc_params->width, jpegenc_params->height, &jpeg_enc_out_param);
	}

	SCI_TRACE_LOW("[JPEG_6600L_StartEncode] start enc, src_aligned_width = %d, slice height = %d", 
										jpegenc_params->width, jpegenc_params->height);

	return ret_value;
}

PUBLIC JPEG_RET_E JPEGENC_stop_encode(JPEGENC_PARAMS_T *jpegenc_params)
{
	JPEG_RET_E	jpeg_ret_value = JPEG_FAILED;

	SCI_PASSERT(jpegenc_params, ("[JPEG_6600L_StopEncode], context_ptr is NULL"));
	jpeg_ret_value = JPEG_HWWriteTail();
	if( JPEG_SUCCESS == jpeg_ret_value)
	{
		jpegenc_params->stream_size = JPEG_HWGetSize( );

		/*if (jpegenc_params->stream_size > jpegenc_params->stream_buf_len)
		{
			SCI_TRACE_LOW("[JPEG_StopEncode] jpeg stream buffer is not enough, stream size = %d, target buffer size = %d",
							jpegenc_params->stream_size, jpegenc_params->stream_buf_len);	
			return JPEG_MEMORY_NOT_ENOUGH;
		}*/
	}
	
	SCI_TRACE_LOW("[JPEG_6600L_StopEncode] start enc, stream_size = %d", jpegenc_params->stream_size);	
	
	return jpeg_ret_value;
}

#if 0
void get_regs(void)
{
	int i, value;
	for(i = 0; i < 6; i++)
	{
		value = VSP_READ_REG(VSP_GLB_REG_BASE + i * 4,"");
		SCI_TRACE_LOW("GLB 0x%x : 0x%x", VSP_GLB_REG_BASE + i * 4, value);
	}		
	for(i = 0; i < 3; i++)
	{
		value = VSP_READ_REG(VSP_AHBM_REG_BASE + i * 4,"");
		SCI_TRACE_LOW("AHBM 0x%x : 0x%x", VSP_AHBM_REG_BASE + i * 4, value);
	}
	for(i = 0; i < 12; i++)
	{
		value = VSP_READ_REG(VSP_BSM_REG_BASE + i * 4,"");
		SCI_TRACE_LOW("BSM 0x%x : 0x%x", VSP_BSM_REG_BASE + i * 4, value);
	}	
	for(i = 0; i < 3; i++)
	{
		value = VSP_READ_REG(VSP_VLC_REG_BASE + i * 4,"");
		SCI_TRACE_LOW("VLC 0x%x : 0x%x", VSP_VLC_REG_BASE + i * 4, value);
	}
	for(i = 0; i < 12; i++)
	{
		value = VSP_READ_REG(VSP_DCAM_REG_BASE + i * 4,"");
		SCI_TRACE_LOW("DCAM 0x%x : 0x%x", VSP_DCAM_REG_BASE + i * 4, value);
	}
	for(i = 0; i < 19; i++)
	{
		value = VSP_READ_REG(VSP_MEA_REG_BASE + i * 4,"");
		SCI_TRACE_LOW("MEA 0x%x : 0x%x", VSP_MEA_REG_BASE + i * 4, value);
	}	
}
#endif
void JPGEENC_Clear_INT(uint32_t mask)
{	
	uint32_t value;
	
	value = VSP_READ_REG(VSP_DCAM_REG_BASE + DCAM_INT_RAW_OFF, "");
	SCI_TRACE_LOW("JPEGENC DCAM_INT_RAW_OFF: 0x%x.", value);	
	value = VSP_READ_REG(VSP_DCAM_REG_BASE + DCAM_INT_CLR_OFF, "read the interrupt clear bits.");
	value |= mask;
	VSP_WRITE_REG(VSP_DCAM_REG_BASE + DCAM_INT_CLR_OFF, value, "clear the VLC interrupt.");
}

void JPEGENC_Handle_BSM_INT(jpegenc_callback callback)
{
	uint32_t tmp = g_stream_buf_id;

	JPGEENC_Clear_INT(0x80);
	if(0 == g_stream_buf_id){				
		g_stream_buf_id = 1;
	}
	else{		
		g_stream_buf_id = 0;				
	}
	JPEG_HWSet_BSM_Buf_WriteOnly(g_stream_buf_id);
	SCI_TRACE_LOW("JPEGENC_Poll_MEA_BSM JPEG_HWSet_BSM_Buf_WriteOnly after.g_stream_buf_id: %d.\n", g_stream_buf_id);
	callback(g_stream_buf_id, g_stream_buf_size, 0);
	SCI_TRACE_LOW("JPEGENC_Poll_MEA_BSM callback after.\n");		
}

//poll MEA done and BSM done
uint32_t JPEGENC_Poll_MEA_BSM(uint32_t time, uint32_t buf_len,  uint32_t slice_num,jpegenc_callback callback,jpegenc_callback updata_yuv_callback)
{
	uint32_t value;
	uint32_t vsp_time_out_cnt = 0;
	uint32_t buf_id = 1; 

	SCI_TRACE_LOW("JPEGENC_Poll_MEA_BSM E,slice_num=%d.\n",slice_num);
			
	while (1)
	{
		value = VSP_READ_REG(VSP_DCAM_REG_BASE + DCAM_INT_RAW_OFF, "read the interrupt register.");		
		if(value & 0x80){ //for BSM done			
			JPEGENC_Handle_BSM_INT(callback);
		}
		if(value & 0x4000){ //for MEA done
			 slice_num--;
			 if(slice_num>0)
			 {
				 JPEG_HWUpdateMEABufInfo();
				 if(NULL != updata_yuv_callback)
				 {
					updata_yuv_callback(0,0,0);
				 }
				JPGEENC_Clear_INT(0x4000);
				JPEG_HWSet_MEA_Buf_ReadOnly(buf_id);
				buf_id = !buf_id;
				SCI_TRACE_LOW("JPEGENC_Poll_MEA_BSM X, buf_id: %d.\n", buf_id);
			 }
			 else
			 {
			 	JPGEENC_Clear_INT(0x4000);
				SCI_TRACE_LOW("JPEGENC_Poll_MEA_BSM X done.\n");
				return 1;
			 }
		}
		if (vsp_time_out_cnt > time)
		{
			SCI_TRACE_LOW("JPEGENC_Poll_MEA_BSM X, fail..\n");
			return 1;
		}
		vsp_time_out_cnt++;
		usleep(1000);
	}	
}

//poll VLC done and BSM done
uint32_t JPEGENC_Poll_VLC_BSM(uint32_t time, uint32_t buf_len,  jpegenc_callback callback)
{
	uint32_t value;
	uint32_t vsp_time_out_cnt = 0;
	
	SCI_TRACE_LOW("JPEGENC_Poll_VLC_BSM E.\n");	
	while (1)
	{
		value = VSP_READ_REG(VSP_DCAM_REG_BASE + DCAM_INT_RAW_OFF, "read the interrupt register.");		
		if(value & 0x100){ //for VLC done			
			JPGEENC_Clear_INT(0x100);
			SCI_TRACE_LOW("JPEGENC_Poll_VLC_BSM X.\n");		
			return 1;
		}
		if(value & 0x80){ //for BSM done		
			JPEGENC_Handle_BSM_INT(callback);		
		}

		if (vsp_time_out_cnt > time)
		{			
			SCI_TRACE_LOW("JPEGENC_Poll_VLC_BSM X, fail.\n");	
			return 1;
		}
		vsp_time_out_cnt++;
		usleep(1000);
	}	
}

uint32_t JPEGENC_encode_one_pic(JPEGENC_PARAMS_T *jpegenc_params,  jpegenc_callback callback)
{
	uint32_t vsp_fd = -1;
	void *vsp_addr = NULL;
	uint32_t ret = 0;
	uint32 value = 0, int_val = 0, temp = 0;
	JPEG_ENC_INPUT_PARA_T input_para_ptr;
	uint32_t slice_height=SLICE_HEIGHT;
	uint32_t slice_num=0;

	g_stream_buf_id = 1;

	slice_height = jpegenc_params->set_slice_height ? jpegenc_params->set_slice_height : SLICE_HEIGHT;
	slice_num = (jpegenc_params->height%slice_height) ? (jpegenc_params->height/slice_height+1):(jpegenc_params->height/slice_height);
	if(0 ==(vsp_fd = open(SPRD_VSP_DRIVER,O_RDWR)))
    	{
        	SCI_TRACE_LOW("JPEGENC open vsp error, vsp_fd: %d.\n", vsp_fd);   
		return -1;
    	}
	else
    	{
        	vsp_addr = mmap(NULL,SPRD_VSP_MAP_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,vsp_fd,0);
		SCI_TRACE_LOW("JPEGENC  vsp addr 0x%x\n",(uint32_t)vsp_addr);
    	}
	
        ret =  ioctl(vsp_fd,VSP_ACQUAIRE,NULL);
	if(ret){
   		 SCI_TRACE_LOW("VSP hardware timeout try again %d\n",ret);	
		ret =  ioctl(vsp_fd,VSP_ACQUAIRE,NULL);
		if(ret){
   			 SCI_TRACE_LOW("VSP hardware timeout give up %d\n",ret);
		 	ret = -1;
			goto error;
		}		 
	}	
       ioctl(vsp_fd,VSP_ENABLE,NULL);
	ioctl(vsp_fd,VSP_RESET,NULL);
	
    	VSP_SetVirtualBaseAddr((uint32)vsp_addr);
    	VSP_reg_reset_callback(VSP_reset_cb,vsp_fd);

	if(JPEG_SUCCESS != JPEGENC_start_encode(jpegenc_params))
	{
		SCI_TRACE_LOW("JPEGENC fail to JPEGENC_start_encode.");
		ret = -1;
		goto error;
	}

	if(jpegenc_params->height > SLICE_HEIGHT)
	{			
		JPEGENC_Poll_MEA_BSM(0xFFF, jpegenc_params->stream_buf_len, slice_num,callback,jpegenc_params->read_callback);		
	}
	
	//poll the end of jpeg encoder
	JPEGENC_Poll_VLC_BSM(0xFFF, jpegenc_params->stream_buf_len, callback);	

	if(JPEG_SUCCESS != JPEGENC_stop_encode(jpegenc_params))
	{
		SCI_TRACE_LOW("JPEGENC fail to JPEGENC_stop_encode.");
		ret = -1;
		goto error;
	}
	if(1 == g_stream_buf_id){
		callback(0, JPEG_HWGetSize(), 1);
	}
	else{
		callback(1, JPEG_HWGetSize(), 1);
	}	

error:
	munmap(vsp_addr,SPRD_VSP_MAP_SIZE);    
	ioctl(vsp_fd,VSP_DISABLE,NULL);
   	ioctl(vsp_fd,VSP_RELEASE,NULL);	
	if(vsp_fd >= 0){
		close(vsp_fd);
	}
		
	return ret;
}

//////////////////////////////////////////////////////////////////////////
#endif //JPEG_ENC
/**---------------------------------------------------------------------------*
**                         Compiler Flag                                      *
**---------------------------------------------------------------------------*/
#ifdef   __cplusplus
    }
#endif
/**---------------------------------------------------------------------------*/
// End 
