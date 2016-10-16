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
#ifndef _JPEG_CODEC_H_
#define _JPEG_CODEC_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "cmr_common.h"
#include "jpeg_exif_header.h"


 #define CMR_JPEG_LOGE(format,...) LOGE(DEBUG_STR format, DEBUG_ARGS, ##__VA_ARGS__)

typedef enum {

	JPEG_SUCCESS = 0,
	JPEG_PARAM_ERR,
	JPEG_INVALID_HANDLE,
	JPEG_NO_MEM,
	JPEG_ERROR

}JPEG_RET;



enum cmr_jpeg_evt {
	CMR_JPEG_DONE = CMR_EVT_JPEG_BASE,
	CMR_JPEG_ERR
};


typedef enum{

	JPEG_YUV_SLICE_ONE_BUF =  0,
	JPEG_YUV_SLICE_MUTI_BUF,

}JPEG_YUV_SLICE_MODE;

//typedef void (*cmr_evt_cb)(int evt, void* data);
//done data
//if error data is not cared.
typedef struct{

	
  	uint32_t      stream_buf_phy;  //stream buffer
	uint32_t      stream_buf_vir;
	uint32_t      stream_size;  //bytes

	uint32_t slice_height;
	uint32_t total_height;  //current total current

}JPEG_ENC_CB_PARAM_T;


typedef struct{

	
  	struct img_frm  *src_img;
	uint32_t slice_height;
	uint32_t total_height;  //current total current

}JPEG_DEC_CB_PARAM_T;

typedef struct
{
	uint32_t max_height;
	uint32_t max_ytouvoffset;
}JPEG_CAPABLIITY_T;


//only support YUV slice, do not support stream slice for simplicity. 
typedef struct{

	uint32_t                            src_fmt;

	struct img_size                     size;
	uint32_t    slice_height;   //slice height must be  8X
	uint32_t    slice_mod;   //JPEG_YUV_SLICE_MODE

	struct img_addr                     src_addr_phy;
	struct img_addr                     src_addr_vir;
	
	struct img_data_end                 src_endian;

	//if slice_height == img height, is the frame mode

	uint32_t   quality_level;
  	uint32_t      stream_buf_phy;
	uint32_t      stream_buf_vir;
	uint32_t      stream_buf_size;  //bytes

	//use by codec
	uint32_t      temp_buf_phy;
	uint32_t      temp_buf_vir;	
	uint32_t      temp_buf_size;  //bytes	

	
}JPEG_ENC_INPUT_PARAM;


typedef struct{

        uint32_t       handle;  //no useful for jpeg code can decode one image at one time.
        				    //only to check the error

}JPEG_ENC_OUTPUT_PARAM;


typedef struct{

	uint32_t handle;// to check the error

	struct img_addr                     src_addr_phy;
	struct img_addr                     src_addr_vir;
	uint32_t   slice_height;  //the slice_height may be different. 
						  //slice height must be 8X


}JPEG_ENC_NXT_PARAM;


typedef struct{

	uint32_t      stream_buf_phy;
	uint32_t      stream_buf_vir;
	uint32_t      stream_buf_size;  //bytes

	struct img_size                     size;
	uint32_t    slice_height;   //slice height must be  8X
	uint32_t    slice_mod;   //JPEG_YUV_SLICE_MODE

	struct img_addr                     dst_addr_phy;
	struct img_addr                     dst_addr_vir;

	struct img_data_end                 dst_endian;

	uint32_t                 dst_fmt;


	//use by codec
	uint32_t      temp_buf_phy;
	uint32_t      temp_buf_vir;	
	uint32_t      temp_buf_size;  //bytes	

}JPEG_DEC_IN_PARAM;


typedef struct
{
	 uint32_t	handle;
}JPEG_DEC_OUT_PARAM;


typedef struct{

	uint32_t handle;

	struct img_addr                     dst_addr_phy;
	struct img_addr                     dst_addr_vir;
	uint32_t        slice_height; 

}JPEG_DEC_NXT_PARAM;



int jpeg_init(void);

void jpeg_getcapability(JPEG_CAPABLIITY_T *param_ptr);

int jpeg_enc_start(JPEG_ENC_INPUT_PARAM *start_in_parm_ptr, JPEG_ENC_OUTPUT_PARAM *start_out_parm_ptr);

int jpeg_enc_next(JPEG_ENC_NXT_PARAM *nxt_param_ptr);

int jpeg_dec_start(JPEG_DEC_IN_PARAM *start_in_parm_ptr, JPEG_DEC_OUT_PARAM *start_out_parm_ptr);

int jpeg_dec_next(JPEG_DEC_NXT_PARAM *next_param_ptr);

int jpeg_stop(uint32_t handle);

int jpeg_deinit(void);

void jpeg_evt_reg(cmr_evt_cb  adp_event_cb);


#ifdef __cplusplus
}
#endif

#endif //for _JPEG_CODEC_H_


