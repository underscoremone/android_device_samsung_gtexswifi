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

#include "cmr_msg.h"
#include "jpeg_codec.h"
#include "cmr_common.h"
#include "jpegdec_api.h"
#include "jpegenc_api.h"


#define JPEG_MSG_QUEUE_SIZE		10
#define JPEG_EXIT_THREAD_FLAG	1
#define JPEG_EVT_ENC_START	(1 << 16)
#define JPEG_EVT_ENC_NEXT	(1 << 17)
#define JPEG_EVT_DEC_START	(1 << 18)
#define JPEG_EVT_DEC_NEXT	(1 << 19)
#define JPEG_EVT_STOP		(1 << 20)
#define JPEG_EVT_KILL		(1 << 21)


#define JPEG_EVT_MASK_BITS	(uint32_t)(JPEG_EVT_ENC_START | JPEG_EVT_ENC_NEXT | \
	JPEG_EVT_DEC_START | JPEG_EVT_DEC_NEXT | JPEG_EVT_STOP | JPEG_EVT_KILL)

typedef struct
{
	pthread_t	v4l2_thread;
	sem_t       stop_sem;
	uint32_t	msg_queue_handle;
	uint32_t	active_handle;
	uint32_t    is_exit_thread;
	cmr_evt_cb  event_cb;
}JPEG_CONTEXT_T;


typedef struct 
{
	uint32_t handle;
	uint32_t type;/*0: enc; 1: dec*/
}JPEG_HANDLE_T;

typedef struct
{
	uint32_t    stream_buf_phy;
	uint32_t    stream_buf_vir;
	uint32_t	stream_buf_size;

	struct img_size	size;
	uint32_t    slice_height;/*slice height must be  8X*/
	uint32_t    slice_mod;/*PEG_YUV_SLICE_MODE*/

	struct img_addr     dst_addr_phy;
	struct img_addr     dst_addr_vir;
	struct img_data_end	dst_endian;

	uint32_t    dst_fmt;

	//use by codec
	uint32_t    temp_buf_phy;
	uint32_t    temp_buf_vir;
	uint32_t	temp_buf_size;
	uint32_t    cur_line_num;
	uint32_t	is_finish; /*0: on going; 1: finisned*/
}JPEG_DEC_T;

typedef struct
{
	uint32_t			src_fmt;
	struct img_size     size;
	uint32_t			slice_height;/*slice height must be  8X, if slice_height == img height, is the frame mode*/
	uint32_t			slice_mod;/*JPEG_YUV_SLICE_MODE*/

	struct img_addr     src_addr_phy;
	struct img_addr     src_addr_vir;
	struct img_data_end src_endian;

	uint32_t    quality_level;
	uint32_t    stream_buf_phy;
	uint32_t    stream_buf_vir;
	uint32_t    stream_buf_size;/*bytes*/

	uint32_t	stream_real_size;
	//use by codec
	uint32_t    temp_buf_phy;
	uint32_t    temp_buf_vir;
	uint32_t	temp_buf_size;/*bytes*/

	//use by codec
	uint32_t    ping_buf_y_phy;
	uint32_t    ping_buf_y_vir;
	uint32_t    ping_buf_u_phy;
	uint32_t	ping_buf_u_vir;

	uint32_t    pang_buf_y_phy;
	uint32_t    pang_buf_y_vir;
	uint32_t    pang_buf_u_phy;
	uint32_t    pang_buf_u_vir;
	uint32_t    cur_line_num;
	uint32_t    cur_id; /*0: ping; 1: pang*/
	uint32_t	is_finish;   /*0: on going; 1: finished*/
}JPEG_ENC_T;

static JPEG_CONTEXT_T  jcontext;
static void* _thread_proc(void* data);
static int _kill_thread(void);

static void savedata(uint32_t buf_addr, uint32_t size) 
{
	FILE *fp = NULL;

	LOGE("jpeg: savedata\n");
	fp = fopen("/data/out.raw", "wb");
	if(0 != fp)
	{
		fwrite((void*)buf_addr, 1, size, fp);
		fclose(fp);
	}else{

		LOGE("can not create savedata\n");
	}

}

static void save_inputdata(uint32_t y_buf_addr,
							uint32_t uv_buf_addr,
							uint32_t size)
{
	FILE *fp = NULL;

	LOGE("jpeg: save input data,size=%d.\n",size);
	fp = fopen("/data/in_y.raw", "wb");
	if(0 != fp) {
		fwrite((void*)y_buf_addr, 1, size, fp);
		fclose(fp);
	}else{
		LOGE("can not create savedata\n");
	}
	fp = fopen("/data/in_uv.raw", "wb");
	if(0 != fp) {
		fwrite((void*)uv_buf_addr, 1, size/2, fp);
		fclose(fp);
	}else{
		LOGE("can not create savedata\n");
	}
}

static uint32_t _format_covert(uint32_t format)
{
	uint32_t jfmt = JPEGENC_YUV_420;
	switch(format)
	{

	case IMG_DATA_TYPE_YUV422:

		jfmt = JPEGENC_YUV_422;
		
	break;

	default:

		LOGV("JPEG, unknow format\n");

	break;

	}

	return jfmt;
}


static uint32_t _quality_covert(uint32_t quality)
{
	uint32_t jq = JPEGENC_QUALITY_HIGH;
	
	return jq;
}

void _prc_enc_cbparam(uint32_t handle, JPEG_ENC_CB_PARAM_T *parm_ptr)
{
	JPEG_ENC_T *enc_cxt_ptr = NULL;

	enc_cxt_ptr = (JPEG_ENC_T * )handle;
	parm_ptr->stream_buf_phy = enc_cxt_ptr->stream_buf_phy;
	parm_ptr->stream_buf_vir = enc_cxt_ptr->stream_buf_vir;
	parm_ptr->stream_size = enc_cxt_ptr->stream_real_size;
	
	parm_ptr->slice_height = enc_cxt_ptr->slice_height;
	parm_ptr->total_height = enc_cxt_ptr->size.height;
	CMR_LOGV("slice_height %d,total_height %d,stream_size 0x%d.",
		parm_ptr->slice_height,parm_ptr->total_height,parm_ptr->stream_size);
	return;
}

void _memcpy_endian_uvconvert(uint32_t dst_buf_addr, uint32_t src_buf_addr, uint32_t size)
{

#if 1
	uint32_t i = 0;
	uint8_t  * src_ptr = (uint8_t  *)src_buf_addr;
	uint8_t  * dst_ptr = (uint8_t  *)dst_buf_addr;

	for(i=0; i<size; i=i+2)
	{
		*dst_ptr++ = src_ptr[i+1];
		*dst_ptr++ = src_ptr[i];
	}
#else

	memcpy(dst_buf_addr, src_buf_addr, size);
#endif
	
}


static  int  _enc_start(uint32_t handle)
{
	int ret = JPEG_SUCCESS;
	uint32_t jpeg_enc_buf_phys_addr;
	uint32_t *jpeg_enc_buf_virt_addr;
	uint32_t jpeg_enc_buf_len;
	uint32_t i;
	uint32_t jpeg_ret = 0;
	JPEG_ENC_T *enc_cxt_ptr = NULL;
	JPEGENC_SLICE_OUT_T slice_out;
	JPEGENC_SLICE_NEXT_T next_slice_parm;
	JPEGENC_PARAMS_T *jenc_parm_ptr = (JPEGENC_PARAMS_T *)malloc(sizeof(JPEGENC_PARAMS_T));

	LOGE("jpeg: _encoder_start: S\n");
	memset((void*)&slice_out,0,sizeof(JPEGENC_SLICE_OUT_T));
	memset((void*)&next_slice_parm,0,sizeof(JPEGENC_SLICE_NEXT_T));
	if(NULL == jenc_parm_ptr){
		return JPEG_NO_MEM;
	}

	enc_cxt_ptr = (JPEG_ENC_T * )handle;
	if(enc_cxt_ptr->slice_height == enc_cxt_ptr->size.height ) {
		jenc_parm_ptr->set_slice_height = 32;//enc_cxt_ptr->slice_height;
	} else{
		jenc_parm_ptr->set_slice_height = enc_cxt_ptr->slice_height;
	}
#if 0
	enc_cxt_ptr->ping_buf_y_phy = enc_cxt_ptr->temp_buf_phy;
	enc_cxt_ptr->ping_buf_y_vir = enc_cxt_ptr->temp_buf_vir;
	enc_cxt_ptr->ping_buf_u_phy = enc_cxt_ptr->temp_buf_phy + enc_cxt_ptr->size.width *  jenc_parm_ptr->set_slice_height;
	enc_cxt_ptr->ping_buf_u_vir = enc_cxt_ptr->temp_buf_vir +  enc_cxt_ptr->size.width *  jenc_parm_ptr->set_slice_height;

	//current, support YUV420 only.
	enc_cxt_ptr->pang_buf_y_phy = enc_cxt_ptr->ping_buf_u_phy+ enc_cxt_ptr->size.width * jenc_parm_ptr->set_slice_height/2;
	enc_cxt_ptr->pang_buf_y_vir = enc_cxt_ptr->ping_buf_u_vir +  enc_cxt_ptr->size.width *  jenc_parm_ptr->set_slice_height/2;
	enc_cxt_ptr->pang_buf_u_phy = enc_cxt_ptr->pang_buf_y_phy+ enc_cxt_ptr->size.width *  jenc_parm_ptr->set_slice_height;
	enc_cxt_ptr->pang_buf_u_vir = enc_cxt_ptr->pang_buf_y_vir +  enc_cxt_ptr->size.width * jenc_parm_ptr->set_slice_height;	
#endif
	jenc_parm_ptr->format = JPEGENC_YUV_420;//_format_covert(enc_cxt_ptr->src_fmt);
	jenc_parm_ptr->quality = _quality_covert(enc_cxt_ptr->quality_level);
	jenc_parm_ptr->width = enc_cxt_ptr->size.width;
	jenc_parm_ptr->height = enc_cxt_ptr->size.height;
#if 0
	jenc_parm_ptr->yuv_virt_buf =  (void *)enc_cxt_ptr->temp_buf_vir;
	jenc_parm_ptr->yuv_phy_buf = enc_cxt_ptr->temp_buf_phy;
#else
	jenc_parm_ptr->yuv_virt_buf =  (void *)enc_cxt_ptr->src_addr_vir.addr_y;
	jenc_parm_ptr->yuv_phy_buf = enc_cxt_ptr->src_addr_phy.addr_y;
	jenc_parm_ptr->yuv_u_virt_buf = (void *)enc_cxt_ptr->src_addr_vir.addr_u;
	jenc_parm_ptr->yuv_u_phy_buf = enc_cxt_ptr->src_addr_phy.addr_u;
	jenc_parm_ptr->yuv_v_virt_buf = (void*)0;//(void *)enc_cxt_ptr->src_addr_vir.addr_v;
	jenc_parm_ptr->yuv_v_phy_buf = 0;//enc_cxt_ptr->src_addr_phy.addr_v;
#endif

	LOGE("jpeg enc yuv phy addr,0x%x 0x%x 0x%x",
		jenc_parm_ptr->yuv_phy_buf,
		jenc_parm_ptr->yuv_u_phy_buf,
		jenc_parm_ptr->yuv_v_phy_buf);

	jpeg_enc_buf_virt_addr = (void *)enc_cxt_ptr->stream_buf_vir;
	jpeg_enc_buf_len = enc_cxt_ptr->stream_buf_size;
	jpeg_enc_buf_phys_addr =  enc_cxt_ptr->stream_buf_phy;

/*	save_inputdata(enc_cxt_ptr->src_addr_vir.addr_y,
					enc_cxt_ptr->src_addr_vir.addr_u,
					enc_cxt_ptr->size.width*enc_cxt_ptr->size.height);
*/
#if 0
	memcpy(enc_cxt_ptr->ping_buf_y_vir,  enc_cxt_ptr->src_addr_vir.addr_y, enc_cxt_ptr->size.width*jenc_parm_ptr->set_slice_height);
	_memcpy_endian_uvconvert(enc_cxt_ptr->ping_buf_u_vir,  enc_cxt_ptr->src_addr_vir.addr_u, enc_cxt_ptr->size.width*jenc_parm_ptr->set_slice_height/2);
#endif
	jenc_parm_ptr->stream_virt_buf[0] = jpeg_enc_buf_virt_addr;
	jenc_parm_ptr->stream_phy_buf[0] = jpeg_enc_buf_phys_addr;
	LOGE("encoder: jpegenc_params[%d]: virt: %x, phys: %x.",i,(uint32_t)jenc_parm_ptr->stream_virt_buf[i],jenc_parm_ptr->stream_phy_buf[i]);

	jenc_parm_ptr->stream_buf_len = jpeg_enc_buf_len;
	jenc_parm_ptr->stream_size = 0;

	//start jpeg enc for both slice and frame
	if(0 != JPEGENC_Slice_Start(jenc_parm_ptr, &slice_out)){
		ret =  JPEG_ERROR;
		goto enc_start_end;
	}
	//if frame, and still use slice mode for sc8810 to keep the same  interface to top layer
	if(enc_cxt_ptr->slice_height == enc_cxt_ptr->size.height ){
		uint32_t cur_slice_height = jenc_parm_ptr->set_slice_height;
		uint32_t slice_num =  enc_cxt_ptr->size.height/cur_slice_height;
		uint32_t cur_ver_pos = 0;
		uint32_t buf_id = 1;
		uint32_t cur_y_buf_adr = 0;
		uint32_t cur_u_buf_adr = 0;

		LOGE("jpeg: slice mode for frame \n.");

		if(0 !=  enc_cxt_ptr->size.height%cur_slice_height){
			slice_num = slice_num+1;
		}

		LOGE("jpeg: slice num: %d\n", slice_num);
		slice_num--;/*start has process one slice*/
		//cur_ver_pos = cur_slice_height;
		do {
			if(enc_cxt_ptr->size.height <(cur_ver_pos+  jenc_parm_ptr->set_slice_height)) {
				cur_slice_height = enc_cxt_ptr->size.height - cur_ver_pos;
			}
			cur_ver_pos = cur_ver_pos + cur_slice_height;

			if(0 == buf_id) {
				buf_id = 1;
				/*cur_y_buf_adr = enc_cxt_ptr->ping_buf_y_vir;*/
				/*cur_u_buf_adr = enc_cxt_ptr->ping_buf_u_vir;*/
			} else{
				buf_id = 0;
				/*cur_y_buf_adr = enc_cxt_ptr->pang_buf_y_vir;*/
				/*cur_u_buf_adr = enc_cxt_ptr->pang_buf_u_vir;*/
			}
			LOGE("jpeg: slice num: y cur: 0x%x\n", cur_y_buf_adr);
			LOGE("jpeg: slice num: u cur: 0x%x\n", cur_u_buf_adr);
			LOGE("jpeg: slice num: cur slice height: %d\n", cur_slice_height);
			LOGE("jpeg: slice num:cur_ver_pos: %d\n", cur_ver_pos);
#if 0
			memcpy((void *)cur_y_buf_adr, (void *)(enc_cxt_ptr->src_addr_vir.addr_y+ cur_ver_pos* enc_cxt_ptr->size.width), enc_cxt_ptr->size.width*cur_slice_height);
			_memcpy_endian_uvconvert((void *)cur_u_buf_adr, (void *)(enc_cxt_ptr->src_addr_vir.addr_u+ cur_ver_pos* enc_cxt_ptr->size.width/2), enc_cxt_ptr->size.width*cur_slice_height/2);
			jpeg_ret = JPEGENC_Slice_Next(0, &slice_out);
#else
			next_slice_parm.slice_height = jenc_parm_ptr->set_slice_height;
			next_slice_parm.yuv_phy_buf = enc_cxt_ptr->src_addr_phy.addr_y+cur_ver_pos* enc_cxt_ptr->size.width;
			next_slice_parm.yuv_u_phy_buf = enc_cxt_ptr->src_addr_phy.addr_u+cur_ver_pos* enc_cxt_ptr->size.width/2;
			next_slice_parm.yuv_v_phy_buf = 0;
			jpeg_ret = JPEGENC_Slice_Next(&next_slice_parm, &slice_out);
#endif
			if(0 != jpeg_ret && 1 != jpeg_ret) {
				CMR_LOGE("JPEGENC_Slice_Next error,ret %d.",jpeg_ret);
				ret =  JPEG_ERROR;
				break;
			}
			if(1 == slice_out.is_over) {
				enc_cxt_ptr->is_finish = 1;
				enc_cxt_ptr->stream_real_size = slice_out.stream_size;
				LOGE("jpeg: slice num:slice size: %d", slice_out.stream_size);
				break;
			}
			slice_num--;
		}while(0 <  slice_num);
		CMR_LOGV("slice_num %d.",slice_num);
	}

	enc_cxt_ptr->cur_id = 0;
	enc_cxt_ptr->cur_line_num = jenc_parm_ptr->set_slice_height;

	LOGE("jpeg:  buf addr: 0x%x,  size: %d",enc_cxt_ptr->stream_buf_vir, enc_cxt_ptr->stream_real_size);
/*	savedata(enc_cxt_ptr->stream_buf_vir, enc_cxt_ptr->stream_real_size);*/
enc_start_end:
	free(jenc_parm_ptr);
	LOGE("jpeg: _encoder_start E.");
	return ret;
}


//assume the slice height is the same, except the last one
static  int  _enc_next(uint32_t handle, JPEG_ENC_NXT_PARAM *param_ptr)
{
	int ret = JPEG_SUCCESS;
	uint32_t jpeg_enc_buf_phys_addr;
	uint32_t *jpeg_enc_buf_virt_addr;
	uint32_t jpeg_enc_buf_len;
	uint32_t i;
	uint32_t cur_line_num = 0 ;//enc_cxt_ptr->cur_line_num;
	uint32_t cur_slice_height;
	uint32_t slice_num =  0;
	uint32_t cur_ver_pos = 0;
	uint32_t buf_id = 0;
	uint32_t cur_y_buf_adr = 0;
	uint32_t cur_u_buf_adr = 0;
	JPEGENC_SLICE_NEXT_T update_parm;
	JPEG_ENC_T *enc_cxt_ptr = NULL;
	JPEGENC_SLICE_OUT_T slice_out;
	 
	LOGV("jpeg: _enc_next: S\n");
	memset((void*)&update_parm,0,sizeof(JPEGENC_SLICE_NEXT_T));
	memset((void*)&slice_out,0,sizeof(JPEGENC_SLICE_OUT_T));

	if(NULL != param_ptr) {
		if(0 != param_ptr->src_addr_phy.addr_y) {
			update_parm.yuv_phy_buf = param_ptr->src_addr_phy.addr_y;
			update_parm.yuv_u_phy_buf = param_ptr->src_addr_phy.addr_u;
			update_parm.yuv_v_phy_buf = param_ptr->src_addr_phy.addr_v;
			update_parm.slice_height = param_ptr->slice_height;
		}
	}else {
		enc_cxt_ptr = (JPEG_ENC_T * )handle;
		cur_line_num = enc_cxt_ptr->cur_line_num;
		cur_slice_height =  enc_cxt_ptr->slice_height;
		slice_num =  enc_cxt_ptr->size.width/cur_slice_height;
		buf_id = enc_cxt_ptr->cur_id;

		if(enc_cxt_ptr->cur_line_num >=  enc_cxt_ptr->size.height) {
			return JPEG_ERROR;
		}

		if(enc_cxt_ptr->size.height <(cur_line_num+ enc_cxt_ptr->slice_height)) {
			cur_slice_height = enc_cxt_ptr->size.height - cur_line_num;
		}
		enc_cxt_ptr->cur_line_num = enc_cxt_ptr->cur_line_num+cur_slice_height;

		if(0 == buf_id){
			buf_id = 1;
			cur_y_buf_adr = enc_cxt_ptr->ping_buf_y_vir;
			cur_u_buf_adr = enc_cxt_ptr->ping_buf_u_vir;
		}else{
			buf_id = 0;
			cur_y_buf_adr = enc_cxt_ptr->pang_buf_y_vir;
			cur_u_buf_adr = enc_cxt_ptr->pang_buf_u_vir;
		}
		enc_cxt_ptr->cur_id = buf_id;
#if 0
		memcpy(cur_y_buf_adr,  param_ptr->src_addr_vir.addr_y, enc_cxt_ptr->size.width*cur_slice_height);
		memcpy(cur_u_buf_adr, param_ptr->src_addr_vir.addr_u, enc_cxt_ptr->size.width*cur_slice_height/2);
#endif
		update_parm.slice_height = enc_cxt_ptr->slice_height;
		update_parm.yuv_phy_buf = enc_cxt_ptr->src_addr_phy.addr_y;
		update_parm.yuv_u_phy_buf = enc_cxt_ptr->src_addr_phy.addr_u;
		update_parm.yuv_v_phy_buf = enc_cxt_ptr->src_addr_phy.addr_v;
	}

 	//encode the jpeg picture by HW.
	if(0 != JPEGENC_Slice_Next(&update_parm, &slice_out)) {
		ret =  JPEG_ERROR;
	}
	
	if(1 == slice_out.is_over){
		enc_cxt_ptr->is_finish = 1;
		enc_cxt_ptr->stream_real_size = slice_out.stream_size;
	}

	LOGV("jpeg: _encoder_start E.");
	return ret;
}

static int _decoder(JPEGDEC_INPUT_PARAMS_T  *input_param)
{
	LOGV("jpeg codec: decoder S.");
	long 	pos;
	uint32_t 	jpeg_dec_buf_len;
	int 		fd;
	int             ret =0;
	JPEGDEC_PARAMS_T  jpegdec_params;

	jpegdec_params.format = input_param->format;
	jpegdec_params.width =   input_param->width;
	jpegdec_params.height = input_param->height;
	jpegdec_params.stream_size = input_param->src_size;
	jpegdec_params.src_buf = input_param->src_virt_buf;
	jpegdec_params.src_phy_buf = input_param->src_phy_buf;
	jpegdec_params.target_buf_Y= input_param->target_virt_buf_Y;
	jpegdec_params.target_phy_buf_Y= input_param->target_phy_buf_Y;
	jpegdec_params.target_buf_UV = input_param->target_virt_buf_UV;
	jpegdec_params.target_phy_buf_UV = input_param->target_phy_buf_UV;

	jpeg_dec_buf_len = jpegdec_params.width*jpegdec_params.height;
	jpegdec_params.temp_buf_len = input_param->src_size + 256;

	jpegdec_params.fw_decode_buf = malloc( 20*1024);
	jpegdec_params.fw_decode_buf_size  = 20*1024;

	jpegdec_params.stream_virt_buf[0] = jpegdec_params.src_buf;
	jpegdec_params.stream_phy_buf[0]  = jpegdec_params.src_phy_buf;

	jpegdec_params.stream_virt_buf[1] =  0;
	jpegdec_params.stream_buf_len = jpegdec_params.stream_size;
	jpegdec_params.yuv_virt_buf      = jpegdec_params.target_buf_Y;
	jpegdec_params.yuv_phy_buf    = jpegdec_params.target_phy_buf_Y;
	jpegdec_params.set_slice_height = input_param->slice_height;
	LOGV("camera_decode_one_pic:set_slice_height=%d.",jpegdec_params.set_slice_height);

	ret = JPEGDEC_decode_one_pic(&jpegdec_params,input_param->write_yuv_callback);
	free(jpegdec_params.fw_decode_buf);
	LOGV("jpeg codec: decoder E: %d.",ret);
	return ret;
}


static int _dec_start(uint32_t handle)
{
	int ret = JPEG_SUCCESS;
	JPEG_DEC_T *dec_cxt_ptr = (JPEG_DEC_T *)handle;
	JPEGDEC_INPUT_PARAMS_T  *input_param_ptr  = (JPEGDEC_INPUT_PARAMS_T  *)malloc(sizeof(JPEGDEC_INPUT_PARAMS_T ));


	if(NULL == input_param_ptr){

		return JPEG_NO_MEM;
	}

	//only support slice mode on 8810
	if(1) {
		if(dec_cxt_ptr->size.height == dec_cxt_ptr->slice_height){
			dec_cxt_ptr->slice_height = 128;
		}
	}

	if(dec_cxt_ptr->size.height >  dec_cxt_ptr->slice_height){  //slice mode
		input_param_ptr->target_phy_buf_Y = dec_cxt_ptr->temp_buf_phy;
		input_param_ptr->target_phy_buf_UV = dec_cxt_ptr->temp_buf_phy;
		input_param_ptr->target_virt_buf_Y = (void *)(dec_cxt_ptr->temp_buf_vir);
		input_param_ptr->target_virt_buf_UV = (void *)(dec_cxt_ptr->temp_buf_vir+dec_cxt_ptr->slice_height*dec_cxt_ptr->size.width);

		 input_param_ptr->slice_height =  dec_cxt_ptr->slice_height;
	}
	else{  //frame mode
		input_param_ptr->target_phy_buf_Y = dec_cxt_ptr->dst_addr_phy.addr_y;
		input_param_ptr->target_phy_buf_UV = dec_cxt_ptr->dst_addr_phy.addr_u;
		input_param_ptr->target_virt_buf_Y = (void *)dec_cxt_ptr->dst_addr_vir.addr_y;
		input_param_ptr->target_virt_buf_UV = (void *)dec_cxt_ptr->dst_addr_vir.addr_u;
	}

	input_param_ptr->crop_x = 0;
	input_param_ptr->crop_y = 0;
	input_param_ptr->crop_width = dec_cxt_ptr->size.width;
	input_param_ptr->crop_height = dec_cxt_ptr->size.height;

	input_param_ptr->format = JPEGDEC_YUV_422;  //
	input_param_ptr->src_phy_buf = dec_cxt_ptr->stream_buf_phy;
	input_param_ptr->src_virt_buf = (void *)dec_cxt_ptr->stream_buf_vir;
	input_param_ptr->src_size =  dec_cxt_ptr->stream_buf_size;

	input_param_ptr->width = dec_cxt_ptr->size.width;
	input_param_ptr->height = dec_cxt_ptr->size.height; 
	input_param_ptr->write_yuv_callback = 0;  //maybe need to add some thing.

	if(-1 == _decoder(input_param_ptr))
	{
		free(input_param_ptr);
		ret = JPEG_ERROR;
	}



	return ret;
	
}

static int   _create_thread(void)
{
	int                      ret = 0;
	pthread_attr_t           attr;

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	ret = pthread_create(&jcontext.v4l2_thread, &attr, _thread_proc, NULL);	
	pthread_attr_destroy(&attr);
	return ret;
}

static int _kill_thread(void)
{
	int                      ret = 0;
	char                     write_ch;
	void                     *dummy;	

	//CMR_CHECK_FD;
	
	CMR_LOGV("Call write function to kill v4l2 manage thread");
	
	//ret = write(fd, &write_ch, 1); // kill thread;
//	if (ret > 0) {
		CMR_LOGV("write OK!");
		ret = pthread_join(jcontext.v4l2_thread, &dummy);
//	}
	CMR_LOGV("kill jpeg thread result %d.\n",ret);
	return ret;		
}

static int _jpeg_stop(uint32_t handle)
{
	JPEG_HANDLE_T *handle_ptr = (JPEG_HANDLE_T*)handle;

	CMR_JPEG_LOGE("_jpeg_stop start\n");
	if(0 == handle_ptr){
		return JPEG_PARAM_ERR;
	}

	if(0 == handle_ptr->type){
		JPEG_ENC_T *enc_cxt_ptr = (JPEG_ENC_T*)handle_ptr->handle;
		if(NULL != enc_cxt_ptr) {
			free(enc_cxt_ptr);
			handle_ptr->handle = 0;
		}
	}else if(1 == handle_ptr->type){
		JPEG_DEC_T *dec_cxt_ptr = (JPEG_DEC_T*)handle_ptr->handle;
		if(NULL != dec_cxt_ptr) {
			free(dec_cxt_ptr);
			handle_ptr->handle = 0;
		}
	} else {
		return JPEG_PARAM_ERR;
	}

	return JPEG_SUCCESS;
}

static void* _thread_proc(void* data)
{
	int                      evt_id;
	int ret;
	uint32_t evt;
	uint32_t handle = 0;
	JPEG_HANDLE_T *handle_ptr = NULL;
	JPEG_ENC_NXT_PARAM *param_ptr = NULL;
	CMR_MSG_INIT(message);
	LOGE("JPEG Thread In \n");

	while(1) {

		ret = cmr_msg_get(jcontext.msg_queue_handle, &message);

		if (ret) {
			CMR_LOGE("jpeg: message queue destroied\n");
			break;
		}

		CMR_LOGE("jpeg: message.msg_type 0x%x\n", message.msg_type);
		evt = (uint32_t)(message.msg_type & JPEG_EVT_MASK_BITS);

		switch(evt){
		case  JPEG_EVT_ENC_START:
			handle = (uint32_t )message.data;
			ret = _enc_start(handle);

			if(JPEG_SUCCESS == ret){
				JPEG_ENC_CB_PARAM_T param;
				memset((void*)&param,0,sizeof(JPEG_ENC_CB_PARAM_T));
				_prc_enc_cbparam(handle, &param);
				jcontext.event_cb(CMR_JPEG_DONE, &param );
			} else{
				jcontext.event_cb(CMR_JPEG_ERR, NULL );
			}

			CMR_LOGE("jpeg:receive JPEG_EVT_ENC_START message\n");
			break;
		case  JPEG_EVT_ENC_NEXT:
			param_ptr = (JPEG_ENC_NXT_PARAM*)message.data;
			handle_ptr = (JPEG_HANDLE_T*)param_ptr->handle;
			handle = handle_ptr->handle;
			if(0 != message.data) {
				ret = _enc_next( handle, (JPEG_ENC_NXT_PARAM *)message.data);
			}

			if(JPEG_SUCCESS == ret){
				JPEG_ENC_CB_PARAM_T param;
				memset((void*)&param,0,sizeof(JPEG_ENC_CB_PARAM_T));
				_prc_enc_cbparam(handle, &param);
				jcontext.event_cb(CMR_JPEG_DONE, &param );
			} else {
				jcontext.event_cb(CMR_JPEG_ERR, NULL );
			}
			break;
		case  JPEG_EVT_DEC_START:
			handle = (uint32_t )message.data;
			ret = _dec_start(handle);
			break;
		case  JPEG_EVT_DEC_NEXT:
			break;
		case  JPEG_EVT_STOP:
			handle = (uint32_t )message.data;
			ret = _jpeg_stop(handle);
			sem_post(&jcontext.stop_sem);
			break;
		case JPEG_EVT_KILL:
			ret = _jpeg_stop(jcontext.is_exit_thread);
			jcontext.is_exit_thread = JPEG_EXIT_THREAD_FLAG;
			break;
		default:
			CMR_LOGE("jpeg: not correct message");
			break;
		}

		if(1 == message.alloc_flag){
			free(message.data);
		}
		CMR_LOGV("in thread proc \n");
		if(JPEG_EXIT_THREAD_FLAG == jcontext.is_exit_thread) {
			break;
		}
	}
		
	CMR_LOGV("JPEG Thrad Out \n");
	return NULL;
}


int jpeg_init(void)
{
	int ret = JPEG_SUCCESS;

	memset(&jcontext,0,sizeof(JPEG_CONTEXT_T));
	sem_init(&jcontext.stop_sem, 0, 0);
	ret = _create_thread();
	ret = cmr_msg_queue_create(JPEG_MSG_QUEUE_SIZE, &jcontext.msg_queue_handle);

	if(CMR_MSG_SUCCESS != ret) {
		ret = JPEG_ERROR;
	}

	return ret;
}

static int _check_enc_start_param(JPEG_ENC_INPUT_PARAM *in_parm_ptr, JPEG_ENC_OUTPUT_PARAM *out_parm_ptr)
{
	int ret = JPEG_SUCCESS;

	CMR_LOGE("w h, %d %d, quality level %d", in_parm_ptr->size.width, in_parm_ptr->size.height,
		in_parm_ptr->quality_level);
	CMR_LOGE("slice height, %d, slice mode %d", in_parm_ptr->slice_height, in_parm_ptr->slice_mod);
	CMR_LOGE("phy addr 0x%x 0x%x, vir addr 0x%x 0x%x",
		in_parm_ptr->src_addr_phy.addr_y, in_parm_ptr->src_addr_phy.addr_u,
		in_parm_ptr->src_addr_vir.addr_y, in_parm_ptr->src_addr_vir.addr_u);
	CMR_LOGE("endian %d %d", in_parm_ptr->src_endian.y_endian, in_parm_ptr->src_endian.uv_endian);
	CMR_LOGE("stream phy 0x%x vir 0x%x, size 0x%x",
		in_parm_ptr->stream_buf_phy,
		in_parm_ptr->stream_buf_vir,
		in_parm_ptr->stream_buf_size);
	CMR_LOGE("temp_buf phy 0x%x vir 0x%x, size 0x%x",
		in_parm_ptr->temp_buf_phy,
		in_parm_ptr->temp_buf_vir,
		in_parm_ptr->temp_buf_size);
	
	return ret;
	
}


static int _get_enc_start_param(JPEG_ENC_T *cxt_ptr, JPEG_ENC_INPUT_PARAM *in_parm_ptr, JPEG_ENC_OUTPUT_PARAM *out_parm_ptr)
{
	int ret = JPEG_SUCCESS;

	CMR_JPEG_LOGE("jpeg_enc_start:all param\n");

	
	CMR_JPEG_LOGE("stream_buf_phy: 0x%x\n",  in_parm_ptr->stream_buf_phy);
	CMR_JPEG_LOGE("stream_buf_vir: 0x%x\n",  in_parm_ptr->stream_buf_vir);
	CMR_JPEG_LOGE("stream_buf_size: 0x%x\n",  in_parm_ptr->stream_buf_size);
	CMR_JPEG_LOGE("temp_buf_phy: 0x%x\n",  in_parm_ptr->temp_buf_phy);
	CMR_JPEG_LOGE("temp_buf_vir: 0x%x\n",  in_parm_ptr->temp_buf_vir);
	CMR_JPEG_LOGE("temp_buf_size: 0x%x\n",  in_parm_ptr->temp_buf_size);
	CMR_JPEG_LOGE("src_buf_phy: 0x%x\n",  in_parm_ptr->temp_buf_phy);
	CMR_JPEG_LOGE("src_buf_vir: 0x%x\n",  in_parm_ptr->temp_buf_vir);
	CMR_JPEG_LOGE("img_size: w:%d, h:%d\n",  in_parm_ptr->size.width,  in_parm_ptr->size.height);
	CMR_JPEG_LOGE("slice_height:%d\n",  in_parm_ptr->slice_height);

	cxt_ptr->stream_buf_phy = in_parm_ptr->stream_buf_phy;
	cxt_ptr->stream_buf_vir =  in_parm_ptr->stream_buf_vir;
	cxt_ptr->stream_buf_size =  in_parm_ptr->stream_buf_size;

	cxt_ptr->temp_buf_phy = in_parm_ptr->temp_buf_phy;
	cxt_ptr->temp_buf_vir = in_parm_ptr->temp_buf_vir;
	cxt_ptr->temp_buf_size = in_parm_ptr->temp_buf_size;

	cxt_ptr->src_addr_phy =  in_parm_ptr->src_addr_phy;
	cxt_ptr->src_addr_vir =  in_parm_ptr->src_addr_vir;
	cxt_ptr->src_endian  =  in_parm_ptr->src_endian;

	cxt_ptr->src_fmt = in_parm_ptr->src_fmt;
	
	cxt_ptr->quality_level = in_parm_ptr->quality_level;

	cxt_ptr->size = in_parm_ptr->size;

	cxt_ptr->slice_height = in_parm_ptr->slice_height;
	cxt_ptr->slice_mod = in_parm_ptr->slice_mod;

	CMR_JPEG_LOGE("jpeg_enc_start:end\n");
	
	return ret;
	
}

void jpeg_getcapability(JPEG_CAPABLIITY_T *param_ptr)
{
	param_ptr->max_height = 1024;
	param_ptr->max_ytouvoffset = 1024;
}

static int _check_dec_start_param(JPEG_DEC_IN_PARAM *start_in_parm_ptr, JPEG_DEC_OUT_PARAM *start_out_parm_ptr)
{
	int ret = JPEG_SUCCESS;
	return ret;
}

static int _get_dec_start_param(JPEG_DEC_T *cxt_ptr, JPEG_DEC_IN_PARAM *in_parm_ptr, JPEG_DEC_OUT_PARAM *out_parm_ptr)
{
	int ret = JPEG_SUCCESS;

	cxt_ptr->stream_buf_phy = in_parm_ptr->stream_buf_phy;
	cxt_ptr->stream_buf_vir =  in_parm_ptr->stream_buf_vir;
	cxt_ptr->stream_buf_size =  in_parm_ptr->stream_buf_size;

	cxt_ptr->temp_buf_phy = in_parm_ptr->temp_buf_phy;
	cxt_ptr->temp_buf_vir = in_parm_ptr->temp_buf_vir;
	cxt_ptr->temp_buf_size = in_parm_ptr->temp_buf_size;

	cxt_ptr->dst_addr_phy =  in_parm_ptr->dst_addr_phy;
	cxt_ptr->dst_addr_vir =  in_parm_ptr->dst_addr_vir;
	cxt_ptr->dst_endian  =  in_parm_ptr->dst_endian;

	cxt_ptr->dst_fmt = in_parm_ptr->dst_fmt;

	cxt_ptr->size = in_parm_ptr->size;

	cxt_ptr->slice_height = in_parm_ptr->slice_height;
	cxt_ptr->slice_mod = in_parm_ptr->slice_mod;
	
	return ret;
	
}

int jpeg_enc_start(JPEG_ENC_INPUT_PARAM *in_parm_ptr, JPEG_ENC_OUTPUT_PARAM *out_parm_ptr)
{
	int ret = JPEG_SUCCESS;
	JPEG_ENC_T *enc_cxt_ptr = 0;
	JPEG_HANDLE_T * handle_ptr = 0;
	CMR_MSG_INIT(message);

	message.msg_type = JPEG_EVT_ENC_START;
	
	if(JPEG_SUCCESS != _check_enc_start_param(in_parm_ptr, out_parm_ptr)){
		return JPEG_PARAM_ERR;
	}

	enc_cxt_ptr = (JPEG_ENC_T *)malloc(sizeof(JPEG_ENC_T));
	
	CMR_JPEG_LOGE("jpeg_enc_start: jpeg enc: 0x%x\n", (uint32_t)enc_cxt_ptr);
	
	if(NULL == enc_cxt_ptr  ) {
		return JPEG_NO_MEM;
	}
	memset(enc_cxt_ptr, 0, sizeof(JPEG_ENC_T));

	handle_ptr = (JPEG_HANDLE_T *)malloc(sizeof(JPEG_HANDLE_T));

	if(NULL == handle_ptr ) {
		free(enc_cxt_ptr);
		return JPEG_NO_MEM;
	}

	if(JPEG_SUCCESS != _get_enc_start_param(enc_cxt_ptr,in_parm_ptr, out_parm_ptr )) {
		return JPEG_PARAM_ERR;
	}

	message.msg_type = JPEG_EVT_ENC_START;
	message.data = enc_cxt_ptr;
	ret = cmr_msg_post(jcontext.msg_queue_handle, &message);

	if(CMR_MSG_SUCCESS != ret) {
		free(handle_ptr);
		free(enc_cxt_ptr);
		return JPEG_ERROR;
	}
	jcontext.active_handle = (uint32_t)(handle_ptr);
	handle_ptr->handle = (uint32_t)enc_cxt_ptr;
	handle_ptr->type = 0;
	out_parm_ptr->handle = (uint32_t)(handle_ptr);
	CMR_LOGV("handle 0x%x.",(uint32_t)handle_ptr);
	return JPEG_SUCCESS;
}

int jpeg_enc_next(JPEG_ENC_NXT_PARAM *param_ptr)
{	
	JPEG_ENC_T *enc_cxt_ptr = 0;
	uint32_t  ret = CMR_MSG_SUCCESS;
	JPEG_HANDLE_T *handle_ptr = NULL;
	JPEG_ENC_NXT_PARAM *data_ptr = NULL;

	uint32_t handle = 0;

	CMR_MSG_INIT(message);

	if(0 == param_ptr){
		return JPEG_PARAM_ERR;
	}
	CMR_LOGV("start,handle 0x%x.",param_ptr->handle);
	handle =  param_ptr->handle;
	handle_ptr = (JPEG_HANDLE_T*)handle;
	if(jcontext.active_handle !=  handle){
		CMR_JPEG_LOGE("jpeg_enc_next error,encode already finish.");
		return JPEG_PARAM_ERR;
	}
	
	enc_cxt_ptr = (JPEG_ENC_T *)handle_ptr->handle;

	if(1 == enc_cxt_ptr->is_finish) {
		CMR_JPEG_LOGE("encode finish.");
		return JPEG_ERROR;
	}

	if(JPEG_YUV_SLICE_MUTI_BUF == enc_cxt_ptr->slice_mod){
		
		if(0 == param_ptr->src_addr_phy.addr_y|| 0 == param_ptr->src_addr_vir.addr_y ||
		    0 == param_ptr->src_addr_phy.addr_u || 0 == param_ptr->src_addr_vir.addr_u
		  ){
			
			return JPEG_PARAM_ERR;
		}
	}

	data_ptr = (JPEG_ENC_NXT_PARAM *)malloc(sizeof(JPEG_ENC_NXT_PARAM ));

	if(0 == data_ptr){
		return JPEG_NO_MEM;
	}

	memcpy(data_ptr, param_ptr, sizeof(JPEG_ENC_NXT_PARAM));


	message.msg_type = JPEG_EVT_ENC_NEXT;
	message.data = data_ptr;
	message.alloc_flag = 1;
	
	ret = cmr_msg_post(jcontext.msg_queue_handle, &message);

	
	if(CMR_MSG_SUCCESS != ret){
		free(data_ptr);
		return JPEG_ERROR;
	}

	return JPEG_SUCCESS;
}


int jpeg_dec_start(JPEG_DEC_IN_PARAM  *in_parm_ptr, JPEG_DEC_OUT_PARAM *out_parm_ptr)
{
	int ret = JPEG_SUCCESS;
	JPEG_DEC_T *dec_cxt_ptr = 0;
	JPEG_HANDLE_T * handle_ptr = 0;
	CMR_MSG_INIT(message);

	message.msg_type = JPEG_EVT_DEC_START;
	
	if(JPEG_SUCCESS != _check_dec_start_param(in_parm_ptr, out_parm_ptr)){
		return JPEG_PARAM_ERR;
	}

	dec_cxt_ptr = (JPEG_DEC_T *)malloc(sizeof(JPEG_DEC_T));

	if(NULL == dec_cxt_ptr ) {
		return JPEG_NO_MEM;
	}
	memset(dec_cxt_ptr, 0, sizeof(JPEG_DEC_T));
	
	handle_ptr = (JPEG_HANDLE_T *)malloc(sizeof(JPEG_HANDLE_T));

	if(NULL == handle_ptr ){
		free(dec_cxt_ptr);
		return JPEG_NO_MEM;
	}

	if(JPEG_SUCCESS != _get_dec_start_param(dec_cxt_ptr,in_parm_ptr, out_parm_ptr )) {
		return JPEG_PARAM_ERR;
	}

	message.msg_type = JPEG_EVT_DEC_START;
	message.data = dec_cxt_ptr;
	ret = cmr_msg_post(jcontext.msg_queue_handle, &message);

	if(CMR_MSG_SUCCESS != ret) {
		free(handle_ptr);
		free(dec_cxt_ptr);
		return JPEG_ERROR;
	}

	handle_ptr->handle = (uint32_t)dec_cxt_ptr;
	handle_ptr->type = 1;/*decode*/
	out_parm_ptr->handle = (uint32_t)handle_ptr;
	
	return JPEG_SUCCESS;
}

//useless function, some mode can not be support.
//slice mode can not be supp
int jpeg_dec_next(JPEG_DEC_NXT_PARAM *next_param_ptr)
{
	return 0;
}

int jpeg_stop(uint32_t handle)
{
	int ret = JPEG_SUCCESS;
	CMR_MSG_INIT(message);

	if(0 == handle){
		return JPEG_PARAM_ERR;
	}

	message.msg_type = JPEG_EVT_STOP;

	CMR_JPEG_LOGE("jpeg_stop start\n");

	CMR_JPEG_LOGE("jpeg_stop: handle: 0x%x\n", (uint32_t)handle);

	message.data = (void*)handle;
	ret = cmr_msg_post(jcontext.msg_queue_handle, &message);

	if(CMR_MSG_SUCCESS != ret) {
		return JPEG_ERROR;
	}

	sem_wait(&jcontext.stop_sem);
	CMR_JPEG_LOGE("jpeg_stop end\n");
	return JPEG_SUCCESS;
}

int jpeg_deinit(void)
{
	int ret = JPEG_SUCCESS;
	CMR_MSG_INIT(message);

	message.msg_type = JPEG_EVT_KILL;

	CMR_JPEG_LOGE("jpeg_deinit start\n");

	ret = cmr_msg_post(jcontext.msg_queue_handle, &message);

	if(CMR_MSG_SUCCESS != ret) {
		return JPEG_ERROR;
	}
	_kill_thread();
	sem_destroy(&jcontext.stop_sem);
	return ret;
}

void jpeg_evt_reg(cmr_evt_cb  adp_event_cb)
{
	jcontext.event_cb = adp_event_cb;
	return ;
}

