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
#include "exif_writer.h"
#include "cmr_msg.h"
#include "jpeg_codec.h"
#include "cmr_common.h"
#include "jpegdec_api.h"
#include "jpegenc_api.h"


#define JPEG_MSG_QUEUE_SIZE		  40
#define JPEG_EXIT_THREAD_FLAG	  1
#define JPEG_SLICE_HEIGHT         128
#define JPEG_BUF_RES_SIZE         256
#define JPEG_DECODE_FW_BUF_SIZE   (20*1024)
#define JPEG_EVT_ENC_START	      (1 << 16)
#define JPEG_EVT_ENC_NEXT	      (1 << 17)
#define JPEG_EVT_DEC_START	      (1 << 18)
#define JPEG_EVT_DEC_NEXT	      (1 << 19)
#define JPEG_EVT_STOP		      (1 << 20)
#define JPEG_EVT_KILL		      (1 << 21)
#define JPEG_EVT_ENC_EXIF	      (1 << 22)
#define JPEG_EVT_ENC_THUMB	      (1 << 23)

#define JPEG_EVT_MASK_BITS	(uint32_t)(JPEG_EVT_ENC_START | JPEG_EVT_ENC_NEXT | \
	JPEG_EVT_DEC_START | JPEG_EVT_DEC_NEXT | JPEG_EVT_STOP | JPEG_EVT_KILL | \
	JPEG_EVT_ENC_EXIF | JPEG_EVT_ENC_THUMB)

typedef struct
{
	pthread_t	v4l2_thread;
	sem_t       stop_sem;
	sem_t       sync_sem;
	uint32_t	msg_queue_handle;
	uint32_t	active_handle;
	uint32_t    is_exit_thread;
	cmr_evt_cb  event_cb;
	uint32_t    is_stop;
	void        *fw_decode_buf;
	uint32_t    fw_decode_buf_size;
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
	uint32_t    set_slice_height;
	uint32_t    slice_mod;/*PEG_YUV_SLICE_MODE*/

	struct img_addr     dst_addr_phy;
	struct img_addr     dst_addr_vir;
	struct img_data_end	dst_endian;

	uint32_t    dst_fmt;

	uint32_t   fw_decode_buf_size;
	void * 	fw_decode_buf;

	//use by codec
	uint32_t    temp_buf_phy;
	uint32_t    temp_buf_vir;
	uint32_t	temp_buf_size;
	uint32_t    cur_line_num;
	uint32_t    handle_line_num;
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
	uint32_t    is_thumbnail;
}JPEG_ENC_T;

static struct jpeg_wexif_cb_param s_exif_output;

static JPEG_CONTEXT_T  jcontext;
static JPEG_ENC_CB_PARAM_T s_thumbnail;
static void* _thread_proc(void* data);
static int _kill_thread(void);
static  int  _dec_next(uint32_t handle, struct jpeg_dec_next_param *param_ptr);

static void savedata(uint32_t buf_addr, uint32_t size)
{
	FILE *fp = NULL;

	CMR_LOGV("jpeg: savedata");
	fp = fopen("/data/out.raw", "wb");
	if(0 != fp)
	{
		fwrite((void*)buf_addr, 1, size, fp);
		fclose(fp);
	}else{

		CMR_LOGE("can not create savedata");
	}

}

static void save_inputdata(uint32_t y_buf_addr,
							uint32_t uv_buf_addr,
							uint32_t size)
{
	FILE *fp = NULL;

	CMR_LOGV("jpeg: save input data,size=%d.",size);
	fp = fopen("/data/in_y.raw", "wb");
	if(0 != fp) {
		fwrite((void*)y_buf_addr, 1, size, fp);
		fclose(fp);
	}else{
		CMR_LOGE("can not create savedata");
	}
	fp = fopen("/data/in_uv.raw", "wb");
	if(0 != fp) {
		fwrite((void*)uv_buf_addr, 1, size/2, fp);
		fclose(fp);
	}else{
		CMR_LOGE("can not create savedata");
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

		CMR_LOGE("JPEG, unknow format");

	break;

	}

	return jfmt;
}


static uint32_t _quality_covert(uint32_t quality)
{
	uint32_t jq = JPEGENC_QUALITY_HIGH;
	if (quality <= 60) {
		jq = JPEGENC_QUALITY_LOW;
	} else if (quality <= 70) {
		jq = JPEGENC_QUALITY_MIDDLE_LOW;
	} else if (quality <= 80) {
		jq = JPEGENC_QUALITY_MIDDLE;
	} else if (quality <= 90) {
		jq = JPEGENC_QUALITY_MIDDLE_HIGH;
	} else {
		jq = JPEGENC_QUALITY_HIGH;
	}
	return jq;
}

void _prc_enc_cbparam(uint32_t handle, JPEG_ENC_CB_PARAM_T *parm_ptr)
{
	JPEG_ENC_T *enc_cxt_ptr = NULL;
/*	FILE *fp = NULL;*/

	enc_cxt_ptr = (JPEG_ENC_T * )handle;
	parm_ptr->stream_buf_phy = enc_cxt_ptr->stream_buf_phy;
	parm_ptr->stream_buf_vir = enc_cxt_ptr->stream_buf_vir;
	parm_ptr->stream_size = enc_cxt_ptr->stream_real_size;

	parm_ptr->slice_height = enc_cxt_ptr->slice_height;
	parm_ptr->total_height = enc_cxt_ptr->cur_line_num;
	CMR_LOGV("slice_height %d,total_height %d,stream_size 0x%x.",
		parm_ptr->slice_height,parm_ptr->total_height,parm_ptr->stream_size);
/*	fp = fopen("/data/1.aw", "wb");
	if(0 != fp) {
		fwrite((void*)parm_ptr->stream_buf_vir, 1, parm_ptr->stream_size, fp);
		CMR_LOGV("save jpg.");
		fclose(fp);
	}*/
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
	int ret = JPEG_CODEC_SUCCESS;
	uint32_t jpeg_enc_buf_phys_addr;
	uint32_t *jpeg_enc_buf_virt_addr;
	uint32_t jpeg_enc_buf_len;
	uint32_t i = 0;
	uint32_t jpeg_ret = 0;
	JPEG_ENC_T *enc_cxt_ptr = NULL;
	JPEGENC_SLICE_OUT_T slice_out;
	JPEGENC_SLICE_NEXT_T next_slice_parm;
	JPEGENC_PARAMS_T *jenc_parm_ptr = (JPEGENC_PARAMS_T *)malloc(sizeof(JPEGENC_PARAMS_T));

	CMR_LOGV("jpeg: _encoder_start: S");
	memset((void*)&slice_out,0,sizeof(JPEGENC_SLICE_OUT_T));
	memset((void*)&next_slice_parm,0,sizeof(JPEGENC_SLICE_NEXT_T));
	if(NULL == jenc_parm_ptr){
		return JPEG_CODEC_NO_MEM;
	}

	enc_cxt_ptr = (JPEG_ENC_T * )handle;
	if(enc_cxt_ptr->slice_height == enc_cxt_ptr->size.height ) {
		jenc_parm_ptr->set_slice_height = JPEG_SLICE_HEIGHT;//enc_cxt_ptr->slice_height;
	} else{
		jenc_parm_ptr->set_slice_height = enc_cxt_ptr->slice_height;
	}
	if(1 == enc_cxt_ptr->is_thumbnail) {
		jenc_parm_ptr->set_slice_height = enc_cxt_ptr->size.height;
		CMR_LOGV("thumbnail enc: set_slice_height = %d.",jenc_parm_ptr->set_slice_height);
	}

	jenc_parm_ptr->format = JPEGENC_YUV_420;//_format_covert(enc_cxt_ptr->src_fmt);
	jenc_parm_ptr->quality = _quality_covert(enc_cxt_ptr->quality_level);
	jenc_parm_ptr->width = enc_cxt_ptr->size.width;
	jenc_parm_ptr->height = enc_cxt_ptr->size.height;
	jenc_parm_ptr->yuv_virt_buf =  (void *)enc_cxt_ptr->src_addr_vir.addr_y;
	jenc_parm_ptr->yuv_phy_buf = enc_cxt_ptr->src_addr_phy.addr_y;
	jenc_parm_ptr->yuv_u_virt_buf = (void *)enc_cxt_ptr->src_addr_vir.addr_u;
	jenc_parm_ptr->yuv_u_phy_buf = enc_cxt_ptr->src_addr_phy.addr_u;
	jenc_parm_ptr->yuv_v_virt_buf = (void*)0;//(void *)enc_cxt_ptr->src_addr_vir.addr_v;
	jenc_parm_ptr->yuv_v_phy_buf = 0;//enc_cxt_ptr->src_addr_phy.addr_v;


	CMR_LOGV("jpeg enc yuv phy addr,0x%x 0x%x 0x%x,slice height %d.",
		jenc_parm_ptr->yuv_phy_buf,
		jenc_parm_ptr->yuv_u_phy_buf,
		jenc_parm_ptr->yuv_v_phy_buf,
		enc_cxt_ptr->slice_height);

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
	CMR_LOGV("encoder: jpegenc_params[%d]: virt: %x, phys: %x,size %d.",
		i,(uint32_t)jenc_parm_ptr->stream_virt_buf[i],jenc_parm_ptr->stream_phy_buf[i],jpeg_enc_buf_len);

	jenc_parm_ptr->stream_buf_len = jpeg_enc_buf_len;
	jenc_parm_ptr->stream_size = 0;

	//start jpeg enc for both slice and frame
	if(0 != JPEGENC_Slice_Start(jenc_parm_ptr, &slice_out)){
		ret =  JPEG_CODEC_ERROR;
		goto enc_start_end;
	}
	enc_cxt_ptr->cur_line_num = jenc_parm_ptr->set_slice_height;
	//if frame, and still use slice mode for sc8810 to keep the same  interface to top layer
	if(enc_cxt_ptr->slice_height == enc_cxt_ptr->size.height ){
		uint32_t cur_slice_height = jenc_parm_ptr->set_slice_height;
		int slice_num =  (int)(enc_cxt_ptr->size.height/cur_slice_height);
		uint32_t cur_ver_pos = enc_cxt_ptr->cur_line_num;
		uint32_t buf_id = 1;
		uint32_t cur_y_buf_adr = 0;
		uint32_t cur_u_buf_adr = 0;

		CMR_LOGV("jpeg: slice mode for frame");

		if(0 !=  enc_cxt_ptr->size.height%cur_slice_height){
			slice_num = slice_num+1;
		}

		CMR_LOGV("jpeg: slice num: %d", slice_num);
		slice_num--;/*start has process one slice*/
		if(0 != slice_num) {
		do {
			if(1 == jcontext.is_stop) {
				CMR_LOGI("force stop enode.");
				ret = JPEG_CODEC_ERROR;
				break;
			}
			next_slice_parm.slice_height = jenc_parm_ptr->set_slice_height;
			next_slice_parm.yuv_phy_buf = enc_cxt_ptr->src_addr_phy.addr_y+enc_cxt_ptr->cur_line_num* enc_cxt_ptr->size.width;
			next_slice_parm.yuv_u_phy_buf = enc_cxt_ptr->src_addr_phy.addr_u+enc_cxt_ptr->cur_line_num* enc_cxt_ptr->size.width/2;
			next_slice_parm.yuv_v_phy_buf = 0;
			jpeg_ret = JPEGENC_Slice_Next(&next_slice_parm, &slice_out);
			if(0 != jpeg_ret && 1 != jpeg_ret) {
				CMR_LOGE("JPEGENC_Slice_Next error,ret %d.",jpeg_ret);
				ret =  JPEG_CODEC_ERROR;
				break;
			}
			enc_cxt_ptr->cur_line_num += cur_slice_height;
			if(1 == slice_out.is_over) {
				enc_cxt_ptr->is_finish = 1;
				enc_cxt_ptr->stream_real_size = slice_out.stream_size;
				enc_cxt_ptr->cur_line_num = enc_cxt_ptr->size.height;
				CMR_LOGV("jpeg: slice num:slice size: %d", slice_out.stream_size);
				break;
			}
			slice_num--;
		}while(0 <  slice_num);
		} else {
			enc_cxt_ptr->is_finish = 1;
			enc_cxt_ptr->stream_real_size = slice_out.stream_size;
			enc_cxt_ptr->cur_line_num = enc_cxt_ptr->size.height;
			CMR_LOGE("jpeg: slice num:slice size: %d", slice_out.stream_size);
		}
		CMR_LOGV("slice_num %d.",slice_num);
	}

	enc_cxt_ptr->cur_id = 0;

	CMR_LOGV("jpeg:  buf addr: 0x%x,  size: %d",enc_cxt_ptr->stream_buf_vir, enc_cxt_ptr->stream_real_size);
/*	savedata(enc_cxt_ptr->stream_buf_vir, enc_cxt_ptr->stream_real_size);*/
enc_start_end:
	free(jenc_parm_ptr);
	CMR_LOGV("jpeg: _encoder_start E.");
	return ret;
}


//assume the slice height is the same, except the last one
static  int  _enc_next(uint32_t handle, struct jpeg_enc_next_param *param_ptr)
{
	int ret = JPEG_CODEC_SUCCESS;
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

	CMR_LOGV("jpeg: _enc_next: S");

	enc_cxt_ptr = (JPEG_ENC_T * )handle;
	if(((enc_cxt_ptr->cur_line_num + enc_cxt_ptr->slice_height)>param_ptr->ready_line_num)
		&& (param_ptr->ready_line_num != enc_cxt_ptr->size.height)){
		CMR_LOGI("ready line num don't enough for a slice.enc line num %d,read line num %d",
			enc_cxt_ptr->cur_line_num,param_ptr->ready_line_num);
		return JPEG_CODEC_ENC_WAIT_SRC;
	}
	memset((void*)&update_parm,0,sizeof(JPEGENC_SLICE_NEXT_T));
	memset((void*)&slice_out,0,sizeof(JPEGENC_SLICE_OUT_T));

	if(0 != param_ptr->src_addr_phy.addr_y) {
			update_parm.yuv_phy_buf = param_ptr->src_addr_phy.addr_y;
			update_parm.yuv_u_phy_buf = param_ptr->src_addr_phy.addr_u;
			update_parm.yuv_v_phy_buf = param_ptr->src_addr_phy.addr_v;
			update_parm.slice_height = param_ptr->slice_height;
	} else {
		cur_line_num = enc_cxt_ptr->cur_line_num;
		cur_slice_height =  enc_cxt_ptr->slice_height;

		if(enc_cxt_ptr->cur_line_num >=  enc_cxt_ptr->size.height) {
			return JPEG_CODEC_ERROR;
		}

		update_parm.slice_height = enc_cxt_ptr->slice_height;
		update_parm.yuv_phy_buf = enc_cxt_ptr->src_addr_phy.addr_y + cur_line_num*enc_cxt_ptr->size.width;
		update_parm.yuv_u_phy_buf = enc_cxt_ptr->src_addr_phy.addr_u + cur_line_num*enc_cxt_ptr->size.width/2;
		update_parm.yuv_v_phy_buf = enc_cxt_ptr->src_addr_phy.addr_v;
	}

	CMR_LOGI("cur_line_num %d, addr y 0x%x,addr u 0x%x.",
		cur_line_num,
		update_parm.yuv_phy_buf,
		update_parm.yuv_u_phy_buf);

 	//encode the jpeg picture by HW.
	if(0 != JPEGENC_Slice_Next(&update_parm, &slice_out)) {
		ret =  JPEG_CODEC_ERROR;
	}
    enc_cxt_ptr->cur_line_num += enc_cxt_ptr->slice_height;
	if(1 == slice_out.is_over){
		enc_cxt_ptr->is_finish = 1;
		enc_cxt_ptr->stream_real_size = slice_out.stream_size;
		enc_cxt_ptr->cur_line_num = enc_cxt_ptr->size.height;
	}

	CMR_LOGV("jpeg: _encoder_start E.");
	return ret;
}

void _dec_callback(uint32_t buf_id, uint32_t stream_size, uint32_t is_last_slice)
{
	JPEG_HANDLE_T           *handle_ptr  = (JPEG_HANDLE_T*)jcontext.active_handle;
	JPEG_DEC_T              *dec_cxt_ptr = (JPEG_DEC_T*)handle_ptr->handle;
	uint32_t                cpy_height   = 0;
	uint32_t                src, dst, i;
	struct img_frm			img_frm;
	JPEG_DEC_CB_PARAM_T     param;

	param.src_img = &img_frm;
	if (dec_cxt_ptr->cur_line_num + dec_cxt_ptr->set_slice_height > dec_cxt_ptr->size.height) {
		cpy_height = dec_cxt_ptr->size.height - dec_cxt_ptr->cur_line_num;
		dec_cxt_ptr->is_finish = 1;
	} else {
		cpy_height = dec_cxt_ptr->set_slice_height;
	}
	param.slice_height = cpy_height;
	CMR_LOGV("jpeg handle 0x%x, dec handle 0x%x, line number %d, cpy_height %d",
		(uint32_t)handle_ptr, (uint32_t)dec_cxt_ptr, dec_cxt_ptr->cur_line_num, cpy_height);
/*
	dst = dec_cxt_ptr->dst_addr_vir.addr_y + (uint32_t)(dec_cxt_ptr->size.width * dec_cxt_ptr->cur_line_num);
	param.src_img->addr_vir.addr_y = dst;
	param.src_img->addr_phy.addr_y = dec_cxt_ptr->dst_addr_phy.addr_y + (uint32_t)(dec_cxt_ptr->size.width * dec_cxt_ptr->cur_line_num);
	src = dec_cxt_ptr->temp_buf_vir;
	CMR_LOGV("copy y,src 0x%x, dst 0x%x", src, dst);
	memcpy((void*)dst, (void*)src, (uint32_t)(cpy_height * dec_cxt_ptr->size.width));
*/
	dst = dec_cxt_ptr->dst_addr_vir.addr_u;// + ((uint32_t)(dec_cxt_ptr->size.width * dec_cxt_ptr->cur_line_num) >> 1);
	param.src_img->addr_vir.addr_u = dst;
	param.src_img->addr_phy.addr_u = dec_cxt_ptr->dst_addr_phy.addr_u;// + ((uint32_t)(dec_cxt_ptr->size.width * dec_cxt_ptr->cur_line_num) >> 1);
	src = dec_cxt_ptr->temp_buf_vir;// + (uint32_t)(dec_cxt_ptr->size.width * dec_cxt_ptr->slice_height);
	CMR_LOGV("copy uv,src 0x%x, dst 0x%x", src, dst);
	for (i = 0; i < (cpy_height >> 1); i++) {
		memcpy((void*)dst,(void*)src,dec_cxt_ptr->size.width);
		dst += dec_cxt_ptr->size.width;
		src += (dec_cxt_ptr->size.width << 1);
	}

	dec_cxt_ptr->cur_line_num += cpy_height;
	dec_cxt_ptr->handle_line_num = cpy_height;
	param.total_height = dec_cxt_ptr->cur_line_num;
	param.src_img->data_end = dec_cxt_ptr->dst_endian;
	param.src_img->fmt = dec_cxt_ptr->dst_fmt;
	param.src_img->size = dec_cxt_ptr->size;
	if((dec_cxt_ptr->slice_height != dec_cxt_ptr->size.height)
		||(param.total_height == dec_cxt_ptr->size.height)) {
		jcontext.event_cb(CMR_JPEG_DEC_DONE, &param);
	}

	return;
}

static int _dec_start(uint32_t handle)
{
	int ret = JPEG_CODEC_SUCCESS;
	JPEG_DEC_T *dec_cxt_ptr = (JPEG_DEC_T *)handle;
	JPEGDEC_PARAMS_T  jpegdec_params;
	JPEGDEC_SLICE_OUT_T slice_out;
	struct jpeg_dec_next_param next_param;

	CMR_LOGI("dec slice height %d.",dec_cxt_ptr->slice_height);

	dec_cxt_ptr->cur_line_num = 0;
	dec_cxt_ptr->is_finish = 0;
	dec_cxt_ptr->fw_decode_buf_size = jcontext.fw_decode_buf_size;
	dec_cxt_ptr->fw_decode_buf = jcontext.fw_decode_buf;

	jpegdec_params.format = JPEGDEC_YUV_422;
	jpegdec_params.width  =   dec_cxt_ptr->size.width;
	jpegdec_params.height = dec_cxt_ptr->size.height;
	jpegdec_params.stream_size = dec_cxt_ptr->stream_buf_size;
	jpegdec_params.src_buf = (void *)dec_cxt_ptr->stream_buf_vir;
	jpegdec_params.src_phy_buf = dec_cxt_ptr->stream_buf_phy;
	jpegdec_params.target_buf_Y = (void *)dec_cxt_ptr->dst_addr_vir.addr_y;
	jpegdec_params.target_phy_buf_Y = dec_cxt_ptr->dst_addr_phy.addr_y;
	jpegdec_params.target_buf_UV = (void *)dec_cxt_ptr->temp_buf_vir;
	jpegdec_params.target_phy_buf_UV = dec_cxt_ptr->temp_buf_phy;

	jpegdec_params.fw_decode_buf = dec_cxt_ptr->fw_decode_buf;
	jpegdec_params.fw_decode_buf_size  = dec_cxt_ptr->fw_decode_buf_size;

	jpegdec_params.stream_virt_buf[0] = jpegdec_params.src_buf;
	jpegdec_params.stream_phy_buf[0]  = jpegdec_params.src_phy_buf;

	jpegdec_params.stream_virt_buf[1] =  0;
	jpegdec_params.stream_buf_len = jpegdec_params.stream_size;
	jpegdec_params.yuv_virt_buf   = jpegdec_params.target_buf_Y;
	jpegdec_params.yuv_phy_buf    = jpegdec_params.target_phy_buf_Y;
	jpegdec_params.set_slice_height = dec_cxt_ptr->slice_height;
	dec_cxt_ptr->set_slice_height = dec_cxt_ptr->slice_height;
	if(dec_cxt_ptr->slice_height == dec_cxt_ptr->size.height) {
		jpegdec_params.set_slice_height = JPEG_SLICE_HEIGHT;
		dec_cxt_ptr->set_slice_height = JPEG_SLICE_HEIGHT;
	}

	memset(&slice_out,0,sizeof(JPEGDEC_SLICE_OUT_T));

	if(0 != JPEGDEC_Slice_Start(&jpegdec_params,  &slice_out)) {
		ret = JPEG_CODEC_ERROR;
	}

	if(dec_cxt_ptr->slice_height == dec_cxt_ptr->size.height) {
		while(1 != dec_cxt_ptr->is_finish) {
			_dec_callback(0,0,0);
			next_param.dst_addr_phy.addr_y = 0;
			next_param.slice_height = JPEG_SLICE_HEIGHT;
			if(JPEG_CODEC_SUCCESS != _dec_next(handle,&next_param)) {
				CMR_LOGE("dec next error!");
				return JPEG_CODEC_ERROR;
			}
		}
	}

	CMR_LOGI("dec start end.");

	return ret;

}

static  int  _dec_next(uint32_t handle, struct jpeg_dec_next_param *param_ptr)
{
	int ret = JPEG_CODEC_SUCCESS;
	JPEGDEC_SLICE_OUT_T slice_out;
	JPEGDEC_SLICE_NEXT_T slice_param;
	JPEG_DEC_T *dec_cxt_ptr = NULL;
	uint32_t dec_line_num = 0;

	CMR_LOGI("handle :0x%x.",handle);

	memset(&slice_out,0,sizeof(JPEGDEC_SLICE_OUT_T));
	if(0 == param_ptr->dst_addr_phy.addr_y) {
		CMR_LOGI("one buffer.");
		dec_cxt_ptr = (JPEG_DEC_T * )handle;
		slice_param.slice_height = dec_cxt_ptr->slice_height;
		slice_param.yuv_phy_buf = dec_cxt_ptr->dst_addr_phy.addr_y + dec_cxt_ptr->cur_line_num*dec_cxt_ptr->size.width;
		dec_cxt_ptr->dst_addr_phy.addr_u += dec_cxt_ptr->handle_line_num*dec_cxt_ptr->size.width>>1;
		dec_cxt_ptr->dst_addr_vir.addr_u += dec_cxt_ptr->handle_line_num*dec_cxt_ptr->size.width>>1;
	} else {
		slice_param.slice_height = param_ptr->slice_height;
		slice_param.yuv_phy_buf = param_ptr->dst_addr_phy.addr_y;
		dec_cxt_ptr->dst_addr_phy.addr_u = param_ptr->dst_addr_phy.addr_u;
		dec_cxt_ptr->dst_addr_vir.addr_u = param_ptr->dst_addr_vir.addr_u;
		dec_cxt_ptr->dst_addr_phy.addr_v = param_ptr->dst_addr_phy.addr_v;
		dec_cxt_ptr->dst_addr_vir.addr_v = param_ptr->dst_addr_vir.addr_v;
	}
	slice_param.yuv_u_phy_buf = dec_cxt_ptr->temp_buf_phy;
    CMR_LOGI("update addr:0x%x,0x%x.",slice_param.yuv_phy_buf,slice_param.yuv_u_phy_buf);
	if(JPEG_CODEC_SUCCESS != JPEGDEC_Slice_Next(&slice_param,&slice_out)) {
		ret = JPEG_CODEC_ERROR;
		CMR_LOGE("dec next error!");
	}

	if(1 == slice_out.is_over){
		dec_cxt_ptr->is_finish = 1;
		CMR_LOGI("dec finish.");
	}

	CMR_LOGI("dec next done,dec height:%d.",dec_cxt_ptr->cur_line_num);
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

	CMR_LOGV("Call write function to kill v4l2 manage thread");

	CMR_LOGV("write OK!");
	ret = pthread_join(jcontext.v4l2_thread, &dummy);

	CMR_LOGV("kill jpeg thread result %d.",ret);
	return ret;
}

static int _jpeg_stop(uint32_t handle)
{
	JPEG_HANDLE_T *handle_ptr = (JPEG_HANDLE_T*)handle;

	CMR_LOGV("_jpeg_stop start\n");
	if(0 == handle_ptr){
		return JPEG_CODEC_PARAM_ERR;
	}

	if(0 == handle_ptr->type){
		JPEG_ENC_T *enc_cxt_ptr = (JPEG_ENC_T*)handle_ptr->handle;
		if(NULL != enc_cxt_ptr) {
			free(enc_cxt_ptr);
			handle_ptr->handle = 0;
		}
		free((void*)handle);
		jcontext.active_handle = 0;
	}else if(1 == handle_ptr->type){
		JPEG_DEC_T *dec_cxt_ptr = (JPEG_DEC_T*)handle_ptr->handle;
		if(NULL != dec_cxt_ptr) {
			free(dec_cxt_ptr);
			handle_ptr->handle = 0;
		}
		free((void*)handle);
		jcontext.active_handle = 0;
	} else {
		return JPEG_CODEC_PARAM_ERR;
	}

	return JPEG_CODEC_SUCCESS;
}

static int _jpeg_enc_wexif(struct jpeg_enc_exif_param *param_ptr,struct jpeg_wexif_cb_param *out_ptr)
{
	int ret = JPEG_CODEC_SUCCESS;
	JINF_WEXIF_IN_PARAM_T input_param;
	JINF_WEXIF_OUT_PARAM_T output_param;

	input_param.exif_info_ptr = param_ptr->exif_ptr;
	input_param.src_jpeg_buf_ptr = (uint8_t*)param_ptr->src_jpeg_addr_virt;
	input_param.src_jpeg_size    = param_ptr->src_jpeg_size;
	input_param.thumbnail_buf_ptr = (uint8_t*)param_ptr->thumbnail_addr_virt;
	input_param.thumbnail_buf_size = param_ptr->thumbnail_size;
	input_param.target_buf_ptr = (uint8_t*)param_ptr->target_addr_virt;
	input_param.target_buf_size = param_ptr->target_size;
	input_param.temp_buf_size = param_ptr->thumbnail_size+21*1024;
	input_param.temp_buf_ptr = (uint8_t*)malloc(input_param.temp_buf_size);
	if (PNULL == input_param.temp_buf_ptr) {
		CMR_LOGE("malloc temp buf for wexif fail.");
		return JPEG_CODEC_NO_MEM;
	}
	input_param.wrtie_file_func = NULL;

	CMR_LOGI("src jpeg addr 0x%x, size %d. thumbnail addr 0x%x, size %d. target addr 0x%x,size %d.",
				(uint32_t)input_param.src_jpeg_buf_ptr,input_param.src_jpeg_size,
				(uint32_t)input_param.thumbnail_buf_ptr,input_param.thumbnail_buf_size,
				(uint32_t)input_param.target_buf_ptr,input_param.target_buf_size);

	ret = IMGJPEG_WriteExif(&input_param,&output_param);

	out_ptr->output_buf_virt_addr = (uint32_t)output_param.output_buf_ptr;
	out_ptr->output_buf_size      = output_param.output_size;
	free(input_param.temp_buf_ptr);
	input_param.temp_buf_ptr = PNULL;
	CMR_LOGI("wexif output: addr 0x%x,size %d.",out_ptr->output_buf_virt_addr,out_ptr->output_buf_size);

	return ret;
}

static void* _thread_proc(void* data)
{
	int                      evt_id;
	int ret;
	uint32_t evt;
	uint32_t handle = 0;
	JPEG_HANDLE_T *handle_ptr = NULL;
	struct jpeg_enc_next_param *param_ptr = NULL;
	struct jpeg_dec_next_param *dec_param_ptr = NULL;
	JPEG_ENC_T *enc_cxt_ptr = NULL;
	struct jpeg_wexif_cb_param wexif_out_param;
	CMR_MSG_INIT(message);
	CMR_LOGV("JPEG Thread In \n");

	while(1) {

		ret = cmr_msg_get(jcontext.msg_queue_handle, &message);

		if (ret) {
			CMR_LOGE("jpeg: message queue destroied");
			break;
		}

		CMR_LOGE("jpeg: message.msg_type 0x%x", message.msg_type);
		evt = (uint32_t)(message.msg_type & JPEG_EVT_MASK_BITS);
		if((1 == jcontext.is_stop) &&(JPEG_EVT_STOP != evt)) {
			CMR_LOGI("discard message 0x%x.",evt);
			goto JPEG_SWITCH_END;
		}
		switch(evt){
		case  JPEG_EVT_ENC_START:
			handle = (uint32_t )message.data;
			ret = _enc_start(handle);

			if(JPEG_CODEC_SUCCESS == ret){
				JPEG_ENC_CB_PARAM_T param;
				memset((void*)&param,0,sizeof(JPEG_ENC_CB_PARAM_T));
				_prc_enc_cbparam(handle, &param);
				jcontext.event_cb(CMR_JPEG_ENC_DONE, &param );
			} else{
				jcontext.event_cb(CMR_JPEG_ERR, NULL );
			}

			CMR_LOGE("jpeg:receive JPEG_EVT_ENC_START message");
			break;
		case  JPEG_EVT_ENC_NEXT:
			param_ptr = (struct jpeg_enc_next_param*)message.data;
			handle_ptr = (JPEG_HANDLE_T*)param_ptr->handle;
			handle = handle_ptr->handle;
			enc_cxt_ptr = (JPEG_ENC_T * )handle;

			do {
				ret = _enc_next( handle, param_ptr);
				if(JPEG_CODEC_SUCCESS != ret) {
					CMR_LOGE("enc next err %d.",ret);
					break;
				}
			}while((param_ptr->ready_line_num >= enc_cxt_ptr->size.height) && (enc_cxt_ptr->cur_line_num<enc_cxt_ptr->size.height));

			if(JPEG_CODEC_ENC_WAIT_SRC != ret) {
				if(JPEG_CODEC_SUCCESS == ret){
					JPEG_ENC_CB_PARAM_T param;
					memset((void*)&param,0,sizeof(JPEG_ENC_CB_PARAM_T));
					_prc_enc_cbparam(handle, &param);
					jcontext.event_cb(CMR_JPEG_ENC_DONE, &param );
				} else {
					jcontext.event_cb(CMR_JPEG_ERR, NULL );
				}
			}
			CMR_LOGI("receive enc next message.");
			break;
		case  JPEG_EVT_DEC_START:
			handle = (uint32_t )message.data;
			ret = _dec_start(handle);
			if(JPEG_CODEC_SUCCESS == ret){
				_dec_callback(0,0,0);
			} else{
				jcontext.event_cb(CMR_JPEG_ERR, NULL );
			}
			CMR_LOGI("jpeg:receive JPEG_EVT_DEC_START message");
			break;
		case  JPEG_EVT_DEC_NEXT:
			dec_param_ptr = (struct jpeg_dec_next_param*)message.data;
			handle_ptr = (JPEG_HANDLE_T*)dec_param_ptr->handle;
			handle = handle_ptr->handle;
			if(0 != message.data) {
				ret = _dec_next( handle, (struct jpeg_dec_next_param *)message.data);
			}

			if(JPEG_CODEC_SUCCESS == ret){
				_dec_callback(0,0,0);
			} else {
				jcontext.event_cb(CMR_JPEG_ERR, NULL );
			}
			CMR_LOGI("jpeg:receive dec next message.");
			break;
		case  JPEG_EVT_STOP:
			handle = (uint32_t )message.data;
			ret = _jpeg_stop(handle);
			sem_post(&jcontext.stop_sem);
			break;
		case JPEG_EVT_KILL:
			ret = _jpeg_stop(jcontext.active_handle);
			jcontext.is_exit_thread = JPEG_EXIT_THREAD_FLAG;
			break;

		case JPEG_EVT_ENC_EXIF:
			ret = _jpeg_enc_wexif((struct jpeg_enc_exif_param*)message.data,&wexif_out_param);
			if(JPEG_CODEC_SUCCESS == ret){
				s_exif_output = wexif_out_param;
			} else {
				s_exif_output.output_buf_size = 0;
			}
			sem_post(&jcontext.sync_sem);
			CMR_LOGI("write exif done,ret = %d.",ret);
			#if 0
			if(JPEG_CODEC_SUCCESS == ret){
				JPEG_WEXIF_CB_PARAM_T param;
				memset((void*)&param,0,sizeof(JPEG_WEXIF_CB_PARAM_T));
				jcontext.event_cb(CMR_JPEG_WEXIF_DONE, &param );
			} else{
				jcontext.event_cb(CMR_JPEG_ERR, NULL );
			}
			#endif
			break;
		case JPEG_EVT_ENC_THUMB:
			handle = (uint32_t )message.data;
			ret = _enc_start(handle);
			memset((void*)&s_thumbnail,0,sizeof(JPEG_ENC_CB_PARAM_T));
			if(JPEG_CODEC_SUCCESS == ret){
				_prc_enc_cbparam(handle, &s_thumbnail);
			}
			sem_post(&jcontext.sync_sem);
			CMR_LOGI("enc thumbnail enc done,ret = %d.",ret);
			break;
		default:
			CMR_LOGE("jpeg: not correct message");
			break;
		}
JPEG_SWITCH_END:
		if(1 == message.alloc_flag){
			free(message.data);
		}
		if(JPEG_EXIT_THREAD_FLAG == jcontext.is_exit_thread) {
			break;
		}
	}

	CMR_LOGV("JPEG Thrad Out");
	return NULL;
}


int jpeg_init(void)
{
	int ret = JPEG_CODEC_SUCCESS;

	memset(&jcontext,0,sizeof(JPEG_CONTEXT_T));
	sem_init(&jcontext.stop_sem, 0, 0);
	sem_init(&jcontext.sync_sem, 0, 0);
	ret = cmr_msg_queue_create(JPEG_MSG_QUEUE_SIZE, &jcontext.msg_queue_handle);
	ret = _create_thread();

	if(CMR_MSG_SUCCESS != ret) {
		ret = JPEG_CODEC_ERROR;
	}
	jcontext.fw_decode_buf_size = JPEG_DECODE_FW_BUF_SIZE;
	jcontext.fw_decode_buf = (void*)malloc(jcontext.fw_decode_buf_size);

	if (PNULL == jcontext.fw_decode_buf) {
		CMR_LOGE("malloc fail.");
		ret = JPEG_CODEC_NO_MEM;
	}
	return ret;
}

static int _check_enc_start_param(struct jpeg_enc_in_param *in_parm_ptr, struct jpeg_enc_out_param *out_parm_ptr)
{
	int ret = JPEG_CODEC_SUCCESS;

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


static int _get_enc_start_param(JPEG_ENC_T *cxt_ptr, struct jpeg_enc_in_param *in_parm_ptr, struct jpeg_enc_out_param *out_parm_ptr)
{
	int ret = JPEG_CODEC_SUCCESS;

	CMR_LOGV("jpeg_enc_start:all param");


	CMR_LOGI("stream_buf_phy: 0x%x",  in_parm_ptr->stream_buf_phy);
	CMR_LOGI("stream_buf_vir: 0x%x",  in_parm_ptr->stream_buf_vir);
	CMR_LOGI("stream_buf_size: 0x%x",  in_parm_ptr->stream_buf_size);
	CMR_LOGI("temp_buf_phy: 0x%x",  in_parm_ptr->temp_buf_phy);
	CMR_LOGI("temp_buf_vir: 0x%x",  in_parm_ptr->temp_buf_vir);
	CMR_LOGI("temp_buf_size: 0x%x",  in_parm_ptr->temp_buf_size);
	CMR_LOGI("src_buf_phy: 0x%x",  in_parm_ptr->temp_buf_phy);
	CMR_LOGI("src_buf_vir: 0x%x",  in_parm_ptr->temp_buf_vir);
	CMR_LOGI("img_size: w:%d, h:%d",  in_parm_ptr->size.width,  in_parm_ptr->size.height);
	CMR_LOGI("slice_height:%d",  in_parm_ptr->slice_height);

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

	return ret;

}

void jpeg_getcapability(JPEG_CAPABLIITY_T *param_ptr)
{
	param_ptr->max_height = 1024;
	param_ptr->max_ytouvoffset = 1024;
}

static int _check_dec_start_param(struct jpeg_dec_in_param *start_in_parm_ptr, struct jpeg_dec_out_param *start_out_parm_ptr)
{
	int ret = JPEG_CODEC_SUCCESS;
	return ret;
}

static int _get_dec_start_param(JPEG_DEC_T *cxt_ptr, struct jpeg_dec_in_param *in_parm_ptr, struct jpeg_dec_out_param *out_parm_ptr)
{
	int ret = JPEG_CODEC_SUCCESS;

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

	CMR_LOGV("stream phy 0x%x vir 0x%x, temp_buf phy 0x%x vir 0x%x",
		cxt_ptr->stream_buf_phy, cxt_ptr->stream_buf_vir,
		cxt_ptr->temp_buf_phy, cxt_ptr->temp_buf_vir);
	CMR_LOGV("dst phy  0x%x 0x%x, vir 0x%x 0x%x",
		cxt_ptr->dst_addr_phy.addr_y, cxt_ptr->dst_addr_phy.addr_u,
		cxt_ptr->dst_addr_vir.addr_y, cxt_ptr->dst_addr_vir.addr_u);

	return ret;

}

static int _check_wexif_param(struct jpeg_enc_exif_param *param_ptr)
{
	int ret = JPEG_CODEC_SUCCESS;

	if((NULL == (uint32_t*)param_ptr->target_addr_virt) || (NULL == (uint32_t*)param_ptr->src_jpeg_addr_virt)) {
		ret = JPEG_CODEC_PARAM_ERR;
	}

	CMR_LOGI("src jpeg addr 0x%x size %d.  thumbnail addr 0x%x,size %d. target addr 0x%x,size %d.",
			param_ptr->src_jpeg_addr_virt,param_ptr->src_jpeg_size,
			param_ptr->thumbnail_addr_virt,param_ptr->thumbnail_size,
			param_ptr->target_addr_virt,param_ptr->target_size);

	return ret;
}

int jpeg_enc_start(struct jpeg_enc_in_param *in_parm_ptr, struct jpeg_enc_out_param *out_parm_ptr)
{
	int ret = JPEG_CODEC_SUCCESS;
	JPEG_ENC_T *enc_cxt_ptr = 0;
	JPEG_HANDLE_T * handle_ptr = 0;
	CMR_MSG_INIT(message);

	message.msg_type = JPEG_EVT_ENC_START;

	if(JPEG_SUCCESS != _check_enc_start_param(in_parm_ptr, out_parm_ptr)){
		return JPEG_CODEC_PARAM_ERR;
	}

	enc_cxt_ptr = (JPEG_ENC_T *)malloc(sizeof(JPEG_ENC_T));

	CMR_LOGV("jpeg_enc_start: jpeg enc: 0x%x", (uint32_t)enc_cxt_ptr);

	if(NULL == enc_cxt_ptr  ) {
		return JPEG_CODEC_NO_MEM;
	}
	memset(enc_cxt_ptr, 0, sizeof(JPEG_ENC_T));

	handle_ptr = (JPEG_HANDLE_T *)malloc(sizeof(JPEG_HANDLE_T));

	if (NULL == handle_ptr ) {
		free(enc_cxt_ptr);
		return JPEG_CODEC_NO_MEM;
	}

	if (JPEG_SUCCESS != _get_enc_start_param(enc_cxt_ptr,in_parm_ptr, out_parm_ptr )) {
		free(enc_cxt_ptr);
		free(handle_ptr);
		return JPEG_CODEC_PARAM_ERR;
	}

	message.msg_type = JPEG_EVT_ENC_START;
	message.data = enc_cxt_ptr;
	message.alloc_flag = 0;
	ret = cmr_msg_post(jcontext.msg_queue_handle, &message);

	if(CMR_MSG_SUCCESS != ret) {
		free(handle_ptr);
		free(enc_cxt_ptr);
		return JPEG_CODEC_ERROR;
	}
	jcontext.active_handle = (uint32_t)(handle_ptr);
	handle_ptr->handle = (uint32_t)enc_cxt_ptr;
	handle_ptr->type = 0;
	out_parm_ptr->handle = (uint32_t)(handle_ptr);
	CMR_LOGV("handle 0x%x.",(uint32_t)handle_ptr);
	return JPEG_CODEC_SUCCESS;
}

int jpeg_enc_next(struct jpeg_enc_next_param *param_ptr)
{
	JPEG_ENC_T *enc_cxt_ptr = 0;
	uint32_t  ret = CMR_MSG_SUCCESS;
	JPEG_HANDLE_T *handle_ptr = NULL;
	struct jpeg_enc_next_param *data_ptr = NULL;

	uint32_t handle = 0;

	CMR_MSG_INIT(message);

	if(0 == param_ptr){
		return JPEG_CODEC_PARAM_ERR;
	}
	CMR_LOGV("start,handle 0x%x.",param_ptr->handle);
	handle =  param_ptr->handle;
	handle_ptr = (JPEG_HANDLE_T*)handle;
	if(jcontext.active_handle !=  handle){
		CMR_LOGE("jpeg_enc_next error,encode already finish.");
		return JPEG_CODEC_PARAM_ERR;
	}

	enc_cxt_ptr = (JPEG_ENC_T *)handle_ptr->handle;

	if(1 == enc_cxt_ptr->is_finish) {
		CMR_LOGE("encode finish.");
		return JPEG_CODEC_ERROR;
	}

	if(JPEG_YUV_SLICE_MUTI_BUF == enc_cxt_ptr->slice_mod){

		if(0 == param_ptr->src_addr_phy.addr_y|| 0 == param_ptr->src_addr_vir.addr_y ||
		    0 == param_ptr->src_addr_phy.addr_u || 0 == param_ptr->src_addr_vir.addr_u
		  ){

			return JPEG_CODEC_PARAM_ERR;
		}
	}

	data_ptr = (struct jpeg_enc_next_param *)malloc(sizeof(struct jpeg_enc_next_param ));

	if(0 == data_ptr){
		return JPEG_CODEC_NO_MEM;
	}

	memcpy(data_ptr, param_ptr, sizeof(struct jpeg_enc_next_param));


	message.msg_type = JPEG_EVT_ENC_NEXT;
	message.data = data_ptr;
	message.alloc_flag = 1;

	ret = cmr_msg_post(jcontext.msg_queue_handle, &message);


	if(CMR_MSG_SUCCESS != ret){
		free(data_ptr);
		return JPEG_CODEC_ERROR;
	}

	return JPEG_CODEC_SUCCESS;
}


int jpeg_dec_start(struct jpeg_dec_in_param  *in_parm_ptr, struct jpeg_dec_out_param *out_parm_ptr)
{
	int ret = JPEG_CODEC_SUCCESS;
	JPEG_DEC_T *dec_cxt_ptr = 0;
	JPEG_HANDLE_T * handle_ptr = 0;
	CMR_MSG_INIT(message);

	message.msg_type = JPEG_EVT_DEC_START;

	if(JPEG_CODEC_SUCCESS != _check_dec_start_param(in_parm_ptr, out_parm_ptr)){
		return JPEG_CODEC_PARAM_ERR;
	}

	dec_cxt_ptr = (JPEG_DEC_T *)malloc(sizeof(JPEG_DEC_T));

	if(NULL == dec_cxt_ptr ) {
		return JPEG_CODEC_NO_MEM;
	}
	memset(dec_cxt_ptr, 0, sizeof(JPEG_DEC_T));

	handle_ptr = (JPEG_HANDLE_T *)malloc(sizeof(JPEG_HANDLE_T));

	if(NULL == handle_ptr ){
		free(dec_cxt_ptr);
		return JPEG_CODEC_NO_MEM;
	}

	if(JPEG_CODEC_SUCCESS != _get_dec_start_param(dec_cxt_ptr,in_parm_ptr, out_parm_ptr )) {
		return JPEG_CODEC_PARAM_ERR;
	}

	message.msg_type = JPEG_EVT_DEC_START;
	message.data = dec_cxt_ptr;
	message.alloc_flag = 0;
	ret = cmr_msg_post(jcontext.msg_queue_handle, &message);

	if(CMR_MSG_SUCCESS != ret) {
		free(handle_ptr);
		free(dec_cxt_ptr);
		return JPEG_CODEC_ERROR;
	}
	jcontext.active_handle = (uint32_t)(handle_ptr);
	handle_ptr->handle = (uint32_t)dec_cxt_ptr;
	handle_ptr->type = 1;/*decode*/
	out_parm_ptr->handle = (uint32_t)handle_ptr;
	CMR_LOGV("jpeg handle 0x%x, dec handle 0x%x", (uint32_t)handle_ptr, (uint32_t)dec_cxt_ptr);
	return JPEG_CODEC_SUCCESS;
}

//useless function, some mode can not be support.
//slice mode can not be supp
int jpeg_dec_next(struct jpeg_dec_next_param *param_ptr)
{
	int ret = JPEG_CODEC_SUCCESS;
	JPEG_DEC_T *dec_cxt_ptr = 0;
	JPEG_HANDLE_T * handle_ptr = 0;
	uint32_t handle = 0;
	struct jpeg_dec_next_param *data_ptr = NULL;

	CMR_MSG_INIT(message);

	if(0 == param_ptr){
		return JPEG_CODEC_PARAM_ERR;
	}
	CMR_LOGV("start,handle 0x%x.",param_ptr->handle);
	handle =  param_ptr->handle;
	handle_ptr = (JPEG_HANDLE_T*)handle;
	if(jcontext.active_handle !=  handle){
		CMR_LOGE("error,decode already finish.");
		return JPEG_CODEC_PARAM_ERR;
	}

	dec_cxt_ptr = (JPEG_DEC_T*)handle_ptr->handle;
	if(1 == dec_cxt_ptr->is_finish) {
		CMR_LOGE("decode finish.");
		return JPEG_CODEC_ERROR;
	}

	if(JPEG_YUV_SLICE_MUTI_BUF == dec_cxt_ptr->slice_mod){
		if(0 == param_ptr->dst_addr_phy.addr_y|| 0 == param_ptr->dst_addr_vir.addr_y ||
		    0 == param_ptr->dst_addr_phy.addr_u || 0 == param_ptr->dst_addr_vir.addr_u
		  ){
			return JPEG_CODEC_PARAM_ERR;
		}
	}
	data_ptr = (struct jpeg_dec_next_param*)malloc(sizeof(struct jpeg_dec_next_param));
	if(0 == data_ptr){
		return JPEG_CODEC_NO_MEM;
	}
	memcpy(data_ptr, param_ptr, sizeof(struct jpeg_dec_next_param));

	message.msg_type = JPEG_EVT_DEC_NEXT;
	message.data = data_ptr;
	message.alloc_flag = 1;

	ret = cmr_msg_post(jcontext.msg_queue_handle, &message);

	if(CMR_MSG_SUCCESS != ret){
		free(data_ptr);
		return JPEG_CODEC_ERROR;
	}

	return ret;
}

int jpeg_stop(uint32_t handle)
{
	int ret = JPEG_CODEC_SUCCESS;
	CMR_MSG_INIT(message);

	if(0 == handle){
		return JPEG_CODEC_PARAM_ERR;
	}

	message.msg_type = JPEG_EVT_STOP;

	CMR_LOGV("jpeg_stop: handle: 0x%x", (uint32_t)handle);

	jcontext.is_stop = 1;
	message.data = (void*)handle;
	ret = cmr_msg_post(jcontext.msg_queue_handle, &message);

	if(CMR_MSG_SUCCESS != ret) {
		return JPEG_CODEC_ERROR;
	}

	sem_wait(&jcontext.stop_sem);
	jcontext.is_stop = 0;
	CMR_LOGV("jpeg_stop end\n");
	return JPEG_CODEC_SUCCESS;
}

int jpeg_deinit(void)
{
	int ret = JPEG_CODEC_SUCCESS;
	CMR_MSG_INIT(message);

	message.msg_type = JPEG_EVT_KILL;

	CMR_LOGV("jpeg_deinit start");

	ret = cmr_msg_post(jcontext.msg_queue_handle, &message);

	if(CMR_MSG_SUCCESS != ret) {
		return JPEG_CODEC_ERROR;
	}
	_kill_thread();
	if (PNULL != jcontext.fw_decode_buf) {
		free(jcontext.fw_decode_buf);
		jcontext.fw_decode_buf = PNULL;
	}
	sem_destroy(&jcontext.stop_sem);
	sem_destroy(&jcontext.sync_sem);
	cmr_msg_queue_destroy(jcontext.msg_queue_handle);
	return ret;
}

void jpeg_evt_reg(cmr_evt_cb  adp_event_cb)
{
	jcontext.event_cb = adp_event_cb;
	return ;
}

int jpeg_enc_add_eixf(struct jpeg_enc_exif_param *param_ptr,struct jpeg_wexif_cb_param *output_ptr)
{
	int ret = JPEG_CODEC_SUCCESS;
	struct jpeg_enc_exif_param *data = NULL;
	CMR_MSG_INIT(message);

	message.msg_type = JPEG_EVT_ENC_EXIF;

	CMR_LOGI("enc add exit stat.");

	ret = _check_wexif_param(param_ptr);

	if(JPEG_CODEC_SUCCESS != ret) {
		CMR_LOGE("input param error.");
		return JPEG_CODEC_PARAM_ERR;
	}

	data = (struct jpeg_enc_exif_param*)malloc(sizeof(struct jpeg_enc_exif_param));

	if(NULL == data) {
		return JPEG_CODEC_NO_MEM;
	}

	*data = *param_ptr;

	message.alloc_flag = 1;
	message.data = data;

	ret = cmr_msg_post(jcontext.msg_queue_handle, &message);

	if(CMR_MSG_SUCCESS != ret) {
		free(data);
		return JPEG_CODEC_ERROR;
	}

	sem_wait(&jcontext.sync_sem);
	if(0 != s_exif_output.output_buf_size) {
		*output_ptr = s_exif_output;
	} else {
		output_ptr->output_buf_size = 0;
		ret = JPEG_CODEC_ERROR;
		CMR_LOGE("write exif fail.");
	}

	CMR_LOGI("output addr 0x%x,size %d.",output_ptr->output_buf_virt_addr,output_ptr->output_buf_size);
	return ret;
}

int jpeg_enc_thumbnail(struct jpeg_enc_in_param *in_parm_ptr, uint32_t *stream_size_ptr)
{
	int ret = JPEG_CODEC_SUCCESS;
	JPEG_ENC_T *enc_cxt_ptr = 0;
	CMR_MSG_INIT(message);

	message.msg_type = JPEG_EVT_ENC_THUMB;

	if(JPEG_SUCCESS != _check_enc_start_param(in_parm_ptr, NULL)){
		return JPEG_CODEC_PARAM_ERR;
	}
/*	save_inputdata(in_parm_ptr->src_addr_vir.addr_y,in_parm_ptr->src_addr_vir.addr_u,320*240);*/

	enc_cxt_ptr = (JPEG_ENC_T *)malloc(sizeof(JPEG_ENC_T));

	CMR_LOGV("thumbnail enc: 0x%x", (uint32_t)enc_cxt_ptr);

	if(NULL == enc_cxt_ptr  ) {
		return JPEG_CODEC_NO_MEM;
	}
	memset(enc_cxt_ptr, 0, sizeof(JPEG_ENC_T));

	if(JPEG_SUCCESS != _get_enc_start_param(enc_cxt_ptr,in_parm_ptr, NULL )) {
		return JPEG_CODEC_PARAM_ERR;
	}

	message.msg_type = JPEG_EVT_ENC_THUMB;
	message.data = enc_cxt_ptr;
	message.alloc_flag = 1;
	enc_cxt_ptr->is_thumbnail = 1;
	ret = cmr_msg_post(jcontext.msg_queue_handle, &message);

	if(CMR_MSG_SUCCESS != ret) {
		free(enc_cxt_ptr);
		return JPEG_CODEC_ERROR;
	}

	sem_wait(&jcontext.sync_sem);
	*stream_size_ptr = 0;
	if(0 != s_thumbnail.stream_size) {
		*stream_size_ptr = s_thumbnail.stream_size;
	} else {
		ret = JPEG_CODEC_ERROR;
	}
/*	savedata(in_parm_ptr->stream_buf_vir,s_thumbnail.stream_size);*/
	CMR_LOGV("return %d.",ret);

	return ret;
}

