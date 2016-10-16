/*
 * =====================================================================================
 *
 *       Filename: isp_video.c
 *
 *       Description:  
 *
 *       Version:  1.0
 *       Created:  07/13/2012 04:49:12 PM
 *       Revision: none
 *       Compiler: gcc
 *
 *       Author:   Binary Yang <Binary.Yang@spreadtrum.com.cn>
 *       Company:  © Copyright 2010 Spreadtrum Communications Inc.
 *
 * =====================================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <stdint.h>
#include <sys/types.h>
#include <semaphore.h>
#include <pthread.h>
#define LOG_TAG "isp-video"
#include <cutils/log.h>
#include "isp_param_tune_com.h"

enum {
	CMD_START_PREVIEW = 1,
	CMD_STOP_PREVIEW,
	CMD_READ_ISP_PARAM,
	CMD_WRITE_ISP_PARAM,
	CMD_GET_PREVIEW_PICTURE,
	CMD_AUTO_UPLOAD,
	CMD_UPLOAD_MAIN_INFO,
	CMD_TAKE_PICTURE,
	CMD_ISP_LEVEL,
	CMD_READ_SENSOR_REG,
	CMD_WRITE_SENSOR_REG,
};

enum {
	REG_START_PREVIEW = 1,
	REG_STOP_PREVIEW,
	REG_TAKE_PICTURE,
	REG_SET_PARAM,
	REG_MAX,
};

// This is the communication frame head
typedef struct msg_head_tag
{
	unsigned int  seq_num;      // Message sequence number, used for flow control
	unsigned short  len;          // The totoal size of the packet "sizeof(MSG_HEAD_T)
	                      // + packet size"
	unsigned char   type;         // Main command type
	unsigned char   subtype;      // Sub command type
}__attribute__((packed)) MSG_HEAD_T;

typedef struct {
	uint32_t headlen;
	uint32_t img_format;
	uint32_t img_size;
	uint32_t totalpacket;
	uint32_t packetsn;
}ISP_IMAGE_HEADER_T;

struct camera_func{
	int32_t(*start_preview) (void);
	int32_t(*stop_preview) (void);
	int32_t(*take_picture) (void);
	int32_t(*set_param) (uint32_t param);
};

#define CMD_BUF_SIZE  1024 // 1k
#define SEND_IMAGE_SIZE 64512 // 63k
#define DATA_BUF_SIZE 65536 //64k
#define PORT_NUM 16666        /* Port number for server */
#define BACKLOG 5
#define ISP_CMD_SUCCESS             0x0000
#define ISP_CMD_FAIL                0x0001
#define IMAGE_RAW_TYPE 0
#define IMAGE_YUV_TYPE 1

#define CLIENT_DEBUG
#ifdef CLIENT_DEBUG
#define DBG ALOGD
#endif

static unsigned char diag_cmd_buf[CMD_BUF_SIZE];
static unsigned char eng_rsp_diag[DATA_BUF_SIZE];
static int preview_flag = 0; // 1: start preview
static int getpic_flag = 0; // 1: call get pic
static sem_t thread_sem_lock;
static int wire_connected = 0;
static int sockfd = 0;
int sequence_num = 0;
struct camera_func s_camera_fun={0x00};
struct camera_func* s_camera_fun_ptr=&s_camera_fun;

struct camera_func* ispvideo_GetCameraFunc(void)
{
	return s_camera_fun_ptr;
}

uint32_t ispvideo_GetIspParamLenFromSt(unsigned char* dig_ptr)
{
	uint32_t data_len=0x00;
	uint16_t* data_ptr=(uint16_t*)(dig_ptr+0x05);

	if(NULL!=dig_ptr)
	{
		data_len=data_ptr[0];
		data_len-=0x08;
	}

	return data_len;
}

int ispvideo_GetIspParamFromSt(unsigned char* dig_ptr, struct isp_parser_buf_rtn* isp_ptr)
{
	int rtn=0x00;

	if((NULL!=dig_ptr)
		&&(NULL!=isp_ptr->buf_addr)
		&&(0x00!=isp_ptr->buf_len))
	{
		memcpy((void*)isp_ptr->buf_addr, (void*)dig_ptr, isp_ptr->buf_len);
	}

	return rtn;
}

uint32_t ispvideo_SetIspParamToSt(unsigned char* dig_ptr, struct isp_parser_buf_in* isp_ptr)
{
	uint32_t buf_len=0x00;

	if((NULL!=dig_ptr)
		&&(NULL!=isp_ptr->buf_addr)
		&&(0x00!=isp_ptr->buf_len))
	{
		memcpy((void*)dig_ptr, (void*)isp_ptr->buf_addr, isp_ptr->buf_len);

		buf_len=isp_ptr->buf_len;
	}

	return buf_len;
}

static int handle_img_data(uint32_t format, uint32_t width,uint32_t height,char *imgptr, int imagelen)
{
	int i,  res, number;
	int len = 0, rlen = 0, rsp_len = 0, extra_len = 0;
	MSG_HEAD_T *msg_ret;
	ISP_IMAGE_HEADER_T isp_msg;
	uint32_t size_id=0x00;

	size_id=ispParserGetSizeID(width,height);

	number = (imagelen + SEND_IMAGE_SIZE - 1) /SEND_IMAGE_SIZE;
	msg_ret = (MSG_HEAD_T *)(eng_rsp_diag+1);
	DBG("%s: imagelen[%d] number[%d]\n",__FUNCTION__, imagelen, number);

	for (i=0; i<number; i++)
	{
		if (i < number-1)
			len = SEND_IMAGE_SIZE;
		else
			len = imagelen-SEND_IMAGE_SIZE*i;
		rsp_len = sizeof(MSG_HEAD_T)+1;

		// combine data
		isp_msg.headlen = 12;
		isp_msg.img_format = format;
		isp_msg.img_size = size_id;
		isp_msg.totalpacket = number;
		isp_msg.packetsn = i+1;
		DBG("%s: request rsp_len[%d] index[%d] len[%d]\n",__FUNCTION__, rsp_len, i, len);
		memcpy(eng_rsp_diag+rsp_len, (char *)&isp_msg, sizeof(ISP_IMAGE_HEADER_T));
		rsp_len += sizeof(ISP_IMAGE_HEADER_T);

		DBG("%s: request rsp_len[%d]\n",__FUNCTION__, rsp_len);
		memcpy(eng_rsp_diag+rsp_len, (char *)imgptr+i*SEND_IMAGE_SIZE, len);
		rsp_len += len;

		eng_rsp_diag[rsp_len] = 0x7e;
		msg_ret->len = rsp_len-1;
		msg_ret->seq_num = sequence_num++;
		DBG("%s: request rsp_len[%d]\n",__FUNCTION__, rsp_len);
		res = send(sockfd, eng_rsp_diag, rsp_len+1, 0);
		DBG("%s: send success. res: %d, rsp_len: %d.\n",__FUNCTION__, res, rsp_len + 1);
	}
	return 0;
}

static int handle_isp_data(unsigned char *buf, unsigned int len)
{
	int rlen = 0, rsp_len = 0, extra_len = 0;
	int ret = 1, res = 0;
	int image_type = 0;
	MSG_HEAD_T *msg, *msg_ret;
	struct camera_func* fun_ptr=ispvideo_GetCameraFunc();

	if (len < sizeof(MSG_HEAD_T)+2){
		DBG("the formal cmd is 0x7e + diag + 0x7e,which is 10Bytes,but the cmd has less than 10 bytes\n");
		return -1;
	}

	msg = (MSG_HEAD_T *)(buf+1);
	if(msg->type != 0xfe)
		return -1;

	rsp_len = sizeof(MSG_HEAD_T)+1;
	memset(eng_rsp_diag,0,sizeof(eng_rsp_diag));
	memcpy(eng_rsp_diag,buf,rsp_len);
	msg_ret = (MSG_HEAD_T *)(eng_rsp_diag+1);

	if(CMD_GET_PREVIEW_PICTURE != msg->subtype) {
		msg_ret->seq_num = sequence_num++;
	}

	switch ( msg->subtype ) {
		case CMD_START_PREVIEW:
			DBG("ISP_TOOL:CMD_START_PREVIEW \n");
			preview_flag = 1;

			eng_rsp_diag[rsp_len] = ISP_CMD_SUCCESS;
			rsp_len++;
			eng_rsp_diag[rsp_len] = 0x7e;
			msg_ret->len = rsp_len-1;
			res = send(sockfd, eng_rsp_diag, rsp_len+1, 0);
			break;

		case CMD_STOP_PREVIEW:
			DBG("ISP_TOOL:CMD_STOP_PREVIEW \n");
			preview_flag = 0;

			eng_rsp_diag[rsp_len] = ISP_CMD_SUCCESS;
			rsp_len++;
			eng_rsp_diag[rsp_len] = 0x7e;
			msg_ret->len = rsp_len-1;
			res = send(sockfd, eng_rsp_diag, rsp_len+1, 0);
			break;

		case CMD_READ_ISP_PARAM:
		{
			DBG("ISP_TOOL:CMD_READ_ISP_PARAM \n");
			/* TODO:read isp param operation */
			// rlen is the size of isp_param
			// pass eng_rsp_diag+rsp_len
#if 1
			struct isp_parser_buf_in in_param={0x00};
			struct isp_parser_buf_rtn rtn_param={0x00};
			struct isp_parser_cmd_param rtn_cmd={0x00};
			uint8_t* dig_ptr=buf;
			uint8_t* isp_ptr=buf+sizeof(MSG_HEAD_T)+1;
			uint8_t i=0x00;
			uint32_t* addr=(uint32_t*)isp_ptr;

			DBG("ISP_TOOL: dig_ptr:0x%x \n",dig_ptr[0]);
			DBG("ISP_TOOL: seq_num:0x%x \n",msg_ret->seq_num);
			DBG("ISP_TOOL: len:0x%x \n",msg_ret->len);
			DBG("ISP_TOOL: type:0x%x \n",msg_ret->type);
			DBG("ISP_TOOL: subtype:0x%x \n",msg_ret->subtype);

			in_param.buf_len=ispvideo_GetIspParamLenFromSt(dig_ptr);
			in_param.buf_addr=(uint32_t)ispParserAlloc(in_param.buf_len);

			for(i=0x00; i<in_param.buf_len; i+=4)
			{
				DBG("ISP_TOOL: isp_param:0x%x \n",addr[0]);
				addr++;
			}

			DBG("ISP_TOOL:isp packet addr:0x%x ,len:0x%x \n",in_param.buf_addr,in_param.buf_len);


			if((0x00!=in_param.buf_len)
				&&(0x00!=in_param.buf_addr))
			{
				ret=ispvideo_GetIspParamFromSt(isp_ptr, (struct isp_parser_buf_rtn*)&in_param);
				ret=ispParser(ISP_PARSER_DOWN, (void*)in_param.buf_addr, (void*)&rtn_cmd);

				addr=(uint32_t*)in_param.buf_addr;

				for(i=0x00; i<in_param.buf_len; i+=4)
				{
					DBG("ISP_TOOL: isp_param:0x%x \n",addr[0]);
					addr++;
				}
				ret=ispParserFree((void*)in_param.buf_addr);

				if(ISP_UP_PARAM==rtn_cmd.cmd)
				{
					ret=ispParser(ISP_PARSER_UP_PARAM, (void*)in_param.buf_addr, (void*)&rtn_param);
					DBG("ISP_TOOL:isp packet addr:0x%x ,len:0x%x \n",rtn_param.buf_addr,rtn_param.buf_len);
					/*
					addr=(uint32_t*)rtn_param.buf_addr;
					for(i=0x00; i<rtn_param.buf_len; i+=4)
					{
						DBG("ISP_TOOL: isp_param:0x%x \n",addr[0]);
						addr++;
					}
					*/
					if(0x00==ret)
					{
						isp_ptr=eng_rsp_diag+sizeof(MSG_HEAD_T)+1;
						rlen=ispvideo_SetIspParamToSt(isp_ptr, (struct isp_parser_buf_in*)&rtn_param);
						/*
						addr=(uint32_t*)isp_ptr;
						for(i=0x00; i<rtn_param.buf_len; i+=4)
						{
							DBG("ISP_TOOL: isp_final_param:0x%x \n",addr[0]);
							addr++;
						}
						*/
						ret=ispParserFree((void*)rtn_param.buf_addr);
					}

					DBG("ISP_TOOL:isp ret:0x%x ,len:0x%x \n",ret,rlen);
				}
			}
#endif
			rsp_len += rlen;
			eng_rsp_diag[rsp_len] = 0x7e;
			msg_ret->len = rsp_len-1;
			DBG("ISP_TOOL:isp ret:0x%x ,len:0x%x \n",ret,msg_ret->len);
			res = send(sockfd, eng_rsp_diag, rsp_len+1, 0);
			break;
		}
		case CMD_WRITE_ISP_PARAM:
		{
			DBG("ISP_TOOL:CMD_WRITE_ISP_PARAM \n");
			/* TODO:write isp param operation */
			// pass buf+sizeof(MSG_HEAD_T)+1
			struct isp_parser_buf_in in_param={0x00};
			uint8_t* dig_ptr=buf;
			uint8_t* isp_ptr=buf+sizeof(MSG_HEAD_T)+1;
			

			in_param.buf_len=ispvideo_GetIspParamLenFromSt(dig_ptr);
			in_param.buf_addr=(uint32_t)ispParserAlloc(in_param.buf_len);
			if((0x00!=in_param.buf_len)
				&&(0x00!=in_param.buf_addr))
			{
				if(NULL!=fun_ptr->stop_preview){
				//	fun_ptr->stop_preview();
				}

				ret=ispvideo_GetIspParamFromSt(isp_ptr, (struct isp_parser_buf_rtn*)&in_param);
				ret=ispParser(ISP_PARSER_DOWN, (void*)in_param.buf_addr, NULL);
				ret=ispParserFree((void*)in_param.buf_addr);

				if(NULL!=fun_ptr->start_preview){
				//	fun_ptr->start_preview();
				}
			}
			

			eng_rsp_diag[rsp_len] = ret;
			rsp_len++;
			eng_rsp_diag[rsp_len] = 0x7e;
			msg_ret->len = rsp_len-1;
			res = send(sockfd, eng_rsp_diag, rsp_len+1, 0);
			break;
		}
		case CMD_GET_PREVIEW_PICTURE:
		{
			DBG("ISP_TOOL:CMD_GET_PREVIEW_PICTURE \n");
			image_type = *(buf+rsp_len);
			getpic_flag = 1;
			sem_wait(&thread_sem_lock);
			getpic_flag = 0;
			break;
		}
		case CMD_UPLOAD_MAIN_INFO:
		{
			DBG("ISP_TOOL:CMD_UPLOAD_MAIN_INFO \n");
			/* TODO:read isp param operation */
			// rlen is the size of isp_param
			// pass eng_rsp_diag+rsp_len
#if 1
			struct isp_parser_buf_in in_param={0x00};
			struct isp_parser_buf_rtn rtn_param={0x00};
			struct isp_parser_cmd_param rtn_cmd={0x00};
			uint8_t* dig_ptr=buf;
			uint8_t* isp_ptr=buf+sizeof(MSG_HEAD_T)+1;
			uint8_t i=0x00;
			uint32_t* addr=(uint32_t*)isp_ptr;

			DBG("ISP_TOOL: dig_ptr:0x%x \n",dig_ptr[0]);
			DBG("ISP_TOOL: seq_num:0x%x \n",msg_ret->seq_num);
			DBG("ISP_TOOL: len:0x%x \n",msg_ret->len);
			DBG("ISP_TOOL: type:0x%x \n",msg_ret->type);
			DBG("ISP_TOOL: subtype:0x%x \n",msg_ret->subtype);

			in_param.buf_len=ispvideo_GetIspParamLenFromSt(dig_ptr);
			in_param.buf_addr=(uint32_t)ispParserAlloc(in_param.buf_len);

			if((0x00!=in_param.buf_len)
				&&(0x00!=in_param.buf_addr))
			{
				ret=ispvideo_GetIspParamFromSt(isp_ptr, (struct isp_parser_buf_rtn*)&in_param);
				ret=ispParser(ISP_PARSER_DOWN, (void*)in_param.buf_addr, (void*)&rtn_cmd);
				ret=ispParserFree((void*)in_param.buf_addr);

				if(ISP_MAIN_INFO==rtn_cmd.cmd)
				{
					ret=ispParser(ISP_PARSER_UP_MAIN_INFO, (void*)in_param.buf_addr, (void*)&rtn_param);
					if(0x00==ret)
					{
						isp_ptr=eng_rsp_diag+sizeof(MSG_HEAD_T)+1;
						rlen=ispvideo_SetIspParamToSt(isp_ptr, (struct isp_parser_buf_in*)&rtn_param);
						ret=ispParserFree((void*)rtn_param.buf_addr);
						addr=(uint32_t*)isp_ptr;
						for(i=0x00; i<rtn_param.buf_len; i+=4)
						{
							DBG("ISP_TOOL: isp_final_param:0x%x \n",addr[0]);
							addr++;
						}
					}
				}
			}
#endif
			rsp_len += rlen;
			eng_rsp_diag[rsp_len] = 0x7e;
			msg_ret->len = rsp_len-1;
			res = send(sockfd, eng_rsp_diag, rsp_len+1, 0);
			break ;
		}
		case CMD_TAKE_PICTURE:
		{
			DBG("ISP_TOOL:CMD_TAKE_PICTURE \n");
#if 1
			struct isp_parser_buf_in in_param={0x00};
			struct isp_parser_buf_rtn rtn_param={0x00};
			struct isp_parser_cmd_param rtn_cmd={0x00};
			uint8_t* dig_ptr=buf;
			uint8_t* isp_ptr=buf+sizeof(MSG_HEAD_T)+1;
			uint8_t i=0x00;
			uint32_t* addr=(uint32_t*)isp_ptr;

			DBG("ISP_TOOL: dig_ptr:0x%x \n",dig_ptr[0]);
			DBG("ISP_TOOL: seq_num:0x%x \n",msg_ret->seq_num);
			DBG("ISP_TOOL: len:0x%x \n",msg_ret->len);
			DBG("ISP_TOOL: type:0x%x \n",msg_ret->type);
			DBG("ISP_TOOL: subtype:0x%x \n",msg_ret->subtype);

			in_param.buf_len=ispvideo_GetIspParamLenFromSt(dig_ptr);
			in_param.buf_addr=(uint32_t)ispParserAlloc(in_param.buf_len);

			for(i=0x00; i<in_param.buf_len; i+=4)
			{
				DBG("ISP_TOOL: isp_param:0x%x \n",addr[0]);
				addr++;
			}

			DBG("ISP_TOOL:isp packet addr:0x%x ,len:0x%x \n",in_param.buf_addr,in_param.buf_len);

			if((0x00!=in_param.buf_len)
				&&(0x00!=in_param.buf_addr))
			{
				ret=ispParser(ISP_PARSER_DOWN, (void*)&in_param, (void*)&rtn_cmd);
				ret=ispParserFree((void*)in_param.buf_addr);

				DBG("ISP_TOOL:rtn_cmd.cmd:0x%x \n",rtn_cmd.cmd);

				if(ISP_CAPTURE==rtn_cmd.cmd)
				{
					if(NULL!=fun_ptr->take_picture){
					//	fun_ptr->take_picture();
					}
				}
			}
#endif
			image_type = *(buf+rsp_len);
			getpic_flag = 1;
			sem_wait(&thread_sem_lock);
			getpic_flag = 0;
			break;
		}
		case CMD_ISP_LEVEL:
		{
			DBG("ISP_TOOL:CMD_ISP_LEVEL \n");
			struct isp_parser_buf_in in_param={0x00};
			struct isp_parser_buf_rtn rtn_param={0x00};
			uint8_t* dig_ptr=buf;
			uint8_t* isp_ptr=buf+sizeof(MSG_HEAD_T)+1;

			in_param.buf_len=ispvideo_GetIspParamLenFromSt(dig_ptr);
			in_param.buf_addr=(uint32_t)ispParserAlloc(in_param.buf_len);

			if((0x00!=in_param.buf_len)
				&&(0x00!=in_param.buf_addr))
			{
				ret=ispvideo_GetIspParamFromSt(isp_ptr, (struct isp_parser_buf_rtn*)&in_param);
				ret=ispParser(ISP_PARSER_DOWN, (void*)in_param.buf_addr, NULL);
				ret=ispParserFree((void*)in_param.buf_addr);
			}

			eng_rsp_diag[rsp_len] = ISP_CMD_SUCCESS;
			rsp_len++;
			eng_rsp_diag[rsp_len] = 0x7e;
			msg_ret->len = rsp_len-1;
			res = send(sockfd, eng_rsp_diag, rsp_len+1, 0);
			break;
		}

		case CMD_READ_SENSOR_REG:
		{// ISSUE
			DBG("ISP_TOOL:CMD_READ_SENSOR_REG \n");
			struct isp_parser_buf_in in_param={0x00};
			struct isp_parser_cmd_param rtn_cmd={0x00};
			struct isp_parser_buf_rtn rtn_param={0x00};
			uint8_t* dig_ptr=buf;
			uint8_t* isp_ptr=buf+sizeof(MSG_HEAD_T)+1;
#if 1
			in_param.buf_len=ispvideo_GetIspParamLenFromSt(dig_ptr);
			in_param.buf_addr=(uint32_t)ispParserAlloc(in_param.buf_len);

			if((0x00!=in_param.buf_len)
				&&(0x00!=in_param.buf_addr))
			{
				ret=ispParser(ISP_PARSER_DOWN, (void*)&in_param, (void*)&rtn_param);
				ret=ispParserFree((void*)in_param.buf_addr);

				ret=ispParser(ISP_PARSER_UP_SENSOR_REG, (void*)&in_param, (void*)&rtn_param);
				if(0x00==ret)
				{
					isp_ptr=eng_rsp_diag+sizeof(MSG_HEAD_T)+1;
					rlen=ispvideo_SetIspParamToSt(isp_ptr, (struct isp_parser_buf_in*)&rtn_param);
					ret=ispParserFree((void*)rtn_param.buf_addr);
				}

				
			}
#endif
			eng_rsp_diag[rsp_len] = ISP_CMD_SUCCESS;
			rsp_len++;
			eng_rsp_diag[rsp_len] = 0x7e;
			msg_ret->len = rsp_len-1;
			res = send(sockfd, eng_rsp_diag, rsp_len+1, 0);
			break;
		}
		case CMD_WRITE_SENSOR_REG:
		{
			DBG("ISP_TOOL:CMD_WRITE_SENSOR_REG \n");
			struct isp_parser_buf_in in_param={0x00};
			struct isp_parser_buf_rtn rtn_param={0x00};
			uint8_t* dig_ptr=buf;
			uint8_t* isp_ptr=buf+sizeof(MSG_HEAD_T)+1;

			in_param.buf_len=ispvideo_GetIspParamLenFromSt(dig_ptr);
			in_param.buf_addr=(uint32_t)ispParserAlloc(in_param.buf_len);

			if((0x00!=in_param.buf_len)
				&&(0x00!=in_param.buf_addr))
			{
				ret=ispvideo_GetIspParamFromSt(isp_ptr, (struct isp_parser_buf_rtn*)&in_param);
				ret=ispParser(ISP_PARSER_DOWN, (void*)&in_param, (void*)&rtn_param);
				ret=ispParserFree((void*)in_param.buf_addr);
			}

			eng_rsp_diag[rsp_len] = ISP_CMD_SUCCESS;
			rsp_len++;
			eng_rsp_diag[rsp_len] = 0x7e;
			msg_ret->len = rsp_len-1;
			res = send(sockfd, eng_rsp_diag, rsp_len+1, 0);
			break;
		}
		default:
			break;
	}				/* -----  end switch  ----- */
	return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */

/*

#define ISP_DATA_YUV422_2FRAME (1<<0)
#define ISP_DATA_YUV420_2FRAME (1<<1)
#define ISP_DATA_NORMAL_RAW10 (1<<2)
#define ISP_DATA_MIPI_RAW10 (1<<3)
#define ISP_DATA_JPG (1<<4)

*/
void send_img_data(uint32_t format, uint32_t width, uint32_t height, char *imgptr, int imagelen)
{
	int ret;

	if (( (preview_flag == 1) && (getpic_flag == 1)) ||
		( (preview_flag == 0) && (getpic_flag == 1)))
	{
		DBG("%s: preview_flag: %d, getpic_flag: %d, imagelen: %d.\n", __FUNCTION__, preview_flag, getpic_flag, imagelen);
		getpic_flag = 0;
		ret = handle_img_data(format, width, height, imgptr, imagelen);
		sem_post(&thread_sem_lock);
		if (ret != 0) {
			DBG("%s: Fail to handle_img_data(). ret = %d.", __FUNCTION__, ret);
		}
	}
	else {
		DBG("%s: no data. preview_flag: %d, getpic_flag: %d.\n", __FUNCTION__, preview_flag, getpic_flag);
	}
}

static void * isp_diag_handler(void *args)
{
	int from = *((int *)args);
	static char *code = "diag channel exit";
	fd_set rfds;
	struct timeval tv;

	FD_ZERO(&rfds);
	FD_SET(from, &rfds);

	sockfd = from;
	/* Read client request, send sequence number back */
	while (wire_connected) {
		int i, cnt, res;
		/* Wait up to  two seconds. */
		tv.tv_sec = 2;
		tv.tv_usec = 0;
		FD_SET(from, &rfds);
		res = select(from + 1, &rfds, NULL, NULL, &tv);
		if (res <= 0) { //timeout or other error
			DBG("No data within five seconds. res:%d\n", res);
			continue;
		}
		//cnt = read(diag->from, diag_cmd_buf, DATA_BUF_SIZE);
		cnt = recv(from, diag_cmd_buf, CMD_BUF_SIZE,
				MSG_DONTWAIT);
		//DBG("read from socket %d\n", cnt);
		if (cnt <= 0) {
			DBG("read socket error %s\n", strerror(errno));
			break;
		}
		DBG("%s: request buffer[%d]\n",__FUNCTION__, cnt);
		for(i=0; i<cnt; i++)
			DBG("%x,",diag_cmd_buf[i]);
		DBG("\n");

		handle_isp_data(diag_cmd_buf, cnt);
	}
	return code;
}

static void * ispserver_thread(void *args)
{
	struct sockaddr claddr;
	int lfd, cfd, optval;
	int log_fd;
	struct sockaddr_in sock_addr;
	socklen_t addrlen;
#ifdef CLIENT_DEBUG
#define ADDRSTRLEN (128)
	char addrStr[ADDRSTRLEN];
	char host[50];
	char service[30];
#endif
	pthread_t tdiag;
	pthread_attr_t attr;

	DBG("isp-video server version 1.0\n");

	memset(&sock_addr, 0, sizeof (struct sockaddr_in));
	sock_addr.sin_family = AF_INET;        /* Allows IPv4*/
	sock_addr.sin_addr.s_addr = INADDR_ANY;/* Wildcard IP address;*/
	sock_addr.sin_port = htons(PORT_NUM);

	lfd = socket(sock_addr.sin_family, SOCK_STREAM, 0);
	if (lfd == -1) {
		 DBG("socket error\n");
		 return NULL;
	}

	optval = 1;
	if (setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) != 0) {
		 DBG("setsockopt error\n");
		 return NULL;
	}

	if (bind(lfd, (struct sockaddr *)&sock_addr, sizeof (struct sockaddr_in)) != 0) {
		DBG("bind error %s\n", strerror(errno));
		 return NULL;
	}

	if (listen(lfd, BACKLOG) == -1){
		DBG("listen error\n");
		 return NULL;
	}

	sem_init(&thread_sem_lock, 0, 0);
	pthread_attr_init(&attr);
	for (;;) {                  /* Handle clients iteratively */
		void * res;
		int ret;

		DBG("log server waiting client dail in...\n");
		/* Accept a client connection, obtaining client's address */
		addrlen = sizeof(struct sockaddr);
		cfd = accept(lfd, &claddr, &addrlen);
		if (cfd == -1) {
			DBG("accept error %s\n", strerror(errno));
			break;
		}
		DBG("log server connected with client\n");
		wire_connected = 1;
		sequence_num = 0;
		/* Ignore the SIGPIPE signal, so that we find out about broken
		 * connection errors via a failure from write().
		 */
		if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
			DBG("signal error\n");
#ifdef CLIENT_DEBUG
		addrlen = sizeof(struct sockaddr);
		if (getnameinfo(&claddr, addrlen, host, 50, service,
			 30, NI_NUMERICHOST) == 0)
			snprintf(addrStr, ADDRSTRLEN, "(%s, %s)", host, service);
		else
			snprintf(addrStr, ADDRSTRLEN, "(?UNKNOWN?)");
		DBG("Connection from %s\n", addrStr);
#endif

		//create a thread for recv cmd
		ret = pthread_create(&tdiag, &attr, isp_diag_handler, &cfd);
		if (ret != 0) {
			DBG("diag thread create success\n");
			break;
		}

		pthread_join(tdiag, &res);
		DBG("diag thread exit success %s\n", (char *)res);
		if (close(cfd) == -1)           /* Close connection */
			DBG("close socket cfd error\n");
	}
	pthread_attr_destroy(&attr);
	if (close(lfd) == -1)           /* Close connection */
		DBG("close socket lfd error\n");

	 return NULL;
}

int ispvideo_RegCameraFunc(uint32_t cmd, int(*func)())
{
	struct camera_func* fun_ptr=ispvideo_GetCameraFunc();
	
	switch(cmd)
	{
		case REG_START_PREVIEW:
		{
			fun_ptr->start_preview=func;
			break;
		}
		case REG_STOP_PREVIEW:
		{
			fun_ptr->stop_preview=func;
			break;
		}
		case REG_TAKE_PICTURE:
		{
			fun_ptr->take_picture=func;
			break;
		}
		case REG_SET_PARAM:
		{
			fun_ptr->set_param=func;
			break;
		}
		default :
		{
			break;
		}
	}
	
	return 0x00;
}


void stopispserver()
{
	/*
	if (sockfd != 0)
		close(sockfd);
	if (serverfd != 0)
		close(serverfd);
	flag = 0;
	*/
}

void startispserver()
{
	pthread_t tdiag;
	pthread_attr_t attr;
	int ret;
	static int flag = 0; // confirm called only once

	if (flag == 1)
		return;

	DBG("startispserver\n");
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	ret = pthread_create(&tdiag, &attr, ispserver_thread, NULL);
	pthread_attr_destroy(&attr);
	if (ret < 0) {
		DBG("pthread_create fail\n");
		return;
	}
	flag = 1;
}
