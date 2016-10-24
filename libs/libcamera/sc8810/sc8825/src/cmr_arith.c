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
#include <stdlib.h>
#include <fcntl.h>              /* low-level i/o */
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <semaphore.h>
#include "SprdOEMCamera.h"
#include "cmr_common.h"
#include "cmr_msg.h"
#include "cmr_oem.h"


#define ARITHMETIC_EVT_FD_START	      (1 << 16)
#define ARITHMETIC_EVT_FD_EXIT	      (1 << 17)
#define ARITHMETIC_EVT_MASK_BITS      (uint32_t)(ARITHMETIC_EVT_FD_START | ARITHMETIC_EVT_FD_EXIT)


#define CAMERA_FD_MSG_QUEUE_SIZE      5
#define IMAGE_FORMAT		          "YVU420_SEMIPLANAR"

enum arithmetic_ret {
	ARITH_SUCCESS = 0,
	ARITH_INIT_FAIL,
	ARITH_START_FAIL,
	ARITH_NO_MEM,
	ARITH_FAIL
};

struct arithmetic_conext{
	uint32_t        fd_msg_que_handle;
	pthread_t       fd_thread;
	sem_t           fd_sync_sem;
	pthread_mutex_t fd_lock;
	uint32_t        fd_exit_flag;
	uint32_t        fd_busy;
	pthread_mutex_t hdr_lock;
	void            *addr;
	void            *phy_addr;
	uint32_t        mem_size;
};

struct arithmetic_hdr_conext{
	unsigned char *addr[HDR_CAP_NUM];
	uint32_t       mem_size;
};

static struct arithmetic_conext s_arithmetix_cxt;
static struct arithmetic_conext *s_arith_cxt = &s_arithmetix_cxt;
static struct arithmetic_hdr_conext s_hdr_cntext;
static struct arithmetic_hdr_conext *s_hdr_cxt = &s_hdr_cntext;



void *arithmetic_fd_thread_proc(void *data)
{
	CMR_MSG_INIT(message);
	int                 ret = CAMERA_SUCCESS;
	int                 evt = 0;
	void                *addr = 0;
	int                 face_num;
	int                 k = 0;
	morpho_FaceRect     *face_rect_ptr;
	unsigned char       *p_format = (unsigned char*)IMAGE_FORMAT;
	camera_frame_type   frame_type;
	int                 fd_exit_flag = 0;

	while (1) {
		ret = cmr_msg_get(s_arith_cxt->fd_msg_que_handle, &message);
		if (ret) {
			CMR_LOGE("Message queue destroied");
			break;
		}

		CMR_LOGV("message.msg_type 0x%x, data 0x%x",
			message.msg_type,
			(unsigned int)message.data);

		evt = (uint32_t)(message.msg_type & ARITHMETIC_EVT_MASK_BITS);
		switch (evt) {
		case ARITHMETIC_EVT_FD_START:
			CMR_PRINT_TIME;
			s_arith_cxt->fd_busy = 1;
			sem_post(&s_arith_cxt->fd_sync_sem);
			pthread_mutex_lock(&s_arith_cxt->fd_lock);
			addr = message.data;
			if (NULL == s_arith_cxt->addr) {
				s_arith_cxt->fd_busy = 0;
				pthread_mutex_unlock(&s_arith_cxt->fd_lock);
				break;
			}
			memcpy(s_arith_cxt->addr,addr,s_arith_cxt->mem_size);
			frame_type.face_num = 0;
			if( 0 != FaceSolid_Function((uint8_t*)s_arith_cxt->addr,
				                         &face_rect_ptr,
				                         (int*)&face_num,
				                         0,p_format)) {
				CMR_LOGE("FaceSolid_Function fail.");
			} else {
				frame_type.face_ptr = face_rect_ptr;
				frame_type.face_num = face_num;
				for (k=0 ; k<face_num ; k++) {
					CMR_LOGI("face num %d,smile_level %d.",k,face_rect_ptr->smile_level);
					face_rect_ptr++;
				}
				camera_call_cb(CAMERA_EVT_CB_FD,
							camera_get_client_data(),
							CAMERA_FUNC_START_PREVIEW,
							(uint32_t)&frame_type);
			}
			s_arith_cxt->fd_busy = 0;
			pthread_mutex_unlock(&s_arith_cxt->fd_lock);
			break;
		case ARITHMETIC_EVT_FD_EXIT:
			fd_exit_flag = 1;
			break;
		default:
			break;
		}
		if (1 == fd_exit_flag) {
			CMR_LOGV("FD proc exit.");
			sem_post(&s_arith_cxt->fd_sync_sem);
			break;
		}
	}
	CMR_LOGV("exit.");
	return NULL;
}

int arithmetic_fd_init(void)
{
	CMR_MSG_INIT(message);
	struct camera_context  *cxt = camera_get_cxt();
	unsigned char          *p_format = (unsigned char*)IMAGE_FORMAT;
	int                    ret = ARITH_SUCCESS;

	CMR_LOGV("inited, %d", cxt->arithmetic_cxt.fd_inited);

	if (cxt->arithmetic_cxt.fd_inited) {
		CMR_LOGE("No need to init fd");
		return 0;
	}

	CMR_PRINT_TIME;
	if ( 0 != FaceSolid_Init(cxt->display_size.height,
		                     cxt->display_size.width,
		                     p_format)) {
		ret = -ARITH_INIT_FAIL;
		CMR_LOGE("FaceSolid_Init fail.");
	} else {
		CMR_LOGI("FaceSolid_Init done.");
	}
	
	if (!ret) {
		ret = cmr_msg_queue_create(CAMERA_FD_MSG_QUEUE_SIZE, &s_arith_cxt->fd_msg_que_handle);
		if (ret) {
			CMR_LOGE("NO Memory, Failed to create FD message queue");
		} else {
			sem_init(&s_arith_cxt->fd_sync_sem, 0, 0);
			pthread_mutex_init(&s_arith_cxt->fd_lock, NULL);
			ret = pthread_create(&s_arith_cxt->fd_thread, NULL, arithmetic_fd_thread_proc, NULL);
			cxt->arithmetic_cxt.fd_inited = 1;
		}
	}
	pthread_mutex_init(&s_arith_cxt->hdr_lock, NULL);
	return ret;
}

int arithmetic_fd_deinit(void)
{
	CMR_MSG_INIT(message);
	int                    ret = ARITH_SUCCESS;
	struct camera_context  *cxt = camera_get_cxt();
		
	CMR_LOGI("s.");
	CMR_PRINT_TIME;
	if (1 == cxt->arithmetic_cxt.fd_inited) {
		message.msg_type = ARITHMETIC_EVT_FD_EXIT;
		ret = cmr_msg_post(s_arith_cxt->fd_msg_que_handle, &message);

		if(CMR_MSG_SUCCESS == ret) {
			sem_wait(&s_arith_cxt->fd_sync_sem);
		}
		sem_destroy(&s_arith_cxt->fd_sync_sem);
		pthread_mutex_destroy(&s_arith_cxt->fd_lock);
		cmr_msg_queue_destroy(s_arith_cxt->fd_msg_que_handle);
		FaceSolid_Finalize();
		cxt->arithmetic_cxt.fd_inited = 0;
		CMR_LOGI("FaceSolid_Finalize done.");
	}
	memset(s_arith_cxt, 0, sizeof(struct arithmetic_conext));
	pthread_mutex_destroy(&s_arith_cxt->hdr_lock);
	CMR_PRINT_TIME;
	CMR_LOGI("e.");
	return ret;
}

int arithmetic_fd_start(void *data_addr)
{
	int ret = ARITH_SUCCESS;
	uint32_t phy_addr = 0;
	CMR_MSG_INIT(message);

	CMR_LOGI("0x%x.",(uint32_t)s_arith_cxt->addr);

	pthread_mutex_lock(&s_arith_cxt->fd_lock);
	if (NULL == s_arith_cxt->addr) {		
		ret = ARITH_NO_MEM;
	}
	pthread_mutex_unlock(&s_arith_cxt->fd_lock);
	CMR_LOGI("%d,%d.",ret,s_arith_cxt->fd_busy);
	if (!ret && (1 != s_arith_cxt->fd_busy)) {
		message.msg_type = ARITHMETIC_EVT_FD_START;
		message.data = data_addr;
		ret = cmr_msg_post(s_arith_cxt->fd_msg_que_handle, &message);

		if(CMR_MSG_SUCCESS == ret) {
			sem_wait(&s_arith_cxt->fd_sync_sem);
		} else {
			ret = ARITH_START_FAIL;
			CMR_LOGE("fail.");
		}
	} else {
		CMR_LOGI("discarded.");
	}
	return ret;
}
void arithmetic_set_mem(uint32_t phy_addr, uint32_t vir_addr, uint32_t mem_size)
{
	pthread_mutex_lock(&s_arith_cxt->fd_lock);
	s_arith_cxt->mem_size = mem_size;
	s_arith_cxt->addr = (void*)vir_addr;
	s_arith_cxt->phy_addr = (void*)phy_addr;
	pthread_mutex_unlock(&s_arith_cxt->fd_lock);
	CMR_LOGI("0x%x,0x%x.",(uint32_t)s_arith_cxt->addr,vir_addr);
}

int arithmetic_hdr_init(uint32_t pic_width, uint32_t pic_height)
{
	int ret = ARITH_SUCCESS;
	uint32_t size = pic_width * pic_height * 3/2;

	pthread_mutex_lock(&s_arith_cxt->hdr_lock);

	s_hdr_cxt->addr[0] = (uint8_t*)malloc(size);
	s_hdr_cxt->addr[1] = (uint8_t*)malloc(size);
	s_hdr_cxt->addr[2] = (uint8_t*)malloc(size);

	if ((PNULL == s_hdr_cxt->addr[0]) || (PNULL == s_hdr_cxt->addr[1]) \
		|| (PNULL == s_hdr_cxt->addr[2])) {
		ret = ARITH_NO_MEM;
		CMR_LOGE("malloc fail.");
	} else {
		s_hdr_cxt->mem_size = size;
	}
	pthread_mutex_unlock(&s_arith_cxt->hdr_lock);
	return ret;
}

int arithmetic_hdr_deinit(void)
{
	int ret = ARITH_SUCCESS;
	CMR_LOGI("s.");
	pthread_mutex_lock(&s_arith_cxt->hdr_lock);
	if (PNULL != s_hdr_cxt->addr[0]) {
		free(s_hdr_cxt->addr[0]);
		s_hdr_cxt->addr[0] = PNULL;
	}
	if (PNULL != s_hdr_cxt->addr[1]) {
		free(s_hdr_cxt->addr[1]);
		s_hdr_cxt->addr[1] = PNULL;
	}
	if (PNULL != s_hdr_cxt->addr[2]) {
		free(s_hdr_cxt->addr[2]);
		s_hdr_cxt->addr[2] = PNULL;
	}
	pthread_mutex_unlock(&s_arith_cxt->hdr_lock);
	CMR_LOGI("e.");
	return ret;
}
int arithmetic_hdr(unsigned char *dst_addr,uint32_t width,uint32_t height)
{
	int           ret = ARITH_SUCCESS;
	uint32_t      size = width*height*3/2;
	unsigned char *temp_addr0 = PNULL;
	unsigned char *temp_addr1 = PNULL;
	unsigned char *temp_addr2 = PNULL;
	char           *p_format = IMAGE_FORMAT;

	pthread_mutex_lock(&s_arith_cxt->hdr_lock);
	temp_addr0 = s_hdr_cxt->addr[0];
	temp_addr1 = s_hdr_cxt->addr[1];
	temp_addr2 = s_hdr_cxt->addr[2];

	if ((NULL != temp_addr0) && (NULL != temp_addr1) && (NULL != temp_addr2)) {
		if (0 != HDR_Function(temp_addr0,temp_addr1,temp_addr2,	temp_addr0,
								height,width,p_format)) {
			CMR_LOGE("hdr error!");
			ret = ARITH_FAIL;
		}
	} else {
			CMR_LOGE("can't handle hdr.");
			ret = ARITH_FAIL;
	}
	pthread_mutex_unlock(&s_arith_cxt->hdr_lock);
	if (ARITH_SUCCESS == ret) {
		CMR_LOGI("hdr done.");
	}
	return ret;
}

void arithmetic_hdr_data(unsigned char *addr,uint32_t size,uint32_t cap_cnt)
{
	CMR_LOGI("0x%x,%d,%d.",(uint32_t)addr,size,cap_cnt);
	if (cap_cnt > HDR_CAP_NUM) {
		CMR_LOGE("cap cnt error,%d.",cap_cnt);
		return;
	}
	pthread_mutex_lock(&s_arith_cxt->hdr_lock);
	if (PNULL == s_hdr_cxt->addr[cap_cnt-1]) {
		CMR_LOGE("no memory.");
		return;
	}

	if (s_hdr_cxt->mem_size >= size) {
		memcpy(s_hdr_cxt->addr[cap_cnt-1],addr,size);
	} else {
		CMR_LOGE("mem size:0x%x,data size:0x%x.",s_hdr_cxt->mem_size,size);
	}
	pthread_mutex_unlock(&s_arith_cxt->hdr_lock);
}
