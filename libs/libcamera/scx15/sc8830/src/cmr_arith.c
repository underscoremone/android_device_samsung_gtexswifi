/*
 * Copyright (C) 2012 The Android Open Source Project
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
#include <fcntl.h>
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

#define ARITHMETIC_EVT_FD_START          (1 << 16)
#define ARITHMETIC_EVT_FD_EXIT           (1 << 17)
#define ARITHMETIC_EVT_FD_INIT           (1 << 18)
#define ARITHMETIC_EVT_MEM               (1 << 19)
#define ARITHMETIC_EVT_MASK_BITS      (uint32_t)(ARITHMETIC_EVT_FD_START | \
					ARITHMETIC_EVT_FD_EXIT | \
					ARITHMETIC_EVT_FD_INIT | \
					ARITHMETIC_EVT_MEM)
#define CAMERA_FD_MSG_QUEUE_SIZE 5
#define IMAGE_FORMAT "YVU420_SEMIPLANAR"

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
	uint32_t        fd_is_inited;
	uint32_t        fd_num;
	uint32_t        arith_fd_flag;
	struct img_size fd_size;
	uint32_t        mem_state;
};

struct arithmetic_hdr_conext{
	unsigned char *addr[HDR_CAP_NUM];
	uint32_t       mem_size;
	uint32_t       inited;
};

static struct arithmetic_conext s_arithmetix_cxt;
static struct arithmetic_conext *s_arith_cxt = &s_arithmetix_cxt;
static struct arithmetic_hdr_conext s_hdr_cntext;
static struct arithmetic_hdr_conext *s_hdr_cxt = &s_hdr_cntext;
static uint32_t check_size_data_invalid(struct img_size * fd_size);
static int arithmetic_fd_call_init(const struct img_size * fd_size);

int FaceSolid_Init(int width, int height)
{
	/*dummy function*/
	CMR_LOGW("dummy function! should not be here!");
	return ARITH_SUCCESS;
}

int FaceSolid_Function(unsigned char *src, ACCESS_FaceRect ** ppDstFaces, int *pDstFaceNum ,int skip)
{
	/*dummy function*/
	CMR_LOGW("dummy function! should not be here!");
	return ARITH_SUCCESS;
}

int FaceSolid_Finalize()
{
	/*dummy function*/
	CMR_LOGW("dummy function! should not be here!");
	return ARITH_SUCCESS;
}

uint32_t arithmetic_fd_is_init(void)
{
	if (0 == s_arith_cxt->fd_is_inited) return 0;
	return 1;
}

uint32_t arithmetic_get_fd_num(void)
{
	return s_arith_cxt->fd_num;
}

uint32_t arithmetic_fd_is_eb(void)
{
	if (0 == s_arith_cxt->arith_fd_flag) return 0;
	return 1;
}

void arithmetic_set_fd_eb(uint32_t param)
{
	s_arith_cxt->arith_fd_flag = param;
}

uint32_t check_size_data_invalid(struct img_size * fd_size)
{
	uint32_t ret = 0;
	if (NULL != fd_size) {
		if ((fd_size->width)&&(fd_size->height)){
			ret= 1;
		}
	}
	return ret;
}

void *arithmetic_fd_thread_proc(void *data)
{
	CMR_MSG_INIT(message);
	int                 ret = CAMERA_SUCCESS;
	int                 evt = 0;
	void                *addr = 0;
	int                 face_num = 0;
	int                 k = 0;
	morpho_FaceRect     *face_rect_ptr = PNULL;
	camera_frame_type   frame_type;
	int                 fd_exit_flag = 0;
	camera_cb_info      cb_info;
	uint32_t            mem_size = 0;

	sem_post(&s_arith_cxt->fd_sync_sem);
	while (1) {
		ret = cmr_msg_get(s_arith_cxt->fd_msg_que_handle, &message, 1);
		if (ret) {
			CMR_LOGE("Message queue destroyed");
			break;
		}

		CMR_LOGI("message.msg_type 0x%x, data 0x%x",
			message.msg_type,
			(unsigned int)message.data);

		evt = (uint32_t)(message.msg_type & ARITHMETIC_EVT_MASK_BITS);

		switch (evt) {
		case ARITHMETIC_EVT_FD_INIT:
			CMR_PRINT_TIME;
			if (check_size_data_invalid((struct img_size *)(message.data))) {
				FaceSolid_Finalize();

				if ( 0 != FaceSolid_Init(((struct img_size *)(message.data))->width,
				((struct img_size *)(message.data))->height)) {
					ret = -ARITH_INIT_FAIL;
					CMR_LOGE("FaceSolid_Init fail.");
				} else {
					CMR_LOGD("FaceSolid_Init done.");
				}
			}

			if (0 != s_arith_cxt->mem_size) {
				CMR_LOGI("initial with mem_size ");
				if (NULL != s_arith_cxt->addr) {
					free(s_arith_cxt->addr);
				}

				s_arith_cxt->addr = malloc(mem_size);
				if (NULL == s_arith_cxt->addr) {
					CMR_LOGE("Fail to alloc FD mem");
					s_arith_cxt->mem_state = ARITH_NO_MEM;
				} else {
					s_arith_cxt->mem_size = mem_size;
					s_arith_cxt->phy_addr = NULL; /*the physical address is unuseable yet*/
					s_arith_cxt->mem_state = ARITH_SUCCESS;
				}
			}
			CMR_PRINT_TIME;
			break;

		case ARITHMETIC_EVT_MEM:
			mem_size = *(uint32_t *)(message.data);
			if (0 == mem_size) {
				if (s_arith_cxt->addr) {
					free(s_arith_cxt->addr);
					s_arith_cxt->addr = NULL;
					s_arith_cxt->mem_size = 0;
					s_arith_cxt->phy_addr = NULL;
				}
				s_arith_cxt->mem_state = ARITH_SUCCESS;
			} else if (s_arith_cxt->mem_size != mem_size) {
				if (s_arith_cxt->addr) {
					free(s_arith_cxt->addr);
					s_arith_cxt->addr = NULL;
					s_arith_cxt->mem_size = 0;
					s_arith_cxt->phy_addr = NULL;
				}

				s_arith_cxt->addr = malloc(mem_size);
				if (NULL == s_arith_cxt->addr) {
					CMR_LOGE("Fail to alloc FD mem");
					s_arith_cxt->mem_state = ARITH_NO_MEM;
				} else {
					s_arith_cxt->mem_size = mem_size;
					s_arith_cxt->phy_addr = NULL; /*the physical address is unuseable yet*/
					s_arith_cxt->mem_state = ARITH_SUCCESS;
				}
			} else {
				CMR_LOGI("same size req, use origin buffer");
			}
			break;

		case ARITHMETIC_EVT_FD_START:
			CMR_PRINT_TIME;
			s_arith_cxt->fd_num = 0;
			s_arith_cxt->fd_busy = 1;
			pthread_mutex_lock(&s_arith_cxt->fd_lock);
			if ((NULL == s_arith_cxt->addr) || (ARITH_SUCCESS != s_arith_cxt->mem_state)) {
				CMR_LOGE("FD memory NULL error");
				sem_post(&s_arith_cxt->fd_sync_sem);
				pthread_mutex_unlock(&s_arith_cxt->fd_lock);
				break;
			}
			if (CMR_IDLE == camera_get_prev_stat()) {
				s_arith_cxt->fd_busy = 0;
				sem_post(&s_arith_cxt->fd_sync_sem);
				pthread_mutex_unlock(&s_arith_cxt->fd_lock);
				break;
			}
			addr = message.data;
			if (NULL == s_arith_cxt->addr) {
				s_arith_cxt->fd_busy = 0;
				sem_post(&s_arith_cxt->fd_sync_sem);
				pthread_mutex_unlock(&s_arith_cxt->fd_lock);
				break;
			}
			memcpy(s_arith_cxt->addr,addr,s_arith_cxt->mem_size);
			sem_post(&s_arith_cxt->fd_sync_sem);
			frame_type.face_num = 0;
			if ( 0 != FaceSolid_Function((uint8_t*)s_arith_cxt->addr,
				&face_rect_ptr,
				(int*)&face_num,
				0)) {
				CMR_LOGE("FaceSolid_Function fail.");
			} else {
				frame_type.face_ptr = face_rect_ptr;
				frame_type.face_num = face_num;
				s_arith_cxt->fd_num = face_num;
				if (CMR_IDLE == camera_get_prev_stat()) {
					s_arith_cxt->fd_busy = 0;
					pthread_mutex_unlock(&s_arith_cxt->fd_lock);
					break;
				}
				for (k=0 ; k<face_num ; k++) {
					CMR_LOGD("face num %d,smile_level %d.",k,face_rect_ptr->smile_level);
					face_rect_ptr++;
				}
				if (s_arith_cxt->arith_fd_flag) {
					camera_direct_call_cb(CAMERA_EVT_CB_FD,
								camera_get_client_data(),
								CAMERA_FUNC_START_PREVIEW,
								(uint32_t)&frame_type);
				}
			}
			s_arith_cxt->fd_busy = 0;
			pthread_mutex_unlock(&s_arith_cxt->fd_lock);
			break;

		case ARITHMETIC_EVT_FD_EXIT:
			FaceSolid_Finalize();
			fd_exit_flag = 1;
			break;

		default:
			break;
		}

		if (message.alloc_flag) {
			free(message.data);
		}

		if (1 == fd_exit_flag) {
			CMR_LOGI("FD proc exit.");
			sem_post(&s_arith_cxt->fd_sync_sem);
			break;
		}

	}
	CMR_LOGI("exit.");
	return NULL;
}

int arithmetic_fd_call_init(const struct img_size * fd_size)
{
	CMR_MSG_INIT(message);
	int          ret = ARITH_SUCCESS;

	message.data = malloc(sizeof(struct img_size));
	if (NULL == message.data) {
		CMR_LOGE("NO mem, Fail to alloc memory for msg data");
		return ARITH_NO_MEM;
	}

	message.alloc_flag = 1;
	memcpy(message.data, fd_size, sizeof(struct img_size));

	message.msg_type = ARITHMETIC_EVT_FD_INIT;
	return cmr_msg_post(s_arith_cxt->fd_msg_que_handle, &message, 1);
}

int arithmetic_fd_init(const struct img_size * fd_size, uint32_t is_support_fd)
{
	int                    ret = ARITH_SUCCESS;
	pthread_attr_t          attr;

	if (!is_support_fd) {
		CMR_LOGW("not support fd, direct return!");
		return ret;
	}

	CMR_LOGI("inited, %d", s_arith_cxt->fd_is_inited);

	if (0 == s_arith_cxt->fd_is_inited) {
		ret = cmr_msg_queue_create(CAMERA_FD_MSG_QUEUE_SIZE, &s_arith_cxt->fd_msg_que_handle);
		if (ret) {
			CMR_LOGE("NO Memory, Failed to create FD message queue");
		} else {
			sem_init(&s_arith_cxt->fd_sync_sem, 0, 0);
			pthread_mutex_init(&s_arith_cxt->fd_lock, NULL);
			pthread_attr_init(&attr);
			pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
			ret = pthread_create(&s_arith_cxt->fd_thread,  &attr, arithmetic_fd_thread_proc, NULL);
			sem_wait(&s_arith_cxt->fd_sync_sem);

			if (arithmetic_fd_call_init(fd_size)) {
				arithmetic_fd_deinit(1);
			} else {
				s_arith_cxt->fd_is_inited = 1;
			}
		}
	} else if ((s_arith_cxt->fd_size.width != fd_size->width) ||
			(s_arith_cxt->fd_size.height != fd_size->height)) {
		if (arithmetic_fd_call_init(fd_size)) {
				arithmetic_fd_deinit(1);
		}
	}

	return ret;
}

int arithmetic_fd_deinit(uint32_t is_support_fd)
{
	CMR_MSG_INIT(message);
	int                    ret = ARITH_SUCCESS;

	if (!is_support_fd) {
		CMR_LOGW("not support fd, direct return!");
		return ret;
	}

	CMR_LOGD("s.");
	CMR_PRINT_TIME;
	if (1 == s_arith_cxt->fd_is_inited) {
		message.msg_type = ARITHMETIC_EVT_FD_EXIT;
		ret = cmr_msg_post(s_arith_cxt->fd_msg_que_handle, &message, 1);

		if (CMR_MSG_SUCCESS == ret) {
			sem_wait(&s_arith_cxt->fd_sync_sem);
		}
		sem_destroy(&s_arith_cxt->fd_sync_sem);
		pthread_mutex_destroy(&s_arith_cxt->fd_lock);
		cmr_msg_queue_destroy(s_arith_cxt->fd_msg_que_handle);
		s_arith_cxt->fd_is_inited = 0;
		CMR_LOGD("FaceSolid_Finalize done.");
	}

	if (s_arith_cxt->addr) {
		free(s_arith_cxt->addr);
		s_arith_cxt->addr = NULL;
		s_arith_cxt->mem_size = 0;
		s_arith_cxt->phy_addr = NULL;
	}

	memset(s_arith_cxt, 0, sizeof(struct arithmetic_conext));

	CMR_PRINT_TIME;
	CMR_LOGD("e.");
	return ret;
}

int arithmetic_fd_start(void *data_addr)
{
	int ret = ARITH_SUCCESS;
	uint32_t phy_addr = 0;
	CMR_MSG_INIT(message);

	CMR_LOGD("0x%x.",(uint32_t)s_arith_cxt->addr);

	CMR_LOGD("%d,%d.",ret,s_arith_cxt->fd_busy);
	if (!ret && (1 != s_arith_cxt->fd_busy)) {
		message.msg_type = ARITHMETIC_EVT_FD_START;
		message.data = data_addr;
		ret = cmr_msg_post(s_arith_cxt->fd_msg_que_handle, &message, 1);

		if (CMR_MSG_SUCCESS == ret) {
			sem_wait(&s_arith_cxt->fd_sync_sem);
		} else {
			ret = ARITH_START_FAIL;
			CMR_LOGE("fail.");
		}
	} else {
		CMR_LOGD("discarded.");
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
	CMR_LOGD("0x%x,0x%x.",(uint32_t)s_arith_cxt->addr,vir_addr);
}

/*mem size = 0, means the memory should be free*/
int arithmetic_mem_handle(uint32_t mem_size)
{
	int ret = ARITH_SUCCESS;
	CMR_MSG_INIT(message);
	CMR_LOGI("E");

	if (s_arith_cxt->fd_is_inited) {
		message.msg_type = ARITHMETIC_EVT_MEM;
		message.data = malloc(sizeof(uint32_t));
		if (NULL == message.data) {
			CMR_LOGE("NO mem, Fail to alloc memory for msg data");
			return ARITH_NO_MEM;
		}
		message.alloc_flag = 1;
		*(uint32_t *)(message.data) = mem_size;;

		ret = cmr_msg_post(s_arith_cxt->fd_msg_que_handle, &message, 1);

		if (CMR_MSG_SUCCESS == ret) {
		} else {
			ret = ARITH_FAIL;
			CMR_LOGE("fail.");
		}
	} else {
		s_arith_cxt->mem_size = mem_size;
	}

	CMR_LOGI("X");
	return ret;
}

int arithmetic_hdr_init(uint32_t pic_width, uint32_t pic_height)
{
	int ret = ARITH_SUCCESS;
	uint32_t size = pic_width * pic_height * 3/2;

	CMR_LOGD("test log.");
	if (s_hdr_cxt->addr[0]) {
		CMR_LOGD("no need to init");
		return ret;
	}
	pthread_mutex_init(&s_arith_cxt->hdr_lock, NULL);
	s_hdr_cxt->inited = 1;

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
	return ret;
}

int arithmetic_hdr_deinit(void)
{
	int ret = ARITH_SUCCESS;
	CMR_LOGD("test log.");

	if (0 == s_hdr_cxt->inited) {
		CMR_LOGD("already deinit.");
		return ret;
	}
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
	s_hdr_cxt->inited = 0;
	pthread_mutex_unlock(&s_arith_cxt->hdr_lock);
	pthread_mutex_destroy(&s_arith_cxt->hdr_lock);

	CMR_LOGD("e.");
	return ret;
}
static void save_input_data(uint32_t width,uint32_t height)
{
	FILE *fp = NULL;
	uint32_t size = width*height*3/2;

	fp = fopen("/data/1.raw", "wb");
	if (0 != fp) {
		fwrite((void*)s_hdr_cxt->addr[0], 1, size, fp);
		fclose(fp);
	} else {
		CMR_LOGE("can not create savedata");
	}
	fp = fopen("/data/2.raw", "wb");
	if (0 != fp) {
		fwrite((void*)s_hdr_cxt->addr[1], 1, size, fp);
		fclose(fp);
	} else {
		CMR_LOGE("can not create savedata");
	}
	fp = fopen("/data/3.raw", "wb");
	if (0 != fp) {
		fwrite((void*)s_hdr_cxt->addr[2], 1, size, fp);
		fclose(fp);
	} else {
		CMR_LOGE("can not create savedata");
	}
}
static void save_hdrdata(void *addr,uint32_t width,uint32_t height)
{
	FILE *fp = NULL;
	uint32_t size = width*height*3/2;

	fp = fopen("/data/4.raw", "wb");
	if (0 != fp) {
		fwrite((void*)addr, 1, size, fp);
		fclose(fp);
	} else {
		CMR_LOGE("can not create savedata");
	}
}

int arithmetic_hdr(struct img_addr *dst_addr,uint32_t width,uint32_t height)
{
	int           ret = ARITH_SUCCESS;
	uint32_t      size = width*height;
	unsigned char *temp_addr0 = PNULL;
	unsigned char *temp_addr1 = PNULL;
	unsigned char *temp_addr2 = PNULL;
	char           *p_format = IMAGE_FORMAT;

	pthread_mutex_lock(&s_arith_cxt->hdr_lock);
	temp_addr0 = s_hdr_cxt->addr[0];
	temp_addr1 = s_hdr_cxt->addr[1];
	temp_addr2 = s_hdr_cxt->addr[2];
	CMR_LOGD("width %d,height %d.",width,height);
	/*save_input_data(width,height);*/

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
	if (NULL != temp_addr0) {
		memcpy((void *)dst_addr->addr_y,(void *)temp_addr0,size);
		memcpy((void *)dst_addr->addr_u,(void *)(temp_addr0+size),size/2);
	}
	/*save_hdrdata(dst_addr,width,height);*/
	pthread_mutex_unlock(&s_arith_cxt->hdr_lock);
	if (ARITH_SUCCESS == ret) {
		CMR_LOGD("hdr done.");
	}
	return ret;
}

void arithmetic_hdr_data(struct img_addr *addr,uint32_t y_size,uint32_t uv_size,uint32_t cap_cnt)
{
	unsigned char *uv_addr = PNULL;
	CMR_LOGD("0x%x,%d,%d.",(uint32_t)addr,y_size,cap_cnt);
	if (cap_cnt > HDR_CAP_NUM) {
		CMR_LOGE("cap cnt error,%d.",cap_cnt);
		return;
	}
	pthread_mutex_lock(&s_arith_cxt->hdr_lock);
	if (PNULL == s_hdr_cxt->addr[cap_cnt-1]) {
		CMR_LOGE("no memory.");
		pthread_mutex_unlock(&s_arith_cxt->hdr_lock);
		return;
	}

	if (s_hdr_cxt->mem_size >= (y_size+uv_size)) {
		memcpy((void *)s_hdr_cxt->addr[cap_cnt-1],(void *)addr->addr_y,y_size);
		uv_addr = s_hdr_cxt->addr[cap_cnt-1]+y_size;
		memcpy((void *)uv_addr,(void *)addr->addr_u,uv_size);
	} else {
		CMR_LOGW("mem size:0x%x,data size:0x%x.",s_hdr_cxt->mem_size,y_size);
	}
	pthread_mutex_unlock(&s_arith_cxt->hdr_lock);
}
