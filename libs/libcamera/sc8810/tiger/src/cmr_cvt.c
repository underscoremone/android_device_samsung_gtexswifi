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
#include <semaphore.h>
#include <unistd.h>
#include "img_scale_u.h"
#include "cmr_cvt.h"
#include "sprd_rot_k.h"

#define CVT_EXIT_IF_ERR(n)                                             \
		do {                                                   \
			if (n) {                                       \
				CMR_LOGE("ret %d", n);                 \
				goto exit;                             \
			}                                              \
		} while(0)


enum scale_work_mode
{
	SC_FRAME = SCALE_MODE_NORMAL,
	SC_SLICE_EXTERNAL = SCALE_MODE_SLICE,
	SC_SLICE_INTERNAL
};

struct scale_cxt {
	struct img_frm    src_frame;
	struct img_frm    dst_frame;
	struct img_rect   src_rect;
	uint32_t          slice_height;
	uint32_t          total_height;
	uint32_t          total_out_height;
	uint32_t          sc_threshold;
	uint32_t          sc_work_mode;
	void*             sc_user_data;
};


static char               rot_dev_name[50] = "/dev/sprd_rotation";
static int                rot_fd = -1;
static cmr_evt_cb         rot_evt_cb = NULL;
static pthread_mutex_t    rot_cb_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t          rot_thread;
static void*              rot_user_data;
static sem_t              rot_sem;

static char               scaler_dev_name[50] = "/dev/sprd_scale";
static int                scaler_fd = -1;
static cmr_evt_cb         scaler_evt_cb = NULL;
static pthread_mutex_t    scaler_cb_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t     scaler_cond;
static pthread_t          scaler_thread;
static struct scale_cxt   *sc_cxt;
static sem_t              scaler_sem;

static int   cmr_rot_create_thread(void);
static int   cmr_rot_kill_thread(void);
static void* cmr_rot_thread_proc(void* data);
static int   cmr_scale_create_thread(void);
static int   cmr_scale_kill_thread(void);
static void* cmr_scale_thread_proc(void* data);
static enum scale_fmt cmr_scale_fmt_cvt(uint32_t cmt_fmt);

static ROT_DATA_FORMAT_E cmr_rot_fmt_cvt(uint32_t cmr_fmt)
{
	ROT_DATA_FORMAT_E        fmt = ROT_FMT_MAX;

	switch (cmr_fmt) {
	case IMG_DATA_TYPE_YUV422:
		fmt = ROT_YUV422;
		break;
	case IMG_DATA_TYPE_YUV420:
		fmt = ROT_YUV420;
		break;
	case IMG_DATA_TYPE_RGB565:
		fmt = ROT_RGB565;
		break;
	case IMG_DATA_TYPE_RGB888:
		fmt = ROT_RGB888;
		break;
	default:
		break;
	}

	return fmt;
}

int cmr_rot_init(void)
{
	int                      ret = 0;

	rot_fd = open(rot_dev_name, O_RDWR, 0);
	
	if (-1 == rot_fd) {
		CMR_LOGE("Fail to open rotation device.");
		return -ENODEV;
   	} else {
		CMR_LOGV("OK to open rotation device.");
	}

	ret = pthread_mutex_init(&rot_cb_mutex, NULL);
	if (ret) {
		CMR_LOGE("Failed to init mutex : %d", ret);
		exit(EXIT_FAILURE);   
	}

	sem_init(&rot_sem, 0, 1);

	ret = cmr_rot_create_thread();
	rot_evt_cb = NULL;

	return ret;
}

int cmr_rot_evt_reg(cmr_evt_cb  rot_event_cb)
{
	pthread_mutex_lock(&rot_cb_mutex);
	rot_evt_cb = rot_event_cb;
	pthread_mutex_unlock(&rot_cb_mutex);
	return 0;
}

int cmr_rot(enum img_rot_angle  angle,
		struct img_frm  *src_img,
		struct img_rect *trim,
		struct img_frm  *dst_img,
		void            *user_data)
{
	struct _rot_cfg_tag      rot_cfg;
	int                      ret = 0;

	CMR_LOGV("angle %d, src 0x%x 0x%x, w h %d %d, dst 0x%x 0x%x",
		angle,
		src_img->addr_phy.addr_y,
		src_img->addr_phy.addr_u,
		src_img->size.width,
		src_img->size.height,
		dst_img->addr_phy.addr_y,
		dst_img->addr_phy.addr_u);

	if (-1 == rot_fd) {
		CMR_LOGE("Invalid fd");
		return -ENODEV;
	}

	if (NULL == src_img || NULL == dst_img) {
		CMR_LOGE("Wrong parameter 0x%x 0x%x", (uint32_t)src_img, (uint32_t)dst_img);
		return -EINVAL;
	}

	if ((uint32_t)angle < (uint32_t)(IMG_ROT_90)) {
		CMR_LOGE("Wrong angle %d", angle);
		return -EINVAL;
	}

	sem_wait(&rot_sem);

	rot_cfg.format          = cmr_rot_fmt_cvt(src_img->fmt);
	if (rot_cfg.format >= ROT_FMT_MAX) {
		CMR_LOGE("Unsupported format %d, %d", src_img->fmt, rot_cfg.format);
		return -EINVAL;
	}

	rot_cfg.angle = angle - IMG_ROT_90 + ROT_90;
	rot_cfg.src_addr.y_addr = src_img->addr_phy.addr_y;
	rot_cfg.src_addr.u_addr = src_img->addr_phy.addr_u;
	rot_cfg.src_addr.v_addr = src_img->addr_phy.addr_v;
	rot_cfg.dst_addr.y_addr = dst_img->addr_phy.addr_y;
	rot_cfg.dst_addr.u_addr = dst_img->addr_phy.addr_u;
	rot_cfg.dst_addr.v_addr = dst_img->addr_phy.addr_v;
	rot_cfg.img_size.w      = (uint16_t)src_img->size.width;
	rot_cfg.img_size.h      = (uint16_t)src_img->size.height;

	rot_user_data = user_data;

	ret = ioctl(rot_fd, ROT_IO_CFG, &rot_cfg);
	if (ret) {
		CMR_LOGE("Unsupported format %d, %d", src_img->fmt, rot_cfg.format);
		return -EINVAL;
	}

	ret = ioctl(rot_fd, ROT_IO_START, 1);


	return ret;
}

int cmr_rot_cpy(struct img_frm  *src_img,
			struct img_frm  *dst_img)
{
	struct _rot_cfg_tag      rot_cfg;
	int                      ret = 0;

	CMR_LOGV("Copy start");

	if (-1 == rot_fd) {
		CMR_LOGE("Invalid fd");
		return -ENODEV;
	}

	if (NULL == src_img || NULL == dst_img) {
		CMR_LOGE("Wrong parameter 0x%x 0x%x", (uint32_t)src_img, (uint32_t)dst_img);
		return -EINVAL;
	}

	rot_cfg.format = src_img->fmt;
	rot_cfg.img_size.w = src_img->size.width;
	rot_cfg.img_size.h = src_img->size.height;
	rot_cfg.src_addr.y_addr = src_img->addr_phy.addr_y;
	rot_cfg.src_addr.u_addr = src_img->addr_phy.addr_u;
	rot_cfg.src_addr.v_addr = 0;
	rot_cfg.dst_addr.y_addr = dst_img->addr_phy.addr_y;
	rot_cfg.dst_addr.u_addr = dst_img->addr_phy.addr_u;
	rot_cfg.dst_addr.v_addr = 0;

	//done
	ret = ioctl(rot_fd, ROT_IO_DATA_COPY, &rot_cfg);
	CMR_LOGV("Copy done");

	return ret;
}
int cmr_rot_deinit(void)
{
	int                      ret = 0;

	CMR_LOGV("Start to close rotation device.");
	
	if (-1 == rot_fd) {
		CMR_LOGE("Invalid fd");
		return -ENODEV;
	}

	sem_wait(&rot_sem);
	sem_post(&rot_sem);

	/* thread should be killed before fd deinited */
	ret = cmr_rot_kill_thread();
	if (ret) {
		CMR_LOGE("Failed to kill the thread. errno : %d", ret);
		exit(EXIT_FAILURE);   
	}

	sem_destroy(&rot_sem);

	/* then close fd */
	if (-1 == close(rot_fd)) {
		exit(EXIT_FAILURE);
	}
	rot_fd = -1;


	pthread_mutex_lock(&rot_cb_mutex);
	rot_evt_cb = NULL;
	pthread_mutex_unlock(&rot_cb_mutex);
	pthread_mutex_destroy(&rot_cb_mutex);
	CMR_LOGV("close device.");
	return 0;
}

static int   cmr_rot_create_thread(void)
{
	int                      ret = 0;
	pthread_attr_t           attr;

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	ret = pthread_create(&rot_thread, &attr, cmr_rot_thread_proc, NULL);
	pthread_attr_destroy(&attr);
	return ret;
}

static int cmr_rot_kill_thread(void)
{
	int                      ret = 0;
	char                     write_ch;
	void                     *dummy;

	if (-1 == rot_fd) {
		CMR_LOGE("invalid fd");
		return -ENODEV;
	}

	ret = write(rot_fd, &write_ch, 1); // kill thread;
	if (ret > 0) {
		ret = pthread_join(rot_thread, &dummy);
	}

	return ret;
}

static void* cmr_rot_thread_proc(void* data)
{
	int                      evt_id;
	struct img_frm           frame;
	uint32_t                 param;

	CMR_LOGV("rot_thread In");

	bzero(&frame, sizeof(frame));
	
	while(1) {
    		if (-1 == ioctl(rot_fd, ROT_IO_IS_DONE, &param)) {
			CMR_LOGV("To exit rot thread");
			break;
	    	} else {
			sem_post(&rot_sem);
		    	frame.reserved = rot_user_data;
		    	evt_id = CMR_IMG_CVT_ROT_DONE;
			pthread_mutex_lock(&rot_cb_mutex);
			if (rot_evt_cb) {
				(*rot_evt_cb)(evt_id, &frame);
			}
			pthread_mutex_unlock(&rot_cb_mutex);

		}
	}

	CMR_LOGV("rot_thread Out");
	return NULL;
}


int cmr_scale_init(void)
{
	int                      ret = 0;

	scaler_fd = open(scaler_dev_name, O_RDWR, 0);

	if (-1 == scaler_fd) {
		CMR_LOGE("Fail to open scaler device.");
		return -ENODEV;
	}

	ret = pthread_mutex_init(&scaler_cb_mutex, NULL);
	if (ret) {
		CMR_LOGE("Failed to init mutex : %d", ret);
		exit(EXIT_FAILURE);
	}

	pthread_cond_init(&scaler_cond, NULL);

	sem_init(&scaler_sem, 0, 1);

	ret = cmr_scale_create_thread();
	
	sc_cxt = malloc(sizeof(struct scale_cxt));
	if (NULL == sc_cxt) {
		CMR_LOGE("No memory!");
		return -1;
   	}

	scaler_evt_cb = NULL;

	return ret;
}

int cmr_scale_local_init(uint32_t slice_height, 
		struct img_frm *src_img,
		struct img_rect *rect,
		struct img_frm *dst_img,
		void           *user_data)
{
	int                      ret = 0;


	if (NULL == src_img || NULL == dst_img) {
		CMR_LOGE("Parameter Error 0x%x 0x%x", (uint32_t)src_img, (uint32_t)dst_img);
		return -EINVAL;
	}

	if (-1 == scaler_fd || NULL == sc_cxt) {
		CMR_LOGE("Scale device has not been opened yet! 0x%X 0x%x ",
			scaler_fd,
			(uint32_t)sc_cxt);
		return -1;
	}

	bzero(sc_cxt, sizeof(struct scale_cxt));
	read(scaler_fd, &sc_cxt->sc_threshold, sizeof(uint32_t));

	memcpy(&sc_cxt->src_frame, src_img, sizeof(struct img_frm));
	memcpy(&sc_cxt->dst_frame, dst_img, sizeof(struct img_frm));

	if (NULL == rect) {
		sc_cxt->src_rect.start_x = 0;
		sc_cxt->src_rect.start_y = 0;
		sc_cxt->src_rect.width   = src_img->size.width;
		sc_cxt->src_rect.height  = src_img->size.height;
	} else {
		memcpy(&sc_cxt->src_rect, rect, sizeof(struct img_rect));
	}

	if (slice_height >= sc_cxt->src_rect.height) {
		if (sc_cxt->dst_frame.size.width > sc_cxt->sc_threshold) {
			sc_cxt->sc_work_mode = SC_SLICE_INTERNAL;
			sc_cxt->slice_height = CMR_SLICE_HEIGHT;
		} else {
			sc_cxt->sc_work_mode = SC_FRAME;
			sc_cxt->slice_height = slice_height;

		}
	} else {
		sc_cxt->sc_work_mode = SC_SLICE_EXTERNAL;
		sc_cxt->slice_height = slice_height;
	}
	sc_cxt->sc_user_data = user_data;
	sc_cxt->total_height = sc_cxt->src_rect.start_y;
	return 0;
}
int cmr_scale_evt_reg(cmr_evt_cb  scale_event_cb)
{
	pthread_mutex_lock(&scaler_cb_mutex);
	scaler_evt_cb = scale_event_cb;
	pthread_mutex_unlock(&scaler_cb_mutex);
	return 0;

}

int  cmr_scale_start(uint32_t slice_height,
		struct img_frm *src_img,
		struct img_rect *rect,
		struct img_frm *dst_img,
		void           *tmp_buf,
		void           *user_data)
{
	int                      ret = 0;
	enum scle_mode           sc_mode;
	enum scale_fmt           fmt;
	struct img_addr          tmp_addr;
	struct scale_frame       sc_frm;

	if (-1 == scaler_fd) {
		CMR_LOGE("Fail to open scaler device.");
		return -ENODEV;
	}

	CMR_LOGI("src, w h %d %d, addr 0x%x 0x%x, fmt %d, endian %d %d",
		src_img->size.width,
		src_img->size.height,
		src_img->addr_phy.addr_y,
		src_img->addr_phy.addr_u,
		src_img->fmt,
		src_img->data_end.y_endian,
		src_img->data_end.uv_endian);

	CMR_LOGI("dst, w h %d %d, addr 0x%x 0x%x, fmt %d, endian %d %d",
		dst_img->size.width,
		dst_img->size.height,
		dst_img->addr_phy.addr_y,
		dst_img->addr_phy.addr_u,
		dst_img->fmt,
		dst_img->data_end.y_endian,
		dst_img->data_end.uv_endian);

	sem_wait(&scaler_sem);

	ret = cmr_scale_local_init(slice_height,
				src_img,
				rect,
				dst_img,
				user_data);
	CVT_EXIT_IF_ERR(ret);
	
	ret = ioctl(scaler_fd, SCALE_IO_INPUT_SIZE, &src_img->size);
	CVT_EXIT_IF_ERR(ret);

	ret = ioctl(scaler_fd, SCALE_IO_INPUT_RECT, &sc_cxt->src_rect);
	CVT_EXIT_IF_ERR(ret);

	fmt = cmr_scale_fmt_cvt(src_img->fmt);
	ret = ioctl(scaler_fd, SCALE_IO_INPUT_FORMAT, &fmt);
	CVT_EXIT_IF_ERR(ret);

	ret = ioctl(scaler_fd, SCALE_IO_INPUT_ENDIAN, &sc_cxt->src_frame.data_end);
	CVT_EXIT_IF_ERR(ret);

	ret = ioctl(scaler_fd, SCALE_IO_INPUT_ADDR, &src_img->addr_phy);
	CVT_EXIT_IF_ERR(ret);

	ret = ioctl(scaler_fd, SCALE_IO_OUTPUT_SIZE, &dst_img->size);
	CVT_EXIT_IF_ERR(ret);

	fmt = cmr_scale_fmt_cvt(dst_img->fmt);
	ret = ioctl(scaler_fd, SCALE_IO_OUTPUT_FORMAT, &fmt);
	CVT_EXIT_IF_ERR(ret);

	ret = ioctl(scaler_fd, SCALE_IO_OUTPUT_ENDIAN, &sc_cxt->dst_frame.data_end);
	CVT_EXIT_IF_ERR(ret);

	ret = ioctl(scaler_fd, SCALE_IO_OUTPUT_ADDR, &dst_img->addr_phy);
	CVT_EXIT_IF_ERR(ret);

	if (SC_FRAME == sc_cxt->sc_work_mode) {
		sc_mode = SCALE_MODE_NORMAL;
		ret = ioctl(scaler_fd, SCALE_IO_SCALE_MODE, &sc_mode);
	} else {
		sc_mode = SCALE_MODE_SLICE;
		ret = ioctl(scaler_fd, SCALE_IO_SCALE_MODE, &sc_mode);
		CVT_EXIT_IF_ERR(ret);
		if (tmp_buf) {
			tmp_addr.addr_y = (uint32_t)tmp_buf;
			tmp_addr.addr_u = tmp_addr.addr_y + (uint32_t)(sc_cxt->slice_height * dst_img->size.width);
			tmp_addr.addr_v = tmp_addr.addr_u + (uint32_t)(sc_cxt->slice_height * dst_img->size.width);
			ret = ioctl(scaler_fd, SCALE_IO_TEMP_BUFF, &tmp_addr);
			CVT_EXIT_IF_ERR(ret);
		}
		ret = ioctl(scaler_fd, SCALE_IO_SLICE_SCALE_HEIGHT, &sc_cxt->slice_height);
	}
	CVT_EXIT_IF_ERR(ret);

	ret = ioctl(scaler_fd, SCALE_IO_START, NULL);
	pthread_mutex_lock(&scaler_cb_mutex);
	if (NULL == scaler_evt_cb) {
		pthread_cond_wait(&scaler_cond, &scaler_cb_mutex);
	}
	pthread_mutex_unlock(&scaler_cb_mutex);
exit:
	return ret;
}

int  cmr_scale_next(uint32_t     slice_height, 
		struct img_addr  *src_addr, 
		struct img_rect  *rect,
		struct img_addr  *dst_addr)
{
	int                      ret = 0;
	struct img_rect          l_rect;
	struct img_addr          l_addr;
	uint32_t                 offset;
	uint32_t                 offset_fact = 0;

	CMR_LOGV("do next slice");

	if (-1 == scaler_fd) {
		CMR_LOGE("Fail to open scaler device.");
		return -ENODEV;
	}

	if (sc_cxt->total_out_height >= sc_cxt->dst_frame.size.height) {
		CMR_LOGE("Slice scaling has been finished");
		return 0;
	}

	if (IMG_DATA_TYPE_YUV420 == sc_cxt->dst_frame.fmt) {
		offset_fact += 1;
	}

	if (0 == slice_height &&
		NULL == src_addr &&
		NULL == rect) {

		CMR_LOGV("auto slice scaling");
		l_rect.start_x = sc_cxt->src_rect.start_x;
		l_rect.start_y = 0;
		l_rect.width   = sc_cxt->src_frame.size.width;
		l_rect.height  = sc_cxt->src_frame.size.height;
		ret = ioctl(scaler_fd, SCALE_IO_INPUT_RECT, &l_rect);
		CVT_EXIT_IF_ERR(ret);

		offset = (uint32_t)(sc_cxt->total_height * sc_cxt->src_frame.size.width);
		l_addr.addr_y = sc_cxt->src_frame.addr_phy.addr_y + offset;
		l_addr.addr_u = sc_cxt->src_frame.addr_phy.addr_u + (offset >> 1);
		ret = ioctl(scaler_fd, SCALE_IO_INPUT_ADDR, &l_addr);
		CVT_EXIT_IF_ERR(ret);

	} else {

		CMR_LOGV("Not auto slice scaling");

		if (src_addr) {
			ret = ioctl(scaler_fd, SCALE_IO_INPUT_ADDR, src_addr);
			CVT_EXIT_IF_ERR(ret);
		}

		if (rect) {
			ret = ioctl(scaler_fd, SCALE_IO_INPUT_RECT, rect);
			CVT_EXIT_IF_ERR(ret);
		}

		if (slice_height) {
			if (slice_height + sc_cxt->total_height > sc_cxt->src_rect.height) {
				slice_height = sc_cxt->src_rect.height - sc_cxt->total_height;
			}
			ret = ioctl(scaler_fd, SCALE_IO_SLICE_SCALE_HEIGHT, slice_height);
			CVT_EXIT_IF_ERR(ret);
		}
	}

	if (dst_addr) {
		ret = ioctl(scaler_fd, SCALE_IO_OUTPUT_ADDR, dst_addr);
		CVT_EXIT_IF_ERR(ret);
	} else {
		offset = (uint32_t)(sc_cxt->dst_frame.size.width * sc_cxt->total_out_height);
		l_addr.addr_y = sc_cxt->dst_frame.addr_phy.addr_y + offset;
		l_addr.addr_u = sc_cxt->dst_frame.addr_phy.addr_u + (offset >> offset_fact);
		ret = ioctl(scaler_fd, SCALE_IO_OUTPUT_ADDR, &l_addr);
		CVT_EXIT_IF_ERR(ret);
	}

	ret = ioctl(scaler_fd, SCALE_IO_CONTINUE, NULL);

exit:
	return ret;
}

int cmr_scale_capability(uint32_t *width)
{
	int                      ret = 0;

	if (-1 == scaler_fd) {
		CMR_LOGE("Fail to open scaler device.");
		return -ENODEV;
	}

	ret = read(scaler_fd, width, sizeof(uint32_t));
	CMR_LOGV("width %d", *width);
	return ret;
}

int cmr_scale_deinit(void)
{
	int                      ret = 0;

	CMR_LOGV("Start to close scale device.");
	
	if (-1 == scaler_fd) {
		CMR_LOGE("Fail to open scaler device.");
		return -ENODEV;
	}

	ioctl(scaler_fd, SCALE_IO_STOP, NULL);

	pthread_mutex_lock(&scaler_cb_mutex);
	if (NULL == scaler_evt_cb) {
		pthread_cond_signal(&scaler_cond);
	}
	pthread_mutex_unlock(&scaler_cb_mutex);

	/* thread should be killed before fd deinited */
	ret = cmr_scale_kill_thread();
	if (ret) {
		CMR_LOGE("Failed to kill the thread. errno : %d", ret);
		exit(EXIT_FAILURE);   
	}

	sem_destroy(&scaler_sem);

	/* then close fd */
	if (-1 == close(scaler_fd)) {
		exit(EXIT_FAILURE);   
	}
	scaler_fd = -1;

	if (sc_cxt) {
		free(sc_cxt);
		sc_cxt = NULL;
	}

	pthread_mutex_lock(&scaler_cb_mutex);
	scaler_evt_cb = NULL;
	pthread_mutex_unlock(&scaler_cb_mutex);
	pthread_mutex_destroy(&scaler_cb_mutex);
	pthread_cond_destroy(&scaler_cond);
	CMR_LOGV("close device.");
	return 0;
}

static int   cmr_scale_create_thread(void)
{
	int                      ret = 0;
	pthread_attr_t           attr;

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	ret = pthread_create(&scaler_thread, &attr, cmr_scale_thread_proc, NULL);
	pthread_attr_destroy(&attr);
	return ret;
}

static int cmr_scale_kill_thread(void)
{
	int                      ret = 0;
	char                     write_ch;
	void                     *dummy;	

	if (-1 == scaler_fd) {
		CMR_LOGE("invalid fd");
		return -1;
	}

	ret = write(scaler_fd, &write_ch, 1); // kill thread;
	if (ret > 0) {
		ret = pthread_join(scaler_thread, &dummy);
	}

	return ret;
}

static void* cmr_scale_thread_proc(void* data)
{
	int                      evt_id;
	struct img_frm           frame;
	struct scale_frame       sc_frm;
	
	CMR_LOGV("scaler_thread In");

	bzero(&frame, sizeof(frame));

	while(1) {
    		if (-1 == ioctl(scaler_fd, SCALE_IO_IS_DONE, &sc_frm)) {
			CMR_LOGV("To exit scaler thread");
			break;
	    	} else {
			sem_post(&scaler_sem);
			pthread_mutex_lock(&scaler_cb_mutex);
			if (NULL == scaler_evt_cb) {
				pthread_cond_signal(&scaler_cond);
			} else {
				evt_id = CMR_IMG_CVT_SC_DONE; 
			    	frame.reserved        = sc_cxt->sc_user_data;
				frame.size.width      = sc_frm.width;
				frame.size.height     = sc_frm.height;
				frame.addr_phy.addr_y = sc_frm.yaddr;
				frame.addr_phy.addr_u = sc_frm.uaddr;
				frame.addr_phy.addr_v = sc_frm.vaddr;
				sc_cxt->total_out_height += sc_frm.height;
				sc_cxt->total_height += sc_cxt->slice_height;
				if (scaler_evt_cb) {
					(*scaler_evt_cb)(evt_id, &frame);
				}
			}
			pthread_mutex_unlock(&scaler_cb_mutex);
		}
	}

	CMR_LOGV("scaler_thread Out");
	return NULL;
}

static enum scale_fmt cmr_scale_fmt_cvt(uint32_t cmt_fmt)
{
	enum scale_fmt           sc_fmt = SCALE_FTM_MAX;
	
	switch (cmt_fmt) {
	case IMG_DATA_TYPE_YUV422:
		sc_fmt = SCALE_YUV422;
		break;
	case IMG_DATA_TYPE_YUV420:
		sc_fmt = SCALE_YUV420;
		break;
	case IMG_DATA_TYPE_RGB565:
		sc_fmt = SCALE_RGB565;
		break;
	case IMG_DATA_TYPE_RGB888:
		sc_fmt = SCALE_RGB888;
		break;
	default:
		CMR_LOGE("Unsupported camera format");
		break;
	}

	return sc_fmt;
}

