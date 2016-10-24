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
#include <semaphore.h>
#include <unistd.h>
#include "cmr_cvt.h"
#include "sprd_rot_k.h"

static char rot_dev_name[50] = "/dev/sprd_rotation";

struct rot_file{
	int fd;
	pthread_mutex_t status_lock;
};

static ROT_DATA_FORMAT_E cmr_rot_fmt_cvt(uint32_t cmr_fmt)
{
	ROT_DATA_FORMAT_E fmt = ROT_FMT_MAX;

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

int cmr_rot_open(void)
{
	int ret = -1;
	struct rot_file *file = NULL;
	int fd = -1;
	int handle = 0;

	file = malloc(sizeof(struct rot_file));
	if (!file) {
		goto open_out;
	}

	fd = open(rot_dev_name, O_RDWR, 0);
	if (fd < 0) {
		CMR_LOGE("Fail to open rotation device.");
		goto rot_free;
	}
	file->fd = fd;

	ret = pthread_mutex_init(&file->status_lock, NULL);
	if (ret) {
		CMR_LOGE("Failed to init status lock");
		goto rot_free;
	}
	handle = (int)file;
	goto open_out;
rot_free:
	if (file)
		free(file);
	file = NULL;
open_out:

	CMR_LOGI("handle=0x%x", handle);

	return handle;
}

int cmr_rot(struct cmr_rot_param *rot_param)
{
	struct _rot_cfg_tag rot_cfg;
	int ret = 0;
	enum img_rot_angle  angle;
	struct img_frm *src_img;
	struct img_frm *dst_img;
	int fd;
	struct rot_file *file = NULL;

	CMR_LOGI("S");

	if (!rot_param) {
		ret = -1;
		goto rot_exit;
	}

	/*check fd*/
	file = (struct rot_file*)rot_param->fd;
	if (!file) {
		ret = -1;
		goto rot_exit;
	}

	pthread_mutex_lock(&file->status_lock);

	fd = file->fd;
	if (fd < 0) {
		CMR_LOGE("Invalid fd");
		ret = -ENODEV;
		goto rot_unlock;
	}

	angle = rot_param->angle;
	src_img = rot_param->src_img;
	dst_img = rot_param->dst_img;

	if (NULL == src_img || NULL == dst_img) {
		CMR_LOGE("Wrong parameter 0x%x 0x%x", (uint32_t)src_img, (uint32_t)dst_img);
		ret = -EINVAL;
		goto rot_unlock;
	}

	CMR_LOGI("angle %d, src 0x%x 0x%x, w h %d %d, dst 0x%x 0x%x",
		angle,
		src_img->addr_phy.addr_y,
		src_img->addr_phy.addr_u,
		src_img->size.width,
		src_img->size.height,
		dst_img->addr_phy.addr_y,
		dst_img->addr_phy.addr_u);

	if ((uint32_t)angle < (uint32_t)(IMG_ROT_90)) {
		CMR_LOGE("Wrong angle %d", angle);
		ret = -EINVAL;
		goto rot_unlock;
	}

	rot_cfg.format = cmr_rot_fmt_cvt(src_img->fmt);
	if (rot_cfg.format >= ROT_FMT_MAX) {
		CMR_LOGE("Unsupported format %d, %d", src_img->fmt, rot_cfg.format);
		ret = -EINVAL;
		goto rot_unlock;
	}

	rot_cfg.angle = angle - IMG_ROT_90 + ROT_90;
	rot_cfg.src_addr.y_addr = src_img->addr_phy.addr_y;
	rot_cfg.src_addr.u_addr = src_img->addr_phy.addr_u;
	rot_cfg.src_addr.v_addr = src_img->addr_phy.addr_v;
	rot_cfg.dst_addr.y_addr = dst_img->addr_phy.addr_y;
	rot_cfg.dst_addr.u_addr = dst_img->addr_phy.addr_u;
	rot_cfg.dst_addr.v_addr = dst_img->addr_phy.addr_v;
	rot_cfg.img_size.w = (uint16_t)src_img->size.width;
	rot_cfg.img_size.h = (uint16_t)src_img->size.height;

	ret = ioctl(fd, ROT_IO_START, &rot_cfg);
	if (ret) {
		CMR_LOGE("Unsupported format %d, %d", src_img->fmt, rot_cfg.format);
		ret = -EINVAL;
		goto rot_unlock;
	}

rot_unlock:
	pthread_mutex_unlock(&file->status_lock);

rot_exit:

	CMR_LOGI("X ret=%d", ret);

	return ret;
}

int cmr_rot_close(int *fd)
{
	int ret = -1;
	struct rot_file *file = (struct rot_file*)(*fd);

	CMR_LOGI("Start to close rotation device.");

	if (!file)
		goto out;

	pthread_mutex_lock(&file->status_lock);
	if (file->fd < 0) {
		CMR_LOGE("Invalid fd");
		ret = -ENODEV;
		goto close_free;
	}

	/* then close fd */
	close(file->fd);

close_free:
	pthread_mutex_unlock(&file->status_lock);

	pthread_mutex_destroy(&file->status_lock);

	free(file);
	*fd = 0;
	ret = 0;
out:

	CMR_LOGI("ret=%d",ret);
	return ret;
}
