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
#include <linux/videodev2.h>
#include <pthread.h>
#include <unistd.h>
#include "cmr_v4l2.h"

#define CMR_TIMING_LEN                   16

#define CMR_CHECK_FD                                                   \
		do {                                                   \
			if (-1 == fd) {                                \
				CMR_LOGE("V4L2 device not opened");    \
				return -1;                             \
			}                                              \
		} while(0)

enum
{
	V4L2_FLAG_TX_DONE  = 0x00,
	V4L2_FLAG_NO_MEM   = 0x01,
	V4L2_FLAG_TX_ERR   = 0x02,
	V4L2_FLAG_CSI2_ERR = 0x03,
	V4L2_FLAG_SYS_BUSY = 0x04,
	V4L2_FLAG_TX_STOP  = 0xFF
};

enum
{
	NO_EXIT = 0,
	EXIT_BY_HOST,
	EXIT_EXCEPTION,
};

static char               dev_name[50] = "/dev/video0";
static int                fd = -1;
static cmr_evt_cb         v4l2_evt_cb = NULL;
static pthread_mutex_t    cb_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t    status_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint32_t           is_on = 0;
static pthread_t          v4l2_thread;

static int      cmr_v4l2_create_thread(void);
static int      cmr_v4l2_kill_thread(void);
static void*    cmr_v4l2_thread_proc(void* data);
static uint32_t cmr_v4l2_get_4cc(uint32_t img_type);

int cmr_v4l2_init(void)
{
	int                      ret = 0;

	CMR_LOGV("Start to open V4L2 device.");
	fd = open(dev_name, O_RDWR, 0);
	if (-1 == fd) {
		CMR_LOGE("Failed to open dcam device.errno : %d", errno);
		fprintf(stderr, "Cannot open '%s': %d, %s\n", dev_name, errno,  strerror(errno));
		exit(EXIT_FAILURE);   
	} else {
		CMR_LOGV("OK to open device.");
	}

	ret = pthread_mutex_init(&cb_mutex, NULL);
	if (ret) {
		CMR_LOGE("Failed to init mutex : %d", errno);
		exit(EXIT_FAILURE);   
	}

	ret = pthread_mutex_init(&status_mutex, NULL);
	if (ret) {
		CMR_LOGE("Failed to init status mutex : %d", errno);
		exit(EXIT_FAILURE);
	}

	ret = cmr_v4l2_create_thread();
	v4l2_evt_cb = NULL;

	return ret;
}

int cmr_v4l2_deinit(void)
{
	int                      ret = 0;

	CMR_LOGV("Start to close V4L2 device.");

	/* thread should be killed before fd deinited */
	ret = cmr_v4l2_kill_thread();
	if (ret) {
		CMR_LOGE("Failed to kill the thread. errno : %d", errno);
		exit(EXIT_FAILURE);
	}

	/* then close fd */
	if (-1 != fd) {
		if (-1 == close(fd)) {
			fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
			exit(EXIT_FAILURE);
		}
		fd = -1;
	}
	CMR_LOGI("thread kill done.");
	pthread_mutex_lock(&cb_mutex);
	v4l2_evt_cb = NULL;
	pthread_mutex_unlock(&cb_mutex);
	pthread_mutex_destroy(&cb_mutex);
	pthread_mutex_destroy(&status_mutex);
	CMR_LOGV("close device.\n");
	return 0;
}

void cmr_v4l2_evt_reg(cmr_evt_cb  v4l2_event_cb)
{
	pthread_mutex_lock(&cb_mutex);
	v4l2_evt_cb = v4l2_event_cb;
	pthread_mutex_unlock(&cb_mutex);
	return;
}


/*

For DV Timing parameter

timing_param[0]       img_fmt
timing_param[1]       img_ptn
timing_param[2]       res
timing_param[3]       deci_factor

if(CCIR)
timing_param[4]       ccir.v_sync_pol
timing_param[5]       ccir.h_sync_pol
timing_param[6]       ccir.pclk_pol
if(MIPI)
timing_param[4]       mipi.use_href
timing_param[5]       mipi.bits_per_pxl
timing_param[6]       mipi.is_loose
timing_param[7]       mipi.lane_num

timing_param[7]       width
timing_param[8]       height

*/
int cmr_v4l2_if_cfg(struct sensor_if *sn_if)
{
	int                      ret = 0;
	struct v4l2_control      ctrl;
	uint32_t                 timing_param[CMR_TIMING_LEN];

	CMR_CHECK_FD;

	timing_param[CMR_TIMING_LEN-1] = sn_if->if_type;
	timing_param[0] = sn_if->img_fmt;
	timing_param[1] = sn_if->img_ptn;
	timing_param[3] = sn_if->frm_deci;
	if (0 == sn_if->if_type) {
		/* CCIR interface */
		timing_param[4] = sn_if->if_spec.ccir.v_sync_pol;
		timing_param[5] = sn_if->if_spec.ccir.h_sync_pol;
		timing_param[6] = sn_if->if_spec.ccir.pclk_pol;
	} else {
		/* MIPI interface */
		timing_param[4] = sn_if->if_spec.mipi.use_href;
		timing_param[5] = sn_if->if_spec.mipi.bits_per_pxl;
		timing_param[6] = sn_if->if_spec.mipi.is_loose;
		timing_param[7] = sn_if->if_spec.mipi.lane_num;

	}

	ctrl.id = V4L2_CID_USER_CLASS;
	ctrl.value = (int32_t)&timing_param[0];

	ret = ioctl(fd, VIDIOC_S_CTRL, &ctrl);

	CMR_LOGV("Set dv timing, ret %d, if type %d, mode %d, deci %d",
		ret,
		timing_param[CMR_TIMING_LEN-1],
		timing_param[0],
		timing_param[3]);

	return ret;
}

int cmr_v4l2_cap_cfg(struct cap_cfg *cfg)
{
	int                      ret = 0;
	struct v4l2_streamparm   stream_parm;
	struct v4l2_crop         crop;
	struct v4l2_fmtdesc      fmtdesc;
	struct v4l2_format       format;
	uint32_t                 found = 0;
	uint32_t                 ret_channel_num = 0;
	uint32_t                 pxl_fmt;
	
	CMR_LOGV("channle number %d , frame number %d", cfg->channel_num, cfg->frm_num);
	
	CMR_CHECK_FD;

	if (NULL == cfg || 0 == cfg->channel_num || 0 == cfg->frm_num)
		return -1;

	/* firstly, set capture mode as single capture or multi-capture */
	stream_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = ioctl(fd, VIDIOC_G_PARM, &stream_parm);
	CMR_RTN_IF_ERR(ret);

	if (1 == cfg->frm_num) {
		stream_parm.parm.capture.capturemode = 0;
	} else {
		stream_parm.parm.capture.capturemode = 1;
	}
	stream_parm.parm.capture.reserved[2] = cfg->sn_size.width;
	stream_parm.parm.capture.reserved[3] = cfg->sn_size.height;
	ret = ioctl(fd, VIDIOC_S_PARM, &stream_parm);
	CMR_RTN_IF_ERR(ret);

	/* secondly, set the crop rectangle PATH1 module, this should be called before VIDIOC_TRY_FMT called */
	crop.c.left   = cfg->cfg0.src_img_rect.start_x;
	crop.c.top    = cfg->cfg0.src_img_rect.start_y;
	crop.c.width  = cfg->cfg0.src_img_rect.width;
	crop.c.height = cfg->cfg0.src_img_rect.height;
	crop.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = ioctl(fd, VIDIOC_S_CROP, &crop);
	CMR_RTN_IF_ERR(ret);

	/* thirdly,  check whether the output format described by cfg->cfg0 can be supported by the low layer */
	found = 0;
	fmtdesc.index = 0;
	fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	pxl_fmt = cmr_v4l2_get_4cc(cfg->cfg0.dst_img_fmt);
	while (0 == ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc)) {
		if (fmtdesc.pixelformat == pxl_fmt) {
			CMR_LOGV("FourCC 0x%x is supported by the low layer", pxl_fmt);
			found = 1;
			break;
		}
		fmtdesc.index++;
	}

	if (found) {
		bzero(&format, sizeof(struct v4l2_format));
		format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		format.fmt.pix.width = cfg->cfg0.dst_img_size.width;
		format.fmt.pix.height = cfg->cfg0.dst_img_size.height;
		format.fmt.pix.pixelformat = pxl_fmt; //fourecc
		format.fmt.pix.priv = cfg->cfg0.need_isp;
		ret = ioctl(fd, VIDIOC_TRY_FMT, &format);
		CMR_LOGV("need binning, %d", format.fmt.pix.sizeimage);
		if (format.fmt.pix.sizeimage) {
			cfg->cfg0.need_binning = 1;
		}
		CMR_RTN_IF_ERR(ret);
		ret_channel_num++;
	}

	/* lastly,  check whether the output format described by cfg->cfg1 can be supported by the low layer */
	if (cfg->channel_num == 2) {
		crop.c.left   = cfg->cfg1.src_img_rect.start_x;
		crop.c.top    = cfg->cfg1.src_img_rect.start_y;
		crop.c.width  = cfg->cfg1.src_img_rect.width;
		crop.c.height = cfg->cfg1.src_img_rect.height;
		crop.type     = V4L2_BUF_TYPE_PRIVATE;
		ret = ioctl(fd, VIDIOC_S_CROP, &crop);
		CMR_RTN_IF_ERR(ret);

		found = 0;
		fmtdesc.index = 0;
		fmtdesc.type = V4L2_BUF_TYPE_PRIVATE;
		pxl_fmt = cmr_v4l2_get_4cc(cfg->cfg1.dst_img_fmt);
		while (0 == ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc)) {
			if (fmtdesc.pixelformat == pxl_fmt) {
				CMR_LOGV("FourCC 0x%x is supported by the low layer", pxl_fmt);
				found = 1;
				break;
			}
			fmtdesc.index++;
		}

		if (found) {
			bzero(&format, sizeof(struct v4l2_format));
			format.type = V4L2_BUF_TYPE_PRIVATE;
			format.fmt.pix.width = cfg->cfg1.dst_img_size.width;
			format.fmt.pix.height = cfg->cfg1.dst_img_size.height;
			format.fmt.pix.pixelformat = pxl_fmt; //fourecc
			CMR_LOGI("type, w, h, pixelformat, 0x%x %d %d 0x%x",
				format.type, format.fmt.pix.width,
				format.fmt.pix.height, format.fmt.pix.pixelformat);

			ret = ioctl(fd, VIDIOC_TRY_FMT, &format);
			if (0 == ret) {
				ret_channel_num++;
				cfg->cfg1.need_isp = format.fmt.pix.priv;
			} else {
				CMR_LOGV("Failed, %d", ret);
			}
		}

	}

exit:

	if (ret_channel_num) {
		ret = 0;
		cfg->channel_num = ret_channel_num;
	}

	CMR_LOGV("ret %d channle number %d", ret, cfg->channel_num);

	return ret;
}

int cmr_v4l2_buff_cfg (struct buffer_cfg *buf_cfg)
{
	int                      ret = 0;
	uint32_t                 i;
	struct v4l2_buffer       v4l2_buf;
	struct v4l2_streamparm   stream_parm;

	CMR_CHECK_FD;

	CMR_LOGV("%d %d 0x%x ", buf_cfg->channel_id, buf_cfg->count, buf_cfg->base_id);
	
	if (NULL == buf_cfg || buf_cfg->count > V4L2_BUF_MAX)
		return -1;


	if (0 == buf_cfg->channel_id) {
		v4l2_buf.type  = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		stream_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	} else {
		v4l2_buf.type  = V4L2_BUF_TYPE_PRIVATE;
		stream_parm.type = V4L2_BUF_TYPE_PRIVATE;
	}

	/* firstly , set the base index for each channel */
	ret = ioctl(fd, VIDIOC_G_PARM, &stream_parm);
	CMR_RTN_IF_ERR(ret);
	stream_parm.parm.capture.reserved[1] = buf_cfg->base_id;
	ret = ioctl(fd, VIDIOC_S_PARM, &stream_parm);
	CMR_RTN_IF_ERR(ret);

/*	CMR_LOGI("wjp:test %d,0x%x.",buf_cfg->count,buf_cfg->addr[0].addr_y);*/

	/* secondly , set the frame address */
	for (i = 0; i < buf_cfg->count; i++) {
		v4l2_buf.m.userptr  = buf_cfg->addr[i].addr_y;
		v4l2_buf.input      = buf_cfg->addr[i].addr_u;
		v4l2_buf.reserved   = buf_cfg->addr[i].addr_v;
		ret = ioctl(fd, VIDIOC_QBUF, &v4l2_buf);
		if (ret) {
			CMR_LOGE("Failed to QBuf, %d", ret);
			break;
		}
	}

exit:
	return ret;

}

int cmr_v4l2_cap_start(uint32_t skip_num)
{
	int                      ret = 0;
	struct v4l2_streamparm   stream_parm;
	enum v4l2_buf_type       buf_type;

	CMR_CHECK_FD;

	stream_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = ioctl(fd, VIDIOC_G_PARM, &stream_parm);
	CMR_RTN_IF_ERR(ret);
	stream_parm.parm.capture.reserved[0] = skip_num; /* just modify the skip number parameter */
	stream_parm.parm.capture.extendedmode = 0; /* set normal stream on/off mode */
	ret = ioctl(fd, VIDIOC_S_PARM, &stream_parm);
	CMR_RTN_IF_ERR(ret);

	ret = ioctl(fd, VIDIOC_STREAMON, &buf_type);
	if (0 == ret) {
		pthread_mutex_lock(&status_mutex);
		is_on = 1;
		pthread_mutex_unlock(&status_mutex);
	}

exit:
	CMR_LOGV("ret = %d.",ret);
	return ret;
}

int cmr_v4l2_cap_stop(void)
{
	int                      ret = 0;
	struct v4l2_streamparm   stream_parm;
	enum v4l2_buf_type       buf_type;

	CMR_CHECK_FD;
	pthread_mutex_lock(&status_mutex);
	is_on = 0;
	pthread_mutex_unlock(&status_mutex);

	stream_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = ioctl(fd, VIDIOC_G_PARM, &stream_parm);
	CMR_RTN_IF_ERR(ret);
	stream_parm.parm.capture.extendedmode = 0; /* set normal stream on/off mode */
	ret = ioctl(fd, VIDIOC_S_PARM, &stream_parm);
	CMR_RTN_IF_ERR(ret);

	ret = ioctl(fd, VIDIOC_STREAMOFF, &buf_type);
	CMR_LOGV("streamoff done.");
exit:

	return ret;
}

int cmr_v4l2_cap_resume(uint32_t skip_number)
{
	int                      ret = 0;
	struct v4l2_streamparm   stream_parm;
	enum v4l2_buf_type       buf_type;

	CMR_CHECK_FD;

	stream_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = ioctl(fd, VIDIOC_G_PARM, &stream_parm);
	CMR_RTN_IF_ERR(ret);
	stream_parm.parm.capture.reserved[0] = skip_number; /* only just modify the skip number parameter */
	stream_parm.parm.capture.extendedmode = 1; /* set lightly stream on/off mode */
	ret = ioctl(fd, VIDIOC_S_PARM, &stream_parm);
	CMR_RTN_IF_ERR(ret);

	ret = ioctl(fd, VIDIOC_STREAMON, &buf_type);

exit:

	return ret;
}

int cmr_v4l2_cap_pause(void)
{
	int                      ret = 0;
	struct v4l2_streamparm   stream_parm;
	enum v4l2_buf_type       buf_type;

	CMR_CHECK_FD;

	stream_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = ioctl(fd, VIDIOC_G_PARM, &stream_parm);
	CMR_RTN_IF_ERR(ret);

	stream_parm.parm.capture.extendedmode = 1; /* set lightly stream on/off mode */
	ret = ioctl(fd, VIDIOC_S_PARM, &stream_parm);
	CMR_RTN_IF_ERR(ret);

	ret = ioctl(fd, VIDIOC_STREAMOFF, &buf_type);

exit:

	return ret;
}

int cmr_v4l2_free_frame(uint32_t channel_num, uint32_t index)
{
	int                      ret = 0;
	struct v4l2_buffer       v4l2_buf;

	CMR_LOGV("channel %d, index 0x%x", channel_num, index);

	CMR_CHECK_FD;

	pthread_mutex_lock(&status_mutex);
	if (0 == is_on) {
		pthread_mutex_unlock(&status_mutex);
		return ret;
	}
	pthread_mutex_unlock(&status_mutex);

	v4l2_buf.index = index;
	if (0 == channel_num) {
		v4l2_buf.type  = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	} else {
		v4l2_buf.type  = V4L2_BUF_TYPE_PRIVATE;
	}
	ret = ioctl(fd, VIDIOC_QBUF, &v4l2_buf);

	return ret;
}

int cmr_v4l2_scale_capability(uint32_t *width, uint32_t *sc_factor)
{
	uint32_t                 rd_word[2];
	int                      ret = 0;

	if (NULL == width || NULL == sc_factor) {
		CMR_LOGE("Wrong param, 0x%x 0x%x", (uint32_t)width, (uint32_t)sc_factor);
		return -ENODEV;
	}
	CMR_CHECK_FD;

	ret = read(fd, rd_word, 2*sizeof(uint32_t));
	*width = rd_word[0];
	*sc_factor = rd_word[1];
	CMR_LOGV("width %d, sc_factor %d", *width, *sc_factor);
	return ret;
}


static int cmr_v4l2_evt_id(int isr_flag)
{
	int                      ret = CMR_V4L2_MAX;

	switch (isr_flag) {
	case V4L2_FLAG_TX_DONE:
		ret = CMR_V4L2_TX_DONE;
		break;
	case V4L2_FLAG_NO_MEM:
		ret = CMR_V4L2_TX_NO_MEM;
		break;
	case V4L2_FLAG_TX_ERR:
		ret = CMR_V4L2_TX_ERROR;
		break;
	case V4L2_FLAG_CSI2_ERR:
		ret = CMR_V4L2_CSI2_ERR;
		break;
	default:
		CMR_LOGV("isr_flag 0x%x", isr_flag);
		break;
	}

	return ret;
}

static int   cmr_v4l2_create_thread(void)
{
	int                      ret = 0;
	pthread_attr_t           attr;

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	ret = pthread_create(&v4l2_thread, &attr, cmr_v4l2_thread_proc, NULL);
	pthread_attr_destroy(&attr);
	return ret;
}

static int cmr_v4l2_kill_thread(void)
{
	int                      ret = 0;
	char                     write_ch;
	void                     *dummy;

	CMR_CHECK_FD;

	CMR_LOGV("Call write function to kill v4l2 manage thread");

	ret = write(fd, &write_ch, 1); // kill thread;
	if (ret > 0) {
		CMR_LOGV("write OK!");
		ret = pthread_join(v4l2_thread, &dummy);
		v4l2_thread = 0;
	}

	return ret;
}

static void* cmr_v4l2_thread_proc(void* data)
{
	struct v4l2_buffer       buf;
	int                      evt_id;
	struct frm_info          frame;
	uint32_t                 on_flag = 0;

	if (-1 == fd) {
		CMR_LOGE("V4L2 device not opened");
		return -1;
	}

	CMR_LOGV("In");

	while(1) {
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (-1 == ioctl(fd, VIDIOC_DQBUF, &buf)) {
			CMR_LOGV("Failed to DQBuf");
			break;
		} else {
			if (V4L2_FLAG_TX_STOP == buf.flags) {
				// stopped , to do release resource
				CMR_LOGV("TX Stopped, exit thread");
				break;
			} else if (V4L2_FLAG_SYS_BUSY == buf.flags) {
				usleep(10000);
				CMR_LOGV("continue.");
				continue;
			} else {
				// normal irq
				evt_id = cmr_v4l2_evt_id(buf.flags);
				if (CMR_V4L2_MAX == evt_id) {
					continue;
				}
				if (V4L2_BUF_TYPE_VIDEO_CAPTURE == buf.type) {
					frame.channel_id = 0;
				} else if (V4L2_BUF_TYPE_PRIVATE == buf.type) {
					frame.channel_id = 1;
				} else {
					continue;
				}
				CMR_LOGV("TX got one frame! buf_type 0x%x, id 0x%x", buf.type, buf.index);
				frame.height   = buf.reserved;
				frame.frame_id = buf.index;
				frame.sec      = buf.timestamp.tv_sec;
				frame.usec     = buf.timestamp.tv_usec;
				frame.length   = buf.sequence;
				memcpy((void*)&frame.data_endian,
					(void*)&buf.bytesused,
					sizeof(struct img_data_end));
				pthread_mutex_lock(&status_mutex);
				on_flag = is_on;
				pthread_mutex_unlock(&status_mutex);
				if (on_flag) {
					pthread_mutex_lock(&cb_mutex);
					if (v4l2_evt_cb) {
						(*v4l2_evt_cb)(evt_id, &frame);
					}
					pthread_mutex_unlock(&cb_mutex);
				}

			}

		}
	}

	CMR_LOGV("Out");
	return NULL;
}

static uint32_t cmr_v4l2_get_4cc(uint32_t img_type)
{
	uint32_t                 ret_4cc;

	switch (img_type) {
	case IMG_DATA_TYPE_YUV422:
		ret_4cc = V4L2_PIX_FMT_YUV422P;
		break;
	case IMG_DATA_TYPE_YUV420:
		ret_4cc = V4L2_PIX_FMT_NV21;
		break;
	case IMG_DATA_TYPE_RGB565:
		ret_4cc = V4L2_PIX_FMT_RGB565;
		break;
	case IMG_DATA_TYPE_RAW:
		ret_4cc = V4L2_PIX_FMT_GREY;
		break;
	case IMG_DATA_TYPE_JPEG:
		ret_4cc = V4L2_PIX_FMT_JPEG;
		break;
	default:
		ret_4cc = V4L2_PIX_FMT_NV21;
		break;
	}

	return ret_4cc;
}



