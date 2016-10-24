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
#include <linux/videodev2.h>
#include <pthread.h>
#include <unistd.h>
#include "cmr_v4l2.h"

#define CMR_TIMING_LEN 16

#define CMR_CHECK_FD \
		do { \
			if (-1 == fd) { \
				CMR_LOGE("V4L2 device not opened"); \
				return -1; \
			} \
		} while(0)

enum
{
	V4L2_FLAG_TX_DONE  = 0x00,
	V4L2_FLAG_NO_MEM   = 0x01,
	V4L2_FLAG_TX_ERR   = 0x02,
	V4L2_FLAG_CSI2_ERR = 0x03,
	V4L2_FLAG_SYS_BUSY = 0x04,
	V4L2_FLAG_TIME_OUT = 0x10,
	V4L2_FLAG_TX_STOP  = 0xFF
};

enum
{
	NO_EXIT = 0,
	EXIT_BY_HOST,
	EXIT_EXCEPTION,
};

/*
	capability       parameters                         structure member
	0x1000           capture mode, single or multi      capture.capturemode
	0x1001           skip number for CAP sub-module     capture.reserved[0];
	0x1002           image width/height from sensor     capture.reserved[2], capture.reserved[3];
	0x1003           base id for each frame             capture.reserved[1];

	0x2000           path skip and deci number          recerved[0] channel, [1] deci number
	0x2001           path pause                         recerved[0] channel
	0x2002           path resume                        recerved[0] channel
*/
enum dcam_parm_id {
	CAPTURE_MODE = 0x1000,
	CAPTURE_SKIP_NUM,
	CAPTURE_SENSOR_SIZE,
	CAPTURE_SENSOR_TRIM,
	CAPTURE_FRM_ID_BASE,
	CAPTURE_SET_CROP,
	CAPTURE_SET_FLASH,
	PATH_FRM_DECI = 0x2000,
	PATH_PAUSE = 0x2001,
	PATH_RESUME = 0x2002,
};

static char               dev_name[50] = "/dev/video0";
static int                fd = -1;
static cmr_evt_cb         v4l2_evt_cb = NULL;
static pthread_mutex_t    cb_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t    status_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint32_t           is_on = 0;
static uint32_t           is_prev_trace = 0;
static uint32_t           is_cap_trace = 0;
static pthread_t          v4l2_thread;
static uint32_t           chn_status[CHN_MAX];
static int                chn_frm_num[CHN_MAX];
static v4l2_stream_on     stream_on_cb = NULL;

static int cmr_v4l2_create_thread(void);
static int cmr_v4l2_kill_thread(void);
static void* cmr_v4l2_thread_proc(void* data);
static uint32_t cmr_v4l2_get_4cc(uint32_t img_type);

int cmr_v4l2_init(void)
{
	int                      ret = 0;

	CMR_LOGI("Start to open V4L2 device.");
	fd = open(dev_name, O_RDWR, 0);
	if (-1 == fd) {
		CMR_LOGE("Failed to open dcam device.errno : %d", errno);
		fprintf(stderr, "Cannot open '%s': %d, %s\n", dev_name, errno,  strerror(errno));
		exit(EXIT_FAILURE);
	} else {
		CMR_LOGI("OK to open device.");
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
	stream_on_cb = NULL;
	is_prev_trace = 0;
	is_cap_trace = 0;
	memset(chn_status, 0, sizeof(chn_status));
	return ret;
}

int cmr_v4l2_deinit(void)
{
	int                      ret = 0;

	CMR_LOGI("Start to close V4L2 device.");

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
	CMR_LOGD("thread kill done.");
	pthread_mutex_lock(&cb_mutex);
	v4l2_evt_cb = NULL;
	is_prev_trace = 0;
	is_cap_trace = 0;
	pthread_mutex_unlock(&cb_mutex);
	pthread_mutex_destroy(&cb_mutex);
	pthread_mutex_destroy(&status_mutex);
	CMR_LOGI("close device.\n");
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

timint_param[14]      IF status

*/
int cmr_v4l2_if_cfg(struct sensor_if *sn_if)
{
	int                      ret = 0;
	struct v4l2_control      ctrl;
	uint32_t                 timing_param[CMR_TIMING_LEN];

	CMR_CHECK_FD;

	timing_param[CMR_TIMING_LEN-1] = sn_if->if_type;
	timing_param[CMR_TIMING_LEN-2] = IF_OPEN;
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
		timing_param[8] = sn_if->if_spec.mipi.pclk;
	}

	ctrl.id = V4L2_CID_USER_CLASS;
	ctrl.value = (int32_t)&timing_param[0];

	ret = ioctl(fd, VIDIOC_S_CTRL, &ctrl);

	CMR_LOGI("Set dv timing, ret %d, if type %d, mode %d, deci %d, status %d",
		ret,
		timing_param[CMR_TIMING_LEN-1],
		timing_param[0],
		timing_param[3],
		timing_param[CMR_TIMING_LEN-2]);

	return ret;
}

int cmr_v4l2_if_decfg(struct sensor_if *sn_if)
{
	int                      ret = 0;
	struct v4l2_control      ctrl;
	uint32_t                 timing_param[CMR_TIMING_LEN];

	CMR_CHECK_FD;

	timing_param[CMR_TIMING_LEN-1] = sn_if->if_type;
	timing_param[CMR_TIMING_LEN-2] = IF_CLOSE;

	ctrl.id = V4L2_CID_USER_CLASS;
	ctrl.value = (int32_t)&timing_param[0];

	ret = ioctl(fd, VIDIOC_S_CTRL, &ctrl);

	CMR_LOGI("Set dv timing, ret %d, if type %d, status %d.",
		ret,
		timing_param[CMR_TIMING_LEN-1],
		timing_param[CMR_TIMING_LEN-2]);

	return ret;
}

int cmr_v4l2_sn_cfg(struct sn_cfg *config)
{
	int                      ret = 0;
	struct v4l2_streamparm   stream_parm;

	stream_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	stream_parm.parm.capture.capability = CAPTURE_MODE;
	if (1 == config->frm_num) {
		stream_parm.parm.capture.capturemode = 0;/* means single-frame sample mode */
	} else {
		stream_parm.parm.capture.capturemode = 1;/* means multi-frame sample mode */
	}
	ret = ioctl(fd, VIDIOC_S_PARM, &stream_parm);
	CMR_RTN_IF_ERR(ret);

	stream_parm.parm.capture.capability = CAPTURE_SENSOR_SIZE;
	stream_parm.parm.capture.reserved[2] = config->sn_size.width;
	stream_parm.parm.capture.reserved[3] = config->sn_size.height;
	ret = ioctl(fd, VIDIOC_S_PARM, &stream_parm);

	CMR_LOGI("sn_trim x y w h %d, %d, %d, %d",
		config->sn_trim.start_x,
		config->sn_trim.start_y,
		config->sn_trim.width,
		config->sn_trim.height);

	stream_parm.parm.capture.capability = CAPTURE_SENSOR_TRIM;
	stream_parm.parm.capture.reserved[0] = config->sn_trim.start_x;
	stream_parm.parm.capture.reserved[1] = config->sn_trim.start_y;
	stream_parm.parm.capture.reserved[2] = config->sn_trim.width;
	stream_parm.parm.capture.reserved[3] = config->sn_trim.height;
	ret = ioctl(fd, VIDIOC_S_PARM, &stream_parm);
exit:
	return ret;

}

int cmr_v4l2_cap_cfg(struct cap_cfg *config)
{
	int                      ret = 0;
	struct v4l2_crop         crop;
	struct v4l2_fmtdesc      fmtdesc;
	struct v4l2_format       format;
	uint32_t                 found = 0;
	uint32_t                 pxl_fmt;
	uint32_t                 cfg_id = 0;
	enum v4l2_buf_type       buf_type;
	struct v4l2_streamparm   stream_parm;

	CMR_CHECK_FD;

	if (NULL == config)
		return -1;

	CMR_LOGI("channel_id %d, frm_num %d.", config->channel_id, config->frm_num);
	cfg_id = config->channel_id;
	stream_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	stream_parm.parm.capture.capability = PATH_FRM_DECI;

	stream_parm.parm.capture.reserved[0] = config->channel_id;
	stream_parm.parm.capture.reserved[1] = config->chn_deci_factor;
	ret = ioctl(fd, VIDIOC_S_PARM, &stream_parm);
	CMR_LOGI("channel_id  %d, deci_factor %d, ret %d \n", config->channel_id, config->chn_deci_factor, ret);

	buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (CHN_1 == cfg_id) {
		chn_frm_num[1] = config->frm_num;
	} else  if (CHN_2 == cfg_id) {
		chn_frm_num[2] = config->frm_num;
	} else {
		/* CHN_0 */
		chn_frm_num[0] = config->frm_num;
	}
#if 0
	/* firstly, set the crop rectangle PATH1 module, this should be called before VIDIOC_TRY_FMT called */
	crop.c.left   = config->cfg.src_img_rect.start_x;
	crop.c.top    = config->cfg.src_img_rect.start_y;
	crop.c.width  = config->cfg.src_img_rect.width;
	crop.c.height = config->cfg.src_img_rect.height;
	crop.type     = buf_type;
	ret = ioctl(fd, VIDIOC_S_CROP, &crop);
	CMR_RTN_IF_ERR(ret);
#endif

	stream_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	stream_parm.parm.capture.capability = CAPTURE_SET_CROP;
	stream_parm.parm.capture.extendedmode = config->channel_id;
	stream_parm.parm.capture.reserved[0] = config->cfg.src_img_rect.start_x;
	stream_parm.parm.capture.reserved[1] = config->cfg.src_img_rect.start_y;
	stream_parm.parm.capture.reserved[2] = config->cfg.src_img_rect.width;
	stream_parm.parm.capture.reserved[3] = config->cfg.src_img_rect.height;
	ret = ioctl(fd, VIDIOC_S_PARM, &stream_parm);
	CMR_RTN_IF_ERR(ret);

	/* secondly,  check whether the output format described by config->cfg[cfg_id] can be supported by the low layer */
	found = 0;
	fmtdesc.index = 0;
	fmtdesc.type  = buf_type;
	fmtdesc.flags = config->channel_id;
	pxl_fmt = cmr_v4l2_get_4cc(config->cfg.dst_img_fmt);
	while (0 == ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc)) {
		if (fmtdesc.pixelformat == pxl_fmt) {
			CMR_LOGI("FourCC 0x%x is supported by the low layer", pxl_fmt);
			found = 1;
			break;
		}
		fmtdesc.index++;
	}

	if (found) {
		bzero(&format, sizeof(struct v4l2_format));
		format.type   = buf_type;
		format.fmt.pix.colorspace = config->channel_id;
		format.fmt.pix.width = config->cfg.dst_img_size.width;
		format.fmt.pix.height = config->cfg.dst_img_size.height;
		format.fmt.pix.pixelformat = pxl_fmt; //fourecc
		format.fmt.pix.priv = config->cfg.need_isp;
		ret = ioctl(fd, VIDIOC_TRY_FMT, &format);
		CMR_LOGI("need binning, %d", format.fmt.pix.sizeimage);
		if (format.fmt.pix.sizeimage) {
			config->cfg.need_binning = 1;
		}
		if (0 == ret) {
			chn_status[cfg_id] = CHN_BUSY;
		} else if (ret > 0) {
			CMR_LOGI("need restart");
			ret = CMR_V4L2_RET_RESTART;
		}
	} else {
		CMR_LOGI("fourcc not founded dst_img_fmt=0x%x \n", config->cfg.dst_img_fmt);
	}

exit:
	CMR_LOGI("ret %d", ret);
	return ret;
}

int cmr_v4l2_buff_cfg (struct buffer_cfg *buf_cfg)
{
	int                      ret = 0;
	uint32_t                 i;
	struct v4l2_buffer       v4l2_buf;
	struct v4l2_streamparm   stream_parm;

	CMR_CHECK_FD;

	if (NULL == buf_cfg || buf_cfg->count > V4L2_BUF_MAX)
		return -1;

	CMR_LOGI("%d %d 0x%x ", buf_cfg->channel_id, buf_cfg->count, buf_cfg->base_id);

	v4l2_buf.type  = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	v4l2_buf.flags = buf_cfg->channel_id;

	/* firstly , set the base index for each channel */
	stream_parm.type                      = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	stream_parm.parm.capture.capability   = CAPTURE_FRM_ID_BASE;
	stream_parm.parm.capture.reserved[1]  = buf_cfg->base_id;
	stream_parm.parm.capture.extendedmode = buf_cfg->channel_id;
	ret = ioctl(fd, VIDIOC_S_PARM, &stream_parm);
	CMR_RTN_IF_ERR(ret);

	/* secondly , set the frame address */
	for (i = 0; i < buf_cfg->count; i++) {
		v4l2_buf.m.userptr  = buf_cfg->addr[i].addr_y;
		v4l2_buf.length        = buf_cfg->addr[i].addr_u;
		v4l2_buf.reserved   = buf_cfg->addr[i].addr_v;
		CMR_LOGD("VIDIOC_QBUF: buf %d: Y=0x%x, U=0x%x, V=0x%x \n",
			i, buf_cfg->addr[i].addr_y, buf_cfg->addr[i].addr_u, buf_cfg->addr[i].addr_v);
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
	int                      i, ret = 0;
	struct v4l2_streamparm   stream_parm;
	enum v4l2_buf_type       buf_type;

	CMR_CHECK_FD;

	stream_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	stream_parm.parm.capture.capability = CAPTURE_SKIP_NUM;
	stream_parm.parm.capture.reserved[0] = skip_num; /* just modify the skip number parameter */
	ret = ioctl(fd, VIDIOC_S_PARM, &stream_parm);
	CMR_RTN_IF_ERR(ret);

	ret = ioctl(fd, VIDIOC_STREAMON, &buf_type);
	if (0 == ret) {
		pthread_mutex_lock(&status_mutex);
		is_on = 1;
		pthread_mutex_unlock(&status_mutex);
	}
	if (stream_on_cb) {
		(*stream_on_cb)(1);
	}
exit:
	CMR_LOGI("ret = %d.",ret);
	return ret;
}

int cmr_v4l2_cap_stop(void)
{
	int                      i, ret = 0;
	enum v4l2_buf_type       buf_type;

	CMR_CHECK_FD;
	pthread_mutex_lock(&status_mutex);
	is_on = 0;
	pthread_mutex_unlock(&status_mutex);

	ret = ioctl(fd, VIDIOC_STREAMOFF, &buf_type);
	for (i = 0; i < CHN_MAX; i ++) {
		chn_status[i] = CHN_IDLE;
	}
	if (stream_on_cb) {
		(*stream_on_cb)(0);
	}

exit:
	CMR_LOGI("ret = %d.",ret);
	return ret;
}

void cmr_v4l2_set_trace_flag(uint32_t trace_owner, uint32_t val)
{
	if (PREV_TRACE == trace_owner) {
		is_prev_trace = val;
	} else if (CAP_TRACE == trace_owner) {
		is_cap_trace = val;
	} else {
		CMR_LOGE("unknown trace owner!");
	}
}

/*
parameters for v4l2_ext_controls
 ctrl_class,     should be reserved by V4L2, can be V4L2_CTRL_CLASS_USER or
                       V4L2_CTRL_CLASS_CAMERA, etc;
 count,          pause/resume control , 0 means pause, 1 means resume;
 error_idx,      channel id;
 reserved[0],    channel id;
 reserved[1],    deci_factor;
 struct v4l2_ext_control *controls;
*/
int cmr_v4l2_cap_resume(uint32_t channel_id, uint32_t skip_number, uint32_t deci_factor, int frm_num)
{
	int                      ret = 0;
	struct v4l2_streamparm   stream_parm;

	CMR_CHECK_FD;

	CMR_LOGI("channel_id %d, frm_num %d", channel_id,frm_num);

	if (CHN_1 == channel_id) {
		chn_frm_num[1] = frm_num;
	} else  if (CHN_2 == channel_id) {
		chn_frm_num[2] = frm_num;
	} else {
		/* CHN_0 */
		chn_frm_num[0] = frm_num;
	}

	stream_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	stream_parm.parm.capture.capability = PATH_RESUME;

	stream_parm.parm.capture.reserved[0] = channel_id;
	chn_status[channel_id] = CHN_BUSY;
	ret = ioctl(fd, VIDIOC_S_PARM, &stream_parm);
	return ret;
}

int cmr_v4l2_cap_pause(uint32_t channel_id, uint32_t reconfig_flag)
{
	int                      ret = 0;
	struct v4l2_streamparm   stream_parm;

	CMR_CHECK_FD;

	CMR_LOGI("channel_id %d,reconfig_flag %d.", channel_id,reconfig_flag);

	stream_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	stream_parm.parm.capture.capability = PATH_PAUSE;

	stream_parm.parm.capture.reserved[0] = channel_id;
	stream_parm.parm.capture.reserved[1] = reconfig_flag;
	chn_status[channel_id] = CHN_IDLE;
	ret = ioctl(fd, VIDIOC_S_PARM, &stream_parm);
	return ret;
}

int cmr_v4l2_get_cap_time(uint32_t *sec, uint32_t *usec)
{
	int                      ret = 0;
	struct v4l2_streamparm   stream_parm;

	stream_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = ioctl(fd, VIDIOC_G_PARM, &stream_parm);
	CMR_RTN_IF_ERR(ret);

	*sec  = stream_parm.parm.capture.timeperframe.numerator;
	*usec = stream_parm.parm.capture.timeperframe.denominator;
	CMR_LOGI("sec=%d, usec=%d \n", *sec, *usec);

exit:
	return ret;

}

int cmr_v4l2_free_frame(uint32_t channel_id, uint32_t index)
{
	int                      ret = 0;
	struct v4l2_buffer       v4l2_buf;

	CMR_LOGV("channel id %d, index 0x%x", channel_id, index);

	CMR_CHECK_FD;

	pthread_mutex_lock(&status_mutex);
	if (0 == is_on) {
		pthread_mutex_unlock(&status_mutex);
		return ret;
	}
	pthread_mutex_unlock(&status_mutex);
	if (CHN_BUSY != chn_status[channel_id]) {
		CMR_LOGI("channel %d not on, no need to free current frame", channel_id);
		ret = 0;
		return ret;
	}
	bzero(&v4l2_buf, sizeof(struct v4l2_buffer));
	v4l2_buf.flags = CMR_V4L2_WRITE_FREE_FRAME;
	v4l2_buf.type  = channel_id;
	v4l2_buf.index = index;
	ret = write(fd, &v4l2_buf, sizeof(struct v4l2_buffer));
	if (ret) {
		CMR_LOGE("Failed to free frame, %d", ret);
		ret = 0;
	}
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
	CMR_LOGI("width %d, sc_factor %d", *width, *sc_factor);
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

	case V4L2_FLAG_TIME_OUT:
		ret = CMR_V4L2_TIME_OUT;
		break;

	default:
		CMR_LOGI("isr_flag 0x%x", isr_flag);
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
	struct v4l2_buffer       v4l2_buf;
	void                     *dummy;

	CMR_CHECK_FD;

	CMR_LOGI("Call write function to kill v4l2 manage thread");
	bzero(&v4l2_buf, sizeof(struct v4l2_buffer));
	v4l2_buf.flags = CMR_V4L2_WRITE_STOP;
	ret = write(fd, &v4l2_buf, sizeof(struct v4l2_buffer)); // kill thread;
	if (ret > 0) {
		CMR_LOGI("write OK!");
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
	int                      frm_num = -1;

	if (-1 == fd) {
		CMR_LOGE("V4L2 device not opened");
		return (void*)-1;
	}

	CMR_LOGI("In");

	while(1) {
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (-1 == ioctl(fd, VIDIOC_DQBUF, &buf)) {
			CMR_LOGI("Failed to DQBuf");
			break;
		} else {
			if (V4L2_FLAG_TX_STOP == buf.flags) {
				// stopped , to do release resource
				CMR_LOGI("TX Stopped, exit thread");
				break;
			} else if (V4L2_FLAG_SYS_BUSY == buf.flags) {
				usleep(10000);
				CMR_LOGI("continue.");
				continue;
			} else {
				// normal irq
				evt_id = cmr_v4l2_evt_id(buf.flags);
				if (CMR_V4L2_MAX == evt_id) {
					continue;
				}

				frame.channel_id = buf.type;
				frm_num = chn_frm_num[buf.type];
				frame.free = 0;
				if (CMR_V4L2_TX_DONE == evt_id) {
					if (frm_num == 0) {
						frame.free = 1;
					} else if (-1 != frm_num) {
						frm_num--;
					}
				}

				if (CHN_1 == frame.channel_id) {
					chn_frm_num[CHN_1] = frm_num;
				} else if (CHN_2 == frame.channel_id) {
					chn_frm_num[CHN_2] = frm_num;
				} else if (CHN_0 == frame.channel_id) {
					chn_frm_num[CHN_0] = frm_num;
				}

				if ((is_prev_trace && CHN_1 == frame.channel_id)
					|| (is_cap_trace && CHN_1 != frame.channel_id))
					CMR_LOGI("got one frame! type 0x%x, id 0x%x, evt_id 0x%x sec %u usec %u",
						buf.type,
						buf.index,
						evt_id,
						(uint32_t)buf.timestamp.tv_sec,
						(uint32_t)buf.timestamp.tv_usec);
				else
					CMR_LOGV("got one frame! type 0x%x, id 0x%x, evt_id 0x%x sec %u usec %u",
						buf.type,
						buf.index,
						evt_id,
						(uint32_t)buf.timestamp.tv_sec,
						(uint32_t)buf.timestamp.tv_usec);

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

	CMR_LOGI("Out");
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

int cmr_v4l2_flash_cb(uint32_t opt)
{
	int                      ret = 0;
	struct v4l2_streamparm   stream_parm;

	CMR_CHECK_FD;
	stream_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	stream_parm.parm.capture.capability = CAPTURE_SET_FLASH;
	stream_parm.parm.capture.reserved[0] = opt;
	ret = ioctl(fd, VIDIOC_S_PARM, &stream_parm);
	if (ret) {
		CMR_LOGE("error");
	}
	return ret;
}
int cmr_v4l2_stream_cb(v4l2_stream_on str_on)
{
	int                      ret = 0;

	stream_on_cb = str_on;

	return ret;
}
