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
#ifndef _CMR_V4L2_H_
#define _CMR_V4L2_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "cmr_common.h"

#define V4L2_BUF_MAX                   8

enum channel_num {
	CHN_0 = 0,
	CHN_1,
	CHN_2,
	CHN_MAX
};

enum channel_status {
	CHN_IDLE = 0,
	CHN_BUSY
};

enum v4l2_sensor_format {
	V4L2_SENSOR_FORMAT_YUV = 0,
	V4L2_SENSOR_FORMAT_SPI,
	V4L2_SENSOR_FORMAT_JPEG,
	V4L2_SENSOR_FORMAT_RAWRGB,
	V4L2_SENSOR_FORMAT_MAX
};

enum cmr_v4l2_evt {
	CMR_V4L2_TX_DONE = CMR_EVT_V4L2_BASE,
	CMR_V4L2_TX_ERROR,
	CMR_V4L2_TX_NO_MEM,
	CMR_V4L2_CSI2_ERR,
	CMR_V4L2_TIME_OUT,
	CMR_V4L2_MAX,
};

enum cmr_v4l2_rtn {
	CMR_V4L2_RET_OK = 0,
	CMR_V4L2_RET_RESTART,
	CMR_V4L2_RET_MAX,
};

enum if_status {
	IF_OPEN = 0,
	IF_CLOSE
};

enum cmr_v4l2_wtite_cmd_id {
	CMR_V4L2_WRITE_STOP = 0x5AA5,
	CMR_V4L2_WRITE_FREE_FRAME = 0xA55A,
	CMR_V4L2_WRITE_MAX
};

enum
{
	PREV_TRACE = 0,
	CAP_TRACE,
	TRACE_MAX,
};
	
struct img_frm_cap {
	struct img_rect                     src_img_rect;
	struct img_size                     dst_img_size;
	uint32_t                            dst_img_fmt;
	uint32_t                            notice_slice_height;
	uint32_t                            need_isp;
	uint32_t                            need_binning;
};

struct sn_cfg {
	struct img_size                     sn_size;
	struct img_rect                     sn_trim;
	uint32_t                            frm_num;
};

struct cap_cfg {
	uint32_t                            channel_id;
	uint32_t                            chn_deci_factor;
	uint32_t                            frm_num;
	struct img_frm_cap                  cfg;
};

struct buffer_cfg {
	uint32_t                            channel_id;
	uint32_t                            base_id;
	uint32_t                            count;
	uint32_t                            length;
	uint32_t                            slice_height;
	uint32_t                            start_buf_id;
	struct img_addr	                    addr[V4L2_BUF_MAX];
};

struct frm_info {
	uint32_t                            channel_id;
	uint32_t                            frame_id;
	uint32_t                            height;
	uint32_t                            sec;
	uint32_t                            usec;
	struct img_data_end                 data_endian;
	uint32_t                            length;
	uint32_t                            free;
};

typedef int (*v4l2_stream_on)(uint32_t is_on);

int cmr_v4l2_init(void);
int cmr_v4l2_deinit(void);
void cmr_v4l2_evt_reg(cmr_evt_cb  v4l2_event_cb);
int cmr_v4l2_if_cfg(struct sensor_if *sn_if);
int cmr_v4l2_if_decfg(struct sensor_if *sn_if);
int cmr_v4l2_sn_cfg(struct sn_cfg *config);
int cmr_v4l2_cap_cfg(struct cap_cfg *config);
int cmr_v4l2_buff_cfg(struct buffer_cfg *buf_cfg);
int cmr_v4l2_cap_start(uint32_t skip_num);
int cmr_v4l2_cap_stop(void);
int cmr_v4l2_cap_resume(uint32_t channel_id, uint32_t skip_number, uint32_t deci_factor, int frm_num);
int cmr_v4l2_cap_pause(uint32_t channel_id, uint32_t reconfig_flag);
int cmr_v4l2_free_frame(uint32_t channel_id, uint32_t index);
int cmr_v4l2_scale_capability(uint32_t *width, uint32_t *sc_factor);
int cmr_v4l2_get_cap_time(uint32_t *sec, uint32_t *usec);
int cmr_v4l2_flash_cb(uint32_t opt);
int cmr_v4l2_stream_cb(v4l2_stream_on str_on);
void cmr_v4l2_set_trace_flag(uint32_t trace_owner, uint32_t val);

#ifdef __cplusplus
}
#endif

#endif //for _CMR_V4L2_H_
