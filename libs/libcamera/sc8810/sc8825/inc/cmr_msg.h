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

#ifndef CMR_MSG_H
#define CMR_MSG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <string.h>

#define      CMR_MSG_MAGIC_CODE           0xEFFEA55A
#define      CMR_MSG_WAIT_TIME            1000 //wait for 1 ms

enum {
	CMR_MSG_SUCCESS = 0,
	CMR_MSG_PARAM_ERR,
	CMR_MSG_INVALID_HANDLE,
	CMR_MSG_NO_OTHER_MSG,
	CMR_MSG_NO_MEM,
};

struct cmr_msg
{
	uint32_t                   msg_type;
	uint32_t                   sub_msg_type;
	void                       *data;
	uint32_t                   alloc_flag; /*0 , no alloc; 1, data alloc-ed by the send */
};

struct cmr_msg_cxt
{
	pthread_mutex_t            mutex;
	sem_t                      msg_sem;
	uint32_t                   msg_count;
	uint32_t                   msg_magic;
	struct cmr_msg             *msg_head;
	struct cmr_msg             *msg_write;
	struct cmr_msg             *msg_read;
};

#define MSG_INIT(name)                  \
{                                       \
	.msg_type     = 0,              \
	.sub_msg_type = 0,              \
	.data         = NULL,           \
	.alloc_flag   = 0,              \
}

#define CMR_MSG_INIT(name)               struct cmr_msg   name = MSG_INIT(name)

int cmr_msg_queue_create(uint32_t count, uint32_t *queue_handle);

int cmr_msg_get(uint32_t queue_handle, struct cmr_msg *message);

int cmr_msg_post(uint32_t queue_handle, struct cmr_msg *message);

int cmr_msg_flush(uint32_t queue_handle, struct cmr_msg *message);

int cmr_msg_queue_destroy(uint32_t queue_handle);

int cmr_msg_peak(uint32_t queue_handle, struct cmr_msg *message);


#ifdef __cplusplus
}
#endif

#endif /* CMR_MSG_H */

