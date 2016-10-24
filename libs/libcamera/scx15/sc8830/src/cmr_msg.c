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
#include "cmr_common.h"
#include "cmr_msg.h"

#define MSG_CHECK_MSG_MAGIC(handle) \
	do { \
		if (((struct cmr_msg_cxt*)handle)->msg_magic != CMR_MSG_MAGIC_CODE) { \
			return CMR_MSG_INVALID_HANDLE; \
		} \
	} while(0)

int cmr_msg_queue_create(unsigned int count, unsigned int *queue_handle)
{
	struct cmr_msg_cxt       *msg_cxt;

	CMR_LOGD("count 0x%x", count);

	if (0 == count) {
		return -CMR_MSG_PARAM_ERR;
	}
	msg_cxt = (struct cmr_msg_cxt*)malloc(sizeof(struct cmr_msg_cxt));
	if (NULL == msg_cxt) {
		return -CMR_MSG_NO_MEM;
	}
	bzero(msg_cxt, sizeof(*msg_cxt));
	msg_cxt->msg_head = (struct cmr_msg*)malloc((unsigned int)(count * sizeof(struct cmr_msg)));
	if (NULL == msg_cxt->msg_head) {
		free(msg_cxt);
		return -CMR_MSG_NO_MEM;
	}
	msg_cxt->msg_magic = CMR_MSG_MAGIC_CODE;
	msg_cxt->msg_count = count;
	msg_cxt->msg_number= 0;
	msg_cxt->msg_read = msg_cxt->msg_head;
	msg_cxt->msg_write = msg_cxt->msg_head;
	pthread_mutex_init(&msg_cxt->mutex, NULL);
	sem_init(&msg_cxt->msg_sem, 0, 0);
	*queue_handle = (unsigned int)msg_cxt;
	CMR_LOGD("queue_handle 0x%x", *queue_handle);

	return CMR_MSG_SUCCESS;
}

int cmr_msg_get(unsigned int queue_handle, struct cmr_msg *message, uint32_t log_level)
{
	struct cmr_msg_cxt *msg_cxt = (struct cmr_msg_cxt*)queue_handle;

	if (0 == queue_handle || NULL == message) {
		return -CMR_MSG_PARAM_ERR;
	}

	MSG_CHECK_MSG_MAGIC(queue_handle);

	sem_wait(&msg_cxt->msg_sem);
	
	pthread_mutex_lock(&msg_cxt->mutex);

	if (msg_cxt->msg_number == 0) {
		pthread_mutex_unlock(&msg_cxt->mutex);
		CMR_LOGE("MSG underflow");
		return CMR_MSG_UNDERFLOW;
	} else {
		
		if (msg_cxt->msg_read != msg_cxt->msg_write) {
			*message = *msg_cxt->msg_read;
			bzero(msg_cxt->msg_read, sizeof(struct cmr_msg));
			msg_cxt->msg_read++;
			if (msg_cxt->msg_read > msg_cxt->msg_head + msg_cxt->msg_count - 1) {
				msg_cxt->msg_read = msg_cxt->msg_head;
			}
		}
		msg_cxt->msg_number --;
	}

	pthread_mutex_unlock(&msg_cxt->mutex);
	if (0 != log_level) {
		CMR_LOGD("queue_handle 0x%x, msg type 0x%x num %d cnt %d",
			queue_handle,
			message->msg_type,
			msg_cxt->msg_number,
			msg_cxt->msg_count);
	} else {
		CMR_LOGV("queue_handle 0x%x, msg type 0x%x num %d cnt %d",
			queue_handle,
			message->msg_type,
			msg_cxt->msg_number,
			msg_cxt->msg_count);
	}
	return CMR_MSG_SUCCESS;
}

int cmr_msg_timedget(unsigned int queue_handle, struct cmr_msg *message)
{
	struct cmr_msg_cxt *msg_cxt = (struct cmr_msg_cxt*)queue_handle;
	struct timespec ts;
	int    ret;

	/* Posix mandates CLOCK_REALTIME here */
	clock_gettime( CLOCK_REALTIME, &ts );
	if (0 == queue_handle || NULL == message) {
		return -CMR_MSG_PARAM_ERR;
	}

	MSG_CHECK_MSG_MAGIC(queue_handle);

	clock_gettime(CLOCK_REALTIME, &ts);

	/* Check it as per Posix */
	if (ts.tv_sec < 0 ||
		ts.tv_nsec < 0 ||
		ts.tv_nsec >= 1000000000) {
		return -CMR_MSG_PARAM_ERR;
	}
	/*wait for 200ms*/
	ts.tv_nsec += CMR_MSG_POLLING_PERIOD;
	if (ts.tv_nsec > 1000000000) {
		ts.tv_sec += 1;
		ts.tv_nsec -= 1000000000;
	}

	ret = sem_timedwait(&msg_cxt->msg_sem, &ts);

	if (ret) {
		return CMR_MSG_NO_OTHER_MSG;
	}

	pthread_mutex_lock(&msg_cxt->mutex);

	if (msg_cxt->msg_number == 0) {
		pthread_mutex_unlock(&msg_cxt->mutex);
		CMR_LOGE("MSG underflow");
		return CMR_MSG_UNDERFLOW;
	} else {
		if (msg_cxt->msg_read != msg_cxt->msg_write) {
			*message = *msg_cxt->msg_read;
			bzero(msg_cxt->msg_read, sizeof(struct cmr_msg));
			msg_cxt->msg_read++;
			if (msg_cxt->msg_read > msg_cxt->msg_head + msg_cxt->msg_count - 1) {
				msg_cxt->msg_read = msg_cxt->msg_head;
			}
		}
		msg_cxt->msg_number --;
	}

	pthread_mutex_unlock(&msg_cxt->mutex);

	CMR_LOGD("queue_handle 0x%x, msg type 0x%x num %d cnt %d",
		queue_handle,
		message->msg_type,
		msg_cxt->msg_number,
		msg_cxt->msg_count);
	return CMR_MSG_SUCCESS;
}

int cmr_msg_post(unsigned int queue_handle, struct cmr_msg *message, uint32_t log_level)
{
	struct cmr_msg_cxt *msg_cxt = NULL;
	struct cmr_msg     *ori_node = NULL;

	if (0 == queue_handle || NULL == message) {
		CMR_LOGW("post msg to NULL queue! discard");
		return -CMR_MSG_PARAM_ERR;
	}

	msg_cxt = (struct cmr_msg_cxt*)queue_handle;
	ori_node = msg_cxt->msg_write;

	if (0 != log_level) {
		CMR_LOGD("queue_handle 0x%x, msg type 0x%x num %d cnt %d",
			queue_handle,
			message->msg_type,
			msg_cxt->msg_number,
			msg_cxt->msg_count);
	} else {
		CMR_LOGV("queue_handle 0x%x, msg type 0x%x num %d cnt %d",
			queue_handle,
			message->msg_type,
			msg_cxt->msg_number,
			msg_cxt->msg_count);
	}
	MSG_CHECK_MSG_MAGIC(queue_handle);

	pthread_mutex_lock(&msg_cxt->mutex);

	if ((msg_cxt->msg_number + 1) >= msg_cxt->msg_count) {
		pthread_mutex_unlock(&msg_cxt->mutex);
		CMR_LOGE("MSG Overflow");
		return CMR_MSG_OVERFLOW;
	} else {
		*msg_cxt->msg_write++ = *message;
		if (msg_cxt->msg_write > msg_cxt->msg_head + msg_cxt->msg_count - 1) {
			msg_cxt->msg_write = msg_cxt->msg_head;
		}

		if (msg_cxt->msg_write == msg_cxt->msg_read) {
			msg_cxt->msg_write = ori_node;
		}
		msg_cxt->msg_number ++;
	}

	pthread_mutex_unlock(&msg_cxt->mutex);

	sem_post(&msg_cxt->msg_sem);
	return CMR_MSG_SUCCESS;
}

int cmr_msg_peak(uint32_t queue_handle, struct cmr_msg *message)
{
	struct cmr_msg_cxt *msg_cxt = (struct cmr_msg_cxt*)queue_handle;
	uint32_t           msg_cnt = 0;
	int                rtn = 0;

	if (0 == queue_handle || NULL == message) {
		return -CMR_MSG_PARAM_ERR;
	}

	MSG_CHECK_MSG_MAGIC(queue_handle);
	rtn = sem_trywait(&msg_cxt->msg_sem);
	if (rtn) {
		return -CMR_MSG_NO_OTHER_MSG;
	}
	pthread_mutex_lock(&msg_cxt->mutex);

	if (msg_cxt->msg_read != msg_cxt->msg_write) {
		*message = *msg_cxt->msg_read++;
		if (msg_cxt->msg_read > msg_cxt->msg_head + msg_cxt->msg_count - 1) {
			msg_cxt->msg_read = msg_cxt->msg_head;
		}
		msg_cxt->msg_number --;
	} else {
		CMR_LOGI("No more unread msg");
		return -CMR_MSG_NO_OTHER_MSG;
	}

	pthread_mutex_unlock(&msg_cxt->mutex);

	CMR_LOGD("queue_handle 0x%x, drop msg type 0x%x", queue_handle, message->msg_type);
	return CMR_MSG_SUCCESS;
}

int cmr_msg_queue_destroy(unsigned int queue_handle)
{
	struct cmr_msg_cxt *msg_cxt = (struct cmr_msg_cxt*)queue_handle;

	CMR_LOGD("queue_handle 0x%x", queue_handle);

	if (0 == queue_handle) {
		CMR_LOGE("zero queue_handle");
		return -CMR_MSG_PARAM_ERR;
	}

	MSG_CHECK_MSG_MAGIC(queue_handle);

	if (msg_cxt->msg_head) {
		free(msg_cxt->msg_head);
		msg_cxt->msg_head = NULL;
	}
	sem_destroy(&msg_cxt->msg_sem);
	pthread_mutex_destroy(&msg_cxt->mutex);
	bzero(msg_cxt, sizeof(*msg_cxt));
	free(msg_cxt);

	return CMR_MSG_SUCCESS;
}
