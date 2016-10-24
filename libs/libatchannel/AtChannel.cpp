#define LOG_TAG "AtChannel"
#define LOG_NDEBUG 0

#include <android/log.h>
#include <binder/IServiceManager.h>
#include <utils/String8.h>
#include <utils/String16.h>
#include <cutils/properties.h>
#include <cutils/sockets.h>
#include <secril-client.h>
#include <telephony/ril.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <AtChannel.h>

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

using namespace std;
using namespace android;

typedef enum _req_state {
	REQ_STATE_PENDING,
	REQ_STATE_UNSOL,
	REQ_STATE_COMPLETED
} req_state_t;

static size_t num_rild = 1;
static HRilClient ril_client[2];
static pthread_mutex_t mutex[2];
static pthread_cond_t cond[2];
static req_state_t req_state[2];
static char data[2][1024];
static size_t datalen[2];

static int cond_timedwait(pthread_mutex_t *mutex, pthread_cond_t *cond, unsigned long sec);
static size_t create_at_req(void *out, size_t size, const char *at);
static void print_hex(const void *data, size_t len);
static int ril_on_unsolicited_handler(HRilClient handle, const void *data, size_t datalen);
static int ril_on_complete_handler(HRilClient handle, const void *data, size_t datalen);
static int ril_get_simId(HRilClient hrc);
static int rild_connect_if_required(int sim_id);

size_t sendAt(void *buf, size_t bufLen, int simId, const char* atCmd)
{
	ALOGD("sendAt: [atCmd=%s]", atCmd);
	ALOGD("sendAt: [simId=%d]", simId);
	size_t result = 0;
	if ((simId == 0 || simId == 1) && rild_connect_if_required(simId) == 0) {
		char at[1024];
		size_t at_len = create_at_req(at, sizeof(at), atCmd);
		bool completed = false;
		int ret;
		pthread_mutex_lock(&mutex[simId]);
		do {
			ALOGI("InvokeOemRequestHookRaw: BEGIN");
			ret = InvokeOemRequestHookRaw(ril_client[simId], at, at_len);
			ALOGI("sendAt(): InvokeOemRequestHookRaw: END [ret=%d]\n", ret);
		} while (ret == RIL_CLIENT_ERR_AGAIN);
		do {
			ALOGI("cond_timedwait(): BEGIN");
			do {
				if (cond_timedwait(&mutex[simId], &cond[simId], 10UL) == ETIMEDOUT)
					break;
			} while (req_state[simId] == REQ_STATE_PENDING);
			ALOGI("cond_timedwait(): END: [%d]\n", req_state[simId]);
			switch (req_state[simId]) {
			case REQ_STATE_UNSOL:
				ALOGI("sendAt(): REQ_STATE_UNSOL: [");
				print_hex(data[simId], datalen[simId]);
				ALOGI("sendAt(): REQ_STATE_UNSOL: ]");
				break;
			case REQ_STATE_COMPLETED:
				ALOGI("sendAt(): REQ_STATE_COMPLETED: [");
				print_hex(data[simId], datalen[simId]);
				ALOGI("sendAt(): REQ_STATE_COMPLETED: ]");
				memset(buf, 0, bufLen);
				memcpy(buf, data[simId], MIN(bufLen, datalen[simId]));
				completed = true;
				break;
			case REQ_STATE_PENDING:
				/* Timedout */
				ALOGW("sendAt(): ######## TIMEDOUT! ########");
				completed = true;
				break;
			}
		} while (!completed);
		if (req_state[simId] == REQ_STATE_COMPLETED)
			result = datalen[simId];
		pthread_mutex_unlock(&mutex[simId]);
	}
	return result;
}

size_t create_at_req(void *out, size_t size, const char *at)
{
	char *ptr = (char *)out;
	ptr[0] = 0x30;
	ptr[1] = 0x00;
	return snprintf(ptr + 2, size - 2, "%s", at) + 2 + 1;
}

void print_hex(const void *data, size_t len)
{
	char hex_buf[1024] = { 0 };
	if (len < 2*sizeof(hex_buf)) {
		char *data_ptr = (char *) data;
		char *ptr = hex_buf;
		memset(hex_buf, 0, sizeof(hex_buf));
		for (size_t i = 0; i < len; ++i) {
			sprintf(ptr, "%02x", data_ptr[i]);
			ptr += 2;
		}
		ALOGI("%s", hex_buf);
	} else {
		ALOGI("(null)");
	}
}

int cond_timedwait(pthread_mutex_t *mutex, pthread_cond_t *cond, unsigned long sec)
{
	struct timespec time_to_wait;
	struct timeval now;
	gettimeofday(&now, NULL);
	time_to_wait.tv_sec = now.tv_sec + sec;
	time_to_wait.tv_nsec = now.tv_usec * 1000UL;
	return pthread_cond_timedwait(cond, mutex, &time_to_wait);
}

int ril_on_unsolicited_handler(HRilClient handle, const void *data, size_t datalen)
{
	int simId;
	if ((simId = ril_get_simId(handle)) >= 0) {
		pthread_mutex_lock(&mutex[simId]);

		memcpy(::data[simId], data, datalen);
		::datalen[simId] = datalen;
		req_state[simId] = REQ_STATE_UNSOL;

		pthread_cond_signal(&cond[simId]);
		pthread_mutex_unlock(&mutex[simId]);
	}
	return 0;
}

int ril_on_complete_handler(HRilClient handle, const void *data, size_t datalen)
{
	int simId;
	if ((simId = ril_get_simId(handle)) >= 0) {
		pthread_mutex_lock(&mutex[simId]);

		memcpy(::data[simId], data, datalen);
		::datalen[simId] = datalen;
		req_state[simId] = REQ_STATE_COMPLETED;

		pthread_cond_signal(&cond[simId]);
		pthread_mutex_unlock(&mutex[simId]);
	}
	return 0;
}

int ril_get_simId(HRilClient hrc)
{
	if (hrc == ril_client[0])
		return 0;
	if (hrc == ril_client[1])
		return 1;
	return -1;
}

int rild_connect_if_required(int sim_id)
{
	int result = 0;
	if (!isConnected_RILD(ril_client[sim_id])) {
		int (*connect_rild)(HRilClient client) = sim_id == 0
				? Connect_RILD
				: Connect_RILD_Second;
		result = connect_rild(ril_client[sim_id]);
		ALOGD("Connect_RILD: [sim_id=%d] [ret=%d]", sim_id, result);
		if (result == 0) {
			ALOGI("RegisterUnsolicitedHandler: [ret=%d]\n",
					RegisterUnsolicitedHandler(
							ril_client[sim_id],
							RIL_REQUEST_OEM_HOOK_RAW,
							ril_on_unsolicited_handler));
			ALOGI("RegisterRequestCompleteHandler: [ret=%d]\n",
					RegisterRequestCompleteHandler(
							ril_client[sim_id],
							RIL_REQUEST_OEM_HOOK_RAW,
							ril_on_complete_handler));
		}
	}
	return result;
}

static void init() __attribute__((constructor));

void init()
{
	char prop[PROPERTY_VALUE_MAX];
	property_get("ro.modem.w.count", prop, "1");
	ALOGD("num_rild: [%d]", (num_rild = atoi(prop)));
	if (num_rild > 2) {
		ALOGW("num_rild: [%d]", num_rild);
		ALOGW("num_rild: set to 2");
		num_rild = 2;
	}
	unsigned long max_waittime = 10;
	for (size_t i = 0; i < num_rild; ++i) {
		ALOGD("pthread_mutex_init: [sim_id=%d] [ret=%d]", i, pthread_mutex_init(&mutex[i], NULL));
		ALOGD("pthread_cond_init: [sim_id=%d] [ret=%d]", i, pthread_cond_init(&cond[i], NULL));
		ALOGD("OpenClient_RILD: [sim_id=%d] [ret=%p]", i, (ril_client[i] = OpenClient_RILD()));
		ALOGD("rild_connect_if_required: [ret=%d]", rild_connect_if_required(i));
	}
}
