#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>

#if 1
#define ALOGI(x...)  fprintf(stderr, "AtChannelTest: " x)
#define ALOGE(x...)  fprintf(stderr, "AtChannelTest: " x)
#else
#define LOG_TAG "AtChannelTest"
#include <cutils/log.h>
#endif

#include "AtChannel.h"

pthread_t       threadid;


void *sendAT(void *arg)
{
    const char* atTestCmd = "AT+CPIN?";
    int atrsp;
    char rec_buf[1024] = { };
    ALOGE("sendAT %s   Current thread id is %u\n", atTestCmd, (unsigned)pthread_self());
    atrsp = sendAt(rec_buf, sizeof(char)*1024, 0, atTestCmd);
    if (atrsp == -1) {
         ALOGE("Send %s, but no response\n", atTestCmd);
     } else {
        ALOGI("The response is %s, len is %d on thread id %u\n", rec_buf, atrsp, (unsigned)pthread_self());
     }

    return ( (void*)100);
}

int main(int argc, char **argv)
{
	int count = 10;
	int err;
	while (count--) {
		ALOGE("main:count is %d\n", 0);
		err = pthread_create(&threadid, NULL, sendAT, NULL);
	}
	sleep(100);
	return 0;
}
