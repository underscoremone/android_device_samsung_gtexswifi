#ifndef ANDROID_ATCHANNEL_H
#define ANDROID_ATCHANNEL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <cutils/sockets.h>
size_t sendAt(void *buf, size_t bufLen, int simId, const char* atCmd);

#ifdef __cplusplus
}
#endif


#endif // ANDROID_ATCHANNEL_H
