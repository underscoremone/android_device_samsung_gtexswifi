#ifndef _ISP_VIDEO_H
#define _ISP_VIDEO_H

#include <stdio.h>

int ispvideo_RegCameraFunc(uint32_t cmd, int(*func)());
void send_img_data(uint32_t format, uint32_t width, uint32_t height, char *imgptr, int imagelen);
void startispserver();
//void stopispserver();

#endif
