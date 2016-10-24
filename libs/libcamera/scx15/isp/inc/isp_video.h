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
#ifndef _ISP_VIDEO_H
#define _ISP_VIDEO_H

#include <stdio.h>

int ispvideo_RegCameraFunc(uint32_t cmd, int(*func)(uint32_t, uint32_t));
void send_img_data(uint32_t format, uint32_t width, uint32_t height, char *imgptr, int imagelen);
void send_capture_data(uint32_t format, uint32_t width, uint32_t height, char *ch0_ptr, int ch0_len,char *ch1_ptr, int ch1_len,char *ch2_ptr, int ch2_len);
void startispserver();
void stopispserver();
void validispserver(int32_t valid);
#endif
