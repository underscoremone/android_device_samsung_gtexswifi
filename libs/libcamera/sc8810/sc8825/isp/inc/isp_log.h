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
#ifndef _ISP_LOG_H_
#define _ISP_LOG_H_

#include <sys/types.h>
#include <utils/Log.h>
#include <stdlib.h>
#include <fcntl.h>/* low-level i/o */
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <errno.h>
/*------------------------------------------------------------------------------*
*					Compiler Flag				*
*-------------------------------------------------------------------------------*/
#ifdef   __cplusplus
extern   "C"
{
#endif
/*------------------------------------------------------------------------------*
*					Micro Define				*
*-------------------------------------------------------------------------------*/
#if 0
#define ISP_DEBUG_STR      "%s, %s, %d line,: "
#define ISP_DEBUG_ARGS    __FILE__,__FUNCTION__,__LINE__
#else
#define ISP_DEBUG_STR      "%s, %d line,: "
#define ISP_DEBUG_ARGS    __FUNCTION__,__LINE__
#endif

#define ISP_LOG(format,...) ALOGE(ISP_DEBUG_STR format, ISP_DEBUG_ARGS, ##__VA_ARGS__)

#define ISP_TRAC(_x_) ISP_LOG _x_
#define ISP_RETURN_IF_FAIL(exp,warning) do{if(exp) {ISP_TRAC(warning); return exp;}}while(0)
#define ISP_TRACE_IF_FAIL(exp,warning) do{if(exp) {ISP_TRAC(warning);}}while(0)

//#define ISP_USER_DRV_DEBUG	0

/*------------------------------------------------------------------------------*
*					Data Structures				*
*-------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*
*					Compiler Flag				*
*-------------------------------------------------------------------------------*/
#ifdef   __cplusplus
}
#endif
/*-----------------------------------------------------------------------------*/
#endif
// End

