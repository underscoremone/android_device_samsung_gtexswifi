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

#define LOG_NDEBUG 0
#define LOG_TAG "SprdCameraHardware"

#include <camera/Camera.h>
#include "SprdCameraHardwareInterface.h"
#include <utils/Log.h>
#include <utils/String16.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <cutils/properties.h>
#include "gralloc_priv.h"
#include "ion_sprd.h"
#include <media/hardware/MetadataBufferType.h>
#include "SprdOEMCamera.h"

#ifdef CONFIG_CAMERA_ISP
extern "C" {
#include "isp_video.h"
}
#endif

namespace android {

/**********************Macro Define**********************/
#define STOP_PREVIEW_BEFORE_CAPTURE 0
#define LOGV ALOGV
#define LOGE ALOGE
#define LOGI ALOGI
#define LOGW ALOGW
#define LOGD ALOGD
#define PRINT_TIME 0
#define ROUND_TO_PAGE(x) (((x)+0xfff)&~0xfff)
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*x))
#define METADATA_SIZE 28/* (7 * 4) */
#define SET_PARM(x,y) do {\
			LOGV("%s: set camera param: %s, %d", __func__, #x, y);\
			camera_set_parm (x, y, NULL, NULL);\
		} while(0)
#define SIZE_ALIGN(x) (((x)+15)&(~15))
#define SWITCH_MONITOR_QUEUE_SIZE 50
#define GET_START_TIME do {\
			s_start_timestamp = systemTime();\
		} while(0)
#define GET_END_TIME do {\
			s_end_timestamp = systemTime();\
		} while(0)
#define GET_USE_TIME do {\
			s_use_time = (s_end_timestamp - s_start_timestamp)/1000000;\
		}while(0)
#define SET_PARAM_TIMEOUT    2000000000     /*2000ms*/
#define CAP_TIMEOUT          5000000000     /*5000ms*/
#define PREV_TIMEOUT         5000000000     /*5000ms*/
#define CANCEL_AF_TIMEOUT    1000000000     /*1000ms*/
#define SET_PARAMS_TIMEOUT   250            /*250 means 250*10ms*/
#define ON_OFF_ACT_TIMEOUT   50             /*50 means 50*10ms*/

#define NO_FREQ_REQ          0
#define NO_FREQ_STR          "0"
#if defined(CONFIG_CAMERA_SMALL_PREVSIZE)
#define BASE_FREQ_REQ        192
#define BASE_FREQ_STR        "192000"
#define MEDIUM_FREQ_REQ      200
#define MEDIUM_FREQ_STR      "200000"
#define HIGH_FREQ_REQ        300
#define HIGH_FREQ_STR        "300000"
#else
#define BASE_FREQ_REQ        200
#define BASE_FREQ_STR        "200000"
#define MEDIUM_FREQ_REQ      300
#define MEDIUM_FREQ_STR      "300000"
#define HIGH_FREQ_REQ        500
#define HIGH_FREQ_STR        "500000"
#endif

/**********************Static Members**********************/
static nsecs_t s_start_timestamp = 0;
static nsecs_t s_end_timestamp = 0;
static int s_use_time = 0;
static int s_mem_method = 0;/*0: physical address, 1: iommu  address*/
static nsecs_t cam_init_begin_time = 0;
bool gIsApctCamInitTimeShow = false;
bool gIsApctRead = false;

gralloc_module_t const* SprdCameraHardware::mGrallocHal = NULL;

const CameraInfo SprdCameraHardware::kCameraInfo[] = {
	{
		CAMERA_FACING_BACK,
		90,/*orientation*/
	},
#ifndef CONFIG_DCAM_SENSOR_NO_FRONT_SUPPORT
	{
		CAMERA_FACING_FRONT,
		270,/*orientation*/
	},
#endif
};

const CameraInfo SprdCameraHardware::kCameraInfo3[] = {
	{
		CAMERA_FACING_BACK,
		90,/*orientation*/
	},

	{
		CAMERA_FACING_FRONT,
		270,/*orientation*/
	},

	{
		2,
		0,/* orientation */
	}
};

int SprdCameraHardware::getPropertyAtv()
{
	char propBuf_atv[PROPERTY_VALUE_MAX] = {0};
	int atv = 0;

	property_get("sys.camera.atv", propBuf_atv, "0");

	if (0 == strcmp(propBuf_atv, "1")) {
		atv = 1;
	} else {
		atv = 0;
	}

	LOGI("getPropertyAtv:%d", atv);

	return atv;
}

int SprdCameraHardware::getNumberOfCameras()
{
	int num = 0;

	if (1 == getPropertyAtv()) {
		num = sizeof(SprdCameraHardware::kCameraInfo3) / sizeof(SprdCameraHardware::kCameraInfo3[0]);
	} else {
		num = sizeof(SprdCameraHardware::kCameraInfo) / sizeof(SprdCameraHardware::kCameraInfo[0]);
	}

	LOGI("getNumberOfCameras:%d",num);

	return num;
}

static void writeCamInitTimeToApct(char *buf)
{
	int apct_dir_fd = open("/data/apct", O_CREAT, 0777);

	if (apct_dir_fd >= 0) {
		fchmod(apct_dir_fd, 0777);
		close(apct_dir_fd);
	}

	int apct_fd = open("/data/apct/apct_data.log", O_CREAT | O_RDWR | O_TRUNC, 0666);

	if (apct_fd >=0) {
		char buf[100] = {0};
		sprintf(buf, "\n%s", buf);
		write(apct_fd, buf, strlen(buf));
		fchmod(apct_fd, 0666);
		close(apct_fd);
	}
}

static void writeCamInitTimeToProc(float init_time)
{
	char cam_time_buf[256] = {0};
	const char *cam_time_proc = "/proc/benchMark/cam_time";

	sprintf(cam_time_buf, "Camera init time:%.2fs", init_time);

	FILE *f = fopen(cam_time_proc,"r+w");
	if (NULL != f) {
		fseek(f,0,0);
		fwrite(cam_time_buf,strlen(cam_time_buf),1,f);
		fclose(f);
	}
	writeCamInitTimeToApct(cam_time_buf);
}

bool getApctCamInitSupport()
{
	if (gIsApctRead) {
		return gIsApctCamInitTimeShow;
	}
	gIsApctRead = true;

	int ret = 0;
	char str[10] = {'\0'};
	const char *FILE_NAME = "/data/data/com.sprd.APCT/apct/apct_support";

	FILE *f = fopen(FILE_NAME, "r");

	if (NULL != f) {
		fseek(f, 0, 0);
		ret = fread(str, 5, 1, f);
		fclose(f);
		if(ret){
			long apct_config = atol(str);
			gIsApctCamInitTimeShow = (apct_config & 0x8010) == 0x8010 ? true : false;
		}
	}
	return gIsApctCamInitTimeShow;
}

int SprdCameraHardware::getCameraInfo(int cameraId, struct camera_info *cameraInfo)
{
	if (1 == getPropertyAtv()) {
		memcpy(cameraInfo, &kCameraInfo3[cameraId], sizeof(CameraInfo));
	} else {
		memcpy(cameraInfo, &kCameraInfo[cameraId], sizeof(CameraInfo));
	}
	return 0;
}

SprdCameraHardware::SprdCameraHardware(int cameraId)
	:
	mPreviewHeapSize(0),
	mPreviewHeapNum(0),
	mPreviewDcamAllocBufferCnt(0),
	mPreviewHeapArray(NULL),
	mRawHeap(NULL),
	mRawHeapSize(0),
	mSubRawHeapNum(0),
	mJpegHeapSize(0),
	mFDAddr(0),
	mMetadataHeap(NULL),
	mParameters(),
	mSetParameters(),
	mSetParametersBak(),
	mPreviewHeight_trimy(0),
	mPreviewWidth_trimx(0),
	mPreviewHeight_backup(0),
	mPreviewWidth_backup(0),
	mPreviewHeight(-1),
	mPreviewWidth(-1),
	mRawHeight(-1),
	mRawWidth(-1),
	mPreviewFormat(1),
	mPictureFormat(1),
	mPreviewStartFlag(0),
	mIsDvPreview(0),
	mRecordingMode(0),
	mBakParamFlag(0),
	mRecordingFirstFrameTime(0),
	mZoomLevel(0),
	mJpegSize(0),
	mNotify_cb(0),
	mData_cb(0),
	mData_cb_timestamp(0),
	mGetMemory_cb(0),
	mUser(0),
	mPreviewWindow(NULL),
	mMsgEnabled(0),
	mIsStoreMetaData(false),
	mIsFreqChanged(false),
	mCameraId(cameraId),
	miSPreviewFirstFrame(1),
	mCaptureMode(CAMERA_ZSL_MODE),
	mCaptureRawMode(0),
#ifdef CONFIG_CAMERA_ROTATION_CAPTURE
	mIsRotCapture(1),
#else
	mIsRotCapture(0),
#endif
	mFlashMask(false),
	mReleaseFLag(false),
	mTimeCoeff(1),
	mPreviewBufferUsage(PREVIEW_BUFFER_USAGE_GRAPHICS),
	mSetDDRFreqCount(0),
	mSetDDRFreq(NO_FREQ_REQ),
	mSwitchMonitorMsgQueHandle(0),
	mSwitchMonitorInited(0)
{
	LOGI("openCameraHardware: E cameraId: %d.", cameraId);

#if defined(CONFIG_BACK_CAMERA_ROTATION)
	if (0 == cameraId) {
		mPreviewBufferUsage = PREVIEW_BUFFER_USAGE_DCAM;
	}
#endif

#if defined(CONFIG_FRONT_CAMERA_ROTATION)
	if (1 == cameraId) {
		mPreviewBufferUsage = PREVIEW_BUFFER_USAGE_DCAM;
	}
#endif

	mOriginalPreviewBufferUsage = mPreviewBufferUsage;
	s_mem_method = MemoryHeapIon::Mm_iommu_is_enabled();
	memset(mPreviewHeapArray_phy, 0, sizeof(mPreviewHeapArray_phy));
	memset(mPreviewHeapArray_vir, 0, sizeof(mPreviewHeapArray_vir));
	memset(mSubRawHeapArray, 0, sizeof(mSubRawHeapArray));
	memset(mPreviewBufferHandle, 0, kPreviewBufferCount * sizeof(void*));
	memset(mPreviewCancelBufHandle, 0, kPreviewBufferCount * sizeof(void*));
	memset(mGraphBufferCount, 0, kPreviewBufferCount * sizeof(uint32_t));
	memset(&mPreviewHeapInfoBak, 0, sizeof(mPreviewHeapInfoBak));
	mPreviewHeapBakUseFlag = 0;
	memset(&mRawHeapInfoBak, 0, sizeof(mRawHeapInfoBak));
	mRawHeapBakUseFlag = 0;

	setCameraState(SPRD_INIT, STATE_CAMERA);

	if (!mGrallocHal) {
		int ret = hw_get_module(GRALLOC_HARDWARE_MODULE_ID, (const hw_module_t **)&mGrallocHal);
		if (ret)
			LOGE("ERR(%s):Fail on loading gralloc HAL", __func__);
	}

	if (1 == getPropertyAtv()) {
		mCameraId = 5;
	} else {
		mCameraId = cameraId;
	}

	initDefaultParameters();
	switch_monitor_thread_init((void *)this);

	LOGI("openCameraHardware: X cameraId: %d.", cameraId);
}

SprdCameraHardware::~SprdCameraHardware()
{
	LOGI("closeCameraHardware: E cameraId: %d.", mCameraId);
	if (!mReleaseFLag) {
		release();
	}
	LOGI("closeCameraHardware: X cameraId: %d.", mCameraId);
}

void SprdCameraHardware::release()
{
	LOGI("mLock:release E .\n");
	Mutex::Autolock l(&mLock);

	// Either preview was ongoing, or we are in the middle or taking a
	// picture.  It's the caller's responsibility to make sure the camera
	// is in the idle or init state before destroying this object.
	LOGI("release:camera state = %s, preview state = %s, capture state = %s",
		getCameraStateStr(getCameraState()), getCameraStateStr(getPreviewState()),
		getCameraStateStr(getCaptureState()));

	switch_monitor_thread_deinit((void *)this);

	if (isCapturing()) {
		cancelPictureInternal();
	}

	if (isPreviewing()) {
		stopPreviewInternal();
	}

	while (0 < mSetDDRFreqCount) {
		if (BAD_VALUE == set_ddr_freq(NO_FREQ_REQ)) {
			mSetDDRFreqCount = 0;
			LOGW("ddr set fail, quit yet!");
			break;
		}
	}

	if (isCameraInit()) {
		// When libqcamera detects an error, it calls camera_cb from the
		// call to camera_stop, which would cause a deadlock if we
		// held the mStateLock.  For this reason, we have an intermediate
		// state SPRD_INTERNAL_STOPPING, which we use to check to see if the
		// camera_cb was called inline.
		setCameraState(SPRD_INTERNAL_STOPPING, STATE_CAMERA);

		LOGI("stopping camera.");
		if (CAMERA_SUCCESS != camera_stop(camera_cb, this)) {
			setCameraState(SPRD_ERROR, STATE_CAMERA);
			if(NULL != mMetadataHeap){
				if(NULL != mMetadataHeap->release){
					LOGV("freePreviewMem start release mMetadataHeap");
					mMetadataHeap->release(mMetadataHeap);
					mMetadataHeap = NULL;
				}
			}
			mReleaseFLag = true;
			LOGE("release X: fail to camera_stop().");
			return;
		}

		WaitForCameraStop();
	}

	if(NULL != mMetadataHeap){
		if(NULL != mMetadataHeap->release){
			LOGV("freePreviewMem start release mMetadataHeap");
			mMetadataHeap->release(mMetadataHeap);
			mMetadataHeap = NULL;
		}
	}
	deinitCapture();

	mCbPrevDataBusyLock.lock();
	/*preview bak heap check and free*/
	if (false == mPreviewHeapInfoBak.busy_flag) {
		LOGI("release free prev heap bak mem");
		clearCameraMem(&mPreviewHeapInfoBak);
		memset(&mPreviewHeapInfoBak, 0, sizeof(mPreviewHeapInfoBak));
	} else {
		LOGE("release prev mem busy, this is unknown error!!!");
	}
	mPreviewHeapBakUseFlag = 0;
	mCbPrevDataBusyLock.unlock();

	mCbCapDataBusyLock.lock();
	/* capture head check and free*/
	if (false == mRawHeapInfoBak.busy_flag) {
		LOGI("release free raw heap bak mem");
		clearCameraMem(&mRawHeapInfoBak);
		memset(&mRawHeapInfoBak, 0, sizeof(mRawHeapInfoBak));
	} else {
		LOGE("release cap mem busy, this is unknown error!!!");
	}
	mRawHeapBakUseFlag = 0;
	mCbCapDataBusyLock.unlock();
	mReleaseFLag = true;
	LOGI("mLock:release X.\n");
}

int SprdCameraHardware::getCameraId() const
{
	return mCameraId;
}

status_t SprdCameraHardware::startPreview()
{
	LOGI("mlock:startPreview: E");
	status_t ret = NO_ERROR;
	Mutex::Autolock l(&mLock);

	waitSetParamsOK();

	setCaptureRawMode(0);

	bool isRecordingMode = (mMsgEnabled & CAMERA_MSG_VIDEO_FRAME) > 0 ? true : false;
	ret = startPreviewInternal(isRecordingMode);

	LOGI("mlock:startPreview: X");
	return ret;
}

void SprdCameraHardware::stopPreview()
{
	LOGI("mlock:stopPreview: E");
	Mutex::Autolock l(&mLock);

	waitSetParamsOK();

	if(NULL == mParameters.get("video-size-values")) {
		if (isRecordingMode()) {
			LOGW("mLock: stopPreview: X. can't stop preview in recording for no-video-size case! direct return!");
			return;
		}
	}

	stopPreviewInternal();
	LOGI("mlock:stopPreview: X");
}

bool SprdCameraHardware::previewEnabled()
{
	bool ret = 0;
	LOGI("mLock:previewEnabled E.\n");
	Mutex::Autolock l(&mLock);
	LOGI("mLock:previewEnabled X.\n");
	return isPreviewing();
}

status_t SprdCameraHardware::setPreviewWindow(preview_stream_ops *w)
{
	int min_bufs;
	bool switch_ret = false;
	status_t ret = 0;

	LOGI("setPreviewWindow E");
	Mutex::Autolock l(&mParamLock);

	LOGI("%s: mPreviewWindow %p target window %p", __func__, mPreviewWindow, w);

	if (!w) {
		switch_ret = switchBufferMode(mPreviewBufferUsage, PREVIEW_BUFFER_USAGE_DCAM);
		if (switch_ret) {
			ret = NO_ERROR;
		} else {
			ret = INVALID_OPERATION;
		}

		LOGE("setPreviewWindow X preview window is NULL!");

		mPreviewWindowLock.lock();
		mPreviewWindow = w;
		mPreviewWindowLock.unlock();

		return ret;
	} else {
		switch_ret = switchBufferMode(mPreviewBufferUsage, mOriginalPreviewBufferUsage);
		if (!switch_ret) {
			ret = INVALID_OPERATION;
			LOGE("switch buffer error!");
			mPreviewWindowLock.lock();
			mPreviewWindow = w;
			mPreviewWindowLock.unlock();
			return ret;
		}
	}

	mPreviewWindowLock.lock();


	if (w->get_min_undequeued_buffer_count(w, &min_bufs)) {
		LOGE("%s: could not retrieve min undequeued buffer count X", __func__);
		mPreviewWindowLock.unlock();
		return INVALID_OPERATION;
	}

	if (min_bufs >= kPreviewBufferCount) {
		LOGE("%s: min undequeued buffer count %d is too high (expecting at most %d)", __func__,
		min_bufs, kPreviewBufferCount - 1);
	}

	LOGI("%s: setting buffer count to %d", __func__, kPreviewBufferCount);
	if (w->set_buffer_count(w, kPreviewBufferCount)) {
		LOGE("%s: could not set buffer count X", __func__);
		mPreviewWindowLock.unlock();
		return INVALID_OPERATION;
	}
	mPreviewWindow = w;

	mPreviewWindowLock.unlock();

	int preview_width;
	int preview_height;
	mParameters.getPreviewSize(&preview_width, &preview_height);
	preview_width = SIZE_ALIGN(preview_width);
	LOGI("%s: preview size: %dx%d.", __func__, preview_width, preview_height);

#if CAM_OUT_YUV420_UV
	int hal_pixel_format = HAL_PIXEL_FORMAT_YCbCr_420_SP;
#else
	int hal_pixel_format = HAL_PIXEL_FORMAT_YCrCb_420_SP;
#endif

	const char *str_preview_format = mParameters.getPreviewFormat();
	int usage;

	LOGI("%s: preview format %s", __func__, str_preview_format);

	if (preview_width < 640) {
		switch_ret = switchBufferMode(mPreviewBufferUsage, PREVIEW_BUFFER_USAGE_DCAM);
	} else {
		switch_ret = switchBufferMode(mPreviewBufferUsage, mOriginalPreviewBufferUsage);
	}
	if (!switch_ret) {
		ret = INVALID_OPERATION;
		LOGE("setPreviewWindow X buffer switch error");
		return ret;
	}

#ifdef CONFIG_CAMERA_DMA_COPY
	usage = GRALLOC_USAGE_SW_WRITE_OFTEN | GRALLOC_USAGE_PRIVATE_0;
#else
	if (PREVIEW_BUFFER_USAGE_DCAM == mPreviewBufferUsage) {
		usage = GRALLOC_USAGE_SW_WRITE_OFTEN;
	} else {
		usage = GRALLOC_USAGE_SW_WRITE_OFTEN | GRALLOC_USAGE_CAMERA_BUFFER;
	}
#endif

	if (w->set_usage(w, usage )) {
		LOGE("%s: could not set usage on gralloc buffer X", __func__);
		return INVALID_OPERATION;
	}

	if (mParameters.getRecordingHint()) {
		mIsDvPreview = 1;
		if (w->set_buffers_geometry(w,
			SIZE_ALIGN(preview_width), SIZE_ALIGN(preview_height),
			hal_pixel_format)) {
			LOGE("%s: could not set buffers geometry to %s X",
				__func__, str_preview_format);
			return INVALID_OPERATION;
		}
	} else {
		mIsDvPreview = 0;
		if (w->set_buffers_geometry(w,
			preview_width, preview_height,
			hal_pixel_format)) {
			LOGE("%s: could not set buffers geometry to %s X",
				__func__, str_preview_format);
			return INVALID_OPERATION;
		}
	}

	if (w->set_crop(w, 0, 0, preview_width-1, preview_height-1)) {
		LOGE("%s: could not set crop to %s X",
			__func__, str_preview_format);
		return INVALID_OPERATION;
	}

	return NO_ERROR;
}

status_t SprdCameraHardware::takePicture()
{
	GET_START_TIME;
	LOGI("takePicture: E");
	print_time();

	Mutex::Autolock l(&mLock);
	print_time();
	takepicture_mode mode = getCaptureMode();
	LOGI("takePicture mode %d", mode);

	if (SPRD_ERROR == mCameraState.capture_state) {
		LOGE("takePicture in error status, deinit capture at first ");
		deinitCapture();
	} else if (SPRD_IDLE != mCameraState.capture_state) {
		LOGE("take picture: action alread exist, direct return!");
		return ALREADY_EXISTS;
	}

	waitSetParamsOK();

	if (camera_set_dimensions(mRawWidth,
				mRawHeight,
				mPreviewWidth,
				mPreviewHeight,
				NULL,
				NULL,
				mCaptureMode != CAMERA_RAW_MODE)) {
		LOGE("take picture parameters error!");
		return UNKNOWN_ERROR;
	}

	if (!iSZslMode()) {
		if (isPreviewing()) {
			LOGI("call stopPreviewInternal in takePicture().");
			stopPreviewInternal();
		}
		LOGI("ok to stopPreviewInternal in takePicture. preview state = %d", getPreviewState());

		if (isPreviewing()) {
			LOGE("takePicture: stop preview error!, preview state = %d", getPreviewState());
			return UNKNOWN_ERROR;
		}
		if (!initCapture(mData_cb != NULL)) {
			deinitCapture();
			LOGE("takePicture initCapture failed. Not taking picture.");
			return UNKNOWN_ERROR;
		}
	}

	if (isCapturing()) {
		WaitForCaptureDone();
	}

	set_ddr_freq(HIGH_FREQ_REQ);

	setCameraState(SPRD_INTERNAL_RAW_REQUESTED, STATE_CAPTURE);
	LOGI("INTERPOLATION::takePicture:mRawWidth=%d,mZoomLevel=%d",mRawWidth,mZoomLevel);
	if (CAMERA_SUCCESS != camera_take_picture(camera_cb, this, mode)) {
		setCameraState(SPRD_ERROR, STATE_CAPTURE);
		LOGE("takePicture: fail to camera_take_picture.");
		return UNKNOWN_ERROR;
	}

	bool result = WaitForCaptureStart();

	print_time();

	LOGI("takePicture: X");

	return result ? NO_ERROR : UNKNOWN_ERROR;
}

status_t SprdCameraHardware::cancelPicture()
{
	Mutex::Autolock l(&mLock);

	return cancelPictureInternal();
}

status_t SprdCameraHardware::setTakePictureSize(uint32_t width, uint32_t height)
{
	mRawWidth = width;
	mRawHeight = height;

	return NO_ERROR;
}

status_t SprdCameraHardware::waitSetParamsOK()
{
	status_t ret = NO_ERROR;
	uint32_t i_count = 0;
	LOGI("waitSetParamsOK E.\n");

	while (SPRD_IDLE != getSetParamsState() || mBakParamFlag) {
		usleep(10*1000);
		if (i_count < SET_PARAMS_TIMEOUT) {
			i_count++;
		} else {
			LOGE("waitSetParamsOK timeout to wait set Parameter OK, skip that!");
			ret = TIMED_OUT;
			break;
		}
	}

	LOGI("waitSetParamsOK X.\n");
	return ret;
}

status_t SprdCameraHardware::startRecording()
{
	status_t ret = NO_ERROR;
	char * isZslSupport = (char *)mParameters.get("zsl-supported");

	LOGI("mLock:startRecording E.\n");
	Mutex::Autolock l(&mLock);
	mRecordingFirstFrameTime = 0;

	waitSetParamsOK();
	camera_set_preview_trace(1);

	if (isPreviewing()) {
		if (camera_is_need_stop_preview()
			|| (isZslSupport
			&& (0 == strcmp("true", isZslSupport))
			&& (1 != mParameters.getInt("zsl")))) {
			LOGI("call stopPreviewInternal in startRecording().");
			setCameraState(SPRD_INTERNAL_PREVIEW_STOPPING, STATE_PREVIEW);
			if (CAMERA_SUCCESS != camera_stop_preview()) {
				setCameraState(SPRD_ERROR, STATE_PREVIEW);
				freePreviewMem();
				LOGE("startRecording X: fail to camera_stop_preview().");
				return INVALID_OPERATION;
			}

			WaitForPreviewStop();

			LOGI("startRecording: Freeing preview heap.");
			freePreviewMem();
		}
	}
	ret = startPreviewInternal(true);
	LOGI("mLock:startRecording X.\n");

	return ret;
}

void SprdCameraHardware::stopRecording()
{
	char * isZslSupport = (char *)mParameters.get("zsl-supported");
	LOGI("stopRecording: E");
	Mutex::Autolock l(&mLock);
	setRecordingMode(false);
	mRecordingFirstFrameTime = 0;
	camera_set_preview_trace(0);
	LOGI("stopRecording: X");
}

void SprdCameraHardware::releaseRecordingFrame(const void *opaque)
{
	LOGI("releaseRecordingFrame E. ");
	uint8_t *addr = (uint8_t *)opaque;
	uint32_t index = (addr - (uint8_t *)mMetadataHeap->data) / (METADATA_SIZE);

	Mutex::Autolock pbl(&mPrevBufLock);

	if (!isPreviewing()) {
		LOGE("releaseRecordingFrame: Preview not in progress!");
		return;
	}

	if (PREVIEW_BUFFER_USAGE_DCAM == mPreviewBufferUsage) {
		//Mutex::Autolock l(&mLock);
		uint32_t *vaddr = NULL;
		uint32_t *paddr = NULL;

		if (mIsStoreMetaData) {
			paddr = (uint32_t *) *((uint32_t*)addr + 1);
			vaddr = (uint32_t *) *((uint32_t*)addr + 2);

			if (camera_get_rot_set()) {
				index += kPreviewBufferCount;
			}

		} else {
			for (index=0; index < mPreviewHeapNum; index++) {
				if ((uint32_t)addr == mPreviewHeapArray_vir[index])
					break;
			}

			if (index < mPreviewHeapNum) {
				vaddr = (uint32_t*)mPreviewHeapArray_vir[index];
				paddr = (uint32_t*)mPreviewHeapArray_phy[index];
			}
		}

		if (index >= mPreviewHeapNum) {
			LOGE("releaseRecordingFrame error: index: %d, data: %x, w=%d, h=%d \n",
			index, (uint32_t)addr, mPreviewWidth, mPreviewHeight);
			return;
		}

		flush_buffer(CAMERA_FLUSH_PREVIEW_HEAP, index,
				(void*)vaddr,
				(void*)paddr,
				(int)mPreviewHeapSize);

		camera_release_frame(index);
	} else {
		Mutex::Autolock pgbcl(&mGraphBufCntLock);
		if (isRecordingMode()) {
			mGraphBufferCount[index]--;
			LOGV("releasePreviewFrame 0x%x count %d", index, mGraphBufferCount[index]);
			if (0 == mGraphBufferCount[index]) {
				if (CAMERA_SUCCESS != camera_release_frame(index)) {
					goto EXIT;
				}
			}
		}
	}
EXIT:
	LOGI("releaseRecordingFrame X: index: %d", index);
}

bool SprdCameraHardware::recordingEnabled()
{
	LOGI("recordingEnabled: E");
	Mutex::Autolock l(&mLock);
	LOGI("recordingEnabled: X");

	return isPreviewing() && (mMsgEnabled & CAMERA_MSG_VIDEO_FRAME);
}

status_t SprdCameraHardware::autoFocus()
{
	LOGI("mLock:autoFocus E.\n");
	Mutex::Autolock l(&mLock);

	if (!isPreviewing()) {
		LOGE("autoFocus: not previewing");
		return INVALID_OPERATION;
	}

	if (SPRD_IDLE != getFocusState()) {
		LOGE("autoFocus existing, direct return!");
		return NO_ERROR;
	}

	waitSetParamsOK();

	mMsgEnabled |= CAMERA_MSG_FOCUS;

	setCameraState(SPRD_FOCUS_IN_PROGRESS, STATE_FOCUS);
	if (0 != camera_start_autofocus(CAMERA_AUTO_FOCUS, camera_cb, this)) {
		LOGE("auto foucs fail.");
		setCameraState(SPRD_IDLE, STATE_FOCUS);
	}

	LOGI("mLock:autoFocus X.\n");
	return NO_ERROR;
}

status_t SprdCameraHardware::cancelAutoFocus()
{
	bool ret = 0;
	LOGI("mLock:CancelFocus E.\n");
	Mutex::Autolock l(&mLock);
	mMsgEnabled &= ~CAMERA_MSG_FOCUS;
	ret = camera_cancel_autofocus();

	WaitForFocusCancelDone();
	LOGI("mLock:CancelFocus X.\n");
	return ret;
}

void SprdCameraHardware::setCaptureRawMode(bool mode)
{
	mCaptureRawMode = mode;
	LOGI("ISP_TOOL: setCaptureRawMode: %d, %d", mode, mCaptureRawMode);
}

void SprdCameraHardware::antiShakeParamSetup( )
{
#ifdef CONFIG_CAMERA_ANTI_SHAKE
	mPreviewWidth = mPreviewWidth_backup + ((((mPreviewWidth_backup/ 10) + 15) >> 4) << 4);
	mPreviewHeight = mPreviewHeight_backup + ((((mPreviewHeight_backup/ 10) + 15) >> 4) << 4);
#endif
}

status_t SprdCameraHardware::checkSetParametersEnvironment( )
{
	status_t ret =  NO_ERROR;
	/*check capture status*/
	if (isCapturing()) {
		LOGE("warning, camera HAL in capturing process, abnormal calling sequence!");
		return PERMISSION_DENIED;
	}

	/*check preview status*/
	if ((SPRD_IDLE != getPreviewState()) && (SPRD_PREVIEW_IN_PROGRESS != getPreviewState())) {
		LOGE("camera HAL in preview changing process, not allow to setParameter");
		return PERMISSION_DENIED;
	}

	return ret;
}

status_t SprdCameraHardware::copyParameters(SprdCameraParameters& cur_params, const SprdCameraParameters& params)
{
	status_t ret =  NO_ERROR;

	int width = 0, height = 0;
	int rawWidth = 0, rawHeight = 0;

	params.getPreviewSize(&width, &height);
	cur_params.setPreviewSize(width, height);

	params.getPictureSize(&rawWidth, &rawHeight);
	cur_params.setPictureSize(rawWidth, rawHeight);
	LOGV("copyParameters:requested picture size %d x %d", rawWidth, rawHeight);

	params.getVideoSize(&width, &height);
	cur_params.setVideoSize(width, height);

	{
	const char* new_previewformat = params.getPreviewFormat();
	if (new_previewformat)
		cur_params.setPreviewFormat(new_previewformat);
	}

	{
	const char* new_pictureformat = params.getPictureFormat();
	if (new_pictureformat)
		cur_params.setPictureFormat(new_pictureformat);
	}

	/*ExposureCompensationStep*/
	{
	const char* new_ExposureCompensationStep = params.get_ExposureCompensationStep();
	if (new_ExposureCompensationStep)
		cur_params.setExposureCompensationStep(new_ExposureCompensationStep);
	}
	/*MaxExposureCompensation*/
	{
	const char* new_MaxExposureCompensation = params.get_MaxExposureCompensation();
	if(new_MaxExposureCompensation)
		cur_params.setMaxExposureCompensation(new_MaxExposureCompensation);
	}

	/*MinExposureCompensation*/
	{
	const char* new_MinExposureCompensation = params.get_MinExposureCompensation();
	if (new_MinExposureCompensation)
		cur_params.setMinExposureCompensation(new_MinExposureCompensation);
	}

	/*SupportedSceneModes*/
	{
	const char* new_SupportedSceneModes = params.get_SupportedSceneModes();
	if (new_SupportedSceneModes)
		cur_params.setSupportedSceneModes(new_SupportedSceneModes);
	}

	/*SupportedPreviewSizes*/
	{
	const char* new_SupportedPreviewSizes = params.get_SupportedPreviewSizes();
	if(new_SupportedPreviewSizes)
		cur_params.setSupportedPreviewSizes(new_SupportedPreviewSizes);
	}

	/*SupportedPreviewFrameRate*/
	{
	const char* new_SupportedPreviewFrameRate = params.get_SupportedPreviewFrameRate();
	if (new_SupportedPreviewFrameRate)
		cur_params.setSupportedPreviewFrameRate(new_SupportedPreviewFrameRate);
	}

	/*SupportedPreviewFpsRange*/
	{
	const char* new_SupportedPreviewFpsRange = params.get_SupportedPreviewFpsRange();
	if (new_SupportedPreviewFpsRange)
		cur_params.setSupportedPreviewFpsRange(new_SupportedPreviewFpsRange);
	}

	/*SupportedPictureSizes*/
	{
	const char* new_SupportedPictureSizes = params.get_SupportedPictureSizes();
	if (new_SupportedPictureSizes)
		cur_params.setSupportedPictureSizes(new_SupportedPictureSizes);
	}

	/*AutoExposureLock*/
	{
	const char* new_AutoExposureLock = params.get_AutoExposureLock();
	if(new_AutoExposureLock)
		cur_params.setAutoExposureLock(new_AutoExposureLock);
	}

	/*AutoExposureLockSupported*/
	{
	const char* new_AutoExposureLockSupported = params.get_AutoExposureLockSupported();
	if (new_AutoExposureLockSupported)
		cur_params.setAutoExposureLockSupported(new_AutoExposureLockSupported);
	}

	/*AutoWhiteBalanceLock*/
	{
	const char* new_AutoWhiteBalanceLock = params.get_AutoWhiteBalanceLock();
	if (new_AutoWhiteBalanceLock)
		cur_params.setAutoWhiteBalanceLock(new_AutoWhiteBalanceLock);
	}

	/*AutoWhiteBalanceLockSupported*/
	{
	const char* new_AutoWhiteBalanceLockSupported = params.get_AutoWhiteBalanceLockSupported();
	if (new_AutoWhiteBalanceLockSupported)
		cur_params.setAutoWhiteBalanceLockSupported(new_AutoWhiteBalanceLockSupported);
	}

	/*HorizontalViewAngle*/
	{
	const char* new_HorizontalViewAngle = params.get_HorizontalViewAngle();
	if(new_HorizontalViewAngle)
		cur_params.setHorizontalViewAngle(new_HorizontalViewAngle);
	}

	/*VerticalViewAngle*/
	{
	const char* new_VerticalViewAngle = params.get_VerticalViewAngle();
	if (new_VerticalViewAngle)
		cur_params.setVerticalViewAngle(new_VerticalViewAngle);
	}

	/*VideoFrameFormat*/
	{
	const char* new_VideoFrameFormat = params.get_VideoFrameFormat();
	if (new_VideoFrameFormat)
		cur_params.setVideoFrameFormat(new_VideoFrameFormat);
	}

	/*SupportedVideoSizes*/
	{
	const char* new_SupportedVideoSizes = params.get_SupportedVideoSizes();
	if (new_SupportedVideoSizes)
		cur_params.setSupportedVideoSizes(new_SupportedVideoSizes);
	}

	/*PreviewFpsRange*/
	{
	const char* new_PreviewFpsRange = params.get_PreviewFpsRange();
	if (new_PreviewFpsRange)
		cur_params.setPreviewFpsRange(new_PreviewFpsRange);
	}

	/*FocusAreas*/
	{
	const char* new_focus_areas = params.get_FocusAreas();
	if (new_focus_areas)
		cur_params.setFocusAreas(new_focus_areas);
	}

	/*MeteringAreas*/
	{
	const char* new_metering_areas = params.get_MeteringAreas();
	if (new_metering_areas)
		cur_params.setMeteringAreas(new_metering_areas);
	}

	/*FocalLength*/
	{
	const char* new_FocalLength = params.get_FocalLength();
	if (new_FocalLength)
		cur_params.setFocalLength(new_FocalLength);
	}

	/*FocusMode*/
	{
	const char* new_focus_mode = params.get_FocusMode();
	if(new_focus_mode)
		cur_params.setFocusMode(new_focus_mode);
	}

	/*SupportedFocusModes*/
	{
	const char* new_SupportedFocusModes = params.get_SupportedFocusModes();
	if (new_SupportedFocusModes)
		cur_params.setSupportedFocusModes(new_SupportedFocusModes);
	}

	/*WhiteBalance*/
	{
	const char* new_whitebalance = params.get_WhiteBalance();
	if (new_whitebalance)
		cur_params.setWhiteBalance(new_whitebalance);
	}

	/*CameraId*/
	{
	const char* new_camera_id = params.get_CameraId();
	if (new_camera_id)
		cur_params.setCameraId(new_camera_id);
	}

	/*JpegQuality*/
	{
	const char* new_jpeg_quality = params.get_JpegQuality();
	if (new_jpeg_quality)
		cur_params.setJpegQuality(new_jpeg_quality);
	}

	/*JpegThumbnailQuality*/
	{
	const char* new_thumbnail_quality = params.get_JpegThumbnailQuality();
	if (new_thumbnail_quality)
		cur_params.setJpegThumbnailQuality(new_thumbnail_quality);
	}

	/*jpegThumbnail Dimension*/
	{
	const char* new_thumbnail_size = params.get_JpegThumbnailSize();
	const char* new_thumbnail_width = params.get_JpegThumbnailWidth();
	const char* new_thumbnail_height = params.get_JpegThumbnailHeight();
	if (new_thumbnail_size)
		cur_params.setJpegThumbnailSize(new_thumbnail_size);
	if (new_thumbnail_width)
		cur_params.setJpegThumbnailWidth(new_thumbnail_width);
	if (new_thumbnail_height)
		cur_params.setJpegThumbnailHeight(new_thumbnail_height);
	}

	/*jpegThumbnail Dimesion Value*/
	{
	const char* new_thumbnail_size_val = params.get_JpegThumbnailSizeValue();
	if (new_thumbnail_size_val)
		cur_params.setJpegThumbnailSizeValue(new_thumbnail_size_val);
	}

	/*rotation*/
	{
	const char*  new_Rotation = params.get_Rotation();
	if (new_Rotation)
		cur_params.setRotation(new_Rotation);
	}

	/*GpsLatitude*/
	{
	const char*  new_GpsLatitude = params.get_GpsLatitude();
	if (new_GpsLatitude) {
		cur_params.setGpsLatitude(new_GpsLatitude);
	} else {
		cur_params.removeGpsLatitude();
	}
	}

	/*GpsLongitude*/
	{
	const char*  new_GpsLongitude = params.get_GpsLongitude();
	if (new_GpsLongitude) {
		cur_params.setGpsLongitude(new_GpsLongitude);
	} else {
		cur_params.removeGpsLongitude();
	}
	}

	/*GpsAltitude*/
	{
	const char*  new_GpsAltitude = params.get_GpsAltitude();
	if (new_GpsAltitude) {
		cur_params.setGpsAltitude(new_GpsAltitude);
	} else {
		cur_params.removeGpsAltitude();
	}
	}

	/*GpsTimestamp*/
	{
	const char*  new_GpsTimestamp = params.get_GpsTimestamp();
	if (new_GpsTimestamp) {
		cur_params.setGpsTimestamp(new_GpsTimestamp);
	} else {
		cur_params.removeGpsTimestamp();
	}
	}

	/*GPS_Processing_Method*/
	{
	const char* new_GPSProcessingMethod = params.get_GPS_Processing_Method();
	if (new_GPSProcessingMethod) {
		cur_params.setGPSProcessingMethod(new_GPSProcessingMethod);
	} else {
		cur_params.removeGPSProcessingMethod();
	}
	}
	/*MaxNumDetectedFacesHW*/
	{
	const char*  new_MaxNumDetectedFacesHW = params.get_MaxNumDetectedFacesHW();
	if (new_MaxNumDetectedFacesHW)
		cur_params.setMaxNumDetectedFacesHW(new_MaxNumDetectedFacesHW);
	}

	/*MaxNumDetectedFacesSW*/
	{
	const char*  new_MaxNumDetectedFacesSW = params.get_MaxNumDetectedFacesSW();
	if (new_MaxNumDetectedFacesSW)
		cur_params.setMaxNumDetectedFacesSW(new_MaxNumDetectedFacesSW);
	}

	/*Effect*/
	{
	const char* new_effect = params.get_Effect();
	if (new_effect)
		cur_params.setEffect(new_effect);
	}

	/*SceneMode*/
	{
	const char* new_scene_mode = params.get_SceneMode();
	if (new_scene_mode)
		cur_params.setSceneMode(new_scene_mode);
	}

	/*SupportedAntibanding*/
	{
	const char* new_SupportedAntibanding = params.get_SupportedAntibanding();
	if (new_SupportedAntibanding)
		cur_params.setSupportedAntibanding(new_SupportedAntibanding);
	}

	/*SupportedSupportedEffects*/
	{
	const char* new_SupportedEffects = params.get_SupportedEffects();
	if (new_SupportedEffects)
		cur_params.setSupportedEffects(new_SupportedEffects);
	}

	/*SupportedSharpness*/
	{
	const char* new_SupportedSharpness = params.get_SupportedSharpness();
	if (new_SupportedSharpness)
		cur_params.setSupportedSharpness(new_SupportedSharpness);
	}

	/*PictureFormat*/
	{
	const char* new_PictureFormat = params.get_PictureFormat();
	if (new_PictureFormat)
		cur_params.setPictureFormat(new_PictureFormat);
	}

	/*SupportedPictureFormat*/
	{
	const char* new_SupportedPictureFormat = params.get_SupportedPictureFormat();
	if (new_SupportedPictureFormat)
		cur_params.setSupportedPictureFormat(new_SupportedPictureFormat);
	}

	/*Zoom*/
	{
	const char* new_zoom = params.get_Zoom();
	if (new_zoom)
		cur_params.setZoom(new_zoom);
	}

	/*MaxZoom*/
	{
	const char* new_maxzoom = params.get_MaxZoom();
	if (new_maxzoom)
		cur_params.setMaxZoom(new_maxzoom);
	}

	/*ZoomRatios*/
	{
	const char* new_zoomratios = params.get_ZoomRatios();
	if (new_zoomratios)
		cur_params.setZoomRatios(new_zoomratios);
	}

	/*ZoomSupported*/
	{
	const char* new_zoomsupported = params.get_ZoomSupported();
	if (new_zoomsupported)
		cur_params.setZoomSupported(new_zoomsupported);
	}

	/*SmoothZoomSupported*/
	{
	const char* new_SmoothZoomSupported = params.get_SmoothZoomSupported();
	if (new_SmoothZoomSupported)
		cur_params.setSmoothZoomSupported(new_SmoothZoomSupported);
	}

	/*SupportedFlashMode*/
	{
	const char* new_SupportedFlashMode = params.get_SupportedFlashMode();
	if (new_SupportedFlashMode)
		cur_params.setSupportedFlashMode(new_SupportedFlashMode);
	}

	/*SupportedWhiteBalance*/
	{
	const char* new_SupportedWhiteBalance = params.get_SupportedWhiteBalance();
	if (new_SupportedWhiteBalance)
		cur_params.setSupportedWhiteBalance(new_SupportedWhiteBalance);
	}

	/*SupportedIso*/
	{
	const char* new_SupportedIso = params.get_SupportedIso();
	if (new_SupportedIso)
		cur_params.setSupportedIso(new_SupportedIso);
	}

	/*SupportedContrast*/
	{
	const char* new_SupportedContrast = params.get_SupportedContrast();
	if (new_SupportedContrast)
		cur_params.setSupportedContrast(new_SupportedContrast);
	}

	/*SupportedSaturation*/
	{
	const char* new_SupportedSaturation = params.get_SupportedSaturation();
	if (new_SupportedSaturation)
		cur_params.setSupportedSaturation(new_SupportedSaturation);
	}

	/*SupportedBrightness*/
	{
	const char* new_SupportedBrightness = params.get_SupportedBrightness();
	if (new_SupportedBrightness)
		cur_params.setSupportedBrightness(new_SupportedBrightness);
	}

	/*Brightness*/
	{
	const char* new_brightness = params.get_Brightness();
	if (new_brightness)
		cur_params.setBrightness(new_brightness);
	}

	/*Sharpness*/
	{
	const char* new_sharpness = params.get_Sharpness();
	if (new_sharpness)
		cur_params.setSharpness(new_sharpness);
	}

	/*Contrast*/
	{
	const char* new_contrast = params.get_Contrast();
	if (new_contrast)
		cur_params.setContrast(new_contrast);
	}

	/*Saturation*/
	{
	const char* new_saturation = params.get_Saturation();
	if (new_saturation)
		cur_params.setSaturation(new_saturation);
	}

	/*ExposureCompensation*/
	{
	const char* new_exposure_Compensation = params.get_ExposureCompensation();
	if (new_exposure_Compensation)
		cur_params.setExposureCompensation(new_exposure_Compensation);
	}

	/*AntiBanding*/
	{
	const char* new_antiBanding = params.get_AntiBanding();
	if (new_antiBanding)
		cur_params.setAntiBanding(new_antiBanding);
	}

	/*Iso*/
	{
	const char* new_iso = params.get_Iso();
	if (new_iso)
		cur_params.setIso(new_iso);
	}

	/*RecordingHint*/
	{
	const char* new_recordingHint = params.get_RecordingHint();
	if (new_recordingHint)
		cur_params.setRecordingHint(new_recordingHint);
	}

	/*FlashMode*/
	{
	const char* new_FlashMode = params.get_FlashMode();
	if (new_FlashMode)
		cur_params.setFlashMode(new_FlashMode);
	}

	/*Slowmotion*/
	{
	const char* new_slowmotion = params.get_Slowmotion();
	if (new_slowmotion)
		cur_params.setSlowmotion(new_slowmotion);
	}
	/*SlowmotionSupported*/
	{
	const char* new_slowmotionsupported = params.get_SlowmotionSupported();
	if (new_slowmotionsupported )
		cur_params.setSlowmotionSupported(new_slowmotionsupported );
	}
	/*PreviewEnv*/
	{
	const char* new_PreviewEnv = params.get_PreviewEnv();
	if (new_PreviewEnv)
		cur_params.setPreviewEnv(new_PreviewEnv);
	}

	/*PreviewFameRate*/
	{
	const char* new_PreviewFameRate = params.get_PreviewFameRate();
	if (new_PreviewFameRate)
		cur_params.setPreviewFameRate(new_PreviewFameRate);
	}

	/*AutoExposureMode*/
	{
	const char* new_AutoExposureMode = params.get_AutoExposureMode();
	if (new_AutoExposureMode)
		cur_params.setAutoExposureMode(new_AutoExposureMode);
	}
	/*VideoStabilition*/
	{
	const char* new_VideoStabilition = params.get_VideoStabilition();
	if (new_VideoStabilition)
		cur_params.setVideoStabilition(new_VideoStabilition);
	}
	/*VideoStabilitionSupported*/
	{
	const char* new_VideoStabilitionSupported = params.get_VideoStabilitionSupported();
	if (new_VideoStabilitionSupported)
		cur_params.setVideoStabilitionSupported(new_VideoStabilitionSupported);
	}
	/*FocusDistances*/
	{
	const char* new_FocusDistances = params.get_FocusDistances();
	if (new_FocusDistances)
		cur_params.setFocusDistances(new_FocusDistances);
	}
	/*MaxNumFocusAreas*/
	{
	const char* new_MaxNumFocusAreas = params.get_MaxNumFocusAreas();
	if (new_MaxNumFocusAreas)
		cur_params.setMaxNumFocusAreas(new_MaxNumFocusAreas);
	}
	/*isZslSupport*/
	{
	const char* new_isZslSupport = (char *)params.get("zsl-supported");
	if (new_isZslSupport)
		cur_params.setZSLSupport(new_isZslSupport);
	}
	/*isFlashModeSupport*/
	{
	const char* new_isFlashModeSupport = (char *)params.get("flash-mode-supported");
	if (new_isFlashModeSupport)
		cur_params.setFlashModeSupport(new_isFlashModeSupport);
	}
	/*SupportedPreviewFormat*/
	{
	const char* new_SupportedPreviewFormat = params.get_SupportedPreviewFormat();
	if (new_SupportedPreviewFormat)
		cur_params.setSupportedPreviewFormat(new_SupportedPreviewFormat);
	}
	/*VideoSnapshotSupported*/
	{
	const char* new_VideoSnapshotSupported = params.get_VideoSnapshotSupported();
	if (new_VideoSnapshotSupported)
		cur_params.setVideoSnapshotSupported(new_VideoSnapshotSupported);
	}

	/*max metering areas number*/
	{
	const char* new_maxNumMeteringArea = params.get_maxNumMeteringArea();
	if (new_maxNumMeteringArea)
		cur_params.setMaxNumMeteringArea(new_maxNumMeteringArea);
	}

	/*the below parameters should be acquired by getInt*/
	/*sensor rotation (sensor rotation should be worked with sensor orientation)*/
	{
	int newRotation = params.getInt("sensorrotation");
	if (-1 != newRotation)
		cur_params.setSensorRotation(newRotation);
	}

	/*sensor orientation (sensor orientation should be worked with sensor rotation)*/
	{
	int newOrientation = params.getInt("sensororientation");
	if (-1 != newOrientation)
		cur_params.setSensorOrientation(newOrientation);
	}

	/*ZSL mode set*/
	{
	int newZslVal = params.getInt("zsl");
	if (-1 != newZslVal)
		cur_params.setZsl(newZslVal);
	}

	/*continuos mode set*/
	{
	int newCapMode = params.getInt("capture-mode");
	if (-1 != newCapMode)
		cur_params.setCapMode(newCapMode);
	}

	return ret;
}

status_t SprdCameraHardware::checkSetParameters(const SprdCameraParameters& params, const SprdCameraParameters& oriParams)
{
	status_t ret =  NO_ERROR;
	int32_t tmpSize = sizeof(SprdCameraParameters);

	ret = memcmp(&params, &oriParams, tmpSize);

	return ret;
}

status_t SprdCameraHardware::checkSetParameters(const SprdCameraParameters& params)
{
	int min,max;
	int max_focus_num = -1;
	int max_metering_num = -1;
	const char* flash_mode;
	const char* focus_mode;

	params.getPreviewFpsRange(&min, &max);
	if ((min > max) || (min < 0) || (max < 0)) {
		LOGE("Error to FPS range: min: %d, max: %d.", min, max);
		return UNKNOWN_ERROR;
	}

	max_focus_num = params.getInt("max-num-focus-areas");
	if (params.chekFocusAreas(max_focus_num)) {
		return BAD_VALUE;
	}

	if (NULL != params.get("max-num-metering-areas")) {
		max_metering_num = params.getInt("max-num-metering-areas");
	} else {
		max_metering_num = mParameters.getInt("max-num-metering-areas");
	}

	if (params.chekMeteringAreas(max_metering_num)) {
		return BAD_VALUE;
	}

	if (((SprdCameraParameters)params).getZoom() > params.getInt("max-zoom")) {
		return BAD_VALUE;
	}

	checkFlashParameter((SprdCameraParameters&)params);

	flash_mode = ((SprdCameraParameters)params).get_FlashMode();
	LOGI("flash-mode:%s.",flash_mode);
	if (!flash_mode) {
#ifndef CONFIG_CAMERA_FLASH_NOT_SUPPORT
		return BAD_VALUE;
#endif
	}
	if (0 == strcmp(flash_mode,"invalid")) {
#ifndef CONFIG_CAMERA_FLASH_NOT_SUPPORT
		return BAD_VALUE;
#endif
	}

	focus_mode = ((SprdCameraParameters)params).get_FocusMode();
	LOGI("focus-mode:%s",focus_mode);
	if (!focus_mode) {
		return BAD_VALUE;
	}
	if (0 == strcmp(focus_mode,"invalid")) {
		LOGE("focus_mode is invalid.");
		return BAD_VALUE;
	}

	int w,h;
	params.getPreviewSize(&w, &h);
	if ((w < 0) || (h < 0)) {
		LOGE("Error to preview size: w: %d, h: %d.", w, h);
		/*mParameters.setPreviewSize(640, 480);*/ /*for cts*/
		return BAD_VALUE;
	}

	if(NULL == mParameters.get("video-size-values")) {
		if (isRecordingMode()) {
			if (mPreviewWidth != w || mPreviewHeight != h) {
				((SprdCameraParameters&)params).setPreviewSize(mPreviewWidth, mPreviewHeight);
				LOGW("should not change preview size in recording for no-video-size!");
			}
			params.getPreviewSize(&w, &h);
		}
	}
	return NO_ERROR;
}

status_t SprdCameraHardware::setParameters(const SprdCameraParameters& params)
{
	struct cmr_msg message = {0, 0, 0, 0};
	status_t ret = NO_ERROR;
	Mutex::Autolock l(&mLock);
	mParamLock.lock();

	if (checkSetParameters(params)) {
		mParamLock.unlock();
		LOGE("setParameters:invalid.");
		return BAD_VALUE;
	}

	if (0 == checkSetParameters(params, mSetParameters)) {
		LOGW("setParameters same parameters with system, directly return!");
		mParamLock.unlock();
		return NO_ERROR;
	} else if (SPRD_IDLE != getSetParamsState() || mBakParamFlag) {
		LOGI("setParameters is handling, backup the parameter!");
		/*mSetParametersBak = params;*/
		ret = copyParameters(mSetParametersBak, params);

		mBakParamFlag = 1;
		mParamLock.unlock();
		return NO_ERROR;
	} else {
		/*mSetParameters = params;*/
		ret = copyParameters(mSetParameters, params);
		/*also update the bak parameter's yet*/
		ret = copyParameters(mSetParametersBak, params);
	}

	message.msg_type = CMR_EVT_SW_MON_SET_PARA;
	message.data = NULL;

	ret = cmr_msg_post(mSwitchMonitorMsgQueHandle, &message, 1);
	if (ret) {
		LOGE("setParameters Fail to send one msg!");
		mParamLock.unlock();
		return NO_ERROR;
	}
	if (mParamWait.waitRelative(mParamLock, SET_PARAM_TIMEOUT)) {
		LOGE("setParameters wait timeout!");
	} else {
		LOGI("setParameters wait OK");
	}
	mParamLock.unlock();
	usleep(10*1000);

	return ret;
}

status_t SprdCameraHardware::checkFlashParameter(SprdCameraParameters& params)
{
	status_t ret =  NO_ERROR;
	SprdCameraParameters::ConfigType configType;

	/*check the if support the flash*/
	if ((0 == strcmp("hdr",params.get_SceneMode())
		|| 1 == params.getInt("zsl"))
		&& (CAMERA_FLASH_MODE_TORCH != params.getFlashMode()
		|| (NULL != params.get("recording-hint")
		&& 0 != strcmp("true",params.get("recording-hint"))))) {
		LOGI("checkFlashParameter - turnoff flash");
		if (0 == strcmp("true", (char*)mParameters.get("flash-mode-supported")))
			params.setFlashMode("off");
		mFlashMask = true;
	} else {
		if (0 == strcmp("true", (char*)mParameters.get("flash-mode-supported"))) {
			mFlashMask = false;
		}
	}


	return ret;
}

status_t SprdCameraHardware::setParametersInternal(const SprdCameraParameters& params)
{
	status_t ret = NO_ERROR;
	uint32_t isZoomChange = 0;
	char * isZslSupport = (char *)"false";

	LOGI("setParametersInternal: E params = %p", &params);
	mParamLock.lock();
#if 0
	// FIXME: verify params
	// yuv422sp is here only for legacy reason. Unfortunately, we release
	// the code with yuv422sp as the default and enforced setting. The
	// correct setting is yuv420sp.
	if (strcmp(params.getPreviewFormat(), "yuv422sp")== 0) {
		mPreviewFormat = 0;
	} else if (strcmp(params.getPreviewFormat(), "yuv420sp") == 0) {
		mPreviewFormat = 1;
	} else if (strcmp(params.getPreviewFormat(), "rgb565") == 0) {
		mPreviewFormat = 2;
	} else if (strcmp(params.getPreviewFormat(), "yuv420p") == 0) {
		mPreviewFormat = 3;
	} else {
		LOGE("Onlyyuv422sp/yuv420sp/rgb565 preview is supported.\n");
		return INVALID_OPERATION;
	}

	if (strcmp(params.getPictureFormat(), "yuv422sp")== 0) {
		mPictureFormat = 0;
	} else if (strcmp(params.getPictureFormat(), "yuv420sp")== 0) {
		mPictureFormat = 1;
	} else if (strcmp(params.getPictureFormat(), "rgb565")== 0) {
		mPictureFormat = 2;
	} else if (strcmp(params.getPictureFormat(), "jpeg")== 0) {
		mPictureFormat = 3;
	} else {
		LOGE("Onlyyuv422sp/yuv420sp/rgb565/jpeg  picture format is supported.\n");
		return INVALID_OPERATION;
	}
#else
	mPreviewFormat = 1;
	mPictureFormat = 1;
#endif

	LOGI("setParametersInternal: mPreviewFormat=%d,mPictureFormat=%d.",mPreviewFormat,mPictureFormat);

	if (0 == checkSetParameters(params, mParameters)) {
		LOGI("setParametersInternal X: same parameters with system, directly return!");
		mParamLock.unlock();
		return NO_ERROR;
	}

	if (mParameters.getZoom() != ((SprdCameraParameters)params).getZoom()) {
		LOGI("setParametersInternal, zoom level changed");
		isZoomChange = 1;
	}

	ret = checkSetParametersEnvironment();
	if (NO_ERROR != ret) {
		LOGW("setParametersInternal cannot set parameters, bakup parameters.");
		mSetParametersBak = params;
		if (!mBakParamFlag) {
			mBakParamFlag = 1;
			if (!isZoomChange) {
				mParamWait.signal();
			}
		}
		LOGW("setParametersInternal X: invalid status , directly return!");
		mParamLock.unlock();
		return NO_ERROR;
	}

	mParameters = params;

	isZslSupport = (char *)params.get("zsl-supported");
	if (NULL != isZslSupport) {
		LOGI("isZslSupport is not NULL.");
		mParameters.setZSLSupport((const char *)(isZslSupport));
	}

	if (!mBakParamFlag) {
		/*if zoom parameter changed, then the action should be sync*/
		if (!isZoomChange) {
			mParamWait.signal();
		}
	} else {
		mBakParamFlag = 0;
	}
	LOGI("setParametersInternal param set OK.");
	mParamLock.unlock();

	/*libqcamera only supports certain size/aspect ratios*/
	/*find closest match that doesn't exceed app's request*/
	int width = 0, height = 0;
	int rawWidth = 0, rawHeight = 0;
	mParameters.getPreviewSize(&width, &height);
	width = SIZE_ALIGN(width);
	LOGI("setParametersInternal: requested preview size %d x %d", width, height);
	/*if in DV mode then set the picture size as preview size*/
	if (1 == mParameters.getRecordingHint()) {
		mParameters.getPreviewSize(&width, &height);
		mParameters.setPictureSize(width, height);
	}
	mParameters.getPictureSize(&rawWidth, &rawHeight);
	LOGI("setParametersInternal:requested picture size %d x %d", rawWidth, rawHeight);

	mPreviewWidth = (width + 1) & ~1;
	mPreviewHeight = (height + 1) & ~1;
	mPreviewWidth_backup = mPreviewWidth;
	mPreviewHeight_backup = mPreviewHeight;
	mRawHeight = (rawHeight + 1) & ~1;
	mRawWidth = (rawWidth + 1) & ~1;

	antiShakeParamSetup();
	LOGI("setParametersInternal: requested picture size %d x %d  preview size %d x %d",
		mRawWidth, mRawHeight, mPreviewWidth, mPreviewHeight);

	camera_cfg_rot_cap_param_reset();

	if (camera_set_change_size(mRawWidth, mRawHeight, mPreviewWidth, mPreviewHeight)) {
		if (isPreviewing()) {
			LOGI("setParametersInternal preview or ZSL size should be changed!");
			stopPreviewInternal();
			if (NO_ERROR != setPreviewWindow(mPreviewWindow)) {
				LOGE("setParametersInternal X: setPreviewWindow fail, unknown error!");
				ret = UNKNOWN_ERROR;
				goto setParamEnd;
			}
			if (NO_ERROR != startPreviewInternal(isRecordingMode())) {
				LOGE("setParametersInternal X: change size startPreviewInternal fail, unknown error!");
				ret = UNKNOWN_ERROR;
				goto setParamEnd;
			}
		} else {
			if (NO_ERROR != setPreviewWindow(mPreviewWindow)) {
				LOGE("setParametersInternal X: setPreviewWindow fail, unknown error!");
				ret = UNKNOWN_ERROR;
				goto setParamEnd;
			}
		}
	}

	if ((1 == mParameters.getInt("zsl")) &&
		((mCaptureMode != CAMERA_ZSL_CONTINUE_SHOT_MODE) && (mCaptureMode != CAMERA_ZSL_MODE))) {
		if (isPreviewing()) {
			LOGI("setParametersInternal mode change:stop preview.ZSL ON");
			stopPreviewInternal();
			if (NO_ERROR != startPreviewInternal(isRecordingMode())) {
				LOGE("setParametersInternal X: open ZSL startPreviewInternal fail, unknown error!");
				ret = UNKNOWN_ERROR;
				goto setParamEnd;
			}
		}
	}
	if ((0 == mParameters.getInt("zsl")) &&
		((mCaptureMode == CAMERA_ZSL_CONTINUE_SHOT_MODE) || (mCaptureMode == CAMERA_ZSL_MODE))) {
		if (isPreviewing()) {
			LOGI("setParametersInternal mode change:stop preview.ZSL OFF");
			stopPreviewInternal();
			if (NO_ERROR != startPreviewInternal(isRecordingMode())) {
				LOGE("setParametersInternal X: close ZSL startPreviewInternal fail, unknown error!");
				ret = UNKNOWN_ERROR;
				goto setParamEnd;
			}
		}
	}

	if (NO_ERROR != setCameraParameters()) {
		ret = UNKNOWN_ERROR;
	}

setParamEnd:
	if (isZoomChange) {
		mParamWait.signal();
	}
	LOGI("setParametersInternal X.\n");

	return ret;
}

SprdCameraParameters SprdCameraHardware::getParameters()
{
	LOGI("getParameters: E");
	Mutex::Autolock l(&mLock);
	Mutex::Autolock pl(&mParamLock);
	LOGI("getParameters: X");

	if ((0 != checkSetParameters(mParameters, mSetParametersBak)) &&
		(0 != mBakParamFlag)) {
		return mSetParametersBak;
	} else {
		return mParameters;
	}
}

void SprdCameraHardware::setCallbacks(camera_notify_callback notify_cb,
		camera_data_callback data_cb,
		camera_data_timestamp_callback data_cb_timestamp,
		camera_request_memory get_memory,
		void *user)
{
	mNotify_cb = notify_cb;
	mData_cb = data_cb;
	mData_cb_timestamp = data_cb_timestamp;
	mGetMemory_cb = get_memory;
	mUser = user;
}

void SprdCameraHardware::enableMsgType(int32_t msgType)
{
	LOGI("mLock:enableMsgType E .\n");
	Mutex::Autolock lock(mLock);
	mMsgEnabled |= msgType;
	LOGI("mLock:enableMsgType X .\n");
}

void SprdCameraHardware::disableMsgType(int32_t msgType)
{
	LOGI("'mLock:disableMsgType E.\n");
	/*Mutex::Autolock lock(mLock);*/
	if (msgType & CAMERA_MSG_VIDEO_FRAME) {
		mRecordingFirstFrameTime = 0;
	}
	mMsgEnabled &= ~msgType;
	LOGI("'mLock:disableMsgType X.\n");
}

bool SprdCameraHardware::msgTypeEnabled(int32_t msgType)
{
	LOGI("mLock:msgTypeEnabled E.\n");
	Mutex::Autolock lock(mLock);
	LOGI("mLock:msgTypeEnabled X.\n");

	return (mMsgEnabled & msgType);
}

status_t SprdCameraHardware::sendCommand(int32_t cmd, int32_t arg1, int32_t arg2)
{
	uint32_t buffer_size = mPreviewWidth * mPreviewHeight * 3 / 2;
	uint32_t addr = 0;
	status_t ret = NO_ERROR;

	LOGE("sendCommand: facedetect mem size 0x%x.",buffer_size);
	if (CAMERA_CMD_START_FACE_DETECTION == cmd) {
		if (0 == mParameters.getInt("max-num-detected-faces-hw")) {
			LOGE("sendCommand: not support the CAMERA_CMD_START_FACE_DETECTION.");
			camera_set_facedetect_support(0);
			ret = BAD_VALUE;
			goto sendCommand_end;
		} else {
			camera_set_facedetect_support(1);
		}
		camera_set_start_facedetect(1, buffer_size);
	} else if(CAMERA_CMD_STOP_FACE_DETECTION == cmd) {
		if (0 == mParameters.getInt("max-num-detected-faces-hw")) {
			LOGE("sendCommand: not support the CAMERA_CMD_STOP_FACE_DETECTION.");
			camera_set_facedetect_support(0);
			ret = BAD_VALUE;
			goto sendCommand_end;
		} else {
			camera_set_facedetect_support(1);
		}
		camera_set_start_facedetect(0, 0);
	}
sendCommand_end:
	LOGI("sendCommand:ret = %d.",ret);
	return ret;
}

status_t SprdCameraHardware::storeMetaDataInBuffers(bool enable)
{
	/*metadata buffer mode can be turned on or off.*/
	if (!enable) {
		LOGE("Non-metadata buffer mode is not supported!");
		mIsStoreMetaData = false;
		return INVALID_OPERATION;
	}

	if (NULL == mMetadataHeap) {
		if (NULL == (mMetadataHeap = mGetMemory_cb(-1, METADATA_SIZE, kPreviewBufferCount, NULL))) {
			LOGE("fail to alloc memory for the metadata for storeMetaDataInBuffers.");
			return INVALID_OPERATION;
		}
	}

	mIsStoreMetaData = true;

	return NO_ERROR;
}

status_t SprdCameraHardware::dump(int fd) const
{
	const size_t SIZE = 256;
	char buffer[SIZE];
	String8 result;
	const Vector<String16> args;

	/*snprintf(buffer, 255, "SprdCameraHardware::dump: state (%s, %s, %s)\n",
		getCameraStateStr(getCameraState()), getCameraStateStr(getPreviewState()), getCameraStateStr(getCaptureState()));*/
	result.append(buffer);
	snprintf(buffer, 255, "preview width(%d) x height (%d)\n", mPreviewWidth, mPreviewHeight);
	result.append(buffer);
	snprintf(buffer, 255, "raw width(%d) x height (%d)\n", mRawWidth, mRawHeight);
	result.append(buffer);
	snprintf(buffer, 255, "preview frame size(%d), raw size (%d), jpeg size (%d) and jpeg max size (%d)\n", mPreviewHeapSize, mRawHeapSize, mJpegSize, mJpegHeapSize);
	result.append(buffer);
	write(fd, result.string(), result.size());
	mParameters.dump(fd, args);
	return NO_ERROR;
}

const char* SprdCameraHardware::getCameraStateStr(
	SprdCameraHardware::Sprd_camera_state s)
{
	static const char* states[] = {
#define STATE_STR(x) #x
	STATE_STR(SPRD_INIT),
	STATE_STR(SPRD_IDLE),
	STATE_STR(SPRD_ERROR),
	STATE_STR(SPRD_PREVIEW_IN_PROGRESS),
	STATE_STR(SPRD_FOCUS_IN_PROGRESS),
	STATE_STR(SPRD_SET_PARAMS_IN_PROGRESS),
	STATE_STR(SPRD_WAITING_RAW),
	STATE_STR(SPRD_WAITING_JPEG),
	STATE_STR(SPRD_INTERNAL_PREVIEW_STOPPING),
	STATE_STR(SPRD_INTERNAL_CAPTURE_STOPPING),
	STATE_STR(SPRD_INTERNAL_PREVIEW_REQUESTED),
	STATE_STR(SPRD_INTERNAL_RAW_REQUESTED),
	STATE_STR(SPRD_INTERNAL_STOPPING),
#undef STATE_STR
	};
	return states[s];
}

void SprdCameraHardware::print_time()
{
#if PRINT_TIME
	struct timeval time;
	gettimeofday(&time, NULL);
	LOGI("time: %lld us.", time.tv_sec * 1000000LL + time.tv_usec);
#endif
}

bool SprdCameraHardware::setCameraDimensions()
{
	if (isPreviewing() || isCapturing()) {
		LOGE("setCameraDimensions: expecting state SPRD_IDLE, not %s, %s",
				getCameraStateStr(getPreviewState()),
				getCameraStateStr(getCaptureState()));
		return false;
	}

	if (0 != camera_set_dimensions(mRawWidth, mRawHeight, mPreviewWidth,
		mPreviewHeight, NULL, NULL, mCaptureMode != CAMERA_RAW_MODE)) {
		return false;
	}

	return true;
}

void SprdCameraHardware::setCameraPreviewMode(bool isRecordMode)
{
	if (isRecordMode) {
		SET_PARM(CAMERA_PARM_PREVIEW_MODE, mParameters.getPreviewFameRate());
	} else {
		if (mIsDvPreview) {
			SET_PARM(CAMERA_PARM_PREVIEW_MODE, mParameters.getPreviewFameRate());
			SET_PARM(CAMERA_PARM_PREVIEW_ENV, mParameters.getPreviewFameRate());
		} else {
			SET_PARM(CAMERA_PARM_PREVIEW_MODE, CAMERA_PREVIEW_MODE_SNAPSHOT);
			SET_PARM(CAMERA_PARM_PREVIEW_ENV, CAMERA_PREVIEW_MODE_SNAPSHOT);
		}
	}
}

bool SprdCameraHardware::isRecordingMode()
{
	return mRecordingMode;
}

void SprdCameraHardware::setRecordingMode(bool enable)
{
	mRecordingMode = enable;
}

void SprdCameraHardware::setCameraState(Sprd_camera_state state, state_owner owner)
{
	Sprd_camera_state   org_state   = SPRD_IDLE;
	volatile Sprd_camera_state      * state_owner = NULL;

	LOGI("setCameraState:state: E");
	Mutex::Autolock stateLock(&mStateLock);
	LOGI("setCameraState:state: %s, owner: %d", getCameraStateStr(state), owner);
	switch (owner) {
		case STATE_CAMERA:
			org_state = mCameraState.camera_state;
			state_owner = &(mCameraState.camera_state);
			break;

		case STATE_PREVIEW:
			org_state = mCameraState.preview_state;
			state_owner = &(mCameraState.preview_state);
			break;

		case STATE_CAPTURE:
			org_state = mCameraState.capture_state;
			state_owner = &(mCameraState.capture_state);
			break;

		case STATE_FOCUS:
			org_state = mCameraState.focus_state;
			state_owner = &(mCameraState.focus_state);
			break;

		case STATE_SET_PARAMS:
			org_state = mCameraState.setParam_state;
			state_owner = &(mCameraState.setParam_state);
			break;
		default:
			LOGE("setCameraState: owner error!");
			break;
	}

	switch (state) {
	/*camera state*/
	case SPRD_INIT:
		mCameraState.camera_state = SPRD_INIT;
		mCameraState.preview_state = SPRD_IDLE;
		mCameraState.capture_state = SPRD_IDLE;
		mCameraState.focus_state = SPRD_IDLE;
		mCameraState.setParam_state = SPRD_IDLE;
		break;

	case SPRD_IDLE:
		*state_owner = SPRD_IDLE;
		break;

	case SPRD_INTERNAL_STOPPING:
		mCameraState.camera_state = state;
		break;

	case SPRD_ERROR:
		*state_owner = SPRD_ERROR;
		break;

	/*preview state*/
	case SPRD_PREVIEW_IN_PROGRESS:
	case SPRD_INTERNAL_PREVIEW_STOPPING:
	case SPRD_INTERNAL_PREVIEW_REQUESTED:
		mCameraState.preview_state = state;
		break;

	/*capture state*/
	case SPRD_INTERNAL_RAW_REQUESTED:
	case SPRD_WAITING_RAW:
	case SPRD_WAITING_JPEG:
	case SPRD_INTERNAL_CAPTURE_STOPPING:
		mCameraState.capture_state = state;
		break;

	/*focus state*/
	case SPRD_FOCUS_IN_PROGRESS:
		mCameraState.focus_state = state;
		break;

	/*set_param state*/
	case SPRD_SET_PARAMS_IN_PROGRESS:
		mCameraState.setParam_state = state;
		break;

	default:
		LOGE("setCameraState: unknown owner");
		break;
	}

	if (org_state != state)
		mStateWait.signal();              /*if state changed should broadcasting*/

	LOGI("setCameraState: X camera state = %s, preview state = %s, capture state = %s focus state = %s set param state = %s",
				getCameraStateStr(mCameraState.camera_state),
				getCameraStateStr(mCameraState.preview_state),
				getCameraStateStr(mCameraState.capture_state),
				getCameraStateStr(mCameraState.focus_state),
				getCameraStateStr(mCameraState.setParam_state));
}

SprdCameraHardware::Sprd_camera_state SprdCameraHardware::getCameraState()
{
	LOGI("getCameraState: %s", getCameraStateStr(mCameraState.camera_state));
	return mCameraState.camera_state;
}

SprdCameraHardware::Sprd_camera_state SprdCameraHardware::getPreviewState()
{
	LOGV("getPreviewState: %s", getCameraStateStr(mCameraState.preview_state));
	return mCameraState.preview_state;
}

SprdCameraHardware::Sprd_camera_state SprdCameraHardware::getCaptureState()
{
	LOGI("getCaptureState: %s", getCameraStateStr(mCameraState.capture_state));
	return mCameraState.capture_state;
}

SprdCameraHardware::Sprd_camera_state SprdCameraHardware::getFocusState()
{
	LOGI("getFocusState: %s", getCameraStateStr(mCameraState.focus_state));
	return mCameraState.focus_state;
}

SprdCameraHardware::Sprd_camera_state SprdCameraHardware::getSetParamsState()
{
	LOGI("getSetParamsState: %s", getCameraStateStr(mCameraState.setParam_state));
	return mCameraState.setParam_state;
}

bool SprdCameraHardware::isCameraInit()
{
	LOGI("isCameraInit: %s", getCameraStateStr(mCameraState.camera_state));
	return (SPRD_IDLE == mCameraState.camera_state);
}

bool SprdCameraHardware::isCameraIdle()
{
	return (SPRD_IDLE == mCameraState.preview_state
			&& SPRD_IDLE == mCameraState.capture_state);
}

bool SprdCameraHardware::isPreviewing()
{
	LOGV("isPreviewing: %s", getCameraStateStr(mCameraState.preview_state));
	return (SPRD_PREVIEW_IN_PROGRESS == mCameraState.preview_state);
}

bool SprdCameraHardware::isCapturing()
{
	bool ret = false;
	LOGI("isCapturing: %s", getCameraStateStr(mCameraState.capture_state));
#if 1
	if ((SPRD_INTERNAL_RAW_REQUESTED == mCameraState.capture_state) ||
		(SPRD_WAITING_RAW == mCameraState.capture_state) ||
		(SPRD_WAITING_JPEG == mCameraState.capture_state)) {
		ret = true;
	} else if ((SPRD_INTERNAL_CAPTURE_STOPPING == mCameraState.capture_state) ||
			(SPRD_ERROR == mCameraState.capture_state)) {
		if (camera_capture_is_idle()) {
			setCameraState(SPRD_IDLE, STATE_CAPTURE);
			LOGI("isCapturing: %s", getCameraStateStr(mCameraState.capture_state));
		} else {
			ret = true;
		}
	} else if (SPRD_IDLE != mCameraState.capture_state) {
		LOGE("isCapturing error: unknown capture status");
		ret = true;
	}
#else
	return (SPRD_IDLE != mCameraState.capture_state);
#endif
	return ret;
}

bool SprdCameraHardware::checkPreviewStateForCapture()
{
	bool ret = true;
	Sprd_camera_state tmpState = SPRD_IDLE;

	tmpState = getCaptureState();
	if ((SPRD_INTERNAL_CAPTURE_STOPPING == tmpState) ||
		(SPRD_ERROR == tmpState)) {
		LOGW("incorrect capture status %s", getCameraStateStr(tmpState));
		ret = false;
	} else {
		tmpState = getPreviewState();
		if (iSZslMode()) {
			if (SPRD_PREVIEW_IN_PROGRESS != tmpState) {
				LOGI("incorrect preview status %d of ZSL capture mode", (uint32_t)tmpState);
				ret = false;
			}
		} else {
			if (SPRD_IDLE != tmpState) {
				LOGI("incorrect preview status %d of normal capture mode", (uint32_t)tmpState);
				ret = false;
			}
		}
	}
	return ret;
}

bool SprdCameraHardware::WaitForCameraStart()
{
	Mutex::Autolock stateLock(&mStateLock);

	while(SPRD_IDLE != mCameraState.camera_state
		&& SPRD_ERROR != mCameraState.camera_state) {
		LOGI("WaitForCameraStart: waiting for SPRD_IDLE");
		mStateWait.wait(mStateLock);
		LOGI("WaitForCameraStart: woke up");
	}

	return SPRD_IDLE == mCameraState.camera_state;
}

bool SprdCameraHardware::WaitForCameraStop()
{
	Mutex::Autolock stateLock(&mStateLock);

	if (SPRD_INTERNAL_STOPPING == mCameraState.camera_state) {
		while(SPRD_INIT != mCameraState.camera_state
			&& SPRD_ERROR != mCameraState.camera_state) {
			LOGI("WaitForCameraStop: waiting for SPRD_IDLE");
			mStateWait.wait(mStateLock);
			LOGI("WaitForCameraStop: woke up");
		}
	}

	return SPRD_INIT == mCameraState.camera_state;
}

bool SprdCameraHardware::WaitForPreviewStart()
{
	Mutex::Autolock stateLock(&mStateLock);

	while (SPRD_PREVIEW_IN_PROGRESS != mCameraState.preview_state
		&& SPRD_ERROR != mCameraState.preview_state) {
		LOGI("WaitForPreviewStart: waiting for SPRD_PREVIEW_IN_PROGRESS");
		if (mStateWait.waitRelative(mStateLock, PREV_TIMEOUT)) {
			LOGE("WaitForPreviewStart: timeout");
			break;
		}
		LOGI("WaitForPreviewStart: woke up");
	}

	return SPRD_PREVIEW_IN_PROGRESS == mCameraState.preview_state;
}

bool SprdCameraHardware::WaitForPreviewStop()
{
	Mutex::Autolock statelock(&mStateLock);

	while (SPRD_IDLE != mCameraState.preview_state
		&& SPRD_ERROR != mCameraState.preview_state) {
		LOGI("WaitForPreviewStop: waiting for SPRD_IDLE");
		mStateWait.wait(mStateLock);
		LOGI("WaitForPreviewStop: woke up");
	}

	return SPRD_IDLE == mCameraState.preview_state;
}

bool SprdCameraHardware::WaitForCaptureStart()
{
	Mutex::Autolock stateLock(&mStateLock);

	/* It's possible for the YUV callback as well as the JPEG callbacks*/
	/*to be invoked before we even make it here, so we check for all*/
	/*possible result states from takePicture.*/
	while (SPRD_WAITING_RAW != mCameraState.capture_state
		&& SPRD_WAITING_JPEG != mCameraState.capture_state
		&& SPRD_IDLE != mCameraState.capture_state
		&& SPRD_ERROR != mCameraState.capture_state
		&& SPRD_ERROR != mCameraState.camera_state) {
		LOGI("WaitForCaptureStart: waiting for SPRD_WAITING_RAW or SPRD_WAITING_JPEG");
		mStateWait.wait(mStateLock);
		LOGI("WaitForCaptureStart: woke up, state is %s",
			getCameraStateStr(mCameraState.capture_state));
	}

	return (SPRD_WAITING_RAW == mCameraState.capture_state
		|| SPRD_WAITING_JPEG == mCameraState.capture_state
		|| SPRD_IDLE == mCameraState.capture_state);
}

bool SprdCameraHardware::WaitForCaptureDone()
{
	Mutex::Autolock stateLock(&mStateLock);
	while (SPRD_IDLE != mCameraState.capture_state
		&& SPRD_ERROR != mCameraState.capture_state) {
		LOGI("WaitForCaptureDone: waiting for SPRD_IDLE");
		if (camera_capture_is_idle()) {
			LOGI("WaitForCaptureDone: for OEM cap is IDLE, set capture state to %s",
				getCameraStateStr(mCameraState.capture_state));
			setCameraState(SPRD_IDLE, STATE_CAPTURE);
		} else {
			if (mStateWait.waitRelative(mStateLock, CAP_TIMEOUT)) {
				LOGE("WaitForCaptureDone timeout");
				break;
			}
		}
		LOGI("WaitForCaptureDone: woke up");
	}

	return SPRD_IDLE == mCameraState.capture_state;
}

bool SprdCameraHardware::WaitForFocusCancelDone()
{
	Mutex::Autolock stateLock(&mStateLock);
	while (SPRD_IDLE != mCameraState.focus_state
		&& SPRD_ERROR != mCameraState.focus_state) {
		LOGI("WaitForFocusCancelDone: waiting for SPRD_IDLE from %s",
			getCameraStateStr(getFocusState()));
		if (mStateWait.waitRelative(mStateLock, CANCEL_AF_TIMEOUT)) {
			LOGE("WaitForFocusCancelDone timeout");
		}
		LOGI("WaitForFocusCancelDone: woke up");
	}

	return SPRD_IDLE == mCameraState.focus_state;
}

bool SprdCameraHardware::startCameraIfNecessary()
{
	if (!isCameraInit()) {
		LOGI("waiting for camera_init to initialize.startCameraIfNecessary");
		if (CAMERA_SUCCESS != camera_init(mCameraId)) {
			setCameraState(SPRD_INIT, STATE_CAMERA);
			LOGE("CameraIfNecessary: fail to camera_init().");
			return false;
		}

		if (!camera_is_sensor_support_zsl()) {
			mParameters.setZSLSupport("false");
		}

		LOGI("waiting for camera_start.g_camera_id: %d.", mCameraId);
		if (CAMERA_SUCCESS != camera_start(camera_cb, this, mPreviewHeight, mPreviewWidth)) {
			setCameraState(SPRD_ERROR, STATE_CAMERA);
			LOGE("CameraIfNecessary: fail to camera_start().");
			return false;
		}

		LOGI("OK to camera_start.");
		WaitForCameraStart();

		set_ddr_freq(BASE_FREQ_REQ);

		LOGI("init camera: initializing parameters");
	} else {
		LOGI("camera hardware has been started already");
	}

	return true;
}

int SprdCameraHardware::Callback_AllocCaptureMem(void* handle, unsigned int size, unsigned int *addr_phy, unsigned int *addr_vir)
{
	SprdCameraHardware* camera = (SprdCameraHardware*)handle;
	sprd_camera_memory_t *memory = NULL;
	*addr_phy = 0;
	*addr_vir = 0;

	LOGI("Callback_AllocCaptureMem: size = %d", size);

	if (NULL == camera) {
		LOGE("Callback_AllocCaptureMem : error camera is null.");
		return -1;
	}

	if (camera->mSubRawHeapNum >= MAX_SUB_RAWHEAP_NUM) {
		LOGE("Callback_AllocCaptureMem: error mSubRawHeapNum=%d", camera->mSubRawHeapNum);
		return -1;
	}

	memory = camera->allocCameraMem(size, 1, true);

	if (NULL == memory) {
		LOGE("Callback_AllocCaptureMem: error memory is null.");
		goto mem_fail;
	}

	camera->mSubRawHeapArray[camera->mSubRawHeapNum] = memory;
	camera->mSubRawHeapNum ++;

	if (NULL == memory->handle) {
		LOGE("Callback_AllocCaptureMem: error memory->handle is null.");
		goto mem_fail;
	}

	*addr_phy = (unsigned int)memory->phys_addr;
	*addr_vir = (unsigned int)memory->data;

	return 0;

mem_fail:
	Callback_FreeCaptureMem(handle);

	return -1;
}

int SprdCameraHardware::Callback_FreeCaptureMem(void* handle)
{
	uint32_t i;
	SprdCameraHardware* camera = (SprdCameraHardware*)handle;

	if (camera == NULL) {
		LOGE("Callback_FreeCaptureMem: error camera is null.");
		return -1;
	}

	LOGI("Callback_FreeCaptureMem: mSubRawHeapNum = %d", camera->mSubRawHeapNum);

	for (i = 0; i < camera->mSubRawHeapNum; i++) {
		if (NULL != camera->mSubRawHeapArray[i]) {
			camera->freeCameraMem(camera->mSubRawHeapArray[i]);
		}
		camera->mSubRawHeapArray[i] = NULL;
	}
	camera->mSubRawHeapNum = 0;

	return 0;
}

sprd_camera_memory_t* SprdCameraHardware::allocCameraMem(int buf_size, int num_bufs, uint32_t is_cache)
{
	sprd_camera_memory_t *memory = (sprd_camera_memory_t *)malloc(sizeof(*memory));
	MemoryHeapIon *pHeapIon = NULL;

	if (NULL == memory) {
		LOGE("allocCameraMem: error memory is null.");
		return NULL;
	} else {
		memset(memory, 0 , sizeof(sprd_camera_memory_t));
	}

	memset(memory, 0, sizeof(*memory));
	memory->busy_flag = false;

	camera_memory_t *camera_memory = NULL;
	int paddr = 0, psize = 0;
	int  mem_size = buf_size * num_bufs ;
	void *base = NULL;
	int result = 0;

	if (0 == s_mem_method) {
		if (is_cache) {
			pHeapIon = new MemoryHeapIon("/dev/ion", mem_size ,0 , (1<<31) | ION_HEAP_ID_MASK_MM);
		} else {
			pHeapIon = new MemoryHeapIon("/dev/ion", mem_size , MemoryHeapBase::NO_CACHING, ION_HEAP_ID_MASK_MM);
		}
	} else {
		if (is_cache) {
			pHeapIon = new MemoryHeapIon("/dev/ion", mem_size ,0 , (1<<31) | ION_HEAP_ID_MASK_SYSTEM);
		} else {
			pHeapIon = new MemoryHeapIon("/dev/ion", mem_size , MemoryHeapBase::NO_CACHING, ION_HEAP_ID_MASK_SYSTEM);
		}
	}

	if (NULL == pHeapIon) {
		LOGE("allocCameraMem: error pHeapIon is null.");
		goto getpmem_end;
	}

	if (NULL == pHeapIon->getBase()
		|| 0xFFFFFFFF == (uint32_t)pHeapIon->getBase()) {
		LOGE("allocCameraMem: error getBase is null.");
		goto getpmem_end;
	}

	camera_memory = mGetMemory_cb(pHeapIon->getHeapID(), buf_size, num_bufs, NULL);

	if(NULL == camera_memory) {
		LOGE("allocCameraMem: error camera_memory is null.");
		goto getpmem_end;
	}

	if (0xFFFFFFFF == (uint32_t)camera_memory->data) {
		camera_memory = NULL;
		LOGE("allocCameraMem: error data is null.");
		goto getpmem_end;
	}

	if (0 == s_mem_method) {
		result = pHeapIon->get_phy_addr_from_ion(&paddr, &psize);
	} else {
		result = pHeapIon->get_mm_iova(&paddr, &psize);
	}

	if (result < 0) {
		LOGE("allocCameraMem: error get pHeapIon addr - method %d result 0x%x ",s_mem_method, result);
		goto getpmem_end;
	}

	base =  pHeapIon->getBase();
	if (0xFFFFFFFF == (uint32_t)base) {
		LOGE("allocCameraMem: error pHeapIon->getBase() failed 0x%x",(uint32_t)base);
		goto  getpmem_end;
	}

getpmem_end:

	memory->ion_heap = pHeapIon;
	memory->camera_memory = camera_memory;
	memory->phys_addr = paddr;
	memory->phys_size = psize;
	if (camera_memory) {
		memory->handle = camera_memory->handle;
	}
	if (pHeapIon) {
		memory->data = pHeapIon->getBase();
	}

	if (camera_memory) {
		if (0 == s_mem_method) {
			LOGI("allocCameraMem: phys_addr 0x%x, data: 0x%x, size: 0x%x, phys_size: 0x%x.",
				memory->phys_addr, (uint32_t)memory->data,
				camera_memory->size, memory->phys_size);
		} else {
			LOGI("allocCameraMem: mm_iova: phys_addr 0x%x, data: 0x%x, size: 0x%x, phys_size: 0x%x.",
				memory->phys_addr, (uint32_t)memory->data,
				camera_memory->size, memory->phys_size);
		}
	} else {
			LOGE("allocCameraMem: phys_addr or mm_iova: error.");
	}

	return memory;
}

void SprdCameraHardware::freeCameraMem(sprd_camera_memory_t* memory)
{
	if (memory) {
		if (NULL == memory->camera_memory) {
			LOGI("freeCameraMem: memory->camera_memory is null");
		} else if (memory->camera_memory->release) {
			memory->camera_memory->release(memory->camera_memory);
			memory->camera_memory = NULL;
		} else {
			LOGE("freeCameraMem: camera_memory->release is null.");
		}

		if(memory->ion_heap) {
			if (0 != s_mem_method) {
				LOGI("free_mm_iova: 0x%x,data: 0x%x, 0x%x",memory->phys_addr, (uint32_t)memory->data,memory->phys_size);
				memory->ion_heap->free_mm_iova(memory->phys_addr, memory->phys_size);
			}
			delete memory->ion_heap;
			memory->ion_heap = NULL;
		}
		free(memory);
	} else {
		LOGI("freeCameraMem: null");
	}
}

void SprdCameraHardware::clearCameraMem(sprd_camera_memory_t* memory)
{
	if (memory) {
		if (NULL == memory->camera_memory) {
			LOGI("clearCameraMem: memory->camera_memory is null");
		} else if (memory->camera_memory->release) {
			memory->camera_memory->release(memory->camera_memory);
			memory->camera_memory = NULL;
		} else {
			LOGE("clearCameraMem: camera_memory->release is null.");
		}

		if (memory->ion_heap) {
			if (0 != s_mem_method) {
				LOGI("free_mm_iova: 0x%x,data: 0x%x, 0x%x",memory->phys_addr, (uint32_t)memory->data,memory->phys_size);
				memory->ion_heap->free_mm_iova(memory->phys_addr, memory->phys_size);
			}
			delete memory->ion_heap;
			memory->ion_heap = NULL;
		}
	} else {
		LOGI("clearCameraMem: null");
	}
}

uint32_t SprdCameraHardware::getPreviewBufferID(buffer_handle_t *buffer_handle)
{
	uint32_t id = 0xFFFFFFFF;

	if (PREVIEW_BUFFER_USAGE_GRAPHICS == mPreviewBufferUsage) {
		int i = 0;
		for (i = 0; i < kPreviewBufferCount; i++) {
			if ((NULL != mPreviewBufferHandle[i]) &&
				(mPreviewBufferHandle[i] == buffer_handle)) {
				id = i;
				break;
			}
		}
	}

	return id;
}

void SprdCameraHardware::canclePreviewMem()
{
	if (PREVIEW_BUFFER_USAGE_GRAPHICS == mPreviewBufferUsage && mPreviewWindow) {
		int i = 0;
		struct private_handle_t *private_h = NULL;
		Mutex::Autolock pgbcl(&mGraphBufCntLock);

		for (i = 0; i < kPreviewBufferCount; i++) {
			if (mPreviewBufferHandle[i]) {
				if (s_mem_method) {
					LOGE("free_mm_iova by  allocatePreviewMemByGraphics:%d",i);
					private_h=(struct private_handle_t*) (*mPreviewBufferHandle[i]);
					MemoryHeapIon::Free_mm_iova(private_h->share_fd,mPreviewHeapArray_phy[i],mPreviewHeapArray_size[i]);
				}
				if (0 != mPreviewWindow->cancel_buffer(mPreviewWindow, mPreviewBufferHandle[i])) {
					LOGE("canclePreviewMem: cancel_buffer error id = %d",i);
				}
				mGraphBufferCount[i] = 0;
				mPreviewBufferHandle[i] = NULL;
			}
		}
	}
}

int SprdCameraHardware::releasePreviewFrame()
{
	int ret = 0;

	if (PREVIEW_BUFFER_USAGE_GRAPHICS == mPreviewBufferUsage
		&& mPreviewWindow) {
		int stride = 0;
		uint32_t free_buffer_id = 0xFFFFFFFF;
		buffer_handle_t *buffer_handle;

		if (0 != mPreviewWindow->dequeue_buffer(mPreviewWindow, &buffer_handle, &stride)) {
			ret = -1;
			LOGE("releasePreviewFrame fail: Could not dequeue gralloc buffer!\n");
		} else {
			free_buffer_id = getPreviewBufferID(buffer_handle);
			if (mPreviewCancelBufHandle[free_buffer_id]  == mPreviewBufferHandle[free_buffer_id]) {
				mPreviewCancelBufHandle[free_buffer_id] = NULL;
				LOGE("It's cancelled buf 0x%x, no need to release", free_buffer_id);
			} else {
				Mutex::Autolock pgbcl(&mGraphBufCntLock);
				if (isRecordingMode()) {
					mGraphBufferCount[free_buffer_id]--;
				} else {
					mGraphBufferCount[free_buffer_id] = 0;
				}
				LOGV("releasePreviewFrame 0x%x count %d", free_buffer_id, mGraphBufferCount[free_buffer_id]);
				if (0 == mGraphBufferCount[free_buffer_id]) {
					if (CAMERA_SUCCESS != camera_release_frame(free_buffer_id)) {
						ret = -1;
					}
				}
			}
		}
	}
	return ret;
}

bool SprdCameraHardware::allocatePreviewMemByGraphics()
{
	if (PREVIEW_BUFFER_USAGE_GRAPHICS == mPreviewBufferUsage) {
		int i = 0, usage = 0, stride = 0, miniUndequeued = 0;
		buffer_handle_t *buffer_handle = NULL;
		struct private_handle_t *private_h = NULL;

		if (!mPreviewWindow) {
			LOGE("mPreviewWindow is NULL");
			return 0;
		}

		if (0 != mPreviewWindow->set_buffer_count(mPreviewWindow, kPreviewBufferCount)) {
			LOGE("allocatePreviewMemByGraphics: could not set buffer count");
			return -1;
		}

		if (0 != mPreviewWindow->get_min_undequeued_buffer_count(mPreviewWindow, &miniUndequeued)) {
			LOGE("allocatePreviewMemByGraphics: minUndequeued error");
		}

		miniUndequeued = (miniUndequeued >= 3) ? miniUndequeued : 3;
		if (miniUndequeued >= kPreviewBufferCount) {
			LOGE("allocatePreviewMemByGraphics: minUndequeued value error: %d",miniUndequeued);
			return -1;
		}

		for (i = 0; i < kPreviewBufferCount; i++ ) {
			if (0 != mPreviewWindow->dequeue_buffer(mPreviewWindow, &buffer_handle, &stride)) {
				LOGE("allocatePreviewMemByGraphics: dequeue_buffer error");
				return -1;
			} else {
				mGraphBufferCount[i] = 0;
			}
			private_h=(struct private_handle_t*) (*buffer_handle);
			LOGI("get buffer handle 0x%x", (uint32_t)private_h);
			if (NULL == private_h) {
				LOGE("NULL buffer handle!");
				return -1;
			}
			if (0  == s_mem_method) {
				int ion_addr=0,ion_size=0;
				if (0 != MemoryHeapIon::Get_phy_addr_from_ion(private_h->share_fd,&ion_addr,&ion_size)) {
					LOGE("allocatePreviewMemByGraphics: Get_phy_addr_from_ion error");
					return -1;
				}
				LOGI("MemoryHeapIon::Get_mm_ion: %d addr 0x%x size 0x%x",i, ion_addr, ion_size);
				mPreviewBufferHandle[i] = buffer_handle;
				mPreviewHeapArray_phy[i] = (uint32_t)ion_addr;
				mPreviewHeapArray_vir[i] = (uint32_t)private_h->base;
				/*mPreviewHeapArray_size[i] = ion_size;*/
			} else {
				int iova_addr=0,iova_size=0;
				LOGI("MemoryHeapIon::Get_mm_iova: %d",i);
				if (MemoryHeapIon::Get_mm_iova(private_h->share_fd,&iova_addr,&iova_size)) {
					LOGE("allocatePreviewMemByGraphics: Get_mm_iova error");
					return -1;
				}
				mPreviewBufferHandle[i] = buffer_handle;
				mPreviewHeapArray_phy[i] = (uint32_t)iova_addr;
				mPreviewHeapArray_vir[i] = (uint32_t)private_h->base;
				mPreviewHeapArray_size[i]=iova_size;
			}
			LOGI("allocatePreviewMemByGraphics: phyaddr:0x%x, base:0x%x, size:0x%x, stride:0x%x ",
				mPreviewHeapArray_phy[i],private_h->base,private_h->size, stride);
			mCancelBufferEb[i] = 0;
		}

		for (i = (kPreviewBufferCount -miniUndequeued); i < kPreviewBufferCount; i++ ) {
			if (0 != mPreviewWindow->cancel_buffer(mPreviewWindow, mPreviewBufferHandle[i])) {
				LOGE("allocatePreviewMemByGraphics: cancel_buffer error: %d",i);
			}
			mPreviewCancelBufHandle[i] = mPreviewBufferHandle[i];
			mCancelBufferEb[i] = 1;
		}
	}
	return 0;
}

bool SprdCameraHardware::allocatePreviewMem()
{
	uint32_t i = 0, j = 0, buffer_start_id = 0, buffer_end_id = 0;
	uint32_t buffer_size = camera_get_size_align_page(mPreviewHeapSize);

	mPreviewHeapNum = kPreviewBufferCount;
	if (camera_get_rot_set()) {
		/* allocate more buffer for rotation */
		mPreviewHeapNum += kPreviewRotBufferCount;
		LOGI("initPreview: rotation, increase buffer: %d \n", mPreviewHeapNum);
	}

	if (allocatePreviewMemByGraphics()) {
		canclePreviewMem();
		return false;
	}

	if (PREVIEW_BUFFER_USAGE_DCAM == mPreviewBufferUsage) {
		mPreviewDcamAllocBufferCnt = mPreviewHeapNum;
		buffer_start_id = 0;
		buffer_end_id = mPreviewHeapNum;
	} else {
		mPreviewDcamAllocBufferCnt = 0;
		buffer_start_id = kPreviewBufferCount;
		buffer_end_id = buffer_start_id;

		if (camera_get_rot_set()) {
			mPreviewDcamAllocBufferCnt = kPreviewRotBufferCount;
			buffer_end_id = kPreviewBufferCount + kPreviewRotBufferCount;
		}

		/*add one node, specially used for mData_cb when receive preview frame*/
		mPreviewDcamAllocBufferCnt += 1;
		buffer_end_id += 1;
	}

	if (mPreviewDcamAllocBufferCnt > 0 && NULL == mPreviewHeapArray) {
		mPreviewHeapArray = (sprd_camera_memory_t**)malloc(mPreviewDcamAllocBufferCnt * sizeof(sprd_camera_memory_t*));
		if (mPreviewHeapArray == NULL) {
			return false;
		} else {
			memset(&mPreviewHeapArray[0], 0, mPreviewDcamAllocBufferCnt * sizeof(sprd_camera_memory_t*));
		}

		for (i = buffer_start_id; i < buffer_end_id; i++) {
			sprd_camera_memory_t* PreviewHeap = allocCameraMem(buffer_size, 1, true);
			if (NULL == PreviewHeap) {
				LOGE("allocatePreviewMem: error PreviewHeap is null, index=%d", i);
				return false;
			}

			if (NULL == PreviewHeap->handle) {
				LOGE("allocatePreviewMem: error handle is null, index=%d", i);
				freeCameraMem(PreviewHeap);
				freePreviewMem();
				return false;
			}

			if (PreviewHeap->phys_addr & 0xFF) {
				LOGE("allocatePreviewMem: error mPreviewHeap is not 256 bytes aligned, index=%d", i);
				freeCameraMem(PreviewHeap);
				freePreviewMem();
				return false;
			}

			mPreviewHeapArray[j++] = PreviewHeap;
			mPreviewHeapArray_phy[i] = PreviewHeap->phys_addr;
			mPreviewHeapArray_vir[i] = (uint32_t)PreviewHeap->data;
		}
	}
	return true;
}

uint32_t SprdCameraHardware::getRedisplayMem()
{
	uint32_t buffer_size = camera_get_size_align_page(SIZE_ALIGN(mPreviewWidth) * (SIZE_ALIGN(mPreviewHeight)) * 3 / 2);

	if (mIsRotCapture) {
		buffer_size <<= 1 ;
	}

	mReDisplayHeap = allocCameraMem(buffer_size, 1, false);

	if (NULL == mReDisplayHeap)
		return 0;

	if (NULL == mReDisplayHeap->handle) {
		LOGE("getRedisplayMem: error handle is null.");
		return 0;
	}

	if (mReDisplayHeap->phys_addr & 0xFF) {
		LOGE("getRedisplayMem: error mReDisplayHeap is not 256 bytes aligned.");
		return 0;
	}
	LOGI("getRedisplayMem: addr=0x%x.",(uint32_t)mReDisplayHeap->data);

	return mReDisplayHeap->phys_addr;
}

void SprdCameraHardware::FreeReDisplayMem()
{
	LOGI("free redisplay mem.");
	freeCameraMem(mReDisplayHeap);
	mReDisplayHeap = NULL;
}

void SprdCameraHardware::freePreviewMem()
{
	uint32_t i;
	LOGI("freePreviewMem E.");
	Mutex::Autolock pcpl(&mPrevBufLock);
	LOGI("freePreviewMem got Prev Cp lock");

	if (mPreviewHeapArray != NULL) {
		for (i = 0; i < mPreviewDcamAllocBufferCnt; i++) {
			if (NO_ERROR == mCbPrevDataBusyLock.tryLock()) {
				if (mPreviewHeapArray[i]) {
					freeCameraMem(mPreviewHeapArray[i]);
				}
				mCbPrevDataBusyLock.unlock();
			} else {
				if ((mPreviewHeapArray[i]) &&
					(true == mPreviewHeapArray[i]->busy_flag)) {
					LOGI("freePreviewMem wait Prev Bak Data lock");
					Mutex::Autolock pbdLock(&mPrevBakDataLock);
					LOGI("freePreviewMem got Prev Bak Data lock");
					LOGE("preview buffer is busy, skip, bakup and free later!");
					if (NULL == mPreviewHeapInfoBak.camera_memory) {
						memcpy(&mPreviewHeapInfoBak, mPreviewHeapArray[i], sizeof(sprd_camera_memory));
					} else {
						LOGE("preview buffer not clear, unknown error!!!");
						memcpy(&mPreviewHeapInfoBak, mPreviewHeapArray[i], sizeof(sprd_camera_memory));
					}
					mPreviewHeapBakUseFlag = 1;
					memset(mPreviewHeapArray[i], 0, sizeof(*mPreviewHeapArray[i]));
					free(mPreviewHeapArray[i]);
				}
			}
			mPreviewHeapArray[i] = NULL;
		}
		free(mPreviewHeapArray);
		mPreviewHeapArray = NULL;
	}

	LOGI("freePreviewMem start cancel Prev Mem");
	canclePreviewMem();
	LOGI("freePreviewMem cancel Prev Mem OK");
	mPreviewHeapSize = 0;
	mPreviewHeapNum = 0;
	LOGI("freePreviewMem X.");
}

bool SprdCameraHardware::initPreview()
{
	uint32_t page_size, buffer_size;
	uint32_t preview_buff_cnt = kPreviewBufferCount;
	if (!startCameraIfNecessary())
		return false;

	setCameraPreviewMode(isRecordingMode());

	//Tell libqcamera what the preview and raw dimensions are.  We
	//call this method even if the preview dimensions have not changed,
	//because the picture ones may have.
	//NOTE: if this errors out, mCameraState != SPRD_IDLE, which will be
	//checked by the caller of this method.
	if (!setCameraDimensions()) {
		LOGE("initPreview: setCameraDimensions failed");
		return false;
	}

	LOGI("initPreview: preview size=%dx%d", mPreviewWidth, mPreviewHeight);

	camerea_set_preview_format(mPreviewFormat);

	switch (mPreviewFormat) {
	case 0:
		case 1:/*yuv420*/
		if (mIsDvPreview) {
			mPreviewHeapSize = SIZE_ALIGN(mPreviewWidth) * SIZE_ALIGN(mPreviewHeight) * 3 / 2;
		} else {
			mPreviewHeapSize = mPreviewWidth * mPreviewHeight * 3 / 2;
		}
		break;

	default:
		return false;
	}

	if (!allocatePreviewMem()) {
		freePreviewMem();
		return false;
	}

	if (camera_set_preview_mem((uint32_t)mPreviewHeapArray_phy,
		(uint32_t)mPreviewHeapArray_vir,
		camera_get_size_align_page(mPreviewHeapSize),
		(uint32_t)mPreviewHeapNum))
		return false;

	return true;
}

bool SprdCameraHardware::switchBufferMode(uint32_t src, uint32_t dst)
{
	bool ret = true;

	if ((src != dst) && (!isPreviewing())) {
		if (SPRD_INTERNAL_PREVIEW_STOPPING == mCameraState.preview_state) {
			/*change to new value*/
			mPreviewBufferUsage = dst;
		} else if (mPreviewHeapArray != NULL) {
			/*free original memory*/
			freePreviewMem();

			/*change to new value*/
			mPreviewBufferUsage = dst;

			/*alloc new memory*/
			if (!allocatePreviewMem()) {
				freePreviewMem();
				ret = false;
			}
		} else {
			/*change to new value*/
			mPreviewBufferUsage = dst;
		}
	}

	if ((src != dst) && (PREVIEW_BUFFER_USAGE_DCAM == dst) && isPreviewing()) {
		stopPreviewInternal();
		mPreviewBufferUsage = PREVIEW_BUFFER_USAGE_DCAM;
		if (NO_ERROR != startPreviewInternal(isRecordingMode())) {
			LOGE("startPreviewInternal in switchBufferMode error!");
			ret = false;
		}
	}

	LOGI("switchBufferMode ret=%d", ret);
	return ret;
}

void SprdCameraHardware::deinitPreview()
{
	freePreviewMem();
	camera_set_preview_mem(0, 0, 0, 0);
}

bool SprdCameraHardware::allocateCaptureMem(bool initJpegHeap)
{
	uint32_t buffer_size = 0;

	LOGI("allocateCaptureMem, mJpegHeapSize = %d, mRawHeapSize = %d",
		mJpegHeapSize, mRawHeapSize);

	buffer_size = camera_get_size_align_page(mRawHeapSize);
	LOGI("allocateCaptureMem:mRawHeap align size = %d . count %d ",buffer_size, kRawBufferCount);
	{
		Mutex::Autolock cbufl(&mCapBufLock);
		mRawHeap = allocCameraMem(buffer_size, kRawBufferCount, true);
		if (NULL == mRawHeap) {
			LOGE("allocateCaptureMem: error mRawHeap is null.");
			goto allocate_capture_mem_failed;
		}

		if (NULL == mRawHeap->handle) {
			LOGE("allocateCaptureMem: error handle is null.");
			goto allocate_capture_mem_failed;
		}
		mCapBufIsAvail = 1;
	}

	if (initJpegHeap) {
		LOGI("allocateCaptureMem: initJpeg");
		mJpegHeap = NULL;

		buffer_size = camera_get_size_align_page(mJpegHeapSize);
		mJpegHeap = new AshmemPool(buffer_size,
					kJpegBufferCount,
					0,
					0,
					"jpeg");

		if (!mJpegHeap->initialized()) {
			LOGE("allocateCaptureMem: erro initializing mJpegHeap failed.");
			goto allocate_capture_mem_failed;
		}

		LOGI("allocateCaptureMem: initJpeg success");
	}
	LOGI("allocateCaptureMem: X");

	return true;

allocate_capture_mem_failed:
	freeCaptureMem();
	mJpegHeap = NULL;
	mJpegHeapSize = 0;

	return false;
}

void SprdCameraHardware::freeCaptureMem()
{
	LOGI("freeCaptureMem E!");
{
	Mutex::Autolock cpl(&mCapBufLock);
	LOGI("free mRawHeap!");

	if (NO_ERROR == mCbCapDataBusyLock.tryLock()) {
		if (mRawHeap) {
			freeCameraMem(mRawHeap);
		}
		mCbCapDataBusyLock.unlock();
	} else {
		if (mRawHeap) {
			if (true == mRawHeap->busy_flag) {
				Mutex::Autolock cbdLock(&mCapBakDataLock);
				LOGE("freeCaptureMem, raw buffer is busy, skip, bakup and free later!");
				if (NULL == mRawHeapInfoBak.camera_memory) {
					memcpy(&mRawHeapInfoBak, mRawHeap, sizeof(*mRawHeap));
				} else {
					LOGE("preview buffer not clear, unknown error!!!");
					memcpy(&mRawHeapInfoBak, mRawHeap, sizeof(*mRawHeap));
				}
				mRawHeapBakUseFlag = 1;
				memset(mRawHeap, 0, sizeof(*mRawHeap));
				free(mRawHeap);
			}
		}
	}
	mRawHeap = NULL;
	mCapBufIsAvail = 0;
}
	mRawHeapSize = 0;

	LOGI("free mSubRawHeap!");

	Callback_FreeCaptureMem((void *)this);

	LOGI("freeCaptureMem X!");
}

bool SprdCameraHardware::initCapture(bool initJpegHeap)
{
	uint32_t local_width = 0, local_height = 0;
	uint32_t mem_size = 0;

	LOGI("initCapture E, %d", initJpegHeap);
	camera_set_capture_trace(1);
	if (!startCameraIfNecessary())
		return false;

	if (camera_set_dimensions(mRawWidth,
		mRawHeight,
		mPreviewWidth,
		mPreviewHeight,
		NULL,
		NULL,
		mCaptureMode != CAMERA_RAW_MODE)){
		return false;
	}

	if (camera_capture_max_img_size(&local_width, &local_height))
		return false;

	if (camera_capture_get_buffer_size(mCameraId, local_width, local_height, &mem_size))
		return false;

	mRawHeapSize = mem_size;
	mJpegHeapSize = mRawHeapSize;
	mJpegHeap = NULL;

	if (!allocateCaptureMem(initJpegHeap)) {
		return false;
	}

	if (camera_set_capture_mem(0,
		(uint32_t)mRawHeap->phys_addr,
		(uint32_t)mRawHeap->data,
		(uint32_t)mRawHeap->phys_size,
		(uint32_t)Callback_AllocCaptureMem,
		(uint32_t)Callback_FreeCaptureMem,
		(uint32_t)this))
		return false;

	LOGI("initCapture X success");
	return true;
}

void SprdCameraHardware::deinitCapture()
{
	freeCaptureMem();
	camera_set_capture_trace(0);
}

status_t SprdCameraHardware::set_ddr_freq(uint32_t mhzVal)
{
	const char*     freq_in_khz = NO_FREQ_STR;
	uint32_t        tmpSetFreqCount = mSetDDRFreqCount;

	LOGI("set_ddr_freq to %d now count %d freq %d E", mhzVal, mSetDDRFreqCount, mSetDDRFreq);
	if (mhzVal == mSetDDRFreq && NO_FREQ_REQ != mhzVal) {
		LOGW("set_ddr_freq same freq %d need not set", mhzVal);
		return NO_ERROR;
	}

	const char* const set_freq = "/sys/devices/platform/scxx30-dmcfreq.0/devfreq/scxx30-dmcfreq.0/ondemand/set_freq";

	FILE* fp = fopen(set_freq, "wb");
	if (NULL == fp) {
		LOGE("set_ddr_freq Failed to open %s X", set_freq);
		return BAD_VALUE;
	}

	switch (mhzVal) {
		case NO_FREQ_REQ:
			tmpSetFreqCount--;
			break;

		case BASE_FREQ_REQ:
			if (NO_FREQ_REQ == mSetDDRFreq) {
				tmpSetFreqCount++;
			} else {
				LOGV("set_ddr_freq clear freq for change!");
				fprintf(fp, "%s", NO_FREQ_STR);
			}
			freq_in_khz = BASE_FREQ_STR;
			break;

		case MEDIUM_FREQ_REQ:
			if (NO_FREQ_REQ == mSetDDRFreq) {
				tmpSetFreqCount++;
			} else {
				LOGV("set_ddr_freq clear freq for change!");
				fprintf(fp, "%s", NO_FREQ_STR);
			}
			freq_in_khz = MEDIUM_FREQ_STR;
			break;

		case HIGH_FREQ_REQ:
			if (NO_FREQ_REQ == mSetDDRFreq) {
				tmpSetFreqCount++;
			} else {
				LOGV("set_ddr_freq clear freq for change!");
				fprintf(fp, "%s", NO_FREQ_STR);
			}
			freq_in_khz = HIGH_FREQ_STR;
			break;

		default:
			LOGE("set_ddr_freq unrecognize set frequency, error!");
			break;
	}

	fclose(fp);
	fp = NULL;

	fp = fopen(set_freq, "wb");
	if (NULL == fp) {
		LOGE("set_ddr_freq Failed to open %s X", set_freq);
		return BAD_VALUE;
	}

	fprintf(fp, "%s", freq_in_khz);
	mSetDDRFreq = mhzVal;
	mSetDDRFreqCount = tmpSetFreqCount;
	LOGI("set_ddr_freq to %skhz now count %d freq %d X", freq_in_khz, mSetDDRFreqCount, mSetDDRFreq);
	fclose(fp);
	return NO_ERROR;
}

status_t SprdCameraHardware::startPreviewInternal(bool isRecording)
{
	char * isZslSupport = (char *)"false";
	LOGI("startPreviewInternal E");

	if ((1 == mParameters.getRecordingHint()) || isRecording) {
		isZslSupport = (char *)mParameters.get("zsl-supported");
		if ((isZslSupport) && (0 == strcmp("true", isZslSupport))) {
			LOGI("zsl-supported is %s", isZslSupport);
			mParameters.setZsl(1);
		} else {
			set_ddr_freq(BASE_FREQ_REQ);
		}
	}

	takepicture_mode mode = getCaptureMode();
	LOGI("startPreviewInternal isRecording=%d.captureMode=%d",isRecording, mCaptureMode);

	if (isPreviewing()) {
		LOGW("startPreviewInternal: already in progress, doing nothing.X");
		setRecordingMode(isRecording);
		setCameraPreviewMode(isRecordingMode());
		return NO_ERROR;
	}

	// We check for these two states explicitly because it is possible
	// for startPreview() to be called in response to a raw or JPEG
	// callback, but before we've updated the state from SPRD_WAITING_RAW
	// or SPRD_WAITING_JPEG to SPRD_IDLE.  This is because in camera_cb(),
	// we update the state *after* we've made the callback.  See that
	// function for an explanation.
	while (isCapturing()) {
		uint32_t wait_cnt = 0;
		if (ON_OFF_ACT_TIMEOUT < wait_cnt) {
			LOGE("startPreviewInternal X Capture state is %s, expecting SPRD_IDLE!",
				getCameraStateStr(mCameraState.capture_state));
			return INVALID_OPERATION;
		} else {
			usleep(10*1000);
			wait_cnt++;
		}
	}

	setRecordingMode(isRecording);

	cameraBakMemCheckAndFree();

	if (!initPreview()) {
		LOGE("startPreviewInternal X initPreview failed.  Not starting preview.");
		deinitPreview();
		return UNKNOWN_ERROR;
	}
	if (1 == mParameters.getInt("zsl")) {
		set_ddr_freq(HIGH_FREQ_REQ);
		deinitCapture();
		if (!initCapture(mData_cb != NULL)) {
			deinitCapture();
			set_ddr_freq(BASE_FREQ_REQ);
			LOGE("startPreviewInternal X initCapture failed. Not taking picture.");
			return UNKNOWN_ERROR;
		}
	}

	setCameraState(SPRD_INTERNAL_PREVIEW_REQUESTED, STATE_PREVIEW);

	camera_ret_code_type qret = camera_start_preview(camera_cb, this,mode);
	if (qret != CAMERA_SUCCESS) {
		LOGE("startPreviewInternal failed: sensor error.");
		setCameraState(SPRD_ERROR, STATE_PREVIEW);
		deinitPreview();
		if (iSZslMode()) {
			set_ddr_freq(BASE_FREQ_REQ);
		}
		return UNKNOWN_ERROR;
	}

	bool result = WaitForPreviewStart();

	LOGI("startPreviewInternal X,mRecordingMode=%d.",isRecordingMode());

	return result ? NO_ERROR : UNKNOWN_ERROR;
}

void SprdCameraHardware::stopPreviewInternal()
{
	nsecs_t start_timestamp = systemTime();
	nsecs_t end_timestamp;
	char * isZslSupport = (char *)"false";
	LOGI("stopPreviewInternal E");

	if (isCapturing()) {
		setCameraState(SPRD_INTERNAL_CAPTURE_STOPPING, STATE_CAPTURE);
		if (0 != camera_stop_capture()) {
			LOGE("stopPreviewInternal: camera_stop_capture failed!");
			return;
		}
	}

	if (!isPreviewing()) {
		LOGE("Preview not in progress! stopPreviewInternal X");
		return;
	}

	setCameraState(SPRD_INTERNAL_PREVIEW_STOPPING, STATE_PREVIEW);

	if(CAMERA_SUCCESS != camera_stop_preview()) {
		setCameraState(SPRD_ERROR, STATE_PREVIEW);
		LOGE("stopPreviewInternal X: fail to camera_stop_preview().");
	}

	if (iSZslMode()) {
		set_ddr_freq(BASE_FREQ_REQ);
	}

	WaitForPreviewStop();

	cameraBakMemCheckAndFree();

	deinitPreview();

	if (iSZslMode()) {
		deinitCapture();
	}

	if (1 == mParameters.getRecordingHint()) {
		isZslSupport = (char *)mParameters.get("zsl-supported");
		if ((isZslSupport) && (0 == strcmp("true", isZslSupport))) {
			LOGI("stopRecording zsl-supported is %s", isZslSupport);
			mParameters.setZsl(0);
		}
	}

	end_timestamp = systemTime();
	LOGI("stopPreviewInternal X Time:%lld(ms).",(end_timestamp - start_timestamp)/1000000);
}

takepicture_mode SprdCameraHardware::getCaptureMode()
{
	if (0 == strcmp("hdr",mParameters.get_SceneMode()) && (1 != mParameters.getRecordingHint())) {
		mCaptureMode = CAMERA_HDR_MODE;
	} else if ((1 == mParameters.getInt("zsl"))&&(1 != mParameters.getInt("capture-mode"))) {
		mCaptureMode = CAMERA_ZSL_CONTINUE_SHOT_MODE;
	} else if ((1 != mParameters.getInt("zsl"))&&(1 != mParameters.getInt("capture-mode"))) {
		mCaptureMode = CAMERA_NORMAL_CONTINUE_SHOT_MODE;
	} else if (1 == mParameters.getInt("zsl")) {
		mCaptureMode = CAMERA_ZSL_MODE;
	} else if (1 != mParameters.getInt("zsl")) {
		mCaptureMode = CAMERA_NORMAL_MODE;
	} else {
		mCaptureMode = CAMERA_NORMAL_MODE;
	}
	if (1 == mCaptureRawMode) {
		mCaptureMode = CAMERA_RAW_MODE;
	}

	LOGI("cap mode %d.\n", mCaptureMode);

	return mCaptureMode;
}

bool SprdCameraHardware::iSDisplayCaptureFrame()
{
	bool ret = true;

	if ((CAMERA_ZSL_MODE == mCaptureMode)
		|| (CAMERA_ZSL_CONTINUE_SHOT_MODE == mCaptureMode)) {
		ret = false;
	}
	LOGI("display capture flag is %d.",ret);

	return ret;
}

bool SprdCameraHardware::iSZslMode()
{
	bool ret = true;

	if ((CAMERA_ZSL_MODE != mCaptureMode)
		&& (CAMERA_ZSL_CONTINUE_SHOT_MODE != mCaptureMode)) {
		ret = false;
	}

	return ret;
}


status_t SprdCameraHardware::cancelPictureInternal()
{
	bool result = true;

	LOGI("cancelPictureInternal: E, state = %s", getCameraStateStr(getCaptureState()));

	switch (getCaptureState()) {
	case SPRD_INTERNAL_RAW_REQUESTED:
	case SPRD_WAITING_RAW:
	case SPRD_WAITING_JPEG:
		LOGI("camera state is %s, stopping picture.", getCameraStateStr(getCaptureState()));

		setCameraState(SPRD_INTERNAL_CAPTURE_STOPPING, STATE_CAPTURE);

		if (0 != camera_stop_capture()) {
			LOGE("cancelPictureInternal: camera_stop_capture failed!");
			return UNKNOWN_ERROR;
		}

		result = WaitForCaptureDone();
		break;

	default:
		LOGW("not taking a picture (state %s)", getCameraStateStr(getCaptureState()));
		break;
	}

	LOGI("cancelPictureInternal: X");
	return result ? NO_ERROR : UNKNOWN_ERROR;
}
void SprdCameraHardware::setCameraPrivateData()
{
	camera_sensor_exif_info exif_info;
	const char * isFlashSupport = mParameters.get("flash-mode-supported");
	memset(&exif_info, 0, sizeof(camera_sensor_exif_info));

	if (isFlashSupport
		&& 0 == strcmp("true", (char*)isFlashSupport)) {
		exif_info.flash = 1;
	}

	camera_config_exif_info(&exif_info);
}
status_t SprdCameraHardware::initDefaultParameters()
{
	uint32_t lcd_w = 0, lcd_h = 0;
	status_t ret = NO_ERROR;

	LOGI("initDefaultParameters E");
	SprdCameraParameters p;

	SprdCameraParameters::ConfigType config = (1 == mCameraId)
			? SprdCameraParameters::kFrontCameraConfig
			: SprdCameraParameters::kBackCameraConfig;

	p.setDefault(config);

	if (getLcdSize(&lcd_w, &lcd_h))
	{
		/*update preivew size by lcd*/
		p.updateSupportedPreviewSizes(lcd_w, lcd_h);
	}

	if (setParametersInternal(p) != NO_ERROR) {
		LOGE("Failed to set default parameters?!");
		ret = UNKNOWN_ERROR;
	} else {
		setCameraPrivateData();
	}

	mParamLock.lock();
	copyParameters(mSetParameters, p);
	copyParameters(mSetParametersBak, p);
	mParamLock.unlock();

	LOGI("initDefaultParameters X.");

	return ret;
}

bool SprdCameraHardware::getLcdSize(uint32_t *width, uint32_t *height)
{
	char const * const device_template[] = {
		"/dev/graphics/fb%u",
		"/dev/fb%u",
		NULL
	};

	int fd = -1;
	int i = 0;
	char name[64];

	if (NULL == width || NULL == height)
		return false;

	while ((fd == -1) && device_template[i]) {
		snprintf(name, 64, device_template[i], 0);
		fd = open(name, O_RDONLY, 0);
		i++;
	}

	LOGI("getLcdSize dev is %s", name);

	if (fd < 0) {
		LOGE("getLcdSize fail to open fb");
		return false;
	}

	struct fb_var_screeninfo info;
	if (ioctl(fd, FBIOGET_VSCREENINFO, &info) == -1) {
		LOGE("getLcdSize fail to get fb info");
		close(fd);
		return false;
	}

	LOGI("getLcdSize w h %d %d", info.yres, info.xres);
	*width  = info.yres;
	*height = info.xres;

	close(fd);
	return true;
}

status_t SprdCameraHardware::setCameraParameters()
{
	bool switch_ret = false;

	LOGI("setCameraParameters: E");

	//Because libqcamera is broken, for the camera_set_parm() calls
	//SprdCameraHardware camera_cb() is called synchronously,
	//so we cannot wait on a state change.  Also, we have to unlock
	//the mStateLock, because camera_cb() acquires it.

	if (true != startCameraIfNecessary())
		return UNKNOWN_ERROR;

	int min,max;
	mParameters.getPreviewFpsRange(&min, &max);
	if ((min > max) || (min < 0) || (max < 0)) {
		LOGE("Error to FPS range: mix: %d, max: %d.", min, max);
		return UNKNOWN_ERROR;
	}
	LOGI("setCameraParameters: preview fps range: min = %d, max = %d", min, max);

	int w,h;
	mParameters.getPreviewSize(&w, &h);
	if ((w < 0) || (h < 0)) {
		mParameters.setPreviewSize(640, 480);
		return UNKNOWN_ERROR;
	}
	LOGI("setCameraParameters: preview size: %dx%d", w, h);

	LOGI("mIsRotCapture:%d.",mIsRotCapture);
	if (mIsRotCapture) {
		SET_PARM(CAMERA_PARAM_ROTATION_CAPTURE, 1);
	} else {
		SET_PARM(CAMERA_PARAM_ROTATION_CAPTURE, 0);
	}

	int rotation = mParameters.getInt("rotation");

	/*Rotation may be negative, but may not be -1, because it has to be a*/
	/*multiple of 90.  That's why we can still interpret -1 as an error*/
	if (rotation == -1) {
		LOGI("rotation not specified or is invalid, defaulting to 0");
		rotation = 0;
	} else if (rotation % 90) {
		LOGI("rotation %d is not a multiple of 90 degrees!  Defaulting to zero.",
		rotation);
		rotation = 0;
	} else {
		/*normalize to [0 - 270] degrees*/
		rotation %= 360;
		if (rotation < 0) rotation += 360;
	}
	SET_PARM(CAMERA_PARM_ENCODE_ROTATION, rotation);

	if (1 == mParameters.getInt("sensororientation")) {
		SET_PARM(CAMERA_PARM_ORIENTATION, 1);
	} else {
		SET_PARM(CAMERA_PARM_ORIENTATION, 0);
	}

	rotation = mParameters.getInt("sensorrotation");
	if (-1 == rotation)
		rotation = 0;
	SET_PARM(CAMERA_PARM_SENSOR_ROTATION, rotation);

	if (0 != rotation) {
		switch_ret = switchBufferMode(mPreviewBufferUsage, PREVIEW_BUFFER_USAGE_DCAM);
	} else {
		switch_ret = switchBufferMode(mPreviewBufferUsage, mOriginalPreviewBufferUsage);
	}
	if (!switch_ret) {
		LOGI("setCameraParameters switch buffer fail");
		return UNKNOWN_ERROR;
	}

	SET_PARM(CAMERA_PARM_SHOT_NUM, mParameters.getInt("capture-mode"));

	int is_mirror = (mCameraId == 1) ? 1 : 0;
	int ret = 0;

	SprdCameraParameters::Size preview_size = {0, 0};
	SprdCameraParameters::Rect preview_rect = {0, 0, 0, 0};
	int area[4 * SprdCameraParameters::kFocusZoneMax + 1] = {0};

	preview_size.width = mPreviewWidth;
	preview_size.height = mPreviewHeight;

	ret = camera_get_preview_rect(&preview_rect.x, &preview_rect.y,
		&preview_rect.width, &preview_rect.height);
	if(ret) {
		LOGI("coordinate_convert: camera_get_preview_rect failed, return \n");
		return UNKNOWN_ERROR;
	}
	mParameters.getFocusAreas(&area[1], &area[0], &preview_size, &preview_rect,
		kCameraInfo[mCameraId].orientation, is_mirror);
	SET_PARM(CAMERA_PARM_FOCUS_RECT, (int32_t)area);

	int ae_mode = mParameters.getAutoExposureMode();
	if (2 == ae_mode) {
		mParameters.getMeteringAreas(&area[1], &area[0], &preview_size, &preview_rect,
					kCameraInfo[mCameraId].orientation, is_mirror);
		SET_PARM(CAMERA_PARM_EXPOSURE_METERING, (int32_t)area);
	} else {
		SET_PARM(CAMERA_PARM_AUTO_EXPOSURE_MODE, ae_mode);
	}

	if (0 == mCameraId) {
		SET_PARM(CAMERA_PARM_AF_MODE, mParameters.getFocusMode());
		if ((NULL != mParameters.get("flash-mode-supported"))
			&& (0 == strcmp(mParameters.get("flash-mode-supported"),"true"))
			&& (false == mFlashMask)) {
			SET_PARM(CAMERA_PARM_FLASH, mParameters.getFlashMode());
		} else {
			SET_PARM(CAMERA_PARM_FLASH, CAMERA_FLASH_MODE_OFF);
		}
	}

	mTimeCoeff = mParameters.getSlowmotion();
	LOGI("mTimeCoeff:%d",mTimeCoeff);
	SET_PARM(CAMERA_PARM_WB, mParameters.getWhiteBalance());
	SET_PARM(CAMERA_PARM_CAMERA_ID, mParameters.getCameraId());
	SET_PARM(CAMERA_PARM_JPEGCOMP, mParameters.getJpegQuality());
	SET_PARM(CAMERA_PARM_THUMBCOMP, mParameters.getJpegThumbnailQuality());
	SET_PARM(CAMERA_PARM_EFFECT, mParameters.getEffect());
	SET_PARM(CAMERA_PARM_SCENE_MODE, mParameters.getSceneMode());

	mZoomLevel = mParameters.getZoom();
	SET_PARM(CAMERA_PARM_ZOOM, mZoomLevel);
	SET_PARM(CAMERA_PARM_BRIGHTNESS, mParameters.getBrightness());
	SET_PARM(CAMERA_PARM_SHARPNESS, mParameters.getSharpness());
	SET_PARM(CAMERA_PARM_CONTRAST, mParameters.getContrast());
	SET_PARM(CAMERA_PARM_SATURATION, mParameters.getSaturation());
	SET_PARM(CAMERA_PARM_EXPOSURE_COMPENSATION, mParameters.getExposureCompensation());
	SET_PARM(CAMERA_PARM_ANTIBANDING, mParameters.getAntiBanding());
	SET_PARM(CAMERA_PARM_ISO, mParameters.getIso());
	SET_PARM(CAMERA_PARM_DCDV_MODE, mParameters.getRecordingHint());

	int ns_mode = mParameters.getInt("nightshot-mode");
	if (ns_mode < 0) ns_mode = 0;
	SET_PARM(CAMERA_PARM_NIGHTSHOT_MODE, ns_mode);

	int luma_adaptation = mParameters.getInt("luma-adaptation");
	if (luma_adaptation < 0) luma_adaptation = 0;
	SET_PARM(CAMERA_PARM_LUMA_ADAPTATION, luma_adaptation);

	double focal_len = atof(mParameters.get("focal-length")) * 1000;
	SET_PARM(CAMERA_PARM_FOCAL_LENGTH, (int32_t)focal_len);

	int th_w, th_h, th_q;
	th_w = mParameters.getInt("jpeg-thumbnail-width");
	if (th_w < 0) LOGW("property jpeg-thumbnail-width not specified");

	th_h = mParameters.getInt("jpeg-thumbnail-height");
	if (th_h < 0) LOGW("property jpeg-thumbnail-height not specified");

	th_q = mParameters.getInt("jpeg-thumbnail-quality");
	if (th_q < 0) LOGW("property jpeg-thumbnail-quality not specified");

	if (th_w >= 0 && th_h >= 0 && th_q >= 0) {
		LOGI("setting thumbnail dimensions to %dx%d, quality %d", th_w, th_h, th_q);

		int ret = camera_set_thumbnail_properties(th_w, th_h, th_q);

		if (ret != CAMERA_SUCCESS) {
			LOGE("camera_set_thumbnail_properties returned %d", ret);
			}
	}

	camera_encode_properties_type encode_properties = {0, CAMERA_JPEG, 0};
	encode_properties.quality = mParameters.getInt("jpeg-quality");

	if (encode_properties.quality < 0) {
		LOGW("JPEG-image quality is not specified "
			"or is negative, defaulting to %d",
			encode_properties.quality);
		encode_properties.quality = 100;
	} else {
		LOGI("Setting JPEG-image quality to %d",
			encode_properties.quality);
	}

	encode_properties.format = CAMERA_JPEG;
	encode_properties.file_size = 0x0;
	camera_set_encode_properties(&encode_properties);

	LOGI("setCameraParameters: X");
	return NO_ERROR;
}

void SprdCameraHardware::getPictureFormat(int * format)
{
	*format = mPictureFormat;
}

#define PARSE_LOCATION(what,type,fmt,desc) do {\
	pt->what = 0;\
	const char *what##_str = mParameters.get("gps-"#what);\
	LOGI("%s: GPS PARM %s --> [%s]", __func__, "gps-"#what, what##_str);\
	if (what##_str) {\
		type what = 0;\
		if (sscanf(what##_str, fmt, &what) == 1) {\
		pt->what = what;\
		} else {\
			LOGE("GPS " #what " %s could not"\
				" be parsed as a " #desc,\
				what##_str);\
			result = false;\
		}\
	} else {\
		LOGW("%s: GPS " #what " not specified: "\
			"defaulting to zero in EXIF header.", __func__);\
		result = false;\
	}\
}while(0)

bool SprdCameraHardware::getCameraLocation(camera_position_type *pt)
{
	bool result = true;

	PARSE_LOCATION(timestamp, long, "%ld", "long");
	if (0 == pt->timestamp)
		pt->timestamp = time(NULL);

	PARSE_LOCATION(altitude, double, "%lf", "double float");
	PARSE_LOCATION(latitude, double, "%lf", "double float");
	PARSE_LOCATION(longitude, double, "%lf", "double float");

	pt->process_method = mParameters.get("gps-processing-method");
/*
	LOGV("%s: setting image location result %d,  ALT %lf LAT %lf LON %lf",
			__func__, result, pt->altitude, pt->latitude, pt->longitude);
*/
	return result;
}

int SprdCameraHardware::uv420CopyTrim(struct _dma_copy_cfg_tag dma_copy_cfg)
{
	uint32_t i = 0;
	uint32_t src_y_addr = 0, src_uv_addr = 0,  dst_y_addr = 0,  dst_uv_addr = 0;

	if (DMA_COPY_YUV400 <= dma_copy_cfg.format ||
		(dma_copy_cfg.src_size.w & 0x01) || (dma_copy_cfg.src_size.h & 0x01) ||
		(dma_copy_cfg.src_rec.x & 0x01) || (dma_copy_cfg.src_rec.y & 0x01) ||
		(dma_copy_cfg.src_rec.w & 0x01) || (dma_copy_cfg.src_rec.h & 0x01) ||
		0 == dma_copy_cfg.src_addr.y_addr || 0 == dma_copy_cfg.src_addr.uv_addr ||
		0 == dma_copy_cfg.dst_addr.y_addr || 0 == dma_copy_cfg.dst_addr.uv_addr ||
		0 == dma_copy_cfg.src_rec.w || 0 == dma_copy_cfg.src_rec.h ||
		0 == dma_copy_cfg.src_size.w|| 0 == dma_copy_cfg.src_size.h ||
		(dma_copy_cfg.src_rec.x + dma_copy_cfg.src_rec.w > dma_copy_cfg.src_size.w) ||
		(dma_copy_cfg.src_rec.y + dma_copy_cfg.src_rec.h > dma_copy_cfg.src_size.h)) {
		LOGE("uv420CopyTrim: param is error. \n");
		return -1;
	}

	src_y_addr = dma_copy_cfg.src_addr.y_addr + dma_copy_cfg.src_rec.y *
			dma_copy_cfg.src_size.w + dma_copy_cfg.src_rec.x;
	src_uv_addr = dma_copy_cfg.src_addr.uv_addr + ((dma_copy_cfg.src_rec.y * dma_copy_cfg.src_size.w) >> 1) +
			dma_copy_cfg.src_rec.x;
	dst_y_addr = dma_copy_cfg.dst_addr.y_addr;
	dst_uv_addr = dma_copy_cfg.dst_addr.uv_addr;

	for (i = 0; i < dma_copy_cfg.src_rec.h; i++) {
		memcpy((void *)dst_y_addr, (void *)src_y_addr, dma_copy_cfg.src_rec.w);
		src_y_addr += dma_copy_cfg.src_size.w;
		dst_y_addr += dma_copy_cfg.src_rec.w;

		if (0 == (i & 0x01)) {
			memcpy((void *)dst_uv_addr, (void *)src_uv_addr, dma_copy_cfg.src_rec.w);
			src_uv_addr += dma_copy_cfg.src_size.w;
			dst_uv_addr += dma_copy_cfg.src_rec.w;
		}
	}

	return 0;
}

int SprdCameraHardware::displayCopy(uint32_t dst_phy_addr, uint32_t dst_virtual_addr,
		uint32_t src_phy_addr, uint32_t src_virtual_addr, uint32_t src_w, uint32_t src_h)
{
	int ret = 0;
	struct _dma_copy_cfg_tag dma_copy_cfg;

	if (!mPreviewWindow || !mGrallocHal)
		return -EOWNERDEAD;

	if (0 == s_mem_method) {
#ifdef CONFIG_CAMERA_DMA_COPY
		dma_copy_cfg.format = DMA_COPY_YUV420;
		dma_copy_cfg.src_size.w = src_w;
		dma_copy_cfg.src_size.h = src_h;
		dma_copy_cfg.src_rec.x = mPreviewWidth_trimx;
		dma_copy_cfg.src_rec.y = mPreviewHeight_trimy;
		dma_copy_cfg.src_rec.w = mPreviewWidth_backup;
		dma_copy_cfg.src_rec.h = mPreviewHeight_backup;
		dma_copy_cfg.src_addr.y_addr = src_phy_addr;
		dma_copy_cfg.src_addr.uv_addr = src_phy_addr + dma_copy_cfg.src_size.w * dma_copy_cfg.src_size.h;
		dma_copy_cfg.dst_addr.y_addr = dst_phy_addr;
		if ((0 == dma_copy_cfg.src_rec.x) && (0 == dma_copy_cfg.src_rec.y) &&
			(dma_copy_cfg.src_size.w == dma_copy_cfg.src_rec.w) &&
			(dma_copy_cfg.src_size.h == dma_copy_cfg.src_rec.h)) {
			dma_copy_cfg.dst_addr.uv_addr = dst_phy_addr + dma_copy_cfg.src_size.w * dma_copy_cfg.src_size.h;
		} else {
			dma_copy_cfg.dst_addr.uv_addr = dst_phy_addr + dma_copy_cfg.src_rec.w * dma_copy_cfg.src_rec.h;
		}
		ret = camera_dma_copy_data(dma_copy_cfg);
#else

#ifdef CONFIG_CAMERA_ANTI_SHAKE
		dma_copy_cfg.format = DMA_COPY_YUV420;
		dma_copy_cfg.src_size.w = src_w;
		dma_copy_cfg.src_size.h = src_h;
		dma_copy_cfg.src_rec.x = mPreviewWidth_trimx;
		dma_copy_cfg.src_rec.y = mPreviewHeight_trimy;
		dma_copy_cfg.src_rec.w = mPreviewWidth_backup;
		dma_copy_cfg.src_rec.h = mPreviewHeight_backup;
		dma_copy_cfg.src_addr.y_addr = src_virtual_addr;
		dma_copy_cfg.src_addr.uv_addr = src_virtual_addr + dma_copy_cfg.src_size.w * dma_copy_cfg.src_size.h;
		dma_copy_cfg.dst_addr.y_addr = dst_virtual_addr;
		if ((0 == dma_copy_cfg.src_rec.x) && (0 == dma_copy_cfg.src_rec.y) &&
			(dma_copy_cfg.src_size.w == dma_copy_cfg.src_rec.w) &&
			(dma_copy_cfg.src_size.h == dma_copy_cfg.src_rec.h)) {
			dma_copy_cfg.dst_addr.uv_addr = dst_virtual_addr + dma_copy_cfg.src_size.w * dma_copy_cfg.src_size.h;
		} else {
			dma_copy_cfg.dst_addr.uv_addr = dst_virtual_addr + dma_copy_cfg.src_rec.w * dma_copy_cfg.src_rec.h;
		}
		ret = uv420CopyTrim(dma_copy_cfg);
#else
		if (mIsDvPreview) {
			memcpy((void *)dst_virtual_addr, (void *)src_virtual_addr, SIZE_ALIGN(src_w)*SIZE_ALIGN(src_h)*3/2);
		} else {
			memcpy((void *)dst_virtual_addr, (void *)src_virtual_addr, src_w*src_h*3/2);
		}
#endif

#endif
	} else {
		if (mIsDvPreview) {
			memcpy((void *)dst_virtual_addr, (void *)src_virtual_addr, SIZE_ALIGN(src_w) * SIZE_ALIGN(src_h) * 3/2);
		} else {
			memcpy((void *)dst_virtual_addr, (void *)src_virtual_addr, src_w*src_h*3/2);
		}
	}
	return ret;
}

bool SprdCameraHardware::displayOneFrameForCapture(uint32_t width, uint32_t height, uint32_t phy_addr, char *virtual_addr)
{
	LOGI("%s: size = %dx%d, addr = %d", __func__, width, height, phy_addr);
	Mutex::Autolock cbLock(&mPreviewCbLock);

	buffer_handle_t *buf_handle = NULL;
	int stride = 0;
	void *vaddr = NULL;
	int ret = 0;
	struct _dma_copy_cfg_tag dma_copy_cfg;
	struct private_handle_t *private_h = NULL;
	uint32_t dst_phy_addr = 0;

	Mutex::Autolock pwl(&mPreviewWindowLock);

	if (!mPreviewWindow || !mGrallocHal || 0 == phy_addr) {
		return false;
	}

	ret = mPreviewWindow->dequeue_buffer(mPreviewWindow, &buf_handle, &stride);
	if (0 != ret) {
		LOGE("%s: failed to dequeue gralloc buffer!", __func__);
		return false;
	}

	ret = mGrallocHal->lock(mGrallocHal, *buf_handle, GRALLOC_USAGE_SW_WRITE_OFTEN,
				0, 0, SIZE_ALIGN(width), SIZE_ALIGN(height), &vaddr);

	if (0 != ret || NULL == vaddr) {
		LOGE("%s: failed to lock gralloc buffer ret=%d,vaddr=0x%x", __func__, ret,(int)vaddr);
		ret = mPreviewWindow->cancel_buffer(mPreviewWindow, buf_handle);
		if (0 != ret) {
			LOGE("%s cancel buffer fail", __func__);
		}
		return false;
	}

	private_h = (struct private_handle_t *)(*buf_handle);
	dst_phy_addr = (uint32_t)(private_h->phyaddr);
	LOGI("displayOneFrameForCapture,0x%x.",(uint32_t)virtual_addr);
	ret = displayCopy(dst_phy_addr, (uint32_t)vaddr, phy_addr, (uint32_t)virtual_addr, width, height);

	mGrallocHal->unlock(mGrallocHal, *buf_handle);

	if (0 != ret) {
		LOGE("%s: camera copy data failed.", __func__);
	}

	ret = mPreviewWindow->enqueue_buffer(mPreviewWindow, buf_handle);
	if (0 != ret) {
		LOGE("%s: enqueue_buffer() failed.", __func__);
		ret = mPreviewWindow->cancel_buffer(mPreviewWindow, buf_handle);
		if (0 != ret) {
			LOGE("%s cancel buffer fail", __func__);
		}
		return false;
	}

	return true;
}


bool SprdCameraHardware::displayOneFrame(uint32_t width, uint32_t height, uint32_t phy_addr, char *virtual_addr, uint32_t id)
{

	void *vaddr = NULL;
	int ret = 0;

	Mutex::Autolock pwl(&mPreviewWindowLock);
	Mutex::Autolock pbl(&mPrevBufLock);

	if (!isPreviewing()) {
		LOGE("not in preview");
		return false;
	}

	if (PREVIEW_BUFFER_USAGE_DCAM == mPreviewBufferUsage) {
		if (!mPreviewWindow || !mGrallocHal || 0 == phy_addr) {
			return false;
		}

		LOGV("%s: size = %dx%d, addr = %d", __func__, width, height, phy_addr);

		buffer_handle_t *buf_handle = NULL;
		int stride = 0;
		struct _dma_copy_cfg_tag dma_copy_cfg;
		struct private_handle_t *private_h = NULL;
		uint32_t dst_phy_addr = 0;

		ret = mPreviewWindow->dequeue_buffer(mPreviewWindow, &buf_handle, &stride);
		if (0 != ret) {
			LOGE("%s: failed to dequeue gralloc buffer!", __func__);
			return false;
		}
		if (mIsDvPreview) {
			ret = mGrallocHal->lock(mGrallocHal, *buf_handle, GRALLOC_USAGE_SW_WRITE_OFTEN,
						0, 0, SIZE_ALIGN(width), SIZE_ALIGN(height), &vaddr);
		} else {
			ret = mGrallocHal->lock(mGrallocHal, *buf_handle, GRALLOC_USAGE_SW_WRITE_OFTEN,
						0, 0, width, height, &vaddr);
		}

		if (0 != ret || NULL == vaddr) {
			LOGE("%s: failed to lock gralloc buffer ret=%d, vaddr=0x%x", __func__,ret,(int)vaddr);
			ret = mPreviewWindow->cancel_buffer(mPreviewWindow, buf_handle);
			if (0 != ret) {
				LOGE("%s cancel buffer fail", __func__);
			}
			return false;
		}

		private_h = (struct private_handle_t *)(*buf_handle);
		dst_phy_addr =  (uint32_t)(private_h->phyaddr);

		if (isPreviewing()) {
			ret = displayCopy(dst_phy_addr, (uint32_t)vaddr, phy_addr, (uint32_t)virtual_addr, width, height);
		} else {
			ret = -ENOSYS;
		}

		mGrallocHal->unlock(mGrallocHal, *buf_handle);

		if (0 != ret) {
			ret = mPreviewWindow->cancel_buffer(mPreviewWindow, buf_handle);
			LOGW("%s: camera copy data skipped.", __func__);
		} else {
			ret = mPreviewWindow->enqueue_buffer(mPreviewWindow, buf_handle);
			if (0 != ret) {
				LOGE("%s: enqueue_buffer() fail.", __func__);
				ret = mPreviewWindow->cancel_buffer(mPreviewWindow, buf_handle);
				if (0 != ret) {
					LOGE("%s cancel buffer fail", __func__);
				}
			}
		}

		if (0 != ret) {
			LOGE("%s: failed in PREVIEW_BUFFER_USAGE_DCAM case.", __func__);
			return false;
		}
	} else {
		if (mCancelBufferEb[id] && (mPreviewCancelBufHandle[id] == mPreviewBufferHandle[id])) {
			LOGW("displayOneFrame skip: Could not enqueue cancel buffer!\n");
			camera_release_frame(id);
			return true;
		} else {
			if (!mPreviewWindow || !mGrallocHal) {
				return false;
			}

			releasePreviewFrame();

			if (mIsDvPreview) {
				ret = mGrallocHal->lock(mGrallocHal, *mPreviewBufferHandle[id], GRALLOC_USAGE_SW_WRITE_OFTEN,
							0, 0, SIZE_ALIGN(width), SIZE_ALIGN(height), &vaddr);
			} else {
				ret = mGrallocHal->lock(mGrallocHal, *mPreviewBufferHandle[id], GRALLOC_USAGE_SW_WRITE_OFTEN,
							0, 0, width, height, &vaddr);
			}
			if (0 != ret || NULL == vaddr) {
				LOGE("%s: failed to lock buffer ret=%d, vaddr=0x%x id=%d",
					__func__,ret,(int)vaddr, id);
				return false;
			}

			mGrallocHal->unlock(mGrallocHal, *mPreviewBufferHandle[id]);
			if (0 != mPreviewWindow->enqueue_buffer(mPreviewWindow, mPreviewBufferHandle[id])) {
				LOGE("displayOneFrame fail: Could not enqueue gralloc buffer!\n");

				if (0 != mPreviewWindow->cancel_buffer(mPreviewWindow, mPreviewBufferHandle[id])) {
					LOGE("%s cancel buffer fail", __func__);
				}

				return false;
			}
			mGraphBufCntLock.lock();
			mGraphBufferCount[id]++;
			mGraphBufCntLock.unlock();
			mCancelBufferEb[id] = 0;
		}
	}
	return true;
}

void SprdCameraHardware::receivePreviewFDFrame(camera_frame_type *frame)
{
	/*Mutex::Autolock cbLock(&mPreviewCbLock);*/

	if (NULL == frame) {
		LOGE("receivePreviewFDFrame: invalid frame pointer");
		return;
	}

	ssize_t offset = frame->buf_id;
	camera_frame_metadata_t metadata;
	camera_face_t face_info[FACE_DETECT_NUM];
	int32_t k = 0;

	LOGI("receive face_num %d.",frame->face_num);
	metadata.number_of_faces = frame->face_num <= FACE_DETECT_NUM ? frame->face_num:FACE_DETECT_NUM;
	if (0 != metadata.number_of_faces) {
		for(k=0 ; k< metadata.number_of_faces ; k++) {
			face_info[k].id = k;
			face_info[k].rect[0] = (frame->face_ptr->sx*2000/mPreviewWidth)-1000;
			face_info[k].rect[1] = (frame->face_ptr->sy*2000/mPreviewHeight)-1000;
			face_info[k].rect[2] = (frame->face_ptr->ex*2000/mPreviewWidth)-1000;
			face_info[k].rect[3] = (frame->face_ptr->ey*2000/mPreviewHeight)-1000;
			LOGI("smile level %d.\n",frame->face_ptr->smile_level);
			face_info[k].score = frame->face_ptr->smile_level;
			frame->face_ptr++;
		}
	}

	metadata.faces = &face_info[0];
	if (mMsgEnabled&CAMERA_MSG_PREVIEW_METADATA) {
		LOGV("smile capture msg is enabled.");
		if (PREVIEW_BUFFER_USAGE_DCAM == mPreviewBufferUsage) {
			uint32_t tmpIndex = offset;
			if (camera_get_rot_set()) {
			tmpIndex += kPreviewBufferCount;
			}
			handleDataCallback(CAMERA_MSG_PREVIEW_METADATA,
			tmpIndex,
			0, &metadata, mUser, 1);
		} else {
			uint32_t dataSize = frame->dx * frame->dy * 3 / 2;
			memcpy(mPreviewHeapArray[mPreviewDcamAllocBufferCnt -1]->camera_memory->data,
			frame->buf_Virt_Addr, dataSize);
			handleDataCallback(CAMERA_MSG_PREVIEW_METADATA,
			mPreviewDcamAllocBufferCnt -1,
			0, &metadata, mUser, 1);
		}
	} else {
		LOGV("smile capture msg is disabled.");
	}
}

void SprdCameraHardware::handleDataCallback(int32_t msg_type,
		uint32_t frame_index, unsigned int index,
		camera_frame_metadata_t *metadata, void *user,
		uint32_t isPrev)
{
	sprd_camera_memory_t *data;
	if (isPrev)
		Mutex::Autolock l(&mCbPrevDataBusyLock);
	else
		Mutex::Autolock cl(&mCbCapDataBusyLock);

	LOGV("handleDataCallback E");
	if (isPrev) {
		if (!isPreviewing()) return;
		data = mPreviewHeapArray[frame_index];
	} else {
		if (!isCapturing()) return;
		data = mRawHeap;
	}
	data->busy_flag = true;
	mData_cb(msg_type, data->camera_memory, index, metadata, user);
	data->busy_flag = false;
	LOGV("handleDataCallback X");
}

void SprdCameraHardware::handleDataCallbackTimestamp(int64_t timestamp,
		int32_t msg_type,
		sprd_camera_memory_t *data, unsigned int index,
		void *user)
{
	Mutex::Autolock l(&mCbPrevDataBusyLock);
	LOGV("handleDataCallbackTimestamp E");
	if (!isPreviewing()) return;
	data->busy_flag = true;
	mData_cb_timestamp(timestamp, msg_type, data->camera_memory, index, user);
	data->busy_flag = false;
	LOGV("handleDataCallbackTimestamp X");
}

void SprdCameraHardware::cameraBakMemCheckAndFree()
{
	if(NO_ERROR == mCbPrevDataBusyLock.tryLock()) {
		Mutex::Autolock pbdLock(&mPrevBakDataLock);
		/*preview bak heap check and free*/
		if ((false == mPreviewHeapInfoBak.busy_flag) &&
			(1 == mPreviewHeapBakUseFlag)) {
			LOGI("cameraBakMemCheckkAndFree free prev bak mem");
			clearCameraMem(&mPreviewHeapInfoBak);
			mPreviewHeapBakUseFlag = 0;
			LOGI("cameraBakMemCheckkAndFree previewHeapBak free OK");
		}
		mCbPrevDataBusyLock.unlock();
	}

	if(NO_ERROR == mCbCapDataBusyLock.tryLock()) {
		Mutex::Autolock cbdLock(&mCapBakDataLock);
		/* capture head check and free*/
		if ((false == mRawHeapInfoBak.busy_flag) &&
			(1 == mRawHeapBakUseFlag)) {
			LOGI("cameraBakMemCheckkAndFree free cap bak mem");
			clearCameraMem(&mRawHeapInfoBak);
			mRawHeapBakUseFlag = 0;
			LOGI("cameraBakMemCheckkAndFree rawHeapBak free OK");
		}
		mCbCapDataBusyLock.unlock();
	}
}

void SprdCameraHardware::receivePreviewFrame(camera_frame_type *frame)
{
	Mutex::Autolock cbLock(&mPreviewCbLock);
	bool is_preview = isPreviewing();
	if (NULL == frame) {
		LOGE("receivePreviewFrame: invalid frame pointer");
		return;
	}

	ssize_t offset = frame->buf_id;
	camera_frame_metadata_t metadata;
	camera_face_t face_info[FACE_DETECT_NUM];
	uint32_t k = 0;
	int width, height, frame_size, offset_size;

	width = frame->dx;/*mPreviewWidth;*/
	height = frame->dy;/*mPreviewHeight;*/
	if (!is_preview)
		LOGI("receivePreviewFrame E: width=%d, height=%d offset=0x%x\n",width, height, offset);
	else
		LOGV("receivePreviewFrame E: width=%d, height=%d offset=0x%x\n",width, height, offset);

	if (miSPreviewFirstFrame) {
		GET_END_TIME;
		GET_USE_TIME;
		LOGE("Launch Camera Time:%d(ms).",s_use_time);

		float cam_init_time;
		if (getApctCamInitSupport()) {
			cam_init_time = ((float)(systemTime() - cam_init_begin_time))/1000000000;
			writeCamInitTimeToProc(cam_init_time);
		}
		miSPreviewFirstFrame = 0;
	}

	if (isPreviewing()) {
		if (!displayOneFrame(width, height, frame->buffer_phy_addr, (char *)frame->buf_Virt_Addr, frame->buf_id)) {
			LOGE("%s: displayOneFrame not successful!", __func__);
		}
	} else {
		LOGE("receivePreviewFrame X not in preview status, direct return!");
		return;
	}

#ifdef CONFIG_CAMERA_ISP
	send_img_data(2, mPreviewWidth, mPreviewHeight, (char *)frame->buf_Virt_Addr, frame->dx * frame->dy * 3 /2);
#endif

	if(mData_cb != NULL)
	{
		LOGV("receivePreviewFrame mMsgEnabled: 0x%x",mMsgEnabled);
		if (mMsgEnabled & CAMERA_MSG_PREVIEW_FRAME) {
			if (PREVIEW_BUFFER_USAGE_DCAM == mPreviewBufferUsage) {
				uint32_t tmpIndex = frame->order_buf_id;
				handleDataCallback(CAMERA_MSG_PREVIEW_FRAME,
					tmpIndex,
					0, NULL, mUser, 1);
			} else {
				uint32_t dataSize = frame->dx * frame->dy * 3 / 2;
				memcpy(mPreviewHeapArray[mPreviewDcamAllocBufferCnt -1]->camera_memory->data,
					frame->buf_Virt_Addr, dataSize);
				handleDataCallback(CAMERA_MSG_PREVIEW_FRAME,
					mPreviewDcamAllocBufferCnt - 1,
					0, NULL, mUser, 1);
			}
		}

		if ((mMsgEnabled & CAMERA_MSG_VIDEO_FRAME) && isRecordingMode()) {
			nsecs_t timestamp = frame->timestamp;
			LOGI("test timestamp = %lld, mIsStoreMetaData: %d. buffer_id 0x%x",
				timestamp,
				mIsStoreMetaData,
				frame->buf_id);
			if (mTimeCoeff > 1) {
				if (0 != mRecordingFirstFrameTime) {
					timestamp = mRecordingFirstFrameTime + (timestamp - mRecordingFirstFrameTime)*mTimeCoeff;
				} else {
					mRecordingFirstFrameTime = timestamp;
					LOGI("first frame.");
				}
			}
			if (mIsStoreMetaData) {
				uint32_t tmpIndex = frame->order_buf_id;
				uint32_t *data = (uint32_t *)mMetadataHeap->data + offset * METADATA_SIZE / 4;
				*data++ = kMetadataBufferTypeCameraSource;
				*data++ = frame->buffer_phy_addr;
				*data++ = (uint32_t)frame->buf_Virt_Addr;
				*data++ = width;
				*data++ = height;
				*data++ = mPreviewWidth_trimx;
				*data = mPreviewHeight_trimy;
				{
					Mutex::Autolock l(&mCbPrevDataBusyLock);
					if(!isPreviewing()) return;
					if (PREVIEW_BUFFER_USAGE_GRAPHICS == mPreviewBufferUsage) {
						tmpIndex = mPreviewDcamAllocBufferCnt - 1;
					}
					mPreviewHeapArray[tmpIndex]->busy_flag = true;
					mData_cb_timestamp(timestamp, CAMERA_MSG_VIDEO_FRAME, mMetadataHeap, offset, mUser);
					mPreviewHeapArray[tmpIndex]->busy_flag = false;
				}
			} else {
				uint32_t tmpIndex = frame->order_buf_id;
				if (PREVIEW_BUFFER_USAGE_GRAPHICS == mPreviewBufferUsage) {
					tmpIndex = mPreviewDcamAllocBufferCnt - 1;
				}
				handleDataCallbackTimestamp(timestamp,
					CAMERA_MSG_VIDEO_FRAME,
					mPreviewHeapArray[tmpIndex],
					0, mUser);
			}
			if (PREVIEW_BUFFER_USAGE_GRAPHICS == mPreviewBufferUsage) {
				Mutex::Autolock pgbcl(&mGraphBufCntLock);
				mGraphBufferCount[offset]++;
			};
		} else {
			if (PREVIEW_BUFFER_USAGE_DCAM == mPreviewBufferUsage) {
				if (CAMERA_SUCCESS != camera_release_frame(offset)) {
					LOGE("receivePreviewFrame: fail to camera_release_frame().offset: %d.", (int)offset);
				}
			}
		}
		// When we are doing preview but not recording, we need to
		// release every preview frame immediately so that the next
		// preview frame is delivered.  However, when we are recording
		// (whether or not we are also streaming the preview frames to
		// the screen), we have the user explicitly release a preview
		// frame via method releaseRecordingFrame().  In this way we
		// allow a video encoder which is potentially slower than the
		// preview stream to skip frames.  Note that we call
		// camera_release_frame() in this method because we first
		// need to check to see if mPreviewCallback != NULL, which
		// requires holding mCallbackLock.
	} else {
		LOGE("receivePreviewFrame: mData_cb is null.");
	}

	if (!is_preview)
		LOGI("receivePreviewFrame X");
	else
		LOGV("receivePreviewFrame X");
}

void SprdCameraHardware::notifyShutter()
{
	LOGI("notifyShutter: E");
	print_time();

	LOGI("notifyShutter mMsgEnabled: 0x%x.", mMsgEnabled);

	if ((CAMERA_ZSL_CONTINUE_SHOT_MODE != mCaptureMode)
		&& (CAMERA_NORMAL_CONTINUE_SHOT_MODE != mCaptureMode)) {
		if (mMsgEnabled & CAMERA_MSG_SHUTTER)
			mNotify_cb(CAMERA_MSG_SHUTTER, 0, 0, mUser);
	} else {
			mNotify_cb(CAMERA_MSG_SHUTTER, 0, 0, mUser);
	}

	print_time();
	LOGI("notifyShutter: X");
}

void SprdCameraHardware::receiveRawPicture(camera_frame_type *frame)
{
	LOGI("receiveRawPicture: E");

	print_time();

	Mutex::Autolock cbLock(&mCaptureCbLock);

	if (NULL == frame) {
		LOGE("receiveRawPicture: invalid frame pointer");
		return;
	}

	if (SPRD_INTERNAL_CAPTURE_STOPPING == getCaptureState()) {
		LOGW("receiveRawPicture: warning: capture state = SPRD_INTERNAL_CAPTURE_STOPPING, return \n");
		return;
	}

	if (iSDisplayCaptureFrame()) {
		uint32_t dst_paddr = 0;
		uint32_t dst_width = mPreviewWidth;
		uint32_t dst_height = mPreviewHeight;

		dst_paddr = getRedisplayMem();

		if (0 == dst_paddr) {
			LOGE("%s: get review memory failed", __func__);
			goto callbackraw;
		}

		if ( 0 != camera_get_data_redisplay(dst_paddr, dst_width, dst_height, frame->buffer_phy_addr,
				frame->buffer_uv_phy_addr, frame->dx, frame->dy)) {
			LOGE("%s: Fail to camera_get_data_redisplay.", __func__);
			FreeReDisplayMem();
			goto callbackraw;
		}

		if (!displayOneFrameForCapture(dst_width, dst_height, dst_paddr, (char *)mReDisplayHeap->data)) {
			LOGE("%s: displayOneFrame failed", __func__);
		}

		FreeReDisplayMem();
	}

callbackraw:
	if (mData_cb!= NULL) {
		ssize_t offset = (uint32_t)frame->buf_Virt_Addr;
		offset -= (uint32_t)mRawHeap->data;
		ssize_t frame_size = 0;

		if (CAMERA_RGB565 == frame->format)
			frame_size = frame->dx * frame->dy * 2;
		else if (CAMERA_YCBCR_4_2_2 == frame->format)
			frame_size = frame->dx * frame->dy * 2;
		else if (CAMERA_YCBCR_4_2_0 == frame->format)
			frame_size = frame->dx * frame->dy * 3 / 2;
		else
			frame_size = frame->dx * frame->dy * 2;

		if (offset + frame_size <= (ssize_t)mRawHeap->phys_size) {
			offset /= frame_size;

			LOGD("mMsgEnabled: 0x%x, offset: %d.",mMsgEnabled, (uint32_t)offset);

			if (mMsgEnabled & CAMERA_MSG_RAW_IMAGE) {
				handleDataCallback(CAMERA_MSG_RAW_IMAGE, 0, offset, NULL, mUser, 0);
			}

			if (mMsgEnabled & CAMERA_MSG_RAW_IMAGE_NOTIFY) {
				LOGD("mMsgEnabled & CAMERA_MSG_RAW_IMAGE_NOTIFY");
				mNotify_cb(CAMERA_MSG_RAW_IMAGE_NOTIFY, 0,0,mUser);
			}
		} else {
			LOGE("receiveRawPicture: virtual address %p is out of range!", frame->buf_Virt_Addr);
		}
	} else {
		LOGD("Raw-picture callback was canceled--skipping.");
	}

	print_time();
	LOGI("receiveRawPicture: X");
}

void SprdCameraHardware::receivePostLpmRawPicture(camera_frame_type *frame)
{
	LOGI("receivePostLpmRawPicture: E");
	print_time();

	Mutex::Autolock cbLock(&mCaptureCbLock);

	if (NULL == frame) {
		LOGE("receivePostLpmRawPicture: invalid frame pointer");
		return;
	}

	if (mData_cb!= NULL) {
		mJpegSize = 0;
		camera_handle_type camera_handle;
		if(CAMERA_SUCCESS != camera_encode_picture(frame, &camera_handle, camera_cb, this)) {
			setCameraState(SPRD_ERROR, STATE_CAPTURE);
			freeCameraMem(mRawHeap);
			mRawHeap = NULL;
			LOGE("receivePostLpmRawPicture: fail to camera_encode_picture().");
		}
	} else {
		LOGW("JPEG callback was cancelled--not encoding image.");
		// We need to keep the raw heap around until the JPEG is fully
		// encoded, because the JPEG encode uses the raw image contained in
		// that heap.
	}

	print_time();
	LOGI("receivePostLpmRawPicture: X");
}

void SprdCameraHardware::receiveJpegPictureFragment( JPEGENC_CBrtnType *encInfo)
{
	LOGI("receiveJpegPictureFragment E.");
	Mutex::Autolock cbLock(&mCaptureCbLock);

	if (NULL == encInfo) {
		LOGE("receiveJpegPictureFragment: invalid enc info pointer");
		return;
	}
	LOGD("receiveJpegPictureFragment ptr val: encInfo 0x%x", (uint32_t)encInfo);

	camera_encode_mem_type *enc = (camera_encode_mem_type *)encInfo->outPtr;
	uint8_t *base = (uint8_t *)mJpegHeap->mHeap->base();
	LOGD("receiveJpegPictureFragment base ptr 0x%x", (uint32_t)base);
	uint32_t size = encInfo->size;
	uint32_t remaining = mJpegHeap->mHeap->virtualSize();
	LOGD("receiveJpegPictureFragment remaining size 0x%x mjpeg size 0x%x", remaining, mJpegSize);

	if (remaining > mJpegSize) {
		remaining -= mJpegSize;
	} else {
		LOGE("size exceed, abnormal!");
	}

	LOGV("receiveJpegPictureFragment: (status %d size %d remaining %d mJpegSize %d)",
		encInfo->status,
		size, remaining,mJpegSize);

	if (size > remaining) {
		LOGE("receiveJpegPictureFragment: size %d exceeds what "
		"remains in JPEG heap (%d), truncating",
		size,
		remaining);
		size = remaining;
	}

	LOGI("receiveJpegPictureFragment : base + mJpegSize: %x, enc->buffer: %x, size: %x", (uint32_t)(base + mJpegSize), (uint32_t)enc->buffer, size);

	mJpegSize += size;

	LOGI("receiveJpegPictureFragment X.");
}

void SprdCameraHardware::receiveJpegPosPicture(void)
{
	LOGI("receiveJpegPosPicture: E");
	print_time();

	Mutex::Autolock cbLock(&mCaptureCbLock);

	if (mData_cb!= NULL) {
		bool encode_location = true;
		camera_position_type pt = {0, 0, 0, 0, NULL};

		encode_location = getCameraLocation(&pt);
		if (encode_location) {
			if (camera_set_position(&pt, NULL, NULL) != CAMERA_SUCCESS) {
				LOGE("%s: camera_set_position: error", __func__);
			}
		} else {
			LOGV("%s: not setting image location", __func__);
		}

		mJpegSize = 0;
	} else {
		LOGV("%s: JPEG callback was cancelled--not encoding image.", __func__);
		// We need to keep the raw heap around until the JPEG is fully
		// encoded, because the JPEG encode uses the raw image contained in
		// that heap.
		if (!iSZslMode()) {
			deinitCapture();
		}
	}

	print_time();
	LOGI("%s: receiveJpegPosPicture X: free mCallbackLock!", __func__);
}

void SprdCameraHardware::receiveJpegPicture(JPEGENC_CBrtnType *encInfo)
{
	GET_END_TIME;
	GET_USE_TIME;
	camera_encode_mem_type *enc = (camera_encode_mem_type *)encInfo->outPtr;
	LOGI("receiveJpegPicture: E image (%d bytes out of %d) Time %d(ms)",
		mJpegSize, mJpegHeap->mBufferSize, s_use_time);
	print_time();
	Mutex::Autolock cbLock(&mCaptureCbLock);

	int index = 0;

	if (mData_cb) {
		LOGI("receiveJpegPicture: mData_cb.");
		// The reason we do not allocate into mJpegHeap->mBuffers[offset] is
		// that the JPEG image's size will probably change from one snapshot
		// to the next, so we cannot reuse the MemoryBase object.
		LOGD("receiveJpegPicture: mMsgEnabled: 0x%x.", mMsgEnabled);

		if ((CAMERA_ZSL_CONTINUE_SHOT_MODE != mCaptureMode)
			&& (CAMERA_NORMAL_CONTINUE_SHOT_MODE != mCaptureMode)) {
			if (mMsgEnabled & CAMERA_MSG_COMPRESSED_IMAGE){
				camera_memory_t *mem = mGetMemory_cb(-1, mJpegSize, 1, 0);
				memcpy(mem->data, enc->buffer, mJpegSize);
				mData_cb(CAMERA_MSG_COMPRESSED_IMAGE,mem, 0, NULL, mUser );
				mem->release(mem);
			}
		} else {
			camera_memory_t *mem = mGetMemory_cb(-1, mJpegSize, 1, 0);
			memcpy(mem->data, enc->buffer, mJpegSize);
			mData_cb(CAMERA_MSG_COMPRESSED_IMAGE,mem, 0, NULL, mUser );
			mem->release(mem);
		}
	} else {
		LOGV("JPEG callback was cancelled--not delivering image.");
	}

	// NOTE: the JPEG encoder uses the raw image contained in mRawHeap, so we need
	// to keep the heap around until the encoding is complete.
	LOGI("receiveJpegPicture: free the Raw and Jpeg mem. 0x%p", mRawHeap);

	if (!iSZslMode()) {
		if (encInfo->need_free) {
			deinitCapture();
			set_ddr_freq(BASE_FREQ_REQ);
		}
	} else {
		mCapBufLock.lock();
		if (SPRD_WAITING_JPEG == getCaptureState() && (mCapBufIsAvail == 1)) {
			flush_buffer(CAMERA_FLUSH_RAW_HEAP_ALL, 0,(void*)0,(void*)0,0);
		}
		mCapBufLock.unlock();
	}
	print_time();
	LOGI("receiveJpegPicture: X callback done.");
}

void SprdCameraHardware::receiveJpegPictureError(void)
{
	LOGI("receiveJpegPictureError.");
	print_time();
	Mutex::Autolock cbLock(&mCaptureCbLock);
	if (!checkPreviewStateForCapture()) {
		LOGE("drop current jpegPictureError msg");
		return;
	}

	int index = 0;
	if (mData_cb) {
		LOGI("receiveJpegPicture: mData_cb.");
		if (mMsgEnabled & CAMERA_MSG_COMPRESSED_IMAGE) {
			mData_cb(CAMERA_MSG_COMPRESSED_IMAGE,NULL, 0, NULL, mUser );
		}
	} else {
		LOGI("JPEG callback was cancelled--not delivering image.");
	}

	// NOTE: the JPEG encoder uses the raw image contained in mRawHeap, so we need
	// to keep the heap around until the encoding is complete.

	print_time();
	LOGI("receiveJpegPictureError: X callback done.");
}

void SprdCameraHardware::receiveCameraExitError(void)
{
	Mutex::Autolock cbPreviewLock(&mPreviewCbLock);
	Mutex::Autolock cbCaptureLock(&mCaptureCbLock);

	if (!checkPreviewStateForCapture()) {
		LOGE("drop current cameraExit msg");
		return;
	}
	if ((mMsgEnabled & CAMERA_MSG_ERROR) && (mData_cb != NULL)) {
		LOGE("HandleErrorState");
		mNotify_cb(CAMERA_MSG_ERROR, 0,0,mUser);
	}

	LOGE("HandleErrorState:don't enable error msg!");
}

void SprdCameraHardware::receiveTakePictureError(void)
{
	Mutex::Autolock cbLock(&mCaptureCbLock);

	if (!checkPreviewStateForCapture()) {
		LOGE("drop current takePictureError msg");
		return;
	}

	LOGE("camera cb: invalid state %s for taking a picture!",
		 getCameraStateStr(getCaptureState()));

	if ((mMsgEnabled & CAMERA_MSG_RAW_IMAGE) && (NULL != mData_cb))
		mData_cb(CAMERA_MSG_RAW_IMAGE, NULL, 0 , NULL, mUser);

	if ((mMsgEnabled & CAMERA_MSG_COMPRESSED_IMAGE) && (NULL != mData_cb))
		mData_cb(CAMERA_MSG_COMPRESSED_IMAGE, NULL, 0, NULL, mUser);
}


/*transite from 'from' state to 'to' state and signal the waitting thread. if the current*/
/*state is not 'from', transite to SPRD_ERROR state should be called from the callback*/
SprdCameraHardware::Sprd_camera_state
SprdCameraHardware::transitionState(SprdCameraHardware::Sprd_camera_state from,
		SprdCameraHardware::Sprd_camera_state to,
		SprdCameraHardware::state_owner owner, bool lock)
{
	volatile SprdCameraHardware::Sprd_camera_state *which_ptr = NULL;
	LOGI("transitionState E");

	if (lock) mStateLock.lock();
	LOGI("transitionState: owner = %d, lock = %d", owner, lock);

	switch (owner) {
	case STATE_CAMERA:
		which_ptr = &mCameraState.camera_state;
		break;

	case STATE_PREVIEW:
		which_ptr = &mCameraState.preview_state;
		break;

	case STATE_CAPTURE:
		which_ptr = &mCameraState.capture_state;
		break;

	case STATE_FOCUS:
		which_ptr = &mCameraState.focus_state;
		break;

	default:
		LOGI("changeState: error owner");
		break;
	}

	if (NULL != which_ptr) {
		if (from != *which_ptr) {
			to = SPRD_ERROR;
		}

		LOGI("changeState: %s --> %s", getCameraStateStr(from),
			getCameraStateStr(to));

		if (*which_ptr != to) {
			*which_ptr = to;
			mStateWait.signal();
		}
	}

	if (lock) mStateLock.unlock();
	LOGI("transitionState X");

	return to;
}

void SprdCameraHardware::HandleStartPreview(camera_cb_type cb,
		int32_t parm4)
{
	LOGV("HandleStartPreview in: cb = %d, parm4 = 0x%x, state = %s",
		cb, parm4, getCameraStateStr(getPreviewState()));

	switch(cb) {
	case CAMERA_RSP_CB_SUCCESS:
		setCameraState(SPRD_PREVIEW_IN_PROGRESS, STATE_PREVIEW);
		break;

	case CAMERA_EVT_CB_FRAME:
		LOGV("CAMERA_EVT_CB_FRAME");
		switch (getPreviewState()) {
		case SPRD_PREVIEW_IN_PROGRESS:
			receivePreviewFrame((camera_frame_type *)parm4);
			break;

		case SPRD_INTERNAL_PREVIEW_STOPPING:
			LOGI("camera cb: discarding preview frame "
			"while stopping preview");
			break;

		default:
			LOGW("HandleStartPreview: invalid state");
			break;
			}
		break;

	case CAMERA_EVT_CB_FD:
		LOGI("CAMERA_EVT_CB_FD");
		if (isPreviewing()) {
			receivePreviewFDFrame((camera_frame_type *)parm4);
		}
		break;

	case CAMERA_EXIT_CB_FAILED:
		LOGE("SprdCameraHardware::camera_cb: @CAMERA_EXIT_CB_FAILURE(%d) in state %s.",
			parm4, getCameraStateStr(getPreviewState()));
		transitionState(getPreviewState(), SPRD_ERROR, STATE_PREVIEW);
		receiveCameraExitError();
		break;

	case CAMERA_EVT_CB_FLUSH:
		LOGE("CAMERA_EVT_CB_FLUSH, 0x%x", mPreviewHeapSize);
		{
			camera_frame_type *frame = (camera_frame_type *)parm4;
			mPrevBufLock.lock();
			if (isPreviewing()) {
				flush_buffer(CAMERA_FLUSH_PREVIEW_HEAP, frame->buf_id,
					(void*)frame->buf_Virt_Addr,
					(void*)frame->buffer_phy_addr,
					mPreviewHeapSize);
			}
			mPrevBufLock.unlock();
		}
		break;
	default:
		transitionState(getPreviewState(), SPRD_ERROR, STATE_PREVIEW);
		LOGE("unexpected cb %d for CAMERA_FUNC_START_PREVIEW.", cb);
		break;
	}

	LOGV("HandleStartPreview out, state = %s", getCameraStateStr(getPreviewState()));
}

void SprdCameraHardware::HandleStopPreview(camera_cb_type cb,
		int32_t parm4)
{
	Sprd_camera_state tmpPrevState = SPRD_IDLE;
	tmpPrevState = getPreviewState();
	LOGI("HandleStopPreview in: cb = %d, parm4 = 0x%x, state = %s",
		cb, parm4, getCameraStateStr(tmpPrevState));

	if ((SPRD_IDLE == tmpPrevState) || (SPRD_INTERNAL_PREVIEW_STOPPING == tmpPrevState)) {
		setCameraState(SPRD_IDLE,
			STATE_PREVIEW);
	} else {
		LOGE("HandleEncode: error preview status, %s",
			getCameraStateStr(tmpPrevState));
		transitionState(tmpPrevState,
			SPRD_ERROR,
			STATE_PREVIEW);
	}

	LOGI("HandleStopPreview out, state = %s", getCameraStateStr(getPreviewState()));
}

void SprdCameraHardware::HandleTakePicture(camera_cb_type cb,
		int32_t parm4)
{
	LOGI("HandleTakePicture E: cb = %d, parm4 = 0x%x, state = %s",
		cb, parm4, getCameraStateStr(getCaptureState()));
	bool encode_location = true;
	camera_position_type pt = {0, 0, 0, 0, NULL};

	switch (cb) {
	case CAMERA_EVT_CB_FLUSH:
		LOGD("capture:flush.");
		mCapBufLock.lock();
		if (mCapBufIsAvail == 1) {
			flush_buffer(CAMERA_FLUSH_RAW_HEAP_ALL, 0,(void*)0,(void*)0,0);
		}
		mCapBufLock.unlock();
		break;
	case CAMERA_RSP_CB_SUCCESS:
		LOGI("HandleTakePicture: CAMERA_RSP_CB_SUCCESS");
		if (SPRD_WAITING_RAW == getCaptureState()) {
			LOGI("CAMERA_RSP_CB_SUCCESS has been called before, skip it");
		} else {
			transitionState(SPRD_INTERNAL_RAW_REQUESTED,
				SPRD_WAITING_RAW,
				STATE_CAPTURE);
		}
		break;
	case CAMERA_EVT_CB_CAPTURE_FRAME_DONE:
		LOGI("HandleTakePicture: CAMERA_EVT_CB_CAPTURE_FRAME_DONE");
		if (checkPreviewStateForCapture()) {
			notifyShutter();
		} else {
			LOGE("HandleTakePicture: no shutter");
		}
		break;
	case CAMERA_EVT_CB_SNAPSHOT_DONE:
		LOGI("HandleTakePicture: CAMERA_EVT_CB_SNAPSHOT_DONE");
		encode_location = getCameraLocation(&pt);
		if (encode_location) {
			if (camera_set_position(&pt, NULL, NULL) != CAMERA_SUCCESS) {
			LOGE("receiveRawPicture: camera_set_position: error");
			}
		} else {
			LOGI("receiveRawPicture: not setting image location");
		}
		if (checkPreviewStateForCapture()) {
			receiveRawPicture((camera_frame_type *)parm4);
		} else {
			LOGE("HandleTakePicture: drop current rawPicture");
		}
		break;

	case CAMERA_EXIT_CB_DONE:
		LOGI("HandleTakePicture: CAMERA_EXIT_CB_DONE");
		if (1 != mParameters.getInt("zsl"))
			set_ddr_freq(BASE_FREQ_REQ);
		if (SPRD_WAITING_RAW == getCaptureState())
		{
			transitionState(SPRD_WAITING_RAW,
				((NULL != mData_cb) ? SPRD_WAITING_JPEG : SPRD_IDLE),
				STATE_CAPTURE);
			// It's important that we call receiveRawPicture() before
			// we transition the state because another thread may be
			// waiting in cancelPicture(), and then delete this object.
			// If the order were reversed, we might call
			// receiveRawPicture on a dead object.
			if (checkPreviewStateForCapture()) {
				receivePostLpmRawPicture((camera_frame_type *)parm4);
			} else {
				LOGE("HandleTakePicture drop current LpmRawPicture");
			}
		}
		break;

	case CAMERA_EXIT_CB_FAILED:
		LOGE("SprdCameraHardware::camera_cb: @CAMERA_EXIT_CB_FAILURE(%d) in state %s.",
			parm4, getCameraStateStr(getCaptureState()));
		transitionState(getCaptureState(), SPRD_ERROR, STATE_CAPTURE);
		receiveCameraExitError();
		if (1 != mParameters.getInt("zsl"))
			set_ddr_freq(BASE_FREQ_REQ);
		break;

	default:
		LOGE("HandleTakePicture: unkown cb = %d", cb);
		transitionState(getCaptureState(), SPRD_ERROR, STATE_CAPTURE);
		receiveTakePictureError();
		break;
	}

	LOGI("HandleTakePicture X, state = %s", getCameraStateStr(getCaptureState()));
}

void SprdCameraHardware::HandleCancelPicture(camera_cb_type cb,
		int32_t parm4)
{
	LOGI("HandleCancelPicture E: cb = %d, parm4 = 0x%x, state = %s",
		cb, parm4, getCameraStateStr(getCaptureState()));

	setCameraState(SPRD_IDLE,
			STATE_CAPTURE);

	LOGI("HandleCancelPicture X, state = %s", getCameraStateStr(getCaptureState()));
}

void SprdCameraHardware::HandleEncode(camera_cb_type cb,
		int32_t parm4)
{
	LOGI("HandleEncode E: cb = %d, parm4 = 0x%x, state = %s",
		cb, parm4, getCameraStateStr(getCaptureState()));

	switch (cb) {
	case CAMERA_RSP_CB_SUCCESS:
		// We already transitioned the camera state to
		// SPRD_WAITING_JPEG when we called
		// camera_encode_picture().
		break;

	case CAMERA_EXIT_CB_DONE:
		LOGI("HandleEncode: CAMERA_EXIT_CB_DONE");
		if ((SPRD_WAITING_JPEG == getCaptureState())) {
			Sprd_camera_state tmpCapState= SPRD_WAITING_JPEG;
			receiveJpegPictureFragment((JPEGENC_CBrtnType *)parm4);
			LOGI("CAMERA_EXIT_CB_DONE MID.");
			if (checkPreviewStateForCapture()) {
				receiveJpegPicture((JPEGENC_CBrtnType *)parm4);
			} else {
				LOGE("HandleEncode: drop current jpgPicture");
			}

			tmpCapState = getCaptureState();
			if ((SPRD_WAITING_JPEG == tmpCapState)
				|| (SPRD_INTERNAL_CAPTURE_STOPPING == tmpCapState)) {
				if (((JPEGENC_CBrtnType *)parm4)->need_free) {
					setCameraState(SPRD_IDLE,
						STATE_CAPTURE);
					if (1 != mParameters.getInt("zsl"))
						set_ddr_freq(BASE_FREQ_REQ);
				} else {
					setCameraState(SPRD_INTERNAL_RAW_REQUESTED,
						STATE_CAPTURE);
				}
			} else if (SPRD_IDLE != tmpCapState) {
				LOGE("HandleEncode: CAMERA_EXIT_CB_DONE error cap status, %s",
					getCameraStateStr(tmpCapState));
				transitionState(tmpCapState,
					SPRD_ERROR,
					STATE_CAPTURE);
				if (1 != mParameters.getInt("zsl"))
					set_ddr_freq(BASE_FREQ_REQ);
			}
		}
		break;

	case CAMERA_EXIT_CB_FAILED:
		LOGI("HandleEncode: CAMERA_EXIT_CB_FAILED");
		transitionState(getCaptureState(), SPRD_ERROR, STATE_CAPTURE);
		receiveCameraExitError();
		if (1 != mParameters.getInt("zsl"))
			set_ddr_freq(BASE_FREQ_REQ);
		break;

	default:
		LOGI("HandleEncode: unkown error = %d", cb);
		transitionState(getCaptureState(), SPRD_ERROR, STATE_CAPTURE);
		receiveJpegPictureError();
		break;
	}

	LOGV("HandleEncode X, state = %s", getCameraStateStr(getCaptureState()));
}

void SprdCameraHardware::HandleFocus(camera_cb_type cb,
		int32_t parm4)
{
	LOGI("HandleFocus E: cb = %d, parm4 = 0x%x, state = %s",
		cb, parm4, getCameraStateStr(getPreviewState()));

	if (NULL == mNotify_cb) {
		LOGE("HandleFocus: mNotify_cb is NULL");
		setCameraState(SPRD_IDLE, STATE_FOCUS);
		return;
	}

	setCameraState(SPRD_IDLE, STATE_FOCUS);

	switch (cb) {
	case CAMERA_RSP_CB_SUCCESS:
		LOGI("camera cb: autofocus has started.");
		break;

	case CAMERA_EXIT_CB_DONE:
		LOGI("camera cb: autofocus succeeded.");
		LOGV("camera cb: autofocus mNotify_cb start.");
		if (mMsgEnabled & CAMERA_MSG_FOCUS)
			mNotify_cb(CAMERA_MSG_FOCUS, 1, 0, mUser);
		else
			LOGE("camera cb: mNotify_cb is null.");

		LOGV("camera cb: autofocus mNotify_cb ok.");
		break;
	case CAMERA_EXIT_CB_ABORT:
		LOGE("camera cb: autofocus aborted");
		break;

	case CAMERA_EXIT_CB_FAILED:
		LOGE("camera cb: autofocus failed");
		if (mMsgEnabled & CAMERA_MSG_FOCUS)
			mNotify_cb(CAMERA_MSG_FOCUS, 0, 0, mUser);
		break;

	case CAMERA_EVT_CB_FOCUS_MOVE:
		LOGE("camera cb: focus moving  %d", parm4);
		mNotify_cb(CAMERA_MSG_FOCUS_MOVE, parm4, 0, mUser);
		break;

	default:
		LOGE("camera cb: unknown cb %d for CAMERA_FUNC_START_FOCUS!", cb);
		if (mMsgEnabled & CAMERA_MSG_FOCUS)
			mNotify_cb(CAMERA_MSG_FOCUS, 0, 0, mUser);
		break;
	}

	LOGI("HandleFocus out, state = %s", getCameraStateStr(getFocusState()));
}

void SprdCameraHardware::HandleStartCamera(camera_cb_type cb,
		int32_t parm4)
{
	LOGI("HandleCameraStart in: cb = %d, parm4 = 0x%x, state = %s",
		cb, parm4, getCameraStateStr(getCameraState()));

	transitionState(SPRD_INIT, SPRD_IDLE, STATE_CAMERA);

	LOGI("HandleCameraStart out, state = %s", getCameraStateStr(getCameraState()));
}

void SprdCameraHardware::HandleStopCamera(camera_cb_type cb, int32_t parm4)
{
	LOGI("HandleStopCamera in: cb = %d, parm4 = 0x%x, state = %s",
		cb, parm4, getCameraStateStr(getCameraState()));

	transitionState(SPRD_INTERNAL_STOPPING, SPRD_INIT, STATE_CAMERA);

	LOGI("HandleStopCamera out, state = %s", getCameraStateStr(getCameraState()));
}

void SprdCameraHardware::camera_cb(camera_cb_type cb,
		const void *client_data,
		camera_func_type func,
		int32_t parm4)
{
	SprdCameraHardware *obj = (SprdCameraHardware *)client_data;

	switch(func) {
	case CAMERA_FUNC_START_PREVIEW:
		obj->HandleStartPreview(cb, parm4);
		break;

	case CAMERA_FUNC_STOP_PREVIEW:
		obj->HandleStopPreview(cb, parm4);
		break;

	case CAMERA_FUNC_RELEASE_PICTURE:
		obj->HandleCancelPicture(cb, parm4);
		break;

	case CAMERA_FUNC_TAKE_PICTURE:
		obj->HandleTakePicture(cb, parm4);
		break;

	case CAMERA_FUNC_ENCODE_PICTURE:
		obj->HandleEncode(cb, parm4);
		break;

	case CAMERA_FUNC_START_FOCUS:
		obj->HandleFocus(cb, parm4);
		break;

	case CAMERA_FUNC_START:
		obj->HandleStartCamera(cb, parm4);
		break;

	case CAMERA_FUNC_STOP:
		obj->HandleStopCamera(cb, parm4);
		break;

	default:
		LOGE("Unknown camera-callback status %d", cb);
		break;
	}
}

int SprdCameraHardware::switch_monitor_thread_init(void *p_data)
{
	struct cmr_msg message = {0, 0, 0, 0};
	int ret = NO_ERROR;
	pthread_attr_t attr;

	SprdCameraHardware *obj = (SprdCameraHardware *)p_data;

	LOGI("switch monitor thread init, %d", obj->mSwitchMonitorInited);

	if (!obj->mSwitchMonitorInited) {
		ret = cmr_msg_queue_create(SWITCH_MONITOR_QUEUE_SIZE, &obj->mSwitchMonitorMsgQueHandle);
		if (ret) {
			LOGE("NO Memory, Failed to create switch monitor message queue\n");
			return ret;
		}
		sem_init(&obj->mSwitchMonitorSyncSem, 0, 0);
		pthread_attr_init(&attr);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
		ret = pthread_create(&obj->mSwitchMonitorThread,
			&attr,
			switch_monitor_thread_proc,
			(void *)obj);
		obj->mSwitchMonitorInited = 1;
		message.msg_type = CMR_EVT_SW_MON_INIT;
		message.data = NULL;
		ret = cmr_msg_post(obj->mSwitchMonitorMsgQueHandle, &message, 1);
		if (ret) {
			LOGE("switch_monitor_thread_init Fail to send one msg!");
		}
		sem_wait(&obj->mSwitchMonitorSyncSem);
	}
	return ret;
}

int SprdCameraHardware::switch_monitor_thread_deinit(void *p_data)
{
	struct cmr_msg message = {0, 0, 0, 0};
	int ret = NO_ERROR;
	SprdCameraHardware * obj = (SprdCameraHardware *)p_data;

	LOGI("switch monitor thread deinit inited, %d", obj->mSwitchMonitorInited);

	if (obj->mSwitchMonitorInited) {
		message.msg_type = CMR_EVT_SW_MON_EXIT;
		ret = cmr_msg_post(obj->mSwitchMonitorMsgQueHandle, &message, 1);
		if (ret) {
			LOGE("Fail to send one msg to camera callback thread");
		}
		sem_wait(&obj->mSwitchMonitorSyncSem);
		sem_destroy(&obj->mSwitchMonitorSyncSem);
		cmr_msg_queue_destroy(obj->mSwitchMonitorMsgQueHandle);
		obj->mSwitchMonitorMsgQueHandle = 0;
		obj->mSwitchMonitorInited = 0;
	}
	return ret ;
}

void SprdCameraHardware::sync_bak_parameters()
{
	Mutex::Autolock l(&mParamLock);

	if (SPRD_IDLE == mCameraState.setParam_state) {
		/*update the bakParameters if there exist difference*/
		if (0 == checkSetParameters(mSetParametersBak, mParameters)) {
			copyParameters(mSetParametersBak, mParameters);
		}
	}
}

void * SprdCameraHardware::switch_monitor_thread_proc(void *p_data)
{
	struct cmr_msg message = {0, 0, 0, 0};
	int exit_flag = 0;
	int ret = NO_ERROR;
	SprdCameraHardware * obj = (SprdCameraHardware *)p_data;

	while (1) {
		ret = cmr_msg_timedget(obj->mSwitchMonitorMsgQueHandle, &message);
		if (CMR_MSG_NO_OTHER_MSG == ret) {
			if (obj->checkSetParameters(obj->mParameters, obj->mSetParametersBak) &&
				obj->mBakParamFlag) {
				obj->setCameraState(SPRD_SET_PARAMS_IN_PROGRESS, STATE_SET_PARAMS);
				LOGD("switch_monitor_thread_proc, bak set");
				obj->setParametersInternal(obj->mSetParametersBak);
				obj->setCameraState(SPRD_IDLE, STATE_SET_PARAMS);
			} else {
				obj->sync_bak_parameters();
			}
			obj->cameraBakMemCheckAndFree();
		} else if (NO_ERROR != ret) {
			CMR_LOGE("Message queue destroyed");
			break;
		} else {
			CMR_LOGI("message.msg_type 0x%x, sub-type 0x%x",
				message.msg_type,
				message.sub_msg_type);

			switch (message.msg_type) {
			case CMR_EVT_SW_MON_INIT:
				LOGI("switch monitor thread msg INITED!");
				obj->setCameraState(SPRD_IDLE, STATE_SET_PARAMS);
				sem_post(&obj->mSwitchMonitorSyncSem);
				break;

			case CMR_EVT_SW_MON_SET_PARA:
				LOGD("switch monitor thread msg SET_PARA!");
				obj->setCameraState(SPRD_SET_PARAMS_IN_PROGRESS, STATE_SET_PARAMS);
				obj->setParametersInternal(obj->mSetParameters);
				obj->setCameraState(SPRD_IDLE, STATE_SET_PARAMS);
				break;

			case CMR_EVT_SW_MON_EXIT:
				LOGI("switch monitor thread msg EXIT!\n");
				exit_flag = 1;
				sem_post(&obj->mSwitchMonitorSyncSem);
				CMR_PRINT_TIME;
				break;

			default:
				LOGE("Unsupported switch monitorMSG");
				break;
			}

			if (1 == message.alloc_flag) {
				if (message.data) {
					free(message.data);
					message.data = 0;
				}
			}
		}
		if (exit_flag) {
			CMR_LOGI("switch monitor thread exit ");
			break;
		}
	}
	return NULL;
}

SprdCameraHardware::MemPool::MemPool(int buffer_size, int num_buffers,
		int frame_size,
		int frame_offset,
		const char *name) :
	mBufferSize(buffer_size),
	mNumBuffers(num_buffers),
	mFrameSize(frame_size),
	mFrameOffset(frame_offset),
	mBuffers(NULL), mName(name)
{
	/*empty*/
}

void SprdCameraHardware::MemPool::completeInitialization()
{
	// If we do not know how big the frame will be, we wait to allocate
	// the buffers describing the individual frames until we do know their
	// size.

	if (mFrameSize > 0) {
		mBuffers = new sp<MemoryBase>[mNumBuffers];
		for (int i = 0; i < mNumBuffers; i++) {
			mBuffers[i] = new
			MemoryBase(mHeap,
			i * mBufferSize + mFrameOffset,
			mFrameSize);
		}
	}
}

SprdCameraHardware::AshmemPool::AshmemPool(int buffer_size, int num_buffers,
		int frame_size,
		int frame_offset,
		const char *name) :
	SprdCameraHardware::MemPool(buffer_size,
		num_buffers,
		frame_size,
		frame_offset,
		name)
{
	LOGI("constructing MemPool %s backed by ashmem: "
		"%d frames @ %d bytes, offset %d, "
		"buffer size %d",
		mName,
		num_buffers, frame_size, frame_offset, buffer_size);

	int page_mask = getpagesize() - 1;
	int ashmem_size = buffer_size * num_buffers;
	ashmem_size += page_mask;
	ashmem_size &= ~page_mask;

	mHeap = new MemoryHeapBase(ashmem_size);

	completeInitialization();
}

SprdCameraHardware::MemPool::~MemPool()
{
	LOGI("destroying MemPool %s", mName);
	if (mFrameSize > 0)
		delete [] mBuffers;
	mHeap.clear();
	LOGI("destroying MemPool %s completed", mName);
}

status_t SprdCameraHardware::MemPool::dump(int fd, const Vector<String16> &args) const
{
	const size_t SIZE = 256;
	char buffer[SIZE];
	String8 result;

	snprintf(buffer, 255, "SprdCameraHardware::AshmemPool::dump\n");
	result.append(buffer);
	if (mName) {
		snprintf(buffer, 255, "mem pool name (%s)\n", mName);
		result.append(buffer);
	}

	if (mHeap != 0) {
		snprintf(buffer, 255, "heap base(%p), size(%d), flags(%d), device(%s)\n",
			mHeap->getBase(), mHeap->getSize(),
			mHeap->getFlags(), mHeap->getDevice());
		result.append(buffer);
	}

	snprintf(buffer, 255, "buffer size (%d), number of buffers (%d),"
		" frame size(%d), and frame offset(%d)\n",
		mBufferSize, mNumBuffers, mFrameSize, mFrameOffset);
	result.append(buffer);
	write(fd, result.string(), result.size());

	return NO_ERROR;
}

int SprdCameraHardware::flush_buffer(camera_flush_mem_type_e  type, int index, void *v_addr, void *p_addr, int size)
{
	int ret = 0;
	sprd_camera_memory_t *pmem = NULL;
	MemoryHeapIon *pHeapIon = NULL;

	switch(type)
	{
	case CAMERA_FLUSH_RAW_HEAP:
		pmem = mRawHeap;
		break;

	case CAMERA_FLUSH_RAW_HEAP_ALL:
		pmem = mRawHeap;
		v_addr = (void*)pmem->data;
		p_addr = (void*)pmem->phys_addr;
		size = (int)pmem->phys_size;
		break;

	case CAMERA_FLUSH_PREVIEW_HEAP:
		if ( (PREVIEW_BUFFER_USAGE_DCAM == mPreviewBufferUsage)
			&& index < (int)mPreviewHeapNum && mPreviewHeapArray != NULL) {
			pmem = mPreviewHeapArray[index];
		}
		break;

	default:
		break;
	}

	if (pmem) {
		pHeapIon = pmem->ion_heap;
	}

	if (pHeapIon) {
		LOGV("flush_buffer index=%d,vaddr=0x%x, paddr=0x%x,size=0x%x", index, (uint32_t)v_addr, (uint32_t)p_addr,size);
		ret = pHeapIon->flush_ion_buffer(v_addr, p_addr, size);
		if (ret) {
			LOGW("flush_buffer abnormal ret=%d", ret);
			LOGW("flush_buffer index=%d,vaddr=0x%x, paddr=0x%x", index, (uint32_t)v_addr, (uint32_t)p_addr);
		}
	}

	return ret;
}

static camera_device_t *g_cam_device;

/** Close this device */
static int HAL_camera_device_close(struct hw_device_t* device)
{
	LOGI("%s", __func__);

	if (device) {
		camera_device_t *cam_device = (camera_device_t *)device;
		delete static_cast<SprdCameraHardware *>(cam_device->priv);
		free(cam_device);
		g_cam_device = 0;
	}

#ifdef CONFIG_CAMERA_ISP
	stopispserver();
	ispvideo_RegCameraFunc(1, NULL);
	ispvideo_RegCameraFunc(2, NULL);
	ispvideo_RegCameraFunc(3, NULL);
	ispvideo_RegCameraFunc(4, NULL);
#endif

	return 0;
}

static inline SprdCameraHardware *obj(struct camera_device *dev)
{
	return reinterpret_cast<SprdCameraHardware *>(dev->priv);
}

/*Set the preview_stream_ops to which preview frames are sent */
static int HAL_camera_device_set_preview_window(struct camera_device *dev,
		struct preview_stream_ops *buf)
{
	LOGV("%s", __func__);
	obj(dev)->waitSetParamsOK();
	return obj(dev)->setPreviewWindow(buf);
}

/*Set the notification and data callbacks */
static void HAL_camera_device_set_callbacks(struct camera_device *dev,
		camera_notify_callback notify_cb,
		camera_data_callback data_cb,
		camera_data_timestamp_callback data_cb_timestamp,
		camera_request_memory get_memory,
		void* user)
{
	LOGV("%s", __func__);
	obj(dev)->setCallbacks(notify_cb, data_cb, data_cb_timestamp,
			get_memory,
			user);
}

/*
The following three functions all take a msg_type, which is a bitmask of
the messages defined in include/ui/Camera.h
*/

/*Enable a message, or set of messages.*/
static void HAL_camera_device_enable_msg_type(struct camera_device *dev, int32_t msg_type)
{
	LOGV("%s", __func__);
	obj(dev)->enableMsgType(msg_type);
}

/**
 * Disable a message, or a set of messages.
 *
 * Once received a call to disableMsgType(CAMERA_MSG_VIDEO_FRAME), camera
 * HAL should not rely on its client to call releaseRecordingFrame() to
 * release video recording frames sent out by the cameral HAL before and
 * after the disableMsgType(CAMERA_MSG_VIDEO_FRAME) call. Camera HAL
 * clients must not modify/access any video recording frame after calling
 * disableMsgType(CAMERA_MSG_VIDEO_FRAME).
 */
static void HAL_camera_device_disable_msg_type(struct camera_device *dev, int32_t msg_type)
{
	LOGV("%s", __func__);
	obj(dev)->disableMsgType(msg_type);
}

/**
 * Query whether a message, or a set of messages, is enabled.  Note that
 * this is operates as an AND, if any of the messages queried are off, this
 * will return false.
 */
static int HAL_camera_device_msg_type_enabled(struct camera_device *dev, int32_t msg_type)
{
	LOGV("%s", __func__);
	return obj(dev)->msgTypeEnabled(msg_type);
}

/*Start preview mode.*/
static int HAL_camera_device_start_preview(struct camera_device *dev)
{
	LOGV("%s", __func__);
	return obj(dev)->startPreview();
}

/*Stop a previously started preview.*/
static void HAL_camera_device_stop_preview(struct camera_device *dev)
{
	LOGV("%s", __func__);
	obj(dev)->stopPreview();
}

/*Returns true if preview is enabled.*/
static int HAL_camera_device_preview_enabled(struct camera_device *dev)
{
	LOGV("%s", __func__);
	return obj(dev)->previewEnabled();
}

/**
 * Request the camera HAL to store meta data or real YUV data in the video
 * buffers sent out via CAMERA_MSG_VIDEO_FRAME for a recording session. If
 * it is not called, the default camera HAL behavior is to store real YUV
 * data in the video buffers.
 *
 * This method should be called before startRecording() in order to be
 * effective.
 *
 * If meta data is stored in the video buffers, it is up to the receiver of
 * the video buffers to interpret the contents and to find the actual frame
 * data with the help of the meta data in the buffer. How this is done is
 * outside of the scope of this method.
 *
 * Some camera HALs may not support storing meta data in the video buffers,
 * but all camera HALs should support storing real YUV data in the video
 * buffers. If the camera HAL does not support storing the meta data in the
 * video buffers when it is requested to do do, INVALID_OPERATION must be
 * returned. It is very useful for the camera HAL to pass meta data rather
 * than the actual frame data directly to the video encoder, since the
 * amount of the uncompressed frame data can be very large if video size is
 * large.
 *
 * @param enable if true to instruct the camera HAL to store
 *      meta data in the video buffers; false to instruct
 *      the camera HAL to store real YUV data in the video
 *      buffers.
 *
 * @return OK on success.
 */
static int HAL_camera_device_store_meta_data_in_buffers(struct camera_device *dev, int enable)
{
	LOGV("%s", __func__);
	return obj(dev)->storeMetaDataInBuffers(enable);
}

/**
 * Start record mode. When a record image is available, a
 * CAMERA_MSG_VIDEO_FRAME message is sent with the corresponding
 * frame. Every record frame must be released by a camera HAL client via
 * releaseRecordingFrame() before the client calls
 * disableMsgType(CAMERA_MSG_VIDEO_FRAME). After the client calls
 * disableMsgType(CAMERA_MSG_VIDEO_FRAME), it is the camera HAL's
 * responsibility to manage the life-cycle of the video recording frames,
 * and the client must not modify/access any video recording frames.
 */
static int HAL_camera_device_start_recording(struct camera_device *dev)
{
	LOGV("%s", __func__);
	return obj(dev)->startRecording();
}

/*Stop a previously started recording.*/
static void HAL_camera_device_stop_recording(struct camera_device *dev)
{
	LOGV("%s", __func__);
	obj(dev)->stopRecording();
}

/*Returns true if recording is enabled.*/
static int HAL_camera_device_recording_enabled(struct camera_device *dev)
{
	LOGV("%s", __func__);
	return obj(dev)->recordingEnabled();
}

/**
 * Release a record frame previously returned by CAMERA_MSG_VIDEO_FRAME.
 *
 * It is camera HAL client's responsibility to release video recording
 * frames sent out by the camera HAL before the camera HAL receives a call
 * to disableMsgType(CAMERA_MSG_VIDEO_FRAME). After it receives the call to
 * disableMsgType(CAMERA_MSG_VIDEO_FRAME), it is the camera HAL's
 * responsibility to manage the life-cycle of the video recording frames.
 */
static void HAL_camera_device_release_recording_frame(struct camera_device *dev,
		const void *opaque)
{
	LOGV("%s", __func__);
	obj(dev)->releaseRecordingFrame(opaque);
}

/**
 * Start auto focus, the notification callback routine is called with
 * CAMERA_MSG_FOCUS once when focusing is complete. autoFocus() will be
 * called again if another auto focus is needed.
 */
static int HAL_camera_device_auto_focus(struct camera_device *dev)
{
	LOGV("%s", __func__);
	return obj(dev)->autoFocus();
}

/**
 * Cancels auto-focus function. If the auto-focus is still in progress,
 * this function will cancel it. Whether the auto-focus is in progress or
 * not, this function will return the focus position to the default.  If
 * the camera does not support auto-focus, this is a no-op.
 */
static int HAL_camera_device_cancel_auto_focus(struct camera_device *dev)
{
	LOGV("%s", __func__);
	return obj(dev)->cancelAutoFocus();
}

/**
 * Take a picture.
 */
static int HAL_camera_device_take_picture(struct camera_device *dev)
{
	LOGV("%s", __func__);
	return obj(dev)->takePicture();
}

/**
 * Cancel a picture that was started with takePicture. Calling this method
 * when no picture is being taken is a no-op.
 */
static int HAL_camera_device_cancel_picture(struct camera_device *dev)
{
	LOGV("%s", __func__);
	return obj(dev)->cancelPicture();
}

/**
 * Set the camera parameters. This returns BAD_VALUE if any parameter is
 * invalid or not supported.
 */
static int HAL_camera_device_set_parameters(struct camera_device *dev,
		const char *parms)
{
	LOGV("%s", __func__);
	String8 str(parms);
	SprdCameraParameters p(str);
	return obj(dev)->setParameters(p);
}

/*Return the camera parameters.*/
char *HAL_camera_device_get_parameters(struct camera_device *dev)
{
	LOGV("%s", __func__);
	String8 str;
	SprdCameraParameters parms = obj(dev)->getParameters();
	str = parms.flatten();
	return strdup(str.string());
}

void HAL_camera_device_put_parameters(struct camera_device *dev, char *parms)
{
	LOGV("%s", __func__);
	free(parms);
}

/*Send command to camera driver.*/
static int HAL_camera_device_send_command(struct camera_device *dev,
		int32_t cmd, int32_t arg1, int32_t arg2)
{
	LOGV("%s", __func__);
	return obj(dev)->sendCommand(cmd, arg1, arg2);
}

/**
 * Release the hardware resources owned by this object.  Note that this is
 * *not* done in the destructor.
 */
static void HAL_camera_device_release(struct camera_device *dev)
{
	LOGV("%s", __func__);
	obj(dev)->release();
}

/*Dump state of the camera hardware*/
static int HAL_camera_device_dump(struct camera_device *dev, int fd)
{
	LOGV("%s", __func__);
	return obj(dev)->dump(fd);
}

static int HAL_getNumberOfCameras()
{
	return SprdCameraHardware::getNumberOfCameras();
}

static int HAL_getCameraInfo(int cameraId, struct camera_info *cameraInfo)
{
	return SprdCameraHardware::getCameraInfo(cameraId, cameraInfo);
}

#define SET_METHOD(m) m : HAL_camera_device_##m

static camera_device_ops_t camera_device_ops = {
	SET_METHOD(set_preview_window),
	SET_METHOD(set_callbacks),
	SET_METHOD(enable_msg_type),
	SET_METHOD(disable_msg_type),
	SET_METHOD(msg_type_enabled),
	SET_METHOD(start_preview),
	SET_METHOD(stop_preview),
	SET_METHOD(preview_enabled),
	SET_METHOD(store_meta_data_in_buffers),
	SET_METHOD(start_recording),
	SET_METHOD(stop_recording),
	SET_METHOD(recording_enabled),
	SET_METHOD(release_recording_frame),
	SET_METHOD(auto_focus),
	SET_METHOD(cancel_auto_focus),
	SET_METHOD(take_picture),
	SET_METHOD(cancel_picture),
	SET_METHOD(set_parameters),
	SET_METHOD(get_parameters),
	SET_METHOD(put_parameters),
	SET_METHOD(send_command),
	SET_METHOD(release),
	SET_METHOD(dump),
};

#undef SET_METHOD

static int HAL_IspVideoStartPreview(uint32_t param1, uint32_t param2)
{
	int rtn=0x00;
	SprdCameraHardware * fun_ptr = dynamic_cast<SprdCameraHardware *>((SprdCameraHardware *)g_cam_device->priv);
	if (NULL != fun_ptr) {
		rtn=fun_ptr->startPreview();
	}
	return rtn;
}

static int HAL_IspVideoStopPreview(uint32_t param1, uint32_t param2)
{
	int rtn=0x00;
	SprdCameraHardware * fun_ptr = dynamic_cast<SprdCameraHardware *>((SprdCameraHardware *)g_cam_device->priv);
	if (NULL != fun_ptr) {
		fun_ptr->stopPreview();
	}
	return rtn;
}

static int HAL_IspVideoSetParam(uint32_t width, uint32_t height)
{
	int rtn=0x00;
	SprdCameraHardware * fun_ptr = dynamic_cast<SprdCameraHardware *>((SprdCameraHardware *)g_cam_device->priv);
	if (NULL != fun_ptr) {
		LOGE("ISP_TOOL: HAL_IspVideoSetParam width:%d, height:%d", width, height);
		fun_ptr->setCaptureRawMode(1);
		rtn=fun_ptr->setTakePictureSize(width,height);
	}
	return rtn;
}

static int HAL_IspVideoTakePicture(uint32_t param1, uint32_t param2)
{
	int rtn=0x00;
	SprdCameraHardware * fun_ptr = dynamic_cast<SprdCameraHardware *>((SprdCameraHardware *)g_cam_device->priv);
	if (NULL != fun_ptr) {
		rtn=fun_ptr->takePicture();
	}
	return rtn;
}

static int HAL_camera_device_open(const struct hw_module_t* module,
		const char *id,
		struct hw_device_t** device)
{
	LOGI("%s", __func__);
	GET_START_TIME;

	if (getApctCamInitSupport())
		cam_init_begin_time = systemTime();

	int cameraId = atoi(id);
	if (cameraId < 0 || cameraId >= HAL_getNumberOfCameras()) {
		LOGE("Invalid camera ID %s", id);
		return -EINVAL;
	}

	if (g_cam_device) {
		if (obj(g_cam_device)->getCameraId() == cameraId) {
			LOGI("returning existing camera ID %s", id);
			goto done;
		} else {
			LOGE("Cannot open camera %d. camera %d is already running!",
				cameraId, obj(g_cam_device)->getCameraId());
			return -ENOSYS;
		}
	}

	g_cam_device = (camera_device_t *)malloc(sizeof(camera_device_t));
	if (!g_cam_device)
		return -ENOMEM;

	g_cam_device->common.tag = HARDWARE_DEVICE_TAG;
	g_cam_device->common.version = 1;
	g_cam_device->common.module = const_cast<hw_module_t *>(module);
	g_cam_device->common.close = HAL_camera_device_close;

	g_cam_device->ops = &camera_device_ops;

	LOGI("%s: open camera %s", __func__, id);

	g_cam_device->priv = new SprdCameraHardware(cameraId);

#ifdef CONFIG_CAMERA_ISP
	startispserver();
	ispvideo_RegCameraFunc(1, HAL_IspVideoStartPreview);
	ispvideo_RegCameraFunc(2, HAL_IspVideoStopPreview);
	ispvideo_RegCameraFunc(3, HAL_IspVideoTakePicture);
	ispvideo_RegCameraFunc(4, HAL_IspVideoSetParam);
#endif

done:
	*device = (hw_device_t *)g_cam_device;

	if (!(((SprdCameraHardware *)(g_cam_device->priv))->isCameraInit())) {
		LOGE("camera init failed!");
		((SprdCameraHardware *)(g_cam_device->priv))->release();
		return -EINVAL;
	}

	LOGI("%s: opened camera %s (%p)", __func__, id, *device);

	return 0;
}

static hw_module_methods_t camera_module_methods = {
	open : HAL_camera_device_open
};

extern "C" {
	struct camera_module HAL_MODULE_INFO_SYM = {
		common : {
			tag                : HARDWARE_MODULE_TAG,
			module_api_version : CAMERA_MODULE_API_VERSION_1_0,
			hal_api_version    : HARDWARE_HAL_API_VERSION,
			id                 : CAMERA_HARDWARE_MODULE_ID,
			name               : "SPRD Camera HAL",
			author             : "Spreadtrum Corporation",
			methods            : &camera_module_methods,
			dso                : NULL,
			reserved : { 0 }
		},
		get_number_of_cameras : HAL_getNumberOfCameras,
		get_camera_info       : HAL_getCameraInfo,
		set_callbacks         : NULL,
		get_vendor_tag_ops    : NULL,
		open_legacy           : NULL,
		set_torch_mode        : NULL,
		init                  : NULL,
		reserved              : { NULL, NULL, NULL, NULL, NULL }
	};
}

}
