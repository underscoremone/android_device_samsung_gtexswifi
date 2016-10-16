/*
* hardware/sprd/hsdroid/libcamera/sprdcamerahardwareinterface.cpp
 * Dcam HAL based on sc8800g2
 *
 * Copyright (C) 2011 Spreadtrum 
 * 
 * Author: Xiaozhe wang <xiaozhe.wang@spreadtrum.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define LOG_NDEBUG 0
#define LOG_TAG "SprdCameraHardware"
#include <utils/Log.h>
//#include <utils/threads.h>
//#include <binder/MemoryHeapPmem.h>
#include <utils/String16.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
//#include <SprdCameraIfc.h>

#include "SprdOEMCamera.h"
#include "SprdCameraHardwareConfig.h"
#include "SprdCameraHardwareStub.h"

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*x))
#define PRINT_TIME 0

extern "C" {

int32_t g_camera_id = -1;
uint32_t g_sprd_zoom_levle = 0;

static inline void print_time()
{
#if PRINT_TIME
    struct timeval time; 
    gettimeofday(&time, NULL);
    LOGV("time: %lld us.", time.tv_sec * 1000000LL + time.tv_usec);
#endif
}

typedef struct {
    int width;
    int height;
} preview_size_type;

// These sizes have to be a multiple of 16 in each dimension
static preview_size_type preview_sizes[] = {
//    { 640, 480 }, //wxz: ???
    { 480, 320 }, // HVGA
    { 432, 320 }, // 1.35-to-1, for photos. (Rounded up from 1.3333 to 1)
    { 352, 288 }, // CIF
    { 320, 240 }, // QVGA
    { 240, 160 }, // SQVGA
    { 176, 144 }, // QCIF
};
#define PREVIEW_SIZE_COUNT (sizeof(preview_sizes)/sizeof(preview_size_type))

// default preview size is QVGA
#define DEFAULT_PREVIEW_SETTING 0

#define LIKELY( exp )       (__builtin_expect( (exp) != 0, true  ))
#define UNLIKELY( exp )     (__builtin_expect( (exp) != 0, false ))
}

namespace android {

    static Mutex singleton_lock;
   // static Mutex rex_init_lock;
    //static Condition rex_init_wait;

    static uint8_t* malloc_preview(uint32_t, uint32_t *, uint32_t);
    static uint8_t* malloc_raw(uint32_t, uint32_t *, uint32_t);
    static int free_preview(uint32_t *, uint32_t , uint32_t);
    static int free_raw(uint32_t *, uint32_t , uint32_t);
    static int reassoc(qdsp_module_type module);
    //static void cb_rex_signal_ready(void);

    SprdCameraHardware::SprdCameraHardware()
        : mParameters(),
          mPreviewHeight(-1),
          mPreviewWidth(-1),
          mRawHeight(-1),
          mRawWidth(-1),
          mCameraState(QCS_INIT),
/*          mShutterCallback(0),
          mRawPictureCallback(0),
          mJpegPictureCallback(0),
          mPictureCallbackCookie(0),
          mAutoFocusCallback(0),
          mAutoFocusCallbackCookie(0),
          mPreviewCallback(0),
          mPreviewCallbackCookie(0),
          mRecordingCallback(0),
          mRecordingCallbackCookie(0),*/
          mPreviewFrameSize(0),
          mRawSize(0),
          mPreviewCount(0),
          mNotify_cb(0),
          mData_cb(0),
          mData_cb_timestamp(0),
          mUser(0),
          mMsgEnabled(0)
    {
    	//g_camera_id = -1;
        LOGV("constructor EX");
    }

    void SprdCameraHardware::initDefaultParameters()
    {
    	LOGV("initDefaultParameters E");
        CameraParameters p;

        preview_size_type* ps = &preview_sizes[DEFAULT_PREVIEW_SETTING];
        p.setPreviewSize(ps->width, ps->height);
        p.setPreviewFrameRate(15);
        p.setPreviewFormat("yuv420sp");
        //p.setPreviewFormat("rgb565");
        p.setPictureFormat("jpeg"); // we do not look at this currently //
        //p.setPictureSize(2048, 1536);
        //p.setPictureSize(640, 480);//wxz: ???
        p.set("jpeg-quality", "100"); // maximum quality

        // These values must be multiples of 16, so we can't do 427x320, which is the exact size on
        // screen we want to display at. 480x360 doesn't work either since it's a multiple of 8.
        p.set("jpeg-thumbnail-width", "480");
        p.set("jpeg-thumbnail-height", "320");
        //p.set("jpeg-thumbnail-width", "512");
        //p.set("jpeg-thumbnail-height", "384");
        p.set("jpeg-thumbnail-quality", "90");

	//wxz20110526: only support the auto focus.
	p.set("focus-mode", "auto");

        //p.set("nightshot-mode", "0"); // off
        //p.set("luma-adaptation", "0"); // FIXME: turning it on causes a crash
        //p.set("antibanding", "auto"); // flicker detection and elimination
        //p.set("whitebalance", "auto");
        //p.set("rotation", "0");	

#if 0
        p.set("gps-timestamp", "1199145600"); // Jan 1, 2008, 00:00:00
        p.set("gps-latitude", "37.736071"); // A little house in San Francisco
        p.set("gps-longitude", "-122.441983");
        p.set("gps-altitude", "21"); // meters
#endif

        // List supported picture size values
        //p.set("picture-size-values", "2048x1536,1600x1200,1024x768");
        //p.set("picture-size-values", "1600x1200,1280x960,640x480"); 
      	if(1 == g_camera_id) //for the front camera
	{	
		uint32_t count, i;
		count = ARRAY_SIZE(sprd_front_camera_hardware_config);
		LOGV("front camera parameters: set count : %d", count);
		for(i = 0; i < count; i++)
		{
			LOGV("front camera parameters: set %d: key: %s, value: %s.", i, sprd_front_camera_hardware_config[i].key, sprd_front_camera_hardware_config[i].value);
			p.set(sprd_front_camera_hardware_config[i].key, sprd_front_camera_hardware_config[i].value);
		}
	}
	else //for the back camera
	{	
		uint32_t count, i;
		count = ARRAY_SIZE(sprd_back_camera_hardware_config);
		LOGV("back camera parameters: set count : %d", count);
		for(i = 0; i < count; i++)
		{
			LOGV("back camera parameters: set %d: key: %s, value: %s.", i, sprd_back_camera_hardware_config[i].key, sprd_back_camera_hardware_config[i].value);
			p.set(sprd_back_camera_hardware_config[i].key, sprd_back_camera_hardware_config[i].value);
		}
	}
	

        // List supported antibanding values
        //p.set("antibanding-values",
         //     "off,50hz,60hz,auto");

        // List supported effects:
//        p.set("effect-values",
//              "off,mono,negative,solarize,sepia,posterize,whiteboard," "blackboard,aqua");

        // List supported exposure-offset:
        //p.set("exposure-offset-values",
         //     "0,1,2,3,4,5,6,7,8,9,10");

        // List of whitebalance values
        //p.set("whitebalance-values",
         //     "auto,incandescent,fluorescent,daylight,cloudy");

        // List of ISO values
        //p.set("iso-values", "auto,high");

        if (setParameters(p) != NO_ERROR) {
            LOGE("Failed to set default parameters?!");
        }
	LOGV("initDefaultParameters X.");
    }

#define ROUND_TO_PAGE(x)  (((x)+0xfff)&~0xfff)

    // Called with mStateLock held!
    void SprdCameraHardware::startCameraIfNecessary()
    {
        if (mCameraState == QCS_INIT) {          
           /* rex_init_lock.lock(); //wxz:???
            rex_start();
            LOGV("waiting for REX to initialize.");
            rex_init_wait.wait(rex_init_lock);
            LOGV("REX is ready.");
            rex_init_lock.unlock();*/
            LOGV("waiting for camera_init to initialize.");	    
	    if(CAMERA_SUCCESS != camera_init(g_camera_id)){
		mCameraState = QCS_ERROR;
                LOGE("CameraIfNecessary: fail to camera_init().");		
		return;
            }
            
            //LOGV("starting REX emulation");
            LOGV("waiting for camera_start.g_camera_id: %d.", g_camera_id);
            // NOTE: camera_start() takes (height, width), not (width, height).
            if(CAMERA_SUCCESS != camera_start(camera_cb, this, mPreviewHeight, mPreviewWidth)){
		mCameraState = QCS_ERROR;
                LOGE("CameraIfNecessary: fail to camera_start().");		
		return;
            }
	    LOGV("OK to camera_start.");
            while(mCameraState != QCS_IDLE &&
                  mCameraState != QCS_ERROR) {
                LOGV("init camera: waiting for QCS_IDLE");
                mStateWait.wait(mStateLock);
                LOGV("init camera: woke up");
            }
            LOGV("init camera: initializing parameters");
        }
        else LOGV("camera hardware has been started already");
    }

void SprdCameraHardware::enableMsgType(int32_t msgType)
{
    Mutex::Autolock lock(mLock);
    mMsgEnabled |= msgType;
}

void SprdCameraHardware::disableMsgType(int32_t msgType)
{
    Mutex::Autolock lock(mLock);
    mMsgEnabled &= ~msgType;
}

bool SprdCameraHardware::msgTypeEnabled(int32_t msgType)
{
    Mutex::Autolock lock(mLock);
    return (mMsgEnabled & msgType);
}

    status_t SprdCameraHardware::dump(int fd, const Vector<String16>& args) const
    {
        const size_t SIZE = 256;
        char buffer[SIZE];
        String8 result;
        
        // Dump internal primitives.
        snprintf(buffer, 255, "SprdCameraHardware::dump: state (%d)\n", mCameraState);
        result.append(buffer);
        snprintf(buffer, 255, "preview width(%d) x height (%d)\n", mPreviewWidth, mPreviewHeight);
        result.append(buffer);
        snprintf(buffer, 255, "raw width(%d) x height (%d)\n", mRawWidth, mRawHeight);
        result.append(buffer);
        snprintf(buffer, 255, "preview frame size(%d), raw size (%d), jpeg size (%d) and jpeg max size (%d)\n", mPreviewFrameSize, mRawSize, mJpegSize, mJpegMaxSize);
        result.append(buffer);
        write(fd, result.string(), result.size());
        
        // Dump internal objects.
        if (mPreviewHeap != 0) {
            mPreviewHeap->dump(fd, args);
        }
        if (mRawHeap != 0) {
            mRawHeap->dump(fd, args);
        }
        if (mJpegHeap != 0) {
            mJpegHeap->dump(fd, args);
        }
        mParameters.dump(fd, args);
        return NO_ERROR;
    }
    
    bool SprdCameraHardware::initPreview()
    {
    	uint32_t page_size, buffer_size;
    	
//      clear_module_pmem(QDSP_MODULE_VFETASK);
        startCameraIfNecessary();

        // Tell libqcamera what the preview and raw dimensions are.  We
        // call this method even if the preview dimensions have not changed,
        // because the picture ones may have.
        //
        // NOTE: if this errors out, mCameraState != QCS_IDLE, which will be
        //       checked by the caller of this method.

        setCameraDimensions();

        LOGV("initPreview: preview size=%dx%d", mPreviewWidth, mPreviewHeight);

	mPreviewFrameSize = mPreviewWidth * mPreviewHeight * 3 / 2; // reality
        //mPreviewFrameSize = mPreviewWidth * mPreviewHeight * 2; //wxz: for path1 YUV422
        //mPreviewFrameSize = camera_get_frame_size(mPreviewWidth, mPreviewHeight, V4L2_PIX_FMT_RGB565X);
        //buffer_size = mPreviewFrameSize;
	//page_size = getpagesize();   
	//buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1); 
	//buffer_size = camera_get_size_align_page(mPreviewFrameSize);
	buffer_size = camera_get_size_align_page(mPreviewWidth * mPreviewHeight * 2);
	/*mPreviewHeap =
            //new PreviewPmemPool(kRawFrameHeaderSize +
            //                    mPreviewWidth * mPreviewHeight * 2, // worst
            new PreviewPmemPool(buffer_size,
                                kPreviewBufferCount,
                                mPreviewFrameSize,
                                0,
                                "preview");
        
        if (!mPreviewHeap->initialized()) {
	    LOGE("Fail to PreviewPmemPool.");
            mPreviewHeap = NULL;
            return false;
        }*/
#if 1
{
	uint32_t num = 100;
	uint32_t i;
	for(i = 0; i <= num; i++)
	{
		mPreviewHeap = new PreviewPmemPool(buffer_size,
                                kPreviewBufferCount,
                                mPreviewFrameSize,
                                0,
                                "preview");
                if (!mPreviewHeap->initialized()) {
			LOGE("Fail to PreviewPmemPool.time: %d.", i + 1);
            		mPreviewHeap = NULL;
			if(i == num)
	            		return false;
			//usleep(1000000);
			usleep(200000);
        	}
		else
			break;
	}
}
#endif

//      camera_af_init();

        return true;
    }

    void SprdCameraHardware::deinitPreview()
    {
        mPreviewHeap = NULL;
    }

    // Called with mStateLock held!
    bool SprdCameraHardware::initRaw(bool initJpegHeap)
    {
    	uint32_t page_size, buffer_size;
		
        LOGV("initRaw E");
        startCameraIfNecessary();

        // Tell libqcamera what the preview and raw dimensions are.  We
        // call this method even if the preview dimensions have not changed,
        // because the picture ones may have.
        //
        // NOTE: if this errors out, mCameraState != QCS_IDLE, which will be
        //       checked by the caller of this method.

        setCameraDimensions();

        LOGV("initRaw: picture size=%dx%d",
             mRawWidth, mRawHeight);

        // Note that we enforce yuv420 in setParameters().

        mRawSize =  mRawWidth * mRawHeight * 3 / 2; // reality 
        //mRawSize =  mRawWidth * mRawHeight * 2; // for YUYV 

        mJpegMaxSize = mRawWidth * mRawHeight * 2;

        LOGD("initRaw: clearing old mJpegHeap.");
        mJpegHeap = NULL;

        LOGV("initRaw: initializing mRawHeap.");
         if(mRawWidth == 2048)
         {
        		buffer_size = kRawFrameHeaderSize + mJpegMaxSize+1024*2048;	
        	}
         else
	{
		buffer_size = kRawFrameHeaderSize + mJpegMaxSize;
	}
	LOGV("INTERPOLATION:initRaw:mRawHeap size = %d .",buffer_size);	
	buffer_size = camera_get_size_align_page(buffer_size);	 
	LOGV("INTERPOLATION:initRaw:mRawHeap align size = %d .",buffer_size);
        /*mRawHeap =
            //new RawPmemPool("/dev/pmem_camera",
            new RawPmemPool("/dev/pmem_adsp",
                            //kRawFrameHeaderSize + mJpegMaxSize, // worst 
                            buffer_size,
                            kRawBufferCount,
                            mRawSize,
                            kRawFrameHeaderSize,
                            "snapshot camera");

        if (!mRawHeap->initialized()) {
            LOGE("initRaw X failed: error initializing mRawHeap");
            mRawHeap = NULL;
            return false;
        }*/
#if 1
{
	uint32_t num = 100;
	uint32_t i;
	for(i = 0; i <= num; i++)
	{
		mRawHeap = new RawPmemPool("/dev/pmem_adsp",
                            //kRawFrameHeaderSize + mJpegMaxSize, // worst 
                            buffer_size,
                            kRawBufferCount,
                            mRawSize,
                            kRawFrameHeaderSize,
                            "snapshot camera");
                if (!mRawHeap->initialized()) {
			LOGE("Fail to RawPmemPool.time: %d.", i + 1);
            		mRawHeap = NULL;
			if(i == num)
	            		return false;
			usleep(200000);
        	}
		else
			break;
	}
}
#endif		

        if (initJpegHeap) {
            LOGV("initRaw: initializing mJpegHeap.");
		buffer_size = mJpegMaxSize;
		//page_size = getpagesize();   
		//buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1); 
		buffer_size = camera_get_size_align_page(buffer_size);
            mJpegHeap =
                //new AshmemPool(mJpegMaxSize,
                new AshmemPool(buffer_size,
                               kJpegBufferCount,
                               0, // we do not know how big the picture wil be
                               0,
                               "jpeg");
            if (!mJpegHeap->initialized()) {
                LOGE("initRaw X failed: error initializing mJpegHeap.");
                mJpegHeap = NULL;
                mRawHeap = NULL;
                return false;
            }
	    LOGV("initJpeg success");
        }

        LOGV("initRaw X success");
        return true;
    }

    void SprdCameraHardware::release()
    {
        LOGV("release E");

        Mutex::Autolock l(&mLock);

        // Either preview was ongoing, or we are in the middle or taking a
        // picture.  It's the caller's responsibility to make sure the camera
        // is in the idle or init state before destroying this object.

        if (mCameraState != QCS_IDLE && mCameraState != QCS_INIT) {
            LOGE("Serious error: the camera state is %s, "
                 "not QCS_IDLE or QCS_INIT!",
                 getCameraStateStr(mCameraState));
        }
        
        mStateLock.lock();
        if (mCameraState != QCS_INIT) {
            // When libqcamera detects an error, it calls camera_cb from the
            // call to camera_stop, which would cause a deadlock if we
            // held the mStateLock.  For this reason, we have an intermediate
            // state QCS_INTERNAL_STOPPING, which we use to check to see if the
            // camera_cb was called inline.
            mCameraState = QCS_INTERNAL_STOPPING;
            mStateLock.unlock();

            LOGV("stopping camera.");
            if(CAMERA_SUCCESS != camera_stop(stop_camera_cb, this)){
                       	mCameraState = QCS_ERROR;
                	mStateLock.unlock();
                	LOGE("release: fail to camera_stop().");		
			return;
	    }

            mStateLock.lock();
            if (mCameraState == QCS_INTERNAL_STOPPING) {
                while (mCameraState != QCS_INIT &&
                       mCameraState != QCS_ERROR) {
                    LOGV("stopping camera: waiting for QCS_INIT");
                    mStateWait.wait(mStateLock);
                }
            }

            //LOGV("Shutting REX down.");
            //rex_shutdown();
            //LOGV("REX has shut down.");
            if(QCS_ERROR != mCameraState)
		mCameraState = QCS_INIT;
        }
        mStateLock.unlock();

        LOGV("release X");
    }

    SprdCameraHardware::~SprdCameraHardware()
    {
        LOGV("~SprdCameraHardware E");
        Mutex::Autolock singletonLock(&singleton_lock);
        singleton.clear();	
        LOGV("~SprdCameraHardware X");
    }

    sp<IMemoryHeap> SprdCameraHardware::getPreviewHeap() const
    {
        LOGV("getPreviewHeap");
#if 1		
        return mPreviewHeap != NULL ? mPreviewHeap->mHeap : NULL;
#else
        return mPreviewHeapTmp != NULL ? mPreviewHeapTmp->mHeap : NULL;
#endif
    }

    sp<IMemoryHeap> SprdCameraHardware::getRawHeap() const
    {
        LOGV("getRawHeap");  
        return mRawHeap != NULL ? mRawHeap->mHeap : NULL;
    }

 	void SprdCameraHardware::setCallbacks(notify_callback notify_cb, data_callback data_cb, data_callback_timestamp data_cb_timestamp, void * user)
	{
		mNotify_cb = notify_cb;
		mData_cb = data_cb;
		mData_cb_timestamp = data_cb_timestamp;
		mUser = user;
	}

    status_t SprdCameraHardware::startPreviewInternal()
//        preview_callback pcb, void *puser,
//        recording_callback rcb, void *ruser)
    {
        LOGV("startPreview E");

        if (mCameraState == QCS_PREVIEW_IN_PROGRESS) {
            LOGE("startPreview is already in progress, doing nothing.");
            // We might want to change the callback functions while preview is
            // streaming, for example to enable or disable recording.
            //setCallbackFuns(pcb, puser, rcb, ruser);
            return NO_ERROR;
        }

        // We check for these two states explicitly because it is possible
        // for startPreview() to be called in response to a raw or JPEG
        // callback, but before we've updated the state from QCS_WAITING_RAW
        // or QCS_WAITING_JPEG to QCS_IDLE.  This is because in camera_cb(),
        // we update the state *after* we've made the callback.  See that
        // function for an explanation.

        if (mCameraState == QCS_WAITING_RAW ||
            mCameraState == QCS_WAITING_JPEG) {
            while (mCameraState != QCS_IDLE &&
                   mCameraState != QCS_ERROR) {
                LOGV("waiting for QCS_IDLE");
                mStateWait.wait(mStateLock);
            }
        }

        if (mCameraState != QCS_IDLE) {
            LOGE("startPreview X Camera state is %s, expecting QCS_IDLE!",
                getCameraStateStr(mCameraState));
            return INVALID_OPERATION;
        }

        if (!initPreview()) {
            LOGE("startPreview X initPreview failed.  Not starting preview.");
            return UNKNOWN_ERROR;
        }

       // setCallbackFuns(pcb, puser, rcb, ruser);

        // hack to prevent first preview frame from being black
        mPreviewCount = 0;

        mCameraState = QCS_INTERNAL_PREVIEW_REQUESTED;
        camera_ret_code_type qret =
            camera_start_preview(camera_cb, this);
	LOGV("camera_start_preview X");
        if (qret == CAMERA_SUCCESS) {
            while(mCameraState != QCS_PREVIEW_IN_PROGRESS &&
                  mCameraState != QCS_ERROR) {
                LOGV("waiting for QCS_PREVIEW_IN_PROGRESS");
                mStateWait.wait(mStateLock);
            }
        }
        else {
            LOGE("startPreview failed: sensor error.");
            mCameraState = QCS_ERROR;
        }

        LOGV("startPreview X");
        return mCameraState == QCS_PREVIEW_IN_PROGRESS ? NO_ERROR : UNKNOWN_ERROR;
    }

    void SprdCameraHardware::stopPreviewInternal()
    {
        LOGV("stopPreviewInternal E");

        if (mCameraState != QCS_PREVIEW_IN_PROGRESS) {
            LOGE("Preview not in progress!");
            return;
        }

        /*if (mAutoFocusCallback != NULL) {
            // WARNING: clear mAutoFocusCallback BEFORE you call
            // camera_stop_focus.  The CAMERA_EXIT_CB_ABORT is (erroneously)
            // delivered inline camera_stop_focus(), and we cannot acquire
            // mStateLock, because that would cause a deadlock.  In any case,
            // CAMERA_EXIT_CB_ABORT is delivered only when we call
            // camera_stop_focus.
            mAutoFocusCallback = NULL;
            camera_stop_focus();
        }*/
        //camera_stop_focus();

        //setCallbackFuns(NULL, NULL, NULL, NULL);
        //setCallbacks(NULL, NULL, NULL, NULL); //wxz: ????
        
        mCameraState = QCS_INTERNAL_PREVIEW_STOPPING;

        if(CAMERA_SUCCESS != camera_stop_preview()){
                mCameraState = QCS_ERROR;
        	mPreviewHeap = NULL; 
                LOGE("stopPreviewInternal: fail to camera_stop_preview().");		
		return;
	}
        while (mCameraState != QCS_IDLE &&
               mCameraState != QCS_ERROR)  {
            LOGV("waiting for QCS_IDLE");
            mStateWait.wait(mStateLock);
        }

        LOGV("stopPreviewInternal: Freeing preview heap.");		
        mPreviewHeap = NULL; 
        //mPreviewCallback = NULL;

        LOGV("stopPreviewInternal: X Preview has stopped.");
    }


    status_t SprdCameraHardware::startPreview()
    {
        Mutex::Autolock l(&mLock);
        Mutex::Autolock stateLock(&mStateLock);
        return startPreviewInternal();
    }

    void SprdCameraHardware::stopPreview() {
        LOGV("stopPreview: E");
        Mutex::Autolock l(&mLock);
  /*      if (!setCallbackFuns(NULL, NULL,
                          mRecordingCallback,
                          mRecordingCallbackCookie)) {
            Mutex::Autolock statelock(&mStateLock);
            stopPreviewInternal();
        }*/
       
         Mutex::Autolock statelock(&mStateLock);
         stopPreviewInternal();
       
        LOGV("stopPreview: X");
    }

    bool SprdCameraHardware::previewEnabled() {
        Mutex::Autolock l(&mLock);
        return mCameraState == QCS_PREVIEW_IN_PROGRESS;
    }

       status_t SprdCameraHardware::startRecording()
    {
        Mutex::Autolock l(&mLock);
        Mutex::Autolock stateLock(&mStateLock);
        return startPreviewInternal();
    }

    void SprdCameraHardware::stopRecording() {
        LOGV("stopRecording: E");
        Mutex::Autolock l(&mLock);
/*        if (!setCallbackFuns(mPreviewCallback,
                          mPreviewCallbackCookie,
                          NULL, NULL)) {
            Mutex::Autolock statelock(&mStateLock);
            stopPreviewInternal();
        }*/
        Mutex::Autolock statelock(&mStateLock);
        stopPreviewInternal();
        LOGV("stopRecording: X");
    }

    bool SprdCameraHardware::recordingEnabled() {
        Mutex::Autolock l(&mLock);
        Mutex::Autolock stateLock(&mStateLock);
        return mCameraState == QCS_PREVIEW_IN_PROGRESS &&
           	 (mMsgEnabled & CAMERA_MSG_VIDEO_FRAME);	
//            mRecordingCallback != NULL;
    }

    void SprdCameraHardware::releaseRecordingFrame(
        const sp<IMemory>& mem __attribute__((unused)))
   {
   	//LOGV("releaseRecordingFrame E. ");
        Mutex::Autolock l(&mLock);
	ssize_t offset;
	size_t size;
	uint32_t index;
	
	mem->getMemory(&offset, &size);
	index =offset / (size * 4 / 3);//wxz20110919: size is for YUV420, but the buffer is alloced by YUV422.
	//LOGV("releaseRecordingFrame: index: %d, offset: %x, size: %x.", index,offset,size);
		
        camera_release_frame(index);
   	//LOGV("releaseRecordingFrame X. ");		
    }


    status_t SprdCameraHardware::autoFocus()
    {
        LOGV("Starting auto focus.");
        Mutex::Autolock l(&mLock);
        Mutex::Autolock lock(&mStateLock);

        if (mCameraState != QCS_PREVIEW_IN_PROGRESS) {
            LOGE("Invalid camera state %s: expecting QCS_PREVIEW_IN_PROGRESS,"
                 " cannot start autofocus!",
                 getCameraStateStr(mCameraState));
            return INVALID_OPERATION;
        }
		
        camera_start_focus(CAMERA_AUTO_FOCUS, camera_cb, this);
        return NO_ERROR;
    }
uint32_t get_physical_address(sp<MemoryHeapBase> mHeap)
{
	struct pmem_region region;
	int fd_pmem = 0;
	
	fd_pmem = mHeap->getHeapID();	
	::ioctl(fd_pmem,PMEM_GET_PHYS,&region);
	return region.offset;
}
bool  SprdCameraHardware::allocZoomBufferForCap(void)
{
	uint32_t num = 100;
	uint32_t i;	
	uint32_t buffer_size, phy_addr;
	uint32_t zoom_level = g_sprd_zoom_levle;
	LOGV("INTERPOLATION:allocZoomBufferForCap:zoom_level = %d.",zoom_level);
	if(0 == zoom_level)
		return false;
	else
		zoom_level++;
	
         buffer_size = (mRawWidth / zoom_level) * (mRawHeight / zoom_level) * 2;		
    
	buffer_size = camera_get_size_align_page(buffer_size);
	for(i = 0; i <= num; i++)
	{
		mJpegencZoomHeap = new RawPmemPool("/dev/pmem_adsp",
                            buffer_size,
                            1,
                            buffer_size,
                            0,
                            "capture zoom");
                if (!mJpegencZoomHeap->initialized()) {
			LOGE("Fail to mJpegencZoomHeap.time: %d.", i + 1);
            		mJpegencZoomHeap = NULL;
			if(i == num)
	            		return false;
			usleep(200000);
        	}
		else
			break;
	}

	phy_addr = get_physical_address(mJpegencZoomHeap->mHeap);
	camera_alloc_zoom_buffer(phy_addr, (uint8_t *)mJpegencZoomHeap->mHeap->base(), buffer_size*zoom_level*zoom_level);
	//camera_alloc_zoom_buffer(phy_addr, buffer_size);

	return true;
}

bool  SprdCameraHardware::allocSwapBufferForCap(void)
{
	uint32_t num = 100;
	uint32_t i;	
	uint32_t buffer_size, phy_addr;
	
       	 buffer_size = 1024 * 1024;		
	LOGV("INTERPOLATION:alloc swap buffer size =%d .",buffer_size);	
	buffer_size = camera_get_size_align_page(buffer_size);
	LOGV("INTERPOLATION:alloc swap buffer align size =%d .",buffer_size);	
	for(i = 0; i <= num; i++)
	{
		mJpegencSwapHeap = new RawPmemPool("/dev/pmem_adsp",
                            buffer_size,
                            1,
                            buffer_size,
                            0,
                            "capture scale");
                if (!mJpegencSwapHeap->initialized()) {
			LOGE("Fail to mJpegencSwapHeap.time: %d.", i + 1);
            		mJpegencSwapHeap = NULL;
			if(i == num)
	            		return false;
			usleep(200000);
        	}
		else
			break;
	}

	phy_addr = get_physical_address(mJpegencSwapHeap->mHeap);
	camera_alloc_swap_buffer(phy_addr);

	return true;
}
    status_t SprdCameraHardware::takePicture()
    {
        /*LOGV("takePicture: E raw_cb = %p, jpeg_cb = %p",
             raw_cb, jpeg_cb);*/
        print_time();

        Mutex::Autolock l(&mLock);
        Mutex::Autolock stateLock(&mStateLock);

        Sprd_camera_state last_state = mCameraState;
        if (mCameraState == QCS_PREVIEW_IN_PROGRESS) {
	    LOGV("call stopPreviewInternal in takePicture().");
            stopPreviewInternal();
        }
	LOGV("ok to stopPreviewInternal in takePicture.");

        // We check for these two states explicitly because it is possible
        // for takePicture() to be called in response to a raw or JPEG
        // callback, but before we've updated the state from QCS_WAITING_RAW
        // or QCS_WAITING_JPEG to QCS_IDLE.  This is because in camera_cb(),
        // we update the state *after* we've made the callback.  See that
        // function for an explanation why.

        if (mCameraState == QCS_WAITING_RAW ||
            mCameraState == QCS_WAITING_JPEG) {
            while (mCameraState != QCS_IDLE &&
                   mCameraState != QCS_ERROR) {
                LOGV("waiting for QCS_IDLE");
                mStateWait.wait(mStateLock);
            }
        }

        if (mCameraState != QCS_IDLE) {
            LOGE("takePicture: %sunexpected state %d, expecting QCS_IDLE",
                 (last_state == QCS_PREVIEW_IN_PROGRESS ?
                  "(stop preview) " : ""),
                 mCameraState);
            // If we had to stop preview in order to take a picture, and
            // we failed to transition to a QCS_IDLE state, that's because
            // of an internal error.
            return last_state == QCS_PREVIEW_IN_PROGRESS ?
                UNKNOWN_ERROR :
                INVALID_OPERATION;
        }
	LOGV("start to initRaw in takePicture.");
        //if (!initRaw(jpeg_cb != NULL)) {
        if (!initRaw(mData_cb != NULL)) { //wxz:???  when to init jpeg?
            LOGE("initRaw failed.  Not taking picture.");
            return UNKNOWN_ERROR;
        }

        if (mCameraState != QCS_IDLE) {
            LOGE("takePicture: (init raw) "
                 "unexpected state %d, expecting QCS_IDLE",
                mCameraState);
            // If we had to stop preview in order to take a picture, and
            // we failed to transition to a QCS_IDLE state, that's because
            // of an internal error.
            return last_state == QCS_PREVIEW_IN_PROGRESS ?
                UNKNOWN_ERROR :
                INVALID_OPERATION;
        }

        /*{
            Mutex::Autolock cbLock(&mCallbackLock);
            mShutterCallback = shutter_cb;
            mRawPictureCallback = raw_cb;
            mJpegPictureCallback = jpeg_cb;
            mPictureCallbackCookie = user;
        }*/

        mCameraState = QCS_INTERNAL_RAW_REQUESTED;

        if( (0 < g_sprd_zoom_levle)&&(mRawWidth!=2048))
	{
		allocZoomBufferForCap();
        	}
        		
	if((mRawWidth == 2048) || (0 < g_sprd_zoom_levle))
	{
		//allocZoomBufferForCap();
		allocSwapBufferForCap();
	}
	LOGV("INTERPOLATION::takePicture:mRawWidth=%d,g_sprd_zoom_levle=%d",mRawWidth,g_sprd_zoom_levle);
	
        //mParameters.set("rotation", 0); //wxz20110802: now cann't support the rotation in capture mode.
        if(CAMERA_SUCCESS != camera_take_picture(camera_cb, this)){//wxz20110901: check the return value.
                mCameraState = QCS_ERROR;
		LOGE("takePicture: fail to camera_take_picture.");
		return UNKNOWN_ERROR;
	}

        // It's possible for the YUV callback as well as the JPEG callbacks
        // to be invoked before we even make it here, so we check for all
        // possible result states from takePicture.

        while (mCameraState != QCS_WAITING_RAW &&
               mCameraState != QCS_WAITING_JPEG &&
               mCameraState != QCS_IDLE &&
               mCameraState != QCS_ERROR)  {
            LOGV("takePicture: waiting for QCS_WAITING_RAW or QCS_WAITING_JPEG");
            mStateWait.wait(mStateLock);
            LOGV("takePicture: woke up, state is %s",
                 getCameraStateStr(mCameraState));
        }

        LOGV("takePicture: X");
        print_time();
        return mCameraState != QCS_ERROR ?
            NO_ERROR : UNKNOWN_ERROR;
    }

	/*
    status_t SprdCameraHardware::cancelPicture(
        bool cancel_shutter, bool cancel_raw, bool cancel_jpeg)
    {
        LOGV("cancelPicture: E cancel_shutter = %d, cancel_raw = %d, cancel_jpeg = %d",
             cancel_shutter, cancel_raw, cancel_jpeg);
        Mutex::Autolock l(&mLock);
        Mutex::Autolock stateLock(&mStateLock);

        switch (mCameraState) {
        case QCS_INTERNAL_RAW_REQUESTED:
        case QCS_WAITING_RAW:
        case QCS_WAITING_JPEG:
            LOGD("camera state is %s, stopping picture.",
                 getCameraStateStr(mCameraState));

            {
                Mutex::Autolock cbLock(&mCallbackLock);
                if (cancel_shutter) mShutterCallback = NULL;
                if (cancel_raw) mRawPictureCallback = NULL;
                if (cancel_jpeg) mJpegPictureCallback = NULL;
            }

            while (mCameraState != QCS_IDLE &&
                   mCameraState != QCS_ERROR)  {
                LOGV("cancelPicture: waiting for QCS_IDLE");
                mStateWait.wait(mStateLock);
            }
            break;
        default:
            LOGV("not taking a picture (state %s)",
                 getCameraStateStr(mCameraState));
        }

        LOGV("cancelPicture: X");
        return NO_ERROR;
    }
*/
    status_t SprdCameraHardware::cancelPicture()
    {       
        Mutex::Autolock l(&mLock);
        Mutex::Autolock stateLock(&mStateLock);

        switch (mCameraState) {
        case QCS_INTERNAL_RAW_REQUESTED:
        case QCS_WAITING_RAW:
        case QCS_WAITING_JPEG:
            LOGD("camera state is %s, stopping picture.",
                 getCameraStateStr(mCameraState));   

            while (mCameraState != QCS_IDLE &&
                   mCameraState != QCS_ERROR)  {
                LOGV("cancelPicture: waiting for QCS_IDLE");
                mStateWait.wait(mStateLock);
            }
            break;
        default:
            LOGV("not taking a picture (state %s)",
                 getCameraStateStr(mCameraState));
        }

        LOGV("cancelPicture: X");
        return NO_ERROR;
    }

    status_t SprdCameraHardware::setParameters(
        const CameraParameters& params)
    {
        LOGV("setParameters: E params = %p", &params);

        Mutex::Autolock l(&mLock);
        Mutex::Autolock lock(&mStateLock);

        // FIXME: verify params
        // yuv422sp is here only for legacy reason. Unfortunately, we release
        // the code with yuv422sp as the default and enforced setting. The
        // correct setting is yuv420sp.
       if ((strcmp(params.getPreviewFormat(), "yuv420sp") != 0) &&
                (strcmp(params.getPreviewFormat(), "yuv422sp") != 0)) {
            LOGE("Only yuv420sp preview is supported");
            return INVALID_OPERATION;
        }

        // FIXME: will this make a deep copy/do the right thing? String8 i
        // should handle it
        
        mParameters = params;
        
        // libqcamera only supports certain size/aspect ratios
        // find closest match that doesn't exceed app's request
        int width, height;
        params.getPreviewSize(&width, &height);
        LOGV("requested size %d x %d", width, height);
	//wxz20111008: If the preview size changes, we will stop preview.
	 if((width != mPreviewWidth) || (height != mPreviewHeight)){        	
		if (mCameraState == QCS_PREVIEW_IN_PROGRESS){
			//mPreviewWidth = width;
			//mPreviewHeight = height;		
			stopPreviewInternal();
		}
	 }
		//wxz20110726: delete them to support the any preview size.
        /*preview_size_type* ps = preview_sizes;
        size_t i;
        for (i = 0; i < PREVIEW_SIZE_COUNT; ++i, ++ps) {
            if (width >= ps->width && height >= ps->height) break;
        }
        // app requested smaller size than supported, use smallest size
        if (i == PREVIEW_SIZE_COUNT) ps--;
        LOGV("actual size %d x %d", ps->width, ps->height);
        mParameters.setPreviewSize(ps->width, ps->height);
		*/
        //mParameters.setPreviewSize(144, 176);
	/*int rotation = mParameters.getInt("rotation");
	if(90 == rotation){
		mPreviewWidth  = height;
		mPreviewHeight = width;
		LOGV("change the preview size : size %d x %d", mPreviewWidth, mPreviewHeight);
	}
	else{
		mPreviewWidth  = width;
		mPreviewHeight = height;
	}*/ 

        mParameters.getPreviewSize(&mPreviewWidth, &mPreviewHeight);
        mParameters.getPictureSize(&mRawWidth, &mRawHeight);
	LOGV("requested picture size %d x %d", mRawWidth, mRawHeight);
	LOGV("requested preview size %d x %d", mPreviewWidth, mPreviewHeight);
        mPreviewWidth = (mPreviewWidth + 1) & ~1;
        mPreviewHeight = (mPreviewHeight + 1) & ~1;
        mRawHeight = (mRawHeight + 1) & ~1;
        mRawWidth = (mRawWidth + 1) & ~1;

        LOGV("setParameters: 1 mCameraState=%d, zoom: %d, max: %d", mCameraState, atoi(mParameters.get("zoom")), CAMERA_ZOOM_MAX);
	//wxz: 20111009: error!!!!!!!
	//wxz20111008: check the zoom value.
	if(CAMERA_ZOOM_MAX <= atoi(mParameters.get("zoom")) ){
        	LOGV("setParameters: 3 mCameraState=%d", mCameraState);
		mParameters.set("zoom", mParameters.get("max-zoom"));
        	LOGV("setParameters: 4 mCameraState=%d", mCameraState);
		return BAD_VALUE;		
	}
        LOGV("setParameters: 2 mCameraState=%d", mCameraState);

        initCameraParameters();

        LOGV("setParameters: X mCameraState=%d", mCameraState);
        //return mCameraState == QCS_IDLE ?
         //   NO_ERROR : UNKNOWN_ERROR;
         return NO_ERROR; //wxz:???
    }

    CameraParameters SprdCameraHardware::getParameters() const
    {
        LOGV("getParameters: EX");
        return mParameters;
    }
	
static CameraInfo sCameraInfo[] = {
    {
        CAMERA_FACING_BACK,
        90,  /* orientation */
    },
    {
        CAMERA_FACING_FRONT,
        90,  /* orientation */
    }    
};	
extern "C" int HAL_getNumberOfCameras()
{
    return sizeof(sCameraInfo) / sizeof(sCameraInfo[0]);
}

extern "C" void HAL_getCameraInfo(int cameraId, struct CameraInfo* cameraInfo)
{
    memcpy(cameraInfo, &sCameraInfo[cameraId], sizeof(CameraInfo));
}
/*extern "C" int HAL_setCameraId(int cameraId)
{
	g_camera_id = cameraId;
	LOGV("HAL_setCameraId.g_camera_id: %d.", g_camera_id);
	return NO_ERROR;
}*/

    extern "C" sp<CameraHardwareInterface> HAL_openCameraHardware(int cameraId)	
    {
        LOGV("openCameraHardware: call createInstance. cameraId: %d.", cameraId);
	g_camera_id = cameraId;
	if(2 == g_camera_id){	
        	return SprdCameraHardwareStub::createInstance();        	
	}
	else{
        	return SprdCameraHardware::createInstance();
	}
    }

    wp<SprdCameraHardware> SprdCameraHardware::singleton;

    // If the hardware already exists, return a strong pointer to the current
    // object. If not, create a new hardware object, put it in the singleton,
    // and return it.
    sp<CameraHardwareInterface> SprdCameraHardware::createInstance()
    {
        //LOGV("createInstance: E");
        LOGV("createInstance: E");

        singleton_lock.lock();
        if (singleton != 0) {
            sp<CameraHardwareInterface> hardware = singleton.promote();
            if (hardware != 0) {
                LOGE("createInstance: X return existing hardware=%p",
                     &(*hardware));
                singleton_lock.unlock();
                return hardware;
            }
        }

        /*{
            struct stat st;
            int rc = stat("/dev/oncrpc", &st);           
            if (rc < 0) {
                LOGV("createInstance: X failed to create hardware: %s",
                     strerror(errno));
                singleton_lock.unlock();
                return NULL;
            }
        }*/
	
        SprdCameraHardware *cam = new SprdCameraHardware();       
        sp<SprdCameraHardware> hardware(cam);
        singleton = hardware;
        singleton_lock.unlock();

        // initDefaultParameters() will cause the camera_cb() to be called.
        // Since the latter tries to promote the singleton object to make sure
        // it still exists, we need to call this function after we have set the
        // singleton.
        cam->initDefaultParameters();
        //LOGV("createInstance: X created hardware=%p", &(*hardware));
        LOGV("createInstance: X created hardware=%p", &(*hardware));
        return hardware;
    }

    // For internal use only, hence the strong pointer to the derived type.
    sp<SprdCameraHardware> SprdCameraHardware::getInstance()
    {
        Mutex::Autolock singletonLock(&singleton_lock);
        sp<CameraHardwareInterface> hardware = singleton.promote();
        return (hardware != 0) ?
            sp<SprdCameraHardware>(static_cast<SprdCameraHardware*>
                                       (hardware.get())) :
            NULL;
    }

    void* SprdCameraHardware::get_preview_mem(uint32_t size,
                                                  uint32_t *phy_addr,
                                                  uint32_t index)
    {
        if (mPreviewHeap != NULL && mPreviewHeap->mHeap != NULL) {
            uint8_t *base = (uint8_t *)mPreviewHeap->mHeap->base();           
            if (base && size <= mPreviewHeap->mSize.len) {
                // For preview, associate the memory with the VFE task in the
                // DSP.  This way, when the DSP gets a command that has a
                // physical address, it knows which pmem region to patch it
                // against.
                //wxz: change the logical address to physical address
                uint32_t physical_addr = get_physical_address(mPreviewHeap->mHeap);
                uint32_t vaddr = (uint32_t)(base + size*index);
               *phy_addr = (uint32_t)(physical_addr + size*index);			

                LOGV("get_preview_mem: base %p MALLOC size %d index %d --> %p",
                     base, size, index, (void *)vaddr);			
                
                return (void *)vaddr;
            }
        }
        LOGV("get_preview_mem: X NULL");
        return NULL;
    }

    void* SprdCameraHardware::get_preview_mem_for_HW(uint32_t size, uint32_t *phy_addr, uint32_t index)
    {	
	uint32_t buffer_size = camera_get_size_align_page(mPreviewWidth * mPreviewHeight * 2);
	uint32_t num = 100;
	uint32_t i;
	for(i = 0; i <= num; i++)
	{
		mPreviewHWHeap = new PreviewPmemPool(buffer_size,
                                kPreviewBufferForHWCount,
                                buffer_size,
                                0,
                                "previewForHW");
                if (!mPreviewHWHeap->initialized()) {
			LOGE("Fail to mPreviewHWHeap.time: %d.", i + 1);
            		mPreviewHWHeap = NULL;
			if(i == num)
	            		return false;			
			usleep(200000);
        	}
		else
			break;
	}
	{
            uint8_t *base = (uint8_t *)mPreviewHWHeap->mHeap->base();           
            if (base && size <= mPreviewHWHeap->mSize.len) {               
                //wxz: change the logical address to physical address
                uint32_t physical_addr = get_physical_address(mPreviewHWHeap->mHeap);
                uint32_t vaddr = (uint32_t)(base + size*index);
               *phy_addr = (uint32_t)(physical_addr + size*index);			

                LOGV("get_preview_mem: base %p MALLOC size %d index %d --> %p",
                     base, size, index, (void *)vaddr);			
                
                return (void *)vaddr;
            }
        }
        LOGV("get_preview_mem: X NULL");
        return NULL;
    }


    void SprdCameraHardware::free_preview_mem(uint32_t *phy_addr,
                                                  uint32_t size,
                                                  uint32_t index)
    {
        LOGV("free_preview_mem: EX NOP");
        return;
    }
    void SprdCameraHardware::free_preview_mem_for_HW(void)
    {
	mPreviewHWHeap = NULL;
        LOGV("free_preview_mem_for_HW: EX NOP");
        return;
    }

    void* SprdCameraHardware::get_raw_mem(uint32_t size,
                                                   uint32_t *phy_addr,
                                                   uint32_t index)
    {
        if (mRawHeap != NULL && mRawHeap->mHeap != NULL) {
            uint8_t *base = (uint8_t *)mRawHeap->mHeap->base();  	    
            if (base && size <= mRawHeap->mSize.len) {
                // For raw snapshot, associate the memory with the VFE and LPM
                // tasks in the DSP.  This way, when the DSP gets a command
                // that has a physical address, it knows which pmem region to
                // patch it against.
                //wxz: change the logical address to physical address
                uint32_t physical_addr = get_physical_address(mRawHeap->mHeap);
                uint32_t vaddr = (uint32_t)(base + size*index);
               *phy_addr = (uint32_t)(physical_addr + size*index);			

                LOGV("get_raw_mem: base %p MALLOC size %d index %d --> %p",
                     base, size, index, (void *)vaddr);			
                
                return (void *)vaddr; 
            }
        }
        LOGV("get_raw_mem: X NULL");
        return NULL;
    }

    void SprdCameraHardware::free_raw_mem(uint32_t *phy_addr,
                                              uint32_t size,
                                              uint32_t index)
    {
        LOGV("free_raw_mem: EX NOP");
        return;
    }
    void* SprdCameraHardware::get_jpeg_encoder_mem_by_HW(uint32_t *phy_addr)
    {
	uint32_t num = 100;
	uint32_t i;
	int buffer_size = camera_get_size_align_page(JPEG_ENC_HW_PMEM);	
	
	for(i = 0; i <= num; i++)
	{
		mJpegencHWHeap = new RawPmemPool("/dev/pmem_adsp",
                            buffer_size,
                            JPEG_ENC_HW_BUF_NUM,
                            buffer_size,
                            0,
                            "jpeg encoder by hw");
                if (!mJpegencHWHeap->initialized()) {
			LOGE("Fail to mJpegencHWHeap.time: %d.", i + 1);
            		mJpegencHWHeap = NULL;
			if(i == num)
	            		return NULL;
			usleep(200000);
        	}
		else
			break;
	}
	
        if (mJpegencHWHeap != NULL && mJpegencHWHeap->mHeap != NULL) {
            	*phy_addr = (uint32_t)get_physical_address(mJpegencHWHeap->mHeap);
                uint32_t vaddr = (uint32_t)mJpegencHWHeap->mHeap->base(); 

                LOGV("get_jpeg_encoder_mem_by_HW: MALLOC size %d --> %p",
                      buffer_size,  (void *)vaddr);                
                return (void *)vaddr;           
        }

        LOGV("get_jpeg_encoder_mem_by_HW: X NULL");
        return NULL;
    }

    void SprdCameraHardware::free_jpeg_encoder_mem_by_HW(void)
    {
    	mJpegencHWHeap = NULL;		
        LOGV("free_raw_mem: EX NOP");	
		
        return;
    }

	
    void* SprdCameraHardware::get_temp_mem_by_HW(uint32_t size, uint32_t count, uint32_t *phy_addr)
    {
	uint32_t num = 100;
	uint32_t i;
	int buffer_size = camera_get_size_align_page(size);	
	LOGV("INTERPOLATION:temp mem size=%d,align size=%d .",size,buffer_size);
	
	for(i = 0; i <= num; i++)
	{
		mTempHWHeap = new RawPmemPool("/dev/pmem_adsp",
                            buffer_size,
                            count,
                            buffer_size,
                            0,
                            "temp memory by hw");
                if (!mTempHWHeap->initialized()) {
			LOGE("Fail to mTempHWHeap.time: %d.", i + 1);
            		mTempHWHeap = NULL;
			if(i == num)
	            		return NULL;
			usleep(200000);
        	}
		else
			break;
	}
	
        if (mTempHWHeap != NULL && mTempHWHeap->mHeap != NULL) {
            	*phy_addr = (uint32_t)get_physical_address(mTempHWHeap->mHeap);
                uint32_t vaddr = (uint32_t)mTempHWHeap->mHeap->base(); 

                LOGV("get_temp_mem_by_HW: MALLOC size: %d, num: %d --> %p",
                      buffer_size, count, (void *)vaddr);                
                return (void *)vaddr;           
        }

        LOGV("get_temp_mem_by_HW: X NULL");
        return NULL;
    }

    void SprdCameraHardware::free_temp_mem_by_HW(void)
    {
    	mTempHWHeap = NULL;		
        LOGV("free_temp_mem: EX NOP");	
		
        return;
    }
    void SprdCameraHardware::receivePreviewFrame(camera_frame_type *frame)
    {
        Mutex::Autolock cbLock(&mCallbackLock);
/*
        // Ignore the first frame--there is a bug in the VFE pipeline and that
        // frame may be bad.
        if (++mPreviewCount == 1) {
            camera_release_frame(0);
            return;
        }

        // Find the offset within the heap of the current buffer.
        ssize_t offset = (uint32_t)frame->buf_Virt_Addr;
        offset -= (uint32_t)mPreviewHeap->mHeap->base();
        //ssize_t frame_size = kRawFrameHeaderSize + frame->dx * frame->dy * 2;        
        ssize_t frame_size = 0;
        if(CAMERA_RGB565 == frame->format)
	        frame_size = frame->dx * frame->dy * 2;        //wxz: for RGB565
	else if(CAMERA_YCBCR_4_2_2 == frame->format)
	        frame_size = frame->dx * frame->dy * 2;        //wxz: for YUV422
	else if(CAMERA_YCBCR_4_2_0 == frame->format)
	        frame_size = frame->dx * frame->dy * 3 / 2;        //wxz: for YUV420
	else
		frame_size = frame->dx * frame->dy * 2;  
        if (offset + frame_size <=
                (ssize_t)mPreviewHeap->mHeap->virtualSize()) {

            offset /= frame_size;
*/
	ssize_t offset = frame->buf_id;   	

        // Ignore the first frame--there is a bug in the VFE pipeline and that
        // frame may be bad.
        if (++mPreviewCount == 1) {
            if(CAMERA_SUCCESS != camera_release_frame(offset)){
		LOGE("receivePreviewFrame: fail to camera_release_frame().offset: %d.", (uint32_t)offset);
	    }
            return;
        }
		
	    if(mData_cb != NULL)
	    {
		//LOGV("receivePreviewFrame mMsgEnabled: 0x%x",mMsgEnabled);
		if (mMsgEnabled & CAMERA_MSG_PREVIEW_FRAME)		
	    		mData_cb(CAMERA_MSG_PREVIEW_FRAME, mPreviewHeap->mBuffers[offset], mUser);
            	if (mMsgEnabled & CAMERA_MSG_VIDEO_FRAME)		
            	{
            		nsecs_t timestamp = systemTime(); 
	    		mData_cb_timestamp(timestamp, CAMERA_MSG_VIDEO_FRAME, mPreviewHeap->mBuffers[offset], mUser);	
			//ssize_t off;
			//size_t size;
			//mPreviewHeap->mBuffers[offset]->getMemory(&off, &size) ;
			//LOGV("receivePreviewFrame: record index: %d, offset: %x, size: %x, frame->buf_Virt_Addr: 0x%x.", offset, off, size, (uint32_t)frame->buf_Virt_Addr);	
            	}
	    	else{
            		if(CAMERA_SUCCESS != camera_release_frame(offset)){
				LOGE("receivePreviewFrame: fail to camera_release_frame().offset: %d.", (uint32_t)offset);
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
                
             }
	    else
		LOGE("receivePreviewFrame: mData_cb is null.");	
/*        }
        else LOGE("Preview frame virtual address %p is out of range!virtualSize: %x.",
                  frame->buf_Virt_Addr, (uint32_t)mPreviewHeap->mHeap->virtualSize());
*/
    }

/*    void
    SprdCameraHardware::notifyShutter()
    {
        LOGV("notifyShutter: E");
        print_time();
        Mutex::Autolock lock(&mStateLock);
        if (mShutterCallback)
            mShutterCallback(mPictureCallbackCookie);
        print_time();
        LOGV("notifyShutter: X");
    }*/
    void SprdCameraHardware::notifyShutter()
    {
        LOGV("notifyShutter: E");
        print_time();
        Mutex::Autolock lock(&mStateLock);
        
        /*if (mShutterCallback)
            mShutterCallback(mPictureCallbackCookie);*/
       //if(mNotify_cb)
       LOGV("notifyShutter mMsgEnabled: 0x%x.", mMsgEnabled);
       if (mMsgEnabled & CAMERA_MSG_SHUTTER) 
       	  mNotify_cb(CAMERA_MSG_SHUTTER, 0, 0, mUser);
        print_time();
        LOGV("notifyShutter: X");
    }

    // Pass the pre-LPM raw picture to raw picture callback.
    // This method is called by a libqcamera thread, different from the one on
    // which startPreview() or takePicture() are called.
    void SprdCameraHardware::receiveRawPicture(camera_frame_type *frame)
    {
        LOGV("receiveRawPicture: E");
        print_time();

        Mutex::Autolock cbLock(&mCallbackLock);

        //if (mRawPictureCallback != NULL) {
        if (mData_cb!= NULL) {
        // Find the offset within the heap of the current buffer.
        ssize_t offset = (uint32_t)frame->buf_Virt_Addr;
        offset -= (uint32_t)mRawHeap->mHeap->base();        
        ssize_t frame_size = 0;
        if(CAMERA_RGB565 == frame->format)
	        frame_size = frame->dx * frame->dy * 2;        //wxz: for RGB565
	else if(CAMERA_YCBCR_4_2_2 == frame->format)
	        frame_size = frame->dx * frame->dy * 2;        //wxz: for YUV422
	else if(CAMERA_YCBCR_4_2_0 == frame->format)
	        frame_size = frame->dx * frame->dy * 3 / 2;        //wxz: for YUV420
	else
		frame_size = frame->dx * frame->dy * 2;  
        if (offset + frame_size <=
                (ssize_t)mRawHeap->mHeap->virtualSize()) {

                offset /= frame_size;
		LOGV("mMsgEnabled: 0x%x, offset: %d.",mMsgEnabled, (uint32_t)offset);
		if (mMsgEnabled & CAMERA_MSG_RAW_IMAGE)		
			mData_cb(CAMERA_MSG_RAW_IMAGE,mRawHeap->mBuffers[offset],mUser);
		//if (mMsgEnabled & CAMERA_MSG_COMPRESSED_IMAGE)	//wxz:???
		//	mData_cb(CAMERA_MSG_COMPRESSED_IMAGE, mRawHeap->mBuffers[offset],mUser);
                /*mRawPictureCallback(mRawHeap->mBuffers[offset],
                                    mPictureCallbackCookie);*/
            }
            else LOGE("receiveRawPicture: virtual address %p is out of range!",
                      frame->buf_Virt_Addr);
        }
        else LOGV("Raw-picture callback was canceled--skipping.");
	mJpegencZoomHeap = NULL; //free the zoom PMEM buffer.
	mJpegencSwapHeap = NULL;
        print_time();
        LOGV("receiveRawPicture: X");
    }

    // Encode the post-LPM raw picture.
    // This method is called by a libqcamera thread, different from the one on
    // which startPreview() or takePicture() are called.

    void
    SprdCameraHardware::receivePostLpmRawPicture(camera_frame_type *frame)
    {
        LOGV("receivePostLpmRawPicture: E");
        print_time();
        Sprd_camera_state new_state = QCS_ERROR;
	

        Mutex::Autolock cbLock(&mCallbackLock);

        //if (mJpegPictureCallback != NULL) {
        if (mData_cb!= NULL) {

            bool encode_location = true;

#define PARSE_LOCATION(what,type,fmt,desc) do {                                           \
                pt.what = 0;                                                              \
                const char *what##_str = mParameters.get("gps-"#what);                    \
                LOGV("receiveRawPicture: GPS PARM %s --> [%s]", "gps-"#what, what##_str); \
                if (what##_str) {                                                         \
                    type what = 0;                                                        \
                    if (sscanf(what##_str, fmt, &what) == 1)                              \
                        pt.what = what;                                                   \
                    else {                                                                \
                        LOGE("GPS " #what " %s could not"                                 \
                              " be parsed as a " #desc,                                   \
                              what##_str);                                                \
                        encode_location = false;                                          \
                    }                                                                     \
                }                                                                         \
                else {                                                                    \
                    LOGW("receiveRawPicture: GPS " #what " not specified: "               \
                          "defaulting to zero in EXIF header.");                          \
                    encode_location = false;                                              \
               }                                                                          \
            } while(0)

            PARSE_LOCATION(timestamp, long, "%ld", "long");
            if (!pt.timestamp) pt.timestamp = time(NULL);
            PARSE_LOCATION(altitude, short, "%hd", "short");
            PARSE_LOCATION(latitude, double, "%lf", "double float");
            PARSE_LOCATION(longitude, double, "%lf", "double float");
	    //PARSE_LOCATION(processing-method, const char*, "%s", "const char*");
	    pt.process_method = mParameters.get("gps-processing-method");

#undef PARSE_LOCATION

            if (encode_location) {
                LOGV("receiveRawPicture: setting image location ALT %d LAT %lf LON %lf",
                     pt.altitude, pt.latitude, pt.longitude);
                if (camera_set_position(&pt, NULL, NULL) != CAMERA_SUCCESS) {
                    LOGE("receiveRawPicture: camera_set_position: error");
                    // return;  // not a big deal
                }
            }
            else 
		LOGV("receiveRawPicture: not setting image location");

            mJpegSize = 0;
	/*
            camera_handle.device = CAMERA_DEVICE_MEM;
            camera_handle.mem.encBuf_num =  MAX_JPEG_ENCODE_BUF_NUM;

            for (int cnt = 0; cnt < MAX_JPEG_ENCODE_BUF_NUM; cnt++) {
                camera_handle.mem.encBuf[cnt].buffer = (uint8_t *)
                    malloc(MAX_JPEG_ENCODE_BUF_LEN);
                camera_handle.mem.encBuf[cnt].buf_len =
                    MAX_JPEG_ENCODE_BUF_LEN;
                camera_handle.mem.encBuf[cnt].used_len = 0;
            } // for 
            */

            if(CAMERA_SUCCESS != camera_encode_picture(frame, &camera_handle, camera_cb, this)){
		mCameraState = QCS_ERROR;
		LOGE("receivePostLpmRawPicture: fail to camera_encode_picture().");
	    }
        }
        else {
            LOGV("JPEG callback was cancelled--not encoding image.");
            // We need to keep the raw heap around until the JPEG is fully
            // encoded, because the JPEG encode uses the raw image contained in
            // that heap.
            mRawHeap = NULL;
        }                    
        print_time();
        LOGV("receivePostLpmRawPicture: X");
    }

    void   SprdCameraHardware::receiveJpegPictureFragment( JPEGENC_CBrtnType *encInfo)
    {
        camera_encode_mem_type *enc =
            (camera_encode_mem_type *)encInfo->outPtr;
        //int index = enc - camera_handle.mem.encBuf; //wxz: ????
        //int index = 0;
        uint8_t *base = (uint8_t *)mJpegHeap->mHeap->base();
        uint32_t size = encInfo->size;
        uint32_t remaining = mJpegHeap->mHeap->virtualSize();
        remaining -= mJpegSize;
	LOGV("receiveJpegPictureFragment E.");
        LOGV("receiveJpegPictureFragment: (status %d size %d remaining %d)",         
             encInfo->status,
             size, remaining);

        if (size > remaining) {
            LOGE("receiveJpegPictureFragment: size %d exceeds what "
                 "remains in JPEG heap (%d), truncating",
                 size,
                 remaining);
            size = remaining;
        }
       
        //camera_handle.mem.encBuf[index].used_len = 0;
	LOGV("receiveJpegPictureFragment : base + mJpegSize: %x, enc->buffer: %x, size: %x", (uint32_t)base, (uint32_t)enc->buffer, size) ;
	memcpy(base + mJpegSize, enc->buffer, size);
        mJpegSize += size;	

	LOGV("receiveJpegPictureFragment X.");
    }

    // This method is called by a libqcamera thread, different from the one on
    // which startPreview() or takePicture() are called.

    void
    SprdCameraHardware::receiveJpegPicture(void)
    {
        LOGV("receiveJpegPicture: E image (%d bytes out of %d)",
             mJpegSize, mJpegHeap->mBufferSize);
        print_time();
        Mutex::Autolock cbLock(&mCallbackLock);

        int index = 0;
	
        //if (mJpegPictureCallback) {
        if (mData_cb) {
	    LOGV("receiveJpegPicture: mData_cb.");
            // The reason we do not allocate into mJpegHeap->mBuffers[offset] is
            // that the JPEG image's size will probably change from one snapshot
            // to the next, so we cannot reuse the MemoryBase object.
            sp<MemoryBase> buffer = new
                MemoryBase(mJpegHeap->mHeap,
                           index * mJpegHeap->mBufferSize +
                           mJpegHeap->mFrameOffset,
                           mJpegSize);
            LOGV("receiveJpegPicture:  mMsgEnabled: 0x%x.", mMsgEnabled);
           // mJpegPictureCallback(buffer, mPictureCallbackCookie);
           if (mMsgEnabled & CAMERA_MSG_COMPRESSED_IMAGE)
	           mData_cb(CAMERA_MSG_COMPRESSED_IMAGE,buffer, mUser );
            buffer = NULL;
        }
        else LOGV("JPEG callback was cancelled--not delivering image.");

        // NOTE: the JPEG encoder uses the raw image contained in mRawHeap, so we need
        // to keep the heap around until the encoding is complete.
        LOGV("receiveJpegPicture: free the Raw and Jpeg mem.");
        mJpegHeap = NULL;	
        mRawHeap = NULL;
        mJpegencHWHeap = NULL;

	/*
        for (int cnt = 0; cnt < MAX_JPEG_ENCODE_BUF_NUM; cnt++) {
            if (camera_handle.mem.encBuf[cnt].buffer != NULL) {
                free(camera_handle.mem.encBuf[cnt].buffer);
                memset(camera_handle.mem.encBuf + cnt, 0,
                       sizeof(camera_encode_mem_type));
            }
        } // for 
        */

        print_time();
        LOGV("receiveJpegPicture: X callback done.");
    }

#if 0    
struct str_map {
        const char *const desc;
        int val;
    };

    static const struct str_map wb_map[] = {
        { "auto", CAMERA_WB_AUTO },
        { "custom", CAMERA_WB_CUSTOM },
        { "incandescent", CAMERA_WB_INCANDESCENT },
        { "fluorescent", CAMERA_WB_FLUORESCENT },
        { "daylight", CAMERA_WB_DAYLIGHT },
        { "cloudy", CAMERA_WB_CLOUDY_DAYLIGHT },
        { "twilight", CAMERA_WB_TWILIGHT },
        { "shade", CAMERA_WB_SHADE },
        { NULL, 0 }
    };

    static const struct str_map effect_map[] = {
        { "off", CAMERA_EFFECT_OFF },
        { "mono", CAMERA_EFFECT_MONO },
        { "negative", CAMERA_EFFECT_NEGATIVE },
        { "solarize", CAMERA_EFFECT_SOLARIZE },
        { "pastel", CAMERA_EFFECT_PASTEL },
        { "mosaic", CAMERA_EFFECT_MOSAIC },
        { "resize", CAMERA_EFFECT_RESIZE },
        { "sepia", CAMERA_EFFECT_SEPIA },
        { "posterize", CAMERA_EFFECT_POSTERIZE },
        { "whiteboard", CAMERA_EFFECT_WHITEBOARD },
        { "blackboard", CAMERA_EFFECT_BLACKBOARD },
        { "aqua", CAMERA_EFFECT_AQUA },
        { NULL, 0 }
    };

    static const struct str_map brightness_map[] = {
        { "0", CAMERA_BRIGHTNESS_0 },
        { "1", CAMERA_BRIGHTNESS_1 },
        { "2", CAMERA_BRIGHTNESS_2 },
        { "3", CAMERA_BRIGHTNESS_3 },
        { "4", CAMERA_BRIGHTNESS_4 },
        { "5", CAMERA_BRIGHTNESS_5 },
        { "6", CAMERA_BRIGHTNESS_6 },
        { "7", CAMERA_BRIGHTNESS_7 },
        { "8", CAMERA_BRIGHTNESS_8 },
        { "9", CAMERA_BRIGHTNESS_9 },
        { "10", CAMERA_BRIGHTNESS_10 },
        { NULL, 0 }
    };
#endif

    static const struct str_map antibanding_map[] = {
        { "off", CAMERA_ANTIBANDING_OFF },
        { "50hz", CAMERA_ANTIBANDING_50HZ },
        { "60hz", CAMERA_ANTIBANDING_60HZ },
        { "auto", CAMERA_ANTIBANDING_AUTO },
        { NULL, 0 }
    };

    static const struct str_map iso_map[] = {
        { "auto", CAMERA_ISO_AUTO },
        { "high", CAMERA_ISO_HIGH },
        { NULL, 0 }
    };

    static int lookup(const struct str_map *const arr, const char *name, int def)
    {
        if (name) {
            const struct str_map * trav = arr;
            while (trav->desc) {
                if (!strcmp(trav->desc, name))
                    return trav->val;
                trav++;
            }
        }
        return def;
    }

    void SprdCameraHardware::initCameraParameters()
    {
        LOGV("initCameraParameters: E");

        // Because libqcamera is broken, for the camera_set_parm() calls
        // SprdCameraHardware camera_cb() is called synchronously,
        // so we cannot wait on a state change.  Also, we have to unlock
        // the mStateLock, because camera_cb() acquires it.

        startCameraIfNecessary();

#define SET_PARM(x,y) do {                                             \
        LOGV("initCameraParameters: set parm: %s, %d", #x, y);         \
        camera_set_parm (x, y, NULL, NULL);                       \
    } while(0)

        // Preview Mode: snapshot or movie 
        SET_PARM(CAMERA_PARM_PREVIEW_MODE, CAMERA_PREVIEW_MODE_SNAPSHOT);

        // Default Rotation - none 
        int rotation = mParameters.getInt("rotation");

        // Rotation may be negative, but may not be -1, because it has to be a
        // multiple of 90.  That's why we can still interpret -1 as an error,
        if (rotation == -1) {
            LOGV("rotation not specified or is invalid, defaulting to 0");
            rotation = 0;
        }
        else if (rotation % 90) {
            LOGV("rotation %d is not a multiple of 90 degrees!  Defaulting to zero.",
                 rotation);
            rotation = 0;
        }
        else {
            // normalize to [0 - 270] degrees
            rotation %= 360;
            if (rotation < 0) rotation += 360;
        }
        SET_PARM(CAMERA_PARM_ENCODE_ROTATION, rotation);
        rotation = mParameters.getInt("sensorrotation");
        SET_PARM(CAMERA_PARM_SENSOR_ROTATION, rotation);

        SET_PARM(CAMERA_PARM_WB,
                 lookup(wb_map,
                        mParameters.get("whitebalance"),
                        CAMERA_WB_AUTO));
        SET_PARM(CAMERA_PARM_CAMERA_ID,
                 lookup(camera_id_map,
                        mParameters.get("cameraid"),
                        CAMERA_CAMERA_ID_BACK));
        SET_PARM(CAMERA_PARM_JPEGCOMP,  atoi(mParameters.get("jpeg-quality")));		
        SET_PARM(CAMERA_PARM_EFFECT,
                 lookup(effect_map,
                        mParameters.get("effect"),
                        CAMERA_EFFECT_NONE));
        SET_PARM(CAMERA_PARM_SCENE_MODE,
                 lookup(scene_mode_map,
                        mParameters.get("scene-mode"),
                        CAMERA_SCENE_MODE_AUTO));
	g_sprd_zoom_levle = lookup(zoom_map,mParameters.get("zoom"), CAMERA_ZOOM_1X);
        SET_PARM(CAMERA_PARM_ZOOM, g_sprd_zoom_levle);		

        SET_PARM(CAMERA_PARM_BRIGHTNESS,
                 lookup(brightness_map,
                        mParameters.get("brightness"),
                        CAMERA_BRIGHTNESS_DEFAULT));
        SET_PARM(CAMERA_PARM_CONTRAST,
                 lookup(contrast_map,
                        mParameters.get("contrast"),
                        CAMERA_CONTRAST_DEFAULT));		

        SET_PARM(CAMERA_PARM_ISO,
                 lookup(iso_map,
                        mParameters.get("iso"),
                        CAMERA_ISO_AUTO));

        SET_PARM(CAMERA_PARM_ANTIBANDING,
                 lookup(antibanding_map,
                        mParameters.get("antibanding"),
                        CAMERA_ANTIBANDING_AUTO));

        int ns_mode = mParameters.getInt("nightshot-mode");
        if (ns_mode < 0) ns_mode = 0;
        SET_PARM(CAMERA_PARM_NIGHTSHOT_MODE, ns_mode);
        
	if(1 == mParameters.getInt("sensororientation")){
        	SET_PARM(CAMERA_PARM_ORIENTATION, 1); //for portrait
	}
	else{
        	SET_PARM(CAMERA_PARM_ORIENTATION, 0); //for landscape
	}

        int luma_adaptation = mParameters.getInt("luma-adaptation");
        if (luma_adaptation < 0) luma_adaptation = 0;
        SET_PARM(CAMERA_PARM_LUMA_ADAPTATION, luma_adaptation);

	double focal_len = atof(mParameters.get("focal-length")) * 1000;
	SET_PARM(CAMERA_PARM_FOCAL_LENGTH,  (int32_t)focal_len);

#undef SET_PARM

#if 0
        // Default Auto FPS: 30 (maximum) 
        camera_set_parm_2 (CAMERA_PARM_PREVIEW_FPS,
                                (1<<16|20), // max frame rate 30
                                (4<<16|20), // min frame rate 5
                                NULL,
                                NULL);
#endif

        int th_w, th_h, th_q;
        th_w = mParameters.getInt("jpeg-thumbnail-width");
        if (th_w < 0) LOGW("property jpeg-thumbnail-width not specified");

        th_h = mParameters.getInt("jpeg-thumbnail-height");
        if (th_h < 0) LOGW("property jpeg-thumbnail-height not specified");

        th_q = mParameters.getInt("jpeg-thumbnail-quality");
        if (th_q < 0) LOGW("property jpeg-thumbnail-quality not specified");

        if (th_w >= 0 && th_h >= 0 && th_q >= 0) {
            LOGI("setting thumbnail dimensions to %dx%d, quality %d",
                 th_w, th_h, th_q);
            int ret = camera_set_thumbnail_properties(th_w, th_h, th_q);
            if (ret != CAMERA_SUCCESS) {
                LOGE("camera_set_thumbnail_properties returned %d", ret);
            }
        }

#if defined FEATURE_CAMERA_ENCODE_PROPERTIES
        // Set Default JPEG encoding--this does not cause a callback 
        encode_properties.quality   = mParameters.getInt("jpeg-quality");
        if (encode_properties.quality < 0) {
            LOGW("JPEG-image quality is not specified "
                 "or is negative, defaulting to %d",
                 encode_properties.quality);
            encode_properties.quality = 100;
        }
        else LOGV("Setting JPEG-image quality to %d",
                  encode_properties.quality);
        encode_properties.format    = CAMERA_JPEG;
        encode_properties.file_size = 0x0;
        camera_set_encode_properties(&encode_properties);
#else
#warning 'FEATURE_CAMERA_ENCODE_PROPERTIES should be enabled!'
#endif


        LOGV("initCameraParameters: X");
    }

    // Called with mStateLock held!
    void SprdCameraHardware::setCameraDimensions()
    {
        if (mCameraState != QCS_IDLE) {
            LOGE("set camera dimensions: expecting state QCS_IDLE, not %s",
                 getCameraStateStr(mCameraState));
            return;
        }

        camera_set_dimensions(mRawWidth,
                                   mRawHeight,
                                   mPreviewWidth,
                                   mPreviewHeight,
                                   NULL,
                                   NULL);

    }

    SprdCameraHardware::Sprd_camera_state
     SprdCameraHardware::change_state(Sprd_camera_state new_state,
        bool lock)
    {
    	LOGV("start to change_state.");
        //if (lock) mStateLock.lock(); //wxz: ???
        LOGV("start to lock.");
        if (new_state != mCameraState) {
            // Due to the fact that we allow only one thread at a time to call
            // startPreview(), stopPreview(), or takePicture(), we know that
            // only one thread at a time may be blocked waiting for a state
            // transition on mStateWait.  That's why we signal(), not
            // broadcast().
LOGV("start to mCameraState.");
            LOGV("state transition %s --> %s",
                 getCameraStateStr(mCameraState),
                 getCameraStateStr(new_state));
LOGV("start to getCameraStateStr.");
            mCameraState = new_state;
            mStateWait.signal();
        }
		LOGV("start to signal.");
       // if (lock) mStateLock.unlock(); //wxz: ???
        LOGV("start to unlock.");
        return new_state;
    }

#define CAMERA_STATE(n) case n: if(n != CAMERA_FUNC_START_PREVIEW || cb != CAMERA_EVT_CB_FRAME) LOGV("STATE %s // STATUS %d", #n, cb);
#define TRANSITION(e,s) do { \
            obj->change_state(obj->mCameraState == e ? s : QCS_ERROR); \
        } while(0)
#define TRANSITION_LOCKED(e,s) do { \
            obj->change_state((obj->mCameraState == e ? s : QCS_ERROR), false); \
        } while(0)
#define TRANSITION_ALWAYS(s) obj->change_state(s)


    // This callback is called from the destructor.
    void SprdCameraHardware::stop_camera_cb(camera_cb_type cb,
                                                const void *client_data,
                                                camera_func_type func,
                                                int32_t parm4)
    {
        SprdCameraHardware *obj =
            (SprdCameraHardware *)client_data;
        switch(func) {
            CAMERA_STATE(CAMERA_FUNC_STOP)
                TRANSITION(QCS_INTERNAL_STOPPING, QCS_INIT);
            break;
        default:
            break;
        }
    }

    void SprdCameraHardware::camera_cb(camera_cb_type cb,
                                           const void *client_data,
                                           camera_func_type func,
                                           int32_t parm4)
    {
        SprdCameraHardware *obj =
            (SprdCameraHardware *)client_data;

        // Promote the singleton to make sure that we do not get destroyed
        // while this callback is executing.
        if (UNLIKELY(getInstance() == NULL)) {
            LOGE("camera object has been destroyed--returning immediately");
            return;
        }

        if (cb == CAMERA_EXIT_CB_ABORT ||     // Function aborted             
            cb == CAMERA_EXIT_CB_DSP_ABORT || // Abort due to DSP failure     
            cb == CAMERA_EXIT_CB_ERROR ||     // Failed due to resource       
            cb == CAMERA_EXIT_CB_FAILED)      // Execution failed or rejected 
        {
            // Autofocus failures occur relatively often and are not fatal, so
            // we do not transition to QCS_ERROR for them.
            if (func != CAMERA_FUNC_START_FOCUS) {
                LOGE("SprdCameraHardware::camera_cb: @CAMERA_EXIT_CB_FAILURE(%d) in state %s.",
                     parm4,
                     obj->getCameraStateStr(obj->mCameraState));
                TRANSITION_ALWAYS(QCS_ERROR);
            }
        }

        switch(func) {
            // This is the commonest case.
            CAMERA_STATE(CAMERA_FUNC_START_PREVIEW)
                switch(cb) {
                case CAMERA_RSP_CB_SUCCESS:
                    TRANSITION(QCS_INTERNAL_PREVIEW_REQUESTED,
                               QCS_PREVIEW_IN_PROGRESS);
                    break;
                case CAMERA_EVT_CB_FRAME:
                    switch (obj->mCameraState) {
                    case QCS_PREVIEW_IN_PROGRESS:
                        if (parm4)
                            obj->receivePreviewFrame((camera_frame_type *)parm4);
                        break;
                    case QCS_INTERNAL_PREVIEW_STOPPING:
                        LOGV("camera cb: discarding preview frame "
                             "while stopping preview");
                        break;
                    default:
                        // transition to QCS_ERROR
                        LOGE("camera cb: invalid state %s for preview!",
                             obj->getCameraStateStr(obj->mCameraState));
                        break;
                    }
// -- this function is called now inside of receivePreviewFrame.
//                    camera_release_frame();

                    break;
                default:
                    // transition to QCS_ERROR
                    LOGE("unexpected cb %d for CAMERA_FUNC_START_PREVIEW.",
                         cb);
                }
                break;
            CAMERA_STATE(CAMERA_FUNC_START)
                TRANSITION(QCS_INIT, QCS_IDLE);
		LOGV("OK to QCS_INIT, QCS_IDLE");
                break;
// -- this case handled in stop_camera_cb() now.
//            CAMERA_STATE(CAMERA_FUNC_STOP)
//                TRANSITION(QCS_INTERNAL_STOPPING, QCS_INIT);
//                break;

            CAMERA_STATE(CAMERA_FUNC_STOP_PREVIEW)
                TRANSITION(QCS_INTERNAL_PREVIEW_STOPPING,
                           QCS_IDLE);
                break;
            CAMERA_STATE(CAMERA_FUNC_TAKE_PICTURE)
                if (cb == CAMERA_RSP_CB_SUCCESS) {
		//	obj->notifyShutter();
                    TRANSITION(QCS_INTERNAL_RAW_REQUESTED,
                               QCS_WAITING_RAW);
                }
                else if (cb == CAMERA_EVT_CB_SNAPSHOT_DONE) {
                    obj->notifyShutter();
                    // Received pre-LPM raw picture. Notify callback now.
                    obj->receiveRawPicture((camera_frame_type *)parm4);		   
                }
                else if (cb == CAMERA_EXIT_CB_DONE)
		{
		     {
                        Mutex::Autolock lock(&obj->mStateLock);
                        TRANSITION_LOCKED(QCS_WAITING_RAW,
                                          //obj->mJpegPictureCallback != NULL ?
                                          obj->mData_cb != NULL ?
                                          QCS_WAITING_JPEG :
                                          QCS_IDLE);
                    }
                    // It's important that we call receiveRawPicture() before
                    // we transition the state because another thread may be
                    // waiting in cancelPicture(), and then delete this object.
                    // If the order were reversed, we might call
                    // receiveRawPicture on a dead object.
                    LOGV("Receiving post LPM raw picture.");
                    obj->receivePostLpmRawPicture((camera_frame_type *)parm4);
                    /*{
                        Mutex::Autolock lock(&obj->mStateLock);
                        TRANSITION_LOCKED(QCS_WAITING_RAW,
                                          //obj->mJpegPictureCallback != NULL ?
                                          obj->mData_cb != NULL ?
                                          QCS_WAITING_JPEG :
                                          QCS_IDLE);
                    }*/
                } else {  // transition to QCS_ERROR
                    if (obj->mCameraState == QCS_ERROR) {
                        LOGE("camera cb: invalid state %s for taking a picture!",
                             obj->getCameraStateStr(obj->mCameraState));
                        //obj->mRawPictureCallback(NULL, obj->mPictureCallbackCookie);
                        if (obj->mMsgEnabled & CAMERA_MSG_RAW_IMAGE)
                       		 obj->mData_cb(CAMERA_MSG_RAW_IMAGE, NULL, obj->mUser);
                        //obj->mJpegPictureCallback(NULL, obj->mPictureCallbackCookie);
                        if (obj->mMsgEnabled & CAMERA_MSG_COMPRESSED_IMAGE)
                        	obj->mData_cb(CAMERA_MSG_COMPRESSED_IMAGE, NULL, obj->mUser);
                        TRANSITION_ALWAYS(QCS_IDLE);
                    }
                }
                break;
            CAMERA_STATE(CAMERA_FUNC_ENCODE_PICTURE)
                switch (cb) {
                case CAMERA_RSP_CB_SUCCESS:
                    // We already transitioned the camera state to
                    // QCS_WAITING_JPEG when we called
                    // camera_encode_picture().
                    break;
                case CAMERA_EXIT_CB_BUFFER:
                    if (obj->mCameraState == QCS_WAITING_JPEG) {
                        obj->receiveJpegPictureFragment(
                            (JPEGENC_CBrtnType *)parm4);
                    }
                    else LOGE("camera cb: invalid state %s for receiving "
                              "JPEG fragment!",
                              obj->getCameraStateStr(obj->mCameraState));
                    break;
                case CAMERA_EXIT_CB_DONE:
                    if (obj->mCameraState == QCS_WAITING_JPEG) {
                        // Receive the last fragment of the image.
                        obj->receiveJpegPictureFragment(
                            (JPEGENC_CBrtnType *)parm4);

                        // The size of the complete JPEG image is in 
                        // mJpegSize.
			LOGV("CAMERA_EXIT_CB_DONE MID.");
                        // It's important that we call receiveJpegPicture()
                        // before we transition the state because another
                        // thread may be waiting in cancelPicture(), and then
                        // delete this object.  If the order were reversed, we
                        // might call receiveRawPicture on a dead object.

                        obj->receiveJpegPicture();

                        TRANSITION(QCS_WAITING_JPEG, QCS_IDLE);
                    }
                    // transition to QCS_ERROR
                    else LOGE("camera cb: invalid state %s for "
                              "receiving JPEG!",
                              obj->getCameraStateStr(obj->mCameraState));
                    break;
                default:
                    // transition to QCS_ERROR
                    LOGE("camera cb: unknown cb %d for JPEG!", cb);
                }
            break;
            CAMERA_STATE(CAMERA_FUNC_START_FOCUS) {
                // NO TRANSITION HERE.  We acquire mStateLock here because it is
                // possible for ::autoFocus to be called after the call to
                // mAutoFocusCallback() but before we set mAutoFocusCallback
                // to NULL.
                //if (obj->mAutoFocusCallback) {
                if (obj->mNotify_cb) {
                    switch (cb) {
                    case CAMERA_RSP_CB_SUCCESS:
                        LOGV("camera cb: autofocus has started.");
                        break;
                    case CAMERA_EXIT_CB_DONE: {
                        LOGV("camera cb: autofocus succeeded.");
                      /*  Mutex::Autolock lock(&obj->mStateLock);
                        if (obj->mAutoFocusCallback) {
                            obj->mAutoFocusCallback(true,
                                    obj->mAutoFocusCallbackCookie);
                            obj->mAutoFocusCallback = NULL;			
                        }*/
                        LOGV("camera cb: autofocus mNotify_cb start.");
			//if(NULL != obj->mNotify_cb)
			if (obj->mMsgEnabled & CAMERA_MSG_FOCUS) 
				obj->mNotify_cb(CAMERA_MSG_FOCUS, 1, 0, obj->mUser);
			else
				LOGE("camera cb: mNotify_cb is null.");		
 		   LOGV("camera cb: autofocus mNotify_cb ok.");
                    }
                        break;
                    case CAMERA_EXIT_CB_ABORT:
                        LOGE("camera cb: autofocus aborted");
                        break;
                    case CAMERA_EXIT_CB_FAILED: {
                        LOGE("camera cb: autofocus failed");
                        Mutex::Autolock lock(&obj->mStateLock);
                        /*if (obj->mAutoFocusCallback) {
                            obj->mAutoFocusCallback(false,
                                    obj->mAutoFocusCallbackCookie);
                            obj->mAutoFocusCallback = NULL;
                        }*/
                        if (obj->mMsgEnabled & CAMERA_MSG_FOCUS)
                        	obj->mNotify_cb(CAMERA_MSG_FOCUS, 0, 0, obj->mUser);
                    }
                        break;
                    default:
                        LOGE("camera cb: unknown cb %d for "
                             "CAMERA_FUNC_START_FOCUS!", cb);
                    }
                }
            } break;
        default:
            // transition to QCS_ERROR
            LOGE("Unknown camera-callback status %d", cb);
        }
    }

#undef TRANSITION
#undef TRANSITION_LOCKED
#undef TRANSITION_ALWAYS
#undef CAMERA_STATE

    static unsigned clp2(unsigned x) {
        x = x - 1;
        x = x | (x >> 1);
        x = x | (x >> 2);
        x = x | (x >> 4);
        x = x | (x >> 8);
        x = x | (x >>16);
        return x + 1;
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
        // empty
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
            LOGV("constructing MemPool %s backed by ashmem: "
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

    SprdCameraHardware::PmemPool::PmemPool(const char *pmem_pool,
                                               int buffer_size, int num_buffers,
                                               int frame_size,
                                               int frame_offset,
                                               const char *name) :
        SprdCameraHardware::MemPool(buffer_size,
                                        num_buffers,
                                        frame_size,
                                        frame_offset,
                                        name)
    {
        LOGV("constructing MemPool %s backed by pmem pool %s: "
             "%d frames @ %d bytes, offset %d, buffer size %d",
             mName,
             pmem_pool, num_buffers, frame_size, frame_offset,
             buffer_size);
        
        // Make a new mmap'ed heap that can be shared across processes.
        
        mAlignedSize = clp2(buffer_size * num_buffers);
        
        sp<MemoryHeapBase> masterHeap = 
            new MemoryHeapBase(pmem_pool, mAlignedSize, 0);
        sp<MemoryHeapPmem> pmemHeap = new MemoryHeapPmem(masterHeap, 0);
        if (pmemHeap->getHeapID() >= 0) {
            pmemHeap->slap();
            masterHeap.clear();
            mHeap = pmemHeap;
            pmemHeap.clear();
            
            mFd = mHeap->getHeapID();
            if (::ioctl(mFd, PMEM_GET_SIZE, &mSize)) {
                LOGE("pmem pool %s ioctl(PMEM_GET_SIZE) error %s (%d)",
                     pmem_pool,
                     ::strerror(errno), errno);
                mHeap.clear();
                return;
            }
            
            LOGV("pmem pool %s ioctl(PMEM_GET_SIZE) is %ld",
                 pmem_pool,
                 mSize.len);
            
            completeInitialization();
        }
        else LOGE("pmem pool %s error: could not create master heap!",
                  pmem_pool);
    }

    SprdCameraHardware::PreviewPmemPool::PreviewPmemPool(
            int buffer_size, int num_buffers,
            int frame_size,
            int frame_offset,
            const char *name) :
        SprdCameraHardware::PmemPool("/dev/pmem_adsp",
                                         buffer_size,
                                         num_buffers,
                                         frame_size,
                                         frame_offset,
                                         name)
    {
        LOGV("constructing PreviewPmemPool");
        if (initialized()) {
	    LOGV("camera_assoc_pmem in constructing PreviewPmemPool");
            camera_assoc_pmem(QDSP_MODULE_VFETASK, mFd,mHeap->base(),mAlignedSize,0); // external 
        }
    }

    SprdCameraHardware::PreviewPmemPool::~PreviewPmemPool()
    {
        LOGV("destroying PreviewPmemPool");
        if(initialized()) {
            void *base = mHeap->base();
            LOGV("releasing PreviewPmemPool memory %p from module %d",
                 base, QDSP_MODULE_VFETASK);
            camera_release_pmem(QDSP_MODULE_VFETASK, base,mAlignedSize,true); 
        }
    }

    SprdCameraHardware::RawPmemPool::RawPmemPool(
            const char *pmem_pool,
            int buffer_size, int num_buffers,
            int frame_size,
            int frame_offset,
            const char *name) :
        SprdCameraHardware::PmemPool(pmem_pool,
                                         buffer_size,
                                         num_buffers,
                                         frame_size,
                                         frame_offset,
                                         name)
    {
        LOGV("constructing RawPmemPool");

        if (initialized()) {
            camera_assoc_pmem(QDSP_MODULE_VFETASK,
                                   mFd,
                                   mHeap->base(),
                                   mAlignedSize,
                                   0); // do not free, main module
            camera_assoc_pmem(QDSP_MODULE_LPMTASK,
                                   mFd,
                                   mHeap->base(),
                                   mAlignedSize,
                                   2); // do not free, dependent module
            camera_assoc_pmem(QDSP_MODULE_JPEGTASK,
                                   mFd,
                                   mHeap->base(),
                                   mAlignedSize,
                                   2); // do not free, dependent module
        }
    }

    SprdCameraHardware::RawPmemPool::~RawPmemPool()
    {
        LOGV("destroying RawPmemPool");
        if(initialized()) {
            void *base = mHeap->base();
            LOGV("releasing RawPmemPool memory %p from modules %d, %d, and %d",
                 base, QDSP_MODULE_VFETASK, QDSP_MODULE_LPMTASK,
                 QDSP_MODULE_JPEGTASK);
            camera_release_pmem(QDSP_MODULE_VFETASK,
                                     base, mAlignedSize, true);
            camera_release_pmem(QDSP_MODULE_LPMTASK, 
                                     base, mAlignedSize, true);
            camera_release_pmem(QDSP_MODULE_JPEGTASK,
                                     base, mAlignedSize, true);
        }
    }
    
    SprdCameraHardware::MemPool::~MemPool()
    {
        LOGV("destroying MemPool %s", mName);
        if (mFrameSize > 0)
            delete [] mBuffers;
        mHeap.clear();
        LOGV("destroying MemPool %s completed", mName);        
    }
    
    status_t SprdCameraHardware::MemPool::dump(int fd, const Vector<String16>& args) const
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
    
    static uint8_t* malloc_preview(uint32_t size,
            uint32_t *phy_addr, uint32_t index)
    {
        sp<SprdCameraHardware> obj = SprdCameraHardware::getInstance();
        if (obj != 0) {
            return (uint8_t *)obj->get_preview_mem(size, phy_addr, index);
        }
        return NULL;
    }

    static int free_preview(uint32_t *phy_addr, uint32_t size,
                            uint32_t index)
    {
        sp<SprdCameraHardware> obj = SprdCameraHardware::getInstance();
        if (obj != 0) {
            obj->free_preview_mem(phy_addr, size, index);
        }
        return 0;
    }

    static uint8_t* malloc_raw(uint32_t size,
                                  uint32_t *phy_addr,
                                  uint32_t index)
    {
        sp<SprdCameraHardware> obj = SprdCameraHardware::getInstance();
        if (obj != 0) {
            return (uint8_t *)obj->get_raw_mem(size, phy_addr, index);
        }
        return NULL;
    }

    static int free_raw(uint32_t *phy_addr,
                        uint32_t size,
                        uint32_t index)
    {
        sp<SprdCameraHardware> obj = SprdCameraHardware::getInstance();
        if (obj != 0) {
            obj->free_raw_mem(phy_addr, size, index);
        }
        return 0;
    }

    /*static void cb_rex_signal_ready(void)
    {
        LOGV("Received REX-ready signal.");
        rex_init_lock.lock();
        rex_init_wait.broadcast();
        rex_init_lock.unlock();
    }*/

    //const char* const SprdCameraHardware::getCameraStateStr( //wxz:???
  /*  const char*  SprdCameraHardware::getCameraStateStr(
        SprdCameraHardware::Sprd_camera_state s)
    {
        static const char* states[] = {
#define STATE_STR(x) #x
            STATE_STR(QCS_INIT),
            STATE_STR(QCS_IDLE),
            STATE_STR(QCS_ERROR),
            STATE_STR(QCS_PREVIEW_IN_PROGRESS),
            STATE_STR(QCS_WAITING_RAW),
            STATE_STR(QCS_WAITING_JPEG),
            STATE_STR(QCS_INTERNAL_PREVIEW_STOPPING),
            STATE_STR(QCS_INTERNAL_PREVIEW_REQUESTED),
            STATE_STR(QCS_INTERNAL_RAW_REQUESTED),
            STATE_STR(QCS_INTERNAL_STOPPING),
#undef STATE_STR
        };
        return states[s];
    }*/
	const char* const SprdCameraHardware::getCameraStateStr(
        SprdCameraHardware::Sprd_camera_state s)
    {
        static const char* states[] = {
#define STATE_STR(x) #x
            STATE_STR(QCS_INIT),
            STATE_STR(QCS_IDLE),
            STATE_STR(QCS_ERROR),
            STATE_STR(QCS_PREVIEW_IN_PROGRESS),
            STATE_STR(QCS_WAITING_RAW),
            STATE_STR(QCS_WAITING_JPEG),
            STATE_STR(QCS_INTERNAL_PREVIEW_STOPPING),
            STATE_STR(QCS_INTERNAL_PREVIEW_REQUESTED),
            STATE_STR(QCS_INTERNAL_RAW_REQUESTED),
            STATE_STR(QCS_INTERNAL_STOPPING),
#undef STATE_STR
        };
        return states[s];
    }

}; // namespace android
