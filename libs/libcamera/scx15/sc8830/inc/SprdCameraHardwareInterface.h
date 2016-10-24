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
#ifndef ANDROID_HARDWARE_SPRD_CAMERA_HARDWARE_H
#define ANDROID_HARDWARE_SPRD_CAMERA_HARDWARE_H

#include <MemoryHeapIon_SPRD.h>
#include <utils/threads.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/types.h>

#include "SprdOEMCamera.h"
#include <utils/threads.h>
#include <utils/RefBase.h>
#include <binder/MemoryBase.h>
#include <binder/MemoryHeapBase.h>
#include <hardware/camera.h>
#include <hardware/gralloc.h>
#include <camera/CameraParameters.h>
#include "SprdCameraParameters.h"
#include "SprdOEMCamera.h"
#include "cmr_oem.h"
#include "sprd_dma_copy_k.h"

namespace android {

typedef void (*preview_callback)(sp<MemoryBase>, void * );
typedef void (*recording_callback)(sp<MemoryBase>, void *);
typedef void (*shutter_callback)(void *);
typedef void (*raw_callback)(sp<MemoryBase> , void *);
typedef void (*jpeg_callback)(sp<MemoryBase>, void *);
typedef void (*autofocus_callback)(bool, void *);

typedef struct sprd_camera_memory {
	camera_memory_t *camera_memory;
	MemoryHeapIon *ion_heap;
	uint32_t phys_addr, phys_size;
	void *handle;
	void *data;
	bool busy_flag;
}sprd_camera_memory_t;


#define MAX_SUB_RAWHEAP_NUM 10

class SprdCameraHardware : public virtual RefBase {
public:
	SprdCameraHardware(int cameraId);
	virtual                      ~SprdCameraHardware();
	inline int                   getCameraId() const;
	virtual void                 release();
	virtual status_t             startPreview();
	virtual void                 stopPreview();
	virtual bool                 previewEnabled();
	virtual status_t             setPreviewWindow(preview_stream_ops *w);
	virtual status_t             startRecording();
	virtual void                 stopRecording();
	virtual void                 releaseRecordingFrame(const void *opaque);
	virtual bool                 recordingEnabled();
	virtual status_t             takePicture();
	virtual status_t             cancelPicture();
	virtual status_t             setTakePictureSize(uint32_t width, uint32_t height);
	virtual status_t             autoFocus();
	virtual status_t             cancelAutoFocus();
	virtual status_t             setParameters(const SprdCameraParameters& params);
	virtual SprdCameraParameters getParameters();
	virtual void                 setCallbacks(camera_notify_callback notify_cb,
						camera_data_callback data_cb,
						camera_data_timestamp_callback data_cb_timestamp,
						camera_request_memory get_memory,
						void *user);
	virtual void                 enableMsgType(int32_t msgType);
	virtual void                 disableMsgType(int32_t msgType);
	virtual bool                 msgTypeEnabled(int32_t msgType);
	virtual status_t             sendCommand(int32_t cmd, int32_t arg1, int32_t arg2);
	virtual status_t             storeMetaDataInBuffers(bool enable);
	virtual status_t             dump(int fd) const;
	void                            setCaptureRawMode(bool mode);
	void		antiShakeParamSetup();
	int		uv420CopyTrim(struct _dma_copy_cfg_tag dma_copy_cfg);
	int		displayCopy(uint32_t dst_phy_addr, uint32_t dst_virtual_addr,
				uint32_t src_phy_addr, uint32_t src_virtual_addr, uint32_t src_w, uint32_t src_h);

	enum camera_flush_mem_type_e {
		CAMERA_FLUSH_RAW_HEAP,
		CAMERA_FLUSH_RAW_HEAP_ALL,
		CAMERA_FLUSH_PREVIEW_HEAP,
		CAMERA_FLUSH_MAX
	};

	int 						 flush_buffer(camera_flush_mem_type_e  type, int index, void *v_addr, void *p_addr, int size);
	sprd_camera_memory_t* 		 allocCameraMem(int buf_size, int num_bufs, uint32_t is_cache);

public:
	static int                   getPropertyAtv();
	static int                   getNumberOfCameras();
	static int                   getCameraInfo(int cameraId, struct camera_info *cameraInfo);
	static const CameraInfo      kCameraInfo[];
	static const CameraInfo      kCameraInfo3[];
	static int                   switch_monitor_thread_init(void *p_data);
	static int                   switch_monitor_thread_deinit(void *p_data);
	static void*                 switch_monitor_thread_proc(void *p_data);
	inline bool                  isCameraInit();
	status_t                     waitSetParamsOK();

private:
	inline void                  print_time();

    // This class represents a heap which maintains several contiguous
    // buffers.  The heap may be backed by pmem (when pmem_pool contains
    // the name of a /dev/pmem* file), or by ashmem (when pmem_pool == NULL).
	struct MemPool : public RefBase {
		MemPool(int buffer_size, int num_buffers, int frame_size,
					int frame_offset, const char *name);
		virtual ~MemPool() = 0;
		void     completeInitialization();
		bool     initialized() const {
			if (mHeap != NULL) {
				if(MAP_FAILED != mHeap->base())
					return true;
				else
					return false;
			} else {
				return false;
			}
		}
		virtual status_t dump(int fd, const Vector<String16>& args) const;
		int mBufferSize;
		int mNumBuffers;
		int mFrameSize;
		int mFrameOffset;
		sp<MemoryHeapBase> mHeap;
		sp<MemoryBase> *mBuffers;
		const char *mName;
	};

	struct AshmemPool : public MemPool {
		AshmemPool(int buffer_size, int num_buffers, int frame_size,
						int frame_offset, const char *name);
	};

	static int Callback_AllocCaptureMem(void* handle, unsigned int size, unsigned int *addr_phy, unsigned int *addr_vir);
	static int Callback_FreeCaptureMem(void* handle);

	void                  freeCameraMem(sprd_camera_memory_t* camera_memory);
	void                  clearCameraMem(sprd_camera_memory_t* camera_memory);
	uint32_t              getPreviewBufferID(buffer_handle_t *buffer_handle);
	void                  canclePreviewMem();
	int                     releasePreviewFrame();
	bool                  allocatePreviewMemByGraphics();
	bool                  allocatePreviewMem();
	void                  freePreviewMem();
	bool                  allocateCaptureMem(bool initJpegHeap);
	void                  freeCaptureMem();
	uint32_t              getRedisplayMem();
	void                  FreeReDisplayMem();
	status_t              checkSetParameters(const SprdCameraParameters& params);
	static void           camera_cb(camera_cb_type cb,
					const void *client_data,
					camera_func_type func,
					int32_t parm4);
	void                  notifyShutter();
	void                  receiveJpegPictureFragment(JPEGENC_CBrtnType *encInfo);
	void                  receiveJpegPosPicture(void);
	void                  receivePostLpmRawPicture(camera_frame_type *frame);
	void                  receiveRawPicture(camera_frame_type *frame);
	void                  receiveJpegPicture(JPEGENC_CBrtnType *encInfo);
	void                  receivePreviewFrame(camera_frame_type *frame);
	void                  receivePreviewFDFrame(camera_frame_type *frame);
	void                  receiveCameraExitError(void);
	void                  receiveTakePictureError(void);
	void                  receiveJpegPictureError(void);
	void                  HandleStopCamera(camera_cb_type cb, int32_t parm4);
	void                  HandleStartCamera(camera_cb_type cb, int32_t parm4);
	void                  HandleStartPreview(camera_cb_type cb, int32_t parm4);
	void                  HandleStopPreview(camera_cb_type cb, int32_t parm4);
	void                  HandleTakePicture(camera_cb_type cb, int32_t parm4);
	void                  HandleEncode(camera_cb_type cb,  int32_t parm4);
	void                  HandleFocus(camera_cb_type cb, int32_t parm4);
	void                  HandleCancelPicture(camera_cb_type cb, int32_t parm4);

	enum Sprd_camera_state {
		SPRD_INIT,
		SPRD_IDLE,
		SPRD_ERROR,
		SPRD_PREVIEW_IN_PROGRESS,
		SPRD_FOCUS_IN_PROGRESS,
		SPRD_SET_PARAMS_IN_PROGRESS,
		SPRD_WAITING_RAW,
		SPRD_WAITING_JPEG,

		// internal states
		SPRD_INTERNAL_PREVIEW_STOPPING,
		SPRD_INTERNAL_CAPTURE_STOPPING,
		SPRD_INTERNAL_PREVIEW_REQUESTED,
		SPRD_INTERNAL_RAW_REQUESTED,
		SPRD_INTERNAL_STOPPING,

	};

	enum state_owner {
		STATE_CAMERA,
		STATE_PREVIEW,
		STATE_CAPTURE,
		STATE_FOCUS,
		STATE_SET_PARAMS,
	};

	typedef struct _camera_state	{
		Sprd_camera_state      camera_state;
		Sprd_camera_state      preview_state;
		Sprd_camera_state      capture_state;
		Sprd_camera_state      focus_state;
		Sprd_camera_state      setParam_state;
	} camera_state;

	const char* getCameraStateStr(Sprd_camera_state s);
	Sprd_camera_state transitionState(Sprd_camera_state from,
						Sprd_camera_state to,
						state_owner owner,
						bool lock = true);
	void                            setCameraState(Sprd_camera_state state,
								state_owner owner = STATE_CAMERA);
	inline Sprd_camera_state        getCameraState();
	inline Sprd_camera_state        getPreviewState();
	inline Sprd_camera_state        getCaptureState();
	inline Sprd_camera_state        getFocusState();
	inline Sprd_camera_state        getSetParamsState();
	inline bool                     isCameraError();
	inline bool                     isCameraIdle();
	inline bool                     isPreviewing();
	inline bool                     isCapturing();
	bool                            WaitForPreviewStart();
	bool                            WaitForPreviewStop();
	bool                            WaitForCaptureStart();
	bool                            WaitForCaptureDone();
	bool                            WaitForCameraStart();
	bool                            WaitForCameraStop();
	bool                            WaitForFocusCancelDone();
	bool                            isRecordingMode();
	void                            setRecordingMode(bool enable);
	bool                            startCameraIfNecessary();
	void                            getPictureFormat(int *format);
	takepicture_mode                getCaptureMode();
	bool                            getCameraLocation(camera_position_type *pt);
	status_t                        startPreviewInternal(bool isRecordingMode);
	void                            stopPreviewInternal();
	status_t                        cancelPictureInternal();
	virtual status_t                setParametersInternal(const SprdCameraParameters& params);
	bool                            initPreview();
	void                            deinitPreview();
	bool                            initCapture(bool initJpegHeap);
	void                            deinitCapture();
	status_t                        initDefaultParameters();
	status_t                        setCameraParameters();
	status_t                        checkSetParametersEnvironment();
	status_t                        copyParameters(SprdCameraParameters& cur_params,
						const SprdCameraParameters& params);
	status_t                        checkSetParameters(const SprdCameraParameters& params,
							const SprdCameraParameters& oriParams);
	bool                            setCameraDimensions();
	void                            setCameraPreviewMode(bool isRecordMode);
	status_t                        set_ddr_freq(uint32_t mhzVal);
	bool                            displayOneFrame(uint32_t width,
								uint32_t height,
								uint32_t phy_addr, char *frame_addr,
								uint32_t id);
	bool                            displayOneFrameForCapture(uint32_t width,
		                              uint32_t height,
		                              uint32_t phy_addr,
		                              char *virtual_addr);
	void                            handleDataCallback(int32_t msg_type,
						uint32_t frame_index, unsigned int index,
						camera_frame_metadata_t *metadata, void *user,
						uint32_t isPrev);
	void                            handleDataCallbackTimestamp(int64_t timestamp,
						int32_t msg_type,
						sprd_camera_memory_t *data, unsigned int index,
						void *user);
	void                            cameraBakMemCheckAndFree();
	void                            sync_bak_parameters();
	bool                            iSDisplayCaptureFrame();
	bool                            iSZslMode();
	bool                            checkPreviewStateForCapture();
	bool                            getLcdSize(uint32_t *width, uint32_t *height);
	bool                            switchBufferMode(uint32_t src, uint32_t dst);
	status_t                        checkFlashParameter(SprdCameraParameters& params);
	void                            setCameraPrivateData(void);

	/* These constants reflect the number of buffers that libqcamera requires
	for preview and raw, and need to be updated when libqcamera
	changes.
	*/
	static const int                kPreviewBufferCount    = 8;
	static const int                kPreviewRotBufferCount = 4;
	static const int                kRawBufferCount        = 1;
	static const int                kJpegBufferCount       = 1;
	static const int                kRawFrameHeaderSize    = 0x0;
	Mutex                           mLock; // API lock -- all public methods
	Mutex                           mCallbackLock;
	Mutex                           mPreviewCbLock;
	Mutex                           mCaptureCbLock;
	Mutex                           mStateLock;
	Mutex                           mPreviewWindowLock;
	Condition                       mStateWait;
	Mutex                           mParamLock;
	Condition                       mParamWait;
	Mutex                           mCbPrevDataBusyLock;
	Mutex                           mPrevBakDataLock;
	Mutex                           mCbCapDataBusyLock;
	Mutex                           mCapBakDataLock;
	Mutex                           mPrevBufLock;
	Mutex                           mCapBufLock;
	Mutex                           mGraphBufCntLock;
	uint32_t                        mCapBufIsAvail;

	uint32_t                        mPreviewHeapSize;
	uint32_t                        mPreviewHeapNum;
	uint32_t                        mPreviewDcamAllocBufferCnt;
	sprd_camera_memory_t*           *mPreviewHeapArray;
	sprd_camera_memory_t            mPreviewHeapInfoBak;
	uint32_t                        mPreviewHeapBakUseFlag;
	sprd_camera_memory_t            mRawHeapInfoBak;
	uint32_t                        mRawHeapBakUseFlag;
	uint32_t                        mPreviewHeapArray_phy[kPreviewBufferCount+kPreviewRotBufferCount+1];
	uint32_t                        mPreviewHeapArray_vir[kPreviewBufferCount+kPreviewRotBufferCount+1];
	uint32_t                        mPreviewHeapArray_size[kPreviewBufferCount+kPreviewRotBufferCount+1];
	buffer_handle_t                 *mPreviewBufferHandle[kPreviewBufferCount];
	buffer_handle_t                 *mPreviewCancelBufHandle[kPreviewBufferCount];
	bool                            mCancelBufferEb[kPreviewBufferCount];
	int32_t                         mGraphBufferCount[kPreviewBufferCount];

	sprd_camera_memory_t            *mRawHeap;
	uint32_t                        mRawHeapSize;

	sprd_camera_memory_t            *mSubRawHeapArray[MAX_SUB_RAWHEAP_NUM];
	uint32_t                        mSubRawHeapNum;

	sp<AshmemPool>                  mJpegHeap;
	uint32_t                        mJpegHeapSize;
	uint32_t                        mFDAddr;
	camera_memory_t                 *mMetadataHeap;
	sprd_camera_memory_t            *mReDisplayHeap;
	//TODO: put the picture dimensions in the CameraParameters object;
	SprdCameraParameters            mParameters;
	SprdCameraParameters            mSetParameters;
	SprdCameraParameters            mSetParametersBak;
	uint32_t                        mPreviewHeight_trimy;
	uint32_t                        mPreviewWidth_trimx;
	int                             mPreviewHeight_backup;
	int                             mPreviewWidth_backup;
	int                             mPreviewHeight;
	int                             mPreviewWidth;
	int                             mRawHeight;
	int                             mRawWidth;
	int                             mPreviewFormat;//0:YUV422;1:YUV420;2:RGB
	int                             mPictureFormat;//0:YUV422;1:YUV420;2:RGB;3:JPEG
	int                             mPreviewStartFlag;
	uint32_t                        mIsDvPreview;

	bool                            mRecordingMode;
	bool                            mBakParamFlag;

	nsecs_t                         mRecordingFirstFrameTime;
	uint32_t                        mZoomLevel;
	/* mJpegSize keeps track of the size of the accumulated JPEG.  We clear it
	when we are about to take a picture, so at any time it contains either
	zero, or the size of the last JPEG picture taken.
	*/
	uint32_t                        mJpegSize;
	camera_notify_callback          mNotify_cb;
	camera_data_callback            mData_cb;
	camera_data_timestamp_callback  mData_cb_timestamp;
	camera_request_memory           mGetMemory_cb;
	void                            *mUser;
	preview_stream_ops              *mPreviewWindow;
	static gralloc_module_t const   *mGrallocHal;
	int32_t                         mMsgEnabled;
	bool                            mIsStoreMetaData;
	bool                            mIsFreqChanged;
	int32_t                         mCameraId;
	volatile camera_state           mCameraState;
	int                             miSPreviewFirstFrame;
	takepicture_mode                mCaptureMode;
	bool                            mCaptureRawMode;
	bool                            mIsRotCapture;
	bool                            mFlashMask;
	bool                            mReleaseFLag;
	uint32_t                        mTimeCoeff;
	uint32_t                        mPreviewBufferUsage;
	uint32_t                        mOriginalPreviewBufferUsage;
	int                             mSetDDRFreqCount;
	uint32_t                        mSetDDRFreq;

	/*callback thread*/
	pthread_t                       mSwitchMonitorThread;
	uint32_t                        mSwitchMonitorMsgQueHandle;
	uint32_t                        mSwitchMonitorInited;
	sem_t                           mSwitchMonitorSyncSem;

};

}; // namespace android

#endif //ANDROID_HARDWARE_SPRD_CAMERA_HARDWARE_H


