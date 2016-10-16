/*
**
** Copyright 2008, The Android Open Source Project
** Copyright 2012, Samsung Electronics Co. LTD
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/

#ifndef SPRD_CAMERA_HW_INTERFACE_2_H
#define SPRD_CAMERA_HW_INTERFACE_2_H

#include <hardware/camera2.h>
#include <camera/Camera.h>
#include <camera/CameraParameters.h>
#include <utils/List.h>
#include <MemoryHeapIon_SPRD.h>
#include <binder/MemoryBase.h>
#include "SprdBaseThread.h"
#include "gralloc_priv.h"
#include <fcntl.h>

#include "SprdCamera2.h"

#include "cutils/properties.h"
#include "SprdOEMCamera.h"
#include "SprdCameraHardwareConfig2.h"

namespace android {


#define NUM_MAX_CAMERA_BUFFERS      (16)

#define NUM_MAX_SUBSTREAM           (4)

#define STREAM_ID_PREVIEW           (1)
#define STREAM_MASK_PREVIEW         (1<<STREAM_ID_PREVIEW)
#define STREAM_ID_RECORD            (2)
#define STREAM_MASK_RECORD          (1<<STREAM_ID_RECORD)
#define STREAM_ID_PRVCB             (3)
#define STREAM_MASK_PRVCB           (1<<STREAM_ID_PRVCB)
#define STREAM_ID_JPEG              (4)
#define STREAM_MASK_JPEG            (1<<STREAM_ID_JPEG)
#define STREAM_ID_ZSL               (5)
#define STREAM_MASK_ZSL             (1<<STREAM_ID_ZSL)
#define STREAM_ID_CAPTURE           (6)
#define STREAM_ID_JPEG_REPROCESS    (8)
#define STREAM_ID_LAST              STREAM_ID_JPEG_REPROCESS

#define SIGNAL_REQ_THREAD_REQ_Q_NOT_EMPTY             (SIGNAL_THREAD_COMMON_LAST<<1)

#define SIGNAL_REQ_THREAD_REQ_DONE          (SIGNAL_THREAD_COMMON_LAST<<3)

#define SIGNAL_REQ_THREAD_PRECAPTURE_METERING_DONE          (SIGNAL_THREAD_COMMON_LAST<<4)

#define SIGNAL_THREAD_RELEASE                   (SIGNAL_THREAD_COMMON_LAST<<8)

#define SIGNAL_STREAM_REPROCESSING_START        (SIGNAL_THREAD_COMMON_LAST<<14)


#define SUBSTREAM_TYPE_NONE         (0)
#define SUBSTREAM_TYPE_JPEG         (1)
#define SUBSTREAM_TYPE_RECORD       (2)
#define SUBSTREAM_TYPE_PRVCB        (3)
#define SUBSTREAM_TYPE_ZSL          (4)
#define FLASH_STABLE_WAIT_TIMEOUT        (10)

#define SIG_WAITING_TICK            (5000)

#define MAX_REQUEST_NUM              (6)


#define ON_HAL_BUFQ         (1)
#define ON_HAL_BUFERR       (1 << 1)
#define ON_HAL_DRIVER       (1 << 2)
#define ON_SERVICE          (1 << 3)
#define ON_HAL_INIT         (1 << 4)/*service deq 6 bufs firstly*/

#define MAX_MISCHEAP_NUM 10

#define ALIGN_ZOOM_CROP_BITS (~0x03)
#define SIZE_ALIGN(x)   (((x)+15)&(~15))
#define FILT_FRM_NUM(num, base) (!((num) % (base)))
#define ZSLCAPFRMINTERVAL		3
#define ZSLPRVFRMINTERVAL		2


typedef struct stream_parameters {
	uint32_t                      width;
	uint32_t                      height;
	int                           format;
	const   camera2_stream_ops_t  *streamOps;
	uint32_t                      usage;
	int                           numSvcBuffers;
	int                           numSvcBufsInHal;
	buffer_handle_t               svcBufHandle[NUM_MAX_CAMERA_BUFFERS];
	int                           svcBufStatus[NUM_MAX_CAMERA_BUFFERS];
	int                           phySize[NUM_MAX_CAMERA_BUFFERS];
	int                           phyAdd[NUM_MAX_CAMERA_BUFFERS];/*for s_mem_method = 1*/
	int                           bufIndex;
	int                           minUndequedBuffer;
	int                           cancelBufNum;
	int64_t                       m_timestamp;
} stream_parameters_t;

typedef struct substream_parameters {
	int                           type;
	uint32_t                      width;
	uint32_t                      height;
	int                           format;
	const   camera2_stream_ops_t  *streamOps;
	uint32_t                      usage;
	int                           numSvcBuffers;
	int                           dataSize;/*get encode jpeg size*/
	buffer_handle_t               svcBufHandle[NUM_MAX_CAMERA_BUFFERS];
	uint32_t                      subStreamAddVirt[NUM_MAX_CAMERA_BUFFERS];
	int                           subStreamGraphicFd[NUM_MAX_CAMERA_BUFFERS];/*previewCbAddPhy*/
	int                           svcBufStatus[NUM_MAX_CAMERA_BUFFERS];
	int                           phySize[NUM_MAX_CAMERA_BUFFERS];
	struct	frm_info              zslCapFrmInfo[NUM_MAX_CAMERA_BUFFERS];
	int                           svcBufIndex;
	int                           numSvcBufsInHal;
	int                           minUndequedBuffer;
} substream_parameters_t;

typedef struct substream_entry {
	int                     priority;
	int                     streamId;
} substream_entry_t;

typedef struct sprd_camera_memory {
	MemoryHeapIon  *ion_heap;
	uint32_t       phys_addr, phys_size;
	void           *data;
} sprd_camera_memory_t;
typedef enum _capture_intent {
	CAPTURE_INTENT_CUSTOM = 0,
	CAPTURE_INTENT_PREVIEW,
	CAPTURE_INTENT_STILL_CAPTURE,
	CAPTURE_INTENT_VIDEO_RECORD,
	CAPTURE_INTENT_VIDEO_SNAPSHOT,
	CAPTURE_INTENT_ZERO_SHUTTER_LAG
} capture_intent;

typedef struct _SprdCamera2Info
{
int32_t        sensorW;
int32_t        sensorH;
int32_t        sensorRawW;
int32_t        sensorRawH;
int            numPreviewResolution;
const int32_t  *PreviewResolutions;
int            numJpegResolution;
const int32_t  *jpegResolutions;
float          minFocusDistance;
float          focalLength;
float          aperture;
float          fnumber;
const uint8_t  *availableAfModes;
const uint8_t  *sceneModeOverrides;
const uint8_t  *availableAeModes;
int            numAvailableAfModes;
int            numSceneModeOverrides;
int            numAvailableAeModes;
/* FD information */
int32_t        maxFaceCount;
}SprdCamera2Info;

typedef struct _reprocess_buf_info{
	buffer_handle_t  *reprocessAcqBuf;
	int              reprocessBufIndex;
} reprocess_buf_info;


class SprdCameraHWInterface2 : public virtual RefBase {
public:
	SprdCameraHWInterface2(int cameraId, camera2_device_t *dev, SprdCamera2Info * camera, int *openInvalid);
	virtual             ~SprdCameraHWInterface2();
	virtual void        release();

	inline  int         getCameraId() const
	{
		return m_CameraId;
	}

	virtual int         setRequestQueueSrcOps(const camera2_request_queue_src_ops_t *request_src_ops);
	virtual int         notifyRequestQueueNotEmpty();
	virtual int         setFrameQueueDstOps(const camera2_frame_queue_dst_ops_t *frame_dst_ops);
	virtual int         getInProgressCount();
	virtual int         flushCapturesInProgress();
	virtual int         ConstructDefaultRequest(int request_template, camera_metadata_t **request);

	virtual int         allocateStream(uint32_t width, uint32_t height,
				int format, const camera2_stream_ops_t *stream_ops,
				uint32_t *stream_id, uint32_t *format_actual, uint32_t *usage, uint32_t *max_buffers);

	virtual int         registerStreamBuffers(uint32_t stream_id, int num_buffers, buffer_handle_t *buffers);
	virtual int         releaseStream(uint32_t stream_id);

	virtual int         allocateReprocessStream(uint32_t width, uint32_t height,
				uint32_t format, const camera2_stream_in_ops_t *reprocess_stream_ops,
				uint32_t *stream_id, uint32_t *consumer_usage, uint32_t *max_buffers);

	virtual int         allocateReprocessStreamFromStream(uint32_t output_stream_id,
	const camera2_stream_in_ops_t  *reprocess_stream_ops, uint32_t *stream_id);
	virtual int         releaseReprocessStream(uint32_t stream_id);
	virtual int         triggerAction(uint32_t trigger_id, int ext1, int ext2);
	virtual int         setNotifyCallback(camera2_notify_callback notify_cb, void *user);
	virtual int         getMetadataVendorTagOps(vendor_tag_query_ops_t **ops);
	virtual int         dump(int fd);

	enum camera_flush_mem_type_e {
		CAMERA_FLUSH_RAW_HEAP,
		CAMERA_FLUSH_RAW_HEAP_ALL,
		CAMERA_FLUSH_PREVIEW_HEAP,
		CAMERA_FLUSH_HAL2_ZSL_BUF,
		CAMERA_FLUSH_MAX
	};

	int                 flush_buffer(camera_flush_mem_type_e  type, int index, void *v_addr, void *p_addr, int size);
	sprd_camera_memory_t  *GetCachePmem(int buf_size, int num_bufs);

private:

	typedef enum _metadata_mode {
		METADATA_MODE_NONE,
		METADATA_MODE_FULL
	} metadata_mode;

	typedef enum _focus_status {
		FOCUS_STAT_INACTIVE = 0,
		FOCUS_STAT_ACTIVE_SCAN,
		FOCUS_STAT_FOCUS_LOCKED,
		FOCUS_STAT_FOCUS_NOT_LOCKED,
		FOCUS_STAT_PASSIVE_SCAN,
		FOCUS_STAT_PASSIVE_LOCKED
	} focus_status;

	typedef enum _ctl_mode {
		CONTROL_NONE = 0,
		CONTROL_OFF,
		CONTROL_AUTO,
		CONTROL_USE_SCENE_MODE
	} ctl_mode;

	enum Sprd_camera_state {
		SPRD_INIT,
		SPRD_IDLE,
		SPRD_ERROR,
		SPRD_PREVIEW_IN_PROGRESS,
		SPRD_FOCUS_IN_PROGRESS,
		SPRD_WAITING_RAW,
		SPRD_WAITING_JPEG,
		/* internal states*/
		SPRD_INTERNAL_PREVIEW_STOPPING,
		SPRD_INTERNAL_CAPTURE_STOPPING,
		SPRD_INTERNAL_PREVIEW_REQUESTED,
		SPRD_INTERNAL_RAW_REQUESTED,
		SPRD_INTERNAL_STOPPING
	};

	enum state_owner {
		STATE_CAMERA,
		STATE_PREVIEW,
		STATE_CAPTURE,
		STATE_FOCUS
	};

	typedef struct _cam_size{
		uint32_t   width;
		uint32_t   height;
	} cam_size;

	typedef struct _camera_state	{
		Sprd_camera_state   camera_state;
		Sprd_camera_state   preview_state;
		Sprd_camera_state   capture_state;
		Sprd_camera_state   focus_state;
	} camera_state;

	typedef struct _camera_req_info {
		camera_metadata_t  *ori_req;
		int32_t            requestID;/*the flag of using at the same time*/
		int32_t            frmCnt;
		int32_t            aeCompensation;
		int32_t            outputStreamMask;
		uint32_t           cropRegion0;
		uint32_t           cropRegion1;
		uint32_t           cropRegion2;
		uint32_t           cropRegion3;
		double             gpsLat;
		double             gpsLon;
		double             gpsAlt;
		int64_t            gpsTimestamp;
		int64_t            sensorTimeStamp;
		cam_size           thumbnailJpgSize;
		uint8_t            gpsProcMethod[32];
		uint8_t            isReprocessing;
		int8_t             sceneMode;
		int8_t             afMode;
		int8_t             flashMode;
		int8_t             aeFlashMode;
		int8_t             awbMode;
		int8_t             isCropSet;
		capture_intent     captureIntent;/*android action*/
		ctl_mode           ctlMode;
		metadata_mode      metadataMode;
		ae_lock            aeLock;
		awb_lock           awbLock;
	} camera_req_info;

	typedef struct _cam_hal_ctl {
	ae_state           aeStatus;
	int                precaptureTrigID;
	int                afTrigID;
	int32_t            reProcMask;
	takepicture_mode   pictureMode;
	} cam_hal_ctl;

	static const int kPreviewBufferCount = 8;
	static const int kPreviewRotBufferCount = 4;
	static const int kRawBufferCount = 1;
	class RequestQueueThread : public SprdBaseThread {
		SprdCameraHWInterface2 *mHardware;
		public:
			RequestQueueThread(SprdCameraHWInterface2 *hw):
			SprdBaseThread(),
			mHardware(hw) { }
			~RequestQueueThread();

			void threadDealWithSiganl()
			{
				mHardware->RequestQueueThreadFunc(this);
				return;
			}

			void	release(void);
			bool	m_releasing;
	};

	class Stream : virtual public RefBase {
		SprdCameraHWInterface2  *mHardware;
		public:
			Stream(SprdCameraHWInterface2 *hw, uint8_t new_index):
			mHardware(hw),
			m_index(new_index),
			m_numRegisteredStream(1),
			m_IsRecevStopMsg(false),
			m_IsFirstFrm(false),
			m_halStopMsg(false) {
				memset(&m_parameters, 0, sizeof(stream_parameters_t));
				for (int i = 0 ; i < NUM_MAX_SUBSTREAM ; i++) {
					m_attachedSubStreams[i].streamId = -1;
					m_attachedSubStreams[i].priority = 0;
				}
			}
			~Stream();
			status_t  attachSubStream(int stream_id, int priority);
			status_t  detachSubStream(int stream_id);
			void      pushBufQ(int index);
			int       popBufQ();
			void      releaseBufQ();
			void      setRecevStopMsg(bool IsRecevMsg);
			bool      getRecevStopMsg();
			void      setHalStopMsg(bool IsStopMsg);
			bool      getHalStopMsg();

			uint8_t               m_index;
			/*private:*/
			stream_parameters_t   m_parameters;
			substream_entry_t     m_attachedSubStreams[NUM_MAX_SUBSTREAM];
			int                   m_numRegisteredStream;
			List<int>             m_bufQueue;
			Mutex                 m_BufQLock;
			Mutex                 m_stateLock;
			bool                  m_IsRecevStopMsg;
			bool                  m_IsFirstFrm;
			bool                  m_halStopMsg;/*oem/hal stop msg*/
	};

	int        allocatePreviewStream(uint32_t width, uint32_t height,
			int format, const camera2_stream_ops_t *stream_ops,
			uint32_t *stream_id, uint32_t *format_actual, uint32_t *usage, uint32_t *max_buffers);

	int        allocatePRVCBStream(uint32_t width, uint32_t height,
			int format, const camera2_stream_ops_t *stream_ops,
			uint32_t *stream_id, uint32_t *format_actual, uint32_t *usage, uint32_t *max_buffers);

	int        allocateJpegStream(uint32_t width, uint32_t height,
			int format, const camera2_stream_ops_t *stream_ops,
			uint32_t *stream_id, uint32_t *format_actual, uint32_t *usage, uint32_t *max_buffers);

	int 	   CameraPreviewReq(camera_req_info *srcreq,bool *IsSetPara);
	int 	   CameraCaptureReq(camera_req_info *srcreq,bool *IsSetPara);
	static int Callback_AllocCapturePmem(void* handle, unsigned int size, unsigned int *addr_phy, unsigned int *addr_vir);
	static int Callback_FreeCapturePmem(void* handle);
	bool       allocateCaptureMem(void);
	status_t   initCapMem(void);

	void       initStreamParam(int type, uint32_t width, uint32_t height, const camera2_stream_ops_t *stream_ops,
			uint32_t stream_id, uint32_t format_actual, uint32_t usage, int bufInHal, int unDeqBuf,int totalBuf = 4);

	void       enqeueMetaDataBufFrmHalToFramework(camera_metadata_t *metData);
	bool       prvFrmCbCheckPrvIsProc(camera_frame_type *frame);
	bool       streamDeqNewBuf(stream_parameters_t *stream, int *IndexRet);
	void       streamPushQAndReleaseFrm(stream_parameters_t *stream);
	int        ConstructProduceReq(camera_metadata_t **request,bool sizeRequest);
	void       freeCaptureMem();
	void       freePreviewMem(int num);
	bool       allocatePreviewMem();
	reprocess_buf_info   GetReprocessAcqBuf();
	void                 SetReprocessAcqBuf(reprocess_buf_info buf);
	int32_t              GetZslStreamMask();
	void                 SetZslStreamMask(int32_t mask);
	bool                 CheckAndroidZslSupport();
	void                 RequestQueueThreadFunc(SprdBaseThread * self);
	void                 SetReqProcessing(bool IsProc);
	bool                 GetReqProcessStatus();
	bool                 GetDcDircToDvSnap();
	void                 SetDcDircToDvSnap(bool dcDircToSnap);
	bool                 GetRecStopMsg();
	void                 SetRecStopMsg(bool recStop);
	uint8_t              GetReprocessingFlag();
	bool                 GetZslReprocStreamStatus();
	void                 SetZslReprocStreamStatus(bool ok);
	bool                 ProcZslCapReq(int32_t mask);
	int64_t              GetSensorTimeStamp(camera_req_info *reqInfo);
	void                 SetSensorTimeStamp(camera_req_info *reqInfo, int64_t timestamp);
	capture_intent       GetCameraCaptureIntent(camera_req_info *reqInfo);
	void                 SetCameraCaptureIntent(camera_req_info *reqInfo, capture_intent intent);
	void                 SetCameraZoomRect(cropZoom *crop);
	takepicture_mode     GetCameraPictureMode();
	void                 SetCameraPictureMode(takepicture_mode mode);
	int32_t              GetOutputStreamMask();
	void                 SetOutputStreamMask(int32_t mask);
	bool                 GetStartPreviewAftPic();
	void                 SetStartPreviewAftPic(bool IsPicPreview);
	bool                 GetIsOutputStream();/*for panoramic*/
	void                 SetIsOutputStream(bool OutputStrm);
	void                 Camera2GetSrvReqInfo( camera_req_info *srcreq, camera_metadata_t *orireq);
	void                 Camera2ProcessReq( camera_req_info *srcreq);
	int                  CameraConvertCropRegion(uint32_t sensorWidth, uint32_t sensorHeight, cropZoom *cropRegion);
	status_t             Camera2RefreshSrvReq(camera_req_info *srcreq, camera_metadata_t *dstreq);
	status_t             CamconstructDefaultRequest(SprdCamera2Info *camHal, int request_template,camera_metadata_t **request, bool sizeRequest);
	bool                 isSupportedJpegResolution(SprdCamera2Info *camHal, int width, int height);
	void                 DisplayPictureImg(camera_frame_type *frame);
	bool                 isSupportedResolution(SprdCamera2Info *camHal, int width, int height);
	int                  displaySubStream(sp<Stream> stream, int32_t *srcBufVirt, int64_t frameTimeStamp, uint16_t subStream);
	void                 receivePreviewFrame(camera_frame_type *frame);
	void                 receivePrevFrmWithCacheMem(camera_frame_type *frame);
	void                 HandleStartPreview(camera_cb_type cb, int32_t parm4);
	void                 HandleStartCamera(camera_cb_type cb, int32_t parm4);
	status_t             startPreviewInternal(bool isRecording);
	void                 stopPreviewInternal(bool isCancelAllBuf);
	void                 stopPreviewSimple(void);/*only call camera_stop_preview*/
	bool                 isPreviewing(void);
	bool                 isCapturing(void);
	bool                 WaitForCaptureDone(void);
	status_t             cancelPictureInternal(void);
	void                 deinitPreview(void);
	bool                 iSZslMode(void);
	void                 PushReqQ(camera_metadata_t *reqInfo);
	int                  GetReqQueueSize();
	camera_metadata_t   *PopReqQ();
	void                ClearReqQ();
	const char          *getCameraStateStr(Sprd_camera_state s);

	Sprd_camera_state   transitionState(Sprd_camera_state from,
			Sprd_camera_state to,
			state_owner owner,
			bool lock = true,bool IsBroadcast = false);

	void                setCameraState(Sprd_camera_state state,
				state_owner owner = STATE_CAMERA);

	inline Sprd_camera_state   getCameraState()
	{
		return mCameraState.camera_state;
	}

	inline Sprd_camera_state   getPreviewState()
	{
		return mCameraState.preview_state;
	}

	inline Sprd_camera_state   getCaptureState()
	{
		return mCameraState.capture_state;
	}

	inline int64_t   halAbs(int64_t value)
	{
		return value > 0 ? value : -value;
	}

	static void   camera_cb(camera_cb_type cb,
			const void *client_data,
			camera_func_type func,
			int32_t parm4);

	bool                  WaitForPreviewStart();
	bool                  WaitForCameraStop();
	bool                  WaitForCaptureStart();
	bool                  WaitForPreviewStop();
	void                  HandleStopCamera(camera_cb_type cb, int32_t parm4);
	void                  HandleStopPreview(camera_cb_type cb, int32_t parm4);
	void                  HandleCancelPicture(camera_cb_type cb,int32_t parm4);
	void                  HandleTakePicture(camera_cb_type cb,int32_t parm4);
	void                  HandleEncode(camera_cb_type cb,int32_t parm4);
	void                  HandleFocus(camera_cb_type cb, int32_t parm4);
	void                  getPreviewBuffer(void);
	int                   flush_preview_buffers();

	sp<RequestQueueThread>   m_RequestQueueThread;
	substream_parameters_t   m_subStreams[STREAM_ID_LAST+1];
	sp<Stream> m_Stream[STREAM_ID_LAST];/*parent stream*/
	SprdCamera2Info      *m_Camera2;
	camera2_request_queue_src_ops_t   *m_requestQueueOps;
	camera2_frame_queue_dst_ops_t     *m_frameQueueOps;
	camera2_notify_callback           m_notifyCb;
	void                              *m_callbackClient;
	camera2_device_t                  *m_halDevice;
	camera_req_info                   m_staticReqInfo;
	cam_hal_ctl                       m_camCtlInfo;
	focus_status                      m_focusStat;
	sprd_camera_memory_t              *mRawHeap;
	uint32_t                          mRawHeapSize;
	uint32_t                          mPreviewHeapNum;
	cam_size                          m_zslValidDataSize;
	bool                              m_IsNeedHalAllocPrvBuf;
	bool                              m_reqIsProcess;
	bool                              m_IsPrvAftPic;
	bool                              m_recStopMsg;
	bool                              m_zslEnqMetaFrm;
	bool                              m_zslReprocOk;
	bool                              m_degenerated_normal_cap;/*zsl degenerated to normal picture*/
	bool                              m_dcDircToDvSnap;/*for cts testVideoSnapshot*/
	bool                              mIsOutPutStream;
	camera_metadata_t                 *m_halRefreshReq;
	static gralloc_module_t const*    m_grallocHal;
	uint32_t                          mPreviewFrmRefreshIndex;
	int64_t                           mPreviewFrmTimestamp[kPreviewBufferCount];
	int32_t                           mPreviewHeapArray_phy[kPreviewBufferCount+kPreviewRotBufferCount+1];
	int32_t                           mPreviewHeapArray_vir[kPreviewBufferCount+kPreviewRotBufferCount+1];
	sprd_camera_memory_t              *mPreviewHeapArray[kPreviewBufferCount+kPreviewRotBufferCount+1];
	sprd_camera_memory_t              mMiscHeapArray[MAX_MISCHEAP_NUM];
	uint32_t                          mMiscHeapNum;
	uint32_t                          m_CapFrmCnt; /*for zsl*/
	uint32_t                          m_PrvFrmCnt;/*for zsl*/
	int                               m_CameraId;
	int                               m_reprocessStreamId;
	const camera2_stream_in_ops_t     *m_reprocessOps;
	reprocess_buf_info                m_reprocessBuf;
	int                               m_reprocessOutputStreamId;
	List<camera_metadata_t *>         m_ReqQueue;
	Mutex                             m_requestMutex;
	Mutex                             m_halCBMutex;//for vary
	Mutex                             m_capBufMutex;
	Mutex                             m_stopPrvFrmCBMutex;//mutex for stop preview and receivepreviewframe
	Mutex                             m_metaDataMutex;
	Mutex                             mStateLock;
	mutable Mutex                     m_afTrigLock;
	Condition                         mStateWait;
	volatile camera_state             mCameraState;

};

}; // namespace android

#endif
