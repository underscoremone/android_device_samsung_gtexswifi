/*
* hardware/sprd/hsdroid/libcamera/sprdcamerahardwarestub.cpp
 * Camera HAL
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

#define LOG_TAG "SprdCameraHardwareStub"
#include <utils/Log.h>

#include "SprdCameraHardwareStub.h"
#include <utils/threads.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <binder/MemoryHeapBase.h>
#include <binder/MemoryHeapPmem.h>
#include <linux/android_pmem.h>

#include <SkStream.h>
#include <SkImageDecoder.h>
#include <SkBitmap.h>
#include <cutils/properties.h>
extern "C" {
	#include <jhead.h>
}

#include "scale_sc8800g2.h"
#include "rotation_sc8800g2.h"

#define PMEM_DEV "/dev/pmem_adsp"
uint32_t g_buffer_size = 0;
uint32_t g_is_first_time = 1; // 1: need to decode jpeg picture; 0: not need.

namespace android {

SprdCameraHardwareStub::SprdCameraHardwareStub()
                  : mParameters(),
                    mPreviewHeap(0),
                    mRawHeap(0),
                    //mFakeCamera(0),
                    mPreviewFrameSize(0),
                    mNotifyCb(0),
                    mDataCb(0),
                    mDataCbTimestamp(0),
                    mCallbackCookie(0),
                    mMsgEnabled(0),
                    mCurrentPreviewFrame(0)
{
    initDefaultParameters();
}

void SprdCameraHardwareStub::initDefaultParameters()
{
    CameraParameters p;

    p.set("preview-size-values","176x144");
    p.setPreviewSize(176, 144);
    p.setPreviewFrameRate(15);
    p.setPreviewFormat("yuv420sp");

    p.set("picture-size-values", "320x240");
    p.setPictureSize(320, 240);
    p.setPictureFormat("jpeg");

    if (setParameters(p) != NO_ERROR) {
        LOGE("Failed to set default parameters?!");
    }
}

void SprdCameraHardwareStub::initHeapLocked()
{
   int page_size = getpagesize();   
   int buffer_size, mem_size;
   sp<MemoryHeapBase> masterHeap;
   
    // Create raw heap.
    int picture_width, picture_height;
    mParameters.getPictureSize(&picture_width, &picture_height);
    LOGD("initHeapLocked: picture size=%dx%d", picture_width, picture_height);
   // buffer_size = (picture_width * picture_height * 3 / 2  + page_size - 1) & ~(page_size - 1);
    mem_size =  picture_width * picture_height * 3 / 2;
    masterHeap = new MemoryHeapBase(PMEM_DEV,mem_size,MemoryHeapBase::NO_CACHING);
     mRawHeap = new MemoryHeapPmem(masterHeap,MemoryHeapBase::NO_CACHING);
        if (mRawHeap->getHeapID() >= 0) {
            mRawHeap->slap();
            masterHeap.clear();           
        }
        else LOGE("Camera Stub raw heap  error: could not create master heap!");
}

SprdCameraHardwareStub::~SprdCameraHardwareStub()
{
    LOGD("~SprdCameraHardwareStub E.");
//    delete mFakeCamera;
//    mFakeCamera = 0; // paranoia
    singleton.clear();
}

sp<IMemoryHeap> SprdCameraHardwareStub::getPreviewHeap() const
{
	//return mPreviewHeap != NULL ? mPreviewHeap->getHeap() : NULL;
    return mPreviewHeap;
}

sp<IMemoryHeap> SprdCameraHardwareStub::getRawHeap() const
{
	//return mRawHeap != NULL ? mRawHeap->getHeap() : NULL;
    return mRawHeap;
}

void SprdCameraHardwareStub::setCallbacks(notify_callback notify_cb,
                                      data_callback data_cb,
                                      data_callback_timestamp data_cb_timestamp,
                                      void* user)
{
    Mutex::Autolock lock(mLock);
    mNotifyCb = notify_cb;
    mDataCb = data_cb;
    mDataCbTimestamp = data_cb_timestamp;
    mCallbackCookie = user;
}

void SprdCameraHardwareStub::enableMsgType(int32_t msgType)
{
    Mutex::Autolock lock(mLock);
    mMsgEnabled |= msgType;
}

void SprdCameraHardwareStub::disableMsgType(int32_t msgType)
{
    Mutex::Autolock lock(mLock);
    mMsgEnabled &= ~msgType;
}

bool SprdCameraHardwareStub::msgTypeEnabled(int32_t msgType)
{
    Mutex::Autolock lock(mLock);
    return (mMsgEnabled & msgType);
}

// ---------------------------------------------------------------------------
static int xioctl(int fd, int request, void * arg) {   
    int r;   
    //do   
        r = ioctl(fd, request, arg);   
    //while (-1 == r && EINTR == errno);   
    return r;   
} 
static int preview_scale_colorformat(SCALE_DATA_FORMAT_E output_fmt, uint32_t output_width, uint32_t output_height, uint32_t output_addr, SCALE_DATA_FORMAT_E input_fmt,uint32_t input_width, uint32_t input_height, uint32_t input_addr)
{
	static int fd = -1; 	
	SCALE_CONFIG_T scale_config;
	SCALE_SIZE_T scale_size;
	SCALE_RECT_T scale_rect;
	SCALE_ADDRESS_T scale_address;
	SCALE_DATA_FORMAT_E data_format;
	//uint32_t sub_sample_en;
	SCALE_MODE_E scale_mode;
	uint32_t mode;
	
	fd = open("/dev/sc8800g_scale", O_RDONLY);//O_RDWR /* required */, 0);  
	if (-1 == fd) 
	{   
		LOGE("Fail to open scale device.");
        	return -1;   
   	 }
    	
	//set mode
	scale_config.id = SCALE_PATH_MODE;	
	scale_mode = SCALE_MODE_SCALE;
	scale_config.param = &scale_mode;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set input data format
	scale_config.id = SCALE_PATH_INPUT_FORMAT;
	data_format = input_fmt;
	scale_config.param = &data_format;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set output data format
	scale_config.id = SCALE_PATH_OUTPUT_FORMAT;
	data_format = output_fmt;
	scale_config.param = &data_format;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set input size
	scale_config.id = SCALE_PATH_INPUT_SIZE;
	scale_size.w = input_width;
	scale_size.h = input_height;
	scale_config.param = &scale_size;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set output size
	scale_config.id = SCALE_PATH_OUTPUT_SIZE;
	scale_size.w = output_width;
	scale_size.h = output_height;
	scale_config.param = &scale_size;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}	
	//set input size
	scale_config.id = SCALE_PATH_INPUT_RECT;
	scale_rect.x = 0;
	scale_rect.y = 0;
	scale_rect.w = input_width;
	scale_rect.h = input_height;
	scale_config.param = &scale_rect;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	if(((input_width > output_width * 4) &&
                (input_width <= output_width * 8)) ||
                ((input_height > output_height * 4) &&
                (input_height <= output_height * 8)))
        {
                //set sub sample mode
                uint32_t mode = 0; //0: 1/2 1:1/4 2:1/8 3:1/16
                uint32_t enable = 1;
                scale_config.id = SCALE_PATH_SUB_SAMPLE_EN;
                scale_config.param = &enable;
		if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
                //if(0 != _SCALE_DriverIOPathConfig(scale_config.id, scale_config.param))
                {
                        //SCALE_PRINT("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
			LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
                        return -1;
                }
                scale_config.id = SCALE_PATH_SUB_SAMPLE_MOD;
                scale_config.param = &mode;
		if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))
                //if(0 != _SCALE_DriverIOPathConfig(scale_config.id, scale_config.param))
                {
                        //SCALE_PRINT("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
			LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
                        return -1;
                }
        }
	//set input address
	scale_config.id = SCALE_PATH_INPUT_ADDR;
	scale_address.yaddr = input_addr; 
	scale_address.uaddr = input_addr + input_width * input_height;
	scale_address.vaddr = scale_address.uaddr;
	scale_config.param = &scale_address;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set output address
	scale_config.id = SCALE_PATH_OUTPUT_ADDR;
	scale_address.yaddr = output_addr; 
	scale_address.uaddr = output_addr + output_width * output_height;
	scale_address.vaddr = output_addr + output_width * output_height * 3 / 2;	
	scale_config.param = &scale_address;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
	//set output endian
	scale_config.id = SCALE_PATH_OUTPUT_ENDIAN;
	mode = 1;
	scale_config.param = &mode;	 
	if (-1 == xioctl(fd, SCALE_IOC_CONFIG, &scale_config))   
	{
		LOGE("Fail to SCALE_IOC_CONFIG: id=%d", scale_config.id);
		return -1;
	}
		
	//done	 
	if (-1 == xioctl(fd, SCALE_IOC_DONE, 0))   
	{
		LOGE("Fail to SCALE_IOC_DONE");
		return -1;
	}

	if(-1 == close(fd))   
	{   
		LOGE("Fail to close scale device.");
        	return -1;   
   	 } 
    	fd = -1;   

	return 0;
}

void SprdCameraHardwareStub::release_frame(void){
	mCurrentPreviewFrame = (mCurrentPreviewFrame + 1) % kBufferCount;
}

static int convert_endian_by_DMA(uint32_t width, uint32_t height, uint32_t out_address,  uint32_t in_address)
{
        SCALE_YUV420_ENDIAN_T yuv_config;
        int fd = -1;

        fd = open("/dev/sc8800g_scale", O_RDONLY);
        if (-1 == fd)
        {
                LOGE("Fail to open scale device for preview DMA.");
                return -1;
         }
        //else
        //      LOGV("OK to open scale device for preview DMA.");

        yuv_config.width = width;
        yuv_config.height = height;
        yuv_config.src_addr = in_address;
        yuv_config.dst_addr = out_address;

        if (-1 == xioctl(fd, SCALE_IOC_YUV420_ENDIAN, &yuv_config))
        {
                LOGE("Fail to SCALE_IOC_YUV420_ENDIAN.");
                return -1;
        }

        if(-1 == close(fd))
        {
                LOGE("Fail to close scale device for preview DMA.");
                return -1;
         }
        fd = -1;

        return 0;
}
static int loadExifInfo(const char* FileName, int readJPG) {
    int Modified = false;
    ReadMode_t ReadMode = READ_METADATA;
   /* if (readJPG) {
        // Must add READ_IMAGE else we can't write the JPG back out.
        ReadMode |= READ_IMAGE;
    }*/

    ResetJpgfile();

    // Start with an empty image information structure.
    memset(&ImageInfo, 0, sizeof(ImageInfo));
    ImageInfo.FlashUsed = -1;
    ImageInfo.MeteringMode = -1;
    ImageInfo.Whitebalance = -1;

    // Store file date/time.
    /*{
        struct stat st;
        if (stat(FileName, &st) >= 0) {
            ImageInfo.FileDateTime = st.st_mtime;
            ImageInfo.FileSize = st.st_size;
        }
    }*/

    strncpy(ImageInfo.FileName, FileName, PATH_MAX);

    return ReadJpegFile(FileName, ReadMode);
}

static int getOrientationAttributes(const char *filename)
{
    int result = 0;
    loadExifInfo(filename, false);  
    result = ImageInfo.Orientation;    
    DiscardData();
	
    return result;
}

static int stub_camera_rotation(uint32_t agree, uint32_t width, uint32_t height, uint32_t in_addr, uint32_t out_addr)
{
        int fd = -1;
        ROTATION_PARAM_T rot_params;

        rot_params.data_format = ROTATION_RGB565;
        switch(agree){
                case 90:
                        rot_params.rotation_dir = ROTATION_90;
                        break;
                case 180:
                        rot_params.rotation_dir = ROTATION_180;
                        break;
                case 270:
                        rot_params.rotation_dir = ROTATION_270;
                        break;
                default:
                        rot_params.rotation_dir = ROTATION_DIR_MAX;
                        break;
        }
        rot_params.img_size.w = width;
        rot_params.img_size.h = height;
        rot_params.src_addr.y_addr = in_addr;
        rot_params.src_addr.uv_addr = rot_params.src_addr.y_addr + rot_params.img_size.w * rot_params.img_size.h;
        rot_params.src_addr.v_addr = rot_params.src_addr.uv_addr;
        rot_params.dst_addr.y_addr = out_addr;
        rot_params.dst_addr.uv_addr = rot_params.dst_addr.y_addr + rot_params.img_size.w * rot_params.img_size.h;
        rot_params.dst_addr.v_addr = rot_params.dst_addr.uv_addr;



        fd = open("/dev/sc8800g_rotation", O_RDWR /* required */, 0);
        if (-1 == fd)
        {
                LOGE("Fail to open rotation device.");
                return -1;
        }
    //  else
    //                  LOGV("OK to open rotation device.");

        //done   
        if (-1 == xioctl(fd, SC8800G_ROTATION_DONE, &rot_params))
        {
                LOGE("Fail to SC8800G_ROTATION_DONE");
                return -1;
        }
        //else
        //      LOGE("OK to SC8800G_ROTATION_DONE.");

        if(-1 == close(fd))
        {
                LOGE("Fail to close rotation device.");
                return -1;
         }
        fd = -1;

        return 0;
}

int SprdCameraHardwareStub::previewThread()
{
	if(1 == g_is_first_time){
    mLock.lock();
        // the attributes below can change under our feet...

	sp<MemoryHeapBase> masterHeap;
	sp<MemoryHeapPmem>  SwapHeap;
	int phys_addr=0;
	uint8_t *virt_addr = NULL;
	int width=0, height=0;
	uint8_t *base = (uint8_t *)mPreviewHeap->getBase();

	//decode the picture and copy picture data decoded to swap buffer
        SkBitmap bp;
        SkImageDecoder::Format fmt;	
	//property_set("hw.camera.picture", "/data/test.jpg");	
	char propBuf[PROPERTY_VALUE_MAX];  
        property_get("gsm.camera.picture", propBuf, "");	
	LOGD("property_get: %s.", propBuf);
	uint32_t orientation = getOrientationAttributes(propBuf);
        uint32_t degree = 0;
        LOGI("wxz:stub camera : orientation: %d.", orientation);
        if(0 != orientation){
                switch(orientation){
                        case 3:
                                degree = 180;
                                break;
                        case 6:
                                degree = 90;
                                break;
                        case 8:
                                degree = 270;
                                break;
                }
        }
        //bool result = SkImageDecoder::DecodeFile("/data/test.jpg", &bp,SkBitmap::kRGB_565_Config, SkImageDecoder::kDecodePixels_Mode, &fmt);
        bool result = SkImageDecoder::DecodeFile(propBuf, &bp,SkBitmap::kRGB_565_Config, SkImageDecoder::kDecodePixels_Mode, &fmt);
       if(!result){
		LOGI("decoder file fail!");
    	}else{
    		if(fmt!= SkImageDecoder::kJPEG_Format){
			LOGI("decoder file not jpeg!");
    		}else{
    			LOGI("width %d,height %d,rowBytesAsPixels %d,config %d,bytesPerPixel %d",bp.width(),bp.height(),bp.rowBytesAsPixels(),bp.config(),bp.bytesPerPixel());
/*{
	FILE *f_yuv=fopen("/data/stub_rgb565.raw","wb");
	short *pixl_addr = (short *) bp.getPixels();
	fwrite(pixl_addr, 1, bp.height() * bp.rowBytesAsPixels() * bp.bytesPerPixel(), f_yuv);
	fclose(f_yuv);
	LOGD("stub_rgb565: size: %d", bp.height() * bp.rowBytesAsPixels() * bp.bytesPerPixel());
}*/				
			width = bp.width() / 4 * 4; //wxz20111207: the scale need width aligned by 4 pixel.
			if(0 != degree){
				height = bp.height() / 4 * 4; //the rotation need height aligned by 4 pixel.
			}
			else{
				height = bp.height();
			}
			int mem_size = width * height * bp.bytesPerPixel() * 2;	
			masterHeap = new MemoryHeapBase(PMEM_DEV,mem_size,MemoryHeapBase::NO_CACHING);
        		SwapHeap = new MemoryHeapPmem(masterHeap,MemoryHeapBase::NO_CACHING);
        		if (SwapHeap->getHeapID() >= 0) {
            			SwapHeap->slap();
            			masterHeap.clear();  
	   			struct pmem_region region;
           			int fd_pmem = 0;
          			fd_pmem = SwapHeap->getHeapID();	
	  			::ioctl(fd_pmem,PMEM_GET_PHYS,&region);
	  			phys_addr = region.offset;
	  			virt_addr = (uint8_t *)SwapHeap->getBase();
				short *pixl = (short *) bp.getPixels();
				for(int j=0; j<height; j++){					
					memcpy(virt_addr+j * width * bp.bytesPerPixel(), pixl, width * bp.bytesPerPixel());
					pixl += bp.rowBytesAsPixels();
				}
        		}
        		else LOGE("Camera Stub preview heap  error: could not create master heap!");
    		}
   	 }


       if(0 != degree){
              stub_camera_rotation(degree, width, height, phys_addr, phys_addr + bp.width() * bp.height() * bp.bytesPerPixel());
            	if((90 == degree) || (270 == degree)){
			uint32_t tmp = width;
			width = height;
			height = tmp;
		}
       }      
	
	//scaling the swap buffer to preview buffer
	int preview_width, preview_height;
	mParameters.getPreviewSize(&preview_width, &preview_height);	
	if(0 == degree){
		preview_scale_colorformat(SCALE_DATA_YUV420, preview_width, preview_height, mBuffersPhys[0], SCALE_DATA_RGB565, width, height, phys_addr);
	}
	else{
		preview_scale_colorformat(SCALE_DATA_YUV420, preview_width, preview_height, mBuffersPhys[0], SCALE_DATA_RGB565, width, height, phys_addr + bp.width() * bp.height() * bp.bytesPerPixel());
	}
	convert_endian_by_DMA(preview_width, preview_height,mBuffersPhys[0], mBuffersPhys[0]);
	SwapHeap.clear(); //wxz20110815: free the SwapHeap.
/*{
	FILE *f_yuv=fopen("/data/stub_rgb565_swap.raw","wb");
	fwrite(virt_addr, 1, width * height * bp.bytesPerPixel(), f_yuv);
	fclose(f_yuv);
	LOGD("stub_rgb565_swap: size: %d", width * height * bp.bytesPerPixel());
}	
{
	FILE *f_yuv=fopen("/data/stub_yuv420.yuv","wb");
	fwrite(base, 1, mPreviewFrameSize, f_yuv);
	fclose(f_yuv);
	LOGD("stub_yuv420: mPreviewFrameSize: %d", mPreviewFrameSize);
}*/	
	//copy data from the first preview buffer to the second.
	//memcpy(base + g_buffer_size, base, g_buffer_size);
	g_is_first_time = 0;
    mLock.unlock();    
	}//if(1 == g_is_first_time)

    {    	
        // Calculate how long to wait between frames.
        int previewFrameRate = mParameters.getPreviewFrameRate();
        int delay = (int)(1000000.0f / float(previewFrameRate));		
	ssize_t offset;
	size_t size;
	mBuffers[mCurrentPreviewFrame]->getMemory(&offset, &size);
	LOGD("camera stub Preview buffer %d: offset: %d, size: %d, mBuffersPhys: %x.",mCurrentPreviewFrame, (uint32_t)offset, (uint32_t)size, (uint32_t)mBuffersPhys[mCurrentPreviewFrame] );     
       // Notify the client of a new frame.
	if (mMsgEnabled & CAMERA_MSG_PREVIEW_FRAME)		
	    	mDataCb(CAMERA_MSG_PREVIEW_FRAME, mBuffers[mCurrentPreviewFrame], mCallbackCookie);   
       if (mMsgEnabled & CAMERA_MSG_VIDEO_FRAME)		
       {
           nsecs_t timestamp = systemTime(); 
	   mDataCbTimestamp(timestamp, CAMERA_MSG_VIDEO_FRAME, mBuffers[mCurrentPreviewFrame], mCallbackCookie);	
       }
	else
		release_frame();            
        //if (mMsgEnabled & CAMERA_MSG_PREVIEW_FRAME)
       //     mDataCb(CAMERA_MSG_PREVIEW_FRAME, mBuffers[mCurrentPreviewFrame], mCallbackCookie);            
          
        // Advance the buffer pointer.
        //mCurrentPreviewFrame = (mCurrentPreviewFrame + 1) % kBufferCount;

        // Wait for it...
        usleep(delay);
    }

    return NO_ERROR;
}

void SprdCameraHardwareStub::initPreview()
{
   int page_size = getpagesize();   
   int buffer_size, mem_size;
   sp<MemoryHeapBase> masterHeap;
   
   //create preview heap.
    int preview_width, preview_height;
    mParameters.getPreviewSize(&preview_width, &preview_height);
    LOGD("initHeapLocked: preview size=%dx%d", preview_width, preview_height);
   //mPreviewFrameSize = preview_width * preview_height * 3 / 2;
   //g_buffer_size = (mPreviewFrameSize + page_size - 1) & ~(page_size - 1);
   //g_buffer_size = (preview_width * preview_height * 2 + page_size - 1) & ~(page_size - 1);
   g_buffer_size = preview_width * preview_height * 3 / 2;
   mem_size = kBufferCount  * g_buffer_size;	
    masterHeap = new MemoryHeapBase(PMEM_DEV,mem_size,MemoryHeapBase::NO_CACHING);
     mPreviewHeap = new MemoryHeapPmem(masterHeap,MemoryHeapBase::NO_CACHING);
        if (mPreviewHeap->getHeapID() >= 0) {
            mPreviewHeap->slap();
            masterHeap.clear();  
	   struct pmem_region region;
           int fd_pmem = 0;
          fd_pmem = mPreviewHeap->getHeapID();	
	  ::ioctl(fd_pmem,PMEM_GET_PHYS,&region);
	  //mBuffers = new sp<MemoryBase>[kBufferCount];
	  for(int i = 0; i  < kBufferCount; i++){
	  	mBuffersPhys[i] = region.offset + i * g_buffer_size;	
                mBuffers[i] = new MemoryBase(mPreviewHeap,  i * g_buffer_size, g_buffer_size);
		ssize_t offset;
		size_t size;
		mBuffers[i]->getMemory(&offset, &size);
		LOGD("Preview buffer %d: offset: %d, size: %d, mBuffersPhys: %x.", i, (uint32_t)offset, (uint32_t)size, (uint32_t)mBuffersPhys[i] );
	  }	
        }
        else LOGE("Camera Stub preview heap  error: could not create master heap!");
}	

status_t SprdCameraHardwareStub::startPreview()
{
    LOGD("startPreview E.");
    Mutex::Autolock lock(mLock);
    initPreview();
    if (mPreviewThread != 0) {
        // already running
        return INVALID_OPERATION;
    }
    g_is_first_time = 1; 
    mPreviewThread = new PreviewThread(this);
    return NO_ERROR;
}

void SprdCameraHardwareStub::stopPreview()
{
    LOGD("stopPreview E.");
    sp<PreviewThread> previewThread;

    { // scope for the lock
        Mutex::Autolock lock(mLock);
        previewThread = mPreviewThread;
    }

    // don't hold the lock while waiting for the thread to quit
    if (previewThread != 0) {
		
        previewThread->requestExitAndWait();
    }
    mPreviewHeap = NULL;

    Mutex::Autolock lock(mLock);
    mPreviewThread.clear();

    LOGD("stopPreview X.");
}

bool SprdCameraHardwareStub::previewEnabled() {
    return mPreviewThread != 0;
}

status_t SprdCameraHardwareStub::startRecording()
{
    return startPreview();
}

void SprdCameraHardwareStub::stopRecording()
{
	return stopPreview();
}

bool SprdCameraHardwareStub::recordingEnabled()
{
    return mPreviewThread != 0;
}

void SprdCameraHardwareStub::releaseRecordingFrame(const sp<IMemory>& mem)
{
	release_frame();
}

// ---------------------------------------------------------------------------

int SprdCameraHardwareStub::beginAutoFocusThread(void *cookie)
{
    SprdCameraHardwareStub *c = (SprdCameraHardwareStub *)cookie;
    return c->autoFocusThread();
}

int SprdCameraHardwareStub::autoFocusThread()
{
    if (mMsgEnabled & CAMERA_MSG_FOCUS)
        mNotifyCb(CAMERA_MSG_FOCUS, true, 0, mCallbackCookie);
    return NO_ERROR;
}

status_t SprdCameraHardwareStub::autoFocus()
{
    Mutex::Autolock lock(mLock);
    if (createThread(beginAutoFocusThread, this) == false)
        return UNKNOWN_ERROR;
    return NO_ERROR;
}

status_t SprdCameraHardwareStub::cancelAutoFocus()
{
    return NO_ERROR;
}

/*static*/ int SprdCameraHardwareStub::beginPictureThread(void *cookie)
{
    SprdCameraHardwareStub *c = (SprdCameraHardwareStub *)cookie;
    return c->pictureThread();
}

int SprdCameraHardwareStub::pictureThread()
{
#if 0
    if (mMsgEnabled & CAMERA_MSG_SHUTTER)
        mNotifyCb(CAMERA_MSG_SHUTTER, 0, 0, mCallbackCookie);

    if (mMsgEnabled & CAMERA_MSG_RAW_IMAGE) {
        //FIXME: use a canned YUV image!
        // In the meantime just make another fake camera picture.
        int w, h;
        mParameters.getPictureSize(&w, &h);
        sp<MemoryBase> mem = new MemoryBase(mRawHeap, 0, w * 2 * h);
        FakeCamera cam(w, h);
        cam.getNextFrameAsYuv422((uint8_t *)mRawHeap->base());
        mDataCb(CAMERA_MSG_RAW_IMAGE, mem, mCallbackCookie);
    }

    if (mMsgEnabled & CAMERA_MSG_COMPRESSED_IMAGE) {
        sp<MemoryHeapBase> heap = new MemoryHeapBase(kCannedJpegSize);
        sp<MemoryBase> mem = new MemoryBase(heap, 0, kCannedJpegSize);
        memcpy(heap->base(), kCannedJpeg, kCannedJpegSize);
        mDataCb(CAMERA_MSG_COMPRESSED_IMAGE, mem, mCallbackCookie);
    }
#endif	
    return NO_ERROR;
}

status_t SprdCameraHardwareStub::takePicture()
{
    LOGD("takePicture E.");
    stopPreview();
    if (createThread(beginPictureThread, this) == false)
        return -1;
    return NO_ERROR;
}

status_t SprdCameraHardwareStub::cancelPicture()
{
    return NO_ERROR;
}

status_t SprdCameraHardwareStub::dump(int fd, const Vector<String16>& args) const
{
#if 0
    const size_t SIZE = 256;
    char buffer[SIZE];
    String8 result;
    AutoMutex lock(&mLock);
    if (mFakeCamera != 0) {
        mFakeCamera->dump(fd);
        mParameters.dump(fd, args);
        snprintf(buffer, 255, " preview frame(%d), size (%d), running(%s)\n", mCurrentPreviewFrame, mPreviewFrameSize, mPreviewRunning?"true": "false");
        result.append(buffer);
    } else {
        result.append("No camera client yet.\n");
    }
    write(fd, result.string(), result.size());
#endif	
    return NO_ERROR;
}

status_t SprdCameraHardwareStub::setParameters(const CameraParameters& params)
{
    LOGD("setParameters E.");
    Mutex::Autolock lock(mLock);
    // XXX verify params

    if (strcmp(params.getPreviewFormat(), "yuv420sp") != 0) {
        LOGE("Only yuv420sp preview is supported");
        return -1;
    }

    if (strcmp(params.getPictureFormat(), "jpeg") != 0) {
        LOGE("Only jpeg still pictures are supported");
        return -1;
    }

    /*int w, h;
    params.getPictureSize(&w, &h);
    if (w != kCannedJpegWidth && h != kCannedJpegHeight) {
        LOGE("Still picture size must be size of canned JPEG (%dx%d)",
             kCannedJpegWidth, kCannedJpegHeight);
        return -1;
    }*/

    mParameters = params;
    initHeapLocked();

    return NO_ERROR;
}

CameraParameters SprdCameraHardwareStub::getParameters() const
{
    Mutex::Autolock lock(mLock);
    return mParameters;
}

status_t SprdCameraHardwareStub::sendCommand(int32_t command, int32_t arg1,
                                         int32_t arg2)
{
    return BAD_VALUE;
}

void SprdCameraHardwareStub::release()
{
    LOGD("release E.");
}

wp<CameraHardwareInterface> SprdCameraHardwareStub::singleton;

sp<CameraHardwareInterface> SprdCameraHardwareStub::createInstance()
{
    LOGD("createInstance E.");
    if (singleton != 0) {
        sp<CameraHardwareInterface> hardware = singleton.promote();
        if (hardware != 0) {
            return hardware;
        }
    }
    LOGD("createInstance singleton = 0.");
    sp<CameraHardwareInterface> hardware(new SprdCameraHardwareStub());
    singleton = hardware;
    return hardware;
}
}; // namespace android
