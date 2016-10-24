/******************************************************************************
 ** File Name:      jpegenc_api.c                                            *
 ** Author:         xiaozhe.wang													  *
 ** DATE:           02/15/2011                                                *
 ** Copyright:      2011 Spreadtrum, Incoporated. All Rights Reserved.        *
 ** Description:    The interface functions for jpeg encoder.									  *
 ** Note:           None                                                      *
******************************************************************************/
/******************************************************************************
 **                        Edit History                                       *
 ** ------------------------------------------------------------------------- *
 ** DATE           NAME             DESCRIPTION                               *
 ** 02/15/2011     xiaozhe.wang	         Create.                                  *
******************************************************************************/
/*----------------------------------------------------------------------------*
**                        Dependencies                                        *
**---------------------------------------------------------------------------*/
//#include "sc8810_video_header.h"

/**---------------------------------------------------------------------------*
**                        Compiler Flag                                       *
**---------------------------------------------------------------------------*/
#ifdef   __cplusplus
    extern   "C" 
    {
#endif
		
#if defined(JPEG_ENC)

#include "jpeg_exif_header.h"

//////////////////////////////////////////////////////////////////////////
typedef void (*jpegenc_callback)(uint32_t buf_id, uint32_t stream_size, uint32_t is_last_slice);

typedef enum
{
	JPEGENC_YUV_420 = 0,
	JPEGENC_YUV_422,
	JPEGENC_YUV_MAX	
}JPEGENC_FORMAT_E;
typedef enum
{
	JPEGENC_QUALITY_LOW = 0,				//bad
	JPEGENC_QUALITY_MIDDLE_LOW,			//poor
	JPEGENC_QUALITY_MIDDLE,				//good
	JPEGENC_QUALITY_MIDDLE_HIGH,			//excellent
	JPEGENC_QUALITY_HIGH,					//oustanding
	JPEGENC_QUALITY_MAX
}JPEGENC_QUALITY_E;
typedef struct fraction_t
{
	uint32_t numerator;
	uint32_t denominator;
}FRACTION_T;
typedef struct jpegenc_params
{
	JPEGENC_FORMAT_E format;
	uint32_t width;
	uint32_t height;
	uint32_t set_slice_height;
	void *yuv_virt_buf;
	uint32_t yuv_phy_buf;
	void *stream_virt_buf[2];
	uint32_t stream_phy_buf[2];	
	uint32_t stream_buf_len;
	uint32_t stream_size;
	JPEGENC_QUALITY_E quality;
	jpegenc_callback read_callback;
    //exif information 
 	uint32_t thumb_width;
	uint32_t thumb_height;
	uint32_t thumb_quality;
	void *thumb_src_yuv_virt_buf;
	uint32_t thumb_src_yuv_phy_buf;
	FRACTION_T Latitude_dd;
	FRACTION_T Latitude_mm;
	FRACTION_T Latitude_ss;
	uint32_t Latitude_ref; //0: North latitude, 1: South latitude	
	FRACTION_T Longitude_dd;
	FRACTION_T Longitude_mm;
	FRACTION_T Longitude_ss;
	uint32_t Longitude_ref; //0: East Longitude, 1: West Longitude	
	const char *image_description;
	const char *make;
	const char *model;
	const char *copyright;
	uint32_t orientation;
	const char *datetime;	
	const char *gps_date;	
	const char *gps_process_method;
	uint32_t gps_hour;	
	uint32_t gps_minuter;
	uint32_t gps_second;	
	FRACTION_T focal_length;
	
	JINF_EXIF_INFO_T *dc_exif_info_ptr;
}JPEGENC_PARAMS_T;

uint32_t JPEGENC_encode_one_pic(JPEGENC_PARAMS_T *jpegenc_params, jpegenc_callback callback);

//////////////////////////////////////////////////////////////////////////
#endif //JPEG_ENC
/**---------------------------------------------------------------------------*
**                         Compiler Flag                                      *
**---------------------------------------------------------------------------*/
#ifdef   __cplusplus
    }
#endif
/**---------------------------------------------------------------------------*/
// End 
