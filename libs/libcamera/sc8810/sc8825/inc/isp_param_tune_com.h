#ifndef _ISP_PARAM_TUNE_COM_H_
#define _ISP_PARAM_TUNE_COM_H_
/*----------------------------------------------------------------------------*
 **                          Dependencies                                     *
 **---------------------------------------------------------------------------*/
#include <sys/types.h>
//#include "isp_com.h"
#include "sensor_drv_u.h"
#include "sensor_raw.h"

/**---------------------------------------------------------------------------*
 **                          Compiler Flag                                    *
 **---------------------------------------------------------------------------*/
#ifdef   __cplusplus
extern   "C"
{
#endif

/**---------------------------------------------------------------------------*
**                               Micro Define                                **
**----------------------------------------------------------------------------*/
typedef int32_t (*isp_fun)(void* param_ptr);

#define ISP_AE_TAB_NUM 0x04
#define ISP_CMC_NUM 0x09
#define ISP_AWB_NUM 0x09
#define ISP_MAP_NUM 0x08

#define ISP_END_ID 0xffff
#define ISP_VERSION_0000_ID 0x0000

#define ISP_PACKET_VERIFY_ID 0x71717567
#define ISP_PACKET_END_ID 0x69656e64

//parser cmd
#define ISP_PARSER_DOWN 0x0000
#define ISP_PARSER_UP_PARAM   0x0001
#define ISP_PARSER_UP_PRV_DATA 0x0003
#define ISP_PARSER_UP_CAP_DATA 0x0004
#define ISP_PARSER_UP_CAP_SIZE 0x0005
#define ISP_PARSER_UP_MAIN_INFO 0x0006
#define ISP_PARSER_UP_SENSOR_REG 0x0007

//packet type
#define ISP_TYPE_CMD   0x0000
#define ISP_TYPE_PARAM 0x0001
#define ISP_TYPE_LEVEL 0x0002
#define ISP_TYPE_PRV_DATA  0x0003
#define ISP_TYPE_CAP_DATA  0x0004
#define ISP_TYPE_MAIN_INFO 0x0005

#define ISP_PACKET_ALL 0x0000
#define ISP_PACKET_BLC 0x0001
#define ISP_PACKET_NLC 0x0002
#define ISP_PACKET_LNC 0x0003
#define ISP_PACKET_AE 0x0004
#define ISP_PACKET_AWB 0x0005
#define ISP_PACKET_BPC 0x0006
#define ISP_PACKET_DENOISE 0x0007
#define ISP_PACKET_GRGB 0x0008
#define ISP_PACKET_CFA 0x0009
#define ISP_PACKET_CMC 0x000A
#define ISP_PACKET_GAMMA 0x000B
#define ISP_PACKET_UV_DIV 0x000C
#define ISP_PACKET_PREF 0x000D
#define ISP_PACKET_BRIGHT 0x000E
#define ISP_PACKET_CONTRAST 0x000F
#define ISP_PACKET_HIST 0x0010
#define ISP_PACKET_AUTO_CONTRAST 0x0011
#define ISP_PACKET_SATURATION 0x0012
#define ISP_PACKET_CSS 0x0013
#define ISP_PACKET_AF 0x0014
#define ISP_PACKET_EDGE 0x0015
#define ISP_PACKET_SPECIAL_EFFECT 0x0016
#define ISP_PACKET_HDR 0x0017
#define ISP_PACKET_GLOBAL 0x0018
#define ISP_PACKET_CHN 0x0019
#define ISP_PACKET_MAX 0xFFFF

#define ISP_DATA_YUV422_2FRAME (1<<0)
#define ISP_DATA_YUV420_2FRAME (1<<1)
#define ISP_DATA_NORMAL_RAW10 (1<<2)
#define ISP_DATA_MIPI_RAW10 (1<<3)
#define ISP_DATA_JPG (1<<4)

#define ISP_SIZE_640x480   (1<<0)
#define ISP_SIZE_800x600   (1<<1)
#define ISP_SIZE_1280x960  (1<<2)
#define ISP_SIZE_1280x1024 (1<<3)
#define ISP_SIZE_1600x1200 (1<<4)
#define ISP_SIZE_2048x1536 (1<<5)
#define ISP_SIZE_2592x1944 (1<<6)
#define ISP_SIZE_3264x2448 (1<<7)


#define ISP_UINT8 0x01
#define ISP_UINT16 0x02
#define ISP_UINT32 0x04
#define ISP_INT8 0x01
#define ISP_INT16 0x02
#define ISP_INT32 0x04

/**---------------------------------------------------------------------------*
**                               Data Prototype                              **
**----------------------------------------------------------------------------*/

enum isp_parser_cmd{
    ISP_PREVIEW=0x00,
    ISP_STOP_PREVIEW,
    ISP_CAPTURE,
    ISP_UP_PREVIEW_DATA,
    ISP_UP_PARAM,
    ISP_CAPTURE_SIZE,
    ISP_MAIN_INFO,
    ISP_PARSER_CMD_MAX
};

enum isp_tune_level{
    ISP_TUNE_AWB_MODE,
    ISP_TUNE_AE_MODE,
    ISP_TUNE_AE_MEASURE_LUM,
    ISP_TUNE_EV,
    ISP_TUNE_FLICKER,
    ISP_TUNE_SPECIAL_EFFECT,
    ISP_TUNE_BRIGHTNESS,
    ISP_TUNE_CONTRAST,
    ISP_TUNE_AUTO_CONTRAST,
    ISP_TUNE_SATURATION,
    ISP_TUNE_AF,
    ISP_TUNE_CSS,
    ISP_TUNE_HDR,
    ISP_TUNE_ISO,
    ISP_TUNE_MAX
};

enum isp_ctrl_cmd{
    ISP_CTRL_AWB_MODE,
    ISP_CTRL_AE_MODE,
    ISP_CTRL_AE_MEASURE_LUM,
    ISP_CTRL_EV,
    ISP_CTRL_FLICKER,
    ISP_CTRL_SPECIAL_EFFECT,
    ISP_CTRL_BRIGHTNESS,
    ISP_CTRL_CONTRAST,
    ISP_CTRL_HIST,
    ISP_CTRL_AUTO_CONTRAST,
    ISP_CTRL_SATURATION,
    ISP_CTRL_AF,
    ISP_CTRL_CSS,
    ISP_CTRL_HDR,
    ISP_CTRL_GLOBAL_GAIN,
    ISP_CTRL_CHN_GAIN,
    ISP_CTRL_EXIF,
    ISP_CTRL_ISO,
    ISP_CTRL_MAX
};

struct isp_main_info{
    char sensor_id[32];
    uint32_t version_id;
    uint32_t preview_format;
    uint32_t preview_size;
    uint32_t capture_format;
    uint32_t capture_size;
};

struct isp_size_info{
    uint32_t size_id;
    uint32_t width;
    uint32_t height;
};

struct isp_version_info{
    uint32_t version_id;
    uint32_t srtuct_size;
    uint32_t reserve;
};

struct isp_param_info{
    char main_type[32];
    char sub_type[32];
    char third_type[32];
    uint32_t data_type;
    uint32_t data_num;
    uint32_t addr_offset;
};

struct isp_parser_up_data{
    uint32_t format;
    uint32_t pattern;
    uint32_t width;
    uint32_t height;
    uint32_t* buf_addr;
    uint32_t buf_len;
};

struct isp_parser_buf_in{
    uint32_t buf_addr;
    uint32_t buf_len;
};

struct isp_parser_buf_rtn{
    uint32_t buf_addr;
    uint32_t buf_len;
};

struct isp_parser_cmd_param{
    enum isp_parser_cmd cmd;
    uint32_t param[16]; // capture param format/width/height
};

struct isp_param_fun{
    uint32_t cmd;
    int32_t (*param_fun)(void* param_ptr);
};

uint32_t ispParserGetSizeID(uint32_t width, uint32_t height);
int32_t ispParser(uint32_t cmd, void* in_param_ptr, void* rtn_param_ptr);
uint32_t* ispParserAlloc(uint32_t size);
int32_t ispParserFree(void* addr);

/**----------------------------------------------------------------------------*
**                         Compiler Flag                                      **
**----------------------------------------------------------------------------*/
#ifdef   __cplusplus
}
#endif
/**---------------------------------------------------------------------------*/
#endif
// End

