/******************************************************************************
 ** Copyright (c) 
 ** File Name:		sensor_GC0311.c 										  *
 ** Author: 													  *
 ** DATE:															  *
 ** Description:   This file contains driver for sensor GC0311. 
 ** 														 
 ******************************************************************************

 ******************************************************************************
 ** 					   Edit History 									  *
 ** ------------------------------------------------------------------------- *
 ** DATE		   NAME 			DESCRIPTION 							  *
 ** 	  
 ******************************************************************************/

/**---------------------------------------------------------------------------*
 ** 						Dependencies									  *
 **---------------------------------------------------------------------------*/
#include "sensor_cfg.h"
#include "sensor_drv_u.h"

/**---------------------------------------------------------------------------*
 ** 						Compiler Flag									  *
 **---------------------------------------------------------------------------*/
/**---------------------------------------------------------------------------*
 ** 					Extern Function Declaration 						  *
 **---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
 ** 						Const variables 								  *
 **---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
 ** 						   Macro Define
 **---------------------------------------------------------------------------*/
#define GC0311_I2C_ADDR_W		0x33 // 0x66 --> 0x66 / 2
#define GC0311_I2C_ADDR_R		0x33 // 0x67 --> 0x67 / 2

#define SENSOR_GAIN_SCALE		16
static uint32_t g_flash_mode_en = 0;
static uint32_t g_auto_flash_mode_en = 0;

typedef enum {
	FLASH_CLOSE = 0x0,
	FLASH_OPEN = 0x1,
	FLASH_TORCH = 0x2,	/*user only set flash to close/open/torch state */
	FLASH_AUTO = 0x3,
	FLASH_CLOSE_AFTER_OPEN = 0x10,	/* following is set to sensor */
	FLASH_HIGH_LIGHT = 0x11,
	FLASH_OPEN_ON_RECORDING = 0x22,
	FLASH_STATUS_MAX
} DCAM_FLASH_STATUS; 
/**---------------------------------------------------------------------------*
 ** 					Local Function Prototypes							  *
 **---------------------------------------------------------------------------*/
LOCAL uint32_t GC0311_PowerOn(uint32_t power_on);
LOCAL uint32_t set_GC0311_ae_enable(uint32_t enable);
LOCAL uint32_t set_preview_mode(uint32_t preview_mode);
LOCAL uint32_t GC0311_Identify(uint32_t param);
/*
LOCAL uint32_t GC0311_BeforeSnapshot(uint32_t param);
*/
LOCAL uint32_t set_brightness(uint32_t level);
LOCAL uint32_t set_contrast(uint32_t level);
LOCAL uint32_t set_sharpness(uint32_t level);
LOCAL uint32_t set_saturation(uint32_t level);
LOCAL uint32_t set_image_effect(uint32_t effect_type);

LOCAL uint32_t read_ev_value(uint32_t value);
LOCAL uint32_t write_ev_value(uint32_t exposure_value);
LOCAL uint32_t read_gain_value(uint32_t value);
LOCAL uint32_t write_gain_value(uint32_t gain_value);
LOCAL uint32_t read_gain_scale(uint32_t value);
LOCAL uint32_t set_frame_rate(uint32_t param);
LOCAL uint32_t GC0311_set_work_mode(uint32_t mode);

LOCAL uint32_t set_GC0311_ev(uint32_t level);
LOCAL uint32_t set_GC0311_awb(uint32_t mode);
LOCAL uint32_t set_GC0311_anti_flicker(uint32_t mode);
LOCAL uint32_t set_GC0311_video_mode(uint32_t mode);

LOCAL uint32_t _gc0311_BeforeSnapshot(uint32_t param);
LOCAL uint32_t _gc0311_flash(uint32_t param);

//LOCAL BOOLEAN gc_enter_effect = SENSOR_FALSE;

/**---------------------------------------------------------------------------*
 ** 						Local Variables 								 *
 **---------------------------------------------------------------------------*/
 typedef enum
{
	FLICKER_50HZ = 0,
	FLICKER_60HZ,
	FLICKER_MAX
}FLICKER_E;
//__align(4) const SENSOR_REG_T GC0311_YUV_640X480[]=
SENSOR_REG_T GC0311_YUV_640X480[]=
{
	{0xfe,  0xf0},
	{0xfe,  0xf0},
	{0xfe,  0xf0},
	{0x42,  0x00},
	{0x4f,  0x00},
	{0x03,  0x01}, 
	{0x04,  0x20},
	{0xfc,  0x16},

	///////////////////////////////////////////////
	/////////// system reg ////////////////////////
	///////////////////////////////////////////////
	{0xf1, 0x07},
	{0xf2, 0x01},
	{0xfc, 0x16},
	///////////////////////////////////////////////
	/////////// CISCTL////////////////////////
	///////////////////////////////////////////////
	{0xfe, 0x00},
	//////window setting/////
	{0x0d, 0x01},
	{0x0e, 0xe8},
	{0x0f, 0x02},
	{0x10, 0x88},
	{0x09, 0x00},
	{0x0a, 0x00},
	{0x0b, 0x00},
	{0x0c, 0x04},
							
	{0x05, 0x02}, 	
	{0x06, 0x2c}, 
	{0x07, 0x00},
	{0x08, 0xb8},
///////20120703	///////////////////
	{0x77, 0x7c},
	{0x78, 0x40},
	{0x79, 0x56},
  /////////////////////////////////////
	{0xfe, 0x01},
	{0x29, 0x00},   //anti-flicker step [11:8]
	{0x2a, 0x60},   //anti-flicker step [7:0]

	{0x2b, 0x02},   //exp level 0  14.28fps
	{0x2c, 0xa0}, 
	{0x2d, 0x03},   //exp level 1  12.50fps
	{0x2e, 0x00}, 
	{0x2f, 0x03},   //exp level 2  10.00fps
	{0x30, 0xc0}, 
	{0x31, 0x07},   //exp level 3  5.00fps
	{0x32, 0x80}, 
	{0x33, 0x20},
	{0xfe, 0x00},
						  
	{0x17, 0x14},
	{0x19, 0x04},
	{0x1f, 0x08},
	{0x20, 0x01},
	{0x21, 0x48},
	{0x1b, 0x48},
	{0x22, 0xba},
	{0x23, 0x06},// 07 --06  20120905
	{0x24, 0x16},
						   				   
	//global gain for range 	
	{0x70, 0x54},
	{0x73, 0x80},
	{0x76, 0x80},
	////////////////////////////////////////////////
	///////////////////////BLK//////////////////////
	////////////////////////////////////////////////
	{0x26, 0xf7},
	{0x28, 0x7f},
	{0x29, 0x40},
	{0x33, 0x18},
	{0x34, 0x18},
	{0x35, 0x18},
	{0x36, 0x18},

	////////////////////////////////////////////////
	//////////////block enable/////////////////////
	////////////////////////////////////////////////
	{0x40, 0xdf}, 
	{0x41, 0x2e}, 

	{0x42, 0xff},

	{0x44, 0xa2},
	{0x46, 0x02},
	{0x4d, 0x01},
	{0x4f, 0x01},
	{0x7e, 0x08},
	{0x7f, 0xc3},
							
	//DN & EEINTP				
	{0x80, 0xe7},
	{0x82, 0x30},
	{0x84, 0x02},
	{0x89, 0x22},
	{0x90, 0xbc},
	{0x92, 0x08},
	{0x94, 0x08},
	{0x95, 0x64},
							 
	/////////////////////ASDE/////////////
	{0x9a, 0x15},
	{0x9c, 0x46},

	///////////////////////////////////////
	////////////////Y gamma ///////////////////
	////////////////////////////////////////////
	{0xfe, 0x00},
	{0x63, 0x00}, 
	{0x64, 0x06}, 
	{0x65, 0x0c}, 
	{0x66, 0x18},
	{0x67, 0x2A},
	{0x68, 0x3D},
	{0x69, 0x50},
	{0x6A, 0x60},
	{0x6B, 0x80},
	{0x6C, 0xA0},
	{0x6D, 0xC0},
	{0x6E, 0xE0},
	{0x6F, 0xFF},
	{0xfe, 0x00},

	///////////////////////////////////////
	////////////////RGB gamma //////////////
	///////////////////////////////////////
	{0xBF, 0x0E},
	{0xc0, 0x1C},
	{0xc1, 0x34},
	{0xc2, 0x48},
	{0xc3, 0x5A},
	{0xc4, 0x6B},
	{0xc5, 0x7B},
	{0xc6, 0x95},
	{0xc7, 0xAB},
	{0xc8, 0xBF},
	{0xc9, 0xCE},
	{0xcA, 0xD9},
	{0xcB, 0xE4},
	{0xcC, 0xEC},
	{0xcD, 0xF7},
	{0xcE, 0xFD},
	{0xcF, 0xFF},

	////////////////////////////
	/////////////YCP//////////////
	////////////////////////////
	{0xd1, 0x36},
	{0xd2, 0x36},
	{0xdd, 0x00},
	{0xed, 0x00},

	{0xde, 0x38},
	{0xe4, 0x88},
	{0xe5, 0x40},

	{0xfe, 0x01},
	{0x18, 0x22},

	//////////////////////////////////
	///////////MEANSURE WINDOW////////
	/////////////////////////////////
	{0x08, 0xa4},
	{0x09, 0xf0},

	///////////////////////////////////////////////
	/////////////// AEC ////////////////////////
	///////////////////////////////////////////////
	{0xfe, 0x01},

	{0x10, 0x08},
			 
	{0x11, 0x11},
	{0x12, 0x14},
	{0x13, 0x40},
	{0x16, 0xd8},
	{0x17, 0x9a},
	{0x18, 0x01},
	{0x21, 0xc0},
	{0x22, 0x40},

	//////////////////////////////
	/////////////AWB///////////////
	////////////////////////////////
	{0x06, 0x10},
	{0x08, 0xa0},
							
	{0x50, 0xfe},
	{0x51, 0x05},
	{0x52, 0x28},
	{0x53, 0x05},
	{0x54, 0x10},
	{0x55, 0x20},
	{0x56, 0x16},
	{0x57, 0x10},
	{0x58, 0xf0},
	{0x59, 0x10},
	{0x5a, 0x10},
	{0x5b, 0xf0},
	{0x5e, 0xe8},
	{0x5f, 0x20},
	{0x60, 0x20},
	{0x61, 0xe0},
							
	{0x62, 0x03},
	{0x63, 0x30},
	{0x64, 0xc0},
	{0x65, 0xd0},
	{0x66, 0x20},
	{0x67, 0x00},

	{0x6d, 0x40},
	{0x6e, 0x08},
	{0x6f, 0x08},
	{0x70, 0x10},
	{0x71, 0x62},
	{0x72, 0x2e},//26 fast mode
	{0x73, 0x71},
	{0x74, 0x23},
							
	{0x75, 0x40},
	{0x76, 0x48},
	{0x77, 0xc2},
	{0x78, 0xa5},
							 
	{0x79, 0x18},
	{0x7a, 0x40},
	{0x7b, 0xb0},
	{0x7c, 0xf5},
							 
	{0x81, 0x80},
	{0x82, 0x60},
	{0x83, 0xa0},
							
	{0x8a, 0xf8},
	{0x8b, 0xf4},
	{0x8c, 0x0a},
	{0x8d, 0x00},
	{0x8e, 0x00},
	{0x8f, 0x00},
	{0x90, 0x12},
							
	{0xfe, 0x00},

	///////////////////////////////////////////////
	/////////// SPI reciver////////////////////////
	///////////////////////////////////////////////
	{0xad, 0x00},

	/////////////////////////////
	///////////LSC///////////////
	/////////////////////////////
	{0xfe, 0x01},
	{0xa0, 0x00},
	{0xa1, 0x3c},
	{0xa2, 0x50},
	{0xa3, 0x00},
	{0xa8, 0x09},
	{0xa9, 0x04},
	{0xaa, 0x00},
	{0xab, 0x0c},
	{0xac, 0x02},
	{0xad, 0x00},
	{0xae, 0x15},
	{0xaf, 0x05},
	{0xb0, 0x00},
	{0xb1, 0x0f},
	{0xb2, 0x06},
	{0xb3, 0x00},
	{0xb4, 0x36},
	{0xb5, 0x2a},
	{0xb6, 0x25},
	{0xba, 0x36},
	{0xbb, 0x25},
	{0xbc, 0x22},
	{0xc0, 0x1e},
	{0xc1, 0x18},
	{0xc2, 0x17},
	{0xc6, 0x1c},
	{0xc7, 0x18},
	{0xc8, 0x17},
	{0xb7, 0x00},
	{0xb8, 0x00},
	{0xb9, 0x00},
	{0xbd, 0x00},
	{0xbe, 0x00},
	{0xbf, 0x00},
	{0xc3, 0x00},
	{0xc4, 0x00},
	{0xc5, 0x00},
	{0xc9, 0x00},
	{0xca, 0x00},
	{0xcb, 0x00},
	{0xa4, 0x00},
	{0xa5, 0x00},
	{0xa6, 0x00},
	{0xa7, 0x00},

//////20120614 start///////
	{0xfe, 0x01},
	{0x74, 0x13},
	{0x15, 0xfe},
	{0x21, 0xe0},

	{0xfe, 0x00},
	{0x41, 0x6e},
	{0x83, 0x03},
	{0x7e, 0x08},
	{0x9c, 0x64},
	{0x95, 0x65},

	{0xd1, 0x2d},
	{0xd2, 0x2d},
	
	{0xb0, 0x13},
	{0xb1, 0x26},
	{0xb2, 0x07},
	{0xb3, 0xf5},
	{0xb4, 0xea},
	{0xb5, 0x21},
	{0xb6, 0x21},
	{0xb7, 0xe4},
	{0xb8, 0xfb},
//////20120614 end///////

	{0xfe, 0x00},
	{0x50, 0x01},
	{0x44, 0xa2},
	//{0x24, 0x16},

};


LOCAL SENSOR_REG_TAB_INFO_T s_GC0311_resolution_Tab_YUV[]=
{
	// COMMON INIT
	{ADDR_AND_LEN_OF_ARRAY(GC0311_YUV_640X480), 640, 480, 24, SENSOR_IMAGE_FORMAT_YUV422},
	
	// YUV422 PREVIEW 1	
	{PNULL, 0, 640, 480,24, SENSOR_IMAGE_FORMAT_YUV422},
	{PNULL, 0, 0, 0, 0, 0},
	{PNULL, 0, 0, 0, 0, 0},
	{PNULL, 0, 0, 0, 0, 0},
	
	// YUV422 PREVIEW 2 
	{PNULL, 0, 0, 0, 0, 0},
	{PNULL, 0, 0, 0, 0, 0},
	{PNULL, 0, 0, 0, 0, 0},
	{PNULL, 0, 0, 0, 0, 0}

};

LOCAL SENSOR_VIDEO_INFO_T s_GC0311_video_info[] = {
	{{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, PNULL},
	{{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, PNULL},
	{{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, PNULL},
	{{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, PNULL},
	{{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, PNULL},
	{{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, PNULL},
	{{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, PNULL},
	{{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, PNULL},
	{{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}, PNULL}
};

LOCAL SENSOR_IOCTL_FUNC_TAB_T s_GC0311_ioctl_func_tab = 
{
    // Internal 
    PNULL,
    GC0311_PowerOn,
    PNULL,
    GC0311_Identify,

    PNULL,			// write register
    PNULL,			// read  register	
    PNULL,
    PNULL,

    // External
    PNULL,// set_GC0311_ae_enable,
    PNULL,//set_hmirror_enable,
    PNULL,//set_vmirror_enable,

    set_brightness,
    set_contrast,
    set_sharpness,
    set_saturation,

    set_preview_mode,	
    set_image_effect,

    _gc0311_BeforeSnapshot,
    PNULL, //_gc0311_after_snapshot,
    _gc0311_flash,

    read_ev_value,
    write_ev_value,
    read_gain_value,
    write_gain_value,
    read_gain_scale,
    set_frame_rate,	
    PNULL,
    PNULL,
    set_GC0311_awb,
    PNULL,
    PNULL,
    set_GC0311_ev,
    PNULL,
    PNULL,
    PNULL,
    PNULL,
    PNULL,
    set_GC0311_anti_flicker,
    set_GC0311_video_mode,
    PNULL,
	PNULL,
	PNULL,
	PNULL,
	PNULL,
	PNULL,
};

/**---------------------------------------------------------------------------*
 ** 						Global Variables								  *
 **---------------------------------------------------------------------------*/
SENSOR_INFO_T g_GC0311_yuv_info =
{
	GC0311_I2C_ADDR_W,				// salve i2c write address
	GC0311_I2C_ADDR_R, 				// salve i2c read address
	
	0,								// bit0: 0: i2c register value is 8 bit, 1: i2c register value is 16 bit
									// bit2: 0: i2c register addr  is 8 bit, 1: i2c register addr  is 16 bit
									// other bit: reseved
	SENSOR_HW_SIGNAL_PCLK_P|\
	SENSOR_HW_SIGNAL_VSYNC_N|\
	SENSOR_HW_SIGNAL_HSYNC_P,		// bit0: 0:negative; 1:positive -> polarily of pixel clock
									// bit2: 0:negative; 1:positive -> polarily of horizontal synchronization signal
									// bit4: 0:negative; 1:positive -> polarily of vertical synchronization signal
									// other bit: reseved											
											
	// preview mode
	SENSOR_ENVIROMENT_NORMAL|\
	SENSOR_ENVIROMENT_NIGHT|\
	SENSOR_ENVIROMENT_SUNNY,		
	
	// image effect
	SENSOR_IMAGE_EFFECT_NORMAL|\
	SENSOR_IMAGE_EFFECT_BLACKWHITE|\
	SENSOR_IMAGE_EFFECT_RED|\
	SENSOR_IMAGE_EFFECT_GREEN|\
	SENSOR_IMAGE_EFFECT_BLUE|\
	SENSOR_IMAGE_EFFECT_YELLOW|\
	SENSOR_IMAGE_EFFECT_NEGATIVE|\
	SENSOR_IMAGE_EFFECT_CANVAS,
	
	// while balance mode
	0,
		
	7,								// bit[0:7]: count of step in brightness, contrast, sharpness, saturation
									// bit[8:31] reseved
	
	SENSOR_LOW_PULSE_RESET,			// reset pulse level
	100,								// reset pulse width(ms)
	
	SENSOR_HIGH_LEVEL_PWDN,			// 1: high level valid; 0: low level valid	
		
	2,								// count of identify code
	{{0xf0, 0xbb},						// supply two code to identify sensor.
	{0xf0, 0xbb}},						// for Example: index = 0-> Device id, index = 1 -> version id
									
	SENSOR_AVDD_2800MV,				// voltage of avdd

	640,							// max width of source image
	480,							// max height of source image
	"GC0311",						// name of sensor												

	SENSOR_IMAGE_FORMAT_YUV422,		// define in SENSOR_IMAGE_FORMAT_E enum,
									// if set to SENSOR_IMAGE_FORMAT_MAX here, image format depent on SENSOR_REG_TAB_INFO_T
	SENSOR_IMAGE_PATTERN_YUV422_YUYV,	// pattern of input image form sensor;			

	s_GC0311_resolution_Tab_YUV,	// point to resolution table information structure
	&s_GC0311_ioctl_func_tab,		// point to ioctl function table
			
	PNULL,							// information and table about Rawrgb sensor
	PNULL,							// extend information about sensor	
	SENSOR_AVDD_1800MV,                     // iovdd
	SENSOR_AVDD_1800MV,                      // dvdd
	4,						//skip frame num before preview
	3,						//skip frame num before capture
	0,						//deci frame num during preview
	0,						//deci frame num during video preview
	0,						//threshold enable(only analog TV)
	0,						//atv output mode 0 fix mode 1 auto mode
	0,						//atv output start postion
	0,						//atv output end postion
	0,
	{SENSOR_INTERFACE_TYPE_CCIR601, 8, 16, 1},
	s_GC0311_video_info,
	4,						//skip frame num while change setting
};

static uint32_t GC0311_PowerOn(uint32_t power_on)
{
	SENSOR_AVDD_VAL_E dvdd_val = g_GC0311_yuv_info.dvdd_val;
	SENSOR_AVDD_VAL_E avdd_val = g_GC0311_yuv_info.avdd_val;
	SENSOR_AVDD_VAL_E iovdd_val = g_GC0311_yuv_info.iovdd_val;
	BOOLEAN power_down = g_GC0311_yuv_info.power_down_level;
	BOOLEAN reset_level = g_GC0311_yuv_info.reset_pulse_level;

	if (SENSOR_TRUE == power_on) {
		Sensor_PowerDown(!power_down);
		usleep(10*1000);
		Sensor_SetMCLK(SENSOR_DEFALUT_MCLK);
		usleep(10*1000);
		Sensor_SetVoltage(dvdd_val, avdd_val, iovdd_val);
		usleep(10*1000);
		Sensor_Reset(reset_level);
	} else {
		Sensor_PowerDown(power_down);
		usleep(5*1000);
		Sensor_SetResetLevel(reset_level);
		usleep(5*1000);
		Sensor_SetVoltage(SENSOR_AVDD_CLOSED, SENSOR_AVDD_CLOSED,
				SENSOR_AVDD_CLOSED);
		usleep(5*1000);
		Sensor_SetMCLK(SENSOR_DISABLE_MCLK);
	}
	SENSOR_PRINT("(1:on, 0:off): %d", power_on);
	return (uint32_t)SENSOR_SUCCESS;
}

/**---------------------------------------------------------------------------*
 ** 							Function  Definitions
 **---------------------------------------------------------------------------*/
LOCAL void GC0311_WriteReg( uint8_t  subaddr, uint8_t data )
{
	#ifndef	_USE_DSP_I2C_
/*		uint8_t cmd[2];
		cmd[0]	=	subaddr;
		cmd[1]	=	data;		
		I2C_WriteCmdArr(GC0311_I2C_ADDR_W, cmd, 2, SENSOR_TRUE);*/
			Sensor_WriteReg_8bits(subaddr, data);
//		Sensor_WriteReg(subaddr, data);
	#else
		//printk("anrry:before DSENSOR_IICWrite\n");
		DSENSOR_IICWrite((uint16_t)subaddr, (uint16_t)data);
		//printk("anrry:after DSENSOR_IICWrite\n");
	#endif

	SENSOR_TRACE("SENSOR: GC0311_WriteReg reg/value(%x,%x) !!\n", subaddr, data);

}

LOCAL uint8_t GC0311_ReadReg( uint8_t  subaddr)
{
	uint8_t value = 0;
	
	#ifndef	_USE_DSP_I2C_
	//I2C_WriteCmdArrNoStop(GC0311_I2C_ADDR_W, &subaddr, 1,SENSOR_TRUE);
	//I2C_ReadCmd(GC0311_I2C_ADDR_R, &value, SENSOR_TRUE);
	//value =Sensor_ReadReg_8bits( subaddr);
	value = Sensor_ReadReg( subaddr);
	#else
		value = (uint16_t)DSENSOR_IICRead((uint16_t)subaddr);
	#endif

    SENSOR_TRACE("SENSOR: GC0311_ReadReg reg/value(%x,%x) !!\n", subaddr, value);
    
	return value;
}

LOCAL uint32_t GC0311_Identify(uint32_t param)
{
	uint8_t reg[2]    = {0xf0, 0xf0};
	uint8_t value[2]  = {0xbb, 0xbb};
	uint8_t ret       = 0;
	uint32_t i;
    uint8_t   err_cnt = 0;
    uint32_t nLoop = 1000;

    for(i = 0; i<2; )
    {
        nLoop = 1000;
        ret = GC0311_ReadReg(reg[i]);
	SENSOR_PRINT("%s sensor_id is %x\n", __func__, reg[i]);

        if( ret != value[i])
        {
            err_cnt++;
            if(err_cnt>3)
            {
            	SENSOR_PRINT( "GC0311 Fail" );
	             return SENSOR_FAIL;
            }
            else
            {
                //Masked by frank.yang,SCI_Sleep() will cause a  Assert when called in boot precedure
                //SCI_Sleep(10);
                while(nLoop--);
                continue;
            }
        }
        err_cnt = 0;
        i++;
    }
	SENSOR_PRINT("GC0311_Identify: it is GC0311\n");
	return (uint32_t)SENSOR_SUCCESS;
}

LOCAL uint32_t set_GC0311_ae_enable(uint32_t enable)
{
	SENSOR_TRACE("set_GC0311_ae_enable: enable = %d\n", enable);
	return 0;
}

/*

LOCAL uint32_t set_hmirror_enable(uint32_t enable)
{
 	uint8_t value = 0;	
	value = GC0311_ReadReg(0x17);
	value = (value & 0xFE) | (enable == 1 ? 0 : 1); //landscape
	SENSOR_TRACE("set_hmirror_enable: enable = %d, 0x14: 0x%x.\n", enable, value);
	GC0311_WriteReg(0x17, value);
	
	return 0;
}


LOCAL uint32_t set_vmirror_enable(uint32_t enable)
{
	uint8_t value = 0;	
	value = GC0311_ReadReg(0x17);
	value = (value & 0xFD) | ((enable & 0x1) << 1); //portrait
	SENSOR_TRACE("set_vmirror_enable: enable = %d, 0x14: 0x%x.\n", enable, value);
	GC0311_WriteReg(0x17, value);
	
	return 0;
}
*/
/******************************************************************************/
// Description: set brightness 
// Global resource dependence: 
// Author:
// Note:
//		level  must smaller than 8
/******************************************************************************/
//__align(4) const SENSOR_REG_T GC0311_brightness_tab[][2]=
SENSOR_REG_T GC0311_brightness_tab[][2]=
{	
    {{0xd5, 0xd0}, {0xff, 0xff}},
    {{0xd5, 0xe0}, {0xff, 0xff}},
    {{0xd5, 0xf0}, {0xff, 0xff}},
    {{0xd5, 0x00}, {0xff, 0xff}},// level zero
    {{0xd5, 0x20}, {0xff, 0xff}},
    {{0xd5, 0x30}, {0xff, 0xff}},
    {{0xd5, 0x40}, {0xff, 0xff}},  
};


LOCAL uint32_t set_brightness(uint32_t level)
{
	uint16_t i;
	SENSOR_REG_T* sensor_reg_ptr = (SENSOR_REG_T*)GC0311_brightness_tab[level];

	//SENSOR_ASSERT(level < 7);
	//SENSOR_ASSERT(PNULL != sensor_reg_ptr);
	
	for(i = 0; (0xFF != sensor_reg_ptr[i].reg_addr) && (0xFF != sensor_reg_ptr[i].reg_value); i++)
	{
		GC0311_WriteReg(sensor_reg_ptr[i].reg_addr, sensor_reg_ptr[i].reg_value);
	}
	SENSOR_Sleep(100); 
	SENSOR_TRACE("set_brightness: level = %d\n", level);
	
	return 0;
}

//__align(4) const SENSOR_REG_T GC0311_ev_tab[][3]=
SENSOR_REG_T GC0311_ev_tab[][4]=
{
    {{0xfe, 0x01}, {0x13, 0x10}, {0xfe, 0x00}, {0xff, 0xff}},
    {{0xfe, 0x01}, {0x13, 0x20}, {0xfe, 0x00}, {0xff, 0xff}},
    {{0xfe, 0x01}, {0x13, 0x30}, {0xfe, 0x00}, {0xff, 0xff}},
    {{0xfe, 0x01}, {0x13, 0x40}, {0xfe, 0x00}, {0xff, 0xff}},// level zero
    {{0xfe, 0x01}, {0x13, 0x60}, {0xfe, 0x00}, {0xff, 0xff}},
    {{0xfe, 0x01}, {0x13, 0x80}, {0xfe, 0x00}, {0xff, 0xff}},
    {{0xfe, 0x01}, {0x13, 0xb0}, {0xfe, 0x00}, {0xff, 0xff}},  
};;


LOCAL uint32_t set_GC0311_ev(uint32_t level)
{
    uint16_t i; 
    SENSOR_REG_T* sensor_reg_ptr = (SENSOR_REG_T*)GC0311_ev_tab[level];

    //SENSOR_ASSERT(PNULL != sensor_reg_ptr);
    //SENSOR_ASSERT(level < 7);
 
    for(i = 0; (0xFF != sensor_reg_ptr[i].reg_addr) ||(0xFF != sensor_reg_ptr[i].reg_value) ; i++)
    {
        GC0311_WriteReg(sensor_reg_ptr[i].reg_addr, sensor_reg_ptr[i].reg_value);
    }

    
    SENSOR_TRACE("SENSOR: set_ev: level = %d\n", level);

    return 0;
}

/******************************************************************************/
// Description: anti 50/60 hz banding flicker
// Global resource dependence: 
// Author:
// Note:
//		level  must smaller than 8
/******************************************************************************/
LOCAL uint32_t set_GC0311_anti_flicker(uint32_t param )
{
    switch (param)
    {
        case FLICKER_50HZ:
    		GC0311_WriteReg(0xfe,0x00);
		GC0311_WriteReg(0x05,0x02);
		GC0311_WriteReg(0x06,0x2c);
		GC0311_WriteReg(0x07,0x00);
		GC0311_WriteReg(0x08,0xb8);
		GC0311_WriteReg(0xfe,0x01);
		GC0311_WriteReg(0x29,0x00);
		GC0311_WriteReg(0x2a,0x60);
		GC0311_WriteReg(0x2b,0x02);
		GC0311_WriteReg(0x2c,0xa0);
		GC0311_WriteReg(0x2d,0x03);
		GC0311_WriteReg(0x2e,0x00);
		GC0311_WriteReg(0x2f,0x03);
		GC0311_WriteReg(0x30,0xc0);
		GC0311_WriteReg(0x31,0x05);
		GC0311_WriteReg(0x32,0x40);
		GC0311_WriteReg(0xfe,0x00);

            break;

        case FLICKER_60HZ:
		GC0311_WriteReg(0xfe,0x00);
		GC0311_WriteReg(0x05,0x02);
		GC0311_WriteReg(0x06,0x4c);
		GC0311_WriteReg(0x07,0x00);
		GC0311_WriteReg(0x08,0x88);
		GC0311_WriteReg(0xfe,0x01);
		GC0311_WriteReg(0x29,0x00);
		GC0311_WriteReg(0x2a,0x4e);
		GC0311_WriteReg(0x2b,0x02);
		GC0311_WriteReg(0x2c,0x70);
		GC0311_WriteReg(0x2d,0x03);
		GC0311_WriteReg(0x2e,0x0c);
		GC0311_WriteReg(0x2f,0x03);
		GC0311_WriteReg(0x30,0xa8);
		GC0311_WriteReg(0x31,0x05);
		GC0311_WriteReg(0x32,0x2e);
		GC0311_WriteReg(0xfe,0x00);
            break;

        default:
            break;
    }

    return 0;
}

/******************************************************************************/
// Description: set video mode
// Global resource dependence: 
// Author:
// Note:
//		 
/******************************************************************************/
//__align(4) const SENSOR_REG_T GC0311_video_mode_nor_tab[][15]=
SENSOR_REG_T GC0311_video_mode_nor_tab[][18]=
{
    // normal mode      14.3 fps
    {
    		{0xfe,0x00},
    		{0x05,0x02},
    		{0x06,0x2c},
    		{0x07,0x00},
    		{0x08,0xb8},
    		{0xfe,0x01},
    		{0x29,0x00},
    		{0x2a,0x60},
    		{0x2b,0x02},
    		{0x2c,0xa0},
    		{0x2d,0x03},
    		{0x2e,0x00},
    		{0x2f,0x03},
    		{0x30,0xc0},
    		{0x31,0x05},
    		{0x32,0x40},
    		{0xfe,0x00},
    		{0xff,0xff}
	},
    //vodeo mode
    {
    		{0xfe,0x00},
    		{0x05,0x02},
    		{0x06,0x2c},
    		{0x07,0x00},
    		{0x08,0xb8},
    		{0xfe,0x01},
    		{0x29,0x00},
    		{0x2a,0x60},
    		{0x2b,0x03},
    		{0x2c,0x00},
    		{0x2d,0x03},
    		{0x2e,0x00},
    		{0x2f,0x03},
    		{0x30,0x00},
    		{0x31,0x05},
    		{0x32,0x40},
    		{0xfe,0x00},
    		{0xff,0xff}
    }

};    

LOCAL uint32_t set_GC0311_video_mode(uint32_t mode)
{
    //uint8_t data=0x00;
    uint16_t i;
    SENSOR_REG_T* sensor_reg_ptr = PNULL;
    uint8_t tempregval = 0;

    //SENSOR_ASSERT(mode <= 1);

    sensor_reg_ptr = (SENSOR_REG_T*)GC0311_video_mode_nor_tab[mode];

    //SENSOR_ASSERT(PNULL != sensor_reg_ptr);

    for(i = 0; (0xFF != sensor_reg_ptr[i].reg_addr) || (0xFF != sensor_reg_ptr[i].reg_value); i++)
    {
    	GC0311_WriteReg(sensor_reg_ptr[i].reg_addr, sensor_reg_ptr[i].reg_value);
    }
//ziji	tempregval = GC0311_ReadReg(0xd2);
//    SENSOR_TRACE("SENSOR: GC0311_ReadReg(0xd2) = %x\n", tempregval);
//    SENSOR_TRACE("SENSOR: set_video_mode: mode = %d\n", mode);
    return 0;
}
//__align(4) const SENSOR_REG_T GC0311_awb_tab[][5]=
SENSOR_REG_T GC0311_awb_tab[][5]=
	{
		   	//AUTO
		   	{
				{0x77, 0x7c},
				{0x78, 0x40},
				{0x79, 0x56},
				{0x42, 0xfe},
				{0xff, 0xff}
			},
			//INCANDESCENCE:
			{
				{0x42, 0xfd},    // Disable AWB
				{0x77, 0x48},
				{0x78, 0x40},
				{0x79, 0x5c},
				{0xff, 0xff}
			},
			//U30
			{
				{0x42, 0xfd},   // Disable AWB
				{0x77, 0x40},
				{0x78, 0x54},
				{0x79, 0x70},
				{0xff, 0xff}
			},
			//CWF  //
			{
				{0x42, 0xfd},   // Disable AWB
				{0x77, 0x40},
				{0x78, 0x54},
				{0x79, 0x70},
				{0xff, 0xff}
			},
			//FLUORESCENT:
			{
				{0x42, 0xfd},   // Disable AWB
				{0x77, 0x40},
				{0x78, 0x42},
				{0x79, 0x50},
				{0xff, 0xff} 
			},
			//SUN:
			{
				{0x42, 0xfd},   // Disable AWB
				{0x77, 0x50},
				{0x78, 0x45},
				{0x79, 0x40},
				{0xff, 0xff} 
			},
		            //CLOUD:
			{
				{0x42, 0xfd},   // Disable AWB
				{0x77, 0x5a},
				{0x78, 0x42},
				{0x79, 0x40},
				{0xff, 0xff}
			},
	};
	
	LOCAL uint32_t set_GC0311_awb(uint32_t mode)
	{
		uint8_t awb_en_value;
		uint32_t i;
		SENSOR_REG_T* sensor_reg_ptr = (SENSOR_REG_T*)GC0311_awb_tab[mode];

		//SENSOR_ASSERT(mode < 7);
		//SENSOR_ASSERT(PNULL != sensor_reg_ptr);


	    for(i = 0; (0xFF != sensor_reg_ptr[i].reg_addr) || (0xFF != sensor_reg_ptr[i].reg_value) ; i++)
	    {
	    	GC0311_WriteReg(sensor_reg_ptr[i].reg_addr, sensor_reg_ptr[i].reg_value);
	    }
		SENSOR_Sleep(100); 
		SENSOR_TRACE("SENSOR: set_awb_mode: mode = %d\n", mode);
		
		return 0;
}

//__align(4) const SENSOR_REG_T GC0311_contrast_tab[][2]=
SENSOR_REG_T GC0311_contrast_tab[][2]=
{
		{
			{0xd3,0x34}, 	{0xff,0xff},
		},

		{
			{0xd3,0x38}, 	{0xff,0xff},
		},

		{
			{0xd3,0x3d}, 	{0xff,0xff},
		},

		{
			{0xd3,0x40}, 	{0xff,0xff},
		},

		{
			{0xd3,0x44}, 	{0xff,0xff},
		},

		{
			{0xd3,0x48}, 	{0xff,0xff},
		},

		{
			{0xd3,0x50}, 	{0xff,0xff},
		},
};

LOCAL uint32_t _gc0311_BeforeSnapshot(uint32_t param)
{
	if (g_flash_mode_en) {
		if(g_auto_flash_mode_en) {  //40
			Sensor_SetFlash(FLASH_OPEN);
			g_auto_flash_mode_en = 1;
		}else {
			Sensor_SetFlash(FLASH_CLOSE);
		}
	}

	return SENSOR_SUCCESS;
}

LOCAL uint32_t _gc0311_flash(uint32_t param)
{
	uint16_t value = 0;

	//printk("SENSOR: _gc0311_flash:param=0x%x.\n", param);
	/* enable flash, disable in _gc0311_BeforeSnapshot */
	g_flash_mode_en = param;
	if(FLASH_OPEN == param) {
			g_auto_flash_mode_en = 1;
		}
	Sensor_SetFlash(param);

	return SENSOR_SUCCESS;
}

LOCAL uint32_t set_contrast(uint32_t level)
{
    uint32_t i;
	SENSOR_REG_T* sensor_reg_ptr = (SENSOR_REG_T*)GC0311_contrast_tab[level];
	//SENSOR_ASSERT(level < 7);
	//SENSOR_ASSERT(PNULL != sensor_reg_ptr);

    for(i = 0; (0xFF != sensor_reg_ptr[i].reg_addr) || (0xFF != sensor_reg_ptr[i].reg_value) ; i++)
    {
    	GC0311_WriteReg(sensor_reg_ptr[i].reg_addr, sensor_reg_ptr[i].reg_value);
    }
    SENSOR_Sleep(20);
    SENSOR_TRACE("set_contrast: level = %d\n", level);
    return 0;
}


LOCAL uint32_t set_sharpness(uint32_t level)
{
	
	return 0;
}


LOCAL uint32_t set_saturation(uint32_t level)
{

	
	return 0;
}

/******************************************************************************/
// Description: set brightness 
// Global resource dependence: 
// Author:
// Note:
//		level  must smaller than 8
/******************************************************************************/

LOCAL uint32_t set_preview_mode(uint32_t preview_mode)
{
	SENSOR_TRACE("set_preview_mode: preview_mode = %d\n", preview_mode);
	
	
	switch (preview_mode)
	{
		case DCAMERA_ENVIRONMENT_NORMAL: 
		{
			GC0311_set_work_mode(0);
			break;
		}
		case DCAMERA_ENVIRONMENT_NIGHT:
		{
			GC0311_set_work_mode(1);
			break;
		}
		case DCAMERA_ENVIRONMENT_SUNNY:
		{
			GC0311_set_work_mode(0);
			break;
		}
		default:
		{
			break;
		}
			
	}
	
	SENSOR_Sleep(250);
	
	return 0;
}

//__align(4) const SENSOR_REG_T GC0311_image_effect_tab[][11]=	
SENSOR_REG_T GC0311_image_effect_tab[][6]=
{
		// effect normal
			{
				{0x43,0x00},
				{0x95,0x45},
				{0xd3,0x40},
				{0xda,0x00},
				{0xdb,0x00},
				{0xff,0xff}
			},
			//effect BLACKWHITE
			{
				{0x43,0x02},
				{0xd3,0x40},
				{0xda,0x00},
				{0xdb,0x00},
				{0xff,0xff},
				{0xff,0xff}
			},
			// effect RED pink
			{
				{0x43,0x02},
				{0x95,0x88},
				{0xd3,0x40},
				{0xda,0x10},
				{0xdb,0x50},
				{0xff, 0xff},
			},
			// effect GREEN
			{
				{0x43,0x02},
				{0x95,0x88},
				{0xd3,0x40},
				{0xda,0xc0},
				{0xdb,0xc0},
				{0xff, 0xff},
			},
			// effect  BLUE
			{
				{0x43,0x02},
				{0xd3,0x40},
				{0xda,0x50},
				{0xdb,0xe0},
				{0xff, 0xff},
				{0xff, 0xff}
			},
			// effect  YELLOW
			{
				{0x43,0x02},
				{0x95,0x88},
				{0xd3,0x40},
				{0xda,0x80},
				{0xdb,0x20},
				{0xff, 0xff}
			},
			// effect NEGATIVE
			{
				{0x43,0x01},
				{0xd3,0x40},
				{0xda,0x00},
				{0xdb,0x00},
				{0xff, 0xff},
				{0xff, 0xff}
			},
			//effect ANTIQUE
			{
				{0x43,0x02},
				{0xd3,0x40},
				{0xda,0xd2},
				{0xdb,0x28},
				{0xff, 0xff},
				{0xff, 0xff}
			},
};
LOCAL uint32_t set_image_effect(uint32_t effect_type)
{
    uint16_t i;
    
    SENSOR_REG_T* sensor_reg_ptr = (SENSOR_REG_T*)GC0311_image_effect_tab[effect_type];

    //SENSOR_ASSERT(PNULL != sensor_reg_ptr);

    for(i = 0; (0xFF != sensor_reg_ptr[i].reg_addr) || (0xFF != sensor_reg_ptr[i].reg_value) ; i++)
    {
        Sensor_WriteReg_8bits(sensor_reg_ptr[i].reg_addr, sensor_reg_ptr[i].reg_value);
    }
    SENSOR_TRACE("-----------set_image_effect: effect_type = %d------------\n", effect_type);
    
    return 0;
}


LOCAL uint32_t read_ev_value(uint32_t value)
{
	return 0;
}

LOCAL uint32_t write_ev_value(uint32_t exposure_value)
{
	
	return 0;	
}

LOCAL uint32_t read_gain_value(uint32_t value)
{

	
	return 0;
}

LOCAL uint32_t write_gain_value(uint32_t gain_value)
{

	
	return 0;
}

LOCAL uint32_t read_gain_scale(uint32_t value)
{
	return SENSOR_GAIN_SCALE;
	
}


LOCAL uint32_t set_frame_rate(uint32_t param)
    
{
	//GC0311_WriteReg( 0xd8, uint8_t data );
	return 0;
}

/******************************************************************************/
// Description:
// Global resource dependence: 
// Author:
// Note:
//		mode 0:normal;	 1:night 
/******************************************************************************/
//__align(4) const SENSOR_REG_T GC0311_mode_tab[][8]=
SENSOR_REG_T GC0311_mode_tab[][8]=
{
	{{0xfe, 0x01},{0x33, 0x20},{0xfe, 0x00},{0xFF, 0xFF},},
	{{0xfe, 0x01},{0x33, 0x30},{0xfe, 0x00},{0xFF, 0xFF},},
};

LOCAL uint32_t GC0311_set_work_mode(uint32_t mode)
{
	uint16_t i;
	SENSOR_REG_T* sensor_reg_ptr = (SENSOR_REG_T*)GC0311_mode_tab[mode];

	//SENSOR_ASSERT(mode <= 1);
	//SENSOR_ASSERT(PNULL != sensor_reg_ptr);
	
	for(i = 0; (0xFF != sensor_reg_ptr[i].reg_addr) || (0xFF != sensor_reg_ptr[i].reg_value); i++)
	{
		GC0311_WriteReg(sensor_reg_ptr[i].reg_addr, sensor_reg_ptr[i].reg_value);
	}

	SENSOR_TRACE("set_work_mode: mode = %d\n", mode);
	return 0;
}
