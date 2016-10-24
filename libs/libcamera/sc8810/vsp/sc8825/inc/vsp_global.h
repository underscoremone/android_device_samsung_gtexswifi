/******************************************************************************
 ** File Name:      vsp_global.h	                                          *
 ** Author:         Xiaowei Luo                                               *
 ** DATE:           06/11/2009                                                *
 ** Copyright:      2009 Spreatrum, Incoporated. All Rights Reserved.         *
 ** Description:    														  *
 *****************************************************************************/
/******************************************************************************
 **                   Edit    History                                         *
 **---------------------------------------------------------------------------* 
 ** DATE          NAME            DESCRIPTION                                 * 
 ** 06/11/2009    Xiaowei Luo     Create.                                     *
 ** 27/09/2012    Leon Li      Modified.                                     *
 *****************************************************************************/
#ifndef _VSP_GLOBAL_H_
#define _VSP_GLOBAL_H_
/*----------------------------------------------------------------------------*
**                        Dependencies                                        *
**---------------------------------------------------------------------------*/

/**---------------------------------------------------------------------------*
**                        Compiler Flag                                       *
**---------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif 
#define DCAM_CLOCK_EN		0x20900000

#define DCAM_CLK_FREQUENCE	0x8b00000c

#define VSP_RESET_ADDR		0x20900210

/*vsp base address*/
#define VSP_CTRL_BASE		0x20c10000
#define VSP_DCAM_BASE		0x20c00000

#define VSP_GLB_REG_BASE	(VSP_CTRL_BASE + 0x0000)
#define VSP_GLB_REG_SIZE	0x1C

/*register offset*/
#define GLB_CFG0_OFF		0x00
#define GLB_CFG1_OFF		0x04
#define GLB_CTRL0_OFF		0x08
#define VSP_TST_OFF			0x0c
#define VSP_DBG_OFF			0x10
#define VSP_BSM_RST_OFF		0x14
#define VSP_TIME_OUT_VAL	0x18

#define GLB_CFG0_WOFF		0x00
#define GLB_CFG1_WOFF		0x01
#define GLB_CTRL0_WOFF		0x02
#define VSP_TST_WOFF			0x03
#define VSP_DBG_WOFF			0x04
#define VSP_BSM_RST_WOFF		0x05
#define VSP_TIME_OUT_VAL_WOFF		0x06


typedef struct ahb_soft_rest_tag
{
	volatile uint32 ahb_soft_reset;		//[31:12]: reserved
										//[11]: emc reset
										//[10]: usbd software reset
										//[9:5]: reserved
										//[4]: vsp software reset
										//[3]: ccir software reset
										//[2]: dcam software reset
										//[1]: dma software reset
										//[0]: reserved

}AHB_SOFT_RST_T;

typedef struct vsp_global_reg_tag
{
	volatile uint32 VSP_CFG0;		//[31:17]: reserved, moved to DCAM:VSP_TIME_OUT
									//[19]: TIME_OUT_ENA, Time out enable, only enable this mode under command queque execution
									//[18]: H264_MP_MODE, H264 Main profile flag, active high 
									//[17]: STREAM_LE, Bit stream is little endian mode, active high
									//[16]: rotation_ena   1: rotation   0: no rotation
									//[15]: DATA_LE, Data is little endian format, 0 - big endian, 1 - little endian
									//[14]: ENC_DEC, codec flag, 1-enc, 0-dec
									//[13]: MEA_EB, active high
									//[12]: DBK_EB, active high
									//[11]: RESERVED
									//[10:8]: STANDARD, VSP standard, 
									//		3'b000 - H.263, 3'b001 -  MPEG4, 3'b010 - JPEG, 3'b011 - FLV_V1, 3'b100-H.264	
									//		3'b101 - RV8, 3'b110 - RV9
									//[7]: ARB_EB, Arbitor enable. Active high
									//[6]: VLD_EB, active high
									//[5]: VLC_EB, vlc enable, active high
									//[4]: BSMW_EB, BSMW enable, active high
									//[3]: BSMR_EB, BSMR enable, active high
									//[2]: MCA_EB, MCA enable, active high
									//[1]: DCT_EB, DCT enable, active high
									//[0]: MBC_EB, MBC enable, active high
	
	volatile uint32 VSP_CFG1;		//[26£º24] MB_FORMAT,3'b000 - 4:2:0 format
									//					 3'b001 - 4:1:1 format
									//					 3'b010 - 4:4:4 format
									//					 3'b011 - 4:2:2 format
									//					 3'b100 - 4:0:0 format
									//[20:12]: MB_Y_MAX, Current MB ID in Y direction, Note: maximum is 256 MCU
									//[8:0]: MB_X_MAX, Current MB ID in X direction, Note: maximum is 256 MCU  

	/*Noted by Xiaowei, for supporting VGA size, the bitwidth of mb_x_id and mb_y_id should be modified from 5 to 6, xiaowei,20081229*/
	volatile uint32 VSP_CTRL0;		//[13:8]: MB_Y_ID, Current MB ID in Y direction, Only for MPEG4/H.263/ASP decoding, Note: maximum is 512 pixel
									//[5:0]: MB_X_ID, Current MB ID in X direction,	Only for MPEG4/H.263/ASP decoding. Note: maximum is 512 pixel

	volatile uint32 VSP_TST;		//[24]: VSP_BUSY
									//[21]: VLD_BUSY
									//[20]: VLC_BUSY
									//[19]: BSM_BUSY
									//[18]: MCA_BUSY
									//[17]: DCT_BUSY
									//[16]: MBIO_BUSY
									//[15:0]: VSP_DEBUG

	volatile uint32 VSP_DBG;		//[6]: AHB_IDLE, active high
									//[5]: VDB_RFIFO_EMP, AHBM RFIFO is empty, active high
									//[4]: VDB_WFIFO_FULL, AHBM WFIFO is full, active high
									//[3]: reserved
									//[2:0]: ARB_STATE, VSP VDB arbiter state.  
	
	volatile uint32 BSM_RST;		//[0]: BSM_RST, Write 1 to rest bsm module

	volatile uint32 TIME_OUT_VAL;		//[31: 0]: Time out value, only valid under command queue mode
	
}VSP_GLB_REG_T;


/**---------------------------------------------------------------------------*
**                         Compiler Flag                                      *
**---------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif 
/**---------------------------------------------------------------------------*/
// End 

#endif //_VSP_GLOBAL_H_

