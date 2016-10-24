/******************************************************************************
 ** File Name:      vsp_dcam.h	                                              *
 ** Author:         Xiaowei Luo                                               *
 ** DATE:           06/11/2009                                                *
 ** Copyright:      2009 Spreatrum, Incoporated. All Rights Reserved.         *
 ** Description:    VSP DCAM Module Driver									  *
 *****************************************************************************/
/******************************************************************************
 **                   Edit    History                                         *
 **---------------------------------------------------------------------------* 
 ** DATE          NAME            DESCRIPTION                                 * 
 ** 06/11/2009    Xiaowei Luo     Create.                                     *
 *****************************************************************************/
#ifndef _VSP_DCAM_H_
#define _VSP_DCAM_H_
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
#define VSP_DCAM_REG_BASE					0x20c00000//(VSP_DCAM_BASE+0x0)
#define VSP_DCAM_REG_SIZE					0x30

#define DCAM_CFG_OFF						0x0
#define DCAM_SRC_SIZE_OFF					0xC
#define DCAM_ISP_DIS_SIZE_OFF				0x10
#define DCAM_VSP_TIME_OUT_OFF				0x14
#define DCAM_INT_STS_OFF					0x20
#define DCAM_INT_MASK_OFF					0x24
#define DCAM_INT_CLR_OFF					0x28
#define DCAM_INT_RAW_OFF					0x2C

typedef struct  
{
	volatile uint32 DCAM_CFG;				//[31:8]: RESERVED
											//[7]: dcam_clk_status, 0: dcam_clk, 1: hclk, software can access the buffer 
											//[4]: CLK_SWITCH, Control the access to share registers and VLD BUFFER 
																	///1:clk_dcam domain, software access all the vsp buffer
																	//0:HCLK domain, hardware access all the vsp buffer
											//[3]: VSP_EB, 1:enable VSP module	0:dcam clock domain	
											//[2]: DBK_BUF1_SWITCH, Switch the dbk_buf1 to software access. Active high.
											//[1]: DBK_BUF0_SWITCH, Switch the dbk_buf 0 to software access. Active high.
											//[0]: DCT_BUF_SWITCH, Switch the dbk_buf 0 to software access. Active high.
	
	volatile uint32 rsv1[2];

	volatile uint32 DCAM_SRC_SZIE;			//Source image size, i.e., input picture size 
											//[31:28]: RESERVED
											//[27:16]: SRC_SIZE_y, Source image Y size, maximum 4092
											//[15:12]: RESERVED
											//[11:0]: SRC_SIZE_X, Source image X size, maximum 4092

	volatile uint32 ISP_DIS_SIZE;			//DISPLAY image size, i.e., SCALER output size.  
											//[31:28]: RESERVED
											//[27:16]: DIS_SIZE_y, DISPLAY image Y size
											//[15:12]: RESERVED
											//[11:0]: DIS_SIZE_X, DISPLAY image X size
	volatile uint32 VSP_TIME_OUT;			//This register only active in vsp mode		
											//[31:18]: RESERVED
											//[17]: Time_out_clr, Write 1 to clear time_out_cnt.
											//[16]: Time_out_en, Time out monitor enable.
											//[15:0]: Time_out_value, The time out threshold vaule, unit is HCLK clock.

	volatile uint32 rsv2[2];

	volatile uint32 DCAM_INT_STS;	
											//[31:15]: RESERVED
											//[14]: VSP_MEA_DONE, only for JPEG encoding.
											//[13]: VSP_VLD_ERROR
											//[12]: VSP_TIMEOUT, Vsp timeout interrupt
											//[11]:JPEG_BUF_OVF, JPEG buffer on pSRAM is overflow
											//[9]:VSP_MBC_DONE, INT from VSP MBC module
											//[8]:VSP_VLC_DONE, INT from VSP VLC module		
											//[7]:VSP_BSM_DONE, INT from VSP BSM module

	volatile uint32 DCAM_INT_MASK;			
											//[31:16]: RESERVED
											//[15:0]: DCAM_INT_MASK, Bit[i] = 1: ISP interrupt is enabled for source i	Bit[i] = 0: ISP interrupt is disabled for source I
	volatile uint32 DCAM_INT_CLR;			
											//[31:16]: RESERVED
											//[15:0]: DCAM_INT_CLR, Write 1 into Bit[i] to clear bit[i] of ISP_INT_RAW register.
	volatile uint32 DCAM_INT_RAW;		
											//[31:16]: RESERVED
											//[15:0]: DCAM interrupt source raw bits		 		
}VSP_DCAM_REG_T;

/**---------------------------------------------------------------------------*
**                         Compiler Flag                                      *
**---------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif 
/**---------------------------------------------------------------------------*/
// End 
#endif//_VSP_DCAM_H_
