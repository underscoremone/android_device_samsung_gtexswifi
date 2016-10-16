/******************************************************************************
 ** File Name:      vsp_ahbm.h	                                              *
 ** Author:         Xiaowei Luo                                               *
 ** DATE:           06/11/2009                                                *
 ** Copyright:      2009 Spreatrum, Incoporated. All Rights Reserved.         *
 ** Description:    VSP AHBM Module Driver									  *
 *****************************************************************************/
/******************************************************************************
 **                   Edit    History                                         *
 **---------------------------------------------------------------------------* 
 ** DATE          NAME            DESCRIPTION                                 * 
 ** 06/11/2009    Xiaowei Luo     Create.                                     *
 *****************************************************************************/
#ifndef _VSP_AHBM_H_
#define _VSP_AHBM_H_
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

#define VSP_AHBM_REG_BASE				(VSP_DCAM_BASE+0x40)
#define VSP_AHBM_REG_SIZE				0x0c

#define AHBM_BURST_GAP_OFFSET			0x00
#define AHBM_ENDAIN_SEL_OFFSET			0x04
#define AHBM_STS_OFFSET					0x08

typedef struct vsp_ahbm_reg_tag 
{
	volatile uint32 AHBM_BURST_GAP;				//[7:0]: the count of interval which was inserted between two block transfer
												//[8]: Frame mode,	1'b0: Y, U/V two  planes	1'b1: Y, U, V three planes
	
	volatile uint32 ENDAIN_SEL;					//[23:4] Uv_offset_addr, UV plane offset address related to Y plane start address, Note: unit is word 
												//[3:2]: Endian_sel_wr: Endian adjust when read data from pSram.	
												//			00: original:{B0,B1,B2,B3}	
												//			01:{B3,B2,B1,B0}	
												//			10:{B2,B3,B0,B1}	
												//			11:reserved
												//[1:0]: Endian_sel_rd
												//			00: original:{B0,B1,B2,B3}	
												//			01:{B3,B2,B1,B0}	
												//			10:{B2,B3,B0,B1}	
												//			11:reserved

	volatile uint32 AHBM_STS;					//[0]: Ahbm_busy	[0]	R/W		1: busy; 0:idle
}VSP_AHBM_REG_T;

#define VSP_AHBM_IsBusy ((*(volatile uint32*)(VSP_AHBM_REG_BASE+AHBM_STS_OFFSET) & 0x1))

/****************************************************************************
description of these buffers:
1. MPEG4 decoding:
	FRM_ADDR_0: Reconstructed Y frame buffer
	FRM_ADDR_1: Reconstructed U frame buffer
	FRM_ADDR_2: Reconstructed V frame buffer

	FRM_ADDR_8: Forward reference Y0 Frame buffer
	FRM_ADDR_9: Forward reference U0 Frame buffer
	FRM_ADDR_10: Forward reference V0 Frame buffer

	FRM_ADDR_11: Forward reference Y1 Frame buffer
	FRM_ADDR_12: Forward reference U1 Frame buffer
	FRM_ADDR_13: Forward reference V1 Frame buffer

	FRM_ADDR_6: Source bit stream buffer for decoding

	FRM_ADDR_7: Top MB bottom data block first line IDCT data for AC/DC prediction

	FRM_ADDR_3: display Y frame buffer
	FRM_ADDR_4: display U frame buffer
	FRM_ADDR_5: display V frame buffer

2. MPEG4 encoding
	FRM_ADDR_8: Source Y frame buffer
	FRM_ADDR_9: Source U frame buffer
	FRM_ADDR_10: Source V frame buffer

	FRM_ADDR_3: Forward reference Y Frame buffer
	FRM_ADDR_4: Forward reference U Frame buffer
	FRM_ADDR_5: Forward reference V Frame buffer

	FRM_ADDR_6: Encoded bit stream buffer0

	FRM_ADDR_0: Reconstructed Y frame buffer 
	FRM_ADDR_1: Reconstructed U frame buffer 
	FRM_ADDR_2: Reconstructed V frame buffer 

3. JPEG decoding  
	FRM_ADDR_0: Reconstructed Y0 frame buffer
	FRM_ADDR_1: Reconstructed U0 frame buffer
	FRM_ADDR_2: Reconstructed V0 frame buffer

	FRM_ADDR_3: Reconstructed Y1 frame buffer
	FRM_ADDR_4: Reconstructed U1 frame buffer
	FRM_ADDR_5: Reconstructed V1 frame buffer

	FRM_ADDR_6: Source bit stream  buffer0 for decoding
	FRM_ADDR_7: Source bit stream  buffer1 for decoding

4. JPEG encoding
	FRM_ADDR_8: Source Y0 frame buffer
	FRM_ADDR_9: Source U0 frame buffer
	FRM_ADDR_10: Source V0 frame buffer

	FRM_ADDR_11: Source Y1 frame buffer
	FRM_ADDR_12: Source U1 frame buffer
	FRM_ADDR_13: Source V1 frame buffer

	FRM_ADDR_6: Encoded bit stream buffer0
	FRM_ADDR_7: Encoded bit stream buffer1
*/

/*define frame address and bitstream address, for RTL simulation*/
#if !defined(_VSP_)
	/*for MPEG4 decoding*/
	#define FRAME0_Y_ADDR			0x00400000	
	#define FRAME1_Y_ADDR			0x004c0000
	#define FRAME2_Y_ADDR			0x00580000

	#define	DISPLAY_FRAME0_Y_ADDR	0x00640000
	#define	DISPLAY_FRAME1_Y_ADDR	0x00700000

	#define DC_AC_PREDICTION_ADDR	0x00300000	

	#define SRC_BIT_STREAM			0x00000000

	/*for MPEG4 encoding*/
	#define REC_FRAME0_Y_ADDR		0x00EC0000	
	#define REC_FRAME1_Y_ADDR		0x00F00000

	#define SRC_FRAME_Y_ADDR		0x00000000

	#define ENC_BIT_STREAM			0x00F40000

	/*for JPEG encoding*/
	#define SRC_FRAME0_Y			0x00100000
	#define SRC_FRAME1_Y			0x00400000

	#define BIT_STREAM_ENC_0		0x00000000
	#define BIT_STREAM_ENC_1		0x00080000

	/*for JPEG decoding*/
	#define DEC_FRAME0_Y			0x00100000		
	#define DEC_FRAME1_Y			0x00400000

	#define BIT_STREAM_DEC_0		0x00000000
	#define BIT_STREAM_DEC_1		0x00080000

	/*for H.264 decoding*/
	#define H264DEC_FRAME0_Y_ADDR	0x00400000	
	#define H264DEC_FRAME1_Y_ADDR	0x00440000
	#define H264DEC_FRAME2_Y_ADDR	0x00480000
	#define H264DEC_FRAME3_Y_ADDR	0x004C0000	
	#define H264DEC_FRAME4_Y_ADDR	0x00500000
	#define H264DEC_FRAME5_Y_ADDR	0x00540000
	#define H264DEC_FRAME6_Y_ADDR	0x00580000	
	#define H264DEC_FRAME7_Y_ADDR	0x005C0000
	#define H264DEC_FRAME8_Y_ADDR	0x00600000
	#define H264DEC_FRAME9_Y_ADDR	0x00640000	
	#define H264DEC_FRAME10_Y_ADDR	0x00680000
	#define H264DEC_FRAME11_Y_ADDR	0x006C0000
	#define H264DEC_FRAME12_Y_ADDR	0x00700000	
	#define H264DEC_FRAME13_Y_ADDR	0x00740000
	#define H264DEC_FRAME14_Y_ADDR	0x00780000
	#define H264DEC_FRAME15_Y_ADDR	0x007C0000	
	#define H264DEC_FRAME16_Y_ADDR	0x00800000	

	#define H264DEC_SRC_BIT_STREAM	0x00000000

	#define INTRA_PREDICTION_ADDR	0x00300000	

#endif //!defined(_VSP_)

/**---------------------------------------------------------------------------*
**                         Compiler Flag                                      *
**---------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif 
/**---------------------------------------------------------------------------*/
// End 
#endif //_VSP_AHBM_H_
