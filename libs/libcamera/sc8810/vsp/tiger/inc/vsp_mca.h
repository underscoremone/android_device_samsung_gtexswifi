/******************************************************************************
 ** File Name:      vsp_mca.h	                                              *
 ** Author:         Xiaowei Luo                                               *
 ** DATE:           06/11/2009                                                *
 ** Copyright:      2009 Spreatrum, Incoporated. All Rights Reserved.         *
 ** Description:    VSP MCA Module Driver									  *
 *****************************************************************************/
/******************************************************************************
 **                   Edit    History                                         *
 **---------------------------------------------------------------------------* 
 ** DATE          NAME            DESCRIPTION                                 * 
 ** 06/11/2009    Xiaowei Luo     Create.                                     *
 *****************************************************************************/
#ifndef _VSP_MCA_H_
#define _VSP_MCA_H_
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
	
#define VSP_MCA_REG_BASE		(VSP_CTRL_BASE + 0x1400)
#define VSP_MCA_REG_SIZE		0x20

#define MCA_CFG_OFF				0x00
#define MCA_BLK_CBUF_OFF		0x04
#define MCA_MV_CBUF_OFF			0x08
#define MCA_DEBUG_OFF           0x1C

#define MCA_CFG_WOFF			0x00
#define MCA_BLK_CBUF_WOFF		0x01
#define MCA_MV_CBUF_WOFF		0x02
#define MCA_DEBUG_WOFF          0x07

#define MC_BLKSIZE_16x16		(3)
#define MC_BLKSIZE_8x8			(2)
#define MC_BLKSIZE_4x4			(1)
#define MC_BLKSIZE_2x2			(0)

typedef struct  
{
	volatile uint32 MCA_CFG;			//[1]: MCA_RND, Rounding Control when in MPEG4 mode
										//[0]: MCA_START, MCA start, active high, one pulse.

	volatile uint32 MCA_BLK_CBUF;		//[28]: REF_Y_END, The following MV command is for the last luma block of this MB
										//[27]: REF_BLK_END, The following MV command is for the last block of this MB
										//[26]: REF_BIR_BLK, Current block is a bidirectional reference block
										//[25~24]: REF_CMD_TYPE, The command type	
										//		2'b00: Y/U/V should share the following MV command
										//		2'b01: The following MV command is for Y
										//		2'b10: The following MV command is for U/V				
										//[23~20]: REF_BW_FM_ID, The related reference frame id for backward reference block	0~15 frame		
										//[19~16]: REF_FW_FM_ID, The related reference frame id for forward reference block		0~15 frame		
										//[15~12]: Reserved
										//[11:8]: REF_BLK_ID, Reference block id	4'h0 ~ 4h3		
										//[1~0]: REF_BLK_SIZE, Reference block size 
										//		2'b00: 2x2 size block	2'b01: 4x4 size block, 2'b10: 8x8 size block, 2'b11: 16x16 size block		
	
	volatile uint32 MCA_MV_CBUF;		//[26:16]: REF_Y_MV0, Motion vector in Y direction for reference block
										//[10:0]: REF_X_MV0, Motion vector in X direction for reference block

	volatile uint32 RSV[4];


	volatile uint32 MCA_DEBUG;			//[24]: MCA_STATUS, 0 - idle, 1 - busy
										//[18:16]: RD_FSM_ST, MCA_RD FSM state
										//[13]: MCA_VDB_REQ, the data transfer request to AHBM
										//[12]: RFIFO_MCA_EMP, AHBM RFIFO empty flag 
										//[9]: MV cmd buffer full
										//[8]: MV cmd buffer empty
										//[5]: Block cmd buffer full
										//[4]: Block cmd buffer empty
										//[2:0]: MCA FSM state
}VSP_MCA_REG_T;

/**---------------------------------------------------------------------------*
**                         Compiler Flag                                      *
**---------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif 
/**---------------------------------------------------------------------------*/
// End 
#endif //_VSP_MCA_H_