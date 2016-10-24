/******************************************************************************
 ** File Name:      vsp_bsmr.h	                                              *
 ** Author:         Xiaowei Luo                                               *
 ** DATE:           11/20/2007                                                *
 ** Copyright:      2007 Spreatrum, Incoporated. All Rights Reserved.         *
 ** Description:    VSP bsm Driver for video codec.	  						  *
 *****************************************************************************/
/******************************************************************************
 **                   Edit    History                                         *
 **---------------------------------------------------------------------------* 
 ** DATE          NAME            DESCRIPTION                                 * 
 ** 11/20/2007    Xiaowei Luo     Create.                                     *
 *****************************************************************************/
#ifndef _VSP_BSMR_H_
#define _VSP_BSMR_H_
/*----------------------------------------------------------------------------*
**                        Dependencies                                        *
**---------------------------------------------------------------------------*/
#include "vsp_global.h"
/**---------------------------------------------------------------------------*
**                        Compiler Flag                                       *
**---------------------------------------------------------------------------*/
#ifdef   __cplusplus
    extern   "C" 
    {
#endif

#define VSP_BSM_REG_BASE	(VSP_CTRL_BASE + 0x400)
#define VSP_BSM_REG_SIZE	0x2C

#define BSM_CFG0_OFF		0x00
#define BSM_CFG1_OFF		0x04
#define BSM_CFG2_OFF		0x08
#define BSM_WDATA_OFF		0x0c
#define BSM_RDATA_OFF		0x10
#define BSM_TOTAL_BITS_OFF	0x14
#define BSM_DEBUG_OFF		0x18
#define BSM_DEBUG1_OFF		0x1c
#define BSM_READY_OFF		0x20
#define BSM_GLO_OPT_OFF		0x24
#define BSM_GLO_RESULT_OFF	0x28

#define BSM_CFG0_WOFF		0x00
#define BSM_CFG1_WOFF		0x01
#define BSM_CFG2_WOFF		0x02
#define BSM_WDATA_WOFF		0x03
#define BSM_RDATA_WOFF		0x04
#define BSM_TOTAL_BITS_WOFF	0x05
#define BSM_DEBUG_WOFF		0x06
#define BSM_DEBUG1_WOFF		0x07
#define BSM_READY_WOFF		0x08
#define BSM_GLO_OPT_WOFF		0x09
#define BSM_GLO_RESULT_WOFF	0x0A

typedef struct  bsmr_control_register_tag
{
	volatile uint32 BSM_CFG0;		//[31]: BUFF0_STATUS, 0: Buffer is ready for write 	1: Buffer is ready for read
									//[30]: BUFF1_STATUS, 0: Buffer is ready for write 	1: Buffer is ready for read
									//[19:0]: BUFFER_SIZE, Buffer size for bit stream, the unit is 1 words		

	volatile uint32 BSM_CFG1;		//[31]: DESTUFFING_EN, Destuffing function enable, active high
									//[30]: BSM_PS_START, Specific char sequence searching start , SW write this bit "1'b1"
									//[27:20]: BSM_PS_BYTE, The demanded specific data byte. The final specific char sequence is: FF "BSM_PS_BYTE"
									//[19:0]: BSM_OFFSET_ADDR, Start offset address in bit stream buffer , unit is word

	volatile uint32 BSM_CFG2;		//[29:24]: OPT_BITS, Number of bits to be flushed, only valid in decoding. The supported range is from 1 to 32 bits
									//[2]: COUNTER_CLR, Clear statistical counter
									//[1]: BSM_CLR, Move data remained in FIFO to external memory or discard data in FIFO
									//[0]: BSM_FLUSH, Remove n bit from bit stream buffer, only valid in decoding		

	volatile uint32 BSM_WDATA;		//[31:0]: BSM_WDATA, The data to be added to the bit stream


	volatile uint32 BSM_RDATA;		//[31:0]: BSM_RDATA, Current 32-bit bit stream in the capture window

	volatile uint32 TOTAL_BITS;		//[31:0]: TOTAL_BITS, The number of bits added to or remove from the bit stream

	volatile uint32 BSM_DEBUG;		//[31]: BSM_STATUS, BSM is active/inactive
									//[30:28]: BSM_STATE, BSM control status
									//[27]: DATA_TRAN, 0: bsm can be cleard,1, can not be cleared
									//[16:12]: BSM_SHIFT_REG, The bit amount has been shifted in BS shifter
									//[9:8]: DESTUFFING_LEFT_DCNT, The remained data amount in the de-stuffing module, uinit is word
									//[4]: PING-PONG_BUF_SEL, Current ping-pong buffer ID, buffer0 or buffer1
									//[3:0]: FIFO_DEPTH, BSM FIFO depth
	
	volatile uint32 BSM_DEBUG1;		//[1]: bsm_ahbm_req, The data transfer request to AHBM
									//[0]: rfifo_fsm_emp, AHBM RFIFO empty flag

	volatile uint32 BSM_RDY;		//[1]: BSM_PS_BUSY, Specific char sequence searching is in process. Active high
									//[0]: 0: SW can't access BSM internal FIFO
									//	1: SW is allowed to access BSM internal FIFO
									//Note: when SW will read/write the 
									//	  BSM FIFO (access the 0x08 register) this bit should be checked.

	volatile uint32 BSM_GLO_OPT;	//[1]: SE_V()
									//[0]: UE_V()

	volatile uint32 BSM_GLO_RESULT;								

}VSP_BSM_REG_T;

/**---------------------------------------------------------------------------*
**                         Compiler Flag                                      *
**---------------------------------------------------------------------------*/
#ifdef   __cplusplus
    }
#endif
/**---------------------------------------------------------------------------*/
// End 
#endif  //_VSP_BSMR_H_
