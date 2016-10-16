/******************************************************************************
 ** File Name:      JpegEnc_bitstream.c                                            *
 ** Author:         yi.wang													  *
 ** DATE:           07/12/2007                                                *
 ** Copyright:      2007 Spreadtrum, Incoporated. All Rights Reserved.        *
 ** Description:    Initialize the encoder									  *
 ** Note:           None                                                      *
******************************************************************************/
/******************************************************************************
 **                        Edit History                                       *
 ** ------------------------------------------------------------------------- *
 ** DATE           NAME             DESCRIPTION                               *
 ** 07/12/2007     yi.wang	         Create.                                  *
******************************************************************************/
/*----------------------------------------------------------------------------*
**                        Dependencies                                        *
**---------------------------------------------------------------------------*/
#include "sc8800g_video_header.h"
/**---------------------------------------------------------------------------*
**                        Compiler Flag                                       *
**---------------------------------------------------------------------------*/
#ifdef   __cplusplus
    extern   "C" 
    {
#endif

#if defined(JPEG_ENC)
//////////////////////////////////////////////////////////////////////////

//used in JPEG encode.
#if _CMODEL_
LOCAL void JPEGFW_PutBits(uint32 val, uint32 nbits)
{
#if _CMODEL_
	write_nbits(val, nbits, 0);
#endif //_CMODEL_

	READ_REG_POLL(VSP_BSM_REG_BASE+BSM_READY_OFF, 1, 1, TIME_OUT_CLK, "BSM_READY: polling bsm rfifo ready");

	VSP_WRITE_REG(VSP_BSM_REG_BASE+BSM_CFG2_OFF, (nbits << 24) , "BSM_CFG2: configure write n bits");
	VSP_WRITE_REG(VSP_BSM_REG_BASE+BSM_WDATA_OFF, val, "BSM_WDATA: write val(n bits) to bitstream, auto-stuffing");
}
#else
__inline void JPEGFW_PutBits(uint32 val, uint32 nbits)
{
	READ_REG_POLL(VSP_BSM_REG_BASE+BSM_DEBUG_OFF, V_BIT_3, 0, TIME_OUT_CLK, "polling bsm fifo fifo depth < 8 words for gob header");

	VSP_WRITE_REG(VSP_BSM_REG_BASE+BSM_CFG2_OFF, (nbits << 24) , "BSM_CFG2: configure write n bits");
	VSP_WRITE_REG(VSP_BSM_REG_BASE+BSM_WDATA_OFF, val, "BSM_WDATA: write val(n bits) to bitstream, auto-stuffing");
}
#endif //_CMODEL_
/*****************************************************************************
**	Name : 			JPEG_PutC
**	Description:	Output CHAR
**	Author:			Yi.wang
**	Note:
*****************************************************************************/
PUBLIC void JPEGFW_PutC(uint8 ch)
{
#if _CMODEL_
	write_nbits(ch, 8, 0);
#endif //_CMODEL_

	READ_REG_POLL(VSP_BSM_REG_BASE+BSM_DEBUG_OFF, V_BIT_3, 0, TIME_OUT_CLK, "polling bsm fifo fifo depth < 8 words for gob header");

	VSP_WRITE_REG(VSP_BSM_REG_BASE+BSM_CFG2_OFF, (8<<24), "BSM_CFG2: configure 8 bit for writing");
	VSP_WRITE_REG(VSP_BSM_REG_BASE+BSM_WDATA_OFF, ch, "BSM_WDATA: configure the value to be written to bitstream");
}

/*****************************************************************************
**	Name : 			JPEG_PutW
**	Description:	Output WORD
**	Author:			Yi.wang
**	Note:
*****************************************************************************/
PUBLIC void JPEGFW_PutW(uint16 w)
{
#if _CMODEL_
	write_nbits(w, 16, 0);
#endif //_CMODEL_

	READ_REG_POLL(VSP_BSM_REG_BASE+BSM_DEBUG_OFF, V_BIT_3, 0, TIME_OUT_CLK, "polling bsm fifo fifo depth < 8 words for gob header");

	VSP_WRITE_REG(VSP_BSM_REG_BASE+BSM_CFG2_OFF, (16<<24), "BSM_CFG2: configure 16 bit for writing");
	VSP_WRITE_REG(VSP_BSM_REG_BASE+BSM_WDATA_OFF, w, "BSM_WDATA: configure the value to be written to bitstream");
}
PUBLIC void JPEGFW_PutW_II(uint16 w)
{
	uint16 tmp = ((w & 0xFF) << 8) | ((w >> 8) & 0xFF);
	JPEGFW_PutW(tmp);
}
PUBLIC void JPEGFW_PutBits32_II(uint32 val, uint32 nbits)
{
	uint32 tmp = (((val & 0xFF) << 24) | (((val >> 8) & 0xFF) << 16) | (((val >> 16) & 0xFF) << 8) | ((val >> 24) & 0xFF) );
	JPEGFW_PutBits(tmp, 32);
}

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
