/******************************************************************************
** File Name:      JpegCodec_bufmgr.c                                             *
** Author:         yi.wang													  *
** DATE:           07/12/2007                                                 *
** Copyright:      2007 Spreadtrum, Incoporated. All Rights Reserved.         *
** Description:    Buffer management									  *
** Note:           None                                                       *
*******************************************************************************

  *******************************************************************************
  **                        Edit History                                      *
  ** -------------------------------------------------------------------------*
  ** DATE           NAME             DESCRIPTION                              *
  ** 07/12/2007     yi.wang	         Create.                                  *
******************************************************************************/
#include "jpegcodec_def.h"
//#include "jpegcodec_global.h"
//#include "vsp_drv_sc8800s4.h"
//#include "video_common.h"		//remove by shan.he
#include "sc8810_video_header.h"

PUBLIC void JPEG_HWSet_BSM_Buf_ReadOnly(uint8 buf_id)
{
	(*(volatile uint32*)(0x20c10400)) &= 0x3fffffff;
	(*(volatile uint32*)(0x20c10400)) |= (1 << (31-buf_id));
}

PUBLIC void JPEG_HWSet_BSM_Buf_WriteOnly(uint8 buf_id)
{
	//(*(volatile uint32*)(0x20c10400)) &= (0xffffffff - (1 << (31 - buf_id)));
	uint32_t  value = VSP_READ_REG(VSP_BSM_REG_BASE+BSM_CFG0_OFF, "");
	value &= (0xffffffff - (1 << (31 - buf_id)));	
	VSP_WRITE_REG(VSP_BSM_REG_BASE+BSM_CFG0_OFF, value, "BSM_CFG0_OFF: set which bsm buffer is valid now.");
}

//PUBLIC void JPEG_HWSet_MBIO_Buf_ReadOnly(uint8 buf_id)
//{
//	(*(volatile uint32*)(0x20c11c10)) &= (1 << buf_id);
//}

PUBLIC void JPEG_HWSet_DBK_Buf_WriteOnly(uint8 buf_id)
{
#if _CMODEL_ //for RTL simulation	
	VSP_WRITE_REG(VSP_DBK_REG_BASE+DBK_VDB_BUF_ST_OFF, 1<<buf_id, "DBK_VDB_BUF_ST: set which mbio buffer is valid now.");
#else
	(*(volatile uint32*)(0x20c11c10)) |= (1 << buf_id);
#endif
}

PUBLIC void JPEG_HWSet_MEA_Buf_ReadOnly(uint8 buf_id)
{
	//(*(volatile uint32*)(VSP_MEA_REG_BASE+MEA_VDB_BUF_ST_OFF)) = 1<<buf_id;
	VSP_WRITE_REG(VSP_MEA_REG_BASE+MEA_VDB_BUF_ST_OFF, 1<<buf_id, "MEA_VDB_BUF_ST_OFF: set which mea buffer is valid now.");
}

PUBLIC void		  JPEG_HWResetVSP(void)
{
	VSP_Reset();
}

PUBLIC BOOLEAN JPEG_HWWaitingEnd(void)
{
	if(VSP_READ_REG_POLL(VSP_AHBM_REG_BASE+AHBM_STS_OFFSET, V_BIT_0, 0, TIME_OUT_CLK, "polling AHB idle status"))
	{
		JPEG_TRACE("TIME OUT!\n");
		return FALSE;
	}

	return TRUE;
}

