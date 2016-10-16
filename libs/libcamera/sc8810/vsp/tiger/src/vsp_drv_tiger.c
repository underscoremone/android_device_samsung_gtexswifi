/******************************************************************************
 ** File Name:      vsp_drv.c                                                 *
 ** Author:         Xiaowei Luo                                               *
 ** DATE:           06/11/2009                                                *
 ** Copyright:      2009 Spreatrum, Incoporated. All Rights Reserved.         *
 ** Description:    VSP Driver												  *
 *****************************************************************************/
/******************************************************************************
 **                   Edit    History                                         *
 **---------------------------------------------------------------------------* 
 ** DATE          NAME            DESCRIPTION                                 * 
 ** 06/11/2009    Xiaowei Luo     Create.                                     *
 *****************************************************************************/

/*----------------------------------------------------------------------------*
**                        Dependencies                                        *
**---------------------------------------------------------------------------*/
#include "sci_types.h"
#include "vsp_drv_sc8810.h"
#if !defined(_VSP_)
#include "common_global.h"
#include "bsm_global.h"
#endif //_CMODEL_

/**---------------------------------------------------------------------------*
**                        Compiler Flag                                       *
**---------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C"
{
#endif 
#ifdef _VSP_LINUX_

uint32 g_vsp_Vaddr_base = 0;
int g_vsp_dev_fd = 0;
FunctionType_ResetVSP ResetVSP_cb=NULL;
PUBLIC void  VSP_SetVirtualBaseAddr(uint32 vsp_Vaddr_base)
{	
	g_vsp_Vaddr_base = vsp_Vaddr_base;
}
PUBLIC void  VSP_reg_reset_callback(FunctionType_ResetVSP cb,int fd)
{
	ResetVSP_cb = cb;
	g_vsp_dev_fd = fd;
}

#endif


#define VSP_96MHz	0x00
#define VSP_64MHz	0x01
#define VSP_48MHz	0x10
#define VSP_26MHz	0x11

/************************************************************************/
/* Reset HW                                                             */
/************************************************************************/
PUBLIC void  VSP_Reset(void)
{
#ifdef _VSP_LINUX_
	if(ResetVSP_cb)
		(*ResetVSP_cb)(g_vsp_dev_fd);
#else
	uint32 cmd = 0;
	
	cmd = VSP_READ_REG(DCAM_CLOCK_EN, "DCAM_CLOCK_EN: Read the dcam clock");
	VSP_WRITE_REG(DCAM_CLOCK_EN, cmd|(1<<13), "DCAM_CLOCK_EN: enable dcam clock");
	
	//dont set DCAM_CLK_FREQUENCE on FPGA board. @xiaowei.luo,20090225
	//VSP_WRITE_REG (DCAM_CLK_FREQUENCE, 0x2003, "configure dcam clock to 80 MHz");
	
	/*reset dcam and vsp*/
	cmd = VSP_READ_REG(VSP_RESET_ADDR, "VSP_RESET_ADDR: Read the vsp reset");
	VSP_WRITE_REG(VSP_RESET_ADDR, cmd | (/*(1<<2) |*/ (1<<15)), "VSP_RESET_ADDR: only reset vsp, don't reset dcam");
	VSP_WRITE_REG(VSP_RESET_ADDR, cmd | (/*(0<<2) |*/ (0<<15)), "VSP_RESET_ADDR: only reset vsp, don't reset dcam");

	cmd = VSP_READ_REG(0x8b000070, "");
	cmd &= ~0xc;
	cmd |= (VSP_64MHz<<2);
	VSP_WRITE_REG(0x8b000070, cmd, "vsp: 64MHz");
#endif		
	//for little endian system
#ifdef CHIP_ENDIAN_LITTLE
	VSP_WRITE_REG(VSP_AHBM_REG_BASE+AHBM_ENDAIN_SEL_OFFSET, 0x5, "ENDAIN_SEL: 0x5 for little endian system");
#else
	VSP_WRITE_REG(VSP_AHBM_REG_BASE+AHBM_ENDAIN_SEL_OFFSET, 0x0, "ENDAIN_SEL: 0x5 for big endian system");
#endif


}

/*only generate firmware command*/
PUBLIC void flush_unalign_bytes(int32 nbytes)
{
	int i = 0;
	uint32 cmd = 0;
	
	cmd = (8<<24) | 1;
	
	for (i = 0; i < nbytes; i++)
	{
		if(VSP_READ_REG_POLL(VSP_BSM_REG_BASE+BSM_DEBUG_OFF, 1<<3, 1<<3, TIME_OUT_CLK,
			"polling bsm fifo fifo depth >= 8 words for gob header"))
		{
			return;
		}
		
		VSP_WRITE_REG(VSP_BSM_REG_BASE+BSM_CFG2_OFF, cmd, "BSM_CFG2: flush one byte");	
	}
}

//allow software to access the vsp buffer
PUBLIC void open_vsp_iram (void)
{
	uint32 cmd;
	
	cmd = (1<<4) | (1<<3);		
	VSP_WRITE_REG(VSP_DCAM_REG_BASE+DCAM_CFG_OFF, cmd, "DCAM_CFG: configure DCAM register");
	
	VSP_READ_REG_POLL(VSP_DCAM_REG_BASE+DCAM_CFG_OFF, 1<<7, 1<<7, TIME_OUT_CLK, "DCAM_CFG: polling dcam clock status");
}

//allow hardware to access the vsp buffer
PUBLIC void close_vsp_iram (void)
{
	uint32 cmd;
	
//	cmd = (0<<4) | (1<<3);	
	cmd = (cmd & ~0x10) | (1 << 3);
	VSP_WRITE_REG(VSP_DCAM_REG_BASE+DCAM_CFG_OFF, cmd, "DCAM_CFG: configure DCAM register");
	
	VSP_READ_REG_POLL (VSP_DCAM_REG_BASE+DCAM_CFG_OFF, 0, 0, TIME_OUT_CLK, "DCAM_CFG: polling dcam clock status");
}

/**
configure the huffman table
**/
PUBLIC void configure_huff_tab(uint32 *pHuff_tab, int32 n)
{
	int i = 0;
	uint32 cmd = 0;
	uint32 val = 0;
	
	open_vsp_iram();

	for(i = 0; i < n; i++)
	{
		val = *pHuff_tab++;
		
		VSP_WRITE_REG(HUFFMAN_TBL_ADDR+i*4, val, "HUFFMAN_TBL_ADDR: configure vlc table");
	}

	close_vsp_iram();
}

/**
stop vsp
**/
PUBLIC void Vsp_Stop()
{
	uint32 cmd = 0;
	
	/*reset dcam and vsp*/
	cmd = VSP_READ_REG(VSP_RESET_ADDR, "VSP_RESET_ADDR: Read the vsp reset");
	VSP_WRITE_REG(VSP_RESET_ADDR, cmd | (/*(1<<2) |*/ (1<<15)), "VSP_RESET_ADDR: only reset vsp, don't reset dcam");
	VSP_WRITE_REG(VSP_RESET_ADDR, cmd | (/*(0<<2) |*/ (0<<15)), "VSP_RESET_ADDR: only reset vsp, don't reset dcam");
}
/**---------------------------------------------------------------------------*
**                         Compiler Flag                                      *
**---------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif 
/**---------------------------------------------------------------------------*/
// End 
