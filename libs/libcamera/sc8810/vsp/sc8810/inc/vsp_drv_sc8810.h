/******************************************************************************
 ** File Name:      vsp_drv_sc8810.h                                         *
 ** Author:         Xiaowei Luo                                               *
 ** DATE:           01/07/2010                                                *
 ** Copyright:      2010 Spreatrum, Incoporated. All Rights Reserved.         *
 ** Description:    														  *
 *****************************************************************************/
/******************************************************************************
 **                   Edit    History                                         *
 **---------------------------------------------------------------------------* 
 ** DATE          NAME            DESCRIPTION                                 * 
 ** 06/11/2009    Xiaowei Luo     Create.                                     *
 *****************************************************************************/
#ifndef _VSP_DRV_SC8810_H_
#define _VSP_DRV_SC8810_H_
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
#include "sci_types.h"
#include "vsp_vld.h"
#include "vsp_global.h"
#include "vsp_ahbm.h"
#include "vsp_bsm.h"
#include "vsp_dcam.h"
#include "vsp_vlc.h"
#include "vsp_mca.h"
#include "vsp_mea.h"
#include "vsp_dbk.h"
#include "vsp_mbc.h"
#include "vsp_dct.h"
#include "vsp_global_define.h"

#ifdef _VSP_LINUX_
#define LOG_TAG "VSP"
#include <utils/Log.h>
#define  SCI_TRACE_LOW   ALOGE
#define SCI_MEMSET  memset
#define SCI_MEMCPY	memcpy
#define SCI_ASSERT(...) 
#define SCI_PASSERT(condition, format...)
#endif 
/*----------------------------------------------------------------------------*
**                            Macro Definitions                               *
**---------------------------------------------------------------------------*/

#define TIME_OUT_CLK			0xffff

#define IS_TIME_OUT				1
#define NOT_TIME_OUT			0

#define MID_SHIFT_BIT			29 //28	//shift bit of module id

extern uint32 *g_cmd_data_ptr;
extern uint32 *g_cmd_info_ptr;

typedef enum{
	VSP_GLB = 0,
	VSP_BSM,
	VSP_VLD,
	//VSP_VLC,
	VSP_RAM10,
	VSP_DCT,
	VSP_MCA,
	VSP_MBC,
	VSP_DBK
	}VSP_MODULE_ID;

/*standard*/
typedef enum {
		VSP_ITU_H263 = 0, 
		VSP_MPEG4,  
		VSP_JPEG,
		VSP_FLV_V1,
		VSP_H264,
		VSP_RV8,
		VSP_RV9
		}VIDEO_STD_E;

#define VSP_INTRA		0
#define VSP_INTER		1

#define VSP_STD_ZIGZAG		0
#define VSP_HOR_ZIGZAG		1
#define VSP_VER_ZIGZAG		2

#ifdef _VSP_LINUX_
extern uint32 g_vsp_Vaddr_base;
typedef int (*FunctionType_ResetVSP)(int fd);
PUBLIC void  VSP_reg_reset_callback(FunctionType_ResetVSP p_cb,int fd);
void  VSP_SetVirtualBaseAddr(uint32 vsp_Vaddr_base);
#endif


PUBLIC void VSP_Reset (void);
PUBLIC void configure_huff_tab (uint32 *pHuff_tab, int32 n);
PUBLIC void flush_unalign_bytes (int32 nbytes);
PUBLIC void open_vsp_iram (void);
PUBLIC void close_vsp_iram (void);


#if !defined(_VSP_)
PUBLIC void vsp_write_register(uint32 reg_addr, int32 value/*, char *pstring*/);
PUBLIC uint32 vsp_read_register (uint32 reg_addr/*, int8 *pString*/);
PUBLIC int32 vsp_read_reg_poll(uint32 addr, uint32 msk,uint32 exp_value, uint32 time/*, char *pstring*/);
#else
#ifdef _VSP_LINUX_
static inline void vsp_write_register(uint32 reg_addr, int32 value/*, char *pstring*/)
#else
PUBLIC __inline void vsp_write_register(uint32 reg_addr, int32 value/*, char *pstring*/)	
#endif
{
#ifdef _VSP_LINUX_
       //extern uint32 g_vsp_Vaddr_base;
	*(volatile uint32 *)(reg_addr-VSP_DCAM_BASE+g_vsp_Vaddr_base)  = (value);
#else
	*(volatile uint32 *)(reg_addr)  = (value);
#endif
}

#ifdef _VSP_LINUX_
static inline uint32 vsp_read_register(uint32 reg_addr/*, char *pstring*/)	
#else
PUBLIC __inline uint32 vsp_read_register(uint32 reg_addr/*, char *pstring*/)	
#endif
{
#ifdef _VSP_LINUX_
       //extern uint32 g_vsp_Vaddr_base;
	return (*(volatile uint32 *)(reg_addr-VSP_DCAM_BASE+g_vsp_Vaddr_base));
#else
	return (*(volatile uint32 *)(reg_addr));
#endif
}

#ifdef _VSP_LINUX_
static inline int32 vsp_read_reg_poll(uint32 reg_addr, uint32 msk,uint32 exp_value, uint32 time/*, char *pstring*/) 
#else
PUBLIC __inline int32 vsp_read_reg_poll(uint32 reg_addr, uint32 msk,uint32 exp_value, uint32 time/*, char *pstring*/)
#endif
{
	uint32 vsp_time_out_cnt = 0;
#ifdef _VSP_LINUX_
       //extern uint32 g_vsp_Vaddr_base;
	while ((  (*(volatile uint32*)(reg_addr-VSP_DCAM_BASE+g_vsp_Vaddr_base)) & msk) != exp_value)
	{
		if (vsp_time_out_cnt > time)
		{
			return 1;
		}
		vsp_time_out_cnt++;
	}
#else	
	while ((*(volatile uint32*)reg_addr & msk) != exp_value)
	{
		if (vsp_time_out_cnt > time)
		{
			return 1;
		}
		vsp_time_out_cnt++;
	}
#endif
	return 0;
}

#define VSP_WRITE_REG(reg_addr, value, pstring) vsp_write_register(reg_addr, value)
#define VSP_READ_REG(reg_addr, pstring)		vsp_read_register(reg_addr)
#define VSP_READ_REG_POLL(reg_addr, msk, exp_value, time, pstring) vsp_read_reg_poll(reg_addr, msk, exp_value, time)


#ifdef _VSP_LINUX_
static inline int32 READ_REG_MBC_ST0_REG(uint32 addr, uint32 msk,uint32 exp_value, char *pstring)
#else
PUBLIC __inline int32 READ_REG_MBC_ST0_REG(uint32 addr, uint32 msk,uint32 exp_value, char *pstring)
#endif
{
	int32 mbc_st0 = VSP_READ_REG(VSP_MBC_REG_BASE+MBC_ST0_OFF, "MBC_ST0: read regist");

	exp_value = mbc_st0 & msk;
	
	return exp_value;	
}

/**
stop vsp
**/
PUBLIC void Vsp_Stop();
#endif //!defined(_VSP_)


/**---------------------------------------------------------------------------*
**                         Compiler Flag                                      *
**---------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif 
/**---------------------------------------------------------------------------*/
// End 
#endif //_VSP_DRV_SC8810_H_
