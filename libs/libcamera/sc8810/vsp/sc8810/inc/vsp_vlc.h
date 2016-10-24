/******************************************************************************
 ** File Name:      vsp_vlc.h	                                              *
 ** Author:         Xiaowei Luo                                               *
 ** DATE:           06/11/2009                                                *
 ** Copyright:      2009 Spreatrum, Incoporated. All Rights Reserved.         *
 ** Description:    VSP VLC Module Driver									  *
 *****************************************************************************/
/******************************************************************************
 **                   Edit    History                                         *
 **---------------------------------------------------------------------------* 
 ** DATE          NAME            DESCRIPTION                                 * 
 ** 06/11/2009    Xiaowei Luo     Create.                                     *
 *****************************************************************************/
#ifndef _VSP_VLC_H_
#define _VSP_VLC_H_
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
#define VSP_VLC_REG_BASE	(VSP_CTRL_BASE + 0x0c00)
#define VSP_VLC_REG_SIZE	0xC

#define VLC_CFG_OFF			0x00
#define VLC_CTRL_OFF		0x04
#define VLC_ST_OFF			0x08

typedef struct  vsp_vlc_reg_tag
{
	
	volatile uint32 VLC_CFG0;		//reserved
									//[16:0]: TOTAL_MCU, The total MCU number of current encoding picture.
		
	volatile uint32 VLC_CTRL;		//[31:24]: MB_X_ID, Current MB ID in direction
									//[21:16]: QP_CUR, Current MB QP
									//[11]: LEFT_AVAIL
									//[10]: TOP_AVAIL
									//[9]: TL_AVAIL
									//[8]: VLC_MB_TYPE
									//[5:0]: VLC_MB_CBP, Bit[5] - Y0, Bit[4] - Y1, Bit[3] - Y2, Bit[2] - Y3, Bit[1] - U,Bit[0] - V

	volatile uint32 VLC_ST;			//[31]: VLC state: 0 ¨C idle, 1 - busy
									//[1]: VLC_START, Write 1 to start VLC. Note: only for video encoding for SW starts VLC every MB.
									//[0]: VLC_CLR, Write 1 to clear JPEG VLC module
}VSP_VLC_REG_T;

/**---------------------------------------------------------------------------*
**                         Compiler Flag                                      *
**---------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif 
/**---------------------------------------------------------------------------*/
// End 
#endif //_VSP_VLC_H_