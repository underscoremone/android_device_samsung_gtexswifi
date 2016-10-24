/******************************************************************************
 ** File Name:      vsp_dbk.h	                                              *
 ** Author:         Xiaowei Luo                                               *
 ** DATE:           06/11/2009                                                *
 ** Copyright:      2009 Spreatrum, Incoporated. All Rights Reserved.         *
 ** Description:    VSP MEA Module Driver									  *
 *****************************************************************************/
/******************************************************************************
 **                   Edit    History                                         *
 **---------------------------------------------------------------------------* 
 ** DATE          NAME            DESCRIPTION                                 * 
 ** 06/11/2009    Xiaowei Luo     Create.                                     *
 *****************************************************************************/
#ifndef _VSP_DBK_H_
#define _VSP_DBK_H_
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
#define VSP_DBK_REG_BASE		(VSP_CTRL_BASE + 0x1C00)
#define VSP_DBK_REG_SIZE		0x3c

#define DBK_CFG_OFF				0x00
#define DBK_SID_CFG_OFF			0x08
#define DBK_MCU_NUM_OFF			0x0C
#define DBK_VDB_BUF_ST_OFF		0x10
#define DBK_CTR0_OFF			0x14
#define DBK_CTR1_OFF			0x18
#define DBK_DEBUG_OFF			0x1C
#define HDBK_MB_INFO_OFF		0x20
#define HDBK_BS_H0_OFF			0x24
#define	HDBK_BS_H1_OFF			0x28
#define HDBK_BS_V0_OFF			0x2c
#define	HDBK_BS_V1_OFF			0x30
#define HDBK_PARS_OFF			0x34
#define HDBK_CFG_FINISH_OFF		0x38
	
typedef struct 
{
	volatile uint32 DBK_CFG;			//[3]:		H264_FMO_MODE, H.264 FMO mode enable, active high. 
										//[2]:		POST_FLT_ENA, Post filtering enable, active high. 
										//[1:0]:	DBK_RUN_MODE, DBK start mode	2'b10"  -- Free run mode; '2'b01'  -- Auto mode		'2'b00'  - Manual mode		

	volatile uint32 RESV0;

	volatile uint32 DBK_SID_CFG;		//[25:16]:	DBK_START_Y_ID
										//[9:0]:	DBK_START_X_ID

	volatile uint32 DBK_MCU_NUM;		//[15:0]:	DBK_MCU_NUM, MBC total MCU number

	volatile uint32 DBK_VDB_BUF_ST;		//[2]:		DBK_JPEG_END, JPEG picture decoding is completed.
										//[1]:		DBK_VDB_BUF1_FULL, VDB buffer1 is full, active high		
										//[0]:		DBK_VDB_BUF0_FULL, VDB buffer0 is full, active high	
	
	volatile uint32 DBK_CTRL0;			//[1]:		DBK_DONE, DBK done state only when under manual mode. Active high, "DBK_start" will clear this bit if it's '1'.
										//[0]:		DBK_START, DBK start only hen under manual mode.	Virtula register, write '1' to this bit, self clear.		

	volatile uint32 DBK_CTRL1;			//[0]:		DBK SW configuration is completed. Note: only used for free run mode to indicates HW to work. Once one picture codec is completed, this signal will be cleared by HW

	volatile uint32 DBK_DEBUG;			//[17]:		SLICE_IDLE, Slice level idle, active high
										//[16]:		FRM_IDLE, Frame level idle, active high
										//[14]:		DBUF_FULL, DBK data buffer full flag, active high
										//[13]:		DBUF_EMP, DBK data buffer empty flag, Active high
										//[12]:		DBUF_PTR, DBK data buffer current pointer
										//[10:8]:	VDBM_FSM_ST
										//[6:4]:	CTRL_FSM_ST
										//			DBK_CTRL FSM state
										//			3'h0: IDLE
										//			3'h1: BYPASS OUT
										//			3'h2: DBK_FLT
										//			3'h3: DBK_OUT
										//			3'h4: WAIT_MBC_DONE
										//			3'h5: LEFT_TRANS
										//			3'h6: WAIT_END
										//[2]:		DBK_VDB_REQ
										//[1]:		WFIFO_DBK_FULL
										//[0]:		RFIFO_DBK_EMP
									
	volatile uint32	HDBK_MB_INFO;		//[29:24]:	mb_x
										//[21:16]:	qp_top;
										//[13:8] :	qp_left;
										//[5:0]  :	qp_cur;

	volatile uint32	HDBK_BS_H0;			//[31:28]:	bs_h7
										//....
										//[15:12]:	bs_h3
										//[11:8]:	bs_h2
										//[7:4]:	bs_h1
										//[3:0]:	bs_h0;
								
	volatile uint32	HDBK_BS_H1;			//[31:28]:	bs_h15
										//....
										//[15:12]:	bs_h11
										//[11:8]:	bs_h10
										//[7:4]:	bs_h9
										//[3:0]:	bs_h8;

	volatile uint32	HDBK_BS_V0;			//[31:28]:	bs_v7
										//....
										//[15:12]:	bs_v3
										//[11:8]:	bs_v2
										//[7:4]:	bs_v1
										//[3:0]:	bs_v0;

	volatile uint32	HDBK_BS_V1;			//[31:28]:	bs_v15
										//....
										//[15:12]:	bs_v11
										//[11:8]:	bs_v10
										//[7:4]:	bs_v9
										//[3:0]:	bs_v8;
	
	volatile uint32	HDBK_PARS;			//[20:16]:	chroma_qp_index_offset [-12, 12]
										//12~8:		alpha_offset	[-12, 12]
										//4~0:		beta_offset	[-12, 12]

	volatile uint32 HDBK_CFG_FINISH;	//0: write 1 to indicate configure finished
	
}VSP_DBK_REG_T;

/*dbk run mode*/
typedef enum {
		DBK_RUN_MANUAL_MODE = 0, 
		DBK_RUN_FREE_MODE = 2 
		}DBK_RUN_MODE_E;	

#define VSP_DBK_IsJpegDecEnd (((*(volatile uint32*)(VSP_DBK_REG_BASE+DBK_VDB_BUF_ST_OFF) & 0x4) == 0x4) ? 1 : 0)
#define VSP_DBK_MCU_X	((*(volatile uint32*)(VSP_DBK_REG_BASE+DBK_SID_CFG_OFF)) & 0x000003ff)
#define VSP_DBK_MCU_Y	(((*(volatile uint32*)(VSP_DBK_REG_BASE + DBK_SID_CFG_OFF))>>16) & 0x000003ff)

/**---------------------------------------------------------------------------*
**                         Compiler Flag                                      *
**---------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif 
/**---------------------------------------------------------------------------*/
// End 
#endif //_VSP_DBK_H_
