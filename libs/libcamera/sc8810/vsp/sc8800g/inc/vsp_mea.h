/******************************************************************************
 ** File Name:      vsp_mea.h	                                              *
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
#ifndef _VSP_MEA_H_
#define _VSP_MEA_H_

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

/* MEA register map */
#define VSP_MEA_REG_BASE		(VSP_CTRL_BASE + 0x2000)
#define VSP_MEA_REG_SIZE		0x4C
	
#define MEA_CFG0_OFF			(0x00)
#define MEA_CFG1_OFF			(0x04)
#define MEA_CFG2_OFF			(0x08)
#define MEA_CFG3_OFF			(0x0C)
#define MEA_CFG4_OFF			(0x10)
#define MEA_CFG5_OFF			(0x14)
#define MEA_REF_CFG_OFF			(0x18)
#define MEA_REF_OFF				(0x1C)
#define MEA_MV_OFF				(0x20)
#define MEA_MV0_OFF				(0x24)
#define MEA_MV1_OFF				(0x28)
#define MEA_MV2_OFF				(0x2C)
#define MEA_MV3_OFF				(0x30)
#define MEA_UV_MV_OFF			(0x34)
#define MEA_CTRL_OFF			(0x38)
#define MEA_DEBUG_OFF			(0x3C)
#define MEA_INTRA_SAD_OFF		(0x40)
#define MEA_VDB_BUF_ST_OFF		(0x44)
#define MEA_MCU_NUM_OFF			(0x48)

#define MEA_RESULT_BIT	1

/*mea control register for MB*/
typedef struct mea_control_tag
{
	volatile uint32 MEA_CFG0;			//[31:25]: reserved
										//[24]: PRE_FLT_ENA, MEA source MB data pre-filtering enable, active high
										//[23:16]: PRE_FLT_TH, MEA pre-filter threshold value.
										//[12:8]: search_rangeY -- size of search window. maxium 63 pixels in vertical component.
										//[7:5]: reserved
										//[4:0]:search_rangeX -- size of search window. maxium 63 pixels in horizontal component.   
	
	volatile uint32 MEA_CFG1;			//[31:16]:mea_sad_thres -- the rhreshold of minimum sad value.
										//[15:8]: max_search_step -- max. search steps for ds algorithm
										//[7:6]: reserved
										//[5]: auto_en -- 0: mea controlled by sw; 1: used hardware pipeline
										//[4]: prediction_en -- nss search algorithm enable
										//[3]: mea_test -- 0: motion estimation will be started at(0,0), 1: start at the prediction point
										//[2]: 4mv_en -- 0: disalbe search in 8x8 mode; 1: enable 8x8 mode
										//[1]: intra_en -- 0: disable intra sad calculation; 1: enable intra sad calculation
										//[0]: me_enable -- 0: disable motion estimation; 1: enable motion estimaion

	volatile uint32 MEA_CFG2;			//[31:26]: reserved
										//[25:16]: mea_y_id -- the id of the current source mb in y direction
										//[9:0]: mea_x_id -- the id of the current source mb in x direction
      
	volatile uint32 MEA_CFG3;			//[31:16]: reserved
										//[15:9] mea_y_predmv -- the prediction motion vectorin y direction
										//[8]: reserved
										//[7:1]: mea_x_predmv -- the prediction motion vector in x direction
										//[0]: reserved

	volatile uint32 MEA_CFG4;			//[31:16]: increase_sad0 -- sad8 increased value
										//[15:0]: reduce_sad -- sad16(0,0) reduced value
      
	volatile uint32 MEA_CFG5;			//[31:16]: reserved
										//[15:0]: increase_sad -- intra sad increased value

	volatile uint32 REF_CFG;				//[31]: transfer_en -- transfer reference Y data enable
										//[30:20]: reserved
										//[19:16]: vdb_burst -- transfer width, the unit is word
										//[15:7]: reserved
										//[6:0]: vdb_line -- transfer height

	volatile uint32	REF_OFFSET;			//[31:29]: reserved
										//[28:16]: ref_addrY -- transfer top-left anddress x offset from picture top-left
										//[15:13]: reserved
										//[12:0]: ref_addrX -- transfer top-left anddress x offset from picture top-left, the unit is word
	
	volatile uint32 mea_result_mv_16x16;//[31:16]: mea_sad -- the initial SAD input and the output SAD
										//[15:8]: mea_y_mv -- the initial search motion vector in Y direction
										//[7:0]: mea_x_mv -- the initial search motion vector in X direction;
	volatile uint32 mea_result_mv_8x8[4];

	volatile uint32 MEA_UV_MV;				//[31:16]: reserved     
										//[15:8]:  mea_uv_y_mv -- u/v motion vector in y direction
										//[7:0]: mea_uv_x_mv -- u/v motion vector in x direction

	volatile uint32 MEA_CTL;			//[31]: mea_done -- 0: running; 1: idle
										//[30:3]: reserved
										//[2:1]: mea_result -- 00: intra mode, 01: 16x16 mode, 10: 8x8 mode
										//[0]: sw_ready -- 1: sw is ready for mea start, auto cleared by hw

	volatile uint32 MEA_DEBUG;			//[31:25]: reserved
										//[24]: mea_wflag
										//[23:18]: reserved//[17:16]: wea_wptr
										//[15:3]: reserved
										//[2:0]: mea_status	
	volatile uint32 INTRA_SAD;			//[31:16]: reserved
										//[15:0]: intra sad
	volatile uint32 MEA_VDB_BUF_ST;		//[31:3]: reserved
										//[2]: MEA_JPEG_END
										//[1]: MEA_VDB_BUF1_RDY
										//[0]: MEA_VDB_BUF0_RDY
	volatile uint32 MEA_MCU_NUM;		//[15:0]:	MEA_MCU_NUM, MEA total MCU number
}VSP_MEA_REG_T;



/**---------------------------------------------------------------------------*
**                         Compiler Flag                                      *
**---------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif 
/**---------------------------------------------------------------------------*/
// End 
#endif //_VSP_MEA_H_
