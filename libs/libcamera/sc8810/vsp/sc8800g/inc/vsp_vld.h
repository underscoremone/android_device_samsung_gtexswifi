/******************************************************************************
 ** File Name:      vsp_vld.h	                                              *
 ** Author:         Xiaowei Luo                                               *
 ** DATE:           06/11/2009                                                *
 ** Copyright:      2009 Spreatrum, Incoporated. All Rights Reserved.         *
 ** Description:    VSP VLD Module Driver									  *
 *****************************************************************************/
/******************************************************************************
 **                   Edit    History                                         *
 **---------------------------------------------------------------------------* 
 ** DATE          NAME            DESCRIPTION                                 * 
 ** 06/11/2009    Xiaowei Luo     Create.                                     *
 *****************************************************************************/
#ifndef _VSP_VLD_H_
#define _VSP_VLD_H_
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
#define VSP_VLD_REG_BASE	(VSP_CTRL_BASE + 0x0800)
#define VSP_VLD_REG_SIZE	0x300

/*MPEG4*/
#define VLD_CTL_OFFSET						0x00
#define VLD_MPEG4_CFG0_OFFSET				0x04
#define VLD_MPEG4_CFG1_OFFSET				0x08
#define VLD_MPEG4_TL_DC_Y_OFFSET			0x0c
#define VLD_MPEG4_TL_DC_UV_OFFSET			0x10
#define VLD_MPEG4_DC_Y10_OFFSET				0x18
#define VLD_MPEG4_DC_Y32_OFFSET				0x1c
#define VLD_MPEG4_DC_UV_OFFSET				0x20

/*JPEG*/
#define VLD_JPEG_RESTART_MCU_CNT_OFFSET		0x70
#define VLD_JPEG_DC_Y_OFFSET				0x74
#define VLD_JPEG_DC_UV_OFFSET				0x78
#define VLD_JPEG_CFG0_OFFSET				0x7c
#define VLD_JPEG_RESTART_MCU_INTV_OFFSET	0x80
#define VLD_DC_VALID_OFFSET					0x84
#define VLD_AC_VALID_OFFSET					0x88
#define VLD_Luma_DC_LUT_OFFSET				0x8C
#define VLD_Chroma_DC_LUT_OFFSET			0xCC
#define VLD_Luma_AC_LUT_OFFSET				0x10C
#define VLD_Chroma_AC_LUT_OFFSET			0x14C

#define VLD_MPEG4_COEFF_WR_OFFSET			0x200

#define HVLD_CTRL_OFFSET					0x00
#define HVLD_MB_INFO_OFFSET					0x04
#define HVLD_TOP_NNZ_Y_OFFSET				0x08
#define HVLD_LEFT_NNZ_Y_OFFSET				0x0C
#define HVLD_TL_NNZ_CB_OFFSET				0x10
#define HVLD_TL_NNZ_CR_OFFSET				0x14
#define HVLD_NNZ_BLK_0_OFFSET				0x18
#define HVLD_NNZ_BLK_1_OFFSET				0x1C
#define HVLD_NNZ_BLK_2_OFFSET				0x20
#define HVLD_NNZ_BLK_3_OFFSET				0x24
#define HVLD_NNZ_BLK_4_OFFSET				0x28
#define HVLD_NNZ_BLK_5_OFFSET				0x2C
#define HVLD_CBP_UV_OFFSET					0x30
#define HVLD_CBP_IQT_OFFSET					0x34
#define HVLD_CBP_MBC_OFFSET					0x38

typedef struct vsp_vld_reg_tag
{
	volatile uint32 VLD_CTL;				//[31]:		VLD_STATUS 0: VLD is idle	1: VLD is busy
											//[30]:		VLD_ERROR, VLD has error, active high
											//[0]:		VLD_START, Start VLD, active high											

	volatile uint32 MPEG4_CFG0;				//[25]:		rvld or not, 1: is rvld, 0: not rvld 
											//[24]:		mb type, 0: intra  1: inter
											//[13:8]:	VLD_CBP, CBP
											//[2:0]:	BLOCK_ID, Current block ID, only active in intra mode		

	volatile uint32 MPEG4_CFG1;				//[28:24]	current mb qp value
											//[21]:		top mb is available
											//[20~16]:	top mb qp
											//[13]:		left mb is available
											//[12~8]:	left mb qp
											//[2]:		is_data_partition, active high
											//[1]:		dc coded as ac
											//[0]:		ac pred enable

	volatile uint32 MPEG4_TL_DC_Y;			//[15~0]:	Top left dc coefficient for Y
	volatile uint32 MPEG4_TL_DC_UV;			//[31~16]:	Top left dc coefficient for V
											//[15~0]:	Top left dc coefficient for U
	
	volatile uint32 rsv0;			

	volatile uint32 MPEG4_DC_Y10;			//[25~16]:  Y1 DC in data partition mode
											//[9~0]:   	Y0 DC in data partition mode

	volatile uint32 MPEG4_DC_Y32;			//[25~16]:  Y3 DC in data partition mode
											//[9~0]:   	Y2 DC in data partition mode

	volatile uint32 MPEG4_DC_UV;			//[25~16]:  V DC in data partition mode
											//[9~0]:  	U DC in data partition mode

	volatile uint32 rsv1 [19];

	/*jpeg configure register*/
	volatile uint32 JPEG_RESTART_MCU_CNT;	//[19:0]: 	Restart MCU counter
	volatile uint32 JPEG_DC_Y;				//[10:0]: 	JPEG_DC_Y,

	volatile uint32 JPEG_DC_UV;				//[26:16]: 	JPEG_DC_V,
											//[10:0]: 	JPEG_DC_U,

	volatile uint32 JPEG_CFG0;				//[19:0]: 	TOTAL_MCU 

	volatile uint32 JPEG_RESTART_MCU_INTV;	//[19:0] 	Restart MCU interval		
	
	volatile uint32 DC_VALID;				//[31:0]: 	[31]: Luma 1-bit DC valid	[30]: Luma 2-bit DC valid
											//[15]:		Chroma 1-bit DC valid		[0]: Chroma 16-bit DC valid
		

	volatile uint32 AC_VALID;				//[31:0]: 	[31]: Luma 1-bit AC valid	[30]: Luma 2-bit AC valid	
											//[15]: 	Chroma 1-bit AC valid		[0]: Chroma 16-bit AC valid
		
	volatile uint32 Luma_DC_LUT[16];		//[15:0]: 	luma DC max code
	
	volatile uint32 Chrom_DC_LUT[16];		//[15:0]: 	chroma DC max code

	volatile uint32 Luma_AC_LUT[16];		//[23:8]:	Luma AC max code
											//[7:0]: 	base address

	volatile uint32 Chroma_AC_LUT[16];		//[23:8]: 	AC max code
											//[7:0]: 	base address

	volatile uint32 rsv2[29];

}VSP_VLD_REG_T;

typedef struct vsp_hvld_reg_tag
{
	volatile uint32		hvld_ctr;			//[0]			start vld
											//[1]			vld_done
											//[2]			vld_error

	volatile uint32		mb_info;			//[1:0]			mb_type
											//[13:8]		cbp
											//[16]			left MB availability
											//[17]			top MB availability
	
	/*neighbor block's nnz configured by software*/
	volatile uint32		top_nnz_y;			//[28:24]:top0, [20:16]: top1, [12:8]: top2, [4:0]: top3	
	volatile uint32		left_nnz_y;			//[28:24]:left0, [20:16]: left1, [12:8]: left2, [4:0]: left3
	volatile uint32		tl_nnz_cb;			//[28:24]:top0, [20:16]: top1, [12:8]: left0, [4:0]: left1
	volatile uint32		tl_nnz_cr;			//[28:24]:top0, [20:16]: top1, [12:8]: left0, [4:0]: left1

	/*current mb's nnz for each 4x4 block*/
	volatile uint32		nnz_blk_0;			//[28:24]:block0, [20:16]: block1, [12:8]: block2, [4:0]: block3
	volatile uint32		nnz_blk_1;
	volatile uint32		nnz_blk_2;
	volatile uint32		nnz_blk_3;
	volatile uint32		nnz_blk_4;
	volatile uint32		nnz_blk_5;	
	volatile uint32		cbp_uv;				//[7:0]: bit0--blk0_u, bit1--blk1_u, ... bit7--blk3_v
	volatile uint32		cbp_iqt;			//[25:0]: cbp for iqt
	volatile uint32		cbp_mbc;			//[23:0]: cbp for mbc
}VSP_HVLD_REG_T;

#define VSP_VLD_IsError	(((*(volatile uint32*)(VSP_VLD_REG_BASE + VLD_CTL_OFFSET) & 0x40000000) == 0x40000000) ? 1 : 0)

//H264 mb_type
#define H264_IPCM		0
#define H264_IMB16X16	1
#define H264_OTHER_TYPE	2
/**---------------------------------------------------------------------------*
**                         Compiler Flag                                      *
**---------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif 
/**---------------------------------------------------------------------------*/
// End 
#endif  //_VSP_VLD_H_
