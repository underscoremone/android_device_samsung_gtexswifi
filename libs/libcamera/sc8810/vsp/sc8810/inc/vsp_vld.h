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

#define VLD_CTL_WOFF						0x00
#define VLD_MPEG4_CFG0_WOFF				0x01
#define VLD_MPEG4_CFG1_WOFF				0x02
#define VLD_MPEG4_TL_DC_Y_WOFF			0x03
#define VLD_MPEG4_TL_DC_UV_WOFF			0x04
#define VLD_MPEG4_DC_Y10_WOFF				0x06
#define VLD_MPEG4_DC_Y32_WOFF				0x07
#define VLD_MPEG4_DC_UV_WOFF				0x08


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

//h264 vld
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

#define HVLD_CODED_DC_FLAG_OFFSET			0x3C
#define HVLD_CTX_BIN0_CAT0_OFFSET			0x40
#define HVLD_CTX_BIN0_CAT1_OFFSET			0x44
#define HVLD_CTX_BIN0_CAT2_OFFSET			0x48
#define HVLD_CTX_BIN0_CAT3_OFFSET			0x4C
#define HVLD_CTX_BIN0_CAT4_OFFSET			0x50
#define HVLD_CTX_BIN0_INC4_CAT0_3_OFFSET	0x54
#define HVLD_CTX_BIN0_INC4_CAT4_OFFSET		0x58
#define HVLD_CTX_BINOTH_CAT0_OFFSET			0x5C
#define HVLD_CTX_BINOTH_CAT1_OFFSET			0x60
#define HVLD_CTX_BINOTH_CAT2_OFFSET			0x64
#define HVLD_CTX_BINOTH_CAT3_OFFSET			0x68
#define HVLD_CTX_BINOTH_CAT4_OFFSET			0x6C
#define HVLD_CTX_BINOTH_INC4_OFFSET			0x70
#define HVLD_ARTHI_BS_STATE_OFFSET			0x74
#define HVLD_VDB_CFG_OFFSET					0x78

#define HVLD_CTRL_WOFF					0x00
#define HVLD_MB_INFO_WOFF					0x01
#define HVLD_TOP_NNZ_Y_WOFF				0x02
#define HVLD_LEFT_NNZ_Y_WOFF				0x03
#define HVLD_TL_NNZ_CB_WOFF				0x04
#define HVLD_TL_NNZ_CR_WOFF				0x05
#define HVLD_NNZ_BLK_0_WOFF				0x06
#define HVLD_NNZ_BLK_1_WOFF				0x07
#define HVLD_NNZ_BLK_2_WOFF				0x08
#define HVLD_NNZ_BLK_3_WOFF				0x09
#define HVLD_NNZ_BLK_4_WOFF				0x0A
#define HVLD_NNZ_BLK_5_WOFF				0x0B
#define HVLD_CBP_UV_WOFF					0x0C
#define HVLD_CBP_IQT_WOFF					0x0D
#define HVLD_CBP_MBC_WOFF					0x0E

#define HVLD_CODED_DC_FLAG_WOFF			0x0F
#define HVLD_CTX_BIN0_CAT0_WOFF			0x10
#define HVLD_CTX_BIN0_CAT1_WOFF			0x11
#define HVLD_CTX_BIN0_CAT2_WOFF			0x12
#define HVLD_CTX_BIN0_CAT3_WOFF			0x13
#define HVLD_CTX_BIN0_CAT4_WOFF			0x14
#define HVLD_CTX_BIN0_INC4_CAT0_3_WOFF	0x15
#define HVLD_CTX_BIN0_INC4_CAT4_WOFF		0x16
#define HVLD_CTX_BINOTH_CAT0_WOFF			0x17
#define HVLD_CTX_BINOTH_CAT1_WOFF			0x18
#define HVLD_CTX_BINOTH_CAT2_WOFF			0x19
#define HVLD_CTX_BINOTH_CAT3_WOFF			0x1A
#define HVLD_CTX_BINOTH_CAT4_WOFF		0x1B
#define HVLD_CTX_BINOTH_INC4_WOFF			0x1C
#define HVLD_ARTHI_BS_STATE_WOFF			0x1D
#define HVLD_VDB_CFG_WOFF					0x1E			

#define RVLD_MB_INFO_OFF					0x00
#define RVLD_INTRA_CBP0_ADDR_OFF			0x04
#define RVLD_INTER_CBP_ADDR_OFF				0x08
#define RVLD_INTRA_DSC4X4_ADDR_OFF			0x0c
#define RVLD_INTER_DSC4X4_ADDR_OFF			0x10
#define RVLD_MB_CTR_OFF						0x14
#define RVLD_MB_CBP_OFF						0x18
	
#define RVLD_MB_INFO_WOFF					0x00
#define RVLD_INTRA_CBP0_ADDR_WOFF			0x01
#define RVLD_INTER_CBP_ADDR_WOFF			0x02
#define RVLD_INTRA_DSC4X4_ADDR_WOFF			0x03
#define RVLD_INTER_DSC4X4_ADDR_WOFF			0x04
#define RVLD_MB_CTR_WOFF					0x05		
#define RVLD_MB_CBP_WOFF					0x06

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
											//[21:20]		left mb type
											//[23:22]		top mb type
											//[31]			is_cabac			
											
	
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

	//for cabac decoding
	volatile uint32		coded_dc_flag;		//[0]			coded_flag_y_dc_a    neighbour MB's dc info
											//[1]			coded_flag_u_dc_a
											//[2]			coded_flag_v_dc_a
											
											//[4]			coded_flag_y_dc_b
											//[5]			coded_flag_u_dc_b
											//[6]			coded_flag_v_dc_b

											//[8]			coded_dc_flag_y		current MB's dc info
											//[9]			coded_dc_flag_u
											//[10]			coded_dc_flag_v

	//context for bin0
	volatile uint32		ctx_bin0_cat0;		//[6:0]: context for inc0	6: mps, [5:0]: state 
											//[14:8]: context for inc1	6: mps, [5:0]: state 
											//[22:16]: context for inc2	6: mps, [5:0]: state 
											//[30:24]: context for inc3	6: mps, [5:0]: state 
		
	volatile uint32		ctx_bin0_cat1;		//same as cat0
	volatile uint32		ctx_bin0_cat2;		//same as cat0
	volatile uint32		ctx_bin0_cat3;		//same as cat0
	volatile uint32		ctx_bin0_cat4;		//same as cat0

	volatile uint32		ctx_bin0_inc4_cat0_3;		//[6:0]: for cat0
													//[14:8]: for cat1
													//[22:16]: for cat2
													//[30:24]: for cat3
	volatile uint32		ctx_bin0_inc4_cat4;		//[6:0]: for cat4

	//context for other bin 
	volatile uint32		ctx_binoth_cat0;		//[6:0]: context for inc0	6: mps, [5:0]: state 
												//[14:8]: context for inc1	6: mps, [5:0]: state 
												//[22:16]: context for inc2	6: mps, [5:0]: state 
												//[30:24]: context for inc3	6: mps, [5:0]: state 
	
	volatile uint32		ctx_binoth_cat1;		//same as cat0
	volatile uint32		ctx_binoth_cat2;		//same as cat0
	volatile uint32		ctx_binoth_cat3;		//same as cat0
	volatile uint32		ctx_binoth_cat4;		//same as cat0
	
	volatile uint32		ctx_binoth_inc4;		//[6:0]: for cat0
												//[14:8]: for cat1
												//[22:16]: for cat2
												//[30:24]: for cat4

	volatile uint32		arith_bs_state;			//[9:0]: offset of bs
												//[24:16]: range of bs

	volatile uint32		vdb_cfg;				//[31]: enable
												//[30:20]: data number(word unit)
												//[19:0]: address offset (word unit)
}VSP_HVLD_REG_T;

typedef struct vsp_rvld_reg_tag
{
	volatile uint32 mb_info;					//[1:0]: mb_type,	0: intra4x4,	1: intra16x16, 
												//					2: inter16x16	3: other type

	//note: cbp table for intra and inter is stored in external memory
	volatile uint32 intra_cbp0_addr;			//[19:0]: Intra_cbp0_addr, Address in sdram for intra cbp0
	volatile uint32 inter_cbp_addr;				//[19:0]: Inter_cbp0_addr, Address in sdram for inter cbp0

	//note: dsc4x4 table is realized by cache
	volatile uint32 intra_dsc4x4_addr;			//[19:0]: Intra_Dsc4x4_addr, Address in sdram for intra dsc4x4
	volatile uint32 inter_dsc4x4_addr;			//[19:0]: Inter_Dsc4x4_addr, Address in sdram for inter dsc4x4

	volatile uint32 mb_ctr;						//[0]: start pulse, MB vld start, write 1 to start vld
												//[1]: 0: done(idle),  1: busy
												//[11:8]: mb rvld status
												//[16]: write 1'b1 to reset max  register, when QP changed

	volatile uint32 cbp;						//[23:0] cbp
}VSP_RVLD_REG_T;

#define VSP_VLD_IsError	(((*(volatile uint32*)(VSP_VLD_REG_BASE + VLD_CTL_OFFSET) & 0x40000000) == 0x40000000) ? 1 : 0)

//H264 mb_type
#define H264_IPCM		0
#define H264_IMB16X16	1
#define H264_IMB4X4		2
#define H264_INTERMB	3

/**---------------------------------------------------------------------------*
**                         Compiler Flag                                      *
**---------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif 
/**---------------------------------------------------------------------------*/
// End 
#endif  //_VSP_VLD_H_
