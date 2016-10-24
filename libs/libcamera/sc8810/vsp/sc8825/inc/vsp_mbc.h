/******************************************************************************
 ** File Name:      vsp_mbc.h	                                              *
 ** Author:         Xiaowei Luo                                               *
 ** DATE:           06/11/2009                                                *
 ** Copyright:      2009 Spreatrum, Incoporated. All Rights Reserved.         *
 ** Description:    VSP MBC Module Driver									  *
 *****************************************************************************/
/******************************************************************************
 **                   Edit    History                                         *
 **---------------------------------------------------------------------------* 
 ** DATE          NAME            DESCRIPTION                                 * 
 ** 06/11/2009    Xiaowei Luo     Create.                                     *
 *****************************************************************************/
#ifndef _VSP_MBC_H_
#define _VSP_MBC_H_
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
#define VSP_MBC_REG_BASE			(VSP_CTRL_BASE + 0x1800)
#define VSP_MBC_REG_SIZE			0x100

#define MBC_CFG_OFF					0x00
#define MBC_CMD0_OFF				0x04
#define MBC_CMD1_OFF				0x08
#define	MBC_CMD2_OFF				0x0C
#define	MBC_CMD3_OFF				0x10
#define MBC_ST0_OFF					0x14
#define MBC_RV_RATIO_CTRL_OFF		0x18
#define MBC_H264_BIR_INFO_OFF		0x1c
#define MBC_H264_WPBUF_SW_OFF		0x20
#define MBC_HY_weight128_OFF		0x24
#define MBC_HU_weight128_OFF		0x28
#define MBC_HV_weight128_OFF		0x2c
#define MBC_HY_WP_BLK_OFF			0x40
#define MBC_HU_WP_BLK_OFF			0x80
#define MBC_HV_WP_BLK_OFF			0xC0

#define MBC_CFG_WOFF					0x00
#define MBC_CMD0_WOFF				0x01
#define MBC_CMD1_WOFF				0x02
#define	MBC_CMD2_WOFF				0x03
#define	MBC_CMD3_WOFF				0x04
#define MBC_ST0_WOFF				0x05
#define MBC_RV_RATIO_CTRL_WOFF		0x06
#define MBC_H264_BIR_INFO_WOFF		0x07
#define MBC_H264_WPBUF_SW_WOFF      0x08
#define MBC_HY_weight128_WOFF       0x09
#define MBC_HU_weight128_WOFF       0x0A
#define MBC_HV_weight128_WOFF       0x0B
#define MBC_HY_WP_BLK0_WOFF         0x10
#define MBC_HY_WP_BLK1_WOFF         0x11
#define MBC_HY_WP_BLK2_WOFF         0x12
#define MBC_HY_WP_BLK3_WOFF         0x13
#define MBC_HY_WP_BLK4_WOFF         0x14
#define MBC_HY_WP_BLK5_WOFF         0x15
#define MBC_HY_WP_BLK6_WOFF         0x16
#define MBC_HY_WP_BLK7_WOFF         0x17
#define MBC_HY_WP_BLK8_WOFF         0x18
#define MBC_HY_WP_BLK9_WOFF         0x19
#define MBC_HY_WP_BLK10_WOFF         0x1A
#define MBC_HY_WP_BLK11_WOFF         0x1B
#define MBC_HY_WP_BLK12_WOFF         0x1C
#define MBC_HY_WP_BLK13_WOFF         0x1D
#define MBC_HY_WP_BLK14_WOFF         0x1E
#define MBC_HY_WP_BLK15_WOFF         0x1F
#define MBC_HU_WP_BLK0_WOFF          0x20
#define MBC_HU_WP_BLK1_WOFF          0x21
#define MBC_HU_WP_BLK2_WOFF          0x22
#define MBC_HU_WP_BLK3_WOFF          0x23
#define MBC_HU_WP_BLK4_WOFF          0x24
#define MBC_HU_WP_BLK5_WOFF          0x25
#define MBC_HU_WP_BLK6_WOFF          0x26
#define MBC_HU_WP_BLK7_WOFF          0x27
#define MBC_HU_WP_BLK8_WOFF          0x28
#define MBC_HU_WP_BLK9_WOFF          0x29
#define MBC_HU_WP_BLK10_WOFF          0x2A
#define MBC_HU_WP_BLK11_WOFF          0x2B
#define MBC_HU_WP_BLK12_WOFF          0x2C
#define MBC_HU_WP_BLK13_WOFF          0x2D
#define MBC_HU_WP_BLK14_WOFF          0x2E
#define MBC_HU_WP_BLK15_WOFF          0x2F

#define MBC_HV_WP_BLK0_WOFF          0x30
#define MBC_HV_WP_BLK1_WOFF          0x31
#define MBC_HV_WP_BLK2_WOFF          0x32
#define MBC_HV_WP_BLK3_WOFF          0x33
#define MBC_HV_WP_BLK4_WOFF          0x34
#define MBC_HV_WP_BLK5_WOFF          0x35
#define MBC_HV_WP_BLK6_WOFF          0x36
#define MBC_HV_WP_BLK7_WOFF          0x37
#define MBC_HV_WP_BLK8_WOFF          0x38
#define MBC_HV_WP_BLK9_WOFF          0x39
#define MBC_HV_WP_BLK10_WOFF          0x3A
#define MBC_HV_WP_BLK11_WOFF          0x3B
#define MBC_HV_WP_BLK12_WOFF          0x3C
#define MBC_HV_WP_BLK13_WOFF          0x3D
#define MBC_HV_WP_BLK14_WOFF          0x3E
#define MBC_HV_WP_BLK15_WOFF          0x3F

typedef struct  
{
	volatile uint32 MBC_CFG;			//[18]: H264_V_SAME, H264  weighting prediction Parameters for V block is same 
										//[17]: H264_U_SAME, H264  weighting prediction Parameters for U block is same 
										//[16]: H264_Y_SAME, H264  weighting prediction Parameters for Y block is same 
										//[13:12]:	DS_COEF, MCU down sample coefficients in X and Y direction,	2'b00: reserved,  2'b01: 1/2,  2'b10: 1/4,	2'b11: reserved
										//[8]:		DS_ENA, MCU down sample enable, only for JPEG decoding, active high
										//[6:4]:	H264_Y_SHIFT, Luma Log(WD)
										//[3:1]:	H264_UV_SHIFT, Chroma Log(WD)
										//[0]:		H264_WP_ENA, H264  weighting prediction enable, active high

	volatile uint32 MBC_CMD0;			//[30]:		MBC_MB_TYPE, MB TYPE	1-Inter MB		0-Intra MB
										//[29:28]   MBC_MB_MODE, 2'b00: i16x16, 2b'01: i4x4, 2'b10: I_PCM, 2'b11: SKIP mode
										//[27:24]   MBC_MB_AVAIL, bit27: top_left mb avail, bit26:left mb avail, bit25: top mb avail, bit24: top_right mb avail
										//[23:0]    MBC_MB_CBP, when MB 8x8 block CBP for IDCT result
										//			[5:0]:	Bit[0]-Y0	Bit[1]-Y1	Bit[2]-Y2	Bit[3]-Y3	Bit[4]-U	Bit[5]-V
										//			when MB 4x4 block cbp for iict result, block 23~0, bit0 indicate block0, in block reconstruction order

	volatile uint32 MBC_CMD1;			//luma mode0
										//[31:28]:	block0 intra prediction mode
										//[27:24]:	block1 intra prediction mode
										//[23:20]:	block2 intra prediction mode
										//[19:16]:	block3 intra prediction mode
										//[15:12]:	block4 intra prediction mode
										//[11:8]:	block5 intra prediction mode
										//[7:4]:	block6 intra prediction mode
										//[3:0]:	block7 intra prediction mode
										
	volatile uint32 MBC_CMD2;			//luma mode1
										//[31:28]:	block8 intra prediction mode
										//[27:24]:	block9 intra prediction mode
										//[23:20]:	block10 intra prediction mode
										//[19:16]:	block11 intra prediction mode
										//[15:12]:	block12 intra prediction mode
										//[11:8]:	block13 intra prediction mode
										//[7:4]:	block14 intra prediction mode
										//[3:0]:	block15 intra prediction mode

	volatile uint32 MBC_CMD3;			//chroma block ipred mode
										//[1:0]:	8x8 chroma ipred mode: 2'b00: mode0, 2'b01: mode1, 2'b10: mode2, 2'b11: mode3

	volatile uint32 MBC_ST0;			//[11]:		MBC_IDCT_FULL, MBC to IDCT buffer full flag, active high
										//[10]:		MBC_MCA_FULL, MBC to MCA buffer full flag, Active high
										//[9]:		DBK_MBC_FULL, DBK to MBC buffer full flag, Active high
										//[8]:		MBC_OBUF_PTR, MBC out data buffer current pointer
										//[6]:		MBC_EOB, Write this bit "1'b1" will generate one "mbc_eob" to DBK
										//[5]:		MBC_DONE, MBC one MB done flag, active high.
										//[4]:		MBC_RDY, MBC ready state, 1'b1: MBC is ready to work, 1'b0: MBC is  not ready 
										//[2:0]:	MBC_FSM

	volatile uint32 RV_RATIO_CTRL;		//[30:16]:		RV_RATIO1, Ratio Control parameter1
										//[14:0]:		RV_RATIO0, Ratio Control parameter0

	volatile uint32 H264_BIR_INFO;		//[3:0]: 8x8 Block Bi-direction info active high
										//		 Bit[0]  - block0 
										//		 Bit[1]  - block1
										//		 Bit[2]  - block2 
										//		 Bit[3]  - block3    

	volatile uint32 H264_WPBUF_SW;		//[0]: H264 weighting prediction parameter buffer switch 

	volatile uint32	H264_weight128[3];	//H264 weight is equal to 128 or not, 
										//idx 0: for Y bolck (4x4); idx 1: for U bolck (2x2); idx 2: for v bolck (2x2)
										//[31]: L1, 4x4blk id: 15
										//[16]: L1, 4x4blk id: 0
										//[15]: L0, 4x4blk id: 15
										//[0]:  L0, 4x4blk id: 0

	volatile uint32	rsv1[4];


	volatile uint32 HY_WP_BLK[16];		//H264 weighting prediction  control register for Y block (4x4)
										//[31:24]: HY_BLK0_O1, Rounding parameter1
										//[23:16]: HY_BLK0_O0, Rounding parameter0
										//[15:8]:  HY_BLK0_W1, Weighting parameter1
										//[7:0]:   HY_BLK0_W0, Weighting parameter0

	volatile uint32 HU_WP_BLK[16];		//H264 weighting prediction  control register for U block (4x4)
										//[31:24]: HU_BLK0_O1, Rounding parameter1
										//[23:16]: HU_BLK0_O0, Rounding parameter0
										//[15:8]:  HU_BLK0_W1, Weighting parameter1
										//[7:0]:   HU_BLK0_W0, Weighting parameter0

	volatile uint32 HV_WP_BLK[16];		//H264 weighting prediction  control register for V block (4x4)
										//[31:24]: HV_BLK0_O1, Rounding parameter1
										//[23:16]: HV_BLK0_O0, Rounding parameter0
										//[15:8]:  HV_BLK0_W1, Weighting parameter1
										//[7:0]:   HV_BLK0_W0, Weighting parameter0

}VSP_MBC_REG_T;


/*down sample*/
typedef enum {
		MBC_DOWN_SAMPLE_DIS = 0, 
		MBC_DOWN_SAMPLE_EN  
		}MBC_DS_ENABLE_E;	

//mbc mb_type
typedef enum
{
	MBC_INTRA_MB = 0,
	MBC_INTER_MB
}MBC_MB_TYPE_E;

//mbc mb_mode (h264)
typedef enum
{
	MBC_I16x16 = 0,
	MBC_I4x4,
	MBC_IPCM,
	MBC_SKIP
}MBC_MB_MODE_E;

/**---------------------------------------------------------------------------*
**                         Compiler Flag                                      *
**---------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif 
/**---------------------------------------------------------------------------*/
// End 
#endif //_VSP_MBC_H_
