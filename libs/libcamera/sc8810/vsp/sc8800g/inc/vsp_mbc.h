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
#define VSP_MBC_REG_SIZE			0x18

#define MBC_CFG_OFF					0x00
#define MBC_CMD0_OFF				0x04
#define MBC_CMD1_OFF				0x08
#define	MBC_CMD2_OFF				0x0C
#define	MBC_CMD3_OFF				0x10
#define MBC_ST0_OFF					0x14

typedef struct  
{
	volatile uint32 MBC_CFG;			//[13:12]:	DS_COEF, MCU down sample coefficients in X and Y direction,	2'b00: reserved,  2'b01: 1/2,  2'b10: 1/4,	2'b11: reserved
										//[8]:		DS_ENA, MCU down sample enable, only for JPEG decoding, active high

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
