/******************************************************************************
 ** File Name:      JpegEnc_init.c                                            *
 ** Author:         yi.wang													  *
 ** DATE:           07/12/2007                                                *
 ** Copyright:      2007 Spreadtrum, Incoporated. All Rights Reserved.        *
 ** Description:    Initialize the encoder									  *
 ** Note:           None                                                      *
******************************************************************************/
/******************************************************************************
 **                        Edit History                                       *
 ** ------------------------------------------------------------------------- *
 ** DATE           NAME             DESCRIPTION                               *
 ** 07/12/2007     yi.wang	         Create.                                  *
******************************************************************************/
/*----------------------------------------------------------------------------*
**                        Dependencies                                        *
**---------------------------------------------------------------------------*/
#include "sc8810_video_header.h"

#if !defined(_SIMULATION_)
//#include "os_api.h"
#endif
/**---------------------------------------------------------------------------*
**                        Compiler Flag                                       *
**---------------------------------------------------------------------------*/
#ifdef   __cplusplus
    extern   "C" 
    {
#endif

#if defined(JPEG_ENC)
//////////////////////////////////////////////////////////////////////////
	#if defined(SMALL_SYS) && defined(_ARM_)

typedef enum
{
	SENSOR_SOF_ID = 0,
	SENSOR_EOF_ID,
	CAP_SOF_ID,
	CAP_EOF_ID,
	ISP_TX_DONE_ID,
	SC_DONE_ID,
	ISP_BUFFER_OVF_ID,
	VSP_BSM_DONE_ID,
	VSP_VLC_DONE_ID,
	VSP_MBIO_DOEN_ID,
	CAP_FIFO_DONE_ID,
	JPEG_BUF_OVF_ID,	
	VSP_TIMEOUT_ID,
	VSP_VLD_ERROR_ID,
	VSP_MEA_DONE_ID,
	ISP_INT_ID_MAX
}DCAM_MODULE_INT_ID_E;
	#endif


#if defined(_ARM_) && defined(SMALL_SYS)
extern void BSM_INT_PROC(void);
extern void MEA_INT_PROC(void);
extern void TIME_OUT_INT_PROC(void);
extern void VLC_DONE_INT_PROC(void); 
#endif

/************************************************************************/
/* configure the TOP register                                           */
/************************************************************************/
PUBLIC void JpegEnc_VspTopRegCfg(void)
{
	int32 cmd = 0;
	JPEG_CODEC_T *jpeg_fw_codec = Get_JPEGEncCodec();
	uint32 pTableAddr = (uint32)VSP_MEMO10_ADDR;
	uint32 int_mask = 0;
	uint32 endian_sel = 0;	
	
	SCI_ASSERT(jpeg_fw_codec != PNULL);

#if _CMODEL_
//	AllocMCUBuf();
//	enc_init_bitstream(jpeg_fw_codec);
	VSP_Init_CModel();
//	Generate_VlcTable();
#endif //_CMODEL_

#if 0
	VSP_Reset();
#else
	//backup the INT register for the VSP reset will clear it
	int_mask = VSP_READ_REG(VSP_DCAM_REG_BASE+DCAM_INT_MASK_OFF, "DCAM_INT_MASK: read  INT bit");

	JPEG_TRACE("[JpegEnc_VspTopRegCfg] int mask = 0x%x", int_mask);

	/*reset vsp*/
	VSP_Reset();
#endif

	cmd = (1<<3);
	VSP_WRITE_REG(VSP_DCAM_REG_BASE+DCAM_CFG_OFF, cmd, "DCAM_CFG: DCAM init");	
	
	//Source Size init
	cmd = ((jpeg_fw_codec->c_height & 0x0fff)<<16) | (jpeg_fw_codec->c_width & 0x0fff);
	VSP_WRITE_REG(VSP_DCAM_REG_BASE+DCAM_SRC_SIZE_OFF, cmd, "DCAM_DCAM_SRC_SZIE: configure DCAM Source size");

#if defined(_VSP_) && defined(SMALL_SYS)
	//enable dcam interrupt
	*(volatile uint32 *)0x80003008 |= (1<<23);	//In 8800G, the interrupt enable bit is BIT23. 
#endif

#if 0
	//INT Enable
	cmd =  (1 << 7) | (1 << 8) | (1 << 14);
	VSP_WRITE_REG(VSP_DCAM_REG_BASE+DCAM_INT_MASK_OFF, cmd, "DCAM_INT_MASK: enable related INT bit");
#else
	//restore the INT
	VSP_WRITE_REG(VSP_DCAM_REG_BASE+DCAM_INT_MASK_OFF, int_mask, "DCAM_INT_MASK: enable related INT bit");

	JPEG_TRACE("[JpegEnc_VspTopRegCfg] after reset, int mask = 0x%x", VSP_READ_REG(VSP_DCAM_REG_BASE+DCAM_INT_MASK_OFF, "DCAM_INT_MASK: read  INT bit"));	
#endif

#ifndef DCAM_VSP_LCD
	#if defined(SMALL_SYS) && defined(_ARM_)
		//enable dcam interrupt
		//*(volatile uint32 *)0x20a00010 |= (1<<27);
		//INT Enable
		//cmd =  (1 << 7) | (1 << 8) | (1 << 9);
			//init int
		DCAMMODULE_Init();
		VSP_WRITE_REG(VSP_DCAM_REG_BASE+DCAM_INT_MASK_OFF, cmd, "DCAM_INT_MASK: enable related INT bit");
		//register the int fun;

		DCAMMODULE_RegisterIntFunc(VSP_BSM_DONE_ID, BSM_INT_PROC);
		DCAMMODULE_RegisterIntFunc(VSP_MEA_DONE_ID, MEA_INT_PROC);
		DCAMMODULE_RegisterIntFunc(VSP_TIMEOUT_ID, TIME_OUT_INT_PROC);
		DCAMMODULE_RegisterIntFunc(VSP_VLC_DONE_ID, VLC_DONE_INT_PROC);	
	#endif
#endif

	open_vsp_iram();	
#if _CMODEL_
	//Source Buffer0 and Buffer1 Addr Init
	VSP_WRITE_REG(pTableAddr+ 16,  SRC_FRAME0_Y>>8, "Source Y0 Frame buffer ");
	VSP_WRITE_REG(pTableAddr+ 20, SRC_FRAME1_Y>>8, "Source Y1 Frame buffer ");

//	VSP_WRITE_REG(VSP_AHBM_REG_BASE+AHBM_BASE_ADDR_OFFSET, (uint32)/*jpeg_fw_codec->stream_0*/0>>26, "AHBM_BASE_ADDR: PSRAM base address offset");

	//encoded bitstream addr0 and addr1
	VSP_WRITE_REG(pTableAddr+ 8, BIT_STREAM_ENC_0>>8, "Encoded bit stream buffer0 ");
	VSP_WRITE_REG(pTableAddr+ 12, BIT_STREAM_ENC_1>>8, "Encoded bit stream buffer1 ");
#else
	//Source Buffer0 and Buffer1 Addr Init
	VSP_WRITE_REG(pTableAddr+ 16, (uint32)(jpeg_fw_codec->YUV_Info_0.y_data_ptr)>>8, "Source Y0 Frame buffer ");
	VSP_WRITE_REG(pTableAddr+ 20, (uint32)(jpeg_fw_codec->YUV_Info_1.y_data_ptr)>>8, "Source Y1 Frame buffer ");

//	VSP_WRITE_REG(VSP_AHBM_REG_BASE+AHBM_BASE_ADDR_OFFSET, (uint32)jpeg_fw_codec->stream_0>>26, "AHBM_BASE_ADDR: PSRAM base address offset");

	//encoded bitstream addr0 and addr1
	VSP_WRITE_REG(pTableAddr+ 8,  (uint32)(jpeg_fw_codec->stream_0)>>8, "Encoded bit stream buffer0 ");
	VSP_WRITE_REG(pTableAddr+ 12, (uint32)(jpeg_fw_codec->stream_1)>>8, "Encoded bit stream buffer1 ");
#endif //_CMODEL_
	close_vsp_iram();

	SCI_TRACE_LOW("stream0: %x, stream1: %x.\n", (uint32_t)jpeg_fw_codec->stream_0,(uint32_t)jpeg_fw_codec->stream_1);

	//VSP_CFG0
	cmd = (1 << 15) | (1 << 14)| (1 << 13) | (JPEG << 8) | (1 << 7) | (1 << 5) | (1 << 4) | (1 << 1) | (0 << 0);
	VSP_WRITE_REG(VSP_GLB_REG_BASE+GLB_CFG0_OFF, cmd, "GLB_CFG0: enable submodule and JPEG standard");
	
	//VSP_CFG1
	cmd = (jpeg_fw_codec->input_mcu_info << 24) | ((jpeg_fw_codec->mcu_num_y & 0x3ff) << 12) | (jpeg_fw_codec->mcu_num_x & 0x3ff);
	VSP_WRITE_REG(VSP_GLB_REG_BASE+GLB_CFG1_OFF, cmd, "VSP_CFG1: input mcu infor, mcu max number x and y");
	
	cmd = ((uint32)0xffff << 0) | (0 << 16);
	VSP_WRITE_REG(VSP_DCAM_REG_BASE+DCAM_VSP_TIME_OUT_OFF, cmd, "DCAM_VSP_TIME_OUT: disable hardware timer out");

	cmd = (jpeg_fw_codec->YUV_Info_0.v_data_ptr == NULL)?1:0;//1: uv_interleaved, two plane, 0: three plane
	cmd = (cmd<<8)|0xff;
	VSP_WRITE_REG(VSP_AHBM_REG_BASE+AHBM_BURST_GAP_OFFSET, cmd, "configure AHB register: BURST_GAP, and frame_mode");

	//now, for uv_interleaved
	cmd = ((jpeg_fw_codec->c_height>>jpeg_fw_codec->scale_factor) * (jpeg_fw_codec->c_width>>jpeg_fw_codec->scale_factor))>>2; //word unit

#ifdef CHIP_ENDIAN_LITTLE
	if (jpeg_fw_codec->YUV_Info_0.input_endian == 1) 
	{
		//little endian
		endian_sel = 0x5;
	}else
	{
		//big endian	
		endian_sel = 0x4;
	}
#endif	

	VSP_WRITE_REG(VSP_AHBM_REG_BASE+AHBM_ENDAIN_SEL_OFFSET, (cmd<<4)|endian_sel, "configure uv offset");
	
	//MEA
	VSP_WRITE_REG(VSP_MEA_REG_BASE+MEA_REF_CFG_OFF, (0), "MEA_REF_CFG: transfer_en disable");
	VSP_WRITE_REG(VSP_MEA_REG_BASE+MEA_CFG1_OFF, (1<<5), "MEA_CFG1: Use hardware pipeline");
	return;
}

PUBLIC void JpegEnc_VspTopUpdateYUVAddr(uint32 y_phy_addr,uint32_t u_phy_addr)
{
	JPEG_CODEC_T *jpeg_fw_codec = Get_JPEGEncCodec();
	uint32 pTableAddr = (uint32)VSP_MEMO10_ADDR;

	jpeg_fw_codec->YUV_Info_0.y_data_ptr = y_phy_addr;
	jpeg_fw_codec->YUV_Info_1.y_data_ptr = y_phy_addr;
	jpeg_fw_codec->YUV_Info_0.u_data_ptr = u_phy_addr;
	jpeg_fw_codec->YUV_Info_1.u_data_ptr = u_phy_addr;

	open_vsp_iram();

	//Source Buffer0 and Buffer1 Addr Init
	VSP_WRITE_REG(pTableAddr+ 16, (uint32)(jpeg_fw_codec->YUV_Info_0.y_data_ptr)>>8, "Source Y0 Frame buffer ");
	VSP_WRITE_REG(pTableAddr+ 20, (uint32)(jpeg_fw_codec->YUV_Info_1.y_data_ptr)>>8, "Source Y1 Frame buffer ");
	close_vsp_iram();
	SCI_TRACE_LOW("jpeg, update yu addr,0x%x,0x%x.\n",y_phy_addr,u_phy_addr);
}
/************************************************************************/
/* Enable sub module                                                    */
/************************************************************************/
PUBLIC void JpegEnc_VspSubModuleCfg(void)
{
	int32 cmd = 0;
	JPEG_CODEC_T *jpeg_fw_codec = Get_JPEGEncCodec();

	SCI_ASSERT(jpeg_fw_codec != PNULL);

	//BSM Module cfg
	cmd = (0<<31) | ((jpeg_fw_codec->pingpang_buf_len+3)>>2);
	VSP_WRITE_REG(VSP_BSM_REG_BASE+BSM_CFG0_OFF, cmd, "BSM_CFG0: buffer0 for write, and the max buffer size");
#if _CMODEL_
	g_bs_pingpang_bfr0 = jpeg_fw_codec->stream_0;
	g_bs_pingpang_bfr1 = jpeg_fw_codec->stream_1;
	init_bsm();
#endif	
	//VLC Module cfg, config total mcu number here, it will be modified for slice mode in JPEG_HWEncStart() function.
	cmd = ((jpeg_fw_codec->mcu_num_y * jpeg_fw_codec->mcu_num_x)  & 0x1ffff);
	VSP_WRITE_REG(VSP_VLC_REG_BASE+VLC_CFG_OFF, cmd, "VLC_CFG_OFF: total mcu number");
	
	//DCT Module cfg
	cmd = (DCT_QUANT_EN << 8) | (DCT_AUTO_MODE << 1) | (DCT_MODE);
	VSP_WRITE_REG(VSP_DCT_REG_BASE+DCT_CONFIG_OFF, cmd, "DCT_CONFIG: enable quant, auto-mode, dct-mode");
//	VSP_WRITE_REG(VSP_DCT_REG_BASE+DCT_CFG_FINISH_OFF, 1, "DCT_CFG_FINISH: config finished");
	
	//MBC Module cfg
	VSP_WRITE_REG(VSP_MBC_REG_BASE+MBC_CFG_OFF, (DOWN_SAMPLE_DIS << 8), "MBIO_CFG: disable down-sample, free-mode");

	//DBK Module cfg
	VSP_WRITE_REG(VSP_DBK_REG_BASE+DBK_CFG_OFF, (0<<2)|(DBK_RUN_FREE_MODE), "DBK_CFG: disable post-filter and free_run_mode");

	return;
}

PUBLIC void JpegEnc_QTableCfg(void)
{
	uint8 tbl_id = 0;
	JPEG_QUALITY_E level = 0;
	int32 qtable_addr = VSP_MEMO0_ADDR;
	int32 cmd = 0;
	JPEG_CODEC_T *jpeg_fw_codec = Get_JPEGEncCodec();
	
	SCI_ASSERT(jpeg_fw_codec != PNULL);

	jpeg_fw_codec->tbl_map[JPEG_FW_Y_ID].quant_tbl_id = JPEG_FW_LUM_ID;
	jpeg_fw_codec->tbl_map[JPEG_FW_U_ID].quant_tbl_id = JPEG_FW_CHR_ID;
	jpeg_fw_codec->tbl_map[JPEG_FW_V_ID].quant_tbl_id = JPEG_FW_CHR_ID;

	level = jpeg_fw_codec->compress_level;

	jpeg_fw_codec->quant_tbl[JPEG_FW_LUM_ID] = jpeg_fw_lum_quant_tbl_default[level];
	jpeg_fw_codec->quant_tbl[JPEG_FW_CHR_ID] = jpeg_fw_chr_quant_tbl_default[level];

	jpeg_fw_codec->quant_tbl_new[JPEG_FW_LUM_ID] = jpeg_fw_new_lum_quant_tbl_default[level];
	jpeg_fw_codec->quant_tbl_new[JPEG_FW_CHR_ID] = jpeg_fw_new_chr_quant_tbl_default[level];
		
	jpeg_fw_codec->quant_tbl_shift[JPEG_FW_LUM_ID] = jpeg_fw_new_lum_quant_tbl_default_shift[level];
	jpeg_fw_codec->quant_tbl_shift[JPEG_FW_CHR_ID] = jpeg_fw_new_chr_quant_tbl_default_shift[level];
	
	cmd = (1<<4) | (1<<3);		
	VSP_WRITE_REG(VSP_DCAM_REG_BASE+DCAM_CFG_OFF, cmd, "DCAM_CFG: allow software to access the vsp buffer");
		
	VSP_READ_REG_POLL(VSP_DCAM_REG_BASE+DCAM_CFG_OFF, 1<<7, 1<<7, TIME_OUT_CLK, "DCAM_CFG: polling dcam clock status");	
	for(tbl_id = 0; tbl_id < 2; tbl_id++)
	{
		int32 i;
		uint16 tmp1 = 0, tmp2 = 0;
		int32 tmp = 0;
		int32 index1 = 0;
		int32 index2 = 0;
		uint16 *quant_tbl_ptr = (uint16*)(&(jpeg_fw_codec->quant_tbl_new[tbl_id][0]));
		uint8 *quant_shift_tbl_ptr = (uint8*)(&(jpeg_fw_codec->quant_tbl_shift[tbl_id][0])); 
		const uint8 *ASIC_DCT_Matrix = (uint8*)jpeg_fw_ASIC_DCT_Matrix;

		for (i=0; i<64; i+=2)
		{
			index1 = ASIC_DCT_Matrix[i];
			index2 = ASIC_DCT_Matrix[i+1];		/*lint !e661*/
			tmp1 = (((quant_tbl_ptr[index1])<<4) | (uint16)(quant_shift_tbl_ptr[index1]-11));
			tmp2 = (((quant_tbl_ptr[index2])<<4) | (uint16)(quant_shift_tbl_ptr[index2]-11));
			tmp = (tmp2 << 16) | (tmp1 & 0xFFFF);
			
			VSP_WRITE_REG(qtable_addr, tmp, " VSP_MEMO0_ADDR: configure Q table");
			qtable_addr += 4;
		}
	}
	
	cmd = (0<<4) | (1<<3);	
	VSP_WRITE_REG(VSP_DCAM_REG_BASE+DCAM_CFG_OFF, cmd, "DCAM_CFG: allow hardware to access the vsp buffer");
		
	VSP_READ_REG_POLL(VSP_DCAM_REG_BASE+DCAM_CFG_OFF, 0, 0, TIME_OUT_CLK, "DCAM_CFG: polling dcam clock status");

	return;
}

/************************************************************************/
/* initialize encoder parameter using input parameter                                              */
/************************************************************************/
PUBLIC JPEG_RET_E JpegEnc_InitParam(JPEG_ENC_INPUT_PARA_T *input_para_ptr)
{
	int32 h_ratio_max, v_ratio_max;
	JPEG_CODEC_T *jpeg_fw_codec = Get_JPEGEncCodec();
	
	SCI_ASSERT(jpeg_fw_codec != PNULL);
	SCI_ASSERT(input_para_ptr != PNULL);

	//check the input parameter!
	if((input_para_ptr->yuv_0_info.input_mcu_info != JPEG_FW_YUV422) &&(input_para_ptr->yuv_0_info.input_mcu_info != JPEG_FW_YUV420) &&
		(input_para_ptr->yuv_1_info.input_mcu_info != JPEG_FW_YUV422) &&(input_para_ptr->yuv_1_info.input_mcu_info != JPEG_FW_YUV420) )
	{
		JPEG_TRACE("mcu information is not valid, only supported yuv422 or yuv420\n");
		return JPEG_FAILED;
	}
	if(jpeg_fw_codec->compress_level > JPEG_QUALITY_MAX)
	{
		JPEG_TRACE("Quant level is not valid, please set right value from [0,4]\n");
		return JPEG_FAILED;
	}
	if((input_para_ptr->width < 1)||(input_para_ptr->height < 1))
	{
		JPEG_TRACE("Too small image size!\n");
		return JPEG_FAILED;
	}
	
	SCI_MEMSET(jpeg_fw_codec, 0, (sizeof(JPEG_CODEC_T)));
	
	jpeg_fw_codec->RST_Count = M_RST0;
	jpeg_fw_codec->mea_bfr0_valid = TRUE;
	jpeg_fw_codec->mea_bfr1_valid = FALSE;

	//Load parameter into JPEG Codec  
	jpeg_fw_codec->work_mode = (uint8)input_para_ptr->work_mode;
	jpeg_fw_codec->is_first_slice = input_para_ptr->is_first_slice;
	jpeg_fw_codec->is_last_slice = input_para_ptr->is_last_slice;
	jpeg_fw_codec->input_mcu_info = (uint8)(input_para_ptr->yuv_0_info.input_mcu_info);
	jpeg_fw_codec->width = (uint16)input_para_ptr->width;
	jpeg_fw_codec->height = (uint16)input_para_ptr->height;
	jpeg_fw_codec->compress_level = input_para_ptr->quant_level;
	SCI_MEMCPY(&(jpeg_fw_codec->YUV_Info_0), &(input_para_ptr->yuv_0_info), (sizeof(YUV_FORMAT_T)));
	SCI_MEMCPY(&(jpeg_fw_codec->YUV_Info_1), &(input_para_ptr->yuv_1_info), (sizeof(YUV_FORMAT_T)));
	jpeg_fw_codec->stream_0 = input_para_ptr->stream_buf0;
	jpeg_fw_codec->stream_1 = input_para_ptr->stream_buf1;
	jpeg_fw_codec->pingpang_buf_len = input_para_ptr->bitstream_buf_len;
	//.....
	
	jpeg_fw_codec->dc_huff_tbl[JPEG_FW_LUM_ID].bits = &jpeg_fw_lum_dc_bits_default[0];
	jpeg_fw_codec->dc_huff_tbl[JPEG_FW_LUM_ID].huffval = &jpeg_fw_lum_dc_huffvalue_default[0];
	jpeg_fw_codec->ac_huff_tbl[JPEG_FW_LUM_ID].bits = &jpeg_fw_lum_ac_bits_default[0];
	jpeg_fw_codec->ac_huff_tbl[JPEG_FW_LUM_ID].huffval = &jpeg_fw_lum_ac_huffvalue_default[0];
	
	jpeg_fw_codec->dc_huff_tbl[JPEG_FW_CHR_ID].bits = &jpeg_fw_chr_dc_bits_default[0];
	jpeg_fw_codec->dc_huff_tbl[JPEG_FW_CHR_ID].huffval = &jpeg_fw_chr_dc_huffvalue_default[0];
	jpeg_fw_codec->ac_huff_tbl[JPEG_FW_CHR_ID].bits = &jpeg_fw_chr_ac_bits_default[0];
	jpeg_fw_codec->ac_huff_tbl[JPEG_FW_CHR_ID].huffval = &jpeg_fw_chr_ac_huffvalue_default[0];

	jpeg_fw_codec->restart_interval = input_para_ptr->restart_interval;
	jpeg_fw_codec->restart_to_go	= 0;
	jpeg_fw_codec->next_restart_num = 0;
	
	/*init sample ratio*/	
	switch(jpeg_fw_codec->input_mcu_info)
	{
	case JPEG_FW_YUV420:
		jpeg_fw_codec->ratio[JPEG_FW_Y_ID].h_ratio = 2;
		jpeg_fw_codec->ratio[JPEG_FW_Y_ID].v_ratio = 2;
		jpeg_fw_codec->ratio[JPEG_FW_U_ID].h_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_U_ID].v_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_V_ID].h_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_V_ID].v_ratio = 1;
		break;
	case JPEG_FW_YUV422:
		jpeg_fw_codec->ratio[JPEG_FW_Y_ID].h_ratio = 2;
		jpeg_fw_codec->ratio[JPEG_FW_Y_ID].v_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_U_ID].h_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_U_ID].v_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_V_ID].h_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_V_ID].v_ratio = 1;
		break;
	default:
		return JPEG_FAILED;
	}
	
	/*get the mcu size*/
	v_ratio_max = JPEG_FW_MAX3(jpeg_fw_codec->ratio[JPEG_FW_Y_ID].v_ratio, jpeg_fw_codec->ratio[JPEG_FW_U_ID].v_ratio, jpeg_fw_codec->ratio[JPEG_FW_V_ID].v_ratio);
	h_ratio_max = JPEG_FW_MAX3(jpeg_fw_codec->ratio[JPEG_FW_Y_ID].h_ratio, jpeg_fw_codec->ratio[JPEG_FW_U_ID].h_ratio, jpeg_fw_codec->ratio[JPEG_FW_V_ID].h_ratio);
	jpeg_fw_codec->mcu_height = 8 * v_ratio_max;
	jpeg_fw_codec->mcu_width = 8 * h_ratio_max;
	
	jpeg_fw_codec->mcu_num_x = (jpeg_fw_codec->width + jpeg_fw_codec->mcu_width -1)/jpeg_fw_codec->mcu_width;
	jpeg_fw_codec->mcu_num_y = (jpeg_fw_codec->height + jpeg_fw_codec->mcu_height -1)/jpeg_fw_codec->mcu_height;
	
	//Adjusted image width and height
	jpeg_fw_codec->c_width = jpeg_fw_codec->mcu_num_x * jpeg_fw_codec->mcu_width;
	jpeg_fw_codec->c_height = jpeg_fw_codec->mcu_num_y * jpeg_fw_codec->mcu_height;

		
	return JPEG_SUCCESS;
}

//////////////////////////////////////////////////////////////////////////
#endif //JPEG_ENC
/**---------------------------------------------------------------------------*
**                         Compiler Flag                                      *
**---------------------------------------------------------------------------*/
#ifdef   __cplusplus
    }
#endif
/**---------------------------------------------------------------------------*/
// End 
