/******************************************************************************
** File Name:      JpegDec_init.c                                             *
** Author:         yi.wang													  *
** DATE:           07/12/2007                                                 *
** Copyright:      2007 Spreadtrum, Incoporated. All Rights Reserved.         *
** Description:    Initialize the encoder									  *
** Note:           None                                                       *
*******************************************************************************/
/******************************************************************************
 **                        Edit History                                       *
 ** ------------------------------------------------------------------------- *
 ** DATE           NAME             DESCRIPTION                               *
 ** 07/12/2007     yi.wang	         Create.                                  *
******************************************************************************/
/*----------------------------------------------------------------------------*
**                        Dependencies                                        *
**---------------------------------------------------------------------------*/
#include "sc8800g_video_header.h"

#if !defined(_SIMULATION_)
#include "os_api.h"
#endif

/**---------------------------------------------------------------------------*
**                        Compiler Flag                                       *
**---------------------------------------------------------------------------*/
#ifdef   __cplusplus
    extern   "C" 
    {
#endif
#if defined(_VSP_) && defined(SMALL_SYS)

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
	ISP_INT_ID_MAX
}DCAM_MODULE_INT_ID_E;
#endif

#if defined(JPEG_DEC)
//////////////////////////////////////////////////////////////////////////

#if defined(_ARM_) && defined(SMALL_SYS)
extern void BSM_INT_PROC(void);
extern void MBIO_INT_PROC(void);
extern void TIME_OUT_INT_PROC(void);
extern void VLC_DONE_INT_PROC(void); 
extern void VLD_ERROR_INT_PROC(void);
#endif
/************************************************************************/
/* JPEG_HW_Cfg, Enable top reg                                          */
/************************************************************************/
PUBLIC void JpegDec_VspTopRegCfg(void)
{
	uint32 cmd = 0;
	JPEG_CODEC_T *jpeg_fw_codec = Get_JPEGDecCodec();
	uint32 pTableAddr = (uint32)VSP_MEMO10_ADDR;
	uint32 int_mask = 0;
	uint32 endian_sel = 0;

#if _CMODEL_
	AllocMCUBuf();
	VSP_Init_CModel();
#endif

#if 0
	/*reset vsp*/
	VSP_Reset();
#else
	//backup the INT mask register for the VSP reset will clear it
	int_mask = VSP_READ_REG(VSP_DCAM_REG_BASE+DCAM_INT_MASK_OFF, "DCAM_INT_MASK: read INT bit");

	JPEG_TRACE("[JpegDec_VspTopRegCfg] int mask = 0x%x", int_mask);

	/*reset vsp*/
	VSP_Reset();
#endif

	//DCAM init
	cmd = (1<<3);
	VSP_WRITE_REG(VSP_DCAM_REG_BASE+DCAM_CFG_OFF, cmd, "DCAM_CFG: DCAM init");	

#if defined(_VSP_) && defined(SMALL_SYS)
	//enable dcam interrupt
	*(volatile uint32 *)0x80003008 |= (1<<23);	//In 8800G, the interrupt enable bit is BIT23. (SC6600L: BIT26)
#endif

	//INT Enable
	if (!(jpeg_fw_codec->is_res_file))
	{
#if 0	
		cmd = (1 << 7) | (1 << 9) | (1 << 13);
		VSP_WRITE_REG(VSP_DCAM_REG_BASE+DCAM_INT_MASK_OFF, cmd, "DCAM_INT_MASK: enable related INT bit");
#else
		//restore the INT mask register
		VSP_WRITE_REG(VSP_DCAM_REG_BASE+DCAM_INT_MASK_OFF, int_mask, "DCAM_INT_MASK: enable related INT bit");
		JPEG_TRACE("[JpegDec_VspTopRegCfg] after reset, int mask = 0x%x", VSP_READ_REG(VSP_DCAM_REG_BASE+DCAM_INT_MASK_OFF, "DCAM_INT_MASK: read INT bit"));
#endif		
	}
	else
	{
		//clear all INTs while decoding SJPG
		int_mask = 0;
		VSP_WRITE_REG(VSP_DCAM_REG_BASE+DCAM_INT_MASK_OFF, int_mask, "DCAM_INT_MASK: enable related INT bit");
	}

	JPEG_TRACE("[JpegDec_VspTopRegCfg] after reset, int mask = 0x%x", VSP_READ_REG(VSP_DCAM_REG_BASE+DCAM_INT_MASK_OFF, "DCAM_INT_MASK: read INT bit"));
	
#if defined(_VSP_) && defined(SMALL_SYS)
	//init int
	DCAMMODULE_Init();

	//register the int fun;
	DCAMMODULE_RegisterIntFunc(VSP_BSM_DONE_ID, BSM_INT_PROC);
	DCAMMODULE_RegisterIntFunc(VSP_MBIO_DOEN_ID, MBIO_INT_PROC);
	DCAMMODULE_RegisterIntFunc(VSP_TIMEOUT_ID, TIME_OUT_INT_PROC);
	DCAMMODULE_RegisterIntFunc(VSP_VLD_ERROR_ID, VLD_ERROR_INT_PROC);
#endif

	open_vsp_iram();	
#if _CMODEL_
	VSP_WRITE_REG(pTableAddr+ 0, DEC_FRAME0_Y>>8, "Reconstructed Y0 frame buffer ");
	VSP_WRITE_REG(pTableAddr+ 4, DEC_FRAME1_Y>>8, "Reconstructed Y1 Frame buffer ");
	
	//decoded bitstream addr0 and addr1
	VSP_WRITE_REG(pTableAddr+ 8, BIT_STREAM_DEC_0>>8, "Decoded bit stream buffer0 ");
	VSP_WRITE_REG(pTableAddr+ 12, BIT_STREAM_DEC_1>>8, "Decoded bit stream buffer1 ");
#else
	VSP_WRITE_REG(pTableAddr+ 0, (uint32)(jpeg_fw_codec->YUV_Info_0.y_data_ptr)>>8, "Reconstructed Y0 frame buffer ");
	VSP_WRITE_REG(pTableAddr+ 4, (uint32)(jpeg_fw_codec->YUV_Info_1.y_data_ptr)>>8, "Reconstructed Y1 Frame buffer ");
	
// 	VSP_WRITE_REG (VSP_AHBM_REG_BASE+AHBM_BASE_ADDR_OFFSET, (uint32)jpeg_fw_codec->stream_0>>26, "AHBM_BASE_ADDR: PSRAM base address offset");

	//decoded bitstream addr0 and addr1
	VSP_WRITE_REG(pTableAddr+ 8, (uint32)(jpeg_fw_codec->stream_0)>>8, "Decoded bit stream buffer0 ");
	VSP_WRITE_REG(pTableAddr+ 12, (uint32)(jpeg_fw_codec->stream_1)>>8, "Decoded bit stream buffer1 ");
#endif//_CMODEL_

	close_vsp_iram();
	
	cmd = (1 << 15) | (0 << 14)| (1 << 12) | (JPEG << 8) | (1 << 7) | (1 << 6) | (1 << 3) | (1 << 1) | (1 << 0);
	VSP_WRITE_REG(VSP_GLB_REG_BASE+GLB_CFG0_OFF, cmd, "VSP_CFG0: enable submodule, JPEG standard");

	cmd = ((uint32)0xffff << 0) | (0 << 16);
	VSP_WRITE_REG(VSP_DCAM_REG_BASE+DCAM_VSP_TIME_OUT_OFF, cmd, "DCAM_VSP_TIME_OUT: disable hardware timer out");

//set the interval of mcu decoding
	cmd = (jpeg_fw_codec->YUV_Info_0.v_data_ptr == NULL)?1:0;//1: uv_interleaved, two plane, 0: three plane
	cmd = (cmd<<8)|0xff;
	VSP_WRITE_REG(VSP_AHBM_REG_BASE+AHBM_BURST_GAP_OFFSET, cmd, "configure AHB register: BURST_GAP, and frame_mode");

	//now, for uv_interleaved
	cmd = ((jpeg_fw_codec->c_height>>jpeg_fw_codec->scale_factor) * (jpeg_fw_codec->c_width>>jpeg_fw_codec->scale_factor))>>2; //word unit

#if defined(CHIP_ENDIAN_LITTLE)
	if (jpeg_fw_codec->YUV_Info_0.input_endian == 0)
	{
		//output big endian YUV
		//VSP_WRITE_REG(VSP_AHBM_REG_BASE+AHBM_ENDAIN_SEL_OFFSET, 0x5, "little endian");
		endian_sel = 0x5;
	}
	else
	{
		//output little endian YUV
		//VSP_WRITE_REG(VSP_AHBM_REG_BASE+AHBM_ENDAIN_SEL_OFFSET, 0x1, "little endian");	
		endian_sel = 0x1;
	}	
#endif	

	VSP_WRITE_REG(VSP_AHBM_REG_BASE+AHBM_ENDAIN_SEL_OFFSET, (cmd<<4)|endian_sel, "configure uv offset");

	return;
}

/************************************************************************/
/* Enable sub module                                                    */
/************************************************************************/
PUBLIC void JpegDec_VspSubModuleCfg(void)
{
	uint32 cmd = 0;
	uint32 down_sample_setting = DOWN_SAMPLE_DIS;
	JPEG_CODEC_T *jpeg_fw_codec = Get_JPEGDecCodec();
	
	//Source Size init
	cmd = (((jpeg_fw_codec->c_height>>jpeg_fw_codec->scale_factor) & 0x0fff)<<16) | ((jpeg_fw_codec->c_width>>jpeg_fw_codec->scale_factor) & 0x0fff);
	VSP_WRITE_REG(VSP_DCAM_REG_BASE+DCAM_SRC_SIZE_OFF, cmd, "DCAM_DCAM_SRC_SZIE: configure DCAM Source size");
		
	///1. enable de-stuffing
	///2. reconfigure current address of source bit stream buffer0, word align(the unit is word)
	VSP_WRITE_REG(VSP_BSM_REG_BASE+BSM_CFG1_OFF, (uint32)1<<31, "BSM_CFG1 enable de-stuffing, and reset the bitstream address"); /*lint !e569*/
	cmd = ((jpeg_fw_codec->pingpang_buf_len+3) >> 2);
	VSP_WRITE_REG(VSP_BSM_REG_BASE+BSM_CFG0_OFF, cmd, "BSM_CFG0: buffer0 for read, and the buffer size");
	
	//VLD Module cfg
	cmd = ((jpeg_fw_codec->mcu_num_y * jpeg_fw_codec->mcu_num_x)  & 0x3ffff );
	VSP_WRITE_REG(VSP_VLD_REG_BASE+VLD_JPEG_CFG0_OFFSET, cmd, "VLD_JPEG_CFG0: total mcu number.");
	
	//DCT Module cfg
	cmd = (((jpeg_fw_codec->input_mcu_info) & 0x7) << 9) | (DCT_QUANT_EN << 8) | (DCT_AUTO_MODE << 1) | (IDCT_MODE);
	VSP_WRITE_REG(VSP_DCT_REG_BASE+DCT_CONFIG_OFF, cmd, "DCT_CONFIG: mcu info, quant enable, auto-mode, idct-mode");
	VSP_WRITE_REG(VSP_DCT_REG_BASE+DCT_CFG_FINISH_OFF, 1, "DCT_CFG_FINISH: dct config finished");

	//MBC Module cfg
	if(JPEG_SCALING_DOWN_ZERO == jpeg_fw_codec->scale_factor)
	{
		down_sample_setting = DOWN_SAMPLE_DIS;
	}else
	{
		down_sample_setting = DOWN_SAMPLE_EN;
	}
	cmd = (jpeg_fw_codec->scale_factor << 12) | (down_sample_setting << 8);
	VSP_WRITE_REG(VSP_MBC_REG_BASE+MBC_CFG_OFF, cmd, "MBC_CFG: scale factor, down sample setting");

	//DBK Module cfg
	VSP_WRITE_REG(VSP_DBK_REG_BASE+DBK_CFG_OFF, (0<<2)|(DBK_RUN_FREE_MODE), "DBK_CFG: disable post-filter and free_run_mode");
}

/************************************************************************/
/* Decode Initiate                                                      */
/************************************************************************/
PUBLIC int32 JPEGFW_UpdateMiscFields(void)
{
	uint16 width = 0, height = 0;
	uint16 v_ratio_max = 0, h_ratio_max = 0;
	uint16 max_width = 4096, max_height = 0;
	uint16 mcu_width = 0, mcu_height = 0;
	uint32 max_mcu_num = 0x3FFFF;
	JPEG_CODEC_T *jpeg_fw_codec = Get_JPEGDecCodec();

	width = jpeg_fw_codec->width;
	height = jpeg_fw_codec->height;
	
	/*get the mcu size*/
	v_ratio_max = JPEG_FW_MAX3(jpeg_fw_codec->ratio[JPEG_FW_Y_ID].v_ratio, jpeg_fw_codec->ratio[JPEG_FW_U_ID].v_ratio, jpeg_fw_codec->ratio[JPEG_FW_V_ID].v_ratio);
	h_ratio_max = JPEG_FW_MAX3(jpeg_fw_codec->ratio[JPEG_FW_Y_ID].h_ratio, jpeg_fw_codec->ratio[JPEG_FW_U_ID].h_ratio, jpeg_fw_codec->ratio[JPEG_FW_V_ID].h_ratio);
		
	mcu_width = jpeg_fw_codec->mcu_width = 8 * h_ratio_max;
	mcu_height = jpeg_fw_codec->mcu_height = 8 * v_ratio_max;
	
	SCI_ASSERT(jpeg_fw_codec->mcu_height != 0);
	SCI_ASSERT(jpeg_fw_codec->mcu_width != 0);
	
	if((jpeg_fw_codec->mcu_height == 0) || (jpeg_fw_codec->mcu_width == 0))
	{
		return JPEG_FAILED;
	}
	jpeg_fw_codec->mcu_num_x = (width + jpeg_fw_codec->mcu_width -1)/jpeg_fw_codec->mcu_width;
	jpeg_fw_codec->mcu_num_y = (height + jpeg_fw_codec->mcu_height -1)/jpeg_fw_codec->mcu_height;

	jpeg_fw_codec->c_width = jpeg_fw_codec->mcu_num_x * jpeg_fw_codec->mcu_width;
	jpeg_fw_codec->c_height = jpeg_fw_codec->mcu_num_y * jpeg_fw_codec->mcu_height;	
	jpeg_fw_codec->out_width = ((jpeg_fw_codec->c_width) >> (jpeg_fw_codec->scale_factor));
	jpeg_fw_codec->out_height = ((jpeg_fw_codec->c_height) >> (jpeg_fw_codec->scale_factor));
	
	jpeg_fw_codec->dc_pred_y = 0;
	jpeg_fw_codec->dc_pred_uv = 0;
	jpeg_fw_codec->restart_mcu_cnt = 0;
	jpeg_fw_codec->bitstream_offset = 0;
	
	if((jpeg_fw_codec->out_width == 0) || (jpeg_fw_codec->out_height == 0))
	{
		return JPEG_FAILED;
	}
		
	switch(jpeg_fw_codec->input_mcu_info) //code in 6600L, NEED be modified in 6800H? xwluo,20090401
	{
	case JPEG_FW_YUV444:
	case JPEG_FW_YUV422:
	case JPEG_FW_YUV411:		
	case JPEG_FW_YUV400:		
		max_height = 8192;
		break;	
	case JPEG_FW_YUV422_R:
	case JPEG_FW_YUV420:		
		max_height = 8192*2;
		break;
	case JPEG_FW_YUV411_R:	
		max_height = 8192*4;
		break;
	default:
		return JPEG_FAILED;
	}
	
	if((jpeg_fw_codec->c_width > max_width) || (jpeg_fw_codec->c_height > max_height) || 
		((uint32)(jpeg_fw_codec->mcu_num_x*jpeg_fw_codec->mcu_num_y) >= max_mcu_num))
	{
		return JPEG_FAILED;
	}


	return JPEG_SUCCESS;	
}

//@yi.wang added for resource pic decode and interleaved dec and enc
PUBLIC JPEG_RET_E JPEG_FWInitDecInput(JPEG_DEC_INPUT_PARA_T *jpeg_dec_input)
{
	JPEG_CODEC_T *jpeg_fw_codec = Get_JPEGDecCodec();
	JPEG_RET_E ret = JPEG_SUCCESS;

	//Load parameter into JPEG Codec  
	jpeg_fw_codec->stream_0					= jpeg_dec_input->pingpong_buf_0_ptr;
	jpeg_fw_codec->stream_1					= jpeg_dec_input->pingpong_buf_1_ptr;
	jpeg_fw_codec->YUV_Info_0.y_data_ptr	= jpeg_dec_input->yuv_0_addr.y_data_ptr;
	jpeg_fw_codec->YUV_Info_0.u_data_ptr	= jpeg_dec_input->yuv_0_addr.u_data_ptr;
	jpeg_fw_codec->YUV_Info_0.v_data_ptr	= jpeg_dec_input->yuv_0_addr.v_data_ptr;
	jpeg_fw_codec->YUV_Info_1.y_data_ptr	= jpeg_dec_input->yuv_1_addr.y_data_ptr;
	jpeg_fw_codec->YUV_Info_1.u_data_ptr	= jpeg_dec_input->yuv_1_addr.u_data_ptr;
	jpeg_fw_codec->YUV_Info_1.v_data_ptr	= jpeg_dec_input->yuv_1_addr.v_data_ptr;
	jpeg_fw_codec->pingpang_buf_len			= jpeg_dec_input->pingpong_buf_len;
	jpeg_fw_codec->scale_factor				= (uint8)jpeg_dec_input->scaling_down_factor;
	jpeg_fw_codec->decoded_stream_len		= jpeg_dec_input->bitstream_len;
	jpeg_fw_codec->out_put_dataType			= (uint8)jpeg_dec_input->data_type;
	jpeg_fw_codec->read						= jpeg_dec_input->read_bitstream;
	//added by xiaowei.luo,20090113
	jpeg_fw_codec->progressive_mode			= jpeg_dec_input->progressive_mode;	
	jpeg_fw_codec->width					= (uint16)jpeg_dec_input->input_width;
	jpeg_fw_codec->height					= (uint16)jpeg_dec_input->input_height;
	jpeg_fw_codec->input_mcu_info			= (uint8)jpeg_dec_input->input_mcu_info;
	jpeg_fw_codec->work_mode				= (uint8)jpeg_dec_input->work_mode;
	jpeg_fw_codec->is_first_slice			= jpeg_dec_input->is_first_slice;
	jpeg_fw_codec->dbk_bfr0_valid			= jpeg_dec_input->dbk_bfr0_valid;
	jpeg_fw_codec->dbk_bfr1_valid			= jpeg_dec_input->dbk_bfr1_valid;
	jpeg_fw_codec->stream_buf0_valid		= jpeg_dec_input->stream_buf0_valid;
	jpeg_fw_codec->stream_buf1_valid		= jpeg_dec_input->stream_buf1_valid;
	jpeg_fw_codec->compress_level			= jpeg_dec_input->quant_level;
	
	if(jpeg_dec_input->scaling_down_factor > 2)
	{
		JPEG_TRACE("Invalid scaling down factor, which must be 0 ~ 2\n");
	//	return JPEG_FAILED;
	}

	SCI_MEMCPY(&jpeg_fw_codec->YUV_Info_0, &(jpeg_dec_input->yuv_0_addr), (sizeof(YUV_FORMAT_T)));
	SCI_MEMCPY(&jpeg_fw_codec->YUV_Info_1, &(jpeg_dec_input->yuv_1_addr), (sizeof(YUV_FORMAT_T)));

	switch(jpeg_fw_codec->input_mcu_info)
	{
	case JPEG_FW_YUV420:
		jpeg_fw_codec->ratio[JPEG_FW_Y_ID].v_ratio = 2;
		jpeg_fw_codec->ratio[JPEG_FW_Y_ID].h_ratio = 2;
		jpeg_fw_codec->ratio[JPEG_FW_U_ID].v_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_U_ID].h_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_V_ID].v_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_V_ID].h_ratio = 1;
		break;
	case JPEG_FW_YUV411:
		jpeg_fw_codec->ratio[JPEG_FW_Y_ID].v_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_Y_ID].h_ratio = 4;
		jpeg_fw_codec->ratio[JPEG_FW_U_ID].v_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_U_ID].h_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_V_ID].v_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_V_ID].h_ratio = 1;
		break;
	case JPEG_FW_YUV444:
		jpeg_fw_codec->ratio[JPEG_FW_Y_ID].v_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_Y_ID].h_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_U_ID].v_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_U_ID].h_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_V_ID].v_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_V_ID].h_ratio = 1;
		break;
	case JPEG_FW_YUV422:
		jpeg_fw_codec->ratio[JPEG_FW_Y_ID].v_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_Y_ID].h_ratio = 2;
		jpeg_fw_codec->ratio[JPEG_FW_U_ID].v_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_U_ID].h_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_V_ID].v_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_V_ID].h_ratio = 1;
		break;
	case JPEG_FW_YUV400:
		jpeg_fw_codec->ratio[JPEG_FW_Y_ID].v_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_Y_ID].h_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_U_ID].v_ratio = 0;
		jpeg_fw_codec->ratio[JPEG_FW_U_ID].h_ratio = 0;
		jpeg_fw_codec->ratio[JPEG_FW_V_ID].v_ratio = 0;
		jpeg_fw_codec->ratio[JPEG_FW_V_ID].h_ratio = 0;
		break;
	case JPEG_FW_YUV422_R:
		jpeg_fw_codec->ratio[JPEG_FW_Y_ID].v_ratio = 2;
		jpeg_fw_codec->ratio[JPEG_FW_Y_ID].h_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_U_ID].v_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_U_ID].h_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_V_ID].v_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_V_ID].h_ratio = 1;
		break;
	case JPEG_FW_YUV411_R:
		jpeg_fw_codec->ratio[JPEG_FW_Y_ID].v_ratio = 4;
		jpeg_fw_codec->ratio[JPEG_FW_Y_ID].h_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_U_ID].v_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_U_ID].h_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_V_ID].v_ratio = 1;
		jpeg_fw_codec->ratio[JPEG_FW_V_ID].h_ratio = 1;
		break;
	default:
	//	PRINTF("Sample Format is undefined!\n");
		return JPEG_FAILED;
	}	

	ret = JPEGFW_UpdateMiscFields();
	if(ret != JPEG_SUCCESS)
	{
		return ret; 
	}

	return JPEG_SUCCESS;
}

//for progressive
PUBLIC void JPEGFW_AllocMCUBuf(void)
{
	JPEG_CODEC_T *jpeg_fw_codec = Get_JPEGDecCodec();
	JPEG_PROGRESSIVE_INFO_T *progressive_info = JPEGFW_GetProgInfo();
	uint16 block_id = 0, block_num = 0;
	uint8 component_id = 0, i = 0;

	progressive_info->block_line[0] = (int16*)JpegDec_ExtraMemAlloc(64*sizeof(int16)*jpeg_fw_codec->mcu_num_x*jpeg_fw_codec->ratio[0].h_ratio*jpeg_fw_codec->ratio[0].v_ratio);
	progressive_info->block_line[1] = (int16*)JpegDec_ExtraMemAlloc(64*sizeof(int16)*jpeg_fw_codec->mcu_num_x);
	progressive_info->block_line[2] = (int16*)JpegDec_ExtraMemAlloc(64*sizeof(int16)*jpeg_fw_codec->mcu_num_x);
	
	block_id = 0;
	for (component_id = 0; component_id < 3; component_id++)
	{
		block_num = (jpeg_fw_codec->ratio[component_id].h_ratio) * (jpeg_fw_codec->ratio[component_id].v_ratio);
		for (i=0; i<block_num; i++)
		{
			//progressive_info->blocks[block_id] = (int16*)JpegDec_ExtraMemAlloc(64*sizeof(int16)); //g_mcu_buf + block_id * 64;
			progressive_info->org_blocks[block_id] = (uint8*)JpegDec_ExtraMemAlloc(64*sizeof(uint8)); //g_mcu_org_buf +block_id * 64;
			progressive_info->blocks_membership[block_id] = component_id;
			block_id++;
		}
	}
}

//////////////////////////////////////////////////////////////////////////
#endif //JPEG_DEC
/**---------------------------------------------------------------------------*
**                         Compiler Flag                                      *
**---------------------------------------------------------------------------*/
#ifdef   __cplusplus
    }
#endif
/**---------------------------------------------------------------------------*/
// End 
