
/*----------------------------------------------------------------------------*
 **                          Dependencies                                     *
 **---------------------------------------------------------------------------*/
#include "isp_param_tune_com.h"
#include "isp_param_tune_v0000.h"
/**---------------------------------------------------------------------------*
 **                          Compiler Flag                                    *
 **---------------------------------------------------------------------------*/
#ifdef   __cplusplus
extern   "C"
{
#endif

/**---------------------------------------------------------------------------*
**                               Micro Define                                **
**----------------------------------------------------------------------------*/
#define ISP_SIZE_NUM 0x08
struct isp_size_info s_isp_size_info[ISP_SIZE_NUM]={
	{ISP_SIZE_640x480, 640, 480},
	{ISP_SIZE_800x600, 800, 600},
	{ISP_SIZE_1280x960, 1280, 960},
	{ISP_SIZE_1280x1024, 1280, 1024},
	{ISP_SIZE_1600x1200, 1600, 1200},
	{ISP_SIZE_2048x1536, 2048, 1536},
	{ISP_SIZE_2592x1944, 2592, 1944},
	{ISP_SIZE_3264x2448, 3264, 2448}
};

/**---------------------------------------------------------------------------*
**                               Data Prototype                              **
**----------------------------------------------------------------------------*/
static int32_t _ispParamVerify(void* in_param_ptr)
{
	int32_t rtn=0x00;
	uint32_t* param_ptr=(uint32_t*)in_param_ptr;
	uint32_t verify_id=param_ptr[0];
	uint32_t packet_size=param_ptr[2];
	uint32_t end_id=param_ptr[(packet_size/4)-1];

	CMR_LOGE("ISP_TOOL:_ispParamVerify param: 0x%x, 0x%x, 0x%x\n", param_ptr[0], param_ptr[1], param_ptr[2]);

	if(ISP_PACKET_VERIFY_ID!=verify_id)
	{
		rtn=0x01;
	}

	if(ISP_PACKET_END_ID!=end_id)
	{
		rtn=0x01;
	}

	return rtn;
}

static uint32_t _ispParserGetType(void* in_param_ptr)
{
	uint32_t* param_ptr=(uint32_t*)in_param_ptr;
	uint32_t type=param_ptr[2];

	return type;
}

enum isp_tune_level _ispLevelConvert(uint32_t module_id)
{
	enum isp_tune_level cmd=ISP_TUNE_MAX;

	switch(module_id)
	{
		case ISP_TUNE_AWB_MODE:
		{
			cmd=ISP_CTRL_AWB_MODE;
			break;
		}
		case ISP_TUNE_AE_MODE:
		{
			cmd=ISP_CTRL_AE_MODE;
			break;
		}
		case ISP_TUNE_AE_MEASURE_LUM:
		{
			cmd=ISP_CTRL_AE_MEASURE_LUM;
			break;
		}
		case ISP_TUNE_EV:
		{
			cmd=ISP_CTRL_EV;
			break;
		}
		case ISP_TUNE_FLICKER:
		{
			cmd=ISP_CTRL_FLICKER;
			break;
		}
		case ISP_TUNE_SPECIAL_EFFECT:
		{
			cmd=ISP_CTRL_SPECIAL_EFFECT;
			break;
		}
		case ISP_TUNE_BRIGHTNESS:
		{
			cmd=ISP_CTRL_BRIGHTNESS;
			break;
		}
		case ISP_TUNE_CONTRAST:
		{
			cmd=ISP_CTRL_CONTRAST;
			break;
		}
		case ISP_TUNE_AUTO_CONTRAST:
		{
			cmd=ISP_CTRL_AUTO_CONTRAST;
			break;
		}
		case ISP_TUNE_SATURATION:
		{
			cmd=ISP_CTRL_SATURATION;
			break;
		}
		case ISP_TUNE_AF:
		{
			cmd=ISP_CTRL_AF;
			break;
		}
		case ISP_TUNE_CSS:
		{
			cmd=ISP_CTRL_CSS;
			break;
		}
		case ISP_TUNE_HDR:
		{
			cmd=ISP_CTRL_HDR;
			break;
		}
		case ISP_TUNE_ISO:
		{
			cmd=ISP_CTRL_ISO;
			break;
		}
		default :
		{
			break;
		}
	}

	return cmd;
}

static int32_t _ispParserDownCmd(void* in_param_ptr, void* rtn_param_ptr)
{
	int32_t rtn=0x00;
	uint32_t* param_ptr=(uint32_t*)in_param_ptr+0x03;
	struct isp_parser_cmd_param* rtn_ptr=(struct isp_parser_cmd_param*)rtn_param_ptr;
	uint32_t cmd=param_ptr[0];
	uint32_t i=0x00;

	rtn_ptr->cmd=cmd;

	CMR_LOGE("ISP_TOOL:_ispParserDownCmd type: 0x%x, 0x%x\n", param_ptr[0], param_ptr[1]);

	switch(cmd)
	{
		case ISP_CAPTURE:
		{
			rtn_ptr->param[0]=param_ptr[2];//format
			for(i=0x00; i<ISP_SIZE_NUM; i++)
			{
				if(s_isp_size_info[i].size_id==param_ptr[3])
				{
					rtn_ptr->param[1]=s_isp_size_info[i].width;//width
					rtn_ptr->param[2]=s_isp_size_info[i].height;//height
					break ;
				}
			}

			break;
		}
		case ISP_PREVIEW:
		case ISP_STOP_PREVIEW:
		case ISP_UP_PREVIEW_DATA:
		case ISP_UP_PARAM:
		case ISP_CAPTURE_SIZE:
		case ISP_MAIN_INFO:
		{
			break;
		}
		default :
		{
			break;
		}
	}

	return rtn;
}

static int32_t _ispParserDownParam(void* in_param_ptr)
{
	uint32_t rtn=0x00;
	uint32_t* param_ptr=(uint32_t*)((uint8_t*)in_param_ptr+0x0c);// packet data
	uint32_t version_id=param_ptr[0];
	uint32_t module_id=param_ptr[1];
	uint32_t* data_ptr=(uint32_t*)((uint8_t*)param_ptr+0x0c); // packet data sub_data
	isp_fun fun_ptr=NULL;

	switch(version_id)
	{
		case ISP_VERSION_0000_ID:
		{
			fun_ptr=ispGetDownParamFunV0000(module_id);
			break;
		}
		default :
		{
			break;
		}
	}

	if(NULL!=fun_ptr)
	{
		rtn=fun_ptr(data_ptr);
	}
	else
	{
		// trace error
	}

	return rtn;
}

static int32_t _ispParserDownLevel(void* in_param_ptr)
{
	int32_t rtn=0x00;
	uint32_t* param_ptr=(uint32_t*)((uint8_t*)in_param_ptr+0x0c);// packet data
	uint32_t module_id=param_ptr[1];
	enum isp_ctrl_cmd cmd=ISP_CTRL_MAX;
	uint32_t level=param_ptr[3];

	cmd=_ispLevelConvert(module_id);

	//rtn=isp_ioctl(cmd, void* param_ptr, PNULL);

	return rtn;
}

static int32_t _ispParserUpMainInfo(void* rtn_param_ptr)
{
	uint32_t rtn=0x00;
	SENSOR_EXP_INFO_T_PTR sensor_info_ptr=Sensor_GetInfo();
	struct isp_parser_buf_rtn* rtn_ptr=(struct isp_parser_buf_rtn*)rtn_param_ptr;
	uint32_t i=0x00;
	uint32_t j=0x00;
	uint32_t* data_addr=NULL;
	uint32_t data_len =0x10;
	struct isp_main_info* param_ptr=NULL;

	data_len=sizeof(struct isp_main_info);
	data_addr=ispParserAlloc(data_len);
	param_ptr=(struct isp_main_info*)data_addr;

	memset((void*)data_addr, 0x00, sizeof(struct isp_main_info));

	rtn_ptr->buf_addr=(uint32_t)data_addr;
	rtn_ptr->buf_len=sizeof(struct isp_main_info);

	if((NULL!=sensor_info_ptr)
		&&(NULL!=sensor_info_ptr->raw_info_ptr))
	{
		strcpy((char*)&param_ptr->sensor_id, sensor_info_ptr->name);
		param_ptr->version_id=sensor_info_ptr->raw_info_ptr->version_info->version_id;

		for(j=0x00; j<ISP_SIZE_NUM; j++)
		{
			if((s_isp_size_info[j].width==sensor_info_ptr->sensor_mode_info[1].width)
				&&(s_isp_size_info[j].height==sensor_info_ptr->sensor_mode_info[1].height))
			{
				param_ptr->preview_size=s_isp_size_info[j].size_id;
				break ;
			}
		}

		for(i=0x00; i<SENSOR_MODE_MAX; i++)
		{
			for(j=0x00; j<ISP_SIZE_NUM; j++)
			{
				if((s_isp_size_info[j].width==sensor_info_ptr->sensor_mode_info[i].width)
					&&(s_isp_size_info[j].height==sensor_info_ptr->sensor_mode_info[i].height))
				{
					param_ptr->capture_size|=s_isp_size_info[j].size_id;
					break ;
				}
			}
		}

		param_ptr->preview_format = ISP_DATA_YUV420_2FRAME;
		param_ptr->capture_format = ISP_DATA_YUV420_2FRAME|ISP_DATA_JPG;

		if(0x00==sensor_info_ptr->sensor_interface.is_loose){
			param_ptr->capture_format |= ISP_DATA_MIPI_RAW10;
		} else {
			param_ptr->capture_format |= ISP_DATA_NORMAL_RAW10;
		}

	} else {
		rtn_ptr->buf_addr=NULL;
		rtn_ptr->buf_len=0x00;
		rtn = 0x01;
	}


	return rtn;
}

static int32_t _ispParserUpParam(void* rtn_param_ptr)
{
	uint32_t rtn=0x00;
	SENSOR_EXP_INFO_T_PTR sensor_info_ptr=Sensor_GetInfo();
	uint32_t version_id=sensor_info_ptr->raw_info_ptr->version_info->version_id;

	CMR_LOGE("ISP_TOOL:_ispParserUpParam %d, %d\n", version_id, ISP_VERSION_0000_ID);

	switch(version_id)
	{
		case ISP_VERSION_0000_ID:
		{
			rtn=ispGetUpParamV0000(NULL, rtn_param_ptr);
			break;
		}
		default :
		{
			break;
		}
	}

	return rtn;
}

static int32_t _ispParserUpdata(void* in_param_ptr, void* rtn_param_ptr)
{
	int32_t rtn=0x00;
	struct isp_parser_up_data* in_ptr=(struct isp_parser_up_data*)in_param_ptr;
	struct isp_parser_buf_rtn* rtn_ptr=(struct isp_parser_buf_rtn*)rtn_param_ptr;
	uint32_t* data_addr=NULL;
	uint32_t data_len=0x10;

	data_len+=in_ptr->buf_len;
	data_addr=ispParserAlloc(data_len);

	if(NULL!=data_addr)
	{
		data_addr[0]=data_len;
		data_addr[1]=in_ptr->format;
		data_addr[2]=in_ptr->pattern;
		data_addr[3]=in_ptr->width;
		data_addr[4]=in_ptr->height;

		memcpy((void*)&data_addr[5], (void*)in_ptr->buf_addr, in_ptr->buf_len);

		// rtn sesult
		rtn_ptr->buf_addr=data_addr;
		rtn_ptr->buf_len=data_len;
	}

	return rtn;
}

static int32_t _ispParserCapturesize(void* in_param_ptr, void* rtn_param_ptr)
{
	int32_t rtn=0x00;
	struct isp_parser_cmd_param* in_ptr=(struct isp_parser_cmd_param*)in_param_ptr;
	struct isp_parser_buf_rtn* rtn_ptr=(struct isp_parser_buf_rtn*)rtn_param_ptr;
	uint32_t* data_addr=NULL;
	uint32_t data_len=0x0c;

	data_addr=ispParserAlloc(data_len);

	if(NULL!=data_addr)
	{
		data_addr[0]=in_ptr->cmd;
		data_addr[1]=data_len;
		data_addr[2]=in_ptr->param[0];

		// rtn sesult
		rtn_ptr->buf_addr=data_addr;
		rtn_ptr->buf_len=data_len;
	}

	return rtn;
}

static int32_t _ispParserDownHnadle(void* in_param_ptr, void* rtn_param_ptr)
{
	int32_t rtn=0x00;
	uint32_t* param_ptr=(uint32_t*)in_param_ptr; //packet
	uint32_t type=param_ptr[1];

	rtn=_ispParamVerify(in_param_ptr);

	//    type=_ispParserGetType(in_param_ptr);

	CMR_LOGE("ISP_TOOL:_ispParserDownHnadle param: 0x%x, 0x%x, 0x%x\n", param_ptr[0], param_ptr[1], param_ptr[2]);

	CMR_LOGE("ISP_TOOL:_ispParserDownHnadle type: 0x%x\n", type);

	switch(type)
	{
		case ISP_TYPE_CMD:
		{
			rtn=_ispParserDownCmd(in_param_ptr, rtn_param_ptr);
			break;
		}
		case ISP_TYPE_PARAM:
		{
			rtn=_ispParserDownParam(in_param_ptr);
			break;
		}
		case ISP_TYPE_LEVEL:
		{
			rtn=_ispParserDownLevel(in_param_ptr);
			break;
		}
		default :
		{
			break;
		}
	}

	return rtn;
}

static int32_t _ispParserUpHnadle(uint32_t cmd, void* in_param_ptr, void* rtn_param_ptr)
{
	int32_t rtn=0x00;
	struct isp_parser_up_data* in_ptr=(struct isp_parser_up_data*)in_param_ptr;
	struct isp_parser_buf_rtn* rtn_ptr=(struct isp_parser_buf_rtn*)rtn_param_ptr;
	uint32_t* data_addr=NULL;
	uint32_t data_len=0x10;

	CMR_LOGE("ISP_TOOL:_ispParserUpHnadle %d\n", cmd);

	switch(cmd)
	{
		case ISP_PARSER_UP_MAIN_INFO:
		{
			rtn=_ispParserUpMainInfo(rtn_param_ptr);
			break;
		}
		case ISP_PARSER_UP_PARAM:
		{
			rtn=_ispParserUpParam(rtn_param_ptr);
			break;
		}
		case ISP_PARSER_UP_PRV_DATA:
		case ISP_PARSER_UP_CAP_DATA:
		{
			rtn=_ispParserUpdata(in_param_ptr, rtn_param_ptr);
			break;
		}
		case ISP_PARSER_UP_CAP_SIZE:
		{
			rtn=_ispParserCapturesize(in_param_ptr, rtn_param_ptr);
			break;
		}
		default :
		{
			break;
		}
	}

	if(0x00==rtn)
	{
		data_len+=rtn_ptr->buf_len;
		data_addr=ispParserAlloc(data_len);

		if(NULL!=data_addr)
		{
			data_addr[0]=ISP_PACKET_VERIFY_ID;

			switch(cmd)
			{
				case ISP_PARSER_UP_PARAM:
				{
					data_addr[1]=ISP_TYPE_PARAM;
					break;
				}
				case ISP_PARSER_UP_PRV_DATA:
				{
					data_addr[1]=ISP_TYPE_PRV_DATA;
					break;
				}
				case ISP_PARSER_UP_CAP_DATA:
				{
					data_addr[1]=ISP_TYPE_CAP_DATA;
					break;
				}
				case ISP_PARSER_UP_CAP_SIZE:
				{
					data_addr[1]=ISP_TYPE_CMD;
					break;
				}
				case ISP_PARSER_UP_MAIN_INFO:
				{
					data_addr[1]=ISP_TYPE_MAIN_INFO;
					break;
				}
				default :
				{
					break;
				}
			}

			data_addr[2]=data_len;

			memcpy((void*)&data_addr[3], (void*)rtn_ptr->buf_addr, rtn_ptr->buf_len);

			data_addr[(data_len>>0x02)-0x01]=ISP_PACKET_END_ID;

			ispParserFree((void*)rtn_ptr->buf_addr);

			// rtn sesult
			rtn_ptr->buf_addr=data_addr;
			rtn_ptr->buf_len=data_len;
		}
	}

	return rtn;
}

uint32_t ispParserGetSizeID(uint32_t width, uint32_t height)
{
	uint32_t i=0x00;
	uint32_t size_id=0x00;

	for(i=0x00; i<ISP_SIZE_NUM; i++)
	{
		if((s_isp_size_info[i].width==width)
			&&(s_isp_size_info[i].height==height))
		{
			size_id=s_isp_size_info[i].size_id;
			break ;
		}
	}

	return size_id;
}

int32_t ispParser(uint32_t cmd, void* in_param_ptr, void* rtn_param_ptr)
{
	int32_t rtn=0x00;

	CMR_LOGE("ISP_TOOL:ispParser\n");

	switch(cmd)
	{
		case ISP_PARSER_UP_MAIN_INFO:
		case ISP_PARSER_UP_PARAM:
		case ISP_PARSER_UP_PRV_DATA:
		case ISP_PARSER_UP_CAP_DATA:
		case ISP_PARSER_UP_CAP_SIZE:
		{
			rtn=_ispParserUpHnadle(cmd, in_param_ptr, rtn_param_ptr);
			break;
		}
		case ISP_PARSER_DOWN:
		{
			rtn=_ispParserDownHnadle(in_param_ptr, rtn_param_ptr);
			break;
		}

		default :
		{
			break;
		}
	}

	return rtn;
}

uint32_t* ispParserAlloc(uint32_t size)
{
	uint32_t* addr=0x00;

	if(0x00!=size)
	{
		addr=(uint32_t*)malloc(size);
	}

	return addr;
}

int32_t ispParserFree(void* addr)
{
	int32_t rtn=0x00;
	void* temp_addr=addr;

	if(NULL!=temp_addr)
	{
		free(temp_addr);
	}

	return rtn;
}
/**----------------------------------------------------------------------------*
**                         Compiler Flag                                      **
**----------------------------------------------------------------------------*/
#ifdef   __cplusplus
}
#endif
/**---------------------------------------------------------------------------*/

// End

