/*
 * Copyright (C) 2012 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

 #include <utils/Log.h>
#if 0
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <mach/regulator.h>
#include <mach/board.h>
#endif
#include <fcntl.h>              /* low-level i/o */
#include <errno.h>
#include <sys/ioctl.h>

#include "sensor_cfg.h"
#include "sensor_drv_u.h"

#define SENSOR_ONE_I2C	1
#define SENSOR_ZERO_I2C	0
#define SENSOR_16_BITS_I2C	2

/*
#define SENSOR_I2C_FREQ      (100*1000)
#define SENSOR_I2C_PORT_0		0
#define SENSOR_I2C_ACK_TRUE		1
#define SENSOR_I2C_ACK_FALSE		0
#define SENSOR_I2C_STOP    1
#define SENSOR_I2C_NOSTOP    0
#define SENSOR_I2C_NULL_HANDLE  -1

#define SENSOR_LOW_SEVEN_BIT     0x7f
#define SENSOR_LOW_EIGHT_BIT     0xff
#define SENSOR_HIGN_SIXTEEN_BIT  0xffff0000
*/

#define SENSOR_LOW_SIXTEEN_BIT  0xffff

#define SCI_TRUE 1
#define SCI_FALSE 0

static char			dev_name[50] = "/dev/sprd_sensor";
static int				g_fd_sensor = -1;
static cmr_evt_cb		sensor_event_cb = NULL;
static pthread_mutex_t	cb_mutex = PTHREAD_MUTEX_INITIALIZER;

/**---------------------------------------------------------------------------*
 **                         Local Variables                                   *
 **---------------------------------------------------------------------------*/
LOCAL SENSOR_INFO_T *s_sensor_list_ptr[SENSOR_ID_MAX];
LOCAL SENSOR_INFO_T *s_sensor_info_ptr = PNULL;
LOCAL SENSOR_EXP_INFO_T s_sensor_exp_info;
LOCAL BOOLEAN s_sensor_init = SENSOR_FALSE;
LOCAL SENSOR_TYPE_E s_sensor_type = SENSOR_TYPE_NONE;
LOCAL SENSOR_MODE_E s_sensor_mode[SENSOR_ID_MAX] =
    { SENSOR_MODE_MAX, SENSOR_MODE_MAX, SENSOR_MODE_MAX };
//LOCAL SENSOR_MUTEX_PTR s_imgsensor_mutex_ptr = PNULL;
LOCAL SENSOR_REGISTER_INFO_T s_sensor_register_info = { 0x00};

LOCAL SENSOR_REGISTER_INFO_T_PTR s_sensor_register_info_ptr =
    &s_sensor_register_info;

LOCAL uint32_t s_flash_mode = 0xff;
static int g_is_main_sensor = 0;
static int g_is_register_sensor = 0;

LOCAL uint16_t g_i2c_addr = 0xff;

#define SIGN_0  0x73
#define SIGN_1  0x69
#define SIGN_2  0x67
#define SIGN_3  0x6e
static BOOLEAN s_sensor_identified = SCI_FALSE;
static BOOLEAN s_sensor_param_saved = SCI_FALSE;
static uint8_t  s_sensor_index[SENSOR_ID_MAX]={0xFF,0xFF,0xFF,0xFF,0xFF};

LOCAL EXIF_SPEC_PIC_TAKING_COND_T s_default_exif={0x00};

/* Sensor Device IO Control  */
#define SENSOR_IOC_MAGIC		'R'

#define SENSOR_IO_PD				_IOW(SENSOR_IOC_MAGIC, 0,  BOOLEAN)
#define SENSOR_IO_SET_AVDD		_IOW(SENSOR_IOC_MAGIC, 1,  uint32_t)
#define SENSOR_IO_SET_DVDD		_IOW(SENSOR_IOC_MAGIC, 2,  uint32_t)
#define SENSOR_IO_SET_IOVDD		_IOW(SENSOR_IOC_MAGIC, 3,  uint32_t)
#define SENSOR_IO_SET_MCLK		_IOW(SENSOR_IOC_MAGIC, 4,  uint32_t)
#define SENSOR_IO_RST			_IOW(SENSOR_IOC_MAGIC, 5,  uint32_t)
#define SENSOR_IO_I2C_INIT		_IOW(SENSOR_IOC_MAGIC, 6,  uint32_t)
#define SENSOR_IO_I2C_DEINIT		_IOW(SENSOR_IOC_MAGIC, 7,  uint32_t)
#define SENSOR_IO_SET_ID			_IOW(SENSOR_IOC_MAGIC, 8,  uint32_t)
#define SENSOR_IO_RST_LEVEL		_IOW(SENSOR_IOC_MAGIC, 9,  uint32_t)
#define SENSOR_IO_I2C_ADDR		_IOW(SENSOR_IOC_MAGIC, 10, uint16_t)
#define SENSOR_IO_I2C_READ		_IOWR(SENSOR_IOC_MAGIC, 11, SENSOR_REG_BITS_T)
#define SENSOR_IO_I2C_WRITE		_IOW(SENSOR_IOC_MAGIC, 12, SENSOR_REG_BITS_T)
#define SENSOR_IO_SET_FLASH		_IOW(SENSOR_IOC_MAGIC, 13, uint32_t)



LOCAL uint32_t _Sensor_InitDefaultExifInfo(void);

static int xioctl(int fd, int request, void * arg) {   
    int r;   
    //do   
        r = ioctl(fd, request, arg);   
    //while (-1 == r && EINTR == errno);   
    return r;   
} 


/* This function is to set power down */
LOCAL int _Sensor_Device_PowerDown(BOOLEAN power_level)
{
	int ret = SENSOR_SUCCESS;

	ret = xioctl(g_fd_sensor, SENSOR_IO_PD, &power_level);
	
	if (0 != ret)
	{
		SENSOR_PRINT_ERR("_Sensor_Device_PowerDown failed,  power_level = %d, ret=%d \n", power_level, ret);
		ret = -1;
	}

	return ret;
}

LOCAL int _Sensor_Device_SetVoltageAVDD(uint32_t vdd_value)
{
	int ret = SENSOR_SUCCESS;

	ret = xioctl(g_fd_sensor, SENSOR_IO_SET_AVDD, &vdd_value);
	if (0 != ret)
	{
		SENSOR_PRINT_ERR("_Sensor_Device_SetVoltageAVDD failed,  vdd_value = %d, ret=%d \n", vdd_value, ret);
		ret = -1;
	}

	return ret;
}

LOCAL int _Sensor_Device_SetVoltageDVDD(uint32_t vdd_value)
{
	int ret = SENSOR_SUCCESS;

	ret = xioctl(g_fd_sensor, SENSOR_IO_SET_DVDD, &vdd_value);
	if (0 != ret)
	{
		SENSOR_PRINT_ERR("_Sensor_Device_SetVoltageDVDD failed,  vdd_value = %d, ret=%d \n", vdd_value, ret);
		ret = -1;
	}

	return ret;
}

LOCAL int _Sensor_Device_SetVoltageIOVDD(uint32_t vdd_value)
{
	int ret = SENSOR_SUCCESS;

	ret = xioctl(g_fd_sensor, SENSOR_IO_SET_IOVDD, &vdd_value);
	if (0 != ret)
	{
		SENSOR_PRINT_ERR("_Sensor_Device_SetVoltageIOVDD failed,  vdd_value = %d, ret=%d \n", vdd_value, ret);
		ret = -1;
	}

	return ret;
}

LOCAL int _Sensor_Device_read(uint8_t *buff, uint32_t size)
{
	int ret = SENSOR_SUCCESS;

	ret = read(g_fd_sensor, buff, size);

	return ret;
}

LOCAL int _Sensor_Device_Write(uint8_t *buff, uint32_t size)
{
	int ret = SENSOR_SUCCESS;

	ret = write(g_fd_sensor, buff, size);

	if(0 != ret)
	{
		SENSOR_PRINT_ERR("_Sensor_Device_Write failed,  buff[0] = %d, size=%d, ret=%d \n", buff[0], size, ret);
		ret = -1;
	}

	return ret;
}


LOCAL int _Sensor_Device_SetMCLK(uint32_t mclk)
{
	int ret = SENSOR_SUCCESS;

	ret = xioctl(g_fd_sensor, SENSOR_IO_SET_MCLK, &mclk);
	
	if (0 != ret)
	{
		SENSOR_PRINT_ERR("_Sensor_Device_SetMCLK failed,  mclk = %d, ret = %d  \n", mclk, ret);
		ret = -1;
	}

	return ret;
}

LOCAL int _Sensor_Device_Reset(uint32_t level)
{
	int ret = SENSOR_SUCCESS;

	ret = xioctl(g_fd_sensor, SENSOR_IO_RST, &level);
	if (0 != ret)
	{
		SENSOR_PRINT_ERR("_Sensor_Device_Reset failed,  level = %d, ret=%d \n", level, ret);
		ret = -1;
	}

	return ret;
}

LOCAL int _Sensor_Device_I2CInit(uint32_t senor_id)
{
	int ret = SENSOR_SUCCESS;

	ret = xioctl(g_fd_sensor, SENSOR_IO_I2C_INIT, &senor_id);
	if (0 != ret)
	{
		SENSOR_PRINT_ERR("_Sensor_Device_I2CInit failed,  senor_id = %d, ret=%d \n", senor_id, ret);
		ret = -1;
	}

	return ret;
}

LOCAL int _Sensor_Device_I2CDeInit(uint32_t senor_id)
{
	int ret = SENSOR_SUCCESS;

	ret = xioctl(g_fd_sensor, SENSOR_IO_I2C_DEINIT, &senor_id);
	if (0 != ret)
	{
		SENSOR_PRINT_ERR("_Sensor_Device_I2CDeInit failed,  senor_id = %d, ret=%d \n", senor_id, ret);
		ret = -1;
	}

	return ret;
}


LOCAL int _Sensor_Device_ResetLevel(uint32_t level)
{
	int ret = SENSOR_SUCCESS;

	ret = xioctl(g_fd_sensor, SENSOR_IO_RST_LEVEL, &level);
	if (0 != ret)
	{
		SENSOR_PRINT_ERR("_Sensor_Device_Reset failed,  level = %d, ret=%d \n", level, ret);
		ret = -1;
	}

	return ret;
}

LOCAL int _Sensor_Device_SetI2cAddr(uint16_t addr)
{
	int ret = SENSOR_SUCCESS;

	ret = xioctl(g_fd_sensor, SENSOR_IO_I2C_ADDR, &addr);
	if (0 != ret)
	{
		SENSOR_PRINT_ERR("_Sensor_Device_SetI2cAddr failed,  addr = 0x%x, ret=%d \n", addr, ret);
		ret = -1;
	}

	return ret;
}

LOCAL int _Sensor_Device_ReadReg(SENSOR_REG_BITS_T_PTR reg)
{
	int ret = SENSOR_SUCCESS;

	ret = xioctl(g_fd_sensor, SENSOR_IO_I2C_READ, reg);
	if (0 != ret)
	{
		SENSOR_PRINT_ERR("_Sensor_Device_ReadReg failed,  addr = 0x%x, value=%x, bit=%d, ret=%d \n", reg->reg_addr, reg->reg_value, reg->reg_bits, ret);
		ret = -1;
	}

	return ret;
}

LOCAL int _Sensor_Device_WriteReg(SENSOR_REG_BITS_T_PTR reg)
{
	int ret = SENSOR_SUCCESS;

	ret = xioctl(g_fd_sensor, SENSOR_IO_I2C_WRITE, reg);
	if (0 != ret)
	{
		SENSOR_PRINT_ERR("_Sensor_Device_WriteReg failed,  addr = 0x%x, value=%x, bit=%d, ret=%d \n", reg->reg_addr, reg->reg_value, reg->reg_bits, ret);
		ret = -1;
	}

	return ret;
}

LOCAL int _Sensor_Device_SetFlash(uint32_t flash_mode)
{
	int ret = SENSOR_SUCCESS;

	ret = xioctl(g_fd_sensor, SENSOR_IO_SET_FLASH, &flash_mode);
	if (0 != ret)
	{
		SENSOR_PRINT_ERR("_Sensor_Device_SetFlash failed,   flash_mode=%d \n",flash_mode);
		ret = -1;
	}

	return ret;
}

SENSOR_TYPE_E _Sensor_GetSensorType(void)
{
	return s_sensor_type;
}

void Sensor_Reset(uint32_t level)
{
	int err = 0xff;
	SENSOR_IOCTL_FUNC_PTR reset_func;
	SENSOR_PRINT_HIGH("Sensor_Reset.\n");

	reset_func = s_sensor_info_ptr->ioctl_func_tab_ptr->reset;

	if (PNULL != reset_func) {
		reset_func(level);
	}else{
		_Sensor_Device_Reset(level);
	}
}


int Sensor_SetMCLK(uint32_t mclk)
{
	int ret;

	SENSOR_PRINT_HIGH("Sensor_SetMCLK mclk = %d \n", mclk);
	
	ret = _Sensor_Device_SetMCLK(mclk);

	SENSOR_PRINT_HIGH("Sensor_SetMCLK Done mclk = %d, ret = %d \n", mclk, ret );
	
	return ret;
}

int Sensor_SetVoltage(SENSOR_AVDD_VAL_E dvdd_val, SENSOR_AVDD_VAL_E avdd_val,
		       SENSOR_AVDD_VAL_E iodd_val)
{

	int err = 0;
	uint32_t volt_value = 0;

	err = _Sensor_Device_SetVoltageAVDD((uint32_t)avdd_val);
	if(SENSOR_SUCCESS != err)
		return err;
	
	
	err = _Sensor_Device_SetVoltageDVDD((uint32_t)dvdd_val);
	if(SENSOR_SUCCESS != err)
		return err;

	err = _Sensor_Device_SetVoltageIOVDD((uint32_t)iodd_val);

	SENSOR_PRINT_HIGH("Sensor_SetVoltage avdd_val = %d,  dvdd_val=%d, iodd_val=%d \n", avdd_val, dvdd_val, iodd_val);

	return err;
}

LOCAL void Sensor_PowerOn(BOOLEAN power_on)
{
	BOOLEAN power_down;
	SENSOR_AVDD_VAL_E dvdd_val;
	SENSOR_AVDD_VAL_E avdd_val;
	SENSOR_AVDD_VAL_E iovdd_val;
	SENSOR_IOCTL_FUNC_PTR power_func;
	uint32_t rst_lvl = s_sensor_info_ptr->reset_pulse_level;

	power_down = (BOOLEAN) s_sensor_info_ptr->power_down_level;
	dvdd_val = s_sensor_info_ptr->dvdd_val;
	avdd_val = s_sensor_info_ptr->avdd_val;
	iovdd_val = s_sensor_info_ptr->iovdd_val;
	power_func = s_sensor_info_ptr->ioctl_func_tab_ptr->power;

	SENSOR_PRINT
	    ("SENSOR: Sensor_PowerOn -> power_on = %d, power_down_level = %d, avdd_val = %d\n",
	     power_on, power_down, avdd_val);

	if (PNULL != power_func) {
		power_func(power_on);
	} else {
		if (power_on) {
			Sensor_PowerDown(power_down);
			Sensor_SetVoltage(dvdd_val, avdd_val, iovdd_val);
			SENSOR_Sleep(10);
			Sensor_SetMCLK(SENSOR_DEFALUT_MCLK);
			SENSOR_Sleep(5); 
			Sensor_PowerDown(!power_down);
			Sensor_Reset(rst_lvl);
		} else {
			Sensor_PowerDown(power_down);
			SENSOR_Sleep(20);
			Sensor_SetMCLK(SENSOR_DISABLE_MCLK);
			Sensor_SetVoltage(SENSOR_AVDD_CLOSED,
					  SENSOR_AVDD_CLOSED,
					  SENSOR_AVDD_CLOSED);
		}
	}
}

BOOLEAN Sensor_PowerDown(BOOLEAN power_level)
{
	SENSOR_IOCTL_FUNC_PTR entersleep_func =
	    s_sensor_info_ptr->ioctl_func_tab_ptr->enter_sleep;

	SENSOR_PRINT("SENSOR: Sensor_PowerDown -> main: power_down %d\n",
		     power_level);

	if (entersleep_func) {
		entersleep_func(power_level);
		return SENSOR_SUCCESS;
	}

	if (-1 == _Sensor_Device_PowerDown(power_level))
		return SENSOR_FAIL;

	return SENSOR_SUCCESS;
}

BOOLEAN Sensor_SetResetLevel(BOOLEAN plus_level)
{
	if (-1 ==_Sensor_Device_ResetLevel((uint32_t)plus_level))
		return SENSOR_FAIL;

	return SENSOR_SUCCESS;
}

LOCAL void Sensor_SetExportInfo(SENSOR_EXP_INFO_T * exp_info_ptr)
{
	SENSOR_REG_TAB_INFO_T *resolution_info_ptr = PNULL;
	SENSOR_TRIM_T_PTR resolution_trim_ptr = PNULL;
	SENSOR_INFO_T *sensor_info_ptr = s_sensor_info_ptr;
	uint32_t i = 0;

	SENSOR_PRINT("SENSOR: Sensor_SetExportInfo.\n");

	SENSOR_MEMSET(exp_info_ptr, 0x00, sizeof(SENSOR_EXP_INFO_T));
	exp_info_ptr->image_format = sensor_info_ptr->image_format;
	exp_info_ptr->image_pattern = sensor_info_ptr->image_pattern;

	exp_info_ptr->pclk_polarity = (sensor_info_ptr->hw_signal_polarity & 0x01);	//the high 3bit will be the phase(delay sel)
	exp_info_ptr->vsync_polarity =
	    ((sensor_info_ptr->hw_signal_polarity >> 2) & 0x1);
	exp_info_ptr->hsync_polarity =
	    ((sensor_info_ptr->hw_signal_polarity >> 4) & 0x1);
	exp_info_ptr->pclk_delay =
	    ((sensor_info_ptr->hw_signal_polarity >> 5) & 0x07);

	exp_info_ptr->source_width_max = sensor_info_ptr->source_width_max;
	exp_info_ptr->source_height_max = sensor_info_ptr->source_height_max;

	exp_info_ptr->environment_mode = sensor_info_ptr->environment_mode;
	exp_info_ptr->image_effect = sensor_info_ptr->image_effect;
	exp_info_ptr->wb_mode = sensor_info_ptr->wb_mode;
	exp_info_ptr->step_count = sensor_info_ptr->step_count;

	exp_info_ptr->ext_info_ptr = sensor_info_ptr->ext_info_ptr;

	exp_info_ptr->preview_skip_num = sensor_info_ptr->preview_skip_num;
	exp_info_ptr->capture_skip_num = sensor_info_ptr->capture_skip_num;
	exp_info_ptr->preview_deci_num = sensor_info_ptr->preview_deci_num;
	exp_info_ptr->video_preview_deci_num =
	    sensor_info_ptr->video_preview_deci_num;

	exp_info_ptr->threshold_eb = sensor_info_ptr->threshold_eb;
	exp_info_ptr->threshold_mode = sensor_info_ptr->threshold_mode;
	exp_info_ptr->threshold_start = sensor_info_ptr->threshold_start;
	exp_info_ptr->threshold_end = sensor_info_ptr->threshold_end;

	exp_info_ptr->ioctl_func_ptr = sensor_info_ptr->ioctl_func_tab_ptr;
	if (PNULL != sensor_info_ptr->ioctl_func_tab_ptr->get_trim) {
		resolution_trim_ptr =
		    (SENSOR_TRIM_T_PTR) sensor_info_ptr->
		    ioctl_func_tab_ptr->get_trim(0x00);
	}
	for (i = SENSOR_MODE_COMMON_INIT; i < SENSOR_MODE_MAX; i++) {
		resolution_info_ptr =
		    &(sensor_info_ptr->resolution_tab_info_ptr[i]);
		if ((PNULL != resolution_info_ptr->sensor_reg_tab_ptr)
		    || ((0x00 != resolution_info_ptr->width)
			&& (0x00 != resolution_info_ptr->width))) {
			exp_info_ptr->sensor_mode_info[i].mode = i;
			exp_info_ptr->sensor_mode_info[i].width =
			    resolution_info_ptr->width;
			exp_info_ptr->sensor_mode_info[i].height =
			    resolution_info_ptr->height;
			if ((PNULL != resolution_trim_ptr)
			    && (0x00 != resolution_trim_ptr[i].trim_width)
			    && (0x00 != resolution_trim_ptr[i].trim_height)) {
				exp_info_ptr->sensor_mode_info[i].trim_start_x =
				    resolution_trim_ptr[i].trim_start_x;
				exp_info_ptr->sensor_mode_info[i].trim_start_y =
				    resolution_trim_ptr[i].trim_start_y;
				exp_info_ptr->sensor_mode_info[i].trim_width =
				    resolution_trim_ptr[i].trim_width;
				exp_info_ptr->sensor_mode_info[i].trim_height =
				    resolution_trim_ptr[i].trim_height;
				exp_info_ptr->sensor_mode_info[i].line_time =
				    resolution_trim_ptr[i].line_time;
			} else {
				exp_info_ptr->sensor_mode_info[i].trim_start_x =
				    0x00;
				exp_info_ptr->sensor_mode_info[i].trim_start_y =
				    0x00;
				exp_info_ptr->sensor_mode_info[i].trim_width =
				    resolution_info_ptr->width;
				exp_info_ptr->sensor_mode_info[i].trim_height =
				    resolution_info_ptr->height;
			}
			/*exp_info_ptr->sensor_mode_info[i].line_time=resolution_trim_ptr[i].line_time; */
			if (SENSOR_IMAGE_FORMAT_MAX !=
			    sensor_info_ptr->image_format) {
				exp_info_ptr->sensor_mode_info[i].image_format =
				    sensor_info_ptr->image_format;
			} else {
				exp_info_ptr->sensor_mode_info[i].image_format =
				    resolution_info_ptr->image_format;
			}
			SENSOR_PRINT
			    ("SENSOR: SENSOR mode Info > mode = %d, width = %d, height = %d, format = %d.\n",
			     i, resolution_info_ptr->width,
			     resolution_info_ptr->height,
			     exp_info_ptr->sensor_mode_info[i].image_format);
		} else {
			exp_info_ptr->sensor_mode_info[i].mode =
			    SENSOR_MODE_MAX;
		}
	}
	exp_info_ptr->sensor_interface = sensor_info_ptr->sensor_interface;
}

int32_t Sensor_WriteReg(uint16_t subaddr, uint16_t data)
{
	int32_t ret = -1;
	SENSOR_IOCTL_FUNC_PTR write_reg_func;

	//      SENSOR_PRINT("this_client->addr=0x%x\n",this_client->addr);
	//      SENSOR_PRINT_ERR("Sensor_WriteReg:addr=0x%x,data=0x%x .\n",subaddr,data);

	write_reg_func = s_sensor_info_ptr->ioctl_func_tab_ptr->write_reg;

	if (PNULL != write_reg_func) {
		if (SENSOR_OP_SUCCESS != write_reg_func((subaddr << S_BIT_4) + data)) {
			SENSOR_PRINT("SENSOR: IIC write : reg:0x%04x, val:0x%04x error\n",
			     				subaddr, data);
		}
	} else {

		SENSOR_REG_BITS_T reg;

		reg.reg_addr = subaddr;
		reg.reg_value = data;
		reg.reg_bits = s_sensor_info_ptr->reg_addr_value_bits;

		ret = _Sensor_Device_WriteReg(&reg);
	}

	return ret;
}

uint16_t Sensor_ReadReg(uint16_t reg_addr)
{

	uint32_t i = 0;
	uint16_t ret_val = 0xffff;
	int ret = -1;

	SENSOR_IOCTL_FUNC_PTR read_reg_func;

	SENSOR_PRINT("Read:this_client->addr=0x%x\n", reg_addr);

	read_reg_func = s_sensor_info_ptr->ioctl_func_tab_ptr->read_reg;

	if (PNULL != read_reg_func) {
		ret_val = (uint16_t)read_reg_func((uint32_t)(reg_addr & SENSOR_LOW_SIXTEEN_BIT));
	} else {
		SENSOR_REG_BITS_T reg;

		reg.reg_addr = reg_addr;
		reg.reg_bits = s_sensor_info_ptr->reg_addr_value_bits;

		ret = _Sensor_Device_ReadReg(&reg);
		if(SENSOR_SUCCESS == ret){
			ret_val = reg.reg_value;
		}
		
	}

	return ret_val;
}

int32_t Sensor_WriteReg_8bits(uint16_t reg_addr, uint8_t value)
{
	int32_t ret = -1;

	if (0xFFFF == reg_addr) {
		SENSOR_Sleep(value);
		SENSOR_PRINT("Sensor_WriteReg_8bits wait %d ms.\n", value);
		return 0;
	}

	SENSOR_REG_BITS_T reg;

	reg.reg_addr = reg_addr;
	reg.reg_value = value;
	reg.reg_bits = SENSOR_I2C_REG_8BIT | SENSOR_I2C_VAL_8BIT;

	ret = _Sensor_Device_WriteReg(&reg);

	return 0;
}

int32_t Sensor_ReadReg_8bits(uint8_t reg_addr, uint8_t * reg_val)
{

	int32_t ret = -1;

	SENSOR_REG_BITS_T reg;

	reg.reg_addr = reg_addr;
	reg.reg_bits = SENSOR_I2C_REG_8BIT | SENSOR_I2C_VAL_8BIT;

	ret = _Sensor_Device_ReadReg(&reg);
	if(SENSOR_SUCCESS == ret){
		*reg_val  = reg.reg_value;
	}

	return ret;
}

ERR_SENSOR_E Sensor_SendRegTabToSensor(SENSOR_REG_TAB_INFO_T *
				       sensor_reg_tab_info_ptr)
{
	uint32_t i;

	SENSOR_PRINT("SENSOR: Sensor_SendRegTabToSensor E.\n");



	for (i = 0; i < sensor_reg_tab_info_ptr->reg_count; i++) {
		Sensor_WriteReg(sensor_reg_tab_info_ptr->
				sensor_reg_tab_ptr[i].reg_addr,
				sensor_reg_tab_info_ptr->
				sensor_reg_tab_ptr[i].reg_value);
	}

	SENSOR_PRINT
	    ("SENSOR: Sensor_SendRegValueToSensor -> reg_count = %d, g_is_main_sensor: %d.\n",
	     sensor_reg_tab_info_ptr->reg_count, g_is_main_sensor);

	SENSOR_PRINT("SENSOR: Sensor_SendRegTabToSensor X.\n");

	return SENSOR_SUCCESS;
}

LOCAL void _Sensor_CleanInformation(void)
{
	SENSOR_REGISTER_INFO_T_PTR sensor_register_info_ptr =
	    s_sensor_register_info_ptr;

	s_sensor_info_ptr = PNULL;
	s_sensor_init = SENSOR_FALSE;
	SENSOR_MEMSET(&s_sensor_exp_info, 0x00, sizeof(s_sensor_exp_info));
	sensor_register_info_ptr->cur_id = SENSOR_ID_MAX;
	return;
}

LOCAL int _Sensor_SetId(SENSOR_ID_E sensor_id)
{

	SENSOR_REGISTER_INFO_T_PTR sensor_register_info_ptr =
	    s_sensor_register_info_ptr;

	sensor_register_info_ptr->cur_id = sensor_id;
	
	SENSOR_PRINT_HIGH
	    ("_Sensor_SetId:sensor_id=%d,g_is_register_sensor=%d,g_is_main_sensor=%d \n",
	     sensor_id, g_is_register_sensor, g_is_main_sensor);

	if (1 == g_is_register_sensor) {
		if ((SENSOR_MAIN == sensor_id) && (1 == g_is_main_sensor))
			return SENSOR_SUCCESS;
		if ((SENSOR_SUB == sensor_id) && (0 == g_is_main_sensor))
			return SENSOR_SUCCESS;
	}
	if ((SENSOR_MAIN == sensor_id) || (SENSOR_SUB == sensor_id)) {
		if (SENSOR_SUB == sensor_id) {
			if ((1 == g_is_register_sensor)  && (1 == g_is_main_sensor)) {
				_Sensor_Device_I2CDeInit(SENSOR_MAIN);
			}
			g_is_main_sensor = 0;
		} else if (SENSOR_MAIN == sensor_id) {
			if ((1 == g_is_register_sensor) && (0 == g_is_main_sensor)) {
				_Sensor_Device_I2CDeInit(SENSOR_SUB);
			}
			g_is_main_sensor = 1;
		}

		if (_Sensor_Device_I2CInit(sensor_id)) {
			SENSOR_PRINT_HIGH("SENSOR: add I2C driver error\n");
			return SENSOR_FAIL;
		} else {
			SENSOR_PRINT_HIGH("SENSOR: add I2C driver OK.\n");
			g_is_register_sensor = 1;
		}
	}

	return SENSOR_SUCCESS;
}

SENSOR_ID_E Sensor_GetCurId(void)
{
	SENSOR_REGISTER_INFO_T_PTR sensor_register_info_ptr =
	    s_sensor_register_info_ptr;
	SENSOR_PRINT_HIGH("Sensor_GetCurId,sensor_id =%d",
	       sensor_register_info_ptr->cur_id);
	return (SENSOR_ID_E) sensor_register_info_ptr->cur_id;
}

uint32_t Sensor_SetCurId(SENSOR_ID_E sensor_id)
{
	SENSOR_PRINT("Sensor_SetCurId : %d.\n", sensor_id);
	if (sensor_id >= SENSOR_ID_MAX) {
		_Sensor_CleanInformation();
		return SENSOR_FAIL;
	}
	if (SENSOR_SUCCESS != _Sensor_SetId(sensor_id)) {
		SENSOR_PRINT("SENSOR: Fail to Sensor_SetCurId.\n");
		return SENSOR_FAIL;
	}
	return SENSOR_SUCCESS;
}

SENSOR_REGISTER_INFO_T_PTR Sensor_GetRegisterInfo(void)
{
	return s_sensor_register_info_ptr;
}

LOCAL void _Sensor_I2CInit(SENSOR_ID_E sensor_id)
{
	SENSOR_REGISTER_INFO_T_PTR sensor_register_info_ptr =
	    s_sensor_register_info_ptr;
	SENSOR_INFO_T** sensor_info_tab_ptr = PNULL;
	SENSOR_INFO_T* sensor_info_ptr= PNULL;
 	uint32_t i2c_clock = 100000;
	uint32_t set_i2c_clock = 0;
	sensor_register_info_ptr->cur_id = sensor_id;

	if (0 == g_is_register_sensor) {
		if ((SENSOR_MAIN == sensor_id) || (SENSOR_SUB == sensor_id)) {

			if(_Sensor_Device_I2CInit(sensor_id)){
				SENSOR_PRINT_ERR("SENSOR: add I2C driver error\n");
				return;
			} else {
				SENSOR_PRINT_ERR("SENSOR: add I2C driver OK.\n");
#if 0
				sensor_info_tab_ptr=(SENSOR_INFO_T**)Sensor_GetInforTab(sensor_id);
				if(sensor_info_tab_ptr)
				{
					sensor_info_ptr = sensor_info_tab_ptr[sensor_id];
				}
				if(sensor_info_ptr)
				{
					set_i2c_clock = sensor_info_ptr->reg_addr_value_bits & SENSOR_I2C_CLOCK_MASK;
					if(SENSOR_I2C_FREQ_100 != set_i2c_clock)
					{
						if(SENSOR_I2C_FREQ_400 == set_i2c_clock)
						{
							sc8810_i2c_set_clk(SENSOR_I2C_ID,400000);					
						}
						else if(SENSOR_I2C_FREQ_200 == set_i2c_clock)
						{
							sc8810_i2c_set_clk(SENSOR_I2C_ID,200000);
						}
						else if(SENSOR_I2C_FREQ_50 == set_i2c_clock)
						{
							sc8810_i2c_set_clk(SENSOR_I2C_ID,50000);
						}
						else if(SENSOR_I2C_FREQ_20 == set_i2c_clock)
						{
							sc8810_i2c_set_clk(SENSOR_I2C_ID,20000);
						}
					}
				}
#endif
				g_is_register_sensor = 1;
			}
		}
	} else {
		SENSOR_PRINT_ERR("Sensor: Init I2c  %d fail!\n", sensor_id);
	}
	SENSOR_PRINT_ERR("_Sensor_I2CInit,sensor_id=%d,g_is_register_sensor=%d\n",
	     						sensor_id, g_is_register_sensor);
}

LOCAL int _Sensor_I2CDeInit(SENSOR_ID_E sensor_id)
{
	if (1 == g_is_register_sensor) {
		if ((SENSOR_MAIN == sensor_id) || (SENSOR_SUB == sensor_id)) {
			_Sensor_Device_I2CDeInit(sensor_id);
			g_is_register_sensor = 0;
			SENSOR_PRINT("SENSOR: delete  I2C  %d driver OK.\n", sensor_id);
		}
	} else {
		SENSOR_PRINT("SENSOR: delete  I2C  %d driver OK.\n", SENSOR_ID_MAX);
	}

	return SENSOR_SUCCESS;
}

LOCAL BOOLEAN _Sensor_Identify(SENSOR_ID_E sensor_id)
{
	uint32_t sensor_index = 0;
	SENSOR_INFO_T** sensor_info_tab_ptr = PNULL;
	uint32_t valid_tab_index_max = 0x00;
	SENSOR_INFO_T* sensor_info_ptr=PNULL;

	BOOLEAN retValue = SCI_FALSE;

	SENSOR_PRINT_HIGH("SENSOR: sensor identifing %d", sensor_id);

	//if already identified
	if(SCI_TRUE == s_sensor_register_info_ptr->is_register[sensor_id])
	{
		SENSOR_PRINT("SENSOR: sensor identified");
		return SCI_TRUE;
	}
	if(s_sensor_identified && (5 != sensor_id)) {
		sensor_index = s_sensor_index[sensor_id];
		SENSOR_PRINT_HIGH("_Sensor_Identify:sensor_index=%d.\n",sensor_index);
		if(0xFF != sensor_index) {
			sensor_info_tab_ptr=(SENSOR_INFO_T**)Sensor_GetInforTab(sensor_id);
			_Sensor_I2CInit(sensor_id);
			sensor_info_ptr = sensor_info_tab_ptr[sensor_index];
			if(NULL==sensor_info_ptr)
			{
				SENSOR_PRINT_ERR("SENSOR: %d info of Sensor_Init table %d is null", sensor_index, (uint)sensor_id);
				_Sensor_I2CDeInit(sensor_id);
				goto IDENTIFY_SEARCH;
			}
			s_sensor_info_ptr = sensor_info_ptr;
			Sensor_PowerOn(SCI_TRUE);
			if(PNULL!=sensor_info_ptr->ioctl_func_tab_ptr->identify)
			{
				g_i2c_addr =  (s_sensor_info_ptr->salve_i2c_addr_w & 0xFF); 
				_Sensor_Device_SetI2cAddr(g_i2c_addr);

				SENSOR_PRINT_ERR("SENSOR:identify  Sensor 01\n");
				if(SENSOR_SUCCESS==sensor_info_ptr->ioctl_func_tab_ptr->identify(SENSOR_ZERO_I2C))
				{
					s_sensor_list_ptr[sensor_id]=sensor_info_ptr;
					s_sensor_register_info_ptr->is_register[sensor_id]=SCI_TRUE;
					s_sensor_register_info_ptr->img_sensor_num++;
					//Sensor_PowerOn(SCI_FALSE);
					retValue = SCI_TRUE;
					SENSOR_PRINT_HIGH("_Sensor_Identify:sensor_id :%d,img_sensor_num=%d\n",
						                                     sensor_id,s_sensor_register_info_ptr->img_sensor_num);
				}
				else
				{
					Sensor_PowerOn(SCI_FALSE);
					_Sensor_I2CDeInit(sensor_id);
					SENSOR_PRINT_HIGH("_Sensor_Identify:identify fail!.\n");
					goto IDENTIFY_SEARCH;
				}
			}
			Sensor_PowerOn(SCI_FALSE);
			_Sensor_I2CDeInit(sensor_id);
			return retValue;
		}
	}
IDENTIFY_SEARCH:
	SENSOR_PRINT_HIGH("_Sensor_Identify:search.\n");
	sensor_info_tab_ptr=(SENSOR_INFO_T**)Sensor_GetInforTab(sensor_id);
	valid_tab_index_max=Sensor_GetInforTabLenght(sensor_id)-SENSOR_ONE_I2C;
	_Sensor_I2CInit(sensor_id);
		  
          //search the sensor in the table
	for(sensor_index=0x00; sensor_index<valid_tab_index_max;sensor_index++)
	{  
		sensor_info_ptr = sensor_info_tab_ptr[sensor_index];

		if(NULL==sensor_info_ptr)
		{
			SENSOR_PRINT_ERR("SENSOR: %d info of Sensor_Init table %d is null", sensor_index, (uint)sensor_id);
			continue ;
		}
		s_sensor_info_ptr = sensor_info_ptr;		
		Sensor_PowerOn(SCI_TRUE);
/*		SENSOR_PRINT_ERR("SENSOR: Sensor_PowerOn done,this_client=0x%x\n",(uint32_t)this_client);
		SENSOR_PRINT_ERR("SENSOR: identify ptr =0x%x\n",(uint32_t)sensor_info_ptr->ioctl_func_tab_ptr->identify);*/

		if(PNULL!=sensor_info_ptr->ioctl_func_tab_ptr->identify)
		{
/*			SENSOR_PRINT_HIGH("SENSOR:identify  Sensor 00:this_client=0x%x,this_client->addr=0x%x,0x%x\n",(uint32_t)this_client,(uint32_t)&this_client->addr,this_client->addr);*/
			if(5 != Sensor_GetCurId()){//test by wang bonnie
				g_i2c_addr =  (s_sensor_info_ptr->salve_i2c_addr_w & 0xFF); 
				_Sensor_Device_SetI2cAddr(g_i2c_addr);
			}
			SENSOR_PRINT_HIGH("SENSOR:identify  Sensor 01\n");
			if(SENSOR_SUCCESS==sensor_info_ptr->ioctl_func_tab_ptr->identify(SENSOR_ZERO_I2C))
			{			         
				s_sensor_list_ptr[sensor_id]=sensor_info_ptr; 
				s_sensor_register_info_ptr->is_register[sensor_id]=SCI_TRUE;
				if(5 != Sensor_GetCurId())//test by wang bonnie
					s_sensor_index[sensor_id] = sensor_index;
				s_sensor_register_info_ptr->img_sensor_num++;
				Sensor_PowerOn(SCI_FALSE);	
				retValue = SCI_TRUE;
				SENSOR_PRINT_HIGH("_Sensor_Identify:sensor_id :%d,img_sensor_num=%d\n",
					                                     sensor_id,s_sensor_register_info_ptr->img_sensor_num);
				break ;
			}
		}
		Sensor_PowerOn(SCI_FALSE);	
	}
         _Sensor_I2CDeInit(sensor_id);
	if(SCI_TRUE == s_sensor_register_info_ptr->is_register[sensor_id])
	{
		SENSOR_PRINT_HIGH("SENSOR TYPE of %d indentify OK",(uint32_t)sensor_id);
		s_sensor_param_saved = SCI_TRUE;
	}
	else
	{
		SENSOR_PRINT_HIGH("SENSOR TYPE of %d indentify FAILURE",(uint32_t)sensor_id);
	}

	return retValue;
}

LOCAL void _Sensor_SetStatus(SENSOR_ID_E sensor_id)
{
	uint32_t i = 0;
	SENSOR_REGISTER_INFO_T_PTR sensor_register_info_ptr =
	    s_sensor_register_info_ptr;

	for (i = 0; i <= SENSOR_SUB; i++) {
		if (i == sensor_id) {
			continue;
		}
		if (SENSOR_TRUE == sensor_register_info_ptr->is_register[i]) {
			_Sensor_SetId(i);
			s_sensor_info_ptr = s_sensor_list_ptr[i];
			if (5 != Sensor_GetCurId()){
				g_i2c_addr =  (s_sensor_info_ptr->salve_i2c_addr_w & 0xFF); 
				_Sensor_Device_SetI2cAddr(g_i2c_addr);
			}

			Sensor_PowerOn(SENSOR_TRUE);
			Sensor_SetExportInfo(&s_sensor_exp_info);
			Sensor_PowerDown((BOOLEAN)
					 s_sensor_info_ptr->power_down_level);
			SENSOR_PRINT_HIGH("SENSOR: Sensor_sleep of id %d", i);
		}
	}
}

LOCAL int _Sensor_DeviceInit()
{
	int                      ret = 0;
	
	SENSOR_PRINT_HIGH("_Sensor_DeviceInit: Start to open sensor device.");

	if(-1 == g_fd_sensor){
		g_fd_sensor = open(dev_name, O_RDWR, 0);
		if (-1 == g_fd_sensor) {
			SENSOR_PRINT_ERR("Failed to open sensor device.errno : %d", errno);
			fprintf(stderr, "Cannot open '%s': %d, %s\n", dev_name, errno,  strerror(errno));   
		} else {
			SENSOR_PRINT_HIGH("#####SENSOR: OK to open device.");       
		}
	}

	ret = pthread_mutex_init(&cb_mutex, NULL);
	if (ret) {
		SENSOR_PRINT("Failed to init mutex : %d", errno);
	}

	sensor_event_cb = NULL;

	return ret;
}

LOCAL int _Sensor_DeviceDeInit()
{
	int ret;
	if(-1 != g_fd_sensor){
		ret = close(g_fd_sensor);
		g_fd_sensor = -1;
		SENSOR_PRINT("SENSOR: _Sensor_DeviceDeInit is done, ret = %d \n", ret);
	}

	return 0;
}

int Sensor_Init(void)
{
	uint32_t sensor_num = 0;

	//SENSOR_PRINT("SENSOR: Sensor_Init, sensor_id: %d \n", sensor_id);

	if (Sensor_IsInit()) {
		SENSOR_PRINT("SENSOR: Sensor_Init is done\n");
		return SENSOR_SUCCESS;
	}

	_Sensor_CleanInformation();
	_Sensor_InitDefaultExifInfo();

	if(_Sensor_DeviceInit()){
		SENSOR_PRINT("SENSOR: _Sensor_DeviceInit error, return \n");
		return SENSOR_SUCCESS;
	}
	
	if(_Sensor_Identify(SENSOR_MAIN))
		sensor_num++;
	
	if(_Sensor_Identify(SENSOR_SUB))
		sensor_num++;
	
	s_sensor_identified = SCI_TRUE;
#if 0 // TODO
	if (5 == sensor_id) {
		SENSOR_Sleep(20);
		if(_Sensor_Identify(SENSOR_ATV))
			sensor_num++;
	}
#endif

	SENSOR_PRINT("SENSOR: Sensor_Init Identify: sensor_num = %d \n", sensor_num);
	return sensor_num;
}

BOOLEAN Sensor_IsInit(void)
{
	return s_sensor_init;
}

int Sensor_Open(uint32_t sensor_id)
{
	uint32_t ret_val = SENSOR_FAIL;
	SENSOR_REGISTER_INFO_T_PTR sensor_register_info_ptr =
	    s_sensor_register_info_ptr;

	if (SENSOR_TRUE == sensor_register_info_ptr->is_register[sensor_id]) {
		_Sensor_SetStatus(sensor_id);
		_Sensor_SetId(sensor_id);
		s_sensor_info_ptr = s_sensor_list_ptr[sensor_id];
		Sensor_SetExportInfo(&s_sensor_exp_info);
		s_sensor_init = SENSOR_TRUE;
		Sensor_PowerOn(SENSOR_TRUE);

		if (5 != Sensor_GetCurId()){
			g_i2c_addr =  (s_sensor_info_ptr->salve_i2c_addr_w & 0xFF); 
			_Sensor_Device_SetI2cAddr(g_i2c_addr);
		}

		SENSOR_PRINT_HIGH("Sensor_Init:sensor_id :%d,addr=0x%x\n", sensor_id, g_i2c_addr);
		ret_val = SENSOR_SUCCESS;
		if (SENSOR_SUCCESS != Sensor_SetMode(SENSOR_MODE_COMMON_INIT)) {
			SENSOR_PRINT_ERR("Sensor set init mode error!\n");
			ret_val = SENSOR_FAIL;
		}
		s_sensor_init = SENSOR_TRUE;
		SENSOR_PRINT("SENSOR: Sensor_Init  Success \n");
	} else {
		SENSOR_PRINT_ERR("Sensor identify fail,sensor_id = %d",
				 sensor_id);
	}

	return ret_val;
}

int Sensor_SetMode(uint32_t mode)
{
	uint32_t mclk;
	SENSOR_IOCTL_FUNC_PTR set_reg_tab_func=s_sensor_info_ptr->ioctl_func_tab_ptr->cus_func_1;

	SENSOR_PRINT("SENSOR: Sensor_SetMode -> mode = %d.\n", mode);
	if (SENSOR_FALSE == Sensor_IsInit()) {
		SENSOR_PRINT
		    ("SENSOR: Sensor_SetResolution -> sensor has not init");
		return SENSOR_OP_STATUS_ERR;
	}

	if (s_sensor_mode[Sensor_GetCurId()] == mode) {
		SENSOR_PRINT("SENSOR: The sensor mode as before");
		return SENSOR_SUCCESS;
	}

	if (PNULL != s_sensor_info_ptr->resolution_tab_info_ptr[mode].sensor_reg_tab_ptr) {
		mclk = s_sensor_info_ptr->resolution_tab_info_ptr[mode].xclk_to_sensor;
		Sensor_SetMCLK(mclk);
		s_sensor_exp_info.image_format =
		    s_sensor_exp_info.sensor_mode_info[mode].image_format;
		Sensor_SendRegTabToSensor
		    (&s_sensor_info_ptr->resolution_tab_info_ptr[mode]);
		s_sensor_mode[Sensor_GetCurId()] = mode;
	} else {
		if(set_reg_tab_func)
			set_reg_tab_func(0);
		SENSOR_PRINT
		    ("SENSOR: Sensor_SetResolution -> No this resolution information !!!");
	}
	return SENSOR_SUCCESS;
}

uint32_t Sensor_Ioctl(uint32_t cmd, uint32_t arg)
{
	SENSOR_IOCTL_FUNC_PTR func_ptr;
	SENSOR_IOCTL_FUNC_TAB_T *func_tab_ptr;
	uint32_t temp;
	uint32_t ret_value = SENSOR_SUCCESS;

	SENSOR_PRINT("SENSOR: Sensor_Ioctl -> cmd = %d, arg = %d.\n", cmd, arg);

	if (!Sensor_IsInit()) {
		SENSOR_PRINT("SENSOR: Sensor_Ioctl -> sensor has not init.\n");
		return SENSOR_OP_STATUS_ERR;
	}

	if (SENSOR_IOCTL_CUS_FUNC_1 > cmd) {
		SENSOR_PRINT
		    ("SENSOR: Sensor_Ioctl - > can't access internal command !\n");
		return SENSOR_SUCCESS;
	}
	func_tab_ptr = s_sensor_info_ptr->ioctl_func_tab_ptr;
	temp = *(uint32_t *) ((uint32_t) func_tab_ptr + cmd * S_BIT_2);
	func_ptr = (SENSOR_IOCTL_FUNC_PTR) temp;

	if (PNULL != func_ptr) {
		//ImgSensor_GetMutex();
		ret_value = func_ptr(arg);
		//ImgSensor_PutMutex();
	} else {
		SENSOR_PRINT
		    ("SENSOR: Sensor_Ioctl -> the ioctl function has not register err!\n");
	}
	return ret_value;
}

SENSOR_EXP_INFO_T *Sensor_GetInfo(void)
{
	if (!Sensor_IsInit()) {
		SENSOR_PRINT("SENSOR: Sensor_GetInfo -> sensor has not init");
		return PNULL;
	}
	return &s_sensor_exp_info;
}

ERR_SENSOR_E Sensor_Close(void)
{
	SENSOR_PRINT("SENSOR: Sensor_close");

	if (1 == g_is_register_sensor) {
		if (1 == g_is_main_sensor) {
			_Sensor_Device_I2CDeInit(SENSOR_MAIN);
		} else {
			_Sensor_Device_I2CDeInit(SENSOR_SUB);
		}
		g_is_register_sensor = 0;
		g_is_main_sensor = 0;
	}

	if (SENSOR_TRUE == Sensor_IsInit()) {
		Sensor_PowerOn(SENSOR_FALSE);
		if (SENSOR_MAIN == Sensor_GetCurId()) {
			SENSOR_PRINT_HIGH("SENSOR: Sensor_close 0.\n");
			if (SCI_TRUE == s_sensor_register_info_ptr->is_register[SENSOR_SUB])
			{
				SENSOR_PRINT_HIGH("SENSOR: Sensor_close 1.\n");
				_Sensor_SetId(SENSOR_SUB);
				s_sensor_info_ptr = s_sensor_list_ptr[SENSOR_SUB];
				Sensor_SetExportInfo(&s_sensor_exp_info);
				Sensor_PowerOn(SENSOR_FALSE);
				if (1 == g_is_register_sensor) {
					SENSOR_PRINT_HIGH ("SENSOR: Sensor_close 2.\n");
					_Sensor_Device_I2CDeInit(SENSOR_SUB);
					g_is_register_sensor = 0;
					g_is_main_sensor = 0;
				}
			}
		} else if (SENSOR_SUB == Sensor_GetCurId()) {
			SENSOR_PRINT_HIGH("SENSOR: Sensor_close 3.\n");
			if (SCI_TRUE ==  s_sensor_register_info_ptr->is_register[SENSOR_MAIN]) {
				SENSOR_PRINT_HIGH("SENSOR: Sensor_close 4.\n");
				_Sensor_SetId(SENSOR_MAIN);
				s_sensor_info_ptr = s_sensor_list_ptr[SENSOR_MAIN];
				Sensor_SetExportInfo(&s_sensor_exp_info);
				Sensor_PowerOn(SENSOR_FALSE);
				if (1 == g_is_register_sensor) {
					SENSOR_PRINT_HIGH ("SENSOR: Sensor_close 5.\n");
					_Sensor_Device_I2CDeInit(SENSOR_MAIN);
					g_is_register_sensor = 0;
					g_is_main_sensor = 0;
				}
			}
		} else if (SENSOR_ATV == Sensor_GetCurId()) {
			if (SCI_TRUE == s_sensor_register_info_ptr->is_register[SENSOR_MAIN]) {
				SENSOR_PRINT_HIGH("SENSOR: Sensor_close 4.\n");
				_Sensor_SetId(SENSOR_MAIN);
				s_sensor_info_ptr = s_sensor_list_ptr[SENSOR_MAIN];
				Sensor_SetExportInfo(&s_sensor_exp_info);
				Sensor_PowerOn(SENSOR_FALSE);
				if (1 == g_is_register_sensor) {
					SENSOR_PRINT_HIGH ("SENSOR: Sensor_close 6.\n");
					_Sensor_Device_I2CDeInit(SENSOR_MAIN);
					g_is_register_sensor = 0;
					g_is_main_sensor = 0;
				}
			}
			if (SCI_TRUE == s_sensor_register_info_ptr->is_register[SENSOR_SUB])
			{
				SENSOR_PRINT_HIGH("SENSOR: Sensor_close 7.\n");
				_Sensor_SetId(SENSOR_SUB);
				s_sensor_info_ptr = s_sensor_list_ptr[SENSOR_SUB];
				Sensor_SetExportInfo(&s_sensor_exp_info);
				Sensor_PowerOn(SENSOR_FALSE);
				if (1 == g_is_register_sensor) {
					SENSOR_PRINT_HIGH ("SENSOR: Sensor_close 8.\n");
					_Sensor_Device_I2CDeInit(SENSOR_SUB);
					g_is_register_sensor = 0;
					g_is_main_sensor = 0;
				}
			}
		}
	}
	SENSOR_PRINT_HIGH("SENSOR: Sensor_close 9.\n");

	_Sensor_DeviceDeInit();
	s_sensor_init = SENSOR_FALSE;
	s_sensor_mode[SENSOR_MAIN] = SENSOR_MODE_MAX;
	s_sensor_mode[SENSOR_SUB] = SENSOR_MODE_MAX;


	return SENSOR_SUCCESS;
}

uint32_t Sensor_SetSensorType(SENSOR_TYPE_E sensor_type)
{
	s_sensor_type = sensor_type;
	return SENSOR_SUCCESS;
}

ERR_SENSOR_E Sensor_SetTiming(SENSOR_MODE_E mode)
{
	uint32_t cur_id = s_sensor_register_info_ptr->cur_id;

	SENSOR_PRINT_HIGH("SENSOR: Sensor_SetTiming -> mode = %d,sensor_id=%d.\n", mode,
	       cur_id);

	if (PNULL !=
	    s_sensor_info_ptr->
	    resolution_tab_info_ptr[mode].sensor_reg_tab_ptr) {
		/*send register value to sensor */
		Sensor_SendRegTabToSensor
		    (&s_sensor_info_ptr->resolution_tab_info_ptr[mode]);
		s_sensor_mode[Sensor_GetCurId()] = mode;
	} else {
		SENSOR_PRINT
		    ("SENSOR: Sensor_SetResolution -> No this resolution information !!!");
	}
	return SENSOR_SUCCESS;
}

int Sensor_CheckTiming(SENSOR_MODE_E mode)
{
	SENSOR_REG_TAB_INFO_T *sensor_reg_tab_info_ptr =
	    &s_sensor_info_ptr->resolution_tab_info_ptr[mode];
	uint32_t i = 0;
	uint16_t data = 0;
	uint32_t cur_id = s_sensor_register_info_ptr->cur_id;
	int ret = SENSOR_SUCCESS;

	SENSOR_PRINT_HIGH("SENSOR: Sensor_CheckTiming -> mode = %d,sensor_id=%d.\n", mode,
	       cur_id);

	if (0 != cur_id)
		return 0;

	for (i = 0; i < sensor_reg_tab_info_ptr->reg_count; i++) {
		if ((0x4202 ==
		     sensor_reg_tab_info_ptr->sensor_reg_tab_ptr[i].reg_addr)
		    || (SENSOR_WRITE_DELAY ==
			sensor_reg_tab_info_ptr->
			sensor_reg_tab_ptr[i].reg_addr))
			continue;
		data =
		    Sensor_ReadReg(sensor_reg_tab_info_ptr->sensor_reg_tab_ptr
				   [i].reg_addr);
		if (data !=
		    sensor_reg_tab_info_ptr->sensor_reg_tab_ptr[i].reg_value) {
			ret = -1;
			SENSOR_PRINT_HIGH("SENSOR: Sensor_CheckTiming report error!.\n");
			break;
		}
	}
	SENSOR_PRINT_HIGH("SENSOR: Sensor_CheckTiming return = %d.\n", ret);
	return ret;
}

uint32_t Sensor_SetFlash(uint32_t flash_mode)
{

	if (s_flash_mode == flash_mode)
		return 0;

	s_flash_mode = flash_mode;
	_Sensor_Device_SetFlash(flash_mode);

	SENSOR_PRINT_HIGH("Sensor_SetFlash:flash_mode=0x%x .\n", flash_mode);

	return 0;
}


LOCAL uint32_t _Sensor_InitDefaultExifInfo(void)
{
	EXIF_SPEC_PIC_TAKING_COND_T* exif_ptr=&s_default_exif;

	memset(&s_default_exif, 0, sizeof(EXIF_SPEC_PIC_TAKING_COND_T));

	SENSOR_PRINT("SENSOR: Sensor_InitDefaultExifInfo \n");

	exif_ptr->valid.FNumber=1;
	exif_ptr->FNumber.numerator=14;
	exif_ptr->FNumber.denominator=5;
	exif_ptr->valid.ExposureProgram=1;
	exif_ptr->ExposureProgram=0x04;
	/*exif_ptr->SpectralSensitivity[MAX_ASCII_STR_SIZE];
	exif_ptr->ISOSpeedRatings;
	exif_ptr->OECF;
	exif_ptr->ShutterSpeedValue;*/
	exif_ptr->valid.ApertureValue=1;
	exif_ptr->ApertureValue.numerator=14;
	exif_ptr->ApertureValue.denominator=5;
	/*exif_ptr->BrightnessValue;
	exif_ptr->ExposureBiasValue;*/
	exif_ptr->valid.MaxApertureValue=1;
	exif_ptr->MaxApertureValue.numerator=14;
	exif_ptr->MaxApertureValue.denominator=5;
	/*exif_ptr->SubjectDistance;
	exif_ptr->MeteringMode;
	exif_ptr->LightSource;
	exif_ptr->Flash;*/
	exif_ptr->valid.FocalLength=1;
	exif_ptr->FocalLength.numerator=289;
	exif_ptr->FocalLength.denominator=100;
	/*exif_ptr->SubjectArea;
	exif_ptr->FlashEnergy;
	exif_ptr->SpatialFrequencyResponse;
	exif_ptr->FocalPlaneXResolution;
	exif_ptr->FocalPlaneYResolution;
	exif_ptr->FocalPlaneResolutionUnit;
	exif_ptr->SubjectLocation[2];
	exif_ptr->ExposureIndex;
	exif_ptr->SensingMethod;*/
	exif_ptr->valid.FileSource=1;
	exif_ptr->FileSource=0x03;
	/*exif_ptr->SceneType;
	exif_ptr->CFAPattern;
	exif_ptr->CustomRendered;*/
	exif_ptr->valid.ExposureMode=1;
	exif_ptr->ExposureMode=0x00;
	exif_ptr->valid.WhiteBalance=1;
	exif_ptr->WhiteBalance=0x00;
	/*exif_ptr->DigitalZoomRatio;
	exif_ptr->FocalLengthIn35mmFilm;
	exif_ptr->SceneCaptureType;
	exif_ptr->GainControl;
	exif_ptr->Contrast;
	exif_ptr->Saturation;
	exif_ptr->Sharpness;
	exif_ptr->DeviceSettingDescription;
	exif_ptr->SubjectDistanceRange;*/
	return SENSOR_SUCCESS;
}
uint32_t Sensor_SetSensorExifInfo(SENSOR_EXIF_CTRL_E cmd, uint32_t param)
{
	SENSOR_EXP_INFO_T_PTR sensor_info_ptr = Sensor_GetInfo();
	EXIF_SPEC_PIC_TAKING_COND_T *sensor_exif_info_ptr = PNULL;

	if (PNULL != sensor_info_ptr->ioctl_func_ptr->get_exif) {
		sensor_exif_info_ptr =
		    (EXIF_SPEC_PIC_TAKING_COND_T *)
		    sensor_info_ptr->ioctl_func_ptr->get_exif(0x00);
	} else {
		sensor_exif_info_ptr = &s_default_exif;
		SENSOR_PRINT
		    ("SENSOR: Sensor_SetSensorExifInfo the get_exif fun is null error \n");
		return SENSOR_FAIL;
	}

	switch (cmd) {
	case SENSOR_EXIF_CTRL_EXPOSURETIME:
		{
			SENSOR_MODE_E img_sensor_mode =
			    s_sensor_mode[Sensor_GetCurId()];
			uint32_t exposureline_time =
			    sensor_info_ptr->
			    sensor_mode_info[img_sensor_mode].line_time;
			uint32_t exposureline_num = param;
			uint32_t exposure_time = 0x00;

			exposure_time = exposureline_time * exposureline_num;
			sensor_exif_info_ptr->valid.ExposureTime = 1;

			if (0x00 == exposure_time) {
				sensor_exif_info_ptr->valid.ExposureTime = 0;
			} else if (1000000 >= exposure_time) {
				sensor_exif_info_ptr->ExposureTime.numerator =
				    0x01;
				sensor_exif_info_ptr->ExposureTime.denominator =
				    1000000 / exposure_time;
			} else {
				uint32_t second = 0x00;
				do {
					second++;
					exposure_time -= 1000000;
					if (1000000 >= exposure_time) {
						break;
					}
				} while (1);
				sensor_exif_info_ptr->ExposureTime.denominator =
				    1000000 / exposure_time;
				sensor_exif_info_ptr->ExposureTime.numerator =
				    sensor_exif_info_ptr->
				    ExposureTime.denominator * second;
			}
			break;
		}
	case SENSOR_EXIF_CTRL_FNUMBER:
		break;
	case SENSOR_EXIF_CTRL_EXPOSUREPROGRAM:
		break;
	case SENSOR_EXIF_CTRL_SPECTRALSENSITIVITY:
		break;
	case SENSOR_EXIF_CTRL_ISOSPEEDRATINGS:
		break;
	case SENSOR_EXIF_CTRL_OECF:
		break;
	case SENSOR_EXIF_CTRL_SHUTTERSPEEDVALUE:
		break;
	case SENSOR_EXIF_CTRL_APERTUREVALUE:
		break;
	case SENSOR_EXIF_CTRL_BRIGHTNESSVALUE:
		break;
	case SENSOR_EXIF_CTRL_EXPOSUREBIASVALUE:
		break;
	case SENSOR_EXIF_CTRL_MAXAPERTUREVALUE:
		break;
	case SENSOR_EXIF_CTRL_SUBJECTDISTANCE:
		break;
	case SENSOR_EXIF_CTRL_METERINGMODE:
		break;
	case SENSOR_EXIF_CTRL_LIGHTSOURCE:
		{
			sensor_exif_info_ptr->valid.LightSource = 1;
			switch (param) {
			case 0:
				sensor_exif_info_ptr->LightSource = 0x00;
				break;
			case 1:
				sensor_exif_info_ptr->LightSource = 0x03;
				break;
			case 2:
				sensor_exif_info_ptr->LightSource = 0x0f;
				break;
			case 3:
				sensor_exif_info_ptr->LightSource = 0x0e;
				break;
			case 4:
				sensor_exif_info_ptr->LightSource = 0x03;
				break;
			case 5:
				sensor_exif_info_ptr->LightSource = 0x01;
				break;
			case 6:
				sensor_exif_info_ptr->LightSource = 0x0a;
				break;
			default:
				sensor_exif_info_ptr->LightSource = 0xff;
				break;
			}
			break;
		}
	case SENSOR_EXIF_CTRL_FLASH:
		break;
	case SENSOR_EXIF_CTRL_FOCALLENGTH:
		break;
	case SENSOR_EXIF_CTRL_SUBJECTAREA:
		break;
	case SENSOR_EXIF_CTRL_FLASHENERGY:
		break;
	case SENSOR_EXIF_CTRL_SPATIALFREQUENCYRESPONSE:
		break;
	case SENSOR_EXIF_CTRL_FOCALPLANEXRESOLUTION:
		break;
	case SENSOR_EXIF_CTRL_FOCALPLANEYRESOLUTION:
		break;
	case SENSOR_EXIF_CTRL_FOCALPLANERESOLUTIONUNIT:
		break;
	case SENSOR_EXIF_CTRL_SUBJECTLOCATION:
		break;
	case SENSOR_EXIF_CTRL_EXPOSUREINDEX:
		break;
	case SENSOR_EXIF_CTRL_SENSINGMETHOD:
		break;
	case SENSOR_EXIF_CTRL_FILESOURCE:
		break;
	case SENSOR_EXIF_CTRL_SCENETYPE:
		break;
	case SENSOR_EXIF_CTRL_CFAPATTERN:
		break;
	case SENSOR_EXIF_CTRL_CUSTOMRENDERED:
		break;
	case SENSOR_EXIF_CTRL_EXPOSUREMODE:
		break;
		
	case SENSOR_EXIF_CTRL_WHITEBALANCE:
		sensor_exif_info_ptr->valid.WhiteBalance = 1;
		if(param)
			sensor_exif_info_ptr->WhiteBalance = 1;
		else
			sensor_exif_info_ptr->WhiteBalance = 0;
		break;
		
	case SENSOR_EXIF_CTRL_DIGITALZOOMRATIO:
		break;
	case SENSOR_EXIF_CTRL_FOCALLENGTHIN35MMFILM:
		break;
	case SENSOR_EXIF_CTRL_SCENECAPTURETYPE:
		{
			sensor_exif_info_ptr->valid.SceneCaptureType = 1;
			switch (param) {
			case 0:
				sensor_exif_info_ptr->SceneCaptureType = 0x00;
				break;
			case 1:
				sensor_exif_info_ptr->SceneCaptureType = 0x03;
				break;
			default:
				sensor_exif_info_ptr->LightSource = 0xff;
				break;
			}
			break;
		}
	case SENSOR_EXIF_CTRL_GAINCONTROL:
		break;
	case SENSOR_EXIF_CTRL_CONTRAST:
		{
			sensor_exif_info_ptr->valid.Contrast = 1;
			switch (param) {
			case 0:
			case 1:
			case 2:
				sensor_exif_info_ptr->Contrast = 0x01;
				break;
			case 3:
				sensor_exif_info_ptr->Contrast = 0x00;
				break;
			case 4:
			case 5:
			case 6:
				sensor_exif_info_ptr->Contrast = 0x02;
				break;
			default:
				sensor_exif_info_ptr->Contrast = 0xff;
				break;
			}
			break;
		}
	case SENSOR_EXIF_CTRL_SATURATION:
		{
			sensor_exif_info_ptr->valid.Saturation = 1;
			switch (param) {
			case 0:
			case 1:
			case 2:
				sensor_exif_info_ptr->Saturation = 0x01;
				break;
			case 3:
				sensor_exif_info_ptr->Saturation = 0x00;
				break;
			case 4:
			case 5:
			case 6:
				sensor_exif_info_ptr->Saturation = 0x02;
				break;
			default:
				sensor_exif_info_ptr->Saturation = 0xff;
				break;
			}
			break;
		}
	case SENSOR_EXIF_CTRL_SHARPNESS:
		{
			sensor_exif_info_ptr->valid.Sharpness = 1;
			switch (param) {
			case 0:
			case 1:
			case 2:
				sensor_exif_info_ptr->Sharpness = 0x01;
				break;
			case 3:
				sensor_exif_info_ptr->Sharpness = 0x00;
				break;
			case 4:
			case 5:
			case 6:
				sensor_exif_info_ptr->Sharpness = 0x02;
				break;
			default:
				sensor_exif_info_ptr->Sharpness = 0xff;
				break;
			}
			break;
		}
	case SENSOR_EXIF_CTRL_DEVICESETTINGDESCRIPTION:
		break;
	case SENSOR_EXIF_CTRL_SUBJECTDISTANCERANGE:
		break;
	default:
		break;
	}
	return SENSOR_SUCCESS;
}

EXIF_SPEC_PIC_TAKING_COND_T *Sensor_GetSensorExifInfo(void)
{
	SENSOR_EXP_INFO_T_PTR sensor_info_ptr = Sensor_GetInfo();
	EXIF_SPEC_PIC_TAKING_COND_T *sensor_exif_info_ptr = PNULL;

	if (PNULL != sensor_info_ptr->ioctl_func_ptr->get_exif) {
		sensor_exif_info_ptr =
		    (EXIF_SPEC_PIC_TAKING_COND_T *)
		    sensor_info_ptr->ioctl_func_ptr->get_exif(0x00);
	} else {
		sensor_exif_info_ptr = &s_default_exif;
		SENSOR_PRINT("SENSOR: Sensor_GetSensorExifInfo the get_exif fun is null, so use the default exif info.\n");
	}
	return sensor_exif_info_ptr;
}

int Sensor_SetSensorParam(uint8_t *buf)
{
	uint32_t i;

	if((SIGN_0 != buf[0]) && (SIGN_1 != buf[1])
	   && (SIGN_2 != buf[2]) && (SIGN_3 != buf[3])) {
		s_sensor_identified = SCI_FALSE;
	} else {
		s_sensor_identified = SCI_TRUE;
		for( i=0 ; i<2 ; i++) {
			s_sensor_index[i] = buf[4+i];
		}
	}
	SENSOR_PRINT_HIGH("Sensor_SetSensorParam:s_sensor_identified=%d,idex is %d,%d.\n",
		    s_sensor_identified,s_sensor_index[0],s_sensor_index[1]);
	return 0;
}

int Sensor_GetSensorParam(uint8_t *buf,uint8_t *is_saved_ptr)
{
	uint32_t i,j=0;
	uint8_t *ptr=buf;

	if(SCI_TRUE == s_sensor_param_saved) {
		*is_saved_ptr = 1;
		*ptr++ = SIGN_0;
		*ptr++ = SIGN_1;
		*ptr++ = SIGN_2;
		*ptr++ = SIGN_3;
		for( i=0 ; i<2 ; i++) {
			*ptr++ = s_sensor_index[i];
		}
		SENSOR_PRINT_HIGH("Sensor_GetSensorParam:index is %d,%d.\n",s_sensor_index[0],s_sensor_index[1]);
	} else {
		*is_saved_ptr = 0;
	}
	return 0;
}

int Sensor_WriteRegs(uint8_t *regPtr, uint32_t length)
{
	int ret;
	ret = _Sensor_Device_Write(regPtr, length);
	
	return ret;
}

int Sensor_EventReg(cmr_evt_cb  event_cb)
{
	pthread_mutex_lock(&cb_mutex);
	sensor_event_cb = event_cb;
	pthread_mutex_unlock(&cb_mutex);
	return 0;
}

int Sensor_GetRawSettings(void **raw_setting, uint32_t *length)
{
	return 0;
}
