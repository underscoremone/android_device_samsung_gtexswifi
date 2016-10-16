#ifndef __CALIBRATION_H__

#define	BATTERY_VOL_PATH	"/sys/class/power_supply/battery/real_time_voltage"
#define	BATTERY_ADC_PATH	"/sys/class/power_supply/battery/real_time_vbat_adc"
#define	CALI_CTRL_FILE_PATH	"/productinfo/adc.bin"
#define	CHARGER_STOP_PATH	"/sys/class/power_supply/battery/stop_charge"
#define ADC_CHAN_FILE_PATH      "/sys/kernel/debug/sc2713-regulator/adc_chan"
#define	FGU_CURRENT_ADC_FILE_PATH	"/sys/class/power_supply/sprdfgu/fgu_current_adc"
#define	FGU_VOL_ADC_FILE_PATH	"/sys/class/power_supply/sprdfgu/fgu_vol_adc"
#define	FGU_CURRENT_FILE_PATH	"/sys/class/power_supply/sprdfgu/fgu_current"
#define	FGU_VOL_FILE_PATH	"/sys/class/power_supply/sprdfgu/fgu_vol"
#define	BATTER_CALI_CONFIG_FILE	CALI_CTRL_FILE_PATH
typedef enum
{
    DIAG_AP_CMD_ADC  = 0x0001,
    DIAG_AP_CMD_LOOP,
    DIAG_AP_CMD_FILE_OPER,
    MAX_DIAG_AP_CMD
} DIAG_AP_CMD_E;


#define AP_ADC_CALIB    1
#define AP_ADC_LOAD     2
#define AP_ADC_SAVE     3
#define	AP_GET_VOLT	4
#define AP_DIAG_LOOP	5
#define AP_GET_FGU_VOL_ADC	6
#define AP_GET_FGU_CURRENT_ADC	7
#define AP_GET_FGU_TYPE	8
#define AP_GET_FGU_VOL_REAL 9
#define AP_GET_FGU_CURRENT_REAL 10


#define	CALI_MAGIC	(0x49424143) //CALI
#define	CALI_COMP	(0x504D4F43) //COMP

typedef struct
{
    unsigned int     	adc[2];           // calibration of ADC, two test point
    unsigned int 	battery[2];       // calibraton of battery(include resistance), two test point
    unsigned int    	reserved[8];      // reserved for feature use.
} AP_ADC_T;

typedef struct
{
    unsigned int	magic;		  // when create ,magic = "CALI"
    unsigned int	cali_flag;        // cali_flag   default 0xFFFFFFFF, when calibration finished,it is set "COMP"
    AP_ADC_T 	adc_para;         // ADC calibration data.
}CALI_INFO_DATA_T;

typedef struct {
    unsigned short  cmd;        // DIAG_AP_CMD_E
    unsigned short  length;   // Length of structure
} TOOLS_DIAG_AP_CMD_T;

typedef struct {
    unsigned int operate; // 0: Get ADC   1: Load ADC    2: Save ADC
    unsigned int parameters[12];
}TOOLS_AP_ADC_REQ_T;

typedef struct {
    unsigned short status;   // ==0: success, != 0: fail
    unsigned short length;   // length of  result
} TOOLS_DIAG_AP_CNF_T;

typedef struct
{
    MSG_HEAD_T  msg_head;
    TOOLS_DIAG_AP_CNF_T diag_ap_cnf;
    TOOLS_AP_ADC_REQ_T ap_adc_req;
}MSG_AP_ADC_CNF;

void initialize_ctrl_file(void);

#define __CALIBRATION_H__

#endif

