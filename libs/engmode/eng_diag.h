#ifndef __ENG_DIAG_H__
#define __ENG_DIAG_H__

/*got it from tool*/
#define DIAG_CHANGE_MODE_F  12
typedef enum
{
    NORMAL_MODE                         = 0,
    LAYER1_TEST_MODE                    = 1,
    ASSERT_BACK_MODE                    = 2,
    CALIBRATION_MODE                    = 3,
    DSP_CODE_DOWNLOAD_BACK              = 4,
    DSP_CODE_DOWNLOAD_BACK_CALIBRATION  = 5,
    BOOT_RESET_MODE                     = 6,
    PRODUCTION_MODE                     = 7,
    RESET_MODE                          = 8,
    CALIBRATION_POST_MODE               = 9,
    PIN_TEST_MODE                       = 10,
    IQC_TEST_MODE                       = 11,
    WATCHDOG_RESET_MODE                 = 12,

    CALIBRATION_NV_ACCESS_MODE          = 13,
    CALIBRATION_POST_NO_LCM_MODE        = 14,

    TD_CALIBRATION_POST_MODE            = 15,
    TD_CALIBRATION_MODE                 = 16,
    TD_CALIBRATION_POST_NO_LCM_MODE     = 17,

    MODE_MAX_TYPE,

    MODE_MAX_MASK                       = 0x7F

}MCU_MODE_E;

#define MAX_IMEI_LENGTH		8
#define MAX_BTADDR_LENGTH	6
#define MAX_WIFIADDR_LENGTH	6
#define GPS_NVINFO_LENGTH	44
#define DIAG_HEADER_LENGTH	8

#define DIAG_CMD_VER		0x00
#define DIAG_CMD_IMEIBTWIFI	0x5E
#define DIAG_CMD_READ		0x80
#define DIAG_CMD_GETVOLTAGE	0x1E
#define DIAG_CMD_APCALI		0x62
#define DIAG_CMD_FACTORYMODE	0x0D
#define DIAG_CMD_ADC_F		0x0F  //add by kenyliu on 2013 07 12 for get ADCV  bug 188809
#define DIAG_CMD_AT 		0x68
#define DIAG_CMD_CHANGEMODE     DIAG_CHANGE_MODE_F

#define DIAG_CMD_DIRECT_PHSCHK  0x5F

#define DIAG_CMD_IMEIBIT		0x01
#define DIAG_CMD_BTBIT			0x04
#define DIAG_CMD_WIFIBIT		0x40
#define DIAG_CMD_AUTOTEST       0x38

#define DIAG_CMD_CURRENT_TEST       0x11

#define DIAG_CMD_ASSERT         0x5

#define AUDIO_NV_ARM_INDI_FLAG          0x02
#define AUDIO_ENHA_EQ_INDI_FLAG         0x04
#define AUDIO_DATA_READY_INDI_FLAG      (AUDIO_NV_ARM_INDI_FLAG|AUDIO_ENHA_EQ_INDI_FLAG)

#define DIAG_SUB_MMICIT_READ    0x1A 

#define ENG_TESTMODE			"engtestmode"
#define ENG_SPRD_VERS			"ro.build.description"

typedef enum
{
    CMD_COMMON=-1,
    CMD_USER_VER,
    CMD_USER_BTWIFI,
    CMD_USER_FACTORYMODE,
    CMD_USER_AUDIO,
    CMD_USER_RESET,
    CMD_USER_GETVOLTAGE,
    CMD_USER_APCALI,
    CMD_USER_APCMD,
    CMD_USER_ADC,
    CMD_USER_PRODUCT_CTRL,
    CMD_USER_DIRECT_PHSCHK,
    CMD_USER_MMICIT_READ,
    CMD_USER_DEEP_SLEEP,
    CMD_USER_FILE_OPER,
    CMD_USER_SHUT_DOWN,
    CMD_USER_AUTOTEST,
    CMD_USER_AUTOTEST_PATH_CONFIRM = 0x1c,
    CMD_INVALID
}DIAG_CMD_TYPE;
typedef enum
{
    CMD_AUTOTEST_DUMMY,
    CMD_AUTOTEST_KEYPAD,
    CMD_AUTOTEST_LCD_PARALLEL,
    CMD_AUTOTEST_LCD_SPI,
    CMD_AUTOTEST_CAMERA_IIC,
    CMD_AUTOTEST_CAMERA_PARALLEL,
    CMD_AUTOTEST_CAMERA_SPI,
    CMD_AUTOTEST_GPIO, // and TP test
    CMD_AUTOTEST_TF,
    CMD_AUTOTEST_SIM,
    CMD_AUTOTEST_MIC,
    CMD_AUTOTEST_SPEAK,
    CMD_AUTOTEST_MISC,
    CMD_AUTOTEST_FM,
    CMD_AUTOTEST_ATV,
    CMD_AUTOTEST_BT,
    CMD_AUTOTEST_WIFI,
    CMD_AUTOTEST_IIC_DEV,
    CMD_AUTOTEST_CHARGE,
	//-- [[
	CMD_AUTOTEST_RSV01,  // 19
	CMD_AUTOTEST_RSV02,  // 20
	CMD_AUTOTEST_SENSOR, // 21
	//-- ]]
    CMD_AUTOTEST_GPS,
    CMD_AUTOTEST_END
}DIAG_AUTOTEST_CMD_TYPE;
struct eng_autotestcmd_str{
    int index;
    int (*cmd_hdlr)(char *, char *);
};

#define ENG_DIAG_SIZE 4096
#define ENG_LINUX_VER	"/proc/version"
#define ENG_ANDROID_VER "ro.build.version.release"
#define ENG_AUDIO       "/sys/class/vbc_param_config/vbc_param_store"
#define ENG_FM_DEVSTAT	"/sys/class/fm_devstat_config/fm_devstat_store"

#define MAX_SN_LEN                  24
#define MAX_STATION_NUM             15
#define MAX_STATION_NAME_LEN        10
#define MAX_LAST_DESCRIPTION_LEN    32

#define RW_MASK                     0x80 //(BIT_7)
#define WRITE_MODE                  0
#define RM_VALID_CMD_MASK           0x7f

#define MSG_NACK                    0
#define MSG_ACK                     1

#define ENG_MAX_NAME_LEN            260
#define MAX_DIAG_TRANSMIT_FILE_LEN  8192

typedef enum{
    IMEI_ERR_NONE = 0,
    IMEI_CRC_ERR,
    IMEI_CMD_ERR,
    IMEI_SAVE_ERR,
    IMEI_READ_ERR
}ERR_IMEI_E;
// This is the communication frame head
typedef struct msg_head_tag
{
    unsigned int  seq_num;      // Message sequence number, used for flow control
    unsigned short  len;          // The totoal size of the packet "sizeof(MSG_HEAD_T)
    // + packet size"
    unsigned char   type;         // Main command type
    unsigned char   subtype;      // Sub command type
}__attribute__((packed)) MSG_HEAD_T;

typedef struct {
    unsigned char byEngineSn[24];
    unsigned int  dwMapVersion;
    unsigned char byActivatecode[16];
}GPS_NV_INFO_T;

typedef struct {
    unsigned char imei1[MAX_IMEI_LENGTH];
    unsigned char imei2[MAX_IMEI_LENGTH];
    unsigned char btaddr[MAX_BTADDR_LENGTH];
    unsigned char gpsinfo[GPS_NVINFO_LENGTH];
    unsigned char wifiaddr[MAX_WIFIADDR_LENGTH];
    unsigned char reserved1[2];
    unsigned char imei3[MAX_IMEI_LENGTH];
    unsigned char imei4[MAX_IMEI_LENGTH];
    unsigned char reserved2[16];
}REF_NVWriteDirect_T;

typedef struct _PHASE_CHECK_HEADER
{
    unsigned int Magic; //"SP09"
    unsigned char SN[MAX_SN_LEN];   //SN,SN_LEN=24
    unsigned char SN2[MAX_SN_LEN];  //Add for Mobile

    unsigned int StationNum;   //The test station number of the testing
    unsigned char StationName[MAX_STATION_NUM][MAX_STATION_NAME_LEN];

    unsigned char Reserved[13]; //value: 0
    unsigned char SignFlag; // internal flag
    char szLastFailDescription[MAX_LAST_DESCRIPTION_LEN];
    unsigned short iTestSign; // Bit0~Bit14 --> station0 ~ station14 if tested. 0:tested,1:not tested.
    unsigned short iItem; // Part1:Bit0~Bit14 indicate test station,0:pass,1:fail
    // Part2:Bit15 set to 0;
}TEST_TRACK_HEADER_T;

typedef struct _PHASE_CHECK_S
{
    TEST_TRACK_HEADER_T header;
}TEST_DATA_INFO_T;

typedef struct
{
    unsigned int file_cmd;
    unsigned char file_name[ENG_MAX_NAME_LEN];
}__attribute__((packed))TOOLS_DIAG_AP_FILEOPER_REQ_T;

typedef struct
{
    unsigned int file_size;
}TOOLS_DIAG_AP_FILE_STATUS_T;

typedef struct
{
    unsigned int status; // 0: finished, 1: under reading or writing
    unsigned int data_len;// specifies the data length to read/write
    unsigned char data[MAX_DIAG_TRANSMIT_FILE_LEN];
}__attribute__((packed))TOOLS_DIAG_AP_FILE_DATA_T;

int eng_diag(char *buf,int len);
int eng_diag_writeimei(char *req, char *rsp);
void *eng_vlog_thread(void *x);
void *eng_vdiag_wthread(void *x);
void *eng_vdiag_rthread(void *x);
void * eng_sd_log(void * args);

#endif
