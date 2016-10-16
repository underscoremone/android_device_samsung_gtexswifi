#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
#include <utils/Log.h>
#include <hardware/hardware.h>
#include <hardware/bluetooth.h>
#include <pthread.h>
#include <ctype.h>
#include <sys/prctl.h>
#include <sys/capability.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <private/android_filesystem_config.h>
#include <android/log.h>
#include <hardware/hardware.h>
#include <hardware/bluetooth.h>

#ifdef LOG_TAG
#undef  LOG_TAG
#endif

#define LOG_TAG "BTENG"

#define BTENG_DEBUG
#ifdef BTENG_DEBUG
#define BTENG_LOGD(x...) ALOGD( x )
#define BTENG_LOGE(x...) ALOGE( x )
#else
#define BTENG_LOGD(x...) do {} while(0)
#define BTENG_LOGE(x...) do {} while(0)
#endif

#define OK_STR "OK"
#define FAIL_STR "FAIL"
#define CMD_RESULT_BUFFER_LEN (128)
#define CASE_RETURN_STR(const) case const: return #const;

typedef enum
{
    BT_ENG_NONE_ERROR,
    BT_ENG_CMD_INVALID,        
    BT_ENG_PARA_INVALID,
    BT_ENG_HAL_LOAD_ERROR,
    BT_ENG_INIT_ERROR,
    BT_ENG_ENABLE_ERROR,
    BT_ENG_DISABLE_ERROR,
    BT_ENG_SET_DUT_MODE_ERROR,
    BT_ENG_CLEANUP_ERROR
}BT_ENG_ERROR_E;

/************************************************************************************
**  Static variables
************************************************************************************/

static unsigned char main_done = 0;
static bt_status_t status;
static bluetooth_device_t* bt_device;
const bt_interface_t* sBtInterface = NULL;

static gid_t groups[] = { AID_NET_BT, AID_INET, AID_NET_BT_ADMIN,
                  AID_SYSTEM, AID_MISC, AID_SDCARD_RW,
                  AID_NET_ADMIN, AID_VPN};

/* Set to 1 when the Bluedroid stack is enabled */
static unsigned char bt_enabled = 0;
static int bteng_client_fd = -1;
static unsigned char bt_hal_load = 0;
static unsigned char bt_init = 0;
/************************************************************************************
**  Static functions
************************************************************************************/
static void bteng_check_return_status(bt_status_t status);
static BT_ENG_ERROR_E bteng_dut_mode_configure(char *p);
static int bteng_send_back_cmd_result(int client_fd, char *str, int isOK);
static void bteng_hal_unload(void);
static void bteng_cleanup(void);
/************************************************************************************
**  Functions
************************************************************************************/
static const char* bteng_dump_bt_status(bt_status_t status)
{
    switch(status)
    {
        CASE_RETURN_STR(BT_STATUS_SUCCESS)
        CASE_RETURN_STR(BT_STATUS_FAIL)
        CASE_RETURN_STR(BT_STATUS_NOT_READY)
        CASE_RETURN_STR(BT_STATUS_NOMEM)
        CASE_RETURN_STR(BT_STATUS_BUSY)
        CASE_RETURN_STR(BT_STATUS_UNSUPPORTED)

        default:
            return "unknown status code";
    }
}

static void bteng_adapter_state_changed(bt_state_t state)
{
    char dut_mode;

    BTENG_LOGD("ADAPTER STATE UPDATED : %s", (state == BT_STATE_OFF)?"OFF":"ON");
    if (state == BT_STATE_ON)
    {
        bt_enabled = 1;
        dut_mode = '1';
        bteng_dut_mode_configure(&dut_mode);
        bteng_send_back_cmd_result(bteng_client_fd, "enter eut mode ok\n", 1);
    } 
    else
    {
        bt_enabled = 0;
        //bteng_cleanup();
        //bteng_hal_unload();

        //send cmd result
        bteng_send_back_cmd_result(bteng_client_fd, "exit eut mode ok\n", 1);
    }
}

static void bteng_dut_mode_recv(uint16_t opcode, uint8_t *buf, uint8_t len)
{
    BTENG_LOGD("DUT MODE RECV : NOT IMPLEMENTED");
}

static void bteng_le_test_mode(bt_status_t status, uint16_t packet_count)
{
    BTENG_LOGD("LE TEST MODE END status:%s number_of_packets:%d", bteng_dump_bt_status(status), packet_count);
}

static void bteng_thread_evt_cb(bt_cb_thread_evt event) 
{
    if (event  == ASSOCIATE_JVM) 
    {
        BTENG_LOGD("Callback thread ASSOCIATE_JVM");
    }
    else if (event == DISASSOCIATE_JVM) 
    {
        BTENG_LOGD("Callback thread DISASSOCIATE_JVM");
    }

    return;
}

static bt_callbacks_t bt_callbacks = {
        sizeof(bt_callbacks_t),
        bteng_adapter_state_changed,
        NULL, /*adapter_properties_cb */
        NULL, /* remote_device_properties_cb */
        NULL, /* device_found_cb */
        NULL, /* discovery_state_changed_cb */
        NULL, /* pin_request_cb  */
        NULL, /* ssp_request_cb  */
        NULL, /*bond_state_changed_cb */
        NULL, /* acl_state_changed_cb */
        bteng_thread_evt_cb, /* thread_evt_cb */
        bteng_dut_mode_recv, /*dut_mode_recv_cb */
        //    NULL, /*authorize_request_cb */
#if BLE_INCLUDED == TRUE
        bteng_le_test_mode /* le_test_mode_cb */
#else
        NULL
#endif
};

/*******************************************************************************
** Load stack lib
*******************************************************************************/
static BT_ENG_ERROR_E bteng_hal_load(void)
{
    int err = 0;
    BT_ENG_ERROR_E ret_val;

    hw_module_t* module;
    hw_device_t* device;

    if(bt_hal_load)
        return BT_ENG_NONE_ERROR;
    
    BTENG_LOGD("Loading HAL lib");

    err = hw_get_module(BT_HARDWARE_MODULE_ID, (hw_module_t const**)&module);
    if (err == 0)
    {
        err = module->methods->open(module, BT_HARDWARE_MODULE_ID, &device);
        if (err == 0) 
        {
            bt_device = (bluetooth_device_t *)device;
            sBtInterface = bt_device->get_bluetooth_interface();
        }
    }

    BTENG_LOGD("HAL library loaded (%s)", strerror(err));

    if(err == 0)
    {
        ret_val = BT_ENG_NONE_ERROR;
        bt_hal_load = 1;
    }
    else
    {
        ret_val = BT_ENG_HAL_LOAD_ERROR;
    }
    
    return ret_val;
}

static void bteng_hal_unload(void)
{
    BTENG_LOGD("Unloading HAL lib");

    sBtInterface = NULL;
    bt_hal_load = 0;

    BTENG_LOGD("HAL library unloaded Success");

    return;
}

static void bteng_config_permissions(void)
{
    struct __user_cap_header_struct header;
    struct __user_cap_data_struct cap;

    BTENG_LOGD("set_aid_and_cap : pid %d, uid %d gid %d", getpid(), getuid(), getgid());

    header.pid = 0;

    prctl(PR_SET_KEEPCAPS, 1, 0, 0, 0);

    setgid(AID_BLUETOOTH);

    header.version = _LINUX_CAPABILITY_VERSION;

    cap.effective = cap.permitted =  cap.inheritable =
                1 << CAP_NET_RAW |
                1 << CAP_NET_ADMIN |
                1 << CAP_NET_BIND_SERVICE |
                1 << CAP_SYS_RAWIO |
                1 << CAP_SYS_NICE |
                1 << CAP_SETGID;

    capset(&header, &cap);
    setgroups(sizeof(groups)/sizeof(groups[0]), groups);
}

static BT_ENG_ERROR_E bteng_bt_init(void)
{
    bt_status_t status;
    BT_ENG_ERROR_E ret_val;

    if(bt_init)
    return   BT_ENG_NONE_ERROR;
    
    BTENG_LOGD("INIT BT ");
    status = sBtInterface->init(&bt_callbacks);

    bteng_check_return_status(status);

    if(status == BT_STATUS_SUCCESS)
    {
        ret_val = BT_ENG_NONE_ERROR;
        bt_init = 1;
    }
    else
    {
        ret_val = BT_ENG_INIT_ERROR;
    }       

    return ret_val;
}

static BT_ENG_ERROR_E bteng_bt_enable(void)
{
    bt_status_t status;
    BT_ENG_ERROR_E ret_val;
    
    BTENG_LOGD("ENABLE BT");

    if (bt_enabled)
    {
        BTENG_LOGD("Bluetooth is already enabled");
        return BT_ENG_ENABLE_ERROR;
    }
    status = sBtInterface->enable();

    bteng_check_return_status(status);

    if(status == BT_STATUS_SUCCESS)
    {
        ret_val = BT_ENG_NONE_ERROR;
    }
    else
    {
        ret_val = BT_ENG_ENABLE_ERROR;
    }       

    return ret_val;
}

static BT_ENG_ERROR_E bteng_bt_disable(void)
{
    bt_status_t status;
    BT_ENG_ERROR_E ret_val;
    
    BTENG_LOGD("DISABLE BT");
    if (!bt_enabled) 
    {
        BTENG_LOGD("Bluetooth is already disabled");
        return BT_ENG_DISABLE_ERROR;
    }
    status = sBtInterface->disable();

    bteng_check_return_status(status);
    
    if(status == BT_STATUS_SUCCESS)
    {
        ret_val = BT_ENG_NONE_ERROR;
    }
    else
    {
        ret_val = BT_ENG_DISABLE_ERROR;
    }
    
    return ret_val;
}

static BT_ENG_ERROR_E bteng_dut_mode_configure(char *p)
{
    int32_t mode = -1;
    bt_status_t status;
    BT_ENG_ERROR_E ret_val;

    BTENG_LOGD("BT DUT MODE CONFIGURE");
    if (!bt_enabled) 
    {
        BTENG_LOGD("Bluetooth must be enabled for test_mode to work.");
        return BT_ENG_SET_DUT_MODE_ERROR;
    }

    if(*p == '0')
    {
        mode = 0;
    }
    else if(*p == '1')
    {
        mode = 1;
    }

    //mode = get_signed_int(&p, mode);
    if ((mode != 0) && (mode != 1)) 
    {
        BTENG_LOGD("Please specify mode: 1 to enter, 0 to exit");
        return BT_ENG_SET_DUT_MODE_ERROR;
    }
    status = sBtInterface->dut_mode_configure(mode);

    bteng_check_return_status(status);

    if(status == BT_STATUS_SUCCESS)
    {
        ret_val = BT_ENG_NONE_ERROR;
    }
    else
    {
        ret_val = BT_ENG_SET_DUT_MODE_ERROR;
    }

    return ret_val;
}

static void bteng_cleanup(void)
{
    BTENG_LOGD("CLEANUP");
    sBtInterface->cleanup();
}

static void bteng_check_return_status(bt_status_t status)
{
    if (status != BT_STATUS_SUCCESS)
    {
        BTENG_LOGD("HAL REQUEST FAILED status : %d (%s)", status, bteng_dump_bt_status(status));
    }
    else
    {
        BTENG_LOGD("HAL REQUEST SUCCESS");
    }
}

static int bteng_send_back_cmd_result(int client_fd, char *str, int isOK)
{
    char buffer[255];

    BTENG_LOGD("SEND CMD RESULT");
    if(client_fd < 0)
    {
        fprintf(stderr, "write %s to invalid fd \n", str);

        return -1;
    }

    memset(buffer, 0, sizeof(buffer));

    if(!str)
    {
        snprintf(buffer, 255, "%s",  (isOK?OK_STR:FAIL_STR));
    }
    else
    {
        snprintf(buffer, 255, "%s %s", (isOK?OK_STR:FAIL_STR), str);
    }

    int ret = write(client_fd, buffer, strlen(buffer)+1);
    if(ret < 0)
    {
        fprintf(stderr, "write %s to client_fd:%d fail (error:%s)", buffer, client_fd, strerror(errno));
        return -1;
    }

    return 0;
}

static BT_ENG_ERROR_E bteng_dut_mode_set( char **argv)
{
    char dut_mode;
    BT_ENG_ERROR_E err = BT_ENG_NONE_ERROR;
    
    if((*argv)[0] == '1')
    {
        BTENG_LOGD("dut_mode_set enable\n");
        bteng_config_permissions();
        err = bteng_hal_load();
        if(err == BT_ENG_NONE_ERROR)
        {
            err = bteng_bt_init();
            if(err == BT_ENG_NONE_ERROR)
            {
                err = bteng_bt_enable();

                //bteng_dut_mode_configure will bte called after bluetooth on.
                //cmd result will send after bluetooth on.
            }
        }
    }
    else
    {
        BTENG_LOGD("dut_mode_set disable\n");

        dut_mode = '0';
        err = bteng_dut_mode_configure(&dut_mode);
        if(err == BT_ENG_NONE_ERROR)
        {
            err = bteng_bt_disable();
            //cmd result will send after bluetooth off.
        }            
    }

    return err;
}

/**
* cmd: bt cmd arg1 arg2 ...
*/
int bt_runcommand(int client_fd, int argc, char **argv)
{
    BT_ENG_ERROR_E err = BT_ENG_NONE_ERROR;
    char result_buf[CMD_RESULT_BUFFER_LEN];
    int ret_val = 0;

    memset(result_buf, 0, sizeof(result_buf));
    bteng_client_fd = client_fd;

    if (strncmp(*argv, "dut_mode_configure", 18) == 0 && argc > 1)
    {
        argv++;
        argc--;

        if(((*argv)[0] == '1') || ((*argv)[0] == '0'))
        {
            BTENG_LOGD("rcv dut_mode_configure cmd.");
            err = bteng_dut_mode_set(argv);
        }
        else
        {
            BTENG_LOGE("dut_mode_configure parameter invalid.");
            fprintf(stderr, "dut_mode_configure parameter invalid\n");
            err = BT_ENG_PARA_INVALID;
        }
    }
    else if(strncmp(*argv, "eut_status", 10) == 0)
    {
        sprintf(result_buf,"return value: %d",  bt_enabled);

        bteng_send_back_cmd_result(client_fd, result_buf, 1);

        err = BT_ENG_NONE_ERROR;
    }
    else
    {
        BTENG_LOGE("rcv cmd is invalid.");
        fprintf(stderr, "rcv cmd is invalid.\n");
        err = BT_ENG_CMD_INVALID;
    }

    if (err == BT_ENG_NONE_ERROR) 
    {
        ret_val = 0;
        //cmd result will send after bluetooth on or off
    } 
    else
    {
        ret_val = 1;

        switch(err)
            {
                case BT_ENG_CMD_INVALID:
                    sprintf(result_buf, "invalid cmd");                
                    break;                        
                case BT_ENG_PARA_INVALID:
                    sprintf(result_buf, "invalid parameter");
                    break; 
                case BT_ENG_HAL_LOAD_ERROR:
                    sprintf(result_buf, "hal load error");
                    break; 
                case BT_ENG_INIT_ERROR:
                    sprintf(result_buf, "bt init error");
                    break; 
                case BT_ENG_ENABLE_ERROR:
                    sprintf(result_buf, "bt enable error");
                    break; 
                case BT_ENG_DISABLE_ERROR:
                    sprintf(result_buf, "bt disable error");
                    break; 
                case BT_ENG_SET_DUT_MODE_ERROR:
                    sprintf(result_buf, "set dut mode error");
                    break; 
                case BT_ENG_CLEANUP_ERROR:
                    sprintf(result_buf, "bt clean up error");
                    break;
                default:
                    break;
            }
        
        bteng_send_back_cmd_result(client_fd, result_buf, 0);
    }

    return ret_val;
}
