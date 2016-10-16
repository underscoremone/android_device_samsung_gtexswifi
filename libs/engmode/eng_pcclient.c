#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/resource.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <semaphore.h>
#include "cutils/sockets.h"
#include "cutils/properties.h"
#include <private/android_filesystem_config.h>
#include "eng_pcclient.h"
#include "eng_diag.h"
#include "engopt.h"
#include "eng_at.h"
#include "eng_sqlite.h"
#include "eng_uevent.h"

#define VLOG_PRI  -20
#define USB_CONFIG_VSER  "vser"
#define SYS_CLASS_ANDUSB_ENABLE "/sys/class/android_usb/android0/enable"

sem_t g_armlog_sem;
extern void	disconnect_vbus_charger(void);

static struct eng_param cmdparam = {
    .califlag = 0,
    .engtest = 0,
    .cp_type = ENG_RUN_TYPE_TD,
    .connect_type = CONNECT_USB,
    .nativeflag = 0,
    .normal_cali = 0
};

static void set_vlog_priority(void)
{
    int inc = VLOG_PRI;
    int res = 0;

    errno = 0;
    res = nice(inc);
    if (res < 0){
        printf("cannot set vlog priority, res:%d ,%s\n", res,
                strerror(errno));
        return;
    }
    int pri = getpriority(PRIO_PROCESS, getpid());
    printf("now vlog priority is %d\n", pri);
    return;
}

/* Parse one parameter which is before a special char for string.
 * buf:[IN], string data to be parsed.
 * gap:[IN], char, get value before this charater.
 * value:[OUT] parameter value
 * return length of parameter
 */
static int cali_parse_one_para(char * buf, char gap, int* value)
{
    int len = 0;
    char *ch = NULL;
    char str[10] = {0};

    if(buf != NULL && value  != NULL){
        ch = strchr(buf, gap);
        if(ch != NULL){
            len = ch - buf ;
            strncpy(str, buf, len);
            *value = atoi(str);
        }
    }
    return len;
}

void eng_check_factorymode(int normal_cali)
{
    int ret;
    int fd;
    int status = eng_sql_string2int_get(ENG_TESTMODE);
    char status_buf[8];
    char modem_diag_value[PROPERTY_VALUE_MAX];
    char usb_config_value[PROPERTY_VALUE_MAX];
    char gser_config[] = {",gser"};
    char build_type[PROPERTY_VALUE_MAX];
    int usb_diag_set = 0;
    int i;
    int property_get_count=0;
#ifdef USE_BOOT_AT_DIAG
    fd=open(ENG_FACOTRYMODE_FILE, O_RDWR|O_CREAT|O_TRUNC, 0660);

    if(fd >= 0){
        ENG_LOG("%s: status=%x\n",__func__, status);
        chmod(ENG_FACOTRYMODE_FILE, 0660);
        property_get("ro.build.type",build_type,"not_find");
        ENG_LOG("%s: build_type: %s", __FUNCTION__, build_type);
        property_get("persist.sys.modem.diag",modem_diag_value,"not_find");
        ENG_LOG("%s: modem_diag_value: %s\n", __FUNCTION__, modem_diag_value);
        if((status==1)||(status == ENG_SQLSTR2INT_ERR)) {
            sprintf(status_buf, "%s", "1");
            if(strcmp(modem_diag_value,",none") == 0) {
                usb_diag_set = 1;
            }
        }else {
            sprintf(status_buf, "%s", "0");
            if(strcmp(modem_diag_value,",none") != 0) {
                usb_diag_set = 1;
                memcpy(gser_config, ",none", 5);
            }
        }

        ENG_LOG("%s: normal_cali: %d\n", __FUNCTION__, normal_cali);
        if((usb_diag_set && (0 == strcmp(build_type, "userdebug"))) || normal_cali){
            do{
                property_get("sys.usb.config",usb_config_value,"not_find");
                if(strcmp(usb_config_value,"not_find") == 0){
                    usleep(200*1000);
                    ENG_LOG("%s: can not find sys.usb.config\n",__FUNCTION__);
                    continue;
                }else{
                    if(normal_cali){
                        property_set("sys.usb.config", USB_CONFIG_VSER);
                        property_get("sys.usb.config",usb_config_value,"not_find");
                        ENG_LOG("%s: get sys.usb.config: %s\n",__FUNCTION__,usb_config_value);
                        property_set("persist.sys.usb.config", USB_CONFIG_VSER);
                        do{
                            usleep(100*1000);
                            property_get("persist.sys.usb.config",usb_config_value,"not_find");
                            property_get_count++;
                        }while(0 != strcmp(usb_config_value, USB_CONFIG_VSER));
                        ENG_LOG("%s: get persist.sys.usb.config: %s,%d\n",__FUNCTION__,usb_config_value,property_get_count);
                        ret=remove("/data/property/persist.sys.usb.config");
                        ENG_LOG("%s: remove persist.sys.usb.config: ret=%d\n",__FUNCTION__,ret);
                    }else{
                        property_set("persist.sys.modem.diag", gser_config);
                        ENG_LOG("%s: set usb property mass_storage,adb,vser,gser\n",__FUNCTION__);
                    }
                    break;
                }
            }while(1);
        }
        ret = write(fd, status_buf, strlen(status_buf)+1);
        ENG_LOG("%s: write %d bytes to %s",__FUNCTION__, ret, ENG_FACOTRYMODE_FILE);

        close(fd);
    }else{
        ENG_LOG("%s: fd: %d, status: %d\n", __FUNCTION__, fd, status);
    }
#endif

    fd=open(ENG_FACOTRYSYNC_FILE, O_RDWR|O_CREAT|O_TRUNC, 0660);
    if(fd >= 0)
        close(fd);
}

static int eng_parse_cmdline(struct eng_param * cmdvalue)
{
    int fd = 0;
    char cmdline[ENG_CMDLINE_LEN] = {0};
    char *str = NULL;
    int mode =  0;
    int freq = 0;
    int device = 0;
    int len = -1;

    if(cmdvalue == NULL)
        return -1;

    fd = open("/proc/cmdline", O_RDONLY);
    if (fd >= 0) {
        if (read(fd, cmdline, sizeof(cmdline)) > 0){
            ALOGD("eng_pcclient: cmdline %s\n",cmdline);
            /*calibration*/
            str = strstr(cmdline, "calibration");
            if ( str  != NULL){
                cmdvalue->califlag = 1;
                disconnect_vbus_charger();
                /*calibration= mode,freq, device. Example: calibration=8,10096,146*/
                str = strchr(str, '=');
                if(str != NULL){
                    str++;
                    /*get calibration mode*/
                    len = cali_parse_one_para(str, ',', &mode);
                    if(len > 0){
                        str = str + len +1;
                        /*get calibration freq*/
                        len = cali_parse_one_para(str, ',', &freq);
                        /*get calibration device*/
                        str = str + len +1;
                        len = cali_parse_one_para(str, ' ', &device);
                    }
                    switch(mode){
                        case 1:
                        case 5:
                        case 7:
                        case 8:
                            cmdvalue->cp_type = ENG_RUN_TYPE_TD;
                            break;
                        case 11:
                        case 12:
                        case 14:
                        case 15:
                            cmdvalue->cp_type = ENG_RUN_TYPE_WCDMA;
                            break;
                        default:
                            break;
                    }

                    /*Device[4:6] : device that AP uses;  0: UART 1:USB  2:SPIPE*/
                    cmdvalue->connect_type = (device >> 4) & 0x3;

                    if(device >>7)
                        cmdvalue->nativeflag = 1;
                    else
                        cmdvalue->nativeflag = 0;

                    ALOGD("eng_pcclient: cp_type=%d, connent_type(AP) =%d, is_native=%d\n",
                            cmdvalue->cp_type, cmdvalue->connect_type, cmdvalue->nativeflag );
                }
            }else{
                /*if not in calibration mode, use default */
                cmdvalue->cp_type = ENG_RUN_TYPE_TD;
                cmdvalue->connect_type = CONNECT_USB;
            }
	      str = strstr(cmdline, "androidboot.mode");
            ENG_LOG("%s: str: %s", __FUNCTION__, str);
            if(str != NULL){
                str = strchr(str, '=');
                ENG_LOG("%s: str: %s", __FUNCTION__, str);
                if(str != NULL){
                    str ++;
                    ENG_LOG("%s: str: %s", __FUNCTION__, str);
                    if(0 == strncmp(str, "engtest", 7)){
                        str = strstr(cmdline, "autotest");
                        ENG_LOG("%s: str: %s", __FUNCTION__, str);
                        if(str != NULL){
                            str = strchr(str, '=');
                            ENG_LOG("%s: str: %s", __FUNCTION__, str);
                            if(str != NULL){
                                str ++;
                                ENG_LOG("%s: str: %s", __FUNCTION__, str);
                                cali_parse_one_para(str, ' ', &(cmdvalue->normal_cali));
                                ENG_LOG("%s: cmdvalue->normal_cali: %d\n", __FUNCTION__, cmdvalue->normal_cali);
                                if(cmdvalue->normal_cali==1){//disable vbus charger in autotest mode
                                    disconnect_vbus_charger();
                                }
                            }
                        }
                    }
                }
            }
            /*engtest*/
            if(strstr(cmdline,"engtest") != NULL)
                cmdvalue->engtest = 1;
        }
        close(fd);
    }

    ENG_LOG("eng_pcclient califlag=%d \n", cmdparam.califlag);

    return 0;
}

static void eng_usb_enable(void)
{
    int fd  = -1;
    int ret = 0;

    fd = open(SYS_CLASS_ANDUSB_ENABLE, O_WRONLY);
    if(fd >= 0){
        ret = write(fd, "1", 1);
        ENG_LOG("%s: Write sys class androidusb enable file: %d\n", __FUNCTION__, ret);
        close(fd);
    }else{
        ENG_LOG("%s: Open sys class androidusb enable file failed!\n", __FUNCTION__);
    }
}

static int eng_get_usb_int(int argc, char** argv, char* at_dev, char* diag_dev, char* log_dev)
{
    int opt  = -1;
    int type = -1;

    do {
        opt = getopt(argc, argv, "t:a:d:l:");
        if(-1 == opt)
            continue;

        switch(opt){
            case 't':
                type = atoi(optarg);
                break;
            case 'a':
                strcpy(at_dev, optarg);
                break;
            case 'd':
                strcpy(diag_dev, optarg);
                break;
            case 'l':
                strcpy(log_dev, optarg);
                break;
            default:
                break;
        }
    }while(-1 != opt);

    return type;
}

static void eng_get_modem_int(int type, char* at_chan, char* diag_chan, char* log_chan)
{
    switch(type){
        case ENG_RUN_TYPE_WCDMA:
            property_get("ro.modem.w.diag", diag_chan, "not_find");
            property_get("ro.modem.w.tty", at_chan, "not_find");
            break;
        case ENG_RUN_TYPE_TD:
            property_get("ro.modem.t.diag", diag_chan, "not_find");
            property_get("ro.modem.t.tty", at_chan, "not_find");
            break;
        case ENG_RUN_TYPE_LTE:
            property_get("ro.modem.lte.diag", diag_chan, "not_find");
            property_get("ro.modem.lte.tty", at_chan, "not_find");
            break;
        case ENG_RUN_TYPE_WCN:
            property_get("ro.modem.wcn.diag", diag_chan, "not_find");
            break;
        default:
            ENG_LOG("%s: Not find corresponding modem interface !\n", __FUNCTION__);
            return;
    }

    if(ENG_RUN_TYPE_WCN != type && 0 != strcmp(at_chan, "not_find")){
        strcat(at_chan, "31"); // channel31 is reserved for eng at
    }
}

int main (int argc, char** argv)
{
    char cmdline[ENG_CMDLINE_LEN];
    int run_type = ENG_RUN_TYPE_TD;
    eng_thread_t t0,t1,t2,t3;
    eng_dev_info_t dev_info = {{"/dev/ttyGS0", "/dev/vser", 0, 1}, {0, 0, 0}};

    run_type = eng_get_usb_int(argc, argv, dev_info.host_int.dev_at,
            dev_info.host_int.dev_diag, dev_info.host_int.dev_log);
    ENG_LOG("engpcclient runtype:%d, atPath:%s, diagPath:%s, logPath:%s, type: %d\n", run_type,
            dev_info.host_int.dev_at, dev_info.host_int.dev_diag, dev_info.host_int.dev_log, dev_info.host_int.dev_type);

    // Get the status of calibration mode & device type.
    eng_parse_cmdline(&cmdparam);
    // Correct diag path and run type by cmdline.
    if(1 == cmdparam.califlag){
        run_type = cmdparam.cp_type;
        dev_info.host_int.cali_flag = cmdparam.califlag;
        dev_info.host_int.dev_type = cmdparam.connect_type;
        if(CONNECT_UART == cmdparam.connect_type){
            strcpy(dev_info.host_int.dev_diag, "/dev/ttyS1");
            dev_info.host_int.dev_type = CONNECT_UART;
        }
    }

    eng_get_modem_int(run_type, dev_info.modem_int.at_chan, dev_info.modem_int.diag_chan,
            dev_info.modem_int.log_chan);
    ENG_LOG("eng_pcclient: modem at chan: %s, modem diag chan: %s,modem log chan: %s\n",
            dev_info.modem_int.at_chan, dev_info.modem_int.diag_chan,dev_info.modem_int.log_chan);

    // Create the sqlite database for factory mode.
    // FIX ME:temporarily check by ENG_RUN_TYPE_WCN/LTE
    if(ENG_RUN_TYPE_WCN != run_type && ENG_RUN_TYPE_LTE != run_type){
        eng_sqlite_create();
        if(cmdparam.califlag != 1){
        if(cmdparam.normal_cali){
          //Change gser port
		  memcpy(dev_info.host_int.dev_diag, "/dev/vser", sizeof("/dev/vser"));
        }
            // Check factory mode and switch device mode.
            eng_check_factorymode(cmdparam.normal_cali);
            if(cmdparam.normal_cali)
                eng_autotestStart();
        }else{
            // Enable usb enum
            eng_usb_enable();
            // Initialize file for ADC
            initialize_ctrl_file();
        }
    }

    set_vlog_priority();

    // Semaphore initialization
    sem_init(&g_armlog_sem, 0, 0);

    if(0 != eng_thread_create(&t0, eng_uevt_thread, NULL)){
        ENG_LOG("uevent thread start error");
    }

    // Create vlog thread for reading diag data from modem and send it to PC.
    if (0 != eng_thread_create( &t1, eng_vlog_thread, &dev_info)){
        ENG_LOG("vlog thread start error");
    }

    // Create vdiag thread for reading diag data from PC, some data will be
    // processed by ENG/AP, and some will be pass to modem transparently.
    if (0 != eng_thread_create( &t2, eng_vdiag_wthread, &dev_info)){
        ENG_LOG("vdiag wthread start error");
    }

    if (0 != eng_thread_create( &t3, eng_vdiag_rthread, &dev_info)){
        ENG_LOG("vdiag rthread start error");
    }

    if(cmdparam.califlag != 1 || cmdparam.nativeflag != 1){
        eng_at_pcmodem(&dev_info);
    }

    while(1){
        sleep(10000);
    }

    return 0;
}
