#include <stdio.h>
#include <stdlib.h>
#include "eut_opt.h"
#include <fcntl.h>

#include <stdlib.h>
#include <stdio.h>

//#include "engopt.h"


#ifndef NUM_ELEMS(x)
#define NUM_ELEMS(x) (sizeof(x)/sizeof(x[0]))
#endif
#define COUNTER_BUF_SIZE 2048

#define WIFI_RATE_REQ_RET         "+SPWIFITEST:RATE="
#define EUT_WIFI_RSSI_REQ_RET     "+SPWIFITEST:RSSI="
#define WIFI_TXGAININDEX_REQ_RET  "+SPWIFITEST:TXGAININDEX="

#define EUT_WIFI_START "START"
#define EUT_WIFI_TXTEST "TXTEST"
#define EUT_WIFI_RXTEST "RXTEST"
#define EUT_WIFI_CWTEST "CWTEST"
#define EUT_WIFI_STOP "STOP"
#define EUT_WIFI_TXSTOP  "TXSTOP"
#define EUT_WIFI_RXPKTCNT "RXPKTCNT"

#ifndef WIFI_DRIVER_MODULE_PATH
#define WIFI_DRIVER_MODULE_PATH     "/system/lib/modules/bcmdhd.ko"
#endif
#ifndef WIFI_DRIVER_FW_PATH_PARAM
#define WIFI_DRIVER_FW_PATH_PARAM	"/sys/module/bcmdhd/parameters/firmware_path"
#endif
#ifndef WIFI_DRIVER_FW_PATH_MFG
//#define WIFI_DRIVER_FW_PATH_MFG "system/vendor/firmware/fw_bcmdhd_mfg.bin"
#define WIFI_DRIVER_FW_PATH_MFG "/system/vendor/firmware/fw_bcmdhd_mfg.bin"
#endif

#ifdef STR(str)
#undef STR
#endif
#define STR(str) str

static int wifieut_state=0;
static int wifiwork_mode;
static float ratio_p;
static int channel_p;
static int tx_power_factor=17;//17dbm=50mW
static long tx_power_factor_p=32767;//same as tx_power_factor=17
static int tx_state;

static int tx_pkt_ipg=800;
static int tx_pkt_len=1500;
static int tx_pkt_count=0;
static int PM_stat=0;

static int rx_state;
static int rx_ratio;
static int rx_channel;
static int rx_power;
static long rx_packcount;
static long rxmfrmocast_priv;
static long rxmfrmocast_next;
static long rx_errphypack=0;//rx_errphypack no use any more
static long rxbyte_priv;
static long rxbyte_next;
static long rx_errpack;
static long rxerror_priv;
static long rxerror_next;

static char cmd_set_ratio[20];
static char counter_respon[COUNTER_BUF_SIZE];

struct wifi_ratio2mode{
    int mode;
    float ratios[10];
};

static struct wifi_ratio2mode mode_list[]={
    {WIFI_MODE_11B,{1, 2, 5.5, 11}},
    {WIFI_MODE_11G,{6,9,12,18,24,36,48,54}},
    {WIFI_MODE_11N,{6.5,13,19.5,26,39,52,58.5,65}}
};

static int get_reflect_factor_pc(int factor,int mode);
static int wifi_clr_rxpackcount_pc(char * result);
static long parse_packcount_pc(char *filename, char *keywordstart, char *keywordend, int *error);
static int start_wifieut_pc(char *result);
static int end_wifieut_pc(char *result);
static int start_wifi_tx_pc(char *result);
static int end_wifi_tx_pc(char *result);
static int start_wifi_rx_pc(char *result);
static int end_wifi_rx_pc(char *result);
static int reflect_factor_pc(int start,int end,int factor);

typedef struct
{
	float rate;
	char *name;
} WIFI_RATE;

static WIFI_RATE g_wifi_rate_table[] = 
{
	{1, "DSSS-1"},
	{2, "DSSS-2"},
	{5.5, "CCK-5.6"},
	{11, "CCK-11"},
	{6, "OFDM-6"},
	{9, "OFDM-9"},
	{12, "OFDM-12"},
	{18, "OFDM-18"},
	{24, "OFDM-24"},
	{36, "OFDM-36"},
	{48, "OFDM-48"},
	{54, "OFDM-54"},
	{6.5, "MCS-0"},
	{13, "MCS-1"},
	{19.5, "MCS-2"},
	{26, "MCS-3"},
	{39, "MCS-4"},
	{52, "MCS-5"},
	{58.5, "MCS-6"},
	{65, "MCS-7"},
};

static float mattch_rate_table_str_pc(char *string)
{
	int i;
	float ret = 0;
	for( i = 0; i < (int)NUM_ELEMS(g_wifi_rate_table); i++)
	{
		if( NULL != strstr(string, g_wifi_rate_table[i].name) )
		{
			ret = g_wifi_rate_table[i].rate;
			break;
		}
	}
	return ret;
}

static char *mattch_rate_table_index_pc(float rate)
{
	int i;
	char *p = NULL;
	for( i = 0; i < (int)NUM_ELEMS(g_wifi_rate_table); i++)
	{
		if((int)(rate*1000) == (int)(g_wifi_rate_table[i].rate*1000))
		{
			p = g_wifi_rate_table[i].name;
			break;
		}
	}
	return p;
}

static int wifieut_pc(int command_code,char *rsp)
{
    ALOGI("wifieut");
    if(command_code == 1)
        start_wifieut_pc(rsp);
    else if(command_code == 0)
        end_wifieut_pc(rsp);
    return 0;
}
static int wifieut_req_pc(char *rsp)
{
    sprintf(rsp,"%s%d",EUT_WIFI_REQ,wifieut_state);
    return 0;
}

static int wifi_tx_pktcount_pc(int count)
{
	if(count>0)
	{
		tx_pkt_count = count;
	}
	return 0;
}

static int wifi_tx_pktlen_pc(int len)
{
	if(len>0)
	{
		tx_pkt_len = len;
	}
	return 0;
}

static int wifi_tx_ipg_pc(int ipg)
{
	if(ipg>0)
	{
		tx_pkt_ipg = ipg;
	}
	return 0;
}

static int wifi_tx_pc(int command_code,char *rsp)
{
    ALOGI("wifi_tx");
    if(command_code == 1)
        start_wifi_tx_pc(rsp);
    else if(command_code == 0)
        end_wifi_tx_pc(rsp);
    return 0;
}

static int wifi_rx_pc(int command_code,char *rsp)
{
    ALOGI("wifi_rx");
    if(command_code == 1){
        start_wifi_rx_pc(rsp);
        if (rx_state == 1)//(!strcmp(rsp,EUT_WIFI_OK)) 
            wifi_clr_rxpackcount_pc(rsp);
    }
    else if(command_code == 0)
        end_wifi_rx_pc(rsp);
    return 0;
}


/*
PM    set driver power management mode:
        0: CAM (constantly awake)
        1: PS  (power-save)
        2: FAST PS mode
*/
static int wifi_pm_pc(int command_code,char *result)
{
    ALOGI("wifi_pm");
    int error =-1;
    if(command_code == 0) {
        error = system("wl PM 0");
            ALOGI("wl PM 0");
            if(error == -1 || error == 127){
                    ALOGE("=== wifi_pm failed on cmd : wl PM 0 ===\n");
                    sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
		      return -1;
            }
		PM_stat=0;
    }else if(command_code == 1){
             error = system("wl PM 1");
            ALOGI("wl PM 1");
            if(error == -1 || error == 127){
                    ALOGE("=== wifi_pm failed on cmd : wl PM 1 ===\n");
                    sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
			return -1;
            }
		PM_stat=1;
    }else if(command_code == 2){
            error = system("wl PM 2");
            ALOGI("wl PM 2");
            if(error == -1 || error == 127){
                    ALOGE("=== wifi_pm failed on cmd : wl PM 2 ===\n");
                    sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
		      return -1;
            }
    }else {
        ALOGE("=== wifi_pm failed ,cmd = %d is not supported!\n",command_code);
        sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
        return -1;
    }
    ALOGI("=== WIFIEUT wifi_pm  succeed! ===\n");
    strcpy(result,EUT_WIFI_OK);
    return 0;
}

static int wifi_pm_sta_pc()
{
	return PM_stat;
}

static int wifi_cw_pc(char *result)
{
    ALOGI("wifi_rx");
    int error = system("wl mpc 0");
        ALOGI("wl mpc 0");
        if(error == -1 || error == 127){
            ALOGE("=== wifi_cw test failed on cmd : wl mpc 0 ===\n");
            sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
        }else{
            error = system("wl up");
            ALOGI("wl up");
            if(error == -1 || error ==127){
                ALOGE("=== wifi_cw test failed on cmd : wl up ===\n");
                sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
            }else{
                error = system("wl band b");
                ALOGI("wl band b");
                if(error == -1 || error == 127){
                    ALOGE("=== wifi_cw test failed on cmd : wl band b ===\n");
                    sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
                }else{
                    error = system("wl phy_tx_tone 400");
                    ALOGI("wl phy_tx_tone 400");
                    if(error == -1 || error == 127){
                        ALOGE("=== wifi_cw test failed on cmd : wl phy_tx_tone 400===\n");
                        sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
                    }else{
			   error = system("wl phy_txpwrindex 20");
			    if(error == -1 || error == 127){
                         ALOGE("=== wifi_cw test failed on cmd : wl phy_txpwrindex 20===\n");
                         sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
                        }else{
				            ALOGI("=== WIFIEUT CW test succeed! ===\n");
                            strcpy(result,EUT_WIFI_OK);
                            tx_state=1;
                        }
                    }
                }
            }
        }
    return 0;
}

static int start_wifieut_pc(char *result)
{
    ALOGI("start_wifieut----------------------");
    int error = system("insmod "STR(WIFI_DRIVER_MODULE_PATH)" firmware_path="STR(WIFI_DRIVER_FW_PATH_MFG)" nvram_path=/system/etc/wifi/bcmdhd.cal");
    if(error == -1 || error ==127){
        ALOGE("=== start_wifieut test failed on cmd : insmod "STR(WIFI_DRIVER_MODULE_PATH)" firmware_path="STR(WIFI_DRIVER_FW_PATH_MFG)" nvram_path=system/etc/wifi/bcmdhd.cal""====\n");
        sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
    }else{
        error = system("ifconfig wlan0 down");
        if(error == -1 || error ==127){
            ALOGE("=== start_wifieut test failed on cmd : ifconfig wlan0 down ===\n");
            sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
        }else{
            error = system("echo -n "STR(WIFI_DRIVER_FW_PATH_MFG)" > "STR(WIFI_DRIVER_FW_PATH_PARAM));
            if(error == -1 || error ==127){
                ALOGE("=== start_wifieut test failed on cmd : echo -n "STR(WIFI_DRIVER_FW_PATH_MFG)" > "STR(WIFI_DRIVER_FW_PATH_PARAM)"===\n");
                sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
            }else{
                error = system("ifconfig wlan0 up");
                if(error == -1 || error ==127){
                    ALOGE("=== start_wifieut test failed on cmd : ifconfig wlan0 down ===\n");
                    sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
                }else{
                    ALOGI("=== WIFIEUT test succeed! ===\n");
                    wifieut_state=1;
                    strcpy(result,EUT_WIFI_OK);
                }
            }
        }
    }
    return 0;
}


static int end_wifieut_pc(char *result)
{
    int error = system("ifconfig wlan0 down");
    if(error == -1 || error ==127){
        ALOGE("=== start_wifieut test failed on cmd : ifconfig wlan0 down ===\n");
        sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
    }else{
        wifieut_state=0;
        strcpy(result,EUT_WIFI_OK);
    }
    return 0;
}

static int start_wifi_tx_pc(char *result)
{
    ALOGI("start_wifi_tx-----------------");
    char cmd_set_channel[20]={0};
    char cmd_set_factor[25]={0};
	char cmd_tx[64]={0};

    sprintf(cmd_set_channel,"wl channel %d",channel_p);
    tx_power_factor = get_reflect_factor_pc(tx_power_factor_p,wifiwork_mode);
    sprintf(cmd_set_factor,"wl txpwr1 -o -d %d",tx_power_factor);

    int error =system("wl down");
    ALOGI("wl down");
    if(error == -1 || error == 127){
        ALOGE("=== start_wifi_tx test failed on cmd : wl down ===\n");
        sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
    }else{
        error = system("wl mpc 0");
        ALOGI("wl mpc 0");
        if(error == -1 || error == 127){
            ALOGE("=== start_wifi_tx test failed on cmd : wl mpc 0 ===\n");
            sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
        }else{
            error = system("wl up");
            ALOGI("wl up");
            if(error == -1 || error ==127){
                ALOGE("=== start_wifi_tx test failed on cmd : wl up ===\n");
                strcpy(result,"WIFI_TX,fail");
            }else{
                error = system("wl PM 0");
                ALOGI("wl PM 0");
                if(error == -1 || error == 127){
                    ALOGE("=== start_wifi_tx test failed on cmd : wl PM 0 ===\n");
                    sprintf(result,"%s=%d",EUT_WIFI_ERROR,error);
                }else{
                    error = system("wl scansuppress 1");
                    ALOGI("wl scansuppress 1");
                    if(error == -1 || error == 127){
                        ALOGE("=== start_wifi_tx test failed on cmd : wl scansuppress 1===\n");
                        sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
                    }else{
                        error = system("wl country ALL");
                        ALOGI("wl country ALL");
                        if(error == -1 || error == 127){
                            ALOGE("=== start_wifi_tx test failed on cmd : wl country ALL 1===\n");
                            sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
                        }else{
                            error = system(cmd_set_channel);
                            ALOGI("wl channel %d",channel_p);
                            if(error == -1 || error == 127){
                                ALOGE("=== start_wifi_tx test failed on cmd : wl country ALL 1===\n");
                                sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
                            }else{
                                error = system(cmd_set_ratio);
                                ALOGI("%s",cmd_set_ratio);
                                if(error == -1 || error == 127){
                                    ALOGE("=== start_wifi_tx test failed on cmd : wl rate===\n");
                                    sprintf(result,"%s=%d",EUT_WIFI_ERROR,error);
                                }else{
                                    error = system(cmd_set_factor);
                                    ALOGI("wl txpwr1 -o -d %d",tx_power_factor);
                                    if(error == -1 || error == 127){
                                        ALOGE("=== start_wifi_tx test failed on cmd : wl txpwr1 -o -d===\n");
                                        sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
                                    }else{
					sprintf(cmd_tx, "wl pkteng_start 00:11:22:33:44:55 tx %d %d %d" ,tx_pkt_ipg ,tx_pkt_len, tx_pkt_count);
                                        error=system(cmd_tx);
                                        ALOGI("%s",cmd_tx);
                                        if(error == -1 || error == 127){
                                            ALOGE("=== start_wifi_tx test failed on cmd : wl pkteng_start 00:11:22:33:44:55 tx 100 1500 0===\n");
                                            sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
                                        }else{
                                            error = system("wl phy_forcecal 1");
                                            ALOGI("wl phy_forcecal 1");
                                            if(error == -1 || error == 127){
                                                ALOGE("=== start_wifi_tx test failed on cmd : wl phy_forcecal 1===\n");
                                                sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
                                            }else{
                                                ALOGI("=== WIFIEUT test succeed! ===\n");
                                                strcpy(result,EUT_WIFI_OK);
                                                tx_state=1;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return 0;
}

static int end_wifi_tx_pc(char *result)
{
    int error = system("wl down");
    ALOGI("end_wifi_tx");
    if(error == -1 || error == 127){
        ALOGE("=== end_wifi_tx test failed on cmd : wl down ===\n");
        sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
    }else{
        ALOGI("=== end_wifi_tx test succeed! ===\n");
        strcpy(result,EUT_WIFI_OK);
        tx_state=0;
    }
    return 0;
}

static int start_wifi_rx_pc(char *result)
{
    ALOGI("start_wifi_rx-----------------");
    char cmd_set_channel[20]={0};
    sprintf(cmd_set_channel,"wl channel %d",channel_p);

    int error =system("wl down");
    ALOGI("wl down");
    if(error == -1 || error == 127){
        ALOGE("=== start_wifi_rx test failed on cmd : wl down ===\n");
        sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
    }else{
        error = system("wl mpc 0");
        ALOGI("wl mpc 0");
        if(error == -1 || error == 127){
            ALOGE("=== start_wifi_rx test failed on cmd : wl mpc 0 ===\n");
            sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
        }else{
            error = system("wl up");
            ALOGI("wl up");
            if(error == -1 || error ==127){
                ALOGE("=== start_wifi_rx test failed on cmd : wl up ===\n");
                sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
            }else{
                error = system("wl PM 0");
                ALOGI("wl PM 0");
                if(error == -1 || error == 127){
                    ALOGE("=== start_wifi_rx test failed on cmd : wl PM 0 ===\n");
                    sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
                }else{
                    error = system("wl scansuppress 1");
                    ALOGI("wl scansuppress 1");
                    if(error == -1 || error == 127){
                        ALOGE("=== start_wifi_rx test failed on cmd : wl scansuppress 1===\n");
                        sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
                    }else{
                        error = system("wl country ALL");
                        ALOGI("wl country ALL");
                        if(error == -1 || error == 127){
                            ALOGE("=== start_wifi_tx test failed on cmd : wl country ALL 1===\n");
                            sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
                        }else{
                            error = system(cmd_set_channel);
                            ALOGI("wl channel %d",channel_p);
                            if(error == -1 || error == 127){
                                ALOGE("=== start_wifi_tx test failed on cmd : wl channel =\n");
                                sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
                            }else{
                                error = system(cmd_set_ratio);
                                ALOGI("wl rate %s %f",cmd_set_ratio,ratio_p);
                                if(error == -1 || error == 127){
                                    ALOGE("=== start_wifi_rx test failed on cmd : wl rate ===\n");
                                    sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
                                }else{
                                    error = system("wl pkteng_start 00:11:22:33:44:55 rx");
                                    if(error == -1 || error == 127){
                                        ALOGE("=== start_wifi_rx test failed on cmd : wl pkteng_start 00:11:22:33:44:55 rx ===\n");
                                        sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
                                    }else{
                                        rx_state=1;
                                        strcpy(result,EUT_WIFI_OK);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        return 0;
    }

    return 0;
}
static int end_wifi_rx_pc(char *result)
{
    int error = system("wl down");
    ALOGI("end_wifi_rx");
    if(error == -1 || error == 127){
        ALOGE("=== end_wifi_rx test failed on cmd : wl down ===\n");
        sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
    }else{
        ALOGI("=== end_wifi_rx  succeed! ===\n");
        strcpy(result,EUT_WIFI_OK);
        rx_state=0;
    }
    return 0;
}
static int set_wifi_ratio_pc(float ratio,char *result)
{
    ratio_p = ratio;
    ALOGI("=== set_wifi_ratio----> %f === %f",ratio_p,ratio);
    int i,m,j,n;
    int break_tag = 0;
    n = (int)NUM_ELEMS(mode_list);
    for(i=0;i<n;i++){
        for(j = 0;j<(int)NUM_ELEMS(mode_list[i].ratios);j++){
            if(ratio == mode_list[i].ratios[j])
            {
                wifiwork_mode=mode_list[i].mode;
                m = j;
                ALOGI("find the ratio %f  wifiwork_mode=%d",ratio,wifiwork_mode);
                break_tag=1;
                break;
            }
        }
        if(break_tag)
            break;
        if(i==n-1){
            sprintf(result,"%s%d",EUT_WIFI_ERROR,EUT_WIFIERR_RATIO);
            ratio_p=0;

            goto fun_out;
        }
    }
    switch(wifiwork_mode){
        case WIFI_MODE_11B:
        case WIFI_MODE_11G:
            sprintf(cmd_set_ratio,"wl rate %f",ratio_p);
            ALOGI("set ratio cmd is %s",cmd_set_ratio);
            break;
        case WIFI_MODE_11N:
            sprintf(cmd_set_ratio,"wl nrate -m %d",m);
            ALOGI("set ratio cmd is %s",cmd_set_ratio);
            break;
    }
    strcpy(result,EUT_WIFI_OK);
fun_out:
    return 0;
}
static int set_wifi_ch_pc(int ch,char *result)
{
    channel_p = ch;
    ALOGI("=== set_wifi_ch ----> %d ===\n",channel_p);
    strcpy(result,"+SPWIFITEST:OK");
    return 0;
}

// for *#*#83780#*#*
static int  set_wifi_tx_factor_83780_pc(int factor,char *result)
{
    tx_power_factor = factor;
    ALOGI("=== set_wifi_tx_factor----> %d===\n",tx_power_factor);
    if(tx_power_factor){
        strcpy(result,EUT_WIFI_OK);
    }else{
        sprintf(result,"%s%d",EUT_WIFI_ERROR,EUT_WIFIERR_TXFAC_SETRATIOFIRST);
    }

    return 0;
}

static int  set_wifi_tx_factor_pc(long factor,char *result)
{
    if((factor>=1)&&(factor<=32767)){
        tx_power_factor = get_reflect_factor_pc(factor,wifiwork_mode);
        tx_power_factor_p = factor;
        ALOGI("=== set_wifi_tx_factor----> %d===\n",tx_power_factor);
        if(tx_power_factor){
            strcpy(result,EUT_WIFI_OK);
        }else{
            sprintf(result,"%s%d",EUT_WIFI_ERROR,EUT_WIFIERR_TXFAC_SETRATIOFIRST);
        }
    }else{
        sprintf(result,"%s%d",EUT_WIFI_ERROR,EUT_WIFIERR_TXFAC);
    }
    return 0;
}
static int get_reflect_factor_pc(int factor,int mode)
{
    int reflect_factor_p;
    ALOGI("get_reflect_factor factor=%d mode=%d",factor,mode);
    switch(mode){
        case WIFI_MODE_11B:
            reflect_factor_p=reflect_factor_pc(6,16,factor);
            break;
        case WIFI_MODE_11G:
            reflect_factor_p=reflect_factor_pc(6,10,factor);
            break;
        case WIFI_MODE_11N:
            reflect_factor_p=reflect_factor_pc(6,15,factor);
            break;
        default:
            return -1;
    }
    return reflect_factor_p;
}
static int reflect_factor_pc(int start,int end,int factor)
{
    int n = end-start+1;
    return  (factor/32767)*n+start;
}

static int set_wifi_mode_pc(char * mode, char * result)
{
    return 0;
}
static int wifi_tx_req_pc(char *result)
{
    sprintf(result,"%s%d",EUT_WIFI_TX_REQ,tx_state);
    return 0;
}
static int wifi_tx_factor_req_pc(char *result)
{
    sprintf(result,"%s%d",EUT_WIFI_TXFAC_REQ,tx_power_factor_p);
    return 0;
}
static int wifi_rx_req_pc(char *result)
{
    sprintf(result,"%s%d",EUT_WIFI_RX_REQ,rx_state);
    return 0;
}
static int wifi_ratio_req_pc(char *result)
{
    sprintf(result,"%s%f",EUT_WIFI_RATIO_REQ,ratio_p);
    return 0;
}
static int wifi_ch_req_pc(char *result)
{
    sprintf(result,"%s%d",EUT_WIFI_CH_REQ,channel_p);
    return 0;
}

static long wifi_rxpackcount_pc(char * result)
{
    int error = system("wl counters > /data/data/wlancounters.txt");
    ALOGI("wifi_rxpackcount");
    if(error == -1 || error == 127){
        ALOGE("=== wifi_rxpackcount on cmd : wl counter===\n");
        sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
    }else{
        ALOGI("=== wifi_rxpackcount! ===\n");
        strcpy(result,EUT_WIFI_OK);
        rxmfrmocast_next=parse_packcount_pc("/data/data/wlancounters.txt", "pktengrxdmcast", "txmpdu_sgi", &error);
        if (0 != error){
            sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
            return 0;
        }
        rx_packcount = rxmfrmocast_next-rxmfrmocast_priv;
        rxerror_next=parse_packcount_pc(NULL, "rxerror", "txprshort", &error);
        if (0 != error){
            sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
            return 0;
        }
        ALOGI("rxpackcount rxmfrmocast_next=%ld, rxerror_next=%ld",rxmfrmocast_next, rxerror_next);
        //rx_errpack no use too. but we still provide it
        rx_errpack = rxerror_next - rxerror_priv;
        //system("rm /data/data/wlancounters.txt");
        sprintf(result,"%s%ld,%ld,%ld",EUT_WIFI_RXPACKCOUNT_PC_REQ,rx_packcount,rx_errphypack,rx_errpack);
        ALOGI("resutl=%s",result);
    }
    return rx_packcount;
}

static int wifi_clr_rxpackcount_pc(char * result)
{
    rx_packcount = 0;
    int error=system("wl counters > /data/data/wlancounters.txt");
    ALOGI("wl counters");
    if(error == -1 || error == 127){
        ALOGE("=== start_wifi_rx test failed on cmd : wl counters===\n");
        sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
        return error;
    }else{
        rxmfrmocast_priv=parse_packcount_pc("/data/data/wlancounters.txt", "pktengrxdmcast", "txmpdu_sgi", &error);
        if (0 != error){
            sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
            return error;
        }
        rxerror_priv=parse_packcount_pc(NULL, "rxerror", "txprshort", &error);
        if (0 != error){
            sprintf(result,"%s%d",EUT_WIFI_ERROR,error);
            return error;
        }
        ALOGI("rxpackcount rxmfrmocast_priv=%ld, rxerror_priv=%ld",rxmfrmocast_priv, rxerror_priv);
    }
    strcpy(result,EUT_WIFI_OK);
    return 0;
}
static long parse_packcount_pc(char *filename, char *keywordstart, char *keywordend, int *error)
{
    int fd,n,len;
    char *p,*q,*s;
    char packcount[20];
    if (filename != NULL)
    {
        memset(counter_respon,0,COUNTER_BUF_SIZE);
        fd = open(filename, O_RDWR|O_NONBLOCK);
        if(fd < 0){
            ALOGE("=== open file  %s error===\n",filename);
            *error = fd;
            return 0;
        }else{
            n = read(fd,counter_respon,COUNTER_BUF_SIZE);
            close(fd);
        }
    }
    p=strstr(counter_respon, keywordstart);
    if(p != NULL){
        q=strstr(p," ");
        s=strstr(q, keywordend);
//        ALOGE("=== open file parse_packcount entry s= %s\n",s);
        len= s-q-1;
       memcpy(packcount,q+1,len);
        *error = 0;
        return atol(packcount);
    }else{
    ALOGE("=== dirty file  %s no find %s ===\n",filename, keywordstart);
        *error = 1;
        return 0;
    }
}

static int set_wifi_rate_pc(char *string, char *rsp)
{
	float rate = -1;
	if(0 == wifieut_state)
	{
		ALOGE("%s(), wifieut_state:%d", __FUNCTION__, wifieut_state);
		sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
		return -1;
	}
	rate = mattch_rate_table_str_pc(string);
	return set_wifi_ratio_pc(rate, rsp);
}

static int wifi_rate_req_pc(char *rsp)
{
	int ret = -1;
	char *str = NULL;
	ALOGI("%s()...\n", __FUNCTION__);
	if(0 == ratio_p)
	{
		ALOGE("%s(), ratio_p is 0", __FUNCTION__);
		goto err;
	}
	str = mattch_rate_table_index_pc(ratio_p);
	if(NULL == str)
	{
		ALOGE("%s(), don't mattch rate", __FUNCTION__);
		goto err;
	}
		
	sprintf(rsp, "%s%s", WIFI_RATE_REQ_RET, str);
	return 0;
err:
	sprintf(rsp, "%s%s", WIFI_RATE_REQ_RET, "null");
	return -1;
}

static int set_wifi_txgainindex_pc(int index, char *rsp)
{
	if(0 == wifieut_state)
	{
		ALOGE("%s(), wifieut_state:%d", __FUNCTION__, wifieut_state);
		goto err;
	}		
	ALOGI("%s(), index:%d\n",  __FUNCTION__, index);
	return set_wifi_tx_factor_83780_pc(index, rsp);

err:
	sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
	return -1;
}

static int wifi_txgainindex_req_pc(char *rsp)
{
	if(0 == wifieut_state)
	{
		ALOGE("%s(), wifieut_state:%d", __FUNCTION__, wifieut_state);
		goto err;
	}	

	sprintf(rsp,"%s%d",WIFI_TXGAININDEX_REQ_RET,tx_power_factor);
	return 0;
err:
	sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
	return -1;
}

static int wifi_rssi_req_pc(char *rsp)
{
	char buf[100]={0};
	FILE *fp;
	int rssi=-100;

	if( 1 == tx_state )
	{
		ALOGE("wifi_rssi_req(),tx_state:%d", tx_state);
		goto err;
	}

	if ((fp = popen("wl rssi", "r" )) == NULL)
	{
		ALOGE("=== wifi_rssi_req popen() fail ===\n");
		sprintf(rsp,"%s%s",EUT_WIFI_ERROR,"popen");
		return -1;
	}
	while(fgets(buf, sizeof(buf), fp) != NULL)
	{
		if (buf[0]=='-' ||(buf[0]>='0' && buf[0]<='9'))
		{
			rssi = atoi(buf);
			break;
		}
	}
	pclose(fp);

	if(-100 == rssi)
	{
		ALOGE("get_rssi cmd  err");
		goto err;
	}
	
	sprintf(rsp, "%s0x%x", EUT_WIFI_RSSI_REQ_RET, rssi);
err:
	sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
	return -1;
}


struct eng_wifi_eutops wifi_eutops ={   wifieut_pc,
    wifieut_req_pc,
    wifi_tx_pc,
    set_wifi_tx_factor_pc,
    wifi_rx_pc,
    set_wifi_mode_pc,
    set_wifi_ratio_pc,
    set_wifi_ch_pc,
    wifi_tx_req_pc,
    wifi_tx_factor_req_pc,
    wifi_rx_req_pc,
    wifi_ratio_req_pc,
    wifi_ch_req_pc,
    wifi_rxpackcount_pc,
//    wifi_clr_rxpackcount,
    set_wifi_rate_pc,				//int (*set_wifi_rate)(char *, char *);
    wifi_rate_req_pc,				//int (*wifi_rate_req)(char *);
    set_wifi_txgainindex_pc,		//int (*set_wifi_txgainindex)(int , char *);
    wifi_txgainindex_req_pc,		//int (*wifi_txgainindex_req)(char *);
    wifi_rssi_req_pc,				//int (*wifi_rssi_req)(char *);
    wifi_tx_pktlen_pc
};

