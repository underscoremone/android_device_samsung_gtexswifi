#include <stdlib.h>
#include <stdio.h>
#include "eut_opt.h"
#include <fcntl.h>
#include "engopt.h"

#define NUM_ELEMS(x) (sizeof(x)/sizeof(x[0]))
#define wifi_eut_debug

#ifdef 	wifi_eut_debug

#define rsp_debug(rsp_)   do{\
		ENG_LOG("%s(), rsp###:%s\n", __FUNCTION__, rsp_);\
	}while(0);

#define wifi_status_dump()   do{\
		ENG_LOG("%s(), eut_enter:%d, tx_start:%d, rx_start:%d\n", __FUNCTION__, g_wifi_data.eut_enter, g_wifi_data.tx_start, g_wifi_data.rx_start);\
	}while(0);

#else

#define rsp_debug(rsp_) 

#endif


//----------------------------------------------------------------
#define TMP_BUF_SIZE  128

#define STR_RET_STATUS            ("ret: status ")
#define STR_RET_REG_VALUE         ("ret: reg value:")
#define STR_RET_RET               ("ret: ")
#define STR_RET_END               (":end")
#define TMP_FILE                  ("/data/wifi_npi_data.log")
#define WIFI_RATE_REQ_RET         "+SPWIFITEST:RATE="
#define WIFI_CHANNEL_REQ_RET      "+SPWIFITEST:CH="
#define WIFI_TXGAININDEX_REQ_RET  "+SPWIFITEST:TXGAININDEX="
#define EUT_WIFI_RXPKTCNT_REQ_RET "+SPWIFITEST:RXPACKCOUNT="
#define EUT_WIFI_RSSI_REQ_RET     "+SPWIFITEST:RSSI="
//----------------------------------------------------------------

typedef struct
{
	unsigned int rx_end_count;
	unsigned int rx_err_end_count;
	unsigned int fcs_fail_count;
}RX_PKTCNT;

typedef struct
{
	int eut_enter;
	int rate;
	int channel;
	int tx_start;
	int rx_start;
	int txgainindex;
	RX_PKTCNT cnt;
} WIFI_ELEMENT;

typedef struct
{
	int rate;
	char *name;
} WIFI_RATE;

static WIFI_ELEMENT g_wifi_data;
static WIFI_RATE g_wifi_rate_table[] = 
{
	{1, "DSSS-1"},
	{2, "DSSS-2"},
	{5, "CCK-5.6"},
	{11, "CCK-11"},
	{6, "OFDM-6"},
	{9, "OFDM-9"},
	{12, "OFDM-12"},
	{18, "OFDM-18"},
	{24, "OFDM-24"},
	{36, "OFDM-36"},
	{48, "OFDM-48"},
	{54, "OFDM-54"},
	{7, "MCS-0"},
	{13, "MCS-1"},
	{19, "MCS-2"},
	{26, "MCS-3"},
	{39, "MCS-4"},
	{52, "MCS-5"},
	{58, "MCS-6"},
	{65, "MCS-7"},
};

static int mattch_rate_table_str(char *string)
{
	int i;
	int ret = -1;
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

static char *mattch_rate_table_index(int rate)
{
	int i;
	int ret = -1;
	char *p = NULL;
	for( i = 0; i < (int)NUM_ELEMS(g_wifi_rate_table); i++)
	{
		if(rate == g_wifi_rate_table[i].rate)
		{
			p = g_wifi_rate_table[i].name;
			break;
		}
	}
	return p;
}

static int get_iwnpi_ret_status(void )
{
	FILE *fp = NULL;
	char *str1 = NULL;
	char *str2 = NULL;
	int ret = -100;
	int len;
	char buf[TMP_BUF_SIZE] = {0};
	char tmp[6] = {0} ;
	if(NULL == (fp = fopen(TMP_FILE, "r+")) )
	{
		ENG_LOG("no %s\n", TMP_FILE);
		return ret;
	}
	len = strlen(STR_RET_STATUS);
	while( !feof(fp) )
	{
		fgets(buf, TMP_BUF_SIZE, fp);
		str1 = strstr(buf, STR_RET_STATUS);
		str2 = strstr(buf, STR_RET_END);
		if( (NULL != str1) && (NULL != str2) )
		{
			len = (int)str2 - (int)str1 - len;
			if( (len >= 1) && (len <= 5) )
			{
				memcpy(tmp, str1 + strlen(STR_RET_STATUS),  len);
				ret = atoi(tmp);
				break;
			}
		}
		memset(buf, 0, TMP_BUF_SIZE);
	}
	fclose(fp);
	return ret;
}

static int get_iwnpi_rssi_ret(void)
{
	FILE *fp = NULL;
	char *str1 = NULL;
	char *str2 = NULL;
	int ret = -100;
	int len;
	char buf[TMP_BUF_SIZE] = {0};
	char tmp[6] = {0} ;
	if(NULL == (fp = fopen(TMP_FILE, "r+")) )
	{
		ENG_LOG("no %s\n", TMP_FILE);
		return ret;
	}
	len = strlen(STR_RET_RET);
	while( !feof(fp) )
	{
		fgets(buf, TMP_BUF_SIZE, fp);
		str1 = strstr(buf, STR_RET_RET);
		str2 = strstr(buf, STR_RET_END);
		if( (NULL != str1) && (NULL != str2) )
		{
			len = (int)str2 - (int)str1 - len;
			if( (len >= 1) && (len <= 5) )
			{
				memcpy(tmp, str1 + strlen(STR_RET_RET),  len);
				ret = atoi(tmp);
				
				break;
			}
		}
		memset(buf, 0, TMP_BUF_SIZE);
	}
	fclose(fp);
	return ret;
}

static int get_iwnpi_ret(char *tmp, char *start, char *end)
{
	FILE *fp = NULL;
	char *str1 = NULL;
	char *str2 = NULL;
	int ret = -100;
	int len;
	char buf[TMP_BUF_SIZE] = {0};
	if((NULL == tmp)||(NULL == start)||(NULL == end))
	{
		ENG_LOG("%s(), par NULL\n",  __FUNCTION__);
		return -1;
	}
	if(NULL == (fp = fopen(TMP_FILE, "r+")) )
	{
		ENG_LOG("no %s\n", TMP_FILE);
		return ret;
	}
	len = strlen(start);
	while( !feof(fp) )
	{
		fgets(buf, TMP_BUF_SIZE, fp);
		str1 = strstr(buf, start);
		str2 = strstr(buf, end);
		if( (NULL != str1) && (NULL != str2) )
		{
			len = (int)str2 - (int)str1 - len;
			if(len > 3)
			{
				memcpy(tmp, str1 + strlen(start),  len);
				//sscanf(tmp, " rx_end_count=%d rx_err_end_count=%d fcs_fail_count=%d ", &(cnt->rx_end_count), &(cnt->rx_err_end_count), &(cnt->fcs_fail_count)  );
				ret = 0;
				break;
			}
			
		}
		memset(buf, 0, TMP_BUF_SIZE);
	}
	fclose(fp);
	return ret;
	
}


static int get_iwnpi_rxpktcnt(RX_PKTCNT *cnt)
{
	FILE *fp = NULL;
	char *str1 = NULL;
	char *str2 = NULL;
	int ret = -100;
	int len;
	char buf[TMP_BUF_SIZE] = {0};
	char tmp[6] = {0} ;
	if(NULL == (fp = fopen(TMP_FILE, "r+")) )
	{
		ENG_LOG("no %s\n", TMP_FILE);
		return ret;
	}
	len = strlen(STR_RET_REG_VALUE);
	while( !feof(fp) )
	{
		fgets(buf, TMP_BUF_SIZE, fp);
		str1 = strstr(buf, STR_RET_REG_VALUE);
		str2 = strstr(buf, STR_RET_END);
		if( (NULL != str1) && (NULL != str2) )
		{
			len = (int)str2 - (int)str1 - len;
			if(len > 10)
			{
				memcpy(tmp, str1 + strlen(STR_RET_REG_VALUE),  len);
				sscanf(tmp, " rx_end_count=%d rx_err_end_count=%d fcs_fail_count=%d ", &(cnt->rx_end_count), &(cnt->rx_err_end_count), &(cnt->fcs_fail_count)  );
				ret = 0;
				break;
			}
			
		}
		memset(buf, 0, TMP_BUF_SIZE);
	}
	fclose(fp);
	return ret;
}


static int start_wifieut(char *rsp)
{
	int ret;
	char cmd[100] = {0};
	ENG_LOG("%s()...\n", __FUNCTION__);	
	if(1 == g_wifi_data.eut_enter)
		goto ok;
	system("rmmod ittiam.ko");
	ret = system("insmod /system/lib/modules/ittiam.ko ");
	if(ret < 0)
	{
		ENG_LOG("%s, insmod /system/lib/modules/ittiam.ko¡¡err\n",  __FUNCTION__);
		goto err;
	}
	system("ifconfig wlan0 up");
	sprintf(cmd, "iwnpi wlan0 start > %s", TMP_FILE);
	ret = system(cmd);
	if(ret < 0)
	{
		ENG_LOG("%s, no iwnpi\n",  __FUNCTION__);
		goto err;
	}
	ret  = get_iwnpi_ret_status();
	if(0 != ret)
	{
		ENG_LOG("iwnpi wlan0 start cmd  err_code:%d", ret);
		goto err;
	}
ok:
	strcpy(rsp, EUT_WIFI_OK);
	memset(&g_wifi_data, 0, sizeof(WIFI_ELEMENT) );
	g_wifi_data.eut_enter = 1;
	rsp_debug(rsp);
	return 0;
err:
	sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
	g_wifi_data.eut_enter = 0;
	rsp_debug(rsp);
	return -1;
}


static int stop_wifieut(char *rsp)
{
	int ret;
	char cmd[100] = {0};
	wifi_status_dump();
	if(1 == g_wifi_data.tx_start)
	{	
		sprintf(cmd, "iwnpi wlan0 tx_stop > %s", TMP_FILE);
		ret = system(cmd);
		if(0 != ret)
			goto noiwnpi;
	}
	if(1 == g_wifi_data.rx_start)
	{
		sprintf(cmd, "iwnpi wlan0 rx_stop > %s", TMP_FILE);
		ret = system(cmd);
		if(0 != ret)
			goto noiwnpi;		
	}
	sprintf(cmd, "iwnpi wlan0 stop > %s", TMP_FILE);
	ret = system(cmd);
	if(ret < 0)
		goto noiwnpi;
	ret  = get_iwnpi_ret_status();
	if(0 != ret)
	{
		ENG_LOG("iwnpi wlan0 stop cmd  err_code:%d", ret);
		goto err;
	}
	system("ifconfig wlan0 down");
	system("rmmod ittiam.ko");
	memset(&g_wifi_data, 0, sizeof(WIFI_ELEMENT) );
	strcpy(rsp, EUT_WIFI_OK);
	rsp_debug(rsp);
	return 0;

noiwnpi:
	ENG_LOG("%s, no iwnpi\n",  __FUNCTION__);
err:
	sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
	rsp_debug(rsp);
	return -1;
}

int wifi_eut_set(int cmd, char *rsp)
{
	ENG_LOG("%s(), cmd:%d\n", __FUNCTION__, cmd);
	if(cmd == 1)
		start_wifieut(rsp);
	else if(cmd == 0)
		stop_wifieut(rsp);
	else	
	{
		ENG_LOG("%s(), cmd don't support", __FUNCTION__);
		sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
	}
	return 0;
}

int wifi_eut_get(char *rsp)
{
	ENG_LOG("%s()...\n", __FUNCTION__);
	sprintf(rsp, "%s%d", EUT_WIFI_REQ, g_wifi_data.eut_enter);
	rsp_debug(rsp);
	return 0;
}

int wifi_rate_set(char *string, char *rsp)
{
	int ret = -1;
	int rate = -1;
	char cmd[100] = {0};
	if(0 == g_wifi_data.eut_enter)
	{
		ENG_LOG("%s(), wifi_eut_enter:%d", __FUNCTION__, g_wifi_data.eut_enter);
		goto err;
	}
	rate = mattch_rate_table_str(string);
	ENG_LOG("%s(), rate:%d", __FUNCTION__, rate);
	if(-1 == rate)
		goto err;
	sprintf(cmd, "iwnpi wlan0 set_rate %d > %s", rate, TMP_FILE);
	ret = system(cmd);
	if(ret < 0 )
	{
		ENG_LOG("no iwnpi\n");
		goto err;
	}
	ret  = get_iwnpi_ret_status();
	if(0 != ret)
	{
		ENG_LOG("%s(), set_rate ret:%d", __FUNCTION__, ret);
		goto err;
	}
	
	g_wifi_data.rate = rate;
	strcpy(rsp, EUT_WIFI_OK);
	rsp_debug(rsp);
	return 0;
err:
	sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
	rsp_debug(rsp);
	return -1;
}

int wifi_rate_get(char *rsp)
{
	int ret = -1;
	char *str = NULL;
	ENG_LOG("%s()...\n", __FUNCTION__);
	if(0 == g_wifi_data.rate)
		goto err;
	str = mattch_rate_table_index(g_wifi_data.rate);
	if(NULL == str)
	{
		ENG_LOG("%s(), don't mattch rate", __FUNCTION__);
		goto err;
	}
	
	sprintf(rsp, "%s%s", WIFI_RATE_REQ_RET, str);
	rsp_debug(rsp);
	return 0;
err:
	sprintf(rsp, "%s%s", WIFI_RATE_REQ_RET, "null");
	rsp_debug(rsp);
	return -1;
}

int wifi_channel_set(int ch,  char *rsp)
{
	int ret = -1;
	char cmd[100] = {0};
	if(0 == g_wifi_data.eut_enter)
	{
		ENG_LOG("%s(), wifi_eut_enter:%d", __FUNCTION__, g_wifi_data.eut_enter);
		goto err;
	}	
	ENG_LOG("%s(), ch:%d\n", __FUNCTION__, ch);
	if( (ch < 1 ) || (ch > 14) )
	{
		ENG_LOG("%s(), channel num err\n", __FUNCTION__);
		goto err;
	}
	sprintf(cmd, "iwnpi wlan0 set_channel %d > %s",  ch, TMP_FILE);
	ret = system(cmd);
	if(ret < 0 )
	{
		ENG_LOG("no iwnpi\n");
		goto err;
	}
	ret = get_iwnpi_ret_status();
	if(0 != ret)
	{
		ENG_LOG("%s(), set_channel err ret:%d\n", __FUNCTION__, ret);
		goto err;
	}
	
	g_wifi_data.channel = ch;
	strcpy(rsp, EUT_WIFI_OK);
	rsp_debug(rsp);
	return 0;
err:

	g_wifi_data.channel = 0;
	sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
	rsp_debug(rsp);
	return -1;
}

int wifi_channel_get(char *rsp)
{
	ENG_LOG("%s(), channel:%d\n",  __FUNCTION__,  g_wifi_data.channel);
	if(0 == g_wifi_data.channel)
	{
		goto err;
	}
	
	sprintf(rsp, "%s%d", WIFI_CHANNEL_REQ_RET, g_wifi_data.channel);
	rsp_debug(rsp);
	return 0;
err:
	sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
	rsp_debug(rsp);
	return -1;
}

int wifi_txgainindex_set(int index, char *rsp)
{
	int ret = -1;
	char cmd[100] = {0};
	if(0 == g_wifi_data.eut_enter)
	{
		ENG_LOG("%s(), wifi_eut_enter:%d", __FUNCTION__, g_wifi_data.eut_enter);
		goto err;
	}		
	ENG_LOG("%s(), index:%d\n",  __FUNCTION__, index);
	sprintf(cmd, "iwnpi wlan0 set_tx_power %d > %s", index, TMP_FILE);
	ret = system(cmd);
	if(ret  < 0 )
	{
		ENG_LOG("no iwnpi\n");
		goto err;
	}
	ret = get_iwnpi_ret_status();
	if(0 != ret)
	{
		ENG_LOG("%s(), set_tx_power err ret:%d\n", __FUNCTION__, ret);
		goto err;
	}
	g_wifi_data.txgainindex = index;
	rsp_debug(rsp);
	strcpy(rsp, EUT_WIFI_OK);
	return 0;
err:
	g_wifi_data.txgainindex = -1;
	sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
	rsp_debug(rsp);
	return -1;
}

int wifi_txgainindex_get(char *rsp)
{
	int ret = -1;
	char cmd[100] = {0};
	int level_a, level_b;
	wifi_status_dump();
	if( 0 == g_wifi_data.eut_enter)
		goto err;
	sprintf(cmd, "iwnpi wlan0 get_tx_power > %s", TMP_FILE);
	ret = system(cmd);
	if(ret  < 0 )
	{
		ENG_LOG("no iwnpi\n");
		goto err;
	}
	memset(cmd, 0, sizeof(cmd));
	ret = get_iwnpi_ret(cmd,STR_RET_RET,STR_RET_END);
	if(0 != ret)
	{
		ENG_LOG("%s(), get_tx_power run err\n", __FUNCTION__ );
		goto err;
	}
	sscanf(cmd, "level_a:%d,level_b:%d\n", &level_a, &level_b);
	sprintf(rsp, "%s%d,%d", WIFI_TXGAININDEX_REQ_RET,level_a,level_b);
	rsp_debug(rsp);
	return 0;
err:
	sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
	rsp_debug(rsp);
	return -1;
}

static int wifi_tx_start(char *rsp)
{
	int ret;
	char cmd[100] = {0};
	wifi_status_dump();
	if(1 == g_wifi_data.tx_start)
		goto ok;
	if(1 == g_wifi_data.rx_start)
		goto err;
	sprintf(cmd, "iwnpi wlan0 tx_start > %s", TMP_FILE);
	ret = system(cmd);
	if(ret < 0)
	{
		ENG_LOG("%s, no iwnpi\n",  __FUNCTION__);
		goto err;
	}
	ret  = get_iwnpi_ret_status();
	if(0 != ret)
	{
		ENG_LOG("iwnpi wlan0 tx_start cmd  err_code:%d", ret);
		goto err;
	}
ok:
	g_wifi_data.tx_start = 1;
	strcpy(rsp, EUT_WIFI_OK);
	rsp_debug(rsp);
	return 0;
err:
	sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
	g_wifi_data.tx_start = 0;
	rsp_debug(rsp);
	return -1;

}


static int wifi_tx_stop(char *rsp)
{
	int ret;
	char cmd[100] = {0};
	wifi_status_dump();
	if(0 == g_wifi_data.tx_start)
		goto ok;
	if(1 == g_wifi_data.rx_start)
		goto err;
	sprintf(cmd, "iwnpi wlan0 tx_stop > %s", TMP_FILE);
	ret = system(cmd);
	if(ret < 0)
	{
		ENG_LOG("%s, no iwnpi\n",  __FUNCTION__);
		goto err;
	}
	ret  = get_iwnpi_ret_status();
	if(0 != ret)
	{
		ENG_LOG("iwnpi wlan0 tx_stop cmd  err_code:%d", ret);
		goto err;
	}
ok:
	g_wifi_data.tx_start = 0;
	strcpy(rsp, EUT_WIFI_OK);
	rsp_debug(rsp);
	return 0;
err:
	sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
	g_wifi_data.tx_start = 0;
	rsp_debug(rsp);
	return -1;

}

int wifi_tx_set(int command_code, char *rsp)
{
	ENG_LOG("%s(), command_code:%d\n", __FUNCTION__, command_code);
	if(0 == g_wifi_data.eut_enter)
	{
		ENG_LOG("%s(), wifi_eut_enter:%d", __FUNCTION__, g_wifi_data.eut_enter);
		sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
		return -1;
	}		
	if((1 == command_code)&&(0 == g_wifi_data.rx_start))
		wifi_tx_start(rsp);
	else if(command_code == 0)
		wifi_tx_stop(rsp);
	else
	{
		sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
		rsp_debug(rsp);
		return -1;
	}
	return 0;
}

int wifi_tx_get(char *rsp)
{
	ENG_LOG("%s()...\n", __FUNCTION__);
	sprintf(rsp, "%s%d", EUT_WIFI_TX_REQ, g_wifi_data.tx_start);
	rsp_debug(rsp);
	return 0;
}

static int wifi_rx_start(char *rsp)
{
	int ret;
	char cmd[100] = {0};
	wifi_status_dump();
	if(1 == g_wifi_data.rx_start)
		goto ok;
	if(1 == g_wifi_data.tx_start)
		goto err;
	sprintf(cmd, "iwnpi wlan0 rx_start > %s", TMP_FILE);

	ret = system(cmd);
	if(ret < 0)
	{
		ENG_LOG("%s, no iwnpi\n",  __FUNCTION__);
		goto err;
	}
	ret  = get_iwnpi_ret_status();
	if(0 != ret)
	{
		ENG_LOG("iwnpi wlan0 rx_start cmd  err_code:%d", ret);
		goto err;
	}

ok:
	g_wifi_data.rx_start = 1;
	strcpy(rsp, EUT_WIFI_OK);
	rsp_debug(rsp);
	return 0;
err:
	sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
	g_wifi_data.rx_start = 0;
	rsp_debug(rsp);
	return -1;
}


static int wifi_rx_stop(char *rsp)
{
	int ret;
	char cmd[100] = {0};
	wifi_status_dump();
	if(0 == g_wifi_data.rx_start)
		goto ok;
	if(1 == g_wifi_data.tx_start)
		goto err;
	sprintf(cmd, "iwnpi wlan0 rx_stop > %s", TMP_FILE);

	ret = system(cmd);
	if(ret < 0 )
	{
		ENG_LOG("%s, no iwnpi\n",  __FUNCTION__);
		goto err;
	}
	ret  = get_iwnpi_ret_status();
	if(0 != ret)
	{
		ENG_LOG("iwnpi wlan0 rx_stop cmd  err_code:%d", ret);
		goto err;
	}

ok:
	g_wifi_data.rx_start = 0;
	strcpy(rsp, EUT_WIFI_OK);
	rsp_debug(rsp);
	return 0;
err:
	sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
	g_wifi_data.rx_start = 0;
	rsp_debug(rsp);
	return -1;

}

int wifi_rx_set(int command_code, char *rsp)
{
	ENG_LOG("%s()...\n", __FUNCTION__);
	if(0 == g_wifi_data.eut_enter)
	{
		ENG_LOG("%s(), wifi_eut_enter:%d", __FUNCTION__, g_wifi_data.eut_enter);
		sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
		return -1;
	}		
	if( (command_code == 1)&&(0 == g_wifi_data.tx_start) )
		wifi_rx_start(rsp);
	else if(command_code == 0)
		wifi_rx_stop(rsp);
	else
	{
		sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
		rsp_debug(rsp);
		return -1;
	}
	return 0;
}

int wifi_rx_get(char *rsp)
{
	ENG_LOG("%s()...\n", __FUNCTION__);
	sprintf(rsp, "%s%d", EUT_WIFI_RX_REQ, g_wifi_data.rx_start);
	rsp_debug(rsp);
	return 0;
}

int wifi_rssi_get(char *rsp)
{
	int ret;
	char cmd[100] = {0};
	wifi_status_dump();
	if( 1 == g_wifi_data.tx_start )
	{
		goto err;
	}
	sprintf(cmd, "iwnpi wlan0 get_rssi > %s", TMP_FILE);
	ret = system(cmd);
	if(ret < 0)
	{
		ENG_LOG("%s, no iwnpi\n",  __FUNCTION__);
		goto err;
	}
	ret  = get_iwnpi_rssi_ret();
	if(-100 == ret)
	{
		ENG_LOG("iwnpi wlan0 get_rssi cmd  err_code:%d", ret);
		goto err;
	}
	
	sprintf(rsp, "%s0x%x", EUT_WIFI_RSSI_REQ_RET, ret);
	rsp_debug(rsp);
	return 0;
err:
	sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
	rsp_debug(rsp);
	return -1;
}


int wifi_rxpktcnt_get(char *rsp)
{
	int ret = -1;
	RX_PKTCNT *cnt = NULL;
	char cmd[100] = {0};
	wifi_status_dump();
	if((1 == g_wifi_data.tx_start) )
	{
		goto err;
	}
	sprintf(cmd, "iwnpi wlan0 get_rx_ok > %s", TMP_FILE);
	ret = system(cmd);
	if(ret < 0)
	{
		ENG_LOG("%s, no iwnpi\n",  __FUNCTION__);
		goto err;		
	}
	cnt = &(g_wifi_data.cnt);
	ret = get_iwnpi_rxpktcnt(cnt);
	if(0 != ret)
	{
		ENG_LOG("%s, no iwnpi\n",  __FUNCTION__);
		goto err;
	}
	
	sprintf(rsp, "%s%d,%d,%d", EUT_WIFI_RXPKTCNT_REQ_RET, g_wifi_data.cnt.rx_end_count, g_wifi_data.cnt.rx_err_end_count, g_wifi_data.cnt.fcs_fail_count);
	rsp_debug(rsp);
	return 0;
err:
	sprintf(rsp, "%s%s", EUT_WIFI_ERROR, "error");
	rsp_debug(rsp);
	return -1;
}






