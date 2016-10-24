#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <cutils/sockets.h>
#include <pthread.h>
#include <utils/Log.h>
#include "../engopt.h"
#include "eng_bcmd_test.h"
#include <string.h>
static char socket_read_buf[SOCKET_BUF_LEN] = {0};
static char socket_write_buf[SOCKET_BUF_LEN] = {0};
#define EUT_WIFI_RXPKTCNT_REQ_RET "+SPWIFITEST:RXPACCOUNT="
#define EUT_WIFI_PARAM_MAXCNT  6


static int str_split(char* buf,char* delim,char* destrArr[],int maxcnt) {
	char *p;
	char* outer=NULL;
	int i =0;
	ENG_LOG("buf =%s ",buf );
	p = strtok_r(buf, delim, &outer);
	while(p != NULL) {
		destrArr[i] = p;
		ENG_LOG("aaacmd i=%d  cmdbuf=%s \n",i,p );
		p = strtok_r(NULL, delim, &outer);
		i++;
	}
	if(i>maxcnt) {
		ENG_LOG("str_split exceed the max count maxcnt=%d  i=%d  \n",maxcnt,i );
		return -1;
	}
       return i;
}



int hardware_sprd_wifi_test(char* buf,char *rspbuf)
{
	char req[256] = {0};
	int channel = 0;
	int rate = 0;
	int powerLevel = 0;
	char cmd[16] = {0};
	char* parameterArr[EUT_WIFI_PARAM_MAXCNT] = {0};
	int parameterCnt = 0;
	long pktcnt=0;
	memset(req, 0, sizeof(req));
	memset(cmd, 0, sizeof(cmd));
	parameterCnt = str_split(buf+TYPE_OFFSET," ",parameterArr,EUT_WIFI_PARAM_MAXCNT);
	if( parameterCnt < 1){
		//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
		return -1;
	}

	if (0 == strncmp(buf+TYPE_OFFSET,WIFI_EUT_START,strlen(WIFI_EUT_START))) {
		ENG_LOG("hardware_sprd_wifi_test START");
		wifieut(OPEN_WIFI,req);
		if (!strcmp(req,EUT_WIFI_OK)) {
			//memcpy(socket_write_buf,TEST_OK,OK_LEN);
			return 6;
		} else {
			//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
			return -1;
		}
	} else if (0 == strncmp(buf+TYPE_OFFSET,WIFI_EUT_STOP,strlen(WIFI_EUT_STOP))) {
		ENG_LOG("hardware_sprd_wifi_test STOP");
		wifieut(CLOSE_WIFI,req);
		if (!strcmp(req,EUT_WIFI_OK)) {
			//memcpy(socket_write_buf,TEST_OK,OK_LEN);
			return 6;
		} else {
			//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
			return -1;
		}
	} else if (0 == strncmp(buf+TYPE_OFFSET,WIFI_EUT_CW_START,strlen(WIFI_EUT_CW_START))) {
		ENG_LOG("hardware_sprd_wifi_test CW_START");
		channel = 1;//ptest->channel;

		set_wifi_ch(channel,req);
		//wifi_sin_wave(req);
		if (!strcmp(req,EUT_WIFI_OK)) {
			//memcpy(socket_write_buf,TEST_OK,OK_LEN);
			return 6;
		} else {
			//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
			return -1;
		}
	} else if (0 == strncmp(buf+TYPE_OFFSET,WIFI_EUT_TX_STOP,strlen(WIFI_EUT_TX_STOP))) {
		ENG_LOG("hardware_sprd_wifi_test type TX_STOP");
		wifi_tx(CLOSE_WIFI,req);
		if (!strcmp(req,EUT_WIFI_OK)) {
			//memcpy(socket_write_buf,TEST_OK,OK_LEN);
			return 6;
		} else {
			//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
			return -1;
		}
	}else if(0 == strncmp(buf+TYPE_OFFSET,WIFI_EUT_SET_LENGTH,strlen(WIFI_EUT_SET_LENGTH))){
		if(parameterArr[1]!=NULL)
		{
			ENG_LOG("hardware_sprd_wifi_test type SET_LENGTH");
			wifi_tx_pktlen(atoi(parameterArr[1]));
			//memcpy(socket_write_buf,TEST_OK,OK_LEN);
			return 6;
		}
		else
		{
			//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
			return -1;
		}
	}else if(0 == strncmp(buf+TYPE_OFFSET,WIFI_EUT_SET_COUNT,strlen(WIFI_EUT_SET_COUNT))){
		if(parameterArr[1]!=NULL)
		{
			ENG_LOG("hardware_sprd_wifi_test type SET_COUNT");
			wifi_tx_pktcount(atoi(parameterArr[1]));
			//memcpy(socket_write_buf,TEST_OK,OK_LEN);
			return 6;
		}
		else
		{
			//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
			return -1;
		}
	}else if (0 == strncmp(buf+TYPE_OFFSET,WIFI_EUT_TX_START,strlen(WIFI_EUT_TX_START))) {
		ENG_LOG("hardware_sprd_wifi_test type TX_START");

		wifi_tx(OPEN_WIFI,req);
		if (!strcmp(req,EUT_WIFI_OK)) {
			//memcpy(socket_write_buf,TEST_OK,OK_LEN);
			return 6;
		} else {
			//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
			return -1;
		}

	} else if (0 == strncmp(buf+TYPE_OFFSET,WIFI_EUT_RX_STOP,strlen(WIFI_EUT_RX_STOP))) {
		ENG_LOG("hardware_sprd_wifi_test type RX_STOP");
		//memcpy(socket_write_buf,TEST_OK,OK_LEN);
		return 6;
	}else if (0 == strncmp(buf+TYPE_OFFSET,WIFI_EUT_RX_START,strlen(WIFI_EUT_RX_START))) {
		ENG_LOG("hardware_sprd_wifi_test RX_START");
		wifi_rx(OPEN_WIFI,req);
		if (!strcmp(req,EUT_WIFI_OK)) {
			wifi_clr_rxpackcount(req);
			if (!strcmp(req,EUT_WIFI_OK)) {
				//memcpy(socket_write_buf,TEST_OK,OK_LEN);
				return 6;
			} else {
				//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
				return -1;
			}
		}else {
			//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
			return -1;
		}
	}else if (0 == strncmp(buf+TYPE_OFFSET,WIFI_EUT_GET_RXOK,strlen(WIFI_EUT_GET_RXOK))){
		ENG_LOG("hardware_sprd_wifi_test GET_RXOK");
		pktcnt=wifi_rxpackcount(req);
		ENG_LOG("hardware_sprd_wifi_test req %s, RET %s",req ,EUT_WIFI_RXPKTCNT_REQ_RET);
		if (0 == strncmp(req,EUT_WIFI_RXPKTCNT_REQ_RET,strlen(EUT_WIFI_RXPKTCNT_REQ_RET))) {
			//memcpy(socket_write_buf,TEST_OK,OK_LEN);
			sprintf(rspbuf,"ret: reg value: rx_end_count=%ld rx_err_end_count=0 fcs_fail_count=0 :end",pktcnt);
			return 6;
		} else {
			//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
			return -1;
		}
	}else if (0 == strncmp(buf+TYPE_OFFSET,WIFI_EUT_SET_CHANNEL,strlen(WIFI_EUT_SET_CHANNEL))){
		ENG_LOG("hardware_sprd_wifi_test SET_CHANNEL");
		int channel = 0;
		if(NULL ==parameterArr[1]) {
			//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
			ENG_LOG("hardware_sprd_wifi_test parameterArr error!");
			return -1;
		}
		channel = atoi(parameterArr[1]);
		set_wifi_ch(channel,req);
		if (!strcmp(req,EUT_WIFI_OK)) {
			//memcpy(socket_write_buf,TEST_OK,OK_LEN);
			return 6;
		} else {
			//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
			return -1;
		}
	}else if (0 == strncmp(buf+TYPE_OFFSET,WIFI_EUT_SET_RATE,strlen(WIFI_EUT_SET_RATE))){
		ENG_LOG("hardware_sprd_wifi_test SET_RATE");
		char* rate;
		if(NULL ==parameterArr[1]) {
			//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
			ENG_LOG("hardware_sprd_wifi_test parameterArr error!");
			return -1;
		}
		set_wifi_rate(parameterArr[1],req);
		if (!strcmp(req,EUT_WIFI_OK)) {
			//memcpy(socket_write_buf,TEST_OK,OK_LEN);
			return 6;
		} else {
			//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
			return -1;
		}
	}else if (0 == strncmp(buf+TYPE_OFFSET,WIFI_EUT_SET_POWER,strlen(WIFI_EUT_SET_POWER))){
		ENG_LOG("hardware_sprd_wifi_test TX_POWER");
		long power = 0;
		if(NULL ==parameterArr[1]) {
			//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
			ENG_LOG("hardware_sprd_wifi_test parameterArr error!");
			return -1;
		}
		power = (long)atoi(parameterArr[1]);
		set_wifi_tx_factor(power,req);
		if (!strcmp(req,EUT_WIFI_OK)) {
			//memcpy(socket_write_buf,TEST_OK,OK_LEN);
			return 6;
		} else {
			//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
			return -1;
		}


	}else if (0 == strncmp(buf+TYPE_OFFSET,CMD_POWER_SAVE,strlen(CMD_POWER_SAVE))){
		ENG_LOG("hardware_sprd_wifi_test CMD_POWER_SAVE");
		wifi_pm(1,req);
		if (!strcmp(req,EUT_WIFI_OK)) {
			//memcpy(socket_write_buf,TEST_OK,OK_LEN);
			return 6;
		} else {
			//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
			return -1;
		}
	}else if (0 == strncmp(buf+TYPE_OFFSET,CMD_DISABLED_POWER_SAVE,strlen(CMD_DISABLED_POWER_SAVE))){
		ENG_LOG("hardware_sprd_wifi_test CMD_DISABLED_POWER_SAVE");
		wifi_pm(0,req);
		if (!strcmp(req,EUT_WIFI_OK)) {
			//memcpy(socket_write_buf,TEST_OK,OK_LEN);
			return 6;
		} else {
			//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
			return -1;
		}
	}else if (0 == strncmp(buf+TYPE_OFFSET,CMD_GET_POWER_SAVE_STATUS,strlen(CMD_GET_POWER_SAVE_STATUS))){
		ENG_LOG("hardware_sprd_wifi_test GET_POWER_SAVE_STATUS");
		if(wifi_pm_sta()==1){
			sprintf(rspbuf,"retult: %d",1);
		}else{
			sprintf(rspbuf,"retult: %d",0);
		}
		return 6;
	}else if (0 == strncmp(buf+TYPE_OFFSET,WIFI_EUT_GUARDINTERVAL,strlen(WIFI_EUT_GUARDINTERVAL)))
	{
		if(parameterArr[1]!=NULL)
		{
			ENG_LOG("hardware_sprd_wifi_test type EUT_GUARDINTERVAL");
			wifi_tx_ipg(atoi(parameterArr[1]));
			//memcpy(socket_write_buf,TEST_OK,OK_LEN);
			return 6;
		}
		else
		{
			//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
			return -1;
		}
	}else if (0 == strncmp(buf+TYPE_OFFSET,WIFI_EUT_SET_PREAMBLE,strlen(WIFI_EUT_SET_PREAMBLE))||
		0 == strncmp(buf+TYPE_OFFSET,WIFI_EUT_BANDWIDTH,strlen(WIFI_EUT_BANDWIDTH))){
		ENG_LOG("!!!===hardware_sprd_wifi_test %s is unsuppoted now!!!!====", buf);
		//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
		return 6;
	}else{
		ENG_LOG("hardware_sprd_wifi_test  shit  error");
		//memcpy(socket_write_buf,TEST_ERROR,ERROR_LEN);
		return 1;
	}
}

