#ifndef _ENG_HARDWARE_TEST_H
#define _ENG_HARDWARE_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include	"../eut_opt.h"

#define SOCKET_BUF_LEN	1024
#define OPEN_WIFI   1
#define CLOSE_WIFI  0
#define OPEN_BT   1
#define CLOSE_BT  0

#define TEST_OK	"OK"
#define TEST_ERROR	"Fail"

#define OK_LEN  strlen(TEST_OK)
#define ERROR_LEN   strlen(TEST_ERROR)

#define TYPE_OFFSET 12
#define CMD_OFFSET 7

#define SPRD_WIFI	1
#define SPRD_BT		2
#define CLOSE_SOCKET    3



#define WIFI_EUT_UP  "iwnpi wlan0 ifaceup"
#define WIFI_EUT_DOWN  "iwnpi wlan0 ifacedown"
#define WIFI_EUT_START  "start"
#define WIFI_EUT_STOP  "stop"
#define WIFI_EUT_SET_CHANNEL  "set_channel"	//_
#define WIFI_EUT_SET_RATE  "set_rate"	//_

// for tx
#define WIFI_EUT_TX_START  "tx_start"
#define WIFI_EUT_TX_STOP  "tx_stop"
#define WIFI_EUT_CW_START  "sin_wave"
#define WIFI_EUT_SET_POWER  "set_tx_power"	//_
/*unsuppot*/
#define WIFI_EUT_SET_LENGTH  "set_pkt_length"	//_
#define WIFI_EUT_SET_COUNT  "set_tx_count"	//_
/*??*/
#define WIFI_EUT_SET_PREAMBLE  "set_preamble"	//_
#define WIFI_EUT_BANDWIDTH  "set_bandwidth"	//_
#define WIFI_EUT_GUARDINTERVAL  "set_guard_interval"	//_
/*unsuppot*/
// for rx
#define WIFI_EUT_RX_START  "rx_start"
#define WIFI_EUT_RX_STOP  "rx_stop"
#define WIFI_EUT_GET_RXOK  "get_rx_ok"

// for reg_wr
#define WIFI_EUT_READ  "get_reg"	//_
#define WIFI_EUT_WRITE  "set_reg"	//_

#define CMD_POWER_SAVE  "lna_on"
#define CMD_DISABLED_POWER_SAVE  "lna_off"
#define CMD_GET_POWER_SAVE_STATUS  "lna_status"



/* Unsigned fixed width types */
typedef uint8_t CsrUint8;
typedef uint16_t CsrUint16;
typedef uint32_t CsrUint32;

/* Signed fixed width types */
typedef int8_t CsrInt8;
typedef int16_t CsrInt16;
typedef int32_t CsrInt32;

/* Boolean */
typedef CsrUint8 CsrBool;


/* MAC address */
typedef struct
{
    CsrUint8 a[6];
} CsrWifiMacAddress;

typedef CsrUint8 CsrWifiPtestPreamble;


typedef struct ptest_cmd{
	CsrUint16 type;
	CsrUint16 band;
	CsrUint16 channel;
	CsrUint16 sFactor;
	union{
		struct {
			CsrUint16 frequency;
			CsrInt16 frequencyOffset;
			CsrUint16 amplitude;
		}ptest_cw;
		struct {
			CsrUint16 rate;
			CsrUint16 powerLevel;
			CsrUint16 length;
			CsrBool enable11bCsTestMode;
			CsrUint32 interval;
			CsrWifiMacAddress destMacAddr;
			CsrWifiPtestPreamble preamble;
		}ptest_tx;
		struct {
			CsrUint16 frequency;
			CsrBool filteringEnable;
		}ptest_rx;
	};
}PTEST_CMD_T;

int hardware_sprd_wifi_test(char* buf,char *rspbuf);
#ifdef __cplusplus
}
#endif

#endif

