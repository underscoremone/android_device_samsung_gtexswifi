#ifndef _ENG_OPT_H
#define _ENG_OPT_H

#include <stdio.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif
#ifdef LOG_TAG
#undef LOG_TAG
#endif

#define LOG_TAG 	"ENGPC"
#include <utils/Log.h>
#define ENG_AT_LOG  ALOGD

#define ENG_TRACING

#ifdef ENG_TRACING
#define ENG_LOG  ALOGD
#else
    //#define ENG_LOG  ALOGD
#define  ENG_LOG(format, ...)
#endif

#define ENG_EUT			"EUT"
#define ENG_EUT_REQ		"EUT?"
#define ENG_WIFICH_REQ		"CH?"
#define ENG_WIFICH		"CH"
#define ENG_WIFIMODE    "MODE"
#define ENG_WIFIRATIO_REQ   "RATIO?"
#define ENG_WIFIRATIO   "RATIO"
#define ENG_WIFITX_FACTOR_REQ "TXFAC?"
#define ENG_WIFITX_FACTOR "TXFAC"
#define ENG_WIFITX_REQ      "TX?"
#define ENG_WIFITX      "TX"
#define ENG_WIFIRX_REQ      "RX?"
#define ENG_WIFIRX      "RX"
#define ENG_WIFI_CLRRXPACKCOUNT	"CLRRXPACKCOUNT"
#define ENG_GPSSEARCH_REQ	"SEARCH?"
#define ENG_GPSSEARCH	"SEARCH"
#define ENG_GPSPRNSTATE_REQ	"PRNSTATE?"
#define ENG_GPSSNR_REQ	"SNR?"
#define ENG_GPSPRN	"PRN"
//------------------------------------------------

#define ENG_WIFIRATE                "RATE"
#define ENG_WIFIRATE_REQ            "RATE?" 
#define ENG_WIFITXGAININDEX         "TXGAININDEX"
#define ENG_WIFITXGAININDEX_REQ     "TXGAININDEX?"
#define ENG_WIFIRX_PACKCOUNT	    "RXPACKCOUNT?"
#define ENG_WIFIRSSI_REQ            "RSSI?"

//------------------------------------------------


#define ENG_AT_CHANNEL

    typedef enum{
        EUT_REQ_INDEX = 0,
        EUT_INDEX,
        WIFICH_REQ_INDEX,
        WIFICH_INDEX,
        WIFIMODE_INDEX,
        WIFIRATIO_REQ_INDEX,
        WIFIRATIO_INDEX,
        WIFITX_FACTOR_REQ_INDEX,
        WIFITX_FACTOR_INDEX,
        WIFITX_REQ_INDEX,
        WIFITX_INDEX,
        WIFIRX_REQ_INDEX,
        WIFIRX_INDEX,
        WIFIRX_PACKCOUNT_INDEX,
        WIFICLRRXPACKCOUNT_INDEX,
        GPSSEARCH_REQ_INDEX,
        GPSSEARCH_INDEX,
        GPSPRNSTATE_REQ_INDEX,
        GPSSNR_REQ_INDEX,
        GPSPRN_INDEX,
//------------------------------------------------
		ENG_WIFIRATE_INDEX,
		ENG_WIFIRATE_REQ_INDEX,
		ENG_WIFITXGAININDEX_INDEX,
		ENG_WIFITXGAININDEX_REQ_INDEX,
		ENG_WIFIRSSI_REQ_INDEX,
//------------------------------------------------        
    }eut_cmd_enum;

    typedef enum{
        BT_MODULE_INDEX=0,
        WIFI_MODULE_INDEX,
        GPS_MODULE_INDEX,
    }eut_modules;

    struct eut_cmd{
        int index;
        char *name;
    };

    typedef enum{
        CONNECT_UART  = 0,
        CONNECT_USB,
        CONNECT_PIPE,
    }eng_connect_type;

    typedef enum{
        ENG_DIAG_RECV_TO_AP,
        ENG_DIAG_RECV_TO_CP,
    }eng_diag_state;

    struct eng_param{
        int califlag;
        int engtest;
        int cp_type;                /*td: CP_TD; wcdma:CP_WCDMA*/
        int connect_type;     /*usb:CONNECT_USB ; uart:CONNECT_UART*/
        int nativeflag;         /*0: vlx, CP directly communicates with PC tool
                                 *1: native, AP directly communicates with PC tool  */
	 int normal_cali;
	};

    typedef  pthread_t                 eng_thread_t;

    typedef void*  (*eng_thread_func_t)( void*  arg );

    int  eng_thread_create( eng_thread_t  *pthread, eng_thread_func_t  start, void*  arg );

#ifdef __cplusplus
}
#endif

#endif
