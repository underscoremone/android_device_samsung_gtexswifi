#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include "engopt.h"
#include "eng_pcclient.h"
#include "eng_cmd4linuxhdlr.h"

static eng_dev_info_t* s_dev_info;
static int at_mux_fd = -1;
static int pc_fd=-1;

static int start_gser(char* ser_path)
{
    struct termios ser_settings;

    if (pc_fd>=0){
        ENG_LOG("%s ERROR : %s\n", __FUNCTION__, strerror(errno));
        close(pc_fd);
    }

    ENG_LOG("open serial\n");
    pc_fd = open(ser_path,O_RDWR);
    if(pc_fd < 0) {
        ENG_LOG("cannot open vendor serial\n");
        return -1;
    }

    tcgetattr(pc_fd, &ser_settings);
    cfmakeraw(&ser_settings);

    ser_settings.c_lflag |= (ECHO | ECHONL);
    ser_settings.c_lflag &= ~ECHOCTL;
    tcsetattr(pc_fd, TCSANOW, &ser_settings);

    return 0;
}

#ifdef CONFIG_BQBTEST
static int g_bqb_mode_start = 0;

int eng_controller2tester(char * controller_buf, unsigned int data_len)
{
    int len;

    len = write(pc_fd,controller_buf,data_len);

    ENG_LOG("bqb test eng_controller2tester %d", len);
    return len;
}
#endif

static void *eng_readpcat_thread(void *par)
{
    int len;
    int written;
    int cur;
    char engbuf[ENG_BUFFER_SIZE];
    char databuf[ENG_BUFFER_SIZE];
    int i, offset_read, length_read, status;
    eng_dev_info_t* dev_info = (eng_dev_info_t*)par;

    for(;;){
        ENG_LOG("%s: wait pcfd=%d\n",__func__,pc_fd);

read_again:
        memset(engbuf, 0, ENG_BUFFER_SIZE);
        if (pc_fd >= 0){
            len = read(pc_fd, engbuf, ENG_BUFFER_SIZE);
            ENG_LOG("%s: wait pcfd=%d buf=%s len=%d",__func__,pc_fd,engbuf,len);

            if (len <= 0) {
                ENG_LOG("%s: read length error %s",__FUNCTION__,strerror(errno));
                sleep(1);
                start_gser(dev_info->host_int.dev_at);
                goto read_again;
            }else{

            #ifdef CONFIG_BQBTEST
		if(strstr(engbuf, "AT+SPBQBTEST=1")) {
			int ret = 0;
			ENG_LOG("bqb test receive AT+SPBQBTEST");
			g_bqb_mode_start = 1;

			 write(pc_fd, "OK\n", strlen("OK\n"));

			 ret = eng_controller_bqb_start();
			ENG_LOG("bqb test test eng_controller_bqb_start ret %d", ret);

			continue;
		}

		if(1 == g_bqb_mode_start) {
		    ENG_LOG("bqb test  send data to cp2, len = %d", len);
		    eng_send_data(engbuf, len);

		    continue;
		}
	    #endif

                // Just send to modem transparently.
                if(at_mux_fd >= 0) {
                    cur = 0;
                    while(cur < len) {
                        do {
                            written = write(at_mux_fd, engbuf + cur, len -cur);
                            ENG_LOG("muxfd=%d, written=%d\n", at_mux_fd, written);
                        }while(written < 0 && errno == EINTR);

                        if(written < 0) {
                            ENG_LOG("%s: write length error %s\n", __FUNCTION__, strerror(errno));
                            break;
                        }
                        cur += written;
                    }
                }else {
                    ENG_LOG("muxfd fail?");
                }
            }
        }else{
            sleep(1);
            start_gser(dev_info->host_int.dev_at);
        }
    }
    return NULL;
}

static void *eng_readmodemat_thread(void *par)
{
    int ret;
    int len;
    char engbuf[ENG_BUFFER_SIZE];
    eng_dev_info_t* dev_info = (eng_dev_info_t*)par;

    for(;;){
        ENG_LOG("%s: wait pcfd=%d\n",__func__,pc_fd);
        memset(engbuf, 0, ENG_BUFFER_SIZE);
        len = read(at_mux_fd, engbuf, ENG_BUFFER_SIZE);
        ENG_LOG("muxfd =%d buf=%s,len=%d\n",at_mux_fd,engbuf,len);
        if (len <= 0) {
            ENG_LOG("%s: read length error %s\n",__FUNCTION__,strerror(errno));
            sleep(1);
            continue;
        }else{
write_again:
            if (pc_fd>=0){
                ret = write(pc_fd,engbuf,len);
                if (ret <= 0) {
                    ENG_LOG("%s: write length error %s\n",__FUNCTION__,strerror(errno));
                    sleep(1);
                    start_gser(dev_info->host_int.dev_at);
                    goto write_again;
                }
            }else{
                sleep(1);
            }
        }
    }
    return NULL;
}

int eng_at_pcmodem(eng_dev_info_t* dev_info)
{
    eng_thread_t t1,t2;

    ENG_LOG("%s ",__func__);

    start_gser(dev_info->host_int.dev_at);

    at_mux_fd = open(dev_info->modem_int.at_chan, O_RDWR);
    if(at_mux_fd < 0){
        ENG_LOG("%s: open %s fail [%s]\n",__FUNCTION__, dev_info->modem_int.at_chan,strerror(errno));
        return -1;
    }

    if (0 != eng_thread_create( &t1, eng_readpcat_thread, (void*)dev_info)){
        ENG_LOG("read pcat thread start error");
    }

    if (0 != eng_thread_create( &t2, eng_readmodemat_thread, (void*)dev_info)){
        ENG_LOG("read modemat thread start error");
    }
    return 0;
}
