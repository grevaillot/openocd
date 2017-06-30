/***************************************************************************
 *   Copyright (C) 2011-2012 by Mathias Kuester                            *
 *   Mathias Kuester <kesmtp@freenet.de>                                   *
 *                                                                         *
 *   Copyright (C) 2012 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Rémi PRUD'HOMME STMicroelectronics 2016                               *
 *                                                                         *
 *   This code is based on https://github.com/texane/stlink                *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* project specific includes */
#include <helper/binarybuffer.h>
#include <jtag/interface.h>
#include <jtag/hla/hla_layout.h>
#include <jtag/hla/hla_transport.h>
#include <jtag/hla/hla_interface.h>
#include <target/target.h>
#include <target/cortex_m.h>

#if WIN32
#include <winsock2.h> 
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/uio.h>
#include <netinet/tcp.h>
#endif


#include "libusb_common.h"

#define CMD_PARSER()\
	char string[15][80];\
	const char  seps[5] = " ";\
	const char *prec = &buf[0];\
	int i = 0;\
	char *token = strtok((char *)prec, seps);\
	while (token != NULL) {\
		strcpy(string[i], token);\
		token = strtok(NULL, seps);\
		i++;\
	} \

#define ENDPOINT_IN  0x80
#define ENDPOINT_OUT 0x00

#define STLINK_WRITE_TIMEOUT 1000
#define STLINK_READ_TIMEOUT 1000

#define BUFFER_LENGTH 120

/** */
struct stlink_tcp_version {
	/** */
	int stlink;
	/** */
	int jtag;
	/** */
	int swim;
	/** highest supported jtag api version */
	/** */
	unsigned short vid;
	/** */
	unsigned short pid;
};

/** */
struct stlink_tcp_handle_s {
	/** */
	/** */
	unsigned int connect_id;

	unsigned int device_id;
  
	int socket;

	/** */
	uint32_t max_mem_packet;
	/** */
	struct stlink_tcp_version version;
	/** */
	uint16_t vid;
	/** */
	uint16_t pid;

	enum hl_transports transport;

	/** this is the currently used jtag api */
	/** */
	struct {
		/** whether SWO tracing is enabled or not */
		bool enabled;
		/** trace module source clock */
		uint32_t source_hz;
	} trace;
	/** reconnect is needed next time we try to query the
	 * status */
	bool reconnect_pending;
};

#define STLINK_TRACE_SIZE               1024
#define STLINK_TRACE_MAX_HZ             2000000
#define STLINK_TRACE_MIN_VERSION        13

/** */
#define STLINK_CORE_RUNNING            0x80
#define STLINK_CORE_HALTED             0x81

#define CHECK_RESPONSE()  buf[0] == '\1'

static bool stlink_tcp_send_string( void *handle, char *cmd_in, char *cmd_out);
static int stlink_tcp_trace_enable(void *handle);
static int stlink_tcp_get_version(void *handle);
static int stlink_tcp_check_voltage(void *handle, float *target_voltage);
static int stlink_usb_exit(void *handle);


/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/** */
static int stlink_tcp_close(void *handle)
{
	struct stlink_tcp_handle_s *h = handle;

	/* exit from jtag mode */
	stlink_usb_exit(h);
        LOG_DEBUG("close stlink socket");
        /* close socket */
        if (h != NULL) {
          close(h->socket);
          free(h);
        }
	return ERROR_OK;
}

/** */
static int stlink_tcp_read_line(void *handle, char *cmd_out) {

	struct stlink_tcp_handle_s *h = (struct stlink_tcp_handle_s *)handle;
        
	long car_readed = 0;
	char c;
	do
	{
          	if ( recv(h->socket, &c, 1, 0) == 1 ) {
                  cmd_out[car_readed] = c;
                } else
                  return 0;
                
		car_readed++;
	}
	while ( c != '\n' && car_readed < BUFFER_LENGTH);

	cmd_out[car_readed-1] = 0;
 	return car_readed;
}

static bool stlink_tcp_send_string( void *handle, char *cmd_in, char *cmd_out) {
	struct stlink_tcp_handle_s *h = (struct stlink_tcp_handle_s *)handle;
        int len = send(h->socket, cmd_in, strlen(cmd_in), 0);
        if ( len  == (int)strlen(cmd_in)) {
          len = stlink_tcp_read_line(h, cmd_out);
          if (cmd_out[0] == '1')
            return true;
        }
        return false;
}

static bool stlink_tcp_write_string_mem( void *handle, char *cmd_in, char *cmd_out, char * buf, int len) {
	struct stlink_tcp_handle_s *h = (struct stlink_tcp_handle_s *)handle;
#ifndef _WIN32
            // il me manque encore les address de destinataire : l'api fait une connection. 
            /* Fill in socket address for the server. We assume a           */
            /* standard port is used.                                       */
            /* socket address for server                                    */
            struct iovec msg_iov[2];
            msg_iov[0].iov_base = cmd_in;
            msg_iov[0].iov_len = strlen(cmd_in);
            msg_iov[1].iov_base = buf;
            msg_iov[1].iov_len = len;
     
            LOG_DEBUG("writev : cmd %s , len = %d, buf len = %d", (char *)msg_iov[0].iov_base, (unsigned int)msg_iov[0].iov_len,  (unsigned int)msg_iov[1].iov_len);        
            unsigned int n = writev(h->socket, msg_iov, 2);
            LOG_DEBUG("writev : returns %d", n); 
            if (n == len + strlen(cmd_in)) {
              stlink_tcp_read_line(h, cmd_out);
              return true;
            } else {
              LOG_INFO("send error");
              return false;
            }
#else
            WSABUF bufs[2];
            unsigned long NumberOfBytesSent;

            bufs[0].buf = cmd_in;
            bufs[0].len = strlen(cmd_in);

            bufs[1].buf = buf;
            bufs[1].len = len;

            //        LOG_DEBUG("WSASend cmd = '%s'", cmd_in);
            //        LOG_DEBUG("WSASend cmd len = %ud, buf len = %ud", (unsigned int)bufs[0].len,  (unsigned int)bufs[1].len);
            int ret = WSASend(h->socket, bufs, 2, &NumberOfBytesSent, 0, NULL, NULL);
            if (ret == SOCKET_ERROR)
              LOG_DEBUG("WSASend returned %d", WSAGetLastError());

            stlink_tcp_read_line(h, cmd_out);
#endif
            return true;
}

static bool stlink_tcp_read_string_mem(void *handle, char *cmd_in, char *cmd_out, char * buffer, int len) 
{
	struct stlink_tcp_handle_s *h = (struct stlink_tcp_handle_s *)handle;

        /* read the buffer after the string if the satus is correct */
        if ( stlink_tcp_send_string( h, cmd_in, cmd_out)) {

          unsigned int received =  0;
          int received2 = 0;
      
          char *buf = cmd_out;
          CMD_PARSER();
         
          unsigned int size_wanted = atoi(string[2]);

          bool completed = false;
          int ind = 0;

          do {
		received2 = recv( h->socket, buffer + received, size_wanted - received, 0);

		if (received2 != -1) {
			received += received2;

			if (received == size_wanted)
				completed = true;
			else
              			ind ++;

			LOG_DEBUG("read the next transfert(%d) of %d byte", ind, received2);
			
            	} else {
#ifdef _WIN32
		  unsigned long err = WSAGetLastError();
	          LOG_DEBUG("recv returned %d", (int)err);
		  if ( err == WSAEWOULDBLOCK)
		    Sleep(50);
#else
	          LOG_DEBUG("recv returned %d", errno);
#endif		  
		  
		}
          } while( !completed);

          return true;
          } 
        else 
		return false;
}


/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
static int stlink_tcp_assert_srst(void *handle, int srst)
{
	struct stlink_tcp_handle_s *h = handle;
	char cmd_in[BUFFER_LENGTH];
	char cmd_out[BUFFER_LENGTH];

	assert(handle != NULL);
        LOG_DEBUG("assert_srst : %d", srst);

	sprintf(cmd_in, "stlink-tcp-assert-srst %d %d\n", h->connect_id, srst); 
	if ( stlink_tcp_send_string(h, cmd_in, cmd_out)) {
		return ERROR_OK;
	} else {
		return ERROR_FAIL;
	}
}

static int stlink_tcp_init_mode(void *handle, int connect_under_reset)
{
        struct stlink_tcp_handle_s *h = handle;
        char cmd_in[BUFFER_LENGTH];
        char cmd_out[BUFFER_LENGTH];

        assert(handle != NULL);


        sprintf(cmd_in, "stlink-usb-init-mode %d %d\n", h->connect_id, connect_under_reset);
        if ( stlink_tcp_send_string(h, cmd_in, cmd_out) ) {
            	return ERROR_OK;
        } else {
         	return ERROR_FAIL;
        }
}

static int stlink_tcp_idcode(void *handle, uint32_t * idcode)
{
        struct stlink_tcp_handle_s *h = handle;
        char cmd_in[BUFFER_LENGTH];
        char cmd_out[BUFFER_LENGTH];
        int status;

        assert(handle != NULL);
	sprintf(cmd_in, "stlink-usb-idcode %x\n",  h->connect_id); 

        if ( stlink_tcp_send_string(h, cmd_in, cmd_out) ) {
	        sscanf(&cmd_out[2], "%d %x", &status, idcode);
                LOG_DEBUG("IDCODE: 0x%08" PRIX32, *idcode);
            	return ERROR_OK;
        } else {
         	return ERROR_FAIL;
        }
}

static int stlink_tcp_read_debug_reg(void *handle, int addr, int * val)
{
        struct stlink_tcp_handle_s *h = handle;
        char cmd_in[BUFFER_LENGTH];
        char cmd_out[BUFFER_LENGTH];
        int status;

        assert(handle != NULL);
	sprintf(cmd_in, "stlink-tcp-read-debug-reg %d %x  %d\n", h->connect_id, addr, 0); 

        if ( stlink_tcp_send_string(h, cmd_in, cmd_out) ) {
            	sscanf(&cmd_out[2], "%d %x", &status, val);
            	return ERROR_OK;
        } else {
         	return ERROR_FAIL;
        }
}

static int stlink_tcp_write_debug_reg(void *handle, uint32_t addr, uint32_t val)
{
        struct stlink_tcp_handle_s *h = handle;
        char cmd_in[BUFFER_LENGTH];
        char cmd_out[BUFFER_LENGTH];

        assert(handle != NULL);

	sprintf(cmd_in, "stlink-tcp-write-debug-reg %d %x %x %d\n", h->connect_id, addr ,val, 0); 
        if ( stlink_tcp_send_string(h, cmd_in, cmd_out) ) {
            	return ERROR_OK;
        } else {
         	return ERROR_FAIL;
        }
}

static int stlink_tcp_read_trace(void *handle, int * buf, int *size)
{
        struct stlink_tcp_handle_s *h = handle;
        char cmd_in[BUFFER_LENGTH];
        char cmd_out[BUFFER_LENGTH];

        assert(handle != NULL);

	sprintf(cmd_in, "stlink-tcp-trace-read %x %x\n", h->connect_id, *size); 
        if ( stlink_tcp_read_string_mem(h, cmd_in, cmd_out, (char *)buf, *size) ) {
            	return ERROR_OK;
        } else {
         	return ERROR_FAIL;
        }
}

static int stlink_tcp_trace_disable(void *handle)
{
        struct stlink_tcp_handle_s *h = handle;
        char cmd_in[BUFFER_LENGTH];
        char cmd_out[BUFFER_LENGTH];

        assert(handle != NULL);

	sprintf(cmd_in, "stlink-tcp-trace-disable %x\n", h->connect_id);
        if ( stlink_tcp_send_string(h, cmd_in, cmd_out) ) {
            	return ERROR_OK;
        } else {
         	return ERROR_FAIL;
        }
}


static int stlink_tcp_trace_enable(void *handle)
{
        struct stlink_tcp_handle_s *h = handle;
        char cmd_in[BUFFER_LENGTH];
        char cmd_out[BUFFER_LENGTH];

        assert(handle != NULL);
        sprintf(cmd_in, "stlink-tcp-trace-enable %x\n", h->connect_id);
        if ( stlink_tcp_send_string(h, cmd_in, cmd_out) ) {
            	return ERROR_OK;
        } else {
         	return ERROR_FAIL;
        }
}

static int stlink_usb_get_version(void *handle, struct stlink_tcp_version *v)
{
        struct stlink_tcp_handle_s *h = handle;
        char cmd_in[BUFFER_LENGTH];
        char buf[BUFFER_LENGTH];

        assert(handle != NULL);
        sprintf(cmd_in, "stlink-tcp-version %d\n",  h->connect_id);
        if ( stlink_tcp_send_string(h, cmd_in, buf)) {
          	CMD_PARSER();
         	sscanf(string[2], "%d", (unsigned int *)&v->stlink);
          	sscanf(string[3], "%d", (unsigned int *)&v->jtag);
          	sscanf(string[4], "%d", (unsigned int *)&v->swim);
          	sscanf(string[5], "%x", (unsigned int *)(void *)&v->pid);
          	sscanf(string[6], "%x", (unsigned int *)(void *)&v->vid);
          	return ERROR_OK;
        } else {
         	return ERROR_FAIL;
        }
}

static int stlink_usb_exit(void *handle)
{
        struct stlink_tcp_handle_s *h = handle;
        char cmd_in[BUFFER_LENGTH];
        char cmd_out[BUFFER_LENGTH];

        assert(handle != NULL);

	sprintf(cmd_in, "stlink-exit-mode %d\n", h->connect_id); 
        if ( stlink_tcp_send_string(h, cmd_in, cmd_out) ) {
             	return ERROR_OK;
        } else {
         	return ERROR_FAIL;
        }
}

static int stlink_usb_check_voltage(void *handle, float *voltage)
{
        struct stlink_tcp_handle_s *h = handle;
        char cmd_in[BUFFER_LENGTH];
        char cmd_out[BUFFER_LENGTH];

        assert(handle != NULL);

	sprintf(cmd_in, "stlink-tcp-check-voltage %d\n", h->connect_id); 
        if ( stlink_tcp_send_string(h, cmd_in, cmd_out) ) {
                  sscanf(&cmd_out[4], "%f", voltage );
            	return ERROR_OK;
        } else {
         	return ERROR_FAIL;
        }
}

static enum target_state stlink_tcp_state(void *handle)
{
        struct stlink_tcp_handle_s *h = handle;
        char cmd_in[BUFFER_LENGTH];
        char cmd_out[BUFFER_LENGTH];
        int state;

        assert(handle != NULL);

	sprintf(cmd_in, "stlink-usb-state %d\n", h->connect_id); 
        if ( stlink_tcp_send_string(h, cmd_in, cmd_out) ) {
          	sscanf(&cmd_out[2], "%d", &state);
               	return state;
        } else {
         	return -1;
        }
}

static int stlink_tcp_reset(void *handle)
{
        struct stlink_tcp_handle_s *h = handle;
        char cmd_in[BUFFER_LENGTH];
        char cmd_out[BUFFER_LENGTH];

        assert(handle != NULL);

        LOG_DEBUG("stlink-usb-reset"); 
	sprintf(cmd_in, "stlink-usb-reset %d\n", h->connect_id); 
        if (stlink_tcp_send_string(h, cmd_in, cmd_out)) {
            	return ERROR_OK;
        } else {
         	return ERROR_FAIL;
        }
}

static int stlink_tcp_read_regs(void *handle)
{
        struct stlink_tcp_handle_s *h = handle;
        char cmd_in[BUFFER_LENGTH];
        char cmd_out[BUFFER_LENGTH];

        assert(handle != NULL);

	sprintf(cmd_in, "stlink-tcp-read-regs %d %x\n", h->connect_id, 0); 
        if (stlink_tcp_send_string(h, cmd_in, cmd_out)) {
            	return ERROR_OK;
        } else {
         	return ERROR_FAIL;
        }
}

static int stlink_tcp_read_reg(void *handle, int num, uint32_t * val)
{
        struct stlink_tcp_handle_s *h = handle;
        char cmd_in[BUFFER_LENGTH];
        char cmd_out[BUFFER_LENGTH];
        uint32_t val2;

        assert(handle != NULL);

	sprintf(cmd_in, "stlink-tcp-read-reg %d %x %x\n", h->connect_id, num, 0); 
        if (stlink_tcp_send_string(h, cmd_in, cmd_out)) {
	        sscanf(&cmd_out[4], "%x", &val2);
                *val = val2;
            	return ERROR_OK;
        } else {
         	return ERROR_FAIL;
        }
}

static int stlink_tcp_write_reg(void *handle, int num, uint32_t val)
{
        struct stlink_tcp_handle_s *h = handle;
        char cmd_in[BUFFER_LENGTH];
        char cmd_out[BUFFER_LENGTH];

        assert(handle != NULL);
        LOG_DEBUG("%s", __func__);

	sprintf(cmd_in, "stlink-tcp-write-reg %d %x %x %x\n", h->connect_id, num , val, 0); 
        if (stlink_tcp_send_string(h, cmd_in, cmd_out) ) {
            	return ERROR_OK;
        } else {
         	return ERROR_FAIL;
        }
}

static int stlink_tcp_write_mem(void *handle, uint32_t addr, uint32_t size, uint32_t count, const uint8_t * buffer)
{
        struct stlink_tcp_handle_s *h = handle;
        char cmd_in[BUFFER_LENGTH];
        char cmd_out[BUFFER_LENGTH];

        assert(handle != NULL);

	if (size == 4 && count == 1) {
		uint32_t v = *(uint32_t*)(void *)buffer; 
		return stlink_tcp_write_debug_reg(handle, addr, v);	  
        } else {
		sprintf(cmd_in, "stlink-tcp-write-mem %d %x %x %x %d\n", h->connect_id, addr , size, count, 0); 
		if (stlink_tcp_write_string_mem(h, cmd_in, cmd_out, (char*)buffer, size * count)) {
			return ERROR_OK;
		} else
			return ERROR_FAIL;
	}
}

static int stlink_tcp_read_mem(void *handle, uint32_t addr, uint32_t size, uint32_t count, uint8_t * buffer)
{
        struct stlink_tcp_handle_s *h = handle;
        char cmd_in[BUFFER_LENGTH];
        unsigned char cmd_out[BUFFER_LENGTH];

        assert(handle != NULL);

        if (size == 4 && count == 1) {
		return stlink_tcp_read_debug_reg(handle, addr, (int*)(void *)buffer);
        } else {
                sprintf(cmd_in, "stlink-tcp-read-mem %d %x %x %x %x\n",  h->connect_id, addr , size, count, 0);

                if ( stlink_tcp_read_string_mem(h, cmd_in, (char *) cmd_out, (char*)buffer, size * count)) {
	                return ERROR_OK;
                } else {
        	        return ERROR_FAIL;
                }
        }
}

static int stlink_tcp_speed(void *handle, int khz, bool query)
{
        struct stlink_tcp_handle_s *h = handle;
        char cmd_in[BUFFER_LENGTH];
        char cmd_out[BUFFER_LENGTH];
        int khz_choosen = 1800;

        if (handle) {
		sprintf(cmd_in, "stlink-tcp-speed %d %x %x\n", h->connect_id, khz ,query); 
		if ( stlink_tcp_send_string(h, cmd_in, cmd_out) ) {
                  sscanf(&cmd_out[2], "%d", &khz_choosen);
                  LOG_INFO("Match for requested speed %d kHz, using %d kHz", khz, khz_choosen);
                }
        }
        
        return khz_choosen;
}

static int stlink_tcp_config_trace(void *handle, int enabled, int e_pin_protocol, int port_size, unsigned int * trace_freq)
{
        struct stlink_tcp_handle_s *h = handle;
        char cmd_in[BUFFER_LENGTH];
        char cmd_out[BUFFER_LENGTH];

        assert(handle != NULL);

	sprintf(cmd_in, "stlink-tcp-config-trace %d %x %x %x\n", h->connect_id, enabled , e_pin_protocol , port_size); 
        if ( stlink_tcp_send_string(h, cmd_in, cmd_out) ) {
            	sscanf(cmd_out, "%d", trace_freq);
            	return ERROR_OK;
        } else {
         	return ERROR_FAIL;
        }
}

/** */
static int stlink_tcp_get_version(void *handle)
{
	struct stlink_tcp_handle_s *h = handle;
        struct stlink_tcp_version v;

	assert(handle != NULL);

        stlink_usb_get_version(h, &v);

	LOG_INFO("STLINK v%d JTAG v%d API v%d VID 0x%04X PID 0x%04X",
		v.stlink,
		v.jtag,
		2,
		v.vid,
		v.pid);

	return ERROR_OK;
}


static int stlink_tcp_check_voltage(void *handle, float *target_voltage)
{
	struct stlink_tcp_handle_s *h = handle;

        stlink_usb_check_voltage(h, target_voltage);
	LOG_INFO("Target voltage: %f", (float)*target_voltage);

	return ERROR_OK;
}

/** */
static int stlink_tcp_trace_read_h(void *handle, uint8_t *buf, size_t *size)
{
	struct stlink_tcp_handle_s *h = handle;

	assert(handle != NULL);

	if (h->trace.enabled && h->version.jtag >= STLINK_TRACE_MIN_VERSION) {
			int res;
			if (*size > 0) {
                          res = stlink_tcp_read_trace(handle, (int *)(void *)buf, (int *)(void *)size);
			if (res != ERROR_OK)
				return res;
			return ERROR_OK;
		}
	}
	*size = 0;
	return ERROR_OK;
}

/** */
static void stlink_tcp_trace_disable_h(void *handle)
{
	int res = ERROR_OK;
	struct stlink_tcp_handle_s *h = handle;

	assert(h != NULL);

        stlink_tcp_trace_disable(h);

	LOG_DEBUG("Tracing: disable");

	if (res == ERROR_OK)
		h->trace.enabled = false;
}

/** */
static int stlink_tcp_trace_enable_h(void *handle)
{
	int res;
	struct stlink_tcp_handle_s *h = handle;


	assert(h != NULL);

	if (h->version.jtag >= STLINK_TRACE_MIN_VERSION) {
          	res = stlink_tcp_trace_enable(handle);
		if (res == ERROR_OK)  {
			h->trace.enabled = true;
			LOG_DEBUG("Tracing: recording at %" PRIu32 "Hz", h->trace.source_hz);
		}
	} else {
		LOG_ERROR("Tracing is not supported by this version.");
		res = ERROR_FAIL;
	}
	return ERROR_OK;
}

/** */
static int stlink_tcp_reset_h(void *handle)
{
	struct stlink_tcp_handle_s *h = handle;
	assert(h != NULL);

        LOG_DEBUG("RESET");

        stlink_tcp_reset(h);
	if (h->trace.enabled) {
		stlink_tcp_trace_disable_h(h);
		int ret =  stlink_tcp_trace_enable_h(h);
                return ret;
	}
	return ERROR_OK;
}

/** */
static int stlink_tcp_run(void *handle)
{
	int res;
	struct stlink_tcp_handle_s *h = handle;

	assert(h != NULL);

        res = stlink_tcp_write_debug_reg(h, DCB_DHCSR, DBGKEY|C_DEBUGEN);
	return res;
}

/** */
static int stlink_tcp_halt(void *handle)
{
	int res;
	struct stlink_tcp_handle_s *h = handle;
	assert(handle != NULL);
        res = stlink_tcp_write_debug_reg(h, DCB_DHCSR, DBGKEY|C_HALT|C_DEBUGEN);
        return res;
}

/** */
static int stlink_tcp_step(void *handle)
{
	struct stlink_tcp_handle_s *h = handle;

	assert(handle != NULL);

        stlink_tcp_write_debug_reg(h, DCB_DHCSR, DBGKEY|C_HALT|C_MASKINTS|C_DEBUGEN);
        stlink_tcp_write_debug_reg(h, DCB_DHCSR, DBGKEY|C_STEP|C_MASKINTS|C_DEBUGEN);
        return stlink_tcp_write_debug_reg(h, DCB_DHCSR, DBGKEY|C_HALT|C_DEBUGEN);
}

/** */

/** */
static int stlink_tcp_override_target(const char *targetname)
{
	return !strcmp(targetname, "cortex_m");
}

static int stlink_config_trace(void *handle, bool enabled, enum tpio_pin_protocol pin_protocol,
			uint32_t port_size, unsigned int *trace_freq)
{
	struct stlink_tcp_handle_s *h = handle;

	if (!enabled) {
		stlink_tcp_trace_disable(h);
		return ERROR_OK;
	}

	if (*trace_freq > STLINK_TRACE_MAX_HZ) {
		LOG_ERROR("ST-LINK doesn't support SWO frequency higher than %u",
			  STLINK_TRACE_MAX_HZ);
		return ERROR_FAIL;
	}

        stlink_tcp_config_trace(h, enabled, pin_protocol,  port_size, trace_freq);

	if (!*trace_freq)
		*trace_freq = STLINK_TRACE_MAX_HZ;
	h->trace.source_hz = *trace_freq;

	return stlink_tcp_trace_enable(h);
}


static int stlink_tcp_open(struct hl_interface_param_s *param, void **fd)
{
	struct stlink_tcp_handle_s *h;
	int count;
        char buf[80];
        char buf_in[80];

	h = (struct stlink_tcp_handle_s *)malloc( sizeof (struct stlink_tcp_handle_s));

        if ( h) {
          h->socket = socket(AF_INET, SOCK_STREAM, 0);

          struct sockaddr_in serv;
          memset(&serv, 0, sizeof(struct sockaddr_in));
          serv.sin_family = AF_INET;
          serv.sin_port = htons(7184);
          serv.sin_addr.s_addr = inet_addr("127.0.0.1");

          LOG_DEBUG("socket : %x", h->socket);

          int flag = 1;
          int res;
          res = setsockopt(h->socket,
                     IPPROTO_TCP,			/* set option at TCP level */
                     TCP_NODELAY,			/* name of option */
                     (char *)&flag,			/* the cast is historical cruft */
                     sizeof(int));			/* length of option value */
         if (res == -1) 
            LOG_ERROR("Error setting socket opts: %s, TCP_NODELAY\n", strerror(errno));

          int a = 49152;
          res = setsockopt(h->socket,
                     SOL_SOCKET,			/* set option at TCP level */
                     SO_RCVBUF,	           		/* name of option */
                     (char *)&a,			/* the cast is historical cruft */
                     sizeof(int));			/* length of option value */

         if (res == -1) 
            LOG_ERROR("Error setting socket opts: %s, SO_RCVBUF\n", strerror(errno));

         res = setsockopt(h->socket,
                          SOL_SOCKET,			/* set option at TCP level */
                          SO_SNDBUF,	       		/* name of option */
                          (char *)&a,			/* the cast is historical cruft */
                          sizeof(int));			/* length of option value */
         
         if (res == -1) 
           LOG_ERROR("Error setting socket opts: %s, SO_SNDBUF\n", strerror(errno));

         LOG_DEBUG("prepare to connect socket : %x", h->socket);

         /* short timeout */        
         if (connect(h->socket, (const struct sockaddr *) &serv, sizeof(serv)) >= 0) {
           LOG_DEBUG("socket connected.");
           stlink_tcp_send_string(h, "register-client openocd ", buf);
           
           if (stlink_tcp_send_string(h, "get-nb-stlink", buf)) {
             sscanf( &buf[2], "%d", &count);
             LOG_DEBUG("%d card detected.", count);
             
             // if card choseen by app get it else card 0
             stlink_tcp_send_string(h, "get-stlink-chosen\n", buf);
	     LOG_DEBUG("get-stlink-chosen 0x%s", &buf[2]);	     
             
             unsigned int key;
	     /* skip the 1 space "1 xxxx"*/
             sscanf(&buf[2], "%x", &key);
             LOG_DEBUG("registred card %s, key %x", buf, key);
             if ( key > 0) {
               h->device_id = key;
             }
             else
               if (count >= 1 ) {                
                 if ( stlink_tcp_send_string(h, "get-stlink-descriptor 0", buf)) {
                   CMD_PARSER();
                   sscanf(string[1], "%x", &h->device_id);
                   LOG_DEBUG("%x usb_key.", h->device_id);
                   sscanf(string[2], "%x", (unsigned int *)(void *)&h->vid);
                   sscanf(string[3], "%x", (unsigned int *)(void *)&h->pid);
                   LOG_DEBUG("PID = 0x%x, VID = 0x%x", h->pid, h->vid);
                 }
               }

             LOG_DEBUG("open-device : 0x%x", h->device_id);     
             sprintf(buf_in, "open-device %x\n", h->device_id);
             if ( stlink_tcp_send_string(h, buf_in, buf)) {
               
		sscanf(&buf[2], "%d", (int *)&h->connect_id);
		LOG_DEBUG("connect_id %d",(int)h->connect_id);

		LOG_DEBUG("stlink_tcp_get_version");     
	       
		stlink_tcp_get_version(h);
               
		float v;
		stlink_tcp_check_voltage(h, &v);
               
		/* cast to void */
		*fd = (void *)h;
                              
		/* initialize the debug hardware , param->connect_under_reset*/
		LOG_DEBUG("param->connect_under_reset %d",(int)param->connect_under_reset);
		int err = stlink_tcp_init_mode(h, param->connect_under_reset);
		LOG_DEBUG("return %d", err);
		return ERROR_OK;
             } else
               LOG_DEBUG("open-device : return error");
	     

            } // get-nb-stlink
          } // connect
        } // malloc
        
        return ERROR_FAIL;
}

/** */
struct hl_layout_api_s stlink_tcp_layout_api = {
	/** */
	.open = stlink_tcp_open,
	/** */
	.close = stlink_tcp_close,
	/** */
	.idcode = stlink_tcp_idcode,
	/** */
	.state = stlink_tcp_state,
	/** */
	.reset = stlink_tcp_reset_h,
	/** */
	.assert_srst = stlink_tcp_assert_srst,
	/** */
	.run = stlink_tcp_run,
	/** */
	.halt = stlink_tcp_halt,
	/** */
	.step = stlink_tcp_step,
	/** */
	.read_regs = stlink_tcp_read_regs,
	/** */
	.read_reg = stlink_tcp_read_reg,
	/** */
	.write_reg = stlink_tcp_write_reg,
	/** */
	.read_mem = stlink_tcp_read_mem,
	/** */
	.write_mem = stlink_tcp_write_mem,
	/** */
        .write_debug_reg = stlink_tcp_write_debug_reg,
	/** */
	.override_target = stlink_tcp_override_target,
	/** ### */
	.speed = stlink_tcp_speed,
	/** */
	.config_trace = stlink_config_trace,
	/** */
	.poll_trace = stlink_tcp_trace_read_h,
};
