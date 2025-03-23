/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#include "TCP_Server.h"
#include "string.h"
#include "lwip/opt.h"

#if LWIP_NETCONN

#include "lwip/sys.h"
#include "lwip/api.h"

static void tcpecho_thread(void *arg)
{
	struct netconn *conn, *newconn;
	err_t err;
	struct netbuf *buf;
	//char msg[100];
	u16_t len;
	u8_t *row_p;
	u8_t row;
	
	LWIP_UNUSED_ARG(arg);

	/* Create a new connection identifier. */
	conn = netconn_new(NETCONN_TCP);
	if(conn == NULL){
		return;
	}
	/* Bind connection to well known port number 7. */
	err = netconn_bind(conn, IP_ADDR_ANY, 7);
	
	if(err == ERR_OK){
		/* Tell connection to go into listening mode. */
		netconn_listen(conn);
	
		while(1){
			/* Grab new connection. */
			err = netconn_accept(conn, &newconn);
			/* Process the new connection. */
			if(err == ERR_OK){
				while(netconn_recv(newconn, &buf) == ERR_OK){
					do {
						//memset(msg, '\0', 100);  // clear the buffer
						//strncpy(msg, buf->p->payload, buf->p->len);   // get the message from the client
						//strcat(msg, "3.");
						//len = (buf->p->len) + 2;
						row_p = buf->p->payload;
						row = *row_p;
						if(row >= 240){	row = 239;	}	// row index is 0-239 in qvga format
						len = 640;
						netconn_write(newconn, OV7670_Get_Pix_Array_By_Row(row), len, NETCONN_COPY);
					} while (netbuf_next(buf) > 0);
					netbuf_delete(buf);
				}
				/* Close connection and discard connection identifier. */
				netconn_close(newconn);
				netconn_delete(newconn);
			}
		}
	} else {
		netconn_delete(conn);
	}
}

void TCP_Server_Init(void)
{
	sys_thread_new("tcpecho_thread", tcpecho_thread, NULL, (configMINIMAL_STACK_SIZE*8), osPriorityAboveNormal);
}

#endif /* LWIP_NETCONN */
