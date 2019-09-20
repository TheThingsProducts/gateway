// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#include "app.h"
#include "app_http_request.h"
#include "subsystem_controller.h"

static int8_t _pumpDNS(const char* hostname, IPV4_ADDR* ipv4Addr);

APP_HTTP_REQUEST_DATA appHTTPRequestData;

http_request request = {0};

uint32_t tcpTimeout   = 0;
bool     timeout      = false;
bool     timeout_wait = false;

uint32_t response_byte_count = 0;

void APP_HTTP_Request_Initialize()
{
    // request = newrequest;
    response_byte_count      = 0;
    request.total_bytes      = 0;
    appHTTPRequestData.state = APP_HTTP_REQUEST_START_CONNECTION;
}

int8_t APP_HTTP_Request_State()
{
    return appHTTPRequestData.state;
}

void APP_HTTP_Request_CloseIfNeeded(void)
{
    if(appHTTPRequestData.socket != INVALID_SOCKET)
    {
        SYS_PRINT("HTTP: Close active socket %d\r\n", appHTTPRequestData.socket);
        NET_PRES_SocketClose(appHTTPRequestData.socket);
        appHTTPRequestData.socket = INVALID_SOCKET;
    }
}

void APP_HTTP_Request_Reset()
{
    APP_HTTP_Request_CloseIfNeeded();
    appHTTPRequestData.state = APP_HTTP_REQUEST_START_CONNECTION;
}

void APP_HTTP_Request_Tasks(void)
{
    switch(appHTTPRequestData.state)
    {
        case APP_HTTP_REQUEST_START_CONNECTION:
        {
            APP_HTTP_Request_CloseIfNeeded(); // Abort any pending http request

            response_byte_count = 0;
            if(timeout)
            {
                timeout_wait = true;
                timeout      = false;
            }
            if(timeout_wait && (SYS_TMR_TickCountGet() - tcpTimeout < SYS_TMR_TickCounterFrequencyGet() * 1))
            {
                // tcpTimeout = SYS_TMR_TickCountGet();
                tcpTimeout   = 0;
                timeout_wait = false;
                timeout      = false;
            }

            TCPIP_DNS_RESULT result = 0;
            result                  = TCPIP_DNS_Resolve(request.host, TCPIP_DNS_TYPE_A);
            // SYS_PRINT("res:%d\r\n",result);
            if(result == TCPIP_DNS_RES_NAME_IS_IPADDRESS)
            {
                SYS_DEBUG(SYS_ERROR_INFO, "HTTP: DNS resolved\r\n");
                IPV4_ADDR addr;
                TCPIP_Helper_StringToIPAddress(request.host, &addr);
                appHTTPRequestData.socket =
                    NET_PRES_SocketOpen(0, NET_PRES_SKT_UNENCRYPTED_STREAM_CLIENT, IP_ADDRESS_TYPE_IPV4, request.port,
                                        (NET_PRES_ADDRESS*)&addr, NULL);
                if(appHTTPRequestData.socket == INVALID_SOCKET)
                {
                    SYS_DEBUG(SYS_ERROR_FATAL, "HTTP: Could not start connection\r\n");
                    appHTTPRequestData.state = APP_HTTP_REQUEST_ERROR;
                }
                else
                {
                    SYS_DEBUG(SYS_ERROR_INFO, "HTTP: Starting connection\r\n");
                    SYS_DEBUG(SYS_ERROR_DEBUG, "HTTP: - Host: %s\r\n", request.host);
                    SYS_DEBUG(SYS_ERROR_DEBUG, "HTTP: - Path: %s\r\n", request.path);
                    SYS_DEBUG(SYS_ERROR_DEBUG, "HTTP: - Port: %u\r\n", request.port);
                    tcpTimeout               = SYS_TMR_TickCountGet();
                    appHTTPRequestData.state = APP_HTTP_REQUEST_WAIT_FOR_CONNECTION;
                }
                break;
            }
            if(result < 0)
            {
                break;
            }

            appHTTPRequestData.state = APP_HTTP_REQUEST_WAIT_ON_DNS;
            tcpTimeout               = SYS_TMR_TickCountGet();
        }
        break;

        case APP_HTTP_REQUEST_WAIT_ON_DNS:
        {
            IPV4_ADDR addr;
            switch(_pumpDNS(request.host, &addr))
            {
                case -1:
                {
                    // Some sort of error, already reported
                    SYS_DEBUG(SYS_ERROR_INFO, "HTTP: DNS ERROR to error state\r\n");
                    APP_HTTP_Request_CloseIfNeeded();
                    appHTTPRequestData.state = APP_HTTP_REQUEST_ERROR;
                }
                break;
                case 0:
                {
                    // Still waiting
                    if(SYS_TMR_TickCountGet() - tcpTimeout >= SYS_TMR_TickCounterFrequencyGet() * 2)
                    {
                        tcpTimeout = SYS_TMR_TickCountGet();
                        timeout    = true;
                        APP_HTTP_Request_CloseIfNeeded();
                        appHTTPRequestData.state = APP_HTTP_REQUEST_ERROR;
                    }
                }
                break;
                case 1:
                {
                    appHTTPRequestData.socket =
                        NET_PRES_SocketOpen(0, NET_PRES_SKT_UNENCRYPTED_STREAM_CLIENT, IP_ADDRESS_TYPE_IPV4,
                                            request.port, (NET_PRES_ADDRESS*)&addr, NULL);
                    if(appHTTPRequestData.socket == INVALID_SOCKET)
                    {
                        SYS_DEBUG(SYS_ERROR_ERROR, "HTTP: Could not start connection\r\n");
                        APP_HTTP_Request_CloseIfNeeded();
                        appHTTPRequestData.state = APP_HTTP_REQUEST_ERROR;
                    }
                    else
                    {
                        SYS_DEBUG(SYS_ERROR_INFO, "HTTP: Starting connection\r\n");
                        SYS_DEBUG(SYS_ERROR_DEBUG, "HTTP: - Host: %s\r\n", request.host);
                        SYS_DEBUG(SYS_ERROR_DEBUG, "HTTP: - Path: %s\r\n", request.path);
                        SYS_DEBUG(SYS_ERROR_DEBUG, "HTTP: - Port: %u\r\n", request.port);
                        tcpTimeout               = SYS_TMR_TickCountGet();
                        appHTTPRequestData.state = APP_HTTP_REQUEST_WAIT_FOR_CONNECTION;
                    }
                }
                break;
            }
        }
        break;

        case APP_HTTP_REQUEST_WAIT_FOR_CONNECTION:
        {
            //  SYS_CONSOLE_MESSAGE("WAITING ON TCP SOCKET\r\n");
            if(!NET_PRES_SocketIsConnected(appHTTPRequestData.socket))
            {
                if(SYS_TMR_TickCountGet() - tcpTimeout >= SYS_TMR_TickCounterFrequencyGet() * 5)
                {
                    tcpTimeout = SYS_TMR_TickCountGet();
                    timeout    = true;
                    APP_HTTP_Request_CloseIfNeeded();
                    appHTTPRequestData.state = APP_HTTP_REQUEST_ERROR;
                }
                // SYS_CONSOLE_MESSAGE("Error: TCP/IP not connected\r\n");
                break;
            }

            /*
             * We increased TCPIP_TCP_CLOSE_WAIT_TIMEOUT to 2 seconds (was 200ms)
             * The HTTP requests are done with "Connection: close", to be able to reuse the socket within 2 seconds we
             * don't want to have graceful close (effectively it will be an abort).
             */
            TCP_OPTION_LINGER_DATA LData;
            NET_PRES_SocketOptionsGet(appHTTPRequestData.socket, TCP_OPTION_LINGER, &LData);
            LData.gracefulEnable = false;
            NET_PRES_SocketOptionsSet(appHTTPRequestData.socket, TCP_OPTION_LINGER, &LData);

            if(request.tls)
            {
                // mbedTLS requires RX and TX buffers to be 16384;
                ASSERT(NET_PRES_SocketOptionsSet(appHTTPRequestData.socket, TCP_OPTION_RX_BUFF, (void*)20000),
                       "setting RX buffer to 16k");
                ASSERT(NET_PRES_SocketOptionsSet(appHTTPRequestData.socket, TCP_OPTION_TX_BUFF, (void*)16384),
                       "setting TX buffer to 16k");

                SYS_DEBUG(SYS_ERROR_INFO, "HTTPS: Connection Opened: Starting TLS Negotiation\r\n");
                if(!NET_PRES_SocketEncryptSocket(appHTTPRequestData.socket))
                {
                    SYS_DEBUG(SYS_ERROR_INFO, "HTTP: TLS Create Connection Failed - Aborting\r\n");
                    APP_HTTP_Request_CloseIfNeeded();
                    appHTTPRequestData.state = APP_HTTP_REQUEST_ERROR;
                }
                else
                {
                    SYS_DEBUG(SYS_ERROR_INFO, "HTTP: Wait for TLS Connect\r\n");
                    appHTTPRequestData.state = APP_HTTP_REQUEST_WAIT_FOR_TLS_CONNECT;
                }
            }
            else
            {
                SYS_DEBUG(SYS_ERROR_INFO, "HTTP: Connection Opened: Starting sending request\r\n");
                appHTTPRequestData.state = APP_HTTP_REQUEST_SEND_REQUEST;
            }
            break;
        }

        case APP_HTTP_REQUEST_WAIT_FOR_TLS_CONNECT:
        {
            if(NET_PRES_SocketIsNegotiatingEncryption(appHTTPRequestData.socket))
            {
                break;
            }
            if(!NET_PRES_SocketIsSecure(appHTTPRequestData.socket))
            {
                SYS_DEBUG(SYS_ERROR_INFO, "HTTP: TLS Connection Negotiation Failed - Aborting\r\n");
                APP_HTTP_Request_CloseIfNeeded();
                appHTTPRequestData.state = APP_HTTP_REQUEST_ERROR;
                break;
            }
            SYS_DEBUG(SYS_ERROR_INFO, "HTTP: TLS Connection Opened: Starting Clear Text Communication\r\n");
            appHTTPRequestData.state = APP_HTTP_REQUEST_SEND_REQUEST;
            break;
        }

        case APP_HTTP_REQUEST_SEND_REQUEST:
        {
            NET_PRES_SocketWrite(appHTTPRequestData.socket, (uint8_t*)request.urlheaders, strlen(request.urlheaders));
            memset(request.response_buffer, 0, sizeof(request.response_buffer));

            if(request.bulk_request == 1)
            {
                appHTTPRequestData.state = APP_HTTP_REQUEST_WAIT_FOR_RESPONSE_BULK;
            }
            else
            {
                appHTTPRequestData.state = APP_HTTP_REQUEST_WAIT_FOR_RESPONSE;
            }
            break;
        }

        case APP_HTTP_REQUEST_WAIT_FOR_RESPONSE_BULK:
        {
            uint16_t available_bytes = 0;
            if((available_bytes = NET_PRES_SocketReadIsReady(appHTTPRequestData.socket)) == 0 &&
               NET_PRES_SocketIsConnected(appHTTPRequestData.socket))
                break;

            else if((available_bytes = NET_PRES_SocketReadIsReady(appHTTPRequestData.socket)) > 0)
            {
                if(request.new_data_flag == 0)
                {
                    if(available_bytes > 1024)
                    {
                        available_bytes = 1024;
                    }
                    uint16_t actual_bytes = NET_PRES_SocketRead(appHTTPRequestData.socket,
                                                                (uint8_t*)request.response_buffer, available_bytes);
                    if(actual_bytes != available_bytes)
                    {
                        SYS_DEBUG(SYS_ERROR_WARNING, "HTTP: Actual read (%u) not equal to available (%u)\r\n",
                                  actual_bytes, available_bytes);
                    }
                    request.available_bytes = actual_bytes;
                    request.total_bytes += actual_bytes;
                    request.new_data_flag = 1;
                    SYS_DEBUG(SYS_ERROR_DEBUG, "HTTP: Got %u bytes, total %u\r\n", actual_bytes, request.total_bytes);
                }
            }

            else if(!NET_PRES_SocketIsConnected(appHTTPRequestData.socket))
            {
                SYS_DEBUG(SYS_ERROR_INFO, "HTTP: Connection Closed\r\n");
                appHTTPRequestData.state = APP_HTTP_REQUEST_HTTP_DONE;
                response_byte_count      = 0;
                break;
            }
            else if(NET_PRES_SocketWasReset(appHTTPRequestData.socket))
            {
                SYS_DEBUG(SYS_ERROR_INFO, "HTTP: Connection was reset\r\n");
                appHTTPRequestData.state = APP_HTTP_REQUEST_ERROR;
                break;
            }
            break;
        }

        case APP_HTTP_REQUEST_WAIT_FOR_RESPONSE:
        {
            uint16_t available_bytes = 0;
            if((available_bytes = NET_PRES_SocketReadIsReady(appHTTPRequestData.socket)) == 0 &&
               NET_PRES_SocketIsConnected(appHTTPRequestData.socket))
                break;
            else if((available_bytes = NET_PRES_SocketReadIsReady(appHTTPRequestData.socket)) > 0)
            {
                SYS_DEBUG(SYS_ERROR_INFO, "HTTP: Got %u bytes\r\n", available_bytes);
                NET_PRES_SocketRead(appHTTPRequestData.socket,
                                    (uint8_t*)&(request.response_buffer[response_byte_count]), available_bytes);
                response_byte_count += available_bytes;

                if(response_byte_count >= RESPONSE_BUFFER_SIZE)
                {
                    SYS_DEBUG(SYS_ERROR_ERROR, "HTTP: PANIC TOO LARGE RESPONSE\r\n");
                    APP_HTTP_Request_CloseIfNeeded();
                    appHTTPRequestData.state = APP_HTTP_REQUEST_ERROR;
                }
            }
            else if(!NET_PRES_SocketIsConnected(appHTTPRequestData.socket))
            {
                SYS_DEBUG(SYS_ERROR_INFO, "HTTP: Connection Closed\r\n");
                APP_HTTP_Request_CloseIfNeeded();
                appHTTPRequestData.state = APP_HTTP_REQUEST_HTTP_DONE;
                request.available_bytes  = response_byte_count;
                response_byte_count      = 0;
                break;
            }
            else if(NET_PRES_SocketWasReset(appHTTPRequestData.socket))
            {
                SYS_DEBUG(SYS_ERROR_INFO, "HTTP: Connection was reset\r\n");
                APP_HTTP_Request_CloseIfNeeded();
                appHTTPRequestData.state = APP_HTTP_REQUEST_ERROR;
                break;
            }
            break;
        }

        case APP_HTTP_REQUEST_HTTP_DONE:
        {
            SYS_DEBUG(SYS_ERROR_INFO, "HTTP: Request done\r\n");
            break;
        }

        case APP_HTTP_REQUEST_ERROR:
        {
            break;
        }

        /* The default state should never be executed. */
        default:
        {
            break;
        }
    }
}

int32_t APP_HTTP_ParseUrl(char* uri, char** host, char** path, uint16_t* port, bool* tls)
{
    char* pos = uri;

    if(strncmp(pos, "https", 5) == 0)
    {
        *tls  = true;
        *port = 443;
    }
    else if(strncmp(pos, "http", 4) == 0)
    {
        *tls  = false;
        *port = 80;
    }
    pos = strstr(uri, "//"); // Check to see if its a proper URL

    if(!pos)
    {
        return -1;
    }
    *host = pos + 2; // This is where the host should start

    pos = strchr(*host, ':');

    if(!pos)
    {
        pos = strchr(*host, '/');
        if(!pos)
        {
            *path = NULL;
        }
        else
        {
            *pos  = '\0';
            *path = pos + 1;
        }
    }
    else
    {
        *pos        = '\0';
        char* portc = pos + 1;

        pos = strchr(portc, '/');
        if(!pos)
        {
            *path = NULL;
        }
        else
        {
            *pos  = '\0';
            *path = pos + 1;
        }
        *port = atoi(portc);
    }
    return 0;
}

static int8_t _pumpDNS(const char* hostname, IPV4_ADDR* ipv4Addr)
{
    IP_MULTI_ADDRESS mAddr;

    TCPIP_DNS_RESULT result = TCPIP_DNS_IsResolved(hostname, &mAddr, IP_ADDRESS_TYPE_IPV4);
    switch(result)
    {
        case TCPIP_DNS_RES_OK:
        {
            // We now have an IPv4 Address
            // Open a socket
            ipv4Addr->Val = mAddr.v4Add.Val;
            return 1;
        }
        case TCPIP_DNS_RES_PENDING:
            return 0;
        case TCPIP_DNS_RES_SERVER_TMO:
        case TCPIP_DNS_RES_NO_IP_ENTRY:
        default:
            SYS_DEBUG(SYS_ERROR_FATAL, "HTTP: TCPIP_DNS_IsResolved returned failure code %d\r\n", result);
            // SYS_CONSOLE_PRINT("status: %i\r\n", result);
            return -1;
    }
    // Should not be here!
    return -1;
}
