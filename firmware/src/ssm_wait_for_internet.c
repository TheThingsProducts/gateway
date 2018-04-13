// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#include "app.h"
#include "app_mqtt.h"
#include "subsystem_controller.h"
#include "tcpip/sntp.h"

/* Wait for internet sequence states */
typedef enum
{
    STATE_WAIT_FOR_NETWORK,
    STATE_AP_ONLY,
    STATE_SETTLE,
    STATE_PING_PROBE,
    STATE_WAIT_FOR_NTP,
    STATE_DONE
} STATE_t;

#define WIFI_CONNECT_TIMEOUT 10 // seconds before switching to AP mode
#define PING_TIMEOUT 60          // seconds before retry a ping
#define PING_RETRIES 2          // retries before giving up
#define WIFI_RETRY_TIMEOUT 120  // seconds before retrying INFRA mode again if WiFi data valid
#define NTP_TIMEOUT 20          // seconds trying to receive NTP before waiting for network again

static STATE_t  _state;
static uint32_t settleStartTick      = 0;
static uint32_t pingStartTick        = 0;
static uint32_t wifiRetryStartTick   = 0;
static bool     firstTimeAPReconnectTimeout;
static uint32_t ntpStartTick        = 0;
static bool     ntpEverSynchronized = false;

static bool        ping_probe_sent           = false;
static bool        ping_probe_reply_received = false;
static uint8_t     ping_retry                = 0;
static ICMP_HANDLE pingHandle;

extern APP_GW_WIFI_DATA appWifiData;

static void PingCallbackFunction(TCPIP_NET_HANDLE hNetIf, IPV4_ADDR* remoteIP, void* data);
static void _changeState(STATE_t newState);

void SSMWaitForInternet_Initialize(void)
{
    //    pingHandle = NULL;
    pingHandle = TCPIP_ICMP_CallbackRegister(&PingCallbackFunction);
    if(pingHandle == NULL)
    {
        FATAL("Failed to install ICMP callback\r\n");
    }
}

bool SSMWaitForInternet_HasInternet(void)
{
    return _state == STATE_DONE;
}

bool SSMWaitForInternet_IsAPOnly(void)
{
    return _state == STATE_AP_ONLY;
}

void SSMWaitForInternet_Enter(void)
{
    _changeState(STATE_WAIT_FOR_NETWORK);
}

/**
 * Start the next readout, depending on availability
 */
static void _changeState(STATE_t newState)
{
    SYS_PRINT("INET: State change to %d\r\n", newState);

    // onEnter
    switch(newState)
    {
        case STATE_WAIT_FOR_NETWORK:
            if(appWifiData.valid)
            {
                APP_WIFI_INFRA_MODE();
            }
            break;

        case STATE_AP_ONLY:
            wifiRetryStartTick = SYS_TMR_TickCountGet();
            APP_WIFI_AP_MODE();
            firstTimeAPReconnectTimeout = true;
            break;

        case STATE_SETTLE:
            SYS_PRINT("INET: Connected to a network, waiting for DHCP lease, checking validity with ping\r\n");
            settleStartTick = SYS_TMR_TickCountGet();
            break;

        case STATE_PING_PROBE:
            ping_probe_reply_received = false;
            ping_probe_sent           = false;
            ping_retry                = 0;
            pingStartTick             = SYS_TMR_TickCountGet();
            break;

        case STATE_WAIT_FOR_NTP:
            ntpStartTick = SYS_TMR_TickCountGet();
            break;

        default:
            break;
    }
    _state = newState;
}

void SSMWaitForInternet_Tasks(void)
{
    switch(_state)
    {
        case STATE_WAIT_FOR_NETWORK:
            if(APP_ETH_Has_Link())
            {
                SYS_PRINT("INET: Gateway has Ethernet\r\n");
                _changeState(STATE_SETTLE);
            }
            else if(!appWifiData.valid)
            {
                SYS_PRINT("INET: No Ethernet and no WiFi config\r\n");
                _changeState(STATE_AP_ONLY);
            }
            if(APP_WIFI_Has_LinkINFRA())
            {
                SYS_PRINT("INET: Gateway has WiFi\r\n");
                _changeState(STATE_SETTLE);
            }
            break;

        case STATE_AP_ONLY:
            if(APP_ETH_Has_Link())
            {
                _changeState(STATE_WAIT_FOR_NETWORK);
            }
            else if(appWifiData.valid && ((SYS_TMR_TickCountGet() - wifiRetryStartTick) >=
                                          (SYS_TMR_TickCounterFrequencyGet() * WIFI_RETRY_TIMEOUT)))
            { // REVIEW: Use isElapsed kind of function
                if(APP_WIFI_Has_LinkAP())
                {
                    if(firstTimeAPReconnectTimeout)
                    {
                        SYS_PRINT("INET: Not trying to connect to WiFi router again because client is connected (after "
                                  "%d seconds)\r\n",
                                  (SYS_TMR_TickCountGet() - wifiRetryStartTick) / SYS_TMR_TickCounterFrequencyGet());
                        firstTimeAPReconnectTimeout = false;
                    }
                }
                else
                {
                    SYS_PRINT("INET: Trying to connect to WiFi router again (after %d seconds)\r\n",
                              (SYS_TMR_TickCountGet() - wifiRetryStartTick) / SYS_TMR_TickCounterFrequencyGet());
                    _changeState(STATE_WAIT_FOR_NETWORK);
                }
            }
            break;

        case STATE_SETTLE:
            if((SYS_TMR_TickCountGet() - settleStartTick) >= (SYS_TMR_TickCounterFrequencyGet() * 5))
            {
                _changeState(STATE_PING_PROBE);
            }
            break;

        case STATE_PING_PROBE:
            if(!ping_probe_sent)
            {
                IPV4_ADDR googledns = {0};
                googledns.v[0]      = 8;
                googledns.v[1]      = 8;
                googledns.v[2]      = 4;
                googledns.v[3]      = 4;

                ping_probe_reply_received = false;

                SYS_PRINT("INET: Ping probe\r\n");
                if(TCPIP_ICMP_EchoRequestSend(TCPIP_STACK_IndexToNet(WIFI_INTERFACE_NUM), &googledns, 0, 0x1234) !=
                   ICMP_ECHO_OK)
                {
                    SYS_PRINT("INET: Error sending probe on WiFi\r\n");
                }
                if(TCPIP_ICMP_EchoRequestSend(TCPIP_STACK_IndexToNet(ETH_INTERFACE_NUM), &googledns, 0, 0x1234) !=
                   ICMP_ECHO_OK)
                {
                    SYS_PRINT("INET: Error sending probe on Eth\r\n");
                }
                ping_probe_sent = true;
            }
            else if(ping_probe_reply_received)
            {
                SYS_PRINT("INET: Ping response from %s, set as default\r\n",
                          TCPIP_STACK_NetNameGet(TCPIP_STACK_NetDefaultGet()));
                if(ntpEverSynchronized)
                {
                    _changeState(STATE_DONE);
                }
                else
                {
                    _changeState(STATE_WAIT_FOR_NTP);
                }
                break;
            }
            else
            {
                if((SYS_TMR_TickCountGet() - pingStartTick) >=
                   (SYS_TMR_TickCounterFrequencyGet() * PING_TIMEOUT)) // REVIEW: Use isElapsed kind of function
                {
                    SYS_PRINT("INET: Ping Timeout of :%d seconds\r\n", PING_TIMEOUT);
                    pingStartTick             = SYS_TMR_TickCountGet();
                    ping_probe_sent           = false;
                    ping_probe_reply_received = false;
                    if(ping_retry >= PING_RETRIES)
                    {
                        _changeState(STATE_WAIT_FOR_NETWORK);
                    }
                    ping_retry++;
                }
            }
            break;

        case STATE_WAIT_FOR_NTP:
        {
            uint32_t lastUpdate = 0;
            TCPIP_SNTP_TimeStampGet(NULL, &lastUpdate);
            if(lastUpdate != 0)
            { // If at least once NTP succeeded
                ntpEverSynchronized = true;
                _changeState(STATE_DONE);
            }
            else
            {
                if((SYS_TMR_TickCountGet() - ntpStartTick) >= (SYS_TMR_TickCounterFrequencyGet() * NTP_TIMEOUT))
                { // REVIEW: Use isElapsed kind of function
                    SYS_PRINT("INET: Not received any NTP response. Wait for network again (after %d seconds).\r\n",
                              (SYS_TMR_TickCountGet() - ntpStartTick) / SYS_TMR_TickCounterFrequencyGet());
                    _changeState(STATE_WAIT_FOR_NETWORK);
                }
                else if(TCPIP_SNTP_ConnectionInitiate() == SNTP_RES_OK)
                {
                    SYS_PRINT("INET: Initiated NTP request.\r\n");
                }
            }
            break;
        }

        case STATE_DONE:
            break;
    }
}

static void PingCallbackFunction(TCPIP_NET_HANDLE hNetIf, IPV4_ADDR* remoteIP, void* data)
{
    // process request from interface hNetIf and remoteIP address
    // uint16_t* pReply = (uint16_t*)data;
    // uint16_t myRecvId = *pReply;
    // uint16_t myRecvSequenceNumber = *(pReply + 1);

    if(!ping_probe_reply_received)
    {
        // check that the sequence number, ID and IP address match, etc.
        ping_probe_reply_received = true;

        // set as default
        TCPIP_STACK_NetDefaultSet(hNetIf);
    }
}
