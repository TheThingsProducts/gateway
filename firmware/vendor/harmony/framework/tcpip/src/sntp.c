/*******************************************************************************
  Simple Network Time Protocol (SNTP) Client Version 3

  Summary:
    Module for Microchip TCP/IP Stack

  Description:
    -Locates an NTP Server from public site using DNS
    -Requests UTC time using SNTP and updates SNTPTime structure
     periodically, according to TCPIP_NTP_QUERY_INTERVAL value
    -Reference: RFC 1305
*******************************************************************************/

/*******************************************************************************
File Name:  SNTP.c
Copyright © 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_SNTP

#include "tcpip/src/tcpip_private.h"

#if defined(TCPIP_STACK_USE_SNTP_CLIENT)


// Defines the structure of an NTP packet
typedef struct
{
    struct
    {
        uint8_t mode            : 3;  // NTP mode
        uint8_t versionNumber   : 3;  // SNTP version number
        uint8_t leapIndicator   : 2;  // Leap second indicator
    } flags;                          // Flags for the packet

    uint8_t stratum;                  // Stratum level of local clock
    int8_t poll;                      // Poll interval
    int8_t precision;                 // Precision (seconds to nearest power of 2)
    uint32_t root_delay;              // Root delay between local machine and server
    uint32_t root_dispersion;         // Root dispersion (maximum error)
    uint32_t ref_identifier;          // Reference clock identifier
    uint32_t ref_ts_secs;             // Reference timestamp (in seconds)
    uint32_t ref_ts_fraq;             // Reference timestamp (fractions)
    uint32_t orig_ts_secs;            // Origination timestamp (in seconds)
    uint32_t orig_ts_fraq;            // Origination timestamp (fractions)
    uint32_t recv_ts_secs;            // Time at which request arrived at sender (seconds)
    uint32_t recv_ts_fraq;            // Time at which request arrived at sender (fractions)
    uint32_t tx_ts_secs;              // Time at which request left sender (seconds)
    uint32_t tx_ts_fraq;              // Time at which request left sender (fractions)

} NTP_PACKET;

// Seconds value obtained by last update
static uint32_t dwSNTPSeconds = 0;

// Tick count of last update
static uint32_t dwLastUpdateTick = 0;

static TCPIP_NET_IF*  pSntpIf = 0;    // we use only one interface for SNTP (for now at least)
static TCPIP_NET_IF*  pSntpDefIf = 0;    // default SNTP interface

static UDP_SOCKET       sntpSocket = INVALID_UDP_SOCKET;    // UDP socket we use

static tcpipSignalHandle sntpSignalHandle = 0;
static int              sntpInitCount = 0;

static char             sntpServerName[TCPIP_NTP_SERVER_MAX_LENGTH + 1];

static uint32_t         sntp_reply_timeout;
static uint32_t         sntp_tstamp_timeout;
static uint32_t         sntp_query_interval;
static uint32_t         sntp_error_interval;

typedef enum
{
    SM_INIT = 0,
    SM_HOME,
    SM_WAIT_DNS,
    SM_DNS_RESOLVED,
    SM_UDP_SEND,
    SM_UDP_RECV,
    SM_SHORT_WAIT,
    SM_WAIT,
    SM_END,

}TCPIP_SNTP_STATE;


static TCPIP_SNTP_STATE     _sntpState = SM_INIT;

// the server address
static IP_MULTI_ADDRESS  ntpServerIP;
static IP_ADDRESS_TYPE   ntpConnection;
static union
{
    struct
    {
        uint32_t    tStampSeconds;
        uint32_t    tStampFraction;
    };
    uint64_t    llStamp;
}ntpTimeStamp;       // last valid time stamp

static uint32_t         ntpLastStampTick;   // time of the last time stamp
static TCPIP_SNTP_RESULT      ntpLastError;

static void _setState(TCPIP_SNTP_STATE newState);
// local prototypes
#if (TCPIP_STACK_DOWN_OPERATION != 0)
static void TCPIP_SNTP_Cleanup(void);
#else
#define TCPIP_SNTP_Cleanup()
#endif  // (TCPIP_STACK_DOWN_OPERATION != 0)

static bool TCPIP_SNTP_ProcessPkt(void);

static void TCPIP_SNTP_Process(void);

static void     TCPIP_SNTP_SocketRxSignalHandler(UDP_SOCKET hUDP, TCPIP_NET_HANDLE hNet, TCPIP_UDP_SIGNAL_TYPE sigType, const void* param);

static __inline__ void __attribute__((always_inline)) TCPIP_SNTP_SetIdleState(TCPIP_SNTP_STATE newState)
{
    pSntpIf = 0;
    ntpTimeStamp.llStamp = 0;
    ntpLastStampTick = 0;
    _setState(newState);
}


// returns true if the module is idle
// and a new connection can be started
static bool _SntpCanStart(void)
{
    return (_sntpState == SM_SHORT_WAIT || _sntpState == SM_WAIT);
}

// returns true if the module is idle
// and SNTP parameters changed
static bool _SntpCanChangeParams(void)
{
    return (_sntpState == SM_HOME || _sntpState == SM_SHORT_WAIT || _sntpState == SM_WAIT || _sntpState == SM_END);
}


bool TCPIP_SNTP_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const TCPIP_SNTP_MODULE_CONFIG* pSNTPConfig)
{

    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        if (_sntpState != SM_INIT) { // only set to home when state machine already exited the init state
            TCPIP_SNTP_SetIdleState(SM_HOME);
        }
        return true;
    }

    // stack init
    while(sntpInitCount == 0)
    {   // first time we run
        if(pSNTPConfig == 0)
        {
            return false;
        }

        strncpy(sntpServerName, pSNTPConfig->ntp_server, sizeof(sntpServerName) - 1);
        sntpServerName[sizeof(sntpServerName) - 1] = 0;
        ntpConnection = pSNTPConfig->ntp_connection_type;
        sntp_reply_timeout = pSNTPConfig->ntp_reply_timeout;
        sntp_tstamp_timeout = pSNTPConfig->ntp_stamp_timeout;
        sntp_query_interval = pSNTPConfig->ntp_success_interval;
        sntp_error_interval = pSNTPConfig->ntp_error_interval;

        pSntpDefIf = (TCPIP_NET_IF*)TCPIP_STACK_NetHandleGet(pSNTPConfig->ntp_interface);

        sntpSocket = TCPIP_UDP_ClientOpen(ntpConnection, TCPIP_NTP_SERVER_REMOTE_PORT, 0);
        if(sntpSocket == INVALID_UDP_SOCKET)
        {
            return false;
        }
        // receiving from multiple SNTP servers
        TCPIP_UDP_OptionsSet(sntpSocket, UDP_OPTION_STRICT_ADDRESS, (void*)false);
        TCPIP_UDP_SignalHandlerRegister(sntpSocket, TCPIP_UDP_SIGNAL_RX_DATA, TCPIP_SNTP_SocketRxSignalHandler, 0);

        // create the SNTP timer
        sntpSignalHandle =_TCPIPStackSignalHandlerRegister(TCPIP_THIS_MODULE_ID, TCPIP_SNTP_Task, TCPIP_NTP_TASK_TICK_RATE); 
        if(sntpSignalHandle == 0)
        {   // cannot create the SNTP timer
            TCPIP_SNTP_Cleanup();
            return false;
        }


        break;
    }

    // Reset per interface state machine and flags to default values
    TCPIP_SNTP_SetIdleState(SM_INIT);

    ntpLastError = SNTP_RES_NTP_SERVER_TMO; 

    sntpInitCount++;
    return true;

}


#if (TCPIP_STACK_DOWN_OPERATION != 0)
void TCPIP_SNTP_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{

    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_DOWN)
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down

    if(sntpInitCount > 0)
    {   // we're up and running
        // interface is going down one way or another
        if(stackCtrl->pNetIf == pSntpIf)
        {   // this interface is going away/re-initialized, etc
            TCPIP_SNTP_SetIdleState(SM_END);
        }

        if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
        {   // whole stack is going down
            if(--sntpInitCount == 0)
            {   // all closed
                // release resources
                TCPIP_SNTP_Cleanup();
            }
        }
    }

}

static void TCPIP_SNTP_Cleanup(void)
{
    if(sntpSocket != INVALID_UDP_SOCKET)
    {
        TCPIP_UDP_Close(sntpSocket);
        sntpSocket = INVALID_UDP_SOCKET;
    }

    if(sntpSignalHandle)
    {
        _TCPIPStackSignalHandlerDeregister(sntpSignalHandle);
        sntpSignalHandle = 0;
    }

}
#endif  // (TCPIP_STACK_DOWN_OPERATION != 0)

void TCPIP_SNTP_Task(void)
{
    TCPIP_MODULE_SIGNAL sigPend;

    sigPend = _TCPIPStackModuleSignalGet(TCPIP_THIS_MODULE_ID, TCPIP_MODULE_SIGNAL_MASK_ALL);

    if(sigPend != 0)
    { // TMO or RX occurred
        TCPIP_SNTP_Process();
    }

}

// send a signal to the SNTP module that data is available
// no manager alert needed since this normally results as a higher layer (UDP) signal
static void TCPIP_SNTP_SocketRxSignalHandler(UDP_SOCKET hUDP, TCPIP_NET_HANDLE hNet, TCPIP_UDP_SIGNAL_TYPE sigType, const void* param)
{
    if(sigType == TCPIP_UDP_SIGNAL_RX_DATA)
    {
        _TCPIPStackModuleSignalRequest(TCPIP_THIS_MODULE_ID, TCPIP_MODULE_SIGNAL_RX_PENDING, true); 
    }
}



static void TCPIP_SNTP_Process(void)
{
    NTP_PACKET          pkt;
    TCPIP_DNS_RESULT    dnsRes;
    static uint32_t     SNTPTimer;
    TCPIP_NET_IF*       pNetIf;
    bool                dataAvlbl;


    switch(_sntpState)
    {
        case SM_INIT:
            // perform delayed initialization
            // convert seconds to sys ticks
            sntp_reply_timeout *= SYS_TMR_TickCounterFrequencyGet();
            sntp_tstamp_timeout *= SYS_TMR_TickCounterFrequencyGet();
            sntp_query_interval *= SYS_TMR_TickCounterFrequencyGet();
            sntp_error_interval *= SYS_TMR_TickCounterFrequencyGet();
            _setState(SM_HOME);
            break;

        case SM_HOME:
            pNetIf = _TCPIPStackAnyNetLinked(false);
            if(pNetIf == 0 || _TCPIPStackIsConfig(pNetIf) != 0)
            {   // not yet up and running
                break;
            }

#if defined (TCPIP_STACK_USE_IPV6)           
            if(ntpConnection == IP_ADDRESS_TYPE_IPV6)
            {
                if(TCPIP_Helper_StringToIPv6Address (sntpServerName, &ntpServerIP.v6Add))
                {   // IPv6 address provided
                    _setState(SM_DNS_RESOLVED;
                    break;
                }
            }
#endif  // defined (TCPIP_STACK_USE_IPV6)
          
#if defined (TCPIP_STACK_USE_IPV4)
            if(ntpConnection == IP_ADDRESS_TYPE_IPV4)
            {
                if(TCPIP_Helper_StringToIPAddress(sntpServerName, &ntpServerIP.v4Add))
                {   // IPv4 address provided
                    _setState(SM_DNS_RESOLVED);
                    break;
                }
            }
#endif  // defined (TCPIP_STACK_USE_IPV4)

            TCPIP_DNS_Resolve(sntpServerName, ntpConnection == IP_ADDRESS_TYPE_IPV6 ? TCPIP_DNS_TYPE_AAAA : TCPIP_DNS_TYPE_A);
            _setState(SM_WAIT_DNS);
            break;

        case SM_WAIT_DNS:
            dnsRes = TCPIP_DNS_IsResolved(sntpServerName, &ntpServerIP, ntpConnection);
            if(dnsRes == TCPIP_DNS_RES_PENDING)
            {   // ongoing operation;
                break;
            }
            else if(dnsRes < 0)
            {   // some DNS error occurred; retry after waiting a while
                SNTPTimer = SYS_TMR_TickCountGet();
                _setState(SM_SHORT_WAIT);
                ntpLastError = SNTP_RES_NTP_DNS_ERR;
            }
            else
            {
                _setState(SM_DNS_RESOLVED);
            }
            break;

        case SM_DNS_RESOLVED:
            // select a running interface
            pSntpIf = pSntpDefIf;
            if(!TCPIP_STACK_NetworkIsLinked(pSntpIf))
            {
                pSntpIf = _TCPIPStackAnyNetLinked(true);
            }

            if(pSntpIf == 0)
            {   // wait some more
                ntpLastError = SNTP_RES_NTP_IF_ERR; 
                break;
            }
                
            TCPIP_UDP_DestinationIPAddressSet(sntpSocket, ntpConnection, &ntpServerIP);
            TCPIP_UDP_SocketNetSet(sntpSocket, pSntpIf);
            _setState(SM_UDP_SEND);
            SNTPTimer = SYS_TMR_TickCountGet();
            break;

        case SM_UDP_SEND:
            // Open up the sending UDP socket
            // Make certain the socket can be written to
            if(!TCPIP_UDP_TxPutIsReady(sntpSocket, sizeof(pkt)))
            {   // Wait no more than 1 sec
                if((SYS_TMR_TickCountGet() - SNTPTimer > 1*SYS_TMR_TickCounterFrequencyGet()))
                {
                    _setState(SM_DNS_RESOLVED);
                    ntpLastError = SNTP_RES_SKT_ERR; 
                    break;
                }
            }

            // Success
            // Transmit a time request packet
            memset(&pkt, 0, sizeof(pkt));
            pkt.flags.versionNumber = TCPIP_NTP_VERSION;
            pkt.flags.mode = 3;             // NTP Client
            pkt.orig_ts_secs = TCPIP_Helper_htonl(TCPIP_NTP_EPOCH);
            // enable packets RX
            TCPIP_UDP_OptionsSet(sntpSocket, UDP_OPTION_RX_QUEUE_LIMIT, (void*)TCPIP_NTP_RX_QUEUE_LIMIT);
            TCPIP_UDP_ArrayPut(sntpSocket, (uint8_t*) &pkt, sizeof(pkt));
            TCPIP_UDP_Flush(sntpSocket);

            SNTPTimer = SYS_TMR_TickCountGet();
            _setState(SM_UDP_RECV);
            break;

        case SM_UDP_RECV:
            // Look for a response time packet
            if(!TCPIP_UDP_GetIsReady(sntpSocket))
            {
                if((SYS_TMR_TickCountGet()) - SNTPTimer <= sntp_reply_timeout )
                {   // no timeout yet
                    break;
                }
            }

            // either we have data or timeout
            // set the error state in case the process is not successful
            _setState(SM_SHORT_WAIT);
            dataAvlbl = false;

            // consume all available data
            while(TCPIP_UDP_GetIsReady(sntpSocket))
            {
                dataAvlbl = true;
                if(TCPIP_SNTP_ProcessPkt())
                {   // successful SNTP packet
                    _setState(SM_WAIT);
                    break;
                }
            }

            if(!dataAvlbl)
            {
                ntpLastError = SNTP_RES_NTP_SERVER_TMO; 
            }

            // disable RX of further packets
            TCPIP_UDP_OptionsSet(sntpSocket, UDP_OPTION_RX_QUEUE_LIMIT, (void*)0);
            // flush any pending data
            TCPIP_UDP_Disconnect(sntpSocket, true);

            SNTPTimer = SYS_TMR_TickCountGet();
            break;

        case SM_SHORT_WAIT:
            // Attempt to requery the NTP server after a specified timeout
            if(SYS_TMR_TickCountGet() - SNTPTimer > sntp_error_interval)
            {
                _setState(SM_HOME);
            }
            break;

        case SM_WAIT:
            // Requery the NTP server after a specified timeout
            if(SYS_TMR_TickCountGet() - SNTPTimer > sntp_query_interval)
            {
                _setState(SM_HOME);
            }

            break;

        default:
            break;
    }

}

// returns true if successful packet
// and current time updated
static bool TCPIP_SNTP_ProcessPkt(void)
{
    NTP_PACKET          pkt;
    uint16_t            w;


    // Get the response time packet
    w = TCPIP_UDP_ArrayGet(sntpSocket, (uint8_t*) &pkt, sizeof(pkt));

    // sanity packet check
    if(w != sizeof(pkt) || pkt.flags.versionNumber != TCPIP_NTP_VERSION )
    {
        ntpLastError = SNTP_RES_NTP_VERSION_ERR; 
        return false;
    }
    if((pkt.tx_ts_secs == 0 && pkt.tx_ts_fraq == 0))
    {
        ntpLastError = SNTP_RES_NTP_TSTAMP_ERR; 
        return false;
    }
    if(pkt.stratum == 0 )
    {
        ntpLastError = SNTP_RES_NTP_KOD_ERR; 
        return false;
    }
    if(pkt.stratum >= TCPIP_NTP_MAX_STRATUM || pkt.flags.leapIndicator == 3 )
    {
        ntpLastError = SNTP_RES_NTP_SYNC_ERR; 
        return false;
    }

    // success
    // get the last timestamp
    ntpTimeStamp.tStampSeconds = pkt.tx_ts_secs;
    ntpTimeStamp.tStampFraction = pkt.tx_ts_fraq;
    ntpLastStampTick = SYS_TMR_TickCountGet();


    // Set out local time to match the returned time
    dwLastUpdateTick = ntpLastStampTick;
    dwSNTPSeconds = TCPIP_Helper_ntohl(pkt.tx_ts_secs) - TCPIP_NTP_EPOCH;
    // Do rounding.  If the partial seconds is > 0.5 then add 1 to the seconds count.
    if(((uint8_t*)&pkt.tx_ts_fraq)[0] & 0x80)
    {
        dwSNTPSeconds++;
    }

    return true;
}


uint32_t TCPIP_SNTP_UTCSecondsGet(void)
{

    uint32_t dwTickDelta;
    uint32_t dwTick;

    // Update the dwSNTPSeconds variable with the number of seconds
    // that has elapsed
    dwTick = SYS_TMR_TickCountGet();
    dwTickDelta = dwTick - dwLastUpdateTick;
    while(dwTickDelta > SYS_TMR_TickCounterFrequencyGet())
    {
        dwSNTPSeconds++;
        dwTickDelta -= SYS_TMR_TickCounterFrequencyGet();
    }


    // Save the tick and residual fractional seconds for the next call
    dwLastUpdateTick = dwTick - dwTickDelta;

    return dwSNTPSeconds;

}

TCPIP_SNTP_RESULT TCPIP_SNTP_ConnectionParamSet(TCPIP_NET_HANDLE netH, IP_ADDRESS_TYPE ntpConnType, const char* ntpServer)
{
    if(!_SntpCanChangeParams())
    {
        return SNTP_RES_BUSY;
    }

    if(netH)
    {
        pSntpDefIf = _TCPIPStackHandleToNet(netH);
    }

    if(ntpConnType != IP_ADDRESS_TYPE_ANY)
    {
        ntpConnection = ntpConnType;
    }

    if(ntpServer)
    {
        strncpy(sntpServerName, ntpServer, sizeof(sntpServerName) - 1);
        sntpServerName[sizeof(sntpServerName) - 1] = 0;
    }

    return SNTP_RES_OK;
}


TCPIP_SNTP_RESULT TCPIP_SNTP_ConnectionInitiate(void)
{
    if(!_SntpCanStart())
    {
        return SNTP_RES_PROGRESS;
    }

    _setState(SM_HOME);
    return SNTP_RES_OK;
}

TCPIP_SNTP_RESULT TCPIP_SNTP_TimeStampGet(uint64_t* pTStamp, uint32_t* pLastUpdate)
{
    TCPIP_SNTP_RESULT res;

    if(ntpTimeStamp.llStamp == 0 || ntpLastStampTick == 0 || (SYS_TMR_TickCountGet() - ntpLastStampTick >  sntp_tstamp_timeout))
    {
        res = SNTP_RES_TSTAMP_STALE;
    }
    else
    {
        res = SNTP_RES_OK;
    }

    if(pTStamp)
    {
        *pTStamp = ntpTimeStamp.llStamp;
    }
    if(pLastUpdate)
    {
        *pLastUpdate = ntpLastStampTick;
    }

    return res;

}

TCPIP_SNTP_RESULT TCPIP_SNTP_LastErrorGet(void)
{
    TCPIP_SNTP_RESULT res = ntpLastError;
    ntpLastError = SNTP_RES_OK;
    return res; 
}

static void _setState(TCPIP_SNTP_STATE newState)
{
    SYS_PRINT("SNTP: State change from %d to %d\r\n", _sntpState, newState);
    _sntpState = newState;
}


#endif  //if defined(TCPIP_STACK_USE_SNTP_CLIENT)
