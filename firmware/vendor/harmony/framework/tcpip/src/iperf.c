/*******************************************************************************
  TCP/IP iperf implementation

  Summary:
    Runs iperf client and server
    
  Description:
    - Implements iperf benchmarking
*******************************************************************************/

/*******************************************************************************
File Name: iperf.c 
Copyright ?2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND,
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

#include <string.h> 

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_IPERF

#include "tcpip/src/tcpip_private.h"


#if defined(TCPIP_STACK_USE_IPERF)


//****************************************************************************
// CONSTANTS (Defines and enums)
//****************************************************************************

#define UDP_FIN_RETRANSMIT_COUNT		10u     // iperf retransmits 10 times the last UDP packet,
#define UDP_FIN_RETRANSMIT_PERIOD       10      // at 10ms apart.
#define TIMING_ERROR_MARGIN              2      // Account for msec tick uncertainty.


// TCP Maximum Segment Size - MSS;
#define IPERF_TCP_MSS  TCPIP_TCP_MAX_SEG_SIZE_TX


typedef enum {
    UDP_PROTOCOL = 1,
    TCP_PROTOCOL
} tIperfProto;

enum {
    IPERF_STANDBY_STATE=1,

    IPERF_RX_START_STATE,
    IPERF_UDP_RX_STATE,
    IPERF_UDP_RX_DRAIN_STATE,
    IPERF_UDP_RX_DONE_STATE,
    IPERF_TCP_RX_LISTEN_STATE,
    IPERF_TCP_RX_STATE,
    IPERF_TCP_RX_DONE_STATE,
    IPERF_RX_DONE_STATE,

    IPERF_TX_START_STATE,
    IPERF_TX_ARP_RESOLVE_STATE,

    IPERF_TCP_TX_OPEN_STATE,
    IPERF_TCP_TX_CONNECT_STATE,
    IPERF_TCP_TX_SEGMENT_STATE,
    IPERF_TCP_TX_DONE_STATE,

    IPERF_UDP_TX_OPEN_STATE,
    IPERF_UDP_TX_DATAGRAM_STATE,
    IPERF_UDP_TX_DONE_STATE
};

typedef enum
{
    INTERVAL_REPORT,
    SUBTOTAL_REPORT,
    SESSION_REPORT
} tIperfReport;

typedef enum
{
    IPERF_TX_WAIT   = 0,    // wait some more for transmitting
    IPERF_TX_OK,            // can go ahead and transmit
    IPERF_TX_FAIL,          // cannot get TX resources, abort
}tIperfTxResult;

//****************************************************************************
// LOCAL DATA TYPES                                                             
//****************************************************************************

/* tIperfState */
typedef struct
{
    uint32_t		mInterval;		// -i
    uint32_t		mAmount;		// -n
    uint32_t		mDuration;		// -t. Default = 10*TICK_SECOND msec
    uint32_t		mDatagramSize;	// -l
    tIperfProto	    mProtocol;		// -b or -u
    uint16_t		mServerPort;	// -p

    uint32_t		mTxRate;		// -b or
                                // -x: NONE-STANDARD IPERF OPTION. Max Tx bps rate for TCP.

    double			totalLen; // mTotalLen
    long 			pktId; 		// datagramID
    long			lastPktId; // lastDatagramID
    uint32_t		errorCount;
    uint32_t		outofOrder;

    TCP_SOCKET tcpServerSock;
    TCP_SOCKET tcpClientSock;

    UDP_SOCKET udpSock;

    TCP_SOCKET_INFO  remoteSide;
    TCPIP_MAC_ADDR   remoteMACAddr; // remote host MAC address
    uint16_t		 localPort;
    IPV4_ADDR        localAddr;     // local address/interface to use

    //struct sockaddr_in remoteAddr;
    // int 		remoteAddrlen;
    // tSocketAddr remoteAddr;

    // Calculated packet period, in msec, to reflect the target bit rate.
    uint32_t		mPktPeriod;

    uint32_t		startTime;
    uint32_t		stopTime;
    uint32_t		nextTxTime;
    //uint32_t		remoteStartTime;
    //uint32_t		remoteStopTime;

    uint32_t		pktCount;


    uint32_t		lastCheckPktCount;  // Check if pktCount changes within mInterval; or kIperfRxTimeOut.
    long			lastCheckPktId;
    uint32_t		lastCheckErrorCount;
    uint32_t		lastCheckTotalLen;
    uint32_t		lastCheckTime;

//	long		mPendingACK;		// number of outstanding TCP ACKs
//	uint8_t		mRetransmit;

    uint32_t      timer;
    uint16_t      remainingTxData;
    uint16_t      availUDPTxSpace;


    uint32_t    txWaitTick;

    uint32_t    txBuffSize;
    uint32_t    rxBuffSize;

    tcpipSignalHandle  signalHandle; 

    // console that invoked iperf
    SYS_CMD_DEVICE_NODE* pCmdIO;
    // interface to use
    TCPIP_NET_HANDLE pNetIf;
    // 
    uint8_t     nAttempts;
    uint8_t 	statusReported;
    uint8_t 	state;
    uint8_t		stopRequested;

    uint8_t     isLastTransmit;
    uint8_t     mServerMode;    // -s or -c
    uint16_t    mMSS;			// -M

} tIperfState;



//
// Data structure used by iperf protocol
//

#define HEADER_VERSION1 0x80000000

typedef struct
{
    long id;
    uint32_t tv_sec;
    uint32_t tv_usec;
} tIperfPktInfo; 	// In the original Iperf, this is the "UDP_datagram" structure.

// tUDP_datagram
typedef struct
{
    uint32_t flags;
    uint32_t total_len1;
    uint32_t total_len2;
    uint32_t stop_sec;
    uint32_t stop_usec;
    uint32_t error_cnt;
    uint32_t outorder_cnt;
    uint32_t datagrams;
    uint32_t jitter1;
    uint32_t jitter2;
} tServerHdr;

typedef struct
{
    uint32_t flags;
    uint32_t numThreads;
    uint32_t mPort;
    uint32_t bufferlen;
    uint32_t mWinBand;
    uint32_t mAmount;
} tClientHdr;

//****************************************************************************
// LOCAL GLOBALS                                                             
//****************************************************************************

#define MAX_BUFFER   (sizeof(tIperfPktInfo) + sizeof(tServerHdr))
uint8_t  g_bfr[ MAX_BUFFER ];


static tIperfState gIperfState;

static int    iperfInitCount = 0;      // iperf module initialization count

static int CommandIperfStart(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int CommandIperfStop(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int CommandIperfNetIf(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int CommandIperfSize(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
// Iperf command table
static const SYS_CMD_DESCRIPTOR    iperfCmdTbl[]=
{
    {"iperf",   	CommandIperfStart, 	":  <iperf> start cmd"},
    {"iperfk",  	CommandIperfStop,	": <iperfk> kill cmd"},
    {"iperfi",  	CommandIperfNetIf,	": <iperfi address>  interface cmd"},
    {"iperfs",  	CommandIperfSize,	": <iperfs tx/rx size> tx/rx size cmd"}
};


//****************************************************************************
// LOCAL Prototypes                                                             
//****************************************************************************
#if defined(TCPIP_STACK_USE_UDP)
static void StateMachineUdpTxDone(void);
static void StateMachineUdpTxDatagram(void);
static uint16_t UdpTxFillDatagram(void);
static void StateMachineUDPTxOpen(void);
static void StateMachineUdpRxDone(void);
static void StateMachineUdpRxDrain(void);
static void StateMachineUdpRx(void);
#endif  // defined(TCPIP_STACK_USE_UDP)

#if defined(TCPIP_STACK_USE_TCP)
static void StateMachineTcpTxDone(void);
static void StateMachineTcpTxSegment(void);
static void TcpTxFillSegment(void);
static void StateMachineTCPTxOpen(void);
static void StateMachineTcpRxDone(void);
static void StateMachineTcpRx(void);
static void StateMachineTcpListen(void);
static void StateMachineTCPTxConnect(void);
#endif  // defined(TCPIP_STACK_USE_TCP)

static void GenericTxDone(void);
static void GenericTxEnd(void);
static tIperfTxResult GenericTxStart(void);
static void GenericTxHeaderPreparation(uint8_t *pData, bool isTheLastTransmit);
static void StateMachineTxArpResolve(void);
static void StateMachineTxStart(void);
static void StateMachineRxDone(void);
static void StateMachineRxStart(void);
static void ReportBW_Jitter_Loss(tIperfReport reportType);
static void ascii_to_u32s(char *ptr, uint32_t *values, uint8_t count);
static void ResetIperfCounters(void);

static void IperfSetState(int newState);

static void TCPIP_IPERF_Process(void);

static void _IperfTCPRxSignalHandler(TCP_SOCKET hTCP, TCPIP_NET_HANDLE hNet, TCPIP_TCP_SIGNAL_TYPE sigType, const void* param);
static void _IperfUDPRxSignalHandler(TCP_SOCKET hTCP, TCPIP_NET_HANDLE hNet, TCPIP_UDP_SIGNAL_TYPE sigType, const void* param);

//****************************************************************************
// Implementation: public functions
//****************************************************************************

bool TCPIP_IPERF_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const void* initData)
{
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }

    // stack init

    if(iperfInitCount == 0)
    {   // first time we run
        memset( &gIperfState, 0, sizeof(tIperfState) );

        gIperfState.state = IPERF_STANDBY_STATE;
        gIperfState.stopRequested = false;

        gIperfState.tcpClientSock = INVALID_SOCKET;
        gIperfState.tcpServerSock = INVALID_SOCKET;
        gIperfState.udpSock = INVALID_SOCKET;
        gIperfState.txBuffSize = TCPIP_IPERF_TX_BUFFER_SIZE;
        gIperfState.rxBuffSize = TCPIP_IPERF_RX_BUFFER_SIZE;

        gIperfState.signalHandle =_TCPIPStackSignalHandlerRegister(TCPIP_THIS_MODULE_ID, TCPIP_IPERF_Task, 0);
        if(gIperfState.signalHandle == 0)
        {   // failed
            return false;
        }

        if(!SYS_CMD_ADDGRP(iperfCmdTbl, sizeof(iperfCmdTbl)/sizeof(*iperfCmdTbl), "iperf", ": iperf commands"))
        {
            return false;
        }
    }

    iperfInitCount++;

    return true;
}

#if (TCPIP_STACK_DOWN_OPERATION != 0)
void TCPIP_IPERF_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_DOWN) // interface down

    if(iperfInitCount > 0)
    {   // we're up and running
        if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
        {   // whole stack is going down
            if(--iperfInitCount == 0)
            {   // all closed
                // release resources
                if(gIperfState.signalHandle != 0)
                {
                    _TCPIPStackSignalHandlerDeregister(gIperfState.signalHandle);
                    gIperfState.signalHandle = 0;
                }
            }
        }
    }

}
#endif  // (TCPIP_STACK_DOWN_OPERATION != 0)

void TCPIP_IPERF_Task(void)
{
    TCPIP_MODULE_SIGNAL sigPend;

    sigPend = _TCPIPStackModuleSignalGet(TCPIP_THIS_MODULE_ID, TCPIP_MODULE_SIGNAL_MASK_ALL);

    if(sigPend != 0)
    { // ASYNC or RX signals occurred
        TCPIP_IPERF_Process();
    }

}


// send a signal to the iperf module that data is available
// no manager alert needed since this normally results as a higher layer (TCP) signal
static void _IperfTCPRxSignalHandler(TCP_SOCKET hTCP, TCPIP_NET_HANDLE hNet, TCPIP_TCP_SIGNAL_TYPE sigType, const void* param)
{
    if(sigType == TCPIP_TCP_SIGNAL_RX_DATA)
    {
        _TCPIPStackModuleSignalRequest(TCPIP_THIS_MODULE_ID, TCPIP_MODULE_SIGNAL_RX_PENDING, true); 
    }
}

// send a signal to the iperf module that data is available
// no manager alert needed since this normally results as a higher layer (TCP) signal
static void _IperfUDPRxSignalHandler(TCP_SOCKET hTCP, TCPIP_NET_HANDLE hNet, TCPIP_UDP_SIGNAL_TYPE sigType, const void* param)
{
    if(sigType == TCPIP_UDP_SIGNAL_RX_DATA)
    {
        _TCPIPStackModuleSignalRequest(TCPIP_THIS_MODULE_ID, TCPIP_MODULE_SIGNAL_RX_PENDING, true); 
    }
}




static void TCPIP_IPERF_Process(void)
{
	if (gIperfState.state == IPERF_STANDBY_STATE)
    { 
       return;
    }

    switch ( gIperfState.state )
    {
        /********************/
        /* RX Client States */
        /********************/

        case IPERF_RX_START_STATE:

            StateMachineRxStart();

            break;

#if defined(TCPIP_STACK_USE_UDP)
        case IPERF_UDP_RX_STATE:

            StateMachineUdpRx();

            break;

        case IPERF_UDP_RX_DRAIN_STATE:

            StateMachineUdpRxDrain();

            break;
#endif  // defined(TCPIP_STACK_USE_UDP)


#if defined(TCPIP_STACK_USE_UDP)
        case IPERF_UDP_RX_DONE_STATE:

            StateMachineUdpRxDone();

            break;
#endif  // defined(TCPIP_STACK_USE_UDP)


#if defined(TCPIP_STACK_USE_TCP)
        case IPERF_TCP_RX_LISTEN_STATE:

            StateMachineTcpListen();

            break;

        case IPERF_TCP_RX_STATE:

            StateMachineTcpRx();

            break;

        case IPERF_TCP_RX_DONE_STATE:

            StateMachineTcpRxDone();

#endif  // defined(TCPIP_STACK_USE_TCP)
            break;

        case IPERF_RX_DONE_STATE:

            StateMachineRxDone();

            break;


       /********************/
       /* TX Client states */
       /********************/


        case IPERF_TX_START_STATE:

            StateMachineTxStart();

            break;


        case IPERF_TX_ARP_RESOLVE_STATE:

           StateMachineTxArpResolve();

           break;

#if defined(TCPIP_STACK_USE_UDP)
        case IPERF_UDP_TX_OPEN_STATE:

            StateMachineUDPTxOpen();

            break;
#endif  // defined(TCPIP_STACK_USE_UDP)

#if defined(TCPIP_STACK_USE_TCP)
        case IPERF_TCP_TX_OPEN_STATE:

            StateMachineTCPTxOpen();

            break;

        case IPERF_TCP_TX_CONNECT_STATE:

            StateMachineTCPTxConnect();

            break;

        case IPERF_TCP_TX_SEGMENT_STATE:

            StateMachineTcpTxSegment();

            break;
#endif  // defined(TCPIP_STACK_USE_TCP)

#if defined(TCPIP_STACK_USE_UDP)
        case IPERF_UDP_TX_DATAGRAM_STATE:

            StateMachineUdpTxDatagram();

            break;
#endif  // defined(TCPIP_STACK_USE_UDP)

#if defined(TCPIP_STACK_USE_TCP)
        case IPERF_TCP_TX_DONE_STATE:

            StateMachineTcpTxDone();

            break;
#endif  // defined(TCPIP_STACK_USE_TCP)


#if defined(TCPIP_STACK_USE_UDP)
        case IPERF_UDP_TX_DONE_STATE:

            StateMachineUdpTxDone();

            break;
#endif  // defined(TCPIP_STACK_USE_UDP)

		default:
			IperfSetState(IPERF_STANDBY_STATE);
			break;

        }
}


//****************************************************************************
// Implementation: local functions
//****************************************************************************

static void ResetIperfCounters(void)
{
    // gIperfState.mAmount = 0;
    // gIperfState.mDuration = 10*1000; // -t: default 10 sec
    // gIperfState.mInterval = 1000; 	// -i: default 1 sec
    gIperfState.mMSS = IPERF_TCP_MSS;
    gIperfState.mDatagramSize = 1470; // -l: default 1470 bytes. UDP datagram size.
    gIperfState.totalLen = 0;
    gIperfState.pktId = 0;
    gIperfState.lastPktId = 0;
    gIperfState.errorCount = 0;
    gIperfState.outofOrder = 0;
    gIperfState.pktCount = 0;
    gIperfState.statusReported = 0;
    gIperfState.startTime = 0;
    gIperfState.stopTime = 0;

    gIperfState.lastCheckPktCount = 0;
    gIperfState.lastCheckPktId = 0;
    gIperfState.lastCheckErrorCount = 0;
    gIperfState.lastCheckTotalLen = 0;
    gIperfState.lastCheckTime = 0;

    gIperfState.isLastTransmit = false;

    gIperfState.txWaitTick = 0;
//	gIperfState.mPendingACK = 0;
//	gIperfState.mRetransmit = 0;

}

static void ascii_to_u32s(char *ptr, uint32_t *values, uint8_t count)
{
    uint8_t i;
    uint32_t tmp;

    // Convert "123.456_78_90k", with count set to 4,  to
    // unsigned 32-bit numbers 123, 456, 78 and 90000, and
    // store them in the values array.

    if(ptr == 0)
    {
        *values = 0;
        return;
    }

    for (i = 0; i < count; i++)
    {
        tmp = 0;

        while ( (*ptr > (int8_t)'9') || (*ptr < (int8_t)'0') )
        {
            if ( (*ptr == (int8_t)' ') || (*ptr == (int8_t)0) ) return; // terminates at blank or NULL.

            ptr++;
        }

        while ( (*ptr <= (int8_t)'9') && (*ptr >= (int8_t)'0') )
        {
            tmp = tmp*10 + *ptr - '0';
            ptr++;
        }
        if ( (*ptr == (int8_t)'k') || (*ptr == (int8_t)'K') )
        {
            tmp = tmp * 1000;
            ptr++;
        }
        else if ( (*ptr == (int8_t)'m') || (*ptr == (int8_t)'M') )
        {
            tmp = tmp * 1000 * 1000;
            ptr++;
        }

        values[i] = tmp;
    }
}

//
// Report bandwith, jitter, and packet loss stastistics.
// Used by in both server and client modes.
//

static void ReportBW_Jitter_Loss(tIperfReport reportType)
{
    uint32_t nAttempted;
    uint32_t nDropped;
    double kbps;
    uint32_t currentTime;
    uint32_t sec;
	uint32_t msec = 0;
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;

    currentTime = SYS_TMR_TickCountGet();

    switch ( reportType )
    {
        case INTERVAL_REPORT:

            nDropped = gIperfState.errorCount - gIperfState.lastCheckErrorCount;

            // bits-per-msec == Kbps



            sec = (currentTime- gIperfState.lastCheckTime)/SYS_TMR_TickCounterFrequencyGet();
			msec = ((double) (currentTime - gIperfState.lastCheckTime)) / (((double)(SYS_TMR_TickCounterFrequencyGet()))/1000);

            if ( gIperfState.state == (uint8_t)IPERF_UDP_TX_DONE_STATE )
            {
               nAttempted = (gIperfState.lastPktId - gIperfState.lastCheckPktId) + nDropped;
            }
            else
            {
                nAttempted = gIperfState.pktId - gIperfState.lastCheckPktId;
            }

			if ( msec == 0u )
            {
                kbps = 0;
            }
            else
            {
				kbps = ((gIperfState.totalLen - gIperfState.lastCheckTotalLen)*((double) 8)) / msec;
            }

            sec = (gIperfState.lastCheckTime - gIperfState.startTime)/SYS_TMR_TickCounterFrequencyGet();

            (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - [%2lu- %2lu sec] %3lu/ %3lu (%2lu%%)    %4lu Kbps\r\n",
                      (unsigned long)sec, 
                      (unsigned long)sec + ( (unsigned long) (gIperfState.mInterval/SYS_TMR_TickCounterFrequencyGet()) ),
                      (unsigned long)nDropped,
                      (unsigned long)nAttempted,
                      (nAttempted == 0u) ? 0 : ((unsigned long)nDropped*100/(unsigned long)nAttempted),
                      (unsigned long) (kbps + ((double) 0.5)));
            break;


        case SUBTOTAL_REPORT:
            // intentional fall-through
        case SESSION_REPORT:

           nDropped = gIperfState.errorCount;

           if (gIperfState.state == (uint8_t)IPERF_UDP_TX_DONE_STATE)
            {
               nAttempted = gIperfState.lastPktId + nDropped;
            }
            else
            {
                nAttempted = gIperfState.lastPktId;
            }

			msec = ((double) (gIperfState.stopTime - gIperfState.startTime)) / (((double)(SYS_TMR_TickCounterFrequencyGet()))/1000);


			if ( msec == 0u )
            {
                kbps = 0;
            }
            else
            {
   				kbps = (gIperfState.totalLen * ((double) 8)) / msec;
            }

            (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - [0.0- %lu.%lu sec] %3lu/ %3lu (%2lu%%)    %4lu Kbps\r\n",
                             (unsigned long)(msec/1000),
                             (unsigned long)((msec%1000)/100),
                             (unsigned long)nDropped,
                             (unsigned long)nAttempted,
                             (nAttempted == 0u) ? 0 : ((unsigned long)nDropped*100/(unsigned long)nAttempted),
                             (unsigned long) (kbps + ((double) 0.5)));
            break;
    }

    if ( reportType == 	SESSION_REPORT )
    {
      (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Session completed ...");
    }

    gIperfState.lastCheckPktId = gIperfState.pktId;
    gIperfState.lastCheckErrorCount = gIperfState.errorCount;
    gIperfState.lastCheckPktCount = gIperfState.pktCount;
    gIperfState.lastCheckTime = currentTime;
    gIperfState.lastCheckTotalLen = gIperfState.totalLen;
}


static void StateMachineRxStart(void)
{
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;
    if ( !gIperfState.mServerMode )
    {
        (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Unsupported Configuration\r\n");
        IperfSetState(IPERF_STANDBY_STATE);
        return;
    }


    switch ( gIperfState.mProtocol )
    {
#if defined(TCPIP_STACK_USE_TCP)
    case TCP_PROTOCOL:	// TCP
        /* TCP Server sockets are allocated for entire runtime duration, a call to disconnect does not free them */
        /* therefore a subsequent N+1 open will fail */
        if ( (gIperfState.tcpServerSock == INVALID_SOCKET) &&
            (gIperfState.tcpServerSock = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, gIperfState.mServerPort, (IP_MULTI_ADDRESS*)&gIperfState.localAddr)) == INVALID_SOCKET )
        {
            /* error case */
            (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Create TCP socket failed\r\n");
            IperfSetState(IPERF_STANDBY_STATE);
            return;
        }

        TCPIP_TCP_SignalHandlerRegister(gIperfState.tcpServerSock, TCPIP_TCP_SIGNAL_RX_DATA, _IperfTCPRxSignalHandler, 0);
#if (TCPIP_TCP_DYNAMIC_OPTIONS != 0)
        if(!TCPIP_TCP_OptionsSet(gIperfState.tcpServerSock, TCP_OPTION_RX_BUFF, (void*)gIperfState.rxBuffSize))
        {
            (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Set of RX buffer size failed\r\n");
        }
#endif  // (TCPIP_TCP_DYNAMIC_OPTIONS != 0)

        IperfSetState(IPERF_TCP_RX_LISTEN_STATE);
        break;
#endif  // defined(TCPIP_STACK_USE_TCP)

#if defined(TCPIP_STACK_USE_UDP)
    case UDP_PROTOCOL:	// UDP
        if ( (gIperfState.udpSock = TCPIP_UDP_ServerOpen(IP_ADDRESS_TYPE_IPV4, gIperfState.mServerPort, (IP_MULTI_ADDRESS*)&gIperfState.localAddr)) == INVALID_UDP_SOCKET )
        {
            /* error case */
            (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Create UDP socket failed\r\n");
             IperfSetState(IPERF_STANDBY_STATE);
            return;
        }
        TCPIP_UDP_SignalHandlerRegister(gIperfState.udpSock, TCPIP_UDP_SIGNAL_RX_DATA, _IperfUDPRxSignalHandler, 0);

        IperfSetState(IPERF_UDP_RX_STATE);
        break;
#endif  // defined(TCPIP_STACK_USE_UDP)

    default:
        (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Protocol error\r\n");
        IperfSetState(IPERF_STANDBY_STATE);
        return;
    }
}












static void StateMachineRxDone(void)
{
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;

   switch( gIperfState.mProtocol)
   {
#if defined(TCPIP_STACK_USE_UDP)
       case UDP_PROTOCOL:
           TCPIP_UDP_Close(  gIperfState.udpSock );
           break;
#endif  // defined(TCPIP_STACK_USE_UDP)

#if defined(TCPIP_STACK_USE_TCP)
       case TCP_PROTOCOL:
           TCPIP_TCP_Close( gIperfState.tcpServerSock );
           gIperfState.tcpServerSock = INVALID_SOCKET;
           break;
#endif  // defined(TCPIP_STACK_USE_TCP)

       default:
           break;
   }

    (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "\r\niperf: Rx done. Socket closed.\r\n");

    // Clear statistics
    ResetIperfCounters();

    // In server mode, continue to accept new session requests ...

    if ((gIperfState.mServerMode == true) 	&&
        (gIperfState.stopRequested == false) )
    {
        (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Ready for the next session.\r\n");

        IperfSetState(IPERF_RX_START_STATE);
    }
    else
    {
        (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: completed.\r\n");
        IperfSetState(IPERF_STANDBY_STATE);
    }

}




/******************************/
/* TX CLIENT CODE BEGINS HERE */
/******************************/


static void StateMachineTxStart(void)
{

   TCPIP_ARP_Resolve(gIperfState.pNetIf, &gIperfState.remoteSide.remoteIPaddress.v4Add);
   IperfSetState(IPERF_TX_ARP_RESOLVE_STATE);
   gIperfState.timer = SYS_TMR_TickCountGet();

}

static void StateMachineTxArpResolve(void)
{
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;

  if ( gIperfState.stopRequested == true )
  {
     (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: client session closed.\r\n");
     IperfSetState(IPERF_STANDBY_STATE);
     return;
  }

  if(!TCPIP_ARP_IsResolved(gIperfState.pNetIf, &gIperfState.remoteSide.remoteIPaddress.v4Add, &gIperfState.remoteMACAddr))
  {
      /* every 3 seconds print a warning */
      if( SYS_TMR_TickCountGet() - gIperfState.timer > 5*SYS_TMR_TickCounterFrequencyGet() )
      {
         (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: ARP unable to resolve the MAC address of remote side.\r\n");
         gIperfState.timer = SYS_TMR_TickCountGet();
      }
      return;
  }

  (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - RemoteNode MAC: %x %x %x %x %x %x\r\n",
           gIperfState.remoteMACAddr.v[0],
           gIperfState.remoteMACAddr.v[1],
           gIperfState.remoteMACAddr.v[2],
           gIperfState.remoteMACAddr.v[3],
           gIperfState.remoteMACAddr.v[4],
           gIperfState.remoteMACAddr.v[5]);
  
#if defined(TCPIP_STACK_USE_UDP)
  if ( gIperfState.mProtocol == UDP_PROTOCOL )
  {
     IperfSetState(IPERF_UDP_TX_OPEN_STATE);
  }
#endif  // defined(TCPIP_STACK_USE_UDP)

#if defined(TCPIP_STACK_USE_TCP)
  if ( gIperfState.mProtocol == TCP_PROTOCOL )
  {
     IperfSetState(IPERF_TCP_TX_OPEN_STATE);
  }
#endif  // defined(TCPIP_STACK_USE_TCP)

}







static void GenericTxHeaderPreparation(uint8_t *pData, bool isTheLastTransmit)
{
    tIperfPktInfo *pPktInfo = NULL;
    uint32_t currentTime;
    tClientHdr *pClientHdr = NULL;
    long tmp2;

    if ( gIperfState.pktId == 0 ) {
        // The first tx packet
    }

    switch ( gIperfState.mProtocol )
    {
#if defined(TCPIP_STACK_USE_TCP)
        case TCP_PROTOCOL: // TCP

            // Required by iperf.
            pClientHdr = (tClientHdr *) pData;

            // We borrow the same tIperfPktInfo structure to embed
            // some useful (non-standard iperf) meta info.
            // Unfortunately, the order has to be reversed.

            pPktInfo = (tIperfPktInfo *) (pClientHdr + 1);

            break;
#endif  // defined(TCPIP_STACK_USE_TCP)

#if defined(TCPIP_STACK_USE_UDP)
        case UDP_PROTOCOL: // UDP

            // Both are required by iperf.

            pPktInfo = (tIperfPktInfo *) pData;
            pClientHdr = (tClientHdr *) (pPktInfo + 1);

            break;
#endif  // defined(TCPIP_STACK_USE_UDP)

        default:
            break;
    }

    // Client header:
    // Needed for all UDP packets.
    // For TCP, only the first two segments need this info. However,
    // there seems to be no harm to put it to all segments though.

    pClientHdr->flags = TCPIP_Helper_htonl( (uint32_t) 0);
    pClientHdr->numThreads = TCPIP_Helper_htonl((uint32_t) 1);
    pClientHdr->mPort = TCPIP_Helper_htonl((uint32_t) gIperfState.mServerPort);
    pClientHdr->bufferlen = TCPIP_Helper_htonl( (uint32_t) 0);
    pClientHdr->mWinBand = TCPIP_Helper_htonl(gIperfState.mTxRate);

    if ( gIperfState.mAmount != 0u )
    {
        pClientHdr->mAmount = TCPIP_Helper_htonl(gIperfState.mAmount);
    }
    else
    {
        pClientHdr->mAmount = TCPIP_Helper_htonl( - (long) (gIperfState.mDuration/10) );
    }

    // Additional info: needed for UDP only.
    // No harm to put it to all TCP segments though.

    if ( isTheLastTransmit == true )
    {
        // The last UDP tx packet. Some caveats:
        // 1. Iperf uses a negative Id for the last tx packet.
        // 2. Its id should not change during retransmit.

        pPktInfo->id = - ( (long) (gIperfState.pktId - gIperfState.nAttempts) );
    }
    else
    {
        pPktInfo->id = gIperfState.pktId;
    }

    pPktInfo->id = TCPIP_Helper_htonl(pPktInfo->id);

    currentTime = SYS_TMR_TickCountGet();

    pPktInfo->tv_sec = TCPIP_Helper_htonl(currentTime / SYS_TMR_TickCounterFrequencyGet());

    /* get the remainder of the ticks using modulus */
    tmp2 = ((gIperfState.stopTime - gIperfState.startTime)%SYS_TMR_TickCounterFrequencyGet());

    /* normalize  to uSecs */
    tmp2 =  tmp2*1000/SYS_TMR_TickCounterFrequencyGet(); /* Convert to mSec */
    tmp2 *= 1000;   /* 1000 uSecs per mSec */


    pPktInfo->tv_usec = TCPIP_Helper_htonl( tmp2 );

    return;
}



static tIperfTxResult GenericTxStart(void)
{
    uint32_t currentTime;
    bool iperfKilled;
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;
 
    currentTime = SYS_TMR_TickCountGet();

    if ( currentTime < (gIperfState.nextTxTime - TIMING_ERROR_MARGIN))
    {
        // Wait until we are scheduled to Tx.
        return IPERF_TX_WAIT;
    }

    iperfKilled = gIperfState.stopRequested;
    if ((iperfKilled == true) ||
            ((gIperfState.mDuration != 0u) &&
             (currentTime > (gIperfState.startTime + gIperfState.mDuration))) ||
            ((gIperfState.mAmount != 0u) &&
             (gIperfState.totalLen > gIperfState.mAmount)))
    {
        // Prepare to transmit the last packet.
        // Although the last packet needs to be retransmitted UDP_FIN_RETRANSMIT_COUNT times,
        // if we are in UDP mode.

        gIperfState.isLastTransmit = true;
    }

    if ( gIperfState.pktId == 0 )
    {
        // The first pkt is going out ...

        // Reset startTime, to get a more accurate report.

        gIperfState.startTime = currentTime;
        gIperfState.nextTxTime = gIperfState.startTime;

        gIperfState.lastCheckTime = 	gIperfState.startTime;

        gIperfState.lastCheckPktId = gIperfState.pktId;
        gIperfState.lastCheckPktCount = gIperfState.pktCount;
        gIperfState.lastCheckErrorCount = gIperfState.errorCount;
        gIperfState.nAttempts = 0;
    }

    switch(gIperfState.mProtocol)
    {
#if defined(TCPIP_STACK_USE_TCP)
        case TCP_PROTOCOL:
            /* Manage socket */
            if( TCPIP_TCP_GetIsReady(gIperfState.tcpClientSock) > 0u )
            {
                TCPIP_TCP_Discard(gIperfState.tcpClientSock);
                return IPERF_TX_WAIT;
            }

            if ( TCPIP_TCP_WasReset(gIperfState.tcpClientSock) )
            {
                // We don't close the socket. We wait for user to "kill iperf" explicitly.
                (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "\r\niperf: Warning, TCP server disconnect detected\r\n");
            }

            if  (( TCPIP_TCP_PutIsReady(gIperfState.tcpClientSock) <= gIperfState.mMSS ) && (!iperfKilled))
                return IPERF_TX_WAIT;

            break;
#endif  // defined(TCPIP_STACK_USE_TCP)

#if defined(TCPIP_STACK_USE_UDP)
        case UDP_PROTOCOL:
            /* Manage socket */
            if( TCPIP_UDP_GetIsReady(gIperfState.udpSock) > 0u )
            {
                TCPIP_UDP_Discard(gIperfState.udpSock);
                return IPERF_TX_WAIT;
            }

            if ( TCPIP_UDP_TxPutIsReady(gIperfState.udpSock, gIperfState.mDatagramSize) < gIperfState.mDatagramSize )
            {
                if(gIperfState.txWaitTick == 0)
                {     // beginning wait period
                    gIperfState.txWaitTick = SYS_TMR_TickCountGet() + ((TCPIP_IPERF_TX_WAIT_TMO * SYS_TMR_TickCounterFrequencyGet()) + 999)/1000;
                    return IPERF_TX_WAIT;
                }
                else if((int32_t)(SYS_TMR_TickCountGet() - gIperfState.txWaitTick) < 0)
                { // wait some more
                    return IPERF_TX_WAIT;
                }

                // TX ready timeout
                (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "iperf: Failed to get %d bytes socket TX space\r\n", gIperfState.mDatagramSize);
                return IPERF_TX_FAIL;
            }
            else
            {    // reset retry tick counter
                gIperfState.txWaitTick = 0;
            }

            break;
#endif  // defined(TCPIP_STACK_USE_UDP)

        default:
            break;
    }


    // One Tx per mPktPeriod msec.
    gIperfState.nextTxTime = currentTime + gIperfState.mPktPeriod;

    GenericTxHeaderPreparation(g_bfr, gIperfState.isLastTransmit);

    switch( gIperfState.mProtocol)
    {
#if defined(TCPIP_STACK_USE_TCP)
        case TCP_PROTOCOL:
            gIperfState.remainingTxData = (gIperfState.mMSS - MAX_BUFFER);

            if (( TCPIP_TCP_ArrayPut(gIperfState.tcpClientSock, (uint8_t*) g_bfr, MAX_BUFFER) != MAX_BUFFER ) && (!iperfKilled))
            {
                (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Socket send failed\r\n");
                gIperfState.errorCount++;
                return IPERF_TX_FAIL;
            }

            break;
#endif  // defined(TCPIP_STACK_USE_TCP)


#if defined(TCPIP_STACK_USE_UDP)
        case UDP_PROTOCOL:

            gIperfState.remainingTxData = (gIperfState.mDatagramSize - MAX_BUFFER);

            if ( TCPIP_UDP_ArrayPut(gIperfState.udpSock, g_bfr, MAX_BUFFER) != MAX_BUFFER )
            {
                (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Socket send failed\r\n");
                gIperfState.errorCount++;
                return IPERF_TX_FAIL;
            }
            break;
#endif  // defined(TCPIP_STACK_USE_UDP)

        default:
            break;
    }

    return IPERF_TX_OK;

}






static void GenericTxEnd(void)
{
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;

    if(  gIperfState.remainingTxData  > 0u )
    {
        /* unhandled error */
        (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Socket send failed\r\n");
        gIperfState.errorCount++;
    }
    else
    {
        // send successful.

        if ( gIperfState.pktCount == 0u )
        {
            // first tx pkt

            IPV4_ADDR lclAddress;

            (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "\n\riperf: Session started ...\r\n");

            lclAddress.Val = TCPIP_STACK_NetAddress(gIperfState.pNetIf);

            (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - Local  %u.%u.%u.%u port %u connected with\r\n",
                    lclAddress.v[0],
                    lclAddress.v[1],
                    lclAddress.v[2],
                    lclAddress.v[3],
                    gIperfState.localPort);

            (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - Remote %u.%u.%u.%u port %u\r\n",
                    gIperfState.remoteSide.remoteIPaddress.v4Add.v[0],
                    gIperfState.remoteSide.remoteIPaddress.v4Add.v[1],
                    gIperfState.remoteSide.remoteIPaddress.v4Add.v[2],
                    gIperfState.remoteSide.remoteIPaddress.v4Add.v[3],
                    gIperfState.mServerPort );

            (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - Target rate = %ld bps, period = %ld ms\r\n",
                    (unsigned long)gIperfState.mTxRate, 
                    (unsigned long)(gIperfState.mPktPeriod*1000/SYS_TMR_TickCounterFrequencyGet()) );

        }

        gIperfState.pktId++;
        gIperfState.pktCount++;

#if defined(TCPIP_STACK_USE_UDP)
        if ( gIperfState.mProtocol == UDP_PROTOCOL )
        {
            gIperfState.totalLen += gIperfState.mDatagramSize;
        }
#endif  // defined(TCPIP_STACK_USE_UDP)

#if defined(TCPIP_STACK_USE_TCP)
        if ( gIperfState.mProtocol == TCP_PROTOCOL )
        {

            gIperfState.totalLen += gIperfState.mMSS;
        }
#endif  // defined(TCPIP_STACK_USE_TCP)


    }

    gIperfState.lastPktId = gIperfState.pktId - 1;



    if ( (int32_t)(SYS_TMR_TickCountGet() - gIperfState.lastCheckTime) >= (gIperfState.mInterval - TIMING_ERROR_MARGIN) )
    {
        // Time to report statistics
        ReportBW_Jitter_Loss(INTERVAL_REPORT);

        //gIperfState.lastCheckPktCount = gIperfState.pktCount;
    }


    if ( gIperfState.isLastTransmit == true )
    {
        switch(gIperfState.mProtocol)
        {
#if defined(TCPIP_STACK_USE_UDP)
            case UDP_PROTOCOL:
                if(++gIperfState.nAttempts < UDP_FIN_RETRANSMIT_COUNT)
                {

                    if ( gIperfState.nAttempts == 1u )
                    {
                        // So the normal pkt statistics is not mixed with the retransmited last pkt.
                        gIperfState.stopTime = SYS_TMR_TickCountGet();

                        ReportBW_Jitter_Loss(SUBTOTAL_REPORT);
                        (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "    -----------------------------------------\r\n"); 
                    }

                    // Don't follow the same transmision rate during retransmit.
                    gIperfState.mPktPeriod = UDP_FIN_RETRANSMIT_PERIOD;
                }
                else
                {
                    IperfSetState(IPERF_UDP_TX_DONE_STATE);
                }
                break;
#endif  // defined(TCPIP_STACK_USE_UDP)

#if defined(TCPIP_STACK_USE_TCP)
            case TCP_PROTOCOL:
                IperfSetState(IPERF_TCP_TX_DONE_STATE);
                break;
#endif  // defined(TCPIP_STACK_USE_TCP)

            default:
                break;
        }

        gIperfState.stopTime = SYS_TMR_TickCountGet();
    }

}




static void GenericTxDone(void)
{
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;
    if ( gIperfState.statusReported == 0u )
    {
        ReportBW_Jitter_Loss(SESSION_REPORT);
        gIperfState.statusReported = 1;
    }

    IperfSetState(IPERF_STANDBY_STATE);

    (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Tx done. Socket closed.\r\n");

    // Clear statistics
    ResetIperfCounters();

    //(gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "    Back to standby mode.\r\n");
    (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: completed.\r\n");

}

// TCP state machine functions
#if defined(TCPIP_STACK_USE_TCP)
static void StateMachineTcpTxDone(void)
{
    GenericTxDone();

    // close gracefully
    TCPIP_TCP_Close(gIperfState.tcpClientSock);
    gIperfState.tcpClientSock = INVALID_SOCKET;
}

static void StateMachineTcpTxSegment(void)
{
    tIperfTxResult txRes = GenericTxStart();

    if(txRes == IPERF_TX_OK)
    {   // go ahead and transmit
       TcpTxFillSegment();
       TCPIP_TCP_Flush(gIperfState.tcpClientSock);
       GenericTxEnd();
    }
    else if(txRes == IPERF_TX_FAIL)
    {
        IperfSetState(IPERF_TCP_TX_DONE_STATE);
        gIperfState.stopTime = SYS_TMR_TickCountGet();
    }
    // else wait...
}

/* This routine does a piecewise send, because the entire RAM buffer may not be available for putArray */
static void TcpTxFillSegment(void)
{
  uint16_t chunk;

  /* Fill the buffer with ASCII char T */
  memset( g_bfr, 0x54, MAX_BUFFER);

  while( gIperfState.remainingTxData > 0u )
  {
    chunk = MAX_BUFFER;

    /* finish case where we get more than is needed */
    if ( gIperfState.remainingTxData < MAX_BUFFER )
      chunk = gIperfState.remainingTxData;

    gIperfState.remainingTxData -= chunk;

    if ( TCPIP_TCP_ArrayPut( gIperfState.tcpClientSock, (uint8_t *) g_bfr, chunk) != chunk )
      return;

  }

}

static void StateMachineTCPTxOpen(void)
{
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;

   if  ( (gIperfState.tcpClientSock = TCPIP_TCP_ClientOpen(IP_ADDRESS_TYPE_IPV4, gIperfState.mServerPort, 0)) == INVALID_SOCKET )
   {
       /* error case */
        (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Create TCP socket failed\r\n");
        IperfSetState(IPERF_STANDBY_STATE);
        return;
    }
   TCPIP_TCP_SignalHandlerRegister(gIperfState.tcpClientSock, TCPIP_TCP_SIGNAL_RX_DATA, _IperfTCPRxSignalHandler, 0);

   if(gIperfState.localAddr.Val != 0)
   {
       TCPIP_TCP_Bind(gIperfState.tcpClientSock, IP_ADDRESS_TYPE_IPV4, gIperfState.mServerPort, (IP_MULTI_ADDRESS*)&gIperfState.localAddr);
   }
   TCPIP_TCP_RemoteBind(gIperfState.tcpClientSock, IP_ADDRESS_TYPE_IPV4, 0,  (IP_MULTI_ADDRESS*)&gIperfState.remoteSide.remoteIPaddress);
    gIperfState.localPort = TCPIP_IPERF_TCP_LOCAL_PORT_START_NUMBER;

    (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "---------------------------------------------------------\r\n");
    (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "iperf: Client connecting to %u.%u.%u.%u, TCP port %u\r\n",
              gIperfState.remoteSide.remoteIPaddress.v4Add.v[0],
              gIperfState.remoteSide.remoteIPaddress.v4Add.v[1],
              gIperfState.remoteSide.remoteIPaddress.v4Add.v[2],
              gIperfState.remoteSide.remoteIPaddress.v4Add.v[3],
              gIperfState.mServerPort );

    IperfSetState(IPERF_TCP_TX_CONNECT_STATE);

#if (TCPIP_TCP_DYNAMIC_OPTIONS != 0)
    if(!TCPIP_TCP_OptionsSet(gIperfState.tcpClientSock, TCP_OPTION_TX_BUFF, (void*)gIperfState.txBuffSize))
    {
        (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Set of TX buffer size failed\r\n");
    }
#endif  // (TCPIP_TCP_DYNAMIC_OPTIONS != 0)

    TCPIP_TCP_Connect(gIperfState.tcpClientSock);
    gIperfState.timer = SYS_TMR_TickCountGet();
}

static void StateMachineTcpRxDone(void)
{
    if ( gIperfState.statusReported == 0u )
    {
        ReportBW_Jitter_Loss(SESSION_REPORT);
        gIperfState.statusReported = 1;
    }

    IperfSetState(IPERF_RX_DONE_STATE);
}

static void StateMachineTcpRx(void)
{
    uint16_t length;
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;

    if( (length = TCPIP_TCP_GetIsReady(gIperfState.tcpServerSock)) == 0 )
    {

      if ( TCPIP_TCP_WasReset(gIperfState.tcpServerSock)  )
      {
          gIperfState.stopTime = SYS_TMR_TickCountGet();
          IperfSetState(IPERF_TCP_RX_DONE_STATE);
          return;
      }

    }
    else
    {
       if ( gIperfState.pktId == 0)
       {
          IPV4_ADDR lclAddress;
          
          // This is the first rx pkt.
          (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "\r\niperf: Session started ...\r\n");

          gIperfState.startTime = SYS_TMR_TickCountGet();
          gIperfState.lastCheckTime = 	gIperfState.startTime;

          gIperfState.lastCheckPktId = gIperfState.pktId;
          lclAddress.Val = TCPIP_STACK_NetAddress(TCPIP_TCP_SocketNetGet(gIperfState.tcpServerSock));

          (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - Local  %u.%u.%u.%u port %u connected with\r\n",
                   lclAddress.v[0],
                   lclAddress.v[1],
                   lclAddress.v[2],
                   lclAddress.v[3],
                   gIperfState.mServerPort);

          (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - Remote %u.%u.%u.%u port %u\r\n",
                   gIperfState.remoteSide.remoteIPaddress.v4Add.v[0],
                   gIperfState.remoteSide.remoteIPaddress.v4Add.v[1],
                   gIperfState.remoteSide.remoteIPaddress.v4Add.v[2],
                   gIperfState.remoteSide.remoteIPaddress.v4Add.v[3],
                   gIperfState.remoteSide.remotePort );
       }

       gIperfState.pktId++;
       gIperfState.pktCount++;
       gIperfState.lastPktId = gIperfState.pktId;
       gIperfState.totalLen += length;

       /* read the remaining datagram payload */
       /* a UdpDiscard would be disingenuous, because it would not reflect the bandwidth at L7 */
       while ( length > 0 )
       {
          uint16_t chunk;

          if ( length <  (uint16_t)MAX_BUFFER )
            chunk = length;
          else
            chunk = MAX_BUFFER;

          TCPIP_TCP_ArrayGet( gIperfState.tcpServerSock, (uint8_t*)g_bfr, chunk);
          length -= chunk;
       }

    }

    if ((gIperfState.pktId != (long)0) &&
       ((int32_t)(SYS_TMR_TickCountGet() - gIperfState.lastCheckTime) > gIperfState.mInterval) )
    {
         // Time to report statistics
         ReportBW_Jitter_Loss(INTERVAL_REPORT);
    }

    if ( gIperfState.stopRequested == true )
    {
       IperfSetState(IPERF_TCP_RX_DONE_STATE);
       gIperfState.stopTime = SYS_TMR_TickCountGet();

       return;
    }
}

static void StateMachineTcpListen(void)
{

   if ( gIperfState.stopRequested == true )
   {
        IperfSetState(IPERF_RX_DONE_STATE);
        return;
   }

   if( TCPIP_TCP_IsConnected(gIperfState.tcpServerSock) )
   {
      TCP_SOCKET_INFO tcpSocketInfo;
	  TCPIP_TCP_SocketInfoGet( gIperfState.tcpServerSock, &tcpSocketInfo);
      memcpy ( (void *) &gIperfState.remoteSide, &tcpSocketInfo, sizeof ( TCP_SOCKET_INFO) );
      IperfSetState(IPERF_TCP_RX_STATE);

      /* clear the stack's reset flag */
      TCPIP_TCP_WasReset(gIperfState.tcpServerSock);
   }
}

static void StateMachineTCPTxConnect(void)
{

    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;
    if ( gIperfState.stopRequested == true )
    {
        IperfSetState(IPERF_TCP_TX_DONE_STATE);
        return;
    }

    if( !TCPIP_TCP_IsConnected(gIperfState.tcpClientSock) )
    {

        // Time out if too much time is spent in this state
        if(SYS_TMR_TickCountGet()- gIperfState.timer > 5*SYS_TMR_TickCounterFrequencyGet())
        {
            TCPIP_TCP_Close(gIperfState.tcpClientSock);
            gIperfState.tcpClientSock = INVALID_SOCKET;
            (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: TCP Client connection timeout\r\n");
            IperfSetState(IPERF_TCP_TX_DONE_STATE);
        }

        return;
    }

    /* reset the reset flag so we don't get a false positive */
    TCPIP_TCP_WasReset(gIperfState.tcpClientSock);

    gIperfState.startTime = SYS_TMR_TickCountGet();
    gIperfState.nextTxTime = gIperfState.startTime + gIperfState.mPktPeriod;
    IperfSetState(IPERF_TCP_TX_SEGMENT_STATE);
}

#endif  // defined(TCPIP_STACK_USE_TCP)

// UDP state machine functions
#if defined(TCPIP_STACK_USE_UDP)
static void StateMachineUdpTxDone(void)
{

    GenericTxDone();

    TCPIP_UDP_Close(gIperfState.udpSock );
}


static void StateMachineUdpTxDatagram(void)
{
    tIperfTxResult txRes = GenericTxStart();

    if ( txRes == IPERF_TX_OK )
    {   // go ahead and transmit
       uint16_t txData = UdpTxFillDatagram();
       if(TCPIP_UDP_Flush(gIperfState.udpSock) == 0)
       {   // failed; discard data
           TCPIP_UDP_TxOffsetSet(gIperfState.udpSock, 0, 0);
       }
       else
       {
           gIperfState.remainingTxData -= txData;
           GenericTxEnd();
       }
    }
    else if(txRes == IPERF_TX_FAIL)
    {
        IperfSetState(IPERF_UDP_TX_DONE_STATE);
        gIperfState.stopTime = SYS_TMR_TickCountGet();
    }
    // else wait...
}

/* This routine does a piece wis send, because the entire RAM pkt buffer may not be available for putArray */
static uint16_t UdpTxFillDatagram(void)
{

    uint16_t chunk;
    uint16_t remainingTxData;
    uint16_t txData = 0;

    /* Fill the buffer with ASCII char U */
    memset( g_bfr, 0x55, MAX_BUFFER);

    remainingTxData = gIperfState.remainingTxData;
    while( remainingTxData > 0u )
    {
        chunk = MAX_BUFFER;

        /* finish case where we get more than is needed */
        if ( remainingTxData < MAX_BUFFER )
            chunk = remainingTxData;

        remainingTxData -= chunk;
        txData += chunk;

        if (  TCPIP_UDP_ArrayPut(gIperfState.udpSock, (uint8_t *) g_bfr, chunk) != chunk )
        {
            break;
        }

    }

    return txData;

}

static void StateMachineUDPTxOpen(void)
{	
	UDP_SOCKET_INFO UdpSkt;
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;
	
    if ( (gIperfState.udpSock = TCPIP_UDP_ClientOpen(IP_ADDRESS_TYPE_IPV4, gIperfState.mServerPort, (IP_MULTI_ADDRESS*)&gIperfState.remoteSide.remoteIPaddress.v4Add)) == INVALID_UDP_SOCKET )
    {
        /* error case */
        (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Create UDP socket failed\r\n");
        IperfSetState(IPERF_STANDBY_STATE);
        return;
    }

    TCPIP_UDP_SignalHandlerRegister(gIperfState.udpSock, TCPIP_UDP_SIGNAL_RX_DATA, _IperfUDPRxSignalHandler, 0);
	
    if(!TCPIP_UDP_OptionsSet(gIperfState.udpSock, UDP_OPTION_TX_BUFF, (void*)gIperfState.mDatagramSize))
    {
        (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Set of TX buffer size failed\r\n");
    }
    if(!TCPIP_UDP_OptionsSet(gIperfState.udpSock, UDP_OPTION_TX_QUEUE_LIMIT, (void*)TCPIP_IPERF_TX_QUEUE_LIMIT))
    {
        (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Set of TX queuing limit failed\r\n");
    }

    TCPIP_UDP_SocketNetSet(gIperfState.udpSock, gIperfState.pNetIf);
	TCPIP_UDP_SocketInfoGet(gIperfState.udpSock, &UdpSkt);
    gIperfState.localPort = UdpSkt.localPort;

    (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "---------------------------------------------------------\r\n");
    (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam,  "iperf: Client connecting to %u.%u.%u.%u, UDP port %u\r\n",
              gIperfState.remoteSide.remoteIPaddress.v4Add.v[0],
              gIperfState.remoteSide.remoteIPaddress.v4Add.v[1],
              gIperfState.remoteSide.remoteIPaddress.v4Add.v[2],
              gIperfState.remoteSide.remoteIPaddress.v4Add.v[3],
              gIperfState.mServerPort );
    IperfSetState(IPERF_UDP_TX_DATAGRAM_STATE);

    gIperfState.startTime = SYS_TMR_TickCountGet();

     // Wait for a few seconds before first TCP tx, so we can resolve ARP.
    gIperfState.nextTxTime = gIperfState.startTime + gIperfState.mPktPeriod;

}

static void StateMachineUdpRxDone(void)
{
    tIperfPktInfo *pPktInfo;
    tServerHdr *pServer_hdr;
    float tmp2;


    if ( gIperfState.statusReported == 0u )
    {
        ReportBW_Jitter_Loss(SESSION_REPORT);
        gIperfState.statusReported = 1;
    }

    /* Drain any waiting pkts */
    if (  TCPIP_UDP_GetIsReady(gIperfState.udpSock)  )
    {
        TCPIP_UDP_Discard(gIperfState.udpSock);
        return;
    }

    // Send the iperf UDP "FIN-ACK" 10 times.
    if ( gIperfState.nAttempts++ > 10u )
    {
        IperfSetState(IPERF_RX_DONE_STATE);
        return;
    }

     /* Make sure space is available to TX the ACK packet of 128 bytes */
    if ( TCPIP_UDP_TxPutIsReady(gIperfState.udpSock, 128 ) > 0u )
    {

      pPktInfo = (tIperfPktInfo *) g_bfr;
      pServer_hdr = (tServerHdr *) (pPktInfo +1);

      pPktInfo->id = TCPIP_Helper_htonl( -gIperfState.lastPktId );
      pPktInfo->tv_sec = 0;
      pPktInfo->tv_usec = 0;

      pServer_hdr->flags = TCPIP_Helper_htonl(HEADER_VERSION1);
      pServer_hdr->total_len1 = 0;
      pServer_hdr->total_len2 = TCPIP_Helper_htonl( (uint32_t) gIperfState.totalLen);

      pServer_hdr->stop_sec =  TCPIP_Helper_htonl( (uint32_t) (gIperfState.stopTime - gIperfState.startTime)/SYS_TMR_TickCounterFrequencyGet());

      /* get the remainder of the ticks using modulus */
      tmp2 = ((gIperfState.stopTime - gIperfState.startTime)%SYS_TMR_TickCounterFrequencyGet());

      /* normalize  to uSecs */
      tmp2 =  tmp2*1000/SYS_TMR_TickCounterFrequencyGet(); /* Convert to mSec */
      tmp2 *= 1000;   /* 1000 uSecs per mSec */


      pServer_hdr->stop_usec = TCPIP_Helper_htonl( (uint32_t) tmp2 );
      pServer_hdr->error_cnt = TCPIP_Helper_htonl( (uint32_t)  gIperfState.errorCount);;
      pServer_hdr->outorder_cnt = TCPIP_Helper_htonl( (uint32_t) gIperfState.outofOrder);
      pServer_hdr->datagrams = TCPIP_Helper_htonl( (uint32_t) gIperfState.lastPktId);
      pServer_hdr->jitter1 = 0;
      pServer_hdr->jitter2 = 0;

      TCPIP_UDP_ArrayPut(gIperfState.udpSock, (uint8_t*)g_bfr, MAX_BUFFER);

      uint8_t tmpBuffer[128-MAX_BUFFER];
      memset(tmpBuffer, 0, sizeof(tmpBuffer));
      TCPIP_UDP_ArrayPut(gIperfState.udpSock, tmpBuffer, sizeof(tmpBuffer));
      
      TCPIP_UDP_Flush(gIperfState.udpSock );

    }

}

static void StateMachineUdpRxDrain(void)
{
    if( TCPIP_UDP_GetIsReady(gIperfState.udpSock) > (uint8_t)0 )
    {
         TCPIP_UDP_Discard(gIperfState.udpSock);
         return;
    }

   /* If iperf kill was done, just jump to closing the socket */
   if ( gIperfState.stopRequested )
   {
       IperfSetState(IPERF_RX_DONE_STATE);
   }
   else /* go ahead an generate reports, etc */
   {
       IperfSetState(IPERF_UDP_RX_DONE_STATE);
   }

}

static void StateMachineUdpRx(void)
{
    uint16_t length =0;
    tIperfPktInfo *pPktInfo;
    UDP_SOCKET_INFO UdpSkt;
    IPV4_ADDR      lclIpAddr, remIpAddr;
    const void* cmdIoParam = gIperfState.pCmdIO->cmdIoParam;


    // Do nothing if no data is waiting
    while( (length = TCPIP_UDP_GetIsReady(gIperfState.udpSock)) >= (uint16_t)(sizeof(tIperfPktInfo)) )
    {
       /* The GetArray should not fail... */
       if ( TCPIP_UDP_ArrayGet(gIperfState.udpSock, (uint8_t*)g_bfr, sizeof(tIperfPktInfo)) != sizeof(tIperfPktInfo) )
       {
          (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: UDP Get Array Failed\r\n");
          IperfSetState(IPERF_UDP_RX_DRAIN_STATE);
          return;
       }

       pPktInfo = (tIperfPktInfo *) g_bfr;
       gIperfState.pktId = TCPIP_Helper_htonl(pPktInfo->id);

       if ( (gIperfState.pktCount == (uint32_t)0) && (gIperfState.pktId < (long)0) )
       {
          // Ignore retransmits from previous session.
          TCPIP_UDP_Discard(gIperfState.udpSock);
          return;
       }

       gIperfState.pktCount++;
       if (gIperfState.pktCount == (uint32_t)1 )
       {
          // The first pkt is used to set up the server,
          // does not count as a data pkt.

          (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "\r\niperf: Session started ...");

          if ( gIperfState.pktId != 0 )
          {
             // We have lost a few packets before the first pkt arrived.
             (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "iperf: - First pkt id = %ld (should be 0)\r\n",
                              gIperfState.pktId);
             // The first data pkt starts with id = 1.
             gIperfState.errorCount	+= 	gIperfState.pktId - 1;
          }

          gIperfState.lastPktId = gIperfState.pktId;

		  TCPIP_UDP_SocketInfoGet(gIperfState.udpSock, &UdpSkt);
          lclIpAddr.Val = TCPIP_STACK_NetAddress(TCPIP_UDP_SocketNetGet(gIperfState.udpSock));
          remIpAddr.Val = UdpSkt.remoteIPaddress.v4Add.Val;

          (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - Local  %u.%u.%u.%u port %u connected with\r\n",
                           lclIpAddr.v[0],
                           lclIpAddr.v[1],
                           lclIpAddr.v[2],
                           lclIpAddr.v[3],
                           gIperfState.mServerPort);
		  
          (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "    - Remote %u.%u.%u.%u port %u\r\n",      
                           remIpAddr.v[0],
                           remIpAddr.v[1],
                           remIpAddr.v[2],
                           remIpAddr.v[3],
                           UdpSkt.remotePort);

          // Store the remote info so we can send the iperf "UDP-FIN-ACK" msg
          gIperfState.remoteSide.remoteIPaddress.v4Add.Val = remIpAddr.Val;
          gIperfState.remoteSide.remotePort =  UdpSkt.remotePort;

          gIperfState.startTime = SYS_TMR_TickCountGet();
          //gIperfState.remoteStartTime = IPERFTOHL(pPktInfo->tv_sec);

          gIperfState.lastCheckTime = 	gIperfState.startTime;

          gIperfState.lastCheckPktId = gIperfState.pktId;
          gIperfState.lastCheckPktCount = gIperfState.pktCount;
          gIperfState.lastCheckErrorCount = gIperfState.errorCount;

          TCPIP_UDP_Discard(gIperfState.udpSock);

          continue;
      }

      gIperfState.totalLen += length;

      if ( gIperfState.pktId < 0 )
      {
         // this is the last datagram
         gIperfState.pktId = - gIperfState.pktId;

         gIperfState.stopTime = SYS_TMR_TickCountGet();
        //gIperfState.remoteStopTime = IPERFTOHL(pPktInfo->tv_sec);

        gIperfState.nAttempts = 0;
        IperfSetState(IPERF_UDP_RX_DRAIN_STATE);
      }

      if ( gIperfState.pktId != gIperfState.lastPktId+1 )
      {
         if ( gIperfState.pktId < gIperfState.lastPktId+1 )
         {
            gIperfState.outofOrder++;
         }
         else
         {
            gIperfState.errorCount += gIperfState.pktId - (gIperfState.lastPktId+1);
         }
      }

      // never decrease pktId (e.g. if we get an out-of-order packet)
      if ( gIperfState.pktId == gIperfState.lastPktId )
      {
         (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Recv duplicated pkt\r\n");
      }

      if ( gIperfState.pktId > gIperfState.lastPktId )
      {
         gIperfState.lastPktId = gIperfState.pktId;
      }

      /* read the remaining datagram payload - the full payload */
      /* a UdpDiscard would be disingenuous, because it would not reflect the bandwidth at L7 */
      length -=  sizeof(tIperfPktInfo);
      while ( length > 0 )
      {
         uint16_t chunk;

         if ( length <  (uint16_t)MAX_BUFFER )
            chunk = length;
         else
            chunk = MAX_BUFFER;

         TCPIP_UDP_ArrayGet(gIperfState.udpSock, (uint8_t*)g_bfr, chunk);
         length -= chunk;
      }


    }  /* end got a datagram */

    if ( (gIperfState.pktCount != (uint32_t)0) &&
         ((int32_t)(SYS_TMR_TickCountGet() - gIperfState.lastCheckTime) > gIperfState.mInterval) )
    {
        if ( gIperfState.pktCount == gIperfState.lastCheckPktCount )
        {
          // No events in gIperfState.mInterval msec, we timed out
          (gIperfState.pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Rx timed out\r\n");

          gIperfState.stopTime = SYS_TMR_TickCountGet();

          gIperfState.nAttempts = 0;
          IperfSetState(IPERF_UDP_RX_DRAIN_STATE);
        }
        else
        {
          ReportBW_Jitter_Loss(INTERVAL_REPORT);
        }
    }

    if ( gIperfState.stopRequested == true )
    {
        IperfSetState(IPERF_UDP_RX_DRAIN_STATE);
        return;
    }

}

#endif  // defined(TCPIP_STACK_USE_UDP)

static int CommandIperfStart(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    uint8_t i;
    char *ptr;
    uint32_t values[4];
    float pktRate;
    uint16_t payloadSize = 0;
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    // Check if iperf is still running
    if ( gIperfState.state != (uint8_t)IPERF_STANDBY_STATE )
    {
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "\r\niperf: session already started\r\n");
            return false;
    }

	// preparation for new iperf test
    gIperfState.mServerMode = false;
#if defined(TCPIP_STACK_USE_TCP)
    gIperfState.mProtocol = TCP_PROTOCOL;   			// default is TCP mode.
#else
    gIperfState.mProtocol = UDP_PROTOCOL;
#endif  // defined(TCPIP_STACK_USE_TCP)
    gIperfState.stopRequested = false;

    gIperfState.mServerPort = TCPIP_IPERF_SERVER_PORT;		// -p. default: server port 5001

    gIperfState.mTxRate = ((uint32_t) 500)*((uint32_t) 1000);		// -b or -x. Target tx rate.
    // KS: default tx rate for iperf is actually 1Mbps. Here we set it to 500Kbps instead.

    gIperfState.mAmount = 0;			// -n: default 0.
    gIperfState.mDuration = ((uint32_t) 10)*((uint32_t) SYS_TMR_TickCounterFrequencyGet()); // -t: default 10 sec.
    gIperfState.mInterval =  SYS_TMR_TickCounterFrequencyGet(); 	// -i: default 1 sec.

    // remember the console we've been invoked from
    gIperfState.pCmdIO = pCmdIO;

    
    // Initialize statistics

    ResetIperfCounters();

    for (i = 1; i < argc; i++)
    {
        if ((memcmp(argv[i], "-s", 2) == 0) || (memcmp(argv[i], "--server", 5) == 0) )
        {
            // Function as an iperf server.

            gIperfState.mServerMode = true;
        }
#if defined(TCPIP_STACK_USE_UDP)
        else if ((memcmp(argv[i], "-u", 2) == 0) || (memcmp(argv[i], "--udp", 5) == 0) )
        {
            // iperf UDP mode.
            gIperfState.mProtocol = UDP_PROTOCOL;
        }
        else if ((memcmp(argv[i], "-b", 2) == 0) || (memcmp(argv[i], "--bandwidth", 5) == 0) )
        {
            // iperf UDP mode.
            gIperfState.mProtocol = UDP_PROTOCOL;

            // Next argument should be the target rate, in bps.
            i++;
            ptr = argv[i];

            ascii_to_u32s(ptr, values, 1);

            gIperfState.mTxRate = values[0];
        }
#endif  // defined(TCPIP_STACK_USE_UDP)
#if defined(TCPIP_STACK_USE_TCP)
        else if ((memcmp(argv[i], "-x", 2) == 0) || (memcmp(argv[i], "--xmitrate", 5) == 0) )
        {
            // NON-STANDARD IPERF OPTION. Set the max TCP tx rate.
            // Next argument should be the target rate, in bps.
            i++;
            ptr = argv[i];

            ascii_to_u32s(ptr, values, 1);

            gIperfState.mTxRate = values[0];
        }
#endif  // defined(TCPIP_STACK_USE_TCP)
        else if ((memcmp(argv[i], "-c", 2) == 0) || (memcmp(argv[i], "--client", 5) == 0) )
        {
            // Function as an iperf client.
            gIperfState.mServerMode = false;

            // Next argument should be the server IP, such as "192.168.1.100".
            i++;
            ptr = argv[i];
            ascii_to_u32s(ptr, values, 4);

            gIperfState.remoteSide.remoteIPaddress.v4Add.v[0] = values[0];
            gIperfState.remoteSide.remoteIPaddress.v4Add.v[1] = values[1];
            gIperfState.remoteSide.remoteIPaddress.v4Add.v[2] = values[2];
            gIperfState.remoteSide.remoteIPaddress.v4Add.v[3] = values[3]; 
        }
        else if ((memcmp(argv[i], "-t", 2) == 0) || (memcmp(argv[i], "--time", 5) == 0) )
        {
            // Next argument should be the (client tx) duration, in seconds.
            i++;
            ptr = argv[i];
            ascii_to_u32s(ptr, values, 1);

            gIperfState.mDuration = values[0]*SYS_TMR_TickCounterFrequencyGet();
            gIperfState.mAmount = 0;
        }
        else if ((memcmp(argv[i], "-n", 2) == 0) || (memcmp(argv[i], "--num", 5) == 0) )
        {
            // Next argument should be the (client tx) size, in bytes.
            i++;
            ptr = argv[i];
            ascii_to_u32s(ptr, values, 1);

            gIperfState.mAmount = values[0];
            gIperfState.mDuration = 0;
        }


#if defined(TCPIP_STACK_USE_TCP)
        else if ((memcmp(argv[i], "-M", 2) == 0) ||
                (memcmp(argv[i], "--mss", 5) == 0) )
        {
            // Next argument should be the (client tcp tx) MSS size, in bytes.

            i++;
            ptr = argv[i];

            ascii_to_u32s(ptr, values, 1);

            gIperfState.mMSS = values[0];
        }
#endif  // defined(TCPIP_STACK_USE_TCP)

        else if ((memcmp(argv[i], "-i", 2) == 0) || (memcmp(argv[i], "--interval", 5) == 0) )
        {
            // Next argument should be the report interval, in seconds.
            i++;
            ptr = argv[i];
            ascii_to_u32s(ptr, values, 1);

            gIperfState.mInterval = values[0]*SYS_TMR_TickCounterFrequencyGet(); // Convert to msec
        }
#if defined(TCPIP_STACK_USE_UDP)
        else if ((memcmp(argv[i], "-l", 2) == 0) || (memcmp(argv[i], "--len", 5) == 0) )
        {
            // Next argument should be the buffer length, in bytes.
            // This is used as the UDP datagram size.
            i++;
            ptr = argv[i];
            ascii_to_u32s(ptr, values, 1);

            if ( values[0] <  MAX_BUFFER  )
            {
               (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, "iperf: The minimum datagram size is %d\r\n", (int)MAX_BUFFER);
               return false;
            }

            gIperfState.mDatagramSize = values[0];
        }
#endif  // defined(TCPIP_STACK_USE_UDP)
    }

    switch (gIperfState.mServerMode)
    {
        case 0:
            // iperf client

            // set the interface to work on
            if((gIperfState.pNetIf = TCPIP_STACK_IPAddToNet(&gIperfState.localAddr, false)) == 0)
            {
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Using the default interface!\r\n");
                gIperfState.pNetIf = TCPIP_STACK_NetDefaultGet();
            }

#if defined(TCPIP_STACK_USE_TCP)
            if(gIperfState.mProtocol == TCP_PROTOCOL)
            {
                payloadSize = 	gIperfState.mMSS;
            }
#endif  // defined(TCPIP_STACK_USE_TCP)

#if defined(TCPIP_STACK_USE_UDP)
            if(gIperfState.mProtocol == UDP_PROTOCOL)
            {
                payloadSize = 	gIperfState.mDatagramSize;
            }
#endif  // defined(TCPIP_STACK_USE_UDP)

            pktRate =  (float) (gIperfState.mTxRate / 8) / (float) payloadSize;
            gIperfState.mPktPeriod =  (uint32_t) ( (float) SYS_TMR_TickCounterFrequencyGet() / pktRate );

            IperfSetState(IPERF_TX_START_STATE);
            break;

        case 1:
            // iperf server
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "---------------------------------------------------------\r\n");

            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: Server listening on ");
#if defined(TCPIP_STACK_USE_UDP)
            if (gIperfState.mProtocol == UDP_PROTOCOL)
            {
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, (const char *)"UDP");
            }    
#endif  // defined(TCPIP_STACK_USE_UDP)
#if defined(TCPIP_STACK_USE_TCP)
            if (gIperfState.mProtocol == TCP_PROTOCOL)
            {
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, (const char *)"TCP");
            }    
#endif  // defined(TCPIP_STACK_USE_TCP)

            (gIperfState.pCmdIO->pCmdApi->print)(cmdIoParam, " port %d\r\n", gIperfState.mServerPort);
            IperfSetState(IPERF_RX_START_STATE);
            break;
    }
    
    return true;
}

static int CommandIperfStop(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    const void* cmdIoParam = pCmdIO->cmdIoParam;
    if (argc > 1)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "iperf: too many args\r\n");
        return false;
    }

    if (gIperfState.state == IPERF_STANDBY_STATE)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "\r\niperf: not started yet!\r\n");
        return false;
    }

    gIperfState.stopRequested = true;
    (*pCmdIO->pCmdApi->msg)(cmdIoParam, "\r\niperf: trying to stop...\r\n");

    return true;
}

static int CommandIperfNetIf(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    IPV4_ADDR   ipAddr;
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if (argc != 2)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "iperfi usage: iperfi address\r\n");
        return false;
    }

    if (gIperfState.state != IPERF_STANDBY_STATE)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "\r\niperfi: cannot change the ip address while running!\r\n");
        return false;
    }


    // argument should be the IP address, such as "192.168.1.100".
    // use "0" for any interface (server mode only)
    if(!TCPIP_Helper_StringToIPAddress(argv[1], &ipAddr))
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "iperfi: use a valid IP address!\r\n");
        return false;
    }

    gIperfState.localAddr.Val = ipAddr.Val;
    (*pCmdIO->pCmdApi->msg)(cmdIoParam, "iperfi: OK, set the IP address\r\n");

    return true;


}

static int CommandIperfSize(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    bool        setTx;
    uint32_t    buffSize;

    const void* cmdIoParam = pCmdIO->cmdIoParam;


    if (argc != 3)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "iperfs usage: iperfs tx/rx size\r\n");
        return false;
    }

    if(strcmp(argv[1], "tx") == 0)
    {
        setTx = true;
    }
    else if(strcmp(argv[1], "rx") == 0)
    {
        setTx = false;
    }
    else
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "iperfs: enter tx or rx\r\n");
        return false;
    }

    buffSize = atoi(argv[2]);
    if(buffSize <= 0 || buffSize >= 65536)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "iperfs: 0 < size < 65536\r\n");
        return false;
    }

    if(setTx)
    {
        gIperfState.txBuffSize = buffSize;
    }
    else
    {
        gIperfState.rxBuffSize = buffSize;
    }

    (*pCmdIO->pCmdApi->print)(cmdIoParam, "iperfs: OK, set %s size to %d\r\n", argv[1], buffSize);

    return true;



}

static void IperfSetState(int newState)
{
    uint8_t oldState = gIperfState.state;

    if(newState == IPERF_STANDBY_STATE)
    {
        if(oldState != IPERF_STANDBY_STATE)
        {   // clear the async request
            _TCPIPStackModuleSignalGet(TCPIP_THIS_MODULE_ID, TCPIP_MODULE_SIGNAL_ASYNC); 
        }
    }
    else if (oldState == IPERF_STANDBY_STATE)
    {   // going busy; set the async request
        _TCPIPStackModuleSignalRequest(TCPIP_THIS_MODULE_ID, TCPIP_MODULE_SIGNAL_ASYNC, 0); 
    }

    gIperfState.state = (uint8_t)newState;

}
#endif  // defined(TCPIP_STACK_USE_IPERF)

