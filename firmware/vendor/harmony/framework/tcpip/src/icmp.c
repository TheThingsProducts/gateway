/*******************************************************************************
  Internet Control Message Protocol (ICMP) Server

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    - Provides "ping" diagnostics
    - Reference: RFC 792
*******************************************************************************/

/*******************************************************************************
File Name:  ICMP.c
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
#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_ICMP

#include "tcpip/src/tcpip_private.h"


#if defined(TCPIP_STACK_USE_IPV4)
#if defined(TCPIP_STACK_USE_ICMP_SERVER) || defined(TCPIP_STACK_USE_ICMP_CLIENT)


// ICMP Header Structure
typedef struct
{
    uint8_t vType;
    uint8_t vCode;
    uint16_t wChecksum;
    uint16_t wIdentifier;
    uint16_t wSequenceNumber;
} ICMP_HEADER;

#define ICMP_TYPE_ECHO_REQUEST      8   // ICMP server is requested echo - type
#define ICMP_CODE_ECHO_REQUEST      0   // ICMP server is requested echo - code


#define ICMP_TYPE_ECHO_REPLY        0   // ICMP client echo reply - type
#define ICMP_CODE_ECHO_REPLY        0   // ICMP client echo reply - code



static int                  icmpInitCount = 0;  // ICMP module initialization count

static const void*          icmpMemH = 0;       // memory handle

static tcpipSignalHandle    signalHandle = 0;   // registered signal handler   

#if defined(TCPIP_STACK_USE_ICMP_CLIENT)

// Callback function for informing the upper-layer protocols about ICMP events
typedef void (*icmpCallback) (TCPIP_NET_HANDLE hNetIf, IPV4_ADDR * remoteIP, void * data);

#if (TCPIP_ICMP_CLIENT_USER_NOTIFICATION != 0)
static PROTECTED_SINGLE_LIST      icmpRegisteredUsers = { {0} };
#endif  // (TCPIP_ICMP_CLIENT_USER_NOTIFICATION != 0)
//
// ICMP callback registration
typedef struct  _TAG_ICMP_LIST_NODE
{
	struct _TAG_ICMP_LIST_NODE* next;		// next node in list
                                            // makes it valid SGL_LIST_NODE node
    icmpCallback                callback;   // handler to be called for ICMP event
}ICMP_LIST_NODE;


// ICMP Packet Structure
typedef struct
{
	uint8_t vType;
	uint8_t vCode;
	uint16_t wChecksum;
	uint16_t wIdentifier;
	uint16_t wSequenceNumber;
	uint32_t wData;
} ICMP_PACKET;

#endif

// local prototypes
static bool _ICMPAckPacket(TCPIP_MAC_PACKET* pkt, const void* ackParam);
static IPV4_PACKET * _ICMPAllocateTxPacketStruct (uint16_t totICMPLen);
static TCPIP_MAC_PKT_ACK_RES _ICMPProcessEchoRequest(TCPIP_NET_IF* pNetIf, TCPIP_MAC_PACKET* pRxPkt, uint32_t destAdd, uint32_t srcAdd);
static void  TCPIP_ICMP_Process(void);
#if (TCPIP_STACK_DOWN_OPERATION != 0)
static void TCPIP_ICMP_Cleanup(void);
#else
#define TCPIP_ICMP_Cleanup()
#endif  // (TCPIP_STACK_DOWN_OPERATION != 0)

#if defined(TCPIP_STACK_USE_ICMP_CLIENT) && (TCPIP_ICMP_CLIENT_USER_NOTIFICATION != 0)
static void _ICMPNotifyClients(TCPIP_NET_HANDLE hNetIf, IPV4_ADDR * remoteIP, void * data);
#else
#define _ICMPNotifyClients(hNetIf, remoteIP, data)
#endif  // defined(TCPIP_STACK_USE_ICMP_CLIENT) && (TCPIP_ICMP_CLIENT_USER_NOTIFICATION != 0)


bool TCPIP_ICMP_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl, const TCPIP_ICMP_MODULE_CONFIG* const pIcmpInit)
{
    if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }

    // stack start up
    if(icmpInitCount == 0)
    {   // first time we're run
        bool iniRes;

        icmpMemH = stackCtrl->memH;
        while(true)
        {
            iniRes = (signalHandle =_TCPIPStackSignalHandlerRegister(TCPIP_THIS_MODULE_ID, TCPIP_ICMP_Task, 0)) != 0;
            if(iniRes == false)
            {
                break;
            }
#if defined(TCPIP_STACK_USE_ICMP_CLIENT) && (TCPIP_ICMP_CLIENT_USER_NOTIFICATION != 0)
            iniRes = TCPIP_Notification_Initialize(&icmpRegisteredUsers);
#endif  // defined(TCPIP_STACK_USE_ICMP_CLIENT) && (TCPIP_ICMP_CLIENT_USER_NOTIFICATION != 0)
            break;
        }
        if(iniRes == false)
        {
            TCPIP_ICMP_Cleanup();
            return false;
        }
    }

    // interface init
    // postpone packet allocation until the TCPIP_ICMP_EchoRequestSend is called
    //

    icmpInitCount++;
    return true;
}


#if (TCPIP_STACK_DOWN_OPERATION != 0)
void TCPIP_ICMP_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_DOWN) // interface down

    if(icmpInitCount > 0)
    {   // we're up and running
        // one way or another this interface is going down

        if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
        {   // stack shut down
            if(--icmpInitCount == 0)
            {   // all closed. release resources
                TCPIP_ICMP_Cleanup();
            }
        }
    }
}

static void TCPIP_ICMP_Cleanup(void)
{
#if defined(TCPIP_STACK_USE_ICMP_CLIENT) && (TCPIP_ICMP_CLIENT_USER_NOTIFICATION != 0)
    TCPIP_Notification_Deinitialize(&icmpRegisteredUsers, icmpMemH);
#endif  // defined(TCPIP_STACK_USE_ICMP_CLIENT) && (TCPIP_ICMP_CLIENT_USER_NOTIFICATION != 0)
    if(signalHandle)
    {
        _TCPIPStackSignalHandlerDeregister(signalHandle);
        signalHandle = 0;
    }
    icmpMemH = 0;
}
#endif  // (TCPIP_STACK_DOWN_OPERATION != 0)

#if defined(TCPIP_STACK_USE_ICMP_CLIENT) && (TCPIP_ICMP_CLIENT_USER_NOTIFICATION != 0)
ICMP_HANDLE TCPIP_ICMP_CallbackRegister (void (*callback)(TCPIP_NET_HANDLE hNetIf, IPV4_ADDR * remoteIP, void * data))
{
    if(callback && icmpMemH)
    {
        ICMP_LIST_NODE* newNode = (ICMP_LIST_NODE*)TCPIP_Notification_Add(&icmpRegisteredUsers, icmpMemH, sizeof(*newNode));
        if(newNode)
        {
            newNode->callback = callback;
            return newNode;
        }
    }

    return 0;
}


bool TCPIP_ICMP_CallbackDeregister(ICMP_HANDLE hIcmp)
{
    if(hIcmp && icmpMemH)
    {
        if(TCPIP_Notification_Remove((SGL_LIST_NODE*)hIcmp, &icmpRegisteredUsers, icmpMemH))
        {
            return true;
        }
    }

    return false;


}
#endif  // defined(TCPIP_STACK_USE_ICMP_CLIENT) && (TCPIP_ICMP_CLIENT_USER_NOTIFICATION != 0)



static IPV4_PACKET * _ICMPAllocateTxPacketStruct (uint16_t totICMPLen)
{
    IPV4_PACKET * ptrPacket;

    ptrPacket = (IPV4_PACKET*)TCPIP_PKT_SocketAlloc(sizeof(IPV4_PACKET), totICMPLen, 0, TCPIP_MAC_PKT_FLAG_ICMPV4 | TCPIP_MAC_PKT_FLAG_IPV4 | TCPIP_MAC_PKT_FLAG_TX);

    if (ptrPacket != 0)
    {
        TCPIP_PKT_PacketAcknowledgeSet(&ptrPacket->macPkt, _ICMPAckPacket, 0);
    }

    return ptrPacket;
}

// packet deallocation function
// packet was transmitted by the IP layer
static bool _ICMPAckPacket(TCPIP_MAC_PACKET* pkt, const void* ackParam)
{
    TCPIP_PKT_PacketFree(pkt);
    return false;
}

#if defined (TCPIP_STACK_USE_ICMP_CLIENT)
ICMP_ECHO_RESULT TCPIP_ICMP_EchoRequestSend (TCPIP_NET_HANDLE netH, IPV4_ADDR * targetAddr, uint16_t sequenceNumber, uint16_t identifier)
{
    IPV4_PACKET*    pTxPkt;
    ICMP_PACKET*    pICMPPkt;
    uint32_t        payload = 0x44332211;
    uint16_t        pktSize = sizeof(ICMP_PACKET);
    ICMP_ECHO_RESULT res;


    while(true)
    {
        // allocate TX packet
        pTxPkt = _ICMPAllocateTxPacketStruct(pktSize);
        if(pTxPkt == 0)
        {
            res = ICMP_ECHO_ALLOC_ERROR;
            break;
        }

        pICMPPkt = (ICMP_PACKET*)pTxPkt->macPkt.pTransportLayer;

        pICMPPkt->vType = ICMP_TYPE_ECHO_REQUEST; 
        pICMPPkt->vCode = ICMP_CODE_ECHO_REQUEST;
        pICMPPkt->wChecksum = 0x0000;
        pICMPPkt->wIdentifier = identifier;
        pICMPPkt->wSequenceNumber = sequenceNumber;
        pICMPPkt->wData = payload;
        pICMPPkt->wChecksum = TCPIP_Helper_CalcIPChecksum((uint8_t*)pICMPPkt, pktSize, 0);
        pTxPkt->destAddress.Val = targetAddr->Val;

        pTxPkt->netIfH = TCPIP_IPV4_SelectSourceInterface(netH, targetAddr, &pTxPkt->srcAddress, false);
        if(pTxPkt->netIfH == 0)
        {   // could not find an route
            res = ICMP_ECHO_ROUTE_ERROR;
            break;
        }

        pTxPkt->macPkt.pDSeg->segLen += pktSize;
        TCPIP_IPV4_PacketFormatTx(pTxPkt, IP_PROT_ICMP, pktSize);
        if(!TCPIP_IPV4_PacketTransmit(pTxPkt))
        {
            res = ICMP_ECHO_TRANSMIT_ERROR;
            break;
        }

        res = ICMP_ECHO_OK;
        break;
    }

    if(res < 0 && pTxPkt != 0)
    {
        TCPIP_PKT_PacketFree(&pTxPkt->macPkt);
    }

    return res;
}


#endif

void  TCPIP_ICMP_Task(void)
{
    TCPIP_MODULE_SIGNAL sigPend;

    sigPend = _TCPIPStackModuleSignalGet(TCPIP_THIS_MODULE_ID, TCPIP_MODULE_SIGNAL_MASK_ALL);

    if((sigPend & TCPIP_MODULE_SIGNAL_RX_PENDING) != 0)
    { //  RX signal occurred
        TCPIP_ICMP_Process();
    }

}

// Processes an ICMP packet and generates an echo reply, if requested
static void  TCPIP_ICMP_Process(void)
{
    TCPIP_NET_IF*           pNetIf;
    TCPIP_MAC_PACKET*       pRxPkt;
    IPV4_HEADER*            pIpv4Header;
    uint32_t                destAdd;
    uint32_t                srcAdd;
    ICMP_HEADER*            pRxHdr;
    uint16_t                icmpTotLength;
    uint16_t                checksum;
    TCPIP_MAC_PKT_ACK_RES   ackRes;



    // extract queued ICMP packets
    while((pRxPkt = _TCPIPStackModuleRxExtract(TCPIP_THIS_MODULE_ID)) != 0)
    {
        pNetIf = (TCPIP_NET_IF*)pRxPkt->pktIf;

        pRxHdr = (ICMP_HEADER*)pRxPkt->pTransportLayer;
        ackRes = TCPIP_MAC_PKT_ACK_RX_OK;

        pIpv4Header = (IPV4_HEADER*)pRxPkt->pNetLayer;
        destAdd = pIpv4Header->DestAddress.Val;
        srcAdd =  pIpv4Header->SourceAddress.Val;


        while(true)
        {
            icmpTotLength = pRxPkt->totTransportLen;

            if(icmpTotLength < sizeof(*pRxHdr))
            {
                ackRes = TCPIP_MAC_PKT_ACK_STRUCT_ERR;
                break;
            }

            // Validate the checksum
            // The checksum data includes the precomputed checksum in the header
            // so a valid packet will always have a checksum of 0x0000
            // 1st segment
            if((pRxPkt->pktFlags & TCPIP_MAC_PKT_FLAG_SPLIT) != 0)
            {
                checksum = TCPIP_Helper_PacketChecksum(pRxPkt, (uint8_t*)pRxHdr, icmpTotLength, 0);
            }
            else
            {
                checksum = TCPIP_Helper_CalcIPChecksum((uint8_t*)pRxHdr, icmpTotLength, 0);
            }

            if(checksum != 0)
            {
                ackRes = TCPIP_MAC_PKT_ACK_CHKSUM_ERR;
                break;
            }

            if(pRxHdr->vType == ICMP_TYPE_ECHO_REQUEST && pRxHdr->vCode == ICMP_CODE_ECHO_REQUEST)
            {   // echo request
                ackRes = _ICMPProcessEchoRequest(pNetIf, pRxPkt, destAdd, srcAdd);
                break;
            }

#if defined(TCPIP_STACK_USE_ICMP_CLIENT)
            if(pRxHdr->vType == ICMP_TYPE_ECHO_REPLY && pRxHdr->vCode == ICMP_CODE_ECHO_REPLY)
            {   // echo reply; check if our own
                // Get the sequence number and identifier fields
#if (TCPIP_ICMP_CLIENT_USER_NOTIFICATION != 0)
                TCPIP_UINT32_VAL userData;
                IPV4_ADDR remoteIPAddr;

                userData.w[0] = pRxHdr->wIdentifier;
                userData.w[1] = pRxHdr->wSequenceNumber;
                remoteIPAddr.Val = srcAdd;

                // Send a message to the application-level Ping driver that we've received an Echo Reply
                _ICMPNotifyClients(pNetIf, &remoteIPAddr, (void *)userData.v);
#endif  // (TCPIP_ICMP_CLIENT_USER_NOTIFICATION != 0)
                ackRes = TCPIP_MAC_PKT_ACK_RX_OK;
                break;
            }
#endif

            // unknown type
            ackRes = TCPIP_MAC_PKT_ACK_TYPE_ERR;
            break;
        }


        TCPIP_PKT_FlightLog(pRxPkt, TCPIP_THIS_MODULE_ID, ackRes, 0);
        TCPIP_PKT_PacketAcknowledge(pRxPkt, ackRes); 
    }
}


// echo request
static TCPIP_MAC_PKT_ACK_RES _ICMPProcessEchoRequest(TCPIP_NET_IF* pNetIf, TCPIP_MAC_PACKET* pRxPkt, uint32_t destAdd, uint32_t srcAdd)
{
    ICMP_HEADER            *pTxHdr;
    IPV4_PACKET            *pTxPkt;
    TCPIP_UINT16_VAL        checksum;
    int16_t                dataToCopy;

    // allocate TX packet
    dataToCopy = pRxPkt->totTransportLen;
    pTxPkt = _ICMPAllocateTxPacketStruct(dataToCopy);
    if(pTxPkt != 0)
    {   // succeeded
        uint8_t* pStartAdd = pRxPkt->pTransportLayer;
        TCPIP_Helper_PacketCopy(pRxPkt, pTxPkt->macPkt.pTransportLayer, &pStartAdd, dataToCopy, true);

        // set the correct vType, vCode, and Checksum
        pTxHdr = (ICMP_HEADER*)pTxPkt->macPkt.pTransportLayer;

        pTxHdr->vType = ICMP_TYPE_ECHO_REPLY;
        pTxHdr->vCode = ICMP_CODE_ECHO_REPLY;
        checksum.Val = pTxHdr->wChecksum;
        checksum.v[0] += 8;	// Subtract 0x0800 from the checksum
        if(checksum.v[0] < 8u)
        {
            checksum.v[1]++;
            if(checksum.v[1] == 0u)
            {
                checksum.v[0]++;
            }
        }

        pTxHdr->wChecksum = checksum.Val;
        pTxPkt->srcAddress.Val = destAdd;
        pTxPkt->destAddress.Val = srcAdd;
        pTxPkt->netIfH = pNetIf;

        pTxPkt->macPkt.pDSeg->segLen += pRxPkt->totTransportLen;
        TCPIP_IPV4_PacketFormatTx(pTxPkt, IP_PROT_ICMP, pRxPkt->totTransportLen);
        if(!TCPIP_IPV4_PacketTransmit(pTxPkt))
        {
            TCPIP_PKT_PacketFree(&pTxPkt->macPkt);
        }
    }

    return TCPIP_MAC_PKT_ACK_RX_OK;
}


#if defined(TCPIP_STACK_USE_ICMP_CLIENT) && (TCPIP_ICMP_CLIENT_USER_NOTIFICATION != 0)

static void _ICMPNotifyClients(TCPIP_NET_HANDLE hNetIf, IPV4_ADDR * remoteIP, void * data)
{
    ICMP_LIST_NODE* dNode;

    TCPIP_Notification_Lock(&icmpRegisteredUsers);
    for(dNode = (ICMP_LIST_NODE*)icmpRegisteredUsers.list.head; dNode != 0; dNode = dNode->next)
    {
        (*dNode->callback)(hNetIf, remoteIP, data);
    }
    TCPIP_Notification_Unlock(&icmpRegisteredUsers);

}

#endif  // defined(TCPIP_STACK_USE_ICMP_CLIENT) && (TCPIP_ICMP_CLIENT_USER_NOTIFICATION != 0)

#endif //#if defined(TCPIP_STACK_USE_ICMP_SERVER) || defined(TCPIP_STACK_USE_ICMP_CLIENT)
#endif  // defined(TCPIP_STACK_USE_IPV4)


