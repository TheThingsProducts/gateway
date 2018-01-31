/*******************************************************************************
  Internet Protocol (IP) Version 4 Communications Layer

  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    -Provides a transport for TCP, UDP, and ICMP messages
    -Reference: RFC 791
*******************************************************************************/

/*******************************************************************************
File Name:  ipv4.c
Copyright © 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY KIND,
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

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_IPV4

#include "tcpip/src/tcpip_private.h"
#include "tcpip/src/ipv4_private.h"

#if defined(TCPIP_STACK_USE_IPV4)


// This is left shifted by 4.  Actual value is 0x04.
#define IPv4_VERSION        (0x40u)

// IHL (Internet Header Length) is # of 32 bit words in a header.
// Since, we do not support options, our IP header length will be
// minimum i.e. 20 bytes : IHL = 20 / 4 = 5.
#define IP_IHL              (0x05)

#define IP_SERVICE_NW_CTRL  (0x07)
#define IP_SERVICE_IN_CTRL  (0x06)
#define IP_SERVICE_ECP      (0x05)
#define IP_SERVICE_OVR      (0x04)
#define IP_SERVICE_FLASH    (0x03)
#define IP_SERVICE_IMM      (0x02)
#define IP_SERVICE_PRIOR    (0x01)
#define IP_SERVICE_ROUTINE  (0x00)

#define IP_SERVICE_N_DELAY  (0x00)
#define IP_SERCICE_L_DELAY  (0x08)
#define IP_SERVICE_N_THRPT  (0x00)
#define IP_SERVICE_H_THRPT  (0x10)
#define IP_SERVICE_N_RELIB  (0x00)
#define IP_SERVICE_H_RELIB  (0x20)

#define IP_SERVICE          (IP_SERVICE_ROUTINE | IP_SERVICE_N_DELAY)

#if defined(TCPIP_STACK_USE_ZEROCONF_MDNS_SD)
  #define MY_IP_TTL           (255)  // Time-To-Live in hops 
  // IP TTL is set to 255 for Multicast DNS compatibility. See mDNS-draft-08, section 4.
#else
  #define MY_IP_TTL           (100)  // Time-To-Live in hops
#endif



static uint16_t             ipv4Identifier = 0;           // Static identifier value for IPv4 headers
static const void*          ipv4MemH = 0;                 // memory handle

static tcpipSignalHandle    signalHandle = 0;

static PROTECTED_SINGLE_LIST ipv4ArpQueue = { {0} };       // queue of packets waiting for ARP resolution

static TCPIP_ARP_HANDLE     ipv4ArpHandle = 0;          // ARP registration handle

static uint16_t             ipv4InitCount = 0;

static PROTECTED_SINGLE_LIST ipv4PacketFilters = { {0} };

static TCPIP_IPV4_FILTER_TYPE ipv4FilterType = 0;       // IPv4 current filter

typedef enum
{
    TCPIP_IPV4_DEST_FAIL    = 0,    // cannot find a destination
    TCPIP_IPV4_DEST_SELF    = 1,    // route it internally
    TCPIP_IPV4_DEST_NETWORK = 2,    // route it externally

}TCPIP_IPV4_DEST_TYPE;



/************************************************************************/
/****************               Prototypes               ****************/
/************************************************************************/


static void TCPIP_IPV4_ArpHandler(TCPIP_NET_HANDLE hNet, const IPV4_ADDR* ipAdd, const TCPIP_MAC_ADDR* MACAddr, TCPIP_ARP_EVENT_TYPE evType, const void* param);

static void SwapIPV4Header(IPV4_HEADER* h);

static bool TCPIP_IPV4_VerifyPkt(TCPIP_NET_IF* pNetIf, TCPIP_MAC_PACKET* pRxPkt);

static bool TCPIP_IPV4_VerifyPktFilters(TCPIP_MAC_PACKET* pRxPkt);

static TCPIP_IPV4_DEST_TYPE TCPIP_IPV4_PktMacDestination(IPV4_PACKET* pPkt, const IPV4_ADDR* pIpAdd, TCPIP_MAC_ADDR** ppMacAdd);

static void TCPIP_IPV4_Process(void);
#if defined(TCPIP_IPV4_DEBUG)
void TCPIP_IPV4_CheckPkt(TCPIP_MAC_PACKET* pRxPkt); 
#endif  // defined(TCPIP_IPV4_DEBUG)

#if (TCPIP_STACK_DOWN_OPERATION != 0)
static void TCPIP_IPV4_Cleanup(void);
static void TCPIP_IPV4_ListPurge(PROTECTED_SINGLE_LIST* pList, TCPIP_NET_IF* pNetIf);
#else
#define TCPIP_IPV4_Cleanup()
#endif  // (TCPIP_STACK_DOWN_OPERATION != 0)

/*****************************************************************************
  Function:
	bool TCPIP_IPV4_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackInit, 
        const TCPIP_IPV4_MODULE_CONFIG* pIpInit)

  Summary:
	Initializes the IP module.

  Description:
	Initializes the IP module.  Sets the dynamic heap used by this module.	

  Precondition:
	None

  Parameters:
	stackInit - Stack initialization parameters
    pIpInit - Unused supplementary data.

  Returns:
  	true
  	
  Remarks:
	None
  ***************************************************************************/
bool TCPIP_IPV4_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackInit, const TCPIP_IPV4_MODULE_CONFIG* pIpInit)
{
    bool iniRes;

    if(stackInit->stackAction == TCPIP_STACK_ACTION_IF_UP)
    {   // interface restart
        return true;
    }

    if(ipv4InitCount == 0)
    {
        while(true)
        {
            ipv4MemH = stackInit->memH;
            ipv4ArpHandle = 0;

            iniRes = (signalHandle =_TCPIPStackSignalHandlerRegister(TCPIP_THIS_MODULE_ID, TCPIP_IPV4_Task, 0)) != 0;
            if(iniRes == false)
            {
                break;
            }

            if((iniRes = TCPIP_Helper_ProtectedSingleListInitialize (&ipv4ArpQueue)) == false)
            {
                break;
            }

            iniRes = TCPIP_Notification_Initialize(&ipv4PacketFilters);

            break;
        }
        
        if(iniRes == false)
        {
            TCPIP_IPV4_Cleanup();
            return false;
        }
    }

    ipv4InitCount++;

    return true;
}

/*****************************************************************************
  Function:
	void TCPIP_IPV4_DeInitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)

  Summary:
	Deinitializes the IP module.

  Description:
	Deinitializes the IP module.	

  Precondition:
	None

  Parameters:
	stackCtrl - Stack control data

  Returns:
  	None
  	
  Remarks:
	None
  ***************************************************************************/
#if (TCPIP_STACK_DOWN_OPERATION != 0)
void TCPIP_IPV4_DeInitialize(const TCPIP_STACK_MODULE_CTRL* const stackCtrl)
{
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT) // stack shut down
    // if(stackCtrl->stackAction == TCPIP_STACK_ACTION_IF_DOWN) // interface down

    if(ipv4InitCount > 0)
    {   // up and running
        // one way or another this interface is going down
        TCPIP_IPV4_ListPurge(&ipv4ArpQueue, stackCtrl->pNetIf);

        if(stackCtrl->stackAction == TCPIP_STACK_ACTION_DEINIT)
        {   // stack shut down

            if (--ipv4InitCount == 0)
            {   // free the ARP resources
                TCPIP_IPV4_Cleanup();
            }
        }
    }
}

static void TCPIP_IPV4_Cleanup(void)
{
    if(ipv4ArpHandle)
    {
        TCPIP_ARP_HandlerDeRegister(ipv4ArpHandle);
        ipv4ArpHandle = 0;
    }

    TCPIP_Notification_Deinitialize(&ipv4PacketFilters, ipv4MemH);
    ipv4MemH = 0;

    TCPIP_Helper_ProtectedSingleListDeinitialize(&ipv4ArpQueue);
    if(signalHandle)
    {
        _TCPIPStackSignalHandlerDeregister(signalHandle);
        signalHandle = 0;
    }

}

// purges a list of IPV4_PACKET packets
// searches for pNetIf match
// if pNetIf == 0 everything matches

static void TCPIP_IPV4_ListPurge(PROTECTED_SINGLE_LIST* pList, TCPIP_NET_IF* pNetIf)
{
    SINGLE_LIST newList;
    IPV4_PACKET *pPkt, *pNext;

    TCPIP_Helper_SingleListInitialize (&newList);

    TCPIP_Helper_ProtectedSingleListLock(pList);
    // traverse the list
    // and find all the packets matching the pNetIf

    pNext = (IPV4_PACKET*)pList->list.head;

    while((pPkt = pNext) != 0)
    {
        pNext = (IPV4_PACKET*)pPkt->macPkt.next;
            
        if(pNetIf == 0 || pNetIf == (TCPIP_NET_IF*)pPkt->netIfH)
        {   // match
            TCPIP_PKT_PacketAcknowledge(&pPkt->macPkt, TCPIP_MAC_PKT_ACK_ARP_NET_ERR);
        }
        else
        {
            TCPIP_Helper_SingleListTailAdd(&newList, (SGL_LIST_NODE*)pPkt);
        }
    }

    // update the arp queue
    pList->list = newList;
    TCPIP_Helper_ProtectedSingleListUnlock(pList);
}

#endif  // (TCPIP_STACK_DOWN_OPERATION != 0)

// selects a source address and an interface based on the IPv4 destination address
// updates the pSrcAddress and returns the needed interface, if successful
// returns 0 if failed
// if srcSet == 1 and netH != 0 the function won't change anything 
// if srcSet == 1 and netH == 0 then the call will never fail 
//      it will use whatever value in pSrcAddress (even 0)
//      and will try to come up with an appropriate interface.
// if srcSet == 0 and netH == 0 it will use the destination address
// if srcSet == 0 and netH != 0 it will use the address of that interface
// returns a valid interface if it succeeds, 0 otherwise
// Condition: netH has to be valid (if non 0)
// The interface is checked to be valid, up and linked
TCPIP_NET_HANDLE TCPIP_IPV4_SelectSourceInterface(TCPIP_NET_HANDLE netH, IPV4_ADDR* pDestAddress, IPV4_ADDR* pSrcAddress, bool srcSet)
{
    int netIx;
    int avlblInterfaces;
    uint32_t    ifAdd, ifMask;
    TCPIP_NET_IF  *pIf;

    pIf = _TCPIPStackHandleToNetUp(netH);

    if(srcSet)
    {   // address in pSrcAddress is valid, no matter what
        if(pIf == 0)
        {   // try to see if it belongs to some interface
            pIf = _TCPIPStackIpAddFromAnyNet(0, pSrcAddress);
            if(pIf == 0)
            {
                pIf = _TCPIPStackAnyNetLinked(true);
            }
        }
        return pIf;
    }

    if(pIf != 0)
    {   // interface is forced
        pSrcAddress->Val = TCPIP_STACK_NetAddressGet(pIf);
        return pIf;
    }

    // use the destination address to decide which interface is this packet going out on

    if(pDestAddress == 0 || pDestAddress->Val == 0)
    {   // unroutable
        return 0;
    }

    if((avlblInterfaces = TCPIP_STACK_NumberOfNetworksGet()) > 1)
    {   // we have a choice
        for(netIx = 0; netIx < avlblInterfaces; netIx++)
        {
            pIf = _TCPIPStackHandleToNetLinked(TCPIP_STACK_IndexToNet(netIx));
            if(pIf)
            {
                ifAdd = TCPIP_STACK_NetAddressGet(pIf);
                ifMask = TCPIP_STACK_NetMaskGet(pIf);

                if((ifAdd & ifMask) == (pDestAddress->Val & ifMask))
                {   // destination address is on this interface
                    pSrcAddress->Val = ifAdd;
                    return pIf;
                }
            }
        }
    }

    // use the default/any interface
    pIf = _TCPIPStackAnyNetLinked(true);
    if(pIf)
    {
        pSrcAddress->Val = TCPIP_STACK_NetAddressGet(pIf);
    }
    return pIf;
}



// transmits a packet over the network
bool TCPIP_IPV4_PacketTransmit(IPV4_PACKET* pPkt)
{
    TCPIP_MAC_ADDR   destMacAdd, *pMacDst;
    TCPIP_NET_IF*    pNetIf;
    TCPIP_IPV4_DEST_TYPE destType;
    
    if(pPkt->macPkt.next != 0)
    {   // no support for chained packets!
        return false;
    }

    // make sure the interface is valid
    pNetIf = _TCPIPStackHandleToNetLinked(pPkt->netIfH);
    if(pNetIf == 0)
    {   // cannot transmit over dead interface
        return false;
    }

    if(_TCPIPStackIsConfig(pNetIf) && (pPkt->macPkt.pktFlags & TCPIP_MAC_PKT_FLAG_CONFIG) == 0)
    {   // no packets go out in stack configuration
       return false;
    } 

    // now select packet's destination MAC address
    pMacDst = &destMacAdd;
    destType = TCPIP_IPV4_PktMacDestination(pPkt, &pPkt->destAddress, &pMacDst);
    if(destType == TCPIP_IPV4_DEST_FAIL) 
    {   // discard, cannot send
        return false;
    }

    pNetIf = (TCPIP_NET_IF*)pPkt->netIfH;   // re-read; could have changed
    if(!TCPIP_PKT_PacketMACFormat(&pPkt->macPkt, pMacDst, (const TCPIP_MAC_ADDR*)TCPIP_STACK_NetMACAddressGet(pNetIf), TCPIP_ETHER_TYPE_IPV4))
    {   // discard, cannot send
        return false;
    }

    if(destType == TCPIP_IPV4_DEST_SELF)
    {
        pPkt->macPkt.pktFlags |= TCPIP_MAC_PKT_FLAG_UNICAST;
        _TCPIPStackInsertRxPacket(pNetIf, &pPkt->macPkt, true);
        return true;
    }


    // TCPIP_IPV4_DEST_NETWORK 
    if(pMacDst == 0)
    {   // queue it
        if(ipv4ArpHandle == 0)
        {
            if((ipv4ArpHandle = TCPIP_ARP_HandlerRegister(0, TCPIP_IPV4_ArpHandler, 0)) == 0)
            {
                SYS_ERROR(SYS_ERROR_WARNING, "IPv4: Failed to register ARP notification! \r\n");
                return false;
            }
        }

        TCPIP_Helper_ProtectedSingleListTailAdd(&ipv4ArpQueue, (SGL_LIST_NODE*)pPkt);
        // ARP notification will be received: either TMO or resolved
        // mark packet as queued 
        pPkt->macPkt.pktFlags |= TCPIP_MAC_PKT_FLAG_QUEUED;
        return true;
    }

  
    if(_TCPIPStackPacketTx(pNetIf, &pPkt->macPkt) >= 0)
    {   // MAC sets itself the TCPIP_MAC_PKT_FLAG_QUEUED
        return true;
    }

    // failed
    return false;

}



// IPv4 formats a IPV4_PACKET and calculates the header checksum
// the source and destination addresses should be updated in the packet 
void TCPIP_IPV4_PacketFormatTx(IPV4_PACKET* pPkt, uint8_t protocol, uint16_t ipLoadLen)
{
    IPV4_HEADER*    pHdr = (IPV4_HEADER*)pPkt->macPkt.pNetLayer;

    pHdr->VersionIHL       = IP_IHL | IPv4_VERSION;
    pHdr->TypeOfService    = IP_SERVICE;
    pHdr->TotalLength = TCPIP_Helper_htons(sizeof(IPV4_HEADER) + ipLoadLen);
    pHdr->Identification   = ++ipv4Identifier;
    pHdr->FragmentInfo     = 0;
    pHdr->TimeToLive       = MY_IP_TTL;
    pHdr->Protocol         = protocol;
    pHdr->HeaderChecksum   = 0;
    pHdr->SourceAddress.Val = pPkt->srcAddress.Val;
    pHdr->DestAddress.Val = pPkt->destAddress.Val;
    // update the checksum
    pHdr->HeaderChecksum   = TCPIP_Helper_CalcIPChecksum((uint8_t*)pHdr, sizeof(IPV4_HEADER), 0);

    pPkt->macPkt.pDSeg->segLen += sizeof(IPV4_HEADER);
}
	

// ARP resolution done
static void TCPIP_IPV4_ArpHandler(TCPIP_NET_HANDLE hNet, const IPV4_ADDR* ipAdd, const TCPIP_MAC_ADDR* MACAddr, TCPIP_ARP_EVENT_TYPE evType, const void* param)
{
    TCPIP_NET_IF* pPktIf;
    SINGLE_LIST newList;
    IPV4_PACKET *pPkt, *pNext;
    TCPIP_MAC_PKT_ACK_RES   pktAckFail;
    TCPIP_MAC_ETHERNET_HEADER* macHdr;
    

    TCPIP_Helper_SingleListInitialize (&newList);
    
    TCPIP_Helper_ProtectedSingleListLock(&ipv4ArpQueue);
    // traverse the ipv4ArpQueue list
    // and find all the packets waiting for the solved address

    pNext = (IPV4_PACKET*)ipv4ArpQueue.list.head;

    while((pPkt = pNext) != 0)
    {
        pNext = (IPV4_PACKET*)pPkt->macPkt.next;
        pPkt->macPkt.next = 0;  // send single packet
            
        pktAckFail = TCPIP_MAC_PKT_ACK_NONE; 
        if(pPkt->arpTarget.Val == ipAdd->Val)
        {   // match
            if(evType >= 0)
            {   // successfully resolved the ARP; update the packet destination
                macHdr = (TCPIP_MAC_ETHERNET_HEADER*)pPkt->macPkt.pMacLayer;
                memcpy(&macHdr->DestMACAddr, MACAddr, sizeof(*MACAddr));
                pPktIf = _TCPIPStackHandleToNet(pPkt->netIfH);
                if(pPktIf == 0 || _TCPIPStackPacketTx(pPktIf, &pPkt->macPkt) < 0)
                {
                    pktAckFail = TCPIP_MAC_PKT_ACK_ARP_NET_ERR; 
                }
            }
            else
            {   // some error;
                pktAckFail = TCPIP_MAC_PKT_ACK_ARP_TMO; 
            }

            if(pktAckFail != TCPIP_MAC_PKT_ACK_NONE)
            {   // some error; discard the packet
                TCPIP_PKT_PacketAcknowledge(&pPkt->macPkt, pktAckFail);
            }
        }
        else
        {
            TCPIP_Helper_SingleListTailAdd(&newList, (SGL_LIST_NODE*)pPkt);
        }
    }

    // update the arp queue
    ipv4ArpQueue.list = newList;
    TCPIP_Helper_ProtectedSingleListUnlock(&ipv4ArpQueue);

}

void  TCPIP_IPV4_Task(void)
{
    TCPIP_MODULE_SIGNAL sigPend;

    sigPend = _TCPIPStackModuleSignalGet(TCPIP_THIS_MODULE_ID, TCPIP_MODULE_SIGNAL_MASK_ALL);

    if((sigPend & TCPIP_MODULE_SIGNAL_RX_PENDING) != 0)
    { // RX signal occurred
        TCPIP_IPV4_Process();
    }

}

// processes an incoming IPV4 packet
static void TCPIP_IPV4_Process(void)
{
    TCPIP_NET_IF* pNetIf;
    TCPIP_MAC_PACKET* pRxPkt;
    uint8_t      headerLen;
    uint16_t     headerChecksum;
    IPV4_HEADER* pHeader;
    uint8_t      ipFrameType;
    bool         pktAccepted;
    TCPIP_MAC_PKT_ACK_RES ackRes;
    TCPIP_STACK_MODULE destId;

    // extract queued IPv4 packets
    while((pRxPkt = _TCPIPStackModuleRxExtract(TCPIP_THIS_MODULE_ID)) != 0)
    {
        pNetIf = (TCPIP_NET_IF*)pRxPkt->pktIf;

        while(true)
        {
            ackRes = TCPIP_MAC_PKT_ACK_NONE;
            pHeader = (IPV4_HEADER*)pRxPkt->pNetLayer;

            // Make sure that this is an IPv4 packet.
            if((pHeader->VersionIHL & 0xf0) != IPv4_VERSION)
            {
                ackRes = TCPIP_MAC_PKT_ACK_STRUCT_ERR;
                break;
            }

            // Throw this packet away if it is a fragment.  
            // We don't support IPv4 fragment reconstruction.
            if(pHeader->FragmentInfo & 0xFF3F)
            {
                ackRes = TCPIP_MAC_PKT_ACK_STRUCT_ERR;
                break;
            }

            // make sure the header length is within packet limits
            headerLen = (pHeader->VersionIHL & 0x0f) << 2;
            if(headerLen > pRxPkt->pDSeg->segLen)
            {
                ackRes = TCPIP_MAC_PKT_ACK_STRUCT_ERR;
                break;
            }

            pRxPkt->pTransportLayer = pRxPkt->pNetLayer + headerLen;
            pRxPkt->pDSeg->segLen -= headerLen;

            // Check the packet arrived on the proper interface and passes the filters
            pktAccepted = TCPIP_IPV4_VerifyPkt(pNetIf, pRxPkt);
            if(!pktAccepted)
            {
                pktAccepted = TCPIP_IPV4_VerifyPktFilters(pRxPkt);
            }

#if defined(TCPIP_IPV4_DEBUG)
        TCPIP_IPV4_CheckPkt(pRxPkt);
#endif  // defined(TCPIP_IPV4_DEBUG)

            if(!pktAccepted)
            {   // discard
                ackRes = TCPIP_MAC_PKT_ACK_SOURCE_ERR;
                break;
            }

            // Validate the IP header.  If it is correct, the checksum 
            // will come out to 0x0000 (because the header contains a 
            // precomputed checksum).  A corrupt header will have a 
            // nonzero checksum.
            headerChecksum = TCPIP_Helper_CalcIPChecksum((uint8_t*)pHeader, headerLen, 0);

            if(headerChecksum)
            {
                // Bad packet. The function caller will be notified by means of the false 
                // return value and it should discard the packet.
                ackRes = TCPIP_MAC_PKT_ACK_CHKSUM_ERR;
                break;
            }

            // Network to host conversion.
            SwapIPV4Header(pHeader);

            // valid IPv4 packet
            ipFrameType = pHeader->Protocol;
            pRxPkt->totTransportLen = pHeader->TotalLength - headerLen;

            // seems a valid IPv4 packet
            // check where it needs to go

            switch(ipFrameType)
            {
#if defined(TCPIP_STACK_USE_ICMP_SERVER) || defined(TCPIP_STACK_USE_ICMP_CLIENT)
                case IP_PROT_ICMP:
                    destId = TCPIP_MODULE_ICMP;
                    break;
#endif

#if defined(TCPIP_STACK_USE_TCP)
                case IP_PROT_TCP:
                    destId = TCPIP_MODULE_TCP;
                    break;
#endif

#if defined(TCPIP_STACK_USE_UDP)
                case IP_PROT_UDP:
                    destId = TCPIP_MODULE_UDP;
                    break;
#endif

                default:
                    // unknown/unsupported; discard
                    ackRes = TCPIP_MAC_PKT_ACK_PROTO_DEST_ERR; 
                    break;
            }

            break;
        }


        if(ackRes != TCPIP_MAC_PKT_ACK_NONE)
        {   // unknown/unsupported; discard
            TCPIP_PKT_FlightLog(pRxPkt, TCPIP_MODULE_NONE, ackRes, 0);
            TCPIP_PKT_PacketAcknowledge(pRxPkt, ackRes); 
        }
        else
        {   // forward this packet and signal
            TCPIP_PKT_FlightLog(pRxPkt, destId, TCPIP_MAC_PKT_ACK_RX_OK, 0);
            _TCPIPStackModuleRxInsert(destId, pRxPkt, true);
        }
    }

}


// Swaps the endinaness of parameters in an IPv4 header
static void SwapIPV4Header(IPV4_HEADER* h)
{
    h->TotalLength      = TCPIP_Helper_ntohs(h->TotalLength);
    h->Identification   = TCPIP_Helper_ntohs(h->Identification);
    h->HeaderChecksum   = TCPIP_Helper_ntohs(h->HeaderChecksum);
}

/*
 Discard packets that are received on the wrong interface
 (have an IP address that doesn't match the interface we
 received the packet on) or are rejected by the current IPv4 packet filters.
 Returns: true if valid packet
          false otherwise.
*/                  
static bool TCPIP_IPV4_VerifyPkt(TCPIP_NET_IF* pNetIf, TCPIP_MAC_PACKET* pRxPkt)
{
    // process first the registered filters
    IPV4_HEADER* pHeader;
    IPV4_ADDR* pktDestIP;
    TCPIP_IPV4_FILTER_TYPE currFilter; // current filter

    if(TCPIP_STACK_NetworkIsUp(pNetIf))
    {   // standard IPv4 filtering
        pHeader = (IPV4_HEADER*)pRxPkt->pNetLayer;
        pktDestIP = &pHeader->DestAddress;
        currFilter = ipv4FilterType;

        if(TCPIP_STACK_AddressIsOfNet(pNetIf, pktDestIP))
        {   // unicast to me
            return (currFilter & TCPIP_IPV4_FILTER_UNICAST) == 0 ? true : false;
        }

        if(TCPIP_STACK_IsBcastAddress(pNetIf, pktDestIP))
        {   // net or limited bcast
            return (currFilter & TCPIP_IPV4_FILTER_BROADCAST) == 0 ? true : false;
        }

        if(TCPIP_Helper_IsMcastAddress(pktDestIP))
        {   // multicast
            return (currFilter & TCPIP_IPV4_FILTER_MULTICAST) == 0 ? true : false;
        }

        // else: unknown packet type
    }

    return false;

}



static bool TCPIP_IPV4_VerifyPktFilters(TCPIP_MAC_PACKET* pRxPkt)
{
    // process the registered filters
    IPV4_FILTER_LIST_NODE* fNode;
    bool pktOk = false;

    TCPIP_Notification_Lock(&ipv4PacketFilters);
    for(fNode = (IPV4_FILTER_LIST_NODE*)ipv4PacketFilters.list.head; fNode != 0; fNode = fNode->next)
    {
        if((*fNode->handler)(pRxPkt, fNode->hParam))
        {   // packet accepted
            pktOk = true;
            break;
        }
    }
    TCPIP_Notification_Unlock(&ipv4PacketFilters);
        
    return pktOk;
}

IPV4_FILTER_HANDLE IPv4RegisterFilter(IPV4_FILTER_FUNC handler, const void* hParam)
{
    if(ipv4MemH && handler)
    {
        IPV4_FILTER_LIST_NODE* newNode = (IPV4_FILTER_LIST_NODE*)TCPIP_Notification_Add(&ipv4PacketFilters, ipv4MemH, sizeof(*newNode));
        if(newNode)
        {
            newNode->handler = handler;
            newNode->hParam = hParam;
            return newNode;
        }
    }

    return 0;

}

// deregister the filter handler
// returns true or false if no such handler registered
bool Ipv4DeRegisterFilter(IPV4_FILTER_HANDLE hFilter)
{
    if(hFilter && ipv4MemH)
    {
        if(TCPIP_Notification_Remove((SGL_LIST_NODE*)hFilter, &ipv4PacketFilters, ipv4MemH))
        {
            return true;
        }
    }

    return false;
}

// select destination MAC address
// ppMacAdd is set to 0 if the MAC address is not available yet
static TCPIP_IPV4_DEST_TYPE TCPIP_IPV4_PktMacDestination(IPV4_PACKET* pPkt, const IPV4_ADDR* pIpAdd, TCPIP_MAC_ADDR** ppMacAdd)
{
    TCPIP_ARP_RESULT  arpRes;
    TCPIP_NET_IF*     pHostIf;
    TCPIP_MAC_ADDR*   pMacDst = *ppMacAdd;
    TCPIP_NET_IF* pNetIf = (TCPIP_NET_IF*)pPkt->netIfH;

    if(TCPIP_STACK_AddressIsOfNet(pNetIf, pIpAdd))
    {   // localhost address
        memcpy(pMacDst, TCPIP_STACK_NetMACAddressGet(pNetIf), sizeof(*pMacDst));
        return TCPIP_IPV4_DEST_SELF; 
    }
    else if((pHostIf = TCPIP_STACK_NetByAddress(pIpAdd)))
    {   // localhost address
        memcpy(pMacDst, TCPIP_STACK_NetMACAddressGet(pHostIf), sizeof(*pMacDst));
        pPkt->netIfH = pHostIf;
        return TCPIP_IPV4_DEST_SELF; 
    }

    if(pIpAdd->Val == 0xffffffff || pIpAdd->Val == TCPIP_STACK_NetAddressBcast(pNetIf))
    {
        memset(pMacDst, 0xff, sizeof(*pMacDst));
    }
    // check ZConf address range
    // IP multicast address range from 224.0.0.0 to 239.255.255.255
    // can be done locally; No need for an ARP request.
    else if ((pIpAdd->v[0] >= 224) && (pIpAdd->v[0] <= 239))
    {
        pMacDst->v[0] = 0x01;
        pMacDst->v[1] = 0x00;
        pMacDst->v[2] = 0x5E;
        pMacDst->v[3] = 0x7f & pIpAdd->v[1];
        pMacDst->v[4] = pIpAdd->v[2];
        pMacDst->v[5] = pIpAdd->v[3];
    }
    else
    {   
        if(_TCPIPStackIpAddFromLAN(pNetIf, pIpAdd))
        {
            pPkt->arpTarget.Val  = pIpAdd->Val;
        }
        else
        {   // not this LAN
            pPkt->arpTarget.Val  = pNetIf->netGateway.Val;
        }

        arpRes = TCPIP_ARP_EntryGet(pNetIf, &pPkt->arpTarget, pMacDst, true);
        if(arpRes == ARP_RES_ENTRY_SOLVED)
        {   // good to transmit
        }
        else if(arpRes == ARP_RES_ENTRY_QUEUED || arpRes == ARP_RES_ENTRY_NEW)
        {   // will have to be queued
            *ppMacAdd = 0;    // not known yet
        }
        else
        {   // discard, cannot send
            return TCPIP_IPV4_DEST_FAIL;
        }
    }

    return TCPIP_IPV4_DEST_NETWORK;
}

TCPIP_IPV4_FILTER_TYPE TCPIP_IPV4_PacketFilterSet(TCPIP_IPV4_FILTER_TYPE filtType)
{
    TCPIP_IPV4_FILTER_TYPE currFilt;

    OSAL_CRITSECT_DATA_TYPE status = OSAL_CRIT_Enter(OSAL_CRIT_TYPE_LOW);
    currFilt = (ipv4FilterType |= filtType);
    OSAL_CRIT_Leave(OSAL_CRIT_TYPE_LOW, status);
    return currFilt;
}

TCPIP_IPV4_FILTER_TYPE TCPIP_IPV4_PacketFilterClear(TCPIP_IPV4_FILTER_TYPE filtType)
{
    TCPIP_IPV4_FILTER_TYPE currFilt;

    OSAL_CRITSECT_DATA_TYPE status = OSAL_CRIT_Enter(OSAL_CRIT_TYPE_LOW);
    currFilt = (ipv4FilterType &= ~filtType);
    OSAL_CRIT_Leave(OSAL_CRIT_TYPE_LOW, status);
    return currFilt;
}

// debugging features
#if defined(TCPIP_IPV4_DEBUG)
void TCPIP_IPV4_CheckPkt(TCPIP_MAC_PACKET* pRxPkt)
{
    IPV4_HEADER* pHeader = (IPV4_HEADER*)pRxPkt->pNetLayer;
    if(pHeader->Protocol == IP_PROT_UDP)
    {   // UDP packet
        UDP_HEADER* pUDPHdr = (UDP_HEADER*)pRxPkt->pTransportLayer;
        UDP_PORT destPort = TCPIP_Helper_ntohs(pUDPHdr->DestinationPort);
        UDP_PORT srcPort = TCPIP_Helper_ntohs(pUDPHdr->SourcePort);
        if(srcPort == destPort)
        {
        }
    }
    else if(pHeader->Protocol == IP_PROT_TCP)
    { 
    }

}

    
#endif  // defined(TCPIP_IPV4_DEBUG)


#endif  // defined(TCPIP_STACK_USE_IPV4)


