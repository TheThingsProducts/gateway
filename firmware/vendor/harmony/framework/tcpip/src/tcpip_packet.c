/*******************************************************************************
  TCPIP network packet manager implementation

  Summary:
    
  Description:
*******************************************************************************/

/*******************************************************************************
File Name: tcpip_packet.c 
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
#include <sys/kmem.h>

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_MANAGER

#include "tcpip_private.h"
#include "tcpip_packet.h"


/*  TCPIP MAC Frame Offset

  Summary:
    An offset from a 4 byte aligned address where the MAC frame should start

  Description:
    Offset used on 32 bit machines that allows alignment of the network layer data.
    This allows improved efficiency for checksum calculations, etc.
  
  Remarks:
    Usual value is 2.

    See notes for the TCPIP_MAC_DATA_SEGMENT.segLoadOffset member.

    It makes sense to have this value as a #define (or global variable) instead of a parameter to the packet allocation function.
    In a system whith different MAC drivers the value may be different from MAC to MAC and then the stack would be forced
    to keep track of what packet could be forwarded on what interface.

*/

#if defined(TCPIP_IF_MRF24W)
    #define TCPIP_MAC_FRAME_OFFSET      6  // aligns ethernet packet and allows room for MRF 4-byte internal header
#elif defined(TCPIP_IF_MRF24WN)
    #define TCPIP_MAC_FRAME_OFFSET      2
#else
    #define TCPIP_MAC_FRAME_OFFSET      2
#endif

static TCPIP_STACK_HEAP_HANDLE    pktMemH = 0;
static bool                 pktK0Heap = 0;

#if defined(TCPIP_PACKET_TRACE_ENABLE)
static TCPIP_PKT_TRACE_ENTRY    _pktTraceTbl[TCPIP_PKT_TRACE_SIZE];


static TCPIP_PKT_TRACE_INFO _pktTraceInfo;  // global counters  


static TCPIP_PKT_TRACE_ENTRY*   _TCPIP_PKT_TraceFindEntry(int moduleId, bool addNewSlot);
static void                     _TCPIP_PKT_TraceAddToEntry(int moduleId, TCPIP_MAC_PACKET* pPkt);
static void                     _TCPIP_PKT_TraceFreeEntry(int moduleId, TCPIP_MAC_PACKET* pPkt);
static void                     _TCPIP_PKT_TraceAckEntry(int moduleId, TCPIP_MAC_PACKET* pPkt, TCPIP_MAC_PKT_ACK_RES ackRes);
static uint32_t                 _TCPIP_PKT_TracePktSize(TCPIP_MAC_PACKET* pPkt);


static /*__inline__*/ void /*__attribute__((always_inline))*/ _TCPIP_PKT_TraceFail(void)
{
    _pktTraceInfo.traceFails++;
}

#endif  // defined(TCPIP_PACKET_TRACE_ENABLE)


#if defined (TCPIP_PACKET_FLIGHT_LOG_ENABLE)

typedef uint8_t _TCPIP_PKT_FLAG_TYPE;       // only 8 flags supported for now
// module logging flags
// a 1 in a bit enables that logging
static _TCPIP_PKT_FLAG_TYPE _TCPIP_PKT_ModFlags[] = 
{
    0,                                      // TCPIP_MODULE_NONE
	TCPIP_PKT_FLIGHT_FLAG_LOG,              // TCPIP_MODULE_MANAGER,   

    TCPIP_PKT_FLIGHT_FLAG_LOG,              // TCPIP_MODULE_ARP,
    TCPIP_PKT_FLIGHT_FLAG_LOG,              // TCPIP_MODULE_IPV4,
	0,                                      // TCPIP_MODULE_IPV6,
	0,                                      // TCPIP_MODULE_LLDP,

    TCPIP_PKT_FLIGHT_FLAG_LOG,              // TCPIP_MODULE_ICMP,      
	0,                                      // TCPIP_MODULE_ICMPV6,
	0,                                      // TCPIP_MODULE_NDP,
    TCPIP_PKT_FLIGHT_FLAG_LOG,              // TCPIP_MODULE_UDP,
    TCPIP_PKT_FLIGHT_FLAG_LOG,              // TCPIP_MODULE_TCP,

    0,                                      // TCPIP_MODULE_DHCP_CLIENT,
    0,                                      // TCPIP_MODULE_DHCP_SERVER,
    0,                                      // TCPIP_MODULE_ANNOUNCE,
    0,                                      // TCPIP_MODULE_DNS_CLIENT,
    0,                                      // TCPIP_MODULE_DNS_SERVER,
    0,                                      // TCPIP_MODULE_ZCLL,              
    0,                                      // TCPIP_MODULE_MDNS,             
	0,                                      // TCPIP_MODULE_NBNS,
    0,                                      // TCPIP_MODULE_SMTP_CLIENT,
    0,                                      // TCPIP_MODULE_SNTP,
    0,                                      // TCPIP_MODULE_FTP_SERVER,
    0,                                      // TCPIP_MODULE_HTTP_SERVER,
    0,                                      // TCPIP_MODULE_HTTP_NET_SERVER,
    0,                                      // TCPIP_MODULE_TELNET_SERVER,
    0,                                      // TCPIP_MODULE_TLS,
    0,                                      // TCPIP_MODULE_SNMP_SERVER,
	0,                                      // TCPIP_MODULE_SNMPV3_SERVER,
    0,                                      // TCPIP_MODULE_DYNDNS_CLIENT,
    0,                                      // TCPIP_MODULE_BERKELEY,
    0,                                      // TCPIP_MODULE_REBOOT_SERVER,
	0,                                      // TCPIP_MODULE_COMMAND,
	0,                                      // TCPIP_MODULE_IPERF,
	0,                                      // TCPIP_MODULE_TFTPC,
    
};

// module names
static const uint8_t _TCPIP_PKT_ModName[][6] __attribute__((aligned(4)))= 
{
    "UNK :\0",             // TCPIP_MODULE_NONE

	"MGR :\0",             // TCPIP_MODULE_MANAGER,   

    "ARP :\0",             // TCPIP_MODULE_ARP,
    "IPV4:\0",             // TCPIP_MODULE_IPV4,
	"IPV6:\0",             // TCPIP_MODULE_IPV6,
	"LLDP:\0",             // TCPIP_MODULE_LLDP,

    "IC4 :\0",             // TCPIP_MODULE_ICMP,      
	"IC6 :\0",             // TCPIP_MODULE_ICMPV6,
	"NDP :\0",             // TCPIP_MODULE_NDP,
    "UDP :\0",             // TCPIP_MODULE_UDP,
    "TCP :\0",             // TCPIP_MODULE_TCP,

    "DHCC:\0",             // TCPIP_MODULE_DHCP_CLIENT,
    "DHCS:\0",             // TCPIP_MODULE_DHCP_SERVER,
    "ANN :\0",             // TCPIP_MODULE_ANNOUNCE,
    "DNSC:\0",             // TCPIP_MODULE_DNS_CLIENT,
    "DNSS:\0",             // TCPIP_MODULE_DNS_SERVER,
    "ZCLL:\0",             // TCPIP_MODULE_ZCLL,              
    "MDNS:\0",             // TCPIP_MODULE_MDNS,             
	"NBNS:\0",             // TCPIP_MODULE_NBNS,
    "SMTP:\0",             // TCPIP_MODULE_SMTP_CLIENT,
    "SNTP:\0",             // TCPIP_MODULE_SNTP,
    "FTPS:\0",             // TCPIP_MODULE_FTP_SERVER,
    "HTTP:\0",             // TCPIP_MODULE_HTTP_SERVER,
    "HTTN:\0",             // TCPIP_MODULE_HTTP_NET_SERVER,
    "TEL :\0",             // TCPIP_MODULE_TELNET_SERVER,
    "TLS :\0",             // TCPIP_MODULE_TLS,
    "SNMP:\0",             // TCPIP_MODULE_SNMP_SERVER,
	"SNM3:\0",             // TCPIP_MODULE_SNMPV3_SERVER,
    "DDNS:\0",             // TCPIP_MODULE_DYNDNS_CLIENT,
    "BSD :\0",             // TCPIP_MODULE_BERKELEY,
    "RBT :\0",             // TCPIP_MODULE_REBOOT_SERVER,
	"CMD :\0",             // TCPIP_MODULE_COMMAND,
	"IPER:\0",             // TCPIP_MODULE_IPERF,
	"TFTC:\0",             // TCPIP_MODULE_TFTPC,
    
};

// table with packet names based on th emodule type 
static const uint8_t _TCPIP_PKT_TypeName[][4] __attribute__((aligned(2))) = 
{
    "XX:\0",       // TCPIP_MODULE_NONE           unknown
    "XX:\0",       // TCPIP_MODULE_MANAGER  unknown
    // 1st layer modules
    "AR:\0",       // TCPIP_MODULE_ARP,
    "I4:\0",       // TCPIP_MODULE_IPV4,
    "I6:\0",       // TCPIP_MODULE_IPV6,
    "LD:\0",       // TCPIP_MODULE_LLDP,
    // 2nd  layer modules
    "C4:\0",       // TCPIP_MODULE_ICMP,
    "C6:\0",       // TCPIP_MODULE_ICMPV6,
    "ND:\0",       // TCPIP_MODULE_NDP,
    "UD:\0",       // TCPIP_MODULE_UDP,
    "TC:\0",       // TCPIP_MODULE_TCP,
};

static bool             _pktFlightLogEnabled  = false;    // quick global enable/disable

static uint32_t         _pktFlightTypeMask = 0;    // masks of enabled types        

static uint32_t         _pktFlightLogCnt   = 0;     // number of packets logged so far

static uint32_t         _pktFlightLineCnt   = 0;     // number of packets per line so far

static TCPIP_MAC_PACKET* _pktFlightLastPkt = 0;      // last logged packet


// size of the logging buffer
// >= 20 !
#define                 TCPIP_PKT_FLIGHT_LOG_BUFFER_LEN    40

// number of loggings before \n for the same packet
#define                 TCPIP_PKT_FLIGHT_LOG_NLINE_LIM     4


// access the module
static /*__inline__*/ int /*__attribute__((always_inline))*/ _TCPIP_PKT_FlightModIx(int moduleId)
{
    if((uint32_t)moduleId < sizeof(_TCPIP_PKT_ModFlags)/sizeof(*_TCPIP_PKT_ModFlags))
    {
        return moduleId;
    }

    return -1;
}



#endif  // defined (TCPIP_PACKET_FLIGHT_LOG_ENABLE)


// API

bool TCPIP_PKT_Initialize(TCPIP_STACK_HEAP_HANDLE heapH)
{
    pktMemH = 0;

    while(heapH != 0)
    {
        TCPIP_MAC_PACKET* allocPtr;

        allocPtr = (TCPIP_MAC_PACKET*)TCPIP_HEAP_Malloc(heapH, sizeof(TCPIP_MAC_PACKET));

        if(allocPtr == 0)
        {
            break;
        }

        TCPIP_HEAP_Free(heapH, allocPtr);
        if(!IS_KVA(allocPtr))
        {   // only kernel space buffers accepted
            break;
        }
        // success
        pktK0Heap = IS_KVA0(allocPtr);
        pktMemH = heapH;

#if defined(TCPIP_PACKET_TRACE_ENABLE)
        memset(_pktTraceTbl, 0, sizeof(_pktTraceTbl));
        memset(&_pktTraceInfo, 0, sizeof(_pktTraceInfo));
        _pktTraceInfo.nEntries = sizeof(_pktTraceTbl)/sizeof(*_pktTraceTbl);
#endif  // defined(TCPIP_PACKET_TRACE_ENABLE)

#if defined (TCPIP_PACKET_FLIGHT_LOG_ENABLE)
        _pktFlightLogEnabled  = false;
        _pktFlightTypeMask = 0;
        _pktFlightLogCnt = _pktFlightLineCnt = 0;
        _pktFlightLastPkt = 0;
#endif  // defined (TCPIP_PACKET_FLIGHT_LOG_ENABLE)

        break;
    }


    return pktMemH != 0;
    
}

void TCPIP_PKT_Deinitialize(void)
{
    pktMemH = 0;
}


TCPIP_MAC_PACKET* _TCPIP_PKT_PacketAlloc(uint16_t pktLen, uint16_t segLoadLen, TCPIP_MAC_PACKET_FLAGS flags)
{
    TCPIP_MAC_PACKET* pPkt;
    TCPIP_MAC_DATA_SEGMENT  *pSeg;
    uint16_t        pktUpLen, allocLen;

    if(pktLen < sizeof(TCPIP_MAC_PACKET))
    {
        pktLen = sizeof(TCPIP_MAC_PACKET);
    }

    pktUpLen = (((pktLen + 3) >> 2) << 2);     // 32 bits round up

    allocLen = pktUpLen + sizeof(*pSeg) + segLoadLen + sizeof(TCPIP_MAC_ETHERNET_HEADER) + TCPIP_MAC_FRAME_OFFSET;

    pPkt = (TCPIP_MAC_PACKET*)TCPIP_HEAP_Malloc(pktMemH, allocLen);

    if(pPkt)
    {   
        // clear the TCPIP_MAC_PACKET and 1st segment fields
        // populate the 1st segment
        memset(pPkt, 0, pktUpLen + sizeof(*pSeg));
        pSeg = (TCPIP_MAC_DATA_SEGMENT*)((uint8_t*)pPkt + pktUpLen);

        pSeg->segSize = segLoadLen + sizeof(TCPIP_MAC_ETHERNET_HEADER);
        pSeg->segLoadOffset = TCPIP_MAC_FRAME_OFFSET;
        pSeg->segLoad = (uint8_t*)(pSeg + 1) + TCPIP_MAC_FRAME_OFFSET;
        pSeg->segFlags = TCPIP_MAC_SEG_FLAG_STATIC; // embedded in TCPIP_MAC_PACKET itself
        pPkt->pDSeg = pSeg;

        pPkt->pMacLayer = pSeg->segLoad;
        pPkt->pktFlags = flags & (~TCPIP_MAC_PKT_FLAG_STATIC);  // this packet is dynamically allocated
        if(segLoadLen)
        {
            pPkt->pNetLayer = pPkt->pMacLayer + sizeof(TCPIP_MAC_ETHERNET_HEADER);
        }

        if(pktK0Heap)
        {
            pPkt = (TCPIP_MAC_PACKET*)KVA0_TO_KVA1(pPkt);
        }
    }

    return pPkt;


}


// allocates a socket packet
TCPIP_MAC_PACKET*  _TCPIP_PKT_SocketAlloc(uint16_t pktLen, uint16_t transpHdrLen, uint16_t payloadLen, TCPIP_MAC_PACKET_FLAGS flags)
{
    uint16_t          netHdrLen, totHdrLen;
    TCPIP_MAC_PACKET* pPkt;

    if((flags & TCPIP_MAC_PKT_FLAG_IPV6) != 0)
    {
        netHdrLen = sizeof(IPV6_HEADER);
    }
    else
    {
        netHdrLen = sizeof(IPV4_HEADER);
    }


    totHdrLen = netHdrLen + transpHdrLen;

    pPkt = _TCPIP_PKT_PacketAlloc(pktLen, totHdrLen +  payloadLen, flags );

    if(pPkt)
    {   // set the layer pointers in place
        if(transpHdrLen)
        {
            pPkt->pTransportLayer = pPkt->pNetLayer + netHdrLen;
        }
    }

    return pPkt;
}




// acknowledges a packet
void _TCPIP_PKT_PacketAcknowledge(TCPIP_MAC_PACKET* pPkt, TCPIP_MAC_PKT_ACK_RES ackRes)
{
    if(ackRes != TCPIP_MAC_PKT_ACK_NONE)
    {
        pPkt->ackRes = ackRes;
    }

    if(pPkt->ackFunc)
    {
       if((*pPkt->ackFunc)(pPkt, pPkt->ackParam))
       {
           pPkt->pktFlags &= ~TCPIP_MAC_PKT_FLAG_QUEUED;
       }
    }
    else
    {
        SYS_ERROR(SYS_ERROR_WARNING, "Packet Ack: orphan packet! \r\n");
    }
}

// frees a previously allocated packet
void _TCPIP_PKT_PacketFree(TCPIP_MAC_PACKET* pPkt)
{

    if((pPkt->pktFlags & TCPIP_MAC_PKT_FLAG_STATIC) == 0)
    {   // we don't deallocate static packets
        TCPIP_MAC_DATA_SEGMENT  *pSeg, *pNSeg;

        for(pSeg = pPkt->pDSeg; pSeg != 0 ; )
        {
            pNSeg = pSeg->next;
            if((pSeg->segFlags & TCPIP_MAC_SEG_FLAG_STATIC) == 0)
            {
                if(pktK0Heap)
                {
                    pSeg = (TCPIP_MAC_DATA_SEGMENT*)KVA1_TO_KVA0(pSeg);
                }
                TCPIP_HEAP_Free(pktMemH, pSeg);
            }
            pSeg = pNSeg;
        }

        if(pktK0Heap)
        {
            pPkt = (TCPIP_MAC_PACKET*)KVA1_TO_KVA0(pPkt);
        }
        TCPIP_HEAP_Free(pktMemH, pPkt);
    }
}

TCPIP_MAC_DATA_SEGMENT* TCPIP_PKT_SegmentAlloc(uint16_t loadLen, uint16_t loadOffset, TCPIP_MAC_SEGMENT_FLAGS flags)
{
    TCPIP_MAC_DATA_SEGMENT* pSeg;
    uint16_t allocSize;

    if(loadLen != 0)
    {
        allocSize = sizeof(*pSeg) + loadLen + loadOffset;
    }
    else
    {
        allocSize = sizeof(*pSeg);
    }

    pSeg = (TCPIP_MAC_DATA_SEGMENT*)TCPIP_HEAP_Malloc(pktMemH, allocSize);

    if(pSeg)
    {
        memset(pSeg, 0, sizeof(*pSeg));

        pSeg->segFlags = flags & (~TCPIP_MAC_SEG_FLAG_STATIC);
        if(loadLen != 0)
        {
            pSeg->segSize = loadLen;
            pSeg->segLoadOffset = loadOffset;
            pSeg->segLoad = (uint8_t*)(pSeg + 1) + loadOffset;
        }

        if(pktK0Heap)
        {
            pSeg = (TCPIP_MAC_DATA_SEGMENT*)KVA0_TO_KVA1(pSeg);
        }
        
    }

    return pSeg;
}

void TCPIP_PKT_SegmentAppend(TCPIP_MAC_PACKET* pPkt, TCPIP_MAC_DATA_SEGMENT* pSeg)
{
    TCPIP_MAC_DATA_SEGMENT  *pN, *prev;

    if((pN = pPkt->pDSeg) == 0)
    {   // insert as root
        pPkt->pDSeg = pSeg;
    }
    else
    {   // traverse the list
        for(prev = 0; pN != 0; prev = pN, pN = pN->next);
        prev->next = pSeg;
    }

}


void TCPIP_PKT_SegmentFree(TCPIP_MAC_DATA_SEGMENT* pSeg)
{
    if( (pSeg->segFlags & TCPIP_MAC_SEG_FLAG_STATIC) == 0)
    {
        if(pktK0Heap)
        {
            pSeg = (TCPIP_MAC_DATA_SEGMENT*)KVA1_TO_KVA0(pSeg);
        }
        TCPIP_HEAP_Free(pktMemH, pSeg);
    }

}

// helpers

bool TCPIP_PKT_PacketMACFormat(TCPIP_MAC_PACKET* pPkt, const TCPIP_MAC_ADDR* dstAddr, const TCPIP_MAC_ADDR* srcAddr, uint16_t pktType)
{
    if(srcAddr)
    {
        TCPIP_MAC_ETHERNET_HEADER* macHdr;
        TCPIP_MAC_ADDR    *destHdrAdd, *srcHdrAdd;

        macHdr = (TCPIP_MAC_ETHERNET_HEADER*)pPkt->pMacLayer;
        srcHdrAdd = &macHdr->SourceMACAddr;

        if(dstAddr)
        {
            destHdrAdd = &macHdr->DestMACAddr;
            memcpy(destHdrAdd, dstAddr, sizeof(*destHdrAdd));
        }

        memcpy(srcHdrAdd, srcAddr, sizeof(*srcHdrAdd));
        // set the MAC frame type
        macHdr->Type = TCPIP_Helper_htons(pktType);

        // update the frame length
        pPkt->pDSeg->segLen += sizeof(TCPIP_MAC_ETHERNET_HEADER);
        return true;
    }

    return false;
}

// returns the segment to which dataAddress belongs
// 0 if not in this packet
TCPIP_MAC_DATA_SEGMENT* TCPIP_PKT_DataSegmentGet(TCPIP_MAC_PACKET* pPkt, const uint8_t* dataAddress, bool srchTransport)
{
    TCPIP_MAC_DATA_SEGMENT  *pStartSeg, *pSeg;

    pStartSeg = 0;

    if(srchTransport)
    {   // search the segment containing the transport data
        for(pSeg = pPkt->pDSeg; pSeg != 0; pSeg = pSeg->next)
        {
            if(pSeg->segLoad <= pPkt->pTransportLayer && pPkt->pTransportLayer <= pSeg->segLoad + pSeg->segSize)
            {   // found segment containing the beg of the transport
                if(pPkt->pTransportLayer <= dataAddress && dataAddress <= pSeg->segLoad + pSeg->segSize)
                {
                    return pSeg;
                }

                pStartSeg = pSeg->next;
                break;
            }
        }
    }
    else
    {
        pStartSeg = pPkt->pDSeg;
    }


    for(pSeg = pStartSeg; pSeg != 0; pSeg = pSeg->next)
    {
        if(pSeg->segLoad <= dataAddress && dataAddress <= pSeg->segLoad + pSeg->segSize)
        {
            return pSeg;
        }
    }

    return 0;
}

uint16_t TCPIP_PKT_PayloadLen(TCPIP_MAC_PACKET* pPkt)
{
    uint32_t payloadSize = 0;

    if(pPkt)
    {
        TCPIP_MAC_DATA_SEGMENT* pSeg = pPkt->pDSeg;

        while(pSeg != 0)
        {
            payloadSize += pSeg->segLen;
            pSeg = pSeg->next;
        }
    }

    return payloadSize;
}

#if defined(TCPIP_PACKET_TRACE_ENABLE)
TCPIP_MAC_PACKET* _TCPIP_PKT_SocketAllocDebug(uint16_t pktLen, uint16_t tHdrLen, uint16_t payloadLen, TCPIP_MAC_PACKET_FLAGS flags, int moduleId)
{
    TCPIP_MAC_PACKET* pPkt = _TCPIP_PKT_SocketAlloc(pktLen, tHdrLen, payloadLen, flags);
    _TCPIP_PKT_TraceAddToEntry(moduleId, pPkt);
    return pPkt;

}

TCPIP_MAC_PACKET* _TCPIP_PKT_PacketAllocDebug(uint16_t pktLen, uint16_t segLoadLen, TCPIP_MAC_PACKET_FLAGS flags, int moduleId)
{
    TCPIP_MAC_PACKET* pPkt = _TCPIP_PKT_PacketAlloc(pktLen, segLoadLen, flags);
    _TCPIP_PKT_TraceAddToEntry(moduleId, pPkt);
    return pPkt;

}


void _TCPIP_PKT_PacketFreeDebug(TCPIP_MAC_PACKET* pPkt, int moduleId)
{
    _TCPIP_PKT_TraceFreeEntry(moduleId, pPkt);
    _TCPIP_PKT_PacketFree(pPkt);
}


void _TCPIP_PKT_PacketAcknowledgeDebug(TCPIP_MAC_PACKET* pPkt, TCPIP_MAC_PKT_ACK_RES ackRes, int moduleId)
{
    _TCPIP_PKT_PacketAcknowledge(pPkt, ackRes);
    _TCPIP_PKT_TraceAckEntry(moduleId, pPkt, ackRes);
}

int TCPIP_PKT_TraceGetEntriesNo(TCPIP_PKT_TRACE_INFO* pTraceInfo)
{
    TCPIP_PKT_TRACE_ENTRY *pEntry;
    int ix;
    int nUsed = 0;


    for(ix = 0, pEntry = _pktTraceTbl; ix < sizeof(_pktTraceTbl)/sizeof(*_pktTraceTbl); ix++, pEntry++)
    {
        if(pEntry->moduleId > 0)
        {
            nUsed++;
        }
    }

    _pktTraceInfo.nUsed = nUsed;
    if(pTraceInfo)
    {
        *pTraceInfo = _pktTraceInfo;
    }


    return nUsed;
}


// populates a trace entry with data for a index
bool TCPIP_PKT_TraceGetEntry(int entryIx, TCPIP_PKT_TRACE_ENTRY* tEntry)
{
    TCPIP_PKT_TRACE_ENTRY *pEntry;

    if(entryIx < sizeof(_pktTraceTbl)/sizeof(*_pktTraceTbl))
    {   // valid index
        pEntry = _pktTraceTbl + entryIx;
        if(pEntry->moduleId > 0)
        {
            *tEntry = *pEntry;
            return true;
        }
    }

    return false;
}

static TCPIP_PKT_TRACE_ENTRY* _TCPIP_PKT_TraceFindEntry(int moduleId, bool addNewSlot)
{
    int ix;
    TCPIP_PKT_TRACE_ENTRY    *freeEntry,*pEntry;

    freeEntry = 0;
    for(ix = 0, pEntry = _pktTraceTbl; ix < sizeof(_pktTraceTbl)/sizeof(*_pktTraceTbl); ix++, pEntry++)
    {
        if(pEntry->moduleId == moduleId)
        {
            return pEntry;
        }
        else if(addNewSlot && freeEntry == 0 && pEntry->moduleId == 0)
        {
            freeEntry = pEntry;
        }
    }

    if(freeEntry)
    {
        memset(freeEntry, 0x0, sizeof(*freeEntry));
        freeEntry->moduleId = moduleId;
    }

    return freeEntry;
}

static uint32_t _TCPIP_PKT_TracePktSize(TCPIP_MAC_PACKET* pPkt)
{
    TCPIP_MAC_DATA_SEGMENT* pSeg = pPkt->pDSeg;
    uint32_t pktSize = ((uint8_t*)pSeg - (uint8_t*)pPkt) + TCPIP_MAC_FRAME_OFFSET + sizeof(*pSeg) + pSeg->segSize;

    while((pSeg = pSeg->next) != 0)
    {
        if((pSeg->segFlags & TCPIP_MAC_SEG_FLAG_STATIC) == 0)
        {
            pktSize += sizeof(*pSeg) + pSeg->segSize;
        }
    }

    return pktSize;

}
    
static void _TCPIP_PKT_TraceAddToEntry(int moduleId, TCPIP_MAC_PACKET* pPkt)
{
    TCPIP_PKT_TRACE_ENTRY* pEntry = _TCPIP_PKT_TraceFindEntry(moduleId, true);

    if(pEntry)
    {
        if(pPkt)
        {
            pEntry->totAllocated++;
            pEntry->currAllocated++;
            pEntry->currSize += _TCPIP_PKT_TracePktSize(pPkt);
        }
        else
        {
            pEntry->totFailed++;
        }
    }
    else
    {
        _TCPIP_PKT_TraceFail();
    }

}



static void _TCPIP_PKT_TraceFreeEntry(int moduleId, TCPIP_MAC_PACKET* pPkt)
{
    TCPIP_PKT_TRACE_ENTRY* pEntry = _TCPIP_PKT_TraceFindEntry(moduleId, false);

    if(pEntry)
    {
        pEntry->currAllocated--;
        pEntry->currSize -= _TCPIP_PKT_TracePktSize(pPkt);
    }
    else
    {
        _TCPIP_PKT_TraceFail();
    }

}

static void _TCPIP_PKT_TraceAckEntry(int moduleId, TCPIP_MAC_PACKET* pPkt, TCPIP_MAC_PKT_ACK_RES ackRes)
{
    TCPIP_PKT_TRACE_ENTRY* pEntry = _TCPIP_PKT_TraceFindEntry(moduleId, false);

    if(pEntry)
    {
        pEntry->nAcks++;
        if(ackRes < 0)
        {
            _pktTraceInfo.traceAckErrors++;
        }
    }
    else
    {
        _pktTraceInfo.traceAckOwnerFails++;
    }

}

#endif  // defined(TCPIP_PACKET_TRACE_ENABLE)


#if defined (TCPIP_PACKET_FLIGHT_LOG_ENABLE)

void _TCPIP_PKT_FlightLog(int moduleId, TCPIP_MAC_PACKET* pPkt, TCPIP_STACK_MODULE type, TCPIP_MAC_PKT_ACK_RES res, const char* format, ...)
{
    char flightBuff[TCPIP_PKT_FLIGHT_LOG_BUFFER_LEN] __attribute__((aligned(4)));

    if(_pktFlightLogEnabled)
    {
        if(type < sizeof(_TCPIP_PKT_TypeName)/sizeof(*_TCPIP_PKT_TypeName))
        {
            uint32_t typeMask = 1 << type;
            if((_pktFlightTypeMask & typeMask) != 0)
            {   // enabled type

                int modIx = _TCPIP_PKT_FlightModIx(moduleId);
                if(modIx >= 0)
                {
                    TCPIP_PKT_FLIGHT_FLAGS modFlags = _TCPIP_PKT_ModFlags[modIx];

                    if((modFlags & TCPIP_PKT_FLIGHT_FLAG_LOG) != 0)
                    {   // logging enabled
                        va_list args;
                        va_start( args, format );

                        char* pBuff = flightBuff;
                        if(pPkt != _pktFlightLastPkt || ++_pktFlightLineCnt == TCPIP_PKT_FLIGHT_LOG_NLINE_LIM)
                        {
                            *pBuff++ = '\r'; *pBuff++ = '\n';
                            _pktFlightLastPkt = pPkt;
                            _pktFlightLineCnt = 0;
                        }

                        sprintf(pBuff, "p_%8x:%s %s %3d ", (uint32_t)pPkt, (char*)(_TCPIP_PKT_ModName + modIx), (char*)(_TCPIP_PKT_TypeName[type] + 0), res);
                        if(format)
                        {
                            size_t sLen = strlen(flightBuff);
                            vsnprintf(flightBuff + sLen, sizeof(flightBuff) - sLen , format, args);
                        }
                        SYS_CONSOLE_MESSAGE(flightBuff);
                        va_end( args );
                        _pktFlightLogCnt++;
                    }
                }

            }
        }
    }
}

void TCPIP_PKT_FlightFlagsUpdate(TCPIP_STACK_MODULE moduleId, TCPIP_PKT_FLIGHT_FLAGS flags, bool enable)
{

    int modIx = _TCPIP_PKT_FlightModIx(moduleId);
    if(modIx >= 0)
    {
        _TCPIP_PKT_FLAG_TYPE* pModFlags = _TCPIP_PKT_ModFlags + modIx;
        
        if(enable)
        {
            *pModFlags |= flags;
        }
        else
        {
            *pModFlags &= ~flags;
        }
    }
}

void TCPIP_PKT_FlightFlagsUpdateAll(TCPIP_PKT_FLIGHT_FLAGS flags, bool enable)
{
    _pktFlightLogEnabled = enable;
}


void TCPIP_PKT_FlightTypeUpdate(TCPIP_STACK_MODULE type, bool enable)
{
    if(type < sizeof(_TCPIP_PKT_TypeName)/sizeof(*_TCPIP_PKT_TypeName))
    {
        uint32_t typeMask = 1 << type;
        if(enable)
        {
            _pktFlightTypeMask |= typeMask;
        }
        else
        {
            _pktFlightTypeMask &= ~typeMask;
        }
    }

}

void TCPIP_PKT_FlightTypeUpdateAll(bool enable)
{
    if(enable)
    {
        _pktFlightTypeMask = 0xffffffff;
    }
    else
    {
        _pktFlightTypeMask = 0;
    }
}

#endif  //  defined (TCPIP_PACKET_FLIGHT_LOG_ENABLE)



