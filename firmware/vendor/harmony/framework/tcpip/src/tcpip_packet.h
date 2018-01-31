/*******************************************************************************
  TCPIP network packet manager - private stack API

  Company:
    Microchip Technology Inc.
    
  File Name:
    tcpip_packet.h

  Summary:
    
  Description:
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright © 2012 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

#ifndef __TCPIP_PACKET_H_
#define __TCPIP_PACKET_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include "tcpip/tcpip_mac.h"

// enables packet trace, debugging
//#define TCPIP_PACKET_TRACE_ENABLE
// enables packet flight logging
//#define TCPIP_PACKET_FLIGHT_LOG_ENABLE

// global trace info
typedef struct
{
    int nEntries;           // all trace entries
    int nUsed;              // used ones
    int traceFails;         // failed to allocate/find a packet entry
    int traceAckErrors;     // packets acknowledged with an error code
    int traceAckOwnerFails; // packets acknowledged but no module owner found
}TCPIP_PKT_TRACE_INFO;

// packet trace
// the moduleId is the one from tcpip.h::TCPIP_STACK_MODULE
// only if TCPIP_PACKET_TRACE_ENABLE defined
typedef struct
{
    int         moduleId;           // info belonging to this module; <0 means slot free
    uint32_t    totAllocated;       // total number of packets allocated successfully by this module
    uint32_t    currAllocated;      // number of packets still allocated by this module
    uint32_t    currSize;           // number of bytes currently allocated
    uint32_t    totFailed;          // total number of packets that failed for this module
    uint32_t    nAcks;              // number of acknowledgments
}TCPIP_PKT_TRACE_ENTRY;

// number of trace slots
// each module that allocates packets should have its slot.
// currently: tcp, udp, icmp, arp, ipv6
#define TCPIP_PKT_TRACE_SIZE        8

// module and packet logging flags
// only if TCPIP_PACKET_FLIGHT_LOG_ENABLE is enabled
// Note: only 8 bit flags supported for now!
// multiple flags can be or-ed together
typedef enum
{
    TCPIP_PKT_FLIGHT_FLAG_LOG     = 0x01, // enables, disables module logging
    TCPIP_PKT_FLIGHT_FLAG_TSTAMP    = 0x02, // enables, disables module time stamp logging

}TCPIP_PKT_FLIGHT_FLAGS;


// Extra TX/RX packet flags
// NOTE: // 16 bits only packet flags!

#define    TCPIP_MAC_PKT_FLAG_ARP           (TCPIP_MAC_PKT_FLAG_USER << 0)  // ARP packet data
                                                                            // set when the packet is ARP
                                                                            //
#define    TCPIP_MAC_PKT_FLAG_NET_TYPE      (TCPIP_MAC_PKT_FLAG_USER << 1)  // the network type: IPv4/IPv6 packet
#define    TCPIP_MAC_PKT_FLAG_IPV4          (0)                             // IPv4 packet data; cleared when the packet is IPV4
#define    TCPIP_MAC_PKT_FLAG_IPV6          (TCPIP_MAC_PKT_FLAG_USER << 1)  // IPv6 packet data; set when the packet is IPV6

#define    TCPIP_MAC_PKT_FLAG_LLDP          (TCPIP_MAC_PKT_FLAG_USER << 2)  // LLDP packet data; set when the packet is LLDP

#define    TCPIP_MAC_PKT_FLAG_ICMP_TYPE     (TCPIP_MAC_PKT_FLAG_USER << 3)  // ICMP packet type
#define    TCPIP_MAC_PKT_FLAG_ICMPV4        (0)                             // ICMPv4 packet data; cleared when the packet is ICMPv4
#define    TCPIP_MAC_PKT_FLAG_ICMPV6        (TCPIP_MAC_PKT_FLAG_USER << 3)  // ICMPv6 packet data; set when the packet is ICMPv6

#define    TCPIP_MAC_PKT_FLAG_NDP           (TCPIP_MAC_PKT_FLAG_USER << 4)  // NDP packet data; set when the packet is NDP

#define    TCPIP_MAC_PKT_FLAG_TRANSP_TYPE   (TCPIP_MAC_PKT_FLAG_USER << 5)  // UDP/TCP packet type
#define    TCPIP_MAC_PKT_FLAG_UDP           (0)                             // UDP packet data; set when the packet is UDP
#define    TCPIP_MAC_PKT_FLAG_TCP           (TCPIP_MAC_PKT_FLAG_USER << 5)  // TCP packet data; set when the packet is TCP

// available                                (TCPIP_MAC_PKT_FLAG_USER << 6)
//

                                            // packet type extraction mask
#define    TCPIP_MAC_PKT_FLAG_TYPE_MASK     (TCPIP_MAC_PKT_FLAG_ARP | TCPIP_MAC_PKT_FLAG_NET_TYPE | TCPIP_MAC_PKT_FLAG_LLDP | TCPIP_MAC_PKT_FLAG_ICMP_TYPE | TCPIP_MAC_PKT_FLAG_NDP | TCPIP_MAC_PKT_FLAG_TRANSP_TYPE)

#define    TCPIP_MAC_PKT_FLAG_CONFIG        (TCPIP_MAC_PKT_FLAG_USER << 7)  // packet needs to be transmitted even when the stack
                                                                            // is not properly configured
                                                                            // probably a stack configuration packet


// initialization API

// sets the heap handle to be used for packet allocations
bool            TCPIP_PKT_Initialize(TCPIP_STACK_HEAP_HANDLE heapH);

void            TCPIP_PKT_Deinitialize(void);


// packet allocation API


// allocates a socket/transport IPv4/IPv6 packet
// The MAC, IPv4/IPv6 and transport headers (tHdrLen != 0) are all located in the 1st segment
// if payloadLen != 0 then the 1st segment will contain the/some payload too
// if needed, extra segments could be eventually added to the packet
// pktLen - size of the packet structure (at least TCPIP_MAC_PACKET will be allocated)
// tHdrLen - optional transport header length to be part of the 1st data segment
// payloadLen - optional transport payload to be part of the 1st data segment
// flags     - packet + 1st segment flags
TCPIP_MAC_PACKET*  TCPIP_PKT_SocketAlloc(uint16_t pktLen, uint16_t tHdrLen, uint16_t payloadLen, TCPIP_MAC_PACKET_FLAGS flags);


// allocates a TCPIP_MAC_PACKET packet (TCPIP_MAC_ETHERNET_HEADER always added);
// pktLen - the size of the packet (it will be 32 bits rounded up)
// segLoadLen - the payload size for the segment associated to this packet; Payload is always 32 bit aligned
//              if 0 no segment is created
// flags are attached to the 1st segment too 
TCPIP_MAC_PACKET* TCPIP_PKT_PacketAlloc(uint16_t pktLen, uint16_t segLoadLen, TCPIP_MAC_PACKET_FLAGS flags);


// forces freeing a previously allocated packet
// note that neither the packet nor segments marked
// with TCPIP_MAC_PKT_FLAG_STATIC are not freed
// Also note that this function does not free explicitly the segment payload.
// A payload that was created contiguously when the segment was created
// will be automatically freed by this function.
void            TCPIP_PKT_PacketFree(TCPIP_MAC_PACKET* pPkt);

extern __inline__ void __attribute__((always_inline)) TCPIP_PKT_PacketAcknowledgeSet(TCPIP_MAC_PACKET* pPkt, TCPIP_MAC_PACKET_ACK_FUNC ackFunc, const void* ackParam)
{
    pPkt->ackFunc = ackFunc;
    pPkt->ackParam = ackParam;
}

// acknowledges a packet
// clears the TCPIP_MAC_PKT_FLAG_QUEUED flag!
// a packet should always have an acknowledgment function
// packet's ackRes is updated only if the parameter ackRes != TCPIP_MAC_PKT_ACK_NONE.
void            TCPIP_PKT_PacketAcknowledge(TCPIP_MAC_PACKET* pPkt, TCPIP_MAC_PKT_ACK_RES ackRes);


//  simple segment allocation/manipulation

// allocates a segment with payload following immediately the segment header 
// this segment can be added to a packet using TCPIP_PKT_SegmentAppend
// loadLen specifies the segment allocated payload (could be 0)
// The segment payload is always allocated to be 32-bit aligned.
// The segment payload pointer will point loadOffset bytes after this address 
// 
TCPIP_MAC_DATA_SEGMENT* TCPIP_PKT_SegmentAlloc(uint16_t loadLen, uint16_t loadOffset, TCPIP_MAC_SEGMENT_FLAGS flags);

// adds a segment to the tail of segments of a packet
// segment should be fully constructed, with flags updated
void            TCPIP_PKT_SegmentAppend(TCPIP_MAC_PACKET* pPkt, TCPIP_MAC_DATA_SEGMENT* pSeg);           

// frees a created segment
void            TCPIP_PKT_SegmentFree(TCPIP_MAC_DATA_SEGMENT* pSeg);


// packet helpers
//

// sets the proper source, destination and type for a packet
// it also updated the packet length to include the MAC header size
// dstAddr can be 0 if not known
// packet should have been properly allocated and pMacLayer set
// returns false if the srcAddress is 0 though
// (intended for checking that a network interface is down)
bool             TCPIP_PKT_PacketMACFormat(TCPIP_MAC_PACKET* pPkt, const TCPIP_MAC_ADDR* dstAddr, const TCPIP_MAC_ADDR* srcAddr, uint16_t pktType);


// returns the segment to which dataAddress belongs
// the search occurs in every segment of the packet
// if srchTransport is set, the search starts with the transport data, i.e.
//      startPoint = pTransportLayer,
//      endPoint = startPoint + totTransportLen
// otherwise
//      startPoint = segLoad
//      endPoint = startPoint + segSize
// 0 if not in this packet
TCPIP_MAC_DATA_SEGMENT* TCPIP_PKT_DataSegmentGet(TCPIP_MAC_PACKET* pPkt, const uint8_t* dataAddress, bool srchTransport);


// simple helper to calculate the payload length of a packet
uint16_t        TCPIP_PKT_PayloadLen(TCPIP_MAC_PACKET* pPkt);


// debugging, tracing
//

// returns the number of entries in the trace
int     TCPIP_PKT_TraceGetEntriesNo(TCPIP_PKT_TRACE_INFO* pTraceInfo);


// populates a trace entry with data for a index
bool    TCPIP_PKT_TraceGetEntry(int entryIx, TCPIP_PKT_TRACE_ENTRY* tEntry);


// logs a packet flight info with extra parameters for a module
void    TCPIP_PKT_FlightLog(TCPIP_MAC_PACKET* pPkt, TCPIP_STACK_MODULE type, TCPIP_MAC_PKT_ACK_RES res, const char* format, ...);

// sets logging flight flags for a module
void    TCPIP_PKT_FlightFlagsUpdate(TCPIP_STACK_MODULE modId, TCPIP_PKT_FLIGHT_FLAGS flags, bool enable);

// enables/disables the packet flight log for all modules
void    TCPIP_PKT_FlightFlagsUpdateAll(TCPIP_PKT_FLIGHT_FLAGS flags, bool enable);

// enables/disables flight logging for a type
void    TCPIP_PKT_FlightTypeUpdate(TCPIP_STACK_MODULE type, bool enable);

// globbally enables/disables all types
void    TCPIP_PKT_FlightTypeUpdateAll(bool enable);

#if defined(TCPIP_PACKET_TRACE_ENABLE)

// proto
TCPIP_MAC_PACKET*   _TCPIP_PKT_SocketAllocDebug(uint16_t pktLen, uint16_t tHdrLen, uint16_t payloadLen, TCPIP_MAC_PACKET_FLAGS flags, int moduleId);
TCPIP_MAC_PACKET*   _TCPIP_PKT_PacketAllocDebug(uint16_t pktLen, uint16_t segLoadLen, TCPIP_MAC_PACKET_FLAGS flags, int moduleId);
void                _TCPIP_PKT_PacketFreeDebug(TCPIP_MAC_PACKET* pPkt, int moduleId);
void                _TCPIP_PKT_PacketAcknowledgeDebug(TCPIP_MAC_PACKET* pPkt, TCPIP_MAC_PKT_ACK_RES ackRes, int moduleId);


// direct calls
#define TCPIP_PKT_SocketAlloc(pktLen, tHdrLen, payloadLen, flags)   _TCPIP_PKT_SocketAllocDebug(pktLen, tHdrLen, payloadLen, flags, TCPIP_THIS_MODULE_ID)
#define TCPIP_PKT_PacketAlloc(pktLen, segLoadLen, flags)            _TCPIP_PKT_PacketAllocDebug(pktLen, segLoadLen, flags, TCPIP_THIS_MODULE_ID)
#define TCPIP_PKT_PacketFree(pPkt)                                  _TCPIP_PKT_PacketFreeDebug(pPkt, TCPIP_THIS_MODULE_ID)
#define TCPIP_PKT_PacketAcknowledge(pPkt, ackRes)                   _TCPIP_PKT_PacketAcknowledgeDebug(pPkt, ackRes, TCPIP_THIS_MODULE_ID)

// pointer calls
typedef TCPIP_MAC_PACKET* (*_TCPIP_PKT_SocketAllocDebugPtr)(uint16_t pktLen, uint16_t tHdrLen, uint16_t payloadLen, TCPIP_MAC_PACKET_FLAGS flags, int moduleId);
typedef TCPIP_MAC_PACKET* (*_TCPIP_PKT_PacketAllocDebugPtr)(uint16_t pktLen, uint16_t segLoadLen, TCPIP_MAC_PACKET_FLAGS flags, int moduleId);
typedef void              (*_TCPIP_PKT_PacketFreeDebugPtr)(TCPIP_MAC_PACKET* pPkt, int moduleId);
typedef void              (*_TCPIP_PKT_PacketAcknowledgeDebugPtr)(TCPIP_MAC_PACKET* pPkt, TCPIP_MAC_PKT_ACK_RES ackRes, int moduleId);


#define     _TCPIP_PKT_SKT_ALLOC_PTR    _TCPIP_PKT_SocketAllocDebugPtr
#define     _TCPIP_PKT_ALLOC_PTR        _TCPIP_PKT_PacketAllocDebugPtr
#define     _TCPIP_PKT_FREE_PTR         _TCPIP_PKT_PacketFreeDebugPtr
#define     _TCPIP_PKT_ACK_PTR          _TCPIP_PKT_PacketAcknowledgeDebugPtr

#define     _TCPIP_PKT_SKT_ALLOC_FNC    _TCPIP_PKT_SocketAllocDebug
#define     _TCPIP_PKT_ALLOC_FNC        _TCPIP_PKT_PacketAllocDebug
#define     _TCPIP_PKT_FREE_FNC         _TCPIP_PKT_PacketFreeDebug
#define     _TCPIP_PKT_ACK_FNC          _TCPIP_PKT_PacketAcknowledgeDebug


#define     _TCPIP_PKT_SKT_ALLOC_BY_PTR(ptr, pktLen, tHdrLen, payloadLen, flags)    (*ptr)(pktLen, tHdrLen, payloadLen, flags, TCPIP_THIS_MODULE_ID)
#define     _TCPIP_PKT_ALLOC_BY_PTR(ptr, pktLen, segLoadLen, flags)                 (*ptr)(pktLen, segLoadLen, flags, TCPIP_THIS_MODULE_ID)
#define     _TCPIP_PKT_FREE_BY_PTR(ptr, pPkt)                                       (*ptr)(pPkt, TCPIP_THIS_MODULE_ID)
#define     _TCPIP_PKT_ACK_BY_PTR(ptr, pPkt, ackRes)                                (*ptr)(pPkt, ackRes, TCPIP_THIS_MODULE_ID)

#else

// proto
TCPIP_MAC_PACKET*   _TCPIP_PKT_SocketAlloc(uint16_t pktLen, uint16_t tHdrLen, uint16_t payloadLen, TCPIP_MAC_PACKET_FLAGS flags);
TCPIP_MAC_PACKET*   _TCPIP_PKT_PacketAlloc(uint16_t pktLen, uint16_t segLoadLen, TCPIP_MAC_PACKET_FLAGS flags);
void                _TCPIP_PKT_PacketFree(TCPIP_MAC_PACKET* pPkt);
void                _TCPIP_PKT_PacketAcknowledge(TCPIP_MAC_PACKET* pPkt, TCPIP_MAC_PKT_ACK_RES ackRes);


// direct calls
#define TCPIP_PKT_SocketAlloc(pktLen, tHdrLen, payloadLen, flags)   _TCPIP_PKT_SocketAlloc(pktLen, tHdrLen, payloadLen, flags)
#define TCPIP_PKT_PacketAlloc(pktLen, segLoadLen, flags)            _TCPIP_PKT_PacketAlloc(pktLen, segLoadLen, flags)
#define TCPIP_PKT_PacketFree(pPkt)                                  _TCPIP_PKT_PacketFree(pPkt)
#define TCPIP_PKT_PacketAcknowledge(pPkt, ackRes)                   _TCPIP_PKT_PacketAcknowledge(pPkt, ackRes)


// pointer calls
typedef TCPIP_MAC_PACKET* (*_TCPIP_PKT_SocketAllocPtr)(uint16_t pktLen, uint16_t tHdrLen, uint16_t payloadLen, TCPIP_MAC_PACKET_FLAGS flags);
typedef TCPIP_MAC_PACKET* (*_TCPIP_PKT_PacketAllocPtr)(uint16_t pktLen, uint16_t segLoadLen, TCPIP_MAC_PACKET_FLAGS flags);
typedef void              (*_TCPIP_PKT_PacketFreePtr)(TCPIP_MAC_PACKET* pPkt);
typedef void              (*_TCPIP_PKT_PacketAcknowledgePtr)(TCPIP_MAC_PACKET* pPkt, TCPIP_MAC_PKT_ACK_RES ackRes);


#define     _TCPIP_PKT_SKT_ALLOC_PTR    _TCPIP_PKT_SocketAllocPtr
#define     _TCPIP_PKT_ALLOC_PTR        _TCPIP_PKT_PacketAllocPtr
#define     _TCPIP_PKT_FREE_PTR         _TCPIP_PKT_PacketFreePtr
#define     _TCPIP_PKT_ACK_PTR          _TCPIP_PKT_PacketAcknowledgePtr


#define     _TCPIP_PKT_SKT_ALLOC_FNC    _TCPIP_PKT_SocketAlloc
#define     _TCPIP_PKT_ALLOC_FNC        _TCPIP_PKT_PacketAlloc
#define     _TCPIP_PKT_FREE_FNC         _TCPIP_PKT_PacketFree
#define     _TCPIP_PKT_ACK_FNC          _TCPIP_PKT_PacketAcknowledge


#define     _TCPIP_PKT_SKT_ALLOC_BY_PTR(ptr, pktLen, tHdrLen, payloadLen, flags)    (*ptr)(pktLen, tHdrLen, payloadLen, flags)
#define     _TCPIP_PKT_ALLOC_BY_PTR(ptr, pktLen, segLoadLen, flags)                 (*ptr)(pktLen, segLoadLen, flags)
#define     _TCPIP_PKT_FREE_BY_PTR(ptr, pPkt)                                       (*ptr)(pPkt)
#define     _TCPIP_PKT_ACK_BY_PTR(ptr, pPkt, ackRes)                                (*ptr)(pPkt, ackRes)


extern __inline__ int __attribute__((always_inline)) TCPIP_PKT_TraceGetEntriesNo(TCPIP_PKT_TRACE_INFO* pTraceInfo)
{
    return 0;
}


// populates a trace entry with data for a index
extern __inline__ bool __attribute__((always_inline)) TCPIP_PKT_TraceGetEntry(int entryIx, TCPIP_PKT_TRACE_ENTRY* tEntry)
{
    return false;
}

#endif  // defined(TCPIP_PACKET_TRACE_ENABLE)

#if defined (TCPIP_PACKET_FLIGHT_LOG_ENABLE)

void      _TCPIP_PKT_FlightLog(int moduleId, TCPIP_MAC_PACKET* pPkt, TCPIP_STACK_MODULE type, TCPIP_MAC_PKT_ACK_RES res, const char* format, ...);

#define TCPIP_PKT_FlightLog(pPkt, type, res, format, ...)              _TCPIP_PKT_FlightLog(TCPIP_THIS_MODULE_ID, pPkt, type, res, format, ##__VA_ARGS__)

#else

extern __inline__ void __attribute__((always_inline)) TCPIP_PKT_FlightLog(TCPIP_MAC_PACKET* pPkt, TCPIP_STACK_MODULE type, TCPIP_MAC_PKT_ACK_RES res, const char* format, ...)
{
}

// enables a packet flight logging for a module
extern __inline__ void __attribute__((always_inline)) TCPIP_PKT_FlightFlagsUpdate(TCPIP_STACK_MODULE modId, TCPIP_PKT_FLIGHT_FLAGS flags, bool enable)
{
}


// enables/disables the packet flight logging for all modules
extern __inline__ void __attribute__((always_inline))   TCPIP_PKT_FlightFlagsUpdateAll(TCPIP_PKT_FLIGHT_FLAGS flags, bool enable)
{
}

// enables/disables flight logging for a type
extern __inline__ void __attribute__((always_inline)) TCPIP_PKT_FlightTypeUpdate(TCPIP_STACK_MODULE type, bool enable)
{
}

extern __inline__ void __attribute__((always_inline)) TCPIP_PKT_FlightTypeUpdateAll(bool enable)
{
}


#endif  //  defined (TCPIP_PACKET_FLIGHT_LOG_ENABLE)

#endif // __TCPIP_PACKET_H_


