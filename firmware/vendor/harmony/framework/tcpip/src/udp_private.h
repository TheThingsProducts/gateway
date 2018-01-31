/*******************************************************************************
  UDP Module private definitions

  Company:
    Microchip Technology Inc.
    
  File Name:
    udp_private.h

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

#ifndef __UDP_PRIVATE_H_
#define __UDP_PRIVATE_H_

#define UDP_CHECKSUM_OFFSET     6u

typedef struct
{
    IPV4_PACKET             v4Pkt;     // safe cast to IPV4_PACKET
    TCPIP_MAC_DATA_SEGMENT  zcSeg[0];  // zero copy data segment (only if BSD used)
}UDP_V4_PACKET;


// flags the packet as belonging to the UDP pool
#define UDP_SOCKET_POOL_BUFFER_FLAG     (TCPIP_MAC_PKT_FLAG_USER)

// minimum size of buffer in the pool
#define UDP_SOCKET_POOL_BUFFER_MIN_SIZE 256

// incoming packet match flags
typedef enum
{
    TCPIP_UDP_PKT_MATCH_IP_TYPE     = 0x01,     // match occurred on the packet type Ipv4/IPv6
    TCPIP_UDP_PKT_MATCH_SRC_PORT    = 0x02,     // match occurred on the packet source port
    TCPIP_UDP_PKT_MATCH_NET         = 0x04,     // match occurred on the packet incoming interface
    TCPIP_UDP_PKT_MACTH_SRC_ADD     = 0x08,     // match occurred on the packet source address

    //
    // good match mask
    TCPIP_UDP_PKT_MACTH_MASK        = (TCPIP_UDP_PKT_MATCH_IP_TYPE | TCPIP_UDP_PKT_MATCH_SRC_PORT | TCPIP_UDP_PKT_MATCH_NET | TCPIP_UDP_PKT_MACTH_SRC_ADD)

}TCPIP_UDP_PKT_MATCH;

// socket flags 
typedef union
{
    struct
    {
        uint16_t    bcastForceType   : 2;   // destination forced to broadcast, if any
        uint16_t    looseRemPort     : 1;   // allows receiving data from any socket having the same destination Port
        // but irrespective its local port
        uint16_t    looseNetIf       : 1;   // allows receiving data on any interface        
        uint16_t    looseRemAddress  : 1;   // allows receiving data from any host address        
        uint16_t    srcSet           : 1;   // source address is set/forced
        uint16_t    srcValid         : 1;   // source address is valid (could be even 0) 
        uint16_t    srcSolved        : 1;   // source address map to network interface is solved 

        uint16_t    destSet          : 1;   // destination address is set/forced
        uint16_t    txSplitAlloc     : 1;   // if set, will perform a split (ZC) TX allocation
        uint16_t    usePool          : 1;   // allows allocating from the private pool        
        uint16_t    stackConfig      : 1;   // socket allocated by some stack configuration module
        uint16_t    openAddType      : 2;   // the address type used at open
        uint16_t    openBindIf       : 1;   // socket is bound to interface when opened 
        uint16_t    openBindAdd      : 1;   // socket is bound to address when opened 
    };
    uint16_t     Val;
}TCPIP_UDP_SKT_FLAGS;



// Stores information about a current UDP socket
typedef struct
{
    // TX side
    uint8_t*        txStart;        // internal TX Buffer; both IPv4 and IPv6
    uint8_t*        txEnd;          // end of TX Buffer
    uint8_t*        txWrite;        // current write pointer into the TX Buffer
    union
    {
        IPV4_PACKET*  pV4Pkt;        // IPv4 use; UDP_V4_PACKET type
        IPV6_PACKET*  pV6Pkt;        // IPv6 use;
        void*         pPkt;          // generic
        // if txSplitAlloc == 0 a socket always has both txStart and pV4Pkt/pV6Pkt available
                // for IPv4 once we have pV4Pkt, txStart should be one of its segments!!!
            // or neither (ADDR_ANY, IPv4/IPv6 not decided yet!)
    };
    uint16_t        txSize;         // size of the txBuffer
    // socket info
    UDP_SOCKET      sktIx;
    IPV4_ADDR       destAddress;    // requested destination address
    IPV4_ADDR       srcAddress;     // only if bound, else 0
    IPV4_ADDR       pktSrcAddress;  // source address of last received packet
    IPV4_ADDR       pktDestAddress; // destination address of last received packet
    TCPIP_NET_IF*   pSktNet;        // local interface; 
                                    // srcAddress can be outside of pSktNet to allow
                                    // sending packets with chosen IP address  
    UDP_PORT        remotePort;		// Remote node's UDP port number
    UDP_PORT        localPort;		// Local UDP port number, or 0 when free
    // rx side
    TCPIP_MAC_PACKET       *pCurrRxPkt;   // current RX packet 
    TCPIP_MAC_DATA_SEGMENT *pCurrRxSeg;   // current segment in the current packet
                                          // support for multi segment packets
    uint16_t        rxSegLen;       // current segment len
    uint16_t        rxTotLen;       // total remaining data length, across segments 
    uint8_t*        rxCurr;         // current RX pointer 
    // socket info + flags
    uint16_t        addType;        // IPV4/6 socket type, IP_ADDRESS_TYPE;
    uint8_t         txAllocLimit;   // max number of packets that can be queued/allocated at a certain time
    uint8_t         txAllocCnt;     // current number of packets that are allocated for TX
    TCPIP_UDP_SKT_FLAGS flags;      // current socket flags
    union
    {
        struct
        {
            uint8_t     rxAutoAdvance    : 1;   // automatic RX socket advance
            uint8_t     rxEnable         : 1;   // enable socket receive
            uint8_t     reserved         : 6;   // reserved for future use
        };
        uint8_t     Val;
    }extFlags;      // additional socket flags

    uint8_t         rxQueueLimit;   // max number of RX packets that can be queued at a certain time
    SINGLE_LIST     rxQueue;        // queued RX packets belonging to this socket
    TCPIP_UDP_SIGNAL_FUNCTION sigHandler;  // socket event handler
    uint16_t        sigMask;         // TCPIP_UDP_SIGNAL_TYPE: active events
    
    uint8_t      padding[2];        // padding; not used

} UDP_SOCKET_DCPT;


#endif  // __UDP_PRIVATE_H_



