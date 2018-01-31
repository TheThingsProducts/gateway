/**********************************************************************
IPv4 Header File

  Company:
    Microchip Technology Inc.
	
  File Name:
    ipv4.h
	
  Summary:
    IPv4 definitions for the Microchip TCP/IP Stack.
	
  Description:
    IP is the workhorse protocol of the TCP/IP protocol suite. All TCP,
    UDP, and ICMP data gets transmitted as IP datagrams. IP provides an
    unreliable, connectionless datagram delivery service.              
  **********************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright 2012-2015 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#ifndef __IPV4_H_
#define __IPV4_H_

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  


// *****************************************************************************
// *****************************************************************************
// Section: Public Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************

typedef struct
{
}TCPIP_IPV4_MODULE_CONFIG;

// *****************************************************************************
/* IPv4 supported protocols

  Summary:
    List of supported protocols.

  Description:
    This is the list of the protocols that are supported by this IPv4 implementation.

  Remarks:
    None.
 */
typedef enum
{
    IP_PROT_ICMP = (1u),
    IP_PROT_TCP = (6u),
    IP_PROT_UDP = (17u),
} IPV4_HEADER_TYPE;


// *****************************************************************************
/* IPv4 packet header definition

  Summary:
    Structure of an IPv4 header.

  Description:
    This is the structure of an IPv4 packet header.

  Remarks:
    None.
 */
typedef struct
{
    uint8_t VersionIHL;
    uint8_t TypeOfService;
    uint16_t TotalLength;
    uint16_t Identification;
    uint16_t FragmentInfo;
    uint8_t TimeToLive;
    uint8_t Protocol;
    uint16_t HeaderChecksum;
    IPV4_ADDR SourceAddress;
    IPV4_ADDR DestAddress;
} IPV4_HEADER;


// *****************************************************************************
/* IPv4 packet structure definition

  Summary:
    IPv4 packet structure.

  Description:
    This is the structure of an IPv4 packet for transmission over the network.

  Remarks:
    None.
 */
typedef struct
{
    TCPIP_MAC_PACKET    macPkt;         // standard MAC packet header
                                        // safe cast to TCPIP_MAC_PACKET

    // additional IPv4 packet data 
    IPV4_ADDR           srcAddress;     // packet source
    IPV4_ADDR           destAddress;    // packet destination
    TCPIP_NET_HANDLE    netIfH;         // packet interface
    IPV4_ADDR           arpTarget;      // ARP resolution target
} IPV4_PACKET;

// *****************************************************************************
/* IPv4 packet filters

  Summary:
    List of supported IPv4 packet filters.

  Description:
    This is the list of the packet filters that are supported by this IPv4 implementation.
    There are 3 types of IPv4 packets currently supported:
        - unicast
        - broadcast
        - multicast
    An IPV4 packet is accepted if the filter corresponding to the packet type is not set 

  Remarks:
    Multiple filters can be set
    
    If no filter is set, all packets are accepted this is the default case.
 */
typedef enum
{
    TCPIP_IPV4_FILTER_NONE      = 0x00,     // no packet filter active. All packets are accepted
    TCPIP_IPV4_FILTER_UNICAST   = 0x01,     // unicast packets will be filtered out
    TCPIP_IPV4_FILTER_BROADCAST = 0x02,     // broadcast packets will be filtered out
    TCPIP_IPV4_FILTER_MULTICAST = 0x04,     // multicast packets will be filtered out
} TCPIP_IPV4_FILTER_TYPE;


// *****************************************************************************
// *****************************************************************************
// Section: Public Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*
  Function:
    TCPIP_IPV4_PacketFormatTx(IPV4_PACKET* pPkt, uint8_t protocol, uint16_t ipLoadLen);

  Summary:
    Formats an IPV4 packet and makes it ready for transmission.

  Description:
    The necessary fields are set into the IPv4 packet.

  Precondition:
    Properly allocated pPkt.
    The source and destination addresses should be updated in the packet.

  Parameters:
    pPkt      - the packet to be formatted
    protocol  - the protocol associated with the packet
    ipLoadLen - the IPv4 packet payload length

  Returns:
    None
      
  Remarks:
    The segments should be properly updated with the right number of bytes (segLen).
    The IP payload length (ipLoadLen) is added only to the 1st segment of the packet!
    Other segments (for packets having multiple packets) are not touched.

 */
void    TCPIP_IPV4_PacketFormatTx(IPV4_PACKET* pPkt, uint8_t protocol, uint16_t ipLoadLen);


// *****************************************************************************
/*
  Function:
    bool TCPIP_IPV4_PacketTransmit(IPV4_PACKET* pPkt);

  Summary:
    Transmits an IPV4 packet over the network.

  Description:
    The IPv4 packet is sent to the MAC for transmission.

  Precondition:
    pPkt should have been properly formatted with TCPIP_IPV4_PacketFormatTx().
    The packet interface should be updated.
        

  Parameters:
    pPkt     - the packet to be transmitted

  Returns:
    - true  - if the packet was handed to the MAC or is queued for transmission
    - false - the packet cannot be transmitted (wrong interface, etc.) 
      
  Remarks:
    Only single packets can be transmitted.
    Chained packets are not supported for now.
 */
bool    TCPIP_IPV4_PacketTransmit(IPV4_PACKET* pPkt);


// *****************************************************************************
/*
  Function:
    TCPIP_NET_HANDLE TCPIP_IPV4_SelectSourceInterface(TCPIP_NET_HANDLE netH, 
	         IPV4_ADDR* pDestAddress, IPV4_ADDR* pSrcAddress, bool srcSet)

  Summary:
    Selects a source address and an interface based on the IPv4 destination address

  Description:
    Updates the pSrcAddress and returns the needed interface, if successful:
    * if srcSet == 1 and netH != 0, the function will not change anything 
    * if srcSet == 1 and netH == 0, the call will never fail it will use whatever 
	  value in pSrcAddress (even 0) and will try to come up with an appropriate interface
    * if srcSet == 0 and netH == 0, it will use the destination address
    * if srcSet == 0 and netH != 0, it will use the address of that interface

  Precondition:
    netH has to be valid (if non-0).

  Parameters:
    netH            - network interface handle
    pDestAddress    - pointer to destination address
    pSrcAddress     - pointer to source address
    srcSet          - boolean; true if address pointed by pSrcAddress is valid

  Returns:
    - A valid interface - if it succeeds and a valid source interface selected
    - 0 - interface selection failed
      
  Remarks:
    None.
 */
TCPIP_NET_HANDLE   TCPIP_IPV4_SelectSourceInterface(TCPIP_NET_HANDLE netH, 
                 IPV4_ADDR* pDestAddress, IPV4_ADDR* pSrcAddress, bool srcSet);

// *****************************************************************************
/*
  Function:
    const IPV4_ADDR*  TCPIP_IPV4_PacketGetDestAddress(TCPIP_MAC_PACKET* pPkt);

  Summary:
    Returns the IPv4 destination address associated with a TCPIP_MAC_PACKET

  Description:
    The function will return a pointer to where the IPv4 destination address
    is located in the TCPIP_MAC_PACKET.
    The TCPIP_MAC_PACKET is supposed to be a valid IPv4 packet that has
    destination address properly set.

  Precondition:
    pPkt - valid IPv4 packet, pNetLayer filed properly set.
        
  Parameters:
    pPkt - packet to query

  Returns:
    - A valid pointer to an IPV4_ADDR - if it succeeds
    - 0 - if call failed
      
  Remarks:
    This function is primarily meant for RX packets.
 */
const IPV4_ADDR*  TCPIP_IPV4_PacketGetDestAddress(TCPIP_MAC_PACKET* pPkt);

extern __inline__ const IPV4_ADDR* __attribute__((always_inline)) TCPIP_IPV4_PacketGetDestAddress(TCPIP_MAC_PACKET* pPkt)
{
    return &((IPV4_HEADER*)pPkt->pNetLayer)->DestAddress;
}

// *****************************************************************************
/*
  Function:
    const IPV4_ADDR*  TCPIP_IPV4_PacketGetSourceAddress(TCPIP_MAC_PACKET* pPkt);

  Summary:
    Returns the IPv4 source address associated with a TCPIP_MAC_PACKET

  Description:
    The function will return a pointer to where the IPv4 source address
    is located in the TCPIP_MAC_PACKET.
    The TCPIP_MAC_PACKET is supposed to be a valid IPv4 packet that has
    properly source address set.

  Precondition:
    pPkt - valid IPv4 packet, pNetLayer filed properly set
        

  Parameters:
    pPkt - packet to query


  Returns:
    - A valid pointer to an IPV4_ADDR - if it succeeds
    - 0 - if call failed
      
  Remarks:
    This function is primarily meant for RX packets.
 */
const IPV4_ADDR*  TCPIP_IPV4_PacketGetSourceAddress(TCPIP_MAC_PACKET* pPkt);

extern __inline__ const IPV4_ADDR* __attribute__((always_inline)) TCPIP_IPV4_PacketGetSourceAddress(TCPIP_MAC_PACKET* pPkt)
{
    return &((IPV4_HEADER*)pPkt->pNetLayer)->SourceAddress;
}

// *****************************************************************************
/*
  Function:
    TCPIP_IPV4_FILTER_TYPE    TCPIP_IPV4_PacketFilterSet(TCPIP_IPV4_FILTER_TYPE filtType);

  Summary:
    Sets the IPV4 packet filters

  Description:
    The function will set the IPv4 packet filters.
    The filters that are present in the mask will be set.
    Other filters won't be touched.

  Precondition:
    filtType - valid IPv4 filter
    IPv4 properly initialiazed
        

  Parameters:
    filtType - packet filter mask to set


  Returns:
    - the current value of the IPV4 packet filters after this mask was applied.
      
  Remarks:
    None.
 */
TCPIP_IPV4_FILTER_TYPE    TCPIP_IPV4_PacketFilterSet(TCPIP_IPV4_FILTER_TYPE filtType);


// *****************************************************************************
/*
  Function:
    TCPIP_IPV4_FILTER_TYPE    TCPIP_IPV4_PacketFilterClear(TCPIP_IPV4_FILTER_TYPE filtType);

  Summary:
    Clears the IPV4 packet filters

  Description:
    The function will clear the IPv4 packet filters.
    The filters that are present in the mask will be cleared.
    Other filters won't be touched.

  Precondition:
    filtType - valid IPv4 filter
    IPv4 properly initialiazed
        

  Parameters:
    filtType - packet filter mask to clear


  Returns:
    - the current value of the IPV4 packet filters after this mask was applied.
      
  Remarks:
    None.
 */
TCPIP_IPV4_FILTER_TYPE    TCPIP_IPV4_PacketFilterClear(TCPIP_IPV4_FILTER_TYPE filtType);


// *****************************************************************************
/*
  Function:
    void  TCPIP_IPV4_Task(void)

  Summary:
    Standard TCP/IP stack module task function.

  Description:
    This function performs IPv4 module tasks in the TCP/IP stack.

  Precondition:
    The IPv4 module should have been initialized.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
*/
void  TCPIP_IPV4_Task(void);

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // __IPV4_H_ 
