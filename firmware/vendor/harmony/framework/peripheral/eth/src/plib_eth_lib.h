/*******************************************************************************
  Ethernet Library Interface Definition 

  Company:
    Microchip Technology Inc.
    
  File Name:
    plib_eth_lib.h

  Summary:
    This file contains the Application Program Interface (API) definition  for
    the Ethernet peripheral library.

  Description:
    This library provides a low-level abstraction of the Ethernet module
    on Microchip PIC32MX family microcontrollers with a convenient C language
    interface.  It can be used to simplify low-level access to the module
    without the necessity of interacting directly with the module's registers,
    thus hiding differences from one microcontroller variant to another.
*******************************************************************************/
//DOM-IGNORE-BEGIN
/*******************************************************************************
Compiler:       Microchip MPLAB C32 v1.00 or higher
Copyright © 2013 released Microchip Technology Inc.  All rights
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
//DOM-IGNORE-END


#ifndef _PLIB_ETH_LIB_H_
#define _PLIB_ETH_LIB_H_

#include <stdlib.h>
#include <xc.h>


#include "peripheral/eth/plib_eth.h"
#if defined(_ETHRXFC_CRCERREN_MASK)
// *****************************************************************************
// *****************************************************************************
// Section: Constants & Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Ethernet Receive Filter Flags

  Summary:
    Defines the receive filter flags

  Description:
    This enumeration defines the receive filter flags.

  Remarks:
    Multiple values can be OR-ed together.

    The values in this enumeration are displayed in the order of priority
    that the receive filter state machine works, with the highest priority first.
    Once a filter accepts or rejects a packet, further filters are not tried.
    If a packet isn't rejected/accepted after all filters are tried, it will be rejected by
    default!
*/

typedef enum
{
    // Frames with wrong CRC are accepted
    ETH_FILT_CRC_ERR_ACCEPT
        /*DOM-IGNORE-BEGIN*/ = _ETHRXFC_CRCERREN_MASK /*DOM-IGNORE-END*/,

    // Runt frames accepted
    ETH_FILT_RUNT_ACCEPT
        /*DOM-IGNORE-BEGIN*/ = _ETHRXFC_RUNTERREN_MASK /*DOM-IGNORE-END*/,

    // Frames with wrong CRC are rejected
    ETH_FILT_CRC_ERR_REJECT
        /*DOM-IGNORE-BEGIN*/ = _ETHRXFC_CRCOKEN_MASK /*DOM-IGNORE-END*/,

    // Runt frames rejected
    ETH_FILT_RUNT_REJECT
        /*DOM-IGNORE-BEGIN*/ = _ETHRXFC_RUNTEN_MASK /*DOM-IGNORE-END*/,

    // Me unicast accepted
    ETH_FILT_ME_UCAST_ACCEPT
        /*DOM-IGNORE-BEGIN*/ = _ETHRXFC_UCEN_MASK /*DOM-IGNORE-END*/,

    // Not me unicast accepted
    ETH_FILT_NOTME_UCAST_ACCEPT
        /*DOM-IGNORE-BEGIN*/ = _ETHRXFC_NOTMEEN_MASK /*DOM-IGNORE-END*/,

    // Multicast accepted
    ETH_FILT_MCAST_ACCEPT
        /*DOM-IGNORE-BEGIN*/ = _ETHRXFC_MCEN_MASK /*DOM-IGNORE-END*/,

    // Broadcast accepted
    ETH_FILT_BCAST_ACCEPT
        /*DOM-IGNORE-BEGIN*/ = _ETHRXFC_BCEN_MASK /*DOM-IGNORE-END*/,

    // Hash table matches destination address accepted
    ETH_FILT_HTBL_ACCEPT
        /*DOM-IGNORE-BEGIN*/ = _ETHRXFC_HTEN_MASK /*DOM-IGNORE-END*/,

    // Magic packet accepted
    ETH_FILT_MAGICP_ACCEPT
        /*DOM-IGNORE-BEGIN*/ = _ETHRXFC_MPEN_MASK /*DOM-IGNORE-END*/,


    // All Filters
    ETH_FILT_ALL_FILTERS
     /*DOM-IGNORE-BEGIN*/    = ETH_FILT_CRC_ERR_ACCEPT  | ETH_FILT_RUNT_ACCEPT        |
                               ETH_FILT_CRC_ERR_REJECT  | ETH_FILT_RUNT_REJECT        |
                               ETH_FILT_ME_UCAST_ACCEPT | ETH_FILT_NOTME_UCAST_ACCEPT |
                               ETH_FILT_MCAST_ACCEPT    | ETH_FILT_BCAST_ACCEPT       |
                               ETH_FILT_HTBL_ACCEPT     | ETH_FILT_MAGICP_ACCEPT  /*DOM-IGNORE-END*/

} ETH_RX_FILTERS;


// *****************************************************************************
/* Ethernet Pattern Match Modes

  Summary:
    Defines pattern match modes

  Description:
    This enumeration defines the Ethernet pattern match modes.

  Remarks:
    These are mutually exclusive modes, not flags.  However,
    ETH_FILT_PMATCH_INVERT act as a flag and can be applied to any value.
*/

typedef enum
{
    // Simple Pattern Match accepted
    ETH_FILT_PMATCH_ACCEPT /*DOM-IGNORE-BEGIN*/ = 1/*DOM-IGNORE-END*/,

    // Pattern Match AND destination==me
    ETH_FILT_PMATCH_ME_UCAST_ACCEPT,

    // Pattern Match AND destination!=me
    ETH_FILT_PMATCH_NOTME_UCAST_ACCEPT,

    // Pattern Match AND destination!=unicast
    ETH_FILT_PMATCH_MCAST_ACCEPT,

    // Pattern Match AND destination==unicast
    ETH_FILT_PMATCH_NOT_MCAST_ACCEPT,

    // Pattern Match AND destination==broadcast
    ETH_FILT_PMATCH_BCAST_ACCEPT,

    // Pattern Match AND destination!=broadcast
    ETH_FILT_PMATCH_NOT_BCAST_ACCEPT,

    // Pattern Match AND hash table filter match (regardless of the
    // ETH_FILT_HTBL_ACCEPT setting)
    ETH_FILT_PMATCH_HTBL_ACCEPT,

    // Pattern Match AND packet ==magic packet
    ETH_FILT_PMATCH_MAGICP_ACCEPT,

    // If set, the pattern must NOT match for a successful Pattern Match to occur!
    ETH_FILT_PMATCH_INVERT /*DOM-IGNORE-BEGIN*/ = 0x80000000 /*DOM-IGNORE-END*/

} ETH_PMATCH_MODE;


// *****************************************************************************
/* Ethernet Event Flags

  Summary:
    Ethernet event flags

  Description:
    This enumeration defines flags for the possible Ethernet events that can
    cause interrupts.
*/

typedef enum
{
    // RX FIFO overflow
    ETH_EV_RXOVFLOW
        /*DOM-IGNORE-BEGIN*/ = _ETHIRQ_RXOVFLW_MASK /*DOM-IGNORE-END*/,

    // RX buffer not available (descriptor overrun)
    ETH_EV_RXBUFNA
        /*DOM-IGNORE-BEGIN*/ = _ETHIRQ_RXBUFNA_MASK /*DOM-IGNORE-END*/,

    // TX abort condition
    ETH_EV_TXABORT
        /*DOM-IGNORE-BEGIN*/ = _ETHIRQ_TXABORT_MASK /*DOM-IGNORE-END*/,

    // TX done
    ETH_EV_TXDONE
        /*DOM-IGNORE-BEGIN*/ = _ETHIRQ_TXDONE_MASK  /*DOM-IGNORE-END*/,

    // RX activity
    ETH_EV_RXACT
        /*DOM-IGNORE-BEGIN*/ = _ETHIRQ_RXACT_MASK   /*DOM-IGNORE-END*/,

    // RX packet pending
    ETH_EV_PKTPEND
        /*DOM-IGNORE-BEGIN*/ = _ETHIRQ_PKTPEND_MASK /*DOM-IGNORE-END*/,

    // RX packet successfully received
    ETH_EV_RXDONE
        /*DOM-IGNORE-BEGIN*/ = _ETHIRQ_RXDONE_MASK  /*DOM-IGNORE-END*/,

    // Full watermark reached
    ETH_EV_FWMARK
        /*DOM-IGNORE-BEGIN*/ = _ETHIRQ_FWMARK_MASK  /*DOM-IGNORE-END*/,

    // Empty watermark reached
    ETH_EV_EWMARK
        /*DOM-IGNORE-BEGIN*/ = _ETHIRQ_EWMARK_MASK  /*DOM-IGNORE-END*/,

    // RX bus error
    ETH_EV_RXBUSERR
        /*DOM-IGNORE-BEGIN*/ = _ETHIRQ_RXBUSE_MASK  /*DOM-IGNORE-END*/,

    // TX bus error
    ETH_EV_TXBUSERR
        /*DOM-IGNORE-BEGIN*/ = _ETHIRQ_TXBUSE_MASK  /*DOM-IGNORE-END*/,

    // All events
    ETH_EV_ALL
    /*DOM-IGNORE-BEGIN*/= (ETH_EV_RXOVFLOW | ETH_EV_RXBUFNA | ETH_EV_TXABORT |
                           ETH_EV_TXDONE   | ETH_EV_RXACT   | ETH_EV_PKTPEND |
                           ETH_EV_RXDONE   | ETH_EV_FWMARK  | ETH_EV_EWMARK  |
                           ETH_EV_RXBUSERR | ETH_EV_TXBUSERR ) /*DOM-IGNORE-END*/

} PLIB_ETH_EVENTS;


/*******************************************************************************
  Function:
    void PLIB_ETH_MACSetAddress (ETH_MODULE_ID index, unsigned char bAddress[6] )

  Summary:
    Sets the MAC address

  Description:
    This function sets the MAC address.

  Precondition:
    None.

  Parameters:
    index       - Identifier for the device instance.
    bAddress    - standard MAC address, 6 bytes, Network order!

  Returns:
    None.

  Example:
    <code>
    unsigned char hostAddress[6]= { 0x00, 0x04, 0xa3, 0x00, 0x00, 0x02};

    PLIB_ETH_MACSetAddress(index,hostAddress);
    </code>

  Remarks:
    The default MAC address is loaded by the device at reset from the
    factory pre-programmed unique MAC address value.

 *****************************************************************************/

void PLIB_ETH_MACSetAddress (ETH_MODULE_ID index, unsigned char bAddress[6] );


/*******************************************************************************
  Function:
    void PLIB_ETH_MACGetAddress (ETH_MODULE_ID index, unsigned char bAddress[6] )

  Summary:
    Returns the current MAC address

  Description:
    This function returns the current MAC address.

  Precondition:
    None.

  Parameters:
    index       - Identifier for the device instance.
    bAddress    - Address to store a standard MAC address, 6 bytes, Network order!

  Returns:
    None.

  Example:
    <code>
    unsigned char currentMacAddress;

    PLIB_ETH_MACGetAddress(index,&currentMacAddress);
    </code>

  Remarks:

 *****************************************************************************/

void PLIB_ETH_MACGetAddress (ETH_MODULE_ID index, unsigned char bAddress[6] );


/*******************************************************************************
  Function:
    void PLIB_ETH_MACSetMaxFrame (ETH_MODULE_ID index, unsigned short maxFrmSz )

  Summary:
    Sets the MAC maximum frame size

  Description:
    This function sets the MAC maximum frame size.

  Precondition:
    Should be called after PLIB_ETH_Init().

  Parameters:
    index       - Identifier for the device instance.
    maxFrmSz    - Maximum frame for the MAC to transmit or receive.

  Returns:
    None.

  Example:
    <code>
    PLIB_ETH_MACSetMaxFrame(index,0x800);
    </code>

  Remarks:
    The default MAC frame size (0x600) is set by the PLIB_ETH_Init() call.

 *****************************************************************************/

void PLIB_ETH_MACSetMaxFrame (ETH_MODULE_ID index, unsigned short maxFrmSz );


/*******************************************************************************
  Function:
    void PLIB_ETH_RxFiltersSet (ETH_MODULE_ID index, ETH_RX_FILTERS rxFilters )

  Summary:
    Sets the acceptance/rejection filters for the Ethernet receive.

  Description:
    This function sets the acceptance/rejection filters for the Ethernet receive
    mechanism.  Multiple filters can be OR-ed together.  All filter specified
    in rxFilters will be set.

  Precondition:
    This function should not be called when Ethernet RX operation is enabled.

  Parameters:
    index       - Identifier for the device instance.
    rxFilters   - RX filters that are to be set

  Returns:
    None.

  Example:
    <code>
    PLIB_ETH_RxFiltersSet(index,
                    ETH_FILT_CRC_ERR_REJECT | ETH_FILT_RUNT_REJECT  |
                    ETH_FILT_ME_UCAST_ACCEPT| ETH_FILT_MCAST_ACCEPT |
                    ETH_FILT_BCAST_ACCEPT   | ETH_FILT_NOTME_UCAST_ACCEPT);
    </code>

  Remarks:
    All filters except Pattern Match can be set. Use PLIB_ETH_RxFiltersPMSet().

    Before enabling the Hash Table filter, the hash table values should be set
    using PLIB_ETH_RxFiltersHTSet().

    See the definition of the ETH_RX_FILTERS for the priority of the RX filters.

 *****************************************************************************/

void PLIB_ETH_RxFiltersSet (ETH_MODULE_ID index, ETH_RX_FILTERS rxFilters );


/*******************************************************************************
  Function:
    void PLIB_ETH_RxFiltersClr (ETH_MODULE_ID index, ETH_RX_FILTERS rxFilters )

  Summary:
    Clears the acceptance/rejection filters for the Ethernet receive.

  Description:
    This function clears the acceptance/rejection filters for the Ethernet receive
    mechanism.  Multiple filters can be OR-ed together.  All filters specified
    in rxFilters will be cleared.

  Precondition:
    This function should not be called when Ethernet RX operation is enabled.

  Parameters:
    index       - Identifier for the device instance.
    rxFilters   - RX filters that are to be cleared

  Returns:
    None.

  Example:
    <code>
    PLIB_ETH_RxFiltersClr(index,ETH_FILT_ALL_FILTERS);
    </code>

  Remarks:
    All filters except Pattern Match can be cleared. Use PLIB_ETH_RxFiltersPMClr().

 *****************************************************************************/

void PLIB_ETH_RxFiltersClr (ETH_MODULE_ID index, ETH_RX_FILTERS rxFilters );


/*******************************************************************************
  Function:
    void PLIB_ETH_RxFiltersWrite ( ETH_MODULE_ID index, ETH_RX_FILTERS rxFilters )

  Summary:
    Updates the acceptance/rejection filters for the Ethernet receive.

  Description:
    This function updates the acceptance/rejection filters for the Ethernet receive
    mechanism to the required value.

  Precondition:
    This function should not be called when Ethernet RX operation is enabled.

  Parameters:
    index       - Identifier for the device instance.
    rxFilters       - RX filters that are to be written

  Returns:
    None.

  Example:
    <code>
    PLIB_ETH_RxFiltersWrite(index, ETH_FILT_ALL_FILTERS);
    </code>

  Remarks:
    All filters except Pattern Match can be updated.

    The Pattern Match filter will be disabled by this call.  Use
    PLIB_ETH_RxFiltersPMSet();

    Before enabling the Hash Table filter, the hash table values should be set
    using PLIB_ETH_RxFiltersHTSet().

    See the definition of the ETH_RX_FILTERS for the priority of the
    RX filters.

 *****************************************************************************/

void PLIB_ETH_RxFiltersWrite ( ETH_MODULE_ID index, ETH_RX_FILTERS rxFilters );


/*******************************************************************************
  Function:
    void  PLIB_ETH_RxFiltersHTSet ( ETH_MODULE_ID index, uint64_t htable )

  Summary:
    Sets the hash table for the hash table RX filter.

  Description:
    This function sets the hash table for the hash table RX filter.

  Precondition:
    This function should not be called when Ethernet RX operation is enabled and the
    hash filter is active.

  Parameters:
    index       - Identifier for the device instance.
    htable      - The hash table itself

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    Properly set the hash table with this function before enabling the Hash
    Table filter.

 *****************************************************************************/

void  PLIB_ETH_RxFiltersHTSet ( ETH_MODULE_ID index, uint64_t htable );


/*******************************************************************************
  Function:
    void PLIB_ETH_RxFiltersPMSet ( ETH_MODULE_ID index,
                                   ETH_PMATCH_MODE mode,
                                   uint64_t matchMask,
                                   unsigned int matchOffs,
                                   unsigned int matchChecksum )

  Summary:
    Enables the Pattern Match filter with the specified settings.

  Description:
    This function enables the Pattern Match filter with the specified settings
    for the offset and window mask. The calculated checksum of the 64 bytes
    window starting at matchOffs and using the matchMask bytes in the window,
    is compared against matchChecksum.  If ETH_FILT_PMATCH_INVERT is used in
    the mode, then the calculated checksum must not match the passed
    matchChecksum in order for the Pattern Match to succeed.

  Precondition:
    This function should not be called when Ethernet RX operation is enabled.

  Parameters:
    index       - Identifier for the device instance.
    mode            - The required Pattern Match Mode

    matchMask       - Mask in the 64 byte window

    matchOffs       - The offset applied to the incoming data (0 to 8128)

    matchChecksum   - The 16 bit checksum used for comparison

  Returns:

  Example:
    <code>
    PLIB_ETH_RxFiltersPMSet(index, ETH_FILT_PMATCH_ACCEPT, MY_PATTERN, 0, MY_CHECKSUM);
    </code>

  Remarks:
 *****************************************************************************/

void PLIB_ETH_RxFiltersPMSet ( ETH_MODULE_ID index,
                              ETH_PMATCH_MODE mode, uint64_t matchMask,
                              unsigned int matchOffs, unsigned int matchChecksum );


/*******************************************************************************
  Function:
    void PLIB_ETH_RxFiltersPMClr ( ETH_MODULE_ID index )

  Summary:
    Disables the Pattern Match receive filter.

  Description:
    This function disables the Pattern Match receive filter.

  Precondition:
    This function should not be called when Ethernet RX operation is enabled.

  Parameters:
    index       - Identifier for the device instance.

  Returns:
    None.

  Example:
    <code>
    PLIB_ETH_RxFiltersPMClr(index);
    </code>

  Remarks:
 *****************************************************************************/

void PLIB_ETH_RxFiltersPMClr ( ETH_MODULE_ID index );


/*******************************************************************************
  Function:
    void PLIB_ETH_EventsEnableSet ( ETH_MODULE_ID index, PLIB_ETH_EVENTS eEvents )

  Summary:
    Enables the events that will generate interrupts for the Ethernet controller.

  Description:
    The function enables the events that will generate interrupts for the Ethernet
    controller.  Multiple events can be OR-ed together.  Any event that is set
    in the eEvents will be enabled , the other events won't be touched.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance.
    eEvents  - Events with the significance described in the PLIB_ETH_EVENTS definition.

  Returns:
    None.

  Example:
    <code>
    PLIB_ETH_EventsEnableSet(index,ETH_EV_ALL);
    </code>

  Remarks:
 *****************************************************************************/

void PLIB_ETH_EventsEnableSet ( ETH_MODULE_ID index, PLIB_ETH_EVENTS eEvents );


/*******************************************************************************
  Function:
    void PLIB_ETH_EventsEnableClr ( ETH_MODULE_ID index, PLIB_ETH_EVENTS eEvents )

  Summary:
    Disables the events that will generate interrupts.

  Description:
    The function disables the events that will generate interrupts for the Ethernet
    controller.  Multiple events can be OR-ed together.  Any event that is set
    in the eEvents will be disabled , the other events won't be touched.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance.
    eEvents  - Events with the significance described in the PLIB_ETH_EVENTS definition.

  Returns:
    None.

  Example:
    <code>
    PLIB_ETH_EventsEnableClr(index,ETH_EV_ALL);
    </code>

  Remarks:
 *****************************************************************************/

void PLIB_ETH_EventsEnableClr ( ETH_MODULE_ID index, PLIB_ETH_EVENTS eEvents );


/*******************************************************************************
  Function:
    void PLIB_ETH_EventsEnableWrite ( ETH_MODULE_ID index, PLIB_ETH_EVENTS eEvents )

  Summary:
    Enables the events that will generate interrupts for the Ethernet controller.

  Description:
    This function enables the events that will generate interrupts for the Ethernet
    controller.  The enabled events are forced to the eEvents value.

  Precondition:
    None.

  Parameters:
    index   - Identifier for the device instance.
    eEvents - Events with the significance described in the PLIB_ETH_EVENTS definition.

  Returns:
    None.

  Example:
    <code>
    PLIB_ETH_EventsEnableWrite( index,
                          ETH_EV_RXOVFLOW | ETH_EV_RXBUFNA | ETH_EV_TXABORT  |
                          ETH_EV_RXACT    | ETH_EV_PKTPEND | ETH_EV_RXBUSERR |
                          ETH_EV_TXBUSERR );
    </code>

  Remarks:
 *****************************************************************************/

void PLIB_ETH_EventsEnableWrite ( ETH_MODULE_ID index, PLIB_ETH_EVENTS eEvents );


/*******************************************************************************
  Function:
    PLIB_ETH_EVENTS PLIB_ETH_EventsEnableGet ( ETH_MODULE_ID index )

  Summary:
    Returns the enabled events for the Ethernet controller.

  Description:
    This function returns the enabled events for the Ethernet controller.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    index   - Identifier for the device instance.
    eEvents - events with the significance described in the PLIB_ETH_EVENTS
    definition.

  Example:
    <code>
    ethEnabledEvents = PLIB_ETH_EventsEnableGet(index);
    </code>

  Remarks:
 *****************************************************************************/

PLIB_ETH_EVENTS PLIB_ETH_EventsEnableGet ( ETH_MODULE_ID index );


/*******************************************************************************
  Function:
    void PLIB_ETH_EventsClr ( ETH_MODULE_ID index, PLIB_ETH_EVENTS eEvents )

  Summary:
    Clears the selected events for the Ethernet controller.

  Description:
    The function clears the selected events for the Ethernet controller.  Multiple
    events can be OR-ed together. Any event that is set in the eEvents will be
    cleared, the other events won't be touched.

  Precondition:
    None.

  Parameters:
    index    - Identifier for the device instance.
    eEvents  - Events with the significance described in the PLIB_ETH_EVENTS definition.

  Returns:
    None.

  Example:
    <code>
    PLIB_ETH_EventsClr(index,ETH_EV_TXDONE);
    </code>

  Remarks:
    The ETH_EV_FWMARK cannot be cleared directly. It is cleared indirectly by
    PLIB_ETH_RxAcknowledgePacket/PLIB_ETH_RxAcknowledgeBuffer.

    The ETH_EV_EWMARK cannot be cleared directly. It is cleared by hardware when
    receiving a new packet.

    The ETH_EV_PKTPEND cannot be cleared directly. It is cleared indirectly by
    PLIB_ETH_RxAcknowledgePacket/PLIB_ETH_RxAcknowledgeBuffer.

 *****************************************************************************/

void PLIB_ETH_EventsClr( ETH_MODULE_ID index, PLIB_ETH_EVENTS eEvents );


/*******************************************************************************
  Function:
    PLIB_ETH_EVENTS PLIB_ETH_EventsGet ( ETH_MODULE_ID index )

  Summary:
    Returns the active events for the Ethernet controller.

  Description:
    This function returns the active events for the Ethernet controller.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.

  Returns:
    Events with the significance described in the PLIB_ETH_EVENTS definition.

  Example:
    <code>
    ethEvents = PLIB_ETH_EventsGet(index);
    </code>

  Remarks:
 *****************************************************************************/

PLIB_ETH_EVENTS PLIB_ETH_EventsGet ( ETH_MODULE_ID index );


/*******************************************************************************
  Function:
    int PLIB_ETH_StatRxOvflCnt ( ETH_MODULE_ID index )

  Summary:
    Returns the current number of dropped receive frames.

  Description:
    This function returns the current number of dropped receive frames by the
    Ethernet controller.  These are frames accepted by the RX filter but dropped due
    to internal receive error (RX FIFO overrun).

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.

  Returns:
    The number of dropped receive frames

  Example:
    <code>
    droppedFrames = PLIB_ETH_StatRxOvflCnt(index);
    </code>

  Remarks:
    This statistics register is cleared by the read operation.

    An RX overflow event is signalled by the ETH_EV_RXOVFLOW event (see
    PLIB_ETH_EVENTS definition).
 *****************************************************************************/

int PLIB_ETH_StatRxOvflCnt ( ETH_MODULE_ID index );

// DOM-IGNORE-BEGIN
#define PLIB_ETH_StatRxOvflCnt  PLIB_ETH_RxOverflowCountGet
// DOM-IGNORE-END


/*******************************************************************************
  Function:
    int PLIB_ETH_StatRxOkCnt ( ETH_MODULE_ID index )

  Summary:
    Returns the current number of successfully received frames.

  Description:
    This function returns the current number of successfully received frames by
    the Ethernet controller.  These are frames accepted by the RX filter (some of
    them may still be dropped because of an RX FIFO overrun).

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.

  Returns:
    The number of frames received OK

  Example:
    <code>
    numReceived = PLIB_ETH_StatRxOkCnt(index);
    </code>

  Remarks:
    This statistics register is cleared by the read operation.

    Frames with FCS or alignment errors will not increment this count.

 *****************************************************************************/

int PLIB_ETH_StatRxOkCnt ( ETH_MODULE_ID index );

// DOM-IGNORE-BEGIN
#define  PLIB_ETH_StatRxOkCnt  PLIB_ETH_FramesRxdOkCountGet
// DOM-IGNORE-END


/*******************************************************************************
  Function:
    int PLIB_ETH_StatRxFcsErrCnt ( ETH_MODULE_ID index )

  Summary:
    Returns the current number of frames with FCS received errors.

  Description:
    The function returns the current number of frames with FCS errors received
    by the Ethernet controller.  These are received frames that have an integral
    number of bytes.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.

  Returns:
    The number of received frames having FCS errors

  Example:
    <code>
    numRxFcsErrors = PLIB_ETH_StatRxFcsErrCnt(index);
    </code>

  Remarks:
    This statistics register is cleared by the read operation.

    Frames with alignment error do not increment this count.

 *****************************************************************************/

int PLIB_ETH_StatRxFcsErrCnt ( ETH_MODULE_ID index );

// DOM-IGNORE-BEGIN
  #define  PLIB_ETH_StatRxFcsErrCnt  PLIB_ETH_FCSErrorCountGet
// DOM-IGNORE-END


/*******************************************************************************
  Function:
    int PLIB_ETH_StatRxAlgnErrCnt ( ETH_MODULE_ID index )

  Summary:
    Returns the current number of frames with alignment received errors

  Description:
    The function returns the current number of frames with alignment errors
    received by the Ethernet controller.  These are received frames that have FCS
    error and they do not contain an integral number of bytes (aka
    dribble-nibble).

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.

  Returns:
    The number of received frames having alignment errors

  Example:
    <code>
    numAlignErrors = PLIB_ETH_StatRxAlgnErrCnt(index);
    </code>

  Remarks:
    This statistics register is cleared by the read operation.

 *****************************************************************************/

int PLIB_ETH_StatRxAlgnErrCnt ( ETH_MODULE_ID index );

// DOM-IGNORE-BEGIN
#define  PLIB_ETH_StatRxAlgnErrCnt  PLIB_ETH_AlignErrorCountGet
// DOM-IGNORE-END


/*******************************************************************************
  Function:
    int PLIB_ETH_StatTxOkCnt ( ETH_MODULE_ID index )

  Summary:
    Returns the current number of frames transmitted successfully

  Description:
    The function returns the current number of frames transmitted successfully by
    the Ethernet controller.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.

  Returns:
    The number of frames transmitted OK

  Example:
    <code>
    numFramesTx = PLIB_ETH_StatTxOkCnt(index);
    </code>

  Remarks:
    This statistics register is cleared by the read operation.

 *****************************************************************************/

int PLIB_ETH_StatTxOkCnt ( ETH_MODULE_ID index );

// DOM-IGNORE-BEGIN
#define  PLIB_ETH_StatTxOkCnt  PLIB_ETH_FramesTxdOkCountGet
// DOM-IGNORE-END


/*******************************************************************************
  Function:
    int PLIB_ETH_StatTxSColCnt ( ETH_MODULE_ID index )

  Summary:
    Returns the current number of successfully transmitted frames on the second try

  Description:
    This function returns the current number of transmitted frames that had
    a collision but were successfully transmitted by the Ethernet controller
    on the second try.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.

  Returns:
    The number of single collision frames

  Example:
    <code>
    numSingleCollisions = PLIB_ETH_StatTxSColCnt(index);
    </code>

  Remarks:
    This statistics register is cleared by the read operation.

 *****************************************************************************/

int PLIB_ETH_StatTxSColCnt ( ETH_MODULE_ID index );

// DOM-IGNORE-BEGIN
#define  PLIB_ETH_StatTxSColCnt  PLIB_ETH_SingleCollisionCountGet
// DOM-IGNORE-END


/*******************************************************************************
  Function:
    int PLIB_ETH_StatTxMColCnt( ETH_MODULE_ID index )

  Summary:
    Returns the number of current frames transmitted after more than one
    collision occurred.

  Description:
    The function returns the current number of frames successfully transmitted
    by the Ethernet controller after there was more than one collision.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the device instance.

  Returns:
    The number of multiple collision frames

  Example:
    <code>
    numMultiCollisions = PLIB_ETH_StatTxMColCnt(index);
    </code>

  Remarks:
    This statistics register is cleared by the read operation.

 *****************************************************************************/

int PLIB_ETH_StatTxMColCnt( ETH_MODULE_ID index );

// DOM-IGNORE-BEGIN
#define  PLIB_ETH_StatTxMColCnt  PLIB_ETH_MultipleCollisionCountClear

// DOM-IGNORE-END

/****************************************************************************
  Function:
    int PLIB_ETH_RxSetBufferSize(ETH_MODULE_ID index, int rxBuffSize)

  PreCondition:    0 < rxBuffSize <= 2032

  Input:           rxBuffSize - size of the rx buffers

  Output:          if succes, it returns the adjusted size of the buffer size
                   <0 for a size error

  Side Effects:    None

  Overview:        This function sets the required buffer size for the receive operation.
                   In this implementation, all receive descriptors use the same buffer size (unlike the transmission flow
                   where each descriptor can have a different buffer size).

  Note:            - This function should be part of the initialization process and shoult NOT be called when the rx process is active!
                   - The receive buffer size is always TRUNCATED to a multiple of 16 bytes.

 *****************************************************************************/

int PLIB_ETH_RxSetBufferSize(ETH_MODULE_ID index, int rxBuffSize);


#endif
#endif  // _PLIB_ETH_LIB_H_

