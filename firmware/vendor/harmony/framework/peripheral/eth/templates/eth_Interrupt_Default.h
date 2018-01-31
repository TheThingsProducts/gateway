/*******************************************************************************
  ETH Peripheral Library Template Implementation

  File Name:
    eth_Interrupt_Default.h

  Summary:
    ETH PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : Interrupt
    and its Variant : Default
    For following APIs :
        PLIB_ETH_InterruptSourceEnable
        PLIB_ETH_InterruptSourceDisable
        PLIB_ETH_InterruptSourceIsEnabled
        PLIB_ETH_InterruptSourcesGet
        PLIB_ETH_InterruptSet
        PLIB_ETH_InterruptClear
        PLIB_ETH_InterruptsGet
        PLIB_ETH_InterruptStatusGet
        PLIB_ETH_ExistsInterrupt

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

#ifndef _ETH_INTERRUPT_DEFAULT_H
#define _ETH_INTERRUPT_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _ETH_TX_BUS_ERROR_ENABLE_VREG(index)
    _ETH_TX_BUS_ERROR_INT_VREG(index)
    _ETH_RX_BUS_ERROR_ENABLE_VREG(index)
    _ETH_RX_BUS_ERROR_INT_VREG(index)
    _ETH_EMPTY_WMARK_ENABLE_VREG(index)
    _ETH_EMPTY_WMARK_INT_VREG(index)
    _ETH_FULL_WMARK_ENABLE_VREG(index)
    _ETH_FULL_WMARK_INT_VREG(index)
    _ETH_RX_DONE_ENABLE_VREG(index)
    _ETH_RX_DONE_INT_VREG(index)
    _ETH_PACKET_PENDING_ENABLE_VREG(index)
    _ETH_PACKET_PENDING_INT_VREG(index)
    _ETH_RX_ACTIVITY_ENABLE_VREG(index)
    _ETH_RX_ACTIVITY_INT_VREG(index)
    _ETH_TX_DONE_ENABLE_VREG(index)
    _ETH_TX_DONE_INT_VREG(index)
    _ETH_TX_ABORT_ENABLE_VREG(index)
    _ETH_TX_ABORT_INT_VREG(index)
    _ETH_RX_BUFFER_NOTAVAIL_ENABLE_VREG(index)
    _ETH_RX_BUFFER_NOTAVAIL_INT_VREG(index)
    _ETH_RX_FIFO_OVERFLOW_ENABLE_VREG(index)
    _ETH_RX_FIFO_OVERFLOW_INT_VREG(index)

  MASKs:
    _ETH_TX_BUS_ERROR_ENABLE_MASK(index)
    _ETH_TX_BUS_ERROR_INT_MASK(index)
    _ETH_RX_BUS_ERROR_ENABLE_MASK(index)
    _ETH_RX_BUS_ERROR_INT_MASK(index)
    _ETH_EMPTY_WMARK_ENABLE_MASK(index)
    _ETH_EMPTY_WMARK_INT_MASK(index)
    _ETH_FULL_WMARK_ENABLE_MASK(index)
    _ETH_FULL_WMARK_INT_MASK(index)
    _ETH_RX_DONE_ENABLE_MASK(index)
    _ETH_RX_DONE_INT_MASK(index)
    _ETH_PACKET_PENDING_ENABLE_MASK(index)
    _ETH_PACKET_PENDING_INT_MASK(index)
    _ETH_RX_ACTIVITY_ENABLE_MASK(index)
    _ETH_RX_ACTIVITY_INT_MASK(index)
    _ETH_TX_DONE_ENABLE_MASK(index)
    _ETH_TX_DONE_INT_MASK(index)
    _ETH_TX_ABORT_ENABLE_MASK(index)
    _ETH_TX_ABORT_INT_MASK(index)
    _ETH_RX_BUFFER_NOTAVAIL_ENABLE_MASK(index)
    _ETH_RX_BUFFER_NOTAVAIL_INT_MASK(index)
    _ETH_RX_FIFO_OVERFLOW_ENABLE_MASK(index)
    _ETH_RX_FIFO_OVERFLOW_INT_MASK(index)

  POSs:
    _ETH_TX_BUS_ERROR_ENABLE_POS(index)
    _ETH_TX_BUS_ERROR_INT_POS(index)
    _ETH_RX_BUS_ERROR_ENABLE_POS(index)
    _ETH_RX_BUS_ERROR_INT_POS(index)
    _ETH_EMPTY_WMARK_ENABLE_POS(index)
    _ETH_EMPTY_WMARK_INT_POS(index)
    _ETH_FULL_WMARK_ENABLE_POS(index)
    _ETH_FULL_WMARK_INT_POS(index)
    _ETH_RX_DONE_ENABLE_POS(index)
    _ETH_RX_DONE_INT_POS(index)
    _ETH_PACKET_PENDING_ENABLE_POS(index)
    _ETH_PACKET_PENDING_INT_POS(index)
    _ETH_RX_ACTIVITY_ENABLE_POS(index)
    _ETH_RX_ACTIVITY_INT_POS(index)
    _ETH_TX_DONE_ENABLE_POS(index)
    _ETH_TX_DONE_INT_POS(index)
    _ETH_TX_ABORT_ENABLE_POS(index)
    _ETH_TX_ABORT_INT_POS(index)
    _ETH_RX_BUFFER_NOTAVAIL_ENABLE_POS(index)
    _ETH_RX_BUFFER_NOTAVAIL_INT_POS(index)
    _ETH_RX_FIFO_OVERFLOW_ENABLE_POS(index)
    _ETH_RX_FIFO_OVERFLOW_INT_POS(index)

  LENs:
    _ETH_TX_BUS_ERROR_ENABLE_LEN(index)
    _ETH_TX_BUS_ERROR_INT_LEN(index)
    _ETH_RX_BUS_ERROR_ENABLE_LEN(index)
    _ETH_RX_BUS_ERROR_INT_LEN(index)
    _ETH_EMPTY_WMARK_ENABLE_LEN(index)
    _ETH_EMPTY_WMARK_INT_LEN(index)
    _ETH_FULL_WMARK_ENABLE_LEN(index)
    _ETH_FULL_WMARK_INT_LEN(index)
    _ETH_RX_DONE_ENABLE_LEN(index)
    _ETH_RX_DONE_INT_LEN(index)
    _ETH_PACKET_PENDING_ENABLE_LEN(index)
    _ETH_PACKET_PENDING_INT_LEN(index)
    _ETH_RX_ACTIVITY_ENABLE_LEN(index)
    _ETH_RX_ACTIVITY_INT_LEN(index)
    _ETH_TX_DONE_ENABLE_LEN(index)
    _ETH_TX_DONE_INT_LEN(index)
    _ETH_TX_ABORT_ENABLE_LEN(index)
    _ETH_TX_ABORT_INT_LEN(index)
    _ETH_RX_BUFFER_NOTAVAIL_ENABLE_LEN(index)
    _ETH_RX_BUFFER_NOTAVAIL_INT_LEN(index)
    _ETH_RX_FIFO_OVERFLOW_ENABLE_LEN(index)
    _ETH_RX_FIFO_OVERFLOW_INT_LEN(index)

*/


//******************************************************************************
/* Function :  ETH_InterruptSourceEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_InterruptSourceEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_InterruptSourceEnable function.
*/

PLIB_TEMPLATE void ETH_InterruptSourceEnable_Default( ETH_MODULE_ID index , ETH_INTERRUPT_SOURCES intmask )
{
    _SFR_SET(_ETH_RX_FIFO_OVERFLOW_ENABLE_VREG(index),(uint32_t)intmask);
}


//******************************************************************************
/* Function :  ETH_InterruptSourceDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_InterruptSourceDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_InterruptSourceDisable function.
*/

PLIB_TEMPLATE void ETH_InterruptSourceDisable_Default( ETH_MODULE_ID index , ETH_INTERRUPT_SOURCES intmask )
{
    _SFR_CLEAR(_ETH_RX_FIFO_OVERFLOW_ENABLE_VREG(index),(uint32_t)intmask);
}


//******************************************************************************
/* Function :  ETH_InterruptSourceIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_InterruptSourceIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_InterruptSourceIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_InterruptSourceIsEnabled_Default( ETH_MODULE_ID index , ETH_INTERRUPT_SOURCES intmask )
{
    return ( _SFR_READ(_ETH_RX_FIFO_OVERFLOW_ENABLE_VREG(index)) & intmask ) > 0 ? true : false;
}


//******************************************************************************
/* Function :  ETH_InterruptSourcesGet_Default

  Summary:
    Implements Default variant of PLIB_ETH_InterruptSourcesGet

  Description:
    This template implements the Default variant of the PLIB_ETH_InterruptSourcesGet function.
*/

PLIB_TEMPLATE ETH_INTERRUPT_SOURCES ETH_InterruptSourcesGet_Default( ETH_MODULE_ID index )
{
    return (ETH_INTERRUPT_SOURCES)_SFR_READ(_ETH_RX_FIFO_OVERFLOW_ENABLE_VREG(index));
}


//******************************************************************************
/* Function :  ETH_InterruptSet_Default

  Summary:
    Implements Default variant of PLIB_ETH_InterruptSet

  Description:
    This template implements the Default variant of the PLIB_ETH_InterruptSet function.
*/

PLIB_TEMPLATE void ETH_InterruptSet_Default( ETH_MODULE_ID index , ETH_INTERRUPT_SOURCES intmask )
{
    _SFR_SET(_ETH_RX_FIFO_OVERFLOW_INT_VREG(index),intmask);
}


//******************************************************************************
/* Function :  ETH_InterruptClear_Default

  Summary:
    Implements Default variant of PLIB_ETH_InterruptClear

  Description:
    This template implements the Default variant of the PLIB_ETH_InterruptClear function.
*/

PLIB_TEMPLATE void ETH_InterruptClear_Default( ETH_MODULE_ID index , ETH_INTERRUPT_SOURCES intmask )
{
    _SFR_CLEAR(_ETH_RX_FIFO_OVERFLOW_INT_VREG(index),intmask);
}


//******************************************************************************
/* Function :  ETH_InterruptsGet_Default

  Summary:
    Implements Default variant of PLIB_ETH_InterruptsGet

  Description:
    This template implements the Default variant of the PLIB_ETH_InterruptsGet function.
*/

PLIB_TEMPLATE ETH_INTERRUPT_SOURCES ETH_InterruptsGet_Default( ETH_MODULE_ID index )
{
    return  (ETH_INTERRUPT_SOURCES)_SFR_READ(_ETH_RX_FIFO_OVERFLOW_INT_VREG(index));
}


//******************************************************************************
/* Function :  ETH_InterruptStatusGet_Default

  Summary:
    Implements Default variant of PLIB_ETH_InterruptStatusGet

  Description:
    This template implements the Default variant of the PLIB_ETH_InterruptStatusGet function.
*/

PLIB_TEMPLATE bool ETH_InterruptStatusGet_Default( ETH_MODULE_ID index , ETH_INTERRUPT_SOURCES intmask )
{
    return  (_SFR_READ(_ETH_RX_FIFO_OVERFLOW_INT_VREG(index)) & intmask)  > 0 ? true : false;
}


//******************************************************************************
/* Function :  ETH_ExistsInterrupt_Default

  Summary:
    Implements Default variant of PLIB_ETH_ExistsInterrupt

  Description:
    This template implements the Default variant of the PLIB_ETH_ExistsInterrupt function.
*/

#define PLIB_ETH_ExistsInterrupt PLIB_ETH_ExistsInterrupt
PLIB_TEMPLATE bool ETH_ExistsInterrupt_Default( ETH_MODULE_ID index )
{
    return true;
}


#endif /*_ETH_INTERRUPT_DEFAULT_H*/

/******************************************************************************
 End of File
*/

