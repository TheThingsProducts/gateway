/*******************************************************************************
  ETH Peripheral Library Template Implementation

  File Name:
    eth_ReceiveWmarks_Default.h

  Summary:
    ETH PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ReceiveWmarks
    and its Variant : Default
    For following APIs :
        PLIB_ETH_RxFullWmarkSet
        PLIB_ETH_RxFullWmarkGet
        PLIB_ETH_RxEmptyWmarkSet
        PLIB_ETH_RxEmptyWmarkGet
        PLIB_ETH_ExistsReceiveWmarks

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

#ifndef _ETH_RECEIVEWMARKS_DEFAULT_H
#define _ETH_RECEIVEWMARKS_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _ETH_RX_FULL_WMARK_VREG(index)
    _ETH_RX_EMPTY_WMARK_VREG(index)

  MASKs:
    _ETH_RX_FULL_WMARK_MASK(index)
    _ETH_RX_EMPTY_WMARK_MASK(index)

  POSs:
    _ETH_RX_FULL_WMARK_POS(index)
    _ETH_RX_EMPTY_WMARK_POS(index)

  LENs:
    _ETH_RX_FULL_WMARK_LEN(index)
    _ETH_RX_EMPTY_WMARK_LEN(index)

*/


//******************************************************************************
/* Function :  ETH_RxFullWmarkSet_Default

  Summary:
    Implements Default variant of PLIB_ETH_RxFullWmarkSet

  Description:
    This template implements the Default variant of the PLIB_ETH_RxFullWmarkSet function.
*/

PLIB_TEMPLATE void ETH_RxFullWmarkSet_Default( ETH_MODULE_ID index , uint8_t watermarkValue )
{
    _SFR_FIELD_WRITE(_ETH_RX_FULL_WMARK_VREG(index),
                     _ETH_RX_FULL_WMARK_MASK(index),
                     _ETH_RX_FULL_WMARK_POS(index) ,
                     watermarkValue                );
}


//******************************************************************************
/* Function :  ETH_RxFullWmarkGet_Default

  Summary:
    Implements Default variant of PLIB_ETH_RxFullWmarkGet

  Description:
    This template implements the Default variant of the PLIB_ETH_RxFullWmarkGet function.
*/

PLIB_TEMPLATE uint8_t ETH_RxFullWmarkGet_Default( ETH_MODULE_ID index )
{
    return (uint8_t)(_SFR_FIELD_READ(_ETH_RX_FULL_WMARK_VREG(index),
                                     _ETH_RX_FULL_WMARK_MASK(index),
                                     _ETH_RX_FULL_WMARK_POS(index) ) );

}


//******************************************************************************
/* Function :  ETH_RxEmptyWmarkSet_Default

  Summary:
    Implements Default variant of PLIB_ETH_RxEmptyWmarkSet

  Description:
    This template implements the Default variant of the PLIB_ETH_RxEmptyWmarkSet function.
*/

PLIB_TEMPLATE void ETH_RxEmptyWmarkSet_Default( ETH_MODULE_ID index , uint8_t watermarkValue )
{
    _SFR_FIELD_WRITE(_ETH_RX_EMPTY_WMARK_VREG(index),
                     _ETH_RX_EMPTY_WMARK_MASK(index),
                     _ETH_RX_EMPTY_WMARK_POS(index) ,
                     watermarkValue                );
}


//******************************************************************************
/* Function :  ETH_RxEmptyWmarkGet_Default

  Summary:
    Implements Default variant of PLIB_ETH_RxEmptyWmarkGet

  Description:
    This template implements the Default variant of the PLIB_ETH_RxEmptyWmarkGet function.
*/

PLIB_TEMPLATE uint8_t ETH_RxEmptyWmarkGet_Default( ETH_MODULE_ID index )
{
    return (uint8_t)(_SFR_FIELD_READ(_ETH_RX_EMPTY_WMARK_VREG(index),
                                     _ETH_RX_EMPTY_WMARK_MASK(index),
                                     _ETH_RX_EMPTY_WMARK_POS(index) ) );
}


//******************************************************************************
/* Function :  ETH_ExistsReceiveWmarks_Default

  Summary:
    Implements Default variant of PLIB_ETH_ExistsReceiveWmarks

  Description:
    This template implements the Default variant of the PLIB_ETH_ExistsReceiveWmarks function.
*/

#define PLIB_ETH_ExistsReceiveWmarks PLIB_ETH_ExistsReceiveWmarks
PLIB_TEMPLATE bool ETH_ExistsReceiveWmarks_Default( ETH_MODULE_ID index )
{
    return true;
}


#endif /*_ETH_RECEIVEWMARKS_DEFAULT_H*/

/******************************************************************************
 End of File
*/

