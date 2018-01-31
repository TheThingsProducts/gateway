/*******************************************************************************
  ETH Peripheral Library Template Implementation

  File Name:
    eth_ReceiveBufferSize_Default.h

  Summary:
    ETH PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ReceiveBufferSize
    and its Variant : Default
    For following APIs :
        PLIB_ETH_ReceiveBufferSizeGet
        PLIB_ETH_ReceiveBufferSizeSet
        PLIB_ETH_ExistsReceiveBufferSize

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

#ifndef _ETH_RECEIVEBUFFERSIZE_DEFAULT_H
#define _ETH_RECEIVEBUFFERSIZE_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _ETH_RX_ENABLE_VREG(index)
    _ETH_RX_DATA_BUFFER_SIZE_VREG(index)

  MASKs:
    _ETH_RX_ENABLE_MASK(index)
    _ETH_RX_DATA_BUFFER_SIZE_MASK(index)

  POSs:
    _ETH_RX_ENABLE_POS(index)
    _ETH_RX_DATA_BUFFER_SIZE_POS(index)

  LENs:
    _ETH_RX_ENABLE_LEN(index)
    _ETH_RX_DATA_BUFFER_SIZE_LEN(index)

*/


//******************************************************************************
/* Function :  ETH_ReceiveBufferSizeGet_Default

  Summary:
    Implements Default variant of PLIB_ETH_ReceiveBufferSizeGet

  Description:
    This template implements the Default variant of the PLIB_ETH_ReceiveBufferSizeGet function.
*/

PLIB_TEMPLATE uint8_t ETH_ReceiveBufferSizeGet_Default( ETH_MODULE_ID index )
{
    return (uint8_t)_SFR_FIELD_READ(_ETH_RX_DATA_BUFFER_SIZE_VREG(index),
                                    _ETH_RX_DATA_BUFFER_SIZE_MASK(index),
                                    _ETH_RX_DATA_BUFFER_SIZE_POS(index));
}


//******************************************************************************
/* Function :  ETH_ReceiveBufferSizeSet_Default

  Summary:
    Implements Default variant of PLIB_ETH_ReceiveBufferSizeSet

  Description:
    This template implements the Default variant of the PLIB_ETH_ReceiveBufferSizeSet function.
*/

PLIB_TEMPLATE void ETH_ReceiveBufferSizeSet_Default( ETH_MODULE_ID index , uint8_t ReceiveBufferSize )
{
    PLIB_ASSERT(!(bool)_SFR_BIT_READ(_ETH_RX_ENABLE_VREG(index),_ETH_RX_ENABLE_POS(index)),
                "Receive must be disabled!");
    _SFR_FIELD_WRITE(_ETH_RX_DATA_BUFFER_SIZE_VREG(index),
                     _ETH_RX_DATA_BUFFER_SIZE_MASK(index),
                     _ETH_RX_DATA_BUFFER_SIZE_POS(index) ,
                      ReceiveBufferSize                   );
}


//******************************************************************************
/* Function :  ETH_ExistsReceiveBufferSize_Default

  Summary:
    Implements Default variant of PLIB_ETH_ExistsReceiveBufferSize

  Description:
    This template implements the Default variant of the PLIB_ETH_ExistsReceiveBufferSize function.
*/

#define PLIB_ETH_ExistsReceiveBufferSize PLIB_ETH_ExistsReceiveBufferSize
PLIB_TEMPLATE bool ETH_ExistsReceiveBufferSize_Default( ETH_MODULE_ID index )
{
    return true;
}


#endif /*_ETH_RECEIVEBUFFERSIZE_DEFAULT_H*/

/******************************************************************************
 End of File
*/

