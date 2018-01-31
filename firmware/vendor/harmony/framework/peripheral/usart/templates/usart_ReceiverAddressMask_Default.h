/*******************************************************************************
  USART Peripheral Library Template Implementation

  File Name:
    usart_ReceiverAddressMask_Default.h

  Summary:
    USART PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ReceiverAddressMask
    and its Variant : Default
    For following APIs :
        PLIB_USART_ExistsReceiverAddressMask
        PLIB_USART_AddressMaskSet
        PLIB_USART_AddressMaskGet

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _USART_RECEIVERADDRESSMASK_DEFAULT_H
#define _USART_RECEIVERADDRESSMASK_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    _USART_RECEIVER_ADDRESS_MASK_VREG(index)

  MASKs: 
    _USART_RECEIVER_ADDRESS_MASK_MASK(index)

  POSs: 
    _USART_RECEIVER_ADDRESS_MASK_POS(index)

  LENs: 
    _USART_RECEIVER_ADDRESS_MASK_LEN(index)

*/


//******************************************************************************
/* Function :  USART_ExistsReceiverAddressMask_Default

  Summary:
    Implements Default variant of PLIB_USART_ExistsReceiverAddressMask

  Description:
    This template implements the Default variant of the PLIB_USART_ExistsReceiverAddressMask function.
*/

#define PLIB_USART_ExistsReceiverAddressMask PLIB_USART_ExistsReceiverAddressMask
PLIB_TEMPLATE bool USART_ExistsReceiverAddressMask_Default( USART_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  USART_AddressMaskSet_Default

  Summary:
    Implements Default variant of PLIB_USART_AddressMaskSet 

  Description:
    This template implements the Default variant of the PLIB_USART_AddressMaskSet function.
*/

PLIB_TEMPLATE void USART_AddressMaskSet_Default( USART_MODULE_ID index , uint8_t mask )
{
   _SFR_FIELD_WRITE(_USART_RECEIVER_ADDRESS_MASK_VREG(index),
                    _USART_RECEIVER_ADDRESS_MASK_MASK(index),
                    _USART_RECEIVER_ADDRESS_MASK_POS(index),
                    mask);
}


//******************************************************************************
/* Function :  USART_AddressMaskGet_Default

  Summary:
    Implements Default variant of PLIB_USART_AddressMaskGet 

  Description:
    This template implements the Default variant of the PLIB_USART_AddressMaskGet function.
*/

PLIB_TEMPLATE uint8_t USART_AddressMaskGet_Default( USART_MODULE_ID index )
{
    return (uint8_t)_SFR_FIELD_READ(_USART_RECEIVER_ADDRESS_MASK_VREG(index),
                           _USART_RECEIVER_ADDRESS_MASK_MASK(index),
                           _USART_RECEIVER_ADDRESS_MASK_POS(index));
}


#endif /*_USART_RECEIVERADDRESSMASK_DEFAULT_H*/

/******************************************************************************
 End of File
*/

