/*******************************************************************************
  USART Peripheral Library Template Implementation

  File Name:
    usart_ReceiverInterruptMode_Default.h

  Summary:
    USART PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ReceiverInterruptMode
    and its Variant : Default
    For following APIs :
        PLIB_USART_ExistsReceiverInterruptMode
        PLIB_USART_ReceiverInterruptModeSelect
        PLIB_USART_InitializeOperation

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

#ifndef _USART_RECEIVERINTERRUPTMODE_DEFAULT_H
#define _USART_RECEIVERINTERRUPTMODE_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    _USART_RECEIVER_FIFO_LEVEL_VREG(index)

  MASKs: 
    _USART_RECEIVER_FIFO_LEVEL_MASK(index)

  POSs: 
    _USART_RECEIVER_FIFO_LEVEL_POS(index)

  LENs: 
    _USART_RECEIVER_FIFO_LEVEL_LEN(index)

*/


//******************************************************************************
/* Function :  USART_ExistsReceiverInterruptMode_Default

  Summary:
    Implements Default variant of PLIB_USART_ExistsReceiverInterruptMode

  Description:
    This template implements the Default variant of the PLIB_USART_ExistsReceiverInterruptMode function.
*/

#define PLIB_USART_ExistsReceiverInterruptMode PLIB_USART_ExistsReceiverInterruptMode
PLIB_TEMPLATE bool USART_ExistsReceiverInterruptMode_Default( USART_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  USART_ReceiverInterruptModeSelect_Default

  Summary:
    Implements Default variant of PLIB_USART_ReceiverInterruptModeSelect 

  Description:
    This template implements the Default variant of the PLIB_USART_ReceiverInterruptModeSelect function.
*/

PLIB_TEMPLATE void USART_ReceiverInterruptModeSelect_Default( USART_MODULE_ID index , USART_RECEIVE_INTR_MODE interruptMode )
{
    _SFR_FIELD_WRITE(_USART_RECEIVER_FIFO_LEVEL_VREG(index),
                    _USART_RECEIVER_FIFO_LEVEL_MASK(index),
                    _USART_RECEIVER_FIFO_LEVEL_POS(index),
                    interruptMode);
}


//******************************************************************************
/* Function :  USART_InitializeOperation_Default

  Summary:
    Implements Default variant of PLIB_USART_InitializeOperation 

  Description:
    This template implements the Default variant of the PLIB_USART_InitializeOperation function.
*/

PLIB_TEMPLATE void USART_InitializeOperation_Default( USART_MODULE_ID index , USART_RECEIVE_INTR_MODE receiveInterruptMode ,
        USART_TRANSMIT_INTR_MODE transmitInterruptMode, USART_OPERATION_MODE operationMode)
{
    SFR_TYPE * usta = _USART_RECEIVER_FIFO_LEVEL_VREG(index);
    SFR_TYPE * umode = _USART_ENABLE_CONTROL_VREG(index);

    uint32_t value = 0;

    // Clear the values initially
    *usta &= ~0xCFCF;
    *umode &= ~0x0300;

    value |= (receiveInterruptMode << 6);
    value |= (transmitInterruptMode << 14);
    *usta |= value;
    value = 0;
    value |= (operationMode << 8);
    *umode |= value; 
}


#endif /*_USART_RECEIVERINTERRUPTMODE_DEFAULT_H*/

/******************************************************************************
 End of File
*/

