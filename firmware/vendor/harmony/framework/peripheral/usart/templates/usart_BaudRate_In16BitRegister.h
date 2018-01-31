/*******************************************************************************
  USART Peripheral Library Template Implementation

  File Name:
    usart_BaudRate_In16BitRegister.h

  Summary:
    USART PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : BaudRate
    and its Variant : In16BitRegister
    For following APIs :
        PLIB_USART_ExistsBaudRate
        PLIB_USART_BaudRateSet
        PLIB_USART_BaudRateGet

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

#ifndef _USART_BAUDRATE_IN16BITREGISTER_H
#define _USART_BAUDRATE_IN16BITREGISTER_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    _USART_BAUD_RATE_VREG(index)

  MASKs: 
    _USART_BAUD_RATE_MASK(index)

  POSs: 
    _USART_BAUD_RATE_POS(index)

  LENs: 
    _USART_BAUD_RATE_LEN(index)

*/


//******************************************************************************
/* Function :  USART_ExistsBaudRate_In16BitRegister

  Summary:
    Implements In16BitRegister variant of PLIB_USART_ExistsBaudRate

  Description:
    This template implements the In16BitRegister variant of the PLIB_USART_ExistsBaudRate function.
*/

#define PLIB_USART_ExistsBaudRate PLIB_USART_ExistsBaudRate
PLIB_TEMPLATE bool USART_ExistsBaudRate_In16BitRegister( USART_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  USART_BaudRateSet_In16BitRegister

  Summary:
    Implements In16BitRegister variant of PLIB_USART_BaudRateSet 

  Description:
    This template implements the In16BitRegister variant of the PLIB_USART_BaudRateSet function.
*/

PLIB_TEMPLATE void USART_BaudRateSet_In16BitRegister( USART_MODULE_ID index , uint32_t clockFrequency , uint32_t baudRate )
{
   uint32_t brg;
   
   brg = (((clockFrequency/baudRate) >> 4) - 1);

   _SFR_FIELD_WRITE(_USART_BAUD_RATE_VREG(index),
                    _USART_BAUD_RATE_MASK(index),
					_USART_BAUD_RATE_POS(index),
					brg);
}


//******************************************************************************
/* Function :  USART_BaudRateGet_In16BitRegister

  Summary:
    Implements In16BitRegister variant of PLIB_USART_BaudRateGet 

  Description:
    This template implements the In16BitRegister variant of the PLIB_USART_BaudRateGet function.
*/

PLIB_TEMPLATE uint32_t USART_BaudRateGet_In16BitRegister( USART_MODULE_ID index , int32_t clockFrequency )
{
    unsigned short baudReg;
	
	baudReg = _SFR_FIELD_READ(_USART_BAUD_RATE_VREG(index),
								_USART_BAUD_RATE_MASK(index),
								_USART_BAUD_RATE_POS(index));
					
	 return clockFrequency/(16 *(baudReg+1));
}


#endif /*_USART_BAUDRATE_IN16BITREGISTER_H*/

/******************************************************************************
 End of File
*/

