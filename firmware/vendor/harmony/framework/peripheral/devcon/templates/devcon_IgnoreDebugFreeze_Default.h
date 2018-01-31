/*******************************************************************************
  DEVCON Peripheral Library Template Implementation

  File Name:
    devcon_IgnoreDebugFreeze_Default.h

  Summary:
    DEVCON PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : IgnoreDebugFreeze
    and its Variant : Default
    For following APIs :
        PLIB_DEVCON_IgnoreDebugFreezeEnable
        PLIB_DEVCON_IgnoreDebugFreezeDisable
        PLIB_DEVCON_ExistsIgnoreDebugFreeze

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DEVCON_IGNOREDEBUGFREEZE_DEFAULT_H
#define _DEVCON_IGNOREDEBUGFREEZE_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _DEVCON_DEBUG_DATA_PORT_USB_ENABLE_VREG(index)
    _DEVCON_DEBUG_DATA_PORT_UART1_ENABLE_VREG(index)
    _DEVCON_DEBUG_DATA_PORT_UART2_ENABLE_VREG(index)
    _DEVCON_DEBUG_DATA_PORT_SPI1_ENABLE_VREG(index)

  MASKs:
    _DEVCON_DEBUG_DATA_PORT_USB_ENABLE_MASK(index)
    _DEVCON_DEBUG_DATA_PORT_UART1_ENABLE_MASK(index)
    _DEVCON_DEBUG_DATA_PORT_UART2_ENABLE_MASK(index)
    _DEVCON_DEBUG_DATA_PORT_SPI1_ENABLE_MASK(index)

  POSs:
    _DEVCON_DEBUG_DATA_PORT_USB_ENABLE_POS(index)
    _DEVCON_DEBUG_DATA_PORT_UART1_ENABLE_POS(index)
    _DEVCON_DEBUG_DATA_PORT_UART2_ENABLE_POS(index)
    _DEVCON_DEBUG_DATA_PORT_SPI1_ENABLE_POS(index)

  LENs:
    _DEVCON_DEBUG_DATA_PORT_USB_ENABLE_LEN(index)
    _DEVCON_DEBUG_DATA_PORT_UART1_ENABLE_LEN(index)
    _DEVCON_DEBUG_DATA_PORT_UART2_ENABLE_LEN(index)
    _DEVCON_DEBUG_DATA_PORT_SPI1_ENABLE_LEN(index)

*/


//******************************************************************************
/* Function :  DEVCON_IgnoreDebugFreezeEnable_Default

  Summary:
    Implements Default variant of PLIB_DEVCON_IgnoreDebugFreezeEnable

  Description:
    This template implements the Default variant of the PLIB_DEVCON_IgnoreDebugFreezeEnable function.
*/

PLIB_TEMPLATE void DEVCON_IgnoreDebugFreezeEnable_Default( DEVCON_MODULE_ID index , DEVCON_DEBUG_PERIPHERAL myPeripheral )
{
    if ( 0 != (myPeripheral & DEVCON_DEBUG_USB) )
    {
        _SFR_BIT_SET(_DEVCON_DEBUG_DATA_PORT_USB_ENABLE_VREG(index),_DEVCON_DEBUG_DATA_PORT_USB_ENABLE_POS(index));
    }

    if ( 0 != (myPeripheral & DEVCON_DEBUG_UART1) )
    {
        _SFR_BIT_SET(_DEVCON_DEBUG_DATA_PORT_UART1_ENABLE_VREG(index),_DEVCON_DEBUG_DATA_PORT_UART1_ENABLE_POS(index));
    }

    if ( 0 != (myPeripheral & DEVCON_DEBUG_UART2) )
    {
        _SFR_BIT_SET(_DEVCON_DEBUG_DATA_PORT_UART2_ENABLE_VREG(index),_DEVCON_DEBUG_DATA_PORT_UART2_ENABLE_POS(index));
    }

    if ( 0 != (myPeripheral & DEVCON_DEBUG_SPI1) )
    {
        _SFR_BIT_SET(_DEVCON_DEBUG_DATA_PORT_SPI1_ENABLE_VREG(index),_DEVCON_DEBUG_DATA_PORT_SPI1_ENABLE_POS(index));
    }

}


//******************************************************************************
/* Function :  DEVCON_IgnoreDebugFreezeDisable_Default

  Summary:
    Implements Default variant of PLIB_DEVCON_IgnoreDebugFreezeDisable

  Description:
    This template implements the Default variant of the PLIB_DEVCON_IgnoreDebugFreezeDisable function.
*/

PLIB_TEMPLATE void DEVCON_IgnoreDebugFreezeDisable_Default( DEVCON_MODULE_ID index , DEVCON_DEBUG_PERIPHERAL myPeripheral )
{
    if ( 0 != (myPeripheral & DEVCON_DEBUG_USB) )
    {
        _SFR_BIT_CLEAR(_DEVCON_DEBUG_DATA_PORT_USB_ENABLE_VREG(index),_DEVCON_DEBUG_DATA_PORT_USB_ENABLE_POS(index));
    }

    if ( 0 != (myPeripheral & DEVCON_DEBUG_UART1) )
    {
        _SFR_BIT_CLEAR(_DEVCON_DEBUG_DATA_PORT_UART1_ENABLE_VREG(index),_DEVCON_DEBUG_DATA_PORT_UART1_ENABLE_POS(index));
    }

    if ( 0 != (myPeripheral & DEVCON_DEBUG_UART2) )
    {
        _SFR_BIT_CLEAR(_DEVCON_DEBUG_DATA_PORT_UART2_ENABLE_VREG(index),_DEVCON_DEBUG_DATA_PORT_UART2_ENABLE_POS(index));
    }

    if ( 0 != (myPeripheral & DEVCON_DEBUG_SPI1) )
    {
        _SFR_BIT_CLEAR(_DEVCON_DEBUG_DATA_PORT_SPI1_ENABLE_VREG(index),_DEVCON_DEBUG_DATA_PORT_SPI1_ENABLE_POS(index));
    }
}


//******************************************************************************
/* Function :  DEVCON_ExistsIgnoreDebugFreeze_Default

  Summary:
    Implements Default variant of PLIB_DEVCON_ExistsIgnoreDebugFreeze

  Description:
    This template implements the Default variant of the PLIB_DEVCON_ExistsIgnoreDebugFreeze function.
*/

#define PLIB_DEVCON_ExistsIgnoreDebugFreeze PLIB_DEVCON_ExistsIgnoreDebugFreeze
PLIB_TEMPLATE bool DEVCON_ExistsIgnoreDebugFreeze_Default( DEVCON_MODULE_ID index )
{
    return true;
}


#endif /*_DEVCON_IGNOREDEBUGFREEZE_DEFAULT_H*/

/******************************************************************************
 End of File
*/

