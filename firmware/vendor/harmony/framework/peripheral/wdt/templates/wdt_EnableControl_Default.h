/*******************************************************************************
  WDT Peripheral Library Template Implementation

  File Name:
    wdt_EnableControl_Default.h

  Summary:
    WDT PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : EnableControl
    and its Variant : Default
    For following APIs :
        PLIB_WDT_ExistsEnableControl
        PLIB_WDT_Enable
        PLIB_WDT_Disable
        PLIB_WDT_IsEnabled

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

#ifndef _WDT_ENABLECONTROL_DEFAULT_H
#define _WDT_ENABLECONTROL_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _WDT_ENABLE_CONTROL_VREG(index)

  MASKs:
    _WDT_ENABLE_CONTROL_MASK(index)

  POSs:
    _WDT_ENABLE_CONTROL_POS(index)

  LENs:
    _WDT_ENABLE_CONTROL_LEN(index)

*/


//******************************************************************************
/* Function :  WDT_ExistsEnableControl_Default

  Summary:
    Implements Default variant of PLIB_WDT_ExistsEnableControl

  Description:
    This template implements the Default variant of the PLIB_WDT_ExistsEnableControl function.
*/

#define PLIB_WDT_ExistsEnableControl PLIB_WDT_ExistsEnableControl
PLIB_TEMPLATE bool WDT_ExistsEnableControl_Default( WDT_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  WDT_Enable_Default

  Summary:
    Implements Default variant of PLIB_WDT_Enable

  Description:
    This template implements the Default variant of the PLIB_WDT_Enable function.
*/

PLIB_TEMPLATE void WDT_Enable_Default( WDT_MODULE_ID index )
{
    _SFR_BIT_SET(_WDT_ENABLE_CONTROL_VREG(index),
                   _WDT_ENABLE_CONTROL_POS(index));
}


//******************************************************************************
/* Function :  WDT_Disable_Default

  Summary:
    Implements Default variant of PLIB_WDT_Disable

  Description:
    This template implements the Default variant of the PLIB_WDT_Disable function.
*/

PLIB_TEMPLATE void WDT_Disable_Default( WDT_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_WDT_ENABLE_CONTROL_VREG(index),
                   _WDT_ENABLE_CONTROL_POS(index));
}


//******************************************************************************
/* Function :  WDT_IsEnabled_Default

  Summary:
    Implements Default variant of PLIB_WDT_IsEnabled

  Description:
    This template implements the Default variant of the PLIB_WDT_IsEnabled function.
*/

PLIB_TEMPLATE bool WDT_IsEnabled_Default( WDT_MODULE_ID index )
{
    return _SFR_BIT_READ(_WDT_ENABLE_CONTROL_VREG(index),
                   _WDT_ENABLE_CONTROL_POS(index));
}


#endif /*_WDT_ENABLECONTROL_DEFAULT_H*/

/******************************************************************************
 End of File
*/

