/*******************************************************************************
  WDT Peripheral Library Template Implementation

  File Name:
    wdt_TimerClear_Default.h

  Summary:
    WDT PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : TimerClear
    and its Variant : Default
    For following APIs :
        PLIB_WDT_ExistsTimerClear
        PLIB_WDT_TimerClear

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

#ifndef _WDT_TIMERCLEAR_DEFAULT_H
#define _WDT_TIMERCLEAR_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _WDT_TIMER_CLEAR_VREG(index)

  MASKs:
    _WDT_TIMER_CLEAR_MASK(index)

  POSs:
    _WDT_TIMER_CLEAR_POS(index)

  LENs:
    _WDT_TIMER_CLEAR_LEN(index)

*/


//******************************************************************************
/* Function :  WDT_ExistsTimerClear_Default

  Summary:
    Implements Default variant of PLIB_WDT_ExistsTimerClear

  Description:
    This template implements the Default variant of the PLIB_WDT_ExistsTimerClear function.
*/

#define PLIB_WDT_ExistsTimerClear PLIB_WDT_ExistsTimerClear
PLIB_TEMPLATE bool WDT_ExistsTimerClear_Default( WDT_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  WDT_TimerClear_Default

  Summary:
    Implements Default variant of PLIB_WDT_TimerClear

  Description:
    This template implements the Default variant of the PLIB_WDT_TimerClear function.
*/

PLIB_TEMPLATE void WDT_TimerClear_Default( WDT_MODULE_ID index )
{
    _SFR_BIT_SET(_WDT_TIMER_CLEAR_VREG(index),
                 _WDT_TIMER_CLEAR_POS(index));
}


#endif /*_WDT_TIMERCLEAR_DEFAULT_H*/

/******************************************************************************
 End of File
*/

