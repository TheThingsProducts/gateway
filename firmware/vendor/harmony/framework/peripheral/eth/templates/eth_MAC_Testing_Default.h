/*******************************************************************************
  ETH Peripheral Library Template Implementation

  File Name:
    eth_MAC_Testing_Default.h

  Summary:
    ETH PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : MAC_Testing
    and its Variant : Default
    For following APIs :
        PLIB_ETH_TestBackPressEnable
        PLIB_ETH_TestBackPressDisable
        PLIB_ETH_TestBackPressIsEnabled
        PLIB_ETH_TestPauseEnable
        PLIB_ETH_TestPauseDisable
        PLIB_ETH_TestPauseIsEnabled
        PLIB_ETH_ShortcutQuantaEnable
        PLIB_ETH_ShortcutQuantaDisable
        PLIB_ETH_ShortcutQuantaIsEnabled
        PLIB_ETH_ExistsMAC_Testing

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

#ifndef _ETH_MAC_TESTING_DEFAULT_H
#define _ETH_MAC_TESTING_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _ETH_TEST_BACKPRESSURE_VREG(index)
    _ETH_TEST_PAUSE_VREG(index)
    _ETH_SHORTCUT_PAUSE_QUANTA_VREG(index)

  MASKs:
    _ETH_TEST_BACKPRESSURE_MASK(index)
    _ETH_TEST_PAUSE_MASK(index)
    _ETH_SHORTCUT_PAUSE_QUANTA_MASK(index)

  POSs:
    _ETH_TEST_BACKPRESSURE_POS(index)
    _ETH_TEST_PAUSE_POS(index)
    _ETH_SHORTCUT_PAUSE_QUANTA_POS(index)

  LENs:
    _ETH_TEST_BACKPRESSURE_LEN(index)
    _ETH_TEST_PAUSE_LEN(index)
    _ETH_SHORTCUT_PAUSE_QUANTA_LEN(index)

*/


//******************************************************************************
/* Function :  ETH_TestBackPressEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_TestBackPressEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_TestBackPressEnable function.
*/

PLIB_TEMPLATE void ETH_TestBackPressEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_TEST_BACKPRESSURE_VREG(index),_ETH_TEST_BACKPRESSURE_POS(index));
}


//******************************************************************************
/* Function :  ETH_TestBackPressDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_TestBackPressDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_TestBackPressDisable function.
*/

PLIB_TEMPLATE void ETH_TestBackPressDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_TEST_BACKPRESSURE_VREG(index),_ETH_TEST_BACKPRESSURE_POS(index));
}


//******************************************************************************
/* Function :  ETH_TestBackPressIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_TestBackPressIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_TestBackPressIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_TestBackPressIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_TEST_BACKPRESSURE_VREG(index),_ETH_TEST_BACKPRESSURE_POS(index));
}


//******************************************************************************
/* Function :  ETH_TestPauseEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_TestPauseEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_TestPauseEnable function.
*/

PLIB_TEMPLATE void ETH_TestPauseEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_TEST_PAUSE_VREG(index),_ETH_TEST_PAUSE_POS(index));
}


//******************************************************************************
/* Function :  ETH_TestPauseDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_TestPauseDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_TestPauseDisable function.
*/

PLIB_TEMPLATE void ETH_TestPauseDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_TEST_PAUSE_VREG(index),_ETH_TEST_PAUSE_POS(index));
}


//******************************************************************************
/* Function :  ETH_TestPauseIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_TestPauseIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_TestPauseIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_TestPauseIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_TEST_PAUSE_VREG(index),_ETH_TEST_PAUSE_POS(index));
}


//******************************************************************************
/* Function :  ETH_ShortcutQuantaEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_ShortcutQuantaEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_ShortcutQuantaEnable function.
*/

PLIB_TEMPLATE void ETH_ShortcutQuantaEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_SHORTCUT_PAUSE_QUANTA_VREG(index),_ETH_SHORTCUT_PAUSE_QUANTA_POS(index));
}


//******************************************************************************
/* Function :  ETH_ShortcutQuantaDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_ShortcutQuantaDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_ShortcutQuantaDisable function.
*/

PLIB_TEMPLATE void ETH_ShortcutQuantaDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_SHORTCUT_PAUSE_QUANTA_VREG(index),_ETH_SHORTCUT_PAUSE_QUANTA_POS(index));
}


//******************************************************************************
/* Function :  ETH_ShortcutQuantaIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_ShortcutQuantaIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_ShortcutQuantaIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_ShortcutQuantaIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_SHORTCUT_PAUSE_QUANTA_VREG(index),_ETH_SHORTCUT_PAUSE_QUANTA_POS(index));

}


//******************************************************************************
/* Function :  ETH_ExistsMAC_Testing_Default

  Summary:
    Implements Default variant of PLIB_ETH_ExistsMAC_Testing

  Description:
    This template implements the Default variant of the PLIB_ETH_ExistsMAC_Testing function.
*/

#define PLIB_ETH_ExistsMAC_Testing PLIB_ETH_ExistsMAC_Testing
PLIB_TEMPLATE bool ETH_ExistsMAC_Testing_Default( ETH_MODULE_ID index )
{
    return true;
}


#endif /*_ETH_MAC_TESTING_DEFAULT_H*/

/******************************************************************************
 End of File
*/

