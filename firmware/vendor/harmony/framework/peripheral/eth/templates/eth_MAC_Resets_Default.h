/*******************************************************************************
  ETH Peripheral Library Template Implementation

  File Name:
    eth_MAC_Resets_Default.h

  Summary:
    ETH PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : MAC_Resets
    and its Variant : Default
    For following APIs :
        PLIB_ETH_MIIResetEnable
        PLIB_ETH_MIIResetDisable
        PLIB_ETH_MIIResetIsEnabled
        PLIB_ETH_SimResetEnable
        PLIB_ETH_SimResetDisable
        PLIB_ETH_SimResetIsEnabled
        PLIB_ETH_MCSRxResetEnable
        PLIB_ETH_MCSRxResetDisable
        PLIB_ETH_MCSRxResetIsEnabled
        PLIB_ETH_RxFuncResetEnable
        PLIB_ETH_RxFuncResetDisable
        PLIB_ETH_RxFuncResetIsEnabled
        PLIB_ETH_MCSTxResetEnable
        PLIB_ETH_MCSTxResetDisable
        PLIB_ETH_MCSTxResetIsEnabled
        PLIB_ETH_TxFuncResetEnable
        PLIB_ETH_TxFuncResetDisable
        PLIB_ETH_TxFuncResetIsEnabled
        PLIB_ETH_ExistsMAC_Resets

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

#ifndef _ETH_MAC_RESETS_DEFAULT_H
#define _ETH_MAC_RESETS_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _ETH_MII_RESET_VREG(index)
    _ETH_SIMULATION_RESET_VREG(index)
    _ETH_RESET_MCS_RX_VREG(index)
    _ETH_RESET_RX_FUNCTION_VREG(index)
    _ETH_RESET_MCS_TX_VREG(index)
    _ETH_RESET_TX_FUNCTION_VREG(index)

  MASKs:
    _ETH_MII_RESET_MASK(index)
    _ETH_SIMULATION_RESET_MASK(index)
    _ETH_RESET_MCS_RX_MASK(index)
    _ETH_RESET_RX_FUNCTION_MASK(index)
    _ETH_RESET_MCS_TX_MASK(index)
    _ETH_RESET_TX_FUNCTION_MASK(index)

  POSs:
    _ETH_MII_RESET_POS(index)
    _ETH_SIMULATION_RESET_POS(index)
    _ETH_RESET_MCS_RX_POS(index)
    _ETH_RESET_RX_FUNCTION_POS(index)
    _ETH_RESET_MCS_TX_POS(index)
    _ETH_RESET_TX_FUNCTION_POS(index)

  LENs:
    _ETH_MII_RESET_LEN(index)
    _ETH_SIMULATION_RESET_LEN(index)
    _ETH_RESET_MCS_RX_LEN(index)
    _ETH_RESET_RX_FUNCTION_LEN(index)
    _ETH_RESET_MCS_TX_LEN(index)
    _ETH_RESET_TX_FUNCTION_LEN(index)

*/


//******************************************************************************
/* Function :  ETH_MIIResetEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_MIIResetEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_MIIResetEnable function.
*/

PLIB_TEMPLATE void ETH_MIIResetEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_MII_RESET_VREG(index),_ETH_MII_RESET_POS(index));
}


//******************************************************************************
/* Function :  ETH_MIIResetDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_MIIResetDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_MIIResetDisable function.
*/

PLIB_TEMPLATE void ETH_MIIResetDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_MII_RESET_VREG(index),_ETH_MII_RESET_POS(index));
}


//******************************************************************************
/* Function :  ETH_MIIResetIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_MIIResetIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_MIIResetIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_MIIResetIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_MII_RESET_VREG(index),_ETH_MII_RESET_POS(index));
}


//******************************************************************************
/* Function :  ETH_SimResetEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_SimResetEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_SimResetEnable function.
*/

PLIB_TEMPLATE void ETH_SimResetEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_SIMULATION_RESET_VREG(index),_ETH_SIMULATION_RESET_POS(index));
}


//******************************************************************************
/* Function :  ETH_SimResetDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_SimResetDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_SimResetDisable function.
*/

PLIB_TEMPLATE void ETH_SimResetDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_SIMULATION_RESET_VREG(index),_ETH_SIMULATION_RESET_POS(index));
}


//******************************************************************************
/* Function :  ETH_SimResetIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_SimResetIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_SimResetIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_SimResetIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_SIMULATION_RESET_VREG(index),_ETH_SIMULATION_RESET_POS(index));
}


//******************************************************************************
/* Function :  ETH_MCSRxResetEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_MCSRxResetEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_MCSRxResetEnable function.
*/

PLIB_TEMPLATE void ETH_MCSRxResetEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_RESET_MCS_RX_VREG(index),_ETH_RESET_MCS_RX_POS(index));
}


//******************************************************************************
/* Function :  ETH_MCSRxResetDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_MCSRxResetDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_MCSRxResetDisable function.
*/

PLIB_TEMPLATE void ETH_MCSRxResetDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_RESET_MCS_RX_VREG(index),_ETH_RESET_MCS_RX_POS(index));
}


//******************************************************************************
/* Function :  ETH_MCSRxResetIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_MCSRxResetIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_MCSRxResetIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_MCSRxResetIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_RESET_MCS_RX_VREG(index),_ETH_RESET_MCS_RX_POS(index));
}


//******************************************************************************
/* Function :  ETH_RxFuncResetEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_RxFuncResetEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_RxFuncResetEnable function.
*/

PLIB_TEMPLATE void ETH_RxFuncResetEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_RESET_RX_FUNCTION_VREG(index),_ETH_RESET_RX_FUNCTION_POS(index));
}


//******************************************************************************
/* Function :  ETH_RxFuncResetDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_RxFuncResetDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_RxFuncResetDisable function.
*/

PLIB_TEMPLATE void ETH_RxFuncResetDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_RESET_RX_FUNCTION_VREG(index),_ETH_RESET_RX_FUNCTION_POS(index));
}


//******************************************************************************
/* Function :  ETH_RxFuncResetIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_RxFuncResetIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_RxFuncResetIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_RxFuncResetIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_RESET_RX_FUNCTION_VREG(index),_ETH_RESET_RX_FUNCTION_POS(index));
}


//******************************************************************************
/* Function :  ETH_MCSTxResetEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_MCSTxResetEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_MCSTxResetEnable function.
*/

PLIB_TEMPLATE void ETH_MCSTxResetEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_RESET_MCS_TX_VREG(index),_ETH_RESET_MCS_TX_POS(index));
}


//******************************************************************************
/* Function :  ETH_MCSTxResetDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_MCSTxResetDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_MCSTxResetDisable function.
*/

PLIB_TEMPLATE void ETH_MCSTxResetDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_RESET_MCS_TX_VREG(index),_ETH_RESET_MCS_TX_POS(index));
}


//******************************************************************************
/* Function :  ETH_MCSTxResetIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_MCSTxResetIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_MCSTxResetIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_MCSTxResetIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_RESET_MCS_TX_VREG(index),_ETH_RESET_MCS_TX_POS(index));
}


//******************************************************************************
/* Function :  ETH_TxFuncResetEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_TxFuncResetEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_TxFuncResetEnable function.
*/

PLIB_TEMPLATE void ETH_TxFuncResetEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_RESET_TX_FUNCTION_VREG(index),_ETH_RESET_TX_FUNCTION_POS(index));
}


//******************************************************************************
/* Function :  ETH_TxFuncResetDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_TxFuncResetDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_TxFuncResetDisable function.
*/

PLIB_TEMPLATE void ETH_TxFuncResetDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_RESET_TX_FUNCTION_VREG(index),_ETH_RESET_TX_FUNCTION_POS(index));
}


//******************************************************************************
/* Function :  ETH_TxFuncResetIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_TxFuncResetIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_TxFuncResetIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_TxFuncResetIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_RESET_TX_FUNCTION_VREG(index),_ETH_RESET_TX_FUNCTION_POS(index));
}


//******************************************************************************
/* Function :  ETH_ExistsMAC_Resets_Default

  Summary:
    Implements Default variant of PLIB_ETH_ExistsMAC_Resets

  Description:
    This template implements the Default variant of the PLIB_ETH_ExistsMAC_Resets function.
*/

#define PLIB_ETH_ExistsMAC_Resets PLIB_ETH_ExistsMAC_Resets
PLIB_TEMPLATE bool ETH_ExistsMAC_Resets_Default( ETH_MODULE_ID index )
{
    return true;
}


#endif /*_ETH_MAC_RESETS_DEFAULT_H*/

/******************************************************************************
 End of File
*/

