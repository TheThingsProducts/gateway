/*******************************************************************************
  DEVCON Peripheral Library Template Implementation

  File Name:
    devcon_MPLL_Default.h

  Summary:
    DEVCON PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : MPLL
    and its Variant : Default
    For following APIs :
        PLIB_DEVCON_MPLLIsReady
        PLIB_DEVCON_MPLLEnable
        PLIB_DEVCON_MPLLDisable
        PLIB_DEVCON_MPLLODiv1Set
        PLIB_DEVCON_MPLLODiv2Set
        PLIB_DEVCON_MPLLVregIsReady
        PLIB_DEVCON_MPLLVregEnable
        PLIB_DEVCON_MPLLVregDisable
        PLIB_DEVCON_MPLLMultiplierSet
        PLIB_DEVCON_MPLLVrefSet
        PLIB_DEVCON_MPLLInputDivSet
        PLIB_DEVCON_ExistsMPLL

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

#ifndef _DEVCON_MPLL_DEFAULT_H
#define _DEVCON_MPLL_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    _DEVCON_MPLL_READY_VREG(index)
    _DEVCON_MPLL_DISABLE_VREG(index)
    _DEVCON_MPLL_OUTPUT_DIVIDER_2_VREG(index)
    _DEVCON_MPLL_OUTPUT_DIVIDER_1_VREG(index)
    _DEVCON_MPLL_VREG_READY_VREG(index)
    _DEVCON_MPLL_VREG_DISABLE_VREG(index)
    _DEVCON_MPLL_MULTIPLIER_VREG(index)
    _DEVCON_MPLL_INT_VREF_CONTROL_VREG(index)
    _DEVCON_MPLL_INPUT_DIVIDER_VREG(index)

  MASKs: 
    _DEVCON_MPLL_READY_MASK(index)
    _DEVCON_MPLL_DISABLE_MASK(index)
    _DEVCON_MPLL_OUTPUT_DIVIDER_2_MASK(index)
    _DEVCON_MPLL_OUTPUT_DIVIDER_1_MASK(index)
    _DEVCON_MPLL_VREG_READY_MASK(index)
    _DEVCON_MPLL_VREG_DISABLE_MASK(index)
    _DEVCON_MPLL_MULTIPLIER_MASK(index)
    _DEVCON_MPLL_INT_VREF_CONTROL_MASK(index)
    _DEVCON_MPLL_INPUT_DIVIDER_MASK(index)

  POSs: 
    _DEVCON_MPLL_READY_POS(index)
    _DEVCON_MPLL_DISABLE_POS(index)
    _DEVCON_MPLL_OUTPUT_DIVIDER_2_POS(index)
    _DEVCON_MPLL_OUTPUT_DIVIDER_1_POS(index)
    _DEVCON_MPLL_VREG_READY_POS(index)
    _DEVCON_MPLL_VREG_DISABLE_POS(index)
    _DEVCON_MPLL_MULTIPLIER_POS(index)
    _DEVCON_MPLL_INT_VREF_CONTROL_POS(index)
    _DEVCON_MPLL_INPUT_DIVIDER_POS(index)

  LENs: 
    _DEVCON_MPLL_READY_LEN(index)
    _DEVCON_MPLL_DISABLE_LEN(index)
    _DEVCON_MPLL_OUTPUT_DIVIDER_2_LEN(index)
    _DEVCON_MPLL_OUTPUT_DIVIDER_1_LEN(index)
    _DEVCON_MPLL_VREG_READY_LEN(index)
    _DEVCON_MPLL_VREG_DISABLE_LEN(index)
    _DEVCON_MPLL_MULTIPLIER_LEN(index)
    _DEVCON_MPLL_INT_VREF_CONTROL_LEN(index)
    _DEVCON_MPLL_INPUT_DIVIDER_LEN(index)

*/


//******************************************************************************
/* Function :  DEVCON_MPLLIsReady_Default

  Summary:
    Implements Default variant of PLIB_DEVCON_MPLLIsReady 

  Description:
    This template implements the Default variant of the PLIB_DEVCON_MPLLIsReady function.
*/

PLIB_TEMPLATE bool DEVCON_MPLLIsReady_Default( DEVCON_MODULE_ID index )
{
    return _SFR_BIT_READ(_DEVCON_MPLL_READY_VREG(index), _DEVCON_MPLL_READY_POS(index)) ? true : false;
}


//******************************************************************************
/* Function :  DEVCON_MPLLEnable_Default

  Summary:
    Implements Default variant of PLIB_DEVCON_MPLLEnable 

  Description:
    This template implements the Default variant of the PLIB_DEVCON_MPLLEnable function.
*/

PLIB_TEMPLATE void DEVCON_MPLLEnable_Default( DEVCON_MODULE_ID index )
{
    _SFR_BIT_WRITE(_DEVCON_MPLL_DISABLE_VREG(index), _DEVCON_MPLL_READY_POS(index), 0);
}


//******************************************************************************
/* Function :  DEVCON_MPLLDisable_Default

  Summary:
    Implements Default variant of PLIB_DEVCON_MPLLDisable 

  Description:
    This template implements the Default variant of the PLIB_DEVCON_MPLLDisable function.
*/

PLIB_TEMPLATE void DEVCON_MPLLDisable_Default( DEVCON_MODULE_ID index )
{
    _SFR_BIT_WRITE(_DEVCON_MPLL_DISABLE_VREG(index), _DEVCON_MPLL_READY_POS(index), 1);
}


//******************************************************************************
/* Function :  DEVCON_MPLLODiv1Set_Default

  Summary:
    Implements Default variant of PLIB_DEVCON_MPLLODiv1Set 

  Description:
    This template implements the Default variant of the PLIB_DEVCON_MPLLODiv1Set function.
*/

PLIB_TEMPLATE void DEVCON_MPLLODiv1Set_Default( DEVCON_MODULE_ID index , DEVCON_MPLL_OUTPUT_DIVIDER bits )
{
    _SFR_FIELD_WRITE(_DEVCON_MPLL_OUTPUT_DIVIDER_1_VREG(index), _DEVCON_MPLL_OUTPUT_DIVIDER_1_MASK(index), _DEVCON_MPLL_OUTPUT_DIVIDER_1_POS(index), bits);
}


//******************************************************************************
/* Function :  DEVCON_MPLLODiv2Set_Default

  Summary:
    Implements Default variant of PLIB_DEVCON_MPLLODiv2Set 

  Description:
    This template implements the Default variant of the PLIB_DEVCON_MPLLODiv2Set function.
*/

PLIB_TEMPLATE void DEVCON_MPLLODiv2Set_Default( DEVCON_MODULE_ID index , DEVCON_MPLL_OUTPUT_DIVIDER bits )
{
    _SFR_FIELD_WRITE(_DEVCON_MPLL_OUTPUT_DIVIDER_2_VREG(index), _DEVCON_MPLL_OUTPUT_DIVIDER_2_MASK(index), _DEVCON_MPLL_OUTPUT_DIVIDER_2_POS(index), bits);
}


//******************************************************************************
/* Function :  DEVCON_MPLLVregIsReady_Default

  Summary:
    Implements Default variant of PLIB_DEVCON_MPLLVregIsReady 

  Description:
    This template implements the Default variant of the PLIB_DEVCON_MPLLVregIsReady function.
*/

PLIB_TEMPLATE bool DEVCON_MPLLVregIsReady_Default( DEVCON_MODULE_ID index )
{
    return _SFR_BIT_READ(_DEVCON_MPLL_VREG_READY_VREG(index), _DEVCON_MPLL_VREG_READY_POS(index)) ? true : false;
}


//******************************************************************************
/* Function :  DEVCON_MPLLVregEnable_Default

  Summary:
    Implements Default variant of PLIB_DEVCON_MPLLVregEnable 

  Description:
    This template implements the Default variant of the PLIB_DEVCON_MPLLVregEnable function.
*/

PLIB_TEMPLATE void DEVCON_MPLLVregEnable_Default( DEVCON_MODULE_ID index )
{
    _SFR_BIT_WRITE(_DEVCON_MPLL_VREG_DISABLE_VREG(index), _DEVCON_MPLL_VREG_DISABLE_POS(index), 0);
}


//******************************************************************************
/* Function :  DEVCON_MPLLVregDisable_Default

  Summary:
    Implements Default variant of PLIB_DEVCON_MPLLVregDisable 

  Description:
    This template implements the Default variant of the PLIB_DEVCON_MPLLVregDisable function.
*/

PLIB_TEMPLATE void DEVCON_MPLLVregDisable_Default( DEVCON_MODULE_ID index )
{
    _SFR_BIT_WRITE(_DEVCON_MPLL_VREG_DISABLE_VREG(index), _DEVCON_MPLL_VREG_DISABLE_POS(index), 1);
}


//******************************************************************************
/* Function :  DEVCON_MPLLMultiplierSet_Default

  Summary:
    Implements Default variant of PLIB_DEVCON_MPLLMultiplierSet 

  Description:
    This template implements the Default variant of the PLIB_DEVCON_MPLLMultiplierSet function.
*/

PLIB_TEMPLATE void DEVCON_MPLLMultiplierSet_Default( DEVCON_MODULE_ID index , uint8_t value )
{
    _SFR_FIELD_WRITE(_DEVCON_MPLL_MULTIPLIER_VREG(index), _DEVCON_MPLL_MULTIPLIER_MASK(index), _DEVCON_MPLL_MULTIPLIER_POS(index), value);
}


//******************************************************************************
/* Function :  DEVCON_MPLLVrefSet_Default

  Summary:
    Implements Default variant of PLIB_DEVCON_MPLLVrefSet 

  Description:
    This template implements the Default variant of the PLIB_DEVCON_MPLLVrefSet function.
*/

PLIB_TEMPLATE void DEVCON_MPLLVrefSet_Default( DEVCON_MODULE_ID index , DEVCON_MPLL_VREF_CONTROL vref )
{
    _SFR_FIELD_WRITE(_DEVCON_MPLL_INT_VREF_CONTROL_VREG(index), _DEVCON_MPLL_INT_VREF_CONTROL_MASK(index), _DEVCON_MPLL_INT_VREF_CONTROL_POS(index), vref);
}


//******************************************************************************
/* Function :  DEVCON_MPLLInputDivSet_Default

  Summary:
    Implements Default variant of PLIB_DEVCON_MPLLInputDivSet 

  Description:
    This template implements the Default variant of the PLIB_DEVCON_MPLLInputDivSet function.
*/

PLIB_TEMPLATE void DEVCON_MPLLInputDivSet_Default( DEVCON_MODULE_ID index , uint8_t value )
{
    _SFR_FIELD_WRITE(_DEVCON_MPLL_INPUT_DIVIDER_VREG(index), _DEVCON_MPLL_INPUT_DIVIDER_MASK(index), _DEVCON_MPLL_INPUT_DIVIDER_POS(index), value);
}


//******************************************************************************
/* Function :  DEVCON_ExistsMPLL_Default

  Summary:
    Implements Default variant of PLIB_DEVCON_ExistsMPLL

  Description:
    This template implements the Default variant of the PLIB_DEVCON_ExistsMPLL function.
*/

#define PLIB_DEVCON_ExistsMPLL PLIB_DEVCON_ExistsMPLL
PLIB_TEMPLATE bool DEVCON_ExistsMPLL_Default( DEVCON_MODULE_ID index )
{
    return true;
}


#endif /*_DEVCON_MPLL_DEFAULT_H*/

/******************************************************************************
 End of File
*/

