/*******************************************************************************
  ETH Peripheral Library Template Implementation

  File Name:
    eth_MIIM_Indicators_Default.h

  Summary:
    ETH PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : MIIM_Indicators
    and its Variant : Default
    For following APIs :
        PLIB_ETH_LinkHasFailed
        PLIB_ETH_DataNotValid
        PLIB_ETH_MIIMIsScanning
        PLIB_ETH_MIIMIsBusy
        PLIB_ETH_ExistsMIIM_Indicators

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

#ifndef _ETH_MIIM_INDICATORS_DEFAULT_H
#define _ETH_MIIM_INDICATORS_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _ETH_LINK_FAILED_VREG(index)
    _ETH_DATA_NOT_VALID_VREG(index)
    _ETH_MIIM_SCANNING_VREG(index)
    _ETH_MIIM_BUSY_VREG(index)

  MASKs:
    _ETH_LINK_FAILED_MASK(index)
    _ETH_DATA_NOT_VALID_MASK(index)
    _ETH_MIIM_SCANNING_MASK(index)
    _ETH_MIIM_BUSY_MASK(index)

  POSs:
    _ETH_LINK_FAILED_POS(index)
    _ETH_DATA_NOT_VALID_POS(index)
    _ETH_MIIM_SCANNING_POS(index)
    _ETH_MIIM_BUSY_POS(index)

  LENs:
    _ETH_LINK_FAILED_LEN(index)
    _ETH_DATA_NOT_VALID_LEN(index)
    _ETH_MIIM_SCANNING_LEN(index)
    _ETH_MIIM_BUSY_LEN(index)

*/


//******************************************************************************
/* Function :  ETH_LinkHasFailed_Default

  Summary:
    Implements Default variant of PLIB_ETH_LinkHasFailed

  Description:
    This template implements the Default variant of the PLIB_ETH_LinkHasFailed function.
*/

PLIB_TEMPLATE bool ETH_LinkHasFailed_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_LINK_FAILED_VREG(index),_ETH_LINK_FAILED_POS(index));
}


//******************************************************************************
/* Function :  ETH_DataNotValid_Default

  Summary:
    Implements Default variant of PLIB_ETH_DataNotValid

  Description:
    This template implements the Default variant of the PLIB_ETH_DataNotValid function.
*/

PLIB_TEMPLATE bool ETH_DataNotValid_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_DATA_NOT_VALID_VREG(index),_ETH_DATA_NOT_VALID_POS(index));
}


//******************************************************************************
/* Function :  ETH_MIIMIsScanning_Default

  Summary:
    Implements Default variant of PLIB_ETH_MIIMIsScanning

  Description:
    This template implements the Default variant of the PLIB_ETH_MIIMIsScanning function.
*/

PLIB_TEMPLATE bool ETH_MIIMIsScanning_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_MIIM_SCANNING_VREG(index),_ETH_MIIM_SCANNING_POS(index));
}


//******************************************************************************
/* Function :  ETH_MIIMIsBusy_Default

  Summary:
    Implements Default variant of PLIB_ETH_MIIMIsBusy

  Description:
    This template implements the Default variant of the PLIB_ETH_MIIMIsBusy function.
*/

PLIB_TEMPLATE bool ETH_MIIMIsBusy_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_MIIM_BUSY_VREG(index),_ETH_MIIM_BUSY_POS(index));
}


//******************************************************************************
/* Function :  ETH_ExistsMIIM_Indicators_Default

  Summary:
    Implements Default variant of PLIB_ETH_ExistsMIIM_Indicators

  Description:
    This template implements the Default variant of the PLIB_ETH_ExistsMIIM_Indicators function.
*/

#define PLIB_ETH_ExistsMIIM_Indicators PLIB_ETH_ExistsMIIM_Indicators
PLIB_TEMPLATE bool ETH_ExistsMIIM_Indicators_Default( ETH_MODULE_ID index )
{
    return true;
}


#endif /*_ETH_MIIM_INDICATORS_DEFAULT_H*/

/******************************************************************************
 End of File
*/

