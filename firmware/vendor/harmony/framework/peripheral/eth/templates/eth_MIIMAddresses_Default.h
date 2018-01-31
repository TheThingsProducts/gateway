/*******************************************************************************
  ETH Peripheral Library Template Implementation

  File Name:
    eth_MIIMAddresses_Default.h

  Summary:
    ETH PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : MIIMAddresses
    and its Variant : Default
    For following APIs :
        PLIB_ETH_PHYAddressGet
        PLIB_ETH_PHYAddressSet
        PLIB_ETH_RegisterAddressGet
        PLIB_ETH_RegisterAddressSet
        PLIB_ETH_ExistsMIIMAddresses

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

#ifndef _ETH_MIIMADDRESSES_DEFAULT_H
#define _ETH_MIIMADDRESSES_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _ETH_MIIM_PHY_ADDRESS_VREG(index)
    _ETH_MIIM_REGISTER_ADDRESS_VREG(index)

  MASKs:
    _ETH_MIIM_PHY_ADDRESS_MASK(index)
    _ETH_MIIM_REGISTER_ADDRESS_MASK(index)

  POSs:
    _ETH_MIIM_PHY_ADDRESS_POS(index)
    _ETH_MIIM_REGISTER_ADDRESS_POS(index)

  LENs:
    _ETH_MIIM_PHY_ADDRESS_LEN(index)
    _ETH_MIIM_REGISTER_ADDRESS_LEN(index)

*/


//******************************************************************************
/* Function :  ETH_PHYAddressGet_Default

  Summary:
    Implements Default variant of PLIB_ETH_PHYAddressGet

  Description:
    This template implements the Default variant of the PLIB_ETH_PHYAddressGet function.
*/

PLIB_TEMPLATE uint8_t ETH_PHYAddressGet_Default( ETH_MODULE_ID index )
{
    return (uint8_t)_SFR_FIELD_READ(_ETH_MIIM_PHY_ADDRESS_VREG(index),
                                    _ETH_MIIM_PHY_ADDRESS_MASK(index),
                                    _ETH_MIIM_PHY_ADDRESS_POS(index) );
}


//******************************************************************************
/* Function :  ETH_PHYAddressSet_Default

  Summary:
    Implements Default variant of PLIB_ETH_PHYAddressSet

  Description:
    This template implements the Default variant of the PLIB_ETH_PHYAddressSet function.
*/

PLIB_TEMPLATE void ETH_PHYAddressSet_Default( ETH_MODULE_ID index , uint8_t phyAddr )
{
    _SFR_FIELD_WRITE(_ETH_MIIM_PHY_ADDRESS_VREG(index),
                     _ETH_MIIM_PHY_ADDRESS_MASK(index),
                     _ETH_MIIM_PHY_ADDRESS_POS(index) ,
                      phyAddr                         );
}


//******************************************************************************
/* Function :  ETH_RegisterAddressGet_Default

  Summary:
    Implements Default variant of PLIB_ETH_RegisterAddressGet

  Description:
    This template implements the Default variant of the PLIB_ETH_RegisterAddressGet function.
*/

PLIB_TEMPLATE uint8_t ETH_RegisterAddressGet_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_FIELD_READ(_ETH_MIIM_REGISTER_ADDRESS_VREG(index),
                                 _ETH_MIIM_REGISTER_ADDRESS_MASK(index),
                                 _ETH_MIIM_REGISTER_ADDRESS_POS(index) );
}


//******************************************************************************
/* Function :  ETH_RegisterAddressSet_Default

  Summary:
    Implements Default variant of PLIB_ETH_RegisterAddressSet

  Description:
    This template implements the Default variant of the PLIB_ETH_RegisterAddressSet function.
*/

PLIB_TEMPLATE void ETH_RegisterAddressSet_Default( ETH_MODULE_ID index , uint8_t regAddr )
{
    _SFR_FIELD_WRITE(_ETH_MIIM_REGISTER_ADDRESS_VREG(index),
                     _ETH_MIIM_REGISTER_ADDRESS_MASK(index),
                     _ETH_MIIM_REGISTER_ADDRESS_POS(index) ,
                      regAddr                              );
}


//******************************************************************************
/* Function :  ETH_ExistsMIIMAddresses_Default

  Summary:
    Implements Default variant of PLIB_ETH_ExistsMIIMAddresses

  Description:
    This template implements the Default variant of the PLIB_ETH_ExistsMIIMAddresses function.
*/

#define PLIB_ETH_ExistsMIIMAddresses PLIB_ETH_ExistsMIIMAddresses
PLIB_TEMPLATE bool ETH_ExistsMIIMAddresses_Default( ETH_MODULE_ID index )
{
    return true;
}


#endif /*_ETH_MIIMADDRESSES_DEFAULT_H*/

/******************************************************************************
 End of File
*/

