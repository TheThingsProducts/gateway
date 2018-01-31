/*******************************************************************************
  ETH Peripheral Library Template Implementation

  File Name:
    eth_InterPacketGaps_Default.h

  Summary:
    ETH PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : InterPacketGaps
    and its Variant : Default
    For following APIs :
        PLIB_ETH_BackToBackIPGGet
        PLIB_ETH_BackToBackIPGSet
        PLIB_ETH_NonBackToBackIPG1Get
        PLIB_ETH_NonBackToBackIPG1Set
        PLIB_ETH_NonBackToBackIPG2Get
        PLIB_ETH_NonBackToBackIPG2Set
        PLIB_ETH_ExistsInterPacketGaps

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

#ifndef _ETH_INTERPACKETGAPS_DEFAULT_H
#define _ETH_INTERPACKETGAPS_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _ETH_BACK2BACK_IPGAP_VREG(index)
    _ETH_NON_BACK2BACK_IPGAP1_VREG(index)
    _ETH_NON_BACK2BACK_IPGAP2_VREG(index)

  MASKs:
    _ETH_BACK2BACK_IPGAP_MASK(index)
    _ETH_NON_BACK2BACK_IPGAP1_MASK(index)
    _ETH_NON_BACK2BACK_IPGAP2_MASK(index)

  POSs:
    _ETH_BACK2BACK_IPGAP_POS(index)
    _ETH_NON_BACK2BACK_IPGAP1_POS(index)
    _ETH_NON_BACK2BACK_IPGAP2_POS(index)

  LENs:
    _ETH_BACK2BACK_IPGAP_LEN(index)
    _ETH_NON_BACK2BACK_IPGAP1_LEN(index)
    _ETH_NON_BACK2BACK_IPGAP2_LEN(index)

*/


//******************************************************************************
/* Function :  ETH_BackToBackIPGGet_Default

  Summary:
    Implements Default variant of PLIB_ETH_BackToBackIPGGet

  Description:
    This template implements the Default variant of the PLIB_ETH_BackToBackIPGGet function.
*/

PLIB_TEMPLATE uint8_t ETH_BackToBackIPGGet_Default( ETH_MODULE_ID index )
{
    return (uint8_t)_SFR_FIELD_READ(_ETH_BACK2BACK_IPGAP_VREG(index),
                                    _ETH_BACK2BACK_IPGAP_MASK(index),
                                    _ETH_BACK2BACK_IPGAP_POS(index) );
}


//******************************************************************************
/* Function :  ETH_BackToBackIPGSet_Default

  Summary:
    Implements Default variant of PLIB_ETH_BackToBackIPGSet

  Description:
    This template implements the Default variant of the PLIB_ETH_BackToBackIPGSet function.
*/

PLIB_TEMPLATE void ETH_BackToBackIPGSet_Default( ETH_MODULE_ID index , uint8_t backToBackIPGValue )
{
    _SFR_FIELD_WRITE(_ETH_BACK2BACK_IPGAP_VREG(index),
                     _ETH_BACK2BACK_IPGAP_MASK(index),
                     _ETH_BACK2BACK_IPGAP_POS(index) ,
                     backToBackIPGValue              );
}


//******************************************************************************
/* Function :  ETH_NonBackToBackIPG1Get_Default

  Summary:
    Implements Default variant of PLIB_ETH_NonBackToBackIPG1Get

  Description:
    This template implements the Default variant of the PLIB_ETH_NonBackToBackIPG1Get function.
*/

PLIB_TEMPLATE uint8_t ETH_NonBackToBackIPG1Get_Default( ETH_MODULE_ID index )
{
    return (uint8_t)_SFR_FIELD_READ(_ETH_NON_BACK2BACK_IPGAP1_VREG(index),
                                    _ETH_NON_BACK2BACK_IPGAP1_MASK(index),
                                    _ETH_NON_BACK2BACK_IPGAP1_POS(index) );
}


//******************************************************************************
/* Function :  ETH_NonBackToBackIPG1Set_Default

  Summary:
    Implements Default variant of PLIB_ETH_NonBackToBackIPG1Set

  Description:
    This template implements the Default variant of the PLIB_ETH_NonBackToBackIPG1Set function.
*/

PLIB_TEMPLATE void ETH_NonBackToBackIPG1Set_Default( ETH_MODULE_ID index , uint8_t nonBackToBackIPGValue )
{
    _SFR_FIELD_WRITE(_ETH_NON_BACK2BACK_IPGAP1_VREG(index),
                     _ETH_NON_BACK2BACK_IPGAP1_MASK(index),
                     _ETH_NON_BACK2BACK_IPGAP1_POS(index) ,
                     nonBackToBackIPGValue                );
}


//******************************************************************************
/* Function :  ETH_NonBackToBackIPG2Get_Default

  Summary:
    Implements Default variant of PLIB_ETH_NonBackToBackIPG2Get

  Description:
    This template implements the Default variant of the PLIB_ETH_NonBackToBackIPG2Get function.
*/

PLIB_TEMPLATE uint8_t ETH_NonBackToBackIPG2Get_Default( ETH_MODULE_ID index )
{
    return (uint8_t)_SFR_FIELD_READ(_ETH_NON_BACK2BACK_IPGAP2_VREG(index),
                                    _ETH_NON_BACK2BACK_IPGAP2_MASK(index),
                                    _ETH_NON_BACK2BACK_IPGAP2_POS(index) );

}


//******************************************************************************
/* Function :  ETH_NonBackToBackIPG2Set_Default

  Summary:
    Implements Default variant of PLIB_ETH_NonBackToBackIPG2Set

  Description:
    This template implements the Default variant of the PLIB_ETH_NonBackToBackIPG2Set function.
*/

PLIB_TEMPLATE void ETH_NonBackToBackIPG2Set_Default( ETH_MODULE_ID index , uint8_t nonBackToBackIPGValue )
{
    _SFR_FIELD_WRITE(_ETH_NON_BACK2BACK_IPGAP2_VREG(index),
                     _ETH_NON_BACK2BACK_IPGAP2_MASK(index),
                     _ETH_NON_BACK2BACK_IPGAP2_POS(index) ,
                     nonBackToBackIPGValue                );
}


//******************************************************************************
/* Function :  ETH_ExistsInterPacketGaps_Default

  Summary:
    Implements Default variant of PLIB_ETH_ExistsInterPacketGaps

  Description:
    This template implements the Default variant of the PLIB_ETH_ExistsInterPacketGaps function.
*/

#define PLIB_ETH_ExistsInterPacketGaps PLIB_ETH_ExistsInterPacketGaps
PLIB_TEMPLATE bool ETH_ExistsInterPacketGaps_Default( ETH_MODULE_ID index )
{
    return true;
}


#endif /*_ETH_INTERPACKETGAPS_DEFAULT_H*/

/******************************************************************************
 End of File
*/

