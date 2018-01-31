/*******************************************************************************
  ETH Peripheral Library Template Implementation

  File Name:
    eth_RetransmissionMaximum_Default.h

  Summary:
    ETH PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : RetransmissionMaximum
    and its Variant : Default
    For following APIs :
        PLIB_ETH_ReTxMaxGet
        PLIB_ETH_ReTxMaxSet
        PLIB_ETH_ExistsRetransmissionMaximum

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

#ifndef _ETH_RETRANSMISSIONMAXIMUM_DEFAULT_H
#define _ETH_RETRANSMISSIONMAXIMUM_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _ETH_RETX_MAX_VREG(index)

  MASKs:
    _ETH_RETX_MAX_MASK(index)

  POSs:
    _ETH_RETX_MAX_POS(index)

  LENs:
    _ETH_RETX_MAX_LEN(index)

*/


//******************************************************************************
/* Function :  ETH_ReTxMaxGet_Default

  Summary:
    Implements Default variant of PLIB_ETH_ReTxMaxGet

  Description:
    This template implements the Default variant of the PLIB_ETH_ReTxMaxGet function.
*/

PLIB_TEMPLATE uint8_t ETH_ReTxMaxGet_Default( ETH_MODULE_ID index )
{
    return (uint8_t)_SFR_FIELD_READ(_ETH_RETX_MAX_VREG(index),
                                    _ETH_RETX_MAX_MASK(index),
                                    _ETH_RETX_MAX_POS(index) );
}


//******************************************************************************
/* Function :  ETH_ReTxMaxSet_Default

  Summary:
    Implements Default variant of PLIB_ETH_ReTxMaxSet

  Description:
    This template implements the Default variant of the PLIB_ETH_ReTxMaxSet function.
*/

PLIB_TEMPLATE void ETH_ReTxMaxSet_Default( ETH_MODULE_ID index , uint16_t retransmitMax )
{
    _SFR_FIELD_WRITE(_ETH_RETX_MAX_VREG(index),
                     _ETH_RETX_MAX_MASK(index),
                     _ETH_RETX_MAX_POS(index) ,
                     retransmitMax            );
}


//******************************************************************************
/* Function :  ETH_ExistsRetransmissionMaximum_Default

  Summary:
    Implements Default variant of PLIB_ETH_ExistsRetransmissionMaximum

  Description:
    This template implements the Default variant of the PLIB_ETH_ExistsRetransmissionMaximum function.
*/

#define PLIB_ETH_ExistsRetransmissionMaximum PLIB_ETH_ExistsRetransmissionMaximum
PLIB_TEMPLATE bool ETH_ExistsRetransmissionMaximum_Default( ETH_MODULE_ID index )
{
    return true;
}


#endif /*_ETH_RETRANSMISSIONMAXIMUM_DEFAULT_H*/

/******************************************************************************
 End of File
*/

