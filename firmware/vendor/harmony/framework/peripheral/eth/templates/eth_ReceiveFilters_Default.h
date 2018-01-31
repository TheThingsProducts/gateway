/*******************************************************************************
  ETH Peripheral Library Template Implementation

  File Name:
    eth_ReceiveFilters_Default.h

  Summary:
    ETH PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ReceiveFilters
    and its Variant : Default
    For following APIs :
        PLIB_ETH_ReceiveFilterEnable
        PLIB_ETH_ReceiveFilterDisable
        PLIB_ETH_ReceiveFilterIsEnable
        PLIB_ETH_ExistsReceiveFilters

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

#ifndef _ETH_RECEIVEFILTERS_DEFAULT_H
#define _ETH_RECEIVEFILTERS_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _ETH_RX_ENABLE_VREG(index)
    _ETH_HASH_FILTER_ENABLE_VREG(index)
    _ETH_MAGIC_FILTER_ENABLE_VREG(index)
    _ETH_INVERT_PATTERN_MATCH_VREG(index)
    _ETH_CRC_ERR_ENABLE_VREG(index)
    _ETH_CRC_OK_ENABLE_VREG(index)
    _ETH_RUNT_ERR_ENABLE_VREG(index)
    _ETH_RUNT_ENABLE_VREG(index)
    _ETH_UNICAST_ENABLE_VREG(index)
    _ETH_NOT_ME_ENABLE_VREG(index)
    _ETH_MULTICAST_ENABLE_VREG(index)
    _ETH_BROADCAST_ENABLE_VREG(index)

  MASKs:
    _ETH_RX_ENABLE_MASK(index)
    _ETH_HASH_FILTER_ENABLE_MASK(index)
    _ETH_MAGIC_FILTER_ENABLE_MASK(index)
    _ETH_INVERT_PATTERN_MATCH_MASK(index)
    _ETH_CRC_ERR_ENABLE_MASK(index)
    _ETH_CRC_OK_ENABLE_MASK(index)
    _ETH_RUNT_ERR_ENABLE_MASK(index)
    _ETH_RUNT_ENABLE_MASK(index)
    _ETH_UNICAST_ENABLE_MASK(index)
    _ETH_NOT_ME_ENABLE_MASK(index)
    _ETH_MULTICAST_ENABLE_MASK(index)
    _ETH_BROADCAST_ENABLE_MASK(index)

  POSs:
    _ETH_RX_ENABLE_POS(index)
    _ETH_HASH_FILTER_ENABLE_POS(index)
    _ETH_MAGIC_FILTER_ENABLE_POS(index)
    _ETH_INVERT_PATTERN_MATCH_POS(index)
    _ETH_CRC_ERR_ENABLE_POS(index)
    _ETH_CRC_OK_ENABLE_POS(index)
    _ETH_RUNT_ERR_ENABLE_POS(index)
    _ETH_RUNT_ENABLE_POS(index)
    _ETH_UNICAST_ENABLE_POS(index)
    _ETH_NOT_ME_ENABLE_POS(index)
    _ETH_MULTICAST_ENABLE_POS(index)
    _ETH_BROADCAST_ENABLE_POS(index)

  LENs:
    _ETH_RX_ENABLE_LEN(index)
    _ETH_HASH_FILTER_ENABLE_LEN(index)
    _ETH_MAGIC_FILTER_ENABLE_LEN(index)
    _ETH_INVERT_PATTERN_MATCH_LEN(index)
    _ETH_CRC_ERR_ENABLE_LEN(index)
    _ETH_CRC_OK_ENABLE_LEN(index)
    _ETH_RUNT_ERR_ENABLE_LEN(index)
    _ETH_RUNT_ENABLE_LEN(index)
    _ETH_UNICAST_ENABLE_LEN(index)
    _ETH_NOT_ME_ENABLE_LEN(index)
    _ETH_MULTICAST_ENABLE_LEN(index)
    _ETH_BROADCAST_ENABLE_LEN(index)

*/


//******************************************************************************
/* Function :  ETH_ReceiveFilterEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_ReceiveFilterEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_ReceiveFilterEnable function.
*/

PLIB_TEMPLATE void ETH_ReceiveFilterEnable_Default( ETH_MODULE_ID index , ETH_RECEIVE_FILTER filter )
{
  //PLIB_ASSERT(!(bool)_SFR_BIT_READ(_ETH_RX_ENABLE_VREG(index),_ETH_RX_ENABLE_POS(index)),
  //            "Receive must be disabled!");
    _SFR_SET(_ETH_BROADCAST_ENABLE_VREG(index),(uint16_t)filter);
}


//******************************************************************************
/* Function :  ETH_ReceiveFilterDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_ReceiveFilterDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_ReceiveFilterDisable function.
*/

PLIB_TEMPLATE void ETH_ReceiveFilterDisable_Default( ETH_MODULE_ID index , ETH_RECEIVE_FILTER filter )
{
  //PLIB_ASSERT(!(bool)_SFR_BIT_READ(_ETH_RX_ENABLE_VREG(index),_ETH_RX_ENABLE_POS(index)),
  //            "Receive must be disabled!");
    _SFR_CLEAR(_ETH_BROADCAST_ENABLE_VREG(index),(uint16_t)filter);
}


//******************************************************************************
/* Function :  ETH_ReceiveFilterIsEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_ReceiveFilterIsEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_ReceiveFilterIsEnable function.
*/

PLIB_TEMPLATE bool ETH_ReceiveFilterIsEnable_Default( ETH_MODULE_ID index , ETH_RECEIVE_FILTER filter )
{
    return (_SFR_READ(_ETH_BROADCAST_ENABLE_VREG(index))&(uint16_t)filter) > 0 ? true : false ;
}


//******************************************************************************
/* Function :  ETH_ExistsReceiveFilters_Default

  Summary:
    Implements Default variant of PLIB_ETH_ExistsReceiveFilters

  Description:
    This template implements the Default variant of the PLIB_ETH_ExistsReceiveFilters function.
*/

#define PLIB_ETH_ExistsReceiveFilters PLIB_ETH_ExistsReceiveFilters
PLIB_TEMPLATE bool ETH_ExistsReceiveFilters_Default( ETH_MODULE_ID index )
{
    return true;
}


#endif /*_ETH_RECEIVEFILTERS_DEFAULT_H*/

/******************************************************************************
 End of File
*/

