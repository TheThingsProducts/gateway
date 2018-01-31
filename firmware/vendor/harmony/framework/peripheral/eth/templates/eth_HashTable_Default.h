/*******************************************************************************
  ETH Peripheral Library Template Implementation

  File Name:
    eth_HashTable_Default.h

  Summary:
    ETH PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : HashTable
    and its Variant : Default
    For following APIs :
        PLIB_ETH_HashTableSet
        PLIB_ETH_HashTableGet
        PLIB_ETH_ExistsHashTable

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

#ifndef _ETH_HASHTABLE_DEFAULT_H
#define _ETH_HASHTABLE_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _ETH_RX_ENABLE_VREG(index)
    _ETH_HASH_FILTER_ENABLE_VREG(index)
    _ETH_HASH_TABLE_LOWER_VREG(index)
    _ETH_HASH_TABLE_UPPER_VREG(index)

  MASKs:
    _ETH_RX_ENABLE_MASK(index)
    _ETH_HASH_FILTER_ENABLE_MASK(index)
    _ETH_HASH_TABLE_LOWER_MASK(index)
    _ETH_HASH_TABLE_UPPER_MASK(index)

  POSs:
    _ETH_RX_ENABLE_POS(index)
    _ETH_HASH_FILTER_ENABLE_POS(index)
    _ETH_HASH_TABLE_LOWER_POS(index)
    _ETH_HASH_TABLE_UPPER_POS(index)

  LENs:
    _ETH_RX_ENABLE_LEN(index)
    _ETH_HASH_FILTER_ENABLE_LEN(index)
    _ETH_HASH_TABLE_LOWER_LEN(index)
    _ETH_HASH_TABLE_UPPER_LEN(index)

*/


//******************************************************************************
/* Function :  ETH_HashTableSet_Default

  Summary:
    Implements Default variant of PLIB_ETH_HashTableSet

  Description:
    This template implements the Default variant of the PLIB_ETH_HashTableSet function.
*/

PLIB_TEMPLATE void ETH_HashTableSet_Default( ETH_MODULE_ID index , uint64_t hashTableValue )
{
    PLIB_ASSERT(
        !(bool)_SFR_BIT_READ(_ETH_RX_ENABLE_VREG(index),_ETH_RX_ENABLE_POS(index)) ||
        !(bool)_SFR_BIT_READ(_ETH_HASH_FILTER_ENABLE_MASK(index),_ETH_HASH_FILTER_ENABLE_POS(index)),
                "Receive must be disabled or Hash Table turned off!");
    _SFR_WRITE(_ETH_HASH_TABLE_LOWER_VREG(index),(uint32_t)hashTableValue);
    _SFR_WRITE(_ETH_HASH_TABLE_UPPER_VREG(index),(uint32_t)(hashTableValue>>32));
}


//******************************************************************************
/* Function :  ETH_HashTableGet_Default

  Summary:
    Implements Default variant of PLIB_ETH_HashTableGet

  Description:
    This template implements the Default variant of the PLIB_ETH_HashTableGet function.
*/

PLIB_TEMPLATE uint32_t ETH_HashTableGet_Default( ETH_MODULE_ID index )
{
    return ( (((uint64_t)_SFR_READ(_ETH_HASH_TABLE_UPPER_VREG(index)))<<32)
            + (uint64_t)_SFR_READ(_ETH_HASH_TABLE_LOWER_VREG(index))      );
}


//******************************************************************************
/* Function :  ETH_ExistsHashTable_Default

  Summary:
    Implements Default variant of PLIB_ETH_ExistsHashTable

  Description:
    This template implements the Default variant of the PLIB_ETH_ExistsHashTable function.
*/

#define PLIB_ETH_ExistsHashTable PLIB_ETH_ExistsHashTable
PLIB_TEMPLATE bool ETH_ExistsHashTable_Default( ETH_MODULE_ID index )
{
    return true;
}


#endif /*_ETH_HASHTABLE_DEFAULT_H*/

/******************************************************************************
 End of File
*/

