/*******************************************************************************
  PCACHE Peripheral Library Template Implementation

  File Name:
    pcache_CacheLineAddr_Default.h

  Summary:
    PCACHE PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : CacheLineAddr
    and its Variant : Default
    For following APIs :
        PLIB_PCACHE_CacheLineAddrSet
        PLIB_PCACHE_CacheLineAddrGet
        PLIB_PCACHE_ExistsCacheLineAddr

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

#ifndef _PCACHE_CACHELINEADDR_DEFAULT_H
#define _PCACHE_CACHELINEADDR_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    _PCACHE_CACHE_LINE_ADDR_VREG(index)

  MASKs: 
    _PCACHE_CACHE_LINE_ADDR_MASK(index)

  POSs: 
    _PCACHE_CACHE_LINE_ADDR_POS(index)

  LENs: 
    _PCACHE_CACHE_LINE_ADDR_LEN(index)

*/


//******************************************************************************
/* Function :  PCACHE_CacheLineAddrSet_Default

  Summary:
    Implements Default variant of PLIB_PCACHE_CacheLineAddrSet 

  Description:
    This template implements the Default variant of the PLIB_PCACHE_CacheLineAddrSet function.
*/

PLIB_TEMPLATE void PCACHE_CacheLineAddrSet_Default( PCACHE_MODULE_ID index , uint32_t addr )
{
    _SFR_FIELD_WRITE(_PCACHE_CACHE_LINE_ADDR_VREG(index), _PCACHE_CACHE_LINE_ADDR_MASK(index), _PCACHE_CACHE_LINE_ADDR_POS(index), (addr >> 4));
}


//******************************************************************************
/* Function :  PCACHE_CacheLineAddrGet_Default

  Summary:
    Implements Default variant of PLIB_PCACHE_CacheLineAddrGet 

  Description:
    This template implements the Default variant of the PLIB_PCACHE_CacheLineAddrGet function.
*/

PLIB_TEMPLATE uint32_t PCACHE_CacheLineAddrGet_Default( PCACHE_MODULE_ID index )
{
    return (_SFR_READ(_PCACHE_CACHE_LINE_ADDR_VREG(index)) & 0x00FFFFF0);
}


//******************************************************************************
/* Function :  PCACHE_ExistsCacheLineAddr_Default

  Summary:
    Implements Default variant of PLIB_PCACHE_ExistsCacheLineAddr

  Description:
    This template implements the Default variant of the PLIB_PCACHE_ExistsCacheLineAddr function.
*/

#define PLIB_PCACHE_ExistsCacheLineAddr PLIB_PCACHE_ExistsCacheLineAddr
PLIB_TEMPLATE bool PCACHE_ExistsCacheLineAddr_Default( PCACHE_MODULE_ID index )
{
    return true;
}


#endif /*_PCACHE_CACHELINEADDR_DEFAULT_H*/

/******************************************************************************
 End of File
*/

