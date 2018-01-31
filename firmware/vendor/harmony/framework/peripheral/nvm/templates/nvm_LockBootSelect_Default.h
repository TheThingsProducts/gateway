/*******************************************************************************
  NVM Peripheral Library Template Implementation

  File Name:
    nvm_LockBootSelect_Default.h

  Summary:
    NVM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : LockBootSelect
    and its Variant : Default
    For following APIs :
        PLIB_NVM_ExistsLockBootSelect
        PLIB_NVM_LockBootMemory
        PLIB_NVM_IsBootMemoryLocked

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _NVM_LOCKBOOTSELECT_DEFAULT_H
#define _NVM_LOCKBOOTSELECT_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    _NVM_LOCK_LOWER_BOOT_MEM_VREG(index)
    _NVM_LOCK_UPPER_BOOT_MEM_VREG(index)

  MASKs: 
    _NVM_LOCK_LOWER_BOOT_MEM_MASK(index)
    _NVM_LOCK_UPPER_BOOT_MEM_MASK(index)

  POSs: 
    _NVM_LOCK_LOWER_BOOT_MEM_POS(index)
    _NVM_LOCK_UPPER_BOOT_MEM_POS(index)

  LENs: 
    _NVM_LOCK_LOWER_BOOT_MEM_LEN(index)
    _NVM_LOCK_UPPER_BOOT_MEM_LEN(index)

*/


//******************************************************************************
/* Function :  NVM_ExistsLockBootSelect_Default

  Summary:
    Implements Default variant of PLIB_NVM_ExistsLockBootSelect

  Description:
    This template implements the Default variant of the PLIB_NVM_ExistsLockBootSelect function.
*/

#define PLIB_NVM_ExistsLockBootSelect PLIB_NVM_ExistsLockBootSelect
PLIB_TEMPLATE bool NVM_ExistsLockBootSelect_Default( NVM_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  NVM_LockBootMemory_Default

  Summary:
    Implements Default variant of PLIB_NVM_LockBootMemory 

  Description:
    This template implements the Default variant of the PLIB_NVM_LockBootMemory function.
*/

PLIB_TEMPLATE void NVM_LockBootMemory_Default( NVM_MODULE_ID index , NVM_BOOT_MEMORY_AREA memoryArea )
{
    _SFR_BIT_CLEAR(_NVM_LOCK_UPPER_BOOT_MEM_VREG(index),
                     (_NVM_LOCK_UPPER_BOOT_MEM_POS(index) + memoryArea) );
}


//******************************************************************************
/* Function :  NVM_IsBootMemoryLocked_Default

  Summary:
    Implements Default variant of PLIB_NVM_IsBootMemoryLocked 

  Description:
    This template implements the Default variant of the PLIB_NVM_IsBootMemoryLocked function.
*/

PLIB_TEMPLATE bool NVM_IsBootMemoryLocked_Default( NVM_MODULE_ID index , NVM_BOOT_MEMORY_AREA memoryArea )
{
    return( !(_SFR_BIT_READ( _NVM_LOCK_UPPER_BOOT_MEM_VREG( index ),
                           (_NVM_LOCK_UPPER_BOOT_MEM_POS( index ) + memoryArea) ) ));
}


#endif /*_NVM_LOCKBOOTSELECT_DEFAULT_H*/

/******************************************************************************
 End of File
*/

