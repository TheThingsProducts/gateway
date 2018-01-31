/*******************************************************************************
  NVM Peripheral Library Template Implementation

  File Name:
    nvm_BootPageWriteProtect_Default.h

  Summary:
    NVM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : BootPageWriteProtect
    and its Variant : Default
    For following APIs :
        PLIB_NVM_ExistsBootPageWriteProtect
        PLIB_NVM_BootPageWriteProtectionEnable
        PLIB_NVM_BootPageWriteProtectionDisable
        PLIB_NVM_IsBootPageWriteProtected

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

#ifndef _NVM_BOOTPAGEWRITEPROTECT_DEFAULT_H
#define _NVM_BOOTPAGEWRITEPROTECT_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    _NVM_BOOT_LBWP0_VREG(index)
    _NVM_BOOT_LBWP1_VREG(index)
    _NVM_BOOT_LBWP2_VREG(index)
    _NVM_BOOT_LBWP3_VREG(index)
    _NVM_BOOT_LBWP4_VREG(index)
    _NVM_BOOT_UBWP0_VREG(index)
    _NVM_BOOT_UBWP1_VREG(index)
    _NVM_BOOT_UBWP2_VREG(index)
    _NVM_BOOT_UBWP3_VREG(index)
    _NVM_BOOT_UBWP4_VREG(index)

  MASKs: 
    _NVM_BOOT_LBWP0_MASK(index)
    _NVM_BOOT_LBWP1_MASK(index)
    _NVM_BOOT_LBWP2_MASK(index)
    _NVM_BOOT_LBWP3_MASK(index)
    _NVM_BOOT_LBWP4_MASK(index)
    _NVM_BOOT_UBWP0_MASK(index)
    _NVM_BOOT_UBWP1_MASK(index)
    _NVM_BOOT_UBWP2_MASK(index)
    _NVM_BOOT_UBWP3_MASK(index)
    _NVM_BOOT_UBWP4_MASK(index)

  POSs: 
    _NVM_BOOT_LBWP0_POS(index)
    _NVM_BOOT_LBWP1_POS(index)
    _NVM_BOOT_LBWP2_POS(index)
    _NVM_BOOT_LBWP3_POS(index)
    _NVM_BOOT_LBWP4_POS(index)
    _NVM_BOOT_UBWP0_POS(index)
    _NVM_BOOT_UBWP1_POS(index)
    _NVM_BOOT_UBWP2_POS(index)
    _NVM_BOOT_UBWP3_POS(index)
    _NVM_BOOT_UBWP4_POS(index)

  LENs: 
    _NVM_BOOT_LBWP0_LEN(index)
    _NVM_BOOT_LBWP1_LEN(index)
    _NVM_BOOT_LBWP2_LEN(index)
    _NVM_BOOT_LBWP3_LEN(index)
    _NVM_BOOT_LBWP4_LEN(index)
    _NVM_BOOT_UBWP0_LEN(index)
    _NVM_BOOT_UBWP1_LEN(index)
    _NVM_BOOT_UBWP2_LEN(index)
    _NVM_BOOT_UBWP3_LEN(index)
    _NVM_BOOT_UBWP4_LEN(index)

*/


//******************************************************************************
/* Function :  NVM_ExistsBootPageWriteProtect_Default

  Summary:
    Implements Default variant of PLIB_NVM_ExistsBootPageWriteProtect

  Description:
    This template implements the Default variant of the PLIB_NVM_ExistsBootPageWriteProtect function.
*/

#define PLIB_NVM_ExistsBootPageWriteProtect PLIB_NVM_ExistsBootPageWriteProtect
PLIB_TEMPLATE bool NVM_ExistsBootPageWriteProtect_Default( NVM_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  NVM_BootPageWriteProtectionEnable_Default

  Summary:
    Implements Default variant of PLIB_NVM_BootPageWriteProtectionEnable 

  Description:
    This template implements the Default variant of the PLIB_NVM_BootPageWriteProtectionEnable function.
*/

PLIB_TEMPLATE void NVM_BootPageWriteProtectionEnable_Default( NVM_MODULE_ID index , NVM_BOOT_MEMORY_PAGE bootPage )
{
    _SFR_BIT_SET(_NVM_BOOT_UBWP0_VREG(index),
                     (_NVM_BOOT_UBWP0_POS(index) + bootPage) );
}


//******************************************************************************
/* Function :  NVM_BootPageWriteProtectionDisable_Default

  Summary:
    Implements Default variant of PLIB_NVM_BootPageWriteProtectionDisable 

  Description:
    This template implements the Default variant of the PLIB_NVM_BootPageWriteProtectionDisable function.
*/

PLIB_TEMPLATE void NVM_BootPageWriteProtectionDisable_Default( NVM_MODULE_ID index , NVM_BOOT_MEMORY_PAGE bootPage )
{
    _SFR_BIT_CLEAR(_NVM_BOOT_UBWP0_VREG(index),
                     (_NVM_BOOT_UBWP0_POS(index) + bootPage) );
}


//******************************************************************************
/* Function :  NVM_IsBootPageWriteProtected_Default

  Summary:
    Implements Default variant of PLIB_NVM_IsBootPageWriteProtected 

  Description:
    This template implements the Default variant of the PLIB_NVM_IsBootPageWriteProtected function.
*/

PLIB_TEMPLATE bool NVM_IsBootPageWriteProtected_Default( NVM_MODULE_ID index , NVM_BOOT_MEMORY_PAGE bootPage )
{
    return(_SFR_BIT_READ( _NVM_BOOT_UBWP0_VREG( index ),
                           (_NVM_BOOT_UBWP0_POS( index ) + bootPage) ));
}


#endif /*_NVM_BOOTPAGEWRITEPROTECT_DEFAULT_H*/

/******************************************************************************
 End of File
*/

