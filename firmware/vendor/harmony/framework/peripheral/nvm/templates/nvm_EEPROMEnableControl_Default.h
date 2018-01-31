/*******************************************************************************
  NVM Peripheral Library Template Implementation

  File Name:
    nvm_EEPROMEnableControl_Default.h

  Summary:
    NVM PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : EEPROMEnableControl
    and its Variant : Default
    For following APIs :
        PLIB_NVM_ExistsEEPROMEnableControl
        PLIB_NVM_EEPROMEnable
        PLIB_NVM_EEPROMDisable
        PLIB_NVM_EEPROMIsReady

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

#ifndef _NVM_EEPROMENABLECONTROL_DEFAULT_H
#define _NVM_EEPROMENABLECONTROL_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    _NVM_EEPROM_ENABLE_VREG(index)
    _NVM_EEPROM_READY_STATUS_VREG(index)

  MASKs: 
    _NVM_EEPROM_ENABLE_MASK(index)
    _NVM_EEPROM_READY_STATUS_MASK(index)

  POSs: 
    _NVM_EEPROM_ENABLE_POS(index)
    _NVM_EEPROM_READY_STATUS_POS(index)

  LENs: 
    _NVM_EEPROM_ENABLE_LEN(index)
    _NVM_EEPROM_READY_STATUS_LEN(index)

*/


//******************************************************************************
/* Function :  NVM_ExistsEEPROMEnableControl_Default

  Summary:
    Implements Default variant of PLIB_NVM_ExistsEEPROMEnableControl

  Description:
    This template implements the Default variant of the PLIB_NVM_ExistsEEPROMEnableControl function.
*/

#define PLIB_NVM_ExistsEEPROMEnableControl PLIB_NVM_ExistsEEPROMEnableControl
PLIB_TEMPLATE bool NVM_ExistsEEPROMEnableControl_Default( NVM_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  NVM_EEPROMEnable_Default

  Summary:
    Implements Default variant of PLIB_NVM_EEPROMEnable 

  Description:
    This template implements the Default variant of the PLIB_NVM_EEPROMEnable function.
*/

PLIB_TEMPLATE void NVM_EEPROMEnable_Default( NVM_MODULE_ID index )
{
    _SFR_BIT_SET(_NVM_EEPROM_ENABLE_VREG(index), _NVM_EEPROM_ENABLE_POS(index));
}


//******************************************************************************
/* Function :  NVM_EEPROMDisable_Default

  Summary:
    Implements Default variant of PLIB_NVM_EEPROMDisable 

  Description:
    This template implements the Default variant of the PLIB_NVM_EEPROMDisable function.
*/

PLIB_TEMPLATE void NVM_EEPROMDisable_Default( NVM_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_NVM_EEPROM_ENABLE_VREG(index), _NVM_EEPROM_ENABLE_POS(index));
}


//******************************************************************************
/* Function :  NVM_EEPROMIsReady_Default

  Summary:
    Implements Default variant of PLIB_NVM_EEPROMIsReady 

  Description:
    This template implements the Default variant of the PLIB_NVM_EEPROMIsReady function.
*/

PLIB_TEMPLATE bool NVM_EEPROMIsReady_Default( NVM_MODULE_ID index )
{
    return (bool)(_SFR_BIT_READ(_NVM_EEPROM_READY_STATUS_VREG(index), _NVM_EEPROM_READY_STATUS_POS(index)));
}


#endif /*_NVM_EEPROMENABLECONTROL_DEFAULT_H*/

/******************************************************************************
 End of File
*/

