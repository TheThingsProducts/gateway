/*******************************************************************************
  RESET Peripheral Library Template Implementation

  File Name:
    reset_ResetReasonStatus_MZ.h

  Summary:
    RESET PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ResetReasonStatus
    and its Variant : MZ
    For following APIs :
        PLIB_RESET_ExistsResetReasonStatus
        PLIB_RESET_ReasonGet
        PLIB_RESET_ReasonClear

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

#ifndef _RESET_RESETREASONSTATUS_MZ_H
#define _RESET_RESETREASONSTATUS_MZ_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    _RESET_RESET_REASON_STATUS_CMR_VREG(index)
    _RESET_RESET_REASON_STATUS_EXTR_VREG(index)
    _RESET_RESET_REASON_STATUS_SWR_VREG(index)
    _RESET_RESET_REASON_STATUS_DMT0_VREG(index)
    _RESET_RESET_REASON_STATUS_WDT0_VREG(index)
    _RESET_RESET_REASON_STATUS_BOR_VREG(index)
    _RESET_RESET_REASON_STATUS_POR_VREG(index)

  MASKs: 
    _RESET_RESET_REASON_STATUS_CMR_MASK(index)
    _RESET_RESET_REASON_STATUS_EXTR_MASK(index)
    _RESET_RESET_REASON_STATUS_SWR_MASK(index)
    _RESET_RESET_REASON_STATUS_DMT0_MASK(index)
    _RESET_RESET_REASON_STATUS_WDT0_MASK(index)
    _RESET_RESET_REASON_STATUS_BOR_MASK(index)
    _RESET_RESET_REASON_STATUS_POR_MASK(index)
	
	_RESET_RESET_PRIMARY_CONFIG_REG_READ_ERROR_MASK(index)
    _RESET_RESET_BOTH_CONFIG_REG_READ_ERROR_MASK(index)

  POSs: 
    _RESET_RESET_REASON_STATUS_CMR_POS(index)
    _RESET_RESET_REASON_STATUS_EXTR_POS(index)
    _RESET_RESET_REASON_STATUS_SWR_POS(index)
    _RESET_RESET_REASON_STATUS_DMT0_POS(index)
    _RESET_RESET_REASON_STATUS_WDT0_POS(index)
    _RESET_RESET_REASON_STATUS_BOR_POS(index)
    _RESET_RESET_REASON_STATUS_POR_POS(index)

  LENs: 
    _RESET_RESET_REASON_STATUS_CMR_LEN(index)
    _RESET_RESET_REASON_STATUS_EXTR_LEN(index)
    _RESET_RESET_REASON_STATUS_SWR_LEN(index)
    _RESET_RESET_REASON_STATUS_DMT0_LEN(index)
    _RESET_RESET_REASON_STATUS_WDT0_LEN(index)
    _RESET_RESET_REASON_STATUS_BOR_LEN(index)
    _RESET_RESET_REASON_STATUS_POR_LEN(index)

*/


//******************************************************************************
/* Function :  RESET_ExistsResetReasonStatus_MZ

  Summary:
    Implements MZ variant of PLIB_RESET_ExistsResetReasonStatus

  Description:
    This template implements the MZ variant of the PLIB_RESET_ExistsResetReasonStatus function.
*/

#define PLIB_RESET_ExistsResetReasonStatus PLIB_RESET_ExistsResetReasonStatus
PLIB_TEMPLATE bool RESET_ExistsResetReasonStatus_MZ( RESET_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  RESET_ReasonGet_MZ

  Summary:
    Implements MZ variant of PLIB_RESET_ReasonGet 

  Description:
    This template implements the MZ variant of the PLIB_RESET_ReasonGet function.
*/

PLIB_TEMPLATE RESET_REASON RESET_ReasonGet_MZ( RESET_MODULE_ID index )
{

        return(RESET_REASON)(
                (_SFR_READ(_RESET_RESET_REASON_STATUS_EXTR_VREG(index)))&
                (
                    (_RESET_RESET_REASON_STATUS_CMR_MASK(index)) |
                    (_RESET_RESET_REASON_STATUS_EXTR_MASK(index)) |
                    (_RESET_RESET_REASON_STATUS_SWR_MASK(index)) |
                    (_RESET_RESET_REASON_STATUS_DMT0_MASK(index))|
                    (_RESET_RESET_REASON_STATUS_WDT0_MASK(index)) |
                    (_RESET_RESET_REASON_STATUS_BOR_MASK(index)) |
                    (_RESET_RESET_REASON_STATUS_POR_MASK(index))
                )
              );
}


//******************************************************************************
/* Function :  RESET_ReasonClear_MZ

  Summary:
    Implements MZ variant of PLIB_RESET_ReasonClear 

  Description:
    This template implements the MZ variant of the PLIB_RESET_ReasonClear function.
*/

PLIB_TEMPLATE void RESET_ReasonClear_MZ( RESET_MODULE_ID index , RESET_REASON reason )
{
   _SFR_READ(_RESET_RESET_REASON_STATUS_EXTR_VREG(index)) = (~reason) & (_SFR_READ(_RESET_RESET_REASON_STATUS_EXTR_VREG(index)));
}


#endif /*_RESET_RESETREASONSTATUS_MZ_H*/

/******************************************************************************
 End of File
*/

