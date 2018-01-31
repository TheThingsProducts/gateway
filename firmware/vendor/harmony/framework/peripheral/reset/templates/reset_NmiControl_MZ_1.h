/*******************************************************************************
  RESET Peripheral Library Template Implementation

  File Name:
    reset_NmiControl_MZ_1.h

  Summary:
    RESET PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : NmiControl
    and its Variant : MZ_1
    For following APIs :
        PLIB_RESET_ExistsNmiControl
        PLIB_RESET_NmiReasonGet
        PLIB_RESET_NmiEventTrigger
        PLIB_RESET_NmiEventClear

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

#ifndef _RESET_NMICONTROL_MZ_1_H
#define _RESET_NMICONTROL_MZ_1_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    _RESET_DMTO_NMI_VREG(index)
    _RESET_WDTO_NMI_VREG(index)
    _RESET_SOFTWARE_NMI_VREG(index)
    _RESET_GLOBAL_NMI_VREG(index)
    _RESET_CLOCK_FAILURE_NMI_VREG(index)
    _RESET_WDTO_IN_SLEEP_VREG(index)
    _RESET_NMI_COUNT_VREG(index)

  MASKs: 
    _RESET_DMTO_NMI_MASK(index)
    _RESET_WDTO_NMI_MASK(index)
    _RESET_SOFTWARE_NMI_MASK(index)
    _RESET_GLOBAL_NMI_MASK(index)
    _RESET_CLOCK_FAILURE_NMI_MASK(index)
    _RESET_WDTO_IN_SLEEP_MASK(index)
    _RESET_NMI_COUNT_MASK(index)

  POSs: 
    _RESET_DMTO_NMI_POS(index)
    _RESET_WDTO_NMI_POS(index)
    _RESET_SOFTWARE_NMI_POS(index)
    _RESET_GLOBAL_NMI_POS(index)
    _RESET_CLOCK_FAILURE_NMI_POS(index)
    _RESET_WDTO_IN_SLEEP_POS(index)
    _RESET_NMI_COUNT_POS(index)

  LENs: 
    _RESET_DMTO_NMI_LEN(index)
    _RESET_WDTO_NMI_LEN(index)
    _RESET_SOFTWARE_NMI_LEN(index)
    _RESET_GLOBAL_NMI_LEN(index)
    _RESET_CLOCK_FAILURE_NMI_LEN(index)
    _RESET_WDTO_IN_SLEEP_LEN(index)
    _RESET_NMI_COUNT_LEN(index)

*/


//******************************************************************************
/* Function :  RESET_ExistsNmiControl_MZ_1

  Summary:
    Implements MZ_1 variant of PLIB_RESET_ExistsNmiControl

  Description:
    This template implements the MZ_1 variant of the PLIB_RESET_ExistsNmiControl function.
*/

#define PLIB_RESET_ExistsNmiControl PLIB_RESET_ExistsNmiControl
PLIB_TEMPLATE bool RESET_ExistsNmiControl_MZ_1( RESET_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  RESET_NmiReasonGet_MZ_1

  Summary:
    Implements MZ_1 variant of PLIB_RESET_NmiReasonGet 

  Description:
    This template implements the MZ_1 variant of the PLIB_RESET_NmiReasonGet function.
*/

PLIB_TEMPLATE RESET_NMI_REASON RESET_NmiReasonGet_MZ_1( RESET_MODULE_ID index )
{
    return(RESET_NMI_REASON)((_SFR_READ(_RESET_DMTO_NMI_VREG(index)))& (~_RESET_NMI_COUNT_MASK(index)));
}


//******************************************************************************
/* Function :  RESET_NmiEventTrigger_MZ_1

  Summary:
    Implements MZ_1 variant of PLIB_RESET_NmiEventTrigger 

  Description:
    This template implements the MZ_1 variant of the PLIB_RESET_NmiEventTrigger function.
*/

PLIB_TEMPLATE void RESET_NmiEventTrigger_MZ_1( RESET_MODULE_ID index , RESET_NMI_REASON nmi_reason )
{
    _SFR_WRITE((_RESET_DMTO_NMI_VREG(index)), (*(_RESET_DMTO_NMI_VREG(index)) | (nmi_reason)));
}


//******************************************************************************
/* Function :  RESET_NmiEventClear_MZ_1

  Summary:
    Implements MZ_1 variant of PLIB_RESET_NmiEventClear 

  Description:
    This template implements the MZ_1 variant of the PLIB_RESET_NmiEventClear function.
*/

PLIB_TEMPLATE void RESET_NmiEventClear_MZ_1( RESET_MODULE_ID index , RESET_NMI_REASON nmi_reason )
{
    _SFR_WRITE((_RESET_DMTO_NMI_VREG(index)), (*(_RESET_DMTO_NMI_VREG(index)) & (~nmi_reason)));
}


#endif /*_RESET_NMICONTROL_MZ_1_H*/

/******************************************************************************
 End of File
*/

