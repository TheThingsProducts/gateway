/*******************************************************************************
  OSC Peripheral Library Template Implementation

  File Name:
    osc_PLLBypass_PIC32WK.h

  Summary:
    OSC PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PLLBypass
    and its Variant : PIC32WK
    For following APIs :
        PLIB_OSC_ExistsPLLBypass
        PLIB_OSC_PLLBypassEnable
        PLIB_OSC_PLLBypassDisable
        PLIB_OSC_PLLBypassStatus

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

#ifndef _OSC_PLLBYPASS_PIC32WK_H
#define _OSC_PLLBYPASS_PIC32WK_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    _OSC_OSC_SYSTEM_BYPASS_VREG(index)
    _OSC_OSC_USB_BYPASS_VREG(index)
    _OSC_OSC_BT_BYPASS_VREG(index)

  MASKs: 
    _OSC_OSC_SYSTEM_BYPASS_MASK(index)
    _OSC_OSC_USB_BYPASS_MASK(index)
    _OSC_OSC_BT_BYPASS_MASK(index)

  POSs: 
    _OSC_OSC_SYSTEM_BYPASS_POS(index)
    _OSC_OSC_USB_BYPASS_POS(index)
    _OSC_OSC_BT_BYPASS_POS(index)

  LENs: 
    _OSC_OSC_SYSTEM_BYPASS_LEN(index)
    _OSC_OSC_USB_BYPASS_LEN(index)
    _OSC_OSC_BT_BYPASS_LEN(index)

*/


//******************************************************************************
/* Function :  OSC_ExistsPLLBypass_PIC32WK

  Summary:
    Implements PIC32WK variant of PLIB_OSC_ExistsPLLBypass

  Description:
    This template implements the PIC32WK variant of the PLIB_OSC_ExistsPLLBypass function.
*/

#define PLIB_OSC_ExistsPLLBypass PLIB_OSC_ExistsPLLBypass
PLIB_TEMPLATE bool OSC_ExistsPLLBypass_PIC32WK( OSC_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  OSC_PLLBypassEnable_PIC32WK

  Summary:
    Implements PIC32WK variant of PLIB_OSC_PLLBypassEnable 

  Description:
    This template implements the PIC32WK variant of the PLIB_OSC_PLLBypassEnable function.
*/

PLIB_TEMPLATE void OSC_PLLBypassEnable_PIC32WK( OSC_MODULE_ID index , OSC_PLL_SELECT pllSel )
{
    switch(pllSel)
	{
		case OSC_PLL_SYSTEM:
			_SFR_BIT_SET(_OSC_OSC_SYSTEM_BYPASS_VREG(index),
                       _OSC_OSC_SYSTEM_BYPASS_POS(index));
			break;
			
		case OSC_PLL_USB:
			_SFR_BIT_SET(_OSC_OSC_USB_BYPASS_VREG(index),
                       _OSC_OSC_USB_BYPASS_POS(index));
			break;
			
		case OSC_PLL_BT:
			_SFR_BIT_SET(_OSC_OSC_BT_BYPASS_VREG(index),
                       _OSC_OSC_BT_BYPASS_POS(index));
			break;
		
		default:
			break;
	}
}


//******************************************************************************
/* Function :  OSC_PLLBypassDisable_PIC32WK

  Summary:
    Implements PIC32WK variant of PLIB_OSC_PLLBypassDisable 

  Description:
    This template implements the PIC32WK variant of the PLIB_OSC_PLLBypassDisable function.
*/

PLIB_TEMPLATE void OSC_PLLBypassDisable_PIC32WK( OSC_MODULE_ID index , OSC_PLL_SELECT pllSel )
{
    switch(pllSel)
	{
		case OSC_PLL_SYSTEM:
			_SFR_BIT_CLEAR(_OSC_OSC_SYSTEM_BYPASS_VREG(index),
                       _OSC_OSC_SYSTEM_BYPASS_POS(index));
			break;
			
		case OSC_PLL_USB:
			_SFR_BIT_CLEAR(_OSC_OSC_USB_BYPASS_VREG(index),
                       _OSC_OSC_USB_BYPASS_POS(index));
			break;
			
		case OSC_PLL_BT:
			_SFR_BIT_CLEAR(_OSC_OSC_BT_BYPASS_VREG(index),
                       _OSC_OSC_BT_BYPASS_POS(index));
			break;
		
		default:
			break;
	}
}


//******************************************************************************
/* Function :  OSC_PLLBypassStatus_PIC32WK

  Summary:
    Implements PIC32WK variant of PLIB_OSC_PLLBypassStatus 

  Description:
    This template implements the PIC32WK variant of the PLIB_OSC_PLLBypassStatus function.
*/

PLIB_TEMPLATE bool OSC_PLLBypassStatus_PIC32WK( OSC_MODULE_ID index , OSC_PLL_SELECT pllSel )
{
    switch(pllSel)
	{
		case OSC_PLL_SYSTEM:
			return _SFR_BIT_READ(_OSC_OSC_SYSTEM_BYPASS_VREG(index),
                       _OSC_OSC_SYSTEM_BYPASS_POS(index));
			
		case OSC_PLL_USB:
			return _SFR_BIT_READ(_OSC_OSC_USB_BYPASS_VREG(index),
                       _OSC_OSC_USB_BYPASS_POS(index));
			
		case OSC_PLL_BT:
			return _SFR_BIT_READ(_OSC_OSC_BT_BYPASS_VREG(index),
                       _OSC_OSC_BT_BYPASS_POS(index));
		
		default:
			break;
	}
}


#endif /*_OSC_PLLBYPASS_PIC32WK_H*/

/******************************************************************************
 End of File
*/

