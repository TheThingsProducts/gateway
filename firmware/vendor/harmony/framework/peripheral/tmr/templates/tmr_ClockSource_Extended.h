/*******************************************************************************
  TMR Peripheral Library Template Implementation

  File Name:
    tmr_ClockSource_Extended.h

  Summary:
    TMR PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ClockSource
    and its Variant : Extended
    For following APIs :
        PLIB_TMR_ClockSourceSelect
        PLIB_TMR_ExistsClockSource

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

#ifndef _TMR_CLOCKSOURCE_EXTENDED_H
#define _TMR_CLOCKSOURCE_EXTENDED_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    _TMR_EXTERNAL_CLOCK_SOURCE_VREG(index)
    _TMR_CLOCK_SOURCE_VREG(index)

  MASKs: 
    _TMR_EXTERNAL_CLOCK_SOURCE_MASK(index)
    _TMR_CLOCK_SOURCE_MASK(index)

  POSs: 
    _TMR_EXTERNAL_CLOCK_SOURCE_POS(index)
    _TMR_CLOCK_SOURCE_POS(index)

  LENs: 
    _TMR_EXTERNAL_CLOCK_SOURCE_LEN(index)
    _TMR_CLOCK_SOURCE_LEN(index)

*/


//******************************************************************************
/* Function :  TMR_ClockSourceSelect_Extended

  Summary:
    Implements Extended variant of PLIB_TMR_ClockSourceSelect 

  Description:
    This template implements the Extended variant of the PLIB_TMR_ClockSourceSelect function.
*/

PLIB_TEMPLATE void TMR_ClockSourceSelect_Extended( TMR_MODULE_ID index , TMR_CLOCK_SOURCE source )
{

    if (source == 0x04)
    {
        /* 0x04 means source is peripheral clock, check the processor header
        file where this function is being called to understand the values */
        
        _SFR_BIT_CLEAR( _TMR_EXTERNAL_CLOCK_SOURCE_VREG( index ),
                                _TMR_CLOCK_SOURCE_POS( index ) );
    }   
    else
    {
        _SFR_BIT_SET( _TMR_EXTERNAL_CLOCK_SOURCE_VREG( index ),
                                _TMR_CLOCK_SOURCE_POS( index ) );
        _SFR_FIELD_WRITE( _TMR_EXTERNAL_CLOCK_SOURCE_VREG(index),
                            _TMR_EXTERNAL_CLOCK_SOURCE_MASK(index),
							_TMR_EXTERNAL_CLOCK_SOURCE_POS(index),
									source );
    }

}


//******************************************************************************
/* Function :  TMR_ExistsClockSource_Extended

  Summary:
    Implements Extended variant of PLIB_TMR_ExistsClockSource

  Description:
    This template implements the Extended variant of the PLIB_TMR_ExistsClockSource function.
*/

#define PLIB_TMR_ExistsClockSource PLIB_TMR_ExistsClockSource
PLIB_TEMPLATE bool TMR_ExistsClockSource_Extended( TMR_MODULE_ID index )
{
    return true;
}


#endif /*_TMR_CLOCKSOURCE_EXTENDED_H*/

/******************************************************************************
 End of File
*/

