/*******************************************************************************
  DMA Peripheral Library Template Implementation

  File Name:
    dma_Busy_Default.h

  Summary:
    DMA PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : Busy
    and its Variant : Default
    For following APIs :
        PLIB_DMA_ExistsBusy
        PLIB_DMA_BusyActiveSet
        PLIB_DMA_BusyActiveReset
        PLIB_DMA_IsBusy

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

#ifndef _DMA_BUSY_DEFAULT_H
#define _DMA_BUSY_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _DMA_BUSY_VREG(index)

  MASKs:
    _DMA_BUSY_MASK(index)

  POSs:
    _DMA_BUSY_POS(index)

  LENs:
    _DMA_BUSY_LEN(index)

*/


//******************************************************************************
/* Function :  DMA_ExistsBusy_Default

  Summary:
    Implements Default variant of PLIB_DMA_ExistsBusy

  Description:
    This template implements the Default variant of the PLIB_DMA_ExistsBusy function.
*/

#define PLIB_DMA_ExistsBusy PLIB_DMA_ExistsBusy
PLIB_TEMPLATE bool DMA_ExistsBusy_Default( DMA_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  DMA_BusyActiveSet_Default

  Summary:
    Implements Default variant of PLIB_DMA_BusyActiveSet

  Description:
    This template implements the Default variant of the PLIB_DMA_BusyActiveSet function.
*/

PLIB_TEMPLATE void DMA_BusyActiveSet_Default( DMA_MODULE_ID index )
{
	_SFR_BIT_SET( _DMA_BUSY_VREG( index ), _DMA_BUSY_POS( index ) );
}


//******************************************************************************
/* Function :  DMA_BusyActiveReset_Default

  Summary:
    Implements Default variant of PLIB_DMA_BusyActiveReset

  Description:
    This template implements the Default variant of the PLIB_DMA_BusyActiveReset function.
*/

PLIB_TEMPLATE void DMA_BusyActiveReset_Default( DMA_MODULE_ID index )
{
	_SFR_BIT_CLEAR( _DMA_BUSY_VREG( index ), _DMA_BUSY_POS( index ) );
}


//******************************************************************************
/* Function :  DMA_IsBusy_Default

  Summary:
    Implements Default variant of PLIB_DMA_IsBusy

  Description:
    This template implements the Default variant of the PLIB_DMA_IsBusy function.
*/

PLIB_TEMPLATE bool DMA_IsBusy_Default( DMA_MODULE_ID index )
{
    return _SFR_BIT_READ(_DMA_BUSY_VREG( index ), _DMA_BUSY_POS( index ));
}


#endif /*_DMA_BUSY_DEFAULT_H*/

/******************************************************************************
 End of File
*/

