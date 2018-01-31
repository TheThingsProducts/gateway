/*******************************************************************************
  DMA Peripheral Library Template Implementation

  File Name:
    dma_ChannelX_Default.h

  Summary:
    DMA PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ChannelX
    and its Variant : Default
    For following APIs :
        PLIB_DMA_ExistsChannelX
        PLIB_DMA_ChannelXEnable
        PLIB_DMA_ChannelXIsEnabled
        PLIB_DMA_ChannelXDisable

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

#ifndef _DMA_CHANNELX_DEFAULT_H
#define _DMA_CHANNELX_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _DMA_CHANNEL_X_VREG(index)

  MASKs:
    _DMA_CHANNEL_X_MASK(index)

  POSs:
    _DMA_CHANNEL_X_POS(index)

  LENs:
    _DMA_CHANNEL_X_LEN(index)

*/


//******************************************************************************
/* Function :  DMA_ExistsChannelX_Default

  Summary:
    Implements Default variant of PLIB_DMA_ExistsChannelX

  Description:
    This template implements the Default variant of the PLIB_DMA_ExistsChannelX function.
*/

#define PLIB_DMA_ExistsChannelX PLIB_DMA_ExistsChannelX
PLIB_TEMPLATE bool DMA_ExistsChannelX_Default( DMA_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  DMA_ChannelXEnable_Default

  Summary:
    Implements Default variant of PLIB_DMA_ChannelXEnable

  Description:
    This template implements the Default variant of the PLIB_DMA_ChannelXEnable function.
*/

PLIB_TEMPLATE void DMA_ChannelXEnable_Default( DMA_MODULE_ID index , DMA_CHANNEL channel )
{
    SFR_TYPE *pDmaChanXControlReg;

    // The Channel 0 control register is at the base of the Channel control register
    // array.  We can index to the others from it, once we have its address.
    pDmaChanXControlReg = (SFR_TYPE *)_DMA_CHANNEL_X_VREG(index);

	_SFR_BIT_SET(&pDmaChanXControlReg[48*channel],
				 _DMA_CHANNEL_X_POS(index) );
}


//******************************************************************************
/* Function :  DMA_ChannelXIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_DMA_ChannelXIsEnabled

  Description:
    This template implements the Default variant of the PLIB_DMA_ChannelXIsEnabled function.
*/

PLIB_TEMPLATE bool DMA_ChannelXIsEnabled_Default( DMA_MODULE_ID index , DMA_CHANNEL channel )
{
    SFR_TYPE *pDmaChanXControlReg;

    // The Channel 0 control register is at the base of the Channel control register
    // array.  We can index to the others from it, once we have its address.
    pDmaChanXControlReg = (SFR_TYPE *)_DMA_CHANNEL_X_VREG(index);

	return _SFR_BIT_READ(&pDmaChanXControlReg[48*channel],
				 _DMA_CHANNEL_X_POS(index) );
}


//******************************************************************************
/* Function :  DMA_ChannelXDisable_Default

  Summary:
    Implements Default variant of PLIB_DMA_ChannelXDisable

  Description:
    This template implements the Default variant of the PLIB_DMA_ChannelXDisable function.
*/

PLIB_TEMPLATE void DMA_ChannelXDisable_Default( DMA_MODULE_ID index , DMA_CHANNEL channel )
{
    SFR_TYPE *pDmaChanXControlReg;

    // The Channel 0 control register is at the base of the Channel control register
    // array.  We can index to the others from it, once we have its address.
    pDmaChanXControlReg = (SFR_TYPE *)_DMA_CHANNEL_X_VREG(index);

	_SFR_BIT_CLEAR(&pDmaChanXControlReg[48*channel],
				 _DMA_CHANNEL_X_POS(index) );
}


#endif /*_DMA_CHANNELX_DEFAULT_H*/

/******************************************************************************
 End of File
*/

