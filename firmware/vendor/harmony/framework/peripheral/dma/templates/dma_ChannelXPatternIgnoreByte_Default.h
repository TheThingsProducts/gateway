/*******************************************************************************
  DMA Peripheral Library Template Implementation

  File Name:
    dma_ChannelXPatternIgnoreByte_Default.h

  Summary:
    DMA PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ChannelXPatternIgnoreByte
    and its Variant : Default
    For following APIs :
        PLIB_DMA_ExistsChannelXPatternIgnoreByte
        PLIB_DMA_ChannelXPatternIgnoreByteEnable
        PLIB_DMA_ChannelXPatternIgnoreByteIsEnabled
        PLIB_DMA_ChannelXPatternIgnoreByteDisable

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

#ifndef _DMA_CHANNELXPATTERNIGNOREBYTE_DEFAULT_H
#define _DMA_CHANNELXPATTERNIGNOREBYTE_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _DMA_CHANNEL_X_PATTERN_IGNORE_ENABLE_VREG(index)

  MASKs:
    _DMA_CHANNEL_X_PATTERN_IGNORE_ENABLE_MASK(index)

  POSs:
    _DMA_CHANNEL_X_PATTERN_IGNORE_ENABLE_POS(index)

  LENs:
    _DMA_CHANNEL_X_PATTERN_IGNORE_ENABLE_LEN(index)

*/


//******************************************************************************
/* Function :  DMA_ExistsChannelXPatternIgnoreByte_Default

  Summary:
    Implements Default variant of PLIB_DMA_ExistsChannelXPatternIgnoreByte

  Description:
    This template implements the Default variant of the PLIB_DMA_ExistsChannelXPatternIgnoreByte function.
*/

#define PLIB_DMA_ExistsChannelXPatternIgnoreByte PLIB_DMA_ExistsChannelXPatternIgnoreByte
PLIB_TEMPLATE bool DMA_ExistsChannelXPatternIgnoreByte_Default( DMA_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  DMA_ChannelXPatternIgnoreByteEnable_Default

  Summary:
    Implements Default variant of PLIB_DMA_ChannelXPatternIgnoreByteEnable

  Description:
    This template implements the Default variant of the PLIB_DMA_ChannelXPatternIgnoreByteEnable function.
*/

PLIB_TEMPLATE void DMA_ChannelXPatternIgnoreByteEnable_Default( DMA_MODULE_ID index , DMA_CHANNEL channel )
{
	SFR_TYPE *pDmaChanXControlReg;

    // The Channel 0 control register is at the base of the Channel control register
    // array.  We can index to the others from it, once we have its address.
    pDmaChanXControlReg = (SFR_TYPE *)_DMA_CHANNEL_X_PATTERN_IGNORE_ENABLE_VREG(index);

	_SFR_BIT_SET(&pDmaChanXControlReg[48*channel],
				 _DMA_CHANNEL_X_PATTERN_IGNORE_ENABLE_POS(index) );
}


//******************************************************************************
/* Function :  DMA_ChannelXPatternIgnoreByteIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_DMA_ChannelXPatternIgnoreByteIsEnabled

  Description:
    This template implements the Default variant of the PLIB_DMA_ChannelXPatternIgnoreByteIsEnabled function.
*/

PLIB_TEMPLATE bool DMA_ChannelXPatternIgnoreByteIsEnabled_Default( DMA_MODULE_ID index , DMA_CHANNEL channel )
{
	SFR_TYPE *pDmaChanXControlReg;

    // The Channel 0 control register is at the base of the Channel control register
    // array.  We can index to the others from it, once we have its address.
    pDmaChanXControlReg = (SFR_TYPE *)_DMA_CHANNEL_X_PATTERN_IGNORE_ENABLE_VREG(index);

	return _SFR_BIT_READ(&pDmaChanXControlReg[48*channel],
				 _DMA_CHANNEL_X_PATTERN_IGNORE_ENABLE_POS(index) );
}


//******************************************************************************
/* Function :  DMA_ChannelXPatternIgnoreByteDisable_Default

  Summary:
    Implements Default variant of PLIB_DMA_ChannelXPatternIgnoreByteDisable

  Description:
    This template implements the Default variant of the PLIB_DMA_ChannelXPatternIgnoreByteDisable function.
*/

PLIB_TEMPLATE void DMA_ChannelXPatternIgnoreByteDisable_Default( DMA_MODULE_ID index , DMA_CHANNEL channel )
{
	SFR_TYPE *pDmaChanXControlReg;

    // The Channel 0 control register is at the base of the Channel control register
    // array.  We can index to the others from it, once we have its address.
    pDmaChanXControlReg = (SFR_TYPE *)_DMA_CHANNEL_X_PATTERN_IGNORE_ENABLE_VREG(index);

	_SFR_BIT_CLEAR(&pDmaChanXControlReg[48*channel],
				 _DMA_CHANNEL_X_PATTERN_IGNORE_ENABLE_POS(index) );
}


#endif /*_DMA_CHANNELXPATTERNIGNOREBYTE_DEFAULT_H*/

/******************************************************************************
 End of File
*/

