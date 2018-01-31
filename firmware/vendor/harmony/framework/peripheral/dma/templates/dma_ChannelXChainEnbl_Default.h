/*******************************************************************************
  DMA Peripheral Library Template Implementation

  File Name:
    dma_ChannelXChainEnbl_Default.h

  Summary:
    DMA PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ChannelXChainEnbl
    and its Variant : Default
    For following APIs :
        PLIB_DMA_ExistsChannelXChainEnbl
        PLIB_DMA_ChannelXChainEnable
        PLIB_DMA_ChannelXChainDisable
        PLIB_DMA_ChannelXChainIsEnabled

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

#ifndef _DMA_CHANNELXCHAINENBL_DEFAULT_H
#define _DMA_CHANNELXCHAINENBL_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _DMA_CHANNEL_X_CHAIN_ENABLE_VREG(index)

  MASKs:
    _DMA_CHANNEL_X_CHAIN_ENABLE_MASK(index)

  POSs:
    _DMA_CHANNEL_X_CHAIN_ENABLE_POS(index)

  LENs:
    _DMA_CHANNEL_X_CHAIN_ENABLE_LEN(index)

*/


//******************************************************************************
/* Function :  DMA_ExistsChannelXChain_Default

  Summary:
    Implements Default variant of PLIB_DMA_ExistsChannelXChainEnbl

  Description:
    This template implements the Default variant of the PLIB_DMA_ExistsChannelXChainEnbl function.
*/

#define PLIB_DMA_ExistsChannelXChainEnbl PLIB_DMA_ExistsChannelXChainEnbl
PLIB_TEMPLATE bool DMA_ExistsChannelXChainEnbl_Default( DMA_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  DMA_ChannelXChainEnable_Default

  Summary:
    Implements Default variant of PLIB_DMA_ChannelXChainEnable

  Description:
    This template implements the Default variant of the PLIB_DMA_ChannelXChainEnable function.
*/

PLIB_TEMPLATE void DMA_ChannelXChainEnable_Default( DMA_MODULE_ID index , DMA_CHANNEL channel )
{
    SFR_TYPE *pDmaChanXControlReg;

    // The Channel 0 control register is at the base of the Channel control register
    // array.  We can index to the others from it, once we have its address.
    pDmaChanXControlReg = (SFR_TYPE *)_DMA_CHANNEL_X_CHAIN_ENABLE_VREG(index);

	_SFR_BIT_SET(&pDmaChanXControlReg[48*channel],
				 _DMA_CHANNEL_X_CHAIN_ENABLE_POS(index) );
}


//******************************************************************************
/* Function :  DMA_ChannelXChainDisable_Default

  Summary:
    Implements Default variant of PLIB_DMA_ChannelXChainDisable

  Description:
    This template implements the Default variant of the PLIB_DMA_ChannelXChainDisable function.
*/

PLIB_TEMPLATE void DMA_ChannelXChainDisable_Default( DMA_MODULE_ID index , DMA_CHANNEL channel )
{
    SFR_TYPE *pDmaChanXControlReg;

    // The Channel 0 control register is at the base of the Channel control register
    // array.  We can index to the others from it, once we have its address.
    pDmaChanXControlReg = (SFR_TYPE *)_DMA_CHANNEL_X_CHAIN_ENABLE_VREG(index);

	_SFR_BIT_CLEAR(&pDmaChanXControlReg[48*channel],
				 _DMA_CHANNEL_X_CHAIN_ENABLE_POS(index) );
}


//******************************************************************************
/* Function :  DMA_ChannelXChainIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_DMA_ChannelXChainIsEnabled

  Description:
    This template implements the Default variant of the PLIB_DMA_ChannelXChainIsEnabled function.
*/

PLIB_TEMPLATE bool DMA_ChannelXChainIsEnabled_Default( DMA_MODULE_ID index , DMA_CHANNEL channel )
{
    SFR_TYPE *pDmaChanXControlReg;

    // The Channel 0 control register is at the base of the Channel control register
    // array.  We can index to the others from it, once we have its address.
    pDmaChanXControlReg = (SFR_TYPE *)_DMA_CHANNEL_X_CHAIN_ENABLE_VREG(index);

	return _SFR_BIT_READ(&pDmaChanXControlReg[48*channel],
				 _DMA_CHANNEL_X_CHAIN_ENABLE_POS(index) );
}


#endif /*_DMA_CHANNELXCHAINENBL_DEFAULT_H*/

/******************************************************************************
 End of File
*/

