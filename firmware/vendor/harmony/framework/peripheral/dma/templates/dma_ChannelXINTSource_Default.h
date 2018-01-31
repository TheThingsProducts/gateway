/*******************************************************************************
  DMA Peripheral Library Template Implementation

  File Name:
    dma_ChannelXINTSource_Default.h

  Summary:
    DMA PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ChannelXINTSource
    and its Variant : Default
    For following APIs :
        PLIB_DMA_ExistsChannelXINTSource
        PLIB_DMA_ChannelXINTSourceEnable
        PLIB_DMA_ChannelXINTSourceDisable
        PLIB_DMA_ChannelXINTSourceIsEnabled

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

#ifndef _DMA_CHANNELXINTSOURCE_DEFAULT_H
#define _DMA_CHANNELXINTSOURCE_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _DMA_CHANNEL_X_INTSOURCE_CHERIE_VREG(index)
    _DMA_CHANNEL_X_INTSOURCE_CHTAIE_VREG(index)
    _DMA_CHANNEL_X_INTSOURCE_CHCCIE_VREG(index)
    _DMA_CHANNEL_X_INTSOURCE_CHBCIE_VREG(index)
    _DMA_CHANNEL_X_INTSOURCE_CHDHIE_VREG(index)
    _DMA_CHANNEL_X_INTSOURCE_CHDDIE_VREG(index)
    _DMA_CHANNEL_X_INTSOURCE_CHSHIE_VREG(index)
    _DMA_CHANNEL_X_INTSOURCE_CHSDIE_VREG(index)

  MASKs:
    _DMA_CHANNEL_X_INTSOURCE_CHERIE_MASK(index)
    _DMA_CHANNEL_X_INTSOURCE_CHTAIE_MASK(index)
    _DMA_CHANNEL_X_INTSOURCE_CHCCIE_MASK(index)
    _DMA_CHANNEL_X_INTSOURCE_CHBCIE_MASK(index)
    _DMA_CHANNEL_X_INTSOURCE_CHDHIE_MASK(index)
    _DMA_CHANNEL_X_INTSOURCE_CHDDIE_MASK(index)
    _DMA_CHANNEL_X_INTSOURCE_CHSHIE_MASK(index)
    _DMA_CHANNEL_X_INTSOURCE_CHSDIE_MASK(index)

  POSs:
    _DMA_CHANNEL_X_INTSOURCE_CHERIE_POS(index)
    _DMA_CHANNEL_X_INTSOURCE_CHTAIE_POS(index)
    _DMA_CHANNEL_X_INTSOURCE_CHCCIE_POS(index)
    _DMA_CHANNEL_X_INTSOURCE_CHBCIE_POS(index)
    _DMA_CHANNEL_X_INTSOURCE_CHDHIE_POS(index)
    _DMA_CHANNEL_X_INTSOURCE_CHDDIE_POS(index)
    _DMA_CHANNEL_X_INTSOURCE_CHSHIE_POS(index)
    _DMA_CHANNEL_X_INTSOURCE_CHSDIE_POS(index)

  LENs:
    _DMA_CHANNEL_X_INTSOURCE_CHERIE_LEN(index)
    _DMA_CHANNEL_X_INTSOURCE_CHTAIE_LEN(index)
    _DMA_CHANNEL_X_INTSOURCE_CHCCIE_LEN(index)
    _DMA_CHANNEL_X_INTSOURCE_CHBCIE_LEN(index)
    _DMA_CHANNEL_X_INTSOURCE_CHDHIE_LEN(index)
    _DMA_CHANNEL_X_INTSOURCE_CHDDIE_LEN(index)
    _DMA_CHANNEL_X_INTSOURCE_CHSHIE_LEN(index)
    _DMA_CHANNEL_X_INTSOURCE_CHSDIE_LEN(index)

*/


//******************************************************************************
/* Function :  DMA_ExistsChannelXINTSource_Default

  Summary:
    Implements Default variant of PLIB_DMA_ExistsChannelXINTSource

  Description:
    This template implements the Default variant of the PLIB_DMA_ExistsChannelXINTSource function.
*/

#define PLIB_DMA_ExistsChannelXINTSource PLIB_DMA_ExistsChannelXINTSource
PLIB_TEMPLATE bool DMA_ExistsChannelXINTSource_Default( DMA_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  DMA_ChannelXINTSourceEnable_Default

  Summary:
    Implements Default variant of PLIB_DMA_ChannelXINTSourceEnable

  Description:
    This template implements the Default variant of the PLIB_DMA_ChannelXINTSourceEnable function.
*/

PLIB_TEMPLATE void DMA_ChannelXINTSourceEnable_Default( DMA_MODULE_ID index , DMA_CHANNEL dmaChannel , DMA_INT_TYPE dmaINTSource )
{
    SFR_TYPE *pDmaChanXControlReg;

    // The Channel 0 control register is at the base of the Channel control register
    // array.  We can index to the others from it, once we have its address.
    pDmaChanXControlReg = (SFR_TYPE *)_DMA_CHANNEL_X_INTSOURCE_CHERIE_VREG(index);

	switch(dmaINTSource)
	{
		case DMA_INT_ADDRESS_ERROR:
			_SFR_BIT_SET(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHERIE_POS(index) );
			break;

		case DMA_INT_TRANSFER_ABORT:
			_SFR_BIT_SET(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHTAIE_POS(index) );
			break;

		case DMA_INT_CELL_TRANSFER_COMPLETE:
			_SFR_BIT_SET(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHCCIE_POS(index) );
			break;

		case DMA_INT_BLOCK_TRANSFER_COMPLETE:
			_SFR_BIT_SET(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHBCIE_POS(index) );
			break;

		case DMA_INT_DESTINATION_HALF_FULL:
			_SFR_BIT_SET(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHDHIE_POS(index) );
			break;

		case DMA_INT_DESTINATION_DONE:
			_SFR_BIT_SET(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHDDIE_POS(index) );
			break;

		case DMA_INT_SOURCE_HALF_EMPTY:
			_SFR_BIT_SET(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHSHIE_POS(index) );
			break;

		case DMA_INT_SOURCE_DONE:
			_SFR_BIT_SET(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHSDIE_POS(index) );
			break;
	}
}


//******************************************************************************
/* Function :  DMA_ChannelXINTSourceDisable_Default

  Summary:
    Implements Default variant of PLIB_DMA_ChannelXINTSourceDisable

  Description:
    This template implements the Default variant of the PLIB_DMA_ChannelXINTSourceDisable function.
*/

PLIB_TEMPLATE void DMA_ChannelXINTSourceDisable_Default( DMA_MODULE_ID index , DMA_CHANNEL dmaChannel , DMA_INT_TYPE dmaINTSource )
{
    SFR_TYPE *pDmaChanXControlReg;

    // The Channel 0 control register is at the base of the Channel control register
    // array.  We can index to the others from it, once we have its address.
    pDmaChanXControlReg = (SFR_TYPE *)_DMA_CHANNEL_X_INTSOURCE_CHERIE_VREG(index);

	switch(dmaINTSource)
	{
		case DMA_INT_ADDRESS_ERROR:
			_SFR_BIT_CLEAR(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHERIE_POS(index) );
			break;

		case DMA_INT_TRANSFER_ABORT:
			_SFR_BIT_CLEAR(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHTAIE_POS(index) );
			break;

		case DMA_INT_CELL_TRANSFER_COMPLETE:
			_SFR_BIT_CLEAR(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHCCIE_POS(index) );
			break;

		case DMA_INT_BLOCK_TRANSFER_COMPLETE:
			_SFR_BIT_CLEAR(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHBCIE_POS(index) );
			break;

		case DMA_INT_DESTINATION_HALF_FULL:
			_SFR_BIT_CLEAR(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHDHIE_POS(index) );
			break;

		case DMA_INT_DESTINATION_DONE:
			_SFR_BIT_CLEAR(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHDDIE_POS(index) );
			break;

		case DMA_INT_SOURCE_HALF_EMPTY:
			_SFR_BIT_CLEAR(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHSHIE_POS(index) );
			break;

		case DMA_INT_SOURCE_DONE:
			_SFR_BIT_CLEAR(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHSDIE_POS(index) );
			break;
	}
}


//******************************************************************************
/* Function :  DMA_ChannelXINTSourceIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_DMA_ChannelXINTSourceIsEnabled

  Description:
    This template implements the Default variant of the PLIB_DMA_ChannelXINTSourceIsEnabled function.
*/

PLIB_TEMPLATE bool DMA_ChannelXINTSourceIsEnabled_Default( DMA_MODULE_ID index , DMA_CHANNEL dmaChannel , DMA_INT_TYPE dmaINTSource )
{
    SFR_TYPE *pDmaChanXControlReg;

    // The Channel 0 control register is at the base of the Channel control register
    // array.  We can index to the others from it, once we have its address.
    pDmaChanXControlReg = (SFR_TYPE *)_DMA_CHANNEL_X_INTSOURCE_CHERIE_VREG(index);

	switch(dmaINTSource)
	{
		case DMA_INT_ADDRESS_ERROR:
			return _SFR_BIT_READ(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHERIE_POS(index) );
			break;

		case DMA_INT_TRANSFER_ABORT:
			return _SFR_BIT_READ(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHTAIE_POS(index) );
			break;

		case DMA_INT_CELL_TRANSFER_COMPLETE:
			return _SFR_BIT_READ(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHCCIE_POS(index) );
			break;

		case DMA_INT_BLOCK_TRANSFER_COMPLETE:
			return _SFR_BIT_READ(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHBCIE_POS(index) );
			break;

		case DMA_INT_DESTINATION_HALF_FULL:
			return _SFR_BIT_READ(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHDHIE_POS(index) );
			break;

		case DMA_INT_DESTINATION_DONE:
			return _SFR_BIT_READ(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHDDIE_POS(index) );
			break;

		case DMA_INT_SOURCE_HALF_EMPTY:
			return _SFR_BIT_READ(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHSHIE_POS(index) );
			break;

		case DMA_INT_SOURCE_DONE:
			return _SFR_BIT_READ(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCE_CHSDIE_POS(index) );
			break;
	}
}


#endif /*_DMA_CHANNELXINTSOURCE_DEFAULT_H*/

/******************************************************************************
 End of File
*/

