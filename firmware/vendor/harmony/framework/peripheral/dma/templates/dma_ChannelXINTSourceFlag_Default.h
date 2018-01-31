/*******************************************************************************
  DMA Peripheral Library Template Implementation

  File Name:
    dma_ChannelXINTSourceFlag_Default.h

  Summary:
    DMA PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : ChannelXINTSourceFlag
    and its Variant : Default
    For following APIs :
        PLIB_DMA_ExistsChannelXINTSourceFlag
        PLIB_DMA_ChannelXINTSourceFlagGet
        PLIB_DMA_ChannelXINTSourceFlagSet
        PLIB_DMA_ChannelXINTSourceFlagClear

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

#ifndef _DMA_CHANNELXINTSOURCEFLAG_DEFAULT_H
#define _DMA_CHANNELXINTSOURCEFLAG_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHERIF_VREG(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHTAIF_VREG(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHCCIF_VREG(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHBCIF_VREG(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHDHIF_VREG(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHDDIF_VREG(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHSHIF_VREG(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHSDIF_VREG(index)

  MASKs:
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHERIF_MASK(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHTAIF_MASK(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHCCIF_MASK(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHBCIF_MASK(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHDHIF_MASK(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHDDIF_MASK(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHSHIF_MASK(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHSDIF_MASK(index)

  POSs:
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHERIF_POS(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHTAIF_POS(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHCCIF_POS(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHBCIF_POS(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHDHIF_POS(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHDDIF_POS(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHSHIF_POS(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHSDIF_POS(index)

  LENs:
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHERIF_LEN(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHTAIF_LEN(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHCCIF_LEN(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHBCIF_LEN(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHDHIF_LEN(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHDDIF_LEN(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHSHIF_LEN(index)
    _DMA_CHANNEL_X_INTSOURCEFLAG_CHSDIF_LEN(index)

*/


//******************************************************************************
/* Function :  DMA_ExistsChannelXINTSourceFlag_Default

  Summary:
    Implements Default variant of PLIB_DMA_ExistsChannelXINTSourceFlag

  Description:
    This template implements the Default variant of the PLIB_DMA_ExistsChannelXINTSourceFlag function.
*/

#define PLIB_DMA_ExistsChannelXINTSourceFlag PLIB_DMA_ExistsChannelXINTSourceFlag
PLIB_TEMPLATE bool DMA_ExistsChannelXINTSourceFlag_Default( DMA_MODULE_ID index )
{
    return true;
}


//******************************************************************************
/* Function :  DMA_ChannelXINTSourceFlagGet_Default

  Summary:
    Implements Default variant of PLIB_DMA_ChannelXINTSourceFlagGet

  Description:
    This template implements the Default variant of the PLIB_DMA_ChannelXINTSourceFlagGet function.
*/

PLIB_TEMPLATE bool DMA_ChannelXINTSourceFlagGet_Default( DMA_MODULE_ID index , DMA_CHANNEL dmaChannel , DMA_INT_TYPE dmaINTSource )
{
    SFR_TYPE *pDmaChanXControlReg;
    bool intSrcFlag = 0;

    // The Channel 0 control register is at the base of the Channel control register
    // array.  We can index to the others from it, once we have its address.
    pDmaChanXControlReg = (SFR_TYPE *)_DMA_CHANNEL_X_INTSOURCEFLAG_CHERIF_VREG(index);

	switch(dmaINTSource)
	{
		case DMA_INT_ADDRESS_ERROR:
                        intSrcFlag = _SFR_BIT_READ(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHERIF_POS(index) );
			break;

		case DMA_INT_TRANSFER_ABORT:
                        intSrcFlag = _SFR_BIT_READ(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHTAIF_POS(index) );
			break;

		case DMA_INT_CELL_TRANSFER_COMPLETE:
                        intSrcFlag = _SFR_BIT_READ(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHCCIF_POS(index) );
			break;

		case DMA_INT_BLOCK_TRANSFER_COMPLETE:
                        intSrcFlag = _SFR_BIT_READ(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHBCIF_POS(index) );
			break;

		case DMA_INT_DESTINATION_HALF_FULL:
                        intSrcFlag = _SFR_BIT_READ(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHDHIF_POS(index) );
			break;

		case DMA_INT_DESTINATION_DONE:
                        intSrcFlag = _SFR_BIT_READ(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHDDIF_POS(index) );
			break;

		case DMA_INT_SOURCE_HALF_EMPTY:
                        intSrcFlag = _SFR_BIT_READ(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHSHIF_POS(index) );
			break;

		case DMA_INT_SOURCE_DONE:
                        intSrcFlag = _SFR_BIT_READ(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHSDIF_POS(index) );
			break;
	}
    return intSrcFlag;
}


//******************************************************************************
/* Function :  DMA_ChannelXINTSourceFlagSet_Default

  Summary:
    Implements Default variant of PLIB_DMA_ChannelXINTSourceFlagSet

  Description:
    This template implements the Default variant of the PLIB_DMA_ChannelXINTSourceFlagSet function.
*/

PLIB_TEMPLATE void DMA_ChannelXINTSourceFlagSet_Default( DMA_MODULE_ID index , DMA_CHANNEL dmaChannel , DMA_INT_TYPE dmaINTSource )
{
    SFR_TYPE *pDmaChanXControlReg;

    // The Channel 0 control register is at the base of the Channel control register
    // array.  We can index to the others from it, once we have its address.
    pDmaChanXControlReg = (SFR_TYPE *)_DMA_CHANNEL_X_INTSOURCEFLAG_CHERIF_VREG(index);

	switch(dmaINTSource)
	{
		case DMA_INT_ADDRESS_ERROR:
			_SFR_BIT_SET(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHERIF_POS(index) );
			break;

		case DMA_INT_TRANSFER_ABORT:
			_SFR_BIT_SET(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHTAIF_POS(index) );
			break;

		case DMA_INT_CELL_TRANSFER_COMPLETE:
			_SFR_BIT_SET(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHCCIF_POS(index) );
			break;

		case DMA_INT_BLOCK_TRANSFER_COMPLETE:
			_SFR_BIT_SET(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHBCIF_POS(index) );
			break;

		case DMA_INT_DESTINATION_HALF_FULL:
			_SFR_BIT_SET(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHDHIF_POS(index) );
			break;

		case DMA_INT_DESTINATION_DONE:
			_SFR_BIT_SET(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHDDIF_POS(index) );
			break;

		case DMA_INT_SOURCE_HALF_EMPTY:
			_SFR_BIT_SET(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHSHIF_POS(index) );
			break;

		case DMA_INT_SOURCE_DONE:
			_SFR_BIT_SET(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHSDIF_POS(index) );
			break;
	}
}


//******************************************************************************
/* Function :  DMA_ChannelXINTSourceFlagClear_Default

  Summary:
    Implements Default variant of PLIB_DMA_ChannelXINTSourceFlagClear

  Description:
    This template implements the Default variant of the PLIB_DMA_ChannelXINTSourceFlagClear function.
*/

PLIB_TEMPLATE void DMA_ChannelXINTSourceFlagClear_Default( DMA_MODULE_ID index , DMA_CHANNEL dmaChannel , DMA_INT_TYPE dmaINTSource )
{
    SFR_TYPE *pDmaChanXControlReg;

    // The Channel 0 control register is at the base of the Channel control register
    // array.  We can index to the others from it, once we have its address.
    pDmaChanXControlReg = (SFR_TYPE *)_DMA_CHANNEL_X_INTSOURCEFLAG_CHERIF_VREG(index);

	switch(dmaINTSource)
	{
		case DMA_INT_ADDRESS_ERROR:
			_SFR_BIT_CLEAR(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHERIF_POS(index) );
			break;

		case DMA_INT_TRANSFER_ABORT:
			_SFR_BIT_CLEAR(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHTAIF_POS(index) );
			break;

		case DMA_INT_CELL_TRANSFER_COMPLETE:
			_SFR_BIT_CLEAR(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHCCIF_POS(index) );
			break;

		case DMA_INT_BLOCK_TRANSFER_COMPLETE:
			_SFR_BIT_CLEAR(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHBCIF_POS(index) );
			break;

		case DMA_INT_DESTINATION_HALF_FULL:
			_SFR_BIT_CLEAR(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHDHIF_POS(index) );
			break;

		case DMA_INT_DESTINATION_DONE:
			_SFR_BIT_CLEAR(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHDDIF_POS(index) );
			break;

		case DMA_INT_SOURCE_HALF_EMPTY:
			_SFR_BIT_CLEAR(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHSHIF_POS(index) );
			break;

		case DMA_INT_SOURCE_DONE:
			_SFR_BIT_CLEAR(&pDmaChanXControlReg[48*dmaChannel],
				 _DMA_CHANNEL_X_INTSOURCEFLAG_CHSDIF_POS(index) );
			break;
	}
}

#endif /*_DMA_CHANNELXINTSOURCEFLAG_DEFAULT_H*/

/******************************************************************************
 End of File
*/

