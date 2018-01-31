/*******************************************************************************
  ETH Peripheral Library Template Implementation

  File Name:
    eth_FramesTransmittedOK_Default.h

  Summary:
    ETH PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : FramesTransmittedOK
    and its Variant : Default
    For following APIs :
        PLIB_ETH_FramesTxdOkCountClear
        PLIB_ETH_FramesTxdOkCountGet
        PLIB_ETH_ExistsFramesTransmittedOK

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

#ifndef _ETH_FRAMESTRANSMITTEDOK_DEFAULT_H
#define _ETH_FRAMESTRANSMITTEDOK_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _ETH_FRAME_TX_OK_COUNT_VREG(index)

  MASKs:
    _ETH_FRAME_TX_OK_COUNT_MASK(index)

  POSs:
    _ETH_FRAME_TX_OK_COUNT_POS(index)

  LENs:
    _ETH_FRAME_TX_OK_COUNT_LEN(index)

*/


//******************************************************************************
/* Function :  ETH_FramesTxdOkCountClear_Default

  Summary:
    Implements Default variant of PLIB_ETH_FramesTxdOkCountClear

  Description:
    This template implements the Default variant of the PLIB_ETH_FramesTxdOkCountClear function.
*/

PLIB_TEMPLATE void ETH_FramesTxdOkCountClear_Default( ETH_MODULE_ID index )
{
    _SFR_WRITE(_ETH_FRAME_TX_OK_COUNT_VREG(index),0x0000);
}


//******************************************************************************
/* Function :  ETH_FramesTxdOkCountGet_Default

  Summary:
    Implements Default variant of PLIB_ETH_FramesTxdOkCountGet

  Description:
    This template implements the Default variant of the PLIB_ETH_FramesTxdOkCountGet function.
*/

PLIB_TEMPLATE uint16_t ETH_FramesTxdOkCountGet_Default( ETH_MODULE_ID index )
{
    return (uint16_t)_SFR_READ(_ETH_FRAME_TX_OK_COUNT_VREG(index));
}


//******************************************************************************
/* Function :  ETH_ExistsFramesTransmittedOK_Default

  Summary:
    Implements Default variant of PLIB_ETH_ExistsFramesTransmittedOK

  Description:
    This template implements the Default variant of the PLIB_ETH_ExistsFramesTransmittedOK function.
*/

#define PLIB_ETH_ExistsFramesTransmittedOK PLIB_ETH_ExistsFramesTransmittedOK
PLIB_TEMPLATE bool ETH_ExistsFramesTransmittedOK_Default( ETH_MODULE_ID index )
{
    return true;
}


#endif /*_ETH_FRAMESTRANSMITTEDOK_DEFAULT_H*/

/******************************************************************************
 End of File
*/

