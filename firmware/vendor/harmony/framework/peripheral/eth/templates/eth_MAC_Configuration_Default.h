/*******************************************************************************
  ETH Peripheral Library Template Implementation

  File Name:
    eth_MAC_Configuration_Default.h

  Summary:
    ETH PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : MAC_Configuration
    and its Variant : Default
    For following APIs :
        PLIB_ETH_LoopbackEnable
        PLIB_ETH_LoopbackDisable
        PLIB_ETH_LoopbackIsEnabled
        PLIB_ETH_TxPauseEnable
        PLIB_ETH_TxPauseDisable
        PLIB_ETH_TxPauseIsEnabled
        PLIB_ETH_RxPauseEnable
        PLIB_ETH_RxPauseDisable
        PLIB_ETH_RxPauseIsEnabled
        PLIB_ETH_PassAllEnable
        PLIB_ETH_PassAllDisable
        PLIB_ETH_PassAllIsEnabled
        PLIB_ETH_ReceiveEnable
        PLIB_ETH_ReceiveDisable
        PLIB_ETH_ReceiveIsEnabled
        PLIB_ETH_ExcessDeferEnable
        PLIB_ETH_ExcessDeferDisable
        PLIB_ETH_ExcessDeferIsEnabled
        PLIB_ETH_BackPresNoBackoffEnable
        PLIB_ETH_BackPresNoBackoffDisable
        PLIB_ETH_BackPresNoBackoffIsEnabled
        PLIB_ETH_NoBackoffEnable
        PLIB_ETH_NoBackoffDisable
        PLIB_ETH_NoBackoffIsEnabled
        PLIB_ETH_LongPreambleEnable
        PLIB_ETH_LongPreambleDisable
        PLIB_ETH_LongPreambleIsEnabled
        PLIB_ETH_PurePreambleEnable
        PLIB_ETH_PurePreambleDisable
        PLIB_ETH_PurePreambleIsEnabled
        PLIB_ETH_AutoDetectPadGet
        PLIB_ETH_AutoDetectPadSet
        PLIB_ETH_AutoDetectPadClear
        PLIB_ETH_CRCEnable
        PLIB_ETH_CRCDisable
        PLIB_ETH_CRCIsEnabled
        PLIB_ETH_DelayedCRCEnable
        PLIB_ETH_DelayedCRCDisable
        PLIB_ETH_DelayedCRCIsEnabled
        PLIB_ETH_HugeFrameEnable
        PLIB_ETH_HugeFrameDisable
        PLIB_ETH_HugeFrameIsEnabled
        PLIB_ETH_FrameLengthCheckEnable
        PLIB_ETH_FrameLengthCheckDisable
        PLIB_ETH_FrameLengthCheckIsEnabled
        PLIB_ETH_FullDuplexEnable
        PLIB_ETH_FullDuplexDisable
        PLIB_ETH_FullDuplexIsEnabled
        PLIB_ETH_ExistsMAC_Configuration

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

#ifndef _ETH_MAC_CONFIGURATION_DEFAULT_H
#define _ETH_MAC_CONFIGURATION_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _ETH_MAC_LOOPBACK_VREG(index)
    _ETH_TX_FLOW_CONTROL_VREG(index)
    _ETH_RX_FLOW_CONTROL_VREG(index)
    _ETH_PASS_ALL_RX_FRAMES_VREG(index)
    _ETH_MAC_RX_ENABLE_VREG(index)
    _ETH_EXCESS_DEFER_VREG(index)
    _ETH_BACKPRESSURE_NOBACKOFF_VREG(index)
    _ETH_NO_BACKOFF_VREG(index)
    _ETH_LONG_PREAMBLE_VREG(index)
    _ETH_PURE_PREAMBLE_VREG(index)
    _ETH_AUTO_DETECT_PAD_VREG(index)
    _ETH_VLAN_PAD_VREG(index)
    _ETH_PAD_CRC_ENABLE_VREG(index)
    _ETH_CRC_ENABLE_VREG(index)
    _ETH_DELAYED_CRC_VREG(index)
    _ETH_HUGE_FRAME_VREG(index)
    _ETH_LENGTH_CHECK_VREG(index)
    _ETH_FULL_DUPLEX_VREG(index)

  MASKs:
    _ETH_MAC_LOOPBACK_MASK(index)
    _ETH_TX_FLOW_CONTROL_MASK(index)
    _ETH_RX_FLOW_CONTROL_MASK(index)
    _ETH_PASS_ALL_RX_FRAMES_MASK(index)
    _ETH_MAC_RX_ENABLE_MASK(index)
    _ETH_EXCESS_DEFER_MASK(index)
    _ETH_BACKPRESSURE_NOBACKOFF_MASK(index)
    _ETH_NO_BACKOFF_MASK(index)
    _ETH_LONG_PREAMBLE_MASK(index)
    _ETH_PURE_PREAMBLE_MASK(index)
    _ETH_AUTO_DETECT_PAD_MASK(index)
    _ETH_VLAN_PAD_MASK(index)
    _ETH_PAD_CRC_ENABLE_MASK(index)
    _ETH_CRC_ENABLE_MASK(index)
    _ETH_DELAYED_CRC_MASK(index)
    _ETH_HUGE_FRAME_MASK(index)
    _ETH_LENGTH_CHECK_MASK(index)
    _ETH_FULL_DUPLEX_MASK(index)

  POSs:
    _ETH_MAC_LOOPBACK_POS(index)
    _ETH_TX_FLOW_CONTROL_POS(index)
    _ETH_RX_FLOW_CONTROL_POS(index)
    _ETH_PASS_ALL_RX_FRAMES_POS(index)
    _ETH_MAC_RX_ENABLE_POS(index)
    _ETH_EXCESS_DEFER_POS(index)
    _ETH_BACKPRESSURE_NOBACKOFF_POS(index)
    _ETH_NO_BACKOFF_POS(index)
    _ETH_LONG_PREAMBLE_POS(index)
    _ETH_PURE_PREAMBLE_POS(index)
    _ETH_AUTO_DETECT_PAD_POS(index)
    _ETH_VLAN_PAD_POS(index)
    _ETH_PAD_CRC_ENABLE_POS(index)
    _ETH_CRC_ENABLE_POS(index)
    _ETH_DELAYED_CRC_POS(index)
    _ETH_HUGE_FRAME_POS(index)
    _ETH_LENGTH_CHECK_POS(index)
    _ETH_FULL_DUPLEX_POS(index)

  LENs:
    _ETH_MAC_LOOPBACK_LEN(index)
    _ETH_TX_FLOW_CONTROL_LEN(index)
    _ETH_RX_FLOW_CONTROL_LEN(index)
    _ETH_PASS_ALL_RX_FRAMES_LEN(index)
    _ETH_MAC_RX_ENABLE_LEN(index)
    _ETH_EXCESS_DEFER_LEN(index)
    _ETH_BACKPRESSURE_NOBACKOFF_LEN(index)
    _ETH_NO_BACKOFF_LEN(index)
    _ETH_LONG_PREAMBLE_LEN(index)
    _ETH_PURE_PREAMBLE_LEN(index)
    _ETH_AUTO_DETECT_PAD_LEN(index)
    _ETH_VLAN_PAD_LEN(index)
    _ETH_PAD_CRC_ENABLE_LEN(index)
    _ETH_CRC_ENABLE_LEN(index)
    _ETH_DELAYED_CRC_LEN(index)
    _ETH_HUGE_FRAME_LEN(index)
    _ETH_LENGTH_CHECK_LEN(index)
    _ETH_FULL_DUPLEX_LEN(index)

*/


//******************************************************************************
/* Function :  ETH_LoopbackEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_LoopbackEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_LoopbackEnable function.
*/

PLIB_TEMPLATE void ETH_LoopbackEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_MAC_LOOPBACK_VREG(index),_ETH_MAC_LOOPBACK_POS(index));
}


//******************************************************************************
/* Function :  ETH_LoopbackDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_LoopbackDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_LoopbackDisable function.
*/

PLIB_TEMPLATE void ETH_LoopbackDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_MAC_LOOPBACK_VREG(index),_ETH_MAC_LOOPBACK_POS(index));
}


//******************************************************************************
/* Function :  ETH_LoopbackIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_LoopbackIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_LoopbackIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_LoopbackIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_MAC_LOOPBACK_VREG(index),_ETH_MAC_LOOPBACK_POS(index));
}


//******************************************************************************
/* Function :  ETH_TxPauseEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_TxPauseEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_TxPauseEnable function.
*/

PLIB_TEMPLATE void ETH_TxPauseEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_TX_FLOW_CONTROL_VREG(index),_ETH_TX_FLOW_CONTROL_POS(index));
}


//******************************************************************************
/* Function :  ETH_TxPauseDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_TxPauseDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_TxPauseDisable function.
*/

PLIB_TEMPLATE void ETH_TxPauseDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_TX_FLOW_CONTROL_VREG(index),_ETH_TX_FLOW_CONTROL_POS(index));
}


//******************************************************************************
/* Function :  ETH_TxPauseIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_TxPauseIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_TxPauseIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_TxPauseIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_TX_FLOW_CONTROL_VREG(index),_ETH_TX_FLOW_CONTROL_POS(index));
}


//******************************************************************************
/* Function :  ETH_RxPauseEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_RxPauseEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_RxPauseEnable function.
*/

PLIB_TEMPLATE void ETH_RxPauseEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_RX_FLOW_CONTROL_VREG(index),_ETH_RX_FLOW_CONTROL_POS(index));
}


//******************************************************************************
/* Function :  ETH_RxPauseDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_RxPauseDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_RxPauseDisable function.
*/

PLIB_TEMPLATE void ETH_RxPauseDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_RX_FLOW_CONTROL_VREG(index),_ETH_RX_FLOW_CONTROL_POS(index));
}


//******************************************************************************
/* Function :  ETH_RxPauseIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_RxPauseIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_RxPauseIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_RxPauseIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_RX_FLOW_CONTROL_VREG(index),_ETH_RX_FLOW_CONTROL_POS(index));
}


//******************************************************************************
/* Function :  ETH_PassAllEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_PassAllEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_PassAllEnable function.
*/

PLIB_TEMPLATE void ETH_PassAllEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_PASS_ALL_RX_FRAMES_VREG(index),_ETH_PASS_ALL_RX_FRAMES_POS(index));
}


//******************************************************************************
/* Function :  ETH_PassAllDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_PassAllDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_PassAllDisable function.
*/

PLIB_TEMPLATE void ETH_PassAllDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_PASS_ALL_RX_FRAMES_VREG(index),_ETH_PASS_ALL_RX_FRAMES_POS(index));
}


//******************************************************************************
/* Function :  ETH_PassAllIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_PassAllIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_PassAllIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_PassAllIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_PASS_ALL_RX_FRAMES_VREG(index),_ETH_PASS_ALL_RX_FRAMES_POS(index));
}


//******************************************************************************
/* Function :  ETH_ReceiveEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_ReceiveEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_ReceiveEnable function.
*/

PLIB_TEMPLATE void ETH_ReceiveEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_MAC_RX_ENABLE_VREG(index),_ETH_MAC_RX_ENABLE_POS(index));
}


//******************************************************************************
/* Function :  ETH_ReceiveDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_ReceiveDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_ReceiveDisable function.
*/

PLIB_TEMPLATE void ETH_ReceiveDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_MAC_RX_ENABLE_VREG(index),_ETH_MAC_RX_ENABLE_POS(index));
}


//******************************************************************************
/* Function :  ETH_ReceiveIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_ReceiveIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_ReceiveIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_ReceiveIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_MAC_RX_ENABLE_VREG(index),_ETH_MAC_RX_ENABLE_POS(index));
}


//******************************************************************************
/* Function :  ETH_ExcessDeferEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_ExcessDeferEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_ExcessDeferEnable function.
*/

PLIB_TEMPLATE void ETH_ExcessDeferEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_EXCESS_DEFER_VREG(index),_ETH_EXCESS_DEFER_POS(index));
}


//******************************************************************************
/* Function :  ETH_ExcessDeferDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_ExcessDeferDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_ExcessDeferDisable function.
*/

PLIB_TEMPLATE void ETH_ExcessDeferDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_EXCESS_DEFER_VREG(index),_ETH_EXCESS_DEFER_POS(index));
}


//******************************************************************************
/* Function :  ETH_ExcessDeferIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_ExcessDeferIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_ExcessDeferIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_ExcessDeferIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_EXCESS_DEFER_VREG(index),_ETH_EXCESS_DEFER_POS(index));
}


//******************************************************************************
/* Function :  ETH_BackPresNoBackoffEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_BackPresNoBackoffEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_BackPresNoBackoffEnable function.
*/

PLIB_TEMPLATE void ETH_BackPresNoBackoffEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_BACKPRESSURE_NOBACKOFF_VREG(index),_ETH_BACKPRESSURE_NOBACKOFF_POS(index));
}


//******************************************************************************
/* Function :  ETH_BackPresNoBackoffDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_BackPresNoBackoffDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_BackPresNoBackoffDisable function.
*/

PLIB_TEMPLATE void ETH_BackPresNoBackoffDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_BACKPRESSURE_NOBACKOFF_VREG(index),_ETH_BACKPRESSURE_NOBACKOFF_POS(index));
}


//******************************************************************************
/* Function :  ETH_BackPresNoBackoffIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_BackPresNoBackoffIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_BackPresNoBackoffIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_BackPresNoBackoffIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_BACKPRESSURE_NOBACKOFF_VREG(index),_ETH_BACKPRESSURE_NOBACKOFF_POS(index));
}


//******************************************************************************
/* Function :  ETH_NoBackoffEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_NoBackoffEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_NoBackoffEnable function.
*/

PLIB_TEMPLATE void ETH_NoBackoffEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_NO_BACKOFF_VREG(index),_ETH_NO_BACKOFF_POS(index));
}


//******************************************************************************
/* Function :  ETH_NoBackoffDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_NoBackoffDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_NoBackoffDisable function.
*/

PLIB_TEMPLATE void ETH_NoBackoffDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_NO_BACKOFF_VREG(index),_ETH_NO_BACKOFF_POS(index));
}


//******************************************************************************
/* Function :  ETH_NoBackoffIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_NoBackoffIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_NoBackoffIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_NoBackoffIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_NO_BACKOFF_VREG(index),_ETH_NO_BACKOFF_POS(index));
}


//******************************************************************************
/* Function :  ETH_LongPreambleEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_LongPreambleEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_LongPreambleEnable function.
*/

PLIB_TEMPLATE void ETH_LongPreambleEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_LONG_PREAMBLE_VREG(index),_ETH_LONG_PREAMBLE_POS(index));
}


//******************************************************************************
/* Function :  ETH_LongPreambleDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_LongPreambleDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_LongPreambleDisable function.
*/

PLIB_TEMPLATE void ETH_LongPreambleDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_LONG_PREAMBLE_VREG(index),_ETH_LONG_PREAMBLE_POS(index));
}


//******************************************************************************
/* Function :  ETH_LongPreambleIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_LongPreambleIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_LongPreambleIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_LongPreambleIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_LONG_PREAMBLE_VREG(index),_ETH_LONG_PREAMBLE_POS(index));
}


//******************************************************************************
/* Function :  ETH_PurePreambleEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_PurePreambleEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_PurePreambleEnable function.
*/

PLIB_TEMPLATE void ETH_PurePreambleEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_PURE_PREAMBLE_VREG(index),_ETH_PURE_PREAMBLE_POS(index));
}


//******************************************************************************
/* Function :  ETH_PurePreambleDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_PurePreambleDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_PurePreambleDisable function.
*/

PLIB_TEMPLATE void ETH_PurePreambleDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_PURE_PREAMBLE_VREG(index),_ETH_PURE_PREAMBLE_POS(index));
}


//******************************************************************************
/* Function :  ETH_PurePreambleIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_PurePreambleIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_PurePreambleIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_PurePreambleIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_PURE_PREAMBLE_VREG(index),_ETH_PURE_PREAMBLE_POS(index));
}


//******************************************************************************
/* Function :  ETH_AutoDetectPadGet_Default

  Summary:
    Implements Default variant of PLIB_ETH_AutoDetectPadGet

  Description:
    This template implements the Default variant of the PLIB_ETH_AutoDetectPadGet function.
*/

PLIB_TEMPLATE ETH_AUTOPAD_OPTION ETH_AutoDetectPadGet_Default( ETH_MODULE_ID index )
{
    /* These all point to the same SFR:
        _ETH_AUTO_DETECT_PAD_VREG(index)
        _ETH_VLAN_PAD_VREG(index)
        _ETH_PAD_CRC_ENABLE_VREG(index)
     */
    return (ETH_AUTOPAD_OPTION)_SFR_FIELD_READ(_ETH_PAD_CRC_ENABLE_VREG(index),
                           _ETH_AUTO_DETECT_PAD_MASK(index) | _ETH_VLAN_PAD_MASK(index) | _ETH_PAD_CRC_ENABLE_MASK(index),
                           _ETH_PAD_CRC_ENABLE_POS(index) );
}


//******************************************************************************
/* Function :  ETH_AutoDetectPadSet_Default

  Summary:
    Implements Default variant of PLIB_ETH_AutoDetectPadSet

  Description:
    This template implements the Default variant of the PLIB_ETH_AutoDetectPadSet function.
*/

PLIB_TEMPLATE void ETH_AutoDetectPadSet_Default( ETH_MODULE_ID index , ETH_AUTOPAD_OPTION option )
{
    _SFR_FIELD_SET(_ETH_PAD_CRC_ENABLE_VREG(index),
                   _ETH_AUTO_DETECT_PAD_MASK(index) | _ETH_VLAN_PAD_MASK(index) | _ETH_PAD_CRC_ENABLE_MASK(index),
                   _ETH_PAD_CRC_ENABLE_POS(index),
                    option                       );
}


//******************************************************************************
/* Function :  ETH_AutoDetectPadClear_Default

  Summary:
    Implements Default variant of PLIB_ETH_AutoDetectPadClear

  Description:
    This template implements the Default variant of the PLIB_ETH_AutoDetectPadClear function.
*/

PLIB_TEMPLATE void ETH_AutoDetectPadClear_Default( ETH_MODULE_ID index , ETH_AUTOPAD_OPTION option )
{
    _SFR_FIELD_CLEAR(_ETH_PAD_CRC_ENABLE_VREG(index),
                     _ETH_AUTO_DETECT_PAD_MASK(index) | _ETH_VLAN_PAD_MASK(index) | _ETH_PAD_CRC_ENABLE_MASK(index),
                     _ETH_PAD_CRC_ENABLE_POS(index) ,
                      option                        );
}


//******************************************************************************
/* Function :  ETH_CRCEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_CRCEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_CRCEnable function.
*/

PLIB_TEMPLATE void ETH_CRCEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_CRC_ENABLE_VREG(index),_ETH_CRC_ENABLE_POS(index));
}


//******************************************************************************
/* Function :  ETH_CRCDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_CRCDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_CRCDisable function.
*/

PLIB_TEMPLATE void ETH_CRCDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_CRC_ENABLE_VREG(index),_ETH_CRC_ENABLE_POS(index));
}


//******************************************************************************
/* Function :  ETH_CRCIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_CRCIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_CRCIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_CRCIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_CRC_ENABLE_VREG(index),_ETH_CRC_ENABLE_POS(index));
}


//******************************************************************************
/* Function :  ETH_DelayedCRCEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_DelayedCRCEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_DelayedCRCEnable function.
*/

PLIB_TEMPLATE void ETH_DelayedCRCEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_DELAYED_CRC_VREG(index),_ETH_DELAYED_CRC_POS(index));
}


//******************************************************************************
/* Function :  ETH_DelayedCRCDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_DelayedCRCDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_DelayedCRCDisable function.
*/

PLIB_TEMPLATE void ETH_DelayedCRCDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_DELAYED_CRC_VREG(index),_ETH_DELAYED_CRC_POS(index));
}


//******************************************************************************
/* Function :  ETH_DelayedCRCIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_DelayedCRCIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_DelayedCRCIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_DelayedCRCIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_DELAYED_CRC_VREG(index),_ETH_DELAYED_CRC_POS(index));
}


//******************************************************************************
/* Function :  ETH_HugeFrameEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_HugeFrameEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_HugeFrameEnable function.
*/

PLIB_TEMPLATE void ETH_HugeFrameEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_HUGE_FRAME_VREG(index),_ETH_HUGE_FRAME_POS(index));
}


//******************************************************************************
/* Function :  ETH_HugeFrameDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_HugeFrameDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_HugeFrameDisable function.
*/

PLIB_TEMPLATE void ETH_HugeFrameDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_HUGE_FRAME_VREG(index),_ETH_HUGE_FRAME_POS(index));
}


//******************************************************************************
/* Function :  ETH_HugeFrameIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_HugeFrameIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_HugeFrameIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_HugeFrameIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_HUGE_FRAME_VREG(index),_ETH_HUGE_FRAME_POS(index));
}


//******************************************************************************
/* Function :  ETH_FrameLengthCheckEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_FrameLengthCheckEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_FrameLengthCheckEnable function.
*/

PLIB_TEMPLATE void ETH_FrameLengthCheckEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_LENGTH_CHECK_VREG(index),_ETH_LENGTH_CHECK_POS(index));
}


//******************************************************************************
/* Function :  ETH_FrameLengthCheckDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_FrameLengthCheckDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_FrameLengthCheckDisable function.
*/

PLIB_TEMPLATE void ETH_FrameLengthCheckDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_LENGTH_CHECK_VREG(index),_ETH_LENGTH_CHECK_POS(index));
}


//******************************************************************************
/* Function :  ETH_FrameLengthCheckIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_FrameLengthCheckIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_FrameLengthCheckIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_FrameLengthCheckIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_LENGTH_CHECK_VREG(index),_ETH_LENGTH_CHECK_POS(index));
}


//******************************************************************************
/* Function :  ETH_FullDuplexEnable_Default

  Summary:
    Implements Default variant of PLIB_ETH_FullDuplexEnable

  Description:
    This template implements the Default variant of the PLIB_ETH_FullDuplexEnable function.
*/

PLIB_TEMPLATE void ETH_FullDuplexEnable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_SET(_ETH_FULL_DUPLEX_VREG(index),_ETH_FULL_DUPLEX_POS(index));
}


//******************************************************************************
/* Function :  ETH_FullDuplexDisable_Default

  Summary:
    Implements Default variant of PLIB_ETH_FullDuplexDisable

  Description:
    This template implements the Default variant of the PLIB_ETH_FullDuplexDisable function.
*/

PLIB_TEMPLATE void ETH_FullDuplexDisable_Default( ETH_MODULE_ID index )
{
    _SFR_BIT_CLEAR(_ETH_FULL_DUPLEX_VREG(index),_ETH_FULL_DUPLEX_POS(index));
}


//******************************************************************************
/* Function :  ETH_FullDuplexIsEnabled_Default

  Summary:
    Implements Default variant of PLIB_ETH_FullDuplexIsEnabled

  Description:
    This template implements the Default variant of the PLIB_ETH_FullDuplexIsEnabled function.
*/

PLIB_TEMPLATE bool ETH_FullDuplexIsEnabled_Default( ETH_MODULE_ID index )
{
    return (bool)_SFR_BIT_READ(_ETH_FULL_DUPLEX_VREG(index),_ETH_FULL_DUPLEX_POS(index));
}


//******************************************************************************
/* Function :  ETH_ExistsMAC_Configuration_Default

  Summary:
    Implements Default variant of PLIB_ETH_ExistsMAC_Configuration

  Description:
    This template implements the Default variant of the PLIB_ETH_ExistsMAC_Configuration function.
*/

#define PLIB_ETH_ExistsMAC_Configuration PLIB_ETH_ExistsMAC_Configuration
PLIB_TEMPLATE bool ETH_ExistsMAC_Configuration_Default( ETH_MODULE_ID index )
{
    return true;
}


#endif /*_ETH_MAC_CONFIGURATION_DEFAULT_H*/

/******************************************************************************
 End of File
*/

