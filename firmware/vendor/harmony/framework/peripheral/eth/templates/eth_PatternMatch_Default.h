/*******************************************************************************
  ETH Peripheral Library Template Implementation

  File Name:
    eth_PatternMatch_Default.h

  Summary:
    ETH PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : PatternMatch
    and its Variant : Default
    For following APIs :
        PLIB_ETH_PatternMatchSet
        PLIB_ETH_PatternMatchGet
        PLIB_ETH_PatternMatchChecksumSet
        PLIB_ETH_PatternMatchChecksumGet
        PLIB_ETH_PatternMatchOffsetSet
        PLIB_ETH_PatternMatchOffsetGet
        PLIB_ETH_PatternMatchModeSet
        PLIB_ETH_PatternMatchModeGet
        PLIB_ETH_ExistsPatternMatch

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

#ifndef _ETH_PATTERNMATCH_DEFAULT_H
#define _ETH_PATTERNMATCH_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _ETH_RX_ENABLE_VREG(index)
    _ETH_PATTERN_MATCH_MODE_VREG(index)
    _ETH_PATTERN_MATCH_LOWER_VREG(index)
    _ETH_PATTERN_MATCH_UPPER_VREG(index)
    _ETH_PATTERN_MATCH_CHECKSUM_VREG(index)
    _ETH_PATTERN_MATCH_OFFSET_VREG(index)

  MASKs:
    _ETH_RX_ENABLE_MASK(index)
    _ETH_PATTERN_MATCH_MODE_MASK(index)
    _ETH_PATTERN_MATCH_LOWER_MASK(index)
    _ETH_PATTERN_MATCH_UPPER_MASK(index)
    _ETH_PATTERN_MATCH_CHECKSUM_MASK(index)
    _ETH_PATTERN_MATCH_OFFSET_MASK(index)

  POSs:
    _ETH_RX_ENABLE_POS(index)
    _ETH_PATTERN_MATCH_MODE_POS(index)
    _ETH_PATTERN_MATCH_LOWER_POS(index)
    _ETH_PATTERN_MATCH_UPPER_POS(index)
    _ETH_PATTERN_MATCH_CHECKSUM_POS(index)
    _ETH_PATTERN_MATCH_OFFSET_POS(index)

  LENs:
    _ETH_RX_ENABLE_LEN(index)
    _ETH_PATTERN_MATCH_MODE_LEN(index)
    _ETH_PATTERN_MATCH_LOWER_LEN(index)
    _ETH_PATTERN_MATCH_UPPER_LEN(index)
    _ETH_PATTERN_MATCH_CHECKSUM_LEN(index)
    _ETH_PATTERN_MATCH_OFFSET_LEN(index)

*/

//******************************************************************************
/* Function :  ETH_PatternMatchSet_Default

  Summary:
    Implements Default variant of PLIB_ETH_PatternMatchSet

  Description:
    This template implements the Default variant of the PLIB_ETH_PatternMatchSet function.
*/

PLIB_TEMPLATE void ETH_PatternMatchSet_Default( ETH_MODULE_ID index , uint64_t patternMatchMaskValue )
{
    PLIB_ASSERT(
        !(bool)_SFR_BIT_READ(_ETH_RX_ENABLE_VREG(index),_ETH_RX_ENABLE_POS(index)) ||
        _SFR_FIELD_READ(_ETH_PATTERN_MATCH_MODE_VREG(index),
                        _ETH_PATTERN_MATCH_MODE_MASK(index),
                        _ETH_PATTERN_MATCH_MODE_POS(index) ) == 0,
                "Receive must be disabled or pattern match mode disabled!");
    _SFR_WRITE(_ETH_PATTERN_MATCH_LOWER_VREG(index),(uint32_t)patternMatchMaskValue);
    _SFR_WRITE(_ETH_PATTERN_MATCH_UPPER_VREG(index),(uint32_t)(patternMatchMaskValue>>32));
}


//******************************************************************************
/* Function :  ETH_PatternMatchGet_Default

  Summary:
    Implements Default variant of PLIB_ETH_PatternMatchGet

  Description:
    This template implements the Default variant of the PLIB_ETH_PatternMatchGet function.
*/

PLIB_TEMPLATE uint64_t ETH_PatternMatchGet_Default( ETH_MODULE_ID index )
{
    return (  (uint64_t)_SFR_READ(_ETH_PATTERN_MATCH_LOWER_VREG(index)) + (( (uint64_t)_SFR_READ(_ETH_PATTERN_MATCH_UPPER_VREG(index)) )<<32)  );
}


//******************************************************************************
/* Function :  ETH_PatternMatchChecksumSet_Default

  Summary:
    Implements Default variant of PLIB_ETH_PatternMatchChecksumSet

  Description:
    This template implements the Default variant of the PLIB_ETH_PatternMatchChecksumSet function.
*/

PLIB_TEMPLATE void ETH_PatternMatchChecksumSet_Default( ETH_MODULE_ID index , uint16_t PatternMatchChecksumValue )
{
    PLIB_ASSERT(
        !(bool)_SFR_BIT_READ(_ETH_RX_ENABLE_VREG(index),_ETH_RX_ENABLE_POS(index)) ||
        _SFR_FIELD_READ(_ETH_PATTERN_MATCH_MODE_VREG(index),
                        _ETH_PATTERN_MATCH_MODE_MASK(index),
                        _ETH_PATTERN_MATCH_MODE_POS(index) ) == 0,
                "Receive must be disabled or pattern match mode disabled!");
    _SFR_WRITE(_ETH_PATTERN_MATCH_CHECKSUM_VREG(index),PatternMatchChecksumValue);
}


//******************************************************************************
/* Function :  ETH_PatternMatchChecksumGet_Default

  Summary:
    Implements Default variant of PLIB_ETH_PatternMatchChecksumGet

  Description:
    This template implements the Default variant of the PLIB_ETH_PatternMatchChecksumGet function.
*/

PLIB_TEMPLATE uint16_t ETH_PatternMatchChecksumGet_Default( ETH_MODULE_ID index )
{
    return (uint16_t)_SFR_READ(_ETH_PATTERN_MATCH_CHECKSUM_VREG(index));
}


//******************************************************************************
/* Function :  ETH_PatternMatchOffsetSet_Default

  Summary:
    Implements Default variant of PLIB_ETH_PatternMatchOffsetSet

  Description:
    This template implements the Default variant of the PLIB_ETH_PatternMatchOffsetSet function.
*/

PLIB_TEMPLATE void ETH_PatternMatchOffsetSet_Default( ETH_MODULE_ID index , uint16_t PatternMatchOffsetValue )
{
    PLIB_ASSERT(
        !(bool)_SFR_BIT_READ(_ETH_RX_ENABLE_VREG(index),_ETH_RX_ENABLE_POS(index)) ||
        _SFR_FIELD_READ(_ETH_PATTERN_MATCH_MODE_VREG(index),
                        _ETH_PATTERN_MATCH_MODE_MASK(index),
                        _ETH_PATTERN_MATCH_MODE_POS(index) ) == 0,
                "Receive must be disabled or pattern match mode disabled!");
    _SFR_WRITE(_ETH_PATTERN_MATCH_OFFSET_VREG(index),PatternMatchOffsetValue);
}


//******************************************************************************
/* Function :  ETH_PatternMatchOffsetGet_Default

  Summary:
    Implements Default variant of PLIB_ETH_PatternMatchOffsetGet

  Description:
    This template implements the Default variant of the PLIB_ETH_PatternMatchOffsetGet function.
*/

PLIB_TEMPLATE uint16_t ETH_PatternMatchOffsetGet_Default( ETH_MODULE_ID index )
{
    return (uint16_t)_SFR_READ(_ETH_PATTERN_MATCH_OFFSET_VREG(index));
}


//******************************************************************************
/* Function :  ETH_PatternMatchModeSet_Default

  Summary:
    Implements Default variant of PLIB_ETH_PatternMatchModeSet

  Description:
    This template implements the Default variant of the PLIB_ETH_PatternMatchModeSet function.
*/

PLIB_TEMPLATE void ETH_PatternMatchModeSet_Default( ETH_MODULE_ID index , ETH_PATTERN_MATCH_MODE modeSel )
{
    PLIB_ASSERT(!(bool)_SFR_BIT_READ(_ETH_RX_ENABLE_VREG(index),_ETH_RX_ENABLE_POS(index)),
                "Receive must be disabled");
    _SFR_FIELD_WRITE(_ETH_PATTERN_MATCH_MODE_VREG(index),
                     _ETH_PATTERN_MATCH_MODE_MASK(index),
                     _ETH_PATTERN_MATCH_MODE_POS(index) ,
                     (uint8_t)modeSel                   );
}


//******************************************************************************
/* Function :  ETH_PatternMatchModeGet_Default

  Summary:
    Implements Default variant of PLIB_ETH_PatternMatchModeGet

  Description:
    This template implements the Default variant of the PLIB_ETH_PatternMatchModeGet function.
*/

PLIB_TEMPLATE ETH_PATTERN_MATCH_MODE ETH_PatternMatchModeGet_Default( ETH_MODULE_ID index )
{
     return (ETH_PATTERN_MATCH_MODE)
                     _SFR_FIELD_READ(_ETH_PATTERN_MATCH_MODE_VREG(index),
                                     _ETH_PATTERN_MATCH_MODE_MASK(index),
                                     _ETH_PATTERN_MATCH_MODE_POS(index) );
}


//******************************************************************************
/* Function :  ETH_ExistsPatternMatch_Default

  Summary:
    Implements Default variant of PLIB_ETH_ExistsPatternMatch

  Description:
    This template implements the Default variant of the PLIB_ETH_ExistsPatternMatch function.
*/

#define PLIB_ETH_ExistsPatternMatch PLIB_ETH_ExistsPatternMatch
PLIB_TEMPLATE bool ETH_ExistsPatternMatch_Default( ETH_MODULE_ID index )
{
    return true;
}


#endif /*_ETH_PATTERNMATCH_DEFAULT_H*/

/******************************************************************************
 End of File
*/

