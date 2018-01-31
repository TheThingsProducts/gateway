/*******************************************************************************
  PCACHE Peripheral Library Template Implementation

  File Name:
    pcache_Word_Default.h

  Summary:
    PCACHE PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : Word
    and its Variant : Default
    For following APIs :
        PLIB_PCACHE_WordRead
        PLIB_PCACHE_WordWrite
        PLIB_PCACHE_ExistsWord

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

#ifndef _PCACHE_WORD_DEFAULT_H
#define _PCACHE_WORD_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    _PCACHE_WORD0_VREG(index)
    _PCACHE_WORD1_VREG(index)
    _PCACHE_WORD2_VREG(index)
    _PCACHE_WORD3_VREG(index)

  MASKs: 
    _PCACHE_WORD0_MASK(index)
    _PCACHE_WORD1_MASK(index)
    _PCACHE_WORD2_MASK(index)
    _PCACHE_WORD3_MASK(index)

  POSs: 
    _PCACHE_WORD0_POS(index)
    _PCACHE_WORD1_POS(index)
    _PCACHE_WORD2_POS(index)
    _PCACHE_WORD3_POS(index)

  LENs: 
    _PCACHE_WORD0_LEN(index)
    _PCACHE_WORD1_LEN(index)
    _PCACHE_WORD2_LEN(index)
    _PCACHE_WORD3_LEN(index)

*/


//******************************************************************************
/* Function :  PCACHE_WordRead_Default

  Summary:
    Implements Default variant of PLIB_PCACHE_WordRead 

  Description:
    This template implements the Default variant of the PLIB_PCACHE_WordRead function.
*/

PLIB_TEMPLATE uint32_t PCACHE_WordRead_Default( PCACHE_MODULE_ID index , uint32_t word )
{
    if (word == 0)
        return _SFR_READ(_PCACHE_WORD0_VREG(index));
    else if (word == 1)
        return _SFR_READ(_PCACHE_WORD1_VREG(index));
    else if (word == 2)
        return _SFR_READ(_PCACHE_WORD2_VREG(index));
    else if (word == 3)
        return _SFR_READ(_PCACHE_WORD3_VREG(index));
    else 
        return 0;
}


//******************************************************************************
/* Function :  PCACHE_WordWrite_Default

  Summary:
    Implements Default variant of PLIB_PCACHE_WordWrite 

  Description:
    This template implements the Default variant of the PLIB_PCACHE_WordWrite function.
*/

PLIB_TEMPLATE void PCACHE_WordWrite_Default( PCACHE_MODULE_ID index , uint32_t word , uint32_t data )
{
    switch (word) {
        case 0:
            _SFR_WRITE(_PCACHE_WORD0_VREG(index), data);
            break;
        case 1:
            _SFR_WRITE(_PCACHE_WORD1_VREG(index), data);
            break;
        case 2:
            _SFR_WRITE(_PCACHE_WORD2_VREG(index), data);
            break;
        case 3:
            _SFR_WRITE(_PCACHE_WORD3_VREG(index), data);
            break;
        default:
            break;
    }
}


//******************************************************************************
/* Function :  PCACHE_ExistsWord_Default

  Summary:
    Implements Default variant of PLIB_PCACHE_ExistsWord

  Description:
    This template implements the Default variant of the PLIB_PCACHE_ExistsWord function.
*/

#define PLIB_PCACHE_ExistsWord PLIB_PCACHE_ExistsWord
PLIB_TEMPLATE bool PCACHE_ExistsWord_Default( PCACHE_MODULE_ID index )
{
    return true;
}


#endif /*_PCACHE_WORD_DEFAULT_H*/

/******************************************************************************
 End of File
*/

