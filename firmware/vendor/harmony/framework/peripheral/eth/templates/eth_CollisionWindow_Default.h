/*******************************************************************************
  ETH Peripheral Library Template Implementation

  File Name:
    eth_CollisionWindow_Default.h

  Summary:
    ETH PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : CollisionWindow
    and its Variant : Default
    For following APIs :
        PLIB_ETH_CollisionWindowGet
        PLIB_ETH_CollisionWindowSet
        PLIB_ETH_ExistsCollisionWindow

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

#ifndef _ETH_COLLISIONWINDOW_DEFAULT_H
#define _ETH_COLLISIONWINDOW_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _ETH_COL_WINDOW_VREG(index)

  MASKs:
    _ETH_COL_WINDOW_MASK(index)

  POSs:
    _ETH_COL_WINDOW_POS(index)

  LENs:
    _ETH_COL_WINDOW_LEN(index)

*/


//******************************************************************************
/* Function :  ETH_CollisionWindowGet_Default

  Summary:
    Implements Default variant of PLIB_ETH_CollisionWindowGet

  Description:
    This template implements the Default variant of the PLIB_ETH_CollisionWindowGet function.
*/

PLIB_TEMPLATE uint8_t ETH_CollisionWindowGet_Default( ETH_MODULE_ID index )
{
    return (uint8_t)_SFR_FIELD_READ(_ETH_COL_WINDOW_VREG(index),
                                    _ETH_COL_WINDOW_MASK(index),
                                    _ETH_COL_WINDOW_POS(index) );
}


//******************************************************************************
/* Function :  ETH_CollisionWindowSet_Default

  Summary:
    Implements Default variant of PLIB_ETH_CollisionWindowSet

  Description:
    This template implements the Default variant of the PLIB_ETH_CollisionWindowSet function.
*/

PLIB_TEMPLATE void ETH_CollisionWindowSet_Default( ETH_MODULE_ID index , uint8_t collisionWindowValue )
{
    _SFR_FIELD_WRITE(_ETH_COL_WINDOW_VREG(index),
                     _ETH_COL_WINDOW_MASK(index),
                     _ETH_COL_WINDOW_POS(index) ,
                     collisionWindowValue       );
}


//******************************************************************************
/* Function :  ETH_ExistsCollisionWindow_Default

  Summary:
    Implements Default variant of PLIB_ETH_ExistsCollisionWindow

  Description:
    This template implements the Default variant of the PLIB_ETH_ExistsCollisionWindow function.
*/

#define PLIB_ETH_ExistsCollisionWindow PLIB_ETH_ExistsCollisionWindow
PLIB_TEMPLATE bool ETH_ExistsCollisionWindow_Default( ETH_MODULE_ID index )
{
    return true;
}


#endif /*_ETH_COLLISIONWINDOW_DEFAULT_H*/

/******************************************************************************
 End of File
*/

