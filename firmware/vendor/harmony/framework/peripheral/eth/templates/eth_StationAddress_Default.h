/*******************************************************************************
  ETH Peripheral Library Template Implementation

  File Name:
    eth_StationAddress_Default.h

  Summary:
    ETH PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : StationAddress
    and its Variant : Default
    For following APIs :
        PLIB_ETH_StationAddressGet
        PLIB_ETH_StationAddressSet
        PLIB_ETH_ExistsStationAddress

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

#ifndef _ETH_STATIONADDRESS_DEFAULT_H
#define _ETH_STATIONADDRESS_DEFAULT_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are

  VREGs:
    _ETH_STATION_ADDRESS_OCTET_1_VREG(index)
    _ETH_STATION_ADDRESS_OCTET_2_VREG(index)
    _ETH_STATION_ADDRESS_OCTET_3_VREG(index)
    _ETH_STATION_ADDRESS_OCTET_4_VREG(index)
    _ETH_STATION_ADDRESS_OCTET_5_VREG(index)
    _ETH_STATION_ADDRESS_OCTET_6_VREG(index)

  MASKs:
    _ETH_STATION_ADDRESS_OCTET_1_MASK(index)
    _ETH_STATION_ADDRESS_OCTET_2_MASK(index)
    _ETH_STATION_ADDRESS_OCTET_3_MASK(index)
    _ETH_STATION_ADDRESS_OCTET_4_MASK(index)
    _ETH_STATION_ADDRESS_OCTET_5_MASK(index)
    _ETH_STATION_ADDRESS_OCTET_6_MASK(index)

  POSs:
    _ETH_STATION_ADDRESS_OCTET_1_POS(index)
    _ETH_STATION_ADDRESS_OCTET_2_POS(index)
    _ETH_STATION_ADDRESS_OCTET_3_POS(index)
    _ETH_STATION_ADDRESS_OCTET_4_POS(index)
    _ETH_STATION_ADDRESS_OCTET_5_POS(index)
    _ETH_STATION_ADDRESS_OCTET_6_POS(index)

  LENs:
    _ETH_STATION_ADDRESS_OCTET_1_LEN(index)
    _ETH_STATION_ADDRESS_OCTET_2_LEN(index)
    _ETH_STATION_ADDRESS_OCTET_3_LEN(index)
    _ETH_STATION_ADDRESS_OCTET_4_LEN(index)
    _ETH_STATION_ADDRESS_OCTET_5_LEN(index)
    _ETH_STATION_ADDRESS_OCTET_6_LEN(index)

*/


//******************************************************************************
/* Function :  ETH_StationAddressGet_Default

  Summary:
    Implements Default variant of PLIB_ETH_StationAddressGet

  Description:
    This template implements the Default variant of the PLIB_ETH_StationAddressGet function.
*/

PLIB_TEMPLATE uint8_t ETH_StationAddressGet_Default( ETH_MODULE_ID index , uint8_t which )
{
    uint8_t staAddrByte = 0xFF;

    switch ( which )
    {
        case 1:
            staAddrByte = _SFR_FIELD_READ(_ETH_STATION_ADDRESS_OCTET_1_VREG(index),
                                          _ETH_STATION_ADDRESS_OCTET_1_MASK(index),
                                          _ETH_STATION_ADDRESS_OCTET_1_POS(index) );
            break;

        case 2:
            staAddrByte = _SFR_FIELD_READ(_ETH_STATION_ADDRESS_OCTET_2_VREG(index),
                                          _ETH_STATION_ADDRESS_OCTET_2_MASK(index),
                                          _ETH_STATION_ADDRESS_OCTET_2_POS(index) );
            break;

        case 3:
            staAddrByte = _SFR_FIELD_READ(_ETH_STATION_ADDRESS_OCTET_3_VREG(index),
                                          _ETH_STATION_ADDRESS_OCTET_3_MASK(index),
                                          _ETH_STATION_ADDRESS_OCTET_3_POS(index) );
            break;

        case 4:
            staAddrByte = _SFR_FIELD_READ(_ETH_STATION_ADDRESS_OCTET_4_VREG(index),
                                          _ETH_STATION_ADDRESS_OCTET_4_MASK(index),
                                          _ETH_STATION_ADDRESS_OCTET_4_POS(index) );
            break;

        case 5:
            staAddrByte = _SFR_FIELD_READ(_ETH_STATION_ADDRESS_OCTET_5_VREG(index),
                                          _ETH_STATION_ADDRESS_OCTET_5_MASK(index),
                                          _ETH_STATION_ADDRESS_OCTET_5_POS(index) );
            break;

        case 6:
            staAddrByte = _SFR_FIELD_READ(_ETH_STATION_ADDRESS_OCTET_6_VREG(index),
                                          _ETH_STATION_ADDRESS_OCTET_6_MASK(index),
                                          _ETH_STATION_ADDRESS_OCTET_6_POS(index) );
            break;
    }

    return staAddrByte;

}


//******************************************************************************
/* Function :  ETH_StationAddressSet_Default

  Summary:
    Implements Default variant of PLIB_ETH_StationAddressSet

  Description:
    This template implements the Default variant of the PLIB_ETH_StationAddressSet function.
*/

PLIB_TEMPLATE void ETH_StationAddressSet_Default( ETH_MODULE_ID index , uint8_t which , uint8_t stationAddress )
{
    switch ( which )
    {
        case 1:
            _SFR_FIELD_WRITE(_ETH_STATION_ADDRESS_OCTET_1_VREG(index),
                             _ETH_STATION_ADDRESS_OCTET_1_MASK(index),
                             _ETH_STATION_ADDRESS_OCTET_1_POS(index) ,
                              stationAddress                         );
            break;

        case 2:
            _SFR_FIELD_WRITE(_ETH_STATION_ADDRESS_OCTET_2_VREG(index),
                             _ETH_STATION_ADDRESS_OCTET_2_MASK(index),
                             _ETH_STATION_ADDRESS_OCTET_2_POS(index) ,
                              stationAddress                         );
            break;

        case 3:
            _SFR_FIELD_WRITE(_ETH_STATION_ADDRESS_OCTET_3_VREG(index),
                             _ETH_STATION_ADDRESS_OCTET_3_MASK(index),
                             _ETH_STATION_ADDRESS_OCTET_3_POS(index) ,
                              stationAddress                         );
            break;

        case 4:
            _SFR_FIELD_WRITE(_ETH_STATION_ADDRESS_OCTET_4_VREG(index),
                             _ETH_STATION_ADDRESS_OCTET_4_MASK(index),
                             _ETH_STATION_ADDRESS_OCTET_4_POS(index) ,
                              stationAddress                         );
            break;

        case 5:
            _SFR_FIELD_WRITE(_ETH_STATION_ADDRESS_OCTET_5_VREG(index),
                             _ETH_STATION_ADDRESS_OCTET_5_MASK(index),
                             _ETH_STATION_ADDRESS_OCTET_5_POS(index) ,
                              stationAddress                         );
            break;

        case 6:
            _SFR_FIELD_WRITE(_ETH_STATION_ADDRESS_OCTET_6_VREG(index),
                             _ETH_STATION_ADDRESS_OCTET_6_MASK(index),
                             _ETH_STATION_ADDRESS_OCTET_6_POS(index) ,
                              stationAddress                         );
            break;
    }
}


//******************************************************************************
/* Function :  ETH_ExistsStationAddress_Default

  Summary:
    Implements Default variant of PLIB_ETH_ExistsStationAddress

  Description:
    This template implements the Default variant of the PLIB_ETH_ExistsStationAddress function.
*/

#define PLIB_ETH_ExistsStationAddress PLIB_ETH_ExistsStationAddress
PLIB_TEMPLATE bool ETH_ExistsStationAddress_Default( ETH_MODULE_ID index )
{
    return true;
}


#endif /*_ETH_STATIONADDRESS_DEFAULT_H*/

/******************************************************************************
 End of File
*/

