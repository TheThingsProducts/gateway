/*******************************************************************************
  Ethernet Library Interface Definition

  Summary:
    This file contains the Application Program Interface (API) definition  for
    the Ethernet peripheral library.

  Description:
    This library provides a low-level abstraction of the Ethernet module
    on Microchip PIC32MX family microcontrollers with a convenient C language
    interface.  It can be used to simplify low-level access to the module
    without the necessity of interacting directly with the module's registers,
    thus hiding differences from one microcontroller variant to another.
*******************************************************************************/
//DOM-IGNORE-BEGIN
/*******************************************************************************
File Name:      plib_eth_lib.c
Processor:      PIC32MX
Compiler:       Microchip MPLAB C32 v1.00 or higher

Copyright © 2013 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND,
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

#include <string.h>
#include "peripheral/eth/plib_eth.h"
#if defined(_ETHRXFC_CRCERREN_MASK)
#include "peripheral/eth/src/plib_eth_lib.h"




/****************************************************************************
 * Function:        PLIB_ETH_MACSetAddress
 *****************************************************************************/
void PLIB_ETH_MACSetAddress(ETH_MODULE_ID index,unsigned char bAddress[6])
{
    PLIB_ETH_StationAddressSet(index,1,bAddress[0]);
    PLIB_ETH_StationAddressSet(index,2,bAddress[1]);
    PLIB_ETH_StationAddressSet(index,3,bAddress[2]);
    PLIB_ETH_StationAddressSet(index,4,bAddress[3]);
    PLIB_ETH_StationAddressSet(index,5,bAddress[4]);
    PLIB_ETH_StationAddressSet(index,6,bAddress[5]);
}


/****************************************************************************
 * Function:        PLIB_ETH_MACGetAddress
 *****************************************************************************/
void PLIB_ETH_MACGetAddress(ETH_MODULE_ID index,unsigned char bAddress[6])
{
    *bAddress++ = PLIB_ETH_StationAddressGet(index,1);
    *bAddress++ = PLIB_ETH_StationAddressGet(index,2);
    *bAddress++ = PLIB_ETH_StationAddressGet(index,3);
    *bAddress++ = PLIB_ETH_StationAddressGet(index,4);
    *bAddress++ = PLIB_ETH_StationAddressGet(index,5);
    *bAddress   = PLIB_ETH_StationAddressGet(index,6);
}


/****************************************************************************
 * Function:        PLIB_ETH_MACSetMaxFrame
 *****************************************************************************/
void PLIB_ETH_MACSetMaxFrame(ETH_MODULE_ID index,unsigned short maxFrmSz)
{
    PLIB_ETH_MaxFrameLengthSet(index,maxFrmSz);
}


/****************************************************************************
 * Function:        PLIB_ETH_RxFiltersSet
 *****************************************************************************/
void PLIB_ETH_RxFiltersSet(ETH_MODULE_ID index,ETH_RX_FILTERS rxFilters)
{
    PLIB_ETH_ReceiveFilterEnable(index,rxFilters);
}


/****************************************************************************
 * Function:        PLIB_ETH_RxFiltersClr
 *****************************************************************************/
void PLIB_ETH_RxFiltersClr(ETH_MODULE_ID index,ETH_RX_FILTERS rxFilters)
{
    PLIB_ETH_ReceiveFilterDisable(index,rxFilters);
}


/****************************************************************************
 * Function:        PLIB_ETH_RxFiltersWrite
 *****************************************************************************/
void PLIB_ETH_RxFiltersWrite(ETH_MODULE_ID index,ETH_RX_FILTERS rxFilters)
{
    PLIB_ETH_ReceiveFilterEnable(index,rxFilters);
    PLIB_ETH_ReceiveFilterDisable(index,~rxFilters);
}


/****************************************************************************
 * Function:        PLIB_ETH_RxFiltersHTSet
 *****************************************************************************/
void PLIB_ETH_RxFiltersHTSet(ETH_MODULE_ID index,uint64_t htable)
{
    PLIB_ETH_HashTableSet(index,htable);
}


/****************************************************************************
 * Function:        PLIB_ETH_RxFiltersPMSet
 *****************************************************************************/
void PLIB_ETH_RxFiltersPMSet(ETH_MODULE_ID index,
                                   ETH_PMATCH_MODE mode,
                                   uint64_t matchMask,
                                   unsigned int matchOffs,
                                   unsigned int matchChecksum)
{
    PLIB_ETH_PatternMatchModeSet(index,ETH_PATTERN_MATCH_DISABLED);

    PLIB_ETH_PatternMatchSet(index,matchMask);
    PLIB_ETH_PatternMatchOffsetSet(index,matchOffs);
    PLIB_ETH_PatternMatchChecksumSet(index,matchChecksum);

    if(mode&ETH_FILT_PMATCH_INVERT)
    {
        PLIB_ETH_ReceiveFilterEnable(index,ETH_PATTERN_MATCH_INVERSION);
    }
    else
    {
        PLIB_ETH_ReceiveFilterDisable(index,ETH_PATTERN_MATCH_INVERSION);
    }

 // Enable Pattern Match mode
    PLIB_ETH_PatternMatchModeSet(index,mode&(~ETH_FILT_PMATCH_INVERT));
}


/****************************************************************************
 * Function:        PLIB_ETH_RxFiltersPMClr
 *****************************************************************************/
void PLIB_ETH_RxFiltersPMClr(ETH_MODULE_ID index)
{
    PLIB_ETH_PatternMatchModeSet(index,(uint64_t)0);
}


/*********************************************************************
 * Function:        void PLIB_ETH_EventsEnableSet(PLIB_ETH_EVENTS eEvents)
 ********************************************************************/
void PLIB_ETH_EventsEnableSet(ETH_MODULE_ID index,PLIB_ETH_EVENTS eEvents)
{
    PLIB_ETH_InterruptSourceEnable(index,eEvents);
}


/*********************************************************************
 * Function:        void PLIB_ETH_EventsEnableClr(PLIB_ETH_EVENTS eEvents)
 ********************************************************************/
void PLIB_ETH_EventsEnableClr(ETH_MODULE_ID index,PLIB_ETH_EVENTS eEvents)
{
    PLIB_ETH_InterruptSourceDisable(index,eEvents);
}


/*********************************************************************
 * Function:        void PLIB_ETH_EventsEnableWrite(PLIB_ETH_EVENTS eEvents)
 ********************************************************************/
void PLIB_ETH_EventsEnableWrite(ETH_MODULE_ID index,PLIB_ETH_EVENTS eEvents)
{
    PLIB_ETH_InterruptSourceEnable(index,eEvents);
    PLIB_ETH_InterruptSourceDisable(index,~eEvents);
}


/*********************************************************************
 * Function:        PLIB_ETH_EVENTS PLIB_ETH_EventsEnableGet(void)
 ********************************************************************/
PLIB_ETH_EVENTS PLIB_ETH_EventsEnableGet(ETH_MODULE_ID index)
{
    return PLIB_ETH_InterruptSourcesGet(index);
}


/*******************************************************************************
  Function:
    void PLIB_ETH_EventsClr ( ETH_MODULE_ID index, PLIB_ETH_EVENTS eEvents )
 */
void PLIB_ETH_EventsClr(ETH_MODULE_ID index,PLIB_ETH_EVENTS eEvents)
{
    PLIB_ETH_InterruptClear(index,eEvents);
}

/*******************************************************************************
  Function:
    PLIB_ETH_EVENTS PLIB_ETH_EventsGet ( ETH_MODULE_ID index )
 */

PLIB_ETH_EVENTS __attribute__((always_inline)) PLIB_ETH_EventsGet(ETH_MODULE_ID index)
{
    return PLIB_ETH_InterruptsGet(index);
}


/****************************************************************************
 * Function:        PLIB_ETH_RxSetBufferSize
 *****************************************************************************/
int PLIB_ETH_RxSetBufferSize(ETH_MODULE_ID index,int rxBuffSize)
{
    rxBuffSize >>= 4;     // truncate
    if(!rxBuffSize)
    {
        return -1;
    }

    PLIB_ETH_ReceiveBufferSizeSet(index, rxBuffSize);

    return rxBuffSize << 4;
}

#endif
