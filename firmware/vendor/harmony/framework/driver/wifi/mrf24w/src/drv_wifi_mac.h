/*******************************************************************************
  MRF24WG Driver Medium Access Control (MAC) Layer

  File Name:
    drv_wifi_mac.h

  Summary:
    MRF24WG Driver Medium Access Control (MAC) Layer

  Description:
    -Provides access to MRF24WG Wi-Fi controller
    -Reference: MRF24WG Datasheet, IEEE 802.11 Standard
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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

#ifndef _DRV_WIFI_MAC_H
#define _DRV_WIFI_MAC_H

// MRF24W MAC descriptor
typedef struct
{
    const TCPIP_MAC_OBJECT *pObj; // safe cast to TCPIP_MAC_DCPT

    // specific MRF24W MAC data
    TCPIP_NET_IF *pNetIf;   // interface we belong to
    bool isInit;            // simple init status flag
    bool isOpen;            // simple open status flag
                            // only one hardware module supported for now
    SYS_STATUS sysStat;     // driver status
    // add other MRF24W data here
} MRF24W_MAC_DCPT;

extern bool g_RxDataLock;

// initialization function
TCPIP_MAC_RES _MRF24W_MACInitialize(const TCPIP_MAC_MODULE_CTRL *const stackData);

void MRF24W_MACPutHeader(TCPIP_MAC_ADDR *remote, uint16_t type, uint16_t dataLen);
bool _MRF24W_MACCheckLink(void);

TCPIP_MAC_RES MRF24W_MACEventInit(MRF24W_MAC_DCPT *pDcpt, TCPIP_MAC_EventF eventF, const void *eventParam, int intPri, int subPri);
TCPIP_MAC_RES MRF24W_MACEventDeInit(MRF24W_MAC_DCPT *pDcpt);
bool _MRF24W_MACEventSetMask(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents, bool enable);
bool _MRF24W_MACEventAcknowledge(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents);
TCPIP_MAC_EVENT _MRF24W_MACEventGetPending(TCPIP_MAC_HANDLE hMac);
void MRF24W_SetUserEvents(uint8_t event, uint16_t eventInfo, bool isMgmt);

#endif /* _DRV_WIFI_MAC_H */

// DOM-IGNORE-END
