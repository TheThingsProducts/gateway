/*******************************************************************************
  MRF24WG Wi-Fi Driver Soft AP Clients Cache Functions

  File Name:
    drv_wifi_softap_client_cache.h

  Summary:
    MRF24WG Wi-Fi Driver Soft AP Clients Cache Functions

  Description:
    MRF24WG Wi-Fi Driver Soft AP Clients Cache Functions
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc. All rights reserved.

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

#ifndef _DRV_WIFI_SOFTAP_CLIENT_CACHE_H
#define _DRV_WIFI_SOFTAP_CLIENT_CACHE_H

/*******************************************************************************
 * Data Types
 *******************************************************************************/
typedef struct
{
    uint8_t mac[6];
} DRV_WIFI_SOFTAP_CLIENT_CACHE;

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
void DRV_WIFI_SoftAPClientCache_Init(void);

bool DRV_WIFI_SoftAPClientCache_Refresh(DRV_WIFI_MGMT_INDICATE_SOFT_AP_EVENT *p_softApEvent);

/*******************************************************************************
    Function:
      bool DRV_WIFI_SoftAPClientCache_LookUpClient(uint8_t *MacAddr)
    Summary:
      Query in Client Table, is this clinet connected or not?.
    Parameters:
      MacAddr - Mac Address
    Returns:
      true - connected
      false - not connected
    Example:
    <code>
    uint8_t MacAddr1[6] = { 00, 01, c8, 00, 00, 01}
    if (true == DRV_WIFI_SoftAPClientCache_LookUpClient(MacAddr1))
        printf("yes, it is connected\r\n");
    else
        printf("No,  not connected\r\n");
    </code>
    Remarks:
      None
 *******************************************************************************/
bool DRV_WIFI_SoftAPClientCache_LookUpClient(uint8_t *MacAddr);

void DRV_WIFI_SoftAPClientCache_Print(void);

#endif /* _DRV_WIFI_SOFTAP_CLIENT_CACHE_H */

// DOM-IGNORE-END
