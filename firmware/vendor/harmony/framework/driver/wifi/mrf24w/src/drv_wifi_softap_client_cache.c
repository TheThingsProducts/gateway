/*******************************************************************************
  MRF24WG Wi-Fi Driver Soft AP Clients Cache Functions

  File Name:
    drv_wifi_softap_client_cache.c

  Summary:
    MRF24WG Wi-Fi Driver Soft AP Clients Cache Functions

  Description:
    MRF24WG Wi-Fi Driver Soft AP Clients Cache Functions
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

#include "drv_wifi_priv.h"

#include "drv_wifi_softap_client_cache.h"

#define CLIENT_CACHE_SIZE 4

static DRV_WIFI_SOFTAP_CLIENT_CACHE s_cache[CLIENT_CACHE_SIZE];

static int LookUpMac(uint8_t *mac)
{
    int i;

    for (i = 0; i < CLIENT_CACHE_SIZE; i++)
    {
        if (memcmp(mac, s_cache[i].mac, 6) == 0)
        {
            return i;
        }
    }
    
    return -1;
}

static int AddNewMac(uint8_t *NewMac)
{
    // look for a new slot
    uint8_t ZeroArray[6] = {0, 0, 0, 0, 0, 0};
    int i;

    for (i = 0; i < CLIENT_CACHE_SIZE; i++)
    {
        if (memcmp(ZeroArray, s_cache[i].mac, 6) == 0)
        {
            break;
        }
    }

    if (i < CLIENT_CACHE_SIZE)
    {
        memcpy(s_cache[i].mac, NewMac, 6);
        return i;
    }
    else
        return -1;
}

static void DelOldMac(uint8_t *NewMac)
{
    int i;
    uint8_t ZeroArray[6] = {0, 0, 0, 0, 0, 0};

    for (i = 0; i < CLIENT_CACHE_SIZE; i++)
    {
        if (memcmp(NewMac, s_cache[i].mac, 6) == 0)
        {
            break;
        }
    }
    
    if (i < CLIENT_CACHE_SIZE)
    {
        memcpy(s_cache[i].mac, ZeroArray, 6);
    }
}

bool DRV_WIFI_SoftAPClientCache_Refresh(DRV_WIFI_MGMT_INDICATE_SOFT_AP_EVENT *p_softApEvent)
{
    int position = 0;
    uint8_t *addr = p_softApEvent->address;

    if (p_softApEvent->event == DRV_WIFI_SOFTAP_EVENT_CONNECTED)
    {
        position = LookUpMac(addr);
        if (position == -1)
            AddNewMac(addr);
    }
    else if (p_softApEvent->event == DRV_WIFI_SOFTAP_EVENT_DISCONNECTED)
    {
        position = LookUpMac(addr);
        if (position != -1)
            DelOldMac(addr);
    }
    
    return true;
}

void DRV_WIFI_SoftAPClientCache_Init(void)
{
    int i;

    for (i = 0; i < CLIENT_CACHE_SIZE; i++)
    {
        s_cache[i].mac[0] = 0;
        s_cache[i].mac[1] = 0;
        s_cache[i].mac[2] = 0;
        s_cache[i].mac[3] = 0;
        s_cache[i].mac[4] = 0;
        s_cache[i].mac[5] = 0;
    }
}

bool DRV_WIFI_SoftAPClientCache_LookUpClient( uint8_t *MacAddr)
{
    int position;
    
    position = LookUpMac(MacAddr);
    if (position == -1) {
        return false;
    }

    return true;
}

void DRV_WIFI_SoftAPClientCache_Print(void)
{
    int i;

    SYS_CONSOLE_MESSAGE("SoftAP Client Table: \r\n--------------------\r\n");
    for (i = 0; i < CLIENT_CACHE_SIZE; i++)
    {
        SYS_CONSOLE_PRINT("%02x:%02x:%02x:%02x:%02x:%02x\r\n",
                s_cache[i].mac[0],
                s_cache[i].mac[1],
                s_cache[i].mac[2],
                s_cache[i].mac[3],
                s_cache[i].mac[4],
                s_cache[i].mac[5]);
    }
}

// DOM-IGNORE-END
