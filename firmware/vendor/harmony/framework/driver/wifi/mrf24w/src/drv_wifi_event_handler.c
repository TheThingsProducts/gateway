/*******************************************************************************
  MRF24WG Driver Event Handler

  File Name:
    drv_wifi_event_handler.c

  Summary:
    MRF24WG Driver Event Handler

  Description:
    -Provides access to MRF24WG Wi-Fi controller
    -Reference: MRF24WG Datasheet, IEEE 802.11 Standard
*******************************************************************************/

//DOM-IGNORE-BEGIN
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

/*================*/
/*    INCLUDES    */
/*================*/
#include "drv_wifi_priv.h"

#include "drv_wifi_config_data.h"
#include "drv_wifi_debug_output.h"
#include "drv_wifi_eint.h"
#include "drv_wifi_mgmt_msg.h"
#include "drv_wifi_raw.h"
#include "drv_wifi_scan_helper.h"
#include "drv_wifi_softap_client_cache.h"

/*===============*/
/*    DEFINES    */
/*===============*/
/*-------------------------------------------*/
/* Connection Manager Event Message Subtypes */
/* (Used in Mgmt Indicate messages)          */
/*-------------------------------------------*/
#define WF_EVENT_CONNECTION_ATTEMPT_STATUS_SUBTYPE (6)
#define WF_EVENT_CONNECTION_LOST_SUBTYPE           (7)
#define WF_EVENT_CONNECTION_REESTABLISHED_SUBTYPE  (8)
#define WF_EVENT_KEY_CALCULATION_REQUEST_SUBTYPE   (9)
#define WF_EVENT_SCAN_RESULTS_READY_SUBTYPE        (11)
#define WF_EVENT_SCAN_IE_RESULTS_READY_SUBTYPE     (12)
#define WF_EVENT_SOFT_AP_EVENT_SUBTYPE             (13)
#define WF_EVENT_DISCONNECT_DONE_SUBTYPE           (14)

/* event values for index 2 of WF_CONNECTION_ATTEMPT_STATUS_EVENT_SUBTYPE */
#define CONNECTION_ATTEMPT_SUCCESSFUL ((uint8_t)1) /* if not 1 then failed to connect and info field is error code */
#define CONNECTION_ATTEMPT_FAILED     ((uint8_t)2)

/* event values for index 2 of WF_EVENT_CONNECTION_LOST_SUBTYPE */
#define CONNECTION_TEMPORARILY_LOST ((uint8_t)1)
#define CONNECTION_PERMANENTLY_LOST ((uint8_t)2)
#define CONNECTION_REESTABLISHED    ((uint8_t)3)

#define isPowerSavePending() ((POWER_SAVE_PENDING & s_pendingEventsFlags) == POWER_SAVE_PENDING)
#define isWpsConnectPending() ((WPS_CONNECT_PENDING & s_pendingEventsFlags) == WPS_CONNECT_PENDING)

/*=================*/
/*    VARIABLES    */
/*=================*/
volatile static uint8_t s_pendingEventsFlags;

tMgmtIndicatePassphraseReady g_passphraseReady;
DRV_WIFI_MGMT_INDICATE_SOFT_AP_EVENT g_softAPEvent;

/*=======================*/
/*    LOCAL FUNCTIONS    */
/*=======================*/
static void DRV_WIFI_PendingEventClear(uint8_t event);
static void DRV_WIFI_ReinforceWep(void);

/*****************************************************************************
 * FUNCTION: DRV_WIFI_ProcessEvent
 *
 * RETURNS: None.
 *
 * PARAMS: event -- event that occurred
 *         eventInfo -- additional information about the event.  Not all events
 *                      have associated info, in which case this value will be
 *                      set to DRV_WIFI_NO_ADDITIONAL_INFO (0xff).
 *
 * NOTES: For events that the application is not interested in simply leave the
 *        case statement empty.
 *
 *        Customize this function as needed for your application.
 *****************************************************************************/
void DRV_WIFI_ProcessEvent(uint16_t event, uint16_t eventInfo)
{
    switch (event) {
    /*--------------------------------------*/
    case DRV_WIFI_EVENT_CONNECTION_SUCCESSFUL:
    /*--------------------------------------*/
        SYS_CONSOLE_MESSAGE("MRF24W Event: Connection Successful\r\n");
        DRV_WIFI_ReinforceWep();
#if (DRV_WIFI_SAVE_WPS_CREDENTIALS == DRV_WIFI_ENABLED)
        if (DRV_WIFI_CONFIG_PARAMS(networkType) == DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE)
            WPSCredentialsSave();
#endif
#if defined(SYS_CONSOLE_ENABLE) || defined(SYS_CONSOLE_INSTANCES_NUMBER)
        OutputConnectionContext();
#endif
        break;

    /*--------------------------------------*/
    case DRV_WIFI_EVENT_CONNECTION_FAILED:
    case DRV_WIFI_EVENT_CONNECTION_TEMPORARILY_LOST:
    case DRV_WIFI_EVENT_CONNECTION_PERMANENTLY_LOST: {
    /*--------------------------------------*/
        uint8_t bssid_blank[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
        DRV_WIFI_BssidSet(bssid_blank);
#if defined(SYS_CONSOLE_ENABLE) || defined(SYS_CONSOLE_INSTANCES_NUMBER)
        OutputConnectionDebugMsg(event, eventInfo);
#endif
        break;
    }

    /*--------------------------------------*/
    case DRV_WIFI_EVENT_CONNECTION_REESTABLISHED:
    /*--------------------------------------*/
        SYS_CONSOLE_MESSAGE("MRF24W Event: Connection Reestablished\r\n");
        DRV_WIFI_ReinforceWep();
#if (DRV_WIFI_SAVE_WPS_CREDENTIALS == DRV_WIFI_ENABLED)
        if (DRV_WIFI_CONFIG_PARAMS(networkType) == DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE)
            WPSCredentialsSave();
#endif
#if defined(SYS_CONSOLE_ENABLE) || defined(SYS_CONSOLE_INSTANCES_NUMBER)
        OutputConnectionContext();
#endif
        break;

    /*--------------------------------------*/
    case DRV_WIFI_EVENT_KEY_CALCULATION_REQUEST:
    /*--------------------------------------*/
        SYS_CONSOLE_MESSAGE("MRF24W Event: DRV_WIFI_EVENT_KEY_CALCULATION_REQUEST\r\n");
        break;

    /*--------------------------------------*/
    case DRV_WIFI_EVENT_INVALID_WPS_PIN:
    /*--------------------------------------*/
        SYS_CONSOLE_MESSAGE("MRF24W Event: Connection Failed : Invalid WPS PIN\r\n");
        break;

    /*--------------------------------------*/
    case DRV_WIFI_EVENT_SCAN_RESULTS_READY:
    /*--------------------------------------*/
        SYS_CONSOLE_PRINT("MRF24W Event: Scan Results Ready, %d results\r\n", eventInfo);
        break;

    /*--------------------------*/
    case DRV_WIFI_EVENT_SOFT_AP:
    /*--------------------------*/
        if (DRV_WIFI_CONFIG_PARAMS(networkType) == DRV_WIFI_NETWORK_TYPE_SOFT_AP)
        {
            DRV_WIFI_MGMT_INDICATE_SOFT_AP_EVENT *p_softApEvent = DRV_WIFI_SoftApEventInfoGet();
#if defined(SYS_CONSOLE_ENABLE) || defined(SYS_CONSOLE_INSTANCES_NUMBER)
            uint8_t i;
            uint8_t *addr = p_softApEvent->address;
#endif
            SYS_CONSOLE_MESSAGE("MRF24W Event: Soft AP, ");
            if (p_softApEvent->event == DRV_WIFI_SOFTAP_EVENT_CONNECTED)
            {
                SYS_CONSOLE_MESSAGE("Connected, ");
            }
            else if (p_softApEvent->event == DRV_WIFI_SOFTAP_EVENT_DISCONNECTED)
            {
                SYS_CONSOLE_MESSAGE("Disconnected, ");
                if (p_softApEvent->reason == DRV_WIFI_SOFTAP_EVENT_LINK_LOST)
                {
                    SYS_CONSOLE_MESSAGE("Link Lost, ");
                }
                else if (p_softApEvent->reason == DRV_WIFI_SOFTAP_EVENT_RECEIVED_DEAUTH)
                {
                    SYS_CONSOLE_MESSAGE("Received Deauth, ");
                }
            }
#if defined(SYS_CONSOLE_ENABLE) || defined(SYS_CONSOLE_INSTANCES_NUMBER)
            for (i = 0; i < 6; ++i)
            {
                if (i < 5)
                {
                    SYS_CONSOLE_PRINT("%02x:", addr[i]);
                }
                else
                {
                    SYS_CONSOLE_PRINT("%02x\r\n", addr[i]);
                }
            }
#endif
        }
        else
        {
            SYS_CONSOLE_MESSAGE("MRF24W Event: Network type is not set to SoftAP, event is ignored\r\n");
        }
        break;

    /*--------------------------------------*/
    case DRV_WIFI_EVENT_DISCONNECT_DONE:
    /*--------------------------------------*/
        SYS_CONSOLE_MESSAGE("MRF24W Event: Disconnect complete\r\n");
        break;

    /*--------------------------------------*/
    case DRV_WIFI_EVENT_ERROR:
    /*--------------------------------------*/
        SYS_CONSOLE_PRINT("MRF24W Event: Wi-Fi error occurred (%d)\r\n  See DRV_WIFI_MGMT_ERRORS\r\n", eventInfo);
        break;
    default:
        DRV_WIFI_ASSERT(false, "MRF24W Event: Unknown Wi-Fi Event\r\n"); /* unknown event */
        break;
    }
}

bool ClientCacheUpdated(bool *connected, uint8_t *mac)
{
    if (g_drv_wifi_priv.clientCache.updated) {
        int i;
        for (i = 0; i < MAX_CLIENT_TABLE_SLOTS; ++i) {
            if (g_drv_wifi_priv.clientCache.updated & 1 << i) {
                *connected = g_drv_wifi_priv.clientCache.bitMap & 1 << i ? true: false;
                memcpy(mac, g_drv_wifi_priv.clientCache.mac[i].addr, 6 * sizeof(uint8_t));
                g_drv_wifi_priv.clientCache.updated &= ~(i << i);
                return true;
            }
        }
    }
    return false;
}

static void ClientCacheUpdate(bool connected, uint8_t *mac)
{
    int i;
    int idx;

    if (connected) {
        /* Check if the MAC address is already in the table. If so, we just update timestamp and return. */
        if (g_drv_wifi_priv.clientCache.bitMap) {
            for (i = 0; i < MAX_CLIENT_TABLE_SLOTS; ++i) {
                if (g_drv_wifi_priv.clientCache.bitMap & 1 << i) {
                    if (!memcmp(g_drv_wifi_priv.clientCache.mac[i].addr, mac, 6)) {
                        g_drv_wifi_priv.clientCache.mac[i].timeStamp = g_drv_wifi_priv.clientCache.seqNum++;
                        return;
                    }
                }
            }
        }

        /* Try to find an empty slot in the table. */
        for (i = 0; i < MAX_CLIENT_TABLE_SLOTS; ++i) {
            if (!(g_drv_wifi_priv.clientCache.bitMap & 1 << i)) {
                idx = i;
                g_drv_wifi_priv.clientCache.bitMap |= 1 << idx;
                g_drv_wifi_priv.clientCache.updated |= 1 << idx;
                memcpy(g_drv_wifi_priv.clientCache.mac[idx].addr, mac, 6);
                g_drv_wifi_priv.clientCache.mac[idx].timeStamp = g_drv_wifi_priv.clientCache.seqNum++;
                return;
            }
        }

        /* Cache table is full. Let's kick out the oldest. */
        for (i = 0; i < MAX_CLIENT_TABLE_SLOTS; ++i) {
            uint32_t min = 0;
            if (g_drv_wifi_priv.clientCache.mac[i].timeStamp >= min) {
                min = g_drv_wifi_priv.clientCache.mac[i].timeStamp;
                idx = i;
            }
        }
        g_drv_wifi_priv.clientCache.bitMap |= 1 << idx;
        g_drv_wifi_priv.clientCache.updated |= 1 << idx;
        memcpy(g_drv_wifi_priv.clientCache.mac[idx].addr, mac, 6);
        g_drv_wifi_priv.clientCache.mac[idx].timeStamp = g_drv_wifi_priv.clientCache.seqNum++;
        return;
    } else {
        /* If the MAC address is in the table, update its status to unconnected. */
        for (i = 0; i < MAX_CLIENT_TABLE_SLOTS; ++i) {
            if (g_drv_wifi_priv.clientCache.bitMap & 1 << i) {
                if (!memcmp(mac, g_drv_wifi_priv.clientCache.mac[i].addr, 6)) {
                    g_drv_wifi_priv.clientCache.bitMap &= ~(1 << i);
                    g_drv_wifi_priv.clientCache.updated |= 1 << i;
                    return;
                }
            }
        }
    }
}

/*****************************************************************************
 * FUNCTION: DRV_WIFI_MgmtIndProcess()
 *
 * RETURNS: Error code.
 *
 * PARAMS: None.
 *
 * NOTES: Processes a management indicate message.
 *****************************************************************************/
void DRV_WIFI_MgmtIndProcess()
{
    tMgmtIndicateHdr hdr;
    uint8_t buf[6];
    uint8_t event = 0xff;
    uint16_t eventInfo;

    // read mgmt indicate header (2 bytes)
    RawRead(RAW_SCRATCH_ID, MGMT_INDICATE_BASE, sizeof(tMgmtIndicateHdr), (uint8_t *)&hdr);

    /* if not a management indicate then fatal error */
    DRV_WIFI_ASSERT(hdr.type == WF_MGMT_INDICATE_TYPE, "Invalid indicate header");

    /* Determine which event occurred and handle it */
    switch (hdr.subType) {
    /*-----------------------------------------------------------------*/
    case WF_EVENT_CONNECTION_ATTEMPT_STATUS_SUBTYPE:
    /*-----------------------------------------------------------------*/
        RawReadRelative(RAW_SCRATCH_ID, 2, buf); /* read first 2 bytes after header */
        /* if connection attempt successful */
        if (buf[0] == CONNECTION_ATTEMPT_SUCCESSFUL)
        {
            event = DRV_WIFI_EVENT_CONNECTION_SUCCESSFUL;
            eventInfo = DRV_WIFI_NO_ADDITIONAL_INFO;
            SignalWiFiConnectionChanged(true);
            SetLogicalConnectionState(true);
            RxTxUnLockTimerStart();
            if (DRV_WIFI_CONFIG_PARAMS(networkType) == DRV_WIFI_NETWORK_TYPE_SOFT_AP)
                DRV_WIFI_SoftAPClientCache_Init();
        }
        /* else connection attempt failed */
        else
        {
            event = DRV_WIFI_EVENT_CONNECTION_FAILED;
            eventInfo = (uint16_t)(buf[0] << 8 | buf[1]); /* contains connection failure code */
            SetLogicalConnectionState(false);
        }
        break;

    /*-----------------------------------------------------------------*/
    case WF_EVENT_CONNECTION_LOST_SUBTYPE:
    /*-----------------------------------------------------------------*/
        /* read next two data bytes in message
           buf[0] -- 1: Connection temporarily lost  2: Connection permanently lost 3: Connection Reestablished
           buf[1] -- 0: Beacon Timeout  1: Deauth from AP  */
        RawReadRelative(RAW_SCRATCH_ID, 2, buf);
        if (buf[0] == CONNECTION_TEMPORARILY_LOST)
        {
            event = DRV_WIFI_EVENT_CONNECTION_TEMPORARILY_LOST;
            eventInfo = (uint16_t)buf[1]; /* lost due to beacon timeout or deauth */
            SignalWiFiConnectionChanged(false);
            SetLogicalConnectionState(false);
        }
        else if (buf[0] == CONNECTION_PERMANENTLY_LOST)
        {
            event = DRV_WIFI_EVENT_CONNECTION_PERMANENTLY_LOST;
            eventInfo = (uint16_t)buf[1]; /* lost due to beacon timeout or deauth */
            g_drv_wifi_priv.isConnPermanentlyLost = true;
            SignalWiFiConnectionChanged(false);
            SetLogicalConnectionState(false);
        }
        else if (buf[0] == CONNECTION_REESTABLISHED)
        {
            event = DRV_WIFI_EVENT_CONNECTION_REESTABLISHED;
            eventInfo = (uint16_t)buf[1]; /* originally lost due to beacon timeout or deauth */
            g_drv_wifi_priv.isConnReestablished = true;
            SignalWiFiConnectionChanged(true);
            SetLogicalConnectionState(true);
        }
        else
        {
            /* invalid parameter in message */
            DRV_WIFI_ASSERT(false, "");
        }
        break;

    /*-----------------------------------------------------------------*/
    case WF_EVENT_KEY_CALCULATION_REQUEST_SUBTYPE:
    /*-----------------------------------------------------------------*/
        event = DRV_WIFI_EVENT_KEY_CALCULATION_REQUEST;
        RawReadRelative(RAW_SCRATCH_ID, sizeof(tMgmtIndicatePassphraseReady), (uint8_t *)&g_passphraseReady);
        DRV_WIFI_PendingEventSet(WPS_CONNECT_PENDING);
        break;

    /*-----------------------------------------------------------------*/
    case WF_EVENT_SCAN_RESULTS_READY_SUBTYPE:
    /*-----------------------------------------------------------------*/
        RawReadRelative(RAW_SCRATCH_ID, 1, buf);
        event = DRV_WIFI_EVENT_SCAN_RESULTS_READY;
        eventInfo = (uint16_t)buf[0]; /* number of scan results */
        DRV_WIFI_ScanComplete(eventInfo);
        break;

    /*-----------------------------------------------------------------*/
    case WF_EVENT_SCAN_IE_RESULTS_READY_SUBTYPE:
    /*-----------------------------------------------------------------*/
        event = DRV_WIFI_EVENT_IE_RESULTS_READY;
        /* read indexes 2 and 3 containing the 16-bit value of IE bytes */
        RawReadRelative(RAW_SCRATCH_ID, 2, (uint8_t *)&eventInfo);
        eventInfo = TCPIP_Helper_ntohs(eventInfo); /* fix endianess of 16-bit value */
        break;

    /*-----------------------------------------------------------------*/
    case WF_EVENT_SOFT_AP_EVENT_SUBTYPE: /* Valid only with 3108 or the later module FW version */
    /*-----------------------------------------------------------------*/
        event = DRV_WIFI_EVENT_SOFT_AP;
        RawReadRelative(RAW_SCRATCH_ID, sizeof(DRV_WIFI_MGMT_INDICATE_SOFT_AP_EVENT), (uint8_t *)&g_softAPEvent);
        if (g_softAPEvent.event == DRV_WIFI_SOFTAP_EVENT_CONNECTED)
            ClientCacheUpdate(true, g_softAPEvent.address);
        else 
            ClientCacheUpdate(false, g_softAPEvent.address);
        break;

    /*-----------------------------------------------------------------*/
    case WF_EVENT_DISCONNECT_DONE_SUBTYPE:
    /*-----------------------------------------------------------------*/
        event = DRV_WIFI_EVENT_DISCONNECT_DONE;
        /* set state to no connection */
        SetLogicalConnectionState(false);
        break;

    /*-----------------------------------------------------------------*/
    default:
    /*-----------------------------------------------------------------*/
        DRV_WIFI_ASSERT(false, "");
        break;
    }

    /* if the user wants to be notified of the event */
    DRV_WIFI_UserEventSet(event, eventInfo, true);
}

/*******************************************************************************
  Function:
    DRV_WIFI_MGMT_INDICATE_SOFT_AP_EVENT *DRV_WIFI_SoftApEventInfoGet(void)

  Summary:
    Gets the stored Soft AP event info.

  Description:
    This function retrieves the additional event info after a Soft AP event has
    occurred.

  Parameters:
    p_event -- pointer to where event info is written.  See
               DRV_WIFI_MGMT_INDICATE_SOFT_AP_EVENT structure.

  Returns:
    None.

  Remarks:
    None.
 *******************************************************************************/
DRV_WIFI_MGMT_INDICATE_SOFT_AP_EVENT *DRV_WIFI_SoftApEventInfoGet(void)
{
    return &g_softAPEvent;
}

// called by TCPIP_STACK_Task() if any Wi-Fi event is pending
void DRV_WIFI_PendingEventHandle(void)
{
    if (isPowerSavePending())
    {
        DRV_WIFI_PendingEventClear(POWER_SAVE_PENDING);
        PowerSaveTask();
    }

    if (isWpsConnectPending())
    {
        DRV_WIFI_PendingEventClear(WPS_CONNECT_PENDING);
        WpaKeySet();
    }
}

bool DRV_WIFI_PendingEventGet(void)
{
    if ((GetRxPendingCount() > 0) || (s_pendingEventsFlags > 0))
        return true;
    
    return false;
}

// sets an event bit
void DRV_WIFI_PendingEventSet(uint8_t event)
{
    bool intEnabled = SYS_INT_SourceDisable(MRF_INT_SOURCE);

    s_pendingEventsFlags |= event;

    if (intEnabled)
    {
        DRV_WIFI_INT_SourceEnable();
    }
}

static void DRV_WIFI_PendingEventClear(uint8_t event)
{
    bool intEnabled = SYS_INT_SourceDisable(MRF_INT_SOURCE);

    s_pendingEventsFlags &= ~event;

    if (intEnabled)
    {
        DRV_WIFI_INT_SourceEnable();
    }
}

void DRV_WIFI_AllEventClear(void)
{
    s_pendingEventsFlags = 0x00;
}

static void DRV_WIFI_ReinforceWep(void)
{
    uint8_t data[2];
    if ((DRV_WIFI_CONFIG_PARAMS(networkType) == DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE) &&
        ((DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_WEP_40) ||
        (DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_WEP_104)))
    {
        data[0] = 1;
        data[1] = 1; 
        SetParamMsgSend(PARAM_SECURITY_CONTROL , (uint8_t *)data, 2);
    }
}

//DOM-IGNORE-END
