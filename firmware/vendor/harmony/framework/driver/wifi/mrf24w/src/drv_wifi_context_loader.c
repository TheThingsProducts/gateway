/*******************************************************************************
  MRF24W Context Loading Support

  File Name:
    drv_wifi_context_loader.c

  Summary:
    Loads Wi-Fi context to MRF24WG.

  Description:
    Loads Wi-Fi context to MRF24WG.
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

/******************/
/*    INCLUDES    */
/******************/
#include "drv_wifi_priv.h"

#include "drv_wifi_config_data.h"
#include "drv_wifi_debug_output.h"

/***************/
/*    TYPES    */
/***************/
typedef enum {
    LOAD_STATE1 = 0,
    LOAD_STATE2,
    LOAD_STATE3
} LOAD_STATE;

typedef enum {
    SET_WPA_STATE1 = 0,
    SET_WPA_STATE2,
    SET_WPA_STATE3
} SET_WPA_STATE;

/*******************/
/*    FUNCTIONS    */
/*******************/
static void SetWepSecurity(void);
static TCPIP_MAC_RES SetWpaSecurity(void);
static void SetWpsSecurity(void);

TCPIP_MAC_RES DRV_WIFI_ContextLoad(void)
{
    static LOAD_STATE loadState = LOAD_STATE1;
    TCPIP_MAC_RES res = TCPIP_MAC_RES_PENDING, tmp;
    DRV_WIFI_SCAN_CONTEXT scanContext;
    uint8_t channelList[] = DRV_WIFI_DEFAULT_CHANNEL_LIST;
#if (DRV_WIFI_DEFAULT_NETWORK_TYPE == DRV_WIFI_NETWORK_TYPE_SOFT_AP)
    uint8_t channelList_postscan[] = {};
#endif

    switch (loadState) {
    //--------------------------
    case LOAD_STATE1:
    //--------------------------
        DRV_WIFI_SsidSet(DRV_WIFI_CONFIG_PARAMS(ssid), DRV_WIFI_CONFIG_PARAMS(ssidLen));
        DRV_WIFI_NetworkTypeSet(DRV_WIFI_CONFIG_PARAMS(networkType));

        if (DRV_WIFI_CONFIG_PARAMS(networkType) == DRV_WIFI_NETWORK_TYPE_SOFT_AP) {
            scanContext.scanType = DRV_WIFI_PASSIVE_SCAN;
        } else if (DRV_WIFI_CONFIG_PARAMS(networkType) == DRV_WIFI_NETWORK_TYPE_ADHOC) {
#if (DRV_WIFI_DEFAULT_ADHOC_PRESCAN == DRV_WIFI_ENABLED)
            scanContext.scanType = DRV_WIFI_PASSIVE_SCAN;
#else
            scanContext.scanType = DRV_WIFI_ACTIVE_SCAN;
#endif
        } else {
            scanContext.scanType = DRV_WIFI_ACTIVE_SCAN;
        }

        scanContext.minChannelTime = DRV_WIFI_DEFAULT_SCAN_MIN_CHANNEL_TIME;
        scanContext.maxChannelTime = DRV_WIFI_DEFAULT_SCAN_MAX_CHANNEL_TIME;
        scanContext.probeDelay = DRV_WIFI_DEFAULT_SCAN_PROBE_DELAY;
        scanContext.scanCount = DRV_WIFI_DEFAULT_SCAN_COUNT;

        DRV_WIFI_ScanContextSet(&scanContext);

        // Behavior of postscan is determined by default network type and current network type together
        // So this preprocessor conditional should not be got rid of
#if (DRV_WIFI_DEFAULT_NETWORK_TYPE == DRV_WIFI_NETWORK_TYPE_SOFT_AP)
        if (DRV_WIFI_CONFIG_PARAMS(networkType) != DRV_WIFI_NETWORK_TYPE_SOFT_AP)
        {
            DRV_WIFI_ChannelListSet(channelList_postscan, sizeof(channelList_postscan));
        }
        else
        {
            DRV_WIFI_ChannelListSet(channelList, sizeof(channelList));
        }
#else /* (DRV_WIFI_DEFAULT_NETWORK_TYPE != DRV_WIFI_NETWORK_TYPE_SOFT_AP) */
        DRV_WIFI_ChannelListSet(channelList, sizeof(channelList));
#endif /* (DRV_WIFI_DEFAULT_NETWORK_TYPE == DRV_WIFI_NETWORK_TYPE_SOFT_AP) */

        // The Retry Count parameter tells the WiFi Connection manager how many attempts to make when trying
        // to connect to an existing network.  In the Infrastructure case, the default is to retry forever so that
        // if the AP is turned off or out of range, the radio will continue to attempt a connection until the
        // AP is eventually back on or in range.  In the Ad Hoc case, the default is to retry 3 times since the
        // purpose of attempting to establish a network in the Ad Hoc case is only to verify that one does not
        // initially exist.  If the retry count was set to DRV_WIFI_RETRY_FOREVER in the AdHoc mode, an Ad Hoc network
        // would never be established.
#if (DRV_WIFI_DEFAULT_NETWORK_TYPE == DRV_WIFI_NETWORK_TYPE_SOFT_AP) || (DRV_WIFI_DEFAULT_NETWORK_TYPE == DRV_WIFI_NETWORK_TYPE_ADHOC)
        if (DRV_WIFI_CONFIG_PARAMS(networkType) == DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE)
        {
            DRV_WIFI_ReconnectModeSet(DRV_WIFI_RETRY_FOREVER,         // retry forever to connect to Wi-Fi network
                                      DRV_WIFI_ATTEMPT_TO_RECONNECT,  // reconnect on deauth from AP
                                      40,                             // beacon timeout is 40 beacon periods
                                      DRV_WIFI_ATTEMPT_TO_RECONNECT); // reconnect on beacon timeout
        }
        else if ((DRV_WIFI_CONFIG_PARAMS(networkType) == DRV_WIFI_NETWORK_TYPE_ADHOC) || 
            (DRV_WIFI_CONFIG_PARAMS(networkType) == DRV_WIFI_NETWORK_TYPE_SOFT_AP))
        {
            DRV_WIFI_ReconnectModeSet(DRV_WIFI_DEFAULT_LIST_RETRY_COUNT,          // retry N times to start or join AdHoc network
                                      DRV_WIFI_DO_NOT_ATTEMPT_TO_RECONNECT, // do not attempt to reconnect on deauth from station
                                      40,                                   // beacon timeout is 40 beacon periods
                                      DRV_WIFI_ATTEMPT_TO_RECONNECT);       // reconnect on beacon timeout
        }
        else
        {
            DRV_WIFI_ASSERT(false, "Please compile with correct Network Type and correct Retry Count");
        }
#else /* (DRV_WIFI_DEFAULT_NETWORK_TYPE != DRV_WIFI_NETWORK_TYPE_SOFT_AP) && (DRV_WIFI_DEFAULT_NETWORK_TYPE != DRV_WIFI_NETWORK_TYPE_ADHOC) */
        DRV_WIFI_ReconnectModeSet(DRV_WIFI_DEFAULT_LIST_RETRY_COUNT,    // retry N times to start or join AdHoc network
                                  DRV_WIFI_ATTEMPT_TO_RECONNECT,  // reconnect on deauth from AP
                                  40,                             // beacon timeout is 40 beacon periods
                                  DRV_WIFI_ATTEMPT_TO_RECONNECT); // reconnect on beacon timeout
#endif /* (DRV_WIFI_DEFAULT_NETWORK_TYPE == DRV_WIFI_NETWORK_TYPE_SOFT_AP) || (DRV_WIFI_DEFAULT_NETWORK_TYPE == DRV_WIFI_NETWORK_TYPE_ADHOC) */

        // Set Tx Mode
        loadState++;
        break;

    //--------------------------
    case LOAD_STATE2:
    //--------------------------
        tmp = TCPIP_MAC_RES_OK;
        if (DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_OPEN)
        {
            DRV_WIFI_SecurityOpenSet();
        }
        else if ((DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_WEP_40) || (DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_WEP_104))
        {
            SetWepSecurity();
        }
        else if ((DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_WPA_AUTO_WITH_KEY)         ||
                 (DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_WPA_AUTO_WITH_PASS_PHRASE) ||
                 (DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_WPA_WITH_KEY) ||
                 (DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_WPA_WITH_PASS_PHRASE) ||
                 (DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_WPA2_WITH_KEY) ||
                 (DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_WPA2_WITH_PASS_PHRASE))
        {
            tmp = SetWpaSecurity();
        }
        else if ((DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_WPS_PIN) ||
                 (DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_WPS_PUSH_BUTTON))
        {
            SetWpsSecurity();
        }
     
        if (TCPIP_MAC_RES_OK == tmp)
        {
            loadState++;
        }
        break;

    //--------------------------
    case LOAD_STATE3:
    //--------------------------
        // override reconnect mode if connection manager disabled
#if (DRV_WIFI_MODULE_CONNECTION_MANAGER == DRV_WIFI_DISABLED)
        DRV_WIFI_ReconnectModeSet(0,                                     // report-only when connection lost (no reconnect)
                                  DRV_WIFI_DO_NOT_ATTEMPT_TO_RECONNECT,  // report-only when deauth received (no reconnect)
                                  40,                                    // set beacon timeout to 40 beacon periods
                                  DRV_WIFI_DO_NOT_ATTEMPT_TO_RECONNECT); // report only when beacon timeout occurs
#endif

        if (DRV_WIFI_CONFIG_PARAMS(networkType) == DRV_WIFI_NETWORK_TYPE_SOFT_AP) {
#if defined(DRV_WIFI_SOFTAP_SEND_KEEP_ALIVE)
#   if (DRV_WIFI_SOFTAP_SEND_KEEP_ALIVE == DRV_WIFI_ENABLED)
            DRV_WIFI_LinkDownThresholdSet(DRV_WIFI_SOFTAP_LINK_LOST_THRESHOLD);
#   endif
#endif
        } else {
#if defined(DRV_WIFI_CHECK_LINK_STATUS)
#   if (DRV_WIFI_CHECK_LINK_STATUS == DRV_WIFI_ENABLED)
            DRV_WIFI_LinkDownThresholdSet(DRV_WIFI_LINK_LOST_THRESHOLD);
#   endif
#endif
        }

        res = TCPIP_MAC_RES_OK;
        loadState = LOAD_STATE1;
        break;
    } // end switch

    return res;
}

static void SetWepSecurity(void)
{
    DRV_WIFI_WEP_CONTEXT wepContext;

    wepContext.wepSecurityType = DRV_WIFI_CONFIG_PARAMS(securityMode);
    wepContext.wepKeyLength = DRV_WIFI_CONFIG_PARAMS(securityKeyLen);
    memcpy(wepContext.wepKey, DRV_WIFI_CONFIG_PARAMS(securityKey), wepContext.wepKeyLength);
    wepContext.wepKeyType = DRV_WIFI_CONFIG_PARAMS(wepKeyType);

    DRV_WIFI_SecurityWepSet(&wepContext);
}

static TCPIP_MAC_RES SetWpaSecurity(void)
{
    DRV_WIFI_WPA_CONTEXT wpaContext;
    static SET_WPA_STATE setWpaState = SET_WPA_STATE1;
    TCPIP_MAC_RES res = TCPIP_MAC_RES_PENDING, tmp;

    switch (setWpaState) {
    case SET_WPA_STATE1:
        if (DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_WPA_AUTO_WITH_PASS_PHRASE ||
            DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_WPA_WITH_PASS_PHRASE ||
            DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_WPA2_WITH_PASS_PHRASE)
        {
            tmp = DRV_WIFI_KeyDerive(DRV_WIFI_CONFIG_PARAMS(securityKeyLen), DRV_WIFI_CONFIG_PARAMS(securityKey), DRV_WIFI_CONFIG_PARAMS(ssidLen), DRV_WIFI_CONFIG_PARAMS(ssid));
            if (TCPIP_MAC_RES_OK == tmp)
            {
                DRV_WIFI_CONFIG_PARAMS(securityMode)--;
                DRV_WIFI_CONFIG_PARAMS(securityKeyLen) = 32;
                setWpaState++;
            }
        }
        else
        {
            setWpaState++;
        }
        break;

    case SET_WPA_STATE2:
        wpaContext.wpaSecurityType = DRV_WIFI_CONFIG_PARAMS(securityMode);
        DRV_WIFI_ASSERT(DRV_WIFI_CONFIG_PARAMS(securityKeyLen) <= sizeof(wpaContext.keyInfo.key), "");
        memcpy(wpaContext.keyInfo.key, DRV_WIFI_CONFIG_PARAMS(securityKey), DRV_WIFI_CONFIG_PARAMS(securityKeyLen));
        wpaContext.keyInfo.keyLength = DRV_WIFI_CONFIG_PARAMS(securityKeyLen);
        DRV_WIFI_SecurityWpaSet(&wpaContext);
        res = TCPIP_MAC_RES_OK;
        setWpaState++;
        //break; // no break, continue to next state

    case SET_WPA_STATE3:
        setWpaState = SET_WPA_STATE1;
        break;
    }

    return res;
}

static void SetWpsSecurity(void)
{
    DRV_WIFI_WPS_CONTEXT wpsContext;

    wpsContext.wpsSecurityType = DRV_WIFI_CONFIG_PARAMS(securityMode);

    if (wpsContext.wpsSecurityType == DRV_WIFI_SECURITY_WPS_PUSH_BUTTON)
    {
        memset(wpsContext.wpsPin, 0x00, DRV_WIFI_WPS_PIN_LENGTH);
        wpsContext.wpsPinLength = 0;
    }
    else
    {
        DRV_WIFI_ASSERT(DRV_WIFI_CONFIG_PARAMS(securityKeyLen) <= sizeof(wpsContext.wpsPin), "");
        memcpy(wpsContext.wpsPin, (const void *)DRV_WIFI_CONFIG_PARAMS(securityKey), DRV_WIFI_CONFIG_PARAMS(securityKeyLen));
        wpsContext.wpsPinLength = DRV_WIFI_CONFIG_PARAMS(securityKeyLen);
    }

    DRV_WIFI_SecurityWpsSet(&wpsContext);
    DRV_WIFI_HostKeyDeriveModeSet();
}

//DOM-IGNORE-END
