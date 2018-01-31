/*******************************************************************************
  MRF24WN Connection Manager Implementation

  File Name:
    wdrv_mrf24wn_connmgr.c

  Summary:


  Description:

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

#include "wdrv_mrf24wn_main.h"

#define DISCONNECT_DONE_NOTIFY() WDRV_SemGive(&g_wdrv_priv.disconnectDoneSync)

static WDRV_CONNECTION_STATES s_ConnectionStatus = WDRV_CSTATE_NOT_CONNECTED;
static bool s_logicalConnection = false;
static char *s_connect_failure_reason[] = {
    "",
    "NO_NETWORK_AVAIL",
    "LOST_LINK",
    "DISCONNECT_CMD",
    "BSS_DISCONNECTED",
    "AUTH_FAILED",
    "ASSOC_FAILED",
    "NO_RESOURCES_AVAIL",
    "CONNECTION_DENIED",
    "",
    "INVALID_PROFILE",
    "",
    "PROFILE_MISMATCH",
    "CONNECTION_EVICTED",
};

static void ConnectionStateSet(bool state);
static void ConnectStatusUpdate(bool connected, uint8_t reason);

bool ClientCacheUpdated(bool *connected, uint8_t *mac)
{
    if (g_wdrv_priv.clientCache.updated) {
        int i;
        for (i = 0; i <  WDRV_MAX_CLIENT_TABLE_SLOTS; ++i) {
            if (g_wdrv_priv.clientCache.updated & 1 << i) {
                *connected = g_wdrv_priv.clientCache.bitMap & 1 << i ? true: false;
                memcpy(mac, g_wdrv_priv.clientCache.mac[i].addr, 6 * sizeof(uint8_t));
                g_wdrv_priv.clientCache.updated &= ~(i << i);
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
        if (g_wdrv_priv.clientCache.bitMap) {
            for (i = 0; i <  WDRV_MAX_CLIENT_TABLE_SLOTS; ++i) {
                if (g_wdrv_priv.clientCache.bitMap & 1 << i) {
                    if (!memcmp(g_wdrv_priv.clientCache.mac[i].addr, mac, 6)) {
                        g_wdrv_priv.clientCache.mac[i].timeStamp = g_wdrv_priv.clientCache.seqNum++;
                        return;
                    }
                }
            }
        }

        /* Try to find an empty slot in the table. */
        for (i = 0; i <  WDRV_MAX_CLIENT_TABLE_SLOTS; ++i) {
            if (!(g_wdrv_priv.clientCache.bitMap & 1 << i)) {
                idx = i;
                g_wdrv_priv.clientCache.bitMap |= 1 << idx;
                g_wdrv_priv.clientCache.updated |= 1 << idx;
                memcpy(g_wdrv_priv.clientCache.mac[idx].addr, mac, 6);
                g_wdrv_priv.clientCache.mac[idx].timeStamp = g_wdrv_priv.clientCache.seqNum++;
                return;
            }
        }

        /* Cache table is full. Let's kick out the oldest. */
        for (i = 0; i <  WDRV_MAX_CLIENT_TABLE_SLOTS; ++i) {
            uint32_t min = 0;
            if (g_wdrv_priv.clientCache.mac[i].timeStamp >= min) {
                min = g_wdrv_priv.clientCache.mac[i].timeStamp;
                idx = i;
            }
        }
        g_wdrv_priv.clientCache.bitMap |= 1 << idx;
        g_wdrv_priv.clientCache.updated |= 1 << idx;
        memcpy(g_wdrv_priv.clientCache.mac[idx].addr, mac, 6);
        g_wdrv_priv.clientCache.mac[idx].timeStamp = g_wdrv_priv.clientCache.seqNum++;
        return;
    } else {
        /* If the MAC address is in the table, update its status to unconnected. */
        for (i = 0; i <  WDRV_MAX_CLIENT_TABLE_SLOTS; ++i) {
            if (g_wdrv_priv.clientCache.bitMap & 1 << i) {
                if (!memcmp(mac, g_wdrv_priv.clientCache.mac[i].addr, 6)) {
                    g_wdrv_priv.clientCache.bitMap &= ~(1 << i);
                    g_wdrv_priv.clientCache.updated |= 1 << i;
                    return;
                }
            }
        }
    }
}

void ProceedConnectEventCB(uint32_t connected, uint8_t devID, uint8_t *mac, bool macConn, uint8_t reason)
{
    /*
     * The meaning of variable "mac" varies.
     * In Infrastructure mode, it most likely points to an AP's MAC address, which is actually
     *  the network BSSID.
     * In Soft AP mode, it sometimes points to a client's MAC address, but sometimes points to
     *  MRF24WN's own MAC address. It points to an all "0xFF" MAC address too in certain cases.
     */
    const uint8_t macAllFF[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    static bool softAPStarted = false;

    if (connected == true) {
        if (WDRV_CONFIG_PARAMS(networkType) == WDRV_NETWORK_TYPE_INFRASTRUCTURE) {
            ConnectionStateSet(true);
            ConnectStatusUpdate(connected, reason);
            WDRV_DBG_INFORM_MESSAGE(("Connected to AP\r\n"));
        } else if (WDRV_CONFIG_PARAMS(networkType) == WDRV_NETWORK_TYPE_SOFT_AP) {
            if (!softAPStarted) {
                softAPStarted = true;
                ConnectionStateSet(true);
                WDRV_DBG_INFORM_MESSAGE(("Soft AP network is enabled\r\n"));
            } else {
                ClientCacheUpdate(connected, mac);
                WDRV_DBG_INFORM_MESSAGE(("A client is connected\r\n"));
            }
        }
    } else if (connected == false) {
        if (WDRV_CONFIG_PARAMS(networkType) == WDRV_NETWORK_TYPE_INFRASTRUCTURE) {
            ConnectionStateSet(false);
            ConnectStatusUpdate(connected, reason);
            WDRV_DBG_INFORM_PRINT(("Connection failed - %s\r\n", s_connect_failure_reason[reason]));
        } else if (WDRV_CONFIG_PARAMS(networkType) == WDRV_NETWORK_TYPE_SOFT_AP) {
            // if the MAC address pointer variable "mac" points to an all "0xFF" MAC address,
            // it means that the Soft AP network is already disabled
            if (memcmp(mac, macAllFF, 6) == 0) {
                softAPStarted = false;
                ConnectionStateSet(false);
                if (g_wdrv_priv.isDisconnectRequested)
                    DISCONNECT_DONE_NOTIFY();
                WDRV_DBG_INFORM_PRINT(("Soft AP network is disabled\r\n"));
            } else {
                ClientCacheUpdate(connected, mac);
                if (g_wdrv_priv.clientCache.bitMap == 0) {
                    WDRV_DBG_INFORM_PRINT(("Last client has left\r\n"));
                } else {
                    WDRV_DBG_INFORM_PRINT(("A client has left\r\n"));
                }
            }
        }
    }
}

bool isLinkUp(void)
{
    return s_logicalConnection;
}

static void ConnectionStateSet(bool state)
{
    s_logicalConnection = state;
}

static void NetModeSet(uint8_t networkType)
{
    switch (networkType) {
    case WDRV_NETWORK_TYPE_INFRASTRUCTURE:
        WDRV_EXT_CmdNetModeBSSSet();
        break;
    case WDRV_NETWORK_TYPE_ADHOC:
        WDRV_ASSERT(false, "Ad-Hoc is not supported for now");
        break;
    case WDRV_NETWORK_TYPE_SOFT_AP:
        WDRV_EXT_CmdNetModeAPSet();
        break;
    case WDRV_NETWORK_TYPE_P2P:
        WDRV_ASSERT(false, "P2P is not supported for now");
        break;
    default:
        WDRV_ASSERT(false, "Undefined network type");
        break;
    }
}

static void SecuritySet(uint8_t securityMode)
{
    bool pinMode;

    switch (securityMode) {
    case WDRV_SECURITY_OPEN:
        WDRV_EXT_CmdSecNoneSet();
        break;
    case WDRV_SECURITY_WEP_40:
    case WDRV_SECURITY_WEP_104:
        if ((WDRV_CONFIG_PARAMS(securityKeyLen) == 5) || (WDRV_CONFIG_PARAMS(securityKeyLen) == 13))
        {
            int i;
            uint8_t tmpBuf[26]; 
            uint8_t c_value;
            for (i = 0; i < WDRV_CONFIG_PARAMS(securityKeyLen) * 2; i++)
            {   
                c_value = (i % 2 == 0 ) ? (WDRV_CONFIG_PARAMS(securityKey)[i / 2] >> 4) : (WDRV_CONFIG_PARAMS(securityKey)[i / 2] & 0x0F);
                tmpBuf[i] = (c_value > 9) ? ('A' + c_value - 0x0A) : ('0' + c_value - 0x00);
            }
            WDRV_CONFIG_PARAMS(securityKeyLen) *= 2;
            memcpy(WDRV_CONFIG_PARAMS(securityKey), tmpBuf, WDRV_CONFIG_PARAMS(securityKeyLen));
            WDRV_CONFIG_PARAMS(securityKey)[WDRV_CONFIG_PARAMS(securityKeyLen)] = 0x00;
        }
        WDRV_EXT_CmdSecWEPSet(WDRV_CONFIG_PARAMS(securityKey), WDRV_CONFIG_PARAMS(securityKeyLen));
        break;
    case WDRV_SECURITY_WPA_WITH_PASS_PHRASE:
        WDRV_EXT_CmdSecWPASet(WDRV_CONFIG_PARAMS(securityKey), WDRV_CONFIG_PARAMS(securityKeyLen));
        break;
    case WDRV_SECURITY_WPA2_WITH_PASS_PHRASE:
        WDRV_EXT_CmdSecWPA2Set(WDRV_CONFIG_PARAMS(securityKey), WDRV_CONFIG_PARAMS(securityKeyLen));
        break;
    case WDRV_SECURITY_WPS_PIN:
    case WDRV_SECURITY_WPS_PUSH_BUTTON:
         pinMode = WDRV_CONFIG_PARAMS(securityMode) == WDRV_SECURITY_WPS_PIN ? true : false;
         WDRV_EXT_CmdSecWpsSet(pinMode, WDRV_CONFIG_PARAMS(securityKey), WDRV_CONFIG_PARAMS(securityKeyLen));
        break;
    default:
        WDRV_ASSERT(false, "Undefined security mode");
        break;
    }
}

TCPIP_MAC_RES WDRV_Connect(void)
{
    TCPIP_MAC_RES res = TCPIP_MAC_RES_OK;

    WDRV_EXT_CmdSSIDSet(WDRV_CONFIG_PARAMS(ssid), WDRV_CONFIG_PARAMS(ssidLen));

    NetModeSet(WDRV_CONFIG_PARAMS(networkType));

    if (WDRV_CONFIG_PARAMS(networkType) == WDRV_NETWORK_TYPE_SOFT_AP)
        WDRV_EXT_CmdChannelSet(WDRV_DEFAULT_CHANNEL);

    SecuritySet(WDRV_CONFIG_PARAMS(securityMode));
    WDRV_EXT_CmdPowerSavePut(false);
    WDRV_DBG_INFORM_MESSAGE(("\r\nStart Wi-Fi Connection . . .\r\n"));

    // start the WiFi connection process
    if ((WDRV_CONFIG_PARAMS(securityMode) != WDRV_SECURITY_WPS_PIN) &&
        (WDRV_CONFIG_PARAMS(securityMode) != WDRV_SECURITY_WPS_PUSH_BUTTON)) {
        if (WDRV_EXT_CmdConnect() == 0)
            s_ConnectionStatus = WDRV_CSTATE_CONNECTION_IN_PROGRESS;
    }

    return res;
}

void WPSDoneCB(void)
{
    uint32_t status;

    WDRV_EXT_WPSResultsRead(p_wdrv_configData, &status);

    if (status == WDRV_SUCCESS) {
        WDRV_DBG_INFORM_MESSAGE(("WPS process finished successfully\r\n"));
        WDRV_CONFIG_DataSave();
        WDRV_Connect();
    } else {
        WDRV_DBG_INFORM_PRINT(("WPS process failed, status = %d\r\n", status));
        /* TODO: handle error case */
    }
}

static void LinkDownTimeoutCallback(uintptr_t context, uint32_t currTick)
{
    if (s_ConnectionStatus == WDRV_CSTATE_CONNECTION_TEMPORARY_LOST) {
        s_ConnectionStatus = WDRV_CSTATE_CONNECTION_PERMANENTLY_LOST;
        WDRV_EXT_CmdDisconnect();
        WDRV_DBG_INFORM_MESSAGE(("Lost connection permanently\r\n"));
    }
}

static void ConnectStatusUpdate(bool connected, uint8_t reason)
{
    static SYS_TMR_HANDLE timer = 0;
    uint16_t timeout;

    if (connected == true) {
        if (s_ConnectionStatus == WDRV_CSTATE_CONNECTION_TEMPORARY_LOST)
            g_wdrv_priv.isConnReestablished = true;
        s_ConnectionStatus = WDRV_CSTATE_CONNECTED_INFRASTRUCTURE;
    } else {
        switch (reason) {
        case WDRV_DISCONNECT_REASON_DISCONNECT_CMD:        // = 0x03,
        case WDRV_DISCONNECT_REASON_NO_NETWORK_AVAIL:      // = 0x01,
        case WDRV_DISCONNECT_REASON_BSS_DISCONNECTED:      // = 0x04,
        case WDRV_DISCONNECT_REASON_AUTH_FAILED:           // = 0x05,
        case WDRV_DISCONNECT_REASON_ASSOC_FAILED:          // = 0x06,
        case WDRV_DISCONNECT_REASON_NO_RESOURCES_AVAIL:    // = 0x07,
        case WDRV_DISCONNECT_REASON_CONNECTION_DENIED:     // = 0x08,
        case WDRV_DISCONNECT_REASON_INVALID_PROFILE:       // = 0x0A,
        case WDRV_DISCONNECT_REASON_PROFILE_MISMATCH:      // = 0x0C,
        case WDRV_DISCONNECT_REASON_CONNECTION_EVICTED:    // = 0x0D,
            if (g_wdrv_priv.isDisconnectRequested)
                DISCONNECT_DONE_NOTIFY();
            s_ConnectionStatus = WDRV_CSTATE_NOT_CONNECTED;
            break;
        case WDRV_DISCONNECT_REASON_LOST_LINK:             // = 0x02
            s_ConnectionStatus = WDRV_CSTATE_CONNECTION_TEMPORARY_LOST;

            if (timer != 0)
                SYS_TMR_CallbackStop(timer);

            timeout = SYS_TMR_TickCounterFrequencyGet() * 30; // wait for 30 seconds
            timer = SYS_TMR_CallbackSingle(timeout, 0, LinkDownTimeoutCallback);
            break;
        default:
            WDRV_ASSERT(false, "Undefined reason code\r\n");
            break;
        }
    }
}

WDRV_CONNECTION_STATES WDRV_ConnectStatus_Get(void)
{
    return s_ConnectionStatus;
}

bool WDRV_APHasClientsConnected(void)
{
    return g_wdrv_priv.clientCache.bitMap > 0;
}
//DOM-IGNORE-END
