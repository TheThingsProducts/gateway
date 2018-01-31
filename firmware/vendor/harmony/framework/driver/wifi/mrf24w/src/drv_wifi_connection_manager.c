/*******************************************************************************
  MRF24WG Driver Connection Manager

  File Name:
    drv_wifi_connection_manager.c

  Summary:
    MRF24WG Driver Connection Manager

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

#include "drv_wifi_priv.h"

#include "drv_wifi_config_data.h"
#include "drv_wifi_debug_output.h"
#include "drv_wifi_mgmt_msg.h"

static bool s_LogicalConnection = false;

static bool isDisconnectSafe(void);

/*******************************************************************************
  Function:
    void DRV_WIFI_Connect(void)

  Summary:
    Commands the MRF24W to start a connection.

  Description:
    Directs the Connection Manager to scan for and connect to a WiFi network.
    This function does not wait until the connection attempt is successful, but
    returns immediately.

  Precondition:
    MACInit must be called first.

  Parameters:
    None
  Returns:
    None.

  Remarks:
    None.
  *****************************************************************************/
void DRV_WIFI_Connect(void)
{
    uint8_t hdrBuf[4];

    /* write out header portion of msg (which is whole msg, there is no data) */
    hdrBuf[0] = WF_MGMT_REQUEST_TYPE;   /* indicate this is a mgmt msg */
    hdrBuf[1] = WF_CM_CONNECT_SUBYTPE;  /* mgmt request subtype */
    hdrBuf[2] = CPID;
    hdrBuf[3] = 0;

    SendMgmtMsg(hdrBuf, sizeof(hdrBuf), NULL, 0);

    /* wait for mgmt response, free it after it comes in (no data needed) */
    WaitForMgmtResponse(WF_CM_CONNECT_SUBYTPE, FREE_MGMT_BUFFER);
}

static bool isDisconnectSafe(void)
{
    uint8_t connectionState;

    DRV_WIFI_ConnectionStateGet(&connectionState);
    if (connectionState == DRV_WIFI_CSTATE_CONNECTED_INFRASTRUCTURE ||
        connectionState == DRV_WIFI_CSTATE_CONNECTED_ADHOC)
    {
        return true;
    }

    return false;
}

// *****************************************************************************
/* Function:
    uint16_t DRV_WIFI_Disconnect(void);

  Summary:
    Directs the MRF24WG to disconnect from a WiFi network.

  Description:
    This function causes the MRF24WG to disconnect from a WiFi network.  No event
    is generated when a connection is terminated via the function call.

  Parameters:
    None

 Returns:
    DRV_WIFI_SUCCESS or DRV_WIFI_ERROR_DISCONNECT_FAILED

 Remarks:
    Disconnection can work only in the connected state.
    To use this API safely, we recommend to disable module FW connection
    manager by setting #define DRV_WIFI_MODULE_CONNECTION_MANAGER == DRV_WIFI_DISABLED
    in system_config.h

*****************************************************************************/
uint16_t DRV_WIFI_Disconnect(void)
{
    uint8_t hdrBuf[2];

    /* check if we can call disconnect. Disconnect can work only in the connected state */
    if (!isDisconnectSafe())
        return DRV_WIFI_ERROR_DISCONNECT_FAILED;

    hdrBuf[0] = WF_MGMT_REQUEST_TYPE;
    hdrBuf[1] = WF_CM_DISCONNECT_SUBYTPE;

    SendMgmtMsg(hdrBuf, sizeof(hdrBuf), NULL, 0);

    /* wait for mgmt response, free it after it comes in (no data needed) */
    WaitForMgmtResponse(WF_CM_DISCONNECT_SUBYTPE, FREE_MGMT_BUFFER);

    return DRV_WIFI_SUCCESS;
}

// *****************************************************************************
/* Function:
    void DRV_WIFI_ConnectionStateGet(uint8_t *p_state);

  Summary:
    Gets the current WiFi connection state

  Description:
    This function gets the current WiFi connection state.

    <table>
    *p_state                                        Description
    --------                                        -----------
    DRV_WIFI_CSTATE_NOT_CONNECTED                   No WiFi connection exists
    DRV_WIFI_CSTATE_CONNECTION_IN_PROGRESS          WiFi connection in progress
    DRV_WIFI_CSTATE_CONNECTED_INFRASTRUCTURE        WiFi connected in infrastructure mode
    DRV_WIFI_CSTATE_CONNECTED_ADHOC                 WiFi connected in adhoc mode
    DRV_WIFI_CSTATE_RECONNECTION_IN_PROGRESS        WiFi in process of reconnecting
    DRV_WIFI_CSTATE_CONNECTION_PERMANENTLY_LOST     WiFi connection permanently lost
    </table>

  Precondition:
    WiFi initialization must be complete

  Parameters:
    p_state - pointer to where state is written (see description)

 Returns:
    None

  Example:
    <code>
        uint8_t state;

        DRV_WIFI_ConnectionStateGet(&state);
    </code>

  Remarks:
    None
*/
void DRV_WIFI_ConnectionStateGet(uint8_t *p_state)
{
    uint8_t hdrBuf[2];
    uint8_t msgData[2];

    hdrBuf[0] = WF_MGMT_REQUEST_TYPE;
    hdrBuf[1] = WF_CM_GET_CONNECTION_STATUS_SUBYTPE;

    SendMgmtMsg(hdrBuf, sizeof(hdrBuf), NULL, 0);

    /* wait for mgmt response, read data, free after read */
    WaitForMgmtResponseAndReadData(WF_CM_GET_CONNECTION_STATUS_SUBYTPE,
                                   sizeof(msgData),                 /* num data bytes to read          */
                                   MGMT_RESP_1ST_DATA_BYTE_INDEX,   /* only used if num data bytes > 0 */
                                   msgData);                        /* only used if num data bytes > 0 */

    *p_state = msgData[0];  /* connection state */

    /* in Soft AP mode, when connected, the return code of MRF24W is DRV_WIFI_CSTATE_CONNECTION_IN_PROGRESS */
    if ((*p_state == DRV_WIFI_CSTATE_CONNECTED_INFRASTRUCTURE) ||
        (*p_state == DRV_WIFI_CSTATE_CONNECTED_ADHOC) ||
        (*p_state == DRV_WIFI_CSTATE_CONNECTION_IN_PROGRESS &&
        DRV_WIFI_CONFIG_PARAMS(networkType) == DRV_WIFI_NETWORK_TYPE_SOFT_AP))
    {
        SetLogicalConnectionState(true);
    }
    else
    {
        SetLogicalConnectionState(false);
    }
}

/*******************************************************************************
  Function:
    bool isLinkUp()

  Summary:
    Query the connection status of the MRF24W.

  Description:
    Determine the fine granularity status of the connection state of the
    MRF24W.

  Precondition:
    MACInit must be called first.

  Parameters:
    None.

  Returns:
    true if the MRF24W is either connected or attempting to connect.
    false for all other conditions.

  Remarks:
    None.
  *****************************************************************************/
bool isLinkUp(void)
{
    return s_LogicalConnection;
}

/*******************************************************************************
  Function:
    void SetLogicalConnectionState(bool state)

  Summary:
    Sets the logical connection state.

  Description:
    Logically, if the MRF24W is either connected or trying to connect, then
    it is "connected".  For all other scenarios, the MRF24W is "not
    connected".

  Precondition:
    MACInit must be called first.

  Parameters:
    state - Current logical connection state of the MRF24W.

  Returns:
    None.

  Remarks:
    None.
  *****************************************************************************/
void SetLogicalConnectionState(bool state)
{
    s_LogicalConnection = state;
}

// *****************************************************************************
/* Function:
    void DRV_WIFI_ConnectContextGet(DRV_WIFI_CONNECTION_CONTEXT *p_ctx);

  Summary:
    Gets the current WiFi connection context

  Description:
    This function gets the current WiFi connection context.

  Precondition:
    WiFi initialization must be complete

  Parameters:
    p_ctx - pointer to where connection context is written.  See
            DRV_WIFI_CONNECTION_CONTEXT structure.

 Returns:
    None

  Example:
    <code>
        DRV_WIFI_CONNECTION_CONTEXT ctx;

        DRV_WIFI_ConnectContextGet(&ctx);
    </code>

  Remarks:
    None
*/
void DRV_WIFI_ConnectContextGet(DRV_WIFI_CONNECTION_CONTEXT *p_ctx)
{
    GetParamMsgSend(PARAM_CONNECT_CONTEXT, (uint8_t *)p_ctx, sizeof(*p_ctx));
}

/*******************************************************************************
  Function:
    TCPIP_MAC_RES DRV_WIFI_KeyDerive(uint8_t key_len, uint8_t *key, uint8_t ssid_len, uint8_t *ssid)

  Summary:
    Derives key from passphrase.

  Description:
     This function derives key from passphrase.

  Precondition:
    MACInit must be called first.

  Parameters:
    key_len: key length
    key: passphrase as an input. key as an output
    ssid_len: ssid length
    ssid: ssid

  Returns:
    TCPIP_MAC_RES.

  Remarks:
     None.
 *****************************************************************************/
TCPIP_MAC_RES DRV_WIFI_KeyDerive(uint8_t key_len, uint8_t *key, uint8_t ssid_len, uint8_t *ssid)
{
    static uint8_t psk[32];
    TCPIP_MAC_RES RetCode;

    DRV_WIFI_ASSERT(key_len >= 8 && key_len <= 63, "WPA Passphrasee should contain 8 to 63 characters");

    key[key_len] = '\0'; // key is an array of 64 elements, max input key_len is 63, so this operation is safe
    RetCode = pbkdf2_sha1((const char *)key, (const char *)ssid, ssid_len, 4096, (uint8_t *)psk, 32);
    if (RetCode == TCPIP_MAC_RES_OK)
    {
        memcpy(key, psk, 32);
    }
    return RetCode;
}

void WpaKeySet(void)
{
    if (TCPIP_MAC_RES_OK == DRV_WIFI_KeyDerive(g_passphraseReady.keyLen, g_passphraseReady.key,
        g_passphraseReady.ssidLen, g_passphraseReady.ssid))
    {
        DRV_WIFI_SetPSK(g_passphraseReady.key);
    }
    else
    {
        DRV_WIFI_PendingEventSet(WPS_CONNECT_PENDING);
    }
}

//DOM-IGNORE-END
