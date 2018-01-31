/*******************************************************************************
  Wi-Fi MAC interface functions

  File Name:
    wdrv_mrf24wn_main.h

  Summary:
    Wi-Fi specific MAC function prototypes called by TCP/IP stack.

  Description:
    Wi-Fi specific MAC function prototypes called by TCP/IP stack.
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

#ifndef _WDRV_MRF24WN_MAIN_H
#define _WDRV_MRF24WN_MAIN_H

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************
#include "wdrv_mrf24wn_priv.h"

/**************************************************************************
  Summary:
    Default values for Wi-Fi Ad-Hoc settings

  Description:
    Wi-Fi AdHoc default settings

    These defines identify various default Wi-Fi AdHoc settings that can be
    used in the WDRV_ADHOC_NETWORK_CONTEXT structure.
*/
#define WDRV_DEFAULT_ADHOC_HIDDEN_SSID false
#define WDRV_DEFAULT_ADHOC_BEACON_PERIOD 100 // ms
#define WDRV_DEFAULT_ADHOC_MODE WDRV_ADHOC_CONNECT_THEN_START

// *****************************************************************************
/*  Wi-Fi PS-Poll Listen Interval default settings

  Summary:
    Default values for Wi-Fi PS PS-Poll Listen Interval settings

  Description
    These defines identify various default Wi-Fi PS-Poll settings that can
    be used in the WDRV_PS_POLL_CONTEXT structure.
*/
#define WDRV_DEFAULT_PS_LISTEN_INTERVAL ((uint16_t)1) // 100 ms multiplier, e.g. 1 * 100 ms = 100 ms
#define WDRV_DEFAULT_PS_DTIM_INTERVAL ((uint16_t)2) // number of beacon periods
#define WDRV_DEFAULT_PS_DTIM_ENABLED true // DTIM wake-up enabled (normally the case)

/************************************************************************
  Summary:
    Selection of different AdHoc connection modes

  Description:
    AdHoc Modes

    This enumeration identifies the AdHoc modes that can be selected when
    connecting in AdHoc mode.
*/
typedef enum adhocMode
{
    WDRV_ADHOC_CONNECT_THEN_START = 0, // try to connect existing AdHoc network, if not found then start network
    WDRV_ADHOC_CONNECT_ONLY = 1, // only connect to existing AdHoc network
    WDRV_ADHOC_START_ONLY = 2  // only start a new AdHoc network
} WDRV_ADHOC_MODES;

/*************************************************************************
  Summary:
    Selection of WPS Authorization types

  Description:
    Wi-Fi WPS authorization types

    This enumeration identifies the WPS authorization types
*/
typedef enum
{
    WDRV_WPS_AUTH_OPEN     = 0x01,
    WDRV_WPS_AUTH_WPA_PSK  = 0x02,
    WDRV_WPS_AUTH_SHARED   = 0x04,
    WDRV_WPS_AUTH_WPA      = 0x08,
    WDRV_WPS_AUTH_WPA2     = 0x10,
    WDRV_WPS_AUTH_WPA2_PSK = 0x20
} WDRV_WPS_AUTH_TYPES;

/************************************************************
ding types

  Description:
    Wi-Fi WPS encoding types

    This enumeration identifies the WPS encoding types
*/
typedef enum
{
    WDRV_WPS_ENC_NONE = 0x01,
    WDRV_WPS_ENC_WEP  = 0x02,
    WDRV_WPS_ENC_TKIP = 0x04,
    WDRV_ENC_AES      = 0x08
} WDRV_WPS_ENCODE_TYPES;

// *****************************************************************************
/*  Deauthorization/Disassociate Reason Codes

  Summary:
    Selection of different codes when a deauthorization or disassociation event has occurred.

  Description
    This enumeration identifies the reason codes for a connection lost due to a
    deauthorization or disassociation from the AP.
*/
typedef enum
{
    WDRV_UNSPECIFIED                    = 1,
    WDRV_REASON_PREV_AUTH_NOT_VALID     = 2,
    WDRV_DEAUTH_LEAVING                 = 3,
    WDRV_DISASSOC_DUE_TO_INACTIVITY     = 4,
    WDRV_DISASSOC_AP_BUSY               = 5,
    WDRV_CLASS2_FRAME_FROM_NONAUTH_STA  = 6,
    WDRV_CLASS3_FRAME_FROM_NONASSOC_STA = 7,
    WDRV_DISASSOC_STA_HAS_LEFT          = 8,
    WDRV_STA_REQ_ASSOC_WITHOUT_AUTH     = 9,
    WDRV_INVALID_IE                     = 13,
    WDRV_MIC_FAILURE                    = 14,
    WDRV_4WAY_HANDSHAKE_TIMEOUT         = 15,
    WDRV_GROUP_KEY_HANDSHAKE_TIMEOUT    = 16,
    WDRV_IE_DIFFERENT                   = 17,
    WDRV_INVALID_GROUP_CIPHER           = 18,
    WDRV_INVALID_PAIRWISE_CIPHER        = 19,
    WDRV_INVALID_AKMP                   = 20,
    WDRV_UNSUPP_RSN_VERSION             = 21,
    WDRV_INVALID_RSN_IE_CAP             = 22,
    WDRV_IEEE8021X_FAILED               = 23,
    WDRV_CIPHER_SUITE_REJECTED          = 24
} WDRV_REASON_CODES;

/***********************************************************
  Summary:
    Wi-Fi Power-Saving states

  Description:
    Wi-Fi Power-Saving states

    This enumeration identifies Wi-Fi Power-Saving states. See
    WDRV_PsPollEnable().
*/
typedef enum
{
    /* enable hibernate mode */
    WDRV_PS_HIBERNATE,
    WDRV_PS_SLEEP,
    WDRV_PS_ACTIVE
} WDRV_POWER_SAVE_STATES;

/***********************************************************
  Summary:
    Wi-Fi Connection states

  Description:
    Wi-Fi Connection States

    This enumeration identifies Wi-Fi Connection states. See
    WDRV_CLI_ConnectionStateGet().
 */
typedef enum
{
    /* No Wi-Fi connection exists */
    WDRV_CSTATE_NOT_CONNECTED               = 1,

    /* Wi-Fi connection in progress */
    WDRV_CSTATE_CONNECTION_IN_PROGRESS      = 2,

    /* Wi-Fi connected in infrastructure mode */
    WDRV_CSTATE_CONNECTED_INFRASTRUCTURE    = 3,

    /* Wi-Fi connected in Ad-Hoc mode */
    WDRV_CSTATE_CONNECTED_ADHOC             = 4,

    /* Wi-Fi in process of reconnecting */
    WDRV_CSTATE_RECONNECTION_IN_PROGRESS    = 5,

    /* Wi-Fi connection temporarily lost */
    WDRV_CSTATE_CONNECTION_TEMPORARY_LOST   = 6,

    /* Wi-Fi connection permanently lost */
    WDRV_CSTATE_CONNECTION_PERMANENTLY_LOST = 7
} WDRV_CONNECTION_STATES;

/***********************************************************
  Summary:
    Wi-Fi Soft AP events

  Description:
    Wi-Fi Soft AP events

    This enumeration identifies Wi-Fi Soft AP events.
 */
typedef enum
{
    WDRV_SOFTAP_EVENT_CONNECTED    = 0,
    WDRV_SOFTAP_EVENT_DISCONNECTED = 1
} WDRV_SOFT_AP_STATES;

/***********************************************************
  Summary:
    Wi-Fi Soft AP event reason codes

  Description:
    Wi-Fi Soft AP event reason codes

    This enumeration identifies Wi-Fi Soft AP events.
 */
typedef enum
{
    WDRV_SOFTAP_EVENT_LINK_LOST       = 0,
    WDRV_SOFTAP_EVENT_RECEIVED_DEAUTH = 1
} WDRV_SOFT_AP_EVENT_REASON_CODES ;

typedef enum
{
    WDRV_DISCONNECT_REASON_NO_NETWORK_AVAIL      = 0x01,
    WDRV_DISCONNECT_REASON_LOST_LINK             = 0x02,
    WDRV_DISCONNECT_REASON_DISCONNECT_CMD        = 0x03,
    WDRV_DISCONNECT_REASON_BSS_DISCONNECTED      = 0x04,
    WDRV_DISCONNECT_REASON_AUTH_FAILED           = 0x05,
    WDRV_DISCONNECT_REASON_ASSOC_FAILED          = 0x06,
    WDRV_DISCONNECT_REASON_NO_RESOURCES_AVAIL    = 0x07,
    WDRV_DISCONNECT_REASON_CONNECTION_DENIED     = 0x08,
    WDRV_DISCONNECT_REASON_INVALID_PROFILE       = 0x0A,
    WDRV_DISCONNECT_REASON_PROFILE_MISMATCH      = 0x0C,
    WDRV_DISCONNECT_REASON_CONNECTION_EVICTED    = 0x0d
} WDRV_DISCONNECTION_REASON;

bool isLinkUp();
void WDRV_TrafficEventInit(TCPIP_MAC_EventF eventF, const void *eventParam);
void WDRV_TrafficEventDeinit(void);
void WDRV_TrafficEventReq(uint16_t event, uint16_t eventInfo);
bool WDRV_TrafficEventMask(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents, bool enable);
bool WDRV_TrafficEventAck(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents);
TCPIP_MAC_EVENT WDRV_TrafficEventGet(TCPIP_MAC_HANDLE hMac);
void WDRV_AllEventClear(void);
void WDRV_EventSet(uint8_t event);

void ProceedConnectEventCB(uint32_t connected, uint8_t devID, uint8_t *mac, bool macConn, uint8_t reason);
TCPIP_MAC_RES WDRV_Connect(void);
WDRV_CONNECTION_STATES WDRV_ConnectStatus_Get(void);
void WPSDoneCB(void);
bool WDRV_APHasClientsConnected(void);

extern WDRV_SCAN_STATUS g_scanStatus;
extern WDRV_SCAN_RESULT g_scanResults[];

#endif /* _WDRV_MRF24WN_MAIN_H */

// DOM-IGNORE-END
