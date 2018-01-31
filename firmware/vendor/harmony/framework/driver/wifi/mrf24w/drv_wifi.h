/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    drv_wifi.h

  Summary:
    MRF24WG Wi-Fi Driver Interface File

  Description:
    Contains all data types, define constants, and function prototypes for
    interfacing to the MRF24WG Wi-Fi driver.
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
// DOM-IGNORE-END

#ifndef _DRV_WIFI_H
#define _DRV_WIFI_H

#include <stdint.h>
#include <stdbool.h>

#if defined(DRV_WIFI_USE_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#endif

// DOM-IGNORE-BEGIN
#ifdef __cplusplus // Provide C++ Compatibility
    extern "C" {
#endif
// DOM-IGNORE-END

#if !defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    #error "TCPIP_STACK_USE_EVENT_NOTIFICATION must be defined for Wi-Fi demos"
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Data Types and Constants
// *****************************************************************************
// *****************************************************************************

// Do not make this an enumerated type!
#define DRV_WIFI_ENABLED      (1)
#define DRV_WIFI_DISABLED     (0)

#define DRV_WIFI_BSSID_LENGTH                   (6)
#define DRV_WIFI_MAX_SSID_LENGTH                (32)
#define DRV_WIFI_RETRY_FOREVER                  (255)
#define DRV_WIFI_RETRY_ADHOC                    (3)

#define DRV_WIFI_MAX_CHANNEL_LIST_LENGTH        (14)
#define DRV_WIFI_MAX_SECURITY_KEY_LENGTH        (64)

#define DRV_WIFI_RTS_THRESHOLD_MAX              (2347)  /* maximum RTS threshold size in bytes */
#define DRV_WIFI_MAX_NUM_RATES                  (8)

/* Key size defines */
#define DRV_WIFI_MIN_WPA_PASS_PHRASE_LENGTH     (8)     // must exclude string terminator
#define DRV_WIFI_MAX_WPA_PASS_PHRASE_LENGTH     (63)    // must exclude string terminator
#define DRV_WIFI_WPA_KEY_LENGTH                 (32)

// WEP Key Lengths
#define DRV_WIFI_WEP40_KEY_LENGTH               (20)    // 4 keys of 5 bytes each
#define DRV_WIFI_WEP104_KEY_LENGTH              (52)    // 4 keys of 13 bytes each
#define DRV_WIFI_MAX_WEP_KEY_LENGTH             (DRV_WIFI_WEP104_KEY_LENGTH)

// WPS PIN Length
#define DRV_WIFI_WPS_PIN_LENGTH                 8       // 7 digits + checksum byte

// eventInfo define for DRV_WIFI_ProcessEvent() when no additional info is supplied
#define DRV_WIFI_NO_ADDITIONAL_INFO             ((uint16_t)0xffff)

// *****************************************************************************
/*  WEP Key Types

  Summary:
    Selections for WEP key type when using WEP security.

  Description
    This enumeration identifies the choices for the WEP key type when using
    WEP security.  The recommended key type (and default) is Open key.
*/
typedef enum
{
    /* use WEP shared key */
    DRV_WIFI_SECURITY_WEP_SHAREDKEY = 0,

    /* use WEP open key (default) */
    DRV_WIFI_SECURITY_WEP_OPENKEY = 1
} DRV_WIFI_WEP_KEY_TYPE;

// *****************************************************************************
/*  Management Message Error Codes

  Summary:
    Error codes returned when a management message is sent to the MRF24WG
    module.

  Description
    This enumeration identifies the errors that can occur when a DRV_WIFI API
    function call results in a management message being sent, via SPI, to the
    MRF24WG module.
*/
typedef enum
{
    DRV_WIFI_SUCCESS                                            = 1,
    DRV_WIFI_ERROR_INVALID_SUBTYPE                              = 2,
    DRV_WIFI_ERROR_OPERATION_CANCELLED                          = 3,
    DRV_WIFI_ERROR_FRAME_END_OF_LINE_OCCURRED                   = 4,
    DRV_WIFI_ERROR_FRAME_RETRY_LIMIT_EXCEEDED                   = 5,
    DRV_WIFI_ERROR_EXPECTED_BSS_VALUE_NOT_IN_FRAME              = 6,
    DRV_WIFI_ERROR_FRAME_SIZE_EXCEEDS_BUFFER_SIZE               = 7,
    DRV_WIFI_ERROR_FRAME_ENCRYPT_FAILED                         = 8,
    DRV_WIFI_ERROR_INVALID_PARAM                                = 9,
    DRV_WIFI_ERROR_AUTH_REQ_ISSUED_WHILE_IN_AUTH_STATE          = 10,
    DRV_WIFI_ERROR_ASSOC_REQ_ISSUED_WHILE_IN_ASSOC_STATE        = 11,
    DRV_WIFI_ERROR_INSUFFICIENT_RESOURCES                       = 12,
    DRV_WIFI_ERROR_TIMEOUT_OCCURRED                             = 13,
    DRV_WIFI_ERROR_BAD_EXCHANGE_ENCOUNTERED_IN_FRAME_RECEPTION  = 14,
    DRV_WIFI_ERROR_AUTH_REQUEST_REFUSED                         = 15,
    DRV_WIFI_ERROR_ASSOCIATION_REQUEST_REFUSED                  = 16,
    DRV_WIFI_ERROR_PRIOR_MGMT_REQUEST_IN_PROGRESS               = 17,
    DRV_WIFI_ERROR_NOT_IN_JOINED_STATE                          = 18,
    DRV_WIFI_ERROR_NOT_IN_ASSOCIATED_STATE                      = 19,
    DRV_WIFI_ERROR_NOT_IN_AUTHENTICATED_STATE                   = 20,
    DRV_WIFI_ERROR_SUPPLICANT_FAILED                            = 21,
    DRV_WIFI_ERROR_UNSUPPORTED_FEATURE                          = 22,
    DRV_WIFI_ERROR_REQUEST_OUT_OF_SYNC                          = 23,
    DRV_WIFI_ERROR_CP_INVALID_ELEMENT_TYPE                      = 24,
    DRV_WIFI_ERROR_CP_INVALID_PROFILE_ID                        = 25,
    DRV_WIFI_ERROR_CP_INVALID_DATA_LENGTH                       = 26,
    DRV_WIFI_ERROR_CP_INVALID_SSID_LENGTH                       = 27,
    DRV_WIFI_ERROR_CP_INVALID_SECURITY_TYPE                     = 28,
    DRV_WIFI_ERROR_CP_INVALID_SECURITY_KEY_LENGTH               = 29,
    DRV_WIFI_ERROR_CP_INVALID_WEP_KEY_ID                        = 30,
    DRV_WIFI_ERROR_CP_INVALID_NETWORK_TYPE                      = 31,
    DRV_WIFI_ERROR_CP_INVALID_ADHOC_MODE                        = 32,
    DRV_WIFI_ERROR_CP_INVALID_SCAN_TYPE                         = 33,
    DRV_WIFI_ERROR_CP_INVALID_CP_LIST                           = 34,
    DRV_WIFI_ERROR_CP_INVALID_CHANNEL_LIST_LENGTH               = 35,
    DRV_WIFI_ERROR_NOT_CONNECTED                                = 36,
    DRV_WIFI_ERROR_ALREADY_CONNECTING                           = 37,

    /* Disconnect failed. Disconnect is allowed only when module is in connected state. */
    DRV_WIFI_ERROR_DISCONNECT_FAILED                            = 38,

    /* No stored scan results. */
    DRV_WIFI_ERROR_NO_STORED_BSS_DESCRIPTOR                     = 39,
    DRV_WIFI_ERROR_INVALID_MAX_POWER                            = 40,
    DRV_WIFI_ERROR_CONNECTION_TERMINATED                        = 41,
    DRV_WIFI_ERROR_HOST_SCAN_NOT_ALLOWED                        = 42,   // Host Scan Failed. Host scan is
                                                                        // allowed only in idle or
                                                                        // connected state
    DRV_WIFI_ERROR_INVALID_WPS_PIN                              = 44    // WPS pin was invalid.
} DRV_WIFI_MGMT_ERRORS;

typedef enum
{
    DRV_WIFI_ERROR_IN_HIBERNATE_MODE                            = 100   // invalid operation while MRF24WG
                                                                        // is in hibernate mode
} DRV_WIFI_GENERAL_ERRORS;

// *****************************************************************************
/*
  Summary:
    Selections for Wi-Fi TX Mode

  Description:
    TX Modes

    This enumeration identifies the choices the MRF24WG TX mode. It is
    recommended to use the DRV_WIFI_TXMODE_G_RATES for best performance.
    See DRV_WIFI_TxModeSet.
*/
typedef enum
{
    /* Use 802.11 'g' rates */
    DRV_WIFI_TXMODE_G_RATES         = 0,

    /* Use only 802.11 'b' rates */
    DRV_WIFI_TXMODE_B_RATES         = 1,

    /* Use only 1 and 2 Mbps rates */
    DRV_WIFI_TXMODE_LEGACY_RATES    = 2
} DRV_WIFI_TX_MODES;

// *****************************************************************************
/*
  Summary:
    Selections for software Multicast filter IDs.

  Description:
    Multicast Filter IDs

    This enumeration identifies the multicast filters that can be selected.
    See DRV_WIFI_MulticastFilterSet.
*/
typedef enum
{
    DRV_WIFI_MULTICAST_FILTER_1       = 4,
    DRV_WIFI_MULTICAST_FILTER_2       = 5,
    DRV_WIFI_MULTICAST_FILTER_3       = 6,
    DRV_WIFI_MULTICAST_FILTER_4       = 7,
    DRV_WIFI_MULTICAST_FILTER_5       = 8,
    DRV_WIFI_MULTICAST_FILTER_6       = 9,
    DRV_WIFI_MULTICAST_FILTER_7       = 10,
    DRV_WIFI_MULTICAST_FILTER_8       = 11,
    DRV_WIFI_MULTICAST_FILTER_9       = 12,
    DRV_WIFI_MULTICAST_FILTER_10      = 13,
    DRV_WIFI_MULTICAST_FILTER_11      = 14,
    DRV_WIFI_MULTICAST_FILTER_12      = 15,
    DRV_WIFI_MULTICAST_FILTER_13      = 16,
    DRV_WIFI_MULTICAST_FILTER_14      = 17,
    DRV_WIFI_MULTICAST_FILTER_15      = 18,
    DRV_WIFI_MULTICAST_FILTER_16      = 19
} DRV_WIFI_MULTICAST_FILTER_IDS;

// *****************************************************************************
/*  Multicast Filter Modes

  Summary:
    Selections for Software Multicast Filters.

  Description
    This enumeration identifies the mode of multicast filters that can be selected.
    See DRV_WIFI_MulticastFilterSet().
*/
typedef enum
{
    /* Discard all received multicast messages. */
    DRV_WIFI_MULTICAST_DISABLE_ALL = 0,

    /* Forward all multicast messages to host MCU. */
    DRV_WIFI_MULTICAST_ENABLE_ALL  = 1,

    /* Use the MAC filtering capability for multicast messages. */
    DRV_WIFI_MULTICAST_USE_FILTERS = 2
} DRV_WIFI_MULTICAST_FILTERS;

#define DRV_WIFI_DEAUTH_REASONCODE_MASK      ((uint8_t)0x80)
#define DRV_WIFI_DISASSOC_REASONCODE_MASK    ((uint8_t)0x40)

// *****************************************************************************
/*
  Summary:
    Selections for events that can occur.

  Description:
    Wi-Fi Events

    This enumeration identifies the Wi-Fi events that can occur and will be
    sent to DRV_WIFI_ProcessEvent.
*/
typedef enum
{
    /* No event has occurred */
    DRV_WIFI_EVENT_NONE                           = 0,

    /* Connection attempt to network successful */
    DRV_WIFI_EVENT_CONNECTION_SUCCESSFUL          = 1,

    /* Connection attempt failed */
    DRV_WIFI_EVENT_CONNECTION_FAILED              = 2,

    /* Connection lost; MRF24WG attempting to reconnect  */
    DRV_WIFI_EVENT_CONNECTION_TEMPORARILY_LOST    = 3,

    /* Connection lost; MRF24WG no longer trying to connect */
    DRV_WIFI_EVENT_CONNECTION_PERMANENTLY_LOST    = 4,

    /* Connection has been reestablished */
    DRV_WIFI_EVENT_CONNECTION_REESTABLISHED       = 5,

    /* Update to FLASH successful */
    DRV_WIFI_EVENT_FLASH_UPDATE_SUCCESSFUL        = 6,

    /* Update to FLASH failed */
    DRV_WIFI_EVENT_FLASH_UPDATE_FAILED            = 7,

    /* Key calculation is required */
    DRV_WIFI_EVENT_KEY_CALCULATION_REQUEST        = 8,

    /* Invalid WPS pin was entered */
    DRV_WIFI_EVENT_INVALID_WPS_PIN                = 9,

    /* Scan results are ready */
    DRV_WIFI_EVENT_SCAN_RESULTS_READY             = 10,

    /* IE data ready */
    DRV_WIFI_EVENT_IE_RESULTS_READY               = 11,

    /* Client connection events */
    DRV_WIFI_EVENT_SOFT_AP                        = 12,

    /* Disconnect done event */
    DRV_WIFI_EVENT_DISCONNECT_DONE                = 13,

    /* Wi-Fi update event occurred */
    DRV_WIFI_EVENT_UPDATE                         = 14,

    /* Wi-Fi error event occurred */
    DRV_WIFI_EVENT_ERROR                          = 15

} DRV_WIFI_EVENTS;

// *****************************************************************************
/*
  Summary:
    Default values for Wi-Fi scan context

  Description:
    Wi-Fi Scan Context default settings

    These defines identify the default Wi-Fi scan context values that
    can be used in the DRV_WIFI_SCAN_CONTEXT structure.
*/
#define DRV_WIFI_DEFAULT_SCAN_COUNT                   (1)
#define DRV_WIFI_DEFAULT_SCAN_MIN_CHANNEL_TIME        (200) // ms
#define DRV_WIFI_DEFAULT_SCAN_MAX_CHANNEL_TIME        (400) // ms
#define DRV_WIFI_DEFAULT_SCAN_PROBE_DELAY             (20)  // us

// *****************************************************************************
/*  Wi-Fi PS-Poll Listen Interval Default Settings

  Summary:
    Default values for Wi-Fi PS-Poll Listen Interval settings.

  Description
    These defines identify various default Wi-Fi PS-Poll settings that can
    be used in the DRV_WIFI_PS_POLL_CONTEXT structure.
*/
#define DRV_WIFI_DEFAULT_PS_LISTEN_INTERVAL ((uint16_t)1)   // 100 ms multiplier, e.g., 1 * 100 ms = 100 ms
#define DRV_WIFI_DEFAULT_PS_DTIM_INTERVAL   ((uint16_t)2)   // number of beacon periods
#define DRV_WIFI_DEFAULT_PS_DTIM_ENABLED    true            // DTIM wake-up enabled (normally the case)

// *****************************************************************************
/*
  Summary:
    Default values for Wi-Fi Ad-Hoc settings.

  Description:
    Wi-Fi Ad-Hoc default settings.

    These defines identify various default Wi-Fi Ad-Hoc settings that can be
    used in the DRV_WIFI_ADHOC_NETWORK_CONTEXT structure.
*/
#define DRV_WIFI_DEFAULT_ADHOC_HIDDEN_SSID      false
#define DRV_WIFI_DEFAULT_ADHOC_BEACON_PERIOD    (100)   // ms
#define DRV_WIFI_DEFAULT_ADHOC_MODE             DRV_WIFI_ADHOC_CONNECT_THEN_START

// *****************************************************************************
/*
  Summary:
    Default values for Wi-Fi Soft AP settings.

  Description:
    Wi-Fi Soft AP default settings.

    These defines identify various default Wi-Fi Soft AP settings that can be
    used in the DRV_WIFI_SOFTAP_NETWORK_CONTEXT structure.
*/
#define DRV_WIFI_DEFAULT_SOFTAP_HIDDEN_SSID false
//#define DRV_WIFI_DEFAULT_SOFTAP_BEACON_PERIOD (100) // ms
//#define DRV_WIFI_DEFAULT_SOFTAP_MODE DRV_WIFI_SOFTAP_CONNECT_THEN_START

// see DRV_WIFI_SecurityWepSet() and DRV_WIFI_WEP_CONTEXT
#define DRV_WIFI_DEFAULT_WEP_KEY_INDEX 0
#define DRV_WIFI_DEFAULT_WEP_KEY_TYPE DRV_WIFI_SECURITY_WEP_OPENKEY

// *****************************************************************************
/*
  Summary:
    Selection of different Ad-Hoc connection modes.

  Description:
    Ad-Hoc Modes

    This enumeration identifies the Ad-Hoc modes that can be selected when
    connecting in Ad-Hoc mode.
*/
typedef enum adhocMode
{
    DRV_WIFI_ADHOC_CONNECT_THEN_START = 0, // try to connect existing Ad-Hoc network,
                                           // if not found then start network
    DRV_WIFI_ADHOC_CONNECT_ONLY = 1,       // only connect to existing Ad-Hoc network
    DRV_WIFI_ADHOC_START_ONLY = 2          // only start a new Ad-Hoc network
} DRV_WIFI_ADHOC_MODES;

// *****************************************************************************
/*
  Summary:
    Selection of different Wi-Fi network types.

  Description:
    Wi-Fi Network Types

    This enumeration identifies the Wi-Fi network types that can be
    selected. Do NOT make these an enumerated type as they are used as a
    compile switch.
*/
#define DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE          (1)
#define DRV_WIFI_NETWORK_TYPE_ADHOC                   (2)
#define DRV_WIFI_NETWORK_TYPE_SOFT_AP                 (4)

// *****************************************************************************
/*
  Summary:
    Selection of different Wi-Fi security types

  Description:
    Wi-Fi Security Types

    This enumeration identifies the Wi-Fi security types that can be
    selected. Do NOT make these an enumerated type as they are used as a
    compile switch.
*/
#define DRV_WIFI_SECURITY_OPEN                        (0)
#define DRV_WIFI_SECURITY_WEP_40                      (1)
#define DRV_WIFI_SECURITY_WEP_104                     (2)
#define DRV_WIFI_SECURITY_WPA_WITH_KEY                (3)
#define DRV_WIFI_SECURITY_WPA_WITH_PASS_PHRASE        (4)
#define DRV_WIFI_SECURITY_WPA2_WITH_KEY               (5)
#define DRV_WIFI_SECURITY_WPA2_WITH_PASS_PHRASE       (6)
#define DRV_WIFI_SECURITY_WPA_AUTO_WITH_KEY           (7)
#define DRV_WIFI_SECURITY_WPA_AUTO_WITH_PASS_PHRASE   (8)
#define DRV_WIFI_SECURITY_WPS_PUSH_BUTTON             (9)
#define DRV_WIFI_SECURITY_WPS_PIN                     (10)

// *****************************************************************************
/*
  Summary:
    apConfig bit

  Description:
    apConfig bit
*/
#define DRV_WIFI_APCONFIG_BIT_PRIVACY                 (0x10)
#define DRV_WIFI_APCONFIG_BIT_PREAMBLE_LONG           (0x20)
#define DRV_WIFI_APCONFIG_BIT_WPA                     (0x40)
#define DRV_WIFI_APCONFIG_BIT_WPA2                    (0x80)

// *****************************************************************************
/*
  Summary:
    Selection of different Wi-Fi scan types.

  Description:
    Wi-Fi Scan Types

    This enumeration identifies the Wi-Fi scan types that can be selected.
*/
typedef enum
{
    DRV_WIFI_ACTIVE_SCAN = 1,
    DRV_WIFI_PASSIVE_SCAN = 2
} DRV_WIFI_SCAN_TYPES;

// *****************************************************************************
/*
  Summary:
    Selection of different reconnection modes.

  Description:
    Wi-Fi Reconnect Modes

    This enumeration identifies the reconnection modes that can be used in
    DRV_WIFI_ReconnectModeSet.
*/
typedef enum
{
    DRV_WIFI_DO_NOT_ATTEMPT_TO_RECONNECT = 0,
    DRV_WIFI_ATTEMPT_TO_RECONNECT = 1
} DRV_WIFI_RECONNECT_MODES;

// *****************************************************************************
/*
  Summary:
    Selection of WPS Authorization Types

  Description:
    Wi-Fi WPS authorization types.

    This enumeration identifies the WPS authorization types.
*/
typedef enum
{
    DRV_WIFI_WPS_AUTH_OPEN       = 0x01,
    DRV_WIFI_WPS_AUTH_WPA_PSK    = 0x02,
    DRV_WIFI_WPS_AUTH_SHARED     = 0x04,
    DRV_WIFI_WPS_AUTH_WPA        = 0x08,
    DRV_WIFI_WPS_AUTH_WPA2       = 0x10,
    DRV_WIFI_WPS_AUTH_WPA2_PSK   = 0x20
} DRV_WIFI_WPS_AUTH_TYPES;

// *****************************************************************************
/*
  Summary:
    Selection of WPS Encoding Types

  Description:
    Wi-Fi WPS encoding types.

    This enumeration identifies the WPS encoding types.
*/
typedef enum
{
    DRV_WIFI_WPS_ENC_NONE        = 0x01,
    DRV_WIFI_WPS_ENC_WEP         = 0x02,
    DRV_WIFI_WPS_ENC_TKIP        = 0x04,
    DRV_WIFI_ENC_AES             = 0x08
} DRV_WIFI_WPS_ENCODE_TYPES;

// *****************************************************************************
/*  EventInfo Types

  Summary:
    Selection of different EventInfo types.

  Description
    This enumeration identifies the eventInfo types used in DRV_WIFI_ProcessEvent,
    case DRV_WIFI_EVENT_CONNECTION_FAILED.
*/
typedef enum
{
    DRV_WIFI_JOIN_FAILURE                         = 2,
    DRV_WIFI_AUTHENTICATION_FAILURE               = 3,
    DRV_WIFI_ASSOCIATION_FAILURE                  = 4,
    DRV_WIFI_WEP_HANDSHAKE_FAILURE                = 5,
    DRV_WIFI_PSK_CALCULATION_FAILURE              = 6,
    DRV_WIFI_PSK_HANDSHAKE_FAILURE                = 7,
    DRV_WIFI_ADHOC_JOIN_FAILURE                   = 8,
    DRV_WIFI_SECURITY_MISMATCH_FAILURE            = 9,
    DRV_WIFI_NO_SUITABLE_AP_FOUND_FAILURE         = 10,
    DRV_WIFI_RETRY_FOREVER_NOT_SUPPORTED_FAILURE  = 11,
    DRV_WIFI_LINK_LOST                            = 12,
    DRV_WIFI_TKIP_MIC_FAILURE                     = 13,
    DRV_WIFI_RSN_MIXED_MODE_NOT_SUPPORTED         = 14,
    DRV_WIFI_RECV_DEAUTH                          = 15,
    DRV_WIFI_RECV_DISASSOC                        = 16,
    DRV_WIFI_WPS_FAILURE                          = 17,
    DRV_WIFI_LINK_DOWN                            = 19
} DRV_WIFI_EVENT_INFO;

// *****************************************************************************
/*  Deauthorization/Disassociate Reason Codes

  Summary:
    Selection of different codes when a deauthorization or disassociation event
    has occurred.

  Description
    This enumeration identifies the reason codes for a connection lost due to a
    deauthorization or disassociation from the AP.
*/
typedef enum
{
    DRV_WIFI_UNSPECIFIED                          = 1,
    DRV_WIFI_REASON_PREV_AUTH_NOT_VALID           = 2,
    DRV_WIFI_DEAUTH_LEAVING                       = 3,
    DRV_WIFI_DISASSOC_DUE_TO_INACTIVITY           = 4,
    DRV_WIFI_DISASSOC_AP_BUSY                     = 5,
    DRV_WIFI_CLASS2_FRAME_FROM_NONAUTH_STA        = 6,
    DRV_WIFI_CLASS3_FRAME_FROM_NONASSOC_STA       = 7,
    DRV_WIFI_DISASSOC_STA_HAS_LEFT                = 8,
    DRV_WIFI_STA_REQ_ASSOC_WITHOUT_AUTH           = 9,
    DRV_WIFI_INVALID_IE                           = 13,
    DRV_WIFI_MIC_FAILURE                          = 14,
    DRV_WIFI_4WAY_HANDSHAKE_TIMEOUT               = 15,
    DRV_WIFI_GROUP_KEY_HANDSHAKE_TIMEOUT          = 16,
    DRV_WIFI_IE_DIFFERENT                         = 17,
    DRV_WIFI_INVALID_GROUP_CIPHER                 = 18,
    DRV_WIFI_INVALID_PAIRWISE_CIPHER              = 19,
    DRV_WIFI_INVALID_AKMP                         = 20,
    DRV_WIFI_UNSUPP_RSN_VERSION                   = 21,
    DRV_WIFI_INVALID_RSN_IE_CAP                   = 22,
    DRV_WIFI_IEEE8021X_FAILED                     = 23,
    DRV_WIFI_CIPHER_SUITE_REJECTED                = 24
} DRV_WIFI_REASON_CODES;

// *****************************************************************************
/*  WPS State Codes

  Summary:
    Selection of different codes when a Extensible Authentication Protocol is
    used.

  Description
    This enumeration identifies the codes that can take place when using EAPOL.
*/
typedef enum
{
    DRV_WIFI_EAPOL_START        = 1,
    DRV_WIFI_EAP_REQ_IDENTITY   = 2,
    DRV_WIFI_EAP_RSP_IDENTITY   = 3,
    DRV_WIFI_EAP_WPS_START      = 4,
    DRV_WIFI_EAP_RSP_M1         = 5,
    DRV_WIFI_EAP_REQ_M2         = 6,
    DRV_WIFI_EAP_RSP_M3         = 7,
    DRV_WIFI_EAP_REQ_M4         = 8,
    DRV_WIFI_EAP_RSP_M5         = 9,
    DRV_WIFI_EAP_REQ_M6         = 10,
    DRV_WIFI_EAP_RSP_M7         = 11,
    DRV_WIFI_EAP_REQ_M8         = 12,
    DRV_WIFI_EAP_RSP_DONE       = 13,
    DRV_WIFI_EAP_FAILURE        = 14
} DRV_WIFI_WPS_STATE_CODES;

// *****************************************************************************
/*  WPS Config Error Codes

  Summary:
    Selection of different codes when a WPS connection fails.

  Description
    This enumeration identifies the codes that can take place when WPS fails.
*/
typedef enum
{
    DRV_WIFI_WPS_NOERR                      = 0,
    DRV_WIFI_WPS_SESSION_OVERLAPPED         = 1,
    DRV_WIFI_WPS_DECRYPT_CRC_FAILURE        = 2,
    DRV_WIFI_WPS_24G_NOT_SUPPORTED          = 3,
    DRV_WIFI_WPS_RETRY_FAILURE              = 4,
    DRV_WIFI_WPS_INVALID_MSG                = 5,
    DRV_WIFI_WPS_AUTH_FAILURE               = 6,
    DRV_WIFI_WPS_ASSOC_FAILURE              = 7,
    DRV_WIFI_WPS_MSG_TIMEOUT                = 8,
    DRV_WIFI_WPS_SESSION_TIMEOUT            = 9,
    DRV_WIFI_WPS_DEVPASSWD_AUTH_FAILURE     = 10,
    DRV_WIFI_WPS_NO_CONN_TOREG              = 11,
    DRV_WIFI_WPS_MULTI_PBC_DETECTED         = 12,
    DRV_WIFI_WPS_EAP_FAILURE                = 13,
    DRV_WIFI_WPS_DEV_BUSY                   = 14,
    DRV_WIFI_WPS_SETUP_LOCKED               = 15
} DRV_WIFI_WPS_ERROR_CONFIG_CODES;

// *****************************************************************************
/*
  Summary:
    Selection of different codes when Wi-Fi connection is temporarily lost.

  Description:
    'Connection Temporarily Lost' event codes.

    This enumeration identifies the codes for a connection temporarily
    lost. These codes are used in DRV_WIFI_ProcessEvent, case
    DRV_WIFI_EVENT_CONNECTION_TEMPORARILY_LOST.
*/
typedef enum
{
    /* connection temporarily lost due to beacon timeout */
    DRV_WIFI_BEACON_TIMEOUT                       = 1,

    /* connection temporarily lost due to deauthorization received from AP */
    DRV_WIFI_DEAUTH_RECEIVED                      = 2,

    /* connection temporarily lost due to disassociation received from AP */
    DRV_WIFI_DISASSOCIATE_RECEIVED                = 3
} DRV_WIFI_EVENT_CONN_TEMP_LOST_CODES;

//************************************************************************
/*
  Summary:
    Selection of different codes when Wi-Fi connection fails due to
    association or authentication failure.

  Description:
    Status codes for connection for association or authentication failure.

    This enumeration identifies the codes for a connection failure due to
    association or authentication failure. These codes are used in
    DRV_WIFI_ProcessEvent, case DRV_WIFI_EVENT_CONNECTION_FAILED.
*/
typedef enum
{
    DRV_WIFI_UNSPECIFIED_FAILURE                  = 1,
    DRV_WIFI_CAPS_UNSUPPORTED                     = 10,
    DRV_WIFI_REASSOC_NO_ASSOC                     = 11,
    DRV_WIFI_ASSOC_DENIED_UNSPEC                  = 12,
    DRV_WIFI_NOT_SUPPORTED_AUTH_ALG               = 13,
    DRV_WIFI_UNKNOWN_AUTH_TRANSACTION             = 14,
    DRV_WIFI_CHALLENGE_FAIL                       = 15,
    DRV_WIFI_AUTH_TIMEOUT                         = 16,
    DRV_WIFI_AP_UNABLE_TO_HANDLE_NEW_STA          = 17,
    DRV_WIFI_ASSOC_DENIED_RATES                   = 18,
    DRV_WIFI_ASSOC_DENIED_NOSHORTPREAMBLE         = 19,
    DRV_WIFI_ASSOC_DENIED_NOPBCC                  = 20,
    DRV_WIFI_ASSOC_DENIED_NOAGILITY               = 21,
    DRV_WIFI_ASSOC_DENIED_NOSHORTTIME             = 25,
    DRV_WIFI_ASSOC_DENIED_NODSSSOFDM              = 26,
    DRV_WIFI_S_INVALID_IE                         = 40,
    DRV_WIFI_S_INVALID_GROUPCIPHER                = 41,
    DRV_WIFI_S_INVALID_PAIRWISE_CIPHER            = 42,
    DRV_WIFI_S_INVALID_AKMP                       = 43,
    DRV_WIFI_UNSUPPORTED_RSN_VERSION              = 44,
    DRV_WIFI_S_INVALID_RSN_IE_CAP                 = 45,
    DRV_WIFI_S_CIPHER_SUITE_REJECTED              = 46,
    DRV_WIFI_TIMEOUT                              = 47
} DRV_WIFI_STATUS_CODES;

// *****************************************************************************
/*
  Summary:
    Codes for MRF Wi-Fi Device Type

  Description:
    MRF Wi-Fi devices type.

    This enumeration identifies MRF Wi-Fi device type. The only device
    supported with this driver is DRV_WIFI_MRF24WG0M_DEVICE.
*/
typedef enum
{
    DRV_WIFI_MRF24WB0M_DEVICE = 1,
    DRV_WIFI_MRF24WG0M_DEVICE = 2
} DRV_WIFI_DEVICE_TYPE;

// *****************************************************************************
/*
  Summary:
    Wi-Fi Regional Domain Codes

  Description:
    This enumeration identifies Wi-Fi regional domain codes. The regional
    domain can be determined by calling DRV_WIFI_RegionalDomainGet.
*/
typedef enum
{
    /* FCC, available channels: 1 - 11 */
    DRV_WIFI_DOMAIN_FCC   = 0,

    /* ESTI, available Channels: 1 - 13 */
    DRV_WIFI_DOMAIN_ETSI  = 2,

    /* Japan, available Channels: 1 - 14 */
    DRV_WIFI_DOMAIN_JAPAN = 7,

    /* Other, available Channels: 1 - 14 */
    DRV_WIFI_DOMAIN_OTHER = 7

} DRV_WIFI_DOMAIN_CODES;

// *****************************************************************************
/*
  Summary:
    Wi-Fi Power-Saving States

  Description:
    This enumeration identifies Wi-Fi power-saving states. See
    DRV_WIFI_PsPollEnable.
*/
typedef enum
{
    /* enable hibernate mode */
    DRV_WIFI_PS_HIBERNATE             = 1,

    /* enable power-saving mode with DTIM enabled */
    DRV_WIFI_PS_PS_POLL_DTIM_ENABLED  = 2,

    /* enable power-saving mode with DTIM disabled */
    DRV_WIFI_PS_PS_POLL_DTIM_DISABLED = 3,

    /* disable power-saving mode*/
    DRV_WIFI_PS_OFF                   = 4
} DRV_WIFI_POWER_SAVE_STATES;

// *****************************************************************************
/*
  Summary:
    Wi-Fi Hibernate States

  Description:
    This enumeration identifies Wi-Fi hibernate states.
*/
typedef enum
{
    DRV_WIFI_HB_NO_SLEEP    = 0,
    DRV_WIFI_HB_ENTER_SLEEP = 1,
    DRV_WIFI_HB_WAIT_WAKEUP = 2
} DRV_WIFI_HIBERNATE_STATES;

// *****************************************************************************
/*
  Summary:
    Wi-Fi Connection States

  Description:
    This enumeration identifies Wi-Fi Connection states. See
    DRV_WIFI_ConnectionStateGet.
*/
typedef enum
{
    /*No Wi-Fi connection exists*/
    DRV_WIFI_CSTATE_NOT_CONNECTED               = 1,

    /*Wi-Fi connection in progress*/
    DRV_WIFI_CSTATE_CONNECTION_IN_PROGRESS      = 2,

    /*Wi-Fi connected in infrastructure mode*/
    DRV_WIFI_CSTATE_CONNECTED_INFRASTRUCTURE    = 3,

    /*Wi-Fi connected in adHoc mode*/
    DRV_WIFI_CSTATE_CONNECTED_ADHOC             = 4,

    /*Wi-Fi in process of reconnecting*/
    DRV_WIFI_CSTATE_RECONNECTION_IN_PROGRESS    = 5,

    /*Wi-Fi connection permanently lost*/
    DRV_WIFI_CSTATE_CONNECTION_PERMANENTLY_LOST = 6
} DRV_WIFI_CONNECTION_STATES;

// *****************************************************************************
/*
  Summary:
    Wi-Fi Soft AP Events

  Description:
    This enumeration identifies Wi-Fi Soft AP events.
*/
typedef enum
{
    DRV_WIFI_SOFTAP_EVENT_CONNECTED    = 0,
    DRV_WIFI_SOFTAP_EVENT_DISCONNECTED = 1
} DRV_WIFI_SOFT_AP_STATES;

// *****************************************************************************
/*
  Summary:
    Wi-Fi Soft AP Event Reason Codes

  Description:
    This enumeration identifies Wi-Fi Soft AP events.
*/
typedef enum
{
    DRV_WIFI_SOFTAP_EVENT_LINK_LOST       = 0,
    DRV_WIFI_SOFTAP_EVENT_RECEIVED_DEAUTH = 1
} DRV_WIFI_SOFT_AP_EVENT_REASON_CODES;

//**************************************************************************
/*
  Summary:
    Wi-Fi MIB states

  Description:
    Wi-Fi MIB states

    This structure contains all the MIB data returned from the MRF24WG when
    DRV_WIFI_MacStatsGet() is called.
*/
typedef struct
{
    /********************************************************************
      Number of frames received with the Protected Frame sub-field of the
      Frame Control field set to zero and the value of
      dot11ExcludeUnencrypted causes that frame to be discarded.
     ********************************************************************/
    uint32_t MibWEPExcludeCtr;

    /*Total number of TX bytes that have been transmitted*/
    uint32_t MibTxBytesCtr;

    /* Number of frames successfully transmitted that had the multicast bit set */
    /*  in the destination MAC address                                          */
    uint32_t MibTxMulticastCtr;

    /***************************************************************
      Number of TX frames that failed due to the number of transmits
      exceeding the retry count
     ***************************************************************/
    uint32_t MibTxFailedCtr;

    /* Number of times a transmitted frame needed to be retried. */
    uint32_t MibTxRtryCtr;

    /* Number of times a frame was successfully transmitted after more than one retransmission. */
    uint32_t MibTxMultRtryCtr;

    /* Number of TX frames successfully transmitted. */
    uint32_t MibTxSuccessCtr;

    /* Number of frames received where the Sequence Control field indicates a duplicate. */
    uint32_t MibRxDupCtr;

    /* Number of CTS frames received in response to an RTS frame. */
    uint32_t MibRxCtsSuccCtr;

    /* Number of times an RTS frame was not received in response to a CTS frame. */
    uint32_t MibRxCtsFailCtr;

    /* Number of times an ACK was not received in response to a TX frame. */
    uint32_t MibRxAckFailCtr;

    /* Total number of Rx bytes received. */
    uint32_t MibRxBytesCtr;

    /* Number of successful received frames (management or data). */
    uint32_t MibRxFragCtr;

    /* Number of frames received with the multicast bit set in the destination MAC address. */
    uint32_t MibRxMultCtr;

    /* Number of frames received with an invalid Frame Checksum (FCS). */
    uint32_t MibRxFCSErrCtr;

    /***********************************************************************
      Number of frames received where the Protected Frame sub-field of the
      Frame Control Field is set to one and the WEPOn value for the key
      mapped to the transmitter's MAC address indicates the frame should not
      have been encrypted.
     ***********************************************************************/
    uint32_t MibRxWEPUndecryptCtr;

    /* Number of times that fragments aged out, or were not received in the allowable time. */
    uint32_t MibRxFragAgedCtr;

    /* Number of MIC failures that have occurred. */
    uint32_t MibRxMICFailureCtr;

} DRV_WIFI_MAC_STATS;

// *****************************************************************************
/*
  Summary:
    Contains data pertaining to MRF24WG device type and version number.

  Description:
    This structure contains MRF24WG device type and version number. See
    DRV_WIFI_DeviceInfoGet.
*/
typedef struct
{
    /* MRF24W device type: DRV_WIFI_DEVICE_TYPE */
    uint8_t deviceType;

    /* MRF24WG ROM version number */
    uint8_t romVersion;

    /* MRF24WG patch version number */
    uint8_t patchVersion;

} DRV_WIFI_DEVICE_INFO;

// *****************************************************************************
/*
  Summary:
    Contains data pertaining to MRF24WG connection context.

  Description:
    This structure contains MRF24WG connection context data. See
    DRV_WIFI_ConnectContextGet.
*/
typedef struct
{
    /* channel number of current connection */
    uint8_t channel;

    /* bssid of connected AP */
    uint8_t bssid[6];

} DRV_WIFI_CONNECTION_CONTEXT;

// *****************************************************************************
/*
  Summary:
    Contains data pertaining to Wi-Fi scan context.

  Description:
    This structure contains MRF24WG scan context data. See
    DRV_WIFI_ScanContextSet.
*/
typedef struct
{
    /* 802.11 allows for active scanning, where the device sends out a broadcast
     probe request seeking an access point.  Also allowed is passive scanning
     where the device only listens to beacons being broadcast from access points.
     Set to DRV_WIFI_ACTIVE_SCAN (default) or DRV_WIFI_PASSIVE_SCAN */
    uint8_t scanType;

    /* The number of times to scan a channel while attempting to find a particular
     access point. Default is 1 */
    uint8_t scanCount;

    /************************************************************************
      The minimum time (in milliseconds) the MRF24WG will wait for a probe
      response after sending a probe request. If no probe responses are
      received in minChannelTime, the MRF24WG will go on to the next channel,
      if any are left to scan, or quit. Default is 200ms.
     ************************************************************************/
    uint16_t minChannelTime;

    /************************************************************************
      If a probe response is received within minChannelTime, the MRF24WG will
      continue to collect any additional probe responses up to maxChannelTime
      before going to the next channel in the channelList. Units are in
      milliseconds. Default is 400ms.
     ************************************************************************/
    uint16_t maxChannelTime;

    /* The number of microseconds to delay before transmitting a probe request
     following the channel change during scanning.  Default is 20uS. */
    uint16_t probeDelay;

} DRV_WIFI_SCAN_CONTEXT;

// *****************************************************************************
/*
  Summary:
    Contains data pertaining to Wi-Fi PS-Poll context.

  Description:
    This structure contains MRF24WG PS-Poll context data. See
    DRV_WIFI_PsPollEnable.
*/
typedef struct
{
    /* Number of 100ms intervals between instances when the MRF24WG wakes up to
     received buffered messages from the network.  Each count represents 100ms.
     For example, 1 = 100ms, 2 = 200ms, etc.  The default is 1 (100ms). */
    uint16_t listenInterval;

    /* Only used if useDtim is true.  The DTIM period indicates how often clients
     serviced by the access point should check for buffered multicast or broadcast
     messages awaiting pickup on the access point.    The DTIM interval is
     measured in number of beacon periods.  Default for DTIM period is 2. */
    uint16_t dtimInterval;

    /* True:  (default) check for buffered multicast or broadcast messages on
              the dtimInterval.
       False: check for buffered multicast or broadcast messages on the listenInterval */
    bool useDtim;

} DRV_WIFI_PS_POLL_CONTEXT;

// *****************************************************************************
/*
  Summary:
    Contains data pertaining to Wi-Fi Ad-Hoc context.

  Description:
    This structure contains MRF24WG Ad-Hoc context data. See
    DRV_WIFI_AdhocContextSet.
*/
typedef struct
{
    /* Defines how to start the Ad-Hoc network. See DRV_WIFI_ADHOC_MODE. */
    /* Default is DRV_WIFI_ADHOC_CONNECT_THEN_START. */
    uint8_t mode;

    /* When starting an Ad-Hoc network, the SSID can be hidden in the beacons. */
    /* Set true to hide the SSID, else false.  Default is false. */
    bool hiddenSsid;

    /* Sets the beacon period, in ms.  Default is 100 ms. */
    uint16_t beaconPeriod;

} DRV_WIFI_ADHOC_NETWORK_CONTEXT;

// *****************************************************************************
/*
  Summary:
    Contains data pertaining to Wi-Fi Soft AP context.

  Description:
    This structure contains MRF24WG Soft AP context data. See
    DRV_WIFI_SoftAPContextSet.
*/
typedef struct
{
    /* Defines how to start the Soft AP network. See DRV_WIFI_SOFTAP_MODE. */
    /* Default is DRV_WIFI_SOFTAP_CONNECT_THEN_START. */
    //uint8_t  mode;

    /* When starting an Soft AP network, the SSID can be hidden in the beacons. */
    /* Set true to hide the SSID, else false.  Default is false. */
    bool hiddenSsid;

} DRV_WIFI_SOFTAP_NETWORK_CONTEXT;

// *****************************************************************************
/*
  Summary:
    Contains data pertaining to Wi-Fi WEP context.

  Description:
    This structure contains MRF24WG WEP context. See
    DRV_WIFI_SecurityWepSet.
*/
typedef struct
{
    /* DRV_WIFI_SECURITY_WEP_40 or DRV_WIFI_SECURITY_WEP_104 */
    uint8_t wepSecurityType;

    /* Array containing four WEP binary keys. This will be four, 5-byte keys for
     WEP-40 or four, thirteen-byte keys for WEP-104. */
    uint8_t wepKey[DRV_WIFI_MAX_WEP_KEY_LENGTH];

    /* number of bytes pointed to by p_wepKey */
    uint8_t wepKeyLength;

    /* DRV_WIFI_SECURITY_WEP_OPENKEY (default) or DRV_WIFI_SECURITY_WEP_SHAREDKEY */
    uint8_t wepKeyType;

} DRV_WIFI_WEP_CONTEXT;

// *****************************************************************************
/*
  Summary:
    Contains data pertaining to Wi-Fi WPA Key.

  Description:
    This structure contains MRF24WG WPA key info. This structure is used in
    the DRV_WIFI_WPA_CONTEXT and DRV_WIFI_WPS_CONTEXT structures.
*/
typedef struct
{
    /* binary key or passphrase */
    uint8_t key[DRV_WIFI_MAX_WPA_PASS_PHRASE_LENGTH];

    /* number of bytes in binary key (always 32) or passphrase */
    uint8_t keyLength;

} DRV_WIFI_WPA_KEY_INFO;

// *****************************************************************************
/*
  Summary:
    Contains data pertaining to Wi-Fi WPA.

  Description:
    This structure contains MRF24WG WPA context. See
    DRV_WIFI_SecurityWpaSet.

    <table>
    DRV_WIFI_SECURITY_WPA_WITH_KEY               Select WPA with binary key
    DRV_WIFI_SECURITY_WPA_WITH_PASS_PHRASE       Select WPA with passphrase
    DRV_WIFI_SECURITY_WPA2_WITH_KEY              Select WPA2 with binary key
    DRV_WIFI_SECURITY_WPA2_WITH_PASS_PHRASE      Select WPA2 with passphrase
    DRV_WIFI_SECURITY_WPA_AUTO_WITH_KEY          Auto-select between WPA/WPA2 with binary key
    DRV_WIFI_SECURITY_WPA_AUTO_WITH_PASS_PHRASE  Auto-select between WPA/WPA2 with passphrase
    </table>
*/
typedef struct
{
    /* desired security type (see description) */
    uint8_t wpaSecurityType;

    /* see DRV_WIFI_WPA_KEY_INFO */
    DRV_WIFI_WPA_KEY_INFO keyInfo;

} DRV_WIFI_WPA_CONTEXT;

// *****************************************************************************
/*
  Summary:
    Contains data pertaining to Wi-Fi WPS security.

  Description:
    This structure contains MRF24WG WPS security context. See
    DRV_WIFI_SecurityWpsSet.
*/
typedef struct
{
    /* DRV_WIFI_SECURITY_WPS_PUSH_BUTTON or DRV_WIFI_SECURITY_WPS_PIN */
    uint8_t wpsSecurityType;

    /* if using DRV_WIFI_SECURITY_WPS_PIN then pointer to 8-digit pin */
    uint8_t wpsPin[DRV_WIFI_WPS_PIN_LENGTH];

    /* should always be 8 if used, 0 if not used*/
    uint8_t wpsPinLength;

} DRV_WIFI_WPS_CONTEXT;

// *****************************************************************************
/*
  Summary:
    Contains data pertaining to Wi-Fi scan results.

  Description:
    This structure contains the result of Wi-Fi scan operation. See
    DRV_WIFI_ScanResultGet.

    apConfig Bit Mask
    <table>
    Bit 7       Bit 6       Bit 5       Bit 4       Bit 3       Bit 2       Bit 1       Bit 0
    -----       -----       -----       -----       -----       -----       -----       -----
    WPA2        WPA         Preamble    Privacy     Reserved    Reserved    Reserved    IE
    </table>

    <table>
    IE         1 if AP broadcasting one or more Information Elements, else 0
    Privacy    0 : AP is open (no security) 1: AP using security, if neither WPA
                and WPA2 set then security is WEP.
    Preamble   0: AP transmitting with short preamble 1: AP transmitting with long preamble
    WPA        Only valid if Privacy is 1. 0: AP does not support WPA 1: AP
                supports WPA
    WPA2       Only valid if Privacy is 1. 0: AP does not support WPA2 1: AP supports WPA2
    </table>
*/
typedef struct
{
    /* Network BSSID value */
    uint8_t    bssid[DRV_WIFI_BSSID_LENGTH];

    /* Network SSID value */
    uint8_t    ssid[DRV_WIFI_MAX_SSID_LENGTH];

    /* Access Point configuration (see description) */
    uint8_t    apConfig;

    /* not used */
    uint8_t    reserved;

    /* Network beacon interval */
    uint16_t    beaconPeriod;

    /* Only valid if bssType = DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE */
    uint16_t    atimWindow;

    /*
      List of Network basic rates.  Each rate has the following format:

          Bit 7
      * 0: rate is not part of the basic rates set
      * 1: rate is part of the basic rates set

          Bits 6:0
      Multiple of 500kbps giving the supported rate.  For example, a value of 2
      (2 * 500kbps) indicates that 1mbps is a supported rate.  A value of 4 in
      this field indicates a 2mbps rate (4 * 500kbps).
      */
    uint8_t    basicRateSet[DRV_WIFI_MAX_NUM_RATES];

    /* Signal strength of received frame beacon or probe response.  Will range
       from a low of 43 to a high of 128. */
    uint8_t    rssi;

    /* Number of valid rates in basicRates */
    uint8_t    numRates;

    /* Part of TIM element */
    uint8_t    dtimPeriod;

    /* DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE or DRV_WIFI_NETWORK_TYPE_ADHOC */
    uint8_t    bssType;

    /* Channel number */
    uint8_t    channel;

    /* Number of valid characters in ssid */
    uint8_t    ssidLen;

} DRV_WIFI_SCAN_RESULT;

// *****************************************************************************
/*
  Summary:
    Contains data pertaining to Wi-Fi software multicast filter
    configuration.

  Description:
    This structure contains data pertaining to the configuration of the
    software multicast config filter.

    <table>
    'action' Field                   Description
    -------------------------------  ---------------------------------------
    DRV_WIFI_MULTICAST_DISABLE_ALL   Multicast filter discards all received
                                      multicast messages.
    DRV_WIFI_MULTICAST_ENABLE_ALL    Multicast filter forwards all received
                                      multicast messages to host.
    DRV_WIFI_MULTICAST_USE_FILTERS   The MAC filter will be used and the
                                      remaining fields define the filter.
    </table>

*/
typedef struct
{
    /* DRV_WIFI_MULTICAST_FILTER_1 through DRV_WIFI_MULTICAST_FILTER_16 */
    uint8_t filterId;

    /* configures the multicast filter (see description) */
    uint8_t action;

    /* Array containing the MAC address to filter on (using the destination
       address of each incoming 802.11 frame).  Specific bytes within the MAC
       address can be designated as "don't care" bytes.  See macBitMask.
       This field in only used if action = WF_MULTICAST_USE_FILTERS. */
    uint8_t macBytes[6];

    /* A byte where bits 5:0 correspond to macBytes[5:0].  If the bit is zero then
       the corresponding MAC byte must be an exact match for the frame to be
       forwarded to the Host PIC.  If the bit is one then the corresponding MAC
       byte is a "don't care" and not used in the Multicast filtering process.
       This field in only used if action = WF_MULTICAST_USE_FILTERS. */
    uint8_t macBitMask;

} DRV_WIFI_MULTICAST_CONFIG;

// *****************************************************************************
/*
  Summary:
    Contains data pertaining to Wi-Fi WPS Credentials.

  Description:
    This structure contains data pertaining to the configuration of the Wi-Fi
    WPS credentials.

    <table>
    'authType' Field             Description
    ---------------------------  --------------
    DRV_WIFI_WPS_AUTH_OPEN       Open Security
    DRV_WIFI_WPS_AUTH_WPA_PSK    WPA with PSK
    DRV_WIFI_WPS_AUTH_SHARED     Shared Key
    DRV_WIFI_WPS_AUTH_WPA        WPA
    DRV_WIFI_WPS_AUTH_WPA2       WPA2
    DRV_WIFI_WPS_AUTH_WPA2_PSK   WPA2 with PSK
    </table>

    <table>
    'encType' Field         Description
    ----------------------  ----------------------------
    DRV_WIFI_WPS_ENC_NONE   No encoding
    DRV_WIFI_WPS_ENC_WEP    WEP encoding
    DRV_WIFI_WPS_ENC_TKIP   TKIP encoding
    DRV_WIFI_ENC_AES        AES encoding
    </table>

*/
typedef struct
{
        /* network SSID */
        uint8_t ssid[DRV_WIFI_MAX_SSID_LENGTH];

        /* binary security key (not used if security is open) */
        uint8_t netKey[DRV_WIFI_MAX_SECURITY_KEY_LENGTH];

        /* WPS authorization type (see description) */
        uint16_t authType;

        /* encoding type (see description) */
        uint16_t encType;

        /* not used */
        uint8_t netIdx;

        /* number of bytes in SSID */
        uint8_t ssidLen;

        /* Only valid encType = WF_ENC_WEP.  This is the index of the WEP key being used. */
        uint8_t keyIdx;

        /* number of bytes in netKey */
        uint8_t keyLen;

        /* MAC address of AP */
        uint8_t bssid[DRV_WIFI_BSSID_LENGTH];
} DRV_WIFI_WPS_CREDENTIAL;

// *****************************************************************************
/*
  Summary:
    Contains data pertaining to Wi-Fi Soft AP event.

  Description:
    This structure contains data pertaining to Soft AP event. See
    DRV_WIFI_SoftApEventInfoGet.
*/
typedef struct
{
    /* event code */
    uint8_t event;

    /* reason code */
    uint8_t reason;

    /* MAC address */
    uint8_t address[6];
} DRV_WIFI_MGMT_INDICATE_SOFT_AP_EVENT;

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - System Level
// *****************************************************************************
// *****************************************************************************

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_Initialize(void)

  Summary:
    Initializes the MRF24WG Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function initializes the MRF24WG driver, making it ready for
    clients to use.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    This function must be called before any other Wi-Fi routine is called.
    The real work of Wi-Fi initialization takes place in an internal state
    machine, whose state is set to the initial value by this function.
*/
void DRV_WIFI_Initialize(void);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_Deinitialize(void)

  Summary:
    De-initializes the MRF24WG Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function de-initializes the MRF24WG driver. It also saves the Wi-Fi
    parameters in non-volatile storage.

  Precondition:
    None.

  Returns:
    None.

  Remarks:
    None.
*/
void DRV_WIFI_Deinitialize(void);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_NetworkTypeSet(uint8_t networkType)

  Summary:
    Sets the Wi-Fi network type.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function selects the Wi-Fi network type.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    networkType -  one of the following:
      DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE
      DRV_WIFI_NETWORK_TYPE_ADHOC
      DRV_WIFI_NETWORK_TYPE_SOFT_AP

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_NetworkTypeSet(DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_NetworkTypeSet(uint8_t networkType);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_NetworkTypeGet(uint8_t *p_networkType)

  Summary:
    Gets the Wi-Fi network type.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gets the Wi-Fi network type.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_networkType -  pointer to where the network type will be written.  One of the following:
                       DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE
                       DRV_WIFI_NETWORK_TYPE_ADHOC
                       DRV_WIFI_NETWORK_TYPE_SOFT_AP

  Returns:
    None.

  Example:
    <code>
        uint8_t networkType;

        DRV_WIFI_NetworkTypeGet(&networkType);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_NetworkTypeGet(uint8_t *p_networkType);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_SsidSet(uint8_t *p_ssid, uint8_t ssidLen)

  Summary:
    Sets the SSID.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    Sets the SSID and SSID Length. Note that an Access Point can have
    either a visible or hidden SSID. If an Access Point uses a hidden SSID
    then an active scan must be used.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_ssid -  pointer to SSID buffer
    ssidLength -  number of bytes in SSID

  Returns:
    None.

  Example:
    <code>
        uint8_t ssid[] = "MySSIDName";
        uint8_t ssidLength = strlen(ssid);

        DRV_WIFI_SsidSet(ssid, &ssidLen);
    </code>

  Remarks:
    Do not include a string terminator in the SSID length. SSIDs are
    case-sensitive. SSID length must be less than or equal to
    DRV_WIFI_MAX_SSID_LENGTH.
*/
void DRV_WIFI_SsidSet(uint8_t *p_ssid, uint8_t ssidLen);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_SsidGet(uint8_t *p_ssid, uint8_t *p_ssidLength)

  Summary:
    Gets the SSID.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    Gets the SSID and SSID Length.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_ssid -  pointer to buffer where SSID will be written
    ssidLength -  number of bytes in SSID

  Returns:
    None.

  Example:
    <code>
        uint8_t ssid[DRV_WIFI_MAX_SSID_LENGTH];
        uint8_t ssidLength;

        DRV_WIFI_SsidGet(ssid, &ssidLength);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_SsidGet(uint8_t *p_ssid, uint8_t *p_ssidLength);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_AdhocContextSet(DRV_WIFI_ADHOC_NETWORK_CONTEXT *p_context)

  Summary:
    Sets the Ad-Hoc context.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the Ad-Hoc context. It is only applicable when the
    DRV_WIFI_NETWORK_TYPE_ADHOC has been selected in
    DRV_WIFI_NetworkTypeSet.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_context -  pointer to Ad-Hoc context data; see definition for the
                 DRV_WIFI_ADHOC_NETWORK_CONTEXT structure.

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_ADHOC_NETWORK_CONTEXT adHocContext;

        adHocContext.mode = DRV_WIFI_ADHOC_CONNECT_THEN_START;
        adHocContext.hiddenSsid = false;
        adHocContext.beaconPeriod = DRV_WIFI_DEFAULT_ADHOC_BEACON_PERIOD;

        DRV_WIFI_AdhocContextSet(&adHocContext);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_AdhocContextSet(DRV_WIFI_ADHOC_NETWORK_CONTEXT *p_context);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_SoftAPContextSet(DRV_WIFI_SOFTAP_NETWORK_CONTEXT *p_context)

  Summary:
    Sets the Soft AP context.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    None.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_context -  pointer to Soft AP context data; see definition for the
                 DRV_WIFI_SOFTAP_NETWORK_CONTEXT structure

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_SOFTAP_NETWORK_CONTEXT SoftAPContext;

        //SoftAPContext.mode = DRV_WIFI_ADHOC_CONNECT_THEN_START;
        SoftAPContext.hiddenSsid = false;
        //SoftAPContext.beaconPeriod = DRV_WIFI_DEFAULT_ADHOC_BEACON_PERIOD;

        DRV_WIFI_SoftAPContextSet(&SoftAPContext);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_SoftAPContextSet(DRV_WIFI_SOFTAP_NETWORK_CONTEXT *p_context);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_BssidSet(uint8_t *p_bssid)

  Summary:
    Sets the Basic Service Set Identifier (BSSID).
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This sets 6 byte (48-bit) MAC address of the Access Point that is being
    scanned for. It is optional to use this. Where it is useful is if there
    are two APs with the same ID; the BSSID is used to connect to the
    specified AP. This setting can be used in lieu of the SSID. Set each
    byte to 0xFF (default) if the BSSID is not being used. Not typically
    needed.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_context -  pointer to BSSID

  Returns:
    None.

  Example:
    <code>
        uint8_t bssid[6];

        bssid[0] = 0x00;
        bssid[1] = 0xe8;
        bssid[2] = 0xc0;
        bssid[3] = 0x11;
        bssid[4] = 0x22;
        bssid[5] = 0x33;

        DRV_WIFI_BssidSet(bssid);

    </code>

  Remarks:
    None.
*/
void DRV_WIFI_BssidSet(uint8_t *p_bssid);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_BssidGet(uint8_t *p_bssid)

  Summary:
    Gets the BSSID set in DRV_WIFI_BssidSet().
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    Retrieves the BSSID set in the previous call to DRV_WIFI_BssidSet().

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_context -  pointer to where BSSID will be written

  Returns:
    None.

  Example:
    <code>
        uint8_t bssid[6];

        DRV_WIFI_BssidGet(bssid);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_BssidGet(uint8_t *p_bssid);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_ChannelListSet(uint8_t *p_channelList, uint8_t numChannels)

  Summary:
    Sets the channel list.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the channel list that the MRF24WG will use when
    scanning or connecting.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_channelList -  list of channels
    numChannels -  number of channels in list; if set to 0, then MRF24WG
                     will set its channel list to all valid channels in its
                     regional domain

  Returns:
    None.

  Example:
    <code>
        uint8_t channelList[1, 6, 11];

        DRV_WIFI_ChannelListSet(channelList, sizeof(channelList));
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_ChannelListSet(uint8_t *p_channelList, uint8_t numChannels);

//*******************************************************************************
/*
  Function:
        void RV_WIFI_ChannelListGet(uint8_t *p_channelList, uint8_t *p_numChannels)

  Summary:
    Gets the channel list.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gets the current channel list.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_channelList -  pointer to where channel list will be written
    numChannels -  pointer to where number of channels in the list will be
                     written

  Returns:
    None.

  Example:
    <code>
        uint8_t channelList[DRV_WIFI_MAX_CHANNEL_LIST_LENGTH];
        uint8_t numChannels;

        DRV_WIFI_ChannelListGet(channelList, &numChannels);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_ChannelListGet(uint8_t *p_channelList, uint8_t *p_numChannels);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_ReconnectModeSet(uint8_t retryCount, uint8_t deauthAction,
                                       uint8_t beaconTimeout, uint8_t beaconTimeoutAction)

  Summary:
    Sets the Wi-Fi reconnection mode.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function controls how the MRF24WG behaves when an existing Wi-Fi
    connection is lost. The MRF24WG can lose an existing connection in one
    of two ways: 1) Beacon timeout; 2) Deauthorization received from AP.

    There are two options with respect to regaining a lost Wi-Fi
    connection: 1) MRF24WG informs the host that the connection was
    temporarily lost and then the MRF24WG retries N times (or forever) to
    regain the connection. 2) MRF24WG simply informs the host application
    that the connection is lost, and it is up to the host to regain the
    connection via the API.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    retryCount -  number of times the MRF24WG should try to regain
                    the connection (see description)
    deauthAction -  in the event of a deauthorization from the AP, the action
                      the MRF24WG should take (see description)
    beaconTimeout -  number of missed beacons before the MRF24WG
                       designates the connection as lost (see description)
    beaconTimeoutAction -  in the event of a beacon timeout, the action the
                             MRF24WG should take (see description)

  Returns:
    None.

  Example:
    <code>
        // Example 1: MRF24WG should retry forever if either a deauth or beacon
        // timeout occurs (beacon timeout is 3 beacon periods).
        DRV_WIFI_ReconnectModeSet(WF_RETRY_FOREVER,
                                  WF_ATTEMPT_TO_RECONNECT,
                                  3,
                                  WF_ATTEMPT_TO_RECONNECT);

        // Example 2: MRF24WG should not do any connection retries and only report
        // deauthorization events to the host.
        DRV_WIFI_ReconnectModeSet(0,
                                  WF_DO_NOT_ATTEMPT_TO_RECONNECT,
                                  0,
                                  WF_DO_NOT_ATTEMPT_TO_RECONNECT);

        // Example 3: MRF24WG should not do any connection retries, but report deauthorization
        // and beacon timeout events to host.  Beacon timeout should be 5 beacon periods.
        DRV_WIFI_ReconnectModeSet(0,
                                  WF_DO_NOT_ATTEMPT_TO_RECONNECT,
                                  5,
                                  WF_DO_NOT_ATTEMPT_TO_RECONNECT);

        // Example 4: MRF24WG should ignore beacon timeouts, but attempt to
        // reconnect 3 times if a deauthorization occurs.
        DRV_WIFI_ReconnectModeSet(3,
                                  WF_ATTEMPT_TO_RECONNECT,
                                  0,
                                  WF_DO_NOT_ATTEMPT_TO_RECONNECT);
    </code>

  Remarks:
    The retryCount parameter also applies when initially connecting. That
    is, the retryCount tells the MRF24WG how many time to try to connect to
    a Wi-Fi network before giving up and generating the
    DRV_WIFI_EVENT_CONNECTION_FAILED event.

    <table>
    'retryCount'   Description
     field
    -------------  --------------------------------------------------------------
    0              Do not try to regain a connection (simply report event to
                    host)
    1:254          Number of times MRF24WG should try to regain the connection
    255            MRF24WG will retry forever (do not use for Ad-Hoc connections)
    </table>

    <table>
    'deauthAction' field                   Description
    -------------------------------------  ------------------------------------
    DRV_WIFI_DO_NOT_ATTEMPT_TO_RECONNECT   Do not attempt to reconnect after a
                                            deauth
    DRV_WIFI_ATTEMPT_TO_RECONNECT          Attempt to reconnect after a deauth
    </table>

    <table>
    'beaconTimeout'   Description
     field
    ----------------  ------------------------------------------------------
    0                 MRF24WG will not monitor the beacon timeout condition
    1:255             Number of missed beacons before designating the
                       connection as lost.
    </table>

    <table>
    'beaconTimeoutAction' field            Description
    -------------------------------------  ----------------------------------
    DRV_WIFI_DO_NOT_ATTEMPT_TO_RECONNECT   Do not attempt to reconnect after
                                            a beacon timeout
    DRV_WIFI_ATTEMPT_TO_RECONNECT          Attempt to reconnect after a
                                            beacon timeout
    </table>
*/
void DRV_WIFI_ReconnectModeSet(uint8_t retryCount, uint8_t deauthAction,
                               uint8_t beaconTimeout, uint8_t beaconTimeoutAction);

//*******************************************************************************
/*
  Function:
        void  DRV_WIFI_ReconnectModeGet(uint8_t *p_retryCount, uint8_t *p_deauthAction,
                                        uint8_t *p_beaconTimeout, uint8_t *p_beaconTimeoutAction)

  Summary:
    Gets the Wi-Fi reconnection mode.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gets the reconnection mode parameters set in
    DRV_WIFI_ReconnectModeGet.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_retryCount -  pointer where retry count is written
    p_deauthAction -  pointer where deauthorization action is written
    p_beaconTimeout -  pointer where beacon timeout is written
    p_beaconTimeoutAction -  pointer where beacon timeout action is written

  Returns:
    None.

  Example:
    <code>
        uint8_t retryCount, deauthAction, beaconTimeout, beaconTimeoutAction;

        DRV_WIFI_ReconnectModeGet(&retryCount,
                                  &deauthAction,
                                  &beaconTimeout,
                                  &beaconTimeoutAction);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_ReconnectModeGet(uint8_t *p_retryCount, uint8_t *p_deauthAction,
                      uint8_t *p_beaconTimeout, uint8_t *p_beaconTimeoutAction);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_Connect(void)

  Summary:
    Directs the MRF24WG to connect to a Wi-Fi network.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function causes the MRF24WG to connect to a Wi-Fi network. Upon
    connection, or a failure to connect, an event will be generated.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete and relevant connection parameters
    must have been set.

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_Connect();
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_Connect(void);

//*******************************************************************************
/*
  Function:
        uint16_t DRV_WIFI_Disconnect(void)

  Summary:
    Directs the MRF24WG to disconnect from a Wi-Fi network.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function causes the MRF24WG to disconnect from a Wi-Fi network. No
    event is generated when a connection is terminated via the function
    call.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete and a connection must be in
    progress.

  Returns:
    DRV_WIFI_SUCCESS or DRV_WIFI_ERROR_DISCONNECT_FAILED.

  Example:
    <code>
        DRV_WIFI_Disconnect();
    </code>

  Remarks:
    None.
*/
uint16_t DRV_WIFI_Disconnect(void);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_ConnectionStateGet(uint8_t *p_state)

  Summary:
    Gets the current Wi-Fi connection state.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gets the current Wi-Fi connection state.

    <table>
    *p_state                                      Description
    --------------------------------------------  ---------------------------
    DRV_WIFI_CSTATE_NOT_CONNECTED                 No Wi-Fi connection exists
    DRV_WIFI_CSTATE_CONNECTION_IN_PROGRESS        Wi-Fi connection in
                                                   progress
    DRV_WIFI_CSTATE_CONNECTED_INFRASTRUCTURE      Wi-Fi connected in
                                                   infrastructure mode
    DRV_WIFI_CSTATE_CONNECTED_ADHOC               Wi-Fi connected in Ad-Hoc
                                                   mode
    DRV_WIFI_CSTATE_RECONNECTION_IN_PROGRESS      Wi-Fi in process of
                                                   reconnecting
    DRV_WIFI_CSTATE_CONNECTION_PERMANENTLY_LOST   Wi-Fi connection
                                                   permanently lost
    </table>

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_state -  pointer to where state is written (see description)

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
void DRV_WIFI_ConnectionStateGet(uint8_t *p_state);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_ConnectContextGet(DRV_WIFI_CONNECTION_CONTEXT *p_ctx)

  Summary:
    Gets the current Wi-Fi connection context.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gets the current Wi-Fi connection context.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_ctx -  pointer to where connection context is written, see
               DRV_WIFI_CONNECTION_CONTEXT structure

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_CONNECTION_CONTEXT ctx;

        DRV_WIFI_ConnectContextGet(&ctx);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_ConnectContextGet(DRV_WIFI_CONNECTION_CONTEXT *p_ctx);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_LinkDownThresholdSet(uint8_t threshold)

  Summary:
    Sets number of consecutive Wi-Fi TX failures before link is considered
    down.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function allows the application to set the number of MRF24WG
    consecutive TX failures before the connection failure event
    (DRV_WIFI_LINK_LOST) is reported to the host application.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    threshold -  0: disabled (default)
                 1-255: number of consecutive TX failures before connection failure
                 event is reported

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_LinkDownThresholdSet(0); // disable link down threshold
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_LinkDownThresholdSet(uint8_t threshold);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_TxModeSet(uint8_t mode)

  Summary:
    Configures 802.11 TX mode.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the MRF24WG TX mode.

    <table>
    mode                           Description
    -----------------------------  --------------------------------
    DRV_WIFI_TXMODE_G_RATES        Use all 802.11g rates (default)
    DRV_WIFI_TXMODE_B_RATES        Use only 802.11b rates
    DRV_WIFI_TXMODE_LEGACY_RATES   Use only 1 and 2 Mbps rates
    </table>

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    mode -  TX mode value (see description)

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_TxModeSet(DRV_WIFI_TXMODE_G_RATES);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_TxModeSet(uint8_t mode);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_TxModeGet(uint8_t *p_mode)

  Summary:
    Gets 802.11 TX mode.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gets the MRF24WG TX mode.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Returns:
    None.

  Example:
    <code>
        uint8_t mode;

        DRV_WIFI_TxModeGet(&mode);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_TxModeGet(uint8_t *p_mode);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_RssiSet(uint8_t rssi)

  Summary:
    Sets RSSI restrictions when connecting.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This setting is only used if: 1) Neither an SSID or BSSID has been
    configured or 2) An SSID is defined and multiple APs are discovered
    with the same SSID.

    <table>
    rssi    Description
    ------  ----------------------------------------------------------------
    0       Connect to first network found
    1-254   Only connect to network if the RSSI is greater or equal to this
             value
    255     Connect to the highest RSSI found (default)
    </table>

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    rssi -  see description

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_RssiSet(255);
    </code>

  Remarks:
    Rarely needed.
*/
void DRV_WIFI_RssiSet(uint8_t rssi);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_RssiGet(uint8_t *p_rssi)

  Summary:
    Gets RSSI value set in DRV_WIFI_RssiSet().
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function retrieves the value set in Gets RSSI value set in
    DRV_WIFI_RssiSet(). It does not retrieve the current connection RSSI
    value. The scan result will yield the current RSSI.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_rssi -  pointer where rssi value is written

  Returns:
    None.

  Example:
    <code>
        uint8_t rssi;

        DRV_WIFI_RssiGet(&rssi);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_RssiGet(uint8_t *p_rssi);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_SecurityOpenSet(void)

  Summary:
    Sets Wi-Fi security to open (no security).
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the Wi-Fi security to open. One can only connect to
    an AP that is running in open mode.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete. Must be in an unconnected state.

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_SecurityOpenSet();
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_SecurityOpenSet(void);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_SecurityWepSet(DRV_WIFI_WEP_CONTEXT *p_context)

  Summary:
    Sets Wi-Fi security to use WEP.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the Wi-Fi security to WEP. One can only connect to
    an AP that is running the same WEP mode.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete. Must be in an unconnected state.

  Parameters:
    p_context -  desired WEP context, see DRV_WIFI_WEP_CONTEXT structure

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_WEP_CONTEXT context;

        context.wepSecurityType = DRV_WIFI_SECURITY_WEP_40;
        context.wepKey[] = {0x5a, 0xfb, 0x6c, 0x8e, 0x77,
                            0xc1, 0x04, 0x49, 0xfd, 0x4e,
                            0x43, 0x18, 0x2b, 0x33, 0x88,
                            0xb0, 0x73, 0x69, 0xf4, 0x78};

        context.wepKeyLength = 20;
        context.wepKeyType = DRV_WIFI_SECURITY_WEP_OPENKEY;
        DRV_WIFI_SecurityOpenSet(&context);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_SecurityWepSet(DRV_WIFI_WEP_CONTEXT *p_context);

//*******************************************************************************
/*
  Function:
        DRV_WIFI_SecurityWpaSet(DRV_WIFI_WPA_CONTEXT* p_context)

  Summary:
    Sets Wi-Fi security to use WPA or WPA2.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the Wi-Fi security to WPA or WPA2. One can only
    connect to an AP that is running the same WPA mode.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete. Must be in an unconnected state.

  Parameters:
    p_context -  desired WPA context, see DRV_WIFI_WPA_CONTEXT structure

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_WPA_CONTEXT context;

        context.wpaSecurityType  = DRV_WIFI_SECURITY_WPA_AUTO_WITH_PASS_PHRASE
        context.keyInfo.key[]    = "MySecretWPAPassPhrase";
        context.keyInfo.keyLenth = strlen(context.keyInfo.key);
        DRV_WIFI_SecurityWpaSet(&context);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_SecurityWpaSet(DRV_WIFI_WPA_CONTEXT* p_context);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_SecurityWpsSet(DRV_WIFI_WPS_CONTEXT *p_context)

  Summary:
    Sets Wi-Fi security to use WPS.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the Wi-Fi security to WPS. One can only connect to
    an AP that supports WPS.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete. Must be in an unconnected state.

  Parameters:
    p_context -  desired WPA context, see DRV_WIFI_WPS_CONTEXT structure

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_WPS_CONTEXT context;
        uint8_t wpsPin[8] = {1, 2, 3, 9, 0, 2, 1, 2};

        context.wpsSecurityType = DRV_WIFI_SECURITY_WPS_PUSH_BUTTON;
        memcpy(context.wpsPin, wpsPin, sizeof(wpsPin));
        context.wpsPinLength = 8;
        DRV_WIFI_SecurityWpsSet(&context);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_SecurityWpsSet(DRV_WIFI_WPS_CONTEXT *p_context);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_SecurityGet(uint8_t *p_securityType,
                                  uint8_t *p_securityKey,
                                  uint8_t *p_securityKeyLength)

  Summary:
    Gets the current Wi-Fi security setting.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gets the current Wi-Fi security setting.

    <table>
    'securityType' Field                          Key      Length
    --------------------------------------------  -------  ---------------------
    DRV_WIFI_SECURITY_OPEN                        N/A      N/A
    DRV_WIFI_SECURITY_WEP_40                      binary   4 keys, 5 bytes each
                                                            (total of 20 bytes)
    DRV_WIFI_SECURITY_WEP_104                     binary   4 keys, 13 bytes each
                                                            (total of 52 bytes)
    DRV_WIFI_SECURITY_WPA_WITH_KEY                binary   32 bytes
    DRV_WIFI_SECURITY_WPA_WITH_PASS_PHRASE        ascii    8-63 ascii characters
    DRV_WIFI_SECURITY_WPA2_WITH_KEY               binary   32 bytes
    DRV_WIFI_SECURITY_WPA2_WITH_PASS_PHRASE       ascii    8-63 ascii characters
    DRV_WIFI_SECURITY_WPA_AUTO_WITH_KEY           binary   32 bytes
    DRV_WIFI_SECURITY_WPA_AUTO_WITH_PASS_PHRASE   ascii    8-63 ascii characters
    </table>

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_securityType -  value corresponding to the security type desired
                        (see description)
    p_securityKey -  binary key or passphrase (not used if security is
                       DRV_WIFI_SECURITY_OPEN)
    p_securityKeyLength -  number of bytes in p_securityKey (not used if
                             security is DRV_WIFI_SECURITY_OPEN)

  Returns:
    None.

  Example:
    <code>
        uint8_t securityType;
        uint8_t securityKey[DRV_WIFI_MAX_SECURITY_KEY_LENGTH];
        uint8_t keyLength;

        DRV_WIFI_SecurityGet(&securityType, securityKey, &keyLength);
    </code>

  Remarks:
    If security was initially set with a passphrase that the MRF24WG used
    to generate a binary key, this function returns the binary key, not the
    passphrase.
*/
void DRV_WIFI_SecurityGet(uint8_t *p_securityType,
                          uint8_t *p_securityKey,
                          uint8_t *p_securityKeyLength);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_SecurityTypeGet(uint8_t *p_securityType)

  Summary:
    Gets the current Wi-Fi security type.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gets the current Wi-Fi security type.

    <table>
    'securityType' Field
    --------------------------------------------
    DRV_WIFI_SECURITY_OPEN
    DRV_WIFI_SECURITY_WEP_40
    DRV_WIFI_SECURITY_WEP_104
    DRV_WIFI_SECURITY_WPA_WITH_KEY
    DRV_WIFI_SECURITY_WPA_WITH_PASS_PHRASE
    DRV_WIFI_SECURITY_WPA2_WITH_KEY
    DRV_WIFI_SECURITY_WPA2_WITH_PASS_PHRASE
    DRV_WIFI_SECURITY_WPA_AUTO_WITH_KEY
    DRV_WIFI_SECURITY_WPA_AUTO_WITH_PASS_PHRASE
    </table>

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_securityType -  value corresponding to the security type desired
                        (see description)

  Returns:
    None.

  Example:
    <code>
        uint8_t securityType;

        DRV_WIFI_SecurityTypeGet(&securityType);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_SecurityTypeGet(uint8_t *p_securityType);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_WepKeyTypeGet(uint8_t *p_keyType)

  Summary:
    Gets the WEP Key type.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gets the WEP key type:
      * DRV_WIFI_SECURITY_WEP_SHAREDKEY
      * DRV_WIFI_SECURITY_WEP_OPENKEY

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Returns:
    None.

  Example:
    <code>
        uint8_t wepKeyType;

        DRV_WIFI_WepKeyTypeGet(&wepKeyType);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_WepKeyTypeGet(uint8_t *p_wepKeyType);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_WPSCredentialsGet(DRV_WIFI_WPS_CREDENTIAL *p_cred)

  Summary:
    Gets the WPS credentials.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gets the WPS credentials from the MRF24WG.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_WPS_CREDENTIAL cred;

        DRV_WIFI_WPSCredentialsGet(&cred);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_WPSCredentialsGet(DRV_WIFI_WPS_CREDENTIAL *p_cred);

//*******************************************************************************
/*
  Function:
        DRV_WIFI_MGMT_INDICATE_SOFT_AP_EVENT *DRV_WIFI_SoftApEventInfoGet(void)

  Summary:
    Gets the stored Soft AP event info.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function retrieves the additional event info after a Soft AP event has
    occurred.

  Precondition:
    Soft AP event must have occurred.

  Parameters:
    p_event -  pointer to where event info is written, see
                 DRV_WIFI_MGMT_INDICATE_SOFT_AP_EVENT structure

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_MGMT_INDICATE_SOFT_AP_EVENT info;

        DRV_WIFI_WPSCredentialsGet(&info);
    </code>

  Remarks:
    None.
*/
 DRV_WIFI_MGMT_INDICATE_SOFT_AP_EVENT *DRV_WIFI_SoftApEventInfoGet(void);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_SetPSK(uint8_t *p_psk)

  Summary:
    Sets the binary WPA PSK code in WPS.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function is used in conjunction with
    DRV_WIFI_YieldPassphraseToHost. It sends the binary key to the
    MRF24WG after the host has converted an ASCII passphrase to a binary
    key.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_psk -  pointer to the binary key

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_YieldPassphraseToHost(&info);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_SetPSK(uint8_t *p_psk);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_DeviceInfoGet(DRV_WIFI_DEVICE_INFO *p_deviceInfo)

  Summary:
    Retrieves MRF24WG device information.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function retrieves MRF24WG device information. See
    DRV_WIFI_DEVICE_INFO structure.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_deviceInfo -  pointer where device info will be written

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_DEVICE_INFO info;

        DRV_WIFI_DeviceInfoGet(&info);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_DeviceInfoGet(DRV_WIFI_DEVICE_INFO *p_deviceInfo);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_MacAddressSet(uint8_t *p_mac)

  Summary:
    Uses a different MAC address for the MRF24WG.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    Directs the MRF24WG module to use the input MAC address instead of its
    factory-default MAC address. This function does not overwrite the
    factory default, which is in Flash memory.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete. Cannot be called when the
    MRF24WG is in a connected state.

  Parameters:
    p_mac -  pointer to 6 byte MAC that will be sent to MRF24WG

  Returns:
    None.

  Example:
    <code>
        uint8_t mac[6] = {0x00, 0x1e, 0xc0, 0x11, 0x22, 0x33};

        DRV_WIFI_MacAddressSet(mac);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_MacAddressSet(uint8_t *p_mac);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_MacAddressGet(uint8_t *p_mac)

  Summary:
    Retrieves the MRF24WG MAC address.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function retrieves the MRF24WG MAC address.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_mac -  pointer where mac address will be written (must point to a
               6 byte buffer)

  Returns:
    None.

  Example:
    <code>
        uint8_t mac[6];

        DRV_WIFI_MacAddressGet(mac);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_MacAddressGet(uint8_t *p_mac);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_TxPowerMaxSet(uint8_t maxTxPower)

  Summary:
    Sets the TX max power on the MRF24WG0M.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    After initialization the MRF24WG0M max TX power is determined by a
    factory-set value. This function can set a different maximum TX power
    levels. However, this function can never set a maximum TX power greater
    than the factory-set value, which can be read via
    DRV_WIFI_TxPowerFactoryMaxGet.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    maxTxPower -  valid range (0 to 17 dBm)

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_TxPowerMaxSet(8); // set max TX power to 8dBm
    </code>

  Remarks:
    No conversion of units needed, input to MRF24WG0M is in dBm.
*/
void DRV_WIFI_TxPowerMaxSet(uint8_t maxTxPower);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_TxPowerMaxGet(uint8_t *p_maxTxPower)

  Summary:
    Gets the TX max power on the MRF24WG0M.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    Gets the TX max power setting from the MRF24WG.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_maxTxPower -  pointer to where max power setting is written (dBm)

  Returns:
    None.

  Example:
    <code>
        uint8_t maxPower;

        DRV_WIFI_TxPowerMaxGet(&maxPower);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_TxPowerMaxGet(uint8_t *p_maxTxPower);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_TxPowerFactoryMaxGet(int8_t *p_factoryMaxTxPower)

  Summary:
    Retrieves the factory-set max TX power from the MRF24WG module.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function retrieves the factory-set max TX power from the MRF24WG.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_factoryMaxTxPower -  pointer to where factory max power is written (dbM)

  Returns:
    None.

  Example:
    <code>
        uint8_t maxPower;

        DRV_WIFI_TxPowerFactoryMaxGet(&maxPower);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_TxPowerFactoryMaxGet(uint8_t *p_factoryMaxTxPower);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_PsPollEnable(DRV_WIFI_PS_POLL_CONTEXT *p_context)

  Summary:
    Enables PS Poll mode.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    Enables PS Poll mode. PS Poll (Power-Save Poll) is a mode allowing for
    longer battery life. The MRF24WG module coordinates with the Access Point
    to go to sleep and wake up at periodic intervals to check for data messages,
    which the Access Point will buffer. The listenInterval in the Connection
    Algorithm defines the sleep interval. By default, PS Poll mode is disabled.

    When PS Poll is enabled, the Wi-Fi Host Driver will automatically force
    the MRF24WG module to wake up each time the Host sends TX data or a
    control message to the module. When the Host message transaction is
    complete the MRF24WG driver will automatically re-enable PS Poll mode.

    When the application is likely to experience a high volume of data
    traffic then PS Poll mode should be disabled for two reasons:
      1. No power savings will be realized in the presence of heavy data
         traffic.
      2. Performance will be impacted adversely as the Wi-Fi Host Driver
         continually activates and deactivates PS Poll mode via SPI messages.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_context -  pointer to PS Poll context, see DRV_WIFI_PS_POLL_CONTEXT
                   structure

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_PS_POLL_CONTEXT context;

        context.listenInterval = DRV_WIFI_DEFAULT_PS_LISTEN_INTERVAL;
        context.dtimInterval = DRV_WIFI_DEFAULT_PS_DTIM_INTERVAL;
        context.useDtim = DRV_WIFI_DEFAULT_PS_DTIM_ENABLED;

        DRV_WIFI_PsPollEnable(&context);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_PsPollEnable(DRV_WIFI_PS_POLL_CONTEXT *p_context);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_PsPollDisable(void)

  Summary:
    Disables PS-Poll mode.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    Disables PS Poll mode. The MRF24WG module will stay active and not go to
    sleep.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_PsPollDisable(&context);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_PsPollDisable(void);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_PowerSaveStateGet(uint8_t *p_powerSaveState)

  Summary:
    Gets the current power-saving state.

  Description:
    This function gets the current MRF24WG power-saving state.

    <table>
    powerSaveState                      Definition
    ----------------------------------  --------------------------------------
    DRV_WIFI_PS_HIBERNATE               MRF24WG in hibernate state
    DRV_WIFI_PS_PS_POLL_DTIM_ENABLED    MRF24WG in PS-Poll mode with DTIM
                                         enabled
    DRV_WIFI_PS_PS_POLL_DTIM_DISABLED   MRF24WG in PS-Poll mode with DTIM
                                         disabled
    DRV_WIFI_PS_POLL_OFF                MRF24WG is not in any power-saving
                                         state
    </table>

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_powerSaveState -  pointer to where power state is written (see description)

  Returns:
    None.

  Example:
    <code>
      uint8_t state;

      DRV_WIFI_PowerSaveStateGet(&state);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_PowerSaveStateGet(uint8_t *p_powerSaveState);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_HibernateEnable(void)

  Summary:
    Puts the MRF24WG module into hibernate mode.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function enables hibernate mode on the MRF24WG module, which effectively turns off
    the device for maximum power savings.

    MRF24WG module state is not maintained when it transitions to hibernate
    mode.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Returns:
    None.

  Example:
    <code>
      DRV_WIFI_HibernateEnable();
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_HibernateEnable(void);

//*******************************************************************************
/*
  Function:
        bool DRV_WIFI_InHibernateMode(void)

  Summary:
    Checks if MRF24WG is in hibernate mode.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function checks if the MRF24WG is in hibernate mode.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Returns:
    - true  - MRF24WG is in hibernate mode
    - false - MRF24WG is not in hibernate mode

  Example:
    <code>
      bool flag;

      flag = DRV_WIFI_InHibernateMode();
    </code>

  Remarks:
    None.
*/
bool DRV_WIFI_InHibernateMode(void);

//*******************************************************************************
/*
  Function:
        bool DRV_WIFI_HibernateModeClear(void)

  Summary:
    Clears current Hibernate mode.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function clears the current Hibernate mode.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Returns:
    None.

  Remarks:
    None.
*/
void DRV_WIFI_HibernateModeClear(void);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_RtsThresholdSet(uint16_t rtsThreshold)

  Summary:
    Sets the RTS Threshold.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the RTS/CTS packet size threshold for when RTS/CTS frame will be
    sent. The default is 2347 bytes - the maximum for 802.11. It is
    recommended that the user leave the default at 2347 until they
    understand the performance and power ramifications of setting it
    smaller. Valid values are from 0 to DRV_WIFI_RTS_THRESHOLD_MAX (2347).

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    rtsThreshold -  Value of the packet size threshold

  Returns:
    None.

  Example:
    <code>
      DRV_WIFI_RtsThresholdSet(DRV_WIFI_RTS_THRESHOLD_MAX);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_RtsThresholdSet(uint16_t rtsThreshold);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_RtsThresholdGet(uint16_t *p_rtsThreshold)

  Summary:
    Gets the RTS Threshold.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function returns the RTS/CTS packet size threshold.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_rtsThreshold -  pointer to where RTS threshold is written

  Returns:
    None.

  Example:
    <code>
      uint16_t threshold;

      DRV_WIFI_RtsThresholdGet(&threshold);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_RtsThresholdGet(uint16_t *p_rtsThreshold);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_RegionalDomainGet(uint8_t *p_regionalDomain)

  Summary:
    Retrieves the MRF24WG Regional domain.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function returns the regional domain on the MRF24WG. Values are:
      * DRV_WIFI_DOMAIN_FCC
      * DRV_WIFI_DOMAIN_ETSI
      * DRV_WIFI_DOMAIN_JAPAN
      * DRV_WIFI_DOMAIN_OTHER

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_regionalDomain -  pointer where the regional domain value will be
                        written

  Returns:
    None.

  Example:
    <code>
      uint8_t domain;

      DRV_WIFI_RegionalDomainGet(&domain);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_RegionalDomainGet(uint8_t *p_regionalDomain);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_MulticastFilterSet(DRV_WIFI_MULTICAST_CONFIG *p_config)

  Summary:
    Sets a multicast address filter using one of the software multicast
    filters.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function allows the application to configure up to two Multicast
    Address Filters on the MRF24WG module. If two active multicast filters are
    set up they are ORed together - the module will receive and pass to the
    Host CPU received packets from either multicast address. The allowable
    values in p_config are:

    filterId -- DRV_WIFI_MULTICAST_FILTER_1 through
    DRV_WIFI_MULTICAST_FILTER_16

    action -- DRV_WIFI_MULTICAST_DISABLE_ALL (default) The Multicast Filter
    discards all received multicast messages - they will not be forwarded
    to the Host PIC. The remaining fields in this structure are ignored.

    DRV_WIFI_MULTICAST_ENABLE_ALL The Multicast Filter forwards all
    received multicast messages to the Host PIC. The remaining fields in
    this structure are ignored.

    DRV_WIFI_MULTICAST_USE_FILTERS The MAC filter will be used and the
    remaining fields in this structure configure which Multicast messages
    are forwarded to the Host PIC.

    macBytes -- Array containing the MAC address to filter on (using the
    destination address of each incoming 802.11 frame). Specific bytes with
    the MAC address can be designated as "don't care" bytes. See
    macBitMask. This field in only used if action =
    DRV_WIFI_MULTICAST_USE_FILTERS.

    macBitMask -- A byte where bits 5:0 correspond to macBytes[5:0]. If the
    bit is zero then the corresponding MAC byte must be an exact match for
    the frame to be forwarded to the Host PIC. If the bit is one then the
    corresponding MAC byte is a "don't care" and not used in the Multicast
    filtering process. This field in only used if action =
    DRV_WIFI_MULTICAST_USE_FILTERS.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.
    DRV_WIFI_MultiCastFilterEnable() must have been called previously.

  Returns:
    None.

  Example:
    <code>
          DRV_WIFI_MULTICAST_CONFIG config;
          uint8_t macMask[] = {01, 00, 5e, ff, ff, ff};  // (0xff(s) are the don't care bytes)

          // configure software multicast filter 1 to filter multicast addresses that
          // start with 01:00:5e
          config.action = DRV_WIFI_MULTICAST_USE_FILTERS;
          config->filterId = DRV_WIFI_MULTICAST_FILTER_1;
          memcpy(config->macBytes, macMask, 6);
          config->macBitMask = 0x38; // bits 5:3 = 1 (don't care on bytes 3,4,5)
                                     // bits 2:0 = 0 (exact match required on bytes 0, 1, 2)
    </code>

  Remarks:
    Cannot mix hardware and software multicast filters.
*/
void DRV_WIFI_MulticastFilterSet(DRV_WIFI_MULTICAST_CONFIG *p_config);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_MacStatsGet(DRV_WIFI_MAC_STATS *p_macStats)

  Summary:
    Gets MAC statistics.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gets the various MAC layer stats as maintained by the
    MRF24WG.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_macStats -  pointer to where MAC statistics are written, see
                    DRV_WIFI_MAC_STATS structure.

  Returns:
    None.

  Example:
    <code>
      DRV_WIFI_MAC_STATS macStats;

      DRV_WIFI_MacStatsGet(&macStats);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_MacStatsGet(DRV_WIFI_MAC_STATS *p_macStats);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_ScanContextSet(DRV_WIFI_SCAN_CONTEXT *p_context)

  Summary:
    Sets the Wi-Fi scan context.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sets the Wi-Fi scan context. The MRF24WG defaults are
    fine for most applications, but they can be changed by this function.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_context -  pointer to scan context, see DRV_WIFI_SCAN_CONTEXT
                   structure.

  Returns:
    None.

  Example:
    <code>
      DRV_WIFI_SCAN_CONTEXT context;

      context.scantype = DRV_WIFI_ACTIVE_SCAN;
      context.scanCount      = 1;
      context.minChannelTime = 200; // ms
      context.maxChannelTime = 400; // ms
      context.probeDelay     = 20; // uS

      DRV_WIFI_ScanContextSet(&context);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_ScanContextSet(DRV_WIFI_SCAN_CONTEXT *p_context);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_ScanContextGet(DRV_WIFI_SCAN_CONTEXT *p_context)

  Summary:
    Gets the Wi-Fi scan context.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gets the Wi-Fi scan context.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    p_context -  pointer to where scan context will be written, see
                   DRV_WIFI_SCAN_CONTEXT structure

  Returns:
    None.

  Example:
    <code>
      DRV_WIFI_SCAN_CONTEXT context;

      DRV_WIFI_ScanContextSet(&context);
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_ScanContextGet(DRV_WIFI_SCAN_CONTEXT *p_context);

//*******************************************************************************
/*
  Function:
        uint16_t DRV_WIFI_Scan(bool scanAll)

  Summary:
    Commands the MRF24WG module to start a scan operation. This will generate
    the WF_EVENT_SCAN_RESULTS_READY event.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function directs the MRF24WG module to initiate a scan operation. The Host
    Application will be notified that the scan results are ready when it
    receives the WF_EVENT_SCAN_RESULTS_READY event. The eventInfo field for
    this event will contain the number of scan results. Once the scan results
    are ready they can be retrieved with DRV_WIFI_ScanResultGet().

    Scan results are retained on the MRF24WG until:
      1. Calling DRV_WIFI_Scan() again (after scan results returned from
         previous call).
      2. MRF24WG reset.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    scanAll -  If false:
                 * If SSID defined then only scan results with that SSID are
                     retained.
                 * If SSID not defined then all scanned SSIDs will be retained.
                 * Only scan results from Infrastructure or Ad-Hoc networks are
                     retained.
                 * The channel list that is scanned will be determined from
                     the channels passed in via DRV_WIFI_ChannelListSet().

               If true:
                 * All scan results are retained (both Infrastructure and Ad-Hoc networks).
                 * All channels within the MRF24WG's regional domain will be scanned.

  Returns:
    None.

  Example:
    <code>
      DRV_WIFI_Scan(true);
    </code>

  Remarks:
    None.
*/
uint16_t DRV_WIFI_Scan(bool scanAll);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_ScanResultGet(uint8_t listIndex, t_wfScanResult *p_scanResult)

  Summary:
    Read selected scan results back from MRF24WG.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    After a scan has completed this function is used to read one scan
    result at a time from the MRF24WG.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete. WF_EVENT_SCAN_RESULTS_READY
    event must have already occurred.

  Parameters:
    listIndex -  index (0 based list) of the scan entry to retrieve
    p_scanResult -  pointer to where scan result is written, see
                      DRV_WIFI_SCAN_RESULT structure

  Returns:
    None.

  Example:
    <code>
      DRV_WIFI_SCAN_RESULT scanResult;

      DRV_WIFI_ScanResultGet(0, &scanResult); // get first scan result in list
    </code>

  Remarks:
    None.
*/
void DRV_WIFI_ScanResultGet(uint8_t listIndex, DRV_WIFI_SCAN_RESULT *p_scanResult);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_ProcessEvent(uint16_t event, uint16_t eventInfo)

  Summary:
    Processes Wi-Fi event.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function is called to process a Wi-Fi event.

  Precondition:
    TCPIP stack should be initialized.

  Parameters:
    event -  event code
    eventInfo -  additional information about the event; not all events have
                   associated info, in which case this value will be set to
                   DRV_WIFI_NO_ADDITIONAL_INFO (0xff)

  Returns:
    None.

  Remarks:
    None.
*/
void DRV_WIFI_ProcessEvent(uint16_t event, uint16_t eventInfo);

//*******************************************************************************
/*
  Function:
        bool DRV_WIFI_ConfigDataLoad(void)

  Summary:
    Loads configuration data from the board EEPROM.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function loads configuration data from the board EEPROM. If not
    present or corrupted then default values will be used.

  Precondition:
    The TCP/IP stack should be initialized.

  Returns:
    - true  - Configuration data was loaded
    - false - Configuration data was not loaded

  Remarks:
    None.
*/
bool DRV_WIFI_ConfigDataLoad(void);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_ConfigDataSave(void)

  Summary:
    Save configuration data to the board EEPROM.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function saves configuration data to the board EEPROM.

  Precondition:
    The TCP/IP stack should be initialized.

  Returns:
    None.

  Remarks:
    None.
*/
void DRV_WIFI_ConfigDataSave(void);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_ConfigDataDelete(void)

  Summary:
    Erases configuration data from the board EEPROM.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function erases configuration data from the board EEPROM.

  Precondition:
    The TCP/IP stack should be initialized.

  Returns:
    None.

  Remarks:
    None.
*/
void DRV_WIFI_ConfigDataDelete(void);

//*******************************************************************************
/*
  Function:
        TCPIP_MAC_RES DRV_WIFI_ContextLoad(void)

  Summary:
    Loads Wi-Fi context to MRF24WG.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function loads Wi-Fi context to MRF24WG.

  Precondition:
    The TCP/IP stack should be initialized.

  Returns:
    TCP/IP stack MAC result.

  Remarks:
    None.
*/
TCPIP_MAC_RES DRV_WIFI_ContextLoad(void);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_INT_Handle(void)

  Summary:
    MRF24WG Wi-Fi driver interrupt handle.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function is the interrupt handle of MRF24WG Wi-Fi driver.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Returns:
    None.

  Remarks:
    None.
*/
void DRV_WIFI_INT_Handle(void);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_MRF24W_ISR(SYS_MODULE_OBJ index)

  Summary:
    MRF24WG Wi-Fi driver interrupt service routine.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function is MRF24WG Wi-Fi driver interrupt service routine.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Returns:
    None.

  Remarks:
    None.
*/
void DRV_WIFI_MRF24W_ISR(SYS_MODULE_OBJ index);

//*******************************************************************************
/*
  Function:
        bool DRV_WIFI_SpiInit(bool (*rx)(unsigned char *buf, uint32_t size),
                                    bool (*tx)(unsigned char *buf, uint32_t size))

  Summary:
    Initializes SPI object for MRF24WG Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function initializes SPI object for MRF24WG Wi-Fi driver.

  Precondition:
    TCP/IP stack should be initialized.

  Parameters:
    rx -  function pointer to RX API
    tx -  function pointer to TX API

  Returns:
    - true  - The SPI object was initialized
    - false - The SPI object was not initialized

  Remarks:
    None.
*/
bool DRV_WIFI_SpiInit(bool (*rx)(unsigned char *buf, uint32_t size),
                      bool (*tx)(unsigned char *buf, uint32_t size));

//*******************************************************************************
/*
  Function:
        bool DRV_WIFI_SpiDmaTx(unsigned char *buf, uint32_t size)

  Summary:
    SPI TX API using DMA.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sends data to the module using DMA over SPI bus.

  Precondition:
    The TCP/IP stack should be initialized.

  Parameters:
    buf -  buffer pointer to the data to be sent
    size -  the data size

  Returns:
    - true  - Data was sent
    - false - Data was not sent

  Remarks:
    None.
*/
bool DRV_WIFI_SpiDmaTx(unsigned char *buf, uint32_t size);

//*******************************************************************************
/*
  Function:
        DRV_WIFI_SpiDmaRx(unsigned char *buf, uint32_t size)

  Summary:
    SPI RX API using DMA.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function receives data from the module using DMA over SPI bus.

  Precondition:
    TCP/IP stack should be initialized.

  Parameters:
    buf -  buffer pointer to the data to be received
    size -  the data size

  Returns:
    - true  - Data was received
    - false - Data was not received

  Remarks:
    None.
*/
bool DRV_WIFI_SpiDmaRx(unsigned char *buf, uint32_t size);

//*******************************************************************************
/*
  Function:
        bool DRV_WIFI_SpiTx(unsigned char *buf, uint32_t size)

  Summary:
    SPI TX API
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function sends data to the module over SPI bus.

  Precondition:
    The TCP/IP stack should be initialized.

  Parameters:
    buf -  buffer pointer to the data to be sent
    size -  the data size

  Returns:
    - true  - Data was sent
    - false - Data was not sent

  Remarks:
    None.
*/
bool DRV_WIFI_SpiTx(unsigned char *buf, uint32_t size);

//*******************************************************************************
/*
  Function:
        DRV_WIFI_SpiRx(unsigned char *buf, uint32_t size)

  Summary:
    SPI Rx API.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function receives data from the module over SPI bus.

  Precondition:
    The TCP/IP stack should be initialized.

  Parameters:
    buf -  buffer pointer to the data to be received
    size -  the data size

  Returns:
    - true  - Data was received
    - false - Data was not received

  Remarks:
    None.
*/
bool DRV_WIFI_SpiRx(unsigned char *buf, uint32_t size);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_SpiClose(void)

  Summary:
    Closes SPI object for MRF24WG Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function closes SPI object for MRF24WG Wi-Fi driver.

  Precondition:
    The TCP/IP stack should be initialized.

  Returns:
    None.

  Remarks:
    None.
*/
void DRV_WIFI_SpiClose(void);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_RSSI_Cache_FromRxDataRead(uint16_t rssi)

  Summary:
    Caches RSSI value from RX data packet.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function caches RSSI value from RX data packet.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    rssi -  RSSI value read from RX data packet

  Returns:
    None.

  Remarks:
    None.
*/
void DRV_WIFI_RSSI_Cache_FromRxDataRead(uint16_t rssi);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_RSSI_Get_FromRxDataRead(uint16_t *mean, uint16_t *last)

  Summary:
    Reads RSSI value from RX data packet.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function reads RSSI value from RX data packet.

  Precondition:
    MRF24WG Wi-Fi initialization must be complete.

  Parameters:
    mean -  pointer to where the average mean RSSI value to be stored
    last -  pointer to where the total count of RSSI values to be stored

  Returns:
    - mean -  the calculated mean RSSI
    - last -  the total count of RSSI values

  Remarks:
    None.
*/
void DRV_WIFI_RSSI_Get_FromRxDataRead(uint16_t *mean, uint16_t *last);

//*******************************************************************************
/*
  Function:
        bool DRV_WIFI_TaskSyncInit(void)

  Summary:
    Initializes RTOS Semaphore and Mutex for MRF24WG Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function initializes RTOS Semaphore and Mutex for MRF24WG Wi-Fi driver.

  Precondition:
    The TCP/IP stack should be initialized.

  Returns:
    - true  - Initialization was successful
    - false - Initialization was not successful

  Remarks:
    None.
*/
bool DRV_WIFI_TaskSyncInit(void);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_Deferred_ISR(void *p_arg)

  Summary:
    Implements MRF24WG Wi-Fi driver deferred ISR.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function implements MRF24WG Wi-Fi driver deferred ISR.

  Precondition:
    The TCP/IP stack should be initialized.

  Returns:
    None.

  Remarks:
    None.
*/
void DRV_WIFI_Deferred_ISR(void *p_arg);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_InitTask(void *p_arg)

  Summary:
    Implements MRF24WG Wi-Fi driver initialization RTOS task.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function implements MRF24WG Wi-Fi driver initialization RTOS task.

  Precondition:
    The TCP/IP stack should be initialized.

  Returns:
    None.

  Remarks:
    None.
*/
void DRV_WIFI_InitTask(void *p_arg);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_MACTask(void *p_arg)

  Summary:
    Implements MRF24WG Wi-Fi driver MAC process RTOS task.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function implements MRF24WG Wi-Fi driver MAC process RTOS task.

  Precondition:
    The TCP/IP stack should be initialized.

  Returns:
    None.

  Remarks:
    None.
*/
void DRV_WIFI_MACTask(void *p_arg);

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_DeferredISR_SemGive(void)

  Summary:
    Gives semaphore to Wi-Fi deferred ISR.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gives semaphore to Wi-Fi Deferred ISR.

  Precondition:
    The TCP/IP stack should be initialized.

  Returns:
    None.

  Remarks:
    None.
*/
void DRV_WIFI_DeferredISR_SemGive(void);

// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

#endif /* _DRV_WIFI_H */
