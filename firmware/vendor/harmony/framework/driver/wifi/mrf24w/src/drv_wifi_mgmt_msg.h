/*******************************************************************************
  MRF24WG Driver Management Messages (Specific to the MRF24WG)

  File Name:
    drv_wifi_mgmt_msg.h

  Summary:
    MRF24WG Driver Management Messages (Specific to the MRF24WG)

  Description:
    The functions in this header file are accessed by the TCP/IP stack via
    function pointers.
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

#ifndef _DRV_WIFI_MGMT_MSG_H
#define _DRV_WIFI_MGMT_MSG_H

#define CPID (1)

extern uint16_t g_mgmt_base; // mgmt msg base index in scratch memory

/* 
 * Locations in scratch memory where tx and rx managment messages are located:
 *
 *            Scratch Memory Map for Mgmt Messages
 *
 * For instance, say g_mgmt_base is 4096, then:
 *
 *  Index
 *  -----
 *           |-----------------------------------|
 *   4096    |                                   |
 *           | Mgmt Tx Message (128 bytes)       |    host --> MRF24WG
 *   4223    |                                   |
 *           |-----------------------------------|
 *   4224    |                                   |
 *           | Mgmt Resp Message (128 bytes)     |    MRF24WG --> host
 *   4351    | (MRF24WG to host)                 |
 *           |-----------------------------------|
 *   4352    |                                   |
 *           | Mgmt Indicate Message (128 bytes) |    MRF24WG --> host
 *   4479    | (MRF24WG to host)                 |
 *           |-----------------------------------|
 *   4480    | Mgmt Resp Ack (1 byte)            |
 *           |-----------------------------------|
 *
 */

#define MGMT_BUF_SIZE               ((uint16_t)128)                      // all mgmt msgs fit within 128 bytes
#define MGMT_ACK_SIZE               ((uint16_t)1)                        // all mgmt ack's are one byte
#define MGMT_TX_BASE                ((uint16_t)g_mgmt_base)              // start of mgmt tx buffer in scratch (host -> MRF24WG)
#define MGMT_RX_BASE                (MGMT_TX_BASE + MGMT_BUF_SIZE)       // start of mgmt response buffer in scratch (MRF24WG -> host)
#define MGMT_INDICATE_BASE          (MGMT_RX_BASE + MGMT_BUF_SIZE)       // start of mgmt indicate buffer in scratch (MRF24WG -> host)
#define MGMT_RX_ACK_BASE            (MGMT_INDICATE_BASE + MGMT_BUF_SIZE) // signals MRF24WG that mgmt response read by host
#define MGMT_INDICATE_ACK_BASE      (MGMT_RX_ACK_BASE + MGMT_ACK_SIZE)   // signals MRF24WG that mgmt indicate read by host

#define MGMT_RESPONSE_SET_FLAG      ((uint8_t)0xa5)       // written by MRF24WG to signal new mgmt response
#define MGMT_RESPONSE_CLEAR_FLAG    ((uint8_t)0x5a)       // written by host to signal mgmt response has been read

#define MGMT_INDICATE_CLEAR_FLAG    ((uint8_t)0x5a)       // written by host to signal mgmt indicate has been read
#define MGMT_INDICATE_SET_FLAG      ((uint8_t)0xa5)       // written by MRF24WG to signal new mgmt indicate

/*----------------------------------------------*/
/* Management Message Request/Response Subtypes */
/*----------------------------------------------*/
typedef enum
{
    /* Misc subtypes */
    WF_SCAN_SUBTYPE                             = 1,
    WF_JOIN_SUBTYPE                             = 2,
    WF_AUTH_SUBTYPE                             = 3,
    WF_ASSOC_SUBTYPE                            = 4,
    WF_DISCONNECT_SUBTYPE                       = 5,
    WF_DISASOCC_SUBTYPE                         = 6,
    WF_SET_POWER_MODE_SUBTYPE                   = 7,
    WF_SET_PM_KEY_SUBTYPE                       = 8,
    WF_SET_WEP_MAP_SUBTYPE                      = 9,
    WF_SET_WEP_KEY_SUBTYPE                      = 10,
    WF_SET_TEMP_KEY_SUBTYPE                     = 11,
    WF_CALC_PSK_KEY_SUBTYPE                     = 12,
    WF_SET_WEP_KEY_ID_SUBTYPE                   = 13,
    WF_CONFIG_KEY_SPACE_SUBTYPE                 = 14,
    WF_SET_PARAM_SUBTYPE                        = 15,
    WF_GET_PARAM_SUBTYPE                        = 16,
    WF_ADHOC_CONNECT_SUBTYPE                    = 17,
    WF_ADHOC_START_SUBTYPE                      = 18,

    /* Connection Profile Message Subtypes */
    WF_CP_CREATE_PROFILE_SUBTYPE                = 21,
    WF_CP_DELETE_PROFILE_SUBTYPE                = 22,
    WF_CP_GET_ID_LIST_SUBTYPE                   = 23,
    WF_CP_SET_ELEMENT_SUBTYPE                   = 24,
    WF_CP_GET_ELEMENT_SUBTYPE                   = 25,

    /* Connection Algorithm Message Subtypes */
    WF_CA_SET_ELEMENT_SUBTYPE                   = 26,
    WF_CA_GET_ELEMENT_SUBTYPE                   = 27,

    /* Connnection Manager Message Subtypes */
    WF_CM_CONNECT_SUBYTPE                       = 28,
    WF_CM_DISCONNECT_SUBYTPE                    = 29,
    WF_CM_GET_CONNECTION_STATUS_SUBYTPE         = 30,

    WF_SCAN_START_SUBTYPE                       = 31,
    WF_SCAN_GET_RESULTS_SUBTYPE                 = 32,

    WF_CM_INFO_SUBTYPE                          = 33,

    WF_SCAN_FOR_IE_SUBTYPE                      = 34, /* not yet supported */
    WF_SCAN_IE_GET_RESULTS_SUBTYPE              = 35, /* not yet supported */

    WF_CM_GET_CONNECTION_STATISTICS_SUBYTPE     = 36, /* not yet supported so moved here for now */
    WF_NUM_REQUEST_SUBTYPES

} tMgmtMsgSubtypes;

/*-------------------------------------------------------------*/
/* Connection Profile Element ID's                             */
/* Used in conjunction with the WF_CP_SET_ELEMENT_SUBTYPE and  */
/* WF_CP_GET_ELEMENT_SUBTYPE message subtypes                  */
/*-------------------------------------------------------------*/
typedef enum
{
    WF_CP_ELEMENT_SSID              = 1,
    WF_CP_ELEMENT_BSSID             = 2,
    WF_CP_ELEMENT_SECURITY          = 3,
    WF_CP_ELEMENT_NETWORK_TYPE      = 4,
    WF_CP_ELEMENT_ADHOC_BEHAVIOR    = 5,
    WF_CP_ELEMENT_WEP_KEY_INDEX     = 6,
    WF_CP_ELEMENT_SSID_TYPE         = 7,
    WF_CP_ELEMENT_WEPKEY_TYPE       = 8,
    WF_CP_ELEMENT_UPDATE_PMK        = 9,
    WF_CP_ELEMENT_READ_WPS_CRED     = 10
} tCPElementIds;

/*-------------------------------------------------------------*/
/* Connection Algorithm Element ID's                           */
/* Used in conjunction with the WF_CA_SET_ELEMENT_SUBTYPE and  */
/* WF_CA_GET_ELEMENT_SUBTYPE message subtypes                  */
/*-------------------------------------------------------------*/
typedef enum
{
    WF_CA_ELEMENT_SCANTYPE                     = 1,
    WF_CA_ELEMENT_RSSI                         = 2,
    WF_CA_ELEMENT_CP_LIST                      = 3,
    WF_CA_ELEMENT_LIST_RETRY_COUNT             = 4,
    WF_CA_ELEMENT_EVENT_NOTIFICATION_ACTION    = 5,
    WF_CA_ELEMENT_BEACON_TIMEOUT_ACTION        = 6,
    WF_CA_ELEMENT_DEAUTH_ACTION                = 7,
    WF_CA_ELEMENT_CHANNEL_LIST                 = 8,
    WF_CA_ELEMENT_LISTEN_INTERVAL              = 9,
    WF_CA_ELEMENT_BEACON_TIMEOUT               = 10,
    WF_CA_ELEMENT_SCAN_COUNT                   = 11,
    WF_CA_ELEMENT_MIN_CHANNEL_TIME             = 12,
    WF_CA_ELEMENT_MAX_CHANNEL_TIME             = 13,
    WF_CA_ELEMENT_PROBE_DELAY                  = 14,
    WF_CA_ELEMENT_DTIM_INTERVAL                = 15,
    WF_CA_ELEMENT_BEACON_PERIOD                = 16
} tCAElementIds;

/* t_wfParam - Names (ID's) of MRF24WG Wi-Fi MAC configurable parameters. */
typedef enum
{
    PARAM_MAC_ADDRESS                 = 1,       /* the device MAC address (6 bytes)                            */
    PARAM_REGIONAL_DOMAIN             = 2,       /* the device Regional Domain (1 byte)                         */
    PARAM_RTS_THRESHOLD               = 3,       /* the RTS byte threshold 256 - 2347 (2 bytes)                 */
    PARAM_LONG_FRAME_RETRY_LIMIT      = 4,       /* the long Frame Retry limit  (1 byte)                        */
    PARAM_SHORT_FRAME_RETRY_LIMIT     = 5,       /* the short Frame Retry limit (1 byte)                        */
    PARAM_TX_LIFETIME_TU              = 6,       /* the Tx Request lifetime in TU's 0 - 4194303 (4 bytes)       */
    PARAM_RX_LIFETIME_TU              = 7,       /* the Rx Frame lifetime in TU's 0 - 4194303 (4 bytes)         */
    PARAM_SUPPLICANT_ON_OFF           = 8,       /* boolean 1 = on 0 = off (1 byte)                             */
    PARAM_CONFIRM_DATA_TX_REQ         = 9,       /* boolean 1 = on 0 = off (1 byte)                             */
    PARAM_MASTER_STATE                = 10,      /* master state of the MAC using enumerated values (1 byte)    */
    PARAM_HOST_ALERT_BITS             = 11,      /* a bit field which enables/disables various asynchronous     */
                                                 /*   indications from the MAC to the host (2 bytes)            */
    PARAM_NUM_MISSED_BEACONS          = 12,      /* number of consecutive beacons MAC can miss before it        */
                                                 /*   considers the network lost (1 byte)                       */
    PARAM_DIFS_AND_EIFS               = 13,      /* delay intervals in usec DIFS and EIFS ( 2 * 2 bytes)        */
    PARAM_TX_POWER                    = 14,      /* max and min boundaries for Tx power (2 * 2 bytes)           */
    PARAM_DEFAULT_DEST_MAC_ADDR       = 15,      /* stores a persistant destination MAC address for small       */
                                                 /*   Tx Requests (6 bytes)                                     */
    PARAM_WPA_INFO_ELEMENT            = 16,      /* stores a WPA info element (IE) in 802.11 IE format.  Used   */
                                                 /*   in Assoc Request and Supplicant exchange (3 - 258 bytes)  */
    PARAM_RSN_INFO_ELEMENT            = 17,      /* stores a RSN info element (IE) in 802.11 IE format.  Used   */
                                                 /*   in Assoc Request and Supplicant exchange (3 - 258 bytes)  */
    PARAM_ON_OFF_RADIO                = 18,      /* bool to force a radio state change 1 = on 0 = off (1 byte)  */
    PARAM_COMPARE_ADDRESS             = 19,      /* a MAC address used to filter received frames                */
                                                 /*   (sizeof(tAddressFilterInput) = 8 bytes)                   */
    PARAM_SUBTYPE_FILTER              = 20,      /* bitfield used to filter received frames based on type and   */
                                                 /* sub-type (sizeof(tAddressFilterInput) = 4 bytes)            */
    PARAM_ACK_CONTROL                 = 21,      /* bitfield used to control the type of frames that cause ACK  */
                                                 /*   responses (sizeof(tAckControlInput) = 4 bytes)            */
    PARAM_STAT_COUNTERS               = 22,      /* complete set of statistics counters that are maintained by  */
                                                 /*   the MAC                                                   */
    PARAM_TX_THROTTLE_TABLE           = 23,      /* custom Tx Rate throttle table to be used to control tx Rate */
    PARAM_TX_THROTTLE_TABLE_ON_OFF    = 24,      /* a boolean to enable/disable use of the throttle Table and a */
                                                 /*   tx rate to use if the throttle table is disabled          */
    PARAM_TX_CONTENTION_ARRAY         = 25,      /* custom Retry contention ladder used for backoff calculation */
                                                 /*   prior to a Tx attempt                                     */
    PARAM_SYSTEM_VERSION              = 26,      /* 2 bytes representation of a version number for the ROM and  */
                                                 /*  Patch                                                      */
    PARAM_STATUE_INFO                 = 27,      /* MAC State information                                       */
    PARAM_SECURITY_CONTROL            = 28,      /* 2 bytes data structure to enable/disable encryption         */
    PARAM_FACTORY_SET_TX_MAX_POWER    = 29,      /* gets the factory-set tx max power level                     */
    PARAM_CONNECT_CONTEXT             = 31,      /* gets current connection status                              */
    PARAM_TX_MODE                     = 34,      /* choose tx mode                                              */
    PARAM_LINK_DOWN_THRESHOLD         = 37,      /* sets link down threshold                                    */
    PARAM_SET_PSK                     = 39,      /* set psk                                                     */
    PARAM_SET_HOST_DERIVE_KEY         = 40,      /* has host derive key from passphrase                         */
    PARAM_SET_MULTICAST_FILTER        = 41       /* set multicast filter                                        */
} t_wfParam;

/* used in byte 2 of WF_CONNECTION_LOST_EVENT_SUBTYPE */
#define WF_CONNECTION_TEMPORARILY_LOST  ((uint8_t)0)
#define WF_CONNECTION_PERMANENTLY_LOST  ((uint8_t)1)

#define WF_FLASH_UPDATE_NOT_SUCCESSFUL ((uint8_t)0)
#define WF_FLASH_UPDATE_SUCCESSFUL     ((uint8_t)1)

#define WF_MAX_TX_MGMT_MSG_SIZE (128)

#define DO_NOT_FREE_MGMT_BUFFER (0)
#define FREE_MGMT_BUFFER (1)

#define MGMT_RESP_1ST_DATA_BYTE_INDEX (4) /* first data byte of Mgmt response starts at index 4 */

#if defined(SYS_DEBUG_ENABLE)
/* this block of defines is used to check illegal reentry when in WF API functions */
#define WF_ENTERING_FUNCTION (1)
#define WF_LEAVING_FUNCTION (0)

/* bit masks for functions that need to be tracked when they are called */
#define WF_PROCESS_EVENT_FUNC ((uint8_t)0x01)

#endif /* SYS_DEBUG_ENABLE */

/*==========================================================================*/
/*                                  TYPEDEFS                                */
/*==========================================================================*/

/* This structure describes the format of the first four bytes of all */
/* mgmt response messages received from the MRF24W                 */
typedef struct mgmtRxHdrStruct
{
    uint8_t  type; /* always 0x02 */
    uint8_t  subtype; /* mgmt msg subtype */
    uint8_t  result; /* 1 if success, else failure */
    uint8_t  macState; /* not used */
} tMgmtMsgRxHdr;

typedef struct mgmtIndicateHdrStruct
{
    uint8_t type; /* always WF_MGMT_INDICATE_MSG_TYPE (2) */
    uint8_t subType; /* event type */
} tMgmtIndicateHdr;

typedef struct mgmtIndicatePassphraseReady
{
    uint8_t keyLen;
    uint8_t key[DRV_WIFI_MAX_SECURITY_KEY_LENGTH];
    uint8_t ssidLen;
    uint8_t ssid[DRV_WIFI_MAX_SSID_LENGTH];
} tMgmtIndicatePassphraseReady;

extern tMgmtIndicatePassphraseReady g_passphraseReady;

/*==========================================================================*/
/*                                  FUNCTION PROTOTYPES                     */
/*==========================================================================*/

void WaitForMgmtResponseAndReadData(uint8_t expectedSubtype,
                                    uint8_t numDataBytes,
                                    uint8_t startIndex,
                                    uint8_t *p_data);

void SendMgmtMsg(uint8_t *p_header,
                 uint8_t headerLength,
                 uint8_t *p_data,
                 uint8_t dataLength);

void   WaitForMgmtResponse(uint8_t expectedSubtype, uint8_t freeAction);

void SetParamMsgSend(uint8_t paramType, uint8_t *p_paramData, uint8_t paramDataLength);

void GetParamMsgSend(uint8_t paramType, uint8_t *p_paramData, uint8_t paramDataLength);

void DRV_WIFI_MgmtIndProcess(void);

void MgmtRespReceivedSet(void);

#endif /* _DRV_WIFI_MGMT_MSG_H */

// DOM-IGNORE-END
