/*************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    wdrv_mrf24wn_common.h

  Summary:
    MRF24WN Driver Common Header File

  Description:

 *************************************************************************/

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

#ifndef _WDRV_MRF24WN_COMMON_H
#define _WDRV_MRF24WN_COMMON_H

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "osal/osal.h"
#include "system/tmr/sys_tmr.h"

#include "wdrv_mrf24wn_priv.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus // Provide C++ Compatibility
    extern "C" {
#endif
// DOM-IGNORE-END

#if !defined(TCPIP_STACK_USE_EVENT_NOTIFICATION)
    #error "TCPIP_STACK_USE_EVENT_NOTIFICATION must be defined for Wi-Fi demos"
#endif

#define WDRV_NETWORK_TYPE_INFRASTRUCTURE 1
#define WDRV_NETWORK_TYPE_ADHOC          2 /* TODO: Invalid for now. Validation is TBD. */
#define WDRV_NETWORK_TYPE_P2P            3 /* TODO: Invalid for now. Validation is TBD. */
#define WDRV_NETWORK_TYPE_SOFT_AP        4

// Do not make this an enumerated type!
#define WDRV_FUNC_ENABLED 1
#define WDRV_FUNC_DISABLED 0

#define WDRV_BSSID_LENGTH 6
#define WDRV_MAX_SSID_LENGTH 32

#define WDRV_MAX_SECURITY_KEY_LENGTH 64

/* Key size defines */
#define WDRV_MIN_WPA_PASS_PHRASE_LENGTH 8
#define WDRV_MAX_WPA_PASS_PHRASE_LENGTH 64 // must include string terminator
#define WDRV_MAX_WPA2_PASS_PHRASE_LENGTH 64

// WEP key lengths
#define WDRV_WEP40_KEY_LENGTH 20 // 4 keys of 5 bytes each
#define WDRV_WEP104_KEY_LENGTH 52 // 4 keys of 13 bytes each
#define WDRV_MAX_WEP_KEY_LENGTH WDRV_WEP104_KEY_LENGTH

// WPS PIN Length
#define WDRV_SECURITY_OPEN                     0
#define WDRV_SECURITY_WEP_40                   1
#define WDRV_SECURITY_WEP_104                  2
#define WDRV_SECURITY_WPA_WITH_PASS_PHRASE     3
#define WDRV_SECURITY_WPA2_WITH_PASS_PHRASE    4
#define WDRV_SECURITY_WPS_PUSH_BUTTON          6
#define WDRV_SECURITY_WPS_PIN                  7

#define WDRV_APCONFIG_BIT_PRIVACY              (0x10)
#define WDRV_APCONFIG_BIT_PREAMBLE_LONG        (0x20)
#define WDRV_APCONFIG_BIT_WPA                  (0x40)
#define WDRV_APCONFIG_BIT_WPA2                 (0x80)

#define WDRV_WPS_PIN_LENGTH                    8 // 7 digits + checksum byte

#define MAX_ALLOWED_BAUD_RATE 25000000

typedef uint32_t *WDRV_MUTEX_HANDLE_TYPE;

#define WDRV_QUEUE_SUCCESS 0
#define WDRV_QUEUE_EMPTY 1
#define WDRV_QUEUE_FULL 2
#define WDRV_QUEUE_FAILURE -1

// board type
#define WDRV_BD_TYPE_MZ_ESK 1
#define WDRV_BD_TYPE_MX_ESK 2
#define WDRV_BD_TYPE_EXP16  3
#define WDRV_BD_TYPE_MEB2   4
#define WDRV_BD_TYPE_CUSTOM 5

/*******************************************************************************
  Summary:
    Contains data pertaining to Wi-Fi scan results

  Description:
    Wi-Fi Scan Results

    This structure contains the result of Wi-Fi scan operation. See
    WDRV_CLI_ScanGetResult().

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
    uint8_t      bssid[WDRV_BSSID_LENGTH];

    /*  Network SSID value */
    uint8_t      ssid[WDRV_MAX_SSID_LENGTH];

    /* Access Point configuration (see description) */
    uint8_t      apConfig;

    /* not used */
    uint8_t      reserved;

    /* Network beacon interval */
    uint16_t     beaconPeriod;

    /* Only valid if bssType = WDRV_NETWORK_TYPE_INFRASTRUCTURE */
    uint16_t     atimWindow;

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
    uint8_t      basicRateSet[8];

    uint8_t      extRateSet[4];

    /* Signal strength of received frame beacon or probe response.  Will range
       from a low of 43 to a high of 128. */
    uint8_t      rssi;

    /* Number of valid rates in basicRates */
    uint8_t      numRates;

    /* Part of TIM element */
    uint8_t      dtimPeriod;

    /* WDRV_NETWORK_TYPE_INFRASTRUCTURE or WDRV_NETWORK_TYPE_ADHOC */
    uint8_t      bssType;

    /* Channel number */
    uint8_t      channel;

    /* Number of valid characters in SSID */
    uint8_t      ssidLen;

} WDRV_SCAN_RESULT;

typedef struct
{
    uint8_t scanState;
    uint16_t numberOfResults;
    uint16_t displayIdx;
} WDRV_SCAN_STATUS;

typedef struct __attribute__((__packed__))
{
    uint32_t verifyFlag; // 0x00000000: empty;    0xffffffff: empty;    0x5a5a5a5a: verified.
    uint8_t networkType;
    uint8_t ssid[WDRV_MAX_SSID_LENGTH];
    uint8_t ssidLen;
    uint8_t securityMode; // WDRV_SECURITY_OPEN or one of the other security modes
    uint8_t securityKey[WDRV_MAX_SECURITY_KEY_LENGTH]; // Wi-Fi security key, or passphrase
    uint8_t securityKeyLen; // number of bytes in security key (can be 0)
} WDRV_CONFIG_DATA;

/* Intentionally define MRF24WN's device type to 3 to provide backward
 * compatibility with MRF24WG.
 */
typedef enum
{
    WDRV_MRF24WN0M_DEVICE = 3
} WDRV_DEVICE_TYPE;

typedef struct
{
    uint8_t deviceType;
    uint8_t romVersion;
    uint8_t patchVersion;
} WDRV_DEVICE_INFO;

/*******************************************************************************
  Summary:
    Contains data pertaining to MRF24WN connection context

  Description:
    Wi-Fi Connection Context

    This structure contains MRF24WN connection context data.
*/
typedef struct
{
    /* channel number of current connection */
    uint16_t channel;

    /* bssid of connected AP */
    uint8_t bssid[6];

} WDRV_CONNECTION_CONTEXT;

/*******************************************************************************
  Summary:
    Contains data pertaining to Wi-Fi scan context

  Description:
    Wi-Fi Scan Context

    This structure contains MRF24WN scan context data.
*/
typedef struct
{
    /* 802.11 allows for active scanning, where the device sends out a broadcast
     probe request seeking an access point.  Also allowed is passive scanning
     where the device only listens to beacons being broadcast from access points.
     Set to WDRV_ACTIVE_SCAN (default) or WDRV_PASSIVE_SCAN */
    uint8_t scanType;

    /* The number of times to scan a channel while attempting to find a particular
     access point. Default is 1 */
    uint8_t scanCount;

    /************************************************************************
      The minimum time (in milliseconds) the MRF24WN will wait for a probe
      response after sending a probe request. If no probe responses are
      received in minChannelTime, the MRF24WN will go on to the next channel,
      if any are left to scan, or quit. Default is 200ms.
      ************************************************************************/
    uint16_t minChannelTime;

    /************************************************************************
      If a probe response is received within minChannelTime, the MRF24WN will
      continue to collect any additional probe responses up to maxChannelTime
      before going to the next channel in the channelList. Units are in
      milliseconds. Default is 400ms.
      ************************************************************************/
    uint16_t maxChannelTime;

    /* The number of microseconds to delay before transmitting a probe request
     following the channel change during scanning.  Default is 20uS. */
    uint16_t probeDelay;

} WDRV_SCAN_CONTEXT;

/*******************************************************************************
  Summary:
    Contains data pertaining to Wi-Fi AdHoc context

  Description:
    Wi-Fi AdHoc Context

    This structure contains MRF24WN AdHoc context data.
*/
typedef struct
{
    /* Defines how to start the AdHoc network. See WDRV_ADHOC_MODE. */
    /* Default is WDRV_ADHOC_CONNECT_THEN_START. */
    uint8_t mode;

    /* When starting an AdHoc network, the SSID can be hidden in the beacons. */
    /* Set true to hide the SSID, else false.  Default is false. */
    bool hiddenSsid;

    /* Sets the beacon period, in ms.  Default is 100 ms. */
    uint16_t beaconPeriod;

} WDRV_ADHOC_NETWORK_CONTEXT;

/*******************************************************************************
  Summary:
    Contains data pertaining to Wi-Fi WEP context

  Description:
    Wi-Fi Wep Security Context

    This structure contains MRF24WN WEP context.
*/
typedef struct
{
    /* WDRV_SECURITY_WEP_40 or WDRV_SECURITY_WEP_104 */
    uint8_t wepSecurityType;

    /* Array containing four WEP binary keys. This will be four, 5-byte keys for
     WEP-40 or four, thirteen-byte keys for WEP-104. */
    uint8_t wepKey[WDRV_MAX_WEP_KEY_LENGTH];

    /* number of bytes pointed to by p_wepKey */
    uint8_t wepKeyLength;

} WDRV_WEP_CONTEXT;

/*******************************************************************************
  Summary:
    Contains data pertaining to Wi-Fi WPA Key

  Description:
    Wi-Fi WPA Key context

    This structure contains MRF24WN WPA key info. This structure is used in
    the WDRV_WPA_CONTEXT and WDRV_WPS_CONTEXT structures.
*/
typedef struct
{
    /* binary key or passphrase */
    uint8_t key[WDRV_MAX_WPA2_PASS_PHRASE_LENGTH];

    /* number of bytes in binary key (always 32) or passphrase */
    uint8_t keyLength;

} WDRV_WPA_KEY_INFO;

/*******************************************************************************
  Summary:
    Contains data pertaining to Wi-Fi WPA.

  Description:
    Wi-Fi WPA context

    This structure contains MRF24WN WPA context.

    <table>
    WDRV_SECURITY_WPA_WITH_PASS_PHRASE    WPA passphrase
    WDRV_SECURITY_WPA2_WITH_PASS_PHRASE    WPA2 passphrase
*/
typedef struct
{
    /* desired security type (see description) */
    uint8_t wpaSecurityType;

    /* see WDRV_WPA_KEY_INFO */
    WDRV_WPA_KEY_INFO keyInfo;

} WDRV_WPA_CONTEXT;

/*******************************************************************************
  Summary:
    Contains data pertaining to Wi-Fi WPS security.

  Description:
    Wi-Fi WPS security context

    This structure contains MRF24WN WPS security context.
*/
typedef struct
{
    /* WDRV_SECURITY_WPS_PUSH_BUTTON or WDRV_SECURITY_WPS_PIN */
    uint8_t wpsSecurityType;

    /* if using WDRV_SECURITY_WPS_PIN then pointer to 8-digit pin */
    uint8_t wpsPin[WDRV_WPS_PIN_LENGTH];

    /* should always be 8 */
    uint8_t wpsPinLength;

} WDRV_WPS_CONTEXT;

/*******************************************************************************
  Summary:
    Contains data pertaining to Wi-Fi security.

  Description:
    Wi-Fi security context

    This structure contains MRF24WN security context.
*/
typedef union
{
    /* set WEP security context */
    WDRV_WEP_CONTEXT wepContext;

    /* set WPA security context */
    WDRV_WPA_CONTEXT wpaContext;

    /* set WPS security context */
    WDRV_WPS_CONTEXT wpsContext;

} WDRV_SECURITY_CONTEXT;

#define WDRV_ENABLE_INTR()              SYS_INT_Enable()
#define WDRV_DISABLE_INTR()             SYS_INT_Disable()
#define WDRV_TIME_DELAY(msec)           WDRV_UsecDelay(msec * 1000)
#define WDRV_TASK_CREATE(func, alias, stackDepth, param, prio, handle) \
                                        WDRV_TaskCreate(func, alias, stackDepth, param, prio, handle, 0)
#define WDRV_TASK_DELETE(handle)        WDRV_TaskDestroy(handle)
#define WDRV_MALLOC(x)                  malloc((x))
#define WDRV_FREE(x)                    free((x))
#define WDRV_MUTEX_CREATE(handle)       WDRV_MutexInit(handle)
#define WDRV_MUTEX_LOCK(handle, waitMS) WDRV_MutexLock(handle, waitMS)
#define WDRV_MUTEX_UNLOCK(handle)       WDRV_MutexUnlock(handle)
#define WDRV_MUTEX_DELETE(handle)       WDRV_MutexDestroy(handle)

#if defined(__PIC32MZ__)
#define WDRV_DCACHE_CLEAN(addr, size) WDRV_DataCacheClean(addr, size)
#else /* !defined(__PIC32MZ__) */
#define WDRV_DCACHE_CLEAN(addr, size) do { } while (0)
#endif /* defined(__PIC32MZ__) */

#define WDRV_DBG_NONE    0
#define WDRV_DBG_ERROR   1
#define WDRV_DBG_INFORM  2
#define WDRV_DBG_TRACE   3
#define WDRV_DBG_VERBOSE 4
#define WDRV_DBG_LEVEL   WDRV_DBG_ERROR

#if WDRV_DBG_LEVEL != WDRV_DBG_NONE
#define DEBUG_LOCK WDRV_MUTEX_LOCK(g_wdrv_priv.debugConsoleLock, 0)
#define DEBUG_UNLOCK WDRV_MUTEX_UNLOCK(g_wdrv_priv.debugConsoleLock)
#define WDRV_DBG_VERBOSE_MESSAGE(x) do { if (WDRV_DBG_LEVEL >= WDRV_DBG_VERBOSE) { DEBUG_LOCK; SYS_MESSAGE x; DEBUG_UNLOCK; } } while (0)
#define WDRV_DBG_TRACE_MESSAGE(x) do { if (WDRV_DBG_LEVEL >= WDRV_DBG_TRACE) { DEBUG_LOCK; SYS_MESSAGE x; DEBUG_UNLOCK; } } while (0)
#define WDRV_DBG_INFORM_MESSAGE(x) do { if (WDRV_DBG_LEVEL >= WDRV_DBG_INFORM) {DEBUG_LOCK; SYS_MESSAGE x; DEBUG_UNLOCK; } } while (0)
#define WDRV_DBG_ERROR_MESSAGE(x) do { if (WDRV_DBG_LEVEL >= WDRV_DBG_ERROR) { DEBUG_LOCK; SYS_MESSAGE x; DEBUG_UNLOCK; } } while (0)
#define WDRV_DBG_VERBOSE_PRINT(x) do { if (WDRV_DBG_LEVEL >= WDRV_DBG_VERBOSE) { DEBUG_LOCK; SYS_PRINT x; DEBUG_UNLOCK; } } while (0)
#define WDRV_DBG_TRACE_PRINT(x) do { if (WDRV_DBG_LEVEL >= WDRV_DBG_TRACE) { DEBUG_LOCK; SYS_PRINT x; DEBUG_UNLOCK; } } while (0)
#define WDRV_DBG_INFORM_PRINT(x) do { if (WDRV_DBG_LEVEL >= WDRV_DBG_INFORM) { DEBUG_LOCK; SYS_PRINT x; DEBUG_UNLOCK; } } while (0)
#define WDRV_DBG_ERROR_PRINT(x) do { if (WDRV_DBG_LEVEL >= WDRV_DBG_ERROR) { DEBUG_LOCK; SYS_PRINT x; DEBUG_UNLOCK; } } while (0)
#define WDRV_DBG_TRACE_MESSAGE_IN_ISR(x)
#else
#define WDRV_DBG_VERBOSE_MESSAGE(x) do { } while(0)
#define WDRV_DBG_TRACE_MESSAGE(x) do { } while(0)
#define WDRV_DBG_INFORM_MESSAGE(x) do { } while(0)
#define WDRV_DBG_ERROR_MESSAGE(x) do { } while(0)
#define WDRV_DBG_VERBOSE_PRINT(x) do { } while (0)
#define WDRV_DBG_TRACE_PRINT(x) do { } while (0)
#define WDRV_DBG_INFORM_PRINT(x) do { } while (0)
#define WDRV_DBG_ERROR_PRINT(x) do { } while (0)
#define WDRV_DBG_TRACE_MESSAGE_IN_ISR(x) do { } while (0)
#endif

typedef struct {
   void (*CopyFrameToStackPacketBuffer_CB)(uint32_t len, uint8_t const *const frame);
   void (*ProceedConnectEvent_CB)(uint32_t connected, uint8_t devID, uint8_t *bssid, bool bssConn, uint8_t reason);
   void (*RFReady_CB)(uint8_t const *const addr);
   void (*ScanDone_CB)(uint32_t status);
   void (*InitDone_CB)(void);
   void (*DeinitDone_CB)(void);
   void (*WPSDone_CB)(void);
} WDRV_CALLBACKS;

typedef void (*GPIO_OUTLOW_T)(uint32_t channel, uint32_t bit_pos);
typedef void (*GPIO_OUTHIGH_T)(uint32_t channel, uint32_t bit_pos);

extern WDRV_CONFIG_DATA *p_wdrv_configData;
#define WDRV_CONFIG_PARAMS(params) p_wdrv_configData->params

#include "wdrv_mrf24wn_api.h"

void WDRV_Assert(int condition, const char *msg, const char *file, int line);

// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

#endif /* _WDRV_MRF24WN_COMMON_H */
