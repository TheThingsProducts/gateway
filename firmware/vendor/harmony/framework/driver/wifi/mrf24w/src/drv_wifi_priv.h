/*******************************************************************************
  MRF24WG Wi-Fi Driver Private Interface Function Prototypes

  File Name:
    drv_wifi_priv.h

  Summary:
    MRF24WG Wi-Fi Driver Private Interface Function Prototypes

  Description:
    MRF24WG Wi-Fi Driver Private Interface Function Prototypes
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

#ifndef _DRV_WIFI_PRIV_H
#define _DRV_WIFI_PRIV_H

#include "tcpip/src/tcpip_private.h"

/****************************/
/*    DEFINES AND MACROS    */
/****************************/
/*----------------------------------------------------------------------*
 * Default interrupt priority to use for the TCPIP interrupts           *
 *----------------------------------------------------------------------*/
#define WF_EVENT_IPL 5
#define WF_EVENT_SIPL 1

#define WF_SetCE_N(level) \
    /* set pin to desired level */ \
    SYS_PORTS_PinWrite(PORTS_ID_0, WF_HIBERNATE_PORT_CHANNEL, WF_HIBERNATE_BIT_POS, level); \
    /* configure I/O as ouput */ \
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, WF_HIBERNATE_PORT_CHANNEL, WF_HIBERNATE_BIT_POS);
#define WF_SetRST_N(level) \
    /* configure the I/O as an output */ \
    SYS_PORTS_PinWrite(PORTS_ID_0, WF_RESET_PORT_CHANNEL, WF_RESET_BIT_POS, level); \
    /* set pin to desired level */ \
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, WF_RESET_PORT_CHANNEL, WF_RESET_BIT_POS);

/* SPI Tx Message Types */
#define WF_DATA_REQUEST_TYPE            ((uint8_t)1)
#define WF_MGMT_REQUEST_TYPE            ((uint8_t)2)

/* SPI Rx Message Types */
#define WF_DATA_TX_CONFIRM_TYPE         ((uint8_t)1)
#define WF_MGMT_CONFIRM_TYPE            ((uint8_t)2)
#define WF_DATA_RX_INDICATE_TYPE        ((uint8_t)3)
#define WF_MGMT_INDICATE_TYPE           ((uint8_t)4)

/* SPI Tx/Rx Data Message Subtypes */
#define WF_STD_DATA_MSG_SUBTYPE         ((uint8_t)1)
#define WF_NULL_DATA_MSG_SUBTYPE        ((uint8_t)2)
/* reserved value                       ((uint8_t)3) */
#define WF_UNTAMPERED_DATA_MSG_SUBTYPE  ((uint8_t)4)

#define WF_TX_DATA_MSG_PREAMBLE_LENGTH  ((uint8_t)4)

#define WF_READ_REGISTER_MASK           ((uint8_t)(0x40))
#define WF_WRITE_REGISTER_MASK          ((uint8_t)(0x00))

/*-----------------------------*/
/* MRF24W 8-bit Host Registers */
/*-----------------------------*/
#define WF_HOST_INTR_REG            ((uint8_t)(0x01)) /* 8-bit register containing 1st level interrupt bits. */
#define WF_HOST_MASK_REG            ((uint8_t)(0x02)) /* 8-bit register containing 1st level interrupt mask. */

/*------------------------------*/
/* MRF24W 16-bit Host Registers */
/*------------------------------*/
#define WF_HOST_MAIL_BOX_0_MSW_REG  ((uint16_t)(0x10))
#define WF_HOST_MAIL_BOX_0_LSW_REG  ((uint16_t)(0x12))
#define WF_HOST_MAIL_BOX_1_MSW_REG  ((uint16_t)(0x14))
#define WF_HOST_MAIL_BOX_1_LSW_REG  ((uint16_t)(0x16))
#define WF_HOST_RAW0_CTRL1_REG      ((uint16_t)(0x26))
#define WF_HOST_RAW1_CTRL1_REG      ((uint16_t)(0x2a))
#define WF_HOST_RAW2_CTRL1_REG      ((uint16_t)(0x19))
#define WF_HOST_RAW3_CTRL1_REG      ((uint16_t)(0x1d))
#define WF_HOST_RAW4_CTRL1_REG      ((uint16_t)(0x0b))
#define WF_HOST_RAW5_CTRL1_REG      ((uint16_t)(0x0f))

#define WF_HOST_RESET_REG           ((uint16_t)(0x3c))
#define WF_HOST_RESET_MASK          ((uint16_t)(0x0001))

/* Scratchpad registers */
#define WF_SCRATCHPAD_0_REG         ((uint16_t)(0x3d))
#define WF_SCRATCHPAD_1_REG         ((uint16_t)(0x3e))

#define WF_HOST_RAW0_STATUS_REG     ((uint16_t)(0x28))

#define WF_HOST_INTR2_REG           ((uint16_t)(0x2d)) /* 16-bit register containing 2nd level interrupt bits */
#define WF_HOST_INTR2_MASK_REG      ((uint16_t)(0x2e))

#define WF_HOST_WFIFO_BCNT0_REG     ((uint16_t)(0x2f)) /* 16-bit register containing available write size for fifo 0 (data tx)*/
                                                       /* (LS 12 bits contain the length)                                     */

#define WF_HOST_WFIFO_BCNT1_REG     ((uint16_t)(0x31)) /* 16-bit register containing available write size for fifo 1 (mgmt tx)*/
                                                       /* (LS 12 bits contain the length)                                     */

#define WF_HOST_RFIFO_BCNT0_REG     ((uint16_t)(0x33)) /* 16-bit register containing number of bytes in read fifo 0 (data rx) */
                                                       /* (LS 12 bits contain the length)                                     */

#define WF_HOST_RFIFO_BCNT1_REG     ((uint16_t)(0x35)) /* 16-bit register containing number of bytes in read fifo 1 (mgmt rx) */
                                                       /* (LS 12 bits contain the length)                                     */

#define WF_PSPOLL_H_REG             ((uint16_t)(0x3d)) /* 16-bit register used to control low power mode                      */
#define WF_INDEX_ADDR_REG           ((uint16_t)(0x3e)) /* 16-bit register to move the data window                             */
#define WF_INDEX_DATA_REG           ((uint16_t)(0x3f)) /* 16-bit register to read or write address-indexed register           */

/*----------------------------------------------------------------------------------------*/
/* WiFi registers accessed via the WF_INDEX_ADDR_REG and WF_INDEX_DATA_REG registers */
/*----------------------------------------------------------------------------------------*/
#define WF_HW_STATUS_REG            ((uint16_t)(0x2a)) /* 16-bit read only register providing hardware status bits */
#define WF_CONFIG_CTRL0_REG         ((uint16_t)(0x2e)) /* 16-bit register used to initiate Hard reset              */
#define WF_WAKE_CONTROL_REG         ((uint16_t)(0x2f)) /* NEW */
#define WF_SCRATCHPAD_0_REG         ((uint16_t)(0x3d)) /* NEW */
#define WF_SCRATCHPAD_1_REG         ((uint16_t)(0x3e)) /* 16-bit register read to determine when low power is done */
#define WF_PSPOLL_CONFIG_REG        ((uint16_t)(0x40)) /* NEW */
#define WF_XTAL_SETTLE_TIME_REG     ((uint16_t)(0x41)) /* NEW */

/* This bit mask is used in the HW_STATUS_REG to determine */
/* when the MRF24W has completed its hardware reset.       */
/*  0 : MRF24W is in reset                                 */
/*  1 : MRF24W is not in reset                             */
#define WF_HW_STATUS_NOT_IN_RESET_MASK ((uint16_t)(0x1000))

/* Definitions represent individual interrupt bits for the 8-bit host interrupt registers */
/*  WF_HOST_INTR_REG and WF_HOST_MASK_REG                                                 */
//#define WF_HOST_INT_MASK_INT2 ((uint8_t)(0x01 | 0x02))
#define WF_HOST_INT_MASK_INT2 ((uint8_t)(0x01))
#define WF_HOST_INT_MASK_RAW_0_INT_0 ((uint8_t)(0x02)) /* Data Rx */
#define WF_HOST_INT_MASK_RAW_1_INT_0 ((uint8_t)(0x04)) /* Data Tx */
#define WF_HOST_INT_MASK_FIFO_0_THRESHOLD ((uint8_t)(0x40))
#define WF_HOST_INT_MASK_FIFO_1_THRESHOLD ((uint8_t)(0x80))
#define WF_HOST_INT_MASK_ALL_INT ((uint8_t)(0xff))

/* Definitions represent individual interrupt bits for */
/*  WF_HOST_INTR2_REG and WF_HOST_MASK2_REG            */
#define WF_HOST_INT_MASK_RAW_2_INT_0 ((uint16_t)(0x0010)) /* Mgmt Rx  */
#define WF_HOST_INT_MASK_RAW_3_INT_0 ((uint16_t)(0x0020)) /* Mgmt Tx  */
#define WF_HOST_INT_MASK_RAW_4_INT_0 ((uint16_t)(0x0004)) /* Scratch  */
#define WF_HOST_INT_MASK_RAW_5_INT_0 ((uint16_t)(0x0008)) /* Not used */
#define WF_HOST_INT_MASK_MAIL_BOX_0_WRT ((uint16_t)(0x0001))
#define WF_HOST_INT_MASK_MAIL_BOX_1_WRT ((uint16_t)(0x0002))

/* Bit mask for all interrupts in the level 2 16-bit interrupt register */
#define WF_HOST_2_INT_MASK_ALL_INT ((uint16_t)(0xffff))

/* these definitions are used in calls to enable and
 * disable interrupt bits. */
#define WF_INT_DISABLE ((uint8_t)0)
#define WF_INT_ENABLE  ((uint8_t)1)

#define WF_LOW_POWER_MODE_OFF (0)
#define WF_LOW_POWER_MODE_ON (1)

#define POWER_SAVE_PENDING (0x10)
#define WPS_CONNECT_PENDING (0x20)

#define WF_MAC_ADDRESS_LENGTH (6)

// maximum number of tx packets that can be queued by stack before error occurs
#define MAX_TX_QUEUE_COUNT (25)

#define MAX_CLIENT_TABLE_SLOTS 6

// Pin Level
typedef enum
{
    WIFI_PIN_LOW = 0,
    WIFI_PIN_HIGH = 1
} WIFI_PIN_LEVEL;

/***************/
/*    TYPES    */
/***************/
typedef struct
{
    volatile uint8_t rawInterrupt;
    bool waitingForRawMoveCompleteInterrupt;
} RAW_MOVE_STATE;

// Stack Client Notification
//
// Storage for MRF24 events.
// Stored until the stack user asks for them.
// Keep different copies for RX and MGMT events
//   since it seems that the Wi-Fi code treats them separately.
typedef struct
{
    uint8_t trafficEvents;
    uint16_t trafficEventInfo;
    uint8_t mgmtEvents;
    uint16_t mgmtEventInfo;
} MRF24W_USR_EV_DCPT;

typedef struct {
    uint8_t addr[6];
    uint32_t timeStamp;
} MAC_ADDR;

typedef struct 
{
    MAC_ADDR mac[MAX_CLIENT_TABLE_SLOTS];
    uint16_t bitMap;
    uint32_t seqNum;
    uint16_t updated;
} DRV_WIFI_CLIENT_CACHE;

// *****************************************************************************
/*
  Summary:
    Contains data pertaining to driver internal control.

  Description:
    Driver control flags.

    This structure contains data pertaining to driver internal control.
 */
typedef struct
{
    bool isDriverOpen;
    bool isInDriverClose;
    bool isScanDone;
    bool initConn;
    bool isConnReestablished;
    bool isConnPermanentlyLost;
    void *initTaskHandle;
    void *deferredISRHandle;
    void *macTaskHandle;
    OSAL_SEM_HANDLE_TYPE deferredISRSync;
    OSAL_SEM_HANDLE_TYPE mgmtRespSync;
    OSAL_SEM_HANDLE_TYPE macTaskSync;
    OSAL_SEM_HANDLE_TYPE dmaTxSync;
    OSAL_SEM_HANDLE_TYPE dmaRxSync;
    OSAL_MUTEX_HANDLE_TYPE initLock;
    OSAL_MUTEX_HANDLE_TYPE spiLock;
    OSAL_MUTEX_HANDLE_TYPE multicastFilterLock;
    OSAL_MUTEX_HANDLE_TYPE mgmtMsgLock;
    DRV_WIFI_CLIENT_CACHE clientCache;
} DRV_WIFI_PRIV;

/*****************/
/*    GLOBALS    */
/*****************/
extern DRV_WIFI_PRIV g_drv_wifi_priv;
extern volatile RAW_MOVE_STATE RawMoveState[];

/*****************************/
/*    FUNCTION PROTOTYPES    */
/*****************************/
// drv_wifi_com.c
void DRV_WIFI_TimerTaskRun(void);
void PowerSaveTask(void);
void MgmtRxProcess(void);
int16_t GetRxPendingCount(void);
void IncrementRxPendingCount(void);
void DecrementRxPendingCount(void);
void SignalWiFiConnectionChanged(bool state);
void ResetPll(void);
void ResetModule(void);
void Write16BitWFRegister(uint8_t regId, uint16_t value);
uint16_t Read16BitWFRegister(uint8_t regId);
void Write8BitWFRegister(uint8_t regId, uint8_t value);
uint8_t Read8BitWFRegister(uint8_t regId);
void WriteWFArray(uint8_t regId, uint8_t *pBuf, uint16_t length);
void ReadWFArray(uint8_t regId, uint8_t *pBuf, uint16_t length);
void DRV_WIFI_DataCacheClean(unsigned char *address, uint32_t size);

// drv_wifi_connection_algorithm.c
void SetListenInterval(uint16_t listenInterval);
void SetDtimInterval(uint16_t dtimInterval);

// drv_wifi_connection_manager.c
bool isLinkUp(void);
void SetLogicalConnectionState(bool state);
void WpaKeySet(void);
TCPIP_MAC_RES DRV_WIFI_KeyDerive(uint8_t key_len, uint8_t *key, uint8_t ssid_len, uint8_t *ssid);

// drv_wifi_connection_profile.c
void ConnectionProfileCreate(void);
void DRV_WIFI_HiddenSsidSet(bool hiddenSsid);
uint8_t DRV_WIFI_HiddenSsidGet(void);
void DRV_WIFI_AdhocModeSet(uint8_t mode);
uint8_t DRV_WIFI_AdhocModeGet(void);

// drv_wifi_events.c
void DRV_WIFI_UserEventSet(uint16_t event, uint16_t eventInfo, bool isMgmt);
uint16_t TrafficEventsGet(uint16_t *pEventInfo);
uint16_t MgmtEventsGet(MRF24W_USR_EV_DCPT *pEventInfo);

// drv_wifi_event_handler.c
void DRV_WIFI_PendingEventHandle(void);
bool DRV_WIFI_PendingEventGet(void);
void DRV_WIFI_PendingEventSet(uint8_t event);
void DRV_WIFI_AllEventClear(void);
bool ClientCacheUpdated(bool *connected, uint8_t *mac);

// drv_wifi_init.c
bool isInitComplete(void);
TCPIP_MAC_RES DRV_WIFI_SystemInit(void);
#if defined(DRV_WIFI_USE_FREERTOS)
void DRV_WIFI_InitTask(void *p_arg);
#endif /* defined(DRV_WIFI_USE_FREERTOS) */

// drv_wifi_mac.c
void DRV_WIFI_DataRxTask(void);
void DRV_WIFI_DataTxTask(void);
TCPIP_MAC_RES DRV_WIFI_RxTaskInit(void);
void DRV_WIFI_TxTaskInit(void);
void RxQueueDeinit(void);
void TxQueueDeinit(void);
bool isRxStateMachineIdle(void);
void SetRxPacketPending(void);
void StartRawMoveTimer(uint16_t rawId);
void StopRawMoveTimer(uint16_t rawId);
bool isDataTxTaskInactive(void);
TCPIP_MAC_RES MRF24W_TxPacketSend(TCPIP_MAC_PACKET *p_packet);
TCPIP_MAC_PACKET *MRF24W_RxPacketGet(void);
void RxTxUnLockTimerStart(void);
#if (DRV_WIFI_SAVE_WPS_CREDENTIALS == DRV_WIFI_ENABLED)
void WPSCredentialsSave(void);
#endif

// drv_wifi_main.c
bool isMacInitialized(void);
void SignalTxStateMachine(void);
void SignalRxStateMachine(void);
const TCPIP_MAC_MODULE_CTRL *GetStackData(void);
TCPIP_MAC_RES MRF24W_MACMulticastFilterSet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_ADDR *DestMACAddr);

// drv_wifi_param_msg.c
uint16_t DRV_WIFI_MgmtBaseGet(void);
void DRV_WIFI_HostKeyDeriveModeSet(void);
void DRV_WIFI_MultiCastFilterEnable(void);

// drv_wifi_pbkdf2.c
TCPIP_MAC_RES pbkdf2_sha1(const char *passphrase, const char *ssid, uint16_t ssid_len, uint16_t iterations, uint8_t *buf, uint16_t buflen);

// drv_wifi_power_save.c
void EnsureWFisAwake(void);
void ConfigureLowPowerMode(uint8_t action);
bool isPsPollEnabled(void);
bool isPsPollActive(void);
bool isSleepNeeded(void);
void SetSleepNeeded(void);
void ClearSleepNeeded(void);
void SetPowerSaveMode(bool state);
bool GetPowerSaveMode(void);

// drv_wifi_scan.c
void DRV_WIFI_PreScanStart(void);

// drv_wifi_spi.c
bool DRV_WIFI_SpiWrite(unsigned char *buf, uint32_t size);
bool DRV_WIFI_SpiRead(unsigned char *p_cmd, uint32_t cmdLen, unsigned char *p_rxBuf, uint32_t rxLen);
bool DRV_WIFI_SpiWrite2(unsigned char *p_cmd, uint32_t cmdLen, unsigned char *p_txBuf, uint32_t txLen);

#if defined(DRV_WIFI_USE_FREERTOS)
#include "drv_wifi_rtos_wrapper.h"
#define WF_INIT_MUTEX_LOCK() DRV_WIFI_MutexLock(&g_drv_wifi_priv.initLock, OSAL_WAIT_FOREVER)
#define WF_INIT_MUTEX_UNLOCK() DRV_WIFI_MutexUnlock(&g_drv_wifi_priv.initLock)
#define WF_SPI_MUTEX_LOCK() DRV_WIFI_MutexLock(&g_drv_wifi_priv.spiLock, OSAL_WAIT_FOREVER)
#define WF_SPI_MUTEX_UNLOCK() DRV_WIFI_MutexUnlock(&g_drv_wifi_priv.spiLock)
#define WF_FILTERSET_MUTEX_LOCK() DRV_WIFI_MutexLock(&g_drv_wifi_priv.multicastFilterLock, OSAL_WAIT_FOREVER)
#define WF_FILTERSET_MUTEX_UNLOCK() DRV_WIFI_MutexUnlock(&g_drv_wifi_priv.multicastFilterLock)
#define WF_MGMTMSG_MUTEX_LOCK() DRV_WIFI_MutexLock(&g_drv_wifi_priv.mgmtMsgLock, OSAL_WAIT_FOREVER)
#define WF_MGMTMSG_MUTEX_UNLOCK() DRV_WIFI_MutexUnlock(&g_drv_wifi_priv.mgmtMsgLock)
#else
#define WF_INIT_MUTEX_LOCK() do {} while (0)
#define WF_INIT_MUTEX_UNLOCK() do {} while (0)
#define WF_SPI_MUTEX_LOCK() do {} while (0)
#define WF_SPI_MUTEX_UNLOCK() do {} while (0)
#define WF_FILTERSET_MUTEX_LOCK() do {} while (0)
#define WF_FILTERSET_MUTEX_UNLOCK() do {} while (0)
#define WF_MGMTMSG_MUTEX_LOCK() do {} while (0)
#define WF_MGMTMSG_MUTEX_UNLOCK() do {} while (0)
#endif /* defined(DRV_WIFI_USE_FREERTOS) */

#endif /*_DRV_WIFI_PRIV_H */

// DOM-IGNORE-END
