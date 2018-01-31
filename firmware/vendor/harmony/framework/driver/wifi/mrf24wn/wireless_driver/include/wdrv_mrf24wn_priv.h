/*******************************************************************************
  WiFi MAC interface functions

  File Name:
    wdrv_mrf24wn_priv.h

  Summary:
    MRF24WN Wi-Fi Driver Private Interface Function Prototypes

  Description:
    MRF24WN Wi-Fi Driver Private Interface Function Prototypes
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

#ifndef _WDRV_MRF24WN_PRIV_H
#define _WDRV_MRF24WN_PRIV_H

#include "tcpip/src/tcpip_private.h"

#define WDRV_MAX_CLIENT_TABLE_SLOTS 10

typedef struct
{
    uint8_t trafficEvents;
    uint16_t trafficEventInfo;
} WDRV_MRF24WN_USREV_DCPT;

// stack internal notification
typedef struct
{
    bool                     mrfEnabledEvents; // group enabled notification events
    volatile TCPIP_MAC_EVENT mrfPendingEvents; // group notification events that are set, waiting to be re-acknowledged
    TCPIP_MAC_EventF         mrfNotifyFnc; // group notification handler
    const void              *mrfNotifyParam; // notification parameter
} WDRV_MRF24WN_EVGROUP_DCPT; // event descriptor

typedef struct {
    uint8_t addr[6];
    uint32_t timeStamp;
} MAC_ADDR;

typedef struct 
{
    MAC_ADDR mac[WDRV_MAX_CLIENT_TABLE_SLOTS];
    uint16_t bitMap;
    uint32_t seqNum;
    uint16_t updated;
} WDRV_CLIENT_CACHE;

typedef struct {
    uint8_t macAddr[6];
    bool initDriverDone;
    bool deinitDriverDone;
    bool updateMacAddressRequired;
    bool isScanDone;
    bool initConn;
    bool isConnReestablished;
    bool isDisconnectRequested;
    OSAL_SEM_HANDLE_TYPE disconnectDoneSync;
    OSAL_SEM_HANDLE_TYPE dmaTxSync;
    OSAL_SEM_HANDLE_TYPE dmaRxSync;
    OSAL_MUTEX_HANDLE_TYPE *debugConsoleLock;
    OSAL_MUTEX_HANDLE_TYPE *multicastFilterLock;
    WDRV_CLIENT_CACHE clientCache;
} WDRV_MRF24WN_PRIV;

typedef enum 
{  
    WDRV_ERROR                = -1,
    WDRV_SUCCESS              = 0,
    WDRV_INVALID_TASK_ID      = 1,
    WDRV_INVALID_PARAMETER    = 2,
    WDRV_INVALID_POINTER      = 3,
    WDRV_ALREADY_EXISTS       = 4,
    WDRV_INVALID_EVENT        = 5,
    WDRV_EVENT_TIMEOUT        = 6,
    WDRV_INVALID_MUTEX        = 7,
    WDRV_TASK_ALREADY_LOCKED  = 8,
    WDRV_MUTEX_ALREADY_LOCKED = 9,  
    WDRV_OUT_OF_MEMORY        = 10,
} WDRV_OSAL_STATUS;

bool WDRV_SemInit(OSAL_SEM_HANDLE_TYPE *SemID);
void WDRV_SemTake(OSAL_SEM_HANDLE_TYPE *SemID, uint16_t timeout);
void WDRV_SemGive(OSAL_SEM_HANDLE_TYPE *SemID);
void WDRV_SemGiveFromISR(OSAL_SEM_HANDLE_TYPE *SemID);
void WDRV_SemDeInit(OSAL_SEM_HANDLE_TYPE *SemID);
uint32_t WDRV_MutexInit(OSAL_MUTEX_HANDLE_TYPE **mutex_ptr);
uint32_t WDRV_MutexDestroy(OSAL_MUTEX_HANDLE_TYPE **mutex_ptr);
uint32_t WDRV_MutexLock(OSAL_MUTEX_HANDLE_TYPE *mutex_ptr, uint32_t tick_count);
uint32_t WDRV_MutexUnlock(OSAL_MUTEX_HANDLE_TYPE *mutex_ptr);
WDRV_OSAL_STATUS WDRV_TaskCreate(void Task(void *), const char *task_name, int stack_size, void *param, 
    unsigned long task_priority, TaskHandle_t *task_handle, bool auto_start);
WDRV_OSAL_STATUS WDRV_TaskDestroy(TaskHandle_t task_handle);
void WDRV_UsecDelay(uint32_t uSec);

void WDRV_PendingEventProcess(void);
bool isEventPending(void);

void WDRV_GPIO_OutLow_PIC32MZ_ESK(uint32_t channel, uint32_t bit_pos);
void WDRV_GPIO_OutLow_PIC32MX_ESK(uint32_t channel, uint32_t bit_pos);
void WDRV_GPIO_OutLow_PIC32MX_EXP16(uint32_t channel, uint32_t bit_pos);
void WDRV_GPIO_OutLow_PIC32MZ_MEB2(uint32_t channel, uint32_t bit_pos);
void WDRV_GPIO_OutLow_Custom_Board(uint32_t channel, uint32_t bit_pos);
void WDRV_GPIO_OutHigh_PIC32MZ_ESK(uint32_t channel, uint32_t bit_pos);
void WDRV_GPIO_OutHigh_PIC32MX_ESK(uint32_t channel, uint32_t bit_pos);
void WDRV_GPIO_OutHigh_PIC32MX_EXP16(uint32_t channel, uint32_t bit_pos);
void WDRV_GPIO_OutHigh_PIC32MZ_MEB2(uint32_t channel, uint32_t bit_pos);
void WDRV_GPIO_OutHigh_Custom_Board(uint32_t channel, uint32_t bit_pos);

TCPIP_MAC_RES WDRV_MRF24WN_MulticastFilterSet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_ADDR *DestMACAddr);
bool isMacInitialized(void);

void WDRV_DataCacheClean(unsigned char *address, uint32_t size);
bool ClientCacheUpdated(bool *connected, uint8_t *mac);

extern WDRV_MRF24WN_PRIV g_wdrv_priv;

#endif /*_WDRV_MRF24WN_PRIV_H */

// DOM-IGNORE-END
