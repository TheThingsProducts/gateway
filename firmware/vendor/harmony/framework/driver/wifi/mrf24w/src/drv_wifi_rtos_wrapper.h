/*******************************************************************************
  MRF24WG OSAL Functions

  File Name:
    drv_wifi_osal.h

  Summary:
    MRF24WG OSAL Functions

  Description:
    OSAL functions for MRF24WG Wi-Fi module.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc. All rights reserved.

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

#ifndef _DRV_WIFI_RTOS_WRAPPER_H
#define _DRV_WIFI_RTOS_WRAPPER_H

# if defined(DRV_WIFI_USE_FREERTOS)

bool DRV_WIFI_SemInit(OSAL_SEM_HANDLE_TYPE *sem);
void DRV_WIFI_SemDeInit(OSAL_SEM_HANDLE_TYPE *sem);
void DRV_WIFI_SemTake(OSAL_SEM_HANDLE_TYPE *sem, uint16_t waitMS);
void DRV_WIFI_SemGive(OSAL_SEM_HANDLE_TYPE *sem);
void DRV_WIFI_ISR_SemGive(OSAL_SEM_HANDLE_TYPE *sem);
bool DRV_WIFI_MutexInit(OSAL_MUTEX_HANDLE_TYPE *mutex);
void DRV_WIFI_MutexDeInit(OSAL_MUTEX_HANDLE_TYPE *mutex);
void DRV_WIFI_MutexLock(OSAL_MUTEX_HANDLE_TYPE *mutex, uint16_t waitMS);
void DRV_WIFI_MutexUnlock(OSAL_MUTEX_HANDLE_TYPE *mutex);
bool DRV_WIFI_TaskSyncInit(void);

/* 
 * OSAL_USE_RTOS == 1 means FreeRTOS_V8.x.x is used. Please refer to
 * osal_definitions.h. FreeRTOS_V8.x.x is also the only RTOS supported so far
 * in MRF24WG driver. Following functions are implemented specifically in this
 * RTOS.
 */
#  if(OSAL_USE_RTOS == 1)
bool DRV_WIFI_TaskCreate(void (*taskFunction)(void *),
        const char *const taskName,
        const uint16_t taskSize,
        void *const taskParameters,
        uint32_t taskPriority,
        void **const p_taskHandleCreated);
void DRV_WIFI_TaskDestroy(void **p_taskHandle);
bool DRV_WIFI_AllTasksCreate(void *initTaskParameters,
        void *deferredISRParameters,
        void *macTaskParameters);
void DRV_WIFI_AllTaskDestroy(void);
void DRV_WIFI_TaskDelay(const uint32_t ticksToDelay);
#  endif /* OSAL_USE_RTOS == 1 */

# endif /* DRV_WIFI_USE_FREERTOS */

#endif /* _DRV_WIFI_RTOS_WRAPPER_H */
