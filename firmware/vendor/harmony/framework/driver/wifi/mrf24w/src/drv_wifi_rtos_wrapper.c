/*******************************************************************************
  MRF24WG OSAL Functions

  File Name:
    drv_wifi_osal.c

  Summary:
    MRF24WG OSAL Functions

  Description:
    OSAL functions for MRF24WG Wi-Fi module.
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

#if defined(DRV_WIFI_USE_FREERTOS)

bool DRV_WIFI_MutexInit(OSAL_MUTEX_HANDLE_TYPE *mutex)
{
    if (OSAL_MUTEX_Create(mutex) == OSAL_RESULT_TRUE)
        return true;
    else
        return false;
}

void DRV_WIFI_MutexDeInit(OSAL_MUTEX_HANDLE_TYPE *mutex)
{
    OSAL_MUTEX_Delete(mutex);
}

void DRV_WIFI_MutexLock(OSAL_MUTEX_HANDLE_TYPE *mutex, uint16_t waitMS)
{
    OSAL_MUTEX_Lock(mutex, waitMS);
}

void DRV_WIFI_MutexUnlock(OSAL_MUTEX_HANDLE_TYPE *mutex)
{
    OSAL_MUTEX_Unlock(mutex);
}

bool DRV_WIFI_SemInit(OSAL_SEM_HANDLE_TYPE *sem)
{
    if (OSAL_SEM_Create(sem, OSAL_SEM_TYPE_COUNTING, 1, 0) == OSAL_RESULT_TRUE)
        return true;
    else
        return false;
}

void DRV_WIFI_SemDeInit(OSAL_SEM_HANDLE_TYPE *sem)
{
    OSAL_SEM_Delete(sem);
}

void DRV_WIFI_SemTake(OSAL_SEM_HANDLE_TYPE *sem, uint16_t waitMS)
{
    OSAL_SEM_Pend(sem, waitMS);
}

void DRV_WIFI_SemGive(OSAL_SEM_HANDLE_TYPE *sem)
{
    OSAL_SEM_Post(sem);
}

void DRV_WIFI_ISR_SemGive(OSAL_SEM_HANDLE_TYPE *sem)
{
    OSAL_SEM_PostISR(sem);
}

/*
 * This is a public API which only allows the user to give semaphore to Wi-Fi
 * Deferred ISR. So it's declared in drv_wifi.h instead of drv_wifi_osal.h.
 */
void DRV_WIFI_DeferredISR_SemGive(void)
{
    DRV_WIFI_ISR_SemGive(&g_drv_wifi_priv.deferredISRSync);
}

bool DRV_WIFI_TaskSyncInit(void)
{
    static bool once = false;
    if (!once) {
        if (!DRV_WIFI_SemInit(&g_drv_wifi_priv.deferredISRSync))
            return false;
        if (!DRV_WIFI_SemInit(&g_drv_wifi_priv.mgmtRespSync))
            return false;
        if (!DRV_WIFI_SemInit(&g_drv_wifi_priv.macTaskSync))
            return false;
        if (!DRV_WIFI_MutexInit(&g_drv_wifi_priv.initLock))
            return false;
        if (!DRV_WIFI_MutexInit(&g_drv_wifi_priv.spiLock))
            return false;
        if (!DRV_WIFI_MutexInit(&g_drv_wifi_priv.multicastFilterLock))
            return false;
        if (!DRV_WIFI_MutexInit(&g_drv_wifi_priv.mgmtMsgLock))
            return false;
        once = true;
    }
    return true;
}

/*
 * OSAL_USE_RTOS == 1 means FreeRTOS_V8.x.x is used. Please refer to
 * osal_definitions.h. FreeRTOS_V8.x.x is also the only RTOS supported so far
 * in MRF24WG driver. Following functions are implemented specifically in this
 * RTOS.
 */
# if(OSAL_USE_RTOS == 1)

bool DRV_WIFI_TaskCreate(void (*taskFunction)(void *),
        const char *const taskName,
        const uint16_t taskSize,
        void *const taskParameters,
        uint32_t taskPriority,
        void **const p_taskHandleCreated)
{
    return xTaskCreate((TaskFunction_t)taskFunction,
                   taskName,
                   taskSize,
                   taskParameters,
                   (UBaseType_t)taskPriority,
                   (TaskHandle_t *)p_taskHandleCreated);
}

void DRV_WIFI_TaskDestroy(void **p_taskHandle)
{
    vTaskDelete((TaskHandle_t)(*p_taskHandle));
    *p_taskHandle = NULL;
}

bool DRV_WIFI_AllTasksCreate(void *initTaskParameters,
        void *deferredISRParameters,
        void *macTaskParameters)
{
    /* Create OS Thread for Wi-Fi RTOS Initialization Task. */
    if (!g_drv_wifi_priv.initTaskHandle) {
        if (!DRV_WIFI_TaskCreate(DRV_WIFI_InitTask,
                     "Wi-Fi RTOS Initialization Task",
                     DRV_WIFI_RTOS_INIT_TASK_SIZE,
                     initTaskParameters,
                     DRV_WIFI_RTOS_INIT_TASK_PRIORITY,
                     &g_drv_wifi_priv.initTaskHandle))
            return false;
    }
    /* Create OS Thread for Wi-Fi RTOS Deferred ISR. */
    if (!g_drv_wifi_priv.deferredISRHandle) {
        if (!DRV_WIFI_TaskCreate(DRV_WIFI_Deferred_ISR,
                     "Wi-Fi RTOS Deferred ISR",
                     DRV_WIFI_RTOS_DEFERRED_ISR_SIZE,
                     deferredISRParameters,
                     DRV_WIFI_RTOS_DEFERRED_ISR_PRIORITY,
                     &g_drv_wifi_priv.deferredISRHandle))
            return false;
    }
    /* Create OS Thread for Wi-Fi RTOS MAC Task. */
    if (!g_drv_wifi_priv.macTaskHandle) {
        if (!DRV_WIFI_TaskCreate(DRV_WIFI_MACTask,
                     "Wi-Fi RTOS MAC Task",
                     DRV_WIFI_RTOS_MAC_TASK_SIZE,
                     macTaskParameters,
                     DRV_WIFI_RTOS_MAC_TASK_PRIORITY,
                     &g_drv_wifi_priv.macTaskHandle))
            return false;
    }
    return true;
}

void DRV_WIFI_AllTaskDestroy(void)
{
    if (g_drv_wifi_priv.initTaskHandle)
        DRV_WIFI_TaskDestroy(&g_drv_wifi_priv.initTaskHandle);
    if (g_drv_wifi_priv.deferredISRHandle)
        DRV_WIFI_TaskDestroy(&g_drv_wifi_priv.deferredISRHandle);
    if (g_drv_wifi_priv.macTaskHandle)
        DRV_WIFI_TaskDestroy(&g_drv_wifi_priv.macTaskHandle);
}

void DRV_WIFI_TaskDelay(const uint32_t ticksToDelay)
{
    vTaskDelay(ticksToDelay);
}

# endif

#endif /* defined(DRV_WIFI_USE_FREERTOS) */

//DOM-IGNORE-END
