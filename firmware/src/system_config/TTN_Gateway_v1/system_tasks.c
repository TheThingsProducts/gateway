/*******************************************************************************
 System Tasks File

  File Name:
    system_tasks.c

  Summary:
    This file contains source code necessary to maintain system's polled state
    machines.

  Description:
    This file contains source code necessary to maintain system's polled state
    machines.  It implements the "SYS_Tasks" function that calls the individual
    "Tasks" functions for all the MPLAB Harmony modules in the system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    polled in the system.  These handles are passed into the individual module
    "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"
#include "app.h"
#include "lora.h"
#include "helper_wdt.h"
#include "track_heap.h"


// *****************************************************************************
// *****************************************************************************
// Section: Local Prototypes
// *****************************************************************************
// *****************************************************************************


 
static void _SYS_Tasks ( void );
void _TCPIP_Tasks(void);
static void _APP_Tasks(void);
static void _LORA_Tasks(void);


// *****************************************************************************
// *****************************************************************************
// Section: System "Tasks" Routine
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Tasks ( void )

  Remarks:
    See prototype in system/common/sys_module.h.
*/

uint32_t thread_0_timeout_timer=0;

#define TIMEOUT_THREAD_0_START (thread_0_timeout_timer = SYS_TMR_TickCountGet())
#define TIMEOUT_THREAD_0(X) (thread_0_timeout_timer > SYS_TMR_TickCountGet() || ((SYS_TMR_TickCountGet() -  thread_0_timeout_timer) >= (SYS_TMR_TickCounterFrequencyGet()*X)))

static TaskHandle_t sysTaskHandle;
static TaskHandle_t tcpipTaskHandle;
static TaskHandle_t appTaskHandle;
static TaskHandle_t loraTaskHandle;

void SYS_Tasks ( void )
{
    /* Create OS Thread for Sys Tasks. */
    xTaskCreate((TaskFunction_t) _SYS_Tasks,
                "Sys Tasks",
                TASK_SIZE_SYS_TASKS, NULL, TASK_PRIORITY_SYS_TASKS, &sysTaskHandle);


    /* Create task for TCPIP state machine*/
    /* Create OS Thread for TCPIP Tasks. */
    xTaskCreate((TaskFunction_t) _TCPIP_Tasks,
                "TCPIP Tasks",
                TASK_SIZE_TCPIP_TASKS, NULL, TASK_PRIORITY_TCPIP_TASKS, &tcpipTaskHandle);

    /* Create OS Thread for APP Tasks. */
    xTaskCreate((TaskFunction_t) _APP_Tasks,
                "APP Tasks",
                TASK_SIZE_APP_TASKS, NULL, TASK_PRIORITY_APP_TASKS, &appTaskHandle);

        /* Create OS Thread for LORA Tasks. */
    xTaskCreate((TaskFunction_t) _LORA_Tasks,
                "LORA Tasks",
                TASK_SIZE_LORA_TASKS, NULL, TASK_PRIORITY_LORA_TASKS, &loraTaskHandle);

    /**************
     * Start RTOS * 
     **************/
    vTaskStartScheduler(); /* This function never returns. */
}


/*******************************************************************************
  Function:
    void _SYS_Tasks ( void )

  Summary:
    Maintains state machines of system modules.
*/
static void _SYS_Tasks ( void)
{
    TIMEOUT_THREAD_0_START;
    while(1)
    {
        /* Maintain system services */
        SYS_DEVCON_Tasks(sysObj.sysDevcon);
        /* Maintain the file system state machine. */
        SYS_FS_Tasks();
        SYS_CONSOLE_Tasks(sysObj.sysConsole0);
        /* SYS_COMMAND layer tasks routine */ 
        SYS_CMD_Tasks();
        /* SYS_TMR Device layer tasks routine */ 
        SYS_TMR_Tasks(sysObj.sysTmr);

        /* Maintain Device Drivers */
        DRV_SST25VF064C_Tasks(sysObj.drvSst25vf064c0);
        DRV_SDCARD_Tasks(sysObj.drvSDCard);

        /* Maintain Middleware */
        NET_PRES_Tasks(sysObj.netPres);
        /* Maintain the TCP/IP Stack*/

        wdt_arm_thread_0();
        wdt_kick();
        if(TIMEOUT_THREAD_0(10))
        {
            TIMEOUT_THREAD_0_START;
            SYS_DEBUG(SYS_ERROR_INFO, "MON: SYS Stack size: %d\r\n",uxTaskGetStackHighWaterMark(sysTaskHandle));
            SYS_DEBUG(SYS_ERROR_INFO, "MON: TCPIP Stack size: %d\r\n",uxTaskGetStackHighWaterMark(tcpipTaskHandle));
            SYS_DEBUG(SYS_ERROR_INFO, "MON: APP Stack size: %d\r\n",uxTaskGetStackHighWaterMark(appTaskHandle));
            SYS_DEBUG(SYS_ERROR_INFO, "MON: LoRa Stack size: %d\r\n",uxTaskGetStackHighWaterMark(loraTaskHandle));


            size_t libc_heap_usage_total = TrackHeap_TotalUsage();
            size_t libc_heap_usage_max = TrackHeap_MaxUsage();
            SYS_DEBUG(SYS_ERROR_INFO, "MON: heap usage: %dKB (%dKB), free: %dKB\r\n", libc_heap_usage_total / 1024, libc_heap_usage_max / 1024,xPortGetFreeHeapSize() / 1024);
        }
        /* Task Delay */
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void _TCPIP_Tasks(void)
{
    while(1)
    {
        /* Maintain the TCP/IP Stack*/
        TCPIP_STACK_Task(sysObj.tcpip);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

/*******************************************************************************
  Function:
    void _APP_Tasks ( void )

  Summary:
    Maintains state machine of APP.
*/

static void _APP_Tasks(void)
{
    vTaskDelay(200 / portTICK_PERIOD_MS); // Poor mans solution to give the debug log time to print the boot header
    while(1)
    {        
        APP_Tasks();
        wdt_arm_thread_1(); 
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}


/*******************************************************************************
  Function:
    void _LORA_Tasks ( void )

  Summary:
    Maintains state machine of LORA.
*/

static void _LORA_Tasks(void)
{
    while(1)
    { 
        LORA_Tasks();
        wdt_arm_thread_2();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}


/*******************************************************************************
 End of File
 */

