/*
    FreeRTOS V8.2.2 - Copyright (C) 2015 Real Time Engineers Ltd. 
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that has become a de facto standard.             *
     *                                                                       *
     *    Help yourself get started quickly and support the FreeRTOS         *
     *    project by purchasing a FreeRTOS tutorial book, reference          *
     *    manual, or both from: http://www.FreeRTOS.org/Documentation        *
     *                                                                       *
     *    Thank you!                                                         *
     *                                                                       *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available from the following
    link: http://www.freertos.org/a00114.html

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
    Integrity Systems to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <xc.h>

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/


#define configUSE_PREEMPTION                    <#if CONFIG_FREERTOS_PREEMPTIVE_SCHEDULER == true>1<#else>0</#if>
#define configUSE_PORT_OPTIMISED_TASK_SELECTION <#if CONFIG_FREERTOS_PORT_OPTIMIZED_TASK_SELECTION == true>1<#else>0</#if>
#define configUSE_TICKLESS_IDLE                 <#if CONFIG_FREERTOS_TICKLESS_IDLE == true>1<#else>0</#if>
<#if CONFIG_FREERTOS_TICKLESS_IDLE == true>
#define configEXPECTED_IDLE_TIME_BEFORE_SLEEP   ${CONFIG_FREERTOS_EXPECTED_IDLE_TIME_BEFORE_SLEEP}
</#if>
#define configCPU_CLOCK_HZ                      ( ${CONFIG_FREERTOS_CPU_CLOCK_HZ?number?c}UL )
#define configPERIPHERAL_CLOCK_HZ               ( ${CONFIG_FREERTOS_PERIPHERAL_CLOCK_HZ?number?c}UL )
<#if CONFIG_FREERTOS_ENABLE_BACKWARD_COMPATIBILITY == true>
#define configTICK_RATE_HZ                      ( ( portTickType ) ${CONFIG_FREERTOS_TICK_RATE_HZ} )
<#else>
#define configTICK_RATE_HZ                      ( ( TickType_t ) ${CONFIG_FREERTOS_TICK_RATE_HZ} )
</#if>
#define configMAX_PRIORITIES                    ( ${CONFIG_FREERTOS_MAX_PRIORITIES}UL )
#define configMINIMAL_STACK_SIZE                ( ${CONFIG_FREERTOS_MINIMAL_STACK_SIZE} )
#define configISR_STACK_SIZE                    ( ${CONFIG_FREERTOS_ISR_STACK_SIZE} )
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ${CONFIG_FREERTOS_TOTAL_HEAP_SIZE} )
#define configMAX_TASK_NAME_LEN                 ( ${CONFIG_FREERTOS_MAX_TASK_NAME_LEN} )
#define configUSE_16_BIT_TICKS                  <#if CONFIG_FREERTOS_USE_16_BIT_TICKS == true>1<#else>0</#if>
<#if CONFIG_FREERTOS_PREEMPTIVE_SCHEDULER == true>
#define configIDLE_SHOULD_YIELD                 <#if CONFIG_FREERTOS_IDLE_SHOULD_YIELD == true>1<#else>0</#if>
</#if>
#define configUSE_MUTEXES                       <#if CONFIG_FREERTOS_USE_MUTEXES == true>1<#else>0</#if>
#define configUSE_RECURSIVE_MUTEXES             <#if CONFIG_FREERTOS_USE_RECURSIVE_MUTEXES == true>1<#else>0</#if>
#define configUSE_COUNTING_SEMAPHORES           <#if CONFIG_FREERTOS_USE_COUNTING_SEMAPHORES == true>1<#else>0</#if>
#define configUSE_TASK_NOTIFICATIONS            <#if CONFIG_FREERTOS_USE_TASK_NOTIFICATIONS == true>1<#else>0</#if>
#define configQUEUE_REGISTRY_SIZE               ${CONFIG_FREERTOS_QUEUE_REGISTRY_SIZE}
#define configUSE_QUEUE_SETS                    <#if CONFIG_FREERTOS_USE_QUEUE_SETS == true>1<#else>0</#if>
#define configUSE_TIME_SLICING                  <#if CONFIG_FREERTOS_USE_TIME_SLICING == true>1<#else>0</#if>
#define configUSE_NEWLIB_REENTRANT              <#if CONFIG_FREERTOS_USE_NEWLIB_REENTRANT == true>1<#else>0</#if>
#define configENABLE_BACKWARD_COMPATIBILITY     <#if CONFIG_FREERTOS_ENABLE_BACKWARD_COMPATIBILITY == true>1<#else>0</#if>
#define configUSE_TASK_FPU_SUPPORT              <#if CONFIG_FREERTOS_USE_TASK_FPU_SUPPORT == true>1<#else>0</#if>

/* Hook function related definitions. */
#define configUSE_IDLE_HOOK                     <#if CONFIG_FREERTOS_IDLE_HOOK == true>1<#else>0</#if>
#define configUSE_TICK_HOOK                     <#if CONFIG_FREERTOS_TICK_HOOK == true>1<#else>0</#if>
#define configCHECK_FOR_STACK_OVERFLOW          <#if CONFIG_FREERTOS_NO_STACK_OVERFLOW_CHECK == true>0<#else><#if CONFIG_FREERTOS_STACK_OVERFLOW_CHECK_METHOD_1 == true>1<#else>2</#if></#if>
#define configUSE_MALLOC_FAILED_HOOK            <#if CONFIG_FREERTOS_USE_MALLOC_FAILED_HOOK == true>1<#else>0</#if>

/* Run time and task stats gathering related definitions. */
#define configGENERATE_RUN_TIME_STATS           <#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS == true>1<#else>0</#if>
#define configUSE_TRACE_FACILITY                <#if CONFIG_FREERTOS_USE_TRACE_FACILITY == true>1<#else>0</#if>
#define configUSE_STATS_FORMATTING_FUNCTIONS    <#if CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS == true>1<#else>0</#if>

/* Co-routine related definitions. */
#define configUSE_CO_ROUTINES                   <#if CONFIG_FREERTOS_USE_CO_ROUTINES == true>1<#else>0</#if>
#define configMAX_CO_ROUTINE_PRIORITIES         ${CONFIG_FREERTOS_MAX_CO_ROUTINE_PRIORITIES}

/* Software timer related definitions. */
#define configUSE_TIMERS                        <#if CONFIG_FREERTOS_USE_TIMERS == true>1<#else>0</#if>
#define configTIMER_TASK_PRIORITY               ${CONFIG_FREERTOS_TIMER_TASK_PRIORITY}
#define configTIMER_QUEUE_LENGTH                ${CONFIG_FREERTOS_TIMER_QUEUE_LENGTH}
#define configTIMER_TASK_STACK_DEPTH            ${CONFIG_FREERTOS_TIMER_TASK_STACK_DEPTH}

/* Misc */
#define configUSE_APPLICATION_TASK_TAG          <#if CONFIG_FREERTOS_USE_APPLICATION_TASK_TAG == true>1<#else>0</#if>

<#if CONFIG_FREERTOS_USE_CONFIGASSERT == true>
/* Prevent C specific syntax being included in assembly files. */
#ifndef __LANGUAGE_ASSEMBLY
    void vAssertCalled( const char *pcFileName, unsigned long ulLine );
    #define configASSERT( x ) if( ( x ) == 0 ) vAssertCalled( __FILE__, __LINE__ )
#endif
</#if>

/* Interrupt nesting behaviour configuration. */

/* The priority at which the tick interrupt runs.  This should probably be kept at 1. */
#define configKERNEL_INTERRUPT_PRIORITY         ${CONFIG_FREERTOS_KERNEL_INTERRUPT_PRIORITY}

/* The maximum interrupt priority from which FreeRTOS.org API functions can be called.  
Only API functions that end in ...FromISR() can be used within interrupts. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    ${CONFIG_FREERTOS_MAX_SYSCALL_INTERRUPT_PRIORITY}

/* Optional functions - most linkers will remove unused functions anyway. */
#define INCLUDE_vTaskPrioritySet                <#if CONFIG_FREERTOS_INCLUDE_VTASKPRIORITYSET == true>1<#else>0</#if>
#define INCLUDE_uxTaskPriorityGet               <#if CONFIG_FREERTOS_INCLUDE_UXTASKPRIORITYGET == true>1<#else>0</#if>
#define INCLUDE_vTaskDelete                     <#if CONFIG_FREERTOS_INCLUDE_VTASKDELETE == true>1<#else>0</#if>
#define INCLUDE_vTaskCleanUpResources           <#if CONFIG_FREERTOS_INCLUDE_VTASKCLEANUPRESOURCES == true>1<#else>0</#if>
#define INCLUDE_vTaskSuspend                    <#if CONFIG_FREERTOS_INCLUDE_VTASKSUSPEND == true>1<#else>0</#if>
#define INCLUDE_vTaskDelayUntil                 <#if CONFIG_FREERTOS_INCLUDE_VTASKDELAYUNTIL == true>1<#else>0</#if>
#define INCLUDE_vTaskDelay                      <#if CONFIG_FREERTOS_INCLUDE_VTASKDELAY == true>1<#else>0</#if>
#define INCLUDE_xTaskGetSchedulerState          <#if CONFIG_FREERTOS_INCLUDE_XTASKGETSCHEDULERSTATE == true>1<#else>0</#if>
#define INCLUDE_xTaskGetCurrentTaskHandle       <#if CONFIG_FREERTOS_INCLUDE_XTASKGETCURRENTTASKHANDLE == true>1<#else>0</#if>
#define INCLUDE_uxTaskGetStackHighWaterMark     <#if CONFIG_FREERTOS_INCLUDE_UXTASKGETSTACKHIGHWATERMARK == true>1<#else>0</#if>
#define INCLUDE_xTaskGetIdleTaskHandle          <#if CONFIG_FREERTOS_INCLUDE_XTASKGETIDLETASKHANDLE == true>1<#else>0</#if>
#define INCLUDE_xTimerGetTimerDaemonTaskHandle  <#if CONFIG_FREERTOS_INCLUDE_XTASKGETTIMERDAEMONTASKHANDLE == true>1<#else>0</#if>
#define INCLUDE_pcTaskGetTaskName               <#if CONFIG_FREERTOS_INCLUDE_PCTASKGETTASKNAME == true>1<#else>0</#if>
#define INCLUDE_eTaskGetState                   <#if CONFIG_FREERTOS_INCLUDE_ETASKGETSTATE == true>1<#else>0</#if>
#define INCLUDE_xEventGroupSetBitFromISR        <#if CONFIG_FREERTOS_INCLUDE_XEVENTGROUPSETBITFROMISR == true>1<#else>0</#if>
#define INCLUDE_xTimerPendFunctionCall          <#if CONFIG_FREERTOS_INCLUDE_XTIMERPENDFUNCTIONCALL == true>1<#else>0</#if>

#endif /* FREERTOS_CONFIG_H */
