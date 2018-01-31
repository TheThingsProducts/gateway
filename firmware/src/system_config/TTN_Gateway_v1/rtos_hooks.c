/*******************************************************************************
 System Tasks File

  File Name:
    rtos_hooks.c

  Summary:
    This file contains source code necessary for rtos hooks

  Description:

  Remarks:
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
// DOM-IGNORE-END
#include "FreeRTOS.h"
#include "task.h"
#include "system_definitions.h"
#include "utilities.h"

/*
*********************************************************************************************************
*                                          vApplicationStackOverflowHook()
*
* Description : Hook function called by FreeRTOS if a stack overflow happens.
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : APP_StateReset()
*
* Note(s)     : none.
*********************************************************************************************************
*/
void vApplicationStackOverflowHook( TaskHandle_t pxTask, signed char *pcTaskName )
{
   ( void ) pcTaskName;
   ( void ) pxTask;

   /* Run time task stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook	function is
   called if a task stack overflow is detected.  Note the system/interrupt
   stack is not checked. */
   FATAL("vApplicationStackOverflowHook in %s\n", pcTaskName);
}

/*
*********************************************************************************************************
*                                     vApplicationMallocFailedHook()
*
* Description : vApplicationMallocFailedHook() will only be called if
*               configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.
*               It is a hook function that will get called if a call to
*               pvPortMalloc() fails.  pvPortMalloc() is called internally by
*               the kernel whenever a task, queue, timer or semaphore is
*               created.  It is also called by various parts of the demo
*               application.  If heap_1.c or heap_2.c are used, then the size of
*               the heap available to pvPortMalloc() is defined by
*               configTOTAL_HEAP_SIZE in FreeRTOSConfig.h, and the
*               xPortGetFreeHeapSize() API function can be used to query the
*               size of free heap space that remains (although it does not
*               provide information on how the remaining heap might be
*               fragmented).
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : APP_StateReset()
*
* Note(s)     : none.
*********************************************************************************************************
*/

void vApplicationMallocFailedHook( void )
{
   /* vApplicationMallocFailedHook() will only be called if
      configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
      function that will get called if a call to pvPortMalloc() fails.
      pvPortMalloc() is called internally by the kernel whenever a task, queue,
      timer or semaphore is created.  It is also called by various parts of the
      demo application.  If heap_1.c or heap_2.c are used, then the size of the
      heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
      FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
      to query the size of free heap space that remains (although it does not
      provide information on how the remaining heap might be fragmented). */
    
    FATAL("vApplicationMallocFailedHook");
}



/*******************************************************************************
 End of File
 */
