/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

#include <xc.h>
#include <sys/attribs.h>
#include "app.h"
#include "lora.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
void IntHandlerChangeNotification_PortF(void)
{
    
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_CHANGE_NOTICE_F);
    WDRV_MRF24WN_ISR((SYS_MODULE_OBJ)0);
}


    
void IntHandlerDrvTmrInstance0(void)
{
    DRV_TMR_Tasks(sysObj.drvTmr0);
}
  
void IntHandlerDrvTmrInstance1(void)
{
    DRV_TMR_Tasks(sysObj.drvTmr1);
}
  
void IntHandlerDrvUsartTransmitInstance0(void)
{
    DRV_USART_TasksTransmit(sysObj.drvUsart0);
}
void IntHandlerDrvUsartReceiveInstance0(void)
{
    DRV_USART_TasksReceive(sysObj.drvUsart0);
}
void IntHandlerDrvUsartErrorInstance0(void)
{
    DRV_USART_TasksError(sysObj.drvUsart0);
}
 
 

 
void IntHandlerDrvUsartTransmitInstance1(void)
{
    DRV_USART_TasksTransmit(sysObj.drvUsart1);
}
void IntHandlerDrvUsartReceiveInstance1(void)
{
    DRV_USART_TasksReceive(sysObj.drvUsart1);
}
void IntHandlerDrvUsartErrorInstance1(void)
{
    DRV_USART_TasksError(sysObj.drvUsart1);
}
 
 

 
void IntHandlerDrvUsartTransmitInstance2(void)
{
    DRV_USART_TasksTransmit(sysObj.drvUsart2);
}
void IntHandlerDrvUsartReceiveInstance2(void)
{
    DRV_USART_TasksReceive(sysObj.drvUsart2);
}
void IntHandlerDrvUsartErrorInstance2(void)
{
    DRV_USART_TasksError(sysObj.drvUsart2);
}

 

 

 
 
 
 
void IntHandlerSPIRxInstance0(void)
{
    DRV_SPI_Tasks(sysObj.spiObjectIdx0);
}
void IntHandlerSPITxInstance0(void)
{
    DRV_SPI_Tasks(sysObj.spiObjectIdx0);
}
void IntHandlerSPIFaultInstance0(void)
{
    DRV_SPI_Tasks(sysObj.spiObjectIdx0);
}
void IntHandlerSPIRxInstance1(void)
{
    DRV_SPI_Tasks(sysObj.spiObjectIdx1);
}
void IntHandlerSPITxInstance1(void)
{
    DRV_SPI_Tasks(sysObj.spiObjectIdx1);
}
void IntHandlerSPIFaultInstance1(void)
{
    DRV_SPI_Tasks(sysObj.spiObjectIdx1);
}
void IntHandlerSPIRxInstance2(void)
{
    DRV_SPI_Tasks(sysObj.spiObjectIdx2);
}
void IntHandlerSPITxInstance2(void)
{
    DRV_SPI_Tasks(sysObj.spiObjectIdx2);
}
void IntHandlerSPIFaultInstance2(void)
{
    DRV_SPI_Tasks(sysObj.spiObjectIdx2);
}

void IntHandlerDrvNvm (void)

{
    DRV_NVM_Tasks(sysObj.drvNvm);

}


void IntHandlerSysDmaInstance0(void)
{          
    SYS_DMA_TasksISR(sysObj.sysDma, DMA_CHANNEL_0);
}

void IntHandlerSysDmaInstance1(void)
{          
    SYS_DMA_TasksISR(sysObj.sysDma, DMA_CHANNEL_1);
}


void IntHandler_ETHMAC(void)
{
    DRV_ETHMAC_Tasks_ISR((SYS_MODULE_OBJ)0);
}

/* This function is used by ETHMAC driver */
bool SYS_INT_SourceRestore(INT_SOURCE src, int level)
{
    if(level)
    {
        SYS_INT_SourceEnable(src);
    }
 
    return level;
}


 
/*******************************************************************************
 End of File
*/

