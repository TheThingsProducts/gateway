/*******************************************************************************
 System Interrupt Source File

  File Name:
    sys_interrupt_a.S

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the 
    interrupt sub-system.
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
/*
*********************************************************************************************************
*                                           INCLUDES
*********************************************************************************************************
*/
#include <xc.h>



#include "ISR_Support.h"






 /* Port F Change Notice Interrupt */
   .extern  IntHandlerChangeNotification_PortF

   .section	.vector_123,code, keep
   .equ     __vector_dispatch_123, IntVectorChangeNotification_PortF
   .global  __vector_dispatch_123
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorChangeNotification_PortF

IntVectorChangeNotification_PortF:
    portSAVE_CONTEXT
    la    s6,  IntHandlerChangeNotification_PortF
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorChangeNotification_PortF


 





/* TMR Instance 0 Interrupt */
   .extern  IntHandlerDrvTmrInstance0

   .section	.vector_9,code, keep
   .equ     __vector_dispatch_9, IntVectorDrvTmrInstance0
   .global  __vector_dispatch_9
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvTmrInstance0

IntVectorDrvTmrInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvTmrInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvTmrInstance0


/* TMR Instance 1 Interrupt */
   .extern  IntHandlerDrvTmrInstance1

   .section	.vector_14,code, keep
   .equ     __vector_dispatch_14, IntVectorDrvTmrInstance1
   .global  __vector_dispatch_14
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvTmrInstance1

IntVectorDrvTmrInstance1:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvTmrInstance1
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvTmrInstance1




/* USART Instance 0 Interrupt */

   .extern  IntHandlerDrvUsartReceiveInstance0

   .section	.vector_171,code, keep
   .equ     __vector_dispatch_171, IntVectorDrvUsartReceiveInstance0
   .global  __vector_dispatch_171
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvUsartReceiveInstance0

IntVectorDrvUsartReceiveInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvUsartReceiveInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvUsartReceiveInstance0


   .extern  IntHandlerDrvUsartTransmitInstance0

   .section	.vector_172,code, keep
   .equ     __vector_dispatch_172, IntVectorDrvUsartTransmitInstance0
   .global  __vector_dispatch_172
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvUsartTransmitInstance0

IntVectorDrvUsartTransmitInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvUsartTransmitInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvUsartTransmitInstance0


   .extern  IntHandlerDrvUsartErrorInstance0

   .section	.vector_170,code, keep
   .equ     __vector_dispatch_170, IntVectorDrvUsartErrorInstance0
   .global  __vector_dispatch_170
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvUsartErrorInstance0

IntVectorDrvUsartErrorInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvUsartErrorInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvUsartErrorInstance0



/* USART Instance 1 Interrupt */

   .extern  IntHandlerDrvUsartReceiveInstance1

   .section	.vector_113,code, keep
   .equ     __vector_dispatch_113, IntVectorDrvUsartReceiveInstance1
   .global  __vector_dispatch_113
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvUsartReceiveInstance1

IntVectorDrvUsartReceiveInstance1:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvUsartReceiveInstance1
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvUsartReceiveInstance1


   .extern  IntHandlerDrvUsartTransmitInstance1

   .section	.vector_114,code, keep
   .equ     __vector_dispatch_114, IntVectorDrvUsartTransmitInstance1
   .global  __vector_dispatch_114
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvUsartTransmitInstance1

IntVectorDrvUsartTransmitInstance1:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvUsartTransmitInstance1
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvUsartTransmitInstance1


   .extern  IntHandlerDrvUsartErrorInstance1

   .section	.vector_112,code, keep
   .equ     __vector_dispatch_112, IntVectorDrvUsartErrorInstance1
   .global  __vector_dispatch_112
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvUsartErrorInstance1

IntVectorDrvUsartErrorInstance1:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvUsartErrorInstance1
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvUsartErrorInstance1



/* USART Instance 2 Interrupt */

   .extern  IntHandlerDrvUsartReceiveInstance2

   .section	.vector_158,code, keep
   .equ     __vector_dispatch_158, IntVectorDrvUsartReceiveInstance2
   .global  __vector_dispatch_158
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvUsartReceiveInstance2

IntVectorDrvUsartReceiveInstance2:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvUsartReceiveInstance2
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvUsartReceiveInstance2


   .extern  IntHandlerDrvUsartTransmitInstance2

   .section	.vector_159,code, keep
   .equ     __vector_dispatch_159, IntVectorDrvUsartTransmitInstance2
   .global  __vector_dispatch_159
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvUsartTransmitInstance2

IntVectorDrvUsartTransmitInstance2:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvUsartTransmitInstance2
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvUsartTransmitInstance2


   .extern  IntHandlerDrvUsartErrorInstance2

   .section	.vector_157,code, keep
   .equ     __vector_dispatch_157, IntVectorDrvUsartErrorInstance2
   .global  __vector_dispatch_157
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvUsartErrorInstance2

IntVectorDrvUsartErrorInstance2:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvUsartErrorInstance2
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvUsartErrorInstance2



/* SPI Instance 0 Interrupt */
   .extern  IntHandlerSPIRxInstance0

   .section	.vector_143,code, keep
   .equ     __vector_dispatch_143, IntVectorSPIRxInstance0
   .global  __vector_dispatch_143
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorSPIRxInstance0

IntVectorSPIRxInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerSPIRxInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorSPIRxInstance0


   .extern  IntHandlerSPITxInstance0

   .section	.vector_144,code, keep
   .equ     __vector_dispatch_144, IntVectorSPITxInstance0
   .global  __vector_dispatch_144
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorSPITxInstance0

IntVectorSPITxInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerSPITxInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorSPITxInstance0


   .extern  IntHandlerSPIFaultInstance0

   .section	.vector_142,code, keep
   .equ     __vector_dispatch_142, IntVectorSPIFaultInstance0
   .global  __vector_dispatch_142
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorSPIFaultInstance0

IntVectorSPIFaultInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerSPIFaultInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorSPIFaultInstance0


/* SPI Instance 1 Interrupt */
   .extern  IntHandlerSPIRxInstance1

   .section	.vector_186,code, keep
   .equ     __vector_dispatch_186, IntVectorSPIRxInstance1
   .global  __vector_dispatch_186
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorSPIRxInstance1

IntVectorSPIRxInstance1:
    portSAVE_CONTEXT
    la    s6,  IntHandlerSPIRxInstance1
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorSPIRxInstance1


   .extern  IntHandlerSPITxInstance1

   .section	.vector_187,code, keep
   .equ     __vector_dispatch_187, IntVectorSPITxInstance1
   .global  __vector_dispatch_187
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorSPITxInstance1

IntVectorSPITxInstance1:
    portSAVE_CONTEXT
    la    s6,  IntHandlerSPITxInstance1
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorSPITxInstance1


   .extern  IntHandlerSPIFaultInstance1

   .section	.vector_185,code, keep
   .equ     __vector_dispatch_185, IntVectorSPIFaultInstance1
   .global  __vector_dispatch_185
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorSPIFaultInstance1

IntVectorSPIFaultInstance1:
    portSAVE_CONTEXT
    la    s6,  IntHandlerSPIFaultInstance1
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorSPIFaultInstance1


/* SPI Instance 2 Interrupt */
   .extern  IntHandlerSPIRxInstance2

   .section	.vector_110,code, keep
   .equ     __vector_dispatch_110, IntVectorSPIRxInstance2
   .global  __vector_dispatch_110
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorSPIRxInstance2

IntVectorSPIRxInstance2:
    portSAVE_CONTEXT
    la    s6,  IntHandlerSPIRxInstance2
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorSPIRxInstance2


   .extern  IntHandlerSPITxInstance2

   .section	.vector_111,code, keep
   .equ     __vector_dispatch_111, IntVectorSPITxInstance2
   .global  __vector_dispatch_111
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorSPITxInstance2

IntVectorSPITxInstance2:
    portSAVE_CONTEXT
    la    s6,  IntHandlerSPITxInstance2
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorSPITxInstance2


   .extern  IntHandlerSPIFaultInstance2

   .section	.vector_109,code, keep
   .equ     __vector_dispatch_109, IntVectorSPIFaultInstance2
   .global  __vector_dispatch_109
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorSPIFaultInstance2

IntVectorSPIFaultInstance2:
    portSAVE_CONTEXT
    la    s6,  IntHandlerSPIFaultInstance2
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorSPIFaultInstance2


/* NVM Interrupt */
   .extern  IntHandlerDrvNvm

   .section	.vector_167,code, keep
   .equ     __vector_dispatch_167, IntVectorDrvNvm
   .global  __vector_dispatch_167
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorDrvNvm

IntVectorDrvNvm:
    portSAVE_CONTEXT
    la    s6,  IntHandlerDrvNvm
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorDrvNvm



/* Sytem DMA Instance 0 Interrupt */
   .extern  IntHandlerSysDmaInstance0

   .section	.vector_134,code, keep
   .equ     __vector_dispatch_134, IntVectorSysDmaInstance0
   .global  __vector_dispatch_134
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorSysDmaInstance0

IntVectorSysDmaInstance0:
    portSAVE_CONTEXT
    la    s6,  IntHandlerSysDmaInstance0
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorSysDmaInstance0


/* Sytem DMA Instance 1 Interrupt */
   .extern  IntHandlerSysDmaInstance1

   .section	.vector_135,code, keep
   .equ     __vector_dispatch_135, IntVectorSysDmaInstance1
   .global  __vector_dispatch_135
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVectorSysDmaInstance1

IntVectorSysDmaInstance1:
    portSAVE_CONTEXT
    la    s6,  IntHandlerSysDmaInstance1
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVectorSysDmaInstance1


/* ETHMAC Interrupt */
   .extern  IntHandler_ETHMAC

   .section	.vector_153,code, keep
   .equ     __vector_dispatch_153, IntVector_ETHMAC
   .global  __vector_dispatch_153
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVector_ETHMAC

IntVector_ETHMAC:
    portSAVE_CONTEXT
    la    s6,  IntHandler_ETHMAC
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVector_ETHMAC



/*******************************************************************************
 End of File
 */

