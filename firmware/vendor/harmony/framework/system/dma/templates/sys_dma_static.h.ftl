/*******************************************************************************
  DMA System Service Interface Declarations for Static Single Instance System Service

  Company:
    Microchip Technology Inc.

  File Name:
    drv_dma_static.h

  Summary:
    DMA driver interface declarations for the static single instance System Service.

  Description:
    The DMA System Service provides a simple interface to manage the DMA
    modules on Microchip microcontrollers. This file defines the interface
    Declarations for the DMA.
    
  Remarks:
    Static interfaces incorporate the System Service instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    
    Static single-open interfaces also eliminate the need for the open handle.
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
//DOM-IGNORE-END


#ifndef _DRV_DMA_STATIC_H
#define _DRV_DMA_STATIC_H

#include "system/dma/sys_dma.h"
#include "peripheral/int/plib_int.h"

<#if CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE0 == true>
extern void ${CONFIG_DRV_DMA_CALL_BACK_FUNCTION0}(DMA_INT_TYPE dmaINTSource);
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE1 == true>
extern void ${CONFIG_DRV_DMA_CALL_BACK_FUNCTION1}(DMA_INT_TYPE dmaINTSource);
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE2 == true>
extern void ${CONFIG_DRV_DMA_CALL_BACK_FUNCTION2}(DMA_INT_TYPE dmaINTSource);
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE3 == true>
extern void ${CONFIG_DRV_DMA_CALL_BACK_FUNCTION3}(DMA_INT_TYPE dmaINTSource);
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE4 == true>
extern void ${CONFIG_DRV_DMA_CALL_BACK_FUNCTION4}(DMA_INT_TYPE dmaINTSource);
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE5 == true>
extern void ${CONFIG_DRV_DMA_CALL_BACK_FUNCTION5}(DMA_INT_TYPE dmaINTSource);
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE6 == true>
extern void ${CONFIG_DRV_DMA_CALL_BACK_FUNCTION6}(DMA_INT_TYPE dmaINTSource);
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE7 == true>
extern void ${CONFIG_DRV_DMA_CALL_BACK_FUNCTION7}(DMA_INT_TYPE dmaINTSource);
</#if>

<#macro DRV_DMA_STATIC_FUNCTIONS INSTANCE M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE>	
void SYS_DMA_Channel${INSTANCE}TransferAdd(void);

void SYS_DMA_Channel${INSTANCE}Enable(void);

void SYS_DMA_Channel${INSTANCE}Setup(SYS_DMA_CHANNEL_OP_MODE modeEnable);

void SYS_DMA_Channel${INSTANCE}ForceStart(void);


bool SYS_DMA_Channel${INSTANCE}IsBusy(void);
void SYS_DMA_Channel${INSTANCE}ForceAbort(void);
void SYS_DMA_Channel${INSTANCE}ForceStart(void);
void SYS_DMA_Channel${INSTANCE}Disable(void);
void SYS_DMA_Suspend(void);
void SYS_DMA_Resume(void);
bool SYS_DMA_IsBusy(void);
uint32_t SYS_DMA_Channel${INSTANCE}GetCRC(void);
void SYS_DMA_Channel${INSTANCE}SetCRC(SYS_DMA_CHANNEL_OPERATION_MODE_CRC crc);
void SYS_DMA_Channel${INSTANCE}SetupMatchAbortMode( uint16_t pattern,
                                         DMA_PATTERN_LENGTH length,
                                         SYS_DMA_CHANNEL_IGNORE_MATCH ignore,
                                         uint8_t ignorePattern );
<#if (M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE == false)>
__attribute__((weak)) void SYS_DMA_Tasks${INSTANCE}(void);
<#else>
void SYS_DMA_Tasks${INSTANCE}(void);
</#if>
</#macro>

// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance for the static System Service
// *****************************************************************************
// *****************************************************************************

<#if CONFIG_SYS_DMA_CHANNEL_0 == true>
<@DRV_DMA_STATIC_FUNCTIONS INSTANCE=0
M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE=CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE0
/>
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_1 == true>
<@DRV_DMA_STATIC_FUNCTIONS INSTANCE=1
M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE=CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE1
/>
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_2 == true>
<@DRV_DMA_STATIC_FUNCTIONS INSTANCE=2
M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE=CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE2
/>
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_3 == true>
<@DRV_DMA_STATIC_FUNCTIONS INSTANCE=3
M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE=CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE3
/>
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_4 == true>
<@DRV_DMA_STATIC_FUNCTIONS INSTANCE=4
M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE=CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE4
/>
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_5 == true>
<@DRV_DMA_STATIC_FUNCTIONS INSTANCE=5
M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE=CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE5
/>
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_6 == true>
<@DRV_DMA_STATIC_FUNCTIONS INSTANCE=6
M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE=CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE6
/>
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_7 == true>
<@DRV_DMA_STATIC_FUNCTIONS INSTANCE=7
M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE=CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE7
/>
</#if>

#endif // #ifndef _DRV_DMA_STATIC_H

/*******************************************************************************
 End of File
*/
