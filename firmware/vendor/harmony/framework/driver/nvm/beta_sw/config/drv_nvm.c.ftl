<#--
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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
 -->

/*** FLASH Driver Initialization Data ***/
<#if CONFIG_USE_DRV_NVM == true>
const DRV_NVM_INIT drvNvmInit =
{
<#if CONFIG_DRV_NVM_POWER_STATE?has_content>
    .moduleInit.sys.powerState = ${CONFIG_DRV_NVM_POWER_STATE},
</#if>
<#if CONFIG_DRV_NVM_PERIPHERAL_ID?has_content>
    .nvmID = ${CONFIG_DRV_NVM_PERIPHERAL_ID},
</#if>
<#if CONFIG_DRV_NVM_INTERRUPT_SOURCE?has_content>
    .interruptSource = ${CONFIG_DRV_NVM_INTERRUPT_SOURCE}
</#if>
};
<#if CONFIG_USE_DRV_NVM_MEDIA == true>
const DRV_NVM_MEDIA_INIT drvNvmMediaInit =
{
    .mediaStartAddress = (uintptr_t)NVM_MEDIA_DATA,
<#if CONFIG_DRV_NVM_MEDIA_SECTOR_SIZE?has_content>
    .nSectors = NVM_MEDIA_SIZE/NVM_MEDIA_SECTOR_SIZE,
    .sectorSizeInBytes = NVM_MEDIA_SECTOR_SIZE,
</#if>
<#if CONFIG_DRV_NVM_INDEX?has_content>
    .drvNVMIndex = ${CONFIG_DRV_NVM_INDEX}
</#if>
};
</#if>
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
