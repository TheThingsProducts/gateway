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
// <editor-fold defaultstate="collapsed" desc="DRV_SST25VF064C Initialization Data">
<#if CONFIG_DRV_SST25VF064C_DRIVER_MODE == "DYNAMIC">
// *****************************************************************************
/* SST25VF064C Driver Initialization Data
*/
<#-- Instance 0 -->
<#if CONFIG_DRV_SST25VF064C_INST_IDX0 == true>

const DRV_SST25VF064C_INIT drvSst25vf064c0InitData =
{
<#if CONFIG_DRV_SST25VF064C_POWER_STATE_IDX0?has_content>
    .moduleInit = {DRV_SST25VF064C_POWER_STATE_IDX0},
</#if>
<#if CONFIG_DRV_SST25VF064C_SPI_DRIVER_INSTANCE_IDX0?has_content>
    .spiDriverModuleIndex = DRV_SST25VF064C_SPI_DRIVER_INSTANCE_IDX0, 
</#if>
<#if CONFIG_DRV_SST25VF064C_HOLD_PIN_PORT_CHANNEL_IDX0?has_content>
    .holdPortChannel = DRV_SST25VF064C_HOLD_PIN_PORT_CHANNEL_IDX0,
</#if>
<#if CONFIG_DRV_SST25VF064C_HOLD_PIN_PORT_BIT_POS_IDX0?has_content>
    .holdBitPosition = DRV_SST25VF064C_HOLD_PIN_PORT_BIT_POS_IDX0,
</#if>
<#if CONFIG_DRV_SST25VF064C_WRITE_PROTECT_PIN_PORT_CHANNEL_IDX0?has_content>
    .writeProtectPortChannel = DRV_SST25VF064C_WRITE_PROTECT_PIN_PORT_CHANNEL_IDX0,
</#if>
<#if CONFIG_DRV_SST25VF064C_WRITE_PROTECT_PIN_BIT_POS_IDX0?has_content>
    .writeProtectBitPosition = DRV_SST25VF064C_WRITE_PROTECT_PIN_BIT_POS_IDX0,
</#if>
<#if CONFIG_DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0?has_content>
    .chipSelectPortChannel = DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
</#if>
<#if CONFIG_DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0?has_content>
    .chipSelectBitPosition = DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0,
</#if>
<#if CONFIG_DRV_SST25VF064C_QUEUE_SIZE_IDX0?has_content>
    .queueSize = DRV_SST25VF064C_QUEUE_SIZE_IDX0,
</#if>
};
</#if>

<#-- Instance 1 -->
<#if CONFIG_DRV_SST25VF064C_INST_IDX1 == true>

const DRV_SST25VF064C_INIT drvSst25vf064c1InitData =
{
<#if CONFIG_DRV_SST25VF064C_POWER_STATE_IDX1?has_content>
    .moduleInit = {DRV_SST25VF064C_POWER_STATE_IDX1},
</#if>
<#if CONFIG_DRV_SST25VF064C_SPI_DRIVER_INSTANCE_IDX1?has_content>
    .spiDriverModuleIndex = DRV_SST25VF064C_SPI_DRIVER_INSTANCE_IDX1, 
</#if>
<#if CONFIG_DRV_SST25VF064C_HOLD_PIN_PORT_CHANNEL_IDX1?has_content>
    .holdPortChannel = DRV_SST25VF064C_HOLD_PIN_PORT_CHANNEL_IDX1,
</#if>
<#if CONFIG_DRV_SST25VF064C_HOLD_PIN_PORT_BIT_POS_IDX1?has_content>
    .holdBitPosition = DRV_SST25VF064C_HOLD_PIN_PORT_BIT_POS_IDX1,
</#if>
<#if CONFIG_DRV_SST25VF064C_WRITE_PROTECT_PIN_PORT_CHANNEL_IDX1?has_content>
    .writeProtectPortChannel = DRV_SST25VF064C_WRITE_PROTECT_PIN_PORT_CHANNEL_IDX1,
</#if>
<#if CONFIG_DRV_SST25VF064C_WRITE_PROTECT_PIN_BIT_POS_IDX1?has_content>
    .writeProtectBitPosition = DRV_SST25VF064C_WRITE_PROTECT_PIN_BIT_POS_IDX1,
</#if>
<#if CONFIG_DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX1?has_content>
    .chipSelectPortChannel = DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX1,
</#if>
<#if CONFIG_DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX1?has_content>
    .chipSelectBitPosition = DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX1,
</#if>
<#if CONFIG_DRV_SST25VF064C_QUEUE_SIZE_IDX1?has_content>
    .queueSize = DRV_SST25VF064C_QUEUE_SIZE_IDX1,
</#if>
};
</#if>

<#-- Instance 2 -->
<#if CONFIG_DRV_SST25VF064C_INST_IDX2 == true>

const DRV_SST25VF064C_INIT drvSst25vf064c2InitData =
{
<#if CONFIG_DRV_SST25VF064C_POWER_STATE_IDX2?has_content>
    .moduleInit = {DRV_SST25VF064C_POWER_STATE_IDX2},
</#if>
<#if CONFIG_DRV_SST25VF064C_SPI_DRIVER_INSTANCE_IDX2?has_content>
    .spiDriverModuleIndex = DRV_SST25VF064C_SPI_DRIVER_INSTANCE_IDX2, 
</#if>
<#if CONFIG_DRV_SST25VF064C_HOLD_PIN_PORT_CHANNEL_IDX2?has_content>
    .holdPortChannel = DRV_SST25VF064C_HOLD_PIN_PORT_CHANNEL_IDX2,
</#if>
<#if CONFIG_DRV_SST25VF064C_HOLD_PIN_PORT_BIT_POS_IDX2?has_content>
    .holdBitPosition = DRV_SST25VF064C_HOLD_PIN_PORT_BIT_POS_IDX2,
</#if>
<#if CONFIG_DRV_SST25VF064C_WRITE_PROTECT_PIN_PORT_CHANNEL_IDX2?has_content>
    .writeProtectPortChannel = DRV_SST25VF064C_WRITE_PROTECT_PIN_PORT_CHANNEL_IDX2,
</#if>
<#if CONFIG_DRV_SST25VF064C_WRITE_PROTECT_PIN_BIT_POS_IDX2?has_content>
    .writeProtectBitPosition = DRV_SST25VF064C_WRITE_PROTECT_PIN_BIT_POS_IDX2,
</#if>
<#if CONFIG_DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX2?has_content>
    .chipSelectPortChannel = DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX2,
</#if>
<#if CONFIG_DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX2?has_content>
    .chipSelectBitPosition = DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX2,
</#if>
<#if CONFIG_DRV_SST25VF064C_QUEUE_SIZE_IDX2?has_content>
    .queueSize = DRV_SST25VF064C_QUEUE_SIZE_IDX2,
</#if>
};
</#if>
</#if>
// </editor-fold>
<#--
/*******************************************************************************
 End of File
*/
-->
