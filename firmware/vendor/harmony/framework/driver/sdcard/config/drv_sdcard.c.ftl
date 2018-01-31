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
// <editor-fold defaultstate="collapsed" desc="DRV_SDCARD Initialization Data">
/*** SDCARD Driver Initialization Data ***/
<#if CONFIG_USE_DRV_SDCARD == true>

const DRV_SDCARD_INIT drvSDCardInit =
{
<#if CONFIG_DRV_SDCARD_SPI_DRV_INSTANCE == "0">
    .spiId = ${CONFIG_DRV_SPI_SPI_ID_IDX0},
</#if>
<#if CONFIG_DRV_SDCARD_SPI_DRV_INSTANCE == "1">
    .spiId = ${CONFIG_DRV_SPI_SPI_ID_IDX1},
</#if>
<#if CONFIG_DRV_SDCARD_SPI_DRV_INSTANCE == "2">
    .spiId = ${CONFIG_DRV_SPI_SPI_ID_IDX2},
</#if>
<#if CONFIG_DRV_SDCARD_SPI_DRV_INSTANCE == "3">
    .spiId = ${CONFIG_DRV_SPI_SPI_ID_IDX3},
</#if>
<#if CONFIG_DRV_SDCARD_SPI_DRV_INSTANCE == "4">
    .spiId = ${CONFIG_DRV_SPI_SPI_ID_IDX4},
</#if>
<#if CONFIG_DRV_SDCARD_SPI_DRV_INSTANCE == "5">
    .spiId = ${CONFIG_DRV_SPI_SPI_ID_IDX5},
</#if>
<#if CONFIG_DRV_SDCARD_SPI_DRV_INSTANCE?has_content>
    .spiIndex = ${CONFIG_DRV_SDCARD_SPI_DRV_INSTANCE},
</#if>
<#if CONFIG_DRV_SDCARD_SPEED?has_content>
    .sdcardSpeedHz = ${CONFIG_DRV_SDCARD_SPEED},
</#if>
<#if CONFIG_DRV_SDCARD_SPI_CLOCK_ID?has_content>
    .spiClk = ${CONFIG_DRV_SDCARD_SPI_CLOCK_ID},
</#if>
<#if CONFIG_DRV_SDCARD_WP_PORT_CHANNEL?has_content>
    .writeProtectPort = ${CONFIG_DRV_SDCARD_WP_PORT_CHANNEL},
    .writeProtectBitPosition = ${CONFIG_DRV_SDCARD_WP_BIT_POSITION},
</#if>
<#if CONFIG_DRV_SDCARD_CS_PORT_CHANNEL?has_content>
    .chipSelectPort = ${CONFIG_DRV_SDCARD_CS_PORT_CHANNEL},
    .chipSelectBitPosition = ${CONFIG_DRV_SDCARD_CS_BIT_POSITION},
</#if>
};
</#if>
// </editor-fold>
<#--
/*******************************************************************************
 End of File
*/
-->
