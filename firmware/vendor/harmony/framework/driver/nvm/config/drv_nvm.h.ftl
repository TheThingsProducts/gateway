<#--
/*******************************************************************************
  NVM Driver Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_nvm.h.ftl

  Summary:
    NVM Driver Freemarker Template File

  Description:

*******************************************************************************/

/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
-->

/*** NVM Driver Configuration ***/
<#if CONFIG_USE_DRV_NVM == true>
<#if CONFIG_DRV_NVM_DRIVER_MODE == "DYNAMIC">
<#if CONFIG_DRV_NVM_DRIVER_TYPE == "BETA">
<#if CONFIG_USE_DRV_NVM_MEDIA == true>
<#-- NVM Media Driver Defines -->
#define DRV_NVM_MEDIA_OBJECT_NUMBER   	${CONFIG_DRV_NVM_MEDIA_OBJECT_NUMBER}
#define NVM_MEDIA_SECTOR_SIZE           ${CONFIG_DRV_NVM_MEDIA_SECTOR_SIZE}
#define NVM_MEDIA_START_ADDRESS         ${CONFIG_DRV_NVM_MEDIA_START_ADDRESS}
#define NVM_MEDIA_SIZE                  ${CONFIG_DRV_NVM_MEDIA_SIZE}
</#if>
</#if>

<#-- NVM Driver Defines -->
#define DRV_NVM_INSTANCES_NUMBER     	${CONFIG_DRV_NVM_INSTANCES_NUMBER}
#define DRV_NVM_CLIENTS_NUMBER        	${CONFIG_DRV_NVM_CLIENTS_NUMBER}
#define DRV_NVM_BUFFER_OBJECT_NUMBER  	${CONFIG_DRV_NVM_BUFFER_OBJECT_NUMBER}

<#if CONFIG_DRV_NVM_INTERRUPT_MODE == true>
#define DRV_NVM_INTERRUPT_MODE        	true
#define DRV_NVM_INTERRUPT_SOURCE      	${CONFIG_DRV_NVM_INTERRUPT_SOURCE}
<#else>
#define DRV_NVM_INTERRUPT_MODE        	false
</#if>

<#if CONFIG_DRV_NVM_DRIVER_TYPE == "BLOCK_DRIVER">
#define DRV_NVM_MEDIA_SIZE              ${CONFIG_DRV_NVM_MEDIA_SIZE}
#define DRV_NVM_MEDIA_START_ADDRESS     ${CONFIG_DRV_NVM_MEDIA_START_ADDRESS}
</#if>

<#if CONFIG_USE_DRV_NVM_ERASE_WRITE == true>
#define DRV_NVM_ERASE_WRITE_ENABLE
</#if>

<#if CONFIG_DRV_NVM_DRIVER_TYPE == "BLOCK_DRIVER">

<#if CONFIG_USE_DRV_NVM_SYS_FS_REGISTER == true>
#define DRV_NVM_SYS_FS_REGISTER
</#if>

<#if CONFIG_USE_DRV_NVM_DISABLE_ERROR_CHECK == true>
#define DRV_NVM_DISABLE_ERROR_CHECK
</#if>

</#if>
</#if>
<#if CONFIG_DRV_NVM_DRIVER_MODE == "STATIC">
<#if CONFIG_DRV_NVM_INTERRUPT_MODE == true>
#define DRV_NVM_INTERRUPT_MODE          true
<#else>
#define DRV_NVM_INTERRUPT_MODE          false
</#if>
#define DRV_NVM_BUFFER_OBJECT_NUMBER    ${CONFIG_DRV_NVM_BUFFER_OBJECT_NUMBER}
#define DRV_NVM_MEDIA_SIZE              ${CONFIG_DRV_NVM_MEDIA_SIZE}
#define DRV_NVM_MEDIA_START_ADDRESS     ${CONFIG_DRV_NVM_MEDIA_START_ADDRESS}
<#if CONFIG_USE_DRV_NVM_ERASE_WRITE == true>
#define DRV_NVM_ERASE_WRITE_ENABLE
</#if>

<#if CONFIG_USE_DRV_NVM_SYS_FS_REGISTER == true>
#define DRV_NVM_SYS_FS_REGISTER
</#if>

<#if CONFIG_USE_DRV_NVM_DISABLE_ERROR_CHECK == true>
#define DRV_NVM_DISABLE_ERROR_CHECK
</#if>
</#if>
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->

