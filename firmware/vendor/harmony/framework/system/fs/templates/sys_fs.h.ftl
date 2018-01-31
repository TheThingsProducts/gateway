<#--
/*******************************************************************************
  File System Service Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    sys_devcon.h

  Summary:
   File System Service Freemarker Template File

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

/*** File System Service Configuration ***/

<#if CONFIG_USE_SYS_FS == true>
#define SYS_FS_MEDIA_NUMBER         	${CONFIG_SYS_FS_INSTANCES_NUMBER}

<#if CONFIG_SYS_FS_AUTO_MOUNT != true>
#define SYS_FS_VOLUME_NUMBER		${CONFIG_SYS_FS_VOLUME_NUMBER}
<#elseif CONFIG_SYS_FS_IDX0 == true && CONFIG_SYS_FS_IDX1 == true && CONFIG_SYS_FS_IDX2 == true && CONFIG_SYS_FS_IDX3 == true>
#define SYS_FS_VOLUME_NUMBER		(${CONFIG_SYS_FS_VOLUME_INSTANCES_NUMBER_IDX0} + ${CONFIG_SYS_FS_VOLUME_INSTANCES_NUMBER_IDX1} + ${CONFIG_SYS_FS_VOLUME_INSTANCES_NUMBER_IDX2} + ${CONFIG_SYS_FS_VOLUME_INSTANCES_NUMBER_IDX3})
<#elseif CONFIG_SYS_FS_IDX0 == true && CONFIG_SYS_FS_IDX1 == true && CONFIG_SYS_FS_IDX2 == true>
#define SYS_FS_VOLUME_NUMBER		(${CONFIG_SYS_FS_VOLUME_INSTANCES_NUMBER_IDX0} + ${CONFIG_SYS_FS_VOLUME_INSTANCES_NUMBER_IDX1} + ${CONFIG_SYS_FS_VOLUME_INSTANCES_NUMBER_IDX2})
<#elseif CONFIG_SYS_FS_IDX0 == true && CONFIG_SYS_FS_IDX1 == true>
#define SYS_FS_VOLUME_NUMBER		(${CONFIG_SYS_FS_VOLUME_INSTANCES_NUMBER_IDX0} + ${CONFIG_SYS_FS_VOLUME_INSTANCES_NUMBER_IDX1})
<#elseif CONFIG_SYS_FS_IDX0 == true>
#define SYS_FS_VOLUME_NUMBER		(${CONFIG_SYS_FS_VOLUME_INSTANCES_NUMBER_IDX0})
</#if>

<#if CONFIG_SYS_FS_AUTO_MOUNT == true>
#define SYS_FS_AUTOMOUNT_ENABLE		true
<#else>
#define SYS_FS_AUTOMOUNT_ENABLE		false
</#if>
#define SYS_FS_MAX_FILES	    	${CONFIG_SYS_FS_MAX_FILES}
#define SYS_FS_MAX_FILE_SYSTEM_TYPE 	${CONFIG_SYS_FS_MAX_FILE_SYSTEM_TYPE}
<#if CONFIG_SYS_FS_MEDIA_MAX_BLOCK_SIZE?has_content>
#define SYS_FS_MEDIA_MAX_BLOCK_SIZE  	${CONFIG_SYS_FS_MEDIA_MAX_BLOCK_SIZE}
#define SYS_FS_MEDIA_MANAGER_BUFFER_SIZE ${CONFIG_SYS_FS_MEDIA_MANAGER_BUFFER_SIZE}
</#if>

<#include "/framework/system/fs/templates/sys_fs_idx.h.ftl">
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
