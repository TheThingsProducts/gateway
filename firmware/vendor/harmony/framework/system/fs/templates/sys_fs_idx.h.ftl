<#--
/*******************************************************************************
  File system Media Driver Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    sys_fs_idx.h.ftl

  Summary:
     File system Media Driver Freemarker Template File

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
<#macro SYS_FS_MEDIA_H INST_INDEX>

<#if CONFIG_DRV_USB_BETA_SW_HOST_SUPPORT == false>
#define SYS_FS_MEDIA_TYPE_IDX${INST_INDEX} 				${.vars["CONFIG_SYS_FS_MEDIA_TYPE_DEFINE_IDX${INST_INDEX}"]}
#define SYS_FS_TYPE_IDX${INST_INDEX} 					${.vars["CONFIG_SYS_FS_TYPE_DEFINE_IDX${INST_INDEX}"]}
</#if>





</#macro>
<#if CONFIG_SYS_FS_IDX0 == true>
	<@SYS_FS_MEDIA_H "0"/>
</#if>
<#if CONFIG_SYS_FS_IDX1 == true>
    <@SYS_FS_MEDIA_H "1"/>
</#if>
<#if CONFIG_SYS_FS_IDX2 == true>
    <@SYS_FS_MEDIA_H "2"/>  
</#if>
<#if CONFIG_SYS_FS_IDX3 == true>
    <@SYS_FS_MEDIA_H "3"/>      
</#if>    
<#if CONFIG_SYS_FS_IDX4 == true>
    <@SYS_FS_MEDIA_H "4"/>  
</#if>    
<#if CONFIG_SYS_FS_IDX5 == true>
    <@SYS_FS_MEDIA_H "5"/>  
</#if>    
<#if CONFIG_SYS_FS_IDX6 == true>
    <@SYS_FS_MEDIA_H "6"/>  
</#if>    
<#if CONFIG_SYS_FS_IDX7 == true>
    <@SYS_FS_MEDIA_H "7"/>  
</#if>    
<#if CONFIG_SYS_FS_IDX8 == true>
    <@SYS_FS_MEDIA_H "8"/>      
</#if>

<#if CONFIG_SYS_FS_IDX0 == true>

<#if CONFIG_SYS_FS_VOL_1_IDX0 == true>
#define SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX0 			"${.vars["CONFIG_SYS_FS_MEDIA_MOUNT_1_NAME_IDX0"]}"
#define SYS_FS_MEDIA_IDX0_DEVICE_NAME_VOLUME_IDX0 			"${.vars["CONFIG_SYS_FS_MEDIA_DEVICE_1_NAME_IDX0"]}"
</#if>
<#if CONFIG_SYS_FS_VOL_2_IDX0 == true>
#define SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX1 			"${.vars["CONFIG_SYS_FS_MEDIA_MOUNT_2_NAME_IDX0"]}"
#define SYS_FS_MEDIA_IDX0_DEVICE_NAME_VOLUME_IDX1 			"${.vars["CONFIG_SYS_FS_MEDIA_DEVICE_2_NAME_IDX0"]}"
</#if>
<#if CONFIG_SYS_FS_VOL_3_IDX0 == true>
#define SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX2 			"${.vars["CONFIG_SYS_FS_MEDIA_MOUNT_3_NAME_IDX0"]}"
#define SYS_FS_MEDIA_IDX0_DEVICE_NAME_VOLUME_IDX2 			"${.vars["CONFIG_SYS_FS_MEDIA_DEVICE_3_NAME_IDX0"]}"
</#if>
<#if CONFIG_SYS_FS_VOL_4_IDX0 == true>
#define SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX3 			"${.vars["CONFIG_SYS_FS_MEDIA_MOUNT_4_NAME_IDX0"]}"
#define SYS_FS_MEDIA_IDX0_DEVICE_NAME_VOLUME_IDX3 			"${.vars["CONFIG_SYS_FS_MEDIA_DEVICE_4_NAME_IDX0"]}"
</#if>
</#if>

<#if CONFIG_SYS_FS_IDX1 == true>
<#if CONFIG_SYS_FS_VOL_1_IDX1 == true>
#define SYS_FS_MEDIA_IDX1_MOUNT_NAME_VOLUME_IDX0 			"${.vars["CONFIG_SYS_FS_MEDIA_MOUNT_1_NAME_IDX1"]}"
#define SYS_FS_MEDIA_IDX1_DEVICE_NAME_VOLUME_IDX0 			"${.vars["CONFIG_SYS_FS_MEDIA_DEVICE_1_NAME_IDX1"]}"
</#if>
<#if CONFIG_SYS_FS_VOL_2_IDX1 == true>
#define SYS_FS_MEDIA_IDX1_MOUNT_NAME_VOLUME_IDX1 			"${.vars["CONFIG_SYS_FS_MEDIA_MOUNT_2_NAME_IDX1"]}"
#define SYS_FS_MEDIA_IDX1_DEVICE_NAME_VOLUME_IDX1 			"${.vars["CONFIG_SYS_FS_MEDIA_DEVICE_2_NAME_IDX1"]}"
</#if>
<#if CONFIG_SYS_FS_VOL_3_IDX1 == true>
#define SYS_FS_MEDIA_IDX1_MOUNT_NAME_VOLUME_IDX2 			"${.vars["CONFIG_SYS_FS_MEDIA_MOUNT_3_NAME_IDX1"]}"
#define SYS_FS_MEDIA_IDX1_DEVICE_NAME_VOLUME_IDX2 			"${.vars["CONFIG_SYS_FS_MEDIA_DEVICE_3_NAME_IDX1"]}"
</#if>
<#if CONFIG_SYS_FS_VOL_4_IDX1 == true>
#define SYS_FS_MEDIA_IDX1_MOUNT_NAME_VOLUME_IDX3 			"${.vars["CONFIG_SYS_FS_MEDIA_MOUNT_4_NAME_IDX1"]}"
#define SYS_FS_MEDIA_IDX1_DEVICE_NAME_VOLUME_IDX3 			"${.vars["CONFIG_SYS_FS_MEDIA_DEVICE_4_NAME_IDX1"]}"
</#if>
</#if>
<#if CONFIG_SYS_FS_IDX2 == true>
<#if CONFIG_SYS_FS_VOL_1_IDX2 == true>
#define SYS_FS_MEDIA_IDX2_MOUNT_NAME_VOLUME_IDX0 			"${.vars["CONFIG_SYS_FS_MEDIA_MOUNT_1_NAME_IDX2"]}"
#define SYS_FS_MEDIA_IDX2_DEVICE_NAME_VOLUME_IDX0 			"${.vars["CONFIG_SYS_FS_MEDIA_DEVICE_1_NAME_IDX2"]}"
</#if>
<#if CONFIG_SYS_FS_VOL_2_IDX2 == true>
#define SYS_FS_MEDIA_IDX2_MOUNT_NAME_VOLUME_IDX1 			"${.vars["CONFIG_SYS_FS_MEDIA_MOUNT_2_NAME_IDX2"]}"
#define SYS_FS_MEDIA_IDX2_DEVICE_NAME_VOLUME_IDX1 			"${.vars["CONFIG_SYS_FS_MEDIA_DEVICE_2_NAME_IDX2"]}"
</#if>
<#if CONFIG_SYS_FS_VOL_3_IDX2 == true>
#define SYS_FS_MEDIA_IDX2_MOUNT_NAME_VOLUME_IDX2 			"${.vars["CONFIG_SYS_FS_MEDIA_MOUNT_3_NAME_IDX2"]}"
#define SYS_FS_MEDIA_IDX2_DEVICE_NAME_VOLUME_IDX2 			"${.vars["CONFIG_SYS_FS_MEDIA_DEVICE_3_NAME_IDX2"]}"
</#if>
<#if CONFIG_SYS_FS_VOL_4_IDX2 == true>
#define SYS_FS_MEDIA_IDX2_MOUNT_NAME_VOLUME_IDX3 			"${.vars["CONFIG_SYS_FS_MEDIA_MOUNT_4_NAME_IDX2"]}"
#define SYS_FS_MEDIA_IDX2_DEVICE_NAME_VOLUME_IDX3 			"${.vars["CONFIG_SYS_FS_MEDIA_DEVICE_4_NAME_IDX2"]}"
</#if>
</#if>
<#if CONFIG_SYS_FS_IDX3 == true>
<#if CONFIG_SYS_FS_VOL_1_IDX2 == true>
#define SYS_FS_MEDIA_IDX3_MOUNT_NAME_VOLUME_IDX0 			"${.vars["CONFIG_SYS_FS_MEDIA_MOUNT_1_NAME_IDX3"]}"
#define SYS_FS_MEDIA_IDX3_DEVICE_NAME_VOLUME_IDX0 			"${.vars["CONFIG_SYS_FS_MEDIA_DEVICE_1_NAME_IDX3"]}"
</#if>
<#if CONFIG_SYS_FS_VOL_2_IDX2 == true>
#define SYS_FS_MEDIA_IDX3_MOUNT_NAME_VOLUME_IDX1 			"${.vars["CONFIG_SYS_FS_MEDIA_MOUNT_2_NAME_IDX3"]}"
#define SYS_FS_MEDIA_IDX3_DEVICE_NAME_VOLUME_IDX1 			"${.vars["CONFIG_SYS_FS_MEDIA_DEVICE_2_NAME_IDX3"]}"
</#if>
<#if CONFIG_SYS_FS_VOL_3_IDX2 == true>
#define SYS_FS_MEDIA_IDX3_MOUNT_NAME_VOLUME_IDX2 			"${.vars["CONFIG_SYS_FS_MEDIA_MOUNT_3_NAME_IDX3"]}"
#define SYS_FS_MEDIA_IDX3_DEVICE_NAME_VOLUME_IDX2 			"${.vars["CONFIG_SYS_FS_MEDIA_DEVICE_3_NAME_IDX3"]}"
</#if>
<#if CONFIG_SYS_FS_VOL_4_IDX2 == true>
#define SYS_FS_MEDIA_IDX3_MOUNT_NAME_VOLUME_IDX3 			"${.vars["CONFIG_SYS_FS_MEDIA_MOUNT_4_NAME_IDX3"]}"
#define SYS_FS_MEDIA_IDX3_DEVICE_NAME_VOLUME_IDX3 			"${.vars["CONFIG_SYS_FS_MEDIA_DEVICE_4_NAME_IDX3"]}"
</#if>
</#if>