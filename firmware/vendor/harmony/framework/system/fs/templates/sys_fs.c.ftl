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
// <editor-fold defaultstate="collapsed" desc="SYS_FS Initialization Data">
/*** File System Initialization Data ***/
<#if CONFIG_USE_SYS_FS == true> 
<#if CONFIG_3RDPARTY_RTOS_USED == "uC/OS-III">
<#if CONFIG_USE_SYS_FS == true>
OS_TCB  _SYS_FS_Tasks_TCB;
CPU_STK _SYS_FS_TasksStk[${CONFIG_SYS_FS_RTOS_TASK_SIZE}];

</#if>
</#if>

<#if CONFIG_SYS_FS_AUTO_MOUNT == true>

const SYS_FS_MEDIA_MOUNT_DATA sysfsMountTable[SYS_FS_VOLUME_NUMBER] = 
{
    
    <#if CONFIG_SYS_FS_IDX0 == true>
    <#if CONFIG_SYS_FS_VOL_1_IDX0 == true>
    {
		.mountName = SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX0,
		.devName   = SYS_FS_MEDIA_IDX0_DEVICE_NAME_VOLUME_IDX0, 
		.mediaType = SYS_FS_MEDIA_TYPE_IDX0,
		.fsType   = SYS_FS_TYPE_IDX0   
    },
    </#if>
    <#if CONFIG_SYS_FS_VOL_2_IDX0 == true>
    {
		.mountName = SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX1,
		.devName   = SYS_FS_MEDIA_IDX0_DEVICE_NAME_VOLUME_IDX1, 
		.mediaType = SYS_FS_MEDIA_TYPE_IDX0,
		.fsType   = SYS_FS_TYPE_IDX0   
    },    
    </#if>
    <#if CONFIG_SYS_FS_VOL_3_IDX0 == true>
    {
		.mountName = SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX2,
		.devName   = SYS_FS_MEDIA_IDX0_DEVICE_NAME_VOLUME_IDX2, 
		.mediaType = SYS_FS_MEDIA_TYPE_IDX0,
		.fsType   = SYS_FS_TYPE_IDX0   
    },    
    </#if>
    <#if CONFIG_SYS_FS_VOL_4_IDX0 == true>
    {
		.mountName = SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX3,
		.devName   = SYS_FS_MEDIA_IDX0_DEVICE_NAME_VOLUME_IDX3, 
		.mediaType = SYS_FS_MEDIA_TYPE_IDX0,
		.fsType   = SYS_FS_TYPE_IDX0   
    },    
    </#if>
    </#if>
    
    <#if CONFIG_SYS_FS_IDX1 == true>
    <#if CONFIG_SYS_FS_VOL_1_IDX1 == true>
    {
		.mountName = SYS_FS_MEDIA_IDX1_MOUNT_NAME_VOLUME_IDX0,
		.devName   = SYS_FS_MEDIA_IDX1_DEVICE_NAME_VOLUME_IDX0, 
		.mediaType = SYS_FS_MEDIA_TYPE_IDX1,
		.fsType   = SYS_FS_TYPE_IDX1   
    },
    </#if>
    <#if CONFIG_SYS_FS_VOL_2_IDX1 == true>
    {
		.mountName = SYS_FS_MEDIA_IDX1_MOUNT_NAME_VOLUME_IDX1,
		.devName   = SYS_FS_MEDIA_IDX1_DEVICE_NAME_VOLUME_IDX1, 
		.mediaType = SYS_FS_MEDIA_TYPE_IDX1,
		.fsType   = SYS_FS_TYPE_IDX1   
    },    
    </#if>
    <#if CONFIG_SYS_FS_VOL_3_IDX1 == true>
    {
		.mountName = SYS_FS_MEDIA_IDX1_MOUNT_NAME_VOLUME_IDX2,
		.devName   = SYS_FS_MEDIA_IDX1_DEVICE_NAME_VOLUME_IDX2, 
		.mediaType = SYS_FS_MEDIA_TYPE_IDX1,
		.fsType   = SYS_FS_TYPE_IDX1   
    },    
    </#if>
    <#if CONFIG_SYS_FS_VOL_4_IDX1 == true>
    {
		.mountName = SYS_FS_MEDIA_IDX1_MOUNT_NAME_VOLUME_IDX3,
		.devName   = SYS_FS_MEDIA_IDX1_DEVICE_NAME_VOLUME_IDX3, 
		.mediaType = SYS_FS_MEDIA_TYPE_IDX1,
		.fsType   = SYS_FS_TYPE_IDX1   
    },    
    </#if>
    </#if>    

    <#if CONFIG_SYS_FS_IDX2 == true>
    <#if CONFIG_SYS_FS_VOL_1_IDX2 == true>
    {
		.mountName = SYS_FS_MEDIA_IDX2_MOUNT_NAME_VOLUME_IDX0,
		.devName   = SYS_FS_MEDIA_IDX2_DEVICE_NAME_VOLUME_IDX0, 
		.mediaType = SYS_FS_MEDIA_TYPE_IDX2,
		.fsType   = SYS_FS_TYPE_IDX2   
    },
    </#if>
    <#if CONFIG_SYS_FS_VOL_2_IDX2 == true>
    {
		.mountName = SYS_FS_MEDIA_IDX2_MOUNT_NAME_VOLUME_IDX1,
		.devName   = SYS_FS_MEDIA_IDX2_DEVICE_NAME_VOLUME_IDX1, 
		.mediaType = SYS_FS_MEDIA_TYPE_IDX2,
		.fsType   = SYS_FS_TYPE_IDX2   
    },    
    </#if>
    <#if CONFIG_SYS_FS_VOL_3_IDX2 == true>
    {
		.mountName = SYS_FS_MEDIA_IDX2_MOUNT_NAME_VOLUME_IDX2,
		.devName   = SYS_FS_MEDIA_IDX2_DEVICE_NAME_VOLUME_IDX2, 
		.mediaType = SYS_FS_MEDIA_TYPE_IDX2,
		.fsType   = SYS_FS_TYPE_IDX2   
    },    
    </#if>
    <#if CONFIG_SYS_FS_VOL_4_IDX2 == true>
    {
		.mountName = SYS_FS_MEDIA_IDX2_MOUNT_NAME_VOLUME_IDX3,
		.devName   = SYS_FS_MEDIA_IDX2_DEVICE_NAME_VOLUME_IDX3, 
		.mediaType = SYS_FS_MEDIA_TYPE_IDX2,
		.fsType   = SYS_FS_TYPE_IDX2   
    },    
    </#if>
    </#if>      


    <#if CONFIG_SYS_FS_IDX3 == true>
    <#if CONFIG_SYS_FS_VOL_1_IDX3 == true>
    {
		.mountName = SYS_FS_MEDIA_IDX3_MOUNT_NAME_VOLUME_IDX0,
		.devName   = SYS_FS_MEDIA_IDX3_DEVICE_NAME_VOLUME_IDX0, 
		.mediaType = SYS_FS_MEDIA_TYPE_IDX3,
		.fsType   = SYS_FS_TYPE_IDX3   
    },
    </#if>
    <#if CONFIG_SYS_FS_VOL_2_IDX3 == true>
    {
		.mountName = SYS_FS_MEDIA_IDX3_MOUNT_NAME_VOLUME_IDX1,
		.devName   = SYS_FS_MEDIA_IDX3_DEVICE_NAME_VOLUME_IDX1, 
		.mediaType = SYS_FS_MEDIA_TYPE_IDX3,
		.fsType   = SYS_FS_TYPE_IDX3   
    },    
    </#if>
    <#if CONFIG_SYS_FS_VOL_3_IDX3 == true>
    {
		.mountName = SYS_FS_MEDIA_IDX3_MOUNT_NAME_VOLUME_IDX2,
		.devName   = SYS_FS_MEDIA_IDX3_DEVICE_NAME_VOLUME_IDX2, 
		.mediaType = SYS_FS_MEDIA_TYPE_IDX3,
		.fsType   = SYS_FS_TYPE_IDX3   
    },    
    </#if>
    <#if CONFIG_SYS_FS_VOL_4_IDX3 == true>
    {
		.mountName = SYS_FS_MEDIA_IDX3_MOUNT_NAME_VOLUME_IDX3,
		.devName   = SYS_FS_MEDIA_IDX3_DEVICE_NAME_VOLUME_IDX3, 
		.mediaType = SYS_FS_MEDIA_TYPE_IDX3,
		.fsType   = SYS_FS_TYPE_IDX3   
    },    
    </#if>
    </#if>     
     
};
</#if>
<#if CONFIG_SYS_FS_AUTO_MOUNT != true>
const SYS_FS_MEDIA_MOUNT_DATA sysfsMountTable[SYS_FS_VOLUME_NUMBER] = 
{
	{NULL}
};
</#if>


<#if CONFIG_SYS_FS_FAT == true> 
<#if CONFIG_SYS_FS_MPFS == true> 
const SYS_FS_REGISTRATION_TABLE sysFSInit [ SYS_FS_MAX_FILE_SYSTEM_TYPE ] =
{
    {
        .nativeFileSystemType = FAT,
        .nativeFileSystemFunctions = &FatFsFunctions
    },
    {
        .nativeFileSystemType = MPFS2,
        .nativeFileSystemFunctions = &MPFSFunctions
    }

};

</#if>
</#if>
</#if>

<#if CONFIG_USE_SYS_FS == true> 
<#if CONFIG_SYS_FS_MPFS == true> 
<#if CONFIG_SYS_FS_FAT == false> 
const SYS_FS_REGISTRATION_TABLE sysFSInit [ SYS_FS_MAX_FILE_SYSTEM_TYPE ] =
{
    {
        .nativeFileSystemType = MPFS2,
        .nativeFileSystemFunctions = &MPFSFunctions
    }
};
</#if>
</#if>
</#if>

<#if CONFIG_USE_SYS_FS == true> 
<#if CONFIG_SYS_FS_FAT == true> 
<#if CONFIG_SYS_FS_MPFS == false> 
const SYS_FS_REGISTRATION_TABLE sysFSInit [ SYS_FS_MAX_FILE_SYSTEM_TYPE ] =
{
    {
        .nativeFileSystemType = FAT,
        .nativeFileSystemFunctions = &FatFsFunctions
    }
};
</#if>
</#if>
</#if>
// </editor-fold>
<#--
/*******************************************************************************
 End of File
*/
-->
