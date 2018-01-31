config SYS_FS_VOL_INSTANCES_NUMBER_GT_${INSTANCE+1}
    bool
    depends on USE_SYS_FS
<#if INSTANCE != 0>
	default n if SYS_FS_VOL_INSTANCES_NUMBER_GT_${INSTANCE} = n     
</#if>	
	default n if SYS_FS_INSTANCES_NUMBER = ${INSTANCE+1}
	default y
	
config SYS_FS_VOL_IDX${INSTANCE}
    depends on USE_SYS_FS 
<#if INSTANCE != 0>
	             && SYS_FS_VOL_INSTANCES_NUMBER_GT_${INSTANCE}
</#if>
    bool "Volume ${INSTANCE}"
    default y
    ---help---
    IDH_HTML_SYSTEM_FS_Configuring_the_Library
    ---endhelp---
    
ifblock SYS_FS_VOL_IDX${INSTANCE}

menu "Volume Configuration (${INSTANCE})"
	depends on SYS_FS_AUTO_MOUNT
	
config SYS_FS_MEDIA_DEVICE_NAME_IDX${INSTANCE}    
    string "Device Name"
    depends on USE_SYS_FS
    default "/dev/mmcblka1"
    ---help---
    IDH_HTML_SYSTEM_FS_Configuring_the_Library
    ---endhelp---
    
config SYS_FS_MEDIA_MOUNT_NAME_IDX${INSTANCE}
    string "Media Mount Name"
    depends on USE_SYS_FS
    default "/mnt/myDrive"
    ---help---
    IDH_HTML_SYSTEM_FS_Configuring_the_Library
    ---endhelp---
    
endmenu 
endif 
