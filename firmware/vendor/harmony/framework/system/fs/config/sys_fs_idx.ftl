config SYS_FS_INSTANCES_NUMBER_GT_${INSTANCE+1}
    bool
    depends on USE_SYS_FS
<#if INSTANCE != 0>
	default n if SYS_FS_INSTANCES_NUMBER_GT_${INSTANCE} = n
</#if>
	default n if SYS_FS_INSTANCES_NUMBER = ${INSTANCE+1}
	default y

config SYS_FS_IDX${INSTANCE}
    depends on USE_SYS_FS
<#if INSTANCE != 0>
	             && SYS_FS_INSTANCES_NUMBER_GT_${INSTANCE}
</#if>
    bool "Media ${INSTANCE}"
    default y
    ---help---
    IDH_HTML_DRV_TMR_INSTANCES_NUMBER
    ---endhelp---

ifblock SYS_FS_IDX${INSTANCE}
menu "Media Configuration (${INSTANCE})"
	depends on SYS_FS_AUTO_MOUNT

config SYS_FS_MEDIA_TYPE_DEFINE_IDX${INSTANCE}
    string "Media Type"
    depends on USE_SYS_FS
    range SYS_FS_MEDIA_TYPE
    default "SYS_FS_MEDIA_TYPE_SD_CARD"
    ---help---
    IDH_HTML_SYSTEM_FS_Configuring_the_Library
    ---endhelp---

config SYS_FS_TYPE_DEFINE_IDX${INSTANCE}
    string "File system Type"
    depends on USE_SYS_FS
    range SYS_FS_FILE_SYSTEM_TYPE
    default "FAT"
    ---help---
    IDH_HTML_SYSTEM_FS_Configuring_the_Library
    ---endhelp---

config SYS_FS_USE_NVM_MBR${INSTANCE}
    bool "Create FAT12 in NVM"
    depends on USE_SYS_FS
    depends on SYS_FS_MEDIA_TYPE_DEFINE_IDX${INSTANCE} = "SYS_FS_MEDIA_TYPE_NVM" && SYS_FS_TYPE_DEFINE_IDX${INSTANCE} = "FAT"
    set SYS_FS_USE_MBR to y if SYS_FS_USE_NVM_MBR${INSTANCE} = y
    default n

config SYS_FS_VOLUME_INSTANCES_NUMBER_IDX${INSTANCE}
    int "Number Of Volumes"
    depends on USE_SYS_FS
    default 1
    ---help---
    IDH_HTML_SYSTEM_FS_Configuring_the_Library
    ---endhelp---

config SYS_FS_VOL_1_INSTANCES_NUMBER_GT_${INSTANCE}
    bool
    depends on USE_SYS_FS
    default n if SYS_FS_VOLUME_INSTANCES_NUMBER_IDX${INSTANCE} = 0
    default y

config SYS_FS_VOL_1_IDX${INSTANCE}
    depends on USE_SYS_FS
    depends on SYS_FS_VOL_1_INSTANCES_NUMBER_GT_${INSTANCE}
    bool "Volume 1"
    default y
    ---help---
    IDH_HTML_SYSTEM_FS_Configuring_the_Library
    ---endhelp---

config SYS_FS_MEDIA_DEVICE_1_NAME_IDX${INSTANCE}
    string "Device Name"
    depends on USE_SYS_FS
    depends on SYS_FS_VOL_1_IDX${INSTANCE}
    default "/dev/mmcblka1" if SYS_FS_MEDIA_TYPE_DEFINE_IDX${INSTANCE} = "SYS_FS_MEDIA_TYPE_SD_CARD"
    default "/dev/nvma1" if SYS_FS_MEDIA_TYPE_DEFINE_IDX${INSTANCE} = "SYS_FS_MEDIA_TYPE_NVM"
    default "/dev/sda1" if SYS_FS_MEDIA_TYPE_DEFINE_IDX${INSTANCE} = "SYS_FS_MEDIA_TYPE_MSD"
    default "/dev/mtda1" if SYS_FS_MEDIA_TYPE_DEFINE_IDX${INSTANCE} = "SYS_FS_MEDIA_TYPE_SPIFLASH"
    ---help---
    IDH_HTML_SYSTEM_FS_Configuring_the_Library
    ---endhelp---

config SYS_FS_MEDIA_MOUNT_1_NAME_IDX${INSTANCE}
    string "Media Mount Name"
    depends on USE_SYS_FS
    depends on SYS_FS_VOL_1_IDX${INSTANCE}
    default "/mnt/myDrive${INSTANCE+1}"
    ---help---
    IDH_HTML_SYSTEM_FS_Configuring_the_Library
    ---endhelp---

config SYS_FS_VOL_2_INSTANCES_NUMBER_GT_${INSTANCE}
    bool
    depends on USE_SYS_FS
    default n if SYS_FS_VOLUME_INSTANCES_NUMBER_IDX${INSTANCE} = 1
    default n if SYS_FS_VOL_1_INSTANCES_NUMBER_GT_${INSTANCE} = n
    default y

config SYS_FS_VOL_2_IDX${INSTANCE}
    depends on USE_SYS_FS
    depends on SYS_FS_VOL_2_INSTANCES_NUMBER_GT_${INSTANCE}
    bool "Volume 2"
    default y
    ---help---
    IDH_HTML_SYSTEM_FS_Configuring_the_Library
    ---endhelp---

config SYS_FS_MEDIA_DEVICE_2_NAME_IDX${INSTANCE}
    string "Device Name"
    depends on USE_SYS_FS
    depends on SYS_FS_VOL_2_IDX${INSTANCE}
    default "/dev/mmcblka2" if SYS_FS_MEDIA_TYPE_DEFINE_IDX${INSTANCE} = "SYS_FS_MEDIA_TYPE_SD_CARD"
    default "/dev/nvma2" if SYS_FS_MEDIA_TYPE_DEFINE_IDX${INSTANCE} = "SYS_FS_MEDIA_TYPE_NVM"
    default "/dev/sda2" if SYS_FS_MEDIA_TYPE_DEFINE_IDX${INSTANCE} = "SYS_FS_MEDIA_TYPE_MSD"
    default "/dev/mtda2" if SYS_FS_MEDIA_TYPE_DEFINE_IDX${INSTANCE} = "SYS_FS_MEDIA_TYPE_SPIFLASH"
    ---help---
    IDH_HTML_SYSTEM_FS_Configuring_the_Library
    ---endhelp---

config SYS_FS_MEDIA_MOUNT_2_NAME_IDX${INSTANCE}
    string "Media Mount Name"
    depends on USE_SYS_FS
    depends on SYS_FS_VOL_2_IDX${INSTANCE}
    default "/mnt/myDrive${INSTANCE+2}"
    ---help---
    IDH_HTML_SYSTEM_FS_Configuring_the_Library
    ---endhelp---

config SYS_FS_VOL_3_INSTANCES_NUMBER_GT_${INSTANCE}
    bool
    depends on USE_SYS_FS
    default n if SYS_FS_VOLUME_INSTANCES_NUMBER_IDX${INSTANCE} = 2
    default n if SYS_FS_VOL_2_INSTANCES_NUMBER_GT_${INSTANCE} = n
    default y

config SYS_FS_VOL_3_IDX${INSTANCE}
    depends on USE_SYS_FS
    depends on SYS_FS_VOL_3_INSTANCES_NUMBER_GT_${INSTANCE}
    bool "Volume 3"
    default y
    ---help---
    IDH_HTML_SYSTEM_FS_Configuring_the_Library
    ---endhelp---

config SYS_FS_MEDIA_DEVICE_3_NAME_IDX${INSTANCE}
    string "Device Name"
    depends on USE_SYS_FS
    depends on SYS_FS_VOL_3_IDX${INSTANCE}
    default "/dev/mmcblka3" if SYS_FS_MEDIA_TYPE_DEFINE_IDX${INSTANCE} = "SYS_FS_MEDIA_TYPE_SD_CARD"
    default "/dev/nvma3" if SYS_FS_MEDIA_TYPE_DEFINE_IDX${INSTANCE} = "SYS_FS_MEDIA_TYPE_NVM"
    default "/dev/sda3" if SYS_FS_MEDIA_TYPE_DEFINE_IDX${INSTANCE} = "SYS_FS_MEDIA_TYPE_MSD"
    default "/dev/mtda3" if SYS_FS_MEDIA_TYPE_DEFINE_IDX${INSTANCE} = "SYS_FS_MEDIA_TYPE_SPIFLASH"
    ---help---
    IDH_HTML_SYSTEM_FS_Configuring_the_Library
    ---endhelp---

config SYS_FS_MEDIA_MOUNT_3_NAME_IDX${INSTANCE}
    string "Media Mount Name"
    depends on USE_SYS_FS
    depends on SYS_FS_VOL_3_IDX${INSTANCE}
    default "/mnt/myDrive${INSTANCE+3}"
    ---help---
    IDH_HTML_SYSTEM_FS_Configuring_the_Library
    ---endhelp---

config SYS_FS_VOL_4_INSTANCES_NUMBER_GT_${INSTANCE}
    bool
    depends on USE_SYS_FS
    default n if SYS_FS_VOLUME_INSTANCES_NUMBER_IDX${INSTANCE} = 3
    default n if SYS_FS_VOL_3_INSTANCES_NUMBER_GT_${INSTANCE} = n
    default y

config SYS_FS_VOL_4_IDX${INSTANCE}
    depends on USE_SYS_FS
    depends on SYS_FS_VOL_4_INSTANCES_NUMBER_GT_${INSTANCE}
    bool "Volume 4"
    default y
    ---help---
    IDH_HTML_SYSTEM_FS_Configuring_the_Library
    ---endhelp---

config SYS_FS_MEDIA_DEVICE_4_NAME_IDX${INSTANCE}
    string "Device Name"
    depends on USE_SYS_FS
    depends on SYS_FS_VOL_4_IDX${INSTANCE}
    default "/dev/mmcblka4" if SYS_FS_MEDIA_TYPE_DEFINE_IDX${INSTANCE} = "SYS_FS_MEDIA_TYPE_SD_CARD"
    default "/dev/nvma4" if SYS_FS_MEDIA_TYPE_DEFINE_IDX${INSTANCE} = "SYS_FS_MEDIA_TYPE_NVM"
    default "/dev/sda4" if SYS_FS_MEDIA_TYPE_DEFINE_IDX${INSTANCE} = "SYS_FS_MEDIA_TYPE_MSD"
    default "/dev/mtda4" if SYS_FS_MEDIA_TYPE_DEFINE_IDX${INSTANCE} = "SYS_FS_MEDIA_TYPE_SPIFLASH"
    ---help---
    IDH_HTML_SYSTEM_FS_Configuring_the_Library
    ---endhelp---

config SYS_FS_MEDIA_MOUNT_4_NAME_IDX${INSTANCE}
    string "Media Mount Name"
    depends on USE_SYS_FS
    depends on SYS_FS_VOL_4_IDX${INSTANCE}
    default "/mnt/myDrive${INSTANCE+4}"
    ---help---
    IDH_HTML_SYSTEM_FS_Configuring_the_Library
    ---endhelp---


endmenu
endif
