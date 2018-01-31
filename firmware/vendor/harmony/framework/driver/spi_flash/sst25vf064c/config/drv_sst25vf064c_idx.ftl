config DRV_SST25VF064C_INSTANCES_NUMBER_GT_${INSTANCE+1}
    depends on USE_DRV_SST25VF064C
    bool
<#if INSTANCE != 0>
	default n if DRV_SST25VF064C_INSTANCES_NUMBER_GT_${INSTANCE} = n     
</#if>	
	default n if DRV_SST25VF064C_INSTANCES_NUMBER = ${INSTANCE+1}
	default y
	
config DRV_SST25VF064C_INST_IDX${INSTANCE}
    depends on USE_DRV_SST25VF064C 
<#if INSTANCE != 0>
	             && DRV_SST25VF064C_INSTANCES_NUMBER_GT_${INSTANCE}
</#if>
    bool "SST25VF064C Driver Instance ${INSTANCE}"
    default y
    ---help---
    IDH_HTML_DRV_SST25VF064C_INSTANCES_NUMBER
    ---endhelp---

ifblock DRV_SST25VF064C_INST_IDX${INSTANCE}

config DRV_SST25VF064C_SPI_DRIVER_INSTANCE_IDX${INSTANCE}
    int "SPI Driver Instance Number"
    depends on USE_DRV_SST25VF064C
    range 0 SPI_NUMBER_OF_MODULES
    default 0
    ---help---
    IDH_HTML_SST25VF064C_MODULE_ID
    ---endhelp---

menu "RTOS Configuration (Instance ${INSTANCE})"
    depends on USE_DRV_SST25VF064C
    depends on USE_3RDPARTY_RTOS
    depends on DRV_SST25VF064C_DRIVER_MODE = "DYNAMIC"

config DRV_SST25VF064C_RTOS_IDX${INSTANCE}
    string "Run This Driver Instance As"
    depends on DRV_SST25VF064C_DRIVER_MODE = "DYNAMIC"
    range 3RDPARTY_RTOS_SYS_TASKS_OPTIONS
    default "Combined with System Tasks"

config DRV_SST25VF064C_IDX${INSTANCE}_RTOS_TASK_SIZE
    int "Task Size"
    depends on DRV_SST25VF064C_RTOS_IDX${INSTANCE} = "Standalone"
    default 1024

config DRV_SST25VF064C_IDX${INSTANCE}_RTOS_TASK_PRIORITY
    int "Task Priority"
    depends on DRV_SST25VF064C_RTOS_IDX${INSTANCE} = "Standalone"
    default 1

config DRV_SST25VF064C_IDX${INSTANCE}_RTOS_USE_DELAY
    bool "Use Task Delay?"
    depends on DRV_SST25VF064C_RTOS_IDX${INSTANCE} = "Standalone"
    default y

config DRV_SST25VF064C_IDX${INSTANCE}_RTOS_DELAY
    int "Task Delay"
    depends on DRV_SST25VF064C_RTOS_IDX${INSTANCE} = "Standalone"
    depends on DRV_SST25VF064C_IDX${INSTANCE}_RTOS_USE_DELAY
    default 1000
endmenu

config DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX${INSTANCE}
    string "Chip Select Port Channel"
    depends on USE_DRV_SST25VF064C
    range PORTS_CHANNEL
    default "PORT_CHANNEL_B"
    ---help---
    IDH_HTML_PORTS_CHANNEL
    ---endhelp---

config DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX${INSTANCE}
    string "Chip Select Port Bit Position"
    depends on USE_DRV_SST25VF064C
    range PORTS_BIT_POS
    default "PORTS_BIT_POS_0"
    ---help---
    IDH_HTML_PORTS_BIT_POS
    ---endhelp---

config DRV_SST25VF064C_HOLD_PIN_IDX${INSTANCE}
    depends on USE_DRV_SST25VF064C 
    bool "Use Hold Pin"
    default n
    ---help---
    IDH_HTML_SST25VF064C_Driver_Library
    ---endhelp---
    
config DRV_SST25VF064C_HOLD_PIN_PORT_CHANNEL_IDX${INSTANCE}
    string "Hold Pin Port Channel"
    depends on USE_DRV_SST25VF064C && DRV_SST25VF064C_HOLD_PIN_IDX${INSTANCE}
    range PORTS_CHANNEL
    default "PORT_CHANNEL_B"
    ---help---
    IDH_HTML_PORTS_CHANNEL
    ---endhelp---

config DRV_SST25VF064C_HOLD_PIN_PORT_BIT_POS_IDX${INSTANCE}
    string "Hold Pin Bit Position"
    depends on USE_DRV_SST25VF064C && DRV_SST25VF064C_HOLD_PIN_IDX${INSTANCE}
    range PORTS_BIT_POS
    default "PORTS_BIT_POS_1"
    ---help---
    IDH_HTML_PORTS_BIT_POS
    ---endhelp---

config DRV_SST25VF064C_WRITE_PROTECT_PIN_IDX${INSTANCE}
    depends on USE_DRV_SST25VF064C 
    bool "Use Write Protect Pin"
    default n
    ---help---
    IDH_HTML_SST25VF064C_Driver_Library
    ---endhelp---
    
config DRV_SST25VF064C_WRITE_PROTECT_PIN_PORT_CHANNEL_IDX${INSTANCE}
    string "Write Protect Pin Port Channel"
    depends on USE_DRV_SST25VF064C && DRV_SST25VF064C_WRITE_PROTECT_PIN_IDX${INSTANCE}
    range PORTS_CHANNEL
    default "PORT_CHANNEL_B"
    ---help---
    IDH_HTML_PORTS_CHANNEL
    ---endhelp---

config DRV_SST25VF064C_WRITE_PROTECT_PIN_BIT_POS_IDX${INSTANCE}
    string "Write Protect Pin Bit Position"
    depends on USE_DRV_SST25VF064C && DRV_SST25VF064C_WRITE_PROTECT_PIN_IDX${INSTANCE}
    range PORTS_BIT_POS
    default "PORTS_BIT_POS_2"
    ---help---
    IDH_HTML_PORTS_BIT_POS
    ---endhelp---
    
config DRV_SST25VF064C_QUEUE_SIZE_IDX${INSTANCE}
    int "Queue Size"
    depends on USE_DRV_SST25VF064C
    default 10
    ---help---
    IDH_HTML_DRV_SST25VF064C_INIT
    ---endhelp---

config DRV_SST25VF064C_POWER_STATE_IDX${INSTANCE}
    string "Power State"
    depends on USE_DRV_SST25VF064C
    range SYS_MODULE_POWER_STATE
    default "SYS_MODULE_POWER_RUN_FULL"
    ---help---
    IDH_HTML_SYS_MODULE_INIT
    ---endhelp---

endif


	
