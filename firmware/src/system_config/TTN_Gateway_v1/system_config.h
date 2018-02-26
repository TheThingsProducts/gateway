/*******************************************************************************
  MPLAB Harmony System Configuration Header

  File Name:
    system_config.h

  Summary:
    Build-time configuration header for the system defined by this MPLAB Harmony
    project.

  Description:
    An MPLAB Project may have multiple configurations.  This file defines the
    build-time options for a single configuration.

  Remarks:
    This configuration header must not define any prototypes or data
    definitions (or include any files that do).  It only provides macro
    definitions for build-time configuration options that are not instantiated
    until used by another MPLAB Harmony module or application.

    Created with MPLAB Harmony Version 1.08.01
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

#ifndef _SYSTEM_CONFIG_H
#define _SYSTEM_CONFIG_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/*  This section Includes other configuration headers necessary to completely
    define this configuration.
*/

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: System Service Configuration
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Common System Service Configuration Options
*/
#define SYS_VERSION_STR           "1.08.01"
#define SYS_VERSION               10801

    
// *****************************************************************************
/* Task size and priorities
 */
#define TASK_SIZE_SYS_TASKS         4096
#define TASK_SIZE_TCPIP_TASKS       4096
#define TASK_SIZE_APP_TASKS         4096
#define TASK_SIZE_LORA_TASKS        4096
//#define TASK_SIZE_LORA_READ         4096  // Defined in app_lora.c, line 1005 (by configMINIMAL_STACK_SIZE)
//#define TASK_SIZE_WIFI_DRV          2048  // Defined in Harmony configuration (CONFIG_DRV_WIFIN_RTOS_MAIN_TASK_SIZE)
//#define TASK_SIZE_MQTT_TASKS        4096  // Hardcoded in MQTTHarmony.c, line 182

#define TASK_PRIORITY_SYS_TASKS     4
#define TASK_PRIORITY_TCPIP_TASKS   6
#define TASK_PRIORITY_APP_TASKS     2
#define TASK_PRIORITY_LORA_TASKS    3
#define TASK_PRIORITY_LORA_READ     7
//#define TASK_PRIORITY_WIFI_DRV      5     // Defined in Harmony configuration (CONFIG_DRV_WIFIN_RTOS_MAIN_TASK_PRIORITY)
//#define TASK_PRIORITY_MQTT_TASKS    1     // Hardcoded in MQTTHarmony.c, line 183
    
// *****************************************************************************
/* Clock System Service Configuration Options
*/
#define SYS_CLK_FREQ                        200000000ul
#define SYS_CLK_BUS_PERIPHERAL_1            100000000ul
#define SYS_CLK_BUS_PERIPHERAL_2            100000000ul
#define SYS_CLK_BUS_PERIPHERAL_3            100000000ul
#define SYS_CLK_BUS_PERIPHERAL_4            100000000ul
#define SYS_CLK_BUS_PERIPHERAL_5            100000000ul
#define SYS_CLK_BUS_PERIPHERAL_7            200000000ul
#define SYS_CLK_BUS_PERIPHERAL_8            100000000ul
#define SYS_CLK_CONFIG_PRIMARY_XTAL         24000000ul
#define SYS_CLK_CONFIG_SECONDARY_XTAL       32768ul
   
/*** Interrupt System Service Configuration ***/
#define SYS_INT                     true

/*** Ports System Service Configuration ***/

#define SYS_PORT_A_ANSEL        0x620
#define SYS_PORT_A_TRIS         0xc63f
#define SYS_PORT_A_LAT          0xc0
#define SYS_PORT_A_ODC          0x0
#define SYS_PORT_A_CNPU         0x0
#define SYS_PORT_A_CNPD         0x0
#define SYS_PORT_A_CNEN         0x0

#define SYS_PORT_B_ANSEL        0x8ff
#define SYS_PORT_B_TRIS         0xffff
#define SYS_PORT_B_LAT          0x0
#define SYS_PORT_B_ODC          0x0
#define SYS_PORT_B_CNPU         0xc000
#define SYS_PORT_B_CNPD         0x0
#define SYS_PORT_B_CNEN         0x0

#define SYS_PORT_C_ANSEL        0xe014
#define SYS_PORT_C_TRIS         0xf01e
#define SYS_PORT_C_LAT          0x0
#define SYS_PORT_C_ODC          0x0
#define SYS_PORT_C_CNPU         0x0
#define SYS_PORT_C_CNPD         0x0
#define SYS_PORT_C_CNEN         0x0

#define SYS_PORT_D_ANSEL        0x0
#define SYS_PORT_D_TRIS         0x8eff
#define SYS_PORT_D_LAT          0x3000
#define SYS_PORT_D_ODC          0x0
#define SYS_PORT_D_CNPU         0x0
#define SYS_PORT_D_CNPD         0x0
#define SYS_PORT_D_CNEN         0x0

#define SYS_PORT_E_ANSEL        0x3c0
#define SYS_PORT_E_TRIS         0x3f7
#define SYS_PORT_E_LAT          0x8
#define SYS_PORT_E_ODC          0x0
#define SYS_PORT_E_CNPU         0x0
#define SYS_PORT_E_CNPD         0x10
#define SYS_PORT_E_CNEN         0x0

#define SYS_PORT_F_ANSEL        0x0
#define SYS_PORT_F_TRIS         0x313c
#define SYS_PORT_F_LAT          0x3
#define SYS_PORT_F_ODC          0x0
#define SYS_PORT_F_CNPU         0x0
#define SYS_PORT_F_CNPD         0x0
#define SYS_PORT_F_CNEN         0x2000

#define SYS_PORT_G_ANSEL        0x8200
#define SYS_PORT_G_TRIS         0xf3c3
#define SYS_PORT_G_LAT          0x0
#define SYS_PORT_G_ODC          0x0
#define SYS_PORT_G_CNPU         0x180
#define SYS_PORT_G_CNPD         0x0
#define SYS_PORT_G_CNEN         0x0

#define SYS_PORT_H_ANSEL        0x0
#define SYS_PORT_H_TRIS         0xff7f
#define SYS_PORT_H_LAT          0x0
#define SYS_PORT_H_ODC          0x0
#define SYS_PORT_H_CNPU         0x0
#define SYS_PORT_H_CNPD         0x0
#define SYS_PORT_H_CNEN         0x0

#define SYS_PORT_J_ANSEL        0x0
#define SYS_PORT_J_TRIS         0xff9f
#define SYS_PORT_J_LAT          0x0
#define SYS_PORT_J_ODC          0x0
#define SYS_PORT_J_CNPU         0x0
#define SYS_PORT_J_CNPD         0x0
#define SYS_PORT_J_CNEN         0x0

#define SYS_PORT_K_ANSEL        0x0
#define SYS_PORT_K_TRIS         0x7f
#define SYS_PORT_K_LAT          0x80
#define SYS_PORT_K_ODC          0x0
#define SYS_PORT_K_CNPU         0x0
#define SYS_PORT_K_CNPD         0x0
#define SYS_PORT_K_CNEN         0x0
/*** Timer System Service Configuration ***/
#define SYS_TMR_POWER_STATE             SYS_MODULE_POWER_RUN_FULL
#define SYS_TMR_DRIVER_INDEX            DRV_TMR_INDEX_0
#define SYS_TMR_MAX_CLIENT_OBJECTS      5
#define SYS_TMR_FREQUENCY               1000
#define SYS_TMR_FREQUENCY_TOLERANCE     10
#define SYS_TMR_UNIT_RESOLUTION         10000
#define SYS_TMR_CLIENT_TOLERANCE        10
#define SYS_TMR_INTERRUPT_NOTIFICATION  true

/*** Console System Service Configuration ***/

#define SYS_CONSOLE_OVERRIDE_STDIO
#define SYS_CONSOLE_DEVICE_MAX_INSTANCES        2
#define SYS_CONSOLE_INSTANCES_NUMBER            1
#define SYS_CONSOLE_UART_IDX               DRV_USART_INDEX_0
#define SYS_CONSOLE_UART_RD_QUEUE_DEPTH    1
#define SYS_CONSOLE_UART_WR_QUEUE_DEPTH    64
#define SYS_CONSOLE_BUFFER_DMA_READY



/*** Debug System Service Configuration ***/
#define SYS_DEBUG_ENABLE
#define DEBUG_PRINT_BUFFER_SIZE       1024
#define SYS_DEBUG_BUFFER_DMA_READY
#define SYS_DEBUG_USE_CONSOLE

/*** Command Processor System Service Configuration ***/
#define SYS_CMD_ENABLE
#define SYS_CMD_DEVICE_MAX_INSTANCES    SYS_CONSOLE_DEVICE_MAX_INSTANCES
#define SYS_CMD_PRINT_BUFFER_SIZE       1024
#define SYS_CMD_BUFFER_DMA_READY        __attribute__((coherent)) __attribute__((aligned(16)))

/*** File System Service Configuration ***/

#define SYS_FS_MEDIA_NUMBER         	2

#define SYS_FS_VOLUME_NUMBER		6

#define SYS_FS_AUTOMOUNT_ENABLE		false
#define SYS_FS_MAX_FILES	    	25
#define SYS_FS_MAX_FILE_SYSTEM_TYPE 	2
#define SYS_FS_MEDIA_MAX_BLOCK_SIZE  	512
#define SYS_FS_MEDIA_MANAGER_BUFFER_SIZE 2048


#define SYS_FS_MEDIA_TYPE_IDX0 				
#define SYS_FS_TYPE_IDX0 					






#define SYS_FS_MEDIA_TYPE_IDX1 				
#define SYS_FS_TYPE_IDX1 					








// *****************************************************************************
/* Random System Service Configuration Options
*/

#define SYS_RANDOM_CRYPTO_SEED_SIZE  55


// *****************************************************************************
// *****************************************************************************
// Section: Driver Configuration
// *****************************************************************************
// *****************************************************************************
/*** Timer Driver Configuration ***/
#define DRV_TMR_INTERRUPT_MODE             true
#define DRV_TMR_INSTANCES_NUMBER           2
#define DRV_TMR_CLIENTS_NUMBER             1

/*** Timer Driver 0 Configuration ***/
#define DRV_TMR_PERIPHERAL_ID_IDX0          TMR_ID_2
#define DRV_TMR_INTERRUPT_SOURCE_IDX0       INT_SOURCE_TIMER_2
#define DRV_TMR_INTERRUPT_VECTOR_IDX0       INT_VECTOR_T2
#define DRV_TMR_ISR_VECTOR_IDX0             _TIMER_2_VECTOR
#define DRV_TMR_INTERRUPT_PRIORITY_IDX0     INT_PRIORITY_LEVEL4
#define DRV_TMR_INTERRUPT_SUB_PRIORITY_IDX0 INT_SUBPRIORITY_LEVEL0
#define DRV_TMR_CLOCK_SOURCE_IDX0           DRV_TMR_CLKSOURCE_INTERNAL
#define DRV_TMR_PRESCALE_IDX0               TMR_PRESCALE_VALUE_256
#define DRV_TMR_OPERATION_MODE_IDX0         DRV_TMR_OPERATION_MODE_16_BIT
#define DRV_TMR_ASYNC_WRITE_ENABLE_IDX0     false
#define DRV_TMR_POWER_STATE_IDX0            SYS_MODULE_POWER_RUN_FULL

/*** Timer Driver 1 Configuration ***/
#define DRV_TMR_PERIPHERAL_ID_IDX1          TMR_ID_3
#define DRV_TMR_INTERRUPT_SOURCE_IDX1       INT_SOURCE_TIMER_3
#define DRV_TMR_INTERRUPT_VECTOR_IDX1       INT_VECTOR_T3
#define DRV_TMR_ISR_VECTOR_IDX1             _TIMER_3_VECTOR
#define DRV_TMR_INTERRUPT_PRIORITY_IDX1     INT_PRIORITY_LEVEL4
#define DRV_TMR_INTERRUPT_SUB_PRIORITY_IDX1 INT_SUBPRIORITY_LEVEL0
#define DRV_TMR_CLOCK_SOURCE_IDX1           DRV_TMR_CLKSOURCE_INTERNAL
#define DRV_TMR_PRESCALE_IDX1               TMR_PRESCALE_VALUE_256
#define DRV_TMR_OPERATION_MODE_IDX1         DRV_TMR_OPERATION_MODE_16_BIT
#define DRV_TMR_ASYNC_WRITE_ENABLE_IDX1     false
#define DRV_TMR_POWER_STATE_IDX1            SYS_MODULE_POWER_RUN_FULL

 // *****************************************************************************
/* USART Driver Configuration Options
*/
#define DRV_USART_INTERRUPT_MODE                    true

#define DRV_USART_BYTE_MODEL_SUPPORT                false

#define DRV_USART_READ_WRITE_MODEL_SUPPORT          true

#define DRV_USART_BUFFER_QUEUE_SUPPORT              true

#define DRV_USART_CLIENTS_NUMBER                    3
#define DRV_USART_SUPPORT_TRANSMIT_DMA              false
#define DRV_USART_SUPPORT_RECEIVE_DMA               false
#define DRV_USART_INSTANCES_NUMBER                  3

#define DRV_USART_PERIPHERAL_ID_IDX0                USART_ID_4
#define DRV_USART_OPER_MODE_IDX0                    DRV_USART_OPERATION_MODE_NORMAL
#define DRV_USART_OPER_MODE_DATA_IDX0               
#define DRV_USART_INIT_FLAG_WAKE_ON_START_IDX0      false
#define DRV_USART_INIT_FLAG_AUTO_BAUD_IDX0          false
#define DRV_USART_INIT_FLAG_STOP_IN_IDLE_IDX0       false
#define DRV_USART_INIT_FLAGS_IDX0                   0
#define DRV_USART_BRG_CLOCK_IDX0                    100000000
#define DRV_USART_BAUD_RATE_IDX0                    115200
#define DRV_USART_LINE_CNTRL_IDX0                   DRV_USART_LINE_CONTROL_8NONE1
#define DRV_USART_HANDSHAKE_MODE_IDX0               DRV_USART_HANDSHAKE_NONE
#define DRV_USART_XMIT_INT_SRC_IDX0                 INT_SOURCE_USART_4_TRANSMIT
#define DRV_USART_RCV_INT_SRC_IDX0                  INT_SOURCE_USART_4_RECEIVE
#define DRV_USART_ERR_INT_SRC_IDX0                  INT_SOURCE_USART_4_ERROR
#define DRV_USART_XMIT_INT_VECTOR_IDX0              INT_VECTOR_UART4_TX
#define DRV_USART_XMIT_INT_PRIORITY_IDX0            INT_PRIORITY_LEVEL1
#define DRV_USART_XMIT_INT_SUB_PRIORITY_IDX0        INT_SUBPRIORITY_LEVEL0
#define DRV_USART_RCV_INT_VECTOR_IDX0               INT_VECTOR_UART4_RX
#define DRV_USART_RCV_INT_PRIORITY_IDX0             INT_PRIORITY_LEVEL1
#define DRV_USART_RCV_INT_SUB_PRIORITY_IDX0         INT_SUBPRIORITY_LEVEL0
#define DRV_USART_ERR_INT_VECTOR_IDX0               INT_VECTOR_UART4_FAULT
#define DRV_USART_ERR_INT_PRIORITY_IDX0             INT_PRIORITY_LEVEL1
#define DRV_USART_ERR_INT_SUB_PRIORITY_IDX0         INT_SUBPRIORITY_LEVEL0

#define DRV_USART_XMIT_QUEUE_SIZE_IDX0              100
#define DRV_USART_RCV_QUEUE_SIZE_IDX0               100


#define DRV_USART_POWER_STATE_IDX0                  SYS_MODULE_POWER_RUN_FULL

#define DRV_USART_PERIPHERAL_ID_IDX1                USART_ID_1
#define DRV_USART_OPER_MODE_IDX1                    DRV_USART_OPERATION_MODE_NORMAL
#define DRV_USART_OPER_MODE_DATA_IDX1               
#define DRV_USART_INIT_FLAG_WAKE_ON_START_IDX1      true
#define DRV_USART_INIT_FLAG_AUTO_BAUD_IDX1          false
#define DRV_USART_INIT_FLAG_STOP_IN_IDLE_IDX1       false
#define DRV_USART_INIT_FLAGS_IDX1                   1
#define DRV_USART_BRG_CLOCK_IDX1                    100000000
#define DRV_USART_BAUD_RATE_IDX1                    115200
#define DRV_USART_LINE_CNTRL_IDX1                   DRV_USART_LINE_CONTROL_8NONE1
#define DRV_USART_HANDSHAKE_MODE_IDX1               DRV_USART_HANDSHAKE_NONE
#define DRV_USART_XMIT_INT_SRC_IDX1                 INT_SOURCE_USART_1_TRANSMIT
#define DRV_USART_RCV_INT_SRC_IDX1                  INT_SOURCE_USART_1_RECEIVE
#define DRV_USART_ERR_INT_SRC_IDX1                  INT_SOURCE_USART_1_ERROR
#define DRV_USART_XMIT_INT_VECTOR_IDX1              INT_VECTOR_UART1_TX
#define DRV_USART_XMIT_INT_PRIORITY_IDX1            INT_PRIORITY_LEVEL1
#define DRV_USART_XMIT_INT_SUB_PRIORITY_IDX1        INT_SUBPRIORITY_LEVEL0
#define DRV_USART_RCV_INT_VECTOR_IDX1               INT_VECTOR_UART1_RX
#define DRV_USART_RCV_INT_PRIORITY_IDX1             INT_PRIORITY_LEVEL1
#define DRV_USART_RCV_INT_SUB_PRIORITY_IDX1         INT_SUBPRIORITY_LEVEL0
#define DRV_USART_ERR_INT_VECTOR_IDX1               INT_VECTOR_UART1_FAULT
#define DRV_USART_ERR_INT_PRIORITY_IDX1             INT_PRIORITY_LEVEL1
#define DRV_USART_ERR_INT_SUB_PRIORITY_IDX1         INT_SUBPRIORITY_LEVEL0
    
#define DRV_USART_XMIT_QUEUE_SIZE_IDX1              100
#define DRV_USART_RCV_QUEUE_SIZE_IDX1               100


#define DRV_USART_POWER_STATE_IDX1                  SYS_MODULE_POWER_RUN_FULL

#define DRV_USART_PERIPHERAL_ID_IDX2                USART_ID_3
#define DRV_USART_OPER_MODE_IDX2                    DRV_USART_OPERATION_MODE_NORMAL
#define DRV_USART_OPER_MODE_DATA_IDX2               
#define DRV_USART_INIT_FLAG_WAKE_ON_START_IDX2      false
#define DRV_USART_INIT_FLAG_AUTO_BAUD_IDX2          false
#define DRV_USART_INIT_FLAG_STOP_IN_IDLE_IDX2       false
#define DRV_USART_INIT_FLAGS_IDX2                   0
#define DRV_USART_BRG_CLOCK_IDX2                    100000000
#define DRV_USART_BAUD_RATE_IDX2                    115200
#define DRV_USART_LINE_CNTRL_IDX2                   DRV_USART_LINE_CONTROL_8NONE1
#define DRV_USART_HANDSHAKE_MODE_IDX2               DRV_USART_HANDSHAKE_NONE
#define DRV_USART_XMIT_INT_SRC_IDX2                 INT_SOURCE_USART_3_TRANSMIT
#define DRV_USART_RCV_INT_SRC_IDX2                  INT_SOURCE_USART_3_RECEIVE
#define DRV_USART_ERR_INT_SRC_IDX2                  INT_SOURCE_USART_3_ERROR
#define DRV_USART_XMIT_INT_PRIORITY_IDX2            INT_PRIORITY_LEVEL1
#define DRV_USART_XMIT_INT_SUB_PRIORITY_IDX2        INT_SUBPRIORITY_LEVEL0
#define DRV_USART_RCV_INT_VECTOR_IDX2               INT_VECTOR_UART3_RX
#define DRV_USART_RCV_INT_PRIORITY_IDX2             INT_PRIORITY_LEVEL1
#define DRV_USART_RCV_INT_SUB_PRIORITY_IDX2         INT_SUBPRIORITY_LEVEL0
#define DRV_USART_ERR_INT_VECTOR_IDX2               INT_VECTOR_UART3_FAULT
#define DRV_USART_ERR_INT_PRIORITY_IDX2             INT_PRIORITY_LEVEL1
#define DRV_USART_ERR_INT_SUB_PRIORITY_IDX2         INT_SUBPRIORITY_LEVEL0

#define DRV_USART_XMIT_QUEUE_SIZE_IDX2              100
#define DRV_USART_RCV_QUEUE_SIZE_IDX2               100


#define DRV_USART_POWER_STATE_IDX2                  SYS_MODULE_POWER_RUN_FULL

#define DRV_USART_QUEUE_DEPTH_COMBINED              600

/*** NVM Driver Configuration ***/

#define DRV_NVM_INSTANCES_NUMBER     	1
#define DRV_NVM_CLIENTS_NUMBER        	2
#define DRV_NVM_BUFFER_OBJECT_NUMBER  	5

#define DRV_NVM_INTERRUPT_MODE        	true
#define DRV_NVM_INTERRUPT_SOURCE      	INT_SOURCE_FLASH_CONTROL

#define DRV_NVM_MEDIA_SIZE              64
#define DRV_NVM_MEDIA_START_ADDRESS     0x9D010000

#define DRV_NVM_ERASE_WRITE_ENABLE


#define DRV_NVM_SYS_FS_REGISTER



/*** SDCARD Driver Configuration ***/
#define DRV_SDCARD_INSTANCES_NUMBER     1
#define DRV_SDCARD_CLIENTS_NUMBER       1
#define DRV_SDCARD_INDEX_MAX            1
#define DRV_SDCARD_INDEX                DRV_SDCARD_INDEX_0
#define DRV_SDCARD_QUEUE_POOL_SIZE      10
#define DRV_SDCARD_SPI_DRV_INSTANCE     0

#define DRV_SDCARD_SYS_FS_REGISTER




/*** SPI Driver Configuration ***/
#define DRV_SPI_NUMBER_OF_MODULES		6
/*** Driver Compilation and static configuration options. ***/
/*** Select SPI compilation units.***/
#define DRV_SPI_POLLED 				0
#define DRV_SPI_ISR 				1
#define DRV_SPI_MASTER 				1
#define DRV_SPI_SLAVE 				0
#define DRV_SPI_RM 					1
#define DRV_SPI_EBM 				1
#define DRV_SPI_8BIT 				1
#define DRV_SPI_16BIT 				0
#define DRV_SPI_32BIT 				0
#define DRV_SPI_DMA 				1

/*** SPI Driver Static Allocation Options ***/
#define DRV_SPI_INSTANCES_NUMBER 		3
#define DRV_SPI_CLIENTS_NUMBER 			3
#define DRV_SPI_ELEMENTS_PER_QUEUE 		10
/*** SPI Driver DMA Options ***/
#define DRV_SPI_DMA_TXFER_SIZE 			512
#define DRV_SPI_DMA_DUMMY_BUFFER_SIZE 	512
/* SPI Driver Instance 0 Configuration */
#define DRV_SPI_SPI_ID_IDX0 				SPI_ID_2
#define DRV_SPI_TASK_MODE_IDX0 				DRV_SPI_TASK_MODE_ISR
#define DRV_SPI_SPI_MODE_IDX0				DRV_SPI_MODE_MASTER
#define DRV_SPI_ALLOW_IDLE_RUN_IDX0			false
#define DRV_SPI_SPI_PROTOCOL_TYPE_IDX0 		DRV_SPI_PROTOCOL_TYPE_STANDARD
#define DRV_SPI_COMM_WIDTH_IDX0 			SPI_COMMUNICATION_WIDTH_8BITS
#define DRV_SPI_SPI_CLOCK_IDX0 				CLK_BUS_PERIPHERAL_2
#define DRV_SPI_BAUD_RATE_IDX0 				100000
#define DRV_SPI_BUFFER_TYPE_IDX0 			DRV_SPI_BUFFER_TYPE_ENHANCED
#define DRV_SPI_CLOCK_MODE_IDX0 			DRV_SPI_CLOCK_MODE_IDLE_LOW_EDGE_FALL
#define DRV_SPI_INPUT_PHASE_IDX0 			SPI_INPUT_SAMPLING_PHASE_AT_END
#define DRV_SPI_TX_INT_SOURCE_IDX0 			INT_SOURCE_SPI_2_TRANSMIT
#define DRV_SPI_RX_INT_SOURCE_IDX0 			INT_SOURCE_SPI_2_RECEIVE
#define DRV_SPI_ERROR_INT_SOURCE_IDX0 		INT_SOURCE_SPI_2_ERROR
#define DRV_SPI_TX_INT_VECTOR_IDX0			INT_VECTOR_SPI2_TX
#define DRV_SPI_RX_INT_VECTOR_IDX0			INT_VECTOR_SPI2_RX
#define DRV_DRV_SPI_ERROR_INT_VECTOR_IDX0	INT_VECTOR_SPI2_FAULT
#define DRV_SPI_TX_INT_PRIORITY_IDX0 		INT_PRIORITY_LEVEL3
#define DRV_SPI_TX_INT_SUB_PRIORITY_IDX0 	INT_SUBPRIORITY_LEVEL0
#define DRV_SPI_RX_INT_PRIORITY_IDX0 		INT_PRIORITY_LEVEL3
#define DRV_SPI_RX_INT_SUB_PRIORITY_IDX0 	INT_SUBPRIORITY_LEVEL0
#define DRV_SPI_ERROR_INT_PRIORITY_IDX0 	INT_PRIORITY_LEVEL3
#define DRV_SPI_ERROR_INT_SUB_PRIORITY_IDX0 INT_SUBPRIORITY_LEVEL0
#define DRV_SPI_QUEUE_SIZE_IDX0 			10
#define DRV_SPI_RESERVED_JOB_IDX0 			1
/* SPI Driver Instance 1 Configuration */
#define DRV_SPI_SPI_ID_IDX1 				SPI_ID_6
#define DRV_SPI_TASK_MODE_IDX1 				DRV_SPI_TASK_MODE_ISR
#define DRV_SPI_SPI_MODE_IDX1				DRV_SPI_MODE_MASTER
#define DRV_SPI_ALLOW_IDLE_RUN_IDX1			false
#define DRV_SPI_SPI_PROTOCOL_TYPE_IDX1 		DRV_SPI_PROTOCOL_TYPE_STANDARD
#define DRV_SPI_COMM_WIDTH_IDX1 			SPI_COMMUNICATION_WIDTH_8BITS
#define DRV_SPI_SPI_CLOCK_IDX1 				CLK_BUS_PERIPHERAL_2
#define DRV_SPI_BAUD_RATE_IDX1 				8000000
#define DRV_SPI_BUFFER_TYPE_IDX1 			DRV_SPI_BUFFER_TYPE_ENHANCED
#define DRV_SPI_CLOCK_MODE_IDX1 			DRV_SPI_CLOCK_MODE_IDLE_HIGH_EDGE_FALL
#define DRV_SPI_INPUT_PHASE_IDX1 			SPI_INPUT_SAMPLING_PHASE_AT_END
#define DRV_SPI_TX_INT_SOURCE_IDX1 			INT_SOURCE_SPI_6_TRANSMIT
#define DRV_SPI_RX_INT_SOURCE_IDX1 			INT_SOURCE_SPI_6_RECEIVE
#define DRV_SPI_ERROR_INT_SOURCE_IDX1 		INT_SOURCE_SPI_6_ERROR
#define DRV_SPI_TX_INT_VECTOR_IDX1			INT_VECTOR_SPI6_TX
#define DRV_SPI_RX_INT_VECTOR_IDX1			INT_VECTOR_SPI6_RX
#define DRV_DRV_SPI_ERROR_INT_VECTOR_IDX1	INT_VECTOR_SPI6_FAULT
#define DRV_SPI_TX_INT_PRIORITY_IDX1 		INT_PRIORITY_LEVEL1
#define DRV_SPI_TX_INT_SUB_PRIORITY_IDX1 	INT_SUBPRIORITY_LEVEL0
#define DRV_SPI_RX_INT_PRIORITY_IDX1 		INT_PRIORITY_LEVEL1
#define DRV_SPI_RX_INT_SUB_PRIORITY_IDX1 	INT_SUBPRIORITY_LEVEL0
#define DRV_SPI_ERROR_INT_PRIORITY_IDX1 	INT_PRIORITY_LEVEL1
#define DRV_SPI_ERROR_INT_SUB_PRIORITY_IDX1 INT_SUBPRIORITY_LEVEL0
#define DRV_SPI_QUEUE_SIZE_IDX1 			10
#define DRV_SPI_RESERVED_JOB_IDX1 			1
#define DRV_SPI_TX_DMA_CHANNEL_IDX1 		DMA_CHANNEL_1
#define DRV_SPI_TX_DMA_THRESHOLD_IDX1 		16
#define DRV_SPI_RX_DMA_CHANNEL_IDX1 		DMA_CHANNEL_0
#define DRV_SPI_RX_DMA_THRESHOLD_IDX1 		16
/* SPI Driver Instance 2 Configuration */
#define DRV_SPI_SPI_ID_IDX2 				SPI_ID_1
#define DRV_SPI_TASK_MODE_IDX2 				DRV_SPI_TASK_MODE_ISR
#define DRV_SPI_SPI_MODE_IDX2				DRV_SPI_MODE_MASTER
#define DRV_SPI_ALLOW_IDLE_RUN_IDX2			false
#define DRV_SPI_SPI_PROTOCOL_TYPE_IDX2 		DRV_SPI_PROTOCOL_TYPE_STANDARD
#define DRV_SPI_COMM_WIDTH_IDX2 			SPI_COMMUNICATION_WIDTH_8BITS
#define DRV_SPI_SPI_CLOCK_IDX2 				CLK_BUS_PERIPHERAL_2
#define DRV_SPI_BAUD_RATE_IDX2 				40000000
#define DRV_SPI_BUFFER_TYPE_IDX2 			DRV_SPI_BUFFER_TYPE_STANDARD
#define DRV_SPI_CLOCK_MODE_IDX2 			DRV_SPI_CLOCK_MODE_IDLE_LOW_EDGE_FALL
#define DRV_SPI_INPUT_PHASE_IDX2 			SPI_INPUT_SAMPLING_PHASE_AT_END
#define DRV_SPI_TX_INT_SOURCE_IDX2 			INT_SOURCE_SPI_1_TRANSMIT
#define DRV_SPI_RX_INT_SOURCE_IDX2 			INT_SOURCE_SPI_1_RECEIVE
#define DRV_SPI_ERROR_INT_SOURCE_IDX2 		INT_SOURCE_SPI_1_ERROR
#define DRV_SPI_TX_INT_VECTOR_IDX2			INT_VECTOR_SPI1_TX
#define DRV_SPI_RX_INT_VECTOR_IDX2			INT_VECTOR_SPI1_RX
#define DRV_DRV_SPI_ERROR_INT_VECTOR_IDX2	INT_VECTOR_SPI1_FAULT
#define DRV_SPI_TX_INT_PRIORITY_IDX2 		INT_PRIORITY_LEVEL1
#define DRV_SPI_TX_INT_SUB_PRIORITY_IDX2 	INT_SUBPRIORITY_LEVEL0
#define DRV_SPI_RX_INT_PRIORITY_IDX2 		INT_PRIORITY_LEVEL1
#define DRV_SPI_RX_INT_SUB_PRIORITY_IDX2 	INT_SUBPRIORITY_LEVEL0
#define DRV_SPI_ERROR_INT_PRIORITY_IDX2 	INT_PRIORITY_LEVEL1
#define DRV_SPI_ERROR_INT_SUB_PRIORITY_IDX2 INT_SUBPRIORITY_LEVEL0
#define DRV_SPI_QUEUE_SIZE_IDX2 			10
#define DRV_SPI_RESERVED_JOB_IDX2 			2

/*** Wi-Fi Driver Configuration ***/


#define MODULE_EVENT_PRINT 0
#define PIN_MODE_PER_PORT_SELECT

#define WDRV_EXT_RTOS_INIT_TASK_SIZE 512u
#define WDRV_EXT_RTOS_INIT_TASK_PRIORITY 1u
#define WDRV_EXT_RTOS_MAIN_TASK_SIZE 2048u
#define WDRV_EXT_RTOS_MAIN_TASK_PRIORITY 5u

#define WDRV_ASSERT(condition, msg) ASSERT(condition, msg)

#define WDRV_SPI_INDEX 1
#define WDRV_SPI_INSTANCE sysObj.spiObjectIdx1

#define WDRV_USE_SPI_DMA


#define WDRV_BOARD_TYPE WDRV_BD_TYPE_CUSTOM

// I/O mappings for general control pins, including CS, HIBERNATE, RESET and INTERRUPT.
#define WF_CS_PORT_CHANNEL PORT_CHANNEL_D
#define WF_CS_BIT_POS      14

#define WF_HIBERNATE_PORT_CHANNEL PORT_CHANNEL_H
#define WF_HIBERNATE_BIT_POS      7

#define WF_RESET_PORT_CHANNEL PORT_CHANNEL_A
#define WF_RESET_BIT_POS      0

#define WF_INT_PORT_CHANNEL PORT_CHANNEL_F
#define WF_INT_BIT_POS      13

#define MRF_INT_SOURCE INT_SOURCE_CHANGE_NOTICE_F
#define MRF_INT_VECTOR INT_VECTOR_CN


#define WDRV_DEFAULT_NETWORK_TYPE WDRV_NETWORK_TYPE_SOFT_AP
#define WDRV_DEFAULT_SSID "Things-Gateway"

#define WDRV_DEFAULT_SECURITY_MODE WDRV_SECURITY_WPA2_WITH_PASS_PHRASE
#define WDRV_DEFAULT_WEP_KEYS_40 "5AFB6C8E77" // default WEP40 key
#define WDRV_DEFAULT_WEP_KEYS_104 "90E96780C739409DA50034FCAA" // default WEP104 key
#define WDRV_DEFAULT_PSK_PHRASE "thethings" // customized WPA-PSK or WPA2-PSK passphrase
#define WDRV_DEFAULT_WPS_PIN "12390212" // default WPS PIN

#define WDRV_DEFAULT_CHANNEL 6
#define WDRV_DEFAULT_POWER_SAVE WDRV_FUNC_DISABLED

// *****************************************************************************
/* SST25VF064C Driver Configuration Options
*/


#define DRV_SST25VF064C_QUEUE_DEPTH_COMBINED                    16
#define DRV_SST25VF064C_CLIENTS_NUMBER                          1
#define DRV_SST25VF064C_INSTANCES_NUMBER                        1

#define DRV_SST25VF064C_POWER_STATE_IDX0                        SYS_MODULE_POWER_RUN_FULL
#define DRV_SST25VF064C_SPI_DRIVER_INSTANCE_IDX0                2
#define DRV_SST25VF064C_HOLD_PIN_PORT_CHANNEL_IDX0              PORT_CHANNEL_D
#define DRV_SST25VF064C_HOLD_PIN_PORT_BIT_POS_IDX0              PORTS_BIT_POS_13
#define DRV_SST25VF064C_WRITE_PROTECT_PIN_PORT_CHANNEL_IDX0     PORT_CHANNEL_J
#define DRV_SST25VF064C_WRITE_PROTECT_PIN_BIT_POS_IDX0          PORTS_BIT_POS_2
#define DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0           PORT_CHANNEL_D
#define DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0           PORTS_BIT_POS_12
#define DRV_SST25VF064C_QUEUE_SIZE_IDX0                         10





// *****************************************************************************
// *****************************************************************************
// Section: Middleware & Other Library Configuration
// *****************************************************************************
// *****************************************************************************
/*** Crypto Library Configuration ***/

#define HAVE_MCAPI


// *****************************************************************************
// *****************************************************************************
// Section: TCPIP Stack Configuration
// *****************************************************************************
// *****************************************************************************
#define TCPIP_STACK_USE_IPV4
#define TCPIP_STACK_USE_TCP
#define TCPIP_STACK_USE_UDP

#define TCPIP_STACK_TICK_RATE		        		5
#define TCPIP_STACK_SECURE_PORT_ENTRIES             10

/* TCP/IP stack event notification */
#define TCPIP_STACK_USE_EVENT_NOTIFICATION
#define TCPIP_STACK_USER_NOTIFICATION   false
#define TCPIP_STACK_DOWN_OPERATION   true
#define TCPIP_STACK_IF_UP_DOWN_OPERATION   true
#define TCPIP_STACK_MAC_DOWN_OPERATION  true
#define TCPIP_STACK_CONFIGURATION_SAVE_RESTORE   true
/*** TCPIP Heap Configuration ***/
#define TCPIP_STACK_USE_EXTERNAL_HEAP
#define TCPIP_STACK_MALLOC_FUNC                     malloc

#define TCPIP_STACK_CALLOC_FUNC                     calloc

#define TCPIP_STACK_FREE_FUNC                       free



#define TCPIP_STACK_HEAP_USE_FLAGS                   TCPIP_STACK_HEAP_FLAG_ALLOC_UNCACHED

#define TCPIP_STACK_HEAP_USAGE_CONFIG                TCPIP_STACK_HEAP_USE_DEFAULT

#define TCPIP_STACK_SUPPORTED_HEAPS                  1

/*** ARP Configuration ***/
#define TCPIP_ARP_CACHE_ENTRIES                 		5
#define TCPIP_ARP_CACHE_DELETE_OLD		        	true
#define TCPIP_ARP_CACHE_SOLVED_ENTRY_TMO			1200
#define TCPIP_ARP_CACHE_PENDING_ENTRY_TMO			60
#define TCPIP_ARP_CACHE_PENDING_RETRY_TMO			2
#define TCPIP_ARP_CACHE_PERMANENT_QUOTA		    		50
#define TCPIP_ARP_CACHE_PURGE_THRESHOLD		    		75
#define TCPIP_ARP_CACHE_PURGE_QUANTA		    		1
#define TCPIP_ARP_CACHE_ENTRY_RETRIES		    		3
#define TCPIP_ARP_GRATUITOUS_PROBE_COUNT			1
#define TCPIP_ARP_TASK_PROCESS_RATE		        	2

/*** DHCP Configuration ***/
#define TCPIP_STACK_USE_DHCP_CLIENT
#define TCPIP_DHCP_TIMEOUT		        		2
#define TCPIP_DHCP_TASK_TICK_RATE	    			200
#define TCPIP_DHCP_HOST_NAME_SIZE	    			20
#define TCPIP_DHCP_CLIENT_CONNECT_PORT  			68
#define TCPIP_DHCP_SERVER_LISTEN_PORT				67
#define TCPIP_DHCP_CLIENT_ENABLED             			true


/*** DHCP Server Configuration ***/
#define TCPIP_STACK_USE_DHCP_SERVER
#define TCPIP_DHCPS_TASK_PROCESS_RATE                               200
#define TCPIP_DHCPS_LEASE_ENTRIES_DEFAULT                           15
#define TCPIP_DHCPS_LEASE_SOLVED_ENTRY_TMO                          1200
#define TCPIP_DHCPS_LEASE_REMOVED_BEFORE_ACK                        5
#define TCPIP_DHCP_SERVER_DELETE_OLD_ENTRIES                        true
#define TCPIP_DHCPS_LEASE_DURATION	TCPIP_DHCPS_LEASE_SOLVED_ENTRY_TMO

/*** DHCP Server Instance 0 Configuration ***/
#define TCPIP_DHCPS_DEFAULT_IP_ADDRESS_RANGE_START_IDX0             "192.168.84.84"

#define TCPIP_DHCPS_DEFAULT_SERVER_IP_ADDRESS_IDX0                  "192.168.84.1"

#define TCPIP_DHCPS_DEFAULT_SERVER_NETMASK_ADDRESS_IDX0             "255.255.255.0"

#define TCPIP_DHCPS_DEFAULT_SERVER_GATEWAY_ADDRESS_IDX0             "192.168.84.1"

#define TCPIP_DHCPS_DEFAULT_SERVER_PRIMARY_DNS_ADDRESS_IDX0         "192.168.84.1"

#define TCPIP_DHCPS_DEFAULT_SERVER_SECONDARY_DNS_ADDRESS_IDX0       "192.168.84.1"

#define TCPIP_DHCP_SERVER_INTERFACE_INDEX_IDX0                      0

#define TCPIP_DHCP_SERVER_POOL_ENABLED_IDX0                         true



/*** DNS Client Configuration ***/
#define TCPIP_STACK_USE_DNS
#define TCPIP_DNS_CLIENT_SERVER_TMO					60
#define TCPIP_DNS_CLIENT_TASK_PROCESS_RATE			200
#define TCPIP_DNS_CLIENT_CACHE_ENTRIES				5
#define TCPIP_DNS_CLIENT_CACHE_ENTRY_TMO			0
#define TCPIP_DNS_CLIENT_CACHE_PER_IPV4_ADDRESS		5
#define TCPIP_DNS_CLIENT_CACHE_PER_IPV6_ADDRESS		1
#define TCPIP_DNS_CLIENT_ADDRESS_TYPE			    IP_ADDRESS_TYPE_IPV4
#define TCPIP_DNS_CLIENT_CACHE_DEFAULT_TTL_VAL		1200
#define TCPIP_DNS_CLIENT_CACHE_UNSOLVED_ENTRY_TMO	10
#define TCPIP_DNS_CLIENT_LOOKUP_RETRY_TMO			5
#define TCPIP_DNS_CLIENT_MAX_HOSTNAME_LEN			128
#define TCPIP_DNS_CLIENT_MAX_SELECT_INTERFACES		4
#define TCPIP_DNS_CLIENT_DELETE_OLD_ENTRIES			true
#define TCPIP_DNS_CLIENT_USER_NOTIFICATION   false



/*** HTTP Configuration ***/
#define TCPIP_STACK_USE_HTTP_SERVER
#define TCPIP_HTTP_MAX_HEADER_LEN		    		255
#define TCPIP_HTTP_CACHE_LEN		        		"600"
#define TCPIP_HTTP_TIMEOUT		            		45
#define TCPIP_HTTP_MAX_CONNECTIONS		    		4
#define TCPIP_HTTP_MAX_TLS_CONNECTIONS		  		0
#define TCPIP_HTTP_DEFAULT_FILE		        		"index.html"
#define TCPIP_HTTPS_DEFAULT_FILE	        		"index.html"
#define TCPIP_HTTP_DEFAULT_LEN		        		10
#define TCPIP_HTTP_MAX_DATA_LEN		        		100
#define TCPIP_HTTP_MIN_CALLBACK_FREE				16
#define TCPIP_HTTP_SKT_TX_BUFF_SIZE		    		1024
#define TCPIP_HTTP_SKT_RX_BUFF_SIZE		    		0
#define TCPIP_HTTP_TLS_SKT_TX_BUFF_SIZE		                0
#define TCPIP_HTTP_TLS_SKT_RX_BUFF_SIZE		                0
#define TCPIP_HTTP_CONFIG_FLAGS		        		1
#define TCPIP_HTTP_USE_POST
#define TCPIP_HTTP_USE_COOKIES
#define TCPIP_HTTP_USE_BASE64_DECODE
#define TCPIP_HTTP_USE_AUTHENTICATION
#define TCPIP_HTTP_TASK_RATE					10


/*** ICMPv4 Server Configuration ***/
#define TCPIP_STACK_USE_ICMP_SERVER

/*** ICMPv4 Client Configuration ***/
#define TCPIP_STACK_USE_ICMP_CLIENT
#define TCPIP_ICMP_CLIENT_USER_NOTIFICATION   true


/*** NBNS Configuration ***/
#define TCPIP_STACK_USE_NBNS
#define TCPIP_NBNS_TASK_TICK_RATE   110


/*** SNTP Configuration ***/
#define TCPIP_STACK_USE_SNTP_CLIENT
#define TCPIP_NTP_DEFAULT_IF		        		"PIC32INT"
#define TCPIP_NTP_VERSION             			    	4
#define TCPIP_NTP_DEFAULT_CONNECTION_TYPE   			IP_ADDRESS_TYPE_IPV4
#define TCPIP_NTP_EPOCH		                		2208988800ul
#define TCPIP_NTP_REPLY_TIMEOUT		        		6
#define TCPIP_NTP_MAX_STRATUM		        		15
#define TCPIP_NTP_TIME_STAMP_TMO				660
#define TCPIP_NTP_SERVER		        		"pool.ntp.org"
#define TCPIP_NTP_SERVER_MAX_LENGTH				30
#define TCPIP_NTP_QUERY_INTERVAL				600
#define TCPIP_NTP_FAST_QUERY_INTERVAL	    			14
#define TCPIP_NTP_TASK_TICK_RATE				1100
#define TCPIP_NTP_RX_QUEUE_LIMIT				2




/*** TCP Configuration ***/
#define TCPIP_TCP_MAX_SEG_SIZE_TX		        	1460
#define TCPIP_TCP_MAX_SEG_SIZE_RX_LOCAL		    		1460
#define TCPIP_TCP_MAX_SEG_SIZE_RX_NON_LOCAL			536
#define TCPIP_TCP_SOCKET_DEFAULT_TX_SIZE			1024
#define TCPIP_TCP_SOCKET_DEFAULT_RX_SIZE			2048
#define TCPIP_TCP_DYNAMIC_OPTIONS             			true
#define TCPIP_TCP_START_TIMEOUT_VAL		        	1000
#define TCPIP_TCP_DELAYED_ACK_TIMEOUT		    		100
#define TCPIP_TCP_FIN_WAIT_2_TIMEOUT		    		5000
#define TCPIP_TCP_KEEP_ALIVE_TIMEOUT		    		10000
#define TCPIP_TCP_CLOSE_WAIT_TIMEOUT		    		2000
#define TCPIP_TCP_MAX_RETRIES		            		5
#define TCPIP_TCP_MAX_UNACKED_KEEP_ALIVES			6
#define TCPIP_TCP_MAX_SYN_RETRIES		        	2
#define TCPIP_TCP_AUTO_TRANSMIT_TIMEOUT_VAL			40
#define TCPIP_TCP_WINDOW_UPDATE_TIMEOUT_VAL			200
#define TCPIP_TCP_MAX_SOCKETS		            		10
#define TCPIP_TCP_TASK_TICK_RATE		        	5


/*** TCPIP MAC Configuration ***/
#define TCPIP_EMAC_TX_DESCRIPTORS				8
#define TCPIP_EMAC_RX_DESCRIPTORS				6
#define TCPIP_EMAC_RX_DEDICATED_BUFFERS				4
#define TCPIP_EMAC_RX_INIT_BUFFERS				    0
#define TCPIP_EMAC_RX_LOW_THRESHOLD				    1
#define TCPIP_EMAC_RX_LOW_FILL				        2
#define TCPIP_EMAC_RX_BUFF_SIZE		    			1536
#define TCPIP_EMAC_RX_MAX_FRAME		    			1536
#define TCPIP_EMAC_RX_FILTERS                       \
                                                    TCPIP_MAC_RX_FILTER_TYPE_BCAST_ACCEPT |\
                                                    TCPIP_MAC_RX_FILTER_TYPE_MCAST_ACCEPT |\
                                                    TCPIP_MAC_RX_FILTER_TYPE_UCAST_ACCEPT |\
                                                    TCPIP_MAC_RX_FILTER_TYPE_RUNT_REJECT |\
                                                    TCPIP_MAC_RX_FILTER_TYPE_CRC_ERROR_REJECT |\
                                                    0
#define TCPIP_EMAC_RX_FRAGMENTS		    			1
#define TCPIP_EMAC_ETH_OPEN_FLAGS       			\
                                                    TCPIP_ETH_OPEN_AUTO |\
                                                    TCPIP_ETH_OPEN_FDUPLEX |\
                                                    TCPIP_ETH_OPEN_HDUPLEX |\
                                                    TCPIP_ETH_OPEN_100 |\
                                                    TCPIP_ETH_OPEN_10 |\
                                                    TCPIP_ETH_OPEN_MDIX_AUTO |\
                                                    0
#define TCPIP_EMAC_PHY_CONFIG_FLAGS     			\
                                                    DRV_ETHPHY_CFG_AUTO | \
                                                    0                                                    
#define TCPIP_EMAC_PHY_LINK_INIT_DELAY  			500
#define TCPIP_EMAC_PHY_ADDRESS		    			0
#define TCPIP_EMAC_MODULE_ID		    			ETH_ID_0
#define TCPIP_EMAC_INTERRUPT_MODE        			true
#define DRV_ETHPHY_INSTANCES_NUMBER				1
#define DRV_ETHPHY_CLIENTS_NUMBER				1
#define DRV_ETHPHY_INDEX		        		1
#define DRV_ETHPHY_PERIPHERAL_ID				1
#define DRV_ETHPHY_NEG_INIT_TMO		    			1
#define DRV_ETHPHY_NEG_DONE_TMO		    			2000
#define DRV_ETHPHY_RESET_CLR_TMO				500
#define DRV_ETHMAC_INSTANCES_NUMBER				1
#define DRV_ETHMAC_CLIENTS_NUMBER				1
#define DRV_ETHMAC_INDEX	    	    			1
#define DRV_ETHMAC_PERIPHERAL_ID				1
#define DRV_ETHMAC_INTERRUPT_VECTOR				INT_VECTOR_ETHERNET
#define DRV_ETHMAC_INTERRUPT_SOURCE				INT_SOURCE_ETH_1
#define DRV_ETHMAC_POWER_STATE		    			SYS_MODULE_POWER_RUN_FULL

#define DRV_ETHMAC_INTERRUPT_MODE        			true



/*** UDP Configuration ***/
#define TCPIP_UDP_MAX_SOCKETS		                	10
#define TCPIP_UDP_SOCKET_DEFAULT_TX_SIZE		    	512
#define TCPIP_UDP_SOCKET_DEFAULT_TX_QUEUE_LIMIT    	 	3
#define TCPIP_UDP_SOCKET_DEFAULT_RX_QUEUE_LIMIT			5
#define TCPIP_UDP_USE_POOL_BUFFERS   false
#define TCPIP_UDP_USE_TX_CHECKSUM             			true

#define TCPIP_UDP_USE_RX_CHECKSUM             			true

#define TCPIP_STACK_USE_ZEROCONF_LINK_LOCAL
#define TCPIP_ZC_LL_PROBE_WAIT 1
#define TCPIP_ZC_LL_PROBE_MIN 1
#define TCPIP_ZC_LL_PROBE_MAX 2
#define TCPIP_ZC_LL_PROBE_NUM 3
#define TCPIP_ZC_LL_ANNOUNCE_WAIT 2
#define TCPIP_ZC_LL_ANNOUNCE_NUM 2
#define TCPIP_ZC_LL_ANNOUNCE_INTERVAL 2
#define TCPIP_ZC_LL_MAX_CONFLICTS 10
#define TCPIP_ZC_LL_RATE_LIMIT_INTERVAL 60
#define TCPIP_ZC_LL_DEFEND_INTERVAL 10
#define TCPIP_ZC_LL_IPV4_LLBASE 0xa9fe0100
#define TCPIP_ZC_LL_IPV4_LLBASE_MASK 0x0000FFFF
#define TCPIP_ZC_LL_TASK_TICK_RATE 333
#define TCPIP_STACK_USE_ZEROCONF_MDNS_SD
#define TCPIP_ZC_MDNS_TASK_TICK_RATE 63
#define TCPIP_ZC_MDNS_PORT 5353
#define TCPIP_ZC_MDNS_MAX_HOST_NAME_SIZE 128
#define TCPIP_ZC_MDNS_MAX_LABEL_SIZE 64
#define TCPIP_ZC_MDNS_MAX_RR_NAME_SIZE 256
#define TCPIP_ZC_MDNS_MAX_SRV_TYPE_SIZE 32
#define TCPIP_ZC_MDNS_MAX_SRV_NAME_SIZE 128
#define TCPIP_ZC_MDNS_MAX_TXT_DATA_SIZE 128
#define TCPIP_ZC_MDNS_RESOURCE_RECORD_TTL_VAL 3600
#define TCPIP_ZC_MDNS_MAX_RR_NUM 4
#define TCPIP_ZC_MDNS_PROBE_WAIT 750
#define TCPIP_ZC_MDNS_PROBE_INTERVAL 250
#define TCPIP_ZC_MDNS_PROBE_NUM 3
#define TCPIP_ZC_MDNS_MAX_PROBE_CONFLICT_NUM 30
#define TCPIP_ZC_MDNS_ANNOUNCE_NUM 3
#define TCPIP_ZC_MDNS_ANNOUNCE_INTERVAL 250
#define TCPIP_ZC_MDNS_ANNOUNCE_WAIT 250


/*** Network Configuration Index 0 ***/
#define TCPIP_NETWORK_DEFAULT_INTERFACE_NAME 			"MRF24WN"
#define TCPIP_IF_MRF24WN
#define TCPIP_NETWORK_DEFAULT_HOST_NAME 			"things-gateway"
#define TCPIP_NETWORK_DEFAULT_MAC_ADDR	 			0
#define TCPIP_NETWORK_DEFAULT_IP_ADDRESS 			"0.0.0.0"
#define TCPIP_NETWORK_DEFAULT_IP_MASK 				"0.0.0.0"
#define TCPIP_NETWORK_DEFAULT_GATEWAY	 			"0.0.0.0"
#define TCPIP_NETWORK_DEFAULT_DNS 				"8.8.8.8"
#define TCPIP_NETWORK_DEFAULT_SECOND_DNS 			"8.8.4.4"
#define TCPIP_NETWORK_DEFAULT_POWER_MODE 			"full"
#define TCPIP_NETWORK_DEFAULT_INTERFACE_FLAGS                       \
                                                    TCPIP_NETWORK_CONFIG_IP_STATIC
#define TCPIP_NETWORK_DEFAULT_MAC_DRIVER 		    WDRV_MRF24WN_MACObject
#define TCPIP_NETWORK_DEFAULT_IPV6_ADDRESS 			0
#define TCPIP_NETWORK_DEFAULT_IPV6_PREFIX_LENGTH    
#define TCPIP_NETWORK_DEFAULT_IPV6_GATEWAY 		    0

/*** Network Configuration Index 0 ***/
#define TCPIP_NETWORK_DEFAULT_INTERFACE_NAME_IDX1 		"PIC32INT"
#define TCPIP_IF_PIC32INT
#define TCPIP_NETWORK_DEFAULT_HOST_NAME_IDX1 			"things-gateway"
#define TCPIP_NETWORK_DEFAULT_MAC_ADDR_IDX1 			0
#define TCPIP_NETWORK_DEFAULT_IP_ADDRESS_IDX1 			"0.0.0.0"
#define TCPIP_NETWORK_DEFAULT_IP_MASK_IDX1 			"0.0.0.0"
#define TCPIP_NETWORK_DEFAULT_GATEWAY_IDX1 			"0.0.0.0"
#define TCPIP_NETWORK_DEFAULT_DNS_IDX1 				"8.8.8.8"
#define TCPIP_NETWORK_DEFAULT_SECOND_DNS_IDX1 			"8.8.4.4"
#define TCPIP_NETWORK_DEFAULT_POWER_MODE_IDX1 			"full"
#define TCPIP_NETWORK_DEFAULT_INTERFACE_FLAGS_IDX1      \
                                                    TCPIP_NETWORK_CONFIG_DHCP_CLIENT_ON |\
                                                    TCPIP_NETWORK_CONFIG_DNS_CLIENT_ON |\
                                                    TCPIP_NETWORK_CONFIG_IP_STATIC
#define TCPIP_NETWORK_DEFAULT_MAC_DRIVER_IDX1 		DRV_ETHMAC_PIC32MACObject
#define TCPIP_NETWORK_DEFAULT_IPV6_ADDRESS_IDX1     0
#define TCPIP_NETWORK_DEFAULT_IPV6_PREFIX_LENGTH_IDX1   
#define TCPIP_NETWORK_DEFAULT_IPV6_GATEWAY_IDX1 		0
/*** tcpip_cmd Configuration ***/
#define TCPIP_STACK_COMMAND_ENABLE
#define TCPIP_STACK_COMMANDS_STORAGE_ENABLE
#define TCPIP_STACK_COMMANDS_ICMP_ECHO_REQUESTS         4
#define TCPIP_STACK_COMMANDS_ICMP_ECHO_REQUEST_DELAY    1000
#define TCPIP_STACK_COMMANDS_ICMP_ECHO_TIMEOUT          5000
#define TCPIP_STACK_COMMANDS_WIFI_ENABLE             	true


/*** TCPIP SYS FS Wrapper ***/
#define SYS_FS_MAX_PATH						80
#define LOCAL_WEBSITE_PATH_FS				"/mnt/mchpSite1"
#define LOCAL_WEBSITE_PATH					"/mnt/mchpSite1/"
#define SYS_FS_DRIVE						"FLASH"
#define SYS_FS_NVM_VOL						"/dev/nvma1"
#define SYS_FS_FATFS_STRING					"FATFS"
#define SYS_FS_MPFS_STRING					"MPFS2"

/* BSP LED Re-directs */
//#define APP_TCPIP_LED_1 BSP_LED_1
//#define APP_TCPIP_LED_2 BSP_LED_2
//#define APP_TCPIP_LED_3 BSP_LED_3

//#define APP_TCPIP_SWITCH_1 BSP_SWITCH_1
//#define APP_TCPIP_SWITCH_2 BSP_SWITCH_2
//#define APP_TCPIP_SWITCH_3 BSP_SWITCH_3
/*** OSAL Configuration ***/
#define OSAL_USE_RTOS          1

/* MPLAB Harmony Net Presentation Layer Definitions*/
#define NET_PRES_NUM_INSTANCE 1
#define NET_PRES_NUM_SOCKETS 1



// *****************************************************************************
// *****************************************************************************
// Section: Application Configuration
// *****************************************************************************
// *****************************************************************************

/*** Application Instance 0 Configuration ***/

/*** Application Instance 1 Configuration ***/

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END


#endif // _SYSTEM_CONFIG_H
/*******************************************************************************
 End of File
*/

