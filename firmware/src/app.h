// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

#ifndef _APP_H
#define _APP_H

#define WIFI_EASY_CONFIG_DEMO
#define WIFI_EASY_CONFIG_DEMO_VERSION_NUMBER "1.0"

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#include <system_config.h>
#include <system_definitions.h>
#include "connector.h"

#include "app_serialflash.h"

#include "wdrv_mrf24wn_iwpriv.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus // Provide C++ Compatibility

extern "C"
{

#endif
    // DOM-IGNORE-END

    // *****************************************************************************
    // *****************************************************************************
    // Section: Type Definitions
    // *****************************************************************************
    // *****************************************************************************

#define TRUE 1
#define FALSE 0

/* SD */
#define DATA_BUFFER_ALIGN

/* WiFi */
#define WF_DISABLED WDRV_FUNC_DISABLED
#define WF_ENABLED WDRV_FUNC_ENABLED

#define WF_NETWORK_TYPE_INFRASTRUCTURE WDRV_NETWORK_TYPE_INFRASTRUCTURE
#define WF_NETWORK_TYPE_ADHOC WDRV_NETWORK_TYPE_ADHOC
#define WF_NETWORK_TYPE_P2P WDRV_NETWORK_TYPE_P2P
#define WF_NETWORK_TYPE_SOFT_AP WDRV_NETWORK_TYPE_SOFT_AP

#define WF_SECURITY_OPEN WDRV_SECURITY_OPEN
#define WF_SECURITY_WEP_40 WDRV_SECURITY_WEP_40
#define WF_SECURITY_WEP_104 WDRV_SECURITY_WEP_104
#define WF_SECURITY_WPA_WITH_KEY 0xff /* Unsupported */
#define WF_SECURITY_WPA_WITH_PASS_PHRASE WDRV_SECURITY_WPA_WITH_PASS_PHRASE
#define WF_SECURITY_WPA2_WITH_KEY 0xff /* Unsupported */
#define WF_SECURITY_WPA2_WITH_PASS_PHRASE WDRV_SECURITY_WPA2_WITH_PASS_PHRASE
#define WF_SECURITY_WPA_AUTO_WITH_KEY 0xff         /* Unsupported */
#define WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE 0xff /* Unsupported */
#define WF_SECURITY_WPS_PUSH_BUTTON WDRV_SECURITY_WPS_PUSH_BUTTON
#define WF_SECURITY_WPS_PIN WDRV_SECURITY_WPS_PIN

#define WF_DEFAULT_ADHOC_HIDDEN_SSID WDRV_DEFAULT_ADHOC_HIDDEN_SSID
#define WF_DEFAULT_ADHOC_BEACON_PERIOD WDRV_DEFAULT_ADHOC_BEACON_PERIOD
#define WF_DEFAULT_ADHOC_MODE WDRV_DEFAULT_ADHOC_MODE

#define WF_DEFAULT_POWER_SAVE WDRV_DEFAULT_POWER_SAVE

#define WF_WEP_KEY_INVALID 0xff

#define WF_ASSERT(condition, msg) ASSERT(condition, msg)

    typedef WDRV_SCAN_RESULT           WF_SCAN_RESULT;
    typedef WDRV_CONFIG_DATA           WF_CONFIG_DATA;
    typedef WDRV_DEVICE_INFO           WF_DEVICE_INFO;
    typedef WDRV_ADHOC_NETWORK_CONTEXT WF_ADHOC_NETWORK_CONTEXT;

    /* Wi-Fi Interface */

    /* It is intentionally declared this way to sync with DRV_WIFI_DEVICE_TYPE. */
    typedef enum
    {
        MRF24WG_MODULE = 2,
        MRF24WN_MODULE = 3,
    } MRF24W_MODULE_TYPE;

#define WIFI_INTERFACE_NUM 0
#define ETH_INTERFACE_NUM 1

#define LORA_SPREAD_SF7 0X02
#define LORA_SPREAD_SF8 0X04
#define LORA_SPREAD_SF9 0X08
#define LORA_SPREAD_SF10 0X10
#define LORA_SPREAD_SF11 0X20
#define LORA_SPREAD_SF12 0X40

#define LORA_BW_500K 0x01
#define LORA_BW_250K 0x02
#define LORA_BW_125K 0x03

#define LORA_MOD_LORA 0x10
#define LORA_MOD_FSK 0x20

#define LORA_TX_STATUS_UNKNOWN 0x00
#define LORA_TX_STATUS_DISABLED 0x01
#define LORA_TX_STATUS_READY 0x02
#define LORA_TX_STATUS_LOADED 0x03
#define LORA_TX_STATUS_EMITTING 0x04

#define UART_BUFF_LENGTH 300
#define LORA_RX_QUEUE_SIZE 30

#define LORA_BAND_ND 0x00
#define LORA_BAND_868 0x01
#define LORA_BAND_915 0x02

#define LORA_CRC_BAD 0x11
#define LORA_CRC_OK 0x10
#define LORA_NO_CRC 0x01

    typedef struct
    {
        uint8_t  rx_status;
        uint32_t frequency;
        uint8_t  if_chain;
        uint8_t  pkt_status;
        uint32_t timestamp;
        uint8_t  rf_chain;
        uint8_t  modulation;
        uint8_t  bandwidth;
        uint32_t datarate;
        uint8_t  coderate;
        int32_t  rssi;
        int32_t  snr_average;
        int32_t  snr_minimum;
        int32_t  snr_maximum;
        uint16_t crc;
        uint16_t payload_size;
        uint8_t  payload[256];
        uint8_t  uploadretry;
    } loraRXPacket;

    typedef struct
    {
        uint32_t frequency;
        uint8_t  tx_mode;
        uint32_t timestamp;
        uint8_t  rf_chain;
        uint8_t  tx_power;
        uint8_t  modulation;
        uint8_t  bandwidth;
        uint32_t datarate;
        uint8_t  coderate;
        bool     invert_polarity;
        uint8_t  frequency_deviation;
        uint8_t  preamble;
        bool     no_crc;
        bool     no_header;
        uint16_t payload_size;
        uint8_t  payload[248];
        uint8_t  uploadretry;
    } loraTXPacket;

    // *****************************************************************************
    /* Application states

      Summary:
        Application states enumeration

      Description:
        This enumeration defines the valid application states.  These states
        determine the behavior of the application at various times.
    */

    /* SD Card */
    typedef enum
    {
        APP_SDCARD_MOUNT_DISK = 0,
        APP_SDCARD_SET_CURRENT_DRIVE,
        APP_SDCARD_CREATE_DIRECTORY,
        APP_SDCARD_OPEN_FILE,
        APP_SDCARD_WRITE_TO_FILE,
        APP_SDCARD_CLOSE_FILE,
        APP_SDCARD_CALCULATE_SHA,
        APP_SDCARD_COMPARE_SHA,
        APP_SDCARD_UNMOUNT_DISK,
        APP_SDCARD_IDLE,
        APP_SDCARD_ERROR,
        APP_SDCARD_ERROR2
    } APP_STATES_SDCARD;

    /* Serial Flash */
    typedef enum
    {
        /* The application mounts the disk. */
        APP_SERIALFLASH_INIT = 0,
        APP_SERIALFLASH_WRITE_ENABLE,
        APP_SERIALFLASH_UNBLOCK_MEMORY,
        APP_SERIALFLASH_OPEN_SPI_FLASH_DRIVER,

        APP_SERIALFLASH_READ_MAGIC_BYTES_WIFI,
        APP_SERIALFLASH_WAIT_FOR_MAGIC_BYTES_WIFI,
        APP_SERIALFLASH_VERIFY_MAGIC_BYTES_WIFI,
        APP_SERIALFLASH_READ_MAGIC_BYTES_ACTIVATION_DATA,
        APP_SERIALFLASH_WAIT_FOR_MAGIC_BYTES_ACTIVATION_DATA,
        APP_SERIALFLASH_VERIFY_MAGIC_BYTES_ACTIVATION_DATA,
        APP_SERIALFLASH_READ_MAGIC_BYTES_FOTA,
        APP_SERIALFLASH_WAIT_FOR_MAGIC_BYTES_FOTA,
        APP_SERIALFLASH_VERIFY_MAGIC_BYTES_FOTA,

        APP_SERIALFLASH_READ_DATA,
        APP_SERIALFLASH_WAIT_FOR_READING_DATA,
        APP_SERIALFLASH_STORE_DATA,
        APP_SERIALFLASH_WAIT_FOR_STORING_DATA,
        APP_SERIALFLASH_VERIFY_STORED_DATA,

        APP_SERIALFLASH_BLOCK_ERASE,
        APP_SERIALFLASH_WAIT_FOR_BLOCK_ERASE,

        APP_SERIALFLASH_CHIP_ERASE,
        APP_SERIALFLASH_WAIT_FOR_CHIP_ERASE,

        APP_SERIALFLASH_IDLE,
        APP_SERIALFLASH_ERROR

    } APP_SERIALFLASH_STATES;

    /* LoRa */
    typedef enum
    {
        APP_LORA_INIT = 0,
        APP_LORA_WAIT_INIT_COMPLETE,
        APP_LORA_WAIT_FOR_APP,
        APP_LORA_CONFIG,
        APP_LORA_WAIT_TTN_CONFIG_COMPLETE,
        APP_LORA_OPERATIONAL
    } APP_STATES_LORA;

    /* BLE */
    typedef enum
    {
        APP_BLE_INIT = 0,
        APP_BLE_SEND,
        APP_BLE_RECEIVE,
        APP_BLE_IDLE
    } APP_STATES_BLE;

    /* MQTT */
    typedef enum
    {
        APP_TTNGWC_INIT,
        APP_TTNGWC_FETCH_SERVER_IP,
        APP_TTNGWC_FETCHING_SERVER_IP,
        APP_TTNGWC_RESET,
        APP_TTNGWC_SOCKET_SETUP,
        APP_TTNGWC_SOCKET_CHECK,
        APP_TTNGWC_SOCKET_WAIT_TLS,
        APP_TTNGWC_CONNECT,
        APP_TTNGWC_ERROR,
        APP_TTNGWC_IDLE,
        APP_TTNGWC_SEND_STATUS,
        APP_TTNGWC_SEND_PACKET,
        APP_TTNGWC_WAIT_ON_DNS
    } APP_STATES_MQTT;

    // *****************************************************************************
    /* Application Data

      Summary:
        Holds application data

      Description:
        This structure holds the application's data.

      Remarks:
        Application strings and buffers are be defined outside this structure.
     */

    typedef struct
    {
        /* The application's current states */
        bool mqtt_connection_state;
        bool selftest_enabled;
    } APP_DATA;

    /* FOTA */
    typedef enum
    {
        KEY,
        FIRMWARE
    } FOTA_FILETYPES;

    /* SD card */
    typedef struct
    {
        APP_STATES_SDCARD state;
        bool              is_present;
        bool              is_mounted;
    } APP_DATA_SDCARD;

    /* Serial Flash */
    typedef struct
    {
        APP_SERIALFLASH_STATES state;
        bool                   has_wifi_data;
        bool                   has_activation_data;
        bool                   has_fota_data;
        uint8_t                status_register;
        // uint8_t                                 configuration_register;
        uint8_t                              eventMap;
        DRV_HANDLE                           driverHandle;
        DRV_SST25VF064C_BLOCK_COMMAND_HANDLE commandHandle[3];
        uint8_t*                             sourceBuffer;
        uint8_t*                             targetBuffer;
        uint32_t                             address;
        uint16_t                             length;
        uint32_t                             image_length;
        uint32_t                             expected_image_length;
        CRYPT_SHA256_CTX                     sha256;
        uint8_t                              sha256_expected[FLASH_LENGTH_FOTA_DATA_SHA256];
    } APP_SERIALFLASH_DATA;

    typedef struct
    {
        uint8_t ssid[32 + 1]; // 32-byte SSID plus null terminator
        uint8_t networkType;
        uint8_t prevSSID[32 + 1]; // previous SSID
        uint8_t prevNetworkType;  // previous network type
        uint8_t wepKeyIndex;
        uint8_t securityMode;
        uint8_t securityKey[64 + 1]; // 64-byte key plus null terminator
        uint8_t securityKeyLen;      // number of bytes in security key (can be 0)
    } WF_REDIRECTION_CONFIG;

    /* LoRa */
    typedef struct
    {
        APP_STATES_LORA state;
        DRV_HANDLE      USARTHandle;
        // SX1301_CONF     LORAconf;

        uint8_t rx_uart_buffer[UART_BUFF_LENGTH]; // data received
        uint8_t tx_uart_buffer[UART_BUFF_LENGTH]; // data to send

        uint8_t      rxitems;
        uint8_t      txitems;
        uint8_t      rxread_pos;
        uint8_t      rxwrite_pos;
        uint8_t      txread_pos;
        uint8_t      txwrite_pos;
        loraRXPacket rx_queue[LORA_RX_QUEUE_SIZE];
        loraTXPacket tx_queue[LORA_RX_QUEUE_SIZE];
    } APP_DATA_LORA;

    /* BLE */
    typedef struct
    {
        APP_STATES_BLE state;
        DRV_HANDLE     USARTHandle;
    } APP_DATA_BLE;

    /* MQTT */
    typedef struct
    {
        APP_STATES_MQTT state;
        TTN*            ttn;
    } APP_DATA_MQTT;

    typedef struct IFChain
    {
        bool     enable;
        uint8_t  radio;
        int32_t  freqOffset;
        uint32_t bandwidth;
        uint8_t  spread_factor;
        uint32_t datarate;
    } IFCHAIN_CONF;

    typedef struct RFChain
    {
        bool    enable;
        int32_t freq;
    } RFCHAIN_CONF;

    typedef struct SX1301conf
    {
        bool         lorawan_public;
        RFCHAIN_CONF rfchain[2];
        IFCHAIN_CONF ifchain[10];
    } SX1301_CONF;

    typedef struct TTN_server
    {
        char     server_address[256];
        uint16_t serv_port_up;
        uint16_t serv_port_down;
        bool     serv_tls;
    } TTNSERVER_CONF;

    typedef struct gateway_conf
    {
        TTNSERVER_CONF ttn_servers[10];
    } GATEWAY_CONF;

    typedef struct
    {
        char code[10];
        char error[200];
    } ERROR;

    typedef struct
    {
        char id[100];
        char key[200];
        char default_account_server_url[256];
        char account_server_url[256];
        char frequency_plan[65];
        char frequency_plan_url[256];
        char firmware_url[256];
        bool auto_update;

        ERROR error;
    } CONFIGURATION;

    typedef struct
    {
        CONFIGURATION configuration;
        SX1301_CONF   configuration_sx1301;
        GATEWAY_CONF  configuration_gateway;

        // SD card
        bool          sdcard_is_present;
        SYS_FS_HANDLE fileHandle;

        uint8_t current_firmware_key[FLASH_LENGTH_FIRMWARE_DATA_SHA256];

        bool locked;                // lock flag for id, key and account url
        bool locked_for_first_time; // locked for first time flag
    } APP_GW_ACTIVATION_DATA;

    typedef struct
    {
        uint8_t ssid[FLASH_LENGTH_WIFI_DATA_SSID];
        uint8_t key[FLASH_LENGTH_WIFI_DATA_SECURITY_KEY];
        uint8_t conn_type;
        uint8_t sec_type;
        bool    valid;
    } APP_GW_WIFI_DATA;

    // *****************************************************************************
    // *****************************************************************************
    // Section: Application Callback Routines
    // *****************************************************************************
    // *****************************************************************************
    /* These routines are called by drivers when certain events occur.
     */

    // *****************************************************************************
    // *****************************************************************************
    // Section: Application Initialization and State Machine Functions
    // *****************************************************************************
    // *****************************************************************************

    /*******************************************************************************
      Function:
        void APP_Initialize ( void )

      Summary:
         MPLAB Harmony application initialization routine.

      Description:
        This function initializes the Harmony application.  It places the
        application in its initial state and prepares it to run so that its
        APP_Tasks function can be called.

      Precondition:
        All other system initialization routines should be called before calling
        this routine (in "SYS_Initialize").

      Parameters:
        None.

      Returns:
        None.

      Example:
        <code>
        APP_Initialize();
        </code>

      Remarks:
        This routine must be called from the SYS_Initialize function.
    */

    void APP_Initialize(void);
    void APP_SDCARD_Initialize(void);
    void APP_SERIALFLASH_Initialize(void);
    void APP_ETH_Initialize(void);
    void APP_WIFI_Initialize(void);
    void APP_LORA_Initialize(void);
    void APP_BLE_Initialize(void);

    void APP_SELFTEST_Initialize(void);

    /*******************************************************************************
      Function:
        void APP_Tasks ( void )

      Summary:
        MPLAB Harmony Demo application tasks function

      Description:
        This routine is the Harmony Demo application's tasks function.  It
        defines the application's state machine and core logic.

      Precondition:
        The system and application initialization ("SYS_Initialize") should be
        called before calling this.

      Parameters:
        None.

      Returns:
        None.

      Example:
        <code>
        APP_Tasks();
        </code>

      Remarks:
        This routine must be called from SYS_Tasks() routine.
     */

    void APP_Tasks(void);

    void APP_SDCARD_Tasks(void);
    void APP_SERIALFLASH_Tasks(void);
    void APP_ETH_Tasks(void);
    void APP_WIFI_Tasks(void);
    void APP_LORA_Tasks(void);
    void APP_BLE_Tasks(void);

    void APP_SELFTEST_Tasks(void);

    bool    GatewayIsOperational(void);
    uint8_t APP_MQTT_GET_STATE();
    bool    APP_WIFI_Has_LinkAP(void);
    bool    APP_WIFI_Has_LinkINFRA(void);
    bool    APP_ETH_Has_Link(void);

    bool selftest_isEnabled(void);
    void selftest_hasEthernet(bool status);
    void selftest_hasWifi(bool status);

    void APP_StatSet(uint8_t stat_num, bool state);

#endif /* _APP_H */

#undef SYS_DEBUG
#define SYS_DEBUG(level, fmt, ...) SYS_DEBUG_PRINT(level, fmt, ##__VA_ARGS__)

// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

/*******************************************************************************
 End of File
 */
