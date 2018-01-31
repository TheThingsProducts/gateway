/*******************************************************************************
  MRF24WG Driver Customization

  Summary:
    Module for Microchip TCP/IP Stack

  Description:
    -Provides access to MRF24WG Wi-Fi controller
    -Reference: MRF24WG Datasheet, IEEE 802.11 Standard
 *******************************************************************************/

/*******************************************************************************
File Name: drv_wifi_config.h
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

#ifndef _WF_CONFIG_H_
#define _WF_CONFIG_H_

#define APP_WIFI_TCPIP_WEB_SERVER_DEMO

#define APP_WIFI_TCPIP_WEB_SERVER_DEMO_VERSION "1.0"

/********************************************
 * Select a default network mode for demo.  *
 *                                          *
 * Web Server Demo:                         *
 *     DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE *
 ********************************************/
#define DRV_WIFI_DEFAULT_NETWORK_TYPE    DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE

/****************************************************************************************************************
 * Configure network type for Infrastructure mode.                                                              *
 *                                                                                                              *
 * Available security configuration:                                                                            *
 * DRV_WIFI_SECURITY_OPEN                      : No security                                                    *
 * DRV_WIFI_SECURITY_WEP_40                    : WEP Encryption using 40 bit keys                               *
 * DRV_WIFI_SECURITY_WEP_104                   : WEP Encryption using 104 bit keys                              *
 * DRV_WIFI_SECURITY_WPA_AUTO_WITH_PASS_PHRASE : WPA-PSK or WPA2-PSK where passphrase is given to MRF24W and it *
 *                                                calculates the binary key and connects.                       *
 * DRV_WIFI_SECURITY_WPS_PUSH_BUTTON           : WPS push button method                                         *
 * DRV_WIFI_SECURITY_WPS_PIN                   : WPS PIN method                                                 *
 ****************************************************************************************************************/
#if (DRV_WIFI_DEFAULT_NETWORK_TYPE == DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE)
    #define DRV_WIFI_DEFAULT_SECURITY_MODE       DRV_WIFI_SECURITY_OPEN
    #define DRV_WIFI_DEFAULT_SSID                "MicrochipDemoApp"       /* Set the SSID of the network we want to join. For WPS Push button set to "". */
    #define DRV_WIFI_DEFAULT_LIST_RETRY_COUNT    (DRV_WIFI_RETRY_FOREVER) /* Number (1..255) of times to try to connect to the SSID when using Infrastructure network type. */
    #define DRV_WIFI_DEFAULT_CHANNEL_LIST        {}                       /* channel list for domain - use default in module */
    #define DRV_WIFI_DEFAULT_POWER_SAVE          DRV_WIFI_DISABLED        /* DRV_WIFI_ENABLED or DRV_WIFI_DISABLED */
#endif

/*------------------------------------------------------*
 * Only used when security is DRV_WIFI_DEFAULT_WPS_PIN. *
 * An example PIN.                                      *
 * The last digit is the checksum of first 7 digits.    *
 * -----------------------------------------------------*/
#define DRV_WIFI_DEFAULT_WPS_PIN    "12390212"

/*----------------------------------------------------------------------------------*
 * MRF24W FW has a built-in connection manager, and it is enabled by default.       *
 * So if you want to run your own connection manager in host side,                  *
 * you should disable the module connection manager to avoid some possible conflict *
 * between the two.  These two APIs can be affected if you do not disable it.       *
 *   A) DRV_WIFI_Disconnect()                                                       *
 *   B) DRV_WIFI_Scan()                                                             *
 * These APIs will return failure when the conflict occurs.                         *
 *                                                                                  *
 * Note: This feature is current available with Infrastructure mode only.           *
 *----------------------------------------------------------------------------------*/
#if (DRV_WIFI_DEFAULT_NETWORK_TYPE == DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE)
    #define DRV_WIFI_MODULE_CONNECTION_MANAGER    DRV_WIFI_ENABLED
#endif

/*------------------------------------------------------------------------------*
 * This option gets the stack to save the credentials to the internal flash.    *
 * And the stored credentials can be used for next connection with the same AP. *
 *------------------------------------------------------------------------------*/
#if (DRV_WIFI_DEFAULT_SECURITY_MODE == DRV_WIFI_SECURITY_WPS_PUSH_BUTTON) || \
    (DRV_WIFI_DEFAULT_SECURITY_MODE == DRV_WIFI_SECURITY_WPS_PIN)
    #define DRV_WIFI_SAVE_WPS_CREDENTIALS    DRV_WIFI_DISABLED
#endif

#define DRV_WIFI_GRATUITOUS_ARP    DRV_WIFI_DISABLED

/*---------------------------------------------------------------------------------*
 * Default WEP keys used in DRV_WIFI_SECURITY_WEP_40 and DRV_WIFI_SECURITY_WEP_104 *
 * security mode.                                                                  *
 *---------------------------------------------------------------------------------*/
#if (DRV_WIFI_DEFAULT_SECURITY_MODE == DRV_WIFI_SECURITY_WEP_40)    \
                    ||                                              \
    (DRV_WIFI_DEFAULT_SECURITY_MODE == DRV_WIFI_SECURITY_WEP_104)

#define DRV_WIFI_DEFAULT_WEP_PHRASE    "WEP Phrase"

#if (DRV_WIFI_DEFAULT_SECURITY_MODE == DRV_WIFI_SECURITY_WEP_40)
// string containing 4 40-bit WEP keys -- corresponding to passphrase of "WEP Phrase"
#define DRV_WIFI_DEFAULT_WEP_KEY_40 "5AFB6C8E77"
#define DRV_WIFI_DEFAULT_WEP_KEYS_40 "\
\x5a\xfb\x6c\x8e\x77\
\xc1\x04\x49\xfd\x4e\
\x43\x18\x2b\x33\x88\
\xb0\x73\x69\xf4\x78"
// Do not indent above string as it will inject spaces.
#endif

#if (DRV_WIFI_DEFAULT_SECURITY_MODE == DRV_WIFI_SECURITY_WEP_104)
// string containing 4 104-bit WEP keys -- corresponding to passphrase of "WEP Phrase"
#define DRV_WIFI_DEFAULT_WEP_KEY_104 "90E96780C739409DA50034FCAA"
#define DRV_WIFI_DEFAULT_WEP_KEYS_104 "\
\x90\xe9\x67\x80\xc7\x39\x40\x9d\xa5\x00\x34\xfc\xaa\
\x77\x4a\x69\x45\xa4\x3d\x66\x63\xfe\x5b\x1d\xb9\xfd\
\x82\x29\x87\x4c\x9b\xdc\x6d\xdf\x87\xd1\xcf\x17\x41\
\xcc\xd7\x62\xde\x92\xad\xba\x3b\x62\x2f\x7f\xbe\xfb"
// Do not indent above string as it will inject spaces.
#endif

#endif /* WEP40 and WEP104 */

/*--------------------------------------------------*
 * Default Pass Phrase Used in WPA-PSK and WPA2-PSK *
 *--------------------------------------------------*/
#if (DRV_WIFI_DEFAULT_SECURITY_MODE == DRV_WIFI_SECURITY_WPA_AUTO_WITH_PASS_PHRASE)

#define DRV_WIFI_DEFAULT_PSK_PHRASE    "Microchip 802.11 Secret PSK Password"

#endif /* WPA-PSK and WPA2-PSK Passphrase */

//#define DRV_WIFI_SAVE_CONFIG_TO_MEMORY // If define this macro, system will save configuration data into MEMORY.
#if defined (DRV_WIFI_SAVE_CONFIG_TO_MEMORY)
 #define DRV_WIFI_MEMORY_INTERNAL_FLASH
 //#define DRV_WIFI_MEMORY_EXTERNAL_EEPROM

 #if defined(DRV_WIFI_MEMORY_INTERNAL_FLASH) && defined(DRV_WIFI_MEMORY_EXTERNAL_EEPROM)
  #error "we cannot define the DRV_WIFI_MEMORY_INTERNAL_FLASH and DRV_WIFI_MEMORY_EXTERNAL_EEPROM at same time"
 #endif
#endif

#if (DRV_WIFI_DEFAULT_NETWORK_TYPE != DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE)
 #if DRV_WIFI_MODULE_CONNECTION_MANAGER == DRV_WIFI_DISABLED
  #error "DRV_WIFI_MODULE_CONNECTION_MANAGER must be enabled if not in infrastructure mode"
 #endif
#endif

#endif /* _WF_CONFIG_H_ */
