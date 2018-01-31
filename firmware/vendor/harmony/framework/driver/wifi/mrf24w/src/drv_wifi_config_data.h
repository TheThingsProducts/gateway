/*******************************************************************************
  MRF24WG Configuration Data

  File Name:
    drv_wifi_config_data.h

  Summary:
    MRF24WG Configuration Data

  Description:
    MRF24WG Configuration Data
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc. All rights reserved.

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

#ifndef _DRV_WIFI_CONFIG_DATA_H
#define _DRV_WIFI_CONFIG_DATA_H

// DOM-IGNORE-BEGIN
#ifdef __cplusplus // Provide C++ Compatibility
    extern "C" {
#endif
// DOM-IGNORE-END

//============================================================================
//                                  Data Types
//============================================================================
typedef struct __attribute__((__packed__))
{
    uint32_t verifyFlag; // 0x00000000: empty;    0xffffffff: empty;    0x5a5a5a5a: verified.
    uint16_t wpsCredSaveFlag;
    uint8_t networkType;
    uint8_t ssid[DRV_WIFI_MAX_SSID_LENGTH];
    uint8_t ssidLen;
    uint8_t wepKeyType;
    uint8_t wepKeyIndex;
    uint8_t securityMode; // DRV_WIFI_SECURITY_OPEN or one of the other security modes
    uint8_t securityKey[DRV_WIFI_MAX_SECURITY_KEY_LENGTH]; // Wi-Fi security key, or passphrase
    uint8_t securityKeyLen; // number of bytes in security key (can be 0)
} DRV_WIFI_CONFIG_DATA;

//============================================================================
//                                  Globals
//============================================================================
extern DRV_WIFI_CONFIG_DATA *p_wifi_configData;
#define DRV_WIFI_CONFIG_PARAMS(params) p_wifi_configData->params

// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

#endif // _DRV_WIFI_CONFIG_DATA_H

// DOM-IGNORE-END
