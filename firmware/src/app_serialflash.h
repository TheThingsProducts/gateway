// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#ifndef _APP_SERIALFLASH_H /* Guard against multiple inclusion */
#define _APP_SERIALFLASH_H

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "app.h"
#include "crypto/crypto.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C"
{
#endif

    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */

#define FLASH_MEMORY_SIZE ((8 * (8 * 1024)) + (2 * (32 * 1024)) + (126 * (64 * 1024))) // 8 * 8KB, 2 * 32KB, 126 * 64KB
#define FLASH_PAGE_SIZE 256
#define FLASH_SECTOR_SIZE 4096

#define FLASH_SECTOR_FIRMWARE_DATA 0
#define FLASH_SECTOR_WIFI_DATA 1
#define FLASH_SECTOR_ACTIVATION_DATA 2
#define FLASH_SECTOR_FOTA_DATA 3
#define FLASH_SECTOR_FOTA_IMAGE 4

#define FLASH_LENGTH_FIRMWARE_DATA_SHA256 CRYPT_SHA256_DIGEST_SIZE

#define FLASH_LENGTH_WIFI_DATA_MAGIC_BYTES 4
#define FLASH_LENGTH_WIFI_DATA_NETWORK_TYPE 1
#define FLASH_LENGTH_WIFI_DATA_SECURITY_MODE 1
#define FLASH_LENGTH_WIFI_DATA_SSID (32 + 1)
#define FLASH_LENGTH_WIFI_DATA_SECURITY_KEY (64 + 1)

#define FLASH_LENGTH_ACTIVATION_DATA_MAGIC_BYTES 4
#define FLASH_LENGTH_ACTIVATION_DATA_GATEWAY_ID 100
#define FLASH_LENGTH_ACTIVATION_DATA_GATEWAY_KEY 200
#define FLASH_LENGTH_ACTIVATION_DATA_ACCOUNT_SERVER_URL 255
#define FLASH_LENGTH_ACTIVATION_DATA_LOCKED 1

#define FLASH_LENGTH_FOTA_DATA_MAGIC_BYTES 4
#define FLASH_LENGTH_FOTA_DATA_IMAGE_LENGTH 4
#define FLASH_LENGTH_FOTA_DATA_SHA256 CRYPT_SHA256_DIGEST_SIZE

#define FLASH_LENGTH_FIRMWARE_DATA FLASH_LENGTH_FIRMWARE_DATA_SHA256
#define FLASH_LENGTH_WIFI_DATA                                                                                         \
    (FLASH_LENGTH_WIFI_DATA_MAGIC_BYTES + FLASH_LENGTH_WIFI_DATA_NETWORK_TYPE + FLASH_LENGTH_WIFI_DATA_SECURITY_MODE + \
     FLASH_LENGTH_WIFI_DATA_SSID + FLASH_LENGTH_WIFI_DATA_SECURITY_KEY)
#define FLASH_LENGTH_ACTIVATION_DATA                                                              \
    (FLASH_LENGTH_ACTIVATION_DATA_MAGIC_BYTES + FLASH_LENGTH_ACTIVATION_DATA_GATEWAY_ID +         \
     FLASH_LENGTH_ACTIVATION_DATA_GATEWAY_KEY + FLASH_LENGTH_ACTIVATION_DATA_ACCOUNT_SERVER_URL + \
     FLASH_LENGTH_ACTIVATION_DATA_LOCKED)
#define FLASH_LENGTH_FOTA_DATA \
    (FLASH_LENGTH_FOTA_DATA_MAGIC_BYTES + FLASH_LENGTH_FOTA_DATA_IMAGE_LENGTH + FLASH_LENGTH_FOTA_DATA_SHA256)

#define FLASH_ADDRESS_FIRMWARE_DATA (FLASH_SECTOR_FIRMWARE_DATA * FLASH_SECTOR_SIZE)
#define FLASH_ADDRESS_FIRMWARE_DATA_SHA256 FLASH_ADDRESS_FIRMWARE_DATA

#define FLASH_ADDRESS_WIFI_DATA (FLASH_SECTOR_WIFI_DATA * FLASH_SECTOR_SIZE)
#define FLASH_ADDRESS_WIFI_DATA_MAGIC_BYTES FLASH_ADDRESS_WIFI_DATA
#define FLASH_ADDRESS_WIFI_DATA_NETWORK_TYPE (FLASH_ADDRESS_WIFI_DATA_MAGIC_BYTES + FLASH_LENGTH_WIFI_DATA_MAGIC_BYTES)
#define FLASH_ADDRESS_WIFI_DATA_SECURITY_MODE \
    (FLASH_ADDRESS_WIFI_DATA_NETWORK_TYPE + FLASH_LENGTH_WIFI_DATA_NETWORK_TYPE)
#define FLASH_ADDRESS_WIFI_DATA_SSID (FLASH_ADDRESS_WIFI_DATA_SECURITY_MODE + FLASH_LENGTH_WIFI_DATA_SECURITY_MODE)
#define FLASH_ADDRESS_WIFI_DATA_SECURITY_KEY (FLASH_ADDRESS_WIFI_DATA_SSID + FLASH_LENGTH_WIFI_DATA_SSID)

#define FLASH_ADDRESS_ACTIVATION_DATA (FLASH_SECTOR_ACTIVATION_DATA * FLASH_SECTOR_SIZE)
#define FLASH_ADDRESS_ACTIVATION_DATA_MAGIC_BYTES FLASH_ADDRESS_ACTIVATION_DATA
#define FLASH_ADDRESS_ACTIVATION_DATA_GATEWAY_ID \
    (FLASH_ADDRESS_ACTIVATION_DATA_MAGIC_BYTES + FLASH_LENGTH_ACTIVATION_DATA_MAGIC_BYTES)
#define FLASH_ADDRESS_ACTIVATION_DATA_GATEWAY_KEY \
    (FLASH_ADDRESS_ACTIVATION_DATA_GATEWAY_ID + FLASH_LENGTH_ACTIVATION_DATA_GATEWAY_ID)
#define FLASH_ADDRESS_ACTIVATION_DATA_ACCOUNT_SERVER_URL \
    (FLASH_ADDRESS_ACTIVATION_DATA_GATEWAY_KEY + FLASH_LENGTH_ACTIVATION_DATA_GATEWAY_KEY)
#define FLASH_ADDRESS_ACTIVATION_DATA_LOCKED \
    (FLASH_ADDRESS_ACTIVATION_DATA_ACCOUNT_SERVER_URL + FLASH_LENGTH_ACTIVATION_DATA_ACCOUNT_SERVER_URL)

#define FLASH_ADDRESS_FOTA_DATA (FLASH_SECTOR_FOTA_DATA * FLASH_SECTOR_SIZE)
#define FLASH_ADDRESS_FOTA_DATA_MAGIC_BYTES FLASH_ADDRESS_FOTA_DATA
#define FLASH_ADDRESS_FOTA_DATA_IMAGE_LENGTH (FLASH_ADDRESS_FOTA_DATA_MAGIC_BYTES + FLASH_LENGTH_FOTA_DATA_MAGIC_BYTES)
#define FLASH_ADDRESS_FOTA_DATA_SHA256 (FLASH_ADDRESS_FOTA_DATA_IMAGE_LENGTH + FLASH_LENGTH_FOTA_DATA_IMAGE_LENGTH)

#define FLASH_ADDRESS_FOTA_IMAGE (FLASH_SECTOR_FOTA_IMAGE * FLASH_SECTOR_SIZE)

    // *****************************************************************************
    // *****************************************************************************
    // Section: Data Types
    // *****************************************************************************
    // *****************************************************************************

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************

    bool APP_SERIALFLASH_IsReady(void);
    bool APP_SERIALFLASH_HasError(void);

    bool APP_SERIALFLASH_HasWifiData(void);
    bool APP_SERIALFLASH_HasActivationData(void);
    bool APP_SERIALFLASH_HasFOTAData(void);

    void APP_SERIALFLASH_SaveFirmwareData(uint8_t* sha256);
    void APP_SERIALFLASH_LoadFirmwareData(void);
    void APP_SERIALFLASH_EraseFirmwareData(void);
    void APP_SERIALFLASH_GetFirmwareChecksum(uint8_t* data);

    void APP_SERIALFLASH_SaveWifiData(uint8_t networktype, uint8_t securitymode, char* ssid, char* securitykey);
    void APP_SERIALFLASH_LoadWifiData(void);
    void APP_SERIALFLASH_EraseWifiData(void);
    void APP_SERIALFLASH_GetNetworkType(uint8_t* data);
    void APP_SERIALFLASH_GetSecurityMode(uint8_t* data);
    void APP_SERIALFLASH_GetSSID(char* data);
    void APP_SERIALFLASH_GetSecurityKey(char* data);

    void APP_SERIALFLASH_SaveActivationData(char* id, char* key, char* url, bool locked);
    void APP_SERIALFLASH_LockActivationData(void);
    void APP_SERIALFLASH_LoadActivationData(void);
    void APP_SERIALFLASH_EraseActivationData(void);
    void APP_SERIALFLASH_GetGatewayID(char* data);
    void APP_SERIALFLASH_GetGatewayKey(char* data);
    void APP_SERIALFLASH_GetAccountServerURL(char* data);
    bool APP_SERIALFLASH_IsLocked(void);

    void APP_SERIALFLASH_LoadFOTAData(void);
    void APP_SERIALFLASH_GetFOTAChecksum(uint8_t* data);

    void APP_SERIALFLASH_InitFOTA(uint32_t image_length);
    void APP_SERIALFLASH_SaveFOTAChecksum(uint8_t* sha256);
    void APP_SERIALFLASH_SaveFOTAImage(uint8_t* data, uint16_t length);
    void APP_SERIALFLASH_FinalizeFOTA(void);
    void APP_SERIALFLASH_EraseFOTAData(void);
    void APP_SERIALFLASH_EraseFOTA(void);

    void APP_SERIALFLASH_EraseChip(void);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _APP_SERIALFLASH_H */

/* *****************************************************************************
 End of File
 */
