/*******************************************************************************
  MRF24WG Configuration Data

  File Name:
    drv_wifi_config_data.c

  Summary:
    MRF24WG Configuration Data

  Description:
    - Stores/retrieves Wi-Fi configuration to/from non-volatile memory (NVM)
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

#include "drv_wifi_priv.h"

#include "drv_wifi_config_data.h"
#include "drv_wifi_debug_output.h"

#ifdef DRV_WIFI_CONFIG_MHC /* Used in MHC */
#define MhcWep40KeyLength 5
#define MhcWep104KeyLength 13
static uint8_t MhcWepKey[MhcWep104KeyLength] = {0}; /* This array is used to store WEP key when MHC is enabled */
#endif

#define DRV_WIFI_NVM_SPACE_MAGIC_NUMBER 0x5a5a5a5a

static DRV_WIFI_CONFIG_DATA s_wif_configData;
DRV_WIFI_CONFIG_DATA *p_wifi_configData = &s_wif_configData;

#if defined(DRV_WIFI_NVM_SPACE_ENABLE)
static DRV_HANDLE s_nvmHandle;

static bool DRV_WIFI_NVM_Read(uint8_t *buf, size_t size, uint32_t startAddr)
{
    DRV_NVM_COMMAND_HANDLE nvmCommandHandle;

    if (startAddr > DRV_NVM_MEDIA_SIZE * 1024)
        return false;

    s_nvmHandle = DRV_NVM_Open(0, DRV_IO_INTENT_READ);
    if (s_nvmHandle == DRV_HANDLE_INVALID) {
        DRV_NVM_Close(s_nvmHandle);
        s_nvmHandle = NULL;
        return false;
    }

    DRV_NVM_Read(s_nvmHandle, &nvmCommandHandle, buf, startAddr, size);
    if (nvmCommandHandle == DRV_NVM_COMMAND_HANDLE_INVALID) {
        DRV_NVM_Close(s_nvmHandle);
        s_nvmHandle = NULL;
        return false;
    }

    DRV_NVM_Close(s_nvmHandle);
    s_nvmHandle = NULL;
    return true;
}

/**
 * Now, this function can only be used to write Wi-Fi configuration. We didn't
 * extend its use cases more broadly because of memory saving reason.
 *
 * It explicitly erases a whole memory page but only writes back the Wi-Fi
 * configuration which occupies a very small chunk of memory. It doesn't read
 * and maintain any data before erasing. So all the previous data is lost.
 * However, this operation is safe under the assumption that the whole memory
 * page starting at startAddr is only managed by MRF24WG driver.
 *
 * If we want to extend this function's usage more in the future, the size of
 * static buffer it maintains should be increased to DRV_NVM_ROW_SIZE or even
 * DRV_NVM_PAGE_SIZE depends on necessity, and it should read data from memory
 * before erasing to prevent data loss.
 */
static bool DRV_WIFI_NVM_PageEraseConfigWrite(uint8_t *buf, size_t size, uint32_t startAddr)
{
    static DRV_WIFI_CONFIG_DATA nvmConfigBuf;
    SYS_FS_MEDIA_GEOMETRY *nvmMediaGeometry;
    DRV_NVM_COMMAND_HANDLE nvmCommandHandle;

    if (size > sizeof(nvmConfigBuf))
        return false;

    if (startAddr > DRV_NVM_MEDIA_SIZE * 1024)
        return false;

    s_nvmHandle = DRV_NVM_Open(0, DRV_IO_INTENT_WRITE);
    if (s_nvmHandle == DRV_HANDLE_INVALID) {
        DRV_NVM_Close(s_nvmHandle);
        s_nvmHandle = NULL;
        return false;
    }

    memcpy(&nvmConfigBuf, buf, size);

    nvmMediaGeometry = DRV_NVM_GeometryGet(s_nvmHandle);
    if (nvmMediaGeometry == NULL) {
        DRV_NVM_Close(s_nvmHandle);
        s_nvmHandle = NULL;
        return false;
    }

    DRV_NVM_EraseWrite(s_nvmHandle,
            &nvmCommandHandle,
            &nvmConfigBuf,
            nvmMediaGeometry->geometryTable[1].numBlocks * startAddr / (DRV_NVM_MEDIA_SIZE * 1024),
            1);

    if (nvmCommandHandle == DRV_NVM_COMMAND_HANDLE_INVALID) {
        DRV_NVM_Close(s_nvmHandle);
        s_nvmHandle = NULL;
        return false;
    }

    DRV_NVM_Close(s_nvmHandle);
    s_nvmHandle = NULL;

    return true;
}

static bool DRV_WIFI_NVM_PageErase(uint32_t startAddr)
{
    SYS_FS_MEDIA_GEOMETRY *nvmMediaGeometry;
    DRV_NVM_COMMAND_HANDLE nvmCommandHandle;

    if (startAddr > DRV_NVM_MEDIA_SIZE * 1024)
        return false;

    s_nvmHandle = DRV_NVM_Open(0, DRV_IO_INTENT_WRITE);
    if (s_nvmHandle == DRV_HANDLE_INVALID) {
        DRV_NVM_Close(s_nvmHandle);
        s_nvmHandle = NULL;
        return false;
    }

    nvmMediaGeometry = DRV_NVM_GeometryGet(s_nvmHandle);
    if (nvmMediaGeometry == NULL) {
        DRV_NVM_Close(s_nvmHandle);
        s_nvmHandle = NULL;
        return false;
    }

    DRV_NVM_Erase(s_nvmHandle,
            &nvmCommandHandle,
            nvmMediaGeometry->geometryTable[2].numBlocks * startAddr / (DRV_NVM_MEDIA_SIZE * 1024),
            1);

    DRV_NVM_Close(s_nvmHandle);
    s_nvmHandle = NULL;

    return true;
}

static bool LoadConfigFromMemory(void)
{
    uint32_t verifyFlag;

    if (s_nvmHandle) {
        SYS_CONSOLE_MESSAGE("Last Wi-Fi configuration NVM operation is not finished, please wait\r\n");
        return false;
    }

    DRV_WIFI_NVM_Read((uint8_t *)&verifyFlag, sizeof(uint32_t), DRV_WIFI_NVM_SPACE_ADDR);

    if (verifyFlag == DRV_WIFI_NVM_SPACE_MAGIC_NUMBER) {
        return DRV_WIFI_NVM_Read((uint8_t *)p_wifi_configData, sizeof(DRV_WIFI_CONFIG_DATA), DRV_WIFI_NVM_SPACE_ADDR);
    } else {
        SYS_CONSOLE_MESSAGE("No stored Wi-Fi configuration found in NVM\r\n");
        return false;
    }
}

static void SaveConfigToMemory(void)
{
    if (s_nvmHandle) {
        SYS_CONSOLE_MESSAGE("Last Wi-Fi configuration NVM operation is not finished, please wait\r\n");
        return;
    }

    DRV_WIFI_CONFIG_PARAMS(verifyFlag) = DRV_WIFI_NVM_SPACE_MAGIC_NUMBER;

    if (DRV_WIFI_NVM_PageEraseConfigWrite((uint8_t *)p_wifi_configData, sizeof(DRV_WIFI_CONFIG_DATA), DRV_WIFI_NVM_SPACE_ADDR))
        SYS_CONSOLE_MESSAGE("MRF24W: NVM save operation succeeded\r\n");
    else
        SYS_CONSOLE_MESSAGE("MRF24W: NVM save operation failed\r\n");
}

static void DeleteConfigInMemory(void)
{
    if (s_nvmHandle) {
        SYS_CONSOLE_MESSAGE("Last Wi-Fi configuration NVM operation is not finished, please wait\r\n");
        return;
    }

    if (DRV_WIFI_NVM_PageErase(DRV_WIFI_NVM_SPACE_ADDR))
        SYS_CONSOLE_MESSAGE("MRF24W: NVM delete operation succeeded\r\n");
    else
        SYS_CONSOLE_MESSAGE("MRF24W: NVM delete operation failed\r\n");
}

#else // !defined(DRV_WIFI_NVM_SPACE_ENABLE)

static DRV_WIFI_CONFIG_DATA s_wif_configDataBackup;

static bool LoadConfigFromMemory(void)
{
    if (s_wif_configDataBackup.verifyFlag == DRV_WIFI_NVM_SPACE_MAGIC_NUMBER) {
        memcpy((uint8_t *)p_wifi_configData, (uint8_t *)&s_wif_configDataBackup, sizeof(DRV_WIFI_CONFIG_DATA));
        return true;
    } else {
        SYS_CONSOLE_MESSAGE("No stored Wi-Fi configuration found\r\n");
        return false;
    }
}

static void SaveConfigToMemory(void)
{
    DRV_WIFI_CONFIG_PARAMS(verifyFlag) = DRV_WIFI_NVM_SPACE_MAGIC_NUMBER;
    memcpy((uint8_t *)&s_wif_configDataBackup, (uint8_t *)p_wifi_configData, sizeof(DRV_WIFI_CONFIG_DATA));
    SYS_CONSOLE_MESSAGE("MRF24W: Save operation succeeded\r\n");
}

static void DeleteConfigInMemory(void)
{
    memset(&s_wif_configDataBackup, 0, sizeof(DRV_WIFI_CONFIG_DATA));
    SYS_CONSOLE_MESSAGE("MRF24W: Delete operation succeeded\r\n");
}

#endif // #if defined(DRV_WIFI_NVM_SPACE_ENABLE)

bool DRV_WIFI_ConfigDataLoad(void)
{
    bool res = LoadConfigFromMemory();

    if (g_drv_wifi_priv.isDriverOpen)
        return res;

    if (!res) { // use hard-coded values in system_config.h
#ifdef DRV_WIFI_CONFIG_MHC
        uint8_t i, j;
#endif

        if (sizeof(DRV_WIFI_DEFAULT_SSID) - 1 > DRV_WIFI_MAX_SSID_LENGTH) {
            SYS_ERROR_PRINT(SYS_ERROR_ERROR, "SSID could not contain more than %u characters\r\n", DRV_WIFI_MAX_SSID_LENGTH);
            return false;
        }

        SYS_CONSOLE_MESSAGE("Using default Wi-Fi configuration\r\n");

        memcpy(DRV_WIFI_CONFIG_PARAMS(ssid), (const void *)DRV_WIFI_DEFAULT_SSID, sizeof(DRV_WIFI_DEFAULT_SSID) - 1);
        DRV_WIFI_CONFIG_PARAMS(ssidLen) = sizeof(DRV_WIFI_DEFAULT_SSID) - 1;
        DRV_WIFI_CONFIG_PARAMS(networkType) = DRV_WIFI_DEFAULT_NETWORK_TYPE;
        DRV_WIFI_CONFIG_PARAMS(securityMode) = DRV_WIFI_DEFAULT_SECURITY_MODE;

        switch (DRV_WIFI_CONFIG_PARAMS(securityMode)) {
        case DRV_WIFI_SECURITY_OPEN:
            memset(DRV_WIFI_CONFIG_PARAMS(securityKey), 0x00, sizeof(DRV_WIFI_CONFIG_PARAMS(securityKey)));
            DRV_WIFI_CONFIG_PARAMS(securityKeyLen) = 0;
            break;

        case DRV_WIFI_SECURITY_WEP_40:
#ifdef DRV_WIFI_CONFIG_MHC
            // Check the DRV_WIFI_DEFAULT_WEP_KEYS_40 length
            DRV_WIFI_ASSERT(sizeof(DRV_WIFI_DEFAULT_WEP_KEY_40) - 1 == 10, "WEP40 (64-bit) key must contain exact 10 characters");
            // Convert character array to hex number array
            for (i = 0, j = 0; i < MhcWep40KeyLength; i++) {
                uint8_t temp = 0;
                DRV_WIFI_ASSERT(j < sizeof(DRV_WIFI_DEFAULT_WEP_KEY_40) - 1, "");
                if ((uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_40 + j * sizeof(uint8_t)) >= (uint8_t)'0' && (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_40 + j * sizeof(uint8_t)) <= (uint8_t)'9')
                    temp = (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_40 + j * sizeof(uint8_t)) - (uint8_t)'0';
                else if ((uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_40 + j * sizeof(uint8_t)) >= (uint8_t)'A' && (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_40 + j * sizeof(uint8_t)) <= (uint8_t)'F')
                    temp = (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_40 + j * sizeof(uint8_t)) - (uint8_t)'A' + 10;
                else if ((uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_40 + j * sizeof(uint8_t)) >= (uint8_t)'a' && (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_40 + j * sizeof(uint8_t)) <= (uint8_t)'f')
                    temp = (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_40 + j * sizeof(uint8_t)) - (uint8_t)'a' + 10;
                else
                    DRV_WIFI_ASSERT(false, "WEP 40(64-bit) key contains invalid hex digit");
                MhcWepKey[i] = temp;
                j++;
                DRV_WIFI_ASSERT(j < sizeof(DRV_WIFI_DEFAULT_WEP_KEY_40) - 1, "");
                if ((uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_40 + j * sizeof(uint8_t)) >= (uint8_t)'0' && (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_40 + j * sizeof(uint8_t)) <= (uint8_t)'9')
                    temp = (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_40 + j * sizeof(uint8_t)) - (uint8_t)'0';
                else if ((uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_40 + j * sizeof(uint8_t)) >= (uint8_t)'A' && (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_40 + j * sizeof(uint8_t)) <= (uint8_t)'F')
                    temp = (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_40 + j * sizeof(uint8_t)) - (uint8_t)'A' + 10;
                else if ((uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_40 + j * sizeof(uint8_t)) >= (uint8_t)'a' && (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_40 + j * sizeof(uint8_t)) <= (uint8_t)'f')
                    temp = (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_40 + j * sizeof(uint8_t)) - (uint8_t)'a' + 10;
                else
                    DRV_WIFI_ASSERT(false, "WEP40 (64-bit) key contains invalid hex digit");
                MhcWepKey[i] = MhcWepKey[i] * 16 + temp;
                j++;
            }
            DRV_WIFI_CONFIG_PARAMS(wepKeyIndex) = DRV_WIFI_DEFAULT_WEP_KEY_INDEX;
            memcpy(DRV_WIFI_CONFIG_PARAMS(securityKey), (const void *)MhcWepKey, MhcWep40KeyLength);
            DRV_WIFI_CONFIG_PARAMS(securityKeyLen) = MhcWep40KeyLength;
            DRV_WIFI_CONFIG_PARAMS(wepKeyType) = DRV_WIFI_DEFAULT_WEP_KEY_TYPE;
#else /* DRV_WIFI_CONFIG_MHC is not defined */
            if (sizeof(DRV_WIFI_DEFAULT_WEP_KEYS_40) - 1 > DRV_WIFI_MAX_SECURITY_KEY_LENGTH) {
                SYS_ERROR_PRINT(SYS_ERROR_ERROR,
                    "\r\nCurrent length of DRV_WIFI_DEFAULT_WEP_KEYS_40 is %u, it must not be longer than %u\r\n",
                    sizeof(DRV_WIFI_DEFAULT_WEP_KEYS_40) - 1, DRV_WIFI_MAX_SECURITY_KEY_LENGTH);
                return false;
            }
            DRV_WIFI_CONFIG_PARAMS(wepKeyIndex) = DRV_WIFI_DEFAULT_WEP_KEY_INDEX;
            memcpy(DRV_WIFI_CONFIG_PARAMS(securityKey), (const void *)DRV_WIFI_DEFAULT_WEP_KEYS_40, sizeof(DRV_WIFI_DEFAULT_WEP_KEYS_40) - 1);
            DRV_WIFI_CONFIG_PARAMS(securityKeyLen) = sizeof(DRV_WIFI_DEFAULT_WEP_KEYS_40) - 1;
            DRV_WIFI_CONFIG_PARAMS(wepKeyType) = DRV_WIFI_DEFAULT_WEP_KEY_TYPE;
#endif /* DRV_WIFI_CONFIG_MHC */
            break;

        case DRV_WIFI_SECURITY_WEP_104:
#ifdef DRV_WIFI_CONFIG_MHC
            // Check the DRV_WIFI_DEFAULT_WEP_KEYS_104 length
            DRV_WIFI_ASSERT(sizeof(DRV_WIFI_DEFAULT_WEP_KEY_104) - 1 == 26, "WEP104 (128-bit) key must contain exact 26 characters");
            // Convert character array to hex number array
            for (i = 0, j = 0; i < MhcWep104KeyLength; i++) {
                uint8_t temp = 0;
                DRV_WIFI_ASSERT(j < sizeof(DRV_WIFI_DEFAULT_WEP_KEY_104) - 1, "");
                if ((uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_104 + j * sizeof(uint8_t)) >= (uint8_t)'0' && (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_104 + j * sizeof(uint8_t)) <= (uint8_t)'9')
                    temp = (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_104 + j * sizeof(uint8_t)) - (uint8_t)'0';
                else if ((uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_104 + j * sizeof(uint8_t)) >= (uint8_t)'A' && (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_104 + j * sizeof(uint8_t)) <= (uint8_t)'F')
                    temp = (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_104 + j * sizeof(uint8_t)) - (uint8_t)'A' + 10;
                else if ((uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_104 + j * sizeof(uint8_t)) >= (uint8_t)'a' && (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_104 + j * sizeof(uint8_t)) <= (uint8_t)'f')
                    temp = (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_104 + j * sizeof(uint8_t)) - (uint8_t)'a' + 10;
                else
                    DRV_WIFI_ASSERT(false, "WEP104 (128-bit) key contains invalid hex digit");
                MhcWepKey[i] = temp;
                j++;
                DRV_WIFI_ASSERT(j < sizeof(DRV_WIFI_DEFAULT_WEP_KEY_104) - 1, "");
                if ((uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_104 + j * sizeof(uint8_t)) >= (uint8_t)'0' && (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_104 + j * sizeof(uint8_t)) <= (uint8_t)'9')
                    temp = (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_104 + j * sizeof(uint8_t)) - (uint8_t)'0';
                else if ((uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_104 + j * sizeof(uint8_t)) >= (uint8_t)'A' && (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_104 + j * sizeof(uint8_t)) <= (uint8_t)'F')
                    temp = (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_104 + j * sizeof(uint8_t)) - (uint8_t)'A' + 10;
                else if ((uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_104 + j * sizeof(uint8_t)) >= (uint8_t)'a' && (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_104 + j * sizeof(uint8_t)) <= (uint8_t)'f')
                    temp = (uint8_t)*(DRV_WIFI_DEFAULT_WEP_KEY_104 + j * sizeof(uint8_t)) - (uint8_t)'a' + 10;
                else
                    DRV_WIFI_ASSERT(false, "WEP104 (128-bit) key contains invalid hex digit");
                MhcWepKey[i] = MhcWepKey[i] * 16 + temp;
                j++;
            }
            DRV_WIFI_CONFIG_PARAMS(wepKeyIndex) = DRV_WIFI_DEFAULT_WEP_KEY_INDEX;
            memcpy(DRV_WIFI_CONFIG_PARAMS(securityKey), (const void *)MhcWepKey, MhcWep104KeyLength);
            DRV_WIFI_CONFIG_PARAMS(securityKeyLen) = MhcWep104KeyLength;
            DRV_WIFI_CONFIG_PARAMS(wepKeyType) = DRV_WIFI_DEFAULT_WEP_KEY_TYPE;
#else /* DRV_WIFI_CONFIG_MHC is not defined */
            if (sizeof(DRV_WIFI_DEFAULT_WEP_KEYS_104) - 1 > DRV_WIFI_MAX_SECURITY_KEY_LENGTH) {
                SYS_ERROR_PRINT(SYS_ERROR_ERROR,
                    "\r\nCurrent length of DRV_WIFI_DEFAULT_WEP_KEYS_104 is %u, it must not be longer than %u\r\n",
                    sizeof(DRV_WIFI_DEFAULT_WEP_KEYS_104) - 1, DRV_WIFI_MAX_SECURITY_KEY_LENGTH);
                return false;
            }
            DRV_WIFI_CONFIG_PARAMS(wepKeyIndex) = DRV_WIFI_DEFAULT_WEP_KEY_INDEX;
            memcpy(DRV_WIFI_CONFIG_PARAMS(securityKey), (const void *)DRV_WIFI_DEFAULT_WEP_KEYS_104, sizeof(DRV_WIFI_DEFAULT_WEP_KEYS_104) - 1);
            DRV_WIFI_CONFIG_PARAMS(securityKeyLen) = sizeof(DRV_WIFI_DEFAULT_WEP_KEYS_104) - 1;
            DRV_WIFI_CONFIG_PARAMS(wepKeyType) = DRV_WIFI_DEFAULT_WEP_KEY_TYPE;
#endif /* DRV_WIFI_CONFIG_MHC */
            break;

        case DRV_WIFI_SECURITY_WPA_AUTO_WITH_PASS_PHRASE:
        case DRV_WIFI_SECURITY_WPA_WITH_PASS_PHRASE:
        case DRV_WIFI_SECURITY_WPA2_WITH_PASS_PHRASE:
            if (sizeof(DRV_WIFI_DEFAULT_PSK_PHRASE) - 1 < DRV_WIFI_MIN_WPA_PASS_PHRASE_LENGTH)
            {
                SYS_ERROR_PRINT(SYS_ERROR_ERROR,
                        "\r\nCurrent length of DRV_WIFI_DEFAULT_PSK_PHRASE is %u, it must not be shorter than %u\r\n",
                        sizeof(DRV_WIFI_DEFAULT_PSK_PHRASE) - 1, DRV_WIFI_MIN_WPA_PASS_PHRASE_LENGTH);
                return false;
            }
            else if (sizeof(DRV_WIFI_DEFAULT_PSK_PHRASE) - 1 > DRV_WIFI_MAX_WPA_PASS_PHRASE_LENGTH)
            {
                SYS_ERROR_PRINT(SYS_ERROR_ERROR,
                        "\r\nCurrent length of DRV_WIFI_DEFAULT_PSK_PHRASE is %u, it must not be longer than %u\r\n",
                        sizeof(DRV_WIFI_DEFAULT_PSK_PHRASE) - 1, DRV_WIFI_MAX_WPA_PASS_PHRASE_LENGTH);
                return false;
            }
            memcpy(DRV_WIFI_CONFIG_PARAMS(securityKey), (const void *)DRV_WIFI_DEFAULT_PSK_PHRASE, sizeof(DRV_WIFI_DEFAULT_PSK_PHRASE) - 1);
            DRV_WIFI_CONFIG_PARAMS(securityKeyLen) = sizeof(DRV_WIFI_DEFAULT_PSK_PHRASE) - 1;
            break;

        case DRV_WIFI_SECURITY_WPS_PUSH_BUTTON:
            memset(DRV_WIFI_CONFIG_PARAMS(securityKey), 0x00, sizeof(DRV_WIFI_CONFIG_PARAMS(securityKey)));
            DRV_WIFI_CONFIG_PARAMS(securityKeyLen) = 0;
            break;

        case DRV_WIFI_SECURITY_WPS_PIN:
            if (sizeof(DRV_WIFI_DEFAULT_WPS_PIN) - 1 > DRV_WIFI_MAX_SECURITY_KEY_LENGTH - 1) {
                SYS_ERROR_PRINT(SYS_ERROR_ERROR,
                    "\r\nCurrent length of DRV_WIFI_DEFAULT_WPS_PIN is %u, it must not be longer than %u\r\n",
                    sizeof(DRV_WIFI_DEFAULT_WPS_PIN) - 1, DRV_WIFI_MAX_SECURITY_KEY_LENGTH - 1);
                return false;
            }
            memcpy(DRV_WIFI_CONFIG_PARAMS(securityKey), (const void *)DRV_WIFI_DEFAULT_WPS_PIN, sizeof(DRV_WIFI_DEFAULT_WPS_PIN) - 1);
            DRV_WIFI_CONFIG_PARAMS(securityKeyLen) = sizeof(DRV_WIFI_DEFAULT_WPS_PIN) - 1;
            break;

        default:
            DRV_WIFI_ASSERT(false, "Invalid security mode");
            break;
        }
    }

    return true;
}

void DRV_WIFI_ConfigDataSave(void)
{
    SaveConfigToMemory();
}

void DRV_WIFI_ConfigDataDelete(void)
{
    DeleteConfigInMemory();
}

// DOM-IGNORE-END
