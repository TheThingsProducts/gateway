// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#include "app.h"
#include "app_serialflash.h"

/* Load config in serial flash update sequence */
typedef enum
{
    STATE_WAIT_SF_READY,
    STATE_WAIT_FW_DATA,
    STATE_WAIT_WIFI_DATA,
    STATE_WAIT_GW_DATA,
    STATE_WAIT_UDP_CONFIG,
    STATE_WAIT_FOTA_OVR,
    STATE_FATAL_ERROR,
    STATE_DONE
} STATE_t;

static STATE_t                _state;
extern APP_GW_WIFI_DATA       appWifiData;
extern APP_GW_ACTIVATION_DATA appGWActivationData;
extern UDP_GW_CONF            g_udp_gw_conf;
extern FOTA_OVERRIDE_CONF     g_fota_override_conf;

bool SSMLoadConfig_IsBusy(void)
{
    return _state != STATE_DONE && _state != STATE_FATAL_ERROR;
}

bool SSMLoadConfig_HasFatalError(void)
{
    return _state == STATE_FATAL_ERROR;
}

void SSMLoadConfig_Enter(void)
{
    _state = STATE_WAIT_SF_READY;
}

/**
 * Start the next readout, depending on availability (REVIEW: check of magic bytes could be integrated here instead of
 * in the serial driver).
 */
void startNext(void)
{
    switch(_state)
    {
        case STATE_WAIT_SF_READY: // After SF is ready start with FW data
            // Start reading the first sector, with firmware data
            APP_SERIALFLASH_LoadFirmwareData();
            _state = STATE_WAIT_FW_DATA;
            break;

        case STATE_WAIT_FW_DATA: // After FW data start with WiFi data (if available)
            if(APP_SERIALFLASH_HasWifiData())
            {
                // magic byte is present
                APP_SERIALFLASH_LoadWifiData();
                _state = STATE_WAIT_WIFI_DATA;
                break;
            }
            else
            {
                appWifiData.valid = FALSE;
            }
            // intentional fall through
        case STATE_WAIT_WIFI_DATA: // After WiFi data start with activation data (if available)
            if(APP_SERIALFLASH_HasActivationData())
            {
                // magic byte is present
                APP_SERIALFLASH_LoadActivationData();
                _state = STATE_WAIT_GW_DATA;
                break;
            }
            // intentional fall through
        case STATE_WAIT_GW_DATA: // After activation data load UDP config (if available)
            if(APP_SERIALFLASH_HasUDPConfig())
            {
                APP_SERIALFLASH_LoadUDPConfig();
                _state = STATE_WAIT_UDP_CONFIG;
                break;
            }
            // intentional fall through
        case STATE_WAIT_UDP_CONFIG: // After UDP config load FOTA override (if available)
            if(APP_SERIALFLASH_HasFOTAOverride())
            {
                APP_SERIALFLASH_LoadFOTAOverride();
                _state = STATE_WAIT_FOTA_OVR;
                break;
            }
            // intentional fall through
        default:
            _state = STATE_DONE;
    }
}

void SSMLoadConfig_Tasks(void)
{
    switch(_state)
    {
        case STATE_WAIT_SF_READY:
            if(APP_SERIALFLASH_IsReady() || APP_SERIALFLASH_HasError())
            {
                startNext();
            }
            break;

        case STATE_WAIT_FW_DATA:
            if(APP_SERIALFLASH_HasError())
            {
                _state = STATE_FATAL_ERROR;
            }
            else if(APP_SERIALFLASH_IsReady())
            {
                // Load the sha256 checksum of the current firmware
                APP_SERIALFLASH_GetFirmwareChecksum(appGWActivationData.current_firmware_key);
                PRINT_ARRAY_UINT8(appGWActivationData.current_firmware_key,
                                  sizeof(appGWActivationData.current_firmware_key), "CNFG", "Firmware HASH (sha256)");

                startNext();
            }
            break;

        case STATE_WAIT_WIFI_DATA:
            if(APP_SERIALFLASH_HasError())
            {
                _state = STATE_FATAL_ERROR;
            }
            else if(APP_SERIALFLASH_IsReady())
            {
                APP_SERIALFLASH_GetSSID(appWifiData.ssid);
                APP_SERIALFLASH_GetSecurityKey(appWifiData.key);
                APP_SERIALFLASH_GetNetworkType(&appWifiData.conn_type);
                APP_SERIALFLASH_GetSecurityMode(&appWifiData.sec_type);
                SYS_PRINT("CNFG: WiFi SSID:      %s\r\n", appWifiData.ssid);
#ifdef __DEBUG
                SYS_PRINT("CNFG: WiFi key:       %s\r\n", appWifiData.key);
#else
                SYS_PRINT("CNFG: WiFi key:       %s\r\n", "***");
#endif
                SYS_PRINT("CNFG: WiFi conn_type: %d\r\n", appWifiData.conn_type);
                SYS_PRINT("CNFG: WiFi sec_type:  %d\r\n", appWifiData.sec_type);
                if(appWifiData.ssid[0] != 0 &&                                           // has an id
                   appWifiData.conn_type != 0 &&                                         // has a connection type
                   (appWifiData.sec_type == WF_SECURITY_OPEN || appWifiData.key[0] != 0) // has a key or is open
                )
                { // then we mark data as valid
                    appWifiData.valid = TRUE;
                }

                startNext();
            }
            break;

        case STATE_WAIT_GW_DATA:
            if(APP_SERIALFLASH_HasError())
            {
                _state = STATE_FATAL_ERROR;
            }
            else if(APP_SERIALFLASH_IsReady())
            {
                APP_SERIALFLASH_GetGatewayID(appGWActivationData.configuration.id);
                SYS_PRINT("CNFG: Gateway ID:         %s\r\n", appGWActivationData.configuration.id);
                APP_SERIALFLASH_GetGatewayKey(appGWActivationData.configuration.key);
#ifdef __DEBUG
                SYS_PRINT("CNFG: Gateway Key:        %s\r\n", appGWActivationData.configuration.key);
#else
                SYS_PRINT("CNFG: Gateway Key:        %s\r\n", "***");
#endif
                APP_SERIALFLASH_GetAccountServerURL(appGWActivationData.configuration.account_server_url);
                SYS_PRINT("CNFG: Account Server URL: %s\r\n", appGWActivationData.configuration.account_server_url);
                appGWActivationData.locked = APP_SERIALFLASH_IsLocked();
                SYS_PRINT("CNFG: Locked:             %s\r\n", appGWActivationData.locked ? "true" : "false");
                appGWActivationData.locked_for_first_time =
                    appGWActivationData.locked ? FALSE : TRUE;
                SYS_PRINT("CNFG: Locked first time:  %s\r\n",
                          appGWActivationData.locked_for_first_time ? "true" : "false");

                startNext();
            }
            break;

        case STATE_WAIT_UDP_CONFIG:
            if(APP_SERIALFLASH_HasError())
            {
                _state = STATE_FATAL_ERROR;
            }
            else if(APP_SERIALFLASH_IsReady())
            {
                APP_SERIALFLASH_GetUDPConfig(&g_udp_gw_conf);
                SYS_PRINT("CNFG: UDP server: %s up:%u dn:%u\r\n", g_udp_gw_conf.server_address,
                          g_udp_gw_conf.port_up, g_udp_gw_conf.port_down);
                startNext();
            }
            break;

        case STATE_WAIT_FOTA_OVR:
            if(APP_SERIALFLASH_HasError())
            {
                _state = STATE_FATAL_ERROR;
            }
            else if(APP_SERIALFLASH_IsReady())
            {
                APP_SERIALFLASH_GetFOTAOverride(&g_fota_override_conf);
                SYS_PRINT("CNFG: FOTA override: %s url:%s\r\n",
                          g_fota_override_conf.override_enabled ? "enabled" : "disabled",
                          g_fota_override_conf.fota_url);
                startNext();
            }
            break;
    }
}
