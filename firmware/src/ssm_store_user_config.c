// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#include "app.h"
#include "subsystem_controller.h"
#include "app_activation.h"
#include "app_frequencyplan.h"
#include "app_configuration.h"

#define RECONNECT_DELAY_TIMEOUT 10 // seconds before reconnect

/* Store user config in serial flash update sequence */
typedef enum
{
    STATE_WAIT_SF_READY,
    STATE_ERASING_ACTIVATION,
    STATE_WRITING_ACTIVATION,
    STATE_ERASING_WIFI,
    STATE_WRITING_WIFI,
    STATE_WAITING_FOR_REDIRECT,
    STATE_READY_FOR_REBOOT,
    STATE_FATAL_ERROR,
} STATE_t;

static void _changeState(STATE_t newState);

static STATE_t  _state;
static uint32_t reconnectDelayStartTick = 0;
static uint32_t freqDelayStartTick      = 0;

// REVIEW: Try not to use external variables but interface via functions
extern bool                   g_redirect_signal;
extern APP_GW_ACTIVATION_DATA appGWActivationData;
extern WF_CONFIG_DATA g_wifi_cfg; // Store WiFi data directly from struct used communicating with the WiFi driver (note
                                  // that it is used for scanning etc as well)

bool SSMStoreUserConfig_IsReadyForReboot()
{
    return _state == STATE_READY_FOR_REBOOT;
}

bool SSMStoreUserConfig_HasFatalError()
{
    return _state == STATE_FATAL_ERROR;
}

void SSMStoreUserConfig_Enter(void)
{
    _changeState(STATE_WAIT_SF_READY);
}

/**
 * Start the next readout, depending on availability
 */
static void _changeState(STATE_t newState)
{

    SYS_PRINT("\r\nCNFG: Store user config state change to %d\r\n", newState);
    _state = newState;
}

void SSMStoreUserConfig_Tasks(void)
{
    switch(_state)
    {
        case STATE_WAIT_SF_READY:
            if(APP_SERIALFLASH_IsReady())
            {
                if(appGWActivationData.locked)
                {
                    APP_SERIALFLASH_EraseWifiData();
                    _changeState(STATE_ERASING_WIFI);
                }
                else
                {
                    APP_SERIALFLASH_EraseActivationData();
                    _changeState(STATE_ERASING_ACTIVATION);
                }
            }
            break;

        case STATE_ERASING_ACTIVATION:
            if(APP_SERIALFLASH_HasError())
            {
                SYS_PRINT("\r\nFLASH: Error erasing activation data\r\n");
                _changeState(STATE_FATAL_ERROR);
            }
            if(APP_SERIALFLASH_IsReady())
            {
                APP_SERIALFLASH_SaveActivationData(
                    (char*)&appGWActivationData.configuration.id, (char*)&appGWActivationData.configuration.key,
                    (char*)&appGWActivationData.configuration.account_server_url, appGWActivationData.locked);
                _changeState(STATE_WRITING_ACTIVATION);
            }
            break;

        case STATE_WRITING_ACTIVATION:
            if(APP_SERIALFLASH_HasError())
            {
                SYS_PRINT("\r\nFLASH: Error writing activation data\r\n");
                _changeState(STATE_FATAL_ERROR);
            }
            if(APP_SERIALFLASH_IsReady())
            {
                APP_SERIALFLASH_EraseWifiData();
                _changeState(STATE_ERASING_WIFI);
            }
            break;

        case STATE_ERASING_WIFI:
            if(APP_SERIALFLASH_HasError())
            {
                SYS_PRINT("\r\nFLASH: Error erasing wifi data\r\n");
                _changeState(STATE_FATAL_ERROR);
            }
            if(APP_SERIALFLASH_IsReady())
            {
                APP_SERIALFLASH_SaveWifiData(g_wifi_cfg.networkType, g_wifi_cfg.securityMode, g_wifi_cfg.ssid,
                                             g_wifi_cfg.securityKey);
                _changeState(STATE_WRITING_WIFI);
            }
            break;

        case STATE_WRITING_WIFI:
            if(APP_SERIALFLASH_HasError())
            {
                SYS_PRINT("\r\nFLASH: Error writing wifi data\r\n");
                _changeState(STATE_FATAL_ERROR);
            }
            if(APP_SERIALFLASH_IsReady())
            {
                _changeState(STATE_WAITING_FOR_REDIRECT);
            }
            break;

        case STATE_WAITING_FOR_REDIRECT:
            if(g_redirect_signal == true)
            {
                _changeState(STATE_READY_FOR_REBOOT);
            }
            break;
    }
}
