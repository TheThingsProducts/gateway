// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#include "app.h"
#include "subsystem_controller.h"
#include "app_activation.h"
#include "app_frequencyplan.h"
#include "app_configuration.h"
#include "error_messages.h"

#define RECONNECT_DELAY_TIMEOUT 10 // seconds before reconnect

/* Load online config sequence states */
typedef enum
{
    STATE_NEEDS_MORE_CONFIG,
    STATE_ACTIVATING,
    //    STATE_ACTIVATING_WITH_KEY,
    STATE_ERASING_ACTIVATION_KEY,
    STATE_STORING_ACTIVATION_KEY,
    STATE_DOWNLOAD_LORA_CONFIG,
    STATE_LOCK_ACTIVATION,
    STATE_DOWNLOAD_FREQPLAN,
    STATE_START_LORA_MODULE,
    STATE_COMMUNICATION_ERROR,
    STATE_DONE
} STATE_t;

static void _changeState(STATE_t newState);

static STATE_t  _state;
static uint32_t reconnectDelayStartTick = 0;
static uint32_t freqDelayStartTick      = 0;
static uint32_t noIdBlinkStartTick      = 0;

// REVIEW: Try to avoid external variables
extern APP_GW_ACTIVATION_DATA appGWActivationData; // Used to read and write activation data from and to

bool SSMLoadOnlineConfig_IsDone(void)
{
    return _state == STATE_DONE;
}

bool SSMLoadOnlineConfig_HasCommunicationError(void)
{
    return _state == STATE_COMMUNICATION_ERROR;
}

bool SSMLoadOnlineConfig_NeedsMoreConfig(void)
{
    return _state == STATE_NEEDS_MORE_CONFIG;
}

void SSMLoadOnlineConfig_Enter(void)
{
    if(appGWActivationData.configuration.id[0] == '\0')
    {
        _changeState(STATE_NEEDS_MORE_CONFIG);
    }
    else if(appGWActivationData.configuration.key[0] == '\0')
    {
        _changeState(STATE_ACTIVATING);
    }
    else
    {
        _changeState(STATE_DOWNLOAD_LORA_CONFIG);
    }
}

void SSMLoadOnlineConfig_Leave(void)
{
    switch(_state)
    {
        case STATE_ACTIVATING:
            //        case STATE_ACTIVATING_WITH_KEY:
        case STATE_DOWNLOAD_LORA_CONFIG:
        case STATE_DOWNLOAD_FREQPLAN:
            APP_HTTP_Request_CloseIfNeeded();
            break;

        default:
            break;
    }
}

/**
 * Start the next readout, depending on availability
 */
static void _changeState(STATE_t newState)
{

    SYS_PRINT("\r\nCNFG: Load online user config state change to %d\r\n", newState);
    switch(newState)
    {
        case STATE_ACTIVATING:
            APP_Activation_Initialize();
            SYS_PRINT("\r\nCNFG: Activating gateway\r\n");
            break;

        case STATE_ERASING_ACTIVATION_KEY:
            APP_SERIALFLASH_EraseActivationData();
            break;

        case STATE_STORING_ACTIVATION_KEY:
            APP_SERIALFLASH_SaveActivationData(
                (char*)&appGWActivationData.configuration.id, (char*)&appGWActivationData.configuration.key,
                (char*)&appGWActivationData.configuration.account_server_url,
                false); // Only lock it after first time successful downloading LoRa config
            break;

        case STATE_DOWNLOAD_LORA_CONFIG:
            APP_Configuration_Initialize();
            reconnectDelayStartTick = SYS_TMR_TickCountGet();
            break;

        case STATE_LOCK_ACTIVATION:
            appGWActivationData.locked = true;
            APP_SERIALFLASH_LockActivationData();
            break;

        case STATE_DOWNLOAD_FREQPLAN:
            APP_FreqPlan_Initialize();
            freqDelayStartTick = SYS_TMR_TickCountGet();
            break;

        case STATE_START_LORA_MODULE:
            SYS_PRINT("\r\nCNFG: Configuring LoRa module\r\n");
            APP_LORA_SetStartEvent();
            break;

        case STATE_COMMUNICATION_ERROR:
            SYS_PRINT("\r\nCNFG: Communication ERROR\r\n");
            break;

        default:
            break;
    }
    _state = newState;
}

void SSMLoadOnlineConfig_Tasks(void)
{
    switch(_state)
    {
        case STATE_NEEDS_MORE_CONFIG:
            // TODO: what do we do then? Display on status page?
            break;

        case STATE_ACTIVATING:
        {
            if(APP_SERIALFLASH_IsReady())
            {
                APP_Activation_Tasks();

                switch(APP_Activation_State())
                { // REVIEW: don't check internal state, but via test function
                    case APP_ACTIVATION_BARE_DONE:
                        if(strcmp(appGWActivationData.configuration.key, "") == 0)
                        {
                            FATAL("Activation done, but key not filled");
                        }

                        SYS_PRINT("\r\nCNFG: Valid key, proceeding\r\n");
                        _changeState(STATE_ERASING_ACTIVATION_KEY);
                        break;

                    case APP_ACTIVATION_BARE_WAS_ACTIVATED:
                        // invalid gateway id, wipe id, start over.
                        SYS_PRINT("\r\nCNFG: Was already activated, wait for a key to be configured\r\n");
                        _changeState(STATE_NEEDS_MORE_CONFIG);
                        break;

                    case APP_ACTIVATION_BARE_NON_EXISTING_ID:
                        SYS_PRINT("\r\nCNFG: Non existing ID, please register this ID with The Things Network\r\n");
                        _changeState(STATE_NEEDS_MORE_CONFIG);
                        break;

                    case APP_ACTIVATION_BARE_ERROR:
                        SYS_PRINT("\r\nCNFG: Activation failed\r\n");
                        ErrorMessageWarning_Set(ERROR_MESSAGE_WARNING_HTTP_COMMUNICATION_FAILURE_ACTIVATION);
                        _changeState(STATE_COMMUNICATION_ERROR);
                        break;
                }
            }
            break;
        }

        case STATE_ERASING_ACTIVATION_KEY:
            if(APP_SERIALFLASH_IsReady())
            {
                _changeState(STATE_STORING_ACTIVATION_KEY);
            }
            break;

        case STATE_STORING_ACTIVATION_KEY:
            if(APP_SERIALFLASH_IsReady())
            {
                SYS_PRINT("\r\nCNFG: Gateway Activated\r\n");
                _changeState(STATE_DOWNLOAD_LORA_CONFIG);
            }
            break;

        case STATE_DOWNLOAD_LORA_CONFIG:
            APP_Configuration_Tasks();

            switch(APP_Configuration_State())
            { // REVIEW: don't check internal state, but via test function
                case APP_CONFIGURATION_DONE:
                    if(appGWActivationData.locked)
                    {
                        _changeState(STATE_DOWNLOAD_FREQPLAN);
                    }
                    else
                    {
                        _changeState(STATE_LOCK_ACTIVATION);
                    }
                    break;

                case APP_CONFIGURATION_INCOMPLETE:
                    _changeState(STATE_NEEDS_MORE_CONFIG);
                    break;

                case APP_CONFIGURATION_NON_EXISTING_ID:
                    // invalid gateway id, wipe id, start over.
                    SYS_PRINT("\r\nCNFG: Non existing gateway ID, wiping and starting over\r\n");
                    appGWActivationData.configuration.id[0]  = '\0';
                    appGWActivationData.configuration.key[0] = '\0';
                    _changeState(STATE_NEEDS_MORE_CONFIG);
                    break;

                case APP_CONFIGURATION_KEY_FAILURE:
                    // invalid gateway key, wipe key start over.
                    SYS_PRINT("\r\nCNFG: Invalid gateway key, wiping and starting over\r\n");
                    appGWActivationData.configuration.key[0] = '\0';
                    _changeState(STATE_ACTIVATING);
                    break;

                case APP_CONFIGURATION_SERVER_BUSY:
                    if(SYS_TMR_TickCountGet() - reconnectDelayStartTick >=
                       SYS_TMR_TickCounterFrequencyGet() *
                           RECONNECT_DELAY_TIMEOUT) // REVIEW: use something like isElapsed function
                    {
                        SYS_PRINT("\r\nCNFG: Server busy, retry\r\n");
                        _changeState(STATE_DOWNLOAD_LORA_CONFIG);
                    }
                    break;

                case APP_CONFIGURATION_ERROR:
                    SYS_PRINT("\r\nCNFG: Downloading gateway configuration failed\r\n");
                    _changeState(STATE_COMMUNICATION_ERROR);
                    ErrorMessageWarning_Set(ERROR_MESSAGE_WARNING_HTTP_COMMUNICATION_FAILURE_CONFIGURATION);
                    break;

                default:
                    break;
            }
            break;

        case STATE_LOCK_ACTIVATION:
            if(APP_SERIALFLASH_IsReady())
            {
                SYS_PRINT("\r\nCNFG: Gateway activation locked\r\n");
                _changeState(STATE_DOWNLOAD_FREQPLAN);
            }
            break;

        case STATE_DOWNLOAD_FREQPLAN:
            // Fetch lora module configuration
            if(SYS_TMR_TickCountGet() - freqDelayStartTick <= SYS_TMR_TickCounterFrequencyGet() * 2)
            {
                break; // take a little break after the settings fetch to be nice on the TTN server
            }

            APP_FreqPlan_Tasks();

            switch(APP_FreqPlan_State())
            { // REVIEW: don't check internal state, but via test function
                case APP_FREQPLAN_DONE:
                    _changeState(STATE_START_LORA_MODULE);
                    break;

                case APP_FREQPLAN_FAILURE:
                case APP_FREQPLAN_SERVER_BUSY:
                    SYS_PRINT("\r\nCNFG: Server busy/failure, retry\r\n");
                    _changeState(STATE_DOWNLOAD_FREQPLAN);
                    break;

                case APP_FREQPLAN_ERROR:
                    SYS_PRINT("\r\nCNFG: Downloading frequency plan failed\r\n");
                    ErrorMessageWarning_Set(ERROR_MESSAGE_WARNING_HTTP_COMMUNICATION_FAILURE_FREQPLAN);
                    _changeState(STATE_COMMUNICATION_ERROR);
                    break;
            }
            break;

        case STATE_START_LORA_MODULE:
            if(APP_LORA_GET_APP_STATE() ==
               APP_LORA_POLL_UART) // REVIEW: don't check internal state, but via test function
            {
                _changeState(STATE_DONE);
            }

        case STATE_COMMUNICATION_ERROR:
            break;

        case STATE_DONE:
            break;
    }
}
