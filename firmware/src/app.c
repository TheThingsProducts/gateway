// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#include "app.h"
#include "app_activation.h"
#include "app_configuration.h"
#include "app_frequencyplan.h"
#include "subsystem_controller.h"
#include "app_http_ttn.h"
#include "app_serialflash.h"
#include "app_ota.h"
#include "helper_wdt.h"
#include "app_mqtt.h"
#include "ssm_button.h"
#include "error_messages.h"

#define FATAL_ERROR_REBOOT_TIMEOUT 10      // seconds till reboot after entering the error state
#define FIRMWARE_CHECK_TIMEOUT (24 * 3600) // one day before recheck if new firmware is available
#define BLINK_FREQ_FAST 10
#define BLINK_FREQ_SLOW 2

typedef enum
{
    APP_STATE_MOUNT_FS,
    APP_STATE_LOAD_CONFIG,
    APP_STATE_WAIT_FOR_INTERNET,
    APP_STATE_LOAD_ONLINE_CONFIG,
    APP_STATE_DOWNLOAD_FIRMWARE,
    APP_STATE_OPERATIONAL,
    APP_STATE_WAIT_FOR_INTERNET_WHILE_OPERATIONAL,
    APP_STATE_STORE_USER_CONFIG,
    APP_STATE_ERASE_ACTI_CONFIG,
    APP_STATE_ERASE_WIFI_CONFIG,
    APP_STATE_REBOOT,
    APP_STATE_ERROR,
    APP_STATE_SELFTEST
} STATE_t;

typedef enum
{
    ERROR_NONE,
    ERROR_ILLEGAL_STATE_CHANGE,
    ERROR_TCPIP_INIT,
    ERROR_SF_ACCESS,
} ERROR_t;

typedef struct
{
    bool hasWifi;
    bool hasEthernet;
} ConnState_t;

static void _changeState(STATE_t newState);
static void _setFatalError(ERROR_t error);
static void _statOn(void);
static void _statOff(void);
static void _statSet(uint8_t stat_num, bool state);
static void _statToggle(uint8_t stat_num);

/*
 * SubStateMachine prototype functions
 */
bool SSMLoadConfig_IsBusy(void);
bool SSMLoadConfig_HasFatalError(void);
void SSMLoadConfig_Enter(void);
void SSMLoadConfig_Tasks(void);
void SSMWaitForInternet_Initialize(void);
bool SSMWaitForInternet_HasInternet(void);
bool SSMWaitForInternet_IsAPOnly(void);
void SSMWaitForInternet_Enter(void);
void SSMWaitForInternet_Tasks(void);
bool SSMLoadOnlineConfig_IsDone(void);
bool SSMLoadOnlineConfig_HasCommunicationError(void);
bool SSMLoadOnlineConfig_NeedsMoreConfig(void);
void SSMLoadOnlineConfig_Enter(void);
void SSMLoadOnlineConfig_Leave(void);
void SSMLoadOnlineConfig_Tasks(void);
bool SSMStoreUserConfig_IsReadyForReboot();
bool SSMStoreUserConfig_HasFatalError();
void SSMStoreUserConfig_Enter(void);
void SSMStoreUserConfig_Tasks(void);

void PingCallbackFunction(TCPIP_NET_HANDLE hNetIf, IPV4_ADDR* remoteIP, void* data);
bool jumperPlaced();
bool hasNetwork();
void rememberNetwork(void);
bool hasNetworkChanged(void);

static STATE_t  _state;
static ERROR_t  _error;
static uint32_t errorBlinkStartTick    = 0;
static uint32_t errorStartTick         = 0;
static uint32_t firmwareCheckStartTick = 0;
static APP_DATA appData;
static bool     LEDstate          = OFF;
static uint8_t  ledButton_counter = 0;

static ConnState_t rememberedNetworkState = {0};
static uint32_t    ledBinkStartTick       = 0;
static uint32_t    buttonReleaseStartTick = 0;

APP_GW_ACTIVATION_DATA appGWActivationData = {0};
APP_GW_WIFI_DATA       appWifiData         = {0};

bool button_was_pressed = false;

static SYS_TMR_HANDLE delayHandle;

WF_CONFIG_DATA g_wifi_cfg;
bool           g_config_changed  = false;
bool           g_redirect_signal = false;

void APP_Initialize(void)
{
    if(jumperPlaced())
    {
        SYS_PRINT("MAIN: SELFTEST ENABLED\r\n\r\n");
        statOff();
        wifiSet(ON);
        ethernetSet(ON);
        bleSet(ON);
        loraSet(ON);
        sdSet(ON);
        APP_SELFTEST_Initialize();
        APP_ETH_Initialize();
        APP_WIFI_Initialize();
        APP_WIFI_AP_MODE();
        appData.selftest_enabled = true;
        _state                   = APP_STATE_SELFTEST;
        return;
    }

    appData.selftest_enabled = false;

    char* id  = "";
    char* key = "";

    memcpy(appGWActivationData.configuration.id, id, strlen(id) + 1);
    memcpy(appGWActivationData.configuration.key, key, strlen(key) + 1);

    char* default_asvrurl = "https://account.thethingsnetwork.org/";

    size_t url_len = sizeof(appGWActivationData.configuration.default_account_server_url);
    strncpy(appGWActivationData.configuration.default_account_server_url, default_asvrurl, url_len);
    appGWActivationData.configuration.default_account_server_url[url_len - 1] = '\0';

    url_len = sizeof(appGWActivationData.configuration.account_server_url);
    strncpy(appGWActivationData.configuration.account_server_url, default_asvrurl, url_len);
    appGWActivationData.configuration.account_server_url[url_len - 1] = '\0';

    statOff();
    APP_SDCARD_Initialize();
    APP_SERIALFLASH_Initialize();

    APP_ETH_Initialize();
    APP_WIFI_Initialize();
    SSMWaitForInternet_Initialize();

    APP_BLE_Initialize();
    APP_MQTT_Initialize();

    /* Place the App state machine in its initial state. */
    _state = APP_STATE_MOUNT_FS;
    _error = ERROR_NONE;

    SYS_PRINT("\r\nMAIN: Initialisation complete\r\n");
}

static void _changeState(STATE_t newState)
{
    // TODO check illegal transitions
    //_setFatalError(ERROR_ILLEGAL_STATE_CHANGE);

    // onLeave
    SYS_PRINT("\r\nMAIN: Leaving state %d\r\n", _state);
    switch(_state)
    {
        case APP_STATE_LOAD_CONFIG:
            break;
        case APP_STATE_LOAD_ONLINE_CONFIG:
            SSMLoadOnlineConfig_Leave(); // Make sure all connections are closed
            break;
        case APP_STATE_DOWNLOAD_FIRMWARE:
            APP_OTA_Leave(); // Make sure all connections are closed
            firmwareCheckStartTick = SYS_TMR_TickCountGet();
            break;
        case APP_STATE_OPERATIONAL:
            appData.mqtt_connection_state = false;
            _statSet(LED_MQTT, OFF);
            APP_MQTT_Reset();
            break;
        case APP_STATE_WAIT_FOR_INTERNET_WHILE_OPERATIONAL:
            break;
        case APP_STATE_STORE_USER_CONFIG:
            break;
        case APP_STATE_ERASE_WIFI_CONFIG:
            break;
        case APP_STATE_ERROR:
            break;
        default:
            break;
    }

    _state = newState;

    // onEnter
    SYS_PRINT("MAIN: Entering state %d\r\n", _state);
    switch(newState)
    {
        case APP_STATE_LOAD_CONFIG:
            SSMLoadConfig_Enter();
            break;
        case APP_STATE_WAIT_FOR_INTERNET:
            ledBinkStartTick = SYS_TMR_TickCountGet();
            SSMWaitForInternet_Enter();
            _statSet(LED_POWER, ON);
            _statSet(LED_INTERNET, OFF); // will be flashing
            _statSet(LED_ACTIVATION, OFF);
            _statSet(LED_MQTT, OFF);
            _statSet(LED_ACTIVITY, OFF);
            break;
        case APP_STATE_LOAD_ONLINE_CONFIG:
            ledBinkStartTick = SYS_TMR_TickCountGet();
            _statSet(LED_POWER, ON);
            _statSet(LED_INTERNET, ON);
            _statSet(LED_ACTIVATION, OFF); // will be flashing
            _statSet(LED_MQTT, OFF);
            _statSet(LED_ACTIVITY, OFF);
            SSMLoadOnlineConfig_Enter();
            break;
        case APP_STATE_DOWNLOAD_FIRMWARE:
            ledBinkStartTick = SYS_TMR_TickCountGet();
            _statSet(LED_POWER, ON); // will be flashing
            _statSet(LED_INTERNET, OFF);
            _statSet(LED_ACTIVATION, OFF);
            _statSet(LED_MQTT, OFF);
            _statSet(LED_ACTIVITY, OFF);
            APP_OTA_Enter();
            break;
        case APP_STATE_OPERATIONAL:
            ledBinkStartTick = SYS_TMR_TickCountGet();
            _statSet(LED_POWER, ON);
            _statSet(LED_INTERNET, ON);
            _statSet(LED_ACTIVATION, ON);
            _statSet(LED_MQTT, OFF); // will be flashing, and kept on once MQTT connection is made
            _statSet(LED_ACTIVITY, OFF);
            rememberNetwork();
            break;
        case APP_STATE_WAIT_FOR_INTERNET_WHILE_OPERATIONAL:
            ledBinkStartTick = SYS_TMR_TickCountGet();
            _statSet(LED_POWER, ON);
            _statSet(LED_INTERNET, OFF); // will be flashing
            _statSet(LED_ACTIVATION, ON);
            _statSet(LED_MQTT, OFF);
            _statSet(LED_ACTIVITY, OFF);
            SSMWaitForInternet_Enter();
            break;
        case APP_STATE_STORE_USER_CONFIG:
            SSMStoreUserConfig_Enter();
            _statSet(LED_POWER, ON);
            _statSet(LED_INTERNET, OFF);   // will be flashing
            _statSet(LED_ACTIVATION, OFF); // will be flashing
            _statSet(LED_MQTT, OFF);
            _statSet(LED_ACTIVITY, OFF);
            break;

        case APP_STATE_ERASE_ACTI_CONFIG:
            APP_SERIALFLASH_EraseActivationData();
            ledBinkStartTick = SYS_TMR_TickCountGet();
            _statSet(LED_POWER, ON);
            //_statSet(LED_INTERNET, );
            _statSet(LED_ACTIVATION, OFF); // will be flashing
            _statSet(LED_MQTT, OFF);
            _statSet(LED_ACTIVITY, OFF);
            break;

        case APP_STATE_ERASE_WIFI_CONFIG:
            APP_SERIALFLASH_EraseWifiData();
            ledBinkStartTick = SYS_TMR_TickCountGet();
            _statSet(LED_POWER, ON);
            _statSet(LED_INTERNET, OFF); // will be flashing
            // _statSet(LED_ACTIVATION, );
            _statSet(LED_MQTT, OFF);
            _statSet(LED_ACTIVITY, OFF);
            break;

        case APP_STATE_REBOOT:
            RebootWithMessage("Application reboot");
            break;

        case APP_STATE_ERROR:
            _statOff();
            errorStartTick = errorBlinkStartTick = SYS_TMR_TickCountGet();
            SYS_PRINT("\r\nMAIN: ERROR %d, soon reboot\r\n", _error);
            break;
        default:
            break;
    }
}

static void _setFatalError(ERROR_t error)
{
    _error = error;
    ErrorMessageFatal_StoreApplicationError(error);
    _changeState(APP_STATE_ERROR);
}

static void _statOn(void)
{
    if(!SSMButton_IsPressed())
    {
        statOn();
    }
}

static void _statOff(void)
{
    if(!SSMButton_IsPressed())
    {
        statOff();
    }
}

static void _statSet(uint8_t stat_num, bool state)
{
    if(!SSMButton_IsPressed())
    {
        statSet(stat_num, state);
    }
}

static void _statToggle(uint8_t stat_num)
{
    if(!SSMButton_IsPressed())
    {
        statToggle(stat_num);
    }
}

void APP_Tasks(void)
{
    // Events
    if(!appData.selftest_enabled)
    {
        if(g_config_changed)
        {
            _changeState(APP_STATE_STORE_USER_CONFIG);
            g_config_changed = false;
        }
        if(SSMButton_IsPressed())
        {
            switch(ledButton_counter)
            {
                case 5:
                    statSet(LED5, ON);
                case 4:
                    statSet(LED4, ON);
                case 3:
                    statSet(LED3, ON);
                case 2:
                    statSet(LED2, ON);
                case 1:
                    statSet(LED1, ON);
                    break;
                case 0:
                    statOff();
                    break;
            }
            if(SSMButton_GetStartTick() > 0)
            {
                if(SYS_TMR_TickCountGet() - SSMButton_GetStartTick() >=
                   SYS_TMR_TickCounterFrequencyGet() * (ledButton_counter + 1))
                {
                    if(ledButton_counter < 5)
                    {
                        ledButton_counter++;
                    }
                }
            }
        }
        if(SSMButton_WasPressed10Milliseconds())
        {
            SSMButton_Reset();
            buttonReleaseStartTick = 0;
            _changeState(APP_STATE_REBOOT);
        }
        if(SSMButton_WasPressed2Seconds())
        {
            SSMButton_Reset();
            buttonReleaseStartTick = 0;
            _changeState(APP_STATE_ERASE_WIFI_CONFIG);
        }
        if(SSMButton_WasPressed5Seconds())
        {
            SSMButton_Reset();
            buttonReleaseStartTick = 0;
            _changeState(APP_STATE_ERASE_ACTI_CONFIG);
        }
    }

    // Decisions
    switch(_state)
    {
        case APP_STATE_MOUNT_FS:
            if(SYS_FS_Mount(SYS_FS_NVM_VOL, LOCAL_WEBSITE_PATH_FS, MPFS2, 0, NULL) == 0)
            {
                _changeState(APP_STATE_LOAD_CONFIG);
            }
            break;

        case APP_STATE_LOAD_CONFIG:
            if(!SSMLoadConfig_IsBusy())
            {
                if(SSMLoadConfig_HasFatalError())
                {
                    _setFatalError(ERROR_SF_ACCESS);
                }
                else
                {
                    _changeState(APP_STATE_WAIT_FOR_INTERNET);
                }
            }
            break;

        case APP_STATE_WAIT_FOR_INTERNET:
            if(SSMWaitForInternet_IsAPOnly())
            {
                if(SYS_TMR_TickCountGet() - ledBinkStartTick >= SYS_TMR_TickCounterFrequencyGet() / BLINK_FREQ_FAST)
                {
                    ledBinkStartTick = SYS_TMR_TickCountGet();
                    _statToggle(LED_INTERNET);
                }
            }
            else
            {
                if(SYS_TMR_TickCountGet() - ledBinkStartTick >= SYS_TMR_TickCounterFrequencyGet() / BLINK_FREQ_SLOW)
                {
                    ledBinkStartTick = SYS_TMR_TickCountGet();
                    _statToggle(LED_INTERNET);
                }
            }
            if(SSMWaitForInternet_HasInternet())
            {
                _changeState(APP_STATE_LOAD_ONLINE_CONFIG);
            }
            break;

        case APP_STATE_LOAD_ONLINE_CONFIG:
            if(SSMLoadOnlineConfig_NeedsMoreConfig())
            {
                if(SYS_TMR_TickCountGet() - ledBinkStartTick >= SYS_TMR_TickCounterFrequencyGet() / BLINK_FREQ_FAST)
                {
                    ledBinkStartTick = SYS_TMR_TickCountGet();
                    _statToggle(LED_ACTIVATION);
                }
            }
            else
            {
                if(SYS_TMR_TickCountGet() - ledBinkStartTick >= SYS_TMR_TickCounterFrequencyGet() / BLINK_FREQ_SLOW)
                {
                    ledBinkStartTick = SYS_TMR_TickCountGet();
                    _statToggle(LED_ACTIVATION);
                }
            }

            if(!hasNetwork() || SSMLoadOnlineConfig_HasCommunicationError())
            {
                _changeState(APP_STATE_WAIT_FOR_INTERNET);
            }
            else if(SSMLoadOnlineConfig_IsDone())
            {
                // only firmware download check when we are not in activation procedure
                if(appGWActivationData.locked_for_first_time == TRUE)
                {
                    SYS_PRINT("MAIN: Skipping firmware check at boot when activating\r\n");
                    _changeState(APP_STATE_OPERATIONAL);
                }
                else
                {
                    _changeState(APP_STATE_DOWNLOAD_FIRMWARE);
                }
            }
            break;

        case APP_STATE_STORE_USER_CONFIG:
            if(SYS_TMR_TickCountGet() - ledBinkStartTick >= SYS_TMR_TickCounterFrequencyGet() / BLINK_FREQ_FAST)
            {
                ledBinkStartTick = SYS_TMR_TickCountGet();
                _statToggle(LED_INTERNET);
                _statToggle(LED_ACTIVATION);
            }
            if(SSMStoreUserConfig_IsReadyForReboot())
            {
                _changeState(APP_STATE_REBOOT);
            }
            if(SSMStoreUserConfig_HasFatalError())
            {
                _setFatalError(ERROR_SF_ACCESS);
            }
            break;

        case APP_STATE_ERASE_ACTI_CONFIG:
            if(SYS_TMR_TickCountGet() - ledBinkStartTick >= SYS_TMR_TickCounterFrequencyGet() / BLINK_FREQ_FAST)
            {
                ledBinkStartTick = SYS_TMR_TickCountGet();
                _statToggle(LED_ACTIVATION);
            }
            if(APP_SERIALFLASH_IsReady())
            {
                _statSet(LED_ACTIVATION, OFF);
                SYS_PRINT("\r\nMAIN: Erased activation config\r\n");
                _changeState(APP_STATE_ERASE_WIFI_CONFIG);
            }
            break;

        case APP_STATE_ERASE_WIFI_CONFIG:
            if(SYS_TMR_TickCountGet() - ledBinkStartTick >= SYS_TMR_TickCounterFrequencyGet() / BLINK_FREQ_FAST)
            {
                ledBinkStartTick = SYS_TMR_TickCountGet();
                _statToggle(LED_INTERNET);
            }
            if(APP_SERIALFLASH_IsReady())
            {
                _statSet(LED_INTERNET, OFF);
                SYS_PRINT("\r\nMAIN: Erased WiFi config, REBOOT\r\n");
                _changeState(APP_STATE_REBOOT); // TODO: Switch to wait for internet after making LoRa driver able to
                                                // restart again
            }
            break;

        case APP_STATE_DOWNLOAD_FIRMWARE:
        {
            if(SYS_TMR_TickCountGet() - ledBinkStartTick >= SYS_TMR_TickCounterFrequencyGet() / BLINK_FREQ_SLOW)
            {
                ledBinkStartTick = SYS_TMR_TickCountGet();
                _statToggle(LED_POWER);
            }
            switch(APP_OTA_State())
            { // REVIEW: Don't use internal state, use test functions instead
                case APP_OTA_DONE:
                    SYS_PRINT("MAIN: No new firmware available\r\n");
                    _changeState(APP_STATE_OPERATIONAL);
                    break;

                case APP_OTA_ERROR:
                    SYS_PRINT("MAIN: Download firmware failed\r\n");
                    ErrorMessageWarning_Set(ERROR_MESSAGE_WARNING_HTTP_COMMUNICATION_FAILURE_FIRMWARE);
                    _changeState(APP_STATE_OPERATIONAL);
                    break;

                case APP_OTA_STORAGE_ERROR:
                    SYS_PRINT("MAIN: Saving firmware failed\r\n");
                    ErrorMessageWarning_Set(ERROR_MESSAGE_WARNING_FIRMWARE_STORAGE_ERROR);
                    _changeState(APP_STATE_OPERATIONAL);
                    break;

                case APP_OTA_READY_FOR_REBOOT:
                    SYS_PRINT("\r\nMAIN: NEW FIRMWARE, REBOOT\r\n");
                    _changeState(APP_STATE_REBOOT);
                    break;
            }
            break;
        }

        case APP_STATE_OPERATIONAL:
            if(!appData.mqtt_connection_state)
            {
                if(SYS_TMR_TickCountGet() - ledBinkStartTick >= SYS_TMR_TickCounterFrequencyGet() / BLINK_FREQ_SLOW)
                {
                    ledBinkStartTick = SYS_TMR_TickCountGet();
                    _statToggle(LED_MQTT);
                }
            }
            if(hasNetworkChanged())
            {
                SYS_PRINT("\r\nMAIN: Network changed\r\n");
                ErrorMessageWarning_Set(ERROR_MESSAGE_WARNING_NETWORK_CHANGE);
                _changeState(APP_STATE_WAIT_FOR_INTERNET_WHILE_OPERATIONAL);
            }
            else if(APP_MQTT_GET_STATE() == APP_TTNGWC_ERROR)
            {
                SYS_PRINT("\r\nMAIN: MQTT error\r\n");
                ErrorMessageWarning_Set(ERROR_MESSAGE_WARNING_MQTT_COMMUNICATION_FAILURE);
                _changeState(APP_STATE_WAIT_FOR_INTERNET_WHILE_OPERATIONAL);
            }
            else if(!appData.mqtt_connection_state && APP_MQTT_GET_STATE() == APP_TTNGWC_IDLE)
            {
                appData.mqtt_connection_state = true;
                _statSet(LED_MQTT, ON);
                SYS_PRINT("\r\n*************************");
                SYS_PRINT("\r\nMAIN: Gateway bridging");
                SYS_PRINT("\r\n*************************\r\n\r\n");
            }
            if(SYS_TMR_TickCountGet() - firmwareCheckStartTick >=
               SYS_TMR_TickCounterFrequencyGet() * FIRMWARE_CHECK_TIMEOUT)
            {
                /* TODO: change the main flow to download the config and checksum every X hours
                        The current implementation just reboots the gateway */
                SYS_PRINT("\r\nMAIN: Rebooting gateway for firmware update check\r\n");
                WatchDogReset();
                //_changeState(APP_STATE_DOWNLOAD_FIRMWARE);
            }
            if(APP_LORA_HAS_CORRECT_FREQ_PLAN() != 1)
            {
                /* Toggle the activation led to signal that something is wrong */
                if(SYS_TMR_TickCountGet() - ledBinkStartTick >= SYS_TMR_TickCounterFrequencyGet() / BLINK_FREQ_FAST)
                {
                    ledBinkStartTick = SYS_TMR_TickCountGet();
                    _statToggle(LED_ACTIVATION);
                }
            }
            break;

        case APP_STATE_WAIT_FOR_INTERNET_WHILE_OPERATIONAL:
            if(SSMWaitForInternet_IsAPOnly())
            {
                if(SYS_TMR_TickCountGet() - ledBinkStartTick >= SYS_TMR_TickCounterFrequencyGet() / BLINK_FREQ_FAST)
                {
                    ledBinkStartTick = SYS_TMR_TickCountGet();
                    _statToggle(LED_INTERNET);
                }
            }
            else
            {
                if(SYS_TMR_TickCountGet() - ledBinkStartTick >= SYS_TMR_TickCounterFrequencyGet() / BLINK_FREQ_SLOW)
                {
                    ledBinkStartTick = SYS_TMR_TickCountGet();
                    _statToggle(LED_INTERNET);
                }
            }
            if(SSMWaitForInternet_HasInternet())
            {
                _changeState(APP_STATE_OPERATIONAL);
            }
            break;

        case APP_STATE_ERROR:
            if(SYS_TMR_TickCountGet() - errorBlinkStartTick >= SYS_TMR_TickCounterFrequencyGet() / BLINK_FREQ_FAST)
            {
                errorBlinkStartTick = SYS_TMR_TickCountGet();
                int ledIdx;
                for(ledIdx = 0; ledIdx < 5; ledIdx++)
                {
                    if((_error & (1 << ledIdx)) > 0)
                    {
                        _statToggle(ledIdx);
                    }
                }
            }
            if(SYS_TMR_TickCountGet() - errorStartTick >=
               SYS_TMR_TickCounterFrequencyGet() * FATAL_ERROR_REBOOT_TIMEOUT)
            {
                SYS_PRINT("\r\nMAIN: ERROR %d, REBOOT\r\n", _error);
                _changeState(APP_STATE_REBOOT);
            }
            break;

        case APP_STATE_REBOOT:
            break;

        case APP_STATE_SELFTEST:
            APP_SELFTEST_Tasks();
            break;
    }

    // Run submodules
    SSMButton_Tasks();
    switch(_state)
    {
        case APP_STATE_MOUNT_FS:
            break;
        case APP_STATE_LOAD_CONFIG:
            APP_SDCARD_Tasks();
            APP_SERIALFLASH_Tasks();
            SSMLoadConfig_Tasks();
            break;
        case APP_STATE_WAIT_FOR_INTERNET:
            APP_ETH_Tasks();
            APP_WIFI_Tasks();
            SSMWaitForInternet_Tasks();
            break;
        case APP_STATE_LOAD_ONLINE_CONFIG:
            APP_ETH_Tasks();
            APP_WIFI_Tasks();
            APP_SERIALFLASH_Tasks();
            SSMLoadOnlineConfig_Tasks();
            break;
        case APP_STATE_DOWNLOAD_FIRMWARE:
            APP_SDCARD_Tasks();
            APP_SERIALFLASH_Tasks();
            APP_ETH_Tasks();
            APP_WIFI_Tasks();
            APP_OTA_Tasks();
            break;
        case APP_STATE_OPERATIONAL:
            APP_ETH_Tasks();
            APP_WIFI_Tasks();
            APP_MQTT_Tasks();
            break;
        case APP_STATE_WAIT_FOR_INTERNET_WHILE_OPERATIONAL:
            APP_ETH_Tasks();
            APP_WIFI_Tasks();
            SSMWaitForInternet_Tasks();
            break;
        case APP_STATE_STORE_USER_CONFIG:
            APP_ETH_Tasks();  // Needed for redirection
            APP_WIFI_Tasks(); // Needed for redirection
            SSMStoreUserConfig_Tasks();
            APP_SERIALFLASH_Tasks();
            break;
        case APP_STATE_ERASE_ACTI_CONFIG:
        case APP_STATE_ERASE_WIFI_CONFIG:
            APP_SERIALFLASH_Tasks();
            break;
        case APP_STATE_REBOOT:
            break;
        case APP_STATE_ERROR:
            break;
        case APP_STATE_SELFTEST:
            APP_ETH_Tasks();
            APP_WIFI_Tasks();
            break;
    }
}

bool hasNetwork()
{
    return APP_ETH_Has_Link() || APP_WIFI_Has_LinkINFRA();
}

void rememberNetwork(void)
{
    rememberedNetworkState.hasEthernet = APP_ETH_Has_Link();
    rememberedNetworkState.hasWifi     = APP_WIFI_Has_LinkINFRA();
}

bool hasNetworkChanged(void)
{
    return rememberedNetworkState.hasEthernet != APP_ETH_Has_Link() ||
           rememberedNetworkState.hasWifi != APP_WIFI_Has_LinkINFRA();
}

bool GatewayIsOperational(void)
{
    return _state == APP_STATE_OPERATIONAL;
}

bool jumperPlaced()
{
    return (PORTEbits.RE4 == 1);
}

bool selftest_isEnabled(void)
{
    return appData.selftest_enabled;
}

void APP_StatSet(uint8_t stat_num, bool state)
{
    _statSet(stat_num, state);
}

/*******************************************************************************
 End of Life
 */
