// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include <sys/attribs.h>
#include "app.h"
#include "app_commands.h"
#include "subsystem_controller.h"
#include "wdrv_mrf24wn_main.h"

#define APP_OSAL_MUTEX_LOCK() APP_OSAL_MutexLock(&s_appLock, OSAL_WAIT_FOREVER)
#define APP_OSAL_MUTEX_UNLOCK() APP_OSAL_MutexUnlock(&s_appLock)

#define WIFI_INTERFACE_NAME "MRF24WN"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef enum
{
    /* Waiting for the TCP stack to be available*/
    STATE_WAIT_FOR_TCP_INIT,
    /* Waiting for the MAC address to be available from the TCP stack*/
    STATE_WAIT_FOR_MAC,
    /* Waiting for the application configures the Wi-Fi settings. */
    STATE_WAIT_CONFIG,
    /* The application configures the Wi-Fi settings. */
    STATE_CONFIG,
    /* In this state, the application runs the Wi-Fi prescan. */
    STATE_PRESCAN,
    /* In this state, the application enables TCP/IP modules such as DHCP, NBNS and mDNS
       in all available interfaces. */
    STATE_MODULES_ENABLE,
    /* In this state, the application can do TCP/IP transactions. */
    STATE_TRANSACT,
    STATE_ERROR,
} STATE_t;

typedef enum
{
    /* Initialize the state machine, and also checks if prescan is allowed. */
    APP_WIFI_PRESCAN_INIT = 0,
    /* In this state the application waits for the prescan to finish. */
    APP_WIFI_PRESCAN_WAIT,
    /* In this state the application saves the prescan results. */
    APP_WIFI_PRESCAN_SAVE,
    /* After prescan, Wi-Fi module is reset in this state. */
    APP_WIFI_PRESCAN_RESET,
    /* In this state, the application waits for Wi-Fi reset to finish. */
    APP_WIFI_PRESCAN_WAIT_RESET,
    /* Prescan is complete. */
    APP_WIFI_PRESCAN_DONE,
} SCAN_STATE_t;

typedef enum
{
    EVENT_NONE,
    EVENT_SWITCH_TO_AP_MODE,
    EVENT_SWITCH_TO_INFRA_MODE
} EVENT_t;

typedef enum
{
    CNFG_NONE,
    CNFG_AP_MODE,
    CNFG_INFRA_MODE
} CONFIG_t;

static STATE_t      _state;
static SCAN_STATE_t _scanState;
static EVENT_t      _event;
static CONFIG_t     _activeConfig;
static bool         _dnsOn;
static bool         _dhcpcOn;
static bool         _dhcpsOn;
static bool         _mdnsOn;

static TCPIP_NET_HANDLE netHandle;
const uint8_t*          macAddress;

static IWPRIV_GET_PARAM     s_app_get_param;
static IWPRIV_SET_PARAM     s_app_set_param;
static IWPRIV_EXECUTE_PARAM s_app_execute_param;

static OSAL_MUTEX_HANDLE_TYPE s_appLock;

WF_REDIRECTION_CONFIG g_redirectionConfig;

IWPRIV_SET_PARAM        ap_infra_set_data;
IWPRIV_GET_PARAM        ap_infra_get_data;
WF_CONFIG_DATA          wf_configData;
extern APP_GW_WIFI_DATA appWifiData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
#if(OSAL_USE_RTOS == 1) /* It means FreeRTOS V8.x.x is used. */
/* The application task runs forever, so no de-init function is provided. */
static bool APP_OSAL_MutexInit(OSAL_MUTEX_HANDLE_TYPE* mutex);
static void APP_OSAL_MutexLock(OSAL_MUTEX_HANDLE_TYPE* mutex, uint16_t waitMS);
static void APP_OSAL_MutexUnlock(OSAL_MUTEX_HANDLE_TYPE* mutex);
#endif

static void    APP_CONSOLE_HeaderDisplay(void);
static void    APP_WIFI_RedirectionConfigInit(void);
static void    APP_WIFI_DHCPS_Sync();
static void    APP_WIFI_IFModules_Disable();
static void    APP_WIFI_IFModules_Enable();
static void    APP_WIFI_IF_Down();
static void    APP_WIFI_IF_Up();
static uint8_t APP_WIFI_Prescan(void);
static void    _changeState(STATE_t newState);
static void    _changeScanState(SCAN_STATE_t newState);

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

#if(OSAL_USE_RTOS == 1) /* It means FreeRTOS V8.x.x is used. */
static bool APP_OSAL_MutexInit(OSAL_MUTEX_HANDLE_TYPE* mutex)
{
    if(OSAL_MUTEX_Create(mutex) == OSAL_RESULT_TRUE)
        return true;
    else
        return false;
}

static void APP_OSAL_MutexLock(OSAL_MUTEX_HANDLE_TYPE* mutex, uint16_t waitMS)
{
    OSAL_MUTEX_Lock(mutex, waitMS);
}

static void APP_OSAL_MutexUnlock(OSAL_MUTEX_HANDLE_TYPE* mutex)
{
    OSAL_MUTEX_Unlock(mutex);
}
#endif

/*******************************************************************************
  Function:
    static void APP_WIFI_RedirectionConfigInit(void)

  Remarks:
    Initialize redirection configuration variable
 */
static void APP_WIFI_RedirectionConfigInit(void)
{
    g_redirectionConfig.ssid[0]        = 0;
    g_redirectionConfig.securityMode   = WF_SECURITY_OPEN;
    g_redirectionConfig.securityKey[0] = 0;
    g_redirectionConfig.wepKeyIndex    = WF_WEP_KEY_INVALID;
    g_redirectionConfig.networkType    = WF_NETWORK_TYPE_INFRASTRUCTURE;
}

static void APP_WIFI_DHCPS_Sync(TCPIP_NET_HANDLE netH)
{
#if defined(TCPIP_STACK_USE_DHCP_SERVER)
    bool           updated;
    TCPIP_MAC_ADDR addr;

    s_app_get_param.clientInfo.addr = addr.v;
    iwpriv_get(CLIENTINFO_GET, &s_app_get_param);
    updated = s_app_get_param.clientInfo.updated;

    if(updated)
        TCPIP_DHCPS_LeaseEntryRemove(netH, (TCPIP_MAC_ADDR*)&addr);
#endif
}

static void APP_WIFI_IFModules_Disable(void)
{
    SYS_PRINT("WIFI: Disabling modules\r\n");
    if(_dnsOn)
    {
        TCPIP_DNS_Disable(netHandle, false);
        _dnsOn = false;
    }
    if(_dhcpsOn)
    {
        TCPIP_DHCPS_Disable(netHandle);
        _dhcpsOn = false;
    }
    if(_dhcpcOn)
    {
        TCPIP_DHCP_Disable(netHandle);
        _dhcpcOn = false;
    }
    if(_mdnsOn)
    {
        TCPIP_MDNS_ServiceDeregister(netHandle);
        _mdnsOn = false;
    }
}

static void APP_WIFI_IFModules_Enable(void)
{
    iwpriv_get(OPERATIONMODE_GET, &s_app_get_param);
    if(s_app_get_param.opMode.isServer)
    {
        SYS_PRINT("WIFI: Enabling modules for server\r\n");
        TCPIP_DHCPS_Enable(netHandle); // start DHCP server
        _dhcpsOn = true;
    }
    else
    {
        SYS_PRINT("WIFI: Enabling modules for client\r\n");
        TCPIP_DHCP_Enable(netHandle); // start DHCP client
        _dhcpcOn = true;
        TCPIP_DNS_Enable(netHandle, TCPIP_DNS_ENABLE_DEFAULT);
        _dnsOn = true;
    }
    TCPIP_MDNS_ServiceRegister(netHandle, "Things-Gateway-WiFi", "_http._tcp.local", 80, ((const uint8_t*)"path=/"), 1,
                               NULL, NULL);
    _mdnsOn = true;
}

static void APP_WIFI_IF_Down(void)
{
    TCPIP_STACK_NetDown(netHandle);
}

static void APP_WIFI_IF_Up(void)
{
    SYS_MODULE_OBJ              tcpipStackObj;
    TCPIP_STACK_INIT            tcpip_init_data;
    const TCPIP_NETWORK_CONFIG* pIfConf;
    uint16_t                    net_ix = TCPIP_STACK_NetIndexGet(netHandle);

    tcpipStackObj = TCPIP_STACK_Initialize(0, 0);
    TCPIP_STACK_InitializeDataGet(tcpipStackObj, &tcpip_init_data);
    pIfConf = tcpip_init_data.pNetConf + net_ix;
    TCPIP_STACK_NetUp(netHandle, pIfConf);
}

static bool WDRV_CONFIG_LoadAP()
{
    WDRV_ASSERT(macAddress != NULL, "MAC address should be known by now");
    snprintf(wf_configData.ssid, WDRV_MAX_SSID_LENGTH, "%s-%02X%02X%02X%02X%02X%02X", WDRV_DEFAULT_SSID, macAddress[0],
             macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]);
    wf_configData.ssid[WDRV_MAX_SSID_LENGTH - 1] = 0;
    wf_configData.ssidLen                        = strlen(wf_configData.ssid);
    wf_configData.networkType                    = WDRV_DEFAULT_NETWORK_TYPE;
    wf_configData.securityMode                   = WDRV_DEFAULT_SECURITY_MODE;

    switch(wf_configData.securityMode)
    {
        case WDRV_SECURITY_OPEN:
            memset(wf_configData.securityKey, 0x00, sizeof(wf_configData.securityKey));
            wf_configData.securityKeyLen = 0;
            break;
        case WDRV_SECURITY_WPA_WITH_PASS_PHRASE:
        case WDRV_SECURITY_WPA2_WITH_PASS_PHRASE:
            memcpy(wf_configData.securityKey, (const void*)WDRV_DEFAULT_PSK_PHRASE,
                   sizeof(WDRV_DEFAULT_PSK_PHRASE) - 1);
            wf_configData.securityKeyLen = sizeof(WDRV_DEFAULT_PSK_PHRASE) - 1;
            break;
        default:
            WDRV_ASSERT(false, "Unsupported security mode\r\n");
            break;
    }
    return true;
}

void WDRV_CONFIG_LoadInfra(char* ssid, char* key, uint8_t sec_type, uint8_t conn_type)
{
    strncpy(wf_configData.ssid, ssid, sizeof(wf_configData.ssid));
    wf_configData.ssidLen      = strlen(ssid);
    wf_configData.networkType  = conn_type;
    wf_configData.securityMode = sec_type;

    strncpy(wf_configData.securityKey, key, sizeof(wf_configData.securityKey));
    wf_configData.securityKeyLen = strlen(key);
}

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

/*******************************************************************************
  Function:
    void APP_WIFI_Initialize(void)

  Remarks:
    None.
 */
void APP_WIFI_Initialize(void)
{
    if(!APP_OSAL_MutexInit(&s_appLock))
    {
        SYS_DEBUG(SYS_ERROR_ERROR, "WIFI: Mutex initialisation failed!\r\n");
        return;
    }
    _dnsOn   = false;
    _dhcpcOn = false;
    _dhcpsOn = false;

    // initialize redirection variable
    APP_WIFI_RedirectionConfigInit();

    /* Place the App state machine in its initial state. */
    _changeState(STATE_WAIT_FOR_TCP_INIT);
    _changeScanState(APP_WIFI_PRESCAN_INIT);
    _activeConfig = CNFG_NONE;

    ap_infra_set_data.config.data = &wf_configData;
    iwpriv_set(CONFIG_SET, &ap_infra_set_data);

    s_app_set_param.conn.initConnAllowed = false;
    iwpriv_set(INITCONN_OPTION_SET, &s_app_set_param);
    s_app_set_param.scan.prescanAllowed = true;
    iwpriv_set(PRESCAN_OPTION_SET, &s_app_set_param);
}

/*******************************************************************************
  Function:
    void APP_WIFI_Tasks(void)

  Remarks:
    None.
 */
void APP_WIFI_Tasks(void)
{
    static bool      wasNetUp[2] = {true, true}; // this app supports 2 interfaces so far
    static uint32_t  startTick   = 0;
    static IPV4_ADDR dwLastIP    = {-1};
    int              i, nNets;

    switch(_event)
    {
        case EVENT_SWITCH_TO_AP_MODE:
            switch(_state)
            {
                case STATE_WAIT_CONFIG:
                case STATE_TRANSACT:
                    // Tear down WiFi interface
                    APP_WIFI_IFModules_Disable();
                    APP_WIFI_IF_Down();

                    // Configure WiFi driver as AP
                    WDRV_CONFIG_LoadAP();
                    ap_infra_set_data.config.data = &wf_configData;
                    iwpriv_set(CONFIG_SET, &ap_infra_set_data);

                    // Configure to allow SSID scan and not connect
                    s_app_set_param.conn.initConnAllowed = false;
                    iwpriv_set(INITCONN_OPTION_SET, &s_app_set_param);
                    s_app_set_param.scan.prescanAllowed = true;
                    iwpriv_set(PRESCAN_OPTION_SET, &s_app_set_param);

                    // Setup WiFi interface (modules will be enabled after SSID scan)
                    APP_WIFI_IF_Up();

                    // Enter configuration state
                    _changeState(STATE_CONFIG);
                    _activeConfig = CNFG_AP_MODE;
                    _event        = EVENT_NONE; // Now we handled the event
                    break;
                default:
                    break; // do nothing yet
            }
            break;

        case EVENT_SWITCH_TO_INFRA_MODE:
            switch(_state)
            {
                case STATE_WAIT_CONFIG:
                case STATE_TRANSACT:
                    if (_activeConfig == CNFG_INFRA_MODE){
                        _event = EVENT_NONE;
                        break;
                    }
                    // Tear down WiFi interface
                    APP_WIFI_IFModules_Disable();
                    APP_WIFI_IF_Down();

                    // Configire WiFi driver in INFRA mode
                    WDRV_CONFIG_LoadInfra(appWifiData.ssid, appWifiData.key, appWifiData.sec_type,
                                          appWifiData.conn_type);
                    ap_infra_set_data.config.data = &wf_configData;
                    iwpriv_set(CONFIG_SET, &ap_infra_set_data);

                    // Configure to auto connect and not SSID scan
                    s_app_set_param.conn.initConnAllowed = true;
                    iwpriv_set(INITCONN_OPTION_SET, &s_app_set_param);
                    s_app_set_param.scan.prescanAllowed = false;
                    iwpriv_set(PRESCAN_OPTION_SET, &s_app_set_param);

                    // Setup WiFi interface (modules will be enabled after SSID scan)
                    APP_WIFI_IF_Up();

                    // Enter configuration state
                    _changeState(STATE_CONFIG);
                    _activeConfig = CNFG_INFRA_MODE;
                    _event        = EVENT_NONE; // Now we handled the event
                    break;
                default:
                    break; // do nothing yet
            }
            break;

        case EVENT_NONE:
            break;
    }

    switch(_state)
    {
        case STATE_WAIT_FOR_TCP_INIT:
        {
            SYS_STATUS tcpipStat = TCPIP_STACK_Status(sysObj.tcpip);
            if(tcpipStat < 0)
            {
                SYS_DEBUG(SYS_ERROR_ERROR, "WIFI: TCP/IP stack initialisation failed!\r\n");
                _changeState(STATE_ERROR);
            }
            else if(tcpipStat == SYS_STATUS_READY)
            {
                netHandle = TCPIP_STACK_IndexToNet(WIFI_INTERFACE_NUM);
                _changeState(STATE_WAIT_FOR_MAC);
            }
            break;
        }
        case STATE_WAIT_FOR_MAC:
            macAddress = TCPIP_STACK_NetAddressMac(netHandle);
            if(macAddress)
            {
                // In the past here was switched to STATE_PRESCAN. But that cause another thread (lora_task)
                // to fail with a hard-fault. Go here ot STATE_WAIT_CONFIG instead such that for sure either
                // the EVENT_SWITCH_TO_AP_MODE or EVENT_SWITCH_TO_INFO_MODE events has been proccessed.
                _changeState(STATE_WAIT_CONFIG);
            }
            break;

        case STATE_WAIT_CONFIG:
            break;

        case STATE_CONFIG:
            /*
             * Following "if condition" is useless when demo firstly
             * boots up, since stack's status has already been checked in
             * APP_TCPIP_WAIT_INIT. But it is necessary in redirection or
             * Wi-Fi interface reset due to connection errors.
             */
            iwpriv_get(INITSTATUS_GET, &s_app_get_param);
            if(s_app_get_param.driverStatus.initStatus == IWPRIV_READY)
            {

                switch(_activeConfig)
                {
                    case CNFG_AP_MODE:
                        _changeState(STATE_PRESCAN);
                        break;
                    case CNFG_INFRA_MODE:
                        APP_WIFI_IFModules_Enable();
                        _changeState(STATE_TRANSACT);

                        // initialize redirection variable
                        APP_WIFI_RedirectionConfigInit();
                        break;
                }
            }
            break;

        case STATE_PRESCAN:
        {
            // if pre-scan option is set to false,
            // this state would just run once and pass,
            // APP_WIFI_Prescan() function would not actually
            // do anything
            uint8_t scanStatus = APP_WIFI_Prescan();
            if(scanStatus == IWPRIV_READY)
            {
                if(_activeConfig == CNFG_NONE)
                {
                    _changeState(STATE_WAIT_CONFIG);
                }
                else
                {
                    _changeState(STATE_MODULES_ENABLE);
                }
            }
            else if(scanStatus == IWPRIV_ERROR)
            {
                SYS_DEBUG(SYS_ERROR_ERROR, "WIFI: Prescan Error\r\n");
                if(_activeConfig == CNFG_NONE)
                {
                    _changeState(STATE_WAIT_CONFIG);
                }
                else
                {
                    _changeState(STATE_MODULES_ENABLE);
                }
            }
            break; // Placing break only once for readability (penalty is one extra state machine cycle of 1ms...)
        }

        case STATE_MODULES_ENABLE:
            s_app_set_param.conn.initConnAllowed = true;
            iwpriv_set(INITCONN_OPTION_SET, &s_app_set_param);
            s_app_set_param.scan.prescanAllowed = false;
            iwpriv_set(PRESCAN_OPTION_SET, &s_app_set_param);
            APP_WIFI_IFModules_Enable();
            _changeState(STATE_TRANSACT);
            // intentional fall through
        case STATE_TRANSACT:
            // wait for redirection command from custom_http_app.c
            iwpriv_get(CONNSTATUS_GET, &s_app_get_param);
            if(s_app_get_param.conn.status == IWPRIV_CONNECTION_FAILED)
            {
                APP_WIFI_IFModules_Disable();
                APP_WIFI_IF_Down();
                APP_WIFI_IF_Up();
                _changeState(STATE_CONFIG);
                break;
            }
            else if(s_app_get_param.conn.status == IWPRIV_CONNECTION_REESTABLISHED)
            {
                // restart dhcp client and config power save
                iwpriv_get(OPERATIONMODE_GET, &s_app_get_param);
                if(!s_app_get_param.opMode.isServer)
                {
                    TCPIP_DHCP_Disable(netHandle);
                    TCPIP_DHCP_Enable(netHandle);
                }
            }

            if(APP_Commands_ScanListDisplay_Get())
                APP_Commands_ScanListEntry_Display();
            else
                SYS_CMD_READY_TO_READ();

            APP_WIFI_DHCPS_Sync(netHandle);

            APP_OSAL_MUTEX_LOCK();
            /*
             * If the IP address of an interface has changed,
             * display the new value on the system console.
             */
            IPV4_ADDR ipAddr;
            ipAddr.Val = TCPIP_STACK_NetAddress(netHandle);
            if(dwLastIP.Val != ipAddr.Val)
            {
                dwLastIP.Val = ipAddr.Val;
                SYS_DEBUG(SYS_ERROR_INFO, "WIFI: IP Address: %d.%d.%d.%d \r\n", ipAddr.v[0], ipAddr.v[1], ipAddr.v[2],
                          ipAddr.v[3]);
                if(selftest_isEnabled())
                {
                    if(ipAddr.v[0] != 0)
                    {
                        selftest_hasWifi(true);
                    }
                }
            }
            APP_OSAL_MUTEX_UNLOCK();
            break;

        default:
            break;
    }
}

static uint8_t APP_WIFI_Prescan(void)
{
    switch(_scanState)
    {
        case APP_WIFI_PRESCAN_INIT:
            iwpriv_get(PRESCAN_OPTION_GET, &s_app_get_param);
            if(s_app_get_param.scan.prescanAllowed)
            {
                iwpriv_get(NETWORKTYPE_GET, &s_app_get_param);
                uint8_t type = s_app_get_param.netType.type;
                iwpriv_get(CONNSTATUS_GET, &s_app_get_param);
                if(type == WF_NETWORK_TYPE_SOFT_AP && s_app_get_param.conn.status == IWPRIV_CONNECTION_SUCCESSFUL)
                    return IWPRIV_ERROR;
                iwpriv_execute(PRESCAN_START, &s_app_execute_param);
                _changeScanState(APP_WIFI_PRESCAN_WAIT);
                break;
            }
            else
            {
                return IWPRIV_READY;
            }

        case APP_WIFI_PRESCAN_WAIT:
            iwpriv_get(PRESCAN_ISFINISHED_GET, &s_app_get_param);
            if(s_app_get_param.scan.prescanFinished)
            {
                iwpriv_get(SCANSTATUS_GET, &s_app_get_param);
                if(s_app_get_param.scan.scanStatus == IWPRIV_SCAN_SUCCESSFUL)
                {
                    _changeScanState(APP_WIFI_PRESCAN_SAVE);
                }
                else
                {
                    _changeScanState(APP_WIFI_PRESCAN_INIT);
                    return IWPRIV_ERROR;
                }
            }
            else
            {
                break;
            }

        case APP_WIFI_PRESCAN_SAVE:
            iwpriv_execute(SCANRESULTS_SAVE, &s_app_execute_param);
            if(s_app_execute_param.scan.saveStatus != IWPRIV_IN_PROGRESS)
            {
                if(_activeConfig == CNFG_NONE)
                {
                    _changeScanState(APP_WIFI_PRESCAN_DONE);
                }
                else
                {
                    _changeScanState(APP_WIFI_PRESCAN_RESET);
                }
            }
            break;

        case APP_WIFI_PRESCAN_RESET:
        {
            APP_WIFI_IF_Down();
            APP_WIFI_IF_Up();
            s_app_set_param.conn.initConnAllowed = true;
            iwpriv_set(INITCONN_OPTION_SET, &s_app_set_param);
            s_app_set_param.scan.prescanAllowed = false;
            iwpriv_set(PRESCAN_OPTION_SET, &s_app_set_param);
            _changeScanState(APP_WIFI_PRESCAN_WAIT_RESET);
            break;
        }

        case APP_WIFI_PRESCAN_WAIT_RESET:
            iwpriv_get(INITSTATUS_GET, &s_app_get_param);
            if(s_app_get_param.driverStatus.initStatus == IWPRIV_READY)
                _changeScanState(APP_WIFI_PRESCAN_DONE);
            else
                break;

        case APP_WIFI_PRESCAN_DONE:
            _changeScanState(APP_WIFI_PRESCAN_INIT);
            return IWPRIV_READY;
    }

    return IWPRIV_IN_PROGRESS;
}

void APP_WIFI_AP_MODE(void)
{
    _event = EVENT_SWITCH_TO_AP_MODE;
}

void APP_WIFI_INFRA_MODE(void)
{
    _event = EVENT_SWITCH_TO_INFRA_MODE;
}

bool APP_WIFI_Has_LinkAP(void)
{
    if(_state == STATE_TRANSACT && (_activeConfig == CNFG_AP_MODE) && WDRV_APHasClientsConnected() &&
       TCPIP_STACK_NetIsLinked(netHandle))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool APP_WIFI_Has_LinkINFRA(void)
{
    if(_state == STATE_TRANSACT && (_activeConfig == CNFG_INFRA_MODE) && TCPIP_STACK_NetIsLinked(netHandle))
    {
        return true;
    }
    else
    {
        return false;
    }
}

static void _changeState(STATE_t newState)
{
    _state = newState;

    // onEnter
    SYS_PRINT("WIFI: Entering state %d\r\n", _state);
    switch(newState)
    {
        default:
            break;
    }
}

static void _changeScanState(SCAN_STATE_t newState)
{
    _scanState = newState;

    // onEnter
    SYS_PRINT("WIFI: Entering SCAN state %d\r\n", _scanState);
    switch(newState)
    {
        default:
            break;
    }
}

/* *****************************************************************************
 End of File
 */
