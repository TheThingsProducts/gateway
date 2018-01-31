/*******************************************************************************
  MRF24WG Private Configuration Support

  File Name:
    drv_wifi_iwpriv.c

  Summary:
    Configure optional (private) parameters of MRF24WG driver.

  Description:
    Functions in this module support the connection process for the
    MRF24WG.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

/******************/
/*    INCLUDES    */
/******************/
#include "drv_wifi_priv.h"

#include "drv_wifi_config_data.h"
#include "drv_wifi_debug_output.h"
#include "drv_wifi_iwpriv.h"
#include "drv_wifi_scan_helper.h"

/*******************/
/*    VARIABLES    */
/*******************/
extern DRV_WIFI_SCAN_STATUS g_scanStatus;
extern DRV_WIFI_CONFIG_DATA *p_wifi_configData;

static uint8_t s_prescan_allowed = false;
static uint8_t s_prescan_inprogress = false; // WF_PRESCAN - This is used only to allow prescan once.

static bool _prescan_isfinished(void)
{
    if (s_prescan_inprogress && g_drv_wifi_priv.isScanDone) {
        s_prescan_inprogress = false;
        return true;
    }
    return false;
}

static IWPRIV_SCAN_STATUS _scanstatus_get(void)
{
    IWPRIV_SCAN_STATUS ret;

    if(IS_SCAN_IN_PROGRESS(g_scanStatus.scanState)) {
        ret = IWPRIV_SCAN_IN_PROGRESS;
    } else if(IS_SCAN_STATE_DISPLAY(g_scanStatus.scanState) && (g_scanStatus.numberOfResults > 0)){
        ret = IWPRIV_SCAN_SUCCESSFUL;
    } else if (IS_SCAN_STATE_DISPLAY(g_scanStatus.scanState) && (g_scanStatus.numberOfResults == 0)) {
        ret = IWPRIV_SCAN_NO_AP_FOUND;
    } else {
        ret = IWPRIV_SCAN_IDLE;
    }
    return ret;
}

static void *_scanresult_get(uint16_t index)
{
    IWPRIV_GET_PARAM get_param;

    iwpriv_get(SCANRESULTS_COUNT_GET, &get_param);
    if (index < get_param.scan.numberOfResults &&
            index < (uint16_t)_DRV_WIFI_ScanResultsBufferSize_Get())
        return &(g_scanResults[index]);
    else
        return NULL;
}

static void _leftclient_get(bool *updated, uint8_t *addr)
{
    bool connected;

    *updated = false;
    if (ClientCacheUpdated(&connected, addr))
        *updated = connected ? false: true;
}

static IWPRIV_CONN_STATUS _connstatus_get(void)
{
    if (isLinkUp()) {
        if (g_drv_wifi_priv.isConnReestablished) {
            g_drv_wifi_priv.isConnReestablished = false;
            return IWPRIV_CONNECTION_REESTABLISHED;
        } else {
            return IWPRIV_CONNECTION_SUCCESSFUL;
        }
    } else {
        if (g_drv_wifi_priv.isConnPermanentlyLost) {
            return IWPRIV_CONNECTION_FAILED;
        } else {
            return IWPRIV_CONNECTION_IDLE;
        }
    }
    return IWPRIV_CONNECTION_IN_PROGRESS;
}

static uint8_t _initstatus_get(void)
{
    if (g_drv_wifi_priv.isDriverOpen)
        return IWPRIV_READY;
    else
        return IWPRIV_IN_PROGRESS;
}

static bool _is_servermode(void)
{
    IWPRIV_GET_PARAM get_param;

    iwpriv_get(NETWORKTYPE_GET, &get_param);
    if (get_param.netType.type == DRV_WIFI_NETWORK_TYPE_SOFT_AP ||
            get_param.netType.type == DRV_WIFI_NETWORK_TYPE_ADHOC)
        return true;
    else
        return false;
}

static void _prescan_option_set(bool scan)
{
    if (scan) {
        s_prescan_allowed = true;
        s_prescan_inprogress = false;
    } else {
        s_prescan_allowed = false;
        s_prescan_inprogress = false;
    }
}

static void _config_write(void *wifi_config)
{
    memcpy(p_wifi_configData, wifi_config, sizeof(DRV_WIFI_CONFIG_DATA));
    //DRV_WIFI_ConfigDataSave();
}

static IWPRIV_STATUS _mcastfilter_set(uint8_t *addr)
{
    if (MRF24W_MACMulticastFilterSet(NULL, (TCPIP_MAC_ADDR *)addr) == TCPIP_MAC_RES_OK)
        return IWPRIV_READY;
    return IWPRIV_ERROR;
}

static void _powersave_set(bool enable)
{
    if (enable) {
        DRV_WIFI_PS_POLL_CONTEXT psPollContext;
        psPollContext.dtimInterval = DRV_WIFI_DEFAULT_PS_DTIM_INTERVAL;
        psPollContext.listenInterval = DRV_WIFI_DEFAULT_PS_LISTEN_INTERVAL;
        psPollContext.useDtim = true;
        DRV_WIFI_PsPollEnable(&psPollContext);
    } else {
        DRV_WIFI_PsPollDisable();
    }
}

static void _prescan_start(void)
{
    s_prescan_inprogress = true;
    DRV_WIFI_PreScanStart();
}

static void _scan_start(void)
{
    IWPRIV_GET_PARAM get_param;

    iwpriv_get(NETWORKTYPE_GET, &get_param);
    if (get_param.netType.type == DRV_WIFI_NETWORK_TYPE_SOFT_AP && isLinkUp()) {
        SYS_CONSOLE_MESSAGE("MRF24W cannot perform scan in Soft AP mode . . .\r\n");
    } else if (DRV_WIFI_ScanStart() == DRV_WIFI_SUCCESS) {
        SYS_CONSOLE_MESSAGE("Scan started . . .\r\n");
    }
}

static IWPRIV_STATUS _scanresults_display(void)
{
    if (IS_SCAN_STATE_DISPLAY(g_scanStatus.scanState)) {
        DRV_WIFI_ScanResultsDisplayManager();
        return IWPRIV_IN_PROGRESS;
    }
    return IWPRIV_READY;
}

static IWPRIV_STATUS _scanresults_save(void)
{
    static uint8_t saveIdx = 0;

    if ((g_scanStatus.numberOfResults == 0) ||
        (IS_SCAN_IN_PROGRESS(g_scanStatus.scanState)) ||
        (!IS_SCAN_STATE_VALID(g_scanStatus.scanState)))
        return IWPRIV_ERROR;
    if (saveIdx < g_scanStatus.numberOfResults) {
        DRV_WIFI_ScanResultsSaveManager(saveIdx++);
        return IWPRIV_IN_PROGRESS;
    } else if (saveIdx == g_scanStatus.numberOfResults) {
        saveIdx = 0;
        return IWPRIV_READY;
    } else {
        saveIdx = 0;
        return IWPRIV_ERROR;
    }
}

void iwpriv_get(IWPRIV_CMD cmd, IWPRIV_GET_PARAM *param)
{
    switch (cmd) {
    case PRESCAN_OPTION_GET:
        param->scan.prescanAllowed = s_prescan_allowed;
        break;
    case PRESCAN_ISFINISHED_GET:
        param->scan.prescanFinished = _prescan_isfinished();
        break;
    case SCANSTATUS_GET:
        param->scan.scanStatus = _scanstatus_get();
        break;
    case SCANRESULT_GET:
        param->scan.data = _scanresult_get(param->scan.index);
        break;
    case SCANRESULTS_COUNT_GET:
        param->scan.numberOfResults = g_scanStatus.numberOfResults;
        break;
    case CONFIG_GET:
        memcpy(param->config.data, p_wifi_configData, sizeof(DRV_WIFI_CONFIG_DATA));
        break;
    case SSID_GET:
        DRV_WIFI_SsidGet(param->ssid.ssid, &param->ssid.ssidLen);
        break;
    case NETWORKTYPE_GET:
        DRV_WIFI_NetworkTypeGet(&param->netType.type);
        break;
    case CLIENTINFO_GET:
        _leftclient_get(&param->clientInfo.updated, param->clientInfo.addr);
        break;
    case CONNSTATUS_GET:
        param->conn.status = _connstatus_get();
        break;
    case DEVICEINFO_GET:
        DRV_WIFI_DeviceInfoGet((DRV_WIFI_DEVICE_INFO *)param->devInfo.data);
        break;
    case INITSTATUS_GET:
        param->driverStatus.initStatus = _initstatus_get();
        break;
    case OPERATIONMODE_GET:
        param->opMode.isServer = _is_servermode();
        break;
    default:
        DRV_WIFI_ASSERT(false, "Invalid iwpriv get command");
        break;
    }
}

void iwpriv_set(IWPRIV_CMD cmd, IWPRIV_SET_PARAM *param)
{
    switch (cmd) {
    case PRESCAN_OPTION_SET:
        _prescan_option_set(param->scan.prescanAllowed);
        break;
    case CONFIG_SET:
        _config_write(param->config.data);
        break;
    case SSID_SET:
        DRV_WIFI_SsidSet(param->ssid.ssid, param->ssid.ssidLen);
        break;
    case NETWORKTYPE_SET:
        DRV_WIFI_NetworkTypeSet(param->netType.type);
        break;
    case ADHOCCTX_SET:
        DRV_WIFI_AdhocContextSet((DRV_WIFI_ADHOC_NETWORK_CONTEXT *)param->ctx.data);
        break;
    case INITCONN_OPTION_SET:
        g_drv_wifi_priv.initConn = param->conn.initConnAllowed;
        break;
    case MULTICASTFILTER_SET:
        param->multicast.status = _mcastfilter_set(param->multicast.addr);
        break;
    case POWERSAVE_SET:
        _powersave_set(param->powerSave.enabled);
        break;
    default:
        DRV_WIFI_ASSERT(false, "Invalid iwpriv set command");
        break;
    }
}

void iwpriv_execute(IWPRIV_CMD cmd, IWPRIV_EXECUTE_PARAM *param)
{
    switch (cmd) {
    case PRESCAN_START:
        _prescan_start();
        break;
    case SCAN_START:
        _scan_start();
        break;
    case SCANRESULTS_DISPLAY:
        param->scan.displayStatus = _scanresults_display();
        break;
    case SCANRESULTS_SAVE:
        param->scan.saveStatus = _scanresults_save();
        break;
    default:
        DRV_WIFI_ASSERT(false, "Invalid iwpriv execute command");
        break;
    }
}

// DOM-IGNORE-END
