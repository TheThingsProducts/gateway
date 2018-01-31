/*******************************************************************************
  MRF24WG Wi-Fi Driver Scan Helper Implementation

  File Name:
    drv_wifi_scan_helper.c

  Summary:
    MRF24WG Wi-Fi Driver Scan Helper Implementation

  Description:
    -Provides access to MRF24WG Wi-Fi controller
    -Reference: MRF24WG Datasheet, IEEE 802.11 Standard
 *******************************************************************************/

//DOM-IGNORE-BEGIN
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

#include <ctype.h> // for function isprint()
#include "drv_wifi_priv.h"

#include "drv_wifi_config_data.h"
#include "drv_wifi_scan_helper.h"

DRV_WIFI_SCAN_STATUS g_scanStatus;
DRV_WIFI_SCAN_RESULT g_scanResults[50];

uint8_t _DRV_WIFI_ScanResultsBufferSize_Get(void)
{
    return sizeof(g_scanResults) / sizeof(g_scanResults[0]);
}

void DRV_WIFI_PreScanStart(void)
{
    uint8_t channelList[] = {};
    DRV_WIFI_SCAN_CONTEXT scanContext;

    g_scanStatus.scanState = 0;
    g_scanStatus.numberOfResults = 0;
    g_scanStatus.displayIdx = 0;
    scanContext.scanType = DRV_WIFI_PASSIVE_SCAN;
    scanContext.minChannelTime = DRV_WIFI_DEFAULT_SCAN_MIN_CHANNEL_TIME;
    scanContext.maxChannelTime = DRV_WIFI_DEFAULT_SCAN_MAX_CHANNEL_TIME;
    scanContext.scanCount = DRV_WIFI_DEFAULT_SCAN_COUNT;
    scanContext.probeDelay = DRV_WIFI_DEFAULT_SCAN_PROBE_DELAY;
    DRV_WIFI_ScanContextSet(&scanContext);
    DRV_WIFI_ChannelListSet(channelList, sizeof(channelList));
    DRV_WIFI_ScanStart();
}

uint16_t DRV_WIFI_ScanStart(void)
{
    if (IS_SCAN_IN_PROGRESS(g_scanStatus.scanState))
    {
        return DRV_WIFI_ERROR_OPERATION_CANCELLED;
    }
    if (DRV_WIFI_Scan(true) != DRV_WIFI_SUCCESS)
    {
        return DRV_WIFI_ERROR_OPERATION_CANCELLED;
    }
    SCAN_SET_IN_PROGRESS(g_scanStatus.scanState);
    SCAN_CLEAR_VALID(g_scanStatus.scanState);
    SCAN_CLEAR_DISPLAY(g_scanStatus.scanState);
    g_drv_wifi_priv.isScanDone = false;
    g_scanStatus.displayIdx = 0;
    return DRV_WIFI_SUCCESS;
}

void DRV_WIFI_ScanComplete(uint16_t scanResults)
{
    /* Clear the scan in progress flag. */
    SCAN_CLEAR_IN_PROGRESS(g_scanStatus.scanState);
    SCAN_SET_VALID(g_scanStatus.scanState);
    SCAN_SET_DISPLAY(g_scanStatus.scanState);
    g_drv_wifi_priv.isScanDone = true;
    g_scanStatus.numberOfResults = scanResults; /* Cache number of APs found in scan. */
}

uint16_t DRV_WIFI_ScanResultGetHelper(uint8_t idx, DRV_WIFI_SCAN_RESULT *p_ScanResult)
{
    if (idx >= g_scanStatus.numberOfResults)
        return DRV_WIFI_ERROR_INVALID_PARAM;

    /*
     * p_ScanResult->ssid contains 32 characters at most. So if ssidLen is 32,
     * ssid will not contain the null character.
     */
    DRV_WIFI_ScanResultGet(idx, p_ScanResult);

    return DRV_WIFI_SUCCESS;
}

void DRV_WIFI_ScanResultsDisplayManager(void)
{
    static bool displayHeader = true;
    uint8_t i;
    uint8_t ssid[32 + 1];

    if ((g_scanStatus.numberOfResults == 0) ||
            (IS_SCAN_IN_PROGRESS(g_scanStatus.scanState)) ||
            (!IS_SCAN_STATE_VALID(g_scanStatus.scanState)) ||
            (!IS_SCAN_STATE_DISPLAY(g_scanStatus.scanState)))
    {
        SCAN_CLEAR_DISPLAY(g_scanStatus.scanState);
        return;
    }

    if (displayHeader)
    {
        g_scanStatus.displayIdx = 0;
        SYS_CONSOLE_MESSAGE((const char *)"\nWi-Fi Scan Results:\r\n\n");
        if (g_scanStatus.numberOfResults == 0)
        {
            /*
             * No AP found, re-initialize the scan context state. Scan is not allowed in Soft AP mode's connected state.
             * However, g_scanStatus.numberOfResults needs to be preserved for webpage using.
             */
            SYS_CONSOLE_MESSAGE((const char *)"No AP found\r\n ");
            SCAN_CLEAR_DISPLAY(g_scanStatus.scanState);
            if (DRV_WIFI_CONFIG_PARAMS(networkType) == DRV_WIFI_NETWORK_TYPE_SOFT_AP) {
                SCAN_CLEAR_VALID(g_scanStatus.scanState);
                return;
            } else {
                return;
            }
        }
        else
        {
            SYS_CONSOLE_MESSAGE((const char *)"    SSID                              RSSI  Channel\r\n");
            SYS_CONSOLE_MESSAGE((const char *)"    --------------------------------  ----  -------\r\n");
            displayHeader = false;
        }
    }

    // If the g_scanResults[] buffer is full, don't try to store more scan results to it.
    if (g_scanStatus.displayIdx > sizeof(g_scanResults) / sizeof(g_scanResults[0]) - 1)
    {
        /*
         * Soft AP mode means this is a pre-scan.
         * When pre-scan is finished, re-initialize the scan context state. Scan is not allowed in Soft AP mode's connected state.
         * However, g_scanStatus.numberOfResults needs to be preserved for webpage using.
         */
        if (DRV_WIFI_CONFIG_PARAMS(networkType) == DRV_WIFI_NETWORK_TYPE_SOFT_AP)
            SCAN_CLEAR_VALID(g_scanStatus.scanState);
        SCAN_CLEAR_DISPLAY(g_scanStatus.scanState);
        g_scanStatus.displayIdx = 0;
        displayHeader = true;
        return;
    }
    DRV_WIFI_ScanResultGetHelper(g_scanStatus.displayIdx, &(g_scanResults[g_scanStatus.displayIdx]));

    /* Display SSID */
    memset(ssid, ' ', sizeof(ssid));
    for (i = 0; i < g_scanResults[g_scanStatus.displayIdx].ssidLen; ++i)
    {
        if (!isprint(g_scanResults[g_scanStatus.displayIdx].ssid[i]))
            ssid[i] = '*';
        else
            ssid[i] = g_scanResults[g_scanStatus.displayIdx].ssid[i];
    }
    ssid[32] = 0;

    /* RSSI_MAX : 128, RSSI_MIN : 43 */
    SYS_CONSOLE_PRINT(" %2d)%s  %2u    %u\r\n",
            g_scanStatus.displayIdx + 1,
            ssid,
            g_scanResults[g_scanStatus.displayIdx].rssi,
            g_scanResults[g_scanStatus.displayIdx].channel);

    // If all the scan results have been displayed and stored, clear status and return.
    if (++g_scanStatus.displayIdx >= g_scanStatus.numberOfResults)
    {
        /*
         * Soft AP mode means this is a pre-scan.
         * When pre-scan is finished, re-initialize the scan context state. Scan is not allowed in Soft AP mode's connected state.
         * However, g_scanStatus.numberOfResults needs to be preserved for webpage using.
         */            
        if (DRV_WIFI_CONFIG_PARAMS(networkType) == DRV_WIFI_NETWORK_TYPE_SOFT_AP)
            SCAN_CLEAR_VALID(g_scanStatus.scanState);
        SCAN_CLEAR_DISPLAY(g_scanStatus.scanState);
        g_scanStatus.displayIdx = 0;
        displayHeader = true;
    }
}

void DRV_WIFI_ScanResultsSaveManager(uint8_t saveIdx)
{
    if ((g_scanStatus.numberOfResults == 0) ||
            (IS_SCAN_IN_PROGRESS(g_scanStatus.scanState)) ||
            (!IS_SCAN_STATE_VALID(g_scanStatus.scanState)))
    {
        return;
    }

    // If the g_scanResults[] buffer is full, don't try to store more scan results to it.
    if (saveIdx >= sizeof(g_scanResults) / sizeof(g_scanResults[0]) - 1)
    {
        /*
         * Soft AP mode means this is a pre-scan.
         * When pre-scan is finished, re-initialize the scan context state. Scan is not allowed in Soft AP mode's connected state.
         * However, g_scanStatus.numberOfResults needs to be preserved for webpage using.
         */            
        if (DRV_WIFI_CONFIG_PARAMS(networkType) == DRV_WIFI_NETWORK_TYPE_SOFT_AP)
            SCAN_CLEAR_VALID(g_scanStatus.scanState);
        if (saveIdx > sizeof(g_scanResults) / sizeof(g_scanResults[0]) - 1)
            return;
    }
    DRV_WIFI_ScanResultGetHelper(saveIdx, &(g_scanResults[saveIdx]));

    // If all the scan results have been stored, clear status and return.
    if (saveIdx == g_scanStatus.numberOfResults - 1)
    {
        if (DRV_WIFI_CONFIG_PARAMS(networkType) == DRV_WIFI_NETWORK_TYPE_SOFT_AP)
        {
            /*
             * Soft AP mode means this is a pre-scan.
             * When pre-scan is finished, re-initialize the scan context state. Scan is not allowed in Soft AP mode's connected state.
             * However, g_scanStatus.numberOfResults needs to be preserved for webpage using.
             */            
            SCAN_CLEAR_VALID(g_scanStatus.scanState);
        }
    }
}

//DOM-IGNORE-END
