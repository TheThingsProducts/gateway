// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

/*******************************************************************************
  File Name:
    app_commands.c

  Summary:
    Commands for the Wi-Fi Easy Configuration Demo

  Description:

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
// DOM-IGNORE-END

#include <ctype.h>
#include "app.h"

#if defined(TCPIP_STACK_COMMAND_ENABLE) && defined(TCPIP_STACK_COMMANDS_WIFI_ENABLE)

static bool             s_scanList_signal      = false;
static bool             s_consoleWriteComplete = true;
static IWPRIV_GET_PARAM s_appcmd_get_param;

static int  _APP_Commands_ScanList(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void _APP_Commands_ConsoleWriteComplete(void* param);

static const SYS_CMD_DESCRIPTOR appCmdTbl[] = {
    {"scanlist", _APP_Commands_ScanList, ": list scan results"},
};

bool APP_Commands_Init(void)
{
    if(!SYS_CMD_ADDGRP(appCmdTbl, sizeof(appCmdTbl) / sizeof(*appCmdTbl), "app", ": app commands"))
    {
        SYS_ERROR(SYS_ERROR_ERROR, "Failed to create TCPIP Commands\r\n", 0);
        return false;
    }

    SYS_CMD_RegisterCallback(_APP_Commands_ConsoleWriteComplete, SYS_CMD_EVENT_WRITE_COMPLETE);

    s_scanList_signal      = false;
    s_consoleWriteComplete = true;

    return true;
}

bool APP_Commands_ScanListDisplay_Get(void)
{
    return s_scanList_signal;
}

void APP_Commands_ScanListEntry_Display(void)
{
    static uint16_t i = 0;
    WF_SCAN_RESULT* scanResult;

    if(!s_consoleWriteComplete)
    {
#if defined(SYS_CONSOLE_UART_IDX)
        static uint8_t temp_fix_uart_console_cnt = 0;
#if defined(OSAL_USE_RTOS)
        if(temp_fix_uart_console_cnt < 20)
        {
#else
        if(temp_fix_uart_console_cnt < 200)
        {
#endif
            ++temp_fix_uart_console_cnt;
            return;
        }
        else
        {
            temp_fix_uart_console_cnt = 0;
            s_consoleWriteComplete    = true;
        }
#else
        return;
#endif
    }

    if(i == 0)
    {
        const char* scanListHeader = "\r\n\r\n"
                                     "Wi-Fi Scan Results:\r\n\r\n"
                                     "     SSID                              RSSI  Channel\r\n"
                                     "     --------------------------------  ----  -------\r\n";
        SYS_CONSOLE_MESSAGE(scanListHeader);
    }

    iwpriv_get(SCANRESULTS_COUNT_GET, &s_appcmd_get_param);
    if(i < s_appcmd_get_param.scan.numberOfResults)
    {
        uint8_t ssid[32 + 1];
        s_appcmd_get_param.scan.index = i;
        iwpriv_get(SCANRESULT_GET, &s_appcmd_get_param);
        scanResult = (WF_SCAN_RESULT*)s_appcmd_get_param.scan.data;
        if(!scanResult)
        {
            i                 = 0;
            s_scanList_signal = false;
            SYS_CONSOLE_MESSAGE("Not enough buffer to hold all the scan results\r\n");
            s_consoleWriteComplete = false;
            return;
        }
        memset(ssid, ' ', sizeof(ssid));
        uint8_t j;
        for(j = 0; j < scanResult->ssidLen; ++j)
        {
            if(!isprint(scanResult->ssid[j]))
            {
                ssid[j] = '*';
            }
            else
            {
                ssid[j] = scanResult->ssid[j];
            }
        }
        ssid[32] = 0; // null character
        SYS_CONSOLE_PRINT(" %2d) %s  %2u    %u\r\n", ++i, ssid, scanResult->rssi, scanResult->channel);
        s_consoleWriteComplete = false;
    }

    iwpriv_get(SCANRESULTS_COUNT_GET, &s_appcmd_get_param);
    if(i == s_appcmd_get_param.scan.numberOfResults)
    {
        i                 = 0;
        s_scanList_signal = false;
    }
}

static int _APP_Commands_ScanList(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if(argc != 1)
    {
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Usage: scanlist\r\n");
        return false;
    }

    iwpriv_get(SCANRESULTS_COUNT_GET, &s_appcmd_get_param);
    if(s_appcmd_get_param.scan.numberOfResults == 0)
    {
        SYS_CONSOLE_MESSAGE((const char*)"No scan result was previously stored, or no AP found\r\n");
        return true;
    }

    s_scanList_signal = true;

    return true;
}

static void _APP_Commands_ConsoleWriteComplete(void* param)
{
    s_consoleWriteComplete = true;
}

#endif // #if defined(TCPIP_STACK_COMMAND_ENABLE) && defined(TCPIP_STACK_COMMANDS_WIFI_ENABLE)
