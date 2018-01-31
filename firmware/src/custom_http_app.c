// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

/*******************************************************************************
  Application to Demo HTTP Server

  Summary:
    Support for HTTP module in Microchip TCP/IP Stack

  Description:
    -Implements the application
    -Reference: RFC 1002
 *******************************************************************************/

/*******************************************************************************
File Name: custom_http_app.c
Copyright (C) 2012 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

#define __CUSTOMHTTPAPP_C

#include <ctype.h>
#include "system_config.h"

#include "tcpip/tcpip.h"
#include "system/tmr/sys_tmr.h"
#include "system/random/sys_random.h"
#include "tcpip/src/common/helpers.h"
#include "crypto/crypto.h"

#include "tcpip/src/tcpip_private.h"
#include "json-builder.h"

#include "app_activation.h"
#include "app_mqtt.h"
#include "bootloader_version.h"
#include "version.h"
#include "time.h"

extern APP_GW_ACTIVATION_DATA appGWActivationData;

/****************************************************************************
  Section:
    Definitions
 ****************************************************************************/
// Use the web page in the Demo App (~2.5kb ROM, ~0b RAM)
#define HTTP_APP_USE_RECONFIG

#ifndef NO_MD5
// Use the MD5 Demo web page (~5kb ROM, ~160b RAM)
#define HTTP_APP_USE_MD5
#endif

#define HTTP_APP_USE_WIFI

#define HTTP_APP_REDIRECTION_DELAY_TIME (1ul) /* second */
#define HTTP_APP_IPV4_ADDRESS_BUFFER_SIZE 20

/****************************************************************************
  Section:
    Function Prototypes
 ****************************************************************************/
static HTTP_IO_RESULT HTTPPostWIFIConfig(HTTP_CONN_HANDLE connHandle, bool fromsettings);

/****************************************************************************
  Section:
    Variables
 ****************************************************************************/
extern bool                  g_redirect_signal;
extern bool                  g_config_changed;
extern WF_CONFIG_DATA        g_wifi_cfg;
extern WF_REDIRECTION_CONFIG g_redirectionConfig;
static bool                  s_scanResultIsValid = false;
static WF_SCAN_RESULT*       s_scanResult;
static IWPRIV_GET_PARAM      s_httpapp_get_param;
static IWPRIV_SET_PARAM      s_httpapp_set_param;
static IWPRIV_EXECUTE_PARAM  s_httpapp_execute_param;
static uint8_t               s_buf_ipv4addr[HTTP_APP_IPV4_ADDRESS_BUFFER_SIZE];

extern const char* const ddnsServiceHosts[];
// RAM allocated for DDNS parameters
#if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
static uint8_t DDNSData[100];
#endif

// Sticky status message variable.
// This is used to indicated whether or not the previous POST operation was
// successful.  The application uses these to store status messages when a
// POST operation redirects.  This lets the application provide status messages
// after a redirect, when connection instance data has already been lost.
static bool lastSuccess = false;

// Stick status message variable.  See lastSuccess for details.
static bool lastFailure = false;

/****************************************************************************
  Section:
    Helper Functions
 ****************************************************************************/

/*******************************************************************************
 * FUNCTION: Helper_HEXCharToBIN
 *
 * RETURNS: binary value associated with ASCII HEX input value
 *
 * PARAMS: hex_char -- ASCII HEX character
 *
 * NOTES: Converts an input ASCII HEX character to its binary value.  Function
 *        does not error check; it assumes only hex characters are passed in.
 *******************************************************************************/
static uint8_t Helper_HEXCharToBIN(uint8_t hex_char)
{
    if((hex_char >= 'a') && (hex_char <= 'f'))
    {
        return (0x0a + (hex_char - 'a'));
    }
    else if((hex_char >= 'A') && (hex_char <= 'F'))
    {
        return (0x0a + (hex_char - 'A'));
    }
    else // ((hex_char >= '0') && (hex_char <= '9'))
    {
        return (0x00 + (hex_char - '0'));
    }
}

/*******************************************************************************
 * FUNCTION: Helper_HEXStrToBIN
 *
 * RETURNS: true if conversion successful, else false
 *
 * PARAMS: p_ascii_hex_str -- ASCII HEX string to be converted
 *         p_bin -- binary value if conversion successful
 *
 * NOTES: Converts an input ASCII HEX string to binary value (up to 32-bit value)
 *******************************************************************************/
static bool Helper_HEXStrToBIN(char* p_ascii_hex_str, uint16_t* p_bin)
{
    int8_t   i;
    uint32_t multiplier = 1;

    *p_bin = 0;

    // not allowed to have a string of more than 4 nibbles
    if(strlen((char*)p_ascii_hex_str) > 8u)
    {
        return false;
    }

    // first, ensure all characters are a hex digit
    for(i = (uint8_t)strlen((char*)p_ascii_hex_str) - 1; i >= 0; --i)
    {
        if(!isxdigit(p_ascii_hex_str[i]))
        {
            return false;
        }
        *p_bin += multiplier * Helper_HEXCharToBIN(p_ascii_hex_str[i]);
        multiplier *= 16;
    }

    return true;
}

static bool Helper_HEXStrToBINInplace(char* p_str, uint8_t expected_binary_size)
{
    char     str_buffer[3];
    uint8_t  binary_index        = 0;
    char*    ascii_hex_str_start = p_str;
    uint16_t bin_buffer          = 0;

    /* gobble up any hex prefix */
    if(memcmp(ascii_hex_str_start, (const char*)"0x", 2) == 0)
    {
        ascii_hex_str_start += 2;
    }

    if(strlen((char*)ascii_hex_str_start) != (expected_binary_size * 2))
    {
        return false;
    }

    while(binary_index < expected_binary_size)
    {
        memcpy(str_buffer, (const char*)ascii_hex_str_start, 2);
        str_buffer[2] = '\0';

        /* convert the hex string to binary value */
        if(!Helper_HEXStrToBIN(str_buffer, &bin_buffer))
        {
            return false;
        }

        p_str[binary_index++] = (uint8_t)bin_buffer;
        ascii_hex_str_start += 2;
    }

    return true;
}

static bool Helper_WIFI_SecurityHandle(WF_REDIRECTION_CONFIG* cfg, const char* str)
{
    uint8_t ascii_key = 0, key_size = 0;
    switch(cfg->securityMode)
    {
        case WF_SECURITY_OPEN: // Keep compiler happy, nothing to do here!
            ascii_key = true;
            break;
        case WF_SECURITY_WEP_40:
            key_size = 10; /* Assume hex size. */
            if(strlen(str) == 5)
            {
                ascii_key = true;
                key_size  = 5; /* ASCII key support. */
            }
            cfg->wepKeyIndex = 0; /* Example uses only key idx 0 (sometimes called 1). */
            break;
        case WF_SECURITY_WEP_104:
            key_size = 26; /* Assume hex size. */
            if(strlen(str) == 13)
            {
                ascii_key = true;
                key_size  = 13; /* ASCII key support. */
            }
            cfg->wepKeyIndex = 0; /* Example uses only key idx 0 (sometimes called 1). */
            break;
        case WF_SECURITY_WPA_WITH_PASS_PHRASE:
        case WF_SECURITY_WPA2_WITH_PASS_PHRASE:
        case WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE:
            ascii_key = true;
            key_size  = strlen(str);
            // between 8 - 63 characters, passphrase
            if((key_size < 8) || (key_size > 63))
                return false;
            break;
    }
    if(strlen(str) != key_size)
    {
        SYS_CONSOLE_MESSAGE("\r\nIncomplete key received\r\n");
        return false;
    }
    memcpy(cfg->securityKey, (void*)str, key_size);
    cfg->securityKey[key_size] = 0; /* terminate string */
    if(!ascii_key)
    {
        key_size /= 2;
        if(!Helper_HEXStrToBINInplace((char*)cfg->securityKey, key_size))
        {
            SYS_CONSOLE_MESSAGE("\r\nFailed to convert ASCII string (representing HEX digits) to real HEX string!\r\n");
            return false;
        }
    }
    cfg->securityKeyLen = key_size;
    return true;
}

static void Helper_WIFI_KeySave(WF_REDIRECTION_CONFIG* redirectCfg, WF_CONFIG_DATA* cfg)
{
    uint8_t key_size = 0;
    switch((uint8_t)redirectCfg->securityMode)
    {
        case WF_SECURITY_WEP_40:
            key_size = 5;
            break;
        case WF_SECURITY_WEP_104:
            key_size = 13;
            break;
        case WF_SECURITY_WPA_WITH_PASS_PHRASE:
        case WF_SECURITY_WPA2_WITH_PASS_PHRASE:
        case WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE:
            key_size = strlen((const char*)(redirectCfg->securityKey)); // ascii so use strlen
            break;
    }
    memcpy(cfg->securityKey, redirectCfg->securityKey, key_size);
    cfg->securityKey[strlen((const char*)(redirectCfg->securityKey))] = 0;
    cfg->securityKeyLen                                               = key_size;
}

static void Helper_APP_RedirectionFlagSet(uintptr_t context, uint32_t currTick)
{
    g_redirect_signal = true;
}

static HTTP_IO_RESULT Helper_APP_ConfigFailure(HTTP_CONN_HANDLE connHandle, uint8_t* httpDataBuff)
{
    lastFailure = true;
    if(httpDataBuff)
        strcpy((char*)httpDataBuff, "/error");
    TCPIP_HTTP_CurrentConnectionStatusSet(connHandle, HTTP_REDIRECT);
    return HTTP_IO_DONE;
}

/****************************************************************************
  Section:
    Authorization Handlers
 ****************************************************************************/

/****************************************************************************
  Function:
    uint8_t TCPIP_HTTP_FileAuthenticate(HTTP_CONN_HANDLE connHandle, uint8_t *cFile)

  Internal:
    See documentation in the TCP/IP Stack API or HTTP.h for details.
 ****************************************************************************/
#if defined(TCPIP_HTTP_USE_AUTHENTICATION)
uint8_t TCPIP_HTTP_FileAuthenticate(HTTP_CONN_HANDLE connHandle, uint8_t* cFile)
{
    // If the filename begins with the folder "protect", then require auth.
    if(memcmp(cFile, (const void*)"protect", 7) == 0)
        return 0x00; // Authentication will be needed later.

    // If the filename begins with the folder "snmp", then require auth.
    if(memcmp(cFile, (const void*)"snmp", 4) == 0)
        return 0x00; // Authentication will be needed later.

#if defined(HTTP_MPFS_UPLOAD_REQUIRES_AUTH)
    if(memcmp(cFile, (const void*)"mpfsupload", 10) == 0)
        return 0x00;
#endif
    // You can match additional strings here to password protect other files.
    // You could switch this and exclude files from authentication.
    // You could also always return 0x00 to require auth for all files.
    // You can return different values (0x00 to 0x79) to track "realms" for below.

    return 0x80; // No authentication required.
}
#endif

/****************************************************************************
  Function:
    uint8_t TCPIP_HTTP_UserAuthenticate(uint8_t *cUser, uint8_t *cPass)

  Internal:
    See documentation in the TCP/IP Stack API or HTTP.h for details.
 ****************************************************************************/
#if defined(TCPIP_HTTP_USE_AUTHENTICATION)
uint8_t TCPIP_HTTP_UserAuthenticate(HTTP_CONN_HANDLE connHandle, uint8_t* cUser, uint8_t* cPass)
{
    if(strcmp((char*)cUser, (const char*)"admin") == 0 && strcmp((char*)cPass, (const char*)"microchip") == 0)
        return 0x80; // We accept this combination

    // You can add additional user/pass combos here.
    // If you return specific "realm" values above, you can base this
    //   decision on what specific file or folder is being accessed.
    // You could return different values (0x80 to 0xff) to indicate
    //   various users or groups, and base future processing decisions
    //   in TCPIP_HTTP_GetExecute/Post or HTTPPrint callbacks on this value.

    return 0x00; // Provided user/pass is invalid
}
#endif

/****************************************************************************
  Section:
    GET Form Handlers
 ****************************************************************************/

/****************************************************************************
  Function:
    HTTP_IO_RESULT TCPIP_HTTP_GetExecute(HTTP_CONN_HANDLE connHandle)

  Internal:
    See documentation in the TCP/IP Stack API or http.h for details.
 ****************************************************************************/
HTTP_IO_RESULT TCPIP_HTTP_GetExecute(HTTP_CONN_HANDLE connHandle)
{
    uint8_t          filename[20];
    uint8_t*         httpDataBuff;
    TCPIP_UINT16_VAL bssIdxStr;

    // Load the file name.
    // Make sure uint8_t filename[] above is large enough for your longest name.
    SYS_FS_FileNameGet(TCPIP_HTTP_CurrentConnectionFileGet(connHandle), filename, 20);

    httpDataBuff = TCPIP_HTTP_CurrentConnectionDataBufferGet(connHandle);

    if(!memcmp(filename, "scan.cgi", 8))
    {
        const uint8_t* ptr;
        const uint8_t* ptr1;

        ptr  = TCPIP_HTTP_ArgGet(httpDataBuff, (const uint8_t*)"scan");
        ptr1 = TCPIP_HTTP_ArgGet(httpDataBuff, (const uint8_t*)"getBss");

        s_httpapp_get_param.config.data = &g_wifi_cfg;
        iwpriv_get(CONFIG_GET, &s_httpapp_get_param);

        if((ptr != NULL) && (ptr1 == NULL))
        {
            // scan request
            s_scanResultIsValid = false;

            /*
             * Display pre-scan results if pre-scan results are available,
             * otherwise initiate a new scan.
             */
            iwpriv_get(SCANRESULTS_COUNT_GET, &s_httpapp_get_param);
            //  if (s_httpapp_get_param.scan.numberOfResults == 0) {
            iwpriv_execute(SCAN_START, &s_httpapp_execute_param);
            do
            {
                iwpriv_get(SCANSTATUS_GET, &s_httpapp_get_param);
            } while(s_httpapp_get_param.scan.scanStatus == IWPRIV_SCAN_IN_PROGRESS);
            do
            {
                iwpriv_execute(SCANRESULTS_SAVE, &s_httpapp_execute_param);
            } while(s_httpapp_execute_param.scan.saveStatus == IWPRIV_IN_PROGRESS);
            // }
        }
        else if((ptr == NULL) && (ptr1 != NULL))
        {
            uint8_t bssIdx;

            // getBss request
            // use the value to get the nth bss stored on chip
            s_scanResultIsValid = false;
            bssIdxStr.v[1]      = *ptr1;
            bssIdxStr.v[0]      = *(ptr1 + 1);
            bssIdx              = hexatob(bssIdxStr.Val);

            s_httpapp_get_param.scan.index = (uint16_t)bssIdx;
            iwpriv_get(SCANRESULT_GET, &s_httpapp_get_param);
            s_scanResult = (WF_SCAN_RESULT*)s_httpapp_get_param.scan.data;

            if(s_scanResult)
            {
                if(s_scanResult->ssidLen < 32)
                    s_scanResult->ssid[s_scanResult->ssidLen] = 0;
                s_scanResultIsValid = true;
            }
        }
        else
        {
            // impossible to get here
        }
    }

    return HTTP_IO_DONE;
}

/****************************************************************************
  Section:
    POST Form Handlers
 ****************************************************************************/
#if defined(TCPIP_HTTP_USE_POST)

/****************************************************************************
  Function:
    HTTP_IO_RESULT TCPIP_HTTP_PostExecute(HTTP_CONN_HANDLE connHandle)

  Internal:
    See documentation in the TCP/IP Stack API or HTTP.h for details.
 ****************************************************************************/
HTTP_IO_RESULT TCPIP_HTTP_PostExecute(HTTP_CONN_HANDLE connHandle)
{
    // Resolve which function to use and pass along
    uint8_t filename[20];

    // Load the file name
    // Make sure uint8_t filename[] above is large enough for your longest name
    SYS_FS_FileNameGet(TCPIP_HTTP_CurrentConnectionFileGet(connHandle), filename, sizeof(filename));

    if(!strcmp((char*)filename, "settings.cgi"))
    {
        return HTTPPostWIFIConfig(connHandle, true);
    }
    return HTTPPostWIFIConfig(connHandle, false);
}

/*******************************************************************************
  Function:
    static HTTP_IO_RESULT HTTPPostWIFIConfig(HTTP_CONN_HANDLE connHandle)

  Summary:
    Processes the Wi-Fi configuration data.

  Description:
    Accepts wireless configuration data from the www site and saves them to a
    structure to be applied by the Wi-Fi module configuration manager.

    The following configurations are possible:
         i) Mode: adhoc or infrastructure
        ii) Security:
               - None
               - WEP 64-bit
               - WEP 128-bit
               - WPA Auto pre-calculated key
               - WPA1 passphrase
               - WPA2 passphrase
               - WPA Auto passphrase
       iii) Key material

    If an error occurs, such as data is invalid they will be redirected to a page
    informing the user of such results.

    NOTE: This code for modified originally from HTTPPostWIFIConfig as distributed
          by Microchip.

  Precondition:
    None.

  Parameters:
    None.

  Return Values:
    HTTP_IO_DONE - all parameters have been processed
    HTTP_IO_NEED_DATA - data needed by this function has not yet arrived
 *******************************************************************************/
#if defined(HTTP_APP_USE_WIFI)
static HTTP_IO_RESULT HTTPPostWIFIConfig(HTTP_CONN_HANDLE connHandle, bool fromsettings)
{
    uint8_t    ssidLen;
    uint32_t   byteCount;
    TCP_SOCKET sktHTTP;
    uint8_t*   httpDataBuff = 0;

    bool gotSSID = false;
    bool gotSEC  = false;
    bool gotKEY  = false;
    bool gotWLAN = false;

    bool gotGWID = false;
    bool gotASRV = false;
    bool gotGWKY = false;

    uint8_t gwid[33];
    uint8_t gwkey[255];
    uint8_t activationurl[255];

    byteCount = TCPIP_HTTP_CurrentConnectionByteCountGet(connHandle);
    sktHTTP   = TCPIP_HTTP_CurrentConnectionSocketGet(connHandle);
    if(byteCount > TCPIP_TCP_GetIsReady(sktHTTP) + TCPIP_TCP_FifoRxFreeGet(sktHTTP))
        return Helper_APP_ConfigFailure(connHandle, httpDataBuff);

    // Ensure that all data is waiting to be parsed.  If not, keep waiting for
    // all of it to arrive.
    if(TCPIP_TCP_GetIsReady(sktHTTP) < byteCount)
        return HTTP_IO_NEED_DATA;

    bool fromapp = false;

    httpDataBuff = TCPIP_HTTP_CurrentConnectionDataBufferGet(connHandle);
    // Read all browser POST data.
    while(TCPIP_HTTP_CurrentConnectionByteCountGet(connHandle))
    {
        // Read a form field name.
        if(TCPIP_HTTP_PostNameRead(connHandle, httpDataBuff, 6) != HTTP_READ_OK)
            return Helper_APP_ConfigFailure(connHandle, httpDataBuff);

        // Read a form field value.
        if(TCPIP_HTTP_PostValueRead(connHandle, httpDataBuff + 6, 152 - 6 - 2) !=
           HTTP_READ_OK) // TCPIP_HTTP_MAX_DATA_LEN
            return Helper_APP_ConfigFailure(connHandle, httpDataBuff);

        // Parse the value that was read.
        if(strcmp((char*)httpDataBuff, (const char*)"wlan") == 0)
        {
            // Get the network type: Ad-Hoc or Infrastructure.
            char networkType[6];
            if(strlen((char*)(httpDataBuff + 6)) > 5) /* Sanity check. */
            {
                gotWLAN = false;
                continue;
            }
            // return Helper_APP_ConfigFailure(connHandle, httpDataBuff);

            memcpy(networkType, (void*)(httpDataBuff + 6), strlen((char*)(httpDataBuff + 6)));
            networkType[strlen((char*)(httpDataBuff + 6))] = 0; /* Terminate string. */
            if(!strcmp((char*)networkType, (const char*)"infra"))
            {
                gotWLAN                         = true;
                g_redirectionConfig.networkType = WF_NETWORK_TYPE_INFRASTRUCTURE;
            }
            else if(strcmp((char*)networkType, "adhoc") == 0)
            {
                WF_ADHOC_NETWORK_CONTEXT adhocContext;
                g_redirectionConfig.networkType = WF_NETWORK_TYPE_ADHOC;

                // Always setup Ad-Hoc to attempt to connect first, then start.
                adhocContext.mode            = WF_DEFAULT_ADHOC_MODE;
                adhocContext.beaconPeriod    = WF_DEFAULT_ADHOC_BEACON_PERIOD;
                adhocContext.hiddenSsid      = WF_DEFAULT_ADHOC_HIDDEN_SSID;
                s_httpapp_set_param.ctx.data = &adhocContext;
                iwpriv_set(ADHOCCTX_SET, &s_httpapp_set_param);
                gotWLAN = true;
            }
            else
            {
                SYS_CONSOLE_MESSAGE("\r\nExiting Via WLAN\r\n");
                // Network type no good. :-(
                SYS_CONSOLE_MESSAGE((const char*)"\r\nInvalid redirection network type\r\n");
                if(!fromsettings)
                    return Helper_APP_ConfigFailure(connHandle, httpDataBuff);
            }

            // Save old network type.
            iwpriv_get(NETWORKTYPE_GET, &s_httpapp_get_param);
            g_redirectionConfig.prevNetworkType = s_httpapp_get_param.netType.type;
        }
        else if(strcmp((char*)httpDataBuff, "ssid") == 0)
        {
            // Get new SSID and make sure it is valid.
            if(strlen((char*)(httpDataBuff + 6)) < 33u)
            {
                memcpy(g_redirectionConfig.ssid, (void*)(httpDataBuff + 6), strlen((char*)(httpDataBuff + 6)));
                g_redirectionConfig.ssid[strlen((char*)(httpDataBuff + 6))] = 0; /* Terminate string. */

                /* Save current profile SSID for displaying later. */
                s_httpapp_get_param.ssid.ssid = g_redirectionConfig.prevSSID;
                iwpriv_get(SSID_GET, &s_httpapp_get_param);
                ssidLen                               = s_httpapp_get_param.ssid.ssidLen;
                g_redirectionConfig.prevSSID[ssidLen] = 0;
                gotSSID                               = true;
            }
            else
            {
                SYS_CONSOLE_MESSAGE("\r\nExiting Via SSID\r\n");
                // Invalid SSID... :-(
                if(!fromsettings)
                    return Helper_APP_ConfigFailure(connHandle, httpDataBuff);
            }
        }
        else if(strcmp((char*)httpDataBuff, (const char*)"sec") == 0)
        {
            char securityMode[7]; // Read security mode.

            if(strlen((char*)(httpDataBuff + 6)) > 6) /* Sanity check. */
            {
                continue;
                // return Helper_APP_ConfigFailure(connHandle, httpDataBuff);
            }

            memcpy(securityMode, (void*)(httpDataBuff + 6), strlen((char*)(httpDataBuff + 6)));
            securityMode[strlen((char*)(httpDataBuff + 6))] = 0; /* Terminate string. */

            if(strcmp((char*)securityMode, (const char*)"no") == 0)
            {
                gotSEC                           = true;
                g_redirectionConfig.securityMode = WF_SECURITY_OPEN;
            }
            else if(strcmp((char*)securityMode, (const char*)"wep40") == 0)
            {
                gotSEC                           = true;
                g_redirectionConfig.securityMode = WF_SECURITY_WEP_40;
            }
            else if(strcmp((char*)securityMode, (const char*)"wep104") == 0)
            {
                gotSEC                           = true;
                g_redirectionConfig.securityMode = WF_SECURITY_WEP_104;
            }
            else if(strcmp((char*)securityMode, (const char*)"wpa1") == 0)
            {
                g_redirectionConfig.securityMode = WF_SECURITY_WPA_WITH_PASS_PHRASE;
                gotSEC                           = true;
            }
            else if(strcmp((char*)securityMode, (const char*)"wpa2") == 0)
            {
                g_redirectionConfig.securityMode = WF_SECURITY_WPA2_WITH_PASS_PHRASE;
                gotSEC                           = true;
            }
            else if(strcmp((char*)securityMode, (const char*)"wpa") == 0)
            {
                gotSEC                           = true;
                g_redirectionConfig.securityMode = WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE;
            }
            else
            {
                SYS_CONSOLE_MESSAGE("\r\nExiting Via SEC\r\n");
                // Security mode no good. :-(
                SYS_CONSOLE_MESSAGE("\r\nInvalid redirection security mode\r\n\r\n");
                if(!fromsettings)
                    return Helper_APP_ConfigFailure(connHandle, httpDataBuff);
            }
        }
        else if(strcmp((char*)httpDataBuff, (const char*)"key") == 0)
        {
            // Read new key material.
            /*
            if (!Helper_WIFI_SecurityHandle(&g_redirectionConfig, (const char *)(httpDataBuff + 6)))
            {
                SYS_CONSOLE_MESSAGE("\r\nExiting Via KEY\r\n");
                SYS_CONSOLE_MESSAGE((const char *)(httpDataBuff + 6));
                if(!fromsettings) return Helper_APP_ConfigFailure(connHandle, httpDataBuff);
            }
            */
            if(strlen((char*)(httpDataBuff + 6)) > 63) /* Sanity check. */
            {
                continue;
            }
            g_redirectionConfig.securityKeyLen = strlen((char*)(httpDataBuff + 6));
            memcpy(g_redirectionConfig.securityKey, (void*)(httpDataBuff + 6), strlen((char*)(httpDataBuff + 6)));
            g_redirectionConfig.securityKey[strlen((char*)(httpDataBuff + 6))] = 0; /* Terminate string. */
            gotKEY                                                             = true;
        }

        if(fromsettings)
        {
            if(strcmp((char*)httpDataBuff, "gwid") == 0 && !appGWActivationData.locked)
            {
                // Get new SSID and make sure it is valid.
                // REVIEW: Use strncpy from new pointer (idem for following copies), e.g.:
                //                char *ptr = httpDataBuff + 6;
                //                if (strlen(ptr) < sizeof(gwid)) {
                //                    strncpy(gwid, ptr, sizeof(gwid));
                //                    // does not extra check for \0 termination as we know strlen(ptr) < sizeof(gwid)
                //                    SYS_PRINT("\r\nGOT GWID via post: ");
                //                    SYS_PRINT(gwid);
                //                    gotGWID=true;
                //                }
                if(strlen((char*)(httpDataBuff + 6)) <
                   33u) // REVIEW: remove magic number by using sizeof destination array
                {
                    memcpy(gwid, (void*)(httpDataBuff + 6), strlen((char*)(httpDataBuff + 6)));
                    gwid[strlen((char*)(httpDataBuff + 6))] = 0; /* Terminate string. */
                    SYS_PRINT("\r\nGOT GWID via post: ");
                    SYS_PRINT(gwid);
                    gotGWID = true;
                }
                else
                {
                    SYS_CONSOLE_MESSAGE("\r\nExiting Via GWID\r\n");
                    // Invalid SSID... :-(
                    return Helper_APP_ConfigFailure(connHandle, httpDataBuff);
                }
            }
            else if(strcmp((char*)httpDataBuff, "asrv") == 0 && !appGWActivationData.locked)
            {
                // Get new SSID and make sure it is valid.
                if(strlen((char*)(httpDataBuff + 6)) <
                   255u) // REVIEW: remove magic number by using sizeof destination array
                {
                    memcpy(activationurl, (void*)(httpDataBuff + 6), strlen((char*)(httpDataBuff + 6)));
                    activationurl[strlen((char*)(httpDataBuff + 6))] = 0; /* Terminate string. */

                    SYS_PRINT("\r\nGOT ASRV via post: ");
                    SYS_PRINT(activationurl);
                    gotASRV = true;
                }
                else
                {
                    SYS_CONSOLE_MESSAGE("\r\nExiting Via ASVR\r\n");
                    // Invalid SSID... :-(
                    return Helper_APP_ConfigFailure(connHandle, httpDataBuff);
                }
            }
            else if(strcmp((char*)httpDataBuff, "app") == 0)
            {
                fromapp = true;
            }
            else if(strcmp((char*)httpDataBuff, "gwky") == 0 && !appGWActivationData.locked)
            {
                // Get new SSID and make sure it is valid.
                if(strlen((char*)(httpDataBuff + 6)) <
                   255u) // REVIEW: remove magic number by using sizeof destination array
                {
                    memcpy(gwkey, (void*)(httpDataBuff + 6), strlen((char*)(httpDataBuff + 6)));
                    gwkey[strlen((char*)(httpDataBuff + 6))] = 0; /* Terminate string. */

                    SYS_PRINT("\r\nGOT gateway-key via post: ");
                    SYS_PRINT(gwkey);
                    gotGWKY = true;
                }
                else
                {
                    SYS_CONSOLE_MESSAGE("\r\nExiting Via GWKY\r\n");
                    // Invalid SSID... :-(
                    return Helper_APP_ConfigFailure(connHandle, httpDataBuff);
                }
            }
        }
    }

    if(appGWActivationData.locked)
    {
        // make sure that when locked no config enters the device
        gotGWID = false;
        gotGWKY = false;
        gotASRV = false;
        SYS_PRINT("Activation locked: ignoring ID, key and url\r\n");
    }
    else
    {
        SYS_PRINT("GOT GWID:%d\r\n", gotGWID);
        SYS_PRINT("GOT GWKY:%d\r\n", gotGWKY);
        SYS_PRINT("GOT ASRV:%d\r\n", gotASRV);
    }

    SYS_PRINT("GOT SSID:%d\r\n", gotSSID);
    SYS_PRINT("GOT KEY:%d\r\n", gotKEY);
    SYS_PRINT("GOT SEC:%d\r\n", gotSEC);
    SYS_PRINT("GOT WLAN:%d\r\n", gotWLAN);

    if(!gotGWID && !gotSSID)
    {
        // if we don't get a new id and no ssid then do nothing
        strcpy((char*)httpDataBuff, "/");
        TCPIP_HTTP_CurrentConnectionStatusSet(connHandle, HTTP_REDIRECT);
        return HTTP_IO_DONE;
    }

    if(gotASRV)
    {
        strncpy(appGWActivationData.configuration.account_server_url, activationurl,
                sizeof(appGWActivationData.configuration.account_server_url));
        // Make sure the string is always terminated by placing a null char in the last element of the array
        appGWActivationData.configuration
            .account_server_url[sizeof(appGWActivationData.configuration.account_server_url) - 1] = '\0';
        SYS_PRINT("Set ASRV to application\r\n");
    }

    if(gotGWID)
    {
        strncpy(appGWActivationData.configuration.id, gwid, sizeof(appGWActivationData.configuration.id));
        // Make sure the string is always terminated by placing a null char in the last element of the array
        appGWActivationData.configuration.id[sizeof(appGWActivationData.configuration.id) - 1] = '\0';
        SYS_PRINT("Set GWID to application\r\n");
    }

    if(gotGWID && gotGWKY) // got a key already, lets not activate.
    {
        strncpy(appGWActivationData.configuration.key, gwkey, sizeof(appGWActivationData.configuration.key));
        // Make sure the string is always terminated by placing a null char in the last element of the array
        appGWActivationData.configuration.key[sizeof(appGWActivationData.configuration.key) - 1] = '\0';
        SYS_PRINT("Set GWKY to application\r\n");
    }

    // happy flow -> user connected over ethernet, no wifi setup, only activation.
    if(gotGWID && gotASRV && !gotSSID)
    {
        SYS_PRINT("Performing ethernet activation\r\n");
        if(!fromapp)
        {
            strcpy((char*)httpDataBuff, "/activate");
            TCPIP_HTTP_CurrentConnectionStatusSet(connHandle, HTTP_REDIRECT);
        }
        uint16_t redirection_delay = SYS_TMR_TickCounterFrequencyGet() * HTTP_APP_REDIRECTION_DELAY_TIME;
        SYS_TMR_CallbackSingle(redirection_delay, 0, Helper_APP_RedirectionFlagSet); // Set the redirect call back as
                                                                                     // well even if no WiFi data is
                                                                                     // configured, as for now it will
                                                                                     // case a reboot as well
        g_config_changed = true;
        return HTTP_IO_DONE;
    }
    else if(gotGWID && gotASRV && gotSSID && gotKEY && gotSEC &&
            gotWLAN) // second happy flow -> user wants to reconnect to wifi, and then activate
    {
        if((g_redirectionConfig.networkType == WF_NETWORK_TYPE_ADHOC) &&
           ((g_redirectionConfig.securityMode == WF_SECURITY_WPA_WITH_PASS_PHRASE) ||
            (g_redirectionConfig.securityMode == WF_SECURITY_WPA2_WITH_PASS_PHRASE) ||
            (g_redirectionConfig.securityMode == WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE)))
            return Helper_APP_ConfigFailure(connHandle, httpDataBuff);

        s_httpapp_get_param.config.data = &g_wifi_cfg;
        iwpriv_get(CONFIG_GET, &s_httpapp_get_param);
        strcpy((char*)g_wifi_cfg.ssid, (char*)g_redirectionConfig.ssid);
        g_wifi_cfg.ssidLen      = strlen((char*)(g_redirectionConfig.ssid));
        g_wifi_cfg.securityMode = g_redirectionConfig.securityMode;
        if(g_redirectionConfig.securityMode != WF_SECURITY_OPEN)
            Helper_WIFI_KeySave(&g_redirectionConfig, &g_wifi_cfg);
        g_wifi_cfg.networkType          = g_redirectionConfig.networkType;
        s_httpapp_set_param.config.data = &g_wifi_cfg;
        iwpriv_set(CONFIG_SET, &s_httpapp_set_param);

        if(!fromapp)
        {
            strcpy((char*)httpDataBuff, "/reconnect");
            TCPIP_HTTP_CurrentConnectionStatusSet(connHandle, HTTP_REDIRECT);
            uint16_t redirection_delay = SYS_TMR_TickCounterFrequencyGet() * HTTP_APP_REDIRECTION_DELAY_TIME;
            SYS_TMR_CallbackSingle(redirection_delay, 0, Helper_APP_RedirectionFlagSet);
        }
        else
        {
            g_redirect_signal = true;
        }

        g_config_changed = true;
        return HTTP_IO_DONE;
    }

    if(!gotSSID || !gotSEC || !gotKEY || !gotWLAN)
        return HTTP_IO_DONE;
    /* Check if WPA hasn't been selected with Ad-Hoc, if it has we choke! */
    if((g_redirectionConfig.networkType == WF_NETWORK_TYPE_ADHOC) &&
       ((g_redirectionConfig.securityMode == WF_SECURITY_WPA_WITH_PASS_PHRASE) ||
        (g_redirectionConfig.securityMode == WF_SECURITY_WPA2_WITH_PASS_PHRASE) ||
        (g_redirectionConfig.securityMode == WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE)))
        return Helper_APP_ConfigFailure(connHandle, httpDataBuff);

    /*
     * All parsing complete!  If we have got to here all data has been validated and
     * We can handle what is necessary to start the reconfigure process of the Wi-Fi device.
     */

    /* Copy Wi-Fi cfg data to be committed to NVM. */
    s_httpapp_get_param.config.data = &g_wifi_cfg;
    iwpriv_get(CONFIG_GET, &s_httpapp_get_param);
    strcpy((char*)g_wifi_cfg.ssid, (char*)g_redirectionConfig.ssid);
    g_wifi_cfg.ssidLen = strlen((char*)(g_redirectionConfig.ssid));
    /* Going to set security type. */
    g_wifi_cfg.securityMode = g_redirectionConfig.securityMode;
    /* Going to save the key, if required. */
    if(g_redirectionConfig.securityMode != WF_SECURITY_OPEN)
        Helper_WIFI_KeySave(&g_redirectionConfig, &g_wifi_cfg);
    /* Going to save the network type. */
    g_wifi_cfg.networkType          = g_redirectionConfig.networkType;
    s_httpapp_set_param.config.data = &g_wifi_cfg;
    iwpriv_set(CONFIG_SET, &s_httpapp_set_param);

    if(!fromapp)
    {
        strcpy((char*)httpDataBuff, "/reconnect");
        TCPIP_HTTP_CurrentConnectionStatusSet(connHandle, HTTP_REDIRECT);

        /* Set 1s delay before redirection, goal is to display the redirection web page. */
        uint16_t redirection_delay = SYS_TMR_TickCounterFrequencyGet() * HTTP_APP_REDIRECTION_DELAY_TIME;
        SYS_TMR_CallbackSingle(redirection_delay, 0, Helper_APP_RedirectionFlagSet);
    }
    else
    {
        g_redirect_signal = true;
    }
    g_config_changed = true;
    return HTTP_IO_DONE;
}
#endif // defined(HTTP_APP_USE_WIFI)

#endif // defined(TCPIP_HTTP_USE_POST)

/****************************************************************************
  Section:
    Dynamic Variable Callback Functions
 ****************************************************************************/

/****************************************************************************
  Function:
    void TCPIP_HTTP_Print_varname(void)

  Internal:
    See documentation in the TCP/IP Stack API or HTTP.h for details.
 ****************************************************************************/
void TCPIP_HTTP_Print_scan(HTTP_CONN_HANDLE connHandle)
{
    uint8_t    scanInProgressString[4];
    TCP_SOCKET sktHTTP = TCPIP_HTTP_CurrentConnectionSocketGet(connHandle);

    iwpriv_get(SCANSTATUS_GET, &s_httpapp_get_param);
    if(s_httpapp_get_param.scan.scanStatus == IWPRIV_SCAN_IN_PROGRESS)
        uitoa((uint16_t) true, scanInProgressString);
    else
        uitoa((uint16_t) false, scanInProgressString);
    TCPIP_TCP_StringPut(sktHTTP, scanInProgressString);
}

void TCPIP_HTTP_Print_prevSSID(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP = TCPIP_HTTP_CurrentConnectionSocketGet(connHandle);

    TCPIP_TCP_StringPut(sktHTTP, (uint8_t*)g_redirectionConfig.prevSSID); // prevSSID
}

void TCPIP_HTTP_Print_nextSSID(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP = TCPIP_HTTP_CurrentConnectionSocketGet(connHandle);

    TCPIP_TCP_StringPut(sktHTTP, (uint8_t*)g_redirectionConfig.ssid); // nextSSID
}

void TCPIP_HTTP_Print_prevWLAN(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP = TCPIP_HTTP_CurrentConnectionSocketGet(connHandle);

    if(g_redirectionConfig.prevNetworkType == WF_NETWORK_TYPE_INFRASTRUCTURE)
    {
        TCPIP_TCP_StringPut(sktHTTP, (const uint8_t*)"Infrastructure (BSS)");
    }
    else if(g_redirectionConfig.prevNetworkType == WF_NETWORK_TYPE_ADHOC)
    {
        TCPIP_TCP_StringPut(sktHTTP, (const uint8_t*)"Ad-Hoc (IBSS)");
    }
    else if(g_redirectionConfig.prevNetworkType == WF_NETWORK_TYPE_SOFT_AP)
    {
        TCPIP_TCP_StringPut(sktHTTP, (const uint8_t*)"Soft AP (BSS)");
    }
    else
    {
        TCPIP_TCP_StringPut(sktHTTP, (const uint8_t*)"Unknown");
    }
}

void TCPIP_HTTP_Print_currWLAN(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP = TCPIP_HTTP_CurrentConnectionSocketGet(connHandle);

    // index.htm differs between Easy Configuration and Wi-Fi G Demo
    // in Wi-Fi G Demo it displays BSS information of MRF24WG.
    s_httpapp_get_param.config.data = &g_wifi_cfg;
    iwpriv_get(CONFIG_GET, &s_httpapp_get_param);
    if(g_wifi_cfg.networkType == WF_NETWORK_TYPE_ADHOC)
        TCPIP_TCP_StringPut(sktHTTP, (const uint8_t*)"Ad-Hoc (IBSS)");
    else if(g_wifi_cfg.networkType == WF_NETWORK_TYPE_INFRASTRUCTURE)
        TCPIP_TCP_StringPut(sktHTTP, (const uint8_t*)"Infrastructure (BSS)");
    else if(g_wifi_cfg.networkType == WF_NETWORK_TYPE_SOFT_AP)
        TCPIP_TCP_StringPut(sktHTTP, (const uint8_t*)"Soft AP (BSS)");
    else if(g_wifi_cfg.networkType == WF_NETWORK_TYPE_P2P)
        TCPIP_TCP_StringPut(sktHTTP, (const uint8_t*)"Wi-Fi Direct (P2P)");
    else
        TCPIP_TCP_StringPut(sktHTTP, (const uint8_t*)"Unknown");
}

void TCPIP_HTTP_Print_nextWLAN(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP = TCPIP_HTTP_CurrentConnectionSocketGet(connHandle);

    if(g_redirectionConfig.networkType == WF_NETWORK_TYPE_INFRASTRUCTURE)
    {
        TCPIP_TCP_StringPut(sktHTTP, (const uint8_t*)"Infrastructure (BSS)");
    }
    else if(g_redirectionConfig.networkType == WF_NETWORK_TYPE_ADHOC)
    {
        TCPIP_TCP_StringPut(sktHTTP, (const uint8_t*)"Ad-Hoc (IBSS)");
    }
    else if(g_redirectionConfig.networkType == WF_NETWORK_TYPE_SOFT_AP)
    {
        TCPIP_TCP_StringPut(sktHTTP, (const uint8_t*)"Soft AP (BSS)");
    }
    else
    {
        TCPIP_TCP_StringPut(sktHTTP, (const uint8_t*)"Unknown");
    }
}

// json_builder_free
uint8_t output_started = 0;
char*   buf;

void TCPIP_HTTP_Print_scanresult(HTTP_CONN_HANDLE connHandle)
{
    s_httpapp_get_param.config.data = &g_wifi_cfg;

    /*
    TCP_SOCKET sktHTTP = TCPIP_HTTP_CurrentConnectionSocketGet(connHandle);
    if (s_httpapp_get_param.scan.scanStatus == IWPRIV_SCAN_IN_PROGRESS)
    {
     TCPIP_TCP_StringPut(sktHTTP, (const uint8_t *)"BUSY");
    }
    else
    {

        uint8_t ap_iterator2=0;
        iwpriv_get(SCANRESULTS_COUNT_GET, &s_httpapp_get_param);
        uint8_t scanLen2 = s_httpapp_get_param.scan.numberOfResults;
        iwpriv_get(CONFIG_GET, &s_httpapp_get_param);
        SYS_PRINT("\r\n SCAN LENGHT: %d\r\n",scanLen2);
        for(ap_iterator2=0;ap_iterator2<scanLen2;ap_iterator2++){
            s_httpapp_get_param.scan.index = (uint16_t)ap_iterator2;
            iwpriv_get(SCANRESULT_GET, &s_httpapp_get_param);
            s_scanResult = (WF_SCAN_RESULT *)s_httpapp_get_param.scan.data;
            if(s_scanResult){
                if (s_scanResult->ssidLen < 32)s_scanResult->ssid[s_scanResult->ssidLen] = 0;
                SYS_PRINT("\r\n -- NAME: ");
                SYS_PRINT(s_scanResult->ssid);
                SYS_PRINT("\r\n -- RSSI: %d",s_scanResult->rssi);
            }
        }
        TCPIP_TCP_StringPut(sktHTTP, (const uint8_t *)"OK");
    }
    return;
    */

    uint32_t callbackPos;
    callbackPos = TCPIP_HTTP_CurrentConnectionCallbackPosGet(connHandle);
    if(callbackPos == 0x00u)
    {
        uint8_t     ap_iterator = 0;
        json_value* arr         = json_array_new(0);
        json_value* obj         = json_object_new(0);
        iwpriv_get(SCANRESULTS_COUNT_GET, &s_httpapp_get_param);
        uint8_t scanLen = s_httpapp_get_param.scan.numberOfResults;
        iwpriv_get(CONFIG_GET, &s_httpapp_get_param);
        json_value* ssidobj[scanLen];
        SYS_PRINT("\r\n SCAN LENGHT: %d\r\n", scanLen);

        for(ap_iterator = 0; ap_iterator < scanLen; ap_iterator++)
        {
            ssidobj[ap_iterator]           = json_object_new(0);
            s_httpapp_get_param.scan.index = (uint16_t)ap_iterator;
            iwpriv_get(SCANRESULT_GET, &s_httpapp_get_param);
            s_scanResult = (WF_SCAN_RESULT*)s_httpapp_get_param.scan.data;
            if(s_scanResult)
            {
                if(s_scanResult->ssidLen > 0 && s_scanResult->ssidLen < 32)
                {
                    s_scanResult->ssid[s_scanResult->ssidLen] = 0;
                    SYS_PRINT("\r\n -- NAME: ");
                    SYS_PRINT(s_scanResult->ssid);

                    uint8_t security = (s_scanResult->apConfig & 0xd0) >> 4;

                    json_object_push(ssidobj[ap_iterator], "name", json_string_new(s_scanResult->ssid));
                    json_object_push(ssidobj[ap_iterator], "rssi", json_integer_new(-1 * s_scanResult->rssi));

                    if(security == 0)
                        json_object_push(ssidobj[ap_iterator], "sec", json_string_new("no"));
                    if(security == 1)
                        json_object_push(ssidobj[ap_iterator], "sec", json_string_new("wep"));
                    if(security == 5)
                        json_object_push(ssidobj[ap_iterator], "sec", json_string_new("wpa1"));
                    if(security == 9 || security == 13)
                        json_object_push(ssidobj[ap_iterator], "sec", json_string_new("wpa2"));

                    if(s_scanResult->bssType == 1)
                        json_object_push(ssidobj[ap_iterator], "type", json_string_new("infra"));
                    else
                        json_object_push(ssidobj[ap_iterator], "type", json_string_new("adhoc"));

                    json_array_push(arr, ssidobj[ap_iterator]);
                    SYS_PRINT("\r\n -- RSSI: %d", s_scanResult->rssi);
                }
                else
                {
                    SYS_PRINT("\r\n -- ignoring SSID with length: %d", s_scanResult->ssidLen);
                }
            }
        }
        SYS_PRINT("\r\n");
        json_object_push(obj, "APS", arr);
        bool scanning = (s_httpapp_get_param.scan.scanStatus == IWPRIV_SCAN_IN_PROGRESS);
        json_object_push(obj, "scanning", json_boolean_new(scanning));
        buf = malloc(json_measure(obj));
        json_serialize(buf, obj);

        callbackPos    = (uint32_t)buf;
        output_started = 0;

        json_builder_free(obj);
    }

    callbackPos =
        (uint32_t)TCPIP_TCP_StringPut(TCPIP_HTTP_CurrentConnectionSocketGet(connHandle), (uint8_t*)callbackPos);
    if(*(uint8_t*)callbackPos == '\0')
    {
        callbackPos    = 0x00;
        output_started = 1;
        free(buf);
    }

    TCPIP_HTTP_CurrentConnectionCallbackPosSet(connHandle, callbackPos);
}

char* settingsbuffer;

void TCPIP_HTTP_Print_gwsettings(HTTP_CONN_HANDLE connHandle)
{
    s_httpapp_get_param.config.data = &g_wifi_cfg;

    uint32_t callbackPos;
    callbackPos = TCPIP_HTTP_CurrentConnectionCallbackPosGet(connHandle);
    if(callbackPos == 0x00u)
    {

        static uint8_t ssidString[33];
        s_httpapp_get_param.ssid.ssid = ssidString;
        iwpriv_get(SSID_GET, &s_httpapp_get_param);

        json_value* obj = json_object_new(0);
        json_object_push(obj, "ssid", json_string_new(ssidString));
        json_object_push(obj, "gwid", json_string_new(appGWActivationData.configuration.id));
        json_value* json_key;
        if(appGWActivationData.locked)
        {
            int key_len = strlen(appGWActivationData.configuration.key);
            if(key_len > 20)
            {
                json_key = json_string_new(&appGWActivationData.configuration.key[key_len - 20]);
            }
            else
            {
                json_key = json_string_new("");
            }
        }
        else
        {
            json_key = json_string_new(appGWActivationData.configuration.key);
        }
        json_object_push(obj, "gwkey", json_key);
        json_object_push(obj, "asrv", json_string_new(appGWActivationData.configuration.account_server_url));
        json_object_push(obj, "locked", json_boolean_new(appGWActivationData.locked));
        settingsbuffer = malloc(json_measure(obj));
        json_serialize(settingsbuffer, obj);

        callbackPos = (uint32_t)settingsbuffer;
        json_builder_free(obj);
    }
    callbackPos =
        (uint32_t)TCPIP_TCP_StringPut(TCPIP_HTTP_CurrentConnectionSocketGet(connHandle), (uint8_t*)callbackPos);
    if(*(uint8_t*)callbackPos == '\0')
    {
        callbackPos = 0x00;
        free(settingsbuffer);
    }
    TCPIP_HTTP_CurrentConnectionCallbackPosSet(connHandle, callbackPos);
}

void TCPIP_HTTP_Print_gwstatus(HTTP_CONN_HANDLE connHandle)
{
    s_httpapp_get_param.config.data = &g_wifi_cfg;

    uint32_t callbackPos;
    callbackPos = TCPIP_HTTP_CurrentConnectionCallbackPosGet(connHandle);
    if(callbackPos == 0x00u)
    {

        static uint8_t ssidString[33];
        s_httpapp_get_param.ssid.ssid = ssidString;
        iwpriv_get(SSID_GET, &s_httpapp_get_param);

        json_value* obj = json_object_new(0);
        json_object_push(obj, "gateway", json_boolean_new(true));
        json_object_push(obj, "hwversion", json_string_new("v1"));

        const bootloader_version_t* pBootloaderVersion = BootloaderVersion_Get();
        const char                  ts_fmt_str[]       = "%Y-%m-%dT%H:%M:%SZ";
        char                        ts_bl_str[100];
        char                        ts_fw_str[100];
        time_t                      ts_bl = pBootloaderVersion->timestamp;
        strftime(ts_bl_str, sizeof(ts_bl_str), ts_fmt_str, gmtime(&ts_bl));
        time_t ts_fw = VERSION_TIMESTAMP;
        strftime(ts_fw_str, sizeof(ts_fw_str), ts_fmt_str, gmtime(&ts_fw));

        char bootloader_version[255] = {0};
        snprintf(bootloader_version, sizeof(bootloader_version) - 1, "r%u-%08x (%s)", pBootloaderVersion->revision,
                 pBootloaderVersion->commit, ts_bl_str);
        json_object_push(obj, "blversion", json_string_new(bootloader_version));

        char firmware_version[255] = {0};
        snprintf(firmware_version, sizeof(firmware_version) - 1, "v%d.%d.%d-%08x (%s)", VERSION_MAJOR, VERSION_MINOR,
                 VERSION_PATCH, VERSION_COMMIT, ts_fw_str);
        json_object_push(obj, "fwversion", json_string_new(firmware_version));

        json_object_push(obj, "uptime", json_integer_new(SYS_TMR_SystemCountGet() / SYS_TMR_SystemCountFrequencyGet()));
        json_object_push(obj, "connected", json_boolean_new(hasNetwork()));

        if(APP_WIFI_Has_LinkINFRA() && APP_ETH_Has_Link())
            json_object_push(obj, "interface", json_string_new("WIFI & Ethernet"));
        else if(!APP_WIFI_Has_LinkINFRA() && APP_ETH_Has_Link())
            json_object_push(obj, "interface", json_string_new("Ethernet"));
        else if(APP_WIFI_Has_LinkINFRA() && !APP_ETH_Has_Link())
            json_object_push(obj, "interface", json_string_new("WIFI"));
        else
            json_object_push(obj, "interface", json_string_new("No"));
        json_object_push(obj, "ssid", json_string_new(ssidString));

        json_object_push(obj, "activation_locked", json_boolean_new(appGWActivationData.locked));
        json_object_push(obj, "configured", json_boolean_new(APP_LORA_HAS_CORRECT_FREQ_PLAN()));
        json_object_push(obj, "region", json_string_new(appGWActivationData.configuration.frequency_plan));
        if(APP_LORA_GW_CARD_VERSION() == LORA_BAND_868)
            json_object_push(obj, "gwcard", json_string_new("868Mhz"));
        else if(APP_LORA_GW_CARD_VERSION() == LORA_BAND_915)
            json_object_push(obj, "gwcard", json_string_new("915Mhz"));
        else
            json_object_push(obj, "gwcard", json_string_new("ND"));

        json_object_push(obj, "connbroker", json_boolean_new(GatewayIsOperational()));
        uint32_t pup = 0, pdown = 0;
        getPacketCount(&pup, &pdown);
        json_object_push(obj, "pup", json_integer_new(pup));
        json_object_push(obj, "pdown", json_integer_new(pdown));

        json_object_push(obj, "estor", json_boolean_new(APP_SDCARD_IsMounted()));

        settingsbuffer = malloc(json_measure(obj));
        ;
        json_serialize(settingsbuffer, obj);

        callbackPos = (uint32_t)settingsbuffer;
        json_builder_free(obj);
    }
    callbackPos =
        (uint32_t)TCPIP_TCP_StringPut(TCPIP_HTTP_CurrentConnectionSocketGet(connHandle), (uint8_t*)callbackPos);
    if(*(uint8_t*)callbackPos == '\0')
    {
        callbackPos = 0x00;
        free(settingsbuffer);
    }
    TCPIP_HTTP_CurrentConnectionCallbackPosSet(connHandle, callbackPos);
}
