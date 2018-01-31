// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#include "app.h"
#include "app_http_request.h"
#include "app_http_ttn.h"
#include "app_ota.h"
#include "app_sdcard.h"
#include "crypto/crypto.h"
#include "subsystem_controller.h"
#include "utilities.h"

APP_OTA_DATA                  appOTAData;
extern APP_GW_ACTIVATION_DATA appGWActivationData;
extern http_request           request;

char APP_URL_Buffer[255];

char* checksums_addr = "checksums";
char* firmware_addr  = "firmware.hex";

bool http_response_split(char** response_p, size_t* header_length, uint16_t* status_code, char** content_p,
                         size_t* content_length);

void reset_http_request(http_request* request)
{
    memset(request->url, 0, sizeof(request->url));
    memset(request->urlheaders, 0, sizeof(request->urlheaders));
    memset(request->response_buffer, 0, sizeof(request->response_buffer));
    memset(request->data_buffer, 0, sizeof(request->data_buffer));
    request->status_code     = 0;
    request->content_length  = 0;
    request->content_pointer = 0;
    request->new_data_flag   = 0;
}

void APP_OTA_Enter(void)
{
    reset_http_request(&request);
    appOTAData.file_type = KEY;

    appOTAData.state = APP_OTA_INIT;
}

void APP_OTA_Leave(void)
{
    APP_HTTP_Request_CloseIfNeeded();
}

APP_OTA_STATES APP_OTA_State()
{
    return appOTAData.state;
}

void APP_OTA_Reset()
{
    reset_http_request(&request);
    appOTAData.state = APP_OTA_INIT;
}

void APP_OTA_Tasks(void)
{
    switch(appOTAData.state)
    {
        case APP_OTA_INIT:
        {
            if(APP_SERIALFLASH_IsReady())
            {
                if(!APP_SERIALFLASH_HasFOTAData())
                {
                    APP_SERIALFLASH_EraseFOTAData();
                }
                appOTAData.state = APP_OTA_WAIT_FOR_START;
            }
            else if(APP_SERIALFLASH_HasError())
            {
                appOTAData.state = APP_OTA_STORAGE_ERROR;
            }
            break;
        }

        case APP_OTA_WAIT_FOR_START:
        {
            if(APP_SERIALFLASH_IsReady())
            {
                APP_SERIALFLASH_LoadFOTAData();
                appOTAData.state = APP_OTA_CONNECTING;
            }
            else if(APP_SERIALFLASH_HasError())
            {
                appOTAData.state = APP_OTA_STORAGE_ERROR;
            }
            break;
        }

        case APP_OTA_CONNECTING:
        {
            strcpy(APP_URL_Buffer, appGWActivationData.configuration.firmware_url);
            SYS_DEBUG(SYS_ERROR_DEBUG, "FIRM: APP_URL_Buffer: %s\r\n", APP_URL_Buffer);

            if(APP_URL_Buffer[0] != '\0')
            {
                SYS_DEBUG(SYS_ERROR_DEBUG, "FIRM: Valid URL\r\n");
                if(APP_HTTP_ParseUrl(APP_URL_Buffer, &request.host, &request.path, &request.port, &request.tls))
                {
                    SYS_DEBUG(SYS_ERROR_ERROR, "FIRM: Could not parse URL '%s'\r\n", APP_URL_Buffer);
                    APP_URL_Buffer[0] = '\0';
                    appOTAData.state  = APP_OTA_ERROR;
                    break;
                }
                else
                {
                    if(appOTAData.file_type == KEY)
                    {
                        appOTAData.state = APP_OTA_REQUEST_KEY;
                    }
                    else
                    {
                        appOTAData.state = APP_OTA_REQUEST_FIRMWARE;
                    }
                }
            }
            else
            {
                SYS_DEBUG(SYS_ERROR_FATAL, "FIRM: EMPTY OR INVALID REQUEST URL\r\n");
                appOTAData.state = APP_OTA_ERROR;
            }
            break;
        }

        case APP_OTA_REQUEST_KEY:
        {
            SYS_DEBUG(SYS_ERROR_INFO, "FIRM: Requesting key ...\r\n");
            appOTAData.file_type = KEY;

            sprintf(request.urlheaders,
                    "GET /%s/%s HTTP/1.1\r\n"
                    "User-Agent: TTNGateway\r\n"
                    "Host: %s\r\n"
                    "Connection: close\r\n\r\n",
                    request.path, checksums_addr, request.host);

            APP_HTTP_Request_Initialize();

            appOTAData.state = APP_OTA_REQUESTING_DATA;
            break;
        }

        case APP_OTA_REQUEST_FIRMWARE:
        {
            SYS_DEBUG(SYS_ERROR_INFO, "FIRM: Requesting firmware ...\r\n");
            appOTAData.file_type = FIRMWARE;

            reset_http_request(&request);

            sprintf(request.urlheaders,
                    "GET /%s/%s HTTP/1.1\r\n"
                    "User-Agent: TTNGateway\r\n"
                    "Host: %s\r\n"
                    "Connection: close\r\n\r\n",
                    request.path, firmware_addr, request.host);
            request.bulk_request = 1;

            APP_HTTP_Request_Initialize();

            appOTAData.state = APP_OTA_REQUESTING_DATA;
            break;
        }

        case APP_OTA_REQUESTING_DATA:
        {
            APP_HTTP_Request_Tasks();

            if(request.bulk_request == 0 && APP_HTTP_Request_State() == APP_HTTP_REQUEST_HTTP_DONE)
            {
                appOTAData.state = APP_OTA_PARSING_RESPONSE;
            }
            else if(request.bulk_request == 1 && request.new_data_flag == 1 &&
                    APP_HTTP_Request_State() == APP_HTTP_REQUEST_WAIT_FOR_RESPONSE_BULK)
            {
                appOTAData.state = APP_OTA_PARSING_RESPONSE;
            }
            else if(APP_HTTP_Request_State() == APP_HTTP_REQUEST_ERROR)
            {
                appOTAData.state = APP_OTA_ERROR;
            }
            break;
        }

        case APP_OTA_PARSING_RESPONSE:
        {
            SYS_DEBUG(SYS_ERROR_WARNING, "FIRM: Starting download\r\n");
            char* p = (char*)&request.response_buffer;
            http_response_split(&p, &request.header_length, &request.status_code, &request.content_pointer,
                                &request.content_length);

            if(request.content_pointer == NULL || request.status_code != 200)
            {
                SYS_DEBUG(SYS_ERROR_WARNING, "FIRM: No data present\r\n");
                appOTAData.state = APP_OTA_DONE;
                break;
            }
            else
            {
                request.available_bytes = request.available_bytes - request.header_length;
                SYS_DEBUG(SYS_ERROR_INFO, "FIRM: available bytes: %u\r\n", request.available_bytes);
            }

            if(APP_SERIALFLASH_IsReady())
            {
                if(appOTAData.file_type == KEY)
                {
                    // Convert downloaded key to Hex for later comparison with current fw key in flash
                    AsciiStringToHex(request.content_pointer, appOTAData.sha256_recv_bin, CRYPT_SHA256_DIGEST_SIZE);

                    uint8_t fota_key[FLASH_LENGTH_FIRMWARE_DATA_SHA256];
                    APP_SERIALFLASH_GetFOTAChecksum(fota_key);

                    if(memcmp(appOTAData.sha256_recv_bin, fota_key, CRYPT_SHA256_DIGEST_SIZE) != 0)
                    {
                        PRINT_ARRAY_UINT8(appOTAData.sha256_recv_bin, CRYPT_SHA256_DIGEST_SIZE, "FIRM",
                                          "Downloaded FOTA key");
                        PRINT_ARRAY_UINT8(fota_key, CRYPT_SHA256_DIGEST_SIZE, "FIRM", "Stored FOTA key");
                        APP_SERIALFLASH_EraseFOTA();
                    }
                    else
                    {
                        PRINT_ARRAY_UINT8(appOTAData.sha256_recv_bin, CRYPT_SHA256_DIGEST_SIZE, "FIRM",
                                          "Downloaded FOTA key");
                        PRINT_ARRAY_UINT8(fota_key, CRYPT_SHA256_DIGEST_SIZE, "FIRM", "Stored FOTA key");
                        SYS_DEBUG(SYS_ERROR_WARNING, "FIRM: Firmware is already downloaded\r\n");
                        appOTAData.state = APP_OTA_DONE;
                        break;
                    }
                }
                else if(appOTAData.file_type == FIRMWARE)
                {
                    APP_SERIALFLASH_InitFOTA((uint32_t)request.content_length);
                }
                appOTAData.state = APP_OTA_WRITING_TO_STORAGE;
                break;
            }
            else if(APP_SERIALFLASH_HasError())
            {
                appOTAData.state = APP_OTA_STORAGE_ERROR;
                break;
            }
            break;
        }

        case APP_OTA_WRITING_TO_STORAGE:
        {
            if(APP_SERIALFLASH_HasError())
            {
                appOTAData.state = APP_OTA_STORAGE_ERROR;
                break;
            }

            if(APP_SERIALFLASH_IsReady())
            {
                if(appOTAData.file_type == KEY)
                {
                    APP_SERIALFLASH_SaveFOTAChecksum(appOTAData.sha256_recv_bin);

                    appOTAData.state = APP_OTA_FINISH_WRITING;

                    break;
                }

                if(request.new_data_flag == 1)
                {
                    SYS_DEBUG(SYS_ERROR_DEBUG, "FIRM: Writing %u bytes\r\n", request.available_bytes);

                    APP_SERIALFLASH_SaveFOTAImage(request.content_pointer, request.available_bytes);

                    // After the first write, the content pointer can point to
                    // the begin of the response buffer again.
                    request.content_pointer = request.response_buffer;
                    request.new_data_flag   = 0;
                }
            }

            if(appOTAData.file_type == FIRMWARE)
            {
                APP_HTTP_Request_Tasks();

                if(APP_HTTP_Request_State() == APP_HTTP_REQUEST_HTTP_DONE)
                {
                    appOTAData.state = APP_OTA_FINISH_WRITING;
                    break;
                }
                else if(APP_HTTP_Request_State() == APP_HTTP_REQUEST_ERROR)
                {
                    SYS_DEBUG(SYS_ERROR_WARNING, "FIRM: TCPIP ERROR\r\n");
                    appOTAData.state = APP_OTA_DONE;
                    break;
                }
            }
            break;
        }

        case APP_OTA_FINISH_WRITING:
        {
            if(APP_SERIALFLASH_HasError())
            {
                appOTAData.state = APP_OTA_STORAGE_ERROR;
                break;
            }

            if(APP_SERIALFLASH_IsReady())
            {
                SYS_DEBUG(SYS_ERROR_WARNING, "FIRM: Finishing writing to storage\r\n");
                if(appOTAData.file_type == KEY)
                {
                    if(memcmp(appOTAData.sha256_recv_bin, appGWActivationData.current_firmware_key,
                              CRYPT_SHA256_DIGEST_SIZE) != 0)
                    {
                        SYS_DEBUG(SYS_ERROR_WARNING, "FIRM: New firmware found, start downloading!\r\n");
                        PRINT_ARRAY_UINT8(appGWActivationData.current_firmware_key, CRYPT_SHA256_DIGEST_SIZE, "FIRM",
                                          "OLD key");
                        PRINT_ARRAY_UINT8(appOTAData.sha256_recv_bin, CRYPT_SHA256_DIGEST_SIZE, "FIRM", "NEW key");
                        appOTAData.state = APP_OTA_REQUEST_FIRMWARE;
                        break;
                    }
                    else
                    {
                        SYS_DEBUG(SYS_ERROR_WARNING, "FIRM: Downloaded key is the same as current key\r\n");
                        appOTAData.state = APP_OTA_DONE;
                        break;
                    }
                }
                else
                {
                    APP_SERIALFLASH_FinalizeFOTA();
                    appOTAData.state = APP_OTA_VERIFY_FIRMWARE;
                    break;
                }
            }
            break;
        }

        case APP_OTA_VERIFY_FIRMWARE:
        {
            if(APP_SERIALFLASH_IsReady())
            {
                // REBOOOOT
                SYS_DEBUG(SYS_ERROR_DEBUG, "FIRM: Downloading finished, waiting for reboot\r\n");
                appOTAData.state = APP_OTA_READY_FOR_REBOOT;
            }
            else if(APP_SERIALFLASH_HasError())
            {
                appOTAData.state = APP_OTA_STORAGE_ERROR;
            }
            break;
        }

        case APP_OTA_DONE:
        {
            appOTAData.state = APP_OTA_INIT;
            break;
        }

        case APP_OTA_ERROR:
        {
            break;
        }

        case APP_OTA_STORAGE_ERROR:
        {
            break;
        }

        case APP_OTA_READY_FOR_REBOOT:
        {
            break;
        }

        default:
        {
            break;
        }
    }
}

bool http_response_split(char** response_p, size_t* header_length, uint16_t* status_code, char** content_p,
                         size_t* content_length)
{
    bool  status             = 0;
    char  delimiter[5]       = "\r\n\r\n";
    char  delimiter_stat[10] = "HTTP/1.1 ";
    char  delimiter_len[17]  = "Content-Length: ";
    char* ptoken;
    char* cont;
    char* stat_code;
    char* cont_len;

    /* Split header and content */
    cont = strstr(*response_p, delimiter);
    if(cont != NULL)
    {
        /* Point to the actual content */
        *content_p = cont + strlen(delimiter);
        status     = 1;
    }

    if(*response_p < *content_p)
        *header_length = *content_p - *response_p;
    else
        *header_length = 0;

    /* Find status code */
    ptoken    = strtok(*response_p, "\r\n");
    stat_code = strstr(ptoken, delimiter_stat);
    if(stat_code != NULL)
    {
        stat_code    = stat_code + strlen(delimiter_stat);
        *status_code = atoi(stat_code);
    }
    else
    {
        *status_code = 0;
        status *= 0;
    }

    /* Find content length */
    *content_length = 0;
    while(ptoken != NULL)
    {
        ptoken   = strtok(NULL, "\r\n");
        cont_len = strstr(ptoken, delimiter_len);
        if(cont_len != NULL)
        {
            cont_len        = cont_len + strlen(delimiter_len);
            *content_length = atoi(cont_len);
            status *= 1;
            break;
        }
    }

    return status;
}
