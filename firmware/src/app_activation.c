// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#include "app.h"
#include "app_http_request.h"
#include "app_http_ttn.h"
#include "app_activation.h"

#include "subsystem_controller.h"

static int parseActivation(const char* JSON_STRING);

char APP_URL_Buffer[256];

APP_ACTIVATION_DATA           appActivationData;
extern APP_GW_ACTIVATION_DATA appGWActivationData;

extern http_request request;
extern char         default_account_server_url;

void APP_Activation_Initialize(void)
{
    memset(request.url, 0, sizeof(request.url));
    memset(request.urlheaders, 0, sizeof(request.urlheaders));
    memset(request.response_buffer, 0, sizeof(request.response_buffer));
    memset(request.data_buffer, 0, sizeof(request.data_buffer));
    appActivationData.state = APP_ACTIVATION_BARE_CONNECTING;
}

int8_t APP_Activation_State()
{
    return appActivationData.state;
}

void APP_Activation_Reset()
{
    appActivationData.state = APP_ACTIVATION_BARE_CONNECTING;
}

void APP_Activation_Tasks(void)
{

    switch(appActivationData.state)
    {
        case APP_ACTIVATION_BARE_CONNECTING:

            strncpy(APP_URL_Buffer, appGWActivationData.configuration.account_server_url, sizeof(APP_URL_Buffer));
            APP_URL_Buffer[sizeof(APP_URL_Buffer) - 1] = 0; // Make sure that it is a string with null termination
            SYS_DEBUG(SYS_ERROR_DEBUG, "ACTI: APP_URL_Buffer: %s\r\n", APP_URL_Buffer);

            if(APP_URL_Buffer[0] == '\0' ||
               APP_HTTP_ParseUrl(APP_URL_Buffer, &request.host, &request.path, &request.port, &request.tls))
            {
                SYS_DEBUG(SYS_ERROR_FATAL, "ACTI: Could not parse URL '%s', fallback on default\r\n", APP_URL_Buffer);
                strncpy(APP_URL_Buffer, appGWActivationData.configuration.default_account_server_url,
                        sizeof(APP_URL_Buffer));
                APP_URL_Buffer[sizeof(APP_URL_Buffer) - 1] = 0; // Make sure that it is a string with null termination

                if(APP_HTTP_ParseUrl(APP_URL_Buffer, &request.host, &request.path, &request.port, &request.tls))
                {
                    FATAL("ACTI: ERROR - Could not parse default URL '%s'", APP_URL_Buffer);
                    break;
                }
            }

            sprintf(request.urlheaders,
                    "PUT /%s/%s/activate HTTP/1.1\r\n"
                    "User-Agent: TTNGateway\r\n"
                    "Host: %s\r\n"
                    "Connection: close\r\n\r\n",
                    "api/v2/gateways", appGWActivationData.configuration.id, request.host);

            APP_HTTP_Request_Initialize();
            appActivationData.state = APP_ACTIVATION_BARE_REQUESTING;

            break;
        case APP_ACTIVATION_BARE_REQUESTING:
        {
            APP_HTTP_Request_Tasks();
            if(APP_HTTP_Request_State() == APP_HTTP_REQUEST_HTTP_DONE)
            {
                appActivationData.state = APP_ACTIVATION_BARE_PARSING;
            }
            else if(APP_HTTP_Request_State() == APP_HTTP_REQUEST_ERROR)
            {
                appActivationData.state = APP_ACTIVATION_BARE_ERROR;
            }
        }
        break;
        case APP_ACTIVATION_BARE_PARSING:
        {
            char response_code[255];

            char* token = strtok(request.response_buffer, "\r\n");
            SYS_DEBUG(SYS_ERROR_DEBUG, "ACTI: Parsing response token: %s\r\n", token);

            uint16_t it = 0;
            for(it = 0; it < strlen(token); it++)
            {
                response_code[it] = request.response_buffer[it];
            }
            response_code[it] = '\0';
            // memcpy(response_code, token, strlen(token)+1);

            SYS_DEBUG(SYS_ERROR_DEBUG, "ACTI: response code: %d\r\n", response_code);

            token = strtok(NULL, "\r\n");
            while(token)
            {
                if(token[0] == '{')
                {
                    // memcpy(appActivationData.json_buffer, token, strlen(token));
                    strcpy(request.data_buffer, token);
                }
                token = strtok(NULL, "\r\n");
            }

            int r = EXIT_FAILURE;
            if(strstr(response_code, "201") != NULL)
            {
                // Success
                // SYS_DEBUG(SYS_ERROR_DEBUG,"ACTI: Response: ");
                // SYS_CONSOLE_PRINT(request.json_buffer);
                r = parseActivation(request.data_buffer);
                // while (r != EXIT_SUCCESS)
                // {}
                if(r == EXIT_SUCCESS)
                    appActivationData.state = APP_ACTIVATION_BARE_DONE;
                else
                    appActivationData.state = APP_ACTIVATION_BARE_ERROR;
            }
            else
            {
                // Failure
                if(APP_HTTP_TTN_ParseError(request.data_buffer) == EXIT_SUCCESS)
                {
                    appActivationData.state = APP_ACTIVATION_BARE_ERROR;

                    if(strcmp(appGWActivationData.configuration.error.code, "403") == 0)
                    {
                        SYS_DEBUG(SYS_ERROR_WARNING, "ACTI: Already activated\r\n");
                        appActivationData.state = APP_ACTIVATION_BARE_WAS_ACTIVATED;
                    }
                    else if(strcmp(appGWActivationData.configuration.error.code, "404") == 0)
                    {
                        SYS_DEBUG(SYS_ERROR_WARNING, "ACTI: Fetch configuration - non existing ID\r\n");
                        appActivationData.state = APP_ACTIVATION_BARE_NON_EXISTING_ID;
                    }
                    SYS_DEBUG(SYS_ERROR_FATAL, "ACTI: Received an error during activation\r\n");
                    break;
                }
                appActivationData.state = APP_ACTIVATION_BARE_ERROR;
            }
            break;
        }
        break;
        case APP_ACTIVATION_BARE_DONE:
            break;

        case APP_ACTIVATION_BARE_WAS_ACTIVATED:
            break;

        case APP_ACTIVATION_BARE_NON_EXISTING_ID:
            break;

        case APP_ACTIVATION_BARE_ERROR:
            break;

        default:
        {
            break;
        }
    }
}

static int parseActivation(const char* JSON_STRING)
{
    int         i;
    int         r;
    jsmn_parser parser;
    jsmntok_t   tokens[5];

    jsmn_init(&parser);

    r = jsmn_parse(&parser, JSON_STRING, strlen(JSON_STRING), tokens, sizeof(tokens) / sizeof(tokens[0]));
    if(r < 0)
    {
        SYS_DEBUG(SYS_ERROR_ERROR, "ACTI: Failed to parse JSON: %d\r\n", r);
        return 1;
    }

    if(r < 1 || tokens[0].type != JSMN_OBJECT)
    {
        SYS_DEBUG(SYS_ERROR_ERROR, "ACTI: Object expected\r\n");
        return 1;
    }

    for(i = 1; i < r; i++)
    {
        if(jsoneq(JSON_STRING, &tokens[i], "key") == 0)
        {

            //    SYS_CONSOLE_PRINT("- Gateway key: %.*s\r\n", tokens[i+1].end-tokens[i+1].start,
            //                                        JSON_STRING + tokens[i+1].start);
            memset(appGWActivationData.configuration.key, 0, sizeof(appGWActivationData.configuration.key));
            memcpy(appGWActivationData.configuration.key, JSON_STRING + tokens[i + 1].start,
                   tokens[i + 1].end - tokens[i + 1].start);
            i++;
        }
        else
        {
            SYS_DEBUG(SYS_ERROR_WARNING, "ACTI: Unexpected key: %.*s\r\n", tokens[i].end - tokens[i].start,
                      JSON_STRING + tokens[i].start);
        }
    }
    return EXIT_SUCCESS;
}
