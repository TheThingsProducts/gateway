// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#include "app.h"
#include "app_http_request.h"
#include "app_http_ttn.h"
#include "app_configuration.h"
#include "app_frequencyplan.h"
#include "app_activation.h"
#include "subsystem_controller.h"

extern char* url;
extern char* id;

extern char APP_URL_Buffer[255];

extern APP_CONFIGURATION_DATA appConfigurationData;
extern APP_GW_ACTIVATION_DATA appGWActivationData;
extern http_request           request;

APP_FREQPLAN_DATA appFreqPlanData;

/* Freq parsing structs */
#define TOKEN_LENGTH(i) tokens[i + 1].end - tokens[i + 1].start
#define TOKEN(i) JSON_STRING + tokens[i + 1].start
#define PRINT_TOKEN(i)                                                                                         \
    SYS_DEBUG(SYS_ERROR_DEBUG, "FREQ: - %.*s:\t %.*s\r\n", TOKEN_LENGTH(i - 1), TOKEN(i - 1), TOKEN_LENGTH(i), \
              TOKEN(i));

jsmntok_t tokens[1024];

void APP_FreqPlan_Initialize(void)
{
    memset(request.url, 0, sizeof(request.url));
    memset(request.urlheaders, 0, sizeof(request.urlheaders));
    memset(request.response_buffer, 0, sizeof(request.response_buffer));
    memset(request.data_buffer, 0, sizeof(request.data_buffer));
    appFreqPlanData.state = APP_FREQPLAN_CONNECTING;
}

int8_t APP_FreqPlan_State()
{
    return appFreqPlanData.state;
}

void APP_FreqPlan_Reset()
{
    appFreqPlanData.state = APP_FREQPLAN_CONNECTING;
}

void APP_FreqPlan_Tasks(void)
{

    switch(appFreqPlanData.state)
    {
        case APP_FREQPLAN_CONNECTING:
        {
            if(strcmp(appGWActivationData.configuration.frequency_plan_url, "") == 0)
            {
                SYS_DEBUG(SYS_ERROR_ERROR, "FREQ: No frequency plan url in memory\r\n");
                appFreqPlanData.state = APP_FREQPLAN_ERROR;
                break;
            }
            strcpy(APP_URL_Buffer, appGWActivationData.configuration.frequency_plan_url);

            SYS_DEBUG(SYS_ERROR_INFO, "FREQ: APP_URL_Buffer: %s\r\n", APP_URL_Buffer);

            if(APP_URL_Buffer[0] != '\0')
            {
                SYS_DEBUG(SYS_ERROR_DEBUG, "FREQ: Valid URL\r\n");
                if(APP_HTTP_ParseUrl(APP_URL_Buffer, &request.host, &request.path, &request.port, &request.tls))
                {
                    SYS_DEBUG(SYS_ERROR_FATAL, "FREQ: Could not parse URL '%s'\r\n", APP_URL_Buffer);
                    APP_URL_Buffer[0]     = '\0';
                    appFreqPlanData.state = APP_FREQPLAN_ERROR;
                    break;
                }

                sprintf(request.urlheaders,
                        "GET /%s?filter=ttn HTTP/1.1\r\n"
                        "User-Agent: TTNGateway\r\n"
                        "Host: %s:%u\r\n"
                        "Connection: close\r\n\r\n",
                        request.path, request.host, request.port);

                APP_HTTP_Request_Initialize();
                appFreqPlanData.state = APP_FREQPLAN_REQUESTING;
            }
            else
            {
                SYS_DEBUG(SYS_ERROR_FATAL, "FREQ: INVALID REQUEST URL\r\n");
                appFreqPlanData.state = APP_FREQPLAN_ERROR;
            }
            break;
            case APP_FREQPLAN_REQUESTING:
            {
                APP_HTTP_Request_Tasks();
                if(APP_HTTP_Request_State() == APP_HTTP_REQUEST_HTTP_DONE)
                {
                    appFreqPlanData.state = APP_FREQPLAN_PARSING;
                }
                else if(APP_HTTP_Request_State() == APP_HTTP_REQUEST_ERROR)
                {
                    appFreqPlanData.state = APP_FREQPLAN_ERROR;
                }
            }
            break;
            case APP_FREQPLAN_PARSING:
            {
                char response_code[255];

                char* token = strtok(request.response_buffer, "\r\n");
                SYS_DEBUG(SYS_ERROR_DEBUG, "FREQ: Parsing response token: %s\r\n", token);

                uint16_t it = 0;
                for(it = 0; it < strlen(token); it++)
                {
                    response_code[it] = request.response_buffer[it];
                    if(it >= 253)
                        break;
                }
                response_code[it] = '\0';
                SYS_DEBUG(SYS_ERROR_DEBUG, "FREQ: response code: %d\r\n", response_code);
                // memcpy(response_code, token, strlen(token)+1);

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

                //    SYS_CONSOLE_MESSAGE("Response:\r\n");
                int r = EXIT_FAILURE;

                if(strstr(response_code, "200") != NULL)
                {
                    // Success
                    // SYS_CONSOLE_PRINT("\r\nResponse: to much");
                    // SYS_CONSOLE_PRINT(request.json_buffer);
                    r = parseFrequency(request.data_buffer);

                    // while (r != EXIT_SUCCESS)
                    // {}
                    if(r == EXIT_SUCCESS)
                        appFreqPlanData.state = APP_FREQPLAN_DONE;
                    else
                        appFreqPlanData.state = APP_FREQPLAN_ERROR;
                }
                else
                {
                    // Failure
                    if(APP_HTTP_TTN_ParseError(request.data_buffer) == EXIT_SUCCESS)
                    {
                        appFreqPlanData.state = APP_FREQPLAN_ERROR;

                        if(strcmp(appGWActivationData.configuration.error.code, "403") == 0)
                        {
                            // SYS_CONSOLE_MESSAGE("\r\nFetch config - Key Failure\r\n");
                            appFreqPlanData.state = APP_FREQPLAN_FAILURE;
                        }
                        SYS_DEBUG(SYS_ERROR_FATAL, "FREQ: Received an error during frequency fetch\r\n");
                        break;
                    }
                    appFreqPlanData.state = APP_FREQPLAN_ERROR;
                }
                // appActivationData.state = APP_IDLE;
                // parsing_is_busy = false;
                break;
            }
        }
        break;
        case APP_FREQPLAN_DONE:
            //  SYS_CONSOLE_MESSAGE(request.json_buffer);
            //  free(request);
            break;

        case APP_FREQPLAN_FAILURE:
            //  SYS_CONSOLE_MESSAGE(request.json_buffer);
            //  free(request);
            break;
        case APP_FREQPLAN_ERROR:
            //  SYS_CONSOLE_MESSAGE(request.json_buffer);
            //  free(request);
            break;
        default:
        {
            break;
        }
    }
}

int parseFrequency(const char* JSON_STRING)
{
    int i;
    int r;
    int rfChainNumber = -1;
    int ifChainNumber = -1;

    jsmn_parser parser;
    jsmn_init(&parser);

    r = jsmn_parse(&parser, JSON_STRING, strlen(JSON_STRING), tokens, sizeof(tokens) / sizeof(tokens[0]));

    if(r < 0)
    {
        SYS_DEBUG(SYS_ERROR_ERROR, "FREQ: Failed to parse JSON: %d\r\n", r);
        return 1;
    }

    /* Assume the top-level element is an object */
    if(r < 1 || tokens[0].type != JSMN_OBJECT)
    {
        SYS_DEBUG(SYS_ERROR_ERROR, "FREQ: Object expected\r\n");
        return 1;
    }

    /* Loop over all keys of the root object */
    for(i = 1; i < r; i++)
    {
        if(jsoneq(JSON_STRING, &tokens[i], "lorawan_public") == 0)
        {
            appGWActivationData.configuration_sx1301.lorawan_public = jsonstring2bool(TOKEN(i));
            PRINT_TOKEN(i);
            i++;
        }
        else if(jsoneqPart(JSON_STRING, &tokens[i], "radio_") != -1)
        {
            // Decode which IF Chain
            rfChainNumber = atoi(JSON_STRING + tokens[i].end - 1);
            // SYS_CONSOLE_PRINT("- radio_%d: %.*s\r\n", rfChainNumber, TOKEN_LENGTH(i), TOKEN(i));
            i++;
        }
        else if(jsoneq(JSON_STRING, &tokens[i], "freq") == 0)
        {

            appGWActivationData.configuration_sx1301.rfchain[rfChainNumber].freq = atoi(TOKEN(i));
            PRINT_TOKEN(i);
            i++;
        }
        else if(jsoneqPart(JSON_STRING, &tokens[i], "chan_multiSF_") != -1)
        {
            // Decode which IF Chain
            ifChainNumber = atoi(JSON_STRING + tokens[i].end - 1);
            // SYS_CONSOLE_PRINT("- chan_multiSF_%d: %.*s\r\n", ifChainNumber, TOKEN_LENGTH(i), TOKEN(i));
            i++;
        }
        else if(jsoneq(JSON_STRING, &tokens[i], "chan_Lora_std") == 0)
        {
            ifChainNumber = 8;
            PRINT_TOKEN(i);
            i++;
        }
        else if(jsoneq(JSON_STRING, &tokens[i], "chan_FSK") == 0)
        {
            ifChainNumber = 9;
            PRINT_TOKEN(i);
            i++;
        }
        else if(jsoneq(JSON_STRING, &tokens[i], "enable") == 0)
        {
            if(ifChainNumber != -1)
            {
                appGWActivationData.configuration_sx1301.ifchain[ifChainNumber].enable = jsonstring2bool(TOKEN(i));
                PRINT_TOKEN(i);
            }
            else if(rfChainNumber != -1)
            {
                appGWActivationData.configuration_sx1301.rfchain[rfChainNumber].enable = jsonstring2bool(TOKEN(i));
                PRINT_TOKEN(i);
            }

            i++;
        }
        else if(jsoneq(JSON_STRING, &tokens[i], "radio") == 0)
        {
            appGWActivationData.configuration_sx1301.ifchain[ifChainNumber].radio = atoi(TOKEN(i));
            PRINT_TOKEN(i);
            i++;
        }
        else if(jsoneq(JSON_STRING, &tokens[i], "if") == 0)
        {
            appGWActivationData.configuration_sx1301.ifchain[ifChainNumber].freqOffset = atoi(TOKEN(i));
            PRINT_TOKEN(i);
            i++;
        }
        else if(jsoneq(JSON_STRING, &tokens[i], "bandwidth") == 0)
        {
            appGWActivationData.configuration_sx1301.ifchain[ifChainNumber].bandwidth = atoi(TOKEN(i));
            PRINT_TOKEN(i);
            i++;
        }
        else if(jsoneq(JSON_STRING, &tokens[i], "spread_factor") == 0)
        {
            appGWActivationData.configuration_sx1301.ifchain[ifChainNumber].spread_factor = atoi(TOKEN(i));
            PRINT_TOKEN(i);
            i++;
        }
        else if(jsoneq(JSON_STRING, &tokens[i], "datarate") == 0)
        {
            appGWActivationData.configuration_sx1301.ifchain[ifChainNumber].datarate = atoi(TOKEN(i));
            PRINT_TOKEN(i);
            i++;
        }
    }

    return EXIT_SUCCESS;
}
