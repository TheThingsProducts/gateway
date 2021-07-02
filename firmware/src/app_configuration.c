// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#include "app.h"
#include "app_http_request.h"
#include "app_http_ttn.h"
#include "app_configuration.h"
#include "app_activation.h"
#include "subsystem_controller.h"

extern char* url;
extern char* id;

const char  EMPTY_STRING[] = "";
static int  _parseSettings(const char* JSON_STRING);
static void _parseMQTTUrl(char* uri, const char** host, const char** path, uint16_t* port, bool* tls);
static bool _storeString(const char* JSON_STRING, jsmntok_t* token, char* dest, size_t dest_size);
extern char APP_URL_Buffer[256];

APP_CONFIGURATION_DATA        appConfigurationData;
extern APP_GW_ACTIVATION_DATA appGWActivationData;

extern http_request request;

SYS_FS_HANDLE fileHandle;

void APP_Configuration_Initialize(void)
{
    memset(request.url, 0, sizeof(request.url));
    memset(request.urlheaders, 0, sizeof(request.urlheaders));
    memset(request.response_buffer, 0, sizeof(request.response_buffer));
    memset(request.data_buffer, 0, sizeof(request.data_buffer));
    appConfigurationData.state = APP_CONFIGURATION_OPEN_FILE;
}

int8_t APP_Configuration_State()
{
    return appConfigurationData.state;
}

void APP_Configuration_Reset()
{
    appConfigurationData.state = APP_CONFIGURATION_OPEN_FILE;
}

void APP_Configuration_Tasks(void)
{
    switch(appConfigurationData.state)
    {
	case APP_CONFIGURATION_OPEN_FILE:
	{
	    fileHandle = SYS_FS_FileOpen("/mnt/myDrive1/config.json",(SYS_FS_FILE_OPEN_READ));
	    if(fileHandle != SYS_FS_HANDLE_INVALID){ // File open is successful
                appConfigurationData.state = APP_CONFIGURATION_READ_FILE;
	    } else {
                SYS_DEBUG(SYS_ERROR_INFO, "CONF: Could not open config.json from SD Card\r\n");
                appConfigurationData.state = APP_CONFIGURATION_CONNECTING;
	    }
	}
	break;
	case APP_CONFIGURATION_READ_FILE:
	{
	    int buffLen = 4096;
	    char buffer[buffLen];
            int readBytes = SYS_FS_FileRead(fileHandle, (void *)buffer, buffLen);
            if(readBytes == -1){
                SYS_DEBUG(SYS_ERROR_INFO, "CONF: Could not read config.json from SD Card\r\n");
                appConfigurationData.state = APP_CONFIGURATION_CONNECTING;
            } else {
                buffer[readBytes] = '\0';
                SYS_DEBUG(SYS_ERROR_INFO, "CONF: config.json read, len %d\r\n", readBytes);
                SYS_FS_FileClose(fileHandle);

                SYS_DEBUG(SYS_ERROR_DEBUG, "CONF: Stack size 1: %d\r\n", uxTaskGetStackHighWaterMark(NULL));
                int r = _parseSettings(buffer);
                SYS_DEBUG(SYS_ERROR_DEBUG, "CONF: Stack size 2: %d\r\n", uxTaskGetStackHighWaterMark(NULL));
                if(r == EXIT_SUCCESS)
                    appConfigurationData.state = APP_CONFIGURATION_DONE;
                else
                    appConfigurationData.state = APP_CONFIGURATION_CONNECTING;
            }
	}
	break;
        case APP_CONFIGURATION_CONNECTING:
        {
            strncpy(APP_URL_Buffer, appGWActivationData.configuration.account_server_url, sizeof(APP_URL_Buffer));
            APP_URL_Buffer[sizeof(APP_URL_Buffer) - 1] = 0; // Make sure that it is a string with null termination
            SYS_DEBUG(SYS_ERROR_DEBUG, "CONF: APP_URL_Buffer: %s\r\n", APP_URL_Buffer);

            if(APP_URL_Buffer[0] == '\0' ||
               APP_HTTP_ParseUrl(APP_URL_Buffer, &request.host, &request.path, &request.port, &request.tls))
            {
                SYS_DEBUG(SYS_ERROR_FATAL, "CONF: Could not parse URL '%s', fallback on default \r\n", APP_URL_Buffer);
                strncpy(APP_URL_Buffer, appGWActivationData.configuration.default_account_server_url,
                        sizeof(APP_URL_Buffer));
                APP_URL_Buffer[sizeof(APP_URL_Buffer) - 1] = 0; // Make sure that it is a string with null termination

                if(APP_HTTP_ParseUrl(APP_URL_Buffer, &request.host, &request.path, &request.port, &request.tls))
                {
                    FATAL("CONF: ERROR - Could not parse default URL '%s'", APP_URL_Buffer);
                    break;
                }
            }

            sprintf(request.urlheaders,
                    "GET /%s/%s?filter=ttn HTTP/1.1\r\n"
                    "User-Agent: TTNGateway\r\n"
                    "Host: %s:%u\r\n"
                    "Authorization: Key %s\r\n"
                    "Connection: close\r\n\r\n",
                    "api/v2/gateways", appGWActivationData.configuration.id, request.host, request.port,
                    appGWActivationData.configuration.key);

            SYS_DEBUG(SYS_ERROR_DEBUG, "CONF: URL HEADER: %s\r\n", request.urlheaders);

            APP_HTTP_Request_Initialize();
            appConfigurationData.state = APP_CONFIGURATION_REQUESTING;
        }
        break;
        case APP_CONFIGURATION_REQUESTING:
        {
            APP_HTTP_Request_Tasks();
            if(APP_HTTP_Request_State() == APP_HTTP_REQUEST_HTTP_DONE)
            {
                appConfigurationData.state = APP_CONFIGURATION_PARSING;
            }
            else if(APP_HTTP_Request_State() == APP_HTTP_REQUEST_ERROR)
            {
                SYS_DEBUG(SYS_ERROR_FATAL, "CONF: ERROR REQUEST\r\n");
                appConfigurationData.state = APP_CONFIGURATION_ERROR;
            }
        }
        break;
        case APP_CONFIGURATION_PARSING:
        {
            char  response_code[255];
            char* token = strtok(request.response_buffer, "\r\n");
            SYS_DEBUG(SYS_ERROR_FATAL, "CONF: Parsing response token: %s\r\n", token);

            uint16_t it = 0;
            for(it = 0; it < strlen(token); it++)
            {
                response_code[it] = request.response_buffer[it];
            }
            response_code[it] = '\0';
            token             = strtok(NULL, "\r\n");
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
            if(strstr(response_code, "200") != NULL)
            {
                // Success
                SYS_DEBUG(SYS_ERROR_DEBUG, "CONF: Response: too much to print\r\n");
                SYS_DEBUG(SYS_ERROR_DEBUG, "CONF: Stack size 1: %d\r\n", uxTaskGetStackHighWaterMark(NULL));
                r = _parseSettings(request.data_buffer);
                SYS_DEBUG(SYS_ERROR_DEBUG, "CONF: Stack size 2: %d\r\n", uxTaskGetStackHighWaterMark(NULL));
                if(r == EXIT_SUCCESS)
                    appConfigurationData.state = APP_CONFIGURATION_VALIDATING;
                else
                    appConfigurationData.state = APP_CONFIGURATION_INCOMPLETE;
            }
            else
            {
                // Failure
                if(APP_HTTP_TTN_ParseError(request.data_buffer) == EXIT_SUCCESS)
                {
                    appConfigurationData.state = APP_CONFIGURATION_ERROR;

                    if(strcmp(appGWActivationData.configuration.error.code, "401") == 0)
                    {
                        SYS_DEBUG(SYS_ERROR_WARNING, "CONF: Fetch configuration - key failure\r\n");
                        appConfigurationData.state = APP_CONFIGURATION_KEY_FAILURE;
                        break;
                    }
                    else if(strcmp(appGWActivationData.configuration.error.code, "50") ==
                            0) // REVIEW: Which code should this be?
                    {
                        SYS_DEBUG(SYS_ERROR_WARNING, "CONF: Fetch configuration - server busy\r\n");
                        appConfigurationData.state = APP_CONFIGURATION_SERVER_BUSY;
                        break;
                    }
                    else if(strcmp(appGWActivationData.configuration.error.code, "404") == 0)
                    {
                        SYS_DEBUG(SYS_ERROR_WARNING, "CONF: Fetch configuration - non existing ID\r\n");
                        appConfigurationData.state = APP_CONFIGURATION_NON_EXISTING_ID;
                        break;
                    }
                    SYS_DEBUG(SYS_ERROR_ERROR, "CONF: Received an error during settings fetch\r\n");
                    break;
                }
                else
                {
                    appConfigurationData.state = APP_CONFIGURATION_ERROR;
                    break;
                }
            }
        }
        break;
        case APP_CONFIGURATION_VALIDATING:
        {
            http_request http_temp = {0};
            size_t       len_temp  = sizeof(APP_URL_Buffer);

            bool status = true;
            // check already parsed MQTT url
            status *= (strlen(appGWActivationData.configuration_gateway.ttn_servers[0].server_address) != 0);
            status *= (appGWActivationData.configuration_gateway.ttn_servers[0].serv_port_up != 0);
            status *= (appGWActivationData.configuration_gateway.ttn_servers[0].serv_port_down != 0);

            // check frequency plan url by dummy parsing it
            status *= (strncpy(APP_URL_Buffer, appGWActivationData.configuration.frequency_plan_url, len_temp) != NULL);
            APP_URL_Buffer[len_temp - 1] = 0; // Make sure that it is a string with null termination
            status *= (APP_HTTP_ParseUrl(APP_URL_Buffer, &http_temp.host, &http_temp.path, &http_temp.port,
                                         &http_temp.tls) == 0);

            // check firmware url by dummy parsing it
            status *= (strncpy(APP_URL_Buffer, appGWActivationData.configuration.firmware_url, len_temp) != NULL);
            APP_URL_Buffer[len_temp - 1] = 0; // Make sure that it is a string with null termination
            status *= (APP_HTTP_ParseUrl(APP_URL_Buffer, &http_temp.host, &http_temp.path, &http_temp.port,
                                         &http_temp.tls) == 0);

            if(status)
                appConfigurationData.state = APP_CONFIGURATION_DONE;
            else
                appConfigurationData.state = APP_CONFIGURATION_INCOMPLETE;
        }
        break;
        case APP_CONFIGURATION_DONE:
            break;
        case APP_CONFIGURATION_KEY_FAILURE:
        case APP_CONFIGURATION_SERVER_BUSY:
        case APP_CONFIGURATION_INCOMPLETE:
        case APP_CONFIGURATION_ERROR:
	    APP_Configuration_Initialize();
            break;
        default:
        {
            break;
        }
    }
}

static bool _storeString(const char* JSON_STRING, jsmntok_t* token, char* dest, size_t dest_size)
{
    int len = token->end - token->start;
    if((len + 1) > dest_size)
    { // if string plus terminating zero is bigger than buffer
        return false;
    }
    else
    {
        memcpy(dest, JSON_STRING + token->start, len);
        dest[len] = 0;
        return true;
    }
}

static int _parseSettings(const char* JSON_STRING)
{
    uint16_t    i;
    int16_t     r;
    uint16_t    max_tokens = 255;
    jsmn_parser parser;
    jsmntok_t   tokens[max_tokens];

    jsmn_init(&parser);

    r = jsmn_parse(&parser, JSON_STRING, strlen(JSON_STRING), tokens, max_tokens);
    if(r < 0)
    {
        SYS_DEBUG(SYS_ERROR_ERROR, "CONF: Failed to parse JSON: %d\r\n", r);
        return EXIT_FAILURE;
    }
    /* Assume the top-level element is an object */
    if(r < 1 || tokens[0].type != JSMN_OBJECT)
    {
        SYS_DEBUG(SYS_ERROR_ERROR, "CONF: Object expected\r\n");
        return EXIT_FAILURE;
    }

    /* Loop over all keys of the root object */
    for(i = 1; i < r; i++)
    {
        if(jsoneq(JSON_STRING, &tokens[i], "frequency_plan") == 0)
        {
            if(!_storeString(JSON_STRING, &tokens[i + 1], appGWActivationData.configuration.frequency_plan,
                             sizeof(appGWActivationData.configuration.frequency_plan)))
            {
                SYS_PRINT("CONF: Failed to store received frequency_plan\r\n");
            }
            i++;
        }
        else if(jsoneq(JSON_STRING, &tokens[i], "frequency_plan_url") == 0)
        {
            if(!_storeString(JSON_STRING, &tokens[i + 1], appGWActivationData.configuration.frequency_plan_url,
                             sizeof(appGWActivationData.configuration.frequency_plan_url)))
            {
                SYS_PRINT("CONF: Failed to store received frequency_plan_url\r\n");
            }
            i++;
        }
        else if(jsoneq(JSON_STRING, &tokens[i], "firmware_url") == 0)
        {
            if(!_storeString(JSON_STRING, &tokens[i + 1], appGWActivationData.configuration.firmware_url,
                             sizeof(appGWActivationData.configuration.firmware_url)))
            {
                SYS_PRINT("CONF: Failed to store received firmware_url\r\n");
            }
            i++;
        }
        else if(jsoneq(JSON_STRING, &tokens[i], "router") == 0)
        {
            const char* nhost;
            const char* npath;
            uint16_t    nport     = 0;
            bool        tls       = false;
            uint16_t    buff_size = sizeof(appGWActivationData.configuration_gateway.ttn_servers[0].server_address);

            while(jsoneq(JSON_STRING, &tokens[i], "mqtt_address") != 0)
            {
                i++;
                if(i > r)
                {
                    SYS_DEBUG(SYS_ERROR_FATAL, "CONF: ERROR: no MQTT router configured\r\n");
                    return EXIT_FAILURE;
                }
            }

            char urlbuff[buff_size];
            if(!_storeString(JSON_STRING, &tokens[i + 1], urlbuff, sizeof(urlbuff)))
            {
                SYS_PRINT("CONF: Failed to handle received mqtt_address\r\n");
            }
            else
            {
                uint16_t nlen = 0;
                SYS_DEBUG(SYS_ERROR_WARNING, "CONF: ROUTER URL: %s\r\n", urlbuff);

                _parseMQTTUrl(urlbuff, &nhost, &npath, &nport, &tls);
                nlen = strlen(nhost);
                nlen += strlen(npath);
                if(nlen < 1024)
                {
                    memset(appGWActivationData.configuration_gateway.ttn_servers[0].server_address, 0, buff_size);
                    sprintf(appGWActivationData.configuration_gateway.ttn_servers[0].server_address, "%s%s", nhost,
                            npath);

                    appGWActivationData.configuration_gateway.ttn_servers[0].serv_port_up   = nport;
                    appGWActivationData.configuration_gateway.ttn_servers[0].serv_port_down = nport;
                    appGWActivationData.configuration_gateway.ttn_servers[0].serv_tls       = tls;
                }
            }
            i++;
        }
        else if(jsoneq(JSON_STRING, &tokens[i], "auto_update") == 0)
        {
            appGWActivationData.configuration.auto_update = jsonstring2bool(JSON_STRING + tokens[i + 1].start);
            i++;
        }
    }
    return EXIT_SUCCESS;
}

/**
 * Parses an given URL.
 * @param uri   Pointer to the URL to parse
 * @param host  Pointer to be filled with host string
 * @param path  Pointer to be filled with path string
 * @param port  Pointer to be filled with port number
 * @param tls   Pointer to be filled with TLS flag
 */
static void _parseMQTTUrl(char* uri, const char** host, const char** path, uint16_t* port, bool* tls)
{
    char* pos = uri;
    bool  protocolImplicit;

    if(strncmp(pos, "mqtts://", 8) == 0)
    {
        protocolImplicit = false;
        *host            = pos + 8;
        *tls             = true;
        *port            = 8883;
    }
    else if(strncmp(pos, "mqtt://", 7) == 0)
    {
        protocolImplicit = false;
        *host            = pos + 7;
        *tls             = false;
        *port            = 1883;
    }
    else
    {
        // No (known) protocol prefix, assume defaults
        protocolImplicit = true;
        *host            = pos;
        *tls             = false; // TLS might be overruled by the port if it is 8883
        *port            = 1883;
    }

    pos = strchr(*host, ':');

    if(!pos)
    {
        pos = strchr(*host, '/');
        if(!pos)
        {
            *path = EMPTY_STRING;
        }
        else
        {
            *pos  = '\0';
            *path = pos + 1;
        }
    }
    else
    {
        *pos        = '\0';
        char* portc = pos + 1;

        pos = strchr(portc, '/');
        if(!pos)
        {
            *path = EMPTY_STRING;
        }
        else
        {
            *pos  = '\0';
            *path = pos + 1;
        }
        *port = atoi(portc);
        if(protocolImplicit && *port == 8883)
        {
            *tls = true;
        }
    }
}
