// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#include "app.h"
#include "app_http_ttn.h"
#include "jsmn.h"
#include "subsystem_controller.h"

extern APP_GW_ACTIVATION_DATA appGWActivationData;

int APP_HTTP_TTN_ParseError(const char* JSON_STRING)
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
        if(jsoneq(JSON_STRING, &tokens[i], "code") == 0)
        {

            SYS_DEBUG(SYS_ERROR_INFO, "ACTI: parsing - Code: %.*s\r\n", tokens[i + 1].end - tokens[i + 1].start,
                      JSON_STRING + tokens[i + 1].start);
            memset(appGWActivationData.configuration.error.code, 0,
                   sizeof(appGWActivationData.configuration.error.code));
            memcpy(appGWActivationData.configuration.error.code, JSON_STRING + tokens[i + 1].start,
                   tokens[i + 1].end - tokens[i + 1].start);
            i++;
        }
        else if(jsoneq(JSON_STRING, &tokens[i], "error") == 0)
        {

            SYS_DEBUG(SYS_ERROR_INFO, "ACTI: parsing - Error: %.*s\r\n", tokens[i + 1].end - tokens[i + 1].start,
                      JSON_STRING + tokens[i + 1].start);
            memset(appGWActivationData.configuration.error.error, 0,
                   sizeof(appGWActivationData.configuration.error.error));
            memcpy(appGWActivationData.configuration.error.error, JSON_STRING + tokens[i + 1].start,
                   tokens[i + 1].end - tokens[i + 1].start);
            i++;
        }
        else
        {
            SYS_DEBUG(SYS_ERROR_INFO, "ACTI: parsing - Unexpected key: %.*s\r\n", tokens[i].end - tokens[i].start,
                      JSON_STRING + tokens[i].start);
        }
    }

    return EXIT_SUCCESS;
}
