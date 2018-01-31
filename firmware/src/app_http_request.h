// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#ifndef _APP_HTTP_REQUEST_H /* Guard against multiple inclusion */
#define _APP_HTTP_REQUEST_H

/* REVIEW: remove or check magic numbers */
#define RESPONSE_BUFFER_SIZE 6200
#define DATA_BUFFER_SIZE 6200

typedef enum
{
    APP_HTTP_REQUEST_START_CONNECTION,
    APP_HTTP_REQUEST_WAIT_ON_DNS,
    APP_HTTP_REQUEST_WAIT_FOR_CONNECTION,
    APP_HTTP_REQUEST_WAIT_FOR_TLS_CONNECT,
    APP_HTTP_REQUEST_SEND_REQUEST,
    APP_HTTP_REQUEST_WAIT_FOR_RESPONSE,
    APP_HTTP_REQUEST_WAIT_FOR_RESPONSE_BULK,
    APP_HTTP_REQUEST_HTTP_DONE,
    APP_HTTP_REQUEST_ERROR
} APP_HTTP_REQUEST_STATES;

typedef struct
{
    APP_HTTP_REQUEST_STATES state;
    TCP_SOCKET              socket;

} APP_HTTP_REQUEST_DATA;

typedef struct
{
    char     url[255];
    char     urlheaders[2048];
    size_t   header_length;
    uint16_t status_code;
    char     response_buffer[RESPONSE_BUFFER_SIZE];
    size_t   content_length;
    char*    content_pointer;
    char     data_buffer[DATA_BUFFER_SIZE];

    char*    host;
    char*    path;
    uint16_t port;
    bool     tls;

    bool   bulk_request;
    size_t total_bytes;
    size_t available_bytes;
    bool   new_data_flag;

} http_request;

void   APP_HTTP_Request_Initialize();
int8_t APP_HTTP_Request_State();
void   APP_HTTP_Request_CloseIfNeeded(void);
void   APP_HTTP_Request_Reset();
void   APP_HTTP_Request_Tasks(void);

int32_t APP_HTTP_ParseUrl(char* uri, char** host, char** path, uint16_t* port, bool* tls);

#endif
