// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#ifndef _APP_ACTIVATION_H /* Guard against multiple inclusion */
#define _APP_ACTIVATION_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "jsmn.h"

    typedef enum
    {
        APP_ACTIVATION_BARE_CONNECTING,
        APP_ACTIVATION_BARE_REQUESTING,
        APP_ACTIVATION_BARE_PARSING,
        APP_ACTIVATION_BARE_DONE,
        APP_ACTIVATION_BARE_WAS_ACTIVATED,
        APP_ACTIVATION_BARE_NON_EXISTING_ID,
        APP_ACTIVATION_BARE_ERROR,
    } APP_ACTIVATION_STATES;

    typedef struct
    {
        APP_ACTIVATION_STATES state;
    } APP_ACTIVATION_DATA;

    void   APP_Activation_Initialize(void);
    int8_t APP_Activation_State();
    void   APP_Activation_Reset();
    void   APP_Activation_Tasks(void);

#ifdef __cplusplus
}
#endif

#endif