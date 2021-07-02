// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#ifndef _APP_CONFIGURATION_H /* Guard against multiple inclusion */
#define _APP_CONFIGURATION_H

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        APP_CONFIGURATION_OPEN_FILE,
        APP_CONFIGURATION_READ_FILE,
        APP_CONFIGURATION_CONNECTING,
        APP_CONFIGURATION_REQUESTING,
        APP_CONFIGURATION_PARSING,
        APP_CONFIGURATION_VALIDATING,
        APP_CONFIGURATION_DONE,
        APP_CONFIGURATION_NON_EXISTING_ID,
        APP_CONFIGURATION_KEY_FAILURE,
        APP_CONFIGURATION_INCOMPLETE,
        APP_CONFIGURATION_SERVER_BUSY,
        APP_CONFIGURATION_ERROR,
    } APP_CONFIGURATION_STATES; // REVIEW: hide internal states, let is access by test functions

    typedef struct
    {
        APP_CONFIGURATION_STATES state;
    } APP_CONFIGURATION_DATA;

    void   APP_Configuration_Initialize(void);
    int8_t APP_Configuration_State();
    void   APP_Configuration_Reset();
    void   APP_Configuration_Tasks(void);

#ifdef __cplusplus
}
#endif

#endif
