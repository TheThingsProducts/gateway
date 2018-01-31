// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#ifndef _APP_OTA_H /* Guard against multiple inclusion */
#define _APP_OTA_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "jsmn.h"

    typedef enum
    {
        APP_OTA_INIT,
        APP_OTA_WAIT_FOR_START,
        APP_OTA_CONNECTING,
        APP_OTA_REQUEST_KEY,
        APP_OTA_REQUEST_FIRMWARE,
        APP_OTA_REQUESTING_DATA,
        APP_OTA_PARSING_RESPONSE,
        APP_OTA_WRITING_TO_STORAGE,
        APP_OTA_FINISH_WRITING,
        APP_OTA_VERIFY_FIRMWARE,
        APP_OTA_DONE,
        APP_OTA_ERROR,
        APP_OTA_STORAGE_ERROR,
        APP_OTA_READY_FOR_REBOOT
    } APP_OTA_STATES;

    typedef struct
    {
        FOTA_FILETYPES file_type;
        APP_OTA_STATES state;
        uint8_t        sha256_recv_bin[64];
    } APP_OTA_DATA;

    void           APP_OTA_Enter(void);
    void           APP_OTA_Leave(void);
    APP_OTA_STATES APP_OTA_State();
    void           APP_OTA_Reset();
    void           APP_OTA_Tasks(void);

#ifdef __cplusplus
}
#endif

#endif /* _APP_OTA_H */
