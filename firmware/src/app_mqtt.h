// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#ifndef _APP_MQTT_H /* Guard against multiple inclusion */
#define _APP_MQTT_H

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C"
{
#endif

#include "app.h"
#include "connector.h"

    uint8_t APP_MQTT_GET_STATE();
    void    APP_MQTT_Reset();
    void    APP_MQTT_Initialize(void);
    void    APP_MQTT_Tasks(void);
    void    handleDownlink(Router__DownlinkMessage* message, void* arg);
    void    getPacketCount(uint32_t* pup, uint32_t* pdown);
    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
