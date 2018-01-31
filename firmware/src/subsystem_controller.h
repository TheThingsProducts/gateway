// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#ifndef _SUBSYSTEM_CONTROLLER_H /* Guard against multiple inclusion */
#define _SUBSYSTEM_CONTROLLER_H

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "app.h"
#include <stdbool.h>
#include <inttypes.h>

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C"
{
#endif

    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */

#define OFF 0
#define ON 1

#define LED1 0
#define LED2 1
#define LED3 2
#define LED4 3
#define LED5 4

#define LED_POWER LED1
#define LED_INTERNET LED2
#define LED_ACTIVATION LED3
#define LED_MQTT LED4
#define LED_ACTIVITY LED5

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************

    void sdSet(bool state);
    bool sdIsOn();

    void wifiSet(bool state);
    bool wifiIsOn();

    void ethernetSet(bool state);
    bool ethernetIsOn();

    void loraSet(bool state);
    bool loraIsOn();

    void bleSet(bool state);
    bool bleIsOn();

    void gpsSet(bool state);
    bool gpsIsOn();

    void statOn();
    void statOff();

    void statSet(uint8_t stat_num, bool state);
    void statToggle(uint8_t stat_num);
    bool statIsOn(uint8_t stat_num);

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _SUBSYSTEM_CONTROLLER_H */

/* *****************************************************************************
 End of File
 */
