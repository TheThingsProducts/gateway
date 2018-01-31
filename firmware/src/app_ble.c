// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "app.h"
#include "subsystem_controller.h"

#define BUF_SIZE 100

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

static APP_DATA_BLE appData;

static bool LEDstate = OFF;

char buffer[BUF_SIZE];

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

/*******************************************************************************
  Function:
    void APP_BLE_Initialize(void)

  Remarks:
    None.
 */
void APP_BLE_Initialize(void)
{
    if(!bleIsOn())
        return;
    // Open USART Driver instance 1 (USART1 in this case) and obtain a handle to it
    appData.USARTHandle = DRV_USART_Open(DRV_USART_INDEX_2, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING);

    /* Place the App state machine in its initial state. */
    appData.state = APP_BLE_INIT;
}

/******************************************************************************
  Function:
    void APP_BLE_Tasks ( void )

  Remarks:
    None.
 */
void APP_BLE_Tasks(void)
{
    if(!bleIsOn())
        return;
    switch(appData.state)
    {
        case APP_BLE_INIT:
        {
            appData.state = APP_BLE_IDLE;
            break;
        }

        case APP_BLE_SEND:
        {
            break;
        }

        case APP_BLE_RECEIVE:
        {
            break;
        }

        case APP_BLE_IDLE:
        {
            break;
        }
    }
}

/* *****************************************************************************
 End of File
 */
