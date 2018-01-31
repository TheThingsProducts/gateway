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
#include "app_commands.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

typedef enum
{
    APP_SELFTEST_INIT = 0,

    APP_PRODUCTIONTEST_INIT,
    APP_PRODUCTIONTEST_RUNNING,
    APP_PRODUCTIONTEST_DONE,

    APP_SELFTEST_IDLE,
    APP_SELFTEST_ERROR
} APP_STATES_SELFTEST;

typedef struct
{
    APP_STATES_SELFTEST state;
    bool                selftest_ethernet;
    bool                selftest_wifi;
} SELFTEST_DATA;

static SELFTEST_DATA selftestData;

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

/**
  @Function
    int ExampleInterfaceFunctionName ( int param1, int param2 )

  @Summary
    Brief one-line description of the function.

  @Remarks
    Refer to the example_file.h interface header for function usage details.
 */

void APP_SELFTEST_Initialize(void)
{
    selftestData.selftest_ethernet = false;
    selftestData.selftest_wifi     = false;
}

void APP_SELFTEST_Tasks(void)
{
    switch(selftestData.state)
    {
        case APP_SELFTEST_INIT:
        {
            SYS_STATUS tcpipStat = TCPIP_STACK_Status(sysObj.tcpip);
            if(tcpipStat < 0)
            { // some error occurred
                SYS_CONSOLE_MESSAGE("APP: TCP/IP stack initialization failed!\r\n");
                selftestData.state = APP_SELFTEST_ERROR;
            }
            else if(tcpipStat == SYS_STATUS_READY)
            {
                SYS_PRINT("\r\n== SELF TEST ==\r\n\r\n");
                selftestData.state = APP_PRODUCTIONTEST_INIT;
            }
            break;
        }

        case APP_PRODUCTIONTEST_INIT:
        {
            SYS_PRINT("SELF TEST:STARTING PRODUCTION TEST\r\n");
            selftestData.state = APP_PRODUCTIONTEST_RUNNING;
            break;
        }

        case APP_PRODUCTIONTEST_RUNNING:
        {
            if((_CP0_GET_COUNT() & 0x2000000) != 0)
            {
                statOff();
            }
            else
            {
                statOn();
            }
            if(selftestData.selftest_ethernet && selftestData.selftest_wifi)
            {
                SYS_PRINT("SELFTEST: PRODUCTION TEST SUCCEEDED\r\n");
                selftestData.state = APP_PRODUCTIONTEST_DONE;
                break;
            }
            if(PORTBbits.RB12 == 0)
            {
                SYS_PRINT("SELFTEST: PRODUCTION TEST CANCELED\r\n");

                selftestData.state = APP_SELFTEST_ERROR;
            }
            break;
        }

        case APP_PRODUCTIONTEST_DONE:
        {
            statOn();
            SYS_PRINT("SELFTEST: PRODUCTION TEST DONE\r\n");
            selftestData.state = APP_SELFTEST_IDLE;
            break;
        }

        case APP_SELFTEST_IDLE:
        {
            break;
        }

        case APP_SELFTEST_ERROR:
        {
            if((_CP0_GET_COUNT() & 0x800000) != 0)
            {
                statOff();
            }
            else
            {
                statOn();
            }
            break;
        }
    }
}

void selftest_hasEthernet(bool status)
{
    selftestData.selftest_ethernet = status;
    SYS_PRINT("SELFTEST: PRODUCTION TEST - ETHERNET OK\r\n");
}

void selftest_hasWifi(bool status)
{
    selftestData.selftest_wifi = status;
    SYS_PRINT("SELFTEST: PRODUCTION TEST - WIFI OK\r\n");
}

/* *****************************************************************************
 End of File
 */
