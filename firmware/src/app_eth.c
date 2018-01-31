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

#include "connector.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

typedef enum
{
    STATE_WAIT_FOR_TCP_INIT,
    /* In this state, the application can do TCP/IP transactions. */
    STATE_TRANSACT,
    STATE_ERROR,
} STATE_t;

static STATE_t _state;

static TCPIP_NET_HANDLE netHandle;

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

/*******************************************************************************
  Function:
    void APP_TCPIP_Initialize(void)

  Remarks:
    None.
 */
void APP_ETH_Initialize(void)
{
    /* Place the App state machine in its initial state. */
    _state = STATE_WAIT_FOR_TCP_INIT;
}

/******************************************************************************
  Function:
    void APP_TCPIP_Tasks ( void )

  Remarks:
    None.
 */
void APP_ETH_Tasks(void)
{
    SYS_STATUS       tcpipStat;
    static IPV4_ADDR dwLastIP = {-1};
    IPV4_ADDR        ipAddr;

    switch(_state)
    {
        case STATE_WAIT_FOR_TCP_INIT:
        {
            SYS_STATUS tcpipStat = TCPIP_STACK_Status(sysObj.tcpip);
            if(tcpipStat < 0)
            {
                SYS_DEBUG(SYS_ERROR_ERROR, "ETH: TCP/IP stack initialisation failed!\r\n");
                _state = STATE_ERROR;
            }
            else if(tcpipStat == SYS_STATUS_READY)
            {
                netHandle = TCPIP_STACK_IndexToNet(ETH_INTERFACE_NUM);
                TCPIP_MDNS_ServiceRegister(netHandle, "Things-Gateway-Eth", "_http._tcp.local", 80,
                                           ((const uint8_t*)"path=/"), 1, NULL, NULL);

                _state = STATE_TRANSACT;
            }
            break;
        }

        case STATE_TRANSACT:

            ipAddr.Val = TCPIP_STACK_NetAddress(netHandle);
            if(dwLastIP.Val != ipAddr.Val)
            {
                dwLastIP.Val = ipAddr.Val;
                SYS_PRINT("ETH: IP Address: %d.%d.%d.%d \r\n", ipAddr.v[0], ipAddr.v[1], ipAddr.v[2], ipAddr.v[3]);
                if(selftest_isEnabled())
                {
                    if(ipAddr.v[0] != 0)
                    {
                        selftest_hasEthernet(true);
                    }
                }
            }
            break;

        default:
            break;
    }
}

bool APP_ETH_Has_Link(void)
{
    if(_state == STATE_TRANSACT && TCPIP_STACK_NetIsLinked(netHandle))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/* *****************************************************************************
 End of File
 */
