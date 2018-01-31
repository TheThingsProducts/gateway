/*******************************************************************************
  TCP Manager internal stack API

  Company:
    Microchip Technology Inc.
    
  File Name:
    tcp_manager.h

  Summary:
    
  Description:
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright © 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
// DOM-IGNORE-END

#ifndef __TCP_MANAGER_H_
#define __TCP_MANAGER_H_


/****************************************************************************
  Section:
    Type Definitions
  ***************************************************************************/


/****************************************************************************
  Section:
    Function Declarations
  ***************************************************************************/

/*****************************************************************************
  Function:
    bool TCPIP_TCP_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackInit, TCPIP_TCP_MODULE_CONFIG* pTcpInit)

  Summary:
    Initializes the TCP module.

  Description:
    Initializes the TCP module.  This function sets up the TCP buffers
    in memory and initializes each socket to the CLOSED state.  If
    insufficient memory was allocated for the TCP sockets, the function
    will call the TCPIPError() error function that can be captured by the debugger
    and will return false.

  Precondition:
    None

  Parameters:
    stackInit   - pointer to stack initialization data; contains heap, interfaces, etc

    pTcpInit    - pointer to a TCP initialization structure containing:
                    - nSockets:         number of sockets to be created
                    - sktTxBuffSize:    default TX buffer size
                    - sktRxBuffSize:    default RX buffer size
  Returns:
    true if initialization succeeded
    false otherwise

  Remarks:
   None
 */
bool TCPIP_TCP_Initialize(const TCPIP_STACK_MODULE_CTRL* const stackInit, const TCPIP_TCP_MODULE_CONFIG* pTcpInit);


/*****************************************************************************
  Function:
    void TCPIP_TCP_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackInit)

  Summary:
    De-Initializes the TCP module.

  Description:
    De-Initializes the TCP module.
    This function initializes each socket to the CLOSED state.
    If dynamic memory was allocated for the TCP sockets, the function
    will deallocate it.

  Precondition:
    TCPIP_TCP_Initialize() should have been called

  Parameters:
    stackInit   - pointer to stack initialization data; contains heap, interfaces, etc
                  and interface that's going down

  Returns:
    None

  Remarks:
 */
void TCPIP_TCP_Deinitialize(const TCPIP_STACK_MODULE_CTRL* const stackInit);



bool TCPIP_TCP_SourceIPAddressSet(TCP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* localAddress);

bool TCPIP_TCP_DestinationIPAddressSet(TCP_SOCKET s, IP_ADDRESS_TYPE addType, IP_MULTI_ADDRESS* remoteAddress);

int     TCPIP_TCP_SocketsNumberGet(void);


// debug/trace support
//

//#define TCPIP_TCP_DEBUG

typedef struct
{
    uint16_t    addType;        // IPv4/IPv6/unknown socket type
    uint16_t    remotePort;     // port no
    uint16_t    localPort;      // port no
    uint16_t    rxSize;         // RX buffer size
    uint16_t    txSize;         // TX buffer size
    uint16_t    state;          // TCP state
    uint16_t    rxPending;      // pending in RX buffer
    uint16_t    txPending;      // pending in TX buffer

}TCPIP_TCP_SKT_DEBUG_INFO;

// number of TCP sockets
int     TCPIP_TCP_DebugSktNo(void);

// fills in a debug info structure for the specified socket
bool    TCPIP_TCP_DebugSktInfo(int sktNo, TCPIP_TCP_SKT_DEBUG_INFO* pInfo);



#endif  // __TCP_MANAGER_H_
