/*******************************************************************************
  BSD Internal Stack API Header File

  Company:
    Microchip Technology Inc.
    
  File Name:
    berkeley_manager.h

  Summary:
    BSD Internal Stack API Header File

  Description:
    This file provides the BSD Internal Stack API definitions.
    
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

#ifndef __BERKELEY_MANAGER_H_
#define __BERKELEY_MANAGER_H_


typedef enum
{
    SKT_CLOSED,             // Socket closed state indicating a free descriptor
    SKT_CREATED,            // Socket created state for TCP and UDP sockets
    SKT_BOUND,              // Socket bound state for TCP and UDP sockets
    SKT_BSD_LISTEN,         // Listening state for TCP BSD listener handle "socket"
    SKT_LISTEN,             // TCP server listen state
    SKT_IN_PROGRESS,        // TCP client connection in progress state
    SKT_EST,                // TCP client or server established state
    SKT_DISCONNECTED        // TCP client or server no longer connected to the remote host (but was historically)
} BSD_SCK_STATE; // Berkeley Socket (BSD) states

struct BSDSocket
{
    int            SocketType; // Socket type
    BSD_SCK_STATE  bsdState; //Socket state
    uint16_t           localPort; //local port
    uint16_t           remotePort; //remote port
    uint32_t          remoteIP; //remote IP
#if defined(TCPIP_STACK_USE_IPV6)
    uint32_t        remoteIPv6[3]; // remote IP for IPv6
    int            addressFamily;
#endif
    int            backlog; // maximum number or client connection
    bool           isServer; // server/client check
    NET_PRES_SKT_HANDLE_T     SocketID; // Socket ID
    uint32_t          localIP; // bound address
#if defined(TCPIP_STACK_USE_IPV6)
    uint32_t        localIPv6[3];
#endif
    uint32_t        rcvBufSize;
    uint32_t        sndBufSize;
    uint16_t        rcvTmOut;
    uint16_t        sndTmOut;
    uint16_t        lingerTmo;
    union {
        struct {
            uint16_t tcpLinger        :1;
            uint16_t tcpKeepAlive     :1;
            uint16_t tcpNoDelay       :1;
            uint16_t tcpExclusiveAccess :1;
            uint16_t tcpTresFlush     :2;
            uint16_t tcpGracefulDisable :1;
            uint16_t udpBcastEnabled    : 1;
            uint16_t reserved           : 8;
        };
        struct {
            uint16_t w :16;
        };
    };
}; // Berkeley Socket structure



/*****************************************************************************
  Function:
    void BerkeleySocketInit(const TCPIP_STACK_MODULE_CTRL* const stackData,
                        const BERKELEY_MODULE_CONFIG* berkeleyData)

  Summary:
    Initializes the Berkeley socket structure array.

  Description:
    This function initializes the Berkeley socket array. This function should
    be called before any BSD socket call.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None

  Remarks:
    None.
 */
bool BerkeleySocketInit(const TCPIP_STACK_MODULE_CTRL* const stackData,
                        const BERKELEY_MODULE_CONFIG* berkeleyData);


/*****************************************************************************
  Function:
    void BerkeleySocketDeinit(void)

  Summary:
    De-Initializes the Berkeley socket structure array.

  Description:
    This function deinitializes the Berkeley socket array. This function should
    be called when closing out any BSD socket call.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None

  Remarks:
    None.
 */
void BerkeleySocketDeInit(const TCPIP_STACK_MODULE_CTRL* const stackData);

#endif  // __BERKELEY_MANAGER_H_


