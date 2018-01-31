/*******************************************************************************
  ICMP Module Definitions for the Microchip TCP/IP Stack

  Company:
    Microchip Technology Inc.
    
  File Name:
    icmp.h

  Summary:
    The Internet Control Message Protocol is used to send error and status messages
    and requests.

  Description:
    The Internet Control Message Protocol is used to send error and status messages 
    and requests. The ICMP module implements the Echo Reply message type (commonly 
    referred to as a ping) which can be used to determine if a specified host is 
    reachable across an IP network from a device running the TCP/IP stack. An ICMP 
    server is also supported to respond to pings from other devices.
*******************************************************************************/
//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright 2012-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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
//DOM-IGNORE-END

#ifndef __ICMP_H
#define __ICMP_H

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  

// result of an ICMPSendEchoRequest call
typedef enum
{
    ICMP_ECHO_OK                = 0,    // operation successful
    // error codes, < 0
    ICMP_ECHO_ALLOC_ERROR       = -1,   // could not allocate memory
    ICMP_ECHO_ROUTE_ERROR       = -2,   // could not find a route to destination
    ICMP_ECHO_TRANSMIT_ERROR    = -3,   // could not transmit (dead interface, etc.)
}ICMP_ECHO_RESULT;

// a handle that a client can use after the event handler has been registered
typedef const void* ICMP_HANDLE;

// *****************************************************************************
/* ICMP Module Configuration Structure Typedef

  Summary:
    Placeholder for ICMP module configuration.

  Description:
    Provides a placeholder for ICMP module configuration.

  Remarks:
    None.
*/
typedef struct
{
} TCPIP_ICMP_MODULE_CONFIG;

// *****************************************************************************
/* Function:
    ICMP_ECHO_RESULT TCPIP_ICMP_EchoRequestSend (TCPIP_NET_HANDLE netH, IPV4_ADDR * targetAddr, 
	                                             uint16_t sequenceNumber, uint16_t identifier)

  Summary:
    Sends an ICMP echo request to a remote node.

  Description:
    This function allows a stack client to send an ICMP query message to a remote host.
    The supplied sequence number and identifier will be used in the query message.
    The user will be notified by the result of the query using a notification handle
    registered by using the TCPIP_ICMP_CallbackRegister function.

  Precondition:
    The TCP/IP Stack must be initialized and up and running.


  Parameters:
    - netH           - The handle of the network interface to use for the request. Can be 0 if 
	                   a default interface is to be selected 
    - targetAddr     - The IP address of the remote Host
    - sequenceNumber - A sequence number to be used in the request
    - identifier     - An identifier to be used in the request

  Returns:
    - ICMP_ECHO_OK     - Indicates the query request was successfully sent
    - ICMP_ECHO_RESULT - The query request was unsuccessfully sent, which results in an error code
                         (interface not ready for transmission, allocation error, etc.)

  Example:
  <code>
    IPV4_ADDR remoteAddress = 0xc0a00101;
    uint16_t mySequenceNumber = 1;
    uint16_t myId = 0x1234;

    if(TCPIP_ICMP_EchoRequestSend(0, &remoteAddress, mySequenceNumber, myId) == ICMP_ECHO_OK )
    {
        // successfully sent the ICMP request
        //
    }
    else
    {
        // process the error
    }
  </code>

  Remarks:
    None.
*/
ICMP_ECHO_RESULT TCPIP_ICMP_EchoRequestSend (TCPIP_NET_HANDLE netH, IPV4_ADDR * targetAddr, 
                                             uint16_t sequenceNumber, uint16_t identifier);


// *****************************************************************************
/* Function:
    ICMP_HANDLE TCPIP_ICMP_CallbackRegister (void (*callback)(TCPIP_NET_HANDLE hNetIf, 
	                                         IPV4_ADDR * remoteIP, void * data))

  Summary:
    Registers a callback to allow the application layer to process incoming ICMPv4 packets

  Description:
    Allows a stack client to be notified of the receiving of a response from an ICMP query.
    Once an Echo request reply is received, the notification handler callback will be called,
    letting the client know of the result of the query.
    The callback will contain as parameters:
        - the network interface handle on which the query reply was received
        - the remote host IP address
        - a 32-bit value containing the sequence number in the low 16-bit part
          and the identifier value in the high 16-bit part.

  Precondition:
    The TCP/IP Stack must be initialized and up and running.

  Parameters:
    callback    - notification function to be registered.
                  This function will be called when an echo request reply is received.

  Returns:
    - A non-null handle if the registration succeeded
    - 0 if the registration operation failed (for example, out of memory)

  Example:
  <code>
    // Callback function prototype
    void MyICMPCallbackFunction(TCPIP_NET_HANDLE hNetIf, IPV4_ADDR * remoteIP, void * data);

    // *****************************************************************************
    // Register callback function
    ICMP_HANDLE hIcmp = TCPIP_ICMP_CallbackRegister(&MyICMPCallbackFunction);
    if(hIcmp == 0)
    {
        // process error; couldn't register a handler
    }

    // success; I can send an Echo request and receive notification


    // *****************************************************************************
    IPV4_ADDR remoteIP = 0xc0a00101;
    uint16_t mySequenceNumber = 1;
    uint16_t myId = 0x1234;
    // send an ICMP query request
    TCPIP_ICMP_EchoRequestSend(&remoteIP, mySequenceNumber, myId);


    // *****************************************************************************
    // process the ICMP reply in the callback function
    void MyICMPCallbackFunction(TCPIP_NET_HANDLE hNetIf, IPV4_ADDR * remoteIP, void * data)
    {
        // process request from interface hNetIf and remoteIP address
        uint16_t* pReply = (uint16_t*)data;
        uint16_t myRecvId = *pReply;
        uint16_t myRecvSequenceNumber = *(pReply + 1);

        // check that the sequence number, ID and IP address match, etc.
    }
  </code>

  Remarks:
    None.
*/
ICMP_HANDLE TCPIP_ICMP_CallbackRegister (void (*callback)(TCPIP_NET_HANDLE hNetIf, 
                                         IPV4_ADDR * remoteIP, void * data));


// *****************************************************************************
/* Function:
    bool  TCPIP_ICMP_CallbackDeregister(ICMP_HANDLE hIcmp)

  Summary:
    Deregisters the ICMP callback function.

  Description:
    This function notifies a stack client to remove its former registered notification handler.
    After this operation the client will no longer be notified about the receiving
    of replies to the ICMP requests.

  Precondition:
    The TCP/IP Stack must be initialized and up and running.
    A previous successful call to TCPIP_ICMP_CallbackRegister has been done.

  Parameters:
    hIcmp   - an ICMP handle obtained by TCPIP_ICMP_CallbackRegister
	
  Returns:
    - true  - if the notification handler has been successfully removed
    - false - if such a notification handler could not be found

  Example:
  <code>
    void MyICMPCallbackFunction(TCPIP_NET_HANDLE hNetIf, IPV4_ADDR * remoteIP, void * data);
    ICMP_HANDLE hIcmp = TCPIP_ICMP_CallbackRegister(&MyICMPCallbackFunction);
    if(hIcmp != 0)
    {
        // successfully registered my handler
        // send requests and process the incoming results
        // ...
        // later on, once we're done, remove the notification handler
        TCPIP_ICMP_CallbackDeregister(hIcmp);
    }
  </code>

  Remarks:
    None.
*/
bool  TCPIP_ICMP_CallbackDeregister(ICMP_HANDLE hIcmp);


// *****************************************************************************
/*
  Function:
    void  TCPIP_ICMP_Task(void)

  Summary:
    Standard TCP/IP stack module task function.

  Description:
    This function performs ICMPv4 module tasks in the TCP/IP stack.

  Precondition:
    The ICMP module should have been initialized.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
*/
void  TCPIP_ICMP_Task(void);

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif  // __ICMP_H
