/*******************************************************************************
  TLS Layer  API Header File

  Company:
    Microchip Technology Inc.
    
  File Name:
    tls.h

  Summary:
    The TLS layer adds encryption support to the TCP layer by providing an interface
    for including an application specific TLS layer

  Description:
    The TLS module adds encryption support to the TCP layer by providing an interface
    for including an application specific TLS layer.  It is up to the application
    to provide the mechanism for negotiating connection, encrypting and decrypting
    traffic.
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

#ifndef __TLS_H
#define __TLS_H

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  

// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************

//*****************************************************************************
/*
  Function:
    bool TCPIP_TCPTLS_ClientStart(TCP_SOCKET hTCP, uint8_t* host, uint8_t certIndex)

  Summary:
    Begins an TLS client session.

  Description:
    This function escalates the current connection to an TLS secured
    connection by initiating an TLS client handshake.

  Precondition:
    TCP is initialized and hTCP is already connected.

  Parameters:
    hTCP        - TCP connection to secure
    host        - Expected host name on certificate (currently ignored)
    certIndex   - Passed through to the TLS layer's clientCreate function.  It 
                  can be used to select a specific certificate for client certificate
                  validation, or it can be ignored

  Return Values:
    - true        - a TLS connection was initiated
    - false       - Insufficient TLS resources (stubs) were available

  Remarks:
    The host parameter is currently ignored and is not validated.
 */
bool  TCPIP_TLSTCP_ClientStart(TCP_SOCKET hTCP, uint8_t* host, uint8_t certIndex);


//*****************************************************************************
/*
  Function:
    bool TCPIP_TCPTLS_ServerStart(TCP_SOCKET hTCP, uint8_t certIndex)

  Summary:
    Begins a TLS server session.

  Description:
    This function sets up a TLS server session when a new connection is
    established on an TLS port.

  Precondition:
    TCP is initialized and hTCP is already connected.

  Parameters:
    hTCP        - TCP connection to secure
    certIndex   - Passed through to the TLS layer's serverCreate function.  It 
                  can be used to select a specific certificate to be used as the
                  server certificate, or it can be ignored

  Return Values:
    - true        - an TLS connection was initiated
    - false       - Insufficient TLS resources (stubs) were available
	
  Remarks:
    None.
 */
bool  TCPIP_TLSTCP_ServerStart(TCP_SOCKET hTCP, uint8_t certIndex);


//*****************************************************************************
/*
  Function:
    bool TCPIP_TCPTLS_StillHandshaking(TCP_SOCKET hTCP)

  Summary:
    Determines if an TLS session is still handshaking.

  Description:
    Call this function after calling TCPIP_TCPTLS_ClientStart until false is
    returned.  Then your application may continue with its normal data
    transfer (which is now secured).

  Precondition:
    TCP is initialized and hTCP is connected.

  Parameters:
    hTCP        - TCP connection to check

  Return Values:
    - true        - TLS handshake is still progressing
    - false       - TLS handshake has completed

  Remarks:
    None.
	*/
bool  TCPIP_TLSTCP_StillHandshaking(TCP_SOCKET hTCP);
 
//******************************************************************************
/*
  Function:
    bool TCPIP_TCP_SocketIsSecuredByTLS(TCP_SOCKET hTCP)

  Summary:
    Determines if a TCP connection is secured with TLS.

  Description:
    Call this function to determine whether or not a TCP connection is
    secured with TLS.

  Precondition:
    TCP is initialized and hTCP is connected.

  Parameters:
    hTCP        - TCP connection to check

  Return Values:
    - true        - Connection is secured via TLS
    - false       - Connection is not secured
	
  Remarks:
    None.

  */
bool  TCPIP_TLSTCP_SocketIsSecuredByTLS(TCP_SOCKET hTCP);

//*****************************************************************************
/*
  Function:
    bool TCPIP_TCPTLS_ClientStart(TCP_SOCKET hTCP, uint8_t* host, uint8_t certIndex)

  Summary:
    Begins an TLS client session.

  Description:
    This function escalates the current connection to an TLS secured
    connection by initiating an TLS client handshake.

  Precondition:
    TCP is initialized and hTCP is already connected.

  Parameters:
    hTCP        - TCP connection to secure
    host        - Expected host name on certificate (currently ignored)
    certIndex   - Passed through to the TLS layer's clientCreate function.  It 
                  can be used to select a specific certificate for client certificate
                  validation, or it can be ignored

  Return Values:
    - true        - a TLS connection was initiated
    - false       - Insufficient TLS resources (stubs) were available

  Remarks:
    The host parameter is currently ignored and is not validated.
 */
bool  TCPIP_TLUDP_ClientStart(UDP_SOCKET hUDP, uint8_t* host, uint8_t certIndex);


//*****************************************************************************
/*
  Function:
    bool TCPIP_TCPTLS_ServerStart(TCP_SOCKET hTCP, uint8_t certIndex)

  Summary:
    Begins a TLS server session.

  Description:
    This function sets up a TLS server session when a new connection is
    established on an TLS port.

  Precondition:
    TCP is initialized and hTCP is already connected.

  Parameters:
    hTCP        - TCP connection to secure
    certIndex   - Passed through to the TLS layer's serverCreate function.  It 
                  can be used to select a specific certificate to be used as the
                  server certificate, or it can be ignored

  Return Values:
    - true        - an TLS connection was initiated
    - false       - Insufficient TLS resources (stubs) were available
 */
bool  TCPIP_TLSUDP_ServerStart(UDP_SOCKET hUDP, uint8_t certIndex);


//*****************************************************************************
/*
  Function:
    bool TCPIP_TCPTLS_StillHandshaking(TCP_SOCKET hTCP)

  Summary:
    Determines if an TLS session is still handshaking.

  Description:
    Call this function after calling TCPIP_TCPTLS_ClientStart until false is
    returned.  Then your application may continue with its normal data
    transfer (which is now secured).

  Precondition:
    TCP is initialized and hTCP is connected.

  Parameters:
    hTCP        - TCP connection to check

  Return Values:
    - true        - TLS handshake is still progressing
    - false       - TLS handshake has completed
	
  Remarks:
    None.
 */
bool  TCPIP_TLSUDP_StillHandshaking(UDP_SOCKET hUDP);
 
//******************************************************************************
/*
  Function:
    bool TCPIP_TCP_SocketIsSecuredByTLS(TCP_SOCKET hTCP)

  Summary:
    Determines if a TCP connection is secured with TLS.

  Description:
    Call this function to determine whether or not a TCP connection is
    secured with TLS.

  Precondition:
    TCP is initialized and hTCP is connected.

  Parameters:
    hTCP        - TCP connection to check

  Return Values:
    - true        - Connection is secured via TLS
    - false       - Connection is not secured
  */
bool  TCPIP_TLSUDP_SocketIsSecuredByTLS(UDP_SOCKET hUDP);


//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif