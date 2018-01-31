/**************************************************************************
Announce Service Module API Definitions Header File

  Company:
    Microchip Technology Inc.
	
  File Name:
    tcpip_announce.h
	
  Summary:
    Announce provides a simple discovery mechanism for Microchip TCP/IP hosts.
	
  Description:
    Announce Service Module for Microchip TCP/IP Stack.
    It is a simple, proprietary discovery mechanism for Microchip TCP/IP hosts.
    It works in conjunction with a GUI tool running on a PC.

  **************************************************************************/
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

#ifndef __TCPIP_ANNOUNCE_H
#define __TCPIP_ANNOUNCE_H

#include <stdint.h>
#include <stdbool.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  

// *****************************************************************************
/*
 Announce Configuration structure placeholder
*/
typedef struct
{
}TCPIP_ANNOUNCE_MODULE_CONFIG;


// *****************************************************************************
/*
  Function:
    void  TCPIP_ANNOUNCE_Task(void)

  Summary:
    Standard TCP/IP stack module task function.

  Description:
    This function performs Announce module tasks in the TCP/IP stack.

  Precondition:
    The Announce module should have been initialized.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
*/
void  TCPIP_ANNOUNCE_Task(void);
    
//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif	//#ifndef __TCPIP_ANNOUNCE_H