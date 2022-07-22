/*********************************************************************
 * File Name: http_print.c
 *
 * Provides callback headers and resolution for user's custom
 * HTTP Application.
 * 
 * This file is automatically generated by the MPFS Utility
 * ALL MODIFICATIONS WILL BE OVERWRITTEN BY THE MPFS GENERATOR
 *
 * Software License Agreement
 *
 * Copyright (C) 2012 Microchip Technology Inc.  All rights
 * reserved.
 * Microchip licenses to you the right to use, modify, copy, and distribute
 * software only embedded on a Microchip microcontroller or digital signal 
 * controller that is integrated into your product or third party product
 * (pursuant to the sublicense terms in the accompanying license agreement)

 * You should refer to the license agreement accompanying this 
 * Software for additional information regarding your rights and 
 * obligations.
 *
 * You should refer to the license agreement accompanying this 
 * Software for additional information regarding your rights and 
 * obligations.
 *
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A 
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE 
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER 
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE.
 *
 *********************************************************************/

#include "tcpip/tcpip.h"

void TCPIP_HTTP_Print(HTTP_CONN_HANDLE connHandle,uint32_t callbackID);
void TCPIP_HTTP_Print_nextSSID(HTTP_CONN_HANDLE connHandle);
void TCPIP_HTTP_Print_prevSSID(HTTP_CONN_HANDLE connHandle);
void TCPIP_HTTP_Print_scanresult(HTTP_CONN_HANDLE connHandle);
void TCPIP_HTTP_Print_scan(HTTP_CONN_HANDLE connHandle);
void TCPIP_HTTP_Print_gwsettings(HTTP_CONN_HANDLE connHandle);
void TCPIP_HTTP_Print_gwstatus(HTTP_CONN_HANDLE connHandle);

void TCPIP_HTTP_Print(HTTP_CONN_HANDLE connHandle,uint32_t callbackID)
{
    TCP_SOCKET sktHTTP; 
    switch(callbackID)
    {
        case 0x00000000:
			TCPIP_HTTP_FileInclude(connHandle,(const uint8_t *)"header.inc");
			break;
        case 0x00000001:
			TCPIP_HTTP_Print_nextSSID(connHandle);
			break;
        case 0x00000002:
			TCPIP_HTTP_Print_prevSSID(connHandle);
			break;
        case 0x00000005:
			TCPIP_HTTP_Print_scanresult(connHandle);
			break;
        case 0x00000006:
			TCPIP_HTTP_Print_scan(connHandle);
			break;
        case 0x00000007:
			TCPIP_HTTP_Print_gwsettings(connHandle);
			break;
        case 0x00000008:
			TCPIP_HTTP_Print_gwstatus(connHandle);
			break;
        default:
            // Output notification for undefined values
            sktHTTP = TCPIP_HTTP_CurrentConnectionSocketGet(connHandle);
            TCPIP_TCP_ArrayPut(sktHTTP, (const uint8_t *)"!DEF", 4);
    }
    return;
}

void TCPIP_HTTP_Print_myVariable(HTTP_CONN_HANDLE connHandle)
{
    TCP_SOCKET sktHTTP = TCPIP_HTTP_CurrentConnectionSocketGet(connHandle);
    TCPIP_TCP_Put(sktHTTP, '~');
    return;
}
