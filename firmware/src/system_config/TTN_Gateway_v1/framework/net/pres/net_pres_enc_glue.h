/*******************************************************************************
 Header file for the wolfSSL glue functions to work with Harmony


  Summary:


  Description:

*******************************************************************************/

/*******************************************************************************
File Name: net_tls_wolfssl_glue.h
Copyright (c) 2013 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY KIND,
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

#ifndef _NET_TLS_WOLFSSL_GLUE_H_
#define _NET_TLS_WOLFSSL_GLUE_H_

#include "system_config.h"
#include "net/pres/net_pres.h"
#include "net/pres/net_pres_encryptionproviderapi.h"
#ifdef __CPLUSPLUS
extern "c" {
#endif
extern NET_PRES_EncProviderObject net_pres_EncProviderStreamClient0;
bool NET_PRES_EncProviderStreamClientInit0(struct _NET_PRES_TransportObject * transObject);
bool NET_PRES_EncProviderStreamClientDeinit0();
bool NET_PRES_EncProviderStreamClientOpen0(uintptr_t transHandle, void * providerData);
bool NET_PRES_EncProviderStreamClientIsInited0();
NET_PRES_EncSessionStatus NET_PRES_EncProviderClientConnect0(void * providerData);
NET_PRES_EncSessionStatus NET_PRES_EncProviderConnectionClose0(void * providerData);
int32_t NET_PRES_EncProviderWrite0(void * providerData, const uint8_t * buffer, uint16_t size);
uint16_t  NET_PRES_EncProviderWriteReady0(void * providerData, uint16_t reqSize, uint16_t minSize);
int32_t NET_PRES_EncProviderRead0(void * providerData, uint8_t * buffer, uint16_t size);
int32_t NET_PRES_EncProviderReadReady0(void * providerData);
int32_t NET_PRES_EncProviderPeek0(void * providerData, uint8_t * buffer, uint16_t size);
#ifdef __CPLUSPLUS
}
#endif
#endif //_NET_TLS_WOLFSSL_GLUE_H_
