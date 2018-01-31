/**************************************************************************
  Crypto Framework Library Header

  Company:
    Microchip Technology Inc.

  File Name:
    port.h
  
  Summary:
    Crypto Framework Library header for cryptographic functions.

  Description:
    This header file contains function prototypes and definitions of
    the data types and constants that make up the Cryptographic Framework
    Library for PIC32 families of Microchip microcontrollers.
**************************************************************************/

//DOM-IGNORE-BEGIN
/******************************************************************************
Copyright © 2013 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END


#ifndef CRYPT_PORT_H
#define CRYPT_PORT_H


#ifdef __cplusplus
    extern "C" {
#endif


#ifdef THREADX
    #ifndef SINGLE_THREADED
        #include "tx_api.h"
    #endif
#elif defined(MICRIUM)
    /* do nothing, just don't pick Unix */
#elif defined(FREERTOS) || defined(WOLFSSL_SAFERTOS)
    /* do nothing */
#elif defined(EBSNET)
    /* do nothing */
#else
    #ifndef SINGLE_THREADED
        #define WOLFSSL_PTHREADS
        #include "crypto/src/pthread.h"
    #endif
    #if defined(OPENSSL_EXTRA) || defined(GOAHEAD_WS)
        #include "crypto/src/unistd.h"      /* for close of BIO */
    #endif
#endif


#ifdef SINGLE_THREADED
    typedef int wolfSSL_Mutex;
#else /* MULTI_THREADED */
    /* FREERTOS comes first to enable use of FreeRTOS Windows simulator only */
    #ifdef FREERTOS
        typedef xSemaphoreHandle wolfSSL_Mutex;
    #elif defined(WOLFSSL_SAFERTOS)
        typedef struct wolfSSL_Mutex {
            signed char mutexBuffer[portQUEUE_OVERHEAD_BYTES];
            xSemaphoreHandle mutex;
        } wolfSSL_Mutex;
    #elif defined(WOLFSSL_PTHREADS)
        typedef pthread_mutex_t wolfSSL_Mutex;
    #elif defined(THREADX)
        typedef TX_MUTEX wolfSSL_Mutex;
    #elif defined(MICRIUM)
        typedef OS_MUTEX wolfSSL_Mutex;
    #elif defined(EBSNET)
        typedef RTP_MUTEX wolfSSL_Mutex;
    #elif defined(FREESCALE_MQX)
        typedef MUTEX_STRUCT wolfSSL_Mutex;
    #else
        #error Need a mutex type in multithreaded mode
    #endif /* FREERTOS */
#endif /* SINGLE_THREADED */

WOLFSSL_LOCAL int InitMutex(wolfSSL_Mutex*);
WOLFSSL_LOCAL int FreeMutex(wolfSSL_Mutex*);
WOLFSSL_LOCAL int LockMutex(wolfSSL_Mutex*);
WOLFSSL_LOCAL int UnLockMutex(wolfSSL_Mutex*);


#ifdef __cplusplus
    }  /* extern "C" */
#endif

#endif /* CRYPT_PORT_H */

