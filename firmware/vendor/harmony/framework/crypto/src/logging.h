/**************************************************************************
  Crypto Framework Library Header

  Company:
    Microchip Technology Inc.

  File Name:
    logging.h
  
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



/* submitted by eof */


#ifndef CRYPTO_LOGGING_H
#define CRYPTO_LOGGING_H

#include "crypto/src/types.h"

#ifdef __cplusplus
    extern "C" {
#endif


enum  CYA_Log_Levels {
    ERROR_LOG = 0,
    INFO_LOG,
    ENTER_LOG,
    LEAVE_LOG,
    OTHER_LOG
};

typedef void (*wolfSSL_Logging_cb)(const int logLevel,
                                  const char *const logMessage);

WOLFSSL_API int wolfSSL_SetLoggingCb(wolfSSL_Logging_cb log_function);

#ifdef DEBUG_WOLFSSL
    /* a is prepended to m and b is appended, creating a log msg a + m + b */
    #define WOLFSSL_LOG_CAT(a, m, b) #a " " m " "  #b

    void WOLFSSL_ENTER(const char* msg);
    void WOLFSSL_LEAVE(const char* msg, int ret);
    #define WOLFSSL_STUB(m) \
        WOLFSSL_MSG(WOLFSSL_LOG_CAT(wolfSSL Stub, m, not implemented))

    void WOLFSSL_ERROR(int);
    void WOLFSSL_MSG(const char* msg);
    void WOLFSSL_BUFFER(byte* buffer, word32 length);

#else /* DEBUG_WOLFSSL   */

    #define WOLFSSL_ENTER(m)
    #define WOLFSSL_LEAVE(m, r)
    #define WOLFSSL_STUB(m)

    #define WOLFSSL_ERROR(e)
    #define WOLFSSL_MSG(m)
    #define WOLFSSL_BUFFER(b, l)

#endif /* DEBUG_WOLFSSL  */

#ifdef __cplusplus
}
#endif
#endif /* CRYPTO_LOGGING_H */

