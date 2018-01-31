/**************************************************************************
  Crypto Framework Library Source

  Company:
    Microchip Technology Inc.

  File Name:
    logging.c
  
  Summary:
    Crypto Framework Libarary source for cryptographic functions.

  Description:
    This source file contains functions that make up the Cryptographic 
	Framework Library for PIC32 families of Microchip microcontrollers.
**************************************************************************/

//DOM-IGNORE-BEGIN
/******************************************************************************
File Name:  logging.c
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



#ifdef HAVE_CONFIG_H
    #include "config.h"
#endif
#include "system_config.h"

#include "crypto/src/settings.h"

/* submitted by eof */

#include "crypto/src/logging.h"
#include "crypto/src/error-crypt.h"


#ifdef __cplusplus
    extern "C" {
#endif
    WOLFSSL_API int  wolfSSL_Debugging_ON(void);
    WOLFSSL_API void wolfSSL_Debugging_OFF(void);
#ifdef __cplusplus
    } 
#endif


#ifdef DEBUG_WOLFSSL

/* Set these to default values initially. */
static wolfSSL_Logging_cb log_function = 0;
static int loggingEnabled = 0;

#endif /* DEBUG_WOLFSSL */


int wolfSSL_SetLoggingCb(wolfSSL_Logging_cb f)
{
#ifdef DEBUG_WOLFSSL
    int res = 0;

    if (f)
        log_function = f;
    else
        res = BAD_FUNC_ARG;

    return res;
#else
    (void)f;
    return NOT_COMPILED_IN;
#endif
}


int wolfSSL_Debugging_ON(void)
{
#ifdef DEBUG_WOLFSSL
    loggingEnabled = 1;
    return 0;
#else
    return NOT_COMPILED_IN;
#endif
}


void wolfSSL_Debugging_OFF(void)
{
#ifdef DEBUG_WOLFSSL
    loggingEnabled = 0;
#endif
}


#ifdef DEBUG_WOLFSSL
#include <stdio.h>   /* for default printf stuff */

#ifdef THREADX
    int dc_log_printf(char*, ...);
#endif

static void wolfssl_log(const int logLevel, const char *const logMessage)
{
    if (log_function)
        log_function(logLevel, logMessage);
    else {
        if (loggingEnabled) {
#ifdef THREADX
            dc_log_printf("%s\n", logMessage);
#elif defined(MICRIUM)
        #if (NET_SECURE_MGR_CFG_EN == DEF_ENABLED)
            NetSecure_TraceOut((CPU_CHAR *)logMessage);
        #endif
#elif defined(WOLFSSL_MDK_ARM)
            fflush(stdout) ;
            printf("%s\n", logMessage);
            fflush(stdout) ;
#elif defined(WOLFSSL_LOG_PRINTF)
            printf("%s\n", logMessage);
#else
            fprintf(stderr, "%s\n", logMessage);
#endif
        }
    }
}


void WOLFSSL_MSG(const char* msg)
{
    if (loggingEnabled)
        wolfssl_log(INFO_LOG , msg);
}


void WOLFSSL_BUFFER(byte* buffer, word32 length)
{
    #define LINE_LEN 16

    if (loggingEnabled) {
        word32 i;
        char line[80];

        if (!buffer) {
            wolfssl_log(INFO_LOG, "\tNULL");

            return;
        }

        sprintf(line, "\t");

        for (i = 0; i < LINE_LEN; i++) {
            if (i < length)
                sprintf(line + 1 + i * 3,"%02x ", buffer[i]);
            else
                sprintf(line + 1 + i * 3, "   ");
        }

        sprintf(line + 1 + LINE_LEN * 3, "| ");

        for (i = 0; i < LINE_LEN; i++)
            if (i < length)
                sprintf(line + 3 + LINE_LEN * 3 + i,
                     "%c", 31 < buffer[i] && buffer[i] < 127 ? buffer[i] : '.');

        wolfssl_log(INFO_LOG, line);

        if (length > LINE_LEN)
            WOLFSSL_BUFFER(buffer + LINE_LEN, length - LINE_LEN);
    }
}


void WOLFSSL_ENTER(const char* msg)
{
    if (loggingEnabled) {
        char buffer[80];
        sprintf(buffer, "wolfSSL Entering %s", msg);
        wolfssl_log(ENTER_LOG , buffer);
    }
}


void WOLFSSL_LEAVE(const char* msg, int ret)
{
    if (loggingEnabled) {
        char buffer[80];
        sprintf(buffer, "wolfSSL Leaving %s, return %d", msg, ret);
        wolfssl_log(LEAVE_LOG , buffer);
    }
}


void WOLFSSL_ERROR(int error)
{
    if (loggingEnabled) {
        char buffer[80];
        sprintf(buffer, "wolfSSL error occurred, error = %d", error);
        wolfssl_log(ERROR_LOG , buffer);
    }
}

#endif  /* DEBUG_WOLFSSL */
