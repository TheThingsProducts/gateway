/**************************************************************************
  Crypto Framework Library Header

  Company:
    Microchip Technology Inc.

  File Name:
    sha512.h
  
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

#ifndef CRYPT_SHA512_H
#define CRYPT_SHA512_H

#include "crypto/src/types.h"

#ifdef WOLFSSL_SHA512


#ifdef __cplusplus
    extern "C" {
#endif


/* in bytes */
enum {
    SHA512              =   4,   /* hash type unique */
    SHA512_BLOCK_SIZE   = 128,
    SHA512_DIGEST_SIZE  =  64,
    SHA512_PAD_SIZE     = 112
};


/* Sha512 digest */
typedef struct Sha512 {
    word32  buffLen;   /* in bytes          */
    word32  loLen;     /* length in bytes   */
    word32  hiLen;     /* length in bytes   */
    word64  digest[SHA512_DIGEST_SIZE / sizeof(word64)];
    word64  buffer[SHA512_BLOCK_SIZE  / sizeof(word64)];
} Sha512;


WOLFSSL_API int wc_InitSha512(Sha512*);
WOLFSSL_API int wc_Sha512Update(Sha512*, const byte*, word32);
WOLFSSL_API int wc_Sha512Final(Sha512*, byte*);

#if defined(WOLFSSL_SHA384)

/* in bytes */
enum {
    SHA384              =   5,   /* hash type unique */
    SHA384_BLOCK_SIZE   = 128,
    SHA384_DIGEST_SIZE  =  48,
    SHA384_PAD_SIZE     = 112
};


/* Sha384 digest */
typedef struct Sha384 {
    word32  buffLen;   /* in bytes          */
    word32  loLen;     /* length in bytes   */
    word32  hiLen;     /* length in bytes   */
    word64  digest[SHA512_DIGEST_SIZE / sizeof(word64)]; /* for transform 512 */
    word64  buffer[SHA384_BLOCK_SIZE  / sizeof(word64)];
} Sha384;

WOLFSSL_API int wc_InitSha384(Sha384*);
WOLFSSL_API int wc_Sha384Update(Sha384*, const byte*, word32);
WOLFSSL_API int wc_Sha384Final(Sha384*, byte*);

#endif /* WOLFSSL_SHA384 */

#ifdef __cplusplus
    } /* extern "C" */
#endif

#endif /* WOLFSSL_SHA512 */
#endif /* CRYPT_SHA512_H */

