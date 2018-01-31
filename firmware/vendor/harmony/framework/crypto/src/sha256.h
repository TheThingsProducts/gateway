/**************************************************************************
  Crypto Framework Library Header

  Company:
    Microchip Technology Inc.

  File Name:
    sha256.h
  
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




/* code submitted by raphael.huck@efixo.com */

#ifndef CRYPT_SHA256_H
#define CRYPT_SHA256_H

#include "crypto/src/types.h"

#ifndef NO_SHA256

#ifdef __cplusplus
    extern "C" {
#endif

#ifdef WOLFSSL_PIC32MZ_HASH
    #include "pic32mz-crypt.h"
#endif

/* in bytes */
enum {
    SHA256              =  2,   /* hash type unique */
    SHA256_BLOCK_SIZE   = 64,
    SHA256_DIGEST_SIZE  = 32,
    SHA256_PAD_SIZE     = 56
};

/* Sha256 digest */
typedef struct Sha256 {
    word32  buffLen;   /* in bytes          */
    word32  loLen;     /* length in bytes   */
    word32  hiLen;     /* length in bytes   */
    word32  digest[SHA256_DIGEST_SIZE / sizeof(word32)];
    word32  buffer[SHA256_BLOCK_SIZE  / sizeof(word32)];
    #ifdef WOLFSSL_PIC32MZ_HASH
        pic32mz_desc desc ; /* Crypt Engine descripter */
    #endif
} Sha256;

WOLFSSL_API int wc_InitSha256(Sha256*);
WOLFSSL_API int wc_Sha256Update(Sha256*, const byte*, word32);
WOLFSSL_API int wc_Sha256Final(Sha256*, byte*);
#ifdef WOLFSSL_PIC32MZ_HASH
WOLFSSL_API void wc_Sha256SizeSet(Sha256*, word32);
#endif

#ifdef __cplusplus
    } /* extern "C" */
#endif

#endif /* NO_SHA256 */
#endif /* CRYPT_SHA256_H */

