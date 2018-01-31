/**************************************************************************
  Crypto Framework Library Source

  Company:
    Microchip Technology Inc.

  File Name:
    hash.h
  
  Summary:
    Crypto Framework Libarary source for cryptographic functions.

  Description:
    This source file contains functions that make up the Cryptographic 
    Framework Library for PIC32 families of Microchip microcontrollers.
**************************************************************************/

//DOM-IGNORE-BEGIN
/******************************************************************************
File Name:  hash.h
Copyright © 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

#ifndef CRYPT_HASH_H
#define CRYPT_HASH_H

#include "crypto/src/types.h"
#include "system_config.h"

#ifdef __cplusplus
    extern "C" {
#endif

/* Hash types */
enum wc_HashType {
    WC_HASH_TYPE_NONE = 0,
    WC_HASH_TYPE_MD2 = 1,
    WC_HASH_TYPE_MD4 = 2,
    WC_HASH_TYPE_MD5 = 3,
    WC_HASH_TYPE_SHA = 4, /* SHA-1 (not old SHA-0) */
    WC_HASH_TYPE_SHA256 = 5,
    WC_HASH_TYPE_SHA384 = 6,
    WC_HASH_TYPE_SHA512 = 7,
};

/* Find largest possible digest size
   Note if this gets up to the size of 80 or over check smallstack build */
#if defined(WOLFSSL_SHA512)
    #define WC_MAX_DIGEST_SIZE SHA512_DIGEST_SIZE
#elif defined(WOLFSSL_SHA384)
    #define WC_MAX_DIGEST_SIZE SHA384_DIGEST_SIZE
#elif !defined(NO_SHA256)
    #define WC_MAX_DIGEST_SIZE SHA256_DIGEST_SIZE
#elif !defined(NO_SHA)
    #define WC_MAX_DIGEST_SIZE SHA_DIGEST_SIZE
#elif !defined(NO_MD5)
    #define WC_MAX_DIGEST_SIZE MD5_DIGEST_SIZE
#else
    #define WC_MAX_DIGEST_SIZE 64 /* default to max size of 64 */
#endif

#ifndef NO_ASN
WOLFSSL_API int wc_HashGetOID(enum wc_HashType hash_type);
#endif

WOLFSSL_API int wc_HashGetDigestSize(enum wc_HashType hash_type);
WOLFSSL_API int wc_Hash(enum wc_HashType hash_type,
    const byte* data, word32 data_len,
    byte* hash, word32 hash_len);


#ifndef NO_MD5
#include "crypto/src/md5.h"
WOLFSSL_API void wc_Md5GetHash(Md5*, byte*);
WOLFSSL_API void wc_Md5RestorePos(Md5*, Md5*);
#if defined(WOLFSSL_TI_HASH)
    WOLFSSL_API void wc_Md5Free(Md5*);
#else
    #define wc_Md5Free(d)
#endif
#endif

#ifndef NO_SHA
#include "crypto/src/sha.h"
WOLFSSL_API int wc_ShaGetHash(Sha*, byte*);
WOLFSSL_API void wc_ShaRestorePos(Sha*, Sha*);
WOLFSSL_API int wc_ShaHash(const byte*, word32, byte*);
#if defined(WOLFSSL_TI_HASH)
     WOLFSSL_API void wc_ShaFree(Sha*);
#else
    #define wc_ShaFree(d)
#endif
#endif

#ifndef NO_SHA256
#include "crypto/src/sha256.h"
WOLFSSL_API int wc_Sha256GetHash(Sha256*, byte*);
WOLFSSL_API void wc_Sha256RestorePos(Sha256*, Sha256*);
WOLFSSL_API int wc_Sha256Hash(const byte*, word32, byte*);
#if defined(WOLFSSL_TI_HASH)
    WOLFSSL_API void wc_Sha256Free(Sha256*);
#else
    #define wc_Sha256Free(d)
#endif
#endif

#ifdef WOLFSSL_SHA512
#include "crypto/src/sha512.h"
WOLFSSL_API int wc_Sha512Hash(const byte*, word32, byte*);
#if defined(WOLFSSL_TI_HASH)
    WOLFSSL_API void wc_Sha512Free(Sha512*);
#else
    #define wc_Sha512Free(d)
#endif
    #if defined(WOLFSSL_SHA384)
        WOLFSSL_API int wc_Sha384Hash(const byte*, word32, byte*);
        #if defined(WOLFSSL_TI_HASH)
            WOLFSSL_API void wc_Sha384Free(Sha384*);
        #else
            #define wc_Sha384Free(d)
        #endif
    #endif /* defined(WOLFSSL_SHA384) */
#endif /* WOLFSSL_SHA512 */


#ifdef __cplusplus
    } /* extern "C" */
#endif

#endif /* CRYPT_HASH_H */
