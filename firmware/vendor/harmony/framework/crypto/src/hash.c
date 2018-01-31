/**************************************************************************
  Crypto Framework Library Source

  Company:
    Microchip Technology Inc.

  File Name:
    hash.c
  
  Summary:
    Crypto Framework Libarary source for cryptographic functions.

  Description:
    This source file contains functions that make up the Cryptographic 
	Framework Library for PIC32 families of Microchip microcontrollers.
**************************************************************************/

//DOM-IGNORE-BEGIN
/******************************************************************************
File Name:  hash.c
Copyright © 2016 released Microchip Technology Inc.  All rights reserved.

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
    #include <config.h>
#endif

#include "crypto/src/settings.h"
#include "crypto/src/logging.h"
#include "crypto/src/error-crypt.h"

#ifndef NO_ASN
#include "crypto/src/asn.h"
#endif
#include "crypto/src/hash.h"


#ifndef NO_ASN
int wc_HashGetOID(enum wc_HashType hash_type)
{
    int oid = HASH_TYPE_E; /* Default to hash type error */
    switch(hash_type)
    {
        case WC_HASH_TYPE_MD2:
#ifdef WOLFSSL_MD2
            oid = MD2h;
#endif
            break;
        case WC_HASH_TYPE_MD5:
#ifndef NO_MD5
            oid = MD5h;
#endif
            break;
        case WC_HASH_TYPE_SHA:
#ifndef NO_SHA
            oid = SHAh;
#endif
            break;
        case WC_HASH_TYPE_SHA256:
#ifndef NO_SHA256
            oid = SHA256h;
#endif
            break;
        case WC_HASH_TYPE_SHA384:
#if defined(WOLFSSL_SHA512) && defined(WOLFSSL_SHA384)
            oid = SHA384h;
#endif
            break;
        case WC_HASH_TYPE_SHA512:
#ifdef WOLFSSL_SHA512
            oid = SHA512h;
#endif
            break;

        /* Not Supported */
        case WC_HASH_TYPE_MD4:
        case WC_HASH_TYPE_NONE:
        default:
            oid = BAD_FUNC_ARG;
            break;
    }
    return oid;
}
#endif

/* Get Hash digest size */
int wc_HashGetDigestSize(enum wc_HashType hash_type)
{
    int dig_size = HASH_TYPE_E; /* Default to hash type error */
    switch(hash_type)
    {
        case WC_HASH_TYPE_MD5:
#ifndef NO_MD5
            dig_size = MD5_DIGEST_SIZE;
#endif
            break;
        case WC_HASH_TYPE_SHA:
#ifndef NO_SHA
            dig_size = SHA_DIGEST_SIZE;
#endif
            break;
        case WC_HASH_TYPE_SHA256:
#ifndef NO_SHA256
            dig_size = SHA256_DIGEST_SIZE;
#endif
            break;
        case WC_HASH_TYPE_SHA384:
#if defined(WOLFSSL_SHA512) && defined(WOLFSSL_SHA384)
            dig_size = SHA384_DIGEST_SIZE;
#endif
            break;
        case WC_HASH_TYPE_SHA512:
#ifdef WOLFSSL_SHA512
            dig_size = SHA512_DIGEST_SIZE;
#endif
            break;

        /* Not Supported */
        case WC_HASH_TYPE_MD2:
        case WC_HASH_TYPE_MD4:
        case WC_HASH_TYPE_NONE:
        default:
            dig_size = BAD_FUNC_ARG;
            break;
    }
    return dig_size;
}

/* Generic Hashing Wrapper */
int wc_Hash(enum wc_HashType hash_type, const byte* data,
    word32 data_len, byte* hash, word32 hash_len)
{
    int ret = HASH_TYPE_E; /* Default to hash type error */
    word32 dig_size;

    /* Validate hash buffer size */
    dig_size = wc_HashGetDigestSize(hash_type);
    if (hash_len < dig_size) {
        return BUFFER_E;
    }
    
    /* Suppress possible unused arg if all hashing is disabled */
    (void)data;
    (void)data_len;
    (void)hash;
    (void)hash_len;

    switch(hash_type)
    {
        case WC_HASH_TYPE_MD5:
#ifndef NO_MD5
            ret = wc_Md5Hash(data, data_len, hash);
#endif
            break;
        case WC_HASH_TYPE_SHA:
#ifndef NO_SHA
            ret = wc_ShaHash(data, data_len, hash);
#endif
            break;
        case WC_HASH_TYPE_SHA256:
#ifndef NO_SHA256
            ret = wc_Sha256Hash(data, data_len, hash);
#endif
            break;
        case WC_HASH_TYPE_SHA384:
#if defined(WOLFSSL_SHA512) && defined(WOLFSSL_SHA384)
            ret = wc_Sha384Hash(data, data_len, hash);
#endif
            break;
        case WC_HASH_TYPE_SHA512:
#ifdef WOLFSSL_SHA512
            ret = wc_Sha512Hash(data, data_len, hash);
#endif
            break;

        /* Not Supported */
        case WC_HASH_TYPE_MD2:
        case WC_HASH_TYPE_MD4:
        case WC_HASH_TYPE_NONE:
        default:
            ret = BAD_FUNC_ARG;
            break;
    }
    return ret;
}


#if !defined(WOLFSSL_TI_HASH)

#if !defined(NO_MD5)
void wc_Md5GetHash(Md5* md5, byte* hash)
{
    Md5 save = *md5 ;
    wc_Md5Final(md5, hash) ;
    *md5 = save ;
}

WOLFSSL_API void wc_Md5RestorePos(Md5* m1, Md5* m2) {
    *m1 = *m2 ;
}

#endif

#if !defined(NO_SHA)
int wc_ShaGetHash(Sha* sha, byte* hash)
{
    int ret ;
    Sha save = *sha ;
    ret = wc_ShaFinal(sha, hash) ;
    *sha = save ;
    return ret ;
}

void wc_ShaRestorePos(Sha* s1, Sha* s2) {
    *s1 = *s2 ;
}

int wc_ShaHash(const byte* data, word32 len, byte* hash)
{
    int ret = 0;
#ifdef WOLFSSL_SMALL_STACK
    Sha* sha;
#else
    Sha sha[1];
#endif

#ifdef WOLFSSL_SMALL_STACK
    sha = (Sha*)XMALLOC(sizeof(Sha), NULL, DYNAMIC_TYPE_TMP_BUFFER);
    if (sha == NULL)
        return MEMORY_E;
#endif

    if ((ret = wc_InitSha(sha)) != 0) {
        WOLFSSL_MSG("wc_InitSha failed");
    }
    else {
        wc_ShaUpdate(sha, data, len);
        wc_ShaFinal(sha, hash);
    }

#ifdef WOLFSSL_SMALL_STACK
    XFREE(sha, NULL, DYNAMIC_TYPE_TMP_BUFFER);
#endif

    return ret;

}

#endif /* !defined(NO_SHA) */

#if !defined(NO_SHA256)
int wc_Sha256GetHash(Sha256* sha256, byte* hash)
{
    int ret ;
    Sha256 save = *sha256 ;
    ret = wc_Sha256Final(sha256, hash) ;
    *sha256 = save ;
    return ret ;
}

void wc_Sha256RestorePos(Sha256* s1, Sha256* s2) {
    *s1 = *s2 ;
}

int wc_Sha256Hash(const byte* data, word32 len, byte* hash)
{
    int ret = 0;
#ifdef WOLFSSL_SMALL_STACK
    Sha256* sha256;
#else
    Sha256 sha256[1];
#endif

#ifdef WOLFSSL_SMALL_STACK
    sha256 = (Sha256*)XMALLOC(sizeof(Sha256), NULL, DYNAMIC_TYPE_TMP_BUFFER);
    if (sha256 == NULL)
        return MEMORY_E;
#endif

    if ((ret = wc_InitSha256(sha256)) != 0) {
        WOLFSSL_MSG("InitSha256 failed");
    }
    else if ((ret = wc_Sha256Update(sha256, data, len)) != 0) {
        WOLFSSL_MSG("Sha256Update failed");
    }
    else if ((ret = wc_Sha256Final(sha256, hash)) != 0) {
        WOLFSSL_MSG("Sha256Final failed");
    }

#ifdef WOLFSSL_SMALL_STACK
    XFREE(sha256, NULL, DYNAMIC_TYPE_TMP_BUFFER);
#endif

    return ret;
}

#endif /* !defined(NO_SHA256) */

#endif /* !defined(WOLFSSL_TI_HASH) */

#if defined(WOLFSSL_SHA512)
int wc_Sha512Hash(const byte* data, word32 len, byte* hash)
{
    int ret = 0;
#ifdef WOLFSSL_SMALL_STACK
    Sha512* sha512;
#else
    Sha512 sha512[1];
#endif

#ifdef WOLFSSL_SMALL_STACK
    sha512 = (Sha512*)XMALLOC(sizeof(Sha512), NULL, DYNAMIC_TYPE_TMP_BUFFER);
    if (sha512 == NULL)
        return MEMORY_E;
#endif

    if ((ret = wc_InitSha512(sha512)) != 0) {
        WOLFSSL_MSG("InitSha512 failed");
    }
    else if ((ret = wc_Sha512Update(sha512, data, len)) != 0) {
        WOLFSSL_MSG("Sha512Update failed");
    }
    else if ((ret = wc_Sha512Final(sha512, hash)) != 0) {
        WOLFSSL_MSG("Sha512Final failed");
    }

#ifdef WOLFSSL_SMALL_STACK
    XFREE(sha512, NULL, DYNAMIC_TYPE_TMP_BUFFER);
#endif

    return ret;
}

#if defined(WOLFSSL_SHA384)
int wc_Sha384Hash(const byte* data, word32 len, byte* hash)
{
    int ret = 0;
#ifdef WOLFSSL_SMALL_STACK
    Sha384* sha384;
#else
    Sha384 sha384[1];
#endif

#ifdef WOLFSSL_SMALL_STACK
    sha384 = (Sha384*)XMALLOC(sizeof(Sha384), NULL, DYNAMIC_TYPE_TMP_BUFFER);
    if (sha384 == NULL)
        return MEMORY_E;
#endif

    if ((ret = wc_InitSha384(sha384)) != 0) {
        WOLFSSL_MSG("InitSha384 failed");
    }
    else if ((ret = wc_Sha384Update(sha384, data, len)) != 0) {
        WOLFSSL_MSG("Sha384Update failed");
    }
    else if ((ret = wc_Sha384Final(sha384, hash)) != 0) {
        WOLFSSL_MSG("Sha384Final failed");
    }

#ifdef WOLFSSL_SMALL_STACK
    XFREE(sha384, NULL, DYNAMIC_TYPE_TMP_BUFFER);
#endif

    return ret;
}

#endif /* defined(WOLFSSL_SHA384) */
#endif /* defined(WOLFSSL_SHA512) */
