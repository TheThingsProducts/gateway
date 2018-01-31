/**************************************************************************
  Crypto Framework Library Header

  Company:
    Microchip Technology Inc.

  File Name:
    random.h
  
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

#ifndef CRYPT_RANDOM_H
#define CRYPT_RANDOM_H

#include "crypto/src/types.h"

#ifdef HAVE_FIPS
/* for fips @wc_fips */
#include "crypto/src/random.h"
#endif

#ifdef __cplusplus
    extern "C" {
#endif

#ifndef HAVE_FIPS /* avoid redefining structs and macros */
#if defined(WOLFSSL_FORCE_RC4_DRBG) && defined(NO_RC4)
    #error Cannot have WOLFSSL_FORCE_RC4_DRBG and NO_RC4 defined.
#endif /* WOLFSSL_FORCE_RC4_DRBG && NO_RC4 */
#if defined(HAVE_HASHDRBG) || defined(NO_RC4)
    #ifdef NO_SHA256
        #error "Hash DRBG requires SHA-256."
    #endif /* NO_SHA256 */

    #include "crypto/src/sha256.h"
#else /* HAVE_HASHDRBG || NO_RC4 */
    #include "crypto/src/arc4.h"
#endif /* HAVE_HASHDRBG || NO_RC4 */

/* OS specific seeder */
typedef struct OS_Seed {
    int fd;
} OS_Seed;

#if defined(HAVE_HASHDRBG) || defined(NO_RC4)


#define DRBG_SEED_LEN (440/8)


struct DRBG; /* Private DRBG state */


/* Hash-based Deterministic Random Bit Generator */
typedef struct WC_RNG {
    struct DRBG* drbg;
    OS_Seed seed;
    byte status;
} WC_RNG;


#else /* HAVE_HASHDRBG || NO_RC4 */


#define WOLFSSL_RNG_CAVIUM_MAGIC 0xBEEF0004

/* secure Random Number Generator */


typedef struct WC_RNG {
    OS_Seed seed;
    Arc4    cipher;
#ifdef HAVE_CAVIUM
    int    devId;           /* nitrox device id */
    word32 magic;           /* using cavium magic */
#endif
} WC_RNG;


#endif /* HAVE_HASH_DRBG || NO_RC4 */

#endif /* HAVE_FIPS */

/* NO_OLD_RNGNAME removes RNG struct name to prevent possible type conflicts,
 * can't be used with CTaoCrypt FIPS */
#if !defined(NO_OLD_RNGNAME) && !defined(HAVE_FIPS)
    #define RNG WC_RNG
#endif

WOLFSSL_LOCAL
int wc_GenerateSeed(OS_Seed* os, byte* seed, word32 sz);

#if defined(HAVE_HASHDRBG) || defined(NO_RC4)

#ifdef HAVE_CAVIUM
    WOLFSSL_API int  wc_InitRngCavium(WC_RNG*, int);
#endif

#endif /* HAVE_HASH_DRBG || NO_RC4 */


WOLFSSL_API int  wc_InitRng(WC_RNG*);
WOLFSSL_API int  wc_RNG_GenerateBlock(WC_RNG*, byte*, word32 sz);
WOLFSSL_API int  wc_RNG_GenerateByte(WC_RNG*, byte*);
WOLFSSL_API int  wc_FreeRng(WC_RNG*);


#if defined(HAVE_HASHDRBG) || defined(NO_RC4)
    WOLFSSL_API int wc_RNG_HealthTest(int reseed,
                                        const byte* entropyA, word32 entropyASz,
                                        const byte* entropyB, word32 entropyBSz,
                                        byte* output, word32 outputSz);
#endif /* HAVE_HASHDRBG || NO_RC4 */

#ifdef __cplusplus
    } /* extern "C" */
#endif

#endif /* CRYPT_RANDOM_H */

