/**************************************************************************
  Crypto Framework Library Header

  Company:
    Microchip Technology Inc.

  File Name:
    md5.h
  
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

#ifndef CRYPT_MD5_H
#define CRYPT_MD5_H

#include "system_config.h"
#include "crypto/src/types.h"

#ifndef NO_MD5

#ifdef HAVE_FIPS
    #define wc_InitMd5   InitMd5
    #define wc_Md5Update Md5Update
    #define wc_Md5Final  Md5Final
    #define wc_Md5Hash   Md5Hash
#endif


#ifdef __cplusplus
    extern "C" {
#endif


/* in bytes */
enum {
#ifdef STM32F2_HASH
    MD5_REG_SIZE    =  4,      /* STM32 register size, bytes */
#endif
    MD5             =  0,      /* hash type unique */
    MD5_BLOCK_SIZE  = 64,
    MD5_DIGEST_SIZE = 16,
    MD5_PAD_SIZE    = 56
};

#if defined(WOLFSSL_PIC32MZ_HASH)
#include "pic32mz-crypt.h"
#endif

/* MD5 digest */
typedef struct Md5 {
    word32  buffLen;   /* in bytes          */
    word32  loLen;     /* length in bytes   */
    word32  hiLen;     /* length in bytes   */
    word32  buffer[MD5_BLOCK_SIZE  / sizeof(word32)];
    #if !defined(WOLFSSL_PIC32MZ_HASH)
    word32  digest[MD5_DIGEST_SIZE / sizeof(word32)];
    #else
    word32  digest[PIC32_HASH_SIZE / sizeof(word32)];
    pic32mz_desc desc ; /* Crypt Engine descripter */
    #endif
} Md5;

WOLFSSL_API void wc_InitMd5(Md5*);
WOLFSSL_API void wc_Md5Update(Md5*, const byte*, word32);
WOLFSSL_API void wc_Md5Final(Md5*, byte*);
WOLFSSL_API int  wc_Md5Hash(const byte*, word32, byte*);
#ifdef WOLFSSL_PIC32MZ_HASH
WOLFSSL_API void wc_Md5SizeSet(Md5*, word32);
#endif

#ifdef __cplusplus
    } /* extern "C" */
#endif

#endif /* NO_MD5 */
#endif /* CRYPT_MD5_H */
