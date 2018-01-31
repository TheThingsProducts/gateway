/**************************************************************************
  Crypto Framework Library Header

  Company:
    Microchip Technology Inc.

  File Name:
    settings.h
  
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



/* Place OS specific preprocessor flags, defines, includes here, will be
   included into every file because types.h includes it */


#ifndef CRYPT_SETTINGS_H
#define CRYPT_SETTINGS_H

#ifdef __cplusplus
    extern "C" {
#endif

/* Uncomment next line if using ThreadX */
/* #define THREADX */

/* Uncomment next line if using Micrium ucOS */
/* #define MICRIUM */

/* Uncomment next line if using Microchip PIC32 ethernet starter kit */
#define MICROCHIP_PIC32

/* Uncomment next line if using Microchip TCP/IP stack, version 5 */
/* #define MICROCHIP_TCPIP_V5 */

/* Uncomment next line if using Microchip TCP/IP stack, version 6 or later */
/* #define MICROCHIP_TCPIP */

/* Uncomment next line if using PIC32MZ Crypto Engine */
/* #define WOLFSSL_MICROCHIP_PIC32MZ */

/* Uncomment next line if using FreeRTOS */
/* #define FREERTOS */

/* Uncomment next line if using FreeRTOS Windows Simulator */
/* #define FREERTOS_WINSIM */

/* Uncomment next line if using RTIP */
/* #define EBSNET */

/* Uncomment next line if using lwip */
/* #define WOLFSSL_LWIP */

/* Uncomment next line if building wolfSSL for a game console */
/* #define WOLFSSL_GAME_BUILD */

/* Uncomment next line if building wolfSSL for LSR */
/* #define WOLFSSL_LSR */

/* Uncomment next line if using Comverge settings */
/* #define COMVERGE */

/* Uncomment next line if using QL SEP settings */
/* #define WOLFSSL_QL */

/* Uncomment next line if building for EROAD */
/* #define WOLFSSL_EROAD */

/* Uncomment next line if building with PicoTCP */
/* #define WOLFSSL_PICOTCP */

/* Uncomment next line if building for PicoTCP demo bundle */
/* #define WOLFSSL_PICOTCP_DEMO */

/* Uncomment next line if using Max Strength build */
/* #define WOLFSSL_MAX_STRENGTH */

/* Uncomment next line if building for VxWorks */
/* #define WOLFSSL_VXWORKS */

/* Uncomment next line to enable deprecated less secure static DH suites */
/* #define WOLFSSL_STATIC_DH */

/* Uncomment next line to enable deprecated less secure static RSA suites */
/* #define WOLFSSL_STATIC_RSA */

/* Uncomment next line if building for ARDUINO */
/* #define WOLFSSL_ARDUINO */

#include "crypto/src/visibility.h"

#ifdef WOLFSSL_USER_SETTINGS
    #include "user_settings.h"
#endif


/* make sure old RNG name is used with CTaoCrypt FIPS */
#ifdef HAVE_FIPS
    #define WC_RNG RNG
#endif


#ifdef COMVERGE
    #define THREADX
    #define HAVE_NETX
    #define WOLFSSL_USER_IO
    #define NO_WRITEV
    #define NO_DEV_RANDOM
    #define NO_FILESYSTEM
    #define NO_SHA512
    #define NO_DH
    /* Allows use of DH with fixed points if uncommented and NO_DH is removed */
    /* WOLFSSL_DH_CONST */
    #define NO_DSA
    #define NO_HC128
    #define NO_RSA
    #define NO_SESSION_CACHE
    #define HAVE_ECC
#endif


#ifdef THREADX
    #define SIZEOF_LONG_LONG 8
#endif

#ifdef HAVE_NETX
    #include "nx_api.h"
#endif

#if defined(HAVE_LWIP_NATIVE) /* using LwIP native TCP socket */
    #define WOLFSSL_LWIP
    #define NO_WRITEV
    #define SINGLE_THREADED
    #define WOLFSSL_USER_IO
    #define NO_FILESYSTEM
#endif

#ifdef MICROCHIP_PIC32
    /* #define WOLFSSL_MICROCHIP_PIC32MZ */
    #define SIZEOF_LONG_LONG 8
    #define SINGLE_THREADED
    #define WOLFSSL_USER_IO
    #define NO_WRITEV
    #define NO_DEV_RANDOM
    #define NO_FILESYSTEM
//    #define USE_FAST_MATH
    #define TFM_TIMING_RESISTANT
    #define NEED_AES_TABLES
    #define WOLFSSL_HAVE_MIN
#endif

#ifdef WOLFSSL_MICROCHIP_PIC32MZ
    #define WOLFSSL_PIC32MZ_CE
    #define WOLFSSL_PIC32MZ_CRYPT
    #define HAVE_AES_ENGINE
    #define WOLFSSL_PIC32MZ_RNG
    /* #define WOLFSSL_PIC32MZ_HASH */
    #define WOLFSSL_AES_COUNTER
    #define HAVE_AESGCM
    #define NO_BIG_INT

#endif

#ifdef MICROCHIP_TCPIP_V5
    /* include timer functions */
    #include "TCPIP Stack/TCPIP.h"
#endif

#ifdef MICROCHIP_TCPIP
    /* include timer, NTP functions */
    #ifdef MICROCHIP_MPLAB_HARMONY
        #include "tcpip/tcpip.h"
    #else
        #include "system/system_services.h"
        #include "tcpip/sntp.h"
    #endif
#endif



#ifdef WOLFSSL_ARDUINO
    #define NO_WRITEV
    #define NO_WOLFSSL_DIR
    #define SINGLE_THREADED
    #define NO_DEV_RANDOM
    #ifndef INTEL_GALILEO /* Galileo has time.h compatibility */
        #define TIME_OVERRIDES /* must define XTIME and XGMTIME externally */
    #endif
    #define WOLFSSL_USER_IO
    #define HAVE_ECC
    #define NO_DH
    #define NO_SESSION_CACHE
    #define USE_SLOW_SHA
    #define NO_WOLFSSL_SERVER
    #define NO_ERROR_STRINGS
#endif

#if defined(WOLFSSL_LEANPSK) && !defined(XMALLOC_USER)
    #include <stdlib.h>
    #define XMALLOC(s, h, type)  malloc((s))
    #define XFREE(p, h, type)    free((p))
    #define XREALLOC(p, n, h, t) realloc((p), (n))
#endif

#if defined(XMALLOC_USER) && defined(SSN_BUILDING_LIBYASSL)
    #undef  XMALLOC
    #define XMALLOC     yaXMALLOC
    #undef  XFREE
    #define XFREE       yaXFREE
    #undef  XREALLOC
    #define XREALLOC    yaXREALLOC
#endif


#ifdef FREERTOS
    #include "FreeRTOS.h"

    /* FreeRTOS pvPortRealloc() only in AVR32_UC3 port */
    #if !defined(XMALLOC_USER) && !defined(NO_WOLFSSL_MEMORY)
        #define XMALLOC(s, h, type)  pvPortMalloc((s))
        #define XFREE(p, h, type)    vPortFree((p))
    #endif

    #ifndef NO_WRITEV
        #define NO_WRITEV
    #endif
    #ifndef HAVE_SHA512
        #ifndef NO_SHA512
            #define NO_SHA512
        #endif
    #endif
    #ifndef HAVE_DH
        #ifndef NO_DH
            #define NO_DH
        #endif
    #endif
    #ifndef NO_DSA
        #define NO_DSA
    #endif
    #ifndef NO_HC128
        #define NO_HC128
    #endif

    #ifndef SINGLE_THREADED
        #include "semphr.h"
    #endif
#endif

#ifdef FREERTOS_TCP

#if !defined(NO_WOLFSSL_MEMORY) && !defined(XMALLOC_USER)
#define XMALLOC(s, h, type)  pvPortMalloc((s))
#define XFREE(p, h, type)    vPortFree((p))
#endif

#define WOLFSSL_GENSEED_FORTEST

#define NO_WOLFSSL_DIR
#define NO_WRITEV
#define WOLFSSL_HAVE_MIN
#define USE_FAST_MATH
#define TFM_TIMING_REGISTANT
#define NO_MAIN_DRIVER

#endif

#ifdef EBSNET
    #include "rtip.h"

    /* #define DEBUG_WOLFSSL */
    #define NO_WOLFSSL_DIR  /* tbd */

    #if (POLLOS)
        #define SINGLE_THREADED
    #endif

    #if (RTPLATFORM)
        #if (!RTP_LITTLE_ENDIAN)
            #define BIG_ENDIAN_ORDER
        #endif
    #else
        #if (!KS_LITTLE_ENDIAN)
            #define BIG_ENDIAN_ORDER
        #endif
    #endif

    #if (WINMSP3)
        #undef SIZEOF_LONG
        #define SIZEOF_LONG_LONG 8
    #else
        #sslpro: settings.h - please implement SIZEOF_LONG and SIZEOF_LONG_LONG
    #endif

    #define XMALLOC(s, h, type) ((void *)rtp_malloc((s), SSL_PRO_MALLOC))
    #define XFREE(p, h, type) (rtp_free(p))
    #define XREALLOC(p, n, h, t) realloc((p), (n))

#endif /* EBSNET */

#ifdef WOLFSSL_LSR
    #define HAVE_WEBSERVER
    #define SIZEOF_LONG_LONG 8
    #define WOLFSSL_LOW_MEMORY
    #define NO_WRITEV
    #define NO_SHA512
    #define NO_DH
    /* Allows use of DH with fixed points if uncommented and NO_DH is removed */
    /* WOLFSSL_DH_CONST */
    #define NO_DSA
    #define NO_HC128
    #define NO_DEV_RANDOM
    #define NO_WOLFSSL_DIR
    #define NO_RABBIT
    #ifndef NO_FILESYSTEM
        #define LSR_FS
        #include "inc/hw_types.h"
        #include "fs.h"
    #endif
    #define WOLFSSL_LWIP
    #include <errno.h>  /* for tcp errno */
    #define WOLFSSL_SAFERTOS
    #if defined(__IAR_SYSTEMS_ICC__)
        /* enum uses enum */
        #pragma diag_suppress=Pa089
    #endif
#endif

#ifdef WOLFSSL_SAFERTOS
    #ifndef SINGLE_THREADED
        #include "SafeRTOS/semphr.h"
    #endif

    #include "SafeRTOS/heap.h"
    #define XMALLOC(s, h, type)  pvPortMalloc((s))
    #define XFREE(p, h, type)    vPortFree((p))
    #define XREALLOC(p, n, h, t) pvPortRealloc((p), (n))
#endif

#ifdef WOLFSSL_LOW_MEMORY
    #undef  RSA_LOW_MEM
    #define RSA_LOW_MEM
    #undef  WOLFSSL_SMALL_STACK
    #define WOLFSSL_SMALL_STACK
    #undef  TFM_TIMING_RESISTANT
    #define TFM_TIMING_RESISTANT
#endif

#ifdef MICRIUM

    #include "stdlib.h"
    #include "net_cfg.h"
    #include "ssl_cfg.h"
    #include "net_secure_os.h"

    #define WOLFSSL_TYPES

    typedef CPU_INT08U byte;
    typedef CPU_INT16U word16;
    typedef CPU_INT32U word32;

    #if (NET_SECURE_MGR_CFG_WORD_SIZE == CPU_WORD_SIZE_32)
        #define SIZEOF_LONG        4
        #undef  SIZEOF_LONG_LONG
    #else
        #undef  SIZEOF_LONG
        #define SIZEOF_LONG_LONG   8
    #endif

    #define STRING_USER

    #define XSTRLEN(pstr) ((CPU_SIZE_T)Str_Len((CPU_CHAR *)(pstr)))
    #define XSTRNCPY(pstr_dest, pstr_src, len_max) \
                    ((CPU_CHAR *)Str_Copy_N((CPU_CHAR *)(pstr_dest), \
                     (CPU_CHAR *)(pstr_src), (CPU_SIZE_T)(len_max)))
    #define XSTRNCMP(pstr_1, pstr_2, len_max) \
                    ((CPU_INT16S)Str_Cmp_N((CPU_CHAR *)(pstr_1), \
                     (CPU_CHAR *)(pstr_2), (CPU_SIZE_T)(len_max)))
    #define XSTRSTR(pstr, pstr_srch) \
                    ((CPU_CHAR *)Str_Str((CPU_CHAR *)(pstr), \
                     (CPU_CHAR *)(pstr_srch)))
    #define XMEMSET(pmem, data_val, size) \
                    ((void)Mem_Set((void *)(pmem), (CPU_INT08U) (data_val), \
                    (CPU_SIZE_T)(size)))
    #define XMEMCPY(pdest, psrc, size) ((void)Mem_Copy((void *)(pdest), \
                     (void *)(psrc), (CPU_SIZE_T)(size)))
    #define XMEMCMP(pmem_1, pmem_2, size) \
                   (((CPU_BOOLEAN)Mem_Cmp((void *)(pmem_1), (void *)(pmem_2), \
                     (CPU_SIZE_T)(size))) ? DEF_NO : DEF_YES)
    #define XMEMMOVE XMEMCPY

#if (NET_SECURE_MGR_CFG_EN == DEF_ENABLED)
    #define MICRIUM_MALLOC
    #define XMALLOC(s, h, type) ((void *)NetSecure_BlkGet((CPU_INT08U)(type), \
                                 (CPU_SIZE_T)(s), (void *)0))
    #define XFREE(p, h, type)   (NetSecure_BlkFree((CPU_INT08U)(type), \
                                 (p), (void *)0))
    #define XREALLOC(p, n, h, t) realloc((p), (n))
#endif

    #if (NET_SECURE_MGR_CFG_FS_EN == DEF_ENABLED)
        #undef  NO_FILESYSTEM
    #else
        #define NO_FILESYSTEM
    #endif

    #if (SSL_CFG_TRACE_LEVEL == WOLFSSL_TRACE_LEVEL_DBG)
        #define DEBUG_WOLFSSL
    #else
        #undef  DEBUG_WOLFSSL
    #endif

    #if (SSL_CFG_OPENSSL_EN == DEF_ENABLED)
        #define OPENSSL_EXTRA
    #else
        #undef  OPENSSL_EXTRA
    #endif

    #if (SSL_CFG_MULTI_THREAD_EN == DEF_ENABLED)
        #undef  SINGLE_THREADED
    #else
        #define SINGLE_THREADED
    #endif

    #if (SSL_CFG_DH_EN == DEF_ENABLED)
        #undef  NO_DH
    #else
        #define NO_DH
    #endif

    #if (SSL_CFG_DSA_EN == DEF_ENABLED)
        #undef  NO_DSA
    #else
        #define NO_DSA
    #endif

    #if (SSL_CFG_PSK_EN == DEF_ENABLED)
        #undef  NO_PSK
    #else
        #define NO_PSK
    #endif

    #if (SSL_CFG_3DES_EN == DEF_ENABLED)
        #undef  NO_DES
    #else
        #define NO_DES
    #endif

    #if (SSL_CFG_AES_EN == DEF_ENABLED)
        #undef  NO_AES
    #else
        #define NO_AES
    #endif

    #if (SSL_CFG_RC4_EN == DEF_ENABLED)
        #undef  NO_RC4
    #else
        #define NO_RC4
    #endif

    #if (SSL_CFG_RABBIT_EN == DEF_ENABLED)
        #undef  NO_RABBIT
    #else
        #define NO_RABBIT
    #endif

    #if (SSL_CFG_HC128_EN == DEF_ENABLED)
        #undef  NO_HC128
    #else
        #define NO_HC128
    #endif

    #if (CPU_CFG_ENDIAN_TYPE == CPU_ENDIAN_TYPE_BIG)
        #define BIG_ENDIAN_ORDER
    #else
        #undef  BIG_ENDIAN_ORDER
        #define LITTLE_ENDIAN_ORDER
    #endif

    #if (SSL_CFG_MD4_EN == DEF_ENABLED)
        #undef  NO_MD4
    #else
        #define NO_MD4
    #endif

    #if (SSL_CFG_WRITEV_EN == DEF_ENABLED)
        #undef  NO_WRITEV
    #else
        #define NO_WRITEV
    #endif

    #if (SSL_CFG_USER_RNG_SEED_EN == DEF_ENABLED)
        #define NO_DEV_RANDOM
    #else
        #undef  NO_DEV_RANDOM
    #endif

    #if (SSL_CFG_USER_IO_EN == DEF_ENABLED)
        #define WOLFSSL_USER_IO
    #else
        #undef  WOLFSSL_USER_IO
    #endif

    #if (SSL_CFG_DYNAMIC_BUFFERS_EN == DEF_ENABLED)
        #undef  LARGE_STATIC_BUFFERS
        #undef  STATIC_CHUNKS_ONLY
    #else
        #define LARGE_STATIC_BUFFERS
        #define STATIC_CHUNKS_ONLY
    #endif

    #if (SSL_CFG_DER_LOAD_EN == DEF_ENABLED)
        #define  WOLFSSL_DER_LOAD
    #else
        #undef   WOLFSSL_DER_LOAD
    #endif

    #if (SSL_CFG_DTLS_EN == DEF_ENABLED)
        #define  WOLFSSL_DTLS
    #else
        #undef   WOLFSSL_DTLS
    #endif

    #if (SSL_CFG_CALLBACKS_EN == DEF_ENABLED)
         #define WOLFSSL_CALLBACKS
    #else
         #undef  WOLFSSL_CALLBACKS
    #endif

    #if (SSL_CFG_FAST_MATH_EN == DEF_ENABLED)
         #define USE_FAST_MATH
    #else
         #undef  USE_FAST_MATH
    #endif

    #if (SSL_CFG_TFM_TIMING_RESISTANT_EN == DEF_ENABLED)
         #define TFM_TIMING_RESISTANT
    #else
         #undef  TFM_TIMING_RESISTANT
    #endif

#endif /* MICRIUM */


#ifdef WOLFSSL_QL
    #ifndef WOLFSSL_SEP
        #define WOLFSSL_SEP
    #endif
    #ifndef OPENSSL_EXTRA
        #define OPENSSL_EXTRA
    #endif
    #ifndef SESSION_CERTS
        #define SESSION_CERTS
    #endif
    #ifndef HAVE_AESCCM
        #define HAVE_AESCCM
    #endif
    #ifndef ATOMIC_USER
        #define ATOMIC_USER
    #endif
    #ifndef WOLFSSL_DER_LOAD
        #define WOLFSSL_DER_LOAD
    #endif
    #ifndef KEEP_PEER_CERT
        #define KEEP_PEER_CERT
    #endif
    #ifndef HAVE_ECC
        #define HAVE_ECC
    #endif
    #ifndef SESSION_INDEX
        #define SESSION_INDEX
    #endif
#endif /* WOLFSSL_QL */


#if !defined(XMALLOC_USER) && !defined(MICRIUM_MALLOC) && \
    !defined(WOLFSSL_LEANPSK) && !defined(NO_WOLFSSL_MEMORY)
    #define USE_WOLFSSL_MEMORY
#endif


#if defined(OPENSSL_EXTRA) && !defined(NO_CERTS)
    #undef  KEEP_PEER_CERT
    #define KEEP_PEER_CERT
#endif


/* stream ciphers except arc4 need 32bit alignment, intel ok without */
#ifndef XSTREAM_ALIGN
    #define XSTREAM_ALIGNMENT
#endif


/* if using hardware crypto and have alignment requirements, specify the
   requirement here.  The record header of SSL/TLS will prevent easy alignment.
   This hint tries to help as much as possible.  */
#ifndef WOLFSSL_GENERAL_ALIGNMENT
    #ifdef WOLFSSL_AESNI
        #define WOLFSSL_GENERAL_ALIGNMENT 16
    #elif defined(XSTREAM_ALIGN)
        #define WOLFSSL_GENERAL_ALIGNMENT  4
    #elif defined(FREESCALE_MMCAU)
        #define WOLFSSL_GENERAL_ALIGNMENT  WOLFSSL_MMCAU_ALIGNMENT
    #else
        #define WOLFSSL_GENERAL_ALIGNMENT  0
    #endif
#endif

#if defined(WOLFSSL_GENERAL_ALIGNMENT) && (WOLFSSL_GENERAL_ALIGNMENT > 0)
    #if defined(_MSC_VER)
        #define XGEN_ALIGN __declspec(align(WOLFSSL_GENERAL_ALIGNMENT))
    #elif defined(__GNUC__)
        #define XGEN_ALIGN __attribute__((aligned(WOLFSSL_GENERAL_ALIGNMENT)))
    #else
        #define XGEN_ALIGN
    #endif
#else
    #define XGEN_ALIGN
#endif

#ifdef HAVE_CRL
    /* not widely supported yet */
    #undef NO_SKID
    #define NO_SKID
#endif


/* user can specify what curves they want with ECC_USER_CURVES otherwise
 * all curves are on by default for now */
#ifndef ECC_USER_CURVES
    #ifndef HAVE_ALL_CURVES
        #define HAVE_ALL_CURVES
    #endif
#endif

/* ECC Configs */
#ifdef HAVE_ECC
    /* By default enable Sign, Verify, DHE, Key Import and Key Export unless explicitly disabled */
    #ifndef NO_ECC_SIGN
        #undef HAVE_ECC_SIGN
        #define HAVE_ECC_SIGN
    #endif
    #ifndef NO_ECC_VERIFY
        #undef HAVE_ECC_VERIFY
        #define HAVE_ECC_VERIFY
    #endif
    #ifndef NO_ECC_DHE
        #undef HAVE_ECC_DHE
        #define HAVE_ECC_DHE
    #endif
    #ifndef NO_ECC_KEY_IMPORT
        #undef HAVE_ECC_KEY_IMPORT
        #define HAVE_ECC_KEY_IMPORT
    #endif
    #ifndef NO_ECC_KEY_EXPORT
        #undef HAVE_ECC_KEY_EXPORT
        #define HAVE_ECC_KEY_EXPORT
    #endif
#endif /* HAVE_ECC */

/* Curve255519 Configs */
#ifdef HAVE_CURVE25519
    /* By default enable shared secret, key export and import */
    #ifndef NO_CURVE25519_SHARED_SECRET
        #undef HAVE_CURVE25519_SHARED_SECRET
        #define HAVE_CURVE25519_SHARED_SECRET
    #endif
    #ifndef NO_CURVE25519_KEY_EXPORT
        #undef HAVE_CURVE25519_KEY_EXPORT
        #define HAVE_CURVE25519_KEY_EXPORT
    #endif
    #ifndef NO_CURVE25519_KEY_IMPORT
        #undef HAVE_CURVE25519_KEY_IMPORT
        #define HAVE_CURVE25519_KEY_IMPORT
    #endif
#endif /* HAVE_CURVE25519 */

/* Ed255519 Configs */
#ifdef HAVE_ED25519
    /* By default enable sign, verify, key export and import */
    #ifndef NO_ED25519_SIGN
        #undef HAVE_ED25519_SIGN
        #define HAVE_ED25519_SIGN
    #endif
    #ifndef NO_ED25519_VERIFY
        #undef HAVE_ED25519_VERIFY
        #define HAVE_ED25519_VERIFY
    #endif
    #ifndef NO_ED25519_KEY_EXPORT
        #undef HAVE_ED25519_KEY_EXPORT
        #define HAVE_ED25519_KEY_EXPORT
    #endif
    #ifndef NO_ED25519_KEY_IMPORT
        #undef HAVE_ED25519_KEY_IMPORT
        #define HAVE_ED25519_KEY_IMPORT
    #endif
#endif /* HAVE_ED25519 */

/* AES Config */
#ifndef NO_AES
    /* By default enable all AES key sizes, decryption and CBC */
    #ifndef AES_MAX_KEY_SIZE
        #undef  AES_MAX_KEY_SIZE
        #define AES_MAX_KEY_SIZE    256
    #endif
    #ifndef NO_AES_DECRYPT
        #undef  HAVE_AES_DECRYPT
        #define HAVE_AES_DECRYPT
    #endif
    #ifndef NO_AES_CBC
        #undef  HAVE_AES_CBC
        #define HAVE_AES_CBC
    #else
        #ifndef WOLFCRYPT_ONLY
            #error "AES CBC is required for TLS and can only be disabled for WOLFCRYPT_ONLY builds"
        #endif
    #endif
#endif

/* If using the max strength build, ensure OLD TLS is disabled. */
#ifdef WOLFSSL_MAX_STRENGTH
    #undef NO_OLD_TLS
    #define NO_OLD_TLS
#endif

/* If not forcing ARC4 as the DRBG or using custom RNG block gen, enable Hash_DRBG */
#undef HAVE_HASHDRBG
#if !defined(WOLFSSL_FORCE_RC4_DRBG) && !defined(CUSTOM_RAND_GENERATE_BLOCK)
    #define HAVE_HASHDRBG
#endif


/* sniffer requires:
 * static RSA cipher suites
 * session stats and peak stats
 */
#ifdef WOLFSSL_SNIFFER
    #ifndef WOLFSSL_STATIC_RSA
        #define WOLFSSL_STATIC_RSA
    #endif
    #ifndef WOLFSSL_SESSION_STATS
        #define WOLFSSL_SESSION_STATS
    #endif
    #ifndef WOLFSSL_PEAK_SESSIONS
        #define WOLFSSL_PEAK_SESSIONS
    #endif
#endif

/* Decode Public Key extras on by default, user can turn off with
 * WOLFSSL_NO_DECODE_EXTRA */
#ifndef WOLFSSL_NO_DECODE_EXTRA
    #ifndef RSA_DECODE_EXTRA
        #define RSA_DECODE_EXTRA
    #endif
    #ifndef ECC_DECODE_EXTRA
        #define ECC_DECODE_EXTRA
    #endif
#endif

/* Place any other flags or defines here */


#ifdef __cplusplus
    }   /* extern "C" */
#endif

#endif
