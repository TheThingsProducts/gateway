/*******************************************************************************
  Application Header

  File Name:
    config.h

  Summary:
 config file for CyaSSL to avoid custom build options

  Description:
 config file for CyaSSL to avoid custom build options
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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


#ifndef CONFIG_H
#define	CONFIG_H

#ifdef	__cplusplus
extern "C" {
#endif

#if defined(INLINE)
#undef INLINE
#define INLINE inline
#endif

#define MICROCHIP_MPLAB_HARMONY
#define MICROCHIP_TCPIP
#define WOLFSSL_HAVE_MIN
#define WOLFSSL_HAVE_MAX

#define MICROCHIP_PIC32_RNG
#define NEED_AES_TABLES
#define SIZEOF_LONG_LONG 8
#define WOLFSSL_USER_IO
#define NO_WRITEV
#define NO_DEV_RANDOM
#define NO_FILESYSTEM
#define WOLFSSL_STATIC_RSA


#define SINGLE_THREADED

#define USE_FAST_MATH
#define TFM_TIMING_RESISTANT
#define NO_WOLFSSL_SERVER


#define NO_OLD_TLS

#ifdef	__cplusplus
}
#endif

#endif	/* CONFIG_H */
