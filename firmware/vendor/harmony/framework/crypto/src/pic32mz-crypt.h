/**************************************************************************
  Crypto Framework Library Header

  Company:
    Microchip Technology Inc.

  File Name:
    pic32mz-crypt.h
  
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

#ifndef PIC32MZ_CRYPT_H
#define PIC32MZ_CRYPT_H

#ifdef __cplusplus
    extern "C" {
#endif

#ifdef  WOLFSSL_MICROCHIP_PIC32MZ

#define MICROCHIP_PIC32
#include <xc.h>
#include <sys/endian.h>
#include <sys/kmem.h>
#include "crypto/crypto.h"

typedef struct saCtrl {
    unsigned int CRYPTOALGO : 4;
    unsigned int MULTITASK : 3;
    unsigned int KEYSIZE : 2;
    unsigned int ENCTYPE : 1;
    unsigned int ALGO : 7;
    unsigned int : 3;
    unsigned int FLAGS : 1;
    unsigned int FB : 1;
    unsigned int LOADIV : 1;
    unsigned int LNC : 1;
    unsigned int IRFLAG : 1;
    unsigned int ICVONLY : 1;
    unsigned int OR_EN : 1;
    unsigned int NO_RX : 1;
    unsigned int : 1;
    unsigned int VERIFY : 1;
    unsigned int : 2;
} saCtrl;

typedef struct securityAssociation {
    saCtrl SA_CTRL;
    unsigned int SA_AUTHKEY[8];
    unsigned int SA_ENCKEY[8];
    unsigned int SA_AUTHIV[8];
    unsigned int SA_ENCIV[4];
} securityAssociation;

typedef struct bdCtrl {
    unsigned int BUFLEN : 16;
    unsigned int CBD_INT_EN : 1;
    unsigned int PKT_INT_EN : 1;
    unsigned int LIFM : 1;
    unsigned int LAST_BD: 1;
    unsigned int : 2;
    unsigned int SA_FETCH_EN : 1;
    unsigned int : 8;
    volatile unsigned int DESC_EN : 1;
} bdCtrl;

typedef struct bufferDescriptor {
    bdCtrl BD_CTRL;
    unsigned int SA_ADDR;
    unsigned int SRCADDR;
    unsigned int DSTADDR;
    unsigned int NXTPTR;
    unsigned int UPDPTR;
    unsigned int MSGLEN;
    unsigned int ENCOFF;
} bufferDescriptor;

#define PIC32_ENCRYPTION      0b1
#define PIC32_DECRYPTION      0b0

#define PIC32_ALGO_HMAC1     0b01000000
#define PIC32_ALGO_SHA256    0b00100000
#define PIC32_ALGO_SHA1      0b00010000
#define PIC32_ALGO_MD5       0b00001000
#define PIC32_ALGO_AES       0b00000100
#define PIC32_ALGO_TDES      0b00000010
#define PIC32_ALGO_DES       0b00000001

#define PIC32_CRYPTOALGO_AES_GCM 0b1110
#define PIC32_CRYPTOALGO_RCTR    0b1101
#define PIC32_CRYPTOALGO_RCBC    0b1001
#define PIC32_CRYPTOALGO_REBC    0b1000
#define PIC32_CRYPTOALGO_TCBC    0b0101
#define PIC32_CRYPTOALGO_CBC     0b0001

#define PIC32_AES_KEYSIZE_256     0b10
#define PIC32_AES_KEYSIZE_192     0b01
#define PIC32_AES_KEYSIZE_128     0b00

#define PIC32_AES_BLOCK_SIZE 16
#define MD5_HASH_SIZE 16
#define SHA1_HASH_SIZE 20
#define SHA256_HASH_SIZE 32
#define PIC32_HASH_SIZE 32

#ifndef PIC32_BLOCK_SIZE
#define PIC32_BLOCK_SIZE 2048
#endif

#define PIC32MZ_MIN_BLOCK  64
//#define PIC32MZ_MAX_BLOCK (64*1024-4)
#define PIC32MZ_MAX_BLOCK (32*1024)

#ifndef PIC32MZ_MAX_BD
#define PIC32MZ_MAX_BD   2
#endif

typedef struct {      /* Crypt Engine descripter */
    int currBd ;
    int err   ;
    unsigned int msgSize;
    uint32_t processed;
    volatile bufferDescriptor 
        bd[PIC32MZ_MAX_BD] __attribute__((aligned (8)));
    securityAssociation 
        sa                 __attribute__((aligned (8)));
    uint32_t dbPtr;
} pic32mz_desc ;

#define PIC32MZ_IF_RAM(addr) (KVA_TO_PA(addr) < 0x1D000000)

#define WAIT_ENGINE \
    { while (CEINTSRCbits.PKTIF == 0); CEINTSRC = 0xF; }

#ifdef DEBUG_CYASSL
static void print_mem(const unsigned char *p, int size) {
    for(; size>0; size--, p++) {
        if(size%4 == 0)printf(" ") ;
            printf("%02x", (int)*p) ;
    }
    puts("") ;
}
#endif

#endif

#ifdef __cplusplus
    } /* extern "C" */
#endif

#endif /* PIC32MZ_CRYPT_H */
