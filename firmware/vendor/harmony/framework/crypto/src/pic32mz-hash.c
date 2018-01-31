/**************************************************************************
  Crypto Framework Library Source

  Company:
    Microchip Technology Inc.

  File Name:
    pic32mz-hash.c
  
  Summary:
    Crypto Framework Libarary source for cryptographic functions.

  Description:
    This source file contains functions that make up the Cryptographic 
    Framework Library for PIC32 families of Microchip microcontrollers.
**************************************************************************/

//DOM-IGNORE-BEGIN
/******************************************************************************
File Name:  pic32mz-hash.c
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

#ifdef HAVE_CONFIG_H
    #include "config.h"
#endif
#include "system_config.h"

#include "crypto/src/settings.h"
#include "system/devcon/sys_devcon.h"

#ifdef WOLFSSL_PIC32MZ_HASH

#include "crypto/src/logging.h"
#include "crypto/src/md5.h"
#include "crypto/src/sha.h"
#include "crypto/src/sha256.h"

#include "crypto/src/pic32mz-crypt.h"

#if PIC32_BLOCK_SIZE < PIC32MZ_MIN_BLOCK
#error Encryption block size must be at least 64 bytes.
#endif

#if !defined(NO_MD5) && !defined(NO_SHA) && !defined(NO_SHA256)

static uint8_t dataBuffer[PIC32MZ_MAX_BD][PIC32_BLOCK_SIZE] __attribute__((aligned (4), coherent));

static void reset_engine(pic32mz_desc *desc, int algo)
{
    int i;
    pic32mz_desc* uc_desc = KVA0_TO_KVA1(desc);

    CECON = 1 << 6;
    while (CECON);
    CEINTSRC = 0xF;     // Clear the interrupt flags

    /* Make sure everything is clear first before we make settings. */
    XMEMSET((void *)&uc_desc->sa, 0, sizeof(uc_desc->sa));

    /* Set up the security association */
    uc_desc->sa.SA_CTRL.ALGO = algo ;
    uc_desc->sa.SA_CTRL.LNC = 1;
    uc_desc->sa.SA_CTRL.FB = 1;
    uc_desc->sa.SA_CTRL.ENCTYPE = 1;
    uc_desc->sa.SA_CTRL.LOADIV = 1;

    /* Set up the buffer descriptor */
    uc_desc->err = 0 ;
    for (i = 0; i < PIC32MZ_MAX_BD; i++)
    {
        XMEMSET((void *)&uc_desc->bd[i], 0, sizeof(uc_desc->bd[i]));
        uc_desc->bd[i].BD_CTRL.LAST_BD = 1;
        uc_desc->bd[i].BD_CTRL.LIFM = 1;
        uc_desc->bd[i].BD_CTRL.PKT_INT_EN = 1;
        uc_desc->bd[i].SA_ADDR = KVA_TO_PA(&uc_desc->sa);
        uc_desc->bd[i].SRCADDR = KVA_TO_PA(&dataBuffer[i]);
        if (PIC32MZ_MAX_BD > i+1)
            uc_desc->bd[i].NXTPTR = KVA_TO_PA(&uc_desc->bd[i+1]);
        else
            uc_desc->bd[i].NXTPTR = KVA_TO_PA(&uc_desc->bd[0]);
        XMEMSET((void *)&dataBuffer[i], 0, PIC32_BLOCK_SIZE);
    }
    uc_desc->bd[0].BD_CTRL.SA_FETCH_EN = 1; // Fetch the security association on the first BD
    desc->dbPtr = 0;
    desc->currBd = 0;
    desc->msgSize = 0;
    desc->processed = 0;
    CEBDPADDR = KVA_TO_PA(&(desc->bd[0]));

    CEPOLLCON = 10;

#if ((__PIC32_FEATURE_SET0 == 'E') && (__PIC32_FEATURE_SET1 == 'C'))   // No output swap
    CECON = 0x27;
#else
    CECON = 0xa7;
#endif
}

#define PIC32MZ_IF_RAM(addr) (KVA_TO_PA(addr) < 0x1D000000)

static void update_engine(pic32mz_desc *desc, const byte *input, word32 len,
                    word32 *hash)
{
    int total ;
    pic32mz_desc    *uc_desc = KVA0_TO_KVA1(desc);

    #ifdef DEBUG_CYASSL
    printf("Input[bd=%d, len=%d]:%x->\n", desc->currBd, len, input) ;
    print_mem(input, 4) ;
    #endif
    uc_desc->bd[desc->currBd].UPDPTR = KVA_TO_PA(hash);
    // Add the data to the current buffer. If the buffer fills, start processing it
    // and fill the next one.
    while (len)
    {
        if (desc->msgSize)
        {
            // If we've been given the message size, we can process along the way.
			// We might have buffered something previously. Fill as needed and process.
			if (desc->dbPtr)
			{
				// Copy enough data to fill the buffer, as possible.
                total = (PIC32_BLOCK_SIZE - desc->dbPtr);
				if (total > len) total = len;
                XMEMCPY(&dataBuffer[desc->currBd][desc->dbPtr], input, total);
				uc_desc->bd[desc->currBd].SRCADDR = KVA_TO_PA(&dataBuffer[desc->currBd]);
			}
			else
			{
                // Make sure we are a multiple of 4 before going straight to 
                // the engine
                if ((len >= PIC32_BLOCK_SIZE) || ((len % 4) == 0))
				{
					// point the current buffer descriptor to the input data and set the size.
					uc_desc->bd[desc->currBd].SRCADDR = KVA_TO_PA(input);
					total = (len > PIC32MZ_MAX_BLOCK ? PIC32MZ_MAX_BLOCK : len);
				}
				else	// Otherwise, we have to buffer it
				{
					XMEMCPY(&dataBuffer[desc->currBd][desc->dbPtr], input, len);
					total = len;
				}
			}
			desc->dbPtr += total;
			len -= total;
			input += total;
			desc->processed += total;
			// Fill in the details in the buffer descriptor
            uc_desc->bd[desc->currBd].MSGLEN = desc->msgSize;
            uc_desc->bd[desc->currBd].UPDPTR = KVA_TO_PA(hash);
			uc_desc->bd[desc->currBd].BD_CTRL.BUFLEN = desc->dbPtr;
			uc_desc->bd[desc->currBd].BD_CTRL.LAST_BD = 0;
			uc_desc->bd[desc->currBd].BD_CTRL.LIFM = 0;
			
			// If we are not the last buffer descriptor, enable it
			// and advance to the next one
			if ((len || (desc->processed != desc->msgSize)) && (desc->dbPtr % 4 == 0))
			{
                uc_desc->bd[desc->currBd].BD_CTRL.DESC_EN = 1;
                desc->currBd++;
                if (desc->currBd >= PIC32MZ_MAX_BD)
                    desc->currBd = 0;
    			while (uc_desc->bd[desc->currBd].BD_CTRL.DESC_EN);
                uc_desc->bd[desc->currBd].BD_CTRL.SA_FETCH_EN = 0;
                desc->dbPtr = 0;
			}
        }
        else
        {
            // We have to buffer everything and keep track of how much has been
            // added in order to get a total size. If the buffer fills, we move
            // to the next one. If we try to add more when the last buffer is
            // full, we error out.
            if (desc->dbPtr == PIC32_BLOCK_SIZE)
            {
                // We filled the last BD buffer, so move on to the next one
                uc_desc->bd[desc->currBd].BD_CTRL.LAST_BD = 0;
                uc_desc->bd[desc->currBd].BD_CTRL.LIFM = 0;
                uc_desc->bd[desc->currBd].BD_CTRL.BUFLEN = PIC32_BLOCK_SIZE;
                desc->dbPtr = 0;
                desc->currBd++;
                if (desc->currBd >= PIC32MZ_MAX_BD)
                {
                    desc->err = 1;
                }
				else
					uc_desc->bd[desc->currBd].UPDPTR = KVA_TO_PA(hash);
            }
            if (len > PIC32_BLOCK_SIZE - desc->dbPtr)
            {
                // We have more data than can be put in the buffer. Fill what we can.
                total = PIC32_BLOCK_SIZE - desc->dbPtr;
                XMEMCPY(&dataBuffer[desc->currBd][desc->dbPtr], input, total);
                len -= total;
                desc->processed += total;
                desc->dbPtr = PIC32_BLOCK_SIZE;
                input += total;
            }
            else
            {
                // Fill up what we have
                XMEMCPY(&dataBuffer[desc->currBd][desc->dbPtr], input, len);
                desc->dbPtr += len;
                desc->processed += len;
                len = 0;
            }
        }
    }
}

static void start_engine(pic32mz_desc *desc) {
    // Wrap up the last buffer descriptor and enable it
    int i ;
    int bufferLen ;    
    pic32mz_desc *uc_desc = KVA0_TO_KVA1(desc);

    bufferLen = desc->dbPtr;
    if (bufferLen % 4)
        bufferLen = (bufferLen + 4) - (bufferLen % 4);
    if (desc->sa.SA_CTRL.ALGO == 0b1)   // DES requires 8-byte packets
    {
        if (bufferLen % 8)
            bufferLen = (bufferLen + 8) - (bufferLen % 8);
    }
    else if (desc->sa.SA_CTRL.ALGO == 0b10) // TDES requires 24-byte packets
    {
        if (bufferLen % 24)
        {
            bufferLen = (bufferLen + 24) - (bufferLen % 24);
        }
    }
    uc_desc->bd[desc->currBd].BD_CTRL.BUFLEN = bufferLen;
    uc_desc->bd[desc->currBd].BD_CTRL.LAST_BD = 1;
    uc_desc->bd[desc->currBd].BD_CTRL.LIFM = 1;
    if (desc->msgSize == 0)
    {
        // We were not given the size, so now we have to go through every BD
        // and give it what will be processed, and enable them.
        for (i = desc->currBd; i >= 0; i--)
        {
            uc_desc->bd[i].MSGLEN = desc->processed;
            uc_desc->bd[i].BD_CTRL.DESC_EN = 1;
        }
    }
    else
    {
        uc_desc->bd[desc->currBd].BD_CTRL.DESC_EN = 1;
    }
}

void wait_engine(pic32mz_desc *desc, char *hash, int hash_sz) {
    unsigned int i;
#if ((__PIC32_FEATURE_SET0 == 'E') && (__PIC32_FEATURE_SET1 == 'C'))   // No output swap
    unsigned int *intptr;
#endif
    pic32mz_desc *uc_desc = KVA0_TO_KVA1(desc);
    bool engineRunning = true;
#undef DEBUG_CYASSL
    #ifdef DEBUG_CYASSL
    printf("desc(%x)[bd:%d * 2, sz:%d]\n", desc, sizeof(desc->bd[0]),
                                                 sizeof(desc->sa) );
    print_mem(KVA0_TO_KVA1(&(desc->bd[0])), sizeof(desc->bd[0])) ;
    print_mem(KVA0_TO_KVA1(&(desc->bd[1])), sizeof(desc->bd[0])) ;
    #endif

    while (engineRunning)
    {
        engineRunning = false;
        for (i = 0; i < PIC32MZ_MAX_BD; i++)
            engineRunning = engineRunning || uc_desc->bd[i].BD_CTRL.DESC_EN;
    }
    XMEMCPY(hash, KVA0_TO_KVA1(hash), hash_sz) ;

    #ifdef DEBUG_CYASSL
    print_mem(KVA0_TO_KVA1(hash), hash_sz) ;
    print_mem(             hash , hash_sz) ;
    #endif
#if ((__PIC32_FEATURE_SET0 == 'E') && (__PIC32_FEATURE_SET1 == 'C'))   // No output swap
    for (i = 0, intptr = (unsigned int *)hash; i < hash_sz/sizeof(unsigned int);
                                                                  i++, intptr++)
    {
        *intptr = ntohl(*intptr);
    }
#endif
}

#endif

#ifndef NO_MD5
void wc_InitMd5(Md5* md5)
{
    WOLFSSL_ENTER("InitMd5\n") ;
    reset_engine(&(md5->desc), PIC32_ALGO_MD5) ;
}

void wc_Md5Update(Md5* md5, const byte* data, word32 len)
{
     WOLFSSL_ENTER("Md5Update\n") ;
     update_engine(&(md5->desc), data, len, md5->digest) ;
}

void wc_Md5Final(Md5* md5, byte* hash)
{
     WOLFSSL_ENTER("Md5Final\n") ;
    start_engine(&(md5->desc)) ;
    wait_engine(&(md5->desc), (char *)md5->digest, MD5_HASH_SIZE) ;
    XMEMCPY(hash, md5->digest, MD5_HASH_SIZE) ;
    wc_InitMd5(md5);  /* reset state */
}

void wc_Md5SizeSet(Md5* md5, word32 len)
{
    WOLFSSL_ENTER("Md5SizeSet\n");
    md5->desc.msgSize = len;
}
#endif

#ifndef NO_SHA
int wc_InitSha(Sha* sha)
{
    WOLFSSL_ENTER("InitSha\n") ;
    reset_engine(&(sha->desc), PIC32_ALGO_SHA1) ;
    return 0;
}

int wc_ShaUpdate(Sha* sha, const byte* data, word32 len)
{
    WOLFSSL_ENTER("ShaUpdate\n") ;
    update_engine(&(sha->desc), data, len, sha->digest) ;
    return 0;
}

int wc_ShaFinal(Sha* sha, byte* hash)
{
    WOLFSSL_ENTER("ShaFinal\n") ;
    start_engine(&(sha->desc)) ;
    wait_engine(&(sha->desc), (char *)sha->digest, SHA1_HASH_SIZE) ;
    XMEMCPY(hash, sha->digest, SHA1_HASH_SIZE) ;

    wc_InitSha(sha);  /* reset state */
    return 0;
}

void wc_ShaSizeSet(Sha* sha, word32 len)
{
    sha->desc.msgSize = len;
}
#endif /* NO_SHA */

#ifndef NO_SHA256
int wc_InitSha256(Sha256* sha256)
{
    WOLFSSL_ENTER("InitSha256\n") ;
    reset_engine(&(sha256->desc), PIC32_ALGO_SHA256) ;
    return 0;
}

int wc_Sha256Update(Sha256* sha256, const byte* data, word32 len)
{
    WOLFSSL_ENTER("Sha256Update\n") ;
    update_engine(&(sha256->desc), data, len, sha256->digest) ;

    return 0;
}

int wc_Sha256Final(Sha256* sha256, byte* hash)
{
    WOLFSSL_ENTER("Sha256Final\n") ;
    start_engine(&(sha256->desc)) ;
    wait_engine(&(sha256->desc), (char *)sha256->digest, SHA256_HASH_SIZE) ;
    XMEMCPY(hash, sha256->digest, SHA256_HASH_SIZE) ;
    wc_InitSha256(sha256);  /* reset state */

    return 0;
}

void wc_Sha256SizeSet(Sha256* sha256, word32 len)
{
    WOLFSSL_ENTER("Sha256SizeSet\n");
    sha256->desc.msgSize = len;
}
#endif /* NO_SHA256 */

#endif



