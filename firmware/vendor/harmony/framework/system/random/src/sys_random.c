/*******************************************************************************
  Random System Service Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    sys_random.c

  Summary:
    Random Generator System Service implementation.

  Description:
    This file contains the implementation for the Random Generator
    System Service.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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
// *****************************************************************************
// *****************************************************************************
// Section: Macro Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system/random/src/sys_random_local.h"

#ifndef SYS_RANDOM_CRYPTO_SEED_SIZE
    #error "SYS_RANDOM_CRYPTO_SEED_SIZE is a required build configuration parameter"
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Local Data
// *****************************************************************************
// *****************************************************************************


static SYS_RANDOM_INIT      randInitDefault = { 0 };

static SYS_RANDOM_OBJECT    sysRandObject = { 0 };

CRYPT_RNG_CTX               sysRandCtx;


SYS_MODULE_OBJ SYS_RANDOM_Initialize( const SYS_MODULE_INDEX index, const SYS_MODULE_INIT * const init )
{
    sysRandObject.index = index;
    sysRandObject.status = SYS_STATUS_BUSY;

    const SYS_RANDOM_INIT* pInitData = (const SYS_RANDOM_INIT*)init;

    if(pInitData == NULL)
    {
        pInitData = &randInitDefault;
    }

    srand(pInitData->seedPseudo);

#ifdef SYS_RANDOM_USE_CRYPTO_STRENGTH
    if(CRYPT_RNG_Initialize(&sysRandCtx) < 0)
    {   // failed
        sysRandObject.status = SYS_STATUS_UNINITIALIZED;
        return SYS_MODULE_OBJ_INVALID;
    }
#endif

    sysRandObject.seedCryptoSize = pInitData->seedCryptoSize;
    sysRandObject.status = SYS_STATUS_READY;
    /* Return dummy object */
    return 0;    
}



void SYS_RANDOM_Deinitialize( SYS_MODULE_OBJ object )
{
    sysRandObject.status = SYS_STATUS_UNINITIALIZED;
    
}




// *****************************************************************************
// *****************************************************************************
// Section: SYS RANDOM Pseudo Random Number Generator Routines
// Use the std C library calls
// *****************************************************************************
// *****************************************************************************


void SYS_RANDOM_PseudoSeedSet( uint32_t seed )
{
    srand(seed);
}



uint32_t SYS_RANDOM_PseudoGet( void )
{
    return (uint32_t)rand();
}

// *****************************************************************************
// *****************************************************************************
// Section: SYS RANDOM Cryptographic Random Number Generator Routines
// *****************************************************************************
// *****************************************************************************

#ifdef SYS_RANDOM_USE_CRYPTO_STRENGTH

void SYS_RANDOM_CryptoSeedSet( void *seed, size_t size)
{
    // TODO
}

size_t SYS_RANDOM_CryptoSeedSizeGet( void )
{
    return (sysRandObject.status == SYS_STATUS_READY) ? sysRandObject.seedCryptoSize : 0;
}



uint32_t SYS_RANDOM_CryptoGet( void )
{
    union
    {
        uint8_t     u8[4];
        uint32_t    u32;
    }sUint = {};


    CRYPT_RNG_BlockGenerate(&sysRandCtx, (unsigned char*)sUint.u8, sizeof(sUint.u8));

    return sUint.u32;
    // TODO: set error status/return error if failed
}


void SYS_RANDOM_CryptoBlockGet( void *buffer, size_t size )
{
    if(buffer != 0 && size != 0)
    {
        CRYPT_RNG_BlockGenerate(&sysRandCtx, (unsigned char*)buffer, size);
    }

    // TODO: set error status/return error if failed

}


uint8_t SYS_RANDOM_CryptoByteGet( void )
{
    uint8_t rNo = 0;
    CRYPT_RNG_Get(&sysRandCtx, (unsigned char*)&rNo);

    return rNo;
    // TODO: set error status/return error if failed
}




void SYS_RANDOM_CryptoEntropyAdd( uint8_t data )
{
    // TODO
}

#endif


