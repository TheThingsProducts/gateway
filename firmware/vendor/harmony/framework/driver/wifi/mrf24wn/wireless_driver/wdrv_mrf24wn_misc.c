/*******************************************************************************
  MRF24WN Wireless Driver

  File Name:  
    wdrv_mrf24wn_misc.c  
  
  Summary:
    Module for Microchip TCP/IP Stack PIC32 implementation
    for multiple MAC support
    
  Description:
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

#include "tcpip/src/tcpip_private.h"

#if defined(__PIC32MZ__)
void WDRV_DataCacheClean(unsigned char* address, uint32_t size)
{
    if (IS_KVA0(address)) {
        uint32_t a = (uint32_t)address & 0xfffffff0;
        uint32_t r = (uint32_t)address & 0x0000000f;
        uint32_t s = ((size + r + 15) >> 4) << 4;

        SYS_DEVCON_DataCacheClean(a, s);
    }
}
#endif /* defined(__PIC32MZ__) */

void WDRV_Assert(int condition, const char * msg, const char * file, int line)
{
    /*
     * Although WDRV_ASSERT is pointing to the system's assert, the blob does use this function still. So re-routing it the hard way
     */
    if (!condition) {
        _AssertImpl(file, line, "wdrv blob", msg);
    }
}

//DOM-IGNORE-END
