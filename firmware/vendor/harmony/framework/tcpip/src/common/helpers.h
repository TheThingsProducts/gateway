/*******************************************************************************
  Header file for common MCHP helpers

  Summary:
    SUMMARY
    
  Description:
    DESCRIPTION
*******************************************************************************/

/*******************************************************************************
File Name: helpers.h 
Copyright © 2012 released Microchip Technology Inc.  All rights
reserved.

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

#ifndef __HELPERS_H_
#define __HELPERS_H_

#include <stdint.h>
#include <stdbool.h>


void        uitoa(uint16_t Value, uint8_t* Buffer);

// Implement consistent ultoa() function
#if (defined(__PIC32MX__) && (__C32_VERSION__ < 112)) || (defined (__C30__) && (__C30_VERSION__ < 325)) || defined(__C30_LEGACY_LIBC__) || defined(__C32_LEGACY_LIBC__)
	// C32 < 1.12 and C30 < v3.25 need this 2 parameter stack implemented function
	void ultoa(uint32_t Value, uint8_t* Buffer);
#else
	// C30 v3.25+, and C32 v1.12+ already have a ultoa() stdlib 
	// library function, but it requires 3 parameters.
	#include <stdlib.h>
	#define ultoa(val,buf)	ultoa((char*)(buf),(val),10)
#endif


uint8_t     hexatob(uint16_t AsciiVal);
uint8_t     btohexa_high(uint8_t b);
uint8_t     btohexa_low(uint8_t b);

char*       strupr(char* s);

char*       strnchr(const char *searchString, size_t count, char c);


size_t      strncpy_m(char* destStr, size_t destSize, int nStrings, ...);


signed char stricmppgm2ram(uint8_t* a, const uint8_t* b);

int16_t     str_replace(uint8_t *vExpression, const uint8_t *vFind, const uint8_t *vReplacement, uint16_t wMaxLen, bool bSearchCaseInsensitive);

#endif  // __HELPERS_H_

