/*******************************************************************************
  Helper Functions for Microchip tcpip

  Summary:
    ARCFOUR Cryptography Library
    
  Description:
    Common Microchip helper functions
*******************************************************************************/

/*******************************************************************************
File Name:  helpers.c
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



#include <ctype.h>
#include <stdarg.h>
#include <string.h>

#include "helpers.h"

typedef union 
{
    uint16_t Val;
    uint8_t v[2];
} MCHP_UINT16_VAL;

typedef union
{
    uint32_t Val;
    uint16_t w[2] __attribute__((packed));
    uint8_t  v[4];
} MCHP_UINT32_VAL;

/*****************************************************************************
  Function:
	void uitoa(uint16_t Value, uint8_t* Buffer)

  Summary:
	Converts an unsigned integer to a decimal string.
	
  Description:
	Converts a 16-bit unsigned integer to a null-terminated decimal string.
	
  Precondition:
	None

  Parameters:
	Value	- The number to be converted
	Buffer	- Pointer in which to store the converted string

  Returns:
  	None
  ***************************************************************************/
void uitoa(uint16_t Value, uint8_t* Buffer)
{
	uint8_t i;
	uint16_t Digit;
	uint16_t Divisor;
	bool Printed = false;

	if(Value)
	{
		for(i = 0, Divisor = 10000; i < 5u; i++)
		{
			Digit = Value/Divisor;
			if(Digit || Printed)
			{
				*Buffer++ = '0' + Digit;
				Value -= Digit*Divisor;
				Printed = true;
			}
			Divisor /= 10;
		}
	}
	else
	{
		*Buffer++ = '0';
	}

	*Buffer = '\0';
}			    


/*****************************************************************************
  Function:
	uint8_t hexatob(uint16_t AsciiVal)

  Summary:
	Converts a hex string to a single byte.
	
  Description:
	Converts a two-character ASCII hex string to a single packed byte.
	
  Precondition:
	None

  Parameters:
	AsciiVal - uint16_t where the LSB is the ASCII value for the lower nibble
					and MSB is the ASCII value for the upper nibble.  Each
					must range from '0'-'9', 'A'-'F', or 'a'-'f'.

  Returns:
  	Resulting packed byte 0x00 - 0xFF.
  ***************************************************************************/
uint8_t hexatob(uint16_t AsciiVal)
{
    MCHP_UINT16_VAL AsciiChars;
    AsciiChars.Val = AsciiVal;

	// Convert lowercase to uppercase
	if(AsciiChars.v[1] > 'F')
		AsciiChars.v[1] -= 'a'-'A';
	if(AsciiChars.v[0] > 'F')
		AsciiChars.v[0] -= 'a'-'A';

	// Convert 0-9, A-F to 0x0-0xF
	if(AsciiChars.v[1] > '9')
		AsciiChars.v[1] -= 'A' - 10;
	else
		AsciiChars.v[1] -= '0';

	if(AsciiChars.v[0] > '9')
		AsciiChars.v[0] -= 'A' - 10;
	else
		AsciiChars.v[0] -= '0';

	// Concatenate
	return (AsciiChars.v[1]<<4) |  AsciiChars.v[0];
}

/*****************************************************************************
  Function:
	uint8_t btohexa_high(uint8_t b)

  Summary:
	Converts the upper nibble of a binary value to a hexadecimal ASCII byte.

  Description:
	Converts the upper nibble of a binary value to a hexadecimal ASCII byte.
	For example, btohexa_high(0xAE) will return 'a'.

  Precondition:
	None

  Parameters:
	b - the byte to convert

  Returns:
  	The upper hexadecimal ASCII byte '0'-'9' or 'a'-'f'.
  ***************************************************************************/
uint8_t btohexa_high(uint8_t b)
{
	b >>= 4;
	return (b>0x9u) ? b+'a'-10:b+'0';
}

/*****************************************************************************
  Function:
	uint8_t btohexa_high(uint8_t b)

  Summary:
	Converts the lower nibble of a binary value to a hexadecimal ASCII byte.

  Description:
	Converts the lower nibble of a binary value to a hexadecimal ASCII byte.
	For example, btohexa_high(0xAE) will return 'e'.

  Precondition:
	None

  Parameters:
	b - the byte to convert

  Returns:
  	The lower hexadecimal ASCII byte '0'-'9' or 'a'-'f'.
  ***************************************************************************/
uint8_t btohexa_low(uint8_t b)
{
	b &= 0x0F;
	return (b>9u) ? b+'a'-10:b+'0';
}

/*****************************************************************************
  Function:
	char* strupr(char* s)

  Summary:
	Converts a string to uppercase.

  Description:
	This function converts strings to uppercase on platforms that do not
	already have this function defined.  All lower-case characters are
	converted, an characters not included in 'a'-'z' are left as-is.

  Precondition:
	None

  Parameters:
	s - the null-terminated string to be converted.

  Returns:
	Pointer to the initial string.
  ***************************************************************************/
char* strupr(char* s)
{
	char c;
	char *t;

	t = s;
	while( (c = *t) )
	{
		if(c >= 'a' && c <= 'z')
		{
			*t -= ('a' - 'A');
		}
		t++;
	}
	return s;
}

/*****************************************************************************
  Function:
	void ultoa(uint32_t Value, uint8_t* Buffer)

  Summary:
	Converts an unsigned integer to a decimal string.
	
  Description:
	Converts a 32-bit unsigned integer to a null-terminated decimal string.
	
  Precondition:
	None

  Parameters:
	Value	- The number to be converted
	Buffer	- Pointer in which to store the converted string

  Returns:
  	None
  ***************************************************************************/
// HI-TECH PICC-18 PRO 9.63, C30 v3.25, and C32 v1.12 already have a ultoa() library function
// C32 < 1.12 and C30 < v3.25 need this function
#if (defined(__PIC32MX__) && (__C32_VERSION__ < 112)) || (defined (__C30__) && (__C30_VERSION__ < 325)) || defined(__C30_LEGACY_LIBC__) || defined(__C32_LEGACY_LIBC__)
void ultoa(uint32_t Value, uint8_t* Buffer)
{
	uint8_t i;
	uint32_t Digit;
	uint32_t Divisor;
	bool Printed = false;

	if(Value)
	{
		for(i = 0, Divisor = 1000000000; i < 10; i++)
		{
			Digit = Value/Divisor;
			if(Digit || Printed)
			{
				*Buffer++ = '0' + Digit;
				Value -= Digit*Divisor;
				Printed = true;
			}
			Divisor /= 10;
		}
	}
	else
	{
		*Buffer++ = '0';
	}

	*Buffer = '\0';
}
#endif


/*****************************************************************************
  Function:
	char * strnchr(const char *searchString, size_t count, char c)

  Summary:
	Searches a string up to a specified number of characters for a specific 
	character.

  Description:
	Searches a string up to a specified number of characters for a specific 
	character.  The string is searched forward and the first occurance 
	location is returned.  If the search character is not present in the 
	string, or if the maximum character count is reached first, then a NULL 
	pointer is returned.

  Precondition:
	None

  Parameters:
	searchString - Pointer to a null terminated string to search.  If count is 
		less than the string size, then the string need not be null terminated.
	count - Maximum number of characters to search before aborting.
	c - Character to search for
	
  Returns:
	Pointer to the first occurance of the character c in the string 
	searchString.  If the character is not found or the maximum count is 
	reached, a NULL pointer is returned.
  ***************************************************************************/
char * strnchr(const char *searchString, size_t count, char c)
{
	char c2;
	
	while(count--)
	{
		c2  = *searchString++;
		if(c2 == 0u)
			return NULL;
		if(c2 == c)
			return (char*)--searchString;
	}
	return NULL;
}

/*****************************************************************************
  Function:
	char* strncpy_m(char* destStr, size_t destSize, int nStrings, ...)

  Summary:
	Copies multiple strings to a destination

  Description:
	Copies multiple strings to a destination
    but doesn't copy more than destSize characters.
    Useful where the destination is actually an array and an extra \0
    won't be appended to overflow the buffer
    
  Precondition:
	- valid string pointers
    - destSize should be > 0

  Parameters:
	destStr - Pointer to a string to be initialized with the multiple strings provided as arguments.

    destSize    - the maximum size of the destStr field, that cannot be exceeded.
                  An \0 won't be appended if the resulting size is > destSize

    nStrings    - number of string parameters to be copied into destStr

    ...         - variable number of arguments
    
	
  Returns:
	Length of the destination string, terminating \0 (if exists) not included
  ***************************************************************************/
size_t strncpy_m(char* destStr, size_t destSize, int nStrings, ...)
{
    va_list     args;
    const char* str;
    char*       end;
    size_t      len;

    destStr[0] = '\0';
    end = destStr + destSize - 1;
    *end = '\0';
    len = 0;
    
    va_start( args, nStrings );
    
    while(nStrings--)
    {
        if(*end)
        {   // if already full don't calculate strlen outside the string area
            len = destSize;
            break;
        }
        
        str = va_arg(args, const char*);
        strncpy(destStr + len, str, destSize - len);
        len += strlen(str);
    }

    va_end( args );
    
    return len;
}

/*****************************************************************************
  Function:
	signed char stricmppgm2ram(uint8_t* a, const uint8_t* b)

  Summary:
	Case-insensitive comparison of a string in RAM to a string in const.

  Description:
	Performs a case-insensitive comparison of a string in RAM to a string
	in const.  This function performs identically to strcmp, except that
	the comparison is not case-sensitive.

  Precondition:
	None

  Parameters:
	a - Pinter to tring in RAM
	b - Pointer to string in const

  Return Values:
  	\-1 - a < b
  	0	- a = b
  	1	- a > b
  ***************************************************************************/
signed char stricmppgm2ram(uint8_t* a, const uint8_t* b)
{
	uint8_t cA, cB;
	
	// Load first two characters
	cA = *a;
	cB = *b;
	
	// Loop until one string terminates
	while(cA != '\0' && cB != '\0')
	{
		// Shift case if necessary
		if(cA >= 'a' && cA <= 'z')
			cA -= 'a' - 'A';
		if(cB >= 'a' && cB <= 'z')
			cB -= 'a' - 'A';
			
		// Compare
		if(cA > cB)
			return 1;
		if(cA < cB)
			return -1;
		
		// Characters matched, so continue
		a++;
		b++;
		cA = *a;
		cB = *b;
	}
	
	// See if one string terminated first
	if(cA > cB)
		return 1;
	if(cA < cB)
		return -1;
		
	// Strings match
	return 0;
}

/*****************************************************************************
  Function:
	int16_t str_replace(uint8_t *vExpression, const uint8_t *vFind, const uint8_t *vReplacement, 
				  uint16_t wMaxLen, bool bSearchCaseInsensitive)

  Summary:
	Replaces all instances of a particular substring with a new string

  Description:
	Searches a string (vExpression) and replaces all instances of a particular 
	substring (vFind) with a new string (vReplacement).  The start offset to 
	being searching and a maximum number of replacements can be specified.  The 
	search can be performed in a case sensitive or case insensitive manner.

  Precondition:
	This function is commented out by default to save code space because 
	it is not used by any current stack features.  However, if you want to use 
	it, go ahead and uncomment it.  It has been tested, so it (should) work 
	correctly.

  Parameters:
	vExpression - Null terminated string to search and make replacements within.
	vFind - Null terminated string to search for.
	vReplacement - Null terminated string to replace all instances of vFind with.
	wMaxLen - Maximum length of the output vExpression string if string 
		expansion is going to occur (replacement length is longer than find 
		length).  If the replacements will cause this maximum string length to 
		be exceeded, then no replacements will be made and a negative result 
		will be returned, indicating failure.  If the replacement length is 
		shorter or equal to the search length, then this parameter is ignored.
	bSearchCaseInsensitive - Boolean indicating if the search should be 
		performed in a case insensitive manner.  Specify true for case 
		insensitive searches (slower) or false for case sensitive 
		searching (faster).

  Remarks:
	If the replacement string length is shorter than or equal to the search 
	string length and the search string occurs in multiple overlapping 
	locations (ex\: expression is "aaa", find is "aa", and replacement is "bb") 
	then the first find match occuring when searching from left to right will 
	be replaced.  (ex\: output expression will be "bba").
	
	However, if the replacement string length is longer than the search string 
	length, the search will occur starting from the end of the string and 
	proceed to the beginning (right to left searching).  In this case if the 
	expression was "aaa", find was "aa", and replacement was "bbb", then the 
	final output expression will be "abbb".  

  Returns:
	If zero or greater, indicates the count of how many replacements were made.  
	If less than zero (negative result), indicates that wMaxLen was too small 
	to make the necessary replacements.  In this case, no replacements were 
	made.
  ***************************************************************************/
#if 0
int16_t str_replace(uint8_t *vExpression, const uint8_t *vFind, const uint8_t *vReplacement, uint16_t wMaxLen, bool bSearchCaseInsensitive)
{
	uint16_t wExpressionLen, wFindLen, wFindLenMinusOne, wReplacementLen;
	uint16_t wFindCount, wReplacementsLeft;
	uint8_t i, j;
	uint8_t vFirstFindChar;
	uint16_t wBytesLeft;
	uint8_t *vDest;
	uint8_t *vExpressionCompare;
	const uint8_t *vFindCompare;
	uint16_t w;

	wFindLen = strlen((const char*)vFind);
	if(wFindLen == 0u)
		return 0;
	
	wExpressionLen = strlen((char*)vExpression);
	wReplacementLen = strlen((const char*)vReplacement);

	wFindCount = 0;
	wFindLenMinusOne = wFindLen - 1;
	vFirstFindChar = *vFind++;
	if(bSearchCaseInsensitive)	// Convert to all lowercase if needed
		if((vFirstFindChar >= (uint8_t)'A') && (vFirstFindChar <= (uint8_t)'Z'))
			vFirstFindChar += 'a' - 'A';

	// If the replacement string is the same length as the search string, then 
	// we can immediately do the needed replacements inline and return.
	if(wFindLen == wReplacementLen)
	{
		for(wBytesLeft = wExpressionLen; wBytesLeft; wBytesLeft--)
		{
			i = *vExpression++;
			if(bSearchCaseInsensitive)
			{
				if((i >= (uint8_t)'A') && (i <= (uint8_t)'Z'))
					i += 'a' - 'A';
				if(i != vFirstFindChar)
					continue;
				vExpressionCompare = vExpression;
				vFindCompare = vFind;
				w = wFindLenMinusOne;
				while(w)
				{
					i = *vExpressionCompare++;
					j = *vFindCompare++;
					if((i >= (uint8_t)'A') && (i <= (uint8_t)'Z'))
						i += 'a' - 'A';
					if((j >= (uint8_t)'A') && (j <= (uint8_t)'Z'))
						j += 'a' - 'A';
					if(i != j)
						break;
					w--;
				}
				if(w)
					continue;
			}
			else
			{
				if(i != vFirstFindChar)
					continue;
				if(memcmp((void*)vExpression, (const void*)vFind, wFindLenMinusOne))
					continue;
			}
	
			memcpy((void*)vExpression-1, (const void*)vReplacement, wReplacementLen);
			wFindCount++;
			vExpression += wFindLenMinusOne;
			wBytesLeft -= wFindLenMinusOne;
		}
		return wFindCount;
	}
	
	
	// If the replacement string is shorter than the search string, then we can 
	// search from left to right and move the string over as we find occurrences.
	if(wFindLen > wReplacementLen)
	{
		vDest = vExpression;
		for(wBytesLeft = wExpressionLen; wBytesLeft; wBytesLeft--)
		{
			i = *vExpression++;
			*vDest++ = i;
			if(bSearchCaseInsensitive)
			{
				if((i >= (uint8_t)'A') && (i <= (uint8_t)'Z'))
					i += 'a' - 'A';
				if(i != vFirstFindChar)
					continue;
				vExpressionCompare = vExpression;
				vFindCompare = vFind;
				w = wFindLenMinusOne;
				while(w)
				{
					i = *vExpressionCompare++;
					j = *vFindCompare++;
					if((i >= (uint8_t)'A') && (i <= (uint8_t)'Z'))
						i += 'a' - 'A';
					if((j >= (uint8_t)'A') && (j <= (uint8_t)'Z'))
						j += 'a' - 'A';
					if(i != j)
						break;
					w--;
				}
				if(w)
					continue;
			}
			else
			{
				if(i != vFirstFindChar)
					continue;
				if(memcmp((void*)vExpression, (const void*)vFind, wFindLenMinusOne))
					continue;
			}
	
			memcpy((void*)vDest-1, (const void*)vReplacement, wReplacementLen);
			vDest += wReplacementLen-1;
			wFindCount++;
			vExpression += wFindLenMinusOne;
			wBytesLeft -= wFindLenMinusOne;
		}
		*vDest = 0x00;	// Write new null terminator since the string may have shrunk
		return wFindCount;
	}
	
	// If the replacement string is longer than the search string, then we will 
	// take a two pass approach.  On the first pass, we will merely count how 
	// many replacements to make.  With this we can calculate how long the 
	// final string is going to be.  On the second pass, we will search from 
	// right to left and expand the string as needed.

	// Pass 1: count how many occurrences of vFind are in vExpression
	for(wBytesLeft = wExpressionLen; wBytesLeft; wBytesLeft--)
	{
		i = *vExpression++;
		if(bSearchCaseInsensitive)
		{
			if((i >= (uint8_t)'A') && (i <= (uint8_t)'Z'))
				i += 'a' - 'A';
			if(i != vFirstFindChar)
				continue;
			vExpressionCompare = vExpression;
			vFindCompare = vFind;
			w = wFindLenMinusOne;
			while(w)
			{
				i = *vExpressionCompare++;
				j = *vFindCompare++;
				if((i >= (uint8_t)'A') && (i <= (uint8_t)'Z'))
					i += 'a' - 'A';
				if((j >= (uint8_t)'A') && (j <= (uint8_t)'Z'))
					j += 'a' - 'A';
				if(i != j)
					break;
				w--;
			}
			if(w)
				continue;
		}
		else
		{
			if(i != vFirstFindChar)
				continue;
			if(memcmp((void*)vExpression, (const void*)vFind, wFindLenMinusOne))
				continue;
		}

		wFindCount++;
		vExpression += wFindLenMinusOne;
		wBytesLeft -= wFindLenMinusOne;
	}
	
	// Return immediately if no replacements are needed
	if(wFindCount == 0u)
		return 0;

	// Pass 2: make replacements and move string over
	vDest = vExpression + wFindCount * (wReplacementLen - wFindLen);
	if(vDest > vExpression - wExpressionLen + wMaxLen)
		return -1;
	*vDest-- = 0x00;	// Write new null terminator
	vExpression -= 1;
	vFind -= 1;
	vFirstFindChar = vFind[wFindLenMinusOne];
	if(bSearchCaseInsensitive)	// Convert to all lowercase if needed
		if((vFirstFindChar >= (uint8_t)'A') && (vFirstFindChar <= (uint8_t)'Z'))
			vFirstFindChar += 'a' - 'A';
	wReplacementsLeft = wFindCount;
	while(wReplacementsLeft)
	{
		i = *vExpression--;
		*vDest-- = i;
		if(bSearchCaseInsensitive)
		{
			if((i >= (uint8_t)'A') && (i <= (uint8_t)'Z'))
				i += 'a' - 'A';
			if(i != vFirstFindChar)
				continue;
			vExpressionCompare = vExpression;
			vFindCompare = &vFind[wFindLenMinusOne-1];
			w = wFindLenMinusOne;
			while(w)
			{
				i = *vExpressionCompare--;
				j = *vFindCompare--;
				if((i >= (uint8_t)'A') && (i <= (uint8_t)'Z'))
					i += 'a' - 'A';
				if((j >= (uint8_t)'A') && (j <= (uint8_t)'Z'))
					j += 'a' - 'A';
				if(i != j)
					break;
				w--;
			}
			if(w)
				continue;
		}
		else
		{
			if(i != vFirstFindChar)
				continue;
			if(memcmp((void*)vExpression-wFindLenMinusOne, (const void*)vFind, wFindLenMinusOne))
				continue;
		}
		memcpy((void*)vDest-wReplacementLen+2, (const void*)vReplacement, wReplacementLen);
		vDest -= wReplacementLen-1;

		vExpression -= wFindLenMinusOne;
		wBytesLeft -= wFindLenMinusOne;
		wReplacementsLeft--;
	}
	return wFindCount;
}
#endif
