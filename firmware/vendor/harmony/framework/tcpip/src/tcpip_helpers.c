/*******************************************************************************
  Helper Functions for Microchip tcpip

  Summary:
    ARCFOUR Cryptography Library
    
  Description:
    Helpers library for Microchip TCP/IP Stack
*******************************************************************************/

/*******************************************************************************
File Name:  tcpip_helpers.c
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

#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_MANAGER

#include "tcpip/src/tcpip_private.h"

#include <ctype.h>
#include <stdarg.h>


// table with private IPv4 addresses, BE
typedef struct
{
    uint32_t    mask;
    uint32_t    address;
}_TCPIP_Helper_PrivateAddEntry;

static const _TCPIP_Helper_PrivateAddEntry _TCPIP_Helper_PrivateAddTbl[] = 
{
    {0x000000ff, 0x0000000a},     // 10.0.0.0/8
    {0x0000f0ff, 0x000010ac},     // 172.16.0.0/12
    {0x0000ffff, 0x0000a8c0},     // 192.168.0.0/16
};

typedef enum
{
    TCPIP_HELPER_PORT_FLAG_STREAM   = 0x01,     // stream socket port
    TCPIP_HELPER_PORT_FLAG_DGRAM    = 0x02,     // datagram socket port

}TCPIP_HELPER_PORT_FLAGS;

typedef struct
{
    uint16_t    port;
    uint16_t    flags;  // TCPIP_HELPER_PORT_FLAGS value
}TCPIP_HELPER_PORT_ENTRY;

static TCPIP_HELPER_PORT_ENTRY tcpSecurePortTbl[TCPIP_STACK_SECURE_PORT_ENTRIES] = 
{
    // port         flags                                                               // module
    {443,           TCPIP_HELPER_PORT_FLAG_STREAM | TCPIP_HELPER_PORT_FLAG_DGRAM},      // HTTPS
    {465,           TCPIP_HELPER_PORT_FLAG_STREAM },                                    // SMTPS
    {587,           TCPIP_HELPER_PORT_FLAG_STREAM },                                    // SMTPS
    {563,           TCPIP_HELPER_PORT_FLAG_STREAM | TCPIP_HELPER_PORT_FLAG_DGRAM},      // NNTPS
    {636,           TCPIP_HELPER_PORT_FLAG_STREAM | TCPIP_HELPER_PORT_FLAG_DGRAM},      // LDAPS
    {989,           TCPIP_HELPER_PORT_FLAG_STREAM | TCPIP_HELPER_PORT_FLAG_DGRAM},      // FTPS
    {990,           TCPIP_HELPER_PORT_FLAG_STREAM | TCPIP_HELPER_PORT_FLAG_DGRAM},      // FTPS
    {992,           TCPIP_HELPER_PORT_FLAG_STREAM | TCPIP_HELPER_PORT_FLAG_DGRAM},      // telnet
    {993,           TCPIP_HELPER_PORT_FLAG_STREAM },                                    // IMAPS
    {0,             TCPIP_HELPER_PORT_FLAG_STREAM | TCPIP_HELPER_PORT_FLAG_DGRAM},      // extra slots available
};

static TCPIP_HELPER_PORT_ENTRY*    _TCPIP_Helper_SecurePortEntry(uint16_t port, TCPIP_HELPER_PORT_ENTRY** pFreeEntry);

/*****************************************************************************
  Function:
    bool TCPIP_Helper_StringToIPAddress(uint8_t* str, IPV4_ADDR* addr)

  Summary:
	Converts a string to an IP address

  Description:
	This function parses a dotted-quad decimal IP address string into an 
	IPV4_ADDR struct.  The output result is big-endian.
	
  Precondition:
	None

  Parameters:
	str - Pointer to a dotted-quad IP address string
	addr - Pointer to IPV4_ADDR in which to store the result

  Return Values:
  	true - an IP address was successfully decoded
  	false - no IP address could be found, or the format was incorrect
  ***************************************************************************/
bool TCPIP_Helper_StringToIPAddress(const char* str, IPV4_ADDR* addr)
{
	TCPIP_UINT32_VAL dwVal;
	uint8_t i, charLen, currentOctet;

	charLen = 0;
	currentOctet = 0;
	dwVal.Val = 0;
    addr->Val = 0;

	while((i = *str++))
	{
		if(currentOctet > 3u)
			break;

		i -= '0';
		

		// Validate the character is a numerical digit or dot, depending on location
		if(charLen == 0u)
		{
			if(i > 9u)
				return false;
		}
		else if(charLen == 3u)
		{
			if(i != (uint8_t)('.' - '0'))
				return false;

			if(dwVal.Val > 0x00020505ul)
				return false;

			addr->v[currentOctet++] = dwVal.v[2]*((uint8_t)100) + dwVal.v[1]*((uint8_t)10) + dwVal.v[0];
			charLen = 0;
			dwVal.Val = 0;
			continue;
		}
		else
		{
			if(i == (uint8_t)('.' - '0'))
			{
				if(dwVal.Val > 0x00020505ul)
					return false;

				addr->v[currentOctet++] = dwVal.v[2]*((uint8_t)100) + dwVal.v[1]*((uint8_t)10) + dwVal.v[0];
				charLen = 0;
				dwVal.Val = 0;
				continue;
			}
			if(i > 9u)
				return false;
		}

		charLen++;
		dwVal.Val <<= 8;
		dwVal.v[0] = i;
	}

	// Make sure the very last character is a valid termination character 
	// (i.e., not more hostname, which could be legal and not an IP 
	// address as in "10.5.13.233.picsaregood.com"
	if(currentOctet != 3 || (i != 0u && i != '/' && i != '\r' && i != '\n' && i != ' ' && i != '\t' && i != ':'))
		return false;

	// Verify and convert the last octet and return the result
	if(dwVal.Val > 0x00020505ul)
		return false;

	addr->v[3] = dwVal.v[2]*((uint8_t)100) + dwVal.v[1]*((uint8_t)10) + dwVal.v[0];

	return true;
}

bool TCPIP_Helper_IPAddressToString(const IPV4_ADDR* ipAdd, char* buff, size_t buffSize)
{
    if(ipAdd && buff)
    {
        size_t len;
        char tempBuff[20];  // enough to hold largest IPv4 address string

        sprintf(tempBuff, "%d.%d.%d.%d", ipAdd->v[0], ipAdd->v[1], ipAdd->v[2], ipAdd->v[3]);
        len = strlen(tempBuff) + 1;
        if(buffSize >= len)
        {
            strcpy(buff, tempBuff);
            return true;
        }
    }

    return false; 
}

//#if defined TCPIP_STACK_USE_IPV6
/*****************************************************************************
  Function:
	bool StringToIPV6Address(uint8_t* str, IPV6_ADDR* addr)

  Summary:
	Converts a string to an IPv6 address

  Description:
	This function parses the text representation of an IPv6 address
	IPV6_ADDR struct.  The output result is big-endian.
	
  Precondition:
	None

  Parameters:
	addr - Pointer to IPV6_ADDR in which to store the result
	str - Pointer to an RFC3513, Section 2.2 text representation of
        an IPv6 address
        Example:    1111:2222:3333:4444:5555:6666:AAAA:FFFF
                    1111:2222::FFFF
                    1111:2222:3333:4444:5555:6666:192.168.1.20

  Return Values:
  	true - an IP address was successfully decoded
  	false - no IP address could be found, or the format was incorrect
  ***************************************************************************/
bool TCPIP_Helper_StringToIPv6Address (const char * str, IPV6_ADDR * addr)
{
    uint8_t shiftIndex = 0xFF;
    uint8_t subString[5];
    uint16_t convertedValue;
    uint8_t i, j;
    uint8_t currentWord;
    char * endPtr;
    
    if (*str == '[')
        str++;

    currentWord = 0;
    i = *str++;
    while (i != ':' && i != 0u && i != ']' && i != '/' && i != '\r' && i != '\n' && i != ' ' && i != '\t')
    {
        j = 0;
        while (i != ':' && i != 0u && i != ']' && i != '/' && i != '\r' && i != '\n' && i != ' ' && i != '\t')
        {
            if (j == 4)
                return false;
                
            subString[j++] = i;
            i = *str++;
        }
        subString[j] = 0;
        
        convertedValue = (uint16_t)strtol((const char *)subString, &endPtr, 16);
        
        addr->w[currentWord++] = TCPIP_Helper_htons(convertedValue);
        
        if(i == 0)
        {   // end of stream
            break;
        }

        if (i == ':')
        {
            if (*str == ':')
            {
                // Double colon - pad with zeros here
                if (shiftIndex == 0xFF)
                {
                    shiftIndex = currentWord;
                }
                else
                {
                    // Can't have two double colons
                    return false;
                }
                i = *str++;
            }
        }
        
        if (i == ',')
        {
            return false;
        }
        
        i = *str++;
    }

    if (shiftIndex != 0xFF)
    {
        for (i = 7, j = currentWord - 1; (int8_t)j >= (int8_t)shiftIndex; i--, j--)
        {
            addr->w[i] = addr->w[j];
        }
        for (i = shiftIndex; i <= 7 - (currentWord - shiftIndex); i++)
        {
            addr->w[i] = 0x0000;
        }
    }

    return true;
}

bool TCPIP_Helper_IPv6AddressToString (const IPV6_ADDR * v6Addr, char* buff, size_t buffSize)
{
    if(v6Addr && buff && buffSize >= 41)
    {
        uint8_t i, j;
        char k;
        char* str = buff;

        for (i = 0; i < 8; i++)
        {
            j = false;
            k = btohexa_high(v6Addr->v[(i<<1)]);
            if (k != '0')
            {
                j = true;
                *str++ = k;
            }
            k = btohexa_low(v6Addr->v[(i<<1)]);
            if (k != '0' || j == true)
            {
                j = true;
                *str++ = k;
            }
            k = btohexa_high(v6Addr->v[1 + (i<<1)]);
            if (k != '0' || j == true)
                *str++ = k;
            k = btohexa_low(v6Addr->v[1 + (i<<1)]);
            *str++ = k;
            if (i != 7)
                *str++ = ':';
        }
        *str = 0;

        return true;
    }

    return false;
}
//#endif

/*****************************************************************************
  Function:
    bool TCPIP_Helper_IsPrivateAddress(uint32_t ipv4Address)

  Summary:
	Detects a private (non-routable) address

  Description:
	This function checks if the passed in IPv4 address
    is a private or a routable address

  Precondition:
	None

  Parameters:
	ipv4Address - IPv4 address to check, network order

  Return Values:
  	true - the IPv4 address is a private address
  	false - the IPv4 address is a routable address
  ***************************************************************************/
bool TCPIP_Helper_IsPrivateAddress(uint32_t ipv4Address)
{
    int ix;
    const _TCPIP_Helper_PrivateAddEntry* pEntry;

    pEntry = _TCPIP_Helper_PrivateAddTbl;
    for(ix = 0; ix < sizeof(_TCPIP_Helper_PrivateAddTbl)/sizeof(*_TCPIP_Helper_PrivateAddTbl); ix++, pEntry++)
    {
        if((ipv4Address & pEntry->mask) == pEntry->address)
        {
            return true;
        }
    }


    return false;
}


/*****************************************************************************
  Function:
    bool TCPIP_Helper_StringToMACAddress(const char* str, uint8_t macAddr[6])

  Summary:
	Converts a string to an MAC address

  Description:
	This function parses a MAC address string "aa:bb:cc:dd:ee:ff"
    or "aa-bb-cc-dd-ee-ff" into an hex MAC address.

  Precondition:
	None

  Parameters:
	str - Pointer to a colon separated MAC address string
	macAddr - Pointer to buffer to store the result

  Return Values:
  	true - a MAC address was successfully decoded
  	false - no MAC address could be found, or the format was incorrect
  ***************************************************************************/
bool TCPIP_Helper_StringToMACAddress(const char* str, uint8_t macAddr[6])
{
    const char  *beg;
    TCPIP_UINT16_VAL    hexDigit;
    int         ix;
    

    beg = str;
    for(ix=0; ix<6; ix++)
    {
        if(!isxdigit(beg[0]) || !isxdigit(beg[1]))
        {
            return false;
        }

        // found valid byte
        hexDigit.v[0] = beg[1];
        hexDigit.v[1] = beg[0];
        *macAddr++ = hexatob(hexDigit.Val);

        // next colon number
        beg += 2;
        if(beg[0] == '\0')
        {
            break;  // done
        }
        else if(beg[0] != ':' && beg[0] != '-')
        {
            return false;   // invalid delimiter
        }
        beg++; // next digit
    }
    
    return ix==5?true:false;    // false if not enough digits    
    
}

bool TCPIP_Helper_MACAddressToString(const TCPIP_MAC_ADDR* macAddr, char* buff, size_t buffSize)
{
    if(macAddr && buff && buffSize >= 18)
    {
        const uint8_t *pAdd = (const uint8_t*)macAddr;

        sprintf(buff, "%.2x:%.2x:%.2x:%.2x:%.2x:%.2x", *pAdd, *(pAdd+1), *(pAdd+2), *(pAdd+3), *(pAdd+4), *(pAdd+5));
        return true;
    }
    return false;
}

typedef struct
{
    TCPIP_MAC_POWER_MODE  pwrMode;    // the power mode
    const char*             pwrName;    // corresponding name
}TCPIP_MAC_POWER_ENTRY;

static const TCPIP_MAC_POWER_ENTRY TCPIP_MAC_POWER_TBL[] = 
{
    {TCPIP_MAC_POWER_NONE, TCPIP_STACK_IF_POWER_NONE},
    {TCPIP_MAC_POWER_FULL, TCPIP_STACK_IF_POWER_FULL},
    {TCPIP_MAC_POWER_LOW,  TCPIP_STACK_IF_POWER_LOW},
    {TCPIP_MAC_POWER_DOWN, TCPIP_STACK_IF_POWER_DOWN},
};



/*****************************************************************************
  Function:
    TCPIP_MAC_POWER_MODE TCPIP_Helper_StringToPowerMode(const char* str)

  Summary:
	Converts a string to a TCPIP stack power mode

  Description:
	This function parses a power mode string ("full", "low", "down", etc)
    to an internal TCPIP_MAC_POWER_MODE. 

  Precondition:
	None

  Parameters:
	str - Pointer to a power mode string

  Return Values:
  	a valid TCPIP_MAC_POWER_MODE - a power mode was successfully decoded
  	TCPIP_MAC_POWER_NONE - no power mode could be found
  ***************************************************************************/
TCPIP_MAC_POWER_MODE TCPIP_Helper_StringToPowerMode(const char* str)
{
    int pwrIx;
    const TCPIP_MAC_POWER_ENTRY* pEntry;

    pEntry = TCPIP_MAC_POWER_TBL + 0;
    for(pwrIx = 0; pwrIx < sizeof(TCPIP_MAC_POWER_TBL)/sizeof(*TCPIP_MAC_POWER_TBL); pwrIx++)
    {
        if(str && pEntry->pwrName)
        {
            if(!strcmp(str, pEntry->pwrName))
            {
                return pEntry->pwrMode;
            }
        }
        pEntry++;
    }

    return TCPIP_MAC_POWER_NONE;
}

const char* TCPIP_Helper_PowerModeToString(TCPIP_MAC_POWER_MODE mode)
{
    if(mode >= 0 && mode < sizeof(TCPIP_MAC_POWER_TBL)/sizeof(*TCPIP_MAC_POWER_TBL))
    {
        return (TCPIP_MAC_POWER_TBL + mode)->pwrName;
    }

    return 0;
}

/*****************************************************************************
  Function:
	uint16_t TCPIP_Helper_Base64Decode(uint8_t* cSourceData, uint16_t wSourceLen, 
						uint8_t* cDestData, uint16_t wDestLen)

  Description:
	Decodes a Base-64 array to its literal representation.
	
  Precondition:
	None

  Parameters:
	cSourceData - Pointer to a string of Base-64 encoded data
	wSourceLen	- Length of the Base-64 source data
	cDestData	- Pointer to write the decoded data
	wSourceLen	- Maximum length that can be written to cDestData

  Returns:
  	Number of decoded bytes written to cDestData.
  
  Remarks:
	This function is binary safe and will ignore invalid characters (CR, LF, 
	etc).  If cSourceData is equal to cDestData, the data will be converted
	in-place.  If cSourceData is not equal to cDestData, but the regions 
	overlap, the behavior is undefined.
	
	Decoded data is always at least 1/4 smaller than the source data.
  ***************************************************************************/
#if defined(TCPIP_HTTP_USE_BASE64_DECODE) || defined(TCPIP_HTTP_NET_USE_BASE64_DECODE)
uint16_t TCPIP_Helper_Base64Decode(uint8_t* cSourceData, uint16_t wSourceLen, uint8_t* cDestData, uint16_t wDestLen)
{
	uint8_t i;
	uint8_t vByteNumber;
	bool bPad;
	uint16_t wBytesOutput;

	vByteNumber = 0;
	wBytesOutput = 0;

	// Loop over all provided bytes
	while(wSourceLen--)
	{
		// Fetch a Base64 byte and decode it to the original 6 bits
		i = *cSourceData++;
		bPad = (i == '=');
		if(i >= 'A' && i <= 'Z')	// Regular data
			i -= 'A' - 0;
		else if(i >= 'a' && i <= 'z')
			i -= 'a' - 26;
		else if(i >= '0' && i <= '9')
			i -= '0' - 52;
		else if(i == '+' || i == '-')
			i = 62;
		else if(i == '/' || i == '_')
			i = 63;
		else 						// Skip all padding (=) and non-Base64 characters
			continue;


		// Write the 6 bits to the correct destination location(s)
		if(vByteNumber == 0u)
		{
			if(bPad)				// Padding here would be illegal, treat it as a non-Base64 chacter and just skip over it
				continue;
			vByteNumber++;
			if(wBytesOutput >= wDestLen)
				break;
			wBytesOutput++;
			*cDestData = i << 2;
		}
		else if(vByteNumber == 1u)
		{
			vByteNumber++;
			*cDestData++ |= i >> 4;
			if(wBytesOutput >= wDestLen)
				break;
			if(bPad)
				continue;
			wBytesOutput++;
			*cDestData = i << 4;
		}
		else if(vByteNumber == 2u)
		{
			vByteNumber++;
			*cDestData++ |= i >> 2;
			if(wBytesOutput >= wDestLen)
				break;
			if(bPad)
				continue;
			wBytesOutput++;
			*cDestData = i << 6;
		}
		else if(vByteNumber == 3u)
		{
			vByteNumber = 0;
			*cDestData++ |= i;
		}
	}

	return wBytesOutput;
}
#endif  // defined(TCPIP_HTTP_USE_BASE64_DECODE) || defined(TCPIP_HTTP_NET_USE_BASE64_DECODE)


/*****************************************************************************
  Function:
	uint16_t TCPIP_Helper_Base64Encode(uint8_t* cSourceData, uint16_t wSourceLen,
						uint8_t* cDestData, uint16_t wDestLen)

  Description:
	Encodes a binary array to Base-64.
	
  Precondition:
	None

  Parameters:
	cSourceData - Pointer to a string of binary data
	wSourceLen	- Length of the binary source data
	cDestData	- Pointer to write the Base-64 encoded data
	wSourceLen	- Maximum length that can be written to cDestData

  Returns:
  	Number of encoded bytes written to cDestData.  This will always be
  	a multiple of 4.
  
  Remarks:
	Encoding cannot be performed in-place.  If cSourceData overlaps with 
	cDestData, the behavior is undefined.
	
	Encoded data is always at least 1/3 larger than the source data.  It may
	be 1 or 2 bytes larger than that.
  ***************************************************************************/
#if defined(TCPIP_STACK_USE_BASE64_ENCODE) || defined(TCPIP_STACK_USE_SMTP_CLIENT) || defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
uint16_t TCPIP_Helper_Base64Encode(uint8_t* cSourceData, uint16_t wSourceLen, uint8_t* cDestData, uint16_t wDestLen)
{
	uint8_t i, j;
	uint8_t vOutput[4];
	uint16_t wOutputLen;

	wOutputLen = 0;
	while(wDestLen >= 4u)
	{
		// Start out treating the output as all padding
		vOutput[0] = 0xFF;
		vOutput[1] = 0xFF;
		vOutput[2] = 0xFF;
		vOutput[3] = 0xFF;

		// Get 3 input octets and split them into 4 output hextets (6-bits each) 
		if(wSourceLen == 0u)
			break;
		i = *cSourceData++;
		wSourceLen--;
		vOutput[0] = (i & 0xFC)>>2;
		vOutput[1] = (i & 0x03)<<4;
		if(wSourceLen)
		{
			i = *cSourceData++;
			wSourceLen--;
			vOutput[1] |= (i & 0xF0)>>4;
			vOutput[2] = (i & 0x0F)<<2;
			if(wSourceLen)
			{
				i = *cSourceData++;
				wSourceLen--;
				vOutput[2] |= (i & 0xC0)>>6;
				vOutput[3] = i & 0x3F;
			}
		}
	
		// Convert hextets into Base 64 alphabet and store result
		for(i = 0; i < 4u; i++)
		{
			j = vOutput[i];

			if(j <= 25u)
				j += 'A' - 0;
			else if(j <= 51u)
				j += 'a' - 26;
			else if(j <= 61u)
				j += '0' - 52;
			else if(j == 62u)
				j = '+';
			else if(j == 63u)
				j = '/';
			else				// Padding
				j = '=';

			*cDestData++ = j;
		}

		// Update counters
		wDestLen -= 4;
		wOutputLen += 4;
	}

	return wOutputLen;
}
#endif // #if defined(TCPIP_STACK_USE_BASE64_ENCODE) || defined(TCPIP_STACK_USE_SMTP) || defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)











/*****************************************************************************
  Function:
	uint16_t TCPIP_Helper_CalcIPChecksum(uint8_t* buffer, uint16_t count, uint16_t seed)

  Summary:
	Calculates an IP checksum value.

  Description:
	This function calculates an IP checksum over an array of input data.  The
	checksum is the 16-bit one's complement of one's complement sum of all 
	words in the data (with zero-padding if an odd number of bytes are 
	summed).  This checksum is defined in RFC 793.

  Precondition:
	None

  Parameters:
	buffer - pointer to the data to be checksummed
	count  - number of bytes to be checksummed
    seed   - start seed

  Returns:
	The calculated checksum.
	
  Note:
	The checksum is implemented as a fast assembly function on PIC32 platforms.
  ***************************************************************************/
#if !defined(__PIC32__)
uint16_t TCPIP_Helper_CalcIPChecksum(uint8_t* buffer, uint16_t count, uint16_t seed)
{
	uint16_t i;
	uint16_t *val;
	union
	{
		uint8_t  b[4];
		uint16_t w[2];
		uint32_t dw;
	} sum;

	val = (uint16_t*)buffer;

	// Calculate the sum of all words
	sum.dw = (uint32_t)seed;
    if ((unsigned int)buffer % 2)
    {
        sum.w[0] += (*(uint8_t *)buffer) << 8;
        val = (uint16_t *)(buffer + 1);
        count--;
    }

	i = count >> 1;

	while(i--)
		sum.dw += (uint32_t)*val++;

	// Add in the sum of the remaining byte, if present
	if(count & 0x1)
		sum.dw += (uint32_t)*(uint8_t*)val;

	// Do an end-around carry (one's complement arrithmatic)
	sum.dw = (uint32_t)sum.w[0] + (uint32_t)sum.w[1];

	// Do another end-around carry in case if the prior add 
	// caused a carry out
	sum.w[0] += sum.w[1];

    if ((unsigned int)buffer % 2)
    {
        sum.w[0] = ((uint16_t)sum.b[0] << 8 ) | (uint16_t)sum.b[1];
    }

	// Return the resulting checksum
	return ~sum.w[0];
}
#endif  // !defined(__PIC32__)

// calculates the IP checksum for a packet with multiple segments
uint16_t TCPIP_Helper_PacketChecksum(TCPIP_MAC_PACKET* pPkt, uint8_t* startAdd, uint16_t len, uint16_t seed)
{
    TCPIP_MAC_DATA_SEGMENT  *pSeg;
    uint8_t* pChkBuff;
    uint16_t checkLength, chkBytes, nBytes;
    uint16_t segChkSum;
    uint32_t calcChkSum;

    if(len == 0)
    {
        return seed;
    }

    calcChkSum = seed;
    checkLength = len;
    nBytes = 0;
    pChkBuff = startAdd; 
    pSeg = TCPIP_PKT_DataSegmentGet(pPkt, startAdd, true);

    while(pSeg != 0 && checkLength != 0)
    {
        chkBytes = (pSeg->segLoad + pSeg->segSize) - pChkBuff;

        if(chkBytes > pSeg->segLen)
        {
            chkBytes = pSeg->segLen;
        } 

        if(chkBytes > checkLength)
        {
            chkBytes = checkLength;
        } 

        if(chkBytes)
        {
            segChkSum = ~TCPIP_Helper_CalcIPChecksum(pChkBuff, chkBytes, 0);
            if((nBytes & 0x1) != 0)
            {
                segChkSum = TCPIP_Helper_htons(segChkSum);
            }

            checkLength -= chkBytes;
            nBytes += chkBytes;
            calcChkSum += segChkSum;
        }
        pSeg = pSeg->next;
        if(pSeg)
        {
            pChkBuff = pSeg->segLoad;
        }
    }

    return ~TCPIP_Helper_ChecksumFold(calcChkSum);
}

uint16_t TCPIP_Helper_ChecksumFold(uint32_t rawChksum)
{
    TCPIP_UINT32_VAL checksum;

    checksum.Val = rawChksum;
    checksum.Val = (uint32_t)checksum.w[0] + (uint32_t)checksum.w[1];
    checksum.w[0] += checksum.w[1];
    return checksum.w[0];
    
}

// copies packet segment data to a linear destination buffer
// updates the pointer to the current location in the packet segment for further copy
// returns the number of total bytes copied
uint16_t TCPIP_Helper_PacketCopy(TCPIP_MAC_PACKET* pSrcPkt, uint8_t* pDest, uint8_t** pStartAdd, uint16_t len, bool srchTransport)
{
    TCPIP_MAC_DATA_SEGMENT* pSeg;
    uint16_t copyLen, copyBytes;
    uint8_t  *pCopyBuff, *pSrcBuff;
    uint16_t totCopyBytes = 0;

    copyLen = len;
    pCopyBuff = pSrcBuff = *pStartAdd; 
    pSeg = TCPIP_PKT_DataSegmentGet(pSrcPkt, pSrcBuff, srchTransport);

    while(pSeg != 0 && copyLen != 0)
    {
        copyBytes = (pSeg->segLoad + pSeg->segSize) - pCopyBuff;

        if(copyBytes > pSeg->segLen)
        {
            copyBytes = pSeg->segLen;
        } 

        if(copyBytes > copyLen)
        {
            copyBytes = copyLen;
        } 

        if(copyBytes)
        {
            memcpy(pDest, pCopyBuff, copyBytes);
            pDest += copyBytes;
            copyLen -= copyBytes;
            pSrcBuff = pCopyBuff + copyBytes;
            totCopyBytes += copyBytes;
        }

        pSeg = pSeg->next;
        if(pSeg)
        {
            pCopyBuff = pSeg->segLoad;
        }
    }
    
    *pStartAdd = pSrcBuff;

    return totCopyBytes;
}
  

/*****************************************************************************
  Function:
	void TCPIP_Helper_FormatNetBIOSName(uint8_t Name[])

  Summary:
	Formats a string to a valid NetBIOS name.

  Description:
	This function formats a string to a valid NetBIOS name.  Names will be
	exactly 16 characters, as defined by the NetBIOS spec.  The 16th 
	character will be a 0x00 byte, while the other 15 will be the 
	provided string, padded with spaces as necessary.

  Precondition:
	None

  Parameters:
	Name - the string to format as a NetBIOS name.  This parameter must have
	  at least 16 bytes allocated.

  Returns:
	None
  ***************************************************************************/
void TCPIP_Helper_FormatNetBIOSName(uint8_t Name[])
{
	uint8_t i;

	Name[15] = '\0';
	strupr((char*)Name);
	i = 0;
	while(i < 15u)
	{
		if(Name[i] == '\0')
		{
			while(i < 15u)
			{
				Name[i++] = ' ';
			}
			break;
		}
		i++;
	}
}





/*****************************************************************************
  Function:
	uint8_t TCPIP_Helper_ExtractURLFields(uint8_t *vURL, 
						  PROTOCOLS *protocol, 
						  uint8_t *vUsername, uint16_t *wUsernameLen, 
						  uint8_t *vPassword, uint16_t *wPasswordLen, 
						  uint8_t *vHostname, uint16_t *wHostnameLen, 
						  uint16_t *wPort, 
						  uint8_t *vFilePath, uint16_t *wFilePathLen)

  Summary:
	Extracts all parameters from an URL string (ex: 
	"http://admin:passwd@www.microchip.com:8080/myfile.gif" is split into 
	{PROTOCOL_HTTP, "admin", "passwd", "www.microchip.com", 8080, "/myfile.gif"}.

  Description:
	Extracts all parameters from an URL string (ex: 
	"http://admin:passwd@www.microchip.com:8080/myfile.gif" is split into 
	{PROTOCOL_HTTP, "admin", "passwd", "www.microchip.com", 8080, "/myfile.gif"}.
	
	The URL string can be null terminated, or alternatively could be terminated 
	by a carriage return or line feed.
	
	If the protocol is unrecognized or the protocol is recognized but the URL 
	is malformed, than an error is safely returned.  For more information on 
	URL/URI interpretation see RFC 2396.

  Precondition:
	This function is commented out by default to save code space because 
	it is not used by any current stack features.  However, if you want to use 
	it, go ahead and uncomment it.  It has been tested, so it (should) work 
	correctly.

  Parameters:
	vURL -	Pointer to null terminated URL to decode and extract from.  This 
		parameter is required and needs to have the minimum RFC 1738 components 
		in it (protocol and hostname).
		
	protocol - Optional pointer to a PROTOCOLS enum to retrieve the decoded 
		protocol type.  If this parameter is unneeded, specify a NULL pointer.  
		The protocol is a required part of the URL, so it must always be 
		present.  The protocol also determines what scheme all other parameters 
		are decoded using, so the function will fail if an unrecognized 
		protocol is provided.  The PROTOCOLS enum members show all of the 
		currently supported protocols for this function.
		
		<p>For the example URL provided in the function description, 
		PROTOCOL_HTTP would be returned for this field.
		
	vUsername - Optional pointer to a buffer to write the decoded username 
		portion of the URL.  If the URL does not contain a username or a NULL 
		pointer is supplied, then this field is ignored.

		<p>For the example URL provided in the function description, "admin" 
		would be returned for this field.
		
	wUsernameLen -
		On call\: Optional pointer to a uint16_t specifying the maximum length of 
		the vUsername buffer, including the null terminator character.
		
		<p>Upon return\: If wUsernameLen and vUsername are non-NULL, the 
		*wUsernameLen uint16_t is updated with the actual number of characters 
		written to the vUsername buffer, including the null terminator 
		character.  If vUsername is NULL but wUsernameLen is non-NULL, then no 
		characters are copied, but *wUsernameLen will return the number of 
		characters required to fit the full username string.  If wUsernameLen 
		is NULL, then the username field in the URL, if present, is ignored and 
		the vUsername pointer is not used.
		
		<p>If zero characters were written, this indicates that the URL did not 
		contain a username field.  If one character was written, this indicates 
		that a username field was present, but was a zero character string 
		(ex\: "").
		 
		<p>For the example URL provided in the function description, 6 (0x0006) 
		would be returned for this field.
		
	vPassword - Optional pointer to a buffer to write the decoded password 
		portion of the URL.  If the URL does not contain a password or a NULL 
		pointer is supplied, then this field is ignored.

		<p>For the example URL provided in the function description, "passwd" 
		would be returned for this field.
		
	wPasswordLen -
		On call\: Optional pointer to a uint16_t specifying the maximum length of 
		the vPassword buffer, including the null terminator character.
		
		<p>Upon return\: If wPasswordLen and vPassword are non-NULL, the 
		*wPasswordLen uint16_t is updated with the actual number of characters 
		written to the vPassword buffer, including the null terminator 
		character.  If vPassword is NULL but wPasswordLen is non-NULL, then no 
		characters are copied, but *wPasswordLen will return the number of 
		characters required to fit the full password string.  If wPasswordLen 
		is NULL, then the password field in the URL, if present, is ignored and 
		the vPassword pointer is not used.
		
		<p>If zero characters were written, this indicates that the URL did not 
		contain a password field.  If one character was written, this indicates 
		that a password field was present, but was a zero character string 
		(ex\: "").
		 
		<p>For the example URL provided in the function description, 7 (0x0007) 
		would be returned for this field.
		
	vHostname - Optional pointer to a buffer to write the decoded hostname 
		portion of the URL.  All Internet URLs must contain a hostname or IP 
		address, however, if a NULL pointer is supplied, then this field is 
		ignored.

		<p>For the example URL provided in the function description, 
		"www.microchip.com" would be returned for this field.  If the URL was 
		"http://192.168.0.1", then this field would be returned as 
		"192.168.0.1".	The IP address would not be decoded to a uint32_t (use the 
        TCPIP_Helper_StringToIPAddress() helper function to do this).
		
	wHostnameLen -
		On call\: Optional pointer to a uint16_t specifying the maximum length of 
		the vHostname buffer, including the null terminator character.
		
		<p>Upon return\: If wHostnameLen and vHostname are non-NULL, the 
		*wHostnameLen uint16_t is updated with the actual number of characters 
		written to the vHostname buffer, including the null terminator 
		character.  If vHostname is NULL but wHostnameLen is non-NULL, then no 
		characters are copied, but *wHostnameLen will return the number of 
		characters required to fit the full hostname string.  If wHostnameLen 
		is NULL, then the hostname field in the URL, is ignored and the 
		vHostname pointer is not used.
		
		<p>For the example URL provided in the function description, 
		18 (0x0012) would be returned for this field.  If the URL was 
		"http://192.168.0.1", then this field would be returned as 12 (0x000C).
		
	wPort - Optional pointer to a uint16_t specifying the TCP or UDP port that the 
		server is listening on.  If the port field is absent from the URL, then 
		this parameter will specify the default port for the protocol.  For 
		example, "http://www.microchip.com" would result in 80 being return as 
		the specified port.
		 
		<p>If the wPort pointer is NULL, then the port field in the URL 
		is ignored, if present.
		
	vFilePath - Optional pointer to a buffer to write the decoded file path 
		portion of the URL.  If a NULL pointer is supplied, then this field is 
		ignored.  If a file path is not present in the URL, then "/" will be 
		returned in this field.  

		<p>For the example URL provided in the function description, 
		"/myfile.gif" would be returned for this field.
		
	wFilePathLen -
		On call\: Optional pointer to a uint16_t specifying the maximum length of 
		the vFilePath buffer, including the null terminator character.
		
		<p>Upon return\: If wFilePathLen and vFilePath are non-NULL, the 
		*wFilePathLen uint16_t is updated with the actual number of characters 
		written to the vFilePath buffer, including the null terminator 
		character.  If vFilePath is NULL but wFilePathLen is non-NULL, then no 
		characters are copied, but *wFilePathLen will return the number of 
		characters required to fit the full file path string.  If wFilePathLen 
		is NULL, then the file path field in the URL, if present, is ignored and 
		the vFilePath pointer is not used.
		
		<p>This function always returns "/" if no file path is present, so
		*wFilePathLen will also be at least 2 characters ('/' and null 
		terminator) if the pointer is non-NULL.
	
		<p>For the example URL provided in the function description, 12 (0x000C) 
		would be returned for this field.
		
  Returns:
	Zero on success.  Nonzero indicates an error code.  If a nonzero error code 
	is returned, none of the returned buffers or pointer values should be 
	treated as valid, but some of them may have been written to.  The following 
	are all possible return values.
	<table>
		0   No error
		1   Protocol unknown (additional code needs to be added to 
			 TCPIP_Helper_ExtractURLFields() and the PROTOCOLS enum needs to be updated if 
			 you want to decode URLs of this protocol type.
		2   URL malformed. Illegal or unknown URL format encountered.
		3   Buffer too small.  One of the input buffer sizes is too small to 
			 contain the URL parameter.
	</table>
  ***************************************************************************/
#if 0	
uint8_t TCPIP_Helper_ExtractURLFields(uint8_t *vURL, PROTOCOLS *protocol, uint8_t *vUsername, uint16_t *wUsernameLen, uint8_t *vPassword, uint16_t *wPasswordLen, uint8_t *vHostname, uint16_t *wHostnameLen, uint16_t *wPort, uint8_t *vFilePath, uint16_t *wFilePathLen)
{
	// These two arrays must exactly match up each other and the PROTOCOLS enum 
	// elements.  The protocol name strings must also be specified in all 
	// lowercase.
	static const char * const	vProtocolNames[] = {"http", "https", "mms", "rtsp"};
	static const uint16_t 		wProtocolPorts[] = { 80,     443,     1755,  554};
	uint16_t w, w2;
	uint8_t i, j;
	PROTOCOLS prot;
	uint8_t *temp, *temp2;
	uint16_t wURLLen;
	uint16_t wLocalPort;
	
	
	// Calculate how long this URL is
	wURLLen = strlen((char*)vURL);
	temp = (uint8_t*)strnchr((char*)vURL, wURLLen, '\r');
	if(temp)
		wURLLen = temp - vURL;
	temp = (uint8_t*)strnchr((char*)vURL, wURLLen, '\n');
	if(temp)
		wURLLen = temp - vURL;
	

	// Parse starting protocol field
	// Find out how long the protocol name field is
	temp = (uint8_t*)strnchr((char*)vURL, wURLLen, ':');
	if(temp == NULL)
		return 2;
	
	// Search protocol list to see if this is a recognized protocol
	for(prot = 0; (uint8_t)prot < sizeof(wProtocolPorts)/sizeof(wProtocolPorts[0]); prot++)
	{
		w = strlen(vProtocolNames[prot]);
		if((uint16_t)(temp - vURL) == w)
		{
			w2 = 0;
			temp2 = vURL;
			while(w)
			{
				i = *temp2++;
				if((i >= 'A') && (i <= 'Z'))
					i += 'a' - 'A';
				if(i != (uint8_t)vProtocolNames[prot][w2++])
					break;
				w--;
			}
			if(w == 0u)
			{
				if(protocol)
					*protocol = prot;
				break;
			}
		}
	}

	// If we've search the whole list and didn't find a match, then 
	// this protocol is unknown and this URL cannot be parsed.
	if((uint8_t)prot >= sizeof(wProtocolPorts)/sizeof(wProtocolPorts[0]))
		return 1;
	
	w = temp - vURL + 1;
	vURL += w;
	wURLLen -= w;

	// Protocols using the authority field all must have a double 
	// slash "//" prefix
	if(wURLLen < 2u)
		return 2;
	for(j = 0; j < 2u; j++)
	{
		i = *vURL++;
		if(i != '/')
			return 2;
	}
	wURLLen -= 2;
	

	// Parse username and password fields
	// See if there is a @ sign, indicating that there is at 
	// least a username and possibly a password in this URL
	temp = (uint8_t*)strnchr((char*)vURL, wURLLen, '@');
	if(temp == NULL)
	{
		if(wUsernameLen)
			*wUsernameLen = 0;
		if(wPasswordLen)
			*wPasswordLen = 0;
	}
	else
	{
		// If we get down here, there is a user name present, let's 
		// see if a password is also present by searching for a 
		// colon between the current string position and the @ 
		// symbol.
		temp2 = (uint8_t*)strnchr((char*)vURL, temp - vURL, ':');
		
		// Calculate username length and password length, including 
		// null terminator (if the field exists)
		if(temp2 == NULL)
		{
			w = temp - vURL + 1;	// Username
			w2 = 0;					// Password
		}
		else
		{
			w = temp2 - vURL + 1;	// Username
			w2 = temp - temp2;		// Password
		}
		
		if(wUsernameLen)
		{
			if(vUsername)
			{
				if(*wUsernameLen < w)
					return 3;
				memcpy((void*)vUsername, (void*)vURL, w - 1);
				vUsername[w-1] = 0;
			}
			*wUsernameLen = w;
		}
	
		if(wPasswordLen)
		{
			if(vPassword)
			{
				if(*wPasswordLen < w2)
					return 3;
				if(w2)
				{
					memcpy((void*)vPassword, (void*)temp2+1, w2 - 1);
					vPassword[w2-1] = 0;
				}
			}
			*wPasswordLen = w2;
		}
	
		vURL += w;
		wURLLen -= w;
		if(w2)
		{
			vURL += w2;
			wURLLen -= w2;
		}
	}


	// Parse hostname field
	// Find the length of the hostname, including NULL 
	// terminator
	temp = (uint8_t*)strnchr((char*)vURL, wURLLen, ':');
	temp2 = (uint8_t*)strnchr((char*)vURL, wURLLen, '/');
	if(temp && temp2)
	{
		if(temp > temp2)
			temp = NULL;
	}
	if(temp == NULL)
	{
		temp = temp2;
		if(temp2 == NULL)
			temp = vURL + wURLLen;
	}
	w = temp - vURL + 1;
	if(wHostnameLen)
	{
		if(vHostname)
		{
			if(*wHostnameLen < w)
				return 3;
			memcpy((void*)vHostname, (void*)vURL, w - 1);
			vHostname[w-1] = 0;
		}
		*wHostnameLen = w;
	}
	vURL += w - 1;
	wURLLen -= w - 1;


	// Parse port field
	if(*vURL == ':')
	{
		vURL++;
		wURLLen--;
		wLocalPort = 0;
		w = wURLLen;
		temp = (uint8_t*)strnchr((char*)vURL, wURLLen, '/');
		if(temp != NULL)
			w = temp - vURL;
		w2 = w;
		if(wPort)
		{
			while(w--)
			{
				wLocalPort *= 10;
				wLocalPort += *vURL++ - '0';
			}
			*wPort = wLocalPort;
		}
		else
			vURL += w2;
		wURLLen -= w2;
	}
	else if(wPort)
		*wPort = wProtocolPorts[prot];


	// Parse file path field
	if(wFilePathLen)
	{
		w = ++wURLLen;
		if(wURLLen == 1u)
			w = 2;
		if(vFilePath)
		{
			if(*wFilePathLen < w)
				return 3;
			if(wURLLen == 1u)
				vFilePath[0] = '/';
			else
				memcpy((void*)vFilePath, (void*)vURL, wURLLen - 1);
			vFilePath[w - 1] = 0;
			*wFilePathLen = w;
			return 0;
		}
		*wFilePathLen = w;
	}
	return 0;
}
#endif

unsigned char TCPIP_Helper_FindCommonPrefix (unsigned char * addr1, unsigned char * addr2, unsigned char bytes)
{
    unsigned char i = 0;
    unsigned char matchLen = 0;
    unsigned char mask = 0x80;
    unsigned char j, k;

    while (i < bytes)
    {
        j = *addr1;
        k = *addr2;
        if (!(j ^ k))
            matchLen += 8;
        else
        {
            while (mask & ~(j ^ k))
            {
                matchLen++;
                mask >>=1;
            }
            break;
        }
        i++;
        addr1++;
        addr2++;
    }

    return matchLen;
}


// lists implementation: single and double linked

void  TCPIP_Helper_SingleListInitialize(SINGLE_LIST* pL)
{
    pL->head = pL->tail = 0;
    pL->nNodes = 0;
}



void  TCPIP_Helper_SingleListHeadAdd(SINGLE_LIST* pL, SGL_LIST_NODE* pN)
{
	pN->next = pL->head;
	pL->head = pN;
	if(pL->tail == 0)
	{  // empty list
		pL->tail = pN;
	}
    pL->nNodes++;
}

void  TCPIP_Helper_SingleListTailAdd(SINGLE_LIST* pL, SGL_LIST_NODE* pN)
{
	pN->next = 0;
	if(pL->tail == 0)
	{
		pL->head = pL->tail = pN;
	}
	else
	{
		pL->tail->next = pN;
		pL->tail = pN;
	}
    pL->nNodes++;
}


// insertion in the middle, not head or tail
void  TCPIP_Helper_SingleListMidAdd(SINGLE_LIST* pL, SGL_LIST_NODE* pN, SGL_LIST_NODE* after)
{
    pN->next = after->next;
    after->next = pN;
    pL->nNodes++; 
}


SGL_LIST_NODE*  TCPIP_Helper_SingleListHeadRemove(SINGLE_LIST* pL)
{
	SGL_LIST_NODE* pN = pL->head;
    if(pN)
    {
        if(pL->head == pL->tail)
        {
            pL->head = pL->tail = 0;
        }
        else
        {
            pL->head = pN->next;
        }
        pL->nNodes--;
    }

	return pN;
}

// removes the next node (following prev) in the list
// if prev == 0 removed the head
SGL_LIST_NODE*  TCPIP_Helper_SingleListNextRemove(SINGLE_LIST* pL, SGL_LIST_NODE* prev)
{
    SGL_LIST_NODE*  pN;

    if(prev == 0)
    {
        return TCPIP_Helper_SingleListHeadRemove(pL);
    }

    pN = prev->next;
    if(pN)
    {
        prev->next = pN->next;
        if(pN == pL->tail)
        {
            pL->tail = prev;
        }
        pL->nNodes--;
    }

    return pN;


}


// removes a node somewhere in the middle
// Note: this is lengthy!
// Use a double linked list if faster operation needed!



SGL_LIST_NODE*  TCPIP_Helper_SingleListNodeRemove(SINGLE_LIST* pL, SGL_LIST_NODE* pN)
{
    if(pN == pL->head)
    {
        TCPIP_Helper_SingleListHeadRemove(pL);
    }
    else
    {
        SGL_LIST_NODE* prev;
        for(prev = pL->head; prev != 0 && prev->next != pN; prev = prev->next);
        if(prev == 0)
        {   // no such node
            return 0;
        }
        // found it
        prev->next = pN->next;
        if(pN == pL->tail)
        {
            pL->tail = prev;
        }
        pL->nNodes--;
    }

    return pN;
}

bool TCPIP_Helper_SingleListFind(SINGLE_LIST* pL, SGL_LIST_NODE* pN)
{
    SGL_LIST_NODE* node;
    for(node = pL->head; node != 0 ; node = node->next)
    {
        if(node == pN)
        {
            return true;
        }
    }

    return false;
}

void  TCPIP_Helper_SingleListAppend(SINGLE_LIST* pDstL, SINGLE_LIST* pAList)
{
	SGL_LIST_NODE* pN;
	while((pN = TCPIP_Helper_SingleListHeadRemove(pAList)))
	{
		TCPIP_Helper_SingleListTailAdd(pDstL, pN);
	}
}


// Protected Single linked list manipulation

bool  TCPIP_Helper_ProtectedSingleListInitialize(PROTECTED_SINGLE_LIST* pL)
{
    TCPIP_Helper_SingleListInitialize(&pL->list);
    pL->semValid = (OSAL_SEM_Create(&pL->semaphore, OSAL_SEM_TYPE_BINARY, 1, 1) == OSAL_RESULT_TRUE);
    return pL->semValid;
}

void  TCPIP_Helper_ProtectedSingleListDeinitialize(PROTECTED_SINGLE_LIST* pL)
{
    if(pL->semValid)
    {
        if (OSAL_SEM_Pend(&pL->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        TCPIP_Helper_SingleListRemoveAll(&pL->list);
        OSAL_SEM_Delete(&pL->semaphore);
        pL->semValid = false;
    }
}

void  TCPIP_Helper_ProtectedSingleListHeadAdd(PROTECTED_SINGLE_LIST* pL, SGL_LIST_NODE* pN)
{
    if(pL->semValid)
    {
        if (OSAL_SEM_Pend(&pL->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        TCPIP_Helper_SingleListHeadAdd(&pL->list, pN);
        if (OSAL_SEM_Post(&pL->semaphore) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
    }
}

void  TCPIP_Helper_ProtectedSingleListTailAdd(PROTECTED_SINGLE_LIST* pL, SGL_LIST_NODE* pN)
{
    if(pL->semValid)
    {
        if (OSAL_SEM_Pend(&pL->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        TCPIP_Helper_SingleListTailAdd(&pL->list, pN);
        if (OSAL_SEM_Post(&pL->semaphore) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
    }

}


// insertion in the middle, not head or tail
void  TCPIP_Helper_ProtectedSingleListMidAdd(PROTECTED_SINGLE_LIST* pL, SGL_LIST_NODE* pN, SGL_LIST_NODE* after)
{
    if(pL->semValid)
    {
        if (OSAL_SEM_Pend(&pL->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        TCPIP_Helper_SingleListMidAdd(&pL->list, pN, after);
        if (OSAL_SEM_Post(&pL->semaphore) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
    }
}

// removes the head node
SGL_LIST_NODE*  TCPIP_Helper_ProtectedSingleListHeadRemove(PROTECTED_SINGLE_LIST* pL)
{

    if(pL->semValid)
    {
        if (OSAL_SEM_Pend(&pL->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        SGL_LIST_NODE * ret = TCPIP_Helper_SingleListHeadRemove(&pL->list);
        if (OSAL_SEM_Post(&pL->semaphore) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        return ret;
    }

    return 0;

}

// removes the next node (following prev) in the list
// if prev == 0 removed the head
SGL_LIST_NODE*  TCPIP_Helper_ProtectedSingleListNextRemove(PROTECTED_SINGLE_LIST* pL, SGL_LIST_NODE* prev)
{

    if(pL->semValid)
    {
        if (OSAL_SEM_Pend(&pL->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        SGL_LIST_NODE * ret = TCPIP_Helper_SingleListNextRemove(&pL->list, prev);
        if (OSAL_SEM_Post(&pL->semaphore) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        return ret;
    }
    return 0;
}



// removes a node anywhere in the list
// Note: this is lengthy!
// Use a double linked list if faster operation needed!
SGL_LIST_NODE*  TCPIP_Helper_ProtectedSingleListNodeRemove(PROTECTED_SINGLE_LIST* pL, SGL_LIST_NODE* pN)
{

    if(pL->semValid)
    {
        if (OSAL_SEM_Pend(&pL->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        SGL_LIST_NODE * ret = TCPIP_Helper_SingleListNodeRemove(&pL->list, pN);
        if (OSAL_SEM_Post(&pL->semaphore) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        return ret;
    }

    return 0;
}



void  TCPIP_Helper_ProtectedSingleListAppend(PROTECTED_SINGLE_LIST* pDstL, SINGLE_LIST* pAList)
{
    if(pDstL->semValid)
    {
        if (OSAL_SEM_Pend(&pDstL->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        TCPIP_Helper_SingleListAppend(&pDstL->list, pAList);
        if (OSAL_SEM_Post(&pDstL->semaphore) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
    }
}



void TCPIP_Helper_ProtectedSingleListRemoveAll(PROTECTED_SINGLE_LIST* pL)
{
    if(pL->semValid)
    {
        if (OSAL_SEM_Pend(&pL->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        TCPIP_Helper_SingleListRemoveAll(&pL->list);
        if (OSAL_SEM_Post(&pL->semaphore) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
    }
}

bool TCPIP_Helper_ProtectedSingleListLock(PROTECTED_SINGLE_LIST* pL)
{
    if(pL->semValid)
    {
        return (OSAL_SEM_Pend(&pL->semaphore, OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE);
    }

    return false;
}

bool TCPIP_Helper_ProtectedSingleListUnlock(PROTECTED_SINGLE_LIST* pL)
{
    if(pL->semValid)
    {
        return (OSAL_SEM_Post(&pL->semaphore) == OSAL_RESULT_TRUE);
    }

    return false;
}


/////  double linked lists manipulation ///////////
//


void  TCPIP_Helper_DoubleListInitialize(DOUBLE_LIST* pL)
{
    pL->head = pL->tail = 0;
    pL->nNodes = 0;
}


void  TCPIP_Helper_DoubleListHeadAdd(DOUBLE_LIST* pL, DBL_LIST_NODE* pN)
{
	if(pL->head == 0)
	{ // empty list, first node
		pL->head = pL->tail = pN;
		pN->next = pN->prev = 0;
	}
	else
	{
		pN->next = pL->head;
		pN->prev = 0;
		pL->head->prev = pN;
		pL->head = pN;
	}		
    pL->nNodes++;
}

void  TCPIP_Helper_DoubleListTailAdd(DOUBLE_LIST* pL, DBL_LIST_NODE* pN)
{
	if(pL->head == 0)
	{ // empty list, first node
		pL->head = pL->tail = pN;
		pN->next = pN->prev = 0;
	}
	else
	{
		pN->next = 0;
		pN->prev = pL->tail;
		pL->tail->next = pN;
		pL->tail = pN;
	}		
    pL->nNodes++;
}

// add node pN in the middle, after existing node "after"
void  TCPIP_Helper_DoubleListMidAdd(DOUBLE_LIST* pL, DBL_LIST_NODE* pN, DBL_LIST_NODE* after)
{
    pN->next = after->next;
    pN->prev = after;
    after->next->prev = pN;
    after->next = pN;
    pL->nNodes++;
}

DBL_LIST_NODE*  TCPIP_Helper_DoubleListHeadRemove(DOUBLE_LIST* pL)
{
    DBL_LIST_NODE* pN = pL->head;
    if(pN)
    {
        if(pL->head == pL->tail)
        {
            pL->head = pL->tail = 0;
        }
        else
        {
            pL->head = pN->next;
            pL->head->prev = 0;
        }
        pL->nNodes--;
    }
    return pN;
}

DBL_LIST_NODE*  TCPIP_Helper_DoubleListTailRemove(DOUBLE_LIST* pL)
{
    DBL_LIST_NODE* pN = pL->tail;
    if(pN)
    {
        if(pL->head == pL->tail)
        {
            pL->head = pL->tail = 0;
        }
        else
        {
            pL->tail = pN->prev;
            pL->tail->next = 0;
        }
        pL->nNodes--;
    }
    return pN;
}

// remove existing node, neither head, nor tail
void  TCPIP_Helper_DoubleListMidRemove(DOUBLE_LIST* pL, DBL_LIST_NODE* pN)
{
    (pN)->prev->next = (pN)->next;
    (pN)->next->prev = (pN)->prev;
    pL->nNodes--;
}

void  TCPIP_Helper_DoubleListNodeRemove(DOUBLE_LIST* pL, DBL_LIST_NODE* pN)
{
	if(pN == pL->head)
	{
		TCPIP_Helper_DoubleListHeadRemove(pL);
	}
	else if(pN == pL->tail)
	{
		TCPIP_Helper_DoubleListTailRemove(pL);
	}
	else
	{
		TCPIP_Helper_DoubleListMidRemove(pL, pN);
	}
}

bool TCPIP_Helper_DoubleListFind(DOUBLE_LIST* pL, DBL_LIST_NODE* pN)
{
    DBL_LIST_NODE* node;
    for(node = pL->head; node != 0 ; node = node->next)
    {
        if(node == pN)
        {
            return true;
        }
    }

    return false;
}



bool  TCPIP_Helper_ProtectedDoubleListInitialize(PROTECTED_DOUBLE_LIST* pL)
{
    TCPIP_Helper_DoubleListInitialize(&pL->list);
    pL->semValid = (OSAL_SEM_Create(&pL->semaphore, OSAL_SEM_TYPE_BINARY, 1, 1) == OSAL_RESULT_TRUE);
    return pL->semValid;
}

void  TCPIP_Helper_ProtectedDoubleListDeinitialize(PROTECTED_DOUBLE_LIST* pL)
{
    if(pL->semValid)
    {
        if (OSAL_SEM_Pend(&pL->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        TCPIP_Helper_DoubleListRemoveAll(&pL->list);
        OSAL_SEM_Delete(&pL->semaphore);
        pL->semValid = false;
    }
}


void  TCPIP_Helper_ProtectedDoubleListHeadAdd(PROTECTED_DOUBLE_LIST* pL, DBL_LIST_NODE* pN)
{
    if(pL->semValid)
    {
        if (OSAL_SEM_Pend(&pL->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        TCPIP_Helper_DoubleListHeadAdd(&pL->list, pN);
        if (OSAL_SEM_Post(&pL->semaphore) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
    }
}

void  TCPIP_Helper_ProtectedDoubleListTailAdd(PROTECTED_DOUBLE_LIST* pL, DBL_LIST_NODE* pN)
{
    if(pL->semValid)
    {
        if (OSAL_SEM_Pend(&pL->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        TCPIP_Helper_DoubleListTailAdd(&pL->list, pN);
        if (OSAL_SEM_Post(&pL->semaphore) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
    }
}


// insertion in the middle, not head or tail
void  TCPIP_Helper_ProtectedDoubleListMidAdd(PROTECTED_DOUBLE_LIST* pL, DBL_LIST_NODE* pN, DBL_LIST_NODE* after)
{
    if(pL->semValid)
    {
        if (OSAL_SEM_Pend(&pL->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        TCPIP_Helper_DoubleListMidAdd(&pL->list, pN, after);
        if (OSAL_SEM_Post(&pL->semaphore) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
    }
}

// removes the head node
DBL_LIST_NODE*  TCPIP_Helper_ProtectedDoubleListHeadRemove(PROTECTED_DOUBLE_LIST* pL)
{
    if(pL->semValid)
    {
        if (OSAL_SEM_Pend(&pL->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        DBL_LIST_NODE * ret = TCPIP_Helper_DoubleListHeadRemove(&pL->list);
        if (OSAL_SEM_Post(&pL->semaphore) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        return ret;
    }

    return 0;
}

// removes the next node (following prev) in the list
// if prev == 0 removed the head
DBL_LIST_NODE*  TCPIP_Helper_ProtectedDoubleListTailRemove(PROTECTED_DOUBLE_LIST* pL)
{
    if(pL->semValid)
    {
        if (OSAL_SEM_Pend(&pL->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        DBL_LIST_NODE * ret = TCPIP_Helper_DoubleListTailRemove(&pL->list);
        if (OSAL_SEM_Post(&pL->semaphore) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        return ret;
    }

    return 0;
}

void  TCPIP_Helper_ProtectedDoubleListMidRemove(PROTECTED_DOUBLE_LIST* pL, DBL_LIST_NODE* pN)
{
    if(pL->semValid)
    {
        if (OSAL_SEM_Pend(&pL->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        TCPIP_Helper_DoubleListMidRemove(&pL->list, pN);
        if (OSAL_SEM_Post(&pL->semaphore) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
    }
}


void  TCPIP_Helper_ProtectedDoubleListNodeRemove(PROTECTED_DOUBLE_LIST* pL, DBL_LIST_NODE* pN)
{
    if(pL->semValid)
    {
        if (OSAL_SEM_Pend(&pL->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        TCPIP_Helper_DoubleListNodeRemove(&pL->list, pN);
        if (OSAL_SEM_Post(&pL->semaphore) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
    }
}

void TCPIP_Helper_ProtectedDoubleListRemoveAll(PROTECTED_DOUBLE_LIST* pL)
{
    if(pL->semValid)
    {
        if (OSAL_SEM_Pend(&pL->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
        TCPIP_Helper_DoubleListRemoveAll(&pL->list);
        if (OSAL_SEM_Post(&pL->semaphore) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
    }
}

void TCPIP_Helper_ProtectedDoubleListLock(PROTECTED_DOUBLE_LIST* pL)
{
    if(pL->semValid)
    {
        if (OSAL_SEM_Pend(&pL->semaphore, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
    }
}

void TCPIP_Helper_ProtectedDoubleListUnlock(PROTECTED_DOUBLE_LIST* pL)
{
    if(pL->semValid)
    {
        if (OSAL_SEM_Post(&pL->semaphore) != OSAL_RESULT_TRUE)
        {
            //SYS_DEBUG LOG
        }
    }
}


// returns the corresponding port entry in the tcpSecurePortTbl
// if not found, could store the a entry location in the table if needed
static TCPIP_HELPER_PORT_ENTRY* _TCPIP_Helper_SecurePortEntry(uint16_t port, TCPIP_HELPER_PORT_ENTRY** pFreeEntry)
{
    TCPIP_HELPER_PORT_ENTRY* pEntry, *pFree;
    int ix;

    pEntry = tcpSecurePortTbl;
    pFree = 0;
    for (ix = 0; ix < sizeof(tcpSecurePortTbl)/sizeof(*tcpSecurePortTbl); ix++, pEntry++)
    {
        if(pEntry->port == 0)
        {
            if(pFree == 0)
            {
                pFree = pEntry;
            }
        }
        else if(pEntry->port == port)
        {
            return pEntry;
        }
    }

    if(pFreeEntry)
    {
        *pFreeEntry = pFree;
    }

    return 0;

}

bool TCPIP_Helper_TCPSecurePortGet(uint16_t tcpPort)
{
    TCPIP_HELPER_PORT_ENTRY* pEntry = _TCPIP_Helper_SecurePortEntry(tcpPort, 0);

    if(pEntry)
    {   // found port
        return (pEntry->flags & TCPIP_HELPER_PORT_FLAG_STREAM) != 0;
    }

    return false;
}

bool TCPIP_Helper_UDPSecurePortGet(uint16_t udpPort)
{
    TCPIP_HELPER_PORT_ENTRY* pEntry = _TCPIP_Helper_SecurePortEntry(udpPort, 0);

    if(pEntry)
    {   // found port
        return (pEntry->flags & TCPIP_HELPER_PORT_FLAG_DGRAM) != 0;
    }

    return false;
}

uint16_t TCPIP_Helper_SecurePortGetByIndex(int index, bool streamSocket, int* pnIndexes)
{
    TCPIP_HELPER_PORT_FLAGS entryFlags = streamSocket ? TCPIP_HELPER_PORT_FLAG_STREAM : TCPIP_HELPER_PORT_FLAG_DGRAM;

    if(pnIndexes)
    {
        *pnIndexes = sizeof(tcpSecurePortTbl)/sizeof(*tcpSecurePortTbl);
    }

    if(index < sizeof(tcpSecurePortTbl)/sizeof(*tcpSecurePortTbl))
    {
        TCPIP_HELPER_PORT_ENTRY* pEntry = tcpSecurePortTbl + index;
        if(pEntry->port != 0)
        {
            if((pEntry->flags & entryFlags) == entryFlags)
            {
                return pEntry->port;
            }
        }
    }

    return 0;
}



bool TCPIP_Helper_SecurePortSet(uint16_t port, bool streamSocket, bool isSecure)
{
    TCPIP_HELPER_PORT_ENTRY* pEntry, *pFree = 0;

    if(port == 0)
    {   // invalid port
        return false;
    }

    TCPIP_HELPER_PORT_FLAGS entryFlags = streamSocket ? TCPIP_HELPER_PORT_FLAG_STREAM : TCPIP_HELPER_PORT_FLAG_DGRAM;

    pEntry = _TCPIP_Helper_SecurePortEntry(port, &pFree);

    if(isSecure)
    {
        if(pEntry == 0)
        {   // not in there already
            if(pFree == 0)
            {   // no more room
                return false;
            }
            pFree->port = port;
            pFree->flags = entryFlags;
        }
        else
        {   // make sure is flagged properly
            pEntry->flags |= entryFlags;
        }
        return true;
    }

    // non secure port: need to remove
    if(pEntry)
    {
        if((pEntry->flags &= ~entryFlags) == 0)
        {   // done, free it
            pEntry->port = 0;
        }
    }
    // else not even in there

    return true;
}





