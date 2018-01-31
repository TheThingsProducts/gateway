#ifndef _ETH_PROCESSOR_H
#define _ETH_PROCESSOR_H

#if defined(__PIC16F__)
    #include "eth_pic_other.h"

#elif defined(__18CXX)
    #include "eth_pic18.h"

#elif defined(_PIC18)
    #include "eth_pic18.h"

#elif defined(__PIC24F__)
    #include "eth_p24fxxxx.h"

#elif defined(__PIC24H__)
    #include "eth_p24hxxxx.h"

#elif defined(__dsPIC30F__)
    #include "eth_p30fxxxx.h"

#elif defined(__dsPIC33E__)
    #include "eth_p33exxxx.h"

#elif defined(__dsPIC33F__)
    #include "eth_p33fxxxx.h"

#elif defined(__PIC32MX__)
    #include "eth_p32xxxx.h"

#elif defined(__PIC32MZ__)
    #include "eth_p32xxxx.h"
	
#elif defined(__PIC32WK__)
    #include "eth_p32xxxx.h"

#else
    #error "Can't find header"

#endif

#endif//_ETH_PROCESSOR_H
