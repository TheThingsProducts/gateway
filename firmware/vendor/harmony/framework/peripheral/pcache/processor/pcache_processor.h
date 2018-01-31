#ifndef _PCACHE_PROCESSOR_H
#define _PCACHE_PROCESSOR_H

#if defined(__PIC16F__)
    #include "pcache_pic_other.h"

#elif defined(__18CXX)
    #include "pcache_pic18.h"

#elif defined(_PIC18)
    #include "pcache_pic18.h"

#elif defined(__PIC24F__)
    #include "pcache_p24Fxxxx.h"

#elif defined(__PIC24H__)
    #include "pcache_p24Hxxxx.h"

#elif defined(__dsPIC30F__)
    #include "pcache_p30Fxxxx.h"

#elif defined(__dsPIC33E__)
    #include "pcache_p33Exxxx.h"

#elif defined(__dsPIC33F__)
    #include "pcache_p33Fxxxx.h"

#elif defined(__PIC32MX__)
    #include "pcache_p32xxxx.h"

#elif defined(__PIC32MZ__)
    #include "pcache_p32xxxx.h"
	
#elif defined(__PIC32WK__)
    #include "pcache_p32xxxx.h"

#else
    #error "Can't find header"

#endif

#endif//_PCACHE_PROCESSOR_H
