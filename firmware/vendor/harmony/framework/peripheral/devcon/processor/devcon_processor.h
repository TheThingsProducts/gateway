#ifndef _DEVCON_PROCESSOR_H
#define _DEVCON_PROCESSOR_H

#if defined(__PIC16F__)
    #include "devcon_pic_other.h"

#elif defined(__18CXX)
    #include "devcon_pic18.h"

#elif defined(_PIC18)
    #include "devcon_pic18.h"

#elif defined(__PIC24F__)
    #include "devcon_p24fxxxx.h"

#elif defined(__PIC24H__)
    #include "devcon_p24hxxxx.h"

#elif defined(__dsPIC30F__)
    #include "devcon_p30fxxxx.h"

#elif defined(__dsPIC33E__)
    #include "devcon_p33exxxx.h"

#elif defined(__dsPIC33F__)
    #include "devcon_p33fxxxx.h"

#elif defined(__PIC32MX__)
    #include "devcon_p32xxxx.h"

#elif defined(__PIC32MZ__)
    #include "devcon_p32xxxx.h"

#elif defined(__PIC32WK__)
    #include "devcon_p32xxxx.h"
	
#else
    #error "Can't find header"

#endif

#endif//_DEVCON_PROCESSOR_H
