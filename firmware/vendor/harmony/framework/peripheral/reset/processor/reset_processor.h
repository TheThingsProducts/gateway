#ifndef _PLIB_RESET_PROCESSOR_H
#define _PLIB_RESET_PROCESSOR_H

#if defined(__PIC16F__)
    #include "reset_pic_other.h"

#elif defined(__18CXX)
    #include "reset_pic18.h"

#elif defined(_PIC18)
    #include "reset_pic18.h"

#elif defined(__PIC24F__)
    #include "reset_p24Fxxxx.h"

#elif defined(__PIC24H__)
    #include "reset_p24Hxxxx.h"

#elif defined(__dsPIC30F__)
    #include "reset_p30Fxxxx.h"

#elif defined(__dsPIC33E__)
    #include "reset_p33Exxxx.h"

#elif defined(__dsPIC33F__)
    #include "reset_p33Fxxxx.h"

#elif defined(__PIC32MX__)
    #include "reset_p32xxxx.h"

#elif defined(__PIC32MZ__)
    #include "reset_p32xxxx.h"
	
#elif defined(__PIC32WK__)
    #include "reset_p32xxxx.h"


#else
    #error "Can't find header"

#endif

#endif//_PLIB_RESET_PROCESSOR_H
