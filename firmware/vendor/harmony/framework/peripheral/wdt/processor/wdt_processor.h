#ifndef _PLIB_WDT_PROCESSOR_H
#define _PLIB_WDT_PROCESSOR_H

#if defined(__PIC16F__)
    #include "wdt_pic_other.h"

#elif defined(__18CXX)
    #include "wdt_pic18.h"

#elif defined(_PIC18)
    #include "wdt_pic18.h"

#elif defined(__PIC24F__)
    #include "wdt_p24fxxxx.h"

#elif defined(__PIC24H__)
    #include "wdt_p24hxxxx.h"

#elif defined(__dwdtC30F__)
    #include "wdt_p30fxxxx.h"

#elif defined(__dwdtC33E__)
    #include "wdt_p33exxxx.h"

#elif defined(__dwdtC33F__)
    #include "wdt_p33fxxxx.h"

#elif defined(__PIC32MX__)
    #include "wdt_p32xxxx.h"

#elif defined(__PIC32MZ__)
    #include "wdt_p32xxxx.h"

#elif defined(__PIC32WK__)
    #include "wdt_p32xxxx.h"

#endif

#endif//_PLIB_WDT_PROCESSOR_H
