#ifndef _PLIB_SPI_PROCESSOR_H
#define _PLIB_SPI_PROCESSOR_H

#if defined(__PIC16F__)
    #include "spi_pic_other.h"

#elif defined(__18CXX)
    #include "spi_pic18.h"

#elif defined(_PIC18)
    #include "spi_pic18.h"

#elif defined(__PIC24F__)
    #include "spi_p24Fxxxx.h"

#elif defined(__PIC24H__)
    #include "spi_p24Hxxxx.h"

#elif defined(__dsPIC30F__)
    #include "spi_p30Fxxxx.h"

#elif defined(__dsPIC33E__)
    #include "spi_p33Exxxx.h"

#elif defined(__dsPIC33F__)
    #include "spi_p33Fxxxx.h"

#elif defined(__PIC32MX__)
    #include "spi_p32xxxx.h"

#elif defined(__PIC32MZ__)
    #include "spi_p32xxxx.h"
	
#elif defined(__PIC32WK__)
    #include "spi_p32xxxx.h"

#else
    #error "Can't find header"

#endif

#endif//_PLIB_SPI_PROCESSOR_H
