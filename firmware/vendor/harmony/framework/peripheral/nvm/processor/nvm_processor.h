#ifndef _PLIB_NVM_PROCESSOR_H
#define _PLIB_NVM_PROCESSOR_H


#if defined(__PIC32MX__)
    #include "nvm_p32xxxx.h"

#elif defined(__PIC32MZ__)
    #include "nvm_p32xxxx.h"
	
#elif defined(__PIC32WK__)
    #include "nvm_p32xxxx.h"

#else
    #error "Can't find header"

#endif

#endif//_PLIB_NVM_PROCESSOR_H
