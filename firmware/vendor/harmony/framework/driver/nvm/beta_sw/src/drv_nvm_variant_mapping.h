/*******************************************************************************
  NVM Driver Feature Variant Implementations

  Company:
    Microchip Technology Inc.

  File Name:
    drv_nvm_variant_mapping.h

  Summary:
    NVM Driver Feature Variant Implementations

  Description:
    This file implements the functions which differ based on different parts
    and various implementations of the same feature.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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
//DOM-IGNORE-END


#ifndef _DRV_NVM_VARIANT_MAPPING_H
#define _DRV_NVM_VARIANT_MAPPING_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#if (true == DRV_NVM_UNIFIED)
    #if !defined(DRV_NVM_INSTANCES_NUMBER)

        /* Map internal macros and functions to the static single open variant */
        #include "driver/nvm/src/static/drv_nvm_hw_static.h"
        #include "driver/nvm/src/client_single/drv_nvm_client_single.h"

    #else // (DRV_NVM_INSTANCES_NUMBER > 1)

        /* Map internal macros and functions to the dynamic variant */
        #include "driver/nvm/src/dynamic/drv_nvm_hw_dynamic.h"
        #include "driver/nvm/src/client_multi/drv_nvm_client_multi.h"

    #endif

#endif


// *****************************************************************************
// *****************************************************************************
// Section: SPI Driver Static Object Generation
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro: _DRV_NVM_OBJ_MAKE_NAME(name)

  Summary:
    Creates an instance-specific static object name

  Description:
     This macro creates the instance-specific name of the given static object
     by inserting the index number into the name.

  Remarks:
    This macro does not affect the dynamic objects
*/

#define _DRV_STATIC_OBJ_NAME_B(name,id)     name ## id

#define _DRV_STATIC_OBJ_NAME_A(name,id)     _DRV_STATIC_OBJ_NAME_B(name,id)

#define _DRV_NVM_OBJ_MAKE_NAME(name)        _DRV_STATIC_OBJ_NAME_A(name, DRV_NVM_CONFIG_INDEX)


// *****************************************************************************
// *****************************************************************************
// Section: Symbolic Constants
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Feature Variant Mapping
// *****************************************************************************
// *****************************************************************************
/* Some variants are determined by hardware feature existence, some features
   are determined user configuration of the driver, and some variants are
   combination of the two.
*/


// *****************************************************************************
/* PLIB ID Static Configuration Override

  Summary:
    Allows static override of the peripehral library ID

  Description:
    These macros allow the peripheral library ID to be statically overriden by 
    the DRV_NVM_PERIPHERAL_ID configuration macro, if it is defined.
    
    _DRV_NVM_PERIPHERAL_ID_GET replaces the value passed in with the value defined by
    the DRV_NVM_PERIPHERAL_ID configuration option.
    
    _DRV_NVM_STATIC_PLIB_ID removes any statement passed into it from the build
    if the DRV_NVM_PERIPHERAL_ID configuration option is defined.
*/

#if defined(DRV_NVM_PERIPHERAL_ID)

    #define _DRV_NVM_PERIPHERAL_ID_GET(plibId)      DRV_NVM_PERIPHERAL_ID
    #define _DRV_NVM_STATIC_PLIB_ID(any)

#else

    #define _DRV_NVM_PERIPHERAL_ID_GET(plibId)      plibId
    #define _DRV_NVM_STATIC_PLIB_ID(any)            any

#endif


// *****************************************************************************
/* Interrupt Source Static Configuration Override

  Summary:
    Allows static override of the interrupt source

  Description:
    These macros allow the interrupt source to be statically overriden by the 
    DRV_NVM_INTERRUPT_SOURCE configuration macro, if it is defined.
    
    _DRV_NVM_GET_INT_SRC replaces the value passed in with the value defined by
    the DRV_NVM_INTERRUPT_SOURCE configuration option.
    
    _DRV_NVM_STATIC_INT_SRC removes any statement passed into it from the build
    if the DRV_NVM_INTERRUPT_SOURCE configuration option is defined.
*/

#if defined(DRV_NVM_INTERRUPT_SOURCE)

    #define _DRV_NVM_GET_INT_SRC(source)            DRV_NVM_INTERRUPT_SOURCE
    #define _DRV_NVM_STATIC_INT_SRC(any)

#else

    #define _DRV_NVM_GET_INT_SRC(source)    source
    #define _DRV_NVM_STATIC_INT_SRC(any)    any

#endif


// *****************************************************************************
/* Interrupt Source Control

  Summary:
    Macros to enable, disable or clear the interrupt source

  Description:
    This macro enables, disables or clears the interrupt source

    The macros get mapped to the respective SYS module APIs if the configuration
    option DRV_NVM_INTERRUPT_MODE is set to true
 
  Remarks:
    This macro is mandatory
*/

#if defined (DRV_NVM_INTERRUPT_MODE)

    #if(DRV_NVM_INTERRUPT_MODE == true)

        #define _DRV_NVM_InterruptSourceEnable(source)          SYS_INT_SourceEnable( source )
        #define _DRV_NVM_InterruptSourceDisable(source)         SYS_INT_SourceDisable( source )
        #define _DRV_NVM_InterruptSourceClear(source)           SYS_INT_SourceStatusClear( source )

        #define _DRV_NVM_InterruptSourceStatusGet(source)       SYS_INT_SourceStatusGet( source )

    #elif (DRV_NVM_INTERRUPT_MODE == false)

        #define _DRV_NVM_InterruptSourceEnable(source)
        #define _DRV_NVM_InterruptSourceDisable(source)         false
        #define _DRV_NVM_InterruptSourceClear(source)           SYS_INT_SourceStatusClear( source )

        #define _DRV_NVM_InterruptSourceStatusGet(source)       SYS_INT_SourceStatusGet( source )

    #else

        #error "No Task mode chosen at build, interrupt or polling needs to be selected. "

    #endif

#else

    #warning "No Task mode chosen at build, interrupt or polling needs to be selected. "

#endif

#if defined (__PIC32MX__)
    #define _DRV_NVM_UnlockKeySequence0(x,y)
#elif defined (__PIC32MZ__)
    #define _DRV_NVM_UnlockKeySequence0(x,y)     PLIB_NVM_FlashWriteKeySequence( x, y)
#endif

#if defined (DRV_NVM_ERASE_WRITE_ENABLE)
    #define DRV_NVM_ERASE_BUFFER_SIZE DRV_NVM_PAGE_SIZE
#else
    #define DRV_NVM_ERASE_BUFFER_SIZE 0
#endif

#define _DRV_NVM_KVA_TO_PA(address)                             (address & 0x1FFFFFFF)

#endif //_DRV_NVM_VARIANT_MAPPING_H

/*******************************************************************************
 End of File
*/

