/*******************************************************************************
  SDCARD Device Driver interface names mapping

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sdcard_mapping.h

  Summary:
    SDCARD Device Driver Interface names mapping

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

#ifndef _DRV_SDCARD_MAPPING_H
#define _DRV_SDCARD_MAPPING_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
/* Note:  See the bottom of file for implementation header include files.
*/

#include <stdint.h>
#include <stdbool.h>

#include "system_config.h"


// *****************************************************************************
// *****************************************************************************
// Section: Build Parameter Checking
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro: DRV_SDCARD_INSTANCES_NUMBER Check

  Summary:
    Checks the DRV_SDCARD_INSTANCES_NUMBER definition

  Description:
    If DRV_SDCARD_INSTANCES_NUMBER is greater than the number of
    SDCARD instances available on the part, an error is generated.

  Remarks:
    The _SDCARD_EXISTS is a processor-specific value defined by the processor
    headers in the PLIB.

    If the configuration does not specify the number of driver instances to
    allocate it defaults to then number of SDCARD instances on the part.
*/

#if defined(DRV_SDCARD_INSTANCES_NUMBER)

    #if (DRV_SDCARD_INSTANCES_NUMBER > SDCARD_MAX_LIMIT )

        #error "The number of SDCARD instances configured is more than the available SDCARDs on the part"

    #endif

#endif


// *****************************************************************************
// *****************************************************************************
// Section: SDCARD Driver API Name Generation
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Macro: _DRV_SDCARD_MAKE_NAME(name)

  Summary:
    Creates an instance-specific interface name

  Description:
    This macro creates the instance-specific name of the given interface
    routine by inserting the index number into the name.

  Remarks:
    None.
*/


#if defined(DRV_SDCARD_INSTANCES_NUMBER)
    // Dynamic Interface Name Generation
    #define _DRV_SDCARD_MAKE_NAME(name)                         DRV_SDCARD_ ## name
#endif

#endif // #ifndef _DRV_SDCARD_MAPPING_H

/*******************************************************************************
 End of File
*/

