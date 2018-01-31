/*******************************************************************************
  SST25VF064C Driver Variant Mapping

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sst25vf064c_variant_mapping.h

  Summary:
    SST25VF064C Driver Variant Mapping

  Description:
    This file provides feature and build variant mapping macros allowing the
    driver to easily be built with different implementation variations based
    on static build-time configuration selections.
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

#ifndef _DRV_SST25VF064C_FEATURE_MAPPING_H
#define _DRV_SST25VF064C_FEATURE_MAPPING_H


// *****************************************************************************
// *****************************************************************************
// Section: Hardware Write Protection Variations
// *****************************************************************************
// *****************************************************************************
/* Mapping of hardware write protection variations
*/

#if (DRV_SST25VF064C_HARDWARE_WRITE_PROTECTION_ENABLE == true)

    /* If hardware write protection is enabled, then user must
       provide a port pin in the initilization structure corresponding
       to WP pin on the flash */

    #define _DRV_SST25VF064C_HardwareWPIsEnabled()      true
#else
    #define _DRV_SST25VF064C_HardwareWPIsEnabled()      false

#endif

// *****************************************************************************
// *****************************************************************************
// Section: Hardware HOLD Feature Variations
// *****************************************************************************
// *****************************************************************************
/* Mapping of hardware HOLD feature variations
*/
#if (DRV_SST25VF064C_HARDWARE_HOLD_ENABLE == true)

    /* If hardware HOLD is enabled, then user must provide a port pin
       in the initilization structure corresponding to HOLD pin on the flash */

    #define _DRV_SST25VF064C_HardwareHoldIsEnabled()    true
#else
    #define _DRV_SST25VF064C_HardwareHoldIsEnabled()    false

#endif


// *****************************************************************************
// *****************************************************************************
// Initializtion Parameter Static Overrides
// *****************************************************************************
// *****************************************************************************



#endif //_DRV_SST25VF064C_FEATURE_MAPPING_H

/*******************************************************************************
 End of File
*/
