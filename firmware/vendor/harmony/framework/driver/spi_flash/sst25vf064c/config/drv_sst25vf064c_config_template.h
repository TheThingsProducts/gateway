/*******************************************************************************
 SST25VF064C Driver Configuration Template

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sst25vf064c_config_template.h

  Summary:
    SST25VF064C Driver Configuration Template.

  Description:
    These file provides the list of all the configurations that can be used with
    the driver. This file should not be included in the driver.

*******************************************************************************/
//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_SST25VF064C_CONFIG_TEMPLATE_H
#define _DRV_SST25VF064C_CONFIG_TEMPLATE_H


//DOM-IGNORE-BEGIN
#error "This is a configuration template file.  Do not include it directly."
//DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Core Functionality Configuration Options
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* SST25VF064C mode

  Summary:
    Determines whether the driver is implemented as static or dynamic.

  Description:
    Determines whether the driver is implemented as static or dynamic. Static
    drivers control the peripheral directly with peripheral library routines.

  Remarks:
    None.
*/

#define DRV_SST25VF064C_MODE                              DYNAMIC

// *****************************************************************************
/* SST25VF064C driver objects configuration

  Summary:
    Sets up the maximum number of hardware instances that can be supported.

  Description
    Sets up the maximum number of hardware instances that can be supported.
    It is recommended that this number be set exactly equal to the number of
    SST25VF064C modules that are needed by the application. Hardware Instance
    support consumes RAM memory space.
    If this macro is not defined, then the driver will be built statically.

  Remarks:
    None.
*/

#define DRV_SST25VF064C_INSTANCES_NUMBER                      2


// *****************************************************************************
/* SST25VF064C Client Count Configuration

  Summary:
    Sets up the maximum number of clients that can be connected to any hardware
    instance.

  Description:
    Sets up the maximum number of clients that can be connected to any hardware
    instance. This value represents the total number of clients to be supported
    across all hardware instances. So if SST25VF064C-1 will be accessed by 2 clients
    and SST25VF064C-2 will accessed by 3 clients, then this number should be 5.
    It is recommended that this be set exactly equal to the number of expected
    clients. Client support consumes RAM memory space.
    If this macro is not defined and the DRV_SST25VF064C_INSTANCES_NUMBER macro is not
    defined, then the driver will be built for static - single client operation.
    If this macro is defined and the DRV_SST25VF064C_INSTANCES_NUMBER macro is not
    defined, then the driver will be built for static - multi client operation.

  Remarks:
    None.
*/

#define DRV_SST25VF064C_CLIENTS_NUMBER                      4

// *****************************************************************************
/* SST25VF064C Driver Instance combined queue depth.

  Summary:
    Number of entries of queues in all instances of the driver.

  Description:
    This macro defines the number of entries of all queues in all instances of
    the driver.

    Each hardware instance supports a buffer queue for all the read/write/erase
    operations. The size of queue is specified either in driver
    initialization (for dynamic build) or by macros (for static build).

    A buffer queue will contain buffer queue entries, each related to a
    BufferAdd request.  This configuration macro defines total number of buffer
    entries that will be available for use between all SST25VF064C driver hardware
    instances. The buffer queue entries are allocated to individual hardware
    instances as requested by hardware instances. Once the request is processed,
    the buffer queue entry is free for use by other hardware instances.

    The total number of buffer entries in the system determines the ability of
    the driver to service non blocking erase/write/read requests. If a free buffer
    entry is not available, the driver will not add the request and will return
    an invalid buffer handle. More the number of buffer entries, greater the
    ability of the driver to service and add requests to its queue. A hardware
    instance additionally can queue up as many buffer entries as  specified by
    its buffer queue size.

*/

#define  DRV_SST25VF064C_QUEUE_DEPTH_COMBINED      7

// *****************************************************************************
/* SST25VF064C Hardware Write Protect Support

  Summary:
    Specifies whether or not the hardware write protect feature is enabled.

  Description:
    This macro defines whether or not the hardware Write Protect feature is enabled.
    If hardware write protection is enabled, the user must provide a port pin
    corresponding to the WP pin on the Flash device.

  Remarks:
    None.
*/

#define DRV_SST25VF064C_HARDWARE_WRITE_PROTECTION_ENABLE    false

// *****************************************************************************
/* SST25VF064C Hardware HOLD Support

  Summary
    Specifies whether or not the hardware hold feature is enabled.

  Description:
    This macro defines whether or not the hardware hold feature is enabled.
    If hardware hold is enabled, the user must provide a port pin corresponding
    to the HOLD pin on the Flash device.

  Remarks:
    None.
*/

#define DRV_SST25VF064C_HARDWARE_HOLD_ENABLE    false

#endif // #ifndef _DRV_SST25VF064C_CONFIG_TEMPLATE_H

/*******************************************************************************
 End of File
*/
