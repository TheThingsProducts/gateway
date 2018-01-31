/*******************************************************************************
  SD Card Driver Configuration Definitions for the template version

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sdcard_config_template.h

  Summary:
    SD Card driver configuration definitions template

  Description:
    These definitions statically define the driver's mode of operation.
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

#ifndef _DRV_SDCARD_CONFIG_TEMPLATE_H
#define _DRV_SDCARD_CONFIG_TEMPLATE_H


//#error "This is a configuration template file.  Do not include it directly."


// *****************************************************************************
/* SD Card hardware instance configuration

  Summary:
    Selects the maximum number of hardware instances that can be supported by
    the dynamic driver

  Description:
    This definition selects the maximum number of hardware instances that can be
    supported by the dynamic driver. Not defining it means using a static driver.

  Remarks:
    None
*/

#define DRV_SDCARD_INSTANCES_NUMBER                1


// *****************************************************************************
/* SD Card Maximum Number of Clients

  Summary:
    Selects the miximum number of clients

  Description:
    This definition select the maximum number of clients that the SD Card driver can
    support at run time. Not defining it means using a single client.

  Remarks:
    None.

*/

#define DRV_SDCARD_CLIENTS_NUMBER                1


// *****************************************************************************
/* SD Card Static Index Selection

  Summary:
    SD Card Static Index selection

  Description:
    SD Card Static Index selection for the driver object reference

  Remarks:
    This index is required to make a reference to the driver object
*/

#define DRV_SDCARD_INDEX_MAX                                1


// *****************************************************************************
/* SD Card power state configuration

  Summary:
    Defines an override of the power state of the SD Card driver.

  Description:
    Defines an override of the power state of the SD Card driver.

  Remarks:
    Note: This feature may not be available in the device or the SD Card module
    selected.
*/

#define DRV_SDCARD_POWER_STATE                 SYS_MODULE_POWER_IDLE_STOP

// *****************************************************************************
/* SDCARD Driver Register with File System

  Summary:
    Register to use with the File system

  Description:
    Specifying this macro enables the SDCARD driver to register its services with
    the SYS FS.

  Remarks:
    This macro is optional and should be specified only if the SDCARD driver is
    to be used with the File System.
*/

#define DRV_SDCARD_SYS_FS_REGISTER

// *****************************************************************************
/* SDCARD Driver Enable Write Protect Check

  Summary:
    Enable SD Card write protect check.

  Description:
    Specifying this macro enables the SDCARD driver to check whether the SD card
    is write protected.

  Remarks:
    None
*/

#define DRV_SDCARD_ENABLE_WRITE_PROTECT_CHECK

#endif // #ifndef _DRV_SDCARD_CONFIG_TEMPLATE_H

/*******************************************************************************
 End of File
*/

