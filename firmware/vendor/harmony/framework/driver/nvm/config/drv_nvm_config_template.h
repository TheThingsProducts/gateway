/******************************************************************************
  NVM Driver Configuration Template Header file.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_nvm_config_template.h

  Summary:
    NVM driver configuration definitions.

  Description:
    This template file describes all the mandatory and optional configuration
    macros that are needed for building the NVM driver. Do not include this file
    in source code.
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

#ifndef _DRV_NVM_CONFIG_TEMPLATE_H
#define _DRV_NVM_CONFIG_TEMPLATE_H

#error "This is a configuration template file.  Do not include it directly."

// *****************************************************************************
/* NVM Driver instance configuration

  Summary:
    Selects the maximum number of Driver instances that can be supported by
    the dynamic driver.

  Description:
    This definition selects the maximum number of Driver instances that can be
    supported by the dynamic driver. In case of this driver, multiple instances
    of the driver could use the same hardware instance.

  Remarks:
    This macro is mandatory when building the driver for dynamic operation.
*/

#define DRV_NVM_INSTANCES_NUMBER                        			1

// *****************************************************************************
/* NVM maximum number of clients

  Summary:
    Selects the maximum number of clients

  Description:
    This definition selects the maximum number of clients that the NVM driver
    can supported at run time. This constant defines the total number of NVM
    driver clients that will be available to all instances of the NVM driver.

  Remarks:
    This macro is mandatory when building the driver for dynamic operation.
*/

#define DRV_NVM_CLIENTS_NUMBER                          			1

// *****************************************************************************
/* NVM Driver maximum number of buffer objects

  Summary:
    Selects the maximum number of buffer objects

  Description:
    This definition selects the maximum number of buffer objects. This
    indirectly also specifies the queue depth. The NVM Driver can queue up
    DRV_NVM_BUFFER_OBJECT_NUMBER of read/write/erase requests before return a
    DRV_NVM_BUFFER_HANDLE_INVALID due to the queue being full. Buffer objects
    are shared by all instances of the driver. Increasing this number increases
    the RAM requirement of the driver.

  Remarks:
    This macro is mandatory when building the driver for dynamic operation.
*/

#define DRV_NVM_BUFFER_OBJECT_NUMBER                          		5

// *****************************************************************************
/* NVM interrupt and polled mode operation control

  Summary:
    Macro specifies operation of the driver to be in the interrupt mode
    or polled mode

  Description:
    This macro specifies operation of the driver to be in the interrupt mode
    or polled mode

    - true  - Select if interrupt mode of NVM operation is desired
    - false - Select if polling mode of NVM operation is desired

    Not defining this option to true or false will result in build error.

  Remarks:
    This macro is mandatory when building the driver for dynamic operation.
*/

#define DRV_NVM_INTERRUPT_MODE                          			true

// *****************************************************************************
/* NVM Media Start Address

  Summary:
    Specifies the NVM Media start address.

  Description:
    This definition specifies the NVM Media Start address parameter.

  Remarks:
    This macro is mandatory when building the driver for dynamic operation.
*/

#define DRV_NVM_MEDIA_START_ADDRESS                                 0x9D010000

// *****************************************************************************
/* NVM Media Size

  Summary:
    Specifies the NVM Media size.

  Description:
    This definition specifies the NVM Media Size to be used. The size is specified
    in number of Kilo Bytes. The media size MUST never exceed physical available
    NVM Memory size. Application code requirements should be kept in mind while
    defining this parameter.

  Remarks:
    This macro is mandatory when building the driver for dynamic operation.
*/

#define DRV_NVM_MEDIA_SIZE                                          32

// *****************************************************************************
/* NVM Driver Register with File System

  Summary:
    Register to use with the File system

  Description:
    Specifying this macro enables the NVM driver to register its services with
    the SYS FS.

  Remarks:
    This macro is optional and should be specified only if the NVM driver is
    to be used with the File System.
*/

#define DRV_NVM_SYS_FS_REGISTER

// *****************************************************************************
/* NVM Driver Erase Write Feature Enable

  Summary:
    Enables support for NVM Driver Erase Write Feature.

  Description:
    Specifying this macro enable row erase write feature. If this macro is
    specified, the drv_nvm_erasewrite.c file should be added in the project.
    Support for DRV_NVM_EraseWrite() function then gets enabled.

  Remarks:
    This macro is optional and should be specified only if the
    DRV_NVM_EraseWrite() function is required.
*/

#define DRV_NVM_ERASE_WRITE_ENABLE

// *****************************************************************************
/* NVM Driver Disable Error Checks

  Summary:
    Disables the error checks in the driver.

  Description:
    Specifying this macro disables the error checks in the driver. Error checks like 
    parameter validation, NULL checks etc, will be disabled in the driver in order to
    optimize the code space. 

  Remarks:
    This macro is optional and should be specified only if
    code space is a constraint.
*/

#define DRV_NVM_DISABLE_ERROR_CHECK

#endif // #ifndef _DRV_NVM_CONFIG_TEMPLATE_H

/*******************************************************************************
 End of File
*/

