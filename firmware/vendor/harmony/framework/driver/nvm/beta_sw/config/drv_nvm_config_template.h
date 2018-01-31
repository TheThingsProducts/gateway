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
/* NVM Media maximum number objects

  Summary:
    Selects the maximum number of NVM media objects

  Description:
    This definition selects the maximum number of NVM media objects. Media
    initialization and media open APIs check for the index passed to make sure it
    is with in the maximum limit, returns an invalid handle in case the index
    is out of bounds.

  Remarks:
    This macro is mandatory when building the media driver for dynamic operation.
*/

#define NVM_MEDIA_OBJECT_NUMBER										1

// *****************************************************************************
/* NVM media start address

  Summary:
    Configures the media start address.

  Description:
    This definition selects the media start address and is used in the NVM media
    driver initialization.

  Remarks:
    This macro is mandatory when building the media driver for dynamic operation.
*/

#define NVM_MEDIA_START_ADDRESS                             		0x9D010000

// *****************************************************************************
/* NVM media size

  Summary:
    Configures the size of the media being accessed.

  Description:
    This definition selects the entire media size and is used in the NVM
    media driver initialization. The media size can never exceed physical
    available NVM memory size. Application code requirements should be kept in
    mind while defining this parameter.
    nSectors = DRV_NVM_MEDIA_SIZE/DRV_NVM_MEDIA_SECTOR_SIZE

  Remarks:
    This macro is mandatory when building the media driver for dynamic operation.
*/

#define NVM_MEDIA_SIZE                             					65536

// *****************************************************************************
/* NVM media sector size

  Summary:
    Configures the sector size of the media being accessed.

  Description:
    This definition selects the sector size. This definition is used in the NVM
    media driver initialization. Sector size reference is carried from File
    System perspective.Sector size for FAT systems in typically 512 KB
    and for MPFS it can be 512/1024/2048 KB

  Remarks:
    This macro is mandatory when building the media driver for dynamic operation.
*/

#define NVM_MEDIA_SECTOR_SIZE                             			512

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
/* NVM Driver Program Row Size.

  Summary:
    Specifies the NVM Driver Program Row Size in bytes.

  Description:
    This definition specifies the NVM Driver Program Row Size in bytes. This
    parameter is device specific and should be obtained from the device specific
    data sheet (Ex: This value is 512 for PIC32MX device variants and 2048 for
    PIC32MZ device variants). The Program Row Size is the minimum block size
    that can be programmed in one program operation.

  Remarks:
    This macro is mandatory when building the driver for dynamic operation.
*/

#define DRV_NVM_ROW_SIZE    										512

// *****************************************************************************
/* NVM Driver Program Page Size.

  Summary:
    Specifies the NVM Driver Program Page Size in bytes.

  Description:
    This definition specifies the NVM Driver Program Page Size in bytes. This
    parameter is device specific and should be obtained from the device specific
    data sheet(Ex: This value is 4096 for PIC32MX device variants and 16384 for
    PIC32MZ device variants).

  Remarks:
    This macro is mandatory when building the driver for dynamic operation.
*/

#define DRV_NVM_PAGE_SIZE    										4096

// *****************************************************************************
/* NVM Driver Program Unlock Key 1

  Summary:
    Specifies the NVM Driver Program Unlock Key 1

  Description:
    This definition specifies the NVM Driver Program Unlock Key 1 parameter is
    device specific and should be obtained from the device specific data sheet.

  Remarks:
    This macro is mandatory when building the driver for dynamic operation.
*/

#define DRV_NVM_UNLOCK_KEY1 										0xAA996655

// *****************************************************************************
/* NVM Driver Program Unlock Key 2

  Summary:
    Specifies the NVM Driver Program Unlock Key 2

  Description:
    This definition specifies the NVM Driver Program Unlock Key 2 parameter is
    device specific and should be obtained from the device specific data sheet.

  Remarks:
    This macro is mandatory when building the driver for dynamic operation.
*/

#define DRV_NVM_UNLOCK_KEY2 										0x556699AA

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


#endif // #ifndef _DRV_NVM_CONFIG_TEMPLATE_H

/*******************************************************************************
 End of File
*/

