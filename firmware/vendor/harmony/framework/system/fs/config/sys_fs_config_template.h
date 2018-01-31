/*******************************************************************************
  File System Service Configuration Templates

  Company:
    Microchip Technology Inc.

  File Name:
    sys_fs_config_template.h

  Summary:
    File System Service configuration templates.

  Description:
    This file contains constants to configure the File System Service.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _SYS_FS_CONFIG_TEMPLATE_H
#define _SYS_FS_CONFIG_TEMPLATE_H


// *****************************************************************************
/* Number of media used in the application

  Summary:
    Number of media used in the application

  Description:
    Number of media used in the application

  Remarks:
    None.
*/

#define SYS_FS_MEDIA_NUMBER                                       1


// *****************************************************************************
/* Number of Volumes

  Summary:
    This macro defines number of volumes used in the application

  Description:
    This macro defines the number of volumes used in the application


  Remarks:
    None.
*/
#define SYS_FS_VOLUME_NUMBER                                 1


// *****************************************************************************
/* Number of simultaneous files access

  Summary:
    Number of simultaneous files access

  Description:
    Number of simultaneous files access


  Remarks:
    None.
*/
#define SYS_FS_MAX_FILES                                 1

// *****************************************************************************
/* Media Sector Size information

  Summary:
    Media Sector Size information

  Description:
    Media Sector Size information


  Remarks:
    None.
*/
#define SYS_FS_MEDIA_MAX_BLOCK_SIZE                                 512

// *****************************************************************************
/* Number of File system types

  Summary:
    Number of file systems used in the application

  Description:
    Number of fil systems used in the application

  Remarks:
    None.
*/
#define SYS_FS_MAX_FILE_SYSTEM_TYPE                                 1
// *****************************************************************************
/* Enable/Disable Auto Mount Feature of File system

  Summary:
    Enable/Disable Auto Mount Feature of File system

  Description:
    Enable/Disable Auto Mount Feature of File system

  Remarks:
    None.
*/
#define SYS_FS_AUTOMOUNT_ENABLE                                 1

#endif //_SYS_DEVCON_CONFIG_TEMPLATE_H

/*******************************************************************************
 End of File
*/

