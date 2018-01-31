/*******************************************************************************
  NVM Media Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_nvm_media.h

  Summary:
    NVM Media Driver Interface Definition

  Description:
    The NVM Media device driver provides a simple interface to access the NVM
    as a media. It provides a sector read and sector write interface.
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


#ifndef _DRV_NVM_MEDIA_H_
#define _DRV_NVM_MEDIA_H_

#include "system_config.h"
#include "system/fs/sys_fs_media_manager.h"
// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Driver NVM Module Index reference

  Summary:
    NVM Media Driver index definitions

  Description:
    These constants provide NVM Media Driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the DRV_NVM_MEDIA_Initialize and  
    DRV_NVM_Media_Open routines to identify the driver instance in use.
*/

#define DRV_NVM_MEDIA_INDEX_0 	    0
#define DRV_NVM_MEDIA_INDEX_1	    1
#define DRV_NVM_MEDIA_INDEX_2	    2
#define DRV_NVM_MEDIA_INDEX_3	    3

// *****************************************************************************
/* NVM Media Driver Initialization Data

  Summary:
    Defines the data required to initialize or reinitialize the NVM Media driver

  Description:
    This data type defines the data required to initialize or reinitialize the
    NVM Media driver.

  Remarks:
    None.
*/

typedef struct _DRV_NVM_MEDIA_INIT
{
    /* Start address of the Media disk. This address
     * should be page (DRV_NVM_PAGE_SIZE) aligned. */ 
    uintptr_t   mediaStartAddress;

    /* Total number of sectors in this disk */
    uint32_t    nSectors;

    /* Size of a sector in bytes */
    uint16_t    sectorSizeInBytes;

    /* The NVM Driver index associated with this
     * instance of the media driver */
    SYS_MODULE_INDEX drvNVMIndex;

}
DRV_NVM_MEDIA_INIT;

typedef struct _DRV_NVM_MEDIA_PROPERTIES
{
    /* Start address of the Media disk. This address
     * should be page (DRV_NVM_PAGE_SIZE) aligned. */
    uintptr_t   mediaStartAddress;

    /* Media Size */
    uint32_t    mediaSize;

    /* Media Partitions */
    uint32_t    mediaPartitions;

    /* Size of a sector in bytes */
    uint16_t    mediaPartitionSizeInBytes;

    /* Disk Number */
    uint32_t    diskNo;

} NVM_MEDIA_PROPERTIES;

// *****************************************************************************
// *****************************************************************************
// Section: NVM Driver Module Interface Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_NVM_MEDIA_Initialize( const SYS_MODULE_INDEX index,
                                           const SYS_MODULE_INIT * const init )

  Summary:
    Initializes the NVM Media Driver instance for the specified driver index

  Description:
    This routine initializes the NVM Media driver instance for the specified driver
    index, making it ready for clients to open and use it.

  Precondition:
    None.

  Input:
    index  - Identifier for the instance to be initialized. 

    init   - Pointer to a data structure containing any data necessary to
             initialize the driver.

  Return:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, it returns SYS_MODULE_OBJ_INVALID.

  Example:
    <code>
    // This code snippet shows an example
    // of initializing the NVM Media Driver.
    // The example create a 16 sector disk
    // starting at location masterBootRecord.
    // The disk number to be reported to the
    // SYS FS Media Manager is DISK0.

    DRV_NVM_MEDIA_INIT    mediaDriverInit;
    SYS_MODULE_OBJ  objectHandle;

    mediaDriverInit.mediaStartAddress = (uintptr_t)masterBootRecord,
    mediaDriverInit.nSectors = 16,
    mediaDriverInit.sectorSizeInBytes = 512,
    mediaDriverInit.drvNVMIndex = DRV_NVM_INDEX_0,
    mediaDriverInit.diskIndex = DISK0

	//usage of DRV_NVM_INDEX_0 indicates usage of Flash-related APIs
	objectHandle = DRV_NVM_MEDIA_Initialize(DRV_NVM_MEDIA_INDEX_0, 
                            (SYS_MODULE_INIT*)mediaDriverInit);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>
  Remarks:
    This routine must be called before any other NVM Media Driver routine 
    is called.

    This routine should only be called once during system initialization.
*/


SYS_MODULE_OBJ DRV_NVM_MEDIA_Initialize(SYS_MODULE_INDEX index, 
        SYS_MODULE_INIT * initData);

// *****************************************************************************
// *****************************************************************************
// Section: The following NVM Media Driver function are invoked exclusively
// by the Media Manager and should not be invoked directly by the application.
// *****************************************************************************
// *****************************************************************************

DRV_HANDLE DRV_NVM_MEDIA_Open(SYS_MODULE_INDEX index, 
        const DRV_IO_INTENT ioIntent);
SYS_FS_MEDIA_STATUS DRV_NVM_MEDIA_MediaStatusGet(DRV_HANDLE handle);
SYS_FS_MEDIA_BUFFER_HANDLE  DRV_NVM_MEDIA_SectorRead(DRV_HANDLE   handle,  uint8_t *buffer, 
							uint32_t sectorStart,uint32_t noOfSectors);
SYS_FS_MEDIA_BUFFER_HANDLE  DRV_NVM_MEDIA_Read ( DRV_HANDLE   handle,  uint8_t *destination,
        uint8_t *source, uint32_t nBytes );
SYS_FS_MEDIA_BUFFER_STATUS DRV_NVM_MEDIA_BufferStatusGet(DRV_HANDLE handle, 
					SYS_FS_MEDIA_BUFFER_HANDLE bufferHandle);
SYS_FS_MEDIA_BUFFER_HANDLE  DRV_NVM_MEDIA_SectorWrite(DRV_HANDLE   handle, 
					  uint32_t sectorStart, uint8_t *buffer,uint32_t noOfSectors);
uintptr_t   DRV_NVM_MEDIA_AddressGet(DRV_HANDLE handle);
void DRV_NVM_MEDIA_Close(DRV_HANDLE client);
#endif
