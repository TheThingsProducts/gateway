/*******************************************************************************
  NVM Driver Media Interface Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_nvm_media.c

  Summary:
    NVM Driver Media Interface Definition

  Description:
    The NVM Driver Media Interface implementation provides a thin API layer to
    acess NVM media through NVM Driver.
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

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************
#include "driver/nvm/beta_sw/src/drv_nvm_media_local.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Structures
// *****************************************************************************
// *****************************************************************************

const SYS_FS_MEDIA_FUNCTIONS nvmMediaFunctions =
{
    .mediaStatusGet     = DRV_NVM_MEDIA_MediaStatusGet,
    .sectorRead         = DRV_NVM_MEDIA_SectorRead,
    .sectorWrite        = DRV_NVM_MEDIA_SectorWrite,
    .bufferStatusGet    = DRV_NVM_MEDIA_BufferStatusGet,
    .Read               = DRV_NVM_MEDIA_Read,
    .addressGet         = DRV_NVM_MEDIA_AddressGet,
    .open               = DRV_NVM_MEDIA_Open,
    .close              = DRV_NVM_MEDIA_Close,
    .tasks              = NULL,
};

// *****************************************************************************
// *****************************************************************************
// Section: Global objects
// *****************************************************************************
// *****************************************************************************

DRV_NVM_MEDIA_OBJECT gDrvNVMMediaObject[DRV_NVM_MEDIA_OBJECT_NUMBER];
uint8_t __attribute__((coherent, aligned(16))) gPagebuffer[DRV_NVM_PAGE_SIZE] = {0};
// *****************************************************************************
// *****************************************************************************
// Section: NVM Driver Media Interface Routines
// *****************************************************************************
// *****************************************************************************

SYS_MODULE_OBJ DRV_NVM_MEDIA_Initialize(SYS_MODULE_INDEX index, SYS_MODULE_INIT * initData)
{
    DRV_NVM_MEDIA_INIT * init = (DRV_NVM_MEDIA_INIT *)initData;
    DRV_NVM_MEDIA_OBJECT * dObj;

    if(index >= DRV_NVM_MEDIA_OBJECT_NUMBER)
    {
        SYS_ASSERT(false, "Invalid Driver NVM Media instance");
        return SYS_MODULE_OBJ_INVALID;
    }

    /* Initialize the NVM media object */
    dObj = &gDrvNVMMediaObject[index];

    dObj->mediaStartAddress = init->mediaStartAddress;
    dObj->nSectors          = init->nSectors;
    dObj->sectorSizeInBytes = init->sectorSizeInBytes;
    dObj->buffer = gPagebuffer;
    /* Try getting an handle to the NVM  driver */

    dObj->drvNVMClientHandle = DRV_NVM_Open(init->drvNVMIndex, DRV_IO_INTENT_READWRITE);
    if(dObj->drvNVMClientHandle == DRV_HANDLE_INVALID)
    {
        SYS_ASSERT(false,"Could not open NVM Driver");
        return SYS_MODULE_OBJ_INVALID;
    }

    dObj->mediaState = SYS_FS_MEDIA_ATTACHED;
    dObj->status = SYS_STATUS_READY;

    /* Call the SYS FS Media Manager Register function */

    if(DRV_NVM_MEDIA_MANAGER_USE)
    {
        dObj->disk = SYS_FS_MEDIA_MANAGER_Register((SYS_MODULE_OBJ)dObj, (SYS_MODULE_INDEX)index, &nvmMediaFunctions, SYS_FS_MEDIA_TYPE_NVM);
    }

    return ((SYS_MODULE_OBJ)dObj);
}

DRV_HANDLE DRV_NVM_MEDIA_Open(SYS_MODULE_INDEX index,
        const DRV_IO_INTENT ioIntent)
{
    if(index >= DRV_NVM_MEDIA_OBJECT_NUMBER)
    {
        SYS_ASSERT(false, "Invalid NVM Media Object");
        return (DRV_HANDLE_INVALID);
    }

    /* Because this is a light layer
     * we treat the driver object as the driver client
     * object */
    return((DRV_HANDLE)(&gDrvNVMMediaObject[index]));
}

void DRV_NVM_MEDIA_Close(DRV_HANDLE client)
{
    /* At this time, this function removes the disk
     * reference that it has */

    DRV_NVM_MEDIA_OBJECT * dObj = (DRV_NVM_MEDIA_OBJECT *)client;
    dObj->disk = -1;

}

SYS_FS_MEDIA_STATUS DRV_NVM_MEDIA_MediaStatusGet(DRV_HANDLE handle)
{
    /* Return the media state */
    return(((DRV_NVM_MEDIA_OBJECT *)handle)->mediaState);
}

uintptr_t DRV_NVM_MEDIA_AddressGet( DRV_HANDLE handle)
{
    DRV_NVM_MEDIA_OBJECT * dObj = (DRV_NVM_MEDIA_OBJECT *)handle;

    return (dObj->mediaStartAddress);
}

SYS_FS_MEDIA_BUFFER_HANDLE  DRV_NVM_MEDIA_SectorRead ( DRV_HANDLE   handle,  uint8_t *buffer,
        uint32_t sectorStart,uint32_t noOfSectors )
{
    DRV_NVM_MEDIA_OBJECT * dObj = (DRV_NVM_MEDIA_OBJECT *)handle;

    uint8_t * address;
    uint32_t size;

    if(sectorStart > dObj->nSectors)
    {
        SYS_ASSERT(false,"The sector start value is out of bound");
        return SYS_FS_MEDIA_BUFFER_HANDLE_INVALID;
    }

    /* Convert the sector address to a NVM address
     * and size to bytes */

    address =  (uint8_t*)(dObj->mediaStartAddress + (sectorStart * dObj->sectorSizeInBytes));
    size = dObj->sectorSizeInBytes * noOfSectors;

    return((SYS_FS_MEDIA_BUFFER_HANDLE) DRV_NVM_Read(dObj->drvNVMClientHandle,
                buffer, address, size));
}

SYS_FS_MEDIA_BUFFER_HANDLE  DRV_NVM_MEDIA_Read ( DRV_HANDLE   handle,  uint8_t *destination,
        uint8_t *source, uint32_t nBytes )
{
    DRV_NVM_MEDIA_OBJECT * dObj = (DRV_NVM_MEDIA_OBJECT *)handle;

    /* Call the NVM Driver Read service */
    return((SYS_FS_MEDIA_BUFFER_HANDLE) DRV_NVM_Read(dObj->drvNVMClientHandle,
                destination, source, nBytes));
}

SYS_FS_MEDIA_BUFFER_HANDLE  DRV_NVM_MEDIA_SectorWrite(DRV_HANDLE   handle,
        uint32_t sectorStart, uint8_t *buffer,uint32_t noOfSectors)
{
    DRV_NVM_MEDIA_OBJECT * dObj = (DRV_NVM_MEDIA_OBJECT *)handle;

    uint8_t * address;
    uint32_t size;
    uint32_t nosOfBytesToReachPageAlign;
    DRV_NVM_BUFFER_HANDLE bufferHandle;
    uint16_t i;

    if(sectorStart > dObj->nSectors)
    {
        SYS_ASSERT(false,"The sector start value is out of bound");
        return SYS_FS_MEDIA_BUFFER_HANDLE_INVALID;
    }

    /* Convert the sector address to a NVM address
     * and size to bytes */
    address =  (uint8_t*)(dObj->mediaStartAddress + (sectorStart * dObj->sectorSizeInBytes));
    size = dObj->sectorSizeInBytes * noOfSectors;

    /* Determine, if the sector address is page aligned.
     * Else, determine the pervious page aligned address.
     * This will make sure that the sector selected will
     * lie in a NVM page, which will be read and then
     * erased, for a subsequent write */

    nosOfBytesToReachPageAlign = ((uint32_t)(address) % DRV_NVM_PAGE_SIZE);

    if(nosOfBytesToReachPageAlign != 0)	// "address" is not page aligned
    {
        /* Following code will make address as page aligned */
        /* In other words, now "address" will point to begin of page
         * which holds the sector we are interested in */
        address = address - nosOfBytesToReachPageAlign;
    }
    /* else	"address" is page aligned, nothing to be done */

    /* Now, read the entire page */
    bufferHandle = DRV_NVM_Read(dObj->drvNVMClientHandle, dObj->buffer, address, DRV_NVM_PAGE_SIZE);
    if(bufferHandle == DRV_NVM_BUFFER_HANDLE_INVALID)
    {
        return (SYS_FS_MEDIA_BUFFER_HANDLE_INVALID);
    }

    /* Now, erase the entire page */
    bufferHandle = DRV_NVM_Erase(dObj->drvNVMClientHandle, address, DRV_NVM_PAGE_SIZE);
    if(bufferHandle != DRV_NVM_BUFFER_HANDLE_INVALID)
    {
        while(DRV_NVM_BufferStatus(dObj->drvNVMClientHandle,bufferHandle) != DRV_NVM_BUFFER_COMPLETED);
    }
    else
    {
        return SYS_FS_MEDIA_BUFFER_HANDLE_INVALID;
    }

    /* Now, modify the buffer holding the data of entire page,
     * by adding the new data of sector to be writte */
    sectorStart %= (DRV_NVM_PAGE_SIZE / dObj->sectorSizeInBytes);
    uint32_t startIndex = sectorStart * dObj->sectorSizeInBytes;
    uint32_t finalIndex = startIndex + (dObj->sectorSizeInBytes * noOfSectors);

    for(i = startIndex; i < finalIndex ; i++)
    {
        dObj->buffer[i] = *buffer;
        buffer++;
    }

    /* Now, write the modified page into NVM memory */
    bufferHandle = DRV_NVM_Write(dObj->drvNVMClientHandle,address, dObj->buffer, DRV_NVM_PAGE_SIZE);
    if(bufferHandle != DRV_NVM_BUFFER_HANDLE_INVALID)
    {
        while(DRV_NVM_BufferStatus(dObj->drvNVMClientHandle,bufferHandle) != DRV_NVM_BUFFER_COMPLETED);

    }
    else
    {
        return SYS_FS_MEDIA_BUFFER_HANDLE_INVALID;
    }
    return bufferHandle;

}

SYS_FS_MEDIA_BUFFER_STATUS DRV_NVM_MEDIA_BufferStatusGet(DRV_HANDLE handle,
        SYS_FS_MEDIA_BUFFER_HANDLE bufferHandle)
{
    DRV_NVM_MEDIA_OBJECT * dObj = (DRV_NVM_MEDIA_OBJECT *) handle;
    return((SYS_FS_MEDIA_BUFFER_STATUS)(DRV_NVM_BufferStatus(dObj->drvNVMClientHandle,
                    (DRV_NVM_BUFFER_HANDLE)bufferHandle)));
}

NVM_MEDIA_PROPERTIES DRV_NVM_MEDIA_MediaPropertiesGet (DRV_HANDLE handle)
{
    DRV_NVM_MEDIA_OBJECT * dObj = (DRV_NVM_MEDIA_OBJECT *) handle;
    NVM_MEDIA_PROPERTIES mediaProperties;

    mediaProperties.diskNo = dObj->disk;
    mediaProperties.mediaStartAddress = dObj->mediaStartAddress;
    mediaProperties.mediaSize = dObj->nSectors * dObj->sectorSizeInBytes;
    mediaProperties.mediaPartitions = dObj->nSectors;
    mediaProperties.mediaPartitionSizeInBytes = dObj->sectorSizeInBytes;
    
    return mediaProperties;
}
