/*******************************************************************************
  NVM Driver Static implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_nvm_static.c

  Summary:
    Source code for the NVM driver static implementation.

  Description:
    The NVM device driver provides a simple interface to manage the NVM
    modules on Microchip microcontrollers. This file contains static implementation
    for the NVM driver.

  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.

    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

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
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"
#include "driver/nvm/src/drv_nvm_static_local.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

/*************************************
* This is the driver static object
*************************************/
DRV_NVM_OBJECT  gDrvNVMObj ;

/*************************************************
 * Driver Buffer Objects. These transport the
 * read, write and erase requests.
 *************************************************/

DRV_NVM_BUFFER_OBJECT   gDrvNVMBufferObject[DRV_NVM_BUFFER_OBJECT_NUMBER];

/************************************************
 * This token is incremented for every request
 * added to the queue and is used to generate
 * a different buffer handle for every request.
 ***********************************************/

uint16_t gDrvNVMBufferToken = 0;

<#if CONFIG_USE_DRV_NVM_ERASE_WRITE == true>
/**************************************************
 * Erase buffer size
 **************************************************/

uint8_t gDrvNVMEraseBuffer[DRV_NVM_INDEX_0+1][DRV_NVM_PAGE_SIZE] __attribute__((coherent, aligned(16)));
</#if>

// *****************************************************************************
// *****************************************************************************
// Section: File System Data
// *****************************************************************************
// *****************************************************************************
<#if CONFIG_SYS_FS_MPFS == true>

extern uint8_t NVM_MEDIA_DATA[];
</#if>

<#if CONFIG_USE_DRV_NVM_SYS_FS_REGISTER == true>
const SYS_FS_MEDIA_FUNCTIONS nvmMediaFunctions =
{
    .mediaStatusGet     = DRV_NVM_IsAttached,
    .mediaGeometryGet   = DRV_NVM_GeometryGet,
    .sectorRead         = DRV_NVM_Read,
<#if CONFIG_USE_DRV_NVM_ERASE_WRITE == true>
    .sectorWrite        = DRV_NVM_EraseWrite,
<#else>
    .sectorWrite        = NULL,
</#if>
    .eventHandlerset    = DRV_NVM_EventHandlerSet,
    .commandStatusGet   = (void *)DRV_NVM_CommandStatus,
    .Read               = DRV_NVM_Read,
    .erase              = DRV_NVM_Erase,
    .addressGet         = DRV_NVM_AddressGet,
    .open               = DRV_NVM_Open,
    .close              = DRV_NVM_Close,
    .tasks              = NULL,
};
</#if>

SYS_FS_MEDIA_REGION_GEOMETRY NVMGeometryTable[3] =
{
    {
        .blockSize = 1,
        .numBlocks = (DRV_NVM_MEDIA_SIZE * 1024),
    },
    {
       .blockSize = DRV_NVM_ROW_SIZE,
       .numBlocks = ((DRV_NVM_MEDIA_SIZE * 1024)/DRV_NVM_ROW_SIZE)
    },
    {
       .blockSize = DRV_NVM_PAGE_SIZE,
       .numBlocks = ((DRV_NVM_MEDIA_SIZE * 1024)/DRV_NVM_PAGE_SIZE)
    }
};

const SYS_FS_MEDIA_GEOMETRY NVMGeometry =
{
    .mediaProperty = SYS_FS_MEDIA_WRITE_IS_BLOCKING,
    .numReadRegions = 1,
    .numWriteRegions = 1,
    .numEraseRegions = 1,
    .geometryTable = (SYS_FS_MEDIA_REGION_GEOMETRY *)&NVMGeometryTable
};

// *****************************************************************************
// *****************************************************************************
// Section: Instance 0 static driver functions : 
//          NVM is single instance at H/W level, so instance index is ignored.
// *****************************************************************************
// *****************************************************************************

SYS_MODULE_OBJ DRV_NVM_Initialize(const SYS_MODULE_INDEX index, const SYS_MODULE_INIT * const init)
{
    DRV_NVM_OBJECT *dObj = (DRV_NVM_OBJECT*)NULL;

    dObj = &gDrvNVMObj;
<#if CONFIG_USE_3RDPARTY_RTOS>

    OSAL_RESULT retVal;

    /* Create a Hardware instance Mutex */
    retVal = OSAL_MUTEX_Create(&dObj->nvmInstanceObjMutex);
    _DRV_NVM_VALIDATE_EXPR((retVal != OSAL_RESULT_TRUE), SYS_MODULE_OBJ_INVALID);
</#if>

    /* Update the NVM Object parameters. */
<#if CONFIG_USE_DRV_NVM_ERASE_WRITE == true>
    dObj->eraseBuffer = &gDrvNVMEraseBuffer[0][0];
</#if>
    dObj->blockStartAddress = DRV_NVM_MEDIA_START_ADDRESS;
    dObj->eventHandler = NULL;

    /* Disable flash access - WREN bit in NVMCON */
    PLIB_NVM_MemoryModifyInhibit(NVM_ID_0);

    /* Interrupt flag cleared on the safer side */
    SYS_INT_SourceStatusClear(${CONFIG_DRV_NVM_INTERRUPT_SOURCE});
<#if CONFIG_DRV_NVM_INTERRUPT_MODE == true>

    /* Enabling the interrupt source */
    SYS_INT_SourceEnable(${CONFIG_DRV_NVM_INTERRUPT_SOURCE});
</#if>
<#if CONFIG_USE_DRV_NVM_SYS_FS_REGISTER == true>

    /* Registering NVM functions with File System */
    SYS_FS_MEDIA_MANAGER_Register((SYS_MODULE_OBJ)dObj, DRV_NVM_INDEX_0, &nvmMediaFunctions, SYS_FS_MEDIA_TYPE_NVM);
</#if>

    /* Return the static system object */
    return (SYS_MODULE_OBJ)DRV_NVM_INDEX_0;
}

void  DRV_NVM_Deinitialize(SYS_MODULE_OBJ object)
{
    DRV_NVM_OBJECT *dObj = (DRV_NVM_OBJECT*)NULL;

    dObj = &gDrvNVMObj;
<#if CONFIG_DRV_NVM_INTERRUPT_MODE == true>

    /* Disable the Interrupt */
    SYS_INT_SourceDisable(${CONFIG_DRV_NVM_INTERRUPT_SOURCE});
</#if>

    /* Reset the queue */
    dObj->writeEraseQ = NULL;
<#if CONFIG_USE_3RDPARTY_RTOS>

    /* Delete the Hardware instance Mutex */
    OSAL_MUTEX_Delete(&dObj->nvmInstanceObjMutex);
</#if>

}

SYS_STATUS DRV_NVM_Status(SYS_MODULE_OBJ object)
{
    /* Return the driver status */
    return SYS_STATUS_READY;
}

void DRV_NVM_Tasks(SYS_MODULE_OBJ object)
{
    DRV_NVM_OBJECT *dObj = (DRV_NVM_OBJECT*)NULL;
    DRV_NVM_BUFFER_OBJECT * bufferObj;
    unsigned int bufferOffset = 1;
    unsigned int offset, i;
    unsigned int blocksInPage;
    unsigned int erasePageAddress;

    dObj = &gDrvNVMObj;

    /* Check the status of interrupt flag, if not set
     * then nothing need to be done yet */
    if(!SYS_INT_SourceStatusGet(${CONFIG_DRV_NVM_INTERRUPT_SOURCE}))
    {
        return;
    }

    /* Clear the interrupt flag */
    SYS_INT_SourceStatusClear(${CONFIG_DRV_NVM_INTERRUPT_SOURCE});

    /* Get the object at the head of the write queue */
    bufferObj = dObj->writeEraseQ;

    if(bufferObj == NULL)
    {
        /* This could happen if the client has closed the driver and the
         * request has been removed from the queue */
        return;
    }

    /* Check if the buffer is complete */
    bufferObj->nBlocksPending --;
    if(bufferObj->nBlocksPending == 0)
    {
        if(((bufferObj->flag == DRV_NVM_BUFFER_FLAG_ERASE_WRITE)
                    && (dObj->eraseWriteStep & DRV_NVM_ERASE_WRITE_STEP_WRITE_PAGE)
                    && (dObj->nRowsPending == 0))
                    || (bufferObj->flag == DRV_NVM_BUFFER_FLAG_ERASE)
                    || (bufferObj->flag == DRV_NVM_BUFFER_FLAG_WRITE))

        {
            /* There are no pending operations on this buffer
             * object. Invoke the client callback */

            bufferObj->status = DRV_NVM_COMMAND_COMPLETED;
            if(dObj->eventHandler != NULL)
            {
                /* Call the event handler */
                dObj->eventHandler(DRV_NVM_EVENT_COMMAND_COMPLETE,
                        (DRV_NVM_COMMAND_HANDLE)bufferObj->commandHandle, dObj->context);
            }

            /* Deallocate the buffer object and get the next buffer object
             * in queue which could be NULL because this is the last object
             * in queue.  */

            bufferObj->inUse = false;
            dObj->writeEraseQ = bufferObj->next;
            bufferObj = bufferObj->next;
            bufferOffset = 0;
        }
        else if ((bufferObj->flag == DRV_NVM_BUFFER_FLAG_ERASE_WRITE)
                && (dObj->eraseWriteStep & DRV_NVM_ERASE_WRITE_STEP_WRITE_PAGE)
                && (dObj->nRowsPending != 0))
        {
            /* There are more blocks pending in the erase write
             * operation. We need to continue erasing. */
            dObj->eraseWriteStep = DRV_NVM_ERASE_WRITE_STEP_ERASE_NEXT_PAGE;
        }
    }

    /* At this point bufferObj is either pointing to a new bufferObj or an
     * existing one that has not completed yet.
     * */
    if (bufferObj == NULL)
    {
        return;
    }

    switch(bufferObj->flag)
    {
        case DRV_NVM_BUFFER_FLAG_WRITE:
            /* We are either starting a new write or continuing an
             * existing one. For a new buffer, bufferOffset will
             * be zero.
             * */
            bufferObj->flashMemPointer += (bufferOffset * bufferObj->blockSize);
            bufferObj->appDataPointer += (bufferOffset * DRV_NVM_ROW_SIZE);
            _DRV_NVM_WriteBufferObjProcess( bufferObj);
            break;

        case DRV_NVM_BUFFER_FLAG_ERASE:

            /* We are either starting a new erase or continuing an
             * existing one.
             * */
            bufferObj->flashMemPointer += (bufferOffset * bufferObj->blockSize);
            _DRV_NVM_EraseBufferObjProcess(bufferObj);
            break;

        case DRV_NVM_BUFFER_FLAG_ERASE_WRITE:

            /* We have a new Erase Write request or if an existing
             * one is continuing. */
            if((bufferObj->nBlocksPending == 0) && (bufferObj->blockSize == 0))
            {
                /* This is a completely new request
                 * For an erase write request the number of blocks pending
                 * first will 1 and a page will be erased. The total number
                 * of blocks to program is tracked in the hardware instance
                 * object.
                 * */
                dObj->nRowsPending = bufferObj->size/DRV_NVM_ROW_SIZE;
                dObj->appDataMemory = bufferObj->appDataPointer;
                dObj->eraseWriteStartAddress = (uint32_t)bufferObj->flashMemPointer;
                dObj->eraseWriteStep = DRV_NVM_ERASE_WRITE_STEP_ERASE_COMPLETE;

                /* Obtain the page address that contains this row and then
                 * set up the buffer object to erase it.
                 * */
                erasePageAddress = ((uint32_t)bufferObj->flashMemPointer / DRV_NVM_PAGE_SIZE) * DRV_NVM_PAGE_SIZE;
                bufferObj->status = DRV_NVM_COMMAND_IN_PROGRESS;
                bufferObj->nBlocksPending = 1;
                bufferObj->blockSize = DRV_NVM_PAGE_SIZE;
                bufferObj->flashMemPointer  = (uint8_t *)erasePageAddress;

                /* Make a back up of the page to be erased */
                for(i = 0; i < DRV_NVM_PAGE_SIZE; i ++)
                {
                    dObj->eraseBuffer[i] = bufferObj->flashMemPointer[i];
                }

                _DRV_NVM_EraseBufferObjProcess(bufferObj);
            }
            else if(dObj->eraseWriteStep & DRV_NVM_ERASE_WRITE_STEP_ERASE_COMPLETE)
            {
                /* We have completed the erase of the erase write step.
                 * Get the offset of the row to be programmed within
                 * this page */
                offset = dObj->eraseWriteStartAddress - (uint32_t)(bufferObj->flashMemPointer);
                blocksInPage = (DRV_NVM_PAGE_SIZE - offset)/DRV_NVM_ROW_SIZE;
                blocksInPage = (blocksInPage > dObj->nRowsPending) ? dObj->nRowsPending : blocksInPage;

                /* Overlay the application row over the erase buffer */
                for(i = 0; i < blocksInPage * DRV_NVM_ROW_SIZE; i ++)
                {
                    dObj->eraseBuffer[offset + i] = dObj->appDataMemory[i];
                }

                /* Update the number of rows pending and source and destination
                 * addresses. */
                dObj->nRowsPending -= blocksInPage;
                dObj->appDataMemory += (blocksInPage * DRV_NVM_ROW_SIZE);
                dObj->eraseWriteStartAddress += (blocksInPage * DRV_NVM_ROW_SIZE);

                /* Update the buffer object for a row write operation. */
                bufferObj->blockSize = DRV_NVM_ROW_SIZE;
                bufferObj->nBlocksPending = DRV_NVM_PAGE_SIZE/DRV_NVM_ROW_SIZE;
                bufferObj->appDataPointer = dObj->eraseBuffer;
                dObj->eraseWriteStep = DRV_NVM_ERASE_WRITE_STEP_WRITE_PAGE;
                _DRV_NVM_WriteBufferObjProcess(bufferObj);
            }
            else if(dObj->eraseWriteStep & DRV_NVM_ERASE_WRITE_STEP_WRITE_PAGE)
            {
                /* This is an on going operation */
                bufferObj->flashMemPointer += DRV_NVM_ROW_SIZE;
                bufferObj->appDataPointer += DRV_NVM_ROW_SIZE;
                _DRV_NVM_WriteBufferObjProcess(bufferObj);
            }
            else if(dObj->eraseWriteStep & DRV_NVM_ERASE_WRITE_STEP_ERASE_NEXT_PAGE)
            {
                /* We have completed erasing and updating a page.
                 * Another page needs to be erased and updated.
                 * */
                bufferObj->flashMemPointer += DRV_NVM_ROW_SIZE;
                bufferObj->blockSize = DRV_NVM_PAGE_SIZE;
                bufferObj->nBlocksPending = 1;
                dObj->eraseWriteStep = DRV_NVM_ERASE_WRITE_STEP_ERASE_COMPLETE;

                /* Make a backup of the page */
                for(i = 0; i < DRV_NVM_PAGE_SIZE; i++)
                {
                    dObj->eraseBuffer[i] = bufferObj->flashMemPointer[i];
                }

                _DRV_NVM_EraseBufferObjProcess( bufferObj);
            }

            break;
        case DRV_NVM_BUFFER_FLAG_READ:
        default:
            break;
    }
}

DRV_HANDLE DRV_NVM_Open(const SYS_MODULE_INDEX index, const DRV_IO_INTENT ioIntent)
{
    /* Return valid handle */
    return ((DRV_HANDLE)DRV_NVM_INDEX_0 );
}

void DRV_NVM_Close(const DRV_HANDLE handle)
{
    return;
}

void DRV_NVM_Read
(
    const DRV_HANDLE handle, 
    DRV_NVM_COMMAND_HANDLE * commandHandle,
    void * targetBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_NVM_OBJECT *dObj = (DRV_NVM_OBJECT*)NULL;
    DRV_NVM_BUFFER_OBJECT * bufferObj;
<#if CONFIG_USE_3RDPARTY_RTOS>
    OSAL_RESULT retVal;
</#if>
    int iEntry, i;
    uint8_t * source = NULL;
    DRV_NVM_COMMAND_HANDLE * tempHandle1, tempHandle2;

    uint8_t *readBuffer = (uint8_t *)targetBuffer;

    dObj = &gDrvNVMObj;

    /* Validate the parameters */
    _DRV_NVM_VALIDATE_EXPR((readBuffer == NULL), (void)0);
    _DRV_NVM_VALIDATE_EXPR(((blockStart + nBlock) > NVMGeometryTable[GEOMETRY_TABLE_READ_ENTRY].numBlocks), (void)0);
    _DRV_NVM_VALIDATE_EXPR((nBlock == 0), (void)0);

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_NVM_COMMAND_HANDLE_INVALID;
<#if CONFIG_USE_3RDPARTY_RTOS>

    /* Acquire the Mutex */
    retVal = OSAL_MUTEX_Lock(&dObj->nvmInstanceObjMutex,OSAL_WAIT_FOREVER);
    _DRV_NVM_VALIDATE_EXPR((retVal != OSAL_RESULT_TRUE), (void)0);
</#if>

    /* Update the block start address */
    source = (uint8_t *)(dObj->blockStartAddress + blockStart);

    /* The read buffer function does not need a task routine as it does not
     * block on hardware. A buffer object is still used to allow the client to
     * track the status of the read. The function will complete the read and
     * exit from the function. */
    for(iEntry = 0; iEntry < DRV_NVM_BUFFER_OBJECT_NUMBER; iEntry ++)
    {
        /* Search for a free buffer object to use */
        if(gDrvNVMBufferObject[iEntry].inUse == false)
        {
            /* Found a free buffer object. */
            bufferObj = &gDrvNVMBufferObject[iEntry];

            bufferObj->inUse           = true;
            bufferObj->commandHandle    = _DRV_NVM_MAKE_HANDLE(gDrvNVMBufferToken, iEntry);
            bufferObj->size            = nBlock;
            bufferObj->appDataPointer  = readBuffer;
            bufferObj->flashMemPointer = source;
            bufferObj->status          = DRV_NVM_COMMAND_IN_PROGRESS;

            for(i = 0; i < nBlock; i ++)
            {
                /* Do the actual read here. Not using a PLIB call here
                 * as the memory on PIC32 is linear. We may have to
                 * use a PLIB call for other architectures. */
                *readBuffer = *source;
                readBuffer++;
                source ++;
            }

            *tempHandle1 = (bufferObj->commandHandle);

            bufferObj->status = DRV_NVM_COMMAND_COMPLETED;

            if(dObj->eventHandler != NULL)
            {
                /* Call the event handler */
                dObj->eventHandler(DRV_NVM_EVENT_COMMAND_COMPLETE,
                        (DRV_NVM_COMMAND_HANDLE)bufferObj->commandHandle, dObj->context);
            }

            bufferObj->inUse = false;

            /* Update the token number. */
            _DRV_NVM_UPDATE_BUF_TOKEN(gDrvNVMBufferToken);
            break;
        }
    }
<#if CONFIG_USE_3RDPARTY_RTOS>

    /* Release the Mutex */
    OSAL_MUTEX_Unlock(&dObj->nvmInstanceObjMutex);
</#if>

    return;
}

void DRV_NVM_Write
(
    const DRV_HANDLE handle,
    DRV_NVM_COMMAND_HANDLE * commandHandle,
    void * sourceBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_NVM_COMMAND_HANDLE * tempHandle1, tempHandle2;

    /* Validate the parameters */
    _DRV_NVM_VALIDATE_EXPR((sourceBuffer == NULL), (void)0);
    _DRV_NVM_VALIDATE_EXPR(((blockStart + nBlock) > NVMGeometryTable[GEOMETRY_TABLE_WRITE_ENTRY].numBlocks), (void)0);
    _DRV_NVM_VALIDATE_EXPR((nBlock == 0), (void)0);

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_NVM_COMMAND_HANDLE_INVALID;

    *tempHandle1 = _DRV_NVM_BlockOperation (sourceBuffer, blockStart, nBlock,
                                             DRV_NVM_BUFFER_FLAG_WRITE, DRV_NVM_ROW_SIZE);
    return;
}

void DRV_NVM_Erase
(
    const DRV_HANDLE handle,
    DRV_NVM_COMMAND_HANDLE * commandHandle,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    /* NVM Driver erase is blocking */
    DRV_NVM_COMMAND_HANDLE * tempHandle1, tempHandle2;

    /* Validate the parameters */
    _DRV_NVM_VALIDATE_EXPR(((blockStart + nBlock) > NVMGeometryTable[GEOMETRY_TABLE_ERASE_ENTRY].numBlocks), (void)0);
    _DRV_NVM_VALIDATE_EXPR((nBlock == 0), (void)0);

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_NVM_COMMAND_HANDLE_INVALID;

    /* The source while calling the below function is set to destination
     * because the source parameter is ignored for erase operation
     * and it cannot be NULL. */
    *tempHandle1 = _DRV_NVM_BlockOperation ((uint8_t *)blockStart, blockStart, nBlock,
                                            DRV_NVM_BUFFER_FLAG_ERASE, DRV_NVM_PAGE_SIZE);
    return;
}
<#if CONFIG_USE_DRV_NVM_ERASE_WRITE>

void DRV_NVM_EraseWrite
(
    const DRV_HANDLE handle,
    DRV_NVM_COMMAND_HANDLE * commandHandle,
    void * sourceBuffer,
    uint32_t writeBlockStart,
    uint32_t nWriteBlock
)
{
    DRV_NVM_OBJECT *dObj = (DRV_NVM_OBJECT*)NULL;
    DRV_NVM_BUFFER_OBJECT * bufferObj;
<#if CONFIG_USE_3RDPARTY_RTOS>
    OSAL_RESULT retVal;
</#if>
    int iEntry;
<#if CONFIG_DRV_NVM_INTERRUPT_MODE == true>
    bool interruptWasEnabled;
</#if>
    uint32_t erasePageAddress;
    DRV_NVM_COMMAND_HANDLE * tempHandle1, tempHandle2;

    dObj = &gDrvNVMObj;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_NVM_COMMAND_HANDLE_INVALID;

    _DRV_NVM_VALIDATE_EXPR((sourceBuffer == NULL), (void)0);
    _DRV_NVM_VALIDATE_EXPR(((writeBlockStart + nWriteBlock) > NVMGeometryTable[GEOMETRY_TABLE_WRITE_ENTRY].numBlocks), (void)0);
    _DRV_NVM_VALIDATE_EXPR((nWriteBlock == 0), (void)0);

    writeBlockStart *= DRV_NVM_ROW_SIZE;
    writeBlockStart += dObj->blockStartAddress;
    nWriteBlock     *= DRV_NVM_ROW_SIZE;

    /* Address should be row aligned */
    _DRV_NVM_VALIDATE_EXPR(((writeBlockStart % DRV_NVM_ROW_SIZE) != 0), (void)0);
<#if CONFIG_USE_3RDPARTY_RTOS>

    /* Acquire the Mutex */
    retVal = OSAL_MUTEX_Lock(&dObj->nvmInstanceObjMutex,OSAL_WAIT_FOREVER);
    _DRV_NVM_VALIDATE_EXPR((retVal != OSAL_RESULT_TRUE), (void)0);
</#if>
<#if CONFIG_DRV_NVM_INTERRUPT_MODE == true>

    /* Disable the interrupt so that any write operation from an interrupt
     * context does not interfere.
     * */
    interruptWasEnabled = SYS_INT_SourceDisable (${CONFIG_DRV_NVM_INTERRUPT_SOURCE});
</#if>

    for(iEntry = 0; iEntry < DRV_NVM_BUFFER_OBJECT_NUMBER; iEntry ++)
    {
        if(gDrvNVMBufferObject[iEntry].inUse == false)
        {
            /* Found a buffer object that can be used */
            bufferObj                       = &gDrvNVMBufferObject[iEntry];
            bufferObj->inUse                = true;
            bufferObj->appDataPointer       = sourceBuffer;
            bufferObj->flashMemPointer      = (uint8_t *)writeBlockStart;
            bufferObj->size                 = nWriteBlock;
            bufferObj->status               = DRV_NVM_COMMAND_QUEUED;
            bufferObj->flag                 = DRV_NVM_BUFFER_FLAG_ERASE_WRITE;
            bufferObj->commandHandle         = _DRV_NVM_MAKE_HANDLE(gDrvNVMBufferToken, iEntry);
            bufferObj->next                 = NULL;
            bufferObj->previous             = NULL;

            /* Set the nWriteBlockPending and block size to 0 to indicate that
             * we have not started processing this yet.
             * */
            bufferObj->nBlocksPending       = 0;
            bufferObj->blockSize            = 0;

            /* Update the token number. */
            _DRV_NVM_UPDATE_BUF_TOKEN(gDrvNVMBufferToken);

            if(dObj->writeEraseQ == NULL)
            {
                /* This means the queue is empty */
                dObj->writeEraseQ = bufferObj;

                /* For an erase write request the number of blocks pending
                 * first will 1 and a page will be erased. The total number
                 * of blocks to program is tracked in the hardware instance
                 * object.  */
                dObj->nRowsPending = nWriteBlock/DRV_NVM_ROW_SIZE;
                dObj->appDataMemory = sourceBuffer;
                dObj->eraseWriteStartAddress = writeBlockStart;
                dObj->eraseWriteStep = DRV_NVM_ERASE_WRITE_STEP_ERASE_COMPLETE;

                /* Obtain the page address that contains this row and then
                 * set up the buffer object to erase it */
                erasePageAddress = (writeBlockStart / DRV_NVM_PAGE_SIZE) * DRV_NVM_PAGE_SIZE;
                bufferObj->status = DRV_NVM_COMMAND_IN_PROGRESS;
                bufferObj->nBlocksPending = 1;
                bufferObj->blockSize = DRV_NVM_PAGE_SIZE;
                bufferObj->flashMemPointer  = (uint8_t *)erasePageAddress;

                /* Make a back up of the page to be erased */
                for(iEntry = 0; iEntry < DRV_NVM_PAGE_SIZE; iEntry ++)
                {
                    dObj->eraseBuffer[iEntry] = bufferObj->flashMemPointer[iEntry];
                }

                _DRV_NVM_EraseBufferObjProcess(bufferObj);
            }
            else
            {
                /* This means there is already a buffer queued up. We add the
                 * buffer to the linked list.
                 * */
                DRV_NVM_BUFFER_OBJECT * iterator;

                iterator = dObj->writeEraseQ;

                /* Find the last object in the queue */
                while(iterator->next != NULL)
                {
                    iterator = iterator->next;
                }

                /* Append the buffer object to the last buffer
                 * object in the queue */
                iterator->next = bufferObj;
                bufferObj->previous = iterator;
                bufferObj->next = NULL;
            }

            *tempHandle1 = bufferObj->commandHandle;
            break;
        }
    }
<#if CONFIG_DRV_NVM_INTERRUPT_MODE == true>

    if(interruptWasEnabled)
    {
        /* Re-Enable the interrupt if it was enabled */
        SYS_INT_SourceEnable(${CONFIG_DRV_NVM_INTERRUPT_SOURCE});
    }
</#if>
<#if CONFIG_USE_3RDPARTY_RTOS>

    /* Release the Mutex */
    OSAL_MUTEX_Unlock(&dObj->nvmInstanceObjMutex);
</#if>

    return;
}
</#if>


DRV_NVM_COMMAND_STATUS DRV_NVM_CommandStatus
(
    const DRV_HANDLE handle,
    const DRV_NVM_COMMAND_HANDLE commandHandle
)
{
    uint16_t iEntry;

    /* The upper 16 bits of the buffer handle
     * are the token and the lower 16 bits of the
     * are buffer index into the gDrvNVMBufferObject
     * array */
    iEntry = commandHandle & 0xFFFF;

    /* Compare the buffer handle with buffer handle
     * in the object */
    if(gDrvNVMBufferObject[iEntry].commandHandle != commandHandle)
    {
        /* This means that object has been re-used by another
         * request. Indicate that the operation is completed.
         * */
        return (DRV_NVM_COMMAND_COMPLETED);
    }

    /* Return the last known buffer object status */
    return (gDrvNVMBufferObject[iEntry].status);
}

SYS_FS_MEDIA_GEOMETRY * DRV_NVM_GeometryGet(const DRV_HANDLE handle)
{
    return (SYS_FS_MEDIA_GEOMETRY *)&NVMGeometry;
}

void DRV_NVM_EventHandlerSet
(
    const DRV_HANDLE handle,
    const void * eventHandler,
    const uintptr_t context
)
{
    /* Set the event handler */
    gDrvNVMObj.eventHandler = eventHandler;
    gDrvNVMObj.context = context;
}

bool DRV_NVM_IsAttached(const DRV_HANDLE handle)
{
    /* This function always returns true */
   return true;
}

bool DRV_NVM_IsWriteProtected(const DRV_HANDLE handle)
{
    /* This function always returns false */
    return false;
}

uintptr_t DRV_NVM_AddressGet(const DRV_HANDLE handle)
{
    /* Return the NVM start address */
    return gDrvNVMObj.blockStartAddress;
}


// *****************************************************************************
// *****************************************************************************
// Section: NVM Driver Local Functions
// *****************************************************************************
// *****************************************************************************

void _DRV_NVM_UnlockSequence
(
    _DRV_NVM_OPERATION_MODE mode
)
{
    /* Disable the global interrupts */
    gDrvNVMObj.intStatus = SYS_INT_Disable();

    /* Select the desired flash operation
     * and then enable flash operations
     * */
    PLIB_NVM_MemoryModifyInhibit(NVM_ID_0);
    PLIB_NVM_MemoryOperationSelect(NVM_ID_0, mode);
    PLIB_NVM_MemoryModifyEnable(NVM_ID_0);

    /* Write the keys.*/
<#if CONFIG_PIC32MZ == true>
    PLIB_NVM_FlashWriteKeySequence(NVM_ID_0, 0);
</#if>
    PLIB_NVM_FlashWriteKeySequence( NVM_ID_0, DRV_NVM_PROGRAM_UNLOCK_KEY1 );
    PLIB_NVM_FlashWriteKeySequence( NVM_ID_0, DRV_NVM_PROGRAM_UNLOCK_KEY2 );

    /* Start the operation */
    PLIB_NVM_FlashWriteStart(NVM_ID_0);

    /* Wait for the operation to finish.*/
    while(!PLIB_NVM_FlashWriteCycleHasCompleted(NVM_ID_0));

    /* Enable the global interrupts if they were enabled */
    if(gDrvNVMObj.intStatus)
    {
        SYS_INT_Enable();
    }
}

void _DRV_NVM_WriteBufferObjProcess
(
    DRV_NVM_BUFFER_OBJECT * bufferObj
)
{
    PLIB_NVM_FlashAddressToModify(NVM_ID_0, _DRV_NVM_KVA_TO_PA((uint32_t)(bufferObj->flashMemPointer)));
    PLIB_NVM_DataBlockSourceAddress(NVM_ID_0, _DRV_NVM_KVA_TO_PA((uint32_t)(bufferObj->appDataPointer))) ;
    _DRV_NVM_UnlockSequence( _DRV_ROW_PROGRAM_OPERATION );
}

void _DRV_NVM_EraseBufferObjProcess
(
    DRV_NVM_BUFFER_OBJECT * bufferObj
)
{
    PLIB_NVM_FlashAddressToModify(NVM_ID_0, _DRV_NVM_KVA_TO_PA((uint32_t)(bufferObj->flashMemPointer)));
    _DRV_NVM_UnlockSequence(_DRV_PAGE_ERASE_OPERATION);
}

DRV_NVM_COMMAND_HANDLE _DRV_NVM_BlockOperation
(
    uint8_t * source,
    uint32_t blockStart,
    uint32_t nBlock,
    DRV_NVM_BUFFER_FLAGS flag,
    uint32_t blockSize
)
{
    int iEntry;
    DRV_NVM_BUFFER_OBJECT * bufferObj;
    DRV_NVM_OBJECT *dObj = (DRV_NVM_OBJECT*)NULL;
<#if CONFIG_USE_3RDPARTY_RTOS>
    OSAL_RESULT retVal;
</#if>

    dObj = &gDrvNVMObj;


    /* Update the block start address and the number of bytes based on
     * the operation being performed.
     * */
    blockStart *= blockSize;
    blockStart += dObj->blockStartAddress;
<#if CONFIG_USE_3RDPARTY_RTOS>

    /* Acquire the Mutex */
    retVal = OSAL_MUTEX_Lock(&dObj->nvmInstanceObjMutex,OSAL_WAIT_FOREVER);
    _DRV_NVM_VALIDATE_EXPR((retVal != OSAL_RESULT_TRUE), DRV_NVM_COMMAND_HANDLE_INVALID);
</#if>

    for(iEntry = 0; iEntry < DRV_NVM_BUFFER_OBJECT_NUMBER; iEntry ++)
    {
        if(gDrvNVMBufferObject[iEntry].inUse == false)
        {
            /* Found a free buffer object */
            bufferObj                  = &gDrvNVMBufferObject[iEntry];
            bufferObj->inUse           = true;
            bufferObj->appDataPointer  = source;
            bufferObj->flashMemPointer = (uint8_t *)blockStart;
            bufferObj->size            = (nBlock * blockSize);
            bufferObj->status          = DRV_NVM_COMMAND_QUEUED;
            bufferObj->flag            = flag;
            bufferObj->commandHandle    = _DRV_NVM_MAKE_HANDLE(gDrvNVMBufferToken, iEntry);
            bufferObj->next            = NULL;
            bufferObj->previous        = NULL;
            bufferObj->nBlocksPending  = nBlock;
            bufferObj->blockSize       = blockSize;

            /* Update the token number. */
            _DRV_NVM_UPDATE_BUF_TOKEN(gDrvNVMBufferToken);
<#if CONFIG_DRV_NVM_INTERRUPT_MODE == true>

            /* Disable the interrupt so that any write operation from an interrupt
             * context does not interfere.
             * */
            dObj->intStatus = SYS_INT_SourceDisable (${CONFIG_DRV_NVM_INTERRUPT_SOURCE});
</#if>

            if(dObj->writeEraseQ == NULL)
            {
                /* Since the write Q is empty, the operation can be started
                 * immediately.
                 * */
                dObj->writeEraseQ = bufferObj;
                bufferObj->status = DRV_NVM_COMMAND_IN_PROGRESS;

                if(DRV_NVM_BUFFER_FLAG_WRITE == flag)
                {
                    /* A write operation needs to be performed */
                    _DRV_NVM_WriteBufferObjProcess(bufferObj);
                }
                else if(DRV_NVM_BUFFER_FLAG_ERASE == flag)
                {
                    /* A erase operation needs to be performed */
                    _DRV_NVM_EraseBufferObjProcess(bufferObj);
                }
            }
            else
            {
                /* This means there is already a buffer queued
                 * up. We add the buffer to the linked list.
                 * */
                DRV_NVM_BUFFER_OBJECT * iterator;

                iterator = dObj->writeEraseQ;

                /* Find the last object in the queue */
                while(iterator->next != NULL)
                {
                    iterator = iterator->next;
                }

                /* Append the buffer object to the last buffer
                 * object in the queue.
                 * */

                iterator->next = bufferObj;
                bufferObj->previous = iterator;
                bufferObj->next = NULL;
            }
<#if CONFIG_DRV_NVM_INTERRUPT_MODE == true>

            if(dObj->intStatus)
            {
                /* Enable interrupts if they were already enabled */
                SYS_INT_SourceEnable(${CONFIG_DRV_NVM_INTERRUPT_SOURCE});
            }
</#if>
<#if CONFIG_USE_3RDPARTY_RTOS>

            /* Release the Mutex */
            OSAL_MUTEX_Unlock(&dObj->nvmInstanceObjMutex);
</#if>

            return((DRV_NVM_COMMAND_HANDLE)bufferObj->commandHandle);
        }
    }
<#if CONFIG_USE_3RDPARTY_RTOS>

    /* Release the Mutex */
    OSAL_MUTEX_Unlock(&dObj->nvmInstanceObjMutex);
</#if>

    /* Could not find a buffer object */
    return(DRV_NVM_COMMAND_HANDLE_INVALID);
}

/*******************************************************************************
 End of File
*/
