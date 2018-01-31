/*******************************************************************************
  NVM Driver Interface Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_nvm_erasewrite.c

  Summary:
    NVM Driver Interface Definition

  Description:
    The NVM Driver provides a interface to access the NVM on the PIC32
    microcontroller. This file implements the NVM Driver Erase Write function.
    This file should be included in the project if NVM driver Erase Write
    functionality is needed.
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

#include "driver/nvm/src/drv_nvm_local.h"

// *****************************************************************************
/* Function:
    void DRV_NVM_EraseWrite
    (
        const DRV_HANDLE handle,
        DRV_NVM_COMMAND_HANDLE * commandHandle,
        void * sourceBuffer,
        uint32_t writeBlockStart,
        uint32_t nWriteBlock
    )

  Summary:
    Erase and Write blocks of data starting from a specified address in flash
    memory.

  Description:
    This function combines the step of erasing a page and then writing the row.
    The application can use this function if it wants to avoid having to
    explicitly delete a page in order to update the rows contained in the page. 

    This function schedules a non-blocking operation to erase and write blocks
    of data into flash memory. The function returns with a valid buffer handle
    in the commandHandle argument if the write request was scheduled successfully.
    The function adds the request to the hardware instance queue and returns 
    immediately. While the request is in the queue, the application buffer is 
    owned by the driver and should not be modified. The function returns 
    DRV_NVM_COMMAND_HANDLE_INVALID in the commandHandle argument under the 
    following circumstances:
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the client opened the driver for read only
    - if the buffer size is 0 
    - if the write queue size is full or queue depth is insufficient
    - if the driver handle is invalid 

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_NVM_EVENT_COMMAND_COMPLETE event if the buffer
    was processed successfully or DRV_NVM_EVENT_COMMAND_ERROR event if 
    the buffer was not processed successfully.

  Remarks:
    Refer to drv_nvm.h for usage information.
*/

void DRV_NVM_EraseWrite
(
    const DRV_HANDLE handle,
    DRV_NVM_COMMAND_HANDLE * commandHandle,
    void * sourceBuffer,
    uint32_t writeBlockStart,
    uint32_t nWriteBlock
)
{
    int iEntry;
    DRV_NVM_BUFFER_OBJECT * bufferObj;
    DRV_NVM_CLIENT_OBJECT * clientObj;
    DRV_NVM_OBJECT * dObj;
    bool interruptWasEnabled;
    uint32_t erasePageAddress;

    DRV_NVM_COMMAND_HANDLE * tempHandle1, tempHandle2;
    
    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_NVM_COMMAND_HANDLE_INVALID;

    /* Validate the driver handle */
    clientObj = _DRV_NVM_ClientHandleValidate(handle);
    _DRV_NVM_VALIDATE_EXPR((clientObj == NULL), (void)0);
    _DRV_NVM_VALIDATE_EXPR((sourceBuffer == NULL), (void)0);
    _DRV_NVM_VALIDATE_EXPR(((writeBlockStart + nWriteBlock) > gNVMGeometryTable[GEOMETRY_TABLE_WRITE_ENTRY].numBlocks), (void)0);
    _DRV_NVM_VALIDATE_EXPR((nWriteBlock == 0), (void)0);

    /* Check if the driver was opened for write */
    _DRV_NVM_VALIDATE_EXPR((!(clientObj->intent & DRV_IO_INTENT_WRITE)), (void)0);

    dObj = clientObj->driverObj;
  
    writeBlockStart *= DRV_NVM_ROW_SIZE;
    writeBlockStart += dObj->blockStartAddress;
    nWriteBlock     *= DRV_NVM_ROW_SIZE;

    /* Address should be row aligned */
    _DRV_NVM_VALIDATE_EXPR(((writeBlockStart % DRV_NVM_ROW_SIZE) != 0), (void)0);

    /* Disable the interrupt so that any write operation from an interrupt 
     * context does not interfere. 
     * */
    interruptWasEnabled = _DRV_NVM_InterruptSourceDisable (dObj->interruptSource);

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
            bufferObj->hClient              = handle;
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

                _DRV_NVM_EraseBufferObjProcess(dObj, bufferObj);
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

    if(interruptWasEnabled)
    {
        /* Re-Enable the interrupt if it was enabled */
        _DRV_NVM_InterruptSourceEnable(dObj->interruptSource);
    }

    return;
}
