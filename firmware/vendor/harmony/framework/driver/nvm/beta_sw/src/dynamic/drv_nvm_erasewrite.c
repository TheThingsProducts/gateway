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

#include "driver/nvm/beta_sw/src/drv_nvm_local.h"

// *****************************************************************************
/* Function:
    DRV_NVM_BUFFER_HANDLE DRV_NVM_EraseWrite
    (
        const DRV_HANDLE handle,  
        uint8_t * targetbuffer,
	    uint8_t * srcAddress,  
        const unsigned int numbytes 
    )

  Summary:
    Erases and then writes a block of data to a specified address in flash memory.

  Description:
    This routine erases the flash memory block to be written to and then writes
    to it.  It returns a buffer handle that can be queried by the
    DRV_NVM_BufferStatus() function to check for completion of the operation.
    The contents of the source buffer should not be changed while the operation
    is in progress.  The target address should be aligned on a DRV_NVM_ROW_SIZE
    byte boundary.  The number of bytes to write should be equal to or should be
    multiples of DRV_NVM_ROW_SIZE. Successful completion of the operation is
    also indicated by a DRV_NVM_EVENT_BUFFER_COMPLETE. This event is generated
    if a event handler callback function was registered. 

    This function combines the step of erasing a page and then writing the row.
    The application can use this function if it wants to avoid having to
    explicitly delete a page in order to update the rows contained in the page. 

  Remarks:
    Refer to drv_nvm.h for usage information.
*/



DRV_NVM_BUFFER_HANDLE DRV_NVM_EraseWrite
(
    DRV_HANDLE handle,
    uint8_t * destination,
    uint8_t * source,
    const unsigned int nBytes
)
{
    int iEntry;
    DRV_NVM_BUFFER_OBJECT * bufferObj;
    DRV_NVM_CLIENT_OBJECT * clientObj;
    DRV_NVM_OBJECT * dObj;
    bool interruptWasEnabled;
    uint32_t erasePageAddress;
    
    /* Validate the driver handle */
    clientObj = _DRV_NVM_ClientHandleValidate(handle);

    if(NULL == clientObj)
    {
        /* Handle is not valid */
        return (DRV_NVM_BUFFER_HANDLE_INVALID);
    }

    if((NULL == destination) || (NULL == source)
            ||(0 == nBytes))
    {
        return (DRV_NVM_BUFFER_HANDLE_INVALID);
    }

    if(!(clientObj->intent & DRV_IO_INTENT_WRITE))
    {
        /* Driver was not opened for write */
        return (DRV_NVM_BUFFER_HANDLE_INVALID);
    }
  
    if((uint32_t)(destination) % DRV_NVM_ROW_SIZE != 0)
    {
        /* Address should be row aligned */
        return(DRV_NVM_BUFFER_HANDLE_INVALID);
    }

    dObj = clientObj->driverObj;

    /* Disable the interrupt so that any
     * write operation from an interrupt context
     * does not interfere. */

    interruptWasEnabled = _DRV_NVM_InterruptSourceDisable (dObj->interruptSource);

    for(iEntry = 0; iEntry < DRV_NVM_BUFFER_OBJECT_NUMBER; iEntry ++)
    {
        if(gDrvNVMBufferObject[iEntry].inUse == false)
        {
            /* Found a buffer object that can be used */

            bufferObj                       = &gDrvNVMBufferObject[iEntry];
            bufferObj->inUse                = true;
            bufferObj->appDataPointer       = source;
            bufferObj->flashMemPointer      = destination;
            bufferObj->size                 = nBytes;
            bufferObj->status               = DRV_NVM_BUFFER_QUEUED;
            bufferObj->flag                 = DRV_NVM_BUFFER_FLAG_ERASE_WRITE;
            bufferObj->hClient              = handle;
            bufferObj->bufferHandle         = (gDrvNVMBufferToken << 16) | iEntry;
            bufferObj->next                 = NULL;
            bufferObj->previous             = NULL;

            /* Set the nBlockPending and block size to 0 to indicate that
             * we have not started processing this yet. */

            bufferObj->nBlocksPending       = 0;
            bufferObj->blockSize            = 0;

            /* Update the token number. If it reaches 0xFFFF
             * then reset it*/

            gDrvNVMBufferToken ++;
            gDrvNVMBufferToken = (gDrvNVMBufferToken == 0xFFFF) ? 0 : gDrvNVMBufferToken;

            if(dObj->writeEraseQ == NULL)
            {
                /* This means the queue is empty */
                dObj->writeEraseQ = bufferObj;

                /* For an erase write request the number of blocks pending
                 * first will 1 and a page will be erased. The total number
                 * of blocks to program is tracked in the hardware instance
                 * object.  */
                dObj->nRowsPending = nBytes/DRV_NVM_ROW_SIZE;
                dObj->appDataMemory = source;
                dObj->eraseWriteStartAddress = (uint32_t)destination;
                dObj->eraseWriteStep = DRV_NVM_ERASE_WRITE_STEP_ERASE_COMPLETE;
                                    
                /* Obtain the page address that contains this row and then
                 * set up the buffer object to erase it */

                erasePageAddress = ((uint32_t)destination / DRV_NVM_PAGE_SIZE) * DRV_NVM_PAGE_SIZE;
                bufferObj->status = DRV_NVM_BUFFER_IN_PROGRESS;
                bufferObj->nBlocksPending = 1;
                bufferObj->blockSize = DRV_NVM_PAGE_SIZE;
                bufferObj->flashMemPointer  = (uint8_t *)erasePageAddress;
                
                /* Make a back up of the page to be erased */
                for(iEntry = 0; iEntry < DRV_NVM_PAGE_SIZE; iEntry ++)
                {
                    dObj->eraseBuffer[iEntry] = bufferObj->flashMemPointer[iEntry];
                }

                PLIB_NVM_MemoryModifyInhibit(dObj->moduleId);
                _DRV_NVM_EraseBufferObjProcess(dObj, bufferObj);
            }

            else
            {
                /* This means there is already a buffer queued
                 * up. We add the buffer to the linked list */
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

            if(interruptWasEnabled)
            {
                _DRV_NVM_InterruptSourceEnable(dObj->interruptSource);
            }

            /*OSAL Mutex UnLock */
            return((DRV_NVM_BUFFER_HANDLE)bufferObj->bufferHandle);
        }
    }

    /* Execution reaches here if a buffer object could not be allocated */

    if(interruptWasEnabled)
    {
        /* Re-Enable the interrupt if it was enabled */
        _DRV_NVM_InterruptSourceEnable(dObj->interruptSource);
    }

    /* Could not find a buffer object */
    return(DRV_NVM_BUFFER_HANDLE_INVALID);
}
