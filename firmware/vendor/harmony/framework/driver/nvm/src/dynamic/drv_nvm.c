/*******************************************************************************
  NVM Driver Interface Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_nvm.c

  Summary:
    NVM Driver Interface Definition

  Description:
    The NVM Driver provides a interface to access the NVM on the PIC32
    microcontroller. This file implements the NVM Driver interface. This file
    should be included in the project if NVM driver functionality is needed.
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
// *****************************************************************************
// Section: Global objects
// *****************************************************************************
// *****************************************************************************

/*************************************
 * NVM driver geometry object
 ************************************/
SYS_FS_MEDIA_GEOMETRY *gDrvNVMMediaGeometry;

/*****************************************
 * Media Geomtery Table.
 ****************************************/
SYS_FS_MEDIA_REGION_GEOMETRY *gNVMGeometryTable;

/*************************************************
 * Hardware instance objects
 *************************************************/

DRV_NVM_OBJECT        gDrvNVMObj[DRV_NVM_INSTANCES_NUMBER];

/*************************************************
 * Driver Client Objects
 *************************************************/

DRV_NVM_CLIENT_OBJECT gDrvNVMClientObj[DRV_NVM_CLIENTS_NUMBER];

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

/**************************************************
 * Erase buffer size in case the erase write
 * feature is enabled
 **************************************************/

uint8_t gDrvNVMEraseBuffer[DRV_NVM_INSTANCES_NUMBER][DRV_NVM_ERASE_BUFFER_SIZE] __attribute__((coherent, aligned(16)));

/*************************************************
 * OSAL Declarations
 *************************************************/
/* NVM Client Object Mutex */
OSAL_MUTEX_DECLARE(nvmClientObjMutex);

/* NVM Buffer Object Mutex*/
OSAL_MUTEX_DECLARE(nvmBufObjMutex);

const SYS_FS_MEDIA_FUNCTIONS nvmMediaFunctions =
{
    .mediaStatusGet     = DRV_NVM_IsAttached,
    .mediaGeometryGet   = DRV_NVM_GeometryGet,
    .sectorRead         = DRV_NVM_Read,
    .sectorWrite        = _DRV_NVM_ERASE_WRITE_WRAPPER,
    .eventHandlerset    = DRV_NVM_EventHandlerSet,
    .commandStatusGet   = (void *)DRV_NVM_CommandStatus,
    .Read               = DRV_NVM_Read,
    .erase              = DRV_NVM_Erase,
    .addressGet         = DRV_NVM_AddressGet,
    .open               = DRV_NVM_Open,
    .close              = DRV_NVM_Close,
    .tasks              = NULL,
};

// *****************************************************************************
// *****************************************************************************
// Section: NVM Driver Local Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void DRIVER _DRV_NVM_UnlockSequence
    (
        NVM_MODULE_ID               plibId,
        _DRV_NVM_OPERATION_MODE     mode
    )

  Summary:
    Executes the NVM Unlock Sequence.

  Description:
    Executes the NVM Unlock Sequence.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _DRV_NVM_UnlockSequence
(
    NVM_MODULE_ID plibId,
    DRV_NVM_OBJECT * dObj,
    _DRV_NVM_OPERATION_MODE mode
)
{
    /* Disable the global interrupts */
    dObj->intStatus = SYS_INT_Disable();

    /* Select the desired flash operation
     * and then enable flash operations 
     * */
    PLIB_NVM_MemoryModifyInhibit(plibId);
    PLIB_NVM_MemoryOperationSelect(plibId, mode);
    PLIB_NVM_MemoryModifyEnable(plibId);

    /* Write the keys. The _DRV_NVM_UnlockKeySequence
     * function on PIC32MZ device provides the additional
     * step of write 0 to the NVM Key register.
     * */
    _DRV_NVM_UnlockKeySequence0(plibId, 0);
    PLIB_NVM_FlashWriteKeySequence( plibId, DRV_NVM_PROGRAM_UNLOCK_KEY1 );
    PLIB_NVM_FlashWriteKeySequence( plibId, DRV_NVM_PROGRAM_UNLOCK_KEY2 );

    /* Start the operation */
    PLIB_NVM_FlashWriteStart(plibId);

    /* Wait for the operation to finish.*/
    while(!PLIB_NVM_FlashWriteCycleHasCompleted(plibId));
    
    /* Enable the global interrupts if they were enabled */
    if(dObj->intStatus)
    {
        SYS_INT_Enable();
    }
}

// *****************************************************************************
/* Function:
    DRV_NVM_CLIENT * _DRV_NVM_ClientHandleValidate( DRV_HANDLE handle );

  Summary:
    Validate the driver handle.

  Description:
    This function validates the driver handle and returns the client object pointer
    associated with the driver handle if the handle is valid. If the driver handle
    is not valid or if the driver is in a not ready state then NULL is returned.

  Remarks:
    This is a local function and should not be called directly by the application.
*/

DRV_NVM_CLIENT_OBJECT * _DRV_NVM_ClientHandleValidate
(
    DRV_HANDLE handle
)
{
    DRV_NVM_CLIENT_OBJECT * clientObj;
    DRV_NVM_OBJECT * dObj;

    /* Validate the handle */
    _DRV_NVM_VALIDATE_EXPR((0 == handle), NULL);
    _DRV_NVM_VALIDATE_EXPR((DRV_HANDLE_INVALID == handle), NULL);

    /* See if the client has been opened */
    clientObj = (DRV_NVM_CLIENT_OBJECT *)handle;
    _DRV_NVM_VALIDATE_EXPR((!clientObj->inUse), NULL);

    /* Check if the driver is ready for operation */
    dObj = (DRV_NVM_OBJECT *)clientObj->driverObj;
    _DRV_NVM_VALIDATE_EXPR((dObj->status != SYS_STATUS_READY), NULL);

    return clientObj;
}

// *****************************************************************************
/* Function:
    void _DRV_NVM_WriteBufferObjProcess
    (
        DRV_NVM_OBJECT * dObj,
        DRV_NVM_BUFFER_OBJECT * bufferObj
    )

  Summary:
    Processes a write command object in the queue.

  Description:
    Processes a write command object in the queue.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _DRV_NVM_WriteBufferObjProcess
(
    DRV_NVM_OBJECT * dObj,
    DRV_NVM_BUFFER_OBJECT * bufferObj
)
{
    PLIB_NVM_FlashAddressToModify(dObj->moduleId, _DRV_NVM_KVA_TO_PA((uint32_t)(bufferObj->flashMemPointer)));
    PLIB_NVM_DataBlockSourceAddress(dObj->moduleId, _DRV_NVM_KVA_TO_PA((uint32_t)(bufferObj->appDataPointer))) ;
    _DRV_NVM_UnlockSequence(dObj->moduleId, dObj, _DRV_ROW_PROGRAM_OPERATION);
}

// *****************************************************************************
/* Function:
    void _DRV_NVM_EraseBufferObjProcess
    (
        DRV_NVM_OBJECT * dObj,
        DRV_NVM_BUFFER_OBJECT * bufferObj
    )

  Summary:
    Processes a erase command object in the queue.

  Description:
    Processes a erase command object in the queue.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

void _DRV_NVM_EraseBufferObjProcess
(
    DRV_NVM_OBJECT * dObj,
    DRV_NVM_BUFFER_OBJECT * bufferObj
)
{
    PLIB_NVM_FlashAddressToModify(dObj->moduleId, _DRV_NVM_KVA_TO_PA((uint32_t)(bufferObj->flashMemPointer)));
    _DRV_NVM_UnlockSequence(dObj->moduleId, dObj, _DRV_PAGE_ERASE_OPERATION);
}

// *****************************************************************************
/* Function:
    DRV_NVM_COMMAND_HANDLE _DRV_NVM_BlockOperation
    (
        DRV_HANDLE handle,
        uint8_t *source,
        uint32_t blockStart,
        uint32_t nBlock,
        DRV_NVM_BUFFER_FLAGS flag,
        uint32_t blockSize
    );

  Summary:
    Block write and page erase function.

  Description:
    This function executes a block write or block erase .

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

DRV_NVM_COMMAND_HANDLE _DRV_NVM_BlockOperation
(
    DRV_HANDLE handle,
    uint8_t * source,
    uint32_t blockStart,
    uint32_t nBlock,
    DRV_NVM_BUFFER_FLAGS flag,
    uint32_t blockSize
)
{
    int iEntry;
    DRV_NVM_BUFFER_OBJECT * bufferObj;
    DRV_NVM_CLIENT_OBJECT * clientObj;
    DRV_NVM_OBJECT * dObj;
    OSAL_RESULT retVal;

    /* Validate the driver handle */
    clientObj = _DRV_NVM_ClientHandleValidate(handle);
    _DRV_NVM_VALIDATE_EXPR((NULL == clientObj), DRV_NVM_COMMAND_HANDLE_INVALID);

    /* Check if the driver was opened with write intent */
    _DRV_NVM_VALIDATE_EXPR((!(clientObj->intent & DRV_IO_INTENT_WRITE)), DRV_NVM_COMMAND_HANDLE_INVALID);

    dObj = clientObj->driverObj;

    /* Update the block start address and the number of bytes based on
     * the operation being performed.
     * */
    blockStart *= blockSize;
    blockStart += dObj->blockStartAddress;

    /* Acquire Buffer Object Mutex */
    retVal = OSAL_MUTEX_Lock(&nvmBufObjMutex,OSAL_WAIT_FOREVER);
    _DRV_NVM_VALIDATE_EXPR((retVal != OSAL_RESULT_TRUE), DRV_NVM_COMMAND_HANDLE_INVALID);

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
            bufferObj->hClient         = handle;
            bufferObj->commandHandle    = _DRV_NVM_MAKE_HANDLE(gDrvNVMBufferToken, iEntry);
            bufferObj->next            = NULL;
            bufferObj->previous        = NULL;
            bufferObj->nBlocksPending  = nBlock;
            bufferObj->blockSize       = blockSize;

            /* Update the token number. */
            _DRV_NVM_UPDATE_BUF_TOKEN(gDrvNVMBufferToken);

            /* Disable the interrupt so that any write operation from an interrupt
             * context does not interfere.
             * */
            dObj->intStatus = _DRV_NVM_InterruptSourceDisable (dObj->interruptSource);

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
                    _DRV_NVM_WriteBufferObjProcess(dObj, bufferObj);
                }
                else if(DRV_NVM_BUFFER_FLAG_ERASE == flag)
                {
                    /* A erase operation needs to be performed */
                    _DRV_NVM_EraseBufferObjProcess(dObj, bufferObj);
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

            if(dObj->intStatus)
            {
                /* Enable interrupts if they were already enabled */
                _DRV_NVM_InterruptSourceEnable(dObj->interruptSource);
            }

            /* Release Buffer Object Mutex */
            OSAL_MUTEX_Unlock(&nvmBufObjMutex);

            return((DRV_NVM_COMMAND_HANDLE)bufferObj->commandHandle);
        }
    }

    /* Release Buffer Object Mutex */
    OSAL_MUTEX_Unlock(&nvmBufObjMutex);

    /* Could not find a buffer object */
    return(DRV_NVM_COMMAND_HANDLE_INVALID);
}

// *****************************************************************************
// *****************************************************************************
// Section: NVM Driver System Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_NVM_Initialize
    (
        const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init
    )

  Summary:
    Initializes the NVM instance for the specified driver index

  Description:
    This routine initializes the NVM driver instance for the specified
    driver index, making it ready for clients to open and use it.

  Remarks:
    Refer to drv_nvm.h for usage information.
*/

SYS_MODULE_OBJ DRV_NVM_Initialize
(
    const SYS_MODULE_INDEX drvIndex,
    const SYS_MODULE_INIT *const init
)
{
    OSAL_RESULT retVal;
    DRV_NVM_OBJECT * dObj = (DRV_NVM_OBJECT*) NULL;
    DRV_NVM_INIT * nvmInit = NULL;

    /* Validate the driver index */
    _DRV_NVM_VALIDATE_EXPR((drvIndex > DRV_NVM_INSTANCES_NUMBER), SYS_MODULE_OBJ_INVALID);

    /* Instance has already been initialized */
    _DRV_NVM_VALIDATE_EXPR((gDrvNVMObj[drvIndex].inUse), SYS_MODULE_OBJ_INVALID);

    /* Assign to the local pointer the init data passed */
    nvmInit = (DRV_NVM_INIT *)init;

    retVal = OSAL_MUTEX_Create(&nvmClientObjMutex);
    _DRV_NVM_VALIDATE_EXPR((retVal != OSAL_RESULT_TRUE), SYS_MODULE_OBJ_INVALID);

    retVal = OSAL_MUTEX_Create(&nvmBufObjMutex);
    _DRV_NVM_VALIDATE_EXPR((retVal != OSAL_RESULT_TRUE), SYS_MODULE_OBJ_INVALID);

    dObj = &gDrvNVMObj[drvIndex];

    /* Indicate tha this object is in use */
    dObj->inUse = true;

    /* Update the NVM PLIB Id */
    dObj->moduleId = nvmInit->nvmID;

    /* Initialize the Interrupt Source */
    dObj->interruptSource = nvmInit->interruptSource;

    /* Disable flash access - WREN bit in NVMCON */
    PLIB_NVM_MemoryModifyInhibit(nvmInit->nvmID);

    /* Initialize number of clients */
    dObj->numClients = 0;

    /* Interrupt flag cleared on the safer side */
    _DRV_NVM_InterruptSourceClear(dObj->interruptSource);

    /* Enable the interrupt source in case of interrupt mode */
    _DRV_NVM_InterruptSourceEnable(dObj->interruptSource);

    /* Set the current driver state */
    dObj->status = SYS_STATUS_READY;

    /* Set the erase buffer */
    dObj->eraseBuffer = &gDrvNVMEraseBuffer[drvIndex][0];

    dObj->blockStartAddress = nvmInit->mediaStartAddress;

    gDrvNVMMediaGeometry = (SYS_FS_MEDIA_GEOMETRY *)nvmInit->nvmMediaGeometry;
    gNVMGeometryTable = nvmInit->nvmMediaGeometry->geometryTable;

    _DRV_NVM_RegisterWithSysFs (dObj, drvIndex, nvmMediaFunctions);
    /* Return the driver index and the System Module Object */
    return drvIndex ;
}

// ****************************************************************************
/* Function:
    void DRV_NVM_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the NVM driver module

  Description:
    Deinitializes the specified instance of the NVM driver module,
    disabling its operation (and any hardware). Invalidates all the
    internal data.

  Remarks:
    Refer to drv_nvm.h for usage information.
*/

void DRV_NVM_Deinitialize
(
    SYS_MODULE_OBJ object
)
{
    DRV_NVM_OBJECT * dObj = (DRV_NVM_OBJECT*)NULL;

    /* Validate the object */
    _DRV_NVM_VALIDATE_EXPR((SYS_MODULE_OBJ_INVALID == object), (void)0);
    _DRV_NVM_VALIDATE_EXPR((object >= DRV_NVM_INSTANCES_NUMBER), (void)0);

    dObj = (DRV_NVM_OBJECT*)&gDrvNVMObj[object];

    /* Disable the Interrupt */
    _DRV_NVM_InterruptSourceDisable(dObj->interruptSource);

    /* Reset the client count and the exclusive flag */
    dObj->numClients = 0;
    dObj->isExclusive = false;

    /* Reset the queue */
    dObj->writeEraseQ = NULL;

    /* Set the Hardware instance object status an un-initialized */
    dObj->status = SYS_STATUS_UNINITIALIZED;

    /* Hardware instance object is no more in use */
    dObj->inUse = false;

    OSAL_MUTEX_Delete(&nvmClientObjMutex);
    OSAL_MUTEX_Delete(&nvmBufObjMutex);
}

// ****************************************************************************
/* Function:
    SYS_STATUS DRV_NVM_Status( SYS_MODULE_OBJ object )

  Summary:
    Gets the current status of the NVM driver module.

  Description:
    This routine provides the current status of the NVM driver module.

  Remarks:
    Refer to drv_nvm.h for usage information.
*/

SYS_STATUS DRV_NVM_Status
(
    SYS_MODULE_OBJ object
)
{
    /* Validate the object */
    _DRV_NVM_VALIDATE_EXPR((SYS_MODULE_OBJ_INVALID == object), SYS_STATUS_UNINITIALIZED);
    _DRV_NVM_VALIDATE_EXPR((object >= DRV_NVM_INSTANCES_NUMBER), SYS_STATUS_UNINITIALIZED);

    /* Return the driver status */
    return (gDrvNVMObj[object].status);
}

// *****************************************************************************
// *****************************************************************************
// Section: NVM Driver Client Routines
// *****************************************************************************
// *****************************************************************************

// ****************************************************************************
/* Function:
    DRV_HANDLE DRV_NVM_Open
    ( 
        const SYS_MODULE_INDEX index,
        const DRV_IO_INTENT ioIntent
    )
    
  Summary:
    Opens the specified NVM driver instance and returns a handle to it
  
  Description:
    This routine opens the specified NVM driver instance and provides a handle. 
    This handle must be provided to all other client-level operations to identify
    the caller and the instance of the driver.
  
  Remarks:
    Refer to drv_nvm.h for usage information.
*/

DRV_HANDLE DRV_NVM_Open
(
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
)
{
    DRV_NVM_CLIENT_OBJECT * clientObj = (DRV_NVM_CLIENT_OBJECT*) gDrvNVMClientObj;
    DRV_NVM_OBJECT * dObj;
    OSAL_RESULT retVal;

    unsigned int iClient;

    /* Validate the driver index */
    _DRV_NVM_VALIDATE_EXPR((drvIndex >= DRV_NVM_INSTANCES_NUMBER), DRV_HANDLE_INVALID);

    dObj = &gDrvNVMObj[drvIndex];
    /* Check if the driver is ready to be opened */
    _DRV_NVM_VALIDATE_EXPR((dObj->status != SYS_STATUS_READY), DRV_HANDLE_INVALID);

    /* Check if the driver has already been opened in exclusive mode */
    _DRV_NVM_VALIDATE_EXPR((dObj->isExclusive), DRV_HANDLE_INVALID);

    /* Driver has already been opened and cannot be
     * opened exclusively */
    _DRV_NVM_VALIDATE_EXPR(((dObj->numClients > 0) && (ioIntent & DRV_IO_INTENT_EXCLUSIVE)), DRV_HANDLE_INVALID);

    /* Obtain the Client object mutex */
    retVal = OSAL_MUTEX_Lock(&nvmClientObjMutex,OSAL_WAIT_FOREVER);
    _DRV_NVM_VALIDATE_EXPR((retVal != OSAL_RESULT_TRUE), DRV_HANDLE_INVALID);

    /* Find available slot in array of client objects */
    for (iClient = 0; iClient < DRV_NVM_CLIENTS_NUMBER ; iClient++)
    {
        if ( !clientObj->inUse )
        {
            /* Found a client object that can be used */
            clientObj->inUse = true;
            clientObj->driverObj =  dObj;
            clientObj->intent = ioIntent;
            clientObj->eventHandler = NULL;
            if(ioIntent & DRV_IO_INTENT_EXCLUSIVE)
            {
                /* Driver was opened in exclusive mode */
                dObj->isExclusive = true;
            }
            dObj->numClients ++;

            OSAL_MUTEX_Unlock(&nvmClientObjMutex);
            /* Found the object */
            return ((DRV_HANDLE)clientObj);
        }
        clientObj += 1;
    }

    OSAL_MUTEX_Unlock(&nvmClientObjMutex);

    /* Couldn't find open slot in object array */
    return DRV_HANDLE_INVALID ;
}

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_NVM_Close( DRV_Handle handle )

  Summary:
    Closes an opened-instance of the NVM driver

  Description:
    This routine closes an opened-instance of the NVM driver, invalidating the
    handle.

  Remarks:
    Refer to drv_nvm.h for usage infomration.
*/

void DRV_NVM_Close
(
    const DRV_HANDLE handle
)
{
    DRV_NVM_CLIENT_OBJECT * clientObj;
    DRV_NVM_OBJECT * dObj;
    DRV_NVM_BUFFER_OBJECT * bufferObject;
    DRV_NVM_BUFFER_OBJECT * prevObject = NULL;

    /* Get the Client object from the handle passed */
    clientObj = _DRV_NVM_ClientHandleValidate(handle);
    /* Check if the driver handle is valid */
    _DRV_NVM_VALIDATE_EXPR((NULL == clientObj), (void)0);

    dObj = clientObj->driverObj;

    /* Disable the interrupt */
    dObj->intStatus = _DRV_NVM_InterruptSourceDisable(dObj->interruptSource);

    bufferObject = dObj->writeEraseQ;
    dObj->writeEraseQ = NULL;

    while(bufferObject != NULL)
    {
        /* Check if this buffer object is owned by this client */
        if(bufferObject->hClient == handle)
        {
            bufferObject->inUse = false;

            if(bufferObject->previous != NULL)
            {
                /* This means that this is not the first object in the queue.
                 * Remove this buffer object from the queue */
                bufferObject->previous->next = bufferObject->next;
                bufferObject->previous = NULL;
            }
        }
        else
        {
            bufferObject->previous = prevObject;
            prevObject = bufferObject;
        }
        /* Get the next object in the queue */
        bufferObject = bufferObject->next;
    }
    /* At this point, all object belonging to this client
     * would have been removed from the queue. We now update
     * the driver queue to point to the first buffer to be
     * processed. */
    bufferObject = dObj->writeEraseQ;
    dObj->writeEraseQ = NULL;
    while(bufferObject != NULL)
    {
        if(bufferObject->inUse)
        {
            dObj->writeEraseQ = bufferObject;
            break;
        }
        bufferObject = bufferObject->next;
    }

    if(dObj->intStatus)
    {
        /* Re-enable the interrupt if it was enabled */
        _DRV_NVM_InterruptSourceEnable(dObj->interruptSource);
    }

    /* At this point, if there are any object left in the queue
     * dObj->writeEraseQ will not be NULL, else it will be
     * NULL */

    /* Update the client count */
    dObj->numClients --;
    dObj->isExclusive = false;

    /* Free the Client Instance */
    clientObj->inUse = false;

    return;
}

// *****************************************************************************
/* Function:
    void DRV_NVM_Read
    (
        const DRV_HANDLE handle,
        DRV_NVM_COMMAND_HANDLE * commandHandle,
        void * targetBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    )

  Summary:
    Reads blocks of data from the specified address in memory.

  Description:
    This routine reads a block of data from the specified address in memory.
    This operation is non blocking and returns with the required data in the
    target buffer. This function should not be used to read areas of memory 
    which are queued to be programmed or erased. If required, the program or
    erase operations should be allowed to complete. The function returns
    DRV_NVM_COMMAND_HANDLE_INVALID in the commandHandle argument under the 
    following circumstances:
    - if the driver handle is invalid
    - if the target buffer pointer is NULL
    - if the number of blocks to be read is zero or more than the actual number
      of blocks available
    - if a buffer object could not be allocated to the request
    - if the client opened the driver in write only mode

  Remarks:
    Refer to drv_nvm.h for usage information.
*/

void DRV_NVM_Read
(
    const DRV_HANDLE handle,
    DRV_NVM_COMMAND_HANDLE * commandHandle,
    void * targetBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    int iEntry, i;
    DRV_NVM_BUFFER_OBJECT * bufferObj;
    DRV_NVM_CLIENT_OBJECT * clientObj;
    uint8_t * source = NULL;
    DRV_NVM_COMMAND_HANDLE * tempHandle1, tempHandle2;
    uint8_t *readBuffer = (uint8_t *)targetBuffer;
    OSAL_RESULT retVal;
    DRV_NVM_OBJECT * dObj;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;

    *tempHandle1 = DRV_NVM_COMMAND_HANDLE_INVALID;    
    clientObj = _DRV_NVM_ClientHandleValidate(handle);

    /* Validate the client handle */
    _DRV_NVM_VALIDATE_EXPR((NULL == clientObj), (void)0);

    /* The driver will perform the read only if
     * the client opened up the driver for read */
    _DRV_NVM_VALIDATE_EXPR((!(clientObj->intent & DRV_IO_INTENT_READ)), (void)0);

    /* Validate the parameters */
    _DRV_NVM_VALIDATE_EXPR((readBuffer == NULL), (void)0);
    _DRV_NVM_VALIDATE_EXPR(((blockStart + nBlock) > gNVMGeometryTable[GEOMETRY_TABLE_READ_ENTRY].numBlocks), (void)0);
    _DRV_NVM_VALIDATE_EXPR((nBlock == 0), (void)0);

    /* Acquire Buffer Object Mutex */
    retVal = OSAL_MUTEX_Lock(&nvmBufObjMutex,OSAL_WAIT_FOREVER);
    _DRV_NVM_VALIDATE_EXPR((retVal != OSAL_RESULT_TRUE), (void)0);

    /* Update the block start address */
    dObj = (DRV_NVM_OBJECT *)clientObj->driverObj;
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
            bufferObj->hClient         = handle;
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

            if(clientObj->eventHandler != NULL)
            {
                /* Call the event handler */
                clientObj->eventHandler(DRV_NVM_EVENT_COMMAND_COMPLETE,
                        (DRV_NVM_COMMAND_HANDLE)bufferObj->commandHandle, clientObj->context);
            }

            bufferObj->inUse = false;

            /* Update the token number. */
            _DRV_NVM_UPDATE_BUF_TOKEN(gDrvNVMBufferToken);
            break;
        }
    }

    /* Release Buffer Object Mutex */
    OSAL_MUTEX_Unlock(&nvmBufObjMutex);

    return;
}

// *****************************************************************************
/* Function:
    void DRV_NVM_Write
    (
        const DRV_HANDLE handle,
        DRV_NVM_COMMAND_HANDLE * commandHandle,
        void * sourceBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    )

  Summary:
    Writes blocks of data starting from the specified address in flash memory.

  Description:
    This function schedules a non-blocking write operation for writing blocks
    of data into flash memory. The function returns with a valid buffer handle
    in the commandHandle argument if the write request was scheduled successfully.
    The function adds the request to the hardware instance queue and returns
    immediately. While the request is in the queue, the application buffer is
    owned by the driver and should not be modified. The function returns 
    DRV_NVM_COMMAND_HANDLE_INVALID in the commandHandle argument under the 
    following circumstances:
    - if a buffer object could not be allocated to the request
    - if the source buffer pointer is NULL
    - if the client opened the driver for read only
    - if the number of blocks to be written is either zero or more than the number
      of blocks actually available
    - if the write queue size is full or queue depth is insufficient
    - if the driver handle is invalid 

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_NVM_EVENT_COMMAND_COMPLETE event if the
    buffer was processed successfully or DRV_NVM_EVENT_COMMAND_ERROR
    event if the buffer was not processed successfully.

  Remarks:
    Refer to drv_nvm.h for usage information.
*/

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
    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;

    *tempHandle1 = DRV_NVM_COMMAND_HANDLE_INVALID;

    _DRV_NVM_VALIDATE_EXPR((sourceBuffer == NULL), (void)0); 
    _DRV_NVM_VALIDATE_EXPR(((blockStart + nBlock) > gNVMGeometryTable[GEOMETRY_TABLE_WRITE_ENTRY].numBlocks), (void)0);
    _DRV_NVM_VALIDATE_EXPR((nBlock == 0), (void)0);

    *tempHandle1 = _DRV_NVM_BlockOperation (handle, sourceBuffer, blockStart, nBlock, 
                                             DRV_NVM_BUFFER_FLAG_WRITE, DRV_NVM_ROW_SIZE);
    return;
}

// **************************************************************************
/* Function:
    void DRV_NVM_Erase
    (
        const DRV_HANDLE handle,
        DRV_NVM_COMMAND_HANDLE * commandHandle,
        uint32_t blockStart,
        uint32_t nBlock
    )
    
  Summary:
    Erase the specified number of blocks of the Flash memory.
  
  Description:
    This function schedules a non-blocking erase operation of flash memory. The
    function returns with a valid erase handle in the commandHandle argument if
    the erase request was scheduled successfully. The function adds the request
    to the hardware instance queue and returns immediately. The function returns
    DRV_NVM_COMMAND_HANDLE_INVALID in the commandHandle argument under the
    following circumstances:
    - if a buffer object could not be allocated to the request
    - if the client opened the driver for read only
    - if the number of blocks to be erased is either zero or more than the number
      of blocks actually available
    - if the erase queue size is full or queue depth is insufficient
    - if the driver handle is invalid 
    
    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_NVM_EVENT_COMMAND_COMPLETE event if the
    erase operation was successful or DRV_NVM_EVENT_COMMAND_ERROR
    event if the erase operation was not successful.

  Remarks:
    Refer to drv_nvm.h for usage information.
*/

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

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;

    *tempHandle1 = DRV_NVM_COMMAND_HANDLE_INVALID;    

    _DRV_NVM_VALIDATE_EXPR(((blockStart + nBlock) > gNVMGeometryTable[GEOMETRY_TABLE_ERASE_ENTRY].numBlocks), (void)0);
    _DRV_NVM_VALIDATE_EXPR((nBlock == 0), (void)0);

    /* The source while calling the below function is set to destination
     * because the source parameter is ignored for erase operation
     * and it cannot be NULL. */
    *tempHandle1 = _DRV_NVM_BlockOperation (handle, (uint8_t *)blockStart, blockStart, nBlock, 
                                            DRV_NVM_BUFFER_FLAG_ERASE, DRV_NVM_PAGE_SIZE);
    return;
}

// *****************************************************************************
/* Function:
    DRV_NVM_COMMAND_STATUS DRV_NVM_CommandStatus
    (
        const DRV_HANDLE handle, 
        const DRV_NVM_COMMAND_HANDLE commandHandle
    );

  Summary:
    Gets the current status of the command.

  Description:
    This routine gets the current status of the buffer. The application must use
    this routine where the status of a scheduled buffer needs to polled on. The
    function may return DRV_NVM_COMMAND_HANDLE_INVALID in a case where the buffer
    handle has expired. A buffer handle expires when the internal buffer object
    is re-assigned to another erase or write request. It is recommended that this
    function be called regularly in order to track the buffer status correctly.

    The application can alternatively register an event handler to receive write
    or erase operation completion events.

  Remarks:
    Refer to drv_nvm.h for usage information.
*/

DRV_NVM_COMMAND_STATUS DRV_NVM_CommandStatus
(
    const DRV_HANDLE handle,
    const DRV_NVM_COMMAND_HANDLE commandHandle
)
{
    uint16_t iEntry;

    /* Validate the client handle */
    _DRV_NVM_VALIDATE_EXPR((NULL == _DRV_NVM_ClientHandleValidate(handle)), DRV_NVM_COMMAND_HANDLE_INVALID);

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

// ****************************************************************************
/* Function:
    void DRV_NVM_Tasks ( SYS_MODULE_OBJ object );

  Summary:
    Maintains the driver's erase and write state machine and implements its
    ISR.

  Description:
    This routine is used to maintain the driver's internal write and erase state
    machine and implement its ISR for interrupt-driven implementations.

  Remarks:
    Refer to drv_nvm.h for usage information.
*/

void DRV_NVM_Tasks
(
    SYS_MODULE_OBJ object
)
{
    DRV_NVM_OBJECT * dObj;
    DRV_NVM_BUFFER_OBJECT * bufferObj;
    DRV_NVM_CLIENT_OBJECT * clientObj;
    unsigned int bufferOffset = 1;
    unsigned int offset, i;
    unsigned int blocksInPage;
    unsigned int erasePageAddress;

    if(SYS_MODULE_OBJ_INVALID == object)
    {
        /* Invalid system object */
        return;
    }

    dObj = &gDrvNVMObj[object];

    if(dObj->status != SYS_STATUS_READY)
    {
        /* The hardware instance is not ready */
        return;
    }

    if(!SYS_INT_SourceStatusGet(dObj->interruptSource))
    {
        return;
    }

    SYS_INT_SourceStatusClear(dObj->interruptSource);

    /* Get the object at the head of the write queue */
    bufferObj = dObj->writeEraseQ;

    if(bufferObj == NULL)
    {
        /* This could happen if the client that is associated with this
         * request has closed the driver and the requet has been removed
         * from the queue */
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
            clientObj = (DRV_NVM_CLIENT_OBJECT *)bufferObj->hClient;
            if(clientObj->eventHandler != NULL)
            {
                /* Call the event handler */
                clientObj->eventHandler(DRV_NVM_EVENT_COMMAND_COMPLETE,
                        (DRV_NVM_COMMAND_HANDLE)bufferObj->commandHandle, clientObj->context);
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
            _DRV_NVM_WriteBufferObjProcess(dObj, bufferObj);
            break;

        case DRV_NVM_BUFFER_FLAG_ERASE:

            /* We are either starting a new erase or continuing an
             * existing one.
             * */
            bufferObj->flashMemPointer += (bufferOffset * bufferObj->blockSize);
            _DRV_NVM_EraseBufferObjProcess(dObj, bufferObj);
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

                _DRV_NVM_EraseBufferObjProcess(dObj, bufferObj);
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
                _DRV_NVM_WriteBufferObjProcess(dObj, bufferObj);
            }
            else if(dObj->eraseWriteStep & DRV_NVM_ERASE_WRITE_STEP_WRITE_PAGE)
            {
                /* This is an on going operation */
                bufferObj->flashMemPointer += DRV_NVM_ROW_SIZE;
                bufferObj->appDataPointer += DRV_NVM_ROW_SIZE;
                _DRV_NVM_WriteBufferObjProcess(dObj, bufferObj);
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

                _DRV_NVM_EraseBufferObjProcess(dObj, bufferObj);
            }

            break;
        case DRV_NVM_BUFFER_FLAG_READ:
        default:
            break;
    }
}

// *****************************************************************************
/* Function:
    void DRV_NVM_EventHandlerSet
    (
        const DRV_HANDLE handle,
        const void * eventHandler,
        const uintptr_t context
    );

  Summary:
    Allows a client to identify an event handling function for the driver to
    call back when queued operation has completed.

  Description:
    This function allows a client to identify an event handling function for
    the driver to call back when queued operation has completed. When a client
    calls a write or erase function, it is provided with a handle identifying
    the buffer that was added to the driver's buffer queue. The driver will 
    pass this handle back to the client by calling "eventHandler" function when
    the queued operation has completed.
    
    The event handler should be set before the client performs any write or erase
    operations that could generate events. The event handler once set, persists
    until the client closes the driver or sets another event handler (which could
    be a "NULL" pointer to indicate no callback).

  Remarks:
    Refer to drv_nvm.h for usage information.
*/

void DRV_NVM_EventHandlerSet
(
    const DRV_HANDLE handle,
    const void * eventHandler,
    const uintptr_t context
)
{
    DRV_NVM_CLIENT_OBJECT * clientObj;

    clientObj = _DRV_NVM_ClientHandleValidate(handle);
    /* Check if the client handle is valid */
    _DRV_NVM_VALIDATE_EXPR((NULL == clientObj), (void)0);

    /* Set the event handler */
    clientObj->eventHandler = eventHandler;
    clientObj->context = context;
}

// *****************************************************************************
/* Function:
    SYS_FS_MEDIA_GEOMETRY * DRV_NVM_GeometryGet( const DRV_HANDLE handle );

  Summary:
    Returns the geometry of the device.

  Description:
    This API gives the following geometrical details of the NVM Flash:
    - Media Property
    - Number of Read/Write/Erase regions in the flash device
    - Number of Blocks and their size in each region of the device

  Remarks:
    Refer to drv_nvm.h for usage information.
*/

SYS_FS_MEDIA_GEOMETRY * DRV_NVM_GeometryGet
(
    const DRV_HANDLE handle
)
{
    /* Validate the driver handle */
    _DRV_NVM_VALIDATE_EXPR((NULL == _DRV_NVM_ClientHandleValidate(handle)), NULL);
    
    return (gDrvNVMMediaGeometry);
}

// *****************************************************************************
/* Function:
    bool DRV_NVM_isAttached( const DRV_HANDLE handle );

  Summary:
    Returns the physical attach status of the NVM.

  Description:
    This function returns the physical attach status of the NVM. This
    function returns false if the driver handle is invalid otherwise returns
    true.

  Remarks:
    Refer to drv_nvm.h for usage information.
*/

bool DRV_NVM_IsAttached
(
    const DRV_HANDLE handle
)
{
    /* Validate the driver handle */
    _DRV_NVM_VALIDATE_EXPR((NULL == _DRV_NVM_ClientHandleValidate(handle)), false);

   return true;
}

// *****************************************************************************
/* Function:
    bool DRV_NVM_isWriteProtected( const DRV_HANDLE handle );

  Summary:
    Returns the write protect status of NVM.

  Description:
    This function returns the write protect status of the NVM. Always returns
    false.

  Remarks:
    Refer to drv_nvm.h for usage information.
*/

bool DRV_NVM_IsWriteProtected
(
    const DRV_HANDLE handle
)
{
    /* This function always returns false */
    return false;
}

// *****************************************************************************
/* Function:
    uintptr_t DRV_NVM_AddressGet
    (
        const DRV_HANDLE handle
    );

  Summary:
    Returns the NVM media start address

  Description:
    This function returns the NVM Media start address.

  Example:
    <code>

    uintptr_t startAddress;
    startAddress = DRV_NVM_AddressGet(drvNVMHandle);

    </code>

  Remarks:
    None.
*/
uintptr_t DRV_NVM_AddressGet
(
    const DRV_HANDLE handle
)
{
    DRV_NVM_CLIENT_OBJECT * clientObj;
    DRV_NVM_OBJECT * dObj;

    /* Validate the handle */
    _DRV_NVM_VALIDATE_EXPR((0 == handle), (uintptr_t)NULL);
    _DRV_NVM_VALIDATE_EXPR((DRV_HANDLE_INVALID == handle), (uintptr_t)NULL);

    /* See if the client has been opened */
    clientObj = (DRV_NVM_CLIENT_OBJECT *)handle;
    _DRV_NVM_VALIDATE_EXPR((!clientObj->inUse), (uintptr_t)NULL);

    /* Check if the driver is ready for operation */
    dObj = (DRV_NVM_OBJECT *)clientObj->driverObj;
    _DRV_NVM_VALIDATE_EXPR((dObj->status != SYS_STATUS_READY), (uintptr_t)NULL);

    return dObj->blockStartAddress;
}

