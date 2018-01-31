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
    microcontroller.  This file implements the NVM Driver interface. This file
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

#include "driver/nvm/beta_sw/src/drv_nvm_local.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global objects
// *****************************************************************************
// *****************************************************************************

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
 * This token is incremented for every requst
 * added the queue and is used to generate
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
     * and then enabled flash operations */
    PLIB_NVM_MemoryModifyInhibit(plibId);
    PLIB_NVM_MemoryOperationSelect(plibId, mode);
    PLIB_NVM_MemoryModifyEnable(plibId);

    /* Write the keys. The _DRV_NVM_UnlockKeySequence
     * function on PIC32MZ device provides the additional
     * step of write 0 to theh NVM Key register. */
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
    Returns NULL if the client handle is not valid

  Description:
    This function returns NULL if the client handle is invalid else return the
    client object associated with the handle.

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

DRV_NVM_CLIENT_OBJECT * _DRV_NVM_ClientHandleValidate(DRV_HANDLE handle)
{
    DRV_NVM_CLIENT_OBJECT * clientObj;
    DRV_NVM_OBJECT * dObj;

    /* Validate the handle */
    if((0 == handle)|| (DRV_HANDLE_INVALID == handle))
    {
        return NULL;
    }

    /* See if the client has been opened */

    clientObj = (DRV_NVM_CLIENT_OBJECT *)handle;

    if(!clientObj->inUse)
    {
        return NULL;
    }

    /* Check if the driver not ready for operation */
    dObj = (DRV_NVM_OBJECT *)clientObj->driverObj;
    if(dObj->status != SYS_STATUS_READY)
    {
        /* Associated hardware instance object is not ready for
         * any operation */
        return NULL;
    }

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
    PLIB_NVM_FlashAddressToModify(dObj->moduleId , _DRV_NVM_KVA_TO_PA((uint32_t)(bufferObj->flashMemPointer)));
    PLIB_NVM_DataBlockSourceAddress ( dObj->moduleId , _DRV_NVM_KVA_TO_PA((uint32_t)(bufferObj->appDataPointer))) ;
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
    PLIB_NVM_FlashAddressToModify(dObj->moduleId , _DRV_NVM_KVA_TO_PA((uint32_t)(bufferObj->flashMemPointer)));
    _DRV_NVM_UnlockSequence(dObj->moduleId, dObj, _DRV_PAGE_ERASE_OPERATION);
}

// *****************************************************************************
/* Function:
    DRV_NVM_BUFFER_HANDLE _DRV_NVM_BlockOperation
    (
        DRV_HANDLE handle,
        uint8_t * destination,
        uint8_t * source,
        const unsigned int nBytes,
        DRV_NVM_BUFFER_FLAGS flag,
        uint32_t blockSize
    )

  Summary:
    Block write and erase assit function.

  Description:
    This function executes a block write or block erase .

  Remarks:
    This is a local function and should not be called directly by the
    application.
*/

DRV_NVM_BUFFER_HANDLE _DRV_NVM_BlockOperation
(
    DRV_HANDLE handle,
    uint8_t * destination,
    uint8_t * source,
    const unsigned int nBytes,
    DRV_NVM_BUFFER_FLAGS flag,
    uint32_t blockSize
)
{
    int iEntry;
    DRV_NVM_BUFFER_OBJECT * bufferObj;
    DRV_NVM_CLIENT_OBJECT * clientObj;
    DRV_NVM_OBJECT * dObj;

    /* Validate the driver handle */
    clientObj = _DRV_NVM_ClientHandleValidate(handle);

    if(NULL == clientObj)
    {
        /* Handle is not valid */
        return (DRV_NVM_BUFFER_HANDLE_INVALID);
    }

    if((NULL == destination) || (NULL == source) ||(0 == nBytes))
    {
        /* Input parameters are not valid */
        return (DRV_NVM_BUFFER_HANDLE_INVALID);
    }

    if(!(clientObj->intent & DRV_IO_INTENT_WRITE))
    {
        /* Driver was not opened for write */
        return (DRV_NVM_BUFFER_HANDLE_INVALID);
    }

    if((uint32_t)(destination) % blockSize != 0)
    {
        /* Address should be row aligned */
        return(DRV_NVM_BUFFER_HANDLE_INVALID);
    }

    dObj = clientObj->driverObj;

    /* Disable the interrupt so that any write operation from an interrupt
     * context does not interfere. */

    dObj->intStatus = _DRV_NVM_InterruptSourceDisable (dObj->interruptSource);

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
            bufferObj->flag		    = flag;
            bufferObj->hClient              = handle;
            bufferObj->bufferHandle         = (gDrvNVMBufferToken << 16) | iEntry;
            bufferObj->next                 = NULL;
            bufferObj->previous             = NULL;
            bufferObj->nBlocksPending       = (((uint32_t)(nBytes))/blockSize);
            bufferObj->blockSize            = blockSize;

            /* Update the token number. If it reaches 0xFFFF
             * then reset it */
            gDrvNVMBufferToken ++;
            gDrvNVMBufferToken = (gDrvNVMBufferToken == 0xFFFF) ? 0 : gDrvNVMBufferToken;

            if(dObj->writeEraseQ == NULL)
            {
                /* Because the write Q is NULL,
                 * we can start an operation */

                dObj->writeEraseQ = bufferObj;
                bufferObj->status = DRV_NVM_BUFFER_IN_PROGRESS;
                PLIB_NVM_MemoryModifyInhibit(dObj->moduleId);

                if(DRV_NVM_BUFFER_FLAG_WRITE == flag)
                {
                    /* A write operation needs to be performed */
                    _DRV_NVM_WriteBufferObjProcess(dObj, bufferObj);
                }
                else if(DRV_NVM_BUFFER_FLAG_ERASE == flag)
                {
                    /* A read operation needs to be performed */
                    _DRV_NVM_EraseBufferObjProcess(dObj, bufferObj);
                }
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

            if(dObj->intStatus)
            {
                /* Enable interrupts if they were already enabled */
                _DRV_NVM_InterruptSourceEnable(dObj->interruptSource);
            }

            /*OSAL Mutex UnLock */
            return((DRV_NVM_BUFFER_HANDLE)bufferObj->bufferHandle);
        }
    }

    if(dObj->intStatus)
    {
        /* Enable interrupts if they were already enabled */
        _DRV_NVM_InterruptSourceEnable(dObj->interruptSource);
    }

    /* Could not find a buffer object */
    return(DRV_NVM_BUFFER_HANDLE_INVALID);
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
    DRV_NVM_OBJECT * dObj = (DRV_NVM_OBJECT*) NULL;
    DRV_NVM_INIT * nvmInit = NULL;

    /* Validate the driver index */
    if ( drvIndex > DRV_NVM_INSTANCES_NUMBER )
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    if(gDrvNVMObj[drvIndex].inUse)
    {
        /* Instance has already been initialized */
        return(SYS_MODULE_OBJ_INVALID);
    }

    /* Assign to the local pointer the init data passed */
    nvmInit = ( DRV_NVM_INIT * )init;

    if (OSAL_MUTEX_Create(&nvmClientObjMutex) != OSAL_RESULT_TRUE)
    {
        SYS_ASSERT(false, "API:DRV_NVM_Initialize - NVM client object mutex can not be created");
        return OSAL_RESULT_FALSE;
    }

    if (OSAL_MUTEX_Create(&nvmBufObjMutex) != OSAL_RESULT_TRUE)
    {
        SYS_ASSERT(false, "API:DRV_NVM_Initialize - NVM buffer object mutex can not be created");
        return OSAL_RESULT_FALSE;
    }

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

void  DRV_NVM_Deinitialize
(
    SYS_MODULE_OBJ object
)
{
    DRV_NVM_OBJECT * dObj = (DRV_NVM_OBJECT*)NULL;

    /* Validate the object */

    if((SYS_MODULE_OBJ_INVALID == object)
            ||(object >= DRV_NVM_INSTANCES_NUMBER))
    {
        return;
    }

    dObj = (DRV_NVM_OBJECT*) &gDrvNVMObj[object];

    /* Disable the Interrupt */
    _DRV_NVM_InterruptSourceDisable(dObj->interruptSource);

    /* Reset the client count and the exclusive
     * flag */
    dObj->numClients = 0;
    dObj->IsExclusive = false;

    /* Reset the queue */
    dObj->writeEraseQ = NULL;

    /* Set the Hardware instance object status an un-initialized */
    dObj->status = SYS_STATUS_UNINITIALIZED;

    /* Hardware instance object is is no more in use */
    dObj->inUse = false;

    if (OSAL_MUTEX_Delete(&nvmClientObjMutex) != OSAL_RESULT_TRUE)
    {
        SYS_ASSERT(false, "API:DRV_NVM_DeInitialize - NVM client object mutex can not be deleted");
    }

    if (OSAL_MUTEX_Delete(&nvmBufObjMutex) != OSAL_RESULT_TRUE)
    {
        SYS_ASSERT(false, "API:DRV_NVM_DeInitialize - NVM buffer object mutex can not be deleted");
    }
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

SYS_STATUS  DRV_NVM_Status( SYS_MODULE_OBJ object  )
{
    /* Validate the object */

    if((SYS_MODULE_OBJ_INVALID == object)
            ||(object >= DRV_NVM_INSTANCES_NUMBER))
    {
        /* Object is invalid */
        return (SYS_STATUS_UNINITIALIZED);
    }

    /* Return the driver status */
    return(gDrvNVMObj[object].status );
}

// *****************************************************************************
// *****************************************************************************
// Section: NVM Driver Client Routines
// *****************************************************************************
// *****************************************************************************

// **************************************************************************
/* Function:
    DRV_HANDLE DRV_NVM_Open
    (
        const SYS_MODULE_INDEX index,
        const DRV_IO_INTENT ioIntent
    );

  Summary:
    Opens the specified timer driver instance and returns a handle to it

  Description:
    This routine opens the specified NVM driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver.

  Remarks:
    Refer to drv_nvm.h for usage information.
*/


DRV_HANDLE  DRV_NVM_Open
(
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
)
{
    DRV_NVM_CLIENT_OBJECT * clientObj = (DRV_NVM_CLIENT_OBJECT*) gDrvNVMClientObj;
    DRV_NVM_OBJECT * dObj;

    unsigned int iClient;

    /* Validate the driver index */
    if (drvIndex >= DRV_NVM_INSTANCES_NUMBER)
    {
        return DRV_HANDLE_INVALID;
    }

    dObj = &gDrvNVMObj[drvIndex];
    if(dObj->status != SYS_STATUS_READY)
    {
        /* Driver is not ready to be opened */
        return DRV_HANDLE_INVALID;
    }

    if(dObj->IsExclusive)
    {
        /* Driver is already opened in exclusive mode */
        return DRV_HANDLE_INVALID;
    }

    if((dObj->numClients > 0) && (ioIntent & DRV_IO_INTENT_EXCLUSIVE))
    {
        /* Driver has already been opened and cannot be
         * opened exclusively */
        return DRV_HANDLE_INVALID;
    }

    if (OSAL_MUTEX_Lock(&nvmClientObjMutex,OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
    {
        SYS_ASSERT(false, "API:DRV_NVM_Deinitialize - NVM client object mutex can not be locked");
        return OSAL_RESULT_FALSE;
    }

    /* Find available slot in array of client objects */
    for (iClient = 0; iClient < DRV_NVM_CLIENTS_NUMBER ; iClient++)
    {
        if ( !clientObj->inUse )
        {
            /* Found a client object that can be used */
            clientObj->inUse = true;
            clientObj->driverObj =  dObj;
            clientObj->status = DRV_NVM_CLIENT_STATUS_READY;
            clientObj->intent = ioIntent;
            clientObj->eventHandler = NULL;
            if(ioIntent & DRV_IO_INTENT_EXCLUSIVE)
            {
                /* Driver was opened in exclusive mode */
                dObj->IsExclusive = true;
            }
            dObj->numClients ++;

            OSAL_MUTEX_Unlock(&nvmClientObjMutex);
            /* Found the object */
            return ( (DRV_HANDLE) clientObj);
        }
        clientObj += 1;
    }

    OSAL_MUTEX_Unlock(&nvmClientObjMutex);

    /* Couldn't find open slot in object array */
    return  DRV_HANDLE_INVALID ;
}

// *****************************************************************************
/* Function:
    void DRV_NVM_Close( DRV_Handle handle )

  Summary:
    Closes an opened-instance of the NVM driver

  Description:
    This routine closes an opened-instance of the NVM driver, invalidating the
    handle.

  Remarks:
    Refer to drv_nvm.h for usage infomration.
*/

void DRV_NVM_Close( const DRV_HANDLE handle)
{
    DRV_NVM_CLIENT_OBJECT * clientObj;
    DRV_NVM_OBJECT * dObj;
    DRV_NVM_BUFFER_OBJECT * bufferObject;

    /* Get the Client object from the handle passed */
    clientObj = _DRV_NVM_ClientHandleValidate(handle);

    if(NULL == clientObj)
    {
        /* Driver handle is not valid */
        return;
    }

    dObj = clientObj->driverObj;

    /* Disable the interrupt */
    dObj->intStatus = _DRV_NVM_InterruptSourceDisable(dObj->interruptSource);

    /* Remove buffer object related to this client
     * from the driver queue */
    bufferObject = dObj->writeEraseQ;
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
           }
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
    dObj->IsExclusive = false;

    /* Free the Client Instance */
    clientObj->inUse = false;
    clientObj->status = DRV_NVM_STATUS_INVALID;

    return;
}

// *****************************************************************************
/* Function:
    DRV_NVM_BUFFER_HANDLE DRV_NVM_Read
    (
        const DRV_HANDLE handle,
        uint8_t * targetbuffer,
	    uint8_t * srcAddress,
        const unsigned int numbytes
    )

  Summary:
    Reads a block of data from the specified address in memory.

  Description:
    This routine reads a block of data from the specified address in memory.
    It returns a buffer handle that can be queried by the DRV_NVM_BufferStatus()
    function to check for completion of the operation.

  Remarks:
    Refer to drv_nvm.h for usage information.
*/

DRV_NVM_BUFFER_HANDLE DRV_NVM_Read
(
    const DRV_HANDLE hClient,
    uint8_t * destination,
    uint8_t * sourceBuffer,
    const unsigned int nBytes
)
{
    int iEntry,i;
    DRV_NVM_BUFFER_OBJECT * bufferObj;
    DRV_NVM_CLIENT_OBJECT * clientObj;
    DRV_NVM_OBJECT * dObj;
    uint8_t * source = sourceBuffer;

    clientObj = _DRV_NVM_ClientHandleValidate(hClient);

    if(NULL == clientObj)
    {
        /* Client handle is not valid */
        return (DRV_NVM_BUFFER_HANDLE_INVALID);
    }

    if((NULL == destination ) || (NULL == source)
            || (0 == nBytes))
    {
        /* Invalid parameters */
        return(DRV_NVM_BUFFER_HANDLE_INVALID);
    }

    if(!(clientObj->intent & DRV_IO_INTENT_READ))
    {
        /* The driver will perform the read only if
         * the client opened up the driver for read */
        return (DRV_NVM_BUFFER_HANDLE_INVALID);
    }

    dObj = clientObj->driverObj;

    if (OSAL_MUTEX_Lock(&nvmBufObjMutex,OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
    {
        SYS_ASSERT(false, "API:DRV_NVM_Close - NVM buffer object mutex can not be locked");
        return OSAL_RESULT_FALSE;
    }

    /* The read buffer function does not need a task routine as it does not
     * block on hardware.  A buffer object is still used to allow the client to
     * track the status of the read. The function will complete the read and
     * exit from the function. */

    for(iEntry = 0; iEntry < DRV_NVM_BUFFER_OBJECT_NUMBER; iEntry ++)
    {
        /* Search for free buffer object to use */

        if(gDrvNVMBufferObject[iEntry].inUse == false)
        {
            /* This means this object can be used */

            bufferObj = &gDrvNVMBufferObject[iEntry];
            bufferObj->inUse = true;
            bufferObj->bufferHandle = (gDrvNVMBufferToken << 16) | iEntry;

            /* Update the token number. If it reaches 0xFFFF
             * then reset it */

            gDrvNVMBufferToken ++;
            gDrvNVMBufferToken = (gDrvNVMBufferToken == 0xFFFF) ? 0 : gDrvNVMBufferToken;

            /* OSAL Unlock Mutex */

            bufferObj->hClient = hClient;
            bufferObj->size = nBytes;
            bufferObj->appDataPointer = destination;
            bufferObj->flashMemPointer = source;
            bufferObj->status = DRV_NVM_BUFFER_IN_PROGRESS;


            for(i = 0; i < nBytes; i ++)
            {

                /* Do the actual read here.
                 * Not using a PLIB call here as the
                 * memory on PIC32 is linear. We may have to
                 * use a PLIB call for other architectures. */

                *destination = *source;
                destination ++;
                source ++;

            }

            bufferObj->status = DRV_NVM_BUFFER_COMPLETED;
            bufferObj->inUse = false;

            OSAL_MUTEX_Unlock(&nvmBufObjMutex);

            return (bufferObj->bufferHandle);
        }
    }

    OSAL_MUTEX_Unlock(&nvmBufObjMutex);

    /* Could not find a free buffer object */
    return DRV_NVM_BUFFER_HANDLE_INVALID;
}

// *****************************************************************************
/* Function:
    DRV_NVM_BUFFER_STATUS DRV_NVM_BufferStatus
    (
        DRV_HANDLE handle,
        const DRV_HANDLE bufferHandle
    );

  Summary:
    Gets the current status of the buffer.

  Description:
    This routine gets the current status of the buffer. The application must
    use this routine in case a polling based implementation is desired.

  Remarks:
    Refer to drv_nvm.h for usage information.
*/

DRV_NVM_BUFFER_STATUS DRV_NVM_BufferStatus
(
    DRV_HANDLE handle,
    const DRV_NVM_BUFFER_HANDLE bufferHandle
)
{
    DRV_NVM_CLIENT_OBJECT * clientObj;
    uint16_t iEntry;

    clientObj = _DRV_NVM_ClientHandleValidate(handle);

    if(NULL == clientObj)
    {
        /* Client handle is not valid */
        return (DRV_NVM_BUFFER_HANDLE_INVALID);
    }

    /* The upper 16 bits of the buffer handle
     * are the token and the lower 16 bits of the
     * are buffer index into the gDrvNVMBufferObject
     * array */

    iEntry = bufferHandle & 0xFFFF;

    /* Compare the buffer handle with buffer handle
     * in the object */

    if(gDrvNVMBufferObject[iEntry].bufferHandle != bufferHandle)
    {
        /* This means that object has been re-used by another
         * request and the handle that client has provided
         * is not valid */
        return (DRV_NVM_BUFFER_HANDLE_INVALID);
    }

    /* Return the last known buffer object status */
    return (gDrvNVMBufferObject[iEntry].status) ;
}

// *****************************************************************************
/* Function:
    DRV_NVM_BUFFER_HANDLE DRV_NVM_Write
    (
        const DRV_HANDLE handle,
        uint8_t * targetbuffer,
	    uint8_t * srcAddress,
        const unsigned int numbytes
    )

  Summary:
    Write a block of data to a specified address in memory.

  Description:
    This routine writes a block of data to a specified address in memory.  It
    returns a buffer handle that can be queried by the DRV_NVM_BufferStatus()
    function to check for completion of the operation. The contents of the
    source buffer should not be changed while the operation is in progress.  The
    target address should be aligned on a DRV_NVM_ROW_SIZE byte boundary.  The
    number of bytes to write should be equal to or should be multiples of
    DRV_NVM_ROW_SIZE.

  Remarks:
    Refer to drv_nvm.h for usage information.
*/

DRV_NVM_BUFFER_HANDLE DRV_NVM_Write
(
    DRV_HANDLE handle,
    uint8_t * destination,
    uint8_t * source,
    const unsigned int nBytes
)
{
    return _DRV_NVM_BlockOperation ( handle, destination, source, nBytes, DRV_NVM_BUFFER_FLAG_WRITE, DRV_NVM_ROW_SIZE );
}

// **************************************************************************
/* Function:
    DRV_NVM_BUFFER_HANDLE DRV_NVM_Erase
    (
        const DRV_HANDLE handle,
        uint8_t * targetbuffer,
        const unsigned int numbytes
    )

  Summary:
    Erase the specified number of pages in flash memory.

  Description:
    This routine erases the specified number of pages in Flash memory. It
    returns a buffer handle that can be queried by the DRV_NVM_BufferStatus()
    function to check for completion of the operation. The target address should
    be aligned on a DRV_NVM_PAGE_SIZE byte boundary. The number of bytes to
    write should be equal to or should be multiples of DRV_NVM_PAGE_SIZE.

  Remarks:
    Refer to drv_nvm.h for usage information.
*/


DRV_NVM_BUFFER_HANDLE DRV_NVM_Erase
(
    DRV_HANDLE handle,
    uint8_t * destination,
    const unsigned int nBytes
)
{
    /* The source while calling the below function is set to destination
     * because the source parameter is ignored for erase operation
     * and it cannot be NULL. */
    return _DRV_NVM_BlockOperation ( handle, destination, destination, nBytes, DRV_NVM_BUFFER_FLAG_ERASE, DRV_NVM_PAGE_SIZE );
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

void DRV_NVM_Tasks(SYS_MODULE_OBJ object)
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

    if(SYS_INT_SourceStatusGet(dObj->interruptSource))
    {
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

                bufferObj->status = DRV_NVM_BUFFER_COMPLETED;
                clientObj = (DRV_NVM_CLIENT_OBJECT *)bufferObj->hClient;
                if(clientObj->eventHandler != NULL)
                {
                    /* Call the event handler */
                    clientObj->eventHandler(DRV_NVM_EVENT_BUFFER_COMPLETE,
                            (DRV_NVM_BUFFER_HANDLE)bufferObj->bufferHandle, clientObj->context);
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
                /* Theere are more blocks pending in the erase write
                 * operation. We need to continue erasing. */
                dObj->eraseWriteStep = DRV_NVM_ERASE_WRITE_STEP_ERASE_NEXT_PAGE;
            }
        }

        /* At this point bufferObj is either pointing to a new
         * bufferObj or an existing one that as not completed
         * yet. */

        if(bufferObj != NULL)
        {
            switch(bufferObj->flag)
            {
                case DRV_NVM_BUFFER_FLAG_WRITE:

                    /* We are either starting a new write or continuing an
                     * existing one. For a new buffer, bufferOffset will
                     * be zero */
                    bufferObj->flashMemPointer += (bufferOffset * bufferObj->blockSize);
                    bufferObj->appDataPointer += (bufferOffset * DRV_NVM_ROW_SIZE);
                    PLIB_NVM_MemoryModifyInhibit(dObj->moduleId);
                    _DRV_NVM_WriteBufferObjProcess(dObj, bufferObj);
                    break;

                case DRV_NVM_BUFFER_FLAG_ERASE:

                    /* We are either starting a new erase or continuing an
                     * existing one. */
                    bufferObj->flashMemPointer += (bufferOffset * bufferObj->blockSize);
                    PLIB_NVM_MemoryModifyInhibit(dObj->moduleId);
                    _DRV_NVM_EraseBufferObjProcess(dObj, bufferObj);
                    break;

                case DRV_NVM_BUFFER_FLAG_ERASE_WRITE:

                    /* We have a new Erase Write request or if an existing
                     * one is continuing. */

                    if((bufferObj->nBlocksPending == 0 )&&(bufferObj->blockSize == 0))
                    {
                        /* This is a completely new request
                         * For an erase write request the number of blocks pending
                         * first will 1 and a page will be erased. The total number
                         * of blocks to program is tracked in the hardware instance
                         * object.  */
                        dObj->nRowsPending = bufferObj->size/DRV_NVM_ROW_SIZE;
                        dObj->appDataMemory = bufferObj->appDataPointer;
                        dObj->eraseWriteStartAddress = (uint32_t)bufferObj->flashMemPointer;
                        dObj->eraseWriteStep = DRV_NVM_ERASE_WRITE_STEP_ERASE_COMPLETE;

                        /* Obtain the page address that contains this row and then
                         * set up the buffer object to erase it */

                        erasePageAddress = ((uint32_t)bufferObj->flashMemPointer / DRV_NVM_PAGE_SIZE) * DRV_NVM_PAGE_SIZE;
                        bufferObj->status = DRV_NVM_BUFFER_IN_PROGRESS;
                        bufferObj->nBlocksPending = 1;
                        bufferObj->blockSize = DRV_NVM_PAGE_SIZE;
                        bufferObj->flashMemPointer  = (uint8_t *)erasePageAddress;

                        /* Make a back up of the page to be erased */
                        for(i = 0; i < DRV_NVM_PAGE_SIZE; i ++)
                        {
                            dObj->eraseBuffer[i] = bufferObj->flashMemPointer[i];
                        }

                        PLIB_NVM_MemoryModifyInhibit(dObj->moduleId);
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
                        PLIB_NVM_MemoryModifyInhibit(dObj->moduleId);
                        _DRV_NVM_WriteBufferObjProcess(dObj, bufferObj);
                    }
                    else if(dObj->eraseWriteStep & DRV_NVM_ERASE_WRITE_STEP_WRITE_PAGE)
                    {
                        /* This is an on going operation */

                        bufferObj->flashMemPointer +=  DRV_NVM_ROW_SIZE;
                        bufferObj->appDataPointer += DRV_NVM_ROW_SIZE;
                        PLIB_NVM_MemoryModifyInhibit(dObj->moduleId);
                        _DRV_NVM_WriteBufferObjProcess(dObj, bufferObj);
                    }
                    else if(dObj->eraseWriteStep & DRV_NVM_ERASE_WRITE_STEP_ERASE_NEXT_PAGE)
                    {
                        /* We have completed erasing and updating a page.
                         * Another page needs to be erased and updated. */

                        bufferObj->flashMemPointer +=  DRV_NVM_ROW_SIZE;
                        bufferObj->blockSize = DRV_NVM_PAGE_SIZE;
                        bufferObj->nBlocksPending = 1;
                        dObj->eraseWriteStep = DRV_NVM_ERASE_WRITE_STEP_ERASE_COMPLETE;

                        /* Make a backup of the page */
                        for(i = 0; i < DRV_NVM_PAGE_SIZE; i++)
                        {
                            dObj->eraseBuffer[i] = bufferObj->flashMemPointer[i];
                        }

                        PLIB_NVM_MemoryModifyInhibit(dObj->moduleId);
                        _DRV_NVM_EraseBufferObjProcess(dObj, bufferObj);
                    }

                    break;
                case DRV_NVM_BUFFER_FLAG_READ:
                default:
                    break;
            }
        }
    }
}

// *****************************************************************************
/* Function:
    void DRV_NVM_BlockEventHandlerSet
    (
        const DRV_HANDLE handle,
        const DRV_NVM_EVENT_HANDLER eventHandler,
        const uintptr_t context
    );

  Summary:
    Allows a client to identify an event handling function for the driver to
    call back when queued operation has completed.

  Description:
    This function allows a client to identify an event handling function
    for the driver to call back when queued operation has completed.
    When a client calls any read, write or erase function, it is provided with a
    handle identifying the buffer that was added to the driver's buffer queue.
    The driver will pass this handle back to the client by calling
    "eventHandler" function when the queued operation has completed.

    The event handler should be set before the client performs any
    read/write/erase operations that could generate events. The event handler
    once set, persists until the client closes the driver or sets another event
    handler (which could be a "NULL" pointer to indicate no callback).

  Remarks:
    Refer to drv_nvm.h for usage information.
*/

void DRV_NVM_BlockEventHandlerSet
(
    const DRV_HANDLE handle,
    const void * eventHandler,
    const uintptr_t context
)
{
    DRV_NVM_CLIENT_OBJECT * clientObj;

    clientObj = _DRV_NVM_ClientHandleValidate(handle);
    if(NULL == clientObj)
    {
        /* Client handle is not valid */
        return;
    }

    /* Set the event handler */
    clientObj->eventHandler = (DRV_NVM_EVENT_HANDLER)eventHandler;
    clientObj->context = context;
}


