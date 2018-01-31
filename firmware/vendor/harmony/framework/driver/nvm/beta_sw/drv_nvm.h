/*******************************************************************************
  NVM Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_nvm.h

  Summary:
    NVM Driver Interface Definition.

  Description:
    The NVM device driver provides a simple interface to manage the NVM modules
    on Microchip microcontrollers. This file defines the interface definition for 
    the NVM driver.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012-2014 released Microchip Technology Inc.  All rights reserved.

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
#ifndef _DRV_NVM_H
#define _DRV_NVM_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
/* Note:  A file that maps the interface definitions above to appropriate static
          implementations (depending on build mode) is included at the bottom of
          this file.
*/
#include "system/common/sys_common.h"

#include "driver/driver_common.h"

#include "system/common/sys_module.h"

#include "system/int/sys_int.h"

#include "osal/osal.h"

#include "peripheral/nvm/plib_nvm.h"

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Driver NVM Module Index reference

  Summary:
    NVM driver index definitions

  Description:
    These constants provide NVM driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the DRV_NVM_Initialize and DRV_NVM_Open
    routines to identify the driver instance in use.
*/

#define      DRV_NVM_INDEX_0      0
#define      DRV_NVM_INDEX_1      1

// *****************************************************************************
/* NVM Driver Module Index Count

  Summary:
    Number of valid NVM driver indices

  Description:
    This constant identifies NVM driver index definitions.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.
    This value is part-specific.
*/

#define DRV_NVM_INDEX_COUNT     1

// *****************************************************************************
/* NVM Driver Buffer Handle 

  Summary:
    This type defines the NVM Driver Buffer handle.

  Description:
    This type defines the NVM Driver Buffer handle.  

  Remarks:
    None.
*/

typedef uintptr_t DRV_NVM_BUFFER_HANDLE;

// *****************************************************************************
/* NVM Driver Buffer Invalid Handle 

  Summary:
    This value defines the NVM Driver Buffer Invalid handle.

  Description:
    This value defines the NVM Driver Buffer Invalid handle. This value is
    returned by read/write/erase routines when the desired operation could
    not be completed.

  Remarks:
    None.
*/

#define DRV_NVM_BUFFER_HANDLE_INVALID /* DOM-IGNORE-BEGIN */ ((DRV_NVM_BUFFER_HANDLE)-1) /* DOM-IGNORE-END */

// *****************************************************************************
/* NVM Client Status

  Summary
    Defines the client status

  Description
    Defines the various client status codes.

  Remarks:
    None
*/

typedef enum
{
    /* Up and running, ready to start new operations */
    DRV_NVM_CLIENT_STATUS_READY
        /* DOM-IGNORE-BEGIN */  = DRV_CLIENT_STATUS_READY + 0 /* DOM-IGNORE-END */,

    /* Operation in progress, unable to start a new one */
    DRV_NVM_CLIENT_STATUS_BUSY
        /* DOM-IGNORE-BEGIN */  = DRV_CLIENT_STATUS_BUSY      /* DOM-IGNORE-END */,

    /* Write operation terminated Occurred */
    DRV_NVM_WRITE_TERMINATED
        /* DOM-IGNORE-BEGIN */  = DRV_CLIENT_STATUS_ERROR - 0 /* DOM-IGNORE-END */,

    /* Erase operation terminated Occurred */
    DRV_NVM_ERASE_TERMINATED
        /* DOM-IGNORE-BEGIN */  = DRV_CLIENT_STATUS_ERROR - 1 /* DOM-IGNORE-END */,

    /* Low voltage Error */
    DRV_NVM_LOW_VOLTAGE_ERROR
        /* DOM-IGNORE-BEGIN */  = DRV_CLIENT_STATUS_ERROR - 2 /* DOM-IGNORE-END */,

    /* Low voltage Error */
    DRV_NVM_LOW_VOLTAGE_DETECT_ERROR
        /* DOM-IGNORE-BEGIN */  = DRV_CLIENT_STATUS_ERROR - 3 /* DOM-IGNORE-END */,

    /* Client Invalid */
    DRV_NVM_STATUS_INVALID
        /* DOM-IGNORE-BEGIN */  = DRV_CLIENT_STATUS_ERROR - 4 /* DOM-IGNORE-END */

} DRV_NVM_CLIENT_STATUS;

// *****************************************************************************
/* NVM Driver Initialization Data

  Summary:
    Defines the data required to initialize or reinitialize the NVM driver

  Description:
    This data type defines the data required to initialize or reinitialize the
    NVM driver.

  Remarks:
    Not all initialization features are available for all devices. Please
	refer to the specific device data sheet to determine availability.
*/

typedef struct
{
    /* System module initialization */
    SYS_MODULE_INIT     moduleInit;

    /* Identifies NVM hardware module (PLIB-level) ID */
    NVM_MODULE_ID       nvmID;

    /* Interrupt Source for Write Interrupt */
    INT_SOURCE          interruptSource;
    
} DRV_NVM_INIT;

// ***********************************************************************
/* NVM Driver Buffer Status

  Summary:
    Specifies the status of the buffer for the read, write and erase
    operations.
	
  Description:
    NVM Driver Buffer Status
    
    This type specifies the status of the buffer for the read, write and
    erase operations.
	
  Remarks:
    None.                                                               
*/  

typedef enum
{
    /*Done OK and ready */
    DRV_NVM_BUFFER_COMPLETED          = 0 ,

    /*Scheduled but not started */
    DRV_NVM_BUFFER_QUEUED             = 1,

    /*Currently being in transfer */
    DRV_NVM_BUFFER_IN_PROGRESS        = 2,

    /*Unknown buffer */
    DRV_NVM_BUFFER_ERROR_UNKNOWN      = -1,

} DRV_NVM_BUFFER_STATUS;

// *****************************************************************************
/* NVM Driver Events

   Summary
    Identifies the possible events that can result from a request.

   Description
    This enumeration identifies the possible events that can result from a 
    Write, or Erase request caused by the client. Note that the NVM driver
    does not generate events for read.

   Remarks:
    One of these values is passed in the "event" parameter of the event 
    handling callback function that client registered with the driver by
    calling the DRV_NVM_EventHandlerSet function when a block 
    request is completed.
*/

typedef enum
{
    /* Buffer operation has been completed successfully. */
    /* Read/Write/Erase Complete */
    DRV_NVM_EVENT_BUFFER_COMPLETE 
       /* DOM-IGNORE-BEGIN */ = 1 /* DOM-IGNORE-END */,

    /* There was an error during the block operation */
    /* Read/Write/Erase Error */
    DRV_NVM_EVENT_BUFFER_ERROR
       /* DOM-IGNORE-BEGIN */ = -1 /* DOM-IGNORE-END */

} DRV_NVM_EVENT;

// *****************************************************************************
/* NVM Driver Event Handler Function Pointer

   Summary
    Pointer to a NVM Driver Event handler function

   Description
    This data type defines the required function signature for the 
    NVM event handling callback function. A client must register
    a pointer to an event handling function whose function signature (parameter
    and return value types) match the types specified by this function pointer
    in order to receive event calls back from the driver.
    
    The parameters and return values and return value are described here and
    a partial example implementation is provided.

  Parameters:
    event           - Identifies the type of event
    
    commandHandle   - Handle returned from the Read/Write/Erase requests
    
    context         - Value identifying the context of the application that
                      registered the event handling function

  Returns:
    None.

  Example:
    <code>
    void APP_MyBufferEventHandler
    (
        DRV_NVM_EVENT event,
        DRV_NVM_BUFFER_HANDLE bufferHandle, 
        uintptr_t context
    )
    {
        MY_APP_DATA_STRUCT pAppData = (MY_APP_DATA_STRUCT) context;
        
        switch(event)
        {
            case DRV_NVM_EVENT_BUFFER_COMPLETE:

                // Handle the completed buffer. 
                break;
            
            case DRV_NVM_EVENT_BUFFER_ERROR:
            default:

                // Handle error.
                break;
        }
    }
    </code>

  Remarks:
    If the event is DRV_NVM_EVENT_BUFFER_COMPLETE, it means that the
    write or a erase operation was completed successfully. 
    
    If the event is DRV_NVM_EVENT_BUFFER_ERROR, it means that the scheduled
    operation was not completed successfully.
     
    The context parameter contains the a handle to the client context,  
    provided at the time the event handling function was  registered using the
    DRV_NVM_EventHandlerSet function. This context handle value is
    passed back to the client as the "context" parameter.  It can be any value
    necessary to identify the client context or instance (such as a pointer to
    the client's data) instance of the client that made the read/write/erase
    request.

    The event handler function executes in the driver peripheral's interrupt
    context when the driver is configured for interrupt mode operation. It is
    recommended of the application to not perform process intensive or blocking
    operations with in this function.

*/

typedef void ( * DRV_NVM_EVENT_HANDLER ) 
(
    DRV_NVM_EVENT event, 
    DRV_NVM_BUFFER_HANDLE bufferHandle,
    uintptr_t context
);

// *****************************************************************************
// *****************************************************************************
// Section: NVM Driver Module Interface Routines
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
    Initializes the NVM instance for the specified driver index.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine initializes the NVM driver instance for the specified
    driver index, making it ready for clients to open and use it.

  Precondition:
    None.
  
  Parameters:
    index -  Identifier for the instance to be initialized also the type of
             memory used
    init -   Pointer to a data structure containing any data necessary to
             initialize the driver.
  
  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, it returns SYS_MODULE_OBJ_INVALID.
  
  Example:
    <code>
    // This code snippet shows an example
    // of initializing the NVM Driver.
    
    DRV_NVM_INIT    NVMInitData;
    SYS_MODULE_OBJ  objectHandle;
    
    NVMInitData.moduleInit.value      = SYS_MODULE_POWER_RUN_FULL;
    NVMInitData.moduleId              = NVM_ID_0;
    NVMInitData.flashInterruptSource  = INT_SOURCE_FLASH_CONTROL;
    
    //usage of DRV_NVM_INDEX_0 indicates usage of Flash-related APIs
    objectHandle = DRV_NVM_Initialize(DRV_NVM_INDEX_0, (SYS_MODULE_INIT*)NVMInitData);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>

  Remarks:
    This routine must be called before any other NVM routine is called.
    
    This routine should only be called once during system initialization
    unless DRV_NVM_Deinitialize is called to deinitialize the driver
    instance.
    
    This routine will NEVER block for hardware access. If the operation
    requires time to allow the hardware to reinitialize, it will be
    reported by the DRV_NVM_Status operation. The system must use
    DRV_NVM_Status to find out when the driver is in the ready state.
    
    Build configuration options may be used to statically override options
    in the "init" structure and will take precedence over initialization
    data passed using this routine.                                                   
*/

SYS_MODULE_OBJ DRV_NVM_Initialize
(
    const SYS_MODULE_INDEX index,
    const SYS_MODULE_INIT * const init
);

// ****************************************************************************
/* Function:
    void DRV_NVM_Deinitialize( SYS_MODULE_OBJ object )
    
  Summary:
    Deinitializes the specified instance of the NVM driver module.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    Deinitializes the specified instance of the NVM driver module, disabling its
    operation (and any hardware). Invalidates all the internal data.
  
  Preconditions:
    Function DRV_NVM_Initialize should have been called before calling
    this function.
  
  Parameter:
    object -  Driver object handle, returned from the DRV_NVM_Initialize
              routine

  Returns:
    None.

  Example:
    <code>
    // This code snippet shows an example
    // of deinitializing the driver.
    
    SYS_MODULE_OBJ      object;     //  Returned from DRV_NVM_Initialize
    SYS_STATUS          status;
    
    DRV_NVM_Deinitialize(object);
    
    status = DRV_NVM_Status(object);
    if (SYS_MODULE_DEINITIALIZED != status)
    {
        // Check again later if you need to know
        // when the driver is deinitialized.
    }
    </code>
  
  Remarks:
    Once the Initialize operation has been called, the Deinitialize
    operation must be called before the Initialize operation can be called
    again.
*/

void DRV_NVM_Deinitialize( SYS_MODULE_OBJ object);

// *************************************************************************
/* Function:
    SYS_STATUS DRV_NVM_Status( SYS_MODULE_OBJ object )
    
  Summary:
    Gets the current status of the NVM driver module.
	<p><b>Implementation:</b> Dynamic</p>
  
  Description:
    This routine provides the current status of the NVM driver module.
  
  Preconditions:
    Function DRV_NVM_Initialize should have been called before calling
    this function.
  
  Parameters:
    object -  Driver object handle, returned from the DRV_NVM_Initialize
              routine
  
  Returns:
    SYS_STATUS_READY - Indicates that the driver is ready and accept
    requests for new operations.
    
    SYS_STATUS_UNINITIALIZED - Indicates the driver is not initialized.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_NVM_Initialize
    SYS_STATUS          NVMStatus;
    
    NVMStatus = DRV_NVM _Status(object);
    else if (SYS_STATUS_ERROR >= NVMStatus)
    {
        // Handle error
    }
    </code>
  
  Remarks:
    This routine will NEVER block waiting for hardware.
*/

SYS_STATUS DRV_NVM_Status( SYS_MODULE_OBJ object);

// ****************************************************************************
/* Function:
    void DRV_NVM_Tasks ( SYS_MODULE_OBJ object );
    
  Summary:
    Maintains the driver's erase and write state machine and implements its
    ISR.
	<p><b>Implementation:</b> Dynamic</p>
  
  Description:
    This routine is used to maintain the driver's internal write and erase
    state machine and implement its ISR for interrupt-driven
    implementations.
  
  Preconditions:
    The DRV_NVM_Initialize routine must have been called for the specified
    NVM driver instance.
  
  Parameters:
    object -  Object handle for the specified driver instance (returned from
              DRV_NVM_Initialize)
  Returns:
    None.
  
  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_NVM_Initialize
    
    while (true)
    {
        DRV_NVM_Tasks (object);
    
        // Do other tasks
    }
    </code>

  Remarks:
    This routine is normally not called directly by an application. It is
    called by the system's Tasks routine (SYS_Tasks) or by the appropriate
    raw ISR.
    
    This routine may execute in an ISR context and will never block or
    access any resources that may cause it to block.                        
*/

void DRV_NVM_Tasks ( SYS_MODULE_OBJ object );

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
    Opens the specified timer driver instance and returns a handle to it.
	<p><b>Implementation:</b> Dynamic</p>
  
  Description:
    This routine opens the specified NVM driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver.
  
  Preconditions:
    Function DRV_NVM_Initialize must have been called before calling this
    function.
  
  Parameters:
    drvIndex -  Identifier for the object instance to be opened
    intent -    Zero or more of the values from the enumeration
                DRV_IO_INTENT "ORed" together to indicate the intended use
                of the driver
  
  Returns:
    If successful, the routine returns a valid open-instance handle (a
    number identifying both the caller and the module instance).
    
    If an error occurs, the return value is DRV_HANDLE_INVALID.
  
  Example:
    <code>
    DRV_HANDLE handle;
    
    handle = DRV_NVM_Open(DRV_NVM_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        // Unable to open the driver
    }
    </code>
  
  Remarks:
    The handle returned is valid until the DRV_NVM_Close routine is called.
    This routine will NEVER block waiting for hardware. If the driver has 
    has already been opened, it cannot be opened exclusively.
*/

DRV_HANDLE DRV_NVM_Open
(
    const SYS_MODULE_INDEX index, 
    const DRV_IO_INTENT ioIntent
);

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_NVM_Close( DRV_Handle handle )

  Summary:
    Closes an opened-instance of the NVM driver.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine closes an opened-instance of the NVM driver, invalidating the
    handle.

  Precondition:
    The DRV_NVM_Initialize routine must have been called for the specified
    NVM driver instance.

    DRV_NVM_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_NVM_Open

    DRV_NVM_Close(handle);
    </code>

  Remarks:
    After calling this routine, the handle passed in "handle" must not be used
    with any of the remaining driver routines.  A new handle must be obtained by
    calling DRV_NVM_Open before the caller may use the driver again.  Usually
    there is no need for the driver client to verify that the Close operation
    has completed.
*/

void DRV_NVM_Close( const DRV_HANDLE handle);

// ****************************************************************************
/* Function:
    DRV_NVM_CLIENT_STATUS DRV_NVM_ClientStatus (DRV_HANDLE handle);
    
  Summary:
    Gets current client-specific status the NVM driver.
	<p><b>Implementation:</b> Dynamic</p>
  
  Description:
    This routine gets the client-specific status of the NVM driver
    associated with the given handle.
  
  Preconditions:
    The DRV_NVM_Initialize routine must have been called.
    
    DRV_NVM_Open must have been called to obtain a valid opened device
    handle.
  
  Parameters:
    handle -  A valid open-instance handle, returned from the driver's open
              routine
  
  Returns:
    A DRV_NVM_CLIENT_STATUS value describing the current status of the
    driver
  
  Example:
    <code>
    DRV_HANDLE                  handle;         // Returned from DRV_NVM_Open
    DRV_NVM_CLIENT_STATUS     clientStatus;
    
    clientStatus = DRV_NVM_ClientStatus(handle);
    if(DRV_NVM_CLIENT_STATUS_ERROR >= clientStatus)
    {
        // Handle the error
    }
    </code>

  Remarks:
    This routine will not block for hardware access and will immediately
    return the current status.                                              
*/

DRV_NVM_CLIENT_STATUS   DRV_NVM_ClientStatus( const DRV_HANDLE handle );

// *****************************************************************************
/* Function:
    DRV_NVM_BUFFER_STATUS   DRV_NVM_BufferStatus
    (
        DRV_HANDLE handle, 
        const DRV_HANDLE bufferHandle
    );

  Summary:
    Gets the current status of the buffer.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine gets the current status of the buffer. The application must use
    this routine where the status of a scheduled buffer needs to polled on. The
    function may return DRV_NVM_BUFFER_HANDLE_INVALID in a case where the buffer
    handle has expired. A buffer handle expires when the internal buffer object
    is re-assigned to another erase or write request.  It is recommended that
    this function be called regularly in order to track the buffer status
    correctly.

    The application can alternatively register an event handler to receive write
    or erase operation completion events.

  Preconditions:
    The DRV_NVM_Initialize routine must have been called.

    DRV_NVM_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    A DRV_NVM_BUFFER_STATUS value describing the current status of the buffer.
    Returns DRV_NVM_BUFFER_HANDLE_INVALID if the client handle or the buffer
    handle is not valid.

  Example:
    <code>
    DRV_HANDLE                  handle;         // Returned from DRV_NVM_Open
    DRV_HANDLE                  bufferHandle;
    DRV_NVM_BUFFER_STATUS       status;
 
    status = DRV_NVM_BufferStatus(handle, bufferHandle);
    if(status == DRV_NVM_BUFFER_COMPLETED)
    {
        // Operation Done
    }
    </code>

  Remarks:
    This routine will not block for hardware access and will immediately return
    the current status.
*/

DRV_NVM_BUFFER_STATUS  DRV_NVM_BufferStatus
(
    DRV_HANDLE handle, 
    const DRV_HANDLE bufferHandle
);

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
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine reads a block of data from the specified address in memory.
    This operation is non blocking and returns with the required data in the
    target buffer. This function should not be to read areas of memory which are
    queued to be programmed or erased. If required, the program or erase
    operations should be allowed to complete. The function returns a buffer
    handle that can be queried using the DRV_NVM_BufferStatus() function. 

  Precondition:
    The DRV_NVM_Initialize routine must have been called for the specified
    NVM driver instance.

    DRV_NVM_Open must have been called to obtain a valid opened device handle.

    DRV_IO_INTENT_READ or DRV_IO_INTENT_READWRITE must have been specified in
    the DRV_NVM_Open call.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

    targetBuffer - Buffer into which the data read from the NVM instance
                   will be placed.

	srcAddress	 -	The base address in NVM memory from data read should begin
					
    numbytes     - Total number of bytes that need to be read from the module
                   instance (must be equal to or less than the size of the
                   buffer)

  Returns:
    DRV_NVM_BUFFER_HANDLE_INVALID in case the operation was not successful. A
    valid handle otherwise.

  Example:
    <code>
    DRV_HANDLE      handle;    // Returned from DRV_NVM_Open
    char            myBuffer[MY_BUFFER_SIZE];
	char			*srcAddress = NVM_BASE_ADDRESS_TO_READ_FROM;
    unsigned int    count = MY_BUFFER_SIZE;
    DRV_NVM_BUFFER_HANDLE bufferHandle;

	bufferHandle  = DRV_NVM_Read(myNVMHandle, &myBuffer[total], srcAddress, count);
    if(DRV_NVM_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Do error handling here
    }
    
    // Wait until the buffer completes. This should not
    // be a while loop if a part of cooperative multi-tasking 
    // routine. In that case, it should be invoked in task
    // state machine.
    while(DRV_NVM_BufferStatus(bufferHandle) != DRV_NVM_BUFFER_COMPLETED);

    </code>

  Remarks:
    None.
*/

DRV_NVM_BUFFER_HANDLE DRV_NVM_Read
(
    const DRV_HANDLE handle,
    uint8_t *targetBuffer, 
    uint8_t *srcAddress,
    const unsigned int numbytes
);

// *****************************************************************************
/* Function:
    DRV_NVM_BUFFER_HANDLE DRV_NVM_Write
    (
        const DRV_HANDLE handle,  
        uint8_t *targetbuffer,
	    uint8_t * srcAddress,  
        const unsigned int numbytes 
    )

  Summary:
    Write a block of data to a specified address in memory.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine writes a block of data to a specified address in memory.  It
    returns a buffer handle that can be queried by the DRV_NVM_BufferStatus()
    function to check for completion of the operation. The contents of the
    source buffer should not be changed while the operation is in progress.  The
    target address should be aligned on a DRV_NVM_ROW_SIZE byte boundary.  The
    number of bytes to write should be equal to or should be multiples of
    DRV_NVM_ROW_SIZE. Successful completion of the operation is also indicated
    by a DRV_NVM_EVENT_BUFFER_COMPLETE. This event is generated if a event
    handler callback function was registered.

  Precondition:
    The DRV_NVM_Initialize routine must have been called for the specified
    NVM driver instance.

    DRV_NVM_Open must have been called to obtain a valid opened device handle.

    DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have been specified in
    the DRV_NVM_Open call.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

    targetBuffer - Buffer in NVM memory where the data from srcAddress 
                   will be placed. Should be aligned on a DRV_NVM_ROW_SIZE
                   byte boundary.

	srcAddress	 -	The source buffer containing data to programmed into NVM

    numbytes     - Total number of bytes that need to be written to the NVM.
                   This should be equal to or a multiple of DRV_NVM_ROW_SIZE.

  Returns:
    DRV_NVM_BUFFER_HANDLE_INVALID in case the operation was not successful. A
    valid handle otherwise.

  Example:
    <code>
    DRV_HANDLE      handle;    // Returned from DRV_NVM_Open
    char            myBuffer[2 * DRV_NVM_ROW_SIZE];
    
    // Destination address should be row aligned.
	char			*destAddress = (char *)NVM_BASE_ADDRESS_TO_WRITE;
    
    unsigned int    count = 2 * MY_BUFFER_SIZE;
    DRV_NVM_BUFFER_HANDLE bufferHandle;

	bufferHandle  = DRV_NVM_Write(myNVMHandle, destAddress, &myBuffer[total], count);
    if(DRV_NVM_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Do error handling here
    }
    
    // Wait until the buffer completes. This should not
    // be a while loop if a part of cooperative multi-tasking 
    // routine. In that case, it should be invoked in task
    // state machine.
    while(DRV_NVM_BufferStatus(bufferHandle) != DRV_NVM_BUFFER_COMPLETED);

    </code>

  Remarks:
    Performing an flash programming operation while executing (fetching)
    instructions from program Flash memory, the CPU stalls (waits) until the
    programming operation is finished. The CPU will not execute any instruction,
    or respond to interrupts, during this time. If any interrupts occur during
    the programming cycle, they remain pending until the cycle completes. This
    make the NVM write operation blocking in nature.
*/

DRV_NVM_BUFFER_HANDLE DRV_NVM_Write
(
    DRV_HANDLE handle, 
    uint8_t * destination,
    uint8_t * source, 
    const unsigned int nBytes
);

// **************************************************************************
/* Function:
    DRV_NVM_BUFFER_HANDLE DRV_NVM_Erase
    (
        const DRV_HANDLE handle,
        uint8_t * targetbuffer, 
        const unsigned int numbytes 
    )
    
  Summary:
    Erase the specified number of pages in Flash memory.
	<p><b>Implementation:</b> Dynamic</p>
  
  Description:
    This routine erases the specified number of pages in Flash memory. It
    returns a buffer handle that can be queried by the DRV_NVM_BufferStatus()
    function to check for completion of the operation. The target address should
    be aligned on a DRV_NVM_PAGE_SIZE byte boundary. The number of bytes to
    write should be multiples of DRV_NVM_PAGE_SIZE.Successful completion of the
    operation is also indicated by a DRV_NVM_EVENT_BUFFER_COMPLETE. This event
    is generated if a event handler callback function was registered.
  
  Preconditions:
    The DRV_NVM_Initialize routine must have been called for the specified
    NVM driver instance.
    
    DRV_NVM_Open must have been called to obtain a valid opened device
    handle.
    
    DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have been specified
    in the DRV_NVM_Open call.
  
  Parameters:
    handle -        A valid open-instance handle, returned from the
                    driver's open routine
    targetBuffer -  Start address in NVM memory from where the erase should
                    begin. Should be aligned on a DRV_NVM_PAGE_SIZE byte
                    boundary.
    numbytes -      Total number of pages to be erased expressed in bytes.
                    This should be equal to or a multiple of
                    DRV_NVM_PAGE_SIZE.
  Returns:
    DRV_NVM_BUFFER_HANDLE_INVALID in case the operation was not successful.
    A valid handle otherwise.
  
  Example:
    <code>
    
    // Returned from DRV_NVM_Open
    DRV_HANDLE      handle;
    
    // Destination address should be page aligned.
    char            *destAddress = (char *)NVM_BASE_ADDRESS_TO_ERASE;
    
    unsigned int    count = 2 * DRV_NVM_PAGE_SIZE;
    DRV_NVM_BUFFER_HANDLE bufferHandle;
    
    bufferHandle  = DRV_NVM_Erase(myNVMHandle, destAddress, count);
    if(DRV_NVM_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Do error handling here
    }
    
    // Wait until the buffer completes. This should not
    // be a while loop if a part of cooperative multi-tasking
    // routine. In that case, it should be invoked in task
    // state machine.
    while(DRV_NVM_BufferStatus(bufferHandle) != DRV_NVM_BUFFER_COMPLETED);
    
    </code>
  
  Remarks:
    Performing an flash erase operation while executing (fetching) instructions
    from program Flash memory, the CPU stalls (waits) until the erase operation is
    finished. The CPU will not execute any instruction, or respond to interrupts,
    during this time. If any interrupts occur during the programming cycle, they
    remain pending until the cycle completes. This make the NVM erase operation
    blocking in nature.  
*/

DRV_NVM_BUFFER_HANDLE DRV_NVM_Erase
(
    const DRV_HANDLE handle,  
    uint8_t * targetbuffer, 
    const unsigned int numbytes 
);

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
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function allows a client to identify an event handling function 
    for the driver to call back when queued operation has completed.  
    When a client calls any read, write or erase function, it is provided with a
    handle identifying the buffer that was added to the driver's buffer queue.
    The driver will pass this handle back to the client by calling
    "eventHandler" function when the queued operation has completed.
    
    In a case where events are required, the event handler should be set before
    the client performs any write or erase operations that could generate
    events. The event handler once set, persists until the client closes the
    driver or sets another event handler (which could be a "NULL" pointer to
    indicate no callback).

  Precondition:
    The DRV_NVM_Initialize function must have been called for the
    specified NVM driver instance.

    DRV_NVM_Open must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

    eventHandler - Pointer to the event handler function implemented by the user
    
    context      - The value of parameter will be passed back to the client 
                   unchanged, when the eventHandler function is called. It can
                   be used to identify any client specific data object that 
                   identifies the instance of the client module (for example, 
                   it may be a pointer to the client module's state structure).

  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific state data object.
    MY_APP_OBJ myAppObj;

    uint8_t myBuffer[MY_BUFFER_SIZE] __attribute__((space(prog), aligned(DRV_NVM_PAGE_SIZE)));
    uint32_t blockStart, nBlock;
    DRV_NVM_BUFFER_HANDLE bufferHandle;

    // myNVMHandle is the handle returned 
    // by the DRV_NVM_Open function.
    
    // Client registers an event handler with driver. This is done once.

    DRV_NVM_BlockEventHandlerSet( myNVMHandle, APP_NVMEventHandler, (uintptr_t)&myAppObj );

    // nBytes should be a multiple of page size
    // myBuffer should be aligned on a page boundary
    bufferHandle = DRV_NVM_Erase( myNVMHandle, &myBuffer, nBytes );

    if(DRV_NVM_BUFFER_HANDLE == commandHandle)
    {
        // Error handling here
    }

    // Event Processing Technique. Event is received when operation is done.

    void APP_NVMEventHandler(DRV_NVM_EVENT event, DRV_NVM_BUFFER_HANDLE handle, uintptr_t context)
    {
        // The context handle was set to an application specific
        // object. It is now retrievable easily in the event handler.
        MY_APP_OBJ myAppObj = (MY_APP_OBJ *) context;

        switch(event)
        {
            case DRV_NVM_EVENT_BUFFER_COMPLETE:

                // This means the page was erased.
                break;
            
            case DRV_NVM_EVENT_BUFFER_ERROR:

                // Error handling here.

                break;

            default:
                break;
        }
    }
    </code>

  Remarks:
    If the client does not want to be notified when the queued operation
    has completed, it does not need to register a callback.
*/

void DRV_NVM_BlockEventHandlerSet
(
    const DRV_HANDLE handle,
    const void * eventHandler,
    const uintptr_t context
);

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
    Erases and then writes a block of data to a specified address in Flash memory.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine erases the Flash memory block to be written to and then writes
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

  Precondition:
    The DRV_NVM_Initialize routine must have been called for the specified
    NVM driver instance.

    DRV_NVM_Open must have been called to obtain a valid opened device handle.

    DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have been specified in
    the DRV_NVM_Open call.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

    targetBuffer - Buffer in NVM memory where the data from srcAddress 
                   will be placed. Should be aligned on a DRV_NVM_ROW_SIZE
                   byte boundary.

	srcAddress	 -	The source buffer containing data to programmed into NVM

    numbytes     - Total number of bytes that need to be written to the NVM.
                   This should be equal to or a multiple of DRV_NVM_ROW_SIZE.

  Returns:
    DRV_NVM_BUFFER_HANDLE_INVALID in case the operation was not successful. A
    valid handle otherwise.

  Example:
    <code>
    DRV_HANDLE      handle;    // Returned from DRV_NVM_Open
    char            myBuffer[2 * DRV_NVM_ROW_SIZE];
    
    // Destination address should be row aligned.
	char			*destAddress = (char *)NVM_BASE_ADDRESS_TO_WRITE;
    
    unsigned int    count = 2 * MY_BUFFER_SIZE;
    DRV_NVM_BUFFER_HANDLE bufferHandle;

	bufferHandle  = DRV_NVM_EraseWrite(myNVMHandle, destAddress, &myBuffer[total], count);
    if(DRV_NVM_BUFFER_HANDLE_INVALID == bufferHandle)
    {
        // Do error handling here
    }
    
    // Wait until the buffer completes. This should not
    // be a while loop if a part of cooperative multi-tasking 
    // routine. In that case, it should be invoked in task
    // state machine.
    while(DRV_NVM_BufferStatus(bufferHandle) != DRV_NVM_BUFFER_COMPLETED);

    </code>

  Remarks:
    In order to use this function, the DRV_NVM_ERASE_WRITE_ENABLE must be
    defined in system_config.h and the drv_nvm_erasewrite.c file must be
    included in the project..
*/

DRV_NVM_BUFFER_HANDLE DRV_NVM_EraseWrite
(
    const DRV_HANDLE handle,  
    uint8_t * targetbuffer,
    uint8_t * srcAddress,  
    const unsigned int numbytes 
);

// ****************************************************************************
// ****************************************************************************
// Section: Included Files (continued)
// ****************************************************************************
// ****************************************************************************
/*  The files included below map the interface definitions above to appropriate
    static implementations, depending on build mode.
*/


#endif // #ifndef _DRV_NVM_H
/*******************************************************************************
 End of File
*/

