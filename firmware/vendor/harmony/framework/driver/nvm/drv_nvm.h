/*******************************************************************************
  NVM Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_nvm.h

  Summary:
    NVM Driver Interface Definition

  Description:
    The NVM driver provides a simple interface to manage the Non Volatile Flash
    Memory on Microchip microcontrollers. This file defines the interface
    definition for the NVM driver.
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
#include "system/fs/sys_fs_media_manager.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
/* NVM Driver command handle.

  Summary:
    Handle identifying commands queued in the driver.

  Description:
    A command handle is returned by a call to the Read, Write or Erase
    functions. This handle allows the application to track the completion
    of the operation. This command handle is also returned to the client
    along with the event that has occurred with respect to the command.
    This allows the application to connect the event to a specific
    command in case where multiple commands are queued.

    The command handle associated with the command request expires when the
    client has been notified of the completion of the command (after event
    handler function that notifies the client returns) or after the command
    has been retired by the driver if no event handler callback was set. 

  Remarks:
    None.
*/

typedef SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE DRV_NVM_COMMAND_HANDLE;

// *****************************************************************************
/* NVM Driver Program Row Size.

  Summary:
    Specifies the NVM Driver Program Row Size in bytes.

  Description:
    This definition specifies the NVM Driver Program Row Size in bytes. This
    parameter is device specific and is obtained from the device specific
    processor header file. The Program Row Size is the minimum block size
    that can be programmed in one program operation.

  Remarks:
    None
*/

#define DRV_NVM_ROW_SIZE                            (NVM_ROW_SIZE)

// *****************************************************************************
/* NVM Driver Program Page Size.

  Summary:
    Specifies the NVM Driver Program Page Size in bytes.

  Description:
    This definition specifies the NVM Driver Program Page Size in bytes. This
    parameter is device specific and is obtained from the device specific
    processor header file.

  Remarks:
    None
*/

#define DRV_NVM_PAGE_SIZE                           (NVM_PAGE_SIZE)

// *****************************************************************************
/* NVM Driver Program Unlock Key 1

  Summary:
    Specifies the NVM Driver Program Unlock Key 1

  Description:
    This definition specifies the NVM Driver Program Unlock Key 1 parameter is
    device specific and is obtained from the device specific processor
    header file.

  Remarks:
    None
*/

#define DRV_NVM_PROGRAM_UNLOCK_KEY1                 (NVM_UNLOCK_KEY1)

// *****************************************************************************
/* NVM Driver Program Unlock Key 2

  Summary:
    Specifies the NVM Driver Program Unlock Key 2

  Description:
    This definition specifies the NVM Driver Program Unlock Key 2 parameter is
    device specific and is obtained from the device specific processor header
    file.

  Remarks:
    None
*/

#define DRV_NVM_PROGRAM_UNLOCK_KEY2                 (NVM_UNLOCK_KEY2)

// *****************************************************************************
/* NVM Driver Invalid Command Handle.

  Summary:
    This value defines the NVM Driver's Invalid Command Handle.

  Description:
    This value defines the NVM Driver Invalid Command Handle. This value
    is returned by read/write/erase routines when the command request was not
    accepted.

  Remarks:
    None.
*/

#define DRV_NVM_COMMAND_HANDLE_INVALID SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID

// *****************************************************************************
/* NVM Driver Events

   Summary
    Identifies the possible events that can result from a request.

   Description
    This enumeration identifies the possible events that can result from a 
    Write or Erase request caused by the client.

   Remarks:
    One of these values is passed in the "event" parameter of the event 
    handling callback function that client registered with the driver by
    calling the DRV_NVM_EventHandlerSet function when a request is completed.
*/

typedef enum
{
    /* Operation has been completed successfully. */
    DRV_NVM_EVENT_COMMAND_COMPLETE = SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_COMPLETE,

    /* There was an error during the operation */
    DRV_NVM_EVENT_COMMAND_ERROR = SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_ERROR 

} DRV_NVM_EVENT;

// ***********************************************************************
/* NVM Driver Command Status

  Summary:
    Specifies the status of the command for the read, write and erase
    operations.
	
  Description:
    NVM Driver command Status
    
    This type specifies the status of the command for the read, write and
    erase operations.
	
  Remarks:
    None.                                                               
*/  
typedef enum
{
    /*Done OK and ready */
    DRV_NVM_COMMAND_COMPLETED          = SYS_FS_MEDIA_COMMAND_COMPLETED,

    /*Scheduled but not started */
    DRV_NVM_COMMAND_QUEUED             = SYS_FS_MEDIA_COMMAND_QUEUED,

    /*Currently being in transfer */
    DRV_NVM_COMMAND_IN_PROGRESS        = SYS_FS_MEDIA_COMMAND_IN_PROGRESS,

    /*Unknown Command */
    DRV_NVM_COMMAND_ERROR_UNKNOWN      = SYS_FS_MEDIA_COMMAND_UNKNOWN,

} DRV_NVM_COMMAND_STATUS;

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

#define  DRV_NVM_INDEX_0      0
#define  DRV_NVM_INDEX_1      1

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

    /* NVM Media start address. The driver treats this address as 
     * block 0 address for read, write and erase operations.
     * */
    uint32_t            mediaStartAddress;

    /* NVM Media geometry object. */
    const SYS_FS_MEDIA_GEOMETRY *nvmMediaGeometry;

} DRV_NVM_INIT;

// *****************************************************************************
/* NVM Driver Event Handler Function Pointer

   Summary
    Pointer to a NVM Driver Event handler function

   Description
    This data type defines the required function signature for the NVM event
    handling callback function. A client must register a pointer to an event
    handling function whose function signature (parameter and return value 
    types) match the types specified by this function pointer in order to 
    receive event calls back from the driver.
    
    The parameters and return values are described here and a partial example
    implementation is provided.

  Parameters:
    event           - Identifies the type of event
    
    commandHandle   - Handle returned from the Read/Write/Erase requests
    
    context         - Value identifying the context of the application that
                      registered the event handling function

  Returns:
    None.

  Example:
    <code>
    void APP_MyNvmEventHandler
    (
        DRV_NVM_EVENT event,
        DRV_NVM_COMMAND_HANDLE commandHandle,
        uintptr_t context
    )
    {
        MY_APP_DATA_STRUCT pAppData = (MY_APP_DATA_STRUCT) context;
        
        switch(event)
        {
            case DRV_NVM_EVENT_COMMAND_COMPLETE:

                // Handle the completed buffer. 
                break;
            
            case DRV_NVM_EVENT_COMMAND_ERROR:
            default:

                // Handle error.
                break;
        }
    }
    </code>

  Remarks:
    If the event is DRV_NVM_EVENT_COMMAND_COMPLETE, it means that the
    write or a erase operation was completed successfully. 
    
    If the event is DRV_NVM_EVENT_COMMAND_ERROR, it means that the scheduled
    operation was not completed successfully.
     
    The context parameter contains the handle to the client context, provided
    at the time the event handling function was  registered using the
    DRV_NVM_EventHandlerSet function. This context handle value is
    passed back to the client as the "context" parameter.  It can be any value
    necessary to identify the client context or instance (such as a pointer to
    the client's data) instance of the client that made the read/write/erase
    request.

    The event handler function executes in the driver peripheral's interrupt
    context when the driver is configured for interrupt mode operation. It is
    recommended of the application to not perform process intensive or blocking
    operations within this function.
*/
typedef SYS_FS_MEDIA_EVENT_HANDLER DRV_NVM_EVENT_HANDLER;

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
    );
    
  Summary:
    Initializes the NVM instance for the specified driver index
    <p><b>Implementation:</b> Static/Dynamic</p>

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
    Otherwise it returns SYS_MODULE_OBJ_INVALID.
  
  Example:
    <code>
    // This code snippet shows an example
    // of initializing the NVM Driver.
    
    SYS_MODULE_OBJ  objectHandle;

    SYS_FS_MEDIA_REGION_GEOMETRY gNvmGeometryTable[3] = 
    {
        {
            // Read Region Geometry
            .blockSize = 1,
            .numBlocks = (DRV_NVM_MEDIA_SIZE * 1024),
        },
        {
            // Write Region Geometry
            .blockSize = DRV_NVM_ROW_SIZE,
            .numBlocks = ((DRV_NVM_MEDIA_SIZE * 1024)/DRV_NVM_ROW_SIZE)
        },
        {
            // Erase Region Geometry
            .blockSize = DRV_NVM_PAGE_SIZE,
            .numBlocks = ((DRV_NVM_MEDIA_SIZE * 1024)/DRV_NVM_PAGE_SIZE)
        }
    };

    const SYS_FS_MEDIA_GEOMETRY gNvmGeometry = 
    {
        .mediaProperty = SYS_FS_MEDIA_WRITE_IS_BLOCKING,

        // Number of read, write and erase entries in the table
        .numReadRegions = 1,
        .numWriteRegions = 1,
        .numEraseRegions = 1,
        .geometryTable = &gNvmGeometryTable
    };

    // FLASH Driver Initialization Data
    const DRV_NVM_INIT drvNvmInit =
    {
        .moduleInit.sys.powerState = SYS_MODULE_POWER_RUN_FULL,
        .nvmID = NVM_ID_0,
        .interruptSource = INT_SOURCE_FLASH_CONTROL,
        .mediaStartAddress = NVM_BASE_ADDRESS,
        .nvmMediaGeometry = &gNvmGeometry
    };

    //usage of DRV_NVM_INDEX_0 indicates usage of Flash-related APIs
    objectHandle = DRV_NVM_Initialize(DRV_NVM_INDEX_0, (SYS_MODULE_INIT*)&drvNVMInit);
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
    void DRV_NVM_Deinitialize
    (
        SYS_MODULE_OBJ object 
    );
    
  Summary:
    Deinitializes the specified instance of the NVM driver module
    <p><b>Implementation:</b> Static/Dynamic</p>

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

void DRV_NVM_Deinitialize
(
    SYS_MODULE_OBJ object
);

// *************************************************************************
/* Function:
    SYS_STATUS DRV_NVM_Status
    (
        SYS_MODULE_OBJ object
    );
    
  Summary:
    Gets the current status of the NVM driver module.
    <p><b>Implementation:</b> Static/Dynamic</p>
  
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
    
    NVMStatus = DRV_NVM_Status(object);
    else if (SYS_STATUS_ERROR >= NVMStatus)
    {
        // Handle error
    }
    </code>
  
  Remarks:
    This routine will NEVER block waiting for hardware.
*/

SYS_STATUS DRV_NVM_Status
(
    SYS_MODULE_OBJ object
);

// ****************************************************************************
/* Function:
    void DRV_NVM_Tasks 
    (
        SYS_MODULE_OBJ object
    );
    
  Summary:
    Maintains the driver's erase and write state machine and implements its
    ISR.
    <p><b>Implementation:</b> Static/Dynamic</p>
  
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

void DRV_NVM_Tasks 
(
    SYS_MODULE_OBJ object
);

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
    );
    
  Summary:
    Opens the specified NVM driver instance and returns a handle to it
    <p><b>Implementation:</b> Static/Dynamic</p>
  
  Description:
    This routine opens the specified NVM driver instance and provides a handle. 
    This handle must be provided to all other client-level operations to identify
    the caller and the instance of the driver.
  
  Preconditions:
    Function DRV_NVM_Initialize must have been called before calling this
    function.
  
  Parameters:
    index  - Identifier for the object instance to be opened
    intent - Zero or more of the values from the enumeration
             DRV_IO_INTENT "ORed" together to indicate the intended use
             of the driver
  
  Returns:
    If successful, the routine returns a valid open-instance handle (a
    number identifying both the caller and the module instance).
    
    If an error occurs, DRV_HANDLE_INVALID is returned. Errors can occur
    under the following circumstances:
    	- if the number of client objects allocated via DRV_NVM_CLIENTS_NUMBER 
          is insufficient
    	- if the client is trying to open the driver but driver has been opened
    	  exclusively by another client
    	- if the client is trying to open the driver exclusively, but has already
          been opened in a non exclusive mode by another client.
    	- if the driver hardware instance being opened is not initialized or is
          invalid
  
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
    void DRV_NVM_Close
    (
        const DRV_HANDLE handle
    );

  Summary:
    Closes an opened-instance of the NVM driver
    <p><b>Implementation:</b> Static/Dynamic</p>

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
    calling DRV_NVM_Open before the caller may use the driver again. Usually
    there is no need for the driver client to verify that the Close operation
    has completed.
*/

void DRV_NVM_Close
(
    const DRV_HANDLE handle
);

// *****************************************************************************
/* Function:
    void DRV_NVM_Read
    (
        const DRV_HANDLE handle,
        DRV_NVM_COMMAND_HANDLE * commandHandle,
        void * targetBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Reads blocks of data from the specified address in memory.
    <p><b>Implementation:</b> Static/Dynamic</p>

  Description:
    This routine reads blocks of data from the specified address in memory.
    This operation is blocking and returns with the required data in the
    target buffer. If an event handler is registered with the driver the
    event handler would be invoked from within this function to indicate the
    status of the operation.
    This function should not be used to read areas of memory which are 
    queued to be programmed or erased. If required, the program or
    erase operations should be allowed to complete. The function returns
    DRV_NVM_COMMAND_HANDLE_INVALID in the commandHandle argument under the 
    following circumstances:
    - if the driver handle is invalid
    - if the target buffer pointer is NULL
    - if the number of blocks to be read is zero or more than the actual number
      of blocks available
    - if a buffer object could not be allocated to the request
    - if the client opened the driver in write only mode

  Precondition:
    The DRV_NVM_Initialize routine must have been called for the specified NVM 
    driver instance.

    DRV_NVM_Open must have been called with DRV_IO_INTENT_READ or 
    DRV_IO_INTENT_READWRITE as the ioIntent to obtain a valid opened device handle.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open function

    commandHandle - Pointer to an argument that will contain the return buffer
                    handle
                   
    targetBuffer  - Buffer into which the data read from the NVM Flash instance
                    will be placed

    blockStart    - Start block address in NVM memory from where the
                    read should begin. It can be any address of the flash.

    nBlock        - Total number of blocks to be read. Each Read block is of 1
                    byte.

  Returns:
    The buffer handle is returned in the commandHandle argument. It will be
    DRV_NVM_COMMAND_HANDLE_INVALID if the request was not successful.

  Example:
    <code>

    uint8_t myBuffer[MY_BUFFER_SIZE];
    
    // address should be block aligned.
    uint32_t blockStart = NVM_BASE_ADDRESS_TO_READ_FROM;
    uint32_t    nBlock = 2;
    DRV_NVM_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;    

    // myNVMHandle is the handle returned 
    // by the DRV_NVM_Open function.
    
    DRV_NVM_Read(myNVMHandle, &commandHandle, &myBuffer, blockStart, nBlock);

    if(DRV_NVM_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }
    else
    {
        // Read Successful
    }

    </code>

  Remarks:
    None.
*/

void DRV_NVM_Read
(
    const DRV_HANDLE handle,
    DRV_NVM_COMMAND_HANDLE * commandHandle,
    void * targetBuffer,
    uint32_t blockStart,
    uint32_t nBlock
);

// *****************************************************************************
/* Function:
    void DRV_NVM_Write
    (
        const DRV_HANDLE handle,
        DRV_NVM_COMMAND_HANDLE * commandHandle,
        void * sourceBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Writes blocks of data starting from the specified address in flash memory.
    <p><b>Implementation:</b> Static/Dynamic</p>

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

  Precondition:
    The DRV_NVM_Initialize() routine must have been called for the specified
    NVM driver instance.

    DRV_NVM_Open() routine must have been called to obtain a valid opened device
    handle. DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have been specified
    as a parameter to this routine.

    The flash address location which has to be written, must have be erased before
    using the DRV_NVM_Erase() routine.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open function

    commandHandle - Pointer to an argument that will contain the return buffer
                    handle
                   
    sourceBuffer  - The source buffer containing data to be programmed into NVM 
                    Flash

    blockStart    - Start block address of NVM Flash where the write should begin. 
                    This address should be aligned on a block boundary.

    nBlock        - Total number of blocks to be written. 

  Returns:
    The buffer handle is returned in the commandHandle argument. It will be
    DRV_NVM_COMMAND_HANDLE_INVALID if the request was not successful.

  Example:
    <code>
    
    uint8_t myBuffer[MY_BUFFER_SIZE];
    
    // address should be block aligned.
    uint32_t blockStart = NVM_BASE_ADDRESS_TO_WRITE_TO;
    uint32_t    nBlock = 2;
    DRV_NVM_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;    

    // myNVMHandle is the handle returned 
    // by the DRV_NVM_Open function.
    
    // Client registers an event handler with driver

    DRV_NVM_EventHandlerSet(myNVMHandle, APP_NVMEventHandler, (uintptr_t)&myAppObj);

    DRV_NVM_Write(myNVMHandle, &commandHandle, &myBuffer, blockStart, nBlock);

    if(DRV_NVM_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer is processed.

    void APP_NVMEventHandler(DRV_NVM_EVENT event, 
            DRV_NVM_COMMAND_HANDLE commandHandle, uintptr_t contextHandle)
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_NVM_EVENT_COMMAND_COMPLETE:

                // This means the data was transferred. 
                break;
            
            case DRV_NVM_EVENT_COMMAND_ERROR:

                // Error handling here.
                break;

            default:
                break;
        }
    }

    </code>

  Remarks:
    Performing a flash programming operation while executing (fetching)
    instructions from program Flash memory, the CPU stalls (waits) until the
    programming operation is finished. The CPU will not execute any instruction,
    or respond to interrupts, during this time. If any interrupts occur during
    the programming cycle, they remain pending until the cycle completes. This
    makes the NVM write operation blocking in nature.
*/

void DRV_NVM_Write
(
    const DRV_HANDLE handle,
    DRV_NVM_COMMAND_HANDLE * commandHandle,
    void * sourceBuffer,
    uint32_t blockStart,
    uint32_t nBlock
);

// **************************************************************************
/* Function:
    void DRV_NVM_Erase
    (
        const DRV_HANDLE handle,
        DRV_NVM_COMMAND_HANDLE * commandHandle,
        uint32_t blockStart,
        uint32_t nBlock
    );
    
  Summary:
    Erase the specified number of blocks of the Flash memory.
    <p><b>Implementation:</b> Static/Dynamic</p>
  
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
  
  Preconditions:
    The DRV_NVM_Initialize() routine must have been called for the specified
    NVM driver instance.
    
    The DRV_NVM_Open() routine must have been called with DRV_IO_INTENT_WRITE
    or DRV_IO_INTENT_READWRITE to obtain a valid opened device handle.
  
  Parameters:
    handle        - A valid open-instance handle, returned from the
                    driver's open function

    commandHandle - Pointer to an argument that will contain the return buffer
                    handle

    blockStart    - Start block address in NVM memory from where the erase
                    should begin. This should be aligned on a 
                    DRV_NVM_PAGE_SIZE byte boundary.

    nBlock        - Total number of blocks to be erased. 
  
  Returns:
    The buffer handle is returned in the commandHandle argument. It Will be
    DRV_NVM_COMMAND_HANDLE_INVALID if the request was not queued.
  
  Example:
    <code>
   
    // Destination address should be block aligned.
    uint32_t blockStart;
    uint32_t nBlock; 
    DRV_NVM_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;    

    // myNVMHandle is the handle returned 
    // by the DRV_NVM_Open function.
    
    // Client registers an event handler with driver

    DRV_NVM_EventHandlerSet(myNVMHandle, APP_NVMEventHandler, (uintptr_t)&myAppObj);

    DRV_NVM_Erase( myNVMHandle, &commandHandle, blockStart, nBlock );

    if(DRV_NVM_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer queue is processed.

    void APP_NVMEventHandler(DRV_NVM_EVENT event, 
            DRV_NVM_COMMAND_HANDLE commandHandle, uintptr_t contextHandle)
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_NVM_EVENT_COMMAND_COMPLETE:

                // This means the data was transferred. 
                break;
            
            case DRV_NVM_EVENT_COMMAND_ERROR:

                // Error handling here.
                break;

            default:
                break;
        }
    }
    
    </code>
    
  Remarks:
    Performing a flash erase operation while executing (fetching) instructions
    from program Flash memory, the CPU stalls (waits) until the erase operation is
    finished. The CPU will not execute any instruction, or respond to interrupts,
    during this time. If any interrupts occur during the programming cycle, they
    remain pending until the cycle completes. This make the NVM erase operation
    blocking in nature.  
*/

void DRV_NVM_Erase
(
    const DRV_HANDLE handle,
    DRV_NVM_COMMAND_HANDLE * commandHandle,
    uint32_t blockStart,
    uint32_t nBlock
);

// *****************************************************************************
/* Function:
    void DRV_NVM_EraseWrite
    (
        const DRV_HANDLE handle,
        DRV_NVM_COMMAND_HANDLE * commandHandle,
        void * sourceBuffer,
        uint32_t writeBlockStart,
        uint32_t nWriteBlock
    );

  Summary:
    Erase and Write blocks of data starting from a specified address in flash
    memory.
    <p><b>Implementation:</b> Static/Dynamic</p>

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

  Precondition:
    The DRV_NVM_Initialize() routine must have been called for the specified
    NVM driver instance.

    The DRV_NVM_Open() must have been called with DRV_IO_INTENT_WRITE or 
    DRV_IO_INTENT_READWRITE as a parameter to obtain a valid opened device handle.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open function

    commandHandle - Pointer to an argument that will contain the return buffer
                    handle. If NULL, then buffer handle is not returned.
                   
    sourceBuffer  - The source buffer containing data to be programmed into NVM
                    Flash

    writeBlockStart - Start block address of NVM Flash where the write should begin.
                    This address should be aligned on a DRV_NVM_ROW_SIZE byte 
                    boundary.

    nWriteBlock   - Total number of blocks to be written. 

  Returns:
    The buffer handle is returned in the commandHandle argument. It Will be
    DRV_NVM_COMMAND_HANDLE_INVALID if the request was not queued.

  Example:
    <code>
    
    uint8_t myBuffer[MY_BUFFER_SIZE];
    
    // address should be block aligned.
    uint32_t blockStart = NVM_BASE_ADDRESS_TO_WRITE_TO;
    uint32_t    nBlock = 2;
    DRV_NVM_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;    

    // myNVMHandle is the handle returned 
    // by the DRV_NVM_Open function.
    
    // Client registers an event handler with driver

    DRV_NVM_EventHandlerSet(myNVMHandle, APP_NVMEventHandler, (uintptr_t)&myAppObj);

    DRV_NVM_EraseWrite(myNVMHandle, &commandHandle, &myBuffer, blockStart, nBlock);

    if(DRV_NVM_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer is processed.

    void APP_NVMEventHandler(DRV_NVM_EVENT event, 
            DRV_NVM_COMMAND_HANDLE commandHandle, uintptr_t contextHandle)
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_NVM_EVENT_COMMAND_COMPLETE:

                // This means the data was transferred. 
                break;
            
            case DRV_NVM_EVENT_COMMAND_ERROR:

                // Error handling here.
                break;

            default:
                break;
        }
    }

    </code>

  Remarks:
    In order to use this function, the DRV_NVM_ERASE_WRITE_ENABLE must be
    defined in system_config.h and the drv_nvm_erasewrite.c file must be
    included in the project.
*/

void DRV_NVM_EraseWrite
(
    const DRV_HANDLE handle,
    DRV_NVM_COMMAND_HANDLE * commandHandle,
    void * sourceBuffer,
    uint32_t writeBlockStart,
    uint32_t nWriteBlock
);

// *****************************************************************************
/* Function:
    DRV_NVM_COMMAND_STATUS DRV_NVM_CommandStatus
    (
        const DRV_HANDLE handle, 
        const DRV_NVM_COMMAND_HANDLE commandHandle
    );

  Summary:
    Gets the current status of the command.
    <p><b>Implementation:</b> Static/Dynamic</p>

  Description:
    This routine gets the current status of the command. The application must use
    this routine where the status of a scheduled command needs to polled on. The
    function may return DRV_NVM_COMMAND_HANDLE_INVALID in a case where the command
    handle has expired. A command handle expires when the internal buffer object
    is re-assigned to another erase or write request. It is recommended that this
    function be called regularly in order to track the command status correctly.

    The application can alternatively register an event handler to receive write
    or erase operation completion events.

  Preconditions:
    The DRV_NVM_Initialize() routine must have been called.

    The DRV_NVM_Open() must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    A DRV_NVM_COMMAND_STATUS value describing the current status of the command.
    Returns DRV_NVM_COMMAND_HANDLE_INVALID if the client handle or the command
    handle is not valid.

  Example:
    <code>
    DRV_HANDLE                  handle;         // Returned from DRV_NVM_Open
    DRV_NVM_COMMAND_HANDLE      commandHandle;
    DRV_NVM_COMMAND_STATUS      status;
 
    status = DRV_NVM_CommandStatus(handle, commandHandle);
    if(status == DRV_NVM_COMMAND_COMPLETED)
    {
        // Operation Done
    }
    </code>

  Remarks:
    This routine will not block for hardware access and will immediately return
    the current status.
*/

DRV_NVM_COMMAND_STATUS DRV_NVM_CommandStatus
(
    const DRV_HANDLE handle, 
    const DRV_NVM_COMMAND_HANDLE commandHandle
);

// *****************************************************************************
/* Function:
    SYS_FS_MEDIA_GEOMETRY * DRV_NVM_GeometryGet
    (
        const DRV_HANDLE handle
    );

  Summary:
    Returns the geometry of the device.
    <p><b>Implementation:</b> Static/Dynamic</p>

  Description:
    This API gives the following geometrical details of the NVM Flash:
    - Media Property
    - Number of Read/Write/Erase regions in the flash device
    - Number of Blocks and their size in each region of the device

  Precondition:
    The DRV_NVM_Initialize() routine must have been called for the
    specified NVM driver instance.

    The DRV_NVM_Open() routine must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    SYS_FS_MEDIA_GEOMETRY - Pointer to structure which holds the media geometry information.

  Example:
    <code> 
    
    SYS_FS_MEDIA_GEOMETRY * nvmFlashGeometry;
    uint32_t readBlockSize, writeBlockSize, eraseBlockSize;
    uint32_t nReadBlocks, nReadRegions, totalFlashSize;

    nvmFlashGeometry = DRV_NVM_GeometryGet(nvmOpenHandle1);

    readBlockSize  = nvmFlashGeometry->geometryTable->blockSize;
    nReadBlocks = nvmFlashGeometry->geometryTable->numBlocks;
    nReadRegions = nvmFlashGeometry->numReadRegions;

    writeBlockSize  = (nvmFlashGeometry->geometryTable +1)->blockSize;
    eraseBlockSize  = (nvmFlashGeometry->geometryTable +2)->blockSize;

    totalFlashSize = readBlockSize * nReadBlocks * nReadRegions;

    </code>

  Remarks:
    None.
*/

SYS_FS_MEDIA_GEOMETRY * DRV_NVM_GeometryGet
(
    const DRV_HANDLE handle
);

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
    <p><b>Implementation:</b> Static/Dynamic</p>

  Description:
    This function allows a client to identify an event handling function for
    the driver to call back when queued operation has completed.  
    When a client calls a write or erase function, it is provided with a
    handle identifying the buffer that was added to the driver's buffer queue.
    The driver will pass this handle back to the client by calling "eventHandler"
    function when the queued operation has completed.
    
    The event handler should be set before the client performs any write or erase
    operations that could generate events. The event handler once set, persists
    until the client closes the driver or sets another event handler (which could
    be a "NULL" pointer to indicate no callback).

  Precondition:
    The DRV_NVM_Initialize() routine must have been called for the
    specified NVM driver instance.

    The DRV_NVM_Open() routine must have been called to obtain a valid opened device
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

    uint8_t myBuffer[MY_BUFFER_SIZE];
    uint32_t blockStart, nBlock;
    DRV_NVM_COMMAND_HANDLE commandHandle;

    // drvNVMHandle is the handle returned 
    // by the DRV_NVM_Open function.
    
    // Client registers an event handler with driver. This is done once.

    DRV_NVM_EventHandlerSet(drvNVMHandle, APP_NVMEventHandler, (uintptr_t)&myAppObj);

    DRV_NVM_Read(drvNVMHandle, &commandHandle, &myBuffer, blockStart, nBlock);

    if(DRV_NVM_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event Processing Technique. Event is received when operation is done.

    void APP_NVMEventHandler(DRV_NVM_EVENT event, 
            DRV_NVM_COMMAND_HANDLE handle, uintptr_t context)
    {
        // The context handle was set to an application specific
        // object. It is now retrievable easily in the event handler.
        MY_APP_OBJ myAppObj = (MY_APP_OBJ *) context;

        switch(event)
        {
            case DRV_NVM_EVENT_COMMAND_COMPLETE:

                // This means the data was transferred. 
                break;
            
            case DRV_NVM_EVENT_COMMAND_ERROR:

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

void DRV_NVM_EventHandlerSet
(
    const DRV_HANDLE handle,
    const void * eventHandler,
    const uintptr_t context
);

// *****************************************************************************
/* Function:
    bool DRV_NVM_IsAttached
    ( 
        const DRV_HANDLE handle 
    );

  Summary:
    Returns the physical attach status of the NVM.
    <p><b>Implementation:</b> Static/Dynamic</p>

  Description:
    This function returns the physical attach status of the NVM.

  Precondition:
    The DRV_NVM_Initialize() routine must have been called for the specified 
    NVM driver instance.

    The DRV_NVM_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    Returns false if the handle is invalid otherwise returns true.

  Example:
    <code> 

    // The NVM media is always attached and so the below
    // always returns true.
    
    bool isNVMAttached;
    isNVMAttached = DRV_NVM_isAttached(drvNVMHandle);

    </code>

  Remarks:
    None.
*/

bool DRV_NVM_IsAttached
(
    const DRV_HANDLE handle
);

// *****************************************************************************
/* Function:
    bool DRV_NVM_IsWriteProtected
    ( 
        const DRV_HANDLE handle 
    );

  Summary:
    Returns the write protect status of the NVM.
    <p><b>Implementation:</b> Static/Dynamic</p>

  Description:
    This function returns the physical attach status of the NVM. This function
    always returns false.

  Precondition:
    The DRV_NVM_Initialize() routine must have been called for the specified 
    NVM driver instance.

    The DRV_NVM_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    Always returns false.

  Example:
    <code>

    // The NVM media is treated as always writeable.
    bool isWriteProtected;
    isWriteProtected = DRV_NVM_IsWriteProtected(drvNVMHandle);

    </code>

  Remarks:
    None.
*/

bool DRV_NVM_IsWriteProtected
(
    const DRV_HANDLE handle
);

// *****************************************************************************
/* Function:
    uintptr_t DRV_NVM_AddressGet
    (
        const DRV_HANDLE handle
    );

  Summary:
    Returns the NVM media start address
    <p><b>Implementation:</b> Static/Dynamic</p>

  Description:
    This function returns the NVM Media start address.

  Precondition:
    The DRV_NVM_Initialize() routine must have been called for the specified 
    NVM driver instance.

    The DRV_NVM_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    Start address of the NVM Media if the handle is valid otherwise NULL.

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
);

#ifdef __cplusplus
}
#endif

#endif // #ifndef _DRV_NVM_H
/*******************************************************************************
 End of File
*/

