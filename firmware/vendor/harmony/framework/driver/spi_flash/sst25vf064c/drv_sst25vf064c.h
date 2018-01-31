/*******************************************************************************
  SPI Flash Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sst25vf064c.h

  Summary:
    SPI Flash Driver Interface Definition

  Description:
    The SPI Flash device driver provides a simple interface to manage the SPI
    Flash modules which are external to Microchip Controllers.
    This file defines the interface definition for the SPI Flash Driver.
******************************************************************************/

//DOM-IGNORE-BEGIN
/******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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
******************************************************************************/
//DOM-IGNORE-END

#ifndef _DRV_SST25VF064C_H
#define _DRV_SST25VF064C_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

/* Note:  A file that maps the interface definitions above to appropriate static
          implementations (depending on build mode) is included at the end of
          this file.
*/

#include "driver/driver_common.h"

#include "system/system.h"

#include "system/int/sys_int.h"
#include "system/ports/sys_ports.h"
#include "driver/spi/drv_spi.h"
#include "system/fs/sys_fs_media_manager.h"

#include "osal/osal.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  


// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Driver SPI Flash Module Index reference

  Summary:
    SPI Flash driver index definitions.

  Description:
    These constants provide SST25VF064C SPI Flash driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the DRV_SST25VF064C_Initialize and
    DRV_SST25VF064C_Open routines to identify the driver instance in use.
*/

#define      DRV_SST25VF064C_INDEX_0      0
#define      DRV_SST25VF064C_INDEX_1      1


// *****************************************************************************
/* SPI Flash Driver Block Command Handle

  Summary:
    Handle identifying block commands of the driver.

  Description:
    A block command handle is returned by a call to the Read, Write, or Erase
    functions. This handle allows the application to track the completion of
    the operation. The handle is returned back to the client by the "event
    handler callback" function registered with the driver.

    The handle assigned to a client request expires when the client has
    been notified of the completion of the operation (after event handler
    function that notifies the client returns) or after the buffer has been
    retired by the driver if no event handler callback was set.

  Remarks:
    None.
*/

typedef SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE  DRV_SST25VF064C_BLOCK_COMMAND_HANDLE;

// *****************************************************************************
/* SPI Flash Driver Block Event Invalid Handle

  Summary:
    This value defines the SPI Flash Driver Block Command Invalid handle.

  Description:
    This value defines the SPI Flash Driver Block Command Invalid handle. It is
    returned by read/write/erase routines when the request could not be taken.

  Remarks:
    None.
*/

#define DRV_SST25VF064C_BLOCK_COMMAND_HANDLE_INVALID   /*DOM-IGNORE-BEGIN*/ SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID /*DOM-IGNORE-END*/

// *****************************************************************************
/* SST25VF064C SPI Flash Driver Events

   Summary:
    Identifies the possible events that can result from a request.

   Description:
    This enumeration identifies the possible events that can result from a
    Read, Write, or Erase request caused by the client.

   Remarks:
    One of these values is passed in the "event" parameter of the event
    handling callback function that client registered with the driver by
    calling the DRV_SST25VF064C_BlockEventHandlerSet function when a block
    request is completed.
*/

typedef enum
{
    /* Block operation has been completed successfully. */
    /* Read/Write/Erase Complete */
    DRV_SST25VF064C_EVENT_BLOCK_COMMAND_COMPLETE
       /*DOM-IGNORE-BEGIN*/ = SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_COMPLETE /*DOM-IGNORE-END*/,

    /* There was an error during the block operation */
    /* Read/Write/Erase Error */
    DRV_SST25VF064C_EVENT_BLOCK_COMMAND_ERROR
       /*DOM-IGNORE-BEGIN*/ = SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_ERROR /*DOM-IGNORE-END*/

} DRV_SST25VF064C_BLOCK_EVENT;

// *****************************************************************************
/* SST25VF064C SPI Flash Driver Event Handler Function Pointer

   Summary:
    Pointer to a SST25VF064C SPI Flash Driver Event handler function.

   Description:
    This data type defines the required function signature for the SST25VF064C
    SPI Flash driver event handling callback function. A client must register
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
        DRV_SST25VF064C_BLOCK_EVENT event,
        DRV_SST25VF064C_BLOCK_COMMAND_HANDLE commandHandle,
        uintptr_t context
    )
    {
        MY_APP_DATA_STRUCT pAppData = (MY_APP_DATA_STRUCT) context;

        switch(event)
        {
            case DRV_SST25VF064C_EVENT_BLOCK_COMMAND_COMPLETE:

                // Handle the completed buffer.
                break;

            case DRV_SST25VF064C_EVENT_BLOCK_COMMAND_ERROR:
            default:

                // Handle error.
                break;
        }
    }
    </code>

  Remarks:
    If the event is DRV_SST25VF064C_EVENT_BLOCK_COMMAND_COMPLETE, it means that the
    data was transferred successfully.

    If the event is DRV_SST25VF064C_EVENT_BLOCK_COMMAND_ERROR, it means that the data
    was not transferred successfully.

    The context parameter contains the a handle to the client context,
    provided at the time the event handling function was  registered using the
    DRV_SST25VF064C_BlockEventHandlerSet function. This context handle value is
    passed back to the client as the "context" parameter.  It can be any value
    necessary to identify the client context or instance (such as a pointer to
    the client's data) instance of the client that made the read/write/erase
    request.

    The event handler function executes in the driver peripheral's interrupt
    context when the driver is configured for interrupt mode operation. It is
    recommended of the application to not perform process intensive or blocking
    operations with in this function.

    The Read, Write, and Erase functions can be called in the event handler to
    add a buffer to the driver queue. These functions can only be called to add
    buffers to the driver whose event handler is running.
*/

typedef void ( *DRV_SST25VF064C_EVENT_HANDLER )
(
    DRV_SST25VF064C_BLOCK_EVENT event,
    DRV_SST25VF064C_BLOCK_COMMAND_HANDLE commandHandle,
    uintptr_t context
);

// ***********************************************************************
/* SST Flash Driver Command Status

  Summary:
    Specifies the status of the command for the read, write and erase
    operations.
	
  Description:
    SST Flash Driver command Status. This type specifies the status of the command 
	for the read, write and erase operations.
	
  Remarks:
    None.                                                               
*/  
typedef enum
{
    /* Requested operation is completed */
    DRV_SST25VF064C_COMMAND_COMPLETED
    /*DOM-IGNORE-BEGIN*/ = SYS_FS_MEDIA_COMMAND_COMPLETED /*DOM-IGNORE-END*/,

    /*Scheduled but not started */
    DRV_SST25VF064C_COMMAND_QUEUED
    /*DOM-IGNORE-BEGIN*/ = SYS_FS_MEDIA_COMMAND_QUEUED /*DOM-IGNORE-END*/,
    
    /*Currently being in transfer */
    DRV_SST25VF064C_COMMAND_IN_PROGRESS
    /*DOM-IGNORE-BEGIN*/ = SYS_FS_MEDIA_COMMAND_IN_PROGRESS /*DOM-IGNORE-END*/,
    
    /*Unknown Command */
    DRV_SST25VF064C_COMMAND_ERROR_UNKNOWN
    /*DOM-IGNORE-BEGIN*/ = SYS_FS_MEDIA_COMMAND_UNKNOWN /*DOM-IGNORE-END*/
    
} DRV_SST25VF064C_COMMAND_STATUS;

// *****************************************************************************
/* SPI Flash Client Status

  Summary:
    Defines the client status.

  Description:
    Defines the various client status codes.

  Remarks:
    None.
*/

typedef enum
{
    /* Up and running, ready to start new operations */
    DRV_SST25VF064C_CLIENT_STATUS_READY = DRV_CLIENT_STATUS_READY + 0,

    /* Operation in progress, unable to start a new one */
    DRV_SST25VF064C_CLIENT_STATUS_BUSY = DRV_CLIENT_STATUS_BUSY,

    /* Client is closed */
    DRV_SST25VF064C_CLIENT_STATUS_CLOSED = DRV_CLIENT_STATUS_CLOSED,

    /* Client Error */
    DRV_SST25VF064C_CLIENT_STATUS_ERROR = DRV_CLIENT_STATUS_ERROR

} DRV_SST25VF064C_CLIENT_STATUS;

// *****************************************************************************
/* SST SPI Flash Driver Initialization Data

  Summary:
    Contains all the data necessary to initialize the SPI Flash device.

  Description:
    This structure contains all of the data necessary to initialize the SPI
    Flash device.

  Remarks:
    A pointer to a structure of this format containing the desired
    initialization data must be passed into the DRV_SST25VF064C_Initialize
    function.
*/

typedef struct
{
    /* System module initialization */
    SYS_MODULE_INIT                     moduleInit;

    /* Identifies the SPI driver to be used */
    SYS_MODULE_INDEX                    spiDriverModuleIndex;

    /* HOLD pin port channel */
    PORTS_CHANNEL                       holdPortChannel;

    /* HOLD pin port position*/
    PORTS_BIT_POS                       holdBitPosition;

    /* Write protect pin port channel */
    PORTS_CHANNEL                       writeProtectPortChannel;

    /* Write Protect Bit pin position */
    PORTS_BIT_POS                       writeProtectBitPosition;

    /* Chip select pin port channel */
    PORTS_CHANNEL                       chipSelectPortChannel;

    /* Chip Select Bit pin position */
    PORTS_BIT_POS                       chipSelectBitPosition;

    /* This is the buffer queue size. This is the maximum
       number of requests that this instance of the driver will
       queue. For a static build of the driver, this is overridden by the
       DRV_SST25VF064C_QUEUE_SIZE macro in system_config.h */
    uint32_t queueSize;

} DRV_SST25VF064C_INIT;

// *****************************************************************************
// *****************************************************************************
// Section: SPI Flash Driver Module Interface Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_SST25VF064C_Initialize
    (
        const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init
    );

  Summary:
    Initializes the SST25VF064C SPI Flash Driver instance for the specified
    driver index.

  Description:
    This function initializes the SPI Flash driver instance for the specified
    driver index, making it ready for clients to open and use it.

  Precondition:
    None.

  Parameters :
    index -  Identifier for the instance to be initialized

    init -   Pointer to a data structure containing data necessary to
             initialize the driver.

  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, it returns SYS_MODULE_OBJ_INVALID.

  Example:
    <code>
    // This code snippet shows an example of initializing the SST25VF064C SPI
    // Flash Driver. SPI driver index 0 is used for the purpose. Pin numbers 1, 2
    // and 3 of port channel B are configured for hold pin, write protection pin
    // and chip select pin respectively. Maximum buffer queue size is set 5.

    DRV_SST25VF064C_INIT   SST25VF064CInitData;
    SYS_MODULE_OBJ      objectHandle;

    SST25VF064CInitData.moduleInit.value      = SYS_MODULE_POWER_RUN_FULL;
    SST25VF064CInitData.spiDriverModuleIndex  = DRV_SPI_INDEX_0;
    SST25VF064CInitData.holdPortChannel       = PORT_CHANNEL_B;
    SST25VF064CInitData.holdBitPosition       = PORTS_BIT_POS_1;
    SST25VF064CInitData.writeProtectPortChannel = PORT_CHANNEL_B;
    SST25VF064CInitData.writeProtectBitPosition = PORTS_BIT_POS_2;
    SST25VF064CInitData.chipSelectPortChannel = PORT_CHANNEL_F;
    SST25VF064CInitData.chipSelectBitPosition = PORTS_BIT_POS_2;
    SST25VF064CInitData.queueSize = 5;

    objectHandle = DRV_SST25VF064C_Initialize(DRV_SST25VF064C_INDEX_0,
                                    (SYS_MODULE_INIT*)SST25VF064CInitData);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>

  Remarks:
    This function must be called before any other SPI Flash function is called.

    This function should only be called once during system initialization
    unless DRV_SST25VF064C_Deinitialize is called to deinitialize the driver
    instance.

    Build configuration options may be used to statically override options
    in the "init" structure and will take precedence over initialization
    data passed using this function.
*/

SYS_MODULE_OBJ DRV_SST25VF064C_Initialize
(
    const SYS_MODULE_INDEX index,
    const SYS_MODULE_INIT * const init
);

//******************************************************************************
/* Function:
    void DRV_SST25VF064C_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the SPI Flash driver module.

  Description:
    Deinitializes the specified instance of the SPI Flash Driver module,
    disabling its operation (and any hardware) and invalidates all of the
    internal data.

  Precondition:
    Function DRV_SST25VF064C_Initialize should have been called before calling
    this function.

  Parameters:
    object -  Driver object handle, returned from the DRV_SST25VF064C_Initialize
              function
  Returns:
    None.

  Example:
    <code>
    // This code snippet shows an example of deinitializing the driver.

    SYS_MODULE_OBJ      object;     //  Returned from DRV_SST25VF064C_Initialize
    SYS_STATUS          status;


    DRV_SST25VF064C_Deinitialize(object);

    status = DRV_SST25VF064C_Status(object);
    if (SYS_MODULE_DEINITIALIZED != status)
    {
        // Check again later if you need to know
        // when the driver is deinitialized.
    }
    </code>

  Remarks:
    Once the Initialize operation has been called, the Deinitialize
    operation must be called before the Initialize operation can be called
    again. This function will NEVER block waiting for hardware.
*/

void DRV_SST25VF064C_Deinitialize( SYS_MODULE_OBJ object);

//*************************************************************************
/* Function:
    SYS_STATUS DRV_SST25VF064C_Status( SYS_MODULE_OBJ object )

  Summary:
    Gets the current status of the SPI Flash Driver module.

  Description:
    This function provides the current status of the SPI Flash Driver module.

  Precondition:
    Function DRV_SST25VF064C_Initialize should have been called before calling
    this function.

  Parameters:
    object -  Driver object handle, returned from the DRV_SST25VF064C_Initialize
              function
  Returns:
    SYS_STATUS_READY - Indicates that the driver is ready and accept requests
                       for new operations

    SYS_STATUS_UNINITIALIZED - Indicates that the driver is not initialized

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_SST25VF064C_Initialize
    SYS_STATUS          SST25VF064CStatus;

    SST25VF064CStatus = DRV_SST25VF064C_Status(object);
    else if (SYS_STATUS_ERROR >= SST25VF064CStatus)
    {
        // Handle error
    }
    </code>

  Remarks:
    A driver can only be opened when its status is SYS_STATUS_READY.

*/

SYS_STATUS DRV_SST25VF064C_Status( SYS_MODULE_OBJ object);

// ***************************************************************************
/* Function:
    void DRV_SST25VF064C_Tasks ( SYS_MODULE_OBJ object );

  Summary:
    Maintains the driver's read, erase, and write state machine and implements
    its ISR.

  Description:
    This function is used to maintain the driver's internal state machine
    and should be called from the system's Tasks function.

  Precondition:
    The DRV_SST25VF064C_Initialize function must have been called for the
    specified SPI Flash driver instance.

  Parameters:
    object -  Object handle for the specified driver instance (returned from
              DRV_SST25VF064C_Initialize)
  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_SST25VF064C_Initialize

    while (true)
    {
        DRV_SST25VF064C_Tasks (object);

        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application. It is
    called by the system's Tasks function (SYS_Tasks).
*/

void DRV_SST25VF064C_Tasks ( SYS_MODULE_OBJ object );

// *****************************************************************************
// *****************************************************************************
// Section: SPI Flash Driver Client Routines
// *****************************************************************************
// *****************************************************************************

/* Function:
    DRV_HANDLE DRV_SST25VF064C_Open
    (
        const SYS_MODULE_INDEX drvIndex,
        const DRV_IO_INTENT ioIntent
    );

  Summary:
    Opens the specified SPI Flash driver instance and returns a handle to it.

  Description:
    This function opens the specified SPI Flash driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver.

  Precondition:
    Function DRV_SST25VF064C_Initialize must have been called before calling
    this function.

  Parameters:
    drvIndex -  Identifier for the object instance to be opened
    ioIntent -  Zero or more of the values from the enumeration
                DRV_IO_INTENT "ORed" together to indicate the intended use
                of the driver

  Returns:
    If successful, the function returns a valid open-instance handle (a
    number identifying both the caller and the module instance).

    If an error occurs, the return value is DRV_HANDLE_INVALID. Errors can occur
	under the following circumstances:
    - if the number of client objects allocated via
         DRV_SST25VF064C_CLIENTS_NUMBER is insufficient
    - if the client is trying to open the driver but driver has been opened
         exclusively by another client
    - if the driver hardware instance being opened is not initialized or is
         invalid
    - if the client is trying to open the driver exclusively, but has already
         been opened in a non exclusive mode by another client.

  Example:
    <code>
    DRV_HANDLE handle;

    handle = DRV_SST25VF064C_Open(DRV_SST25VF064C_INDEX_0,
                                                    DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        // Unable to open the driver
    }
    </code>

  Remarks:
    The driver will always work in Non-Blocking mode even if IO-intent is
    selected as blocking.

    The handle returned is valid until the DRV_SST25VF064C_Close function is
    called.

    This function will NEVER block waiting for hardware.
*/

DRV_HANDLE DRV_SST25VF064C_Open
(
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
);

// *****************************************************************************
/* Function:
    void DRV_SST25VF064C_Close( DRV_Handle handle );

  Summary:
    Closes an opened-instance of the SPI Flash driver.

  Description:
    This function closes an opened-instance of the SPI Flash driver, invalidating
    the handle.

  Precondition:
    The DRV_SST25VF064C_Initialize function must have been called for the
    specified SPI Flash driver instance.

    DRV_SST25VF064C_Open must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    None.

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_SST25VF064C_Open

    DRV_SST25VF064C_Close(handle);
    </code>

  Remarks:
    After calling this function, the handle passed in "handle" must not be used
    with any of the remaining driver routines.  A new handle must be obtained by
    calling DRV_SST25VF064C_Open before the caller may use the driver again.

    Note: Usually, there is no need for the driver client to verify that the
    Close operation has completed.
*/

void DRV_SST25VF064C_Close( const DRV_HANDLE handle);

// ****************************************************************************
/* Function:
    DRV_SST25VF064C_CLIENT_STATUS DRV_SST25VF064C_ClientStatus(DRV_HANDLE handle);

  Summary:
    Gets current client-specific status of the SPI Flash driver.

  Description:
    This function gets the client-specific status of the SPI Flash driver
    associated with the given handle.

  Precondition:
    The DRV_SST25VF064C_Initialize function must have been called.

    DRV_SST25VF064C_Open must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle -  A valid open instance handle, returned from the driver's open
              function

  Returns:
    A DRV_SST25VF064C_CLIENT_STATUS value describing the current status of the
    driver.

  Example:
    <code>
    DRV_HANDLE      handle;         // Returned from DRV_SST25VF064C_Open
    DRV_SST25VF064C_CLIENT_STATUS     clientStatus;

    clientStatus = DRV_SST25VF064C_ClientStatus(handle);
    if(DRV_SST25VF064C_CLIENT_STATUS_READY == clientStatus)
    {
        // do the tasks
    }
    </code>

  Remarks:
    This function will not block for hardware access and will immediately
    return the current status.
*/

DRV_SST25VF064C_CLIENT_STATUS   DRV_SST25VF064C_ClientStatus( const DRV_HANDLE handle );

// *****************************************************************************
/* Function:
    void DRV_SST25VF064C_BlockEventHandlerSet
    (
        const DRV_HANDLE handle,
        const DRV_SST25VF064C_EVENT_HANDLER eventHandler,
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

  Precondition:
    The DRV_SST25VF064C_Initialize function must have been called for the
    specified SPI FLash driver instance.

    DRV_SST25VF064C_Open must have been called to obtain a valid opened device
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
    DRV_SST25VF064C_BLOCK_COMMAND_HANDLE commandHandle;

    // mySST25VF064CHandle is the handle returned
    // by the DRV_SST25VF064C_Open function.

    // Client registers an event handler with driver. This is done once.

    DRV_SST25VF064C_BlockEventHandlerSet( mySST25VF064CHandle,
                    APP_SST25VF064CEventHandler, (uintptr_t)&myAppObj );

    DRV_SST25VF064C_BlockRead( mySST25VF064CHandle, commandHandle,
                                            &myBuffer, blockStart, nBlock );

    if(DRV_SST25VF064C_BLOCK_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event Processing Technique. Event is received when operation is done.

    void APP_SST25VF064CEventHandler(DRV_SST25VF064C_BLOCK_EVENT event,
            DRV_SST25VF064C_BLOCK_COMMAND_HANDLE handle, uintptr_t context)
    {
        // The context handle was set to an application specific
        // object. It is now retrievable easily in the event handler.
        MY_APP_OBJ myAppObj = (MY_APP_OBJ *) context;

        switch(event)
        {
            case DRV_SST25VF064C_EVENT_BLOCK_COMMAND_COMPLETE:

                // This means the data was transferred.
                break;

            case DRV_SST25VF064C_EVENT_BLOCK_COMMAND_ERROR:

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

void DRV_SST25VF064C_BlockEventHandlerSet
(
    const DRV_HANDLE handle,
    const DRV_SST25VF064C_EVENT_HANDLER eventHandler,
    const uintptr_t context
);

// **************************************************************************
/* Function:
    void DRV_SST25VF064C_BlockErase
    (
        const DRV_HANDLE handle,
        DRV_SST25VF064C_BLOCK_COMMAND_HANDLE * commandHandle,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Erase the specified number of blocks in Flash memory.

  Description:
    This function schedules a non-blocking erase operation in Flash memory.
    The function returns with a valid erase handle in the commandHandle argument
    if the erase request was scheduled successfully. The function adds the
    request to the hardware instance queue and returns immediately.
    The function returns DRV_SST25VF064C_BLOCK_COMMAND_HANDLE_INVALID in the
    commandHandle argument under the following circumstances:
    - if the client opened the driver for read only
    - if nBlock is 0
    - if the queue size is full or queue depth is insufficient
    - if the driver handle is invalid
    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_SST25VF064C_EVENT_ERASE_COMPLETE event if the
    erase operation was successful or DRV_SST25VF064C_EVENT_ERASE_ERROR
    event if the erase operation was not successful.

  Precondition:
    The DRV_SST25VF064C_Initialize function must have been called for the
    specified SPI Flash driver instance.

    DRV_SST25VF064C_Open must have been called to obtain a valid opened device
    handle.

    DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have been specified
    in the DRV_SST25VF064C_Open call.

  Parameters:
    handle -       A valid open-instance handle, returned from the
                   driver's open function
    commandHandle -  Pointer to an argument that will contain the return buffer
                   handle
    blockStart -   Start block address in SST25VF064C memory from where the
                   erase should begin. LSBs (A0-A11) of block start address
                   will be ignored to align it with Erase block size boundary.

    nBlock -       Total number of blocks to be erased. Each Erase block is of
                   size 4 KByte.

  Returns:
    The buffer handle is returned in the commandHandle argument. It Will be
    DRV_BUFFER_HANDLE_INVALID if the request was not queued.

  Example:
    <code>

    // Destination address should be block aligned.
    uint32_t blockStart;
    uint32_t nBlock;
    DRV_SST25VF064C_BLOCK_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;

    // mySST25VF064CHandle is the handle returned
    // by the DRV_SST25VF064C_Open function.

    // Client registers an event handler with driver

    DRV_SST25VF064C_BlockEventHandlerSet(mySST25VF064CHandle,
                    APP_SST25VF064CEventHandler, (uintptr_t)&myAppObj);

    DRV_SST25VF064C_BlockErase( mySST25VF064CHandle, commandHandle,
                                                        blockStart, nBlock );

    if(DRV_SST25VF064C_BLOCK_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer queue is processed.

    void APP_SST25VF064CEventHandler(DRV_SST25VF064C_BLOCK_EVENT event,
            DRV_SST25VF064C_BLOCK_COMMAND_HANDLE commandHandle, uintptr_t contextHandle)
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_SST25VF064C_EVENT_ERASE_COMPLETE:

                // This means the data was transferred.
                break;

            case DRV_SST25VF064C_EVENT_ERASE_ERROR:

                // Error handling here.

                break;

            default:
                break;
        }
    }

    </code>

  Remarks:
    Write Protection will be disabled for the complete Flash memory region
    in the beginning by default.
*/

void DRV_SST25VF064C_BlockErase
(
    const DRV_HANDLE handle,
    DRV_SST25VF064C_BLOCK_COMMAND_HANDLE * commandHandle,
    uint32_t blockStart,
    uint32_t nBlock
);

// *****************************************************************************
/* Function:
    void DRV_SST25VF064C_BlockRead
    (
        const DRV_HANDLE handle,
        DRV_SST25VF064C_BLOCK_COMMAND_HANDLE * commandHandle,
        uint8_t *targetBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Reads blocks of data starting from the specified address in Flash memory.

  Description:
    This function schedules a non-blocking read operation for reading blocks of
    data from Flash memory. The function returns with a valid handle in
    the commandHandle argument if the read request was scheduled successfully.
    The function adds the request to the hardware instance queue and
    returns immediately. While the request is in the queue, the application
    buffer is owned by the driver and should not be modified.  The function
    returns DRV_SST25VF064C_BLOCK_COMMAND_HANDLE_INVALID in the commandHandle
    argument under the following circumstances:
    - if a buffer could not be allocated to the request
    - if the target buffer pointer is NULL
    - if the client opened the driver for write only
    - if the buffer size is 0
    - if the read queue size is full or queue depth is insufficient
    - if the driver handle is invalid
    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_SST25VF064C_EVENT_BLOCK_COMMAND_COMPLETE event if the
    buffer was processed successfully of DRV_SST25VF064C_EVENT_BLOCK_COMMAND_ERROR
    event if the buffer was not processed successfully.

  Precondition:
    The DRV_SST25VF064C_Initialize function must have been called for the
    specified SPI Flash driver instance.

    DRV_SST25VF064C_Open must have been called to obtain a valid opened device
    handle.

    DRV_IO_INTENT_READ or DRV_IO_INTENT_READWRITE must have been specified in
    the DRV_SST25VF064C_Open call.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open function

    commandHandle - Pointer to an argument that will contain the return buffer
                    handle

    *targetBuffer - Buffer into which the data read from the SPI Flash instance
                    will be placed

    blockStart    - Start block address in SST25VF064C memory from where the
                    read should begin. It can be any address of the Flash.

    nBlock        - Total number of blocks to be read. Each Read block is of 1
                    byte.

  Returns:
    The buffer handle is returned in the commandHandle argument. It will be
    DRV_BUFFER_HANDLE_INVALID if the request was not successful.

  Example:
    <code>

    uint8_t myBuffer[MY_BUFFER_SIZE];

    // address should be block aligned.
    uint32_t blockStart = SST25VF064C_BASE_ADDRESS_TO_READ_FROM;
    uint32_t    nBlock = 2;
    DRV_SST25VF064C_BLOCK_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;

    // mySST25VF064CHandle is the handle returned
    // by the DRV_SST25VF064C_Open function.

    // Client registers an event handler with driver

    DRV_SST25VF064C_BlockEventHandlerSet(mySST25VF064CHandle,
                    APP_SST25VF064CEventHandler, (uintptr_t)&myAppObj);

    DRV_SST25VF064C_BlockRead( mySST25VF064CHandle, commandHandle,
                                            &myBuffer, blockStart, nBlock );

    if(DRV_SST25VF064C_BLOCK_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event is received when the buffer is processed.

    void APP_SST25VF064CEventHandler(DRV_SST25VF064C_BLOCK_EVENT event,
            DRV_SST25VF064C_BLOCK_COMMAND_HANDLE commandHandle, uintptr_t contextHandle)
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_SST25VF064C_EVENT_BLOCK_COMMAND_COMPLETE:

                // This means the data was transferred.
                break;

            case DRV_SST25VF064C_EVENT_BLOCK_COMMAND_ERROR:

                // Error handling here.

                break;

            default:
                break;
        }
    }

    </code>

  Remarks:
    The maximum read speed is 33 MHz.
*/

void DRV_SST25VF064C_BlockRead
(
    const DRV_HANDLE handle,
    DRV_SST25VF064C_BLOCK_COMMAND_HANDLE * commandHandle,
    uint8_t *targetBuffer,
    uint32_t blockStart,
    uint32_t nBlock
);

// *****************************************************************************
/* Function:
    void DRV_SST25VF064C_BlockWrite
    (
        DRV_HANDLE handle,
        DRV_SST25VF064C_BLOCK_COMMAND_HANDLE * commandHandle,
        uint8_t *sourceBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Write blocks of data starting from a specified address in Flash memory.

  Description:
    This function schedules a non-blocking write operation for writing blocks of
    data into Flash memory. The function returns with a valid buffer handle in
    the commandHandle argument if the write request was scheduled successfully.
    The function adds the request to the hardware instance queue and
    returns immediately. While the request is in the queue, the application
    buffer is owned by the driver and should not be modified.  The function
    returns DRV_SST25VF064C_BLOCK_COMMAND_HANDLE_INVALID in the commandHandle
    argument under the following circumstances:
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the client opened the driver for read only
    - if the buffer size is 0
    - if the write queue size is full or queue depth is insufficient
    - if the driver handle is invalid
    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_SST25VF064C_EVENT_BLOCK_COMMAND_COMPLETE event if the
    buffer was processed successfully or DRV_SST25VF064C_EVENT_BLOCK_COMMAND_ERROR
    event if the buffer was not processed successfully.

  Precondition:
    The DRV_SST25VF064C_Initialize function must have been called for the
    specified SPI Flash driver instance.

    DRV_SST25VF064C_Open must have been called to obtain a valid opened
    device handle.

    DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have been specified in
    the DRV_SST25VF064C_Open call.

    The Flash address location which has to be written, must be erased before
    using the API DRV_SST25VF064C_BlockErase().

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

    commandHandle -Pointer to an argument that will contain the return buffer
                   handle

    sourceBuffer - The source buffer containing data to be programmed into SPI
                   Flash

    blockStart -   Start block address of SST25VF064C Flash where the write
                   should begin. It can be any address of the Flash.
    nBlock -       Total number of blocks to be written. Each write block is of
                   1 byte.

  Returns:
    The buffer handle is returned in the commandHandle argument. It will be
    DRV_BUFFER_HANDLE_INVALID if the request was not successful.

  Example:
    <code>

    uint8_t myBuffer[MY_BUFFER_SIZE];

    // address should be block aligned.
    uint32_t blockStart = SST25VF064C_BASE_ADDRESS_TO_WRITE_TO;
    uint32_t    nBlock = 2;
    DRV_SST25VF064C_BLOCK_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;

    // mySST25VF064CHandle is the handle returned
    // by the DRV_SST25VF064C_Open function.

    // Client registers an event handler with driver

    DRV_SST25VF064C_BlockEventHandlerSet(mySST25VF064CHandle,
                    APP_SST25VF064CEventHandler, (uintptr_t)&myAppObj);

    DRV_SST25VF064C_BlockWrite( mySST25VF064CHandle, commandHandle,
                                            &myBuffer, blockStart, nBlock );

    if(DRV_SST25VF064C_BLOCK_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer is processed.

    void APP_SST25VF064CEventHandler(DRV_SST25VF064C_BLOCK_EVENT event,
            DRV_SST25VF064C_BLOCK_COMMAND_HANDLE commandHandle, uintptr_t contextHandle)
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_SST25VF064C_EVENT_BLOCK_COMMAND_COMPLETE:

                // This means the data was transferred.
                break;

            case DRV_SST25VF064C_EVENT_BLOCK_COMMAND_ERROR:

                // Error handling here.

                break;

            default:
                break;
        }
    }

    </code>

  Remarks:
    In the case of multi bytes write operation, byte by byte writing will happen
    instead of Address auto Increment writing.

    Write Protection will be disabled for the complete Flash memory region
    in the beginning by default.
*/

void DRV_SST25VF064C_BlockWrite
(
    DRV_HANDLE handle,
    DRV_SST25VF064C_BLOCK_COMMAND_HANDLE * commandHandle,
    uint8_t *sourceBuffer,
    uint32_t blockStart,
    uint32_t nBlock
);

// *****************************************************************************
/* Function:
    DRV_SST25VF064C_COMMAND_STATUS DRV_SST25VF064C_CommandStatus
    (
        const DRV_HANDLE handle,
        const DRV_SST25VF064C_BLOCK_COMMAND_HANDLE commandHandle
    );

  Summary:
    Gets the current status of the command.

  Description:
    This routine gets the current status of the buffer. The application must use
    this routine where the status of a scheduled buffer needs to polled on. The
    function may return DRV_SST25VF064C_BLOCK_COMMAND_HANDLE_INVALID in a case
    where the buffer handle has expired. A buffer handle expires when the
    internal buffer object is re-assigned to another erase, read or write
    request. It is recommended that this function be called regularly in order
    to track the buffer status correctly.

    The application can alternatively register an event handler to receive
    write, read or erase operation completion events.

  Precondition:
    Block command request must have been made using Erase, Read or Write APIs
    to get a valid command handle.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open routine
    commandHandle -  A valid command handle, returned from Read/Write/Erase APIs.

  Returns:
    A DRV_SST25VF064C_COMMAND_STATUS value describing the current status of the
    buffer.
    Returns DRV_SST25VF064C_BLOCK_COMMAND_HANDLE_INVALID if the client handle
    or the command handle is not valid.

  Example:
    <code>
    DRV_HANDLE      sstOpenHandle;   // Returned from DRV_SST25VF064C_Open
    DRV_SST25VF064C_BLOCK_COMMAND_HANDLE  commandHandle;
    DRV_SST25VF064C_BlockErase
                            (
                                sstOpenHandle,
                                &commandHandle,
                                0,
                                1
                            );

    if(DRV_SST25VF064C_CommandStatus(sstOpenHandle, commandHandle) == DRV_SST25VF064C_COMMAND_COMPLETED );
    {
        // do something
    }
    </code>

  Remarks:
    This function will not block for hardware access and will immediately
    return the current status.
*/

DRV_SST25VF064C_COMMAND_STATUS   DRV_SST25VF064C_CommandStatus
( 
    const DRV_HANDLE handle,
    const DRV_SST25VF064C_BLOCK_COMMAND_HANDLE  commandHandle
);

// *****************************************************************************
/* Function:
    SYS_FS_MEDIA_GEOMETRY DRV_SST25VF064C_GeometryGet( DRV_HANDLE handle );

  Summary:
    Returns the geometry of the device.

  Description:
    This API gives the following geometrical details of the SST25VF064C Flash:
    - Media Property
    - Number of Read/Write/Erase regions in the Flash device
    - Number of Blocks and their size in each region of the device

  Precondition:
    None.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    SYS_FS_MEDIA_GEOMETRY - Structure which holds the media geometry information.

  Example:
    <code>

    SYS_FS_MEDIA_GEOMETRY * sstFlashGeometry;
    uint32_t readBlockSize, writeBlockSize, eraseBlockSize;
    uint32_t nReadBlocks, nReadRegions, totalFlashSize;

    sstFlashGeometry = DRV_SST25VF064C_GeometryGet(sstOpenHandle1);

    // read block size should be 1 byte
    readBlockSize  = sstFlashGeometry->geometryTable->blockSize;
    nReadBlocks = sstFlashGeometry->geometryTable->numBlocks;
    nReadRegions = sstFlashGeometry->numReadRegions;

    // write block size should be 1 byte
    writeBlockSize  = (sstFlashGeometry->geometryTable +1)->blockSize;
    // erase block size should be 4k byte
    eraseBlockSize  = (sstFlashGeometry->geometryTable +2)->blockSize;

    // total Flash size should be 8 MB
    totalFlashSize = readBlockSize * nReadBlocks * nReadRegions;

    </code>

  Remarks:
    This function is typically used by File System Media Manager.
*/

SYS_FS_MEDIA_GEOMETRY * DRV_SST25VF064C_GeometryGet( DRV_HANDLE handle );

// *****************************************************************************
/* Function:
    bool DRV_SST25VF064C_MediaIsAttached(DRV_HANDLE handle);

  Summary:
    Returns the status of the media.

  Description:
    This function determines whether or not the media is attached.

  Precondition:
    None.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    - True         - Media is attached
    - False        - Media is not attached

  Example:
    <code>

    if (DRV_SST25VF064C_MediaIsAttached(handle))
    {
    // Do Something
    }

    </code>

  Remarks:
    This function is typically used by File System Media Manager.
*/

bool DRV_SST25VF064C_MediaIsAttached(DRV_HANDLE handle);

// ****************************************************************************
// ****************************************************************************
// Section: Included Files (continued)
// ****************************************************************************
// ****************************************************************************

/*  The files included below map the interface definitions above to appropriate
    static implementations, depending on build mode.
*/

// #include "driver/SST25VF064C/drv_SST25VF064C_mapping.h"

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // #ifndef _DRV_SST25VF064C_H
/*******************************************************************************
 End of File
*/

