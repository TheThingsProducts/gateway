/*******************************************************************************
  SPI Driver Interface

  Company:
    Microchip Technology Inc.

  File Name:
    drv_spi.h

  Summary:
    SPI device driver interface file.

  Description:
    The SPI  driver provides a simple interface to manage the SPI module.
    This file defines the interface definitions and prototypes for the SPI driver.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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


#ifndef _DRV_SPI_H
#define _DRV_SPI_H


// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "system_config.h"
#include "driver/driver_common.h"			// Common Driver Definitions
#include "peripheral/spi/plib_spi.h"		// SPI PLIB Header
#include "system/common/sys_common.h"      	// Common System Service Definitions
#include "system/common/sys_module.h"      	// Module/Driver Definitions
#include "system/int/sys_int.h"            	// System Interrupt Definitions
#include "system/clk/sys_clk.h"
#include "system/ports/sys_ports.h"
#include "system/dma/sys_dma.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END  

// *****************************************************************************
/* SPI Driver Buffer Handle

  Summary:
    Handle identifying a read or write buffer passed to the driver.

  Description:
    A buffer handle value is returned by a call to the DRV_SPI_BufferAddRead()/
    DRV_SPI_BufferAddWrite or DRV_SPI_BufferAddReadWrite() functions. This handle
	is associated with the buffer passed into the function and it allows the
	application to track the completion of the data from (or into) that buffer.
	The buffer handle value returned from the "buffer add" function is returned
	back to the client by the "callback" function registered with the driver.

    The buffer handle assigned to a client request expires when the client has
    been notified of the completion of the buffer transfer (after event handler
    function that notifies the client returns) or after the buffer has been
    retired by the driver if no event handler callback was set.

  Remarks:
    None.
*/

typedef uintptr_t DRV_SPI_BUFFER_HANDLE;



// *****************************************************************************
/* SPI Driver Invalid Buffer Handle

  Summary:
    Definition of an invalid buffer handle.

  Description:
    This is the definition of an invalid buffer handle. An invalid buffer handle
    is returned by DRV_SPI_BufferAddRead() and DRV_SPI_BufferAddWrite()
    function if the buffer add request was not successful.

  Remarks:
    None.
*/

#define DRV_SPI_BUFFER_HANDLE_INVALID ((DRV_SPI_BUFFER_HANDLE)(-1))


// *****************************************************************************
/* SPI Driver Module Index Numbers

  Summary:
    SPI driver index definitions.

  Description:
    These constants provide the SPI driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the DRV_SPI_Initialize and
    DRV_SPI_Open functions to identify the driver instance in use.
*/

#define DRV_SPI_INDEX_0         0
#define DRV_SPI_INDEX_1         1
#define DRV_SPI_INDEX_2         2
#define DRV_SPI_INDEX_3         3
#define DRV_SPI_INDEX_4         4
#define DRV_SPI_INDEX_5         5


// *****************************************************************************
/* SPI Driver Module Index Count

  Summary:
    Number of valid SPI driver indices.

  Description:
    This constant identifies the number of valid SPI driver indices.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.

    This value is derived from device-specific header files defined as part of the
    peripheral libraries.
*/

#define DRV_SPI_INDEX_COUNT     SPI_NUMBER_OF_MODULES


// *****************************************************************************
/* SPI Clock Mode Selection

  Summary:
    Identifies the various clock modes of the SPI module.

  Description:
    This enumeration identifies the various clock modes of the SPI module.

  Remarks:
    None.
*/

typedef enum
{
    /* SPI Clock Mode 0 - Idle State Low & Sampling on Rising Edge */
    DRV_SPI_CLOCK_MODE_IDLE_LOW_EDGE_RISE
        /*DOM-IGNORE-BEGIN*/  = 0 /*DOM-IGNORE-END*/,

    /* SPI Clock Mode 1 - Idle State Low & Sampling on Falling Edge */
    DRV_SPI_CLOCK_MODE_IDLE_LOW_EDGE_FALL
        /*DOM-IGNORE-BEGIN*/  = 1 /*DOM-IGNORE-END*/,

    /* SPI Clock Mode 2 - Idle State High & Sampling on Falling Edge */
    DRV_SPI_CLOCK_MODE_IDLE_HIGH_EDGE_FALL
        /*DOM-IGNORE-BEGIN*/  = 2 /*DOM-IGNORE-END*/,

    /* SPI Clock Mode 3 - Idle State High & Sampling on Rising Edge */
    DRV_SPI_CLOCK_MODE_IDLE_HIGH_EDGE_RISE
        /*DOM-IGNORE-BEGIN*/  = 3 /*DOM-IGNORE-END*/

} DRV_SPI_CLOCK_MODE;


// *****************************************************************************
/* SPI Buffer Type Selection

  Summary:
    Identifies the various buffer types of the SPI module.

  Description:
    This enumeration identifies the various buffer types of the SPI module.

  Remarks:
    None.
*/

typedef enum
{
    /* SPI Buffer Type Standard */
    DRV_SPI_BUFFER_TYPE_STANDARD
        /*DOM-IGNORE-BEGIN*/  = 0 /*DOM-IGNORE-END*/,

    /* SPI Enhanced Buffer Type */
    DRV_SPI_BUFFER_TYPE_ENHANCED
        /*DOM-IGNORE-BEGIN*/  = 1 /*DOM-IGNORE-END*/

} DRV_SPI_BUFFER_TYPE;


// *****************************************************************************
/* SPI Protocols Enumeration

  Summary:
    Identifies the various protocols of the SPI module.

  Description:
    This enumeration identifies the various protocols of the SPI module.

  Remarks:
    None.
*/

typedef enum
{
    /* SPI Protocol Type Standard */
    DRV_SPI_PROTOCOL_TYPE_STANDARD
        /*DOM-IGNORE-BEGIN*/  = 0 /*DOM-IGNORE-END*/,

    /* SPI Protocol Type Framed */
    DRV_SPI_PROTOCOL_TYPE_FRAMED
        /*DOM-IGNORE-BEGIN*/  = 1 /*DOM-IGNORE-END*/,

    /*SPI Protocol Type Audio*/
    DRV_SPI_PROTOCOL_TYPE_AUDIO
        /*DOM-IGNORE-BEGIN*/  = 2 /*DOM-IGNORE-END*/

} DRV_SPI_PROTOCOL_TYPE;




// *****************************************************************************
/* SPI Driver Buffer Events

   Summary
    Identifies the possible events that can result from a buffer add request.

   Description
    This enumeration identifies the possible events that can result from a
    buffer add request caused by the client calling either the
    DRV_SPI_BufferAddRead or DRV_SPI_BufferAddWrite functions.

   Remarks:
    One of these values is passed in the "event" parameter of the event
    handling callback function that the client registered with the driver by
    calling the DRV_SPI_BufferEventHandlerSet function when a buffer
    transfer request is completed.

*/

typedef enum
{
    /* Buffer is pending to get processed */
    DRV_SPI_BUFFER_EVENT_PENDING,

    /* Buffer is being processed */
    DRV_SPI_BUFFER_EVENT_PROCESSING,

    /* All data from or to the buffer was transferred successfully. */
    DRV_SPI_BUFFER_EVENT_COMPLETE,

    /* There was an error while processing the buffer transfer request. */
    DRV_SPI_BUFFER_EVENT_ERROR

}DRV_SPI_BUFFER_EVENT;


// *****************************************************************************
/* SPI Usage Modes Enumeration

  Summary:
    Identifies the various usage modes of the SPI module.

  Description:
    This enumeration identifies the various usage modes of the SPI module.

  Remarks:
    None.
*/

typedef enum
{
    /* SPI Mode Master */
    DRV_SPI_MODE_MASTER
        /*DOM-IGNORE-BEGIN*/  = 0 /*DOM-IGNORE-END*/,

    /* SPI Mode Slave */
    DRV_SPI_MODE_SLAVE
        /*DOM-IGNORE-BEGIN*/  = 1 /*DOM-IGNORE-END*/

} DRV_SPI_MODE;


// *****************************************************************************
/* SPI Task Modes Enumeration

  Summary:
    Identifies the various modes of how the tasks function will be run.

  Description:
    This enumeration identifies the various tasks mode

  Remarks:
    None.
*/


typedef enum
{
    /* Task is configured to run in polled mode */
    DRV_SPI_TASK_MODE_POLLED
        /*DOM-IGNORE-BEGIN*/  = 0 /*DOM-IGNORE-END*/,

    /* Task is configured to run in interrupt mode */
    DRV_SPI_TASK_MODE_ISR
        /*DOM-IGNORE-BEGIN*/  = 1 /*DOM-IGNORE-END*/

} DRV_SPI_TASK_MODE;

// *****************************************************************************
/* SPI Driver Buffer Event Handler Function Pointer

   Summary:
    Pointer to a SPI Driver Buffer Event handler function

   Description:
    This data type defines the required function signature for the SPI driver
    buffer event handling callback function. A client must register a pointer
    to a buffer event handling function who's function signature (parameter
    and return value types) match the types specified by this function pointer
    in order to receive buffer related event calls back from the driver.

    The parameters and return values and return value are described here and
    a partial example implementation is provided.

  Parameters:
    event           - Identifies the type of event

    bufferHandle    - Handle identifying the buffer to which the vent relates

	context         - Value identifying the context of the application that
					  registered the event handling function.

  Returns:
    None.

  Example:
    <code>
    void APP_MyBufferEventHandler( DRV_SPI_BUFFER_EVENT event,
                                   DRV_SPI_BUFFER_HANDLE bufferHandle,
                                   uintptr_t context )
    {
        MY_APP_DATA_STRUCT pAppData = (MY_APP_DATA_STRUCT) context;

        switch(event)
        {
            case DRV_SPI_BUFFER_EVENT_COMPLETE:

                // Handle the completed buffer.
                break;

            case DRV_SPI_BUFFER_EVENT_ERROR:
            default:

                // Handle error.
                break;
        }
    }
    </code>

  Remarks:
    If the event is DRV_SPI_BUFFER_EVENT_COMPLETE, it means that the data was
    transferred successfully.

    If the event is DRV_SPI_BUFFER_EVENT_ERROR, it means that the data was
    not transferred successfully.

    The bufferHandle parameter contains the buffer handle of the buffer that
    failed.

    The context parameter contains the a handle to the client context,
    provided at the time the event handling function was  registered using the
    DRV_SPI_BufferEventHandlerSet function.  This context handle value is
    passed back to the client as the "context" parameter.  It can be any value
    necessary to identify the client context or instance (such as a pointer to
    the client's data) instance of the client that made the buffer add request.

    The event handler function executes in an interrupt context when the driver
    is configured for interrupt mode operation. It is recommended of the
    application to not perform process intensive operations with in this
    function.
*/

typedef void ( *DRV_SPI_BUFFER_EVENT_HANDLER )  (DRV_SPI_BUFFER_EVENT event,
        DRV_SPI_BUFFER_HANDLE bufferHandle, void * context );


// *****************************************************************************
/* SPI Driver Initialization Data

  Summary:
    Defines the data required to initialize or reinitialize the SPI driver

  Description:
    This data type defines the data required to initialize or reinitialize the
    SPI driver. If the driver is built statically, the members of this data
    structure are statically over-ridden by static override definitions in the
    system_config.h file.

  Remarks:
    None.
*/

typedef struct _DRV_SPI_INIT
{
    /* System module initialization */
    SYS_MODULE_INIT                 		moduleInit;

    /* Identifies peripheral (PLIB-level) ID */
    SPI_MODULE_ID                   		spiId;

    /* SPI Task Mode Type*/
    DRV_SPI_TASK_MODE                           taskMode;

    /* SPI Usage Mode Type */
    DRV_SPI_MODE                    		spiMode;

    /* Allow SPI to run when CPU goes to idle mode*/
    bool                                        allowIdleRun;

    /* SPI Protocol Type */
    DRV_SPI_PROTOCOL_TYPE           		spiProtocolType;

    /* SPI Slave Mode SSx Pin Select */
    bool                                    spiSlaveSSPin;

    /* Framed mode Sync Pulse*/
    SPI_FRAME_SYNC_PULSE                        frameSyncPulse;

    /* Framed Mode pulse polarity*/
    SPI_FRAME_PULSE_POLARITY                    framePulsePolarity;

    /* Framed Mode Pulse Direction*/
    SPI_FRAME_PULSE_DIRECTION                   framePulseDirection;

    /* Framed mode Pulse Edge*/
    SPI_FRAME_PULSE_EDGE                        framePulseEdge;

    /* Framed Mode Pulse width */
    SPI_FRAME_PULSE_WIDTH                       framePulseWidth;
    
    /* Audio mode transmit mode*/
    SPI_AUDIO_TRANSMIT_MODE                     audioTransmitMode;

    /* Audio mode protocol mode*/
    SPI_AUDIO_PROTOCOL                          audioProtocolMode;

    /* Communication Width */
    SPI_COMMUNICATION_WIDTH         		commWidth;

    /* Peripheral clock used by the SPI driver*/
    CLK_BUSES_PERIPHERAL                        spiClk;

    /* Baud Rate Value */
    uint32_t                        		baudRate;

    /* SPI Buffer Type */
    DRV_SPI_BUFFER_TYPE             		bufferType;

    /* SPI Clock mode */
    DRV_SPI_CLOCK_MODE              		clockMode;

    /* SPI Input Sample Phase Selection */
    SPI_INPUT_SAMPLING_PHASE        		inputSamplePhase;

    /* Transmit/Receive or Transmit Interrupt Source for SPI module */
    INT_SOURCE                      		txInterruptSource;

    /* Receive Interrupt Source for SPI module */
    INT_SOURCE                      		rxInterruptSource;

    /* Error Interrupt Source for SPI module */
    INT_SOURCE                      		errInterruptSource;

    /* While using standard buffer and polled mode how many transfers to do
       before yielding to other tasks*/
    uint8_t                                     numTrfsSmPolled;

    /* DMA Channel for the Transmitter*/
    DMA_CHANNEL                                 txDmaChannel;
    /* Threshold for the minimum number of bytes to send to use DMA*/
    uint8_t                                     txDmaThreshold;

    /* DMA Channel for the Receiver*/
    DMA_CHANNEL                                 rxDmaChannel;
    /* Threshold for the minimum number of bytes to receive to use DMA*/
    uint8_t                                     rxDmaThreshold;

	/* This is the buffer queue size. This is the maximum
     number of  transfer requests that driver will queue.  */
    uint8_t                    		            queueSize;

	/* This controls the minimum number of jobs that the driver will be able
         to accept without running out of memory.  The driver will reserve this
         number of jobs from the global SPI queue so that it will always be available*/
        uint8_t         jobQueueReserveSize;
        


        /* This callback is fired when an operation is about to start on the
           SPI bus.  This allows the user to set any pins that need to be set.
           This callback may be called from an ISR so should not include OSAL
           calls.  The context parameter is the same one passed into the
           BufferAddRead, BufferAddWrite, BufferAddWriteRead function.
         */
        DRV_SPI_BUFFER_EVENT_HANDLER operationStarting;

        /* This callback is fired when an operation has just completed on the
           SPI bus.  This allows the user to set any pins that need to be set.
           This callback may be called from an ISR so should not include OSAL
           calls.  The context parameter is the same one passed into the
           BufferAddRead, BufferAddWrite, BufferAddWriteRead function.
         */
        DRV_SPI_BUFFER_EVENT_HANDLER operationEnded;


} DRV_SPI_INIT;

// *****************************************************************************
/* SPI Driver Client Specific Configuration

  Summary:
    Defines the data that can be changed per client.

  Description:
    This data type defines the data can be configured per client.  This data can
    be per client, and overrides the configuration data contained inside of DRV_SPI_INIT.
 
  Remarks:
    None.
*/

typedef struct _DRV_SPI_CLIENT_DATA
{
    /* Baud Rate Value */
    uint32_t                        		baudRate;
    /* This callback is fired when an operation is about to start on the
       SPI bus.  This allows the user to set any pins that need to be set.
       This callback may be called from an ISR so should not include OSAL
       calls.  The context parameter is the same one passed into the
       BufferAddRead, BufferAddWrite, BufferAddWriteRead function.
     */
    DRV_SPI_BUFFER_EVENT_HANDLER operationStarting;

    /* This callback is fired when an operation has just completed on the
       SPI bus.  This allows the user to set any pins that need to be set.
       This callback may be called from an ISR so should not include OSAL
       calls.  The context parameter is the same one passed into the
       BufferAddRead, BufferAddWrite, BufferAddWriteRead function.
     */
    DRV_SPI_BUFFER_EVENT_HANDLER operationEnded;

}DRV_SPI_CLIENT_DATA;

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - System Level
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
      SYS_MODULE_OBJ DRV_SPI_Initialize( const SYS_MODULE_INDEX index,
                                           const SYS_MODULE_INIT * const init )

  Summary:
    Initializes the SPI instance for the specified driver index.
	<p><b>Implementation:</b> Static/Dynamic</p>

  Description:
    This routine initializes the SPI driver instance for the specified driver
    index, making it ready for clients to open and use it. The initialization
    data is specified by the 'init' parameter. The initialization may fail if the
    number of driver objects allocated are insufficient or if the specified
    driver instance is already initialized. The driver instance index is
    independent of the SPI module ID. For example, driver instance 0 can be
    assigned to SPI2.  If the driver is built statically, then some of the
    initialization parameters are overridden by configuration macros. Refer to
    the description of the DRV_SPI_INIT data structure for more details on
    which members on this data structure are overridden.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the instance to be initialized.  Please note this
             is not the SPI id.  The hardware SPI id is set in the initialization
             structure.  This is the index of the driver index to use.

    init   - Pointer to a data structure containing any data necessary to
             initialize the driver. If this pointer is NULL, the driver
             uses the static initialization override macros for each
             member of the initialization data structure.

  Returns:
    - If successful - returns a valid handle to a driver instance object
    - If unsuccessful - returns SYS_MODULE_OBJ_INVALID

  Example:
    <code>
    DRV_SPI_INIT        init;
    SYS_MODULE_OBJ      objectHandle;

    // Populate the SPI initialization structure
    init.spiId = SPI_ID_1,
    init.taskMode = DRV_SPI_TASK_MODE_ISR,
    init.spiMode = DRV_SPI_MODE_MASTER,
    init.allowIdleRun = false,
    init.spiProtocolType = DRV_SPI_PROTOCOL_TYPE_STANDARD,
    init.commWidth = SPI_COMMUNICATION_WIDTH_8BITS,
    init.spiClk = CLK_BUS_PERIPHERAL_2,
    init.baudRate = 10000000,
    init.bufferType = DRV_SPI_BUFFER_TYPE_ENHANCED,
    init.clockMode = DRV_SPI_CLOCK_MODE_IDLE_LOW_EDGE_FALL,
    init.inputSamplePhase = SPI_INPUT_SAMPLING_PHASE_IN_MIDDLE,
    init.txInterruptSource = INT_SOURCE_SPI_1_TRANSMIT,
    init.rxInterruptSource = INT_SOURCE_SPI_1_RECEIVE,
    init.errInterruptSource = INT_SOURCE_SPI_1_ERROR,
    init.queueSize = 10,
    init.jobQueueReserveSize = 1,

    objectHandle = DRV_SPI_Initialize(DRV_SPI_INDEX_1, (SYS_MODULE_INIT*)usartInitData);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>
  Remarks:
    This routine must be called before any other SPI routine is called.

    This routine should only be called once during system initialization
    unless DRV_SPI_Deinitialize is called to deinitialize the driver
    instance. This routine will NEVER block for hardware access.
*/

SYS_MODULE_OBJ DRV_SPI_Initialize( const SYS_MODULE_INDEX index,
                                           const SYS_MODULE_INIT * const init );

/*************************************************************************
  Function:
       void DRV_SPI_Deinitialize ( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the SPI driver module.
	<p><b>Implementation:</b> Static/Dynamic</p>

  Description:
    Deinitializes the specified instance of the SPI driver module,
    disabling its operation (and any hardware) and invalidates all of the
    internal data.

  Precondition:
    Function DRV_SPI_Initialize must have been called before calling this
    routine and a valid SYS_MODULE_OBJ must have been returned.

  Parameters:
    object -  Driver object handle, returned from DRV_SPI_Initialize

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     //  Returned from DRV_SPI_Initialize
    SYS_STATUS          status;

    DRV_SPI_Deinitialize ( object );

    status = DRV_SPI_Status( object );
    if( SYS_MODULE_UNINITIALIZED == status )
    {
        // Check again later if you need to know
        // when the driver is deinitialized.
    }
    </code>

  Remarks:
    Once the Initialize operation has been called, the De-initialize
    operation must be called before the Initialize operation can be called
    again.

    This function will NEVER block waiting for hardware. If the operation
    requires time to allow the hardware to complete, this will be reported
    by the DRV_SPI_Status operation. The system has to use DRV_SPI_Status
    to find out when the module is in the ready state.
*/

void DRV_SPI_Deinitialize ( SYS_MODULE_OBJ object );


/**************************************************************************
  Function:
       SYS_STATUS DRV_SPI_Status ( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the SPI driver module.
    <p><b>Implementation:</b> Static/Dynamic</p>

  Description:
    This function provides the current status of the SPI driver module.

  Precondition:
    The DRV_SPI_Initialize function must have been called before calling
    this function.

  Parameters:
    object -  Driver object handle, returned from DRV_SPI_Initialize

  Returns:
    - SYS_STATUS_READY - Indicates that the driver is busy with a previous
    system level operation and cannot start another

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_SPI_Initialize
    SYS_STATUS          status;

    status = DRV_SPI_Status( object );
    if( SYS_STATUS_READY != status )
    {
        // Handle error
    }
    </code>

  Remarks:
    Any value greater than SYS_STATUS_READY is also a normal running state
    in which the driver is ready to accept new operations.

    SYS_MODULE_UNINITIALIZED - Indicates that the driver has been
    deinitialized

    This value is less than SYS_STATUS_ERROR.

    This function can be used to determine when any of the driver's module
    level operations has completed.

    If the status operation returns SYS_STATUS_BUSY, the previous operation
    has not yet completed. Once the status operation returns
    SYS_STATUS_READY, any previous operations have completed.

    The value of SYS_STATUS_ERROR is negative (-1). Any value less than
    that is also an error state.

    This function will NEVER block waiting for hardware.

    If the Status operation returns an error value, the error may be
    cleared by calling the reinitialize operation. If that fails, the
    deinitialize operation will need to be called, followed by the
    initialize operation to return to normal operations.
*/

SYS_STATUS DRV_SPI_Status ( SYS_MODULE_OBJ object );


// *****************************************************************************
/* Function:
    void DRV_SPI_Tasks ( SYS_MODULE_OBJ object );

  Summary:
    Maintains the driver's state machine and implements its ISR.
    <p><b>Implementation:</b> Static/Dynamic</p>

  Description:
	This routine is used to maintain the driver's internal state
	machine and implement its transmit ISR for interrupt-driven implementations.
	In polling mode, this function should be called from the SYS_Tasks()
	function. In interrupt mode, this function should be called in the transmit
	interrupt service routine of the USART that is associated with this USART
	driver hardware instance.


  Precondition:
    The DRV_SPI_Initialize routine must have been called for the specified
    SPI driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_SPI_Initialize)

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_SPI_Initialize

    while( true )
    {
        DRV_SPI_Tasks ( object );

        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application.  It is
    called by the system's Tasks routine (SYS_Tasks) or by the appropriate raw
    ISR.

    This function may execute in an ISR context and will never block or access any
    resources that may cause it to block.
*/

void DRV_SPI_Tasks ( SYS_MODULE_OBJ object );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client Level
// *****************************************************************************
// *****************************************************************************

/**************************************************************************
  Function:
       DRV_HANDLE DRV_SPI_Open ( const SYS_MODULE_INDEX drvIndex,
                                const DRV_IO_INTENT    ioIntent )

  Summary:
    Opens the specified SPI driver instance and returns a handle to it.
    <p><b>Implementation:</b> Static/Dynamic</p>
	
  Description:
    This routine opens the specified SPI driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent
    parameter defines how the client interacts with this driver instance.

    If ioIntent is DRV_IO_INTENT_READ, the client will only be read from
    the driver. If ioIntent is DRV_IO_INTENT_WRITE, the client will only be
    able to write to the driver. If the ioIntent in
    DRV_IO_INTENT_READWRITE, the client will be able to do both, read and
    write.

    Specifying a DRV_IO_INTENT_EXCLUSIVE will cause the driver to provide
    exclusive access to this client. The driver cannot be opened by any
    other client.
	
  Precondition:
    The DRV_SPI_Initialize function must have been called before calling
    this function.
	
  Parameters:
    drvIndex -  Index of the driver initialized with DRV_SPI_Initialize().
                Please note this is not the SPI ID.
    ioIntent -    Zero or more of the values from the enumeration
                DRV_IO_INTENT ORed together to indicate the intended use of
                the driver
				
  Returns:
    If successful, the routine returns a valid open-instance handle (a
    number identifying both the caller and the module instance).

    If an error occurs, the return value is DRV_HANDLE_INVALID. An error
    can occur when the following is true:
      * if the number of client objects allocated via
        DRV_SPI_INSTANCES_NUMBER is insufficient
      * if the client is trying to open the driver but driver has been
        opened exclusively by another client
      * if the driver hardware instance being opened is not initialized or
        is invalid
		
  Example:
    <code>
    DRV_HANDLE  handle;

    handle = DRV_SPI_Open( DRV_SPI_INDEX_0, DRV_IO_INTENT_EXCLUSIVE );

    if( DRV_HANDLE_INVALID == handle )
    {
        // Unable to open the driver
    }
    </code>
	
  Remarks:
    The handle returned is valid until the DRV_SPI_Close routine is
    called. This routine will NEVER block waiting for hardware. If the
    requested intent flags are not supported, the routine will return
    DRV_HANDLE_INVALID. This function is thread safe in a RTOS application.
    It should not be called in an ISR.
*/

DRV_HANDLE DRV_SPI_Open ( const SYS_MODULE_INDEX drvIndex,
                         const DRV_IO_INTENT    ioIntent );

/**************************************************************************
  Function:
       int32_t DRV_SPI_ClientConfigure ( DRV_HANDLE handle,
                                const DRV_SPI_CLIENT_DATA * cfgData  )

  Summary:
    Configures a SPI client with specific data.
    <p><b>Implementation:</b> Static/Dynamic</p>
	
  Description:
    This routine takes a DRV_SPI_CLIENT_DATA structure and sets client specific options.
    Whenever a new SPI job is started these values will be used.  Passing in NULL will 
    reset the client back to configuration parameters passed to driver initialization. 
    A zero in any of the structure elements will reset that specific configuration back
    to the driver default.
  
  Precondition:
    The DRV_SPI_Open function must have been called before calling
    this function.
	
  Parameters:
    handle -  handle of the client returned by DRV_SPI_Open.
    cfgData -    Client-specific configuration data.
	
  Returns:
    - If successful - the routing will return greater than or equal to zero
    - If an error occurs - the return value is negative
*/
int32_t DRV_SPI_ClientConfigure ( DRV_HANDLE handle,
                                const DRV_SPI_CLIENT_DATA * cfgData  );

// *****************************************************************************
/* Function:
    void DRV_SPI_Close ( DRV_HANDLE handle )

  Summary:
    Closes an opened instance of the SPI driver.
    <p><b>Implementation:</b> Static/Dynamic</p>

  Description:
    This function closes an opened instance of the SPI driver, invalidating the
    handle.

  Precondition:
    The DRV_SPI_Initialize routine must have been called for the specified
    SPI driver instance.

    DRV_SPI_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None.

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_SPI_Open

    DRV_SPI_Close ( handle );
    </code>

  Remarks:
	After calling this routine, the handle passed in "handle" must not be used
    with any of the remaining driver routines.  A new handle must be obtained by
    calling DRV_SPI_Open before the caller may use the driver again.  This
    function is thread safe in a RTOS application.

    Note:
    Usually there is no need for the driver client to verify that the Close
    operation has completed.
*/

void DRV_SPI_Close ( DRV_HANDLE handle );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client level Read & Write APIs
// *****************************************************************************
// *****************************************************************************
/* These are non-blocking APIs. It doesn't wait until the operation gets
   finished. The actual operation will happen it the task routine. The status of
   this operation can be monitored using DRV_SPI_BufferStatus function. In
   polling mode, User must ensure that the code gets time to execute the task
   routine. */

/*******************************************************************************
  Function:
 	DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddRead ( DRV_HANDLE handle, void *rxBuffer,
                                       size_t size, DRV_SPI_BUFFER_EVENT_HANDLER completeCB,
                                              void * context )

  Summary:
    Registers a buffer for a read operation. Actual transfer will happen in
    the Task function.
    <p><b>Implementation:</b> Static/Dynamic</p>

  Description:
    Registers a buffer for a read operation. Actual transfer will happen in
    the Task function. The status of this operation can be monitored using
    DRV_SPI_BufferStatus function. A optional callback can also be
    provided that will be called when the operation is complete.

  Precondition:
    The DRV_SPI_Initialize routine must have been called for the specified
    SPI driver instance.

    DRV_SPI_Open must have been called to obtain a valid opened device
    handle.

    DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have been specified
    in the DRV_SPI_Open call.

  Parameters:
    handle -    A valid open-instance handle, returned from the driver's
                open routine

  	rxBuffer -  The buffer to which the data should be written to.

  	size -      Number of bytes to be read from the SPI bus.

    completeCB - Pointer to a function to be called when this queued operation is complete.

    context - unused by the driver but this is passed to the callback when it is called.

  Returns:
    If the buffer add request is successful, a valid buffer handle is returned.
    If request is not queued up, DRV_SPI_BUFFER_HANDLE_INVALID is returned.

  Example:
    <code>
    DRV_HANDLE      handle;    // Returned from DRV_SPI_Open
    char   myBuffer[MY_BUFFER_SIZE], state = 0;
	DRV_SPI_BUFFER_HANDLE bufferHandle;

    switch ( state )
    {
		case 0:
			bufferHandle = DRV_SPI_BufferAddRead( handle, myBuffer, 10, NULL, NULL );
            if(bufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID )
            {
                state++;
            }
			break;
		case 1:
			if( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus( bufferHandle ) )
			{
				state++;
				// All transmitter data has been sent successfully.
			}
			break;
	}
    </code>

  Remarks:
    This API will be deprecated soon, so avoid using it. Use "DRV_SPI_BufferAddRead2"
    instead of it.

 */

DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddRead ( DRV_HANDLE handle,
                                              void *rxBuffer,
                                              size_t size,
                                              DRV_SPI_BUFFER_EVENT_HANDLER completeCB,
                                              void * context);


/*******************************************************************************
  Function:
 	DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddWrite ( DRV_HANDLE handle, void *txBuffer,
                                       size_t size, DRV_SPI_BUFFER_EVENT_HANDLER completeCB,
                                              void * context )

  Summary:
    Registers a buffer for a write operation. Actual transfer will happen
    in the Task function.
    <p><b>Implementation:</b> Static/Dynamic</p>

  Description:
    Registers a buffer for a write operation. Actual transfer will happen
    in the Task function. The status of this operation can be monitored
    using DRV_SPI_BufferStatus function.  A optional callback can also be
    provided that will be called when the operation is complete.

  Precondition:
    The DRV_SPI_Initialize routine must have been called for the specified
    SPI driver instance.

    DRV_SPI_Open must have been called to obtain a valid opened device
    handle.

    DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have been specified
    in the DRV_SPI_Open call.

  Parameters:
    handle -    A valid open-instance handle, returned from the driver's
                open routine
    txBuffer -  The buffer which hold the data.
    size -      Number of bytes to be written to the SPI bus.
    completeCB - Pointer to a function to be called when this queued operation is complete
    context - unused by the driver but this is passed to the callback when it is called

  Returns:
    If the buffer add request is successful, a valid buffer handle is returned.
    If request is not queued up, DRV_SPI_BUFFER_HANDLE_INVALID is returned.

  Example:
    <code>
    DRV_HANDLE      handle;    // Returned from DRV_SPI_Open
	char   myBuffer[MY_BUFFER_SIZE], state = 0;
	DRV_SPI_BUFFER_HANDLE bufferHandle;

	switch ( state )
	{
		case 0:
			bufferHandle = DRV_SPI_BufferAddWrite( handle, myBuffer, 10, NULL, NULL );
            if(bufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID )
            {
                state++;
            }
			break;
		case 1:
			if( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus( bufferHandle ) )
			{
				state++;
				// All transmitter data has been sent successfully.
			}
			break;
	}

    </code>

  Remarks:
    This API will be deprecated soon, so avoid using it. Use "DRV_SPI_BufferAddWrite2"
    instead of it.
*/

DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddWrite ( DRV_HANDLE handle, void *txBuffer,
                                    size_t size,
                                              DRV_SPI_BUFFER_EVENT_HANDLER completeCB,
                                              void * context );


/*******************************************************************************
  Function:
       DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddWriteRead( DRV_HANDLE handle,
       					void *txBuffer, void *rxBuffer, size_t size,  )

  Summary:
    Registers a buffer for a read and write operation. Actual transfer will
    happen in the Task function.
	<p><b>Implementation:</b> Static/Dynamic</p>

  Description:
    Registers a buffer for a read and write operation. Actual transfer will
    happen in the Task function. The status of this operation can be
    monitored using DRV_SPI_BufferStatus function.  A optional callback can also be
    provided that will be called when the operation is complete.

  Precondition:
    The DRV_SPI_Initialize routine must have been called for the specified
    SPI driver instance.

    DRV_SPI_Open must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle -    A valid open-instance handle, returned from the driver's
                open routine
    txBuffer -  The buffer which hold the data.
    txSize   -  Number of bytes to be written to the SPI bus.
    rxBuffer -  The buffer to which the data should be written to.
    rxSize -    Number of bytes to be read from the SPI bus
    completeCB - Pointer to a function to be called when this queued operation is complete
    context - unused by the driver but this is passed to the callback when it is called
 
  Returns:
    If the buffer add request is successful, a valid buffer handle is returned.
    If request is not queued up, DRV_SPI_BUFFER_HANDLE_INVALID is returned.

  Example:
	<code>
	DRV_HANDLE      handle;    // Returned from DRV_SPI_Open
	char   myReadBuffer[MY_BUFFER_SIZE], myWriteBuffer[MY_BUFFER_SIZE], state = 0;
	DRV_SPI_BUFFER_HANDLE bufferHandle;

	switch ( state )
	{
		case 0:
			bufferHandle = DRV_SPI_BufferAddWriteRead( handle, myWriteBuffer, 10, myReadBuffer, 10, NULL, NULL );
            if(bufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID )
            {
                state++;
            }
			break;
		case 1:
			if( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus( bufferHandle ) )
			{
				state++;
				// All transmitter data has been sent successfully.
			}
			break;
	}

	</code>

  Remarks:
    This API will be deprecated soon, so avoid using it. Use "DRV_SPI_BufferAddWriteRead2"
    instead of it.

*/

DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddWriteRead ( DRV_HANDLE handle, void *txBuffer, size_t txSize,
                                                   void *rxBuffer, size_t rxSize,
                                                   DRV_SPI_BUFFER_EVENT_HANDLER completeCB,
                                                   void * context );


/*******************************************************************************
  Function:
 	DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddRead2 ( DRV_HANDLE handle, void *rxBuffer,
                                       size_t size, DRV_SPI_BUFFER_EVENT_HANDLER completeCB,
                                              void * context, DRV_SPI_BUFFER_HANDLE * jobHandle  )

  Summary:
    Registers a buffer for a read operation. Actual transfer will happen in
    the Task function.
    <p><b>Implementation:</b> Static/Dynamic</p>

  Description:
    Registers a buffer for a read operation. Actual transfer will happen in
    the Task function. The status of this operation can be monitored using
    DRV_SPI_BufferStatus function. A optional callback can also be
    provided that will be called when the operation is complete.

  Precondition:
    The DRV_SPI_Initialize routine must have been called for the specified
    SPI driver instance.

    DRV_SPI_Open must have been called to obtain a valid opened device
    handle.

    DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have been specified
    in the DRV_SPI_Open call.

  Parameters:
    handle -    A valid open-instance handle, returned from the driver's
                open routine

  	rxBuffer -  The buffer to which the data should be written to.

  	size -      Number of bytes to be read from the SPI bus.

    completeCB - Pointer to a function to be called when this queued operation is complete

    context - unused by the driver but this is passed to the callback when it is called
    jobHandle - pointer to the buffer handle, this will be set before the function returns and can be used in the ISR callback.

  Returns:
    If the buffer add request is successful, a valid buffer handle is returned.
    If request is not queued up, DRV_SPI_BUFFER_HANDLE_INVALID is returned.

  Example:
    <code>
    DRV_HANDLE      handle;    // Returned from DRV_SPI_Open
    char   myBuffer[MY_BUFFER_SIZE], state = 0;
	DRV_SPI_BUFFER_HANDLE bufferHandle, bufferHandle2;

    switch ( state )
    {
		case 0:
			bufferHandle = DRV_SPI_BufferAddRead2( handle, myBuffer, 10, NULL, NULL, bufferHandle2 );
            if(bufferHandle2 != DRV_SPI_BUFFER_HANDLE_INVALID )
            {
                state++;
            }
			break;
		case 1:
			if( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus( bufferHandle2 ) )
			{
				state++;
				// All transmitter data has been sent successfully.
			}
			break;
	}
    </code>

  Remarks:
    None.

 */

DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddRead2 ( DRV_HANDLE handle,
                                              void *rxBuffer,
                                              size_t size,
                                              DRV_SPI_BUFFER_EVENT_HANDLER completeCB,
                                              void * context, DRV_SPI_BUFFER_HANDLE * jobHandle);


/*******************************************************************************
  Function:
 	DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddWrite2 ( DRV_HANDLE handle, void *txBuffer,
                                       size_t size, DRV_SPI_BUFFER_EVENT_HANDLER completeCB,
                                              void * context, DRV_SPI_BUFFER_HANDLE * jobHandle )

  Summary:
    Registers a buffer for a write operation. Actual transfer will happen
    in the Task function.
    <p><b>Implementation:</b> Static/Dynamic</p>

  Description:
    Registers a buffer for a write operation. Actual transfer will happen
    in the Task function. The status of this operation can be monitored
    using DRV_SPI_BufferStatus function.  A optional callback can also be
    provided that will be called when the operation is complete.

  Precondition:
    The DRV_SPI_Initialize routine must have been called for the specified
    SPI driver instance.

    DRV_SPI_Open must have been called to obtain a valid opened device
    handle.

    DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have been specified
    in the DRV_SPI_Open call.

  Parameters:
    handle -    A valid open-instance handle, returned from the driver's
                open routine
    txBuffer -  The buffer which hold the data.
    size -      Number of bytes to be written to the SPI bus.
    completeCB - Pointer to a function to be called when this queued operation is complete
    context - unused by the driver but this is passed to the callback when it is called
    jobHandle - pointer to the buffer handle, this will be set before the function returns and can be used in the ISR callback.

  Returns:
    If the buffer add request is successful, a valid buffer handle is returned.
    If request is not queued up, DRV_SPI_BUFFER_HANDLE_INVALID is returned.

  Example:
    <code>
    DRV_HANDLE      handle;    // Returned from DRV_SPI_Open
	char   myBuffer[MY_BUFFER_SIZE], state = 0;
	DRV_SPI_BUFFER_HANDLE bufferHandle, bufferHandle2;

	switch ( state )
	{
		case 0:
			bufferHandle = DRV_SPI_BufferAddWrite2( handle, myBuffer, 10, NULL, NULL, bufferHandle2 );
            if(bufferHandle2 != DRV_SPI_BUFFER_HANDLE_INVALID )
            {
                state++;
            }
			break;
		case 1:
			if( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus( bufferHandle2 ) )
			{
				state++;
				// All transmitter data has been sent successfully.
			}
			break;
	}

    </code>

  Remarks:
    None.
*/

DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddWrite2 ( DRV_HANDLE handle, void *txBuffer,
                                    size_t size,
                                              DRV_SPI_BUFFER_EVENT_HANDLER completeCB,
                                              void * context, DRV_SPI_BUFFER_HANDLE * jobHandle );


/*******************************************************************************
  Function:
       DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddWriteRead2( DRV_HANDLE handle,
       					void *txBuffer, void *rxBuffer, size_t size,
                                                   DRV_SPI_BUFFER_EVENT_HANDLER completeCB,
                                                   void * context,  DRV_SPI_BUFFER_HANDLE * jobHandle  )

  Summary:
    Registers a buffer for a read and write operation. Actual transfer will
    happen in the Task function.
	<p><b>Implementation:</b> Static/Dynamic</p>

  Description:
    Registers a buffer for a read and write operation. Actual transfer will
    happen in the Task function. The status of this operation can be
    monitored using DRV_SPI_BufferStatus function.  A optional callback can also be
    provided that will be called when the operation is complete.

  Precondition:
    The DRV_SPI_Initialize routine must have been called for the specified
    SPI driver instance.

    DRV_SPI_Open must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle -    A valid open-instance handle, returned from the driver's
                open routine
    txBuffer -  The buffer which hold the data.
    txSize   -  Number of bytes to be written to the SPI bus.
    rxBuffer -  The buffer to which the data should be written to.
    rxSize -    Number of bytes to be read from the SPI bus
    completeCB - Pointer to a function to be called when this queued operation is complete
    context - unused by the driver but this is passed to the callback when it is called
    jobHandle - pointer to the buffer handle, this will be set before the function returns and can be used in the ISR callback.
  
   Returns:
    If the buffer add request is successful, a valid buffer handle is returned.
    If request is not queued up, DRV_SPI_BUFFER_HANDLE_INVALID is returned.

  Example:
	<code>
	DRV_HANDLE      handle;    // Returned from DRV_SPI_Open
	char   myReadBuffer[MY_BUFFER_SIZE], myWriteBuffer[MY_BUFFER_SIZE], state = 0;
	DRV_SPI_BUFFER_HANDLE bufferHandle, bufferHandle2;

	switch ( state )
	{
		case 0:
			bufferHandle = DRV_SPI_BufferAddWriteRead2( handle, myWriteBuffer, 10, myReadBuffer, 10, NULL, NULL, bufferHandle2 );
            if(bufferHandle2 != DRV_SPI_BUFFER_HANDLE_INVALID )
            {
                state++;
            }
			break;
		case 1:
			if( DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus( bufferHandle2 ) )
			{
				state++;
				// All transmitter data has been sent successfully.
			}
			break;
	}

	</code>

  Remarks:
    None.
*/

DRV_SPI_BUFFER_HANDLE DRV_SPI_BufferAddWriteRead2 ( DRV_HANDLE handle, void *txBuffer, size_t txSize,
                                                   void *rxBuffer, size_t rxSize,
                                                   DRV_SPI_BUFFER_EVENT_HANDLER completeCB,
                                                   void * context,  DRV_SPI_BUFFER_HANDLE * jobHandle);



// *****************************************************************************
/* Function:
    DRV_SPI_BUFFER_EVENT DRV_SPI_BufferStatus ( DRV_SPI_BUFFER_HANDLE bufferHandle )

  Summary:
    Returns the transmitter and receiver transfer status.
    <p><b>Implementation:</b> Static/Dynamic</p>

  Description:
    This returns the transmitter and receiver transfer status.

  Precondition:
    The DRV_SPI_Initialize routine must have been called for the specified
    SPI driver instance.

    DRV_SPI_Open must have been called to obtain a valid opened device handle.

    DRV_SPI_BufferAdd<Transfer> must have been called to obtain
    the buffer handle associated with that transfer.

  Parameters:
    bufferHandle    - A valid buffer handle, returned from the driver's
                      data transfer routine

  Returns:
    A DRV_SPI_BUFFER_STATUS value describing the current status of the
    transfer.

  Example:
    <code>
    // Buffer handle returned from the data transfer function
    DRV_SPI_BUFFER_HANDLE bufferHandle;

    if( DRV_SPI_BUFFER_STATUS_SUCCESS & DRV_SPI_BufferStatus( bufferHandle ) )
    {
        // All transmitter data has been sent.
    }
    </code>

  Remarks:
    The returned status may contain a value with more than one of the bits
    specified in the DRV_SPI_BUFFER_STATUS enumeration set.  The caller
    should perform an AND with the bit of interest and verify if the
    result is non-zero (as shown in the example) to verify the desired status
    bit.
*/

DRV_SPI_BUFFER_EVENT DRV_SPI_BufferStatus ( DRV_SPI_BUFFER_HANDLE bufferHandle );


// *****************************************************************************
/* Function:
    bool DRV_SPIn_ReceiverBufferIsFull(void)

  Summary:
    Returns the receive buffer status. 'n' represents the instance of the 
	SPI driver used.
	<p><b>Implementation:</b> Static</p>

  Description:
    This function returns the receive buffer status (full/empty).

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Receive Buffer Status
    - 1 - Full
    - 0 - Empty

  Example:
    <code>
    bool rxBufStat;
    // Using instance 1 of SPI driver, that is n = 1
    rxBufStat = DRV_SPI1_ReceiverBufferIsFull();
	
	if (rxBufStat)
	{
	...
	}
    </code>

  Remarks:
    None.
*/

 bool DRV_SPIn_ReceiverBufferIsFull(void);

// *****************************************************************************
/* Function:
    bool DRV_SPIn_TransmitterBufferIsFull(void)

  Summary:
    Returns the transmit buffer status. 'n' represents the instance of the 
	SPI driver used.
	<p><b>Implementation:</b> Static</p>

  Description:
    This function returns the transmit buffer status (full/empty).

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Transmit Buffer Status
    - 1 - Full
    - 0 - Empty

  Example:
    <code>
    bool txBufStat;
    // Using instance 1 of SPI driver, that is n = 1
    txBufStat = DRV_SPI1_TransmitterBufferIsFull();
	
	if (txBufStat)
	{
	...
	}
    </code>

  Remarks:
    None.
*/
 
 bool DRV_SPIn_TransmitterBufferIsFull(void);
 

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END


#endif // #ifndef _DRV_SPI_H

/*******************************************************************************
 End of File
*/

