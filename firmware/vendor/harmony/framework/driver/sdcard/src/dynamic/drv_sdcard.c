/*******************************************************************************
  SD CARD Device Driver Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sdcard_dynamic.c

  Summary:
    SD CARD Device Driver Dynamic Implementation

  Description:
    The SD CARD device driver provides a simple interface to manage the SD CARD
    modules on Microchip microcontrollers. This file Implements the core
    interface routines for the SD CARD driver.

    While building the driver from source, ALWAYS use this file in the build.
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

#include <string.h>
#include "driver/sdcard/src/drv_sdcard_local.h"
#include "system/ports/sys_ports.h"
#include "system/clk/sys_clk.h"
#include "osal/osal.h"

// *****************************************************************************
// *****************************************************************************
// Section: File Scope Variables
// *****************************************************************************
// *****************************************************************************

uint8_t gDrvSDCARDBuffer [DRV_SDCARD_INSTANCES_NUMBER][16] __attribute__((coherent, aligned(16)));
uint8_t gDrvSDCARDClkPulseData [DRV_SDCARD_INSTANCES_NUMBER][10] __attribute__((coherent, aligned(16)));
uint8_t gDrvSDCARDCsdData [DRV_SDCARD_INSTANCES_NUMBER][19] __attribute__((coherent, aligned(16)));

uint8_t gDrvSDCARDInitCount = 0;
OSAL_MUTEX_DECLARE(gDrvSDCARDClientMutex);

/************************************************
 * This token is incremented for every request
 * added to the queue and is used to generate
 * a different buffer handle for every request.
 ***********************************************/
uint16_t gDrvSDCARDBufferToken = 0;

// *****************************************************************************
/* SD Card command table

  Summary:
    Defines the a command table for SD card.

  Description:
    This data structure makes a command table for the SD Card with the command,
    its CRC, expected response and a flag indicating whether the driver expects
    more data or not. This makes the SD card commands easier to handle.

  Remarks:
    The actual response for the command 'CMD_SD_SEND_OP_COND'is R3, but it has
    same number of bytes as R7. So R7 is used in the table.
*/

const DRV_SDCARD_CMD_OBJ gDrvSDCARDCmdTable[] =
{
    /* Command                             crc     response       more data */
    {CMD_VALUE_GO_IDLE_STATE,              0x95,   RESPONSE_R1,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_SEND_OP_COND,               0xF9,   RESPONSE_R1,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_SEND_IF_COND,               0x87,   RESPONSE_R7,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_SEND_CSD,                   0xAF,   RESPONSE_R1,         DRV_SDCARD_GET_MOREDATA},
    {CMD_VALUE_SEND_CID,                   0x1B,   RESPONSE_R1,         DRV_SDCARD_GET_MOREDATA},
    {CMD_VALUE_STOP_TRANSMISSION,          0xC3,   RESPONSE_R1,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_SEND_STATUS,                0xAF,   RESPONSE_R2,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_SET_BLOCKLEN,               0xFF,   RESPONSE_R1,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_READ_SINGLE_BLOCK,          0xFF,   RESPONSE_R1,         DRV_SDCARD_GET_MOREDATA},
    {CMD_VALUE_READ_MULTI_BLOCK,           0xFF,   RESPONSE_R1,         DRV_SDCARD_GET_MOREDATA},
    {CMD_VALUE_WRITE_SINGLE_BLOCK,         0xFF,   RESPONSE_R1,         DRV_SDCARD_GET_MOREDATA},
    {CMD_VALUE_WRITE_MULTI_BLOCK,          0xFF,   RESPONSE_R1,         DRV_SDCARD_GET_MOREDATA},
    {CMD_VALUE_TAG_SECTOR_START,           0xFF,   RESPONSE_R1,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_TAG_SECTOR_END,             0xFF,   RESPONSE_R1,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_ERASE,                      0xDF,   RESPONSE_R1b,        DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_APP_CMD,                    0x73,   RESPONSE_R1,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_READ_OCR,                   0x25,   RESPONSE_R7,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_CRC_ON_OFF,                 0x25,   RESPONSE_R1,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_SD_SEND_OP_COND,            0xFF,   RESPONSE_R7,         DRV_SDCARD_GET_NODATA},
    {CMD_VALUE_SET_WR_BLK_ERASE_COUNT,     0xFF,   RESPONSE_R1,         DRV_SDCARD_GET_NODATA}
};

// *****************************************************************************
/* SD card media functions.

  Summary:
    These functions are used by the 'media manager' to access the SD card.

  Description:
	These functions are used by the 'media manager' to access the SD card. The
	call will be by using a function pointer. So SD card driver must attach these
	functions to the media manager on initialize.

  Remarks:
    None.
*/

const SYS_FS_MEDIA_FUNCTIONS sdcardMediaFunctions =
{
    .mediaStatusGet     = DRV_SDCARD_IsAttached,
    .mediaGeometryGet   = DRV_SDCARD_GeometryGet,
    .sectorRead         = DRV_SDCARD_Read,
    .sectorWrite        = DRV_SDCARD_Write,
    .eventHandlerset    = DRV_SDCARD_EventHandlerSet,
    .commandStatusGet   = (void *)DRV_SDCARD_CommandStatus,
    .open               = DRV_SDCARD_Open,
    .close              = DRV_SDCARD_Close,
    .tasks              = DRV_SDCARD_Tasks
};

// *****************************************************************************
/* Driver Hardware instance objects.

  Summary:
    Defines the hardware instances objects that are available on the part

  Description:
    This data type defines the hardware instance objects that are available on
    the part, so as to capture the hardware state of the instance.

  Remarks:
    Not all modes are available on all micro-controllers.
*/

DRV_SDCARD_OBJ              gDrvSDCARDObj[DRV_SDCARD_INSTANCES_NUMBER];

// *****************************************************************************
/* Driver Client instance objects.

  Summary:
    Defines the Client instances objects that are available on the part

  Description:
    This data type defines the Client instance objects that are available on
    the part, so as to capture the Client state of the instance.

  Remarks:
    None
*/

DRV_SDCARD_CLIENT_OBJ       gDrvSDCARDClientObj[DRV_SDCARD_CLIENTS_NUMBER * DRV_SDCARD_INSTANCES_NUMBER];


// *****************************************************************************
/* Driver buffer objects.

  Summary:
    Transfer objects for the driver queue.

  Description:
    This instance of the structure is used as transfer objects for the driver
    queue.

  Remarks:
    None
*/

DRV_SDCARD_XFER_OBJECT      gDrvSDCARDTransferObj[DRV_SDCARD_INSTANCES_NUMBER][DRV_SDCARD_QUEUE_POOL_SIZE];


// *****************************************************************************
/* Driver queue object

  Summary:
    Variables to handle the queue.

  Description:
    This instance of the structure holds the variables to handle the queue.

  Remarks:
    None
*/

DRV_SDCARD_QUEUE_OBJECT   	gDrvSDCARDQueueObj[DRV_SDCARD_INSTANCES_NUMBER];

// *****************************************************************************
/* Macro: _DRV_SDCARD_CLIENT_OBJ(obj,mem)

  Summary:
    Returns the appropriate client member

  Description:
    Return the indexed dynamic object.
*/

#define _DRV_SDCARD_CLIENT_OBJ(obj,mem)    gDrvSDCARDClientObj[obj].mem


// *****************************************************************************
/* Macro: _DRV_SDCARD_CLIENT_OBJ_GET(obj)

  Summary:
    Returns the appropriate client instance

  Description:
    Return the indexed dynamic object.
*/

#define _DRV_SDCARD_CLIENT_OBJ_GET(obj)    &gDrvSDCARDClientObj[obj]


// *****************************************************************************
/* Macro: _DRV_SDCARD_INDEX_GET(drvIndex)

  Summary:
    Returns the appropriate driver id for the configuration

  Description:
    Return the dynamic index passed into the macro.

*/

#define _DRV_SDCARD_INDEX_GET(drvIndex)                            (drvIndex)


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:

   void _DRV_SDCARD_SpiBufferEventHandler 
   (
       DRV_SPI_BUFFER_EVENT event,
       DRV_SPI_BUFFER_HANDLE bufferHandle, 
       void * context
   )

  Summary:
    SPI Buffer Event Handler

  Description:
    SPI Buffer event handler. This handler controls the selection and 
    de-selection of the Chip Select line of the SD Card.

  Remarks:
    None
*/

void _DRV_SDCARD_SpiBufferEventHandler 
(
    DRV_SPI_BUFFER_EVENT event,
    DRV_SPI_BUFFER_HANDLE bufferHandle, 
    void * context
)
{
    DRV_SDCARD_OBJ  *dObj;
    
    if (context == NULL)
    {
        return;
    }

    dObj = (DRV_SDCARD_OBJ *)context;

    if (bufferHandle != dObj->spiBufferHandle)
    {
        return;
    }

    switch (event)
    {
        case DRV_SPI_BUFFER_EVENT_PROCESSING:
            {
                /* Select the chip */
                _DRV_SDCARD_CHIP_SELECT (dObj->chipSelectPort, dObj->chipSelectBitPosition);
                break;
            }

        case DRV_SPI_BUFFER_EVENT_COMPLETE:
        case DRV_SPI_BUFFER_EVENT_ERROR:
        default:
            {

                /* De select the chip */
                _DRV_SDCARD_CHIP_DESELECT (dObj->chipSelectPort, dObj->chipSelectBitPosition);
                break;
            }
    }
}

//******************************************************************************
/* Function:
   static void _DRV_SDCARD_SetupHardware 
   (
       DRV_SDCARD_INIT *sdcardInit
   )

  Summary:
    Configure the ports/pins used by the SDCARD driver.

  Description:
    This function setups the SPI Chip Select, SD Card write protect and
    Card Detect lines.

  Remarks:
    None
*/

static void _DRV_SDCARD_SetupHardware 
(
    DRV_SDCARD_INIT *sdcardInit
)
{
    /* Set the Card 'write protection status' pin */
    SYS_PORTS_PinDirectionSelect (PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT,
            sdcardInit->writeProtectPort,
            sdcardInit->writeProtectBitPosition);

    /* Set the Card 'Chip Select' pin */
    SYS_PORTS_PinDirectionSelect (PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT,
            sdcardInit->chipSelectPort,
            sdcardInit->chipSelectBitPosition);
}

//******************************************************************************
/* Function:
   static void _DRV_SDCARD_UpdateGeometry
   (
       DRV_SDCARD_OBJ *dObj
   )

  Summary:
    Update the SDCARD media geometry.

  Description:
    This function updates the SDCARD media geometry object.

  Remarks:
    None
*/
static void _DRV_SDCARD_UpdateGeometry
(
    DRV_SDCARD_OBJ *dObj
)
{
    uint8_t i = 0;

    /* Update the Media Geometry Table */
    for (i = 0; i <= GEOMETRY_TABLE_ERASE_ENTRY; i++) 
    {
        dObj->mediaGeometryTable[i].blockSize = 512;
        dObj->mediaGeometryTable[i].numBlocks = dObj->discCapacity;
    }

    /* Update the Media Geometry Main Structure */
    dObj->mediaGeometryObj.mediaProperty = (SYS_FS_MEDIA_READ_IS_BLOCKING | SYS_FS_MEDIA_WRITE_IS_BLOCKING),

    /* Number of read, write and erase entries in the table */
    dObj->mediaGeometryObj.numReadRegions = 1,
    dObj->mediaGeometryObj.numWriteRegions = 1,
    dObj->mediaGeometryObj.numEraseRegions = 1,
    dObj->mediaGeometryObj.geometryTable = (SYS_FS_MEDIA_REGION_GEOMETRY *)&dObj->mediaGeometryTable;
}

//******************************************************************************
/* Function:
   
   static void _DRV_SDCARD_RemoveQueuedRequests 
   (
       DRV_SDCARD_OBJ *dObj
   )

  Summary:
    Remove the queued objects on an SD card detach.

  Description:
    This function removes the queued objects when the SD card has been detached.

  Remarks:
    None
*/

static void _DRV_SDCARD_RemoveQueuedRequests 
(
    DRV_SDCARD_OBJ *dObj
)
{
    uint8_t i = 0;

    DRV_SDCARD_QUEUE_OBJECT *qObj;
    DRV_SDCARD_CLIENT_OBJ   *clientObj;

    if (dObj->mediaState != SYS_FS_MEDIA_ATTACHED)
    {
        return;
    }

    /* The media was earlier attached */
    qObj = (DRV_SDCARD_QUEUE_OBJECT *)dObj->queueHandle;
    dObj->mediaState = SYS_FS_MEDIA_DETACHED;

    if (!qObj)
    {
        return;
    }

    /* Reset the indexes */
    qObj->startIndex = qObj->endIndex;

    /* The media was attached earlier. */
    for (i = 0; i < qObj->size; i++)
    {
        if (qObj->bufferPool[i].status == DRV_SDCARD_COMMAND_QUEUED)
        {
            qObj->bufferPool[i].status = DRV_SDCARD_COMMAND_ERROR_UNKNOWN;
            qObj->bufferPool[i].inUse = false;

            clientObj = (DRV_SDCARD_CLIENT_OBJ*)_DRV_SDCARD_CLIENT_OBJ_GET(qObj->bufferPool[i].hClient);
            if(clientObj->eventHandler != NULL)
            {
                /* Call the event handler */
                clientObj->eventHandler(DRV_SDCARD_EVENT_COMMAND_ERROR,
                        (DRV_SDCARD_COMMAND_HANDLE)qObj->bufferPool[i].commandHandle, clientObj->context);
            }
        }
    }
}

//******************************************************************************
/* Function:
   
   static void _DRV_SDCARD_CheckWriteProtectStatus 
   (
       DRV_SDCARD_OBJ *dObj
   )

  Summary:
    Check and update the SD Card write protection status.

  Description:
    This function checks and updates the SD card write protection status if
    the feature is enabled. Otherwise the write protections status is set to
    false.

  Remarks:
    None
*/

static void _DRV_SDCARD_CheckWriteProtectStatus 
(
    DRV_SDCARD_OBJ *dObj
)
{
    dObj->isWriteProtected = false;

    /* Check if the Write Protect check is enabled */
    if (_DRV_SDCARD_EnableWriteProtectCheck())
    {
        /* Read from the pin */
        dObj->isWriteProtected = _DRV_SDCARD_PORT_PIN_READ (dObj->writeProtectPort, dObj->writeProtectBitPosition);
    }
}

//******************************************************************************
/* Function:
   
   static DRV_SDCARD_CLIENT_OBJ * _DRV_SDCARD_ClientHandleValidate
   (
       DRV_HANDLE handle
   )

  Summary:
    Validate the client object

  Description:
    This function validates if the client object handle passed.

  Remarks:
    None
*/

static DRV_SDCARD_CLIENT_OBJ * _DRV_SDCARD_ClientHandleValidate
(
    DRV_HANDLE handle
)
{
    DRV_SDCARD_CLIENT_OBJ * clientObj;
    DRV_SDCARD_OBJ * dObj;

    /* Validate the handle */
    if ((handle < 0) || (handle > (DRV_SDCARD_CLIENTS_NUMBER * DRV_SDCARD_INSTANCES_NUMBER)))
    {
        return NULL;
    }

    /* See if the client has been opened */
    clientObj = _DRV_SDCARD_CLIENT_OBJ_GET(handle);
    if (!clientObj->inUse)
    {
        return NULL;
    }

    /* Check if the driver is ready for operation */
    dObj = (DRV_SDCARD_OBJ *)clientObj->driverObject;
    if (dObj->status != SYS_STATUS_READY)
    {
        return NULL;
    }

    return clientObj;
}

// *****************************************************************************
/* Function:

   static DRV_SDCARD_QUEUE_HANDLE _DRV_SDCARD_QueueInitialize 
   ( 
       const SYS_MODULE_INDEX drvIndex
   )

  Summary:
    Initializes the queue.

  Description:
    This function intializes the queue bufferpool pointer with that of the 
    transfer object address and resets the queue indexes.

  Parameters:
    drvIndex - Index of SD card driver opened.

  Returns:
    Handle to the initialized queue.

  Remarks:
    None
*/

static DRV_SDCARD_QUEUE_HANDLE _DRV_SDCARD_QueueInitialize 
( 
    const SYS_MODULE_INDEX drvIndex
)
{
    gDrvSDCARDQueueObj[drvIndex].bufferPool = &gDrvSDCARDTransferObj[drvIndex][0];
    gDrvSDCARDQueueObj[drvIndex].startIndex = 0;
    gDrvSDCARDQueueObj[drvIndex].endIndex = 0;
    gDrvSDCARDQueueObj[drvIndex].size = DRV_SDCARD_QUEUE_POOL_SIZE;

    return ((DRV_SDCARD_QUEUE_HANDLE)&gDrvSDCARDQueueObj[drvIndex].startIndex);
}

// *****************************************************************************
/* Function:
    static DRV_SDCARD_COMMAND_HANDLE _DRV_SDCARD_AddToQueue 
    ( 
        DRV_HANDLE handle,
        DRV_SDCARD_TRANSFER_TYPE readWrite,
        uint8_t *buffer,
        uint32_t blockStart,
        uint32_t nBlock
    )
  Summary:
    Adds a new xfer object to the queue.

  Description:
    Adds a new xfer object to the queue if the queue is not full and
    if the media is attached.

  Parameters:
    handle      - Driver Handle
    readWrite   - Indicates if operation is read or write
    buffer      - buffer to be used for the operation
    blockStart  - media start address
    nBlock      - number of blocks

  Returns:
    DRV_SDCARD_COMMAND_HANDLE_INVALID - On failure to add to the queue. This could be
                                        be due to either the queue being full or the
                                        media being detached.
    VALID Handle                      - Successfully added to the queue.

  Remarks:
    None.
*/

static DRV_SDCARD_COMMAND_HANDLE _DRV_SDCARD_AddToQueue 
(
    DRV_HANDLE handle,
    DRV_SDCARD_TRANSFER_TYPE readWrite,
    uint8_t *buffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    uint8_t                 index;
    DRV_SDCARD_QUEUE_OBJECT *qObj;
    DRV_SDCARD_CLIENT_OBJ   *clientObj = (DRV_SDCARD_CLIENT_OBJ*)_DRV_SDCARD_CLIENT_OBJ_GET(handle);
    DRV_SDCARD_OBJ          *dObj = (DRV_SDCARD_OBJ *)clientObj->driverObject;
    DRV_SDCARD_XFER_OBJECT  *xferObj;        

    qObj = (DRV_SDCARD_QUEUE_OBJECT*)&gDrvSDCARDQueueObj[clientObj->drvIndex];
    index = qObj->endIndex;

    if ((qObj->bufferPool[index].inUse == true) || (dObj->mediaState != SYS_FS_MEDIA_ATTACHED))
    {
        return DRV_SDCARD_COMMAND_HANDLE_INVALID;
    }

    xferObj = &qObj->bufferPool[index];

    xferObj->inUse         = true;
    xferObj->hClient       = handle;
    xferObj->commandHandle = _DRV_SDCARD_MAKE_HANDLE(gDrvSDCARDBufferToken, index);
    xferObj->buffer        = buffer;
    xferObj->readWrite     = readWrite;
    xferObj->sectorAddress = blockStart;
    xferObj->sectorCount   = nBlock;
    xferObj->status        = DRV_SDCARD_COMMAND_QUEUED;
    
    qObj->endIndex = (index + 1) % qObj->size;

    _DRV_SDCARD_UPDATE_BUF_TOKEN(gDrvSDCARDBufferToken);

    return xferObj->commandHandle;
}

// *****************************************************************************
/* Function:
    static DRV_SDCARD_XFER_OBJECT* _DRV_SDCARD_ReadFromQueue
    (
        DRV_SDCARD_QUEUE_HANDLE *handle
    );

  Summary:
    Read the next available object from the queue.

  Description:
    Read the next available object from the queue.

  Parameters:
    handle -    A queue handle.

  Returns:
    NULL                   - If the queue is empty.
    DRV_SDCARD_XFER_OBJECT - Pointer to the xfer object.

  Remarks:
    None.
*/

static DRV_SDCARD_XFER_OBJECT* _DRV_SDCARD_ReadFromQueue 
(
    DRV_SDCARD_QUEUE_HANDLE handle
)
{
    DRV_SDCARD_QUEUE_OBJECT *qObj = (DRV_SDCARD_QUEUE_OBJECT*)handle;
    DRV_SDCARD_XFER_OBJECT  *tObj;

    /* Check for queue empty */
    if (qObj->bufferPool[qObj->startIndex].inUse == false)
    {
        return NULL;
    }

    tObj = &qObj->bufferPool[qObj->startIndex];

    /* Move the queue pointer */
    qObj->startIndex = (qObj->startIndex + 1) % qObj->size;

    return tObj;
}

//******************************************************************************
/* Function:
    static void _DRV_SDCARD_CommandSend ( DRV_HANDLE handle, DRV_SDCARD_COMMANDS command,
                                uint32_t address )

  Summary:
    Sends command to the SD Card.

  Description:
    This routine sends a command to the SD card. The response will be updated in
    the driver instance object. The code is written in an event driven method.
    The user is required to call this API multiple times till the status becomes
    'complete'.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's
                   open routine.

    command     - Command to send.

    address     - If there is an address associated with the command. If there
                    is no address associated, then pass '0'.

  Returns:
    None

  Remarks:
    It is expected to call this routine continuously until we get the status as
    'execution successful'. The caller should not execute this function again
    after the execution of the command. It will cause to execute the same command
    again.
*/

static void _DRV_SDCARD_CommandSend
(
    SYS_MODULE_OBJ object,
    DRV_SDCARD_COMMANDS command,
    uint32_t address
)
{
    DRV_SDCARD_OBJ *dObj = (DRV_SDCARD_OBJ*)_DRV_SDCARD_INSTANCE_GET(object);
    uint8_t endianArray[4];
    DRV_SPI_BUFFER_EVENT spiRetVal;

    switch (dObj->cmdState)
    {
        case DRV_SDCARD_CMD_FRAME_PACKET:
            {
                dObj->cmdRespTmrFlag = false;

                /* SD card follows big-endian format */
                *((uint32_t*)endianArray) = *((uint32_t*)&address);

                /* Form the packet */
                dObj->pCmdResp[0] = (gDrvSDCARDCmdTable[command].commandCode | DRV_SDCARD_TRANSMIT_SET);
                dObj->pCmdResp[1] = endianArray[3];
                dObj->pCmdResp[2] = endianArray[2];
                dObj->pCmdResp[3] = endianArray[1];
                dObj->pCmdResp[4] = endianArray[0];
                dObj->pCmdResp[5] = gDrvSDCARDCmdTable[command].crc;
                /* Dummy data. Only used in case of DRV_SDCARD_STOP_TRANSMISSION */
                dObj->pCmdResp[6] = 0xFF;

                dObj->cmdState = DRV_SDCARD_CMD_SEND_PACKET;
                break;
            }

        case DRV_SDCARD_CMD_SEND_PACKET:
            {
                /* Write the framed packet to the card */
                DRV_SPI_BufferAddWrite2(dObj->spiClientHandle, dObj->pCmdResp,
                        DRV_SDCARD_PACKET_SIZE, 0, dObj, &dObj->spiBufferHandle);

                if (dObj->spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID)
                {
                    dObj->cmdState = DRV_SDCARD_CMD_EXEC_ERROR;
                    break;
                }

                if (command != DRV_SDCARD_STOP_TRANSMISSION)
                {
                    dObj->cmdState = DRV_SDCARD_CMD_CHECK_TRANSFER_COMPLETE;
                }
                else
                {
                    /* Do an extra read for this command */
                    dObj->cmdState = DRV_SDCARD_CMD_CHECK_SPL_CASE;
                }
                break;
            }

        case DRV_SDCARD_CMD_CHECK_SPL_CASE:
            {
                spiRetVal = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    /* Do a dummy read */
                    DRV_SPI_BufferAddRead2(dObj->spiClientHandle, dObj->pCmdResp, 1,
                    0, dObj, &dObj->spiBufferHandle);

                    dObj->cmdState = DRV_SDCARD_CMD_CHECK_TRANSFER_COMPLETE;
                }

                if ((spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR) || 
                    (dObj->spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID))
                {
                    dObj->cmdState = DRV_SDCARD_CMD_EXEC_ERROR;
                }
            }
            break;

        case DRV_SDCARD_CMD_CHECK_TRANSFER_COMPLETE:
            {
                spiRetVal = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    DRV_SPI_BufferAddRead2 (dObj->spiClientHandle, &dObj->cmdResponse.response1.byte,
                        1, 0, dObj, &dObj->spiBufferHandle);
                    /* Act as per the response type */
                    dObj->cmdState = DRV_SDCARD_CMD_CHECK_RESP_TYPE;
                }

                if ((spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR) || 
                    (dObj->spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID))
                {
                    dObj->cmdState = DRV_SDCARD_CMD_EXEC_ERROR;
                }
            }
            break;

        case DRV_SDCARD_CMD_CHECK_RESP_TYPE:
            {
                spiRetVal = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    if (dObj->cmdResponse.response1.byte == DRV_SDCARD_MMC_FLOATING_BUS)
                    {
                        if (dObj->cmdRespTmrFlag == false)
                        {
                            dObj->cmdRespTmrHandle = SYS_TMR_DelayMS (_DRV_SDCARD_FLOATING_BUS_TIMEOUT);
                            dObj->cmdRespTmrFlag = true;
                        }

                        if (SYS_TMR_DelayStatusGet(dObj->cmdRespTmrHandle))
                        {
                            /* Abort the command operation. */
                            dObj->cmdState = DRV_SDCARD_CMD_EXEC_ERROR;
                            dObj->cmdRespTmrFlag = false;
                        }
                        else
                        {
                            dObj->cmdState = DRV_SDCARD_CMD_CHECK_TRANSFER_COMPLETE;
                        }

                        break;
                    }

                    /* Received the response. Stop the timer */
                    SYS_TMR_CallbackStop (dObj->cmdRespTmrHandle);
                    dObj->cmdRespTmrFlag = false;

                    switch (gDrvSDCARDCmdTable[command].responseType)
                    {
                        case RESPONSE_R1:
                            {
                                /* Device requires at least 8 clock pulses after the response
                                   has been sent, before it can process the next command.
                                   CS may be high or low */
                                DRV_SPI_BufferAddRead2 (dObj->spiClientHandle, dObj->pCmdResp, 
                                        _DRV_SDCARD_SEND_8_CLOCKS, 0, dObj, &dObj->spiBufferHandle);

                                dObj->cmdState = DRV_SDCARD_CMD_EXEC_CHECK_COMPLETION;
                            }
                            break;

                        case RESPONSE_R2:
                            {
                                /* We already received the first byte, just make sure it is in the
                                   correct location in the structure. */
                                dObj->cmdResponse.response2.byte1 = dObj->cmdResponse.response1.byte;

                                /* Fetch the second byte of the response. */
                                DRV_SPI_BufferAddRead2 (dObj->spiClientHandle, &dObj->cmdResponse.response2.byte1,
                                    1, 0, dObj, &dObj->spiBufferHandle);

                                dObj->cmdState = DRV_SDCARD_CMD_HANDLE_R2_RESPONSE;
                            }
                            break;

                        case RESPONSE_R1b:
                            {
                                /* Keep trying to read from the media, until it signals it is no longer
                                   busy. It will continuously send 0x00 bytes until it is not busy.
                                   A non-zero value means it is ready for the next command.
                                   The R1b response is received after a CMD12,  CMD_STOP_TRANSMISSION
                                   command, where the media card may be busy writing its internal buffer
                                   to the flash memory.  This can typically take a few milliseconds,
                                   with a recommended maximum time-out of 250ms or longer for SD cards.
                                 */
                                DRV_SPI_BufferAddRead2 (dObj->spiClientHandle,
                                        (uint8_t*)&dObj->cmdResponse.response1.byte, 1, 
                                        0, dObj, &dObj->spiBufferHandle);

                                dObj->cmdState = DRV_SDCARD_CMD_R1B_READ_BACK;
                            }
                            break;

                        case RESPONSE_R7:
                            {
                                /* Fetch the other four bytes of the R3 or R7 response. */
                                /* Note: The SD card argument response field is 32-bit, big endian format. 
                                   However, the C compiler stores 32-bit values in little endian in RAM. 
                                   When writing to the bytes, make sure the order it gets stored in is correct. */
                                DRV_SPI_BufferAddRead2 (dObj->spiClientHandle, dObj->pCmdResp, 4, 
                                            0, dObj, &dObj->spiBufferHandle);

                                /* Expected response it of R7 type */
                                dObj->cmdState = DRV_SDCARD_CMD_HANDLE_R7_RESPONSE;
                            }
                            break;

                        case RESPONSE_R3:
                        default:
                            break;
                    }
                }

                if ((spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR) || 
                    (dObj->spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID))
                {
                    dObj->cmdState = DRV_SDCARD_CMD_EXEC_ERROR;
                }
            }
            break;

        case DRV_SDCARD_CMD_R1B_READ_BACK:
            {
                spiRetVal = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    if (dObj->cmdResponse.response1.byte != 0x00)
                    {
                        /* Device requires at least 8 clock pulses after the response
                           has been sent, before it can process the next command.
                           CS may be high or low */
                        DRV_SPI_BufferAddWrite2 (dObj->spiClientHandle, dObj->pClkPulseData, 
                        _DRV_SDCARD_SEND_8_CLOCKS, 0, dObj, &dObj->spiBufferHandle);

                        dObj->cmdState = DRV_SDCARD_CMD_EXEC_CHECK_COMPLETION;

                        /* Received the response. Stop the timer */
                        SYS_TMR_CallbackStop (dObj->cmdRespTmrHandle);
                        dObj->cmdRespTmrFlag = false;
                    }
                    else
                    {
                        if (dObj->cmdRespTmrFlag == false)
                        {
                            dObj->cmdRespTmrHandle = SYS_TMR_DelayMS (_DRV_SDCARD_R1B_RESP_TIMEOUT);
                            dObj->cmdRespTmrFlag = true;
                        }

                        if (SYS_TMR_DelayStatusGet(dObj->cmdRespTmrHandle))
                        {
                            /* Abort the command operation. */
                            dObj->cmdState = DRV_SDCARD_CMD_EXEC_ERROR;
                            dObj->cmdRespTmrFlag = false;
                            break;
                        }

                        DRV_SPI_BufferAddRead2 (dObj->spiClientHandle, (uint8_t*)&dObj->cmdResponse.response1.byte,
                                1, 0, dObj, &dObj->spiBufferHandle);

                        dObj->cmdState = DRV_SDCARD_CMD_R1B_READ_BACK;
                    }
                }

                if ((spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR) || 
                    (dObj->spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID))
                {
                    dObj->cmdState = DRV_SDCARD_CMD_EXEC_ERROR;
                    SYS_TMR_CallbackStop (dObj->cmdRespTmrHandle);
                    dObj->cmdRespTmrFlag = false;
                }
            }
            break;

        case DRV_SDCARD_CMD_HANDLE_R2_RESPONSE:
            {
                spiRetVal = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    /* Device requires at least 8 clock pulses after the response
                       has been sent, before if can process the next command.
                       CS may be high or low */
                    DRV_SPI_BufferAddWrite2 (dObj->spiClientHandle, dObj->pClkPulseData, 
                        _DRV_SDCARD_SEND_8_CLOCKS, 0, dObj, &dObj->spiBufferHandle);

                    dObj->cmdState = DRV_SDCARD_CMD_EXEC_CHECK_COMPLETION;
                }

                if ((spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR) || 
                    (dObj->spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID))
                {
                    dObj->cmdState = DRV_SDCARD_CMD_EXEC_ERROR;
                }
            }
            break;

        case DRV_SDCARD_CMD_HANDLE_R7_RESPONSE:
            {
                spiRetVal = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    /* Handle the endianness */
                    dObj->cmdResponse.response7.bytewise.argument.ocrRegisterByte0 =
                        dObj->pCmdResp[3];
                    dObj->cmdResponse.response7.bytewise.argument.ocrRegisterByte1 =
                        dObj->pCmdResp[2];
                    dObj->cmdResponse.response7.bytewise.argument.ocrRegisterByte2 =
                        dObj->pCmdResp[1];
                    dObj->cmdResponse.response7.bytewise.argument.ocrRegisterByte3 =
                        dObj->pCmdResp[0];

                    /* Device requires at least 8 clock pulses after the response
                       has been sent, before if can process the next command.
                       CS may be high or low */
                    DRV_SPI_BufferAddWrite2 (dObj->spiClientHandle, dObj->pClkPulseData, 
                            _DRV_SDCARD_SEND_8_CLOCKS, 0, dObj, &dObj->spiBufferHandle);

                    dObj->cmdState = DRV_SDCARD_CMD_EXEC_CHECK_COMPLETION;
                }

                if ((spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR) ||
                    (dObj->spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID))
                {
                    dObj->cmdState = DRV_SDCARD_CMD_EXEC_ERROR;
                }
            }
            break;

        case DRV_SDCARD_CMD_EXEC_CHECK_COMPLETION:
            {
                spiRetVal = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    dObj->cmdState = DRV_SDCARD_CMD_CONFIRM_COMPLETE;
                }
                else if (spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    dObj->cmdState = DRV_SDCARD_CMD_EXEC_ERROR;
                }
            }
            break;

        case DRV_SDCARD_CMD_CONFIRM_COMPLETE:
            dObj->cmdState = DRV_SDCARD_CMD_EXEC_IS_COMPLETE;
            break;

        case DRV_SDCARD_CMD_EXEC_ERROR:
        case DRV_SDCARD_CMD_EXEC_IS_COMPLETE:
            /* This code will be the first case statement getting executed on calling
               _DRV_SDCARD_CommandSend function, except first time */
            dObj->cmdState = DRV_SDCARD_CMD_FRAME_PACKET;
            break;

        default:
            break;
    }
} /* End of _DRV_SDCARD_CommandSend */

//******************************************************************************
/* Function:
    static bool _DRV_SDCARD_MediaCommandDetect ( SYS_MODULE_OBJ object )

  Summary:
    Determines whether an SD card is present using a command response method.

  Description:
    This routine determines whether an SD card is present using a command
    response method. If it is a Micro SD card(doesn't has a card detect pin)
    calling this API is the only option detect the presence of the card. This API
    could be called directly or DRV_SDCARD_MediaDetect function will call this
    API based on the configuration.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    true - The Card is present.
    false - The Card is not present.
*/

uint16_t resetTimeout = 0;

static bool _DRV_SDCARD_MediaCommandDetect 
(
    SYS_MODULE_OBJ object
)
{
    DRV_SDCARD_OBJ *dObj = (DRV_SDCARD_OBJ*)_DRV_SDCARD_INSTANCE_GET(object);
    DRV_SPI_BUFFER_EVENT spiRetVal;

    switch (dObj->cmdDetectState)
    {
        case DRV_SDCARD_CMD_DETECT_START_INIT:
            {
                /* If the SPI module is not enabled, then the media has evidently not
                   been initialized.  Try to send CMD0 and CMD13 to reset the device and
                   get it into SPI mode (if present), and then request the status of
                   the media.  If this times out, then the card is presumably not
                   physically present */
                dObj->spiClientData.baudRate = _DRV_SDCARD_SPI_INITIAL_SPEED;
                DRV_SPI_ClientConfigure (dObj->spiClientHandle, &dObj->spiClientData);
                dObj->cmdDetectState = DRV_SDCARD_CMD_DETECT_CHECK_FOR_CARD;
                /* Note: Intentional fallthrough */
            }
        case DRV_SDCARD_CMD_DETECT_CHECK_FOR_CARD:
            {
                /* Send CMD0 to reset the media. If the card is physically present,
                   then we should get a valid response. Toggle chip select, to make
                   media abandon whatever it may have been doing before.  This ensures
                   the CMD0 is sent freshly after CS is asserted low, minimizing risk
                   of SPI clock pulse master/slave synchronization problems, due to
                   possible application noise on the SCK line. */
                _DRV_SDCARD_CHIP_DESELECT (dObj->chipSelectPort,
                        dObj->chipSelectBitPosition);

                /* Send some "extraneous" clock pulses.  If a previous command was
                   terminated before it completed normally, the card might not have
                   received the required clocking following the transfer. */
                DRV_SPI_BufferAddWrite2 (dObj->spiClientHandle, dObj->pClkPulseData, 
                            MEDIA_INIT_ARRAY_SIZE, 0, dObj, &dObj->spiBufferHandle);

                if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                {
                    dObj->cmdDetectState = DRV_SDCARD_CMD_DETECT_READ_SPI_DATA;
                }
            }
            break;

        case DRV_SDCARD_CMD_DETECT_READ_SPI_DATA:
            {
                spiRetVal = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    /* We send dummy data to send clock pulses. Clear the receive
                       interrupt caused due to that. */
                    DRV_SPI_BufferAddRead2 (dObj->spiClientHandle, dObj->pCmdResp, 1, 0, 
                            dObj, &dObj->spiBufferHandle);

                    if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                    {
                        dObj->cmdDetectState = DRV_SDCARD_CMD_DETECT_READ_SPI_DATA_DONE;
                    }
                    else
                    {
                        dObj->cmdDetectState = DRV_SDCARD_CMD_DETECT_DELAY_CHECK_FOR_CARD;
                    }
                }
                else if (spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    dObj->cmdDetectState = DRV_SDCARD_CMD_DETECT_DELAY_CHECK_FOR_CARD;
                }
            }
            break;

        case DRV_SDCARD_CMD_DETECT_READ_SPI_DATA_DONE:
            {
                spiRetVal = DRV_SPI_BufferStatus (dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    dObj->cmdDetectState = DRV_SDCARD_CMD_DETECT_RESET_SDCARD;
                }
                else if (spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    dObj->cmdDetectState = DRV_SDCARD_CMD_DETECT_DELAY_CHECK_FOR_CARD;
                }
            }
            break;

        case DRV_SDCARD_CMD_DETECT_RESET_SDCARD:
            {
                _DRV_SDCARD_CommandSend (object, DRV_SDCARD_GO_IDLE_STATE, 0x00);

                if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE)
                {
                    /* R1 response byte should be 0x01 after CMD_GO_IDLE_STATE */
                    if (dObj->cmdResponse.response1.byte != CMD_R1_END_BIT_SET)
                    {
                        /* Assuming that the card is not present. */
                        dObj->cmdDetectState = DRV_SDCARD_CMD_DETECT_DELAY_CHECK_FOR_CARD;
                    }
                    else
                    {
                        dObj->cmdDetectState = DRV_SDCARD_CMD_DETECT_INIT_DUMMY_STATE;
                        return true;
                    }
                }
                else if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_ERROR)
                {
                    dObj->cmdDetectState = DRV_SDCARD_CMD_DETECT_DELAY_CHECK_FOR_CARD;
                }
            }
            break;

        case DRV_SDCARD_CMD_DETECT_DELAY_CHECK_FOR_CARD:
            resetTimeout ++;
            if (resetTimeout >= 50)
            {
                dObj->cmdDetectState = DRV_SDCARD_CMD_DETECT_CHECK_FOR_CARD;
                resetTimeout = 0;
            }
            break;

        case DRV_SDCARD_CMD_DETECT_INIT_DUMMY_STATE:
            break;

        case DRV_SDCARD_CMD_DETECT_CHECK_FOR_DETACH:
            {
                if (dObj->sdState == TASK_STATE_CARD_COMMAND)
                {
                    return true;
                }

                dObj->sdState = TASK_STATE_CARD_STATUS;

                _DRV_SDCARD_CommandSend (object, DRV_SDCARD_SEND_STATUS, 0x00);
                if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE)
                {
                    /* For status command SD card will respond with R2 type packet */
                    dObj->sdState = TASK_STATE_IDLE;

                    if ((dObj->cmdResponse.response2.word & 0xEC0C) != 0x0000)
                    {
                        dObj->cmdDetectState = DRV_SDCARD_CMD_DETECT_CHECK_FOR_CARD;
                        return false;
                    }
                    else
                    {
                        dObj->cmdDetectState = DRV_SDCARD_CMD_DETECT_CHECK_FOR_DETACH;
                    }
                }
                else if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_ERROR)
                {
                    dObj->sdState = TASK_STATE_IDLE;
                    dObj->cmdDetectState = DRV_SDCARD_CMD_DETECT_CHECK_FOR_CARD;
                    return false;
                }
            }
            return true;

        default:
            break;
    }

    return false;
}

//******************************************************************************
/* Function:
    static void _DRV_SDCARD_MediaInitialize ( SYS_MODULE_OBJ object )

  Summary:
    Initializes the SD card.

  Description:
    The function updates MEDIA_INFORMATION structure.  The
    errorCode member may contain the following values:
        * MEDIA_NO_ERROR - The media initialized successfully
        * MEDIA_CANNOT_INITIALIZE - Cannot initialize the media.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's
                   open routine.

  Returns:
    None
*/

static void _DRV_SDCARD_MediaInitialize ( SYS_MODULE_OBJ object )
{
    /* Get the driver object */
    DRV_SDCARD_OBJ *dObj = ( DRV_SDCARD_OBJ* ) _DRV_SDCARD_INSTANCE_GET ( object );
    uint8_t cSizeMultiplier;
    uint16_t blockLength;
    uint32_t cSize;
    uint32_t mult;
    DRV_SPI_BUFFER_EVENT        spiRetVal;    

    /* Check what state we are in, to decide what to do */
    switch (dObj->mediaInitState)
    {
        case DRV_SDCARD_INIT_CHIP_DESELECT:
            {
                dObj->discCapacity = 0;
                dObj->sdCardType = DRV_SDCARD_MODE_NORMAL;

                /* Keep the chip select high(not selected) to send clock pulses  */
                _DRV_SDCARD_CHIP_DESELECT (dObj->chipSelectPort, dObj->chipSelectBitPosition);

                /* 400kHz. Initialize SPI port to <= 400kHz */
                dObj->spiClientData.baudRate = _DRV_SDCARD_SPI_INITIAL_SPEED;
                DRV_SPI_ClientConfigure (dObj->spiClientHandle, &dObj->spiClientData);

                dObj->mediaInitState = DRV_SDCARD_INIT_RAMP_TIME;
            }
            break;

        case DRV_SDCARD_INIT_RAMP_TIME:
            {
                /* Send at least 74 clock pulses with chip select high */
                DRV_SPI_BufferAddWrite2 (dObj->spiClientHandle, ( uint8_t* ) dObj->pClkPulseData,
                        MEDIA_INIT_ARRAY_SIZE, 0, 0, &dObj->spiBufferHandle);

                if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                {
                    /* Check the status of completion in the next state */
                    dObj->mediaInitState = DRV_SDCARD_INIT_CHIP_SELECT;
                }
                else
                {
                    dObj->mediaInitState = DRV_SDCARD_INIT_ERROR;
                }
            }
            break;

        case DRV_SDCARD_INIT_CHIP_SELECT:
            {
                /* This ensures the CMD0 is sent freshly after CS is asserted low,
                   minimizing risk of SPI clock pulse master/slave synchronization problems,
                   due to possible application noise on the SCK line.
                 */
                dObj->mediaInitState = DRV_SDCARD_INIT_RESET_SDCARD;
            }
            break;

        case DRV_SDCARD_INIT_RESET_SDCARD:
            {
                /* Send the command (CMD0) to software reset the device  */
                _DRV_SDCARD_CommandSend(object, DRV_SDCARD_GO_IDLE_STATE, 0x00);

                /* Change from this state only on completion of command execution */
                if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE)
                {
                    if (dObj->cmdResponse.response1.byte == CMD_R1_END_BIT_SET)
                    {
                        dObj->mediaInitState = DRV_SDCARD_INIT_CHK_IFACE_CONDITION;
                    }
                    else
                    {
                        dObj->mediaInitState = DRV_SDCARD_INIT_RAMP_TIME;
                    }
                }
                else if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_ERROR)
                {
                    dObj->mediaInitState = DRV_SDCARD_INIT_ERROR;
                }
            }
            break;

        case DRV_SDCARD_INIT_CHK_IFACE_CONDITION:
            {
                /* Send CMD8 (SEND_IF_COND) to specify/request the SD card interface
                   condition (ex: indicate what voltage the host runs at).
                   0x000001AA --> VHS = 0001b = 2.7V to 3.6V.  The 0xAA LSB is the check
                   pattern, and is arbitrary, but 0xAA is recommended (good blend of 0's
                   and '1's). The SD card has to echo back the check pattern correctly
                   however, in the R7 response. If the SD card doesn't support the
                   operating voltage range of the host, then it may not respond. If it
                   does support the range, it will respond with a type R7 response packet
                   (6 bytes/48 bits). Additionally, if the SD card is MMC or SD card
                   v1.x spec device, then it may respond with invalid command.  If it is
                   a v2.0 spec SD card, then it is mandatory that the card respond to CMD8
                 */
                _DRV_SDCARD_CommandSend(object, DRV_SDCARD_SEND_IF_COND, 0x1AA);

                /* Note: CRC value in the table is set for value "0x1AA", it should
                   be changed if a different value is passed. */

                /* Change from this state only on completion of command execution */
                if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE)
                {
                    if (((dObj->cmdResponse.response7.bytewise.argument.ocrRegister & 0xFFF) == 0x1AA)
                            && (false == dObj->cmdResponse.response7.bitwise.bits.illegalCommand))
                    {
                        dObj->sdHcHost = 1;
                    }
                    else
                    {
                        dObj->sdHcHost = 0;
                    }

                    dObj->mediaInitState = DRV_SDCARD_INIT_READ_OCR_REGISTER;
                }
                else if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_ERROR)
                {
                    dObj->mediaInitState = DRV_SDCARD_INIT_ERROR;
                }
            }
            break;

        case DRV_SDCARD_INIT_READ_OCR_REGISTER:
            /*Send CMD58 (Read OCR [operating conditions register]).  Response
            type is R3, which has 5 bytes. Byte 4 = normal R1 response byte,
            Bytes 3-0 are = OCR register value.
            */
            _DRV_SDCARD_CommandSend (object, DRV_SDCARD_READ_OCR, 0x00);

            /* Change from this state only on completion of command execution */
            if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE)
            {
                dObj->mediaInitState = DRV_SDCARD_INIT_SEND_APP_CMD;
                dObj->appCmdTmrFlag = true;
            }
            else if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_ERROR)
            {
                dObj->mediaInitState = DRV_SDCARD_INIT_ERROR;
            }
            break;

        case DRV_SDCARD_INIT_SEND_APP_CMD:
            {
                /* Send CMD55 (lets SD card know that the next command is application
                   specific (going to be ACMD41)) */
                _DRV_SDCARD_CommandSend(object, DRV_SDCARD_APP_CMD, 0x00);

                /* Change from this state only on completion of command execution */
                if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE)
                {
                    dObj->mediaInitState = DRV_SDCARD_INIT_SEND_ACMD41;
                }
                else if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_ERROR)
                {
                    dObj->mediaInitState = DRV_SDCARD_INIT_ERROR;

                    dObj->appCmdTmrFlag = false;
                    /* Stop the timer */
                    SYS_TMR_CallbackStop (dObj->appCmdTmrHandle);
                }
            }
            break;

        case DRV_SDCARD_INIT_SEND_ACMD41:
            {
                /* Send ACMD41.  This is to check if the SD card is finished booting
                   up/ready for full frequency and all further commands.  Response is
                   R3 type (6 bytes/48 bits, middle four bytes contain potentially useful
                   data). */
                /* Note: When sending ACMD41, the HCS bit is bit 30, and must be = 1 to
                   tell SD card the host supports SDHC
                 */
                _DRV_SDCARD_CommandSend(object, DRV_SDCARD_SD_SEND_OP_COND, dObj->sdHcHost << 30);

                /* Change from this state only on completion of command execution */
                if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE)
                {
                    if (dObj->cmdResponse.response1.byte == 0x00)
                    {
                        dObj->mediaInitState = DRV_SDCARD_INIT_READ_OCR;
                        dObj->appCmdTmrFlag = false;
                        /* Stop the timer */
                        SYS_TMR_CallbackStop (dObj->appCmdTmrHandle);
                    }
                    else
                    {
                        if (dObj->appCmdTmrFlag == false)
                        {
                            dObj->appCmdTmrHandle = SYS_TMR_DelayMS (_DRV_SDCARD_APP_CMD_RESP_TIMEOUT_IN_MS);
                            dObj->appCmdTmrFlag = true;
                        }

                        if (SYS_TMR_DelayStatusGet(dObj->appCmdTmrHandle))
                        {
                            dObj->mediaInitState = DRV_SDCARD_INIT_ERROR;
                            dObj->appCmdTmrFlag = false;
                        }
                        else
                        {
                            dObj->mediaInitState = DRV_SDCARD_INIT_SEND_APP_CMD;
                        }
                    }
                }
                else if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_ERROR)
                {
                    dObj->appCmdTmrFlag = false;
                    /* Stop the timer */
                    SYS_TMR_CallbackStop (dObj->appCmdTmrHandle);
                    dObj->mediaInitState = DRV_SDCARD_INIT_ERROR;
                }
            }
            break;

        case DRV_SDCARD_INIT_READ_OCR:
            {
                /* Now send CMD58(Read OCR register ). The OCR register contains
                   important info we will want to know about the card (ex: standard
                   capacity vs. SDHC).
                 */
                _DRV_SDCARD_CommandSend(object, DRV_SDCARD_READ_OCR, 0x00);

                /* Change from this state only on completion of command execution */
                if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE)
                {
                    /* Now check the CCS bit (OCR bit 30) in the OCR register, which
                       is in our response packet. This will tell us if it is a SD high
                       capacity (SDHC) or standard capacity device. */
                    /* Note: the HCS bit is only valid when the busy bit is also set
                       (indicating device ready).
                     */
                    if (dObj->cmdResponse.response7.bytewise.argument.ocrRegister & 0x40000000)
                    {
                        dObj->sdCardType = DRV_SDCARD_MODE_HC;
                    }
                    else
                    {
                        dObj->sdCardType = DRV_SDCARD_MODE_NORMAL;
                    }

                    /* Card initialization is complete, switch to normal operation */
                    dObj->mediaInitState = DRV_SDCARD_INIT_INCR_CLOCK_SPEED;
                }
                else if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_ERROR)
                {
                    dObj->mediaInitState = DRV_SDCARD_INIT_ERROR;
                }
            }
            break;

        case DRV_SDCARD_INIT_INCR_CLOCK_SPEED:
            {
                /* Temporarily keep the card de-selected */
                _DRV_SDCARD_CHIP_DESELECT(dObj->chipSelectPort, dObj->chipSelectBitPosition);

                /* Basic initialization of media is now complete.  The card will now
                   use push/pull outputs with fast drivers.  Therefore, we can now increase
                   SPI speed to either the maximum of the micro-controller or maximum of
                   media, whichever is slower.  MMC media is typically good for at least
                   20Mbps SPI speeds. SD cards would typically operate at up to 25Mbps
                   or higher SPI speeds.
                 */
                /* 400kHz. Initialize SPI port to <= 400kHz */
                dObj->spiClientData.baudRate = dObj->sdcardSpeedHz;
                DRV_SPI_ClientConfigure (dObj->spiClientHandle, &dObj->spiClientData);
                /* Do a dummy read to ensure that the receiver buffer is cleared */
                DRV_SPI_BufferAddRead2 (dObj->spiClientHandle, dObj->pCmdResp, 10, 0, 0, 
                                &dObj->spiBufferHandle);

                dObj->mediaInitState = DRV_SDCARD_INIT_INCR_CLOCK_SPEED_STATUS;
            }
            break;

            case DRV_SDCARD_INIT_INCR_CLOCK_SPEED_STATUS:
            {
                spiRetVal = DRV_SPI_BufferStatus (dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    dObj->mediaInitState = DRV_SDCARD_INIT_READ_CSD;
                }
                else if (spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    dObj->mediaInitState = DRV_SDCARD_INIT_ERROR;
                }
                break;
            }
	    
        case DRV_SDCARD_INIT_READ_CSD:
            {
                /* CMD9: Read CSD data structure */
                _DRV_SDCARD_CommandSend (object, DRV_SDCARD_SEND_CSD, 0x00);

                /* Change from this state only on completion of command execution */
                if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE)
                {
                    if (dObj->cmdResponse.response1.byte == 0x00)
                    {
                        dObj->mediaInitState = DRV_SDCARD_INIT_READ_CSD_DATA;
                    }
                    else
                    {
                        dObj->mediaInitState = DRV_SDCARD_INIT_ERROR;
                    }
                }
                else if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_ERROR)
                {
                    dObj->mediaInitState = DRV_SDCARD_INIT_ERROR;
                }
            }
            break;

        case DRV_SDCARD_INIT_READ_CSD_DATA:
            {
                /* According to the simplified spec, section 7.2.6, the card will respond
                   with a standard response token, followed by a data block of 16 bytes
                   suffixed with a 16-bit CRC.
                 */
                DRV_SPI_BufferAddRead2 (dObj->spiClientHandle, dObj->pCsdData,
                        _DRV_SDCARD_CSD_READ_SIZE, 0, dObj, &dObj->spiBufferHandle);

                if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                {
                    dObj->mediaInitState = DRV_SDCARD_INIT_PROCESS_CSD;
                }
                else
                {
                    dObj->mediaInitState = DRV_SDCARD_INIT_ERROR;
                }
            }
            break;

        case DRV_SDCARD_INIT_PROCESS_CSD:
            {
                spiRetVal = DRV_SPI_BufferStatus (dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    uint8_t *csdPtr;
                    /* Extract some fields from the response for computing the card
                       capacity. */
                    /* Note: The structure format depends on if it is a CSD V1 or V2 device.
                       Therefore, need to first determine version of the specs that the card
                       is designed for, before interpreting the individual fields.
                     */
                    /* Calculate the MDD_SDSPI_finalLBA (see SD card physical layer
                       simplified spec 2.0, section 5.3.2).
                       In USB mass storage applications, we will need this information
                       to correctly respond to SCSI get capacity requests.  Note: method
                       of computing MDD_SDSPI_finalLBA TODO depends on CSD structure spec
                       version (either v1 or v2).
                     */
                    csdPtr = &dObj->pCsdData[0];
                    if (dObj->pCsdData[0] == DRV_SDCARD_DATA_START_TOKEN)
                    {
                        /* Note: This is a workaround. Some cards issue data start token
                        before sending the 16 byte csd data and some don't. */
                        csdPtr = &dObj->pCsdData[1];
                    }

                    if (csdPtr[0] & _DRV_SDCARD_CHECK_V2_DEVICE)
                    {
                        /* Check CSD_STRUCTURE field for v2+ struct device */
                        /* Must be a v2 device (or a reserved higher version, that
                           doesn't currently exist) */
                        /* Extract the C_SIZE field from the response.  It is a 22-bit
                           number in bit position 69:48.  This is different from v1.
                           It spans bytes 7, 8, and 9 of the response.
                         */
                        cSize = (((uint32_t)csdPtr[7] & 0x3F) << 16) | ((uint16_t)csdPtr[8] << 8) | csdPtr[9];
                        dObj->discCapacity = ((uint32_t)(cSize + 1) * (uint16_t)(1024u));
                    }
                    else /* Not a V2 device, Must be a V1 device */
                    {
                        /* Must be a v1 device. Extract the C_SIZE field from the
                           response.  It is a 12-bit number in bit position 73:62.
                           Although it is only a 12-bit number, it spans bytes 6, 7,
                           and 8, since it isn't byte aligned.
                         */
                        cSize = csdPtr[6] & 0x3;
                        cSize <<= 8;
                        cSize |= csdPtr[7];
                        cSize <<= 2;
                        cSize |= (csdPtr[8] >> 6);
                        /* Extract the C_SIZE_MULT field from the response.  It is a
                           3-bit number in bit position 49:47 */
                        cSizeMultiplier = (csdPtr[9] & 0x03) << 1;
                        cSizeMultiplier |= ((csdPtr[10] & 0x80) >> 7);

                        /* Extract the BLOCK_LEN field from the response. It is a
                           4-bit number in bit position 83:80
                         */
                        blockLength = csdPtr[5] & 0x0F;
                        blockLength = 1 << (blockLength - 9);

                        /* Calculate the capacity (see SD card physical layer simplified
                           spec 2.0, section 5.3.2). In USB mass storage applications,
                           we will need this information to correctly respond to SCSI get
                           capacity requests (which will cause MDD_SDSPI_ReadCapacity()
                           to get called).
                         */

                        mult = 1 << (cSizeMultiplier + 2);
                        dObj->discCapacity = (((uint32_t)(cSize + 1) * mult) * blockLength);
                    }

                    dObj->mediaInitState = DRV_SDCARD_INIT_TURN_OFF_CRC;
                }
                else if (spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    dObj->mediaInitState = DRV_SDCARD_INIT_ERROR;
                }
            }
            break;

        case DRV_SDCARD_INIT_TURN_OFF_CRC:
            {
                /* Turn off CRC7 if we can, might be an invalid cmd on some
                   cards (CMD59). */
                /* Note: POR default for the media is normally with CRC checking
                   off in SPI mode anyway, so this is typically redundant.
                 */
                _DRV_SDCARD_CommandSend (object, DRV_SDCARD_CRC_ON_OFF, 0x00);

                /* Change from this state only on completion of command execution */
                if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE)
                {
                    dObj->mediaInitState = DRV_SDCARD_INIT_SET_BLOCKLEN;
                }
                else if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_ERROR)
                {
                    dObj->mediaInitState = DRV_SDCARD_INIT_ERROR;
                }
            }
            break;

        case DRV_SDCARD_INIT_SET_BLOCKLEN:
            {
                /* Now set the block length to media sector size. It
                   should be already set to this. */
                _DRV_SDCARD_CommandSend(object, DRV_SDCARD_SET_BLOCKLEN, _DRV_SDCARD_MEDIA_BLOCK_SIZE);

                /* Change from this state only on completion of command execution */
                if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE)
                {
                    dObj->mediaInitState = DRV_SDCARD_INIT_SD_INIT_DONE;
                }
                else if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_ERROR)
                {
                    dObj->mediaInitState = DRV_SDCARD_INIT_ERROR;
                }
            }
            break;

        case DRV_SDCARD_INIT_SD_INIT_DONE:
            /* Coming for the first time */
            dObj->mediaInitState = DRV_SDCARD_INIT_CHIP_DESELECT;
            break;

        case DRV_SDCARD_INIT_ERROR:
            dObj->mediaInitState = DRV_SDCARD_INIT_CHIP_DESELECT;
            break;

        default:
            break;
    }
}

// *****************************************************************************
/* Function:
    static void _DRV_SDCARD_AttachDetachTasks 
     ( 
         SYS_MODULE_OBJ object
     );

  Summary:
    Maintains the driver's state machine.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine is used to maintain the driver's internal state machine.

  Precondition:
    The DRV_SDCARD_Initialize routine must have been called for the specified
    SDCARD driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_SDCARD_Initialize)

  Returns:
    None

  Remarks:
*/

static void _DRV_SDCARD_AttachDetachTasks 
( 
    SYS_MODULE_OBJ object 
)
{
    DRV_SDCARD_OBJ *dObj;
    OSAL_RESULT retVal;

    /* Get the driver object */
    dObj = (DRV_SDCARD_OBJ*)_DRV_SDCARD_INSTANCE_GET(object);

    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_ASSERT(false, "SDCard Driver: OSAL_MUTEX_Lock failed");
    }

    /* Check what state we are in, to decide what to do */
    switch ( dObj->taskState )
    {
        case DRV_SDCARD_TASK_OPEN_SPI:
            {
                /* Open the SPI in Read/Write mode. No other tasks will be
                   allowed to control the SD card. But there could be multiple clients
                   for SD card driver */
                dObj->spiClientHandle = DRV_SPI_Open (dObj->spiIndex, DRV_IO_INTENT_READWRITE);
                if(dObj->spiClientHandle != DRV_HANDLE_INVALID)
                {
                    dObj->taskState = DRV_SDCARD_TASK_CHECK_DEVICE;
                }
            }
            break;

        case DRV_SDCARD_TASK_CHECK_DEVICE:
            {
                /* Check for device attach */
                dObj->isAttached = _DRV_SDCARD_MediaCommandDetect (object);
                if (dObj->isAttachedLastStatus != dObj->isAttached)
                {
                    dObj->isAttachedLastStatus = dObj->isAttached;
                    /* We should call a function on device attach and detach */
                    if (DRV_SDCARD_IS_ATTACHED == dObj->isAttached)
                    {
                        /* An SD card seems to be present. Initiate a full card initialization. */
                        dObj->taskState = DRV_SDCARD_TASK_MEDIA_INIT;
                    }
                    else
                    {
                        _DRV_SDCARD_RemoveQueuedRequests (dObj);
                        dObj->mediaState = SYS_FS_MEDIA_DETACHED;
                    }
                }
            }
            break;

        case DRV_SDCARD_TASK_MEDIA_INIT:
            {
                /* Update the card details to the internal data structure */
                _DRV_SDCARD_MediaInitialize (object);

                /* Once the initialization is complete, move to the next stage */
                if (dObj->mediaInitState == DRV_SDCARD_INIT_SD_INIT_DONE)
                {
                    /* Check and update the card's write protected status */
                    _DRV_SDCARD_CheckWriteProtectStatus (dObj);

                    /* Update the Media Geometry structure */
                    _DRV_SDCARD_UpdateGeometry (dObj);

                    dObj->cmdDetectState = DRV_SDCARD_CMD_DETECT_CHECK_FOR_DETACH;
                    /* State that the device is attached. */
                    dObj->mediaState = SYS_FS_MEDIA_ATTACHED;
                    dObj->taskState = DRV_SDCARD_TASK_CHECK_DEVICE;
                }
                else if (dObj->mediaInitState == DRV_SDCARD_INIT_ERROR)
                {
                    dObj->cmdDetectState = DRV_SDCARD_CMD_DETECT_CHECK_FOR_CARD;
                    dObj->taskState = DRV_SDCARD_TASK_CHECK_DEVICE;
                }

                break;
            }

        case DRV_SDCARD_TASK_IDLE:
            {
                break;
            }
    }

    retVal = OSAL_MUTEX_Unlock(&dObj->mutex);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_ASSERT(false, "SDCard Driver: OSAL_MUTEX_Unlock failed");
    }
} /* _DRV_SDCARD_AttachDetachTasks */

// *****************************************************************************
/* Function:
    static void _DRV_SDCARD_BufferIOTasks ( SYS_MODULE_OBJ object )

  Summary:
    Maintains the Buffer I/O state machine.

  Description:
    This routine is used to maintain the Buffer I/O internal state machine.

  Precondition:
    The DRV_SDCARD_Initialize routine must have been called for the specified
    SDCARD driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_SDCARD_Initialize)

  Returns:
    None

  Remarks:
    This routine is normally not called directly by an application.  It is
    called by the File system media manager.
*/

static void _DRV_SDCARD_BufferIOTasks
( 
    SYS_MODULE_OBJ object
)
{
    DRV_SDCARD_OBJ              *dObj;
    DRV_SDCARD_XFER_OBJECT      *lObj;
    OSAL_RESULT                 retVal;
    DRV_SPI_BUFFER_EVENT        spiRetVal;    
    DRV_SDCARD_CLIENT_OBJ       *clientObj;

    DRV_SDCARD_EVENT            evtStatus = DRV_SDCARD_EVENT_COMMAND_COMPLETE;

    /* Get the driver object */
    dObj = (DRV_SDCARD_OBJ*)_DRV_SDCARD_INSTANCE_GET(object);

    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_ASSERT(false, "SDCard Driver: OSAL_MUTEX_Lock failed");
    }

    /* Assign a local pointer for faster operation */
    lObj = dObj->localTaskObj;

    /* Check what state we are in, to decide what to do */
    switch (dObj->taskBufferIOState)
    {
        case DRV_SDCARD_BUFFER_IO_CHECK_DEVICE:
            if (dObj->mediaState != SYS_FS_MEDIA_ATTACHED)
            {
                break;
            }

            /* Process reads/writes only if the media is present.
               Intentional fallthrough.
               */

        case DRV_SDCARD_TASK_PROCESS_QUEUE:
            {
                if (dObj->sdState != TASK_STATE_IDLE)
                    break;

                /* Get the first in element from the queue */
                dObj->localTaskObj = _DRV_SDCARD_ReadFromQueue (dObj->queueHandle);
                if (dObj->localTaskObj == NULL)
                {
                    /* If there are no read queued, check for device attach/detach */
                    dObj->taskBufferIOState = DRV_SDCARD_BUFFER_IO_CHECK_DEVICE;
                    break;
                }

                dObj->sdState = TASK_STATE_CARD_COMMAND;

                lObj = dObj->localTaskObj;
                lObj->blockIncr = 1;
                lObj->status = DRV_SDCARD_COMMAND_IN_PROGRESS;

                if (dObj->sdCardType == DRV_SDCARD_MODE_NORMAL)
                {
                    lObj->sectorAddress <<= 9;
                    lObj->blockIncr       = _DRV_SDCARD_MEDIA_BLOCK_SIZE;
                }

                /* Navigate to different cases based on read/write flags */
                if (lObj->readWrite == DRV_SDCARD_TRANSFER_READ)
                {
                    if (lObj->sectorCount == 1)
                    {
                        lObj->command = DRV_SDCARD_READ_SINGLE_BLOCK;
                    }
                    else
                    {
                        lObj->command = DRV_SDCARD_READ_MULTI_BLOCK;
                    }

                    dObj->taskBufferIOState = DRV_SDCARD_TASK_PROCESS_READ;
                }
                else
                {
                    if (lObj->sectorCount == 1)
                    {
                        lObj->command = DRV_SDCARD_WRITE_SINGLE_BLOCK;
                    }
                    else
                    {
                        lObj->command = DRV_SDCARD_WRITE_MULTI_BLOCK;
                    }

                    dObj->taskBufferIOState = DRV_SDCARD_TASK_PROCESS_WRITE;
                }
            }
            break;

        case DRV_SDCARD_TASK_PROCESS_READ:
            {
                /* Note: _DRV_SDCARD_CommandSend() sends 8 SPI clock cycles after
                   getting the response. This meets the NAC Min timing parameter, so
                   we don't need additional clocking here.
                 */
                _DRV_SDCARD_CommandSend (object, lObj->command, lObj->sectorAddress);
                if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE)
                {
                    if (dObj->cmdResponse.response1.byte == 0x00)
                    {
                        dObj->rwTmrFlag = true; 
                        dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_GET_BUSY_STATE;
                    }
                    else
                    {
                        /* Perhaps the card isn't initialized or present */
                        dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_ABORT;
                    }
                }
                else if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_ERROR)
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_ABORT;
                }
            }
            break;

        case DRV_SDCARD_TASK_READ_GET_BUSY_STATE:
            {
                DRV_SPI_BufferAddRead2 (dObj->spiClientHandle, dObj->pCmdResp, 1, 
                            0, dObj, &dObj->spiBufferHandle);

                if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_WAIT_START_TOKEN;

                    if (dObj->rwTmrFlag == true)
                    {
                        /* Kick start a timer with 100ms as the timeout value. If the start token
                           is not received when the timer fires then fail the operation.
                         */
                        dObj->rwTmrHandle = SYS_TMR_DelayMS (_DRV_SDCARD_READ_TIMEOUT_IN_MS);
                        dObj->rwTmrFlag = false;
                    }
                }
                else
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_ABORT;
                }
            }
            break;

        case DRV_SDCARD_TASK_READ_WAIT_START_TOKEN:
            {
                /* In this case, we have already issued the READ_MULTI_BLOCK command
                   to the media, and we need to keep polling the media until it sends
                   us the data start token byte. This could typically take a
                   couple of milliseconds, up to a maximum of 100ms.
                 */
                spiRetVal = DRV_SPI_BufferStatus (dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    if (dObj->pCmdResp[0] == DRV_SDCARD_DATA_START_TOKEN)
                    {
                        DRV_SPI_BufferAddRead2 (dObj->spiClientHandle, lObj->buffer,
                                _DRV_SDCARD_MEDIA_BLOCK_SIZE, 0, dObj, &dObj->spiBufferHandle);

                        dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_CHECK_DATA_READ_CMPLTE;

                        /* Received the start token. Stop the timer */
                        SYS_TMR_CallbackStop (dObj->rwTmrHandle);
                    }
                    else
                    {
                        if (SYS_TMR_DelayStatusGet(dObj->rwTmrHandle))
                        {
                            /* Abort the read operation as the card has failed to send the start token. */
                            dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_ABORT;
                        }
                        else
                        {
                            dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_GET_BUSY_STATE;
                        }
                    }
                }

                if ((spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR) || (dObj->spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID))
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_ABORT;

                    /* Error state. Stop the timer and reset the flag. */
                    SYS_TMR_CallbackStop (dObj->rwTmrHandle);
                }
            }
            break;

        case DRV_SDCARD_TASK_READ_CHECK_DATA_READ_CMPLTE:
            {
                spiRetVal = DRV_SPI_BufferStatus (dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    /* Do a dummy read for the CRC bytes */
                    DRV_SPI_BufferAddRead2 (dObj->spiClientHandle, dObj->pCmdResp, 2,
                            0, dObj, &dObj->spiBufferHandle);
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_BLOCK_CMPLTE_CRC_READ;
                }

                if ((spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR) || 
                        (dObj->spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID))
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_ABORT;
                }
            }
            break;

        case DRV_SDCARD_TASK_READ_BLOCK_CMPLTE_CRC_READ:
            {
                spiRetVal = DRV_SPI_BufferStatus (dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    if (lObj->command == DRV_SDCARD_READ_MULTI_BLOCK)
                    {
                        lObj->sectorCount--;
                        if (lObj->sectorCount == 0)
                        {
                            dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_STOP_TRANSMISSION;
                        }
                        else
                        {
                            lObj->buffer += _DRV_SDCARD_MEDIA_BLOCK_SIZE;

                            dObj->rwTmrFlag = true;
                            dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_GET_BUSY_STATE;
                        }
                    }
                    else
                    {
                        dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_CMPLTE_SEND_CLOCKS;
                    }
                }
                else if (spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_ABORT;
                }
            }
            break;

        case DRV_SDCARD_TASK_READ_CMPLTE_SEND_CLOCKS:
            {
                /* Send 8 clock pulses */
                DRV_SPI_BufferAddRead2 (dObj->spiClientHandle, dObj->pCmdResp, 1,
                        0, dObj, &dObj->spiBufferHandle);
                if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_PROCESS_NEXT;
                }
                else 
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_ABORT;
                }
            }
            break;

        case DRV_SDCARD_TASK_READ_STOP_TRANSMISSION:
            {
                _DRV_SDCARD_CommandSend (object, DRV_SDCARD_STOP_TRANSMISSION, 0);
                if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE)
                {
                    DRV_SPI_BufferAddRead2 (dObj->spiClientHandle, dObj->pCmdResp, 8, 
                            0, dObj, &dObj->spiBufferHandle);
                    if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                    {
                        dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_PROCESS_NEXT;
                    }
                    else
                    {
                        dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_ABORT;
                    }
                }
                else if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_ERROR)
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_ABORT;
                }
            }
            break;

        case DRV_SDCARD_TASK_READ_PROCESS_NEXT:
            {
                spiRetVal = DRV_SPI_BufferStatus (dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    dObj->taskBufferIOState = DRV_SDCARD_PROCESS_NEXT;
                }
                else if (spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_READ_ABORT;
                }
            }
            break;

        case DRV_SDCARD_TASK_PROCESS_WRITE:
            {
                /* Send the write single or write multi command, with the LBA or byte
                   address (depending upon SDHC or standard capacity card) */
                _DRV_SDCARD_CommandSend (object, lObj->command, lObj->sectorAddress);

                if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_IS_COMPLETE)
                {
                    if (dObj->cmdResponse.response1.byte == 0x00)
                    {
                        dObj->taskBufferIOState = DRV_SDCARD_TASK_SEND_WRITE_PACKET;
                    }
                    else
                    {
                        dObj->taskBufferIOState = DRV_SDCARD_TASK_WRITE_ABORT;
                    }
                }
                else if (dObj->cmdState == DRV_SDCARD_CMD_EXEC_ERROR)
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_WRITE_ABORT;
                }
            }
            break;

        case DRV_SDCARD_TASK_SEND_WRITE_PACKET:
            {
                if (lObj->command == DRV_SDCARD_WRITE_MULTI_BLOCK)
                {
                    dObj->pCmdResp[0] = DRV_SDCARD_DATA_START_MULTI_BLOCK_TOKEN;
                }
                else
                {
                    dObj->pCmdResp[0] = DRV_SDCARD_DATA_START_TOKEN;
                }

                DRV_SPI_BufferAddWrite2 (dObj->spiClientHandle, dObj->pCmdResp, 1, 
                        0, dObj, &dObj->spiBufferHandle);
                if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_SEND_RAW_DATA_WRITE;
                }
                else 
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_WRITE_ABORT;
                }
            }
            break;

        case DRV_SDCARD_TASK_SEND_RAW_DATA_WRITE:
            {
                /* Now send a packet of raw data bytes to the media, over SPI.
                   This code directly impacts data throughput in a significant way.
                   Special care should be used to make sure this code is speed optimized.
                 */
                spiRetVal = DRV_SPI_BufferStatus (dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    DRV_SPI_BufferAddWrite2 (dObj->spiClientHandle, lObj->buffer, 512, 
                            0, dObj, &dObj->spiBufferHandle);
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_SEND_CRC;
                }

                if ((spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR) || (dObj->spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID))
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_WRITE_ABORT;
                }
            }
            break;

        case DRV_SDCARD_TASK_SEND_CRC:
            {
                /* Check if we have finished sending a 512 byte block.  If so,
                   need to receive 16-bit CRC, and retrieve the data_response token
                 */
                spiRetVal = DRV_SPI_BufferStatus (dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    /* By default, the media doesn't use CRC unless it is enabled manually
                       during the card initialization sequence.
                     */
                    /* Send 16-bit dummy CRC for the data block that was just sent. */
                    DRV_SPI_BufferAddWrite2 (dObj->spiClientHandle, dObj->pClkPulseData, 2, 
                            0, dObj, &dObj->spiBufferHandle);
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_WRITE_RESPONSE_GET;
                }

                if ((spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR) || (dObj->spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID))
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_WRITE_ABORT;
                }
            }
            break;

        case DRV_SDCARD_TASK_WRITE_RESPONSE_GET:
            {
                spiRetVal = DRV_SPI_BufferStatus (dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    DRV_SPI_BufferAddRead2 (dObj->spiClientHandle, dObj->pCmdResp , 1, 
                            0, dObj, &dObj->spiBufferHandle);
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_WRITE_RESPONSE_GET_TOKEN_MASK;
                }

                if ((spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR) || (dObj->spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID))
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_WRITE_ABORT;
                }
            }
            break;

        case DRV_SDCARD_TASK_WRITE_RESPONSE_GET_TOKEN_MASK:
            {
                spiRetVal = DRV_SPI_BufferStatus (dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    /* Read response token byte from media, mask out top three don't
                       care bits, and check if there was an error */
                    if ((dObj->pCmdResp[0] & DRV_SDCARD_WRITE_RESPONSE_TOKEN_MASK)
                            != DRV_SDCARD_DATA_ACCEPTED)
                    {
                        /* Something went wrong.  Try and terminate as gracefully as
                           possible, so as allow possible recovery */
                        dObj->taskBufferIOState = DRV_SDCARD_TASK_WRITE_ABORT;
                        break;
                    }
                    else
                    {
                        /* The media will now send busy token (0x00) bytes until
                           it is internally ready again (after the block is successfully
                           written and the card is ready to accept a new block).
                         */
                        /* Dummy read to gobble up a byte (ex: to ensure we meet NBR timing parameter  */
                        DRV_SPI_BufferAddRead2 ( dObj->spiClientHandle, dObj->pCmdResp, 1, 
                                0, dObj, &dObj->spiBufferHandle);
                        dObj->taskBufferIOState = DRV_SDCARD_WRITE_CHECK_STILL_BUSY;
                        dObj->rwTmrFlag = true;
                    }
                }

                if ((spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR) || (dObj->spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID))
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_WRITE_ABORT;
                }
            }
            break;

        case DRV_SDCARD_WRITE_CHECK_STILL_BUSY:
            {
                spiRetVal = DRV_SPI_BufferStatus (dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    if (dObj->pCmdResp[0] == 0x00)
                    {
                        /* Still busy */
                        DRV_SPI_BufferAddRead2 (dObj->spiClientHandle, dObj->pCmdResp, 1, 
                                0, dObj, &dObj->spiBufferHandle);
                        if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                        {
                            dObj->taskBufferIOState = DRV_SDCARD_WRITE_CHECK_STILL_BUSY;
                            if (dObj->rwTmrFlag == true)
                            {
                                /* Kick start a timer with 250ms as the timeout value. If the card
                                is still busy at the end of the timeout then abort the operation. */
                                dObj->rwTmrHandle = SYS_TMR_DelayMS (_DRV_SDCARD_WRITE_TIMEOUT_IN_MS);
                                dObj->rwTmrFlag = false;
                            }

                            if (SYS_TMR_DelayStatusGet(dObj->rwTmrHandle))
                            {
                                /* Abort the write operation as the card has failed to complete the operation
                                   in time. */
                                dObj->taskBufferIOState = DRV_SDCARD_TASK_WRITE_ABORT;
                            }
                        }
                        else
                        {
                            dObj->taskBufferIOState = DRV_SDCARD_TASK_WRITE_ABORT;

                            dObj->rwTmrFlag = false;
                            /* Error State. Stop the timer */
                            SYS_TMR_CallbackStop (dObj->rwTmrHandle);
                        }

                        break;
                    }

                    /* The card is out of the busy state. Stop the timer */
                    SYS_TMR_CallbackStop (dObj->rwTmrHandle);
                    dObj->rwTmrFlag = false;

                    /* The media is done and is no longer busy.  Go ahead and
                       either send the next packet of data to the media, or the stop
                       token if we are finished.
                     */
                    if (lObj->command == DRV_SDCARD_WRITE_MULTI_BLOCK)
                    {
                        lObj->sectorCount --;
                        if (lObj->sectorCount == 0)
                        {
                            dObj->pCmdResp[0] = DRV_SDCARD_DATA_STOP_TRAN_TOKEN;
                            DRV_SPI_BufferAddWrite2 (dObj->spiClientHandle, dObj->pCmdResp, 1,
                                    0, dObj, &dObj->spiBufferHandle);

                            dObj->taskBufferIOState = DRV_SDCARD_WRITE_STOP_TRAN_CMPLT;
                        }
                        else
                        {
                            lObj->buffer += _DRV_SDCARD_MEDIA_BLOCK_SIZE;
                            dObj->taskBufferIOState = DRV_SDCARD_TASK_SEND_WRITE_PACKET;
                            break;
                        }
                    }
                    else
                    {
                        /* Send eight clock pulses */
                        DRV_SPI_BufferAddRead2 (dObj->spiClientHandle, dObj->pCmdResp, 1, 
                                0, dObj, &dObj->spiBufferHandle);

                        /* Get the response in another state */
                        dObj->taskBufferIOState = DRV_SDCARD_WRITE_CHECK_8CLOCKS;
                    }
                }

                if ((spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR) || (dObj->spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID))
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_WRITE_ABORT;
                }
            }
            break;

        case DRV_SDCARD_WRITE_STOP_TRAN_CMPLT:
            {
                spiRetVal = DRV_SPI_BufferStatus (dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    /* Send eight clock pulses */
                    DRV_SPI_BufferAddRead2 (dObj->spiClientHandle, dObj->pCmdResp, 1, 
                            0, dObj, &dObj->spiBufferHandle);

                    /* Get the response in another state */
                    dObj->taskBufferIOState = DRV_SDCARD_WRITE_STOP_MULTIPLE;

                    dObj->rwTmrFlag = true;
                }

                if ((spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR) || (dObj->spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID))
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_WRITE_ABORT;
                }
            }
            break;

        case DRV_SDCARD_WRITE_STOP_MULTIPLE:
            {
                spiRetVal = DRV_SPI_BufferStatus (dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    /* Check whether the card is still busy */
                    DRV_SPI_BufferAddRead2 (dObj->spiClientHandle, dObj->pCmdResp, 1, 
                            0, dObj, &dObj->spiBufferHandle);

                    /* Get the response in another state */
                    dObj->taskBufferIOState = DRV_SDCARD_WRITE_STOP_MULTIPLE_STATUS;
                }

                if ((spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR) || (dObj->spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID))
                {
                    SYS_TMR_CallbackStop (dObj->rwTmrHandle);
                    dObj->rwTmrFlag = false;
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_WRITE_ABORT;
                }
            }
            break;

        case DRV_SDCARD_WRITE_STOP_MULTIPLE_STATUS:
            {
                spiRetVal = DRV_SPI_BufferStatus (dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    /* We already sent the stop transmit token for the multi-block write
                       operation.  Now all we need to do, is keep waiting until the card
                       signals it is no longer busy.  Card will keep sending 0x00 bytes
                       until it is no longer busy.
                     */
                    if (dObj->pCmdResp[0] == 0)
                    {
                        dObj->taskBufferIOState = DRV_SDCARD_WRITE_STOP_MULTIPLE;

                        if (dObj->rwTmrFlag == true)
                        {
                            /* Kick start a timer with 250ms as the timeout value. If the start token
                               is not received when the timer fires then fail the operation.
                             */
                            dObj->rwTmrHandle = SYS_TMR_DelayMS (_DRV_SDCARD_WRITE_TIMEOUT_IN_MS);
                            dObj->rwTmrFlag = false;
                        }

                        if (SYS_TMR_DelayStatusGet(dObj->rwTmrHandle))
                        {
                            /* Abort the write operation as the card has failed to complete the operation
                               in time. */
                            dObj->taskBufferIOState = DRV_SDCARD_TASK_WRITE_ABORT;
                        }
                    }
                    else
                    {
                        SYS_TMR_CallbackStop (dObj->rwTmrHandle);
                        dObj->rwTmrFlag = false;
                        dObj->taskBufferIOState = DRV_SDCARD_PROCESS_NEXT;
                    }
                }
                else if (spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    SYS_TMR_CallbackStop (dObj->rwTmrHandle);
                    dObj->rwTmrFlag = false;
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_WRITE_ABORT;
                }
            }
            break;

        case DRV_SDCARD_WRITE_CHECK_8CLOCKS:
            {
                spiRetVal = DRV_SPI_BufferStatus (dObj->spiBufferHandle);
                if (spiRetVal == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    dObj->taskBufferIOState = DRV_SDCARD_PROCESS_NEXT;
                }
                else if (spiRetVal == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    dObj->taskBufferIOState = DRV_SDCARD_TASK_WRITE_ABORT;
                }
            }
            break;

        case DRV_SDCARD_PROCESS_NEXT:
        case DRV_SDCARD_TASK_READ_ABORT:
        case DRV_SDCARD_TASK_WRITE_ABORT:
            {
                /* De select the chip */
                _DRV_SDCARD_CHIP_DESELECT (dObj->chipSelectPort, dObj->chipSelectBitPosition);

                if (dObj->taskBufferIOState == DRV_SDCARD_PROCESS_NEXT)
                {
                    lObj->status = DRV_SDCARD_COMMAND_COMPLETED;
                    evtStatus = DRV_SDCARD_EVENT_COMMAND_COMPLETE;
                }
                else
                {
                    lObj->status = DRV_SDCARD_COMMAND_ERROR_UNKNOWN;
                    evtStatus = DRV_SDCARD_EVENT_COMMAND_ERROR;
                }

                /* Mark the buffer object as free */
                lObj->inUse = false;

                clientObj = (DRV_SDCARD_CLIENT_OBJ*)_DRV_SDCARD_CLIENT_OBJ_GET(lObj->hClient);
                if(clientObj->eventHandler != NULL)
                {
                    /* Call the event handler */
                    clientObj->eventHandler(evtStatus,
                            (DRV_SDCARD_COMMAND_HANDLE)lObj->commandHandle, clientObj->context);
                }

                dObj->sdState = TASK_STATE_IDLE;
                dObj->taskBufferIOState = DRV_SDCARD_BUFFER_IO_CHECK_DEVICE;
                break;
            }
    }

    retVal = OSAL_MUTEX_Unlock(&dObj->mutex);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_ASSERT(false, "SDCard Driver: OSAL_MUTEX_Unlock failed");
    }

} /* DRV_SDCARD_BufferIOTasks */


// *****************************************************************************
// *****************************************************************************
// Section: Driver Interface Function Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_SDCARD_Initialize 
     (
         const SYS_MODULE_INDEX index,
         const SYS_MODULE_INIT  * const init
     );

  Summary:
    Initializes the SD Card driver.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine initializes the SD Card driver, making it ready for clients to
    open and use the driver.

  Precondition:
    None.

  Parameters:
    drvIndex        - Index for the driver instance to be initialized

    init            - Pointer to a data structure containing any data necessary
                      to initialize the driver.

  Returns:
    If successful, returns a valid handle to a driver object. Otherwise, it
    returns SYS_MODULE_OBJ_INVALID.

  Remarks:
    This routine must be called before any other SD Card routine is called.

    This routine should only be called once during system initialization
    unless DRV_SDCARD_Deinitialize is called to deinitialize the driver instance.

    This routine will NEVER block for hardware access. If the operation requires
    time to allow the hardware to reinitialize, it will be reported by the
    DRV_SDCARD_Status operation. The system must use DRV_SDCARD_Status to find out
    when the driver is in the ready state.

*/

SYS_MODULE_OBJ DRV_SDCARD_Initialize 
(
    const SYS_MODULE_INDEX drvIndex,
    const SYS_MODULE_INIT  * const init
)
{
    DRV_SDCARD_INIT *sdcardInit = NULL;
    DRV_SDCARD_OBJ  *dObj 		= NULL;
    uint8_t          index      = 0;

    /* Validate the driver index */
    if ( _DRV_SDCARD_INDEX_GET(drvIndex ) >= DRV_SDCARD_INDEX_COUNT)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    /* Allocate the driver object and set the operation flag to be in use */
    dObj = _DRV_SDCARD_INSTANCE_GET(drvIndex);
    if (dObj->inUse)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    if (init == NULL)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    if (gDrvSDCARDInitCount == 0)
    {
        /* Create the Client Object mutex */
        if (OSAL_MUTEX_Create(&gDrvSDCARDClientMutex) != OSAL_RESULT_TRUE)
        {
            return SYS_MODULE_OBJ_INVALID;
        }
    }
    gDrvSDCARDInitCount++;

    /* Assign to the local pointer the init data passed */
    sdcardInit = (DRV_SDCARD_INIT *)init;

    /* Initialize the driver object's structure members */
    memset (dObj, 0, sizeof(DRV_SDCARD_OBJ));

    if (OSAL_MUTEX_Create(&dObj->mutex) != OSAL_RESULT_TRUE)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    /* Set that this instance of the driver is in use */
    dObj->inUse                     = true;
    dObj->taskState                 = DRV_SDCARD_TASK_OPEN_SPI;
    dObj->drvIndex                  = drvIndex;
    /* Data to send only clock pulses */
	/* SPI driver index */
	dObj->spiIndex                  = sdcardInit->spiIndex;
    /* These pins are I/O pins used by the SD card */
    dObj->chipSelectBitPosition     = sdcardInit->chipSelectBitPosition;
    dObj->chipSelectPort            = sdcardInit->chipSelectPort;
    dObj->writeProtectBitPosition   = sdcardInit->writeProtectBitPosition;
    dObj->writeProtectPort          = sdcardInit->writeProtectPort;

    /* Speed at which SD card is going to operate at. This should be less than
    the maximum SPI frequency and should be supported by the SD card used */
    dObj->sdcardSpeedHz             = sdcardInit->sdcardSpeedHz;
    dObj->taskBufferIOState         = DRV_SDCARD_BUFFER_IO_CHECK_DEVICE ;
    dObj->sdState                   = TASK_STATE_IDLE;

	/* Initialize the SD Card queue */
    dObj->queueHandle               = _DRV_SDCARD_QueueInitialize (drvIndex);

    /* Initialize the SPI Client Data Structure */
    dObj->spiClientData.baudRate    = _DRV_SDCARD_SPI_INITIAL_SPEED;
    dObj->spiClientData.operationStarting = _DRV_SDCARD_SpiBufferEventHandler;
    dObj->spiClientData.operationEnded = _DRV_SDCARD_SpiBufferEventHandler;

    /* Reset the SDCARD attach/detach variables */
    dObj->isAttached = DRV_SDCARD_IS_DETACHED;
    dObj->isAttachedLastStatus = DRV_SDCARD_IS_DETACHED;
    dObj->mediaState = SYS_FS_MEDIA_DETACHED;

    /* Set up the pointers */
    dObj->pCmdResp = &gDrvSDCARDBuffer[drvIndex][0];
    dObj->pCsdData = &gDrvSDCARDCsdData[drvIndex][0];
    dObj->pClkPulseData = &gDrvSDCARDClkPulseData[drvIndex][0];

    /* Setup the Hardware */
    _DRV_SDCARD_SetupHardware (sdcardInit);

    for (index = 0; index < MEDIA_INIT_ARRAY_SIZE; index++)
    {
        dObj->pClkPulseData[index] = 0xFF;
    }

    _DRV_SDCARD_RegisterWithSysFs(drvIndex, drvIndex, sdcardMediaFunctions);

    /* Update the status */
    dObj->status = SYS_STATUS_READY;

    /* Return the object structure */
    return ((SYS_MODULE_OBJ)drvIndex);

} /* DRV_SDCARD_Initialize */

// *****************************************************************************
/* Function:
    void DRV_SDCARD_Reinitialize 
     (
         SYS_MODULE_OBJ          object,
         const SYS_MODULE_INIT * const init
     );

  Summary:
    Reinitializes the driver and refreshes any associated hardware settings.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine reinitializes the driver and refreshes any associated hardware
    settings using the given initialization data, but it will not interrupt any
    ongoing operations.

  Precondition:
    Function DRV_SDCARD_Initialize must have been called before calling this
    routine and a valid SYS_MODULE_OBJ must have been returned.

  Parameters:
    object          - Driver object handle, returned from the DRV_SDCARD_Initialize
                      routine

    init            - Pointer to the initialization data structure

  Returns:
    None

  Remarks:
    This function can be called multiple times to reinitialize the module.

    This operation can be used to refresh any supported hardware registers as
    specified by the initialization data or to change the power state of the
    module.

    This routine will NEVER block for hardware access. If the operation requires
    time to allow the hardware to reinitialize, it will be reported by the
    DRV_SDCARD_Status operation. The system must use DRV_SDCARD_Status to find out
    when the driver is in the ready state.

*/

void DRV_SDCARD_Reinitialize( SYS_MODULE_OBJ        object ,
                              const SYS_MODULE_INIT * const init )
{
    DRV_SDCARD_INIT 			*sdcardInit = NULL;
    DRV_SDCARD_OBJ           	*dObj 		= ( DRV_SDCARD_OBJ* ) NULL;

    /* Validate the driver object */
    SYS_ASSERT ( object != SYS_MODULE_OBJ_INVALID, "Invalid system object handle" );

    /* get the driver object */
    dObj = ( DRV_SDCARD_OBJ* ) _DRV_SDCARD_INSTANCE_GET( object );

    if (OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE) {SYS_ASSERT(false, "SDCard Driver: OSAL_MUTEX_Lock failed");}

    /* Assign to the local pointer the init data passed */
    sdcardInit = ( DRV_SDCARD_INIT * ) init;

	/* Set the current driver state */
    ( dObj->status ) = SYS_STATUS_UNINITIALIZED;

    /* Setup the Hardware */
    _DRV_SDCARD_SetupHardware(sdcardInit);
    /* Update the status */
    ( dObj->status ) = SYS_STATUS_READY;
    if (OSAL_MUTEX_Unlock(&dObj->mutex) != OSAL_RESULT_TRUE) {SYS_ASSERT(false, "SDCard Driver: OSAL_MUTEX_Unlock failed");}

} /* DRV_SDCARD_Reinitialize */


// *****************************************************************************
/* Function:
    void DRV_SDCARD_Deinitialize 
     ( 
         SYS_MODULE_OBJ object 
     );

  Summary:
    Deinitializes the specified instance of the SD Card driver module.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    Deinitializes the specified instance of the SD Card driver module, disabling
    its operation (and any hardware). Invalidates all the internal data.

  Precondition:
    Function DRV_SDCARD_Initialize must have been called before calling this
    routine and a valid SYS_MODULE_OBJ must have been returned.

  Parameters:
    object          - Driver object handle, returned from the
					  DRV_SDCARD_Initialize routine.

  Returns:
    None.

  Remarks:
    Once the Initialize operation has been called, the Deinitialize operation
    must be called before the Initialize operation can be called again.

    This routine will NEVER block waiting for hardware. If the operation
    requires time to allow the hardware to complete, this will be reported by
    the DRV_SDCARD_Status operation.  The system has to use DRV_SDCARD_Status to 
	check if the de-initialization is complete.
*/

void DRV_SDCARD_Deinitialize
(
    SYS_MODULE_OBJ object
)
{
    DRV_SDCARD_OBJ *dObj;

    dObj = (DRV_SDCARD_OBJ*)_DRV_SDCARD_INSTANCE_GET(object);

    dObj->inUse  = false;
    dObj->status = SYS_STATUS_UNINITIALIZED;

    if (OSAL_MUTEX_Delete(&dObj->mutex) != OSAL_RESULT_TRUE) {SYS_ASSERT(false, "SDCard Driver: OSAL_MUTEX_Delete failed");}
    gDrvSDCARDInitCount--;
    if (gDrvSDCARDInitCount == 0)
    {
        /* Remove queued requests */
        _DRV_SDCARD_RemoveQueuedRequests (dObj);

        /* Reset the SDCARD attach/detach variables */
        dObj->isAttached = DRV_SDCARD_IS_DETACHED;
        dObj->isAttachedLastStatus = DRV_SDCARD_IS_DETACHED;
        dObj->mediaState = SYS_FS_MEDIA_DETACHED;

        dObj->sdState = TASK_STATE_IDLE;
        dObj->taskState = DRV_SDCARD_TASK_IDLE;

        /* Close the SPI instance */
        DRV_SPI_Close (dObj->spiClientHandle);
        if (OSAL_MUTEX_Delete(&gDrvSDCARDClientMutex) != OSAL_RESULT_TRUE) {SYS_ASSERT(false, "SDCard Driver: OSAL_MUTEX_Delete failed");}
    }

} /* DRV_SDCARD_Deinitialize */


// *****************************************************************************
/* Function:
    SYS_STATUS DRV_SDCARD_Status 
     ( 
         SYS_MODULE_OBJ object
     );

  Summary:
    Provides the current status of the SD Card driver module.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine provides the current status of the SD Card driver module.

  Precondition:
    Function DRV_SDCARD_Initialize must have been called before calling this
    function

  Parameters:
    object                    - Driver object handle, returned from the
                                DRV_SDCARD_Initialize routine

  Returns:
    SYS_STATUS_READY          - Indicates that the driver is busy with a
                                previous system level operation and cannot start
                                another

                                Note Any value greater than SYS_STATUS_READY is
                                also a normal running state in which the driver
                                is ready to accept new operations.

    SYS_STATUS_BUSY           - Indicates that the driver is busy with a
                                previous system level operation and cannot start
                                another

    SYS_STATUS_ERROR          - Indicates that the driver is in an error state

                                Note:  Any value less than SYS_STATUS_ERROR is
                                also an error state.

    SYS_MODULE_DEINITIALIZED  - Indicates that the driver has been deinitialized

                                Note:  This value is less than SYS_STATUS_ERROR

  Remarks:
    This operation can be used to determine when any of the driver's module
    level operations has completed.

    If the status operation returns SYS_STATUS_BUSY, then a previous operation
    has not yet completed. If the status operation returns SYS_STATUS_READY,
    then it indicates that all previous operations have completed.

    The value of SYS_STATUS_ERROR is negative (-1).  Any value less than that is
    also an error state.

    This routine will NEVER block waiting for hardware.

    If the Status operation returns an error value, the error may be cleared by
    calling the reinitialize operation. If that fails, the deinitialize
    operation will need to be called, followed by the initialize operation to
    return to normal operations.
*/

SYS_STATUS DRV_SDCARD_Status
(
    SYS_MODULE_OBJ object
)
{
    DRV_SDCARD_OBJ *dObj;

    dObj = (DRV_SDCARD_OBJ*)_DRV_SDCARD_INSTANCE_GET(object);

    /* Return the driver status */
    return dObj->status;

} /* DRV_SDCARD_Status */

// *****************************************************************************
/* Function:
    void DRV_SDCARD_Tasks 
     ( 
         SYS_MODULE_OBJ object
     );

  Summary:
    Maintains the driver's state machine.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine is used to maintain the driver's internal state machine.

  Precondition:
    The DRV_SDCARD_Initialize routine must have been called for the specified
    SDCARD driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_SDCARD_Initialize)

  Returns:
    None

  Remarks:
    This routine is normally not called directly by an application. It is
    called by the system's Tasks routine (SYS_Tasks) or by the appropriate raw
    ISR.

    This routine may execute in an ISR context and will never block or access any
    resources that may cause it to block.
*/

void DRV_SDCARD_Tasks 
( 
    SYS_MODULE_OBJ object 
)
{
    _DRV_SDCARD_AttachDetachTasks (object);
    _DRV_SDCARD_BufferIOTasks (object);
}

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_SDCARD_Open 
     (
         const SYS_MODULE_INDEX drvIndex,
         const DRV_IO_INTENT    intent
     );

  Summary:
    Opens the specified SD Card driver instance and returns a handle to it.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine opens the specified SD Card driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver.

  Precondition:
    Function DRV_SDCARD_Initialize must have been called before calling this
    function.

  Parameters:
    drvIndex    - Identifier for the object instance to be opened

    intent      - Zero or more of the values from the enumeration
                  DRV_IO_INTENT "ORed" together to indicate the intended use
                  of the driver

  Returns:
    If successful, the routine returns a valid open-instance handle (a number
    identifying both the caller and the module instance).

    If an error occurs, the return value is DRV_HANDLE_INVALID.

  Remarks:
    The handle returned is valid until the DRV_SDCARD_Close routine is called.

    This routine will NEVER block waiting for hardware.

    If the DRV_IO_INTENT_BLOCKING is requested and the driver was built
    appropriately to support blocking behavior, then other client-level
    operations may block waiting on hardware until they are complete.

    If the requested intent flags are not supported, the routine will return
    DRV_HANDLE_INVALID.
*/

DRV_HANDLE DRV_SDCARD_Open
(
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
)
{
    uint32_t iClient;
    DRV_SDCARD_CLIENT_OBJ *clientObj; 
    DRV_SDCARD_OBJ *dObj;
    OSAL_RESULT retVal;
   
    /* Validate the driver index */
    if (drvIndex >= DRV_SDCARD_INDEX_COUNT)
    {
        return DRV_HANDLE_INVALID;
    }

    /* Allocate the driver object and set the operation flag to be in use */
    dObj = (DRV_SDCARD_OBJ*)_DRV_SDCARD_INSTANCE_GET(drvIndex);

    /* Acquire the driver object mutex */
    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        return DRV_HANDLE_INVALID;
    }

    if (dObj->status != SYS_STATUS_READY)
    {
        OSAL_MUTEX_Unlock(&dObj->mutex);
        return DRV_HANDLE_INVALID;
    }

    /* The SDCARD driver doesn't support blocking intent. Flag error. */
    /* Flag error if max number of clients are already open.
     * Flag error if driver was already opened exclusively.
     * Flag error if the client is trying to open the driver exclusively
     * when it is already open by other clients in non-exclusive mode.
     * */
    if ((DRV_IO_ISBLOCKING(ioIntent)) ||
        (dObj->numClients == DRV_SDCARD_CLIENTS_NUMBER) ||
        (dObj->isExclusive) || 
        ((dObj->numClients > 0) && DRV_IO_ISEXCLUSIVE(ioIntent)))
    {
        OSAL_MUTEX_Unlock(&dObj->mutex);
        return DRV_HANDLE_INVALID;
    }

    retVal = OSAL_MUTEX_Lock(&gDrvSDCARDClientMutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        OSAL_MUTEX_Unlock(&dObj->mutex);
		return DRV_HANDLE_INVALID;
    }

    /* Find available slot in array of client objects */
    clientObj = (DRV_SDCARD_CLIENT_OBJ *)gDrvSDCARDClientObj;
    for (iClient = 0; iClient < (DRV_SDCARD_CLIENTS_NUMBER * DRV_SDCARD_INSTANCES_NUMBER); iClient++)
    {
        if (!clientObj->inUse)
        {
            clientObj->inUse        = true;
            clientObj->eventHandler = NULL;
            clientObj->context      = 0;
            clientObj->drvIndex     = drvIndex;
            clientObj->driverObject = (DRV_SDCARD_OBJ_HANDLE)dObj;
            clientObj->intent       = ioIntent;

            dObj->numClients++;

            if (DRV_IO_ISEXCLUSIVE(ioIntent))
            {
                dObj->isExclusive = true;
            }

            OSAL_MUTEX_Unlock(&gDrvSDCARDClientMutex);
            OSAL_MUTEX_Unlock(&dObj->mutex);

            return ((DRV_HANDLE)iClient);
        }

        clientObj++;
    }

    OSAL_MUTEX_Unlock(&gDrvSDCARDClientMutex);
    OSAL_MUTEX_Unlock(&dObj->mutex);

    return DRV_HANDLE_INVALID;
} /* DRV_SDCARD_Open */


// *****************************************************************************
/* Function:
    void DRV_SDCARD_Close
     (
         DRV_HANDLE handle
     );

  Summary:
    Closes an opened-instance of the SD Card driver.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine closes an opened-instance of the SD Card driver, invalidating
    the handle.

  Precondition:
    The DRV_SDCARD_Initialize routine must have been called for the specified
    SD Card driver instance.

    DRV_SDCARD_Open must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Remarks:
    After calling this routine, the handle passed in "handle" must not be used
    with any of the remaining driver routines.  A new handle must be obtained by
    calling DRV_SDCARD_Open before the caller may use the driver again.

    If DRV_IO_INTENT_BLOCKING was requested and the driver was built
    appropriately to support blocking behavior call may block until the
    operation is complete.

    If DRV_IO_INTENT_NON_BLOCKING request the driver client can call the
    DRV_SDCARD_Status operation to find out when the module is in
    the ready state (the handle is no longer valid).

    Note:
    Usually there is no need for the driver client to verify that the Close
    operation has completed.
*/
void DRV_SDCARD_Close( DRV_HANDLE handle )
{
    /* Multi client variables are removed from single client builds. */
    DRV_SDCARD_CLIENT_OBJ   *clientObj =
        ( DRV_SDCARD_CLIENT_OBJ* ) _DRV_SDCARD_CLIENT_OBJ_GET ( handle );
     DRV_SDCARD_OBJ* dObj = ( DRV_SDCARD_OBJ* )clientObj->driverObject;
     if (OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE) {SYS_ASSERT(false, "SDCard Driver: OSAL_MUTEX_Lock failed");}

    /* Update the Client Status */
    clientObj->inUse = false;

    dObj->numClients--;
    if (OSAL_MUTEX_Unlock(&dObj->mutex) != OSAL_RESULT_TRUE) {SYS_ASSERT(false, "SDCard Driver: OSAL_MUTEX_Unlock failed");}

} /* DRV_SDCARD_Close */

// *****************************************************************************
/* Function:
    void DRV_SDCARD_Read
    (
        const DRV_HANDLE handle,
        DRV_SDCARD_COMMAND_HANDLE * commandHandle,
        void * targetBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Reads blocks of data from the specified block address of the SD Card.

  Description:
    This function schedules a non-blocking read operation for reading blocks
    of data from the SD Card. The function returns with a valid buffer handle
    in the commandHandle argument if the read request was scheduled successfully.
    The function adds the request to the hardware instance queue and returns
    immediately. While the request is in the queue, the application buffer is
    owned by the driver and should not be modified. The function returns 
    DRV_SDCARD_COMMAND_HANDLE_INVALID in the commandHandle argument under the 
    following circumstances:
    - if the driver handle is invalid
    - if the target buffer pointer is NULL
    - if the number of blocks to be read is zero or more than the actual number
      of blocks available
    - if a buffer object could not be allocated to the request
    - if the client opened the driver in write only mode

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_SDCARD_EVENT_COMMAND_COMPLETE event if the
    buffer was processed successfully or DRV_SDCARD_EVENT_COMMAND_ERROR
    event if the buffer was not processed successfully.

  Precondition:
    The DRV_SDCARD_Initialize routine must have been called for the specified SDCARD 
    driver instance.

    DRV_SDCARD_Open must have been called with DRV_IO_INTENT_READ or 
    DRV_IO_INTENT_READWRITE as the ioIntent to obtain a valid opened device handle.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open function

    commandHandle - Pointer to an argument that will contain the return buffer
                    handle
                   
    targetBuffer  - Buffer into which the data read from the SD Card will be placed

    blockStart    - Start block address of the SD Card from where the read should begin.

    nBlock        - Total number of blocks to be read.

  Returns:
    The buffer handle is returned in the commandHandle argument. It will be
    DRV_SDCARD_COMMAND_HANDLE_INVALID if the request was not successful.

  Remarks:
    None.
*/

void DRV_SDCARD_Read 
(
    const DRV_HANDLE handle,
    DRV_SDCARD_COMMAND_HANDLE *commandHandle,
    void *targetBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_SDCARD_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_SDCARD_CLIENT_OBJ     *clientObj;
    DRV_SDCARD_OBJ            *dObj;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_SDCARD_COMMAND_HANDLE_INVALID;

    clientObj = _DRV_SDCARD_ClientHandleValidate (handle);
    if (clientObj == NULL)
        return;

    if (!(clientObj->intent & DRV_IO_INTENT_READ))
        return;

    if ((targetBuffer == NULL) || (nBlock == 0))
        return;

    dObj = (DRV_SDCARD_OBJ*)clientObj->driverObject;

    if (((blockStart + nBlock) > dObj->mediaGeometryTable[GEOMETRY_TABLE_READ_ENTRY].numBlocks))
        return;

    if (OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE) 
        return;

    /* Add to the queue specifying the type as READ */
    *tempHandle1 = _DRV_SDCARD_AddToQueue (handle, DRV_SDCARD_TRANSFER_READ, targetBuffer, blockStart, nBlock);

    OSAL_MUTEX_Unlock(&dObj->mutex);
}

// *****************************************************************************
/* Function:
    void DRV_SDCARD_Write
    (
        const DRV_HANDLE handle,
        DRV_SDCARD_COMMAND_HANDLE * commandHandle,
        void * sourceBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Writes blocks of data starting at the specified address of the SD Card.

  Description:
    This function schedules a non-blocking write operation for writing blocks
    of data to the SD Card. The function returns with a valid buffer handle
    in the commandHandle argument if the write request was scheduled successfully.
    The function adds the request to the hardware instance queue and returns
    immediately. While the request is in the queue, the application buffer is
    owned by the driver and should not be modified. The function returns 
    DRV_SDCARD_COMMAND_HANDLE_INVALID in the commandHandle argument under the 
    following circumstances:
    - if a buffer object could not be allocated to the request
    - if the source buffer pointer is NULL
    - if the client opened the driver for read only
    - if the number of blocks to be written is either zero or more than the number
      of blocks actually available
    - if the write queue size is full or queue depth is insufficient
    - if the driver handle is invalid 

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_SDCARD_EVENT_COMMAND_COMPLETE event if the
    buffer was processed successfully or DRV_SDCARD_EVENT_COMMAND_ERROR
    event if the buffer was not processed successfully.

  Precondition:
    The DRV_SDCARD_Initialize() routine must have been called for the specified
    SDCARD driver instance.

    DRV_SDCARD_Open() routine must have been called to obtain a valid opened device
    handle. DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have been specified
    as a parameter to this routine.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open function

    commandHandle - Pointer to an argument that will contain the return buffer
                    handle
                   
    sourceBuffer  - The source buffer containing data to be programmed to the SD Card.

    blockStart    - Start block address of SD Card where the writes should begin. 

    nBlock        - Total number of blocks to be written. 

  Returns:
    The buffer handle is returned in the commandHandle argument. It will be
    DRV_SDCARD_COMMAND_HANDLE_INVALID if the request was not successful.

  Remarks:
    None.
*/

void DRV_SDCARD_Write
(
    const DRV_HANDLE handle,
    DRV_SDCARD_COMMAND_HANDLE *commandHandle,
    void *sourceBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_SDCARD_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_SDCARD_CLIENT_OBJ     *clientObj;
    DRV_SDCARD_OBJ            *dObj;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_SDCARD_COMMAND_HANDLE_INVALID;

    clientObj = _DRV_SDCARD_ClientHandleValidate (handle);
    if (clientObj == NULL)
        return;

    if (!(clientObj->intent & DRV_IO_INTENT_WRITE))
        return;

    if ((sourceBuffer == NULL) || (nBlock == 0))
        return;

    dObj = (DRV_SDCARD_OBJ*)clientObj->driverObject;

    if (((blockStart + nBlock) > dObj->mediaGeometryTable[GEOMETRY_TABLE_WRITE_ENTRY].numBlocks))
        return;

    /* Return error if the card is write protected */
    if (dObj->isWriteProtected)
        return;

    if (OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
        return;

    /* Add to the queue specifying the type as WRITE */
    *tempHandle1 = _DRV_SDCARD_AddToQueue (handle, DRV_SDCARD_TRANSFER_WRITE, sourceBuffer, blockStart, nBlock);

    OSAL_MUTEX_Unlock(&dObj->mutex);
}

// *****************************************************************************
/* Function:
    DRV_SDCARD_COMMAND_STATUS DRV_SDCARD_CommandStatus
    (
        const DRV_HANDLE handle, 
        const DRV_SDCARD_COMMAND_HANDLE commandHandle
    );

  Summary:
    Gets the current status of the command.

  Description:
    This routine gets the current status of the command. The application must use
    this routine where the status of a scheduled command needs to be polled on. The
    function may return DRV_SDCARD_COMMAND_HANDLE_INVALID in a case where the command
    handle has expired. A command handle expires when the internal buffer object
    is re-assigned to another read or write request. It is recommended that this
    function be called regularly in order to track the command status correctly.

    The application can alternatively register an event handler to receive read or
    write operation completion events.

  Preconditions:
    The DRV_SDCARD_Initialize() routine must have been called.

    The DRV_SDCARD_Open() must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    A DRV_SDCARD_COMMAND_STATUS value describing the current status of the command.
    Returns DRV_SDCARD_COMMAND_HANDLE_INVALID if the client handle or the command
    handle is not valid.

  Remarks:
    This routine will not block for hardware access and will immediately return
    the current status.
*/

DRV_SDCARD_COMMAND_STATUS DRV_SDCARD_CommandStatus
(
    const DRV_HANDLE handle, 
    const DRV_SDCARD_COMMAND_HANDLE commandHandle
)
{
    uint16_t index;
    DRV_SDCARD_CLIENT_OBJ *clientObj;
    DRV_SDCARD_QUEUE_OBJECT *qObj;

    clientObj = _DRV_SDCARD_ClientHandleValidate (handle);
    if (clientObj == NULL)
    {
        return DRV_SDCARD_COMMAND_ERROR_UNKNOWN;
    }

    qObj = (DRV_SDCARD_QUEUE_OBJECT *)&gDrvSDCARDQueueObj[clientObj->drvIndex];

    index = commandHandle & 0xFFFF;

    if (qObj->bufferPool[index].commandHandle != commandHandle)
    {
        return DRV_SDCARD_COMMAND_COMPLETED;
    }

    return qObj->bufferPool[index].status;
}

// *****************************************************************************
/* Function:
    SYS_FS_MEDIA_GEOMETRY * DRV_SDCARD_GeometryGet
    (
        const DRV_HANDLE handle
    );

  Summary:
    Returns the geometry of the device.

  Description:
    This API gives the following geometrical details of the SD Card.
    - Media Property
    - Number of Read/Write/Erase regions in the SD Card
    - Number of Blocks and their size in each region of the device

  Precondition:
    The DRV_SDCARD_Initialize() routine must have been called for the
    specified SDCARD driver instance.

    The DRV_SDCARD_Open() routine must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    SYS_FS_MEDIA_GEOMETRY - Pointer to structure which holds the media geometry information.

  Remarks:
    None.
*/

SYS_FS_MEDIA_GEOMETRY * DRV_SDCARD_GeometryGet
(
    const DRV_HANDLE handle
)
{
    DRV_SDCARD_CLIENT_OBJ *clientObj;
    DRV_SDCARD_OBJ        *dObj;

    clientObj = _DRV_SDCARD_ClientHandleValidate (handle);
    if (clientObj != NULL)
    {
        dObj = (DRV_SDCARD_OBJ*)clientObj->driverObject;
        return (&dObj->mediaGeometryObj);
    }

    return NULL;
}

// *****************************************************************************
/* Function:
    void DRV_SDCARD_EventHandlerSet
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
    the driver to call back when queued operation has completed.  
    When a client queues a request for a read or a write operation, it is provided 
    with a handle identifying the buffer that was added to the driver's buffer queue.
    The driver will pass this handle back to the client by calling "eventHandler"
    function when the queued operation has completed.
    
    The event handler should be set before the client performs any read or write
    operations that could generate events. The event handler once set, persists
    until the client closes the driver or sets another event handler (which could
    be a "NULL" pointer to indicate no callback).

  Precondition:
    The DRV_SDCARD_Initialize() routine must have been called for the specified
    SDCARD driver instance.

    The DRV_SDCARD_Open() routine must have been called to obtain a valid opened
    device handle.

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

  Remarks:
    If the client does not want to be notified when the queued operation
    has completed, it does not need to register a callback.
*/

void DRV_SDCARD_EventHandlerSet
(
    const DRV_HANDLE handle,
    const void * eventHandler,
    const uintptr_t context
)
{
    DRV_SDCARD_CLIENT_OBJ *clientObj;

    clientObj = _DRV_SDCARD_ClientHandleValidate (handle);
    if (clientObj != NULL)
    {
        /* Set the event handler */
        clientObj->eventHandler = eventHandler;
        clientObj->context = context;
    }
}


// *****************************************************************************
/* Function:
    bool DRV_SDCARD_IsAttached
    ( 
        const DRV_HANDLE handle 
    );

  Summary:
    Returns the physical attach status of the SD Card.

  Description:
    This function returns the physical attach status of the SD Card.

  Precondition:
    The DRV_SDCARD_Initialize() routine must have been called for the specified 
    SDCARD driver instance.

    The DRV_SDCARD_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    Returns false if the handle is invalid otherwise returns the attach status
    of the SD Card. Returns true if the SD Card is attached and initialized by the
    SDCARD driver otherwise returns false.

  Remarks:
    None.
*/

bool DRV_SDCARD_IsAttached
(
    const DRV_HANDLE handle
)
{
    DRV_SDCARD_CLIENT_OBJ *clientObj;
    DRV_SDCARD_OBJ *dObj;

    clientObj = _DRV_SDCARD_ClientHandleValidate (handle);
    if (clientObj == NULL)
        return false;

    dObj = (DRV_SDCARD_OBJ*)clientObj->driverObject;
    return dObj->mediaState;
}
    
// *****************************************************************************
/* Function:
    bool DRV_SDCARD_IsWriteProtected
    ( 
        const DRV_HANDLE handle 
    );

  Summary:
    Returns the write protect status of the SDCARD.

  Description:
    This function returns the physical attach status of the SDCARD. This function 
    returns true if the SD Card is write protected otherwise it returns false.

  Precondition:
    The DRV_SDCARD_Initialize() routine must have been called for the specified 
    SDCARD driver instance.

    The DRV_SDCARD_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    Returns true if the attached SD Card is write protected.
    Returns false if the handle is not valid, or if the SD Card is not write protected.

  Remarks:
    None.
*/

bool DRV_SDCARD_IsWriteProtected
(
    const DRV_HANDLE handle
)
{
    DRV_SDCARD_CLIENT_OBJ *clientObj;
    DRV_SDCARD_OBJ *dObj;

    clientObj = _DRV_SDCARD_ClientHandleValidate (handle);
    if (clientObj == NULL)
        return false;

    dObj = (DRV_SDCARD_OBJ*)clientObj->driverObject;
    if (dObj->mediaState == SYS_FS_MEDIA_DETACHED)
        return false;

    return dObj->isWriteProtected;
}

/*******************************************************************************
End of File
*/

