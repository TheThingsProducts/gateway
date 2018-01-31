/*******************************************************************************
  NVM Driver Local Data Structures

  Company:
    Microchip Technology Inc.

  File Name:
    drv_nvm_local.h

  Summary:
    NVM driver local declarations and definitions

  Description:
    This file contains the timer driver's local declarations and definitions.
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

#ifndef _DRV_NVM_LOCAL_H
#define _DRV_NVM_LOCAL_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/nvm/drv_nvm.h"
#include "driver/nvm/src/drv_nvm_variant_mapping.h"

// *****************************************************************************
// *****************************************************************************
// Section: Version Numbers
// *****************************************************************************
// *****************************************************************************
/* Versioning of the driver */

// *****************************************************************************
/* NVM Driver Version Macros

  Summary:
    NVM driver version

  Description:
    These constants provide NVM driver version information. The driver
    version is
    DRV_NVM_VERSION_MAJOR.DRV_NVM_VERSION_MINOR.DRV_NVM_VERSION_PATCH.
    It is represented in DRV_NVM_VERSION as
    MAJOR *10000 + MINOR * 100 + PATCH, so as to allow comparisons.
    It is also represented in string format in DRV_NVM_VERSION_STR.
    DRV_NVM_TYPE provides the type of the release when the release is alpha
    or beta. The interfaces DRV_NVM_VersionGet() and
    DRV_NVM_VersionStrGet() provide interfaces to the access the version
    and the version string.

  Remarks:
    Modify the return value of DRV_NVM_VersionStrGet and the
    DRV_NVM_VERSION_MAJOR, DRV_NVM_VERSION_MINOR,
    DRV_NVM_VERSION_PATCH and DRV_NVM_VERSION_TYPE
*/

#define _DRV_NVM_VERSION_MAJOR         0
#define _DRV_NVM_VERSION_MINOR         2
#define _DRV_NVM_VERSION_PATCH         0
#define _DRV_NVM_VERSION_TYPE          "Alpha"
#define _DRV_NVM_VERSION_STR           "0.2.0 Alpha"

// *****************************************************************************
/* NVM Flash Read/Write/Erase Region Index Numbers

  Summary:
    NVM Geometry Table Index definitions.

  Description:
    These constants provide NVM Geometry Table index definitions.

  Remarks:
    None
*/
#define GEOMETRY_TABLE_READ_ENTRY   (0)
#define GEOMETRY_TABLE_WRITE_ENTRY  (1)
#define GEOMETRY_TABLE_ERASE_ENTRY  (2)

/*****************************************************************************
 * If the NVM needs to be controlled by media manager, then declare the
 * following as 1. Otherwise, declare as 0.
 *
 *****************************************************************************/

// *****************************************************************************
/* NVM Driver Buffer Handle Macros

  Summary:
    NVM driver Buffer Handle Macros

  Description:
    Buffer handle related utility macros. NVM driver buffer handle is a 
    combination of buffer token and the buffer object index. The buffertoken
    is a 16 bit number that is incremented for every new write or erase request
    and is used along with the buffer object index to generate a new buffer 
    handle for every request.

  Remarks:
    None
*/

#define _DRV_NVM_BUF_TOKEN_MAX         (0xFFFF)
#define _DRV_NVM_MAKE_HANDLE(token, index) ((token) << 16 | (index))
#define _DRV_NVM_UPDATE_BUF_TOKEN(token) \
{ \
    (token)++; \
    (token) = ((token) == _DRV_NVM_BUF_TOKEN_MAX) ? 0: (token); \
}

// *****************************************************************************
// *****************************************************************************
// Section: Local Data Type Definitions
// *****************************************************************************
// *****************************************************************************

extern SYS_FS_MEDIA_REGION_GEOMETRY *gNVMGeometryTable;
/****************************************
 * This enumeration defines the possible 
 * NVM driver operations.
 ****************************************/

typedef enum
{
    /* Write Operation to be performed on the buffer*/
    DRV_NVM_BUFFER_FLAG_WRITE                 /*DOM-IGNORE-BEGIN*/ = 1 << 0/*DOM-IGNORE-END*/,

    /* Read Operation to be performed on the buffer */
    DRV_NVM_BUFFER_FLAG_READ                  /*DOM-IGNORE-BEGIN*/ = 1 << 1/*DOM-IGNORE-END*/,

    /* Erase Operation to be performed on the buffer */
    DRV_NVM_BUFFER_FLAG_ERASE                  /*DOM-IGNORE-BEGIN*/ = 1 << 2/*DOM-IGNORE-END*/,

    /* Erase and write operation */
    DRV_NVM_BUFFER_FLAG_ERASE_WRITE             /*DOM-IGNORE-BEGIN*/ = 1 << 3/*DOM-IGNORE-END*/

} DRV_NVM_BUFFER_FLAGS;

/*******************************************
 * NVM Driver Buffer Object that services
 * a driver request.
 ******************************************/

typedef struct _DRV_NVM_BUFFER_OBJECT
{
    /* True if object is allocated */
    bool inUse;

    /* Client source or destination pointer */
    uint8_t * appDataPointer;

    /* Flash memory pointer */
    uint8_t * flashMemPointer;

    /* Size of the request */
    uint32_t size;

    /* Client that owns this buffer */
    DRV_HANDLE hClient;

    /* Present status of this command */
    DRV_NVM_COMMAND_STATUS status;

    /* Type of NVM driver operation */
    DRV_NVM_BUFFER_FLAGS flag;

    /* Pointer to the next buffer in the queue */
    struct _DRV_NVM_BUFFER_OBJECT * next;

    /* Pointer to the previous buffer in the queue */
    struct _DRV_NVM_BUFFER_OBJECT * previous;

    /* Current command handle of this buffer object */
    DRV_NVM_COMMAND_HANDLE commandHandle;

    /* Number of pending blocks to be procssed */
    uint32_t nBlocksPending;

    /* Size of each block in this request */
    uint32_t blockSize;

} DRV_NVM_BUFFER_OBJECT;

typedef enum 
{
    DRV_NVM_ERASE_WRITE_STEP_ERASE_COMPLETE = 0x1,
    DRV_NVM_ERASE_WRITE_STEP_WRITE_PAGE = 0x2,
    DRV_NVM_ERASE_WRITE_STEP_ERASE_NEXT_PAGE = 0x4
            
} DRV_NVM_ERASE_WRITE_STEP;

/*******************************************
 * Internal operation type 
 ******************************************/
typedef enum {

    _DRV_WORD_PROGRAM_OPERATION = 0x1,
    _DRV_ROW_PROGRAM_OPERATION = 0x3,
    _DRV_PAGE_ERASE_OPERATION = 0x4,
    _DRV_FLASH_ERASE_OPERATION = 0x5

} _DRV_NVM_OPERATION_MODE;

/**************************************
 * NVM Driver Hardware Instance Object
 **************************************/
typedef struct
{
    /* The module index associated with the object*/
    NVM_MODULE_ID moduleId;

    /* Object Index */
    SYS_MODULE_INDEX objIndex;

    /* The buffer Q for the write operations */
    DRV_NVM_BUFFER_OBJECT * writeEraseQ;

    /* The status of the driver */
    SYS_STATUS status;

    /* Flag to indicate in use  */
    bool inUse;

    /* Flag to indicate that SAMPLE is used in exclusive access mode */
    bool isExclusive;

    /* Number of clients connected to the hardware instance */
    uint8_t numClients;

    /* Interrupt Source for TX Interrupt */
    INT_SOURCE interruptSource;

    /* Interrupt status flag*/
    bool intStatus;

    /* Erase page buffer */
    uint8_t * eraseBuffer;

    /* Current write buffer address in case of
     * erase and write operation */
    uint32_t eraseWriteStartAddress;

    /* No of erase and write blocks pending */
    uint32_t    nRowsPending;

    /* Blocks overlayed in this page */
    uint8_t * appDataMemory;

    /* Erase Write Step */
    DRV_NVM_ERASE_WRITE_STEP eraseWriteStep;

    /* Block start address */
    uint32_t blockStartAddress;

} DRV_NVM_OBJECT;

/**************************************
 * NVM Driver Client 
 **************************************/
typedef struct _DRV_NVM_CLIENT_OBJ_STRUCT
{
    /* The hardware instance object associate with the client */
    void * driverObj;

    /* Status of the client object */
    SYS_STATUS sysStatus;

    /* The intent with which the client was opened */
    DRV_IO_INTENT intent;

    /* Flag to indicate in use */
    bool inUse;

    /* Client specific event handler */
    DRV_NVM_EVENT_HANDLER  eventHandler;

    /* Client specific context */
    uintptr_t context;

} DRV_NVM_CLIENT_OBJECT;

/****************************************
 * Local functions
 *****************************************/
void _DRV_NVM_UnlockSequence (NVM_MODULE_ID plibId,DRV_NVM_OBJECT * dObj,_DRV_NVM_OPERATION_MODE mode);
DRV_NVM_CLIENT_OBJECT * _DRV_NVM_ClientHandleValidate(DRV_HANDLE handle);
void  _DRV_NVM_EraseBufferObjProcess(DRV_NVM_OBJECT * dObj, DRV_NVM_BUFFER_OBJECT * bufferObj);
DRV_NVM_COMMAND_HANDLE _DRV_NVM_BlockOperation (DRV_HANDLE handle,uint8_t * sourceBuffer, uint32_t blockStart,
                                               uint32_t nBlock, DRV_NVM_BUFFER_FLAGS flag,
                                               uint32_t blockSize);
extern DRV_NVM_BUFFER_OBJECT gDrvNVMBufferObject[];
extern uint16_t gDrvNVMBufferToken;

#endif //#ifndef _DRV_NVM_LOCAL_H

/*******************************************************************************
 End of File
*/

