/*******************************************************************************
  NVM Driver Local Data Structures for static implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_nvm_static_local.h

  Summary:
    NVM Driver Local Data Structures for static implementation

  Description:
    Driver Local Data Structures for static implementation
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
//DOM-IGNORE-END

#ifndef DRV_NVM_STATIC_LOCAL_H
#define DRV_NVM_STATIC_LOCAL_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/nvm/drv_nvm.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

<#macro DRV_NVM_LOCAL_DATA DRV_INSTANCE>
// *****************************************************************************
// *****************************************************************************
// Section: Data Type Definitions
// *****************************************************************************
// *****************************************************************************

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


/****************************************
 * Virtual address to physical address mapping
 ****************************************/

#define _DRV_NVM_KVA_TO_PA(address)          (address & 0x1FFFFFFF)


/****************************************
 * Parameter validation macro
 ****************************************/

#define _DRV_NVM_VALIDATE_EXPR(y, z) if ((y)) return (z);


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
 * Erase Write Operation steps
 ******************************************/

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


/**************************************
 * NVM Driver Hardware Instance Object
 **************************************/
typedef struct
{

    /* Interrupt status flag*/
    bool intStatus;

    /* Erase page buffer */
    uint8_t * eraseBuffer;

    /* The buffer Q for the write operations */
    DRV_NVM_BUFFER_OBJECT * writeEraseQ;

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

    /* Client specific event handler */
    DRV_NVM_EVENT_HANDLER  eventHandler;

    /* Client specific context */
    uintptr_t context;
<#if CONFIG_USE_3RDPARTY_RTOS>

    /* NVM Hardware instance Object Mutex */
    OSAL_MUTEX_DECLARE(nvmInstanceObjMutex);
</#if>

} DRV_NVM_OBJECT;


/****************************************
 * Local functions
 *****************************************/
void _DRV_NVM_UnlockSequence (_DRV_NVM_OPERATION_MODE mode);
void _DRV_NVM_WriteBufferObjProcess(DRV_NVM_BUFFER_OBJECT * bufferObj);
void  _DRV_NVM_EraseBufferObjProcess(DRV_NVM_BUFFER_OBJECT * bufferObj);
DRV_NVM_COMMAND_HANDLE _DRV_NVM_BlockOperation (uint8_t * sourceBuffer, uint32_t blockStart,
                                                uint32_t nBlock, DRV_NVM_BUFFER_FLAGS flag,
                                                uint32_t blockSize);

</#macro>
<#if CONFIG_USE_DRV_NVM == true>
<@DRV_NVM_LOCAL_DATA
DRV_INSTANCE="0"
/>
</#if>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

#endif //#ifndef DRV_NVM_STATIC_LOCAL_H

/*******************************************************************************
 End of File
*/

