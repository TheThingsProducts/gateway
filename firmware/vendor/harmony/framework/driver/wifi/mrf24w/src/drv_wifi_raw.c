/*******************************************************************************
  MRF24WG RAW Driver

  File Name:
    drv_wifi_raw.c

  Summary:
    MRF24WG RAW Driver

  Description:
    -Provides access to MRF24WG Wi-Fi controller
    -Reference: MRF24WG Datasheet, IEEE 802.11 Standard
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

/******************/
/*    INCLUDES    */
/******************/
#include "drv_wifi_priv.h"

#include "drv_wifi_debug_output.h"
#include "drv_wifi_raw.h"

/*****************/
/*    DEFINES    */
/*****************/
// RAW register masks
#define WF_RAW_STATUS_REG_BUSY_MASK ((uint16_t)(0x0001))
#define WF_RAW_STATUS_REG_ERROR_MASK ((uint16_t)(0x0002))

#define NUM_RAW_WINDOWS (6) /* only using raw windows 0 thru 4 */

/**************************/
/*    LOCAL DATA TYPES    */
/**************************/
typedef struct
{
    uint16_t rawId;           // RAW ID of engine we are waiting on
    uint8_t rawIntMask;       // mask to see if correct raw ID move completed
    bool intDisabled;         // saved state of EINT prior to Raw move
    uint32_t startTickCount;  // start time of raw move
    uint32_t maxAllowedTicks; // max allowed wait time for a raw move
} RAW_MOVE_WAIT_STATE;

/********************************/
/*    LOCAL GLOBAL VARIABLES    */
/********************************/
/* raw registers for each raw window being used */
static const uint8_t s_RawIndexReg[NUM_RAW_WINDOWS] = {RAW_0_INDEX_REG, RAW_1_INDEX_REG, RAW_2_INDEX_REG, RAW_3_INDEX_REG, RAW_4_INDEX_REG, RAW_5_INDEX_REG};
static const uint8_t s_RawStatusReg[NUM_RAW_WINDOWS] = {RAW_0_STATUS_REG, RAW_1_STATUS_REG, RAW_2_STATUS_REG, RAW_3_STATUS_REG, RAW_4_STATUS_REG, RAW_5_STATUS_REG};
static const uint16_t s_RawCtrl0Reg[NUM_RAW_WINDOWS] = {RAW_0_CTRL_0_REG, RAW_1_CTRL_0_REG, RAW_2_CTRL_0_REG, RAW_3_CTRL_0_REG, RAW_4_CTRL_0_REG, RAW_5_CTRL_0_REG};
static const uint16_t s_RawCtrl1Reg[NUM_RAW_WINDOWS] = {RAW_0_CTRL_1_REG, RAW_1_CTRL_1_REG, RAW_2_CTRL_1_REG, RAW_3_CTRL_1_REG, RAW_4_CTRL_1_REG, RAW_5_CTRL_1_REG};
static const uint16_t s_RawDataReg[NUM_RAW_WINDOWS] = {RAW_0_DATA_REG, RAW_1_DATA_REG, RAW_2_DATA_REG, RAW_3_DATA_REG, RAW_4_DATA_REG, RAW_5_DATA_REG};

/* interrupt mask for each raw window; note that raw0 and raw1 are really 8 bit values and will be cast when used */
static const uint16_t s_RawIntMask[NUM_RAW_WINDOWS] = {WF_HOST_INT_MASK_RAW_0_INT_0,   /* used in HOST_INTR reg (8-bit register)   */
                                                       WF_HOST_INT_MASK_RAW_1_INT_0,   /* used in HOST_INTR reg (8-bit register)   */
                                                       WF_HOST_INT_MASK_RAW_2_INT_0,   /* used in HOST_INTR2 reg (16-bit register) */
                                                       WF_HOST_INT_MASK_RAW_3_INT_0,   /* used in HOST_INTR2 reg (16-bit register) */
                                                       WF_HOST_INT_MASK_RAW_4_INT_0,   /* used in HOST_INTR2 reg (16-bit register) */
                                                       WF_HOST_INT_MASK_RAW_5_INT_0};  /* used in HOST_INTR2 reg (16-bit register) */

static RAW_MOVE_WAIT_STATE s_rawMoveWaitState;

volatile RAW_MOVE_STATE RawMoveState[5];

/* bit mask where each bit corresponds to a raw window id.  If set, the raw index has been set past */
/* the end of the raw window.                                                                       */
const uint8_t g_RawAccessOutOfBoundsMask[NUM_RAW_WINDOWS] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20};

uint8_t g_RawIndexPastEnd = 0x00; /* no indexes are past end of window */

/***********************************/
/*    LOCAL FUNCTION PROTOTYPES    */
/***********************************/
static void RawMove(uint16_t rawId, uint16_t srcDest, bool rawIsDestination, uint16_t size);

/*****************************************************************************
  Function:
    bool AllocateDataTxBuffer(uint16_t bytesNeeded)

  Summary:
    Allocates a Data Tx buffer for use by the TCP/IP stack.

  Description:
    Determines if WiFi chip has enough memory to allocate a Tx data buffer, and,
    if so, allocates it.

  Precondition:
    None.

  Parameters:
    bytesNeeded -- number of bytes needed for the data Tx message

  Returns:
    True if data Tx buffer successfully allocated, else False.

  Remarks:
    None.
 *****************************************************************************/
bool AllocateDataTxBuffer(uint16_t bytesNeeded)
{
    uint16_t bufAvail;

    /* Ensure the MRF24W is awake (only applies if PS-Poll was enabled) */
    EnsureWFisAwake();

    /* get total bytes available for DATA tx memory pool */
    bufAvail = Read16BitWFRegister(WF_HOST_WFIFO_BCNT0_REG) & 0x0fff; /* LS 12 bits contain length */

    /* if enough bytes available to allocate */
    if ( bufAvail >= bytesNeeded )
    {
        RawMove(RAW_DATA_TX_ID, RAW_DATA_POOL, true, bytesNeeded);
        return true;
    }
    /* else not enough bytes available at this time to satisfy request */
    else
    {
        //SYS_CONSOLE_MESSAGE("Not enough bytes\r\n");
        return false;
    }
}

void  ClearAllIndexOutofBoundsFlags(void)
{
   g_RawIndexPastEnd = 0;
}

void SendRAWDataFrame(uint16_t bufLen)
{
    /* Notify WiFi device that there is a transmit frame to send .  The frame will */
    /* be automatically deallocated after RF transmission is completed.            */
    RawMove(RAW_DATA_TX_ID, RAW_MAC, false, bufLen);
}

/*****************************************************************************
  Function:
    void DeallocateDataRxBuffer(void)

  Summary:
    Deallocates a Data Rx buffer.

  Description:
    Typically called by MACGetHeader(), the assumption being that when the stack
    is checking for a newly received data message it is finished with the previously
    received data message.  Also called by MACGetHeader() if the SNAP header is invalid
    and the packet is thrown away.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 *****************************************************************************/
void DeallocateDataRxBuffer(void)
{
    EnsureWFisAwake();

    /* perform deallocation of raw Rx buffer */
    RawMove(RAW_DATA_RX_ID, RAW_DATA_POOL, false, 0);
}

void RawMountRxDataBuffer(void)
{
    // start mount process on MRF,
    // interrupt occurs when mount complete and Rx state machine will run
    RawMove(RAW_DATA_RX_ID, RAW_MAC, true, 0);
}

/*****************************************************************************
  Function:
    uint16_t ScratchMount(uint8_t rawId)

  Summary:
    Mounts RAW scratch window.

  Description:
    The scratch window is not dynamically allocated, but references a static
    portion of the WiFi device RAM. Thus, the Scratch data is not lost when
    the scratch window is unmounted.

  Precondition:
    None.

  Parameters:
    rawId -- RAW window ID being used to mount the scratch data

  Returns:
    Size, in bytes, of Scratch buffer.

  Remarks:
    None
 *****************************************************************************/
uint16_t ScratchMount(uint8_t rawId)
{
    uint16_t byteCount = 0;

    RawMove(rawId, RAW_SCRATCH_POOL, true, 0);
    return byteCount;
}

/*****************************************************************************
  Function:
    void ScratchUnmount(uint8_t rawId)

  Summary:
    Unmounts RAW scratch window.

  Description:
    The scratch window is not dynamically allocated, but references a static
    portion of the WiFi device RAM. Thus, the Scratch data is not lost when
    the scratch window is unmounted.

  Precondition:
    None.

  Parameters:
    rawId -- RAW window ID that was used to mount the scratch window

  Returns:
    Size, in bytes, of Scratch buffer.

  Remarks:
    None
 *****************************************************************************/
void ScratchUnmount(uint8_t rawId)
{
    RawMove(rawId, RAW_SCRATCH_POOL, false, 0);
}

/*
 *********************************************************************************************************
 * RawRead()
 *
 * Description : Reads the specified number of bytes from a mounted RAW window from the specified starting
 *               index.
 *
 * Argument(s) : rawId -- RAW window ID being read from
 *               startIndex -- start index within RAW window to read from
 *               length -- number of bytes to read from the RAW window
 *               p_dest -- pointer to Host buffer where read data is copied
 *
 * Return(s)   : error code
 *
 * Caller(s)   : WF Driver
 *
 * Notes       : None.
 *
 *********************************************************************************************************
 */
void RawRead(uint8_t rawId, uint16_t startIndex, uint16_t length, uint8_t *p_dest)
{
    RawSetIndex(rawId, startIndex);
    RawGetByte(rawId, p_dest, length);
}

// similar to RawRead, but takes advantage of the auto-increment feature and does
// not set the index before reading
void RawReadRelative(uint8_t rawId, uint16_t length, uint8_t *p_dest)
{
    RawGetByte(rawId, p_dest, length);
}

/*
 *********************************************************************************************************
 * RawWrite()
 *
 * Description : Writes the specified number of bytes to a mounted RAW window at the specified starting
 *               index.
 *
 * Argument(s) : rawId -- RAW window ID being written to
 *               startIndex -- start index within RAW window to write to
 *               length -- number of bytes to write to RAW window
 *               p_src -- pointer to Host buffer write data
 *
 * Return(s)   : None.
 *
 * Caller(s)   : WF Driver
 *
 * Notes       : None.
 *
 *********************************************************************************************************
 */
void RawWrite(uint8_t rawId, uint16_t startIndex, uint16_t length, uint8_t *p_src)
{
    /*set raw index in dest memory */
    RawSetIndex(rawId, startIndex);

    /* write data to RAW window */
    RawSetByte(rawId, p_src, length);
}

/*****************************************************************************
  Function: void RawMove(uint16_t rawId,
                         uint16_t srcDest,
                         bool     rawIsDestination,
                         uint16_t size)

  Summary:
    Performs RAW Move operation.

  Description:
    Raw Moves perform a variety of operations (e.g. allocating tx buffers,
    mounting rx buffers, copying from one raw window to another, etc.).

  Precondition:
    None.

  Parameters:
    rawId -- Raw ID 0 thru 5, except is srcDest is RAW_COPY, in which case rawId
             contains the source address in the upper 4 bits and destination
             address in lower 4 bits.

    srcDest -- object that will either be the source or destination of the move:
                RAW_MAC
                RAW_MGMT_POOL
                RAW_DATA_POOL
                RAW_SCRATCH_POOL
                RAW_STACK_MEM
                RAW_COPY ( this object not allowed, handled in RawToRawCopy() )

    rawIsDestination -- true if srcDest is the destination, false if srcDest is
                        the source of the move

    size -- number of bytes to overlay (not always applicable)

  Returns:
    None.

  Remarks:
    None.
 *****************************************************************************/
static void RawMove(uint16_t rawId,
                    uint16_t srcDest,
                    bool rawIsDestination,
                    uint16_t size)
{
    uint8_t regId;
    uint16_t ctrlVal = 0;

    s_rawMoveWaitState.rawId = rawId;

    /* create control value that will be written to raw control register, which initiates the raw move */
    if (rawIsDestination)
    {
        ctrlVal |= 0x8000;
    }

    ctrlVal |= (srcDest << 8);            /* defines are already shifted by 4 bits */
    ctrlVal |= ((size >> 8) & 0x0f) << 8; /* MS 4 bits of size (bits 11:8)         */
    ctrlVal |= (size & 0x00ff);           /* LS 8 bits of size (bits 7:0)          */

    // needs to be set before doing raw move
    RawMoveState[rawId].rawInterrupt = 0;
    RawMoveState[rawId].waitingForRawMoveCompleteInterrupt = true;

    /*------------------------------------------------------------------------------------------------*/
    /* now that the expected raw move complete interrupt has been cleared and we are ready to receive */
    /* it, initiate the raw move operation by writing to the appropriate RawCtrl0.                    */
    /*------------------------------------------------------------------------------------------------*/
    regId = s_RawCtrl0Reg[rawId];         /* get RawCtrl0 register address for desired raw ID */
    Write16BitWFRegister(regId, ctrlVal); /* write ctrl value to register                     */

    /* create mask to check against for Raw Move complete interrupt for either RAW0 or RAW1 */
    if (rawId <= RAW_ID_1)
    {
        /* will be either raw 0 or raw 1 */
        s_rawMoveWaitState.rawIntMask = (rawId == RAW_ID_0) ? WF_HOST_INT_MASK_RAW_0_INT_0 : WF_HOST_INT_MASK_RAW_1_INT_0;
    }
    else
    {
        /* will be INTR2 bit in host register, signifying RAW2, RAW3, or RAW4 */
        s_rawMoveWaitState.rawIntMask = WF_HOST_INT_MASK_INT2;
    }

    StartRawMoveTimer(rawId);
}

/*****************************************************************************
 * FUNCTION: isRawMoveComplete
 *
 * RETURNS: true if raw move complete, else false
 *
 * PARAMS:
 *   p_status -- pointer to current state of Raw move
 *   RM_COMPLETE -- Raw move completed successfully, function will return true in conjunction with this state
 *   RM_WAITING -- still waiting for Raw move to complete, function will return true in conjunction with this state
 *   RM_TIMEOUT -- Raw move failed due to timeout, function will return true in conjunction with this state
 *   p_byteCount -- pointer to where byte count is written upon raw move completion; only
 *                  valid when function returns true (not always used)
 *
 *  NOTES: Checks if a RAW move to complete.
 *****************************************************************************/
bool isRawMoveComplete(int rawId, int *p_status, uint16_t *p_byteCount)
{
    uint8_t regId;

    *p_byteCount = 0;
    *p_status = RM_WAITING;
    bool retCode = false;

    // if received an external interrupt that signalled the RAW Move completed
    if (RawMoveState[rawId].rawInterrupt & s_rawMoveWaitState.rawIntMask)
    {
        RawMoveState[rawId].waitingForRawMoveCompleteInterrupt = false;

        // stop raw move timer
        StopRawMoveTimer(s_rawMoveWaitState.rawId);

        /* read the byte count and save it */
        regId = s_RawCtrl1Reg[s_rawMoveWaitState.rawId];
        *p_byteCount = Read16BitWFRegister(regId) & 0x0fff; // LS 12 bits contain byte count (when used)
        *p_status = RM_COMPLETE;

        retCode = true;
    }

    return retCode;
}

/*****************************************************************************
  Function:
    bool RawSetIndex(uint16_t rawId, uint16_t index)

  Summary:
    Sets the index within the specified RAW window.

  Description:
    Sets the index within the specified RAW window.  If attempt to set RAW index
    outside boundaries of RAW window (past the end) this function will time out.
    It's legal to set the index past the end of the raw window so long as there
    is no attempt to read or write at that index.

  Precondition:
    None.

  Parameters:
    rawId -- RAW window ID
    index -- desired index within RAW window

  Returns:
    True if successful, False if caller tried to set raw index past end of
    window.

  Remarks:
    None.
 *****************************************************************************/
bool RawSetIndex(uint16_t rawId, uint16_t index)
{
    uint8_t regId;
    uint16_t regValue;
    uint32_t startTickCount;
    uint32_t maxAllowedTicks;

    /* get the index register associated with the raw ID and write to it */
    regId = s_RawIndexReg[rawId];
    Write16BitWFRegister(regId, index);

    /* Get the raw status register address associated with the raw ID.  This will be polled to         */
    /* determine that:                                                                                 */
    /*  1) raw set index completed successfully  OR                                                    */
    /*  2) raw set index failed, implying that the raw index was set past the end of the raw window    */
    regId = s_RawStatusReg[rawId];

    maxAllowedTicks = SYS_TMR_TickCounterFrequencyGet() / 200; /* 5ms */
    startTickCount = SYS_TMR_TickCountGet();

    /* read the status register until set index operation completes or times out */
    while (1)
    {
        regValue = Read16BitWFRegister(regId);
        if ((regValue & WF_RAW_STATUS_REG_BUSY_MASK) == 0)
        {
            clearIndexOutOfBoundsFlag(rawId);
            return true;
        }

        /* if timed out then trying to set index past end of raw window, which is OK so long as the app */
        /* doesn't try to access it                                                                     */
        if (SYS_TMR_TickCountGet() - startTickCount >= maxAllowedTicks)
        {
            setIndexOutOfBoundsFlag(rawId);
            return false; /* timed out waiting for Raw set index to complete */
        }

        DRV_WIFI_TimerTaskRun();
    }
}

/*****************************************************************************
 * FUNCTION: uint16_t RawGetIndex(uint16_t rawId)
 *
 * RETURNS: Returns the current RAW index for the specified RAW engine.
 *
 * PARAMS:
 *   rawId - RAW ID
 *
 * NOTES: None.
 *****************************************************************************/
uint16_t RawGetIndex(uint16_t rawId)
{
    uint8_t  regId;
    uint16_t index;

    regId = s_RawIndexReg[rawId];

    index = Read16BitWFRegister(regId);

    return index;
}

/*****************************************************************************
 * FUNCTION: void RawGetByte(uint16_t rawId, uint8_t *pBuffer, uint16_t length)
 *
 * RETURNS: error code
 *
 * PARAMS:
 *   rawId - RAW ID
 *   pBuffer - buffer to read bytes into
 *   length - number of bytes to read
 *
 *  NOTES: Reads bytes from the RAW engine.
 *****************************************************************************/
void RawGetByte(uint16_t rawId, uint8_t *pBuffer, uint16_t length)
{
    uint8_t regId;

    /* attempting to read past end of RAW buffer */
    DRV_WIFI_ASSERT(!isIndexOutOfBounds(rawId), "");

    regId = s_RawDataReg[rawId];
    ReadWFArray(regId, pBuffer, length);
}

/*****************************************************************************
 * FUNCTION: void RawSetByte(uint16_t rawId, uint8_t *pBuffer, uint16_t length)
 *
 * RETURNS: None.
 *
 * PARAMS:
 *   rawId - RAW ID
 *   pBuffer - buffer containing bytes to write
 *   length - number of bytes to read
 *
 * NOTES: Writes bytes to RAW window.
 *****************************************************************************/
void RawSetByte(uint16_t rawId, uint8_t *pBuffer, uint16_t length)
{
    uint8_t regId;

    /* attempting to write past end of RAW buffer */
    DRV_WIFI_ASSERT(!isIndexOutOfBounds(rawId), "");

    /* write data to raw window */
    regId = s_RawDataReg[rawId];
    WriteWFArray(regId, pBuffer, length);
}

//DOM-IGNORE-END
