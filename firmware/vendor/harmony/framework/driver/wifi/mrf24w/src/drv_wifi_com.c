/*******************************************************************************
  MRF24WG Driver Com Layer (Specific to the MRF24WG)

  File Name:
    drv_wifi_com.c

  Summary:
    Module for Microchip TCP/IP Stack

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

#include "drv_wifi_config_data.h"
#include "drv_wifi_debug_output.h"
#include "drv_wifi_eint.h"
#include "drv_wifi_mgmt_msg.h"
#include "drv_wifi_raw.h"

/*****************/
/*    DEFINES    */
/*****************/
// HOST_RESET_REG masks
#define HR_CPU_RST_N_MASK                  ((uint16_t)0x01 << 15)
#define HR_DISABLE_DOWNLOAD_SCRAMBLER_MASK ((uint16_t)0x01 << 14)
#define HR_FORCE_CPU_CLK_FREEZE_MASK       ((uint16_t)0x01 << 13)
#define HR_HOST_ANA_SPI_EN_MASK            ((uint16_t)0x01 << 12)
#define HR_HOST_ANA_SPI_DIN_MASK           ((uint16_t)0x01 << 11)
#define HR_HOST_ANA_SPI_DOUT_MASK          ((uint16_t)0x01 << 10)
#define HR_HOST_ANA_SPI_CLK_MASK           ((uint16_t)0x01 << 9)
#define HR_HOST_ANA_SPI_CSN_MASK           ((uint16_t)0x07 << 6) // 8:6
#define HR_RESERVED_2_MASK                 ((uint16_t)0x01 << 5)
#define HR_HOST_SPI_DISABLE_MASK           ((uint16_t)0x01 << 4)
#define HR_HOST_ENABLE_NEW_PROG_MASK       ((uint16_t)0x01 << 3)
#define HR_HOST_ENABLE_DOWNLOAD_MASK       ((uint16_t)0x01 << 2)
#define HR_HOST_FAST_RESET_MASK            ((uint16_t)0x01 << 1)
#define HR_HOST_RESET_MASK                 ((uint16_t)0x01 << 0)

// This block of defines needed to restart PLL
#define ANALOG_PORT_3_REG_TYPE ((uint32_t)0x09) // 16-bit analog register in SPI Port 3
#define ANALOG_PORT_2_REG_TYPE ((uint32_t)0x08) // 16-bit analog register in SPI Port 2
#define ANALOG_PORT_1_REG_TYPE ((uint32_t)0x0a) // 16-bit analog register in SPI Port 1
#define ANALOG_PORT_0_REG_TYPE ((uint32_t)0x0b) // 16-bit analog register in SPI Port 0

#define SPI_WRITE_MASK                   (uint8_t)0x00 // bit 0 = 0
#define SPI_READ_MASK                    (uint8_t)0x01 // bit 0 = 1
#define SPI_AUTO_INCREMENT_ENABLED_MASK  (uint8_t)0x00 // bit 1 = 0
#define SPI_AUTO_INCREMENT_DISABLED_MASK (uint8_t)0x02 // bit 1 = 1

#define PLL9_REG   ((uint32_t)(9 * 2)) // SPI Port 3 Registers (Port 5 if going through Master SPI controller)
#define PLL8_REG   ((uint32_t)(8 * 2))
#define PLL7_REG   ((uint32_t)(7 * 2))
#define PLL6_REG   ((uint32_t)(6 * 2))
#define PLL5_REG   ((uint32_t)(5 * 2))
#define PLL4_REG   ((uint32_t)(4 * 2))
#define PLL3_REG   ((uint32_t)(3 * 2))
#define PLL2_REG   ((uint32_t)(2 * 2))
#define PLL1_REG   ((uint32_t)(1 * 2))
#define PLL0_REG   ((uint32_t)(0 * 2))
// end PLL block

#define OSC0_REG            ((uint32_t)(0 * 2))
#define OSC1_REG            ((uint32_t)(1 * 2))
#define OSC2_REG            ((uint32_t)(2 * 2))
#define PLDO_REG            ((uint32_t)(3 * 2))
#define BIAS_REG            ((uint32_t)(4 * 2))
#define ANALOG_SPARE_REG    ((uint32_t)(5 * 2))

/**************************/
/*    STATIC VARIABLES    */
/**************************/
static bool s_WiFiConnectionChanged = false;
static bool s_WiFiConnection = false;
/* TODO: Currently s_RxPendingCount is possible to be negative. It needs to be fixed. */
static int16_t s_RxPendingCount = 0;

/***********************************/
/*    LOCAL FUNCTION PROTOTYPES    */
/***********************************/
static uint8_t GetSpiPortWithBitBang(uint8_t regType);
static void WriteAnalogRegisterBitBang(uint8_t regType, uint16_t address, uint16_t value);
static bool isMgmtResponseMsg(void);
static bool isMgmtIndicateMsg(void);

void DRV_WIFI_TimerTaskRun(void)
{
    SYS_TMR_Tasks(sysObj.sysTmr);
    DRV_TMR_Tasks(sysObj.drvTmr0);
}

void PowerSaveTask(void)
{
    // if application has disabled power save mode
    if (GetPowerSaveMode() == false)
        return;

    // if application wants PS-Poll, but the WiFi driver temporarily disabled it
    // to send a message
    if (isSleepNeeded()) {
        // if have not lost connection or started DCHP task in interim
        if (s_WiFiConnection == true)
            ConfigureLowPowerMode(WF_LOW_POWER_MODE_ON);

        ClearSleepNeeded();
    }
}

void SignalWiFiConnectionChanged(bool state)
{
    s_WiFiConnectionChanged = true;
    s_WiFiConnection = state;
}

void MgmtRxProcess(void)
{
    if (isMgmtResponseMsg()) {
        /* signal host state machine that a mgmt response has been received */
        MgmtRespReceivedSet();
    } else if (isMgmtIndicateMsg()) {
        DRV_WIFI_MgmtIndProcess();
    }
}

static bool isMgmtResponseMsg(void)
{
    uint8_t flag;

    // read the 'mailbox' for the mgmt response and return true if mgmt response msg is present
    RawRead(RAW_SCRATCH_ID, MGMT_RX_ACK_BASE, 1, &flag);

    // if 'mailbox' flag set
    if (flag == MGMT_RESPONSE_SET_FLAG)
    {
        // clear the 'mailbox' and return true
        RawSetIndex(RAW_SCRATCH_ID, MGMT_RX_ACK_BASE);
        flag = MGMT_RESPONSE_CLEAR_FLAG;
        RawSetByte(RAW_SCRATCH_ID, &flag, 1);
        return true;
    }
    // else no mgmt response msg
    else
    {
        return false;
    }
}

static bool isMgmtIndicateMsg(void)
{
    uint8_t flag;

    // read the 'mailbox' for the mgmt indicate and return true if mgmt indicate msg is present
    RawRead(RAW_SCRATCH_ID, MGMT_INDICATE_ACK_BASE, 1, &flag);

    // if 'mailbox' flag set
    if (flag == MGMT_INDICATE_SET_FLAG)
    {
        // clear the 'mailbox' and return true
        RawSetIndex(RAW_SCRATCH_ID, MGMT_INDICATE_ACK_BASE);
        flag = MGMT_INDICATE_CLEAR_FLAG;
        RawSetByte(RAW_SCRATCH_ID, &flag, 1);
        return true;
    }
    // else no mgmt indicate msg
    else
    {
        return false;
    }
}

/*****************************************************************************
 * FUNCTION: Read8BitWFRegister
 *
 * RETURNS: register value
 *
 * PARAMS:
 *      regId -- ID of 8-bit register being read
 *
 *  NOTES: Reads WF 8-bit register
 *****************************************************************************/
uint8_t Read8BitWFRegister(uint8_t regId)
{
    uint8_t txBuf[3], rxBuf[3];

    WF_SPI_MUTEX_LOCK();

    bool intEnabled = SYS_INT_SourceDisable(MRF_INT_SOURCE);

    txBuf[0] = regId | WF_READ_REGISTER_MASK;
    txBuf[1] = 0xff; 
    txBuf[2] = 0xff; // have to be 0x00 or oxff for DRV_SPI_BufferAddWriteRead()

    DRV_WIFI_SpiRead(
            txBuf,
            1,
            rxBuf,
            1);

    if (intEnabled)
    {
        DRV_WIFI_INT_SourceEnable();
    }

    WF_SPI_MUTEX_UNLOCK();

    return rxBuf[0];
}

/*****************************************************************************
 * FUNCTION: Write8BitWFRegister
 *
 * RETURNS: None
 *
 * PARAMS:
 *      regId -- ID of 8-bit register being written to
 *      value -- value to write
 *
 *  NOTES: Writes WF 8-bit register
 *****************************************************************************/
void Write8BitWFRegister(uint8_t regId, uint8_t value)
{
    uint8_t txBuf[3];

    WF_SPI_MUTEX_LOCK();
    bool intEnabled = SYS_INT_SourceDisable(MRF_INT_SOURCE);

    txBuf[0] = regId | WF_WRITE_REGISTER_MASK;
    txBuf[1] = value;

    DRV_WIFI_SpiWrite(txBuf, 2);

    if (intEnabled)
    {
        DRV_WIFI_INT_SourceEnable();
    }

    WF_SPI_MUTEX_UNLOCK();
}

/*****************************************************************************
 * FUNCTION: Read16BitWFRegister
 *
 * RETURNS: register value
 *
 * PARAMS:
 *      regId -- ID of 16-bit register being read
 *
 *  NOTES: Reads WF 16-bit register
 *****************************************************************************/
uint16_t Read16BitWFRegister(uint8_t regId)
{
    uint8_t txBuf[3], rxBuf[3];

    WF_SPI_MUTEX_LOCK();

    bool intEnabled = SYS_INT_SourceDisable(MRF_INT_SOURCE);

    txBuf[0] = regId | WF_READ_REGISTER_MASK;
    txBuf[1] = 0xff; 
    txBuf[2] = 0xff; // have to be 0x00 or oxff for DRV_SPI_BufferAddWriteRead()

    DRV_WIFI_SpiRead(
            txBuf,
            1,
            rxBuf,
            2);

    if (intEnabled)
    {
        DRV_WIFI_INT_SourceEnable();
    }

    WF_SPI_MUTEX_UNLOCK();

    return (((uint16_t)rxBuf[0]) << 8) | ((uint16_t)(rxBuf[1]));
}

/*****************************************************************************
 * FUNCTION: Write16BitWFRegister
 *
 * RETURNS: None
 *
 * PARAMS:
 *      regId -- ID of 16-bit register being written to
 *      value -- value to write
 *
 *  NOTES: Writes WF 16-bit register
 *****************************************************************************/
void Write16BitWFRegister(uint8_t regId, uint16_t value)
{
    uint8_t txBuf[3];

    WF_SPI_MUTEX_LOCK();
    bool intEnabled = SYS_INT_SourceDisable(MRF_INT_SOURCE);

    txBuf[0] = regId | WF_WRITE_REGISTER_MASK;
    txBuf[1] = (uint8_t)(value >> 8);     /* MS byte being written */
    txBuf[2] = (uint8_t)(value & 0x00ff); /* LS byte being written */

    DRV_WIFI_SpiWrite(txBuf, 3);

    if (intEnabled)
    {
        DRV_WIFI_INT_SourceEnable();
    }

    WF_SPI_MUTEX_UNLOCK();
}

/*****************************************************************************
 * FUNCTION: WriteWFArray
 *
 * RETURNS: None
 *
 * PARAMS:
 *      regId  -- Raw register being written to
 *      pBuf   -- pointer to array of bytes being written
 *      length -- number of bytes in pBuf
 *
 *  NOTES: Writes a data block to specified raw register
 *****************************************************************************/
void WriteWFArray(uint8_t regId, uint8_t *p_Buf, uint16_t length)
{
    uint8_t txBuf[3];

    WF_SPI_MUTEX_LOCK();
    bool intEnabled = SYS_INT_SourceDisable(MRF_INT_SOURCE);

    txBuf[0] = regId;

    DRV_WIFI_SpiWrite2(txBuf, 1, p_Buf, length);

    if (intEnabled)
    {
        DRV_WIFI_INT_SourceEnable();
    }

    WF_SPI_MUTEX_UNLOCK();
}

/*****************************************************************************
 * FUNCTION: ReadWFArray
 *
 * RETURNS: None
 *
 * PARAMS:
 *      regId  -- Raw register being read from
 *      pBuf   -- pointer where to write out bytes
 *      length -- number of bytes to read
 *
 *  NOTES: Reads a block of data from a raw register
 *****************************************************************************/
void ReadWFArray(uint8_t  regId, uint8_t *p_Buf, uint16_t length)
{
    uint8_t txBuf[3];

    WF_SPI_MUTEX_LOCK();

    bool intEnabled = SYS_INT_SourceDisable(MRF_INT_SOURCE);

    /* output command byte */
    txBuf[0] = regId | WF_READ_REGISTER_MASK;

    DRV_WIFI_SpiRead(txBuf, 1, p_Buf, length);

    if (intEnabled)
    {
        DRV_WIFI_INT_SourceEnable();
    }

    WF_SPI_MUTEX_UNLOCK();
}

/*****************************************************************************
 * FUNCTION: ResetModule
 *
 * RETURNS: N/A
 *
 * PARAMS:
 *      N/A
 *
 *
 *  NOTES: Performs the necessary SPI operations to cause the MRF24W to do a soft
 *         reset.
 *
 *         This function waits for the MRF24WG to complete its initialization before
 *         returning to the caller.  The largest part of the wait is for the MRF24WG
 *         to download any patch code in FLASH into its RAM.
 *****************************************************************************/
void ResetModule(void)
{
    uint16_t value;
    uint32_t timeoutPeriod;
    static uint32_t startTickCount;

    /* clear the power bit to disable low power mode on the MRF24W */
    Write16BitWFRegister(WF_PSPOLL_H_REG, 0x0000);

    /* set HOST_RESET bit in register to put device in reset */
    Write16BitWFRegister(WF_HOST_RESET_REG, Read16BitWFRegister(WF_HOST_RESET_REG) | WF_HOST_RESET_MASK);

    /* clear HOST_RESET bit in register to take device out of reset */
    Write16BitWFRegister(WF_HOST_RESET_REG, Read16BitWFRegister(WF_HOST_RESET_REG) & ~WF_HOST_RESET_MASK);

    /* after reset is started poll register to determine when HW reset has completed */
    timeoutPeriod = SYS_TMR_TickCounterFrequencyGet() * 3; /* 3000 ms */
    startTickCount = SYS_TMR_TickCountGet();
    do
    {
        DRV_WIFI_TimerTaskRun();
        Write16BitWFRegister(WF_INDEX_ADDR_REG, WF_HW_STATUS_REG);
        value = Read16BitWFRegister(WF_INDEX_DATA_REG);
        DRV_WIFI_ASSERT(SYS_TMR_TickCountGet() - startTickCount < timeoutPeriod, "");
    } while ((value & WF_HW_STATUS_NOT_IN_RESET_MASK) == 0);

    /* if SPI not connected will read all 1's */
    DRV_WIFI_ASSERT(value != 0xffff, "");

    timeoutPeriod = SYS_TMR_TickCounterFrequencyGet() / 10; /* 100 ms */
    startTickCount = SYS_TMR_TickCountGet();
    do
    {
        DRV_WIFI_TimerTaskRun();
    } while (SYS_TMR_TickCountGet() - startTickCount < timeoutPeriod);

    /* now that chip has come out of HW reset, poll the FIFO byte count register */
    /* which will be set to a non-zero value when the MRF24W initialization is   */
    /* complete.                                                                 */
    timeoutPeriod = SYS_TMR_TickCounterFrequencyGet() * 3; /* 3000 ms */
    startTickCount = SYS_TMR_TickCountGet();
    do
    {
        DRV_WIFI_TimerTaskRun();
        value = Read16BitWFRegister(WF_HOST_WFIFO_BCNT0_REG);
        DRV_WIFI_ASSERT(SYS_TMR_TickCountGet() - startTickCount < timeoutPeriod, "");
    } while (value == 0);
}

void IncrementRxPendingCount(void)
{
    ++s_RxPendingCount;
}

void DecrementRxPendingCount(void)
{
    --s_RxPendingCount;
}

int16_t GetRxPendingCount(void)
{
    return s_RxPendingCount;
}

// When bit-banging, determines which SPI port to use based on the type of register we are accessing
static uint8_t GetSpiPortWithBitBang(uint8_t regType)
{
    if (regType == ANALOG_PORT_3_REG_TYPE)
    {
        return 2;
    }
    else if (regType == ANALOG_PORT_2_REG_TYPE)
    {
        return 3;
    }
    else if (regType == ANALOG_PORT_1_REG_TYPE)
    {
        return 1;
    }
    else if (regType == ANALOG_PORT_0_REG_TYPE)
    {
        return 0;
    }
    else
    {
        return 0xff; // should never happen
    }
}

// required work-around for MRF silicon to properly reset device
static void WriteAnalogRegisterBitBang(uint8_t regType, uint16_t address, uint16_t value)
{
    uint8_t spiPort;
    uint16_t hrVal;
    uint8_t bitMask8;
    uint16_t bitMask16;
    uint8_t i;
    uint8_t regAddress;

    spiPort = GetSpiPortWithBitBang(regType); // extract SPI port (0-3) from the register type

    // Enable the on-chip SPI and select the desired bank (0-3)
    hrVal = (HR_HOST_ANA_SPI_EN_MASK | (spiPort << 6));
    Write16BitWFRegister(WF_HOST_RESET_REG, hrVal);

    // create register address byte
    regAddress = (address << 2) | SPI_AUTO_INCREMENT_ENABLED_MASK | SPI_WRITE_MASK;

    // bit-bang the regAddress byte, MS bit to LS bit
    bitMask8 = 0x80;        // start with MS bit of byte being bit-banged out
    for (i = 0; i < 8; ++i)
    {
        hrVal &= ~(HR_HOST_ANA_SPI_DOUT_MASK | HR_HOST_ANA_SPI_CLK_MASK); // zero out DOUT and CLK

        // mask out ADDRESS bit being clocked and write to HOST_ANA_SPI_DOUT (bit 10) in HOST_RESET_REG with the HOST_ANA_SPI_CLK low
        hrVal |= (regAddress & bitMask8) << (3 + i);  // first time: bit 7 << 3, second time: bit 6 << 4, etc.
        Write16BitWFRegister(WF_HOST_RESET_REG, hrVal);

        // now toggle SPI clock high, on rising edge this bit is clocked out
        hrVal |= HR_HOST_ANA_SPI_CLK_MASK;
        Write16BitWFRegister(WF_HOST_RESET_REG, hrVal);

        bitMask8 >>= 1; //  # get to next bit in address byte
    }

    // bit bang data from MS bit to LS bit
    bitMask16 = 0x8000;        // start with MS bit of byte being bit-banged out
    for (i = 0; i < 16; ++i)
    {
        hrVal &= ~(HR_HOST_ANA_SPI_DOUT_MASK | HR_HOST_ANA_SPI_CLK_MASK); // zero out DOUT and CLK

        // mask in data bit being clock out and write to HOST_ANA_SPI_DOUT (bit 10) in HOST_RESET_REG with the HOST_ANA_SPI_CLK low
        if ((15 - i) >= 10) // bits 15:10 need to be right-shifted
        {
            hrVal |= (value & bitMask16) >> (5 - i);  // first time: bit 15 << 5, second time: bit  14 << 4, etc.
        }
        else // bits 10:0 need to be left-shifted
        {
            hrVal |= (value & bitMask16) << (i - 5);        // first time: bit 10 << 0, second time: bit  9 << 1, etc.
        }

        Write16BitWFRegister(WF_HOST_RESET_REG, hrVal);

        // now toggle SPI clock high, on rising edge this bit is clocked out
        hrVal |= HR_HOST_ANA_SPI_CLK_MASK;
        Write16BitWFRegister(WF_HOST_RESET_REG, hrVal);

        bitMask16 = bitMask16 >> 1;  // go to next bit in data byte
    }

    // Disable the on-chip SPI
    hrVal &= ~HR_HOST_ANA_SPI_EN_MASK;
    Write16BitWFRegister(WF_HOST_RESET_REG, hrVal);
}

void ResetPll(void)
{
    // shuttle Ostrich workaround (won't affect production parts)
    WriteAnalogRegisterBitBang(ANALOG_PORT_3_REG_TYPE, PLL0_REG, 0x8021);
    WriteAnalogRegisterBitBang(ANALOG_PORT_3_REG_TYPE, PLL0_REG, 0x6021);

    // production Ostrich workaround (won't affect shuttle parts)
    WriteAnalogRegisterBitBang(ANALOG_PORT_1_REG_TYPE, OSC0_REG, 0x6b80);
    WriteAnalogRegisterBitBang(ANALOG_PORT_1_REG_TYPE, BIAS_REG, 0xc000);
}

// called from MRF24W_Tasks_ISR()
void DRV_WIFI_INT_Handle(void)
{
    uint8_t eintHostInt;
    uint16_t hostInt2 = 0; // avoid warning
    uint32_t mgmtEventInfo;

    SYS_INT_SourceDisable(MRF_INT_SOURCE); // disable further interrupts

    // read the active int bits and mask with enabled interrupts to get the current interrupts
    eintHostInt = Read8BitWFRegister(WF_HOST_INTR_REG) & Read8BitWFRegister(WF_HOST_MASK_REG);

    // if level two interrupt occurred (mgmt response/indicate msg after hardware init; before
    // hardware init it will be the scratch memory mount to raw 4
    if (eintHostInt & WF_HOST_INT_MASK_INT2)
    {
        // read level 2 cause bits and clear level 2 interrupt sources
        // (after init, only mgmt messages generate this interrupt)
        hostInt2 = Read16BitWFRegister(WF_HOST_INTR2_REG);
        Write16BitWFRegister(WF_HOST_INTR2_REG, hostInt2);

        // read the mailbox register and verify that mgmt response or indicate is ready to read
        mgmtEventInfo = ((uint32_t)Read16BitWFRegister(WF_HOST_MAIL_BOX_1_MSW_REG)) << 16;
        mgmtEventInfo |= ((uint32_t)Read16BitWFRegister(WF_HOST_MAIL_BOX_1_LSW_REG));
        if ((mgmtEventInfo >> 24) == 0x60) // if mgmt rx msg
        {
            MgmtRxProcess();
        }
    }

    //------------------------------------
    // if still in initialization
    //------------------------------------
    if (!isInitComplete())
    {
        // if waiting for unmount from RAW 1 to complete (at init, scratch is mounted by MRF to RAW 1; it will
        // be unmounted and then remounted to RAW 4)
        if (RawMoveState[RAW_DATA_TX_ID].waitingForRawMoveCompleteInterrupt)
        {
            // if scratch unmount from RAW 1 complete
            if ((eintHostInt & WF_HOST_INT_MASK_RAW_1_INT_0) == WF_HOST_INT_MASK_RAW_1_INT_0)
            {
                /* save the copy of the active interrupts */
                RawMoveState[RAW_DATA_TX_ID].rawInterrupt = eintHostInt;
                RawMoveState[RAW_DATA_TX_ID].waitingForRawMoveCompleteInterrupt = false;
            }
        }

        // if waiting for scratch mount to RAW 4 to complete
        if (RawMoveState[RAW_SCRATCH_ID].waitingForRawMoveCompleteInterrupt)
        {
           // if got a level 2 interrupt
           if (eintHostInt & WF_HOST_INT_MASK_INT2)
           {
               // if scratch mount to RAW 4 complete
               if ((hostInt2 & WF_HOST_INT_MASK_RAW_4_INT_0) == WF_HOST_INT_MASK_RAW_4_INT_0)
               {
                   /* save the copy of the active interrupts */
                   RawMoveState[RAW_SCRATCH_ID].rawInterrupt = eintHostInt;
                   RawMoveState[RAW_SCRATCH_ID].waitingForRawMoveCompleteInterrupt = false;
               }
           }
        }
    }
    //--------------------------------------------------------------------------
    // else init complete, so only raw move completes that can occur are data rx
    // and data tx raw move completes
    //--------------------------------------------------------------------------
    else
    {
        // if Rx raw move complete (signal for either mounting or unmounting an Rx packet)
        if (RawMoveState[RAW_DATA_RX_ID].waitingForRawMoveCompleteInterrupt)
        {
            // if got a Raw Move complete for Data Rx raw window
            if ((eintHostInt & WF_HOST_INT_MASK_RAW_0_INT_0) == WF_HOST_INT_MASK_RAW_0_INT_0)
            {
                RawMoveState[RAW_DATA_RX_ID].rawInterrupt = eintHostInt;
                RawMoveState[RAW_DATA_RX_ID].waitingForRawMoveCompleteInterrupt = false;

                // trigger event so DRV_WIFI_DataRxTask() will run
                SignalRxStateMachine();
            }
        }

        // if tx raw move complete (signal for allocating a tx packet or sending a tx packet)
        if (RawMoveState[RAW_DATA_TX_ID].waitingForRawMoveCompleteInterrupt)
        {
            // if got a Raw Move complete for Data Tx window
            if ((eintHostInt & WF_HOST_INT_MASK_RAW_1_INT_0) == WF_HOST_INT_MASK_RAW_1_INT_0)
            {
                RawMoveState[RAW_DATA_TX_ID].rawInterrupt = eintHostInt;
                RawMoveState[RAW_DATA_TX_ID].waitingForRawMoveCompleteInterrupt = false;

                // trigger event so DRV_WIFI_DataTxTask() will run
                SignalTxStateMachine();
            }
        }

        // if got a FIFO 0 Threshold Interrupt (Data Fifo Rx signal)
        if ((eintHostInt & WF_HOST_INT_MASK_FIFO_0_THRESHOLD) == WF_HOST_INT_MASK_FIFO_0_THRESHOLD)
        {
            if (isRxStateMachineIdle())
            {
                // trigger event so DRV_WIFI_DataRxTask() will run
                SignalRxStateMachine();
            }
            else
            {
                SetRxPacketPending();
            }
        }
    }

    // re-enable Wi-Fi interrupt
    DRV_WIFI_INT_SourceEnable();

    // clear host int sources on MRF24WG, which will cause MRF to raise interrupt line high
    Write8BitWFRegister(WF_HOST_INTR_REG, eintHostInt);
}

void DRV_WIFI_DataCacheClean(unsigned char *address, uint32_t size)
{
#if defined(__PIC32MZ__)
    if (IS_KVA0(address)) {
        uint32_t a = (uint32_t)address & 0xfffffff0;
        uint32_t r = (uint32_t)address & 0x0000000f;
        uint32_t s = ((size + r + 15) >> 4) << 4;

        SYS_DEVCON_DataCacheClean(a, s);
    }
#endif
}

//DOM-IGNORE-END
