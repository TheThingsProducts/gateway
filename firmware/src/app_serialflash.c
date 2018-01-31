// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "app.h"
#include "app_serialflash.h"

#define ACTIVATION_DATA_LOCKED 0x00
#define ACTIVATION_DATA_NOT_LOCKED 0xFF

#define SERIALFLASH_BLOCK_OPERATION_COMPLETE 0x7
#define SERIALFLASH_BLOCK_OPERATION_ERROR 0x80

#define SERIALFLASH_SPI_COMMAND_NOP 0x00      // No Operation
#define SERIALFLASH_SPI_COMMAND_RSTEN 0x66    // Reset Enable
#define SERIALFLASH_SPI_COMMAND_RST 0x99      // Reset Memory
#define SERIALFLASH_SPI_COMMAND_RDSR 0x05     // Read Status Register
#define SERIALFLASH_SPI_COMMAND_RDCR 0x35     // Read Configuration Register
#define SERIALFLASH_SPI_COMMAND_JEDEC_ID 0x9F // JEDEC-ID Read
#define SERIALFLASH_SPI_COMMAND_WREN 0x06     // Write Enable
#define SERIALFLASH_SPI_COMMAND_SE 0x20       // Erase 4 KBytes of Memory Array
#define SERIALFLASH_SPI_COMMAND_CE 0xC7       // Erase Full Array
#define SERIALFLASH_SPI_COMMAND_RBPR 0x72     // Read Block-Protection Register
#define SERIALFLASH_SPI_COMMAND_LBPR 0x8D     // Lock Down Block-Protection Register
#define SERIALFLASH_SPI_COMMAND_ULBPR 0x98    // Global Block Protection Unlock

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

static APP_SERIALFLASH_DATA serialflashData;

static void APP_SST25VF064CEventHandler(DRV_SST25VF064C_BLOCK_EVENT event, DRV_SST25VF064C_BLOCK_COMMAND_HANDLE handle,
                                        uintptr_t context);

static uint8_t APP_SerialFlash_GetStatus(void);
static bool    APP_SerialFlash_IsBusy(void);

static void APP_SerialFlash_GetJedecID(char* pc);
static void APP_SerialFlash_GetStatusRegister(char* pc);
static void APP_SerialFlash_GetConfigurationRegister(char* pc);
static void APP_SerialFlash_GetBlockProtectionRegister(char* pc);
static void APP_SerialFlash_WriteEnable(void);
static void APP_SerialFlash_LockDownBlockProtectionRegister(void);
static void APP_SerialFlash_UnblockMemory(void);
// static void APP_SerialFlash_BlockErase(uint32_t block);
static void APP_SerialFlash_ChipErase(void);
static void APP_SerialFlash_Reset(void);

static void APP_SerialFlash_PreFillData(void);

char JedecID[60];
char StatusReg[60];
char ConfigReg[60];
char BlockProtReg[80];

uint8_t dataBuffer1[FLASH_SECTOR_SIZE + FLASH_PAGE_SIZE];
uint8_t dataBuffer2[FLASH_SECTOR_SIZE + FLASH_PAGE_SIZE];

uint32_t nRemaining         = 0;
bool     fota_image_storage = false;

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

/*
 * Event handler for SST25VF064C driver's Block operations
 */
static void APP_SST25VF064CEventHandler(DRV_SST25VF064C_BLOCK_EVENT event, DRV_SST25VF064C_BLOCK_COMMAND_HANDLE handle,
                                        uintptr_t context)
{
    switch(event)
    {
        case DRV_SST25VF064C_EVENT_BLOCK_COMMAND_COMPLETE:
        {

            /* Update the eventMap depending on the command completed. */
            if(handle == serialflashData.commandHandle[0])
            {
                serialflashData.eventMap |= 0x01 << 0;
            }

            if(handle == serialflashData.commandHandle[1])
            {
                serialflashData.eventMap |= 0x01 << 1;
            }

            if(handle == serialflashData.commandHandle[2])
            {
                serialflashData.eventMap |= 0x01 << 2;
            }

            break;
        }

        case DRV_SST25VF064C_EVENT_BLOCK_COMMAND_ERROR:
        {
            /* Set the MSB of eventMap to indicate that there has been an error. */
            serialflashData.eventMap |= 0x01 << 7;
            break;
        }

        default:
        {
            break;
        }
    }
    // SYS_DEBUG(SYS_ERROR_DEBUG, "FLASH: eventMap: %02x\r\n", serialflashData.eventMap);
}

/*
 * SPI command to read out the serial flash's status register
 */
static uint8_t APP_SerialFlash_GetStatus(void)
{
    uint8_t data[1];

    SYS_INT_SourceDisable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_ERROR_INT_SOURCE_IDX2);

    while(PLIB_SPI_TransmitBufferIsEmpty(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    SYS_PORTS_PinClear(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                       DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);

    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_RDSR);
    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    data[0] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);

    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_NOP);
    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    data[0] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);

    SYS_PORTS_PinSet(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                     DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);

    SYS_INT_SourceEnable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_ERROR_INT_SOURCE_IDX2);

    return data[0];
}

/*
 * Command to read out the serial flash's status register and check if the BUSY bits are set
 */
static bool APP_SerialFlash_IsBusy(void)
{
    return APP_SerialFlash_GetStatus() & 0x81;
}

/*
 * SPI command to read out the JEDEC-ID and store it
 */
static void APP_SerialFlash_GetJedecID(char* pc)
{
    uint8_t data[3];

    SYS_INT_SourceDisable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_ERROR_INT_SOURCE_IDX2);

    while(PLIB_SPI_TransmitBufferIsEmpty(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    SYS_PORTS_PinClear(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                       DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);

    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_JEDEC_ID);
    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    data[0] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);

    uint8_t i;
    for(i = 0; i < 3; i++)
    {
        PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_NOP);
        while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2) == false)
            ;
        data[i] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);
    }

    SYS_PORTS_PinSet(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                     DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);
    sprintf(pc, "Manufacturer ID:%02x Device Type:%02x Device ID:%02x", data[0], data[1], data[2]);

    SYS_INT_SourceEnable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_ERROR_INT_SOURCE_IDX2);
}

/*
 * SPI command to read out the status register and store it
 */
static void APP_SerialFlash_GetStatusRegister(char* pc)
{
    uint8_t data[1];

    SYS_INT_SourceDisable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_ERROR_INT_SOURCE_IDX2);

    while(PLIB_SPI_TransmitBufferIsEmpty(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    SYS_PORTS_PinClear(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                       DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);

    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_RDSR);
    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    data[0] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);

    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_NOP);
    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    data[0] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);

    SYS_PORTS_PinSet(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                     DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);
    sprintf(pc, "Status Register[7:0]: %02x", data[0]);

    SYS_INT_SourceEnable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_ERROR_INT_SOURCE_IDX2);
}

/*
 * SPI command to read out the configuration register and store it
 */
static void APP_SerialFlash_GetConfigurationRegister(char* pc)
{
    uint8_t data[1];

    SYS_INT_SourceDisable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_ERROR_INT_SOURCE_IDX2);

    while(PLIB_SPI_TransmitBufferIsEmpty(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    SYS_PORTS_PinClear(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                       DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);

    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_RDCR);
    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    data[0] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);

    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_NOP);
    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    data[0] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);

    SYS_PORTS_PinSet(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                     DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);
    sprintf(pc, "Configuration Register[7:0]: %02x", data[0]);

    SYS_INT_SourceEnable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_ERROR_INT_SOURCE_IDX2);
}

/*
 * SPI command to read out the protection register and store it
 */
static void APP_SerialFlash_GetBlockProtectionRegister(char* pc)
{
    uint8_t data[18];

    SYS_INT_SourceDisable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_ERROR_INT_SOURCE_IDX2);

    while(PLIB_SPI_TransmitBufferIsEmpty(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    SYS_PORTS_PinClear(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                       DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);

    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_RBPR);
    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    data[0] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);

    uint8_t i;
    for(i = 0; i < 18; i++)
    {
        PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_NOP);
        while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2) == false)
            ;
        data[i] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);
    }

    SYS_PORTS_PinSet(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                     DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);
    sprintf(pc,
            "Block-Protection Register[143:0]: %02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x "
            "%02x%02x%02x%02x",
            data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10],
            data[11], data[12], data[13], data[14], data[15], data[16], data[17]);

    SYS_INT_SourceEnable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_ERROR_INT_SOURCE_IDX2);
}

/*
 * SPI command for write enable
 */
static void APP_SerialFlash_WriteEnable(void)
{
    SYS_INT_SourceDisable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_ERROR_INT_SOURCE_IDX2);

    while(PLIB_SPI_TransmitBufferIsEmpty(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    SYS_PORTS_PinClear(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                       DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);

    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_WREN);
    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);

    SYS_PORTS_PinSet(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                     DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);

    SYS_INT_SourceEnable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_ERROR_INT_SOURCE_IDX2);
}

/*
 * SPI command to lock down the protection register
 */
static void APP_SerialFlash_LockDownBlockProtectionRegister(void)
{
    SYS_INT_SourceDisable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_ERROR_INT_SOURCE_IDX2);

    while(PLIB_SPI_TransmitBufferIsEmpty(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    SYS_PORTS_PinClear(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                       DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);

    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_LBPR);
    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);

    SYS_PORTS_PinSet(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                     DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);

    SYS_INT_SourceEnable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_ERROR_INT_SOURCE_IDX2);
}

/*
 * SPI command for global block protection unlock
 */
static void APP_SerialFlash_UnblockMemory(void)
{
    SYS_INT_SourceDisable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_ERROR_INT_SOURCE_IDX2);

    while(PLIB_SPI_TransmitBufferIsEmpty(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    SYS_PORTS_PinClear(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                       DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);

    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_ULBPR);
    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);

    SYS_PORTS_PinSet(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                     DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);

    SYS_INT_SourceEnable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_ERROR_INT_SOURCE_IDX2);
}

/*
 * SPI command for Block erase
 */
// static void APP_SerialFlash_BlockErase(uint32_t block)
//{
//    uint8_t data[2];
//
//    SYS_INT_SourceDisable(DRV_SPI_RX_INT_SOURCE_IDX2);
//    SYS_INT_SourceDisable(DRV_SPI_TX_INT_SOURCE_IDX2);
//    SYS_INT_SourceDisable(DRV_SPI_ERROR_INT_SOURCE_IDX2);
//
//    while (PLIB_SPI_TransmitBufferIsEmpty(DRV_SPI_SPI_ID_IDX2) == false);
//    SYS_PORTS_PinClear(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
//    DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);
//
//    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_RDSR);
//    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2)==false);
//    data[0] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);
//
//    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_NOP);
//    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2)==false);
//    data[0] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);
//
//    SYS_PORTS_PinSet(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
//    DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);
//    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
//    SYS_PORTS_PinClear(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
//    DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);
//
//    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_ULBPR);
//    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2)==false);
//    data[0] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);
//
//    SYS_PORTS_PinSet(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
//    DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);
//    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
//    SYS_PORTS_PinClear(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
//    DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);
//
//    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_WREN);
//    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2)==false);
//    data[1] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);
//
//    SYS_PORTS_PinSet(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
//    DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);
//    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
//    SYS_PORTS_PinClear(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
//    DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);
//
//    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_SE);
//    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2)==false);
//    data[1] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);
//
//    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_NOP);
//    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2)==false);
//    data[1] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);
//
//    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_NOP);
//    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2)==false);
//    data[1] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);
//
//    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_NOP);
//    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2)==false);
//    data[1] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);
//
//    SYS_PORTS_PinSet(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
//    DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);
//
//    for(;;)
//    {
//        asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
//        SYS_PORTS_PinClear(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
//        DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);
//
//        PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_RDSR);
//        while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2)==false);
//        data[1] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);
//
//        PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_RDSR);
//        while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2)==false);
//        data[1] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);
//        if((data[1]&0x81)==0)break;
//
//        SYS_PORTS_PinSet(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
//        DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);
//    }
//
//    SYS_INT_SourceEnable(DRV_SPI_RX_INT_SOURCE_IDX2);
//    SYS_INT_SourceEnable(DRV_SPI_TX_INT_SOURCE_IDX2);
//    SYS_INT_SourceEnable(DRV_SPI_ERROR_INT_SOURCE_IDX2);
//
//}

/*
 * SPI command for chip erase
 */
static void APP_SerialFlash_ChipErase(void)
{
    SYS_INT_SourceDisable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_ERROR_INT_SOURCE_IDX2);

    while(PLIB_SPI_TransmitBufferIsEmpty(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    SYS_PORTS_PinClear(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                       DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);

    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_CE);
    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);

    SYS_PORTS_PinSet(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                     DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);

    SYS_INT_SourceEnable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_ERROR_INT_SOURCE_IDX2);
}

/*
 * SPI command for reset
 */
static void APP_SerialFlash_Reset(void)
{
    uint8_t data[1];

    SYS_INT_SourceDisable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceDisable(DRV_SPI_ERROR_INT_SOURCE_IDX2);

    while(PLIB_SPI_TransmitBufferIsEmpty(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    SYS_PORTS_PinClear(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                       DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);

    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_RSTEN);
    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    data[0] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);

    SYS_PORTS_PinSet(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                     DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    asm("nop");
    SYS_PORTS_PinClear(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                       DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);

    PLIB_SPI_BufferWrite(DRV_SPI_SPI_ID_IDX2, SERIALFLASH_SPI_COMMAND_RST);
    while(PLIB_SPI_ReceiverBufferIsFull(DRV_SPI_SPI_ID_IDX2) == false)
        ;
    data[0] = PLIB_SPI_BufferRead(DRV_SPI_SPI_ID_IDX2);

    SYS_PORTS_PinSet(PORTS_ID_0, DRV_SST25VF064C_CHIP_SELECT_PORT_CHANNEL_IDX0,
                     DRV_SST25VF064C_CHIP_SELECT_PORT_BIT_POS_IDX0);

    SYS_INT_SourceEnable(DRV_SPI_RX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_TX_INT_SOURCE_IDX2);
    SYS_INT_SourceEnable(DRV_SPI_ERROR_INT_SOURCE_IDX2);
}

/*
 * Function to prefill databuffers
 */
static void APP_SerialFlash_PreFillData(void)
{
    uint16_t i = 0;

    for(i = 0; i < FLASH_SECTOR_SIZE; i++)
    {
        serialflashData.sourceBuffer[i] = 0;
        serialflashData.targetBuffer[i] = 0;
    }
}

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

/*******************************************************************************
  Function:
    void APP_Initialize(void)

  Remarks:
    See prototype in app.h.
 */
void APP_SERIALFLASH_Initialize(void)
{
    serialflashData.state = APP_SERIALFLASH_INIT;

    serialflashData.has_wifi_data       = false;
    serialflashData.has_activation_data = false;
    serialflashData.has_fota_data       = false;

    serialflashData.sourceBuffer = &dataBuffer1[0]; // Buffer for storing data to write to serialflash
    serialflashData.targetBuffer = &dataBuffer2[0]; // Buffer for data read from serialflash
    APP_SerialFlash_PreFillData();

    serialflashData.commandHandle[0] = 0; // Handle for erase operation
    serialflashData.commandHandle[1] = 0; // Handle for write operation
    serialflashData.commandHandle[2] = 0; // Handle for read operation
    serialflashData.eventMap         = 0; // Event map for block operations

    serialflashData.address = 0; // Starting address for a block operation
    serialflashData.length  = 0; // Length in bytes for a block operation

    serialflashData.image_length          = 0; // Total bytes of FOTA image written
    serialflashData.expected_image_length = 0; // Total bytes of FOTA image to write
}

/*******************************************************************************
  Function:
    void APP_Tasks(void)

  Remarks:
    See prototype in app.h.
 */
void APP_SERIALFLASH_Tasks(void)
{
    switch(serialflashData.state)
    {
        // Wait until serialflash isn't busy, so we can start configuration
        case APP_SERIALFLASH_INIT:
        {
            if(APP_SerialFlash_IsBusy())
            {
                break;
            }
            serialflashData.state = APP_SERIALFLASH_WRITE_ENABLE;
            break;
        }
        // If write isn't enabled, enable it
        case APP_SERIALFLASH_WRITE_ENABLE:
        {
            uint8_t status = APP_SerialFlash_GetStatus();
            if(status & 0x81)
            {
                // BUSY
                // Internal Write operation is in progress
                break;
            }
            if(!(status & 0x02))
            {
                SYS_DEBUG(SYS_ERROR_DEBUG, "FLASH: Write Enable\r\n");
                APP_SerialFlash_WriteEnable();
                break;
            }
            serialflashData.state = APP_SERIALFLASH_UNBLOCK_MEMORY;
            break;
        }
        // Global Block Protection Unlock
        case APP_SERIALFLASH_UNBLOCK_MEMORY:
        {
            uint8_t status = APP_SerialFlash_GetStatus();
            if(status & 0x81)
            {
                // BUSY
                // Internal Write operation is in progress
                break;
            }
            SYS_DEBUG(SYS_ERROR_DEBUG, "FLASH: Global Block Protection Unlock\r\n");
            APP_SerialFlash_UnblockMemory();
            serialflashData.state = APP_SERIALFLASH_OPEN_SPI_FLASH_DRIVER;
            break;
        }
        // Initialize the SST25VF064C driver,
        // Print the JEDEC-ID and registers
        case APP_SERIALFLASH_OPEN_SPI_FLASH_DRIVER:
        {
            if(APP_SerialFlash_IsBusy())
            {
                break;
            }
            serialflashData.driverHandle =
                DRV_SST25VF064C_Open(DRV_SST25VF064C_INDEX_0, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING);
            if(serialflashData.driverHandle != DRV_HANDLE_INVALID)
            {
                /* Driver open was successful. Register a event handler
                 * function with the driver. */
                DRV_SST25VF064C_BlockEventHandlerSet(serialflashData.driverHandle, APP_SST25VF064CEventHandler, NULL);

                APP_SerialFlash_GetJedecID(JedecID);
                APP_SerialFlash_GetStatusRegister(StatusReg);
                APP_SerialFlash_GetConfigurationRegister(ConfigReg);
                APP_SerialFlash_GetBlockProtectionRegister(BlockProtReg);
                SYS_DEBUG(SYS_ERROR_DEBUG, "FLASH: %s\r\n", JedecID);
                SYS_DEBUG(SYS_ERROR_DEBUG, "FLASH: %s\r\n", StatusReg);
                SYS_DEBUG(SYS_ERROR_DEBUG, "FLASH: %s\r\n", ConfigReg);
                SYS_DEBUG(SYS_ERROR_DEBUG, "FLASH: %s\r\n", BlockProtReg);

                serialflashData.state = APP_SERIALFLASH_READ_MAGIC_BYTES_WIFI;
                break;
            }
            else
            {
                /* Stay in the same state until a valid driver handle is returned. */
            }
            break;
        }

        // Schedule a read operation to read if the magic bytes are set in the Wifi data sector
        case APP_SERIALFLASH_READ_MAGIC_BYTES_WIFI:
        {
            if(APP_SerialFlash_IsBusy())
            {
                break;
            }
            serialflashData.commandHandle[0] = 0; // Erase
            serialflashData.commandHandle[1] = 0; // Write
            serialflashData.commandHandle[2] = 0; // Read
            serialflashData.eventMap         = 0;
            DRV_SST25VF064C_BlockRead(serialflashData.driverHandle, &serialflashData.commandHandle[2],
                                      &serialflashData.targetBuffer[0], FLASH_ADDRESS_WIFI_DATA_MAGIC_BYTES,
                                      FLASH_LENGTH_WIFI_DATA_MAGIC_BYTES);
            if(serialflashData.commandHandle[2] == DRV_SST25VF064C_BLOCK_COMMAND_HANDLE_INVALID)
            {
                SYS_DEBUG(SYS_ERROR_ERROR, "FLASH: Failed to queue the read operation\r\n");
                serialflashData.state = APP_SERIALFLASH_ERROR;
                break;
            }
            serialflashData.state = APP_SERIALFLASH_WAIT_FOR_MAGIC_BYTES_WIFI;
            break;
        }
        // Wait for the scheduled operation to complete
        case APP_SERIALFLASH_WAIT_FOR_MAGIC_BYTES_WIFI:
        {
            if(serialflashData.eventMap == SERIALFLASH_BLOCK_OPERATION_COMPLETE)
            {
                serialflashData.state = APP_SERIALFLASH_VERIFY_MAGIC_BYTES_WIFI;
            }
            if(serialflashData.eventMap & SERIALFLASH_BLOCK_OPERATION_ERROR)
            {
                SYS_DEBUG(SYS_ERROR_ERROR, "FLASH: There was an error while processing a read operation\r\n");
                serialflashData.state = APP_SERIALFLASH_ERROR;
            }
            break;
        }
        // Verify if the magic bytes are set
        case APP_SERIALFLASH_VERIFY_MAGIC_BYTES_WIFI:
        {
            // Magic bytes: 0xDEADBEEF
            if((serialflashData.targetBuffer[0] == 0xDE) && (serialflashData.targetBuffer[1] == 0xAD) &&
               (serialflashData.targetBuffer[2] == 0xBE) && (serialflashData.targetBuffer[3] == 0xEF))
            {
                SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Magic bytes found: wifi config present\r\n");
                serialflashData.has_wifi_data = true;
            }
            else
            {
                SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Magic bytes not found: no stored wifi config present\r\n");
                serialflashData.has_wifi_data = false;
            }
            serialflashData.state = APP_SERIALFLASH_READ_MAGIC_BYTES_ACTIVATION_DATA;
            break;
        }
        // Schedule a read operation to read if the magic bytes are set in the activation data sector
        case APP_SERIALFLASH_READ_MAGIC_BYTES_ACTIVATION_DATA:
        {
            if(APP_SerialFlash_IsBusy())
            {
                break;
            }
            serialflashData.commandHandle[0] = 0; // Erase
            serialflashData.commandHandle[1] = 0; // Write
            serialflashData.commandHandle[2] = 0; // Read
            serialflashData.eventMap         = 0;
            DRV_SST25VF064C_BlockRead(serialflashData.driverHandle, &serialflashData.commandHandle[2],
                                      &serialflashData.targetBuffer[0], FLASH_ADDRESS_ACTIVATION_DATA_MAGIC_BYTES,
                                      FLASH_LENGTH_ACTIVATION_DATA_MAGIC_BYTES);
            if(serialflashData.commandHandle[2] == DRV_SST25VF064C_BLOCK_COMMAND_HANDLE_INVALID)
            {
                SYS_DEBUG(SYS_ERROR_ERROR, "FLASH: Failed to queue the read operation\r\n");
                serialflashData.state = APP_SERIALFLASH_ERROR;
                break;
            }
            serialflashData.state = APP_SERIALFLASH_WAIT_FOR_MAGIC_BYTES_ACTIVATION_DATA;
            break;
        }
        // Wait for the scheduled operation to complete
        case APP_SERIALFLASH_WAIT_FOR_MAGIC_BYTES_ACTIVATION_DATA:
        {
            if(serialflashData.eventMap == SERIALFLASH_BLOCK_OPERATION_COMPLETE)
            {
                serialflashData.state = APP_SERIALFLASH_VERIFY_MAGIC_BYTES_ACTIVATION_DATA;
            }
            if(serialflashData.eventMap & SERIALFLASH_BLOCK_OPERATION_ERROR)
            {
                SYS_DEBUG(SYS_ERROR_ERROR, "FLASH: There was an error while processing a read operation\r\n");
                serialflashData.state = APP_SERIALFLASH_ERROR;
            }
            break;
        }
        // Verify if the magic bytes are set
        case APP_SERIALFLASH_VERIFY_MAGIC_BYTES_ACTIVATION_DATA:
        {
            // Magic bytes: 0xDEADBEEF
            if((serialflashData.targetBuffer[0] == 0xDE) && (serialflashData.targetBuffer[1] == 0xAD) &&
               (serialflashData.targetBuffer[2] == 0xBE) && (serialflashData.targetBuffer[3] == 0xEF))
            {
                SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Magic bytes found: activation data present\r\n");
                serialflashData.has_activation_data = true;
            }
            else
            {
                SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Magic bytes not found: no stored activation data present\r\n");
                serialflashData.has_activation_data = false;
            }
            serialflashData.state = APP_SERIALFLASH_READ_MAGIC_BYTES_FOTA;
            break;
        }
        // Schedule a read operation to read if the magic bytes are set in the FOTA data sector
        case APP_SERIALFLASH_READ_MAGIC_BYTES_FOTA:
        {
            if(APP_SerialFlash_IsBusy())
            {
                break;
            }
            serialflashData.commandHandle[0] = 0; // Erase
            serialflashData.commandHandle[1] = 0; // Write
            serialflashData.commandHandle[2] = 0; // Read
            serialflashData.eventMap         = 0;
            DRV_SST25VF064C_BlockRead(serialflashData.driverHandle, &serialflashData.commandHandle[2],
                                      &serialflashData.targetBuffer[0], FLASH_ADDRESS_FOTA_DATA_MAGIC_BYTES,
                                      FLASH_LENGTH_FOTA_DATA_MAGIC_BYTES);
            if(serialflashData.commandHandle[2] == DRV_SST25VF064C_BLOCK_COMMAND_HANDLE_INVALID)
            {
                SYS_DEBUG(SYS_ERROR_ERROR, "FLASH: Failed to queue the read operation\r\n");
                serialflashData.state = APP_SERIALFLASH_ERROR;
                break;
            }
            serialflashData.state = APP_SERIALFLASH_WAIT_FOR_MAGIC_BYTES_FOTA;
            break;
        }
        // Wait for the scheduled operation to complete
        case APP_SERIALFLASH_WAIT_FOR_MAGIC_BYTES_FOTA:
        {
            if(serialflashData.eventMap == SERIALFLASH_BLOCK_OPERATION_COMPLETE)
            {
                serialflashData.state = APP_SERIALFLASH_VERIFY_MAGIC_BYTES_FOTA;
            }
            if(serialflashData.eventMap & SERIALFLASH_BLOCK_OPERATION_ERROR)
            {
                SYS_DEBUG(SYS_ERROR_ERROR, "FLASH: There was an error while processing a read operation\r\n");
                serialflashData.state = APP_SERIALFLASH_ERROR;
            }
            break;
        }
        // Verify if the magic bytes are set
        case APP_SERIALFLASH_VERIFY_MAGIC_BYTES_FOTA:
        {
            // Magic bytes: 0xDEADBEEF
            if((serialflashData.targetBuffer[0] == 0xDE) && (serialflashData.targetBuffer[1] == 0xAD) &&
               (serialflashData.targetBuffer[2] == 0xBE) && (serialflashData.targetBuffer[3] == 0xEF))
            {
                SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Magic bytes found: FOTA data present\r\n");
                serialflashData.has_fota_data = true;
            }
            else
            {
                SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Magic bytes not found: no stored FOTA data present\r\n");
                serialflashData.has_fota_data = false;
            }
            serialflashData.state = APP_SERIALFLASH_IDLE;
            break;
        }
        // Init done, switch to IDLE state

        // Schedule a read operation from the specified address with the specified length
        case APP_SERIALFLASH_READ_DATA:
        {
            serialflashData.commandHandle[0] = 0; // Erase
            serialflashData.commandHandle[1] = 0; // Write
            serialflashData.commandHandle[2] = 0; // Read
            serialflashData.eventMap         = 0;

            SYS_DEBUG(SYS_ERROR_DEBUG, "FLASH: Reading %u bytes from address %u\r\n", serialflashData.length,
                      serialflashData.address);

            DRV_SST25VF064C_BlockRead(serialflashData.driverHandle, &serialflashData.commandHandle[2],
                                      &serialflashData.targetBuffer[0], serialflashData.address,
                                      serialflashData.length);
            if(serialflashData.commandHandle[2] == DRV_SST25VF064C_BLOCK_COMMAND_HANDLE_INVALID)
            {
                SYS_DEBUG(SYS_ERROR_ERROR, "FLASH: Failed to queue the read operation\r\n");
                serialflashData.state = APP_SERIALFLASH_ERROR;
                break;
            }

            serialflashData.state = APP_SERIALFLASH_WAIT_FOR_READING_DATA;
            break;
        }
        // Wait for the scheduled operation to complete
        case APP_SERIALFLASH_WAIT_FOR_READING_DATA:
        {
            if(serialflashData.eventMap == SERIALFLASH_BLOCK_OPERATION_COMPLETE)
            {
                SYS_DEBUG(SYS_ERROR_DEBUG, "FLASH: Loading data succeeded\r\n");
                serialflashData.state = APP_SERIALFLASH_IDLE;
            }
            if(serialflashData.eventMap & SERIALFLASH_BLOCK_OPERATION_ERROR)
            {
                SYS_DEBUG(SYS_ERROR_ERROR, "FLASH: Loading data failed\r\n");
                serialflashData.state = APP_SERIALFLASH_ERROR;
            }
            break;
        }
        // Init done, switch back to IDLE state

        // Schedule a write (and read operation to verify if the write operation was successful) from the specified
        // address with the specified length
        case APP_SERIALFLASH_STORE_DATA:
        {
            serialflashData.commandHandle[0] = 0; // Erase
            serialflashData.commandHandle[1] = 0; // Write
            serialflashData.commandHandle[2] = 0; // Read
            serialflashData.eventMap         = 0;

            APP_SerialFlash_UnblockMemory();

            // Data needs to be page aligned to perform page writes instead of block writes (= mucho faster)

            // Store the length of the data we are trying to write
            nRemaining = serialflashData.length;

            // Check if the starting address is page aligned
            if((serialflashData.address % FLASH_PAGE_SIZE))
            {
                // The address isn't page aligned, so calculate the length we can use until the next page aligned
                // address The remainder of the data will be written in the next cycle
                if(serialflashData.length > (FLASH_PAGE_SIZE - (serialflashData.address % FLASH_PAGE_SIZE)))
                {
                    serialflashData.length = (FLASH_PAGE_SIZE - (serialflashData.address % FLASH_PAGE_SIZE));
                }
            }

            // NB. If the previous if statement was true, this will always be false
            if(serialflashData.length >= FLASH_PAGE_SIZE)
            {
                // We have more data than 1 page, so take the length of 1 page,
                // the remainder of the data will be written in the next cycle
                serialflashData.length = FLASH_PAGE_SIZE;
            }

            // Store the length of the remaining data for the next write cycle
            nRemaining -= serialflashData.length;

            // Schedule a write (and read for verification) operation
            SYS_DEBUG(SYS_ERROR_DEBUG, "FLASH: Writing %u bytes to address %u\r\n", serialflashData.length,
                      serialflashData.address);
            DRV_SST25VF064C_BlockWrite(serialflashData.driverHandle, &serialflashData.commandHandle[1],
                                       &serialflashData.sourceBuffer[0], serialflashData.address,
                                       serialflashData.length);
            if(serialflashData.commandHandle[1] == DRV_SST25VF064C_BLOCK_COMMAND_HANDLE_INVALID)
            {
                SYS_DEBUG(SYS_ERROR_ERROR, "FLASH: Failed to queue the write operation\r\n");
                serialflashData.state = APP_SERIALFLASH_ERROR;
                break;
            }
            DRV_SST25VF064C_BlockRead(serialflashData.driverHandle, &serialflashData.commandHandle[2],
                                      &serialflashData.targetBuffer[0], serialflashData.address,
                                      serialflashData.length);
            if(serialflashData.commandHandle[2] == DRV_SST25VF064C_BLOCK_COMMAND_HANDLE_INVALID)
            {
                SYS_DEBUG(SYS_ERROR_ERROR, "FLASH: Failed to queue the read operation\r\n");
                serialflashData.state = APP_SERIALFLASH_ERROR;
                break;
            }
            serialflashData.state = APP_SERIALFLASH_WAIT_FOR_STORING_DATA;
            break;
        }
        // Wait for the scheduled operations to complete
        case APP_SERIALFLASH_WAIT_FOR_STORING_DATA:
        {
            if(serialflashData.eventMap == SERIALFLASH_BLOCK_OPERATION_COMPLETE)
            {
                SYS_DEBUG(SYS_ERROR_DEBUG, "FLASH: Write done\r\n");
                serialflashData.state = APP_SERIALFLASH_VERIFY_STORED_DATA;
            }
            if(serialflashData.eventMap & SERIALFLASH_BLOCK_OPERATION_ERROR)
            {
                SYS_DEBUG(SYS_ERROR_ERROR, "FLASH: There was an error while processing the queued commands\r\n");
                serialflashData.state = APP_SERIALFLASH_ERROR;
            }
            break;
        }
        // Verify if the data read equals the data written
        case APP_SERIALFLASH_VERIFY_STORED_DATA:
        {
            uint32_t index;
            // Loop trough both buffers and see if they are equal for the given length
            for(index = 0; index < serialflashData.length; index++)
            {
                if(serialflashData.sourceBuffer[index] != serialflashData.targetBuffer[index])
                {
                    SYS_DEBUG(SYS_ERROR_ERROR, "FLASH: Failed to store data\r\n");
                    SYS_DEBUG(SYS_ERROR_ERROR, " - Address: %u Source: 0x%02X Target: 0x%02X\r\n",
                              (serialflashData.address + index), serialflashData.sourceBuffer[index],
                              serialflashData.targetBuffer[index]);
                    serialflashData.state = APP_SERIALFLASH_ERROR;
                    break;
                }
            }
            if(serialflashData.state != APP_SERIALFLASH_ERROR)
            {
                // The write operation was successful (and verified), so see if we need to updates any status variables
                SYS_DEBUG(SYS_ERROR_DEBUG, "FLASH: Data stored successfully\r\n");
                if((serialflashData.address >= FLASH_ADDRESS_WIFI_DATA) &&
                   (serialflashData.address < (FLASH_ADDRESS_WIFI_DATA + FLASH_LENGTH_WIFI_DATA)))
                {
                    serialflashData.has_wifi_data = true;
                }
                else if((serialflashData.address >= FLASH_ADDRESS_ACTIVATION_DATA) &&
                        (serialflashData.address < (FLASH_ADDRESS_ACTIVATION_DATA + FLASH_LENGTH_ACTIVATION_DATA)))
                {
                    serialflashData.has_activation_data = true;
                }
                else if((serialflashData.address >= FLASH_ADDRESS_FOTA_DATA_MAGIC_BYTES) &&
                        (serialflashData.address <
                         (FLASH_ADDRESS_FOTA_DATA_MAGIC_BYTES + FLASH_LENGTH_FOTA_DATA_MAGIC_BYTES)))
                {
                    serialflashData.has_fota_data = true;
                }
                else if(serialflashData.address >= FLASH_ADDRESS_FOTA_IMAGE)
                {
                    // Update the length of the data written of the FOTA image
                    serialflashData.image_length += serialflashData.length;
                    SYS_DEBUG(SYS_ERROR_DEBUG, "FLASH: Stored FOTA FW Image size: %u / %u Bytes\r\n",
                              serialflashData.image_length, serialflashData.expected_image_length);
                }

                // Check if we have more data remaining for a next write cycle, otherwise move to IDLE
                if(nRemaining > 0)
                {
                    // Check if the FOTA image storage flag is set
                    // If we are writing a FOTA image we maybe need to wait until the next
                    // APP_SERIALFLASH_StoreFOTAImage() call is made so we can hopefully fill another page
                    if(fota_image_storage)
                    {
                        if((nRemaining < FLASH_PAGE_SIZE) &&
                           (serialflashData.image_length + nRemaining < serialflashData.expected_image_length))
                        {
                            // There's not enough in the remainder to fill another page and
                            // we're still expecting more data, (total size written < total size expected)
                            // Move to IDLE and wait for another call to APP_SERIALFLASH_StoreFOTAImage()
                            fota_image_storage    = false;
                            serialflashData.state = APP_SERIALFLASH_IDLE;
                            break;
                        }
                    }
                    // Prepare for a next write cycle:
                    // 1. Move the remaining data to the front of the buffer
                    for(index = 0; index < nRemaining; index++)
                    {
                        serialflashData.sourceBuffer[index] =
                            serialflashData.sourceBuffer[index + serialflashData.length];
                    }
                    // 2. Update the address and length for a next write cycle
                    serialflashData.address += serialflashData.length;
                    serialflashData.length = nRemaining;
                    serialflashData.state  = APP_SERIALFLASH_STORE_DATA;
                }
                else
                {
                    // No remaining data, move to IDLE
                    serialflashData.state = APP_SERIALFLASH_IDLE;
                }
            }
            break;
        }

        // Schedule a erase operation from the specified address with the specified length (erase length is the amount
        // of sector(s), not the amount of bytes)
        case APP_SERIALFLASH_BLOCK_ERASE:
        {
            SYS_DEBUG(SYS_ERROR_DEBUG, "FLASH: Erasing %u sector(s) (%u Bytes) from address %u\r\n",
                      serialflashData.length, (serialflashData.length * FLASH_SECTOR_SIZE), serialflashData.address);
            serialflashData.commandHandle[0] = 0; // Erase
            serialflashData.commandHandle[1] = 0; // Write
            serialflashData.commandHandle[2] = 0; // Read
            serialflashData.eventMap         = 0;
            DRV_SST25VF064C_BlockErase(serialflashData.driverHandle, &serialflashData.commandHandle[0],
                                       serialflashData.address, serialflashData.length);
            if(serialflashData.commandHandle[0] == DRV_SST25VF064C_BLOCK_COMMAND_HANDLE_INVALID)
            {
                SYS_DEBUG(SYS_ERROR_ERROR, "FLASH: Failed to queue the erase operation\r\n");
                serialflashData.state = APP_SERIALFLASH_ERROR;
                break;
            }
            serialflashData.state = APP_SERIALFLASH_WAIT_FOR_BLOCK_ERASE;
            break;
        }
        // Wait for the scheduled operation to complete
        case APP_SERIALFLASH_WAIT_FOR_BLOCK_ERASE:
        {
            if(serialflashData.eventMap == SERIALFLASH_BLOCK_OPERATION_COMPLETE)
            {
                SYS_DEBUG(SYS_ERROR_DEBUG, "FLASH: Block erase succeeded\r\n");
                // The erase operation was successful, so see if we need to updates any status variables
                if((serialflashData.address >= FLASH_ADDRESS_WIFI_DATA) &&
                   (serialflashData.address < (FLASH_ADDRESS_WIFI_DATA + FLASH_SECTOR_SIZE)))
                {
                    serialflashData.has_wifi_data = false;
                }
                else if((serialflashData.address >= FLASH_ADDRESS_ACTIVATION_DATA) &&
                        (serialflashData.address < (FLASH_ADDRESS_ACTIVATION_DATA + FLASH_SECTOR_SIZE)))
                {
                    serialflashData.has_activation_data = false;
                }
                else if((serialflashData.address >= FLASH_ADDRESS_FOTA_DATA) &&
                        (serialflashData.address < (FLASH_ADDRESS_FOTA_DATA + FLASH_SECTOR_SIZE)))
                {
                    serialflashData.has_fota_data = false;
                }
                serialflashData.state = APP_SERIALFLASH_IDLE;
            }
            if(serialflashData.eventMap & SERIALFLASH_BLOCK_OPERATION_ERROR)
            {
                SYS_DEBUG(SYS_ERROR_ERROR, "FLASH: Block erase failed\r\n");
                serialflashData.state = APP_SERIALFLASH_ERROR;
            }
            break;
        }

        // Perform a chip erase
        case APP_SERIALFLASH_CHIP_ERASE:
        {
            APP_SerialFlash_ChipErase();
            serialflashData.state = APP_SERIALFLASH_WAIT_FOR_CHIP_ERASE;
            break;
        }
        // Wait for the chip erase to complete
        case APP_SERIALFLASH_WAIT_FOR_CHIP_ERASE:
        {
            if(APP_SerialFlash_IsBusy())
            {
                break;
            }
            SYS_DEBUG(SYS_ERROR_DEBUG, "FLASH: Chip erase complete\r\n");
            serialflashData.state = APP_SERIALFLASH_IDLE;
            break;
        }

        // IDLE state means all the operations are done and succesful
        // Wait for another call for a operation to be scheduled
        case APP_SERIALFLASH_IDLE:
            break;
        // ERROR state means something went wrong
        case APP_SERIALFLASH_ERROR:
            break;

        // Shouldn't be here, ever.
        default:
            serialflashData.state = APP_SERIALFLASH_ERROR;
            break;
    }
}

/*******************************************************************************
 */

/*
 * Function to check if serialflash is ready for a new operation
 */
bool APP_SERIALFLASH_IsReady(void)
{
    return (serialflashData.state == APP_SERIALFLASH_IDLE);
}

/*
 * Function to check if serialflash is in an error state
 */
bool APP_SERIALFLASH_HasError(void)
{
    return (serialflashData.state == APP_SERIALFLASH_ERROR);
}

/*
 * Function to check if the 'WiFi data is present'-flag is set
 */
bool APP_SERIALFLASH_HasWifiData(void)
{
    return serialflashData.has_wifi_data;
}

/*
 * Function to check if the 'Activation data is present'-flag is set
 */
bool APP_SERIALFLASH_HasActivationData(void)
{
    return serialflashData.has_activation_data;
}

/*
 * Function to check if the 'FOTA data is present'-flag is set
 */
bool APP_SERIALFLASH_HasFOTAData(void)
{
    return serialflashData.has_fota_data;
}

/*
 * Function to store firmware data:
 * 1. Current firmware checksum
 */
void APP_SERIALFLASH_SaveFirmwareData(uint8_t* sha256)
{
    SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Storing Firmware Data");
    SYS_DEBUG(SYS_ERROR_DEBUG, ":\r\n - %s", sha256);
    SYS_DEBUG(SYS_ERROR_INFO, "\r\n");

    serialflashData.address = FLASH_ADDRESS_FIRMWARE_DATA;
    serialflashData.length  = FLASH_LENGTH_FIRMWARE_DATA;

    APP_SerialFlash_PreFillData();

    uint16_t i = 0;
    for(i = 0; i < FLASH_LENGTH_FIRMWARE_DATA_SHA256; i++)
    {
        serialflashData.sourceBuffer[(FLASH_ADDRESS_FIRMWARE_DATA_SHA256 - FLASH_ADDRESS_FIRMWARE_DATA + i)] =
            sha256[i];
    }

    serialflashData.state = APP_SERIALFLASH_STORE_DATA;
}

/*
 * Function to read the Firmware data sector and store it in the buffer
 * This needs to be called before the get functions
 */
void APP_SERIALFLASH_LoadFirmwareData(void)
{
    SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Loading Firmware Data\r\n");

    serialflashData.address = FLASH_ADDRESS_FIRMWARE_DATA;
    serialflashData.length  = FLASH_LENGTH_FIRMWARE_DATA;

    APP_SerialFlash_PreFillData();

    serialflashData.state = APP_SERIALFLASH_READ_DATA;
}

/*
 * Function to erase the Firmware data sector
 */
void APP_SERIALFLASH_EraseFirmwareData(void)
{
    SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Erasing Firmware Data\r\n");

    serialflashData.address = FLASH_ADDRESS_FIRMWARE_DATA;
    serialflashData.length  = 1;

    serialflashData.state = APP_SERIALFLASH_BLOCK_ERASE;
}

/*
 * Function to get the Firmware checksum, stored in the buffer
 * APP_SERIALFLASH_LoadFirmwareData() needs to be called first
 * to load the firmware data into the buffer
 */
void APP_SERIALFLASH_GetFirmwareChecksum(uint8_t* data)
{
    memcpy(&data[0],
           (uint8_t*)&serialflashData.targetBuffer[(FLASH_ADDRESS_FIRMWARE_DATA_SHA256 - FLASH_ADDRESS_FIRMWARE_DATA)],
           FLASH_LENGTH_FIRMWARE_DATA_SHA256);
}

/*
 * Function to store WiFi data:
 * 1. Network type
 * 2. Security Mode
 * 3. SSID
 * 4. Security Key
 */
void APP_SERIALFLASH_SaveWifiData(uint8_t networktype, uint8_t securitymode, char* ssid, char* securitykey)
{
    SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Storing WiFi Data");
    SYS_DEBUG(SYS_ERROR_DEBUG, ":\r\n - %u\r\n - %u\r\n - %s\r\n - %s", networktype, securitymode, ssid, securitykey);
    SYS_DEBUG(SYS_ERROR_INFO, "\r\n");

    serialflashData.address = FLASH_ADDRESS_WIFI_DATA;
    serialflashData.length  = FLASH_LENGTH_WIFI_DATA;

    APP_SerialFlash_PreFillData();

    serialflashData.sourceBuffer[(FLASH_ADDRESS_WIFI_DATA_MAGIC_BYTES - FLASH_ADDRESS_WIFI_DATA)]     = 0xDE;
    serialflashData.sourceBuffer[(FLASH_ADDRESS_WIFI_DATA_MAGIC_BYTES - FLASH_ADDRESS_WIFI_DATA + 1)] = 0xAD;
    serialflashData.sourceBuffer[(FLASH_ADDRESS_WIFI_DATA_MAGIC_BYTES - FLASH_ADDRESS_WIFI_DATA + 2)] = 0xBE;
    serialflashData.sourceBuffer[(FLASH_ADDRESS_WIFI_DATA_MAGIC_BYTES - FLASH_ADDRESS_WIFI_DATA + 3)] = 0xEF;

    serialflashData.sourceBuffer[(FLASH_ADDRESS_WIFI_DATA_NETWORK_TYPE - FLASH_ADDRESS_WIFI_DATA)]  = networktype;
    serialflashData.sourceBuffer[(FLASH_ADDRESS_WIFI_DATA_SECURITY_MODE - FLASH_ADDRESS_WIFI_DATA)] = securitymode;

    uint16_t i = 0;
    for(i = 0; i < FLASH_LENGTH_WIFI_DATA_SSID; i++)
    {
        serialflashData.sourceBuffer[(FLASH_ADDRESS_WIFI_DATA_SSID - FLASH_ADDRESS_WIFI_DATA + i)] = ssid[i];
    }
    for(i = 0; i < FLASH_LENGTH_WIFI_DATA_SECURITY_KEY; i++)
    {
        serialflashData.sourceBuffer[(FLASH_ADDRESS_WIFI_DATA_SECURITY_KEY - FLASH_ADDRESS_WIFI_DATA + i)] =
            securitykey[i];
    }

    serialflashData.state = APP_SERIALFLASH_STORE_DATA;
}

/*
 * Function to read the Wifi data sector and store it in the buffer
 * This needs to be called before the get functions
 */
void APP_SERIALFLASH_LoadWifiData(void)
{
    SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Loading WiFi Data\r\n");

    serialflashData.address = FLASH_ADDRESS_WIFI_DATA;
    serialflashData.length  = FLASH_LENGTH_WIFI_DATA;

    APP_SerialFlash_PreFillData();

    serialflashData.state = APP_SERIALFLASH_READ_DATA;
}

/*
 * Function to erase the WiFi data sector
 */
void APP_SERIALFLASH_EraseWifiData(void)
{
    SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Erasing WiFi Data\r\n");

    serialflashData.address = FLASH_ADDRESS_WIFI_DATA;
    serialflashData.length  = 1;

    serialflashData.state = APP_SERIALFLASH_BLOCK_ERASE;
}

/*
 * Function to get the network type, stored in the buffer
 * APP_SERIALFLASH_LoadWifiData() needs to be called first
 * to load the wifi data into the buffer
 */
void APP_SERIALFLASH_GetNetworkType(uint8_t* data)
{
    memcpy(&data[0],
           (uint8_t*)&serialflashData.targetBuffer[(FLASH_ADDRESS_WIFI_DATA_NETWORK_TYPE - FLASH_ADDRESS_WIFI_DATA)],
           FLASH_LENGTH_WIFI_DATA_NETWORK_TYPE);
}

/*
 * Function to get the security mode, stored in the buffer
 * APP_SERIALFLASH_LoadWifiData() needs to be called first
 * to load the wifi data into the buffer
 */
void APP_SERIALFLASH_GetSecurityMode(uint8_t* data)
{
    memcpy(&data[0],
           (uint8_t*)&serialflashData.targetBuffer[(FLASH_ADDRESS_WIFI_DATA_SECURITY_MODE - FLASH_ADDRESS_WIFI_DATA)],
           FLASH_LENGTH_WIFI_DATA_SECURITY_MODE);
}

/*
 * Function to get the SSID, stored in the buffer
 * APP_SERIALFLASH_LoadWifiData() needs to be called first
 * to load the wifi data into the buffer
 */
void APP_SERIALFLASH_GetSSID(char* data)
{
    memcpy(&data[0], (char*)&serialflashData.targetBuffer[(FLASH_ADDRESS_WIFI_DATA_SSID - FLASH_ADDRESS_WIFI_DATA)],
           FLASH_LENGTH_WIFI_DATA_SSID);
}

/*
 * Function to get the security key, stored in the buffer
 * APP_SERIALFLASH_LoadWifiData() needs to be called first
 * to load the wifi data into the buffer
 */
void APP_SERIALFLASH_GetSecurityKey(char* data)
{
    memcpy(&data[0],
           (char*)&serialflashData.targetBuffer[(FLASH_ADDRESS_WIFI_DATA_SECURITY_KEY - FLASH_ADDRESS_WIFI_DATA)],
           FLASH_LENGTH_WIFI_DATA_SECURITY_KEY);
}

/*
 * Function to store Activation data:
 * 1. Gateway ID
 * 2. Gateway Key
 * 3. Account Server URL
 * 4. Activation Data locked?
 */
void APP_SERIALFLASH_SaveActivationData(char* id, char* key, char* url, bool locked)
{
    SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Storing Activation Data");
    SYS_DEBUG(SYS_ERROR_DEBUG, ":\r\n - %s\r\n - %s\r\n - %s", id, key, url);
    SYS_DEBUG(SYS_ERROR_INFO, "\r\n");

    serialflashData.address = FLASH_ADDRESS_ACTIVATION_DATA;
    serialflashData.length  = FLASH_LENGTH_ACTIVATION_DATA;

    APP_SerialFlash_PreFillData();

    serialflashData.sourceBuffer[(FLASH_ADDRESS_ACTIVATION_DATA_MAGIC_BYTES - FLASH_ADDRESS_ACTIVATION_DATA)] = 0xDE;
    serialflashData.sourceBuffer[(FLASH_ADDRESS_ACTIVATION_DATA_MAGIC_BYTES - FLASH_ADDRESS_ACTIVATION_DATA + 1)] =
        0xAD;
    serialflashData.sourceBuffer[(FLASH_ADDRESS_ACTIVATION_DATA_MAGIC_BYTES - FLASH_ADDRESS_ACTIVATION_DATA + 2)] =
        0xBE;
    serialflashData.sourceBuffer[(FLASH_ADDRESS_ACTIVATION_DATA_MAGIC_BYTES - FLASH_ADDRESS_ACTIVATION_DATA + 3)] =
        0xEF;

    uint16_t i = 0;
    for(i = 0; i < FLASH_LENGTH_ACTIVATION_DATA_GATEWAY_ID; i++)
    {
        serialflashData.sourceBuffer[(FLASH_ADDRESS_ACTIVATION_DATA_GATEWAY_ID - FLASH_ADDRESS_ACTIVATION_DATA + i)] =
            id[i];
    }
    for(i = 0; i < FLASH_LENGTH_ACTIVATION_DATA_GATEWAY_KEY; i++)
    {
        serialflashData.sourceBuffer[(FLASH_ADDRESS_ACTIVATION_DATA_GATEWAY_KEY - FLASH_ADDRESS_ACTIVATION_DATA + i)] =
            key[i];
    }
    for(i = 0; i < FLASH_LENGTH_ACTIVATION_DATA_ACCOUNT_SERVER_URL; i++)
    {
        serialflashData
            .sourceBuffer[(FLASH_ADDRESS_ACTIVATION_DATA_ACCOUNT_SERVER_URL - FLASH_ADDRESS_ACTIVATION_DATA + i)] =
            url[i];
    }
    serialflashData.sourceBuffer[FLASH_ADDRESS_ACTIVATION_DATA_LOCKED - FLASH_ADDRESS_ACTIVATION_DATA] =
        locked ? ACTIVATION_DATA_LOCKED : ACTIVATION_DATA_NOT_LOCKED;

    serialflashData.state = APP_SERIALFLASH_STORE_DATA;
}

/*
 * Function to set the activation data locked byte
 */
void APP_SERIALFLASH_LockActivationData(void)
{
    SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Lock Activation Data\r\n");

    serialflashData.address = FLASH_ADDRESS_ACTIVATION_DATA_LOCKED;
    serialflashData.length  = FLASH_LENGTH_ACTIVATION_DATA_LOCKED;

    APP_SerialFlash_PreFillData();

    serialflashData.sourceBuffer[0] = ACTIVATION_DATA_LOCKED;

    serialflashData.state = APP_SERIALFLASH_STORE_DATA;
}

/*
 * Function to read the Activation data sector and store it in the buffer
 * This needs to be called before the get functions
 */
void APP_SERIALFLASH_LoadActivationData(void)
{
    SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Loading Activation Data\r\n");

    serialflashData.address = FLASH_ADDRESS_ACTIVATION_DATA;
    serialflashData.length  = FLASH_LENGTH_ACTIVATION_DATA;

    APP_SerialFlash_PreFillData();

    serialflashData.state = APP_SERIALFLASH_READ_DATA;
}

/*
 * Function to erase the Activation data sector
 */
void APP_SERIALFLASH_EraseActivationData(void)
{
    SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Erasing Activation Data\r\n");

    serialflashData.address = FLASH_ADDRESS_ACTIVATION_DATA;
    serialflashData.length  = 1;

    serialflashData.state = APP_SERIALFLASH_BLOCK_ERASE;
}

/*
 * Function to get the gateway ID, stored in the buffer
 * APP_SERIALFLASH_LoadActivationData() needs to be called first
 * to load the activation data into the buffer
 */
void APP_SERIALFLASH_GetGatewayID(char* data)
{
    memcpy(&data[0],
           (char*)&serialflashData
               .targetBuffer[(FLASH_ADDRESS_ACTIVATION_DATA_GATEWAY_ID - FLASH_ADDRESS_ACTIVATION_DATA)],
           FLASH_LENGTH_ACTIVATION_DATA_GATEWAY_ID);
}

/*
 * Function to get the gateway Key, stored in the buffer
 * APP_SERIALFLASH_LoadActivationData() needs to be called first
 * to load the activation data into the buffer
 */
void APP_SERIALFLASH_GetGatewayKey(char* data)
{
    memcpy(&data[0],
           (char*)&serialflashData
               .targetBuffer[(FLASH_ADDRESS_ACTIVATION_DATA_GATEWAY_KEY - FLASH_ADDRESS_ACTIVATION_DATA)],
           FLASH_LENGTH_ACTIVATION_DATA_GATEWAY_KEY);
}

/*
 * Function to get the Account Server URL, stored in the buffer
 * APP_SERIALFLASH_LoadActivationData() needs to be called first
 * to load the activation data into the buffer
 */
void APP_SERIALFLASH_GetAccountServerURL(char* data)
{
    memcpy(&data[0],
           (char*)&serialflashData
               .targetBuffer[(FLASH_ADDRESS_ACTIVATION_DATA_ACCOUNT_SERVER_URL - FLASH_ADDRESS_ACTIVATION_DATA)],
           FLASH_LENGTH_ACTIVATION_DATA_ACCOUNT_SERVER_URL);
}

/*
 * Function to check if the locked byte is set, stored in the buffer
 * APP_SERIALFLASH_LoadActivationData() needs to be called first
 * to load the activation data into the buffer
 */
bool APP_SERIALFLASH_IsLocked(void)
{
    return serialflashData.targetBuffer[FLASH_ADDRESS_ACTIVATION_DATA_LOCKED - FLASH_ADDRESS_ACTIVATION_DATA] !=
           ACTIVATION_DATA_NOT_LOCKED;
}

/*
 * Function to read the FOTA data sector and store it in the buffer
 * This needs to be called before the get functions
 */
void APP_SERIALFLASH_LoadFOTAData(void)
{
    SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Loading FOTA Data\r\n");

    serialflashData.address = FLASH_ADDRESS_FOTA_DATA;
    serialflashData.length  = FLASH_LENGTH_FOTA_DATA;

    APP_SerialFlash_PreFillData();

    serialflashData.state = APP_SERIALFLASH_READ_DATA;
}

/*
 * Function to get the FOTA checksum, stored in the buffer
 * APP_SERIALFLASH_LoadFOTAData() needs to be called first
 * to load the FOTA data into the buffer
 */
void APP_SERIALFLASH_GetFOTAChecksum(uint8_t* data)
{
    memcpy(&data[0], (char*)&serialflashData.targetBuffer[(FLASH_ADDRESS_FOTA_DATA_SHA256 - FLASH_ADDRESS_FOTA_DATA)],
           FLASH_LENGTH_FOTA_DATA_SHA256);
}

/*
 * Function to initialze FOTA image storage
 * Stores the expected image size and initializes SHA256 calculation
 */
void APP_SERIALFLASH_InitFOTA(uint32_t image_length)
{
    // Check if there is enough room in the serialflash to store this amount of data
    if((FLASH_ADDRESS_FOTA_IMAGE + image_length) >= FLASH_MEMORY_SIZE)
    {
        SYS_DEBUG(SYS_ERROR_ERROR, "FLASH: Insufficient storage available\r\n");
        // serialflashData.state = APP_SERIALFLASH_ERROR;
        APP_SERIALFLASH_EraseFOTA();
    }
    else
    {
        serialflashData.image_length          = 0;            // Bytes written so far
        serialflashData.expected_image_length = image_length; // Total image size to be written

        // Initialize SHA256 calculation
        CRYPT_SHA256_Initialize(&serialflashData.sha256);
        CRYPT_SHA256_DataSizeSet(&serialflashData.sha256, serialflashData.expected_image_length);
    }
}

/*
 * Function to store FOTA checksum
 */
void APP_SERIALFLASH_SaveFOTAChecksum(uint8_t* sha256)
{
    SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Storing FOTA Checksum");
    SYS_DEBUG(SYS_ERROR_DEBUG, ":\r\n - %s", sha256);
    SYS_DEBUG(SYS_ERROR_INFO, "\r\n");

    serialflashData.address = FLASH_ADDRESS_FOTA_DATA_SHA256;
    serialflashData.length  = FLASH_LENGTH_FOTA_DATA_SHA256;

    APP_SerialFlash_PreFillData();

    uint16_t i = 0;
    for(i = 0; i < serialflashData.length; i++)
    {
        serialflashData.sourceBuffer[i] = sha256[i];
        serialflashData.sha256_expected[i] =
            sha256[i]; // Stored for validation with the calculated checksum after the image is written
    }

    serialflashData.state = APP_SERIALFLASH_STORE_DATA;
}

/*
 * Function to store a FOTA image datachunk
 */
void APP_SERIALFLASH_SaveFOTAImage(uint8_t* data, uint16_t length)
{
    ASSERT(length <= FLASH_SECTOR_SIZE, "length max sector size");
    if(serialflashData.image_length == 0)
    {
        SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Storing FOTA Image\r\n");
    }
    else
    {
        SYS_DEBUG(SYS_ERROR_DEBUG, "FLASH: Storing FOTA Image\r\n");
    }

    // First check if there is still data remaining from a previous write
    uint16_t i = 0;
    uint8_t  remainder[nRemaining];
    if(nRemaining > 0)
    {
        // Place the remaining data in a temporary buffer
        for(i = 0; i < nRemaining; i++)
        {
            remainder[i] = serialflashData.sourceBuffer[serialflashData.length + i];
        }
    }

    serialflashData.address = FLASH_ADDRESS_FOTA_IMAGE + serialflashData.image_length;
    serialflashData.length  = length + nRemaining;

    // Check if there is enough room available to store the data
    if((serialflashData.address + serialflashData.length) >= FLASH_MEMORY_SIZE)
    {
        SYS_DEBUG(SYS_ERROR_ERROR, "FLASH: Out of storage\r\n");
        APP_SERIALFLASH_FinalizeFOTA();
        return;
    }

    // Clear the buffers
    APP_SerialFlash_PreFillData();

    // Place the stored remainder in the beginning of the buffer to write
    for(i = 0; i < nRemaining; i++)
    {
        serialflashData.sourceBuffer[i] = remainder[i];
    }
    // Place the new data after this in the same buffer
    for(i = 0; i < length; i++)
    {
        serialflashData.sourceBuffer[nRemaining + i] = data[i];
    }
    // Add the newly passed data to the SHA256 calculation
    CRYPT_SHA256_DataAdd(&serialflashData.sha256, data, length);

    // Set the FOTA image storage flag, and start the write operation
    fota_image_storage    = true;
    serialflashData.state = APP_SERIALFLASH_STORE_DATA;
}

void APP_SERIALFLASH_FinalizeFOTA(void)
{
    // Check if the amount written matches the amount we expected to write
    if(serialflashData.image_length == serialflashData.expected_image_length)
    {
        // Finalize SHA256 calculation
        uint8_t sha256_calculated[FLASH_LENGTH_FOTA_DATA_SHA256];
        CRYPT_SHA256_Finalize(&serialflashData.sha256, sha256_calculated);

        // Check if the calculated FOTA checksum matches the stored FOTA checksum
        if(!memcmp(serialflashData.sha256_expected, sha256_calculated, FLASH_LENGTH_FOTA_DATA_SHA256))
        {
            SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Finalizing FOTA\r\n");
            // Set the magic bytes
            serialflashData.sourceBuffer[0] = 0xDE;
            serialflashData.sourceBuffer[1] = 0xAD;
            serialflashData.sourceBuffer[2] = 0xBE;
            serialflashData.sourceBuffer[3] = 0xEF;
            // Set the image size
            serialflashData.sourceBuffer[4] = (uint8_t)((serialflashData.image_length & 0xFF000000) >> 24);
            serialflashData.sourceBuffer[5] = (uint8_t)((serialflashData.image_length & 0x00FF0000) >> 16);
            serialflashData.sourceBuffer[6] = (uint8_t)((serialflashData.image_length & 0x0000FF00) >> 8);
            serialflashData.sourceBuffer[7] = (uint8_t)((serialflashData.image_length & 0x000000FF) >> 0);
            // Start the write operation
            serialflashData.address = FLASH_ADDRESS_FOTA_DATA_MAGIC_BYTES;
            serialflashData.length  = FLASH_LENGTH_FOTA_DATA_MAGIC_BYTES + FLASH_LENGTH_FOTA_DATA_IMAGE_LENGTH;
            serialflashData.state   = APP_SERIALFLASH_STORE_DATA;
        }
        else
        {
            SYS_DEBUG(SYS_ERROR_ERROR, "FLASH: Incorrect SHA256 Checksum\r\n");
            APP_SERIALFLASH_EraseFOTA();
        }
    }
    else
    {
        SYS_DEBUG(SYS_ERROR_ERROR, "FLASH: Stored FOTA FW size (%u) does not match the expected size (%u)\r\n",
                  serialflashData.image_length, serialflashData.expected_image_length);
        APP_SERIALFLASH_EraseFOTA();
    }
}

/*
 * Function to erase the FOTA data sector and all the following sectors (where the FOTA image is stored)
 */
void APP_SERIALFLASH_EraseFOTAData(void)
{
    SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Erasing FOTA Data\r\n");

    serialflashData.address = FLASH_ADDRESS_FOTA_DATA;
    serialflashData.length  = 1;

    serialflashData.state = APP_SERIALFLASH_BLOCK_ERASE;
}

/*
 * Function to erase the FOTA data sector and all the following sectors (where the FOTA image is stored)
 */
void APP_SERIALFLASH_EraseFOTA(void)
{
    SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Erasing FOTA Data and Image\r\n");

    serialflashData.address = FLASH_ADDRESS_FOTA_DATA;
    serialflashData.length  = ((FLASH_MEMORY_SIZE - FLASH_ADDRESS_FOTA_DATA) / FLASH_SECTOR_SIZE);

    serialflashData.state = APP_SERIALFLASH_BLOCK_ERASE;
}

/*
 * Function to erase the entire serialflash
 */
void APP_SERIALFLASH_EraseChip(void)
{
    SYS_DEBUG(SYS_ERROR_INFO, "FLASH: Erasing serial flash\r\n");
    serialflashData.has_wifi_data         = false;
    serialflashData.has_activation_data   = false;
    serialflashData.has_fota_data         = false;
    serialflashData.image_length          = 0;
    serialflashData.expected_image_length = 0;
    serialflashData.state                 = APP_SERIALFLASH_CHIP_ERASE;
}

/* *****************************************************************************
 End of File
 */
