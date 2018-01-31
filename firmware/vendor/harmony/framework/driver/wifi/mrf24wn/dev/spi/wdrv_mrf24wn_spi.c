/*******************************************************************************
  MRF24W SPI Driver

  File Name: 
    wdrv_mrf24wn_spi.c  
  
  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    Supports SPI communications to the MRF24W module.
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

#include "system_definitions.h"
#include "wdrv_mrf24wn_common.h"

/* 
 * Max SPI clock rate MRF24WN can support is 10 MHz. 
 * Here we check the validity based on the numbers from "SPI Clcok Reference Table.txt".
 * You can find the table at framework/driver/wifi directory.
 */
#if WDRV_BOARD_TYPE == WDRV_BD_TYPE_MZ_ESK || WDRV_BOARD_TYPE == WDRV_BD_TYPE_MEB2 || \
    WDRV_BOARD_TYPE == WDRV_BD_TYPE_MX_ESK || WDRV_BOARD_TYPE == WDRV_BD_TYPE_EXP16
#   if DRV_SPI_BAUDRATE > 10000000
#error "MRF2WN does not support the configured SPI clock rate. 10 MHz is the max rate it can support."
#   endif
#endif 

#if defined(WDRV_USE_SPI_DMA) && (DRV_SPI_DMA == 0)
#error "DRV_SPI_DMA is disabled. It needs to be enabled to use SPI with DMA."
#endif

#if defined(WDRV_USE_SPI_DMA)
#define SPI_DMA_DCACHE_CLEAN(addr, size) WDRV_DCACHE_CLEAN(addr, size)
#define SPI_TRANSACTION_COMPLETE_NOTIFY(sem) WDRV_SemGiveFromISR(sem)
#define SPI_WAIT_FOR_RX_COMPLETION() WDRV_SemTake(&g_wdrv_priv.dmaRxSync, OSAL_WAIT_FOREVER)
#define SPI_WAIT_FOR_TX_COMPLETION() WDRV_SemTake(&g_wdrv_priv.dmaTxSync, OSAL_WAIT_FOREVER)
#define SPI_DMA_MAX_TX_SIZE DRV_SPI_DMA_TXFER_SIZE
#define SPI_DMA_MAX_RX_SIZE DRV_SPI_DMA_DUMMY_BUFFER_SIZE
#else /* defined(WDRV_USE_SPI_DMA) */
#define SPI_DMA_DCACHE_CLEAN(addr, size) do { } while (0)
#define SPI_TRANSACTION_COMPLETE_NOTIFY(sem) do { } while (0)
#define SPI_WAIT_FOR_RX_COMPLETION() _Spi_WaitForCompletion(&s_Spi_Rx_Done)
#define SPI_WAIT_FOR_TX_COMPLETION() _Spi_WaitForCompletion(&s_Spi_Tx_Done)
#define SPI_DMA_MAX_TX_SIZE -1 /* Unused. Just to avoid compilation error */
#define SPI_DMA_MAX_RX_SIZE  -1 /* Unused. Just to avoid compilation error */
#endif /* defined(WDRV_USE_SPI_DMA) */

#if !defined(WDRV_USE_SPI_DMA)
static SYS_MODULE_OBJ *p_drvSPIObject = &WDRV_SPI_INSTANCE;
#endif
static DRV_HANDLE s_SpiHandle = NULL;
static DRV_SPI_BUFFER_HANDLE s_SpiBufferHandleTx;
static DRV_SPI_BUFFER_HANDLE s_SpiBufferHandleRx;
static volatile bool s_Spi_Tx_Done = true;
static volatile bool s_Spi_Rx_Done = true;

static void CS_Init(void)
{
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, WF_CS_PORT_CHANNEL, WF_CS_BIT_POS);
}

static void CS_Assert()
{
    SYS_PORTS_PinClear(PORTS_ID_0, WF_CS_PORT_CHANNEL, WF_CS_BIT_POS);
}

static void CS_Deassert()
{
    SYS_PORTS_PinSet(PORTS_ID_0, WF_CS_PORT_CHANNEL, WF_CS_BIT_POS);
}

static bool Spi_Init(void)
{
    if (s_SpiHandle == NULL) {
        s_SpiHandle = DRV_SPI_Open(WDRV_SPI_INDEX, DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_BLOCKING);
        if (s_SpiHandle == (DRV_SPI_BUFFER_HANDLE)NULL)
        {
            WDRV_ASSERT(false, "SPI init failed");
            return false;
        }
    }

    WDRV_SemInit(&g_wdrv_priv.dmaTxSync);
    WDRV_SemInit(&g_wdrv_priv.dmaRxSync);

    return true;
}

static void Spi_Close(void)
{
    WDRV_SemDeInit(&g_wdrv_priv.dmaTxSync);
    WDRV_SemDeInit(&g_wdrv_priv.dmaRxSync);
    //DRV_SPI_Close(s_SpiHandle);
}

static void SPI_TxComplete(DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE bufferHandle, void *context )
{
    s_Spi_Tx_Done = true;
    SPI_TRANSACTION_COMPLETE_NOTIFY(&g_wdrv_priv.dmaTxSync);
}

static void SPI_RxComplete(DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE bufferHandle, void *context )
{
    s_Spi_Rx_Done = true;
    SPI_TRANSACTION_COMPLETE_NOTIFY(&g_wdrv_priv.dmaRxSync);
}

#if !defined(WDRV_USE_SPI_DMA)
static void _Spi_WaitForCompletion(volatile bool *spiDone)
{
    while (*spiDone == false)
        DRV_SPI_Tasks(*p_drvSPIObject);
}
#endif

static bool _Spi_Tx(unsigned char *buf, uint32_t size)
{
    bool ret = true;

    SPI_DMA_DCACHE_CLEAN(buf, size);
    s_Spi_Tx_Done = false;
    s_SpiBufferHandleTx = DRV_SPI_BufferAddWrite(s_SpiHandle, buf, size, SPI_TxComplete, 0);
    if (s_SpiBufferHandleTx == (DRV_SPI_BUFFER_HANDLE)NULL)
        ret = false;

    SPI_WAIT_FOR_TX_COMPLETION();
    return ret;;
}

static bool _Spi_Rx(unsigned char *const buf, uint32_t size)
{
    bool ret = true;

    SPI_DMA_DCACHE_CLEAN(buf, size);
    s_Spi_Rx_Done = false;
    s_SpiBufferHandleRx = DRV_SPI_BufferAddRead(s_SpiHandle, buf, size, SPI_RxComplete, 0);
    if (s_SpiBufferHandleRx == (DRV_SPI_BUFFER_HANDLE)NULL)
        ret = false;

    SPI_WAIT_FOR_RX_COMPLETION();

    return ret;
}

static bool Spi_Read(unsigned char *p_cmd, uint32_t cmdLen, unsigned char *p_rxBuf, uint32_t rxLen)
{
    bool ret = true;
    int c = 0;

    CS_Assert();
    if (cmdLen > 0)
        ret = _Spi_Tx(p_cmd, cmdLen);

    if (ret && rxLen > 0) {
        while (rxLen > SPI_DMA_MAX_RX_SIZE) {
            ret = _Spi_Rx(p_rxBuf + c * SPI_DMA_MAX_RX_SIZE, SPI_DMA_MAX_RX_SIZE);
            rxLen -= SPI_DMA_MAX_RX_SIZE;
            c ++;
        }
        
        if (rxLen > 0)
            ret = _Spi_Rx(p_rxBuf + c * SPI_DMA_MAX_RX_SIZE, rxLen);
    }

    CS_Deassert();
    WDRV_ASSERT(ret, "Spi_Read failed");        
    return ret;
}

static bool Spi_Write(unsigned char *p_txBuf, uint32_t txLen)
{
    bool ret = true;
    int c = 0;
    
    CS_Assert();

    while (txLen > SPI_DMA_MAX_TX_SIZE) {
        ret = _Spi_Tx(p_txBuf + c * SPI_DMA_MAX_TX_SIZE, SPI_DMA_MAX_TX_SIZE);
        txLen -= SPI_DMA_MAX_TX_SIZE;
        c ++;
    }
    
    if (txLen > 0)
        ret = _Spi_Tx(p_txBuf + c * SPI_DMA_MAX_TX_SIZE, txLen);

    CS_Deassert();
    WDRV_ASSERT(ret, "Spi_Write failed");        
    return ret;
}

void WDRV_SPI_Out(uint8_t *const bufOut, uint16_t OutSize)
{
    bool intEnabled = SYS_INT_SourceDisable(MRF_INT_SOURCE);

    Spi_Write(bufOut, OutSize);
    if(intEnabled)
        WDRV_INTR_SourceEnable();
}

void WDRV_SPI_In(uint8_t *const OutBuf, uint16_t OutSize, uint8_t *const InBuf, uint16_t InSize)
{
    bool intEnabled = SYS_INT_SourceDisable(MRF_INT_SOURCE);

    Spi_Read(OutBuf, OutSize, InBuf, InSize);
    if(intEnabled)
        WDRV_INTR_SourceEnable();
}

void WDRV_SPI_Init(void)
{     
    CS_Init();     
    CS_Deassert();
    Spi_Init(); 
}

void WDRV_SPI_Deinit(void)
{
    Spi_Close();
}

//DOM-IGNORE-END
