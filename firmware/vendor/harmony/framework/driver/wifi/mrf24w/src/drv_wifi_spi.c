/*******************************************************************************
  MRF24WG Wi-Fi Driver SPI Communication Support

  File Name:
    drv_wifi_spi.c

  Summary:
    MRF24WG Wi-Fi Driver SPI Communication Support

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

#include "drv_wifi_priv.h"

#include "drv_wifi_debug_output.h"
#include "drv_wifi_spi.h"

/* 
 * Max SPI clock rate MRF24WG can support is 20 MHz. 
 * Here we check the validity based on the numbers from "SPI Clcok Reference Table.txt".
 * You can find the table at framework/driver/wifi directory.
 */
#if DRV_SPI_BAUDRATE > 19000000
#error "MRF2WG does not support the configured SPI clock rate. 20 MHz is the max rate it can support."
#endif 

#if defined(DRV_WIFI_USE_SPI_DMA) && !defined(DRV_WIFI_USE_FREERTOS)
#error "Wi-Fi Driver SPI DMA operation is not supported in bare-metal environment."
#elif defined(DRV_WIFI_USE_SPI_DMA) && (DRV_SPI_DMA == 0)
#error "DRV_SPI_DMA is disabled. It needs to be enabled to use SPI with DMA."
#endif

/* 
 * This macro allows to divide a long packet to smaller chunks and transfer them one by one in multiple SPI transactions.
 * When transferring the entire packet at a transaction is desired, set this value to 0. This is valid only for non-DMA operation. 
 */
#define MAX_SPI_RX_CHUNK_PER_TRANSACTION 100

#if defined(DRV_WIFI_USE_SPI_DMA)
#define WF_SYS_INT_STATUS_GET_AND_DISABLE() do { } while (0)
#define WF_SYS_INT_STATUS_RESTORE() do { } while (0)
#define SPI_DMA_DCACHE_CLEAN(addr, size) DRV_WIFI_DataCacheClean(addr, size)
#define SPI_TRANSACTION_COMPLETE_NOTIFY(sem) DRV_WIFI_ISR_SemGive(sem)
#define SPI_WAIT_FOR_RX_COMPLETION() DRV_WIFI_SemTake(&g_drv_wifi_priv.dmaRxSync, OSAL_WAIT_FOREVER)
#define SPI_WAIT_FOR_TX_COMPLETION() DRV_WIFI_SemTake(&g_drv_wifi_priv.dmaTxSync, OSAL_WAIT_FOREVER)
#define SPI_DMA_MAX_TX_SIZE DRV_SPI_DMA_TXFER_SIZE
#define SPI_DMA_MAX_RX_SIZE DRV_SPI_DMA_DUMMY_BUFFER_SIZE
#else /* !defined(DRV_WIFI_USE_SPI_DMA) */ 
#define WF_SYS_INT_STATUS_GET_AND_DISABLE() do { s_intStatusSaving = SYS_INT_StatusGetAndDisable(); } while (0)
#define WF_SYS_INT_STATUS_RESTORE() do { SYS_INT_StatusRestore(s_intStatusSaving); } while (0)
#define SPI_DMA_DCACHE_CLEAN(addr, size) do { } while (0)
#define SPI_TRANSACTION_COMPLETE_NOTIFY(sem) do { } while (0)
#define SPI_WAIT_FOR_RX_COMPLETION() do { } while (0)
#define SPI_WAIT_FOR_TX_COMPLETION() do { } while (0)
#define SPI_DMA_MAX_TX_SIZE -1 /* Unused. Just to avoid compilation error */
#define SPI_DMA_MAX_RX_SIZE  -1 /* Unused. Just to avoid compilation error */
#endif /* defined(DRV_WIFI_USE_SPI_DMA) */

#if defined(DRV_WIFI_USE_FREERTOS)
#define SPI_SEM_INIT() do { DRV_WIFI_SemInit(&g_drv_wifi_priv.dmaTxSync); DRV_WIFI_SemInit(&g_drv_wifi_priv.dmaRxSync); } while (0)
#define SPI_SEM_DEINIT() do { DRV_WIFI_SemDeInit(&g_drv_wifi_priv.dmaTxSync); DRV_WIFI_SemDeInit(&g_drv_wifi_priv.dmaRxSync); } while (0)
#else
#define SPI_SEM_INIT() do { } while (0)
#define SPI_SEM_DEINIT() do { } while (0)
#endif
static SYS_MODULE_OBJ *p_drvSPIObject = &DRV_WIFI_SPI_INSTANCE;
static DRV_HANDLE s_SpiHandle = NULL;
static DRV_SPI_BUFFER_HANDLE s_SpiBufferHandleTx;
static DRV_SPI_BUFFER_HANDLE s_SpiBufferHandleRx;
static volatile SYS_INT_PROCESSOR_STATUS s_intStatusSaving;

static bool (*s_spi_tx)(unsigned char *buf, uint32_t size) = NULL;
static bool (*s_spi_rx)(unsigned char *buf, uint32_t size) = NULL;

bool DRV_WIFI_SpiInit(bool (*rx)(unsigned char *buf, uint32_t size), bool (*tx)(unsigned char *buf, uint32_t size))
{
    WF_CS_Init();
    WF_CS_Deassert();

    if (s_SpiHandle == NULL) {
        s_SpiHandle = DRV_SPI_Open(DRV_WIFI_SPI_INDEX, DRV_IO_INTENT_READWRITE);
        if (s_SpiHandle == (DRV_SPI_BUFFER_HANDLE)NULL)
        {
            DRV_WIFI_ASSERT(false, "SPI init failed");
            return false;
        }
    }

    SPI_SEM_INIT();

    s_spi_tx = tx;
    s_spi_rx = rx;

    return true;
}

void DRV_WIFI_SpiClose()
{
    SPI_SEM_DEINIT();
    
    s_spi_tx = NULL;
    s_spi_rx = NULL;
}

static void SPI_DMA_TxComplete(DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE bufferHandle, void *context )
{
    SPI_TRANSACTION_COMPLETE_NOTIFY(&g_drv_wifi_priv.dmaTxSync);
}

static void SPI_DMA_RxComplete(DRV_SPI_BUFFER_EVENT event, DRV_SPI_BUFFER_HANDLE bufferHandle, void *context )
{
    SPI_TRANSACTION_COMPLETE_NOTIFY(&g_drv_wifi_priv.dmaRxSync);
}

static bool _DRV_WIFI_SpiDmaTx(unsigned char *buf, uint32_t size)
{
    bool ret = true;

    SPI_DMA_DCACHE_CLEAN(buf, size);
    s_SpiBufferHandleTx = DRV_SPI_BufferAddWrite(s_SpiHandle, buf, size, SPI_DMA_TxComplete, 0);
    if (s_SpiBufferHandleTx == (DRV_SPI_BUFFER_HANDLE)NULL)
        ret = false;

    SPI_WAIT_FOR_TX_COMPLETION();

    return ret;
}

static bool _DRV_WIFI_SpiDmaRx(unsigned char *buf, uint32_t size)
{
    bool ret = true;

    SPI_DMA_DCACHE_CLEAN(buf, size);
    s_SpiBufferHandleRx = DRV_SPI_BufferAddRead(s_SpiHandle, buf, size, SPI_DMA_RxComplete, 0);
    if (s_SpiBufferHandleRx == (DRV_SPI_BUFFER_HANDLE)NULL)
        ret = false;

    SPI_WAIT_FOR_RX_COMPLETION();
    return ret;
}

bool DRV_WIFI_SpiDmaTx(unsigned char *buf, uint32_t size)
{
    bool ret = true;
    int c = 0;
    
    while (size > SPI_DMA_MAX_TX_SIZE) {
        ret = _DRV_WIFI_SpiDmaTx(buf + c * SPI_DMA_MAX_TX_SIZE, SPI_DMA_MAX_TX_SIZE);
        size -= SPI_DMA_MAX_TX_SIZE;
        c ++;
    }
    
    if (size > 0)
        ret = _DRV_WIFI_SpiDmaTx(buf + c * SPI_DMA_MAX_TX_SIZE, size);

    return ret;
}

bool DRV_WIFI_SpiDmaRx(unsigned char *buf, uint32_t size)
{
    bool ret = true;
    int c = 0;
    
    while (size > SPI_DMA_MAX_RX_SIZE) {
        ret = _DRV_WIFI_SpiDmaRx(buf + c * SPI_DMA_MAX_RX_SIZE, SPI_DMA_MAX_RX_SIZE);
        size -= SPI_DMA_MAX_RX_SIZE;
        c ++;
    }
    
    if (size > 0)
        ret = _DRV_WIFI_SpiDmaRx(buf + c * SPI_DMA_MAX_RX_SIZE, size);

    return ret;
}

bool DRV_WIFI_SpiTx(unsigned char *buf, uint32_t size)
{
    s_SpiBufferHandleTx = DRV_SPI_BufferAddWrite(s_SpiHandle, buf, size, 0, 0);
    if (s_SpiBufferHandleTx == (DRV_SPI_BUFFER_HANDLE)NULL)
        return false;

    while (DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(s_SpiBufferHandleTx))
        DRV_SPI_Tasks(*p_drvSPIObject);

    return true;
}

bool DRV_WIFI_SpiWrite(unsigned char *buf, uint32_t size)
{
    bool ret;

    DRV_WIFI_ASSERT(s_spi_tx != NULL, ""); 

    if (size > 0) {
        WF_CS_Assert();
        ret = s_spi_tx(buf, size);
        WF_CS_Deassert();
        DRV_WIFI_ASSERT(ret, "Spi_Write failed"); 
        return ret;
    }

    return true;
}

bool DRV_WIFI_SpiWrite2(unsigned char *p_cmd, uint32_t cmdLen, unsigned char *p_txBuf, uint32_t txLen)
{
    bool ret = true;

    DRV_WIFI_ASSERT(s_spi_tx != NULL, ""); 

    WF_CS_Assert();
    ret = s_spi_tx(p_cmd, cmdLen);
    if (ret && txLen > 0)
        ret = s_spi_tx(p_txBuf, txLen);
    WF_CS_Deassert();

    DRV_WIFI_ASSERT(ret, "Spi_Write2 failed");
    return ret;
}

#if MAX_SPI_RX_CHUNK_PER_TRANSACTION > 0
bool DRV_WIFI_SpiRx(unsigned char *buf, uint32_t size)
{
    uint32_t chunk_size;
    uint32_t read_size;

    read_size = 0;
    WF_SYS_INT_STATUS_RESTORE();
    do {
        WF_SYS_INT_STATUS_GET_AND_DISABLE();

        chunk_size = size < MAX_SPI_RX_CHUNK_PER_TRANSACTION ? size : MAX_SPI_RX_CHUNK_PER_TRANSACTION;
        s_SpiBufferHandleRx = DRV_SPI_BufferAddRead(s_SpiHandle, buf + read_size, chunk_size, 0, 0);
        if (s_SpiBufferHandleRx == (DRV_SPI_BUFFER_HANDLE)NULL)
            return false;

        while (DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(s_SpiBufferHandleRx))
            DRV_SPI_Tasks(*p_drvSPIObject);

        size -= chunk_size;
        read_size += chunk_size;

        if (size > 0)
            WF_SYS_INT_STATUS_RESTORE();
    } while (size > 0);

    return true;
}
#else /* MAX_SPI_RX_CHUNK_PER_TRANSACTION == 0 */
bool DRV_WIFI_SpiRx(unsigned char *buf, uint32_t size)
{
    s_SpiBufferHandleRx = DRV_SPI_BufferAddRead(s_SpiHandle, buf, size, 0, 0);
    if (s_SpiBufferHandleRx == (DRV_SPI_BUFFER_HANDLE)NULL)
        return false;

    while (DRV_SPI_BUFFER_EVENT_COMPLETE != DRV_SPI_BufferStatus(s_SpiBufferHandleRx))
        DRV_SPI_Tasks(*p_drvSPIObject);

    return true;
}
#endif /* MAX_SPI_RX_CHUNK_PER_TRANSACTION > 0  */

bool DRV_WIFI_SpiRead(unsigned char *p_cmd, uint32_t cmdLen, unsigned char *p_rxBuf, uint32_t rxLen)
{
    bool ret = true;

    DRV_WIFI_ASSERT(s_spi_tx != NULL, ""); 

    WF_SYS_INT_STATUS_GET_AND_DISABLE();
    WF_CS_Assert();
    ret = s_spi_tx(p_cmd, cmdLen);
    if (ret && rxLen > 0)
        ret = s_spi_rx(p_rxBuf, rxLen);
    WF_CS_Deassert();
    WF_SYS_INT_STATUS_RESTORE();

    DRV_WIFI_ASSERT(ret, "Spi_Read failed");
    return ret;
}

//DOM-IGNORE-END
