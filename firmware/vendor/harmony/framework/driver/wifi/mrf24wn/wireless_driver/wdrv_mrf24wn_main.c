/*******************************************************************************
  MRF24WN Wireless Driver

  File Name:
    wdrv_mrf24wn_main.c

  Summary:
    Module for Microchip TCP/IP Stack PIC32 implementation
    for multiple MAC support

  Description:

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

#include "wdrv_mrf24wn_main.h"
#include "wdrv_mrf24wn_scan_helper.h"

#include "trcUser.h"
#include "trcTypes.h"

#define TCPIP_THIS_MODULE_ID TCPIP_MODULE_MAC_MRF24WN

#define ETH_HEADER_SIZE 14
#define NUM_PREALLOCATED_RX_PACKETS 8
#define MAX_IP_PACKET_SIZE 1564 // including header
#define MAX_RX_PACKET_SIZE 1518
#define MAX_TX_PACKET_SIZE 1518
#define MAX_MULTICAST_FILTER_SIZE 16

#define WAIT_FOR_DISCONNECT_COMPLETE() WDRV_SemTake(&g_wdrv_priv.disconnectDoneSync, OSAL_WAIT_FOREVER)

// used to keep track of RX packets queued for stack
typedef struct
{
    int front;
    int rear;
    TCPIP_MAC_PACKET *items[NUM_PREALLOCATED_RX_PACKETS + 1];
} t_fifo;

// MRF24WN wireless driver descriptor
typedef struct
{
    const TCPIP_MAC_OBJECT *pObj; // safe cast to TCPIP_MAC_DCPT
    TCPIP_NET_IF *pNetIf;         // interface we belong to
    bool isInit;                  // simple init status flag
    bool isOpen;                  // simple open status flag
    SYS_STATUS sysStat;           // driver status
} WDRV_MRF24WN_DCPT;

static _TCPIP_PKT_ALLOC_PTR s_pktAllocF = 0;
static _TCPIP_PKT_FREE_PTR s_pktFreeF = 0;
static _TCPIP_PKT_ACK_PTR s_pktAckF = 0;
static t_fifo s_rxFifo;
static SINGLE_LIST s_dataRxQueue; // queue of data rx packets waiting to be processed by stack from host
static TCPIP_MAC_PACKET *s_rxpacket_buffer[NUM_PREALLOCATED_RX_PACKETS];
static uint8_t *s_txpacket_buffer = NULL;
static TCPIP_MAC_MODULE_CTRL s_stackData;
static TCPIP_MAC_ADDR s_MulticastFilter[MAX_MULTICAST_FILTER_SIZE];
WDRV_MRF24WN_PRIV g_wdrv_priv =
{
    /* explicity initialize g_wdrv_priv.initConn to true */
    .initConn = true
};

static uint32_t s_wdrvext_config[] = {MODULE_EVENT_PRINT, WDRV_EXT_RTOS_INIT_TASK_PRIORITY, WDRV_EXT_RTOS_MAIN_TASK_PRIORITY,
    WDRV_EXT_RTOS_INIT_TASK_SIZE, WDRV_EXT_RTOS_MAIN_TASK_SIZE, WDRV_BOARD_TYPE};

static SYS_MODULE_OBJ WDRV_MRF24WN_Initialize(const SYS_MODULE_INDEX index, const SYS_MODULE_INIT *const init);
static void WDRV_MRF24WN_Reinitialize(SYS_MODULE_OBJ object, const SYS_MODULE_INIT *const init);
static void WDRV_MRF24WN_Deinitialize(SYS_MODULE_OBJ object);
static void WDRV_MRF24WN_Tasks(SYS_MODULE_OBJ object);
static SYS_STATUS WDRV_MRF24WN_Status(SYS_MODULE_OBJ object);
static DRV_HANDLE WDRV_MRF24WN_Open(const SYS_MODULE_INDEX drvIndex, const DRV_IO_INTENT intent);
static void WDRV_MRF24WN_Close(TCPIP_MAC_HANDLE hMac);
static TCPIP_MAC_RES WDRV_MRF24WN_RegisterStatisticsGet(DRV_HANDLE hMac, TCPIP_MAC_STATISTICS_REG_ENTRY *pRegEntries,
        int nEntries, int *pHwEntries);
static bool WDRV_MRF24WN_CheckLink(TCPIP_MAC_HANDLE hMac);
static bool WDRV_MRF24WN_PowerMode(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_POWER_MODE pwrMode);
static TCPIP_MAC_RES WDRV_MRF24WN_TxPacket(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PACKET *ptrPacket);
static TCPIP_MAC_PACKET *WDRV_MRF24WN_RxPacket (TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RES *pRes,
        const TCPIP_MAC_PACKET_RX_STAT **ppPktStat);
static TCPIP_MAC_RES WDRV_MRF24WN_Process(TCPIP_MAC_HANDLE hMac);
static TCPIP_MAC_RES WDRV_MRF24WN_StatisticsGet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RX_STATISTICS *pRxStatistics,
        TCPIP_MAC_TX_STATISTICS *pTxStatistics);
static TCPIP_MAC_RES WDRV_MRF24WN_ParametersGet(DRV_HANDLE hMac, TCPIP_MAC_PARAMETERS *pMacParams);
static bool WDRV_MRF24WN_EventSetMask(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents, bool enable);
static bool WDRV_MRF24WN_EventAcknowledge(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents);
static TCPIP_MAC_EVENT WDRV_MRF24WN_EventGet(TCPIP_MAC_HANDLE hMac);
static size_t WDRV_MRF24WN_GetConfig(TCPIP_MODULE_MAC_ID modId, void *configBuff, size_t buffSize,
    size_t *pConfigSize);
static void PowerDown(void);
static void PowerUp(void);
static void FifoInit(t_fifo *const p_fifo);
static bool isFifoEmpty(t_fifo *const p_fifo);
static void FifoInsert(t_fifo *const p_fifo, TCPIP_MAC_PACKET *p_packet);
static TCPIP_MAC_PACKET *FifoRemove(t_fifo *const p_fifo);
static TCPIP_MAC_PACKET *GetRxPacket(void);
static bool RxDataCallback(TCPIP_MAC_PACKET *pktHandle, const void *ackParam);
static void InitRxBuffer(void);
static void DeInitRxBuffer(void);
static void InitTxBuffer(void);
static void DeInitTxBuffer(void);
static void CopyFrameToStackPacketBufferCB(uint32_t len, uint8_t const *const frame);
TCPIP_MAC_RES WDRV_MRF24WN_MulticastFilterSet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_ADDR *DestMACAddr);

const TCPIP_MAC_OBJECT WDRV_MRF24WN_MACObject =
{
    TCPIP_MODULE_MAC_MRF24WN,
    "MRF24WN",
    WDRV_MRF24WN_Initialize,
    WDRV_MRF24WN_Deinitialize,
    WDRV_MRF24WN_Reinitialize,
    WDRV_MRF24WN_Status,
    WDRV_MRF24WN_Tasks,
    WDRV_MRF24WN_Open,
    WDRV_MRF24WN_Close,
    WDRV_MRF24WN_CheckLink,
    WDRV_MRF24WN_MulticastFilterSet,
    WDRV_MRF24WN_PowerMode,
    WDRV_MRF24WN_TxPacket,
    WDRV_MRF24WN_RxPacket,
    WDRV_MRF24WN_Process,
    WDRV_MRF24WN_StatisticsGet,
    WDRV_MRF24WN_ParametersGet,
    WDRV_MRF24WN_RegisterStatisticsGet,
    WDRV_MRF24WN_GetConfig,
    WDRV_MRF24WN_EventSetMask,
    WDRV_MRF24WN_EventAcknowledge,
    WDRV_MRF24WN_EventGet,
};

// only one hardware instance for now!
static WDRV_MRF24WN_DCPT wdrv_mrf24wn_dcpt =
{
    &WDRV_MRF24WN_MACObject,  // specific PIC32 MAC data
    0,                        // pNetIf
    0,                        // isInit
    0,                        // isOpen
    SYS_STATUS_UNINITIALIZED, // sysStat
};

static traceLabel _TRACE;

static void RFReadyCB(uint8_t const *const addr)
{
    g_wdrv_priv.updateMacAddressRequired = 1;
    memcpy(g_wdrv_priv.macAddr, addr, 6);
}

static void ScanDoneCB(uint32_t status)
{
    if (status == 0) { // 0 means success
        WDRV_EXT_ScanDoneSet();
        if (WDRV_EXT_CmdScanGet(&g_scanStatus.numberOfResults) == WDRV_SUCCESS) {
            SCAN_CLEAR_IN_PROGRESS(g_scanStatus.scanState);
            SCAN_SET_VALID(g_scanStatus.scanState);
            SCAN_SET_DISPLAY(g_scanStatus.scanState);
            g_wdrv_priv.isScanDone = true;
        }
    } else {
        WDRV_DBG_INFORM_PRINT(("Scan failed, status = %d\r\n", status));
    }
}

static void InitDoneCB(void)
{
    g_wdrv_priv.initDriverDone = true;
}

static void DeinitDoneCB(void)
{
    g_wdrv_priv.deinitDriverDone = true;
}

static TCPIP_MAC_RES WDRV_MRF24WN_RegisterStatisticsGet(DRV_HANDLE hMac, TCPIP_MAC_STATISTICS_REG_ENTRY *pRegEntries,
    int nEntries, int *pHwEntries)
{
    /* unsupported */
    return TCPIP_MAC_RES_OP_ERR;
}

static void WDRV_MRF24WN_Close(TCPIP_MAC_HANDLE hMac)
{
    WDRV_MRF24WN_DCPT *pMacD;

    pMacD = &wdrv_mrf24wn_dcpt;
    if (pMacD->isOpen == 1) {
        if (isLinkUp()) {
            g_wdrv_priv.isDisconnectRequested = true;
            if (WDRV_EXT_CmdDisconnect() == TCPIP_MAC_RES_OK)
                WAIT_FOR_DISCONNECT_COMPLETE();
            g_wdrv_priv.isDisconnectRequested = false;
        }
        pMacD->isOpen = 0;
    }
}

static bool WDRV_MRF24WN_CheckLink(TCPIP_MAC_HANDLE hMac)
{
    if (WDRV_CONFIG_PARAMS(networkType) == WDRV_NETWORK_TYPE_SOFT_AP) {
        // This is a workaround. For TCP stack users only mark SoftAP with link
        // if one or more clients are connected. Don't change the isLinkUp function
        // without fixing all the places where isLinkUp is used in WDRV.
        return WDRV_APHasClientsConnected();
    } else {
        return isLinkUp();
    }
}

static void MulticastFilter_Initialize(void)
{
    int i;
    static bool InitOnce = false;

    if (InitOnce == false) {
        InitOnce = true;
        for (i = 0; i < MAX_MULTICAST_FILTER_SIZE; i++) {
            memset(&s_MulticastFilter[i], 0, sizeof(TCPIP_MAC_ADDR));
        }
    }
}

static bool isMulticastAddrSet(TCPIP_MAC_ADDR *addr)
{
    int i;

    for (i = 0; i < MAX_MULTICAST_FILTER_SIZE; i++) {
        if (memcmp(addr, &s_MulticastFilter[i], 6) == 0)
            return true;
    }

    return false;
}

static void MulticastAddr_Set(TCPIP_MAC_ADDR *addr, int index)
{
     memcpy(&s_MulticastFilter[index], addr, 6);
}

TCPIP_MAC_RES WDRV_MRF24WN_MulticastFilterSet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_ADDR *DestMACAddr)
{
    int i;
    TCPIP_MAC_RES res;
    uint8_t all_zeros[6] = {0, 0, 0, 0, 0, 0};

    WDRV_MUTEX_LOCK(g_wdrv_priv.multicastFilterLock, OSAL_WAIT_FOREVER);

    do {
        if (isMulticastAddrSet(DestMACAddr)) {
            res = TCPIP_MAC_RES_OK;
            break;
        }

        for (i = 0; i < MAX_MULTICAST_FILTER_SIZE; i++) {
            if (memcmp(all_zeros, &s_MulticastFilter[i], 6) == 0)
                break;
        }

        if (i == MAX_MULTICAST_FILTER_SIZE) {
            WDRV_ASSERT(false, "Multicast filter is full\r\n");
            res = TCPIP_MAC_RES_OP_ERR;
            break;
        }

        MulticastAddr_Set(DestMACAddr, i);
        res = TCPIP_MAC_RES_OK;
    } while(0);

    WDRV_MUTEX_UNLOCK(g_wdrv_priv.multicastFilterLock);

    return res;
}

static bool WDRV_MRF24WN_PowerMode(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_POWER_MODE pwrMode)
{
    if (pwrMode == TCPIP_MAC_POWER_FULL)
        PowerUp();
    else if (pwrMode == TCPIP_MAC_POWER_DOWN)
        PowerDown();

    return true;
}

static TCPIP_MAC_RES WDRV_MRF24WN_TxPacket(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PACKET *ptrPacket)
{
    TCPIP_MAC_RES res = TCPIP_MAC_RES_OK;
    TCPIP_MAC_PACKET const *p_packet = ptrPacket;
    TCPIP_MAC_DATA_SEGMENT *p_seg;
    uint8_t *p_segData;
    uint32_t sendResult;
    uint16_t curIndex = 0;

    if (isLinkUp() == false) {
        WDRV_DBG_INFORM_MESSAGE(("MRF24WN is in unconnected state, dropped the Tx packet\r\n"));
        res = TCPIP_MAC_RES_PACKET_ERR;
        // call stack ack function to let it know packet was transmitted
        if (s_pktAckF)
            _TCPIP_PKT_ACK_BY_PTR(s_pktAckF, ptrPacket, TCPIP_MAC_PKT_ACK_TX_OK);
        else
            WDRV_ASSERT(false, "Should never happen");
        return res;
    }

    // write out the first segment to MRF, including prepended internal header
    p_seg = p_packet->pDSeg; // point to first segment
    p_segData = p_seg->segLoad;

    if (p_seg->segLen > MAX_TX_PACKET_SIZE) {
        WDRV_DBG_ERROR_PRINT(("Invalid packet length %d, dropped the Tx packet\r\n", p_seg->segLen));
        res = TCPIP_MAC_RES_PACKET_ERR;
        // call stack ack function to let it know packet was transmitted
        if (s_pktAckF)
            _TCPIP_PKT_ACK_BY_PTR(s_pktAckF, ptrPacket, TCPIP_MAC_PKT_ACK_TX_OK);
        else
            WDRV_ASSERT(false, "Should never happen");
        return res;
    }

    memcpy(s_txpacket_buffer, p_segData, p_seg->segLen);
    curIndex += p_seg->segLen;

    while (p_seg->next != NULL) {
        if (curIndex > MAX_TX_PACKET_SIZE) {
            WDRV_DBG_ERROR_PRINT(("Invalid packet length %d, dropped the Tx packet\r\n", curIndex));
            res = TCPIP_MAC_RES_PACKET_ERR;
            // call stack ack function to let it know packet was transmitted
            if (s_pktAckF)
                _TCPIP_PKT_ACK_BY_PTR(s_pktAckF, ptrPacket, TCPIP_MAC_PKT_ACK_TX_OK);
            else
                WDRV_ASSERT(false, "Should never happen");
            return res;
        }
        p_seg = p_seg->next;
        p_segData = p_seg->segLoad;
        memcpy(s_txpacket_buffer + curIndex, p_segData, p_seg->segLen);
        curIndex += p_seg->segLen;
    }

    sendResult = WDRV_EXT_DataSend(curIndex, s_txpacket_buffer);
    if (sendResult != 0) {
        res = TCPIP_MAC_RES_PACKET_ERR;
        WDRV_DBG_TRACE_MESSAGE(("No Tx buffer is available, dropped the packet\r\n"));
    } else {
        WDRV_DBG_TRACE_MESSAGE(("Sent packet to module\r\n"));
    }

    // call stack ack function to let it know packet was transmitted
    if (s_pktAckF)
        _TCPIP_PKT_ACK_BY_PTR(s_pktAckF, ptrPacket, TCPIP_MAC_PKT_ACK_TX_OK);
    else
        WDRV_ASSERT(false, "Should never happen");
    return res;
}

static TCPIP_MAC_PACKET *WDRV_MRF24WN_RxPacket (TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RES *pRes,
    const TCPIP_MAC_PACKET_RX_STAT **ppPktStat)
{
    WDRV_DBG_TRACE_MESSAGE(("Received packet from module\r\n"));
    return GetRxPacket();
}

static TCPIP_MAC_RES WDRV_MRF24WN_Process(TCPIP_MAC_HANDLE hMac)
{
    /* unsupported */
    return TCPIP_MAC_RES_OK;
}

static TCPIP_MAC_RES WDRV_MRF24WN_StatisticsGet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RX_STATISTICS *pRxStatistics,
    TCPIP_MAC_TX_STATISTICS *pTxStatistics)
{
    /* unsupported */
    return TCPIP_MAC_RES_OP_ERR;
}

static TCPIP_MAC_RES WDRV_MRF24WN_ParametersGet(DRV_HANDLE hMac, TCPIP_MAC_PARAMETERS *pMacParams)
{
    WDRV_MRF24WN_DCPT *pDcpt = (WDRV_MRF24WN_DCPT *)hMac;

    if (pDcpt->sysStat == SYS_STATUS_READY) {
        if (pMacParams) {
            memcpy(pMacParams->ifPhyAddress.v, s_stackData.ifPhyAddress.v, sizeof(pMacParams->ifPhyAddress));
            pMacParams->processFlags = (TCPIP_MAC_PROCESS_FLAG_RX | TCPIP_MAC_PROCESS_FLAG_TX);
            pMacParams->macType = TCPIP_MAC_TYPE_WLAN;
        }

        return TCPIP_MAC_RES_OK;
    }

    return TCPIP_MAC_RES_IS_BUSY;
}

static bool WDRV_MRF24WN_EventSetMask(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents, bool enable)
{
    return WDRV_TrafficEventMask(hMac, macEvents, enable);
}

static bool WDRV_MRF24WN_EventAcknowledge(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents)
{
    return WDRV_TrafficEventAck(hMac, macEvents);
}

static TCPIP_MAC_EVENT WDRV_MRF24WN_EventGet(TCPIP_MAC_HANDLE hMac)
{
    return  WDRV_TrafficEventGet(hMac);
}

static void PowerDown(void)
{
    WDRV_EXT_PowerUpDown(false);
}

static void PowerUp(void)
{
    WDRV_EXT_PowerUpDown(true);
}

static void FifoInit(t_fifo *const p_fifo)
{
    memset(p_fifo, 0x00, sizeof(t_fifo));
    p_fifo->front = p_fifo->rear = NUM_PREALLOCATED_RX_PACKETS - 1;
}

static bool isFifoEmpty(t_fifo *const p_fifo)
{
    return p_fifo->front == p_fifo->rear;
}

static void FifoInsert(t_fifo *const p_fifo, TCPIP_MAC_PACKET *p_packet)
{
    if (p_fifo->rear == NUM_PREALLOCATED_RX_PACKETS - 1)
        p_fifo->rear = 0;
    else
        ++p_fifo->rear;

    p_fifo->items[p_fifo->rear] = p_packet;
}

static TCPIP_MAC_PACKET *FifoRemove(t_fifo *const p_fifo)
{
    if (p_fifo->front == NUM_PREALLOCATED_RX_PACKETS - 1)
        p_fifo->front = 0;
    else
        ++p_fifo->front;

    return p_fifo->items[p_fifo->front];
}

// retrieve the oldest of the queued Rx packets to deliver to the stack
static TCPIP_MAC_PACKET *GetRxPacket(void)
{
    if (!isFifoEmpty(&s_rxFifo))
        return FifoRemove(&s_rxFifo);
    else
        return NULL; // signals no rx packet to process
}

static bool RxDataCallback(TCPIP_MAC_PACKET *pktHandle, const void *ackParam)
{
    ackParam = ackParam; // ignore warning

    vTracePrintF(_TRACE, "RxDataCallback %d", s_dataRxQueue.nNodes);

    if (pktHandle){
        // if this is packet allocated at init and is going to be reused
        if ((pktHandle->pDSeg->segFlags & TCPIP_MAC_SEG_FLAG_RX_STICKY) == TCPIP_MAC_SEG_FLAG_RX_STICKY) {
            pktHandle->pktFlags &= ~TCPIP_MAC_PKT_FLAG_QUEUED;
            TCPIP_Helper_SingleListTailAdd(&s_dataRxQueue, (SGL_LIST_NODE *)pktHandle); // add packet back to free list
        }
    } else {
        WDRV_ASSERT(false, "pktHandle cannot be null");
    }

    return false;
}

static void InitRxBuffer(void)
{
    int32_t i;
    // create a queue to hold pointers to preallocated Rx packets
    TCPIP_Helper_SingleListInitialize(&s_dataRxQueue);

    for (i = 0; i < NUM_PREALLOCATED_RX_PACKETS; ++i) {
        // preallocate Rx buffers to store Rx packets as they come in (1500 bytes data plus header and checksum)
        s_rxpacket_buffer[i] = s_pktAllocF ? _TCPIP_PKT_ALLOC_BY_PTR(s_pktAllocF, sizeof(TCPIP_MAC_PACKET), MAX_RX_PACKET_SIZE, 0) : 0; //_TCPIP_PKT_PacketAllocDebug

        if (s_rxpacket_buffer[i] != NULL) {
            s_rxpacket_buffer[i]->next = NULL;
            s_rxpacket_buffer[i]->ackFunc = RxDataCallback;
            s_rxpacket_buffer[i]->ackParam = NULL;
            s_rxpacket_buffer[i]->pktFlags = 0;
            s_rxpacket_buffer[i]->pDSeg->segFlags |= TCPIP_MAC_SEG_FLAG_RX_STICKY;
            TCPIP_Helper_SingleListTailAdd(&s_dataRxQueue, (SGL_LIST_NODE *)s_rxpacket_buffer[i]);
        } else {
            WDRV_ASSERT(false, "");
        }
    }

    FifoInit(&s_rxFifo);
}

static void DeInitRxBuffer(void)
{
    int i;

    if (s_pktFreeF != NULL) {
        for (i = 0; i < NUM_PREALLOCATED_RX_PACKETS; ++i) {
            if (s_rxpacket_buffer[i] != NULL) {
                _TCPIP_PKT_FREE_BY_PTR(s_pktFreeF, s_rxpacket_buffer[i]);
                s_rxpacket_buffer[i] = NULL;
            }
        }
    }
}

static void InitTxBuffer(void)
{
    s_txpacket_buffer = WDRV_MALLOC(MAX_IP_PACKET_SIZE);
    if (s_txpacket_buffer == NULL) {
        WDRV_ASSERT(false, "");
    }
}

static void DeInitTxBuffer(void)
{
    if (s_txpacket_buffer != NULL) {
        WDRV_FREE(s_txpacket_buffer);
        s_txpacket_buffer = NULL;
    }
}

// Finds an available RX packet structure from the list that was allocated and
// queued up at initialization process.
static TCPIP_MAC_PACKET *GetAvailRxBuf(void)
{
    TCPIP_MAC_PACKET *p_packet = NULL;

    vTracePrintF(_TRACE, "GetAvailRxBuf %d", s_dataRxQueue.nNodes);
    
    if (s_dataRxQueue.nNodes > 0) {
        p_packet = (TCPIP_MAC_PACKET *)TCPIP_Helper_SingleListHeadRemove(&s_dataRxQueue);
        WDRV_ASSERT(p_packet != NULL, "Should never happen");
    }

    return p_packet;
}

static bool isBroadcastPacket(const uint8_t *addr)
{
    int i;

    for (i = 0; i < 6; i++) {
        if (addr[i] != 0xff)
            return false;
    }

    return true;
}

static bool isMulticastPacket(uint8_t const *const frame)
{
    if (((frame[0] & 0x01) == 0x01) && (!isBroadcastPacket(&frame[0])))
        return true;
    else
        return false;
}

static bool isSolicitedMulticastPacket(uint8_t const *const frame)
{
    return isMulticastAddrSet((TCPIP_MAC_ADDR *)frame);
}

static bool isPacketValid(uint8_t const *const frame)
{
    if (isMulticastPacket(frame)) {
        if (isSolicitedMulticastPacket(frame))
            return true;
        else
            return false;
    } else {
        return true;
    }

    return false;
}

static void CopyFrameToStackPacketBufferCB(uint32_t len, uint8_t const *const frame)
{
    TCPIP_MAC_PACKET *p_packet;

    WDRV_DBG_TRACE_MESSAGE(("Received packet\r\n"));

    if (isPacketValid(frame) == false)
        return;

    if (isLinkUp() == false) {
        WDRV_DBG_INFORM_MESSAGE(("MRF24WN is in unconnected state, the received packet will be discarded\r\n"));
        return;
    }

    p_packet = GetAvailRxBuf();

    if (p_packet == NULL) {
        while(1) {SYS_DEBUG_BreakPoint();}
        WDRV_DBG_INFORM_MESSAGE(("No Rx buffer is available, the received packet will be dropped\r\n"));
        return;
    }

    // mark packet as queued and stuff in timestamp
    p_packet->pDSeg->segLen = len - ETH_HEADER_SIZE;
    p_packet->pktFlags |= TCPIP_MAC_PKT_FLAG_QUEUED;
    p_packet->tStamp = SYS_TMR_TickCountGet();

    // Note: re-set pMacLayer and pNetLayer; IPv6 changes these pointers inside the packet, so
    //       when Rx packets are reused this is needed.
    memcpy(p_packet->pDSeg->segLoad, frame, len);
    p_packet->pMacLayer = p_packet->pDSeg->segLoad;
    p_packet->pNetLayer = p_packet->pMacLayer + sizeof(TCPIP_MAC_ETHERNET_HEADER);

    // store packet pointer in FIFO and signal stack that rx packet ready to process
    FifoInsert(&s_rxFifo, p_packet);

    // notify stack of Rx packet has arrived.
    WDRV_TrafficEventReq(TCPIP_EV_RX_DONE, 0);
}

static void Gpio_Initialize(void)
{
#define GPIO_OUTLOW_FUNC(HW) WDRV_GPIO_OutLow_##HW
#define GPIO_OUTHIGH_FUNC(HW) WDRV_GPIO_OutHigh_##HW

    GPIO_OUTLOW_T gpioOutLow;
    GPIO_OUTHIGH_T gpioOutHigh;

#if WDRV_BOARD_TYPE == WDRV_BD_TYPE_MZ_ESK
    gpioOutLow = GPIO_OUTLOW_FUNC(PIC32MZ_ESK);
    gpioOutHigh = GPIO_OUTHIGH_FUNC(PIC32MZ_ESK);
#elif WDRV_BOARD_TYPE == WDRV_BD_TYPE_MX_ESK
    gpioOutLow = GPIO_OUTLOW_FUNC(PIC32MX_ESK);
    gpioOutHigh = GPIO_OUTHIGH_FUNC(PIC32MX_ESK);
#elif WDRV_BOARD_TYPE == WDRV_BD_TYPE_EXP16
    gpioOutLow = GPIO_OUTLOW_FUNC(PIC32MX_EXP16);
    gpioOutHigh = GPIO_OUTHIGH_FUNC(PIC32MX_EXP16);
#elif WDRV_BOARD_TYPE == WDRV_BD_TYPE_MEB2
    gpioOutLow = GPIO_OUTLOW_FUNC(PIC32MZ_MEB2);
    gpioOutHigh = GPIO_OUTHIGH_FUNC(PIC32MZ_MEB2);
#elif WDRV_BOARD_TYPE ==  WDRV_BD_TYPE_CUSTOM
    gpioOutLow = GPIO_OUTLOW_FUNC(Custom_Board);
    gpioOutHigh = GPIO_OUTHIGH_FUNC(Custom_Board);
#endif

    WDRV_GPIO_Init(gpioOutLow, gpioOutHigh);
}

static SYS_MODULE_OBJ WDRV_MRF24WN_Initialize(const SYS_MODULE_INDEX index, const SYS_MODULE_INIT *const init)
{
    WDRV_MRF24WN_DCPT *pDcpt;
    const TCPIP_MAC_MODULE_CTRL *const stackData = ((TCPIP_MAC_INIT *)init)->macControl;
    WDRV_CALLBACKS CB;

    WDRV_ASSERT((stackData != NULL), "stackData is null");
    
    _TRACE	  = xTraceOpenLabel("Wifi Events");

    pDcpt = &wdrv_mrf24wn_dcpt; // no other instance supported
    if (pDcpt->isOpen != 0)
        return (SYS_MODULE_OBJ)pDcpt; // already initialized, have a client connected

    /* initialize g_wdrv_priv, intentionally exclude initConn */
    g_wdrv_priv.initDriverDone = false;
    g_wdrv_priv.deinitDriverDone = false;
    g_wdrv_priv.updateMacAddressRequired = false;
    g_wdrv_priv.isScanDone = false;
    g_wdrv_priv.isConnReestablished = false;
    g_wdrv_priv.isDisconnectRequested = false;
    WDRV_SemInit(&g_wdrv_priv.disconnectDoneSync);
    WDRV_MUTEX_CREATE(&g_wdrv_priv.debugConsoleLock);
    WDRV_MUTEX_CREATE(&g_wdrv_priv.multicastFilterLock);
    memset(g_wdrv_priv.macAddr, 0, sizeof(uint8_t) * 6);

    WDRV_DBG_INFORM_MESSAGE(("MRF24WN: Initializing . . .\r\n")); // Only after created mutex

    s_stackData = *stackData;
    s_pktAllocF = (_TCPIP_PKT_ALLOC_PTR)stackData->pktAllocF;
    s_pktFreeF = (_TCPIP_PKT_FREE_PTR)stackData->pktFreeF;
    s_pktAckF = (_TCPIP_PKT_ACK_PTR)stackData->pktAckF;

    if (stackData->moduleId != TCPIP_MODULE_MAC_MRF24WN)
        return SYS_MODULE_OBJ_INVALID; // only type MRF24WN is supported

    WDRV_TrafficEventInit(stackData->eventF, stackData->eventParam);

    InitTxBuffer();
    InitRxBuffer();
    WDRV_AllEventClear();

#if defined(TCPIP_STACK_COMMANDS_WIFI_ENABLE)
    WDRV_CLI_Init();
#endif

    Gpio_Initialize();
    WDRV_SPI_Init();

#if (WDRV_BOARD_TYPE == WDRV_BD_TYPE_MEB2)
    /*
     * MEB II (Multimedia Expansion Board II) comes out with the built-in MRF24WG module on it.
     * MRF24WG shares the same SPI bus lines with MRF24WN, which results in bus collision.
     * To avoid the collision, it is required to disable MRF24WG module with MEB II.
     */
    WDRV_GPIO_MRF24WG_Disable();
#endif

    WDRV_EXT_Misc_Config(s_wdrvext_config);

    CB.CopyFrameToStackPacketBuffer_CB = CopyFrameToStackPacketBufferCB;
    CB.ProceedConnectEvent_CB = ProceedConnectEventCB;
    CB.RFReady_CB = RFReadyCB;
    CB.ScanDone_CB = ScanDoneCB;
    CB.InitDone_CB = InitDoneCB;
    CB.DeinitDone_CB = DeinitDoneCB;
    CB.WPSDone_CB = WPSDoneCB;
    WDRV_EXT_Initialize(&CB);

    //if (!WDRV_CONFIG_DataLoad())      // NOTE: the data is loaded by application
    //    return SYS_MODULE_OBJ_INVALID;

    pDcpt->isInit = true;
    pDcpt->sysStat = SYS_STATUS_BUSY;

    return (SYS_MODULE_OBJ)pDcpt;
}

static void WDRV_MRF24WN_Tasks(SYS_MODULE_OBJ object)
{
    static uint32_t startTick = 0;
    static bool connect_start_wait = false;
    TCPIP_MAC_RES res = TCPIP_MAC_RES_PENDING;
    WDRV_MRF24WN_DCPT *pDcpt = (WDRV_MRF24WN_DCPT *)object;

    switch(pDcpt->sysStat) {
    case SYS_STATUS_UNINITIALIZED:
        break;
    case SYS_STATUS_BUSY:
        if (g_wdrv_priv.updateMacAddressRequired) {
            memcpy((uint8_t *)(s_stackData.ifPhyAddress.v), g_wdrv_priv.macAddr, 6);
            WDRV_DBG_INFORM_PRINT(("## MAC address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                    s_stackData.ifPhyAddress.v[0], s_stackData.ifPhyAddress.v[1],
                    s_stackData.ifPhyAddress.v[2], s_stackData.ifPhyAddress.v[3],
                    s_stackData.ifPhyAddress.v[4], s_stackData.ifPhyAddress.v[5]));
            g_wdrv_priv.updateMacAddressRequired = 0;
        }

        if (g_wdrv_priv.initDriverDone) {
            if (wdrv_mrf24wn_dcpt.isOpen == 0) {
                /* In WPS mode, before connection, wait for 3 seconds */
                if (WDRV_CONFIG_PARAMS(securityMode) == WDRV_SECURITY_WPS_PUSH_BUTTON ||
                    WDRV_CONFIG_PARAMS(securityMode) == WDRV_SECURITY_WPS_PIN) {
                    if (connect_start_wait == false) {
                        connect_start_wait = true;
                        startTick = SYS_TMR_TickCountGet();
                    } else {
                        if (SYS_TMR_TickCountGet() - startTick >= SYS_TMR_TickCounterFrequencyGet() * 3ul) {
                            if (g_wdrv_priv.initConn)
                                res = WDRV_Connect();
                            else
                                res = TCPIP_MAC_RES_OK;
                        }
                    }
                } else {
                    if (g_wdrv_priv.initConn)
                        res = WDRV_Connect();
                    else
                        res = TCPIP_MAC_RES_OK;
                }

                if (res == TCPIP_MAC_RES_OK)
                    wdrv_mrf24wn_dcpt.isOpen = 1;
            } else {
                res = TCPIP_MAC_RES_OK;
            }
        }

        if (res == TCPIP_MAC_RES_OK) {
            pDcpt->sysStat = SYS_STATUS_READY;
            WDRV_DBG_TRACE_MESSAGE(("MRF24WN: Initialization is complete\r\n"));
        } else if (res != TCPIP_MAC_RES_PENDING) {
            WDRV_CONFIG_DataSave();
            WDRV_AllEventClear();
            WDRV_TrafficEventDeinit();
            pDcpt->sysStat = SYS_STATUS_ERROR;
        }
        break;
    case SYS_STATUS_READY:
    case SYS_STATUS_READY_EXTENDED:
        if (isEventPending())
            WDRV_PendingEventProcess();
        break;
    default: // SYS_STATUS_ERROR
        WDRV_ASSERT(false, "Should never happen");
        break;
    }
}

static void WDRV_MRF24WN_Reinitialize(SYS_MODULE_OBJ object, const SYS_MODULE_INIT *const init)
{
    WDRV_DBG_INFORM_MESSAGE(("MRF24WN: Re-initializing . . .[UNSUPPORTED]\r\n"));
    /* unsupported */
}

static void WDRV_MRF24WN_Deinitialize(SYS_MODULE_OBJ object)
{
    WDRV_MRF24WN_DCPT *pDcpt;

    pDcpt = &wdrv_mrf24wn_dcpt; // no other instance supported

    WDRV_DBG_INFORM_MESSAGE(("MRF24WN: De-initializing . . .\r\n"));
    WDRV_CONFIG_DataSave();
    WDRV_AllEventClear();
    WDRV_TrafficEventDeinit();
    DeInitRxBuffer();
    DeInitTxBuffer();
    WDRV_EXT_Deinitialize();
    WDRV_GPIO_DeInit();
    WDRV_SPI_Deinit();

    WDRV_MUTEX_DELETE(&g_wdrv_priv.debugConsoleLock);
    WDRV_MUTEX_DELETE(&g_wdrv_priv.multicastFilterLock);
    WDRV_SemDeInit(&g_wdrv_priv.disconnectDoneSync);
    pDcpt->isInit = false;
}

static SYS_STATUS WDRV_MRF24WN_Status(SYS_MODULE_OBJ object)
{
    WDRV_MRF24WN_DCPT *pDcpt = (WDRV_MRF24WN_DCPT *)object;

    return pDcpt->sysStat;
}

static size_t WDRV_MRF24WN_GetConfig(TCPIP_MODULE_MAC_ID modId, void *configBuff, size_t buffSize, size_t *pConfigSize)
{
    /* unsupported */
    return 0;
}

static DRV_HANDLE WDRV_MRF24WN_Open(const SYS_MODULE_INDEX drvIndex, const DRV_IO_INTENT intent)
{
    WDRV_MRF24WN_DCPT *pMacD;
    DRV_HANDLE hMac = DRV_HANDLE_INVALID;
    TCPIP_MAC_RES res = TCPIP_MAC_RES_PENDING;

    pMacD = &wdrv_mrf24wn_dcpt;
    if (pMacD->isOpen == 0) {
        MulticastFilter_Initialize();
        if (g_wdrv_priv.initDriverDone) {
            if (g_wdrv_priv.initConn) {
                res = WDRV_Connect();
                if (res == TCPIP_MAC_RES_OK)
                    pMacD->isOpen = 1;
            } else {
                pMacD->isOpen = 1;
            }
        }
        hMac = (DRV_HANDLE)pMacD;
    }

    return hMac;
}

bool isMacInitialized(void)
{
    WDRV_MRF24WN_DCPT *pDcpt;

    pDcpt = &wdrv_mrf24wn_dcpt; // no other instance supported
    return pDcpt->isInit;
}

//DOM-IGNORE-END
