/*******************************************************************************
  MRF24WG Driver Medium Access Control (MAC) Layer

  File Name:
    drv_wifi_main.c

  Summary:
    MRF24WG Driver Medium Access Control (MAC) Layer

  Description:
    The functions in this file are accessed by the TCP/IP stack via
    function pointers.
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
#include "drv_wifi_mac.h"

#define MAX_MULTICAST_FILTER_SIZE 16

static SYS_MODULE_OBJ MRF24W_MACInitialize(const SYS_MODULE_INDEX index, const SYS_MODULE_INIT *const init);
static void MRF24W_MACDeinitialize(SYS_MODULE_OBJ object);
static void MRF24W_MACReinitialize(SYS_MODULE_OBJ object, const SYS_MODULE_INIT *const init);
static SYS_STATUS MRF24W_MACStatus(SYS_MODULE_OBJ object);
static DRV_HANDLE MRF24W_MACOpen(const SYS_MODULE_INDEX drvIndex, const DRV_IO_INTENT intent);
static void MRF24W_MACTasks(SYS_MODULE_OBJ object);

static void MRF24W_MACClose(TCPIP_MAC_HANDLE hMac);
static bool MRF24W_MACCheckLink(TCPIP_MAC_HANDLE hMac);
static bool MRF24W_MACPowerMode(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_POWER_MODE pwrMode);
static TCPIP_MAC_RES MRF24W_MACTxPacket(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PACKET *ptrPacket);
static TCPIP_MAC_PACKET *MRF24W_MACRxPacket(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RES *pRes,
    const TCPIP_MAC_PACKET_RX_STAT **ppPktStat);
static TCPIP_MAC_RES MRF24W_MACProcess(TCPIP_MAC_HANDLE hMac);
static TCPIP_MAC_RES MRF24W_MACStatisticsGet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RX_STATISTICS *pRxStatistics,
    TCPIP_MAC_TX_STATISTICS *pTxStatistics);
static TCPIP_MAC_RES MRF24W_MACParametersGet(DRV_HANDLE hMac, TCPIP_MAC_PARAMETERS *pMacParams);
static TCPIP_MAC_RES MRF24W_MACRegisterStatisticsGet(DRV_HANDLE hMac, TCPIP_MAC_STATISTICS_REG_ENTRY *pRegEntries,
    int nEntries, int *pHwEntries);
static size_t MRF24W_MACGetConfig(DRV_HANDLE hMac, void *configBuff, size_t buffSize, size_t *pConfigSize);

static bool MRF24W_MACEventSetMask(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents, bool enable);
static bool MRF24W_MACEventAcknowledge(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents);
static TCPIP_MAC_EVENT MRF24W_MACEventGetPending(TCPIP_MAC_HANDLE hMac);
static TCPIP_MAC_RES _MRF24W_MACOpen(void);
static TCPIP_MAC_RES _MRF24W_MACProcess(void);
static void _MRF24W_MACDeinitialize(MRF24W_MAC_DCPT *pDcpt);
static void MulticastFilter_Initialize(void);
TCPIP_MAC_RES MRF24W_MACMulticastFilterSet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_ADDR *DestMACAddr);


// the PIC32 MRF24W MAC descriptor
// no support for multiple instances
/*static*/ const TCPIP_MAC_OBJECT DRV_MRF24W_MACObject =
{
    TCPIP_MODULE_MAC_MRF24W,
    "MRF24W",
    MRF24W_MACInitialize,
    MRF24W_MACDeinitialize,
    MRF24W_MACReinitialize,
    MRF24W_MACStatus,
    MRF24W_MACTasks,
    MRF24W_MACOpen,
    MRF24W_MACClose,
    MRF24W_MACCheckLink,
    MRF24W_MACMulticastFilterSet,
    MRF24W_MACPowerMode,
    MRF24W_MACTxPacket,
    MRF24W_MACRxPacket,
    MRF24W_MACProcess,
    MRF24W_MACStatisticsGet,
    MRF24W_MACParametersGet,
    MRF24W_MACRegisterStatisticsGet,
    MRF24W_MACGetConfig,
    MRF24W_MACEventSetMask,
    MRF24W_MACEventAcknowledge,
    MRF24W_MACEventGetPending,
};

// only one hardware instance for now!
static MRF24W_MAC_DCPT s_pic32_mrf24w_mac_dcpt[1] =
{
    {
        &DRV_MRF24W_MACObject,
        // specific PIC32 MAC data
        0,                        // pNetIf
        0,                        // isInit
        0,                        // isOpen
        SYS_STATUS_UNINITIALIZED, // sysStat
    }
};

DRV_WIFI_PRIV g_drv_wifi_priv =
{
    /* explicity initialize g_drv_wifi_priv.initConn to true */
    .initConn = true
};

static bool s_runTxStateMachine = false;
static bool s_runRxStateMachine = false;
static TCPIP_MAC_MODULE_CTRL s_content_stackData;
static TCPIP_MAC_ADDR s_MulticastFilter[MAX_MULTICAST_FILTER_SIZE];
static uint16_t s_multicastFilterIndex = 0;

/*
 * interface functions
 */
static SYS_MODULE_OBJ MRF24W_MACInitialize(const SYS_MODULE_INDEX index, const SYS_MODULE_INIT *const init)
{
    const TCPIP_MAC_MODULE_CTRL *const stackData = ((TCPIP_MAC_INIT *)init)->macControl;
    MRF24W_MAC_DCPT *pDcpt = s_pic32_mrf24w_mac_dcpt + 0; // no other instance supported

    if (pDcpt->isInit != 0)
        return (SYS_MODULE_OBJ)pDcpt; // already initialized, have a client connected

    if (stackData->moduleId != TCPIP_MODULE_MAC_MRF24W)
        return SYS_MODULE_OBJ_INVALID; // no other type supported

    if (MRF24W_MACEventInit(pDcpt, stackData->eventF, stackData->eventParam, WF_EVENT_IPL, WF_EVENT_SIPL) != TCPIP_MAC_RES_OK)
        return SYS_MODULE_OBJ_INVALID;

    _MRF24W_MACInitialize(stackData);

    DRV_WIFI_Initialize(); // reset DRV_WIFI_SystemInit() state

#if defined(DRV_WIFI_USE_FREERTOS)
    if (!DRV_WIFI_TaskSyncInit()) // semaphores and mutex init
        return SYS_MODULE_OBJ_INVALID;

    if (!DRV_WIFI_AllTasksCreate((void *)pDcpt, NULL, NULL))
        return SYS_MODULE_OBJ_INVALID; // create all Wi-Fi tasks
#endif

    s_content_stackData = *stackData; // save pointer to stack data
    pDcpt->isInit = true;
    pDcpt->sysStat = SYS_STATUS_BUSY; // more init works are waiting, so set it to busy
    return (SYS_MODULE_OBJ)pDcpt;
}

static void MRF24W_MACTasks(SYS_MODULE_OBJ object)
{
    TCPIP_MAC_RES res;
    MRF24W_MAC_DCPT *pDcpt = (MRF24W_MAC_DCPT *)object;

    switch (pDcpt->sysStat) {
    case SYS_STATUS_UNINITIALIZED:
        break;
    case SYS_STATUS_BUSY:
#if defined(DRV_WIFI_USE_FREERTOS)
        if (!isInitComplete())
            break;
#else
        // perform the Wi-Fi initialization
        res = DRV_WIFI_SystemInit();
        if (res == TCPIP_MAC_RES_PENDING) {
            break; // not ready yet
        } else if (res == TCPIP_MAC_RES_INIT_FAIL) {
            pDcpt->sysStat = SYS_STATUS_ERROR;
            SYS_CONSOLE_MESSAGE("Wi-Fi Driver reports SYS_STATUS_ERROR!\r\n");
            break;
        }
#endif

        res = _MRF24W_MACOpen();
        if (res == TCPIP_MAC_RES_PENDING) {
            break; // not ready yet
        } else if (res == TCPIP_MAC_RES_OK) {
            g_drv_wifi_priv.isDriverOpen = true;
#if defined(DRV_WIFI_USE_FREERTOS)
            DRV_WIFI_TaskDestroy(&(g_drv_wifi_priv.initTaskHandle));
#endif
            pDcpt->sysStat = SYS_STATUS_READY;
        } else { // some error occurred
            MRF24W_MACClose((TCPIP_MAC_HANDLE)pDcpt);
            _MRF24W_MACDeinitialize(pDcpt);
            pDcpt->sysStat = SYS_STATUS_ERROR;
            SYS_CONSOLE_MESSAGE("Wi-Fi Driver reports SYS_STATUS_ERROR!\r\n");
        }
        break;
    case SYS_STATUS_READY:
    case SYS_STATUS_READY_EXTENDED:
        if (DRV_WIFI_PendingEventGet())
            DRV_WIFI_PendingEventHandle();
        break;
    default: // SYS_STATUS_ERROR, SYS_STATUS_ERROR_EXTENDED
        break;
    }
}

static void MRF24W_MACDeinitialize(SYS_MODULE_OBJ object)
{
    MRF24W_MAC_DCPT *pDcpt = (MRF24W_MAC_DCPT *)object;

    _MRF24W_MACDeinitialize(pDcpt);
}

static void MRF24W_MACReinitialize(SYS_MODULE_OBJ object, const SYS_MODULE_INIT *const init)
{
    /* unsupported */
}

static SYS_STATUS MRF24W_MACStatus(SYS_MODULE_OBJ object)
{
    MRF24W_MAC_DCPT *pDcpt = (MRF24W_MAC_DCPT *)object;

    return pDcpt->sysStat;
}

static size_t MRF24W_MACGetConfig(DRV_HANDLE hMac, void *configBuff, size_t buffSize, size_t *pConfigSize)
{
    /* unsupported */
    return 0;
}

static DRV_HANDLE MRF24W_MACOpen(const SYS_MODULE_INDEX drvIndex, const DRV_IO_INTENT intent)
{
    MRF24W_MAC_DCPT *pDcpt = s_pic32_mrf24w_mac_dcpt + 0; // no other instance supported
    DRV_HANDLE hMac = DRV_HANDLE_INVALID;

    if (!(pDcpt->isOpen))
    { // only one client for now
        pDcpt->isOpen = true;
        hMac = (DRV_HANDLE)pDcpt;
        MulticastFilter_Initialize();
    }

    return hMac;
}

static void MRF24W_MACClose(TCPIP_MAC_HANDLE hMac)
{
    if (isLinkUp()) {
        if (DRV_WIFI_Disconnect() == DRV_WIFI_SUCCESS) {
            g_drv_wifi_priv.isInDriverClose = true;
            while (isLinkUp()) {
                _MRF24W_MACProcess();
                if (DRV_WIFI_PendingEventGet()) {
                    DRV_WIFI_PendingEventHandle();
                }
            }
            g_drv_wifi_priv.isInDriverClose = false;
        }
    }
    MRF24W_MAC_DCPT *pDcpt = (MRF24W_MAC_DCPT *)hMac;
    pDcpt->isOpen = false;
    g_drv_wifi_priv.isDriverOpen = false;
}

static bool MRF24W_MACCheckLink(TCPIP_MAC_HANDLE hMac)
{
    return _MRF24W_MACCheckLink();
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

    s_multicastFilterIndex = 0;
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

TCPIP_MAC_RES MRF24W_MACMulticastFilterSet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_ADDR *DestMACAddr)
{
    uint8_t all_zeros[6] = {0, 0, 0, 0, 0, 0};
    DRV_WIFI_MULTICAST_CONFIG p_config;
    int i;
    TCPIP_MAC_RES res;

    WF_FILTERSET_MUTEX_LOCK();

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
            DRV_WIFI_ASSERT(false, "Multicast filter is full\r\n");
            res = TCPIP_MAC_RES_OP_ERR;
            break;
        }

        MulticastAddr_Set(DestMACAddr, i);
        DRV_WIFI_MultiCastFilterEnable();
        p_config.filterId = s_multicastFilterIndex + 4;
        p_config.action = DRV_WIFI_MULTICAST_USE_FILTERS;
        memcpy((void *)p_config.macBytes, (uint8_t *)DestMACAddr, WF_MAC_ADDRESS_LENGTH);
        p_config.macBitMask = 0x3F;
        DRV_WIFI_MulticastFilterSet(&p_config);
        s_multicastFilterIndex++;

        res = TCPIP_MAC_RES_OK;
    } while(0);

    WF_FILTERSET_MUTEX_UNLOCK();

    return res;
}

static bool MRF24W_MACPowerMode(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_POWER_MODE pwrMode)
{
    /* Unsupported */
    return true;
}

static TCPIP_MAC_RES MRF24W_MACTxPacket(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_PACKET *ptrPacket)
{
    TCPIP_MAC_RES res;

    if (DRV_WIFI_InHibernateMode())
    {
        DRV_WIFI_UserEventSet(DRV_WIFI_EVENT_ERROR, DRV_WIFI_ERROR_IN_HIBERNATE_MODE, true);
        return TCPIP_MAC_RES_QUEUE_TX_FULL;
    }

    res = MRF24W_TxPacketSend(ptrPacket);
    return res;
}

static TCPIP_MAC_PACKET *MRF24W_MACRxPacket(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RES *pRes, const TCPIP_MAC_PACKET_RX_STAT **ppPktStat)
{
    // get the oldest Rx packet
    return MRF24W_RxPacketGet();
}

static void DataRxTx_Handle(void)
{
    if (s_runRxStateMachine) {
        s_runRxStateMachine = false;
        DRV_WIFI_DataRxTask();
    }

    if (s_runTxStateMachine) {
        s_runTxStateMachine = false;
        DRV_WIFI_DataTxTask();
    }
}

static TCPIP_MAC_RES _MRF24W_MACProcess(void)
{
    MRF24W_USR_EV_DCPT eventInfo;

    if (MgmtEventsGet(&eventInfo))
        DRV_WIFI_ProcessEvent(eventInfo.mgmtEvents, eventInfo.mgmtEventInfo);

    DataRxTx_Handle();

    return TCPIP_MAC_RES_OK;
}

static TCPIP_MAC_RES MRF24W_MACProcess(TCPIP_MAC_HANDLE hMac)
{
    TCPIP_MAC_RES res;

#if defined(DRV_WIFI_USE_FREERTOS)
    DRV_WIFI_SemGive(&g_drv_wifi_priv.macTaskSync);
    res  = TCPIP_MAC_RES_OK;
#else
    res = _MRF24W_MACProcess();
#endif /* defined(DRV_WIFI_USE_FREERTOS) */

    return res;
}

#if defined(DRV_WIFI_USE_FREERTOS)
void DRV_WIFI_MACTask(void *p_arg)
{
    while (1) {
        DRV_WIFI_SemTake(&g_drv_wifi_priv.macTaskSync, OSAL_WAIT_FOREVER);
        _MRF24W_MACProcess();
    }
}
#endif /* defined(DRV_WIFI_USE_FREERTOS) */

static TCPIP_MAC_RES MRF24W_MACStatisticsGet(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_RX_STATISTICS *pRxStatistics, TCPIP_MAC_TX_STATISTICS* pTxStatistics)
{
    /* unsupported */
    return TCPIP_MAC_RES_OP_ERR;
}

static TCPIP_MAC_RES MRF24W_MACParametersGet(DRV_HANDLE hMac, TCPIP_MAC_PARAMETERS *pMacParams)
{
    MRF24W_MAC_DCPT* pDcpt = (MRF24W_MAC_DCPT*)hMac;

    if (pDcpt->sysStat == SYS_STATUS_READY)
    {
        if (pMacParams)
        {
            memcpy(pMacParams->ifPhyAddress.v, s_content_stackData.ifPhyAddress.v, sizeof(pMacParams->ifPhyAddress));
            pMacParams->processFlags = (TCPIP_MAC_PROCESS_FLAG_RX | TCPIP_MAC_PROCESS_FLAG_TX);
            pMacParams->macType = TCPIP_MAC_TYPE_WLAN;
        }

        return TCPIP_MAC_RES_OK;
    }

    return TCPIP_MAC_RES_IS_BUSY;
}

static TCPIP_MAC_RES MRF24W_MACRegisterStatisticsGet(DRV_HANDLE hMac, TCPIP_MAC_STATISTICS_REG_ENTRY* pRegEntries, int nEntries, int* pHwEntries)
{
    // not supported for now
    return TCPIP_MAC_RES_OP_ERR;
}

static bool MRF24W_MACEventSetMask(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents, bool enable)
{
    return _MRF24W_MACEventSetMask(hMac, macEvents, enable);
}

static bool MRF24W_MACEventAcknowledge(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents)
{
    return _MRF24W_MACEventAcknowledge(hMac, macEvents);
}

static TCPIP_MAC_EVENT MRF24W_MACEventGetPending(TCPIP_MAC_HANDLE hMac)
{
    return _MRF24W_MACEventGetPending(hMac);
}

static TCPIP_MAC_RES _MRF24W_MACOpen(void)
{
    TCPIP_MAC_RES res;

    res = DRV_WIFI_ContextLoad();

    if (res == TCPIP_MAC_RES_OK) {
        if (g_drv_wifi_priv.initConn) {
            OutputDriverConfig();
            SYS_CONSOLE_MESSAGE("\r\nStart Wi-Fi Connect . . .\r\n");
            DRV_WIFI_PsPollDisable();
            DRV_WIFI_Connect();
        }
    } else if ((res != TCPIP_MAC_RES_OK) && (res != TCPIP_MAC_RES_PENDING)) {
        /* TODO : handle the error case, currently use assert */
        DRV_WIFI_ASSERT(false, "");
    }

    return res;
}

static void _MRF24W_MACDeinitialize(MRF24W_MAC_DCPT *pDcpt)
{
    if (pDcpt->isInit) {
        do {
            // de-init DRV_WIFI_DataRxTask(), finish what we are processing
            // DRV_WIFI_DataTxTask() does not need to be called in non-RTOS environment
            DataRxTx_Handle();
        } while (s_runRxStateMachine || s_runTxStateMachine);
        DRV_WIFI_Deinitialize();
        ResetModule(); // will stop the MR24W from transmitting
        DRV_WIFI_SpiClose();
        DRV_WIFI_AllEventClear();
        MRF24W_MACEventDeInit(pDcpt);
        DRV_WIFI_HibernateEnable();
        pDcpt->isInit = false;
    }
}

const TCPIP_MAC_MODULE_CTRL *GetStackData(void)
{
    return &s_content_stackData;
}

bool isMacInitialized(void)
{
    MRF24W_MAC_DCPT *pDcpt;

    pDcpt = s_pic32_mrf24w_mac_dcpt + 0; // no other instance supported
    return pDcpt->isInit;
}

void SignalTxStateMachine(void)
{
    s_runTxStateMachine = true;
    DRV_WIFI_UserEventSet(TCPIP_MAC_EV_TX_DONE, 0, false); // will result in MRF24W_MACProcess being called
}

void SignalRxStateMachine(void)
{
    s_runRxStateMachine = true;
    DRV_WIFI_UserEventSet(TCPIP_MAC_EV_RX_DONE, 0, false); // will result in MRF24W_MACProcess being called
}

//DOM-IGNORE-END
