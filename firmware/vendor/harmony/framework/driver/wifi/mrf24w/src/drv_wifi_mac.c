/*******************************************************************************
  MRF24WG Driver Medium Access Control (MAC) Layer

  File Name:
    drv_wifi_mac.c

  Summary:
    MRF24WG Driver Medium Access Control (MAC) Layer

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
#include "drv_wifi_eint.h"
#include "drv_wifi_raw.h"

/*****************/
/*    DEFINES    */
/*****************/
//#define DBG_TX_PRINT(s) SYS_CONSOLE_MESSAGE(s)
#define DBG_TX_PRINT(s)

//#define DBG_RX_PRINT(s) SYS_CONSOLE_MESSAGE(s)
#define DBG_RX_PRINT(s)

#define TCPIP_THIS_MODULE_ID TCPIP_MODULE_MAC_MRF24W

#define MAX_RX_PACKET_SIZE (1518)

#define NUM_PREALLOCATED_RX_PACKETS (4)

#define SNAP_HDR_LENGTH (6)
#define SNAP_VAL        (0xaa)
#define SNAP_CTRL_VAL   (0x03)
#define SNAP_TYPE_VAL   (0x00)

#define SNAP_HEADER_OFFSET      (10) // offset in RX_PACKET_HEADER where SNAP header starts
#define ETH_HEADER_START_OFFSET (16) // offset in Rx packet where Ethernet header starts
#define WF_RX_PREAMBLE_SIZE     (sizeof(WF_RX_PREAMBLE))
#define WF_TX_PREAMBLE_SIZE     (sizeof(WF_TX_PREAMBLE))
#define ETH_HEADER_SIZE         (14)

#define DRV_WIFI_WPS_CREDENTIALS_SAVE_FLAG 0x5a5a
/* Make sure to use power of 2 for the size. Otherwise pseudo mod operation will not work.
   To adjust RSSI index, we use bit-and operation instead of expensive % operation. */
#define MAX_RSSI_CACHE_SIZE 16

/**************************/
/*    LOCAL DATA TYPES    */
/**************************/
//----------------
// RAW Init States
//----------------
typedef enum
{
    R_INIT_BEGIN = 0,
    R_INIT_WAIT_FOR_SCRATCH_UNMOUNT = 1,
    R_INIT_WAIT_FOR_SCRATCH_MOUNT = 2
} RAW_INIT_STATE;

//---------------
// Rx Data States
//---------------
typedef enum
{
    DATA_RX_IDLE = 0,
    WAIT_FOR_TX_IDLE = 1,
    WAIT_FOR_DATA_RX_MOUNT = 2,
    WAIT_FOR_DATA_RX_UNMOUNT = 3
} RX_STATE;

//---------------
// Tx Data States
//---------------
typedef enum
{
    DATA_TX_IDLE = 0,
    WAIT_FOR_TX_MEMORY = 1,
    WAIT_FOR_DATA_TX_MOUNT = 2,
    WAIT_FOR_DATA_TX_SIGNAL = 3,
    WAIT_FOR_RX_IDLE = 4
} TX_STATE;

// this structure is present in all Rx packets received from the MRF24WG, at the
// start of the Rx RAW buffer, starting at index 0.  Payload data immediately follows
// this structure
typedef struct
{
    uint8_t type; // always WF_DATA_RX_INDICATE_TYPE (3)
    uint8_t subtype;
    uint16_t rssi; // not used
    uint32_t arrivalTime; // not used
    uint16_t dataLength; // number of bytes of payload which immediately follow this structure
    uint8_t snapHeader[6]; // SNAP header
    uint8_t destAddress[6]; // destination MAC address (start of Ethernet header)
    uint8_t srcMacAddress[6]; // source MAC address
    uint16_t ethType; // Ethernet type code
} RX_PACKET_HEADER;

typedef struct
{
    uint8_t type;
    uint8_t subType;
} RX_PACKET_HEADER_PREAMBLE;

typedef struct
{
    uint8_t reserved[4];
} WF_TX_PREAMBLE;

typedef struct
{
    uint8_t SNAP[SNAP_HDR_LENGTH];
    // Ethernet header presented to stack starts here.
    // Following names are exactly the same as TCPIP_MAC_ETHERNET_HEADER in tcpip_mac.h.
    TCPIP_MAC_ADDR DestMACAddr;
    TCPIP_MAC_ADDR SourceMACAddr;
    TCPIP_UINT16_VAL Type;
} WF_RX_PREAMBLE;

// used to keep track of Rx packets queued for stack
typedef struct
{
    int front;
    int rear;
    TCPIP_MAC_PACKET *items[NUM_PREALLOCATED_RX_PACKETS + 1];
} WF_RX_FIFO;

/*******************/
/*    VARIABLES    */
/*******************/
bool g_RxDataLock = true;

static uint8_t s_RawInitState;                    // state for Raw Init state machine
static uint8_t s_TxDataState;                     // state for Data Tx state machine
static uint8_t s_RxDataState;                     // state for Data Rx state machine
static SYS_TMR_HANDLE s_DataTxRawMoveTimer = 0;   // timer callback for Data Tx raw move complete
static SYS_TMR_HANDLE s_DataRxRawMoveTimer = 0;   // timer callback for Data Rx raw move complete
static SYS_TMR_HANDLE s_ScratchRawMoveTimer = 0;  // timer callback for Scratch raw move complete
static SYS_TMR_HANDLE s_RxTxUnlockSleepTimer = 0; // timer, when resume, delay 0.2 second before handle RX packet
static SINGLE_LIST s_DataTxQueue = {0};           // queue of data Tx packets waiting to be transmitted from host
static SINGLE_LIST s_DataRxQueue = {0};           // queue of data Rx packets waiting to be processed by stack from host
static WF_RX_FIFO s_RxFifo;                       // FIFO to hold pointers to Rx packets
static bool s_TxWaitingForRxIdle;
static bool s_RxPacketPending;

static TCPIP_MAC_PACKET *RxBuffer_packet[NUM_PREALLOCATED_RX_PACKETS] = {NULL};
static uint8_t dump_buffer[512] = {0};

// packet allocation functions as passed by the stack
static _TCPIP_PKT_ALLOC_PTR s_pktAllocF = 0;
static _TCPIP_PKT_FREE_PTR s_pktFreeF = 0;
static _TCPIP_PKT_ACK_PTR s_pktAckF = 0;

static uint16_t s_rssiCache[MAX_RSSI_CACHE_SIZE];
static uint32_t s_rssiIndex = 0;
static uint16_t s_adjustedIndex = 0;
static uint32_t s_rssiSum = 0;
static uint16_t s_rssiShiftDivisor = 0;
static uint16_t s_rssiIndexWrapped = 0;
static bool s_readRssiEnabled = false;

/***********************************/
/*    LOCAL FUNCTION PROTOTYPES    */
/***********************************/
static void RawInit(void);
static void RawMoveDataRxTimeoutHandler(uintptr_t context, uint32_t currTick);
static void RawMoveDataTxTimeoutHandler(uintptr_t context, uint32_t currTick);
static void RawMoveScratchTimeoutHandler(uintptr_t context, uint32_t currTick);
static bool RxDataCallback(TCPIP_MAC_PACKET *pktHandle, const void *ackParam);
static bool isRxPacketHeaderValid(void);
static bool isTxStateMachineIdle(void);
static TCPIP_MAC_PACKET *GetAvailRxBuf(void);
static void RxFifoInit(WF_RX_FIFO *p_fifo);
static bool isRxFifoEmpty(WF_RX_FIFO *p_fifo);
static void RxFifoInsert(WF_RX_FIFO *p_fifo, TCPIP_MAC_PACKET *p_packet);
static TCPIP_MAC_PACKET *RxFifoRemove(WF_RX_FIFO *p_fifo);

#if (DRV_WIFI_SAVE_WPS_CREDENTIALS == DRV_WIFI_ENABLED)
enum
{
    WEP_SHORT_KEY_SIZE = 5,
    WEP_LONG_KEY_SIZE = 13
};

enum
{
    SECURITY_NONE,
    SECURITY_OPEN,
    SECURITY_SHARED_KEY40,
    SECURITY_SHARED_KEY104,
    SECURITY_OPEN_KEY40,
    SECURITY_OPEN_KEY104,
    SECURITY_WPA1_PSK_KEY,
    SECURITY_WPA1_PSK_PASS,
    SECURITY_WPA2_PSK_KEY,
    SECURITY_WPA2_PSK_PASS,
    SECURITY_WPAUTO_PSK_KEY,
    SECURITY_WPAUTO_PSK_PASS,
    SECURITY_WPA_ENTERPRISE,
    SECURITY_WPS_PIN,
    SECURITY_WPS_PSB,
};

enum
{
    WEP_KEYIDX_MAX = 4,
    MSK_MAX = 64,
    PIN_MAX = 8,
};

typedef struct
{
    uint8_t key_idx;
    uint8_t key[WEP_KEYIDX_MAX][5];
}sec_wep40;

typedef struct
{
    uint8_t key_idx;
    uint8_t key[WEP_KEYIDX_MAX][13];
}sec_wep104;

typedef struct
{
    uint8_t key_len;
    uint8_t key[MSK_MAX];
}sec_wpa_psk;

typedef struct
{
    uint8_t pin[PIN_MAX];
}sec_wps;

typedef union
{
    sec_wep40 wep40;
    sec_wep104 wep104;
    sec_wpa_psk wpa_psk;
    sec_wps wps;
}sec_key;

static uint8_t ConvAscii2Hex(uint8_t a)
{
    if (a >= '0' && a <= '9')
        return (uint8_t)(a - 48);
    if (a >= 'a' && a <= 'f')
        return (uint8_t)(a - 97 + 10);
    if (a >= 'A' && a <= 'F')
        return (uint8_t)(a - 65 + 10);

    return '?';
}

static void ConvAsciiKey2Hex(uint8_t *key, uint8_t keyLen, uint8_t *hexKey)
{
    uint8_t i;

    for (i = 0; i < keyLen; i += 2)
    {
        hexKey[i / 2] = ConvAscii2Hex(key[i]) << 4;
        hexKey[i / 2] |= ConvAscii2Hex(key[i + 1]);
    }
}

static void ConfigWep(DRV_WIFI_WPS_CREDENTIAL *cred, uint8_t *secType, sec_key *key)
{
    uint8_t i;
    uint8_t wep_key[WEP_LONG_KEY_SIZE];
    sec_wep40 *wep_ctx = (sec_wep40 *)key;
    uint8_t *keys = (uint8_t *)wep_ctx + 1;
    uint8_t key_len = 0;

    if (cred->keyLen == WEP_SHORT_KEY_SIZE * 2)
    {
        *secType = DRV_WIFI_SECURITY_WEP_40;
        ConvAsciiKey2Hex(cred->netKey, cred->keyLen, wep_key);
        key_len = cred->keyLen / 2;
    }
    else if (cred->keyLen == WEP_SHORT_KEY_SIZE)
    {
        *secType = DRV_WIFI_SECURITY_WEP_40;
        memcpy(wep_key, cred->netKey, cred->keyLen);
        key_len = cred->keyLen;
    }
    else if (cred->keyLen == WEP_LONG_KEY_SIZE * 2)
    {
        *secType = DRV_WIFI_SECURITY_WEP_104;
        ConvAsciiKey2Hex(cred->netKey, cred->keyLen, wep_key);
        key_len = cred->keyLen / 2;
    }
    else if (cred->keyLen == WEP_LONG_KEY_SIZE)
    {
        *secType = DRV_WIFI_SECURITY_WEP_104;
        memcpy(wep_key, cred->netKey, cred->keyLen);
        key_len = cred->keyLen;
    }
    else
    {
        //DRV_WIFI_ASSERT(false, "");
    }

    for (i = 0; i < 4; i++)
    {
        memcpy(keys + i * key_len, wep_key, key_len);
    }

    wep_ctx->key_idx = cred->keyIdx - 1;
}

void WPSCredentialsSave(void)
{
    DRV_WIFI_WPS_CREDENTIAL cred;
    sec_key key;
    uint8_t *psk;
    static bool once = false;

    if (DRV_WIFI_CONFIG_PARAMS(wpsCredSaveFlag) == DRV_WIFI_WPS_CREDENTIALS_SAVE_FLAG) {
        once = true;
    }
    if (!once)
    {
        DRV_WIFI_WPSCredentialsGet(&cred);
        memcpy((void *)(DRV_WIFI_CONFIG_PARAMS(ssid)), (void *)cred.ssid, cred.ssidLen);
        if (cred.ssidLen < DRV_WIFI_MAX_SSID_LENGTH) {
            DRV_WIFI_CONFIG_PARAMS(ssid)[cred.ssidLen]=0;
        }
        DRV_WIFI_CONFIG_PARAMS(ssidLen) = cred.ssidLen;

        switch (cred.authType) {
        case DRV_WIFI_WPS_AUTH_OPEN:
            if (cred.encType == DRV_WIFI_WPS_ENC_NONE)
            {
                DRV_WIFI_CONFIG_PARAMS(securityMode) = DRV_WIFI_SECURITY_OPEN;
            }
            else if (cred.encType == DRV_WIFI_WPS_ENC_WEP)
            {
                ConfigWep(&cred, &(DRV_WIFI_CONFIG_PARAMS(securityMode)), &key);
                if (DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_WEP_40)
                {
                    memcpy((void *)DRV_WIFI_CONFIG_PARAMS(securityKey), (void *)key.wep40.key, WEP_SHORT_KEY_SIZE * 4);
                    DRV_WIFI_CONFIG_PARAMS(securityKeyLen) = WEP_SHORT_KEY_SIZE * 4;
                }
                else if (DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_WEP_104)
                {
                    memcpy((void *)DRV_WIFI_CONFIG_PARAMS(securityKey), (void *)key.wep104.key, WEP_LONG_KEY_SIZE * 4);
                    DRV_WIFI_CONFIG_PARAMS(securityKeyLen) = WEP_LONG_KEY_SIZE * 4;
                }
                else
                {
                    //DRV_WIFI_ASSERT(false, "");
                }
            }
            break;

        case DRV_WIFI_WPS_AUTH_SHARED:
            ConfigWep(&cred, &DRV_WIFI_CONFIG_PARAMS(securityMode), &key);
            if (DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_WEP_40)
            {
                memcpy((void *)DRV_WIFI_CONFIG_PARAMS(securityKey), (void *)key.wep40.key, WEP_SHORT_KEY_SIZE * 4);
                DRV_WIFI_CONFIG_PARAMS(securityKeyLen) = WEP_SHORT_KEY_SIZE * 4;
            }
            else if (DRV_WIFI_CONFIG_PARAMS(securityMode) == DRV_WIFI_SECURITY_WEP_104)
            {
                memcpy((void *)DRV_WIFI_CONFIG_PARAMS(securityKey), (void *)key.wep104.key, WEP_LONG_KEY_SIZE * 4);
                DRV_WIFI_CONFIG_PARAMS(securityKeyLen) = WEP_LONG_KEY_SIZE * 4;
            }
            else
            {
                //DRV_WIFI_ASSERT(false, "");
            }
            break;

        case DRV_WIFI_WPS_AUTH_WPA_PSK:
        case DRV_WIFI_WPS_AUTH_WPA2_PSK:
            psk = (uint8_t *)DRV_WIFI_CONFIG_PARAMS(securityKey);
            memset((void *)psk, 0x00, 64);
            if (cred.keyLen == 64)
            {
                DRV_WIFI_CONFIG_PARAMS(securityMode) = DRV_WIFI_SECURITY_WPA_AUTO_WITH_KEY;
                DRV_WIFI_CONFIG_PARAMS(securityKeyLen) = 32;
                ConvAsciiKey2Hex(cred.netKey, cred.keyLen, psk);
            }
            else if (cred.keyLen >= 8 && cred.keyLen < 64)
            {
                DRV_WIFI_CONFIG_PARAMS(securityMode) = DRV_WIFI_SECURITY_WPA_AUTO_WITH_PASS_PHRASE;
                DRV_WIFI_CONFIG_PARAMS(securityKeyLen) = cred.keyLen;
                if (DRV_WIFI_CONFIG_PARAMS(securityKeyLen) > 8 && cred.netKey[DRV_WIFI_CONFIG_PARAMS(securityKeyLen) - 1] == '\0')
                {
                    --DRV_WIFI_CONFIG_PARAMS(securityKeyLen);
                }
                memcpy(psk, cred.netKey, DRV_WIFI_CONFIG_PARAMS(securityKeyLen));
            }
            break;

        default:
            //DRV_WIFI_ASSERT(false, "");
            break;
        } // end switch

        DRV_WIFI_CONFIG_PARAMS(wpsCredSaveFlag) = DRV_WIFI_WPS_CREDENTIALS_SAVE_FLAG;
        DRV_WIFI_ConfigDataSave();
        once = true;
    }
}
#endif /* DRV_WIFI_SAVE_WPS_CREDENTIALS == DRV_WIFI_ENABLED */

/*******************************************************************************
  Function:
        void DRV_WIFI_RSSI_Cache_FromRxDataRead(uint16_t rssi)

  Summary:
    Caches RSSI value from Rx data packet.

  Description:
    This function caches RSSI value from Rx data packet.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    rssi - RSSI value read from Rx data packet

  Return:
    None.

  Remarks:
    None.
 *******************************************************************************/
void DRV_WIFI_RSSI_Cache_FromRxDataRead(uint16_t rssi)
{
    static int32_t initialized = 0;

    if (initialized == 0) {
        int32_t i;
        uint16_t cacheSize;

        for (i = 0; i < MAX_RSSI_CACHE_SIZE; i++)
            s_rssiCache[i] = 0;
        cacheSize = MAX_RSSI_CACHE_SIZE;
        while (cacheSize > 1) {
            ++s_rssiShiftDivisor;
            cacheSize /= 2;
        }
        initialized = 1;
    }

    s_adjustedIndex = s_rssiIndex & (MAX_RSSI_CACHE_SIZE - 1);
    s_rssiSum -= s_rssiCache[s_adjustedIndex]; // delete old data
    s_rssiSum += rssi; // add new data
    s_rssiCache[s_adjustedIndex] = rssi;
    if (s_rssiIndex == 0xffffffff)
        s_rssiIndexWrapped = 1;

    ++s_rssiIndex;
}

/*******************************************************************************
  Function:
        void DRV_WIFI_RSSI_Get_FromRxDataRead(uint16_t *mean, uint16_t *last)

  Summary:
    Reads RSSI value from Rx data packet.

  Description:
    This function reads RSSI value from Rx data packet.

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    mean -  pointer to where the average mean RSSI value to be stored
    last -  pointer to where the total count of RSSI values to be stored

  Return:
    mean -  the calculated mean RSSI
    last -  the total count of RSSI values

  Remarks:
    None.
 *******************************************************************************/
void DRV_WIFI_RSSI_Get_FromRxDataRead(uint16_t *mean, uint16_t *last)
{
    uint16_t divisor;

    if (s_rssiIndexWrapped == 0)
        divisor = s_rssiIndex < MAX_RSSI_CACHE_SIZE ? s_rssiIndex : MAX_RSSI_CACHE_SIZE;
    else
        divisor = MAX_RSSI_CACHE_SIZE;

    if (divisor == MAX_RSSI_CACHE_SIZE) {
        *mean = s_rssiSum >> s_rssiShiftDivisor; // to avoid expensive division operation
    } else {
        if (divisor != 0)
            *mean = s_rssiSum / divisor;
        else
            *mean = 0;
    }

    if (s_adjustedIndex == 0)
        *last = s_rssiCache[MAX_RSSI_CACHE_SIZE - 1];
    else
        *last = s_rssiCache[s_adjustedIndex - 1];
}

/*******************************************************************************
 * Function:        void _MRF24W_MACInitialize(
 *                      const TCPIP_MAC_MODULE_CTRL *const stackData)
 *
 * PreCondition:    None.
 *
 * Input:           stackData - MAC Initialization Data provided by TCP/IP stack
 *
 * Output:          None.
 *
 * Side Effects:    None.
 *******************************************************************************/
void _MRF24W_MACInitialize(const TCPIP_MAC_MODULE_CTRL *const stackData)
{
    if (stackData) { // first time we're called
        s_pktAllocF = (_TCPIP_PKT_ALLOC_PTR)stackData->pktAllocF;
        s_pktFreeF = (_TCPIP_PKT_FREE_PTR)stackData->pktFreeF;
        s_pktAckF = (_TCPIP_PKT_ACK_PTR)stackData->pktAckF;
#if defined(WF_READ_RSSI_FROM_DATA)
        s_readRssiEnabled = true;
#endif
    }
}

void RawResetInitStateMachine(void)
{
    s_RawInitState = R_INIT_BEGIN;
}

int RawInitStateMachine(void)
{
    int retCode = RAW_INIT_BUSY;
    int status;
    uint16_t byteCount;

    switch (s_RawInitState) {
    //-------------------------------------
    case R_INIT_BEGIN:
    //-------------------------------------
        // By default the firmware mounts Scratch to RAW 1 after reset.  If desired,
        // we can read the SysInfo data block from the Scratch.  We are not using this
        // data, so unmount the scratch from this RAW window.
        ScratchUnmount(RAW_ID_1);
        s_RawInitState = R_INIT_WAIT_FOR_SCRATCH_UNMOUNT;
        break;

    //-------------------------------------
    case R_INIT_WAIT_FOR_SCRATCH_UNMOUNT:
    //-------------------------------------
        // if raw move completed
        if (isRawMoveComplete(RAW_ID_1, &status, &byteCount))
        {
            /* Mount scratch memory, index defaults to 0.  This will stay permanently mounted.    */
            /* If one needs to know, this function returns the number of bytes in scratch memory. */
            ScratchMount(RAW_SCRATCH_ID);
            s_RawInitState = R_INIT_WAIT_FOR_SCRATCH_MOUNT;
        }
        // else if raw move not completed and it timed out
        else if (status == RM_TIMEOUT)
        {
            retCode = RAW_INIT_SCRATCH_UNMOUNT_FAIL;
        }
        break;

    //-------------------------------------
    case R_INIT_WAIT_FOR_SCRATCH_MOUNT:
    //-------------------------------------
        if (isRawMoveComplete(RAW_SCRATCH_ID, &status, &byteCount))
        {
            RawInit(); // complete raw init
            retCode = RAW_INIT_COMPLETE;
        }
        // else if raw move not completed and it timed out
        else if (status == RM_TIMEOUT)
        {
            retCode = RAW_INIT_SCRATCH_MOUNT_FAIL;
        }
        break;
    }

    return retCode;
}

/******************************************************************************
 * Function:        void RawInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Initializes the RAW window states.
 *
 * Note:            None
 ******************************************************************************/
static void RawInit(void)
{
    ClearAllIndexOutofBoundsFlags(); /* no raw indexes have been set past end of raw window */
}

/******************************************************************************
 * Function:        bool _MRF24W_MACCheckLink(void)
 *
 * PreCondition:    None.
 *
 * Input:           None.
 *
 * Output:          true: If the PHY reports that a link partner is present
 *                        and the link has been up continuously since the last
 *                        call to _MRF24W_MACCheckLink().
 *                  false: If the PHY reports no link partner, or the link went
 *                         down momentarily since the last call to _MRF24W_MACCheckLink().
 *
 * Side Effects:    None.
 *
 * Overview:        Returns the PHSTAT1.LLSTAT bit.
 *
 * Note:            None.
 ******************************************************************************/
bool _MRF24W_MACCheckLink(void)
{
    return isLinkUp();
}

static void RxTxUnLockSleepTimeoutHandler(uintptr_t context, uint32_t currTick)
{
    g_RxDataLock = false;
}

void RxTxUnLockTimerStart(void)
{
    uint16_t timeout;

    timeout = SYS_TMR_TickCounterFrequencyGet() / 5;
    s_RxTxUnlockSleepTimer = SYS_TMR_CallbackSingle(timeout, 0, RxTxUnLockSleepTimeoutHandler);
}

// called by RawMove()
void StartRawMoveTimer(uint16_t rawId)
{
    uint16_t timeout;

    timeout = SYS_TMR_TickCounterFrequencyGet() * 4;

    if (rawId == RAW_DATA_RX_ID)
    {
        s_DataRxRawMoveTimer = SYS_TMR_CallbackSingle(timeout, 0, RawMoveDataRxTimeoutHandler);
    }
    else if (rawId == RAW_DATA_TX_ID)
    {
        s_DataTxRawMoveTimer = SYS_TMR_CallbackSingle(timeout, 0, RawMoveDataTxTimeoutHandler);
    }
    else // must be RAW_SCRATCH_ID (only at init)
    {
        s_ScratchRawMoveTimer = SYS_TMR_CallbackSingle(timeout, 0, RawMoveScratchTimeoutHandler);
    }
}

void StopRawMoveTimer(uint16_t rawId)
{
    if (rawId == RAW_DATA_RX_ID)
    {
       SYS_TMR_CallbackStop(s_DataRxRawMoveTimer);
    }
    else if (rawId == RAW_DATA_TX_ID)
    {
        SYS_TMR_CallbackStop(s_DataTxRawMoveTimer);
    }
    else // must be RAW_SCRATCH_ID (only at init)
    {
        SYS_TMR_CallbackStop(s_ScratchRawMoveTimer);
    }
}

// called from timer interrupt when timeout occurs
static void RawMoveDataRxTimeoutHandler(uintptr_t context, uint32_t currTick)
{
    DRV_WIFI_ASSERT(false, "Rx Timeout");
}

// called from timer interrupt when timeout occurs
static void RawMoveDataTxTimeoutHandler(uintptr_t context, uint32_t currTick)
{
    DRV_WIFI_ASSERT(false, "Tx Timeout");
}

static void RawMoveScratchTimeoutHandler(uintptr_t context, uint32_t currTick)
{
    DRV_WIFI_ASSERT(false, "Scratch Timeout");
}

void DRV_WIFI_TxTaskInit(void)
{
    s_TxDataState = DATA_TX_IDLE;
    s_TxWaitingForRxIdle = false;

    // create a queue to hold data Tx messages that need to be sent to MRF
    TCPIP_Helper_SingleListInitialize(&s_DataTxQueue);
}

TCPIP_MAC_RES DRV_WIFI_RxTaskInit(void)
{
    uint8_t i;

    s_RxDataState = DATA_RX_IDLE;
    s_RxPacketPending = false;

    // create a queue to hold pointers to preallocated Rx packets
    TCPIP_Helper_SingleListInitialize(&s_DataRxQueue);

    for (i = 0; i < NUM_PREALLOCATED_RX_PACKETS; ++i)
    {
        // preallocate Rx buffers to store Rx packets as they come in (1500 bytes data plus header and checksum)
        RxBuffer_packet[i] = s_pktAllocF ? _TCPIP_PKT_ALLOC_BY_PTR(s_pktAllocF, sizeof(TCPIP_MAC_PACKET), MAX_RX_PACKET_SIZE, 0) : 0;
        if (RxBuffer_packet[i] != NULL)
        {
            RxBuffer_packet[i]->next = NULL;
            RxBuffer_packet[i]->ackFunc = RxDataCallback;
            RxBuffer_packet[i]->ackParam = NULL;
            RxBuffer_packet[i]->pktFlags = 0;
            RxBuffer_packet[i]->pDSeg->segFlags |= TCPIP_MAC_SEG_FLAG_RX_STICKY; // reuse; don't deallocate
            TCPIP_Helper_SingleListTailAdd(&s_DataRxQueue, (SGL_LIST_NODE *)RxBuffer_packet[i]);
        }
        // else allocation failed
        else
        {
            SYS_ERROR_PRINT(SYS_ERROR_FATAL, "\r\n%s, line %u\r\n", __FILE__, __LINE__);
            return TCPIP_MAC_RES_INIT_FAIL;
        }
    }

    // create another FIFO to store pointers
    RxFifoInit(&s_RxFifo);

    return TCPIP_MAC_RES_OK;
}

void RxQueueDeinit(void)
{
    int i;
    if (s_pktFreeF != NULL)
    {
        for (i = 0; i < NUM_PREALLOCATED_RX_PACKETS; ++i)
        {
            if (RxBuffer_packet[i] != NULL)
            {
                _TCPIP_PKT_FREE_BY_PTR(s_pktFreeF, RxBuffer_packet[i]);
                RxBuffer_packet[i] = NULL;
            }
        }
    }
}

static void RxFifoInit(WF_RX_FIFO *p_fifo)
{
    memset(p_fifo, 0x00, sizeof(WF_RX_FIFO));
    p_fifo->front = p_fifo->rear = NUM_PREALLOCATED_RX_PACKETS - 1;
}

static bool isRxFifoEmpty(WF_RX_FIFO *p_fifo)
{
    return p_fifo->front == p_fifo->rear;
}

static void RxFifoInsert(WF_RX_FIFO *p_fifo, TCPIP_MAC_PACKET *p_packet)
{
    if (p_fifo->rear == NUM_PREALLOCATED_RX_PACKETS - 1)
    {
        p_fifo->rear = 0;
    }
    else
    {
        ++p_fifo->rear;
    }

    p_fifo->items[p_fifo->rear] = p_packet;
}

static TCPIP_MAC_PACKET *RxFifoRemove(WF_RX_FIFO *p_fifo)
{
    if (p_fifo->front == NUM_PREALLOCATED_RX_PACKETS - 1)
    {
        p_fifo->front = 0;
    }
    else
    {
        ++p_fifo->front;
    }

    return p_fifo->items[p_fifo->front];
}

// called by stack when done with Rx packet
static bool RxDataCallback(TCPIP_MAC_PACKET *pktHandle, const void *ackParam)
{
    ackParam = ackParam; // ignore warning

    if (pktHandle)
    {
        // if this is packet allocated at init and is going to be reused
        if ((pktHandle->pDSeg->segFlags & TCPIP_MAC_SEG_FLAG_RX_STICKY) == TCPIP_MAC_SEG_FLAG_RX_STICKY)
        {
            pktHandle->pktFlags &= ~TCPIP_MAC_PKT_FLAG_QUEUED;
            TCPIP_Helper_SingleListTailAdd(&s_DataRxQueue, (SGL_LIST_NODE *)pktHandle); // add packet back to free list
        }
        // else this is a packet that we dynamically allocated because the 'permanent' packets were all in use
        else
        {
            // deallocate the Rx packet
            _TCPIP_PKT_FREE_BY_PTR(s_pktFreeF, pktHandle);
            pktHandle = NULL;
        }
    }
    else
    {
        SYS_CONSOLE_MESSAGE("!!!\r\n");
    }
    return false;
}

// called when Raw move complete event occurs
void DRV_WIFI_DataRxTask(void)
{
    int status;
    uint16_t byteCount;
    uint16_t ethPacketLength; // includes header
    TCPIP_MAC_PACKET *p_packet;
    const TCPIP_MAC_MODULE_CTRL *p_ctrl = GetStackData();
    const uint8_t *p_Mac = TCPIP_STACK_NetAddressMac(TCPIP_STACK_IndexToNet(p_ctrl->netIx));

    DecrementRxPendingCount();

    if (!g_drv_wifi_priv.isInDriverClose) { // normal operation

        SYS_INT_SourceDisable(MRF_INT_SOURCE);

        DBG_RX_PRINT("RX: ");
        switch (s_RxDataState) {
        //======================================================================
        case DATA_RX_IDLE:
            DBG_RX_PRINT("0 ");
            if (isTxStateMachineIdle())
            {
                DBG_RX_PRINT("1 ");
                RawMountRxDataBuffer(); // start the raw Rx data mount
                s_RxDataState = WAIT_FOR_DATA_RX_MOUNT;
            }
            else
            {
                DBG_RX_PRINT("2 ");
                // schedule state machine to run again
                SignalRxStateMachine();
                IncrementRxPendingCount();
                s_RxDataState = WAIT_FOR_TX_IDLE;
            }
            break;

        //======================================================================
        case WAIT_FOR_TX_IDLE:
            // if Tx state machine is idle, or Tx state machine is waiting for Rx state machine
            if (isTxStateMachineIdle())
            {
                DBG_RX_PRINT("3 ");
                // then run this (Rx) state machine and let Tx state machine wait
                RawMountRxDataBuffer(); // start the raw Rx data mount
                s_RxDataState = WAIT_FOR_DATA_RX_MOUNT;
            }
            else
            {
                DBG_RX_PRINT("4 ");
                // schedule this state machine to run again (and recheck if Tx is idle)
                SignalRxStateMachine();
                IncrementRxPendingCount();
            }
            break;

        //======================================================================
        case WAIT_FOR_DATA_RX_MOUNT:
            // this should always be true as the event is only triggered, and this
            // function should only be called, after the Raw Move has completed.
            if (isRawMoveComplete(RAW_DATA_RX_ID, &status, &byteCount))
            {
                DBG_RX_PRINT("5 ");
                { // for debugging
                    if ( !((byteCount > 0) && (byteCount <= 1530)) )
                    {
                        //SYS_ERROR_PRINT(SYS_ERROR_WARNING, "\r\nRx Packet is 0 byte or\r\nRx Packet is larger than 1528 bytes for UDP or 1530 bytes for TCP\r\n");
                        if (byteCount == 0)
                        {
                            SYS_CONSOLE_PRINT("Rx Packet status = %d, byteCount = %d\r\n", status, byteCount);
                        }
                        else
                        {
                            SYS_CONSOLE_PRINT("Rx Packet is larger than 1528 bytes for UDP or 1530 bytes for TCP, status = %d, byteCount = %d\r\n", status, byteCount);
                        }
                        DRV_WIFI_ASSERT(false, "");
                    }
                }

                // if locked, receive packet then throw it away
                if (g_RxDataLock)
                {
                    if (byteCount > ETH_HEADER_START_OFFSET)
                    {
                        uint32_t count_read = 0, tmp = 0;

                        //SYS_CONSOLE_PRINT("byteCount = %d\r\n",byteCount);
                        ethPacketLength = byteCount - ETH_HEADER_START_OFFSET;
                        while (ethPacketLength > 0)
                        {
                            if (ethPacketLength > 512)
                                tmp = 512;
                            else
                                tmp = ethPacketLength;

                            RawRead(RAW_DATA_RX_ID, ETH_HEADER_START_OFFSET + count_read, tmp, dump_buffer);
                            ethPacketLength -= tmp;
                            count_read += tmp;
                        }
                    }
                }
                // if raw move successfully completed and the Rx packet header is valid
                else if ((status == RM_COMPLETE) && (isRxPacketHeaderValid()))
                {
                    DBG_RX_PRINT("6 ");
                    // get an Rx buffer structure from queue to copy Rx packet to (from MRF to Rx buffer struct)
                    p_packet = GetAvailRxBuf();

                    // if packet structure available
                    if (p_packet != NULL)
                    {
                        DBG_RX_PRINT("7 ");
                        // read Ethernet packet into host buffer
                        // set raw pointer to start of 802.11 payload (start of Ethernet packet)
                        ethPacketLength = byteCount - ETH_HEADER_START_OFFSET;
                        RawRead(RAW_DATA_RX_ID, ETH_HEADER_START_OFFSET, ethPacketLength, p_packet->pDSeg->segLoad);

                        // if we received our own broadcast then throw it away
                        if ( memcmp(&p_packet->pDSeg->segLoad[6], p_Mac, 6) == 0)
                        {
                            DBG_RX_PRINT("8 ");
                            // put buffer back in free list
                            TCPIP_Helper_SingleListTailAdd(&s_DataRxQueue, (SGL_LIST_NODE *)p_packet);
                        }
                        // else flag packet as queued and signal stack to process it
                        else
                        {
                            DBG_RX_PRINT("9 ");
                            // mark packet as queued and stuff in timestamp
                            p_packet->pDSeg->segLen = ethPacketLength - ETH_HEADER_SIZE;
                            p_packet->pktFlags |= TCPIP_MAC_PKT_FLAG_QUEUED;
                            p_packet->tStamp = SYS_TMR_TickCountGet();

                            // Note: re-set pMacLayer and pNetLayer; IPv6 changes these pointers inside the packet, so
                            //       when Rx packets are reused this is needed.
                            p_packet->pMacLayer = p_packet->pDSeg->segLoad;
                            p_packet->pNetLayer = p_packet->pMacLayer + sizeof(TCPIP_MAC_ETHERNET_HEADER);

                            // store packet pointer in FIFO and signal stack that Rx packet ready to process
                            RxFifoInsert(&s_RxFifo, p_packet);

                            // notify stack of Rx packet
                            // SYS_CONSOLE_MESSAGE(" NS\r\n");
                            DRV_WIFI_UserEventSet(TCPIP_MAC_EV_RX_DONE, 0, false);
                        }
                    }
                    // else out of pre-queued Rx packets, or, could not allocate a new one
                    else
                    {
                        SYS_CONSOLE_MESSAGE("Out of Rx -- throw away packet\r\n");
                    }
                }
                // else if raw move timed out (note: invalid, proprietary packet is OK, so does not cause assert)
                else if (status == RM_TIMEOUT)
                {
                    DRV_WIFI_ASSERT(false, "");
                }
                else
                {
                    // We were hitting this case when a proprietary packet was received, so we should
                    // not assert, but simply throw packet away, and not assert as we had been doing.
                    ;
                }

                // begin raw unmount of data Rx packet (it's either been buffered for stack or being thrown away)
                DeallocateDataRxBuffer();
                s_RxDataState = WAIT_FOR_DATA_RX_UNMOUNT;
            }
            // else Raw Move not yet complete (still waiting)
            else
            {
                DRV_WIFI_ASSERT(false, "");
            }
            break;

        //======================================================================
        case WAIT_FOR_DATA_RX_UNMOUNT:
            if (isRawMoveComplete(RAW_DATA_RX_ID, &status, &byteCount))
            {
                DBG_RX_PRINT("10 ");
                // if timed out waiting for raw move to complete

                s_RxDataState = DATA_RX_IDLE;

                // if notified of new Rx packet while waiting for Raw move complete
                if (s_RxPacketPending == true)
                {
                    DBG_RX_PRINT("12 ");
                    // kick-start state machine again
                    s_RxPacketPending = false;
                    SignalRxStateMachine();
                    IncrementRxPendingCount();
                }
                // else if Tx state machine waiting for Rx state machine to go idle
                else if (s_TxWaitingForRxIdle == true)
                {
                    DBG_RX_PRINT("11 ");
                    // kick-start Tx state machine so it can finish
                    s_TxWaitingForRxIdle = false;
                    SignalTxStateMachine();
                }
            }
            else
            {
                DRV_WIFI_ASSERT(false, "");
            }
            break;
        } // end switch

        DBG_RX_PRINT("\r\n");
        DRV_WIFI_INT_SourceEnable();

    } else { // special case when running in MAC_Close() process

        SYS_INT_SourceDisable(MRF_INT_SOURCE);

        switch (s_RxDataState) {
        //======================================================================
        case DATA_RX_IDLE:

        //======================================================================
        case WAIT_FOR_TX_IDLE:
            break;

        //======================================================================
        case WAIT_FOR_DATA_RX_MOUNT:
            while (1) {
                // in de-init process, we stay and wait
                if (isRawMoveComplete(RAW_DATA_RX_ID, &status, &byteCount)) {
                    if ( !((byteCount > 0) && (byteCount <= 1530)) ) { // for debugging
                        if (byteCount == 0) {
                            SYS_CONSOLE_PRINT("Rx Packet status = %d, byteCount = %d\r\n", status, byteCount);
                        } else {
                            SYS_CONSOLE_PRINT("Rx Packet is larger than 1528 bytes for UDP or 1530 bytes for TCP, status = %d, byteCount = %d\r\n", status, byteCount);
                        }
                        DRV_WIFI_ASSERT(false, "");
                    }

                    // if raw move successfully completed and the Rx packet header is valid
                    if ((status == RM_COMPLETE) && (isRxPacketHeaderValid())) {
                        // get an Rx buffer structure from queue to copy Rx packet to (from MRF24W to Rx buffer struct)
                        p_packet = GetAvailRxBuf();

                        // if packet structure available
                        if (p_packet != NULL) {
                            // read Ethernet packet into host buffer
                            // set raw pointer to start of 802.11 payload (start of Ethernet packet)
                            ethPacketLength = byteCount - ETH_HEADER_START_OFFSET;
                            RawRead(RAW_DATA_RX_ID, ETH_HEADER_START_OFFSET, ethPacketLength, p_packet->pDSeg->segLoad);

                            // if we received our own broadcast then throw it away
                            if (memcmp(&p_packet->pDSeg->segLoad[6], p_Mac, 6) == 0) {
                                // put buffer back in free list
                                TCPIP_Helper_SingleListTailAdd(&s_DataRxQueue, (SGL_LIST_NODE *)p_packet);
                            } else { // else flag packet as queued and signal stack to process it
                                // mark packet as queued and stuff in timestamp
                                p_packet->pDSeg->segLen = ethPacketLength - ETH_HEADER_SIZE;
                                p_packet->pktFlags |= TCPIP_MAC_PKT_FLAG_QUEUED;
                                p_packet->tStamp = SYS_TMR_TickCountGet();

                                // Note: re-set pMacLayer and pNetLayer; IPv6 changes these pointers inside the packet, so
                                //       when Rx packets are reused this is needed.
                                p_packet->pMacLayer = p_packet->pDSeg->segLoad;
                                p_packet->pNetLayer = p_packet->pMacLayer + sizeof(TCPIP_MAC_ETHERNET_HEADER);

                                // store packet pointer in FIFO and signal stack that Rx packet ready to process
                                RxFifoInsert(&s_RxFifo, p_packet);

                                // notify stack of Rx packet
                                DRV_WIFI_UserEventSet(TCPIP_MAC_EV_RX_DONE, 0, false);
                            }
                        } else { // else out of pre-queued Rx packets, or, could not allocate a new one
                            SYS_CONSOLE_MESSAGE("Out of Rx -- throw away packet\r\n");
                        }
                    } else if (status == RM_TIMEOUT) {
                        // else if raw move timed out (note: invalid, proprietary packet is OK, so does not cause assert)
                        DRV_WIFI_ASSERT(false, "");
                    } else {
                        // we were hitting this case when a proprietary packet was received, so we should
                        // not assert, but simply throw packet away, and not assert as we had been doing
                        ;
                    }

                    // begin raw unmount of data Rx packet (it's either been buffered for stack or being thrown away)
                    DeallocateDataRxBuffer();
                    s_RxDataState = WAIT_FOR_DATA_RX_UNMOUNT;
                    break; // out of while (1) loop
                }
            }

        //======================================================================
        case WAIT_FOR_DATA_RX_UNMOUNT:
            while (1) {
                if (isRawMoveComplete(RAW_DATA_RX_ID, &status, &byteCount)) {
                    s_RxDataState = DATA_RX_IDLE;
                    // if notified of new Rx packet while waiting for raw move complete
                    if (s_RxPacketPending == true) {
                        // kick-start state machine again, but in de-init process, we actually do nothing
                        s_RxPacketPending = false;
                        SignalRxStateMachine();
                        IncrementRxPendingCount();
                    } else if (s_TxWaitingForRxIdle == true) {
                        // else if Tx state machine waiting for Rx state machine to go idle
                        // kick-start Tx state machine so it can finish
                        s_TxWaitingForRxIdle = false;
                        SignalTxStateMachine();
                    }
                    break; // out of while (1) loop
                }
            }
        } // end switch

        DRV_WIFI_INT_SourceEnable();

    } // end of if (!g_drv_wifi_priv.isInDriverClose)
}

static bool isRxPacketHeaderValid(void)
{
    RX_PACKET_HEADER_PREAMBLE wfPreamble;
    uint8_t snapHdr[6];

    if (s_readRssiEnabled) {
        struct {
            uint16_t rxPreamble, rssi, dummy1, dummy2, dummy3;
            uint8_t snapHeader[6], dstAddr[WF_MAC_ADDRESS_LENGTH], srcAddr[WF_MAC_ADDRESS_LENGTH];
            uint32_t type;
        } RAW_HEADER; /* 28 bytes */

        RawRead(RAW_DATA_RX_ID, 0, WF_RX_PREAMBLE_SIZE + 10, (uint8_t *)&RAW_HEADER);
        DRV_WIFI_RSSI_Cache_FromRxDataRead(RAW_HEADER.rssi >> 8); /* RSSI_MAX (128), RSSI_MIN (43). The larger the stronger */
        memcpy((void *)&wfPreamble, (void *)&RAW_HEADER.rxPreamble, sizeof(RX_PACKET_HEADER_PREAMBLE));
        DRV_WIFI_ASSERT(wfPreamble.type == WF_DATA_RX_INDICATE_TYPE, "");
        memcpy((void *)&snapHdr, (void *)RAW_HEADER.snapHeader, sizeof(RAW_HEADER.snapHeader));
    } else {
        RawRead(RAW_DATA_RX_ID, 0, sizeof(RX_PACKET_HEADER_PREAMBLE), (uint8_t *)&wfPreamble);
        DRV_WIFI_ASSERT(wfPreamble.type == WF_DATA_RX_INDICATE_TYPE, "");
        RawRead(RAW_DATA_RX_ID, SNAP_HEADER_OFFSET, 6, (uint8_t *)&snapHdr);
    }

    // verify that the expected bytes contain the SNAP header; if not, this is a
    // proprietary packet that will be thrown away by Rx state machine.
    if (!(snapHdr[0] == SNAP_VAL &&
          snapHdr[1] == SNAP_VAL &&
          snapHdr[2] == SNAP_CTRL_VAL &&
          snapHdr[3] == SNAP_TYPE_VAL &&
          snapHdr[4] == SNAP_TYPE_VAL &&
          snapHdr[5] == SNAP_TYPE_VAL)) {
        return false;
    }

    return true;
}

// finds an available Rx packet structure from the list that was allocated and
// queued up at init
static TCPIP_MAC_PACKET *GetAvailRxBuf(void)
{
    TCPIP_MAC_PACKET *p_packet = NULL;

    // if free list has an available Rx packet buffer
    if (s_DataRxQueue.nNodes > 0)
    {
        p_packet = (TCPIP_MAC_PACKET *)TCPIP_Helper_SingleListHeadRemove(&s_DataRxQueue);
        DRV_WIFI_ASSERT(p_packet != NULL, "");
    }
    return p_packet;
}

bool isDataTxTaskInactive(void)
{
    return (s_TxDataState == DATA_TX_IDLE);
}

// retrieve the oldest of the queued Rx packets to deliver to the stack
TCPIP_MAC_PACKET *MRF24W_RxPacketGet(void)
{
    if (!isRxFifoEmpty(&s_RxFifo))
    {
        return RxFifoRemove(&s_RxFifo);
    }
    else
    {
        return NULL; // signals no Rx packet to process
    }
}

static uint16_t GetTxPacketLength(TCPIP_MAC_PACKET *p_packet)
{
    uint16_t packetLength;
    TCPIP_MAC_DATA_SEGMENT *p_seg = p_packet->pDSeg; // point to first segment

    // number of bytes in first segment
    packetLength = p_seg->segLen;

    // loop thru any other segments and add their data length
    while (p_seg->next != NULL)
    {
        p_seg = p_seg->next;
        packetLength += p_seg->segLen;
    }

    // return the packet length plus extra 4 bytes needed for internal header
    return packetLength + WF_TX_DATA_MSG_PREAMBLE_LENGTH;
}

// called by MRF24W_MACTxPacket when stack wants to send a packet
TCPIP_MAC_RES MRF24W_TxPacketSend(TCPIP_MAC_PACKET *p_packet)
{
    if (DRV_WIFI_InHibernateMode())
    {
        SYS_CONSOLE_MESSAGE("MRF24WG is in Hibernate mode, dropped the Tx packet\r\n");
        return TCPIP_MAC_RES_QUEUE_TX_FULL;
    }

    // if we get too many Tx packets queued up, start throwing them away
    if (s_DataTxQueue.nNodes >= MAX_TX_QUEUE_COUNT)
    {
        SYS_CONSOLE_PRINT("Tx queue is full (%d), dropped the Tx packet\r\n", s_DataTxQueue.nNodes);
        return TCPIP_MAC_RES_QUEUE_TX_FULL;
    }

    // if a packet size is too huge, we drop the packet.
    if (p_packet->pDSeg->segLen > 1518)
    {
        SYS_CONSOLE_PRINT("Dropped the Tx packet. Invalid packet length. Too large: %d\r\n",p_packet->pDSeg->segLen);
        if (s_pktAckF)
        {
            _TCPIP_PKT_ACK_BY_PTR(s_pktAckF, p_packet, TCPIP_MAC_PKT_ACK_TX_OK);
        }
        return TCPIP_MAC_RES_OK;
    }

    // queue the Tx Data packet pointer
    TCPIP_Helper_SingleListTailAdd(&s_DataTxQueue, (SGL_LIST_NODE *)p_packet);

    // signal list manager that this packet cannot be reused until the
    // ack function is called.
    p_packet->pktFlags |= TCPIP_MAC_PKT_FLAG_QUEUED;

    // If there is only a single message in the queue then
    // kick-started the Tx Data Task.  If there is more than one message in the queue
    // then we've had at least two calls to this function before the Tx Data Task had
    // a chance to run, but we only want to call it once, not twice.  The task
    // will check for additional Tx packets when it finishes the previous one.
    if (s_DataTxQueue.nNodes == 1) // node check debug
    {
        SignalTxStateMachine();
    }

    return TCPIP_MAC_RES_OK;
}

static void PrepTxPacketHeaders(TCPIP_MAC_PACKET *p_packet)
{
    uint8_t *p_load;
    TCPIP_MAC_ETHERNET_HEADER *p_etherHeader;
    uint16_t type;

    p_etherHeader = (TCPIP_MAC_ETHERNET_HEADER *)(p_packet->pMacLayer);
    type = p_etherHeader->Type;

    // overwrite source address in Ethernet header with SNAP header.  The Ethernet
    // source address starts at index 6 in the Ethernet packet
    p_load = p_packet->pDSeg->segLoad; // start of packet

    // overwrite bytes [6] thru [11] in ethernet header to SNAP header
    p_load[6] = SNAP_VAL;
    p_load[7] = SNAP_VAL;
    p_load[8] = SNAP_CTRL_VAL;
    p_load[9] = SNAP_TYPE_VAL;
    p_load[10] = SNAP_TYPE_VAL;
    p_load[11] = SNAP_TYPE_VAL;

    // overwrite indexes [12], [13] with Ethernet type for protocol being used (already in network order)
    p_load[12] = (uint8_t)type;
    p_load[13] = (uint8_t)(type >> 8);

    // now that the packet has been modified for WiFi mac, prepend the internal msg
    // header used by SPI interface.  These four bytes are immediately prior to the
    // p_DSeg pointer (the actual 'secret' buffer is the size of segLoadOffset).
    p_load = p_packet->pDSeg->segLoad - WF_TX_PREAMBLE_SIZE; // have p_load point to 4 bytes before pDSeg->segLoad

    // write out internal preamble
    p_load[0] = WF_DATA_REQUEST_TYPE;
    p_load[1] = WF_STD_DATA_MSG_SUBTYPE;
    p_load[2] = 1;
    p_load[3] = 0;
}

static bool isTxStateMachineIdle(void)
{
    // Tx state machine is logically idle if its state is idle or it is waiting for the Rx state machine to go idle
    return (s_TxDataState == DATA_TX_IDLE) || (s_TxDataState == WAIT_FOR_RX_IDLE);
}

bool isRxStateMachineIdle(void)
{
    return (s_RxDataState == DATA_RX_IDLE);
}

void SetRxPacketPending(void)
{
    s_RxPacketPending = true;
}

void DRV_WIFI_DataTxTask(void)
{
    int status;
    uint16_t byteCount;
    uint8_t *p_segData;
    TCPIP_MAC_DATA_SEGMENT *p_seg;
    static TCPIP_MAC_PACKET *p_packet = NULL; // packet currently being sent
    static uint16_t packetLength = 0; // total length of buffer needed on MRF
    bool txStateMachineActive = true;

    SYS_INT_SourceDisable(MRF_INT_SOURCE);
    DBG_TX_PRINT("TX: ");

    while (txStateMachineActive)
    {
        txStateMachineActive = false;

        switch (s_TxDataState) {
        //======================================================================
        case DATA_TX_IDLE:
            DBG_TX_PRINT("1 ");
            // if any Tx packets in queue
            if (s_DataTxQueue.nNodes > 0)
            {
                DBG_TX_PRINT("2 ");
                // point to Tx packet, but do not yet remove it from the queue
                p_packet = (TCPIP_MAC_PACKET *)s_DataTxQueue.head;
                DRV_WIFI_ASSERT(p_packet != NULL, "");

                // prepend internal WiFi header and modify Ethernet header to SNAP; get packet length
                PrepTxPacketHeaders(p_packet);
                packetLength = GetTxPacketLength(p_packet);

                if (isRxStateMachineIdle())
                {
                    DBG_TX_PRINT("3 ");
                    // start raw mount if bytes available
                    if (AllocateDataTxBuffer(packetLength))
                    {
                        DBG_TX_PRINT("4 ");
                        s_TxDataState = WAIT_FOR_DATA_TX_MOUNT;
                    }
                    // else not enough bytes available on MRF right now
                    else
                    {
                        DBG_TX_PRINT("5 ");
                        // schedule state machine to run again and check if it can proceed
                        SignalTxStateMachine();
                        s_TxDataState = WAIT_FOR_TX_MEMORY;
                    }
                }
                // else need to wait for Rx state machine to finish
                else
                {
                    DBG_TX_PRINT("6 ");
                    s_TxDataState = WAIT_FOR_RX_IDLE;
                    s_TxWaitingForRxIdle = true; // let Rx state machine know that Tx state machine is waiting for it
                }
            }
            else
            {
                DRV_WIFI_ASSERT(p_packet != NULL, "");
            }
            break;

        //======================================================================
        case WAIT_FOR_RX_IDLE:
            DBG_TX_PRINT("7 ");
            if (isRxStateMachineIdle()) // should always be true
            {
                // start raw mount if bytes available
                if (AllocateDataTxBuffer(packetLength))
                {
                    DBG_TX_PRINT("8 ");
                    s_TxDataState = WAIT_FOR_DATA_TX_MOUNT;
                }
                // else not enough bytes available on MRF right now
                else
                {
                    DBG_TX_PRINT("9 ");
                    // schedule state machine to run again
                    SignalTxStateMachine();
                    s_TxDataState = WAIT_FOR_TX_MEMORY;
                }
            }
            else
            {
                DRV_WIFI_ASSERT(false, "");
            }
            break;

        //======================================================================
        case WAIT_FOR_TX_MEMORY:
            DBG_TX_PRINT("10 ");
            if (isRxStateMachineIdle())
            {
                DBG_TX_PRINT("11 ");
                // attempt to allocate data buffer on MRF (if bytes available then Raw Move has been initiated)
                if (AllocateDataTxBuffer(packetLength))
                {
                    DBG_TX_PRINT("12 ");
                    s_TxDataState = WAIT_FOR_DATA_TX_MOUNT;
                }
                // else not enough bytes available on MRF right now
                else
                {
                    DBG_TX_PRINT("13 ");
                    // schedule state machine to run again
                    SignalTxStateMachine(); // should not make a difference
                }
            }
            else
            {
                DBG_TX_PRINT("14 ");
                s_TxDataState = WAIT_FOR_RX_IDLE;
                s_TxWaitingForRxIdle = true; // let Rx state machine know that Tx state machine is waiting for it
            }
            break;

        //======================================================================
        case WAIT_FOR_DATA_TX_MOUNT:
            DBG_TX_PRINT("15 ");
            // if Tx data buffer successfully mounted (or timed out trying)
            if (isRawMoveComplete(RAW_DATA_TX_ID, &status, &byteCount))
            {
                DBG_TX_PRINT("16 ");
                if ((status == RM_COMPLETE) && (byteCount != 0))
                {
                    DBG_TX_PRINT("17 ");
                    // if, after the raw mount, we didn't get all the bytes we needed
                    if (byteCount < packetLength)
                    {
                        SYS_CONSOLE_MESSAGE("IVB\r\n");
                    }

                    uint16_t curIndex = 0;

                    // write out the first segment to MRF, including prepended internal header
                    p_seg = p_packet->pDSeg; // point to first segment
                    p_segData = p_seg->segLoad - WF_TX_PREAMBLE_SIZE;
                    RawWrite(RAW_DATA_TX_ID, 0, p_seg->segLen + WF_TX_PREAMBLE_SIZE, p_segData);
                    curIndex += p_seg->segLen + WF_TX_PREAMBLE_SIZE;

                    // write out any other segments to MRF
                    while (p_seg->next != NULL)
                    {
                        DBG_TX_PRINT("18 ");
                        p_seg = p_seg->next;
                        p_segData = p_seg->segLoad;
                        RawWrite(RAW_DATA_TX_ID, curIndex, p_seg->segLen, p_segData);
                        curIndex += p_seg->segLen;
                    }

                    // Now that full packet copied to MRF24WG, signal it that packet ready to transmit (start raw move)
                    SendRAWDataFrame(packetLength);

                    s_TxDataState = WAIT_FOR_DATA_TX_SIGNAL;
                }
                else if (byteCount == 0)
                {
                    DRV_WIFI_ASSERT(false, "");
                }
            }
            else
            {
                DRV_WIFI_ASSERT(false, "");
            }
            break;

        //======================================================================
        case WAIT_FOR_DATA_TX_SIGNAL:
            DBG_TX_PRINT("19 ");
            // if raw move complete that signals MRF24WG that there is a Tx data
            // packet to transmit
            if (isRawMoveComplete(RAW_DATA_TX_ID, &status, &byteCount))
            {
                DBG_TX_PRINT("20 ");
                if (status == RM_COMPLETE)
                {
                    DBG_TX_PRINT("21 ");

                    // remove Tx packet from the list
                    p_packet = (TCPIP_MAC_PACKET *)TCPIP_Helper_SingleListHeadRemove(&s_DataTxQueue);
                    //SYS_CONSOLE_PRINT("R=%d\r\n", s_DataTxQueue.nNodes);

                    // call stack ack function to let it know packet was transmitted
                    if (s_pktAckF)
                    {
                        _TCPIP_PKT_ACK_BY_PTR(s_pktAckF, p_packet, TCPIP_MAC_PKT_ACK_TX_OK);
                    }
                    else
                    {
                        DRV_WIFI_ASSERT(false, "");
                    }

                    // if any pending Tx packets
                    if (s_DataTxQueue.nNodes > 0)
                    {
                        DBG_TX_PRINT("22 ");
                        txStateMachineActive = true;
                    }

                    s_TxDataState = DATA_TX_IDLE; // ready for next Tx data
                }
                else
                {
                    DRV_WIFI_ASSERT(false, "");
                }
            }
            else
            {
                DRV_WIFI_ASSERT(false, "");
            }
            break;
        } // end switch

    } // end while

    DRV_WIFI_INT_SourceEnable();

    DBG_TX_PRINT("\r\n");
}

void TxQueueDeinit(void)
{
    TCPIP_MAC_PACKET *p_packet = NULL;
    // remove Tx packet from the list
    while (s_DataTxQueue.nNodes > 0)
    {
        p_packet = (TCPIP_MAC_PACKET *)TCPIP_Helper_SingleListHeadRemove(&s_DataTxQueue);
        // call stack ack function to let it know packet was transmitted
        if (s_pktAckF)
        {
            _TCPIP_PKT_ACK_BY_PTR(s_pktAckF, p_packet, TCPIP_MAC_PKT_ACK_TX_OK);
        }
        else
        {
            DRV_WIFI_ASSERT(false, "");
        }
    }
}

//DOM-IGNORE-END
