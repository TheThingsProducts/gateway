/*******************************************************************************
  MRF24WG Driver Initialization

  File Name:
    drv_wifi_init.c

  Summary:
    MRF24WG Driver Initialization

  Description:
    -Provides access to MRF24WG Wi-Fi controller
    -Reference: MRF24WG Datasheet, IEEE 802.11 Standard
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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

#include "drv_wifi_commands.h"
#include "drv_wifi_config_data.h"
#include "drv_wifi_debug_output.h"
#include "drv_wifi_eint.h"
#include "drv_wifi_mac.h"
#include "drv_wifi_raw.h"

/*****************/
/*    DEFINES    */
/*****************/
#define EXPECTED_MRF24W_VERSION_NUMBER (2)

/* This MAC address is the default MAC address used in TCP/IP stack.  If the */
/* user leaves this MAC address unchanged then the Wi-Fi Driver will get the */
/* unique MAC address from the MRF24W and have the stack use it.             */
#define MCHP_DEFAULT_MAC_ADDRESS_BYTE_1 (0x00)
#define MCHP_DEFAULT_MAC_ADDRESS_BYTE_2 (0x04)
#define MCHP_DEFAULT_MAC_ADDRESS_BYTE_3 (0xa3)
#define MCHP_DEFAULT_MAC_ADDRESS_BYTE_4 (0x00)
#define MCHP_DEFAULT_MAC_ADDRESS_BYTE_5 (0x00)
#define MCHP_DEFAULT_MAC_ADDRESS_BYTE_6 (0x00)

#define WF_NOTIFY_CONNECTION_ATTEMPT_SUCCESSFUL ((uint8_t)(0x01))
#define WF_NOTIFY_CONNECTION_ATTEMPT_FAILED     ((uint8_t)(0x02))
#define WF_NOTIFY_CONNECTION_TEMPORARILY_LOST   ((uint8_t)(0x04))
#define WF_NOTIFY_CONNECTION_PERMANENTLY_LOST   ((uint8_t)(0x08))
#define WF_NOTIFY_CONNECTION_REESTABLISHED      ((uint8_t)(0x10))
#define WF_NOTIFY_ALL_EVENTS                    ((uint8_t)(0x1f))

// Initialize SM states
typedef enum
{
    I_INIT_BEGIN          = 0,
    I_CHIP_RESET          = 1,
    I_RAW_INIT            = 2,
    I_SEND_INIT_MGMT_MSGS = 4,
    I_INIT_COMPLETE       = 5,
} INIT_STATE;

typedef enum
{
    CR_BEGIN                = 0,
    CR_WAIT_HW_RESET        = 1,
    CR_WAIT_FIFO_BYTE_COUNT = 2,
} CHIPRESET_STATE;

/*******************/
/*    VARIABLES    */
/*******************/
/* This MAC address is the default MAC address used in TCP/IP stack.  If the */
/* user leaves this MAC address unchanged then the Wi-Fi Driver will get the */
/* unique MAC address from the MRF24W and have the stack use it.             */
static const uint8_t MchpDefaultMacAddress[WF_MAC_ADDRESS_LENGTH] = {0x00u, 0x04u, 0xA3u, 0x00u, 0x00u, 0x00u};

static uint8_t s_initState;
static uint8_t s_chipResetState;
static bool s_chipResetTimeout;

uint16_t g_mgmt_base; // mgmt msg base index in scratch memory

/***********************************/
/*    LOCAL FUNCTION PROTOTYPES    */
/***********************************/
static int ChipResetStateMachine(void);
static void MacAddrSet(void);
static void ChipResetTimeoutCallback(uintptr_t context, uint32_t currTick);
static void InitWiFiInterrupts(void);
static void HostInterrupt2RegInit(uint16_t hostIntMaskRegMask, uint8_t state);
static void HostInterruptRegInit(uint8_t hostIntrMaskRegMask, uint8_t state);

/*******************************************************************************
 * Function:        TCPIP_MAC_RES DRV_WIFI_SystemInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          TCPIP_MAC_RES_OK if initialization succeeded,
 *                  TCPIP_MAC_RES_PENDING if initialization was ongoing,
 *                  error code otherwise.
 *
 * Side Effects:    None
 *
 * Overview:        Performs the Wi-Fi driver initialization, resets
 *                  MRF24WG, and configures MRF24WG for operations.
 *
 * Note:            None
 *******************************************************************************/
TCPIP_MAC_RES DRV_WIFI_SystemInit(void)
{
    TCPIP_MAC_RES res = TCPIP_MAC_RES_PENDING;
    RAW_INIT_STATUS rawInitStatus;
    DRV_WIFI_DEVICE_INFO deviceInfo;

    // Check for and process events. The async handler is
    // not invoked during initialization at a system level, so
    // it needs to be done here.
    DRV_WIFI_PendingEventHandle();

    switch (s_initState) {
    //----------------------------------------
    case I_INIT_BEGIN:
    //----------------------------------------
        if (!DRV_WIFI_ConfigDataLoad())
        {
            res = TCPIP_MAC_RES_INIT_FAIL;
        }

        /* initialize g_drv_wifi_priv, intentionally exclude initConn */
        g_drv_wifi_priv.isDriverOpen = false;
        g_drv_wifi_priv.isInDriverClose = false;
        g_drv_wifi_priv.isScanDone = false;
        g_drv_wifi_priv.isConnReestablished = false;
        g_drv_wifi_priv.isConnPermanentlyLost = false;

#ifdef DRV_WIFI_USE_SPI_DMA
        if (!DRV_WIFI_SpiInit(DRV_WIFI_SpiDmaRx, DRV_WIFI_SpiDmaTx))
#else /* !DRV_WIFI_USE_SPI_DMA */
        /* initialize the SPI interface */
        if (!DRV_WIFI_SpiInit(DRV_WIFI_SpiRx, DRV_WIFI_SpiTx))
#endif /* DRV_WIFI_USE_SPI_DMA */
        {
            res = TCPIP_MAC_RES_INIT_FAIL;
        }

        /* Toggle the module into and then out of hibernate */
        WF_SetCE_N(WIFI_PIN_HIGH); /* disable module */
        WF_SetCE_N(WIFI_PIN_LOW); /* enable module */

        /* Toggle the module into and out of reset */
        WF_SetRST_N(WIFI_PIN_LOW); // put module into reset
        WF_SetRST_N(WIFI_PIN_HIGH); // take module out of of reset

        SYS_INT_Enable(); // for SPI + DMA interrupt mode, we have to enable INT now
        ResetPll(); // needed until PLL fix made in A2 silicon
        DRV_WIFI_AllEventClear();
        s_chipResetState = CR_BEGIN;
        WF_INIT_MUTEX_LOCK();
        s_initState = I_CHIP_RESET;
        WF_INIT_MUTEX_UNLOCK();
#if defined(MRF24W_USE_CN_INT)
        // if don't read pin status, board cannot boot after programming
        SYS_PORTS_PinRead(PORTS_ID_0, WF_INT_PORT_CHANNEL, WF_INT_BIT_POS);
#endif
        break;

    //----------------------------------------
    case I_CHIP_RESET:
    //----------------------------------------
        res = ChipResetStateMachine();
        // if chip reset complete
        if (res == TCPIP_MAC_RES_OK)
        {
            // TBD -- disable low power mode
            res = TCPIP_MAC_RES_PENDING; // not done initializing, so set code back to busy
            RawResetInitStateMachine();
            WF_INIT_MUTEX_LOCK();
            s_initState = I_RAW_INIT;
            WF_INIT_MUTEX_UNLOCK();
        }
        break;

    //----------------------------------------
    case I_RAW_INIT:
    //----------------------------------------
        rawInitStatus = RawInitStateMachine();
        // if RAW init complete
        if (rawInitStatus == RAW_INIT_COMPLETE)
        {
            WF_INIT_MUTEX_LOCK();
            s_initState = I_SEND_INIT_MGMT_MSGS;
            WF_INIT_MUTEX_UNLOCK();
            res = TCPIP_MAC_RES_PENDING; // not done initializing, so still busy with init
        }
        // else if not still busy doing raw init then raw init failed
        else if (res != TCPIP_MAC_RES_PENDING)
        {
            // TBD: signal Wi-Fi fatal error
            res = TCPIP_MAC_RES_INIT_FAIL;
        }
        break;

    //----------------------------------------
    case I_SEND_INIT_MGMT_MSGS:
    //----------------------------------------
        g_mgmt_base = DRV_WIFI_MgmtBaseGet();
        DRV_WIFI_DeviceInfoGet(&deviceInfo);
        ConnectionProfileCreate();
        DRV_WIFI_AllEventClear();
        DRV_WIFI_ASSERT(deviceInfo.romVersion == 0x31 || deviceInfo.romVersion == 0x32, "");

        /* send init messages to MRF24W */
        MacAddrSet();

        // init data tx and rx state machines
        DRV_WIFI_TxTaskInit();
        if (DRV_WIFI_RxTaskInit() == TCPIP_MAC_RES_INIT_FAIL)
        {
            // before returning failure code, deallocate any packets previously allocated in this loop
            RxQueueDeinit();
            res = TCPIP_MAC_RES_INIT_FAIL;
        }

        #if defined(SYS_CONSOLE_ENABLE) || defined(SYS_CONSOLE_INSTANCES_NUMBER)
        DRV_WIFI_ConfigDataValidate();
        #endif

#if defined(TCPIP_STACK_COMMANDS_WIFI_ENABLE)
        // initialize Wi-Fi console commands
        Cmd_Init();
#endif

        WF_INIT_MUTEX_LOCK();
        s_initState = I_INIT_COMPLETE;
        WF_INIT_MUTEX_UNLOCK();

    //-----------------------------------------
    case I_INIT_COMPLETE:
    //-----------------------------------------
        res = TCPIP_MAC_RES_OK;
        break;
    } // end switch

    return res;
}

#if defined(DRV_WIFI_USE_FREERTOS)
void DRV_WIFI_InitTask(void *p_arg)
{
    while (1) {
        if (DRV_WIFI_SystemInit() == TCPIP_MAC_RES_INIT_FAIL) {
            ((MRF24W_MAC_DCPT *)p_arg)->sysStat = SYS_STATUS_ERROR;
            SYS_CONSOLE_MESSAGE("Wi-Fi Driver reports SYS_STATUS_ERROR!\r\n");
            while (1)
                DRV_WIFI_TaskDelay(SYS_TMR_TickCounterFrequencyGet() / 1000);
        }
        DRV_WIFI_TaskDelay(SYS_TMR_TickCounterFrequencyGet() / 1000);
    }
}
#endif /* defined(DRV_WIFI_USE_FREERTOS) */

bool isInitComplete(void)
{
    return s_initState == I_INIT_COMPLETE;
}

static void ChipResetTimeoutCallback(uintptr_t context, uint32_t currTick)
{
    s_chipResetTimeout = true;
}

static int ChipResetStateMachine(void)
{
    uint16_t value;
    uint16_t timeout;
    static SYS_TMR_HANDLE timer = 0;

    TCPIP_MAC_RES res = TCPIP_MAC_RES_PENDING;

    DRV_WIFI_HibernateModeClear();  // reset always clears Hibernate mode

    switch (s_chipResetState) {
    //----------------------------------------
    case CR_BEGIN:
    //----------------------------------------
        /* clear the power bit to disable low power mode on the MRF24W */
        Write16BitWFRegister(WF_PSPOLL_H_REG, 0x0000);

        /* set HOST_RESET bit in register to put device in reset */
        Write16BitWFRegister(WF_HOST_RESET_REG, Read16BitWFRegister(WF_HOST_RESET_REG) | WF_HOST_RESET_MASK);

        /* clear HOST_RESET bit in register to take device out of reset */
        Write16BitWFRegister(WF_HOST_RESET_REG, Read16BitWFRegister(WF_HOST_RESET_REG) & ~WF_HOST_RESET_MASK);

        /* after reset is started poll register to determine when HW reset has completed */
        s_chipResetTimeout = false;
        timeout = SYS_TMR_TickCounterFrequencyGet() * 3;
        timer = SYS_TMR_CallbackSingle(timeout, 0, ChipResetTimeoutCallback);

        s_chipResetState = CR_WAIT_HW_RESET;
        break;

    //----------------------------------------
    case CR_WAIT_HW_RESET:
    //----------------------------------------
        // if HW reset completed
        Write16BitWFRegister(WF_INDEX_ADDR_REG, WF_HW_STATUS_REG);
        value = Read16BitWFRegister(WF_INDEX_DATA_REG);
        if ((value & WF_HW_STATUS_NOT_IN_RESET_MASK) != 0)
        {
            // if value 0xffff then most likely SPI is not connected
            if (value == 0xffff)
            {
                res = TCPIP_MAC_RES_INIT_FAIL;
            }
            // else successful HW reset, set timer and go to next state
            else
            {
                uint16_t timeout;

                s_chipResetTimeout = false;
                timeout = SYS_TMR_TickCounterFrequencyGet() * 3;
                SYS_TMR_CallbackStop(timer);
                timer = SYS_TMR_CallbackSingle(timeout, 0, ChipResetTimeoutCallback);

                s_chipResetState = CR_WAIT_FIFO_BYTE_COUNT;
            }
        }
        // else if timed out waiting for HW reset
        else if (s_chipResetTimeout)
        {
            res = TCPIP_MAC_RES_INIT_FAIL;
        }
        break;

    //----------------------------------------
    case CR_WAIT_FIFO_BYTE_COUNT:
    //----------------------------------------
        // if chip reset complete
        value = Read16BitWFRegister(WF_HOST_WFIFO_BCNT0_REG);
        if (value != 0)
        {
            SYS_TMR_CallbackStop(timer);
            InitWiFiInterrupts();
            res = TCPIP_MAC_RES_OK;
        }
        // else timed out
        else if (s_chipResetTimeout)
        {
            res = TCPIP_MAC_RES_INIT_FAIL;
        }
        break;

    } // end switch

    return res;
}

static void InitWiFiInterrupts(void)
{
    uint8_t  mask8;
    uint16_t mask16;

    /* disable the interrupts gated by the 16-bit host int register */
    HostInterrupt2RegInit(WF_HOST_2_INT_MASK_ALL_INT, (uint16_t)WF_INT_DISABLE);

    /* disable the interrupts gated the by main 8-bit host int register */
    HostInterruptRegInit(WF_HOST_INT_MASK_ALL_INT, WF_INT_DISABLE);

    /* initialize the External Interrupt allowing the MRF24W to interrupt */
    /* the Host from this point forward.                                  */
    DRV_WIFI_INT_Init();
    DRV_WIFI_INT_SourceEnable();

    /* enable the following MRF24W interrupts in the INT1 8-bit register */
    mask8 = (WF_HOST_INT_MASK_FIFO_0_THRESHOLD | /* Data Rx Msg interrupt                  */
             WF_HOST_INT_MASK_RAW_0_INT_0      | /* RAW0 Move Complete (Data Rx) interrupt */
             WF_HOST_INT_MASK_RAW_1_INT_0      | /* RAW1 Move Complete (Data Tx) interrupt */
             WF_HOST_INT_MASK_INT2);             /* Interrupt 2 interrupt                  */
    HostInterruptRegInit(mask8, WF_INT_ENABLE);

    /* enable the following MRF24W interrupts in the INT2 16-bit register */
    mask16 = (WF_HOST_INT_MASK_RAW_4_INT_0     | /* RAW4 Move Complete (Scratch) interrupt */
              WF_HOST_INT_MASK_MAIL_BOX_1_WRT);  /* used for mgmt msg signalling           */
    HostInterrupt2RegInit(mask16, WF_INT_ENABLE);
}

/*******************************************************************************
 * FUNCTION: HostInterrupt2RegInit
 *
 * RETURNS: N/A
 *
 * PARAMS:
 *      hostIntrMaskRegMask - The bit mask to be modified.
 *      state               - One of WF_INT_DISABLE, WF_INT_ENABLE where
 *                             Disable implies clearing the bits and enable sets the bits.
 *
 *
 *  NOTES: Initializes the 16-bit Host Interrupt register on the MRF24W with the
 *      specified mask value either setting or clearing the mask register
 *      as determined by the input parameter state.
 *******************************************************************************/
static void HostInterrupt2RegInit(uint16_t hostIntMaskRegMask, uint8_t  state)
{
    uint16_t int2MaskValue;

    /* Host Int Register is a status register where each bit indicates a specific event  */
    /* has occurred. In addition, writing a 1 to a bit location in this register clears  */
    /* the event.                                                                        */

    /* Host Int Mask Register is used to enable those events activated in Host Int Register */
    /* to cause an interrupt to the host                                                    */

    /* read current state of int2 mask reg */
    int2MaskValue = Read16BitWFRegister(WF_HOST_INTR2_MASK_REG);

    /* if caller is disabling a set of interrupts */
    if (state == WF_INT_DISABLE)
    {
        /* zero out that set of interrupts in the interrupt mask copy */
        int2MaskValue &= ~hostIntMaskRegMask;
    }
    /* else caller is enabling a set of interrupts */
    else
    {
        /* set to 1 that set of interrupts in the interrupt mask copy */
        int2MaskValue |= hostIntMaskRegMask;
    }

    /* write out new interrupt mask value */
    Write16BitWFRegister(WF_HOST_INTR2_MASK_REG, int2MaskValue);

    /* ensure that pending interrupts from those updated interrupts are cleared */
    Write16BitWFRegister(WF_HOST_INTR2_REG, hostIntMaskRegMask);
}

/*******************************************************************************
 * FUNCTION: HostInterruptRegInit
 *
 * RETURNS: N/A
 *
 * PARAMS:
 *      hostIntrMaskRegMask - The bit mask to be modified.
 *      state -  one of WF_EXINT_DISABLE, WF_EXINT_ENABLE where
 *                Disable implies clearing the bits and enable sets the bits.
 *
 *
 *  NOTES: Initializes the 8-bit Host Interrupt register on the MRF24W with the
 *      specified mask value either setting or clearing the mask register
 *      as determined by the input parameter state.  The process requires
 *      2 spi operations which are performed in a blocking fashion.  The
 *      function does not return until both spi operations have completed.
 *******************************************************************************/
static void HostInterruptRegInit(uint8_t hostIntrMaskRegMask, uint8_t state)
{
    uint8_t hostIntMaskValue;

    /* Host Int Register is a status register where each bit indicates a specific event  */
    /* has occurred. In addition, writing a 1 to a bit location in this register clears  */
    /* the event.                                                                        */

    /* Host Int Mask Register is used to enable those events activated in Host Int Register */
    /* to cause an interrupt to the host                                                    */

    /* read current state of Host Interrupt Mask register */
    hostIntMaskValue = Read8BitWFRegister(WF_HOST_MASK_REG);

    /* if caller is disabling a set of interrupts */
    if (state == WF_INT_DISABLE)
    {
        /* zero out that set of interrupts in the interrupt mask copy */
        hostIntMaskValue = (hostIntMaskValue & ~hostIntrMaskRegMask);
    }
    /* else caller is enabling a set of interrupts */
    else
    {
        /* set to 1 that set of interrupts in the interrupt mask copy */
        hostIntMaskValue = (hostIntMaskValue & ~hostIntrMaskRegMask) | hostIntrMaskRegMask;
    }

    /* write out new interrupt mask value */
    Write8BitWFRegister(WF_HOST_MASK_REG, hostIntMaskValue);

    /* ensure that pending interrupts from those updated interrupts are cleared */
    Write8BitWFRegister(WF_HOST_INTR_REG, hostIntrMaskRegMask);
}

// *****************************************************************************
/*
  Function:
        void DRV_WIFI_Initialize(void)

  Summary:
    Initializes the MRF24WG Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function initializes the MRF24WG driver, making it ready for
    clients to use.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    This function must be called before any other Wi-Fi routine is called.
    The real work of Wi-Fi initialization takes place in an internal state
    machine, whose state is set to the initial value by this function.
*/
void DRV_WIFI_Initialize(void)
{
    // Initialization takes place in DRV_WIFI_SystemInit().
    s_initState = I_INIT_BEGIN;
}

//*******************************************************************************
/*
  Function:
        void DRV_WIFI_Deinitialize(void)

  Summary:
    De-initializes the MRF24WG Wi-Fi driver.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function de-initializes the MRF24WG driver. It also saves the Wi-Fi
    parameters in non-volatile storage.

  Precondition:
    None.

  Returns:
    None.

  Remarks:
    None.
*/
void DRV_WIFI_Deinitialize(void)
{
    RxQueueDeinit();
    TxQueueDeinit();
    StopRawMoveTimer(RAW_DATA_RX_ID);
    StopRawMoveTimer(RAW_DATA_TX_ID);
    StopRawMoveTimer(RAW_SCRATCH_ID);
    g_RxDataLock = true;
    DRV_WIFI_ConfigDataSave();
}

static void MacAddrSet(void)
{
    const TCPIP_MAC_MODULE_CTRL *p_ctrl = GetStackData();
    uint8_t allZeroes[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    /* If the user has left the default MAC address in TCP/IP stack unchanged then use */
    /* the unique MRF24W MAC address so prevent multiple devices from having the same  */
    /* MAC address.                                                                    */
    if ( (memcmp((void *)p_ctrl->ifPhyAddress.v, (void *)MchpDefaultMacAddress, WF_MAC_ADDRESS_LENGTH) == 0) ||
         (memcmp((void *)p_ctrl->ifPhyAddress.v, (void *)allZeroes, WF_MAC_ADDRESS_LENGTH) == 0) )
    {
        /* get the MRF24W MAC address and overwrite the MAC in pNetIf */
        DRV_WIFI_MacAddressGet((uint8_t *)(p_ctrl->ifPhyAddress.v));
    }
    /* Else presume the user has a unique MAC address of their own that they wish to use. */
    else
    {
        /* set MAC address with user-supplied MAC */
        DRV_WIFI_MacAddressSet((uint8_t *)(p_ctrl->ifPhyAddress.v));
    }
}

// DOM-IGNORE-END
