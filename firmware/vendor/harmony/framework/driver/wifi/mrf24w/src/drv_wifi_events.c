/*******************************************************************************
  MRF24WG Wi-Fi Driver MAC Events

  File Name:
    drv_wifi_events.c

  Summary:
    MRF24WG Wi-Fi Driver MAC Events

  Description:
    MRF24WG Wi-Fi Driver MAC events implementation. Processes Wi-Fi events
    via callback functions.
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

#include "drv_wifi_mac.h"

// stack internal notification
typedef struct
{
    bool                     mrfEnabledEvents; // group enabled notification events
    volatile TCPIP_MAC_EVENT mrfPendingEvents; // group notification events that are set, waiting to be re-acknowledged
    TCPIP_MAC_EventF         mrfNotifyFnc;     // group notification handler
    const void*              mrfNotifyParam;   // notification parameter
} MRF24W_EV_GROUP_DCPT; // event descriptor

static MRF24W_USR_EV_DCPT s_mrfUserEvent; // stack user events
static MRF24W_EV_GROUP_DCPT s_mrfGroupDcpt;

/*******************************************************************************
  Function:
    TCPIP_MAC_RES MRF24W_MACEventInit(MRF24W_MAC_DCPT  *pDcpt,
                                      TCPIP_MAC_EventF eventF,
                                      const void       *eventParam,
                                      int              intPri,
                                      int              intSubPri)

  Summary:
    Initializes the ethernet event notification.

  Description:
    This function initializes the ethernet event notification.
    It performs any resource allocation that may be needed.

  Precondition:
    None.

  Parameters:
    hMac - parameter identifying the intended MAC
    intPri - priority of the TCPIP interrupt events
    intSubPri - sub-priority of the TCPIP interrupt events

  Returns:
    TCPIP_MAC_RES_OK if initialization succeeded,
    an error code otherwise.

  Example:
    None.

  Remarks:
    Not multi-threaded safe.
 *******************************************************************************/
TCPIP_MAC_RES MRF24W_MACEventInit(MRF24W_MAC_DCPT  *pDcpt,
                                  TCPIP_MAC_EventF eventF,
                                  const void       *eventParam,
                                  int              intPri,
                                  int              intSubPri)
{
    pDcpt = pDcpt; // avoid compiler warning
    intPri = intPri; // avoid compiler warning
    intSubPri = intSubPri; // avoid compiler warning

    s_mrfGroupDcpt.mrfEnabledEvents = false;
    s_mrfGroupDcpt.mrfPendingEvents = 0;
    s_mrfGroupDcpt.mrfNotifyFnc = eventF; // set new handler
    s_mrfGroupDcpt.mrfNotifyParam = eventParam;

    s_mrfUserEvent.trafficEvents = s_mrfUserEvent.mgmtEvents = 0;
    s_mrfUserEvent.trafficEventInfo = s_mrfUserEvent.mgmtEventInfo = 0;

    return TCPIP_MAC_RES_OK;
}

/*******************************************************************************
  Function:
    TCPIP_MAC_RES MRF24W_MACEventDeInit(MRF24W_MAC_DCPT *pDcpt)

  Summary:
    De-initializes the Ethernet event notification.

  Description:
    This function de-initializes the Ethernet event notification.
    It performs any resource clean-up that may be needed.

  Precondition:
    None.

  Parameters:
    hMac - parameter identifying the intended MAC

  Returns:
    TCPIP_MAC_RES_OK always.

  Example:
    <code>
    MRF24W_MACEventDeInit( hMac );
    </code>

  Remarks:
    Not multi-threaded safe.
 *******************************************************************************/
TCPIP_MAC_RES MRF24W_MACEventDeInit(MRF24W_MAC_DCPT *pDcpt)
{
    SYS_INT_SourceDisable(MRF_INT_SOURCE); // stop MRF ints
    SYS_INT_SourceStatusClear(MRF_INT_SOURCE);

    s_mrfGroupDcpt.mrfNotifyFnc = 0;
    s_mrfGroupDcpt.mrfEnabledEvents = false;
    s_mrfGroupDcpt.mrfPendingEvents = 0;

    return TCPIP_MAC_RES_OK;
}

/*******************************************************************************
  Function:
    bool _MRF24W_MACEventSetMask(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents, bool enable)

  Summary:
    Adds new events to the list of the enabled ones.

  Description:
    This function sets new enabled events.
    Multiple events can be or-ed together.
    All events that are set will be added to the notification process. The other events will not be touched.
    The stack (or stack user) has to catch the events that are notified and process them:
      - The stack should process the TCPIP_MAC_EV_RX_DONE, TCPIP_MAC_EV_TX_DONE transfer events.
      - Process the specific condition and acknowledge them calling _MRF24W_MACEventAcknowledge() so that they can be re-enabled.

  Precondition:
    TCPIP_STACK_Init() should have been called.

  Parameters:
    hMac - parameter identifying the intended MAC
    tcpipEvents - events the user of the stack wants to add for notification
    enable - boolean to enable/disable the event notification

  Returns:
    True if operation succeeded,
    false code otherwise.

  Example:
    <code>
    _MRF24W_MACEventSetMask( hMac, TCPIP_MAC_EV_RX_OVFLOW | TCPIP_MAC_EV_RX_BUFNA, true );
    </code>

  Remarks:
    Globally enable/disable all notifications for now.
 *******************************************************************************/
bool _MRF24W_MACEventSetMask(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents, bool enable)
{
    if (enable)
    {
        s_mrfGroupDcpt.mrfEnabledEvents = true;
        SYS_INT_SourceEnable(MRF_INT_SOURCE); // start MRF ints
    }
    else
    {
        SYS_INT_SourceDisable(MRF_INT_SOURCE); // stop MRF ints
        s_mrfGroupDcpt.mrfEnabledEvents = false;
    }

    return true;
}

/*******************************************************************************
  Function:
    bool _MRF24W_MACEventAcknowledge(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents)

  Summary:
    Acknowledges and re-enables processed events.

  Description:
    This function acknowledges and re-enables processed events.
    Multiple events can be or-ed together as they are processed together.
    The events acknowledged by this function should be the events that have been retrieved from the stack
    by calling _MRF24W_MACEventGetPending() or have been passed to the user by the stack using the notification handler
    and have been processed and have to be re-enabled.

  Precondition:
    TCPIP_STACK_Init() should have been called.

  Parameters:
    hMac - parameter identifying the intended MAC
    tcpipEvents - the events that the user processed and need to be re-enabled

  Returns:
    True if events acknowledged,
    false if no events to be acknowledged,
    an error code otherwise.

  Example:
    <code>
    _MRF24W_MACEventAcknowledge( hMac, stackNewEvents );
    </code>

  Remarks:
    All events should be acknowledged, in order to be re-enabled.
    For now, the re-enabling is done internally by the MRF processing events code.
 *******************************************************************************/
bool _MRF24W_MACEventAcknowledge(TCPIP_MAC_HANDLE hMac, TCPIP_MAC_EVENT macEvents)
{
    if (s_mrfGroupDcpt.mrfPendingEvents)
    {
        s_mrfGroupDcpt.mrfPendingEvents = 0;
        return true;
    }
    else
    {
        return false;
    }
}

/*******************************************************************************
  Function:
    TCPIP_MAC_EVENT _MRF24W_MACEventGetPending(TCPIP_MAC_HANDLE hMac)

  Summary:
    Returns the currently pending events.

  Description:
    This function returns the currently pending events.
    Multiple events can be or-ed together as they accumulate.
    The stack should be called for processing whenever a stack managed event (TCPIP_MAC_EV_RX_DONE, TCPIP_MAC_EV_TX_DONE) is present.
    The other, non critical events, may not be managed by the stack and passed to an user.
    They will have to be eventually acknowledged if re-enabling is needed.

  Precondition:
    TCPIP_STACK_Init() should have been called.

  Parameters:
    hMac - parameter identifying the intended MAC

  Returns:
    The currently stack pending events.

  Example:
    <code>
    TCPIP_MAC_EVENT currEvents = _MRF24W_MACEventGetPending( hMac);
    </code>

  Remarks:
    This is the preferred method to get the current pending MAC events.
    The stack maintains a proper image of the events from their occurrence to their acknowledgement.

    Even with a notification handler in place it's better to use this function to get the current pending events
    rather than using the events passed by the notification handler which could be stale.

    The events are persistent. They shouldn't be re-enabled unless they have been processed and
    the condition that generated them was removed.
    Re-enabling them immediately without proper processing will have dramatic effects on system performance.

    The returned value is just a momentary value. The pending events can change any time.
 *******************************************************************************/
TCPIP_MAC_EVENT _MRF24W_MACEventGetPending(TCPIP_MAC_HANDLE hMac)
{
    return s_mrfGroupDcpt.mrfPendingEvents;
}

/*******************************************************************************
  Function:
    void DRV_WIFI_UserEventSet(uint16_t event, uint16_t eventInfo, bool isMgmt)

  Summary:
    Sets the current events from the MRF24WG.

  Description:
    This function sets the current event(s) from the MRF24WG.

  Parameters:
    event -- current MRF24W traffic event
    eventInfo -- additional event info
                 This is not applicable to all events.
    isMgmt -- specifies traffic or management event

  Returns:
    None.
 *******************************************************************************/
void DRV_WIFI_UserEventSet(uint16_t event, uint16_t eventInfo, bool isMgmt)
{
    if (isMgmt) {
        // TODO: handle if (s_mrfUserEvent.mgmtEvents != 0)
        s_mrfUserEvent.mgmtEvents = event;
        s_mrfGroupDcpt.mrfPendingEvents = event; // add this line so event function sees event?
        s_mrfUserEvent.mgmtEventInfo = eventInfo;
    } else {
        s_mrfUserEvent.trafficEvents = event;
        s_mrfGroupDcpt.mrfPendingEvents = event; // add this line so event function sees event?
        s_mrfUserEvent.trafficEventInfo = eventInfo;
    }

    // let user know of event
    if (s_mrfGroupDcpt.mrfNotifyFnc) {
        (*s_mrfGroupDcpt.mrfNotifyFnc)(s_mrfGroupDcpt.mrfPendingEvents, s_mrfGroupDcpt.mrfNotifyParam);
    }
}

/*******************************************************************************
  Function:
    uint16_t TrafficEventsGet(uint16_t *pEventInfo)

  Summary:
    Returns the current traffic events from the MRF24WG.

  Description:
    This function returns the current traffic events from the MRF24WG.

  Precondition:
    TCPIP stack should be initialized.

  Parameters:
    pEventInfo -- address to store additional information about the traffic event
                  This is not applicable to all events.

  Returns:
    The traffic event that occurred.

  Remarks:
    None.
 *******************************************************************************/
uint16_t TrafficEventsGet(uint16_t *pEventInfo)
{
    uint16_t res = s_mrfUserEvent.trafficEvents;

    if (pEventInfo)
    {
        *pEventInfo = s_mrfUserEvent.trafficEventInfo;
    }

    s_mrfUserEvent.trafficEvents = 0;
    s_mrfUserEvent.trafficEventInfo = 0;

    return res;
}

/*******************************************************************************
  Function:
    uint16_t MgmtEventsGet(MRF24W_USR_EV_DCPT *pEventInfo)

  Summary:
    Returns the current management events from the MRF24W.

  Description:
    None.

  Precondition:
    TCPIP stack should be initialized.

  Parameters:
    pEventInfo -- address to store additional information about the management event
                  This is not applicable to all events.

  Returns:
    The management event that occurred, or 0 if no mgmt event has occurred

  Remarks:
    None.
 *******************************************************************************/
uint16_t MgmtEventsGet(MRF24W_USR_EV_DCPT *pEventInfo)
{
    uint16_t res;

    // if mgmt event has occurred
    if (s_mrfUserEvent.mgmtEvents > 0)
    {
        // copy info to caller structure
        pEventInfo->mgmtEvents = s_mrfUserEvent.mgmtEvents;
        pEventInfo->mgmtEventInfo = s_mrfUserEvent.mgmtEventInfo;
        res = pEventInfo->mgmtEvents;
    }
    else
    {
        res = 0; // signals no mgmt event has occurred
    }

    // clear global structure for next mgmt event
    s_mrfUserEvent.mgmtEvents = 0;
    s_mrfUserEvent.mgmtEventInfo = 0;

    return res;
}

/*******************************************************************************
 * Function: void DRV_WIFI_MRF24W_ISR(SYS_MODULE_OBJ index)
 *
 * PreCondition: TCPIP_STACK_Init(), _MRF24W_MACEventSetMask() should have been called.
 *
 * Input: p - unused
 *
 * Output: None.
 *
 * Side Effects: None.
 *
 * Overview: This function processes the MRF24WG interrupts and reports the events back to the user.
 *
 * Note: None.
 *******************************************************************************/
void DRV_WIFI_MRF24W_ISR(SYS_MODULE_OBJ index)
{
    if (s_mrfGroupDcpt.mrfEnabledEvents ) {
        s_mrfGroupDcpt.mrfPendingEvents = TCPIP_MAC_EV_TX_DONE;
        if (s_mrfGroupDcpt.mrfNotifyFnc)
            (*s_mrfGroupDcpt.mrfNotifyFnc)(s_mrfGroupDcpt.mrfPendingEvents, s_mrfGroupDcpt.mrfNotifyParam);
    }

    /* invoke MRF handler */
    DRV_WIFI_INT_Handle();
}

#if defined(DRV_WIFI_USE_FREERTOS)
void DRV_WIFI_Deferred_ISR(void *p_arg)
{
    while (1) {
        DRV_WIFI_SemTake(&g_drv_wifi_priv.deferredISRSync, OSAL_WAIT_FOREVER);
        DRV_WIFI_MRF24W_ISR((SYS_MODULE_OBJ)0);
    }
}
#endif /* defined(DRV_WIFI_USE_FREERTOS) */

//DOM-IGNORE-END
