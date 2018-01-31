/*******************************************************************************
  MRF24WG Driver Management Set/Get Parameter Messages (Specific to the MRF24WG)

  File Name:
    drv_wifi_param_msg.c

  Summary:
    MRF24WG Driver Management Set/Get Parameter Messages (Specific to the MRF24WG)

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
#include "drv_wifi_mgmt_msg.h"

/*****************/
/*    DEFINES    */
/*****************/
#define MSG_PARAM_START_DATA_INDEX (6)
#define MULTICAST_ADDRESS (6)
#define ADDRESS_FILTER_DEACTIVATE (0)

/**************************/
/*    LOCAL DATA TYPES    */
/**************************/
typedef struct multicastFilterMsgStruct
{
    uint8_t filterId;
    uint8_t filterAction;
    uint8_t macAddress[6];
    uint8_t macBitMask;
} tMulticastFilterMsg;

/*******************************************************************************
  Function:
    void DRV_WIFI_LinkDownThresholdSet(uint8_t threshold)

  Summary:
    Sets number of consecutive WiFi Tx failures before link is considered down.

  Description:
    This function allows the application to set the number of MRF24W consecutive Tx failures
    before the connection failure event (DRV_WIFI_LINK_LOST) is reported to the host application.

  Parameters:
    threshold --  0: disabled (default)
                  1-255:  number of consecutive Tx failures before connection failure event is reported

  Returns:
    None.

  Remarks:
    None.
 *******************************************************************************/
void DRV_WIFI_LinkDownThresholdSet(uint8_t threshold)
{
    SetParamMsgSend(PARAM_LINK_DOWN_THRESHOLD, &threshold, sizeof(threshold));
}

/*******************************************************************************
  Function:
    void DRV_WIFI_TxModeSet(uint8_t mode)

  Summary:
    Configures 802.11 Tx mode.

  Description:
    This function sets the MRF24WG Tx mode.

        <table>
        Mode                            Description
        ----                            -----------
        DRV_WIFI_TXMODE_G_RATES         Use all 802.11g rates (default)
        DRV_WIFI_TXMODE_B_RATES         Use only 802.11b rates
        DRV_WIFI_TXMODE_LEGACY_RATES    Use only 1 and 2 mbps rates
        </table>

  Parameters:
    mode - Tx mode value (see description)

  Returns:
    None.
 *******************************************************************************/
void DRV_WIFI_TxModeSet(uint8_t mode)
{
    SetParamMsgSend(PARAM_TX_MODE, (uint8_t *)&mode, 1);
}

/*******************************************************************************
  Function:
    void DRV_WIFI_TxModeGet(uint8_t *p_mode);

  Summary:
    Gets 802.11 Tx mode

  Description:
    This function gets the MRF24WG Tx mode.

  Parameters:
    p_mode -- Pointer to where mode will be written.  See DRV_WIFI_TX_MODES.

  Returns:
    None.
 *******************************************************************************/
void DRV_WIFI_TxModeGet(uint8_t *p_mode)
{
    GetParamMsgSend(PARAM_TX_MODE, p_mode, 1);
}

/*******************************************************************************
  Function:
    void DRV_WIFI_DeviceInfoGet(DRV_WIFI_DEVICE_INFO *p_deviceInfo)

  Summary:
    Retrieves MRF24WG device information.

  Description:
    This function retrieves MRF24WG device information.  See DRV_WIFI_DEVICE_INFO
    structure.

  Parameters:
    p_deviceInfo - Pointer where device info will be written.

  Returns:
    None.
 *******************************************************************************/
void DRV_WIFI_DeviceInfoGet(DRV_WIFI_DEVICE_INFO *p_deviceInfo)
{
    uint8_t msgData[2];

    GetParamMsgSend(PARAM_SYSTEM_VERSION, msgData, sizeof(msgData));

    p_deviceInfo->deviceType = 0xff;
    p_deviceInfo->romVersion = msgData[0];
    p_deviceInfo->patchVersion = msgData[1];

    if (p_deviceInfo->romVersion == 0x12)
    {
        p_deviceInfo->deviceType = DRV_WIFI_MRF24WB0M_DEVICE;
    }
    else if (p_deviceInfo->romVersion == 0x30 || p_deviceInfo->romVersion == 0x31)
    {
        p_deviceInfo->deviceType = DRV_WIFI_MRF24WG0M_DEVICE; /* need part number */
    }
    else
    {
        DRV_WIFI_ASSERT(false, "");
    }
}

/*******************************************************************************
  Function:
    uint16_t DRV_WIFI_MgmtBaseGet(void)

  Summary:
    Retrieves WF Mgmt base address.

  Description:


  Precondition:
    MACInit must be called first.

  Parameters:
   None.

  Returns:
    Base address.

  Remarks:
    None.
 *******************************************************************************/
uint16_t DRV_WIFI_MgmtBaseGet(void)
{
    Write16BitWFRegister(WF_INDEX_ADDR_REG, WF_SCRATCHPAD_0_REG);
    return Read16BitWFRegister(WF_INDEX_DATA_REG);
}

/*******************************************************************************
  Function:
    void DRV_WIFI_MacAddressSet(uint8_t *p_mac)

  Summary:
    Uses a different MAC address for the MRF24W.

  Description:
    Directs the MRF24W to use the input MAC address instead of its
    factory-default MAC address.  This function does not overwrite the factory
    default, which is in FLASH memory.

  Parameters:
    p_mac - Pointer to 6-byte MAC that will be sent to MRF24WG.

  Returns:
    None.
 *******************************************************************************/
void DRV_WIFI_MacAddressSet(uint8_t *p_mac)
{
    SetParamMsgSend(PARAM_MAC_ADDRESS, p_mac, WF_MAC_ADDRESS_LENGTH);
}

/*******************************************************************************
  Function:
    void DRV_WIFI_MacAddressGet(uint8_t *p_mac)

  Summary:
    Retrieves the MRF24WG MAC address.

  Description:
    This function retrieves the MRF24WG MAC address.

  Parameters:
    p_mac - Pointer where mac address will be written (must point to a 6-byte buffer).

  Returns:
    None.
 *******************************************************************************/
void DRV_WIFI_MacAddressGet(uint8_t *p_mac)
{
    GetParamMsgSend(PARAM_MAC_ADDRESS, p_mac, WF_MAC_ADDRESS_LENGTH);
}

/*******************************************************************************
  Function:
    void DRV_WIFI_MulticastFilterSet(DRV_WIFI_MULTICAST_CONFIG *p_config);

  Summary:
    Sets a multicast address filter using one of the software multicast filters.

  Description:
    This function allows the application to configure up to two Multicast
    Address Filters on the MRF24W.  If two active multicast filters are set
    up they are OR’d together – the MRF24W will receive and pass to the Host
    CPU received packets from either multicast address.
    The allowable values in p_config are:

    filterId -- DRV_WIFI_MULTICAST_FILTER_1 thru DRV_WIFI_MULTICAST_FILTER_16

    action -- DRV_WIFI_MULTICAST_DISABLE_ALL (default)
              The Multicast Filter discards all received
              multicast messages – they will not be forwarded
              to the Host PIC.  The remaining fields in this
              structure are ignored.

              DRV_WIFI_MULTICAST_ENABLE_ALL
              The Multicast Filter forwards all received multicast messages
              to the Host PIC. The remaining fields in this structure are
              ignored.

              DRV_WIFI_MULTICAST_USE_FILTERS
              The MAC filter will be used and the remaining fields in this
              structure configure which Multicast messages are forwarded to
              the Host PIC.

    macBytes -- Array containing the MAC address to filter on (using the destination
                address of each incoming 802.11 frame).  Specific bytes with the
                MAC address can be designated as ‘don’t care’ bytes.  See macBitMask.
                This field in only used if action = DRV_WIFI_MULTICAST_USE_FILTERS.

    macBitMask -- A byte where bits 5:0 correspond to macBytes[5:0].  If the bit is
                  zero then the corresponding MAC byte must be an exact match for the
                  frame to be forwarded to the Host PIC.  If the bit is one then the
                  corresponding MAC byte is a ‘don’t care’ and not used in the
                  Multicast filtering process.  This field in only used if
                  action = DRV_WIFI_MULTICAST_USE_FILTERS.

    By default, both Multicast Filters are inactive.

    Example -- Filter on Multicast Address of 01:00:5e:xx:xx:xx where xx are don't care bytes.
                  p_config->filterId = DRV_WIFI_MULTICAST_FILTER_1

                                         [0] [1] [2] [3] [4] [5]
                  p_config->macBytes[] = 01, 00, 5e, ff, ff, ff  (0xff are the don't care bytes)

                  p_config->macBitMask = 0x38 --> bits 5:3 = 1 (don't care on bytes 3,4,5)
                                              --> bits 2:0 = 0 (exact match required on bytes 0, 1, 2)

  Precondition:
    MACInit must be called first.

  Parameters:
    p_config -- Pointer to the multicast config structure.  See documentation.

  Returns:
    None.

  Remarks:
    None.
 *******************************************************************************/
 void DRV_WIFI_MulticastFilterSet(DRV_WIFI_MULTICAST_CONFIG *p_config)
{
    tMulticastFilterMsg msg;
    uint8_t filterId;

    filterId = p_config->filterId;

    DRV_WIFI_ASSERT(p_config->action <= DRV_WIFI_MULTICAST_USE_FILTERS, "");

    /* if want no multicast messages forwarded to the host PIC */
    if (p_config->action == DRV_WIFI_MULTICAST_DISABLE_ALL)
    {
        msg.filterId = DRV_WIFI_MULTICAST_FILTER_1;
        msg.filterAction = ADDRESS_FILTER_DEACTIVATE;
        memset(msg.macAddress, 0xff, WF_MAC_ADDRESS_LENGTH);
        msg.macBitMask = 0x00; /* don't care */
    }
    /* else if want all multicast messages forwarded to the host PIC */
    else if (p_config->action == DRV_WIFI_MULTICAST_ENABLE_ALL)
    {
        msg.filterId = DRV_WIFI_MULTICAST_FILTER_1;
        msg.filterAction = MULTICAST_ADDRESS;
        memcpy((void *)msg.macAddress, (void *)p_config->macBytes, WF_MAC_ADDRESS_LENGTH);
        msg.macBitMask = 0x3f;  /* don't care from host, but MRF24WG needs to see this bitmask */
    }
    /* else if want a single multicast address or group of multicast addresses forwarded to Host PIC */
    else if (p_config->action == DRV_WIFI_MULTICAST_USE_FILTERS)
    {
        msg.filterId = filterId;
        msg.filterAction = MULTICAST_ADDRESS;
        memcpy((void *)&msg.macAddress, (void *)p_config->macBytes, WF_MAC_ADDRESS_LENGTH);
        msg.macBitMask = p_config->macBitMask;
    }
    /* invalid action */
    else
    {
        DRV_WIFI_ASSERT(false, "");
    }

    SetParamMsgSend(PARAM_COMPARE_ADDRESS, (uint8_t *)&msg, sizeof(msg));
}

/*******************************************************************************
  Function:
    void DRV_WIFI_RegionalDomainGet(uint8_t *p_regionalDomain)

  Summary:
    Retrieves the MRF24WG Regional domain.

  Description:
    Gets the regional domain on the MRF24W.  Values are:
    * DRV_WIFI_DOMAIN_FCC
    * DRV_WIFI_DOMAIN_ETSI
    * DRV_WIFI_DOMAIN_JAPAN
    * DRV_WIFI_DOMAIN_OTHER

  Parameters:
    p_regionalDomain - Pointer where the regional domain value will be written.

  Returns:
    None.
 *******************************************************************************/
void DRV_WIFI_RegionalDomainGet(uint8_t *p_regionalDomain)
{
    GetParamMsgSend(PARAM_REGIONAL_DOMAIN, p_regionalDomain, 1);
}

/*******************************************************************************
  Function:
    void DRV_WIFI_RtsThresholdSet(uint16_t rtsThreshold)

  Summary:
    Sets the RTS Threshold.

  Description:
    Sets the RTS/CTS packet size threshold for when RTS/CTS frame will be sent.
    The default is 2347 bytes – the maximum for 802.11.  It is recommended that
    the user leave the default at 2347 until they understand the performance and
    power ramifications of setting it smaller.  Valid values are from 0 to
    DRV_WIFI_RTS_THRESHOLD_MAX (2347).

  Parameters:
    rtsThreshold - Value of the packet size threshold.

  Returns:
    None.

  Remarks:
    None.
 *******************************************************************************/
void DRV_WIFI_RtsThresholdSet(uint16_t rtsThreshold)
{
    DRV_WIFI_ASSERT(rtsThreshold <= DRV_WIFI_RTS_THRESHOLD_MAX, "");

    /* correct endianness before sending message */
    rtsThreshold = TCPIP_Helper_htons(rtsThreshold);

    SetParamMsgSend(PARAM_RTS_THRESHOLD, (uint8_t *)&rtsThreshold, sizeof(rtsThreshold));
}

/*******************************************************************************
  Function:
    void DRV_WIFI_RtsThresholdGet(uint16_t *p_rtsThreshold)

  Summary:
    Gets the RTS Threshold.

  Description:
    Gets the RTS/CTS packet size threshold.

  Parameters:
    p_rtsThreshold - Pointer to where RTS threshold is written.

  Returns:
    None.

  Remarks:
    None.
 *******************************************************************************/
void DRV_WIFI_RtsThresholdGet(uint16_t *p_rtsThreshold)
{
    GetParamMsgSend(PARAM_RTS_THRESHOLD, (uint8_t *)p_rtsThreshold, sizeof(uint16_t));

    /* correct endianness before sending message */
    *p_rtsThreshold = TCPIP_Helper_htons(*p_rtsThreshold);
}

/*******************************************************************************
  Function:
    void DRV_WIFI_MultiCastFilterEnable(void)

  Summary:
    Forces the MRF24WG to use software multicast filters instead of
    hardware multicast filters.
    <p><b>Implementation:</b> Dynamic</p>

  Description:
    This function allows the application to configure up to 16 software
    multicast address Filters on the MRF24WG0MA/B.

  Precondition:
    Wi-Fi initialization must be complete.

  Returns:
    None.

  Example:
    <code>
      DRV_WIFI_MultiCastFilterEnable();
    </code>

  Remarks:
    Cannot mix hardware and software multicast filters.
 *******************************************************************************/
void DRV_WIFI_MultiCastFilterEnable(void)
{
    uint8_t enable = 1;

    SetParamMsgSend(PARAM_SET_MULTICAST_FILTER, (uint8_t *)&enable, sizeof(enable));
}

/*******************************************************************************
  Function:
    void DRV_WIFI_MacStatsGet(DRV_WIFI_MAC_STATS *p_macStats)

  Summary:
    Gets MAC statistics.

  Description:
    This function gets the various MAC layer stats as maintained by the MRF24WG.

  Parameters:
    p_macStats - Pointer to where MAC statistics are written.  See DRV_WIFI_MAC_STATS
                 structure.

  Returns:
    None.
 *******************************************************************************/
void DRV_WIFI_MacStatsGet(DRV_WIFI_MAC_STATS *p_macStats)
{
    uint32_t *p_value;
    uint8_t numElements;
    uint8_t i;

    GetParamMsgSend(PARAM_STAT_COUNTERS, (uint8_t *)p_macStats, sizeof(DRV_WIFI_MAC_STATS));

    /* calculate number of 32-bit counters in the stats structure and point to first element */
    numElements = sizeof(DRV_WIFI_MAC_STATS) / sizeof(uint32_t);
    p_value = (uint32_t *)p_macStats;

    /* correct endianness on all counters in structure */
    for (i = 0; i < numElements; ++i)
    {
        *p_value = TCPIP_Helper_ntohl(*p_value);
        ++p_value;
    }
}

/*******************************************************************************
  Function:
    void DRV_WIFI_HostKeyDeriveModeSet(void);

  Summary:
    Has host get WPA-PSK passphrase and derive the key.

  Description:
    This function has host MCU derive the binary key from the passphrase which is received 
    from AP in WPS process. 

  Parameters:
    None.

  Returns:
    None.
 *******************************************************************************/
void DRV_WIFI_HostKeyDeriveModeSet(void)
{
    uint8_t yield = 1;

    SetParamMsgSend(PARAM_SET_HOST_DERIVE_KEY, (uint8_t *)&yield, sizeof(yield));
}

/*******************************************************************************
  Function:
    void DRV_WIFI_SetPSK(uint8_t *p_psk)

  Summary:
    Sets the binary WPA PSK code in WPS.

  Description:
    This function sets the binary WPA PSK code in WPS.

  Parameters:
    p_psk - pointer to the binary key

  Returns:
    None.
 *******************************************************************************/
void DRV_WIFI_SetPSK(uint8_t *p_psk)
{
    SetParamMsgSend(PARAM_SET_PSK, (uint8_t *)p_psk, 32);
}

/*******************************************************************************
  Function:
    void SetParamMsgSend(uint8_t paramType,
                         uint8_t *p_paramData,
                         uint8_t paramDataLength)

  Summary:
    Sends a SetParam Mgmt request to MRF24W and waits for response.

  Description:
    Index Set Param Request
    ----- -----------------
    0     type            (always 0x02 signifying a mgmt request)
    1     subtype         (always 0x10 signifying a Set Param Msg)
    2     param ID [msb]  (MS byte of parameter ID being requested, e.g.
                           PARAM_SYSTEM_VERSION)
    3     param ID [lsb]  (LS byte of parameter ID being requested. e.g.
                           PARAM_SYSTEM_VERSION)
    4     payload[0]      first byte of param data
    N     payload[n]      Nth byte of payload data

    Index  Set Param Response
    ------ ------------------
    0      type           (always 0x02 signifying a mgmt response)
    1      subtype        (always 0x10 signifying a Param Response Msg
    2      result         (1 if successful -- any other value indicates failure
    3      mac state      (not used)

  Precondition:
    MACInit must be called first.

  Parameters:
    paramType - Parameter type associated with the SetParam msg.
    p_paramData - Pointer to parameter data.
    paramDataLength - Number of bytes pointed to by p_paramData.

  Returns:
    None.

  Remarks:
    None.
 *******************************************************************************/
void SetParamMsgSend(uint8_t paramType,
                     uint8_t *p_paramData,
                     uint8_t paramDataLength)
{
    uint8_t hdr[4];

    hdr[0] = WF_MGMT_REQUEST_TYPE;
    hdr[1] = WF_SET_PARAM_SUBTYPE;
    hdr[2] = 0x00;      /* MS 8 bits of param ID, always 0 */
    hdr[3] = paramType; /* LS 8 bits of param ID */

    SendMgmtMsg(hdr, sizeof(hdr), p_paramData, paramDataLength); 

    /* wait for MRF24W management response; free response because not needed */
    WaitForMgmtResponse(WF_SET_PARAM_SUBTYPE, FREE_MGMT_BUFFER);
}

/*******************************************************************************
  Function:
    void GetParamMsgSend(uint8_t paramType,
                         uint8_t *p_paramData,
                         uint8_t paramDataLength)

  Summary:
    Sends a GetParam Mgmt request to MRF24W and waits for response.

  Description:
    After response is received the param data is read from message and written
    to p_paramData.  It is up to the caller to fix up endianness.

    Index Get Param Request
    ----- -----------------
    0     type            (always 0x02 signifying a mgmt request)
    1     subtype         (always 0x10 signifying a Get Param Msg)
    2     param ID [msb]  (MS byte of parameter ID being requested, e.g.
                           PARAM_SYSTEM_VERSION)
    3     param ID [lsb]  (LS byte of parameter ID being requested, e.g.
                           PARAM_SYSTEM_VERSION)

    Index  Get Param Response
    ------ ------------------
    0      type           (always 0x02 signifying a mgmt response)
    1      subtype        (always 0x10 signifying a Param Response Msg
    2      result         (1 if successful -- any other value indicates failure
    3      mac state      (not used)
    4      data length    Length of response data starting at index 6 (in bytes)
    5      not used
    6      Data[0]        first byte of returned parameter data
    N      Data[N]        Nth byte of param data

  Precondition:
    MACInit must be called first.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
 *******************************************************************************/
void GetParamMsgSend(uint8_t paramType,
                     uint8_t *p_paramData,
                     uint8_t paramDataLength)
{
    uint8_t hdr[4];

    hdr[0] = WF_MGMT_REQUEST_TYPE;
    hdr[1] = WF_GET_PARAM_SUBTYPE;
    hdr[2] = 0x00;      /* MS 8 bits of param ID, always 0 */
    hdr[3] = paramType; /* LS 8 bits of param ID           */

    SendMgmtMsg(hdr, sizeof(hdr), NULL, 0);             

    WaitForMgmtResponseAndReadData(WF_GET_PARAM_SUBTYPE, paramDataLength,            
        MSG_PARAM_START_DATA_INDEX, p_paramData);

}

//DOM-IGNORE-END
