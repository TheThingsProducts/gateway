/*******************************************************************************
  MRF24WG Driver Scan Functions

  File Name:
    drv_wifi_scan.c

  Summary:
    MRF24WG Driver Scan Functions

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

#include "drv_wifi_priv.h"

#include "drv_wifi_config_data.h"
#include "drv_wifi_debug_output.h"
#include "drv_wifi_mgmt_msg.h"
#include "drv_wifi_scan_helper.h"

static bool isScanAllowed(void)
{
    uint8_t connectionState;

    DRV_WIFI_ConnectionStateGet(&connectionState);
    switch (DRV_WIFI_CONFIG_PARAMS(networkType)) {
        /*
         * In Infrastructure or Ad-Hoc mode, MRF24W can scan in either idle or
         * connected state.
         */
        case DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE:
        case DRV_WIFI_NETWORK_TYPE_ADHOC:
            if (connectionState == DRV_WIFI_CSTATE_CONNECTION_IN_PROGRESS ||
                    connectionState == DRV_WIFI_CSTATE_RECONNECTION_IN_PROGRESS)
                return false;
            break;
        /*
         * In Soft AP mode, MRF24W can only scan in idle state. However, when
         * connected, the return code of MRF24W is DRV_WIFI_CSTATE_CONNECTION_IN_PROGRESS.
         */
        case DRV_WIFI_NETWORK_TYPE_SOFT_AP:
            if (connectionState == DRV_WIFI_CSTATE_CONNECTION_IN_PROGRESS)
                return false;
            break;
        default:
            DRV_WIFI_ASSERT(false, "Invalid Wi-Fi network type");
            break;
    }
    return true;
}

/*******************************************************************************
  Function:
    uint16_t DRV_WIFI_Scan(bool scanAll)

  Summary:
    Commands the MRF24W to start a scan operation.  This will generate the
    WF_EVENT_SCAN_RESULTS_READY event.

  Description:
    Directs the MRF24W to initiate a scan operation.  The Host Application will
    be notified that the scan results are ready when it receives the
    WF_EVENT_SCAN_RESULTS_READY event.  The eventInfo field for this event will
    contain the number of scan results.  Once the scan results are ready they
    can be retrieved with DRV_WIFI_ScanResultGet().

    Scan results are retained on the MRF24W until:
    1. Calling DRV_WIFI_Scan() again (after scan results returned from previous
         call).
    2. MRF24W reset.

  Parameters:
    scanAll -
            If false:
            * If SSID defined then only scan results with that SSID are retained.
            * If SSID not defined then all scanned SSID’s will be retained
            * Only scan results from Infrastructure or AdHoc networks are retained
            * The channel list that is scanned will be determined from the channels
              passed in via DRV_WIFI_ChannelListSet().

            If true:
            * All scan results are retained (both Infrastructure and Ad Hoc
               networks).
            * All channels within the MRF24W’s regional domain will be
               scanned.

  Returns:
    None.

  Remarks:
    In Infrastructure or Ad-Hoc mode, host scan can only work in the idle or
    connected state.
    In Soft AP mode, host scan can only work in the idle state.
    To use this API safely, we recommend to disable MRF24W FW connection manager
    by setting DRV_WIFI_MODULE_CONNECTION_MANAGER == DRV_WIFI_DISABLED in system_config.h.
 *******************************************************************************/
uint16_t DRV_WIFI_Scan(bool scanAll)
{
    uint8_t hdr[4];

    if (!isScanAllowed())
        return DRV_WIFI_ERROR_OPERATION_CANCELLED;

    hdr[0] = WF_MGMT_REQUEST_TYPE;
    hdr[1] = WF_SCAN_START_SUBTYPE;
    hdr[2] = (scanAll == true) ? 0xff : CPID;
    hdr[3] = 0; /* not used */

    SendMgmtMsg(hdr, sizeof(hdr), NULL, 0);

    /* wait for mgmt response, free it after it comes in (no data needed) */
    WaitForMgmtResponse(WF_SCAN_START_SUBTYPE, FREE_MGMT_BUFFER);
    return DRV_WIFI_SUCCESS;
}

/*******************************************************************************
  Function:
    void DRV_WIFI_ScanResultGet(uint8_t listIndex, t_wfScanResult *p_scanResult)

  Summary:
    Read single selected scan result back from MRF24W.

  Description:
    After a scan has completed this function is used to read one scan result at
    a time from the MRF24WG.

  Parameters:
    listIndex - Index (0-based list) of the scan entry to retrieve.
    p_scanResult - Pointer to where scan result is written.  See DRV_WIFI_SCAN_RESULT
                   structure.

  Returns:
    None.
 *******************************************************************************/
void DRV_WIFI_ScanResultGet(uint8_t listIndex, DRV_WIFI_SCAN_RESULT *p_scanResult)
{
    uint8_t hdr[4];

    hdr[0] = WF_MGMT_REQUEST_TYPE;
    hdr[1] = WF_SCAN_GET_RESULTS_SUBTYPE;
    hdr[2] = listIndex; /* scan result index to read from */
    hdr[3] = 1;         /* number of results to read */

    SendMgmtMsg(hdr, sizeof(hdr), NULL, 0);

    /* index 4 contains number of scan results returned, index 5 is first byte of first scan result */
    WaitForMgmtResponseAndReadData(WF_SCAN_GET_RESULTS_SUBTYPE, sizeof(DRV_WIFI_SCAN_RESULT),
        5, (uint8_t *)p_scanResult);

    /* fix up endianness on the two 16-bit values in the scan results */
    p_scanResult->beaconPeriod = TCPIP_Helper_ntohs(p_scanResult->beaconPeriod);
    p_scanResult->atimWindow = TCPIP_Helper_ntohs(p_scanResult->atimWindow);
}

//DOM-IGNORE-END
