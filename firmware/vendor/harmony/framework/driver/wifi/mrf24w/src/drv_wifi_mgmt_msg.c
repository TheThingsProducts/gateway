/*******************************************************************************
  MRF24WG Driver Management Messages (Specific to the MRF24WG)

  File Name:
    drv_wifi_mgmt_msg.c

  Summary:
    MRF24WG Driver Management Messages (Specific to the MRF24WG)

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

#include "drv_wifi_debug_output.h"
#include "drv_wifi_mgmt_msg.h"
#include "drv_wifi_raw.h"

static volatile bool s_mgmtRespMsgReceived = false;

/*****************************************************************************
 * FUNCTION: void MgmtRespReceivedSet(void)
 *
 * RETURNS: None.
 *
 * PARAMS: None.
 *
 *  NOTES: Called by MgmtRxProcess() when a mgmt response msg has been received.
 *         This function then sets a local flag for this module indicating
 *         the event.
 *****************************************************************************/
void MgmtRespReceivedSet(void)
{
    s_mgmtRespMsgReceived = true;
#if defined(DRV_WIFI_USE_FREERTOS)    
    DRV_WIFI_SemGive(&g_drv_wifi_priv.mgmtRespSync);
#endif
}

void SendMgmtMsg(uint8_t *p_header,
                 uint8_t headerLength,
                 uint8_t *p_data,
                 uint8_t dataLength)
{
    if (DRV_WIFI_InHibernateMode())
    {
        DRV_WIFI_UserEventSet(DRV_WIFI_EVENT_ERROR, DRV_WIFI_ERROR_IN_HIBERNATE_MODE, true);
        return;
    }

    WF_MGMTMSG_MUTEX_LOCK();

    EnsureWFisAwake();

    /* write out management header */
    RawSetIndex(RAW_SCRATCH_ID, MGMT_TX_BASE);
    RawSetByte(RAW_SCRATCH_ID, p_header, headerLength);

    /* write out data (if any) */
    if (dataLength > 0)
    {
        RawSetByte(RAW_SCRATCH_ID, p_data, dataLength);
    }

    /* Signal MRF24WG that mgmt message ready to process */
    Write16BitWFRegister(WF_HOST_MAIL_BOX_0_MSW_REG, 0x0400);
    Write16BitWFRegister(WF_HOST_MAIL_BOX_0_LSW_REG, 0x0000);
}

static void _WaitForMgmtResponse(void)
{
#if defined(DRV_WIFI_USE_FREERTOS)
    /* Wait for Semaphore */
    DRV_WIFI_SemTake(&g_drv_wifi_priv.mgmtRespSync, OSAL_WAIT_FOREVER);
#else /* !defined(DRV_WIFI_USE_FREERTOS) */
    /* Wait until mgmt response is received */
    while (s_mgmtRespMsgReceived == false)
        ;
#endif /* defined(DRV_WIFI_USE_FREERTOS) */

    /* set this back to false so the next mgmt send won't think he has a response before one is received */
    s_mgmtRespMsgReceived = false;
}

void WaitForMgmtResponse(uint8_t expectedSubtype, uint8_t freeAction)
{
    tMgmtMsgRxHdr hdr;

    if (DRV_WIFI_InHibernateMode())
        return;

    _WaitForMgmtResponse();

    /* if the caller wants to delete the response immediately (doesn't need any data from it */
    if (freeAction == FREE_MGMT_BUFFER)
    {
        /* read and verify result before freeing up buffer to ensure our message send was successful */
        RawRead(RAW_SCRATCH_ID, MGMT_RX_BASE, (uint16_t)(sizeof(tMgmtMsgRxHdr)), (uint8_t *)&hdr);

        /* Mgmt response 'result' field should always indicate success.  If this assert is hit the error codes are located */
        /* drv_wifi.h.  Search for DRV_WIFI_SUCCESS for the list of error codes. */

        /* mgmt response subtype had better match subtype we were expecting */
        DRV_WIFI_ASSERT(hdr.subtype == expectedSubtype, "");

        if (hdr.result != DRV_WIFI_SUCCESS)
        {
            DRV_WIFI_UserEventSet(DRV_WIFI_EVENT_ERROR, hdr.result, true);
        }
    }
    WF_MGMTMSG_MUTEX_UNLOCK();
}

void WaitForMgmtResponseAndReadData(uint8_t expectedSubtype,
                                    uint8_t numDataBytes,
                                    uint8_t startIndex,
                                    uint8_t *p_data)
{
    tMgmtMsgRxHdr hdr; /* management msg header struct */

    if (DRV_WIFI_InHibernateMode())
        return;

    _WaitForMgmtResponse();

    /* if made it here then received a management message */
    RawRead(RAW_SCRATCH_ID, MGMT_RX_BASE, (uint16_t)(sizeof(tMgmtMsgRxHdr)), (uint8_t *)&hdr);

    // debug, remove later
    DRV_WIFI_ASSERT(hdr.subtype == expectedSubtype, "");

    /* check header result and subtype fields */
    DRV_WIFI_ASSERT(hdr.result == DRV_WIFI_SUCCESS && hdr.subtype == expectedSubtype, "");

    /* if caller wants to read data from this mgmt response */
    if (numDataBytes > 0)
    {
        RawRead(RAW_SCRATCH_ID, MGMT_RX_BASE + startIndex, numDataBytes, p_data);
    }
    WF_MGMTMSG_MUTEX_UNLOCK();
}

//DOM-IGNORE-END
