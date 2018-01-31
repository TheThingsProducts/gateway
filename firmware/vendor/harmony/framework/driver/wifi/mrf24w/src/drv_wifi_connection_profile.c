/*******************************************************************************
  MRF24WG Driver Connection Profile

  File Name:
    drv_wifi_connection_profile.c

  Summary:
    MRF24WG Driver Connection Profile

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

#include "drv_wifi_config_data.h"
#include "drv_wifi_debug_output.h"
#include "drv_wifi_mgmt_msg.h"
#include "drv_wifi_raw.h"

/***************/
/*    TYPES    */
/***************/
/* header format for response to CP Get Element message */
typedef struct
{
    tMgmtMsgRxHdr mgmtHdr; /* normal 4-byte hdr for all mgmt responses */
    uint8_t profileId; // connection profile ID
    uint8_t elementId;
    uint8_t elementDataLength;
    /* element data follows */
} CPELEMENT_RESPONSE_HDR;

/***********************************/
/*    LOCAL FUNCTION PROTOTYPES    */
/***********************************/
static void SetSecurity(uint8_t securityType,
                        uint8_t *p_securityKey,
                        uint8_t securityKeyLength);

static void CPElementSet(uint8_t elementId,
                         uint8_t *p_elementData,
                         uint8_t elementDataLength);

static void CPElementGet(uint8_t elementId,
                         uint8_t *p_elementData,
                         uint8_t elementDataLength,
                         uint8_t dataReadAction);

// creates a connection profile on MRF24WG that will be used forever
void ConnectionProfileCreate(void)
{
    uint8_t  hdr[2];

    hdr[0] = WF_MGMT_REQUEST_TYPE;
    hdr[1] = WF_CP_CREATE_PROFILE_SUBTYPE;

    SendMgmtMsg(hdr, sizeof(hdr), NULL, 0);             

    /* wait for MRF24W management response, read data, free response after read */
    WaitForMgmtResponse(WF_CP_CREATE_PROFILE_SUBTYPE, FREE_MGMT_BUFFER);
}

// *****************************************************************************
/* Function:
    void DRV_WIFI_SsidSet(uint8_t *p_ssid,  uint8_t ssidLen)

  Summary:
    Sets the SSID.

  Description:
    Sets the SSID and SSID Length.  Note that an Access Point can have either a
    visible or hidden SSID.  If an Access Point uses a hidden SSID then an
    active scan must be used.

  Parameters:
    p_ssid - pointer to SSID buffer
    ssidLength - number of bytes in SSID

  Returns:
    None.
 */
void DRV_WIFI_SsidSet(uint8_t *p_ssid,  uint8_t ssidLen)
{
    DRV_WIFI_ASSERT(ssidLen <= DRV_WIFI_MAX_SSID_LENGTH, "SSID is too long");
    CPElementSet(WF_CP_ELEMENT_SSID, /* Element ID */
                 (uint8_t *)p_ssid, /* pointer to element data */
                 ssidLen); /* number of element data bytes */
    memcpy(DRV_WIFI_CONFIG_PARAMS(ssid), p_ssid, ssidLen);
    DRV_WIFI_CONFIG_PARAMS(ssidLen) = ssidLen;
    // no need to do the following if (DRV_WIFI_CONFIG_PARAMS(ssidLen == DRV_WIFI_MAX_SSID_LENGTH)
    if (DRV_WIFI_CONFIG_PARAMS(ssidLen) < DRV_WIFI_MAX_SSID_LENGTH)
        DRV_WIFI_CONFIG_PARAMS(ssid)[ssidLen] = 0x00;
}

// *****************************************************************************
/* Function:
    void DRV_WIFI_SsidGet(uint8_t *p_ssid, uint8_t *p_ssidLength)

  Summary:
    Gets the SSID.

  Description:
    Gets the SSID and SSID Length.

  Parameters:
    p_ssid - pointer to buffer where SSID will be written
    ssidLength - number of bytes in SSID

  Returns:
    None.
 */
void DRV_WIFI_SsidGet(uint8_t *p_ssid, uint8_t *p_ssidLength)
{
    CPELEMENT_RESPONSE_HDR mgmtHdr;

    /* Request SSID, but don't have this function read data or free response buffer. */
    CPElementGet(WF_CP_ELEMENT_SSID, /* Element ID */
                 NULL, /* ptr to element data (not used here) */
                 0, /* num data bytes to read (not used here */
                 false); /* no read, leave response mounted */

    /* At this point, management response is mounted and ready to be read.                 */
    /* Set raw index to 0, read normal 4 byte header plus the next 3 bytes, these will be: */
    /*   profile id [4]                                                                    */
    /*   element id [5]                                                                    */
    /*   element data length [6]                                                           */
    RawRead(RAW_SCRATCH_ID, MGMT_RX_BASE, sizeof(CPELEMENT_RESPONSE_HDR), (uint8_t *)&mgmtHdr);

    /* extract SSID length and write to caller */
    *p_ssidLength = mgmtHdr.elementDataLength;

    /* copy SSID name to callers buffer */
    RawReadRelative(RAW_SCRATCH_ID, *p_ssidLength, p_ssid);
}

// *****************************************************************************
/* Function:
    void DRV_WIFI_BssidSet(uint8_t *p_bssid)

  Summary:
    Sets the the Basic Service Set Identifier (BSSID).

  Description:
    This sets 6 byte (48-bit) MAC address of the Access Point that is being scanned for.
    It is optional to use this.  Where it is useful is if there are two AP's with the
    same ID; the BSSID is used to connect to the specified AP.  This setting can
    be used in lieu of the SSID.  Set each byte to 0xFF (default) if the BSSID is
    not being used.  Not typically needed.

  Parameters:
    p_context - pointer to BSSID

  Returns:
    None.
 */
void DRV_WIFI_BssidSet(uint8_t *p_bssid)
{
    CPElementSet(WF_CP_ELEMENT_BSSID, /* Element ID */
                 p_bssid, /* pointer to element data */
                 DRV_WIFI_BSSID_LENGTH); /* number of element data bytes */
}

// *****************************************************************************
/* Function:
    void DRV_WIFI_BssidGet(uint8_t *p_bssid)

  Summary:
    Gets the the BSSID set in DRV_WIFI_BssidSet().

  Description:
    Retrieves the BSSID set in the previous call to DRV_WIFI_BssidSet().

  Parameters:
    p_context - pointer to where BSSID will be written

  Returns:
    None.
 */
void DRV_WIFI_BssidGet(uint8_t *p_bssid)
{
    CPElementGet(WF_CP_ELEMENT_BSSID, /* Element ID */
                 p_bssid, /* pointer to element data */
                 DRV_WIFI_BSSID_LENGTH, /* number of element data bytes */
                 true); /* read data, free buffer after read */
}

// *****************************************************************************
/* Function:
    void DRV_WIFI_NetworkTypeSet(uint8_t networkType)

  Summary:
    Sets the Wi-Fi network type.

  Description:
    This function selects the WiFi network type.

  Parameters:
    networkType - One of the following:
                    DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE
                    DRV_WIFI_NETWORK_TYPE_ADHOC
                    DRV_WIFI_NETWORK_TYPE_SOFT_AP

  Returns:
    None.
 */
void DRV_WIFI_NetworkTypeSet(uint8_t networkType)
{
    CPElementSet(WF_CP_ELEMENT_NETWORK_TYPE, /* Element ID */
                 &networkType, /* pointer to element data */
                 1); /* number of element data bytes */
    DRV_WIFI_CONFIG_PARAMS(networkType) = networkType;
}

// *****************************************************************************
/* Function:
    void DRV_WIFI_NetworkTypeGet(uint8_t *p_networkType)

  Summary:
    Gets the Wi-Fi network type.

  Description:
    This function gets the WiFi network type.

  Parameters:
    p_networkType - Pointer to where the network type will be written.
                    One of the following:
                      DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE
                      DRV_WIFI_NETWORK_TYPE_ADHOC
                      DRV_WIFI_NETWORK_TYPE_SOFT_AP

  Returns:
    None.
 */
void DRV_WIFI_NetworkTypeGet(uint8_t *p_networkType)
{
    CPElementGet(WF_CP_ELEMENT_NETWORK_TYPE, /* element ID */
                 p_networkType, /* element data pointer */
                 1, /* read one byte */
                 true); /* read data, free buffer */
}

/*******************************************************************************
  Function:
    void SetWepKeyType(uint8_t wepKeyType)

  Summary:
    Sets the WEP key type.

  Description:
    Sets the WEP key type.

  Precondition:
    MACInit must be called first.

  Parameters:
    wepKeyType -- DRV_WIFI_SECURITY_WEP_SHAREDKEY or DRV_WIFI_SECURITY_WEP_OPENKEY (default)

  Returns:
    None.

  Remarks:
    None.
 *******************************************************************************/
void SetWepKeyType(uint8_t wepKeyType)
{
    CPElementSet(WF_CP_ELEMENT_WEPKEY_TYPE, /* Element ID */
                 &wepKeyType, /* pointer to element data */
                 1); /* number of element data bytes */
    DRV_WIFI_CONFIG_PARAMS(wepKeyType) = wepKeyType;
}

/*******************************************************************************
  Function:
    void DRV_WIFI_WepKeyTypeGet(uint8_t *p_keyType)

  Summary:
    Gets the WEP Key type.

  Description:
    This function gets the WEP key type:
      * DRV_WIFI_SECURITY_WEP_SHAREDKEY
      * DRV_WIFI_SECURITY_WEP_OPENKEY

  Parameters:
    p_keyType -- pointer to where key type will be written

  Returns:
    None.
 *******************************************************************************/
void DRV_WIFI_WepKeyTypeGet(uint8_t *p_wepKeyType)
{
    CPElementGet(WF_CP_ELEMENT_WEPKEY_TYPE, /* element ID */
                 p_wepKeyType, /* element data pointer */
                 1, /* read one byte */
                 true); /* read data, free buffer */
}

/*******************************************************************************
  Function:
    void DRV_WIFI_WPSCredentialsGet(DRV_WIFI_WPS_CREDENTIAL *p_cred)

  Summary:
    Gets the WPS credentials.

  Description:
    This function gets the WPS credentials from the MRF24WG.

  Parameters:
    p_cred -- pointer to where WPS credentials will be written

  Returns:
    None.
 *******************************************************************************/
void DRV_WIFI_WPSCredentialsGet(DRV_WIFI_WPS_CREDENTIAL *p_cred)
{
    CPElementGet(WF_CP_ELEMENT_READ_WPS_CRED, /* Element ID */
                 (uint8_t *)p_cred, /* pointer to element data */
                 sizeof(*p_cred), /* number of element data bytes */
                 true); /* read data, free buffer after read */

    // fix 16-bit endianness
    p_cred->encType = TCPIP_Helper_ntohs(p_cred->encType);
    p_cred->authType = TCPIP_Helper_ntohs(p_cred->authType);
}

// *****************************************************************************
/* Function:
    void DRV_WIFI_SecurityOpenSet(void)

  Summary:
    Sets Wi-Fi security to open (no security).

  Description:
    This function sets the Wi-Fi security to open.  One can only connect to an AP
    that is running in open mode.

  Parameters:
    None.

 Returns:
    None.
 */
void DRV_WIFI_SecurityOpenSet()
{
    SetSecurity(DRV_WIFI_SECURITY_OPEN, NULL, 0);
}

// *****************************************************************************
/* Function:
    void DRV_WIFI_SecurityWepSet(DRV_WIFI_WEP_CONTEXT *p_context)

  Summary:
    Sets Wi-Fi security to use WEP.

  Description:
    This function sets the Wi-Fi security to WEP.  One can only connect to an AP
    that is running the same WEP mode.

  Precondition:
    Wi-Fi initialization must be complete.  Must be in an unconnected state.

  Parameters:
    p_context - Desired WEP context.  See DRV_WIFI_WEP_CONTEXT structure.

  Returns:
    None.

  Example:
    <code>
        DRV_WIFI_WEP_CONTEXT context;

        context.wepSecurityType = DRV_WIFI_SECURITY_WEP_40;
        context.wepKey[] = {0x5a, 0xfb, 0x6c, 0x8e, 0x77,
                            0xc1, 0x04, 0x49, 0xfd, 0x4e,
                            0x43, 0x18, 0x2b, 0x33, 0x88,
                            0xb0, 0x73, 0x69, 0xf4, 0x78};

        context.wepKeyLength = 20;
        context.wepKeyType = DRV_WIFI_SECURITY_WEP_OPENKEY;
        DRV_WIFI_SecurityWepSet(&context);
    </code>

  Remarks:
    None.
 */
void DRV_WIFI_SecurityWepSet(DRV_WIFI_WEP_CONTEXT *p_context)
{
    SetSecurity(p_context->wepSecurityType,
                p_context->wepKey,
                p_context->wepKeyLength);

    SetWepKeyType(p_context->wepKeyType);
}

// *****************************************************************************
/* Function:
    DRV_WIFI_SecurityWpaSet(DRV_WIFI_WPA_CONTEXT *p_context)

  Summary:
    Sets WiFi security to use WPA or WPA2.

  Description:
    This function sets the WiFi security to WPA or WPA2.  One can only connect to
    an AP that is running the same WPA mode.

  Parameters:
    p_context - Desired WPA context.  See DRV_WIFI_WPA_CONTEXT structure.

 Returns:
    None.
 */
void DRV_WIFI_SecurityWpaSet(DRV_WIFI_WPA_CONTEXT *p_context)
{
#if defined(WF_ERROR_CHECKING)
    uint32_t errorCode;

    errorCode = UdSetSecurityWpa(p_context);
    if (errorCode != UD_SUCCESS)
    {
        EventEnqueue(WF_EVENT_ERROR, errorCode);
        return;
    }
#endif /* WF_ERROR_CHECKING */
    SetSecurity(p_context->wpaSecurityType,
                p_context->keyInfo.key,
                p_context->keyInfo.keyLength);
}

// *****************************************************************************
/* Function:
    void DRV_WIFI_SecurityWpsSet(DRV_WIFI_WPS_CONTEXT *p_context)

  Summary:
    Sets WiFi security to use WPS.

  Description:
    This function sets the WiFi security to WPS.  One can only connect to
    an AP that supports WPS.

  Parameters:
    p_context - Desired WPA context.  See DRV_WIFI_WPS_CONTEXT structure.

 Returns:
    None.
 */
void DRV_WIFI_SecurityWpsSet(DRV_WIFI_WPS_CONTEXT *p_context)
{
    SetSecurity(p_context->wpsSecurityType,
                p_context->wpsPin,
                p_context->wpsPinLength);
}

/*******************************************************************************
  Function:
    void SetSecurity(uint8_t securityType,
                     uint8_t *p_securityKey,
                     uint8_t securityKeyLength)

  Summary:
    Sets the security.

  Description:
    Configures security.

    <table>
    Security                                      Key         Length
    --------                                      ---         ------
    DRV_WIFI_SECURITY_OPEN                        N/A         N/A
    DRV_WIFI_SECURITY_WEP_40                      hex         4, 5 byte keys
    DRV_WIFI_SECURITY_WEP_104                     hex         4, 13 byte keys
    DRV_WIFI_SECURITY_WPA_AUTO_WITH_KEY           hex         32 bytes
    DRV_WIFI_SECURITY_WPA_AUTO_WITH_PASS_PHRASE   ascii       8-63 ascii characters
    DRV_WIFI_SECURITY_WPA_WITH_KEY                hex         32 bytes
    DRV_WIFI_SECURITY_WPA_WITH_PASS_PHRASE        ascii       8-63 ascii characters
    DRV_WIFI_SECURITY_WPA2_WITH_KEY               hex         32 bytes
    DRV_WIFI_SECURITY_WPA2_WITH_PASS_PHRASE       ascii       8-63 ascii characters
    </table>

  Precondition:
    MACInit must be called first.

  Parameters:
    securityType - Value corresponding to the security type desired.
    p_securityKey - Binary key or passphrase (not used if security is
                     DRV_WIFI_SECURITY_OPEN).
    securityKeyLength - Number of bytes in p_securityKey (not used if security
                         is DRV_WIFI_SECURITY_OPEN).

  Returns:
    None.

  Remarks:
    None.
 *******************************************************************************/
static void SetSecurity(uint8_t securityType,
                        uint8_t *p_securityKey,
                        uint8_t securityKeyLength)
{
    uint8_t hdrBuf[7];
    uint8_t *p_key;

    /* write out header portion of msg */
    hdrBuf[0] = WF_MGMT_REQUEST_TYPE;      /* indicate this is a mgmt msg     */
    hdrBuf[1] = WF_CP_SET_ELEMENT_SUBTYPE; /* MGMT Request Subtype            */
    hdrBuf[2] = CPID;                      /* Connection Profile ID           */
    hdrBuf[3] = WF_CP_ELEMENT_SECURITY;    /* Element ID                      */

    /* Next to header bytes are really part of data, but need to put them in header */
    /* bytes in order to prepend to security key.                                   */
    hdrBuf[5] = securityType;
    hdrBuf[6] = 0; /* only support wep key index 0 */

    /* if security is open (no key) or WPS push button method */
    if (securityType == DRV_WIFI_SECURITY_OPEN ||
        securityType == DRV_WIFI_SECURITY_WPS_PUSH_BUTTON)
    {
        hdrBuf[4] = 2; /* only data is security type and wep index */
        p_key = NULL;
        securityKeyLength = 0;
    }
    /* else security is selected, so need to send key */
    else
    {
        hdrBuf[4] = 2 + securityKeyLength; /* data is security type + wep index + key */
        p_key = p_securityKey;
    }

    SendMgmtMsg(hdrBuf, sizeof(hdrBuf), p_key, securityKeyLength); 

    /* wait for mgmt response, free after it comes in, don't need data bytes */
    WaitForMgmtResponse(WF_CP_SET_ELEMENT_SUBTYPE, FREE_MGMT_BUFFER);

    DRV_WIFI_CONFIG_PARAMS(securityMode) = securityType;
    DRV_WIFI_CONFIG_PARAMS(securityKeyLen) = securityKeyLength;
    memcpy(DRV_WIFI_CONFIG_PARAMS(securityKey), p_securityKey, securityKeyLength);
}

/*******************************************************************************
  Function:
    void DRV_WIFI_SecurityGet(uint8_t *p_securityType,
                              uint8_t *p_securityKey,
                              uint8_t *p_securityKeyLength)

  Summary:
    Gets the current Wi-Fi security setting.

  Description:
    This function gets the current Wi-Fi security setting.

    <table>
    Security                                      Key         Length
    --------                                      ---         ------
    DRV_WIFI_SECURITY_OPEN                        N/A         N/A
    DRV_WIFI_SECURITY_WEP_40                      binary      4 keys, 5 bytes each (total of 20 bytes)
    DRV_WIFI_SECURITY_WEP_104                     binary      4 keys, 13 bytes each (total of 52 bytes)
    DRV_WIFI_SECURITY_WPA_AUTO_WITH_KEY           binary      32 bytes
    DRV_WIFI_SECURITY_WPA_AUTO_WITH_PASS_PHRASE   ascii       8-63 ascii characters
    DRV_WIFI_SECURITY_WPA_WITH_KEY                binary      32 bytes
    DRV_WIFI_SECURITY_WPA_WITH_PASS_PHRASE        ascii       8-63 ascii characters
    DRV_WIFI_SECURITY_WPA2_WITH_KEY               binary      32 bytes
    DRV_WIFI_SECURITY_WPA2_WITH_PASS_PHRASE       ascii       8-63 ascii characters
    </table>

  Precondition:
    Wi-Fi initialization must be complete.

  Parameters:
    p_securityType - Value corresponding to the security type desired.
    p_securityKey - Binary key or passphrase (not used if security is DRV_WIFI_SECURITY_OPEN).
    p_securityKeyLength - Number of bytes in p_securityKey (not used if security is DRV_WIFI_SECURITY_OPEN).

  Returns:
    None.

   Example:
    <code>
        uint8_t securityType;
        uint8_t securityKey[DRV_WIFI_MAX_SECURITY_KEY_LENGTH];
        uint8_t keyLength;

        DRV_WIFI_SecurityGet(&securityType, securityKey, &keyLength);
    </code>

  Remarks:
    If security was initially set with a passphrase that the MRF24WG used to generate
    a binary key, this function returns the binary key, not the passphrase.
 *******************************************************************************/
void DRV_WIFI_SecurityGet(uint8_t *p_securityType,
                          uint8_t *p_securityKey,
                          uint8_t *p_securityKeyLength)
{
    CPELEMENT_RESPONSE_HDR mgmtHdr;
    uint8_t keyLength;
    uint8_t wepKeyIndex;

    /* send request, wait for mgmt response, do not read and do not free up response buffer */
    CPElementGet(WF_CP_ELEMENT_SECURITY, /* Element ID */
                 NULL,                   /* do not read */
                 0,                      /* do not read */
                 false);                 /* do not read, do not free MGMT buffer */

    /* at this point, management response is mounted and ready to be read */

    /* At this point, management response is mounted and ready to be read.                 */
    /* Set raw index to 0, read normal 4 byte header plus the next 3 bytes, these will be: */
    /*   profile id             [4]                                                        */
    /*   element id             [5]                                                        */
    /*   element data length    [6]                                                        */
    RawRead(RAW_SCRATCH_ID, MGMT_RX_BASE, sizeof(CPELEMENT_RESPONSE_HDR), (uint8_t *)&mgmtHdr);

    // security type
    RawReadRelative(RAW_SCRATCH_ID, 1, p_securityType);

    // read wep key index to increment pointer in raw buffer, but throw away, it is always 0
    RawReadRelative(RAW_SCRATCH_ID, 1, &wepKeyIndex);

    /* determine security key length and read if it is present */
    keyLength = mgmtHdr.elementDataLength - 2;
    if (keyLength > 0)
    {
        *p_securityKeyLength = keyLength;
        RawReadRelative(RAW_SCRATCH_ID, keyLength, p_securityKey);
    }
    /* no security key, so set key length param to 0 */
    else
    {
        *p_securityKeyLength = 0;
    }
}

void DRV_WIFI_SecurityTypeGet(uint8_t *p_securityType)
{
    CPELEMENT_RESPONSE_HDR mgmtHdr;

    /* send request, wait for mgmt response, do not read and do not free up response buffer */
    CPElementGet(WF_CP_ELEMENT_SECURITY, /* Element ID */
                 NULL,                   /* do not read */
                 0,                      /* do not read */
                 false);                 /* do not read, do not free mgmt buffer */

    /* at this point, management response is mounted and ready to be read */

    /* At this point, management response is mounted and ready to be read.                 */
    /* Set raw index to 0, read normal 4 byte header plus the next 3 bytes, these will be: */
    /*   profile id             [4]                                                        */
    /*   element id             [5]                                                        */
    /*   element data length    [6]                                                        */
    RawRead(RAW_SCRATCH_ID, MGMT_RX_BASE, sizeof(CPELEMENT_RESPONSE_HDR), (uint8_t *)&mgmtHdr);

    // security type
    RawReadRelative(RAW_SCRATCH_ID, 1, p_securityType);
}

void DRV_WIFI_HiddenSsidSet(bool hiddenSsid)
{
    CPElementSet(WF_CP_ELEMENT_SSID_TYPE, // Element ID
                 (uint8_t *)&hiddenSsid,  // pointer to element data
                 1);                      // number of element data bytes
}

uint8_t DRV_WIFI_HiddenSsidGet(void)
{
    uint8_t hidden;

    CPElementGet(WF_CP_ELEMENT_SSID_TYPE, /* element ID             */
                 &hidden,                 /* element data pointer   */
                 1,                       /* read one byte          */
                 true);                   /* read data, free buffer */

    return hidden;
}

// Called from SetAdhocContext().  Error checking performed there.
void DRV_WIFI_AdhocModeSet(uint8_t mode)
{
    CPElementSet(WF_CP_ELEMENT_ADHOC_BEHAVIOR, // Element ID
                 &mode,                        // pointer to element data
                 1);                           // number of element data bytes
}

uint8_t DRV_WIFI_AdhocModeGet(void)
{
    uint8_t adhocMode;

    CPElementGet(WF_CP_ELEMENT_ADHOC_BEHAVIOR, /* element ID             */
                 &adhocMode,                   /* element data pointer   */
                 1,                            /* read one byte          */
                 true);                        /* read data, free buffer */

    return adhocMode;
}

/*******************************************************************************
  Function:
    static void CPElementSet(uint8_t elementId,
                             uint8_t *p_elementData,
                             uint8_t elementDataLength)

  Summary:
    Set an element of the connection profile on the MRF24W.

  Description:
    All Connection Profile 'Set Element' functions call this function to
    construct the management message.  The caller must fix up any endian issues
    prior to calling this function.

  Precondition:
    MACInit must be called first.

  Parameters:
    elementId - Element that is being set.
    p_elementData - Pointer to element data.
    elementDataLength - Number of bytes pointed to by p_elementData.

  Returns:
    None.

  Remarks:
    None.
 *******************************************************************************/
static void CPElementSet(uint8_t elementId,
                         uint8_t *p_elementData,
                         uint8_t elementDataLength)
{
    uint8_t hdrBuf[5];

    /* Write out header portion of msg. */
    hdrBuf[0] = WF_MGMT_REQUEST_TYPE;      /* indicate this is a mgmt msg     */
    hdrBuf[1] = WF_CP_SET_ELEMENT_SUBTYPE; /* MGMT Request Subtype            */
    hdrBuf[2] = CPID;                      /* Connection Profile ID           */
    hdrBuf[3] = elementId;                 /* Element ID                      */
    hdrBuf[4] = elementDataLength;         /* number of bytes of element data */

    SendMgmtMsg(hdrBuf, sizeof(hdrBuf), p_elementData, elementDataLength);  

    /* wait for mgmt response, free after it comes in, don't need data bytes */
    WaitForMgmtResponse(WF_CP_SET_ELEMENT_SUBTYPE, FREE_MGMT_BUFFER);
}

/*******************************************************************************
  Function:
    static void CPElementGet(uint8_t elementId,
                             uint8_t *p_elementData,
                             uint8_t elementDataLength,
                             uint8_t dataReadAction)

  Summary:
    Get an element of the connection profile on the MRF24W.

  Description:
    All Connection Profile 'Get Element' functions call this function to
    construct the management message.  The caller must fix up any endian issues
    prior to calling this function.

  Precondition:
    MACInit must be called first.

  Parameters:
    elementId - Element that is being read.
    p_elementData - Pointer to where element data will be written.
    elementDataLength - Number of element data bytes that will be read.
    dataReadAction - If true then read data per paramters and free mgmt
                      response buffer. If false then return after response
                      received, do not read any data as the caller will do that,
                      and don't free buffer, as caller will do that as well.

  Returns:
    None.

  Remarks:
    None.
 *******************************************************************************/
static void CPElementGet(uint8_t elementId,
                         uint8_t *p_elementData,
                         uint8_t elementDataLength,
                         uint8_t dataReadAction) /* true or false */
{
    uint8_t  hdrBuf[4];

    hdrBuf[0] = WF_MGMT_REQUEST_TYPE;      /* indicate this is a mgmt msg */
    hdrBuf[1] = WF_CP_GET_ELEMENT_SUBTYPE; /* mgmt request subtype        */
    hdrBuf[2] = CPID;                      /* Connection Profile ID       */
    hdrBuf[3] = elementId;                 /* Element ID                  */

    SendMgmtMsg(hdrBuf, sizeof(hdrBuf), NULL, 0);                 

    if (dataReadAction == (uint8_t)true)
    {
        /* wait for mgmt response, read desired data, and then free response buffer */
        WaitForMgmtResponseAndReadData(WF_CP_GET_ELEMENT_SUBTYPE, elementDataLength,                   
            sizeof(CPELEMENT_RESPONSE_HDR), p_elementData);                      
    }
    else
    {
        /* wait for mgmt response, don't read any data bytes, do not release mgmt buffer */
        WaitForMgmtResponse(WF_CP_GET_ELEMENT_SUBTYPE, DO_NOT_FREE_MGMT_BUFFER);
    }
}

//DOM-IGNORE-END
