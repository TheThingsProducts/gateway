/*******************************************************************************
  MRF24WG Driver Customization

  Summary:
    Module for Microchip TCP/IP Stack

  Description:
    -Provides access to MRF24WG Wi-Fi controller
    -Reference: MRF24WG Datasheet, IEEE 802.11 Standard
 *******************************************************************************/

/*******************************************************************************
File Name: drv_wifi_config.h
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _WF_CONFIG_H_
#define _WF_CONFIG_H_

#define APP_WIFI_EASY_CONFIG_DEMO

#define APP_WIFI_EASY_CONFIG_DEMO_VERSION_NUMBER "1.0"

/*------------------------------------------------------------------------*
 * By default MRF24W module came pre-programmed with unique MAC address.  *
 * This can be overridden with new MAC address in application by defining *
 * MY_DEFAULT_MAC_ADDRESS                                                 *
 * -----------------------------------------------------------------------*/
//#defined MY_DEFAULT_MAC_ADDRESS

/*******************************************
 * Select a default network mode for demo. *
 *                                         *
 * Easy Configuration Demo:                *
 *     DRV_WIFI_NETWORK_TYPE_ADHOC         *
 *     DRV_WIFI_NETWORK_TYPE_SOFT_AP       *
 *******************************************/
#define DRV_WIFI_DEFAULT_NETWORK_TYPE    DRV_WIFI_NETWORK_TYPE_SOFT_AP

#if (DRV_WIFI_DEFAULT_NETWORK_TYPE == DRV_WIFI_NETWORK_TYPE_ADHOC)
/*************************************************************************
 * Configure network for Ad-Hoc mode.                                    *
 *                                                                       *
 * Available security configuration:                                     *
 *      DRV_WIFI_SECURITY_OPEN       : No security                       *
 *      DRV_WIFI_SECURITY_WEP_40     : WEP encryption using 40 bit keys  *
 *      DRV_WIFI_SECURITY_WEP_104    : WEP encryption using 104 bit keys *
 *************************************************************************/
    #define DRV_WIFI_DEFAULT_SSID                "MicrochipDemoAdHoc"
    #define DRV_WIFI_DEFAULT_SECURITY_MODE       DRV_WIFI_SECURITY_OPEN
    #define DRV_WIFI_DEFAULT_LIST_RETRY_COUNT    (DRV_WIFI_RETRY_ADHOC) /* Number (1..254) of times to try to connect to the SSID when using Ad-Hoc network type. */
    #define DRV_WIFI_DEFAULT_CHANNEL_LIST        {}                     /* Ad-Hoc network channel - use domain's default channel in module */
    #define DRV_WIFI_DEFAULT_POWER_SAVE          DRV_WIFI_DISABLED      /* PS POLL not supported in Ad-Hoc - must be set to DRV_WIFI_DISABLED */

#elif (DRV_WIFI_DEFAULT_NETWORK_TYPE == DRV_WIFI_NETWORK_TYPE_SOFT_AP)
/*************************************************************************
 * Configure network for Soft AP mode.                                   *
 *                                                                       *
 * Available security configuration:                                     *
 *      DRV_WIFI_SECURITY_OPEN       : No security                       *
 *      DRV_WIFI_SECURITY_WEP_40     : WEP Encryption using 40 bit keys  *
 *      DRV_WIFI_SECURITY_WEP_104    : WEP Encryption using 104 bit keys *
 *************************************************************************/

   /* Soft AP function has the full "EasyConfig" function.
    * Your STA can connect to the Soft AP as a client, get the DHCP IP, run ping, and run web browser to connect to Web Server
    * of Soft AP. It will allow you to re-connect / redirect to another AP in infrastructure mode.
    * The reason this demo sits here is simply A) borrow DHCP server; B) borrow HTTP server.
    *
    * Before starting up as Soft AP, prescan will be executed.
    * For Soft AP, default channel is assigned as MY_DEFAULT_CHANNEL_LIST i.e single channel 6. This means Soft AP
    * will start up in channel 6.
    * When scan option is selected in EZConfig web browser in Soft AP mode, the prescan results will be displayed.
    * Repeated selection of this scan option will always display the prescan results. From this prescan list, the user can select
    * an AP to be redirected to.
    * When redirected to another AP, the channel list will change to scan all channels.
    *
    * Also note that this is a very simplified Soft AP. So its function is limited as, A) no routing supported; B) only 1 client allowed
    * at a time.  And security wise currently it supports both open mode and WEP security.
    *
    * Soft AP's default IP is 192.168.1.3 and its Network Mask is 255.255.0.0
    * Soft AP on certain setups with IP address 192.168.1.1 has problem with DHCP client assigning new IP address on redirection.
    * 192.168.1.1 is a common IP address with most APs. This is still under investigation.
    * For now, assign this as 192.168.1.3.
    */
    #define DRV_WIFI_DEFAULT_SSID                "MCHPSoftAP"
    #define DRV_WIFI_DEFAULT_SECURITY_MODE       DRV_WIFI_SECURITY_OPEN
    #define DRV_WIFI_DEFAULT_LIST_RETRY_COUNT    (DRV_WIFI_RETRY_ADHOC) /* Dummy, Not Used */
    #define DRV_WIFI_DEFAULT_CHANNEL_LIST        {6}                    /* Set Soft AP network channel. */
    #define DRV_WIFI_SOFTAP_CHECK_LINK_STATUS    DRV_WIFI_DISABLED      /* DRV_WIFI_ENABLED to use with DRV_WIFI_SOFTAP_LINK_FAILURE_THRESHOLD */
    #define DRV_WIFI_DEFAULT_POWER_SAVE          DRV_WIFI_DISABLED      /* PS POLL not supported in Soft AP - must be set to DRV_WIFI_DISABLED */

    /* Consecutive null packet transmission failures for this amount of times.
     * Then Soft AP considers the client has gone away. This is only effective
     * when DRV_WIFI_SOFTAP_CHECK_LINK_STATUS is enabled.
     */
    #if (DRV_WIFI_SOFTAP_CHECK_LINK_STATUS == DRV_WIFI_ENABLED)
        #define DRV_WIFI_SOFTAP_LINK_FAILURE_THRESHOLD    40
    #endif
#endif /* DRV_WIFI_DEFAULT_NETWORK_TYPE */

/***************************************************************
 * MRF24WG specific features - uncomment to enable the feature *
 ***************************************************************/

/* Conduct scan prior to connecting to network. */
#define DRV_WIFI_DEFAULT_ADHOC_PRESCAN    DRV_WIFI_DISABLED

/*------------------------------------------------------*
 * Only used when security is DRV_WIFI_DEFAULT_WPS_PIN. *
 * An example PIN.                                      *
 * The last digit is the checksum of first 7 digits.    *
 * -----------------------------------------------------*/
#define DRV_WIFI_DEFAULT_WPS_PIN    "12390212"

/*----------------------------------------------------------------------------------*
 * MRF24W FW has a built-in connection manager, and it is enabled by default.       *
 * So if you want to run your own connection manager in host side,                  *
 * you should disable the module connection manager to avoid some possible conflict *
 * between the two.  These two APIs can be affected if you do not disable it.       *
 *   A) DRV_WIFI_Disconnect()                                                       *
 *   B) DRV_WIFI_Scan()                                                             *
 * These APIs will return failure when the conflict occurs.                         *
 *                                                                                  *
 * Note: This feature is current available with Infrastructure mode only.           *
 *----------------------------------------------------------------------------------*/
#define DRV_WIFI_MODULE_CONNECTION_MANAGER    DRV_WIFI_ENABLED

/*------------------------------------------------------------------------------*
 * This option gets the stack to save the credentials to the internal flash.    *
 * And the stored credentials can be used for next connection with the same AP. *
 *------------------------------------------------------------------------------*/
#if (DRV_WIFI_DEFAULT_SECURITY_MODE == DRV_WIFI_SECURITY_WPS_PUSH_BUTTON) || \
    (DRV_WIFI_DEFAULT_SECURITY_MODE == DRV_WIFI_SECURITY_WPS_PIN)
    #define DRV_WIFI_SAVE_WPS_CREDENTIALS    DRV_WIFI_DISABLED
#endif

/*------------------------------------------------------------------------------------------*
 * Select IP Configuration                                                                  *
 *                                                                                          *
 * Note: Select Static IP for DRV_WIFI_NETWORK_TYPE_ADHOC and DRV_WIFI_NETWORK_TYPE_SOFT_AP *
 *------------------------------------------------------------------------------------------*/
#if (DRV_WIFI_DEFAULT_NETWORK_TYPE == DRV_WIFI_NETWORK_TYPE_ADHOC || DRV_WIFI_DEFAULT_NETWORK_TYPE == DRV_WIFI_NETWORK_TYPE_SOFT_AP)
    #define DRV_WIFI_ENABLE_STATIC_IP // static IP
#else
    #define DRV_WIFI_ENABLE_DHCP // dynamic IP
#endif

#define DRV_WIFI_GRATUITOUS_ARP    DRV_WIFI_DISABLED

/*---------------------------------------------------------------------------------*
 * Default WEP keys used in DRV_WIFI_SECURITY_WEP_40 and DRV_WIFI_SECURITY_WEP_104 *
 * security mode.                                                                  *
 *---------------------------------------------------------------------------------*/
#if (DRV_WIFI_DEFAULT_SECURITY_MODE == DRV_WIFI_SECURITY_WEP_40)    \
                    ||                                              \
    (DRV_WIFI_DEFAULT_SECURITY_MODE == DRV_WIFI_SECURITY_WEP_104)

#define DRV_WIFI_DEFAULT_WEP_PHRASE    "WEP Phrase"

#if (DRV_WIFI_DEFAULT_SECURITY_MODE == DRV_WIFI_SECURITY_WEP_40)
// string containing 4 40-bit WEP keys -- corresponding to passphrase of "WEP Phrase"
#define DRV_WIFI_DEFAULT_WEP_KEY_40 "5AFB6C8E77"
#define DRV_WIFI_DEFAULT_WEP_KEYS_40 "\
\x5a\xfb\x6c\x8e\x77\
\xc1\x04\x49\xfd\x4e\
\x43\x18\x2b\x33\x88\
\xb0\x73\x69\xf4\x78"
// Do not indent above string as it will inject spaces.
#endif

#if (DRV_WIFI_DEFAULT_SECURITY_MODE == DRV_WIFI_SECURITY_WEP_104)
// string containing 4 104-bit WEP keys -- corresponding to passphrase of "WEP Phrase"
#define DRV_WIFI_DEFAULT_WEP_KEY_104 "90E96780C739409DA50034FCAA"
#define DRV_WIFI_DEFAULT_WEP_KEYS_104 "\
\x90\xe9\x67\x80\xc7\x39\x40\x9d\xa5\x00\x34\xfc\xaa\
\x77\x4a\x69\x45\xa4\x3d\x66\x63\xfe\x5b\x1d\xb9\xfd\
\x82\x29\x87\x4c\x9b\xdc\x6d\xdf\x87\xd1\xcf\x17\x41\
\xcc\xd7\x62\xde\x92\xad\xba\x3b\x62\x2f\x7f\xbe\xfb"
// Do not indent above string as it will inject spaces.
#endif

#endif /* WEP40 and WEP104 */

#endif /* _WF_CONFIG_H_ */
