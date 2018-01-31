<#--
/*******************************************************************************
  Wi-Fi Driver Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_wifi.h.ftl

  Summary:
    Wi-Fi Driver Freemarker Template File

  Description:

 *******************************************************************************/

/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
 *******************************************************************************/
-->

/*** Wi-Fi Driver Configuration ***/
<#if CONFIG_DRV_WIFI_DEVICE == "MRF24WG">

#define DRV_WIFI_CONFIG_MHC
<#if CONFIG_3RDPARTY_RTOS_USED == "FreeRTOS_V8.x.x">
#define DRV_WIFI_USE_FREERTOS

#define DRV_WIFI_RTOS_INIT_TASK_SIZE ${CONFIG_DRV_WIFI_RTOS_INIT_TASK_SIZE}u
#define DRV_WIFI_RTOS_INIT_TASK_PRIORITY ${CONFIG_DRV_WIFI_RTOS_INIT_TASK_PRIORITY}u
#define DRV_WIFI_RTOS_DEFERRED_ISR_SIZE ${CONFIG_DRV_WIFI_RTOS_DEFERRED_ISR_SIZE}u
#define DRV_WIFI_RTOS_DEFERRED_ISR_PRIORITY ${CONFIG_DRV_WIFI_RTOS_DEFERRED_ISR_PRIORITY}u
#define DRV_WIFI_RTOS_MAC_TASK_SIZE ${CONFIG_DRV_WIFI_RTOS_MAC_TASK_SIZE}u
#define DRV_WIFI_RTOS_MAC_TASK_PRIORITY ${CONFIG_DRV_WIFI_RTOS_MAC_TASK_PRIORITY}u
</#if><#-- CONFIG_3RDPARTY_RTOS_USED -->

#define DRV_WIFI_ASSERT(condition, msg) DRV_WIFI_Assert(condition, msg, __FILE__, __LINE__)

<#if CONFIG_DRV_WIFI_SPI_INSTANCE_INDEX == '0'>
#define DRV_WIFI_SPI_INDEX 0
#define DRV_WIFI_SPI_INSTANCE sysObj.spiObjectIdx0
<#elseif CONFIG_DRV_WIFI_SPI_INSTANCE_INDEX == '1'>
#define DRV_WIFI_SPI_INDEX 1
#define DRV_WIFI_SPI_INSTANCE sysObj.spiObjectIdx1
<#elseif CONFIG_DRV_WIFI_SPI_INSTANCE_INDEX == '2'>
#define DRV_WIFI_SPI_INDEX 2
#define DRV_WIFI_SPI_INSTANCE sysObj.spiObjectIdx2
<#elseif CONFIG_DRV_WIFI_SPI_INSTANCE_INDEX == '3'>
#define DRV_WIFI_SPI_INDEX 3
#define DRV_WIFI_SPI_INSTANCE sysObj.spiObjectIdx3
<#elseif CONFIG_DRV_WIFI_SPI_INSTANCE_INDEX == '4'>
#define DRV_WIFI_SPI_INDEX 4
#define DRV_WIFI_SPI_INSTANCE sysObj.spiObjectIdx4
<#elseif CONFIG_DRV_WIFI_SPI_INSTANCE_INDEX == '5'>
#define DRV_WIFI_SPI_INDEX 5
#define DRV_WIFI_SPI_INSTANCE sysObj.spiObjectIdx5
</#if>

<#if CONFIG_DRV_WIFI_USE_SPI_DMA == true>
#define DRV_WIFI_USE_SPI_DMA
</#if><#-- CONFIG_DRV_WIFI_USE_SPI_DMA -->

<#if CONFIG_DRV_WIFI_STORE_CONFIG_IN_NVM>
#define DRV_WIFI_NVM_SPACE_ENABLE
#define DRV_WIFI_NVM_SPACE_ADDR (${CONFIG_DRV_WIFI_NVM_START_ADDRESS} * 1024)
</#if>

<#if CONFIG_BSP_PIC32MX795_PIM_E16 == true>
#define EX16
<#elseif CONFIG_BSP_PIC32MX_ETH_SK == true>
#define MRF24W_USE_CN_INT

// For CN9(GPIO_G7) as extern interrupt, it is better to use another GPIO (GPIO_E0) to read interrupt high or low.
// We need a jump line to connect GPIO_E0 to GPIO_G7.
//#define WF_VERIFY_EINT_BY_ANOTHER_GPIO
#if defined(WF_VERIFY_EINT_BY_ANOTHER_GPIO)
// Use GPIO_E0. Please use a jump line to connect GPIO_E0 to GPIO_G7.
#define WF_INT_PORT_CHANNEL_READ PORT_CHANNEL_E
#define WF_INT_BIT_POS_READ      0
#else
// Still read GPIO_G7. 
#define WF_INT_PORT_CHANNEL_READ PORT_CHANNEL_G
#define WF_INT_BIT_POS_READ      7
#endif
</#if><#-- CONFIG_BSP -->

// I/O mappings for general control pins, including CS, HIBERNATE, RESET and INTERRUPT.
#define WF_CS_PORT_CHANNEL PORT_CHANNEL_${CONFIG_DRV_WIFI_CS_PORT_CHANNEL}
#define WF_CS_BIT_POS      ${CONFIG_DRV_WIFI_CS_BIT_POS}

#define WF_HIBERNATE_PORT_CHANNEL PORT_CHANNEL_${CONFIG_DRV_WIFI_HIBERNATE_PORT_CHANNEL}
#define WF_HIBERNATE_BIT_POS      ${CONFIG_DRV_WIFI_HIBERNATE_BIT_POS}

#define WF_RESET_PORT_CHANNEL PORT_CHANNEL_${CONFIG_DRV_WIFI_RESET_PORT_CHANNEL}
#define WF_RESET_BIT_POS      ${CONFIG_DRV_WIFI_RESET_BIT_POS}

#define WF_INT_PORT_CHANNEL PORT_CHANNEL_${CONFIG_DRV_WIFI_INT_PORT_CHANNEL}
#define WF_INT_BIT_POS      ${CONFIG_DRV_WIFI_INT_BIT_POS}

<#if CONFIG_DRV_WIFI_INTERRUPT_SYSTEM_SERVICE == "External Interrupt">
 <#if CONFIG_EXT_INT_INST_IDX0_USED_BY_DRV_WIFI>
#define MRF_INT_SOURCE ${CONFIG_EXT_INT_INTERRUPT_SOURCE_IDX0}
#define MRF_INT_VECTOR ${CONFIG_EXT_INT_INTERRUPT_VECTOR_IDX0}
 <#elseif CONFIG_EXT_INT_INST_IDX1_USED_BY_DRV_WIFI>
#define MRF_INT_SOURCE ${CONFIG_EXT_INT_INTERRUPT_SOURCE_IDX1}
#define MRF_INT_VECTOR ${CONFIG_EXT_INT_INTERRUPT_VECTOR_IDX1}
 <#elseif CONFIG_EXT_INT_INST_IDX2_USED_BY_DRV_WIFI>
#define MRF_INT_SOURCE ${CONFIG_EXT_INT_INTERRUPT_SOURCE_IDX2}
#define MRF_INT_VECTOR ${CONFIG_EXT_INT_INTERRUPT_VECTOR_IDX2}
 <#elseif CONFIG_EXT_INT_INST_IDX3_USED_BY_DRV_WIFI>
#define MRF_INT_SOURCE ${CONFIG_EXT_INT_INTERRUPT_SOURCE_IDX3}
#define MRF_INT_VECTOR ${CONFIG_EXT_INT_INTERRUPT_VECTOR_IDX3}
 <#elseif CONFIG_EXT_INT_INST_IDX4_USED_BY_DRV_WIFI>
#define MRF_INT_SOURCE ${CONFIG_EXT_INT_INTERRUPT_SOURCE_IDX4}
#define MRF_INT_VECTOR ${CONFIG_EXT_INT_INTERRUPT_VECTOR_IDX4}
 <#else>
#error "Please select an external interrupt instance for Wi-Fi driver."
 </#if><#-- Wi-Fi Driver External Interrupt Instance -->
<#elseif CONFIG_DRV_WIFI_INTERRUPT_SYSTEM_SERVICE == "Change Notification">
#define MRF_INT_SOURCE INT_SOURCE_CHANGE_NOTICE
#define MRF_INT_VECTOR INT_VECTOR_CN
</#if><#-- CONFIG_DRV_WIFI_INTERRUPT_SYSTEM_SERVICE -->

<#if CONFIG_DRV_WIFI_NETWORK_TYPE == "Infrastructure">
#define DRV_WIFI_DEFAULT_NETWORK_TYPE       DRV_WIFI_NETWORK_TYPE_INFRASTRUCTURE
<#if CONFIG_DRV_WIFI_SECURITY_MODE_INFRASTRUCTURE == "WPS Push Button">
#define DRV_WIFI_DEFAULT_SSID               ""
<#else>
#define DRV_WIFI_DEFAULT_SSID               "${CONFIG_DRV_WIFI_SSID_NAME}"
</#if>
#define DRV_WIFI_DEFAULT_LIST_RETRY_COUNT   (DRV_WIFI_RETRY_FOREVER) /* Number (1..255) of times to try to connect to the SSID when using Infrastructure network type */
#define DRV_WIFI_DEFAULT_CHANNEL_LIST       {} /* Channel list for Domain - use default in module */

<#if CONFIG_DRV_WIFI_SECURITY_MODE_INFRASTRUCTURE == "Open">
#define DRV_WIFI_DEFAULT_SECURITY_MODE      DRV_WIFI_SECURITY_OPEN
#define DRV_WIFI_DEFAULT_WEP_PHRASE         "WEP Phrase" // default WEP passphrase
#define DRV_WIFI_DEFAULT_WEP_KEY_40         "5AFB6C8E77" // default WEP40 key
#define DRV_WIFI_DEFAULT_WEP_KEY_104        "90E96780C739409DA50034FCAA" // default WEP104 key
#define DRV_WIFI_DEFAULT_PSK_PHRASE         "Microchip 802.11 Secret PSK Password" // default WPA passphrase
#define DRV_WIFI_DEFAULT_WPS_PIN            "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFI_SECURITY_MODE_INFRASTRUCTURE == "WEP-40 (64-bit)">
#define DRV_WIFI_DEFAULT_SECURITY_MODE      DRV_WIFI_SECURITY_WEP_40
#define DRV_WIFI_DEFAULT_WEP_PHRASE         "${CONFIG_DRV_WIFI_WEP_PASS_PHRASE}" // customized WEP passphrase
#define DRV_WIFI_DEFAULT_WEP_KEY_40         "${CONFIG_DRV_WIFI_WEP_KEY_40}" // customized WEP40 key
#define DRV_WIFI_DEFAULT_WEP_KEY_104        "90E96780C739409DA50034FCAA" // default WEP104 key
#define DRV_WIFI_DEFAULT_PSK_PHRASE         "Microchip 802.11 Secret PSK Password" // default WPA passphrase
#define DRV_WIFI_DEFAULT_WPS_PIN            "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFI_SECURITY_MODE_INFRASTRUCTURE == "WEP-104 (128-bit)">
#define DRV_WIFI_DEFAULT_SECURITY_MODE      DRV_WIFI_SECURITY_WEP_104
#define DRV_WIFI_DEFAULT_WEP_PHRASE         "${CONFIG_DRV_WIFI_WEP_PASS_PHRASE}" // customized WEP passphrase
#define DRV_WIFI_DEFAULT_WEP_KEY_40         "5AFB6C8E77" // default WEP40 key
#define DRV_WIFI_DEFAULT_WEP_KEY_104        "${CONFIG_DRV_WIFI_WEP_KEY_104}" // customized WEP104 key
#define DRV_WIFI_DEFAULT_PSK_PHRASE         "Microchip 802.11 Secret PSK Password" // default WPA passphrase
#define DRV_WIFI_DEFAULT_WPS_PIN            "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFI_SECURITY_MODE_INFRASTRUCTURE == "WPA-PSK">
#define DRV_WIFI_DEFAULT_SECURITY_MODE      DRV_WIFI_SECURITY_WPA_WITH_PASS_PHRASE
#define DRV_WIFI_DEFAULT_WEP_PHRASE         "WEP Phrase" // default WEP passphrase
#define DRV_WIFI_DEFAULT_WEP_KEY_40         "5AFB6C8E77" // default WEP40 key
#define DRV_WIFI_DEFAULT_WEP_KEY_104        "90E96780C739409DA50034FCAA" // default WEP104 key
#define DRV_WIFI_DEFAULT_PSK_PHRASE         "${CONFIG_DRV_WIFI_WPA_PASS_PHRASE}" // customized WPA passphrase
#define DRV_WIFI_DEFAULT_WPS_PIN            "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFI_SECURITY_MODE_INFRASTRUCTURE == "WPA2-PSK">
#define DRV_WIFI_DEFAULT_SECURITY_MODE      DRV_WIFI_SECURITY_WPA2_WITH_PASS_PHRASE
#define DRV_WIFI_DEFAULT_WEP_PHRASE         "WEP Phrase" // default WEP passphrase
#define DRV_WIFI_DEFAULT_WEP_KEY_40         "5AFB6C8E77" // default WEP40 key
#define DRV_WIFI_DEFAULT_WEP_KEY_104        "90E96780C739409DA50034FCAA" // default WEP104 key
#define DRV_WIFI_DEFAULT_PSK_PHRASE         "${CONFIG_DRV_WIFI_WPA_PASS_PHRASE}" // customized WPA passphrase
#define DRV_WIFI_DEFAULT_WPS_PIN            "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFI_SECURITY_MODE_INFRASTRUCTURE == "WPA-PSK/WPA2-PSK Auto">
#define DRV_WIFI_DEFAULT_SECURITY_MODE      DRV_WIFI_SECURITY_WPA_AUTO_WITH_PASS_PHRASE
#define DRV_WIFI_DEFAULT_WEP_PHRASE         "WEP Phrase" // default WEP passphrase
#define DRV_WIFI_DEFAULT_WEP_KEY_40         "5AFB6C8E77" // default WEP40 key
#define DRV_WIFI_DEFAULT_WEP_KEY_104        "90E96780C739409DA50034FCAA" // default WEP104 key
#define DRV_WIFI_DEFAULT_PSK_PHRASE         "${CONFIG_DRV_WIFI_WPA_PASS_PHRASE}" // customized WPA passphrase
#define DRV_WIFI_DEFAULT_WPS_PIN            "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFI_SECURITY_MODE_INFRASTRUCTURE == "WPS Push Button">
#define DRV_WIFI_DEFAULT_SECURITY_MODE      DRV_WIFI_SECURITY_WPS_PUSH_BUTTON
#define DRV_WIFI_DEFAULT_WEP_PHRASE         "WEP Phrase" // default WEP passphrase
#define DRV_WIFI_DEFAULT_WEP_KEY_40         "5AFB6C8E77" // default WEP40 key
#define DRV_WIFI_DEFAULT_WEP_KEY_104        "90E96780C739409DA50034FCAA" // default WEP104 key
#define DRV_WIFI_DEFAULT_PSK_PHRASE         "Microchip 802.11 Secret PSK Password" // default WPA passphrase
#define DRV_WIFI_DEFAULT_WPS_PIN            "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFI_SECURITY_MODE_INFRASTRUCTURE == "WPS PIN">
#define DRV_WIFI_DEFAULT_SECURITY_MODE      DRV_WIFI_SECURITY_WPS_PIN
#define DRV_WIFI_DEFAULT_WEP_PHRASE         "WEP Phrase" // default WEP passphrase
#define DRV_WIFI_DEFAULT_WEP_KEY_40         "5AFB6C8E77" // default WEP40 key
#define DRV_WIFI_DEFAULT_WEP_KEY_104        "90E96780C739409DA50034FCAA" // default WEP104 key
#define DRV_WIFI_DEFAULT_PSK_PHRASE         "Microchip 802.11 Secret PSK Password" // default WPA passphrase
#define DRV_WIFI_DEFAULT_WPS_PIN            "${CONFIG_DRV_WIFI_WPS_PIN}" // customized WPS PIN
</#if><#-- CONFIG_DRV_WIFI_SECURITY_MODE_INFRASTRUCTURE -->

<#if CONFIG_DRV_WIFI_SECURITY_MODE_INFRASTRUCTURE == "WPS Push Button" || CONFIG_DRV_WIFI_SECURITY_MODE_INFRASTRUCTURE == "WPS PIN">
#define DRV_WIFI_SAVE_WPS_CREDENTIALS       DRV_WIFI_ENABLED
<#else>
#define DRV_WIFI_SAVE_WPS_CREDENTIALS       DRV_WIFI_DISABLED
</#if><#-- CONFIG_DRV_WIFI_SECURITY_MODE_INFRASTRUCTURE -->

#define DRV_WIFI_CHECK_LINK_STATUS          DRV_WIFI_DISABLED /* Gets the MRF to check the link status relying on Tx failures. */
#define DRV_WIFI_LINK_LOST_THRESHOLD        40 /* Consecutive Tx transmission failures to be considered the AP is gone away. */

/* 
 * MRF24W FW has a built-in connection manager, and it is enabled by default.
 * If you want to run your own connection manager in host side, you should
 * disable the FW connection manager to avoid possible conflict between the two.
 * Especially these two APIs can be affected if you do not disable it.
 * A) uint16_t DRV_WIFI_Disconnect(void)
 * B) uint16_t DRV_WIFI_Scan(bool scanAll)
 * These APIs will return failure when the conflict occurs.
 */
#define DRV_WIFI_MODULE_CONNECTION_MANAGER  DRV_WIFI_ENABLED

#define DRV_WIFI_DEFAULT_POWER_SAVE         DRV_WIFI_DISABLED /* DRV_WIFI_ENABLED or DRV_WIFI_DISABLED */
#define DRV_WIFI_SOFTWARE_MULTICAST_FILTER  DRV_WIFI_ENABLED
<#elseif CONFIG_DRV_WIFI_NETWORK_TYPE == "Ad-Hoc">
#define DRV_WIFI_DEFAULT_NETWORK_TYPE       DRV_WIFI_NETWORK_TYPE_ADHOC
#define DRV_WIFI_DEFAULT_SSID               "${CONFIG_DRV_WIFI_SSID_NAME}"
#define DRV_WIFI_DEFAULT_LIST_RETRY_COUNT   (DRV_WIFI_RETRY_ADHOC) /* Number (1..254) of times to try to connect to the SSID when using Ad/Hoc network type */
#define DRV_WIFI_DEFAULT_CHANNEL_LIST       {${CONFIG_DRV_WIFI_OPERATING_CHANNEL_ADHOC_SOFTAP}} /* Set Ad-Hoc network channel */

<#if CONFIG_DRV_WIFI_SECURITY_MODE_ADHOC_SOFTAP == "Open">
#define DRV_WIFI_DEFAULT_SECURITY_MODE      DRV_WIFI_SECURITY_OPEN
#define DRV_WIFI_DEFAULT_WEP_PHRASE         "WEP Phrase" // default WEP passphrase
#define DRV_WIFI_DEFAULT_WEP_KEY_40         "5AFB6C8E77" // default WEP40 key
#define DRV_WIFI_DEFAULT_WEP_KEY_104        "90E96780C739409DA50034FCAA" // default WEP104 key
#define DRV_WIFI_DEFAULT_PSK_PHRASE         "Microchip 802.11 Secret PSK Password" // default WPA passphrase
#define DRV_WIFI_DEFAULT_WPS_PIN            "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFI_SECURITY_MODE_ADHOC_SOFTAP == "WEP-40 (64-bit)">
#define DRV_WIFI_DEFAULT_SECURITY_MODE      DRV_WIFI_SECURITY_WEP_40
#define DRV_WIFI_DEFAULT_WEP_PHRASE         "${CONFIG_DRV_WIFI_WEP_PASS_PHRASE}" // customized WEP passphrase
#define DRV_WIFI_DEFAULT_WEP_KEY_40         "${CONFIG_DRV_WIFI_WEP_KEY_40}" // customized WEP40 key
#define DRV_WIFI_DEFAULT_WEP_KEY_104        "90E96780C739409DA50034FCAA" // default WEP104 key
#define DRV_WIFI_DEFAULT_PSK_PHRASE         "Microchip 802.11 Secret PSK Password" // default WPA passphrase
#define DRV_WIFI_DEFAULT_WPS_PIN            "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFI_SECURITY_MODE_ADHOC_SOFTAP == "WEP-104 (128-bit)">
#define DRV_WIFI_DEFAULT_SECURITY_MODE      DRV_WIFI_SECURITY_WEP_104
#define DRV_WIFI_DEFAULT_WEP_PHRASE         "${CONFIG_DRV_WIFI_WEP_PASS_PHRASE}" // customized WEP passphrase
#define DRV_WIFI_DEFAULT_WEP_KEY_40         "5AFB6C8E77" // default WEP40 key
#define DRV_WIFI_DEFAULT_WEP_KEY_104        "${CONFIG_DRV_WIFI_WEP_KEY_104}" // customized WEP104 key
#define DRV_WIFI_DEFAULT_PSK_PHRASE         "Microchip 802.11 Secret PSK Password" // default WPA passphrase
#define DRV_WIFI_DEFAULT_WPS_PIN            "12390212" // default WPS PIN
</#if><#-- CONFIG_DRV_WIFI_SECURITY_MODE_ADHOC_SOFTAP -->

#define DRV_WIFI_SAVE_WPS_CREDENTIALS       DRV_WIFI_DISABLED

#define DRV_WIFI_DEFAULT_ADHOC_PRESCAN      DRV_WIFI_DISABLED

#define DRV_WIFI_CHECK_LINK_STATUS          DRV_WIFI_DISABLED /* Gets the MRF to check the link status relying on Tx failures. */
#define DRV_WIFI_LINK_LOST_THRESHOLD        40 /* Consecutive Tx transmission failures to be considered the AP is gone away. */

/* 
 * MRF24W FW has a built-in connection manager, and it is enabled by default.
 * If you want to run your own connection manager in host side, you should
 * disable the FW connection manager to avoid possible conflict between the two.
 * Especially these two APIs can be affected if you do not disable it.
 * A) uint16_t DRV_WIFI_Disconnect(void)
 * B) uint16_t DRV_WIFI_Scan(bool scanAll)
 * These APIs will return failure when the conflict occurs.
 */
#define DRV_WIFI_MODULE_CONNECTION_MANAGER  DRV_WIFI_ENABLED

#define DRV_WIFI_DEFAULT_POWER_SAVE         DRV_WIFI_DISABLED /* PS_POLL not supported in Ad-Hoc - must be set to DRV_WIFI_DISABLED */
#define DRV_WIFI_SOFTWARE_MULTICAST_FILTER  DRV_WIFI_ENABLED

#define DRV_WIFI_ENABLE_STATIC_IP
<#elseif CONFIG_DRV_WIFI_NETWORK_TYPE == "Soft AP">
#define DRV_WIFI_DEFAULT_NETWORK_TYPE       DRV_WIFI_NETWORK_TYPE_SOFT_AP
#define DRV_WIFI_DEFAULT_SSID               "${CONFIG_DRV_WIFI_SSID_NAME}"
#define DRV_WIFI_DEFAULT_LIST_RETRY_COUNT   (DRV_WIFI_RETRY_ADHOC) /* Dummy, not used */
#define DRV_WIFI_DEFAULT_CHANNEL_LIST       {${CONFIG_DRV_WIFI_OPERATING_CHANNEL_ADHOC_SOFTAP}} /* Set Soft AP network channel */

<#if CONFIG_DRV_WIFI_SECURITY_MODE_ADHOC_SOFTAP == "Open">
#define DRV_WIFI_DEFAULT_SECURITY_MODE      DRV_WIFI_SECURITY_OPEN
#define DRV_WIFI_DEFAULT_WEP_PHRASE         "WEP Phrase" // default WEP passphrase
#define DRV_WIFI_DEFAULT_WEP_KEY_40         "5AFB6C8E77" // default WEP40 key
#define DRV_WIFI_DEFAULT_WEP_KEY_104        "90E96780C739409DA50034FCAA" // default WEP104 key
#define DRV_WIFI_DEFAULT_PSK_PHRASE         "Microchip 802.11 Secret PSK Password" // default WPA passphrase
#define DRV_WIFI_DEFAULT_WPS_PIN            "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFI_SECURITY_MODE_ADHOC_SOFTAP == "WEP-40 (64-bit)">
#define DRV_WIFI_DEFAULT_SECURITY_MODE      DRV_WIFI_SECURITY_WEP_40
#define DRV_WIFI_DEFAULT_WEP_PHRASE         "${CONFIG_DRV_WIFI_WEP_PASS_PHRASE}" // customized WEP passphrase
#define DRV_WIFI_DEFAULT_WEP_KEY_40         "${CONFIG_DRV_WIFI_WEP_KEY_40}" // customized WEP40 key
#define DRV_WIFI_DEFAULT_WEP_KEY_104        "90E96780C739409DA50034FCAA" // default WEP104 key
#define DRV_WIFI_DEFAULT_PSK_PHRASE         "Microchip 802.11 Secret PSK Password" // default WPA passphrase
#define DRV_WIFI_DEFAULT_WPS_PIN            "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFI_SECURITY_MODE_ADHOC_SOFTAP == "WEP-104 (128-bit)">
#define DRV_WIFI_DEFAULT_SECURITY_MODE      DRV_WIFI_SECURITY_WEP_104
#define DRV_WIFI_DEFAULT_WEP_PHRASE         "${CONFIG_DRV_WIFI_WEP_PASS_PHRASE}" // customized WEP passphrase
#define DRV_WIFI_DEFAULT_WEP_KEY_40         "5AFB6C8E77" // default WEP40 key
#define DRV_WIFI_DEFAULT_WEP_KEY_104        "${CONFIG_DRV_WIFI_WEP_KEY_104}" // customized WEP104 key
#define DRV_WIFI_DEFAULT_PSK_PHRASE         "Microchip 802.11 Secret PSK Password" // default WPA passphrase
#define DRV_WIFI_DEFAULT_WPS_PIN            "12390212" // default WPS PIN
</#if><#-- CONFIG_DRV_WIFI_SECURITY_MODE_ADHOC_SOFTAP -->

#define DRV_WIFI_SAVE_WPS_CREDENTIALS       DRV_WIFI_DISABLED

#define DRV_WIFI_CHECK_LINK_STATUS          DRV_WIFI_DISABLED /* Gets the MRF to check the link status relying on Tx failures. */
#define DRV_WIFI_LINK_LOST_THRESHOLD        40 /* Consecutive Tx transmission failures to be considered the AP is gone away. */
#define DRV_WIFI_SOFTAP_SEND_KEEP_ALIVE     DRV_WIFI_DISABLED /* Gets Soft AP to send keep alive packets to clients. */
#define DRV_WIFI_SOFTAP_LINK_LOST_THRESHOLD 40 /* Consecutive null packet transmission failures to be considered the client STA is gone away. */

/* 
 * MRF24W FW has a built-in connection manager, and it is enabled by default.
 * If you want to run your own connection manager in host side, you should
 * disable the FW connection manager to avoid possible conflict between the two.
 * Especially these two APIs can be affected if you do not disable it.
 * A) uint16_t DRV_WIFI_Disconnect(void)
 * B) uint16_t DRV_WIFI_Scan(bool scanAll)
 * These APIs will return failure when the conflict occurs.
 */
#define DRV_WIFI_MODULE_CONNECTION_MANAGER  DRV_WIFI_ENABLED

#define DRV_WIFI_DEFAULT_POWER_SAVE         DRV_WIFI_DISABLED /* PS_POLL not supported in Soft AP - must be set to DRV_WIFI_DISABLED */
#define DRV_WIFI_SOFTWARE_MULTICAST_FILTER  DRV_WIFI_ENABLED

#define DRV_WIFI_ENABLE_STATIC_IP
</#if><#-- CONFIG_DRV_WIFI_NETWORK_TYPE -->

</#if><#-- CONFIG_DRV_WIFI_DEVICE == "MRF24WG" -->

<#if CONFIG_DRV_WIFI_DEVICE == "MRF24WN">

#define MODULE_EVENT_PRINT 0
#define PIN_MODE_PER_PORT_SELECT

#define WDRV_EXT_RTOS_INIT_TASK_SIZE ${CONFIG_DRV_WIFIN_RTOS_INIT_TASK_SIZE}u
#define WDRV_EXT_RTOS_INIT_TASK_PRIORITY ${CONFIG_DRV_WIFIN_RTOS_INIT_TASK_PRIORITY}u
#define WDRV_EXT_RTOS_MAIN_TASK_SIZE ${CONFIG_DRV_WIFIN_RTOS_MAIN_TASK_SIZE}u
#define WDRV_EXT_RTOS_MAIN_TASK_PRIORITY ${CONFIG_DRV_WIFIN_RTOS_MAIN_TASK_PRIORITY}u

#define WDRV_ASSERT(condition, msg) WDRV_Assert(condition, msg, __FILE__, __LINE__)

<#if CONFIG_DRV_WIFI_SPI_INSTANCE_INDEX == '0'>
#define WDRV_SPI_INDEX 0
#define WDRV_SPI_INSTANCE sysObj.spiObjectIdx0
<#elseif CONFIG_DRV_WIFI_SPI_INSTANCE_INDEX == '1'>
#define WDRV_SPI_INDEX 1
#define WDRV_SPI_INSTANCE sysObj.spiObjectIdx1
<#elseif CONFIG_DRV_WIFI_SPI_INSTANCE_INDEX == '2'>
#define WDRV_SPI_INDEX 2
#define WDRV_SPI_INSTANCE sysObj.spiObjectIdx2
<#elseif CONFIG_DRV_WIFI_SPI_INSTANCE_INDEX == '3'>
#define WDRV_SPI_INDEX 3
#define WDRV_SPI_INSTANCE sysObj.spiObjectIdx3
<#elseif CONFIG_DRV_WIFI_SPI_INSTANCE_INDEX == '4'>
#define WDRV_SPI_INDEX 4
#define WDRV_SPI_INSTANCE sysObj.spiObjectIdx4
<#elseif CONFIG_DRV_WIFI_SPI_INSTANCE_INDEX == '5'>
#define WDRV_SPI_INDEX 5
#define WDRV_SPI_INSTANCE sysObj.spiObjectIdx5
</#if>

<#if CONFIG_DRV_WIFI_USE_SPI_DMA == true>
#define WDRV_USE_SPI_DMA
</#if><#-- CONFIG_DRV_WIFI_USE_SPI_DMA -->

<#if CONFIG_DRV_WIFI_STORE_CONFIG_IN_NVM>
#define WDRV_NVM_SPACE_ENABLE
#define WDRV_NVM_SPACE_ADDR (${CONFIG_DRV_WIFI_NVM_START_ADDRESS} * 1024)
</#if>

<#if CONFIG_BSP_PIC32MX795_PIM_E16 == true>
#define EX16 // used by application

#define WDRV_BOARD_TYPE WDRV_BD_TYPE_EXP16 // used by Wi-Fi driver
<#elseif CONFIG_BSP_PIC32MX_ETH_SK == true>
#define MRF24WN_USE_CN_INT

#define WDRV_BOARD_TYPE WDRV_BD_TYPE_MX_ESK

// For CN9(GPIO_G7) as extern interrupt, it is better to use another GPIO (GPIO_E0) to read interrupt high or low.
// We need a jump line to connect GPIO_E0 to GPIO_G7.
//#define WF_VERIFY_EINT_BY_ANOTHER_GPIO
#if defined(WF_VERIFY_EINT_BY_ANOTHER_GPIO)
// Use GPIO_E0. Please use a jump line to connect GPIO_E0 to GPIO_G7. 
#define WF_INT_PORT_CHANNEL_READ PORT_CHANNEL_E
#define WF_INT_BIT_POS_READ      0
#else
// Still read GPIO_G7.
#define WF_INT_PORT_CHANNEL_READ PORT_CHANNEL_G
#define WF_INT_BIT_POS_READ      7
#endif
<#elseif CONFIG_BSP_PIC32MZ_EC_SK == true>
#define WDRV_BOARD_TYPE WDRV_BD_TYPE_MZ_ESK
<#elseif CONFIG_BSP_PIC32MZ_EC_SK_MEB2 == true>
#define WDRV_BOARD_TYPE WDRV_BD_TYPE_MEB2
<#else>
#define WDRV_BOARD_TYPE WDRV_BD_TYPE_CUSTOM
</#if><#-- CONFIG_BSP -->

// I/O mappings for general control pins, including CS, HIBERNATE, RESET and INTERRUPT.
#define WF_CS_PORT_CHANNEL PORT_CHANNEL_${CONFIG_DRV_WIFI_CS_PORT_CHANNEL}
#define WF_CS_BIT_POS      ${CONFIG_DRV_WIFI_CS_BIT_POS}

#define WF_HIBERNATE_PORT_CHANNEL PORT_CHANNEL_${CONFIG_DRV_WIFI_HIBERNATE_PORT_CHANNEL}
#define WF_HIBERNATE_BIT_POS      ${CONFIG_DRV_WIFI_HIBERNATE_BIT_POS}

#define WF_RESET_PORT_CHANNEL PORT_CHANNEL_${CONFIG_DRV_WIFI_RESET_PORT_CHANNEL}
#define WF_RESET_BIT_POS      ${CONFIG_DRV_WIFI_RESET_BIT_POS}

#define WF_INT_PORT_CHANNEL PORT_CHANNEL_${CONFIG_DRV_WIFI_INT_PORT_CHANNEL}
#define WF_INT_BIT_POS      ${CONFIG_DRV_WIFI_INT_BIT_POS}

<#if CONFIG_DRV_WIFI_INTERRUPT_SYSTEM_SERVICE == "External Interrupt">
 <#if CONFIG_EXT_INT_INST_IDX0_USED_BY_DRV_WIFI>
#define MRF_INT_SOURCE ${CONFIG_EXT_INT_INTERRUPT_SOURCE_IDX0}
#define MRF_INT_VECTOR ${CONFIG_EXT_INT_INTERRUPT_VECTOR_IDX0}
 <#elseif CONFIG_EXT_INT_INST_IDX1_USED_BY_DRV_WIFI>
#define MRF_INT_SOURCE ${CONFIG_EXT_INT_INTERRUPT_SOURCE_IDX1}
#define MRF_INT_VECTOR ${CONFIG_EXT_INT_INTERRUPT_VECTOR_IDX1}
 <#elseif CONFIG_EXT_INT_INST_IDX2_USED_BY_DRV_WIFI>
#define MRF_INT_SOURCE ${CONFIG_EXT_INT_INTERRUPT_SOURCE_IDX2}
#define MRF_INT_VECTOR ${CONFIG_EXT_INT_INTERRUPT_VECTOR_IDX2}
 <#elseif CONFIG_EXT_INT_INST_IDX3_USED_BY_DRV_WIFI>
#define MRF_INT_SOURCE ${CONFIG_EXT_INT_INTERRUPT_SOURCE_IDX3}
#define MRF_INT_VECTOR ${CONFIG_EXT_INT_INTERRUPT_VECTOR_IDX3}
 <#elseif CONFIG_EXT_INT_INST_IDX4_USED_BY_DRV_WIFI>
#define MRF_INT_SOURCE ${CONFIG_EXT_INT_INTERRUPT_SOURCE_IDX4}
#define MRF_INT_VECTOR ${CONFIG_EXT_INT_INTERRUPT_VECTOR_IDX4}
 <#else>
#error "Please select an external interrupt instance for Wi-Fi driver."
 </#if><#-- Wi-Fi Driver External Interrupt Instance -->
<#elseif CONFIG_DRV_WIFI_INTERRUPT_SYSTEM_SERVICE == "Change Notification">
#define MRF_INT_SOURCE INT_SOURCE_CHANGE_NOTICE
#define MRF_INT_VECTOR INT_VECTOR_CN
</#if><#-- CONFIG_DRV_WIFI_INTERRUPT_SYSTEM_SERVICE -->

<#if CONFIG_DRV_WIFIN_NETWORK_TYPE == "Infrastructure">
#define WDRV_DEFAULT_NETWORK_TYPE WDRV_NETWORK_TYPE_INFRASTRUCTURE
<#if CONFIG_DRV_WIFIN_SECURITY_MODE_INFRASTRUCTURE == "WPS Push Button">
#define WDRV_DEFAULT_SSID ""
<#else>
#define WDRV_DEFAULT_SSID "${CONFIG_DRV_WIFIN_SSID_NAME}"
</#if>

<#if CONFIG_DRV_WIFIN_SECURITY_MODE_INFRASTRUCTURE == "Open">
#define WDRV_DEFAULT_SECURITY_MODE WDRV_SECURITY_OPEN
#define WDRV_DEFAULT_WEP_KEYS_40 "5AFB6C8E77" // default WEP40 key
#define WDRV_DEFAULT_WEP_KEYS_104 "90E96780C739409DA50034FCAA" // default WEP104 key
#define WDRV_DEFAULT_PSK_PHRASE "Microchip 802.11 Secret PSK Password" // default WPA-PSK or WPA2-PSK passphrase
#define WDRV_DEFAULT_WPS_PIN "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFIN_SECURITY_MODE_INFRASTRUCTURE == "WEP-40 (64-bit)">
#define WDRV_DEFAULT_SECURITY_MODE WDRV_SECURITY_WEP_40
#define WDRV_DEFAULT_WEP_KEYS_40 "${CONFIG_DRV_WIFIN_WEP_KEY_40}" // customized WEP40 key
#define WDRV_DEFAULT_WEP_KEYS_104 "90E96780C739409DA50034FCAA" // default WEP104 key
#define WDRV_DEFAULT_PSK_PHRASE "Microchip 802.11 Secret PSK Password" // default WPA-PSK or WPA2-PSK passphrase
#define WDRV_DEFAULT_WPS_PIN "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFIN_SECURITY_MODE_INFRASTRUCTURE == "WEP-104 (128-bit)">
#define WDRV_DEFAULT_SECURITY_MODE WDRV_SECURITY_WEP_104
#define WDRV_DEFAULT_WEP_KEYS_40 "5AFB6C8E77" // default WEP40 key
#define WDRV_DEFAULT_WEP_KEYS_104 "${CONFIG_DRV_WIFIN_WEP_KEY_104}" // customized WEP104 key
#define WDRV_DEFAULT_PSK_PHRASE "Microchip 802.11 Secret PSK Password" // default WPA-PSK or WPA2-PSK passphrase
#define WDRV_DEFAULT_WPS_PIN "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFIN_SECURITY_MODE_INFRASTRUCTURE == "WPA-PSK">
#define WDRV_DEFAULT_SECURITY_MODE WDRV_SECURITY_WPA_WITH_PASS_PHRASE
#define WDRV_DEFAULT_WEP_KEYS_40 "5AFB6C8E77" // default WEP40 key
#define WDRV_DEFAULT_WEP_KEYS_104 "90E96780C739409DA50034FCAA" // default WEP104 key
#define WDRV_DEFAULT_PSK_PHRASE "${CONFIG_DRV_WIFIN_WPA_PASS_PHRASE}" // customized WPA-PSK or WPA2-PSK passphrase
#define WDRV_DEFAULT_WPS_PIN "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFIN_SECURITY_MODE_INFRASTRUCTURE == "WPA2-PSK">
#define WDRV_DEFAULT_SECURITY_MODE WDRV_SECURITY_WPA2_WITH_PASS_PHRASE
#define WDRV_DEFAULT_WEP_KEYS_40 "5AFB6C8E77" // default WEP40 key
#define WDRV_DEFAULT_WEP_KEYS_104 "90E96780C739409DA50034FCAA" // default WEP104 key
#define WDRV_DEFAULT_PSK_PHRASE "${CONFIG_DRV_WIFIN_WPA_PASS_PHRASE}" // customized WPA-PSK or WPA2-PSK passphrase
#define WDRV_DEFAULT_WPS_PIN "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFIN_SECURITY_MODE_INFRASTRUCTURE == "WPS Push Button">
#define WDRV_DEFAULT_SECURITY_MODE WDRV_SECURITY_WPS_PUSH_BUTTON
#define WDRV_DEFAULT_WEP_KEYS_40 "5AFB6C8E77" // default WEP40 key
#define WDRV_DEFAULT_WEP_KEYS_104 "90E96780C739409DA50034FCAA" // default WEP104 key
#define WDRV_DEFAULT_PSK_PHRASE "Microchip 802.11 Secret PSK Password" // default WPA-PSK or WPA2-PSK passphrase
#define WDRV_DEFAULT_WPS_PIN "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFIN_SECURITY_MODE_INFRASTRUCTURE == "WPS PIN">
#define WDRV_DEFAULT_SECURITY_MODE WDRV_SECURITY_WPS_PIN
#define WDRV_DEFAULT_WEP_KEYS_40 "5AFB6C8E77" // default WEP40 key
#define WDRV_DEFAULT_WEP_KEYS_104 "90E96780C739409DA50034FCAA" // default WEP104 key
#define WDRV_DEFAULT_PSK_PHRASE "Microchip 802.11 Secret PSK Password" // default WPA-PSK or WPA2-PSK passphrase
#define WDRV_DEFAULT_WPS_PIN "${CONFIG_DRV_WIFIN_WPS_PIN}" // customized WPS PIN
</#if><#-- CONFIG_DRV_WIFIN_SECURITY_MODE_INFRASTRUCTURE -->

#define WDRV_DEFAULT_CHANNEL 6
#define WDRV_DEFAULT_POWER_SAVE WDRV_FUNC_DISABLED
</#if><#-- CONFIG_DRV_WIFIN_NETWORK_TYPE -->

<#if CONFIG_DRV_WIFIN_NETWORK_TYPE == "Soft AP" >
#define WDRV_DEFAULT_NETWORK_TYPE WDRV_NETWORK_TYPE_SOFT_AP
#define WDRV_DEFAULT_SSID "${CONFIG_DRV_WIFIN_SSID_NAME}"

<#if CONFIG_DRV_WIFIN_SECURITY_MODE_SOFTAP == "Open">
#define WDRV_DEFAULT_SECURITY_MODE WDRV_SECURITY_OPEN
#define WDRV_DEFAULT_WEP_KEYS_40 "5AFB6C8E77" // default WEP40 key
#define WDRV_DEFAULT_WEP_KEYS_104 "90E96780C739409DA50034FCAA" // default WEP104 key
#define WDRV_DEFAULT_PSK_PHRASE "Microchip 802.11 Secret PSK Password" // default WPA-PSK or WPA2-PSK passphrase
#define WDRV_DEFAULT_WPS_PIN "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFIN_SECURITY_MODE_SOFTAP == "WEP-40 (64-bit)">
#define WDRV_DEFAULT_SECURITY_MODE WDRV_SECURITY_WEP_40
#define WDRV_DEFAULT_WEP_KEYS_40 "${CONFIG_DRV_WIFIN_WEP_KEY_40}" // customized WEP40 key
#define WDRV_DEFAULT_WEP_KEYS_104 "90E96780C739409DA50034FCAA" // default WEP104 key
#define WDRV_DEFAULT_PSK_PHRASE "Microchip 802.11 Secret PSK Password" // default WPA-PSK or WPA2-PSK passphrase
#define WDRV_DEFAULT_WPS_PIN "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFIN_SECURITY_MODE_SOFTAP == "WEP-104 (128-bit)">
#define WDRV_DEFAULT_SECURITY_MODE WDRV_SECURITY_WEP_104
#define WDRV_DEFAULT_WEP_KEYS_40 "5AFB6C8E77" // default WEP40 key
#define WDRV_DEFAULT_WEP_KEYS_104 "${CONFIG_DRV_WIFIN_WEP_KEY_104}" // customized WEP104 key
#define WDRV_DEFAULT_PSK_PHRASE "Microchip 802.11 Secret PSK Password" // default WPA-PSK or WPA2-PSK passphrase
#define WDRV_DEFAULT_WPS_PIN "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFIN_SECURITY_MODE_SOFTAP == "WPA-PSK">
#define WDRV_DEFAULT_SECURITY_MODE WDRV_SECURITY_WPA_WITH_PASS_PHRASE
#define WDRV_DEFAULT_WEP_KEYS_40 "5AFB6C8E77" // default WEP40 key
#define WDRV_DEFAULT_WEP_KEYS_104 "90E96780C739409DA50034FCAA" // default WEP104 key
#define WDRV_DEFAULT_PSK_PHRASE "${CONFIG_DRV_WIFIN_WPA_PASS_PHRASE}" // customized WPA-PSK or WPA2-PSK passphrase
#define WDRV_DEFAULT_WPS_PIN "12390212" // default WPS PIN
<#elseif CONFIG_DRV_WIFIN_SECURITY_MODE_SOFTAP == "WPA2-PSK">
#define WDRV_DEFAULT_SECURITY_MODE WDRV_SECURITY_WPA2_WITH_PASS_PHRASE
#define WDRV_DEFAULT_WEP_KEYS_40 "5AFB6C8E77" // default WEP40 key
#define WDRV_DEFAULT_WEP_KEYS_104 "90E96780C739409DA50034FCAA" // default WEP104 key
#define WDRV_DEFAULT_PSK_PHRASE "${CONFIG_DRV_WIFIN_WPA_PASS_PHRASE}" // customized WPA-PSK or WPA2-PSK passphrase
#define WDRV_DEFAULT_WPS_PIN "12390212" // default WPS PIN
</#if><#-- CONFIG_DRV_WIFIN_SECURITY_MODE_SOFTAP -->

#define WDRV_DEFAULT_CHANNEL ${CONFIG_DRV_WIFIN_OPERATING_CHANNEL_SOFTAP}
#define WDRV_DEFAULT_POWER_SAVE WDRV_FUNC_DISABLED
</#if><#-- CONFIG_DRV_WIFIN_NETWORK_TYPE -->

</#if><#-- CONFIG_DRV_WIFI_DEVICE == "MRF24WN" -->
<#--
/*******************************************************************************
 End of File
 */
-->
