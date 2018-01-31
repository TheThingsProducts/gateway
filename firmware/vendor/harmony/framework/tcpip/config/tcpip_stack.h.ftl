<#--
/*******************************************************************************
  TCPIP System Config Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    tcpip_stack.h.ftl

  Summary:
    TCPIP System Config Freemarker Template File

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

// *****************************************************************************
// *****************************************************************************
// Section: TCPIP Stack Configuration
// *****************************************************************************
// *****************************************************************************
<#if CONFIG_USE_TCPIP_STACK == true>
<#if CONFIG_TCPIP_STACK_USE_IPV4 == true>
#define TCPIP_STACK_USE_IPV4
</#if>
<#if CONFIG_TCPIP_STACK_USE_IPV6 == true>
#define TCPIP_STACK_USE_IPV6
</#if>
<#if CONFIG_TCPIP_USE_TCP == true>
#define TCPIP_STACK_USE_TCP
</#if>
<#if CONFIG_TCPIP_USE_UDP == true>
#define TCPIP_STACK_USE_UDP
</#if>

#define TCPIP_STACK_TICK_RATE		        		${CONFIG_TCPIP_STACK_TICK_RATE}
#define TCPIP_STACK_SECURE_PORT_ENTRIES             ${CONFIG_TCPIP_STACK_SECURE_PORT_ENTRIES}

/* TCP/IP stack event notification */
<#if CONFIG_TCPIP_STACK_EVENT_NOTIFICATION == true>
#define TCPIP_STACK_USE_EVENT_NOTIFICATION
</#if>
<#if CONFIG_TCPIP_STACK_USER_NOTIFICATION == true>
#define TCPIP_STACK_USER_NOTIFICATION   true
<#else>
#define TCPIP_STACK_USER_NOTIFICATION   false
</#if>
<#if CONFIG_TCPIP_STACK_DOWN_OPERATION == false>
#define TCPIP_STACK_DOWN_OPERATION   false
#define TCPIP_STACK_IF_UP_DOWN_OPERATION   false
<#else>
#define TCPIP_STACK_DOWN_OPERATION   true
<#if CONFIG_TCPIP_STACK_IF_UP_DOWN_OPERATION == true>
#define TCPIP_STACK_IF_UP_DOWN_OPERATION   true
<#else>
#define TCPIP_STACK_IF_UP_DOWN_OPERATION   false
</#if>
</#if>
<#if CONFIG_TCPIP_STACK_DOWN_OPERATION == true>
#define TCPIP_STACK_MAC_DOWN_OPERATION  true
<#else>
<#if CONFIG_TCPIP_STACK_MAC_DOWN_OPERATION == true>
#define TCPIP_STACK_MAC_DOWN_OPERATION   true
<#else>
#define TCPIP_STACK_MAC_DOWN_OPERATION   false
</#if>
</#if>
<#if CONFIG_TCPIP_STACK_CONFIGURATION_SAVE_RESTORE == true>
#define TCPIP_STACK_CONFIGURATION_SAVE_RESTORE   true
<#else>
#define TCPIP_STACK_CONFIGURATION_SAVE_RESTORE   false
</#if>
<#include "/framework/tcpip/config/tcpip_heap.h.ftl">
<#include "/framework/tcpip/config/arp.h.ftl">
<#include "/framework/tcpip/config/berkeley_api.h.ftl">
<#include "/framework/tcpip/config/tls.h.ftl">
<#include "/framework/tcpip/config/ddns.h.ftl">
<#include "/framework/tcpip/config/dhcp.h.ftl">
<#include "/framework/tcpip/config/dhcps.h.ftl">
<#include "/framework/tcpip/config/dns.h.ftl">
<#include "/framework/tcpip/config/dnss.h.ftl">
<#include "/framework/tcpip/config/ftp.h.ftl">
<#include "/framework/tcpip/config/http.h.ftl">
<#include "/framework/tcpip/config/http_net.h.ftl">
<#include "/framework/tcpip/config/icmp.h.ftl">
<#include "/framework/tcpip/config/iperf.h.ftl">
<#include "/framework/tcpip/config/ipv6.h.ftl">
<#include "/framework/tcpip/config/nbns.h.ftl">
<#include "/framework/tcpip/config/smtp.h.ftl">
<#include "/framework/tcpip/config/sntp.h.ftl">
<#include "/framework/tcpip/config/snmp.h.ftl">
<#include "/framework/tcpip/config/tcp.h.ftl">
<#include "/framework/tcpip/config/tcpip_announce.h.ftl">
<#include "/framework/tcpip/config/tcpip_mac.h.ftl">
<#include "/framework/tcpip/config/tcpip_reboot.h.ftl">
<#include "/framework/tcpip/config/telnet.h.ftl">
<#include "/framework/tcpip/config/udp.h.ftl">
<#include "/framework/tcpip/config/zeroconf.h.ftl">
<#include "/framework/tcpip/config/network_config_idx0.h.ftl">
<#include "/framework/tcpip/config/network_config_idx1.h.ftl">
<#include "/framework/tcpip/config/tcpip_cmd.h.ftl">
<#include "/framework/tcpip/config/tftpc.h.ftl">
<#if CONFIG_TCPIP_STACK_USE_HTTP_SERVER == true || CONFIG_TCPIP_STACK_USE_HTTP_NET_SERVER == true || CONFIG_TCPIP_USE_SNMP == true || CONFIG_TCPIP_USE_FTP_MODULE == true || CONFIG_TCPIP_USE_TFTPC_MODULE>
<#include "/framework/tcpip/config/sys_fs_wrapper.h.ftl">
</#if>
<#if CONFIG_TCPIP_STACK_USE_HTTP_SERVER == true || CONFIG_TCPIP_STACK_USE_HTTP_NET_SERVER == true || CONFIG_TCPIP_USE_SNMP == true>
<#if CONFIG_BSP_PIC32MX795_PIM_E16 == true>
/* BSP LED Re-directs */
#define APP_TCPIP_LED_1 BSP_LED_3
#define APP_TCPIP_LED_2 BSP_LED_4
#define APP_TCPIP_LED_3 BSP_LED_5

#define APP_TCPIP_SWITCH_1 BSP_SWITCH_4
#define APP_TCPIP_SWITCH_2 BSP_SWITCH_5
#define APP_TCPIP_SWITCH_3 BSP_SWITCH_6
<#elseif CONFIG_BSP_WIFI_G_DB == true>
/* BSP LED Re-directs */
#define APP_TCPIP_LED_1 BSP_LED_0
#define APP_TCPIP_LED_2 BSP_LED_1
#define APP_TCPIP_LED_3 BSP_LED_2

#define APP_TCPIP_SWITCH_1 BSP_SWITCH_0
<#elseif CONFIG_BSP_CHIPKIT_WF32 == true>
/* BSP LED Re-directs */
#define APP_TCPIP_LED_1 BSP_LED_3
#define APP_TCPIP_LED_2 BSP_LED_4
#define APP_TCPIP_LED_3 BSP_LED_5

#define APP_TCPIP_SWITCH_1 BSP_SWITCH_2
#define APP_TCPIP_SWITCH_2 BSP_SWITCH_3
#define APP_TCPIP_SWITCH_3 BSP_SWITCH_2
<#else>
/* BSP LED Re-directs */
#define APP_TCPIP_LED_1 BSP_LED_1
#define APP_TCPIP_LED_2 BSP_LED_2
#define APP_TCPIP_LED_3 BSP_LED_3

#define APP_TCPIP_SWITCH_1 BSP_SWITCH_1
#define APP_TCPIP_SWITCH_2 BSP_SWITCH_2
#define APP_TCPIP_SWITCH_3 BSP_SWITCH_3
</#if>
</#if>
</#if>
