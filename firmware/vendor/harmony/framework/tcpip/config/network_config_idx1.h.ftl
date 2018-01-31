<#--
/*******************************************************************************
  network_config_IDX1 Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    network_config_IDX1.h.ftl

  Summary:
    network_config_IDX1 Freemarker Template File

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
<#if CONFIG_TCPIP_STACK_NETWORK_CONFIG_IDX1>

/*** Network Configuration Index 0 ***/
#define TCPIP_NETWORK_DEFAULT_INTERFACE_NAME_IDX1 		"${CONFIG_TCPIP_NETWORK_DEFAULT_INTERFACE_NAME_IDX1}"
<#if CONFIG_TCPIP_NETWORK_DEFAULT_INTERFACE_NAME_IDX1 == "PIC32INT">
#define TCPIP_IF_PIC32INT
<#elseif CONFIG_TCPIP_NETWORK_DEFAULT_INTERFACE_NAME_IDX1 = "MRF24W">
#define TCPIP_IF_MRF24W
<#elseif CONFIG_TCPIP_NETWORK_DEFAULT_INTERFACE_NAME_IDX1 = "MRF24WN">
#define TCPIP_IF_MRF24WN
</#if>
#define TCPIP_NETWORK_DEFAULT_HOST_NAME_IDX1 			"${CONFIG_TCPIP_NETWORK_DEFAULT_HOST_NAME_IDX1}"
<#if CONFIG_TCPIP_NETWORK_DEFAULT_MAC_ADDR_IDX1?has_content>
#define TCPIP_NETWORK_DEFAULT_MAC_ADDR_IDX1 			"${CONFIG_TCPIP_NETWORK_DEFAULT_MAC_ADDR_IDX1}"
<#else>
#define TCPIP_NETWORK_DEFAULT_MAC_ADDR_IDX1 			0
</#if>
#define TCPIP_NETWORK_DEFAULT_IP_ADDRESS_IDX1 			"${CONFIG_TCPIP_NETWORK_DEFAULT_IP_ADDRESS_IDX1}"
#define TCPIP_NETWORK_DEFAULT_IP_MASK_IDX1 			"${CONFIG_TCPIP_NETWORK_DEFAULT_IP_MASK_IDX1}"
#define TCPIP_NETWORK_DEFAULT_GATEWAY_IDX1 			"${CONFIG_TCPIP_NETWORK_DEFAULT_GATEWAY_IDX1}"
#define TCPIP_NETWORK_DEFAULT_DNS_IDX1 				"${CONFIG_TCPIP_NETWORK_DEFAULT_DNS_IDX1}"
#define TCPIP_NETWORK_DEFAULT_SECOND_DNS_IDX1 			"${CONFIG_TCPIP_NETWORK_DEFAULT_SECOND_DNS_IDX1}"
#define TCPIP_NETWORK_DEFAULT_POWER_MODE_IDX1 			"${CONFIG_TCPIP_NETWORK_DEFAULT_POWER_MODE_IDX1}"
#define TCPIP_NETWORK_DEFAULT_INTERFACE_FLAGS_IDX1      \
<#if CONFIG_TCPIP_NETWORK_INTERFACE_FLAG_DHCP_CLIENT_IDX1>
                                                    TCPIP_NETWORK_CONFIG_DHCP_CLIENT_ON |\
</#if>
<#if CONFIG_TCPIP_NETWORK_INTERFACE_FLAG_ZCLL_IDX1>
                                                    TCPIP_NETWORK_CONFIG_ZCLL_ON |\
</#if>
<#if CONFIG_TCPIP_NETWORK_INTERFACE_FLAG_DHCP_SERVER_IDX1>
                                                    TCPIP_NETWORK_CONFIG_DHCP_SERVER_ON |\
</#if>
<#if CONFIG_TCPIP_NETWORK_INTERFACE_FLAG_DNS_CLIENT_IDX1>
                                                    TCPIP_NETWORK_CONFIG_DNS_CLIENT_ON |\
</#if>
<#if CONFIG_TCPIP_NETWORK_INTERFACE_FLAG_DNS_SERVER_IDX1>
                                                    TCPIP_NETWORK_CONFIG_DNS_SERVER_ON |\
</#if>
<#if CONFIG_TCPIP_NETWORK_INTERFACE_FLAG_IPV6_ADDRESS_IDX1>
                                                    TCPIP_NETWORK_CONFIG_IPV6_ADDRESS |\
</#if>
                                                    TCPIP_NETWORK_CONFIG_IP_STATIC
#define TCPIP_NETWORK_DEFAULT_MAC_DRIVER_IDX1 		${CONFIG_TCPIP_NETWORK_DEFAULT_MAC_DRIVER_IDX1}
<#if CONFIG_TCPIP_NETWORK_DEFAULT_IPV6_ADDRESS_IDX1?has_content>
#define TCPIP_NETWORK_DEFAULT_IPV6_ADDRESS_IDX1     "${CONFIG_TCPIP_NETWORK_DEFAULT_IPV6_ADDRESS_IDX1}"
<#else>
#define TCPIP_NETWORK_DEFAULT_IPV6_ADDRESS_IDX1     0
</#if>
#define TCPIP_NETWORK_DEFAULT_IPV6_PREFIX_LENGTH_IDX1   ${CONFIG_TCPIP_NETWORK_DEFAULT_IPV6_PREFIX_LENGTH_IDX1}
<#if CONFIG_TCPIP_NETWORK_DEFAULT_IPV6_GATEWAY_IDX1?has_content>
#define TCPIP_NETWORK_DEFAULT_IPV6_GATEWAY_IDX1 		"${CONFIG_TCPIP_NETWORK_DEFAULT_IPV6_GATEWAY_IDX1}"
<#else>
#define TCPIP_NETWORK_DEFAULT_IPV6_GATEWAY_IDX1 		0
</#if>
</#if>
