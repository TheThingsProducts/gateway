<#--
/*******************************************************************************
  Wi-Fi Driver Initialization File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_wifi_init.c.ftl

  Summary:
    This file contains source code necessary to initialize the Wi-Fi driver.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, configuration bits, and allocates
    any necessary global system resources, such as the sysObj structure that
    contains the object handles to all the MPLAB Harmony module objects in
    the system.
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

<#if CONFIG_DRV_WIFI_DEVICE == "MRF24WG">
 <#if CONFIG_BSP_PIC32MX_ETH_SK == true>
#if defined(MRF24W_USE_CN_INT)
    // If using CN9 interrupt, we should use a jump line to use another GPIO to read CN9 pin status
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,
                                 SYS_PORTS_DIRECTION_INPUT,
                                 WF_INT_PORT_CHANNEL_READ,
                                 WF_INT_BIT_POS_READ);
#endif
 </#if><#-- CONFIG_BSP_PIC32MX_ETH_SK == true -->
</#if><#-- CONFIG_DRV_WIFI_DEVICE == "MRF24WG" -->
<#if CONFIG_DRV_WIFI_DEVICE == "MRF24WN">
 <#if CONFIG_BSP_PIC32MX_ETH_SK == true>
#if defined(MRF24WN_USE_CN_INT)
    // If using CN9 interrupt, we should use a jump line to use another GPIO to read CN9 pin status
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,
                                 SYS_PORTS_DIRECTION_INPUT,
                                 WF_INT_PORT_CHANNEL_READ,
                                 WF_INT_BIT_POS_READ);
#endif
 </#if><#-- CONFIG_BSP_PIC32MX_ETH_SK == true -->
</#if><#-- CONFIG_DRV_WIFI_DEVICE == "MRF24WN" -->
<#--
/*******************************************************************************
 End of File
 */
-->
