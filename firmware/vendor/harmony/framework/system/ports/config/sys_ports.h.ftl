<#--
/*******************************************************************************
  Ports System Service Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    sys_ports.h.ftl

  Summary:
    Ports System Service Freemarker Template File

  Description:
    This file contains the PORTS system service configuration template.
*******************************************************************************/

/*******************************************************************************
Copyright (c) 2014-2015 released Microchip Technology Inc.  All rights reserved.

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
<#if CONFIG_USE_SYS_PORTS == true>

/*** Ports System Service Configuration ***/
<#if CONFIG_HAVE_PPS == false>
#define SYS_PORT_AD1PCFG        ~${CONFIG_SYS_PORT_ADPCFG}
#define SYS_PORT_CNPUE          ${CONFIG_SYS_PORT_CNPU}
#define SYS_PORT_CNEN           ${CONFIG_SYS_PORT_CNEN}
<#if CONFIG_USE_PORT_A == true>

#define SYS_PORT_A_TRIS         ${CONFIG_SYS_PORT_A_TRIS}
#define SYS_PORT_A_LAT          ${CONFIG_SYS_PORT_A_LAT}
#define SYS_PORT_A_ODC          ${CONFIG_SYS_PORT_A_ODC}
</#if>
<#if CONFIG_USE_PORT_B == true>

#define SYS_PORT_B_TRIS         ${CONFIG_SYS_PORT_B_TRIS}
#define SYS_PORT_B_LAT          ${CONFIG_SYS_PORT_B_LAT}
#define SYS_PORT_B_ODC          ${CONFIG_SYS_PORT_B_ODC}
</#if>
<#if CONFIG_USE_PORT_C == true>

#define SYS_PORT_C_TRIS         ${CONFIG_SYS_PORT_C_TRIS}
#define SYS_PORT_C_LAT          ${CONFIG_SYS_PORT_C_LAT}
#define SYS_PORT_C_ODC          ${CONFIG_SYS_PORT_C_ODC}
</#if>
<#if CONFIG_USE_PORT_D == true>

#define SYS_PORT_D_TRIS         ${CONFIG_SYS_PORT_D_TRIS}
#define SYS_PORT_D_LAT          ${CONFIG_SYS_PORT_D_LAT}
#define SYS_PORT_D_ODC          ${CONFIG_SYS_PORT_D_ODC}
</#if>
<#if CONFIG_USE_PORT_E == true>

#define SYS_PORT_E_TRIS         ${CONFIG_SYS_PORT_E_TRIS}
#define SYS_PORT_E_LAT          ${CONFIG_SYS_PORT_E_LAT}
#define SYS_PORT_E_ODC          ${CONFIG_SYS_PORT_E_ODC}
</#if>
<#if CONFIG_USE_PORT_F == true>

#define SYS_PORT_F_TRIS         ${CONFIG_SYS_PORT_F_TRIS}
#define SYS_PORT_F_LAT          ${CONFIG_SYS_PORT_F_LAT}
#define SYS_PORT_F_ODC          ${CONFIG_SYS_PORT_F_ODC}
</#if>
<#if CONFIG_USE_PORT_G == true>

#define SYS_PORT_G_TRIS         ${CONFIG_SYS_PORT_G_TRIS}
#define SYS_PORT_G_LAT          ${CONFIG_SYS_PORT_G_LAT}
#define SYS_PORT_G_ODC          ${CONFIG_SYS_PORT_G_ODC}
</#if>
<#else>
<#if CONFIG_USE_PORT_A == true>

#define SYS_PORT_A_ANSEL        ${CONFIG_SYS_PORT_A_ANSEL}
#define SYS_PORT_A_TRIS         ${CONFIG_SYS_PORT_A_TRIS}
#define SYS_PORT_A_LAT          ${CONFIG_SYS_PORT_A_LAT}
#define SYS_PORT_A_ODC          ${CONFIG_SYS_PORT_A_ODC}
#define SYS_PORT_A_CNPU         ${CONFIG_SYS_PORT_A_CNPU}
#define SYS_PORT_A_CNPD         ${CONFIG_SYS_PORT_A_CNPD}
#define SYS_PORT_A_CNEN         ${CONFIG_SYS_PORT_A_CNEN}
</#if>
<#if CONFIG_USE_PORT_B == true>

#define SYS_PORT_B_ANSEL        ${CONFIG_SYS_PORT_B_ANSEL}
#define SYS_PORT_B_TRIS         ${CONFIG_SYS_PORT_B_TRIS}
#define SYS_PORT_B_LAT          ${CONFIG_SYS_PORT_B_LAT}
#define SYS_PORT_B_ODC          ${CONFIG_SYS_PORT_B_ODC}
#define SYS_PORT_B_CNPU         ${CONFIG_SYS_PORT_B_CNPU}
#define SYS_PORT_B_CNPD         ${CONFIG_SYS_PORT_B_CNPD}
#define SYS_PORT_B_CNEN         ${CONFIG_SYS_PORT_B_CNEN}
</#if>
<#if CONFIG_USE_PORT_C == true>

#define SYS_PORT_C_ANSEL        ${CONFIG_SYS_PORT_C_ANSEL}
#define SYS_PORT_C_TRIS         ${CONFIG_SYS_PORT_C_TRIS}
#define SYS_PORT_C_LAT          ${CONFIG_SYS_PORT_C_LAT}
#define SYS_PORT_C_ODC          ${CONFIG_SYS_PORT_C_ODC}
#define SYS_PORT_C_CNPU         ${CONFIG_SYS_PORT_C_CNPU}
#define SYS_PORT_C_CNPD         ${CONFIG_SYS_PORT_C_CNPD}
#define SYS_PORT_C_CNEN         ${CONFIG_SYS_PORT_C_CNEN}
</#if>
<#if CONFIG_USE_PORT_D == true>

#define SYS_PORT_D_ANSEL        ${CONFIG_SYS_PORT_D_ANSEL}
#define SYS_PORT_D_TRIS         ${CONFIG_SYS_PORT_D_TRIS}
#define SYS_PORT_D_LAT          ${CONFIG_SYS_PORT_D_LAT}
#define SYS_PORT_D_ODC          ${CONFIG_SYS_PORT_D_ODC}
#define SYS_PORT_D_CNPU         ${CONFIG_SYS_PORT_D_CNPU}
#define SYS_PORT_D_CNPD         ${CONFIG_SYS_PORT_D_CNPD}
#define SYS_PORT_D_CNEN         ${CONFIG_SYS_PORT_D_CNEN}
</#if>
<#if CONFIG_USE_PORT_E == true>

#define SYS_PORT_E_ANSEL        ${CONFIG_SYS_PORT_E_ANSEL}
#define SYS_PORT_E_TRIS         ${CONFIG_SYS_PORT_E_TRIS}
#define SYS_PORT_E_LAT          ${CONFIG_SYS_PORT_E_LAT}
#define SYS_PORT_E_ODC          ${CONFIG_SYS_PORT_E_ODC}
#define SYS_PORT_E_CNPU         ${CONFIG_SYS_PORT_E_CNPU}
#define SYS_PORT_E_CNPD         ${CONFIG_SYS_PORT_E_CNPD}
#define SYS_PORT_E_CNEN         ${CONFIG_SYS_PORT_E_CNEN}
</#if>
<#if CONFIG_USE_PORT_F == true>

#define SYS_PORT_F_ANSEL        ${CONFIG_SYS_PORT_F_ANSEL}
#define SYS_PORT_F_TRIS         ${CONFIG_SYS_PORT_F_TRIS}
#define SYS_PORT_F_LAT          ${CONFIG_SYS_PORT_F_LAT}
#define SYS_PORT_F_ODC          ${CONFIG_SYS_PORT_F_ODC}
#define SYS_PORT_F_CNPU         ${CONFIG_SYS_PORT_F_CNPU}
#define SYS_PORT_F_CNPD         ${CONFIG_SYS_PORT_F_CNPD}
#define SYS_PORT_F_CNEN         ${CONFIG_SYS_PORT_F_CNEN}
</#if>
<#if CONFIG_USE_PORT_G == true>

#define SYS_PORT_G_ANSEL        ${CONFIG_SYS_PORT_G_ANSEL}
#define SYS_PORT_G_TRIS         ${CONFIG_SYS_PORT_G_TRIS}
#define SYS_PORT_G_LAT          ${CONFIG_SYS_PORT_G_LAT}
#define SYS_PORT_G_ODC          ${CONFIG_SYS_PORT_G_ODC}
#define SYS_PORT_G_CNPU         ${CONFIG_SYS_PORT_G_CNPU}
#define SYS_PORT_G_CNPD         ${CONFIG_SYS_PORT_G_CNPD}
#define SYS_PORT_G_CNEN         ${CONFIG_SYS_PORT_G_CNEN}
</#if>
<#if CONFIG_USE_PORT_H == true>

#define SYS_PORT_H_ANSEL        ${CONFIG_SYS_PORT_H_ANSEL}
#define SYS_PORT_H_TRIS         ${CONFIG_SYS_PORT_H_TRIS}
#define SYS_PORT_H_LAT          ${CONFIG_SYS_PORT_H_LAT}
#define SYS_PORT_H_ODC          ${CONFIG_SYS_PORT_H_ODC}
#define SYS_PORT_H_CNPU         ${CONFIG_SYS_PORT_H_CNPU}
#define SYS_PORT_H_CNPD         ${CONFIG_SYS_PORT_H_CNPD}
#define SYS_PORT_H_CNEN         ${CONFIG_SYS_PORT_H_CNEN}
</#if>
<#if CONFIG_USE_PORT_J == true>

#define SYS_PORT_J_ANSEL        ${CONFIG_SYS_PORT_J_ANSEL}
#define SYS_PORT_J_TRIS         ${CONFIG_SYS_PORT_J_TRIS}
#define SYS_PORT_J_LAT          ${CONFIG_SYS_PORT_J_LAT}
#define SYS_PORT_J_ODC          ${CONFIG_SYS_PORT_J_ODC}
#define SYS_PORT_J_CNPU         ${CONFIG_SYS_PORT_J_CNPU}
#define SYS_PORT_J_CNPD         ${CONFIG_SYS_PORT_J_CNPD}
#define SYS_PORT_J_CNEN         ${CONFIG_SYS_PORT_J_CNEN}
</#if>
<#if CONFIG_USE_PORT_K == true>

#define SYS_PORT_K_ANSEL        ${CONFIG_SYS_PORT_K_ANSEL}
#define SYS_PORT_K_TRIS         ${CONFIG_SYS_PORT_K_TRIS}
#define SYS_PORT_K_LAT          ${CONFIG_SYS_PORT_K_LAT}
#define SYS_PORT_K_ODC          ${CONFIG_SYS_PORT_K_ODC}
#define SYS_PORT_K_CNPU         ${CONFIG_SYS_PORT_K_CNPU}
#define SYS_PORT_K_CNPD         ${CONFIG_SYS_PORT_K_CNPD}
#define SYS_PORT_K_CNEN         ${CONFIG_SYS_PORT_K_CNEN}
</#if>
</#if>
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
