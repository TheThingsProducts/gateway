/*******************************************************************************
  MRF24W GPIO Driver

  File Name: 
    wdrv_mrf24wn_gpio.c  
  
  Summary:
    Module for Microchip TCP/IP Stack
    
  Description:
    Supports SPI communications to the MRF24W module
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

#include "system_definitions.h"

GPIO_OUTLOW_T s_GPIOOutLow = NULL;
GPIO_OUTHIGH_T s_GPIOOutHigh = NULL;

void WDRV_GPIO_Init(GPIO_OUTLOW_T L, GPIO_OUTHIGH_T H)      
{
   s_GPIOOutLow = L;
   s_GPIOOutHigh = H;
}

void WDRV_GPIO_DeInit(void)    
{
   s_GPIOOutLow = NULL;
   s_GPIOOutHigh = NULL;
}

void WDRV_GPIO_OutLow_PIC32MZ_ESK(uint32_t channel, uint32_t bit_pos)
{
#if defined(PLIB_PORTS_ExistsPinModePerPort)
    if (PLIB_PORTS_ExistsPinModePerPort(PORTS_ID_0)) {
        PLIB_PORTS_PinModePerPortSelect(PORTS_ID_0, channel, bit_pos, PORTS_PIN_MODE_DIGITAL);
    }
#endif
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, channel, bit_pos);
    SYS_PORTS_PinClear(PORTS_ID_0, channel, bit_pos);
}

void WDRV_GPIO_OutLow_PIC32MX_ESK(uint32_t channel, uint32_t bit_pos)
{
    SYS_PORTS_PinWrite(PORTS_ID_0, channel, bit_pos, 0);
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, channel, bit_pos);
}

void WDRV_GPIO_OutLow_PIC32MX_EXP16(uint32_t channel, uint32_t bit_pos)
{
    SYS_PORTS_PinWrite(PORTS_ID_0, channel, bit_pos, 0);
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, channel, bit_pos);
}

void WDRV_GPIO_OutLow_PIC32MZ_MEB2(uint32_t channel, uint32_t bit_pos)
{
#if defined(PLIB_PORTS_ExistsPinModePerPort)
    if (PLIB_PORTS_ExistsPinModePerPort(PORTS_ID_0)) {
        PLIB_PORTS_PinModePerPortSelect(PORTS_ID_0, channel, bit_pos, PORTS_PIN_MODE_DIGITAL);
    }
#endif
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, channel, bit_pos);
    SYS_PORTS_PinWrite(PORTS_ID_0, channel, bit_pos,0);
}

void WDRV_GPIO_OutLow_Custom_Board(uint32_t channel, uint32_t bit_pos)
{
#ifdef PIN_MODE_PER_PORT_SELECT
#if defined(PLIB_PORTS_ExistsPinModePerPort)
    if (PLIB_PORTS_ExistsPinModePerPort(PORTS_ID_0)) {
        PLIB_PORTS_PinModePerPortSelect(PORTS_ID_0, channel, bit_pos, PORTS_PIN_MODE_DIGITAL);
    }
#endif /* PLIB_PORTS_ExistsPinModePerPort */
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, channel, bit_pos);
    SYS_PORTS_PinWrite(PORTS_ID_0, channel, bit_pos,0);
#else
    #error "pin mode per port is not selected. Pls select a proper one."
#endif /* PIN_MODE_PER_PORT_SELECT */
}

void WDRV_GPIO_OutLow(uint32_t channel, uint32_t bit_pos)        
{ 
    WDRV_ASSERT((int)s_GPIOOutLow, "s_GPIOOutLow is not valid.");
    s_GPIOOutLow(channel, bit_pos);
}

void WDRV_GPIO_OutHigh_PIC32MZ_ESK(uint32_t channel, uint32_t bit_pos)
{
#if defined(PLIB_PORTS_ExistsPinModePerPort)
    if (PLIB_PORTS_ExistsPinModePerPort(PORTS_ID_0)) {
        PLIB_PORTS_PinModePerPortSelect(PORTS_ID_0, channel, bit_pos, PORTS_PIN_MODE_DIGITAL);
    }
#endif
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, channel, bit_pos);
    SYS_PORTS_PinSet(PORTS_ID_0, channel, bit_pos);
}

void WDRV_GPIO_OutHigh_PIC32MX_ESK(uint32_t channel, uint32_t bit_pos)
{
    SYS_PORTS_PinWrite(PORTS_ID_0, channel, bit_pos, 1);
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, channel, bit_pos);
}

void WDRV_GPIO_OutHigh_PIC32MX_EXP16(uint32_t channel, uint32_t bit_pos)
{
    SYS_PORTS_PinWrite(PORTS_ID_0, channel, bit_pos, 1);
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, channel, bit_pos);
}

void WDRV_GPIO_OutHigh_PIC32MZ_MEB2(uint32_t channel, uint32_t bit_pos)
{
#if defined(PLIB_PORTS_ExistsPinModePerPort)
    if (PLIB_PORTS_ExistsPinModePerPort(PORTS_ID_0)) {
        PLIB_PORTS_PinModePerPortSelect(PORTS_ID_0, channel, bit_pos, PORTS_PIN_MODE_DIGITAL);
    }
#endif
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, channel, bit_pos);
    SYS_PORTS_PinWrite(PORTS_ID_0, channel, bit_pos, 1);
}

void WDRV_GPIO_OutHigh_Custom_Board(uint32_t channel, uint32_t bit_pos)
{
#ifdef PIN_MODE_PER_PORT_SELECT
#if defined(PLIB_PORTS_ExistsPinModePerPort)
    if (PLIB_PORTS_ExistsPinModePerPort(PORTS_ID_0)) {
        PLIB_PORTS_PinModePerPortSelect(PORTS_ID_0, channel, bit_pos, PORTS_PIN_MODE_DIGITAL);
    }
#endif /* PLIB_PORTS_ExistsPinModePerPort */
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, channel, bit_pos);
    SYS_PORTS_PinWrite(PORTS_ID_0, channel, bit_pos,1);
#else
    #error "pin mode per port is not selected. Pls select a proper one."
#endif /* PIN_MODE_PER_PORT_SELECT */
}

void WDRV_GPIO_OutHigh(uint32_t channel, uint32_t bit_pos)      
{
    WDRV_ASSERT((int)s_GPIOOutHigh, "s_GPIOOutHigh is not valid.");
    s_GPIOOutHigh(channel, bit_pos);
}

void WDRV_GPIO_PowerUp(void)
{
    WDRV_GPIO_OutHigh(WF_HIBERNATE_PORT_CHANNEL, WF_HIBERNATE_BIT_POS); 
}

void WDRV_GPIO_PowerDown(void)
{
    WDRV_GPIO_OutLow(WF_HIBERNATE_PORT_CHANNEL, WF_HIBERNATE_BIT_POS);
}

void WDRV_GPIO_MRF24WG_Disable(void)
{
#if WDRV_BOARD_TYPE == WDRV_BD_TYPE_MEB2
    WDRV_GPIO_OutHigh(PORT_CHANNEL_H, 14);
#endif
}

//DOM-IGNORE-END
