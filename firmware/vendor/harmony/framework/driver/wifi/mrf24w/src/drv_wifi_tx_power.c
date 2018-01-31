/*******************************************************************************
  MRF24WG Driver Transmit (TX) Power Functions

  File Name:
    drv_wifi_tx_power.c

  Summary:
    MRF24WG Driver Transmit (TX) Power Functions

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

/*******************************************************************************
  Function:
    void DRV_WIFI_TxPowerMaxSet(uint8_t maxTxPower)

  Summary:
    Sets the Tx max power on the MRF24WG0M.

  Description:
    After initialization the MRF24WG0M max Tx power is determined by a
    factory-set value.  This function can set a different maximum
    Tx power levels.  However, this function can never set a maximum Tx power
    greater than the factory-set value, which can be read via
    DRV_WIFI_TxPowerFactoryMaxGet().

 Parameters:
    maxTxPower - valid range (0 to 17 dBm)

  Returns:
    None.
 *******************************************************************************/
void DRV_WIFI_TxPowerMaxSet(uint8_t maxTxPower)
{
    uint8_t factoryMaxPower;
    uint8_t msgData[2];
    uint16_t max = (uint16_t)maxTxPower;

    DRV_WIFI_TxPowerFactoryMaxGet(&factoryMaxPower);

    /* cannot set max tx power greater than factor-set max tx power */
    DRV_WIFI_ASSERT(maxTxPower <= factoryMaxPower, "");

    msgData[0] = (uint8_t)(max >> 8); /* msb of max power */
    msgData[1] = (uint8_t)(max & 0xff); /* lsb of max power */

    SetParamMsgSend(PARAM_TX_POWER, msgData, sizeof(msgData));
}

/*******************************************************************************
  Function:
    void DRV_WIFI_TxPowerMaxGet(uint8_t *p_maxTxPower);

  Summary:
    Gets the Tx max power on the MRF24WG0M.

  Description:
    Gets the Tx max power setting from the MRF24WG.

  Parameters:
    p_maxTxPower - pointer to where max power setting is written (dBm)

  Returns:
    None.

  Remarks:
    None
 *******************************************************************************/
void DRV_WIFI_TxPowerMaxGet(uint8_t *p_maxTxPower)
{
    uint8_t msgData[6];
    uint16_t tmp;

    GetParamMsgSend(PARAM_TX_POWER, msgData, sizeof(msgData));

    /* max tx power is a 16-bit value stored in bytes [1:0] msg data */
    tmp = ((uint16_t)(msgData[0]) << 8);
    tmp |= (uint16_t)msgData[1];
    *p_maxTxPower = (uint8_t)tmp;
}

/*******************************************************************************
  Function:
    void DRV_WIFI_TxPowerFactoryMaxGet(int8_t *p_factoryMaxTxPower)

  Summary:
    Retrieves the factory-set max Tx power from the MRF24W.

  Description:
    This function retrieves the factory-set max tx power from the MRF24WG.

  Parameters:
    p_factoryMaxTxPower - pointer to where factory max power is written (dbM)

  Returns:
    None.

  Remarks:
    None.
 *******************************************************************************/
void DRV_WIFI_TxPowerFactoryMaxGet(uint8_t *p_factoryMaxTxPower)
{
    uint8_t msgData[2];

    /* read max and min factory-set power levels */
    GetParamMsgSend(PARAM_FACTORY_SET_TX_MAX_POWER, msgData, sizeof(msgData));

    /* msgData[0] = max power, msgData[1] = min power */
    *p_factoryMaxTxPower = msgData[0];
}

//DOM-IGNORE-END
