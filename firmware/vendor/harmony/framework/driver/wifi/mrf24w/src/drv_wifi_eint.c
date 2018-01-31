/*******************************************************************************
  MRF24WG Wi-Fi Driver Feature to Support External Interrupt

  Summary:
    MRF24WG Wi-Fi Driver Feature to Support External Interrupt

  Description:
    MRF24WG Wi-Fi Driver Feature to Support External Interrupt
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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

void DRV_WIFI_INT_SourceEnable(void)
{
    bool pinStatus = true;
#if defined(MRF24W_USE_CN_INT)
    pinStatus = SYS_PORTS_PinRead(PORTS_ID_0, WF_INT_PORT_CHANNEL_READ, WF_INT_BIT_POS_READ);
#else
    pinStatus = SYS_PORTS_PinRead(PORTS_ID_0, WF_INT_PORT_CHANNEL, WF_INT_BIT_POS);
#endif
    // If INT line is low, we might have missed an interrupt, hence force an interrupt to
    // clear the status
    if (pinStatus == 0) {
#if defined(PLIB_INT_ExistsSourceFlag)
        if (PLIB_INT_ExistsSourceFlag(INT_ID_0)) {
            PLIB_INT_SourceFlagSet(INT_ID_0, MRF_INT_SOURCE);
        }
#endif
    }

    /* enable the external interrupt */
    SYS_INT_SourceEnable(MRF_INT_SOURCE);
}

void DRV_WIFI_INT_Init(void)
{
    /* disable the external interrupt */
    SYS_INT_SourceDisable(MRF_INT_SOURCE);

    /* clear and enable the interrupt */
    SYS_INT_SourceStatusClear(MRF_INT_SOURCE); // clear status
}

//DOM-IGNORE-END
