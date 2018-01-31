/*******************************************************************************
  MRF24WG Wi-Fi Driver Feature to Support External Interrupt

  File Name:
    drv_wifi_eint.h

  Summary:
    MRF24WG Wi-Fi Driver Feature to Support External Interrupt

  Description:
    MRF24WG Wi-Fi Driver Feature to Support External Interrupt
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc. All rights reserved.

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

#ifndef _DRV_WIFI_EINT_H
#define _DRV_WIFI_EINT_H

// *****************************************************************************
// *****************************************************************************
// Section: MRF24W External Interrupt Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
        void DRV_WIFI_INT_Init(void)

  Summary:
    Initializes The MRF24W External Interrupt.

  Description:
    This routine initializes the MRF24W External Interrupt.

  Conditions:
    None.

  Input:
    None.

  Return:
    None.

  Remarks:
    None.
 *******************************************************************************/
void DRV_WIFI_INT_Init(void);

/*******************************************************************************
  Function:
        void DRV_WIFI_INT_SourceEnable(void)

  Summary:
    Enables MRF24W External Interrupt.

  Description:
    This routine enabled the MRF24W External Interrupt.

  Conditions:
    None.

  Input:
    None.

  Return:
    None.

  Remarks:
    None.
 *******************************************************************************/
void DRV_WIFI_INT_SourceEnable(void);

#endif /* _DRV_WIFI_EINT_H */

// DOM-IGNORE-END
