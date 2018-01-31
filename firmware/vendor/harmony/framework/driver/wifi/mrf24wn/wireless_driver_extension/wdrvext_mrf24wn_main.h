/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    wdrvext_mrf24wn_main.h

  Summary:
    MRF24WN Wireless Driver Extension

  Description:

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
// DOM-IGNORE-END

#ifndef _WDRVEXT_MRF24WN_MAIN_H
#define _WDRVEXT_MRF24WN_MAIN_H

#include <stdbool.h>

#define WDRV_SPI_BAUDRATE 1000000

typedef struct {
    void (*CopyFrameToStackPacketBuffer_CB)(uint32_t len, uint8_t const *const frame);
    void (*ProceedConnectEvent_CB)(uint32_t connected, uint8_t devID, uint8_t *bssid, bool bssConn, uint8_t reason);
    void (*RFReady_CB)(uint8_t const *const addr);
    void (*ScanDone_CB)(uint32_t status);
    void (*InitDone_CB)(void);
    void (*DeinitDone_CB)(void);
    void (*WPSDone_CB)(void);
    uint8_t address[6];
    uint32_t versions[2];
} WDRV_MRF24WN_UPPER_CTX;

typedef struct {
    volatile uint32_t driverTaskSync;
    void *mainDriverHandle;
    uint32_t event_print;
    uint32_t init_task_priority;
    uint32_t main_task_priority;
    uint32_t init_task_stack_size;
    uint32_t main_task_stack_size;
    uint32_t board_type;
} WDRVEXT_MRF24WN_PRIV;

extern WDRVEXT_MRF24WN_PRIV g_wdrvext_priv;

#endif /* _WDRVEXT_MRF24WN_MAIN_H */
