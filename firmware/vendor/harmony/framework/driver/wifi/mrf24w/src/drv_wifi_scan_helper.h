/*******************************************************************************
  MRF24WG Wi-Fi Driver Scan Helper Implementation

  File Name:
    drv_wifi_scan_helper.h

  Summary:
    MRF24WG Wi-Fi Driver Scan Helper Implementation

  Description:
    -Provides access to MRF24WG Wi-Fi controller
    -Reference: MRF24WG Datasheet, IEEE 802.11 Standard
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc. All rights reserved.

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

#ifndef _DRV_WIFI_SCAN_H
#define _DRV_WIFI_SCAN_H

/* Scan status/control bits */
#define SCAN_STATE_IN_PROGRESS 0x0001 /* If scan is in progress */
#define SCAN_STATE_VALID_RESULTS 0x0002 /* If we have the valid scan results */
#define SCAN_STATE_DISPLAY_RESULTS 0x0004 /* This flag is only used to control DRV_WIFI_ScanResultsDisplayManager() */

#define IS_SCAN_IN_PROGRESS(x)    ((x) & SCAN_STATE_IN_PROGRESS)
#define IS_SCAN_STATE_VALID(x)    ((x) & SCAN_STATE_VALID_RESULTS)
#define IS_SCAN_STATE_DISPLAY(x)  ((x) & SCAN_STATE_DISPLAY_RESULTS)
#define SCAN_SET_IN_PROGRESS(x)   ((x) |= SCAN_STATE_IN_PROGRESS)
#define SCAN_SET_VALID(x)         ((x) |= SCAN_STATE_VALID_RESULTS)
#define SCAN_SET_DISPLAY(x)       ((x) |= SCAN_STATE_DISPLAY_RESULTS)
#define SCAN_CLEAR_IN_PROGRESS(x) ((x) &= ~SCAN_STATE_IN_PROGRESS)
#define SCAN_CLEAR_VALID(x)       ((x) &= ~SCAN_STATE_VALID_RESULTS)
#define SCAN_CLEAR_DISPLAY(x)     ((x) &= ~SCAN_STATE_DISPLAY_RESULTS)

typedef struct
{
    uint8_t scanState;
    uint16_t numberOfResults;
    uint16_t displayIdx;
} DRV_WIFI_SCAN_STATUS;

extern DRV_WIFI_SCAN_RESULT g_scanResults[];

uint8_t _DRV_WIFI_ScanResultsBufferSize_Get(void);
void DRV_WIFI_ScanComplete(uint16_t scanResults);
void DRV_WIFI_ScanResultsDisplayManager(void);
void DRV_WIFI_ScanResultsSaveManager(uint8_t saveIdx);
uint16_t DRV_WIFI_ScanStart(void);
uint16_t DRV_WIFI_ScanResultGetHelper(uint8_t Idx, DRV_WIFI_SCAN_RESULT *p_ScanResult);

#endif /* _DRV_WIFI_SCAN_HELPER_H */

// DOM-IGNORE-END
