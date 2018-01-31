// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#ifndef _APP_LORA_H /* Guard against multiple inclusion */
#define _APP_LORA_H

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C"
{
#endif

#include "app.h"
#include "connector.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

// loraRXPacket* dequeueLoRaRX(void);

uint8_t hasLoraPacketInQueue();
void    APP_LORA_SetStartEvent(void);

bool dequeueLoRaRX(loraRXPacket* pkt);

void enqueueLoRaRX(loraRXPacket* pkt);

bool dequeueLoRaTX(loraTXPacket* pkt);

void enqueueLoRaTX(loraTXPacket* pkt);

uint8_t hasLoraRXPacketInQueue(void);
uint8_t hasLoraTXPacketInQueue(void);

uint8_t APP_LORA_GET_APP_STATE(void);
void    lora_read(void);
bool    APP_LORA_HAS_CORRECT_FREQ_PLAN(void);
uint8_t APP_LORA_GW_CARD_VERSION(void);

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
