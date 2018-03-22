// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#ifndef _ERROR_MESSAGES_H
#define _ERROR_MESSAGES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define ERROR_MESSAGES_PAYLOAD_SIZE 8

typedef enum
{ // for backward compatibility don't change, only add
    ERROR_MESSAGES_TYPE_NONE,
    ERROR_MESSAGES_TYPE_POWER_CYCLE,
    ERROR_MESSAGES_TYPE_GENERAL_EXCEPTION,
    ERROR_MESSAGES_TYPE_INVALID_POINTER,
    ERROR_MESSAGES_TYPE_ASSERT,
    ERROR_MESSAGES_TYPE_APPLICATION_ERROR,
    ERROR_MESSAGES_TYPE_WARNING,
} ERROR_MESSAGES_TYPES_t;

typedef enum
{ // for backward compatibility don't change, only add
    ERROR_MESSAGE_WARNING_INVALID_FREQUENCY_PLAN,
    ERROR_MESSAGE_WARNING_HTTP_COMMUNICATION_FAILURE_ACTIVATION,
    ERROR_MESSAGE_WARNING_HTTP_COMMUNICATION_FAILURE_CONFIGURATION,
    ERROR_MESSAGE_WARNING_HTTP_COMMUNICATION_FAILURE_FREQPLAN,
    ERROR_MESSAGE_WARNING_HTTP_COMMUNICATION_FAILURE_FIRMWARE,
    ERROR_MESSAGE_WARNING_MQTT_COMMUNICATION_FAILURE,
    ERROR_MESSAGE_WARNING_FIRMWARE_STORAGE_ERROR,
    ERROR_MESSAGE_WARNING_LORA_CONFIG_FAILURE,
    ERROR_MESSAGE_WARNING_NETWORK_CHANGE,
    ERROR_MESSAGE_WARNING_LORA_IDLE_ONE_DAY,
    ERROR_MESSAGE_WARNING_LORA_IDLE_ONE_WEEK,
    ERROR_MESSAGE_WARNING_LORA_TX_TOO_LATE,
    ERROR_MESSAGE_WARNING_LORA_UART_ERROR,
} ERROR_MESSAGE_WARNING_t;

void ErrorMessage_Initialize();
void ErrorMessageFatal_StoreGeneralException(uint32_t code, uint32_t address);
void ErrorMessageFatal_StoreInvalidPointer(uint32_t bad_address, uint32_t address);
void ErrorMessageFatal_StoreAssert(char* file, uint16_t line);
void ErrorMessageFatal_StoreApplicationError(uint32_t error);

void ErrorMessageWarning_Set(ERROR_MESSAGE_WARNING_t config);

bool ErrorMessageFatal_BuildMessage(char message[ERROR_MESSAGES_PAYLOAD_SIZE * 2 + 1]);
bool ErrorMessageWarning_BuildMessage(char message[ERROR_MESSAGES_PAYLOAD_SIZE * 2 + 1]);

#endif /* _ERROR_MESSAGES_H */
