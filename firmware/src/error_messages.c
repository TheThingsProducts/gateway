// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#include <string.h>
#include <stdio.h>
#include "helper_wdt.h"
#include "error_messages.h"

// A magic number to verify if the struct is filled by the application with valid data
#define ERROR_MESSAGES_MAGIC 0x6572726F            // ascii "erro"
#define ERROR_MESSAGES_ADDRESS 0x8007FFE0          // end - 0x20
#define ERROR_MESSAGES_ADDRESS_UNCACHED 0xA007FFE0 // end - 0x20 [needed for functions used in exceptions]

typedef struct __attribute__((packed, aligned(1)))
{
    uint8_t reboot_reason;
} payload_power_cycle_t;

typedef struct __attribute__((packed, aligned(1)))
{
    uint32_t code;
    uint32_t address;
} payload_general_exception_t;

typedef struct __attribute__((packed, aligned(1)))
{
    uint32_t epc;
    uint32_t nested_epc;
} payload_invalid_pointer_t;

typedef struct __attribute__((packed, aligned(1)))
{
    uint8_t  file[6];
    uint16_t line;
} payload_assert_t;

typedef struct __attribute__((packed, aligned(1)))
{
    uint32_t error;
} payload_application_error_t;

typedef union {
    uint8_t                     arr[ERROR_MESSAGES_PAYLOAD_SIZE];
    payload_power_cycle_t       power_cycle;
    payload_general_exception_t general_exception;
    payload_invalid_pointer_t   invalid_pointer;
    payload_assert_t            assert;
    payload_application_error_t application_error;
} payload_union_t;

_Static_assert(sizeof(payload_union_t) <= 8, "Payload must be max 8");

// The structure containing the error information
typedef volatile struct __attribute__((packed, aligned(1)))
{
    uint32_t        magic;
    uint8_t         type;
    payload_union_t payload;
} reboot_t;

_Static_assert(sizeof(reboot_t) <= 16, "Reboot data must be max 16");

static bool   _RetrieveFatalError(ERROR_MESSAGES_TYPES_t* type, uint8_t payload[ERROR_MESSAGES_PAYLOAD_SIZE]);
static size_t _PayloadSize(ERROR_MESSAGES_TYPES_t type);
static void   _ClearFatal(void);

static uint32_t newWarningsBitmask = 0;
volatile reboot_t __attribute__((address(ERROR_MESSAGES_ADDRESS), persistent)) REBOOT_ERROR_INFO;

void ErrorMessage_Initialize()
{
    if(REBOOT_ERROR_INFO.magic != ERROR_MESSAGES_MAGIC || REBOOT_ERROR_INFO.type == ERROR_MESSAGES_TYPE_NONE)
    {
        // No stored error, assume a power cycle
        REBOOT_ERROR_INFO.magic                             = ERROR_MESSAGES_MAGIC;
        REBOOT_ERROR_INFO.type                              = ERROR_MESSAGES_TYPE_POWER_CYCLE;
        REBOOT_ERROR_INFO.payload.power_cycle.reboot_reason = wdt_get_reboot_reason();
    }

    PRINT_ARRAY_UINT8(&REBOOT_ERROR_INFO, 16, "BOOT", "persisted info");
}

void ErrorMessageFatal_StoreGeneralException(uint32_t code, uint32_t address)
{
    reboot_t* p                          = ((reboot_t*)ERROR_MESSAGES_ADDRESS_UNCACHED);
    p->magic                             = ERROR_MESSAGES_MAGIC;
    p->type                              = ERROR_MESSAGES_TYPE_GENERAL_EXCEPTION;
    p->payload.general_exception.code    = code;
    p->payload.general_exception.address = address;
}

void ErrorMessageFatal_StoreInvalidPointer(uint32_t epc, uint32_t nested_epc)
{
    reboot_t* p                           = ((reboot_t*)ERROR_MESSAGES_ADDRESS_UNCACHED);
    p->magic                              = ERROR_MESSAGES_MAGIC;
    p->type                               = ERROR_MESSAGES_TYPE_INVALID_POINTER;
    p->payload.invalid_pointer.epc        = epc;
    p->payload.invalid_pointer.nested_epc = nested_epc;
}

void ErrorMessageFatal_StoreAssert(char* file, uint16_t line)
{
    REBOOT_ERROR_INFO.magic = ERROR_MESSAGES_MAGIC;
    REBOOT_ERROR_INFO.type  = ERROR_MESSAGES_TYPE_ASSERT;

    // Store last characters of the file name before the extension
    char* f = strrchr(file, '.'); // find extension
    if(f == NULL)
        f = &file[strlen(file)]; // in case not found point to the end of the string
    int i;
    for(i = sizeof(REBOOT_ERROR_INFO.payload.assert.file) - 1; i >= 0; i--)
    {
        f--;
        REBOOT_ERROR_INFO.payload.assert.file[i] =
            f >= file ? *f : ' '; // copy last characters, if filename is short prefix it with spaces
    }

    REBOOT_ERROR_INFO.payload.assert.line = line;
}

void ErrorMessageFatal_StoreApplicationError(uint32_t error)
{
    REBOOT_ERROR_INFO.magic                           = ERROR_MESSAGES_MAGIC;
    REBOOT_ERROR_INFO.type                            = ERROR_MESSAGES_TYPE_INVALID_POINTER;
    REBOOT_ERROR_INFO.payload.application_error.error = error;
}

static bool _RetrieveFatalError(ERROR_MESSAGES_TYPES_t* type, uint8_t payload[ERROR_MESSAGES_PAYLOAD_SIZE])
{
    if(REBOOT_ERROR_INFO.magic == ERROR_MESSAGES_MAGIC && REBOOT_ERROR_INFO.type != ERROR_MESSAGES_TYPE_NONE)
    {
        *type = (ERROR_MESSAGES_TYPES_t)REBOOT_ERROR_INFO.type;
        int i;
        for(i = 0; i < ERROR_MESSAGES_PAYLOAD_SIZE; i++)
        {
            payload[i] = REBOOT_ERROR_INFO.payload.arr[i];
        }
        return true;
    }

    return false;
}

static size_t _PayloadSize(ERROR_MESSAGES_TYPES_t type)
{
    size_t size;
    switch(type)
    {
        case ERROR_MESSAGES_TYPE_POWER_CYCLE:
            size = sizeof(payload_power_cycle_t);
            break;
        case ERROR_MESSAGES_TYPE_GENERAL_EXCEPTION:
            size = sizeof(payload_general_exception_t);
            break;
        case ERROR_MESSAGES_TYPE_INVALID_POINTER:
            size = sizeof(payload_invalid_pointer_t);
            break;
        case ERROR_MESSAGES_TYPE_ASSERT:
            size = sizeof(payload_assert_t);
            break;
        case ERROR_MESSAGES_TYPE_APPLICATION_ERROR:
            size = sizeof(payload_application_error_t);
            break;
        case ERROR_MESSAGES_TYPE_WARNING:
        case ERROR_MESSAGES_TYPE_NONE:
        default:
            size = 0;
            break;
    }
    return size;
}

static void _ClearFatal()
{
    REBOOT_ERROR_INFO.magic = ERROR_MESSAGES_MAGIC;
    REBOOT_ERROR_INFO.type  = ERROR_MESSAGES_TYPE_NONE;
}

void ErrorMessageWarning_Set(ERROR_MESSAGE_WARNING_t newWarning)
{
    newWarningsBitmask |= 1 << newWarning;
}

bool ErrorMessageFatal_BuildMessage(char message[ERROR_MESSAGES_PAYLOAD_SIZE * 2 + 1])
{
    ERROR_MESSAGES_TYPES_t type;
    uint8_t                payload[ERROR_MESSAGES_PAYLOAD_SIZE];

    if(_RetrieveFatalError(&type, payload) && type != ERROR_MESSAGES_TYPE_NONE)
    {
        _ClearFatal();
        size_t size = _PayloadSize(type);

        sprintf(message, "%02X", type);
        int i;
        for(i = 0; i < size; i++)
        {
            sprintf(&message[2 + (i << 1)], "%02X", payload[i]);
        }
        return true;
    }

    return false;
}

bool ErrorMessageWarning_BuildMessage(char message[ERROR_MESSAGES_PAYLOAD_SIZE * 2 + 1])
{
    if(newWarningsBitmask > 0)
    {
        sprintf(message, "%02X%08X", ERROR_MESSAGES_TYPE_WARNING, newWarningsBitmask);
        newWarningsBitmask = 0;
        return true;
    }

    return false;
}
