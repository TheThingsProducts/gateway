/* Created by plibgen $Revision: 1.31 $ */

#ifndef _DMA_P32MZ2048EFM144_H
#define _DMA_P32MZ2048EFM144_H

/* Section 1 - Enumerate instances, define constants, VREGs */

#include <xc.h>
#include <stdbool.h>

#include "peripheral/peripheral_common_32bit.h"

/* Default definition used for all API dispatch functions */
#ifndef PLIB_INLINE_API
    #define PLIB_INLINE_API extern inline
#endif

/* Default definition used for all other functions */
#ifndef PLIB_INLINE
    #define PLIB_INLINE extern inline
#endif

typedef enum {

    DMA_ID_0 = 0,
    DMA_NUMBER_OF_MODULES

} DMA_MODULE_ID;

typedef enum {

    DMA_CHANNEL_0 = 0x0,
    DMA_CHANNEL_1 = 0x1,
    DMA_CHANNEL_2 = 0x2,
    DMA_CHANNEL_3 = 0x3,
    DMA_CHANNEL_4 = 0x4,
    DMA_CHANNEL_5 = 0x5,
    DMA_CHANNEL_6 = 0x6,
    DMA_CHANNEL_7 = 0x7,
    DMA_NUMBER_OF_CHANNELS = 0x8

} DMA_CHANNEL;

typedef enum {

    DMA_CHANNEL_0_INT_SOURCE = 134,
    DMA_CHANNEL_1_INT_SOURCE = 135,
    DMA_CHANNEL_2_INT_SOURCE = 136,
    DMA_CHANNEL_3_INT_SOURCE = 137,
    DMA_CHANNEL_4_INT_SOURCE = 138,
    DMA_CHANNEL_5_INT_SOURCE = 139,
    DMA_CHANNEL_6_INT_SOURCE = 140,
    DMA_CHANNEL_7_INT_SOURCE = 141

} DMA_CHANNEL_INT_SOURCE;

typedef enum {

    DMA_CHANNEL_PRIORITY_0 = 0x0,
    DMA_CHANNEL_PRIORITY_1 = 0x1,
    DMA_CHANNEL_PRIORITY_2 = 0x2,
    DMA_CHANNEL_PRIORITY_3 = 0x3

} DMA_CHANNEL_PRIORITY;

typedef enum {

    DMA_CHANNEL_TRIGGER_TRANSFER_START = 0x0,
    DMA_CHANNEL_TRIGGER_TRANSFER_ABORT = 0x1,
    DMA_CHANNEL_TRIGGER_PATTERN_MATCH_ABORT = 0x2

} DMA_CHANNEL_TRIGGER_TYPE;

typedef enum {

    DMA_TRIGGER_SOURCE_NONE = -1,
    DMA_TRIGGER_TIMER_CORE = 0,
    DMA_TRIGGER_SOFTWARE_0 = 1,
    DMA_TRIGGER_SOFTWARE_1 = 2,
    DMA_TRIGGER_EXTERNAL_0 = 3,
    DMA_TRIGGER_EXTERNAL_1 = 8,
    DMA_TRIGGER_EXTERNAL_2 = 13,
    DMA_TRIGGER_EXTERNAL_3 = 18,
    DMA_TRIGGER_EXTERNAL_4 = 23,
    DMA_TRIGGER_TIMER_1 = 4,
    DMA_TRIGGER_TIMER_2 = 9,
    DMA_TRIGGER_TIMER_3 = 14,
    DMA_TRIGGER_TIMER_4 = 19,
    DMA_TRIGGER_TIMER_5 = 24,
    DMA_TRIGGER_TIMER_6 = 28,
    DMA_TRIGGER_TIMER_7 = 32,
    DMA_TRIGGER_TIMER_8 = 36,
    DMA_TRIGGER_TIMER_9 = 40,
    DMA_TRIGGER_INPUT_CAPTURE_1 = 6,
    DMA_TRIGGER_INPUT_CAPTURE_2 = 11,
    DMA_TRIGGER_INPUT_CAPTURE_3 = 16,
    DMA_TRIGGER_INPUT_CAPTURE_4 = 21,
    DMA_TRIGGER_INPUT_CAPTURE_5 = 26,
    DMA_TRIGGER_INPUT_CAPTURE_6 = 30,
    DMA_TRIGGER_INPUT_CAPTURE_7 = 34,
    DMA_TRIGGER_INPUT_CAPTURE_8 = 38,
    DMA_TRIGGER_INPUT_CAPTURE_9 = 42,
    DMA_TRIGGER_INPUT_CAPTURE_1_ERROR = 5,
    DMA_TRIGGER_INPUT_CAPTURE_2_ERROR = 10,
    DMA_TRIGGER_INPUT_CAPTURE_3_ERROR = 15,
    DMA_TRIGGER_INPUT_CAPTURE_4_ERROR = 20,
    DMA_TRIGGER_INPUT_CAPTURE_5_ERROR = 25,
    DMA_TRIGGER_INPUT_CAPTURE_6_ERROR = 29,
    DMA_TRIGGER_INPUT_CAPTURE_7_ERROR = 33,
    DMA_TRIGGER_INPUT_CAPTURE_8_ERROR = 37,
    DMA_TRIGGER_INPUT_CAPTURE_9_ERROR = 41,
    DMA_TRIGGER_OUTPUT_COMPARE_1 = 7,
    DMA_TRIGGER_OUTPUT_COMPARE_2 = 12,
    DMA_TRIGGER_OUTPUT_COMPARE_3 = 17,
    DMA_TRIGGER_OUTPUT_COMPARE_4 = 22,
    DMA_TRIGGER_OUTPUT_COMPARE_5 = 27,
    DMA_TRIGGER_OUTPUT_COMPARE_6 = 31,
    DMA_TRIGGER_OUTPUT_COMPARE_7 = 35,
    DMA_TRIGGER_OUTPUT_COMPARE_8 = 39,
    DMA_TRIGGER_OUTPUT_COMPARE_9 = 43,
    DMA_TRIGGER_SPI_1_ERROR = 109,
    DMA_TRIGGER_SPI_1_RECEIVE = 110,
    DMA_TRIGGER_SPI_1_TRANSMIT = 111,
    DMA_TRIGGER_SPI_2_ERROR = 142,
    DMA_TRIGGER_SPI_2_RECEIVE = 143,
    DMA_TRIGGER_SPI_2_TRANSMIT = 144,
    DMA_TRIGGER_SPI_3_ERROR = 154,
    DMA_TRIGGER_SPI_3_RECEIVE = 155,
    DMA_TRIGGER_SPI_3_TRANSMIT = 156,
    DMA_TRIGGER_SPI_4_ERROR = 163,
    DMA_TRIGGER_SPI_4_RECEIVE = 164,
    DMA_TRIGGER_SPI_4_TRANSMIT = 165,
    DMA_TRIGGER_SPI_5_ERROR = 176,
    DMA_TRIGGER_SPI_5_RECEIVE = 177,
    DMA_TRIGGER_SPI_5_TRANSMIT = 178,
    DMA_TRIGGER_SPI_6_ERROR = 185,
    DMA_TRIGGER_SPI_6_RECEIVE = 186,
    DMA_TRIGGER_SPI_6_TRANSMIT = 187,
    DMA_TRIGGER_I2C_1_ERROR = 115,
    DMA_TRIGGER_I2C_1_SLAVE = 116,
    DMA_TRIGGER_I2C_1_MASTER = 117,
    DMA_TRIGGER_I2C_2_ERROR = 148,
    DMA_TRIGGER_I2C_2_SLAVE = 149,
    DMA_TRIGGER_I2C_2_MASTER = 150,
    DMA_TRIGGER_I2C_3_ERROR = 160,
    DMA_TRIGGER_I2C_3_SLAVE = 161,
    DMA_TRIGGER_I2C_3_MASTER = 162,
    DMA_TRIGGER_I2C_4_ERROR = 173,
    DMA_TRIGGER_I2C_4_SLAVE = 174,
    DMA_TRIGGER_I2C_4_MASTER = 175,
    DMA_TRIGGER_I2C_5_ERROR = 182,
    DMA_TRIGGER_I2C_5_SLAVE = 183,
    DMA_TRIGGER_I2C_5_MASTER = 184,
    DMA_TRIGGER_USART_1_ERROR = 112,
    DMA_TRIGGER_USART_1_RECEIVE = 113,
    DMA_TRIGGER_USART_1_TRANSMIT = 114,
    DMA_TRIGGER_USART_2_ERROR = 145,
    DMA_TRIGGER_USART_2_RECEIVE = 146,
    DMA_TRIGGER_USART_2_TRANSMIT = 147,
    DMA_TRIGGER_USART_3_ERROR = 157,
    DMA_TRIGGER_USART_3_RECEIVE = 158,
    DMA_TRIGGER_USART_3_TRANSMIT = 159,
    DMA_TRIGGER_USART_4_ERROR = 170,
    DMA_TRIGGER_USART_4_RECEIVE = 171,
    DMA_TRIGGER_USART_4_TRANSMIT = 172,
    DMA_TRIGGER_USART_5_ERROR = 179,
    DMA_TRIGGER_USART_5_RECEIVE = 180,
    DMA_TRIGGER_USART_5_TRANSMIT = 181,
    DMA_TRIGGER_USART_6_ERROR = 188,
    DMA_TRIGGER_USART_6_RECEIVE = 189,
    DMA_TRIGGER_USART_6_TRANSMIT = 190,
    DMA_TRIGGER_CHANGE_NOTICE_A = 118,
    DMA_TRIGGER_CHANGE_NOTICE_B = 119,
    DMA_TRIGGER_CHANGE_NOTICE_C = 120,
    DMA_TRIGGER_CHANGE_NOTICE_D = 121,
    DMA_TRIGGER_CHANGE_NOTICE_E = 122,
    DMA_TRIGGER_CHANGE_NOTICE_F = 123,
    DMA_TRIGGER_CHANGE_NOTICE_G = 124,
    DMA_TRIGGER_CHANGE_NOTICE_H = 125,
    DMA_TRIGGER_CHANGE_NOTICE_J = 126,
    DMA_TRIGGER_CHANGE_NOTICE_K = 127,
    DMA_TRIGGER_DMA_0 = 134,
    DMA_TRIGGER_DMA_1 = 135,
    DMA_TRIGGER_DMA_2 = 136,
    DMA_TRIGGER_DMA_3 = 137,
    DMA_TRIGGER_DMA_4 = 138,
    DMA_TRIGGER_DMA_5 = 139,
    DMA_TRIGGER_DMA_6 = 140,
    DMA_TRIGGER_DMA_7 = 141,
    DMA_TRIGGER_COMPARATOR_1 = 130,
    DMA_TRIGGER_COMPARATOR_2 = 131,
    DMA_TRIGGER_ADC_1 = 44,
    DMA_TRIGGER_ADC1_DC1 = 46,
    DMA_TRIGGER_ADC1_DC2 = 47,
    DMA_TRIGGER_ADC1_DC3 = 48,
    DMA_TRIGGER_ADC1_DC4 = 49,
    DMA_TRIGGER_ADC1_DC5 = 50,
    DMA_TRIGGER_ADC1_DC6 = 51,
    DMA_TRIGGER_ADC1_DF1 = 52,
    DMA_TRIGGER_ADC1_DF2 = 53,
    DMA_TRIGGER_ADC1_DF3 = 54,
    DMA_TRIGGER_ADC1_DF4 = 55,
    DMA_TRIGGER_ADC1_DF5 = 56,
    DMA_TRIGGER_ADC1_DF6 = 57,
    DMA_TRIGGER_ADC1_DATA0 = 59,
    DMA_TRIGGER_ADC1_DATA1 = 60,
    DMA_TRIGGER_ADC1_DATA2 = 61,
    DMA_TRIGGER_ADC1_DATA3 = 62,
    DMA_TRIGGER_ADC1_DATA4 = 63,
    DMA_TRIGGER_ADC1_DATA5 = 64,
    DMA_TRIGGER_ADC1_DATA6 = 65,
    DMA_TRIGGER_ADC1_DATA7 = 66,
    DMA_TRIGGER_ADC1_DATA8 = 67,
    DMA_TRIGGER_ADC1_DATA9 = 68,
    DMA_TRIGGER_ADC1_DATA10 = 69,
    DMA_TRIGGER_ADC1_DATA11 = 70,
    DMA_TRIGGER_ADC1_DATA12 = 71,
    DMA_TRIGGER_ADC1_DATA13 = 72,
    DMA_TRIGGER_ADC1_DATA14 = 73,
    DMA_TRIGGER_ADC1_DATA15 = 74,
    DMA_TRIGGER_ADC1_DATA16 = 75,
    DMA_TRIGGER_ADC1_DATA17 = 76,
    DMA_TRIGGER_ADC1_DATA18 = 77,
    DMA_TRIGGER_ADC1_DATA19 = 78,
    DMA_TRIGGER_ADC1_DATA20 = 79,
    DMA_TRIGGER_ADC1_DATA21 = 80,
    DMA_TRIGGER_ADC1_DATA22 = 81,
    DMA_TRIGGER_ADC1_DATA23 = 82,
    DMA_TRIGGER_ADC1_DATA24 = 83,
    DMA_TRIGGER_ADC1_DATA25 = 84,
    DMA_TRIGGER_ADC1_DATA26 = 85,
    DMA_TRIGGER_ADC1_DATA27 = 86,
    DMA_TRIGGER_ADC1_DATA28 = 87,
    DMA_TRIGGER_ADC1_DATA29 = 88,
    DMA_TRIGGER_ADC1_DATA30 = 89,
    DMA_TRIGGER_ADC1_DATA31 = 90,
    DMA_TRIGGER_ADC1_DATA32 = 91,
    DMA_TRIGGER_ADC1_DATA33 = 92,
    DMA_TRIGGER_ADC1_DATA34 = 93,
    DMA_TRIGGER_ADC1_DATA35 = 94,
    DMA_TRIGGER_ADC1_DATA36 = 95,
    DMA_TRIGGER_ADC1_DATA37 = 96,
    DMA_TRIGGER_ADC1_DATA38 = 97,
    DMA_TRIGGER_ADC1_DATA39 = 98,
    DMA_TRIGGER_ADC1_DATA40 = 99,
    DMA_TRIGGER_ADC1_DATA41 = 100,
    DMA_TRIGGER_ADC1_DATA42 = 101,
    DMA_TRIGGER_ADC1_DATA43 = 102,
    DMA_TRIGGER_ADC1_DATA44 = 103,
    DMA_TRIGGER_CORE_PERF_COUNT = 104,
    DMA_TRIGGER_CORE_FAST_DEBUG_CHAN = 105,
    DMA_TRIGGER_SYSTEM_BUS_PROTECTION = 106,
    DMA_TRIGGER_CRYPTO = 107,
    DMA_TRIGGER_PARALLEL_PORT = 128,
    DMA_TRIGGER_CAN_1 = 151,
    DMA_TRIGGER_CAN_2 = 152,
    DMA_TRIGGER_RTCC = 166,
    DMA_TRIGGER_FLASH_CONTROL = 167,
    DMA_TRIGGER_USB_1 = 132,
    DMA_TRIGGER_USB_DMA = 133,
    DMA_TRIGGER_ETH_1 = 153,
    DMA_TRIGGER_PARALLEL_PORT_ERROR = 129,
    DMA_TRIGGER_PREFETCH = 168,
    DMA_TRIGGER_SQI1 = 169

} DMA_TRIGGER_SOURCE;

typedef enum {

    DMA_CRC_IP_HEADER = 0x1,
    DMA_CRC_LFSR = 0x0

} DMA_CRC_TYPE;

typedef enum {

    DMA_CRC_BYTEORDER_NO_SWAPPING = 0x0,
    DMA_CRC_SWAP_BYTE_ON_WORD_BOUNDARY = 0x1,
    DMA_CRC_SWAP_HALF_WORD_ON_WORD_BOUNDARY = 0x2,
    DMA_CRC_SWAP_BYTE_ON_HALF_WORD_BOUNDARY = 0x3

} DMA_CRC_BYTE_ORDER;

typedef enum {

    DMA_INT_ADDRESS_ERROR = 0x1,
    DMA_INT_TRANSFER_ABORT = 0x2,
    DMA_INT_CELL_TRANSFER_COMPLETE = 0x4,
    DMA_INT_BLOCK_TRANSFER_COMPLETE = 0x8,
    DMA_INT_DESTINATION_HALF_FULL = 0x10,
    DMA_INT_DESTINATION_DONE = 0x20,
    DMA_INT_SOURCE_HALF_EMPTY = 0x40,
    DMA_INT_SOURCE_DONE = 0x80

} DMA_INT_TYPE;

typedef enum {

    DMA_CRC_BIT_ORDER_LSB = 0x1,
    DMA_CRC_BIT_ORDER_MSB = 0x0

} DMA_CRC_BIT_ORDER;

typedef enum {

    DMA_PATTERN_MATCH_LENGTH_1BYTE = 0x0,
    DMA_PATTERN_MATCH_LENGTH_2BYTES = 0x1

} DMA_PATTERN_LENGTH;

typedef enum {

    DMA_CHANNEL_COLLISION_NOT_SUPPORTED = 0x0

} DMA_CHANNEL_COLLISION;

typedef enum {

    DMA_PING_PONG_MODE_NOT_SUPPORTED = 0x0

} DMA_PING_PONG_MODE;

typedef enum {

    DMA_CHANNEL_TRANSFER_DIRECTION_NOT_SUPPORTED = 0

} DMA_CHANNEL_TRANSFER_DIRECTION;

typedef enum {

    DMA_ADDRESS_OFFSET_TYPE_NOT_SUPPORTED = 0

} DMA_ADDRESS_OFFSET_TYPE;

typedef enum {

    DMA_SOURCE_ADDRESSING_MODE_NOT_SUPPORTED = 0

} DMA_SOURCE_ADDRESSING_MODE;

typedef enum {

    DMA_DESTINATION_ADDRESSING_MODE_NOT_SUPPORTED = 0

} DMA_DESTINATION_ADDRESSING_MODE;

typedef enum {

    DMA_CHANNEL_ADDRESSING_MODE_NOT_SUPPORTED = 0

} DMA_CHANNEL_ADDRESSING_MODE;

typedef enum {

    DMA_CHANNEL_DATA_SIZE_NOT_SUPPORTED = 0

} DMA_CHANNEL_DATA_SIZE;

typedef enum {

    DMA_TRANSFER_MODE_NOT_SUPPORTED = 0

} DMA_TRANSFER_MODE;

PLIB_INLINE SFR_TYPE* _DMA_BUSY_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DMACON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_SUSPEND_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DMACON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_ENABLE_CONTROL_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DMACON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNELBITS_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DMASTAT;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_LASTBUSACCESS_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DMASTAT;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_RECENTADDRESS_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DMAADDR;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CRC_CHANNEL_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCRCCON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CRC_TYPE_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCRCCON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CRC_APPEND_MODE_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCRCCON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CRC_ENABLE_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCRCCON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CRC_POLYNOMIAL_LENGTH_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCRCCON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CRC_BIT_ORDER_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCRCCON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CRC_WRITE_BYTE_ORDER_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCRCCON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CRC_BYTE_ORDER_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCRCCON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CRC_DATA_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCRCDATA;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CRC_XOR_ENABLE_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCRCXOR;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_PRIORITY_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0CON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_EVENT_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0CON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_AUTO_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0CON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_CHAIN_ENABLE_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0CON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_DISABLED_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0CON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0CON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_CHAIN_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0CON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_PATTERN_LENGTH_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0CON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_PATTERN_IGNORE_ENABLE_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0CON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_BUSY_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0CON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_PATTERN_IGNORE_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0CON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_TRIGGER_AIRQEN_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0ECON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_TRIGGER_SIRQEN_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0ECON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_TRIGGER_PATEN_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0ECON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_ABORT_TRANSFER_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0ECON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_START_TRANSFER_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0ECON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_STARTIRQ_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0ECON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_ABORTIRQ_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0ECON;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_INTSOURCEFLAG_CHERIF_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0INT;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_INTSOURCEFLAG_CHTAIF_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0INT;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_INTSOURCEFLAG_CHCCIF_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0INT;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_INTSOURCEFLAG_CHBCIF_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0INT;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_INTSOURCEFLAG_CHDHIF_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0INT;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_INTSOURCEFLAG_CHDDIF_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0INT;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_INTSOURCEFLAG_CHSHIF_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0INT;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_INTSOURCEFLAG_CHSDIF_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0INT;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_INTSOURCE_CHERIE_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0INT;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_INTSOURCE_CHTAIE_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0INT;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_INTSOURCE_CHCCIE_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0INT;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_INTSOURCE_CHBCIE_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0INT;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_INTSOURCE_CHDHIE_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0INT;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_INTSOURCE_CHDDIE_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0INT;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_INTSOURCE_CHSHIE_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0INT;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_INTSOURCE_CHSDIE_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0INT;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_SOURCESTARTADDRESS_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0SSA;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_DESTINATIONSTARTADDRESS_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0DSA;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_SOURCESIZE_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0SSIZ;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_DESTINATIONSIZE_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0DSIZ;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_SOURCEPOINTER_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0SPTR;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_DESTINATIONPOINTER_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0DPTR;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_CELLSIZE_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0CSIZ;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_CELLPROGRESSPOINTER_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0CPTR;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _DMA_CHANNEL_X_PATTERNDATA_VREG(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return &DCH0DAT;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_BUSY_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DMACON_DMABUSY_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_SUSPEND_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DMACON_SUSPEND_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_ENABLE_CONTROL_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DMACON_ON_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNELBITS_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DMASTAT_DMACH_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_LASTBUSACCESS_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DMASTAT_RDWR_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_RECENTADDRESS_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)-1;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_CHANNEL_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_CRCCH_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_TYPE_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_CRCTYP_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_APPEND_MODE_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_CRCAPP_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_ENABLE_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_CRCEN_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_POLYNOMIAL_LENGTH_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_PLEN_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_BIT_ORDER_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_BITO_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_WRITE_BYTE_ORDER_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_WBO_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_BYTE_ORDER_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_BYTO_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_DATA_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)-1;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_XOR_ENABLE_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)-1;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_PRIORITY_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHPRI_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_EVENT_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHEDET_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_AUTO_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHAEN_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_CHAIN_ENABLE_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHCHN_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_DISABLED_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHAED_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHEN_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_CHAIN_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHCHNS_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_PATTERN_LENGTH_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHPATLEN_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_PATTERN_IGNORE_ENABLE_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHPIGNEN_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_BUSY_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHBUSY_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_PATTERN_IGNORE_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHPIGN_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_TRIGGER_AIRQEN_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_AIRQEN_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_TRIGGER_SIRQEN_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_SIRQEN_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_TRIGGER_PATEN_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_PATEN_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_ABORT_TRANSFER_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_CABORT_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_START_TRANSFER_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_CFORCE_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_STARTIRQ_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_CHSIRQ_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_ABORTIRQ_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_CHAIRQ_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHERIF_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHERIF_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHTAIF_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHTAIF_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHCCIF_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHCCIF_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHBCIF_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHBCIF_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHDHIF_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHDHIF_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHDDIF_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHDDIF_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHSHIF_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHSHIF_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHSDIF_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHSDIF_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHERIE_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHERIE_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHTAIE_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHTAIE_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHCCIE_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHCCIE_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHBCIE_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHBCIE_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHDHIE_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHDHIE_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHDDIE_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHDDIE_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHSHIE_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHSHIE_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHSDIE_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHSDIE_MASK;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_SOURCESTARTADDRESS_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)-1;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_DESTINATIONSTARTADDRESS_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)-1;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_SOURCESIZE_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)-1;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_DESTINATIONSIZE_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)-1;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_SOURCEPOINTER_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)-1;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_DESTINATIONPOINTER_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)-1;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_CELLSIZE_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)-1;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_CELLPROGRESSPOINTER_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)-1;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_PATTERNDATA_MASK(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)-1;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_BUSY_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DMACON_DMABUSY_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_SUSPEND_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DMACON_SUSPEND_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_ENABLE_CONTROL_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DMACON_ON_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNELBITS_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DMASTAT_DMACH_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_LASTBUSACCESS_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DMASTAT_RDWR_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_RECENTADDRESS_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)0;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_CHANNEL_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_CRCCH_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_TYPE_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_CRCTYP_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_APPEND_MODE_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_CRCAPP_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_ENABLE_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_CRCEN_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_POLYNOMIAL_LENGTH_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_PLEN_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_BIT_ORDER_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_BITO_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_WRITE_BYTE_ORDER_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_WBO_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_BYTE_ORDER_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_BYTO_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_DATA_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)0;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_XOR_ENABLE_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)0;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_PRIORITY_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHPRI_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_EVENT_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHEDET_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_AUTO_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHAEN_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_CHAIN_ENABLE_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHCHN_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_DISABLED_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHAED_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHEN_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_CHAIN_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHCHNS_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_PATTERN_LENGTH_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHPATLEN_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_PATTERN_IGNORE_ENABLE_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHPIGNEN_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_BUSY_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHBUSY_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_PATTERN_IGNORE_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHPIGN_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_TRIGGER_AIRQEN_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_AIRQEN_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_TRIGGER_SIRQEN_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_SIRQEN_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_TRIGGER_PATEN_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_PATEN_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_ABORT_TRANSFER_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_CABORT_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_START_TRANSFER_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_CFORCE_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_STARTIRQ_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_CHSIRQ_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_ABORTIRQ_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_CHAIRQ_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHERIF_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHERIF_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHTAIF_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHTAIF_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHCCIF_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHCCIF_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHBCIF_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHBCIF_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHDHIF_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHDHIF_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHDDIF_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHDDIF_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHSHIF_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHSHIF_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHSDIF_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHSDIF_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHERIE_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHERIE_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHTAIE_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHTAIE_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHCCIE_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHCCIE_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHBCIE_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHBCIE_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHDHIE_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHDHIE_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHDDIE_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHDDIE_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHSHIE_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHSHIE_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHSDIE_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHSDIE_POSITION;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_SOURCESTARTADDRESS_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)0;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_DESTINATIONSTARTADDRESS_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)0;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_SOURCESIZE_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)0;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_DESTINATIONSIZE_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)0;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_SOURCEPOINTER_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)0;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_DESTINATIONPOINTER_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)0;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_CELLSIZE_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)0;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_CELLPROGRESSPOINTER_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)0;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_PATTERNDATA_POS(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)0;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_BUSY_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DMACON_DMABUSY_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_SUSPEND_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DMACON_SUSPEND_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_ENABLE_CONTROL_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DMACON_ON_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNELBITS_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DMASTAT_DMACH_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_LASTBUSACCESS_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DMASTAT_RDWR_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_RECENTADDRESS_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_CHANNEL_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_CRCCH_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_TYPE_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_CRCTYP_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_APPEND_MODE_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_CRCAPP_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_ENABLE_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_CRCEN_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_POLYNOMIAL_LENGTH_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_PLEN_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_BIT_ORDER_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_BITO_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_WRITE_BYTE_ORDER_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_WBO_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_BYTE_ORDER_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCRCCON_BYTO_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_DATA_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CRC_XOR_ENABLE_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_PRIORITY_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHPRI_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_EVENT_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHEDET_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_AUTO_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHAEN_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_CHAIN_ENABLE_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHCHN_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_DISABLED_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHAED_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHEN_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_CHAIN_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHCHNS_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_PATTERN_LENGTH_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHPATLEN_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_PATTERN_IGNORE_ENABLE_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHPIGNEN_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_BUSY_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHBUSY_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_PATTERN_IGNORE_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0CON_CHPIGN_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_TRIGGER_AIRQEN_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_AIRQEN_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_TRIGGER_SIRQEN_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_SIRQEN_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_TRIGGER_PATEN_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_PATEN_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_ABORT_TRANSFER_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_CABORT_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_START_TRANSFER_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_CFORCE_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_STARTIRQ_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_CHSIRQ_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_ABORTIRQ_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0ECON_CHAIRQ_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHERIF_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHERIF_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHTAIF_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHTAIF_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHCCIF_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHCCIF_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHBCIF_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHBCIF_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHDHIF_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHDHIF_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHDDIF_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHDDIF_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHSHIF_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHSHIF_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCEFLAG_CHSDIF_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHSDIF_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHERIE_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHERIE_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHTAIE_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHTAIE_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHCCIE_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHCCIE_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHBCIE_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHBCIE_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHDHIE_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHDHIE_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHDDIE_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHDDIE_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHSHIE_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHSHIE_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_INTSOURCE_CHSDIE_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return _DCH0INT_CHSDIE_LENGTH;
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_SOURCESTARTADDRESS_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_DESTINATIONSTARTADDRESS_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_SOURCESIZE_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_DESTINATIONSIZE_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_SOURCEPOINTER_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_DESTINATIONPOINTER_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_CELLSIZE_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_CELLPROGRESSPOINTER_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _DMA_CHANNEL_X_PATTERNDATA_LEN(DMA_MODULE_ID i)
{
    switch (i) {
        case DMA_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/dma_Busy_Default.h"
#include "../templates/dma_Suspend_Default.h"
#include "../templates/dma_StopInIdle_Unsupported.h"
#include "../templates/dma_EnableControl_Default.h"
#include "../templates/dma_ChannelBits_Default.h"
#include "../templates/dma_LastBusAccess_Default.h"
#include "../templates/dma_RecentAddress_Default.h"
#include "../templates/dma_CRCChannel_Default.h"
#include "../templates/dma_CRCType_Default.h"
#include "../templates/dma_CRCAppendMode_Default.h"
#include "../templates/dma_Crc_Default.h"
#include "../templates/dma_CRCPolynomialLength_Default.h"
#include "../templates/dma_CRCBitOrder_Default.h"
#include "../templates/dma_CRCWriteByteOrder_Default.h"
#include "../templates/dma_CRCByteOrder_Default.h"
#include "../templates/dma_CRCData_Default.h"
#include "../templates/dma_CRCXOREnable_Default.h"
#include "../templates/dma_ChannelXPriority_Default.h"
#include "../templates/dma_ChannelXEvent_Default.h"
#include "../templates/dma_ChannelXAuto_Default.h"
#include "../templates/dma_ChannelXChainEnbl_Default.h"
#include "../templates/dma_ChannelXDisabled_Default.h"
#include "../templates/dma_ChannelX_Default.h"
#include "../templates/dma_ChannelXChain_Default.h"
#include "../templates/dma_ChannelXPatternLength_Default.h"
#include "../templates/dma_ChannelXPatternIgnoreByte_Default.h"
#include "../templates/dma_ChannelXBusy_Default.h"
#include "../templates/dma_ChannelXPatternIgnore_Default.h"
#include "../templates/dma_ChannelXTrigger_Default.h"
#include "../templates/dma_AbortTransfer_Default.h"
#include "../templates/dma_StartTransfer_Default.h"
#include "../templates/dma_ChannelXStartIRQ_Default.h"
#include "../templates/dma_ChannelXAbortIRQ_Default.h"
#include "../templates/dma_ChannelXINTSourceFlag_Default.h"
#include "../templates/dma_ChannelXINTSource_Default.h"
#include "../templates/dma_ChannelXSourceStartAddress_Default.h"
#include "../templates/dma_ChannelXDestinationStartAddress_Default.h"
#include "../templates/dma_ChannelXSourceSize_Default.h"
#include "../templates/dma_ChannelXDestinationSize_Default.h"
#include "../templates/dma_ChannelXSourcePointer_Default.h"
#include "../templates/dma_ChannelXDestinationPointer_Default.h"
#include "../templates/dma_ChannelXCellSize_Default.h"
#include "../templates/dma_ChannelXCellProgressPointer_Default.h"
#include "../templates/dma_ChannelXPatternData_Default.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API bool PLIB_DMA_ExistsBusy(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsBusy_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_BusyActiveSet(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_BusyActiveSet_Default(index);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_DMA_BusyActiveReset(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_BusyActiveReset_Default(index);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_IsBusy(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_IsBusy_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsSuspend(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsSuspend_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_SuspendEnable(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_SuspendEnable_Default(index);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_DMA_SuspendDisable(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_SuspendDisable_Default(index);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_SuspendIsEnabled(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_SuspendIsEnabled_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsStopInIdle(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsStopInIdle_Unsupported(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_DMA_StopInIdleEnable(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_StopInIdleEnable_Unsupported(index);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_DMA_StopInIdleDisable(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_StopInIdleDisable_Unsupported(index);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsEnableControl(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsEnableControl_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_Enable(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_Enable_Default(index);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_DMA_Disable(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_Disable_Default(index);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_IsEnabled(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_IsEnabled_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelBits(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelBits_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint8_t PLIB_DMA_ChannelBitsGet(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelBitsGet_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsLastBusAccess(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsLastBusAccess_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_LastBusAccessIsRead(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_LastBusAccessIsRead_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_LastBusAccessIsWrite(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_LastBusAccessIsWrite_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsRecentAddress(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsRecentAddress_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint32_t PLIB_DMA_RecentAddressAccessed(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_RecentAddressAccessed_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (uint32_t)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsCRCChannel(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsCRCChannel_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_CRCChannelSelect(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_CRCChannelSelect_Default(index, channel);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API DMA_CHANNEL PLIB_DMA_CRCChannelGet(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_CRCChannelGet_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (DMA_CHANNEL)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsCRCType(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsCRCType_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API DMA_CRC_TYPE PLIB_DMA_CRCTypeGet(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_CRCTypeGet_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (DMA_CRC_TYPE)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_CRCTypeSet(DMA_MODULE_ID index, DMA_CRC_TYPE CRCType)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_CRCTypeSet_Default(index, CRCType);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsCRCAppendMode(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsCRCAppendMode_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_CRCAppendModeEnable(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_CRCAppendModeEnable_Default(index);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_DMA_CRCAppendModeDisable(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_CRCAppendModeDisable_Default(index);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_CRCAppendModeIsEnabled(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_CRCAppendModeIsEnabled_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsCRC(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsCRC_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_CRCEnable(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_CRCEnable_Default(index);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_DMA_CRCDisable(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_CRCDisable_Default(index);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_CRCIsEnabled(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_CRCIsEnabled_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsCRCPolynomialLength(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsCRCPolynomialLength_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_CRCPolynomialLengthSet(DMA_MODULE_ID index, uint8_t polyLength)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_CRCPolynomialLengthSet_Default(index, polyLength);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint8_t PLIB_DMA_CRCPolynomialLengthGet(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_CRCPolynomialLengthGet_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsCRCBitOrder(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsCRCBitOrder_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_CRCBitOrderSelect(DMA_MODULE_ID index, DMA_CRC_BIT_ORDER bitOrder)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_CRCBitOrderSelect_Default(index, bitOrder);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsCRCWriteByteOrder(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsCRCWriteByteOrder_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_CRCWriteByteOrderAlter(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_CRCWriteByteOrderAlter_Default(index);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_DMA_CRCWriteByteOrderMaintain(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_CRCWriteByteOrderMaintain_Default(index);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsCRCByteOrder(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsCRCByteOrder_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_CRCByteOrderSelect(DMA_MODULE_ID index, DMA_CRC_BYTE_ORDER byteOrder)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_CRCByteOrderSelect_Default(index, byteOrder);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API DMA_CRC_BYTE_ORDER PLIB_DMA_CRCByteOrderGet(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_CRCByteOrderGet_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (DMA_CRC_BYTE_ORDER)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsCRCData(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsCRCData_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint32_t PLIB_DMA_CRCDataRead(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_CRCDataRead_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (uint32_t)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_CRCDataWrite(DMA_MODULE_ID index, uint32_t DMACRCdata)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_CRCDataWrite_Default(index, DMACRCdata);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsCRCXOREnable(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsCRCXOREnable_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_CRCXOREnableSet(DMA_MODULE_ID index, uint32_t DMACRCXOREnableMask)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_CRCXOREnableSet_Default(index, DMACRCXOREnableMask);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint32_t PLIB_DMA_CRCXOREnableGet(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_CRCXOREnableGet_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (uint32_t)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXPriority(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXPriority_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXPrioritySelect(DMA_MODULE_ID index, DMA_CHANNEL channel, DMA_CHANNEL_PRIORITY channelPriority)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXPrioritySelect_Default(index, channel, channelPriority);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API DMA_CHANNEL_PRIORITY PLIB_DMA_ChannelXPriorityGet(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXPriorityGet_Default(index, channel);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (DMA_CHANNEL_PRIORITY)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXEvent(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXEvent_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ChannelXEventIsDetected(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXEventIsDetected_Default(index, channel);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXAuto(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXAuto_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXAutoEnable(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXAutoEnable_Default(index, channel);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXAutoDisable(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXAutoDisable_Default(index, channel);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ChannelXAutoIsEnabled(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXAutoIsEnabled_Default(index, channel);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXChainEnbl(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXChainEnbl_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXChainEnable(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXChainEnable_Default(index, channel);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXChainDisable(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXChainDisable_Default(index, channel);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ChannelXChainIsEnabled(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXChainIsEnabled_Default(index, channel);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXDisabled(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXDisabled_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXDisabledEnablesEvents(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXDisabledEnablesEvents_Default(index, channel);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXDisabledDisablesEvents(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXDisabledDisablesEvents_Default(index, channel);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelX(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelX_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXEnable(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXEnable_Default(index, channel);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ChannelXIsEnabled(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXIsEnabled_Default(index, channel);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXDisable(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXDisable_Default(index, channel);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXChain(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXChain_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXChainToLower(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXChainToLower_Default(index, channel);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXChainToHigher(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXChainToHigher_Default(index, channel);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXPatternLength(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXPatternLength_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXPatternLengthSet(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel, DMA_PATTERN_LENGTH patternLen)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXPatternLengthSet_Default(index, dmaChannel, patternLen);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API DMA_PATTERN_LENGTH PLIB_DMA_ChannelXPatternLengthGet(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXPatternLengthGet_Default(index, dmaChannel);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (DMA_PATTERN_LENGTH)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXPatternIgnoreByte(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXPatternIgnoreByte_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXPatternIgnoreByteEnable(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXPatternIgnoreByteEnable_Default(index, channel);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ChannelXPatternIgnoreByteIsEnabled(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXPatternIgnoreByteIsEnabled_Default(index, channel);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXPatternIgnoreByteDisable(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXPatternIgnoreByteDisable_Default(index, channel);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXBusy(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXBusy_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXBusyActiveSet(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXBusyActiveSet_Default(index, channel);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXBusyInActiveSet(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXBusyInActiveSet_Default(index, channel);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ChannelXBusyIsBusy(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXBusyIsBusy_Default(index, channel);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXPatternIgnore(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXPatternIgnore_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXPatternIgnoreSet(DMA_MODULE_ID index, DMA_CHANNEL channel, uint8_t pattern)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXPatternIgnoreSet_Default(index, channel, pattern);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint8_t PLIB_DMA_ChannelXPatternIgnoreGet(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXPatternIgnoreGet_Default(index, channel);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXTrigger(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXTrigger_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXTriggerEnable(DMA_MODULE_ID index, DMA_CHANNEL channel, DMA_CHANNEL_TRIGGER_TYPE trigger)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXTriggerEnable_Default(index, channel, trigger);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ChannelXTriggerIsEnabled(DMA_MODULE_ID index, DMA_CHANNEL channel, DMA_CHANNEL_TRIGGER_TYPE trigger)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXTriggerIsEnabled_Default(index, channel, trigger);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXTriggerDisable(DMA_MODULE_ID index, DMA_CHANNEL channel, DMA_CHANNEL_TRIGGER_TYPE trigger)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXTriggerDisable_Default(index, channel, trigger);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API DMA_CHANNEL_INT_SOURCE PLIB_DMA_ChannelXTriggerSourceNumberGet(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXTriggerSourceNumberGet_Default(index, channel);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (DMA_CHANNEL_INT_SOURCE)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsAbortTransfer(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsAbortTransfer_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_AbortTransferSet(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_AbortTransferSet_Default(index, channel);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsStartTransfer(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsStartTransfer_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_StartTransferSet(DMA_MODULE_ID index, DMA_CHANNEL channel)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_StartTransferSet_Default(index, channel);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXStartIRQ(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXStartIRQ_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXStartIRQSet(DMA_MODULE_ID index, DMA_CHANNEL channel, DMA_TRIGGER_SOURCE IRQnum)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXStartIRQSet_Default(index, channel, IRQnum);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXAbortIRQ(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXAbortIRQ_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXAbortIRQSet(DMA_MODULE_ID index, DMA_CHANNEL channel, DMA_TRIGGER_SOURCE IRQ)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXAbortIRQSet_Default(index, channel, IRQ);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXINTSourceFlag(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXINTSourceFlag_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ChannelXINTSourceFlagGet(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel, DMA_INT_TYPE dmaINTSource)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXINTSourceFlagGet_Default(index, dmaChannel, dmaINTSource);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXINTSourceFlagSet(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel, DMA_INT_TYPE dmaINTSource)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXINTSourceFlagSet_Default(index, dmaChannel, dmaINTSource);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXINTSourceFlagClear(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel, DMA_INT_TYPE dmaINTSource)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXINTSourceFlagClear_Default(index, dmaChannel, dmaINTSource);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXINTSource(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXINTSource_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXINTSourceEnable(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel, DMA_INT_TYPE dmaINTSource)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXINTSourceEnable_Default(index, dmaChannel, dmaINTSource);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXINTSourceDisable(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel, DMA_INT_TYPE dmaINTSource)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXINTSourceDisable_Default(index, dmaChannel, dmaINTSource);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ChannelXINTSourceIsEnabled(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel, DMA_INT_TYPE dmaINTSource)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXINTSourceIsEnabled_Default(index, dmaChannel, dmaINTSource);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXSourceStartAddress(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXSourceStartAddress_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint32_t PLIB_DMA_ChannelXSourceStartAddressGet(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXSourceStartAddressGet_Default(index, dmaChannel);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (uint32_t)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXSourceStartAddressSet(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel, uint32_t sourceStartAddress)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXSourceStartAddressSet_Default(index, dmaChannel, sourceStartAddress);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXDestinationStartAddress(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXDestinationStartAddress_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint32_t PLIB_DMA_ChannelXDestinationStartAddressGet(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXDestinationStartAddressGet_Default(index, dmaChannel);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (uint32_t)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXDestinationStartAddressSet(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel, uint32_t destinationStartAddress)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXDestinationStartAddressSet_Default(index, dmaChannel, destinationStartAddress);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXSourceSize(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXSourceSize_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint16_t PLIB_DMA_ChannelXSourceSizeGet(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXSourceSizeGet_Default(index, dmaChannel);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXSourceSizeSet(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel, uint16_t sourceSize)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXSourceSizeSet_Default(index, dmaChannel, sourceSize);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXDestinationSize(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXDestinationSize_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint16_t PLIB_DMA_ChannelXDestinationSizeGet(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXDestinationSizeGet_Default(index, dmaChannel);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXDestinationSizeSet(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel, uint16_t destinationSize)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXDestinationSizeSet_Default(index, dmaChannel, destinationSize);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXSourcePointer(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXSourcePointer_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint16_t PLIB_DMA_ChannelXSourcePointerGet(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXSourcePointerGet_Default(index, dmaChannel);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXDestinationPointer(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXDestinationPointer_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint16_t PLIB_DMA_ChannelXDestinationPointerGet(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXDestinationPointerGet_Default(index, dmaChannel);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXCellSize(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXCellSize_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint16_t PLIB_DMA_ChannelXCellSizeGet(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXCellSizeGet_Default(index, dmaChannel);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXCellSizeSet(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel, uint16_t CellSize)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXCellSizeSet_Default(index, dmaChannel, CellSize);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXCellProgressPointer(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXCellProgressPointer_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint16_t PLIB_DMA_ChannelXCellProgressPointerGet(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXCellProgressPointerGet_Default(index, dmaChannel);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API bool PLIB_DMA_ExistsChannelXPatternData(DMA_MODULE_ID index)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ExistsChannelXPatternData_Default(index);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint16_t PLIB_DMA_ChannelXPatternDataGet(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel)
{
    switch (index) {
        case DMA_ID_0 :
            return DMA_ChannelXPatternDataGet_Default(index, dmaChannel);
        case DMA_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API void PLIB_DMA_ChannelXPatternDataSet(DMA_MODULE_ID index, DMA_CHANNEL dmaChannel, uint16_t patternData)
{
    switch (index) {
        case DMA_ID_0 :
            DMA_ChannelXPatternDataSet_Default(index, dmaChannel, patternData);
            break;
        case DMA_NUMBER_OF_MODULES :
        default :
            break;
    }
}

#endif
