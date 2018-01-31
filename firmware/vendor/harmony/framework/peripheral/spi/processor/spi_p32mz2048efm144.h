/* Created by plibgen $Revision: 1.31 $ */

#ifndef _SPI_P32MZ2048EFM144_H
#define _SPI_P32MZ2048EFM144_H

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

    SPI_ID_1 = 0,
    SPI_ID_2,
    SPI_ID_3,
    SPI_ID_4,
    SPI_ID_5,
    SPI_ID_6,
    SPI_NUMBER_OF_MODULES

} SPI_MODULE_ID;

typedef enum {

    SPI_INPUT_SAMPLING_PHASE_IN_MIDDLE = 0,
    SPI_INPUT_SAMPLING_PHASE_AT_END = 1

} SPI_INPUT_SAMPLING_PHASE;

typedef enum {

    SPI_OUTPUT_DATA_PHASE_ON_IDLE_TO_ACTIVE_CLOCK = 0,
    SPI_OUTPUT_DATA_PHASE_ON_ACTIVE_TO_IDLE_CLOCK = 1

} SPI_OUTPUT_DATA_PHASE;

typedef enum {

    SPI_COMMUNICATION_WIDTH_8BITS = 0,
    SPI_COMMUNICATION_WIDTH_16BITS = 1,
    SPI_COMMUNICATION_WIDTH_32BITS = 2

} SPI_COMMUNICATION_WIDTH;

typedef enum {

    SPI_CLOCK_POLARITY_IDLE_LOW = 0,
    SPI_CLOCK_POLARITY_IDLE_HIGH = 1

} SPI_CLOCK_POLARITY;

typedef enum {

    SPI_FIFO_INTERRUPT_WHEN_TRANSMIT_BUFFER_IS_NOT_FULL = 0,
    SPI_FIFO_INTERRUPT_WHEN_TRANSMIT_BUFFER_IS_1HALF_EMPTY_OR_MORE = 1,
    SPI_FIFO_INTERRUPT_WHEN_TRANSMIT_BUFFER_IS_COMPLETELY_EMPTY = 2,
    SPI_FIFO_INTERRUPT_WHEN_TRANSMISSION_IS_COMPLETE = 3,
    SPI_FIFO_INTERRUPT_WHEN_RECEIVE_BUFFER_IS_FULL = 4,
    SPI_FIFO_INTERRUPT_WHEN_RECEIVE_BUFFER_IS_1HALF_FULL_OR_MORE = 5,
    SPI_FIFO_INTERRUPT_WHEN_RECEIVE_BUFFER_IS_NOT_EMPTY = 6,
    SPI_FIFO_INTERRUPT_WHEN_BUFFER_IS_EMPTY = 7

} SPI_FIFO_INTERRUPT;

typedef enum {

    SPI_ERROR_INTERRUPT_FRAME_ERROR_OVERFLOW = 0,
    SPI_ERROR_INTERRUPT_RECEIVE_OVERFLOW = 1,
    SPI_ERROR_INTERRUPT_TRANSMIT_UNDERRUN = 2

} SPI_ERROR_INTERRUPT;

typedef enum {

    SPI_PIN_SLAVE_SELECT = 0,
    SPI_PIN_DATA_IN = 1,
    SPI_PIN_DATA_OUT = 2

} SPI_PIN;

typedef enum {

    SPI_FIFO_TYPE_RECEIVE = 0,
    SPI_FIFO_TYPE_TRANSMIT = 1

} SPI_FIFO_TYPE;

typedef enum {

    SPI_BAUD_RATE_PBCLK_CLOCK = 0,
    SPI_BAUD_RATE_MCLK_CLOCK = 1

} SPI_BAUD_RATE_CLOCK;

typedef enum {

    SPI_FRAME_SYNC_PULSE_ON_EVERY_DATA_CHARACTER = 0,
    SPI_FRAME_SYNC_PULSE_ON_EVERY_2_DATA_CHARACTER = 1,
    SPI_FRAME_SYNC_PULSE_ON_EVERY_4_DATA_CHARACTER = 2,
    SPI_FRAME_SYNC_PULSE_ON_EVERY_8_DATA_CHARACTER = 3,
    SPI_FRAME_SYNC_PULSE_ON_EVERY_16_DATA_CHARACTER = 4,
    SPI_FRAME_SYNC_PULSE_ON_EVERY_32_DATA_CHARACTER = 5

} SPI_FRAME_SYNC_PULSE;

typedef enum {

    SPI_FRAME_PULSE_POLARITY_ACTIVE_LOW = 0,
    SPI_FRAME_PULSE_POLARITY_ACTIVE_HIGH = 1

} SPI_FRAME_PULSE_POLARITY;

typedef enum {

    SPI_FRAME_PULSE_DIRECTION_OUTPUT = 0,
    SPI_FRAME_PULSE_DIRECTION_INPUT = 1

} SPI_FRAME_PULSE_DIRECTION;

typedef enum {

    SPI_FRAME_PULSE_EDGE_PRECEDES_FIRST_BIT_CLOCK = 0,
    SPI_FRAME_PULSE_EDGE_COINCIDES_FIRST_BIT_CLOCK = 1

} SPI_FRAME_PULSE_EDGE;

typedef enum {

    SPI_FRAME_PULSE_WIDTH_ONE_CLOCK_WIDE = 0,
    SPI_FRAME_PULSE_WIDTH_ONE_WORD_LENGTH = 1

} SPI_FRAME_PULSE_WIDTH;

typedef enum {

    SPI_AUDIO_PROTOCOL_I2S = 0,
    SPI_AUDIO_PROTOCOL_LEFT_JUSTIFIED = 1,
    SPI_AUDIO_PROTOCOL_RIGHT_JUSTIFIED = 2,
    SPI_AUDIO_PROTOCOL_PCM_DSP = 3

} SPI_AUDIO_PROTOCOL;

typedef enum {

    SPI_AUDIO_TRANSMIT_STEREO = 0,
    SPI_AUDIO_TRANSMIT_MONO = 1

} SPI_AUDIO_TRANSMIT_MODE;

typedef enum {

    SPI_AUDIO_ERROR_RECEIVE_OVERFLOW = 0,
    SPI_AUDIO_ERROR_TRANSMIT_UNDERRUN = 1

} SPI_AUDIO_ERROR;

typedef enum {

    SPI_AUDIO_COMMUNICATION_16DATA_16FIFO_16CHANNEL = 0,
    SPI_AUDIO_COMMUNICATION_16DATA_16FIFO_32CHANNEL = 1,
    SPI_AUDIO_COMMUNICATION_32DATA_32FIFO_32CHANNEL = 2,
    SPI_AUDIO_COMMUNICATION_24DATA_32FIFO_32CHANNEL = 3

} SPI_AUDIO_COMMUNICATION_WIDTH;

PLIB_INLINE SFR_TYPE* _SPI_FRAMED_COMMUNICATION_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_FRAME_SYNC_PULSE_DIRECTION_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_FRAME_SYNC_PULSE_POLARITY_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_SLAVE_SELECT_CONTROL_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_FRAME_SYNC_PULSE_WIDTH_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_FRAME_SYNC_PULSE_COUNTER_PIC32_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_BAUD_RATE_CLOCK_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_FRAME_SYNC_PULSE_EDGE_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_FIFO_CONTROL_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_ENABLE_CONTROL_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_STOP_IN_IDLE_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_PIN_CONTROL_SDO_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_MODE32_COMMUNICATION_WIDTH_PIC32_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_MODE32_AUDIO_COMMUNICATION_WIDTH_PIC32_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_MODE16_COMMUNICATION_WIDTH_PIC32_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_MODE16_AUDIO_COMMUNICATION_WIDTH_PIC32_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_INPUT_SAMPLE_PHASE_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_OUTPUT_DATA_PHASE_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_PIN_CONTROL_SS_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_CLOCK_POLARITY_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_MASTER_CONTROL_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_PIN_CONTROL_SDI_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON;
        case SPI_ID_2 :
            return &SPI2CON;
        case SPI_ID_3 :
            return &SPI3CON;
        case SPI_ID_4 :
            return &SPI4CON;
        case SPI_ID_5 :
            return &SPI5CON;
        case SPI_ID_6 :
            return &SPI6CON;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_RX_FIFO_COUNT_PIC32_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1STAT;
        case SPI_ID_2 :
            return &SPI2STAT;
        case SPI_ID_3 :
            return &SPI3STAT;
        case SPI_ID_4 :
            return &SPI4STAT;
        case SPI_ID_5 :
            return &SPI5STAT;
        case SPI_ID_6 :
            return &SPI6STAT;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_TX_FIFO_COUNT_PIC32_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1STAT;
        case SPI_ID_2 :
            return &SPI2STAT;
        case SPI_ID_3 :
            return &SPI3STAT;
        case SPI_ID_4 :
            return &SPI4STAT;
        case SPI_ID_5 :
            return &SPI5STAT;
        case SPI_ID_6 :
            return &SPI6STAT;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_FRAME_ERROR_STATUS_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1STAT;
        case SPI_ID_2 :
            return &SPI2STAT;
        case SPI_ID_3 :
            return &SPI3STAT;
        case SPI_ID_4 :
            return &SPI4STAT;
        case SPI_ID_5 :
            return &SPI5STAT;
        case SPI_ID_6 :
            return &SPI6STAT;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_BUS_STATUS_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1STAT;
        case SPI_ID_2 :
            return &SPI2STAT;
        case SPI_ID_3 :
            return &SPI3STAT;
        case SPI_ID_4 :
            return &SPI4STAT;
        case SPI_ID_5 :
            return &SPI5STAT;
        case SPI_ID_6 :
            return &SPI6STAT;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_TRANSMIT_UNDER_RUN_STATUS_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1STAT;
        case SPI_ID_2 :
            return &SPI2STAT;
        case SPI_ID_3 :
            return &SPI3STAT;
        case SPI_ID_4 :
            return &SPI4STAT;
        case SPI_ID_5 :
            return &SPI5STAT;
        case SPI_ID_6 :
            return &SPI6STAT;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_FIFO_REGISTER_EMPTY_STATUS_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1STAT;
        case SPI_ID_2 :
            return &SPI2STAT;
        case SPI_ID_3 :
            return &SPI3STAT;
        case SPI_ID_4 :
            return &SPI4STAT;
        case SPI_ID_5 :
            return &SPI5STAT;
        case SPI_ID_6 :
            return &SPI6STAT;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_RECEIVER_OVERFLOW_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1STAT;
        case SPI_ID_2 :
            return &SPI2STAT;
        case SPI_ID_3 :
            return &SPI3STAT;
        case SPI_ID_4 :
            return &SPI4STAT;
        case SPI_ID_5 :
            return &SPI5STAT;
        case SPI_ID_6 :
            return &SPI6STAT;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_RECEIVE_FIFO_STATUS_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1STAT;
        case SPI_ID_2 :
            return &SPI2STAT;
        case SPI_ID_3 :
            return &SPI3STAT;
        case SPI_ID_4 :
            return &SPI4STAT;
        case SPI_ID_5 :
            return &SPI5STAT;
        case SPI_ID_6 :
            return &SPI6STAT;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_TRANSMIT_BUFFER_EMPTY_STATUS_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1STAT;
        case SPI_ID_2 :
            return &SPI2STAT;
        case SPI_ID_3 :
            return &SPI3STAT;
        case SPI_ID_4 :
            return &SPI4STAT;
        case SPI_ID_5 :
            return &SPI5STAT;
        case SPI_ID_6 :
            return &SPI6STAT;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_TRANSMIT_BUFFER_FULL_STATUS_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1STAT;
        case SPI_ID_2 :
            return &SPI2STAT;
        case SPI_ID_3 :
            return &SPI3STAT;
        case SPI_ID_4 :
            return &SPI4STAT;
        case SPI_ID_5 :
            return &SPI5STAT;
        case SPI_ID_6 :
            return &SPI6STAT;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_RECEIVE_BUFFER_STATUS_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1STAT;
        case SPI_ID_2 :
            return &SPI2STAT;
        case SPI_ID_3 :
            return &SPI3STAT;
        case SPI_ID_4 :
            return &SPI4STAT;
        case SPI_ID_5 :
            return &SPI5STAT;
        case SPI_ID_6 :
            return &SPI6STAT;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_READ_DATA_SIGN_STATUS_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON2;
        case SPI_ID_2 :
            return &SPI2CON2;
        case SPI_ID_3 :
            return &SPI3CON2;
        case SPI_ID_4 :
            return &SPI4CON2;
        case SPI_ID_5 :
            return &SPI5CON2;
        case SPI_ID_6 :
            return &SPI6CON2;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_FRAME_ERROR_INTERRUPT_CONTROL_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON2;
        case SPI_ID_2 :
            return &SPI2CON2;
        case SPI_ID_3 :
            return &SPI3CON2;
        case SPI_ID_4 :
            return &SPI4CON2;
        case SPI_ID_5 :
            return &SPI5CON2;
        case SPI_ID_6 :
            return &SPI6CON2;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_RX_OVERFLOW_ERROR_INTERRUPT_CONTROL_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON2;
        case SPI_ID_2 :
            return &SPI2CON2;
        case SPI_ID_3 :
            return &SPI3CON2;
        case SPI_ID_4 :
            return &SPI4CON2;
        case SPI_ID_5 :
            return &SPI5CON2;
        case SPI_ID_6 :
            return &SPI6CON2;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_TX_UNDERRUN_ERROR_INTERRUPT_CONTROL_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON2;
        case SPI_ID_2 :
            return &SPI2CON2;
        case SPI_ID_3 :
            return &SPI3CON2;
        case SPI_ID_4 :
            return &SPI4CON2;
        case SPI_ID_5 :
            return &SPI5CON2;
        case SPI_ID_6 :
            return &SPI6CON2;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_RX_OVERFLOW_AUDIO_ERROR_CONTROL_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON2;
        case SPI_ID_2 :
            return &SPI2CON2;
        case SPI_ID_3 :
            return &SPI3CON2;
        case SPI_ID_4 :
            return &SPI4CON2;
        case SPI_ID_5 :
            return &SPI5CON2;
        case SPI_ID_6 :
            return &SPI6CON2;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_TX_UNDERRUN_AUDIO_ERROR_CONTROL_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON2;
        case SPI_ID_2 :
            return &SPI2CON2;
        case SPI_ID_3 :
            return &SPI3CON2;
        case SPI_ID_4 :
            return &SPI4CON2;
        case SPI_ID_5 :
            return &SPI5CON2;
        case SPI_ID_6 :
            return &SPI6CON2;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_AUDIO_PROTOCOL_CONTROL_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON2;
        case SPI_ID_2 :
            return &SPI2CON2;
        case SPI_ID_3 :
            return &SPI3CON2;
        case SPI_ID_4 :
            return &SPI4CON2;
        case SPI_ID_5 :
            return &SPI5CON2;
        case SPI_ID_6 :
            return &SPI6CON2;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_AUDIO_TRANSMIT_MODE_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON2;
        case SPI_ID_2 :
            return &SPI2CON2;
        case SPI_ID_3 :
            return &SPI3CON2;
        case SPI_ID_4 :
            return &SPI4CON2;
        case SPI_ID_5 :
            return &SPI5CON2;
        case SPI_ID_6 :
            return &SPI6CON2;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_AUDIO_PROTOCOL_MODE_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1CON2;
        case SPI_ID_2 :
            return &SPI2CON2;
        case SPI_ID_3 :
            return &SPI3CON2;
        case SPI_ID_4 :
            return &SPI4CON2;
        case SPI_ID_5 :
            return &SPI5CON2;
        case SPI_ID_6 :
            return &SPI6CON2;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_BUFFER_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1BUF;
        case SPI_ID_2 :
            return &SPI2BUF;
        case SPI_ID_3 :
            return &SPI3BUF;
        case SPI_ID_4 :
            return &SPI4BUF;
        case SPI_ID_5 :
            return &SPI5BUF;
        case SPI_ID_6 :
            return &SPI6BUF;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _SPI_BAUD_RATE_VREG(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return &SPI1BRG;
        case SPI_ID_2 :
            return &SPI2BRG;
        case SPI_ID_3 :
            return &SPI3BRG;
        case SPI_ID_4 :
            return &SPI4BRG;
        case SPI_ID_5 :
            return &SPI5BRG;
        case SPI_ID_6 :
            return &SPI6BRG;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAMED_COMMUNICATION_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_FRMEN_MASK;
        case SPI_ID_2 :
            return _SPI2CON_FRMEN_MASK;
        case SPI_ID_3 :
            return _SPI3CON_FRMEN_MASK;
        case SPI_ID_4 :
            return _SPI4CON_FRMEN_MASK;
        case SPI_ID_5 :
            return _SPI5CON_FRMEN_MASK;
        case SPI_ID_6 :
            return _SPI6CON_FRMEN_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_SYNC_PULSE_DIRECTION_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_FRMSYNC_MASK;
        case SPI_ID_2 :
            return _SPI2CON_FRMSYNC_MASK;
        case SPI_ID_3 :
            return _SPI3CON_FRMSYNC_MASK;
        case SPI_ID_4 :
            return _SPI4CON_FRMSYNC_MASK;
        case SPI_ID_5 :
            return _SPI5CON_FRMSYNC_MASK;
        case SPI_ID_6 :
            return _SPI6CON_FRMSYNC_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_SYNC_PULSE_POLARITY_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_FRMPOL_MASK;
        case SPI_ID_2 :
            return _SPI2CON_FRMPOL_MASK;
        case SPI_ID_3 :
            return _SPI3CON_FRMPOL_MASK;
        case SPI_ID_4 :
            return _SPI4CON_FRMPOL_MASK;
        case SPI_ID_5 :
            return _SPI5CON_FRMPOL_MASK;
        case SPI_ID_6 :
            return _SPI6CON_FRMPOL_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_SLAVE_SELECT_CONTROL_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MSSEN_MASK;
        case SPI_ID_2 :
            return _SPI2CON_MSSEN_MASK;
        case SPI_ID_3 :
            return _SPI3CON_MSSEN_MASK;
        case SPI_ID_4 :
            return _SPI4CON_MSSEN_MASK;
        case SPI_ID_5 :
            return _SPI5CON_MSSEN_MASK;
        case SPI_ID_6 :
            return _SPI6CON_MSSEN_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_SYNC_PULSE_WIDTH_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_FRMSYPW_MASK;
        case SPI_ID_2 :
            return _SPI2CON_FRMSYPW_MASK;
        case SPI_ID_3 :
            return _SPI3CON_FRMSYPW_MASK;
        case SPI_ID_4 :
            return _SPI4CON_FRMSYPW_MASK;
        case SPI_ID_5 :
            return _SPI5CON_FRMSYPW_MASK;
        case SPI_ID_6 :
            return _SPI6CON_FRMSYPW_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_SYNC_PULSE_COUNTER_PIC32_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_FRMCNT_MASK;
        case SPI_ID_2 :
            return _SPI2CON_FRMCNT_MASK;
        case SPI_ID_3 :
            return _SPI3CON_FRMCNT_MASK;
        case SPI_ID_4 :
            return _SPI4CON_FRMCNT_MASK;
        case SPI_ID_5 :
            return _SPI5CON_FRMCNT_MASK;
        case SPI_ID_6 :
            return _SPI6CON_FRMCNT_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_BAUD_RATE_CLOCK_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MCLKSEL_MASK;
        case SPI_ID_2 :
            return _SPI2CON_MCLKSEL_MASK;
        case SPI_ID_3 :
            return _SPI3CON_MCLKSEL_MASK;
        case SPI_ID_4 :
            return _SPI4CON_MCLKSEL_MASK;
        case SPI_ID_5 :
            return _SPI5CON_MCLKSEL_MASK;
        case SPI_ID_6 :
            return _SPI6CON_MCLKSEL_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_SYNC_PULSE_EDGE_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_SPIFE_MASK;
        case SPI_ID_2 :
            return _SPI2CON_SPIFE_MASK;
        case SPI_ID_3 :
            return _SPI3CON_SPIFE_MASK;
        case SPI_ID_4 :
            return _SPI4CON_SPIFE_MASK;
        case SPI_ID_5 :
            return _SPI5CON_SPIFE_MASK;
        case SPI_ID_6 :
            return _SPI6CON_SPIFE_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FIFO_CONTROL_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_ENHBUF_MASK;
        case SPI_ID_2 :
            return _SPI2CON_ENHBUF_MASK;
        case SPI_ID_3 :
            return _SPI3CON_ENHBUF_MASK;
        case SPI_ID_4 :
            return _SPI4CON_ENHBUF_MASK;
        case SPI_ID_5 :
            return _SPI5CON_ENHBUF_MASK;
        case SPI_ID_6 :
            return _SPI6CON_ENHBUF_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_ENABLE_CONTROL_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_ON_MASK;
        case SPI_ID_2 :
            return _SPI2CON_ON_MASK;
        case SPI_ID_3 :
            return _SPI3CON_ON_MASK;
        case SPI_ID_4 :
            return _SPI4CON_ON_MASK;
        case SPI_ID_5 :
            return _SPI5CON_ON_MASK;
        case SPI_ID_6 :
            return _SPI6CON_ON_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_STOP_IN_IDLE_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_SIDL_MASK;
        case SPI_ID_2 :
            return _SPI2CON_SIDL_MASK;
        case SPI_ID_3 :
            return _SPI3CON_SIDL_MASK;
        case SPI_ID_4 :
            return _SPI4CON_SIDL_MASK;
        case SPI_ID_5 :
            return _SPI5CON_SIDL_MASK;
        case SPI_ID_6 :
            return _SPI6CON_SIDL_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_PIN_CONTROL_SDO_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_DISSDO_MASK;
        case SPI_ID_2 :
            return _SPI2CON_DISSDO_MASK;
        case SPI_ID_3 :
            return _SPI3CON_DISSDO_MASK;
        case SPI_ID_4 :
            return _SPI4CON_DISSDO_MASK;
        case SPI_ID_5 :
            return _SPI5CON_DISSDO_MASK;
        case SPI_ID_6 :
            return _SPI6CON_DISSDO_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_MODE32_COMMUNICATION_WIDTH_PIC32_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MODE32_MASK;
        case SPI_ID_2 :
            return _SPI2CON_MODE32_MASK;
        case SPI_ID_3 :
            return _SPI3CON_MODE32_MASK;
        case SPI_ID_4 :
            return _SPI4CON_MODE32_MASK;
        case SPI_ID_5 :
            return _SPI5CON_MODE32_MASK;
        case SPI_ID_6 :
            return _SPI6CON_MODE32_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_MODE32_AUDIO_COMMUNICATION_WIDTH_PIC32_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MODE32_MASK;
        case SPI_ID_2 :
            return _SPI2CON_MODE32_MASK;
        case SPI_ID_3 :
            return _SPI3CON_MODE32_MASK;
        case SPI_ID_4 :
            return _SPI4CON_MODE32_MASK;
        case SPI_ID_5 :
            return _SPI5CON_MODE32_MASK;
        case SPI_ID_6 :
            return _SPI6CON_MODE32_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_MODE16_COMMUNICATION_WIDTH_PIC32_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MODE16_MASK;
        case SPI_ID_2 :
            return _SPI2CON_MODE16_MASK;
        case SPI_ID_3 :
            return _SPI3CON_MODE16_MASK;
        case SPI_ID_4 :
            return _SPI4CON_MODE16_MASK;
        case SPI_ID_5 :
            return _SPI5CON_MODE16_MASK;
        case SPI_ID_6 :
            return _SPI6CON_MODE16_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_MODE16_AUDIO_COMMUNICATION_WIDTH_PIC32_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MODE16_MASK;
        case SPI_ID_2 :
            return _SPI2CON_MODE16_MASK;
        case SPI_ID_3 :
            return _SPI3CON_MODE16_MASK;
        case SPI_ID_4 :
            return _SPI4CON_MODE16_MASK;
        case SPI_ID_5 :
            return _SPI5CON_MODE16_MASK;
        case SPI_ID_6 :
            return _SPI6CON_MODE16_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_INPUT_SAMPLE_PHASE_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_SMP_MASK;
        case SPI_ID_2 :
            return _SPI2CON_SMP_MASK;
        case SPI_ID_3 :
            return _SPI3CON_SMP_MASK;
        case SPI_ID_4 :
            return _SPI4CON_SMP_MASK;
        case SPI_ID_5 :
            return _SPI5CON_SMP_MASK;
        case SPI_ID_6 :
            return _SPI6CON_SMP_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_OUTPUT_DATA_PHASE_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_CKE_MASK;
        case SPI_ID_2 :
            return _SPI2CON_CKE_MASK;
        case SPI_ID_3 :
            return _SPI3CON_CKE_MASK;
        case SPI_ID_4 :
            return _SPI4CON_CKE_MASK;
        case SPI_ID_5 :
            return _SPI5CON_CKE_MASK;
        case SPI_ID_6 :
            return _SPI6CON_CKE_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_PIN_CONTROL_SS_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_SSEN_MASK;
        case SPI_ID_2 :
            return _SPI2CON_SSEN_MASK;
        case SPI_ID_3 :
            return _SPI3CON_SSEN_MASK;
        case SPI_ID_4 :
            return _SPI4CON_SSEN_MASK;
        case SPI_ID_5 :
            return _SPI5CON_SSEN_MASK;
        case SPI_ID_6 :
            return _SPI6CON_SSEN_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_CLOCK_POLARITY_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_CKP_MASK;
        case SPI_ID_2 :
            return _SPI2CON_CKP_MASK;
        case SPI_ID_3 :
            return _SPI3CON_CKP_MASK;
        case SPI_ID_4 :
            return _SPI4CON_CKP_MASK;
        case SPI_ID_5 :
            return _SPI5CON_CKP_MASK;
        case SPI_ID_6 :
            return _SPI6CON_CKP_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_MASTER_CONTROL_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MSTEN_MASK;
        case SPI_ID_2 :
            return _SPI2CON_MSTEN_MASK;
        case SPI_ID_3 :
            return _SPI3CON_MSTEN_MASK;
        case SPI_ID_4 :
            return _SPI4CON_MSTEN_MASK;
        case SPI_ID_5 :
            return _SPI5CON_MSTEN_MASK;
        case SPI_ID_6 :
            return _SPI6CON_MSTEN_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_PIN_CONTROL_SDI_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_DISSDI_MASK;
        case SPI_ID_2 :
            return _SPI2CON_DISSDI_MASK;
        case SPI_ID_3 :
            return _SPI3CON_DISSDI_MASK;
        case SPI_ID_4 :
            return _SPI4CON_DISSDI_MASK;
        case SPI_ID_5 :
            return _SPI5CON_DISSDI_MASK;
        case SPI_ID_6 :
            return _SPI6CON_DISSDI_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_STXISEL_MASK;
        case SPI_ID_2 :
            return _SPI2CON_STXISEL_MASK;
        case SPI_ID_3 :
            return _SPI3CON_STXISEL_MASK;
        case SPI_ID_4 :
            return _SPI4CON_STXISEL_MASK;
        case SPI_ID_5 :
            return _SPI5CON_STXISEL_MASK;
        case SPI_ID_6 :
            return _SPI6CON_STXISEL_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_SRXISEL_MASK;
        case SPI_ID_2 :
            return _SPI2CON_SRXISEL_MASK;
        case SPI_ID_3 :
            return _SPI3CON_SRXISEL_MASK;
        case SPI_ID_4 :
            return _SPI4CON_SRXISEL_MASK;
        case SPI_ID_5 :
            return _SPI5CON_SRXISEL_MASK;
        case SPI_ID_6 :
            return _SPI6CON_SRXISEL_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RX_FIFO_COUNT_PIC32_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_RXBUFELM_MASK;
        case SPI_ID_2 :
            return _SPI2STAT_RXBUFELM_MASK;
        case SPI_ID_3 :
            return _SPI3STAT_RXBUFELM_MASK;
        case SPI_ID_4 :
            return _SPI4STAT_RXBUFELM_MASK;
        case SPI_ID_5 :
            return _SPI5STAT_RXBUFELM_MASK;
        case SPI_ID_6 :
            return _SPI6STAT_RXBUFELM_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TX_FIFO_COUNT_PIC32_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_TXBUFELM_MASK;
        case SPI_ID_2 :
            return _SPI2STAT_TXBUFELM_MASK;
        case SPI_ID_3 :
            return _SPI3STAT_TXBUFELM_MASK;
        case SPI_ID_4 :
            return _SPI4STAT_TXBUFELM_MASK;
        case SPI_ID_5 :
            return _SPI5STAT_TXBUFELM_MASK;
        case SPI_ID_6 :
            return _SPI6STAT_TXBUFELM_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_ERROR_STATUS_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_FRMERR_MASK;
        case SPI_ID_2 :
            return _SPI2STAT_FRMERR_MASK;
        case SPI_ID_3 :
            return _SPI3STAT_FRMERR_MASK;
        case SPI_ID_4 :
            return _SPI4STAT_FRMERR_MASK;
        case SPI_ID_5 :
            return _SPI5STAT_FRMERR_MASK;
        case SPI_ID_6 :
            return _SPI6STAT_FRMERR_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_BUS_STATUS_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPIBUSY_MASK;
        case SPI_ID_2 :
            return _SPI2STAT_SPIBUSY_MASK;
        case SPI_ID_3 :
            return _SPI3STAT_SPIBUSY_MASK;
        case SPI_ID_4 :
            return _SPI4STAT_SPIBUSY_MASK;
        case SPI_ID_5 :
            return _SPI5STAT_SPIBUSY_MASK;
        case SPI_ID_6 :
            return _SPI6STAT_SPIBUSY_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TRANSMIT_UNDER_RUN_STATUS_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPITUR_MASK;
        case SPI_ID_2 :
            return _SPI2STAT_SPITUR_MASK;
        case SPI_ID_3 :
            return _SPI3STAT_SPITUR_MASK;
        case SPI_ID_4 :
            return _SPI4STAT_SPITUR_MASK;
        case SPI_ID_5 :
            return _SPI5STAT_SPITUR_MASK;
        case SPI_ID_6 :
            return _SPI6STAT_SPITUR_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FIFO_REGISTER_EMPTY_STATUS_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SRMT_MASK;
        case SPI_ID_2 :
            return _SPI2STAT_SRMT_MASK;
        case SPI_ID_3 :
            return _SPI3STAT_SRMT_MASK;
        case SPI_ID_4 :
            return _SPI4STAT_SRMT_MASK;
        case SPI_ID_5 :
            return _SPI5STAT_SRMT_MASK;
        case SPI_ID_6 :
            return _SPI6STAT_SRMT_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RECEIVER_OVERFLOW_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPIROV_MASK;
        case SPI_ID_2 :
            return _SPI2STAT_SPIROV_MASK;
        case SPI_ID_3 :
            return _SPI3STAT_SPIROV_MASK;
        case SPI_ID_4 :
            return _SPI4STAT_SPIROV_MASK;
        case SPI_ID_5 :
            return _SPI5STAT_SPIROV_MASK;
        case SPI_ID_6 :
            return _SPI6STAT_SPIROV_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RECEIVE_FIFO_STATUS_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPIRBE_MASK;
        case SPI_ID_2 :
            return _SPI2STAT_SPIRBE_MASK;
        case SPI_ID_3 :
            return _SPI3STAT_SPIRBE_MASK;
        case SPI_ID_4 :
            return _SPI4STAT_SPIRBE_MASK;
        case SPI_ID_5 :
            return _SPI5STAT_SPIRBE_MASK;
        case SPI_ID_6 :
            return _SPI6STAT_SPIRBE_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TRANSMIT_BUFFER_EMPTY_STATUS_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPITBE_MASK;
        case SPI_ID_2 :
            return _SPI2STAT_SPITBE_MASK;
        case SPI_ID_3 :
            return _SPI3STAT_SPITBE_MASK;
        case SPI_ID_4 :
            return _SPI4STAT_SPITBE_MASK;
        case SPI_ID_5 :
            return _SPI5STAT_SPITBE_MASK;
        case SPI_ID_6 :
            return _SPI6STAT_SPITBE_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TRANSMIT_BUFFER_FULL_STATUS_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPITBF_MASK;
        case SPI_ID_2 :
            return _SPI2STAT_SPITBF_MASK;
        case SPI_ID_3 :
            return _SPI3STAT_SPITBF_MASK;
        case SPI_ID_4 :
            return _SPI4STAT_SPITBF_MASK;
        case SPI_ID_5 :
            return _SPI5STAT_SPITBF_MASK;
        case SPI_ID_6 :
            return _SPI6STAT_SPITBF_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RECEIVE_BUFFER_STATUS_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPIRBF_MASK;
        case SPI_ID_2 :
            return _SPI2STAT_SPIRBF_MASK;
        case SPI_ID_3 :
            return _SPI3STAT_SPIRBF_MASK;
        case SPI_ID_4 :
            return _SPI4STAT_SPIRBF_MASK;
        case SPI_ID_5 :
            return _SPI5STAT_SPIRBF_MASK;
        case SPI_ID_6 :
            return _SPI6STAT_SPIRBF_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_READ_DATA_SIGN_STATUS_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_SPISGNEXT_MASK;
        case SPI_ID_2 :
            return _SPI2CON2_SPISGNEXT_MASK;
        case SPI_ID_3 :
            return _SPI3CON2_SPISGNEXT_MASK;
        case SPI_ID_4 :
            return _SPI4CON2_SPISGNEXT_MASK;
        case SPI_ID_5 :
            return _SPI5CON2_SPISGNEXT_MASK;
        case SPI_ID_6 :
            return _SPI6CON2_SPISGNEXT_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_ERROR_INTERRUPT_CONTROL_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_FRMERREN_MASK;
        case SPI_ID_2 :
            return _SPI2CON2_FRMERREN_MASK;
        case SPI_ID_3 :
            return _SPI3CON2_FRMERREN_MASK;
        case SPI_ID_4 :
            return _SPI4CON2_FRMERREN_MASK;
        case SPI_ID_5 :
            return _SPI5CON2_FRMERREN_MASK;
        case SPI_ID_6 :
            return _SPI6CON2_FRMERREN_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RX_OVERFLOW_ERROR_INTERRUPT_CONTROL_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_SPIROVEN_MASK;
        case SPI_ID_2 :
            return _SPI2CON2_SPIROVEN_MASK;
        case SPI_ID_3 :
            return _SPI3CON2_SPIROVEN_MASK;
        case SPI_ID_4 :
            return _SPI4CON2_SPIROVEN_MASK;
        case SPI_ID_5 :
            return _SPI5CON2_SPIROVEN_MASK;
        case SPI_ID_6 :
            return _SPI6CON2_SPIROVEN_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TX_UNDERRUN_ERROR_INTERRUPT_CONTROL_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_SPITUREN_MASK;
        case SPI_ID_2 :
            return _SPI2CON2_SPITUREN_MASK;
        case SPI_ID_3 :
            return _SPI3CON2_SPITUREN_MASK;
        case SPI_ID_4 :
            return _SPI4CON2_SPITUREN_MASK;
        case SPI_ID_5 :
            return _SPI5CON2_SPITUREN_MASK;
        case SPI_ID_6 :
            return _SPI6CON2_SPITUREN_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RX_OVERFLOW_AUDIO_ERROR_CONTROL_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_IGNROV_MASK;
        case SPI_ID_2 :
            return _SPI2CON2_IGNROV_MASK;
        case SPI_ID_3 :
            return _SPI3CON2_IGNROV_MASK;
        case SPI_ID_4 :
            return _SPI4CON2_IGNROV_MASK;
        case SPI_ID_5 :
            return _SPI5CON2_IGNROV_MASK;
        case SPI_ID_6 :
            return _SPI6CON2_IGNROV_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TX_UNDERRUN_AUDIO_ERROR_CONTROL_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_IGNTUR_MASK;
        case SPI_ID_2 :
            return _SPI2CON2_IGNTUR_MASK;
        case SPI_ID_3 :
            return _SPI3CON2_IGNTUR_MASK;
        case SPI_ID_4 :
            return _SPI4CON2_IGNTUR_MASK;
        case SPI_ID_5 :
            return _SPI5CON2_IGNTUR_MASK;
        case SPI_ID_6 :
            return _SPI6CON2_IGNTUR_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_AUDIO_PROTOCOL_CONTROL_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_AUDEN_MASK;
        case SPI_ID_2 :
            return _SPI2CON2_AUDEN_MASK;
        case SPI_ID_3 :
            return _SPI3CON2_AUDEN_MASK;
        case SPI_ID_4 :
            return _SPI4CON2_AUDEN_MASK;
        case SPI_ID_5 :
            return _SPI5CON2_AUDEN_MASK;
        case SPI_ID_6 :
            return _SPI6CON2_AUDEN_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_AUDIO_TRANSMIT_MODE_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_AUDMONO_MASK;
        case SPI_ID_2 :
            return _SPI2CON2_AUDMONO_MASK;
        case SPI_ID_3 :
            return _SPI3CON2_AUDMONO_MASK;
        case SPI_ID_4 :
            return _SPI4CON2_AUDMONO_MASK;
        case SPI_ID_5 :
            return _SPI5CON2_AUDMONO_MASK;
        case SPI_ID_6 :
            return _SPI6CON2_AUDMONO_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_AUDIO_PROTOCOL_MODE_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_AUDMOD_MASK;
        case SPI_ID_2 :
            return _SPI2CON2_AUDMOD_MASK;
        case SPI_ID_3 :
            return _SPI3CON2_AUDMOD_MASK;
        case SPI_ID_4 :
            return _SPI4CON2_AUDMOD_MASK;
        case SPI_ID_5 :
            return _SPI5CON2_AUDMOD_MASK;
        case SPI_ID_6 :
            return _SPI6CON2_AUDMOD_MASK;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_BUFFER_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return (SFR_DATA)-1;
        case SPI_ID_2 :
            return (SFR_DATA)-1;
        case SPI_ID_3 :
            return (SFR_DATA)-1;
        case SPI_ID_4 :
            return (SFR_DATA)-1;
        case SPI_ID_5 :
            return (SFR_DATA)-1;
        case SPI_ID_6 :
            return (SFR_DATA)-1;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_BAUD_RATE_MASK(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return (SFR_DATA)-1;
        case SPI_ID_2 :
            return (SFR_DATA)-1;
        case SPI_ID_3 :
            return (SFR_DATA)-1;
        case SPI_ID_4 :
            return (SFR_DATA)-1;
        case SPI_ID_5 :
            return (SFR_DATA)-1;
        case SPI_ID_6 :
            return (SFR_DATA)-1;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAMED_COMMUNICATION_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_FRMEN_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_FRMEN_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_FRMEN_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_FRMEN_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_FRMEN_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_FRMEN_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_SYNC_PULSE_DIRECTION_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_FRMSYNC_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_FRMSYNC_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_FRMSYNC_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_FRMSYNC_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_FRMSYNC_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_FRMSYNC_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_SYNC_PULSE_POLARITY_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_FRMPOL_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_FRMPOL_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_FRMPOL_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_FRMPOL_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_FRMPOL_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_FRMPOL_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_SLAVE_SELECT_CONTROL_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MSSEN_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_MSSEN_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_MSSEN_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_MSSEN_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_MSSEN_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_MSSEN_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_SYNC_PULSE_WIDTH_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_FRMSYPW_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_FRMSYPW_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_FRMSYPW_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_FRMSYPW_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_FRMSYPW_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_FRMSYPW_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_SYNC_PULSE_COUNTER_PIC32_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_FRMCNT_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_FRMCNT_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_FRMCNT_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_FRMCNT_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_FRMCNT_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_FRMCNT_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_BAUD_RATE_CLOCK_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MCLKSEL_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_MCLKSEL_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_MCLKSEL_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_MCLKSEL_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_MCLKSEL_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_MCLKSEL_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_SYNC_PULSE_EDGE_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_SPIFE_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_SPIFE_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_SPIFE_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_SPIFE_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_SPIFE_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_SPIFE_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FIFO_CONTROL_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_ENHBUF_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_ENHBUF_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_ENHBUF_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_ENHBUF_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_ENHBUF_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_ENHBUF_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_ENABLE_CONTROL_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_ON_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_ON_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_ON_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_ON_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_ON_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_ON_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_STOP_IN_IDLE_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_SIDL_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_SIDL_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_SIDL_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_SIDL_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_SIDL_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_SIDL_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_PIN_CONTROL_SDO_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_DISSDO_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_DISSDO_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_DISSDO_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_DISSDO_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_DISSDO_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_DISSDO_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_MODE32_COMMUNICATION_WIDTH_PIC32_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MODE32_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_MODE32_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_MODE32_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_MODE32_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_MODE32_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_MODE32_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_MODE32_AUDIO_COMMUNICATION_WIDTH_PIC32_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MODE32_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_MODE32_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_MODE32_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_MODE32_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_MODE32_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_MODE32_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_MODE16_COMMUNICATION_WIDTH_PIC32_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MODE16_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_MODE16_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_MODE16_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_MODE16_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_MODE16_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_MODE16_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_MODE16_AUDIO_COMMUNICATION_WIDTH_PIC32_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MODE16_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_MODE16_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_MODE16_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_MODE16_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_MODE16_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_MODE16_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_INPUT_SAMPLE_PHASE_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_SMP_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_SMP_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_SMP_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_SMP_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_SMP_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_SMP_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_OUTPUT_DATA_PHASE_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_CKE_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_CKE_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_CKE_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_CKE_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_CKE_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_CKE_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_PIN_CONTROL_SS_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_SSEN_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_SSEN_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_SSEN_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_SSEN_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_SSEN_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_SSEN_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_CLOCK_POLARITY_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_CKP_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_CKP_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_CKP_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_CKP_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_CKP_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_CKP_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_MASTER_CONTROL_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MSTEN_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_MSTEN_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_MSTEN_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_MSTEN_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_MSTEN_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_MSTEN_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_PIN_CONTROL_SDI_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_DISSDI_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_DISSDI_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_DISSDI_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_DISSDI_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_DISSDI_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_DISSDI_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_STXISEL_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_STXISEL_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_STXISEL_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_STXISEL_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_STXISEL_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_STXISEL_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_SRXISEL_POSITION;
        case SPI_ID_2 :
            return _SPI2CON_SRXISEL_POSITION;
        case SPI_ID_3 :
            return _SPI3CON_SRXISEL_POSITION;
        case SPI_ID_4 :
            return _SPI4CON_SRXISEL_POSITION;
        case SPI_ID_5 :
            return _SPI5CON_SRXISEL_POSITION;
        case SPI_ID_6 :
            return _SPI6CON_SRXISEL_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RX_FIFO_COUNT_PIC32_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_RXBUFELM_POSITION;
        case SPI_ID_2 :
            return _SPI2STAT_RXBUFELM_POSITION;
        case SPI_ID_3 :
            return _SPI3STAT_RXBUFELM_POSITION;
        case SPI_ID_4 :
            return _SPI4STAT_RXBUFELM_POSITION;
        case SPI_ID_5 :
            return _SPI5STAT_RXBUFELM_POSITION;
        case SPI_ID_6 :
            return _SPI6STAT_RXBUFELM_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TX_FIFO_COUNT_PIC32_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_TXBUFELM_POSITION;
        case SPI_ID_2 :
            return _SPI2STAT_TXBUFELM_POSITION;
        case SPI_ID_3 :
            return _SPI3STAT_TXBUFELM_POSITION;
        case SPI_ID_4 :
            return _SPI4STAT_TXBUFELM_POSITION;
        case SPI_ID_5 :
            return _SPI5STAT_TXBUFELM_POSITION;
        case SPI_ID_6 :
            return _SPI6STAT_TXBUFELM_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_ERROR_STATUS_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_FRMERR_POSITION;
        case SPI_ID_2 :
            return _SPI2STAT_FRMERR_POSITION;
        case SPI_ID_3 :
            return _SPI3STAT_FRMERR_POSITION;
        case SPI_ID_4 :
            return _SPI4STAT_FRMERR_POSITION;
        case SPI_ID_5 :
            return _SPI5STAT_FRMERR_POSITION;
        case SPI_ID_6 :
            return _SPI6STAT_FRMERR_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_BUS_STATUS_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPIBUSY_POSITION;
        case SPI_ID_2 :
            return _SPI2STAT_SPIBUSY_POSITION;
        case SPI_ID_3 :
            return _SPI3STAT_SPIBUSY_POSITION;
        case SPI_ID_4 :
            return _SPI4STAT_SPIBUSY_POSITION;
        case SPI_ID_5 :
            return _SPI5STAT_SPIBUSY_POSITION;
        case SPI_ID_6 :
            return _SPI6STAT_SPIBUSY_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TRANSMIT_UNDER_RUN_STATUS_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPITUR_POSITION;
        case SPI_ID_2 :
            return _SPI2STAT_SPITUR_POSITION;
        case SPI_ID_3 :
            return _SPI3STAT_SPITUR_POSITION;
        case SPI_ID_4 :
            return _SPI4STAT_SPITUR_POSITION;
        case SPI_ID_5 :
            return _SPI5STAT_SPITUR_POSITION;
        case SPI_ID_6 :
            return _SPI6STAT_SPITUR_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FIFO_REGISTER_EMPTY_STATUS_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SRMT_POSITION;
        case SPI_ID_2 :
            return _SPI2STAT_SRMT_POSITION;
        case SPI_ID_3 :
            return _SPI3STAT_SRMT_POSITION;
        case SPI_ID_4 :
            return _SPI4STAT_SRMT_POSITION;
        case SPI_ID_5 :
            return _SPI5STAT_SRMT_POSITION;
        case SPI_ID_6 :
            return _SPI6STAT_SRMT_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RECEIVER_OVERFLOW_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPIROV_POSITION;
        case SPI_ID_2 :
            return _SPI2STAT_SPIROV_POSITION;
        case SPI_ID_3 :
            return _SPI3STAT_SPIROV_POSITION;
        case SPI_ID_4 :
            return _SPI4STAT_SPIROV_POSITION;
        case SPI_ID_5 :
            return _SPI5STAT_SPIROV_POSITION;
        case SPI_ID_6 :
            return _SPI6STAT_SPIROV_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RECEIVE_FIFO_STATUS_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPIRBE_POSITION;
        case SPI_ID_2 :
            return _SPI2STAT_SPIRBE_POSITION;
        case SPI_ID_3 :
            return _SPI3STAT_SPIRBE_POSITION;
        case SPI_ID_4 :
            return _SPI4STAT_SPIRBE_POSITION;
        case SPI_ID_5 :
            return _SPI5STAT_SPIRBE_POSITION;
        case SPI_ID_6 :
            return _SPI6STAT_SPIRBE_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TRANSMIT_BUFFER_EMPTY_STATUS_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPITBE_POSITION;
        case SPI_ID_2 :
            return _SPI2STAT_SPITBE_POSITION;
        case SPI_ID_3 :
            return _SPI3STAT_SPITBE_POSITION;
        case SPI_ID_4 :
            return _SPI4STAT_SPITBE_POSITION;
        case SPI_ID_5 :
            return _SPI5STAT_SPITBE_POSITION;
        case SPI_ID_6 :
            return _SPI6STAT_SPITBE_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TRANSMIT_BUFFER_FULL_STATUS_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPITBF_POSITION;
        case SPI_ID_2 :
            return _SPI2STAT_SPITBF_POSITION;
        case SPI_ID_3 :
            return _SPI3STAT_SPITBF_POSITION;
        case SPI_ID_4 :
            return _SPI4STAT_SPITBF_POSITION;
        case SPI_ID_5 :
            return _SPI5STAT_SPITBF_POSITION;
        case SPI_ID_6 :
            return _SPI6STAT_SPITBF_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RECEIVE_BUFFER_STATUS_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPIRBF_POSITION;
        case SPI_ID_2 :
            return _SPI2STAT_SPIRBF_POSITION;
        case SPI_ID_3 :
            return _SPI3STAT_SPIRBF_POSITION;
        case SPI_ID_4 :
            return _SPI4STAT_SPIRBF_POSITION;
        case SPI_ID_5 :
            return _SPI5STAT_SPIRBF_POSITION;
        case SPI_ID_6 :
            return _SPI6STAT_SPIRBF_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_READ_DATA_SIGN_STATUS_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_SPISGNEXT_POSITION;
        case SPI_ID_2 :
            return _SPI2CON2_SPISGNEXT_POSITION;
        case SPI_ID_3 :
            return _SPI3CON2_SPISGNEXT_POSITION;
        case SPI_ID_4 :
            return _SPI4CON2_SPISGNEXT_POSITION;
        case SPI_ID_5 :
            return _SPI5CON2_SPISGNEXT_POSITION;
        case SPI_ID_6 :
            return _SPI6CON2_SPISGNEXT_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_ERROR_INTERRUPT_CONTROL_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_FRMERREN_POSITION;
        case SPI_ID_2 :
            return _SPI2CON2_FRMERREN_POSITION;
        case SPI_ID_3 :
            return _SPI3CON2_FRMERREN_POSITION;
        case SPI_ID_4 :
            return _SPI4CON2_FRMERREN_POSITION;
        case SPI_ID_5 :
            return _SPI5CON2_FRMERREN_POSITION;
        case SPI_ID_6 :
            return _SPI6CON2_FRMERREN_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RX_OVERFLOW_ERROR_INTERRUPT_CONTROL_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_SPIROVEN_POSITION;
        case SPI_ID_2 :
            return _SPI2CON2_SPIROVEN_POSITION;
        case SPI_ID_3 :
            return _SPI3CON2_SPIROVEN_POSITION;
        case SPI_ID_4 :
            return _SPI4CON2_SPIROVEN_POSITION;
        case SPI_ID_5 :
            return _SPI5CON2_SPIROVEN_POSITION;
        case SPI_ID_6 :
            return _SPI6CON2_SPIROVEN_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TX_UNDERRUN_ERROR_INTERRUPT_CONTROL_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_SPITUREN_POSITION;
        case SPI_ID_2 :
            return _SPI2CON2_SPITUREN_POSITION;
        case SPI_ID_3 :
            return _SPI3CON2_SPITUREN_POSITION;
        case SPI_ID_4 :
            return _SPI4CON2_SPITUREN_POSITION;
        case SPI_ID_5 :
            return _SPI5CON2_SPITUREN_POSITION;
        case SPI_ID_6 :
            return _SPI6CON2_SPITUREN_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RX_OVERFLOW_AUDIO_ERROR_CONTROL_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_IGNROV_POSITION;
        case SPI_ID_2 :
            return _SPI2CON2_IGNROV_POSITION;
        case SPI_ID_3 :
            return _SPI3CON2_IGNROV_POSITION;
        case SPI_ID_4 :
            return _SPI4CON2_IGNROV_POSITION;
        case SPI_ID_5 :
            return _SPI5CON2_IGNROV_POSITION;
        case SPI_ID_6 :
            return _SPI6CON2_IGNROV_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TX_UNDERRUN_AUDIO_ERROR_CONTROL_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_IGNTUR_POSITION;
        case SPI_ID_2 :
            return _SPI2CON2_IGNTUR_POSITION;
        case SPI_ID_3 :
            return _SPI3CON2_IGNTUR_POSITION;
        case SPI_ID_4 :
            return _SPI4CON2_IGNTUR_POSITION;
        case SPI_ID_5 :
            return _SPI5CON2_IGNTUR_POSITION;
        case SPI_ID_6 :
            return _SPI6CON2_IGNTUR_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_AUDIO_PROTOCOL_CONTROL_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_AUDEN_POSITION;
        case SPI_ID_2 :
            return _SPI2CON2_AUDEN_POSITION;
        case SPI_ID_3 :
            return _SPI3CON2_AUDEN_POSITION;
        case SPI_ID_4 :
            return _SPI4CON2_AUDEN_POSITION;
        case SPI_ID_5 :
            return _SPI5CON2_AUDEN_POSITION;
        case SPI_ID_6 :
            return _SPI6CON2_AUDEN_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_AUDIO_TRANSMIT_MODE_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_AUDMONO_POSITION;
        case SPI_ID_2 :
            return _SPI2CON2_AUDMONO_POSITION;
        case SPI_ID_3 :
            return _SPI3CON2_AUDMONO_POSITION;
        case SPI_ID_4 :
            return _SPI4CON2_AUDMONO_POSITION;
        case SPI_ID_5 :
            return _SPI5CON2_AUDMONO_POSITION;
        case SPI_ID_6 :
            return _SPI6CON2_AUDMONO_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_AUDIO_PROTOCOL_MODE_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_AUDMOD_POSITION;
        case SPI_ID_2 :
            return _SPI2CON2_AUDMOD_POSITION;
        case SPI_ID_3 :
            return _SPI3CON2_AUDMOD_POSITION;
        case SPI_ID_4 :
            return _SPI4CON2_AUDMOD_POSITION;
        case SPI_ID_5 :
            return _SPI5CON2_AUDMOD_POSITION;
        case SPI_ID_6 :
            return _SPI6CON2_AUDMOD_POSITION;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_BUFFER_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return (SFR_DATA)0;
        case SPI_ID_2 :
            return (SFR_DATA)0;
        case SPI_ID_3 :
            return (SFR_DATA)0;
        case SPI_ID_4 :
            return (SFR_DATA)0;
        case SPI_ID_5 :
            return (SFR_DATA)0;
        case SPI_ID_6 :
            return (SFR_DATA)0;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_BAUD_RATE_POS(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return (SFR_DATA)0;
        case SPI_ID_2 :
            return (SFR_DATA)0;
        case SPI_ID_3 :
            return (SFR_DATA)0;
        case SPI_ID_4 :
            return (SFR_DATA)0;
        case SPI_ID_5 :
            return (SFR_DATA)0;
        case SPI_ID_6 :
            return (SFR_DATA)0;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAMED_COMMUNICATION_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_FRMEN_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_FRMEN_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_FRMEN_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_FRMEN_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_FRMEN_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_FRMEN_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_SYNC_PULSE_DIRECTION_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_FRMSYNC_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_FRMSYNC_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_FRMSYNC_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_FRMSYNC_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_FRMSYNC_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_FRMSYNC_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_SYNC_PULSE_POLARITY_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_FRMPOL_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_FRMPOL_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_FRMPOL_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_FRMPOL_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_FRMPOL_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_FRMPOL_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_SLAVE_SELECT_CONTROL_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MSSEN_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_MSSEN_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_MSSEN_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_MSSEN_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_MSSEN_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_MSSEN_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_SYNC_PULSE_WIDTH_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_FRMSYPW_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_FRMSYPW_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_FRMSYPW_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_FRMSYPW_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_FRMSYPW_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_FRMSYPW_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_SYNC_PULSE_COUNTER_PIC32_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_FRMCNT_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_FRMCNT_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_FRMCNT_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_FRMCNT_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_FRMCNT_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_FRMCNT_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_BAUD_RATE_CLOCK_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MCLKSEL_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_MCLKSEL_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_MCLKSEL_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_MCLKSEL_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_MCLKSEL_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_MCLKSEL_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_SYNC_PULSE_EDGE_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_SPIFE_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_SPIFE_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_SPIFE_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_SPIFE_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_SPIFE_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_SPIFE_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FIFO_CONTROL_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_ENHBUF_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_ENHBUF_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_ENHBUF_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_ENHBUF_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_ENHBUF_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_ENHBUF_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_ENABLE_CONTROL_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_ON_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_ON_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_ON_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_ON_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_ON_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_ON_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_STOP_IN_IDLE_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_SIDL_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_SIDL_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_SIDL_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_SIDL_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_SIDL_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_SIDL_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_PIN_CONTROL_SDO_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_DISSDO_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_DISSDO_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_DISSDO_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_DISSDO_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_DISSDO_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_DISSDO_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_MODE32_COMMUNICATION_WIDTH_PIC32_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MODE32_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_MODE32_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_MODE32_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_MODE32_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_MODE32_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_MODE32_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_MODE32_AUDIO_COMMUNICATION_WIDTH_PIC32_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MODE32_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_MODE32_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_MODE32_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_MODE32_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_MODE32_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_MODE32_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_MODE16_COMMUNICATION_WIDTH_PIC32_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MODE16_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_MODE16_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_MODE16_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_MODE16_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_MODE16_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_MODE16_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_MODE16_AUDIO_COMMUNICATION_WIDTH_PIC32_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MODE16_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_MODE16_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_MODE16_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_MODE16_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_MODE16_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_MODE16_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_INPUT_SAMPLE_PHASE_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_SMP_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_SMP_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_SMP_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_SMP_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_SMP_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_SMP_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_OUTPUT_DATA_PHASE_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_CKE_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_CKE_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_CKE_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_CKE_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_CKE_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_CKE_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_PIN_CONTROL_SS_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_SSEN_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_SSEN_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_SSEN_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_SSEN_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_SSEN_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_SSEN_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_CLOCK_POLARITY_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_CKP_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_CKP_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_CKP_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_CKP_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_CKP_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_CKP_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_MASTER_CONTROL_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_MSTEN_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_MSTEN_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_MSTEN_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_MSTEN_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_MSTEN_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_MSTEN_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_PIN_CONTROL_SDI_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_DISSDI_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_DISSDI_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_DISSDI_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_DISSDI_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_DISSDI_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_DISSDI_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_STXISEL_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_STXISEL_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_STXISEL_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_STXISEL_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_STXISEL_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_STXISEL_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON_SRXISEL_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON_SRXISEL_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON_SRXISEL_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON_SRXISEL_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON_SRXISEL_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON_SRXISEL_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RX_FIFO_COUNT_PIC32_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_RXBUFELM_LENGTH;
        case SPI_ID_2 :
            return _SPI2STAT_RXBUFELM_LENGTH;
        case SPI_ID_3 :
            return _SPI3STAT_RXBUFELM_LENGTH;
        case SPI_ID_4 :
            return _SPI4STAT_RXBUFELM_LENGTH;
        case SPI_ID_5 :
            return _SPI5STAT_RXBUFELM_LENGTH;
        case SPI_ID_6 :
            return _SPI6STAT_RXBUFELM_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TX_FIFO_COUNT_PIC32_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_TXBUFELM_LENGTH;
        case SPI_ID_2 :
            return _SPI2STAT_TXBUFELM_LENGTH;
        case SPI_ID_3 :
            return _SPI3STAT_TXBUFELM_LENGTH;
        case SPI_ID_4 :
            return _SPI4STAT_TXBUFELM_LENGTH;
        case SPI_ID_5 :
            return _SPI5STAT_TXBUFELM_LENGTH;
        case SPI_ID_6 :
            return _SPI6STAT_TXBUFELM_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_ERROR_STATUS_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_FRMERR_LENGTH;
        case SPI_ID_2 :
            return _SPI2STAT_FRMERR_LENGTH;
        case SPI_ID_3 :
            return _SPI3STAT_FRMERR_LENGTH;
        case SPI_ID_4 :
            return _SPI4STAT_FRMERR_LENGTH;
        case SPI_ID_5 :
            return _SPI5STAT_FRMERR_LENGTH;
        case SPI_ID_6 :
            return _SPI6STAT_FRMERR_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_BUS_STATUS_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPIBUSY_LENGTH;
        case SPI_ID_2 :
            return _SPI2STAT_SPIBUSY_LENGTH;
        case SPI_ID_3 :
            return _SPI3STAT_SPIBUSY_LENGTH;
        case SPI_ID_4 :
            return _SPI4STAT_SPIBUSY_LENGTH;
        case SPI_ID_5 :
            return _SPI5STAT_SPIBUSY_LENGTH;
        case SPI_ID_6 :
            return _SPI6STAT_SPIBUSY_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TRANSMIT_UNDER_RUN_STATUS_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPITUR_LENGTH;
        case SPI_ID_2 :
            return _SPI2STAT_SPITUR_LENGTH;
        case SPI_ID_3 :
            return _SPI3STAT_SPITUR_LENGTH;
        case SPI_ID_4 :
            return _SPI4STAT_SPITUR_LENGTH;
        case SPI_ID_5 :
            return _SPI5STAT_SPITUR_LENGTH;
        case SPI_ID_6 :
            return _SPI6STAT_SPITUR_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FIFO_REGISTER_EMPTY_STATUS_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SRMT_LENGTH;
        case SPI_ID_2 :
            return _SPI2STAT_SRMT_LENGTH;
        case SPI_ID_3 :
            return _SPI3STAT_SRMT_LENGTH;
        case SPI_ID_4 :
            return _SPI4STAT_SRMT_LENGTH;
        case SPI_ID_5 :
            return _SPI5STAT_SRMT_LENGTH;
        case SPI_ID_6 :
            return _SPI6STAT_SRMT_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RECEIVER_OVERFLOW_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPIROV_LENGTH;
        case SPI_ID_2 :
            return _SPI2STAT_SPIROV_LENGTH;
        case SPI_ID_3 :
            return _SPI3STAT_SPIROV_LENGTH;
        case SPI_ID_4 :
            return _SPI4STAT_SPIROV_LENGTH;
        case SPI_ID_5 :
            return _SPI5STAT_SPIROV_LENGTH;
        case SPI_ID_6 :
            return _SPI6STAT_SPIROV_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RECEIVE_FIFO_STATUS_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPIRBE_LENGTH;
        case SPI_ID_2 :
            return _SPI2STAT_SPIRBE_LENGTH;
        case SPI_ID_3 :
            return _SPI3STAT_SPIRBE_LENGTH;
        case SPI_ID_4 :
            return _SPI4STAT_SPIRBE_LENGTH;
        case SPI_ID_5 :
            return _SPI5STAT_SPIRBE_LENGTH;
        case SPI_ID_6 :
            return _SPI6STAT_SPIRBE_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TRANSMIT_BUFFER_EMPTY_STATUS_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPITBE_LENGTH;
        case SPI_ID_2 :
            return _SPI2STAT_SPITBE_LENGTH;
        case SPI_ID_3 :
            return _SPI3STAT_SPITBE_LENGTH;
        case SPI_ID_4 :
            return _SPI4STAT_SPITBE_LENGTH;
        case SPI_ID_5 :
            return _SPI5STAT_SPITBE_LENGTH;
        case SPI_ID_6 :
            return _SPI6STAT_SPITBE_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TRANSMIT_BUFFER_FULL_STATUS_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPITBF_LENGTH;
        case SPI_ID_2 :
            return _SPI2STAT_SPITBF_LENGTH;
        case SPI_ID_3 :
            return _SPI3STAT_SPITBF_LENGTH;
        case SPI_ID_4 :
            return _SPI4STAT_SPITBF_LENGTH;
        case SPI_ID_5 :
            return _SPI5STAT_SPITBF_LENGTH;
        case SPI_ID_6 :
            return _SPI6STAT_SPITBF_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RECEIVE_BUFFER_STATUS_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1STAT_SPIRBF_LENGTH;
        case SPI_ID_2 :
            return _SPI2STAT_SPIRBF_LENGTH;
        case SPI_ID_3 :
            return _SPI3STAT_SPIRBF_LENGTH;
        case SPI_ID_4 :
            return _SPI4STAT_SPIRBF_LENGTH;
        case SPI_ID_5 :
            return _SPI5STAT_SPIRBF_LENGTH;
        case SPI_ID_6 :
            return _SPI6STAT_SPIRBF_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_READ_DATA_SIGN_STATUS_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_SPISGNEXT_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON2_SPISGNEXT_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON2_SPISGNEXT_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON2_SPISGNEXT_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON2_SPISGNEXT_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON2_SPISGNEXT_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_FRAME_ERROR_INTERRUPT_CONTROL_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_FRMERREN_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON2_FRMERREN_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON2_FRMERREN_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON2_FRMERREN_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON2_FRMERREN_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON2_FRMERREN_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RX_OVERFLOW_ERROR_INTERRUPT_CONTROL_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_SPIROVEN_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON2_SPIROVEN_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON2_SPIROVEN_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON2_SPIROVEN_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON2_SPIROVEN_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON2_SPIROVEN_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TX_UNDERRUN_ERROR_INTERRUPT_CONTROL_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_SPITUREN_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON2_SPITUREN_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON2_SPITUREN_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON2_SPITUREN_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON2_SPITUREN_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON2_SPITUREN_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_RX_OVERFLOW_AUDIO_ERROR_CONTROL_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_IGNROV_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON2_IGNROV_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON2_IGNROV_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON2_IGNROV_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON2_IGNROV_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON2_IGNROV_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_TX_UNDERRUN_AUDIO_ERROR_CONTROL_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_IGNTUR_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON2_IGNTUR_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON2_IGNTUR_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON2_IGNTUR_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON2_IGNTUR_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON2_IGNTUR_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_AUDIO_PROTOCOL_CONTROL_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_AUDEN_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON2_AUDEN_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON2_AUDEN_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON2_AUDEN_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON2_AUDEN_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON2_AUDEN_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_AUDIO_TRANSMIT_MODE_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_AUDMONO_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON2_AUDMONO_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON2_AUDMONO_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON2_AUDMONO_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON2_AUDMONO_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON2_AUDMONO_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_AUDIO_PROTOCOL_MODE_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return _SPI1CON2_AUDMOD_LENGTH;
        case SPI_ID_2 :
            return _SPI2CON2_AUDMOD_LENGTH;
        case SPI_ID_3 :
            return _SPI3CON2_AUDMOD_LENGTH;
        case SPI_ID_4 :
            return _SPI4CON2_AUDMOD_LENGTH;
        case SPI_ID_5 :
            return _SPI5CON2_AUDMOD_LENGTH;
        case SPI_ID_6 :
            return _SPI6CON2_AUDMOD_LENGTH;
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_BUFFER_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case SPI_ID_2 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case SPI_ID_3 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case SPI_ID_4 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case SPI_ID_5 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case SPI_ID_6 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _SPI_BAUD_RATE_LEN(SPI_MODULE_ID i)
{
    switch (i) {
        case SPI_ID_1 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case SPI_ID_2 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case SPI_ID_3 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case SPI_ID_4 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case SPI_ID_5 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case SPI_ID_6 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/spi_EnableControl_Default.h"
#include "../templates/spi_StopInIdle_Default.h"
#include "../templates/spi_ReceiverOverflow_Default.h"
#include "../templates/spi_TransmitBufferFullStatus_Default.h"
#include "../templates/spi_TransmitBufferEmptyStatus_Default.h"
#include "../templates/spi_ReceiveBufferStatus_Default.h"
#include "../templates/spi_PinControl_PIC32.h"
#include "../templates/spi_CommunicationWidth_PIC32.h"
#include "../templates/spi_AudioCommunicationWidth_PIC32.h"
#include "../templates/spi_InputSamplePhase_Default.h"
#include "../templates/spi_OutputDataPhase_Default.h"
#include "../templates/spi_ClockPolarity_Default.h"
#include "../templates/spi_MasterControl_Default.h"
#include "../templates/spi_BaudRate_Default.h"
#include "../templates/spi_BusStatus_Default.h"
#include "../templates/spi_ReadDataSignStatus_Default.h"
#include "../templates/spi_SlaveSelectControl_Default.h"
#include "../templates/spi_TransmitUnderRunStatus_Default.h"
#include "../templates/spi_FIFOControl_Default.h"
#include "../templates/spi_FIFOCount_PIC32.h"
#include "../templates/spi_ReceiveFIFOStatus_Default.h"
#include "../templates/spi_FIFOShiftRegisterEmptyStatus_Default.h"
#include "../templates/spi_FIFOInterruptMode_PIC32.h"
#include "../templates/spi_FramedCommunication_Default.h"
#include "../templates/spi_FrameSyncPulseDirection_Default.h"
#include "../templates/spi_FrameSyncPulsePolarity_Default.h"
#include "../templates/spi_FrameSyncPulseEdge_Default.h"
#include "../templates/spi_FrameSyncPulseWidth_Default.h"
#include "../templates/spi_FrameSyncPulseCounter_PIC32.h"
#include "../templates/spi_FrameErrorStatus_Default.h"
#include "../templates/spi_Buffer_Default.h"
#include "../templates/spi_BaudRateClock_Default.h"
#include "../templates/spi_ErrorInterruptControl_Default.h"
#include "../templates/spi_AudioErrorControl_Default.h"
#include "../templates/spi_AudioProtocolControl_Default.h"
#include "../templates/spi_AudioTransmitMode_Default.h"
#include "../templates/spi_AudioProtocolMode_Default.h"
#include "../templates/spi_Buffer32bit_Default.h"
#include "../templates/spi_Buffer16bit_Default.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void PLIB_SPI_Enable(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_Enable_Default(index);
            break;
        case SPI_ID_2 :
            SPI_Enable_Default(index);
            break;
        case SPI_ID_3 :
            SPI_Enable_Default(index);
            break;
        case SPI_ID_4 :
            SPI_Enable_Default(index);
            break;
        case SPI_ID_5 :
            SPI_Enable_Default(index);
            break;
        case SPI_ID_6 :
            SPI_Enable_Default(index);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_SPI_Disable(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_Disable_Default(index);
            break;
        case SPI_ID_2 :
            SPI_Disable_Default(index);
            break;
        case SPI_ID_3 :
            SPI_Disable_Default(index);
            break;
        case SPI_ID_4 :
            SPI_Disable_Default(index);
            break;
        case SPI_ID_5 :
            SPI_Disable_Default(index);
            break;
        case SPI_ID_6 :
            SPI_Disable_Default(index);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsEnableControl(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsEnableControl_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsEnableControl_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsEnableControl_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsEnableControl_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsEnableControl_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsEnableControl_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_StopInIdleEnable(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_StopInIdleEnable_Default(index);
            break;
        case SPI_ID_2 :
            SPI_StopInIdleEnable_Default(index);
            break;
        case SPI_ID_3 :
            SPI_StopInIdleEnable_Default(index);
            break;
        case SPI_ID_4 :
            SPI_StopInIdleEnable_Default(index);
            break;
        case SPI_ID_5 :
            SPI_StopInIdleEnable_Default(index);
            break;
        case SPI_ID_6 :
            SPI_StopInIdleEnable_Default(index);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_SPI_StopInIdleDisable(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_StopInIdleDisable_Default(index);
            break;
        case SPI_ID_2 :
            SPI_StopInIdleDisable_Default(index);
            break;
        case SPI_ID_3 :
            SPI_StopInIdleDisable_Default(index);
            break;
        case SPI_ID_4 :
            SPI_StopInIdleDisable_Default(index);
            break;
        case SPI_ID_5 :
            SPI_StopInIdleDisable_Default(index);
            break;
        case SPI_ID_6 :
            SPI_StopInIdleDisable_Default(index);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsStopInIdleControl(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsStopInIdleControl_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsStopInIdleControl_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsStopInIdleControl_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsStopInIdleControl_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsStopInIdleControl_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsStopInIdleControl_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ReceiverHasOverflowed(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ReceiverHasOverflowed_Default(index);
        case SPI_ID_2 :
            return SPI_ReceiverHasOverflowed_Default(index);
        case SPI_ID_3 :
            return SPI_ReceiverHasOverflowed_Default(index);
        case SPI_ID_4 :
            return SPI_ReceiverHasOverflowed_Default(index);
        case SPI_ID_5 :
            return SPI_ReceiverHasOverflowed_Default(index);
        case SPI_ID_6 :
            return SPI_ReceiverHasOverflowed_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_ReceiverOverflowClear(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_ReceiverOverflowClear_Default(index);
            break;
        case SPI_ID_2 :
            SPI_ReceiverOverflowClear_Default(index);
            break;
        case SPI_ID_3 :
            SPI_ReceiverOverflowClear_Default(index);
            break;
        case SPI_ID_4 :
            SPI_ReceiverOverflowClear_Default(index);
            break;
        case SPI_ID_5 :
            SPI_ReceiverOverflowClear_Default(index);
            break;
        case SPI_ID_6 :
            SPI_ReceiverOverflowClear_Default(index);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsReceiverOverflow(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsReceiverOverflow_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsReceiverOverflow_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsReceiverOverflow_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsReceiverOverflow_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsReceiverOverflow_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsReceiverOverflow_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_SPI_TransmitBufferIsFull(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_TransmitBufferIsFull_Default(index);
        case SPI_ID_2 :
            return SPI_TransmitBufferIsFull_Default(index);
        case SPI_ID_3 :
            return SPI_TransmitBufferIsFull_Default(index);
        case SPI_ID_4 :
            return SPI_TransmitBufferIsFull_Default(index);
        case SPI_ID_5 :
            return SPI_TransmitBufferIsFull_Default(index);
        case SPI_ID_6 :
            return SPI_TransmitBufferIsFull_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsTransmitBufferFullStatus(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsTransmitBufferFullStatus_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsTransmitBufferFullStatus_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsTransmitBufferFullStatus_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsTransmitBufferFullStatus_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsTransmitBufferFullStatus_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsTransmitBufferFullStatus_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_SPI_TransmitBufferIsEmpty(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_TransmitBufferIsEmpty_Default(index);
        case SPI_ID_2 :
            return SPI_TransmitBufferIsEmpty_Default(index);
        case SPI_ID_3 :
            return SPI_TransmitBufferIsEmpty_Default(index);
        case SPI_ID_4 :
            return SPI_TransmitBufferIsEmpty_Default(index);
        case SPI_ID_5 :
            return SPI_TransmitBufferIsEmpty_Default(index);
        case SPI_ID_6 :
            return SPI_TransmitBufferIsEmpty_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsTransmitBufferEmptyStatus(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsTransmitBufferEmptyStatus_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsTransmitBufferEmptyStatus_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsTransmitBufferEmptyStatus_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsTransmitBufferEmptyStatus_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsTransmitBufferEmptyStatus_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsTransmitBufferEmptyStatus_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ReceiverBufferIsFull(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ReceiverBufferIsFull_Default(index);
        case SPI_ID_2 :
            return SPI_ReceiverBufferIsFull_Default(index);
        case SPI_ID_3 :
            return SPI_ReceiverBufferIsFull_Default(index);
        case SPI_ID_4 :
            return SPI_ReceiverBufferIsFull_Default(index);
        case SPI_ID_5 :
            return SPI_ReceiverBufferIsFull_Default(index);
        case SPI_ID_6 :
            return SPI_ReceiverBufferIsFull_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsReceiveBufferStatus(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsReceiveBufferStatus_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsReceiveBufferStatus_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsReceiveBufferStatus_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsReceiveBufferStatus_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsReceiveBufferStatus_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsReceiveBufferStatus_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_PinEnable(SPI_MODULE_ID index, SPI_PIN pin)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_PinEnable_PIC32(index, pin);
            break;
        case SPI_ID_2 :
            SPI_PinEnable_PIC32(index, pin);
            break;
        case SPI_ID_3 :
            SPI_PinEnable_PIC32(index, pin);
            break;
        case SPI_ID_4 :
            SPI_PinEnable_PIC32(index, pin);
            break;
        case SPI_ID_5 :
            SPI_PinEnable_PIC32(index, pin);
            break;
        case SPI_ID_6 :
            SPI_PinEnable_PIC32(index, pin);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_SPI_PinDisable(SPI_MODULE_ID index, SPI_PIN pin)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_PinDisable_PIC32(index, pin);
            break;
        case SPI_ID_2 :
            SPI_PinDisable_PIC32(index, pin);
            break;
        case SPI_ID_3 :
            SPI_PinDisable_PIC32(index, pin);
            break;
        case SPI_ID_4 :
            SPI_PinDisable_PIC32(index, pin);
            break;
        case SPI_ID_5 :
            SPI_PinDisable_PIC32(index, pin);
            break;
        case SPI_ID_6 :
            SPI_PinDisable_PIC32(index, pin);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsPinControl(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsPinControl_PIC32(index);
        case SPI_ID_2 :
            return SPI_ExistsPinControl_PIC32(index);
        case SPI_ID_3 :
            return SPI_ExistsPinControl_PIC32(index);
        case SPI_ID_4 :
            return SPI_ExistsPinControl_PIC32(index);
        case SPI_ID_5 :
            return SPI_ExistsPinControl_PIC32(index);
        case SPI_ID_6 :
            return SPI_ExistsPinControl_PIC32(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_CommunicationWidthSelect(SPI_MODULE_ID index, SPI_COMMUNICATION_WIDTH width)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_CommunicationWidthSelect_PIC32(index, width);
            break;
        case SPI_ID_2 :
            SPI_CommunicationWidthSelect_PIC32(index, width);
            break;
        case SPI_ID_3 :
            SPI_CommunicationWidthSelect_PIC32(index, width);
            break;
        case SPI_ID_4 :
            SPI_CommunicationWidthSelect_PIC32(index, width);
            break;
        case SPI_ID_5 :
            SPI_CommunicationWidthSelect_PIC32(index, width);
            break;
        case SPI_ID_6 :
            SPI_CommunicationWidthSelect_PIC32(index, width);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsCommunicationWidth(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsCommunicationWidth_PIC32(index);
        case SPI_ID_2 :
            return SPI_ExistsCommunicationWidth_PIC32(index);
        case SPI_ID_3 :
            return SPI_ExistsCommunicationWidth_PIC32(index);
        case SPI_ID_4 :
            return SPI_ExistsCommunicationWidth_PIC32(index);
        case SPI_ID_5 :
            return SPI_ExistsCommunicationWidth_PIC32(index);
        case SPI_ID_6 :
            return SPI_ExistsCommunicationWidth_PIC32(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_AudioCommunicationWidthSelect(SPI_MODULE_ID index, SPI_AUDIO_COMMUNICATION_WIDTH mode)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_AudioCommunicationWidthSelect_PIC32(index, mode);
            break;
        case SPI_ID_2 :
            SPI_AudioCommunicationWidthSelect_PIC32(index, mode);
            break;
        case SPI_ID_3 :
            SPI_AudioCommunicationWidthSelect_PIC32(index, mode);
            break;
        case SPI_ID_4 :
            SPI_AudioCommunicationWidthSelect_PIC32(index, mode);
            break;
        case SPI_ID_5 :
            SPI_AudioCommunicationWidthSelect_PIC32(index, mode);
            break;
        case SPI_ID_6 :
            SPI_AudioCommunicationWidthSelect_PIC32(index, mode);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsAudioCommunicationWidth(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsAudioCommunicationWidth_PIC32(index);
        case SPI_ID_2 :
            return SPI_ExistsAudioCommunicationWidth_PIC32(index);
        case SPI_ID_3 :
            return SPI_ExistsAudioCommunicationWidth_PIC32(index);
        case SPI_ID_4 :
            return SPI_ExistsAudioCommunicationWidth_PIC32(index);
        case SPI_ID_5 :
            return SPI_ExistsAudioCommunicationWidth_PIC32(index);
        case SPI_ID_6 :
            return SPI_ExistsAudioCommunicationWidth_PIC32(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_InputSamplePhaseSelect(SPI_MODULE_ID index, SPI_INPUT_SAMPLING_PHASE phase)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_InputSamplePhaseSelect_Default(index, phase);
            break;
        case SPI_ID_2 :
            SPI_InputSamplePhaseSelect_Default(index, phase);
            break;
        case SPI_ID_3 :
            SPI_InputSamplePhaseSelect_Default(index, phase);
            break;
        case SPI_ID_4 :
            SPI_InputSamplePhaseSelect_Default(index, phase);
            break;
        case SPI_ID_5 :
            SPI_InputSamplePhaseSelect_Default(index, phase);
            break;
        case SPI_ID_6 :
            SPI_InputSamplePhaseSelect_Default(index, phase);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsInputSamplePhase(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsInputSamplePhase_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsInputSamplePhase_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsInputSamplePhase_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsInputSamplePhase_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsInputSamplePhase_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsInputSamplePhase_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_OutputDataPhaseSelect(SPI_MODULE_ID index, SPI_OUTPUT_DATA_PHASE phase)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_OutputDataPhaseSelect_Default(index, phase);
            break;
        case SPI_ID_2 :
            SPI_OutputDataPhaseSelect_Default(index, phase);
            break;
        case SPI_ID_3 :
            SPI_OutputDataPhaseSelect_Default(index, phase);
            break;
        case SPI_ID_4 :
            SPI_OutputDataPhaseSelect_Default(index, phase);
            break;
        case SPI_ID_5 :
            SPI_OutputDataPhaseSelect_Default(index, phase);
            break;
        case SPI_ID_6 :
            SPI_OutputDataPhaseSelect_Default(index, phase);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsOutputDataPhase(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsOutputDataPhase_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsOutputDataPhase_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsOutputDataPhase_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsOutputDataPhase_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsOutputDataPhase_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsOutputDataPhase_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_ClockPolaritySelect(SPI_MODULE_ID index, SPI_CLOCK_POLARITY polarity)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_ClockPolaritySelect_Default(index, polarity);
            break;
        case SPI_ID_2 :
            SPI_ClockPolaritySelect_Default(index, polarity);
            break;
        case SPI_ID_3 :
            SPI_ClockPolaritySelect_Default(index, polarity);
            break;
        case SPI_ID_4 :
            SPI_ClockPolaritySelect_Default(index, polarity);
            break;
        case SPI_ID_5 :
            SPI_ClockPolaritySelect_Default(index, polarity);
            break;
        case SPI_ID_6 :
            SPI_ClockPolaritySelect_Default(index, polarity);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsClockPolarity(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsClockPolarity_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsClockPolarity_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsClockPolarity_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsClockPolarity_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsClockPolarity_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsClockPolarity_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_MasterEnable(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_MasterEnable_Default(index);
            break;
        case SPI_ID_2 :
            SPI_MasterEnable_Default(index);
            break;
        case SPI_ID_3 :
            SPI_MasterEnable_Default(index);
            break;
        case SPI_ID_4 :
            SPI_MasterEnable_Default(index);
            break;
        case SPI_ID_5 :
            SPI_MasterEnable_Default(index);
            break;
        case SPI_ID_6 :
            SPI_MasterEnable_Default(index);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_SPI_SlaveEnable(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_SlaveEnable_Default(index);
            break;
        case SPI_ID_2 :
            SPI_SlaveEnable_Default(index);
            break;
        case SPI_ID_3 :
            SPI_SlaveEnable_Default(index);
            break;
        case SPI_ID_4 :
            SPI_SlaveEnable_Default(index);
            break;
        case SPI_ID_5 :
            SPI_SlaveEnable_Default(index);
            break;
        case SPI_ID_6 :
            SPI_SlaveEnable_Default(index);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsMasterControl(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsMasterControl_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsMasterControl_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsMasterControl_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsMasterControl_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsMasterControl_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsMasterControl_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_BaudRateSet(SPI_MODULE_ID index, uint32_t clockFrequency, uint32_t baudRate)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_BaudRateSet_Default(index, clockFrequency, baudRate);
            break;
        case SPI_ID_2 :
            SPI_BaudRateSet_Default(index, clockFrequency, baudRate);
            break;
        case SPI_ID_3 :
            SPI_BaudRateSet_Default(index, clockFrequency, baudRate);
            break;
        case SPI_ID_4 :
            SPI_BaudRateSet_Default(index, clockFrequency, baudRate);
            break;
        case SPI_ID_5 :
            SPI_BaudRateSet_Default(index, clockFrequency, baudRate);
            break;
        case SPI_ID_6 :
            SPI_BaudRateSet_Default(index, clockFrequency, baudRate);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsBaudRate(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsBaudRate_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsBaudRate_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsBaudRate_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsBaudRate_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsBaudRate_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsBaudRate_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_SPI_IsBusy(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_IsBusy_Default(index);
        case SPI_ID_2 :
            return SPI_IsBusy_Default(index);
        case SPI_ID_3 :
            return SPI_IsBusy_Default(index);
        case SPI_ID_4 :
            return SPI_IsBusy_Default(index);
        case SPI_ID_5 :
            return SPI_IsBusy_Default(index);
        case SPI_ID_6 :
            return SPI_IsBusy_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsBusStatus(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsBusStatus_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsBusStatus_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsBusStatus_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsBusStatus_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsBusStatus_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsBusStatus_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ReadDataIsSignExtended(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ReadDataIsSignExtended_Default(index);
        case SPI_ID_2 :
            return SPI_ReadDataIsSignExtended_Default(index);
        case SPI_ID_3 :
            return SPI_ReadDataIsSignExtended_Default(index);
        case SPI_ID_4 :
            return SPI_ReadDataIsSignExtended_Default(index);
        case SPI_ID_5 :
            return SPI_ReadDataIsSignExtended_Default(index);
        case SPI_ID_6 :
            return SPI_ReadDataIsSignExtended_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsReadDataSignStatus(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsReadDataSignStatus_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsReadDataSignStatus_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsReadDataSignStatus_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsReadDataSignStatus_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsReadDataSignStatus_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsReadDataSignStatus_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_SlaveSelectEnable(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_SlaveSelectEnable_Default(index);
            break;
        case SPI_ID_2 :
            SPI_SlaveSelectEnable_Default(index);
            break;
        case SPI_ID_3 :
            SPI_SlaveSelectEnable_Default(index);
            break;
        case SPI_ID_4 :
            SPI_SlaveSelectEnable_Default(index);
            break;
        case SPI_ID_5 :
            SPI_SlaveSelectEnable_Default(index);
            break;
        case SPI_ID_6 :
            SPI_SlaveSelectEnable_Default(index);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_SPI_SlaveSelectDisable(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_SlaveSelectDisable_Default(index);
            break;
        case SPI_ID_2 :
            SPI_SlaveSelectDisable_Default(index);
            break;
        case SPI_ID_3 :
            SPI_SlaveSelectDisable_Default(index);
            break;
        case SPI_ID_4 :
            SPI_SlaveSelectDisable_Default(index);
            break;
        case SPI_ID_5 :
            SPI_SlaveSelectDisable_Default(index);
            break;
        case SPI_ID_6 :
            SPI_SlaveSelectDisable_Default(index);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsSlaveSelectControl(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsSlaveSelectControl_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsSlaveSelectControl_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsSlaveSelectControl_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsSlaveSelectControl_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsSlaveSelectControl_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsSlaveSelectControl_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_SPI_TransmitUnderRunStatusGet(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_TransmitUnderRunStatusGet_Default(index);
        case SPI_ID_2 :
            return SPI_TransmitUnderRunStatusGet_Default(index);
        case SPI_ID_3 :
            return SPI_TransmitUnderRunStatusGet_Default(index);
        case SPI_ID_4 :
            return SPI_TransmitUnderRunStatusGet_Default(index);
        case SPI_ID_5 :
            return SPI_TransmitUnderRunStatusGet_Default(index);
        case SPI_ID_6 :
            return SPI_TransmitUnderRunStatusGet_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_TransmitUnderRunStatusClear(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_TransmitUnderRunStatusClear_Default(index);
            break;
        case SPI_ID_2 :
            SPI_TransmitUnderRunStatusClear_Default(index);
            break;
        case SPI_ID_3 :
            SPI_TransmitUnderRunStatusClear_Default(index);
            break;
        case SPI_ID_4 :
            SPI_TransmitUnderRunStatusClear_Default(index);
            break;
        case SPI_ID_5 :
            SPI_TransmitUnderRunStatusClear_Default(index);
            break;
        case SPI_ID_6 :
            SPI_TransmitUnderRunStatusClear_Default(index);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsTransmitUnderRunStatus(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsTransmitUnderRunStatus_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsTransmitUnderRunStatus_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsTransmitUnderRunStatus_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsTransmitUnderRunStatus_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsTransmitUnderRunStatus_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsTransmitUnderRunStatus_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_FIFOEnable(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_FIFOEnable_Default(index);
            break;
        case SPI_ID_2 :
            SPI_FIFOEnable_Default(index);
            break;
        case SPI_ID_3 :
            SPI_FIFOEnable_Default(index);
            break;
        case SPI_ID_4 :
            SPI_FIFOEnable_Default(index);
            break;
        case SPI_ID_5 :
            SPI_FIFOEnable_Default(index);
            break;
        case SPI_ID_6 :
            SPI_FIFOEnable_Default(index);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_SPI_FIFODisable(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_FIFODisable_Default(index);
            break;
        case SPI_ID_2 :
            SPI_FIFODisable_Default(index);
            break;
        case SPI_ID_3 :
            SPI_FIFODisable_Default(index);
            break;
        case SPI_ID_4 :
            SPI_FIFODisable_Default(index);
            break;
        case SPI_ID_5 :
            SPI_FIFODisable_Default(index);
            break;
        case SPI_ID_6 :
            SPI_FIFODisable_Default(index);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsFIFOControl(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsFIFOControl_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsFIFOControl_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsFIFOControl_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsFIFOControl_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsFIFOControl_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsFIFOControl_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint8_t PLIB_SPI_FIFOCountGet(SPI_MODULE_ID index, SPI_FIFO_TYPE type)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_FIFOCountGet_PIC32(index, type);
        case SPI_ID_2 :
            return SPI_FIFOCountGet_PIC32(index, type);
        case SPI_ID_3 :
            return SPI_FIFOCountGet_PIC32(index, type);
        case SPI_ID_4 :
            return SPI_FIFOCountGet_PIC32(index, type);
        case SPI_ID_5 :
            return SPI_FIFOCountGet_PIC32(index, type);
        case SPI_ID_6 :
            return SPI_FIFOCountGet_PIC32(index, type);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsFIFOCount(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsFIFOCount_PIC32(index);
        case SPI_ID_2 :
            return SPI_ExistsFIFOCount_PIC32(index);
        case SPI_ID_3 :
            return SPI_ExistsFIFOCount_PIC32(index);
        case SPI_ID_4 :
            return SPI_ExistsFIFOCount_PIC32(index);
        case SPI_ID_5 :
            return SPI_ExistsFIFOCount_PIC32(index);
        case SPI_ID_6 :
            return SPI_ExistsFIFOCount_PIC32(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ReceiverFIFOIsEmpty(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ReceiverFIFOIsEmpty_Default(index);
        case SPI_ID_2 :
            return SPI_ReceiverFIFOIsEmpty_Default(index);
        case SPI_ID_3 :
            return SPI_ReceiverFIFOIsEmpty_Default(index);
        case SPI_ID_4 :
            return SPI_ReceiverFIFOIsEmpty_Default(index);
        case SPI_ID_5 :
            return SPI_ReceiverFIFOIsEmpty_Default(index);
        case SPI_ID_6 :
            return SPI_ReceiverFIFOIsEmpty_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsReceiveFIFOStatus(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsReceiveFIFOStatus_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsReceiveFIFOStatus_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsReceiveFIFOStatus_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsReceiveFIFOStatus_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsReceiveFIFOStatus_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsReceiveFIFOStatus_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_SPI_FIFOShiftRegisterIsEmpty(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_FIFOShiftRegisterIsEmpty_Default(index);
        case SPI_ID_2 :
            return SPI_FIFOShiftRegisterIsEmpty_Default(index);
        case SPI_ID_3 :
            return SPI_FIFOShiftRegisterIsEmpty_Default(index);
        case SPI_ID_4 :
            return SPI_FIFOShiftRegisterIsEmpty_Default(index);
        case SPI_ID_5 :
            return SPI_FIFOShiftRegisterIsEmpty_Default(index);
        case SPI_ID_6 :
            return SPI_FIFOShiftRegisterIsEmpty_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsFIFOShiftRegisterEmptyStatus(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsFIFOShiftRegisterEmptyStatus_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsFIFOShiftRegisterEmptyStatus_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsFIFOShiftRegisterEmptyStatus_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsFIFOShiftRegisterEmptyStatus_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsFIFOShiftRegisterEmptyStatus_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsFIFOShiftRegisterEmptyStatus_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_FIFOInterruptModeSelect(SPI_MODULE_ID index, SPI_FIFO_INTERRUPT mode)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_FIFOInterruptModeSelect_PIC32(index, mode);
            break;
        case SPI_ID_2 :
            SPI_FIFOInterruptModeSelect_PIC32(index, mode);
            break;
        case SPI_ID_3 :
            SPI_FIFOInterruptModeSelect_PIC32(index, mode);
            break;
        case SPI_ID_4 :
            SPI_FIFOInterruptModeSelect_PIC32(index, mode);
            break;
        case SPI_ID_5 :
            SPI_FIFOInterruptModeSelect_PIC32(index, mode);
            break;
        case SPI_ID_6 :
            SPI_FIFOInterruptModeSelect_PIC32(index, mode);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsFIFOInterruptMode(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsFIFOInterruptMode_PIC32(index);
        case SPI_ID_2 :
            return SPI_ExistsFIFOInterruptMode_PIC32(index);
        case SPI_ID_3 :
            return SPI_ExistsFIFOInterruptMode_PIC32(index);
        case SPI_ID_4 :
            return SPI_ExistsFIFOInterruptMode_PIC32(index);
        case SPI_ID_5 :
            return SPI_ExistsFIFOInterruptMode_PIC32(index);
        case SPI_ID_6 :
            return SPI_ExistsFIFOInterruptMode_PIC32(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_FramedCommunicationEnable(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_FramedCommunicationEnable_Default(index);
            break;
        case SPI_ID_2 :
            SPI_FramedCommunicationEnable_Default(index);
            break;
        case SPI_ID_3 :
            SPI_FramedCommunicationEnable_Default(index);
            break;
        case SPI_ID_4 :
            SPI_FramedCommunicationEnable_Default(index);
            break;
        case SPI_ID_5 :
            SPI_FramedCommunicationEnable_Default(index);
            break;
        case SPI_ID_6 :
            SPI_FramedCommunicationEnable_Default(index);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_SPI_FramedCommunicationDisable(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_FramedCommunicationDisable_Default(index);
            break;
        case SPI_ID_2 :
            SPI_FramedCommunicationDisable_Default(index);
            break;
        case SPI_ID_3 :
            SPI_FramedCommunicationDisable_Default(index);
            break;
        case SPI_ID_4 :
            SPI_FramedCommunicationDisable_Default(index);
            break;
        case SPI_ID_5 :
            SPI_FramedCommunicationDisable_Default(index);
            break;
        case SPI_ID_6 :
            SPI_FramedCommunicationDisable_Default(index);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsFramedCommunication(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsFramedCommunication_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsFramedCommunication_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsFramedCommunication_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsFramedCommunication_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsFramedCommunication_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsFramedCommunication_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_FrameSyncPulseDirectionSelect(SPI_MODULE_ID index, SPI_FRAME_PULSE_DIRECTION direction)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_FrameSyncPulseDirectionSelect_Default(index, direction);
            break;
        case SPI_ID_2 :
            SPI_FrameSyncPulseDirectionSelect_Default(index, direction);
            break;
        case SPI_ID_3 :
            SPI_FrameSyncPulseDirectionSelect_Default(index, direction);
            break;
        case SPI_ID_4 :
            SPI_FrameSyncPulseDirectionSelect_Default(index, direction);
            break;
        case SPI_ID_5 :
            SPI_FrameSyncPulseDirectionSelect_Default(index, direction);
            break;
        case SPI_ID_6 :
            SPI_FrameSyncPulseDirectionSelect_Default(index, direction);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsFrameSyncPulseDirection(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsFrameSyncPulseDirection_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsFrameSyncPulseDirection_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsFrameSyncPulseDirection_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsFrameSyncPulseDirection_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsFrameSyncPulseDirection_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsFrameSyncPulseDirection_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_FrameSyncPulsePolaritySelect(SPI_MODULE_ID index, SPI_FRAME_PULSE_POLARITY polarity)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_FrameSyncPulsePolaritySelect_Default(index, polarity);
            break;
        case SPI_ID_2 :
            SPI_FrameSyncPulsePolaritySelect_Default(index, polarity);
            break;
        case SPI_ID_3 :
            SPI_FrameSyncPulsePolaritySelect_Default(index, polarity);
            break;
        case SPI_ID_4 :
            SPI_FrameSyncPulsePolaritySelect_Default(index, polarity);
            break;
        case SPI_ID_5 :
            SPI_FrameSyncPulsePolaritySelect_Default(index, polarity);
            break;
        case SPI_ID_6 :
            SPI_FrameSyncPulsePolaritySelect_Default(index, polarity);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsFrameSyncPulsePolarity(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsFrameSyncPulsePolarity_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsFrameSyncPulsePolarity_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsFrameSyncPulsePolarity_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsFrameSyncPulsePolarity_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsFrameSyncPulsePolarity_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsFrameSyncPulsePolarity_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_FrameSyncPulseEdgeSelect(SPI_MODULE_ID index, SPI_FRAME_PULSE_EDGE edge)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_FrameSyncPulseEdgeSelect_Default(index, edge);
            break;
        case SPI_ID_2 :
            SPI_FrameSyncPulseEdgeSelect_Default(index, edge);
            break;
        case SPI_ID_3 :
            SPI_FrameSyncPulseEdgeSelect_Default(index, edge);
            break;
        case SPI_ID_4 :
            SPI_FrameSyncPulseEdgeSelect_Default(index, edge);
            break;
        case SPI_ID_5 :
            SPI_FrameSyncPulseEdgeSelect_Default(index, edge);
            break;
        case SPI_ID_6 :
            SPI_FrameSyncPulseEdgeSelect_Default(index, edge);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsFrameSyncPulseEdge(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsFrameSyncPulseEdge_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsFrameSyncPulseEdge_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsFrameSyncPulseEdge_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsFrameSyncPulseEdge_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsFrameSyncPulseEdge_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsFrameSyncPulseEdge_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_FrameSyncPulseWidthSelect(SPI_MODULE_ID index, SPI_FRAME_PULSE_WIDTH width)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_FrameSyncPulseWidthSelect_Default(index, width);
            break;
        case SPI_ID_2 :
            SPI_FrameSyncPulseWidthSelect_Default(index, width);
            break;
        case SPI_ID_3 :
            SPI_FrameSyncPulseWidthSelect_Default(index, width);
            break;
        case SPI_ID_4 :
            SPI_FrameSyncPulseWidthSelect_Default(index, width);
            break;
        case SPI_ID_5 :
            SPI_FrameSyncPulseWidthSelect_Default(index, width);
            break;
        case SPI_ID_6 :
            SPI_FrameSyncPulseWidthSelect_Default(index, width);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsFrameSyncPulseWidth(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsFrameSyncPulseWidth_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsFrameSyncPulseWidth_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsFrameSyncPulseWidth_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsFrameSyncPulseWidth_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsFrameSyncPulseWidth_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsFrameSyncPulseWidth_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_FrameSyncPulseCounterSelect(SPI_MODULE_ID index, SPI_FRAME_SYNC_PULSE pulse)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_FrameSyncPulseCounterSelect_PIC32(index, pulse);
            break;
        case SPI_ID_2 :
            SPI_FrameSyncPulseCounterSelect_PIC32(index, pulse);
            break;
        case SPI_ID_3 :
            SPI_FrameSyncPulseCounterSelect_PIC32(index, pulse);
            break;
        case SPI_ID_4 :
            SPI_FrameSyncPulseCounterSelect_PIC32(index, pulse);
            break;
        case SPI_ID_5 :
            SPI_FrameSyncPulseCounterSelect_PIC32(index, pulse);
            break;
        case SPI_ID_6 :
            SPI_FrameSyncPulseCounterSelect_PIC32(index, pulse);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsFrameSyncPulseCounter(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsFrameSyncPulseCounter_PIC32(index);
        case SPI_ID_2 :
            return SPI_ExistsFrameSyncPulseCounter_PIC32(index);
        case SPI_ID_3 :
            return SPI_ExistsFrameSyncPulseCounter_PIC32(index);
        case SPI_ID_4 :
            return SPI_ExistsFrameSyncPulseCounter_PIC32(index);
        case SPI_ID_5 :
            return SPI_ExistsFrameSyncPulseCounter_PIC32(index);
        case SPI_ID_6 :
            return SPI_ExistsFrameSyncPulseCounter_PIC32(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_SPI_FrameErrorStatusGet(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_FrameErrorStatusGet_Default(index);
        case SPI_ID_2 :
            return SPI_FrameErrorStatusGet_Default(index);
        case SPI_ID_3 :
            return SPI_FrameErrorStatusGet_Default(index);
        case SPI_ID_4 :
            return SPI_FrameErrorStatusGet_Default(index);
        case SPI_ID_5 :
            return SPI_FrameErrorStatusGet_Default(index);
        case SPI_ID_6 :
            return SPI_FrameErrorStatusGet_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_FrameErrorStatusClear(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_FrameErrorStatusClear_Default(index);
            break;
        case SPI_ID_2 :
            SPI_FrameErrorStatusClear_Default(index);
            break;
        case SPI_ID_3 :
            SPI_FrameErrorStatusClear_Default(index);
            break;
        case SPI_ID_4 :
            SPI_FrameErrorStatusClear_Default(index);
            break;
        case SPI_ID_5 :
            SPI_FrameErrorStatusClear_Default(index);
            break;
        case SPI_ID_6 :
            SPI_FrameErrorStatusClear_Default(index);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsFrameErrorStatus(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsFrameErrorStatus_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsFrameErrorStatus_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsFrameErrorStatus_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsFrameErrorStatus_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsFrameErrorStatus_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsFrameErrorStatus_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_BufferClear(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_BufferClear_Default(index);
            break;
        case SPI_ID_2 :
            SPI_BufferClear_Default(index);
            break;
        case SPI_ID_3 :
            SPI_BufferClear_Default(index);
            break;
        case SPI_ID_4 :
            SPI_BufferClear_Default(index);
            break;
        case SPI_ID_5 :
            SPI_BufferClear_Default(index);
            break;
        case SPI_ID_6 :
            SPI_BufferClear_Default(index);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint8_t PLIB_SPI_BufferRead(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_BufferRead_Default(index);
        case SPI_ID_2 :
            return SPI_BufferRead_Default(index);
        case SPI_ID_3 :
            return SPI_BufferRead_Default(index);
        case SPI_ID_4 :
            return SPI_BufferRead_Default(index);
        case SPI_ID_5 :
            return SPI_BufferRead_Default(index);
        case SPI_ID_6 :
            return SPI_BufferRead_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_BufferWrite(SPI_MODULE_ID index, uint8_t data)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_BufferWrite_Default(index, data);
            break;
        case SPI_ID_2 :
            SPI_BufferWrite_Default(index, data);
            break;
        case SPI_ID_3 :
            SPI_BufferWrite_Default(index, data);
            break;
        case SPI_ID_4 :
            SPI_BufferWrite_Default(index, data);
            break;
        case SPI_ID_5 :
            SPI_BufferWrite_Default(index, data);
            break;
        case SPI_ID_6 :
            SPI_BufferWrite_Default(index, data);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsBuffer(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsBuffer_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsBuffer_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsBuffer_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsBuffer_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsBuffer_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsBuffer_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void* PLIB_SPI_BufferAddressGet(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_BufferAddressGet_Default(index);
        case SPI_ID_2 :
            return SPI_BufferAddressGet_Default(index);
        case SPI_ID_3 :
            return SPI_BufferAddressGet_Default(index);
        case SPI_ID_4 :
            return SPI_BufferAddressGet_Default(index);
        case SPI_ID_5 :
            return SPI_BufferAddressGet_Default(index);
        case SPI_ID_6 :
            return SPI_BufferAddressGet_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (void*)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_BaudRateClockSelect(SPI_MODULE_ID index, SPI_BAUD_RATE_CLOCK type)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_BaudRateClockSelect_Default(index, type);
            break;
        case SPI_ID_2 :
            SPI_BaudRateClockSelect_Default(index, type);
            break;
        case SPI_ID_3 :
            SPI_BaudRateClockSelect_Default(index, type);
            break;
        case SPI_ID_4 :
            SPI_BaudRateClockSelect_Default(index, type);
            break;
        case SPI_ID_5 :
            SPI_BaudRateClockSelect_Default(index, type);
            break;
        case SPI_ID_6 :
            SPI_BaudRateClockSelect_Default(index, type);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsBaudRateClock(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsBaudRateClock_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsBaudRateClock_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsBaudRateClock_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsBaudRateClock_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsBaudRateClock_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsBaudRateClock_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_ErrorInterruptEnable(SPI_MODULE_ID index, SPI_ERROR_INTERRUPT error)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_ErrorInterruptEnable_Default(index, error);
            break;
        case SPI_ID_2 :
            SPI_ErrorInterruptEnable_Default(index, error);
            break;
        case SPI_ID_3 :
            SPI_ErrorInterruptEnable_Default(index, error);
            break;
        case SPI_ID_4 :
            SPI_ErrorInterruptEnable_Default(index, error);
            break;
        case SPI_ID_5 :
            SPI_ErrorInterruptEnable_Default(index, error);
            break;
        case SPI_ID_6 :
            SPI_ErrorInterruptEnable_Default(index, error);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_SPI_ErrorInterruptDisable(SPI_MODULE_ID index, SPI_ERROR_INTERRUPT error)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_ErrorInterruptDisable_Default(index, error);
            break;
        case SPI_ID_2 :
            SPI_ErrorInterruptDisable_Default(index, error);
            break;
        case SPI_ID_3 :
            SPI_ErrorInterruptDisable_Default(index, error);
            break;
        case SPI_ID_4 :
            SPI_ErrorInterruptDisable_Default(index, error);
            break;
        case SPI_ID_5 :
            SPI_ErrorInterruptDisable_Default(index, error);
            break;
        case SPI_ID_6 :
            SPI_ErrorInterruptDisable_Default(index, error);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsErrorInterruptControl(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsErrorInterruptControl_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsErrorInterruptControl_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsErrorInterruptControl_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsErrorInterruptControl_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsErrorInterruptControl_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsErrorInterruptControl_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_AudioErrorEnable(SPI_MODULE_ID index, SPI_AUDIO_ERROR error)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_AudioErrorEnable_Default(index, error);
            break;
        case SPI_ID_2 :
            SPI_AudioErrorEnable_Default(index, error);
            break;
        case SPI_ID_3 :
            SPI_AudioErrorEnable_Default(index, error);
            break;
        case SPI_ID_4 :
            SPI_AudioErrorEnable_Default(index, error);
            break;
        case SPI_ID_5 :
            SPI_AudioErrorEnable_Default(index, error);
            break;
        case SPI_ID_6 :
            SPI_AudioErrorEnable_Default(index, error);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_SPI_AudioErrorDisable(SPI_MODULE_ID index, SPI_AUDIO_ERROR error)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_AudioErrorDisable_Default(index, error);
            break;
        case SPI_ID_2 :
            SPI_AudioErrorDisable_Default(index, error);
            break;
        case SPI_ID_3 :
            SPI_AudioErrorDisable_Default(index, error);
            break;
        case SPI_ID_4 :
            SPI_AudioErrorDisable_Default(index, error);
            break;
        case SPI_ID_5 :
            SPI_AudioErrorDisable_Default(index, error);
            break;
        case SPI_ID_6 :
            SPI_AudioErrorDisable_Default(index, error);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsAudioErrorControl(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsAudioErrorControl_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsAudioErrorControl_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsAudioErrorControl_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsAudioErrorControl_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsAudioErrorControl_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsAudioErrorControl_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_AudioProtocolEnable(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_AudioProtocolEnable_Default(index);
            break;
        case SPI_ID_2 :
            SPI_AudioProtocolEnable_Default(index);
            break;
        case SPI_ID_3 :
            SPI_AudioProtocolEnable_Default(index);
            break;
        case SPI_ID_4 :
            SPI_AudioProtocolEnable_Default(index);
            break;
        case SPI_ID_5 :
            SPI_AudioProtocolEnable_Default(index);
            break;
        case SPI_ID_6 :
            SPI_AudioProtocolEnable_Default(index);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_SPI_AudioProtocolDisable(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_AudioProtocolDisable_Default(index);
            break;
        case SPI_ID_2 :
            SPI_AudioProtocolDisable_Default(index);
            break;
        case SPI_ID_3 :
            SPI_AudioProtocolDisable_Default(index);
            break;
        case SPI_ID_4 :
            SPI_AudioProtocolDisable_Default(index);
            break;
        case SPI_ID_5 :
            SPI_AudioProtocolDisable_Default(index);
            break;
        case SPI_ID_6 :
            SPI_AudioProtocolDisable_Default(index);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsAudioProtocolControl(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsAudioProtocolControl_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsAudioProtocolControl_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsAudioProtocolControl_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsAudioProtocolControl_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsAudioProtocolControl_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsAudioProtocolControl_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_AudioTransmitModeSelect(SPI_MODULE_ID index, SPI_AUDIO_TRANSMIT_MODE mode)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_AudioTransmitModeSelect_Default(index, mode);
            break;
        case SPI_ID_2 :
            SPI_AudioTransmitModeSelect_Default(index, mode);
            break;
        case SPI_ID_3 :
            SPI_AudioTransmitModeSelect_Default(index, mode);
            break;
        case SPI_ID_4 :
            SPI_AudioTransmitModeSelect_Default(index, mode);
            break;
        case SPI_ID_5 :
            SPI_AudioTransmitModeSelect_Default(index, mode);
            break;
        case SPI_ID_6 :
            SPI_AudioTransmitModeSelect_Default(index, mode);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsAudioTransmitMode(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsAudioTransmitMode_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsAudioTransmitMode_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsAudioTransmitMode_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsAudioTransmitMode_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsAudioTransmitMode_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsAudioTransmitMode_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_AudioProtocolModeSelect(SPI_MODULE_ID index, SPI_AUDIO_PROTOCOL mode)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_AudioProtocolModeSelect_Default(index, mode);
            break;
        case SPI_ID_2 :
            SPI_AudioProtocolModeSelect_Default(index, mode);
            break;
        case SPI_ID_3 :
            SPI_AudioProtocolModeSelect_Default(index, mode);
            break;
        case SPI_ID_4 :
            SPI_AudioProtocolModeSelect_Default(index, mode);
            break;
        case SPI_ID_5 :
            SPI_AudioProtocolModeSelect_Default(index, mode);
            break;
        case SPI_ID_6 :
            SPI_AudioProtocolModeSelect_Default(index, mode);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_ExistsAudioProtocolMode(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_ExistsAudioProtocolMode_Default(index);
        case SPI_ID_2 :
            return SPI_ExistsAudioProtocolMode_Default(index);
        case SPI_ID_3 :
            return SPI_ExistsAudioProtocolMode_Default(index);
        case SPI_ID_4 :
            return SPI_ExistsAudioProtocolMode_Default(index);
        case SPI_ID_5 :
            return SPI_ExistsAudioProtocolMode_Default(index);
        case SPI_ID_6 :
            return SPI_ExistsAudioProtocolMode_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint32_t PLIB_SPI_BufferRead32bit(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_BufferRead32bit_Default(index);
        case SPI_ID_2 :
            return SPI_BufferRead32bit_Default(index);
        case SPI_ID_3 :
            return SPI_BufferRead32bit_Default(index);
        case SPI_ID_4 :
            return SPI_BufferRead32bit_Default(index);
        case SPI_ID_5 :
            return SPI_BufferRead32bit_Default(index);
        case SPI_ID_6 :
            return SPI_BufferRead32bit_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (uint32_t)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_BufferWrite32bit(SPI_MODULE_ID index, uint32_t data)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_BufferWrite32bit_Default(index, data);
            break;
        case SPI_ID_2 :
            SPI_BufferWrite32bit_Default(index, data);
            break;
        case SPI_ID_3 :
            SPI_BufferWrite32bit_Default(index, data);
            break;
        case SPI_ID_4 :
            SPI_BufferWrite32bit_Default(index, data);
            break;
        case SPI_ID_5 :
            SPI_BufferWrite32bit_Default(index, data);
            break;
        case SPI_ID_6 :
            SPI_BufferWrite32bit_Default(index, data);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_Exists32bitBuffer(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_Exists32bitBuffer_Default(index);
        case SPI_ID_2 :
            return SPI_Exists32bitBuffer_Default(index);
        case SPI_ID_3 :
            return SPI_Exists32bitBuffer_Default(index);
        case SPI_ID_4 :
            return SPI_Exists32bitBuffer_Default(index);
        case SPI_ID_5 :
            return SPI_Exists32bitBuffer_Default(index);
        case SPI_ID_6 :
            return SPI_Exists32bitBuffer_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint16_t PLIB_SPI_BufferRead16bit(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_BufferRead16bit_Default(index);
        case SPI_ID_2 :
            return SPI_BufferRead16bit_Default(index);
        case SPI_ID_3 :
            return SPI_BufferRead16bit_Default(index);
        case SPI_ID_4 :
            return SPI_BufferRead16bit_Default(index);
        case SPI_ID_5 :
            return SPI_BufferRead16bit_Default(index);
        case SPI_ID_6 :
            return SPI_BufferRead16bit_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API void PLIB_SPI_BufferWrite16bit(SPI_MODULE_ID index, uint16_t data)
{
    switch (index) {
        case SPI_ID_1 :
            SPI_BufferWrite16bit_Default(index, data);
            break;
        case SPI_ID_2 :
            SPI_BufferWrite16bit_Default(index, data);
            break;
        case SPI_ID_3 :
            SPI_BufferWrite16bit_Default(index, data);
            break;
        case SPI_ID_4 :
            SPI_BufferWrite16bit_Default(index, data);
            break;
        case SPI_ID_5 :
            SPI_BufferWrite16bit_Default(index, data);
            break;
        case SPI_ID_6 :
            SPI_BufferWrite16bit_Default(index, data);
            break;
        case SPI_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_SPI_Exists16bitBuffer(SPI_MODULE_ID index)
{
    switch (index) {
        case SPI_ID_1 :
            return SPI_Exists16bitBuffer_Default(index);
        case SPI_ID_2 :
            return SPI_Exists16bitBuffer_Default(index);
        case SPI_ID_3 :
            return SPI_Exists16bitBuffer_Default(index);
        case SPI_ID_4 :
            return SPI_Exists16bitBuffer_Default(index);
        case SPI_ID_5 :
            return SPI_Exists16bitBuffer_Default(index);
        case SPI_ID_6 :
            return SPI_Exists16bitBuffer_Default(index);
        case SPI_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

#endif
