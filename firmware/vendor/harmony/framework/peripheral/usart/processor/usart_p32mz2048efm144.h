/* Created by plibgen $Revision: 1.31 $ */

#ifndef _USART_P32MZ2048EFM144_H
#define _USART_P32MZ2048EFM144_H

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

    USART_ID_1 = 0,
    USART_ID_2,
    USART_ID_3,
    USART_ID_4,
    USART_ID_5,
    USART_ID_6,
    USART_NUMBER_OF_MODULES

} USART_MODULE_ID;

typedef enum {

    USART_HANDSHAKE_MODE_FLOW_CONTROL = 0x00,
    USART_HANDSHAKE_MODE_SIMPLEX = 0x01

} USART_HANDSHAKE_MODE;

typedef enum {

    USART_ENABLE_TX_RX_BCLK_USED = 0x03,
    USART_ENABLE_TX_RX_CTS_RTS_USED = 0x02,
    USART_ENABLE_TX_RX_RTS_USED = 0x01,
    USART_ENABLE_TX_RX_USED = 0x00

} USART_OPERATION_MODE;

typedef enum {

    USART_8N1 = 0x00,
    USART_8E1 = 0x01,
    USART_8O1 = 0x02,
    USART_9N1 = 0x03,
    USART_8N2 = 0x04,
    USART_8E2 = 0x05,
    USART_8O2 = 0x06,
    USART_9N2 = 0x07

} USART_LINECONTROL_MODE;

typedef enum {

    USART_BRG_CLOCK_SOURCE_NONE

} USART_BRG_CLOCK_SOURCE;

typedef enum {

    USART_ERROR_NONE = 0x00,
    USART_ERROR_RECEIVER_OVERRUN = 0x01,
    USART_ERROR_FRAMING = 0x02,
    USART_ERROR_PARITY = 0x04

} USART_ERROR;

typedef enum {

    USART_TRANSMIT_FIFO_NOT_FULL = 0x00,
    USART_TRANSMIT_FIFO_IDLE = 0x01,
    USART_TRANSMIT_FIFO_EMPTY = 0x02

} USART_TRANSMIT_INTR_MODE;

typedef enum {

    USART_RECEIVE_FIFO_3B4FULL = 0x02,
    USART_RECEIVE_FIFO_HALF_FULL = 0x01,
    USART_RECEIVE_FIFO_ONE_CHAR = 0x00

} USART_RECEIVE_INTR_MODE;

PLIB_INLINE SFR_TYPE* _USART_LINE_CONTROL_MODE_STOP_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1MODE;
        case USART_ID_2 :
            return &U2MODE;
        case USART_ID_3 :
            return &U3MODE;
        case USART_ID_4 :
            return &U4MODE;
        case USART_ID_5 :
            return &U5MODE;
        case USART_ID_6 :
            return &U6MODE;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_LINE_CONTROL_MODE_DATA_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1MODE;
        case USART_ID_2 :
            return &U2MODE;
        case USART_ID_3 :
            return &U3MODE;
        case USART_ID_4 :
            return &U4MODE;
        case USART_ID_5 :
            return &U5MODE;
        case USART_ID_6 :
            return &U6MODE;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_BAUD_RATE_HIGH_ENABLE_16_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1MODE;
        case USART_ID_2 :
            return &U2MODE;
        case USART_ID_3 :
            return &U3MODE;
        case USART_ID_4 :
            return &U4MODE;
        case USART_ID_5 :
            return &U5MODE;
        case USART_ID_6 :
            return &U6MODE;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_RECEIVER_POLARITY_INVERT_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1MODE;
        case USART_ID_2 :
            return &U2MODE;
        case USART_ID_3 :
            return &U3MODE;
        case USART_ID_4 :
            return &U4MODE;
        case USART_ID_5 :
            return &U5MODE;
        case USART_ID_6 :
            return &U6MODE;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_BAUD_RATE_AUTO_DETECT_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1MODE;
        case USART_ID_2 :
            return &U2MODE;
        case USART_ID_3 :
            return &U3MODE;
        case USART_ID_4 :
            return &U4MODE;
        case USART_ID_5 :
            return &U5MODE;
        case USART_ID_6 :
            return &U6MODE;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_LOOP_BACK_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1MODE;
        case USART_ID_2 :
            return &U2MODE;
        case USART_ID_3 :
            return &U3MODE;
        case USART_ID_4 :
            return &U4MODE;
        case USART_ID_5 :
            return &U5MODE;
        case USART_ID_6 :
            return &U6MODE;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_WAKE_ON_START_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1MODE;
        case USART_ID_2 :
            return &U2MODE;
        case USART_ID_3 :
            return &U3MODE;
        case USART_ID_4 :
            return &U4MODE;
        case USART_ID_5 :
            return &U5MODE;
        case USART_ID_6 :
            return &U6MODE;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_OPERATION_MODE_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1MODE;
        case USART_ID_2 :
            return &U2MODE;
        case USART_ID_3 :
            return &U3MODE;
        case USART_ID_4 :
            return &U4MODE;
        case USART_ID_5 :
            return &U5MODE;
        case USART_ID_6 :
            return &U6MODE;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_HAND_SHAKE_MODE_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1MODE;
        case USART_ID_2 :
            return &U2MODE;
        case USART_ID_3 :
            return &U3MODE;
        case USART_ID_4 :
            return &U4MODE;
        case USART_ID_5 :
            return &U5MODE;
        case USART_ID_6 :
            return &U6MODE;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_IRDA_ENABLE_CONTROL_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1MODE;
        case USART_ID_2 :
            return &U2MODE;
        case USART_ID_3 :
            return &U3MODE;
        case USART_ID_4 :
            return &U4MODE;
        case USART_ID_5 :
            return &U5MODE;
        case USART_ID_6 :
            return &U6MODE;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_STOP_IN_IDLE_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1MODE;
        case USART_ID_2 :
            return &U2MODE;
        case USART_ID_3 :
            return &U3MODE;
        case USART_ID_4 :
            return &U4MODE;
        case USART_ID_5 :
            return &U5MODE;
        case USART_ID_6 :
            return &U6MODE;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_ENABLE_CONTROL_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1MODE;
        case USART_ID_2 :
            return &U2MODE;
        case USART_ID_3 :
            return &U3MODE;
        case USART_ID_4 :
            return &U4MODE;
        case USART_ID_5 :
            return &U5MODE;
        case USART_ID_6 :
            return &U6MODE;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_RECEIVER_DATA_AVAILABLE_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1STA;
        case USART_ID_2 :
            return &U2STA;
        case USART_ID_3 :
            return &U3STA;
        case USART_ID_4 :
            return &U4STA;
        case USART_ID_5 :
            return &U5STA;
        case USART_ID_6 :
            return &U6STA;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_RECEIVER_OVERRUN_ERROR_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1STA;
        case USART_ID_2 :
            return &U2STA;
        case USART_ID_3 :
            return &U3STA;
        case USART_ID_4 :
            return &U4STA;
        case USART_ID_5 :
            return &U5STA;
        case USART_ID_6 :
            return &U6STA;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_RECEIVER_FRAMING_ERROR_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1STA;
        case USART_ID_2 :
            return &U2STA;
        case USART_ID_3 :
            return &U3STA;
        case USART_ID_4 :
            return &U4STA;
        case USART_ID_5 :
            return &U5STA;
        case USART_ID_6 :
            return &U6STA;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_RECEIVER_PARITY_ERROR_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1STA;
        case USART_ID_2 :
            return &U2STA;
        case USART_ID_3 :
            return &U3STA;
        case USART_ID_4 :
            return &U4STA;
        case USART_ID_5 :
            return &U5STA;
        case USART_ID_6 :
            return &U6STA;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_RECEIVER_IDLE_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1STA;
        case USART_ID_2 :
            return &U2STA;
        case USART_ID_3 :
            return &U3STA;
        case USART_ID_4 :
            return &U4STA;
        case USART_ID_5 :
            return &U5STA;
        case USART_ID_6 :
            return &U6STA;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_RECEIVER_ADDRESS_DETECT_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1STA;
        case USART_ID_2 :
            return &U2STA;
        case USART_ID_3 :
            return &U3STA;
        case USART_ID_4 :
            return &U4STA;
        case USART_ID_5 :
            return &U5STA;
        case USART_ID_6 :
            return &U6STA;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_RECEIVER_FIFO_LEVEL_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1STA;
        case USART_ID_2 :
            return &U2STA;
        case USART_ID_3 :
            return &U3STA;
        case USART_ID_4 :
            return &U4STA;
        case USART_ID_5 :
            return &U5STA;
        case USART_ID_6 :
            return &U6STA;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_TRANSMITTER_EMPTY_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1STA;
        case USART_ID_2 :
            return &U2STA;
        case USART_ID_3 :
            return &U3STA;
        case USART_ID_4 :
            return &U4STA;
        case USART_ID_5 :
            return &U5STA;
        case USART_ID_6 :
            return &U6STA;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_TRANSMITTER_BUFFER_FULL_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1STA;
        case USART_ID_2 :
            return &U2STA;
        case USART_ID_3 :
            return &U3STA;
        case USART_ID_4 :
            return &U4STA;
        case USART_ID_5 :
            return &U5STA;
        case USART_ID_6 :
            return &U6STA;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_TRANSMITTER_ENABLE_CONTROL_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1STA;
        case USART_ID_2 :
            return &U2STA;
        case USART_ID_3 :
            return &U3STA;
        case USART_ID_4 :
            return &U4STA;
        case USART_ID_5 :
            return &U5STA;
        case USART_ID_6 :
            return &U6STA;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_TRANSMITTER_BREAK_SEND_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1STA;
        case USART_ID_2 :
            return &U2STA;
        case USART_ID_3 :
            return &U3STA;
        case USART_ID_4 :
            return &U4STA;
        case USART_ID_5 :
            return &U5STA;
        case USART_ID_6 :
            return &U6STA;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_RECEIVER_ENABLE_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1STA;
        case USART_ID_2 :
            return &U2STA;
        case USART_ID_3 :
            return &U3STA;
        case USART_ID_4 :
            return &U4STA;
        case USART_ID_5 :
            return &U5STA;
        case USART_ID_6 :
            return &U6STA;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_TRANSMITTER_IDLE_IS_LOW32_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1STA;
        case USART_ID_2 :
            return &U2STA;
        case USART_ID_3 :
            return &U3STA;
        case USART_ID_4 :
            return &U4STA;
        case USART_ID_5 :
            return &U5STA;
        case USART_ID_6 :
            return &U6STA;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_TRANSMITTER_INTR_MODE_SELECT_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1STA;
        case USART_ID_2 :
            return &U2STA;
        case USART_ID_3 :
            return &U3STA;
        case USART_ID_4 :
            return &U4STA;
        case USART_ID_5 :
            return &U5STA;
        case USART_ID_6 :
            return &U6STA;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_RECEIVER_ADDRESS_AUTO_DETECT_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1STA;
        case USART_ID_2 :
            return &U2STA;
        case USART_ID_3 :
            return &U3STA;
        case USART_ID_4 :
            return &U4STA;
        case USART_ID_5 :
            return &U5STA;
        case USART_ID_6 :
            return &U6STA;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_RECEIVER_ADDRESS_AUTO_DETECT_ENABLE_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1STA;
        case USART_ID_2 :
            return &U2STA;
        case USART_ID_3 :
            return &U3STA;
        case USART_ID_4 :
            return &U4STA;
        case USART_ID_5 :
            return &U5STA;
        case USART_ID_6 :
            return &U6STA;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_TRANSMITTER_SEND_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1TXREG;
        case USART_ID_2 :
            return &U2TXREG;
        case USART_ID_3 :
            return &U3TXREG;
        case USART_ID_4 :
            return &U4TXREG;
        case USART_ID_5 :
            return &U5TXREG;
        case USART_ID_6 :
            return &U6TXREG;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_TRANSMITTER_DATA_ONLY_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1TXREG;
        case USART_ID_2 :
            return &U2TXREG;
        case USART_ID_3 :
            return &U3TXREG;
        case USART_ID_4 :
            return &U4TXREG;
        case USART_ID_5 :
            return &U5TXREG;
        case USART_ID_6 :
            return &U6TXREG;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_RECEIVER_DATA_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1RXREG;
        case USART_ID_2 :
            return &U2RXREG;
        case USART_ID_3 :
            return &U3RXREG;
        case USART_ID_4 :
            return &U4RXREG;
        case USART_ID_5 :
            return &U5RXREG;
        case USART_ID_6 :
            return &U6RXREG;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_BAUD_RATE_HIGH_16_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1BRG;
        case USART_ID_2 :
            return &U2BRG;
        case USART_ID_3 :
            return &U3BRG;
        case USART_ID_4 :
            return &U4BRG;
        case USART_ID_5 :
            return &U5BRG;
        case USART_ID_6 :
            return &U6BRG;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _USART_BAUD_RATE_VREG(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return &U1BRG;
        case USART_ID_2 :
            return &U2BRG;
        case USART_ID_3 :
            return &U3BRG;
        case USART_ID_4 :
            return &U4BRG;
        case USART_ID_5 :
            return &U5BRG;
        case USART_ID_6 :
            return &U6BRG;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_LINE_CONTROL_MODE_STOP_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_STSEL_MASK;
        case USART_ID_2 :
            return _U2MODE_STSEL_MASK;
        case USART_ID_3 :
            return _U3MODE_STSEL_MASK;
        case USART_ID_4 :
            return _U4MODE_STSEL_MASK;
        case USART_ID_5 :
            return _U5MODE_STSEL_MASK;
        case USART_ID_6 :
            return _U6MODE_STSEL_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_LINE_CONTROL_MODE_DATA_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_PDSEL_MASK;
        case USART_ID_2 :
            return _U2MODE_PDSEL_MASK;
        case USART_ID_3 :
            return _U3MODE_PDSEL_MASK;
        case USART_ID_4 :
            return _U4MODE_PDSEL_MASK;
        case USART_ID_5 :
            return _U5MODE_PDSEL_MASK;
        case USART_ID_6 :
            return _U6MODE_PDSEL_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_BAUD_RATE_HIGH_ENABLE_16_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_BRGH_MASK;
        case USART_ID_2 :
            return _U2MODE_BRGH_MASK;
        case USART_ID_3 :
            return _U3MODE_BRGH_MASK;
        case USART_ID_4 :
            return _U4MODE_BRGH_MASK;
        case USART_ID_5 :
            return _U5MODE_BRGH_MASK;
        case USART_ID_6 :
            return _U6MODE_BRGH_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_POLARITY_INVERT_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_RXINV_MASK;
        case USART_ID_2 :
            return _U2MODE_RXINV_MASK;
        case USART_ID_3 :
            return _U3MODE_RXINV_MASK;
        case USART_ID_4 :
            return _U4MODE_RXINV_MASK;
        case USART_ID_5 :
            return _U5MODE_RXINV_MASK;
        case USART_ID_6 :
            return _U6MODE_RXINV_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_BAUD_RATE_AUTO_DETECT_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_ABAUD_MASK;
        case USART_ID_2 :
            return _U2MODE_ABAUD_MASK;
        case USART_ID_3 :
            return _U3MODE_ABAUD_MASK;
        case USART_ID_4 :
            return _U4MODE_ABAUD_MASK;
        case USART_ID_5 :
            return _U5MODE_ABAUD_MASK;
        case USART_ID_6 :
            return _U6MODE_ABAUD_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_LOOP_BACK_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_LPBACK_MASK;
        case USART_ID_2 :
            return _U2MODE_LPBACK_MASK;
        case USART_ID_3 :
            return _U3MODE_LPBACK_MASK;
        case USART_ID_4 :
            return _U4MODE_LPBACK_MASK;
        case USART_ID_5 :
            return _U5MODE_LPBACK_MASK;
        case USART_ID_6 :
            return _U6MODE_LPBACK_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_WAKE_ON_START_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_WAKE_MASK;
        case USART_ID_2 :
            return _U2MODE_WAKE_MASK;
        case USART_ID_3 :
            return _U3MODE_WAKE_MASK;
        case USART_ID_4 :
            return _U4MODE_WAKE_MASK;
        case USART_ID_5 :
            return _U5MODE_WAKE_MASK;
        case USART_ID_6 :
            return _U6MODE_WAKE_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_OPERATION_MODE_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_UEN_MASK;
        case USART_ID_2 :
            return _U2MODE_UEN_MASK;
        case USART_ID_3 :
            return _U3MODE_UEN_MASK;
        case USART_ID_4 :
            return _U4MODE_UEN_MASK;
        case USART_ID_5 :
            return _U5MODE_UEN_MASK;
        case USART_ID_6 :
            return _U6MODE_UEN_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_HAND_SHAKE_MODE_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_RTSMD_MASK;
        case USART_ID_2 :
            return _U2MODE_RTSMD_MASK;
        case USART_ID_3 :
            return _U3MODE_RTSMD_MASK;
        case USART_ID_4 :
            return _U4MODE_RTSMD_MASK;
        case USART_ID_5 :
            return _U5MODE_RTSMD_MASK;
        case USART_ID_6 :
            return _U6MODE_RTSMD_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_IRDA_ENABLE_CONTROL_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_IREN_MASK;
        case USART_ID_2 :
            return _U2MODE_IREN_MASK;
        case USART_ID_3 :
            return _U3MODE_IREN_MASK;
        case USART_ID_4 :
            return _U4MODE_IREN_MASK;
        case USART_ID_5 :
            return _U5MODE_IREN_MASK;
        case USART_ID_6 :
            return _U6MODE_IREN_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_STOP_IN_IDLE_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_SIDL_MASK;
        case USART_ID_2 :
            return _U2MODE_SIDL_MASK;
        case USART_ID_3 :
            return _U3MODE_SIDL_MASK;
        case USART_ID_4 :
            return _U4MODE_SIDL_MASK;
        case USART_ID_5 :
            return _U5MODE_SIDL_MASK;
        case USART_ID_6 :
            return _U6MODE_SIDL_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_ENABLE_CONTROL_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_ON_MASK;
        case USART_ID_2 :
            return _U2MODE_ON_MASK;
        case USART_ID_3 :
            return _U3MODE_ON_MASK;
        case USART_ID_4 :
            return _U4MODE_ON_MASK;
        case USART_ID_5 :
            return _U5MODE_ON_MASK;
        case USART_ID_6 :
            return _U6MODE_ON_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_DATA_AVAILABLE_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_URXDA_MASK;
        case USART_ID_2 :
            return _U2STA_URXDA_MASK;
        case USART_ID_3 :
            return _U3STA_URXDA_MASK;
        case USART_ID_4 :
            return _U4STA_URXDA_MASK;
        case USART_ID_5 :
            return _U5STA_URXDA_MASK;
        case USART_ID_6 :
            return _U6STA_URXDA_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_OVERRUN_ERROR_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_OERR_MASK;
        case USART_ID_2 :
            return _U2STA_OERR_MASK;
        case USART_ID_3 :
            return _U3STA_OERR_MASK;
        case USART_ID_4 :
            return _U4STA_OERR_MASK;
        case USART_ID_5 :
            return _U5STA_OERR_MASK;
        case USART_ID_6 :
            return _U6STA_OERR_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_FRAMING_ERROR_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_FERR_MASK;
        case USART_ID_2 :
            return _U2STA_FERR_MASK;
        case USART_ID_3 :
            return _U3STA_FERR_MASK;
        case USART_ID_4 :
            return _U4STA_FERR_MASK;
        case USART_ID_5 :
            return _U5STA_FERR_MASK;
        case USART_ID_6 :
            return _U6STA_FERR_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_PARITY_ERROR_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_PERR_MASK;
        case USART_ID_2 :
            return _U2STA_PERR_MASK;
        case USART_ID_3 :
            return _U3STA_PERR_MASK;
        case USART_ID_4 :
            return _U4STA_PERR_MASK;
        case USART_ID_5 :
            return _U5STA_PERR_MASK;
        case USART_ID_6 :
            return _U6STA_PERR_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_IDLE_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_RIDLE_MASK;
        case USART_ID_2 :
            return _U2STA_RIDLE_MASK;
        case USART_ID_3 :
            return _U3STA_RIDLE_MASK;
        case USART_ID_4 :
            return _U4STA_RIDLE_MASK;
        case USART_ID_5 :
            return _U5STA_RIDLE_MASK;
        case USART_ID_6 :
            return _U6STA_RIDLE_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_ADDRESS_DETECT_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_ADDEN_MASK;
        case USART_ID_2 :
            return _U2STA_ADDEN_MASK;
        case USART_ID_3 :
            return _U3STA_ADDEN_MASK;
        case USART_ID_4 :
            return _U4STA_ADDEN_MASK;
        case USART_ID_5 :
            return _U5STA_ADDEN_MASK;
        case USART_ID_6 :
            return _U6STA_ADDEN_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_FIFO_LEVEL_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_URXISEL_MASK;
        case USART_ID_2 :
            return _U2STA_URXISEL_MASK;
        case USART_ID_3 :
            return _U3STA_URXISEL_MASK;
        case USART_ID_4 :
            return _U4STA_URXISEL_MASK;
        case USART_ID_5 :
            return _U5STA_URXISEL_MASK;
        case USART_ID_6 :
            return _U6STA_URXISEL_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_EMPTY_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_TRMT_MASK;
        case USART_ID_2 :
            return _U2STA_TRMT_MASK;
        case USART_ID_3 :
            return _U3STA_TRMT_MASK;
        case USART_ID_4 :
            return _U4STA_TRMT_MASK;
        case USART_ID_5 :
            return _U5STA_TRMT_MASK;
        case USART_ID_6 :
            return _U6STA_TRMT_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_BUFFER_FULL_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_UTXBF_MASK;
        case USART_ID_2 :
            return _U2STA_UTXBF_MASK;
        case USART_ID_3 :
            return _U3STA_UTXBF_MASK;
        case USART_ID_4 :
            return _U4STA_UTXBF_MASK;
        case USART_ID_5 :
            return _U5STA_UTXBF_MASK;
        case USART_ID_6 :
            return _U6STA_UTXBF_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_ENABLE_CONTROL_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_UTXEN_MASK;
        case USART_ID_2 :
            return _U2STA_UTXEN_MASK;
        case USART_ID_3 :
            return _U3STA_UTXEN_MASK;
        case USART_ID_4 :
            return _U4STA_UTXEN_MASK;
        case USART_ID_5 :
            return _U5STA_UTXEN_MASK;
        case USART_ID_6 :
            return _U6STA_UTXEN_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_BREAK_SEND_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_UTXBRK_MASK;
        case USART_ID_2 :
            return _U2STA_UTXBRK_MASK;
        case USART_ID_3 :
            return _U3STA_UTXBRK_MASK;
        case USART_ID_4 :
            return _U4STA_UTXBRK_MASK;
        case USART_ID_5 :
            return _U5STA_UTXBRK_MASK;
        case USART_ID_6 :
            return _U6STA_UTXBRK_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_ENABLE_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_URXEN_MASK;
        case USART_ID_2 :
            return _U2STA_URXEN_MASK;
        case USART_ID_3 :
            return _U3STA_URXEN_MASK;
        case USART_ID_4 :
            return _U4STA_URXEN_MASK;
        case USART_ID_5 :
            return _U5STA_URXEN_MASK;
        case USART_ID_6 :
            return _U6STA_URXEN_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_IDLE_IS_LOW32_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_UTXINV_MASK;
        case USART_ID_2 :
            return _U2STA_UTXINV_MASK;
        case USART_ID_3 :
            return _U3STA_UTXINV_MASK;
        case USART_ID_4 :
            return _U4STA_UTXINV_MASK;
        case USART_ID_5 :
            return _U5STA_UTXINV_MASK;
        case USART_ID_6 :
            return _U6STA_UTXINV_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_INTR_MODE_SELECT_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_UTXISEL_MASK;
        case USART_ID_2 :
            return _U2STA_UTXISEL_MASK;
        case USART_ID_3 :
            return _U3STA_UTXISEL_MASK;
        case USART_ID_4 :
            return _U4STA_UTXISEL_MASK;
        case USART_ID_5 :
            return _U5STA_UTXISEL_MASK;
        case USART_ID_6 :
            return _U6STA_UTXISEL_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_ADDRESS_AUTO_DETECT_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_ADDR_MASK;
        case USART_ID_2 :
            return _U2STA_ADDR_MASK;
        case USART_ID_3 :
            return _U3STA_ADDR_MASK;
        case USART_ID_4 :
            return _U4STA_ADDR_MASK;
        case USART_ID_5 :
            return _U5STA_ADDR_MASK;
        case USART_ID_6 :
            return _U6STA_ADDR_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_ADDRESS_AUTO_DETECT_ENABLE_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_ADM_EN_MASK;
        case USART_ID_2 :
            return _U2STA_ADM_EN_MASK;
        case USART_ID_3 :
            return _U3STA_ADM_EN_MASK;
        case USART_ID_4 :
            return _U4STA_ADM_EN_MASK;
        case USART_ID_5 :
            return _U5STA_ADM_EN_MASK;
        case USART_ID_6 :
            return _U6STA_ADM_EN_MASK;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_SEND_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return (SFR_DATA)-1;
        case USART_ID_2 :
            return (SFR_DATA)-1;
        case USART_ID_3 :
            return (SFR_DATA)-1;
        case USART_ID_4 :
            return (SFR_DATA)-1;
        case USART_ID_5 :
            return (SFR_DATA)-1;
        case USART_ID_6 :
            return (SFR_DATA)-1;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_DATA_ONLY_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return (SFR_DATA)-1;
        case USART_ID_2 :
            return (SFR_DATA)-1;
        case USART_ID_3 :
            return (SFR_DATA)-1;
        case USART_ID_4 :
            return (SFR_DATA)-1;
        case USART_ID_5 :
            return (SFR_DATA)-1;
        case USART_ID_6 :
            return (SFR_DATA)-1;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_DATA_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return (SFR_DATA)-1;
        case USART_ID_2 :
            return (SFR_DATA)-1;
        case USART_ID_3 :
            return (SFR_DATA)-1;
        case USART_ID_4 :
            return (SFR_DATA)-1;
        case USART_ID_5 :
            return (SFR_DATA)-1;
        case USART_ID_6 :
            return (SFR_DATA)-1;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_BAUD_RATE_HIGH_16_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return (SFR_DATA)-1;
        case USART_ID_2 :
            return (SFR_DATA)-1;
        case USART_ID_3 :
            return (SFR_DATA)-1;
        case USART_ID_4 :
            return (SFR_DATA)-1;
        case USART_ID_5 :
            return (SFR_DATA)-1;
        case USART_ID_6 :
            return (SFR_DATA)-1;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_BAUD_RATE_MASK(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return (SFR_DATA)-1;
        case USART_ID_2 :
            return (SFR_DATA)-1;
        case USART_ID_3 :
            return (SFR_DATA)-1;
        case USART_ID_4 :
            return (SFR_DATA)-1;
        case USART_ID_5 :
            return (SFR_DATA)-1;
        case USART_ID_6 :
            return (SFR_DATA)-1;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_LINE_CONTROL_MODE_STOP_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_STSEL_POSITION;
        case USART_ID_2 :
            return _U2MODE_STSEL_POSITION;
        case USART_ID_3 :
            return _U3MODE_STSEL_POSITION;
        case USART_ID_4 :
            return _U4MODE_STSEL_POSITION;
        case USART_ID_5 :
            return _U5MODE_STSEL_POSITION;
        case USART_ID_6 :
            return _U6MODE_STSEL_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_LINE_CONTROL_MODE_DATA_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_PDSEL_POSITION;
        case USART_ID_2 :
            return _U2MODE_PDSEL_POSITION;
        case USART_ID_3 :
            return _U3MODE_PDSEL_POSITION;
        case USART_ID_4 :
            return _U4MODE_PDSEL_POSITION;
        case USART_ID_5 :
            return _U5MODE_PDSEL_POSITION;
        case USART_ID_6 :
            return _U6MODE_PDSEL_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_BAUD_RATE_HIGH_ENABLE_16_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_BRGH_POSITION;
        case USART_ID_2 :
            return _U2MODE_BRGH_POSITION;
        case USART_ID_3 :
            return _U3MODE_BRGH_POSITION;
        case USART_ID_4 :
            return _U4MODE_BRGH_POSITION;
        case USART_ID_5 :
            return _U5MODE_BRGH_POSITION;
        case USART_ID_6 :
            return _U6MODE_BRGH_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_POLARITY_INVERT_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_RXINV_POSITION;
        case USART_ID_2 :
            return _U2MODE_RXINV_POSITION;
        case USART_ID_3 :
            return _U3MODE_RXINV_POSITION;
        case USART_ID_4 :
            return _U4MODE_RXINV_POSITION;
        case USART_ID_5 :
            return _U5MODE_RXINV_POSITION;
        case USART_ID_6 :
            return _U6MODE_RXINV_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_BAUD_RATE_AUTO_DETECT_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_ABAUD_POSITION;
        case USART_ID_2 :
            return _U2MODE_ABAUD_POSITION;
        case USART_ID_3 :
            return _U3MODE_ABAUD_POSITION;
        case USART_ID_4 :
            return _U4MODE_ABAUD_POSITION;
        case USART_ID_5 :
            return _U5MODE_ABAUD_POSITION;
        case USART_ID_6 :
            return _U6MODE_ABAUD_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_LOOP_BACK_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_LPBACK_POSITION;
        case USART_ID_2 :
            return _U2MODE_LPBACK_POSITION;
        case USART_ID_3 :
            return _U3MODE_LPBACK_POSITION;
        case USART_ID_4 :
            return _U4MODE_LPBACK_POSITION;
        case USART_ID_5 :
            return _U5MODE_LPBACK_POSITION;
        case USART_ID_6 :
            return _U6MODE_LPBACK_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_WAKE_ON_START_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_WAKE_POSITION;
        case USART_ID_2 :
            return _U2MODE_WAKE_POSITION;
        case USART_ID_3 :
            return _U3MODE_WAKE_POSITION;
        case USART_ID_4 :
            return _U4MODE_WAKE_POSITION;
        case USART_ID_5 :
            return _U5MODE_WAKE_POSITION;
        case USART_ID_6 :
            return _U6MODE_WAKE_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_OPERATION_MODE_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_UEN_POSITION;
        case USART_ID_2 :
            return _U2MODE_UEN_POSITION;
        case USART_ID_3 :
            return _U3MODE_UEN_POSITION;
        case USART_ID_4 :
            return _U4MODE_UEN_POSITION;
        case USART_ID_5 :
            return _U5MODE_UEN_POSITION;
        case USART_ID_6 :
            return _U6MODE_UEN_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_HAND_SHAKE_MODE_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_RTSMD_POSITION;
        case USART_ID_2 :
            return _U2MODE_RTSMD_POSITION;
        case USART_ID_3 :
            return _U3MODE_RTSMD_POSITION;
        case USART_ID_4 :
            return _U4MODE_RTSMD_POSITION;
        case USART_ID_5 :
            return _U5MODE_RTSMD_POSITION;
        case USART_ID_6 :
            return _U6MODE_RTSMD_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_IRDA_ENABLE_CONTROL_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_IREN_POSITION;
        case USART_ID_2 :
            return _U2MODE_IREN_POSITION;
        case USART_ID_3 :
            return _U3MODE_IREN_POSITION;
        case USART_ID_4 :
            return _U4MODE_IREN_POSITION;
        case USART_ID_5 :
            return _U5MODE_IREN_POSITION;
        case USART_ID_6 :
            return _U6MODE_IREN_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_STOP_IN_IDLE_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_SIDL_POSITION;
        case USART_ID_2 :
            return _U2MODE_SIDL_POSITION;
        case USART_ID_3 :
            return _U3MODE_SIDL_POSITION;
        case USART_ID_4 :
            return _U4MODE_SIDL_POSITION;
        case USART_ID_5 :
            return _U5MODE_SIDL_POSITION;
        case USART_ID_6 :
            return _U6MODE_SIDL_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_ENABLE_CONTROL_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_ON_POSITION;
        case USART_ID_2 :
            return _U2MODE_ON_POSITION;
        case USART_ID_3 :
            return _U3MODE_ON_POSITION;
        case USART_ID_4 :
            return _U4MODE_ON_POSITION;
        case USART_ID_5 :
            return _U5MODE_ON_POSITION;
        case USART_ID_6 :
            return _U6MODE_ON_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_DATA_AVAILABLE_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_URXDA_POSITION;
        case USART_ID_2 :
            return _U2STA_URXDA_POSITION;
        case USART_ID_3 :
            return _U3STA_URXDA_POSITION;
        case USART_ID_4 :
            return _U4STA_URXDA_POSITION;
        case USART_ID_5 :
            return _U5STA_URXDA_POSITION;
        case USART_ID_6 :
            return _U6STA_URXDA_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_OVERRUN_ERROR_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_OERR_POSITION;
        case USART_ID_2 :
            return _U2STA_OERR_POSITION;
        case USART_ID_3 :
            return _U3STA_OERR_POSITION;
        case USART_ID_4 :
            return _U4STA_OERR_POSITION;
        case USART_ID_5 :
            return _U5STA_OERR_POSITION;
        case USART_ID_6 :
            return _U6STA_OERR_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_FRAMING_ERROR_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_FERR_POSITION;
        case USART_ID_2 :
            return _U2STA_FERR_POSITION;
        case USART_ID_3 :
            return _U3STA_FERR_POSITION;
        case USART_ID_4 :
            return _U4STA_FERR_POSITION;
        case USART_ID_5 :
            return _U5STA_FERR_POSITION;
        case USART_ID_6 :
            return _U6STA_FERR_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_PARITY_ERROR_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_PERR_POSITION;
        case USART_ID_2 :
            return _U2STA_PERR_POSITION;
        case USART_ID_3 :
            return _U3STA_PERR_POSITION;
        case USART_ID_4 :
            return _U4STA_PERR_POSITION;
        case USART_ID_5 :
            return _U5STA_PERR_POSITION;
        case USART_ID_6 :
            return _U6STA_PERR_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_IDLE_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_RIDLE_POSITION;
        case USART_ID_2 :
            return _U2STA_RIDLE_POSITION;
        case USART_ID_3 :
            return _U3STA_RIDLE_POSITION;
        case USART_ID_4 :
            return _U4STA_RIDLE_POSITION;
        case USART_ID_5 :
            return _U5STA_RIDLE_POSITION;
        case USART_ID_6 :
            return _U6STA_RIDLE_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_ADDRESS_DETECT_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_ADDEN_POSITION;
        case USART_ID_2 :
            return _U2STA_ADDEN_POSITION;
        case USART_ID_3 :
            return _U3STA_ADDEN_POSITION;
        case USART_ID_4 :
            return _U4STA_ADDEN_POSITION;
        case USART_ID_5 :
            return _U5STA_ADDEN_POSITION;
        case USART_ID_6 :
            return _U6STA_ADDEN_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_FIFO_LEVEL_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_URXISEL_POSITION;
        case USART_ID_2 :
            return _U2STA_URXISEL_POSITION;
        case USART_ID_3 :
            return _U3STA_URXISEL_POSITION;
        case USART_ID_4 :
            return _U4STA_URXISEL_POSITION;
        case USART_ID_5 :
            return _U5STA_URXISEL_POSITION;
        case USART_ID_6 :
            return _U6STA_URXISEL_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_EMPTY_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_TRMT_POSITION;
        case USART_ID_2 :
            return _U2STA_TRMT_POSITION;
        case USART_ID_3 :
            return _U3STA_TRMT_POSITION;
        case USART_ID_4 :
            return _U4STA_TRMT_POSITION;
        case USART_ID_5 :
            return _U5STA_TRMT_POSITION;
        case USART_ID_6 :
            return _U6STA_TRMT_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_BUFFER_FULL_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_UTXBF_POSITION;
        case USART_ID_2 :
            return _U2STA_UTXBF_POSITION;
        case USART_ID_3 :
            return _U3STA_UTXBF_POSITION;
        case USART_ID_4 :
            return _U4STA_UTXBF_POSITION;
        case USART_ID_5 :
            return _U5STA_UTXBF_POSITION;
        case USART_ID_6 :
            return _U6STA_UTXBF_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_ENABLE_CONTROL_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_UTXEN_POSITION;
        case USART_ID_2 :
            return _U2STA_UTXEN_POSITION;
        case USART_ID_3 :
            return _U3STA_UTXEN_POSITION;
        case USART_ID_4 :
            return _U4STA_UTXEN_POSITION;
        case USART_ID_5 :
            return _U5STA_UTXEN_POSITION;
        case USART_ID_6 :
            return _U6STA_UTXEN_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_BREAK_SEND_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_UTXBRK_POSITION;
        case USART_ID_2 :
            return _U2STA_UTXBRK_POSITION;
        case USART_ID_3 :
            return _U3STA_UTXBRK_POSITION;
        case USART_ID_4 :
            return _U4STA_UTXBRK_POSITION;
        case USART_ID_5 :
            return _U5STA_UTXBRK_POSITION;
        case USART_ID_6 :
            return _U6STA_UTXBRK_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_ENABLE_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_URXEN_POSITION;
        case USART_ID_2 :
            return _U2STA_URXEN_POSITION;
        case USART_ID_3 :
            return _U3STA_URXEN_POSITION;
        case USART_ID_4 :
            return _U4STA_URXEN_POSITION;
        case USART_ID_5 :
            return _U5STA_URXEN_POSITION;
        case USART_ID_6 :
            return _U6STA_URXEN_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_IDLE_IS_LOW32_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_UTXINV_POSITION;
        case USART_ID_2 :
            return _U2STA_UTXINV_POSITION;
        case USART_ID_3 :
            return _U3STA_UTXINV_POSITION;
        case USART_ID_4 :
            return _U4STA_UTXINV_POSITION;
        case USART_ID_5 :
            return _U5STA_UTXINV_POSITION;
        case USART_ID_6 :
            return _U6STA_UTXINV_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_INTR_MODE_SELECT_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_UTXISEL_POSITION;
        case USART_ID_2 :
            return _U2STA_UTXISEL_POSITION;
        case USART_ID_3 :
            return _U3STA_UTXISEL_POSITION;
        case USART_ID_4 :
            return _U4STA_UTXISEL_POSITION;
        case USART_ID_5 :
            return _U5STA_UTXISEL_POSITION;
        case USART_ID_6 :
            return _U6STA_UTXISEL_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_ADDRESS_AUTO_DETECT_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_ADDR_POSITION;
        case USART_ID_2 :
            return _U2STA_ADDR_POSITION;
        case USART_ID_3 :
            return _U3STA_ADDR_POSITION;
        case USART_ID_4 :
            return _U4STA_ADDR_POSITION;
        case USART_ID_5 :
            return _U5STA_ADDR_POSITION;
        case USART_ID_6 :
            return _U6STA_ADDR_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_ADDRESS_AUTO_DETECT_ENABLE_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_ADM_EN_POSITION;
        case USART_ID_2 :
            return _U2STA_ADM_EN_POSITION;
        case USART_ID_3 :
            return _U3STA_ADM_EN_POSITION;
        case USART_ID_4 :
            return _U4STA_ADM_EN_POSITION;
        case USART_ID_5 :
            return _U5STA_ADM_EN_POSITION;
        case USART_ID_6 :
            return _U6STA_ADM_EN_POSITION;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_SEND_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return (SFR_DATA)0;
        case USART_ID_2 :
            return (SFR_DATA)0;
        case USART_ID_3 :
            return (SFR_DATA)0;
        case USART_ID_4 :
            return (SFR_DATA)0;
        case USART_ID_5 :
            return (SFR_DATA)0;
        case USART_ID_6 :
            return (SFR_DATA)0;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_DATA_ONLY_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return (SFR_DATA)0;
        case USART_ID_2 :
            return (SFR_DATA)0;
        case USART_ID_3 :
            return (SFR_DATA)0;
        case USART_ID_4 :
            return (SFR_DATA)0;
        case USART_ID_5 :
            return (SFR_DATA)0;
        case USART_ID_6 :
            return (SFR_DATA)0;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_DATA_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return (SFR_DATA)0;
        case USART_ID_2 :
            return (SFR_DATA)0;
        case USART_ID_3 :
            return (SFR_DATA)0;
        case USART_ID_4 :
            return (SFR_DATA)0;
        case USART_ID_5 :
            return (SFR_DATA)0;
        case USART_ID_6 :
            return (SFR_DATA)0;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_BAUD_RATE_HIGH_16_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return (SFR_DATA)0;
        case USART_ID_2 :
            return (SFR_DATA)0;
        case USART_ID_3 :
            return (SFR_DATA)0;
        case USART_ID_4 :
            return (SFR_DATA)0;
        case USART_ID_5 :
            return (SFR_DATA)0;
        case USART_ID_6 :
            return (SFR_DATA)0;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_BAUD_RATE_POS(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return (SFR_DATA)0;
        case USART_ID_2 :
            return (SFR_DATA)0;
        case USART_ID_3 :
            return (SFR_DATA)0;
        case USART_ID_4 :
            return (SFR_DATA)0;
        case USART_ID_5 :
            return (SFR_DATA)0;
        case USART_ID_6 :
            return (SFR_DATA)0;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_LINE_CONTROL_MODE_STOP_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_STSEL_LENGTH;
        case USART_ID_2 :
            return _U2MODE_STSEL_LENGTH;
        case USART_ID_3 :
            return _U3MODE_STSEL_LENGTH;
        case USART_ID_4 :
            return _U4MODE_STSEL_LENGTH;
        case USART_ID_5 :
            return _U5MODE_STSEL_LENGTH;
        case USART_ID_6 :
            return _U6MODE_STSEL_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_LINE_CONTROL_MODE_DATA_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_PDSEL_LENGTH;
        case USART_ID_2 :
            return _U2MODE_PDSEL_LENGTH;
        case USART_ID_3 :
            return _U3MODE_PDSEL_LENGTH;
        case USART_ID_4 :
            return _U4MODE_PDSEL_LENGTH;
        case USART_ID_5 :
            return _U5MODE_PDSEL_LENGTH;
        case USART_ID_6 :
            return _U6MODE_PDSEL_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_BAUD_RATE_HIGH_ENABLE_16_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_BRGH_LENGTH;
        case USART_ID_2 :
            return _U2MODE_BRGH_LENGTH;
        case USART_ID_3 :
            return _U3MODE_BRGH_LENGTH;
        case USART_ID_4 :
            return _U4MODE_BRGH_LENGTH;
        case USART_ID_5 :
            return _U5MODE_BRGH_LENGTH;
        case USART_ID_6 :
            return _U6MODE_BRGH_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_POLARITY_INVERT_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_RXINV_LENGTH;
        case USART_ID_2 :
            return _U2MODE_RXINV_LENGTH;
        case USART_ID_3 :
            return _U3MODE_RXINV_LENGTH;
        case USART_ID_4 :
            return _U4MODE_RXINV_LENGTH;
        case USART_ID_5 :
            return _U5MODE_RXINV_LENGTH;
        case USART_ID_6 :
            return _U6MODE_RXINV_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_BAUD_RATE_AUTO_DETECT_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_ABAUD_LENGTH;
        case USART_ID_2 :
            return _U2MODE_ABAUD_LENGTH;
        case USART_ID_3 :
            return _U3MODE_ABAUD_LENGTH;
        case USART_ID_4 :
            return _U4MODE_ABAUD_LENGTH;
        case USART_ID_5 :
            return _U5MODE_ABAUD_LENGTH;
        case USART_ID_6 :
            return _U6MODE_ABAUD_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_LOOP_BACK_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_LPBACK_LENGTH;
        case USART_ID_2 :
            return _U2MODE_LPBACK_LENGTH;
        case USART_ID_3 :
            return _U3MODE_LPBACK_LENGTH;
        case USART_ID_4 :
            return _U4MODE_LPBACK_LENGTH;
        case USART_ID_5 :
            return _U5MODE_LPBACK_LENGTH;
        case USART_ID_6 :
            return _U6MODE_LPBACK_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_WAKE_ON_START_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_WAKE_LENGTH;
        case USART_ID_2 :
            return _U2MODE_WAKE_LENGTH;
        case USART_ID_3 :
            return _U3MODE_WAKE_LENGTH;
        case USART_ID_4 :
            return _U4MODE_WAKE_LENGTH;
        case USART_ID_5 :
            return _U5MODE_WAKE_LENGTH;
        case USART_ID_6 :
            return _U6MODE_WAKE_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_OPERATION_MODE_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_UEN_LENGTH;
        case USART_ID_2 :
            return _U2MODE_UEN_LENGTH;
        case USART_ID_3 :
            return _U3MODE_UEN_LENGTH;
        case USART_ID_4 :
            return _U4MODE_UEN_LENGTH;
        case USART_ID_5 :
            return _U5MODE_UEN_LENGTH;
        case USART_ID_6 :
            return _U6MODE_UEN_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_HAND_SHAKE_MODE_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_RTSMD_LENGTH;
        case USART_ID_2 :
            return _U2MODE_RTSMD_LENGTH;
        case USART_ID_3 :
            return _U3MODE_RTSMD_LENGTH;
        case USART_ID_4 :
            return _U4MODE_RTSMD_LENGTH;
        case USART_ID_5 :
            return _U5MODE_RTSMD_LENGTH;
        case USART_ID_6 :
            return _U6MODE_RTSMD_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_IRDA_ENABLE_CONTROL_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_IREN_LENGTH;
        case USART_ID_2 :
            return _U2MODE_IREN_LENGTH;
        case USART_ID_3 :
            return _U3MODE_IREN_LENGTH;
        case USART_ID_4 :
            return _U4MODE_IREN_LENGTH;
        case USART_ID_5 :
            return _U5MODE_IREN_LENGTH;
        case USART_ID_6 :
            return _U6MODE_IREN_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_STOP_IN_IDLE_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_SIDL_LENGTH;
        case USART_ID_2 :
            return _U2MODE_SIDL_LENGTH;
        case USART_ID_3 :
            return _U3MODE_SIDL_LENGTH;
        case USART_ID_4 :
            return _U4MODE_SIDL_LENGTH;
        case USART_ID_5 :
            return _U5MODE_SIDL_LENGTH;
        case USART_ID_6 :
            return _U6MODE_SIDL_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_ENABLE_CONTROL_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1MODE_ON_LENGTH;
        case USART_ID_2 :
            return _U2MODE_ON_LENGTH;
        case USART_ID_3 :
            return _U3MODE_ON_LENGTH;
        case USART_ID_4 :
            return _U4MODE_ON_LENGTH;
        case USART_ID_5 :
            return _U5MODE_ON_LENGTH;
        case USART_ID_6 :
            return _U6MODE_ON_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_DATA_AVAILABLE_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_URXDA_LENGTH;
        case USART_ID_2 :
            return _U2STA_URXDA_LENGTH;
        case USART_ID_3 :
            return _U3STA_URXDA_LENGTH;
        case USART_ID_4 :
            return _U4STA_URXDA_LENGTH;
        case USART_ID_5 :
            return _U5STA_URXDA_LENGTH;
        case USART_ID_6 :
            return _U6STA_URXDA_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_OVERRUN_ERROR_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_OERR_LENGTH;
        case USART_ID_2 :
            return _U2STA_OERR_LENGTH;
        case USART_ID_3 :
            return _U3STA_OERR_LENGTH;
        case USART_ID_4 :
            return _U4STA_OERR_LENGTH;
        case USART_ID_5 :
            return _U5STA_OERR_LENGTH;
        case USART_ID_6 :
            return _U6STA_OERR_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_FRAMING_ERROR_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_FERR_LENGTH;
        case USART_ID_2 :
            return _U2STA_FERR_LENGTH;
        case USART_ID_3 :
            return _U3STA_FERR_LENGTH;
        case USART_ID_4 :
            return _U4STA_FERR_LENGTH;
        case USART_ID_5 :
            return _U5STA_FERR_LENGTH;
        case USART_ID_6 :
            return _U6STA_FERR_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_PARITY_ERROR_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_PERR_LENGTH;
        case USART_ID_2 :
            return _U2STA_PERR_LENGTH;
        case USART_ID_3 :
            return _U3STA_PERR_LENGTH;
        case USART_ID_4 :
            return _U4STA_PERR_LENGTH;
        case USART_ID_5 :
            return _U5STA_PERR_LENGTH;
        case USART_ID_6 :
            return _U6STA_PERR_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_IDLE_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_RIDLE_LENGTH;
        case USART_ID_2 :
            return _U2STA_RIDLE_LENGTH;
        case USART_ID_3 :
            return _U3STA_RIDLE_LENGTH;
        case USART_ID_4 :
            return _U4STA_RIDLE_LENGTH;
        case USART_ID_5 :
            return _U5STA_RIDLE_LENGTH;
        case USART_ID_6 :
            return _U6STA_RIDLE_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_ADDRESS_DETECT_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_ADDEN_LENGTH;
        case USART_ID_2 :
            return _U2STA_ADDEN_LENGTH;
        case USART_ID_3 :
            return _U3STA_ADDEN_LENGTH;
        case USART_ID_4 :
            return _U4STA_ADDEN_LENGTH;
        case USART_ID_5 :
            return _U5STA_ADDEN_LENGTH;
        case USART_ID_6 :
            return _U6STA_ADDEN_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_FIFO_LEVEL_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_URXISEL_LENGTH;
        case USART_ID_2 :
            return _U2STA_URXISEL_LENGTH;
        case USART_ID_3 :
            return _U3STA_URXISEL_LENGTH;
        case USART_ID_4 :
            return _U4STA_URXISEL_LENGTH;
        case USART_ID_5 :
            return _U5STA_URXISEL_LENGTH;
        case USART_ID_6 :
            return _U6STA_URXISEL_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_EMPTY_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_TRMT_LENGTH;
        case USART_ID_2 :
            return _U2STA_TRMT_LENGTH;
        case USART_ID_3 :
            return _U3STA_TRMT_LENGTH;
        case USART_ID_4 :
            return _U4STA_TRMT_LENGTH;
        case USART_ID_5 :
            return _U5STA_TRMT_LENGTH;
        case USART_ID_6 :
            return _U6STA_TRMT_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_BUFFER_FULL_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_UTXBF_LENGTH;
        case USART_ID_2 :
            return _U2STA_UTXBF_LENGTH;
        case USART_ID_3 :
            return _U3STA_UTXBF_LENGTH;
        case USART_ID_4 :
            return _U4STA_UTXBF_LENGTH;
        case USART_ID_5 :
            return _U5STA_UTXBF_LENGTH;
        case USART_ID_6 :
            return _U6STA_UTXBF_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_ENABLE_CONTROL_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_UTXEN_LENGTH;
        case USART_ID_2 :
            return _U2STA_UTXEN_LENGTH;
        case USART_ID_3 :
            return _U3STA_UTXEN_LENGTH;
        case USART_ID_4 :
            return _U4STA_UTXEN_LENGTH;
        case USART_ID_5 :
            return _U5STA_UTXEN_LENGTH;
        case USART_ID_6 :
            return _U6STA_UTXEN_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_BREAK_SEND_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_UTXBRK_LENGTH;
        case USART_ID_2 :
            return _U2STA_UTXBRK_LENGTH;
        case USART_ID_3 :
            return _U3STA_UTXBRK_LENGTH;
        case USART_ID_4 :
            return _U4STA_UTXBRK_LENGTH;
        case USART_ID_5 :
            return _U5STA_UTXBRK_LENGTH;
        case USART_ID_6 :
            return _U6STA_UTXBRK_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_ENABLE_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_URXEN_LENGTH;
        case USART_ID_2 :
            return _U2STA_URXEN_LENGTH;
        case USART_ID_3 :
            return _U3STA_URXEN_LENGTH;
        case USART_ID_4 :
            return _U4STA_URXEN_LENGTH;
        case USART_ID_5 :
            return _U5STA_URXEN_LENGTH;
        case USART_ID_6 :
            return _U6STA_URXEN_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_IDLE_IS_LOW32_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_UTXINV_LENGTH;
        case USART_ID_2 :
            return _U2STA_UTXINV_LENGTH;
        case USART_ID_3 :
            return _U3STA_UTXINV_LENGTH;
        case USART_ID_4 :
            return _U4STA_UTXINV_LENGTH;
        case USART_ID_5 :
            return _U5STA_UTXINV_LENGTH;
        case USART_ID_6 :
            return _U6STA_UTXINV_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_INTR_MODE_SELECT_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_UTXISEL_LENGTH;
        case USART_ID_2 :
            return _U2STA_UTXISEL_LENGTH;
        case USART_ID_3 :
            return _U3STA_UTXISEL_LENGTH;
        case USART_ID_4 :
            return _U4STA_UTXISEL_LENGTH;
        case USART_ID_5 :
            return _U5STA_UTXISEL_LENGTH;
        case USART_ID_6 :
            return _U6STA_UTXISEL_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_ADDRESS_AUTO_DETECT_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_ADDR_LENGTH;
        case USART_ID_2 :
            return _U2STA_ADDR_LENGTH;
        case USART_ID_3 :
            return _U3STA_ADDR_LENGTH;
        case USART_ID_4 :
            return _U4STA_ADDR_LENGTH;
        case USART_ID_5 :
            return _U5STA_ADDR_LENGTH;
        case USART_ID_6 :
            return _U6STA_ADDR_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_ADDRESS_AUTO_DETECT_ENABLE_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return _U1STA_ADM_EN_LENGTH;
        case USART_ID_2 :
            return _U2STA_ADM_EN_LENGTH;
        case USART_ID_3 :
            return _U3STA_ADM_EN_LENGTH;
        case USART_ID_4 :
            return _U4STA_ADM_EN_LENGTH;
        case USART_ID_5 :
            return _U5STA_ADM_EN_LENGTH;
        case USART_ID_6 :
            return _U6STA_ADM_EN_LENGTH;
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_SEND_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_2 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_3 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_4 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_5 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_6 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_TRANSMITTER_DATA_ONLY_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_2 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_3 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_4 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_5 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_6 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_RECEIVER_DATA_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_2 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_3 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_4 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_5 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_6 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_BAUD_RATE_HIGH_16_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_2 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_3 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_4 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_5 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_6 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _USART_BAUD_RATE_LEN(USART_MODULE_ID i)
{
    switch (i) {
        case USART_ID_1 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_2 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_3 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_4 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_5 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_ID_6 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case USART_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/usart_EnableControl_Default.h"
#include "../templates/usart_HandShakeMode_Default.h"
#include "../templates/usart_IrDAControl_Default.h"
#include "../templates/usart_LineControlMode_RXandTXCombined.h"
#include "../templates/usart_Loopback_Default.h"
#include "../templates/usart_OperationMode_Default.h"
#include "../templates/usart_StopInIdle_Default.h"
#include "../templates/usart_RunInOverflow_Unsupported.h"
#include "../templates/usart_BRGClockSourceSelect_Unsupported.h"
#include "../templates/usart_UsartModuleStatus_Unsupported.h"
#include "../templates/usart_RunInSleepMode_Unsupported.h"
#include "../templates/usart_WakeOnStart_Default.h"
#include "../templates/usart_BaudRate_In16BitRegister.h"
#include "../templates/usart_BaudRateAutoDetect_Default.h"
#include "../templates/usart_BaudRateHigh_In16BitRegister.h"
#include "../templates/usart_Receiver_Default.h"
#include "../templates/usart_Receiver9Bits_Default.h"
#include "../templates/usart_ReceiverAddressAutoDetect_Default.h"
#include "../templates/usart_ReceiverAddressDetect_Default.h"
#include "../templates/usart_ReceiverAddress_Default.h"
#include "../templates/usart_ReceiverAddressMask_Unsupported.h"
#include "../templates/usart_ReceiverDataAvailable_Default.h"
#include "../templates/usart_ReceiverEnableControl_Default.h"
#include "../templates/usart_ReceiverIdle_Default.h"
#include "../templates/usart_ReceiverFramingError_Default.h"
#include "../templates/usart_ReceiverInterruptMode_Default.h"
#include "../templates/usart_ReceiverPolarityInvert_Default.h"
#include "../templates/usart_ReceiverParityError_Default.h"
#include "../templates/usart_ReceiverOverrunError_Default.h"
#include "../templates/usart_Transmitter_Default.h"
#include "../templates/usart_Transmitter9Bits_InDataOnly.h"
#include "../templates/usart_TransmitterBreak_Default.h"
#include "../templates/usart_TransmitterBufferFull_Default.h"
#include "../templates/usart_TransmitterEmpty_Default.h"
#include "../templates/usart_TransmitterEnableControl_Default.h"
#include "../templates/usart_TransmitterInterruptMode_Default.h"
#include "../templates/usart_TransmitterIdleIsLow_pic32.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API bool PLIB_USART_ExistsEnable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsEnable_Default(index);
        case USART_ID_2 :
            return USART_ExistsEnable_Default(index);
        case USART_ID_3 :
            return USART_ExistsEnable_Default(index);
        case USART_ID_4 :
            return USART_ExistsEnable_Default(index);
        case USART_ID_5 :
            return USART_ExistsEnable_Default(index);
        case USART_ID_6 :
            return USART_ExistsEnable_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_Disable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_Disable_Default(index);
            break;
        case USART_ID_2 :
            USART_Disable_Default(index);
            break;
        case USART_ID_3 :
            USART_Disable_Default(index);
            break;
        case USART_ID_4 :
            USART_Disable_Default(index);
            break;
        case USART_ID_5 :
            USART_Disable_Default(index);
            break;
        case USART_ID_6 :
            USART_Disable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_USART_Enable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_Enable_Default(index);
            break;
        case USART_ID_2 :
            USART_Enable_Default(index);
            break;
        case USART_ID_3 :
            USART_Enable_Default(index);
            break;
        case USART_ID_4 :
            USART_Enable_Default(index);
            break;
        case USART_ID_5 :
            USART_Enable_Default(index);
            break;
        case USART_ID_6 :
            USART_Enable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_USART_BaudSetAndEnable(USART_MODULE_ID index, uint32_t systemClock, uint32_t baud)
{
    switch (index) {
        case USART_ID_1 :
            USART_BaudSetAndEnable_Default(index, systemClock, baud);
            break;
        case USART_ID_2 :
            USART_BaudSetAndEnable_Default(index, systemClock, baud);
            break;
        case USART_ID_3 :
            USART_BaudSetAndEnable_Default(index, systemClock, baud);
            break;
        case USART_ID_4 :
            USART_BaudSetAndEnable_Default(index, systemClock, baud);
            break;
        case USART_ID_5 :
            USART_BaudSetAndEnable_Default(index, systemClock, baud);
            break;
        case USART_ID_6 :
            USART_BaudSetAndEnable_Default(index, systemClock, baud);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsHandshakeMode(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsHandshakeMode_Default(index);
        case USART_ID_2 :
            return USART_ExistsHandshakeMode_Default(index);
        case USART_ID_3 :
            return USART_ExistsHandshakeMode_Default(index);
        case USART_ID_4 :
            return USART_ExistsHandshakeMode_Default(index);
        case USART_ID_5 :
            return USART_ExistsHandshakeMode_Default(index);
        case USART_ID_6 :
            return USART_ExistsHandshakeMode_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_HandshakeModeSelect(USART_MODULE_ID index, USART_HANDSHAKE_MODE handshakeConfig)
{
    switch (index) {
        case USART_ID_1 :
            USART_HandshakeModeSelect_Default(index, handshakeConfig);
            break;
        case USART_ID_2 :
            USART_HandshakeModeSelect_Default(index, handshakeConfig);
            break;
        case USART_ID_3 :
            USART_HandshakeModeSelect_Default(index, handshakeConfig);
            break;
        case USART_ID_4 :
            USART_HandshakeModeSelect_Default(index, handshakeConfig);
            break;
        case USART_ID_5 :
            USART_HandshakeModeSelect_Default(index, handshakeConfig);
            break;
        case USART_ID_6 :
            USART_HandshakeModeSelect_Default(index, handshakeConfig);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsIrDA(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsIrDA_Default(index);
        case USART_ID_2 :
            return USART_ExistsIrDA_Default(index);
        case USART_ID_3 :
            return USART_ExistsIrDA_Default(index);
        case USART_ID_4 :
            return USART_ExistsIrDA_Default(index);
        case USART_ID_5 :
            return USART_ExistsIrDA_Default(index);
        case USART_ID_6 :
            return USART_ExistsIrDA_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_IrDADisable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_IrDADisable_Default(index);
            break;
        case USART_ID_2 :
            USART_IrDADisable_Default(index);
            break;
        case USART_ID_3 :
            USART_IrDADisable_Default(index);
            break;
        case USART_ID_4 :
            USART_IrDADisable_Default(index);
            break;
        case USART_ID_5 :
            USART_IrDADisable_Default(index);
            break;
        case USART_ID_6 :
            USART_IrDADisable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_USART_IrDAEnable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_IrDAEnable_Default(index);
            break;
        case USART_ID_2 :
            USART_IrDAEnable_Default(index);
            break;
        case USART_ID_3 :
            USART_IrDAEnable_Default(index);
            break;
        case USART_ID_4 :
            USART_IrDAEnable_Default(index);
            break;
        case USART_ID_5 :
            USART_IrDAEnable_Default(index);
            break;
        case USART_ID_6 :
            USART_IrDAEnable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsLineControlMode(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsLineControlMode_RXandTXCombined(index);
        case USART_ID_2 :
            return USART_ExistsLineControlMode_RXandTXCombined(index);
        case USART_ID_3 :
            return USART_ExistsLineControlMode_RXandTXCombined(index);
        case USART_ID_4 :
            return USART_ExistsLineControlMode_RXandTXCombined(index);
        case USART_ID_5 :
            return USART_ExistsLineControlMode_RXandTXCombined(index);
        case USART_ID_6 :
            return USART_ExistsLineControlMode_RXandTXCombined(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_LineControlModeSelect(USART_MODULE_ID index, USART_LINECONTROL_MODE dataFlowConfig)
{
    switch (index) {
        case USART_ID_1 :
            USART_LineControlModeSelect_RXandTXCombined(index, dataFlowConfig);
            break;
        case USART_ID_2 :
            USART_LineControlModeSelect_RXandTXCombined(index, dataFlowConfig);
            break;
        case USART_ID_3 :
            USART_LineControlModeSelect_RXandTXCombined(index, dataFlowConfig);
            break;
        case USART_ID_4 :
            USART_LineControlModeSelect_RXandTXCombined(index, dataFlowConfig);
            break;
        case USART_ID_5 :
            USART_LineControlModeSelect_RXandTXCombined(index, dataFlowConfig);
            break;
        case USART_ID_6 :
            USART_LineControlModeSelect_RXandTXCombined(index, dataFlowConfig);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsLoopback(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsLoopback_Default(index);
        case USART_ID_2 :
            return USART_ExistsLoopback_Default(index);
        case USART_ID_3 :
            return USART_ExistsLoopback_Default(index);
        case USART_ID_4 :
            return USART_ExistsLoopback_Default(index);
        case USART_ID_5 :
            return USART_ExistsLoopback_Default(index);
        case USART_ID_6 :
            return USART_ExistsLoopback_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_LoopbackEnable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_LoopbackEnable_Default(index);
            break;
        case USART_ID_2 :
            USART_LoopbackEnable_Default(index);
            break;
        case USART_ID_3 :
            USART_LoopbackEnable_Default(index);
            break;
        case USART_ID_4 :
            USART_LoopbackEnable_Default(index);
            break;
        case USART_ID_5 :
            USART_LoopbackEnable_Default(index);
            break;
        case USART_ID_6 :
            USART_LoopbackEnable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_USART_LoopbackDisable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_LoopbackDisable_Default(index);
            break;
        case USART_ID_2 :
            USART_LoopbackDisable_Default(index);
            break;
        case USART_ID_3 :
            USART_LoopbackDisable_Default(index);
            break;
        case USART_ID_4 :
            USART_LoopbackDisable_Default(index);
            break;
        case USART_ID_5 :
            USART_LoopbackDisable_Default(index);
            break;
        case USART_ID_6 :
            USART_LoopbackDisable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsOperationMode(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsOperationMode_Default(index);
        case USART_ID_2 :
            return USART_ExistsOperationMode_Default(index);
        case USART_ID_3 :
            return USART_ExistsOperationMode_Default(index);
        case USART_ID_4 :
            return USART_ExistsOperationMode_Default(index);
        case USART_ID_5 :
            return USART_ExistsOperationMode_Default(index);
        case USART_ID_6 :
            return USART_ExistsOperationMode_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_OperationModeSelect(USART_MODULE_ID index, USART_OPERATION_MODE operationmode)
{
    switch (index) {
        case USART_ID_1 :
            USART_OperationModeSelect_Default(index, operationmode);
            break;
        case USART_ID_2 :
            USART_OperationModeSelect_Default(index, operationmode);
            break;
        case USART_ID_3 :
            USART_OperationModeSelect_Default(index, operationmode);
            break;
        case USART_ID_4 :
            USART_OperationModeSelect_Default(index, operationmode);
            break;
        case USART_ID_5 :
            USART_OperationModeSelect_Default(index, operationmode);
            break;
        case USART_ID_6 :
            USART_OperationModeSelect_Default(index, operationmode);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsStopInIdle(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsStopInIdle_Default(index);
        case USART_ID_2 :
            return USART_ExistsStopInIdle_Default(index);
        case USART_ID_3 :
            return USART_ExistsStopInIdle_Default(index);
        case USART_ID_4 :
            return USART_ExistsStopInIdle_Default(index);
        case USART_ID_5 :
            return USART_ExistsStopInIdle_Default(index);
        case USART_ID_6 :
            return USART_ExistsStopInIdle_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_StopInIdleEnable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_StopInIdleEnable_Default(index);
            break;
        case USART_ID_2 :
            USART_StopInIdleEnable_Default(index);
            break;
        case USART_ID_3 :
            USART_StopInIdleEnable_Default(index);
            break;
        case USART_ID_4 :
            USART_StopInIdleEnable_Default(index);
            break;
        case USART_ID_5 :
            USART_StopInIdleEnable_Default(index);
            break;
        case USART_ID_6 :
            USART_StopInIdleEnable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_USART_StopInIdleDisable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_StopInIdleDisable_Default(index);
            break;
        case USART_ID_2 :
            USART_StopInIdleDisable_Default(index);
            break;
        case USART_ID_3 :
            USART_StopInIdleDisable_Default(index);
            break;
        case USART_ID_4 :
            USART_StopInIdleDisable_Default(index);
            break;
        case USART_ID_5 :
            USART_StopInIdleDisable_Default(index);
            break;
        case USART_ID_6 :
            USART_StopInIdleDisable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsRunInOverflow(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsRunInOverflow_Unsupported(index);
        case USART_ID_2 :
            return USART_ExistsRunInOverflow_Unsupported(index);
        case USART_ID_3 :
            return USART_ExistsRunInOverflow_Unsupported(index);
        case USART_ID_4 :
            return USART_ExistsRunInOverflow_Unsupported(index);
        case USART_ID_5 :
            return USART_ExistsRunInOverflow_Unsupported(index);
        case USART_ID_6 :
            return USART_ExistsRunInOverflow_Unsupported(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USART_RunInOverflowEnable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_RunInOverflowEnable_Unsupported(index);
            break;
        case USART_ID_2 :
            USART_RunInOverflowEnable_Unsupported(index);
            break;
        case USART_ID_3 :
            USART_RunInOverflowEnable_Unsupported(index);
            break;
        case USART_ID_4 :
            USART_RunInOverflowEnable_Unsupported(index);
            break;
        case USART_ID_5 :
            USART_RunInOverflowEnable_Unsupported(index);
            break;
        case USART_ID_6 :
            USART_RunInOverflowEnable_Unsupported(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USART_RunInOverflowDisable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_RunInOverflowDisable_Unsupported(index);
            break;
        case USART_ID_2 :
            USART_RunInOverflowDisable_Unsupported(index);
            break;
        case USART_ID_3 :
            USART_RunInOverflowDisable_Unsupported(index);
            break;
        case USART_ID_4 :
            USART_RunInOverflowDisable_Unsupported(index);
            break;
        case USART_ID_5 :
            USART_RunInOverflowDisable_Unsupported(index);
            break;
        case USART_ID_6 :
            USART_RunInOverflowDisable_Unsupported(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USART_RunInOverflowIsEnabled(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_RunInOverflowIsEnabled_Unsupported(index);
        case USART_ID_2 :
            return USART_RunInOverflowIsEnabled_Unsupported(index);
        case USART_ID_3 :
            return USART_RunInOverflowIsEnabled_Unsupported(index);
        case USART_ID_4 :
            return USART_RunInOverflowIsEnabled_Unsupported(index);
        case USART_ID_5 :
            return USART_RunInOverflowIsEnabled_Unsupported(index);
        case USART_ID_6 :
            return USART_RunInOverflowIsEnabled_Unsupported(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsBRGClockSourceSelect(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsBRGClockSourceSelect_Unsupported(index);
        case USART_ID_2 :
            return USART_ExistsBRGClockSourceSelect_Unsupported(index);
        case USART_ID_3 :
            return USART_ExistsBRGClockSourceSelect_Unsupported(index);
        case USART_ID_4 :
            return USART_ExistsBRGClockSourceSelect_Unsupported(index);
        case USART_ID_5 :
            return USART_ExistsBRGClockSourceSelect_Unsupported(index);
        case USART_ID_6 :
            return USART_ExistsBRGClockSourceSelect_Unsupported(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USART_BRGClockSourceSelect(USART_MODULE_ID index, USART_BRG_CLOCK_SOURCE brgClockSource)
{
    switch (index) {
        case USART_ID_1 :
            USART_BRGClockSourceSelect_Unsupported(index, brgClockSource);
            break;
        case USART_ID_2 :
            USART_BRGClockSourceSelect_Unsupported(index, brgClockSource);
            break;
        case USART_ID_3 :
            USART_BRGClockSourceSelect_Unsupported(index, brgClockSource);
            break;
        case USART_ID_4 :
            USART_BRGClockSourceSelect_Unsupported(index, brgClockSource);
            break;
        case USART_ID_5 :
            USART_BRGClockSourceSelect_Unsupported(index, brgClockSource);
            break;
        case USART_ID_6 :
            USART_BRGClockSourceSelect_Unsupported(index, brgClockSource);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API USART_BRG_CLOCK_SOURCE _PLIB_UNSUPPORTED PLIB_USART_BRGClockSourceGet(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_BRGClockSourceGet_Unsupported(index);
        case USART_ID_2 :
            return USART_BRGClockSourceGet_Unsupported(index);
        case USART_ID_3 :
            return USART_BRGClockSourceGet_Unsupported(index);
        case USART_ID_4 :
            return USART_BRGClockSourceGet_Unsupported(index);
        case USART_ID_5 :
            return USART_BRGClockSourceGet_Unsupported(index);
        case USART_ID_6 :
            return USART_BRGClockSourceGet_Unsupported(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (USART_BRG_CLOCK_SOURCE)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsModuleBusyStatus(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsModuleBusyStatus_Unsupported(index);
        case USART_ID_2 :
            return USART_ExistsModuleBusyStatus_Unsupported(index);
        case USART_ID_3 :
            return USART_ExistsModuleBusyStatus_Unsupported(index);
        case USART_ID_4 :
            return USART_ExistsModuleBusyStatus_Unsupported(index);
        case USART_ID_5 :
            return USART_ExistsModuleBusyStatus_Unsupported(index);
        case USART_ID_6 :
            return USART_ExistsModuleBusyStatus_Unsupported(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USART_ModuleIsBusy(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ModuleIsBusy_Unsupported(index);
        case USART_ID_2 :
            return USART_ModuleIsBusy_Unsupported(index);
        case USART_ID_3 :
            return USART_ModuleIsBusy_Unsupported(index);
        case USART_ID_4 :
            return USART_ModuleIsBusy_Unsupported(index);
        case USART_ID_5 :
            return USART_ModuleIsBusy_Unsupported(index);
        case USART_ID_6 :
            return USART_ModuleIsBusy_Unsupported(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsRunInSleepMode(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsRunInSleepMode_Unsupported(index);
        case USART_ID_2 :
            return USART_ExistsRunInSleepMode_Unsupported(index);
        case USART_ID_3 :
            return USART_ExistsRunInSleepMode_Unsupported(index);
        case USART_ID_4 :
            return USART_ExistsRunInSleepMode_Unsupported(index);
        case USART_ID_5 :
            return USART_ExistsRunInSleepMode_Unsupported(index);
        case USART_ID_6 :
            return USART_ExistsRunInSleepMode_Unsupported(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USART_RunInSleepModeEnable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_RunInSleepModeEnable_Unsupported(index);
            break;
        case USART_ID_2 :
            USART_RunInSleepModeEnable_Unsupported(index);
            break;
        case USART_ID_3 :
            USART_RunInSleepModeEnable_Unsupported(index);
            break;
        case USART_ID_4 :
            USART_RunInSleepModeEnable_Unsupported(index);
            break;
        case USART_ID_5 :
            USART_RunInSleepModeEnable_Unsupported(index);
            break;
        case USART_ID_6 :
            USART_RunInSleepModeEnable_Unsupported(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USART_RunInSleepModeDisable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_RunInSleepModeDisable_Unsupported(index);
            break;
        case USART_ID_2 :
            USART_RunInSleepModeDisable_Unsupported(index);
            break;
        case USART_ID_3 :
            USART_RunInSleepModeDisable_Unsupported(index);
            break;
        case USART_ID_4 :
            USART_RunInSleepModeDisable_Unsupported(index);
            break;
        case USART_ID_5 :
            USART_RunInSleepModeDisable_Unsupported(index);
            break;
        case USART_ID_6 :
            USART_RunInSleepModeDisable_Unsupported(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_USART_RunInSleepModeIsEnabled(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_RunInSleepModeIsEnabled_Unsupported(index);
        case USART_ID_2 :
            return USART_RunInSleepModeIsEnabled_Unsupported(index);
        case USART_ID_3 :
            return USART_RunInSleepModeIsEnabled_Unsupported(index);
        case USART_ID_4 :
            return USART_RunInSleepModeIsEnabled_Unsupported(index);
        case USART_ID_5 :
            return USART_RunInSleepModeIsEnabled_Unsupported(index);
        case USART_ID_6 :
            return USART_RunInSleepModeIsEnabled_Unsupported(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsWakeOnStart(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsWakeOnStart_Default(index);
        case USART_ID_2 :
            return USART_ExistsWakeOnStart_Default(index);
        case USART_ID_3 :
            return USART_ExistsWakeOnStart_Default(index);
        case USART_ID_4 :
            return USART_ExistsWakeOnStart_Default(index);
        case USART_ID_5 :
            return USART_ExistsWakeOnStart_Default(index);
        case USART_ID_6 :
            return USART_ExistsWakeOnStart_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_WakeOnStartEnable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_WakeOnStartEnable_Default(index);
            break;
        case USART_ID_2 :
            USART_WakeOnStartEnable_Default(index);
            break;
        case USART_ID_3 :
            USART_WakeOnStartEnable_Default(index);
            break;
        case USART_ID_4 :
            USART_WakeOnStartEnable_Default(index);
            break;
        case USART_ID_5 :
            USART_WakeOnStartEnable_Default(index);
            break;
        case USART_ID_6 :
            USART_WakeOnStartEnable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_USART_WakeOnStartDisable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_WakeOnStartDisable_Default(index);
            break;
        case USART_ID_2 :
            USART_WakeOnStartDisable_Default(index);
            break;
        case USART_ID_3 :
            USART_WakeOnStartDisable_Default(index);
            break;
        case USART_ID_4 :
            USART_WakeOnStartDisable_Default(index);
            break;
        case USART_ID_5 :
            USART_WakeOnStartDisable_Default(index);
            break;
        case USART_ID_6 :
            USART_WakeOnStartDisable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_USART_WakeOnStartIsEnabled(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_WakeOnStartIsEnabled_Default(index);
        case USART_ID_2 :
            return USART_WakeOnStartIsEnabled_Default(index);
        case USART_ID_3 :
            return USART_WakeOnStartIsEnabled_Default(index);
        case USART_ID_4 :
            return USART_WakeOnStartIsEnabled_Default(index);
        case USART_ID_5 :
            return USART_WakeOnStartIsEnabled_Default(index);
        case USART_ID_6 :
            return USART_WakeOnStartIsEnabled_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_InitializeModeGeneral(USART_MODULE_ID index, bool autobaud, bool loopBackMode, bool wakeFromSleep, bool irdaMode, bool stopInIdle)
{
    switch (index) {
        case USART_ID_1 :
            USART_InitializeModeGeneral_Default(index, autobaud, loopBackMode, wakeFromSleep, irdaMode, stopInIdle);
            break;
        case USART_ID_2 :
            USART_InitializeModeGeneral_Default(index, autobaud, loopBackMode, wakeFromSleep, irdaMode, stopInIdle);
            break;
        case USART_ID_3 :
            USART_InitializeModeGeneral_Default(index, autobaud, loopBackMode, wakeFromSleep, irdaMode, stopInIdle);
            break;
        case USART_ID_4 :
            USART_InitializeModeGeneral_Default(index, autobaud, loopBackMode, wakeFromSleep, irdaMode, stopInIdle);
            break;
        case USART_ID_5 :
            USART_InitializeModeGeneral_Default(index, autobaud, loopBackMode, wakeFromSleep, irdaMode, stopInIdle);
            break;
        case USART_ID_6 :
            USART_InitializeModeGeneral_Default(index, autobaud, loopBackMode, wakeFromSleep, irdaMode, stopInIdle);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API USART_ERROR PLIB_USART_ErrorsGet(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ErrorsGet_Default(index);
        case USART_ID_2 :
            return USART_ErrorsGet_Default(index);
        case USART_ID_3 :
            return USART_ErrorsGet_Default(index);
        case USART_ID_4 :
            return USART_ErrorsGet_Default(index);
        case USART_ID_5 :
            return USART_ErrorsGet_Default(index);
        case USART_ID_6 :
            return USART_ErrorsGet_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (USART_ERROR)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsBaudRate(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsBaudRate_In16BitRegister(index);
        case USART_ID_2 :
            return USART_ExistsBaudRate_In16BitRegister(index);
        case USART_ID_3 :
            return USART_ExistsBaudRate_In16BitRegister(index);
        case USART_ID_4 :
            return USART_ExistsBaudRate_In16BitRegister(index);
        case USART_ID_5 :
            return USART_ExistsBaudRate_In16BitRegister(index);
        case USART_ID_6 :
            return USART_ExistsBaudRate_In16BitRegister(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_BaudRateSet(USART_MODULE_ID index, uint32_t clockFrequency, uint32_t baudRate)
{
    switch (index) {
        case USART_ID_1 :
            USART_BaudRateSet_In16BitRegister(index, clockFrequency, baudRate);
            break;
        case USART_ID_2 :
            USART_BaudRateSet_In16BitRegister(index, clockFrequency, baudRate);
            break;
        case USART_ID_3 :
            USART_BaudRateSet_In16BitRegister(index, clockFrequency, baudRate);
            break;
        case USART_ID_4 :
            USART_BaudRateSet_In16BitRegister(index, clockFrequency, baudRate);
            break;
        case USART_ID_5 :
            USART_BaudRateSet_In16BitRegister(index, clockFrequency, baudRate);
            break;
        case USART_ID_6 :
            USART_BaudRateSet_In16BitRegister(index, clockFrequency, baudRate);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint32_t PLIB_USART_BaudRateGet(USART_MODULE_ID index, int32_t clockFrequency)
{
    switch (index) {
        case USART_ID_1 :
            return USART_BaudRateGet_In16BitRegister(index, clockFrequency);
        case USART_ID_2 :
            return USART_BaudRateGet_In16BitRegister(index, clockFrequency);
        case USART_ID_3 :
            return USART_BaudRateGet_In16BitRegister(index, clockFrequency);
        case USART_ID_4 :
            return USART_BaudRateGet_In16BitRegister(index, clockFrequency);
        case USART_ID_5 :
            return USART_BaudRateGet_In16BitRegister(index, clockFrequency);
        case USART_ID_6 :
            return USART_BaudRateGet_In16BitRegister(index, clockFrequency);
        case USART_NUMBER_OF_MODULES :
        default :
            return (uint32_t)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsBaudRateAutoDetect(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsBaudRateAutoDetect_Default(index);
        case USART_ID_2 :
            return USART_ExistsBaudRateAutoDetect_Default(index);
        case USART_ID_3 :
            return USART_ExistsBaudRateAutoDetect_Default(index);
        case USART_ID_4 :
            return USART_ExistsBaudRateAutoDetect_Default(index);
        case USART_ID_5 :
            return USART_ExistsBaudRateAutoDetect_Default(index);
        case USART_ID_6 :
            return USART_ExistsBaudRateAutoDetect_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_BaudRateAutoDetectEnable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_BaudRateAutoDetectEnable_Default(index);
            break;
        case USART_ID_2 :
            USART_BaudRateAutoDetectEnable_Default(index);
            break;
        case USART_ID_3 :
            USART_BaudRateAutoDetectEnable_Default(index);
            break;
        case USART_ID_4 :
            USART_BaudRateAutoDetectEnable_Default(index);
            break;
        case USART_ID_5 :
            USART_BaudRateAutoDetectEnable_Default(index);
            break;
        case USART_ID_6 :
            USART_BaudRateAutoDetectEnable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_USART_BaudRateAutoDetectIsComplete(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_BaudRateAutoDetectIsComplete_Default(index);
        case USART_ID_2 :
            return USART_BaudRateAutoDetectIsComplete_Default(index);
        case USART_ID_3 :
            return USART_BaudRateAutoDetectIsComplete_Default(index);
        case USART_ID_4 :
            return USART_BaudRateAutoDetectIsComplete_Default(index);
        case USART_ID_5 :
            return USART_BaudRateAutoDetectIsComplete_Default(index);
        case USART_ID_6 :
            return USART_BaudRateAutoDetectIsComplete_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsBaudRateHigh(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsBaudRateHigh_In16BitRegister(index);
        case USART_ID_2 :
            return USART_ExistsBaudRateHigh_In16BitRegister(index);
        case USART_ID_3 :
            return USART_ExistsBaudRateHigh_In16BitRegister(index);
        case USART_ID_4 :
            return USART_ExistsBaudRateHigh_In16BitRegister(index);
        case USART_ID_5 :
            return USART_ExistsBaudRateHigh_In16BitRegister(index);
        case USART_ID_6 :
            return USART_ExistsBaudRateHigh_In16BitRegister(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_BaudRateHighSet(USART_MODULE_ID index, uint32_t clockFrequency, uint32_t baudRate)
{
    switch (index) {
        case USART_ID_1 :
            USART_BaudRateHighSet_In16BitRegister(index, clockFrequency, baudRate);
            break;
        case USART_ID_2 :
            USART_BaudRateHighSet_In16BitRegister(index, clockFrequency, baudRate);
            break;
        case USART_ID_3 :
            USART_BaudRateHighSet_In16BitRegister(index, clockFrequency, baudRate);
            break;
        case USART_ID_4 :
            USART_BaudRateHighSet_In16BitRegister(index, clockFrequency, baudRate);
            break;
        case USART_ID_5 :
            USART_BaudRateHighSet_In16BitRegister(index, clockFrequency, baudRate);
            break;
        case USART_ID_6 :
            USART_BaudRateHighSet_In16BitRegister(index, clockFrequency, baudRate);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_USART_BaudRateHighDisable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_BaudRateHighDisable_In16BitRegister(index);
            break;
        case USART_ID_2 :
            USART_BaudRateHighDisable_In16BitRegister(index);
            break;
        case USART_ID_3 :
            USART_BaudRateHighDisable_In16BitRegister(index);
            break;
        case USART_ID_4 :
            USART_BaudRateHighDisable_In16BitRegister(index);
            break;
        case USART_ID_5 :
            USART_BaudRateHighDisable_In16BitRegister(index);
            break;
        case USART_ID_6 :
            USART_BaudRateHighDisable_In16BitRegister(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_USART_BaudRateHighEnable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_BaudRateHighEnable_In16BitRegister(index);
            break;
        case USART_ID_2 :
            USART_BaudRateHighEnable_In16BitRegister(index);
            break;
        case USART_ID_3 :
            USART_BaudRateHighEnable_In16BitRegister(index);
            break;
        case USART_ID_4 :
            USART_BaudRateHighEnable_In16BitRegister(index);
            break;
        case USART_ID_5 :
            USART_BaudRateHighEnable_In16BitRegister(index);
            break;
        case USART_ID_6 :
            USART_BaudRateHighEnable_In16BitRegister(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsReceiver(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsReceiver_Default(index);
        case USART_ID_2 :
            return USART_ExistsReceiver_Default(index);
        case USART_ID_3 :
            return USART_ExistsReceiver_Default(index);
        case USART_ID_4 :
            return USART_ExistsReceiver_Default(index);
        case USART_ID_5 :
            return USART_ExistsReceiver_Default(index);
        case USART_ID_6 :
            return USART_ExistsReceiver_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API int8_t PLIB_USART_ReceiverByteReceive(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ReceiverByteReceive_Default(index);
        case USART_ID_2 :
            return USART_ReceiverByteReceive_Default(index);
        case USART_ID_3 :
            return USART_ReceiverByteReceive_Default(index);
        case USART_ID_4 :
            return USART_ReceiverByteReceive_Default(index);
        case USART_ID_5 :
            return USART_ReceiverByteReceive_Default(index);
        case USART_ID_6 :
            return USART_ReceiverByteReceive_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (int8_t)0;
    }
}

PLIB_INLINE_API void* PLIB_USART_ReceiverAddressGet(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ReceiverAddressGet_Default(index);
        case USART_ID_2 :
            return USART_ReceiverAddressGet_Default(index);
        case USART_ID_3 :
            return USART_ReceiverAddressGet_Default(index);
        case USART_ID_4 :
            return USART_ReceiverAddressGet_Default(index);
        case USART_ID_5 :
            return USART_ReceiverAddressGet_Default(index);
        case USART_ID_6 :
            return USART_ReceiverAddressGet_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (void*)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsReceiver9Bits(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsReceiver9Bits_Default(index);
        case USART_ID_2 :
            return USART_ExistsReceiver9Bits_Default(index);
        case USART_ID_3 :
            return USART_ExistsReceiver9Bits_Default(index);
        case USART_ID_4 :
            return USART_ExistsReceiver9Bits_Default(index);
        case USART_ID_5 :
            return USART_ExistsReceiver9Bits_Default(index);
        case USART_ID_6 :
            return USART_ExistsReceiver9Bits_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API int16_t PLIB_USART_Receiver9BitsReceive(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_Receiver9BitsReceive_Default(index);
        case USART_ID_2 :
            return USART_Receiver9BitsReceive_Default(index);
        case USART_ID_3 :
            return USART_Receiver9BitsReceive_Default(index);
        case USART_ID_4 :
            return USART_Receiver9BitsReceive_Default(index);
        case USART_ID_5 :
            return USART_Receiver9BitsReceive_Default(index);
        case USART_ID_6 :
            return USART_Receiver9BitsReceive_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (int16_t)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsReceiverAddressAutoDetect(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsReceiverAddressAutoDetect_Default(index);
        case USART_ID_2 :
            return USART_ExistsReceiverAddressAutoDetect_Default(index);
        case USART_ID_3 :
            return USART_ExistsReceiverAddressAutoDetect_Default(index);
        case USART_ID_4 :
            return USART_ExistsReceiverAddressAutoDetect_Default(index);
        case USART_ID_5 :
            return USART_ExistsReceiverAddressAutoDetect_Default(index);
        case USART_ID_6 :
            return USART_ExistsReceiverAddressAutoDetect_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_ReceiverAddressAutoDetectEnable(USART_MODULE_ID index, int8_t Mask)
{
    switch (index) {
        case USART_ID_1 :
            USART_ReceiverAddressAutoDetectEnable_Default(index, Mask);
            break;
        case USART_ID_2 :
            USART_ReceiverAddressAutoDetectEnable_Default(index, Mask);
            break;
        case USART_ID_3 :
            USART_ReceiverAddressAutoDetectEnable_Default(index, Mask);
            break;
        case USART_ID_4 :
            USART_ReceiverAddressAutoDetectEnable_Default(index, Mask);
            break;
        case USART_ID_5 :
            USART_ReceiverAddressAutoDetectEnable_Default(index, Mask);
            break;
        case USART_ID_6 :
            USART_ReceiverAddressAutoDetectEnable_Default(index, Mask);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_USART_ReceiverAddressAutoDetectDisable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_ReceiverAddressAutoDetectDisable_Default(index);
            break;
        case USART_ID_2 :
            USART_ReceiverAddressAutoDetectDisable_Default(index);
            break;
        case USART_ID_3 :
            USART_ReceiverAddressAutoDetectDisable_Default(index);
            break;
        case USART_ID_4 :
            USART_ReceiverAddressAutoDetectDisable_Default(index);
            break;
        case USART_ID_5 :
            USART_ReceiverAddressAutoDetectDisable_Default(index);
            break;
        case USART_ID_6 :
            USART_ReceiverAddressAutoDetectDisable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsReceiverAddressDetect(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsReceiverAddressDetect_Default(index);
        case USART_ID_2 :
            return USART_ExistsReceiverAddressDetect_Default(index);
        case USART_ID_3 :
            return USART_ExistsReceiverAddressDetect_Default(index);
        case USART_ID_4 :
            return USART_ExistsReceiverAddressDetect_Default(index);
        case USART_ID_5 :
            return USART_ExistsReceiverAddressDetect_Default(index);
        case USART_ID_6 :
            return USART_ExistsReceiverAddressDetect_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_ReceiverAddressDetectEnable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_ReceiverAddressDetectEnable_Default(index);
            break;
        case USART_ID_2 :
            USART_ReceiverAddressDetectEnable_Default(index);
            break;
        case USART_ID_3 :
            USART_ReceiverAddressDetectEnable_Default(index);
            break;
        case USART_ID_4 :
            USART_ReceiverAddressDetectEnable_Default(index);
            break;
        case USART_ID_5 :
            USART_ReceiverAddressDetectEnable_Default(index);
            break;
        case USART_ID_6 :
            USART_ReceiverAddressDetectEnable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_USART_ReceiverAddressDetectDisable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_ReceiverAddressDetectDisable_Default(index);
            break;
        case USART_ID_2 :
            USART_ReceiverAddressDetectDisable_Default(index);
            break;
        case USART_ID_3 :
            USART_ReceiverAddressDetectDisable_Default(index);
            break;
        case USART_ID_4 :
            USART_ReceiverAddressDetectDisable_Default(index);
            break;
        case USART_ID_5 :
            USART_ReceiverAddressDetectDisable_Default(index);
            break;
        case USART_ID_6 :
            USART_ReceiverAddressDetectDisable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_USART_ReceiverAddressIsReceived(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ReceiverAddressIsReceived_Default(index);
        case USART_ID_2 :
            return USART_ReceiverAddressIsReceived_Default(index);
        case USART_ID_3 :
            return USART_ReceiverAddressIsReceived_Default(index);
        case USART_ID_4 :
            return USART_ReceiverAddressIsReceived_Default(index);
        case USART_ID_5 :
            return USART_ReceiverAddressIsReceived_Default(index);
        case USART_ID_6 :
            return USART_ReceiverAddressIsReceived_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsReceiverAddress(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsReceiverAddress_Default(index);
        case USART_ID_2 :
            return USART_ExistsReceiverAddress_Default(index);
        case USART_ID_3 :
            return USART_ExistsReceiverAddress_Default(index);
        case USART_ID_4 :
            return USART_ExistsReceiverAddress_Default(index);
        case USART_ID_5 :
            return USART_ExistsReceiverAddress_Default(index);
        case USART_ID_6 :
            return USART_ExistsReceiverAddress_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_AddressSet(USART_MODULE_ID index, uint8_t address)
{
    switch (index) {
        case USART_ID_1 :
            USART_AddressSet_Default(index, address);
            break;
        case USART_ID_2 :
            USART_AddressSet_Default(index, address);
            break;
        case USART_ID_3 :
            USART_AddressSet_Default(index, address);
            break;
        case USART_ID_4 :
            USART_AddressSet_Default(index, address);
            break;
        case USART_ID_5 :
            USART_AddressSet_Default(index, address);
            break;
        case USART_ID_6 :
            USART_AddressSet_Default(index, address);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint8_t PLIB_USART_AddressGet(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_AddressGet_Default(index);
        case USART_ID_2 :
            return USART_AddressGet_Default(index);
        case USART_ID_3 :
            return USART_AddressGet_Default(index);
        case USART_ID_4 :
            return USART_AddressGet_Default(index);
        case USART_ID_5 :
            return USART_AddressGet_Default(index);
        case USART_ID_6 :
            return USART_AddressGet_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsReceiverAddressMask(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsReceiverAddressMask_Unsupported(index);
        case USART_ID_2 :
            return USART_ExistsReceiverAddressMask_Unsupported(index);
        case USART_ID_3 :
            return USART_ExistsReceiverAddressMask_Unsupported(index);
        case USART_ID_4 :
            return USART_ExistsReceiverAddressMask_Unsupported(index);
        case USART_ID_5 :
            return USART_ExistsReceiverAddressMask_Unsupported(index);
        case USART_ID_6 :
            return USART_ExistsReceiverAddressMask_Unsupported(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_USART_AddressMaskSet(USART_MODULE_ID index, uint8_t mask)
{
    switch (index) {
        case USART_ID_1 :
            USART_AddressMaskSet_Unsupported(index, mask);
            break;
        case USART_ID_2 :
            USART_AddressMaskSet_Unsupported(index, mask);
            break;
        case USART_ID_3 :
            USART_AddressMaskSet_Unsupported(index, mask);
            break;
        case USART_ID_4 :
            USART_AddressMaskSet_Unsupported(index, mask);
            break;
        case USART_ID_5 :
            USART_AddressMaskSet_Unsupported(index, mask);
            break;
        case USART_ID_6 :
            USART_AddressMaskSet_Unsupported(index, mask);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_USART_AddressMaskGet(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_AddressMaskGet_Unsupported(index);
        case USART_ID_2 :
            return USART_AddressMaskGet_Unsupported(index);
        case USART_ID_3 :
            return USART_AddressMaskGet_Unsupported(index);
        case USART_ID_4 :
            return USART_AddressMaskGet_Unsupported(index);
        case USART_ID_5 :
            return USART_AddressMaskGet_Unsupported(index);
        case USART_ID_6 :
            return USART_AddressMaskGet_Unsupported(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsReceiverDataAvailableStatus(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsReceiverDataAvailableStatus_Default(index);
        case USART_ID_2 :
            return USART_ExistsReceiverDataAvailableStatus_Default(index);
        case USART_ID_3 :
            return USART_ExistsReceiverDataAvailableStatus_Default(index);
        case USART_ID_4 :
            return USART_ExistsReceiverDataAvailableStatus_Default(index);
        case USART_ID_5 :
            return USART_ExistsReceiverDataAvailableStatus_Default(index);
        case USART_ID_6 :
            return USART_ExistsReceiverDataAvailableStatus_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ReceiverDataIsAvailable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ReceiverDataIsAvailable_Default(index);
        case USART_ID_2 :
            return USART_ReceiverDataIsAvailable_Default(index);
        case USART_ID_3 :
            return USART_ReceiverDataIsAvailable_Default(index);
        case USART_ID_4 :
            return USART_ReceiverDataIsAvailable_Default(index);
        case USART_ID_5 :
            return USART_ReceiverDataIsAvailable_Default(index);
        case USART_ID_6 :
            return USART_ReceiverDataIsAvailable_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsReceiverEnable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsReceiverEnable_Default(index);
        case USART_ID_2 :
            return USART_ExistsReceiverEnable_Default(index);
        case USART_ID_3 :
            return USART_ExistsReceiverEnable_Default(index);
        case USART_ID_4 :
            return USART_ExistsReceiverEnable_Default(index);
        case USART_ID_5 :
            return USART_ExistsReceiverEnable_Default(index);
        case USART_ID_6 :
            return USART_ExistsReceiverEnable_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_ReceiverEnable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_ReceiverEnable_Default(index);
            break;
        case USART_ID_2 :
            USART_ReceiverEnable_Default(index);
            break;
        case USART_ID_3 :
            USART_ReceiverEnable_Default(index);
            break;
        case USART_ID_4 :
            USART_ReceiverEnable_Default(index);
            break;
        case USART_ID_5 :
            USART_ReceiverEnable_Default(index);
            break;
        case USART_ID_6 :
            USART_ReceiverEnable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_USART_ReceiverDisable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_ReceiverDisable_Default(index);
            break;
        case USART_ID_2 :
            USART_ReceiverDisable_Default(index);
            break;
        case USART_ID_3 :
            USART_ReceiverDisable_Default(index);
            break;
        case USART_ID_4 :
            USART_ReceiverDisable_Default(index);
            break;
        case USART_ID_5 :
            USART_ReceiverDisable_Default(index);
            break;
        case USART_ID_6 :
            USART_ReceiverDisable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsReceiverIdleStatus(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsReceiverIdleStatus_Default(index);
        case USART_ID_2 :
            return USART_ExistsReceiverIdleStatus_Default(index);
        case USART_ID_3 :
            return USART_ExistsReceiverIdleStatus_Default(index);
        case USART_ID_4 :
            return USART_ExistsReceiverIdleStatus_Default(index);
        case USART_ID_5 :
            return USART_ExistsReceiverIdleStatus_Default(index);
        case USART_ID_6 :
            return USART_ExistsReceiverIdleStatus_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ReceiverIsIdle(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ReceiverIsIdle_Default(index);
        case USART_ID_2 :
            return USART_ReceiverIsIdle_Default(index);
        case USART_ID_3 :
            return USART_ReceiverIsIdle_Default(index);
        case USART_ID_4 :
            return USART_ReceiverIsIdle_Default(index);
        case USART_ID_5 :
            return USART_ReceiverIsIdle_Default(index);
        case USART_ID_6 :
            return USART_ReceiverIsIdle_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsReceiverFramingErrorStatus(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsReceiverFramingErrorStatus_Default(index);
        case USART_ID_2 :
            return USART_ExistsReceiverFramingErrorStatus_Default(index);
        case USART_ID_3 :
            return USART_ExistsReceiverFramingErrorStatus_Default(index);
        case USART_ID_4 :
            return USART_ExistsReceiverFramingErrorStatus_Default(index);
        case USART_ID_5 :
            return USART_ExistsReceiverFramingErrorStatus_Default(index);
        case USART_ID_6 :
            return USART_ExistsReceiverFramingErrorStatus_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ReceiverFramingErrorHasOccurred(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ReceiverFramingErrorHasOccurred_Default(index);
        case USART_ID_2 :
            return USART_ReceiverFramingErrorHasOccurred_Default(index);
        case USART_ID_3 :
            return USART_ReceiverFramingErrorHasOccurred_Default(index);
        case USART_ID_4 :
            return USART_ReceiverFramingErrorHasOccurred_Default(index);
        case USART_ID_5 :
            return USART_ReceiverFramingErrorHasOccurred_Default(index);
        case USART_ID_6 :
            return USART_ReceiverFramingErrorHasOccurred_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsReceiverInterruptMode(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsReceiverInterruptMode_Default(index);
        case USART_ID_2 :
            return USART_ExistsReceiverInterruptMode_Default(index);
        case USART_ID_3 :
            return USART_ExistsReceiverInterruptMode_Default(index);
        case USART_ID_4 :
            return USART_ExistsReceiverInterruptMode_Default(index);
        case USART_ID_5 :
            return USART_ExistsReceiverInterruptMode_Default(index);
        case USART_ID_6 :
            return USART_ExistsReceiverInterruptMode_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_ReceiverInterruptModeSelect(USART_MODULE_ID index, USART_RECEIVE_INTR_MODE interruptMode)
{
    switch (index) {
        case USART_ID_1 :
            USART_ReceiverInterruptModeSelect_Default(index, interruptMode);
            break;
        case USART_ID_2 :
            USART_ReceiverInterruptModeSelect_Default(index, interruptMode);
            break;
        case USART_ID_3 :
            USART_ReceiverInterruptModeSelect_Default(index, interruptMode);
            break;
        case USART_ID_4 :
            USART_ReceiverInterruptModeSelect_Default(index, interruptMode);
            break;
        case USART_ID_5 :
            USART_ReceiverInterruptModeSelect_Default(index, interruptMode);
            break;
        case USART_ID_6 :
            USART_ReceiverInterruptModeSelect_Default(index, interruptMode);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_USART_InitializeOperation(USART_MODULE_ID index, USART_RECEIVE_INTR_MODE receiveInterruptMode, USART_TRANSMIT_INTR_MODE transmitInterruptMode, USART_OPERATION_MODE operationMode)
{
    switch (index) {
        case USART_ID_1 :
            USART_InitializeOperation_Default(index, receiveInterruptMode, transmitInterruptMode, operationMode);
            break;
        case USART_ID_2 :
            USART_InitializeOperation_Default(index, receiveInterruptMode, transmitInterruptMode, operationMode);
            break;
        case USART_ID_3 :
            USART_InitializeOperation_Default(index, receiveInterruptMode, transmitInterruptMode, operationMode);
            break;
        case USART_ID_4 :
            USART_InitializeOperation_Default(index, receiveInterruptMode, transmitInterruptMode, operationMode);
            break;
        case USART_ID_5 :
            USART_InitializeOperation_Default(index, receiveInterruptMode, transmitInterruptMode, operationMode);
            break;
        case USART_ID_6 :
            USART_InitializeOperation_Default(index, receiveInterruptMode, transmitInterruptMode, operationMode);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsReceiverIdleStateLowEnable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsReceiverIdleStateLowEnable_Default(index);
        case USART_ID_2 :
            return USART_ExistsReceiverIdleStateLowEnable_Default(index);
        case USART_ID_3 :
            return USART_ExistsReceiverIdleStateLowEnable_Default(index);
        case USART_ID_4 :
            return USART_ExistsReceiverIdleStateLowEnable_Default(index);
        case USART_ID_5 :
            return USART_ExistsReceiverIdleStateLowEnable_Default(index);
        case USART_ID_6 :
            return USART_ExistsReceiverIdleStateLowEnable_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_ReceiverIdleStateLowEnable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_ReceiverIdleStateLowEnable_Default(index);
            break;
        case USART_ID_2 :
            USART_ReceiverIdleStateLowEnable_Default(index);
            break;
        case USART_ID_3 :
            USART_ReceiverIdleStateLowEnable_Default(index);
            break;
        case USART_ID_4 :
            USART_ReceiverIdleStateLowEnable_Default(index);
            break;
        case USART_ID_5 :
            USART_ReceiverIdleStateLowEnable_Default(index);
            break;
        case USART_ID_6 :
            USART_ReceiverIdleStateLowEnable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_USART_ReceiverIdleStateLowDisable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_ReceiverIdleStateLowDisable_Default(index);
            break;
        case USART_ID_2 :
            USART_ReceiverIdleStateLowDisable_Default(index);
            break;
        case USART_ID_3 :
            USART_ReceiverIdleStateLowDisable_Default(index);
            break;
        case USART_ID_4 :
            USART_ReceiverIdleStateLowDisable_Default(index);
            break;
        case USART_ID_5 :
            USART_ReceiverIdleStateLowDisable_Default(index);
            break;
        case USART_ID_6 :
            USART_ReceiverIdleStateLowDisable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsReceiverParityErrorStatus(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsReceiverParityErrorStatus_Default(index);
        case USART_ID_2 :
            return USART_ExistsReceiverParityErrorStatus_Default(index);
        case USART_ID_3 :
            return USART_ExistsReceiverParityErrorStatus_Default(index);
        case USART_ID_4 :
            return USART_ExistsReceiverParityErrorStatus_Default(index);
        case USART_ID_5 :
            return USART_ExistsReceiverParityErrorStatus_Default(index);
        case USART_ID_6 :
            return USART_ExistsReceiverParityErrorStatus_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ReceiverParityErrorHasOccurred(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ReceiverParityErrorHasOccurred_Default(index);
        case USART_ID_2 :
            return USART_ReceiverParityErrorHasOccurred_Default(index);
        case USART_ID_3 :
            return USART_ReceiverParityErrorHasOccurred_Default(index);
        case USART_ID_4 :
            return USART_ReceiverParityErrorHasOccurred_Default(index);
        case USART_ID_5 :
            return USART_ReceiverParityErrorHasOccurred_Default(index);
        case USART_ID_6 :
            return USART_ReceiverParityErrorHasOccurred_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsReceiverOverrunStatus(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsReceiverOverrunStatus_Default(index);
        case USART_ID_2 :
            return USART_ExistsReceiverOverrunStatus_Default(index);
        case USART_ID_3 :
            return USART_ExistsReceiverOverrunStatus_Default(index);
        case USART_ID_4 :
            return USART_ExistsReceiverOverrunStatus_Default(index);
        case USART_ID_5 :
            return USART_ExistsReceiverOverrunStatus_Default(index);
        case USART_ID_6 :
            return USART_ExistsReceiverOverrunStatus_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_ReceiverOverrunErrorClear(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_ReceiverOverrunErrorClear_Default(index);
            break;
        case USART_ID_2 :
            USART_ReceiverOverrunErrorClear_Default(index);
            break;
        case USART_ID_3 :
            USART_ReceiverOverrunErrorClear_Default(index);
            break;
        case USART_ID_4 :
            USART_ReceiverOverrunErrorClear_Default(index);
            break;
        case USART_ID_5 :
            USART_ReceiverOverrunErrorClear_Default(index);
            break;
        case USART_ID_6 :
            USART_ReceiverOverrunErrorClear_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_USART_ReceiverOverrunHasOccurred(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ReceiverOverrunHasOccurred_Default(index);
        case USART_ID_2 :
            return USART_ReceiverOverrunHasOccurred_Default(index);
        case USART_ID_3 :
            return USART_ReceiverOverrunHasOccurred_Default(index);
        case USART_ID_4 :
            return USART_ReceiverOverrunHasOccurred_Default(index);
        case USART_ID_5 :
            return USART_ReceiverOverrunHasOccurred_Default(index);
        case USART_ID_6 :
            return USART_ReceiverOverrunHasOccurred_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsTransmitter(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsTransmitter_Default(index);
        case USART_ID_2 :
            return USART_ExistsTransmitter_Default(index);
        case USART_ID_3 :
            return USART_ExistsTransmitter_Default(index);
        case USART_ID_4 :
            return USART_ExistsTransmitter_Default(index);
        case USART_ID_5 :
            return USART_ExistsTransmitter_Default(index);
        case USART_ID_6 :
            return USART_ExistsTransmitter_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_TransmitterByteSend(USART_MODULE_ID index, int8_t data)
{
    switch (index) {
        case USART_ID_1 :
            USART_TransmitterByteSend_Default(index, data);
            break;
        case USART_ID_2 :
            USART_TransmitterByteSend_Default(index, data);
            break;
        case USART_ID_3 :
            USART_TransmitterByteSend_Default(index, data);
            break;
        case USART_ID_4 :
            USART_TransmitterByteSend_Default(index, data);
            break;
        case USART_ID_5 :
            USART_TransmitterByteSend_Default(index, data);
            break;
        case USART_ID_6 :
            USART_TransmitterByteSend_Default(index, data);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void* PLIB_USART_TransmitterAddressGet(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_TransmitterAddressGet_Default(index);
        case USART_ID_2 :
            return USART_TransmitterAddressGet_Default(index);
        case USART_ID_3 :
            return USART_TransmitterAddressGet_Default(index);
        case USART_ID_4 :
            return USART_TransmitterAddressGet_Default(index);
        case USART_ID_5 :
            return USART_TransmitterAddressGet_Default(index);
        case USART_ID_6 :
            return USART_TransmitterAddressGet_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (void*)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsTransmitter9BitsSend(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsTransmitter9BitsSend_InDataOnly(index);
        case USART_ID_2 :
            return USART_ExistsTransmitter9BitsSend_InDataOnly(index);
        case USART_ID_3 :
            return USART_ExistsTransmitter9BitsSend_InDataOnly(index);
        case USART_ID_4 :
            return USART_ExistsTransmitter9BitsSend_InDataOnly(index);
        case USART_ID_5 :
            return USART_ExistsTransmitter9BitsSend_InDataOnly(index);
        case USART_ID_6 :
            return USART_ExistsTransmitter9BitsSend_InDataOnly(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_Transmitter9BitsSend(USART_MODULE_ID index, int8_t data, bool Bit9th)
{
    switch (index) {
        case USART_ID_1 :
            USART_Transmitter9BitsSend_InDataOnly(index, data, Bit9th);
            break;
        case USART_ID_2 :
            USART_Transmitter9BitsSend_InDataOnly(index, data, Bit9th);
            break;
        case USART_ID_3 :
            USART_Transmitter9BitsSend_InDataOnly(index, data, Bit9th);
            break;
        case USART_ID_4 :
            USART_Transmitter9BitsSend_InDataOnly(index, data, Bit9th);
            break;
        case USART_ID_5 :
            USART_Transmitter9BitsSend_InDataOnly(index, data, Bit9th);
            break;
        case USART_ID_6 :
            USART_Transmitter9BitsSend_InDataOnly(index, data, Bit9th);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsTransmitterBreak(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsTransmitterBreak_Default(index);
        case USART_ID_2 :
            return USART_ExistsTransmitterBreak_Default(index);
        case USART_ID_3 :
            return USART_ExistsTransmitterBreak_Default(index);
        case USART_ID_4 :
            return USART_ExistsTransmitterBreak_Default(index);
        case USART_ID_5 :
            return USART_ExistsTransmitterBreak_Default(index);
        case USART_ID_6 :
            return USART_ExistsTransmitterBreak_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_TransmitterBreakSend(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_TransmitterBreakSend_Default(index);
            break;
        case USART_ID_2 :
            USART_TransmitterBreakSend_Default(index);
            break;
        case USART_ID_3 :
            USART_TransmitterBreakSend_Default(index);
            break;
        case USART_ID_4 :
            USART_TransmitterBreakSend_Default(index);
            break;
        case USART_ID_5 :
            USART_TransmitterBreakSend_Default(index);
            break;
        case USART_ID_6 :
            USART_TransmitterBreakSend_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_USART_TransmitterBreakSendIsComplete(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_TransmitterBreakSendIsComplete_Default(index);
        case USART_ID_2 :
            return USART_TransmitterBreakSendIsComplete_Default(index);
        case USART_ID_3 :
            return USART_TransmitterBreakSendIsComplete_Default(index);
        case USART_ID_4 :
            return USART_TransmitterBreakSendIsComplete_Default(index);
        case USART_ID_5 :
            return USART_TransmitterBreakSendIsComplete_Default(index);
        case USART_ID_6 :
            return USART_TransmitterBreakSendIsComplete_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsTransmitterBufferFullStatus(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsTransmitterBufferFullStatus_Default(index);
        case USART_ID_2 :
            return USART_ExistsTransmitterBufferFullStatus_Default(index);
        case USART_ID_3 :
            return USART_ExistsTransmitterBufferFullStatus_Default(index);
        case USART_ID_4 :
            return USART_ExistsTransmitterBufferFullStatus_Default(index);
        case USART_ID_5 :
            return USART_ExistsTransmitterBufferFullStatus_Default(index);
        case USART_ID_6 :
            return USART_ExistsTransmitterBufferFullStatus_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_TransmitterBufferIsFull(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_TransmitterBufferIsFull_Default(index);
        case USART_ID_2 :
            return USART_TransmitterBufferIsFull_Default(index);
        case USART_ID_3 :
            return USART_TransmitterBufferIsFull_Default(index);
        case USART_ID_4 :
            return USART_TransmitterBufferIsFull_Default(index);
        case USART_ID_5 :
            return USART_TransmitterBufferIsFull_Default(index);
        case USART_ID_6 :
            return USART_TransmitterBufferIsFull_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_TransmitterIsEmpty(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_TransmitterIsEmpty_Default(index);
        case USART_ID_2 :
            return USART_TransmitterIsEmpty_Default(index);
        case USART_ID_3 :
            return USART_TransmitterIsEmpty_Default(index);
        case USART_ID_4 :
            return USART_TransmitterIsEmpty_Default(index);
        case USART_ID_5 :
            return USART_TransmitterIsEmpty_Default(index);
        case USART_ID_6 :
            return USART_TransmitterIsEmpty_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsTransmitterEmptyStatus(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsTransmitterEmptyStatus_Default(index);
        case USART_ID_2 :
            return USART_ExistsTransmitterEmptyStatus_Default(index);
        case USART_ID_3 :
            return USART_ExistsTransmitterEmptyStatus_Default(index);
        case USART_ID_4 :
            return USART_ExistsTransmitterEmptyStatus_Default(index);
        case USART_ID_5 :
            return USART_ExistsTransmitterEmptyStatus_Default(index);
        case USART_ID_6 :
            return USART_ExistsTransmitterEmptyStatus_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsTransmitterEnable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsTransmitterEnable_Default(index);
        case USART_ID_2 :
            return USART_ExistsTransmitterEnable_Default(index);
        case USART_ID_3 :
            return USART_ExistsTransmitterEnable_Default(index);
        case USART_ID_4 :
            return USART_ExistsTransmitterEnable_Default(index);
        case USART_ID_5 :
            return USART_ExistsTransmitterEnable_Default(index);
        case USART_ID_6 :
            return USART_ExistsTransmitterEnable_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_TransmitterEnable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_TransmitterEnable_Default(index);
            break;
        case USART_ID_2 :
            USART_TransmitterEnable_Default(index);
            break;
        case USART_ID_3 :
            USART_TransmitterEnable_Default(index);
            break;
        case USART_ID_4 :
            USART_TransmitterEnable_Default(index);
            break;
        case USART_ID_5 :
            USART_TransmitterEnable_Default(index);
            break;
        case USART_ID_6 :
            USART_TransmitterEnable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_USART_TransmitterDisable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_TransmitterDisable_Default(index);
            break;
        case USART_ID_2 :
            USART_TransmitterDisable_Default(index);
            break;
        case USART_ID_3 :
            USART_TransmitterDisable_Default(index);
            break;
        case USART_ID_4 :
            USART_TransmitterDisable_Default(index);
            break;
        case USART_ID_5 :
            USART_TransmitterDisable_Default(index);
            break;
        case USART_ID_6 :
            USART_TransmitterDisable_Default(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_USART_TransmitterInterruptModeSelect(USART_MODULE_ID index, USART_TRANSMIT_INTR_MODE fifolevel)
{
    switch (index) {
        case USART_ID_1 :
            USART_TransmitterInterruptModeSelect_Default(index, fifolevel);
            break;
        case USART_ID_2 :
            USART_TransmitterInterruptModeSelect_Default(index, fifolevel);
            break;
        case USART_ID_3 :
            USART_TransmitterInterruptModeSelect_Default(index, fifolevel);
            break;
        case USART_ID_4 :
            USART_TransmitterInterruptModeSelect_Default(index, fifolevel);
            break;
        case USART_ID_5 :
            USART_TransmitterInterruptModeSelect_Default(index, fifolevel);
            break;
        case USART_ID_6 :
            USART_TransmitterInterruptModeSelect_Default(index, fifolevel);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsTransmitterInterruptMode(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsTransmitterInterruptMode_Default(index);
        case USART_ID_2 :
            return USART_ExistsTransmitterInterruptMode_Default(index);
        case USART_ID_3 :
            return USART_ExistsTransmitterInterruptMode_Default(index);
        case USART_ID_4 :
            return USART_ExistsTransmitterInterruptMode_Default(index);
        case USART_ID_5 :
            return USART_ExistsTransmitterInterruptMode_Default(index);
        case USART_ID_6 :
            return USART_ExistsTransmitterInterruptMode_Default(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_USART_ExistsTransmitterIdleIsLow(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            return USART_ExistsTransmitterIdleIsLow_pic32(index);
        case USART_ID_2 :
            return USART_ExistsTransmitterIdleIsLow_pic32(index);
        case USART_ID_3 :
            return USART_ExistsTransmitterIdleIsLow_pic32(index);
        case USART_ID_4 :
            return USART_ExistsTransmitterIdleIsLow_pic32(index);
        case USART_ID_5 :
            return USART_ExistsTransmitterIdleIsLow_pic32(index);
        case USART_ID_6 :
            return USART_ExistsTransmitterIdleIsLow_pic32(index);
        case USART_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_USART_TransmitterIdleIsLowDisable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_TransmitterIdleIsLowDisable_pic32(index);
            break;
        case USART_ID_2 :
            USART_TransmitterIdleIsLowDisable_pic32(index);
            break;
        case USART_ID_3 :
            USART_TransmitterIdleIsLowDisable_pic32(index);
            break;
        case USART_ID_4 :
            USART_TransmitterIdleIsLowDisable_pic32(index);
            break;
        case USART_ID_5 :
            USART_TransmitterIdleIsLowDisable_pic32(index);
            break;
        case USART_ID_6 :
            USART_TransmitterIdleIsLowDisable_pic32(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_USART_TransmitterIdleIsLowEnable(USART_MODULE_ID index)
{
    switch (index) {
        case USART_ID_1 :
            USART_TransmitterIdleIsLowEnable_pic32(index);
            break;
        case USART_ID_2 :
            USART_TransmitterIdleIsLowEnable_pic32(index);
            break;
        case USART_ID_3 :
            USART_TransmitterIdleIsLowEnable_pic32(index);
            break;
        case USART_ID_4 :
            USART_TransmitterIdleIsLowEnable_pic32(index);
            break;
        case USART_ID_5 :
            USART_TransmitterIdleIsLowEnable_pic32(index);
            break;
        case USART_ID_6 :
            USART_TransmitterIdleIsLowEnable_pic32(index);
            break;
        case USART_NUMBER_OF_MODULES :
        default :
            break;
    }
}

#endif
