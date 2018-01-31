/* Created by plibgen $Revision: 1.31 $ */

#ifndef _ETH_P32MZ2048EFM144_H
#define _ETH_P32MZ2048EFM144_H

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

    ETH_ID_0 = 0,
    ETH_NUMBER_OF_MODULES

} ETH_MODULE_ID;

PLIB_INLINE SFR_TYPE* _ETH_PAUSE_TIMER_VALUE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHCON1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_ETHERNET_ON_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHCON1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_STOP_IN_IDLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHCON1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_TX_REQUEST_TO_SEND_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHCON1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RX_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHCON1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_AUTO_FLOW_CONTROL_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHCON1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_MANUAL_FLOW_CONTROL_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHCON1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_BUFCNT_DECREMENT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHCON1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RX_DATA_BUFFER_SIZE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHCON2;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_TX_START_ADDRESS_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHTXST;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RX_START_ADDRESS_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHRXST;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_HASH_TABLE_LOWER_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHHT0;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_HASH_TABLE_UPPER_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHHT1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_PATTERN_MATCH_LOWER_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHPMM0;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_PATTERN_MATCH_UPPER_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHPMM1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_PATTERN_MATCH_CHECKSUM_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHPMCS;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_PATTERN_MATCH_OFFSET_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHPMO;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_HASH_FILTER_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHRXFC;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_MAGIC_FILTER_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHRXFC;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_INVERT_PATTERN_MATCH_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHRXFC;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_PATTERN_MATCH_MODE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHRXFC;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_CRC_ERR_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHRXFC;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_CRC_OK_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHRXFC;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RUNT_ERR_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHRXFC;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RUNT_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHRXFC;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_UNICAST_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHRXFC;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_NOT_ME_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHRXFC;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_MULTICAST_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHRXFC;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_BROADCAST_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHRXFC;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RX_FULL_WMARK_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHRXWM;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RX_EMPTY_WMARK_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHRXWM;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_TX_BUS_ERROR_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIEN;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RX_BUS_ERROR_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIEN;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_EMPTY_WMARK_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIEN;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_FULL_WMARK_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIEN;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RX_DONE_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIEN;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_PACKET_PENDING_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIEN;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RX_ACTIVITY_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIEN;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_TX_DONE_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIEN;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_TX_ABORT_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIEN;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RX_BUFFER_NOTAVAIL_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIEN;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RX_FIFO_OVERFLOW_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIEN;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_TX_BUS_ERROR_INT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIRQ;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RX_BUS_ERROR_INT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIRQ;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_EMPTY_WMARK_INT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIRQ;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_FULL_WMARK_INT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIRQ;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RX_DONE_INT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIRQ;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_PACKET_PENDING_INT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIRQ;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RX_ACTIVITY_INT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIRQ;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_TX_DONE_INT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIRQ;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_TX_ABORT_INT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIRQ;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RX_BUFFER_NOTAVAIL_INT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIRQ;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RX_FIFO_OVERFLOW_INT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHIRQ;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_PACKET_BUFFER_COUNT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHSTAT;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_ETHERNET_MODULE_BUSY_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHSTAT;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_TX_BUSY_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHSTAT;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RX_BUSY_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHSTAT;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RX_OVERFLOW_COUNT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHRXOVFLOW;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_FRAME_TX_OK_COUNT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHFRMTXOK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_SINGLE_COL_FRAME_COUNT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHSCOLFRM;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_MULTI_COL_FRAME_COUNT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHMCOLFRM;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_FRAMES_RCVD_OK_COUNT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHFRMRXOK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_FCS_ERR_COUNT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHFCSERR;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_ALIGNMENT_ERR_COUNT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &ETHALGNERR;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_MII_RESET_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_SIMULATION_RESET_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RESET_MCS_RX_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RESET_RX_FUNCTION_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RESET_MCS_TX_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RESET_TX_FUNCTION_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_MAC_LOOPBACK_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_TX_FLOW_CONTROL_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RX_FLOW_CONTROL_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_PASS_ALL_RX_FRAMES_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_MAC_RX_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_EXCESS_DEFER_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG2;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_BACKPRESSURE_NOBACKOFF_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG2;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_NO_BACKOFF_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG2;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_LONG_PREAMBLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG2;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_PURE_PREAMBLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG2;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_AUTO_DETECT_PAD_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG2;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_VLAN_PAD_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG2;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_PAD_CRC_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG2;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_CRC_ENABLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG2;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_DELAYED_CRC_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG2;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_HUGE_FRAME_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG2;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_LENGTH_CHECK_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG2;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_FULL_DUPLEX_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CFG2;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_BACK2BACK_IPGAP_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1IPGT;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_NON_BACK2BACK_IPGAP1_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1IPGR;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_NON_BACK2BACK_IPGAP2_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1IPGR;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_COL_WINDOW_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CLRT;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RETX_MAX_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1CLRT;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_MAX_FRAME_LENGTH_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1MAXF;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RESET_RMII_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1SUPP;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_SPEED_RMII_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1SUPP;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_TEST_BACKPRESSURE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1TEST;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_TEST_PAUSE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1TEST;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_SHORTCUT_PAUSE_QUANTA_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1TEST;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_RESET_MIIM_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1MCFG;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_MII_CLOCK_SEL_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1MCFG;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_NO_PREAMBLE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1MCFG;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_SCAN_INCREMENT_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1MCFG;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_MIIM_SCAN_MODE_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1MCMD;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_MIIM_READ_COMMAND_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1MCMD;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_MIIM_PHY_ADDRESS_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1MADR;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_MIIM_REGISTER_ADDRESS_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1MADR;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_MIIM_WRITE_DATA_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1MWTD;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_MIIM_READ_DATA_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1MRDD;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_LINK_FAILED_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1MIND;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_DATA_NOT_VALID_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1MIND;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_MIIM_SCANNING_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1MIND;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_MIIM_BUSY_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1MIND;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_STATION_ADDRESS_OCTET_6_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1SA0;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_STATION_ADDRESS_OCTET_5_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1SA0;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_STATION_ADDRESS_OCTET_4_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1SA1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_STATION_ADDRESS_OCTET_3_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1SA1;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_STATION_ADDRESS_OCTET_2_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1SA2;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _ETH_STATION_ADDRESS_OCTET_1_VREG(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return &EMAC1SA2;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PAUSE_TIMER_VALUE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_PTV_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_ETHERNET_ON_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_ON_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STOP_IN_IDLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_SIDL_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_REQUEST_TO_SEND_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_TXRTS_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_RXEN_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_AUTO_FLOW_CONTROL_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_AUTOFC_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MANUAL_FLOW_CONTROL_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_MANFC_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_BUFCNT_DECREMENT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_BUFCDEC_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_DATA_BUFFER_SIZE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON2_RXBUF_SZ_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_START_ADDRESS_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHTXST_TXSTADDR_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_START_ADDRESS_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXST_RXSTADDR_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_HASH_TABLE_LOWER_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHHT0_HTLOWER_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_HASH_TABLE_UPPER_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHHT1_HTUPPER_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PATTERN_MATCH_LOWER_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHPMM0_PMMLOWER_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PATTERN_MATCH_UPPER_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHPMM1_PMMUPPER_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PATTERN_MATCH_CHECKSUM_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHPMCS_PMCS_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PATTERN_MATCH_OFFSET_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHPMO_PMO_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_HASH_FILTER_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_HTEN_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MAGIC_FILTER_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_MPEN_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_INVERT_PATTERN_MATCH_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_NOTPM_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PATTERN_MATCH_MODE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_PMMODE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_CRC_ERR_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_CRCERREN_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_CRC_OK_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_CRCOKEN_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RUNT_ERR_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_RUNTERREN_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RUNT_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_RUNTEN_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_UNICAST_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_UCEN_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_NOT_ME_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_NOTMEEN_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MULTICAST_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_MCEN_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_BROADCAST_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_BCEN_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_FULL_WMARK_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXWM_RXFWM_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_EMPTY_WMARK_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXWM_RXEWM_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_BUS_ERROR_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_TXBUSEIE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_BUS_ERROR_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_RXBUSEIE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_EMPTY_WMARK_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_EWMARKIE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_FULL_WMARK_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_FWMARKIE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_DONE_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_RXDONEIE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PACKET_PENDING_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_PKTPENDIE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_ACTIVITY_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_RXACTIE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_DONE_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_TXDONEIE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_ABORT_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_TXABORTIE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_BUFFER_NOTAVAIL_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_RXBUFNAIE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_FIFO_OVERFLOW_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_RXOVFLWIE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_BUS_ERROR_INT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_TXBUSE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_BUS_ERROR_INT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_RXBUSE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_EMPTY_WMARK_INT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_EWMARK_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_FULL_WMARK_INT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_FWMARK_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_DONE_INT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_RXDONE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PACKET_PENDING_INT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_PKTPEND_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_ACTIVITY_INT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_RXACT_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_DONE_INT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_TXDONE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_ABORT_INT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_TXABORT_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_BUFFER_NOTAVAIL_INT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_RXBUFNA_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_FIFO_OVERFLOW_INT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_RXOVFLW_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PACKET_BUFFER_COUNT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHSTAT_BUFCNT_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_ETHERNET_MODULE_BUSY_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHSTAT_ETHBUSY_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_BUSY_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHSTAT_TXBUSY_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_BUSY_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHSTAT_RXBUSY_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_OVERFLOW_COUNT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXOVFLOW_RXOVFLWCNT_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_FRAME_TX_OK_COUNT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHFRMTXOK_FRMTXOKCNT_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_SINGLE_COL_FRAME_COUNT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHSCOLFRM_SCOLFRMCNT_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MULTI_COL_FRAME_COUNT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHMCOLFRM_MCOLFRMCNT_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_FRAMES_RCVD_OK_COUNT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHFRMRXOK_FRMRXOKCNT_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_FCS_ERR_COUNT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHFCSERR_FCSERRCNT_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_ALIGNMENT_ERR_COUNT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHALGNERR_ALGNERRCNT_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MII_RESET_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_SOFTRESET_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_SIMULATION_RESET_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_SIMRESET_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RESET_MCS_RX_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_RESETRMCS_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RESET_RX_FUNCTION_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_RESETRFUN_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RESET_MCS_TX_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_RESETTMCS_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RESET_TX_FUNCTION_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_RESETTFUN_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MAC_LOOPBACK_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_LOOPBACK_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_FLOW_CONTROL_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_TXPAUSE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_FLOW_CONTROL_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_RXPAUSE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PASS_ALL_RX_FRAMES_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_PASSALL_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MAC_RX_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_RXENABLE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_EXCESS_DEFER_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_EXCESSDFR_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_BACKPRESSURE_NOBACKOFF_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_BPNOBKOFF_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_NO_BACKOFF_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_NOBKOFF_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_LONG_PREAMBLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_LONGPRE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PURE_PREAMBLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_PUREPRE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_AUTO_DETECT_PAD_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_AUTOPAD_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_VLAN_PAD_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_VLANPAD_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PAD_CRC_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_PADENABLE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_CRC_ENABLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_CRCENABLE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_DELAYED_CRC_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_DELAYCRC_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_HUGE_FRAME_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_HUGEFRM_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_LENGTH_CHECK_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_LENGTHCK_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_FULL_DUPLEX_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_FULLDPLX_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_BACK2BACK_IPGAP_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1IPGT_B2BIPKTGP_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_NON_BACK2BACK_IPGAP1_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1IPGR_NB2BIPKTGP1_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_NON_BACK2BACK_IPGAP2_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1IPGR_NB2BIPKTGP2_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_COL_WINDOW_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CLRT_CWINDOW_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RETX_MAX_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CLRT_RETX_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MAX_FRAME_LENGTH_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MAXF_MACMAXF_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RESET_RMII_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SUPP_RESETRMII_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_SPEED_RMII_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SUPP_SPEEDRMII_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TEST_BACKPRESSURE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1TEST_TESTBP_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TEST_PAUSE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1TEST_TESTPAUSE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_SHORTCUT_PAUSE_QUANTA_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1TEST_SHRTQNTA_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RESET_MIIM_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MCFG_RESETMGMT_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MII_CLOCK_SEL_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MCFG_CLKSEL_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_NO_PREAMBLE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MCFG_NOPRE_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_SCAN_INCREMENT_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MCFG_SCANINC_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_SCAN_MODE_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MCMD_SCAN_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_READ_COMMAND_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MCMD_READ_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_PHY_ADDRESS_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MADR_PHYADDR_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_REGISTER_ADDRESS_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MADR_REGADDR_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_WRITE_DATA_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MWTD_MWTD_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_READ_DATA_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MRDD_MRDD_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_LINK_FAILED_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MIND_LINKFAIL_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_DATA_NOT_VALID_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MIND_NOTVALID_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_SCANNING_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MIND_SCAN_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_BUSY_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MIND_MIIMBUSY_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STATION_ADDRESS_OCTET_6_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SA0_STNADDR6_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STATION_ADDRESS_OCTET_5_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SA0_STNADDR5_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STATION_ADDRESS_OCTET_4_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SA1_STNADDR4_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STATION_ADDRESS_OCTET_3_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SA1_STNADDR3_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STATION_ADDRESS_OCTET_2_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SA2_STNADDR2_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STATION_ADDRESS_OCTET_1_MASK(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SA2_STNADDR1_MASK;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PAUSE_TIMER_VALUE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_PTV_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_ETHERNET_ON_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_ON_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STOP_IN_IDLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_SIDL_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_REQUEST_TO_SEND_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_TXRTS_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_RXEN_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_AUTO_FLOW_CONTROL_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_AUTOFC_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MANUAL_FLOW_CONTROL_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_MANFC_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_BUFCNT_DECREMENT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_BUFCDEC_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_DATA_BUFFER_SIZE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON2_RXBUF_SZ_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_START_ADDRESS_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHTXST_TXSTADDR_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_START_ADDRESS_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXST_RXSTADDR_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_HASH_TABLE_LOWER_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHHT0_HTLOWER_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_HASH_TABLE_UPPER_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHHT1_HTUPPER_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PATTERN_MATCH_LOWER_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHPMM0_PMMLOWER_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PATTERN_MATCH_UPPER_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHPMM1_PMMUPPER_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PATTERN_MATCH_CHECKSUM_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHPMCS_PMCS_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PATTERN_MATCH_OFFSET_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHPMO_PMO_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_HASH_FILTER_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_HTEN_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MAGIC_FILTER_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_MPEN_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_INVERT_PATTERN_MATCH_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_NOTPM_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PATTERN_MATCH_MODE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_PMMODE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_CRC_ERR_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_CRCERREN_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_CRC_OK_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_CRCOKEN_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RUNT_ERR_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_RUNTERREN_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RUNT_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_RUNTEN_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_UNICAST_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_UCEN_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_NOT_ME_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_NOTMEEN_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MULTICAST_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_MCEN_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_BROADCAST_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_BCEN_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_FULL_WMARK_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXWM_RXFWM_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_EMPTY_WMARK_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXWM_RXEWM_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_BUS_ERROR_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_TXBUSEIE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_BUS_ERROR_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_RXBUSEIE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_EMPTY_WMARK_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_EWMARKIE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_FULL_WMARK_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_FWMARKIE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_DONE_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_RXDONEIE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PACKET_PENDING_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_PKTPENDIE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_ACTIVITY_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_RXACTIE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_DONE_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_TXDONEIE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_ABORT_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_TXABORTIE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_BUFFER_NOTAVAIL_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_RXBUFNAIE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_FIFO_OVERFLOW_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_RXOVFLWIE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_BUS_ERROR_INT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_TXBUSE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_BUS_ERROR_INT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_RXBUSE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_EMPTY_WMARK_INT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_EWMARK_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_FULL_WMARK_INT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_FWMARK_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_DONE_INT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_RXDONE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PACKET_PENDING_INT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_PKTPEND_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_ACTIVITY_INT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_RXACT_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_DONE_INT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_TXDONE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_ABORT_INT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_TXABORT_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_BUFFER_NOTAVAIL_INT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_RXBUFNA_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_FIFO_OVERFLOW_INT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_RXOVFLW_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PACKET_BUFFER_COUNT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHSTAT_BUFCNT_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_ETHERNET_MODULE_BUSY_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHSTAT_ETHBUSY_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_BUSY_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHSTAT_TXBUSY_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_BUSY_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHSTAT_RXBUSY_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_OVERFLOW_COUNT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXOVFLOW_RXOVFLWCNT_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_FRAME_TX_OK_COUNT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHFRMTXOK_FRMTXOKCNT_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_SINGLE_COL_FRAME_COUNT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHSCOLFRM_SCOLFRMCNT_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MULTI_COL_FRAME_COUNT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHMCOLFRM_MCOLFRMCNT_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_FRAMES_RCVD_OK_COUNT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHFRMRXOK_FRMRXOKCNT_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_FCS_ERR_COUNT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHFCSERR_FCSERRCNT_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_ALIGNMENT_ERR_COUNT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHALGNERR_ALGNERRCNT_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MII_RESET_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_SOFTRESET_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_SIMULATION_RESET_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_SIMRESET_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RESET_MCS_RX_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_RESETRMCS_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RESET_RX_FUNCTION_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_RESETRFUN_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RESET_MCS_TX_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_RESETTMCS_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RESET_TX_FUNCTION_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_RESETTFUN_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MAC_LOOPBACK_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_LOOPBACK_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_FLOW_CONTROL_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_TXPAUSE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_FLOW_CONTROL_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_RXPAUSE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PASS_ALL_RX_FRAMES_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_PASSALL_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MAC_RX_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_RXENABLE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_EXCESS_DEFER_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_EXCESSDFR_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_BACKPRESSURE_NOBACKOFF_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_BPNOBKOFF_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_NO_BACKOFF_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_NOBKOFF_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_LONG_PREAMBLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_LONGPRE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PURE_PREAMBLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_PUREPRE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_AUTO_DETECT_PAD_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_AUTOPAD_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_VLAN_PAD_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_VLANPAD_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PAD_CRC_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_PADENABLE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_CRC_ENABLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_CRCENABLE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_DELAYED_CRC_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_DELAYCRC_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_HUGE_FRAME_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_HUGEFRM_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_LENGTH_CHECK_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_LENGTHCK_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_FULL_DUPLEX_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_FULLDPLX_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_BACK2BACK_IPGAP_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1IPGT_B2BIPKTGP_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_NON_BACK2BACK_IPGAP1_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1IPGR_NB2BIPKTGP1_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_NON_BACK2BACK_IPGAP2_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1IPGR_NB2BIPKTGP2_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_COL_WINDOW_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CLRT_CWINDOW_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RETX_MAX_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CLRT_RETX_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MAX_FRAME_LENGTH_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MAXF_MACMAXF_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RESET_RMII_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SUPP_RESETRMII_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_SPEED_RMII_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SUPP_SPEEDRMII_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TEST_BACKPRESSURE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1TEST_TESTBP_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TEST_PAUSE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1TEST_TESTPAUSE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_SHORTCUT_PAUSE_QUANTA_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1TEST_SHRTQNTA_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RESET_MIIM_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MCFG_RESETMGMT_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MII_CLOCK_SEL_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MCFG_CLKSEL_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_NO_PREAMBLE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MCFG_NOPRE_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_SCAN_INCREMENT_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MCFG_SCANINC_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_SCAN_MODE_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MCMD_SCAN_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_READ_COMMAND_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MCMD_READ_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_PHY_ADDRESS_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MADR_PHYADDR_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_REGISTER_ADDRESS_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MADR_REGADDR_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_WRITE_DATA_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MWTD_MWTD_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_READ_DATA_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MRDD_MRDD_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_LINK_FAILED_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MIND_LINKFAIL_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_DATA_NOT_VALID_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MIND_NOTVALID_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_SCANNING_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MIND_SCAN_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_BUSY_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MIND_MIIMBUSY_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STATION_ADDRESS_OCTET_6_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SA0_STNADDR6_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STATION_ADDRESS_OCTET_5_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SA0_STNADDR5_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STATION_ADDRESS_OCTET_4_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SA1_STNADDR4_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STATION_ADDRESS_OCTET_3_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SA1_STNADDR3_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STATION_ADDRESS_OCTET_2_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SA2_STNADDR2_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STATION_ADDRESS_OCTET_1_POS(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SA2_STNADDR1_POSITION;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PAUSE_TIMER_VALUE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_PTV_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_ETHERNET_ON_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_ON_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STOP_IN_IDLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_SIDL_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_REQUEST_TO_SEND_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_TXRTS_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_RXEN_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_AUTO_FLOW_CONTROL_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_AUTOFC_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MANUAL_FLOW_CONTROL_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_MANFC_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_BUFCNT_DECREMENT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON1_BUFCDEC_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_DATA_BUFFER_SIZE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHCON2_RXBUF_SZ_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_START_ADDRESS_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHTXST_TXSTADDR_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_START_ADDRESS_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXST_RXSTADDR_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_HASH_TABLE_LOWER_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHHT0_HTLOWER_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_HASH_TABLE_UPPER_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHHT1_HTUPPER_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PATTERN_MATCH_LOWER_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHPMM0_PMMLOWER_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PATTERN_MATCH_UPPER_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHPMM1_PMMUPPER_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PATTERN_MATCH_CHECKSUM_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHPMCS_PMCS_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PATTERN_MATCH_OFFSET_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHPMO_PMO_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_HASH_FILTER_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_HTEN_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MAGIC_FILTER_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_MPEN_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_INVERT_PATTERN_MATCH_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_NOTPM_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PATTERN_MATCH_MODE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_PMMODE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_CRC_ERR_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_CRCERREN_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_CRC_OK_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_CRCOKEN_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RUNT_ERR_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_RUNTERREN_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RUNT_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_RUNTEN_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_UNICAST_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_UCEN_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_NOT_ME_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_NOTMEEN_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MULTICAST_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_MCEN_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_BROADCAST_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXFC_BCEN_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_FULL_WMARK_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXWM_RXFWM_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_EMPTY_WMARK_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXWM_RXEWM_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_BUS_ERROR_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_TXBUSEIE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_BUS_ERROR_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_RXBUSEIE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_EMPTY_WMARK_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_EWMARKIE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_FULL_WMARK_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_FWMARKIE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_DONE_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_RXDONEIE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PACKET_PENDING_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_PKTPENDIE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_ACTIVITY_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_RXACTIE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_DONE_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_TXDONEIE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_ABORT_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_TXABORTIE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_BUFFER_NOTAVAIL_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_RXBUFNAIE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_FIFO_OVERFLOW_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIEN_RXOVFLWIE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_BUS_ERROR_INT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_TXBUSE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_BUS_ERROR_INT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_RXBUSE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_EMPTY_WMARK_INT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_EWMARK_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_FULL_WMARK_INT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_FWMARK_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_DONE_INT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_RXDONE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PACKET_PENDING_INT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_PKTPEND_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_ACTIVITY_INT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_RXACT_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_DONE_INT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_TXDONE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_ABORT_INT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_TXABORT_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_BUFFER_NOTAVAIL_INT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_RXBUFNA_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_FIFO_OVERFLOW_INT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHIRQ_RXOVFLW_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PACKET_BUFFER_COUNT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHSTAT_BUFCNT_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_ETHERNET_MODULE_BUSY_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHSTAT_ETHBUSY_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_BUSY_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHSTAT_TXBUSY_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_BUSY_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHSTAT_RXBUSY_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_OVERFLOW_COUNT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHRXOVFLOW_RXOVFLWCNT_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_FRAME_TX_OK_COUNT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHFRMTXOK_FRMTXOKCNT_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_SINGLE_COL_FRAME_COUNT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHSCOLFRM_SCOLFRMCNT_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MULTI_COL_FRAME_COUNT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHMCOLFRM_MCOLFRMCNT_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_FRAMES_RCVD_OK_COUNT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHFRMRXOK_FRMRXOKCNT_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_FCS_ERR_COUNT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHFCSERR_FCSERRCNT_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_ALIGNMENT_ERR_COUNT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _ETHALGNERR_ALGNERRCNT_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MII_RESET_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_SOFTRESET_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_SIMULATION_RESET_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_SIMRESET_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RESET_MCS_RX_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_RESETRMCS_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RESET_RX_FUNCTION_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_RESETRFUN_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RESET_MCS_TX_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_RESETTMCS_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RESET_TX_FUNCTION_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_RESETTFUN_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MAC_LOOPBACK_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_LOOPBACK_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TX_FLOW_CONTROL_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_TXPAUSE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RX_FLOW_CONTROL_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_RXPAUSE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PASS_ALL_RX_FRAMES_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_PASSALL_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MAC_RX_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG1_RXENABLE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_EXCESS_DEFER_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_EXCESSDFR_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_BACKPRESSURE_NOBACKOFF_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_BPNOBKOFF_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_NO_BACKOFF_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_NOBKOFF_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_LONG_PREAMBLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_LONGPRE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PURE_PREAMBLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_PUREPRE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_AUTO_DETECT_PAD_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_AUTOPAD_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_VLAN_PAD_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_VLANPAD_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_PAD_CRC_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_PADENABLE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_CRC_ENABLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_CRCENABLE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_DELAYED_CRC_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_DELAYCRC_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_HUGE_FRAME_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_HUGEFRM_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_LENGTH_CHECK_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_LENGTHCK_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_FULL_DUPLEX_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CFG2_FULLDPLX_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_BACK2BACK_IPGAP_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1IPGT_B2BIPKTGP_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_NON_BACK2BACK_IPGAP1_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1IPGR_NB2BIPKTGP1_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_NON_BACK2BACK_IPGAP2_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1IPGR_NB2BIPKTGP2_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_COL_WINDOW_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CLRT_CWINDOW_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RETX_MAX_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1CLRT_RETX_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MAX_FRAME_LENGTH_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MAXF_MACMAXF_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RESET_RMII_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SUPP_RESETRMII_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_SPEED_RMII_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SUPP_SPEEDRMII_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TEST_BACKPRESSURE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1TEST_TESTBP_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_TEST_PAUSE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1TEST_TESTPAUSE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_SHORTCUT_PAUSE_QUANTA_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1TEST_SHRTQNTA_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_RESET_MIIM_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MCFG_RESETMGMT_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MII_CLOCK_SEL_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MCFG_CLKSEL_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_NO_PREAMBLE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MCFG_NOPRE_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_SCAN_INCREMENT_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MCFG_SCANINC_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_SCAN_MODE_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MCMD_SCAN_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_READ_COMMAND_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MCMD_READ_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_PHY_ADDRESS_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MADR_PHYADDR_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_REGISTER_ADDRESS_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MADR_REGADDR_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_WRITE_DATA_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MWTD_MWTD_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_READ_DATA_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MRDD_MRDD_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_LINK_FAILED_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MIND_LINKFAIL_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_DATA_NOT_VALID_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MIND_NOTVALID_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_SCANNING_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MIND_SCAN_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_MIIM_BUSY_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1MIND_MIIMBUSY_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STATION_ADDRESS_OCTET_6_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SA0_STNADDR6_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STATION_ADDRESS_OCTET_5_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SA0_STNADDR5_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STATION_ADDRESS_OCTET_4_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SA1_STNADDR4_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STATION_ADDRESS_OCTET_3_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SA1_STNADDR3_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STATION_ADDRESS_OCTET_2_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SA2_STNADDR2_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _ETH_STATION_ADDRESS_OCTET_1_LEN(ETH_MODULE_ID i)
{
    switch (i) {
        case ETH_ID_0 :
            return _EMAC1SA2_STNADDR1_LENGTH;
        case ETH_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/eth_PauseTimer_Default.h"
#include "../templates/eth_Enable_Default.h"
#include "../templates/eth_StopInIdle_Default.h"
#include "../templates/eth_TransmitRTS_Default.h"
#include "../templates/eth_ReceiveEnable_Default.h"
#include "../templates/eth_AutoFlowControl_Default.h"
#include "../templates/eth_ManualFlowControl_Default.h"
#include "../templates/eth_RxBufferCountDecrement_Default.h"
#include "../templates/eth_ReceiveBufferSize_Default.h"
#include "../templates/eth_TxPacketDescriptorAddress_Default.h"
#include "../templates/eth_RxPacketDescriptorAddress_Default.h"
#include "../templates/eth_HashTable_Default.h"
#include "../templates/eth_PatternMatch_Default.h"
#include "../templates/eth_ReceiveFilters_Default.h"
#include "../templates/eth_ReceiveWmarks_Default.h"
#include "../templates/eth_Interrupt_Default.h"
#include "../templates/eth_EthernetControllerStatus_Default.h"
#include "../templates/eth_ReceiveOverflowCount_Default.h"
#include "../templates/eth_FramesTransmittedOK_Default.h"
#include "../templates/eth_CollisionCounts_Default.h"
#include "../templates/eth_FramexReceivedOK_Default.h"
#include "../templates/eth_FCSErrorCount_Default.h"
#include "../templates/eth_AlignmentErrorCount_Default.h"
#include "../templates/eth_MAC_Resets_Default.h"
#include "../templates/eth_MAC_Configuration_Default.h"
#include "../templates/eth_InterPacketGaps_Default.h"
#include "../templates/eth_CollisionWindow_Default.h"
#include "../templates/eth_RetransmissionMaximum_Default.h"
#include "../templates/eth_MaxFrameLength_Default.h"
#include "../templates/eth_RMII_Support_Default.h"
#include "../templates/eth_MAC_Testing_Default.h"
#include "../templates/eth_MIIM_Config_Default.h"
#include "../templates/eth_MIIMScanMode_Default.h"
#include "../templates/eth_MIIMReadWrite_Default.h"
#include "../templates/eth_MIIMAddresses_Default.h"
#include "../templates/eth_MIIWriteReadData_Default.h"
#include "../templates/eth_MIIM_Indicators_Default.h"
#include "../templates/eth_StationAddress_Default.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void PLIB_ETH_PauseTimerSet(ETH_MODULE_ID index, uint16_t PauseTimerValue)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_PauseTimerSet_Default(index, PauseTimerValue);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint16_t PLIB_ETH_PauseTimerGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_PauseTimerGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsPauseTimer(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsPauseTimer_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_Enable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_Enable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_Disable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_Disable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_IsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_IsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsEnable_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_StopInIdleEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_StopInIdleEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_StopInIdleDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_StopInIdleDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_StopInIdleIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_StopInIdleIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsStopInIdle(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsStopInIdle_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_TxRTSEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_TxRTSEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_TxRTSDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_TxRTSDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_TxRTSIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_TxRTSIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsTransmitRTS(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsTransmitRTS_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_RxEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_RxEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_RxDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_RxDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_RxIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_RxIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsRxEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsRxEnable_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_AutoFlowControlEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_AutoFlowControlEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_AutoFlowControlDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_AutoFlowControlDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_AutoFlowControlIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_AutoFlowControlIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsAutoFlowControl(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsAutoFlowControl_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_ManualFlowControlEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_ManualFlowControlEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_ManualFlowControlDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_ManualFlowControlDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ManualFlowControlIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ManualFlowControlIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsManualFlowControl(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsManualFlowControl_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_RxBufferCountDecrement(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_RxBufferCountDecrement_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsRxBufferCountDecrement(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsRxBufferCountDecrement_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint8_t PLIB_ETH_ReceiveBufferSizeGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ReceiveBufferSizeGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_ReceiveBufferSizeSet(ETH_MODULE_ID index, uint8_t ReceiveBufferSize)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_ReceiveBufferSizeSet_Default(index, ReceiveBufferSize);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsReceiveBufferSize(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsReceiveBufferSize_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_TxPacketDescAddrSet(ETH_MODULE_ID index, uint8_t* txPacketDescStartAddr)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_TxPacketDescAddrSet_Default(index, txPacketDescStartAddr);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint8_t* PLIB_ETH_TxPacketDescAddrGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_TxPacketDescAddrGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint8_t*)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsTxPacketDescriptorAddress(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsTxPacketDescriptorAddress_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_RxPacketDescAddrSet(ETH_MODULE_ID index, uint8_t* rxPacketDescStartAddr)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_RxPacketDescAddrSet_Default(index, rxPacketDescStartAddr);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint8_t* PLIB_ETH_RxPacketDescAddrGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_RxPacketDescAddrGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint8_t*)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsRxPacketDescriptorAddress(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsRxPacketDescriptorAddress_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_HashTableSet(ETH_MODULE_ID index, uint64_t hashTableValue)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_HashTableSet_Default(index, hashTableValue);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint32_t PLIB_ETH_HashTableGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_HashTableGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint32_t)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsHashTable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsHashTable_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_PatternMatchSet(ETH_MODULE_ID index, uint64_t patternMatchMaskValue)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_PatternMatchSet_Default(index, patternMatchMaskValue);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint64_t PLIB_ETH_PatternMatchGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_PatternMatchGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint64_t)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_PatternMatchChecksumSet(ETH_MODULE_ID index, uint16_t PatternMatchChecksumValue)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_PatternMatchChecksumSet_Default(index, PatternMatchChecksumValue);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint16_t PLIB_ETH_PatternMatchChecksumGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_PatternMatchChecksumGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_PatternMatchOffsetSet(ETH_MODULE_ID index, uint16_t PatternMatchOffsetValue)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_PatternMatchOffsetSet_Default(index, PatternMatchOffsetValue);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint16_t PLIB_ETH_PatternMatchOffsetGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_PatternMatchOffsetGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_PatternMatchModeSet(ETH_MODULE_ID index, ETH_PATTERN_MATCH_MODE modeSel)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_PatternMatchModeSet_Default(index, modeSel);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API ETH_PATTERN_MATCH_MODE PLIB_ETH_PatternMatchModeGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_PatternMatchModeGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (ETH_PATTERN_MATCH_MODE)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsPatternMatch(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsPatternMatch_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_ReceiveFilterEnable(ETH_MODULE_ID index, ETH_RECEIVE_FILTER filter)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_ReceiveFilterEnable_Default(index, filter);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_ReceiveFilterDisable(ETH_MODULE_ID index, ETH_RECEIVE_FILTER filter)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_ReceiveFilterDisable_Default(index, filter);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ReceiveFilterIsEnable(ETH_MODULE_ID index, ETH_RECEIVE_FILTER filter)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ReceiveFilterIsEnable_Default(index, filter);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsReceiveFilters(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsReceiveFilters_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_RxFullWmarkSet(ETH_MODULE_ID index, uint8_t watermarkValue)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_RxFullWmarkSet_Default(index, watermarkValue);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint8_t PLIB_ETH_RxFullWmarkGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_RxFullWmarkGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_RxEmptyWmarkSet(ETH_MODULE_ID index, uint8_t watermarkValue)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_RxEmptyWmarkSet_Default(index, watermarkValue);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint8_t PLIB_ETH_RxEmptyWmarkGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_RxEmptyWmarkGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsReceiveWmarks(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsReceiveWmarks_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_InterruptSourceEnable(ETH_MODULE_ID index, ETH_INTERRUPT_SOURCES intmask)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_InterruptSourceEnable_Default(index, intmask);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_InterruptSourceDisable(ETH_MODULE_ID index, ETH_INTERRUPT_SOURCES intmask)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_InterruptSourceDisable_Default(index, intmask);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_InterruptSourceIsEnabled(ETH_MODULE_ID index, ETH_INTERRUPT_SOURCES intmask)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_InterruptSourceIsEnabled_Default(index, intmask);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API ETH_INTERRUPT_SOURCES PLIB_ETH_InterruptSourcesGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_InterruptSourcesGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (ETH_INTERRUPT_SOURCES)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_InterruptSet(ETH_MODULE_ID index, ETH_INTERRUPT_SOURCES intmask)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_InterruptSet_Default(index, intmask);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_InterruptClear(ETH_MODULE_ID index, ETH_INTERRUPT_SOURCES intmask)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_InterruptClear_Default(index, intmask);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API ETH_INTERRUPT_SOURCES PLIB_ETH_InterruptsGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_InterruptsGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (ETH_INTERRUPT_SOURCES)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_InterruptStatusGet(ETH_MODULE_ID index, ETH_INTERRUPT_SOURCES intmask)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_InterruptStatusGet_Default(index, intmask);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsInterrupt(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsInterrupt_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint8_t PLIB_ETH_RxPacketCountGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_RxPacketCountGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_EthernetIsBusy(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_EthernetIsBusy_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_TransmitIsBusy(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_TransmitIsBusy_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ReceiveIsBusy(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ReceiveIsBusy_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsEthernetControllerStatus(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsEthernetControllerStatus_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_RxOverflowCountClear(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_RxOverflowCountClear_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint16_t PLIB_ETH_RxOverflowCountGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_RxOverflowCountGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsReceiveOverflowCount(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsReceiveOverflowCount_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_FramesTxdOkCountClear(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_FramesTxdOkCountClear_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint16_t PLIB_ETH_FramesTxdOkCountGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_FramesTxdOkCountGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsFramesTransmittedOK(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsFramesTransmittedOK_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_SingleCollisionCountClear(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_SingleCollisionCountClear_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint16_t PLIB_ETH_SingleCollisionCountGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_SingleCollisionCountGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_MultipleCollisionCountClear(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MultipleCollisionCountClear_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint16_t PLIB_ETH_MultipleCollisionCountGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_MultipleCollisionCountGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsCollisionCounts(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsCollisionCounts_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_FramesRxdOkCountClear(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_FramesRxdOkCountClear_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint16_t PLIB_ETH_FramesRxdOkCountGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_FramesRxdOkCountGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsFramexReceivedOK(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsFramexReceivedOK_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_FCSErrorCountClear(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_FCSErrorCountClear_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint16_t PLIB_ETH_FCSErrorCountGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_FCSErrorCountGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsFCSErrorCount(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsFCSErrorCount_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_AlignErrorCountClear(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_AlignErrorCountClear_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint16_t PLIB_ETH_AlignErrorCountGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_AlignErrorCountGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsAlignmentErrorCount(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsAlignmentErrorCount_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_MIIResetEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MIIResetEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_MIIResetDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MIIResetDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_MIIResetIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_MIIResetIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_SimResetEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_SimResetEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_SimResetDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_SimResetDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_SimResetIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_SimResetIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_MCSRxResetEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MCSRxResetEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_MCSRxResetDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MCSRxResetDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_MCSRxResetIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_MCSRxResetIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_RxFuncResetEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_RxFuncResetEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_RxFuncResetDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_RxFuncResetDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_RxFuncResetIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_RxFuncResetIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_MCSTxResetEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MCSTxResetEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_MCSTxResetDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MCSTxResetDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_MCSTxResetIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_MCSTxResetIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_TxFuncResetEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_TxFuncResetEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_TxFuncResetDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_TxFuncResetDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_TxFuncResetIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_TxFuncResetIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMAC_Resets(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsMAC_Resets_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_LoopbackEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_LoopbackEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_LoopbackDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_LoopbackDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_LoopbackIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_LoopbackIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_TxPauseEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_TxPauseEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_TxPauseDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_TxPauseDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_TxPauseIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_TxPauseIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_RxPauseEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_RxPauseEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_RxPauseDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_RxPauseDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_RxPauseIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_RxPauseIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_PassAllEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_PassAllEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_PassAllDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_PassAllDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_PassAllIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_PassAllIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_ReceiveEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_ReceiveEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_ReceiveDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_ReceiveDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ReceiveIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ReceiveIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_ExcessDeferEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_ExcessDeferEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_ExcessDeferDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_ExcessDeferDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExcessDeferIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExcessDeferIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_BackPresNoBackoffEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_BackPresNoBackoffEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_BackPresNoBackoffDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_BackPresNoBackoffDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_BackPresNoBackoffIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_BackPresNoBackoffIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_NoBackoffEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_NoBackoffEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_NoBackoffDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_NoBackoffDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_NoBackoffIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_NoBackoffIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_LongPreambleEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_LongPreambleEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_LongPreambleDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_LongPreambleDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_LongPreambleIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_LongPreambleIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_PurePreambleEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_PurePreambleEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_PurePreambleDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_PurePreambleDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_PurePreambleIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_PurePreambleIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API ETH_AUTOPAD_OPTION PLIB_ETH_AutoDetectPadGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_AutoDetectPadGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (ETH_AUTOPAD_OPTION)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_AutoDetectPadSet(ETH_MODULE_ID index, ETH_AUTOPAD_OPTION option)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_AutoDetectPadSet_Default(index, option);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_AutoDetectPadClear(ETH_MODULE_ID index, ETH_AUTOPAD_OPTION option)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_AutoDetectPadClear_Default(index, option);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_CRCEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_CRCEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_CRCDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_CRCDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_CRCIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_CRCIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_DelayedCRCEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_DelayedCRCEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_DelayedCRCDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_DelayedCRCDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_DelayedCRCIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_DelayedCRCIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_HugeFrameEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_HugeFrameEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_HugeFrameDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_HugeFrameDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_HugeFrameIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_HugeFrameIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_FrameLengthCheckEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_FrameLengthCheckEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_FrameLengthCheckDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_FrameLengthCheckDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_FrameLengthCheckIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_FrameLengthCheckIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_FullDuplexEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_FullDuplexEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_FullDuplexDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_FullDuplexDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_FullDuplexIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_FullDuplexIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMAC_Configuration(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsMAC_Configuration_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint8_t PLIB_ETH_BackToBackIPGGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_BackToBackIPGGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_BackToBackIPGSet(ETH_MODULE_ID index, uint8_t backToBackIPGValue)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_BackToBackIPGSet_Default(index, backToBackIPGValue);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint8_t PLIB_ETH_NonBackToBackIPG1Get(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_NonBackToBackIPG1Get_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_NonBackToBackIPG1Set(ETH_MODULE_ID index, uint8_t nonBackToBackIPGValue)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_NonBackToBackIPG1Set_Default(index, nonBackToBackIPGValue);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint8_t PLIB_ETH_NonBackToBackIPG2Get(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_NonBackToBackIPG2Get_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_NonBackToBackIPG2Set(ETH_MODULE_ID index, uint8_t nonBackToBackIPGValue)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_NonBackToBackIPG2Set_Default(index, nonBackToBackIPGValue);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsInterPacketGaps(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsInterPacketGaps_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint8_t PLIB_ETH_CollisionWindowGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_CollisionWindowGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_CollisionWindowSet(ETH_MODULE_ID index, uint8_t collisionWindowValue)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_CollisionWindowSet_Default(index, collisionWindowValue);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsCollisionWindow(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsCollisionWindow_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint8_t PLIB_ETH_ReTxMaxGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ReTxMaxGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_ReTxMaxSet(ETH_MODULE_ID index, uint16_t retransmitMax)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_ReTxMaxSet_Default(index, retransmitMax);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsRetransmissionMaximum(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsRetransmissionMaximum_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint16_t PLIB_ETH_MaxFrameLengthGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_MaxFrameLengthGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_MaxFrameLengthSet(ETH_MODULE_ID index, uint16_t MaxFrameLength)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MaxFrameLengthSet_Default(index, MaxFrameLength);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMaxFrameLength(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsMaxFrameLength_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_RMIIResetEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_RMIIResetEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_RMIIResetDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_RMIIResetDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_RMIIResetIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_RMIIResetIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API ETH_RMII_SPEED PLIB_ETH_RMIISpeedGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_RMIISpeedGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (ETH_RMII_SPEED)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_RMIISpeedSet(ETH_MODULE_ID index, ETH_RMII_SPEED RMIISpeed)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_RMIISpeedSet_Default(index, RMIISpeed);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsRMII_Support(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsRMII_Support_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_TestBackPressEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_TestBackPressEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_TestBackPressDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_TestBackPressDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_TestBackPressIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_TestBackPressIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_TestPauseEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_TestPauseEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_TestPauseDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_TestPauseDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_TestPauseIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_TestPauseIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_ShortcutQuantaEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_ShortcutQuantaEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_ShortcutQuantaDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_ShortcutQuantaDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ShortcutQuantaIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ShortcutQuantaIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMAC_Testing(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsMAC_Testing_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_MIIMResetEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MIIMResetEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_MIIMResetDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MIIMResetDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_MIIMResetIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_MIIMResetIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API ETH_MIIM_CLK PLIB_ETH_MIIMClockGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_MIIMClockGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (ETH_MIIM_CLK)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_MIIMClockSet(ETH_MODULE_ID index, ETH_MIIM_CLK MIIMClock)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MIIMClockSet_Default(index, MIIMClock);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_MIIMNoPreEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MIIMNoPreEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_MIIMNoPreDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MIIMNoPreDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_MIIMNoPreIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_MIIMNoPreIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_MIIMScanIncrEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MIIMScanIncrEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_MIIMScanIncrDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MIIMScanIncrDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_MIIMScanIncrIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_MIIMScanIncrIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMIIM_Config(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsMIIM_Config_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_MIIMScanModeEnable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MIIMScanModeEnable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_MIIMScanModeDisable(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MIIMScanModeDisable_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_MIIMScanModeIsEnabled(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_MIIMScanModeIsEnabled_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMIIMScanMode(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsMIIMScanMode_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_MIIMReadStart(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MIIMReadStart_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_ETH_MIIMWriteStart(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MIIMWriteStart_Default(index);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMIIMReadWrite(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsMIIMReadWrite_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint8_t PLIB_ETH_PHYAddressGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_PHYAddressGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_PHYAddressSet(ETH_MODULE_ID index, uint8_t phyAddr)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_PHYAddressSet_Default(index, phyAddr);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint8_t PLIB_ETH_RegisterAddressGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_RegisterAddressGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_RegisterAddressSet(ETH_MODULE_ID index, uint8_t regAddr)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_RegisterAddressSet_Default(index, regAddr);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMIIMAddresses(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsMIIMAddresses_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_MIIMWriteDataSet(ETH_MODULE_ID index, uint16_t writeData)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_MIIMWriteDataSet_Default(index, writeData);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint16_t PLIB_ETH_MIIMReadDataGet(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_MIIMReadDataGet_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint16_t)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMIIWriteReadData(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsMIIWriteReadData_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_LinkHasFailed(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_LinkHasFailed_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_DataNotValid(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_DataNotValid_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_MIIMIsScanning(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_MIIMIsScanning_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_MIIMIsBusy(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_MIIMIsBusy_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsMIIM_Indicators(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsMIIM_Indicators_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint8_t PLIB_ETH_StationAddressGet(ETH_MODULE_ID index, uint8_t which)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_StationAddressGet_Default(index, which);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API void PLIB_ETH_StationAddressSet(ETH_MODULE_ID index, uint8_t which, uint8_t stationAddress)
{
    switch (index) {
        case ETH_ID_0 :
            ETH_StationAddressSet_Default(index, which, stationAddress);
            break;
        case ETH_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_ETH_ExistsStationAddress(ETH_MODULE_ID index)
{
    switch (index) {
        case ETH_ID_0 :
            return ETH_ExistsStationAddress_Default(index);
        case ETH_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

#endif
