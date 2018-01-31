/* Created by plibgen $Revision: 1.31 $ */

#ifndef _RESET_P32MZ2048EFM144_H
#define _RESET_P32MZ2048EFM144_H

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

    RESET_ID_0 = 0,
    RESET_NUMBER_OF_MODULES

} RESET_MODULE_ID;

typedef enum {

    RESET_REASON_NONE = 0x00000000,
    RESET_REASON_POWERON = 0x00000003,
    RESET_REASON_BROWNOUT = 0x00000002,
    RESET_REASON_WDT_TIMEOUT = 0x00000010,
    RESET_REASON_DMT_TIMEOUT = 0x00000020,
    RESET_REASON_SOFTWARE = 0x00000040,
    RESET_REASON_MCLR = 0x00000080,
    RESET_REASON_CONFIG_MISMATCH = 0x00000200,
    RESET_REASON_ALL = 0x000002F3

} RESET_REASON;

typedef enum {

    PRIMARY_CONFIG_REG_READ_ERROR = 0x08000000,
    PRIMARY_AND_ALTERNATIVE_CONFIG_REG_READ_ERROR = 0x04000000,
    NO_CONFIG_REG_READ_ERROR = 0x00000000

} RESET_CONFIG_REG_READ_ERROR;

typedef enum {

    WDTO_NMI = 0x01000000,
    DMTO_NMI = 0x02000000,
    SOFTWARE_NMI = 0x00800000,
    GLOBAL_NMI = 0x00080000,
    CF_NMI = 0x00020000,
    WDTS_NMI = 0x00010000

} RESET_NMI_REASON;

PLIB_INLINE SFR_TYPE* _RESET_RESET_PRIMARY_CONFIG_REG_READ_ERROR_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &RCON;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _RESET_RESET_BOTH_CONFIG_REG_READ_ERROR_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &RCON;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _RESET_RESET_REASON_STATUS_CMR_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &RCON;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _RESET_RESET_REASON_STATUS_EXTR_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &RCON;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _RESET_RESET_REASON_STATUS_SWR_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &RCON;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _RESET_RESET_REASON_STATUS_DMT0_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &RCON;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _RESET_RESET_REASON_STATUS_WDT0_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &RCON;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _RESET_RESET_REASON_STATUS_BOR_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &RCON;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _RESET_RESET_REASON_STATUS_POR_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &RCON;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _RESET_SLEEP_STATUS_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &RCON;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _RESET_IDLE_STATUS_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &RCON;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _RESET_SOFTWARE_RESET_TRIGGER_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &RSWRST;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _RESET_DMTO_NMI_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &RNMICON;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _RESET_WDTO_NMI_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &RNMICON;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _RESET_SOFTWARE_NMI_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &RNMICON;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _RESET_GLOBAL_NMI_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &RNMICON;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _RESET_CLOCK_FAILURE_NMI_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &RNMICON;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _RESET_WDTO_IN_SLEEP_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &RNMICON;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _RESET_NMI_COUNT_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &RNMICON;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _RESET_REGISTER_LOCK_VREG(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return &SYSKEY;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_PRIMARY_CONFIG_REG_READ_ERROR_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_BCFGERR_MASK;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_BOTH_CONFIG_REG_READ_ERROR_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_BCFGFAIL_MASK;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_CMR_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_CMR_MASK;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_EXTR_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_EXTR_MASK;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_SWR_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_SWR_MASK;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_DMT0_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_DMTO_MASK;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_WDT0_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_WDTO_MASK;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_BOR_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_BOR_MASK;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_POR_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_POR_MASK;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_SLEEP_STATUS_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_SLEEP_MASK;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_IDLE_STATUS_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_IDLE_MASK;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_SOFTWARE_RESET_TRIGGER_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RSWRST_SWRST_MASK;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_DMTO_NMI_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_DMTO_MASK;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_WDTO_NMI_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_WDTO_MASK;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_SOFTWARE_NMI_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_SWNMI_MASK;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_GLOBAL_NMI_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_GNMI_MASK;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_CLOCK_FAILURE_NMI_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_CF_MASK;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_WDTO_IN_SLEEP_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_WDTS_MASK;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_NMI_COUNT_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_NMICNT_MASK;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_REGISTER_LOCK_MASK(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return (SFR_DATA)-1;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_PRIMARY_CONFIG_REG_READ_ERROR_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_BCFGERR_POSITION;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_BOTH_CONFIG_REG_READ_ERROR_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_BCFGFAIL_POSITION;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_CMR_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_CMR_POSITION;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_EXTR_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_EXTR_POSITION;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_SWR_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_SWR_POSITION;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_DMT0_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_DMTO_POSITION;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_WDT0_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_WDTO_POSITION;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_BOR_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_BOR_POSITION;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_POR_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_POR_POSITION;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_SLEEP_STATUS_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_SLEEP_POSITION;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_IDLE_STATUS_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_IDLE_POSITION;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_SOFTWARE_RESET_TRIGGER_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RSWRST_SWRST_POSITION;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_DMTO_NMI_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_DMTO_POSITION;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_WDTO_NMI_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_WDTO_POSITION;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_SOFTWARE_NMI_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_SWNMI_POSITION;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_GLOBAL_NMI_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_GNMI_POSITION;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_CLOCK_FAILURE_NMI_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_CF_POSITION;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_WDTO_IN_SLEEP_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_WDTS_POSITION;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_NMI_COUNT_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_NMICNT_POSITION;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_REGISTER_LOCK_POS(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return (SFR_DATA)0;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_PRIMARY_CONFIG_REG_READ_ERROR_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_BCFGERR_LENGTH;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_BOTH_CONFIG_REG_READ_ERROR_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_BCFGFAIL_LENGTH;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_CMR_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_CMR_LENGTH;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_EXTR_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_EXTR_LENGTH;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_SWR_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_SWR_LENGTH;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_DMT0_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_DMTO_LENGTH;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_WDT0_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_WDTO_LENGTH;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_BOR_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_BOR_LENGTH;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_RESET_REASON_STATUS_POR_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_POR_LENGTH;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_SLEEP_STATUS_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_SLEEP_LENGTH;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_IDLE_STATUS_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RCON_IDLE_LENGTH;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_SOFTWARE_RESET_TRIGGER_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RSWRST_SWRST_LENGTH;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_DMTO_NMI_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_DMTO_LENGTH;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_WDTO_NMI_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_WDTO_LENGTH;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_SOFTWARE_NMI_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_SWNMI_LENGTH;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_GLOBAL_NMI_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_GNMI_LENGTH;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_CLOCK_FAILURE_NMI_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_CF_LENGTH;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_WDTO_IN_SLEEP_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_WDTS_LENGTH;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_NMI_COUNT_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return _RNMICON_NMICNT_LENGTH;
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _RESET_REGISTER_LOCK_LEN(RESET_MODULE_ID i)
{
    switch (i) {
        case RESET_ID_0 :
            return (SFR_DATA)sizeof(SFR_DATA);
        case RESET_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/reset_ResetReasonStatus_MZ.h"
#include "../templates/reset_SoftwareResetTrigger_Default.h"
#include "../templates/reset_ConfigRegReadError_Default.h"
#include "../templates/reset_NmiControl_MZ_1.h"
#include "../templates/reset_WdtoInSleep_Default.h"
#include "../templates/reset_NmiCounter_Default.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API bool PLIB_RESET_ExistsResetReasonStatus(RESET_MODULE_ID index)
{
    switch (index) {
        case RESET_ID_0 :
            return RESET_ExistsResetReasonStatus_MZ(index);
        case RESET_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API RESET_REASON PLIB_RESET_ReasonGet(RESET_MODULE_ID index)
{
    switch (index) {
        case RESET_ID_0 :
            return RESET_ReasonGet_MZ(index);
        case RESET_NUMBER_OF_MODULES :
        default :
            return (RESET_REASON)0;
    }
}

PLIB_INLINE_API void PLIB_RESET_ReasonClear(RESET_MODULE_ID index, RESET_REASON reason)
{
    switch (index) {
        case RESET_ID_0 :
            RESET_ReasonClear_MZ(index, reason);
            break;
        case RESET_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_RESET_ExistsSoftwareResetTrigger(RESET_MODULE_ID index)
{
    switch (index) {
        case RESET_ID_0 :
            return RESET_ExistsSoftwareResetTrigger_Default(index);
        case RESET_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_RESET_SoftwareResetEnable(RESET_MODULE_ID index)
{
    switch (index) {
        case RESET_ID_0 :
            RESET_SoftwareResetEnable_Default(index);
            break;
        case RESET_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_RESET_ExistsConfigRegReadError(RESET_MODULE_ID index)
{
    switch (index) {
        case RESET_ID_0 :
            return RESET_ExistsConfigRegReadError_Default(index);
        case RESET_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API RESET_CONFIG_REG_READ_ERROR PLIB_RESET_ConfigRegReadErrorGet(RESET_MODULE_ID index)
{
    switch (index) {
        case RESET_ID_0 :
            return RESET_ConfigRegReadErrorGet_Default(index);
        case RESET_NUMBER_OF_MODULES :
        default :
            return (RESET_CONFIG_REG_READ_ERROR)0;
    }
}

PLIB_INLINE_API bool PLIB_RESET_ExistsNmiControl(RESET_MODULE_ID index)
{
    switch (index) {
        case RESET_ID_0 :
            return RESET_ExistsNmiControl_MZ_1(index);
        case RESET_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API RESET_NMI_REASON PLIB_RESET_NmiReasonGet(RESET_MODULE_ID index)
{
    switch (index) {
        case RESET_ID_0 :
            return RESET_NmiReasonGet_MZ_1(index);
        case RESET_NUMBER_OF_MODULES :
        default :
            return (RESET_NMI_REASON)0;
    }
}

PLIB_INLINE_API void PLIB_RESET_NmiEventTrigger(RESET_MODULE_ID index, RESET_NMI_REASON nmi_reason)
{
    switch (index) {
        case RESET_ID_0 :
            RESET_NmiEventTrigger_MZ_1(index, nmi_reason);
            break;
        case RESET_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_RESET_NmiEventClear(RESET_MODULE_ID index, RESET_NMI_REASON nmi_reason)
{
    switch (index) {
        case RESET_ID_0 :
            RESET_NmiEventClear_MZ_1(index, nmi_reason);
            break;
        case RESET_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_RESET_ExistsWdtoInSleep(RESET_MODULE_ID index)
{
    switch (index) {
        case RESET_ID_0 :
            return RESET_ExistsWdtoInSleep_Default(index);
        case RESET_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_RESET_WdtTimeOutHasOccurredInSleep(RESET_MODULE_ID index)
{
    switch (index) {
        case RESET_ID_0 :
            return RESET_WdtTimeOutHasOccurredInSleep_Default(index);
        case RESET_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_RESET_ExistsNmiCounter(RESET_MODULE_ID index)
{
    switch (index) {
        case RESET_ID_0 :
            return RESET_ExistsNmiCounter_Default(index);
        case RESET_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_RESET_NmiCounterValueSet(RESET_MODULE_ID index, RESET_NMI_COUNT_TYPE nmi_count)
{
    switch (index) {
        case RESET_ID_0 :
            RESET_NmiCounterValueSet_Default(index, nmi_count);
            break;
        case RESET_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API RESET_NMI_COUNT_TYPE PLIB_RESET_NmiCounterValueGet(RESET_MODULE_ID index)
{
    switch (index) {
        case RESET_ID_0 :
            return RESET_NmiCounterValueGet_Default(index);
        case RESET_NUMBER_OF_MODULES :
        default :
            return (RESET_NMI_COUNT_TYPE)0;
    }
}

#endif
