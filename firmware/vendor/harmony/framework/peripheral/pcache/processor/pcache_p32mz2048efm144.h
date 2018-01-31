/* Created by plibgen $Revision: 1.31 $ */

#ifndef _PCACHE_P32MZ2048EFM144_H
#define _PCACHE_P32MZ2048EFM144_H

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

    PCACHE_ID_0 = 0,
    PCACHE_NUMBER_OF_MODULES

} PCACHE_MODULE_ID;

typedef enum {

    PLIB_PCACHE_PREFETCH_DISABLE = 0x00,
    PLIB_PCACHE_PREFETCH_ENABLE_CPU_INST = 0x01,
    PLIB_PCACHE_PREFETCH_ENABLE_CPU_INST_DATA = 0x02,
    PLIB_PCACHE_PREFETCH_ENABLE_ALL = 0x03

} PLIB_PCACHE_PREFETCH_ENABLE;

typedef enum {

    PLIB_PCACHE_DATA_ENABLE_NONE

} PLIB_PCACHE_DATA_ENABLE;

PLIB_INLINE SFR_TYPE* _PCACHE_WAIT_STATE_VREG(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return &PRECON;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PCACHE_PREFETCH_ENABLE_VREG(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return &PRECON;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PCACHE_FLASH_SEC_INT_VREG(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return &PRECON;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PCACHE_FLASH_SEC_COUNT_VREG(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return &PRESTAT;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PCACHE_FLASH_SEC_STATUS_VREG(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return &PRESTAT;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _PCACHE_FLASH_DED_STATUS_VREG(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return &PRESTAT;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_DATA _PCACHE_WAIT_STATE_MASK(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return _PRECON_PFMWS_MASK;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PCACHE_PREFETCH_ENABLE_MASK(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return _PRECON_PREFEN_MASK;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PCACHE_FLASH_SEC_INT_MASK(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return _PRECON_PFMSECEN_MASK;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PCACHE_FLASH_SEC_COUNT_MASK(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return _PRESTAT_PFMSECCNT_MASK;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PCACHE_FLASH_SEC_STATUS_MASK(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return _PRESTAT_PFMSEC_MASK;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PCACHE_FLASH_DED_STATUS_MASK(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return _PRESTAT_PFMDED_MASK;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PCACHE_WAIT_STATE_POS(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return _PRECON_PFMWS_POSITION;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PCACHE_PREFETCH_ENABLE_POS(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return _PRECON_PREFEN_POSITION;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PCACHE_FLASH_SEC_INT_POS(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return _PRECON_PFMSECEN_POSITION;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PCACHE_FLASH_SEC_COUNT_POS(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return _PRESTAT_PFMSECCNT_POSITION;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PCACHE_FLASH_SEC_STATUS_POS(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return _PRESTAT_PFMSEC_POSITION;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PCACHE_FLASH_DED_STATUS_POS(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return _PRESTAT_PFMDED_POSITION;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PCACHE_WAIT_STATE_LEN(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return _PRECON_PFMWS_LENGTH;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PCACHE_PREFETCH_ENABLE_LEN(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return _PRECON_PREFEN_LENGTH;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PCACHE_FLASH_SEC_INT_LEN(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return _PRECON_PFMSECEN_LENGTH;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PCACHE_FLASH_SEC_COUNT_LEN(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return _PRESTAT_PFMSECCNT_LENGTH;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PCACHE_FLASH_SEC_STATUS_LEN(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return _PRESTAT_PFMSEC_LENGTH;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _PCACHE_FLASH_DED_STATUS_LEN(PCACHE_MODULE_ID i)
{
    switch (i) {
        case PCACHE_ID_0 :
            return _PRESTAT_PFMDED_LENGTH;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/pcache_WaitState_Default.h"
#include "../templates/pcache_PrefetchEnable_Default.h"
#include "../templates/pcache_DataCacheEnable_Unsupported.h"
#include "../templates/pcache_FlashSECInt_Default.h"
#include "../templates/pcache_FlashDEDStatus_Default.h"
#include "../templates/pcache_FlashSECStatus_Default.h"
#include "../templates/pcache_FlashSECCount_Default.h"
#include "../templates/pcache_InvalidateOnPFMProgram_Unsupported.h"
#include "../templates/pcache_CacheLineSelect_Unsupported.h"
#include "../templates/pcache_CacheLineType_Unsupported.h"
#include "../templates/pcache_CacheLineLock_Unsupported.h"
#include "../templates/pcache_CacheLineValid_Unsupported.h"
#include "../templates/pcache_CacheLineAddr_Unsupported.h"
#include "../templates/pcache_CacheLineFlashType_Unsupported.h"
#include "../templates/pcache_CacheLineMask_Unsupported.h"
#include "../templates/pcache_Word_Unsupported.h"
#include "../templates/pcache_LeastRecentlyUsedState_Unsupported.h"
#include "../templates/pcache_CacheHit_Unsupported.h"
#include "../templates/pcache_CacheMiss_Unsupported.h"
#include "../templates/pcache_PrefetchAbort_Unsupported.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API bool PLIB_PCACHE_ExistsWaitState(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsWaitState_Default(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_PCACHE_WaitStateSet(PCACHE_MODULE_ID index, uint32_t clocks)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_WaitStateSet_Default(index, clocks);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint32_t PLIB_PCACHE_WaitStateGet(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_WaitStateGet_Default(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (uint32_t)0;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsPrefetchEnable(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsPrefetchEnable_Default(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_PCACHE_PrefetchEnableSet(PCACHE_MODULE_ID index, PLIB_PCACHE_PREFETCH_ENABLE region)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_PrefetchEnableSet_Default(index, region);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API PLIB_PCACHE_PREFETCH_ENABLE PLIB_PCACHE_PrefetchEnableGet(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_PrefetchEnableGet_Default(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (PLIB_PCACHE_PREFETCH_ENABLE)0;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsDataCacheEnable(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsDataCacheEnable_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_DataCacheEnableSet(PCACHE_MODULE_ID index, PLIB_PCACHE_DATA_ENABLE dcache_en)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_DataCacheEnableSet_Unsupported(index, dcache_en);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API PLIB_PCACHE_DATA_ENABLE _PLIB_UNSUPPORTED PLIB_PCACHE_DataCacheEnableGet(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_DataCacheEnableGet_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (PLIB_PCACHE_DATA_ENABLE)0;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsFlashSECInt(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsFlashSECInt_Default(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_PCACHE_FlashSECIntEnable(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_FlashSECIntEnable_Default(index);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_PCACHE_FlashSECIntDisable(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_FlashSECIntDisable_Default(index);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsFlashDEDStatus(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsFlashDEDStatus_Default(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_FlashDEDStatusGet(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_FlashDEDStatusGet_Default(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_PCACHE_FlashDEDStatusClear(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_FlashDEDStatusClear_Default(index);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsFlashSECStatus(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsFlashSECStatus_Default(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_FlashSECStatusGet(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_FlashSECStatusGet_Default(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_PCACHE_FlashSECStatusSet(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_FlashSECStatusSet_Default(index);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_PCACHE_FlashSECStatusClear(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_FlashSECStatusClear_Default(index);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsFlashSECCount(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsFlashSECCount_Default(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_PCACHE_FlashSECCountSet(PCACHE_MODULE_ID index, uint8_t count)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_FlashSECCountSet_Default(index, count);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint8_t PLIB_PCACHE_FlashSECCountGet(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_FlashSECCountGet_Default(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (uint8_t)0;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsInvalidateOnPFMProgram(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsInvalidateOnPFMProgram_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_InvalidateOnPFMProgramAll(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_InvalidateOnPFMProgramAll_Unsupported(index);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_InvalidateOnPFMProgramUnlocked(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_InvalidateOnPFMProgramUnlocked_Unsupported(index);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCacheLine(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsCacheLine_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_CacheLineSelect(PCACHE_MODULE_ID index, uint32_t cache_line)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_CacheLineSelect_Unsupported(index, cache_line);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_CacheLineDeselect(PCACHE_MODULE_ID index, uint32_t cache_line)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_CacheLineDeselect_Unsupported(index, cache_line);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_CacheLineData(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_CacheLineData_Unsupported(index);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_CacheLineInst(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_CacheLineInst_Unsupported(index);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PCACHE_CacheLineIsInst(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_CacheLineIsInst_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCacheLineType(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsCacheLineType_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_CacheLineLock(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_CacheLineLock_Unsupported(index);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_CacheLineUnlock(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_CacheLineUnlock_Unsupported(index);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PCACHE_CacheLineIsLocked(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_CacheLineIsLocked_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCacheLineLock(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsCacheLineLock_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_CacheLineValid(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_CacheLineValid_Unsupported(index);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_CacheLineInvalid(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_CacheLineInvalid_Unsupported(index);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PCACHE_CacheLineIsValid(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_CacheLineIsValid_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCacheLineValid(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsCacheLineValid_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_CacheLineAddrSet(PCACHE_MODULE_ID index, uint32_t addr)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_CacheLineAddrSet_Unsupported(index, addr);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_PCACHE_CacheLineAddrGet(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_CacheLineAddrGet_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (uint32_t)0;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCacheLineAddr(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsCacheLineAddr_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_CacheLineFlashTypeBoot(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_CacheLineFlashTypeBoot_Unsupported(index);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_CacheLineFlashTypeInst(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_CacheLineFlashTypeInst_Unsupported(index);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PCACHE_CacheLineFlashTypeIsInst(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_CacheLineFlashTypeIsInst_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCacheLineFlashType(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsCacheLineFlashType_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_CacheLineMaskSet(PCACHE_MODULE_ID index, uint32_t mask)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_CacheLineMaskSet_Unsupported(index, mask);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_PCACHE_CacheLineMaskGet(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_CacheLineMaskGet_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (uint32_t)0;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCacheLineMask(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsCacheLineMask_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_PCACHE_WordRead(PCACHE_MODULE_ID index, uint32_t word)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_WordRead_Unsupported(index, word);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (uint32_t)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_WordWrite(PCACHE_MODULE_ID index, uint32_t word, uint32_t data)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_WordWrite_Unsupported(index, word, data);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsWord(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsWord_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_PCACHE_LeastRecentlyUsedStateRead(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_LeastRecentlyUsedStateRead_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (uint32_t)0;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsLeastRecentlyUsedState(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsLeastRecentlyUsedState_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_PCACHE_CacheHitRead(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_CacheHitRead_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (uint32_t)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_CacheHitWrite(PCACHE_MODULE_ID index, uint32_t data)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_CacheHitWrite_Unsupported(index, data);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCacheHit(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsCacheHit_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_PCACHE_CacheMissRead(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_CacheMissRead_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (uint32_t)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_CacheMissWrite(PCACHE_MODULE_ID index, uint32_t data)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_CacheMissWrite_Unsupported(index, data);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsCacheMiss(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsCacheMiss_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_PCACHE_PrefetchAbortRead(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_PrefetchAbortRead_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (uint32_t)0;
    }
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PCACHE_PrefetchAbortWrite(PCACHE_MODULE_ID index, uint32_t data)
{
    switch (index) {
        case PCACHE_ID_0 :
            PCACHE_PrefetchAbortWrite_Unsupported(index, data);
            break;
        case PCACHE_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_PCACHE_ExistsPrefetchAbort(PCACHE_MODULE_ID index)
{
    switch (index) {
        case PCACHE_ID_0 :
            return PCACHE_ExistsPrefetchAbort_Unsupported(index);
        case PCACHE_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

#endif
