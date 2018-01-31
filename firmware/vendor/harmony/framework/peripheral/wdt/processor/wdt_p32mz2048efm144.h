/* Created by plibgen $Revision: 1.31 $ */

#ifndef _WDT_P32MZ2048EFM144_H
#define _WDT_P32MZ2048EFM144_H

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

    WDT_ID_0 = 0,
    WDT_NUMBER_OF_MODULES

} WDT_MODULE_ID;

PLIB_INLINE SFR_TYPE* _WDT_WINDOW_ENABLE_VREG(WDT_MODULE_ID i)
{
    switch (i) {
        case WDT_ID_0 :
            return &WDTCON;
        case WDT_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _WDT_POSTSCALER_VALUE_VREG(WDT_MODULE_ID i)
{
    switch (i) {
        case WDT_ID_0 :
            return &WDTCON;
        case WDT_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _WDT_ENABLE_CONTROL_VREG(WDT_MODULE_ID i)
{
    switch (i) {
        case WDT_ID_0 :
            return &WDTCON;
        case WDT_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_TYPE* _WDT_TIMER_CLEAR_KEY_VREG(WDT_MODULE_ID i)
{
    switch (i) {
        case WDT_ID_0 :
            return &WDTCON;
        case WDT_NUMBER_OF_MODULES :
        default :
            return (SFR_TYPE*)-1;
    }
}

PLIB_INLINE SFR_DATA _WDT_WINDOW_ENABLE_MASK(WDT_MODULE_ID i)
{
    switch (i) {
        case WDT_ID_0 :
            return _WDTCON_WDTWINEN_MASK;
        case WDT_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _WDT_POSTSCALER_VALUE_MASK(WDT_MODULE_ID i)
{
    switch (i) {
        case WDT_ID_0 :
            return _WDTCON_RUNDIV_MASK;
        case WDT_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _WDT_ENABLE_CONTROL_MASK(WDT_MODULE_ID i)
{
    switch (i) {
        case WDT_ID_0 :
            return _WDTCON_ON_MASK;
        case WDT_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _WDT_TIMER_CLEAR_KEY_MASK(WDT_MODULE_ID i)
{
    switch (i) {
        case WDT_ID_0 :
            return _WDTCON_WDTCLRKEY_MASK;
        case WDT_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _WDT_WINDOW_ENABLE_POS(WDT_MODULE_ID i)
{
    switch (i) {
        case WDT_ID_0 :
            return _WDTCON_WDTWINEN_POSITION;
        case WDT_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _WDT_POSTSCALER_VALUE_POS(WDT_MODULE_ID i)
{
    switch (i) {
        case WDT_ID_0 :
            return _WDTCON_RUNDIV_POSITION;
        case WDT_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _WDT_ENABLE_CONTROL_POS(WDT_MODULE_ID i)
{
    switch (i) {
        case WDT_ID_0 :
            return _WDTCON_ON_POSITION;
        case WDT_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _WDT_TIMER_CLEAR_KEY_POS(WDT_MODULE_ID i)
{
    switch (i) {
        case WDT_ID_0 :
            return _WDTCON_WDTCLRKEY_POSITION;
        case WDT_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _WDT_WINDOW_ENABLE_LEN(WDT_MODULE_ID i)
{
    switch (i) {
        case WDT_ID_0 :
            return _WDTCON_WDTWINEN_LENGTH;
        case WDT_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _WDT_POSTSCALER_VALUE_LEN(WDT_MODULE_ID i)
{
    switch (i) {
        case WDT_ID_0 :
            return _WDTCON_RUNDIV_LENGTH;
        case WDT_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _WDT_ENABLE_CONTROL_LEN(WDT_MODULE_ID i)
{
    switch (i) {
        case WDT_ID_0 :
            return _WDTCON_ON_LENGTH;
        case WDT_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _WDT_TIMER_CLEAR_KEY_LEN(WDT_MODULE_ID i)
{
    switch (i) {
        case WDT_ID_0 :
            return _WDTCON_WDTCLRKEY_LENGTH;
        case WDT_NUMBER_OF_MODULES :
        default :
            return (SFR_DATA)-1;
    }
}

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/wdt_EnableControl_Default.h"
#include "../templates/wdt_WindowEnable_Default.h"
#include "../templates/wdt_TimerClear_WithKey.h"
#include "../templates/wdt_PostscalerValue_Default.h"
#include "../templates/wdt_SleepModePostscalerValue_Unsupported.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API bool PLIB_WDT_ExistsEnableControl(WDT_MODULE_ID index)
{
    switch (index) {
        case WDT_ID_0 :
            return WDT_ExistsEnableControl_Default(index);
        case WDT_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_WDT_Enable(WDT_MODULE_ID index)
{
    switch (index) {
        case WDT_ID_0 :
            WDT_Enable_Default(index);
            break;
        case WDT_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_WDT_Disable(WDT_MODULE_ID index)
{
    switch (index) {
        case WDT_ID_0 :
            WDT_Disable_Default(index);
            break;
        case WDT_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_WDT_IsEnabled(WDT_MODULE_ID index)
{
    switch (index) {
        case WDT_ID_0 :
            return WDT_IsEnabled_Default(index);
        case WDT_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API bool PLIB_WDT_ExistsWindowEnable(WDT_MODULE_ID index)
{
    switch (index) {
        case WDT_ID_0 :
            return WDT_ExistsWindowEnable_Default(index);
        case WDT_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_WDT_WindowEnable(WDT_MODULE_ID index)
{
    switch (index) {
        case WDT_ID_0 :
            WDT_WindowEnable_Default(index);
            break;
        case WDT_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API void PLIB_WDT_WindowDisable(WDT_MODULE_ID index)
{
    switch (index) {
        case WDT_ID_0 :
            WDT_WindowDisable_Default(index);
            break;
        case WDT_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_WDT_ExistsTimerClear(WDT_MODULE_ID index)
{
    switch (index) {
        case WDT_ID_0 :
            return WDT_ExistsTimerClear_WithKey(index);
        case WDT_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API void PLIB_WDT_TimerClear(WDT_MODULE_ID index)
{
    switch (index) {
        case WDT_ID_0 :
            WDT_TimerClear_WithKey(index);
            break;
        case WDT_NUMBER_OF_MODULES :
        default :
            break;
    }
}

PLIB_INLINE_API bool PLIB_WDT_ExistsPostscalerValue(WDT_MODULE_ID index)
{
    switch (index) {
        case WDT_ID_0 :
            return WDT_ExistsPostscalerValue_Default(index);
        case WDT_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API char PLIB_WDT_PostscalerValueGet(WDT_MODULE_ID index)
{
    switch (index) {
        case WDT_ID_0 :
            return WDT_PostscalerValueGet_Default(index);
        case WDT_NUMBER_OF_MODULES :
        default :
            return (char)0;
    }
}

PLIB_INLINE_API bool PLIB_WDT_ExistsSleepModePostscalerValue(WDT_MODULE_ID index)
{
    switch (index) {
        case WDT_ID_0 :
            return WDT_ExistsSleepModePostscalerValue_Unsupported(index);
        case WDT_NUMBER_OF_MODULES :
        default :
            return (bool)0;
    }
}

PLIB_INLINE_API char _PLIB_UNSUPPORTED PLIB_WDT_SleepModePostscalerValueGet(WDT_MODULE_ID index)
{
    switch (index) {
        case WDT_ID_0 :
            return WDT_SleepModePostscalerValueGet_Unsupported(index);
        case WDT_NUMBER_OF_MODULES :
        default :
            return (char)0;
    }
}

#endif
