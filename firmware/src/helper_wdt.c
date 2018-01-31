// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#include <app.h>
#include "peripheral/wdt/plib_wdt.h"
#include "peripheral/tmr/plib_tmr.h"
#include "peripheral/reset/plib_reset.h"
#include "peripheral/int/plib_int.h"

#include "helper_wdt.h"

static bool    t1           = false;
static bool    t2           = false;
static bool    t3           = false;
static uint8_t rebootReason = 0;

void wdt_init()
{
    // bool was_wdt_reset =false;
    rebootReason = PLIB_RESET_ReasonGet(RESET_ID_0);
    if((rebootReason & RESET_REASON_WDT_TIMEOUT) == RESET_REASON_WDT_TIMEOUT)
    {
        SYS_DEBUG(SYS_ERROR_FATAL, "\r\nWDT : BOARD RESET\r\n");
        // was_wdt_reset=true;
    }
    else
    {
        SYS_DEBUG(SYS_ERROR_FATAL, "\r\nWDT : Clean reset\r\n");
        // was_wdt_reset=false;
    }
    SYS_DEBUG(SYS_ERROR_FATAL, "WDT : REASON: %d\r\n", rebootReason);

    PLIB_WDT_Disable(WDT_ID_0);
    PLIB_WDT_TimerClear(WDT_ID_0);
    PLIB_RESET_ReasonClear(RESET_ID_0, RESET_REASON_ALL);
    PLIB_WDT_Enable(WDT_ID_0);
}

uint8_t wdt_get_reboot_reason(void)
{
    return rebootReason;
}

void wdt_arm_thread_0()
{
    t1 = true;
}

void wdt_arm_thread_1()
{
    t2 = true;
}

void wdt_arm_thread_2()
{
    t3 = true;
}

void wdt_kick()
{
    if(t1 && t2 && t3)
    {
        t1 = false;
        t2 = false;
        t3 = false;
        PLIB_WDT_TimerClear(WDT_ID_0);
    }
}
