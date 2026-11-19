// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#include <app.h>
#include "peripheral/wdt/plib_wdt.h"
#include "peripheral/tmr/plib_tmr.h"
#include "peripheral/reset/plib_reset.h"
#include "peripheral/int/plib_int.h"

#include "peripheral/usart/plib_usart.h"
#include "FreeRTOS.h"
#include "task.h"

#include "helper_wdt.h"

/* Diagnostics for watchdog resets (issue #5): markers that the tick hook reports on the console
 * when the watchdog has not been cleared for 6 s. */
volatile uint8_t  wdt_sys_stage = 0; // sub-step the SYS task is in
volatile uint32_t wdt_tls_loops = 0; // iterations of the TLS handshake loop
volatile int32_t  wdt_tls_last  = 0; // last return value of mbedtls_ssl_handshake
static volatile uint32_t ticksSinceKick = 0;

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
        ticksSinceKick = 0;
        PLIB_WDT_TimerClear(WDT_ID_0);
    }
}

/* Polling console output, usable from the tick interrupt (no printf: small ISR stack). */
static void wdt_putc(char c)
{
    while(PLIB_USART_TransmitterBufferIsFull(USART_ID_4))
        ;
    PLIB_USART_TransmitterByteSend(USART_ID_4, c);
}

static void wdt_puts(const char* s)
{
    while(*s)
        wdt_putc(*s++);
}

static void wdt_putu(uint32_t v)
{
    char b[11];
    int  i = 10;
    b[10]  = '\0';
    do
    {
        b[--i] = '0' + (v % 10);
        v /= 10;
    } while(v && i);
    wdt_puts(&b[i]);
}

/* Called from the FreeRTOS tick interrupt. Reports every 0.5 s once the watchdog has not been
 * cleared for 6 s (it expires after about 8 s). */
void wdt_tick_check(void)
{
    uint32_t t = ++ticksSinceKick;
    if(t >= 6 * configTICK_RATE_HZ && (t % (configTICK_RATE_HZ / 2)) == 0)
    {
        int32_t last = wdt_tls_last;
        wdt_puts("\r\n*** WDT pending ");
        wdt_putu(t * 10 / configTICK_RATE_HZ);
        wdt_puts(" ds, armed sys/app/lora=");
        wdt_putc(t1 ? '1' : '0');
        wdt_putc(t2 ? '1' : '0');
        wdt_putc(t3 ? '1' : '0');
        wdt_puts(" sysstage=");
        wdt_putu(wdt_sys_stage);
        wdt_puts(" tlsloops=");
        wdt_putu(wdt_tls_loops);
        wdt_puts(" tlsret=");
        if(last < 0)
        {
            wdt_putc('-');
            last = -last;
        }
        wdt_putu((uint32_t)last);
        wdt_puts(" cur=");
        wdt_puts(pcTaskGetTaskName(NULL));
        wdt_puts("\r\n");
    }
}
