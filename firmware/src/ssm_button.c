// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#include <stdint.h>
#include "system_definitions.h"
#include "ssm_button.h"

/* Load config in serial flash update sequence */
typedef enum
{
    STATE_IDLE,
    STATE_IS_PRESSED,
    STATE_WAS_PRESSED_10_MILLISECONDS,
    STATE_WAS_PRESSED_2_SECONDS,
    STATE_WAS_PRESSED_5_SECONDS,
} STATE_t;

static void _setState(STATE_t newState);
static bool _isButtonPressed();

static STATE_t  _state;
static uint32_t startTick;

bool SSMButton_WasPressed10Milliseconds(void)
{
    return _state == STATE_WAS_PRESSED_10_MILLISECONDS;
}

bool SSMButton_WasPressed2Seconds(void)
{
    return _state == STATE_WAS_PRESSED_2_SECONDS;
}

bool SSMButton_WasPressed5Seconds(void)
{
    return _state == STATE_WAS_PRESSED_5_SECONDS;
}

bool SSMButton_IsPressed(void)
{
    return _isButtonPressed();
}

void SSMButton_Reset(void)
{
    _state = STATE_IDLE;
}

uint32_t SSMButton_GetStartTick(void)
{
    return startTick;
}

static void _setState(STATE_t newState)
{
    _state = newState;
    switch(_state)
    {
        case STATE_IDLE:
            break;
        case STATE_IS_PRESSED:
            startTick = SYS_TMR_TickCountGet();
            break;
        case STATE_WAS_PRESSED_10_MILLISECONDS:
            break;
        case STATE_WAS_PRESSED_2_SECONDS:
            break;
        case STATE_WAS_PRESSED_5_SECONDS:
            break;
    }

    SYS_PRINT("BTN: change to state: %d\r\n", _state);
}

void SSMButton_Tasks(void)
{
    switch(_state)
    {
        case STATE_IDLE:
            if(_isButtonPressed())
            {
                _setState(STATE_IS_PRESSED);
            }
            break;

        case STATE_IS_PRESSED:
            if(!_isButtonPressed())
            {
                if(SYS_TMR_TickCountGet() - startTick >= SYS_TMR_TickCounterFrequencyGet() * 5)
                {
                    _setState(STATE_WAS_PRESSED_5_SECONDS);
                }
                else if(SYS_TMR_TickCountGet() - startTick >= SYS_TMR_TickCounterFrequencyGet() * 2)
                {
                    _setState(STATE_WAS_PRESSED_2_SECONDS);
                }
                else if(SYS_TMR_TickCountGet() - startTick >= SYS_TMR_TickCounterFrequencyGet() / 100)
                {
                    _setState(STATE_WAS_PRESSED_10_MILLISECONDS);
                }
                else
                {
                    _setState(STATE_IDLE);
                }
            }
            break;

        case STATE_WAS_PRESSED_10_MILLISECONDS:
            break;

        case STATE_WAS_PRESSED_2_SECONDS:
            break;

        case STATE_WAS_PRESSED_5_SECONDS:
            break;
    }
}

static bool _isButtonPressed()
{
    return (PORTBbits.RB12 == 0);
}
