// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

/* ************************************************************************** */
/** Descriptive File Name

  @File Name
    utilities.h

  @Summary
    All sorts of utilities functions

 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

#include "utilities.h"
#include "task.h"
#include "subsystem_controller.h"

#define FATAL_MSG_BUFFER_SIZE 256
static char _fatalMsgBuffer[FATAL_MSG_BUFFER_SIZE];

static void _SoftReset(void);
static void _FatalMessage(const char* format, ...);
static void _vFatalMessage(const char* format, va_list args);

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

void PRINT_ARRAY_UINT8(uint8_t* array, size_t length, char* service, char* name)
{
    int i = 0;
    SYS_PRINT("%s: (%s) ", service, name);
    for(i = 0; i < length; i++)
    {
        SYS_PRINT("%.2X ", array[i]);
    }
    SYS_PRINT("\r\n");
}

unsigned char AsciiToHexNibble(unsigned char data)
{
    if(data < '0') // return 0 for an invalid characters
    {
        return 0;
    }
    else if(data <= '9') // handle numbers
    {
        return (data - '0');
    }
    else if(data < 'A')
    {
        return 0;
    }
    else if(data <= 'F') // handle uppercase letters
    {
        return (data - 'A' + 10);
    }
    else if(data < 'a')
    {
        return 0;
    }
    else if(data <= 'f') // handle lowercase letters
    {
        return (data - 'a' + 10);
    }
    else
    {
        return 0;
    }
}

void AsciiStringToHex(const char* AsciiString, uint8_t* buff, size_t length)
{
    uint8_t i1 = 0;
    uint8_t i2 = 0;
    for(i1 = 0; i1 < length; i1++)
    {
        buff[i1] = AsciiToHexByte(AsciiString[i2], AsciiString[i2 + 1]);
        i2 += 2;
    }
}

static void _SoftReset(void)
{
    wifiSet(OFF);
    ethernetSet(OFF);
    bleSet(OFF);
    loraSet(OFF);
    sdSet(OFF);

    // Wait after putting modules off (with a busy while-loop as timer is guaranteed running)
    uint32_t countdown = 100000000;
    for(; countdown > 0; countdown--)
        ;

    SYS_RESET_SoftwareReset();
}

void WatchDogReset(void)
{
    vTaskDelay(500 / portTICK_PERIOD_MS); // Give the debug messages give chance to be printed
    while(1)
        ; // Endless loop
}

void _AssertImpl(const char* file, int line, char* cond, const char* format, ...)
{
    taskDISABLE_INTERRUPTS();
    vTaskSuspendAll();

    ErrorMessageFatal_StoreAssert(file, line);

    _FatalMessage("\r\n*** assert %s:%d:%s:", file, line, cond);
    va_list args;
    va_start(args, format);
    _vFatalMessage(format, args);
    va_end(args);
    _FatalMessage("\r\n\r\n");

    SYS_DEBUG_BreakPoint();
    _SoftReset();
}

void RebootWithMessage(const char* format, ...)
{
    taskDISABLE_INTERRUPTS();
    vTaskSuspendAll();

    _FatalMessage("\r\n*** ");
    va_list args;
    va_start(args, format);
    _vFatalMessage(format, args);
    va_end(args);
    _FatalMessage("\r\n\r\n");

    SYS_DEBUG_BreakPoint();
    _SoftReset();
}

static void _FatalMessage(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    _vFatalMessage(format, args);
    va_end(args);
}

static void _vFatalMessage(const char* format, va_list args)
{

    size_t len = vsnprintf(_fatalMsgBuffer, sizeof(_fatalMsgBuffer), format, args);
    if(len >= sizeof(_fatalMsgBuffer))
    {
        len = sizeof(_fatalMsgBuffer) - 1;
    }
    _fatalMsgBuffer[len] = '\0';

    int i;
    for(i = 0; i < len; i++)
    {
        while(PLIB_USART_TransmitterBufferIsFull(USART_ID_4))
            ;
        PLIB_USART_TransmitterByteSend(USART_ID_4, _fatalMsgBuffer[i]);
    }
    while(!PLIB_USART_TransmitterIsEmpty(USART_ID_4))
        ; // Wait till all is out
}

/* *****************************************************************************
 End of File
 */
