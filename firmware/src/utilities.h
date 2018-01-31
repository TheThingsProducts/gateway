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

#ifndef _UTILITIES_H /* Guard against multiple inclusion */
#define _UTILITIES_H

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <system_config.h>
#include <system_definitions.h>

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C"
{
#endif

    void PRINT_ARRAY_UINT8(uint8_t* array, size_t length, char* service, char* name);

#define AsciiToHexByte(m, l) ((AsciiToHexNibble(m) << 4) | AsciiToHexNibble(l))
    unsigned char AsciiToHexNibble(unsigned char data);
    void          AsciiStringToHex(const char* AsciiString, uint8_t* buff, size_t length);

    void SoftReset(void);
    void WatchDogReset(void);
    void _AssertImpl(const char* file, int line, char* cond, const char* format, ...);
    void RebootWithMessage(const char* format, ...);

#define FATAL(fmt, ...) _AssertImpl(__FILE__, __LINE__, "fatal", fmt, ##__VA_ARGS__)
#define ASSERT(cond, fmt, ...)                                          \
    {                                                                   \
        if(!(cond))                                                     \
        {                                                               \
            _AssertImpl(__FILE__, __LINE__, #cond, fmt, ##__VA_ARGS__); \
        }                                                               \
    }

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _UTILITIES_H */

/* *****************************************************************************
 End of File
 */
