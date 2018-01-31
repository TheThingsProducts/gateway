// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#ifndef _HELPER_WDT_H /* Guard against multiple inclusion */
#define _HELPER_WDT_H

#include <stdint.h>

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C"
{
#endif

    void    wdt_init();
    uint8_t wdt_get_reboot_reason();

    void wdt_arm_thread_0();
    void wdt_arm_thread_1();
    void wdt_arm_thread_2();

    void wdt_kick();

#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
