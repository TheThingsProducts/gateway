/*******************************************************************************
  System Definitions

  File Name:
    system_definitions.h

  Summary:
    MPLAB Harmony project system definitions.

  Description:
    This file contains the system-wide prototypes and definitions for an MPLAB
    Harmony project.
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef _SYS_DEFINITIONS_H
#define _SYS_DEFINITIONS_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "system/common/sys_common.h"
#include "system/common/sys_module.h"
#include "system/clk/sys_clk.h"
#include "system/clk/sys_clk_static.h"
#include "system/devcon/sys_devcon.h"
#include "system/int/sys_int.h"
#include "system/dma/sys_dma.h"
#include "system/fs/sys_fs.h"
#include "system/fs/sys_fs_media_manager.h"
#include "system/console/sys_console.h"
#include "system/random/sys_random.h"
#include "system/fs/mpfs/mpfs.h"
#include "system/fs/fat_fs/src/file_system/ff.h"
#include "system/fs/fat_fs/src/file_system/ffconf.h"
#include "system/fs/fat_fs/src/hardware_access/diskio.h"
#include "system/tmr/sys_tmr.h"
#include "system/reset/sys_reset.h"
#include "driver/tmr/drv_tmr.h"
#include "driver/usart/drv_usart.h"
#include "driver/nvm/drv_nvm.h"
#include "driver/spi_flash/sst25vf064c/drv_sst25vf064c.h"
#include "system/ports/sys_ports.h"
#include "driver/sdcard/drv_sdcard.h"
#include "driver/spi/drv_spi.h"
#include "system/debug/sys_debug.h"
#include "system/command/sys_command.h"



#include "tcpip/tcpip.h"
#include "driver/ethmac/drv_ethmac.h"
#include "wdrv_mrf24wn_common.h"
#include "FreeRTOS.h"
#include "task.h"
#include "net/pres/net_pres.h"
#include "net/pres/net_pres_encryptionproviderapi.h"
#include "net/pres/net_pres_transportapi.h"
#include "net/pres/net_pres_socketapi.h"
#include "app.h"
#include "lora.h"
#include "utilities.h"


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* System Objects

  Summary:
    Structure holding the system's object handles

  Description:
    This structure contains the object handles for all objects in the
    MPLAB Harmony project's system configuration.

  Remarks:
    These handles are returned from the "Initialize" functions for each module
    and must be passed into the "Tasks" function for each module.
*/

typedef struct
{
    SYS_MODULE_OBJ  sysDevcon;
    SYS_MODULE_OBJ  sysTmr;
    SYS_MODULE_OBJ  sysDma;
    SYS_MODULE_OBJ  drvTmr0;
    SYS_MODULE_OBJ  drvTmr1;
    SYS_MODULE_OBJ  drvUsart0;
    SYS_MODULE_OBJ  drvUsart1;
    SYS_MODULE_OBJ  drvUsart2;
    SYS_MODULE_OBJ  drvSst25vf064c0;
    SYS_MODULE_OBJ  drvNvm;
    SYS_MODULE_OBJ  drvSDCard;
    SYS_MODULE_OBJ  sysDebug;
    SYS_MODULE_OBJ  sysConsole0;

    /*** SPI Object for Index 0 ***/
    SYS_MODULE_OBJ				spiObjectIdx0;
    
    /*** SPI Object for Index 1 ***/
    SYS_MODULE_OBJ				spiObjectIdx1;
    
    /*** SPI Object for Index 2 ***/
    SYS_MODULE_OBJ				spiObjectIdx2;
    SYS_MODULE_OBJ  tcpip;
    SYS_MODULE_OBJ  netPres;

} SYSTEM_OBJECTS;


// *****************************************************************************
// *****************************************************************************
// Section: extern declarations
// *****************************************************************************
// *****************************************************************************

extern SYSTEM_OBJECTS sysObj;


//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* _SYS_DEFINITIONS_H */
/*******************************************************************************
 End of File
*/

