// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "app_sdcard.h"
#include "app.h"
#include "subsystem_controller.h"
#include "utilities.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

static APP_DATA_SDCARD sdcardData;

#define MOUNT_TIMEOUT 2 // seconds before giving up mounting
static uint32_t startMountTick = 0;

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

/*******************************************************************************
  Function:
    void APP_SDCARD_Initialize(void)

  Remarks:
    None.
 */
void APP_SDCARD_Initialize(void)
{
    sdcardData.is_present = false;
    sdcardData.is_mounted = false;
    sdcardData.state      = APP_SDCARD_IDLE;
}

/*******************************************************************************
  Function:
    void APP_SDCARD_Tasks(void)

  Remarks:
    None.
 */
void APP_SDCARD_Tasks(void)
{
    // SD_PRESENT
    if(PORTKbits.RK0 == 0)
    {
        sdcardData.is_present = true;
    }
    else
    {
        sdcardData.is_present = false;
    }

    switch(sdcardData.state)
    {
        case APP_SDCARD_MOUNT_DISK:
        {
            if(SYS_FS_Mount("/dev/mmcblka1", "/mnt/myDrive", FAT, 0, NULL) != 0)
            {
                /* The disk could not be mounted. Try
                 * mounting again untill success within timeout. */
                if(SYS_TMR_TickCountGet() - startMountTick >= SYS_TMR_TickCounterFrequencyGet() * MOUNT_TIMEOUT)
                {
                    sdcardData.state = APP_SDCARD_IDLE;
                }
                else
                {
                    sdcardData.state = APP_SDCARD_MOUNT_DISK;
                }
            }
            else
            {
                /* Mount was successful. Unmount the disk, for testing. */
                SYS_DEBUG(SYS_ERROR_INFO, "SDCARD: Mounted disk\r\n");
                sdcardData.is_mounted = true;
                sdcardData.state      = APP_SDCARD_SET_CURRENT_DRIVE;
            }
            break;
        }

        case APP_SDCARD_SET_CURRENT_DRIVE:
        {
            if(SYS_FS_CurrentDriveSet("/mnt/myDrive") == SYS_FS_RES_FAILURE)
            {
                /* Error while setting current drive */
                sdcardData.state = APP_SDCARD_ERROR;
            }
            else
            {
                sdcardData.state = APP_SDCARD_IDLE;
            }
            break;
        }

        case APP_SDCARD_UNMOUNT_DISK:
        {
            if(SYS_FS_Unmount("/mnt/myDrive") != 0)
            {
                /* The disk could not be un mounted. Try
                 * un mounting again untill success within timeout. */
                if(SYS_TMR_TickCountGet() - startMountTick >= SYS_TMR_TickCounterFrequencyGet() * MOUNT_TIMEOUT)
                {
                    sdcardData.state = APP_SDCARD_IDLE;
                }
                else
                {
                    sdcardData.state = APP_SDCARD_UNMOUNT_DISK;
                }
            }
            else
            {
                /* UnMount was successful. */
                SYS_DEBUG(SYS_ERROR_INFO, "SDCARD: Unmounted disk\r\n");
                sdcardData.is_mounted = false;
                sdcardData.state      = APP_SDCARD_IDLE;
            }
            break;
        }

        case APP_SDCARD_IDLE:
        {
            if((sdcardData.is_present) && (!sdcardData.is_mounted))
            {
                startMountTick   = SYS_TMR_TickCountGet();
                sdcardData.state = APP_SDCARD_MOUNT_DISK;
            }
            if((!sdcardData.is_present) && (sdcardData.is_mounted))
            {
                startMountTick   = SYS_TMR_TickCountGet();
                sdcardData.state = APP_SDCARD_UNMOUNT_DISK;
            }
            break;
        }

        case APP_SDCARD_ERROR:
        {
            SYS_FS_ERROR err;
            err = SYS_FS_Error();
            SYS_PRINT("SDCARD: FS error: %u\r\n", err);
            sdcardData.state = APP_SDCARD_ERROR2;
            break;
        }
        case APP_SDCARD_ERROR2:
        {
            break;
        }

        default:
        {
            break;
        }
    }
}

bool APP_SDCARD_IsReady(void)
{
    return (sdcardData.state == APP_SDCARD_IDLE);
}

bool APP_SDCARD_IsMounted(void)
{
    return sdcardData.is_mounted;
}

bool APP_SDCARD_HasError(void)
{
    return (sdcardData.state == APP_SDCARD_ERROR || sdcardData.state == APP_SDCARD_ERROR2);
}

APP_STATES_SDCARD APP_SDCARD_State(void)
{
    return sdcardData.state;
}

/* *****************************************************************************
 End of File
 */
