/*******************************************************************************
    NVM Disk Images FAT Type support definitions file

  File Name:
    nvm_disk_images.h

  Summary:
    This file contains definitions needed for the FAT type disk image
    in NVM Memory.

  Description:
    This file contains definitions needed for the FAT type disk image
    in NVM Memory. This file may be generated manually as a part of
    a SYS FS Configuration utility.
 *******************************************************************************/


// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

#ifndef _NVM_DISK_IMAGES_H_
#define _NVM_DISK_IMAGES_H_

#include <stdint.h>

/******************************************************************************
* --------------------------------------------------------------------------
* The size (in number of sectors) of the desired usable data portion of the
* MSD volume
* --------------------------------------------------------------------------
* Note: Windows 7 appears to require a minimum capacity of at least 13 sectors.
* Note2: Windows will not be able to format a drive if it is too small.  The
* reason for this, is that Windows will try to put a "heavyweight"
* (comparatively) filesystem on the drive, which will consume ~18kB of overhead
* for the filesystem.  If the total drive size is too small to fit the
* filesystem, then Windows will give an error. This also means that formatting
* the drive will "shrink" the usuable data storage area, since the default
* FAT12 filesystem implemented in the Files.c data tables is very
* lightweight, with very low overhead.
* Note3: It is important to make sure that no part of the MSD volume shares
* a flash erase page with the firmware program memory.  This can be done by
* using a custom modified linker script, or by carefully selecting the
* starting address and the total size of the MSD volume.  See also below code
* comments. Note4: It is also important to make sure that no part of the MSD
* volume shares an erase page with the erase page that contains the
* microcontroller's configuration bits (for microcontrollers that use flash
* for storing the configuration bits, see device datasheet). This can be
* accomplished by using a modified linker script, which protects the flash page
* with the configuration bits (if applicable), or, by carefully choosing the
* FILES_ADDRESS and FS_INTERNAL_FLASH_DRIVE_CAPACITY, to make sure the MSD
* volume does extend into the erase page with the configuration
* bits.
******************************************************************************/
#define FS_INTERNAL_FLASH_DRIVE_CAPACITY 30

/***********************************
 * Size of a sector in bytes
 **********************************/
#define MEDIA_SECTOR_SIZE 		512

/*******************************************************************/
/******** File System selection options ************************/
/*******************************************************************/
#define ERASE_BLOCK_SIZE        16384
#define WRITE_BLOCK_SIZE        16384

#define INITIALIZATION_VALUE		0x55

#if !defined(FS_INTERNAL_FLASH_MAX_NUM_FILES_IN_ROOT)
    #define FS_INTERNAL_FLASH_MAX_NUM_FILES_IN_ROOT 16
#endif

/******************************************************************************
 * Note: If only 1 FAT sector is used, assuming 12-bit (1.5 byte) FAT entry
 * size (ex: FAT12 filesystem), then the total FAT entries that can fit in a
 * single 512 byte FAT sector is (512 bytes) / (1.5 bytes/entry) = 341 entries.
 * This allows the FAT table to reference up to 341*512 = ~174kB of space.
 * Therfore, more FAT sectors are needed if creating an MSD volume bigger than
 * this.
 *****************************************************************************/

#define FS_INTERNAL_FLASH_NUM_RESERVED_SECTORS 1
#define FS_INTERNAL_FLASH_NUM_VBR_SECTORS 1
#define FS_INTERNAL_FLASH_NUM_FAT_SECTORS 1
#define FS_INTERNAL_FLASH_NUM_ROOT_DIRECTORY_SECTORS ((FS_INTERNAL_FLASH_MAX_NUM_FILES_IN_ROOT+15)/16) //+15 because the compiler truncates
#define FS_INTERNAL_FLASH_OVERHEAD_SECTORS (\
            FS_INTERNAL_FLASH_NUM_RESERVED_SECTORS + \
            FS_INTERNAL_FLASH_NUM_VBR_SECTORS + \
            FS_INTERNAL_FLASH_NUM_ROOT_DIRECTORY_SECTORS + \
            FS_INTERNAL_FLASH_NUM_FAT_SECTORS)
#define FS_INTERNAL_FLASH_TOTAL_DISK_SIZE (\
            FS_INTERNAL_FLASH_OVERHEAD_SECTORS + \
            FS_INTERNAL_FLASH_DRIVE_CAPACITY)
#define FS_INTERNAL_FLASH_PARTITION_SIZE (uint32_t)(FS_INTERNAL_FLASH_TOTAL_DISK_SIZE - 1)  //-1 is to exclude the sector used for the MBR


/**********************************************************
 * Do some build time error checking
 **********************************************************/

#if (FS_INTERNAL_FLASH_MAX_NUM_FILES_IN_ROOT>64)
    #if defined(__C30__)
        #error "PSV only allows 32KB of memory.  The drive options selected result in more than 32KB of data.  Please reduce the number of root directory entries possible"
    #endif
#endif

#if (MEDIA_SECTOR_SIZE != 512)
    #error "The current implementation of internal flash file system, only supports a media sector size of 512.  Please modify your selected value in the FSconfig.h file."
#endif

#if (FS_INTERNAL_FLASH_MAX_NUM_FILES_IN_ROOT != 16) && \
    (FS_INTERNAL_FLASH_MAX_NUM_FILES_IN_ROOT != 32) && \
    (FS_INTERNAL_FLASH_MAX_NUM_FILES_IN_ROOT != 48) && \
    (FS_INTERNAL_FLASH_MAX_NUM_FILES_IN_ROOT != 64)
    #error "Number of root file entries must be a multiple of 16.  Please adjust the definition in the FSconfig.h file."
#endif

#if (ERASE_BLOCK_SIZE > WRITE_BLOCK_SIZE)
    #define BLOCK_ALIGNMENT ERASE_BLOCK_SIZE
#else
    #define BLOCK_ALIGNMENT WRITE_BLOCK_SIZE
#endif

#define MBR_ATTRIBUTES                      __attribute__((aligned (ERASE_BLOCK_SIZE),section(".FS_FILES")))
#define PARTITION_ATTRIBUTES(sector_num)    __attribute__ ((section(".FS_FILES")))


/*********** Sector Address Calculation macros ********************
 * These macros are used to calculate the sector address of each
 * of the blocks.  These are then used to locate where the blocks
 * go in program memory on certain processors using processor specific
 * attribute() commands
 *******************************************************************/

#define BOOT_SECTOR_ADDRESS         1
#define FAT0_ADDRESS                (BOOT_SECTOR_ADDRESS + 1)
#define FATx_ADDRESS                (FAT0_ADDRESS + 1)
#define ROOTDIRECTORY0_ADDRESS      (FAT0_ADDRESS + FS_INTERNAL_FLASH_NUM_FAT_SECTORS)

#if (FS_INTERNAL_FLASH_MAX_NUM_FILES_IN_ROOT>16)
    #define ROOTDIRECTORY1_ADDRESS  (ROOTDIRECTORY0_ADDRESS + 1)
#else
    #define ROOTDIRECTORY1_ADDRESS  (ROOTDIRECTORY0_ADDRESS)
#endif

#if (FS_INTERNAL_FLASH_MAX_NUM_FILES_IN_ROOT>32)
    #define ROOTDIRECTORY2_ADDRESS  (ROOTDIRECTORY1_ADDRESS + 1)
#else
    #define ROOTDIRECTORY2_ADDRESS  (ROOTDIRECTORY1_ADDRESS)
#endif

#if (FS_INTERNAL_FLASH_MAX_NUM_FILES_IN_ROOT>48)
    #define ROOTDIRECTORY3_ADDRESS  (ROOTDIRECTORY2_ADDRESS + 1)
#else
    #define ROOTDIRECTORY3_ADDRESS  (ROOTDIRECTORY2_ADDRESS)
#endif

#define SLACK0_ADDRESS              (ROOTDIRECTORY3_ADDRESS + 1)
#define SLACK1_ADDRESS              (SLACK0_ADDRESS + 1)
#define SLACK2_ADDRESS              (SLACK1_ADDRESS + 1)
#define SLACK3_ADDRESS              (SLACK2_ADDRESS + 1)
#define SLACK4_ADDRESS              (SLACK3_ADDRESS + 1)
#define SLACK5_ADDRESS              (SLACK4_ADDRESS + 1)
#define SLACK6_ADDRESS              (SLACK5_ADDRESS + 1)
#define SLACK7_ADDRESS              (SLACK6_ADDRESS + 1)

extern const uint8_t MBR_ATTRIBUTES masterBootRecord[];
#endif
