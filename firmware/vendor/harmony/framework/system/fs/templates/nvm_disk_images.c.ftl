/*******************************************************************************
    NVM Disk Images FAT Type Disk image
  
  File Name:
    nvm_disk_images.c    

  Summary:
    This file contains the FAT type disk image implementation. 

  Description:
    This file contains the FAT type disk image implementation. 
    This file may be generated manually as a part of SYS FS
    Configuration utility.
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

#include "system_definitions.h"
#include "nvm_disk_images.h"

struct fatfsImage
{
    uint8_t masterBootRecord[MEDIA_SECTOR_SIZE];
    uint8_t bootSector[MEDIA_SECTOR_SIZE];
    uint8_t fileAllocationTable[MEDIA_SECTOR_SIZE];

#if(FS_INTERNAL_FLASH_NUM_FAT_SECTORS > 1)
    uint8_t FATx[MEDIA_SECTOR_SIZE*(FS_INTERNAL_FLASH_NUM_FAT_SECTORS - 1)];
#endif

    uint8_t rootDirectory[MEDIA_SECTOR_SIZE];

#if (FS_INTERNAL_FLASH_MAX_NUM_FILES_IN_ROOT>16)
    uint8_t RootDirectory1[MEDIA_SECTOR_SIZE];
#endif

#if (FS_INTERNAL_FLASH_MAX_NUM_FILES_IN_ROOT>32)
    uint8_t RootDirectory2[MEDIA_SECTOR_SIZE];
#endif

#if (FS_INTERNAL_FLASH_MAX_NUM_FILES_IN_ROOT>48)
    uint8_t RootDirectory3[MEDIA_SECTOR_SIZE];
#endif

#if (FS_INTERNAL_FLASH_DRIVE_CAPACITY>0)
    uint8_t slack0_0[MEDIA_SECTOR_SIZE];
#endif

#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY>1)
#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 64)
    uint8_t slack1[MEDIA_SECTOR_SIZE*63];
#else
    uint8_t slack1[MEDIA_SECTOR_SIZE*(FS_INTERNAL_FLASH_DRIVE_CAPACITY - 1u)];
#endif
#endif

#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 64)
#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 127)
    uint8_t slack2[MEDIA_SECTOR_SIZE*63];
#else
    uint8_t slack2[MEDIA_SECTOR_SIZE*(FS_INTERNAL_FLASH_DRIVE_CAPACITY - 64u)];
#endif
#endif

#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 127)
#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 190)
    uint8_t slack3[MEDIA_SECTOR_SIZE*63];
#else
    uint8_t slack3[MEDIA_SECTOR_SIZE*(FS_INTERNAL_FLASH_DRIVE_CAPACITY - 127u)];
#endif
#endif

#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 190)
#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 253)
    uint8_t slack4[MEDIA_SECTOR_SIZE*63];
#else
    uint8_t slack4[MEDIA_SECTOR_SIZE*(FS_INTERNAL_FLASH_DRIVE_CAPACITY - 190u)];
#endif
#endif

#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 253)
#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 316)
    uint8_t slack5[MEDIA_SECTOR_SIZE*63];
#else
    uint8_t slack5[MEDIA_SECTOR_SIZE*(FS_INTERNAL_FLASH_DRIVE_CAPACITY - 253u)];
#endif
#endif

#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 316)
#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 379)
    uint8_t slack6[MEDIA_SECTOR_SIZE*63];
#else
    uint8_t slack6[MEDIA_SECTOR_SIZE*(FS_INTERNAL_FLASH_DRIVE_CAPACITY - 316u)];
#endif
#endif

#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 379)
#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY < 442)
    uint8_t slack7[MEDIA_SECTOR_SIZE*(FS_INTERNAL_FLASH_DRIVE_CAPACITY - 379u)];
#endif
#endif

};

const struct fatfsImage diskImage __attribute__((keep)) __attribute__((address(DRV_NVM_MEDIA_START_ADDRESS))) = 
{

/********************************
 * Master boot record at LBA = 0
 ********************************/
{
    /*************
     * Code Area
     *************/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x0000
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x0010
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x0020
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x0030
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x0040
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x0050
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x0060
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x0070
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x0080
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x0090
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x00A0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x00B0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x00C0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x00D0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x00E0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x00F0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x0100
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x0110
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x0120
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x0130
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x0140
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x0150
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x0160
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x0170
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                                     //0x0180

    /*******************************************************
     * IBM 9 byte/entry x 4 entries primary partition table
     *******************************************************/
    
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                                                             //0x018A
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x0190
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,             //0x01A0

    /* ??? */
    0x00, 0x00,                                                                                     //0x01AE
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,                                                 //0x01B0

    /*********************
     * Disk signature
     ********************/
    0xF5, 0x8B, 0x16, 0xEA,                                                                         //0x01B8

    /* ??? - usually 0x0000 */
    0x00, 0x00,                                                                                     //0x01BC

    /***********************************************************
     * Table of Primary Partitions (16 bytes/entry x 4 entries)
     * Note: Multi-byte fields are in little endian format.
     * Partition Entry 1
     ***********************************************************/                                  //0x01BE

    0x80,                   /* Status - 0x80 (bootable), 0x00 (not bootable), other (error)     */
    0x01, 0x01, 0x00,       /* Cylinder-head-sector address of first sector in partition        */
    0x01,                   /* Partition type - 0x01 = FAT12 up to 2MB                          */
    0x07, 0xFF, 0xE6,       /* Cylinder-head-sector address of last sector in partition         */
    0x01, 0x00, 0x00, 0x00, /* Logical Block Address (LBA) of first sector in partition         */

    (uint8_t)FS_INTERNAL_FLASH_PARTITION_SIZE,
    (uint8_t)(FS_INTERNAL_FLASH_PARTITION_SIZE >> 8),
    (uint8_t)(FS_INTERNAL_FLASH_PARTITION_SIZE >> 16),
    (uint8_t)(FS_INTERNAL_FLASH_PARTITION_SIZE >> 24), /* Length of partition in sectors (MBR sits at LBA = 0, and is not in the partition.) */

    /* Partition Entry 2 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x01CE
    /* Partition Entry 3 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x01DE
    /* Partition Entry 4 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x01EE

    /* MBR signature */
    0x55, 0xAA                                                                                      //0x01FE
},

/*************************************************************
 * Partition boot sector at LBA = 1. Physical Sector - 1, 
 * Logical Sector - 0 of this partition. This is the first
 * sector in the partition, and is known as the "volume boot 
 * record" or "partition boot sector" Note: This table is 
 * filesystem specific.  Re-formatting the drive will
 * overwrite this table.
 *************************************************************/  

{
    0xEB, 0x3C, 0x90,			                    /*  Jump instruction        */
    'M', 'S', 'D', 'O', 'S', '5', '.', '0',	            /*  OEM Name "MSDOS5.0"     */
    (MEDIA_SECTOR_SIZE&0xFF), (MEDIA_SECTOR_SIZE>>8),	    /*  Bytes per sector (MEDIA_SECTOR_SIZE)    */
    0x01,			                            /*  Sectors per cluster                     */
    FS_INTERNAL_FLASH_NUM_RESERVED_SECTORS, 0x00,          /*  Reserved sector count (usually 1 for FAT12 or FAT16)    */
    0x01,			                            /*  number of FATs                                          */
    FS_INTERNAL_FLASH_MAX_NUM_FILES_IN_ROOT, 0x00,         /*  Max number of root directory entries - 16 files allowed */
    0x00, 0x00,			                            /*  total sectors (0x0000 means: use the 4 byte field at offset 0x20 instead)   */
    0xF8,			                            /*  Media Descriptor                                                            */
    FS_INTERNAL_FLASH_NUM_FAT_SECTORS, 0x00,               /*  Sectors per FAT     */
    0x3F, 0x00,	                                            /*  Sectors per track   */
    0xFF, 0x00,                                             /*  Number of heads     */
    0x01, 0x00, 0x00, 0x00,		                    /*  Hidden sectors      */
    (uint8_t)FS_INTERNAL_FLASH_PARTITION_SIZE,
    (uint8_t)(FS_INTERNAL_FLASH_PARTITION_SIZE >> 8),
    (uint8_t)(FS_INTERNAL_FLASH_PARTITION_SIZE >> 16),
    (uint8_t)(FS_INTERNAL_FLASH_PARTITION_SIZE >> 24),	    /*  Total sectors (when WORD value at offset 20 is 0x0000) */

    0x00,			                                /*  Physical drive number       */
    0x00,			                                /*  Reserved("current head")    */
    0x29,			                                /*  Signature                   */
    0x32, 0x67, 0x94, 0xC4,		                        /*  ID(serial number)           */
    'N', 'O', ' ', 'N', 'A', 'M', 'E', ' ', ' ', ' ', ' ',	/*  Volume Label (11 bytes) - "NO NAME    " */
    'F', 'A', 'T', '1', '2', ' ', ' ', ' ',	                /*  FAT system "FAT12   "                   */

    /* Operating system boot code */

    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x55, 0xAA			/* End of sector (0x55AA) */
},

/******************************************************************************
 * First FAT sector at LBA = 2
 * Please see:  http://technet.microsoft.com/en-us/library/cc938438.aspx
 * For short summary on how this table works.
 * Note: This table consists of a series of 12-bit entries, and are fully packed 
 * (no pad bits).  This means every other byte is a "shared" byte, that is split
 * down the middle and is part of two adjacent 12-bit entries.  
 * The entries are in little endian format.
 ******************************************************************************/
{
    0xF8,0x0F,   //Copy of the media descriptor 0xFF8
    0x00,
    0xFF,0x0F    
},

/******************************************************************************
 * Optional additional FAT space here, only needed for drives > ~174kB.
 *****************************************************************************/

#if(FS_INTERNAL_FLASH_NUM_FAT_SECTORS > 1)
{
    0
},
#endif

{
    //Root
    'D','r','i','v','e',' ','N','a','m','e',' ',    /* Drive Name (11 characters, padded with spaces) */
    0x08,                                           /* specify this entry as a volume label */
    0x00,                                           /* Reserved             */
    0x00, 0x00, 0x00, 0x00, 0x11,                   /* Create time          */
    0x00, 0x11,                                     /* Last Access          */
    0x00, 0x00,                                     /* EA-index             */
    0x00, 0x00, 0x00, 0x11,                         /* Last modified time   */
    0x00, 0x00,                                     /* First FAT cluster    */
    0x00, 0x00, 0x00, 0x00,                         /* File Size (number of bytes)  */

    'F','I','L','E',' ',' ',' ',' ',                /* File name (exactly 8 characters)         */
    'T','X','T',                                    /* File extension (exactly 3 characters)    */
    0x20,                                           /* specify this entry as a volume label     */
    0x00,                                           /* Reserved     */
    0x06, 0x28, 0x78, 0xDE, 0x38,                   /* Create time  */
    0xDE, 0x38,                                     /* Last Access  */
    0x00, 0x00,                                     /* EA-index      */
    0x04, 0x77, 0xDE, 0x38,                         /* Last modified */
    0x02, 0x00,                                     /* First FAT cluster */
    0x04, 0x00, 0x00, 0x00,                         /* File Size (number of bytes) */
},

#if (FS_INTERNAL_FLASH_MAX_NUM_FILES_IN_ROOT>16)
{
    0
},
#endif

#if (FS_INTERNAL_FLASH_MAX_NUM_FILES_IN_ROOT>32)
{
    0
},
#endif

#if (FS_INTERNAL_FLASH_MAX_NUM_FILES_IN_ROOT>48)
{
    0
},
#endif

/************************************************************
 * Data Sectors 
 ***********************************************************/  

/*************************************************************************
 * Create a place holder in flash for each of sector of data defined by 
 * the FS_INTERNAL_FLASH_DRIVE_CAPACITY definition.  We will initialize 
 * the the first sector worth placeholder with the ASCII contents "Data".
 * This is the contents of the FILE.TXT, based on our RootDirectory0[] and
 * FAT0[] settings above.
 ***************************************************************************/

#if (FS_INTERNAL_FLASH_DRIVE_CAPACITY>0)
{
    'D','a','t','a'
},
#endif

/******************************************************************************
 * The rest of the MSD volume is unused/blank/not currently filled with any 
 * file(s). However, we still need to declare a uint8_t array to fill the space,
 * so the linker knows not to allocate anything else into this region of flash
 * memory. This array declaration could be:
 * const uint8_t PARTITION_ATTRIBUTES slack1
 *           [MEDIA_SECTOR_SIZE*(FS_INTERNAL_FLASH_DRIVE_CAPACITY - 1u)] = {0};
 * In practice, some compilers will run into limitations when trying to declare
 * arrays with more than 32767 elements.  Therefore, we declare the MSD volume
 * placeholder as a series of arrays instead (each 32767 bytes or less), so as 
 * to support large MSD volume sizes.
 ********************************************************************************/

#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY>1)
    #if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 64)
    {
        0
    },
    #else
    {
        0
    },
    #endif
#endif

#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 64)
    #if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 127)
    {
        0
    },
    #else
    {
        0
    },
    #endif
#endif

#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 127)
    #if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 190)
    {
        0
    },
    #else
    {
        0
    },
    #endif
#endif

#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 190)
    #if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 253)
    {
        0
    },
    #else
    {
        0
    },
    #endif
#endif

#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 253)
    #if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 316)
    {
        0
    },
    #else
    {
        0
    },
    #endif
#endif

#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 316)
    #if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 379)
    {
        0
    },
    #else
    {
        0
    },
    #endif
#endif

#if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 379)
    #if(FS_INTERNAL_FLASH_DRIVE_CAPACITY >= 442)
        #error "Your MSD Volume is larger than this example has provisions for.  Double click this message and add more flash memory placeholder bytes."

        /**********************************************************************
         * If your FS_INTERNAL_FLASH_DRIVE_CAPACITY is > 442 sectors, then you 
         * need to declare more place holder. uint8_t arrays to allocate to the
         * MSD volume.  If you don't do this, the linker might try to "re-use" 
         * the flash memory by placing program code inside the MSD volume,
         * which would cause unanticipated behavior. Please use the existing
         * slack1[] to slack6[] placeholder array declarations as an 
         * example/template to follow, and keep adding as many more slackx[] 
         * arrays as needed to meet your FS_INTERNAL_FLASH_DRIVE_CAPACITY 
         * size requirements.
         *********************************************************************/
    #else
    {
        0
    },
    #endif
#endif

};

