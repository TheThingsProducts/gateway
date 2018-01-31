// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#ifndef _BOOTLOADER_VERSION_H_
#define _BOOTLOADER_VERSION_H_

// A magic number to verify if the struct is filled by the bootloader with valid data
#define BOOTLOADER_VERSION_MAGIC 0x626f6f74   // ascii "boot"
#define BOOTLOADER_VERSION_ADDRESS 0x8007FFF0 // end - 0x10

// The structure containing the bootloader version information
typedef struct
{
    uint32_t magic;
    uint32_t revision;
    uint32_t commit;
    uint32_t timestamp;
} bootloader_version_t;

static inline const bootloader_version_t* BootloaderVersion_Get(void)
{
    // The bootload has stored the bootloader version information at the end of the ram secion
    bootloader_version_t* p = (bootloader_version_t*)BOOTLOADER_VERSION_ADDRESS;

    // Check if the bootloader has filled the version information
    if(p->magic != BOOTLOADER_VERSION_MAGIC)
    {
        // clear the bootloader version information if the magic is not correct
        memset(p, 0, sizeof(*p));
        p->magic = BOOTLOADER_VERSION_MAGIC;
    }

    // Return the pointer to the bootloader version information
    return p;
}

#endif // _BOOTLOADER_VERSION_H_
