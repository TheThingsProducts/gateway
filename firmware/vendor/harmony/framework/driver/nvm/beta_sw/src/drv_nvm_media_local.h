#ifndef _DRV_NVM_MEDIA_LOCAL_H_
#define _DRV_NVM_MEDIA_LOCAL_H_

#include "driver/nvm/beta_sw/drv_nvm.h"
#include "driver/nvm/beta_sw/drv_nvm_media.h"


typedef struct _DRV_NVM_MEDIA_OBJECT
{
    uintptr_t   mediaStartAddress;
    int         disk;
    uint32_t    nSectors;
    uint16_t    sectorSizeInBytes;

    DRV_HANDLE  drvNVMClientHandle;
    SYS_STATUS  status;

    SYS_FS_MEDIA_STATUS             mediaState;
    uint8_t* buffer;
}
DRV_NVM_MEDIA_OBJECT;

/****************************************************************
 * For an application like File system, the NVM media need to be
 * registered with Media manager. I such case, declare the following
 * as 1.
 * For other applications such as Bootloader, there is no need to
 * register the NVM media with Media maanger. Hence, declare the
 * following as 0.
 *****************************************************************/

#define DRV_NVM_MEDIA_MANAGER_USE       1

#endif
