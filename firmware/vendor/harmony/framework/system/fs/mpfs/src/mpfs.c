/*******************************************************************************
  Microchip File System (MPFS)

  Company:
    Microchip Technology Inc.

  File Name:
    sys_fs_mpfs.c

  Summary:
    Microchip File System (MPFS) APIs.

  Description:
    This file contains Microchip File System (MPFS) APIs. It is mainly used for
	accessing web pages and other files from internal program memory or an
	external serial EEPROM memory.
*******************************************************************************/
//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include "system/fs/mpfs/src/mpfs_local.h"


/****************************************************************************
  Section: Module-Only Globals and Functions
  ***************************************************************************/

/* MPFSStubs[0] is reserved for internal use (FAT access) */
static MPFS_STUB MPFSStubs [ SYS_FS_MAX_FILES + 1 ];

/* FAT record cache */
static MPFS_FAT_RECORD fatCache;

SYS_MPFS_OBJECT mpfsObject;

/****************************************************************************
 Function pointers
*****************************************************************************/
const SYS_FS_FUNCTIONS MPFSFunctions =
{
    .mount  = MPFS_Mount,
    .unmount = MPFS_Unmount,
    .open   = MPFS_Open,
    .read   = MPFS_Read,
    .write  = NULL,
    .close  = MPFS_Close,
    .seek   = MPFS_Seek,
    .tell   = MPFS_GetPosition,
    .eof    = MPFS_EOF,
    .size   = MPFS_GetSize,
    .fstat   = MPFS_Stat,
    .mkdir = NULL,
    .chdir = NULL,
    .remove = NULL,
    .getlabel = NULL,
    .setlabel = NULL,
    .truncate = NULL,
    .currWD = NULL,
    .chdrive = NULL,
    .chmode = NULL,
    .chtime = NULL,
    .rename = NULL,
    .sync = NULL,
    .getstrn = NULL,
    .putchr = NULL,
    .putstrn = NULL,
    .formattedprint = NULL,
    .testerror = NULL,
    .formatDisk = NULL,
    .openDir = NULL,
    .readDir = NULL,
    .closeDir = NULL,
    .partitionDisk = NULL,
    .getCluster = NULL
};

/****************************************************************************
  Section:Handle Management Functions
*****************************************************************************/


/*****************************************************************************
  Function:
    MPFS_RESULT MPFS_Mount ( uint8_t diskNo);

  Description:
    Mounts a file MPFS file system.

  Precondition:
    None

  Parameters:
    diskNo - disk index in the Filesystem framework
  Returns:
    None.
*/

int MPFS_Mount ( uint8_t diskNo )
{
    uint32_t index;

    mpfsObject.numFiles = 1;
    mpfsObject.isMPFSLocked = false;

    mpfsObject.diskNumber = diskNo;

    /* Take MPFSStubs[0] for MPFS internal usage */
    MPFSStubs[0].basePointer = SYS_FS_MEDIA_MANAGER_AddressGet(diskNo);
    for(index = 1; index <= SYS_FS_MAX_FILES ; index++)
    {
        MPFSStubs[index].addr = MPFS_INVALID;
        MPFSStubs[index].basePointer = SYS_FS_MEDIA_MANAGER_AddressGet(diskNo);
    }
    MPFSStubs[0].addr = 0;
    MPFSStubs[0].bytesRem = 8;


    // Now, using the file object, read the MPFS file and compare content (string compare)
    MPFSGetArray(0, (uint8_t*)&fatCache, 6);
    if(!memcmp((void*)&fatCache, (const void*)"MPFS\x02\x01", 6))
    {
        MPFSGetArray(0, (uint8_t*)&mpfsObject.numFiles, 2);
    }
    else
    {
        mpfsObject.numFiles = 0;
    }

    mpfsObject.fatCacheID = MPFS_INVALID_FAT;

    return MPFS_OK;
}

/*****************************************************************************
  Function:
    MPFS_RESULT MPFS_Unmount ( uint8_t diskNo );

  Description:
    Unmounts a file MPFS file system.

  Precondition:
    None

  Parameters:
    diskNo - disk index in the Filesystem framework
  Returns:
    None.
*/

int MPFS_Unmount ( uint8_t diskNo )
{
    if( diskNo >= SYS_FS_VOLUME_NUMBER )
        return MPFS_DISK_ERR;

    mpfsObject.numFiles = 0;
    mpfsObject.isMPFSLocked = false;
    mpfsObject.diskNumber = 0;
    mpfsObject.fatCacheID = MPFS_INVALID_FAT;

    return MPFS_OK;
}

/*****************************************************************************
  Function:
    int MPFS_Open ( uintptr_t handle, const char* filewithDisk, uint8_t mode )

  Description:
    Opens a file in the MPFS2 file system.

  Precondition:
    None

  Parameters:
    handle          - handle to be used later for reading file and other operation
    filewithDisk    - String of disk number + file name
    mode            - future use. Pass zero in the present implementation.

  Returns:
    success         - MPFS_OK
    failure         - MPFS_NO_FILE
*/

int MPFS_Open ( uintptr_t handle, const char* filewithDisk, uint8_t mode )
{
    uint16_t nameHash = 0, ix = 0;
    uint16_t hashCache[8] = {};
    volatile const char *ptr = (volatile const char *)NULL;
    uint8_t c = 0;
    uint32_t hMPFS = 0;
    volatile const char* cFile = filewithDisk + 3;    // Take the file name without the disk number (ignore "0:/", so increment 3)


    if ( mpfsObject.mpfsMediaHandle == DRV_HANDLE_INVALID )
    {
        /* no opened media/uninitialized */
        return MPFS_INVALID_HANDLE;
    }

    /* Make sure MPFS is unlocked and we got a filename */
    if ( *cFile == '\0' )
    {
        return MPFS_INVALID_HANDLE;
    }

    if ( mpfsObject.isMPFSLocked == true  )
    {
        return MPFS_INVALID_HANDLE;
    }
    /* Calculate the name hash to speed up searching */
    for ( nameHash = 0, ptr = cFile; *ptr != '\0'; ptr++ )
    {
        nameHash += *ptr;
        nameHash <<= 1;
    }

    // Take a free file object as we want to open the MPFS file
    for(hMPFS = 1; hMPFS <= SYS_FS_MAX_FILES ; hMPFS++)
    {
        if( MPFSStubs[hMPFS].addr == MPFS_INVALID ) // not in use, so take it
        {
            break;
        }

    }

    /* If there is already more files opened than the allowed limit */
    if( hMPFS >= SYS_FS_MAX_FILES + 1 )
    {
        return MPFS_INVALID_HANDLE;
    }

    /* Read in hashes, and check remainder on a match.  Store 8 in cache
    for performance. */
    for ( ix = 0; ix < mpfsObject.numFiles; ix++ )
    {
        /* For new block of 8, read in data */
        if ( ( ix & 0x07 ) == 0u )
        {
            MPFSStubs[0].addr = 8 + ix * 2;
            MPFSStubs[0].bytesRem = 16;
            MPFSGetArray ( 0, ( uint8_t* )hashCache, 16 );
        }

        /* If the hash matches, compare the full filename */
        if ( hashCache[ix&0x07] == nameHash )
        {
            _LoadFATRecord(ix);
            MPFSStubs[0].addr = fatCache.string;
            MPFSStubs[0].bytesRem = 255;

            /* Loop over filename to perform comparison */
            for ( ptr = cFile; *ptr != '\0'; ptr++ )
            {
                MPFSGet ( 0, &c );
                if( *ptr != c )
                {
                        break;
                }
            }

            MPFSGet ( 0, &c );

            if ( ( c == '\0' ) && ( *ptr == '\0' ) )
            {
                /* Filename matches, so return true */
                MPFSStubs[hMPFS].addr = fatCache.data;
                MPFSStubs[hMPFS].bytesRem = fatCache.len;
                MPFSStubs[hMPFS].fatID = ix;

                /* Send the reference to the higher layer */
                MPFSStubs[hMPFS].index = hMPFS;
                *(uintptr_t *)handle =  MPFSStubs[hMPFS].index;

                /* Return the status */
                return MPFS_OK;
            }
        }
    }

    /* No file name matched, so return nothing  */
    return MPFS_NO_FILE;
}


/*****************************************************************************
  Function:
    int MPFS_Read ( uintptr_t handle, void* buff, uint16_t btr, uint16_t * br )

  Description:
    Reads a file in the MPFS2 file system.

  Precondition:
    None

  Parameters:
    handle  - a valie handle to the file
    buff    - pointer to buffer to read data
    btr     - number of bytes to read
    br      - pointer to variable which holds the number of bytes actually read

  Returns:
    MPFS_OK
*/

int MPFS_Read ( uintptr_t handle, void* buff, uint32_t btr, uint32_t * br )
{
    MPFS_HANDLE address = ((MPFS_HANDLE )handle);
    *br = MPFSGetArray ( address, buff, btr );


    return MPFS_OK;
}


/*****************************************************************************
  Function:
    int MPFS_Close ( uintptr_t handle )

  Description:
    Closes a file in the MPFS2 file system.

  Precondition:
    None

  Parameters:
    handle - a valie handle to the file

  Returns:
    MPFS_OK;
*/

int MPFS_Close ( uintptr_t handle )
{
    MPFS_HANDLE address = ((MPFS_HANDLE )handle);
    MPFSClose ( address );
    return MPFS_OK;
}


/*****************************************************************************
  Function:
    uint32_t MPFS_GetSize ( uintptr_t handle )

  Description:
    Obtains the size of the file

  Precondition:
    None

  Parameters:
    handle - a valie handle to the file

  Returns:
    The size of file.
*/
uint32_t MPFS_GetSize ( uintptr_t handle )
{
    MPFS_HANDLE address = ((MPFS_HANDLE )handle);

    return (MPFSGetSize ( address ));
}

/*****************************************************************************
  Function:
    uint32_t MPFS_GetPosition ( uintptr_t handle)

  Description:
    Obtains the present file pointer position

  Precondition:
    None

  Parameters:
    handle - a valie handle to the file

  Returns:
    The present file pointer.
*/
uint32_t MPFS_GetPosition ( uintptr_t handle)
{
    MPFS_HANDLE hMPFS = ((MPFS_HANDLE )handle);

    return (MPFSGetPosition ( hMPFS ));
}

/*****************************************************************************
  Function:
    bool MPFS_EOF( uintptr_t handle )

  Description:
    Returns if the present file pointer already reached end of file?

  Precondition:
    None

  Parameters:
    handle - a valie handle to the file

  Returns:
    End of file     - true
    Not end of file - false
*/
bool MPFS_EOF( uintptr_t handle )
{
    MPFS_HANDLE hMPFS = ((MPFS_HANDLE )handle);
    volatile unsigned long len, position;

    len = MPFSGetSize ( hMPFS );
    position = MPFSGetPosition ( hMPFS );

    if(position == len)
        return 1;
    else
        return 0;
//    return (position == len ? 1 : 0);
}

/*****************************************************************************
  Function:
    int MPFS_Stat ( const char* filewithDisk, uintptr_t stat_str )

  Description:
    Returns the status (property) of the file

  Precondition:
    None

  Parameters:
    filewithDisk    -   string containing the Disk number appended to file name
    stat_str        -   pointer to structure which will return the file status

  Returns:
    Success     - MPFS_OK
    Failure      - MPFS_NO_FILE
*/
int MPFS_Stat ( const char* filewithDisk, uintptr_t stat_str )
{
    uint16_t nameHash = 0, ix = 0;
    uint16_t hashCache[8] = {};
    const char *ptr = (const char *)NULL, *ptr1 = (const char *)NULL;
    uint8_t c = 0;
    const char* cFile = filewithDisk + 3;    // Take the file name without the disk number (ignore "0:/", so increment 2)
    unsigned char i = 0;

    MPFS_STATUS *stat = (MPFS_STATUS *) stat_str;


    if ( mpfsObject.mpfsMediaHandle == DRV_HANDLE_INVALID )
    {
        /* no opened media/uninitialized */
        return 1;
    }

    /* Make sure MPFS is unlocked and we got a filename */
    if ( *cFile == '\0' )
    {
        return 1;
    }

    if ( mpfsObject.isMPFSLocked == true  )
    {
        return 1;
    }
    /* Calculate the name hash to speed up searching */
    for ( nameHash = 0, ptr = cFile; *ptr != '\0'; ptr++ )
    {
        nameHash += *ptr;
        nameHash <<= 1;
    }

    /* Read in hashes, and check remainder on a match.  Store 8 in cache
    for performance. */
    for ( ix = 0; ix < mpfsObject.numFiles; ix++ )
    {
        /* For new block of 8, read in data */
        if ( ( ix & 0x07 ) == 0u )
        {
            MPFSStubs[0].addr = 8 + ix * 2;
            MPFSStubs[0].bytesRem = 16;
            MPFSGetArray ( 0, ( uint8_t* )hashCache, 16 );
        }

        /* If the hash matches, compare the full filename */
        if ( hashCache[ix&0x07] == nameHash )
        {
            _LoadFATRecord(ix);
            MPFSStubs[0].addr = fatCache.string;
            MPFSStubs[0].bytesRem = 255;

            /* Loop over filename to perform comparison */
            for ( ptr = cFile; *ptr != '\0'; ptr++ )
            {
                MPFSGet ( 0, &c );
                if( *ptr != c )
                {
                        break;
                }
            }

            MPFSGet ( 0, &c );

            if ( ( c == '\0' ) && ( *ptr == '\0' ) )
            {
                stat->fattrib = fatCache.flags;
                stat->fdate = (unsigned short)(fatCache.timestamp >> 16);
                stat->ftime = (unsigned short)(fatCache.timestamp);
                stat->fsize = fatCache.len;
                for ( ptr1 = cFile, i = 0; *ptr1 != '\0'; ptr1++, i++ )
                {
                    stat->fname[i] = *ptr1;
                }

                /* Return the status */
                return 0;
            }
        }
    }

    /* No file name matched, so return nothing  */
    return MPFS_NO_FILE;
}

/*****************************************************************************
  Function:
	MPFS_HANDLE MPFSOpenID ( uint16_t hFatID )

  Summary:
	Quickly re-opens a file.

  Description:
	Quickly re-opens a file in the MPFS2 file system.  Use this function
	along with MPFSGetID() to quickly re-open a file without tying up
	a permanent MPFSStub.

  Precondition:
	None

  Parameters:
	hFatID - the ID of a previous opened file in the FAT

  Returns:
	An MPFS_HANDLE to the opened file if found, or MPFS_INVALID_HANDLE
	if the file could not be found or no free handles exist.
  ***************************************************************************/

//MPFS_HANDLE MPFSOpenID ( uint16_t hFatID )
//{
//	MPFS_HANDLE hMPFS;
//
//    if ( mpfsObject.mpfsMediaHandle == DRV_HANDLE_INVALID )
//    {
//		/* no opened media/uninitialized */
//		return MPFS_INVALID_HANDLE;
//    }
//
//	/* Make sure MPFS is unlocked and we got a valid id */
//	if( ( mpfsObject.isMPFSLocked == true ) || hFatID > mpfsObject.numFiles )
//	{
//		return MPFS_INVALID_HANDLE;
//	}
//
//	/* Find a free file handle to use */
//	for ( hMPFS = 1; hMPFS <= SYS_FS_MAX_FILES; hMPFS++ )
//	{
//		if ( MPFSStubs[hMPFS].addr == MPFS_INVALID )
//		{
//			break;
//		}
//	}
//	if ( hMPFS == SYS_FS_MAX_FILES )
//	{
//		return MPFS_INVALID_HANDLE;
//	}
//
//	/* Load the FAT record */
//	_LoadFATRecord ( hFatID );
//
//	/* Set up the file handle */
//	MPFSStubs[hMPFS].fatID = hFatID;
//	MPFSStubs[hMPFS].addr = fatCache.data;
//	MPFSStubs[hMPFS].bytesRem = fatCache.len;
//
//	return hMPFS;
//}


/*****************************************************************************
  Function:
	void MPFSClose ( MPFS_HANDLE hMPFS )

  Summary:
	Closes a file.

  Description:
	Closes a file and releases its stub back to the pool of available
	handles.

  Precondition:
	None

  Parameters:
	hMPFS - the file handle to be closed

  Returns:
	None
*****************************************************************************/

void MPFSClose ( MPFS_HANDLE hMPFS )
{
	if ( hMPFS != 0u && ( hMPFS <= SYS_FS_MAX_FILES ) )
    {
	    MPFSStubs[hMPFS].addr = MPFS_INVALID;
    }
}


/****************************************************************************
  Section:
	Data Reading Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	bool MPFSGet ( MPFS_HANDLE hMPFS, uint8_t* c )

  Description:
	Reads a byte from a file.

  Precondition:
	The file handle referenced by hMPFS is already open.

  Parameters:
	hMPFS - the file handle from which to read
	c - Where to store the byte that was read

  Return Values:
	true - The byte was successfully read
	false - No byte was read because either the handle was invalid or
	        the end of the file has been reached.
*/

bool MPFSGet ( MPFS_HANDLE hMPFS, uint8_t* c )
{
	/* Make sure we're reading a valid address */
	if ( hMPFS > SYS_FS_MAX_FILES )
	{
		return false;
	}
	if ( ( MPFSStubs[hMPFS].addr == MPFS_INVALID ) ||
		( MPFSStubs[hMPFS].bytesRem == 0u ) )
	{
		return false;
	}
	if ( c == 0 )
	{
		MPFSStubs[hMPFS].addr++;
		MPFSStubs[hMPFS].bytesRem--;
		return true;
	}

    /* read a character */
    {
        /* failed */
        //return false;
    }
    SYS_FS_MEDIA_MANAGER_Read (mpfsObject.diskNumber, c, ((uint8_t *) MPFSStubs[hMPFS].basePointer + MPFSStubs[hMPFS].addr ), 1);

    MPFSStubs[hMPFS].addr++;
    MPFSStubs[hMPFS].bytesRem--;

    return true;
}


/*****************************************************************************
  Function:
	uint32_t MPFSGetArray ( MPFS_HANDLE hMPFS, uint8_t* cData, uint32_t wLen )

  Description:
	Reads a series of bytes from a file.

  Precondition:
	The file handle referenced by hMPFS is already open.

  Parameters:
	hMPFS - the file handle from which to read
	cData - where to store the bytes that were read
	wLen - how many bytes to read

  Returns:
	The number of bytes successfully read.  If this is less than wLen,
	an EOF occurred while attempting to read.
*/

uint32_t MPFSGetArray ( MPFS_HANDLE hMPFS, uint8_t* cData, uint32_t wLen )
{
    /* Make sure we're reading a valid address */
    if ( hMPFS > SYS_FS_MAX_FILES ) // To accomodate "MPFS_Stat"
    {
        return 0;
    }
    /* Determine how many we can actually read */
    if(wLen > MPFSStubs[hMPFS].bytesRem)
    {
        wLen = MPFSStubs[hMPFS].bytesRem;
    }
    /* Make sure we're reading a valid address */
    if(MPFSStubs[hMPFS].addr == MPFS_INVALID || wLen == 0u)
    {
        return 0;
    }
    if(cData == 0)
    {
        MPFSStubs[hMPFS].addr += wLen;
        MPFSStubs[hMPFS].bytesRem -= wLen;
        return wLen;
    }

    /* Read the data */
    SYS_FS_MEDIA_MANAGER_Read (mpfsObject.diskNumber, cData,
            ((uint8_t *)MPFSStubs[hMPFS].basePointer + MPFSStubs[hMPFS].addr), wLen );
    {
        /* failed */
        //return 0;
    }

    MPFSStubs[hMPFS].addr += wLen;
    MPFSStubs[hMPFS].bytesRem -= wLen;
    return wLen;
}


/*****************************************************************************
  Function:
	bool MPFSGetLong ( MPFS_HANDLE hMPFS, uint32_t* ul )

  Description:
	Reads a uint32_t or Long value from the MPFS.

  Precondition:
	The file handle referenced by hMPFS is already open.

  Parameters:
	hMPFS - the file handle from which to read
	ul - where to store the uint32_t or long value that was read

  Returns:
	true - The byte was successfully read
	false - No byte was read because either the handle was invalid or
	        the end of the file has been reached.
*/

bool MPFSGetLong ( MPFS_HANDLE hMPFS, uint32_t* ul )
{
	return ( MPFSGetArray ( hMPFS, ( uint8_t* )ul, 4 ) == 4u );
}


/*****************************************************************************
  Function:
	bool MPFSSeek(MPFS_HANDLE hMPFS, uint32_t dwOffset, MPFS_SEEK_MODE tMode)

  Description:
	Moves the current read pointer to a new location.

  Precondition:
	The file handle referenced by hMPFS is already open.

  Parameters:
	hMPFS - the file handle to seek with
	dwOffset - offset from the specified position in the specified direction
	tMode - one of the MPFS_SEEK_MODE constants

  Returns:
	true - the seek was successful
	false - either the new location or the handle itself was invalid
*/

//bool MPFSSeek ( MPFS_HANDLE hMPFS, uint32_t dwOffset, MPFS_SEEK_MODE tMode )
//{
//	uint32_t temp;
//
//	/* Make sure a valid file is open */
//	if( hMPFS > SYS_FS_MAX_FILES )
//	{
//		return false;
//	}
//
//	/* MPFS address is may not be valid */
//	if( MPFSStubs[hMPFS].addr == MPFS_INVALID )
//	{
//		return false;
//	}
//
//	switch ( tMode )
//	{
//		/* Seek offset bytes from start */
//		case MPFS_SEEK_START:
//			temp = MPFSGetSize ( hMPFS );
//			if ( dwOffset > temp )
//			{
//				return false;
//			}
//			MPFSStubs[hMPFS].addr = MPFSGetStartAddr( hMPFS ) + dwOffset;
//			MPFSStubs[hMPFS].bytesRem = temp - dwOffset;
//			return true;
//
//		/* Seek forwards offset bytes */
//		case MPFS_SEEK_FORWARD:
//			if ( dwOffset > MPFSStubs[hMPFS].bytesRem )
//			{
//				return false;
//			}
//			MPFSStubs[hMPFS].addr += dwOffset;
//			MPFSStubs[hMPFS].bytesRem -= dwOffset;
//			return true;
//
//		/* Seek backwards offset bytes */
//		case MPFS_SEEK_REWIND:
//			temp = MPFSGetStartAddr ( hMPFS );
//
//			if ( MPFSStubs [ hMPFS ].addr < ( temp + dwOffset ) )
//			{
//				return false;
//			}
//			MPFSStubs[hMPFS].addr -= dwOffset;
//			MPFSStubs[hMPFS].bytesRem += dwOffset;
//			return true;
//
//		/* Seek so that offset bytes remain in file */
//		case MPFS_SEEK_END:
//			temp = MPFSGetSize( hMPFS );
//			if( dwOffset > temp )
//			{
//				return false;
//			}
//			MPFSStubs[hMPFS].addr = MPFSGetEndAddr ( hMPFS ) - dwOffset;
//			MPFSStubs[hMPFS].bytesRem = dwOffset;
//
//			return true;
//		default:
//			return false;
//	}
//}
int MPFS_Seek ( uintptr_t handle, uint32_t dwOffset )
{
	uint32_t temp;
        MPFS_HANDLE hMPFS = ((MPFS_HANDLE )handle);

	/* Make sure a valid file is open */
	if( hMPFS > SYS_FS_MAX_FILES )
	{
		return 1;
	}

	/* MPFS address is may not be valid */
	if( MPFSStubs[hMPFS].addr == MPFS_INVALID )
	{
		return 1;
	}

        temp = MPFSGetSize ( hMPFS );
        if ( dwOffset > temp )
        {
                return 1;
        }
        MPFSStubs[hMPFS].addr = MPFSGetStartAddr( hMPFS ) + dwOffset;
        MPFSStubs[hMPFS].bytesRem = temp - dwOffset;
        return 0;
}

/****************************************************************************
  Section:Data Writing Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	MPFS_HANDLE MPFSFormat ( void )

  Summary:
	Prepares the MPFS image for writing.

  Description:
	Prepares the MPFS image for writing and locks the image so that other
	processes may not access it.

  Precondition:
	None

  Parameters:
	None

  Returns:
	An MPFS handle that can be used for MPFSPut commands, or
	MPFS_INVALID_HANDLE when the EEPROM failed to initialize for writing.

  Remarks:
	In order to prevent misreads, the MPFS will be inaccessible until
	MPFSClose is called.  This function is not available when the MPFS
	is stored in internal Flash program memory.
*/

//MPFS_HANDLE MPFSFormat ( void )
//{
//	uint8_t ix;
//
//	/* Close all files */
//	for(ix = 0; ix < SYS_FS_MAX_FILES; ix++)
//    {
//		MPFSStubs[ix].addr = MPFS_INVALID;
//    }
//
//	/* Lock the image */
//	mpfsObject.isMPFSLocked = true;
//
//    /* Set FAT ptr for writing */
//    MPFSStubs[0].addr = 0;
//    MPFSStubs[0].fatID = 0xffff;
////????????    MPFSStubs[0].bytesRem = DRV_NVM_ClientSizeGet ( mpfsObject.mpfsMediaHandle );
//
// //????????????   if ( DRV_MEDIA_WriteSetOffset ( mpfsObject.mpfsMediaHandle, 0 ) != DRV_CLIENT_STATUS_READY )
//    {
//		/* failed */
//        return MPFS_INVALID_HANDLE;
//    }
//
//    return 0;
//}


/*****************************************************************************
  Function:
	uint32_t MPFSPutArray ( MPFS_HANDLE hMPFS, uint8_t *cData, uint32_t wLen )

  Description:
	Writes an array of data to the MPFS image.

  Precondition:
	MPFSFormat was sucessfully called.

  Parameters:
	hMPFS - the file handle for writing
	cData - the array of bytes to write
	wLen - how many bytes to write

  Returns:
	The number of bytes successfully written.

  Remarks:
	For EEPROM, the actual write may not initialize until the internal write
	page is full.  To ensure that previously written data gets stored,
	MPFSPutEnd must be called after the last call to MPFSPutArray.
*/
/*
uint32_t MPFSPutArray ( MPFS_HANDLE hMPFS, uint8_t* cData, uint32_t wLen )
{
    if ( hMPFS == 0 )
    {
        return DRV_NVM_Write ( mpfsObject.mpfsMediaHandle, cData, wLen );
    }

    return 0;
}
*/
/*****************************************************************************
  Function:
	void MPFSPutEnd ( void )

  Description:
	Finalizes an MPFS writing operation.

  Precondition:
	MPFSFormat and MPFSPutArray were sucessfully called.

  Parameters:
	final - true if the application is done writing, false if MPFS2 called
		this function locally.

  Returns:
	None
*/

//bool MPFSPutEnd(MPFS_HANDLE hMPFS, bool final)
//{
//    int res;
//
//    if ( hMPFS == 0 )
//    {
//        mpfsObject.isMPFSLocked = false;
//        //??????????????????res = DRV_MEDIA_WriteFlush ( mpfsObject.mpfsMediaHandle );
//
//        if ( res == 0 )
//        {
//            /* success */
//			if ( final )
//            {
//                _Validate();
//            }
//        }
//
//        return res == 0;
//    }
//
//    return false;
//}


/****************************************************************************
  Section: Meta Data Accessors
*****************************************************************************/

/*****************************************************************************
  Function:
	static void _LoadFATRecord ( uint16_t fatID )

  Description:
	Loads the FAT record for a specified handle.

  Precondition:
	None

  Parameters:
	fatID - the ID of the file whose FAT is to be loaded

  Returns:
	None

  Remarks:
	The FAT record will be stored in fatCache.
*/

static void _LoadFATRecord ( uint16_t fatID )
{
    if ( ( fatID == mpfsObject.fatCacheID ) || ( fatID >= mpfsObject.numFiles ) )
    {
        return;
    }
    /* Read the FAT record to the cache */
    MPFSStubs[0].bytesRem = 22;
    MPFSStubs[0].addr = 8 + ( mpfsObject.numFiles * 2 ) + ( fatID * 22 );

    MPFSGetArray ( 0, ( uint8_t* )&fatCache, 22 );

    mpfsObject.fatCacheID = fatID;
}


/*****************************************************************************
  Function:
	uint32_t MPFSGetTimestamp ( MPFS_HANDLE hMPFS )

  Description:
	Reads the timestamp for the specified file.

  Precondition:
	The file handle referenced by hMPFS is already open.

  Parameters:
	hMPFS - the file handle from which to read the metadata

  Returns:
	The timestamp that was read as a uint32_t
*/

uint32_t MPFSGetTimestamp ( MPFS_HANDLE hMPFS )
{
	/* Make sure a valid file is open */
	if ( hMPFS > SYS_FS_MAX_FILES )
	{
		return 0x00000000;
	}
	if ( MPFSStubs[hMPFS].addr == MPFS_INVALID )
	{
		return 0x00000000;
	}

	/* Move to the point for reading */
	_LoadFATRecord ( MPFSStubs[hMPFS].fatID );

	return fatCache.timestamp;
}

/*****************************************************************************
  Function:
	uint32_t MPFSGetMicrotime ( MPFS_HANDLE hMPFS )

  Description:
	Reads the microtime portion of a file's timestamp.

  Precondition:
	The file handle referenced by hMPFS is already open.

  Parameters:
	hMPFS - the file handle from which to read the metadata

  Returns:
	The microtime that was read as a uint32_t
*/

uint32_t MPFSGetMicrotime ( MPFS_HANDLE hMPFS )
{
	/* Make sure a valid file is open */
	if( hMPFS > SYS_FS_MAX_FILES )
	{
		return 0x00000000;
	}
	if ( MPFSStubs[hMPFS].addr == MPFS_INVALID )
	{
		return 0x00000000;
	}

	/* Move to the point for reading */
	_LoadFATRecord( MPFSStubs[hMPFS].fatID );

	return fatCache.microtime;
}

/*****************************************************************************
  Function:
	uint16_t MPFSGetFlags ( MPFS_HANDLE hMPFS )

  Description:
	Reads a file's flags.

  Precondition:
	The file handle referenced by hMPFS is already open.

  Parameters:
	hMPFS - the file handle from which to read the metadata

  Returns:
	The flags that were associated with the file
*/

uint16_t MPFSGetFlags(MPFS_HANDLE hMPFS)
{
	/* Make sure a valid file is open */
	if ( hMPFS > SYS_FS_MAX_FILES )
	{
		return 0x0000;
	}
	if ( MPFSStubs[hMPFS].addr == MPFS_INVALID )
	{
		return 0x0000;
	}
	/* Move to the point for reading */
	_LoadFATRecord ( MPFSStubs[hMPFS].fatID );

	return fatCache.flags;
}


/*****************************************************************************
  Function:
	uint32_t MPFSGetSize ( MPFS_HANDLE hMPFS )

  Description:
	Reads the size of a file.

  Precondition:
	The file handle referenced by hMPFS is already open.

  Parameters:
	hMPFS - the file handle from which to read the metadata

  Returns:
	The size that was read as a uint32_t
*/

uint32_t MPFSGetSize ( MPFS_HANDLE hMPFS )
{
	/* Make sure a valid file is open */
	if ( hMPFS > SYS_FS_MAX_FILES )
	{
		return 0x00000000;
	}
	if ( MPFSStubs[hMPFS].addr == MPFS_INVALID )
	{
		return 0x00000000;
	}

	/* Move to the point for reading */
	_LoadFATRecord(MPFSStubs[hMPFS].fatID);

	return fatCache.len;
}

/*****************************************************************************
  Function:
	uint32_t MPFSGetBytesRem(MPFS_HANDLE hMPFS)

  Description:
	Determines how many bytes remain to be read.

  Precondition:
	The file handle referenced by hMPFS is already open.

  Parameters:
	hMPFS - the file handle from which to read the metadata

  Returns:
	The number of bytes remaining in the file as a uint32_t
*/

uint32_t MPFSGetBytesRem ( MPFS_HANDLE hMPFS )
{
	/* Make sure a valid file is open */
	if( hMPFS > SYS_FS_MAX_FILES )
	{
		return 0x00000000;
	}
	if ( MPFSStubs[hMPFS].addr == MPFS_INVALID )
	{
		return 0x00000000;
	}

	return MPFSStubs[hMPFS].bytesRem;
}


/*****************************************************************************
  Function:
	MPFS_PTR MPFSGetStartAddr ( MPFS_HANDLE hMPFS )

  Description:
	Reads the starting address of a file.

  Precondition:
	The file handle referenced by hMPFS is already open.

  Parameters:
	hMPFS - the file handle from which to read the metadata

  Returns:
	The starting address of the file in the MPFS image
*/

MPFS_PTR MPFSGetStartAddr ( MPFS_HANDLE hMPFS )
{
	/* Make sure a valid file is open */
	if ( hMPFS > SYS_FS_MAX_FILES )
	{
		return 0;
	}
	if( MPFSStubs[hMPFS].addr == MPFS_INVALID )
	{
		return MPFS_INVALID;
	}

	/* Move to the point for reading */
	_LoadFATRecord(MPFSStubs[hMPFS].fatID);

	return fatCache.data;
}

/*****************************************************************************
  Function:
	MPFS_PTR MPFSGetEndAddr ( MPFS_HANDLE hMPFS )

  Description:
	Determines the ending address of a file.

  Precondition:
	The file handle referenced by hMPFS is already open.

  Parameters:
	hMPFS - the file handle from which to read the metadata

  Returns:
	The address just after the file ends (start address of next file)
*/

MPFS_PTR MPFSGetEndAddr ( MPFS_HANDLE hMPFS )
{
	/* Make sure a valid file is open */
	if ( hMPFS > SYS_FS_MAX_FILES )
	{
		return MPFS_INVALID;
	}

	if ( MPFSStubs[hMPFS].addr == MPFS_INVALID )
	{
		return MPFS_INVALID;
	}
	/* Move to the point for reading */
	_LoadFATRecord ( MPFSStubs[hMPFS].fatID );

	return fatCache.data + fatCache.len;
}


/*****************************************************************************
  Function:
	bool MPFSGetFilename ( MPFS_HANDLE hMPFS, uint8_t* cName, uint16_t wLen )

  Description:
	Reads the file name of a file that is already open.

  Precondition:
	The file handle referenced by hMPFS is already open.

  Parameters:
	hMPFS - the file handle from which to determine the file name
	cName - where to store the name of the file
	wLen - the maximum length of data to store in cName

  Return Values:
	true - the file name was successfully located
	false - the file handle provided is not currently open
*/

bool MPFSGetFilename ( MPFS_HANDLE hMPFS, uint8_t* cName, uint16_t wLen )
{
	uint32_t addr;

	/* Make sure a valid file is open */
	if ( hMPFS > SYS_FS_MAX_FILES )
	{
		return false;
	}
	if (MPFSStubs[hMPFS].addr == MPFS_INVALID )
	{
		return false;
	}

	/* Move to the point for reading */
	_LoadFATRecord ( MPFSStubs[hMPFS].fatID );

	addr = fatCache.string;
	MPFSStubs[0].addr = addr;
	MPFSStubs[0].bytesRem = 255;

	/* Read the value and return */
	MPFSGetArray(0, cName, wLen);

	return true;
}

/*****************************************************************************
  Function:
	uint32_t MPFSGetPosition(MPFS_HANDLE hMPFS)

  Description:
	Determines the current position in the file

  Precondition:
	The file handle referenced by hMPFS is already open.

  Parameters:
	hMPFS - the file handle for which to determine position

  Returns:
	The position in the file as a uint32_t (or MPFS_PTR)

  Remarks:
	Calling MPFSSeek(hMPFS, pos, MPFS_SEEK_START) will return the pointer
	to this position at a later time.  (Where pos is the value returned by
	this function.)
*/

uint32_t MPFSGetPosition ( MPFS_HANDLE hMPFS )
{
	if ( hMPFS > SYS_FS_MAX_FILES )
	{
		return 0;
	}

	return MPFSStubs[hMPFS].addr - MPFSGetStartAddr ( hMPFS );
}


/*****************************************************************************
  Function:
	uint16_t MPFSGetID ( MPFS_HANDLE hMPFS )

  Description:
	Determines the ID in the FAT for a file.

  Precondition:
	The file handle referenced by hMPFS is already open.

  Parameters:
	hMPFS - the file handle from which to read the metadata

  Returns:
	The ID in the FAT for this file

  Remarks:
	Use this function in association with MPFSOpenID to quickly access file
	without permanently reserving a file handle.
*/

uint16_t MPFSGetID ( MPFS_HANDLE hMPFS )
{
	return MPFSStubs[hMPFS].fatID;
}


/****************************************************************************
  Section: 	Utility Functions
  ***************************************************************************/

/*****************************************************************************
  Function:
	void _Validate ( void )

  Summary:
	Validates the MPFS Image

  Description:
	Verifies that the MPFS image is valid, and reads the number of
	available files from the image header.  This function is called on
	boot, and again after any image is written.

  Precondition:
	None

  Parameters:
	None

  Returns:
	None
*/

//static void _Validate(void)
//{
//    /* If this function causes an Address Error Exception on 16-bit
//    platforms with code stored in internal Flash, make sure your
//    compiler memory model settings are correct.
//
//    In MPLAB, choose Project Menu > Build Options > Project.
//    Select the MPLAB C30 tab and change Cagetory to Memory Model.
//    Ensure that Large Code Model is selected, and that the remaining
//    options are set to Default. */
//
//    /* Validate the image and update numFiles */
//    MPFSStubs[0].addr = 0;
//    MPFSStubs[0].bytesRem = 8;
//    MPFSGetArray ( 0, ( uint8_t* ) &fatCache, 6 );
//    if ( !memcmp ( ( void* )&fatCache, ( const void* )"MPFS\x02\x01", 6 ) )
//    {
//        MPFSGetArray ( 0, ( uint8_t* )&mpfsObject.numFiles, 2 );
//    }
//    else
//    {
//        mpfsObject.numFiles = 0;
//    }
//    mpfsObject.numFiles = 1;
//    mpfsObject.fatCacheID = MPFS_INVALID_FAT;
//}

