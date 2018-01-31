/*******************************************************************************
 Module for Microchip Graphics Library

  Company:
    Microchip Technology Inc.

  File Name:
    gfx_palette.h

  Summary:
    The header file defines all palette APIs used in the graphics library.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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
 
#ifndef _GFX_PALETTE_H
// DOM-IGNORE-BEGIN
    #define _GFX_PALETTE_H
// DOM-IGNORE-END

#include "system_config.h"
#include "gfx/gfx_primitive.h"
#include <stdint.h>
#include "gfx/gfx_types_resource.h"
#include "gfx/gfx_types_palette.h"

#ifdef __cplusplus
    extern "C" {
#endif
        
#if defined(GFX_LIB_CFG_USE_PALETTE)
/*********************************************************************
* Function: void GFX_PaletteInit(void)
*
* Overview: Initializes the color look up table (CLUT).
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: All rendering will use the new palette entries.
*
********************************************************************/
void    GFX_PaletteInit(void);

/*********************************************************************
* Function: void GFX_PaletteEnable(void)
*
* Overview: Enables the Palette mode.
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects:
*
********************************************************************/
void    GFX_PaletteEnable(void);

/*********************************************************************
* Function: void DisablePalette(void)
*
* Overview: Disables the Palette mode.
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects:
*
********************************************************************/
void    GFX_PaletteDisable(void);

/*********************************************************************
* Function: uint8_t GFX_IsPaletteEnabled(void)
*
* Overview: Returns if the Palette mode is enabled or not.
*
* PreCondition: none
*
* Input: none
*
* Output: Returns the palette mode status.
*		  1 - If the palette mode is enabled 
*		  0 - If the palette mode is disabled 
*
* Side Effects:
*
********************************************************************/
uint8_t    GFX_IsPaletteEnabled(void);

/*********************************************************************
* Function: uint8_t GFX_PaletteChangeErrorGet(void)
*
* Overview: Returns the Palette change error status
*
* PreCondition: none
*
* Input: none
*
* Output: Returns the palette change status.
*		  1 - If the palette change error occurred
*		  0 - If no palette change error occurred
*
* Side Effects: none
*
********************************************************************/
uint8_t    GFX_PaletteChangeErrorGet(void);

/*********************************************************************
* Function: void GFX_PaletteChangeErrorClear(void)
*
* Overview: Clears the Palette change error status
*
* PreCondition: none
*
* Input: none
*
* Output: none
*
* Side Effects: none
*
********************************************************************/
void    GFX_PaletteChangeErrorClear(void);

/*********************************************************************
* Function: uint8_t GFX_PaletteBppSet(uint8_t bpp)
*
* Overview: Sets the color look up table (CLUT) number of valid entries.
*
* PreCondition: Palette must be initialized by PaletteInit().
*
* Input: bpp - Bits per pixel
*
* Output: Returns the status of the change.
*		  0 - Change was successful
*		  1 - Change was not successful
*
* Side Effects: none
*
********************************************************************/
uint8_t    GFX_PaletteBppSet(uint8_t bpp);

/*********************************************************************
* Function: void GFX_PaletteChangeRequest(GFX_RESOURCE_HDR *pPalette, uint16_t startEntry, uint16_t length)
*
* Overview: Loads the palettes from the flash during vertical blanking period
*           if vertical blanking period is present (TFT mode), otherwise
*           this function will load the palette immediately.
*
* PreCondition: Palette must be initialized by PaletteInit().
*
* Input: pPalette   - Pointer to the palette resource
*        startEntry - Start entry to load (inclusive)
*        length     - Number of entries
*
* Output: none
*
* Side Effects: There may be a slight flicker when the Palette entries
*               are getting loaded one by one.
*
********************************************************************/
void    GFX_PaletteChangeRequest(GFX_RESOURCE_HDR *pPalette, uint16_t startEntry, uint16_t length);

/*********************************************************************
* Macro: GFX_EntirePaletteChangeRequest(pPalette)
*
* Overview: Loads all the palette entries from the flash during
*           vertical blanking period if possible, otherwise
*           loads immediately.
*
* PreCondition: PPalette must be initialized by PaletteInit().
*
* Input: pPalette - Pointer to the palette resource
*
* Output: none
*
* Side Effects: There may be a slight flicker when the Palette entries
*               are getting loaded one by one.
*
********************************************************************/
#define GFX_EntirePaletteChangeRequest(pPalette)    RequestPaletteChange(pPalette, 0, 256)

/*********************************************************************
* Function: uint8_t GFX_PaletteSet(GFX_RESOURCE_HDR *pPalette, uint16_t startEntry, uint16_t length)
*
* Overview: Programs a block of palette entries starting from startEntry and 
*			until startEntry + length from the flash immediately.
*
* PreCondition: Palette must be initialized by PaletteInit().
*
* Input: pPalette   - Pointer to the palette resource
*        startEntry - Start entry to load (inclusive)
*        length     - Number of entries
*
* Output: Returns the status of the palette set.
*                 - 3 - GFX_RESOURCE_HDR type is not a valid type palette 
*                 - 2 - pPalette is NULL
*                 - 1 - startingEntry + length > max entries
*                 - 0 - palette is set with no errors.
*
* Side Effects: There may be a slight flicker when the Palette entries
*               are getting loaded one by one.
*
********************************************************************/
uint8_t    GFX_PaletteSet(GFX_RESOURCE_HDR *pPalette, uint16_t startEntry, uint16_t length);

/*********************************************************************
* Macro: GFX_EntirePaletteSet(pPalette)
*
* Overview: Programs the whole 256 entry palette with new color values
*			from flash.
*
* PreCondition: Palette must be initialized by PaletteInit().
*
* Input: pPalette - Pointer to the palette resource
*
* Output: Returns the status of the palette set.
*		  0 - Set was successful
*		  1 - Set was not successful
*
* Side Effects: There may be a slight flicker when the Palette entries
*               are getting loaded one by one.
*
********************************************************************/
#define GFX_EntirePaletteSet(pPalette)  SetPalette(pPalette, 0, 256)

/*********************************************************************
* Function: uint8_t GFX_PaletteFlashSet(const PALETTE_ENTRY *pPaletteEntry, uint16_t startEntry, uint16_t length)
*
* Overview: Loads the palettes from the flash immediately.
*
* PreCondition: Palette must be initialized by PaletteInit().
*
* Input: pPaletteEntry   - Pointer to the palette table in ROM
*        startEntry      - Start entry to load (inclusive)
*        length          - Number of entries
*
* Output: Returns the status of the palette set.
*		  0 - Set was successful
*		  1 - Set was not successful
*
* Side Effects: There may be a slight flicker when the Palette entries
*               are getting loaded one by one.
*
********************************************************************/
uint8_t    GFX_PaletteFlashSet(GFX_FONT_SPACE GFX_PALETTE_ENTRY *pPaletteEntry, uint16_t startEntry, uint16_t length);

#endif 

#ifdef __cplusplus
    }
#endif
    
#endif // _GFX_PALETTE_H
