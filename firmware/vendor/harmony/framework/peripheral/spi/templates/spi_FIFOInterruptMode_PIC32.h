/*******************************************************************************
  SPI Peripheral Library Template Implementation

  File Name:
    spi_FIFOInterruptMode_PIC32.h

  Summary:
    SPI PLIB Template Implementation

  Description:
    This header file contains template implementations
    For Feature : FIFOInterruptMode
    and its Variant : PIC32
    For following APIs :
        PLIB_SPI_FIFOInterruptModeSelect
        PLIB_SPI_ExistsFIFOInterruptMode

*******************************************************************************/

//DOM-IGNORE-BEGIN
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

//DOM-IGNORE-END

#ifndef _SPI_FIFOINTERRUPTMODE_PIC32_H
#define _SPI_FIFOINTERRUPTMODE_PIC32_H

//******************************************************************************
/* Routines available for accessing VREGS, MASKS, POS, LEN are 

  VREGs: 
    _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_VREG(index)
    _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_VREG(index)

  MASKs: 
    _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_MASK(index)
    _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_MASK(index)

  POSs: 
    _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_POS(index)
    _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_POS(index)

  LENs: 
    _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_LEN(index)
    _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_LEN(index)

*/


//******************************************************************************
/* Function :  SPI_FIFOInterruptModeSelect_PIC32

  Summary:
    Implements PIC32 variant of PLIB_SPI_FIFOInterruptModeSelect 

  Description:
    This template implements the PIC32 variant of the PLIB_SPI_FIFOInterruptModeSelect function.
*/

PLIB_TEMPLATE void SPI_FIFOInterruptModeSelect_PIC32( SPI_MODULE_ID index , SPI_FIFO_INTERRUPT mode )
{
  switch( mode )
    {
        case SPI_FIFO_INTERRUPT_WHEN_TRANSMIT_BUFFER_IS_NOT_FULL:
				_SFR_FIELD_WRITE( _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_VREG( index ),
								  _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_MASK( index ),
								  _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_POS( index ),
								  3 );							  
            break;
        case SPI_FIFO_INTERRUPT_WHEN_TRANSMIT_BUFFER_IS_1HALF_EMPTY_OR_MORE:
				_SFR_FIELD_WRITE( _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_VREG( index ),
								  _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_MASK( index ),
								  _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_POS( index ),
								  2 );								
            break;
        case SPI_FIFO_INTERRUPT_WHEN_TRANSMIT_BUFFER_IS_COMPLETELY_EMPTY:
				_SFR_FIELD_WRITE( _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_VREG( index ),
								  _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_MASK( index ),
								  _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_POS( index ),
								  1 );								
            break;			
        case SPI_FIFO_INTERRUPT_WHEN_TRANSMISSION_IS_COMPLETE:
				_SFR_FIELD_WRITE( _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_VREG( index ),
								  _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_MASK( index ),
								  _SPI_TX_FIFO_INTERRUPT_MODE_PIC32_POS( index ),
								  0 );								
            break;	
        case SPI_FIFO_INTERRUPT_WHEN_RECEIVE_BUFFER_IS_FULL:
				_SFR_FIELD_WRITE( _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_VREG( index ),
								  _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_MASK( index ),
								  _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_POS( index ),
								  3 );							  
            break;
        case SPI_FIFO_INTERRUPT_WHEN_RECEIVE_BUFFER_IS_1HALF_FULL_OR_MORE:
				_SFR_FIELD_WRITE( _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_VREG( index ),
								  _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_MASK( index ),
								  _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_POS( index ),
								  2 );								
            break;
        case SPI_FIFO_INTERRUPT_WHEN_RECEIVE_BUFFER_IS_NOT_EMPTY:
				_SFR_FIELD_WRITE( _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_VREG( index ),
								  _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_MASK( index ),
								  _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_POS( index ),
								  1 );								
            break;			
        case SPI_FIFO_INTERRUPT_WHEN_BUFFER_IS_EMPTY:
				_SFR_FIELD_WRITE( _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_VREG( index ),
								  _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_MASK( index ),
								  _SPI_RX_FIFO_INTERRUPT_MODE_PIC32_POS( index ),
								  0 );								
            break;				
        default:
            break;
    }
}


//******************************************************************************
/* Function :  SPI_ExistsFIFOInterruptMode_PIC32

  Summary:
    Implements PIC32 variant of PLIB_SPI_ExistsFIFOInterruptMode

  Description:
    This template implements the PIC32 variant of the PLIB_SPI_ExistsFIFOInterruptMode function.
*/

#define PLIB_SPI_ExistsFIFOInterruptMode PLIB_SPI_ExistsFIFOInterruptMode
PLIB_TEMPLATE bool SPI_ExistsFIFOInterruptMode_PIC32( SPI_MODULE_ID index )
{
    return true;
}


#endif /*_SPI_FIFOINTERRUPTMODE_PIC32_H*/

/******************************************************************************
 End of File
*/

