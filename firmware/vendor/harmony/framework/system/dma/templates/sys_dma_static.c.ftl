/*******************************************************************************
  DMA System Service Functions for Static Single Instance System Service

  Company:
    Microchip Technology Inc.

  File Name:
    drv_dma_static.c

  Summary:
    DMA System Service implementation for the static single instance System Service.

  Description:
    The DMA device System Service provides a simple interface to manage the DMA
    modules on Microchip microcontrollers.
    
  Remarks:
    Static interfaces incorporate the System Service instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    Static single-open interfaces also eliminate the need for the open handle.
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
<#macro DRV_DMA_STATIC_FUNCTIONS INSTANCE DMA_CHANNEL 
DMA_CELL_SIZE DMA_TRIGGER DMA_SOURCE_SIZE DMA_DESTINATION_SIZE DMA_TRIGGER
DMA_SOURCE_ADDRESS DMA_DESTINATION_ADDRESS M_INT_SRC
M_SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE 
M_SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE
M_SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE
M_DRV_DMA_CALL_BACK_FUNCTION
>
// *****************************************************************************
// *****************************************************************************
// Section: DMA Channel ${INSTANCE} functions
// *****************************************************************************
// *****************************************************************************	

void SYS_DMA_Channel${INSTANCE}TransferAdd(void)
{
   
   /* Set the source and destination addresses (addresses are converted from virtual to physical) */
   PLIB_DMA_ChannelXSourceStartAddressSet(DMA_ID_0, ${DMA_CHANNEL}, ${DMA_SOURCE_ADDRESS});
   PLIB_DMA_ChannelXDestinationStartAddressSet(DMA_ID_0, ${DMA_CHANNEL}, ${DMA_DESTINATION_ADDRESS});

   /* Set the source and destination sizes */
   PLIB_DMA_ChannelXSourceSizeSet(DMA_ID_0, ${DMA_CHANNEL}, ${DMA_SOURCE_SIZE});
   PLIB_DMA_ChannelXDestinationSizeSet(DMA_ID_0, ${DMA_CHANNEL}, ${DMA_DESTINATION_SIZE});

   /* Set the number of bytes per transfer */
   PLIB_DMA_ChannelXCellSizeSet(DMA_ID_0, ${DMA_CHANNEL}, ${DMA_CELL_SIZE});
   
   
}

void SYS_DMA_Channel${INSTANCE}Enable(void)
{
   /* Enable DMA channel ${INSTANCE} */
   PLIB_DMA_ChannelXEnable(DMA_ID_0, ${DMA_CHANNEL});
}

void SYS_DMA_Channel${INSTANCE}Setup(SYS_DMA_CHANNEL_OP_MODE modeEnable)
{

    /* Pattern match Abort */
    if(SYS_DMA_CHANNEL_OP_MODE_MATCH_ABORT & modeEnable)
    {
        PLIB_DMA_ChannelXTriggerEnable( DMA_ID_0,${DMA_CHANNEL},DMA_CHANNEL_TRIGGER_PATTERN_MATCH_ABORT);
    }

    /* Channel chaining mode */
    if(SYS_DMA_CHANNEL_OP_MODE_CHAIN_LOW & modeEnable)
    {
        PLIB_DMA_ChannelXChainEnable( DMA_ID_0, ${DMA_CHANNEL});
    }

    /* Auto Enable mode */
    if(SYS_DMA_CHANNEL_OP_MODE_AUTO & modeEnable)
    {
        PLIB_DMA_ChannelXAutoEnable( DMA_ID_0, ${DMA_CHANNEL});
    }

    /* CRC Enable */
    if(SYS_DMA_CHANNEL_OP_MODE_CRC & modeEnable)
    {
        PLIB_DMA_CRCEnable(DMA_ID_0);
    }
	
<#if M_SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE == true>
	PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_ADDRESS_ERROR);
	PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_ADDRESS_ERROR);
	PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_BLOCK_TRANSFER_COMPLETE);
	PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_BLOCK_TRANSFER_COMPLETE);
</#if>
<#if M_SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE == true>
	PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_ADDRESS_ERROR);
	PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_ADDRESS_ERROR);
	PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_DESTINATION_DONE);
	PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_DESTINATION_HALF_FULL);
	PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_DESTINATION_HALF_FULL);
	PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_DESTINATION_DONE);
</#if>
<#if M_SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE == true>
	PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_ADDRESS_ERROR);
	PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_ADDRESS_ERROR);
	PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_SOURCE_DONE);
	PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_SOURCE_HALF_EMPTY);
	PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_SOURCE_HALF_EMPTY);
	PLIB_DMA_ChannelXINTSourceEnable(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_SOURCE_DONE);
</#if>

    /* DMA transfer to start */
    PLIB_DMA_ChannelXTriggerEnable(DMA_ID_0, ${DMA_CHANNEL}, DMA_CHANNEL_TRIGGER_TRANSFER_START);
    PLIB_DMA_ChannelXStartIRQSet(DMA_ID_0, ${DMA_CHANNEL}, ${DMA_TRIGGER});
}

bool SYS_DMA_Channel${INSTANCE}IsBusy(void)
{
    bool isBusy;

    isBusy = false;
#if defined(PLIB_DMA_ExistsChannelXBusy)
    if(true == PLIB_DMA_ExistsChannelXBusy(DMA_ID_0))
    {
        if(true == PLIB_DMA_ChannelXBusyIsBusy( DMA_ID_0,${DMA_CHANNEL}))
        {
            isBusy = true;
        }
    }
#endif
    return isBusy;
}

void SYS_DMA_Channel${INSTANCE}ForceAbort(void)
{
   PLIB_DMA_AbortTransferSet(DMA_ID_0, ${DMA_CHANNEL});
}

void SYS_DMA_Channel${INSTANCE}ForceStart(void)
{
   PLIB_DMA_StartTransferSet(DMA_ID_0, ${DMA_CHANNEL});
}

void SYS_DMA_Channel${INSTANCE}Disable(void)
{
   PLIB_DMA_ChannelXDisable(DMA_ID_0, ${DMA_CHANNEL});
}

uint32_t SYS_DMA_Channel${INSTANCE}GetCRC(void)
{
    uint32_t crcData;
#if defined (PLIB_DMA_ExistsCRCData)
    if(true == PLIB_DMA_ExistsCRCData(DMA_ID_0))
    {
        crcData = PLIB_DMA_CRCDataRead(DMA_ID_0);
    }
#endif
    return crcData;
}

void SYS_DMA_Channel${INSTANCE}SetCRC(SYS_DMA_CHANNEL_OPERATION_MODE_CRC crc)
{
    /* Enable CRC for the Channel and the type */
#if defined (PLIB_DMA_ExistsCRCChannel)
    if(true == PLIB_DMA_ExistsCRCChannel(DMA_ID_0))
    {
        PLIB_DMA_CRCChannelSelect(DMA_ID_0, ${DMA_CHANNEL});  
    }
#endif

#if defined (PLIB_DMA_ExistsCRCType)
    if(true == PLIB_DMA_ExistsCRCType(DMA_ID_0))
    {
        PLIB_DMA_CRCTypeSet(DMA_ID_0, crc.type);
    }
#endif

    /* Setup the polynomial length */
    PLIB_DMA_CRCPolynomialLengthSet(DMA_ID_0, crc.polyLength);
    
    /* Setup the bit order and byte order */
#if defined (PLIB_DMA_ExistsCRCBitOrder)
    if(true == PLIB_DMA_ExistsCRCBitOrder(DMA_ID_0))
    {
        PLIB_DMA_CRCBitOrderSelect(DMA_ID_0, crc.bitOrder);
    }
#endif    

#if defined (PLIB_DMA_ExistsCRCByteOrder)
    if(true == PLIB_DMA_ExistsCRCByteOrder(DMA_ID_0))
    {
        PLIB_DMA_CRCByteOrderSelect(DMA_ID_0, crc.byteOrder);
    }
#endif

    /* BYTE/WORD order reallignment */
#if defined (PLIB_DMA_ExistsCRCWriteByteOrder)
    if(true == PLIB_DMA_ExistsCRCWriteByteOrder(DMA_ID_0))
    {
        if(SYS_DMA_CRC_WRITE_ORDER_MAINTAIN == crc.writeOrder)
        {
            PLIB_DMA_CRCWriteByteOrderMaintain(DMA_ID_0);
        }
        else if(SYS_DMA_CRC_WRITE_ORDER_CHANGE == crc.writeOrder)
        {
            PLIB_DMA_CRCWriteByteOrderAlter(DMA_ID_0);
        }
        else
        {
        }
    }
#endif

    /* CRC Data */
#if defined (PLIB_DMA_ExistsCRCData)
    if(true == PLIB_DMA_ExistsCRCData(DMA_ID_0))
    {
        PLIB_DMA_CRCDataWrite(DMA_ID_0, crc.data);
    }
#endif


    /* Enable CRC XOR */
#if defined (PLIB_DMA_ExistsCRCXOREnable)
    if(true == PLIB_DMA_ExistsCRCXOREnable(DMA_ID_0))
    {
        PLIB_DMA_CRCXOREnableSet(DMA_ID_0, crc.xorBitMask);
    }
#endif

}

void SYS_DMA_Channel${INSTANCE}SetupMatchAbortMode( uint16_t pattern,
                                         DMA_PATTERN_LENGTH length,
                                         SYS_DMA_CHANNEL_IGNORE_MATCH ignore,
                                         uint8_t ignorePattern )
{

    /* Set the length for the pattern match */
#if defined(PLIB_DMA_ExistsChannelXPatternLength)
    if(true == PLIB_DMA_ExistsChannelXPatternLength(DMA_ID_0))
    {
        PLIB_DMA_ChannelXPatternLengthSet(DMA_ID_0,${DMA_CHANNEL},length);
    }
#endif
    /* Set the pattern for the pattern match */  
    PLIB_DMA_ChannelXPatternDataSet(DMA_ID_0,${DMA_CHANNEL},pattern);

if(SYS_DMA_CHANNEL_IGNORE_MATCH_ENABLE == ignore)
{
    #if defined(PLIB_DMA_ExistsChannelXPatternIgnoreByte)    
        if(true == PLIB_DMA_ExistsChannelXPatternIgnoreByte(DMA_ID_0))
        {
            /* Enable the ignore byte and the pattern */    
            PLIB_DMA_ChannelXPatternIgnoreByteEnable(DMA_ID_0,${DMA_CHANNEL}); 
        }
    #endif  
    #if defined(PLIB_DMA_ExistsChannelXPatternIgnore)    
        if(true == PLIB_DMA_ExistsChannelXPatternIgnore(DMA_ID_0))
        {  
            PLIB_DMA_ChannelXPatternIgnoreSet(DMA_ID_0,${DMA_CHANNEL},ignorePattern);
        }
    #endif
}
}

<#if (M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE == false)>
__attribute__((weak)) void SYS_DMA_Tasks${INSTANCE}(void)
<#else>
void SYS_DMA_Tasks${INSTANCE}(void)
</#if>
{          
	//check address error
    if (PLIB_DMA_ChannelXINTSourceFlagGet(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_ADDRESS_ERROR))
    {
		PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_ADDRESS_ERROR);
<#if M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE == true>
		${M_DRV_DMA_CALL_BACK_FUNCTION}(DMA_INT_ADDRESS_ERROR);
</#if>
    }
	//check transfer abort 
    if (PLIB_DMA_ChannelXINTSourceFlagGet(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_TRANSFER_ABORT))
    {
		PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_TRANSFER_ABORT);
<#if M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE == true>
		${M_DRV_DMA_CALL_BACK_FUNCTION}(DMA_INT_TRANSFER_ABORT);
</#if>
    }
<#if M_SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE == true>
	//check block transfer complete
    if (PLIB_DMA_ChannelXINTSourceFlagGet(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_BLOCK_TRANSFER_COMPLETE))
    {
		PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_BLOCK_TRANSFER_COMPLETE);
<#if M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE == true>
		${M_DRV_DMA_CALL_BACK_FUNCTION}(DMA_INT_BLOCK_TRANSFER_COMPLETE);
</#if>
    }
</#if>



<#if M_SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE == true>
    //check destination buffer full flag
    if (PLIB_DMA_ChannelXINTSourceFlagGet(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_DESTINATION_DONE))
    {   
        //clear both done and half since half flag is also set when it buffer is full
        PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_DESTINATION_HALF_FULL);
        PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_DESTINATION_DONE);
<#if M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE == true>
		${M_DRV_DMA_CALL_BACK_FUNCTION}(DMA_INT_DESTINATION_DONE);
</#if>
    }
    //check destination buffer half full flag
    if (PLIB_DMA_ChannelXINTSourceFlagGet(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_DESTINATION_HALF_FULL))
    {
        PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_DESTINATION_HALF_FULL);
<#if M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE == true>
		${M_DRV_DMA_CALL_BACK_FUNCTION}(DMA_INT_DESTINATION_HALF_FULL);
</#if>
    }
</#if>


<#if M_SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE == true>
    //check source buffer empty flag
    if (PLIB_DMA_ChannelXINTSourceFlagGet(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_SOURCE_DONE))
    {
        //clear both done and half since half flag is also set when it buffer is empty
        PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_SOURCE_HALF_EMPTY);
        PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_SOURCE_DONE);
<#if M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE == true>
		${M_DRV_DMA_CALL_BACK_FUNCTION}(DMA_INT_SOURCE_DONE);
</#if>
    }
    //check source buffer half empty flag
    if (PLIB_DMA_ChannelXINTSourceFlagGet(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_SOURCE_HALF_EMPTY))
    {
        PLIB_DMA_ChannelXINTSourceFlagClear(DMA_ID_0, ${DMA_CHANNEL}, DMA_INT_SOURCE_HALF_EMPTY);
<#if M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE == true>
		${M_DRV_DMA_CALL_BACK_FUNCTION}(DMA_INT_SOURCE_HALF_EMPTY);
</#if>
    }
</#if>
    /* Clear the interrupt flag */
    PLIB_INT_SourceFlagClear(INT_ID_0, ${M_INT_SRC});
}
</#macro>

// *****************************************************************************
// *****************************************************************************
// Header Includes
// *****************************************************************************
// *****************************************************************************
#include "framework/system/dma/sys_dma_static.h"

// *****************************************************************************
// *****************************************************************************
// Section: static functions
// *****************************************************************************
// *****************************************************************************
SYS_MODULE_OBJ SYS_DMA_Initialize(const SYS_MODULE_INIT * const init)
{
   /* Disable the DMA module */
   PLIB_DMA_Disable(DMA_ID_0);

   /* Set the priority levels for the DMA Channels */
<#if CONFIG_SYS_DMA_CHANNEL_0 == true>
   PLIB_DMA_ChannelXPrioritySelect(DMA_ID_0, DMA_CHANNEL_0, 0);
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_1 == true>
   PLIB_DMA_ChannelXPrioritySelect(DMA_ID_0, DMA_CHANNEL_1, 1);
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_2 == true>
   PLIB_DMA_ChannelXPrioritySelect(DMA_ID_0, DMA_CHANNEL_2, 2);
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_3 == true>
   PLIB_DMA_ChannelXPrioritySelect(DMA_ID_0, DMA_CHANNEL_3, 3);
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_4 == true>
   PLIB_DMA_ChannelXPrioritySelect(DMA_ID_0, DMA_CHANNEL_4, 4);
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_5 == true>
   PLIB_DMA_ChannelXPrioritySelect(DMA_ID_0, DMA_CHANNEL_5, 5);
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_6 == true>
   PLIB_DMA_ChannelXPrioritySelect(DMA_ID_0, DMA_CHANNEL_6, 6);
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_7 == true>
   PLIB_DMA_ChannelXPrioritySelect(DMA_ID_0, DMA_CHANNEL_7, 7);
</#if>
   /* Enable the DMA module */
   PLIB_DMA_Enable(DMA_ID_0);
   
   return (SYS_MODULE_OBJ)0;
}

<#if CONFIG_SYS_DMA_CHANNEL_0 == true>
<@DRV_DMA_STATIC_FUNCTIONS INSTANCE=0 DMA_CHANNEL=CONFIG_SYS_DMA_CHANNEL_ID_IDX0 
DMA_CELL_SIZE=CONFIG_DRV_DMA_CELL_SIZE0 DMA_TRIGGER=CONFIG_DRV_DMA_TRIGGER0 DMA_SOURCE_SIZE=CONFIG_DRV_DMA_SOURCE_SIZE0
 DMA_DESTINATION_SIZE=CONFIG_DRV_DMA_DESTINATION_SIZE0 DMA_TRIGGER=CONFIG_DRV_DMA_TRIGGER0 
DMA_SOURCE_ADDRESS=CONFIG_DRV_DMA_SOURCE_ADDRESS0 
DMA_DESTINATION_ADDRESS=CONFIG_DRV_DMA_DESTINATION_ADDRESS0
M_INT_SRC=CONFIG_SYS_DMA_ISR_SOURCE_CH0
M_SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE=CONFIG_SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE0
M_SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE=CONFIG_SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE0
M_SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE=CONFIG_SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE0
M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE=CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE0
M_DRV_DMA_CALL_BACK_FUNCTION = CONFIG_DRV_DMA_CALL_BACK_FUNCTION0
/>
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_1 == true>
<@DRV_DMA_STATIC_FUNCTIONS INSTANCE=1 DMA_CHANNEL=CONFIG_SYS_DMA_CHANNEL_ID_IDX1 
DMA_CELL_SIZE=CONFIG_DRV_DMA_CELL_SIZE1 DMA_TRIGGER=CONFIG_DRV_DMA_TRIGGER1 DMA_SOURCE_SIZE=CONFIG_DRV_DMA_SOURCE_SIZE1
 DMA_DESTINATION_SIZE=CONFIG_DRV_DMA_DESTINATION_SIZE1 DMA_TRIGGER=CONFIG_DRV_DMA_TRIGGER1 
DMA_SOURCE_ADDRESS=CONFIG_DRV_DMA_SOURCE_ADDRESS1 
DMA_DESTINATION_ADDRESS=CONFIG_DRV_DMA_DESTINATION_ADDRESS1
M_INT_SRC=CONFIG_SYS_DMA_ISR_SOURCE_CH1
M_SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE=CONFIG_SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE1
M_SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE=CONFIG_SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE1
M_SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE=CONFIG_SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE1
M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE=CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE1
M_DRV_DMA_CALL_BACK_FUNCTION = CONFIG_DRV_DMA_CALL_BACK_FUNCTION1
/>
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_2 == true>
<@DRV_DMA_STATIC_FUNCTIONS INSTANCE=2 DMA_CHANNEL=CONFIG_SYS_DMA_CHANNEL_ID_IDX2 
DMA_CELL_SIZE=CONFIG_DRV_DMA_CELL_SIZE2 DMA_TRIGGER=CONFIG_DRV_DMA_TRIGGER2 DMA_SOURCE_SIZE=CONFIG_DRV_DMA_SOURCE_SIZE2
 DMA_DESTINATION_SIZE=CONFIG_DRV_DMA_DESTINATION_SIZE2 DMA_TRIGGER=CONFIG_DRV_DMA_TRIGGER2 
DMA_SOURCE_ADDRESS=CONFIG_DRV_DMA_SOURCE_ADDRESS2 
DMA_DESTINATION_ADDRESS=CONFIG_DRV_DMA_DESTINATION_ADDRESS2
M_INT_SRC=CONFIG_SYS_DMA_ISR_SOURCE_CH2
M_SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE=CONFIG_SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE2
M_SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE=CONFIG_SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE2
M_SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE=CONFIG_SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE2
M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE=CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE2
M_DRV_DMA_CALL_BACK_FUNCTION = CONFIG_DRV_DMA_CALL_BACK_FUNCTION2
/>
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_3 == true>
<@DRV_DMA_STATIC_FUNCTIONS INSTANCE=3 DMA_CHANNEL=CONFIG_SYS_DMA_CHANNEL_ID_IDX3
DMA_CELL_SIZE=CONFIG_DRV_DMA_CELL_SIZE3 DMA_TRIGGER=CONFIG_DRV_DMA_TRIGGER3 DMA_SOURCE_SIZE=CONFIG_DRV_DMA_SOURCE_SIZE3
 DMA_DESTINATION_SIZE=CONFIG_DRV_DMA_DESTINATION_SIZE3 DMA_TRIGGER=CONFIG_DRV_DMA_TRIGGER3 
DMA_SOURCE_ADDRESS=CONFIG_DRV_DMA_SOURCE_ADDRESS3 
DMA_DESTINATION_ADDRESS=CONFIG_DRV_DMA_DESTINATION_ADDRESS3
M_INT_SRC=CONFIG_SYS_DMA_ISR_SOURCE_CH3
M_SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE=CONFIG_SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE3
M_SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE=CONFIG_SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE3
M_SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE=CONFIG_SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE3
M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE=CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE3
M_DRV_DMA_CALL_BACK_FUNCTION = CONFIG_DRV_DMA_CALL_BACK_FUNCTION3
/>
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_4 == true>
<@DRV_DMA_STATIC_FUNCTIONS INSTANCE=4 DMA_CHANNEL=CONFIG_SYS_DMA_CHANNEL_ID_IDX4 
DMA_CELL_SIZE=CONFIG_DRV_DMA_CELL_SIZE4 DMA_TRIGGER=CONFIG_DRV_DMA_TRIGGER4 DMA_SOURCE_SIZE=CONFIG_DRV_DMA_SOURCE_SIZE4
 DMA_DESTINATION_SIZE=CONFIG_DRV_DMA_DESTINATION_SIZE4 DMA_TRIGGER=CONFIG_DRV_DMA_TRIGGER4 
DMA_SOURCE_ADDRESS=CONFIG_DRV_DMA_SOURCE_ADDRESS4 
DMA_DESTINATION_ADDRESS=CONFIG_DRV_DMA_DESTINATION_ADDRESS4
M_INT_SRC=CONFIG_SYS_DMA_ISR_SOURCE_CH4
M_SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE=CONFIG_SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE4
M_SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE=CONFIG_SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE4
M_SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE=CONFIG_SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE4
M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE=CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE4
M_DRV_DMA_CALL_BACK_FUNCTION = CONFIG_DRV_DMA_CALL_BACK_FUNCTION4
/>
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_5 == true>
<@DRV_DMA_STATIC_FUNCTIONS INSTANCE=5 DMA_CHANNEL=CONFIG_SYS_DMA_CHANNEL_ID_IDX5 
DMA_CELL_SIZE=CONFIG_DRV_DMA_CELL_SIZE5 DMA_TRIGGER=CONFIG_DRV_DMA_TRIGGER5 DMA_SOURCE_SIZE=CONFIG_DRV_DMA_SOURCE_SIZE5
 DMA_DESTINATION_SIZE=CONFIG_DRV_DMA_DESTINATION_SIZE5 DMA_TRIGGER=CONFIG_DRV_DMA_TRIGGER5 
DMA_SOURCE_ADDRESS=CONFIG_DRV_DMA_SOURCE_ADDRESS5 
DMA_DESTINATION_ADDRESS=CONFIG_DRV_DMA_DESTINATION_ADDRESS5 
M_INT_SRC=CONFIG_SYS_DMA_ISR_SOURCE_CH5
M_SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE=CONFIG_SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE5
M_SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE=CONFIG_SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE5
M_SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE=CONFIG_SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE5
M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE=CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE5
M_DRV_DMA_CALL_BACK_FUNCTION = CONFIG_DRV_DMA_CALL_BACK_FUNCTION5
/>
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_6 == true>
<@DRV_DMA_STATIC_FUNCTIONS INSTANCE=6 DMA_CHANNEL=CONFIG_SYS_DMA_CHANNEL_ID_IDX6 
DMA_CELL_SIZE=CONFIG_DRV_DMA_CELL_SIZE6 DMA_TRIGGER=CONFIG_DRV_DMA_TRIGGER6 DMA_SOURCE_SIZE=CONFIG_DRV_DMA_SOURCE_SIZE6
 DMA_DESTINATION_SIZE=CONFIG_DRV_DMA_DESTINATION_SIZE6 DMA_TRIGGER=CONFIG_DRV_DMA_TRIGGER6 
DMA_SOURCE_ADDRESS=CONFIG_DRV_DMA_SOURCE_ADDRESS6 
DMA_DESTINATION_ADDRESS=CONFIG_DRV_DMA_DESTINATION_ADDRESS6 
M_INT_SRC=CONFIG_SYS_DMA_ISR_SOURCE_CH6
M_SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE=CONFIG_SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE6
M_SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE=CONFIG_SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE6
M_SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE=CONFIG_SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE6
M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE=CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE6
M_DRV_DMA_CALL_BACK_FUNCTION = CONFIG_DRV_DMA_CALL_BACK_FUNCTION6
/>
</#if>
<#if CONFIG_SYS_DMA_CHANNEL_7 == true>
<@DRV_DMA_STATIC_FUNCTIONS INSTANCE=7 DMA_CHANNEL=CONFIG_SYS_DMA_CHANNEL_ID_IDX7 
DMA_CELL_SIZE=CONFIG_DRV_DMA_CELL_SIZE7 DMA_TRIGGER=CONFIG_DRV_DMA_TRIGGER7 DMA_SOURCE_SIZE=CONFIG_DRV_DMA_SOURCE_SIZE7
 DMA_DESTINATION_SIZE=CONFIG_DRV_DMA_DESTINATION_SIZE7 DMA_TRIGGER=CONFIG_DRV_DMA_TRIGGER7 
DMA_SOURCE_ADDRESS=CONFIG_DRV_DMA_SOURCE_ADDRESS7 
DMA_DESTINATION_ADDRESS=CONFIG_DRV_DMA_DESTINATION_ADDRESS7 
M_INT_SRC=CONFIG_SYS_DMA_ISR_SOURCE_CH7
M_SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE=CONFIG_SYS_DMA_BLOCK_TRANSFER_INTERRUPT_MODE7
M_SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE=CONFIG_SYS_DMA_HALF_AND_FULL_DESTINATION_INTERRUPT_MODE7
M_SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE=CONFIG_SYS_DMA_HALF_AND_FULL_SOURCE_INTERRUPT_MODE7
M_SYS_DMA_CHANNEL_CALL_BACK_ENABLE=CONFIG_SYS_DMA_CHANNEL_CALL_BACK_ENABLE7
M_DRV_DMA_CALL_BACK_FUNCTION = CONFIG_DRV_DMA_CALL_BACK_FUNCTION7
/>
</#if>

 
/*******************************************************************************
 End of File
*/
