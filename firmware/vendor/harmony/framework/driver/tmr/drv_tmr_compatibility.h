/*******************************************************************************
  Timer Device Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_tmr_deprecated.h

  Summary:
    Timer device driver interface header file.

  Description:
    This header file contains the function prototypes and definitions of the
    data types and constants that make up the interface to the Timer device
    driver.
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

#ifndef _DRV_TMR_DEPRECATED_H
#define _DRV_TMR_DEPRECATED_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/* Function:
    void DRV_TMR_Tasks_ISR ( SYS_MODULE_OBJ object )

  Summary:
    Maintains the driver's state machine, processes the events and implements 
	its ISR.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function is used to maintain the driver's internal state machine and
    processes the timer events in interrupt-driven implementations
    (DRV_TMR_INTERRUPT_MODE == true).

  Precondition:
    The DRV_TMR_Initialize function must have been called for the specified Timer
    driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_TMR_Initialize)

  Returns:
    None

  Example:
    <code>
        void __ISR(_TIMER_2_VECTOR, ipl4) _InterruptHandler_TMR2(void)
        {
            DRV_TMR_Tasks_ISR(appDrvObjects.drvTmrObject);
        }
    </code>

  Remarks:
    This function is normally not called directly by an application.
    It is called by the timer driver raw ISR.

    This function will execute in an ISR context and will never block or access any
    resources that may cause it to block.
*/

 inline void DRV_TMR_Tasks_ISR( SYS_MODULE_OBJ object)     
{
	DRV_TMR_Tasks(object);
}

// *****************************************************************************
/* Function:
    void DRV_TMR_CounterValue16BitSet 
    (
        DRV_HANDLE handle, 
        uint16_t counterPeriod 
    )

  Summary:
    Updates the 16-bit Timer's counter register.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function updates the 16-bit Timer's value in the counter register. This
    is valid only if the 16-bit mode of the timer is selected Otherwise use
    DRV_TMR_CounterValue32BitSet function.

  Remarks:
    Refer to drv_tmr.h for usage information.
*/

 inline void DRV_TMR_CounterValue16BitSet 
    (
        DRV_HANDLE handle, 
        uint16_t counterPeriod 
    )
{
	DRV_TMR_CounterValueSet(handle,counterPeriod);
} 

// *****************************************************************************
/* Function:
    void DRV_TMR_CounterValue32BitSet 
    (
        DRV_HANDLE handle, 
        uint32_t counterPeriod 
    )

  Summary:
    Updates the 32-bit Timer's counter register.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function updates the 32-bit Timer's value in the counter register. This
    is valid only if the 32-bit mode of the timer is selected Otherwise use
    DRV_TMR_CounterValue16BitSet function.

  Remarks:
    Refer to drv_tmr.h for usage information.
*/

 inline void DRV_TMR_CounterValue32BitSet 
    (
        DRV_HANDLE handle, 
        uint32_t counterPeriod 
    )
{
	DRV_TMR_CounterValueSet(handle,counterPeriod);
}     

// *****************************************************************************
/* Function:
    uint16_t DRV_TMR_CounterValue16BitGet ( DRV_HANDLE handle )

  Summary:
    Reads the 16-bit Timer's counter register.o

  Description:
    This function returns the 16-bit Timer's value in the counter register. This 
	is valid only if the 16-bit mode of the timer is selected.
	Otherwise use DRV_TMR_CounterValue32BitGet function.

  Remarks:
    Refer to drv_tmr.h for usage information.
*/

 inline uint16_t DRV_TMR_CounterValue16BitGet ( DRV_HANDLE handle )
{
	return ((uint16_t)DRV_TMR_CounterValueGet(handle));
}         

// *****************************************************************************
/* Function:
    uint32_t DRV_TMR_CounterValue32BitGet ( DRV_HANDLE handle )

  Summary:
    Reads the 32-bit Timer's counter register.

  Description:
    This function returns the 32-bit Timer's value in the counter register. This
    is valid only if the 32-bit mode of the timer is selected Otherwise use
    DRV_TMR_CounterValue16BitGet function.

  Remarks:
    Refer to drv_tmr.h for usage information.
*/

 inline uint32_t DRV_TMR_CounterValue32BitGet ( DRV_HANDLE handle )
{
	return DRV_TMR_CounterValueGet(handle);
}      

// *****************************************************************************
/* Function:
    void DRV_TMR_Alarm16BitRegister 
    ( 
        DRV_HANDLE handle, 
        uint16_t period, 
        bool isPeriodic, 
        uintptr_t context, 
        DRV_TMR_CALLBACK callBack 
    )

  Summary:
    Sets up an alarm.

  Description:
    This function sets up an alarm, allowing the client to receive a callback
    from the driver when the counter period elapses.  Alarms can be one-shot or
    periodic.  This API is valid only if the 16-bit mode of the timer is
    selected.  Otherwise use DRV_TMR_Alarm32BitRegister function. 

  Remarks:
    Refer to drv_tmr.h for usage information.
*/

inline  void DRV_TMR_Alarm16BitRegister 
( 
        DRV_HANDLE handle, 
        uint16_t period, 
        bool isPeriodic, 
        uintptr_t context, 
        DRV_TMR_CALLBACK callBack 
)
{
	DRV_TMR_AlarmRegister(handle,period,isPeriodic,context,callBack);
}	

// *****************************************************************************
/* Function:
    void DRV_TMR_Alarm32BitRegister 
    (
        DRV_HANDLE handle,
        uint32_t period, 
        bool isPeriodic, 
        uintptr_t context,	
        DRV_TMR_CALLBACK callBack 
    )

  Summary:
    Sets up an alarm.

  Description:
    This function sets up an alarm, allowing the client to receive a callback
    from the driver when the counter period elapses.  Alarms can be one-shot or
    periodic.  This API is valid only if the 32-bit mode of the timer is
    selected Otherwise use DRV_TMR_Alarm16BitRegister function. 

  Remarks:
    Refer to drv_tmr.h for usage information.
*/

inline void DRV_TMR_Alarm32BitRegister 
    (
        DRV_HANDLE handle,
        uint32_t period, 
        bool isPeriodic, 
        uintptr_t context,	
        DRV_TMR_CALLBACK callBack 
    )   
{
	DRV_TMR_AlarmRegister(handle,period,isPeriodic,context,callBack);
}    

// *****************************************************************************
/* Function:
    void DRV_TMR_AlarmPeriod16BitSet ( DRV_HANDLE handle, uint16_t value )

  Summary:
    Updates the 16-bit Timer's period.

  Description:
    This function updates the 16-bit Timer's period. This API is valid only if 
	the 16-bit mode of the timer is selected
	Otherwise use DRV_TMR_AlarmPeriod32BitSet function. 

  Remarks:
    Refer to drv_tmr.h for usage information.
*/

inline void DRV_TMR_AlarmPeriod16BitSet ( DRV_HANDLE handle, uint16_t value )      
{
	DRV_TMR_AlarmPeriodSet(handle,value);
}

// *****************************************************************************
/* Function:
    void DRV_TMR_AlarmPeriod32BitSet ( DRV_HANDLE handle, uint32_t period )

  Summary:
    Updates the 32-bit Timer's period.

  Description:
    This function updates the 32-bit Timer's period. This API is valid only if 
	the 32-bit mode of the timer is selected
	Otherwise use DRV_TMR_AlarmPeriod16BitSet function. 

  Remarks:
    Refer to drv_tmr.h for usage information.
*/

 inline void DRV_TMR_AlarmPeriod32BitSet ( DRV_HANDLE handle, uint32_t period )
{
	DRV_TMR_AlarmPeriodSet(handle,period);
}      

// *****************************************************************************
/* Function:
    uint16_t DRV_TMR_AlarmPeriod16BitGet ( DRV_HANDLE handle )

  Summary:
    Provides the 16-bit Timer's period.

  Description:
    This function gets the 16-bit Timer's period. This API is valid only if 
	the 16-bit mode of the timer is selected.
	Otherwise use DRV_TMR_AlarmPeriod32BitGet function.

  Remarks:
    Refer to drv_tmr.h for usage information.
*/

 inline uint16_t DRV_TMR_AlarmPeriod16BitGet ( DRV_HANDLE handle )
{
	return ((uint16_t)DRV_TMR_AlarmPeriodGet(handle));
}     

// *****************************************************************************
/* Function:
    uint32_t DRV_TMR_AlarmPeriod32BitGet ( DRV_HANDLE handle )

  Summary:
    Provides the 32-bit Timer's period.

  Description:
    This function gets the 32-bit Timer's period. This API is valid only if 
	the 32-bit mode of the timer is selected
	Otherwise use DRV_TMR_AlarmPeriod16BitGet function. 

  Remarks:
    Refer to drv_tmr.h for usage information.
*/

 inline uint32_t DRV_TMR_AlarmPeriod32BitGet ( DRV_HANDLE handle )
{
	return DRV_TMR_AlarmPeriodGet(handle);
}      

// *****************************************************************************
/* Function:
    void DRV_TMR_Alarm16BitDeregister ( DRV_HANDLE handle )

  Summary:
    Removes a previously set alarm.

  Description:
    This function removes a previously set alarm. This API is valid only if 
	the 16-bit mode of the timer is selected
	Otherwise use DRV_TMR_Alarm32BitDeregister function.

  Remarks:
    Refer to drv_tmr.h for usage information.
*/

 inline void DRV_TMR_Alarm16BitDeregister ( DRV_HANDLE handle )
{
	DRV_TMR_AlarmDeregister(handle);
}    

// *****************************************************************************
/* Function:
    void DRV_TMR_Alarm32BitDeregister ( DRV_HANDLE handle )

  Summary:
    Removes a previously set alarm.

  Description:
    This function removes a previously set alarm. This API is valid only if 
	the 32-bit mode of the timer is selected
	Otherwise use DRV_TMR_Alarm16BitDeregister function.

  Remarks:
    Refer to drv_tmr.h for usage information.
*/

 inline void DRV_TMR_Alarm32BitDeregister ( DRV_HANDLE handle )
{
	DRV_TMR_AlarmDeregister(handle);
}   

#endif // #ifndef _DRV_TMR_DEPRECATED_H

/*******************************************************************************
 End of File
*/

