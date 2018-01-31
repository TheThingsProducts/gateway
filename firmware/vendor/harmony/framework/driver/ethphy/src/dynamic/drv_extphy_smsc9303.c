/*******************************************************************************
  SMSC LAN9303 PHY API for Microchip TCP/IP Stack
*******************************************************************************/

/*******************************************************************************
File Name:  drv_extphy_smsc9303.c
Copyright © 2014 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND,
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
#include "driver/ethphy/src/drv_ethphy_local.h"

#include "drv_extphy_smsc9303.h"

/****************************************************************************
 *                 Driver Overrides                                         *
 ****************************************************************************/

DRV_ETHPHY_RESULT DRV_ETHPHY_Smsc9303LinkStatusGet( DRV_HANDLE handle, DRV_ETHPHY_LINK_STATUS* pLinkStat, bool refresh );

const DRV_ETHPHY_OBJECT_BASE  DRV_ETHPHY_OBJECT_BASE_smsc9303 = 
{
    DRV_ETHPHY_Initialize,
    DRV_ETHPHY_Reinitialize,
    DRV_ETHPHY_Deinitialize,
    DRV_ETHPHY_Status,
    DRV_ETHPHY_Tasks,
    DRV_ETHPHY_Open,
    DRV_ETHPHY_Close,
    DRV_ETHPHY_ClientStatus,
    DRV_ETHPHY_ClientOperationResult,
    DRV_ETHPHY_ClientOperationAbort,
    DRV_ETHPHY_SMIRead,
    DRV_ETHPHY_SMIWrite,
    DRV_ETHPHY_SMIScanStart,
    DRV_ETHPHY_SMIScanStop,
    DRV_ETHPHY_SMIScanStatusGet,
    DRV_ETHPHY_SMIScanDataGet,
    DRV_ETHPHY_SMIStatus,
    DRV_ETHPHY_SMIClockSet,
    DRV_ETHPHY_PhyAddressGet,
    DRV_ETHPHY_Setup,
    DRV_ETHPHY_RestartNegotiation,
    DRV_ETHPHY_HWConfigFlagsGet,
    DRV_ETHPHY_NegotiationIsComplete,
    DRV_ETHPHY_NegotiationResultGet,
    DRV_ETHPHY_Smsc9303LinkStatusGet,
    DRV_ETHPHY_Reset,
    DRV_ETHPHY_VendorDataGet,
    DRV_ETHPHY_VendorDataSet,
    DRV_ETHPHY_VendorSMIReadStart,
    DRV_ETHPHY_VendorSMIReadResultGet,
    DRV_ETHPHY_VendorSMIWriteStart,
};


/****************************************************************************
 *                 interface functions
 ****************************************************************************/

uint32_t DRV_ETHPHY_SMC9303_SMIExtRead(DRV_HANDLE handle, uint16_t rIx);
uint16_t DRV_ETHPHY_SMC9303_SMIRead(DRV_HANDLE handle, uint8_t addr, uint16_t rIx);
void DRV_ETHPHY_SMC9303_SMIExtWrite(DRV_HANDLE handle, uint16_t rIx, uint32_t val);

static DRV_ETHPHY_RESULT DRV_ETHPHY_SMC9303_VendorSMIExtWrite(const DRV_ETHPHY_OBJECT_BASE* pBaseObj, DRV_HANDLE hClientObj, uint16_t rIx, uint32_t val);

/****************************************************************************
 * Function:        DRV_EXTPHY_MIIConfigure
 *
 * PreCondition:    - Communication to the PHY should have been established.
 *
 * Input:   		handle - A valid open-instance handle, returned from the driver's open routine   
 *					cFlags - the requested configuration flags: DRV_ETHPHY_CFG_RMII/DRV_ETHPHY_CFG_MII
 *
 * Output:          DRV_ETHPHY_RES_OK - success,
 *                  an error code otherwise
 *
 *
 * Side Effects:    None
 *
 * Overview:        This function configures the PHY in one of MII/RMII operation modes.
 *
 *****************************************************************************/


#if SMSC_9303_CMD_PROCESSOR
extern DRV_HANDLE sHClientObj;
#endif

static DRV_ETHPHY_RESULT DRV_EXTPHY_MIIConfigure(const DRV_ETHPHY_OBJECT_BASE* pBaseObj, DRV_HANDLE hClientObj,DRV_ETHPHY_CONFIG_FLAGS cFlags)
{
#if SMSC_9303_CMD_PROCESSOR
    sHClientObj = hClientObj;
#endif
        __SMSC9303_LED_CFG_t led;
        led.d = 0;
        led.LED_EN0 = 1;
        led.LED_EN1 = 1;
        led.LED_EN2 = 0;
        led.LED_EN3 = 1;
        led.LED_EN4 = 1;
        led.LED_EN5 = 0;

        led.LED_FUN = 2;

        //DRV_ETHPHY_SMC9303_SMIExtWrite(hClientObj, PHY_REG_LED_CFG, led.d);

        return DRV_ETHPHY_SMC9303_VendorSMIExtWrite(pBaseObj, hClientObj, PHY_REG_LED_CFG, led.d);
}


/****************************************************************************
 * Function:        DRV_EXTPHY_MDIXConfigure
 *
 * PreCondition:    - Communication to the PHY should have been established.
 *
 * Input:           handle - A valid open-instance handle, returned from the driver's open routine
 *					oFlags - the requested open flags: TCPIP_ETH_OPEN_MDIX_AUTO, TCPIP_ETH_OPEN_MDIX_NORM/TCPIP_ETH_OPEN_MDIX_SWAP
 *
 * Output:          DRV_ETHPHY_RES_OK - success,
 *                  an error code otherwise
 *
 *
 * Side Effects:    None
 *
 * Overview:        This function configures the MDIX mode for the PHY.
 *
 * Note:            None
 *****************************************************************************/
static DRV_ETHPHY_RESULT DRV_EXTPHY_MDIXConfigure(const DRV_ETHPHY_OBJECT_BASE* pBaseObj, DRV_HANDLE hClientObj, TCPIP_ETH_OPEN_FLAGS oFlags)
{
	return DRV_ETHPHY_RES_OK;	

}

/****************************************************************************
 * Function:        EthPhyMIIMClock
 *
 * PreCondition:    None
 *
 * Input:           handle - A valid open-instance handle, returned from the driver's open routine
 *
 * Output:          PHY MIIM clock, Hz
 *
 *
 * Side Effects:    None
 *
 * Overview:        This function returns the maximum clock frequency that the PHY can use for the MIIM transactions
 *
 * Note:            None
 *****************************************************************************/
static unsigned int DRV_EXTPHY_SMIClockGet(const DRV_ETHPHY_OBJECT_BASE* pBaseObj, DRV_HANDLE handle)
{
	return 2500000;		//  2.5 MHz max clock supported
}

// the DRV_ETHPHY_OBJECT
const DRV_ETHPHY_OBJECT  DRV_ETHPHY_OBJECT_SMSC_LAN9303 = 
{
    DRV_EXTPHY_MIIConfigure,
    DRV_EXTPHY_MDIXConfigure,
    DRV_EXTPHY_SMIClockGet,
    0,                          // no WOL functionality yet
};






#define DRV_ETHPHY_SMC32_PhyAddr(reg) (((reg >> 6) & 0xf) | 0x10)
#define DRV_ETHPHY_SMC32_RegAddr(reg) ((reg >> 1) & 0x1f)

static DRV_ETHPHY_RESULT DRV_ETHPHY_SMC9303_VendorSMIExtWrite(const DRV_ETHPHY_OBJECT_BASE* pBaseObj, DRV_HANDLE hClientObj, uint16_t rIx, uint32_t val)
{
    uint32_t    writeExPhase = 0;

    DRV_ETHPHY_RESULT res = pBaseObj->DRV_ETHPHY_VendorDataGet(hClientObj, &writeExPhase);

    if(res < 0)
    {   // some error occurred
        return res;
    }

    switch(writeExPhase)
    {
        case 0:
            // 1st part of the extended write
            res = pBaseObj->DRV_ETHPHY_VendorSMIWriteStart(hClientObj, DRV_ETHPHY_SMC32_RegAddr(rIx), val & 0xffff, DRV_ETHPHY_SMC32_PhyAddr(rIx));
            if(res < 0)
            {   // some error
                return res;
            }
            else if(res == DRV_ETHPHY_RES_PENDING)
            {   // stay in this state & retry
                return DRV_ETHPHY_RES_PENDING;
            }

            // advance state
            pBaseObj->DRV_ETHPHY_VendorDataSet(hClientObj, ++writeExPhase);
            return DRV_ETHPHY_RES_PENDING;

        case 1:
            // 2nd part of the extended write
            res = pBaseObj->DRV_ETHPHY_VendorSMIWriteStart(hClientObj, DRV_ETHPHY_SMC32_RegAddr(rIx) + 1, (val >> 16) & 0xffff, DRV_ETHPHY_SMC32_PhyAddr(rIx));
            if(res < 0)
            {   // some error
                return res;
            }
            else if(res == DRV_ETHPHY_RES_PENDING)
            {   // stay in this state & retry
                return DRV_ETHPHY_RES_PENDING;
            }

            // done
            return DRV_ETHPHY_RES_OK;

        default:
            // shouldn't happen
            return DRV_ETHPHY_RES_OPERATION_ERR;

    }

}

uint16_t DRV_ETHPHY_SMIReadResultGet(ETH_MODULE_ID ethphyId)
{
    while ( PLIB_ETH_MIIMIsBusy(ethphyId) )
    {// wait op complete
        //Do nothing.
    }

    PLIB_ETH_MIIMWriteStart(ethphyId);         // Stop read cycle.
    return PLIB_ETH_MIIMReadDataGet(ethphyId); // The read register
}

uint32_t DRV_ETHPHY_SMC9303_SMIExtRead(DRV_HANDLE handle, uint16_t rIx)
{
    uint32_t ret = 0;

    DRV_ETHPHY_CLIENT_OBJ * hClientObj = (DRV_ETHPHY_CLIENT_OBJ *) handle;
    ETH_MODULE_ID    ethphyId;  // Assume instance ID for PHY same as Ethernet MAC PLIB

    if (hClientObj == NULL)
    {
        SYS_ERROR(SYS_ERROR_ERROR, "SMC9303 read: Bad client handle!");
        return 0;
    }

    ethphyId = hClientObj->ethphyId;
    while ( PLIB_ETH_MIIMIsBusy(ethphyId) )
    {// wait in case of some previous operation
        //Do nothing.
    }

    PLIB_ETH_PHYAddressSet(ethphyId, DRV_ETHPHY_SMC32_PhyAddr(rIx));
    PLIB_ETH_RegisterAddressSet(ethphyId,DRV_ETHPHY_SMC32_RegAddr(rIx));
    PLIB_ETH_MIIMReadStart(ethphyId);  // Start read

    ret = DRV_ETHPHY_SMIReadResultGet(ethphyId) & 0xffff;

    PLIB_ETH_PHYAddressSet(ethphyId, DRV_ETHPHY_SMC32_PhyAddr(rIx));

    // Refer to section 10.2 of the 9303 data sheet Page 126.
    // Register Address field bit 0 is used as the upper/lower word select.

    PLIB_ETH_RegisterAddressSet(ethphyId,DRV_ETHPHY_SMC32_RegAddr(rIx) + 1);
    PLIB_ETH_MIIMReadStart(ethphyId);  // Start read

    ret |= DRV_ETHPHY_SMIReadResultGet(ethphyId) << 16;

    return ret;
}

uint16_t DRV_ETHPHY_SMC9303_SMIRead(DRV_HANDLE handle, uint8_t addr, uint16_t rIx)
{
    uint16_t ret = 0;

    DRV_ETHPHY_CLIENT_OBJ * hClientObj = (DRV_ETHPHY_CLIENT_OBJ *) handle;
    ETH_MODULE_ID    ethphyId;  // Assume instance ID for PHY same as Ethernet MAC PLIB

    if (hClientObj == NULL)
    {
        SYS_ERROR(SYS_ERROR_ERROR, "SMC9303 read: Bad client handle!");
        return 0;
    }

    ethphyId = hClientObj->ethphyId;
    while ( PLIB_ETH_MIIMIsBusy(ethphyId) )
    {// wait in case of some previous operation
        //Do nothing.
    }

    PLIB_ETH_PHYAddressSet(ethphyId, addr);
    PLIB_ETH_RegisterAddressSet(ethphyId, rIx);
    PLIB_ETH_MIIMReadStart(ethphyId);  // Start read

    ret = DRV_ETHPHY_SMIReadResultGet(ethphyId) & 0xff;

    return ret;
}


void DRV_ETHPHY_SMC9303_SMIExtWrite(DRV_HANDLE handle, uint16_t rIx, uint32_t val)
{
    DRV_ETHPHY_CLIENT_OBJ * hClientObj = (DRV_ETHPHY_CLIENT_OBJ *) handle;
    ETH_MODULE_ID    ethphyId;  // Assume instance ID for PHY same as Ethernet MAC PLIB


    if (hClientObj == NULL)
    {
        SYS_ERROR(SYS_ERROR_ERROR, "SMC9303 write: Bad client handle!");
        return;
    }
    
    ethphyId = hClientObj->ethphyId;
    while ( PLIB_ETH_MIIMIsBusy(ethphyId) )
    {// wait in case of some previous operation
        //Do nothing.
    }

    PLIB_ETH_PHYAddressSet(ethphyId, DRV_ETHPHY_SMC32_PhyAddr(rIx));
    PLIB_ETH_RegisterAddressSet(ethphyId,DRV_ETHPHY_SMC32_RegAddr(rIx));
    PLIB_ETH_MIIMWriteDataSet(ethphyId,val & 0xffff);

    while ( PLIB_ETH_MIIMIsBusy(ethphyId) )
    {// wait in case of some previous operation
        //Do nothing.
    }

    PLIB_ETH_PHYAddressSet(ethphyId, DRV_ETHPHY_SMC32_PhyAddr(rIx));
    // Refer to section 10.2 of the 9303 data sheet Page 126.
    // Register Address field bit 0 is used as the upper/lower word select.
    PLIB_ETH_RegisterAddressSet(ethphyId,DRV_ETHPHY_SMC32_RegAddr(rIx)+1);
    PLIB_ETH_MIIMWriteDataSet(ethphyId, (val >> 16) & 0xffff);

} /* DRV_ETHPHY_SMIWriteStart */

int32_t DRV_ETHPHY_SMSC9303_ReadEEPROM(DRV_HANDLE handle, uint8_t * buffer, uint16_t addr, uint16_t len)
{
    __SMSC9303_E2P_CMD_t cmd, cmdr;
    cmd.d = 0;
    cmdr.d = 0;
    cmdr.EPC_BUSY = 1;

    cmd.EPC_BUSY = 1;
    cmd.EPC_COMMAND = 0;
    cmd.EPC_TIMEOUT = 1;

    if ((uint32_t)addr + (uint32_t)len > 0x0000ffffl)
    {
        return -1; // Overflow
    }

    int x = 0;
    for (x = 0; x < len; x++)
    {
        cmd.EPC_ADDRESS = addr + x;
        DRV_ETHPHY_SMC9303_SMIExtWrite(handle, PHY_REG_E2P_CMD, cmd.d);

        while (cmdr.EPC_BUSY == 1)
        {
            cmdr.d = DRV_ETHPHY_SMC9303_SMIExtRead(handle, PHY_REG_E2P_CMD);
        }

        __SMSC9303_E2P_DATA_t data;
        data.d = DRV_ETHPHY_SMC9303_SMIExtRead(handle, PHY_REG_E2P_DATA);
        buffer[x] = data.EEPROM_DATA;
    }
    return len;
}
int32_t DRV_ETHPHY_SMSC9303_WriteEEPROM(DRV_HANDLE handle, uint8_t * buffer, uint16_t addr, uint16_t len)
{
    __SMSC9303_E2P_CMD_t cmd, cmdr;
    cmd.d = 0;
    cmdr.d = 0;

    cmdr.EPC_BUSY = 1;
    cmd.EPC_BUSY = 1;
    cmd.EPC_COMMAND = 3;
    cmd.EPC_TIMEOUT = 1;

    if ((uint32_t)addr + (uint32_t)len > 0x0000ffffl)
    {
        return -1; // Overflow
    }

    int x = 0;
    for (x = 0; x < len; x++)
    {
        cmd.EPC_ADDRESS = addr + x;
        DRV_ETHPHY_SMC9303_SMIExtWrite(handle, PHY_REG_E2P_CMD, cmd.d);
        __SMSC9303_E2P_DATA_t data = {};
        data.EEPROM_DATA = buffer[x];
        DRV_ETHPHY_SMC9303_SMIExtWrite(handle, PHY_REG_E2P_DATA, data.d);

        while (cmdr.EPC_BUSY == 1)
        {
            cmdr.d = DRV_ETHPHY_SMC9303_SMIExtRead(handle, PHY_REG_E2P_CMD);
        }
    }
    return len;
}

int32_t DRV_ETHPHY_SMSC9303_ReadRegister(DRV_HANDLE handle, uint16_t rIx, uint32_t * val)
{
    if (rIx < PHY_REG_LAST_DIRECT_ACCESS)
    {
        *val = DRV_ETHPHY_SMC9303_SMIExtRead(handle, rIx);
        return 0;
    }

    __SMSC9303_SWITCH_CSR_CMD_t cmd, cmdr;
    cmd.d = 0;
    cmdr.d = 0;
    cmd.CSR_BUSY = 1;
    cmd.R_nW = 1;
    cmd.AUTO_INC = 0;
    cmd.AUTO_DEC = 0;
    cmd.CSR_BE = 0;
    cmd.CSR_ADDR = rIx;

    cmdr.CSR_BUSY = 1;

    DRV_ETHPHY_SMC9303_SMIExtWrite(handle, PHY_REG_SWITCH_CSR_CMD, cmd.d);

    while (cmdr.CSR_BUSY == 1)
    {
        cmdr.d = DRV_ETHPHY_SMC9303_SMIExtRead(handle, PHY_REG_SWITCH_CSR_CMD);
    }

    *val = DRV_ETHPHY_SMC9303_SMIExtRead(handle, PHY_REG_SWITCH_CSR_DATA);

    return 0;
}


int32_t DRV_ETHPHY_SMSC9303_WriteRegister(DRV_HANDLE handle, uint16_t rIx, uint32_t val, uint8_t bytesToWrite)
{
    if (rIx < PHY_REG_LAST_DIRECT_ACCESS)
    {
        DRV_ETHPHY_SMC9303_SMIExtWrite(handle, rIx, val);
        return 0;
    }

    __SMSC9303_SWITCH_CSR_CMD_t cmd, cmdr;
    cmd.d = 0;
    cmdr.d = 0;
    cmd.CSR_BUSY = 1;
    cmd.R_nW = 0;
    cmd.AUTO_INC = 0;
    cmd.AUTO_DEC = 0;
    cmd.CSR_BE = bytesToWrite;
    cmd.CSR_ADDR = rIx;

    cmdr.CSR_BUSY = 1;

    DRV_ETHPHY_SMC9303_SMIExtWrite(handle, PHY_REG_SWITCH_CSR_DATA, val);
    DRV_ETHPHY_SMC9303_SMIExtWrite(handle, PHY_REG_SWITCH_CSR_CMD, cmd.d);

    while (cmdr.CSR_BUSY == 1)
    {
        cmdr.d = DRV_ETHPHY_SMC9303_SMIExtRead(handle, PHY_REG_SWITCH_CSR_CMD);
    }

    return 0;
}


DRV_ETHPHY_RESULT DRV_ETHPHY_Smsc9303LinkStatusGet( DRV_HANDLE handle, DRV_ETHPHY_LINK_STATUS* pLinkStat, bool refresh )
{
    DRV_ETHPHY_CLIENT_OBJ * hClientObj = (DRV_ETHPHY_CLIENT_OBJ *) handle;

    /* Check for the Client validity */
    if(hClientObj == 0)
    {
        return DRV_ETHPHY_RES_HANDLE_ERR;
    }


    if(hClientObj->status != DRV_ETHPHY_CLIENT_STATUS_READY)
    {   // another op going on
        return DRV_ETHPHY_RES_NOT_READY_ERR;
    }

    // basic sanity check
    if(pLinkStat == 0)
    {
        return DRV_ETHPHY_RES_OPERATION_ERR;
    }

    __BMSTATbits_t b1, b2;
    b1.w = DRV_ETHPHY_SMC9303_SMIRead(handle, 1, PHY_REG_BMSTAT);
    b2.w = DRV_ETHPHY_SMC9303_SMIRead(handle, 2, PHY_REG_BMSTAT);
    
    if (pLinkStat)
    {
        if (b1.LINK_STAT || b2.LINK_STAT)
        {
            *pLinkStat = DRV_ETHPHY_LINK_ST_UP;
        }
        else
        {
            *pLinkStat = DRV_ETHPHY_LINK_ST_DOWN;
        }
        if (b1.REM_FAULT || b2.REM_FAULT)
        {
            *pLinkStat |= DRV_ETHPHY_LINK_ST_REMOTE_FAULT;
        }
    }
        
    return DRV_ETHPHY_RES_PENDING;
}