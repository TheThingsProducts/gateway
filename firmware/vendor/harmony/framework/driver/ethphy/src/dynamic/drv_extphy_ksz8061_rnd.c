/*******************************************************************************
  Micrel KSZ8061RND PHY API for Microchip TCP/IP Stack
*******************************************************************************/

/*******************************************************************************
File Name:  drv_extphy_ksz8061_rnd.c
Copyright © 2012 released Microchip Technology Inc.  All rights
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

#include "driver/ethphy/src/dynamic/drv_extphy_ksz8061_rnd.h"

/****************************************************************************
 *                 interface functions
 ****************************************************************************/


/****************************************************************************
 * Function:        DRV_EXTPHY_MIIConfigure
 *
 * PreCondition:    - Communication to the PHY should have been established.
 *
 * Input:           handle - A valid open-instance handle, returned from the driver's open routine
 *                  cFlags - the requested configuration flags: DRV_ETHPHY_CFG_RMII/DRV_ETHPHY_CFG_MII
 *
 * Output:          DRV_ETHPHY_RES_OK - operation completed successfully
 *
 *                  DRV_ETHPHY_RES_PENDING - operation pending; the call needs to be re-issued until
 *                                    DRV_ETHPHY_RES_OK or an error reported
 *
 *
 *                  < 0         - an error occurred and the operation was aborted
 *
 *
 * Side Effects:    None
 *
 * Overview:        This function configures the PHY in one of MII/RMII operation modes.
 *
 * Note:            None
 *****************************************************************************/
static DRV_ETHPHY_RESULT DRV_EXTPHY_MIIConfigure(const DRV_ETHPHY_OBJECT_BASE* pBaseObj, DRV_HANDLE hClientObj, DRV_ETHPHY_CONFIG_FLAGS cFlags)
{
    // done - the module is RMII only
    return DRV_ETHPHY_RES_OK;
}


/****************************************************************************
 * Function:        DRV_EXTPHY_MDIXConfigure
 *
 * PreCondition:    - Communication to the PHY should have been established.
 *
 * Input            handle - A valid open-instance handle, returned from the driver's open routine
 *                  oFlags - the requested open flags: TCPIP_ETH_OPEN_MDIX_AUTO, TCPIP_ETH_OPEN_MDIX_NORM/TCPIP_ETH_OPEN_MDIX_SWAP
 *
 * Output:          DRV_ETHPHY_RES_OK - operation completed successfully
 *
 *                  DRV_ETHPHY_RES_PENDING - operation pending; the call needs to be re-issued until
 *                                    DRV_ETHPHY_RES_OK or an error reported
 *
 *
 *                  < 0         - an error occurred and the operation was aborted
 *
 * Side Effects:    None
 *
 * Overview:        This function configures the MDIX mode for the PHY.
 *
 * Note:            None
 *****************************************************************************/
static DRV_ETHPHY_RESULT DRV_EXTPHY_MDIXConfigure(const DRV_ETHPHY_OBJECT_BASE* pBaseObj, DRV_HANDLE hClientObj, TCPIP_ETH_OPEN_FLAGS oFlags)
{
    union
    {
        struct
        {
            uint16_t low;
            uint16_t high;
        };
        uint32_t    w;
    }vendorData;

    uint16_t    phyReg;
    uint16_t    mdixConfPhase;
    int         phyAddress;

    DRV_ETHPHY_RESULT res = pBaseObj->DRV_ETHPHY_VendorDataGet(hClientObj, &vendorData.w);

    if(res < 0)
    {   // some error occurred
        return res;
    }

    mdixConfPhase = vendorData.low;

    pBaseObj->DRV_ETHPHY_PhyAddressGet(hClientObj, &phyAddress);

    switch(mdixConfPhase)
    {
        case 0:
            res = pBaseObj->DRV_ETHPHY_VendorSMIReadStart(hClientObj, PHY_REG_PHY_CTRL_2, phyAddress);
            if(res < 0)
            {   // some error
                return res;
            }
            else if(res == DRV_ETHPHY_RES_PENDING)
            {   // retry
                return DRV_ETHPHY_RES_PENDING;
            }
            
            // advance to the next phase
            pBaseObj->DRV_ETHPHY_VendorDataSet(hClientObj, ++mdixConfPhase);
            return DRV_ETHPHY_RES_PENDING;

        case 1:
            res = pBaseObj->DRV_ETHPHY_VendorSMIReadResultGet(hClientObj, &phyReg);
            if(res < 0)
            {   // some error
                return res;
            }
            else if(res == DRV_ETHPHY_RES_PENDING)
            {   // retry
                return DRV_ETHPHY_RES_PENDING;
            }

            // got PHY_REG_SPECIAL_CTRL result
            // not used bits should be 0
            phyReg &= _PHY2CTRL_MDI_MDIX_SELECT_MASK | _PHY2CTRL_PAIR_SWAP_DISABLE_MASK;

            if(oFlags & TCPIP_ETH_OPEN_MDIX_AUTO)
            {   // enable Auto-MDIX
                phyReg &= ~_PHY2CTRL_PAIR_SWAP_DISABLE_MASK;
            }
            else
            {   // no Auto-MDIX
                phyReg |= _PHY2CTRL_PAIR_SWAP_DISABLE_MASK;    // disable Auto-MDIX
                if(oFlags & TCPIP_ETH_OPEN_MDIX_SWAP)
                {
                    phyReg |= _PHY2CTRL_MDI_MDIX_SELECT_MASK; // swap
                }
                else
                {
                    phyReg &= ~_PHY2CTRL_MDI_MDIX_SELECT_MASK;    // normal
                }
            }
            vendorData.low = mdixConfPhase + 1;
            vendorData.high = phyReg;
            pBaseObj->DRV_ETHPHY_VendorDataSet(hClientObj, vendorData.w);
            return DRV_ETHPHY_RES_PENDING;
 
        case 2:
            phyReg = vendorData.high;
            // update the PHY_REG_SPECIAL_CTRL Register
            res = pBaseObj->DRV_ETHPHY_VendorSMIWriteStart(hClientObj, PHY_REG_PHY_CTRL_2, phyReg, phyAddress);
            if(res < 0)
            {   // some error
                return res;
            }
            else if(res == DRV_ETHPHY_RES_PENDING)
            {   // retry
                return DRV_ETHPHY_RES_PENDING;
            }

            // done    
            return DRV_ETHPHY_RES_OK;


        default:
            // shouldn't happen
            return DRV_ETHPHY_RES_OPERATION_ERR; 
    }

}


/****************************************************************************
 * Function:        DRV_EXTPHY_SMIClockGet
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
    return DRV_ETHPHY_SMI_CLOCK_SPEED;     //  2.5 MHz max clock supported
}

// the DRV_ETHPHY_OBJECT

const DRV_ETHPHY_OBJECT  DRV_ETHPHY_OBJECT_MICREL_KSZ8061RND = 
{
    DRV_EXTPHY_MIIConfigure,
    DRV_EXTPHY_MDIXConfigure,
    DRV_EXTPHY_SMIClockGet,
    0,                          // no WOL functionality yet
};


