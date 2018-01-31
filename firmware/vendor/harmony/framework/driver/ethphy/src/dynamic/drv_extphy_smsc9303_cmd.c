/*******************************************************************************
  SMSC LAN9303 Command definitions

  Company:
    Microchip Technology Inc.

  File Name:
    eth_pic32_ext_phy_smsc9303_cmd.c

  Summary:
    SMSC LAN9303 Command Implementations

  Description:
    This file provides the SMSC LAN9303 definitions.
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright © 2014 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

#include "drv_extphy_smsc9303_cmd.h"
#include "drv_extphy_smsc9303.h"

#include "system/tmr/sys_tmr.h"

#if SMSC_9303_CMD_PROCESSOR

DRV_HANDLE sHClientObj;

static int _DRV_ETHPHY_SMSC9303_CMD_StartLEDTest(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _DRV_ETHPHY_SMSC9303_CMD_StopLEDTest(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);

static int _DRV_ETHPHY_SMSC9303_CMD_MAC_RX_CNTRs(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _DRV_ETHPHY_SMSC9303_CMD_MAC_TX_CNTRs(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _DRV_ETHPHY_SMSC9303_CMD_SWE_CNTRs(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _DRV_ETHPHY_SMSC9303_CMD_BM_CNTRs(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _DRV_ETHPHY_SMSC9303_CMD_RegRawRead(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _DRV_ETHPHY_SMSC9303_CMD_RegRawWrite(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);

static int _DRV_ETHPHY_SMSC9303_CMD_Main(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int _DRV_ETHPHY_SMSC9303_CMD_PrintHelp(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);

static const SYS_CMD_DESCRIPTOR    _sms9303Tbl[]=
{
    {"smsc9303",     _DRV_ETHPHY_SMSC9303_CMD_Main,              ": SMS LAN 9303 Commands"},
};


int32_t DRV_ETHPHY_SMSC9303_InitCmdProcessor()
{
    if (!SYS_CMD_ADDGRP(_sms9303Tbl, sizeof(_sms9303Tbl)/sizeof(*_sms9303Tbl), "smsc9303", ": SMS LAN 9303 Commands"))
    {
        SYS_ERROR(SYS_ERROR_ERROR, "Failed to create SMS LAN 9303 Commands\r\n");
        return -1;
    }
    return 0;
}

static __SMSC9303_LED_CFG_t _sOriginalLEDValue;
static __SMSC9303_GPIO_DATA_DIR_t _sGPIOData;
static SYS_TMR_HANDLE _ledTestTimer;
static bool _testRunning = false;

void _DRV_ETHPHY_SMSC9303_CMD_LedTestTimer(uintptr_t context, uint32_t currTick)
{
    _sGPIOData.GPIOD0 ^= 1;
    _sGPIOData.GPIOD1 ^= 1;
    _sGPIOData.GPIOD2 ^= 1;
    _sGPIOData.GPIOD3 ^= 1;
    _sGPIOData.GPIOD4 ^= 1;
    _sGPIOData.GPIOD5 ^= 1;

    DRV_ETHPHY_SMSC9303_WriteRegister(sHClientObj, PHY_REG_GPIO_DATA_DIR, _sGPIOData.d, 0xff);
}

int _DRV_ETHPHY_SMSC9303_CMD_StartLEDTest(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{

    DRV_ETHPHY_SMSC9303_ReadRegister(sHClientObj, PHY_REG_LED_CFG, (uint32_t*)&(_sOriginalLEDValue));

    __SMSC9303_LED_CFG_t newLEDValue;
    newLEDValue.d = 0;
    DRV_ETHPHY_SMSC9303_WriteRegister(sHClientObj, PHY_REG_LED_CFG, newLEDValue.d, 0xff);


    _sGPIOData.d = 0;
    _sGPIOData.GPDIR0 = 1;
    _sGPIOData.GPDIR1 = 1;
    _sGPIOData.GPDIR2 = 1;
    _sGPIOData.GPDIR3 = 1;
    _sGPIOData.GPDIR4 = 1;
    _sGPIOData.GPDIR5 = 1;

    _sGPIOData.GPIOD0 = 0;
    _sGPIOData.GPIOD1 = 1;
    _sGPIOData.GPIOD2 = 1;
    _sGPIOData.GPIOD3 = 1;
    _sGPIOData.GPIOD4 = 1;
    _sGPIOData.GPIOD5 = 1;

    DRV_ETHPHY_SMSC9303_WriteRegister(sHClientObj, PHY_REG_GPIO_DATA_DIR, _sGPIOData.d, 0xff);

    __SMSC9303_GPIO_CFG_t gpiocfg;
    gpiocfg.d = 0;

    gpiocfg.GPIOBUF4 = 1;
    DRV_ETHPHY_SMSC9303_WriteRegister(sHClientObj, PHY_REG_GPIO_CFG, gpiocfg.d, 0xff);


    _ledTestTimer = SYS_TMR_CallbackPeriodic(500, 0, &_DRV_ETHPHY_SMSC9303_CMD_LedTestTimer);

    return 0;
}


int _DRV_ETHPHY_SMSC9303_CMD_StopLEDTest(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    SYS_TMR_CallbackStop(_ledTestTimer);

    DRV_ETHPHY_SMSC9303_WriteRegister(sHClientObj, PHY_REG_LED_CFG, _sOriginalLEDValue.d, 0xff);

    return 0;
}

int _DRV_ETHPHY_SMSC9303_CMD_PrintHelp(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    const void* cmdIoParam = pCmdIO->cmdIoParam;
    (*pCmdIO->pCmdApi->msg)(cmdIoParam, "Usage: smsc9303 \r\n");
    (*pCmdIO->pCmdApi->msg)(cmdIoParam, "                ledtest {start|stop} \r\n");
    (*pCmdIO->pCmdApi->msg)(cmdIoParam, "                counter \r\n");
    (*pCmdIO->pCmdApi->msg)(cmdIoParam, "                        mac {rx|tx} {0|1|2}\r\n");
    (*pCmdIO->pCmdApi->msg)(cmdIoParam, "                        swe\r\n");
    (*pCmdIO->pCmdApi->msg)(cmdIoParam, "                        bm\r\n");
    (*pCmdIO->pCmdApi->msg)(cmdIoParam, "                read <reg addr hex>\r\n");
    (*pCmdIO->pCmdApi->msg)(cmdIoParam, "                write <reg addr hex> <reg val hex>\r\n");
    return 0;
}


int _DRV_ETHPHY_SMSC9303_CMD_Main(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    const void* cmdIoParam = pCmdIO->cmdIoParam;
    if (argc < 2)
    {
        _DRV_ETHPHY_SMSC9303_CMD_PrintHelp(pCmdIO, argc, argv);
        return 0;
    }

    if (strcmp("ledtest", argv[1]) == 0)
    {
        if (argc < 3)
        {
            _DRV_ETHPHY_SMSC9303_CMD_PrintHelp(pCmdIO, argc, argv);
            return 0;
        }
        if (strcmp("start", argv[2]) == 0)
        {
            if (_testRunning)
            {
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, "ledtest already running \r\n");
                return 0;
            }
            _DRV_ETHPHY_SMSC9303_CMD_StartLEDTest(pCmdIO, argc, argv);
            _testRunning = true;
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "ledtest started \r\n");
            return 0;
        }
        else if (strcmp("stop", argv[2]) == 0)
        {
            if (!_testRunning)
            {
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, "ledtest not running \r\n");
                return 0;
            }
            _DRV_ETHPHY_SMSC9303_CMD_StopLEDTest(pCmdIO, argc, argv);
            _testRunning = false;
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, "ledtest stopped \r\n");
            return 0;
        }
        else
        {
            _DRV_ETHPHY_SMSC9303_CMD_PrintHelp(pCmdIO, argc, argv);
            return 0;
        }
    }
    else if (strcmp("counter", argv[1]) == 0)
    {
        if (argc < 3)
        {
            _DRV_ETHPHY_SMSC9303_CMD_PrintHelp(pCmdIO, argc, argv);
            return 0;
        }
        if (strcmp("mac", argv[2]) == 0)
        {
            if (argc < 5)
            {
                _DRV_ETHPHY_SMSC9303_CMD_PrintHelp(pCmdIO, argc, argv);
                return 0;
            }
            if (strcmp("rx", argv[3]) == 0)
            {
                return _DRV_ETHPHY_SMSC9303_CMD_MAC_RX_CNTRs(pCmdIO, argc, argv);
            }
            else if (strcmp("tx", argv[3]) == 0)
            {
                return  _DRV_ETHPHY_SMSC9303_CMD_MAC_TX_CNTRs(pCmdIO, argc, argv);
            }
            else
            {
                 _DRV_ETHPHY_SMSC9303_CMD_PrintHelp(pCmdIO, argc, argv);
                return 0;
            }
        }
        else if (strcmp("swe", argv[2]) == 0)
        {
            return _DRV_ETHPHY_SMSC9303_CMD_SWE_CNTRs(pCmdIO, argc, argv);
        }
        else if (strcmp("bm", argv[2]) == 0)
        {
            return _DRV_ETHPHY_SMSC9303_CMD_BM_CNTRs(pCmdIO, argc, argv);
        }
        else
        {
            _DRV_ETHPHY_SMSC9303_CMD_PrintHelp(pCmdIO, argc, argv);
            return 0;
        }
    }
    else if (strcmp("read", argv[1]) == 0)
    {
        return _DRV_ETHPHY_SMSC9303_CMD_RegRawRead(pCmdIO, argc, argv);
    }
    else if (strcmp("write", argv[1]) == 0)
    {
        return _DRV_ETHPHY_SMSC9303_CMD_RegRawWrite(pCmdIO, argc, argv);
    }

    return 0;
}

#define PRINT_CNTR_PORT(CNTRNAME) DRV_ETHPHY_SMSC9303_ReadRegister(sHClientObj, CNTRNAME ## 0 + (port <<10), (uint32_t*)&(cntr)); \
    (*pCmdIO->pCmdApi->print)(cmdIoParam, #CNTRNAME "%d (%04x)= %08X\r\n", port, CNTRNAME ## 0 + (port <<10), cntr.COUNT);

int _DRV_ETHPHY_SMSC9303_CMD_MAC_RX_CNTRs(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{

    uint16_t port = argv[4][0] - '0';

    if (port > 2)
    {
        _DRV_ETHPHY_SMSC9303_CMD_PrintHelp(pCmdIO, argc, argv);
        return 0;
    }

    const void* cmdIoParam = pCmdIO->cmdIoParam;

    __SMSC9303_Count_t cntr;
    cntr.d = 0;

    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_UNDSZE_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_64_CNT_);

    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_65_TO_127_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_128_TO_255_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_256_TO_511_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_512_TO_1023_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_1024_TO_MAX_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_OVRSZE_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_PKTOK_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_CRCERR_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_MULCST_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_BRDCST_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_PAUSE_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_FRAG_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_JABB_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_ALIGN_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_PKTLEN_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_GOODPKTLEN_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_SYMBL_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_RX_CTLFRM_CNT_);

    return 0;
}

int _DRV_ETHPHY_SMSC9303_CMD_MAC_TX_CNTRs(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    uint16_t port = argv[4][0] - '0';

    if (port > 2)
    {
        _DRV_ETHPHY_SMSC9303_CMD_PrintHelp(pCmdIO, argc, argv);
        return 0;
    }
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    __SMSC9303_Count_t cntr;
    cntr.d = 0;

    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_TX_DEFER_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_TX_PAUSE_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_TX_PKTOK_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_TX_64_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_TX_65_TO_127_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_TX_128_TO_255_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_TX_256_TO_511_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_TX_512_TO_1023_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_TX_1024_TO_MAX_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_TX_UNDSIZE_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_TX_PKTLEN_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_TX_BRDCST_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_TX_MULCST_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_TX_LATECOL_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_TX_EXCOL_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_TX_SINGLECOL_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_TX_MULTICOL_CNT_);
    PRINT_CNTR_PORT(PHY_REG_SF_IA_MAC_TX_TOTALCOL_CNT_);


    return 0;
}

#define PRINT_CNTR(CNTRNAME) DRV_ETHPHY_SMSC9303_ReadRegister(sHClientObj, CNTRNAME, (uint32_t*)&(cntr)); \
    (*pCmdIO->pCmdApi->print)(cmdIoParam, #CNTRNAME " (%04x)= %08X\r\n", CNTRNAME, cntr.COUNT);


int _DRV_ETHPHY_SMSC9303_CMD_SWE_CNTRs(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    __SMSC9303_Count_t cntr;
    cntr.d = 0;

    PRINT_CNTR(PHY_REG_SF_IA_SWE_FILTERED_CNT_0);
    PRINT_CNTR(PHY_REG_SF_IA_SWE_FILTERED_CNT_1);
    PRINT_CNTR(PHY_REG_SF_IA_SWE_FILTERED_CNT_2);
    PRINT_CNTR(PHY_REG_SF_IA_SWE_LRN_DISCRD_CNT_0);
    PRINT_CNTR(PHY_REG_SF_IA_SWE_LRN_DISCRD_CNT_1);
    PRINT_CNTR(PHY_REG_SF_IA_SWE_LRN_DISCRD_CNT_2);

    return 0;
}

int _DRV_ETHPHY_SMSC9303_CMD_BM_CNTRs(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    __SMSC9303_Count_t cntr;
    cntr.d = 0;

    PRINT_CNTR(PHY_REG_SF_IA_BM_DRP_CNT_SRC_0);
    PRINT_CNTR(PHY_REG_SF_IA_BM_DRP_CNT_SRC_1);
    PRINT_CNTR(PHY_REG_SF_IA_BM_DRP_CNT_SRC_2);
    PRINT_CNTR(PHY_REG_SF_IA_BM_RATE_DRP_CNT_SRC_0);
    PRINT_CNTR(PHY_REG_SF_IA_BM_RATE_DRP_CNT_SRC_1);
    PRINT_CNTR(PHY_REG_SF_IA_BM_RATE_DRP_CNT_SRC_2);

    return 0;
}

int _DRV_ETHPHY_SMSC9303_CMD_RegRawRead(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{

    if (argc < 3)
    {
        _DRV_ETHPHY_SMSC9303_CMD_PrintHelp(pCmdIO, argc, argv);
        return 0;
    }

    const void* cmdIoParam = pCmdIO->cmdIoParam;

    uint32_t reg = strtol(argv[2], NULL, 16);
    uint32_t val;

    int32_t result = DRV_ETHPHY_SMSC9303_ReadRegister(sHClientObj, reg, &val);
    (*pCmdIO->pCmdApi->print)(cmdIoParam, "Register Read: Result (%08x) 0x(%08x)= 0x%08X\r\n", result, reg, val);

    return 0;
}

int _DRV_ETHPHY_SMSC9303_CMD_RegRawWrite(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    if (argc < 4)
    {
        _DRV_ETHPHY_SMSC9303_CMD_PrintHelp(pCmdIO, argc, argv);
        return 0;
    }
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    __SMSC9303_Count_t cntr;
    cntr.d = 0;

    uint32_t reg = strtol(argv[2], NULL, 16);
    uint32_t val = strtol(argv[3], NULL, 16);

    int32_t result = DRV_ETHPHY_SMSC9303_WriteRegister(sHClientObj, reg, val, 0xf);
    (*pCmdIO->pCmdApi->print)(cmdIoParam, "Register Write: Result (%08x) 0x(%08x)= 0x%08X\r\n", result, reg, val);

    return 0;
}


#endif

