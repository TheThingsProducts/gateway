/*******************************************************************************
  MAC Configuration file

  Company:
    Microchip Technology Inc.
    
  File Name:
    tcpip_mac_config.h

  Summary:
    Configuration file
    
  Description:
    This file contains the MAC module configuration options
    
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright © 2013 released Microchip Technology Inc.  All rights reserved.

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
#ifndef _MAC_CONFIG_H_
#define _MAC_CONFIG_H_

// =======================================================================
//   TCPIP_MODULE_MAC_PIC32INT - PIC32MX PIC32MZ MAC Layer Options
//   If not using a PIC32MX or PIC32MZ device with internal MAC, ignore this section.
// =======================================================================

// MAC Configuration parameters.
// Note: These values are used as defaults.
// The actual values passed in at initialization time
// take precedence.

// Number of the TX descriptors to be created.
// Because a TCP packet can span at most 3 buffers,
// the value should always be >= 4
// The amount of memory needed per descriptor is not high (around 24 bytes)
// so when high MAC TX performance is needed
// make sure that this number is >= 8.
#define TCPIP_EMAC_TX_DESCRIPTORS	8

// Number of the RX descriptors to be created.
// If not using the run time replenish mechanism (see below)
// it should match the number of dedicated buffers:
// TCPIP_EMAC_RX_DEDICATED_BUFFERS;
// Otherwise it should be bigger than the sum of dedicated + non-dedicated buffers:
// TCPIP_EMAC_RX_DESCRIPTORS > TCPIP_EMAC_RX_DEDICATED_BUFFERS + replenish_buffers
#define TCPIP_EMAC_RX_DESCRIPTORS	10

// Number of MAC dedicated RX packet buffers.
// These buffers are always owned by the MAC.
// Note that the MAC driver allocates these buffers for
// storing the incoming network packets.
// The bigger the storage capacity, the higher data throughput can be obtained.
// Note that these packet buffers are allocated from the private TCP/IP heap
// that is specified by the TCPIP_STACK_DRAM_SIZE setting.
#define    TCPIP_EMAC_RX_DEDICATED_BUFFERS  4

// Number of non-dedicated buffers for the MAC initialization
// Buffers allocated at the MAC driver initialization.
#define    TCPIP_EMAC_RX_INIT_BUFFERS       0

// Minumum threshold for the buffer replenish process.
// Whenever the number of RX scheduled buffers is <= than this threshold
// the MAC driver will allocate new non-dedicated buffers
// (meaning that they will be released to the TCP/IP heap once they are processed).
// Setting this value to 0 disables the buffer replenishing process.
#define    TCPIP_EMAC_RX_LOW_THRESHOLD      1

// Number of RX buffers to allocate when below threshold condition is detected.
// If 0, the MAC driver will allocate (scheduled buffers - rxThres)
// If !0, the MAC driver will allocate exactly TCPIP_EMAC_RX_LOW_FILL buffers
#define    TCPIP_EMAC_RX_LOW_FILL           2


// Size of a RX packet buffer. Should be multiple of 16.
// This is the size of all receive packet buffers processed by the ETHC.
// The size should be enough to accommodate any network received packet.
// If the packets are larger, they will have to take multiple RX buffers
// and the packet manipulation is less efficient.
//#define	TCPIP_EMAC_RX_BUFF_SIZE	512
// Together with TCPIP_EMAC_RX_DEDICATED_BUFFERS it has impact on TCPIP_STACK_DRAM_SIZE setting.
#define	TCPIP_EMAC_RX_BUFF_SIZE	1536

// Maximum MAC supported RX frame size.
// Any incoming ETH frame that's longer than this size
// will be discarded.
// The default value is 1536
// (allows for VLAN tagged frames, although the VLAN
// tagged frames are discarded).
// Normally there's no need to touch this value
// unless you know exactly the maximum size of the 
// frames you want to process or you need to control
// packets fragmentation (together with the TCPIP_EMAC_RX_BUFF_SIZE).
// Note: Always multiple of 16.
#define TCPIP_EMAC_RX_MAX_FRAME   1536


// MAC maximum number of supported fragments.
// Based on the values of TCPIP_EMAC_RX_MAX_FRAME and TCPIP_EMAC_RX_BUFF_SIZE
// an incoming frame may span multiple RX buffers (fragments).
// Note that excessive fragmentation leads to performance degradation.
// The default and recommended value should be 1.
//#define TCPIP_EMAC_RX_FRAGMENTS      1
// Alternatively you can use the calculation of the
// number of fragments based on the selected RX sizes:
#define TCPIP_EMAC_RX_FRAGMENTS      ((TCPIP_EMAC_RX_MAX_FRAME + (TCPIP_EMAC_RX_BUFF_SIZE -1 )) / (TCPIP_EMAC_RX_BUFF_SIZE))


// MAC RX Filters
// These filters define the packets that are accepted and rejected by the MAC driver
// Adjust to your needs
// The default value allows the processing of unicast, multicast and broadcast packets that 
// have a valid CRC
#define TCPIP_EMAC_RX_FILTERS   (TCPIP_MAC_RX_FILTER_TYPE_DEFAULT)

// Flags to use for the Ethernet connection
// A TCPIP_ETH_OPEN_FLAGS value.
// Set to TCPIP_ETH_OPEN_DEFAULT unless very good reason
// to use different value
#define TCPIP_EMAC_ETH_OPEN_FLAGS     (TCPIP_ETH_OPEN_DEFAULT)

// Flags to configure the MAC ->PHY connection
// a DRV_ETHPHY_CONFIG_FLAGS
// This depends on the actual connection
// (MII/RMII, default/alternate I/O)
// The DRV_ETHPHY_CFG_AUTO value will use the configuration
// fuses setting
#define TCPIP_EMAC_PHY_CONFIG_FLAGS   (DRV_ETHPHY_CFG_AUTO)


// The value of the delay for the link initialization, ms
// This insures that the PHY is ready to transmit after it is reset
// A usual value is 500 ms.
// Adjust to your needs.
#define TCPIP_EMAC_PHY_LINK_INIT_DELAY        (500)


// The PHY address, as configured on the board.
// By default all the PHYs respond to address 0
#define TCPIP_EMAC_PHY_ADDRESS                (0)



// =======================================================================
//   TCPIP_MODULE_MAC_MRF24W - MAC Layer Options
//   If not using an external MRF24W Wi-Fi MAC, ignore this section.
// =======================================================================



#endif  // _MAC_CONFIG_H_


