/*******************************************************************************
  Micrel KSZ8061RND definitions

  Company:
    Microchip Technology Inc.
    
  File Name:
    drv_extphy_ksz8061_rnd.h

  Summary:
    Micrel KSZ8061RND register definitions

  Description:
    This file provides the Micrel KSZ8061RND definitions.

*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright © 2012 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _KSZ_8061_RND_H_

#define _KSZ_8061_RND_H_

#ifdef DRV_ETHPHY_SMI_CLOCK_SPEED
    #warning Ethernet PHY SMI Clock Speed previously defined. Please make sure that it is smaller than 2.5 MHz
#else
    #define DRV_ETHPHY_SMI_CLOCK_SPEED 2500000
#endif 

typedef enum
{
    /*
    // basic registers, accross all registers: 0-1
    PHY_REG_BMCON       = 0,
    PHY_REG_BMSTAT      = 1,
    // extended registers: 2-15
    PHY_REG_PHYID1      = 2,
    PHY_REG_PHYID2      = 3,
    PHY_REG_ANAD        = 4,
    PHY_REG_ANLPAD      = 5,
    PHY_REG_ANLPADNP    = 5,
    PHY_REG_ANEXP       = 6,
    PHY_REG_ANNPTR      = 7,
    PHY_REG_ANLPRNP     = 8,
    */
    /*
    // specific vendor registers: 16-31
    PHY_REG_SILICON_REV     = 16,
    PHY_REG_MODE_CTRL       = 17,
    PHY_REG_SPECIAL_MODE    = 18,
    PHY_REG_SYMBOL_ERR_CNT  = 26,
    PHY_REG_SPECIAL_CTRL    = 27,
    PHY_REG_INT_SOURCE      = 29,
    PHY_REG_INT_MASK        = 30,
    PHY_REG_PHY_CTRL        = 31,
    */
    // specific vendor registers: 16-31
    PHY_REG_DIGITAL_CTRL            = 16,
    PHY_REG_AFE_CTRL_0              = 17,
    PHY_REG_AFE_CTRL_2              = 19,
    PHY_REG_AFE_CTRL_3              = 20,
    PHY_REG_RXER_CNT                = 21,
    PHY_REG_OP_MODE                 = 22,
    PHY_REG_OP_MODE_STRAP_STATUS    = 23,
    PHY_REG_EXPAND_XTRL             = 24,
    PHY_REG_INTERRUPT_CTRL_STATUS   = 27,
    PHY_REG_FUNCTION_CTRL           = 28,
    PHY_REG_LINKMD_CTRL_STATUS      = 29,
    PHY_REG_PHY_CTRL_1              = 30,
    PHY_REG_PHY_CTRL_2              = 31,
    //
    //PHY_REGISTERS     = 32    // total number of registers
}ePHY_VENDOR_REG;
// updated version of ePHY_REG


// vendor registers
// Register 10h - Digital Control Register
//10.15:5 Reserved RW 0000_0000_000
//10.4 PLL off in EDPD Mode RW 0
//    This mode may optionally be combined with
//    EDPD mode for additional power reduction.
//    1 = PLL is off in EDPD mode
//    0 = PLL is on in EDPD mode
//10.3:0 Reserved RW 0000
typedef union {
  struct {
    unsigned                        :11;
    unsigned PLL_OFF_IN_EPD_MODE    :1; 
    unsigned                        :4;
  };
  struct {
    unsigned short w                :16;
  };
} __DIGITALCTRLbits_t;   // Register 16 - Digital Control Register
#define _DIGITALCTRL_REG_MASK        0x0010

// Register 11h ? AFE Control 0 Register
// 11.15:7 Reserved RW 0000_0000_0
// 11.6 Slow Oscillator Mode RW 0
//     This mode substitutes the 25MHz clock with a
//     slow oscillator clock, to save oscillator power
//     during power down.
//     1 = Slow Oscillator mode enabled
//     0 = Slow Oscillator mode disabled
// 11.5:0 Reserved RW 00_0000
typedef union {
  struct {
    unsigned                  :9;
    unsigned SLOW_OSC_MODE    :1;
    unsigned                  :6;
  };
  struct {
    unsigned short w:16;
  };
} __AFECTRL_0_bits_t; // Register 17 ? AFE Control 0 Register
#define _AFECTRL0_SLOWOSC_MASK         0x0040

// Register 13h ? AFE Control 2 Register
//13.15 LinkMD Detector Threshold RW 0
//    Sets the threshold for the LinkMD pulse 
//    detector. Use high threshold with the large
//    LinkMD pulse, and the low threshold with the
//    small LinkMD pulse.
//    Also see MMD address 1Bh, register 0h bits[7:4].
//    1 = Enable high threshold comparator
//    0 = Disable high threshold comparator
//13.14:1 Reserved RW 000_0000_0000_000
//13.0 Slow Oscillator Mode for Ultra Deep Sleep Mode RW 0
//    This mode substitutes the 25MHz clock with a
//    slow oscillator clock, to save oscillator power if
//    register access is required during Ultra Deep
//    Sleep Mode. Note that the 1.2V supply is
//    required if this mode is used.
//    1 = Slow Oscillator mode enabled
//    0 = Slow Oscillator mode disabled
typedef union {
  struct {
    unsigned LINKMD_DETECT_TRSH     :1;
    unsigned                        :14;
    unsigned ULTRA_DEEP_SLEEP_MODE  :1;
  };
  struct {
    unsigned short w                :16;
  };
} __AFECTRL_1_bits_t;  // Register 19 ? AFE Control 2 Register
#define _AFECTRL1_LINKMD_DET_MASK                   0x8000
#define _AFECTRL1_ULTRA_DEEP_SLEEP_MODE_MASK        0x0001

// Register 14h ? AFE Control Register 3
//14.15:7 Reserved RW 0000_0000_0
//14.6 Ultra Deep Sleep method RW 0
//    1 = CPU Control method. Entry into Ultra Deep
//        Sleep Mode determined by value of register bit 14.5
//    0 = Automatic method. Enter into Ultra Deep
//        Sleep Mode automatically when no cable
//        energy is detected
//14.5 Manual Ultra Deep Sleep Mode RW 0
//    1 = Enter into Standby Mode
//    0 = Normal Mode
//    This bit is used to enter Ultra Deep Sleep Mode
//    when the CPU Control method is selected in bit 14.6. 
//    To exit Ultra Deep Sleep Mode, a hardware reset is required.
//14.4 NV Register Access RW 0
//    1 = Enable the non-volatile copy of register 14h
//        and bits [9:8] and [1:0] of 13h.
//    0 = Disable access to non-volatile registers
//    When Ultra Deep Sleep Mode is enabled, this
//    bit must be set to 1.
//14.3 Ultra Deep Sleep Mode and SIGDET Enable RW 0
//    1 = Ultra Deep Sleep Mode is enabled (but not
//        necessarily entered), and SIGDET indicates
//        cable energy detected
//    0 = Ultra Deep Sleep Mode is disabled, and
//        SIGDET output signal is forced true.
//14.2 Disable RX internal termination RW 0
//    1 = Disable RX internal termination
//    0 = Enable RX internal termination
//    [Has no effect on TX internal termination.]
//14.1 Signal Detect de-assertion timing delay RW 0
//    When Ultra Deep Sleep Mode is enabled, this
//    bit determines the delay from loss of cable
//    energy to de-assertion of SIGDET. When
//    automatic method is selected for Ultra Deep
//    Sleep Mode, this delay also applies to powering
//    down.
//    1 = Increased delay. This setting is required to
//        allow automatic exiting of Ultra Deep Sleep
//        Mode (automatic method) if the link partner
//        auto-negotiation is enabled, if auto-MDI/MDI-X
//        is enabled, or if linking at 10Base-T.
//    0 = Minimum delay. When using the Automatic
//        method for Ultra Deep Sleep Mode, use this
//        setting only if the link partner?s auto-negotiation
//        is disabled, auto-MDI/MDI-X is disabled, and
//        linking is at 100Base-TX. This setting may also
//        be used for CPU Control method.
//14.0 Signal Detect polarity RW 0
//    1 = SIGDET is active low (low = signal
//        detected)
//    0 = SIGDET is active high (high = signal
//        detected)
typedef union {
  struct {
    unsigned                                        :9;
    unsigned ULTRA_DEEP_SLEEP_METHOD                :1;
    unsigned MANUAL_ULTRA_DEEP_SLEEP_MODE           :1;
    unsigned NV_REG_ACCESS                          :1;
    unsigned ULTRA_DEEP_SLEEP_MODE_AND_SIGDET_EN    :1;
    unsigned DISABLE_RX_TERMINATION                 :1;
    unsigned SIGNAL_DETECT_DEASSERTION_TIMING_DELAY :1;
    unsigned SIGNAL_DETECT_POLARITY                 :1;
  };
  struct {
    unsigned short w      :16;
  };
} __AFECTRL_3_bits_t;  // Register 19 ? AFE Control 2 Register
#define _AFECTRL3_UDSP_METHOD           0x0040;
#define _AFECTRL3_MAN_UDSP_MODE         0x0020;
#define _AFECTRL3_NV_REG_ACCESS         0x0010;
#define _AFECTRL3_UDSP_AND_SIGDET_EN    0x0008;
#define _AFECTRL3_DIS_RX_TERM           0x0004;
#define _AFECTRL3_SIG_DET_DTD           0x0002;
#define _AFECTRL3_SIG_DET_POL           0x0001;

// Register 15h ? RXER Counter
// 15.15:0 RXER Counter Receive error counter for symbol error frames RO/SC 0000h
typedef union {
  struct {
    unsigned RXER_CNT   :16;
  };
  struct {
    unsigned short w    :16;
  };
} __REXRCNTbits_t;  // Register 15h ? RXER Counter
#define _REXRCNT_MASK       0xFFFF

// Register 16h ? Operation Mode
// 16.15:13 Reserved RW 000
// 16.12 QWF disable
//    1 = Disable Quiet-WIRE Filtering
//    0 = Enable Quiet-WIRE Filtering
//    RW Strapping input at RXER pin
// 16.11:0 Reserved RW 0000_0000_0000
typedef union {
  struct {
    unsigned                :3;
    unsigned QWF_DISABLE    :1;
    unsigned                :12;
  };
  struct {
    unsigned short w  :16;
  };
} __OPERATIONMODEbits_t;    // Register 16h ? Operation Mode
#define _OPERATIONMODE_QWF_DISABLE_MASK       0xFFFF

// Register 17h ? Operation Mode Strap Status
// 17.15:13 PHYAD[2:0] strap-in status RO xxx
//[000] = Strap to PHY Address 0
//[001] = Strap to PHY Address 1
//[010] = Strap to PHY Address 2
//[011] = Strap to PHY Address 3
//[100] = Strap to PHY Address 4
//[101] = Strap to PHY Address 5
//[110] = Strap to PHY Address 6
//[111] = Strap to PHY Address 7
// 17.12:9 Reserved RO
// 17.8 QWF strap-in status RO x
//    1 = Strap to enable Quiet-WIRE Filtering
// 17.7 Reserved RO 0
// 17.6 RMII B-to-B strap-in status RO x
//    1 = Strap to RMII Back-to-Back mode
// 17.5 NAND Tree strap-in status RO x
//    1 = Strap to NAND Tree mode
// 17.4:2 Reserved RO 0
// 17.1 RMII strap-in status 
    // 1 = Strap to RMII normal mode RO
// 17.0 Reserved RO 0
typedef union {
  struct {
    unsigned PHYADD         :3;
    unsigned                :4;
    unsigned QWF            :1;
    unsigned                :1;
    unsigned RMII_B_TO_B    :1;
    unsigned NAND_TREE      :1;
    unsigned                :3;
    unsigned RMII           :1;
    unsigned                :1;
  };
  struct {
    unsigned short w  :16;
  };
} __OPMODESTRAPSTATUSbits_t;  // Register 17h ? Operation Mode Strap Status

// Register 18h ? Expanded Control
// 18.15:12 Reserved RW 0000
// 18.11 Energy Detect Power Down Mode disable RW 1
//    1 = Disable Energy Detect Power Down (EDPD) Mode
//    0 = Enable EDPD Mode
// 18.10 RX PHY Latency RW 0
//     1 = Variable RX PHY latency with no preamble suppression
//     0 = Fixed RX PHY latency with possible suppression of one preamble octet
// 18.9:7 Reserved RW 00_0
// 18.6 Enable 10BT Preamble RW 0
//     When in Back-to-Back Mode and in 10Base-T, this bit must be set.
// 18.5:0 Reserved RW 00_0001
typedef union {
  struct {
    unsigned                        :4; // 15:12
    unsigned EDPD_DISBL             :1; // 11
    unsigned RX_PHY_LATENCY         :1; // 10
    unsigned                        :1; // 9:7
    unsigned ENABLE_10BT_PREAMBLE   :1; // 6
    unsigned                        :6; // 5:0
  };
  struct {
    unsigned short w       :16;
  };
} __EXPANDEDCTRLbits_t;  // reg 31: PHY_REG_PHY_CTRL
#define _EXPANDEDCTRL_EDPD_DISBL_MASK           0x0800
#define _EXPANDEDCTRL_RX_PHY_LATENCY_MASK       0x0400
#define _EXPANDEDCTRL_ENABLE_10BT_PREAMBLE_MASK       0x0040

// Register 1Bh ? Interrupt Control/Status
// 1B.15 Jabber Interrupt Enable RW 0
//    1 = Enable Jabber Interrupt
//    0 = Disable Jabber Interrupt 
//1B.14 Receive Error Interrupt Enable RW 0
//    1 = Enable Receive Error Interrupt
//    0 = Disable Receive Error Interrupt
//1B.13 Page Received Interrupt Enable RW 0
//    1 = Enable Page Received Interrupt
//    0 = Disable Page Received Interrupt
//1B.12 Parallel Detect Fault Interrupt Enable RW 0
//    1 = Enable Parallel Detect Fault Interrupt
//    0 = Disable Parallel Detect Fault Interrupt
//1B.11 Link Partner Acknowledge Interrupt Enable
//    1 = Enable Link Partner Acknowledge Interrupt
//    0 = Disable Link Partner Acknowledge Interrupt
//1B.10 Link Down Interrupt Enable RW 0
//    1= Enable Link Down Interrupt
//    0 = Disable Link Down Interrupt
//1B.9 Remote Fault Interrupt Enable RW 0
//    1 = Enable Remote Fault Interrupt
//    0 = Disable Remote Fault Interrupt
//1B.8 Link Up Interrupt Enable RW 0
//    1 = Enable Link Up Interrupt
//    0 = Disable Link Up Interrupt
//1B.7 Jabber Interrupt RO/SC 0
//    1 = Jabber occurred
//    0 = Jabber did not occurred
//1B.6 Receive Error Interrupt RO/SC 0
//    1 = Receive Error occurred
//    0 = Receive Error did not occurred RO/SC 0
//1B.5 Page Receive Interrupt RO/SC 0
//    1 = Page Receive occurred
//    0 = Page Receive did not occur RO/SC 0
//1B.4 Parallel Detect Fault Interrupt RO/SC 0
//    1 = Parallel Detect Fault occurred
//    0 = Parallel Detect Fault did not occur RO/SC 0
//1B.3 Link Partner Acknowledge Interrupt RO/SC 0
//    1 = Link Partner Acknowledge occurred
//    0 = Link Partner Acknowledge did not occur RO/SC 0
//1B.2 Link Down Interrupt RO/SC 0
//    1 = Link Down occurred
//    0 = Link Down did not occur RO/SC 0
//1B.1 Remote Fault Interrupt RO/SC 0
//    1 = Remote Fault occurred
//    0 = Remote Fault did not occur RO/SC 0
//1B.0 Link Up Interrupt RO/SC 0
//    1 = Link Up occurred 
//    0 = Link Up did not occur
typedef union {
  struct {
    unsigned JABBER_IE                  :1; // 15
    unsigned RX_ERR_IE                  :1; // 14
    unsigned PAGE_RECEIVED_IE           :1; // 13
    unsigned PARALLEL_DETECT_FAULT_IE   :1; // 12
    unsigned LINK_PARTNER_ACK_IE        :1; // 11
    unsigned LINK_DOWN_IE               :1; // 10
    unsigned REMOTE_FAULT_IE            :1; // 9
    unsigned LINK_UP_IE                 :1; // 8
    unsigned JABBER_IF                  :1; // 7
    unsigned RX_ERR_IF                  :1; // 6
    unsigned PAGE_RECEIVED_IF           :1; // 5
    unsigned PARALLEL_DETECT_FAULT_IF   :1; // 4
    unsigned LINK_PARTNER_ACK_IF        :1; // 3
    unsigned LINK_DOWN_IF               :1; // 2
    unsigned REMOTE_FAULT_IF            :1; // 1
    unsigned LINK_UP_IF                 :1; // 0
  };
  struct {
    unsigned short w       :16;
  };
} __ICSbits_t;  // Register 1Bh ? Interrupt Control/Status
#define _ICS_JABBER_IE_MASK                 0x8000
#define _ICS_RX_ERR_IE_MASK                 0x4000
#define _ICS_PAGE_RECEIVED_IE_MASK          0x2000
#define _ICS_PARALLEL_DETECT_FAULT_IE_MASK  0x1000
#define _ICS_LINK_PARTNER_ACK_IE_MASK       0x0800
#define _ICS_LINK_DOWN_IE_MASK              0x0400
#define _ICS_REMOTE_FAULT_IE_MASK           0x0200
#define _ICS_LINK_UP_IE_MASK                0x0100
#define _ICS_JABBER_IF_MASK                 0x0080
#define _ICS_RX_ERR_IF_MASK                 0x0040
#define _ICS_PAGE_RECEIVED_IF_MASK          0x0020
#define _ICS_PARALLEL_DETECT_FAULT_IF_MASK  0x0010
#define _ICS_LINK_PARTNER_ACK_IF_MASK       0x0008
#define _ICS_LINK_DOWN_IF_MASK              0x0004
#define _ICS_REMOTE_FAULT_IF_MASK           0x0002
#define _ICS_LINK_UP_IF_MASK                0x0001

// Register 1Ch ? Function Control
//1C.15:6 Reserved RW 0000_0000_00
//1C.5 Local Loopback Option RW 0
//    1 = Enable local loopback
//    0 = Disable local loopback
//    Local loopback must be enabled both here and in register 0h.
//1C.4:0 Reserved RW 1_0000
typedef union {
  struct {
    unsigned                        :10; // 15:6
    unsigned LL_OPT                 :1; // 5
    unsigned                        :5; // 4:0
  };
  struct {
    unsigned short w       :16;
  };
} __FUNCCTRLbits_t;  // Register 1Ch ? Function Control
#define _FUNCCTRL_LL_OPT_MASK           0x0020

// Register 1Dh ? LinkMD® Control/Status
//1D.15 Cable Diagnostic Test Enable RW/SC 0
//    1 = Enable cable diagnostic test. After test has
//        completed, this bit is self-cleared.
//    0 = Indicates cable diagnostic test (if enabled)
//        has completed and the status information is valid for read.
//1D.14:13 Cable Diagnostic Test Result RO 00
//    [00] = normal condition
//    [01] = open condition has been detected in cable
//    [10] = short condition has been detected in cable
//    [11] = cable diagnostic test has failed 
//1D.12 Short Cable Indicator RO 0
//    1 = Short cable (<10 meter) has been detected by LinkMD®.
//1D.11:9 Reserved RW 000
//1D.8:0 Cable Fault Counter Distance to fault RO 0_0000_0000
typedef union {
  struct {
    unsigned TEST_EN                :1; // 15
    unsigned TEST_RES               :2; // 14:13
    unsigned SHORT_CABLE_INDICATOR  :1; // 12
    unsigned                        :3; // 11:9
    unsigned CABLE_FAULT_COUNTER    :9; // 8:0
  };
  struct {
    unsigned short w       :16;
  };
} __LINKMDCTRLSTATUSbits_t;  // Register 1Dh ? LinkMD® Control/Status
#define _LINKMDCTRLSTATUS_TEST_EN_MASK                  0x8000
#define _LINKMDCTRLSTATUS_TEST_RES_MASK                 0x6000
#define _LINKMDCTRLSTATUS_SHORT_CABLE_INDICATOR_MASK    0x4000
#define _LINKMDCTRLSTATUS_CABLE_FAULT_COUNTER_MASK      0x01FF
            
// Register 1Eh ? PHY Control 1
//1E.15:10 Reserved RO 0000_00
//1E.9 Enable Pause (Flow Control) RO 0
//    1 = Flow control capable
//    0 = No flow control capability
//1E.8 Link Status RO
//    1 = Link is up
//    0 = Link is down
//1E.7 Polarity Status RO
//    1 = Polarity is reversed
//    0 = Polarity is not reversed
//1E.6 Reserved RO 0
//1E.5 MDI/MDI-X State RO
//    1 = MDI-X
//    0 = MDI
//1E.4 Energy Detect RO
//    1 = Presence of signal on receive differential pair
//    0 = No signal detected on receive differential pair
//1E.3 PHY Isolate RW 0
//    1 = PHY in isolate mode
//    0 = PHY in normal operation
//    [Same as register bit 0.10]
//1E.2:0 Operation Mode Indication RO
//    [000] = still in auto-negotiation
//    [001] = 10Base-T half-duplex
//    [010] = 100Base-TX half-duplex
//    [011] = reserved
//    [100] = reserved
//    [101] = 10Base-T full-duplex
//    [110] = 100Base-TX full-duplex
//    [111] = reserved
typedef union {
  struct {
    unsigned                        :6; // 15:10
    unsigned ENABLE_PAUSE           :1; // 9
    unsigned LINK_STATUS           :1; // 8
    unsigned POLARITY_STATUS           :1; // 7
    unsigned            :1; // 6
    unsigned MDI_MDIX_STATE           :1; // 5
    unsigned ENERGY_DETECT           :1; // 4
    unsigned PHY_ISOLATE           :1; // 3
    unsigned OP_MODE           :3; // 2:0
  };
  struct {
    unsigned short w       :16;
  };
} __PHY1CTRLbits_t;  // Register 1Eh ? PHY Control 1
#define _PHY1CTRL_ENABLE_PAUSE_MASK           0x0200
#define _PHY1CTRL_LINK_STATUS_MASK           0x0100
#define _PHY1CTRL_POLARITY_STATUS_MASK           0x0080
#define _PHY1CTRL_MDI_MDIX_STATE_MASK           0x0020
#define _PHY1CTRL_ENERGY_DETECT_MASK           0x0010
#define _PHY1CTRL_PHY_ISOLATE_MASK           0x0008
#define _PHY1CTRL_OP_MODE_MASK           0x0003


// Register 1Fh ? PHY Control 2
//1F.15 HP_MDIX RW 1
//    1 = HP Auto MDI/MDI-X mode
//    0 = Micrel Auto MDI/MDI-X mode
//1F.14 MDI/MDI-X Select RW 0
//    When Auto MDI/MDI-X is disabled,
//    1 = MDI-X Mode Transmit on RXP,RXM and Receive on TXP,TXM
//    0 = MDI Mode Transmit on TXP,TXM and Receive on RXP,RXM
//1F.13 Pair Swap Disable RW Value determined by pin strapping option
//    1 = Disable auto MDI/MDI-X
//    0 = Enable auto MDI/MDI-X
//1F.12 Reserved RW 0
//1F.11 Force Link RW 0
//    1 = Force link pass
//    0 = Normal link operation
//    This bit bypasses the control logic and allow
//    transmitter to send pattern even if there is no link.
//1F.10 Power Saving RW 0
//    1 = Enable power saving
//    0 = Disable power saving
//1F.9 Interrupt Level RW 0
//    1 = Interrupt pin active high
//    0 = Interrupt pin active low
//1F.8 Enable Jabber RW 1
//    1 = Enable jabber counter
//    0 = Disable jabber counter
//1F.7:6 Reserved RW 00
//1F.5:4 LED Mode RW 00
//    [00] = LED1: Speed, LED0: Link / Activity
//    [01] = LED1: Activity, LED0: Link
//    [10] = reserved
//    [11] = reserved
//1F.3 Disable Transmitter RW 0
//    1 = Disable transmitter
//    0 = Enable transmitter
//1F.2 Remote Loopback RW 0
//    1 = Remote (analog) loopback is enabled
//    0 = Normal mode
//1F.1 Enable SQE Test RW 0
//    1 = Enable SQE test
//    0 = Disable SQE test
//1F.0 Disable Data Scrambling RW 0
//    1 = Disable scrambler
//    0 = Enable scrambler
typedef union {
  struct {
    unsigned HP_MDIX                    :1; // 15
    unsigned MDI_MDIX_SELECT            :1; // 14
    unsigned PAIR_SWAP_DISABLE          :1; // 13
    unsigned                            :1; // 12
    unsigned FORCE_LINK                 :1; // 11
    unsigned POWER_SAVING               :1; // 10
    unsigned INTERRUPT_LEVEL            :1; // 9
    unsigned ENABLE_JABBER              :1; // 8
    unsigned                            :2; // 7:6
    unsigned LED_MODE                   :2; // 5:4
    unsigned DISABLE_TX                 :1; // 3
    unsigned REMOTE_LOOPBACK            :1; // 2
    unsigned ENABLE_SQE_TEST            :1; // 1
    unsigned DISABLE_DATA_SCRAMBLING    :1; // 0
  };
  struct {
    unsigned short w       :16;
  };
} __PHY2CTRLbits_t;  // Register 1Fh ? PHY Control 2
#define _PHY2CTRL_HP_MDIX_MASK                  0x8000 // 15
#define _PHY2CTRL_MDI_MDIX_SELECT_MASK          0x4000 // 14
#define _PHY2CTRL_PAIR_SWAP_DISABLE_MASK        0x2000
#define _PHY2CTRL_FORCE_LINK_MASK               0x0800
#define _PHY2CTRL_POWER_SAVING_MASK             0x0400
#define _PHY2CTRL_INTERRUPT_LEVEL_MASK          0x0200
#define _PHY2CTRL_ENABLE_JABBER_MASK            0x0100
#define _PHY2CTRL_LED_MODE_MASK                 0x0030
#define _PHY2CTRL_DISABLE_TX_MASK               0x0008
#define _PHY2CTRL_REMOTE_LOOPBACK_MASK          0x0004
#define _PHY2CTRL_ENABLE_SQE_TEST_MASK          0x0002
#define _PHY2CTRL_DISABLE_DATA_SCRAMBLING_MASK  0x0001







#endif  // _KSZ_8061_RND_H_

