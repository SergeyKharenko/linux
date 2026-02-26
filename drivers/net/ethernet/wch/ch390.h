/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * CH390 Ethernet Registers Definitions
 *
 * This file defines the registers and associated constants for the
 * CH390 100Mbps Ethernet controller.
 *
 * Copyright (C) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Copyright (C) 2026 Sergey Kharenko
 * Author:       WCH <tech@wch.cn>
 * Contributor:  Sergey Kharenko <skharenko@hust.edu.cn>
 *
 * This file is part of the CH390 Ethernet driver. It provides symbolic
 * names for registers to facilitate driver development and integration
 * with the Linux networking stack.
 */

#ifndef CH390_H_
#define CH390_H_

#include <linux/bitops.h>

#define CH390_VID 0x1C00
#define CH390_PID 0x9151

/* Common register */
#define CH390_NCR 0x00
#define NCR_WAKEEN BIT(6) /* Enable wakeup function */
#define NCR_MACPD BIT(5) /* Enable wake-up frame notification */
#define NCR_FDX BIT(3) /* Duplex mode of the internal PHY */
#define NCR_LBK_MAC BIT(1) /* MAC loop-back */
#define NCR_RST BIT(0) /* Softwate reset */

#define CH390_NSR 0x01
#define NSR_SPEED BIT(7) /* Speed of internal PHY */
#define NSR_LINKST BIT(6) /* Link status of internal PHY */
#define NSR_WAKEST BIT(5) /* Wakeup event status */
#define NSR_TX2END BIT(3) /* Tx packet B complete status */
#define NSR_TX1END BIT(2) /* Tx packet A complete status */
#define NSR_RXOV BIT(1) /* Rx fifo overflow */
#define NSR_RXRDY BIT(0)

#define CH390_TCR 0x02
#define TCR_TJDIS BIT(6) /* Transmit jabber timer */
#define TCR_PAD_DIS2 BIT(4) /* PAD appends for packet B */
#define TCR_CRC_DIS2 BIT(3) /* CRC appends for packet B */
#define TCR_PAD_DIS1 BIT(2) /* PAD appends for packet A */
#define TCR_CRC_DIS1 BIT(1) /* CRC appends for packet A */
#define TCR_TXREQ BIT(0) /* Tx request */

#define CH390_TSRA 0x03

#define CH390_TSRB 0x04
#define TSR_TJTO BIT(7) /* Transmit jabber time out */
#define TSR_LC BIT(6) /* Loss of carrier */
#define TSR_NC BIT(5) /* No carrier */
#define TSR_LCOL BIT(4) /* Late collision */
#define TSR_COL BIT(3) /* Collision packet */
#define TSR_EC BIT(2) /* Excessive collision */

#define CH390_RCR 0x05
#define RCR_DEFAULT 0x00 /* Default settings */
#define RCR_WTDIS BIT(6) /* Disable 2048 bytes watch dog */
#define RCR_DIS_CRC BIT(4) /* Discard CRC error packet */
#define RCR_ALL BIT(3) /* Pass all multicast */
#define RCR_RUNT BIT(2) /* Pass runt packet */
#define RCR_PRMSC BIT(1) /* Promiscuous mode */
#define RCR_RXEN BIT(0) /* Enable RX */

#define CH390_RSR 0x06
#define RSR_RF BIT(7) /* Rnt frame */
#define RSR_MF BIT(6) /* Multicast frame */
#define RSR_LCS BIT(5) /* Late collision seen */
#define RSR_RWTO BIT(4) /* Receive watchdog time-out */
#define RSR_PLE BIT(3) /* Physical layer error */
#define RSR_AE BIT(2) /* Alignment error */
#define RSR_CE BIT(1) /* CRC error */
#define RSR_FOE BIT(0) /* FIFO overflow error */
#define RSR_ERR_BITS \
	(RSR_RF | RSR_LCS | RSR_RWTO | RSR_PLE | RSR_AE | RSR_CE | RSR_FOE)

#define CH390_ROCR 0x07

#define CH390_BPTR 0x08

#define CH390_FCTR 0x09
#define FCTR_HWOT(ot) (((ot) & 0xf) << 4)
#define FCTR_LWOT(ot) ((ot) & 0xf)

#define CH390_FCR 0x0A
#define FCR_TXP0 BIT(7) /* Force TX Pause Packetwith 0000H */
#define FCR_TXPF BIT(6) /* Force TX PausePacketwith FFFFH */
#define FCR_TXPEN BIT(5) /* TX Pause Packet Enable */
#define FCR_BKPA BIT(4) /* Back Pressure Mode */
#define FCR_BKPM BIT(3) /* Back Pressure Mode */
#define FCR_RXPS BIT(2) /* RX Pause Packet Status,Latch andReadClearly */
#define FCR_RXPCS BIT(1) /* RX Pause Packet Current Status */
#define FCR_FLCE BIT(0) /* Flow Control Enable */
#define FCR_RXTX_BITS (FCR_TXPEN | FCR_BKPM | FCR_FLCE)

#define CH390_EPCR 0x0B
#define EPCR_REEP BIT(5) /* Reload EEPROM */
#define EPCR_WEP BIT(4) /* Write EEPROM enable */
#define EPCR_EPOS BIT(3) /* EEPROM or PHY operation select */
#define EPCR_ERPRR BIT(2) /* EEPROM or PHY read command */
#define EPCR_ERPRW BIT(1) /* EEPROM or PHY write command */
#define EPCR_ERRE BIT(0) /* EEPROM or PHY access status */

#define CH390_EPAR 0x0C

#define CH390_EPDRL 0x0D
#define CH390_EPDRH 0x0E

#define CH390_WCR 0x0F
#define WCR_LINKEN BIT(5) /* Link status change wakeup */
#define WCR_SAMPLEEN BIT(4) /* Sample frame wakeup */
#define WCR_MAGICEN BIT(3) /* Magic packet wakeup */
#define WCR_LINKST BIT(2) /* Link status change event */
#define WCR_SAMPLEST BIT(1) /* Sample frame event */
#define WCR_MAGICST BIT(0) /* Magic packet event */

#define CH390_PAR 0x10

#define CH390_MAR 0x16

#define CH390_GPCR 0x1E

#define CH390_GPR 0x1F
#define GPR_PHYPD BIT(0) /* PHY power down */

#define CH390_TRPAL 0x22
#define CH390_TRPAH 0x23

#define CH390_RWPAL 0x24
#define CH390_RWPAH 0x25

#define CH390_VIDL 0x28
#define CH390_VIDH 0x29

#define CH390_PIDL 0x2A
#define CH390_PIDH 0x2B

#define CH390_CHIPR 0x2C

#define CH390_TCR2 0x2D

#define CH390_ATCR 0x30
#define ATCR_AUTO_TX BIT(7) /* Enabled auto transmit*/

#define CH390_TCSCR 0x31
#define TCSCR_ALL 0x1F
#define TCSCR_IPv6TCPCSE BIT(4) /* IPv6 TCP checksum generation */
#define TCSCR_IPv6UDPCSE BIT(3) /* IPv6 UDP checksum generation */
#define TCSCR_UDPCSE BIT(2) /* UDP checksum generation */
#define TCSCR_TCPCSE BIT(1) /* TCP checksum generation */
#define TCSCR_IPCSE BIT(0) /* IP checksum generation */

#define CH390_RCSCSR 0x32
#define RCSCSR_UDPS BIT(7) /* UDP checksum status */
#define RCSCSR_TCPS BIT(6) /* TCP checksum status */
#define RCSCSR_IPS BIT(5) /* IP checksum status */
#define RCSCSR_UDPP BIT(4) /* UDP packet of current received packet */
#define RCSCSR_TCPP BIT(3) /* TCP packet of current received packet */
#define RCSCSR_IPP BIT(2) /* IP packet of current received packet */
#define RCSCSR_RCSEN BIT(1) /* Receive checksum checking enable */
#define RCSCSR_DCSE BIT(0) /* Discard checksum error packet */

#define CH390_MPAR 0x33

#define CH390_SBCR 0x38

#define CH390_INTCR 0x39
#define INCR_TYPE_OD 0x02
#define INCR_TYPE_PP 0x00
#define INCR_POL_L 0x01
#define INCR_POL_H 0x00

#define CH390_ALNCR 0x4A

#define CH390_SCCR 0x50
#define SCCR_DIS_CLK 0x01

#define CH390_RSCCR 0x51

#define CH390_RLENCR 0x52
#define RLENCR_RXLEN BIT(7) /* Enable RX Length Filter */
#define RLENCR_MAXRXLEN_MASK 0x1F

#define CH390_BCASTCR 0x53

#define CH390_INTCKCR 0x54

#define CH390_MPTRCR 0x55
#define MPTRCR_RST_TX BIT(1) /* Reset TX Memory Pointer */
#define MPTRCR_RST_RX BIT(0) /* Reset RX Memory Pointer */

#define CH390_MLEDCR 0x57
#define MLEDCR_LED_MOD3 BIT(7)
#define MLEDCR_LED_TYPE1 BIT(1)
#define MLEDCR_LED_TYPE0 BIT(0)
#define MLEDCR_LED_MOD1 (MLEDCR_LED_MOD3 | MLEDCR_LED_TYPE0)

#define CH390_MRCMDX 0x70
#define CH390_MRCMDX1 0x71
#define CH390_MRCMD 0x72

#define CH390_MRRL 0x74
#define CH390_MRRH 0x75

#define CH390_MWCMDX 0x76
#define CH390_MWCMD 0x78

#define CH390_MWRL 0x7A
#define CH390_MWRH 0x7B

#define CH390_TXPLL 0x7C
#define CH390_TXPLH 0x7D

#define CH390_ISR 0x7E
#define ISR_LNKCHG BIT(5) /* Link status change */
#define ISR_ROO BIT(3) /* Receive overflow counter overflow */
#define ISR_ROS BIT(2) /* Receive overflow */
#define ISR_PT BIT(1) /* Packet transmitted */
#define ISR_PR BIT(0) /* Packet received */
#define ISR_CLR_INT (ISR_LNKCHG | ISR_ROO | ISR_ROS | ISR_PT | ISR_PR)
#define ISR_STOP_MRCMD (ISR_IOMODE)

#define CH390_IMR 0x7F
#define IMR_NONE 0x00 /* Disable all interrupt */
#define IMR_ALL 0xFF /* Enable all interrupt */
#define IMR_PAR BIT(7) /* Pointer auto-return mode */
#define IMR_LNKCHGI BIT(5) /* Enable link status change interrupt */
#define IMR_UDRUNI BIT(4) /* Enable transmit under-run interrupt */
#define IMR_ROOI BIT(3)
/* Enable receive overflow counter overflow interrupt */
#define IMR_ROI BIT(2) /* Enable receive overflow interrupt */
#define IMR_PTI BIT(1) /* Enable packet transmitted interrupt */
#define IMR_PRI BIT(0) /* Enable packet received interrupt */

/* SPI commands */
#define OPC_REG_W 0x80 /* Register Write */
#define OPC_REG_R 0x00 /* Register Read */
#define OPC_MEM_DMY_R 0x70 /* Memory Dummy Read */
#define OPC_MEM_WRITE 0xF8 /* Memory Write */
#define OPC_MEM_READ 0x72 /* Memory Read */
#define OPT_MEM_AUTO_TX 0xFC

/* GPIO */
#define CH390_GPIO1 0x02
#define CH390_GPIO2 0x04
#define CH390_GPIO3 0x08

/* PHY register */
#define CH390_PHY_ADDR 1
#define CH390_PHY 0x40
#define CH390_PHY_BMCR 0x00
#define CH390_PHY_BMSR 0x01
#define CH390_PHY_PHYID1 0x02
#define CH390_PHY_PHYID2 0x03
#define CH390_PHY_ANAR 0x04
#define CH390_PHY_ANLPAR 0x05
#define CH390_PHY_ANER 0x06
#define CH390_PHY_PAGE_SEL 0x1F
#define CH390_TX_QUE_HI_WATER 50
#define CH390_TX_QUE_LO_WATER 25
#define CH390_EEPROM_MAGIC 0x9151

/* Packet status */
#define CH390_PKT_NONE 0x00 /* No packet received */
#define CH390_PKT_RDY 0x01 /* Packet ready to receive */
#define CH390_PKT_ERR 0xFE /* Un-stable states mask */
#define CH390_PKT_ERR_WITH_RCSEN 0xE2 /* Un-stable states mask when RCSEN = 1 */
#define CH390_PKT_MAX 1536 /* Received packet max size */
#define CH390_PKT_MIN 64

#endif /* CH390_H_ */
