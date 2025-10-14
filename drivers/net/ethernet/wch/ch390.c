// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * CH390H/D SPI Ethernet driver
 *
 * Driver for the CH390H/D SPI to 100Mbps Ethernet controller.
 *
 * Copyright (C) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Author:       WCH <tech@wch.cn>
 * Maintainer:   Sergey Kharenko <skharenko@hust.edu.cn>
 *
 * This driver provides support for the CH390H/D Ethernet controller
 * connected via SPI. It integrates with the Linux networking stack
 * through the standard net_device interface.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/crc32.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/version.h>

#include "ch390.h"

#define DRVNAME_CH390 "ch390"

#define CH390_PRINT_ERROR(format, ...) \
	netif_err(db, drv, db->ndev, "%s: " format, __func__)

#define CH390_PRINT_INFO(format, ...) \
	netif_info(db, drv, db->ndev, "%s: " format, __func__)

#define CH390_GOTO_ON_ERROR(x, goto_tag, format, ...)                          \
	do {                                                                   \
		ret = (x);                                                     \
		if (ret < 0) {                                                 \
			netif_err(db, drv, db->ndev, "%s: " format, __func__); \
			goto goto_tag;                                         \
		}                                                              \
	} while (0)

#define CH390_RETURN_ON_ERROR(x, format, ...)                                  \
	do {                                                                   \
		ret = (x);                                                     \
		if (ret < 0) {                                                 \
			netif_err(db, drv, db->ndev, "%s: " format, __func__); \
			return ret;                                            \
		}                                                              \
	} while (0)

/**
 * struct ch390_rxhdr - rx packet data header
 * @headbyte: lead byte equal to 0x01 notifies a valid packet
 * @status: status bits for the received packet
 * @rxlen: packet length
 *
 * The Rx packed, entered into the FIFO memory, start with these
 * four bytes which is the Rx header, followed by the ethernet
 * packet data and ends with an appended 4-byte CRC data.
 * Both Rx packet and CRC data are for check purpose and finally
 * are dropped by this driver
 */
struct ch390_rxhdr {
	u8 headbyte;
	u8 status;
	__le16 rxlen;
};

/**
 * struct board_info - Private driver data for CH390 network device
 *
 * @spidev:               Pointer to the underlying SPI device.
 * @ndev:                 Pointer to the associated network device.
 * @mdiobus:              Pointer to the MDIO bus used for PHY communication.
 * @phydev:               Pointer to the PHY device structure.
 *
 * @msg_enable:           Debug message level flags.
 *
 * @txq:                  Queue of outgoing packets (struct sk_buff) waiting for transmission.
 * @async_tx_work:        Work item for asynchronous packet transmission.
 * @async_rx_mode_work:   Work item for applying receiver mode changes asynchronously.
 *
 * @pause:                Current Ethernet flow control (pause frame) settings.
 * @wolinfo:              WOL (Wake-on-LAN) capability and configuration info.
 * @wol_gpiod:            GPIO descriptor for the Wake-on-LAN (WOL) signal.
 * @wol_irq:              IRQ number associated with the WOL GPIO.
 *
 * @dev_lock:             Mutex protecting general device operations.
 * @spi_lock:             Mutex protecting SPI register access and bus transactions.
 *
 * @stats:                Network statistics (64-bit counters).
 * @syncp:                Synchronization primitive for safely updating 64-bit stats.
 *
 * @hash_table:           Local cache of the 64-bit multicast hash table.
 * @rcr:                  Cached value of the Receive Control Register (RCR).
 *
 * @has_eeprom:           True if the board has an EEPROM attached.
 * @irq_high:             True if the device uses a high-level triggered IRQ.
 * @rx_csum:              True if RX checksum offloading is enabled.
 */
struct board_info {
	struct spi_device *spidev;
	struct net_device *ndev;
	struct mii_bus *mdiobus;
	struct phy_device *phydev;

	u32 msg_enable;

	struct sk_buff_head txq;
	struct work_struct async_tx_work;
	struct work_struct async_rx_mode_work;

	struct ethtool_pauseparam pause;
	struct ethtool_wolinfo wolinfo;
	struct gpio_desc *wol_gpiod;
	int wol_irq;

	struct mutex dev_lock;
	struct mutex spi_lock;

	struct rtnl_link_stats64 stats;
	struct u64_stats_sync syncp;

	u8 hash_table[8];
	u8 rcr;

	bool has_eeprom;
	bool irq_high;
	bool rx_csum;
};

static inline struct board_info *to_ch390_board(struct net_device *ndev)
{
	return netdev_priv(ndev);
}

/**
 * ch390_io_register_write - Write a byte to a CH390 register.
 * @db:  Pointer to the driver's private data structure.
 * @reg: The address of the register to write.
 * @val: The value to write to the register.
 *
 * This is a low-level function for writing to an internal register of the
 * CH390 via the SPI interface. Access to the register is protected by a mutex.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static inline int ch390_io_register_write(struct board_info *db, u8 reg, u8 val)
{
	int ret;

	reg |= OPC_REG_W;
	struct spi_transfer trans[] = { { .tx_buf = &reg, .len = 1 },
					{ .tx_buf = &val, .len = 1 } };

	mutex_lock(&db->spi_lock);
	ret = spi_sync_transfer(db->spidev, trans, 2);
	mutex_unlock(&db->spi_lock);
	return ret;
}

/**
 * ch390_io_register_read - Read a byte from a CH390 register.
 * @db:  Pointer to the driver's private data structure.
 * @reg: The address of the register to read.
 * @val: Pointer to a u8 to store the read value.
 *
 * This is a low-level function for reading from an internal register of the
 * CH390 via the SPI interface. Access to the register is protected by a mutex.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static inline int ch390_io_register_read(struct board_info *db, u8 reg,
					 void *val)
{
	int ret;

	reg |= OPC_REG_R;
	struct spi_transfer trans[] = { { .tx_buf = &reg, .len = 1 },
					{ .rx_buf = val, .len = 1 } };

	mutex_lock(&db->spi_lock);
	ret = spi_sync_transfer(db->spidev, trans, 2);
	mutex_unlock(&db->spi_lock);
	return ret;
}

/**
 * ch390_io_memory_write - Write a block of data to CH390's internal memory.
 * @db:   Pointer to the driver's private data structure.
 * @buff: Pointer to the data buffer to write.
 * @len:  The number of bytes to write.
 *
 * Writes data to the CH390's internal RAM, typically for a transmit packet.
 * Access is protected by a mutex.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static inline int ch390_io_memory_write(struct board_info *db, const void *buff,
					size_t len)
{
	int ret;
	u8 reg = OPC_MEM_WRITE;
	struct spi_transfer trans[] = { { .tx_buf = &reg, .len = 1 },
					{ .tx_buf = buff, .len = len } };

	mutex_lock(&db->spi_lock);
	ret = spi_sync_transfer(db->spidev, trans, 2);
	mutex_unlock(&db->spi_lock);
	return ret;
}

/**
 * ch390_io_memory_read - Read a block of data from CH390's internal memory.
 * @db:   Pointer to the driver's private data structure.
 * @buff: Pointer to a buffer to store the read data.
 * @len:  The number of bytes to read.
 *
 * Reads data from the CH390's internal RAM, typically for a received packet.
 * Access is protected by a mutex.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static inline int ch390_io_memory_read(struct board_info *db, void *buff,
				       size_t len)
{
	int ret;
	u8 reg = OPC_MEM_READ;
	struct spi_transfer trans[] = { { .tx_buf = &reg, .len = 1 },
					{ .rx_buf = buff, .len = len } };

	mutex_lock(&db->spi_lock);
	ret = spi_sync_transfer(db->spidev, trans, 2);
	mutex_unlock(&db->spi_lock);
	return ret;
}

/**
 * ch390_epcr_busy_wait - Wait for an EEPROM or PHY operation to complete.
 * @db: Pointer to the driver's private data structure.
 *
 * Polls the EPCR_ERRE bit in the EPCR register to wait for the completion of an
 * EEPROM or PHY access operation.
 *
 * Return: 0 on success, or -ETIMEDOUT on timeout.
 */
static inline int ch390_epcr_busy_wait(struct board_info *db)
{
	int ret;
	u8 cnt = 0;
	u8 epcr;

	while (cnt < 100) {
		CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_EPCR,
							     &epcr),
				      "read EPCR failed");
		if (!(epcr & EPCR_ERRE))
			return 0;
		usleep_range(50, 100);
		cnt++;
	}
	netdev_err(db->ndev, "eeprom/phy in processing get timeout");
	return -ETIMEDOUT;
}

/**
 * ch390_eeprom_read - Read a 16-bit word from the EEPROM.
 * @db:     Pointer to the driver's private data structure.
 * @offset: The word offset (not byte) in the EEPROM to read from.
 * @data:   Pointer to a u16 to store the read data.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_eeprom_read(struct board_info *db, int offset, u16 *data)
{
	int ret;
	u8 epdrl, epdrh;

	mutex_lock(&db->dev_lock);

	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_EPAR, offset),
			      "write EPAR failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_EPCR,
						      EPCR_ERPRR),
			      "write EPCR failed");

	CH390_RETURN_ON_ERROR(ch390_epcr_busy_wait(db), "read eeprom failed");

	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_EPCR, 0),
			      "write EPCR failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_EPDRL, &epdrl),
			      "read EPDRL failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_EPDRH, &epdrh),
			      "read EPDRH failed");

	mutex_unlock(&db->dev_lock);

	*data = le16_to_cpu(epdrh << 8 | epdrl);
	return ret;
}

/**
 * ch390_eeprom_write - Write a 16-bit word to the EEPROM.
 * @db:     Pointer to the driver's private data structure.
 * @offset: The word offset (not byte) in the EEPROM to write to.
 * @data:   The 16-bit data to write.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_eeprom_write(struct board_info *db, int offset, u16 data)
{
	int ret;

	data = cpu_to_le16(data);

	mutex_lock(&db->dev_lock);

	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_EPAR, offset),
			      "write EPAR failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_EPDRL,
						      data & 0xFF),
			      "write EPDRL failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_EPDRH,
						      data >> 8),
			      "write EPDRH failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_EPCR,
						      EPCR_WEP | EPCR_ERPRW),
			      "write EPCR failed");

	CH390_RETURN_ON_ERROR(ch390_epcr_busy_wait(db), "write eeprom failed");

	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_EPCR, 0),
			      "write EPCR failed");

	mutex_unlock(&db->dev_lock);
	return ret;
}

/**
 * ch390_phyread - Read a register from the internal PHY.
 * @context: Pointer to the driver's private data structure.
 * @reg:     The PHY register address to read.
 * @data:    Pointer to a u16 to store the read value.
 *
 * This function is used as a backend for the MDIO read operation.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_phyread(void *context, u8 reg, u16 *data)
{
	struct board_info *db = context;
	int ret;
	u8 epdrl, epdrh;

	mutex_lock(&db->dev_lock);

	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_EPAR,
						      CH390_PHY | reg),
			      "write EPAR failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_EPCR,
						      EPCR_ERPRR | EPCR_EPOS),
			      "write EPCR failed");

	CH390_RETURN_ON_ERROR(ch390_epcr_busy_wait(db), "read phy failed");

	*data = 0;

	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_EPCR, 0),
			      "write EPCR failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_EPDRL, &epdrl),
			      "read EPDRL failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_EPDRH, &epdrh),
			      "read EPDRH failed");

	*data = le16_to_cpu(epdrh << 8 | epdrl);

	mutex_unlock(&db->dev_lock);

	return ret;
}

/**
 * ch390_phywrite - Write a register to the internal PHY.
 * @context: Pointer to the driver's private data structure.
 * @reg:     The PHY register address to write.
 * @data:    The 16-bit value to write.
 *
 * This function is used as a backend for the MDIO write operation.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_phywrite(void *context, u8 reg, u16 data)
{
	struct board_info *db = context;
	int ret;

	data = cpu_to_le16(data);

	mutex_lock(&db->dev_lock);

	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_EPAR,
						      CH390_PHY | reg),
			      "write EPAR failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_EPDRL,
						      data & 0xFF),
			      "write EPDRL failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_EPDRH,
						      data >> 8),
			      "write EPDRH failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_EPCR,
						      EPCR_EPOS | EPCR_ERPRW),
			      "write EPCR failed");

	CH390_RETURN_ON_ERROR(ch390_epcr_busy_wait(db), "write phy failed");

	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_EPCR, 0),
			      "write EPCR failed");

	mutex_unlock(&db->dev_lock);
	return ret;
}

/**
 * ch390_mdio_read - MDIO bus read callback.
 * @bus:    Pointer to the mii_bus structure.
 * @addr:   The PHY address on the MDIO bus.
 * @regnum: The register offset to read.
 *
 * This function is registered with the MDIO subsystem to handle read requests.
 * It only responds to reads for this driver's internal PHY.
 *
 * Return: The 16-bit value read from the PHY register, or 0xFFFF on error.
 */
static int ch390_mdio_read(struct mii_bus *bus, int addr, int regnum)
{
	struct board_info *db = bus->priv;
	int ret;
	u16 val = 0xFFFF;

	if (addr == CH390_PHY_ADDR)
		CH390_RETURN_ON_ERROR(ch390_phyread(db, regnum, &val),
				      "read phy failed");
	return (int)val;
}

/**
 * ch390_mdio_write - MDIO bus write callback.
 * @bus:    Pointer to the mii_bus structure.
 * @addr:   The PHY address on the MDIO bus.
 * @regnum: The register offset to write.
 * @val:    The 16-bit value to write.
 *
 * This function is registered with the MDIO subsystem to handle write requests.
 * It only responds to writes for this driver's internal PHY.
 *
 * Return: 0 on success, or -ENODEV if the PHY address is incorrect.
 */
static int ch390_mdio_write(struct mii_bus *bus, int addr, int regnum, u16 val)
{
	struct board_info *db = bus->priv;
	int ret;

	if (addr == CH390_PHY_ADDR)
		CH390_RETURN_ON_ERROR(ch390_phywrite(db, regnum, val),
				      "write phy failed");

	return -ENODEV;
}

/**
 * ch390_drop_frame - Discard the current received frame from RX buffer.
 * @db:  Pointer to the driver's private data structure.
 * @len: Length of the frame to discard.
 *
 * Advances the RX memory read pointer past the current frame, effectively
 * dropping it. Used when a received frame has errors or cannot be processed.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_drop_frame(struct board_info *db, size_t len)
{
	int ret;
	u8 mrrh, mrrl;

	CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_MRRH, &mrrh),
			      "read MRRH failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_MRRL, &mrrl),
			      "read MRRL failed");

	u16 addr = mrrh << 8 | mrrl;

	addr = le16_to_cpu(addr);
	addr += len;
	addr = addr < 0x4000 ? addr : addr - 0x3400;
	addr = cpu_to_le16(addr);

	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_MRRH,
						      addr >> 8),
			      "write MRRH failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_MRRL,
						      addr & 0xFF),
			      "write MRRL failed");
	return 0;
}

/**
 * ch390_update_fcr - Update the Flow Control Register.
 * @db: Pointer to the driver's private data structure.
 *
 * Configures the hardware flow control settings in the FCR register based on
 * the cached values in `db->pause`.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_update_fcr(struct board_info *db)
{
	int ret;
	u8 fcr;

	CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_FCR, &fcr),
			      "read FCR failed");
	if (db->pause.rx_pause)
		fcr |= FCR_BKPM | FCR_FLCE;
	else
		fcr &= ~(FCR_BKPM | FCR_FLCE);

	if (db->pause.tx_pause)
		fcr |= FCR_TXPEN;
	else
		fcr &= ~FCR_TXPEN;
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_FCR, fcr),
			      "write FCR failed");
	return 0;
}

/**
 * ch390_verify_id - Verify the Chip's Vendor and Product ID.
 * @db: Pointer to the driver's private data structure.
 *
 * Reads the VID and PID registers to ensure that the correct chip is present.
 *
 * Return: 0 if IDs match, or -ENODEV if they do not.
 */
static int ch390_verify_id(struct board_info *db)
{
	struct device *dev = &db->spidev->dev;
	int ret;
	u8 id[2];
	u8 chipr;
	u16 pid, vid;

	CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_VIDL, &id[0]),
			      "read VIDL failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_VIDH, &id[1]),
			      "read VIDH failed");
	vid = le16_to_cpu(id[1] << 8 | id[0]);
	if (vid != CH390_VID) {
		dev_err(dev, "dev vid error as %04x !", vid);
		return -ENODEV;
	}

	CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_PIDL, &id[0]),
			      "read PIDL failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_PIDH, &id[1]),
			      "read PIDH failed");
	pid = le16_to_cpu(id[1] << 8 | id[0]);
	if (pid != CH390_PID) {
		dev_err(dev, "dev pid error as %04x !", pid);
		return -ENODEV;
	}

	CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_CHIPR, &chipr),
			      "read CHIPR failed");
	dev_info(dev, "chip %02x found", chipr);
	return 0;
}

/**
 * ch390_init_mac_addr - Initialize the device MAC address.
 * @ndev: Pointer to the network device structure.
 * @db:   Pointer to the driver's private data structure.
 *
 * Reads the MAC address from the hardware's PAR registers. If the address is
 * invalid, a random MAC address is generated and written back to the hardware.
 * The MAC address is then set for the net device.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_init_mac_addr(struct net_device *ndev, struct board_info *db)
{
	u8 addr[ETH_ALEN];
	int ret;

	for (int i = 0; i < ETH_ALEN; i++) {
		CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_PAR + i,
							     addr + i),
				      "read PAR failed");
	}

	if (!is_valid_ether_addr(addr)) {
		eth_hw_addr_random(ndev);

		for (int i = 0; i < ETH_ALEN; i++) {
			CH390_RETURN_ON_ERROR(
				ch390_io_register_write(db, CH390_PAR + i,
							(ndev->dev_addr)[i]),
				"write PAR failed");
		}
		dev_dbg(&db->spidev->dev, "Use random MAC address");
	} else {
		eth_hw_addr_set(ndev, addr);
	}
	return 0;
}

/**
 * ch390_init_hw_offload - Initialize hardware checksum offload
 * @db: [in] Pointer to board_info structure representing the device
 *
 * This function enables hardware checksum offload for both
 * transmit and receive paths. It writes the relevant control
 * registers:
 *   - TCSCR: transmit checksum control register, enable all TX checksum
 *   - RCSCSR: receive checksum and data checksum enable
 *
 * Returns: 0 on success, negative error code on failure
 */
static int ch390_init_hw_offload(struct board_info *db)
{
	int ret;

	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_TCSCR,
						      TCSCR_ALL),
			      "write TCSCR failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_RCSCSR,
						      RCSCSR_RCSEN |
							      RCSCSR_DCSE),
			      "write RCSCSR failed");
	return 0;
}

/**
 * ch390_init_hw_rxlen_filter - Initialize hardware RX length filter
 * @db: [in] Pointer to board_info structure representing the device
 *
 * This function configures the hardware to automatically drop
 * received frames longer than CH390_PKT_MAX. It sets the RLENCR
 * register:
 *   - RXLEN: enable RX length filter
 *   - MAXRXLEN: maximum allowed frame length (CH390_PKT_MAX)
 *
 * Returns: 0 on success, negative error code on failure
 */
static int ch390_init_hw_rxlen_filter(struct board_info *db)
{
	int ret;
	u8 rlencr = 0;

	rlencr |= RLENCR_RXLEN;
	rlencr |= ((CH390_PKT_MAX >> 6) & RLENCR_MAXRXLEN_MASK);
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_RLENCR, rlencr),
			      "write RLENCR failed");
	return 0;
}

/*
 * Ethtool operations
 */

/**
 * ch390_get_drvinfo - Implementation of ethtool get_drvinfo.
 * @ndev: Pointer to the network device structure.
 * @info: Pointer to the ethtool_drvinfo structure to be filled.
 */
static void ch390_get_drvinfo(struct net_device *ndev,
			      struct ethtool_drvinfo *info)
{
	strscpy(info->driver, DRVNAME_CH390, sizeof(info->driver));
}

/**
 * ch390_set_msglevel - Implementation of ethtool set_msglevel.
 * @ndev:  Pointer to the network device structure.
 * @value: The new message level.
 */
static void ch390_set_msglevel(struct net_device *ndev, u32 value)
{
	struct board_info *db = to_ch390_board(ndev);

	db->msg_enable = value;
}

/**
 * ch390_get_msglevel - Implementation of ethtool get_msglevel.
 * @ndev: Pointer to the network device structure.
 *
 * Return: The current message level.
 */
static u32 ch390_get_msglevel(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);

	return db->msg_enable;
}

/**
 * ch390_get_eeprom_len - Implementation of ethtool get_eeprom_len.
 * @ndev: Pointer to the network device structure.
 *
 * Return: The EEPROM size (128 bytes) if present, otherwise 0.
 */
static int ch390_get_eeprom_len(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);

	if (db->has_eeprom)
		return 128;
	else
		return 0;
}

/**
 * ch390_get_eeprom - Implementation of ethtool get_eeprom.
 * @ndev: Pointer to the network device structure.
 * @ee:   Pointer to the ethtool_eeprom structure with read parameters.
 * @data: Buffer to store the read EEPROM data.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_get_eeprom(struct net_device *ndev, struct ethtool_eeprom *ee,
			    u8 *data)
{
	struct board_info *db = to_ch390_board(ndev);

	if (!db->has_eeprom)
		return -ENXIO;

	int offset = ee->offset;
	int len = ee->len;
	int ret;

	if ((len | offset) & 1)
		return -EINVAL;

	ee->magic = CH390_EEPROM_MAGIC;

	while (len > 0) {
		CH390_RETURN_ON_ERROR(ch390_eeprom_read(db, offset / 2,
							(u16 *)data),
				      "read eeprom failed");
		data += 2;
		offset += 2;
		len -= 2;
	}
	return 0;
}

/**
 * ch390_set_eeprom - Implementation of ethtool set_eeprom.
 * @ndev: Pointer to the network device structure.
 * @ee:   Pointer to the ethtool_eeprom structure with write parameters.
 * @data: Buffer containing the data to write to the EEPROM.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_set_eeprom(struct net_device *ndev, struct ethtool_eeprom *ee,
			    u8 *data)
{
	struct board_info *db = to_ch390_board(ndev);

	if (!db->has_eeprom)
		return -ENXIO;

	int offset = ee->offset;
	int len = ee->len;
	int ret;

	if ((len | offset) & 1)
		return -EINVAL;

	if (ee->magic != CH390_EEPROM_MAGIC)
		return -EINVAL;

	while (len > 0) {
		CH390_RETURN_ON_ERROR(ch390_eeprom_write(db, offset / 2,
							 *(u16 *)data),
				      "write eeprom failed");
		data += 2;
		offset += 2;
		len -= 2;
		if (len == 1) {
			CH390_RETURN_ON_ERROR(ch390_eeprom_write(db, offset / 2,
								 (u16)(*data)),
					      "write eeprom failed");
			break;
		}
	}
	return 0;
}

/**
 * ch390_get_pauseparam - Implementation of ethtool get_pauseparam.
 * @ndev:  Pointer to the network device structure.
 * @pause: Pointer to the ethtool_pauseparam structure to be filled.
 */
static void ch390_get_pauseparam(struct net_device *ndev,
				 struct ethtool_pauseparam *pause)
{
	struct board_info *db = to_ch390_board(ndev);

	*pause = db->pause;
}

/**
 * ch390_set_pauseparam - Implementation of ethtool set_pauseparam.
 * @ndev:  Pointer to the network device structure.
 * @pause: Pointer to the ethtool_pauseparam structure with new settings.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_set_pauseparam(struct net_device *ndev,
				struct ethtool_pauseparam *pause)
{
	struct board_info *db = to_ch390_board(ndev);

	db->pause = *pause;

	if (pause->autoneg == AUTONEG_DISABLE)
		return ch390_update_fcr(db);

	phy_set_sym_pause(db->phydev, pause->rx_pause, pause->tx_pause,
			  pause->autoneg);
	phy_start_aneg(db->phydev);

	return 0;
}

/**
 * ch390_get_wol - Get current Wake-on-LAN (WOL) settings.
 * @ndev:  Pointer to the network device.
 * @info:  Pointer to ethtool_wolinfo structure to be filled.
 */
static void ch390_get_wol(struct net_device *ndev, struct ethtool_wolinfo *info)
{
	struct board_info *db = to_ch390_board(ndev);

	info->wolopts = db->wolinfo.wolopts;
	info->supported = db->wolinfo.supported;
}

/**
 * ch390_set_wol - Configure Wake-on-LAN (WOL) options.
 * @ndev:  Pointer to the network device.
 * @info:  Pointer to ethtool_wolinfo structure containing requested WOL options.
 *
 * Returns 0 on success or a negative error code if any SPI register
 * access fails.
 */
static int ch390_set_wol(struct net_device *ndev, struct ethtool_wolinfo *info)
{
	struct board_info *db = to_ch390_board(ndev);
	int ret;
	u8 wcr = 0;
	u8 ncr;

	if (info->wolopts == db->wolinfo.wolopts)
		return 0;
	if (info->wolopts & WAKE_PHY)
		wcr |= WCR_LINKEN;
	if (info->wolopts & WAKE_MAGIC)
		wcr |= WCR_MAGICEN;
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_WCR, wcr),
			      "write WCR failed");

	CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_NCR, &ncr),
			      "read NCR failed");
	if (info->wolopts)
		ncr |= NCR_WAKEEN;
	else
		ncr &= ~NCR_WAKEEN;
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_NCR, ncr),
			      "write NCR failed");

	db->wolinfo.wolopts = info->wolopts;
	return 0;
}

static const struct ethtool_ops ch390_ethtool_ops = {
	.get_drvinfo = ch390_get_drvinfo,
	.get_link_ksettings = phy_ethtool_get_link_ksettings,
	.set_link_ksettings = phy_ethtool_set_link_ksettings,
	.get_msglevel = ch390_get_msglevel,
	.set_msglevel = ch390_set_msglevel,
	.nway_reset = phy_ethtool_nway_reset,
	.get_link = ethtool_op_get_link,
	.get_eeprom_len = ch390_get_eeprom_len,
	.get_eeprom = ch390_get_eeprom,
	.set_eeprom = ch390_set_eeprom,
	.get_pauseparam = ch390_get_pauseparam,
	.set_pauseparam = ch390_set_pauseparam,
	.get_wol = ch390_get_wol,
	.set_wol = ch390_set_wol
};

/**
 * ch390_reset - Perform a software reset of the CH390 chip.
 * @db: Pointer to the driver's private data structure.
 *
 * This function initiates a software reset and waits for it to complete.
 *
 * Return: 0 on success, or -ETIMEDOUT on timeout.
 */
static int ch390_reset(struct board_info *db)
{
	int ret;
	u8 cnt = 0;
	u8 ncr;

	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_NCR, NCR_RST),
			      "write NCR failed"); /* NCR reset */

	while (cnt < 100) {
		CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_NCR,
							     &ncr),
				      "read NCR failed");
		if (!(ncr & NCR_RST))
			return 0;
		cnt++;
		usleep_range(500, 1000);
	}
	return -ETIMEDOUT;
}

/**
 * ch390_start - Initialize and enable the CH390 chip for operation.
 * @db: Pointer to the driver's private data structure.
 *
 * This function is called from ndo_open. It configures the essential
 * registers, enables the receiver, and sets up interrupts.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_start(struct board_info *db)
{
	int ret;
	u8 rcr;

	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_GPR, 0x00),
			      "write GPR failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_MLEDCR,
						      MLEDCR_LED_MOD1),
			      "write MLEDCR failed");
	usleep_range(1000, 2000);

	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_MPTRCR,
						      MPTRCR_RST_RX),
			      "write MPTRCR failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_ISR,
						      ISR_CLR_INT),
			      "write ISR failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_IMR,
						      IMR_LNKCHGI | IMR_PAR |
							      IMR_PRI),
			      "write IMR failed");

	CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_RCR, &rcr),
			      "read RCR failed");
	rcr |= RCR_RXEN;
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_RCR, rcr),
			      "write RCR failed");
	return 0;
}

/**
 * ch390_stop - Disable the CH390 chip.
 * @db: Pointer to the driver's private data structure.
 *
 * This function is called from ndo_stop. It disables interrupts, powers down
 * the PHY, and disables the receiver.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_stop(struct board_info *db)
{
	int ret;
	u8 rcr;

	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_IMR, IMR_PAR),
			      "write IMR failed");
	/*
	 * GPR power off of the internal phy,
	 * the internal phy still could be accessed after this GPR power off control
	 */
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_GPR, GPR_PHYPD),
			      "power off phy failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_RCR, &rcr),
			      "read RCR failed");
	rcr &= ~RCR_RXEN;
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_RCR, rcr),
			      "write RCR failed");
	return 0;
}

/**
 * ch390_transmit - Transmit a single network packet.
 * @db:   Pointer to the driver's private data structure.
 * @buff: Pointer to the packet data.
 * @len:  Length of the packet data.
 *
 * Writes the packet data to the CH390's transmit buffer, sets the length
 * registers, and issues the transmit command.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_transmit(struct board_info *db, u8 *buff, unsigned int len)
{
	int ret;

	len = cpu_to_le32(len);
	unsigned int temp_low = len & 0xff;
	unsigned int temp_high = (len >> 8) & 0xff;
	u8 val, temp;
	u8 timeout = 0;

	CH390_RETURN_ON_ERROR(ch390_io_memory_write(db, buff, len),
			      "write tx mem failed");

	do {
		CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_TCR,
							     &temp),
				      "read TCR failed");
		timeout++;
		if (timeout == 100) {
			CH390_PRINT_ERROR("wait tx timeout");
			return -EBUSY;
		}
	} while (temp & TCR_TXREQ);

	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_TXPLL,
						      temp_low),
			      "write TXPLL failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_TXPLH,
						      temp_high),
			      "write TXPLH failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_TCR, &val),
			      "read TCR failed");
	CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_TCR,
						      val | TCR_TXREQ),
			      "write TCR failed");
	return 0;
}

/**
 * ch390_receive - Receive a single network packet.
 * @db:  Pointer to the driver's private data structure.
 * @skb: Pointer to an sk_buff pointer.
 *
 * Checks if a packet is ready. If so, it reads the header, checks for errors,
 * allocates an sk_buff, and reads the packet data into it.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static struct sk_buff *ch390_receive(struct board_info *db)
{
	int ret;
	u16 len;
	u8 ready;
	struct sk_buff *skb = NULL;

	CH390_GOTO_ON_ERROR(ch390_io_register_read(db, CH390_MRCMDX, &ready),
			    err, "read MRCMDX failed");
	CH390_GOTO_ON_ERROR(ch390_io_register_read(db, CH390_MRCMDX, &ready),
			    err, "read MRCMDX failed");

	if ((!db->rx_csum && (ready & CH390_PKT_ERR)) ||
	    (db->rx_csum && (ready & CH390_PKT_ERR_WITH_RCSEN))) {
		CH390_GOTO_ON_ERROR(ch390_io_register_write(db, CH390_RCR, 0),
				    err, "write RCR failed");
		CH390_GOTO_ON_ERROR(ch390_io_register_write(db, CH390_MPTRCR,
							    MPTRCR_RST_RX),
				    err, "write MPTRCR failed");
		CH390_GOTO_ON_ERROR(ch390_io_register_write(db, CH390_MRRH,
							    0x0C),
				    err, "write MRRH failed");
		usleep_range(1000, 2000);
		CH390_GOTO_ON_ERROR(ch390_io_register_write(db, CH390_RCR,
							    RCR_RXEN),
				    err, "write RCR failed");
		return NULL;
	}

	struct ch390_rxhdr rx_header;

	if (ready & CH390_PKT_RDY) {
		CH390_GOTO_ON_ERROR(ch390_io_memory_read(db, (u8 *)&rx_header,
							 sizeof(rx_header)),
				    err, "peek rx header failed");
		len = le16_to_cpu(rx_header.rxlen);
		if (rx_header.status & RSR_ERR_BITS) {
			u64_stats_update_begin(&db->syncp);
			db->stats.rx_dropped++;
			db->stats.rx_errors++;
			u64_stats_update_end(&db->syncp);
			ch390_drop_frame(db, len);
			return NULL;
		}

		skb = dev_alloc_skb(len);
		if (!skb) {
			u64_stats_update_begin(&db->syncp);
			db->stats.rx_dropped++;
			db->stats.rx_errors++;
			u64_stats_update_end(&db->syncp);
			ch390_drop_frame(db, len);
			return NULL;
		}

		void *ptr = skb_put(skb, len - ETH_FCS_LEN);

		CH390_GOTO_ON_ERROR(ch390_io_memory_read(db, ptr, len), err_rcd,
				    "read rx data failed");
		u64_stats_update_begin(&db->syncp);
		db->stats.rx_packets++;
		db->stats.rx_bytes += len;
		u64_stats_update_end(&db->syncp);
	}

	return skb;

err_rcd:
	u64_stats_update_begin(&db->syncp);
	db->stats.rx_errors++;
	u64_stats_update_end(&db->syncp);
	dev_kfree_skb(skb);
err:
	return NULL;
}

/**
 * ch390_irq_handler - The interrupt handler (top half).
 * @irq: The interrupt number.
 * @pw:  Pointer to the driver's private data structure.
 *
 * This function is the primary interrupt handler. It simply schedules the
 * receive work queue to process the interrupt in a bottom-half context.
 *
 * Return: IRQ_HANDLED.
 */
static irqreturn_t ch390_irq_handler(int irq, void *pw)
{
	struct board_info *db = pw;
	int ret;
	u8 status;
	struct sk_buff *skb = NULL;

	CH390_GOTO_ON_ERROR(ch390_io_register_read(db, CH390_ISR, &status), err,
			    "read ISR failed");
	CH390_GOTO_ON_ERROR(ch390_io_register_write(db, CH390_ISR, status), err,
			    "write ISR failed");

	if (status & ISR_LNKCHG)
		phy_mac_interrupt(db->phydev);

	if (status & ISR_PR) {
		while (1) {
			mutex_lock(&db->dev_lock);
			skb = ch390_receive(db);
			mutex_unlock(&db->dev_lock);
			if (!skb)
				break;

			skb->protocol = eth_type_trans(skb, db->ndev);
			if (db->ndev->features & NETIF_F_RXCSUM)
				skb_checksum_none_assert(skb);
			netif_rx(skb);
		}
	}

err:
	return IRQ_HANDLED;
}

/**
 * ch390_async_transmit - Asynchronous transmit work function (bottom half).
 * @work: Pointer to the work_struct.
 *
 * Dequeues packets from the transmit queue (`txq`) and sends them to the
 * hardware. This function runs in a workqueue context.
 */
static void ch390_async_transmit(struct work_struct *work)
{
	struct board_info *db =
		container_of(work, struct board_info, async_tx_work);
	struct net_device *ndev = db->ndev;
	int ret;

	mutex_lock(&db->dev_lock);
	while (!skb_queue_empty(&db->txq)) {
		struct sk_buff *skb;
		unsigned int len;

		skb = skb_dequeue(&db->txq);

		if (skb) {
			ret = ch390_transmit(db, skb->data, skb->len);
			len = skb->len;
			dev_kfree_skb(skb);

			if (ret < 0) {
				u64_stats_update_begin(&db->syncp);
				db->stats.tx_dropped++;
				db->stats.tx_errors++;
				u64_stats_update_end(&db->syncp);
				goto err;
			}
			u64_stats_update_begin(&db->syncp);
			db->stats.tx_packets++;
			db->stats.tx_bytes += len;
			u64_stats_update_end(&db->syncp);
		}

		if (netif_queue_stopped(ndev) &&
		    (skb_queue_len(&db->txq) < CH390_TX_QUE_LO_WATER))
			netif_wake_queue(ndev);
	}

	mutex_unlock(&db->dev_lock);
	return;

err:
	netdev_err(db->ndev, "transmit packet error");
	mutex_unlock(&db->dev_lock);
}

/**
 * ch390_async_apply_rx_mode - Apply new RX mode settings (bottom half).
 * @work: Pointer to the work_struct.
 *
 * Writes the cached MAC address, multicast hash table, and receive control
 * register values to the hardware. This is done in a workqueue to avoid
 * sleeping in the ndo_set_rx_mode callback.
 */
static void ch390_async_apply_rx_mode(struct work_struct *work)
{
	struct board_info *db =
		container_of(work, struct board_info, async_rx_mode_work);
	struct net_device *ndev = db->ndev;
	int ret;

	mutex_lock(&db->dev_lock);

	for (int i = 0; i < ETH_ALEN; i++) {
		CH390_GOTO_ON_ERROR(ch390_io_register_write(db, CH390_PAR + i,
							    ndev->dev_addr[i]),
				    err, "write PAR failed");
	}

	for (int i = 0; i < 8; i++) {
		CH390_GOTO_ON_ERROR(ch390_io_register_write(db, CH390_MAR + i,
							    db->hash_table[i]),
				    err, "write MAR failed");
	}
	CH390_GOTO_ON_ERROR(ch390_io_register_write(db, CH390_RCR, db->rcr),
			    err, "write RCR failed");

err:
	mutex_unlock(&db->dev_lock);
}

/**
 * ch390_open - Open the network device (ndo_open).
 * @ndev: Pointer to the network device structure.
 *
 * Called when the network device is brought up (e.g., via "ifconfig up").
 * It initializes hardware, starts the PHY, and enables the transmit queue.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_open(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);
	int ret;

	db->rcr = RCR_DIS_CRC | RCR_RXEN;
	memset(db->hash_table, 0, sizeof(db->hash_table));

	phy_support_sym_pause(db->phydev);
	phy_start(db->phydev);

	/* flow control parameters init */
	db->pause.rx_pause = true;
	db->pause.tx_pause = true;
	db->pause.autoneg = AUTONEG_DISABLE;

	if (db->phydev->autoneg)
		db->pause.autoneg = AUTONEG_ENABLE;

	ret = ch390_start(db);
	if (ret) {
		phy_stop(db->phydev);
		return ret;
	}

	netif_wake_queue(ndev);
	return 0;
}

/**
 * ch390_close - Close the network device (ndo_stop).
 * @ndev: Pointer to the network device structure.
 *
 * Called when the network device is brought down (e.g., via "ifconfig down").
 * It stops the hardware, stops the PHY, and cleans up running tasks.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_close(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);
	int ret;

	ret = ch390_stop(db);
	if (ret)
		return ret;

	flush_work(&db->async_tx_work);
	flush_work(&db->async_rx_mode_work);

	phy_stop(db->phydev);

	netif_stop_queue(ndev);

	skb_queue_purge(&db->txq);

	return 0;
}

/**
 * ch390_start_xmit - Transmit a packet (ndo_start_xmit).
 * @skb:  The socket buffer containing the packet to transmit.
 * @ndev: Pointer to the network device structure.
 *
 * This function is called by the network subsystem to send a packet. It queues
 * the packet and schedules the transmit workqueue to perform the actual I/O.
 *
 * Return: NETDEV_TX_OK.
 */
static netdev_tx_t ch390_start_xmit(struct sk_buff *skb,
				    struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);

	skb_queue_tail(&db->txq, skb);
	if (skb_queue_len(&db->txq) > CH390_TX_QUE_HI_WATER)
		netif_stop_queue(ndev); /* enforce limit queue size */
	schedule_work(&db->async_tx_work);
	return NETDEV_TX_OK;
}

/**
 * ch390_set_rx_mode - Configure the packet reception mode (ndo_set_rx_mode).
 * @ndev: Pointer to the network device structure.
 *
 * Called by the network subsystem to change the receiver's filter settings,
 * such as enabling promiscuous mode or updating the multicast address list.
 * It calculates the new settings and schedules a work item to apply them.
 */
static void ch390_set_rx_mode(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);
	struct netdev_hw_addr *ha;
	u8 rcr = RCR_DIS_CRC | RCR_RXEN;
	u32 hash_val;
	u8 hash_table[8];

	/* rx control */
	if (ndev->flags & IFF_PROMISC)
		rcr |= RCR_PRMSC;

	if (ndev->flags & IFF_ALLMULTI)
		rcr |= RCR_ALL;

	/* broadcast address */
	hash_table[0] = 0;
	hash_table[1] = 0;
	hash_table[2] = 0;
	hash_table[3] = 0;
	hash_table[4] = 0;
	hash_table[5] = 0;
	hash_table[6] = 0;
	hash_table[7] = 0x80;

	/* the multicast address in Hash Table : 64 bits */
	netdev_for_each_mc_addr(ha, ndev) {
		hash_val = crc32_le(~0, ha->addr, ETH_ALEN) & GENMASK(5, 0);
		hash_table[hash_val / 8] |= BIT(hash_val % 8);
	}

	/* schedule work to do the actual set of the data if needed */
	if (memcmp(db->hash_table, hash_table, sizeof(hash_table)) ||
	    db->rcr != rcr) {
		memcpy(db->hash_table, hash_table, sizeof(hash_table));
		db->rcr = rcr;
		schedule_work(&db->async_rx_mode_work);
	}
}

/**
 * ch390_set_mac_address - Set the MAC address (ndo_set_mac_address).
 * @ndev: Pointer to the network device structure.
 * @p:    Pointer to a sockaddr containing the new MAC address.
 *
 * This function allows changing the device's hardware MAC address.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_set_mac_address(struct net_device *ndev, void *p)
{
	struct board_info *db = to_ch390_board(ndev);
	struct sockaddr *addr = p;
	int ret;

	if (!(ndev->priv_flags & IFF_LIVE_ADDR_CHANGE) && netif_running(ndev))
		return -EBUSY;
	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	eth_commit_mac_addr_change(ndev, p);

	for (int i = 0; i < ETH_ALEN; i++) {
		CH390_RETURN_ON_ERROR(
			ch390_io_register_write(db, CH390_PAR + i,
						(ndev->dev_addr)[i]),
			"write PAR failed");
	}
	return ret;
}

/**
 * ch390_get_stats - Get network statistics (ndo_get_stats64).
 * @ndev:    Pointer to the network device structure.
 * @storage: Pointer to rtnl_link_stats64 to store the statistics.
 *
 * Provides the kernel with the driver's collected network statistics.
 */
static void ch390_get_stats(struct net_device *ndev,
			    struct rtnl_link_stats64 *storage)
{
	struct board_info *db = to_ch390_board(ndev);
	uint start;

	do {
		start = u64_stats_fetch_begin(&db->syncp);
		*storage = db->stats;
	} while (u64_stats_fetch_retry(&db->syncp, start));
}

/**
 * ch390_set_features - Configure offload and loopback features
 * @dev: Pointer to the struct net_device
 * @features: Bitmask of network device features to enable or disable
 *
 * This function is called by the networking stack to enable or disable
 * specific net_device features, such as:
 *  - NETIF_F_LOOPBACK: internal MAC loopback mode
 *  - NETIF_F_HW_CSUM: hardware checksum offload for transmit
 *  - NETIF_F_RXCSUM: hardware checksum offload for receive
 *
 * The function updates the corresponding CH390 registers:
 *  - NCR: Network Control Register (loopback)
 *  - TCSCR: Transmit Checksum Control Register (TX checksum)
 *  - RCSCSR: Receive Checksum Control and Status Register (RX checksum)
 *
 * Returns:
 *  0 on success
 *  Non-zero if any register read/write fails
 */
static int ch390_set_features(struct net_device *dev,
			      netdev_features_t features)
{
	struct board_info *db = to_ch390_board(dev);
	int ret = 0;
	int ncr;
	int tcscr;
	int rcscsr;

	/* Configure MAC loopback */
	if (features & NETIF_F_LOOPBACK) {
		CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_NCR,
							     &ncr),
				      "read NCR failed");
		ncr |= NCR_LBK_MAC;
		CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_NCR,
							      ncr),
				      "write NCR failed");
	} else {
		CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_NCR,
							     &ncr),
				      "read NCR failed");
		ncr &= ~NCR_LBK_MAC;
		CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_NCR,
							      ncr),
				      "write NCR failed");
	}

	/* Configure TX hardware checksum offload */
	if (features & NETIF_F_HW_CSUM) {
		CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_TCSCR,
							     &tcscr),
				      "read TCSCR failed");
		tcscr |= TCSCR_ALL;
		CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_TCSCR,
							      tcscr),
				      "write TCSCR failed");
	} else {
		CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_TCSCR,
							     &tcscr),
				      "read TCSCR failed");
		tcscr &= ~TCSCR_ALL;
		CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_TCSCR,
							      tcscr),
				      "write TCSCR failed");
	}

	/* Configure RX hardware checksum offload */
	if (features & NETIF_F_RXCSUM) {
		CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_RCSCSR,
							     &rcscsr),
				      "read RCSCSR failed");
		rcscsr |= (RCSCSR_RCSEN | RCSCSR_DCSE);
		CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_RCSCSR,
							      rcscsr),
				      "write RCSCSR failed");
		db->rx_csum = true;
	} else {
		CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_RCSCSR,
							     &rcscsr),
				      "read RCSCSR failed");
		rcscsr &= ~(RCSCSR_RCSEN | RCSCSR_DCSE);
		CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_RCSCSR,
							      rcscsr),
				      "write RCSCSR failed");
		db->rx_csum = false;
	}

	return 0;
}

static const struct net_device_ops ch390_netdev_ops = {
	.ndo_open = ch390_open,
	.ndo_stop = ch390_close,
	.ndo_start_xmit = ch390_start_xmit,
	.ndo_set_rx_mode = ch390_set_rx_mode,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_mac_address = ch390_set_mac_address,
	.ndo_get_stats64 = ch390_get_stats,
	.ndo_set_features = ch390_set_features
};

/**
 * ch390_wol_irq_handler - Handle Wake-on-LAN (WOL) GPIO interrupt
 * @irq:   IRQ number triggered by the WOL GPIO
 * @data:  Pointer to driver private data (struct board_info)
 *
 * Returns: IRQ_HANDLED to indicate the interrupt has been handled
 */
static irqreturn_t ch390_wol_irq_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}

/**
 * ch390_mdio_register - Allocate and register the MDIO bus.
 * @db: Pointer to the driver's private data structure.
 *
 * Sets up the MII bus structure and registers it with the kernel's MDIO
 * subsystem, allowing communication with the internal PHY.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_mdio_register(struct board_info *db)
{
	struct spi_device *spi = db->spidev;
	int ret;

	db->mdiobus = mdiobus_alloc();
	if (!db->mdiobus)
		return -ENOMEM;

	db->mdiobus->priv = db;
	db->mdiobus->read = ch390_mdio_read;
	db->mdiobus->write = ch390_mdio_write;
	db->mdiobus->name = "ch390-mdiobus";
	db->mdiobus->phy_mask = (u32)~BIT(1);
	db->mdiobus->parent = &spi->dev;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0))
	snprintf(db->mdiobus->id, MII_BUS_ID_SIZE, "ch390-%s.%u",
		 dev_name(&spi->dev), spi_get_chipselect(spi, 0));
#else
	snprintf(db->mdiobus->id, MII_BUS_ID_SIZE, "ch390-%s.%u",
		 dev_name(&spi->dev), spi->chip_select);
#endif

	ret = mdiobus_register(db->mdiobus);
	if (ret) {
		netdev_err(db->ndev, "can't register MDIO bus");
		goto err;
	}
	return 0;

err:
	mdiobus_free(db->mdiobus);
	return ret;
}

/**
 * ch390_mdio_unregister - Unregister and free the MDIO bus.
 * @db: Pointer to the driver's private data structure.
 */
static void ch390_mdio_unregister(struct board_info *db)
{
	mdiobus_unregister(db->mdiobus);
	mdiobus_free(db->mdiobus);
}

/**
 * ch390_handle_link_change - PHY link state change handler.
 * @ndev: Pointer to the network device structure.
 *
 * This function is a callback that is invoked by the PHY library when the
 * link status changes (e.g., cable connected/disconnected). It updates flow
 * control settings accordingly.
 */
static void ch390_handle_link_change(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);

	phy_print_status(db->phydev);

	/*
	 * only write pause settings to mac. since mac and phy are integrated
	 * together, such as link state, speed and duplex are sync already
	 */
	if (db->phydev->link) {
		netif_carrier_on(db->ndev);
		if (db->phydev->pause) {
			db->pause.rx_pause = true;
			db->pause.tx_pause = true;
		}
	} else {
		netif_carrier_off(db->ndev);
	}
	ch390_update_fcr(db);
}

/**
 * ch390_phy_connect - Connect the driver to the PHY device.
 * @db: Pointer to the driver's private data structure.
 *
 * Uses the registered MDIO bus to find and connect to the internal PHY.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_phy_connect(struct board_info *db)
{
	char phy_id[MII_BUS_ID_SIZE + 3];

	snprintf(phy_id, sizeof(phy_id), PHY_ID_FMT, db->mdiobus->id,
		 CH390_PHY_ADDR);

	db->phydev = phy_connect(db->ndev, phy_id, ch390_handle_link_change,
				 PHY_INTERFACE_MODE_INTERNAL);
	if (IS_ERR(db->phydev))
		return PTR_ERR(db->phydev);
	return 0;
}

/**
 * ch390_request_irq - Request and configure the interrupt line.
 * @db: Pointer to the driver's private data structure.
 *
 * Requests the IRQ from the kernel and configures the hardware interrupt
 * polarity based on the device tree settings.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_request_irq(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	struct spi_device *spi = db->spidev;
	int ret;

	ndev->irq = spi->irq;
	if (db->irq_high) {
		CH390_GOTO_ON_ERROR(
			request_threaded_irq(spi->irq, NULL, ch390_irq_handler,
					     IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					     ndev->name, db),
			err, "fail to request irq");
		CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_INTCR,
							      INCR_POL_H),
				      "write INTCR failed");
	} else {
		CH390_GOTO_ON_ERROR(
			request_threaded_irq(spi->irq, NULL, ch390_irq_handler,
					     IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					     ndev->name, db),
			err, "fail to request irq");
		CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_INTCR,
							      INCR_POL_L),
				      "write INTCR failed");
	}

	db->phydev->irq = PHY_MAC_INTERRUPT;
	return 0;

err:
	netdev_err(ndev, "failed to request irq!");
	return ret;
}

/**
 * ch390_probe - Probe function for the SPI device.
 * @spi: Pointer to the SPI device structure.
 *
 * This is the main entry point for the driver. It's called by the SPI subsystem
 * when a device matching this driver is found. It handles memory allocation,
 * hardware reset and verification, MAC address initialization, MDIO and PHY
 * setup, and registration of the network device.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct net_device *ndev;
	struct board_info *db;
	bool support_eeprom;
	int ret = 0;

	ndev = alloc_etherdev(sizeof(*db));
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, dev);
	dev_set_drvdata(dev, ndev);

	db = netdev_priv(ndev);

	db->msg_enable = NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK;
	db->spidev = spi;
	db->ndev = ndev;
	db->rx_csum = true;
	memset(&db->stats, 0, sizeof(struct rtnl_link_stats64));
	skb_queue_head_init(&db->txq);

	ndev->netdev_ops = &ch390_netdev_ops;
	ndev->ethtool_ops = &ch390_ethtool_ops;
	ndev->features = NETIF_F_HW_CSUM | NETIF_F_RXCSUM;
	ndev->hw_features = ndev->features | NETIF_F_LOOPBACK;

	mutex_init(&db->dev_lock);
	mutex_init(&db->spi_lock);

	INIT_WORK(&db->async_tx_work, ch390_async_transmit);
	INIT_WORK(&db->async_rx_mode_work, ch390_async_apply_rx_mode);

	support_eeprom = (bool)dev_get_drvdata(dev);
	if (of_find_property(dev->of_node, "wch,eeprom", NULL)) {
		if (support_eeprom) {
			db->has_eeprom = true;
		} else {
			CH390_PRINT_ERROR("chip not support eeprom!");
			db->has_eeprom = false;
		}
	} else {
		db->has_eeprom = false;
	}

	if (irq_get_trigger_type(spi->irq) == IRQ_TYPE_LEVEL_HIGH)
		db->irq_high = true;
	else
		db->irq_high = false;

	db->wol_gpiod = devm_gpiod_get_optional(&spi->dev, "wakeup", GPIOD_IN);

	if (!db->wol_gpiod) {
		dev_err(&spi->dev,
			"no valid wakeup-gpios specified, forcibly disabling WOL support!");
		db->wolinfo.supported = 0;
	} else {
		db->wol_irq = gpiod_to_irq(db->wol_gpiod);
		db->wolinfo.supported = WAKE_PHY | WAKE_MAGIC;
		device_init_wakeup(&spi->dev, true);
		CH390_GOTO_ON_ERROR(devm_request_irq(&spi->dev, db->wol_irq,
						     ch390_wol_irq_handler,
						     IRQF_TRIGGER_RISING,
						     "ch390-wol", db),
				    err_nd, "request wol irq failed");
	}
	db->wolinfo.wolopts = 0;

	CH390_GOTO_ON_ERROR(ch390_reset(db), err_nd, "reset hardware failed");
	msleep(25);
	CH390_GOTO_ON_ERROR(ch390_verify_id(db), err_nd,
			    "verify hardware failed");
	CH390_GOTO_ON_ERROR(ch390_init_mac_addr(ndev, db), err_nd,
			    "init mac address failed");
	CH390_GOTO_ON_ERROR(ch390_init_hw_offload(db), err_nd,
			    "init hardware offload failed");
	CH390_GOTO_ON_ERROR(ch390_init_hw_rxlen_filter(db), err_nd,
			    "init hardware rx length filter failed");
	CH390_GOTO_ON_ERROR(register_netdev(ndev), err_nd,
			    "register netdev failed");
	CH390_GOTO_ON_ERROR(ch390_mdio_register(db), err_nd,
			    "register mdio failed");
	CH390_GOTO_ON_ERROR(ch390_phy_connect(db), err_mdio,
			    "connect phy failed");
	CH390_GOTO_ON_ERROR(ch390_request_irq(db), err_phy,
			    "request irq failed");
	return 0;

err_phy:
	phy_disconnect(db->phydev);
err_mdio:
	ch390_mdio_unregister(db);
err_nd:
	free_netdev(ndev);
	return ret;
}

/**
 * ch390_remove - Remove function for the SPI device.
 * @spi: Pointer to the SPI device structure.
 *
 * This function is called when the device is removed from the system. It
 * unregisters the network device, disconnects the PHY, frees the IRQ, and
 * releases all allocated resources.
 */
static void ch390_remove(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct board_info *db = to_ch390_board(ndev);

	free_irq(db->spidev->irq, db);
	phy_disconnect(db->phydev);
	ch390_mdio_unregister(db);
	unregister_netdev(ndev);
	free_netdev(ndev);
}

#ifdef CONFIG_PM_SLEEP
/**
 * ch390_suspend - Device suspend callback
 * @dev: Pointer to the struct device representing the network device
 *
 * Called when the system enters a low-power state. The function:
 * 1. Retrieves the network device associated with the device.
 * 2. Retrieves the board-specific information.
 * 3. Writes to the CH390 SCCR register to disable the device clock for power saving.
 *
 * Returns:
 * 0 on success
 * Non-zero if writing to the register fails
 */
static int ch390_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct board_info *db = to_ch390_board(ndev);
	int ret;
	u8 ncr;

	if (device_may_wakeup(&db->spidev->dev) && db->wolinfo.wolopts) {
		enable_irq_wake(db->wol_irq);
		CH390_RETURN_ON_ERROR(ch390_io_register_read(db, CH390_NCR,
							     &ncr),
				      "read NCR failed");
		if (ncr & NCR_MACPD)
			ncr &= ~NCR_MACPD;
		else
			ncr |= NCR_MACPD;
		CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_NCR,
							      ncr),
				      "write NCR failed");
	} else {
		CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_SCCR,
							      SCCR_DIS_CLK),
				      "write SCCR failed");
	}

	return 0;
}

/**
 * ch390_resume - Device resume callback
 * @dev: Pointer to the struct device representing the network device
 *
 * Called when the system resumes from a low-power state. The function:
 * 1. Retrieves the network device associated with the device.
 * 2. Retrieves the board-specific information.
 * 3. Writes to the CH390 RSCCR register to restore the device clock.
 * 4. Waits for 2 milliseconds to ensure clock stabilization.
 *
 * Returns:
 * 0 on success
 * Non-zero if writing to the register fails
 */
static int ch390_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct board_info *db = to_ch390_board(ndev);
	int ret;

	if (device_may_wakeup(&db->spidev->dev) && db->wolinfo.wolopts) {
		disable_irq_wake(db->wol_irq);
	} else {
		CH390_RETURN_ON_ERROR(ch390_io_register_write(db, CH390_RSCCR,
							      0x00),
				      "write RSCCR failed");
		usleep_range(2000, 5000); // Wait 2ms for clock stabilization
	}
	return 0;
}
#endif
DEFINE_SIMPLE_DEV_PM_OPS(ch390_pm_ops, ch390_suspend, ch390_resume);

/*
 * Device tree match table for CH390 variants.
 *
 * The 'data' field is used to indicate whether the NIC supports
 * an EEPROM. A value of 'true' means EEPROM is present/usable,
 * while 'false' indicates no EEPROM support.
 */
static const struct of_device_id ch390_match_table[] = {
	{ .compatible = "wch,ch390h", .data = (void *)true },
	{ .compatible = "wch,ch390d", .data = (void *)false },
	{}
};

static const struct spi_device_id ch390_id_table[] = { { "ch390h", 0 },
						       { "ch390d", 1 },
						       {} };

static struct spi_driver ch390_driver = {
	.driver = { .name = DRVNAME_CH390,
		    .of_match_table = ch390_match_table,
		    .pm = &ch390_pm_ops },
	.probe = ch390_probe,
	.remove = ch390_remove,
	.id_table = ch390_id_table,
};

module_spi_driver(ch390_driver);

MODULE_AUTHOR("Sergey Kharenko <skharenko@hust.edu.cn>");
MODULE_DESCRIPTION("SPI ethernet driver for CH390H/D");
MODULE_VERSION("0.2.0");
MODULE_LICENSE("GPL");
