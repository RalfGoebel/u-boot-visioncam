/*
 * Copyright (C) 2016 IMAGO Technologies GmbH - http://www.imago-technologies.com
 *
 * Author: Ralf Goebel <ralf.goebel@imago-technologies.com>
 *
 * Based on board/ti/dra7xx/evm.c
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _MUX_DATA_VCXM_H_
#define _MUX_DATA_VCXM_H_

#include <asm/arch/mux_dra7xx.h>

const struct pad_conf_entry core_padconf_array_visioncam_xm[] = {
	{VIN2A_D3, M14 | PIN_OUTPUT},	// GPIO4_4 => FPGA nConfig
	{VIN2A_D4, M14 | PIN_INPUT},	// GPIO4_5 => FPGA nStatus
	{VIN2A_D6, M14 | PIN_INPUT},	// GPIO4_7 => FPGA CONF_DONE
	{VIN2A_D16, M14 | PIN_OUTPUT},	// GPIO4_24 => FPGA Reset

	{VOUT1_D12, M14 | PIN_INPUT_PULLUP},	// GPIO8_12 => not 100 MBit

	// SPI4 => FPGA config.
	{GPMC_A8, (M8 | PIN_OUTPUT)},	/* gpmc_a8.spi4_sclk => DCLK */
	{GPMC_A10, (M8 | PIN_OUTPUT)},	/* gpmc_a10.spi4_d0 => DATA0 */

	// I2C3 => RTC
	{GPMC_CLK, (M8 | PIN_INPUT_PULLUP)},	/* gpmc_clk.i2c3_scl */
	{GPMC_ADVN_ALE, (M8 | PIN_INPUT_PULLUP)},	/* gpmc_advn_ale.i2c3_sda */

	// RGMII
	{MDIO_MCLK, (M0 | PIN_INPUT_PULLUP)},	/* mdio_mclk.mdio_mclk */
	{MDIO_D, (M0 | PIN_INPUT_PULLUP)},	/* mdio_d.mdio_d */
	{RGMII0_TXC, (M0 | PIN_INPUT_PULLDOWN | MANUAL_MODE)},	/* rgmii0_txc.rgmii0_txc */
	{RGMII0_TXCTL, (M0 | PIN_INPUT_PULLDOWN | MANUAL_MODE)},	/* rgmii0_txctl.rgmii0_txctl */
	{RGMII0_TXD3, (M0 | PIN_INPUT_PULLDOWN | MANUAL_MODE)},	/* rgmii0_txd3.rgmii0_txd3 */
	{RGMII0_TXD2, (M0 | PIN_INPUT_PULLDOWN | MANUAL_MODE)},	/* rgmii0_txd2.rgmii0_txd2 */
	{RGMII0_TXD1, (M0 | PIN_INPUT_PULLDOWN | MANUAL_MODE)},	/* rgmii0_txd1.rgmii0_txd1 */
	{RGMII0_TXD0, (M0 | PIN_INPUT_PULLDOWN | MANUAL_MODE)},	/* rgmii0_txd0.rgmii0_txd0 */
	{RGMII0_RXC, (M0 | PIN_INPUT_PULLDOWN | MANUAL_MODE)},	/* rgmii0_rxc.rgmii0_rxc */
	{RGMII0_RXCTL, (M0 | PIN_INPUT_PULLDOWN | MANUAL_MODE)},	/* rgmii0_rxctl.rgmii0_rxctl */
	{RGMII0_RXD3, (M0 | PIN_INPUT_PULLUP | MANUAL_MODE)},	/* rgmii0_rxd3.rgmii0_rxd3 */
	{RGMII0_RXD2, (M0 | PIN_INPUT_PULLUP | MANUAL_MODE)},	/* rgmii0_rxd2.rgmii0_rxd2 */
	{RGMII0_RXD1, (M0 | PIN_INPUT_PULLUP | MANUAL_MODE)},	/* rgmii0_rxd1.rgmii0_rxd1 */
	{RGMII0_RXD0, (M0 | PIN_INPUT_PULLUP | MANUAL_MODE)},	/* rgmii0_rxd0.rgmii0_rxd0 */

	// PRU
	{XREF_CLK0, (M11 | PIN_INPUT_PULLDOWN)},	/* xref_clk0.pr2_mii1_col */
	{XREF_CLK1, (M11 | PIN_INPUT_PULLDOWN)},	/* xref_clk1.pr2_mii1_crs */

	{MMC1_CLK, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_clk.mmc1_clk */
	{MMC1_CMD, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_cmd.mmc1_cmd */
	{MMC1_DAT0, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_dat0.mmc1_dat0 */
	{MMC1_DAT1, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_dat1.mmc1_dat1 */
	{MMC1_DAT2, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_dat2.mmc1_dat2 */
	{MMC1_DAT3, (M0 | PIN_INPUT_PULLUP)},	/* mmc1_dat3.mmc1_dat3 */
	{MMC1_SDCD, (M14 | PIN_INPUT_PULLUP)},	/* mmc1_sdcd.gpio6_27 */
	{MMC1_SDWP, (M15 | PIN_OUTPUT)},	/* mmc1_sdwp.DriverOff */
	{UART2_CTSN, (M2 | PIN_INPUT_SLEW)},	/* uart2_ctsn.uart3_rxd */
	{UART2_RTSN, (M1 | PIN_INPUT_SLEW)},	/* uart2_rtsn.uart3_txd */
	{ON_OFF, (M1 | PIN_OUTPUT_PULLUP)},	/* on_off.on_off */
};

const struct pad_conf_entry early_padconf[] = {
	{UART2_CTSN, (M2 | PIN_INPUT_SLEW)},	/* uart2_ctsn.uart3_rxd */
	{UART2_RTSN, (M1 | PIN_INPUT_SLEW)},	/* uart2_rtsn.uart3_txd */
	{I2C1_SDA, (PIN_INPUT_PULLUP | M0)},	/* I2C1_SDA */
	{I2C1_SCL, (PIN_INPUT_PULLUP | M0)},	/* I2C1_SCL */
};

#ifdef CONFIG_IODELAY_RECALIBRATION

const struct iodelay_cfg_entry iodelay_cfg_array_vcxm[] = {
	{0x06F0, 480, 0},	/* CFG_RGMII0_RXC_IN */
	{0x06FC, 111, 1641},	/* CFG_RGMII0_RXCTL_IN */
	{0x0708, 272, 1116},	/* CFG_RGMII0_RXD0_IN */
	{0x0714, 243, 1260},	/* CFG_RGMII0_RXD1_IN */
	{0x0720, 0, 1614},	/* CFG_RGMII0_RXD2_IN */
	{0x072C, 105, 1673},	/* CFG_RGMII0_RXD3_IN */
	{0x0740, 531, 120},	/* CFG_RGMII0_TXC_OUT */
	{0x074C, 201, 60},	/* CFG_RGMII0_TXCTL_OUT */
	{0x0758, 229, 120},	/* CFG_RGMII0_TXD0_OUT */
	{0x0764, 141, 0},	/* CFG_RGMII0_TXD1_OUT */
	{0x0770, 495, 120},	/* CFG_RGMII0_TXD2_OUT */
	{0x077C, 660, 120},	/* CFG_RGMII0_TXD3_OUT */
};


#endif
#endif /* _MUX_DATA_VCXM_H_ */
