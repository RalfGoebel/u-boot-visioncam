/*
 * Copyright (C) 2016 IMAGO Technologies GmbH - http://www.imago-technologies.com
 *
 * Author: Ralf Goebel <ralf.goebel@imago-technologies.com>
 *
 * Configuration settings for the VisionCam XM smart camera.
 * See ti_omap5_common.h for omap5 common settings.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_VISIONCAM_XM_H
#define __CONFIG_VISIONCAM_XM_H

#define CONFIG_AM57XX

#define CONFIG_IODELAY_RECALIBRATION

#define CONFIG_BOARD_EARLY_INIT_F
/* Call board_late_init() */
#define CONFIG_BOARD_LATE_INIT

#define CONFIG_NR_DRAM_BANKS		2

#define CONFIG_ENV_SIZE			(64 << 10)
#define CONFIG_ENV_IS_IN_FAT
#define FAT_ENV_INTERFACE		"mmc"
#define FAT_ENV_DEVICE_AND_PART	"0:1"
#define FAT_ENV_FILE			"uboot.env"

#define CONSOLEDEV			"ttyS2"
#define CONFIG_SYS_NS16550_COM1		UART1_BASE	/* Base EVM has UART0 */
#define CONFIG_SYS_NS16550_COM2		UART2_BASE	/* UART2 */
#define CONFIG_SYS_NS16550_COM3		UART3_BASE	/* UART3 */
#define CONFIG_BAUDRATE			115200

#define CONFIG_SYS_OMAP_ABE_SYSCK

/* Define the default GPT table for eMMC */
/*#define PARTS_DEFAULT \
	"uuid_disk=${uuid_gpt_disk};" \
	"name=rootfs,start=2MiB,size=-,uuid=${uuid_gpt_rootfs}"*/

#undef CONFIG_OMAP3_SPI

#include <configs/ti_omap5_common.h>

#undef CONFIG_EXTRA_ENV_SETTINGS
#define CONFIG_EXTRA_ENV_SETTINGS \
	DEFAULT_LINUX_BOOT_ENV \
	DEFAULT_MMC_TI_ARGS \
	"console=" CONSOLEDEV ",115200n8\0" \
	"fdtfile=am572x-visioncam-xm.dtb\0" \
	"bootpart=0:1\0" \
	"bootdir=/boot\0" \
	"loadimage=load mmc ${bootpart} ${loadaddr} ${bootdir}/${bootfile}\0" \
	"mmcboot=mmc dev ${mmcdev}; " \
		"if mmc rescan; then " \
			"echo SD/MMC found on device ${mmcdev};" \
			"if run loadimage; then " \
				"run loadfdt; " \
				"echo Booting from mmc${mmcdev} ...; " \
				"run args_mmc; " \
				"bootz ${loadaddr} - ${fdtaddr}; " \
			"fi;" \
		"fi;\0" \
	"loadfdt=load mmc ${bootpart} ${fdtaddr} ${bootdir}/${fdtfile};\0" \
	"loadfpga=load mmc ${bootpart} ${loadaddr} ${bootdir}/${fpgafile}; fpga_init ${loadaddr} ${filesize};\0" \
	NETARGS

#undef CONFIG_BOOTCOMMAND
#define CONFIG_BOOTCOMMAND \
	"run envboot; " \
	"run loadfpga; " \
	"run mmcboot;" \
	""

/* Enhance our eMMC support / experience. */
/*#define CONFIG_CMD_GPT
#define CONFIG_EFI_PARTITION*/

/* CPSW Ethernet */
#define CONFIG_BOOTP_DNS		/* Configurable parts of CMD_DHCP */
#define CONFIG_BOOTP_DNS2
#define CONFIG_BOOTP_SEND_HOSTNAME
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_SUBNETMASK
#define CONFIG_NET_RETRY_COUNT		10
#define CONFIG_DRIVER_TI_CPSW		/* Driver for IP block */
#define CONFIG_MII			/* Required in net/eth.c */
#define CONFIG_PHY_GIGE			/* per-board part of CPSW */
#define CONFIG_PHYLIB
#define PHY_ANEG_TIMEOUT	8000	/* PHY needs longer aneg time at 1G */

/*#define CONFIG_SUPPORT_EMMC_BOOT*/

/* EEPROM */
#define CONFIG_EEPROM_CHIP_ADDRESS 0x50
#define CONFIG_EEPROM_BUS_ADDRESS 0

/* RTC MCP79410 */
#define CONFIG_CMD_DATE
#define CONFIG_RTC_MCP79411
#define CONFIG_SYS_RTC_BUS_NUM 2
#define CONFIG_SYS_I2C_RTC_ADDR 0x6f

#endif /* __CONFIG_VISIONCAM_XM_H */
