/*
 * visionsensor_pv.h
 *
 * Copyright (C) IMAGO Technologies GmbH - <http://www.imago-technologies.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_VISIONSENSOR_PV_H
#define __CONFIG_VISIONSENSOR_PV_H

#define CONFIG_BOARD_LATE_INIT
#define CONFIG_ARCH_CPU_INIT
/*#define CONFIG_LAST_STAGE_INIT*/  /* => last_stage_init() in board.c */

#define CONFIG_MAX_RAM_BANK_SIZE	(512 << 20)	/* 512 MB */
#define CONFIG_SYS_TIMERBASE		0x48040000	/* Use Timer2 */

#include <asm/arch/omap.h>

/* NS16550 Configuration */
#define CONFIG_SYS_NS16550_CLK		48000000
#if !defined(CONFIG_SPL_DM) || !defined(CONFIG_DM_SERIAL)
#define CONFIG_SYS_NS16550_REG_SIZE    (-4)
#define CONFIG_SYS_NS16550_SERIAL
#endif

/* I2C Configuration */
#define CONFIG_CMD_EEPROM
#define CONFIG_ENV_EEPROM_IS_ON_I2C
#define CONFIG_SYS_I2C_EEPROM_ADDR	0x50	/* Main EEPROM */
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN	2

/* Power */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_TPS65218

/* SPL defines. */
#define CONFIG_SPL_TEXT_BASE		CONFIG_ISW_ENTRY_ADDR
#define CONFIG_SYS_SPL_ARGS_ADDR	(CONFIG_SYS_SDRAM_BASE + \
					 (128 << 20))

/* Enabling L2 Cache */
#define CONFIG_SYS_L2_PL310
#define CONFIG_SYS_PL310_BASE	0x48242000

/*
 * Since SPL did pll and ddr initialization for us,
 * we don't need to do it twice.
 */
#if !defined(CONFIG_SPL_BUILD) && !defined(CONFIG_QSPI_BOOT)
#define CONFIG_SKIP_LOWLEVEL_INIT
#endif

/*
 * When building U-Boot such that there is no previous loader
 * we need to call board_early_init_f.  This is taken care of in
 * s_init when we have SPL used.
 */
#if !defined(CONFIG_SKIP_LOWLEVEL_INIT) && !defined(CONFIG_SPL)
#define CONFIG_BOARD_EARLY_INIT_F
#endif

/* Now bring in the rest of the common code. */
#include <configs/ti_armv7_omap.h>

/* Always 64 KiB env size */
#define CONFIG_ENV_SIZE			(64 << 10)

/*#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG*/

/* Clock Defines */
#define V_OSCK				26000000  /* Clock output from T2 */
#define V_SCLK				(V_OSCK)

/* NS16550 Configuration */
#define CONFIG_SYS_NS16550_COM1		0x44e09000	/* Base EVM has UART0 */

#define CONFIG_ENV_IS_IN_FAT
#define FAT_ENV_INTERFACE		"mmc"
#define FAT_ENV_DEVICE_AND_PART		"0:1"
#define FAT_ENV_FILE			"uboot.env"
#define CONFIG_FAT_WRITE

#define CONFIG_SPL_LDSCRIPT		"arch/arm/mach-omap2/u-boot-spl.lds"

/*#define CONFIG_SPL_LOAD_FIT_ADDRESS 0x80800000*/

/*
 * Disable MMC DM for SPL build and can be re-enabled after adding
 * DM support in SPL
 */
#ifdef CONFIG_SPL_BUILD
#undef CONFIG_TIMER
/*#undef CONFIG_DM_NAND*/
#endif

#define DFUARGS

/* SPI */
#undef CONFIG_OMAP3_SPI
#define CONFIG_TI_EDMA3

/* Enhance our eMMC support / experience. */
#define CONFIG_CMD_GPT
#define CONFIG_EFI_PARTITION

#ifndef CONFIG_SPL_BUILD
#include <environment/ti/mmc.h>

#define CONFIG_EXTRA_ENV_SETTINGS \
	DEFAULT_LINUX_BOOT_ENV \
	DEFAULT_MMC_TI_ARGS \
	"fdtfile=undefined\0" \
	"bootpart=0:1\0" \
	"bootdir=\0" \
    "boot_fit=0\0" \
	"bootfile=undefined\0" \
	"console=ttyS0,115200n8\0" \
    "finduuid=part uuid mmc 0:2 uuid\0" \
	"optargs=\0" \

#define CONFIG_BOOTCOMMAND \
	"run envboot;" \
	"run mmcboot;" \

#endif

#ifndef CONFIG_SPL_BUILD
/* CPSW Ethernet */
#define CONFIG_MII
#define CONFIG_BOOTP_DEFAULT
#define CONFIG_BOOTP_DNS
#define CONFIG_BOOTP_DNS2
#define CONFIG_BOOTP_SEND_HOSTNAME
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_SUBNETMASK
#define CONFIG_NET_RETRY_COUNT		10
/*#define CONFIG_PHY_GIGE*/
#endif

#define CONFIG_DRIVER_TI_CPSW
#define CONFIG_PHYLIB
#define PHY_ANEG_TIMEOUT	8000 /* PHY needs longer aneg time at 1G */

#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_ETH_SUPPORT)
#undef CONFIG_ENV_IS_IN_FAT
#define CONFIG_ENV_IS_NOWHERE
#endif

#define CONFIG_SYS_RX_ETH_BUFFER	64

#define NANDARGS
#define NANDBOOT

#if defined(CONFIG_TI_SECURE_DEVICE)
/* Avoid relocating onto firewalled area at end of DRAM */
#define CONFIG_PRAM (64 * 1024)
#endif /* CONFIG_TI_SECURE_DEVICE */

#endif	/* __CONFIG_VISIONSENSOR_PV_H */
