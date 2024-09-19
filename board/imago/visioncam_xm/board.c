/*
 * Copyright (C) 2016 IMAGO Technologies GmbH - http://www.imago-technologies.com
 *
 * Author: Ralf Goebel <ralf.goebel@imago-technologies.com>
 *
 * Based on board/ti/dra7xx/evm.c
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <palmas.h>
#include <fdt_support.h>
#include <asm/omap_common.h>
#include <asm/emif.h>
#include <asm/gpio.h>
#include <asm/arch/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/dra7xx_iodelay.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/gpio.h>
#include <asm/arch/omap.h>
#include <environment.h>
#include <rtc.h>
#include <miiphy.h>

#include "altera.h"
#include "mux_data.h"

#define GPIO_NOT100MBIT	((8-1)*32 + 12)	// gpio8_12

DECLARE_GLOBAL_DATA_PTR;

/* GPIO 7_11 */
//#define GPIO_DDR_VTT_EN 203

#define SYSINFO_BOARD_NAME_MAX_LEN	45

const struct omap_sysinfo sysinfo = {
	"Board: UNKNOWN REV UNKNOWN\n"
};

static const struct dmm_lisa_map_regs am571x_vcxm_lisa_regs = {
	.dmm_lisa_map_3 = 0x80600200,
	.is_ma_present  = 0x1
};

void emif_get_dmm_regs(const struct dmm_lisa_map_regs **dmm_lisa_regs)
{
	*dmm_lisa_regs = &am571x_vcxm_lisa_regs;
}

static const struct emif_regs beagle_x15_emif1_ddr3_532mhz_emif_regs = {
	.sdram_config_init	= 0x61851b32,
	.sdram_config		= 0x61851b32,
	.sdram_config2		= 0x08000000,
	.ref_ctrl		= 0x000040F1,
	.ref_ctrl_final		= 0x00001035,
	.sdram_tim1		= 0xcccf36ab,
	.sdram_tim2		= 0x308f7fda,
	.sdram_tim3		= 0x409f88a8,
	.read_idle_ctrl		= 0x00050000,
	.zq_config		= 0x5007190b,
	.temp_alert_config	= 0x00000000,
	.emif_ddr_phy_ctlr_1_init = 0x0024400b,
	.emif_ddr_phy_ctlr_1	= 0x0e24400b,
	.emif_ddr_ext_phy_ctrl_1 = 0x10040100,
	.emif_ddr_ext_phy_ctrl_2 = 0x00910091,
	.emif_ddr_ext_phy_ctrl_3 = 0x00950095,
	.emif_ddr_ext_phy_ctrl_4 = 0x009b009b,
	.emif_ddr_ext_phy_ctrl_5 = 0x009e009e,
	.emif_rd_wr_lvl_rmp_win	= 0x00000000,
	.emif_rd_wr_lvl_rmp_ctl	= 0x80000000,
	.emif_rd_wr_lvl_ctl	= 0x00000000,
	.emif_rd_wr_exec_thresh	= 0x00000305
};

/* Ext phy ctrl regs 1-35 */
static const u32 beagle_x15_emif1_ddr3_ext_phy_ctrl_const_regs[] = {
	0x10040100,
	0x00910091,
	0x00950095,
	0x009B009B,
	0x009E009E,
	0x00980098,
	0x00340034,
	0x00350035,
	0x00340034,
	0x00310031,
	0x00340034,
	0x007F007F,
	0x007F007F,
	0x007F007F,
	0x007F007F,
	0x007F007F,
	0x00480048,
	0x004A004A,
	0x00520052,
	0x00550055,
	0x00500050,
	0x00000000,
	0x00600020,
	0x40011080,
	0x08102040,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0
};

static const struct emif_regs beagle_x15_emif2_ddr3_532mhz_emif_regs = {
	.sdram_config_init	= 0x61851b32,
	.sdram_config		= 0x61851b32,
	.sdram_config2		= 0x08000000,
	.ref_ctrl		= 0x000040F1,
	.ref_ctrl_final		= 0x00001035,
	.sdram_tim1		= 0xcccf36b3,
	.sdram_tim2		= 0x308f7fda,
	.sdram_tim3		= 0x407f88a8,
	.read_idle_ctrl		= 0x00050000,
	.zq_config		= 0x5007190b,
	.temp_alert_config	= 0x00000000,
	.emif_ddr_phy_ctlr_1_init = 0x0024400b,
	.emif_ddr_phy_ctlr_1	= 0x0e24400b,
	.emif_ddr_ext_phy_ctrl_1 = 0x10040100,
	.emif_ddr_ext_phy_ctrl_2 = 0x00910091,
	.emif_ddr_ext_phy_ctrl_3 = 0x00950095,
	.emif_ddr_ext_phy_ctrl_4 = 0x009b009b,
	.emif_ddr_ext_phy_ctrl_5 = 0x009e009e,
	.emif_rd_wr_lvl_rmp_win	= 0x00000000,
	.emif_rd_wr_lvl_rmp_ctl	= 0x80000000,
	.emif_rd_wr_lvl_ctl	= 0x00000000,
	.emif_rd_wr_exec_thresh	= 0x00000305
};

static const u32 beagle_x15_emif2_ddr3_ext_phy_ctrl_const_regs[] = {
	0x10040100,
	0x00910091,
	0x00950095,
	0x009B009B,
	0x009E009E,
	0x00980098,
	0x00340034,
	0x00350035,
	0x00340034,
	0x00310031,
	0x00340034,
	0x007F007F,
	0x007F007F,
	0x007F007F,
	0x007F007F,
	0x007F007F,
	0x00480048,
	0x004A004A,
	0x00520052,
	0x00550055,
	0x00500050,
	0x00000000,
	0x00600020,
	0x40011080,
	0x08102040,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0,
	0x0
};

void emif_get_reg_dump(u32 emif_nr, const struct emif_regs **regs)
{
	switch (emif_nr) {
	case 1:
		*regs = &beagle_x15_emif1_ddr3_532mhz_emif_regs;
		break;
	case 2:
		*regs = &beagle_x15_emif2_ddr3_532mhz_emif_regs;
		break;
	}
}

void emif_get_ext_phy_ctrl_const_regs(u32 emif_nr, const u32 **regs, u32 *size)
{
	switch (emif_nr) {
	case 1:
		*regs = beagle_x15_emif1_ddr3_ext_phy_ctrl_const_regs;
		*size = ARRAY_SIZE(beagle_x15_emif1_ddr3_ext_phy_ctrl_const_regs);
		break;
	case 2:
		*regs = beagle_x15_emif2_ddr3_ext_phy_ctrl_const_regs;
		*size = ARRAY_SIZE(beagle_x15_emif2_ddr3_ext_phy_ctrl_const_regs);
		break;
	}
}

struct vcores_data beagle_x15_volts = {
	.mpu.value[OPP_NOM]	= VDD_MPU_DRA7_NOM,
	.mpu.efuse.reg[OPP_NOM]	= STD_FUSE_OPP_VMIN_MPU_NOM,
	.mpu.efuse.reg_bits     = DRA752_EFUSE_REGBITS,
	.mpu.addr		= TPS659038_REG_ADDR_SMPS12,
	.mpu.pmic		= &tps659038,
	.mpu.abb_tx_done_mask	= OMAP_ABB_MPU_TXDONE_MASK,

	.eve.value[OPP_NOM]	= VDD_EVE_DRA7_NOM,
	.eve.value[OPP_OD]	= VDD_EVE_DRA7_OD,
	.eve.value[OPP_HIGH]	= VDD_EVE_DRA7_HIGH,
	.eve.efuse.reg[OPP_NOM]	= STD_FUSE_OPP_VMIN_DSPEVE_NOM,
	.eve.efuse.reg[OPP_OD]	= STD_FUSE_OPP_VMIN_DSPEVE_OD,
	.eve.efuse.reg[OPP_HIGH]	= STD_FUSE_OPP_VMIN_DSPEVE_HIGH,
	.eve.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.eve.addr		= TPS659038_REG_ADDR_SMPS45,
	.eve.pmic		= &tps659038,
	.eve.abb_tx_done_mask	= OMAP_ABB_EVE_TXDONE_MASK,

	.gpu.value[OPP_NOM]	= VDD_GPU_DRA7_NOM,
	.gpu.value[OPP_OD]	= VDD_GPU_DRA7_OD,
	.gpu.value[OPP_HIGH]	= VDD_GPU_DRA7_HIGH,
	.gpu.efuse.reg[OPP_NOM]	= STD_FUSE_OPP_VMIN_GPU_NOM,
	.gpu.efuse.reg[OPP_OD]	= STD_FUSE_OPP_VMIN_GPU_OD,
	.gpu.efuse.reg[OPP_HIGH]	= STD_FUSE_OPP_VMIN_GPU_HIGH,
	.gpu.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.gpu.addr		= TPS659038_REG_ADDR_SMPS45,
	.gpu.pmic		= &tps659038,
	.gpu.abb_tx_done_mask	= OMAP_ABB_GPU_TXDONE_MASK,

	.core.value[OPP_NOM]	= VDD_CORE_DRA7_NOM,
	.core.efuse.reg[OPP_NOM]	= STD_FUSE_OPP_VMIN_CORE_NOM,
	.core.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.core.addr		= TPS659038_REG_ADDR_SMPS6,
	.core.pmic		= &tps659038,

	.iva.value[OPP_NOM]	= VDD_IVA_DRA7_NOM,
	.iva.value[OPP_OD]	= VDD_IVA_DRA7_OD,
	.iva.value[OPP_HIGH]	= VDD_IVA_DRA7_HIGH,
	.iva.efuse.reg[OPP_NOM]	= STD_FUSE_OPP_VMIN_IVA_NOM,
	.iva.efuse.reg[OPP_OD]	= STD_FUSE_OPP_VMIN_IVA_OD,
	.iva.efuse.reg[OPP_HIGH]	= STD_FUSE_OPP_VMIN_IVA_HIGH,
	.iva.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.iva.addr		= TPS659038_REG_ADDR_SMPS45,
	.iva.pmic		= &tps659038,
	.iva.abb_tx_done_mask	= OMAP_ABB_IVA_TXDONE_MASK,
};

int get_voltrail_opp(int rail_offset)
{
	int opp;

	switch (rail_offset) {
	case VOLT_MPU:
		opp = DRA7_MPU_OPP;
		break;
	case VOLT_CORE:
		opp = DRA7_CORE_OPP;
		break;
	case VOLT_GPU:
		opp = DRA7_GPU_OPP;
		break;
	case VOLT_EVE:
		opp = DRA7_DSPEVE_OPP;
		break;
	case VOLT_IVA:
		opp = DRA7_IVA_OPP;
		break;
	default:
		opp = OPP_NOM;
	}

	return opp;
}


#ifdef CONFIG_SPL_BUILD


#define GPIO_SD_SUPPLY	((3-1)*32 + 1)	// gpio3_1

/* Override function to read eeprom information */
void do_board_detect(void)
{
	int rc;
	u8 reg;

	i2c_set_bus_num(0);

	// check if UART should be turned off
	rc = i2c_read(CONFIG_EEPROM_CHIP_ADDRESS, 128, 2, &reg, 1);
	if (rc == 0 && reg == 'u')
	{
		struct pad_conf_entry disable_uart[] = {
			{UART2_CTSN, (M15 | PIN_INPUT_PULLUP)},
			{UART2_RTSN, (M15 | PIN_INPUT_PULLUP)},
		};
		do_set_mux32(dra7xx_ctrl.control_padconf_core_base, disable_uart, ARRAY_SIZE(disable_uart));
	}

	// Workaround fuer Warmstart-Probleme mit der SD-Karte, welche ohne ein
	// Power-Off nicht mehr in den UHS Modus schaltet.
#if 1
	// Ab Null-Serie: Versorgung der SD Karte kann ausgeschaltet werden
	if (omap_revision() != DRA752_ES1_1)
	{
		// SD I/O-Spannung abschalten:
		palmas_i2c_read_u8(TPS65903X_CHIP_P1, 0x50, &reg);
		reg &= ~1;
		palmas_i2c_write_u8(TPS65903X_CHIP_P1, 0x50, reg);

		// SD Versorgung abschalten:
		gpio_request(GPIO_SD_SUPPLY, "SD_ON");
		gpio_direction_output(GPIO_SD_SUPPLY, 1);

		// Warten, bis Spannung unter 0,5 V gesunken ist
		udelay(300000);

		// Spannungen wieder einschalten
		gpio_set_value(GPIO_SD_SUPPLY, 0);
		reg |= 1;
		palmas_i2c_write_u8(TPS65903X_CHIP_P1, 0x50, reg);
		udelay(10000);
	}
#else
	{
        // PMIC Cold-Reset => Bootloader haengt danach manchmal
        // siehe: https://e2e.ti.com/support/embedded/linux/f/354/t/532312

        /* Read PMIC backup register 0, bit 0 */
        palmas_i2c_read_u8(TPS65903X_CHIP_P1, 0x18, &reg);
        if (reg & 1)
        {
            /* Cold-reset required, reset the flag first */
            reg &= ~1;
            palmas_i2c_write_u8(TPS65903X_CHIP_P1, 0x18, reg);

            /* Restart device by writing a 1 to the SW_RST bit */
            palmas_i2c_write_u8(TPS65903X_CHIP_P1, 0xA0, 3);

            while (1);
        }
        else
        {
            /* next reboot requires a cold-reset, set the flag to 1 */
            reg |= 1;
            palmas_i2c_write_u8(TPS65903X_CHIP_P1, 0x18, reg);
        }
	}
#endif
}

#else	/* CONFIG_SPL_BUILD */

/* Override function to read eeprom information: actual i2c read done by SPL*/
void do_board_detect(void)
{
	snprintf(sysinfo.board_string, SYSINFO_BOARD_NAME_MAX_LEN, "Board: VisionCam XM\n");

}

static void setup_board_eeprom_env(void)
{
	setenv("board_name", "visioncam_xm");
/*	setenv("board_rev", "unknown");
	setenv("board_serial", "unknown");*/
}

#endif	/* CONFIG_SPL_BUILD */

void vcores_init(void)
{
	*omap_vcores = &beagle_x15_volts;
}

void hw_data_init(void)
{
	*prcm = &dra7xx_prcm;
	*dplls_data = &dra7xx_dplls;
	*ctrl = &dra7xx_ctrl;
}

int board_init(void)
{
	gd->bd->bi_boot_params = (CONFIG_SYS_SDRAM_BASE + 0x100);

	return 0;
}

/* RTC: fix problem with 32-Bit userspace programs after 2038.
 * see also: https://github.com/systemd/systemd/issues/1143 */
void fixup_rtc(void)
{
	static const char * const weekdays[] = {
		"Sun", "Mon", "Tues", "Wednes", "Thurs", "Fri", "Satur",
	};
	struct rtc_time tm;
	int res;
	int old_bus;

	old_bus = i2c_get_bus_num();
	i2c_set_bus_num(CONFIG_SYS_RTC_BUS_NUM);

	res = rtc_get(&tm);
	if (res) {
		puts("fixup_rtc(): Get date failed!\n");
	}
	else
	{
		printf ("RTC:   Date: %4d-%02d-%02d (%sday)    Time: %2d:%02d:%02d\n",
			tm.tm_year, tm.tm_mon, tm.tm_mday,
			(tm.tm_wday<0 || tm.tm_wday>6) ?
				"unknown " : weekdays[tm.tm_wday],
			tm.tm_hour, tm.tm_min, tm.tm_sec);

		if (tm.tm_year == 2036)
			puts("  Warning: After Jan-01-2037 the RTC will be reset to Jan-01-2037 to avoid problems with 32-bit userspace programs!\n");
		else if (tm.tm_year >= 2037)
		{
			puts("  Warning: Setting date to Jan-01-2037 to avoid problems with 32-bit userspace programs!\n");
			tm.tm_year = 2037;
			tm.tm_mon = 1;
			tm.tm_mday = 1;
			tm.tm_wday = 3; // Thrusday
			res = rtc_set(&tm);
			if (res) {
				puts("fixup_rtc(): Get date failed!\n");
			}
		}
	}

	i2c_set_bus_num(old_bus);
}

u32 optimize_vcore_voltage(struct volts const *v);

#ifndef CONFIG_SPL_BUILD

static void board_set_ethaddr(void)
{
	uint8_t mac_addr[6];
	uint32_t mac_hi, mac_lo;

	/* try reading mac address from efuse */
	mac_lo = readl((*ctrl)->control_core_mac_id_0_lo);
	mac_hi = readl((*ctrl)->control_core_mac_id_0_hi);
	mac_addr[0] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = mac_hi & 0xFF;
	mac_addr[3] = (mac_lo & 0xFF0000) >> 16;
	mac_addr[4] = (mac_lo & 0xFF00) >> 8;
	mac_addr[5] = mac_lo & 0xFF;

	if (is_valid_ethaddr(mac_addr))
		eth_setenv_enetaddr("ethaddr", mac_addr);
}

int board_phy_config(struct phy_device *phydev)
{
	gpio_request(GPIO_NOT100MBIT, "not100MBit");
	if (gpio_get_value(GPIO_NOT100MBIT) == 0)
	{
		printf("100Mbps connector detected, limiting PHY to 100Mbps - ");

		phy_write(phydev, 1, 9, 0);
		phy_write(phydev, 1, 0, 0x1340);
	}

	return 0;
}


int board_late_init(void)
{
	u8 reg;

	setup_board_eeprom_env();

	// Beim Runterfahren wird DEV_CTRL.DEV_ON auf 0 gesetzt, POWERHOLD
	// muss dazu ignoriert werden (hat hoehere Prioritaet):

	// DEV_CTRL.DEV_ON = 1
	palmas_i2c_write_u8(TPS65903X_CHIP_P1, 0xA0, 0x1);

	// GPIO7: ignore POWERHOLD
	palmas_i2c_read_u8(TPS65903X_CHIP_P1, 0xFB, &reg);
	reg &= ~0x20;
	palmas_i2c_write_u8(TPS65903X_CHIP_P1, 0xFB, reg);

/*	printf("core: %u mv => %u mv\n", (*omap_vcores)->core.value, optimize_vcore_voltage(&(*omap_vcores)->core));
	printf("mpu: %u mv => %u mv\n", (*omap_vcores)->mpu.value, optimize_vcore_voltage(&(*omap_vcores)->mpu));
	printf("eve: %u mv => %u mv\n", (*omap_vcores)->eve.value, optimize_vcore_voltage(&(*omap_vcores)->eve));
	printf("gpu: %u mv => %u mv\n", (*omap_vcores)->gpu.value, optimize_vcore_voltage(&(*omap_vcores)->gpu));
	printf("iva: %u mv => %u mv\n", (*omap_vcores)->iva.value, optimize_vcore_voltage(&(*omap_vcores)->iva));*/

	// MAC address: always use efuse, ignore environment variable 'ethaddr'
	board_set_ethaddr();
	fixup_rtc();

	altera_power_on();
	
	return 0;
}

#endif

void set_muxconf_regs(void)
{
	do_set_mux32((*ctrl)->control_padconf_core_base,
		     early_padconf, ARRAY_SIZE(early_padconf));
}

#ifdef CONFIG_IODELAY_RECALIBRATION
void recalibrate_iodelay(void)
{
	const struct pad_conf_entry *pconf;
	const struct iodelay_cfg_entry *iod;
	int pconf_sz, iod_sz;
	int ret;

	pconf = core_padconf_array_visioncam_xm;
	pconf_sz = ARRAY_SIZE(core_padconf_array_visioncam_xm);
	if (omap_revision() == DRA752_ES1_1)
	{
		iod = iodelay_cfg_array_vcxm_sr1_1;
		iod_sz = ARRAY_SIZE(iodelay_cfg_array_vcxm_sr1_1);
	}
	else
	{	// SR 2.0
		iod = iodelay_cfg_array_vcxm_sr2_0;
		iod_sz = ARRAY_SIZE(iodelay_cfg_array_vcxm_sr2_0);
	}

	/* Setup I/O isolation */
	ret = __recalibrate_iodelay_start();
	if (ret)
		goto err;

	/* Do the muxing here */
	do_set_mux32((*ctrl)->control_padconf_core_base, pconf, pconf_sz);

	/* Setup IOdelay configuration */
	ret = do_set_iodelay((*ctrl)->iodelay_config_base, iod, iod_sz);
err:
	/* Closeup.. remove isolation */
	__recalibrate_iodelay_end(ret);
}
#endif

#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_GENERIC_MMC)
int board_mmc_init(bd_t *bis)
{
	omap_mmc_init(0, 0, 0, -1, -1);
	omap_mmc_init(1, 0, 0, -1, -1);
	return 0;
}
#endif

#ifdef CONFIG_OMAP_HSMMC
int platform_fixup_disable_uhs_mode(void)
{
/*	volatile unsigned int *mmc1_capa = (volatile unsigned int *)0x4809C240;
	printf("platform_fixup_disable_uhs_mode(): mmc1_capa=0x%08x\n", *mmc1_capa);
	*mmc1_capa &= ~(1<<26);

	// Silicon Advisory i843: max. 96 MHz
	printf("platform_fixup_disable_uhs_mode(): %d\n", omap_revision() == DRA752_ES1_1);
	return omap_revision() == DRA752_ES1_1;*/
	return 0;
}
#endif

#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_OS_BOOT)
int spl_start_uboot(void)
{
	/* break into full u-boot on 'c' */
	if (serial_tstc() && serial_getc() == 'c')
		return 1;

#ifdef CONFIG_SPL_ENV_SUPPORT
	env_init();
	env_relocate_spec();
	if (getenv_yesno("boot_os") != 1)
		return 1;
#endif

	return 0;
}
#endif


int board_early_init_f(void)
{
	return 0;
}

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	if (/*board_is_vcxm() && */!strcmp(name, "am572x-visioncam-xm"))
		return 0;
/*	else if (board_is_x15() && !strcmp(name, "am57xx-beagle-x15"))
		return 0;
	else if (board_is_am572x_evm() && !strcmp(name, "am57xx-beagle-x15"))
		return 0;
	else if (board_is_am572x_idk() && !strcmp(name, "am572x-idk"))
		return 0;
	else if (board_is_am571x_idk() && !strcmp(name, "am571x-idk"))
		return 0;*/
	else
		return -1;
}
#endif

static void ft_set_thermal_trip(void *fdt, char *path, int temp)
{
	int ret;
	int offs = fdt_path_offset(fdt, path);

	if (offs < 0)
		printf("Node %s not found.\n", path);
	else {
		u32 value = cpu_to_fdt32(85000);
		ret = fdt_setprop(fdt, offs, "temperature", &value, sizeof(value));
		if (ret < 0)
			printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
	}
}

static int ft_mmc_fixup(void *fdt, bd_t *bd)
{
	const char *path;
	int offs;
	int ret;

	if (omap_revision() == DRA752_ES1_1)
	{
		printf("Detected Silicon Revision 1.1,\n");

		path = "/ocp/mmc@4809c000";
		offs = fdt_path_offset(fdt, path);
		if (offs < 0)
			printf("Node %s not found.\n", path);
		else {
			printf("  => MMC: disabling UHS-104 mode in the Device Tree (advisory i843)\n");
			fdt_delprop(fdt, offs, "sd-uhs-sdr104");
			printf("  => MMC: updating pad delays in the Device Tree\n");
			const char data[] = "default\0hs\0sdr12\0sdr25\0sdr50\0ddr50\0sdr104\0ddr50-rev20\0sdr104-rev20";
			ret = fdt_setprop(fdt, offs, "pinctrl-names", data, sizeof(data));
			if (ret < 0) {
				printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
			}
		}
	}

	if ((readl((u32 *) (*ctrl)->control_std_fuse_die_id_2) & (1<<18)) == 0) {
		printf("Detected commercial device, updating thermal trip points\n");
		ft_set_thermal_trip(fdt, "/thermal-zones/cpu_thermal/trips/cpu_alert", 85000);
		ft_set_thermal_trip(fdt, "/thermal-zones/cpu_thermal/trips/cpu_crit", 90000);
		ft_set_thermal_trip(fdt, "/thermal-zones/gpu_thermal/trips/gpu_crit", 90000);
		ft_set_thermal_trip(fdt, "/thermal-zones/core_thermal/trips/core_crit", 90000);
		ft_set_thermal_trip(fdt, "/thermal-zones/dspeve_thermal/trips/dspeve_crit", 90000);
		ft_set_thermal_trip(fdt, "/thermal-zones/iva_thermal/trips/iva_crit", 90000);
	}

	gpio_request(GPIO_NOT100MBIT, "not100MBit");
	if (gpio_get_value(GPIO_NOT100MBIT) == 0)
	{
		printf("NET: Detected 100Mbps connector, limiting PHY to 100Mbps in the Device Tree\n");

		path = "/ocp/ethernet@48484000/mdio@48485000/phy@1";
		offs = fdt_path_offset(fdt, path);
		if (offs < 0) {
			printf("Node %s not found.\n", path);
			return -1;
		}

		u32 value = cpu_to_fdt32(100);
		ret = fdt_setprop(fdt, offs, "max-speed", &value, sizeof(value));
		if (ret < 0) {
			printf("libfdt fdt_setprop(): %s\n", fdt_strerror(ret));
			return -1;
		}
	}

	return 0;
}
#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
int ft_board_setup(void *blob, bd_t *bd)
{
	ft_cpu_setup(blob, bd);

	ft_mmc_fixup(blob, bd);

	return 0;
}
#endif
