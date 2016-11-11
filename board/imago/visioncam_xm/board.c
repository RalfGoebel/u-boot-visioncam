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

//#include "board_detect.h"
#include "altera.h"
#include "mux_data.h"

/*#define board_is_x15()		board_ti_is("BBRDX15_")
#define board_is_x15_revb1()	(board_ti_is("BBRDX15_") && \
				 (strncmp("B.10", board_ti_get_rev(), 3) <= 0))
#define board_is_am572x_evm()	board_ti_is("AM572PM_")
#define board_is_am572x_evm_reva3()	\
				(board_ti_is("AM572PM_") && \
				 (strncmp("A.30", board_ti_get_rev(), 3) <= 0))
#define board_is_vcxm()	board_ti_is("VCXM")
#define board_is_am572x_idk()	board_ti_is("AM572IDK")
#define board_is_am571x_idk()	board_ti_is("AM571IDK")*/

#if 0//def CONFIG_DRIVER_TI_CPSW
#include <cpsw.h>
#endif

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
	.mpu.value		= VDD_MPU_DRA7,
	.mpu.efuse.reg		= STD_FUSE_OPP_VMIN_MPU,
	.mpu.efuse.reg_bits     = DRA752_EFUSE_REGBITS,
	.mpu.addr		= TPS659038_REG_ADDR_SMPS12,
	.mpu.pmic		= &tps659038,
	.mpu.abb_tx_done_mask = OMAP_ABB_MPU_TXDONE_MASK,

	.eve.value		= VDD_EVE_DRA7,
	.eve.efuse.reg		= STD_FUSE_OPP_VMIN_DSPEVE,
	.eve.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.eve.addr		= TPS659038_REG_ADDR_SMPS45,
	.eve.pmic		= &tps659038,
	.eve.abb_tx_done_mask	= OMAP_ABB_EVE_TXDONE_MASK,

	.gpu.value		= VDD_GPU_DRA7,
	.gpu.efuse.reg		= STD_FUSE_OPP_VMIN_GPU,
	.gpu.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.gpu.addr		= TPS659038_REG_ADDR_SMPS45,
	.gpu.pmic		= &tps659038,
	.gpu.abb_tx_done_mask	= OMAP_ABB_GPU_TXDONE_MASK,

	.core.value		= VDD_CORE_DRA7,
	.core.efuse.reg		= STD_FUSE_OPP_VMIN_CORE,
	.core.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.core.addr		= TPS659038_REG_ADDR_SMPS6,
	.core.pmic		= &tps659038,

	.iva.value		= VDD_IVA_DRA7,
	.iva.efuse.reg		= STD_FUSE_OPP_VMIN_IVA,
	.iva.efuse.reg_bits	= DRA752_EFUSE_REGBITS,
	.iva.addr		= TPS659038_REG_ADDR_SMPS45,
	.iva.pmic		= &tps659038,
	.iva.abb_tx_done_mask	= OMAP_ABB_IVA_TXDONE_MASK,
};


#ifdef CONFIG_SPL_BUILD
/* No env to setup for SPL */
static inline void setup_board_eeprom_env(void) { }

/* Override function to read eeprom information */
void do_board_detect(void)
{
	int rc;
	u8 reg;

	i2c_set_bus_num(0);

//	i2c_probe(TPS65903X_CHIP_P1);

/*	rc = ti_i2c_eeprom_am_get(CONFIG_EEPROM_BUS_ADDRESS,
				  CONFIG_EEPROM_CHIP_ADDRESS);
	if (rc)
		printf("ti_i2c_eeprom_init failed %d\n", rc);*/

	/* Read PMIC backup register 0, bit 0 */
	palmas_i2c_read_u8(TPS65903X_CHIP_P1, 0x18, &reg);
	if (reg & 1)
	{
		/* Cold-reset required, reset the flag first */
		reg &= ~1;
		palmas_i2c_write_u8(TPS65903X_CHIP_P1, 0x18, reg);

		/* Restart device by writing a 1 to the SW_RST bit */
		palmas_i2c_write_u8(TPS65903X_CHIP_P1, 0xA0, 3);
	}
	else
	{
		/* next reboot requires a cold-reset, set the flag to 1 */
		reg |= 1;
		palmas_i2c_write_u8(TPS65903X_CHIP_P1, 0x18, reg);
	}
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

void vcores_update(void)
{
/*	if (board_is_am572x_idk())
		*omap_vcores = &am572x_idk_volts;
	else if (board_is_am571x_idk())
		*omap_vcores = &am571x_idk_volts;*/
}

void hw_data_init(void)
{
	*prcm = &dra7xx_prcm;
	*dplls_data = &dra7xx_dplls;
	*omap_vcores = &beagle_x15_volts;
	*ctrl = &dra7xx_ctrl;
}

int board_init(void)
{
	gd->bd->bi_boot_params = (CONFIG_SYS_SDRAM_BASE + 0x100);

	return 0;
}

#if !defined(CONFIG_SPL_BUILD)
static u64 mac_to_u64(u8 mac[6])
{
	int i;
	u64 addr = 0;

	for (i = 0; i < 6; i++) {
		addr <<= 8;
		addr |= mac[i];
	}

	return addr;
}

static void u64_to_mac(u64 addr, u8 mac[6])
{
	mac[5] = addr;
	mac[4] = addr >> 8;
	mac[3] = addr >> 16;
	mac[2] = addr >> 24;
	mac[1] = addr >> 32;
	mac[0] = addr >> 40;
}

#if 0
void board_set_ethaddr(void)
{
	uint8_t mac_addr[6];
	int i;
	u64 mac1, mac2;
	u8 mac_addr1[6], mac_addr2[6];
	int num_macs;
	/*
	 * Export any Ethernet MAC addresses from EEPROM.
	 * On AM57xx the 2 MAC addresses define the address range
	 */
	board_ti_get_eth_mac_addr(0, mac_addr1);
	board_ti_get_eth_mac_addr(1, mac_addr2);

	if (is_valid_ethaddr(mac_addr1) && is_valid_ethaddr(mac_addr2)) {
		mac1 = mac_to_u64(mac_addr1);
		mac2 = mac_to_u64(mac_addr2);

		/* must contain an address range */
		num_macs = mac2 - mac1 + 1;
		/* <= 50 to protect against user programming error */
		if (num_macs > 0 && num_macs <= 50) {
			for (i = 0; i < num_macs; i++) {
				u64_to_mac(mac1 + i, mac_addr);
				if (is_valid_ethaddr(mac_addr)) {
					eth_setenv_enetaddr_by_index("eth",
								     i + 2,
								     mac_addr);
				}
			}
		}
	}
}
#endif

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

#endif

u32 optimize_vcore_voltage(struct volts const *v);

int board_late_init(void)
{
	u8 reg;

	setup_board_eeprom_env();

	// Beim Runterfahren wird DEV_CTRL.DEV_ON auf 0 gesetzt, POWERHOLD muss dazu ignoriert werden:

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

#if !defined(CONFIG_SPL_BUILD)
//	board_set_ethaddr();
	fixup_rtc();
#endif

	altera_power_on();
	
	return 0;
}

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

/*	if (board_is_vcxm()) */{
		pconf = core_padconf_array_visioncam_xm;
		pconf_sz = ARRAY_SIZE(core_padconf_array_visioncam_xm);
		iod = iodelay_cfg_array_vcxm;
		iod_sz = ARRAY_SIZE(iodelay_cfg_array_vcxm);
	}
#if 0
	else if (board_is_am572x_idk()) {
		pconf = core_padconf_array_essential_am572x_idk;
		pconf_sz = ARRAY_SIZE(core_padconf_array_essential_am572x_idk);
		iod = iodelay_cfg_array_am572x_idk;
		iod_sz = ARRAY_SIZE(iodelay_cfg_array_am572x_idk);
	} else if (board_is_am571x_idk()) {
		pconf = core_padconf_array_essential_am571x_idk;
		pconf_sz = ARRAY_SIZE(core_padconf_array_essential_am571x_idk);
		iod = iodelay_cfg_array_am571x_idk;
		iod_sz = ARRAY_SIZE(iodelay_cfg_array_am571x_idk);
	} else {
		/* Common for X15/GPEVM */
		pconf = core_padconf_array_essential_x15;
		pconf_sz = ARRAY_SIZE(core_padconf_array_essential_x15);
		/* There never was an SR1.0 X15.. So.. */
		if (omap_revision() == DRA752_ES1_1) {
			iod = iodelay_cfg_array_x15_sr1_1;
			iod_sz = ARRAY_SIZE(iodelay_cfg_array_x15_sr1_1);
		} else {
			/* Since full production should switch to SR2.0  */
			iod = iodelay_cfg_array_x15_sr2_0;
			iod_sz = ARRAY_SIZE(iodelay_cfg_array_x15_sr2_0);
		}
	}
#endif

	/* Setup I/O isolation */
	ret = __recalibrate_iodelay_start();
	if (ret)
		goto err;

	/* Do the muxing here */
	do_set_mux32((*ctrl)->control_padconf_core_base, pconf, pconf_sz);

#if 0
	/* Now do the weird minor deltas that should be safe */
	if (board_is_x15() || board_is_am572x_evm()) {
		if (board_is_x15_revb1() || board_is_am572x_evm_reva3()) {
			pconf = core_padconf_array_delta_x15_sr2_0;
			pconf_sz = ARRAY_SIZE(core_padconf_array_delta_x15_sr2_0);
		} else {
			pconf = core_padconf_array_delta_x15_sr1_1;
			pconf_sz = ARRAY_SIZE(core_padconf_array_delta_x15_sr1_1);
		}
		do_set_mux32((*ctrl)->control_padconf_core_base, pconf, pconf_sz);
	}
#endif

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


#if 0 //def CONFIG_DRIVER_TI_CPSW

/* Delay value to add to calibrated value */
#define RGMII0_TXCTL_DLY_VAL		((0x3 << 5) + 0x8)
#define RGMII0_TXD0_DLY_VAL		((0x3 << 5) + 0x8)
#define RGMII0_TXD1_DLY_VAL		((0x3 << 5) + 0x2)
#define RGMII0_TXD2_DLY_VAL		((0x4 << 5) + 0x0)
#define RGMII0_TXD3_DLY_VAL		((0x4 << 5) + 0x0)
#define VIN2A_D13_DLY_VAL		((0x3 << 5) + 0x8)
#define VIN2A_D17_DLY_VAL		((0x3 << 5) + 0x8)
#define VIN2A_D16_DLY_VAL		((0x3 << 5) + 0x2)
#define VIN2A_D15_DLY_VAL		((0x4 << 5) + 0x0)
#define VIN2A_D14_DLY_VAL		((0x4 << 5) + 0x0)

static void cpsw_control(int enabled)
{
	/* VTP can be added here */
}

static struct cpsw_slave_data cpsw_slaves[] = {
	{
		.slave_reg_ofs	= 0x208,
		.sliver_reg_ofs	= 0xd80,
		.phy_addr	= 1,
	},
	{
		.slave_reg_ofs	= 0x308,
		.sliver_reg_ofs	= 0xdc0,
		.phy_addr	= 2,
	},
};

static struct cpsw_platform_data cpsw_data = {
	.mdio_base		= CPSW_MDIO_BASE,
	.cpsw_base		= CPSW_BASE,
	.mdio_div		= 0xff,
	.channels		= 8,
	.cpdma_reg_ofs		= 0x800,
	.slaves			= 1,
	.slave_data		= cpsw_slaves,
	.ale_reg_ofs		= 0xd00,
	.ale_entries		= 1024,
	.host_port_reg_ofs	= 0x108,
	.hw_stats_reg_ofs	= 0x900,
	.bd_ram_ofs		= 0x2000,
	.mac_control		= (1 << 5),
	.control		= cpsw_control,
	.host_port_num		= 0,
	.version		= CPSW_CTRL_VERSION_2,
};

int board_eth_init(bd_t *bis)
{
	int ret;
	uint8_t mac_addr[6];
	uint32_t mac_hi, mac_lo;
	uint32_t ctrl_val;

	puts("board_eth_init()\n");
	while (1);

	/* try reading mac address from efuse */
	mac_lo = readl((*ctrl)->control_core_mac_id_0_lo);
	mac_hi = readl((*ctrl)->control_core_mac_id_0_hi);
	mac_addr[0] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = mac_hi & 0xFF;
	mac_addr[3] = (mac_lo & 0xFF0000) >> 16;
	mac_addr[4] = (mac_lo & 0xFF00) >> 8;
	mac_addr[5] = mac_lo & 0xFF;

	if (!getenv("ethaddr")) {
		printf("<ethaddr> not set. Validating first E-fuse MAC\n");

		if (is_valid_ethaddr(mac_addr))
			eth_setenv_enetaddr("ethaddr", mac_addr);
	}

	mac_lo = readl((*ctrl)->control_core_mac_id_1_lo);
	mac_hi = readl((*ctrl)->control_core_mac_id_1_hi);
	mac_addr[0] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = mac_hi & 0xFF;
	mac_addr[3] = (mac_lo & 0xFF0000) >> 16;
	mac_addr[4] = (mac_lo & 0xFF00) >> 8;
	mac_addr[5] = mac_lo & 0xFF;

	if (!getenv("eth1addr")) {
		if (is_valid_ethaddr(mac_addr))
			eth_setenv_enetaddr("eth1addr", mac_addr);
	}

	ctrl_val = readl((*ctrl)->control_core_control_io1) & (~0x33);
	ctrl_val |= 0x22;
	writel(ctrl_val, (*ctrl)->control_core_control_io1);

	/* The phy address for the AM57xx IDK are different than x15 */
/*	if (board_is_am572x_idk() || board_is_am571x_idk()) {
		cpsw_data.slave_data[0].phy_addr = 0;
		cpsw_data.slave_data[1].phy_addr = 1;
	}*/

	ret = cpsw_register(&cpsw_data);
	if (ret < 0)
		printf("Error %d registering CPSW switch\n", ret);

	return ret;
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

#define GPIO_NOT100MBIT	260

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

	gpio_request(GPIO_NOT100MBIT, "not100MBit");	// gpio8_12
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
