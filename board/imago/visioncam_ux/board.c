/*
 * board.c
 *
 * Board functions for IMAGO VisionCam UX
 *
 * Copyright (C) IMAGO Technologies GmbH - http://www.imago-technologies.com/
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <i2c.h>
#include <linux/errno.h>
#include <spl.h>
#include <usb.h>
#include <asm/omap_sec_common.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mux.h>
#include <asm/arch/ddr_defs.h>
#include <asm/arch/gpio.h>
#include <asm/emif.h>
#include "board.h"
#include <power/pmic.h>
#include <power/tps65218.h>
#include <miiphy.h>
#include <cpsw.h>
#include <linux/usb/gadget.h>
#include <dwc3-uboot.h>
#include <dwc3-omap-uboot.h>

DECLARE_GLOBAL_DATA_PTR;

static struct ctrl_dev *cdev = (struct ctrl_dev *)CTRL_DEVICE_BASE;

#ifndef CONFIG_SKIP_LOWLEVEL_INIT

const struct dpll_params dpll_mpu[NUM_CRYSTAL_FREQ][NUM_OPPS] = {
	{	/* 19.2 MHz */
		{125, 3, 2, -1, -1, -1, -1},	/* OPP 50 */
		{-1, -1, -1, -1, -1, -1, -1},	/* OPP RESERVED	*/
		{125, 3, 1, -1, -1, -1, -1},	/* OPP 100 */
		{150, 3, 1, -1, -1, -1, -1},	/* OPP 120 */
		{125, 2, 1, -1, -1, -1, -1},	/* OPP TB */
		{625, 11, 1, -1, -1, -1, -1}	/* OPP NT */
	},
	{	/* 24 MHz */
		{300, 23, 1, -1, -1, -1, -1},	/* OPP 50 */
		{-1, -1, -1, -1, -1, -1, -1},	/* OPP RESERVED	*/
		{600, 23, 1, -1, -1, -1, -1},	/* OPP 100 */
		{720, 23, 1, -1, -1, -1, -1},	/* OPP 120 */
		{800, 23, 1, -1, -1, -1, -1},	/* OPP TB */
		{1000, 23, 1, -1, -1, -1, -1}	/* OPP NT */
	},
	{	/* 25 MHz */
		{300, 24, 1, -1, -1, -1, -1},	/* OPP 50 */
		{-1, -1, -1, -1, -1, -1, -1},	/* OPP RESERVED	*/
		{600, 24, 1, -1, -1, -1, -1},	/* OPP 100 */
		{720, 24, 1, -1, -1, -1, -1},	/* OPP 120 */
		{800, 24, 1, -1, -1, -1, -1},	/* OPP TB */
		{1000, 24, 1, -1, -1, -1, -1}	/* OPP NT */
	},
	{	/* 26 MHz */
		{300, 25, 1, -1, -1, -1, -1},	/* OPP 50 */
		{-1, -1, -1, -1, -1, -1, -1},	/* OPP RESERVED	*/
		{600, 25, 1, -1, -1, -1, -1},	/* OPP 100 */
		{720, 25, 1, -1, -1, -1, -1},	/* OPP 120 */
		{800, 25, 1, -1, -1, -1, -1},	/* OPP TB */
		{1000, 25, 1, -1, -1, -1, -1}	/* OPP NT */
	},
};

const struct dpll_params dpll_core[NUM_CRYSTAL_FREQ] = {
		{625, 11, -1, -1, 10, 8, 4},	/* 19.2 MHz */
		{1000, 23, -1, -1, 10, 8, 4},	/* 24 MHz */
		{1000, 24, -1, -1, 10, 8, 4},	/* 25 MHz */
		{1000, 25, -1, -1, 10, 8, 4}	/* 26 MHz */
};

const struct dpll_params dpll_per[NUM_CRYSTAL_FREQ] = {
		{400, 7, 5, -1, -1, -1, -1},	/* 19.2 MHz */
		{400, 9, 5, -1, -1, -1, -1},	/* 24 MHz */
		{384, 9, 5, -1, -1, -1, -1},	/* 25 MHz */
		{480, 12, 5, -1, -1, -1, -1}	/* 26 MHz */
};

const struct dpll_params epos_evm_dpll_ddr[NUM_CRYSTAL_FREQ] = {
		{665, 47, 1, -1, 4, -1, -1}, /*19.2*/
		{133, 11, 1, -1, 4, -1, -1}, /* 24 MHz */
		{266, 24, 1, -1, 4, -1, -1}, /* 25 MHz */
		{133, 12, 1, -1, 4, -1, -1}  /* 26 MHz */
};

static const u32 ext_phy_ctrl_const_base_lpddr2[] = {
	0x00500050,
	0x00350035,
	0x00350035,
	0x00350035,
	0x00350035,
	0x00350035,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000000,
	0x40001000,
	0x08102040
};

const struct ctrl_ioregs ioregs_lpddr2 = {
	.cm0ioctl		= LPDDR2_ADDRCTRL_IOCTRL_VALUE,
	.cm1ioctl		= LPDDR2_ADDRCTRL_WD0_IOCTRL_VALUE,
	.cm2ioctl		= LPDDR2_ADDRCTRL_WD1_IOCTRL_VALUE,
	.dt0ioctl		= LPDDR2_DATA0_IOCTRL_VALUE,
	.dt1ioctl		= LPDDR2_DATA0_IOCTRL_VALUE,
	.dt2ioctrl		= LPDDR2_DATA0_IOCTRL_VALUE,
	.dt3ioctrl		= LPDDR2_DATA0_IOCTRL_VALUE,
	.emif_sdram_config_ext	= 0x1,
};

const struct emif_regs emif_regs_lpddr2 = {
	.sdram_config			= 0x808012B2,
	.ref_ctrl			= 0x0000040D,		// t_REFI=3.9 us
	.sdram_tim1			= 0xEA86B411,
	.sdram_tim2			= 0x1025094A,
	.sdram_tim3			= 0x0F6BA22F,
	.read_idle_ctrl			= 0x00050000,
	.zq_config			= 0x50074BE4,
	.temp_alert_config		= 0x0,
	.emif_rd_wr_lvl_rmp_win		= 0x0,
	.emif_rd_wr_lvl_rmp_ctl		= 0x0,
	.emif_rd_wr_lvl_ctl		= 0x0,
	.emif_ddr_phy_ctlr_1		= 0x0E284006,
	.emif_rd_wr_exec_thresh		= 0x80000405,
	.emif_ddr_ext_phy_ctrl_1	= 0x04010040,
	.emif_ddr_ext_phy_ctrl_2	= 0x00500050,
	.emif_ddr_ext_phy_ctrl_3	= 0x00500050,
	.emif_ddr_ext_phy_ctrl_4	= 0x00500050,
	.emif_ddr_ext_phy_ctrl_5	= 0x00500050,
	.emif_prio_class_serv_map	= 0x80000001,
	.emif_connect_id_serv_1_map	= 0x80000094,
	.emif_connect_id_serv_2_map	= 0x00000000,
	.emif_cos_config			= 0x000FFFFF
};

void emif_get_ext_phy_ctrl_const_regs(const u32 **regs, u32 *size)
{
	*regs = ext_phy_ctrl_const_base_lpddr2;
	*size = ARRAY_SIZE(ext_phy_ctrl_const_base_lpddr2);

	return;
}

const struct dpll_params *get_dpll_ddr_params(void)
{
	int ind = get_sys_clk_index();

	return &epos_evm_dpll_ddr[ind];
}


/*
 * get_opp_offset:
 * Returns the index for safest OPP of the device to boot.
 * max_off:	Index of the MAX OPP in DEV ATTRIBUTE register.
 * min_off:	Index of the MIN OPP in DEV ATTRIBUTE register.
 * This data is read from dev_attribute register which is e-fused.
 * A'1' in bit indicates OPP disabled and not available, a '0' indicates
 * OPP available. Lowest OPP starts with min_off. So returning the
 * bit with rightmost '0'.
 */
static int get_opp_offset(int max_off, int min_off)
{
	struct ctrl_stat *ctrl = (struct ctrl_stat *)CTRL_BASE;
	int opp, offset, i;

	/* Bits 0:11 are defined to be the MPU_MAX_FREQ */
	opp = readl(&ctrl->dev_attr) & ~0xFFFFF000;

	for (i = max_off; i >= min_off; i--) {
		offset = opp & (1 << i);
		if (!offset)
			return i;
	}

	return min_off;
}

const struct dpll_params *get_dpll_mpu_params(void)
{
	int opp = get_opp_offset(DEV_ATTR_MAX_OFFSET, DEV_ATTR_MIN_OFFSET);
	u32 ind = get_sys_clk_index();

	return &dpll_mpu[ind][opp];
}

const struct dpll_params *get_dpll_core_params(void)
{
	int ind = get_sys_clk_index();

	return &dpll_core[ind];
}

const struct dpll_params *get_dpll_per_params(void)
{
	int ind = get_sys_clk_index();

	return &dpll_per[ind];
}

void scale_vcores_generic(u32 m)
{
	int mpu_vdd;

	if (i2c_probe(TPS65218_CHIP_PM))
		return;

	switch (m) {
	case 1000:
		mpu_vdd = TPS65218_DCDC_VOLT_SEL_1330MV;
		break;
	case 800:
		mpu_vdd = TPS65218_DCDC_VOLT_SEL_1260MV;
		break;
	case 720:
		mpu_vdd = TPS65218_DCDC_VOLT_SEL_1200MV;
		break;
	case 600:
		mpu_vdd = TPS65218_DCDC_VOLT_SEL_1100MV;
		break;
	case 300:
		mpu_vdd = TPS65218_DCDC_VOLT_SEL_0950MV;
		break;
	default:
		puts("Unknown MPU clock, not scaling\n");
		return;
	}

	/* Set DCDC1 (CORE) voltage to 1.1V */
	if (tps65218_voltage_update(TPS65218_DCDC1,
				    TPS65218_DCDC_VOLT_SEL_1100MV)) {
		printf("%s failure\n", __func__);
		return;
	}

	/* Set DCDC2 (MPU) voltage */
	if (tps65218_voltage_update(TPS65218_DCDC2, mpu_vdd)) {
		printf("%s failure\n", __func__);
		return;
	}

	/* Set DCDC3 (DDR) voltage */
	if (tps65218_voltage_update(TPS65218_DCDC3,
	    0x0c/*TPS65218_DCDC3_VOLT_SEL_1350MV*/)) {
		printf("%s failure\n", __func__);
		return;
	}
}

void gpi2c_init(void)
{
	/* When needed to be invoked prior to BSS initialization */
	static bool first_time = true;
/*	struct cm_perpll *const cmper = (struct cm_perpll *)CM_PER;
	u32 *const clk_domains[] = {0};
    u32 *const clk_modules_explicit_en[] = {&cmper->i2c2clkctrl, 0};*/

	if (first_time) {
//	    do_enable_clocks(clk_domains, clk_modules_explicit_en, 1);
		enable_i2c0_pin_mux();
		i2c_set_bus_num(0);
//		i2c_init(CONFIG_SYS_OMAP24_I2C_SPEED,
//			 CONFIG_SYS_OMAP24_I2C_SLAVE);
		first_time = false;
	}
}

void scale_vcores(void)
{
	const struct dpll_params *mpu_params;

	/* Ensure I2C is initialized for PMIC configuration */
	gpi2c_init();

	/* Get the frequency */
	mpu_params = get_dpll_mpu_params();

	scale_vcores_generic(mpu_params->m);
}

void set_uart_mux_conf(void)
{
	enable_uart0_pin_mux();
}

void set_mux_conf_regs(void)
{
	enable_board_pin_mux();
}

void sdram_init(void)
{
	/*
	 * 512 MB LPDDR2 connected to EMIF.
	 */
	config_ddr(0, &ioregs_lpddr2, NULL, NULL, &emif_regs_lpddr2, 0);
}
#endif

/* setup board specific PMIC */
int power_init_board(void)
{
    struct pmic *p;

    power_tps65218_init(0);

    p = pmic_get("TPS65218_PMIC");
    if (p && !pmic_probe(p))
        puts("PMIC:  TPS65218\n");

    // DCDC4: Force PWM mode
    tps65218_reg_write(TPS65218_PROT_LEVEL_1, TPS65218_DCDC4, 0, 0x80);

    // GPIO3: set output to high (Linux gpio-powerdown)
    tps65218_reg_write(TPS65218_PROT_LEVEL_1, TPS65218_ENABLE2, 0x40, 0x40);

	return 0;
}

int board_init(void)
{
	struct l3f_cfg_bwlimiter *bwlimiter = (struct l3f_cfg_bwlimiter *)L3F_CFG_BWLIMITER;
	u32 mreqprio_0, mreqprio_1, modena_init0_bw_fractional,
	    modena_init0_bw_integer, modena_init0_watermark_0;

	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;
	gpmc_init();

	/* Clear all important bits for DSS errata that may need to be tweaked*/
	mreqprio_0 = readl(&cdev->mreqprio_0) & MREQPRIO_0_SAB_INIT1_MASK &
	                   MREQPRIO_0_SAB_INIT0_MASK;

	mreqprio_1 = readl(&cdev->mreqprio_1) & MREQPRIO_1_DSS_MASK;

	modena_init0_bw_fractional = readl(&bwlimiter->modena_init0_bw_fractional) &
	                                   BW_LIMITER_BW_FRAC_MASK;

	modena_init0_bw_integer = readl(&bwlimiter->modena_init0_bw_integer) &
	                                BW_LIMITER_BW_INT_MASK;

	modena_init0_watermark_0 = readl(&bwlimiter->modena_init0_watermark_0) &
	                                 BW_LIMITER_BW_WATERMARK_MASK;

	/* Setting MReq Priority of the DSS*/
	mreqprio_0 |= 0x77;

	/*
	 * Set L3 Fast Configuration Register
	 * Limiting bandwith for ARM core to 700 MBPS
	 */
	modena_init0_bw_fractional |= 0x10;
	modena_init0_bw_integer |= 0x3;

	writel(mreqprio_0, &cdev->mreqprio_0);
	writel(mreqprio_1, &cdev->mreqprio_1);

	writel(modena_init0_bw_fractional, &bwlimiter->modena_init0_bw_fractional);
	writel(modena_init0_bw_integer, &bwlimiter->modena_init0_bw_integer);
	writel(modena_init0_watermark_0, &bwlimiter->modena_init0_watermark_0);

	return 0;
}

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	set_board_info_env(NULL);

	/*
	 * Default FIT boot on HS devices. Non FIT images are not allowed
	 * on HS devices.
	 */
	if (get_device_type() == HS_DEVICE)
		setenv("boot_fit", "1");
#endif
	return 0;
}
#endif

/**
 * Loeschen der MAC Adresse in der Umgebung nach initr_net(),
 * damit die MAC Adresse nach 'saveenv' nicht auf der SD-Karte landet.
 */
int last_stage_init(void)
{
    int repeatable;
    unsigned long ticks;
    char *argv[] = {"setenv", "ethaddr"};

    cmd_process(0, 2, argv, &repeatable, &ticks);

    return 0;
}

#ifndef CONFIG_DM_ETH
#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_SPL_ETH_SUPPORT) && defined(CONFIG_SPL_BUILD))
static void cpsw_control(int enabled)
{
	/* Additional controls can be added here */
	return;
}

static struct cpsw_slave_data cpsw_slaves[] = {
	{
		.slave_reg_ofs	= 0x208,
		.sliver_reg_ofs	= 0xd80,
		.phy_addr	= 16,
	},
	{
		.slave_reg_ofs	= 0x308,
		.sliver_reg_ofs	= 0xdc0,
		.phy_addr	= 1,
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
#endif


/*
 * This function will:
 * Read the eFuse for MAC addresses, and set ethaddr/eth1addr/usbnet_devaddr
 * in the environment
 * Perform fixups to the PHY present on certain boards.  We only need this
 * function in:
 * - SPL with either CPSW or USB ethernet support
 * - Full U-Boot, with either CPSW or USB ethernet
 * Build in only these cases to avoid warnings about unused variables
 * when we build an SPL that has neither option but full U-Boot will.
 */
#if ((defined(CONFIG_SPL_ETH_SUPPORT) || \
	defined(CONFIG_SPL_USBETH_SUPPORT)) && \
	defined(CONFIG_SPL_BUILD)) || \
	((defined(CONFIG_DRIVER_TI_CPSW) || \
	  defined(CONFIG_USB_ETHER)) && !defined(CONFIG_SPL_BUILD))
int board_eth_init(bd_t *bis)
{
	int rv;
	uint8_t mac_addr[6];
	uint32_t mac_hi, mac_lo;

	/* try reading mac address from efuse */
	mac_lo = readl(&cdev->macid0l);
	mac_hi = readl(&cdev->macid0h);
	mac_addr[0] = mac_hi & 0xFF;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	mac_addr[4] = mac_lo & 0xFF;
	mac_addr[5] = (mac_lo & 0xFF00) >> 8;

#if 0
	// MAC Adresse in der Umgebung immer (er)setzen
    eth_setenv_enetaddr("ethaddr", mac_addr);
#else
	if (!getenv("ethaddr")) {
//		puts("<ethaddr> not set. Validating first E-fuse MAC, ");
		if (is_valid_ethaddr(mac_addr))
			eth_setenv_enetaddr("ethaddr", mac_addr);
	}
	else
        puts("Using MAC address <ethaddr> from environment, ");
#endif

	writel(RMII_MODE_ENABLE | RMII_CHIPCKL_ENABLE, &cdev->miisel);
	cpsw_slaves[0].phy_if = PHY_INTERFACE_MODE_RMII;
	cpsw_slaves[0].phy_addr = 2;

	rv = cpsw_register(&cpsw_data);
	if (rv < 0) {
		printf("Error %d registering CPSW switch\n", rv);
		return rv;
	}

	return rv;
}
#endif
#endif

#if defined(CONFIG_OF_LIBFDT) && defined(CONFIG_OF_BOARD_SETUP)
int ft_board_setup(void *blob, bd_t *bd)
{
	ft_cpu_setup(blob, bd);

	return 0;
}
#endif


#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	if (!strcmp(name, "am437x-visioncam-ux"))
		return 0;
	else
		return -1;
}
#endif

#ifdef CONFIG_TI_SECURE_DEVICE
void board_fit_image_post_process(void **p_image, size_t *p_size)
{
	secure_boot_verify_image(p_image, p_size);
}

void board_tee_image_process(ulong tee_image, size_t tee_size)
{
	secure_tee_install((u32)tee_image);
}

U_BOOT_FIT_LOADABLE_HANDLER(IH_TYPE_TEE, board_tee_image_process);
#endif
