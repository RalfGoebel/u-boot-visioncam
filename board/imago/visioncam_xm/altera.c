//#define DEBUG

#include <dm.h>
#include <common.h>
#include <asm/omap_common.h>
#include <asm/arch/clock.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <spi.h>

#include "altera.h"

#define TPS659038_REG_ADDR_SMPS9 0x3B 
#define TPS659038_REG_ADDR_SMPS7_CTRL	0x30
#define TPS659038_REG_ADDR_SMPS9_CTRL	0x38

#if !defined(CONFIG_SPL_BUILD)

static int pmic_set_voltage(u32 vcore_reg, u32 volt_mv)
{
	struct pmic_data *pmic = &tps659038;
	u32 offset_code;
	u32 offset = volt_mv;
	u32 step;

	/* convert to uV for better accuracy in the calculations */
	offset *= 1000;

	if (volt_mv > 1500000) {
		offset -= 2*pmic->base_offset;
		step = 20000;
	}
	else {
		offset -= pmic->base_offset;
		step = 10000;
	}

	offset_code = (offset + step - 1) / step;
	offset_code += pmic->start_code;
	if (volt_mv > 1500000)
		offset_code |= 0x80;

	debug("pmic_set_voltage: volt - %d offset_code - 0x%x\n", volt_mv, offset_code);

	if (pmic->pmic_write(pmic->i2c_slave_addr, vcore_reg, offset_code)) {
		printf("Scaling voltage failed for 0x%x\n", vcore_reg);
		return -1;
	}
		
	return 0;
}

static int pmic_enable_voltage(u32 vcore_reg)
{
	struct pmic_data *pmic = &tps659038;

	// MODE_SLEEP + MODE_ACTIVE => Forced PWM
	if (pmic->pmic_write(pmic->i2c_slave_addr, vcore_reg, 0x5)) {
		printf("Enabling voltage failed for 0x%x\n", vcore_reg);
		return -1;
	}
	
	return 0;
}

int altera_power_on(void)
{
	puts("Power up FPGA supply...\n");
	
	tps659038.pmic_bus_init();
	
	if (pmic_set_voltage(TPS659038_REG_ADDR_SMPS7, 2500) < 0)
		return -1;
	if (pmic_enable_voltage(TPS659038_REG_ADDR_SMPS7_CTRL) < 0)
		return -1;
	if (pmic_set_voltage(TPS659038_REG_ADDR_SMPS9, 1200) < 0)
		return -1;
	if (pmic_enable_voltage(TPS659038_REG_ADDR_SMPS9_CTRL) < 0)
		return -1;

/*	fat_read_file("/VCXS.rbf", void *buf, loff_t offset, loff_t len,
			  loff_t *actread)*/
	
	return 0;
}

DECLARE_GLOBAL_DATA_PTR;

static int do_spi_xfer(int bus, int cs, void *data_out, void *data_in, unsigned int size)
{
	struct spi_slave *slave;
	int ret = 0;

	char name[30], *str;
	struct udevice *dev;

	snprintf(name, sizeof(name), "generic_%d:%d", bus, cs);
	str = strdup(name);
	if (!str)
		return -ENOMEM;
	ret = spi_get_bus_and_cs(bus, cs, 24000000, SPI_MODE_0, "spi_generic_drv",
				 str, &dev, &slave);
	if (ret)
		return ret;

	ret = spi_claim_bus(slave);
	if (ret)
		goto done;
	ret = spi_xfer(slave, 8*size, data_out, data_in, SPI_XFER_BEGIN | SPI_XFER_END);
	if (ret) {
		printf("Error %d during SPI transaction\n", ret);
	}
done:
	spi_release_bus(slave);

	return ret;
}

#define GPIO_NCONFIG			100
#define GPIO_NSTATUS			101
#define GPIO_CONF_DONE			103
#define GPIO_RESET				120

static int altera_config(void *data_addr, unsigned int size, int pcie_x2)
{
	const void *blob = gd->fdt_blob;
	struct udevice *dev;
	int node;
	ulong start;
	int i;

	puts("Configuring FPGA...\n");

	// Enable PCIe clock output 'ljcb_clk' => CTRL_CORE_SMA_SW_6.PCIE_TX_RX_CONTROL = 1
	volatile unsigned int *CTRL_CORE_SMA_SW_6 = (volatile unsigned int *)0x4A003C14;
	*CTRL_CORE_SMA_SW_6 = ((*CTRL_CORE_SMA_SW_6) & ~0x30000) | 0x10000;

	if (pcie_x2)
	{
		// Enable PCIESS1 x2 mode (~ +0.085 W)
		volatile unsigned int *CTRL_CORE_PCIE_CONTROL = (volatile unsigned int *)0x4A003C3C;
		*CTRL_CORE_PCIE_CONTROL |= 0x5;
	}

	// Enable MCSPI4 clock
	setbits_le32((*prcm)->cm_l4per_mcspi4_clkctrl, MODULE_CLKCTRL_MODULEMODE_SW_EXPLICIT_EN);

	// Find MCSPI device in the device tree
	node = fdtdec_get_chosen_node(blob, "fpga-init-dev");
	if (node < 0)
	{
		puts("Missing 'fpga-init-dev' node!\n");
		return -1;
	}
	if (uclass_get_device_by_of_offset(UCLASS_SPI, node, &dev)) {
		puts("Unable to get configuration device!\n");
		return -1;
	}

	gpio_request(GPIO_NCONFIG, "nConfig");
	gpio_direction_output(GPIO_NCONFIG, 1);
	gpio_request(GPIO_NSTATUS, "nStatus");
	gpio_request(GPIO_CONF_DONE, "CONF_DONE");
	// Reset einschalten, wird vom PCIe Kernel-Treiber wieder abgeschaltet
	gpio_request(GPIO_RESET, "nReset");
	gpio_direction_output(GPIO_RESET, 0);

	// nConfig = GND
	gpio_set_value(GPIO_NCONFIG, 0);
	udelay(100);
	// nConfig = VCC
    gpio_set_value(GPIO_NCONFIG, 1);

    // wait for nStatus
    start = get_timer(0);
    while (gpio_get_value(GPIO_NSTATUS) == 0) {
		if (get_timer(start) > 1000) {
			puts("Error: nSTATUS != VCC\n");
			return -1;
		}
    }

    // Check CONF_DONE
	if (gpio_get_value(GPIO_CONF_DONE)) {
		puts("CONF_DONE != GND!\n");
		return -1;
	}

	udelay(500);	// t_CF2CK

	// Swap bit order
	unsigned int *pSrc32 = (unsigned int *)data_addr;
	for (i = 0; i < (size+3) / 4; i++)
	{
	    asm("rbit %1,%0" : "=r" (pSrc32[i]) : "r" (pSrc32[i]));
	    asm("rev %1,%0" : "=r" (pSrc32[i]) : "r" (pSrc32[i]));
	}

	// Send data now
	if (do_spi_xfer(dev->seq, 0, data_addr, NULL, size) < 0)
		return -1;

//	udelay(100000);

	// Check CONF_DONE
	if (!gpio_get_value(GPIO_CONF_DONE)) {
		puts("CONF_DONE != VCC!\n");
		gpio_direction_input(GPIO_NCONFIG);
		return -1;
	}

	gpio_direction_input(GPIO_NCONFIG);

	return CMD_RET_SUCCESS;
}

struct spi_slave *slave;

static int do_altera_config(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int result;
	void *addr;
	unsigned int size;
	int pcie_x2 = 0;
	
	if (argc < 3) {
		puts("missing arguments!\n");
		return CMD_RET_FAILURE;
	}

	if (argc > 3) {
		if (strcmp(argv[3], "pcie_x2") == 0)
			pcie_x2 = 1;
	}

//	altera_power_on();

	addr = (void *)simple_strtoul(argv[1], NULL, 16);
	size = simple_strtoul(argv[2], NULL, 16);
	result = altera_config(addr, size, pcie_x2);
	if (result != CMD_RET_SUCCESS)
		return result;

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	fpga_init,	4,	0,	do_altera_config,
	"Configure FPGA device",
	"<address> <length> [pcie_x2]\n"
);
#if 0
static int imago_fpga_probe(struct udevice *dev)
{
	slave = dev_get_parent_priv(dev);
//	struct dm_spi_slave_platdata *plat = dev_get_parent_platdata(dev);
//	struct spi_flash *flash;

/*	flash = dev_get_uclass_priv(dev);
	flash->dev = dev;
	flash->spi = slave;
	debug("%s: slave=%p, cs=%d\n", __func__, slave, plat->cs);
	return spi_flash_probe_slave(flash);*/
	return 0;
}

/*static const struct dm_spi_flash_ops spi_flash_std_ops = {
	.read = spi_flash_std_read,
	.write = spi_flash_std_write,
	.erase = spi_flash_std_erase,
};*/

static const struct udevice_id imago_fpga_ids[] = {
	{ .compatible = "imago-fpga" },
	{ }
};

U_BOOT_DRIVER(imago_fpga) = {
	.name		= "imago_fpga",
	.id		= UCLASS_MISC,
	.of_match	= imago_fpga_ids,
	.probe		= imago_fpga_probe,
	.priv_auto_alloc_size = 0,//sizeof(struct spi_flash),
	.ops		= NULL,//&spi_flash_std_ops,
};
#endif

#endif
