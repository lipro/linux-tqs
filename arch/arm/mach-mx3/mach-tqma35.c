/*
 *  Copyright (C) 2010 Daniel Gericke, TQ-Systems
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/irq.h>

#include <linux/platform_device.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/plat-ram.h>
#include <linux/memory.h>
#include <linux/gpio.h>
#include <linux/smsc911x.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/i2c/pca953x.h>
#include <linux/usb/otg.h>
#include <linux/usb/ulpi.h>
#include <linux/fsl_devices.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>

#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/imx-uart.h>
#if defined CONFIG_I2C_IMX || defined CONFIG_I2C_IMX_MODULE
#include <mach/i2c.h>
#endif
#if defined CONFIG_SPI_IMX || defined CONFIG_SPI_IMX_MODULE
#include <mach/spi.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#endif
#include <mach/iomux-mx35.h>
#include <mach/ipu.h>
#include <mach/mx3fb.h>
#include <mach/mxc_nand.h>
#include <mach/mxc_ehci.h>
#include <mach/ulpi.h>
#include <mach/audmux.h>
#include <mach/ssi.h>
#include <mach/mmc.h>
#include <mach/mxc_flexcan.h>

#include "devices.h"

extern void mxc_mmc_force_detect(int id);

/*
 * GPIO pad index from port and pin
 */
#define TQMA35_GPIO_ID(__gpio_port, __gpio_pin) \
		((__gpio_port - 1) * 32 + __gpio_pin)

/* additional GPIO pins */
#define GPIO3_5                TQMA35_GPIO_ID(3, 5)
#define ETH1_PME       TQMA35_GPIO_ID(3, 4)
#define FEC_INT                TQMA35_GPIO_ID(2, 25)
#define SD3_CD         TQMA35_GPIO_ID(2, 18)
#define SD3_WP         TQMA35_GPIO_ID(2, 23)
#define LM75_ALARM     TQMA35_GPIO_ID(1, 22)
#define WAKEUP_IO      TQMA35_GPIO_ID(1, 4)
#define OVER_TEMP      TQMA35_GPIO_ID(1, 12)
#define TQMA35_USB_HSH_ULPI_CS TQMA35_GPIO_ID(2, 26)
#define AC97_GPIO_TXFS	TQMA35_GPIO_ID(2, 31)
#define AC97_GPIO_TXD	TQMA35_GPIO_ID(2, 28)
#define AC97_GPIO_RESET	TQMA35_GPIO_ID(2,  0)

/* MMC pins */
#define MMC1_CD TQMA35_GPIO_ID(2, 0)
#define MMC1_WP TQMA35_GPIO_ID(2, 7)

#define MMC3_CD TQMA35_GPIO_ID(2, 18)
#define MMC3_WP TQMA35_GPIO_ID(2, 23)

#define LAN9115_IRQ_GPIO3_3  TQMA35_GPIO_ID(3, 3)

/* GPIO definition for PCA9554 (0x20) at MBa35 */
#define TQMA35_MBA35_24V_OUTPUT_BASE	100
#define TQMA35_MBA35_24V_OUTPUT_0	(TQMA35_MBA35_24V_OUTPUT_BASE + 0)
#define TQMA35_MBA35_24V_OUTPUT_1	(TQMA35_MBA35_24V_OUTPUT_BASE + 1)
#define TQMA35_MBA35_24V_OUTPUT_2	(TQMA35_MBA35_24V_OUTPUT_BASE + 2)
#define TQMA35_MBA35_24V_OUTPUT_3	(TQMA35_MBA35_24V_OUTPUT_BASE + 3)
#define TQMA35_MBA35_24V_OUTPUT_4	(TQMA35_MBA35_24V_OUTPUT_BASE + 4)
#define TQMA35_MBA35_24V_OUTPUT_5	(TQMA35_MBA35_24V_OUTPUT_BASE + 5)
#define TQMA35_MBA35_24V_OUTPUT_6	(TQMA35_MBA35_24V_OUTPUT_BASE + 6)
#define TQMA35_MBA35_24V_OUTPUT_7	(TQMA35_MBA35_24V_OUTPUT_BASE + 7)

/* GPIO definition for PCA9554 (0x21) at MBa35 */
#define TQMA35_MBA35_MISC_I0_BASE	110
#define TQMA35_MBA35_24V_INPUT_0	(TQMA35_MBA35_MISC_I0_BASE + 0)
#define TQMA35_MBA35_24V_INPUT_1	(TQMA35_MBA35_MISC_I0_BASE + 1)
#define TQMA35_MBA35_24V_INPUT_2	(TQMA35_MBA35_MISC_I0_BASE + 2)
#define TQMA35_MBA35_24V_INPUT_3	(TQMA35_MBA35_MISC_I0_BASE + 3)
#define TQMA35_MBA35_USER_LED1		(TQMA35_MBA35_MISC_I0_BASE + 4)
#define TQMA35_MBA35_USER_LED2		(TQMA35_MBA35_MISC_I0_BASE + 5)
#define TQMA35_MBA35_BKL_ON		(TQMA35_MBA35_MISC_I0_BASE + 6)
#define TQMA35_MBA35_LVDS_ENA		(TQMA35_MBA35_MISC_I0_BASE + 7)

/*
 * provide an empty release function, this is needed in case a complex device
 * loading failed during probe and platform_device_unregister is called
 */
static void tqma35_dummy_release(struct device *dev)
{
}

#if defined(CONFIG_FB_MX3)

static const struct fb_videomode fb_modedb[] = {
#if defined(CONFIG_FB_MX3_DISPLAY_CLAA070LC0ACW)
	{
		/* 800x480 @ 60 Hz */
		.name		= "CLAA070LC0ACW",
		.refresh	= 60,
		.xres		= 800,
		.yres		= 480,
		.pixclock	= 37037,
		.left_margin	= 50,
		.right_margin	= 50,
		.upper_margin	= 10,
		.lower_margin	= 10,
		.hsync_len	= 1,
		.vsync_len	= 1,
		.sync		= FB_SYNC_OE_ACT_HIGH | FB_SYNC_CLK_IDLE_EN,
		.vmode		= FB_VMODE_NONINTERLACED,
		.flag		= 0,
	},
#endif
#if defined(CONFIG_FB_MX3_DISPLAY_FG0700K5DSSWBG01)
	{
		/* 800x480 @ 60 Hz */
		.name		= "FG0700K5DSSWBG01",
		.refresh	= 60,
		.xres		= 800,
		.yres		= 480,
		.pixclock	= 30066,
		.left_margin	= 128,
		.right_margin	= 90,
		.upper_margin	= 10,
		.lower_margin	= 35,
		.hsync_len	= 1,
		.vsync_len	= 1,
		.sync		= FB_SYNC_OE_ACT_HIGH | FB_SYNC_CLK_IDLE_EN,
		.vmode		= FB_VMODE_NONINTERLACED,
		.flag		= 0,
	},
#endif
};

static struct mx3fb_platform_data mx3fb_pdata = {
	.dma_dev	= &mx3_ipu.dev,
#if defined(CONFIG_FB_MX3_DISPLAY_CLAA070LC0ACW)
	.name		= "CLAA070LC0ACW",
#elif defined(CONFIG_FB_MX3_DISPLAY_FG0700K5DSSWBG01)
	.name           = "FG0700K5DSSWBG01",
#endif
	.mode		= fb_modedb,
	.num_modes	= ARRAY_SIZE(fb_modedb),
#if defined(CONFIG_FB_MX3_BACKLIGHT)
#if defined(CONFIG_FB_MX3_DISPLAY_CLAA070LC0ACW)
       .phys_bl_off    = 0,
       .phys_bl_min    = 1,
       .phys_bl_max    = 255,
#elif defined(CONFIG_FB_MX3_DISPLAY_FG0700K5DSSWBG01)
       .phys_bl_off    = 0,
       .phys_bl_min    = 1,
       .phys_bl_max    = 255,
#endif
#endif /* CONFIG_FB_MX3_BACKLIGHT */
};
#endif /* CONFIG_FB_MX3 */

static struct ipu_platform_data mx3_ipu_data = {
	.irq_base = MXC_IPU_IRQ_START,
};

static struct physmap_flash_data tqma35_flash_data = {
	.width  = 2,
};

static struct resource tqma35_flash_resource = {
	.start	= 0xa0000000,
	.end	= 0xa1ffffff,
	.flags	= IORESOURCE_MEM,
};

static struct platform_device tqma35_flash = {
	.name	= "physmap-flash",
	.id	= 0,
	.dev	= {
		.platform_data  = &tqma35_flash_data,
		.release = tqma35_dummy_release,
	},
	.resource = &tqma35_flash_resource,
	.num_resources = 1,
};

static struct imxuart_platform_data uart0_pdata = {
	.flags = 0, /* RTS/CTS pins used for CSI */
};

static struct imxuart_platform_data uart1_pdata = {
	.flags = IMXUART_HAVE_RTSCTS,
};

static struct imxuart_platform_data uart2_pdata = {
	.flags = IMXUART_HAVE_RTSCTS,
};

#if defined(CONFIG_SMSC911X)
/*
 * SMSC 9115
 * Network support
 */
static struct resource tqma35_smc911x_resources[] = {
	{
		.start	= MX35_CS1_BASE_ADDR,
		.end	= MX35_CS1_BASE_ADDR + SZ_32M - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= (MXC_BOARD_IRQ_START + LAN9115_IRQ_GPIO3_3),
		.end	= (MXC_BOARD_IRQ_START + LAN9115_IRQ_GPIO3_3),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};

static struct smsc911x_platform_config smsc911x_info = {
	.flags		= SMSC911X_USE_16BIT | SMSC911X_SAVE_MAC_ADDRESS,
	.irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type       = SMSC911X_IRQ_TYPE_PUSH_PULL,
};

static struct platform_device tqma35_smc911x_device = {
	.name           = "smsc911x",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(tqma35_smc911x_resources),
	.resource       = tqma35_smc911x_resources,
	.dev            = {
		.platform_data = &smsc911x_info,
		.release = tqma35_dummy_release,
	},
};

static int tqma35_register_smsc9115(void)
{
	/* SMSC9115 IRQ pin */
	/* must be done befor registerimg the device */
	if (0 == gpio_request(LAN9115_IRQ_GPIO3_3, "LAN9115_IRQ")) {
		gpio_direction_input(LAN9115_IRQ_GPIO3_3);
		tqma35_smc911x_resources[1].start =
			gpio_to_irq(LAN9115_IRQ_GPIO3_3);
		tqma35_smc911x_resources[1].end =
			gpio_to_irq(LAN9115_IRQ_GPIO3_3);
		platform_device_register(&tqma35_smc911x_device);
		return 0;
	}
	pr_err("gpio_request for LAN9115_IRQ_GPIO3_3 failed\n");
	return -1;
}
#endif

#if defined CONFIG_SPI_IMX || defined CONFIG_SPI_IMX_MODULE

/* CSPI1 */

/* use GPIO based chipselect for CSPI1 */
#define TQMA35_SPI0_CS0 TQMA35_GPIO_ID(1, 18)
#define TQMA35_SPI0_CS1 TQMA35_GPIO_ID(1, 19)
#define TQMA35_SPI0_CS3 TQMA35_GPIO_ID(2, 6)

static int tqma35_spi0_cs[] = {
	TQMA35_SPI0_CS0,
	TQMA35_SPI0_CS1,
	TQMA35_SPI0_CS3,
};

static struct spi_imx_master tqma35_spi0_master = {
	.chipselect	= tqma35_spi0_cs,
	.num_chipselect	= ARRAY_SIZE(tqma35_spi0_cs),
};

/* CSPI2 */

/* use GPIO based chipselect for CSPI2 */
#define TQMA35_SPI1_CS0 TQMA35_GPIO_ID(1, 6)

static int tqma35_spi1_cs[] = {
	TQMA35_SPI1_CS0,
};

static struct spi_imx_master tqma35_spi1_master = {
	.chipselect	= tqma35_spi1_cs,
	.num_chipselect	= ARRAY_SIZE(tqma35_spi1_cs),
};

#if defined(CONFIG_TOUCHSCREEN_ADS7846) || \
	defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)

#define TQMA35_ADS7846_PENDOWN_GPIO1_5 TQMA35_GPIO_ID(1, 5)

static int tqma35_ads7846_get_pendown_state(void);

static struct ads7846_platform_data tqma35_ads7846_config __initdata = {
	.get_pendown_state	= tqma35_ads7846_get_pendown_state,
	.keep_vref_on		= 1, /* force vref on between samples */
	.model			= 7846,
/* TODO: optimize settling time if needed, check if debouncing is ok */
	.settle_delay_usecs	= 100, /* double sampling with 100 usec delay */
/*
 * simple debouncing filter, ok if rep samples with delta < tol, break if max
 * samples and not ok
 */
	.debounce_max		= 10, /* max samples */
	.debounce_tol		= 3,/* max AD-Count tolerance between samples */
	.debounce_rep		= 1,  /* samples within tolerance */
};

static int tqma35_ads7846_get_pendown_state(void)
{
	return !gpio_get_value(TQMA35_ADS7846_PENDOWN_GPIO1_5);
}

static void tqma35_ads7846_dev_init(struct spi_board_info *board_info,
				size_t length)
{
	unsigned i;
	if (gpio_request(TQMA35_ADS7846_PENDOWN_GPIO1_5,
			"ADS7846 pendown") < 0) {
		printk(KERN_ERR "can't get ads746 pen down GPIO\n");
		return;
	}
	gpio_direction_input(TQMA35_ADS7846_PENDOWN_GPIO1_5);
	for (i = 0; i < length; ++i) {
		if (0 == strncmp("ads7846", board_info[i].modalias, 7)) {
			board_info[i].irq =
				gpio_to_irq(TQMA35_ADS7846_PENDOWN_GPIO1_5);
			break;
		}
	}
	if (i >= length) {
		printk(KERN_ERR "can't find ts spi config for down GPIO\n");
		return;
	}
}
#endif /* CONFIG_TOUCHSCREEN_ADS7846 */


static struct spi_board_info tqma35_spi_board_info[] __initdata = {
#if defined(CONFIG_TOUCHSCREEN_ADS7846) || \
	defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)
	{
		.modalias	= "ads7846",
		.bus_num	= 1,
		.chip_select	= 0,
		.max_speed_hz	= 1000000, /* max is appr. 125 kHz * 26 @ 3 V*/
		.irq		= 0,
		.platform_data	= &tqma35_ads7846_config,
		.mode		= SPI_MODE_2,
	},
#endif
};


static int tqma35_register_spi(void)
{
	mxc_register_device(&mxc_spi_device0, &tqma35_spi0_master);
	printk(KERN_INFO "cspi1 registered\n");
	mxc_register_device(&mxc_spi_device1, &tqma35_spi1_master);
	printk(KERN_INFO "cspi2 registered\n");

#if defined(CONFIG_TOUCHSCREEN_ADS7846) || \
	defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)
	/* call local init, needs to map gpio for irq before loading device */
	tqma35_ads7846_dev_init(tqma35_spi_board_info,
		ARRAY_SIZE(tqma35_spi_board_info));
#endif
	spi_register_board_info(tqma35_spi_board_info,
				ARRAY_SIZE(tqma35_spi_board_info));

	return 0;
}
#else
static inline int tqma35_register_spi(void)
{
	return 0;
}
#endif /* CONFIG_SPI_IMX */

#if defined CONFIG_I2C_IMX || defined CONFIG_I2C_IMX_MODULE
static struct imxi2c_platform_data tqma35_i2c0_data = {
	.bitrate = 100000,
};

static struct at24_platform_data board_eeprom = {
	.byte_len = 4096,
	.page_size = 32,
	.flags = AT24_FLAG_ADDR16,
};

/* I2C IO-Expander support for MBa35 */
#if defined(CONFIG_GPIO_PCA953X)

/* 24V general purpose output @ I2C addr */

static int mba35_24v_output_setup(struct i2c_client *client, unsigned gpio,
					unsigned ngpio, void *c)
{
/*
 * implement special pin setup handling as needed
 */
#if 0
	int ret;
	unsigned i;
	char name[32];

	pr_info("mba35_24v_output_setup ...\n");
	for (i = 0; i < ngpio; ++i) {
		sprintf(name, "MBA35_24V_OUT_%u", i);
		ret = gpio_request(gpio + i, name);
		if (ret) {
			pr_warning("Cannot open IO expander pin %d\n", i);
			goto setup_24v_fail;
		}
		/*
		 * deselect all functionalities, pins are pulled down
		 * by default
		 */
		gpio_direction_output(gpio + i, 0);
	}
	return 0;
setup_24v_fail:
	for (; i >= 0; --i) {
		gpio_set_value(gpio + i, 0);
		gpio_free(gpio + i);
	}
	pr_warning("mba35_24v_output_setup failed\n");
	return ret;
#else
	pr_info("mba35_24v_output_setup ...\n");
	return 0;
#endif
}

static int mba35_24v_output_teardown(struct i2c_client *client,
					unsigned gpio, unsigned ngpio, void *c)
{
/*
 * implement special pin teardown handling as needed
 */
#if 0
	/* deselect all functionalities, pins are pulled down by default */
	unsigned i;

	pr_info("mba35_24v_output_teardown\n");
	for (i = 0; i < ngpio; ++i) {
		gpio_set_value(gpio + i, 0);
		gpio_free(gpio + i);
	}
	return 0;
#else
	pr_info("mba35_24v_output_teardown\n");
	return 0;
#endif
}

static struct pca953x_platform_data mba35_24v_output = {
	/* number of the first GPIO */
	.gpio_base = TQMA35_MBA35_24V_OUTPUT_BASE,

	/* initial polarity inversion setting */
	/* uint16_t	invert, */
	/* interrupt base, irq pin not connected */
	.irq_base = 0,
	.context = 0,	/* param to setup/teardown */
	.setup = mba35_24v_output_setup,
	.teardown = mba35_24v_output_teardown,
};

static int mba35_misc_gpio_setup(struct i2c_client *client, unsigned gpio,
					unsigned ngpio, void *c)
{
/*
 * implement special pin setup handling as needed
 */
#if 0
	int ret;
	unsigned i;
	char name[32];

	pr_info("mba35_misc_gpio_setup ...\n");
	for (i = 0; i < 4; ++i) {
		sprintf(name, "MBA35_24V_IN_%u", i);
		ret = gpio_request(gpio + i, name);
		if (ret) {
			pr_warning("Cannot open UI expander pin %d\n", i);
			goto setup_misc_in_fail;
		}
		/* deselect all functionalities */
		gpio_direction_input(gpio + i);
	}
	ret = gpio_request(gpio + 4, "MBA35_LED1");
	if (ret) {
		pr_warning("Cannot open UI expander pin %d\n", 4);
		goto setup_misc_led1_fail;
	}
	/* user LED 1 off */
	gpio_direction_output(gpio + 4, 0);
	ret = gpio_request(gpio + 5, "MBA35_LED2");
	if (ret) {
		pr_warning("Cannot open UI expander pin %d\n", 5);
		goto setup_misc_led2_fail;
	}
	/* user LED 2 off */
	gpio_direction_output(gpio + 5, 0);
	ret = gpio_request(gpio + 6, "MBA35_BKL_ON");
	if (ret) {
		pr_warning("Cannot open UI expander pin %d\n", 6);
		goto setup_misc_bkl_on_fail;
	}
	/* user BKL_ON: pulled down with 100k @ LT3518 */
	gpio_direction_output(gpio + 6, 0);
	ret = gpio_request(gpio + 7, "MBA35_LVDS_ENA");
	if (ret) {
		pr_warning("Cannot open UI expander pin %d\n", 7);
		goto setup_misc_lvds_ena_fail;
	}
	/* user LVDS-ENA: pulled down with 4k7k @ LVDS transmitter */
	gpio_direction_output(gpio + 7, 0);

	return 0;
setup_misc_lvds_ena_fail:
	gpio_free(gpio + 7);
setup_misc_bkl_on_fail:
	gpio_free(gpio + 6);
setup_misc_led2_fail:
	gpio_free(gpio + 5);
setup_misc_led1_fail:
	gpio_free(gpio + 4);
setup_misc_in_fail:
	for (i = 0; i < 4; --i)
		gpio_free(gpio + i);

	pr_warning("mba35_misc_gpio_setup failed\n");
	return ret;
#else
	pr_info("mba35_misc_gpio_setup ...\n");
	return 0;
#endif
}


static int mba35_misc_gpio_teardown(struct i2c_client *client,
					unsigned gpio, unsigned ngpio, void *c)
{
/*
 * implement special pin teardown handling as needed
 */
#if 0
	/* deselect all functionalities */
	unsigned i;
	pr_info("mba35_misc_gpio_teardown\n");
	for (i = 4; i < 8; ++i) {
		gpio_set_value(gpio + i, 0);
		gpio_free(gpio + i);
	}
	for (i = 0; i < 4; ++i)
		gpio_free(gpio + i);

	return 0;
#else
	pr_info("mba35_misc_gpio_teardown ...\n");
	return 0;
#endif
}

static struct pca953x_platform_data mba35_misc_gpio = {
	/* number of the first GPIO */
	.gpio_base = TQMA35_MBA35_MISC_I0_BASE,
	/* initial polarity inversion setting */
	/* uint16_t	invert; */
	/* interrupt base */
	.irq_base = 0,
	.context = 0,	/* param to setup/teardown */
	.setup = mba35_misc_gpio_setup,
	.teardown = mba35_misc_gpio_teardown,
	/* char		**names; */
};
#endif

static struct i2c_board_info tqma35_i2c0_devices[] = {
#if defined(CONFIG_SENSORS_LM75)
	/* sensor on base board */
	{
		I2C_BOARD_INFO("lm75", 0x49), /* A0=1, A1=0, A2=0 */
	},
#endif
	/*
	 * TODO: port driver from sound/soc/imx/imx-3stack-sgtl5000.c found in
	 * sound/soc/imx/imx-3stack-sgtl5000.c see
	 * http://opensource.freescale.com/git,
	 * latest is rel_imx_2.6.31_10.09.00
	 */
	{
		I2C_BOARD_INFO("sgtl5000-i2c", 0x0a),
	},
#if defined(CONFIG_GPIO_PCA953X)
	{
		I2C_BOARD_INFO("pca9554", 0x20), /* A0=0, A1=0, A2=0 */
		.platform_data = &mba35_24v_output,
	},
	{
		I2C_BOARD_INFO("pca9554", 0x21), /* A0=1, A1=0, A2=0 */
		.platform_data = &mba35_misc_gpio,
	},
#endif
};

static struct imxi2c_platform_data tqma35_i2c1_data = {
	.bitrate = 100000,
};

static struct i2c_board_info tqma35_i2c1_devices[] = {
	{
		I2C_BOARD_INFO("at24", 0x50), /* E0=0, E1=0, E2=0 */
		.platform_data = &board_eeprom,
	},
#if defined(CONFIG_SENSORS_LM75)
	/* sensor on cpu board */
	{
		I2C_BOARD_INFO("lm75", 0x48), /* A0=0, A1=0, A2=0 */
	},
#endif
#if defined(CONFIG_RTC_DRV_DS1307)
	{
		I2C_BOARD_INFO("ds1339", 0x68),
	},
#endif
};

static void tqma35_register_i2c(void)
{
	i2c_register_board_info(0, tqma35_i2c0_devices,
			ARRAY_SIZE(tqma35_i2c0_devices));
	i2c_register_board_info(1, tqma35_i2c1_devices,
			ARRAY_SIZE(tqma35_i2c1_devices));

	mxc_register_device(&mxc_i2c_device0, &tqma35_i2c0_data);
	mxc_register_device(&mxc_i2c_device1, &tqma35_i2c1_data);
}
#endif

static struct platform_device *devices[] __initdata = {
	&tqma35_flash,
	&mxc_fec_device,
};

static void flexcan_xcvr_enable(int id, int en)
{
	static int pwdn;

	if (id < 0 || id > 1)
		return;

	if (en) {
		if (!(pwdn++))
			;
			/* TODO: Adapt the power function main board specific
			pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_2,1,0);
			*/
	} else {
		if (!(--pwdn))
			;
			/* TODO: Adapt the power function main board specific
			pmic_gpio_set_bit_val(MCU_GPIO_REG_GPIO_CONTROL_2,1,1);
			*/
	}
}

struct flexcan_platform_data flexcan_data0 = {
	.xcvr_enable = flexcan_xcvr_enable,
/* TODO: Adapt the power function main board specific
	.active = gpio_can_active,
	.inactive = gpio_can_inactive,
*/
};

struct flexcan_platform_data flexcan_data1 = {
	.xcvr_enable = flexcan_xcvr_enable,
/* TODO: Adapt the power function main board specific
	.active = gpio_can_active,
	.inactive = gpio_can_inactive,
*/
};

static struct pad_desc tqma35_pads[] = {
	/* UART1 */
	MX35_PAD_TXD1__UART1_TXD_MUX,
	MX35_PAD_RXD1__UART1_RXD_MUX,
	/* UART2 */
	MX35_PAD_CTS2__UART2_CTS,
	MX35_PAD_RTS2__UART2_RTS,
	MX35_PAD_TXD2__UART2_TXD_MUX,
	MX35_PAD_RXD2__UART2_RXD_MUX,
	/* UART3 */
	MX35_PAD_ATA_DATA9__UART3_CTS,
	MX35_PAD_ATA_DATA8__UART3_RTS,
	MX35_PAD_ATA_DATA11__UART3_TXD_MUX,
	MX35_PAD_ATA_DATA10__UART3_RXD_MUX,
	/* LAN9115_IRQ */
	MX35_PAD_MLB_CLK__GPIO3_3,
	/* FEC */
	MX35_PAD_FEC_TX_CLK__FEC_TX_CLK,
	MX35_PAD_FEC_RX_CLK__FEC_RX_CLK,
	MX35_PAD_FEC_RX_DV__FEC_RX_DV,
	MX35_PAD_FEC_COL__FEC_COL,
	MX35_PAD_FEC_RDATA0__FEC_RDATA_0,
	MX35_PAD_FEC_TDATA0__FEC_TDATA_0,
	MX35_PAD_FEC_TX_EN__FEC_TX_EN,
	MX35_PAD_FEC_MDC__FEC_MDC,
	MX35_PAD_FEC_MDIO__FEC_MDIO,
	MX35_PAD_FEC_TX_ERR__FEC_TX_ERR,
	MX35_PAD_FEC_RX_ERR__FEC_RX_ERR,
	MX35_PAD_FEC_CRS__FEC_CRS,
	MX35_PAD_FEC_RDATA1__FEC_RDATA_1,
	MX35_PAD_FEC_TDATA1__FEC_TDATA_1,
	MX35_PAD_FEC_RDATA2__FEC_RDATA_2,
	MX35_PAD_FEC_TDATA2__FEC_TDATA_2,
	MX35_PAD_FEC_RDATA3__FEC_RDATA_3,
	MX35_PAD_FEC_TDATA3__FEC_TDATA_3,
	/* I2C1 */
	MX35_PAD_I2C1_CLK__I2C1_SCL,
	MX35_PAD_I2C1_DAT__I2C1_SDA,
	/* I2C2 */
	MX35_PAD_I2C2_CLK__I2C2_SCL,
	MX35_PAD_I2C2_DAT__I2C2_SDA,
	/* CSI */
	MX35_PAD_RTS1__IPU_CSI_D_0,
	MX35_PAD_CTS1__IPU_CSI_D_1,
	MX35_PAD_ATA_INTRQ__IPU_CSI_D_2,
	MX35_PAD_ATA_BUFF_EN__IPU_CSI_D_3,
	MX35_PAD_ATA_DMARQ__IPU_CSI_D_4,
	MX35_PAD_ATA_DA0__IPU_CSI_D_5,
	MX35_PAD_ATA_DA1__IPU_CSI_D_6,
	MX35_PAD_ATA_DA2__IPU_CSI_D_7,

	MX35_PAD_CSI_D8__IPU_CSI_D_8,
	MX35_PAD_CSI_D9__IPU_CSI_D_9,
	MX35_PAD_CSI_D10__IPU_CSI_D_10,
	MX35_PAD_CSI_D11__IPU_CSI_D_11,
	MX35_PAD_CSI_D12__IPU_CSI_D_12,
	MX35_PAD_CSI_D13__IPU_CSI_D_13,
	MX35_PAD_CSI_D14__IPU_CSI_D_14,
	MX35_PAD_CSI_D15__IPU_CSI_D_15,

	MX35_PAD_CSI_PIXCLK__IPU_CSI_PIXCLK,
	MX35_PAD_CSI_MCLK__IPU_CSI_MCLK,
	MX35_PAD_CSI_VSYNC__IPU_CSI_VSYNC,
	MX35_PAD_CSI_HSYNC__IPU_CSI_HSYNC,

	/* Display */
	MX35_PAD_LD0__IPU_DISPB_DAT_0,
	MX35_PAD_LD1__IPU_DISPB_DAT_1,
	MX35_PAD_LD2__IPU_DISPB_DAT_2,
	MX35_PAD_LD3__IPU_DISPB_DAT_3,
	MX35_PAD_LD4__IPU_DISPB_DAT_4,
	MX35_PAD_LD5__IPU_DISPB_DAT_5,
	MX35_PAD_LD6__IPU_DISPB_DAT_6,
	MX35_PAD_LD7__IPU_DISPB_DAT_7,
	MX35_PAD_LD8__IPU_DISPB_DAT_8,
	MX35_PAD_LD9__IPU_DISPB_DAT_9,
	MX35_PAD_LD10__IPU_DISPB_DAT_10,
	MX35_PAD_LD11__IPU_DISPB_DAT_11,
	MX35_PAD_LD12__IPU_DISPB_DAT_12,
	MX35_PAD_LD13__IPU_DISPB_DAT_13,
	MX35_PAD_LD14__IPU_DISPB_DAT_14,
	MX35_PAD_LD15__IPU_DISPB_DAT_15,
	MX35_PAD_LD16__IPU_DISPB_DAT_16,
	MX35_PAD_LD17__IPU_DISPB_DAT_17,
	MX35_PAD_D3_HSYNC__IPU_DISPB_D3_HSYNC,
	MX35_PAD_D3_FPSHIFT__IPU_DISPB_D3_CLK,
	MX35_PAD_D3_DRDY__IPU_DISPB_D3_DRDY,
	MX35_PAD_CONTRAST__IPU_DISPB_CONTR,
	MX35_PAD_D3_VSYNC__IPU_DISPB_D3_VSYNC,
	MX35_PAD_D3_REV__IPU_DISPB_D3_REV,
	MX35_PAD_D3_CLS__IPU_DISPB_D3_CLS,
	/* gpio */
	MX35_PAD_MLB_SIG__GPIO3_5, /* Free usable */
	MX35_PAD_MLB_DAT__GPIO3_4, /* ETH1_PME */
	MX35_PAD_ATA_DATA12__GPIO2_25, /* FEC INT */
	MX35_PAD_NFRB__GPIO2_23, /* SD3 WP */
	MX35_PAD_ATA_DATA5__GPIO2_18, /* SD3 CD */
	MX35_PAD_SCKR__GPIO1_4, /* WAKEUP */
	MX35_PAD_TX3_RX2__GPIO1_12, /* OVER_TEMP */
	MX35_PAD_NF_CE0__GPIO1_22, /* LM75_ALARM */
	/* USBOTG */
	MX35_PAD_USBOTG_PWR__USB_TOP_USBOTG_PWR,
	MX35_PAD_USBOTG_OC__USB_TOP_USBOTG_OC,
	/* USB HSH */
	MX35_PAD_GPIO3_0__USB_TOP_USBH2_CLK,
	MX35_PAD_ATA_DATA13__GPIO2_26,
	MX35_PAD_NFALE__USB_TOP_USBH2_STP,
	MX35_PAD_NFCLE__USB_TOP_USBH2_NXT,
	MX35_PAD_NFRE_B__USB_TOP_USBH2_DIR,
	MX35_PAD_SD2_DATA1__USB_TOP_USBH2_DATA_0,
	MX35_PAD_SD2_DATA2__USB_TOP_USBH2_DATA_1,
	MX35_PAD_SD2_DATA3__USB_TOP_USBH2_DATA_2,
	MX35_PAD_NFWE_B__USB_TOP_USBH2_DATA_3,
	MX35_PAD_SD2_CMD__USB_TOP_USBH2_DATA_4,
	MX35_PAD_SD2_CLK__USB_TOP_USBH2_DATA_5,
	MX35_PAD_SD2_DATA0__USB_TOP_USBH2_DATA_6,
	MX35_PAD_NFWP_B__USB_TOP_USBH2_DATA_7,
	/* SSI */
	MX35_PAD_STXFS4__AUDMUX_AUD4_TXFS,
	MX35_PAD_STXD4__AUDMUX_AUD4_TXD,
	MX35_PAD_SRXD4__AUDMUX_AUD4_RXD,
	MX35_PAD_SCK4__AUDMUX_AUD4_TXC,
	/* ESDHC1 */
	MX35_PAD_SD1_CMD__ESDHC1_CMD,
	MX35_PAD_SD1_CLK__ESDHC1_CLK,
	MX35_PAD_SD1_DATA0__ESDHC1_DAT0,
	MX35_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX35_PAD_SD1_DATA2__ESDHC1_DAT2,
	MX35_PAD_SD1_DATA3__ESDHC1_DAT3,
	/* ESDHC1 CD */
	MX35_PAD_GPIO2_0__GPIO2_0,
	/* ESDHC1 WP */
	MX35_PAD_ATA_CS1__GPIO2_7,

	/* ESDHC3 */
	MX35_PAD_ATA_DATA4__ESDHC3_CMD,
	MX35_PAD_ATA_DATA3__ESDHC3_CLK,
	MX35_PAD_ATA_DIOR__ESDHC3_DAT0,
	MX35_PAD_ATA_DIOW__ESDHC3_DAT1,
	MX35_PAD_ATA_DMACK__ESDHC3_DAT2,
	MX35_PAD_ATA_RESET_B__ESDHC3_DAT3,
	MX35_PAD_ATA_IORDY__ESDHC3_DAT4,
	MX35_PAD_ATA_DATA0__ESDHC3_DAT5,
	MX35_PAD_ATA_DATA1__ESDHC3_DAT6,
	MX35_PAD_ATA_DATA2__ESDHC3_DAT7,

	/* ESDHC3 CD */
	MX35_PAD_ATA_DATA5__GPIO2_18,
	/* ESDHC3 WP */
	MX35_PAD_NFRB__GPIO2_23,

	/* SPI1 */
	MX35_PAD_CSPI1_MOSI__CSPI1_MOSI,
	MX35_PAD_CSPI1_MISO__CSPI1_MISO,
	MX35_PAD_CSPI1_SCLK__CSPI1_SCLK,
	MX35_PAD_CSPI1_SPI_RDY__CSPI1_RDY,
	/* Do GPIO based chip-select */
	/*
	MX35_PAD_CSPI1_SS0__CSPI1_SS0,
	MX35_PAD_CSPI1_SS1__CSPI1_SS1,
	MX35_PAD_ATA_CS0__CSPI1_SS3,
	*/
	MX35_PAD_CSPI1_SS0__GPIO1_18,
	MX35_PAD_CSPI1_SS1__GPIO1_19,
	MX35_PAD_ATA_CS0__GPIO2_6,
	/* SPI2 */
	MX35_PAD_STXD5__CSPI2_MOSI,
	MX35_PAD_SRXD5__CSPI2_MISO,
	MX35_PAD_SCK5__CSPI2_SCLK,
	MX35_PAD_STXFS5__CSPI2_RDY,
	/* Do GPIO based chip-select */
	/* MX35_PAD_HCKR__CSPI2_SS0, */
	MX35_PAD_HCKR__GPIO1_6,

	/* Touch */
	MX35_PAD_FSR__GPIO1_5,
	/* CAN */
	MX35_PAD_ATA_DATA7__CAN1_RXCAN,
	MX35_PAD_ATA_DATA6__CAN1_TXCAN,
	MX35_PAD_TX4_RX1__CAN2_RXCAN,
	MX35_PAD_TX5_RX0__CAN2_TXCAN,
};

static struct mxc_usbh_platform_data otg_pdata = {
	.portsc	= MXC_EHCI_MODE_UTMI,
	.flags	= MXC_EHCI_INTERFACE_DIFF_UNI,
};

#if defined(CONFIG_USB_ULPI)
static int usbh1_phy_init(struct platform_device *pdev)
{
	gpio_set_value(TQMA35_USB_HSH_ULPI_CS, 0);
	pr_info("usbh1_phy_init\n");
	return 0;
}

static int usbh1_phy_exit(struct platform_device *pdev)
{
	pr_info("usbh1_phy_exit\n");
	gpio_set_value(TQMA35_USB_HSH_ULPI_CS, 1);
	return 0;
}
static struct mxc_usbh_platform_data usbh1_pdata = {
	.init   = usbh1_phy_init,
	.exit   = usbh1_phy_exit,
	.portsc = MXC_EHCI_MODE_ULPI,
	.flags  = 0,
};
static void tqma35_init_usbh1(void)
{
	if (0 == gpio_request(TQMA35_USB_HSH_ULPI_CS, "usb-host2-cs")) {
		gpio_direction_output(TQMA35_USB_HSH_ULPI_CS, 1);
		usbh1_pdata.otg = otg_ulpi_create(&mxc_ulpi_access_ops,
				USB_OTG_DRV_VBUS | USB_OTG_DRV_VBUS_EXT);
		mxc_register_device(&mxc_usbh1, &usbh1_pdata);
	}
}
#endif

static struct fsl_usb2_platform_data otg_device_pdata = {
	.operating_mode = FSL_USB2_DR_DEVICE,
	.phy_mode       = FSL_USB2_PHY_UTMI,
};



static int otg_mode_host;

static int __init tqma35_otg_mode(char *options)
{
	if (!strcmp(options, "host"))
		otg_mode_host = 1;
	else if (!strcmp(options, "device"))
		otg_mode_host = 0;
	else
		pr_info("otg_mode neither \"host\" nor \"device\". "
			"Defaulting to device\n");
	return 0;
}
__setup("otg_mode=", tqma35_otg_mode);

unsigned int tqma35_mmc_cd_status(struct device *dev)
{
	struct platform_device *mxcsdhc_device = to_platform_device(dev);

	switch (mxcsdhc_device->id) {
	case 0:
		return gpio_get_value(MMC1_CD);
	case 2:
		/* eMMC is always present */
		return 0;
		/* return gpio_get_value(MMC3_CD);*/
	default :
		return 0;
	}
}

int tqma35_mmc_wp_status(struct device *dev)
{
	struct platform_device *mxcsdhc_device = to_platform_device(dev);

	if (mxcsdhc_device->id == 1)
		return gpio_get_value(MMC1_WP);

	return 0;
}


static struct mxc_mmc_platform_data tqma35_mmc1_pdata = {
	.ocr_mask = MMC_VDD_32_33,
	.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_NEEDS_POLL,
	.min_clk = 150000,
	.max_clk = 52000000,
	.card_fixed = 0,
	.card_inserted_state = 0,
	.status = tqma35_mmc_cd_status,
	.wp_status = tqma35_mmc_wp_status,
	.clock_mmc = "sdhc",
};

static struct mxc_mmc_platform_data tqma35_mmc2_pdata = {
	.ocr_mask = MMC_VDD_32_33,
	.caps = MMC_CAP_8_BIT_DATA | MMC_CAP_NONREMOVABLE,
	.min_clk = 400000,
	.max_clk = 52000000,
	.card_fixed = 1,
	.card_inserted_state = 0,
	.status = tqma35_mmc_cd_status,
	.clock_mmc = "sdhc",
};

static int __init mxc_late_init_mmc(void)
{
	/* force detection of the fixed eMMC */
/*	mxc_mmc_force_detect(2); */
	return 0;
}

late_initcall(mxc_late_init_mmc);

/*
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	printk(KERN_INFO "mxc_board_init() called\n");
	mxc_iomux_v3_setup_multiple_pads(tqma35_pads, ARRAY_SIZE(tqma35_pads));

	mxc_audmux_v2_configure_port(3,
			MXC_AUDMUX_V2_PTCR_SYN | /* 4wire mode */
			MXC_AUDMUX_V2_PTCR_TFSEL(0) |
			MXC_AUDMUX_V2_PTCR_TFSDIR,
			MXC_AUDMUX_V2_PDCR_RXDSEL(0));

	mxc_audmux_v2_configure_port(0,
			MXC_AUDMUX_V2_PTCR_SYN | /* 4wire mode */
			MXC_AUDMUX_V2_PTCR_TCSEL(3) |
			MXC_AUDMUX_V2_PTCR_TCLKDIR, /* clock is output */
			MXC_AUDMUX_V2_PDCR_RXDSEL(3));

#if defined(CONFIG_SMSC911X)
	tqma35_register_smsc9115();
#endif

	platform_add_devices(devices, ARRAY_SIZE(devices));

	mxc_register_device(&mxc_uart_device0, &uart0_pdata);
/*	mxc_register_device(&imx_ssi_device0, &tqma35_ssi_pdata);*/

	mxc_register_device(&mxc_uart_device1, &uart1_pdata);
	mxc_register_device(&mxc_uart_device2, &uart2_pdata);

	tqma35_register_spi();

#if defined CONFIG_I2C_IMX || defined CONFIG_I2C_IMX_MODULE
	tqma35_register_i2c();
#endif

	mxc_register_device(&flexcan_device0, &flexcan_data0);
	mxc_register_device(&flexcan_device1, &flexcan_data1);

	/* Request MMC1_CD */
	gpio_request(MMC1_CD, "MMC1");
	/* Request MMC1_WP */
	gpio_request(MMC1_WP, "MMC1");
	tqma35_mmc1_pdata.detect_irq = gpio_to_irq(MMC1_CD);
	mxc_register_device(&mxc_sdhc_device0, &tqma35_mmc1_pdata);

#if 0
	/* Request MMC3_CD */
	gpio_request(MMC3_CD, "MMC3");
	/* Request MMC3_WP */
	gpio_request(MMC3_WP, "MMC3");
	tqma35_mmc2_pdata.detect_irq = gpio_to_irq(MMC3_CD);
	mxc_register_device(&mxc_sdhc_device2, &tqma35_mmc2_pdata);
#endif

	mxc_register_device(&mx3_ipu, &mx3_ipu_data);
#if defined(CONFIG_FB_MX3)
	mxc_register_device(&mx3_fb, &mx3fb_pdata);
#endif
	if (otg_mode_host) {
/*		otg_pdata.otg = otg_ulpi_create(&mxc_ulpi_access_ops,
				USB_OTG_DRV_VBUS | USB_OTG_DRV_VBUS_EXT);
*/
		mxc_register_device(&mxc_otg_host, &otg_pdata);
	}

	if (!otg_mode_host)
		mxc_register_device(&mxc_otg_udc_device, &otg_device_pdata);

#if defined(CONFIG_USB_ULPI)
	tqma35_init_usbh1();
#endif
}

static void __init tqma35_timer_init(void)
{
	mx35_clocks_init();
}

struct sys_timer tqma35_timer = {
	.init	= tqma35_timer_init,
};

MACHINE_START(TQMA35, "TQMa35")
	/* Maintainer: TQ-Systems */
	.phys_io	= MX35_AIPS1_BASE_ADDR,
	.io_pg_offst	= ((MX35_AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params    = MX3x_PHYS_OFFSET + 0x100,
	.map_io         = mx35_map_io,
	.init_irq       = mx35_init_irq,
	.init_machine   = mxc_board_init,
	.timer          = &tqma35_timer,
MACHINE_END
