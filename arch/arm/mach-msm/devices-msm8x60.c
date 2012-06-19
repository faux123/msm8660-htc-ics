/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/ion.h>
#include <mach/irqs.h>
#include <mach/dma.h>
#include <asm/mach/mmc.h>
#include <asm/clkdev.h>
#include <linux/msm_kgsl.h>
#include <linux/msm_rotator.h>
#include <mach/msm_hsusb.h>
#include "footswitch.h"
#include "clock.h"
#include "clock-rpm.h"
#include "clock-voter.h"
#include "devices.h"
#include "devices-msm8x60.h"
#include <linux/mfd/pmic8058.h>
#include <linux/pmic8058-xoadc.h>
#include <linux/msm_adc.h>
#include <linux/dma-mapping.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>
#include <asm/clkdev.h>
#include <asm/mach/flash.h>
#include <mach/msm_serial_hs_lite.h>
#include <mach/msm_bus.h>
#include <mach/msm_bus_board.h>
#include <mach/socinfo.h>
#include <mach/msm_memtypes.h>
#include <mach/msm_tsif.h>
#include <mach/scm-io.h>
#ifdef CONFIG_MSM_DSPS
#include <mach/msm_dsps.h>
#endif
#include <linux/android_pmem.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <mach/mdm.h>
#include <mach/rpm.h>
#include <mach/board.h>
#include <sound/apr_audio.h>
#include "rpm_stats.h"
#include "mpm.h"
#include <mach/msm_watchdog.h>
#include <mach/msm_dcvs.h>

/* Address of GSBI blocks */
#define MSM_GSBI1_PHYS	0x16000000
#define MSM_GSBI2_PHYS	0x16100000
#define MSM_GSBI3_PHYS	0x16200000
#define MSM_GSBI4_PHYS	0x16300000
#define MSM_GSBI5_PHYS	0x16400000
#define MSM_GSBI6_PHYS	0x16500000
#define MSM_GSBI7_PHYS	0x16600000
#define MSM_GSBI8_PHYS	0x19800000
#define MSM_GSBI9_PHYS	0x19900000
#define MSM_GSBI10_PHYS	0x19A00000
#define MSM_GSBI11_PHYS	0x19B00000
#define MSM_GSBI12_PHYS	0x19C00000

/* GSBI QUPe devices */
#define MSM_GSBI1_QUP_PHYS	0x16080000
#define MSM_GSBI2_QUP_PHYS	0x16180000
#define MSM_GSBI3_QUP_PHYS	0x16280000
#define MSM_GSBI4_QUP_PHYS	0x16380000
#define MSM_GSBI5_QUP_PHYS	0x16480000
#define MSM_GSBI6_QUP_PHYS	0x16580000
#define MSM_GSBI7_QUP_PHYS	0x16680000
#define MSM_GSBI8_QUP_PHYS	0x19880000
#define MSM_GSBI9_QUP_PHYS	0x19980000
#define MSM_GSBI10_QUP_PHYS	0x19A80000
#define MSM_GSBI11_QUP_PHYS	0x19B80000
#define MSM_GSBI12_QUP_PHYS	0x19C80000

/* GSBI UART devices */
#define MSM_UART1DM_PHYS    (MSM_GSBI6_PHYS + 0x40000)
#define INT_UART1DM_IRQ     GSBI6_UARTDM_IRQ
#define INT_UART2DM_IRQ     GSBI12_UARTDM_IRQ
#define MSM_UART2DM_PHYS    0x19C40000
#define MSM_UART3DM_PHYS    (MSM_GSBI3_PHYS + 0x40000)
#define INT_UART3DM_IRQ     GSBI3_UARTDM_IRQ
#define TCSR_BASE_PHYS      0x16b00000

#define MSM_UART11DM_PHYS    (MSM_GSBI11_PHYS + 0x40000)
#define INT_UART11DM_IRQ     GSBI11_UARTDM_IRQ

/* PRNG device */
#define MSM_PRNG_PHYS		0x16C00000
#define MSM_UART9DM_PHYS    (MSM_GSBI9_PHYS + 0x40000)
#define INT_UART9DM_IRQ     GSBI9_UARTDM_IRQ

struct platform_device msm_camera_sensor_webcam;

#ifdef CONFIG_MSM_DSPS
#define GSBI12_DEV (&msm_dsps_device.dev)
#else
#define GSBI12_DEV (&msm_gsbi12_qup_i2c_device.dev)
#endif

struct _irq_state *irq_count_info_ptr;
struct _handle_irq *handle_irq;

void __init msm8x60_init_irq(void)
{
	unsigned int i;

	msm_mpm_irq_extn_init();
	gic_init(0, GIC_PPI_START, MSM_QGIC_DIST_BASE, (void *)MSM_QGIC_CPU_BASE);

	/* Edge trigger PPIs except AVS_SVICINT and AVS_SVICINTSWDONE */
	writel(0xFFFFD7FF, MSM_QGIC_DIST_BASE + GIC_DIST_CONFIG + 4);

	irq_count_info_ptr = MSM_IRQ_COUNT_ADDR;
	handle_irq = MSM_DO_IRQ_INFO_BASE;

	/* FIXME: Not installing AVS_SVICINT and AVS_SVICINTSWDONE yet
	 * as they are configured as level, which does not play nice with
	 * handle_percpu_irq.
	 */
	for (i = GIC_PPI_START; i < GIC_SPI_START; i++) {
		if (i != AVS_SVICINT && i != AVS_SVICINTSWDONE)
			irq_set_handler(i, handle_percpu_irq);
	}
}

static struct resource msm_uart1_dm_resources[] = {
	{
		.start = MSM_UART1DM_PHYS,
		.end   = MSM_UART1DM_PHYS + PAGE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART1DM_IRQ,
		.end   = INT_UART1DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		/* GSBI6 is UARTDM1 */
		.start = MSM_GSBI6_PHYS,
		.end   = MSM_GSBI6_PHYS + 4 - 1,
		.name  = "gsbi_resource",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = DMOV_HSUART1_TX_CHAN,
		.end   = DMOV_HSUART1_RX_CHAN,
		.name  = "uartdm_channels",
		.flags = IORESOURCE_DMA,
	},
	{
		.start = DMOV_HSUART1_TX_CRCI,
		.end   = DMOV_HSUART1_RX_CRCI,
		.name  = "uartdm_crci",
		.flags = IORESOURCE_DMA,
	},
};

static u64 msm_uart_dm1_dma_mask = DMA_BIT_MASK(32);

struct platform_device msm_device_uart_dm1 = {
	.name = "msm_serial_hs",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_uart1_dm_resources),
	.resource = msm_uart1_dm_resources,
	.dev            = {
		.dma_mask = &msm_uart_dm1_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource msm_uart3_dm_resources[] = {
	{
		.start = MSM_UART3DM_PHYS,
		.end   = MSM_UART3DM_PHYS + PAGE_SIZE - 1,
		.name  = "uartdm_resource",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART3DM_IRQ,
		.end   = INT_UART3DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MSM_GSBI3_PHYS,
		.end   = MSM_GSBI3_PHYS + PAGE_SIZE - 1,
		.name  = "gsbi_resource",
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device msm_device_uart_dm3 = {
	.name = "msm_serial_hsl",
	.id = 2,
	.num_resources = ARRAY_SIZE(msm_uart3_dm_resources),
	.resource = msm_uart3_dm_resources,
};

static struct resource msm_uart11_dm_resources[] = {
	{
		.start = MSM_UART11DM_PHYS,
		.end   = MSM_UART11DM_PHYS + PAGE_SIZE - 1,
		.name  = "uartdm_resource",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART11DM_IRQ,
		.end   = INT_UART11DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		/* GSBI 11 is UART_GSBI11 */
		.start = MSM_GSBI11_PHYS,
		.end   = MSM_GSBI11_PHYS + PAGE_SIZE - 1,
		.name  = "gsbi_resource",
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device msm_device_uart_dm11 = {
	.name = "msm_serial_hsl",
	.id = 3,
	.num_resources = ARRAY_SIZE(msm_uart11_dm_resources),
	.resource = msm_uart11_dm_resources,
};

static struct resource msm_uart12_dm_resources[] = {
	{
		.start = MSM_UART2DM_PHYS,
		.end   = MSM_UART2DM_PHYS + PAGE_SIZE - 1,
		.name  = "uartdm_resource",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART2DM_IRQ,
		.end   = INT_UART2DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		/* GSBI 12 is UARTDM2 */
		.start = MSM_GSBI12_PHYS,
		.end   = MSM_GSBI12_PHYS + PAGE_SIZE - 1,
		.name  = "gsbi_resource",
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device msm_device_uart_dm12 = {
	.name = "msm_serial_hsl",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_uart12_dm_resources),
	.resource = msm_uart12_dm_resources,
};

#ifdef CONFIG_MSM_GSBI9_UART
static struct msm_serial_hslite_platform_data uart_gsbi9_pdata = {
	.config_gpio	= 1,
	.uart_tx_gpio	= 67,
	.uart_rx_gpio	= 66,
};

static struct resource msm_uart_gsbi9_resources[] = {
       {
		.start	= MSM_UART9DM_PHYS,
		.end	= MSM_UART9DM_PHYS + PAGE_SIZE - 1,
		.name	= "uartdm_resource",
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_UART9DM_IRQ,
		.end	= INT_UART9DM_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		/* GSBI 9 is UART_GSBI9 */
		.start	= MSM_GSBI9_PHYS,
		.end	= MSM_GSBI9_PHYS + PAGE_SIZE - 1,
		.name	= "gsbi_resource",
		.flags	= IORESOURCE_MEM,
	},
};
struct platform_device *msm_device_uart_gsbi9;
struct platform_device *msm_add_gsbi9_uart(void)
{
	return platform_device_register_resndata(NULL, "msm_serial_hsl",
					1, msm_uart_gsbi9_resources,
					ARRAY_SIZE(msm_uart_gsbi9_resources),
					&uart_gsbi9_pdata,
					sizeof(uart_gsbi9_pdata));
}
#endif

static struct resource gsbi2_qup_spi_resources[] = {
	{
		.name   = "spi_base",
		.start  = MSM_GSBI2_QUP_PHYS,
		.end    = MSM_GSBI2_QUP_PHYS + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "gsbi_base",
		.start  = MSM_GSBI2_PHYS,
		.end    = MSM_GSBI2_PHYS + 4 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "spi_irq_in",
		.start  = GSBI2_QUP_IRQ,
		.end    = GSBI2_QUP_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spidm_channels",
		.start  = 5,
		.end    = 6,
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spidm_crci",
		.start  = 8,
		.end    = 7,
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spi_clk",
		.start  = 40,
		.end    = 40,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "spi_cs",
		.start  = 39,
		.end    = 39,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "spi_miso",
		.start  = 38,
		.end    = 38,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "spi_mosi",
		.start  = 37,
		.end    = 37,
		.flags  = IORESOURCE_IO,
	},
};

/* Use GSBI2 QUP for SPI-1 */
struct platform_device msm_gsbi2_qup_spi_device = {
	.name       = "spi_qsd",
	.id     = 1,
	.num_resources  = ARRAY_SIZE(gsbi2_qup_spi_resources),
	.resource   = gsbi2_qup_spi_resources,
};

static struct resource gsbi3_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI3_QUP_PHYS,
		.end	= MSM_GSBI3_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI3_PHYS,
		.end	= MSM_GSBI3_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI3_QUP_IRQ,
		.end	= GSBI3_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "i2c_clk",
		.start	= 44,
		.end	= 44,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "i2c_sda",
		.start	= 43,
		.end	= 43,
		.flags	= IORESOURCE_IO,
	},
};

static struct resource gsbi3_qup_spi_resources[] = {
	{
		.name   = "spi_base",
		.start  = MSM_GSBI3_QUP_PHYS,
		.end    = MSM_GSBI3_QUP_PHYS + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "gsbi_base",
		.start  = MSM_GSBI3_PHYS,
		.end    = MSM_GSBI3_PHYS + 4 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "spi_irq_in",
		.start  = GSBI3_QUP_IRQ,
		.end    = GSBI3_QUP_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spidm_channels",
		.start  = 5,
		.end    = 6,
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spidm_crci",
		.start  = 8,
		.end    = 7,
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spi_clk",
		.start  = 44,
		.end    = 44,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "spi_cs2",
		.start  = 43,
		.end    = 43,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "spi_miso",
		.start  = 42,
		.end    = 42,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "spi_mosi",
		.start  = 41,
		.end    = 41,
		.flags  = IORESOURCE_IO,
	},
};

/* Use GSBI3 QUP for SPI-2 */
struct platform_device msm_gsbi3_qup_spi_device = {
	.name           = "spi_qsd",
	.id             = 2,
	.num_resources  = ARRAY_SIZE(gsbi3_qup_spi_resources),
	.resource       = gsbi3_qup_spi_resources,
};

static struct resource gsbi8_qup_spi_resources[] = {
	{
		.name	= "spi_base",
		.start	= MSM_GSBI8_QUP_PHYS,
		.end	= MSM_GSBI8_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_base",
		.start	= MSM_GSBI8_PHYS,
		.end	= MSM_GSBI8_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "spi_irq_in",
		.start	= GSBI8_QUP_IRQ,
		.end	= GSBI8_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spidm_channels",
		.start  = 5,
		.end    = 6,
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spidm_crci",
		.start  = 8,
		.end    = 7,
		.flags  = IORESOURCE_DMA,
	},
};

/* Use GSBI8 QUP for SPI-3 */
struct platform_device msm_gsbi8_qup_spi_device = {
	.name		= "spi_qsd",
	.id		= 3,
	.num_resources	= ARRAY_SIZE(gsbi8_qup_spi_resources),
	.resource	= gsbi8_qup_spi_resources,
};


static struct resource gsbi4_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI4_QUP_PHYS,
		.end	= MSM_GSBI4_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI4_PHYS,
		.end	= MSM_GSBI4_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI4_QUP_IRQ,
		.end	= GSBI4_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "i2c_clk",
		.start	= 48,
		.end	= 48,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "i2c_sda",
		.start	= 47,
		.end	= 47,
		.flags	= IORESOURCE_IO,
	},
};

static struct resource gsbi5_qup_i2c_resources[] = {
	{
		.name   = "qup_phys_addr",
		.start  = MSM_GSBI5_QUP_PHYS,
		.end    = MSM_GSBI5_QUP_PHYS + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "gsbi_qup_i2c_addr",
		.start  = MSM_GSBI5_PHYS,
		.end    = MSM_GSBI5_PHYS + 4 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "qup_err_intr",
		.start  = GSBI5_QUP_IRQ,
		.end    = GSBI5_QUP_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name	= "i2c_clk",
		.start	= 52,
		.end	= 52,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "i2c_sda",
		.start	= 51,
		.end	= 51,
		.flags	= IORESOURCE_IO,
	},
};

static struct resource gsbi7_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI7_QUP_PHYS,
		.end	= MSM_GSBI7_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI7_PHYS,
		.end	= MSM_GSBI7_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI7_QUP_IRQ,
		.end	= GSBI7_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "i2c_clk",
		.start	= 60,
		.end	= 60,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "i2c_sda",
		.start	= 59,
		.end	= 59,
		.flags	= IORESOURCE_IO,
	},
};

static struct resource gsbi8_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI8_QUP_PHYS,
		.end	= MSM_GSBI8_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI8_PHYS,
		.end	= MSM_GSBI8_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI8_QUP_IRQ,
		.end	= GSBI8_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "i2c_clk",
		.start	= 65,
		.end	= 65,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "i2c_sda",
		.start	= 64,
		.end	= 64,
		.flags	= IORESOURCE_IO,
	},
};

static struct resource gsbi9_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI9_QUP_PHYS,
		.end	= MSM_GSBI9_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI9_PHYS,
		.end	= MSM_GSBI9_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI9_QUP_IRQ,
		.end	= GSBI9_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource gsbi10_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI10_QUP_PHYS,
		.end	= MSM_GSBI10_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI10_PHYS,
		.end	= MSM_GSBI10_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI10_QUP_IRQ,
		.end	= GSBI10_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "i2c_clk",
		.start	= 73,
		.end	= 73,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "i2c_sda",
		.start	= 72,
		.end	= 72,
		.flags	= IORESOURCE_IO,
	},
};


static struct resource gsbi12_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI12_QUP_PHYS,
		.end	= MSM_GSBI12_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI12_PHYS,
		.end	= MSM_GSBI12_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI12_QUP_IRQ,
		.end	= GSBI12_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "i2c_clk",
		.start	= 115,
		.end	= 115,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "i2c_sda",
		.start	= 116,
		.end	= 116,
		.flags	= IORESOURCE_IO,
	},
};

#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors grp3d_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_3D,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors grp3d_low_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_3D,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = KGSL_CONVERT_TO_MBPS(990),
	},
};

static struct msm_bus_vectors grp3d_nominal_low_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_3D,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = KGSL_CONVERT_TO_MBPS(1300),
	},
};

#ifdef CONFIG_GPU_OVERCLOCK
static struct msm_bus_vectors grp3d_nominal_high_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_3D,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = KGSL_CONVERT_TO_MBPS(2484),
	},
};

static struct msm_bus_vectors grp3d_max_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_3D,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = KGSL_CONVERT_TO_MBPS(2976),
	},
};
#else
static struct msm_bus_vectors grp3d_nominal_high_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_3D,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = KGSL_CONVERT_TO_MBPS(2008),
	},
};

static struct msm_bus_vectors grp3d_max_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_3D,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = KGSL_CONVERT_TO_MBPS(2484),
	},
};
#endif

static struct msm_bus_paths grp3d_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(grp3d_init_vectors),
		grp3d_init_vectors,
	},
	{
		ARRAY_SIZE(grp3d_low_vectors),
		grp3d_low_vectors,
	},
	{
		ARRAY_SIZE(grp3d_nominal_low_vectors),
		grp3d_nominal_low_vectors,
	},
	{
		ARRAY_SIZE(grp3d_nominal_high_vectors),
		grp3d_nominal_high_vectors,
	},
	{
		ARRAY_SIZE(grp3d_max_vectors),
		grp3d_max_vectors,
	},
};

static struct msm_bus_scale_pdata grp3d_bus_scale_pdata = {
	grp3d_bus_scale_usecases,
	ARRAY_SIZE(grp3d_bus_scale_usecases),
	.name = "grp3d",
};

static struct msm_bus_vectors grp2d0_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_2D_CORE0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors grp2d0_nominal_low_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_2D_CORE0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = KGSL_CONVERT_TO_MBPS(990),
	},
};

static struct msm_bus_vectors grp2d0_nominal_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_2D_CORE0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = KGSL_CONVERT_TO_MBPS(1638),
	},
};

static struct msm_bus_vectors grp2d0_max_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_2D_CORE0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = KGSL_CONVERT_TO_MBPS(2048),
	},
};

static struct msm_bus_paths grp2d0_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(grp2d0_init_vectors),
		grp2d0_init_vectors,
	},
	{
		ARRAY_SIZE(grp2d0_nominal_low_vectors),
		grp2d0_nominal_vectors,
	},
	{
		ARRAY_SIZE(grp2d0_nominal_vectors),
		grp2d0_nominal_vectors,
	},
	{
		ARRAY_SIZE(grp2d0_max_vectors),
		grp2d0_max_vectors,
	},
};

static struct msm_bus_scale_pdata grp2d0_bus_scale_pdata = {
	grp2d0_bus_scale_usecases,
	ARRAY_SIZE(grp2d0_bus_scale_usecases),
	.name = "grp2d0",
};

static struct msm_bus_vectors grp2d1_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_2D_CORE1,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
};

static struct msm_bus_vectors grp2d1_nominal_low_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_2D_CORE1,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = KGSL_CONVERT_TO_MBPS(990),
	},
};

static struct msm_bus_vectors grp2d1_nominal_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_2D_CORE1,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = KGSL_CONVERT_TO_MBPS(1638),
	},
};

static struct msm_bus_vectors grp2d1_max_vectors[] = {
	{
		.src = MSM_BUS_MASTER_GRAPHICS_2D_CORE1,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = KGSL_CONVERT_TO_MBPS(2048),
	},
};

static struct msm_bus_paths grp2d1_bus_scale_usecases[] = {
	{
		ARRAY_SIZE(grp2d1_init_vectors),
		grp2d1_init_vectors,
	},
	{
		ARRAY_SIZE(grp2d1_nominal_low_vectors),
		grp2d1_nominal_vectors,
	},
	{
		ARRAY_SIZE(grp2d1_nominal_vectors),
		grp2d1_nominal_vectors,
	},
	{
		ARRAY_SIZE(grp2d1_max_vectors),
		grp2d1_max_vectors,
	},
};

static struct msm_bus_scale_pdata grp2d1_bus_scale_pdata = {
	grp2d1_bus_scale_usecases,
	ARRAY_SIZE(grp2d1_bus_scale_usecases),
	.name = "grp2d1",
};
#endif

#ifdef CONFIG_HW_RANDOM_MSM
static struct resource rng_resources = {
	.flags = IORESOURCE_MEM,
	.start = MSM_PRNG_PHYS,
	.end   = MSM_PRNG_PHYS + SZ_512 - 1,
};

struct platform_device msm_device_rng = {
	.name          = "msm_rng",
	.id            = 0,
	.num_resources = 1,
	.resource      = &rng_resources,
};
#endif

static struct resource kgsl_3d0_resources[] = {
	{
		.name = KGSL_3D0_REG_MEMORY,
		.start = 0x04300000, /* GFX3D address */
		.end = 0x0431ffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = KGSL_3D0_IRQ,
		.start = GFX3D_IRQ,
		.end = GFX3D_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

#ifdef CONFIG_GPU_OVERCLOCK
static struct kgsl_device_platform_data kgsl_3d0_pdata = {
	.pwrlevel = {
		{
			.gpu_freq = 300000000,
			.bus_freq = 4,
			.io_fraction = 0,
		},
		{
			.gpu_freq = 266667000,
			.bus_freq = 3,
			.io_fraction = 0,
		},
		{
			.gpu_freq = 200000000,
			.bus_freq = 2,
			.io_fraction = 100,
		},
		{
			.gpu_freq = 177778000,
			.bus_freq = 1,
			.io_fraction = 100,
		},
		{
			.gpu_freq = 27000000,
			.bus_freq = 0,
		},
	},
	.init_level = 0,
	.num_levels = 5,
	.set_grp_async = NULL,
	.idle_timeout = HZ/5,
	.nap_allowed = true,
	.clk_map = KGSL_CLK_CORE | KGSL_CLK_IFACE | KGSL_CLK_MEM_IFACE,
#ifdef CONFIG_MSM_BUS_SCALING
	.bus_scale_table = &grp3d_bus_scale_pdata,
#endif
};
#else
static struct kgsl_device_platform_data kgsl_3d0_pdata = {
	.pwrlevel = {
		{
			.gpu_freq = 266667000,
			.bus_freq = 4,
			.io_fraction = 0,
		},
		{
			.gpu_freq = 228571000,
			.bus_freq = 3,
			.io_fraction = 33,
		},
		{
			.gpu_freq = 200000000,
			.bus_freq = 2,
			.io_fraction = 100,
		},
		{
			.gpu_freq = 177778000,
			.bus_freq = 1,
			.io_fraction = 100,
		},
		{
			.gpu_freq = 27000000,
			.bus_freq = 0,
		},
	},
	.init_level = 0,
	.num_levels = 5,
	.set_grp_async = NULL,
	.idle_timeout = HZ/5,
	.nap_allowed = true,
	.clk_map = KGSL_CLK_CORE | KGSL_CLK_IFACE | KGSL_CLK_MEM_IFACE,
#ifdef CONFIG_MSM_BUS_SCALING
	.bus_scale_table = &grp3d_bus_scale_pdata,
#endif
};
#endif

struct platform_device msm_kgsl_3d0 = {
	.name = "kgsl-3d0",
	.id = 0,
	.num_resources = ARRAY_SIZE(kgsl_3d0_resources),
	.resource = kgsl_3d0_resources,
	.dev = {
		.platform_data = &kgsl_3d0_pdata,
	},
};

static struct resource kgsl_2d0_resources[] = {
	{
		.name = KGSL_2D0_REG_MEMORY,
		.start = 0x04100000, /* Z180 base address */
		.end = 0x04100FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = KGSL_2D0_IRQ,
		.start = GFX2D0_IRQ,
		.end = GFX2D0_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct kgsl_device_platform_data kgsl_2d0_pdata = {
	.pwrlevel = {
		{
			.gpu_freq = 228571000,
			.bus_freq = 3,
		},
		{
			.gpu_freq = 177778000,
			.bus_freq = 2,
		},
		{
			.gpu_freq = 128000000,
			.bus_freq = 1,
		},
		{
			.gpu_freq = 27000000,
			.bus_freq = 0,
		},
	},
	.init_level = 0,
	.num_levels = 4,
	.set_grp_async = NULL,
	.idle_timeout = HZ/5,
	.nap_allowed = true,
	.clk_map = KGSL_CLK_CORE | KGSL_CLK_IFACE,
#ifdef CONFIG_MSM_BUS_SCALING
	.bus_scale_table = &grp2d0_bus_scale_pdata,
#endif
};

struct platform_device msm_kgsl_2d0 = {
	.name = "kgsl-2d0",
	.id = 0,
	.num_resources = ARRAY_SIZE(kgsl_2d0_resources),
	.resource = kgsl_2d0_resources,
	.dev = {
		.platform_data = &kgsl_2d0_pdata,
	},
};

static struct resource kgsl_2d1_resources[] = {
	{
		.name = KGSL_2D1_REG_MEMORY,
		.start = 0x04200000, /* Z180 device 1 base address */
		.end =   0x04200FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = KGSL_2D1_IRQ,
		.start = GFX2D1_IRQ,
		.end = GFX2D1_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct kgsl_device_platform_data kgsl_2d1_pdata = {
	.pwrlevel = {
		{
			.gpu_freq = 200000000,
			.bus_freq = 3,
		},
		{
			.gpu_freq = 160000000,
			.bus_freq = 2,
		},
		{
			.gpu_freq = 96000000,
			.bus_freq = 1,
		},
		{
			.gpu_freq = 27000000,
			.bus_freq = 0,
		},
	},
	.init_level = 0,
	.num_levels = 4,
	.set_grp_async = NULL,
	.idle_timeout = HZ/5,
	.nap_allowed = true,
	.clk_map = KGSL_CLK_CORE | KGSL_CLK_IFACE,
#ifdef CONFIG_MSM_BUS_SCALING
	.bus_scale_table = &grp2d1_bus_scale_pdata,
#endif
};

struct platform_device msm_kgsl_2d1 = {
	.name = "kgsl-2d1",
	.id = 1,
	.num_resources = ARRAY_SIZE(kgsl_2d1_resources),
	.resource = kgsl_2d1_resources,
	.dev = {
		.platform_data = &kgsl_2d1_pdata,
	},
};

/*
 * this a software workaround for not having two distinct board
 * files for 8660v1 and 8660v2. 8660v1 has a faulty 2d clock, and
 * this workaround detects the cpu version to tell if the kernel is on a
 * 8660v1, and should disable the 2d core. it is called from the board file
 */
void __init msm8x60_check_2d_hardware(void)
{
	if ((SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 1) &&
	    (SOCINFO_VERSION_MINOR(socinfo_get_version()) == 0)) {
		printk(KERN_WARNING "kgsl: 2D cores disabled on 8660v1\n");
		kgsl_2d0_pdata.clk_map = 0;
	}
}

/* Use GSBI3 QUP for /dev/i2c-0 */
struct platform_device msm_gsbi3_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI3_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi3_qup_i2c_resources),
	.resource	= gsbi3_qup_i2c_resources,
};

/* Use GSBI4 QUP for /dev/i2c-1 */
struct platform_device msm_gsbi4_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI4_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi4_qup_i2c_resources),
	.resource	= gsbi4_qup_i2c_resources,
};

/* Use GSBI5 QUP for /dev/i2c-9 */
struct platform_device msm_gsbi5_qup_i2c_device = {
	.name           = "qup_i2c",
	.id             = MSM_GSBI5_QUP_I2C_BUS_ID,
	.num_resources  = ARRAY_SIZE(gsbi5_qup_i2c_resources),
	.resource       = gsbi5_qup_i2c_resources,
};

/* Use GSBI8 QUP for /dev/i2c-3 */
struct platform_device msm_gsbi8_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI8_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi8_qup_i2c_resources),
	.resource	= gsbi8_qup_i2c_resources,
};

/* Use GSBI9 QUP for /dev/i2c-2 */
struct platform_device msm_gsbi9_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI9_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi9_qup_i2c_resources),
	.resource	= gsbi9_qup_i2c_resources,
};

/* Use GSBI7 QUP for /dev/i2c-4 (Marimba) */
struct platform_device msm_gsbi7_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI7_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi7_qup_i2c_resources),
	.resource	= gsbi7_qup_i2c_resources,
};

/* Use GSBI10 QUP for /dev/i2c-5 */
struct platform_device msm_gsbi10_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI10_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi10_qup_i2c_resources),
	.resource	= gsbi10_qup_i2c_resources,
};


/* Use GSBI12 QUP for /dev/i2c-10 */
struct platform_device msm_gsbi12_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= MSM_GSBI12_QUP_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(gsbi12_qup_i2c_resources),
	.resource	= gsbi12_qup_i2c_resources,
};

#ifdef CONFIG_MSM_SSBI
/* 8058 PMIC SSBI */
#define MSM_SSBI_PMIC1_PHYS	0x00500000
static struct resource resources_ssbi_pmic1_resource[] = {
	{
		.start  = MSM_SSBI_PMIC1_PHYS,
		.end    = MSM_SSBI_PMIC1_PHYS + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

struct platform_device msm_device_ssbi_pmic1 = {
	.name           = "msm_ssbi",
	.id             = 0,
	.resource       = resources_ssbi_pmic1_resource,
	.num_resources  = ARRAY_SIZE(resources_ssbi_pmic1_resource),
};
/* 8901 PMIC SSBI */
#define MSM_SSBI_PMIC2_PHYS	0x00C00000
static struct resource resources_ssbi_pmic2_resource[] = {
	{
		.start	= MSM_SSBI_PMIC2_PHYS,
		.end	= MSM_SSBI_PMIC2_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_ssbi_pmic2 = {
	.name           = "msm_ssbi",
	.id             = 1,
	.resource       = resources_ssbi_pmic2_resource,
	.num_resources	= ARRAY_SIZE(resources_ssbi_pmic2_resource),
};
#endif

#ifdef CONFIG_I2C_SSBI
/* 8901 PMIC SSBI on /dev/i2c-7 */
#define MSM_SSBI2_PMIC2B_PHYS	0x00C00000
static struct resource msm_ssbi2_resources[] = {
	{
		.name   = "ssbi_base",
		.start	= MSM_SSBI2_PMIC2B_PHYS,
		.end	= MSM_SSBI2_PMIC2B_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_ssbi2 = {
	.name		= "i2c_ssbi",
	.id		= MSM_SSBI2_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(msm_ssbi2_resources),
	.resource	= msm_ssbi2_resources,
};

/* CODEC SSBI on /dev/i2c-8 */
#define MSM_SSBI3_PHYS  0x18700000
static struct resource msm_ssbi3_resources[] = {
	{
		.name   = "ssbi_base",
		.start  = MSM_SSBI3_PHYS,
		.end    = MSM_SSBI3_PHYS + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

struct platform_device msm_device_ssbi3 = {
	.name		= "i2c_ssbi",
	.id		= MSM_SSBI3_I2C_BUS_ID,
	.num_resources	= ARRAY_SIZE(msm_ssbi3_resources),
	.resource	= msm_ssbi3_resources,
};
#endif /* CONFIG_I2C_SSBI */

#ifdef CONFIG_MSM_SSBI
struct platform_device msm_device_pm8058 = {
	.name	= "pm8058-core",
	.id		= 0,
};

struct platform_device msm_device_pm8901 = {
	.name	= "pm8901-core",
	.id		= 1,
};
#endif

static struct resource gsbi1_qup_spi_resources[] = {
	{
		.name	= "spi_base",
		.start	= MSM_GSBI1_QUP_PHYS,
		.end	= MSM_GSBI1_QUP_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_base",
		.start	= MSM_GSBI1_PHYS,
		.end	= MSM_GSBI1_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "spi_irq_in",
		.start	= GSBI1_QUP_IRQ,
		.end	= GSBI1_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spidm_channels",
		.start  = 5,
		.end    = 6,
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spidm_crci",
		.start  = 8,
		.end    = 7,
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spi_clk",
		.start  = 36,
		.end    = 36,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "spi_miso",
		.start  = 34,
		.end    = 34,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "spi_mosi",
		.start  = 33,
		.end    = 33,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "spi_cs",
		.start  = 35,
		.end    = 35,
		.flags  = IORESOURCE_IO,
	},
};

/* Use GSBI1 QUP for SPI-0 */
struct platform_device msm_gsbi1_qup_spi_device = {
	.name		= "spi_qsd",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(gsbi1_qup_spi_resources),
	.resource	= gsbi1_qup_spi_resources,
};

#define MSM_SDC1_BASE         0x12400000
#define MSM_SDC1_DML_BASE     (MSM_SDC1_BASE + 0x800)
#define MSM_SDC1_BAM_BASE     (MSM_SDC1_BASE + 0x2000)
#define MSM_SDC2_BASE         0x12140000
#define MSM_SDC2_DML_BASE     (MSM_SDC2_BASE + 0x800)
#define MSM_SDC2_BAM_BASE     (MSM_SDC2_BASE + 0x2000)
#define MSM_SDC3_BASE         0x12180000
#define MSM_SDC3_DML_BASE     (MSM_SDC3_BASE + 0x800)
#define MSM_SDC3_BAM_BASE     (MSM_SDC3_BASE + 0x2000)
#define MSM_SDC4_BASE         0x121C0000
#define MSM_SDC4_DML_BASE     (MSM_SDC4_BASE + 0x800)
#define MSM_SDC4_BAM_BASE     (MSM_SDC4_BASE + 0x2000)
#define MSM_SDC5_BASE         0x12200000
#define MSM_SDC5_DML_BASE     (MSM_SDC5_BASE + 0x800)
#define MSM_SDC5_BAM_BASE     (MSM_SDC5_BASE + 0x2000)

static struct resource resources_sdc1[] = {
	{
		.start	= MSM_SDC1_BASE,
		.end	= MSM_SDC1_DML_BASE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= SDC1_IRQ_0,
		.end	= SDC1_IRQ_0,
		.flags	= IORESOURCE_IRQ,
	},
#ifdef CONFIG_MMC_MSM_SPS_SUPPORT
	{
		.name   = "sdcc_dml_addr",
		.start	= MSM_SDC1_DML_BASE,
		.end	= MSM_SDC1_BAM_BASE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "sdcc_bam_addr",
		.start	= MSM_SDC1_BAM_BASE,
		.end	= MSM_SDC1_BAM_BASE + (2 * SZ_4K) - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "sdcc_bam_irq",
		.start	= SDC1_BAM_IRQ,
		.end	= SDC1_BAM_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
#else
	{
		.name	= "sdcc_dma_chnl",
		.start	= DMOV_SDC1_CHAN,
		.end	= DMOV_SDC1_CHAN,
		.flags	= IORESOURCE_DMA,
	},
	{
		.name	= "sdcc_dma_crci",
		.start	= DMOV_SDC1_CRCI,
		.end	= DMOV_SDC1_CRCI,
		.flags	= IORESOURCE_DMA,
	}
#endif /* CONFIG_MMC_MSM_SPS_SUPPORT */
};

static struct resource resources_sdc2[] = {
	{
		.start	= MSM_SDC2_BASE,
		.end	= MSM_SDC2_DML_BASE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= SDC2_IRQ_0,
		.end	= SDC2_IRQ_0,
		.flags	= IORESOURCE_IRQ,
	},
#ifdef CONFIG_MMC_MSM_SPS_SUPPORT
	{
		.name   = "sdcc_dml_addr",
		.start	= MSM_SDC2_DML_BASE,
		.end	= MSM_SDC2_BAM_BASE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "sdcc_bam_addr",
		.start	= MSM_SDC2_BAM_BASE,
		.end	= MSM_SDC2_BAM_BASE + (2 * SZ_4K) - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "sdcc_bam_irq",
		.start	= SDC2_BAM_IRQ,
		.end	= SDC2_BAM_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
#else
	{
		.name	= "sdcc_dma_chnl",
		.start	= DMOV_SDC2_CHAN,
		.end	= DMOV_SDC2_CHAN,
		.flags	= IORESOURCE_DMA,
	},
	{
		.name	= "sdcc_dma_crci",
		.start	= DMOV_SDC2_CRCI,
		.end	= DMOV_SDC2_CRCI,
		.flags	= IORESOURCE_DMA,
	}
#endif /* CONFIG_MMC_MSM_SPS_SUPPORT */
};

static struct resource resources_sdc3[] = {
	{
		.start	= MSM_SDC3_BASE,
		.end	= MSM_SDC3_DML_BASE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= SDC3_IRQ_0,
		.end	= SDC3_IRQ_0,
		.flags	= IORESOURCE_IRQ,
	},
#ifdef CONFIG_MMC_MSM_SPS_SUPPORT
	{
		.name   = "sdcc_dml_addr",
		.start	= MSM_SDC3_DML_BASE,
		.end	= MSM_SDC3_BAM_BASE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "sdcc_bam_addr",
		.start	= MSM_SDC3_BAM_BASE,
		.end	= MSM_SDC3_BAM_BASE + (2 * SZ_4K) - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "sdcc_bam_irq",
		.start	= SDC3_BAM_IRQ,
		.end	= SDC3_BAM_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
#else
	{
		.name	= "sdcc_dma_chnl",
		.start	= DMOV_SDC3_CHAN,
		.end	= DMOV_SDC3_CHAN,
		.flags	= IORESOURCE_DMA,
	},
	{
		.name	= "sdcc_dma_crci",
		.start	= DMOV_SDC3_CRCI,
		.end	= DMOV_SDC3_CRCI,
		.flags	= IORESOURCE_DMA,
	},
#endif /* CONFIG_MMC_MSM_SPS_SUPPORT */
};

static struct resource resources_sdc4[] = {
	{
		.start	= MSM_SDC4_BASE,
		.end	= MSM_SDC4_DML_BASE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= SDC4_IRQ_0,
		.end	= SDC4_IRQ_0,
		.flags	= IORESOURCE_IRQ,
	},
#ifdef CONFIG_MMC_MSM_SPS_SUPPORT
	{
		.name   = "sdcc_dml_addr",
		.start	= MSM_SDC4_DML_BASE,
		.end	= MSM_SDC4_BAM_BASE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "sdcc_bam_addr",
		.start	= MSM_SDC4_BAM_BASE,
		.end	= MSM_SDC4_BAM_BASE + (2 * SZ_4K) - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "sdcc_bam_irq",
		.start	= SDC4_BAM_IRQ,
		.end	= SDC4_BAM_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
#else
	{
		.name	= "sdcc_dma_chnl",
		.start	= DMOV_SDC4_CHAN,
		.end	= DMOV_SDC4_CHAN,
		.flags	= IORESOURCE_DMA,
	},
	{
		.name	= "sdcc_dma_crci",
		.start	= DMOV_SDC4_CRCI,
		.end	= DMOV_SDC4_CRCI,
		.flags	= IORESOURCE_DMA,
	},
#endif /* CONFIG_MMC_MSM_SPS_SUPPORT */
};

static struct resource resources_sdc5[] = {
	{
		.start	= MSM_SDC5_BASE,
		.end	= MSM_SDC5_DML_BASE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= SDC5_IRQ_0,
		.end	= SDC5_IRQ_0,
		.flags	= IORESOURCE_IRQ,
	},
#ifdef CONFIG_MMC_MSM_SPS_SUPPORT
	{
		.name   = "sdcc_dml_addr",
		.start	= MSM_SDC5_DML_BASE,
		.end	= MSM_SDC5_BAM_BASE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "sdcc_bam_addr",
		.start	= MSM_SDC5_BAM_BASE,
		.end	= MSM_SDC5_BAM_BASE + (2 * SZ_4K) - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "sdcc_bam_irq",
		.start	= SDC5_BAM_IRQ,
		.end	= SDC5_BAM_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
#else
	{
		.name	= "sdcc_dma_chnl",
		.start	= DMOV_SDC5_CHAN,
		.end	= DMOV_SDC5_CHAN,
		.flags	= IORESOURCE_DMA,
	},
	{
		.name	= "sdcc_dma_crci",
		.start	= DMOV_SDC5_CRCI,
		.end	= DMOV_SDC5_CRCI,
		.flags	= IORESOURCE_DMA,
	},
#endif /* CONFIG_MMC_MSM_SPS_SUPPORT */
};

struct platform_device msm_device_sdc1 = {
	.name		= "msm_sdcc",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(resources_sdc1),
	.resource	= resources_sdc1,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc2 = {
	.name		= "msm_sdcc",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(resources_sdc2),
	.resource	= resources_sdc2,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc3 = {
	.name		= "msm_sdcc",
	.id		= 3,
	.num_resources	= ARRAY_SIZE(resources_sdc3),
	.resource	= resources_sdc3,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc4 = {
	.name		= "msm_sdcc",
	.id		= 4,
	.num_resources	= ARRAY_SIZE(resources_sdc4),
	.resource	= resources_sdc4,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc5 = {
	.name		= "msm_sdcc",
	.id		= 5,
	.num_resources	= ARRAY_SIZE(resources_sdc5),
	.resource	= resources_sdc5,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

static struct platform_device *msm_sdcc_devices[] __initdata = {
	&msm_device_sdc1,
	&msm_device_sdc2,
	&msm_device_sdc3,
	&msm_device_sdc4,
	&msm_device_sdc5,
};

int __init msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat)
{
	struct platform_device	*pdev;

	if (controller < 1 || controller > 5)
		return -EINVAL;

	pdev = msm_sdcc_devices[controller-1];
	pdev->dev.platform_data = plat;
	return platform_device_register(pdev);
}

#define MIPI_DSI_HW_BASE	0x04700000
#define ROTATOR_HW_BASE		0x04E00000
#define TVENC_HW_BASE		0x04F00000
#define MDP_HW_BASE		0x05100000

static struct resource msm_mipi_dsi_resources[] = {
	{
		.name   = "mipi_dsi",
		.start  = MIPI_DSI_HW_BASE,
		.end    = MIPI_DSI_HW_BASE + 0x000F0000 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = DSI_IRQ,
		.end    = DSI_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_mipi_dsi_device = {
	.name   = "mipi_dsi",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_mipi_dsi_resources),
	.resource       = msm_mipi_dsi_resources,
};

static struct resource msm_mdp_resources[] = {
	{
		.name   = "mdp",
		.start  = MDP_HW_BASE,
		.end    = MDP_HW_BASE + 0x000F0000 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_MDP,
		.end    = INT_MDP,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_mdp_device = {
	.name   = "mdp",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_mdp_resources),
	.resource       = msm_mdp_resources,
};
#ifdef CONFIG_MSM_ROTATOR
static struct resource resources_msm_rotator[] = {
	{
		.start	= 0x04E00000,
		.end	= 0x04F00000 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= ROT_IRQ,
		.end	= ROT_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_rot_clocks rotator_clocks[] = {
	{
		.clk_name = "core_clk",
		.clk_type = ROTATOR_CORE_CLK,
		.clk_rate = 160 * 1000 * 1000,
	},
	{
		.clk_name = "iface_clk",
		.clk_type = ROTATOR_PCLK,
		.clk_rate = 0,
	},
};

static struct msm_rotator_platform_data rotator_pdata = {
	.number_of_clocks = ARRAY_SIZE(rotator_clocks),
	.hardware_version_number = 0x01010307,
	.rotator_clks = rotator_clocks,
	.regulator_name = "fs_rot",
};

struct platform_device msm_rotator_device = {
	.name		= "msm_rotator",
	.id		= 0,
	.num_resources  = ARRAY_SIZE(resources_msm_rotator),
	.resource       = resources_msm_rotator,
	.dev		= {
		.platform_data = &rotator_pdata,
	},
};
#endif


/* Sensors DSPS platform data */
#ifdef CONFIG_MSM_DSPS

#define PPSS_REG_PHYS_BASE	0x12080000

#define MHZ (1000*1000)

#define TCSR_GSBI_IRQ_MUX_SEL	0x0044

#define GSBI_IRQ_MUX_SEL_MASK	0xF
#define GSBI_IRQ_MUX_SEL_DSPS	0xB

static void dsps_init1(struct msm_dsps_platform_data *data)
{
	int val;

	/* route GSBI12 interrutps to DSPS */
	val = secure_readl(MSM_TCSR_BASE +  TCSR_GSBI_IRQ_MUX_SEL);
	val &= ~GSBI_IRQ_MUX_SEL_MASK;
	val |= GSBI_IRQ_MUX_SEL_DSPS;
	secure_writel(val, MSM_TCSR_BASE + TCSR_GSBI_IRQ_MUX_SEL);
}

static struct dsps_clk_info dsps_clks[] = {
	{
		.name = "iface_clk",
		.rate =	0, /* no rate just on/off */
	},
	{
		.name = "mem_clk",
		.rate =	0, /* no rate just on/off */
	},
	{
		.name = "gsbi_qup_clk",
		.rate =	24 * MHZ, /* See clk_tbl_gsbi_qup[] */
	},
	{
		.name = "dfab_dsps_clk",
		.rate =	64 * MHZ, /* Same rate as USB. */
	}
};

static struct dsps_regulator_info dsps_regs[] = {
	{
		.name = "8058_l5",
		.volt = 2850000, /* in uV */
	},
	{
		.name = "8058_s3",
		.volt = 1800000, /* in uV */
	}
};

/*
 * Note: GPIOs field is	intialized in run-time at the function
 * msm8x60_init_dsps().
 */

struct msm_dsps_platform_data msm_dsps_pdata = {
	.clks = dsps_clks,
	.clks_num = ARRAY_SIZE(dsps_clks),
	.gpios = NULL,
	.gpios_num = 0,
	.regs = dsps_regs,
	.regs_num = ARRAY_SIZE(dsps_regs),
	.init = dsps_init1,
	.signature = DSPS_SIGNATURE,
};

static struct resource msm_dsps_resources[] = {
	{
		.start = PPSS_REG_PHYS_BASE,
		.end   = PPSS_REG_PHYS_BASE + SZ_8K - 1,
		.name  = "ppss_reg",
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device msm_dsps_device = {
	.name          = "msm_dsps",
	.id            = 0,
	.num_resources = ARRAY_SIZE(msm_dsps_resources),
	.resource      = msm_dsps_resources,
	.dev.platform_data = &msm_dsps_pdata,
};

#endif /* CONFIG_MSM_DSPS */

#ifdef CONFIG_FB_MSM_TVOUT
static struct resource msm_tvenc_resources[] = {
	{
		.name   = "tvenc",
		.start  = TVENC_HW_BASE,
		.end    = TVENC_HW_BASE + PAGE_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	}
};

static struct resource tvout_device_resources[] = {
	{
		.name  = "tvout_device_irq",
		.start = TV_ENC_IRQ,
		.end   = TV_ENC_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};
#endif
static void __init msm_register_device(struct platform_device *pdev, void *data)
{
	int ret;

	pdev->dev.platform_data = data;

	ret = platform_device_register(pdev);
	if (ret)
		dev_err(&pdev->dev,
			  "%s: platform_device_register() failed = %d\n",
			  __func__, ret);
}

static struct platform_device msm_lcdc_device = {
	.name   = "lcdc",
	.id     = 0,
};

#ifdef CONFIG_FB_MSM_TVOUT
static struct platform_device msm_tvenc_device = {
	.name   = "tvenc",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_tvenc_resources),
	.resource       = msm_tvenc_resources,
};

static struct platform_device msm_tvout_device = {
	.name = "tvout_device",
	.id = 0,
	.num_resources = ARRAY_SIZE(tvout_device_resources),
	.resource = tvout_device_resources,
};
#endif

#ifdef CONFIG_MSM_BUS_SCALING
static struct platform_device msm_dtv_device = {
	.name   = "dtv",
	.id     = 0,
};
#endif

void __init msm_fb_register_device(char *name, void *data)
{
	if (!strncmp(name, "mdp", 3))
		msm_register_device(&msm_mdp_device, data);
	else if (!strncmp(name, "lcdc", 4))
		msm_register_device(&msm_lcdc_device, data);
	else if (!strncmp(name, "mipi_dsi", 8))
		msm_register_device(&msm_mipi_dsi_device, data);
#ifdef CONFIG_FB_MSM_TVOUT
	else if (!strncmp(name, "tvenc", 5))
		msm_register_device(&msm_tvenc_device, data);
	else if (!strncmp(name, "tvout_device", 12))
		msm_register_device(&msm_tvout_device, data);
#endif
#ifdef CONFIG_MSM_BUS_SCALING
	else if (!strncmp(name, "dtv", 3))
		msm_register_device(&msm_dtv_device, data);
#endif
	else
		printk(KERN_ERR "%s: unknown device! %s\n", __func__, name);
}

static struct resource resources_otg[] = {
	{
		.start	= 0x12500000,
		.end	= 0x12500000 + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= USB1_HS_IRQ,
		.end	= USB1_HS_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_otg = {
	.name		= "msm_otg",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_otg),
	.resource	= resources_otg,
};

static struct resource resources_hsusb[] = {
	{
		.start	= 0x12500000,
		.end	= 0x12500000 + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= USB1_HS_IRQ,
		.end	= USB1_HS_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};
static u64 dma_mask = 0xffffffffULL;
struct platform_device msm_device_gadget_peripheral = {
	.name		= "msm_hsusb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_hsusb),
	.resource	= resources_hsusb,
	.dev		= {
		.dma_mask 		= &dma_mask,
		.coherent_dma_mask	= 0xffffffffULL,
	},
};
#ifdef CONFIG_USB_EHCI_MSM_72K
static struct resource resources_hsusb_host[] = {
	{
		.start	= 0x12500000,
		.end	= 0x12500000 + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= USB1_HS_IRQ,
		.end	= USB1_HS_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_hsusb_host = {
	.name		= "msm_hsusb_host",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_hsusb_host),
	.resource	= resources_hsusb_host,
	.dev		= {
		.dma_mask 		= &dma_mask,
		.coherent_dma_mask	= 0xffffffffULL,
	},
};

static struct platform_device *msm_host_devices[] = {
	&msm_device_hsusb_host,
};

int msm_add_host(unsigned int host, struct msm_usb_host_platform_data *plat)
{
	struct platform_device	*pdev;

	pdev = msm_host_devices[host];
	if (!pdev)
		return -ENODEV;
	pdev->dev.platform_data = plat;
	return platform_device_register(pdev);
}
#endif

#define MSM_TSIF0_PHYS       (0x18200000)
#define MSM_TSIF1_PHYS       (0x18201000)
#define MSM_TSIF_SIZE        (0x200)
#define TCSR_ADM_0_A_CRCI_MUX_SEL 0x0070

#define TSIF_0_CLK       GPIO_CFG(93, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_0_EN        GPIO_CFG(94, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_0_DATA      GPIO_CFG(95, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_0_SYNC      GPIO_CFG(96, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_1_CLK       GPIO_CFG(97, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_1_EN        GPIO_CFG(98, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_1_DATA      GPIO_CFG(99, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_1_SYNC      GPIO_CFG(100, 1, GPIO_CFG_INPUT, \
	GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static const struct msm_gpio tsif0_gpios[] = {
	{ .gpio_cfg = TSIF_0_CLK,  .label =  "tsif_clk", },
	{ .gpio_cfg = TSIF_0_EN,   .label =  "tsif_en", },
	{ .gpio_cfg = TSIF_0_DATA, .label =  "tsif_data", },
	{ .gpio_cfg = TSIF_0_SYNC, .label =  "tsif_sync", },
};

static const struct msm_gpio tsif1_gpios[] = {
	{ .gpio_cfg = TSIF_1_CLK,  .label =  "tsif_clk", },
	{ .gpio_cfg = TSIF_1_EN,   .label =  "tsif_en", },
	{ .gpio_cfg = TSIF_1_DATA, .label =  "tsif_data", },
	{ .gpio_cfg = TSIF_1_SYNC, .label =  "tsif_sync", },
};

static void tsif_release(struct device *dev)
{
}

static void tsif_init1(struct msm_tsif_platform_data *data)
{
	int val;

	/* configure mux to use correct tsif instance */
	val = secure_readl(MSM_TCSR_BASE + TCSR_ADM_0_A_CRCI_MUX_SEL);
	val |= 0x80000000;
	secure_writel(val, MSM_TCSR_BASE + TCSR_ADM_0_A_CRCI_MUX_SEL);
}

struct msm_tsif_platform_data tsif1_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif1_gpios),
	.gpios = tsif1_gpios,
	.tsif_pclk = "iface_clk",
	.tsif_ref_clk = "ref_clk",
	.init = tsif_init1
};

struct resource tsif1_resources[] = {
	[0] = {
		.flags = IORESOURCE_IRQ,
		.start = TSIF2_IRQ,
		.end   = TSIF2_IRQ,
	},
	[1] = {
		.flags = IORESOURCE_MEM,
		.start = MSM_TSIF1_PHYS,
		.end   = MSM_TSIF1_PHYS + MSM_TSIF_SIZE - 1,
	},
	[2] = {
		.flags = IORESOURCE_DMA,
		.start = DMOV_TSIF_CHAN,
		.end   = DMOV_TSIF_CRCI,
	},
};

static void tsif_init0(struct msm_tsif_platform_data *data)
{
	int val;

	/* configure mux to use correct tsif instance */
	val = secure_readl(MSM_TCSR_BASE + TCSR_ADM_0_A_CRCI_MUX_SEL);
	val &= 0x7FFFFFFF;
	secure_writel(val, MSM_TCSR_BASE + TCSR_ADM_0_A_CRCI_MUX_SEL);
}

struct msm_tsif_platform_data tsif0_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif0_gpios),
	.gpios = tsif0_gpios,
	.tsif_pclk = "iface_clk",
	.tsif_ref_clk = "ref_clk",
	.init = tsif_init0
};
struct resource tsif0_resources[] = {
	[0] = {
		.flags = IORESOURCE_IRQ,
		.start = TSIF1_IRQ,
		.end   = TSIF1_IRQ,
	},
	[1] = {
		.flags = IORESOURCE_MEM,
		.start = MSM_TSIF0_PHYS,
		.end   = MSM_TSIF0_PHYS + MSM_TSIF_SIZE - 1,
	},
	[2] = {
		.flags = IORESOURCE_DMA,
		.start = DMOV_TSIF_CHAN,
		.end   = DMOV_TSIF_CRCI,
	},
};

struct platform_device msm_device_tsif[2] = {
	{
		.name          = "msm_tsif",
		.id            = 0,
		.num_resources = ARRAY_SIZE(tsif0_resources),
		.resource      = tsif0_resources,
		.dev = {
			.release       = tsif_release,
			.platform_data = &tsif0_platform_data
		},
	},
	{
		.name          = "msm_tsif",
		.id            = 1,
		.num_resources = ARRAY_SIZE(tsif1_resources),
		.resource      = tsif1_resources,
		.dev = {
			.release       = tsif_release,
			.platform_data = &tsif1_platform_data
		},
	}
};

struct platform_device msm_device_smd = {
	.name           = "msm_smd",
	.id             = -1,
};

static struct msm_watchdog_pdata msm_watchdog_pdata = {
	.pet_time = 10000,
	.bark_time = 18000,
	.has_secure = true,
};

struct platform_device msm8660_device_watchdog = {
	.name = "msm_watchdog",
	.id = -1,
	.dev = {
		.platform_data = &msm_watchdog_pdata,
	},
};

static struct resource msm_dmov_resource_adm0[] = {
	{
		.start = INT_ADM0_AARM,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = 0x18320000,
		.end = 0x18320000 + SZ_1M - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct resource msm_dmov_resource_adm1[] = {
	{
		.start = INT_ADM1_AARM,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = 0x18420000,
		.end = 0x18420000 + SZ_1M - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct msm_dmov_pdata msm_dmov_pdata_adm0 = {
	.sd = 1,
	.sd_size = 0x800,
};

static struct msm_dmov_pdata msm_dmov_pdata_adm1 = {
	.sd = 1,
	.sd_size = 0x800,
};

struct platform_device msm_device_dmov_adm0 = {
	.name	= "msm_dmov",
	.id	= 0,
	.resource = msm_dmov_resource_adm0,
	.num_resources = ARRAY_SIZE(msm_dmov_resource_adm0),
	.dev = {
		.platform_data = &msm_dmov_pdata_adm0,
	},
};

struct platform_device msm_device_dmov_adm1 = {
	.name	= "msm_dmov",
	.id	= 1,
	.resource = msm_dmov_resource_adm1,
	.num_resources = ARRAY_SIZE(msm_dmov_resource_adm1),
	.dev = {
		.platform_data = &msm_dmov_pdata_adm1,
	},
};

/* MSM Video core device */
#ifdef CONFIG_MSM_BUS_SCALING
static struct msm_bus_vectors vidc_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT1,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab = 0,
		.ib = 0,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab = 0,
		.ib = 0,
	},
};
static struct msm_bus_vectors vidc_venc_vga_vectors[] = {
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 54525952,
		.ib  = 436207616,
	},
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT1,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 72351744,
		.ib  = 289406976,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 500000,
		.ib  = 1000000,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 500000,
		.ib  = 1000000,
	},
};
static struct msm_bus_vectors vidc_vdec_vga_vectors[] = {
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 40894464,
		.ib  = 327155712,
	},
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT1,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 48234496,
		.ib  = 192937984,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 500000,
		.ib  = 2000000,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 500000,
		.ib  = 2000000,
	},
};
static struct msm_bus_vectors vidc_venc_720p_vectors[] = {
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 163577856,
		.ib  = 1308622848,
	},
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT1,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 219152384,
		.ib  = 876609536,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 1750000,
		.ib  = 3500000,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 1750000,
		.ib  = 3500000,
	},
};
static struct msm_bus_vectors vidc_vdec_720p_vectors[] = {
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 121634816,
		.ib  = 973078528,
	},
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT1,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 155189248,
		.ib  = 620756992,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 1750000,
		.ib  = 7000000,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 1750000,
		.ib  = 7000000,
	},
};
static struct msm_bus_vectors vidc_venc_1080p_vectors[] = {
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 372244480,
		.ib  = 1861222400,
	},
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT1,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 501219328,
		.ib  = 2004877312,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 2500000,
		.ib  = 5000000,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 2500000,
		.ib  = 5000000,
	},
};
static struct msm_bus_vectors vidc_vdec_1080p_vectors[] = {
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 222298112,
		.ib  = 1778384896,
	},
	{
		.src = MSM_BUS_MASTER_HD_CODEC_PORT1,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 330301440,
		.ib  = 1321205760,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 2500000,
		.ib  = 700000000,
	},
	{
		.src = MSM_BUS_MASTER_AMPSS_M0,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 2500000,
		.ib  = 10000000,
	},
};

static struct msm_bus_paths vidc_bus_client_config[] = {
	{
		ARRAY_SIZE(vidc_init_vectors),
		vidc_init_vectors,
	},
	{
		ARRAY_SIZE(vidc_venc_vga_vectors),
		vidc_venc_vga_vectors,
	},
	{
		ARRAY_SIZE(vidc_vdec_vga_vectors),
		vidc_vdec_vga_vectors,
	},
	{
		ARRAY_SIZE(vidc_venc_720p_vectors),
		vidc_venc_720p_vectors,
	},
	{
		ARRAY_SIZE(vidc_vdec_720p_vectors),
		vidc_vdec_720p_vectors,
	},
	{
		ARRAY_SIZE(vidc_venc_1080p_vectors),
		vidc_venc_1080p_vectors,
	},
	{
		ARRAY_SIZE(vidc_vdec_1080p_vectors),
		vidc_vdec_1080p_vectors,
	},
};

static struct msm_bus_scale_pdata vidc_bus_client_data = {
	vidc_bus_client_config,
	ARRAY_SIZE(vidc_bus_client_config),
	.name = "vidc",
};

#endif

#define MSM_VIDC_BASE_PHYS 0x04400000
#define MSM_VIDC_BASE_SIZE 0x00100000

static struct resource msm_device_vidc_resources[] = {
	{
		.start	= MSM_VIDC_BASE_PHYS,
		.end	= MSM_VIDC_BASE_PHYS + MSM_VIDC_BASE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= VCODEC_IRQ,
		.end	= VCODEC_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct msm_vidc_platform_data vidc_platform_data = {
#ifdef CONFIG_MSM_BUS_SCALING
	.vidc_bus_client_pdata = &vidc_bus_client_data,
#endif
	.memtype = MEMTYPE_SMI_KERNEL,
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
	.enable_ion = 1,
#else
	.enable_ion = 0,
#endif
	.disable_dmx = 0,
	.disable_fullhd = 0
};

struct platform_device msm_device_vidc = {
	.name = "msm_vidc",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_device_vidc_resources),
	.resource = msm_device_vidc_resources,
	.dev = {
		.platform_data	= &vidc_platform_data,
	},
};

#if defined(CONFIG_MSM_RPM_STATS_LOG)
static struct msm_rpmstats_platform_data msm_rpm_stat_pdata = {
	.phys_addr_base = 0x00107E04,
	.phys_size = SZ_8K,
};

struct platform_device msm_rpm_stat_device = {
	.name = "msm_rpm_stat",
	.id = -1,
	.dev = {
		.platform_data = &msm_rpm_stat_pdata,
	},
};
#endif

#define SHARED_IMEM_TZ_BASE 0x2a05f720
static struct resource tzlog_resources[] = {
	{
		.start = SHARED_IMEM_TZ_BASE,
		.end = SHARED_IMEM_TZ_BASE + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct platform_device msm_device_tz_log = {
	.name		= "tz_log",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(tzlog_resources),
	.resource	= tzlog_resources,
};

#ifdef CONFIG_MSM_MPM
static uint16_t msm_mpm_irqs_m2a[MSM_MPM_NR_MPM_IRQS] = {
	[1] = MSM_GPIO_TO_INT(61),
	[4] = MSM_GPIO_TO_INT(87),
	[5] = MSM_GPIO_TO_INT(88),
	[6] = MSM_GPIO_TO_INT(89),
	[7] = MSM_GPIO_TO_INT(90),
	[8] = MSM_GPIO_TO_INT(91),
	[9] = MSM_GPIO_TO_INT(34),
	[10] = MSM_GPIO_TO_INT(38),
	[11] = MSM_GPIO_TO_INT(42),
	[12] = MSM_GPIO_TO_INT(46),
	[13] = MSM_GPIO_TO_INT(50),
	[14] = MSM_GPIO_TO_INT(54),
	[15] = MSM_GPIO_TO_INT(58),
	[16] = MSM_GPIO_TO_INT(63),
	[17] = MSM_GPIO_TO_INT(160),
	[18] = MSM_GPIO_TO_INT(162),
	[19] = MSM_GPIO_TO_INT(144),
	[20] = MSM_GPIO_TO_INT(146),
	[25] = USB1_HS_IRQ,
	[26] = TV_ENC_IRQ,
	[27] = HDMI_IRQ,
	[29] = MSM_GPIO_TO_INT(123),
	[30] = MSM_GPIO_TO_INT(172),
	[31] = MSM_GPIO_TO_INT(99),
	[32] = MSM_GPIO_TO_INT(96),
	[33] = MSM_GPIO_TO_INT(67),
	[34] = MSM_GPIO_TO_INT(71),
	[35] = MSM_GPIO_TO_INT(105),
	[36] = MSM_GPIO_TO_INT(117),
	[37] = MSM_GPIO_TO_INT(29),
	[38] = MSM_GPIO_TO_INT(30),
	[39] = MSM_GPIO_TO_INT(31),
	[40] = MSM_GPIO_TO_INT(37),
	[41] = MSM_GPIO_TO_INT(40),
	[42] = MSM_GPIO_TO_INT(41),
	[43] = MSM_GPIO_TO_INT(45),
	[44] = MSM_GPIO_TO_INT(51),
	[45] = MSM_GPIO_TO_INT(52),
	[46] = MSM_GPIO_TO_INT(57),
	[47] = MSM_GPIO_TO_INT(73),
	[48] = MSM_GPIO_TO_INT(93),
	[49] = MSM_GPIO_TO_INT(94),
	[50] = MSM_GPIO_TO_INT(103),
	[51] = MSM_GPIO_TO_INT(104),
	[52] = MSM_GPIO_TO_INT(106),
	[53] = MSM_GPIO_TO_INT(115),
	[54] = MSM_GPIO_TO_INT(124),
	[55] = MSM_GPIO_TO_INT(125),
	[56] = MSM_GPIO_TO_INT(126),
	[57] = MSM_GPIO_TO_INT(127),
	[58] = MSM_GPIO_TO_INT(128),
	[59] = MSM_GPIO_TO_INT(129),
};

static uint16_t msm_mpm_bypassed_apps_irqs[] = {
	TLMM_MSM_SUMMARY_IRQ,
	RPM_SCSS_CPU0_GP_HIGH_IRQ,
	RPM_SCSS_CPU0_GP_MEDIUM_IRQ,
	RPM_SCSS_CPU0_GP_LOW_IRQ,
	RPM_SCSS_CPU0_WAKE_UP_IRQ,
	RPM_SCSS_CPU1_GP_HIGH_IRQ,
	RPM_SCSS_CPU1_GP_MEDIUM_IRQ,
	RPM_SCSS_CPU1_GP_LOW_IRQ,
	RPM_SCSS_CPU1_WAKE_UP_IRQ,
	MARM_SCSS_GP_IRQ_0,
	MARM_SCSS_GP_IRQ_1,
	MARM_SCSS_GP_IRQ_2,
	MARM_SCSS_GP_IRQ_3,
	MARM_SCSS_GP_IRQ_4,
	MARM_SCSS_GP_IRQ_5,
	MARM_SCSS_GP_IRQ_6,
	MARM_SCSS_GP_IRQ_7,
	MARM_SCSS_GP_IRQ_8,
	MARM_SCSS_GP_IRQ_9,
	LPASS_SCSS_GP_LOW_IRQ,
	LPASS_SCSS_GP_MEDIUM_IRQ,
	LPASS_SCSS_GP_HIGH_IRQ,
	SDC4_IRQ_0,
	SPS_MTI_31,
};

struct msm_mpm_device_data msm_mpm_dev_data = {
	.irqs_m2a = msm_mpm_irqs_m2a,
	.irqs_m2a_size = ARRAY_SIZE(msm_mpm_irqs_m2a),
	.bypassed_apps_irqs = msm_mpm_bypassed_apps_irqs,
	.bypassed_apps_irqs_size = ARRAY_SIZE(msm_mpm_bypassed_apps_irqs),
	.mpm_request_reg_base = MSM_RPM_BASE + 0x9d8,
	.mpm_status_reg_base = MSM_RPM_BASE + 0xdf8,
	.mpm_apps_ipc_reg = MSM_GCC_BASE + 0x008,
	.mpm_apps_ipc_val =  BIT(1),
	.mpm_ipc_irq = RPM_SCSS_CPU0_GP_MEDIUM_IRQ,

};
#endif


#ifdef CONFIG_MSM_BUS_SCALING
struct platform_device msm_bus_sys_fabric = {
	.name  = "msm_bus_fabric",
	.id    =  MSM_BUS_FAB_SYSTEM,
};
struct platform_device msm_bus_apps_fabric = {
	.name  = "msm_bus_fabric",
	.id    = MSM_BUS_FAB_APPSS,
};
struct platform_device msm_bus_mm_fabric = {
	.name  = "msm_bus_fabric",
	.id    = MSM_BUS_FAB_MMSS,
};
struct platform_device msm_bus_sys_fpb = {
	.name  = "msm_bus_fabric",
	.id    = MSM_BUS_FAB_SYSTEM_FPB,
};
struct platform_device msm_bus_cpss_fpb = {
	.name  = "msm_bus_fabric",
	.id    = MSM_BUS_FAB_CPSS_FPB,
};
#endif

#ifdef CONFIG_SND_SOC_MSM8660_APQ
struct platform_device msm_pcm = {
	.name   = "msm-pcm-dsp",
	.id     = -1,
};

struct platform_device msm_pcm_routing = {
	.name   = "msm-pcm-routing",
	.id     = -1,
};

struct platform_device msm_cpudai0 = {
	.name   = "msm-dai-q6",
	.id     = PRIMARY_I2S_RX,
};

struct platform_device msm_cpudai1 = {
	.name   = "msm-dai-q6",
	.id     = PRIMARY_I2S_TX,
};

struct platform_device msm_cpudai_hdmi_rx = {
	.name   = "msm-dai-q6",
	.id     = HDMI_RX,
};

struct platform_device msm_cpudai_bt_rx = {
	.name   = "msm-dai-q6",
	.id     = INT_BT_SCO_RX,
};

struct platform_device msm_cpudai_bt_tx = {
	.name   = "msm-dai-q6",
	.id     = INT_BT_SCO_TX,
};

struct platform_device msm_cpudai_fm_rx = {
	.name   = "msm-dai-q6",
	.id     = INT_FM_RX,
};

struct platform_device msm_cpudai_fm_tx = {
	.name   = "msm-dai-q6",
	.id     = INT_FM_TX,
};

struct platform_device msm_cpu_fe = {
	.name   = "msm-dai-fe",
	.id     = -1,
};

struct platform_device msm_stub_codec = {
	.name   = "msm-stub-codec",
	.id     = 1,
};

struct platform_device msm_voice = {
	.name   = "msm-pcm-voice",
	.id     = -1,
};

struct platform_device msm_voip = {
	.name   = "msm-voip-dsp",
	.id     = -1,
};

struct platform_device msm_lpa_pcm = {
	.name   = "msm-pcm-lpa",
	.id     = -1,
};

struct platform_device msm_pcm_hostless = {
	.name   = "msm-pcm-hostless",
	.id     = -1,
};
#endif

struct platform_device asoc_msm_pcm = {
	.name   = "msm-dsp-audio",
	.id     = 0,
};

struct platform_device asoc_msm_dai0 = {
	.name   = "msm-codec-dai",
	.id     = 0,
};

struct platform_device asoc_msm_dai1 = {
	.name   = "msm-cpu-dai",
	.id     = 0,
};

#if defined (CONFIG_MSM_8x60_VOIP)
struct platform_device asoc_msm_mvs = {
	.name   = "msm-mvs-audio",
	.id     = 0,
};

struct platform_device asoc_mvs_dai0 = {
	.name   = "mvs-codec-dai",
	.id     = 0,
};

struct platform_device asoc_mvs_dai1 = {
	.name   = "mvs-cpu-dai",
	.id     = 0,
};
#endif

struct platform_device *msm_footswitch_devices[] = {
	FS_8X60(FS_IJPEG,	"fs_ijpeg"),
	/*FS_8X60(FS_MDP,	"fs_mdp"),*//*re-enable when find out why mdp_p can't turn off*/
	FS_8X60(FS_ROT,	"fs_rot"),
	FS_8X60(FS_VED,	"fs_ved"),
	FS_8X60(FS_VFE,	"fs_vfe"),
	FS_8X60(FS_VPE,	"fs_vpe"),
	FS_8X60(FS_GFX3D,	"fs_gfx3d"),
	FS_8X60(FS_GFX2D0,	"fs_gfx2d0"),
	FS_8X60(FS_GFX2D1,	"fs_gfx2d1"),
};

unsigned msm_num_footswitch_devices = ARRAY_SIZE(msm_footswitch_devices);

#ifdef CONFIG_MSM_RPM
struct msm_rpm_map_data rpm_map_data[] __initdata = {
	MSM_RPM_MAP(TRIGGER_TIMED_TO, TRIGGER_TIMED, 1),
	MSM_RPM_MAP(TRIGGER_TIMED_SCLK_COUNT, TRIGGER_TIMED, 1),
	MSM_RPM_MAP(TRIGGER_SET_FROM, TRIGGER_SET, 1),
	MSM_RPM_MAP(TRIGGER_SET_TO, TRIGGER_SET, 1),
	MSM_RPM_MAP(TRIGGER_SET_TRIGGER, TRIGGER_SET, 1),
	MSM_RPM_MAP(TRIGGER_CLEAR_FROM, TRIGGER_CLEAR, 1),
	MSM_RPM_MAP(TRIGGER_CLEAR_TO, TRIGGER_CLEAR, 1),
	MSM_RPM_MAP(TRIGGER_CLEAR_TRIGGER, TRIGGER_CLEAR, 1),

	MSM_RPM_MAP(CXO_CLK, CXO_CLK, 1),
	MSM_RPM_MAP(PXO_CLK, PXO_CLK, 1),
	MSM_RPM_MAP(PLL_4, PLL_4, 1),
	MSM_RPM_MAP(APPS_FABRIC_CLK, APPS_FABRIC_CLK, 1),
	MSM_RPM_MAP(SYSTEM_FABRIC_CLK, SYSTEM_FABRIC_CLK, 1),
	MSM_RPM_MAP(MM_FABRIC_CLK, MM_FABRIC_CLK, 1),
	MSM_RPM_MAP(DAYTONA_FABRIC_CLK, DAYTONA_FABRIC_CLK, 1),
	MSM_RPM_MAP(SFPB_CLK, SFPB_CLK, 1),
	MSM_RPM_MAP(CFPB_CLK, CFPB_CLK, 1),
	MSM_RPM_MAP(MMFPB_CLK, MMFPB_CLK, 1),
	MSM_RPM_MAP(SMI_CLK, SMI_CLK, 1),
	MSM_RPM_MAP(EBI1_CLK, EBI1_CLK, 1),

	MSM_RPM_MAP(APPS_L2_CACHE_CTL, APPS_L2_CACHE_CTL, 1),

	MSM_RPM_MAP(APPS_FABRIC_HALT_0, APPS_FABRIC_HALT, 2),
	MSM_RPM_MAP(APPS_FABRIC_CLOCK_MODE_0, APPS_FABRIC_CLOCK_MODE, 3),
	MSM_RPM_MAP(APPS_FABRIC_ARB_0, APPS_FABRIC_ARB, 6),

	MSM_RPM_MAP(SYSTEM_FABRIC_HALT_0, SYSTEM_FABRIC_HALT, 2),
	MSM_RPM_MAP(SYSTEM_FABRIC_CLOCK_MODE_0, SYSTEM_FABRIC_CLOCK_MODE, 3),
	MSM_RPM_MAP(SYSTEM_FABRIC_ARB_0, SYSTEM_FABRIC_ARB, 22),

	MSM_RPM_MAP(MM_FABRIC_HALT_0, MM_FABRIC_HALT, 2),
	MSM_RPM_MAP(MM_FABRIC_CLOCK_MODE_0, MM_FABRIC_CLOCK_MODE, 3),
	MSM_RPM_MAP(MM_FABRIC_ARB_0, MM_FABRIC_ARB, 23),

	MSM_RPM_MAP(SMPS0B_0, SMPS0B, 2),
	MSM_RPM_MAP(SMPS1B_0, SMPS1B, 2),
	MSM_RPM_MAP(SMPS2B_0, SMPS2B, 2),
	MSM_RPM_MAP(SMPS3B_0, SMPS3B, 2),
	MSM_RPM_MAP(SMPS4B_0, SMPS4B, 2),
	MSM_RPM_MAP(LDO0B_0, LDO0B, 2),
	MSM_RPM_MAP(LDO1B_0, LDO1B, 2),
	MSM_RPM_MAP(LDO2B_0, LDO2B, 2),
	MSM_RPM_MAP(LDO3B_0, LDO3B, 2),
	MSM_RPM_MAP(LDO4B_0, LDO4B, 2),
	MSM_RPM_MAP(LDO5B_0, LDO5B, 2),
	MSM_RPM_MAP(LDO6B_0, LDO6B, 2),
	MSM_RPM_MAP(LVS0B, LVS0B, 1),
	MSM_RPM_MAP(LVS1B, LVS1B, 1),
	MSM_RPM_MAP(LVS2B, LVS2B, 1),
	MSM_RPM_MAP(LVS3B, LVS3B, 1),
	MSM_RPM_MAP(MVS, MVS, 1),

	MSM_RPM_MAP(SMPS0_0, SMPS0, 2),
	MSM_RPM_MAP(SMPS1_0, SMPS1, 2),
	MSM_RPM_MAP(SMPS2_0, SMPS2, 2),
	MSM_RPM_MAP(SMPS3_0, SMPS3, 2),
	MSM_RPM_MAP(SMPS4_0, SMPS4, 2),
	MSM_RPM_MAP(LDO0_0, LDO0, 2),
	MSM_RPM_MAP(LDO1_0, LDO1, 2),
	MSM_RPM_MAP(LDO2_0, LDO2, 2),
	MSM_RPM_MAP(LDO3_0, LDO3, 2),
	MSM_RPM_MAP(LDO4_0, LDO4, 2),
	MSM_RPM_MAP(LDO5_0, LDO5, 2),
	MSM_RPM_MAP(LDO6_0, LDO6, 2),
	MSM_RPM_MAP(LDO7_0, LDO7, 2),
	MSM_RPM_MAP(LDO8_0, LDO8, 2),
	MSM_RPM_MAP(LDO9_0, LDO9, 2),
	MSM_RPM_MAP(LDO10_0, LDO10, 2),
	MSM_RPM_MAP(LDO11_0, LDO11, 2),
	MSM_RPM_MAP(LDO12_0, LDO12, 2),
	MSM_RPM_MAP(LDO13_0, LDO13, 2),
	MSM_RPM_MAP(LDO14_0, LDO14, 2),
	MSM_RPM_MAP(LDO15_0, LDO15, 2),
	MSM_RPM_MAP(LDO16_0, LDO16, 2),
	MSM_RPM_MAP(LDO17_0, LDO17, 2),
	MSM_RPM_MAP(LDO18_0, LDO18, 2),
	MSM_RPM_MAP(LDO19_0, LDO19, 2),
	MSM_RPM_MAP(LDO20_0, LDO20, 2),
	MSM_RPM_MAP(LDO21_0, LDO21, 2),
	MSM_RPM_MAP(LDO22_0, LDO22, 2),
	MSM_RPM_MAP(LDO23_0, LDO23, 2),
	MSM_RPM_MAP(LDO24_0, LDO24, 2),
	MSM_RPM_MAP(LDO25_0, LDO25, 2),
	MSM_RPM_MAP(LVS0, LVS0, 1),
	MSM_RPM_MAP(LVS1, LVS1, 1),
	MSM_RPM_MAP(NCP_0, NCP, 2),

	MSM_RPM_MAP(CXO_BUFFERS, CXO_BUFFERS, 1),
};
unsigned int rpm_map_data_size = ARRAY_SIZE(rpm_map_data);

struct platform_device msm_rpm_device = {
	.name = "msm_rpm",
	.id = -1,
};
#endif

struct flash_platform_data msm_nand_data = {
	.parts          = NULL,
	.nr_parts       = 0,
};

#ifdef CONFIG_SENSORS_MSM_ADC
static void pmic8058_xoadc_mpp_config(void)
{
	/* Do not set mpp to amux in here.
	   Set the mpp to amux mapping when needed and then reset */
}

static struct regulator *vreg_ldo18_adc;

static int pmic8058_xoadc_vreg_config(int on)
{
	int rc;

	if (on) {
		rc = regulator_enable(vreg_ldo18_adc);
		if (rc)
			pr_err("%s: Enable of regulator ldo18_adc "
						"failed\n", __func__);
	} else {
		rc = regulator_disable(vreg_ldo18_adc);
		if (rc)
			pr_err("%s: Disable of regulator ldo18_adc "
						"failed\n", __func__);
	}

	return rc;
}

static int pmic8058_xoadc_vreg_setup(void)
{
	int rc;

	vreg_ldo18_adc = regulator_get(NULL, "8058_l18");
	if (IS_ERR(vreg_ldo18_adc)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_ldo18_adc));
		rc = PTR_ERR(vreg_ldo18_adc);
		goto fail;
	}

	rc = regulator_set_voltage(vreg_ldo18_adc, 2200000, 2200000);
	if (rc) {
		pr_err("%s: unable to set ldo18 voltage to 2.2V\n", __func__);
		goto fail;
	}

	rc = regulator_enable(vreg_ldo18_adc);
	if (rc) {
		pr_err("%s: Enable of regulator ldo18_adc "
					"failed\n", __func__);
		goto fail;
	}
	return rc;
fail:
	regulator_put(vreg_ldo18_adc);
	return rc;
}

static void pmic8058_xoadc_vreg_shutdown(void)
{
	regulator_put(vreg_ldo18_adc);
}

/* usec. For this ADC,
 * this time represents clk rate @ txco w/ 1024 decimation ratio.
 * Each channel has different configuration, thus at the time of starting
 * the conversion, xoadc will return actual conversion time
 */
static struct adc_properties pm8058_xoadc_data = {
	.adc_reference          = 2200, /* milli-voltage for this adc */
	.bitresolution         = 15,
	.bipolar                = 0,
	.conversiontime         = 54,
};

struct xoadc_platform_data pm8058_xoadc_pdata = {
	.xoadc_prop = &pm8058_xoadc_data,
	.xoadc_mpp_config = pmic8058_xoadc_mpp_config,
	.xoadc_vreg_set = pmic8058_xoadc_vreg_config,
	.xoadc_num = XOADC_PMIC_0,
	.xoadc_vreg_setup = pmic8058_xoadc_vreg_setup,
	.xoadc_vreg_shutdown = pmic8058_xoadc_vreg_shutdown,
};
#endif

static int msm8660_LPM_latency = 1000; /* >100 usec for WFI */

struct platform_device msm8660_cpu_idle_device = {
	.name   = "msm_cpu_idle",
	.id     = -1,
	.dev = {
		.platform_data = &msm8660_LPM_latency,
	},
};

static struct msm_dcvs_freq_entry msm8660_freq[] = {
	{ 384000, 166981,  345600},
	{ 702000, 213049,  632502},
	{1026000, 285712,  925613},
	{1242000, 383945, 1176550},
	{1458000, 419729, 1465478},
	{1512000, 434116, 1546674},

};

static struct msm_dcvs_core_info msm8660_core_info = {
	.freq_tbl = &msm8660_freq[0],
	.core_param = {
		.max_time_us = 100000,
		.num_freq = ARRAY_SIZE(msm8660_freq),
	},
	.algo_param = {
		.slack_time_us = 58000,
		.scale_slack_time = 0,
		.scale_slack_time_pct = 0,
		.disable_pc_threshold = 1458000,
		.em_window_size = 100000,
		.em_max_util_pct = 97,
		.ss_window_size = 1000000,
		.ss_util_pct = 95,
		.ss_iobusy_conv = 100,
	},
};

struct platform_device msm8660_msm_gov_device = {
	.name = "msm_dcvs_gov",
	.id = -1,
	.dev = {
		.platform_data = &msm8660_core_info,
	},
};

