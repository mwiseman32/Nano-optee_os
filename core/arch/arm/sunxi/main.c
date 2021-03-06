// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2014, Allwinner Technology Co., Ltd.
 * Copyright (c) 2018, Linaro Limited
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <console.h>
#include <io.h>
#include <stdint.h>
#include <drivers/gic.h>
#include <drivers/serial8250_uart.h>
#include <drivers/tzc380.h>
#include <kernel/generic_boot.h>
#include <kernel/misc.h>
#include <kernel/panic.h>
#include <kernel/pm_stubs.h>
#include <kernel/tz_ssvce_def.h>
#include <mm/core_mmu.h>
#include <mm/core_memprot.h>
#include <mm/tee_pager.h>
#include <platform_config.h>
#include <sm/tee_mon.h>
#include <sm/optee_smc.h>
#include <tee/entry_fast.h>
#include <tee/entry_std.h>


#ifdef GIC_BASE
register_phys_mem(MEM_AREA_IO_SEC, GIC_BASE, CORE_MMU_DEVICE_SIZE);
#endif

#ifdef CONSOLE_UART_BASE
register_phys_mem(MEM_AREA_IO_NSEC,
		  CONSOLE_UART_BASE, SUNXI_UART_REG_SIZE);
#endif

#ifdef SUNXI_TZPC_BASE
register_phys_mem(MEM_AREA_IO_SEC, SUNXI_TZPC_BASE, SUNXI_TZPC_REG_SIZE);
#define REG_TZPC_SMTA_DECPORT0_STA_REG      (0x0004)
#define REG_TZPC_SMTA_DECPORT0_SET_REG      (0x0008)
#define REG_TZPC_SMTA_DECPORT0_CLR_REG      (0x000C)
#define REG_TZPC_SMTA_DECPORT1_STA_REG      (0x0010)
#define REG_TZPC_SMTA_DECPORT1_SET_REG      (0x0014)
#define REG_TZPC_SMTA_DECPORT1_CLR_REG      (0x0018)
#define REG_TZPC_SMTA_DECPORT2_STA_REG      (0x001c)
#define REG_TZPC_SMTA_DECPORT2_SET_REG      (0x0020)
#define REG_TZPC_SMTA_DECPORT2_CLR_REG      (0x0024)
#endif

#ifdef SUNXI_CPUCFG_BASE
register_phys_mem(MEM_AREA_IO_SEC, SUNXI_CPUCFG_BASE, SUNXI_CPUCFG_REG_SIZE);
#endif

#ifdef SUNXI_PRCM_BASE
register_phys_mem(MEM_AREA_IO_SEC, SUNXI_PRCM_BASE, SUNXI_PRCM_REG_SIZE);
#endif

#ifdef CFG_TZC380
vaddr_t smc_base(void);
register_phys_mem(MEM_AREA_IO_SEC, SUNXI_SMC_BASE, TZC400_REG_SIZE);
#define SMC_MASTER_BYPASS 0x18
#define SMC_MASTER_BYPASS_EN_MASK 0x1
#endif
#ifdef GIC_BASE
static struct gic_data gic_data;
#endif
#ifdef SUNXI_TZPC_BASE
static void tzpc_init(void);
#endif

static struct gic_data gic_data;
static void tzpc_init(void);

static void main_fiq(void)
{
	panic();
}

static const struct thread_handlers handlers = {
	.std_smc = tee_entry_std,
	.fast_smc = tee_entry_fast,
	.nintr = main_fiq,
#if defined(CFG_WITH_ARM_TRUSTED_FW)
	.cpu_on = cpu_on_handler,
	.cpu_off = pm_do_nothing,
	.cpu_suspend = pm_do_nothing,
	.cpu_resume = pm_do_nothing,
	.system_off = pm_do_nothing,
	.system_reset = pm_do_nothing,
#else
	.cpu_on = pm_panic,
	.cpu_off = pm_panic,
	.cpu_suspend = pm_panic,
	.cpu_resume = pm_panic,
	.system_off = pm_panic,
	.system_reset = pm_panic,
#endif
};

static struct serial8250_uart_data console_data;

const struct thread_handlers *generic_boot_get_handlers(void)
{
	return &handlers;
}

void console_init(void)
{
	serial8250_uart_init(&console_data,
			     CONSOLE_UART_BASE,
			     CONSOLE_UART_CLK_IN_HZ,
			     CONSOLE_BAUDRATE);
	register_serial_console(&console_data.chip);
}

#ifdef SUNXI_TZPC_BASE
static void tzpc_init(void)
{
	vaddr_t tzpc;

	tzpc = (vaddr_t)phys_to_virt(SUNXI_TZPC_BASE, MEM_AREA_IO_SEC);

	DMSG("SMTA_DECPORT0=%x", read32(tzpc + REG_TZPC_SMTA_DECPORT0_STA_REG));
	DMSG("SMTA_DECPORT1=%x", read32(tzpc + REG_TZPC_SMTA_DECPORT1_STA_REG));
	DMSG("SMTA_DECPORT2=%x", read32(tzpc + REG_TZPC_SMTA_DECPORT2_STA_REG));

	/* Allow all peripherals for normal world */
	write32(0xbe, tzpc + REG_TZPC_SMTA_DECPORT0_SET_REG);
	write32(0xff, tzpc + REG_TZPC_SMTA_DECPORT1_SET_REG);
	write32(0x7f, tzpc + REG_TZPC_SMTA_DECPORT2_SET_REG);

	DMSG("SMTA_DECPORT0=%x", read32(tzpc + REG_TZPC_SMTA_DECPORT0_STA_REG));
	DMSG("SMTA_DECPORT1=%x", read32(tzpc + REG_TZPC_SMTA_DECPORT1_STA_REG));
	DMSG("SMTA_DECPORT2=%x", read32(tzpc + REG_TZPC_SMTA_DECPORT2_STA_REG));
}
#else
static inline void tzpc_init(void)
{
}
#endif /* SUNXI_TZPC_BASE */
#ifndef CFG_WITH_ARM_TRUSTED_FW

void main_init_gic(void)
{
	vaddr_t gicc_base;
	vaddr_t gicd_base;

	gicc_base = core_mmu_get_va(GIC_BASE + GICC_OFFSET, MEM_AREA_IO_SEC);
	gicd_base = core_mmu_get_va(GIC_BASE + GICD_OFFSET, MEM_AREA_IO_SEC);

	if (!gicc_base || !gicd_base)
		panic();

	/* Initialize GIC */
	gic_init(&gic_data, gicc_base, gicd_base);
	itr_init(&gic_data.chip);
}


void main_secondary_init_gic(void)
{
	gic_cpu_init(&gic_data);
}
#endif 

#ifdef ARM32
void plat_cpu_reset_late(void)
{
	assert(!cpu_mmu_enabled());

	if (get_core_pos())
		return;

	tzpc_init();
}

#endif
/*
 * Allwinner's A64 has TZC380 like controller called SMC that can
 * be programmed to protect parts of DRAM from non-secure world.
 */
#ifdef CFG_TZC380
vaddr_t smc_base(void)
{
	return (vaddr_t)phys_to_virt(SUNXI_SMC_BASE, MEM_AREA_IO_SEC);
}
static TEE_Result smc_init(void)
{
	uint32_t val = 0;
	vaddr_t base = smc_base();
	if (!base) {
		EMSG("smc not mapped");
		panic();
	}
	tzc_init(base);
	tzc_configure_region(0, 0x0, TZC_ATTR_REGION_SIZE(TZC_REGION_SIZE_1G) |
			     TZC_ATTR_REGION_EN_MASK | TZC_ATTR_SP_ALL);
	tzc_configure_region(1, 0x0, TZC_ATTR_REGION_SIZE(TZC_REGION_SIZE_32M) |
			     TZC_ATTR_REGION_EN_MASK | TZC_ATTR_SP_S_RW);
	/* SoC specific bits */
	val = read32(base + SMC_MASTER_BYPASS);
	val = val & ~(SMC_MASTER_BYPASS_EN_MASK);
	write32(val, base + SMC_MASTER_BYPASS);
	return TEE_SUCCESS;
}
driver_init(smc_init);
#endif /* CFG_TZC380 */
