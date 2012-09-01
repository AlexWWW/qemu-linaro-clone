/*
 *  NXP LPC32xx emulation
 *
 *  Copyright (c) 2012  Alex Vedenov <vedenov86743+qemu at gmail>
 *
 *  License: GPL v2, like kernel
 *
 */

#include "boards.h"
#include "sysemu.h"
#include "sysbus.h"
#include "arm-misc.h"
#include "loader.h"
#include "lpc32xx.h"
#if 000
#include "omap.h"          /* UARTs */
#endif

LPC32xxState *lpc32xx_init(MemoryRegion *system_mem,
        unsigned long ram_size)
{
#if 000
    const target_phys_addr_t *map = daughterboard->motherboard_map;
#endif
    qemu_irq *cpu_pic;
    int n;
    LPC32xxState *s = g_new(LPC32xxState, 1);
    MemoryRegion *iram = &s->iram_mem;
    MemoryRegion *irom = &s->irom_mem;
    unsigned long iram_size;
    unsigned long irom_size;
    MemoryRegion *iram_at0 = &s->iram_alias_mem;
    MemoryRegion *irom_at0 = &s->irom_alias_mem;
    MemoryRegion *dram0 = &s->dram0_mem;
    MemoryRegion *dram1 = &s->dram1_mem;
    unsigned long dram0_size;
    unsigned long dram1_size;
    qemu_irq mic [32];
    qemu_irq sic1[32];
    qemu_irq sic2[32];
    DeviceState *dev;
    SysBusDevice *busdev;


    s->cpu = cpu_arm_init("arm926");
    if (!s->cpu) {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(1);
    }

    cpu_pic = arm_pic_init_cpu(s->cpu);

    /* IRAM - 128-256kB populated */

    iram_size = 256 * 1024;
    memory_region_init_ram(iram, "lpc32xx-iram", iram_size);
    vmstate_register_ram_global(iram);
    memory_region_add_subregion(system_mem, LPC32XX_IRAM_BASE, iram);

    /* IROM - 16kB populated */

    irom_size = 16 * 1024;
    memory_region_init_ram(irom, "lpc32xx-irom", irom_size);
    vmstate_register_ram_global(irom);
    memory_region_set_readonly(irom, true);
    memory_region_add_subregion(system_mem, LPC32XX_IROM_BASE, irom);

    /* IROM/IRAM mapped at 0x00000000 */

    memory_region_init_alias(irom_at0, "lpc32xx-alias-irom",
                             irom, 0, irom_size);
    memory_region_set_readonly(irom_at0, true);
if(0)
    memory_region_add_subregion(system_mem, 0x00000000,
                                irom_at0);

    memory_region_init_alias(iram_at0, "lpc32xx-alias-iram",
                             iram, 0, iram_size);
    memory_region_add_subregion(system_mem, 0x00000000,
                                iram_at0);

    /* off-chip DRAM 0x80000000, 0xa0000000 */

    dram0_size = ram_size;
    dram1_size = 0;
    if(0x20000000 < dram0_size) {
        dram0_size = 0x20000000;
        dram1_size = ram_size - 0x20000000;
    }
    memory_region_init_ram(dram0, "lpc32xx-dram", dram0_size);
    vmstate_register_ram_global(dram0);
    memory_region_add_subregion(system_mem, LPC32XX_EMC_DYCS0_BASE, dram0);

    if(dram1_size) {
        dram1 = dram1;
    }

    /* off-chip SRAM 0xe0000000..0xe3000000 */

    /* SCB SERIAL_ID0..3 0x40004130..c */

    /* CLK/PM */

    dev = sysbus_create_simple("lpc32xx-clkpm", LPC32XX_CLK_PM_BASE, NULL);
    dev->id = "clkpm";

    /* MIC, SIC1, SIC2 */

    dev = sysbus_create_varargs("lpc32xx-mic", LPC32XX_MIC_BASE,
                                cpu_pic[ARM_PIC_CPU_IRQ], 
                                cpu_pic[ARM_PIC_CPU_FIQ], 
                                NULL);
    dev->id = "mic ";
    for (n = 0; n < 32; n++) {
        mic [n] = qdev_get_gpio_in(dev, n);
    }

    /* sic irq/fiq lines are active low hence invert connection */

    dev = sysbus_create_varargs("lpc32xx-sic1", LPC32XX_SIC1_BASE,
                                qemu_irq_invert(mic[IRQ_LPC32XX_SUB1IRQ]),
                                qemu_irq_invert(mic[IRQ_LPC32XX_SUB1FIQ]),
                                NULL);
    dev->id = "sic1";
    for (n = 0; n < 32; n++) {
        sic1[n] = qdev_get_gpio_in(dev, n);
    }

    dev = sysbus_create_varargs("lpc32xx-sic2", LPC32XX_SIC2_BASE,
                                qemu_irq_invert(mic[IRQ_LPC32XX_SUB2IRQ]),
                                qemu_irq_invert(mic[IRQ_LPC32XX_SUB2FIQ]),
                                NULL);
    dev->id = "sic2";
    for (n = 0; n < 32; n++) {
        sic2[n] = qdev_get_gpio_in(dev, n);
    }

    *sic1 = *sic1;
    *sic2 = *sic2;

    /* standard timers */

    s->timer[0] =
    dev = sysbus_create_varargs("lpc32xx-timer", LPC32XX_TIMER0_BASE,
                                qemu_irq_invert(mic[IRQ_LPC32XX_TIMER0]),
                                NULL);
    dev->id = "tist0";

    s->timer[1] =
    dev = sysbus_create_varargs("lpc32xx-timer", LPC32XX_TIMER1_BASE,
                                qemu_irq_invert(mic[IRQ_LPC32XX_TIMER1]),
                                NULL);
    dev->id = "tist1";




    /* UARTs */

    s->uart[5] = 
    dev = qdev_create(NULL, "lpc32xx-uart");
    dev->id = "uart5";
    qdev_prop_set_uint32(dev, "mmio_size", 0x400);
    qdev_prop_set_uint32(dev, "baudrate",
                         750000);
#if 000
                         omap_clk_getrate(omap_findclk(s, "uart5_ck")) / 16);
#endif
    qdev_prop_set_chr(dev, "chardev", serial_hds[0]);
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    sysbus_connect_irq(busdev, 0, mic[IRQ_LPC32XX_UART_IIR5]);
    /* [tbd] U5_RX -> sic2[IRQ_LPC32XX_U5_RX] */
/*
    sysbus_connect_irq(busdev, 1, irq_input[DMA_UART5_TX]);
    sysbus_connect_irq(busdev, 2, irq_input[DMA_UART5_RX]);
 */
    sysbus_mmio_map(busdev, 0, LPC32XX_UART5_BASE);


    return s;
}

