/*
 *  NXP LPC32xx emulation
 *
 *  Copyright (c) 2012  Alex Vedenov <vedenov86743+qemu at gmail>
 *
 *  License: GPL v2, like kernel
 *
 */


#ifndef LPC32xx_H_
#define LPC32xx_H_

#include "qemu-common.h"
#include "memory.h"
#include "lpc32xx-platform.h"
#include "lpc32xx-irqs.h"


#define LPC32xx_BASE_BOOT_ADDR           LPC32XX_EMC_DYCS0_BASE


#if 000
typedef struct LPC32xxIrq {
    qemu_irq int_combiner_irq[LPC32xx_MAX_INT_COMBINER_IN_IRQ];
    qemu_irq ext_combiner_irq[LPC32xx_MAX_EXT_COMBINER_IN_IRQ];
    qemu_irq int_gic_irq[LPC32xx_INT_GIC_NIRQ];
    qemu_irq ext_gic_irq[LPC32xx_EXT_GIC_NIRQ];
    qemu_irq board_irqs[LPC32xx_MAX_INT_COMBINER_IN_IRQ];
} LPC32xxIrq;
#endif

typedef struct LPC32xxState {
    ARMCPU *cpu;
    qemu_irq *irq_table;

    MemoryRegion chipid_mem;
    MemoryRegion iram_mem;
    MemoryRegion iram_alias_mem;
    MemoryRegion irom_mem;
    MemoryRegion irom_alias_mem;
    MemoryRegion dram0_mem;
    MemoryRegion dram1_mem;

    DeviceState *timer[6];

    DeviceState *uart[8];

#if 000
    i2c_bus *i2c_if[LPC32xx_I2C_NUMBER];
#endif
} LPC32xxState;

LPC32xxState *lpc32xx_init(MemoryRegion *system_mem,
        unsigned long ram_size);

#if 000
/* Initialize lpc32xx IRQ subsystem stub */
qemu_irq *lpc32xx_init_irq(LPC32xxIrq *env);

/* Initialize board IRQs.
 * These IRQs contain splitted Int/External Combiner and External Gic IRQs */
void lpc32xx_init_board_irqs(LPC32xxIrq *s);

/* Get IRQ number from lpc32xx IRQ subsystem stub.
 * To identify IRQ source use internal combiner group and bit number
 *  grp - group number
 *  bit - bit number inside group */
uint32_t lpc32xx_get_irq(uint32_t grp, uint32_t bit);

/*
 * Get Combiner input GPIO into irqs structure
 */
void lpc32xx_combiner_get_gpioin(LPC32xxIrq *irqs, DeviceState *dev,
        int ext);
#endif

/*
 * lpc32xx UART
 */
DeviceState *lpc32xx_uart_create(target_phys_addr_t addr,
                                    int fifo_size,
                                    int channel,
                                    CharDriverState *chr,
                                    qemu_irq irq);

#endif /* LPC32xx_H_ */
