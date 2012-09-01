/*
 *  LPC32xx clock and power control emulation
 *
 *  Copyright (c) 2012  Alex Vedenov <vedenov86743+qemu at gmail>
 *
 *  License: GPL v2, like kernel
 *
 */

#include "hw.h"
#include "qemu-timer.h"
#include "ptimer.h"
#include "sysbus.h"
#include "lpc32xx.h"


#define V_CLKPM_RESET   01
#define V_CLKPM_RD      01
#define V_CLKPM_WR      01
#define V_CLKPM_INIT    01


typedef struct lpc32xx_clkpm_state {
    SysBusDevice busdev;
    MemoryRegion iomem;

    uint32_t debug;
    uint32_t bootmap;
    uint32_t p01_er;
    uint32_t usbdiv;
    uint32_t int_er;
    uint32_t int_rs;
    uint32_t int_sr;
    uint32_t int_ap;
    uint32_t pin_er;
    uint32_t pin_rs;
    uint32_t pin_sr;
    uint32_t pin_ap;
    uint32_t hckdiv;
    uint32_t pwr;
    uint32_t pll397;
    uint32_t osc;
    uint32_t sysclk;
    uint32_t lcdclk;
    uint32_t hckpll;
    uint32_t adcclk_1;
    uint32_t usbctl;

    uint32_t sspclk;
    uint32_t i2sclk;

    uint32_t sdcard;
    uint32_t macclk;
    uint32_t test_clk;
    uint32_t swint;
    uint32_t i2cclk;
    uint32_t kbdclk;
    uint32_t adcclk;
    uint32_t pwmclk;
    uint32_t tmrclk;
    uint32_t tmrclk_1;
    uint32_t spiclk;
    uint32_t flashclk;
    uint32_t uart3;
    uint32_t uart4;
    uint32_t uart5;
    uint32_t uart6;
    uint32_t irdaclk;
    uint32_t uartclk;
    uint32_t dmaclk;
    uint32_t autoclock;
/*
    uint32_t devid [4];
 */

    qemu_irq irq;

} lpc32xx_clkpm_state;

static const VMStateDescription vmstate_lpc32xx_clkpm = {
    .name = "lpc32xx-clkpm",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
#if 000
        VMSTATE_UINT32(ir, lpc32xx_timer_state),
        VMSTATE_UINT32(tcr, lpc32xx_timer_state),
        VMSTATE_UINT32(tc, lpc32xx_timer_state),
        VMSTATE_UINT32(pr, lpc32xx_timer_state),
        VMSTATE_UINT32(pc, lpc32xx_timer_state),
        VMSTATE_UINT32(mcr, lpc32xx_timer_state),
        VMSTATE_UINT32(ccr, lpc32xx_timer_state),
        VMSTATE_UINT32(emr, lpc32xx_timer_state),
        VMSTATE_UINT32(ctcr, lpc32xx_timer_state),
#endif
        VMSTATE_END_OF_LIST()
    }
};


static void lpc32xx_clkpm_update(lpc32xx_clkpm_state *s)
{
#if 000
    if (s->int_level && (s->cr & CR_OCIEN)) {
        qemu_irq_raise(s->irq);
    } else {
        qemu_irq_lower(s->irq);
    }
#endif
    s = s;
}

static void lpc32xx_clkpm_reset(DeviceState *dev)
{
    lpc32xx_clkpm_state *s = container_of(dev, lpc32xx_clkpm_state, busdev.qdev);

if(V_CLKPM_RESET)
    fprintf(stderr, "%s.z\n", s->busdev.qdev.id);

    s->debug = 0;
    s->bootmap = 0;
    s->p01_er = 0;
    s->usbdiv = 0x0000000c;
    s->int_er = 0;
    s->int_rs = 0;
    s->int_sr = 0;
    s->int_ap = 0;
    s->pin_er = 0;
    s->pin_rs = 0;
    s->pin_sr = 0;
    s->pin_ap = 0;
    s->hckdiv = 0;
    s->pwr = 0x00000012;
    s->pll397 = 0;
    s->osc = 0x00000100;
    s->sysclk = 0x00000b48;
    s->lcdclk = 0;
    s->hckpll = 0;
    s->adcclk_1 = 0;
    s->usbctl = 0x00080000;

    s->sspclk = 0;
    s->i2sclk = 0;

    s->sdcard = 0;
    s->macclk = 0;
    s->test_clk = 0;
    s->swint = 0;
    s->i2cclk = 0;
    s->kbdclk = 0;
    s->adcclk = 0;
    s->pwmclk = 0;
    s->tmrclk = 0;
    s->tmrclk_1 = 0;
    s->spiclk = 0;
    s->flashclk = 0x00000003;
    s->uart3 = 0;
    s->uart4 = 0;
    s->uart5 = 0;
    s->uart6 = 0;
    s->irdaclk = 0;
    s->uartclk = 0x0000000f;
    s->dmaclk = 0x00000001;
    s->autoclock = 0;

    /* tune-up */

    /* 
     * PLL feedback divider (m)
     * required to pass fcco check in arch/arm/mach-lpc32xx/common.c
     */
    s->hckpll |= (12 - 1) << 1;

}


static uint64_t lpc32xx_clkpm_read__proper(void *opaque, target_phys_addr_t offset,
                                            unsigned size)
{
    lpc32xx_clkpm_state *s = (lpc32xx_clkpm_state *)opaque;

    switch (offset >> 2) {
    case 0x00>>2:
      return s->debug;
    case 0x14>>2:
      return s->bootmap;
    case 0x18>>2:
      return s->p01_er;
    case 0x1c>>2:
      return s->usbdiv;
    case 0x20>>2:
      return s->int_er;
    case 0x24>>2:
      return s->int_rs;
    case 0x28>>2:
      return s->int_sr;
    case 0x2c>>2:
      return s->int_ap;
    case 0x30>>2:
      return s->pin_er;
    case 0x34>>2:
      return s->pin_rs;
    case 0x38>>2:
      return s->pin_sr;
    case 0x3c>>2:
      return s->pin_ap;
    case 0x40>>2:
      return s->hckdiv;
    case 0x44>>2:
      return s->pwr;
    case 0x48>>2:
      return s->pll397;
    case 0x4c>>2:
      return s->osc;
    case 0x50>>2:
      return s->sysclk;
    case 0x54>>2:
      return s->lcdclk;
    case 0x58>>2:
      return s->hckpll;
    case 0x60>>2:
      return s->adcclk_1;
    case 0x64>>2:
      return s->usbctl;

    case 0x78>>2:
      return s->sspclk;
    case 0x7c>>2:
      return s->i2sclk;

    case 0x80>>2:
      return s->sdcard;
    case 0x90>>2:
      return s->macclk;
    case 0xa4>>2:
      return s->test_clk;
    case 0xa8>>2:
      return s->swint;
    case 0xac>>2:
      return s->i2cclk;
    case 0xb0>>2:
      return s->kbdclk;
    case 0xb4>>2:
      return s->adcclk;
    case 0xb8>>2:
      return s->pwmclk;
    case 0xbc>>2:
      return s->tmrclk;
    case 0xc0>>2:
      return s->tmrclk_1;
    case 0xc4>>2:
      return s->spiclk;
    case 0xc8>>2:
      return s->flashclk;
    case 0xd0>>2:
      return s->uart3;
    case 0xd4>>2:
      return s->uart4;
    case 0xd8>>2:
      return s->uart5;
    case 0xdc>>2:
      return s->uart6;
    case 0xe0>>2:
      return s->irdaclk;
    case 0xe4>>2:
      return s->uartclk;
    case 0xe8>>2:
      return s->dmaclk;
    case 0xec>>2:
      return s->autoclock;
    }

    fprintf(stderr, "lpc32xx_clkpm_read: Bad offset 0x%02x\n",
            (int)offset);
    return 0;
}

static uint64_t lpc32xx_clkpm_read(void *opaque, target_phys_addr_t offset,
                                    unsigned size)
{
    lpc32xx_clkpm_state *s = (lpc32xx_clkpm_state *) opaque;
    uint64_t value = lpc32xx_clkpm_read__proper(opaque, offset, size);

if(V_CLKPM_RD)
    fprintf(stderr, "%s.r +%02lx[%u] %08lx\n",
            s->busdev.qdev.id,
            (long)offset, size, (long)value);

    return value;
}

static void lpc32xx_clkpm_write(void *opaque, target_phys_addr_t offset,
                                 uint64_t value, unsigned size)
{
    lpc32xx_clkpm_state *s = (lpc32xx_clkpm_state *)opaque;

if(V_CLKPM_WR)
    fprintf(stderr, "%s.w +%02lx[%u] %08lx%s\n",
            s->busdev.qdev.id,
            (long)offset, size, (long)value,
            offset==0x00? " <= ": "");

    switch (offset >> 2) {

    case 0x00>>2:
      s->debug = value;
      break;
    case 0x14>>2:
      s->bootmap = value;
      break;
    case 0x18>>2:
      s->p01_er = value;
      break;
    case 0x1c>>2:
      s->usbdiv = value;
      break;
    case 0x20>>2:
      s->int_er = value;
      break;
    case 0x24>>2:
      s->int_rs = value;
      break;
    case 0x28>>2:
      s->int_sr = value;
      break;
    case 0x2c>>2:
      s->int_ap = value;
      break;
    case 0x30>>2:
      s->pin_er = value;
      break;
    case 0x34>>2:
      s->pin_rs = value;
      break;
    case 0x38>>2:
      s->pin_sr = value;
      break;
    case 0x3c>>2:
      s->pin_ap = value;
      break;
    case 0x40>>2:
      s->hckdiv = value;
      break;
    case 0x44>>2:
      s->pwr = value;
      break;
    case 0x48>>2:
      s->pll397 = value;
      break;
    case 0x4c>>2:
      s->osc = value;
      break;
    case 0x50>>2:
      s->sysclk = value;
      break;
    case 0x54>>2:
      s->lcdclk = value;
      break;
    case 0x58>>2:
      s->hckpll = value;
      break;
    case 0x60>>2:
      s->adcclk_1 = value;
      break;
    case 0x64>>2:
      s->usbctl = value;
      break;

    case 0x78>>2:
      s->sspclk = value;
      break;
    case 0x7c>>2:
      s->i2sclk = value;
      break;

    case 0x80>>2:
      s->sdcard = value;
      break;
    case 0x90>>2:
      s->macclk = value;
      break;
    case 0xa4>>2:
      s->test_clk = value;
      break;
    case 0xa8>>2:
      s->swint = value;
      break;
    case 0xac>>2:
      s->i2cclk = value;
      break;
    case 0xb0>>2:
      s->kbdclk = value;
      break;
    case 0xb4>>2:
      s->adcclk = value;
      break;
    case 0xb8>>2:
      s->pwmclk = value;
      break;
    case 0xbc>>2:
      s->tmrclk = value;
      break;
    case 0xc0>>2:
      s->tmrclk_1 = value;
      break;
    case 0xc4>>2:
      s->spiclk = value;
      break;
    case 0xc8>>2:
      s->flashclk = value;
      break;
    case 0xd0>>2:
      s->uart3 = value;
      break;
    case 0xd4>>2:
      s->uart4 = value;
      break;
    case 0xd8>>2:
      s->uart5 = value;
      break;
    case 0xdc>>2:
      s->uart6 = value;
      break;
    case 0xe0>>2:
      s->irdaclk = value;
      break;
    case 0xe4>>2:
      s->uartclk = value;
      break;
    case 0xe8>>2:
      s->dmaclk = value;
      break;
    case 0xec>>2:
      s->autoclock = value;
      break;

    default:
        fprintf(stderr, "lpc32xx_clkpm_write: Bad offset 0x%02x\n",
                (int)offset);
    }

    lpc32xx_clkpm_update(s);
}

static const MemoryRegionOps lpc32xx_clkpm_ops = {
    .read = lpc32xx_clkpm_read,
    .write = lpc32xx_clkpm_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int lpc32xx_clkpm_init(SysBusDevice *dev)
{
    lpc32xx_clkpm_state *s = FROM_SYSBUS(lpc32xx_clkpm_state, dev);

if(V_CLKPM_INIT)
    fprintf(stderr, "%s.i\n", "clkpm");

    sysbus_init_irq(dev, &s->irq);

    memory_region_init_io(&s->iomem, &lpc32xx_clkpm_ops,
                          s, "lpc32xx-clkpm", 0x0100);
    sysbus_init_mmio(dev, &s->iomem);

    return 0;
}

static void lpc32xx_clkpm_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc  = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);
    k->init = lpc32xx_clkpm_init;
    dc->vmsd = &vmstate_lpc32xx_clkpm;
    dc->reset = lpc32xx_clkpm_reset;
    dc->desc = "LPC32xx Clock and Power Managament";
}

static const TypeInfo lpc32xx_clkpm_info = {
    .name = "lpc32xx-clkpm",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(lpc32xx_clkpm_state),
    .class_init = lpc32xx_clkpm_class_init,
};

static void lpc32xx_clkpm_register_types(void)
{
    type_register_static(&lpc32xx_clkpm_info);
}

type_init(lpc32xx_clkpm_register_types)
