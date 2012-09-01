/*
 *  NXP LPC32xx interrupt controller emulation
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


#define V_PIC_UPD       01
#define V_SET_IRQ       01


typedef struct lpc32xx_pic_state
{
    SysBusDevice busdev;
    MemoryRegion iomem;
    uint32_t pending;
    uint32_t enable;
    uint32_t polarity;
    uint32_t edge_triggered;
    uint32_t is_fiq;
    qemu_irq irq;
    qemu_irq fiq;
} lpc32xx_pic_state;

static const VMStateDescription vmstate_lpc32xx_pic = {
    .name = "lpc32xx-pic",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(pending, lpc32xx_pic_state),
        VMSTATE_UINT32(enable, lpc32xx_pic_state),
        VMSTATE_UINT32(polarity, lpc32xx_pic_state),
        VMSTATE_UINT32(edge_triggered, lpc32xx_pic_state),
        VMSTATE_UINT32(is_fiq, lpc32xx_pic_state),
        VMSTATE_END_OF_LIST()
    }
};

static void lpc32xx_pic_update(lpc32xx_pic_state *s)
{
    uint32_t flags = s->pending & s->enable;

if(V_PIC_UPD)
    fprintf (stderr, "pic.upd %s RSR.IER.APR.ATR %08x.%08x.%08x.%08x  irq:fiq %d:%d\n",
             s->busdev.qdev.id,
             s->pending, s->enable,
             s->polarity, s->is_fiq,
             !!(flags & ~s->is_fiq),
             !!(flags &  s->is_fiq));

    qemu_set_irq(s->fiq, !!(flags &  s->is_fiq));
    qemu_set_irq(s->irq, !!(flags & ~s->is_fiq));
}

static void lpc32xx_pic_set_irq(void *opaque, int irq, int level)
{
    lpc32xx_pic_state *s = (lpc32xx_pic_state *)opaque;
    unsigned line = 1u << irq;
    /* [tbd] edge */
    bool active = (!!level) == !!(s->polarity & line);

if(V_SET_IRQ)
    fprintf (stderr, "pic.irq %s %2d@%d %08x:%d\n",
             s->busdev.qdev.id,
             irq, level,
             line, active);

    if (active)
        s->pending |=  line;
    else
        s->pending &= ~line;
    lpc32xx_pic_update(s);
}

static uint64_t lpc32xx_pic_read(void *opaque, target_phys_addr_t offset,
                             unsigned size)
{
    lpc32xx_pic_state *s = (lpc32xx_pic_state *)opaque;

    switch (offset >> 2) {
    case 0: /* Enable */
        return s->enable;
    case 1: /* Raw Status (pending ints) */
        return s->pending;
    case 2: /* Status */
        return s->pending & s->enable;
    case 3: /* Activation Polarity */
        return s->polarity;
    case 4: /* Activation Type */
        return s->edge_triggered;
    case 5: /* Interrupt Type */
        return s->is_fiq;
    default:
        printf ("lpc32xx_pic_read: Bad register offset 0x%x\n", (int)offset);
        return 0;
    }
}

static void lpc32xx_pic_write(void *opaque, target_phys_addr_t offset,
                          uint64_t value, unsigned size)
{
    lpc32xx_pic_state *s = (lpc32xx_pic_state *)opaque;

    switch (offset >> 2) {
    case 0: /* Enable */
        s->enable = value;
        break;
    case 1: /* Raw Status (pending ints) */
        s->pending &= ~(value & s->edge_triggered);
        break;
    case 3: /* Activation Polarity */
        s->polarity = value;
        break;
    case 4: /* Activation Type */
        s->edge_triggered = value;
        break;
    case 5: /* Interrupt Type */
        s->is_fiq = value;
        break;
    default:
        printf ("lpc32xx_pic_write: Bad register offset 0x%x\n", (int)offset);
        return;
    }
    lpc32xx_pic_update(s);
}

static const MemoryRegionOps lpc32xx_pic_ops = {
    .read = lpc32xx_pic_read,
    .write = lpc32xx_pic_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void lpc32xx_reset(DeviceState *d)
{
    lpc32xx_pic_state *s = DO_UPCAST(lpc32xx_pic_state, busdev.qdev, d);

    s->pending = 0;
    s->enable = 0;
    s->polarity = 0;
    s->edge_triggered = 0;
    s->is_fiq = 0;
}

static int lpc32xx_pic_init_named(SysBusDevice *dev, const char *name)
{
    lpc32xx_pic_state *s = FROM_SYSBUS(lpc32xx_pic_state, dev);

    memory_region_init_io(&s->iomem, &lpc32xx_pic_ops, s, name, 0x4000);
    sysbus_init_mmio(dev, &s->iomem);
    qdev_init_gpio_in(&dev->qdev, lpc32xx_pic_set_irq, 32);
    sysbus_init_irq(dev, &s->irq);
    sysbus_init_irq(dev, &s->fiq);
    return 0;
}

static int lpc32xx_mic_init(SysBusDevice *dev)
{
    return lpc32xx_pic_init_named(dev, "lpc32xx-mic");
}

static int lpc32xx_sic1_init(SysBusDevice *dev)
{
    return lpc32xx_pic_init_named(dev, "lpc32xx-sic1");
}

static int lpc32xx_sic2_init(SysBusDevice *dev)
{
    return lpc32xx_pic_init_named(dev, "lpc32xx-sic2");
}


static void lpc32xx_mic_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = lpc32xx_mic_init;
    dc->no_user = 1;
    dc->vmsd = &vmstate_lpc32xx_pic;
}

static void lpc32xx_sic1_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = lpc32xx_sic1_init;
    dc->no_user = 1;
    dc->vmsd = &vmstate_lpc32xx_pic;
}

static void lpc32xx_sic2_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = lpc32xx_sic2_init;
    dc->no_user = 1;
    dc->reset = lpc32xx_reset;
    dc->vmsd = &vmstate_lpc32xx_pic;
}

static TypeInfo lpc32xx_mic_info = {
    .name          = "lpc32xx-mic",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(lpc32xx_pic_state),
    .class_init    = lpc32xx_mic_class_init,
};

static TypeInfo lpc32xx_sic1_info = {
    .name          = "lpc32xx-sic1",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(lpc32xx_pic_state),
    .class_init    = lpc32xx_sic1_class_init,
};

static TypeInfo lpc32xx_sic2_info = {
    .name          = "lpc32xx-sic2",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(lpc32xx_pic_state),
    .class_init    = lpc32xx_sic2_class_init,
};

static void lpc32xx_register_types(void)
{
    type_register_static(&lpc32xx_mic_info);
    type_register_static(&lpc32xx_sic1_info);
    type_register_static(&lpc32xx_sic2_info);
}

type_init(lpc32xx_register_types)
