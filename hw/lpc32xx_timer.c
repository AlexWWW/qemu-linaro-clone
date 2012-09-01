/*
 *  LPC32xx timer emulation
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


#define V_GET_COUNT     0
#define V_SET_FREQ      01
#define V_TIMER_RD      01
#define V_TIMER_WR      01
#define V_TIMER_TICK    01
#define V_TIMER_INIT    01


#if 0001
struct ptimer_state
{
    uint8_t enabled; /* 0 = disabled, 1 = periodic, 2 = oneshot.  */
    uint64_t limit;
    uint64_t delta;
    uint32_t period_frac;
    int64_t period;
    int64_t last_event;
    int64_t next_event;
    QEMUBH *bh;
    QEMUTimer *timer;
};
#endif


typedef struct lpc32xx_timer_state {
    SysBusDevice busdev;
    ptimer_state *timer;
    MemoryRegion iomem;
    DeviceState *ccm;

    uint32_t ir;
    uint32_t tcr;
    uint32_t tc;
    uint32_t pr;
    uint32_t pc;
    uint32_t mcr;
    uint32_t mr [4];
    uint32_t ccr;
    uint32_t cr [4];
    uint32_t emr;
    uint32_t ctcr;

    uint32_t freq;
    int int_level;
    qemu_irq irq;

} lpc32xx_timer_state;

static const VMStateDescription vmstate_lpc32xx_timer = {
    .name = "lpc32xx-timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(ir, lpc32xx_timer_state),
        VMSTATE_UINT32(tcr, lpc32xx_timer_state),
        VMSTATE_UINT32(tc, lpc32xx_timer_state),
        VMSTATE_UINT32(pr, lpc32xx_timer_state),
        VMSTATE_UINT32(pc, lpc32xx_timer_state),
        VMSTATE_UINT32(mcr, lpc32xx_timer_state),
        VMSTATE_UINT32(ccr, lpc32xx_timer_state),
        VMSTATE_UINT32(emr, lpc32xx_timer_state),
        VMSTATE_UINT32(ctcr, lpc32xx_timer_state),
        VMSTATE_END_OF_LIST()
    }
};

static void lpc32xx_timer_update(lpc32xx_timer_state *s)
{
    qemu_set_irq(s->irq, !!(s->ir & 0xff));
}

static void lpc32xx_timer_reset(DeviceState *dev)
{
    lpc32xx_timer_state *s = container_of(dev, lpc32xx_timer_state, busdev.qdev);

    s->ir = 0;
    s->tcr = 0;
    s->tc = 0;
    s->pr = 0;
    s->pc = 0;
    s->mcr = 0;
    s->ccr = 0;
    s->emr = 0;
    s->ctcr = 0;

    ptimer_stop(s->timer);
#if 000
    ptimer_set_count(s->timer, TIMER_MAX);
#endif
    ptimer_set_count(s->timer, 0xffffffff);

}

static uint64_t lpc32xx_timer_read__proper(void *opaque, target_phys_addr_t offset,
                                           unsigned size)
{
    lpc32xx_timer_state *s = (lpc32xx_timer_state *)opaque;

    switch (offset >> 2) {
    case 0: /* Interrupt */
        return s->ir;
    case 1: /* Timer Control */
        return s->tcr;
    case 2: /* Timer Counter */
        /* TCR.CRST = 0, TCR.CENB = 1 */
        if ((s->tcr & 0x03) == 0x01) {
            uint64_t counter = ptimer_get_count(s->timer);
if(V_GET_COUNT)
            fprintf(stderr, "%s.c %08x%08x\n", s->busdev.qdev.id,
                (uint32_t)(counter >> 32),
                (uint32_t)(counter & 0xffffffff));
/*
            return counter;
 */
            return (unsigned)(0-counter);
        }
        return s->tc;
    case 3: /* Prescale */
        return s->pr;
    case 4: /* Prescale Counter */
        /* [tbd] interpolate pc */
        return s->pc;
    case 5: /* Match Control */
        return s->mcr;
    case 6:
    case 7:
    case 8:
    case 9: /* Match */
        return s->mr [(offset >> 2)-6];
    case 10: /* Capture Control */
        return s->ccr;
    case 11:
    case 12:
    case 13:
    case 14: /* Capture */
        return s->cr [(offset >> 2)-11];
    case 15: /* External Match */
        return s->emr;
    case 0x70>>2: /* Count Control */
        return s->ctcr;
#if 000
    case 4: /* CNT */
        return ptimer_get_count(s->timer);
#endif
    }

    fprintf(stderr, "lpc32xx_timer_read: Bad offset 0x%02x\n",
            (int)offset);
    return 0;
}

static uint64_t lpc32xx_timer_read(void *opaque, target_phys_addr_t offset,
                                   unsigned size)
{
    lpc32xx_timer_state *s = (lpc32xx_timer_state *) opaque;
    uint64_t value = lpc32xx_timer_read__proper(opaque, offset, size);

if(V_TIMER_RD)
    fprintf(stderr, "%s.r +%02lx[%u] %08lx\n",
            s->busdev.qdev.id,
            (long)offset, size, (long)value);

    return value;
}

static void set_timer_freq(lpc32xx_timer_state *s)
{
    int clksrc;
    unsigned prescaler;
#if 000
    uint32_t freq;
#endif

    int clksrc_freq;
    int64_t clksrc_period;
    uint32_t clksrc_period_frac;

    int64_t period;

/*
    clksrc = (s->cr & CR_CLKSRC_MASK) >> CR_CLKSRC_SHIFT;
    prescaler = 1 + ((s->cr >> CR_PRESCALE_SHIFT) & CR_PRESCALE_MASK);
    freq = lpc32xx_clock_frequency(s->ccm, lpc32xx_timerp_clocks[clksrc]) / prescaler;
 */
    clksrc = 13000000;
    /* [tbd] uint32_t prescaler overflows on pr = ~0 */
    prescaler = 1 + s->pr;
#if 000
    /* fkd up with large pr and then freq = 0 */
    freq = clksrc / prescaler;

if(V_SET_FREQ)
    fprintf(stderr, "%s.f %9u\n", s->busdev.qdev.id, freq);

    s->freq = freq;

    if (freq) {
        ptimer_set_freq(s->timer, freq);
if(V_SET_FREQ)
        fprintf(stderr, "%s.p %08x%08x %08x\n", s->busdev.qdev.id, 
            (uint32_t)(s->timer->period >> 32),
            (uint32_t)(s->timer->period & 0xffffffff),
            s->timer->period_frac);
    }
#endif
    /* make fn conditions happier */
    s->freq = 1 + clksrc / prescaler;

    /* these are all constants[clksrc] */
    clksrc_freq = clksrc;
    clksrc_period = 1000000000ll / clksrc_freq;
    clksrc_period_frac = (1000000000ll << 32) / clksrc_freq;

    period = prescaler * clksrc_period;
    period += (prescaler * (uint64_t)clksrc_period_frac) >> 32;

    ptimer_set_period(s->timer, period);
if(V_SET_FREQ)
    fprintf(stderr, "%s.p %08x%08x %08x\n", s->busdev.qdev.id, 
            (uint32_t)(s->timer->period >> 32),
            (uint32_t)(s->timer->period & 0xffffffff),
            s->timer->period_frac);

}

static void lpc32xx_timer_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
    lpc32xx_timer_state *s = (lpc32xx_timer_state *)opaque;

if(V_TIMER_WR)
    fprintf(stderr, "%s.w +%02lx[%u] %08lx\n",
            s->busdev.qdev.id,
            (long)offset, size, (long)value);

    switch (offset >> 2) {
    case 0: /* Interrupt */
        s->ir &= ~value;
        break;
    case 1: /* Timer Control */
        s->tcr = value;
        if (s->tcr & 0x02) {
            s->tc = 0;
            s->pc = 0;
        }
        if ((s->tcr & 0x03) == 0x01 && s->freq) {
            /* any match actions requested? */
            if(s->mcr & 07777) {
                /* [tbd] support MCR1-3 */
                if(s->mcr & 00007) {
                    ptimer_set_count(s->timer, s->mr [0]);
                    ptimer_run(s->timer, 1);
                    break;
                }
            }
            ptimer_run(s->timer, 0);
        } else {
            ptimer_stop(s->timer);
        }
        break;
    case 2: /* Timer Counter */
        s->tc = value;
        ptimer_set_count(s->timer, s->tc);
        break;
    case 3: /* Prescale */
/*
        if (0 < value)
            printf("%s.w ** PR > 0 is not supported\n", s->busdev.qdev.id);
 */
        s->pr = value;
        set_timer_freq(s);
        break;
    case 4: /* Prescale Counter */
        s->pc = value;
        break;
    case 5: /* Match Control */
        /* [tbd] support MR1-3 */
        if (value & 07770)
            printf("%s.w ** MCR1-3 are not supported\n", s->busdev.qdev.id);
        s->mcr = value;
        break;
    case 6:
    case 7:
    case 8:
    case 9: /* Match */
        s->mr[(offset >> 2)-6] = value;
        break;
    case 10: /* Capture Control */
        /* [tbd] need gpio input pins for external sources */
        /* [tbd] support CCR0-3 */
        if(value & (07777))
            printf("%s.w ** CCR0-3 are not supported\n", s->busdev.qdev.id);
        s->ccr = value;
        break;
#if 000 
    /* CR0-3 are RO */
    case 11:
    case 12:
    case 13:
    case 14: /* Capture */
        s->cr[(offset >> 2)-11] = value;
        break;
#endif
    case 15: /* External Match */
        s->emr = value;
        break;
    case 0x70>>2: /* Count Control */
        s->ctcr = value;
        break;

    default:
        fprintf(stderr, "lpc32xx_timer_write: Bad offset 0x%02x\n",
                   (int)offset);
    }

    lpc32xx_timer_update(s);
}

static void lpc32xx_timer_tick(void *opaque)
{
    lpc32xx_timer_state *s = (lpc32xx_timer_state *)opaque;

if(V_TIMER_TICK)
    fprintf(stderr, "%s.k\n", s->busdev.qdev.id);

    /* MR0 */
    if (s->mcr & 00001)
        s->ir |= 0x01;
    if (s->mcr & 00002)
        s->tc = 0;
    if (s->mcr & 00004)
        s->tcr &= ~0x01;

    /* [tbd] support MR1-3, CR0-3 interrupts */

    lpc32xx_timer_update(s);
}

static const MemoryRegionOps lpc32xx_timer_ops = {
    .read = lpc32xx_timer_read,
    .write = lpc32xx_timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int lpc32xx_timer_init(SysBusDevice *dev)
{
    lpc32xx_timer_state *s = FROM_SYSBUS(lpc32xx_timer_state, dev);
    QEMUBH *bh;

if(V_TIMER_INIT)
    fprintf(stderr, "%s.i\n", s->busdev.qdev.id);

    sysbus_init_irq(dev, &s->irq);

    memory_region_init_io(&s->iomem, &lpc32xx_timer_ops,
                          s, "lpc32xx-timer", 0x4000);
    sysbus_init_mmio(dev, &s->iomem);

    bh = qemu_bh_new(lpc32xx_timer_tick, s);
    s->timer = ptimer_init(bh);

    return 0;
}

static void lpc32xx_timer_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc  = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);
    k->init = lpc32xx_timer_init;
    dc->vmsd = &vmstate_lpc32xx_timer;
    dc->reset = lpc32xx_timer_reset;
    dc->desc = "LPC32xx Standard Timer";
}

static const TypeInfo lpc32xx_timer_info = {
    .name = "lpc32xx-timer",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(lpc32xx_timer_state),
    .class_init = lpc32xx_timer_class_init,
};

static void lpc32xx_timer_register_types(void)
{
    type_register_static(&lpc32xx_timer_info);
}

type_init(lpc32xx_timer_register_types)
