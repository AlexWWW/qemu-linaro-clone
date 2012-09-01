/*
 *  NXP LPC32xx based boards emulation
 *
 *  Copyright (c) 2012  Alex Vedenov <vedenov86743+qemu at gmail>
 *
 *  License: GPL v2, like kernel
 *
 */

#include "sysemu.h"
#include "sysbus.h"
#include "net.h"
#include "arm-misc.h"
#include "exec-memory.h"
#include "boards.h"
#include "lpc32xx.h"

//#undef DEBUG

#define DEBUG

#ifdef DEBUG
    #undef PRINT_DEBUG
    #define  PRINT_DEBUG(fmt, args...) \
        do { \
            fprintf(stderr, "  [%s:%d]   "fmt, __func__, __LINE__, ##args); \
        } while (0)
#else
    #define  PRINT_DEBUG(fmt, args...)  do {} while (0)
#endif

typedef enum LPC32xxBoardType {
    LPC32xx_BOARD_PHY3250,
    LPC32xx_BOARD_EA3250,
    LPC32xx_BOARD_FDI3250,
    LPC32xx_NUM_OF_BOARDS
} LPC32xxBoardType;

static int lpc32xx_board_id[LPC32xx_NUM_OF_BOARDS] = {
    [LPC32xx_BOARD_PHY3250]  = 0x2511,
    [LPC32xx_BOARD_EA3250]   = 0x2512,
    [LPC32xx_BOARD_FDI3250]  = 0x2513,
};

static unsigned long lpc32xx_board_ram_size[LPC32xx_NUM_OF_BOARDS] = {
    [LPC32xx_BOARD_PHY3250]  = 0x40000000,
    [LPC32xx_BOARD_EA3250]   = 0x04000000, /* 64 Mb */
    [LPC32xx_BOARD_FDI3250]  = 0x40000000,
};

static struct arm_boot_info lpc32xx_board_binfo = {
    .loader_start     = LPC32xx_BASE_BOOT_ADDR,
    .nb_cpus          = 1,
};

static QEMUMachine lpc32xx_machines[LPC32xx_NUM_OF_BOARDS];

static LPC32xxState *lpc32xx_boards_init_common(
        const char *kernel_filename,
        const char *kernel_cmdline,
        const char *initrd_filename,
        LPC32xxBoardType board_type)
{
    lpc32xx_board_binfo.ram_size = lpc32xx_board_ram_size[board_type];
    lpc32xx_board_binfo.board_id = lpc32xx_board_id[board_type];
    lpc32xx_board_binfo.kernel_filename = kernel_filename;
    lpc32xx_board_binfo.initrd_filename = initrd_filename;
    lpc32xx_board_binfo.kernel_cmdline = kernel_cmdline;

    PRINT_DEBUG("\n ram_size: %luMiB [0x%08lx]\n"
            " kernel_filename: %s\n"
            " kernel_cmdline: %s\n"
            " initrd_filename: %s\n",
            lpc32xx_board_ram_size[board_type] / 1048576,
            lpc32xx_board_ram_size[board_type],
            kernel_filename,
            kernel_cmdline,
            initrd_filename);

    return lpc32xx_init(get_system_memory(),
            lpc32xx_board_ram_size[board_type]);
}

static void ea3250_init(ram_addr_t ram_size,
        const char *boot_device,
        const char *kernel_filename, const char *kernel_cmdline,
        const char *initrd_filename, const char *cpu_model)
{
    LPC32xxState *s = lpc32xx_boards_init_common(kernel_filename,
            kernel_cmdline, initrd_filename, LPC32xx_BOARD_EA3250);

    arm_load_kernel(s->cpu, &lpc32xx_board_binfo);
}

static void fdi3250_init(ram_addr_t ram_size,
        const char *boot_device,
        const char *kernel_filename, const char *kernel_cmdline,
        const char *initrd_filename, const char *cpu_model)
{
    LPC32xxState *s = lpc32xx_boards_init_common(kernel_filename,
            kernel_cmdline, initrd_filename, LPC32xx_BOARD_FDI3250);

    arm_load_kernel(s->cpu, &lpc32xx_board_binfo);
}

static void phy3250_init(ram_addr_t ram_size,
        const char *boot_device,
        const char *kernel_filename, const char *kernel_cmdline,
        const char *initrd_filename, const char *cpu_model)
{
    LPC32xxState *s = lpc32xx_boards_init_common(kernel_filename,
            kernel_cmdline, initrd_filename, LPC32xx_BOARD_PHY3250);

    arm_load_kernel(s->cpu, &lpc32xx_board_binfo);
}

static QEMUMachine lpc32xx_machines[LPC32xx_NUM_OF_BOARDS] = {
    [LPC32xx_BOARD_PHY3250] = {
        .name = "phy3250",
        .desc = "Phytec 3250 development board",
        .init = phy3250_init,
    },
    [LPC32xx_BOARD_EA3250] = {
        .name = "ea3250",
        .desc = "Embedded Artists LPC3250 Developer\'s Kit",
        .init = ea3250_init,
    },
    [LPC32xx_BOARD_FDI3250] = {
        .name = "fdi3250",
        .desc = "Future Designs LPC3250 Touch screen kit",
        .init = fdi3250_init,
    },
};

static void lpc32xx_machine_init(void)
{
    qemu_register_machine(&lpc32xx_machines[LPC32xx_BOARD_PHY3250]);
    qemu_register_machine(&lpc32xx_machines[LPC32xx_BOARD_EA3250]);
    qemu_register_machine(&lpc32xx_machines[LPC32xx_BOARD_FDI3250]);
}

machine_init(lpc32xx_machine_init);
