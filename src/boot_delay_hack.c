#include <zephyr/kernel.h>
#include <zephyr/init.h>

static int force_boot_delay(void) {
    k_busy_wait(1000 * 2000);
    return 0;
}

// POST_KERNELより早い PRE_KERNEL_2 フェーズで実行
SYS_INIT(force_boot_delay, POST_KERNEL, 10);