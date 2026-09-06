/*
 * ハードウェアウォッチドッグ + システムワークキュー死活監視
 *
 * 目的は2つある。
 *
 * 1. フリーズ時に電源再投入を不要にする（心拍停止から約20秒で自動リセット）
 *
 * 2. フリーズの原因を2つの候補から確定させる
 *      自動で復帰した   → システムワークキューが停止していた
 *                          (PMW3610のレベル割り込み無条件再武装ループ、
 *                           あるいはEC11経由の zmk_hog_send_mouse_report の詰まり)
 *      復帰しなかった   → システムワークキューは生きていた
 *                          (hog_work_q が bt_gatt_notify_cb の K_FOREVER で無限待ち。
 *                           Zephyr host/att.c の bt_att_chan_create_pdu 参照)
 *
 * 仕組み:
 *   システムワークキュー上の delayable work が心拍カウンタを進める。
 *   k_timer（システムタイマ割り込み文脈で動くので、スレッドが全滅しても生きている）が
 *   そのカウンタを監視し、進んでいる間だけWDTに餌をやる。
 *   心拍が STALL_LIMIT_MS 止まったら餌やりを止め、ハードウェアにリセットさせる。
 *
 * 注意:
 *   nRF52のWDTは一度started になるとソフトウェアからは停止できない。
 *   無効化したい場合は shield の .conf から CONFIG_WATCHDOG=y を消すこと。
 */

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#if IS_ENABLED(CONFIG_WATCHDOG) && DT_NODE_HAS_STATUS(DT_ALIAS(watchdog0), okay)

#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

#if IS_ENABLED(CONFIG_HWINFO)
#include <zephyr/drivers/hwinfo.h>
#endif

/* CONFIG_LOG_DEFAULT_LEVEL は Kconfig の `if LOG` 内でしか定義されないため使えない
 * (このビルドは CONFIG_ZMK_USB_LOGGING=n でログ無効になりうる)。
 * log.h が常に定義する LOG_LEVEL_INF を使う。 */
LOG_MODULE_REGISTER(sosho_wdt, LOG_LEVEL_INF);

/* WDTのタイムアウト。UF2書き込み中にリセットが割り込まないよう余裕を持たせている */
#define WDT_TIMEOUT_MS 15000

/* 心拍の刻み = WDTへの餌やり間隔 */
#define HEARTBEAT_PERIOD_MS 500

/* 心拍がこの時間止まったらシステムワークキューが死んだと判定する。
 * 通常運転でシステムワークキューが5秒連続で塞がることはない
 * （最悪ケースの zmk_hog_send_mouse_report の k_msgq_put でも1回あたり100ms）。 */
#define STALL_LIMIT_MS 5000

static const struct device *const wdt_dev = DEVICE_DT_GET(DT_ALIAS(watchdog0));
static int wdt_channel = -1;

static atomic_t heartbeat;

/* 以下2つは feed_timer_handler からのみ触るのでアトミック不要 */
static atomic_val_t last_heartbeat;
static uint32_t stalled_ms;

static void heartbeat_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(heartbeat_work, heartbeat_work_handler);

static void heartbeat_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    atomic_inc(&heartbeat);
    k_work_schedule(&heartbeat_work, K_MSEC(HEARTBEAT_PERIOD_MS));
}

static void feed_timer_handler(struct k_timer *timer) {
    ARG_UNUSED(timer);

    atomic_val_t now = atomic_get(&heartbeat);

    if (now != last_heartbeat) {
        last_heartbeat = now;
        stalled_ms = 0;
    } else {
        stalled_ms += HEARTBEAT_PERIOD_MS;
    }

    if (stalled_ms < STALL_LIMIT_MS) {
        wdt_feed(wdt_dev, wdt_channel);
    }
    /* else: 餌やりを止める。WDT_TIMEOUT_MS 後にハードウェアがSoCをリセットする。 */
}
static K_TIMER_DEFINE(feed_timer, feed_timer_handler, NULL);

static void report_reset_cause(void) {
#if IS_ENABLED(CONFIG_HWINFO)
    uint32_t cause = 0;

    if (hwinfo_get_reset_cause(&cause) != 0) {
        return;
    }
    hwinfo_clear_reset_cause();

    /* ログはシリアルコンソールで読むのでASCIIにしておく */
    if (cause & RESET_WATCHDOG) {
        LOG_WRN("previous boot was a WATCHDOG reset (cause 0x%08x)"
                " -> the system workqueue had stalled",
                (unsigned int)cause);
    } else {
        LOG_INF("reset cause 0x%08x", (unsigned int)cause);
    }
#endif
}

static int sosho_wdt_init(void) {
    if (!device_is_ready(wdt_dev)) {
        LOG_ERR("watchdog device not ready");
        return -ENODEV;
    }

    report_reset_cause();

    const struct wdt_timeout_cfg cfg = {
        .flags = WDT_FLAG_RESET_SOC,
        .window =
            {
                .min = 0,
                .max = WDT_TIMEOUT_MS,
            },
        .callback = NULL,
    };

    wdt_channel = wdt_install_timeout(wdt_dev, &cfg);
    if (wdt_channel < 0) {
        LOG_ERR("wdt_install_timeout failed (%d)", wdt_channel);
        return wdt_channel;
    }

    atomic_set(&heartbeat, 0);
    last_heartbeat = 0;
    stalled_ms = 0;

    /* 心拍だけ先に回し始めておく。餌やりタイマは wdt_setup の後で起動する
     * （wdt_feed を start 前に呼ばないため）。 */
    k_work_schedule(&heartbeat_work, K_NO_WAIT);

    /* WDT_OPT_PAUSE_IN_SLEEP は意図的に渡していない。渡すとCPUアイドル中に
     * カウンタが止まるため、「スレッドがブロックしていてCPUは暇」というケース
     * （= hog_work_q の無限待ち側）を取りこぼす。 */
    int err = wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
    if (err) {
        LOG_ERR("wdt_setup failed (%d)", err);
        return err;
    }

    k_timer_start(&feed_timer, K_MSEC(HEARTBEAT_PERIOD_MS), K_MSEC(HEARTBEAT_PERIOD_MS));

    LOG_INF("watchdog armed: timeout %dms, stall limit %dms", WDT_TIMEOUT_MS, STALL_LIMIT_MS);

    return 0;
}

SYS_INIT(sosho_wdt_init, APPLICATION, 90);

#endif /* CONFIG_WATCHDOG && watchdog0 okay */
