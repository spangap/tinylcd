/**
 * tinylcd.cpp — paged mono-OLED UI, end to end.
 *
 * See tinylcd.h for the API contract. Layout:
 *
 *   1. Page registry + cross-task requests (atomics; safe from any task/ISR).
 *   2. u8g2 glue: I2C byte callback + GPIO/delay callback on the new
 *      i2c_master driver. The task owns the bus.
 *   3. Rendering: splash, page dispatch, dot indicator.
 *   4. Button: ANYEDGE ISR wakes the task; a millis()-timestamped state
 *      machine turns edges into clicks (no polling while idle).
 *   5. The display task: one itsPoll() deadline loop.
 *   6. Service + CLI.
 *
 * Everything display-side runs on the tinylcd task. Other tasks only touch
 * the registry spinlock, the request atomics and ITS aux sends — never u8g2.
 */
#include "tinylcd.h"

#include <atomic>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_ll.h"

#include "cli.h"
#include "compat.h"
#include "i2c_helper.h"
#include "its.h"
#include "log.h"
#include "pm.h"
#include "spangap.h"
#include "spi_helper.h"
#include "storage.h"

#define TINYLCD_HAVE_PANEL (CONFIG_TINYLCD_SDA_PIN >= 0 && CONFIG_TINYLCD_SCL_PIN >= 0)

/* =========================================================================
 * 1. Page registry + cross-task requests
 * ========================================================================= */

#define TINYLCD_MAX_PAGES 16   /* handle == index; bounded by the redraw bitmask */
#define TINYLCD_RUN_PORT  1    /* ITS aux port for tinylcdRun payloads */

struct TinyPage {
    const char*       name;
    tinylcd_draw_cb_t cb;
    int               refreshMs;
};

static TinyPage            s_pages[TINYLCD_MAX_PAGES];
static std::atomic<int>    s_pageCount{0};
static std::atomic<int>    s_current{TINYLCD_NO_PAGE};   /* NO_PAGE = splash/blank */
static portMUX_TYPE        s_lock = portMUX_INITIALIZER_UNLOCKED;

/* Cross-task requests, consumed by the task each wake. navTarget latest-wins;
 * navDelta accumulates; redrawMask is per-page bits checked against s_current
 * at render time (that check is what makes tinylcdDraw a no-op for a page that
 * is not on screen). */
static std::atomic<int>      s_navDelta{0};
static std::atomic<int>      s_navTarget{TINYLCD_NO_PAGE};
static std::atomic<uint32_t> s_redrawMask{0};
static TaskHandle_t          s_task = nullptr;

/* tinylcdRun before the task exists: parked here, drained at task start. The
 * queue is what frees callers from ordering against tinylcd's own init (net's
 * platform-band hook runs before the whole straddle band, for instance). */
struct TinyRun { tinylcd_fn_t fn; void* arg; };
static TinyRun s_earlyRuns[16];
static int     s_earlyRunCount = 0;

static void tinylcdWake(void)
{
    if (s_task) xTaskNotifyGive(s_task);
}

tinylcd_page_t tinylcdAddPage(const char* name, tinylcd_draw_cb_t cb, int refreshMs)
{
    tinylcd_page_t h = TINYLCD_NO_PAGE;
    portENTER_CRITICAL(&s_lock);
    int n = s_pageCount.load(std::memory_order_relaxed);
    if (n < TINYLCD_MAX_PAGES) {
        s_pages[n] = { name, cb, refreshMs };
        s_pageCount.store(n + 1, std::memory_order_release);
        h = n;
    }
    portEXIT_CRITICAL(&s_lock);
    if (h == TINYLCD_NO_PAGE) {
        err("page table full (%d), '%s' dropped", TINYLCD_MAX_PAGES, name);
        return h;
    }
    /* A new page changes the dot row of whatever is on screen. */
    if (s_current.load() != TINYLCD_NO_PAGE) tinylcdDraw(s_current.load());
    return h;
}

void tinylcdPageNext(void) { s_navDelta.fetch_add(1);  tinylcdWake(); }
void tinylcdPagePrev(void) { s_navDelta.fetch_sub(1);  tinylcdWake(); }

void tinylcdPageShow(tinylcd_page_t page)
{
    if (page < 0 || page >= s_pageCount.load()) return;
    s_navTarget.store(page);
    tinylcdWake();
}

bool tinylcdPageIsCurrent(tinylcd_page_t page)
{
    return page != TINYLCD_NO_PAGE && s_current.load() == page;
}

void tinylcdDraw(tinylcd_page_t page)
{
    if (page < 0 || page >= TINYLCD_MAX_PAGES) return;
    s_redrawMask.fetch_or(1u << page);
    tinylcdWake();
}

void tinylcdRun(tinylcd_fn_t fn, void* arg)
{
    if (!fn) return;
    if (s_task) {
        TinyRun r = { fn, arg };
        if (!itsSendAuxByTaskHandle(s_task, TINYLCD_RUN_PORT, &r, sizeof r,
                                    pdMS_TO_TICKS(1000)))
            err("tinylcdRun: aux send failed, callback dropped");
        return;
    }
    portENTER_CRITICAL(&s_lock);
    bool queued = s_earlyRunCount < (int)(sizeof s_earlyRuns / sizeof s_earlyRuns[0]);
    if (queued) s_earlyRuns[s_earlyRunCount++] = { fn, arg };
    portEXIT_CRITICAL(&s_lock);
    if (!queued) err("tinylcdRun: early queue full, callback dropped");
}

#if TINYLCD_HAVE_PANEL

/* =========================================================================
 * 2. u8g2 glue — I2C byte + GPIO/delay callbacks
 * ========================================================================= */

static u8g2_t                    s_u8g2;
static i2c_master_bus_handle_t   s_i2cBus = nullptr;
static i2c_master_dev_handle_t   s_i2cDev = nullptr;

/* Screen power state — s.tinylcd.standby semantics: >0 = panel sleep after
 * that many seconds without user activity, 0 = always on, -1 = boot with the
 * panel asleep (button wakes), -2 = park the panel asleep and never use it.
 * "Asleep" is SetPowerSave(1): display off, charge pump off — the panel's
 * lowest-power state short of cutting its rail (which is board-owned).
 * All three are task-only (the storage handler runs on the task too). */
static int      s_standby      = 60;
static bool     s_screenOn     = true;
static uint32_t s_lastActivity = 0;
static int      s_orientation  = 0;   /* s.tinylcd.orientation mirror */
static int      s_pageIndicator = 1;  /* s.tinylcd.page_indicator: 0=top,
                                       * 1=bottom, 2=top in orientation 0,
                                       * 3=opposite of 2 */

static void screenSet(bool on)
{
    if (on == s_screenOn) return;
    u8g2_SetPowerSave(&s_u8g2, on ? 0 : 1);
    s_screenOn = on;
    /* Ephemeral (unprefixed = not persisted), subscribable: a page holding a
     * special mode it entered via an absorbed event watches this and reverts
     * to its default rendering when standby ends the session. */
    storageSet("tinylcd.active", on ? 1 : 0);
    if (!on && s_pageCount.load() > 0) {
        /* Standby resets the ring: the next wake starts at the first page. */
        s_current.store(0);
        s_redrawMask.store(0);
    }
}

/* u8g2 brackets every complete I2C transaction in one START/END_TRANSFER pair,
 * so the SEND messages between them are collected here and issued as a single
 * i2c_master_transmit. A framebuffer transaction is a 0x40 control byte plus at
 * most 24 data bytes; a command transaction is a 0x00 control byte plus the
 * command and its args, and 160 bytes covers the longest init run.
 *
 * u8g2 discards the result of every transaction, so a failed one is invisible
 * to it — s_i2cFault carries the failure up to sendBuffer instead. All three
 * are task-only: the tinylcd task is the sole caller of u8g2. */
static uint8_t s_i2cBuf[160];
static size_t  s_i2cLen = 0;
static bool    s_i2cFault = false;

static uint8_t tinylcdByteCb(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr)
{
    switch (msg) {
    case U8X8_MSG_BYTE_INIT: {
        i2c_master_bus_config_t bus = {};
        bus.i2c_port          = -1;   /* any free controller */
        bus.sda_io_num        = (gpio_num_t)CONFIG_TINYLCD_SDA_PIN;
        bus.scl_io_num        = (gpio_num_t)CONFIG_TINYLCD_SCL_PIN;
        bus.clk_source        = I2C_CLK_SRC_DEFAULT;
        bus.glitch_ignore_cnt = 7;
        bus.flags.enable_internal_pullup = SPANGAP_I2C_PULLUP;
        if (i2c_new_master_bus(&bus, &s_i2cBus) != ESP_OK) return 0;
        i2c_device_config_t dev = {};
        dev.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev.device_address  = u8x8_GetI2CAddress(u8x8) >> 1;   /* u8g2 keeps it shifted */
        dev.scl_speed_hz    = CONFIG_TINYLCD_I2C_HZ;
        if (i2c_master_bus_add_device(s_i2cBus, &dev, &s_i2cDev) != ESP_OK) return 0;
        return 1;
    }
    case U8X8_MSG_BYTE_START_TRANSFER:
        s_i2cLen = 0;
        return 1;
    case U8X8_MSG_BYTE_SEND:
        if (s_i2cLen + arg_int > sizeof s_i2cBuf) return 0;
        memcpy(s_i2cBuf + s_i2cLen, arg_ptr, arg_int);
        s_i2cLen += arg_int;
        return 1;
    case U8X8_MSG_BYTE_END_TRANSFER:
        /* The timeout argument is milliseconds, not ticks. A 25-byte
         * transaction takes well under a millisecond at the configured clock;
         * the margin is there to ride out clock stretching, not to hide a
         * wedged bus. */
        if (i2c_master_transmit(s_i2cDev, s_i2cBuf, s_i2cLen, 100) == ESP_OK)
            return 1;
        s_i2cFault = true;
        return 0;
    }
    return 0;
}

static uint8_t tinylcdGpioCb(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int, void* arg_ptr)
{
    switch (msg) {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
#if CONFIG_TINYLCD_RST_PIN >= 0
        {
            gpio_config_t rst = {};
            rst.pin_bit_mask = 1ULL << CONFIG_TINYLCD_RST_PIN;
            rst.mode         = GPIO_MODE_OUTPUT;
            gpio_config(&rst);
            /* Keep the reset line driven through light sleep: pin isolation
             * (CONFIG_PM_SLP_DISABLE_GPIO) would float it, and a drifting-low
             * RST hard-resets the panel mid-sleep. */
            gpio_sleep_sel_dis((gpio_num_t)CONFIG_TINYLCD_RST_PIN);
        }
#endif
        return 1;
    case U8X8_MSG_DELAY_MILLI:      delay(arg_int);              return 1;
    case U8X8_MSG_DELAY_10MICRO:    esp_rom_delay_us(10);        return 1;
    case U8X8_MSG_DELAY_100NANO:    esp_rom_delay_us(1);         return 1;
    case U8X8_MSG_GPIO_RESET:
#if CONFIG_TINYLCD_RST_PIN >= 0
        gpio_set_level((gpio_num_t)CONFIG_TINYLCD_RST_PIN, arg_int);
#endif
        return 1;
    }
    return 1;   /* unhandled GPIO msgs (menu pins etc.) are fine to ack */
}

/* =========================================================================
 * 3. Rendering
 * ========================================================================= */

/* Sending the buffer is 8 tile rows of 8 I2C transactions each: per row one
 * transaction carrying the column+page address, then six data chunks written
 * at the pointer that address left behind. A lost address transaction
 * therefore misplaces a whole row — one 8-pixel band left stale, another
 * overwritten — and a lost data chunk desynchronises the pointer for the rest
 * of the row, so no single transaction can be usefully retried on its own.
 *
 * The repair for any of them is a whole fresh frame, which re-issues every
 * address command and so ends with the panel in a known state whatever its
 * pointer did. Attempts are bounded, and the clean path pays nothing but a
 * flag test. The bus reset between attempts clocks out a slave still holding
 * SDA low; there is no delay in the loop, and the driver call blocks on its
 * own completion, never on a spin. */
#define TINYLCD_SEND_ATTEMPTS 3

static void sendBuffer(void)
{
    static bool faultLogged = false;
    for (int attempt = 1; ; attempt++) {
        s_i2cFault = false;
        u8g2_SendBuffer(&s_u8g2);
        if (!s_i2cFault) { faultLogged = false; return; }
        if (attempt >= TINYLCD_SEND_ATTEMPTS) {
            /* Once per run of failures: a panel that stays unreachable must
             * not turn every redraw into a log line. */
            if (!faultLogged) {
                warn("panel transfer failing after %d attempts", attempt);
                faultLogged = true;
            }
            return;
        }
        i2c_master_bus_reset(s_i2cBus);
    }
}

/* Splash: CONFIG_SPANGAP_FW_NAME centered, in the biggest vendored font that
 * fits. Height reference: logisoso cap heights are ~the point size. */
static void drawSplash(void)
{
    static const uint8_t* fonts[] = {
        u8g2_font_logisoso20_tr, u8g2_font_logisoso16_tr, u8g2_font_7x13B_tr,
    };
    const char* name = CONFIG_SPANGAP_FW_NAME;
    for (const uint8_t* f : fonts) {
        u8g2_SetFont(&s_u8g2, f);
        u8g2_uint_t w = u8g2_GetStrWidth(&s_u8g2, name);
        if (w <= 124 || f == fonts[2]) {
            int a = u8g2_GetAscent(&s_u8g2);
            u8g2_DrawStr(&s_u8g2, (128 - w) / 2, (64 + a) / 2, name);
            return;
        }
    }
}

/* Dot indicator: one dot per page, the current one filled, overlaying the top
 * (rows 0..4) or bottom (rows 59..63) strip per s.tinylcd.page_indicator —
 * fixed edge, or following/countering the orientation so it can sit on the
 * button's side of the case. Skipped for a single page. */
static bool dotsAtTop(void)
{
    switch (s_pageIndicator) {
    case 0:  return true;
    case 1:  return false;
    case 2:  return s_orientation == 0;
    default: return s_orientation != 0;   /* 3 = opposite of 2 */
    }
}

static void drawDots(int count, int current)
{
    if (count < 2) return;
    const int pitch = 8;
    int cy = dotsAtTop() ? 2 : 61;
    int x0 = 64 - (pitch * (count - 1)) / 2;
    for (int i = 0; i < count; i++) {
        if (i == current) u8g2_DrawDisc(&s_u8g2, x0 + i * pitch, cy, 2, U8G2_DRAW_ALL);
        else              u8g2_DrawCircle(&s_u8g2, x0 + i * pitch, cy, 1, U8G2_DRAW_ALL);
    }
}

static void render(bool splash)
{
    u8g2_ClearBuffer(&s_u8g2);
    int cur = s_current.load();
    if (splash || cur == TINYLCD_NO_PAGE) {
        drawSplash();
    } else {
        s_pages[cur].cb(cur, &s_u8g2, TINYLCD_EV_NONE);
        drawDots(s_pageCount.load(), cur);
    }
    sendBuffer();
}

/* Offer an input event to the current page, as a draw call: cleared buffer,
 * and when the page absorbs it (returns true) what it drew IS the response —
 * dots overlaid, buffer sent. False when there is no page to ask or the page
 * declined (its drawing is discarded; the caller applies the default). */
static bool dispatchEvent(int cur, tinylcd_ev_t ev)
{
    if (cur == TINYLCD_NO_PAGE || !s_screenOn) return false;
    u8g2_ClearBuffer(&s_u8g2);
    if (!s_pages[cur].cb(cur, &s_u8g2, ev)) return false;
    drawDots(s_pageCount.load(), cur);
    sendBuffer();
    return true;
}

/* =========================================================================
 * 4. Button — edges from an ISR, clicks decided on the task
 * ========================================================================= */

enum btn_act_t {
    BTN_NONE,
    BTN_CLICK,   /* 30–500 ms press, screen was on: page advance */
    BTN_HOLD,    /* held past 500 ms, screen was on: screen off (fires once) */
    BTN_WAKE,    /* press started with the screen off: wake (press absorbed) */
};

#if CONFIG_TINYLCD_BUTTON_PIN >= 0

/* The trigger is LOW_LEVEL, not ANYEDGE, because the button must work through
 * automatic light sleep: CONFIG_PM_SLP_DISABLE_GPIO isolates pins on sleep
 * entry, GPIO wake only speaks level triggers, and pmGpioWakeEnable both arms
 * the wake and exempts the pin from sleep isolation (same arrangement as the
 * T-Deck's centre button — an edge-typed ISR simply never fires on an idle,
 * sleeping node). A level ISR re-fires as long as the finger is down, so the
 * ISR silences the pin on first fire (LL register write — the driver call
 * takes a non-ISR spinlock) and the task re-arms it once the press ends. */

static void IRAM_ATTR tinylcdBtnISR(void* arg)
{
    gpio_ll_intr_disable(&GPIO, (gpio_num_t)CONFIG_TINYLCD_BUTTON_PIN);
    BaseType_t hp = pdFALSE;
    if (s_task) vTaskNotifyGiveFromISR(s_task, &hp);
    portYIELD_FROM_ISR(hp);
}

/* Light-sleep wake backstop (IDLE-task context, every light-sleep exit): a
 * bouncing press can land the ~1 ms sleep-exit window on a bounce-high gap,
 * so the level ISR never latches it — but a real press holds the pin low, so
 * re-check on the wake itself and poke the task directly. */
static void tinylcdSleepWake(int cause)
{
    if (cause != ESP_SLEEP_WAKEUP_GPIO) return;
    if (gpio_get_level((gpio_num_t)CONFIG_TINYLCD_BUTTON_PIN) != 0) return;
    if (s_task) xTaskNotifyGive(s_task);
}

static void buttonInit(void)
{
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << CONFIG_TINYLCD_BUTTON_PIN;
    io.mode         = GPIO_MODE_INPUT;
    io.pull_up_en   = GPIO_PULLUP_ENABLE;    /* BOOT-style button: active-low */
    io.intr_type    = GPIO_INTR_LOW_LEVEL;
    gpio_config(&io);
    spiHelperEnsureGpioIsr(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add((gpio_num_t)CONFIG_TINYLCD_BUTTON_PIN, tinylcdBtnISR, nullptr);
    pmGpioWakeEnable(CONFIG_TINYLCD_BUTTON_PIN, GPIO_INTR_LOW_LEVEL);
    pmOnLightSleepWake(tinylcdSleepWake);
}

/* Called on every task wake; the press is tracked by level sampling (the ISR
 * is off from first fire until release), timestamps doing the debounce. A
 * press that wakes the screen is absorbed whole — it must not also page-flip
 * or, held long, immediately re-sleep the screen it just woke. BTN_HOLD fires
 * at the threshold, not release, so the 50 ms held sampling is what times it. */
static btn_act_t buttonPoll(bool screenOn, bool* held)
{
    static bool     down = false, absorbed = false, consumed = false;
    static uint32_t downAt = 0;
    btn_act_t act = BTN_NONE;
    uint32_t  now = millis();
    bool level = gpio_get_level((gpio_num_t)CONFIG_TINYLCD_BUTTON_PIN);
    if (!level && !down) {
        down = true; downAt = now; consumed = false;
        absorbed = !screenOn;
        if (absorbed) act = BTN_WAKE;
    } else if (!level && down) {
        if (!absorbed && !consumed && now - downAt >= 500) {
            consumed = true;   /* long press: act now, release is spent */
            act = BTN_HOLD;
        }
    } else if (level && down) {
        down = false;
        uint32_t dt = now - downAt;
        if (!absorbed && !consumed && dt >= 30 && dt < 500) act = BTN_CLICK;
        /* Press over: re-arm the level interrupt for the next one. */
        gpio_intr_enable((gpio_num_t)CONFIG_TINYLCD_BUTTON_PIN);
    } else {
        /* Idle sample with the pin high: covers a press so short it was gone
         * before this task woke — the ISR already silenced the pin, and
         * without this re-arm it would stay silenced for good. Idempotent on
         * the wakes where the interrupt is already live. */
        gpio_intr_enable((gpio_num_t)CONFIG_TINYLCD_BUTTON_PIN);
    }
    *held = down;
    return act;
}

#else
static void buttonInit(void) {}
static btn_act_t buttonPoll(bool, bool* held) { *held = false; return BTN_NONE; }
#endif

/* =========================================================================
 * 5. The display task
 * ========================================================================= */

static void tinylcdTaskFn(void* arg)
{
    itsServerInit();
    itsOnAux(TINYLCD_RUN_PORT, [](TaskHandle_t sender, const void* data, size_t len) {
        if (len != sizeof(TinyRun)) return;
        TinyRun r;
        memcpy(&r, data, sizeof r);
        r.fn(r.arg);
    });

#ifdef CONFIG_TINYLCD_SH1106
    u8g2_Setup_sh1106_i2c_128x64_noname_f(&s_u8g2, U8G2_R0, tinylcdByteCb, tinylcdGpioCb);
    const char* ctrl = "sh1106";
#else
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&s_u8g2, U8G2_R0, tinylcdByteCb, tinylcdGpioCb);
    const char* ctrl = "ssd1306";
#endif
    u8x8_SetI2CAddress(u8g2_GetU8x8(&s_u8g2), CONFIG_TINYLCD_I2C_ADDR << 1);
    u8g2_InitDisplay(&s_u8g2);
    if (!s_i2cDev) {
        /* Bus/panel init failed (pins wrong, panel absent): log once and end.
         * Registrations stay valid no-ops; nothing else references the task. */
        err("panel init failed (sda=%d scl=%d addr=0x%02x)",
            CONFIG_TINYLCD_SDA_PIN, CONFIG_TINYLCD_SCL_PIN, CONFIG_TINYLCD_I2C_ADDR);
        s_task = nullptr;
        killSelf();
    }
    u8g2_SetPowerSave(&s_u8g2, 0);
    u8g2_SetFontMode(&s_u8g2, 1);

    /* The two landscape orientations, runtime-switchable. SetFlipMode is the
     * SSD1306/SH1106 hardware 180° (segment-remap + COM-scan); the redraw
     * request repaints panel RAM in the new scan order. */
    NOW_AND_ON_CHANGE("s.tinylcd.orientation", {
        s_orientation = atoi(val) != 0;
        u8g2_SetFlipMode(&s_u8g2, s_orientation);
        tinylcdDraw(s_current.load() == TINYLCD_NO_PAGE ? 0 : s_current.load());
    });

    /* Where the page dots sit (may track the orientation — see drawDots). */
    NOW_AND_ON_CHANGE("s.tinylcd.page_indicator", {
        s_pageIndicator = atoi(val);
        tinylcdDraw(s_current.load() == TINYLCD_NO_PAGE ? 0 : s_current.load());
    });

    /* External control: writing tinylcd.active (web, CLI, another straddle)
     * sleeps or wakes the screen immediately. tinylcd's own transitions write
     * the same key, so the handler runs under its own echo — screenSet's
     * same-state no-op is what keeps that loop-free. */
    storageSubscribeChanges("tinylcd.active", ON_CHANGE {
        if (atoi(val) != 0) {
            if (s_standby == -2) { storageSet("tinylcd.active", 0); return; }
            if (!s_screenOn) {
                screenSet(true);
                s_lastActivity = millis();
                int cur = s_current.load();
                if (cur != TINYLCD_NO_PAGE) s_redrawMask.fetch_or(1u << cur);
            }
        } else {
            screenSet(false);
        }
    });

    /* Standby mode, live (see the s_standby comment for the semantics). The
     * handler just records; the loop applies. Re-stamping activity makes a
     * lowered timeout count from the change, not from stale history. */
    NOW_AND_ON_CHANGE("s.tinylcd.standby", {
        s_standby = atoi(val);
        s_lastActivity = millis();
    });

    buttonInit();

    if (s_standby < 0) {
        screenSet(false);   /* -1 boots dark (button wakes), -2 parks for good */
    } else {
        render(true);
    }
    storageSet("tinylcd.active", s_screenOn ? 1 : 0);   /* seed the key */
    info("up: %s %dx%d sda=%d scl=%d, splash %d ms, standby %d", ctrl, 128, 64,
         CONFIG_TINYLCD_SDA_PIN, CONFIG_TINYLCD_SCL_PIN, CONFIG_TINYLCD_SPLASH_MS,
         s_standby);

    /* Early tinylcdRun callers (pre-task registrations) run now, on this task. */
    for (;;) {
        TinyRun r = { nullptr, nullptr };
        portENTER_CRITICAL(&s_lock);
        if (s_earlyRunCount > 0) r = s_earlyRuns[--s_earlyRunCount];
        portEXIT_CRITICAL(&s_lock);
        if (!r.fn) break;
        r.fn(r.arg);
    }

    uint32_t splashUntil = millis() + (s_screenOn ? CONFIG_TINYLCD_SPLASH_MS : 0);
    bool     splash      = s_screenOn;   /* dark boots skip the splash phase */
    bool     btnHeld     = false;
    uint32_t lastRefresh = 0;
    int      clicks      = 0;   /* multi-click accumulator */
    uint32_t lastClickAt = 0;
    s_lastActivity = millis();
    const uint32_t CLICK_GAP_MS = 350;   /* max intra-group click spacing */

    for (;;) {
        /* Single wait point: nearest of splash end / held-button sampling /
         * periodic page refresh / standby timeout; ITS aux + storage callbacks
         * + wake notifications all land here. A dark screen has no deadlines
         * of its own — the task parks until something (button, nav, aux)
         * notifies. The 50 ms held-press sampling both guards against a lost
         * release edge and times the long-press-off threshold. */
        uint32_t now = millis();
        TickType_t wait = portMAX_DELAY;
        int cur = s_current.load();
        if (s_screenOn) {
            if (splash) {
                wait = pdMS_TO_TICKS(splashUntil > now ? splashUntil - now : 1);
            } else if (cur != TINYLCD_NO_PAGE && s_pages[cur].refreshMs > 0) {
                uint32_t due = lastRefresh + s_pages[cur].refreshMs;
                wait = pdMS_TO_TICKS(due > now ? due - now : 1);
            }
            if (s_standby > 0) {
                uint32_t off = s_lastActivity + (uint32_t)s_standby * 1000;
                TickType_t w = pdMS_TO_TICKS(off > now ? off - now : 1);
                if (w < wait) wait = w;
            }
        }
        if (clicks > 0) {
            /* A click group is pending: wake at the gap deadline to close it. */
            uint32_t due = lastClickAt + CLICK_GAP_MS;
            TickType_t w = pdMS_TO_TICKS(due > now ? due - now : 1);
            if (w < wait) wait = w;
        }
        if (btnHeld && wait > pdMS_TO_TICKS(50)) wait = pdMS_TO_TICKS(50);
        itsPoll(wait);
        while (itsPoll(0)) {}   /* drain: aux runs + storage change callbacks */

        now = millis();
        bool dirty = false;
        int  count = s_pageCount.load();
        cur = s_current.load();

        switch (buttonPoll(s_screenOn, &btnHeld)) {
        case BTN_WAKE:
            if (s_standby != -2) {
                screenSet(true);
                s_lastActivity = now;
                lastRefresh = 0;   /* stale page: repaint counts as refreshed */
                dirty = true;
            }
            break;
        case BTN_HOLD:
            /* Offered to the page first: an absorbing page keeps the screen
             * on and its drawing is the response; otherwise sleep. */
            if (splash || !dispatchEvent(cur, TINYLCD_EV_LONGPRESS)) screenSet(false);
            else { s_lastActivity = now; lastRefresh = now; }
            break;
        case BTN_CLICK:
            clicks++;
            lastClickAt = now;
            s_lastActivity = now;
            break;
        default:
            break;
        }

        /* Close a click group at three, or when the gap expires. Singles are
         * plain navigation; doubles/triples are offered to the page and fall
         * back to advancing by the click count when declined — so rapid
         * paging on an event-blind page still works. */
        if (clicks > 0 && (clicks >= 3 || now - lastClickAt >= CLICK_GAP_MS)) {
            int n = clicks;
            clicks = 0;
            if (n == 1) {
                s_navDelta.fetch_add(1);
            } else {
                tinylcd_ev_t ev = (n == 2) ? TINYLCD_EV_DOUBLECLICK
                                           : TINYLCD_EV_TRIPLECLICK;
                if (splash || !dispatchEvent(cur, ev)) s_navDelta.fetch_add(n);
                else { s_lastActivity = now; lastRefresh = now; }
            }
        }

        /* Navigation requests (button, API, CLI). Any nav ends the splash,
         * counts as activity, and wakes a dark screen (except mode -2). */
        int delta  = s_navDelta.exchange(0);
        int target = s_navTarget.exchange(TINYLCD_NO_PAGE);
        if ((delta || target != TINYLCD_NO_PAGE) && count > 0) {
            if (splash) { splash = false; cur = 0; }
            if (cur == TINYLCD_NO_PAGE) cur = 0;   /* first show after a dark boot */
            if (target != TINYLCD_NO_PAGE) cur = target;
            cur = ((cur + delta) % count + count) % count;
            s_current.store(cur);
            s_redrawMask.store(0);
            lastRefresh = now;
            s_lastActivity = now;
            if (!s_screenOn && s_standby != -2) screenSet(true);
            dirty = true;
        }

        if (splash && now >= splashUntil) {
            splash = false;
            if (count > 0) { s_current.store(0); cur = 0; lastRefresh = now; }
            dirty = true;
        }
        /* A dark boot never ran the splash phase: land on page 0 for the
         * first wake as soon as pages exist. */
        if (!splash && cur == TINYLCD_NO_PAGE && count > 0) {
            s_current.store(0); cur = 0; lastRefresh = now;
            dirty = true;
        }

        if (!splash && cur != TINYLCD_NO_PAGE) {
            if ((s_redrawMask.exchange(0) & (1u << cur)) && s_screenOn) dirty = true;
            int rm = s_pages[cur].refreshMs;
            if (s_screenOn && rm > 0 && now - lastRefresh >= (uint32_t)rm) {
                lastRefresh = now; dirty = true;
            }
        }

        /* Standby enforcement: the timeout, and mode -2 swatting any wake
         * that slipped through (e.g. the mode changed under a lit screen). */
        if (s_screenOn && s_standby > 0 &&
            now - s_lastActivity >= (uint32_t)s_standby * 1000)
            screenSet(false);
        if (s_standby == -2 && s_screenOn) screenSet(false);

        if (dirty && s_screenOn) render(splash);
    }
}

#endif /* TINYLCD_HAVE_PANEL */

/* =========================================================================
 * 6. Service + CLI
 * ========================================================================= */

static void cliPage(const char* args)
{
    if (cliWantsHelp(args)) {
        cliPrintf("%-*s list OLED pages / switch: page [next|prev|<name|#>]\n",
                  CLI_HELP_COL, "page [target]");
        return;
    }
    int count = s_pageCount.load();
    if (!*args) {
        int cur = s_current.load();
        for (int i = 0; i < count; i++)
            cliPrintf("%c %d %s\n", i == cur ? '*' : ' ', i, s_pages[i].name);
        if (!count) cliPrintf("no pages registered\n");
        return;
    }
    if (!strcmp(args, "next")) { tinylcdPageNext(); return; }
    if (!strcmp(args, "prev")) { tinylcdPagePrev(); return; }
    for (int i = 0; i < count; i++)
        if (!strcmp(args, s_pages[i].name)) { tinylcdPageShow(i); return; }
    char* end;
    long n = strtol(args, &end, 10);
    if (*end == '\0' && n >= 0 && n < count) { tinylcdPageShow((int)n); return; }
    cliPrintf("usage: page [next|prev|<name|#>]\n");
}

void TinylcdService::onInit()
{
#if TINYLCD_HAVE_PANEL
    /* s.tinylcd.* defaults are seeded by the settings: block in straddle.yaml
     * (spangapSettingsGenDefaults runs before this walk) — except the page
     * indicator, whose default is board-dependent (the dots default to the
     * button's side), so its settings row is defaultless and the seed comes
     * from the board-settable Kconfig value here. */
    storageDefault("s.tinylcd.page_indicator", CONFIG_TINYLCD_PAGE_INDICATOR);
    cliRegisterCmd("page", cliPage);
    s_task = spawnTask(tinylcdTaskFn, "tinylcd", 6144, nullptr, 4,
                       CORE_SECONDARY_NO_LCD);
    if (!s_task) err("task spawn failed");
#endif
    /* No pins published (headless build): APIs stay callable no-ops. */
}
