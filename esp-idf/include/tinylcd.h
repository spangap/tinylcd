/**
 * tinylcd.h — paged status UI for tiny mono OLEDs (SSD1306 / SH1106, 128×64).
 *
 * The platform's on-device UI is colour-TFT LVGL (spangap-lcd). tinylcd is the
 * other end of the scale: a 1-bpp u8g2 framebuffer, a flat ring of full-screen
 * pages, one button. Boards whose only screen is a small I2C OLED pull this in
 * (additional_installs) and publish their pins via CONFIG_TINYLCD_* in a
 * when:-gated kconfig group; straddles contribute pages from a
 * conditional/tinylcd/ slice so a headless or TFT build never compiles them.
 *
 * Model:
 *   - A page is a name plus a draw callback. Registration returns a handle.
 *   - tinylcd owns the display task; every draw callback runs on it, handed the
 *     live u8g2_t* with the buffer already cleared. Draw, return; tinylcd
 *     overlays the page-indicator dots and sends the buffer.
 *   - tinylcdDraw(handle) requests a redraw and is a no-op unless that page is
 *     the one on screen — so a page subscribes to the storage vars it renders
 *     and its change callback is one line: tinylcdDraw(myPage).
 *   - Subscriptions must live on the tinylcd task (storage dispatches change
 *     callbacks on the subscribing task): wrap them in tinylcdRun(). Calls made
 *     before the task is up are queued and run at task start, so init-band
 *     ordering against tinylcd never matters.
 *   - At boot the display shows CONFIG_SPANGAP_FW_NAME big and centered; pages
 *     take over CONFIG_TINYLCD_SPLASH_MS later. Nothing blocks anywhere.
 *
 * The u8g2 draw API and the vendored font roster (see
 * components/u8g2/csrc/u8g2_fonts_selected.c) come with this header.
 */
#pragma once

#include "sdkconfig.h"
#include "service.h"
#include "u8g2.h"

/** Page handle: >= 0 from tinylcdAddPage, TINYLCD_NO_PAGE when invalid/full. */
typedef int tinylcd_page_t;
#define TINYLCD_NO_PAGE (-1)

/** Input events delivered to the current page through its draw callback. */
typedef enum {
    TINYLCD_EV_NONE = 0,     /* ordinary redraw — return value ignored */
    TINYLCD_EV_LONGPRESS,    /* button held >= 500 ms */
    TINYLCD_EV_DOUBLECLICK,
    TINYLCD_EV_TRIPLECLICK,
} tinylcd_ev_t;

/* ── Page layout: a title line, then body lines ─────────────────────────────
 *
 * Every page draws the same shape — one line naming what is on screen, then
 * the readout under it — and the two constants below are where that shape is
 * stated, so a page places its title by name rather than by a number it picked.
 *
 * The numbers are chosen for the panels that are NOT one colour. The common
 * 128x64 OLED comes in a two-tone glass whose top **16 rows are yellow** and
 * whose remaining 48 are blue, with a physical dead gap along the seam. That is
 * fixed geometry, not a variant to configure: keep the title above row 16 and
 * the body below it and the seam lands in the space between them, which reads
 * as a title bar on that glass and as ordinary spacing on a uniform one. A page
 * that splits the difference gets its title sliced lengthways, gap and all.
 *
 * So: title baseline at TINYLCD_TITLE_Y, and no body line's baseline ABOVE
 * TINYLCD_BODY_Y (lower is free — a page with three body lines rather than four
 * may start further down). With the dots on top the title has rows 6..15 to
 * live in, which fits a 13-pixel font's capitals and nothing larger.
 */
#define TINYLCD_TITLE_Y  15   /* baseline of the title line (yellow band, above the seam) */
#define TINYLCD_BODY_Y   25   /* highest baseline a body line may use (below the seam) */

/** Draw callback: render the whole page into `g` (buffer pre-cleared, sent by
 *  tinylcd after return). Runs on the tinylcd task only. The dot indicator
 *  overlays the top (rows 0..4) or bottom (rows 59..63) strip depending on
 *  s.tinylcd.page_indicator — keep both strips plus a small gap clear; rows
 *  8..55 are safely the page's, and the title line above reaches row 6.
 *
 *  `ev` is the input event this draw responds to — TINYLCD_EV_NONE for
 *  ordinary redraws. Return true to absorb the event: what the callback just
 *  drew is shown as the response. Return false to decline it and get the
 *  default instead (the drawing is discarded): LONGPRESS puts the screen to
 *  sleep, DOUBLE/TRIPLECLICK advance two/three pages. A page that only ever
 *  draws returns false unconditionally. */
typedef bool (*tinylcd_draw_cb_t)(tinylcd_page_t page, u8g2_t* g, tinylcd_ev_t ev);

/** Callback run on the tinylcd task via tinylcdRun (mirrors lcdRun/ON_LCD). */
typedef void (*tinylcd_fn_t)(void* arg);
#define ON_TINYLCD [](void* arg)

/** Register a page. `name` is a short static string (CLI + logs). refreshMs
 *  > 0 redraws the page on that period while it is on screen — for data with
 *  no change signal to subscribe to; 0 = event-driven only. Callable from any
 *  task, any time (before tinylcd's own init included). Returns the handle,
 *  or TINYLCD_NO_PAGE when the page table is full. */
tinylcd_page_t tinylcdAddPage(const char* name, tinylcd_draw_cb_t cb,
                              int refreshMs = 0);

/** Step to the next / previous page (wraps). Safe from any task. */
void tinylcdPageNext(void);
void tinylcdPagePrev(void);

/** Bring the given page on screen. Safe from any task. */
void tinylcdPageShow(tinylcd_page_t page);

/** True when `page` is the page currently on screen. */
bool tinylcdPageIsCurrent(tinylcd_page_t page);

/** Request a redraw of `page`. No-op unless it is on screen — so change
 *  callbacks call this unconditionally. Safe from any task. */
void tinylcdDraw(tinylcd_page_t page);

/** Run fn(arg) on the tinylcd task — where storage subscriptions whose
 *  callbacks drive pages must be registered. Queued if the task is not up
 *  yet; runs shortly after in registration order. Safe from any task. */
void tinylcdRun(tinylcd_fn_t fn, void* arg = nullptr);

/**
 * Boot service. onInit seeds s.tinylcd.* defaults, registers the `page` CLI
 * verb and spawns the display task (which owns the I2C bus on
 * CONFIG_TINYLCD_SDA/SCL_PIN, the u8g2 init, the splash and the button). With
 * the pins unset (-1, no board values published) everything above degrades to
 * cheap no-ops and no task is spawned.
 */
class TinylcdService : public Service {
public:
    void onInit() override;
};
