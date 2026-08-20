# tinylcd — paged status UI for tiny mono OLEDs

**tinylcd** drives the small 128×64 I2C OLEDs (SSD1306 / SH1106) that LoRa
boards carry, as a flat ring of full-screen **status pages** with a dot
indicator and a one-button page-advance. It is the counterpart to
[spangap-lcd](../spangap-lcd) at the other end of the display scale: where that
is colour-TFT LVGL with apps, navigation and a shell, this is a 1 KB 1-bpp
framebuffer, the vendored [u8g2](https://github.com/olikraus/u8g2) draw API,
and nothing else.

A board whose only screen is such an OLED stages tinylcd via
`additional_installs:` and publishes its pins as `CONFIG_TINYLCD_*` values in a
`when: spangap/tinylcd`-gated `kconfig:` group. Other straddles contribute
pages from an `esp-idf/conditional/tinylcd/` slice — a headless or TFT build
never compiles them. With no pins published, every API below degrades to a
cheap no-op and no task is spawned: staging tinylcd is always safe.

## The page model

A page is a name plus a draw callback. Registration returns a handle:

```cpp
#include "tinylcd.h"

static tinylcd_page_t s_page = TINYLCD_NO_PAGE;

static bool draw(tinylcd_page_t page, u8g2_t* g, tinylcd_ev_t ev) {
    u8g2_SetFont(g, u8g2_font_7x13B_tr);
    u8g2_DrawStr(g, 0, TINYLCD_TITLE_Y, "my page");
    u8g2_SetFont(g, u8g2_font_6x10_tf);
    u8g2_DrawStr(g, 0, TINYLCD_BODY_Y, "something to report");
    return false;   /* draw-only page: no events handled */
}

// in the straddle's init hook:
s_page = tinylcdAddPage("mypage", draw);
tinylcdRun(ON_TINYLCD {
    storageSubscribeChanges("some.live.key", ON_CHANGE { tinylcdDraw(s_page); });
});
```

- Every draw callback runs on the **tinylcd task**, handed the live `u8g2_t*`
  with the buffer already cleared; tinylcd overlays the page-indicator dots
  and sends the buffer. The dots sit on the top (rows 0–4) or bottom (rows
  59–63) strip per `s.tinylcd.page_indicator`, so keep both strips plus a
  small gap clear — rows 8–55 are safely the page's. Draw the full page each
  call — there is no partial-update contract.
- **Place the title with `TINYLCD_TITLE_Y` and keep body lines at or below
  `TINYLCD_BODY_Y`.** Half the 128×64 panels in circulation are two-tone glass:
  the top **16 rows are yellow**, the other 48 blue, with a physical dead gap
  along the seam. It is fixed geometry, not something to configure — a title
  above row 16 and a body below it puts the seam in the space between them,
  which reads as a title bar there and as ordinary spacing on a uniform panel,
  while a line that straddles row 16 is sliced lengthways. With the dots on top
  the title has rows 6–15, which fits a 13-pixel font's capitals and nothing
  taller. A page that is a ladder of equal lines rather than title-and-body
  places its own baselines, subject to the same rule: no line across row 16.
- `tinylcdDraw(handle)` requests a redraw and is a **no-op unless that page is
  on screen** — so a page subscribes to the storage vars it renders and its
  change callback is the one line above, fired as often as the data likes.
- Storage change callbacks dispatch on the subscribing task, so subscriptions
  that drive pages must be made **on** the tinylcd task: wrap them in
  `tinylcdRun()`. Calls made before the task exists are queued and run at task
  start — init-band ordering against tinylcd never matters.
- For data with no change signal (in-memory peer tables etc.) pass a
  `refreshMs` to `tinylcdAddPage`: the page redraws on that period **while on
  screen**, and never otherwise.
- Navigation: `tinylcdPageNext()` / `tinylcdPagePrev()` (wrap around),
  `tinylcdPageShow(handle)`, `tinylcdPageIsCurrent(handle)` — all safe from
  any task.
- **Input events.** The `ev` argument is `TINYLCD_EV_NONE` on ordinary
  redraws; button gestures on the page's watch are offered to it first as
  `TINYLCD_EV_LONGPRESS` / `DOUBLECLICK` / `TRIPLECLICK`. Returning **true
  absorbs the event** — what the callback just drew is shown as the response.
  Returning false gets the default: long press puts the screen to sleep,
  double/triple click advance two/three pages (so rapid paging on an
  event-blind page still works). A page holding a special mode it entered via
  an absorbed event should subscribe to `tinylcd.active` and revert to its
  default rendering when the value drops to 0 — standby ends the session and
  resets the ring to the first page.

At boot the panel shows `CONFIG_SPANGAP_FW_NAME` big and centered; the first
registered page takes over `CONFIG_TINYLCD_SPLASH_MS` (default 3000) later, or
on the first button click / navigation call. Nothing blocks anywhere: the task
is a single `itsPoll()` deadline loop.

## Hardware knobs (set by the board)

| Key | Meaning |
|---|---|
| `CONFIG_TINYLCD_SDA_PIN` / `SCL_PIN` | OLED I2C pins; either -1 = no panel, all no-ops |
| `CONFIG_TINYLCD_RST_PIN` | panel reset GPIO, -1 = none |
| `CONFIG_TINYLCD_I2C_ADDR` | 7-bit address, default 0x3C |
| `CONFIG_TINYLCD_SSD1306` / `SH1106` | controller choice (0.96" vs 1.3" panels) |
| `CONFIG_TINYLCD_BUTTON_PIN` | active-low page-advance button (usually BOOT/GPIO 0), -1 = none |
| `CONFIG_TINYLCD_SPLASH_MS` | splash duration |
| `CONFIG_TINYLCD_PAGE_INDICATOR` | seed for `s.tinylcd.page_indicator` (default 1) — boards set it so the dots default to the button's side |

tinylcd owns the I2C bus on those pins (created with the `i2c_master` driver,
any free controller). A board with other devices on the same physical bus
cannot use tinylcd as-is — see [INTERNALS.md](INTERNALS.md).

A click is a 30–500 ms press (next page; clicks under 350 ms apart group into
double/triple events for the page); holding past 500 ms puts the screen to
sleep unless the page absorbs the long press; any press while the screen is
dark wakes it (and is absorbed).

## Fonts

u8g2 is vendored **without** upstream's 38 MB font archive; the shipped roster
lives in `esp-idf/components/u8g2/csrc/u8g2_fonts_selected.c` (extraction
recipe in its header):

`u8g2_font_4x6_tr` · `u8g2_font_5x8_tr` · `u8g2_font_6x10_tf` ·
`u8g2_font_7x13B_tr` · `u8g2_font_logisoso16_tr` · `u8g2_font_logisoso20_tr` ·
`u8g2_font_logisoso24_tn`

Referencing an unvendored font fails at link time, not runtime.

## Storage variables

Both `s.tinylcd.*` keys are editable under **Settings / TinyLCD** (declarative
pane, web + LCD) and are seeded from that block.

| Key | Meaning |
|---|---|
| `s.tinylcd.orientation` | 0/1 — the two landscape orientations (hardware 180°, live) |
| `s.tinylcd.page_indicator` | where the page dots sit, live: **0** top, **1** bottom, **2** top in orientation 0 / bottom in 1, **3** opposite of 2 — modes 2/3 let the dots track the side the button is on. Default comes from the board (`CONFIG_TINYLCD_PAGE_INDICATOR`, 1 when unset) |
| `s.tinylcd.standby` | screen power policy, live. **>0** (default 60): panel sleep after that many seconds without user activity. **0**: always on. **-1**: boots dark, button wakes. **-2**: panel parked asleep at init, never used |
| `tinylcd.active` | ephemeral, two-way: tinylcd publishes 1/0 on wake/standby (the revert signal for pages holding a special mode), and writing it from outside sleeps or wakes the screen immediately (a wake in mode -2 is refused and written back to 0) |

"Asleep" is the controller's sleep mode (display + charge pump off, ~µA —
its lowest state short of cutting the rail, which is board-owned). User
activity = button presses and navigation calls (`page` CLI included); data
redraws deliberately don't count, so a chatty page can't keep the screen lit.
A press while dark wakes the screen and is absorbed; nav/`tinylcdPageShow`
calls wake it too (never in mode -2).

## CLI

`page` lists pages (current starred); `page next|prev|<name|#>` switches.

## Dependencies

- [spangap-core](../spangap-core) — base runtime (storage, log, CLI, ITS).

## Read next

- [INTERNALS.md](INTERNALS.md) — task/threading model, the panel transfer
  sequence and its failure repair, the u8g2 vendoring trim, and the I2C
  bus-ownership caveat.
