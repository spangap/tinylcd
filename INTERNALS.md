# tinylcd internals

Design notes for maintainers. The API contract lives in
`esp-idf/include/tinylcd.h`; user-facing behaviour in [README.md](README.md).

## Threading model

One task ("tinylcd", prio 4, `CORE_SECONDARY_NO_LCD`) owns everything
display-side: the I2C bus, the u8g2 state, every draw callback. The task's
main loop is a single `itsPoll()` wait whose timeout is the nearest of splash
end / held-button sampling / the current page's `refreshMs` — `portMAX_DELAY`
when none applies, so an idle display costs nothing.

Cross-task requests never touch u8g2. They are three atomics consumed once per
wake — `s_navDelta` (accumulating ±steps), `s_navTarget` (latest-wins absolute
page), `s_redrawMask` (per-page bits) — plus a plain `xTaskNotifyGive` wake,
which `itsPoll` treats as a spurious-but-harmless wake. The redraw mask is
checked against the current page at render time; that check, not the request
site, is what makes `tinylcdDraw(h)` a no-op for an off-screen page. The mask
is cleared on page switch so stale requests can't ghost-redraw.

`tinylcdRun` marshals arbitrary callbacks onto the task via an ITS aux message
(port 1, an 8-byte `{fn, arg}` payload). Before the task exists the pair is
parked in a fixed 16-slot early queue drained at task start — that queue is
what frees callers (e.g. a platform-band net hook) from ordering against
tinylcd's straddle-band init. Page registration itself is just a
spinlock-guarded append to a static table, callable from anywhere at any time.

Storage subscriptions dispatch on the subscribing task (via its `itsPoll`), so
any subscription whose callback drives a page must be registered from a
`tinylcdRun` callback — never from a consumer's `onInit` (the main task dies
after boot, taking its subscriptions with it).

## Button

The trigger is **LOW_LEVEL, not edge**, because the button must work through
automatic light sleep: `CONFIG_PM_SLP_DISABLE_GPIO` isolates every pin on
sleep entry and GPIO wake only speaks level triggers, so an edge-typed ISR
simply never fires on an idle, sleeping node (the symptom: display fine,
button dead). `pmGpioWakeEnable(pin, GPIO_INTR_LOW_LEVEL)` arms the wake,
sets the interrupt type and exempts the pin from sleep isolation — the same
arrangement as the T-Deck's centre button, minus its hold tiers.

Level semantics bring the storm problem: the ISR re-fires as long as the
finger is down, so it silences the pin on first fire (`gpio_ll_intr_disable`,
an LL register write — the driver call takes a non-ISR spinlock) and notifies
the task. The task samples the level on every wake and runs the press state
machine: 30–500 ms release = click; crossing 500 ms held fires the long
press once (threshold-timed by the 50 ms held sampling, not the release); a
press that starts on a dark screen wakes it and is absorbed whole — it must
not also page-flip or, held long, re-sleep the screen it just woke. Release
(or an idle high sample — a press too short to be seen must not leave the
pin silenced for good) re-arms the interrupt. A `pmOnLightSleepWake`
backstop re-checks the line on each GPIO wake because a bouncing press can
land the sleep-exit window in a bounce-high gap. There is no polling while
idle. The ISR service is installed via
`spiHelperEnsureGpioIsr(ESP_INTR_FLAG_IRAM)`, shared with LoRa's DIO1.

## Input events

Clicks less than 350 ms apart group into one gesture; the group closes at
three or when the gap expires, so a single click pays up to 350 ms of latency
— the price of distinguishing it from a double. Gestures are offered to the
current page through its draw callback (`dispatchEvent`: cleared buffer, call
with the event, and an absorbing return means what the page drew IS the
response — dots overlaid, buffer sent). Declined gestures get the defaults:
long press → standby, double/triple → advance by the click count, which
preserves rapid paging on event-blind pages. Declined drawing is simply not
sent — the panel still shows the previous frame, so no repaint is needed.
Events are never delivered during the splash or to a dark screen.

## Standby

`s.tinylcd.standby` (README has the value semantics) is applied entirely by
the task loop; the storage handler just records the mode and re-stamps the
activity clock so a lowered timeout counts from the change. "Off" is
`SetPowerSave(1)` — the controller's sleep mode, the panel's floor without
cutting its rail (rail power is board-owned; if a board ever wants the rail
gated too, that's a board hook, not tinylcd's). Activity = button presses
and navigation (CLI included); `tinylcdDraw` requests and periodic refresh
deliberately don't count and are skipped while dark, so a chatty page
neither keeps the screen lit nor wakes the task for nothing — a dark screen
leaves the loop with no deadlines at all (`portMAX_DELAY`). Dark boots
(-1/-2) skip the splash phase entirely and land on page 0 at first wake;
mode -2 is enforced in the loop (any wake that slips through is swatted), so
a live mode change from -2 needs no reboot.

Entering standby publishes `tinylcd.active = 0` (ephemeral — unprefixed keys
don't persist) and resets the ring to the first page; waking publishes 1.
The key is the revert signal for a page holding a special mode entered via
an absorbed event: its state machine can't otherwise tell that the session
ended, because standby is deliberately not delivered as a draw event — the
screen is off, there is nothing to draw.

The key is two-way: tinylcd also subscribes to it, so an external write
sleeps or wakes the screen immediately (a wake in mode -2 is refused and
written back to 0). Its own transition writes echo into that same handler;
`screenSet`'s same-state no-op is what keeps the publish→notify→publish
cycle from looping.

## Panel transfers

One `u8g2_SendBuffer` is 8 tile rows; per tile row, MCU → panel is:

```
[0x00 0x40]                     set display start line
[0x00 0x1x 0x0x 0xbY]           column high/low nibble + page address Y
[0x40 <24 data bytes>]  × 5
[0x40 <8 data bytes>]
```

Eight I2C transactions per row, 64 per frame, each a complete START…STOP that
tinylcd assembles in `s_i2cBuf` and issues as one `i2c_master_transmit` (the
bus is created with `trans_queue_depth` 0, so every transmit is synchronous
and fully reaped before it returns; the driver holds a PM lock across it, so
light sleep cannot land inside one).

u8g2 discards the return value of every transaction at all three of its
layers, so a failure is invisible to it and it never retries. That matters
because the address transaction and the data that follows it are separate
transactions against a stateful pointer: lose the address and the row's data
lands wherever the pointer already was — one 8-pixel band stale, another
overwritten — and lose a data chunk and the pointer is short for the rest of
the row. Neither can be repaired by re-sending that transaction alone.

So `tinylcdByteCb` records a failure in `s_i2cFault` and `sendBuffer` is what
acts on it: it re-sends the entire frame, which re-issues every address
command and lands the panel in a known state regardless of where its pointer
was left. Attempts are bounded by `TINYLCD_SEND_ATTEMPTS`, with an
`i2c_master_bus_reset` between them to clock out a slave holding SDA low, no
delay in the loop, and one log line per run of failures rather than per
redraw. A clean frame costs one flag test.

This covers transfers the controller reports as failed. A bit error the slave
still ACKs is undetectable from the master, and repaints are demand-driven
(see the render policy in the task loop) — a page with no `refreshMs` and no
change signal holds whatever is on the panel indefinitely. A page whose
correctness matters more than an idle display's zero cost buys its own repair
cadence with `refreshMs`; tinylcd does not repaint on a timer of its own.

What makes a transaction fail in the first place is usually the lines rather
than the code. `CONFIG_TINYLCD_I2C_HZ` defaults to 400 kHz, which needs pull-ups
of a few kOhm to meet the rise-time budget; the SoC's internal ones are around
45 kOhm and are enough only at 100 kHz on short traces. A panel module with its
own 4.7 kOhm resistors is fine, and one without them is marginal at 400 kHz in
exactly the intermittent way the frame re-send exists to paper over. The two
levers are `CONFIG_TINYLCD_I2C_HZ` and `CONFIG_SPANGAP_I2C_INTERNAL_PULLUP` —
the latter lives in spangap-core because the pull-ups are a property of the
board's wiring, shared by every chip on the bus, not of the panel.

The transmit timeout argument is milliseconds, not ticks.

## u8g2 vendoring

`esp-idf/components/u8g2/` is upstream `csrc/` verbatim at the commit noted in
`u8g2_fonts_selected.c`, minus two trims:

- **`u8g2_fonts.c` (38 MB) → `u8g2_fonts_selected.c` (~25 KB).** The monolith
  is one array per font with zero cross-references, so fonts extract verbatim;
  the selected file carries the roster and the awk recipe. Every font in
  `u8g2.h` is declared `extern`, so an unvendored font is a link error — never
  a silent runtime blank.
- **MUI dropped** (`mui*.c/h`, u8g2's built-in menu framework) — tinylcd has
  its own page model.

The component compiles `-w` (upstream does not survive the project's
`-Werror`; component-PRIVATE flags win over IDF's forced flags, same
arrangement as rns' bzip2). It is REQUIRES-public from tinylcd because
`tinylcd.h` hands `u8g2_t*` to consumers. u8g2's full-framebuffer variant
(`*_f`) keeps the 1 KB buffer in BSS; there is no per-frame allocation.

## I2C bus ownership

The task creates its own `i2c_master` bus on `CONFIG_TINYLCD_SDA/SCL_PIN`
(port -1 = any free controller). That is correct for the current boards, where
the OLED is alone on its header pins — but `i2c_new_master_bus` on an
already-claimed port is a conflict, so a future board sharing the OLED bus
with other devices (RTC, IMU) needs the T-Deck arrangement instead: the board
owns the bus and exposes an accessor, and tinylcd grows a way to accept a
handle. Don't paper over it with a second bus on the same pins.

## Failure containment

Panel init failure (absent panel, wrong pins) logs one error and ends the
task; registrations and API calls remain valid no-ops. The `-1`-pins build
compiles out everything past the registry — staging tinylcd can never break a
headless build.

## Rejected alternatives

- **LVGL on the mono OLED** (spangap-lcd with a 1-bpp theme): ~100× the RAM
  and flash for a screen that shows six lines of text; and spangap-lcd's app
  model (launcher, navigation, input) has no meaning behind one button.
- **u8g2 page-buffer mode** (128 B instead of 1 KB): re-runs every draw
  callback 8× per frame to save RAM these boards aren't short of.
- **Per-page storage-key lists in the registry** (tinylcd subscribing on the
  page's behalf): saves each consumer two lines but bakes storage semantics
  into the page model; `tinylcdRun` keeps the registry dumb and lets pages
  subscribe to anything, not just storage.
