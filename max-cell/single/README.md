# Single-variable plugins

Four independent Max for Live devices, each decoding and normalising **one**
CodeCell channel. They replace the single multi-channel `stickB` device for the
current phase: simpler to reason about, and each can live on its own Ableton
track driving its own sound.

| Plugin | Device | JS file | Firmware suffix | stickB channel |
|---|---|---|---|---|
| Gyro | `gyro.amxd` | `gyro.js` | `_gyro` | 4 |
| Inertia | `inertia.amxd` | `inertia.js` | `_inertia` | 0 |
| Linear | `linear.amxd` | `linear.js` | `_lin` | 10 |
| Compass | `compass.amxd` | `compass.js` | `_comp` | 5 |

Each `.js` is fully self-contained — decode UDP → auto-range → noise floor →
smooth. There is **no `setvar` and no channel table**, so the old "the `.amxd`
overrides the JS suffix" trap cannot happen here.

Each device has **two outlets**:
- **outlet 0** — the smoothed variable, 0–127.
- **outlet 1** — the battery, passed through raw (0–100 %, `101` = charging,
  `102` = USB with no battery). The battery is always sent by the firmware every
  ~5 s regardless of `SENSOR_PROFILE`, so even a one-sensor prop reports it. The
  device shows it as a `Battery: N%` label plus a dot that pulses on each ping —
  your at-a-glance "the board is alive" indicator.

### The Smooth knob

Each device has a **`Smooth`** parameter (a live number box, 0–100, saved with the
Live set and automatable). It controls how organic the curve is, in real time:

- **0** = raw / instant (the value jumps as fast as the sensor).
- **50** (default) = moderate glide.
- **100** = very smooth / slow (big spins and frantic juggling become gentle
  curves instead of sudden jumps).

Under the hood it sends `smoothamount <0..100>` to the JS, which maps it to the
one-pole coefficient. Turn it up while the diabolo spins fast; turn it down when
you want the sound to react sharply.

## ⚠️ The `.amxd` files are auto-generated and UNVERIFIED

They were carved programmatically from `stickB` (one channel's mapping strip and
the battery readout kept, the other ten channel strips + all `setvar` removed,
`js` repointed to the single-variable file with a 2nd outlet for the battery).
The structure was validated (no dangling wires, parameter registry consistent),
**but they have not been opened in Max/Ableton.** Before trusting them:

1. Drop each on a track in Ableton and confirm it loads with **no Max console
   errors**.
2. Confirm the number box moves when the matching prop streams.
3. If any device errors or won't map, fall back to the manual recipe at the bottom
   of this file — carving a duplicate of `stickB` by hand is the safe path.

Known cosmetic caveat: the kept widget row keeps its original position from the
12-row layout, so the device may look sparse or off-centre in presentation mode.
That's layout only, not function — tidy it in Max if it bothers you.

Also: keep each `.amxd` **next to its `.js`** (both live here in `max/single/`),
or **Freeze** the device in Max so the JS embeds — otherwise `js gyro.js` won't
resolve and the device receives nothing.

## Ports: all four ship listening on 9999

The generated devices all use `udpreceive 9999`, matching the current firmware, so
a prop flashed the normal way works immediately — **but load only one at a time**,
because two `udpreceive` on the same port clash (see below). To run several at
once, change each device's `udpreceive` port and set each prop's `UDP_PORT` to
match. Suggested assignment: gyro 9001, inertia 9002, linear 9003, compass 9004.

## Why a different port per plugin

Two Max `udpreceive` objects **cannot share a port** in the same Live set — the
second one fails to bind. So the four devices listen on four different ports
(9001–9004 above), and each physical prop's firmware is told which port to send
to via `#define UDP_PORT` in its `wifi_configs.h`.

Model: **one prop → one variable → one port → one plugin.** A gyro prop sets
`UDP_PORT 9001` and `SENSOR_PROFILE (MOTION_GYRO)`; load the Gyro device and
point it at that prop with `setdevice`.

> **The rule is one `udpreceive` per port, always.** So to run two gyro props at
> once on separate tracks, give the second prop its own port (e.g. `UDP_PORT
> 9011`) and load a second Gyro device whose `udpreceive` is `9011`. Then use
> `setdevice` in each so they filter to their own prop by name.

## Building each `.amxd` (done once, inside Max)

The `.amxd` is a binary Max device and can't be generated from here — build each
one in the Max editor. Fastest path is to duplicate an existing device and swap
two things:

1. **Duplicate** `max/stickB/First_OSC_STICKB_v4.amxd`, rename per variable
   (e.g. `gyro.amxd`) into `max/single/`.
2. Open it in Max (Edit). Delete the old multi-outlet `js` and the whole
   `setvar` column — you don't need them.
3. Add the receiver and JS:
   ```
   [udpreceive 9001 @outputformat rawbytes]  ->  [js gyro.js]  ->  (your mapping)
   ```
   Use the port from the table for that variable.
4. Add the device-name box (reuse the v13 fix so editing the name works):
   ```
   [textedit]  ->  [route text]  ->  [prepend setdevice]  ->  [js gyro.js]
   ```
   `route text` strips the word `text` that `textedit` prepends, so the JS
   receives `setdevice stickA`, not `setdevice text stickA`.
5. Optional smoother glide: add `[metro 20]  ->  [js gyro.js]` and send the
   message `clocked 1`. Then the value eases at a steady 50 fps regardless of how
   bursty the packets are. Leave `clocked 0` (default) and it works without a metro.
6. Save. `autowatch 1` in the JS means later edits to the `.js` reload live — you
   won't need to reopen the device to pick up JS changes.

Repeat for inertia / linear / compass with their JS file and port.

## Tuning (live messages into the `js` object)

Defaults are already set per variable, but to dial in:

- `printrange` — post the learned min/max after juggling a while; tells you the
  real span the auto-range is working with.
- `minspan <n>` — how big the real swing must be to reach full 0–127. Lower =
  more sensitive (and more noise); higher = only big motions reach the top.
- `smoothing <0..1>` — glide amount. Lower = smoother/slower, higher = snappier.
- `resetrange` — forget the learned range (after moving venue, or a bad spike).
- `autorange 0` — bypass auto-range and just smooth the raw firmware value.

## Firmware side (per prop)

In that prop's `wifi_configs.h` (git-ignored, per machine):

```c
#define DEVICE_NAME    "stickA"
#define SENSOR_PROFILE (MOTION_GYRO)   // only this sensor is powered -> cooler board
#define UDP_PORT       9001            // matches the Gyro plugin's udpreceive
```

Both `SENSOR_PROFILE` and `UDP_PORT` are optional and fall back to the old
behaviour (all sensors, port 9999) when omitted, so existing props keep working.
