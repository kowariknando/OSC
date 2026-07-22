# OSC — Kinetic Art Installation

## What this project is

An art installation where physical **pendulums** (sticks / diabolos) carry
**CodeCell C6** boards. The boards read their own motion and stream it over Wi-Fi
to a computer running **Ableton Live**, where a **Max for Live** device turns the
movement into sound.

Signal chain:

```
Pendulum + CodeCell C6  →  Wi-Fi (UDP)  →  Max for Live device  →  Ableton Live
                                                                        ↓
                                                       (planned) MIDI → hardware synths
```

**Current goal:** get the Max for Live device working reliably with one device
(`stickB`). MIDI output to external synths is a later phase — do not build it yet
unless asked.

## Working conventions

- **English only.** All code, comments, commit messages, variable names,
  documentation, and conversation. Parts of the repo still contain Spanish and
  Catalan text from earlier versions — translate it whenever you touch it.
- **Cross-platform.** One developer works on **Windows**, one on **macOS**.
  Never commit absolute paths, OS-specific scripts, or editor-local settings.
- The author is new to Claude Code and to Max/MSP JavaScript. Explain the
  reasoning behind changes, don't just apply them.

## Repository layout

| Path | What it is |
|---|---|
| `src/prop_firmware/main.cpp` | The **only** firmware. Same code for every device. |
| `include/wifi_configs.h` | Per-device secrets + name. **Git-ignored** — see below. |
| `platformio.ini` | PlatformIO build config, ESP32-C6 target. |
| `max/stickB/First_OSC_STICKB_v4.amxd` | The Max for Live device (binary). |
| `max/stickB/codecell_receive2.js` | JS parser inside the `.amxd` that decodes UDP (all channels). |
| `max/single/*.js` | Single-variable plugins (gyro/inertia/linear/compass). See `max/single/README.md`. |
| `sdkconfig.esp32-c6-devkitc-1` | Generated ESP-IDF config. Don't hand-edit. |

One firmware serves all physical devices. To add a pendulum you change
`DEVICE_NAME` in `wifi_configs.h` and reflash — **never** fork `main.cpp`.

## Setup for a new machine

### `wifi_configs.h` must never be committed

`include/wifi_configs.h` holds a real Wi-Fi password and is git-ignored **on
purpose**. This is a deliberate decision, not an oversight:

- Never add it to git, never remove its line from `.gitignore`, and never
  `git add -f` it.
- Never paste its real contents into commit messages, issues, or documentation —
  including this file. Use placeholders.
- If it is ever committed by accident, the password must be treated as leaked and
  changed on the router. Deleting the file in a later commit does **not** remove
  it from git history.

Each developer keeps their own local copy. It is expected that a fresh clone
**will not compile** until you create it by hand:

```c
#define DEVICE_NAME "stickB"          // unique per physical pendulum
#define WIFI_NAME   "your_network"
#define WIFI_PASS   "your_password"
#define WIFI_IP     "192.168.1.42"    // IP of the computer running Ableton

// Optional, per-device. Omit both to keep the old behaviour (all sensors, 9999).
#define SENSOR_PROFILE (MOTION_GYRO)  // only power the sensors this prop uses
#define UDP_PORT       9001           // must match the plugin's udpreceive port
```

`WIFI_IP` is the **receiving computer's** IP and differs per machine and per
network. Both developers need their own value. Re-check it after joining a new
Wi-Fi network — a wrong IP fails silently (the board connects, the data goes
nowhere).

`SENSOR_PROFILE` and `UDP_PORT` are **optional per-device tuning** (added for the
single-variable plugins — see below). `SENSOR_PROFILE` is a sum of CodeCell mode
flags (`MOTION_GYRO`, `MOTION_LINEAR_ACC`, `LIGHT`, …); the board only powers and
streams those, so fewer flags means a **cooler board and less Wi-Fi traffic**.
`UDP_PORT` is which port this prop streams to. Both fall back to the old defaults
(the full sensor set, port 9999) when not defined, so omitting them is safe.

## Build and flash

PlatformIO, environment `codecell_c6`:

```bash
pio run                  # build
pio run --target upload  # flash over USB-C
pio device monitor       # serial, 115200 baud
```

Serial prints the Wi-Fi connection progress, the device name, and a battery
percentage every 5 seconds. If nothing arrives in Max, check serial first — it
tells you whether the board is even on the network.

## The UDP protocol (the contract between firmware and Max)

The firmware sends **plain-text UDP**, not real OSC, despite the project name.
Format, one message per packet:

```
<DEVICE_NAME>_<suffix> <value>;
```

Example: `stickB_gyro 64;`

Port **9999**, received in Max by `udpreceive 9999 @outputformat rawbytes`, then
parsed by `codecell_receive2.js`.

Messages are **sent only when the value changes**, to keep traffic low. A silent
channel means "unchanged", not "broken".

### Channels

Outlet numbers refer to `codecell_receive2.js`. Most values are scaled to
**0–127** so they map directly onto MIDI ranges later.

| Outlet | Suffix | Source | Range |
|---|---|---|---|
| 0 | `_inertia` | Accelerometer magnitude, smoothed | 0–127 |
| 1 | `_prox` | Proximity sensor | 0–127 |
| 2 | `_state` | Motion state | 1=table, 2=stationary, 3=stable, 4=motion |
| 3 | `_act` | Activity classifier | 1–8 (see CodeCell docs) |
| 4 | `_gyro` | Gyroscope magnitude | 0–127 |
| 5 | `_comp` | Magnetometer magnitude | 0–127 |
| 6 | `_rot` | Yaw, −180…180 remapped | 0–127 |
| 7 | `_steps` | Step counter | raw count |
| 8 | `_light` | Ambient light | 0–127 |
| 9 | `_grav` | Gravity vector magnitude | 0–127 |
| 10 | `_lin` | Linear acceleration (gravity removed) | 0–127 |
| 11 | `_bat` | Battery, every 5 s | 0–100, 101=charging, 102=USB only |

Two further channels carry sleep state rather than sensor data:

| Outlet | Suffix | Meaning | Range |
|---|---|---|---|
| 12 | `_sleep` | 1 just before sleeping, 0 on boot/wake | 0 or 1 |
| 13 | `_timeout` | Current idle timeout, echoed on boot and on change | minutes, 0=off |

Because waking is a full reboot, `_sleep 0` on boot doubles as "I am awake
again" — that is what stops a sleep indicator in Max from latching on forever.

**Changing a suffix breaks the Max device silently.** The firmware builds these
tags in `setup()`; the JS rebuilds the same strings in `updateAllTags()`. Both
sides must be edited together.

### Which channels actually matter for pendulums

For swinging motion the useful signals are `_lin` (net movement without
gravity), `_gyro` (rotation speed), `_rot` (orientation) and `_inertia`. Those
four respond directly and predictably to how the pendulum is moving, so they are
the ones to map to sound.

`_steps` and `_act` are different. The BNO085 calculates them with algorithms
designed to recognise a *human body* walking, running or cycling. A swinging
pendulum is not a walking person, so these two outputs will produce numbers that
look active but do not correspond to anything real — a pendulum might register
"steps" or report "cycling" for no meaningful reason.

There is no harm in leaving them connected in the Max device; they cost nothing
and are occasionally interesting to watch. The point is simply: **do not build
musical behaviour on top of them.** If a synth parameter is driven by `_act`, it
will change unpredictably and you will not be able to tell why.

## Sleep and power saving

Rehearsals do not need every pendulum awake the whole time. The firmware
deep-sleeps a board after a period with no movement, and Ableton can also put
boards to sleep on demand.

**Waking is always physical: tap the pendulum.** Deep sleep powers the Wi-Fi
radio down, so no network message can reach a sleeping board — this is a
hardware fact, not a missing feature. The BNO085 stays awake on its own tiny
power budget watching for a tap, and pulls the ESP32 back up when it feels one.
Waking is a full reboot, so expect Wi-Fi to reconnect in a few seconds and the
IMU to need its usual 10–30 s to settle.

Auto-sleep is **skipped while the board is on USB** (battery reads 101 or 102),
so it never sleeps out from under you while flashing or debugging.

### Commands from Max to the boards

The firmware listens on the same port it sends on — **`UDP_PORT`**, which is
`9999` unless a prop overrides it (see the single-variable plugins). Prefix
`all_` targets every board, `<DEVICE_NAME>_` targets one:

| Command | Effect |
|---|---|
| `all_sleep 1` | Every board deep-sleeps now |
| `stickB_sleep 1` | Only that board sleeps |
| `all_timeout <minutes>` | Set the idle timeout, stored in flash |
| `all_timeout 0` | Disable auto-sleep — use this during a performance |

Send them from Max with `udpsend 255.255.255.255 9999` (broadcast, so no board
IPs are needed). If broadcast is blocked on the network, send to each board's IP
instead.

**Port caveat with single-variable props.** A command only reaches a board
listening on the port you send to. Props on the default `9999` (like `stickB`)
are covered by the line above. A prop flashed with `UDP_PORT 9001` only hears
sleep commands sent to **9001**, so add one `udpsend 255.255.255.255 9001` per
port in use (or set every prop back to 9999 during rehearsals, when there is no
port clash to avoid). Sending to a board's own IP works too.

Max's own `udpreceive` on any of these ports also hears the broadcasts. They are
harmless: the tag matches no channel, so the JS drops them.

The timeout lives in **NVS flash** (`Preferences`, namespace `osc`, key
`timeout`), so it survives sleep, reboots and reflashing. That is the whole
point: set it once from Ableton, never reflash to change it.
`DEFAULT_IDLE_TIMEOUT_MINUTES` in `main.cpp` is only the value used on a board
that has never been told otherwise.

### Wire format, both directions

Outgoing data is plain text. Incoming commands are parsed leniently and accept
**both** plain text (`all_sleep 1;`) and **OSC** (`/all_sleep` + `,i` + int32),
because Max's `udpsend` emits OSC framing rather than raw text while `netcat`
and similar tools send plain text. Testing a command by hand from a terminal and
from Max therefore exercises two different code paths — both are supported on
purpose, so don't "simplify" one away.

## Known issues

**Two channels are dead** — the commit that translated the firmware to English
did not translate the JS, so the tag strings no longer match:

| Firmware sends | JS listens for | Result |
|---|---|---|
| `stickB_inertia` | `stickB_inercia` | outlet 0 never fires |
| `stickB_light` | `stickB_llum` | outlet 8 never fires |

The JS side has been fixed. **The `.amxd` side has not**, and it is the one that
actually decides: the patch sends `setvar 0 inercia` and `setvar 8 llum` to the
JS object on load, which overwrites the JS defaults. Until those two message
boxes are edited inside Max, outlets 0 and 8 stay dead.

To finish the fix, open the device in Max and change:
- the message box `setvar 0 inercia` → `setvar 0 inertia`
- the message box `setvar 8 llum` → `setvar 8 light`

This is a general trap: **`setvar` messages in the `.amxd` override the defaults
in the JS.** Renaming a channel means editing both, and the `.amxd` wins.

**Outlets 12 and 13 are not wired yet.** `codecell_receive2.js` now declares 14
outlets, but the `.amxd` still has to be opened in Max to connect the two new
ones to a sleep indicator and a timeout display. Until then the sleep state
simply isn't visible in Ableton — the firmware and JS sides are done.

Adding outlets at the *end* does not disturb outlets 0–11, so existing patch
cords survive. No `setvar` is needed for 12 and 13 either: the patch never sends
one for those channels, so the JS defaults (`sleep`, `timeout`) stand.

Also outstanding:
- `platformio.ini` has a Spanish comment on the `ARDUINO_ESP32C6_DEV` flag.
- `.DS_Store` files are committed and should be removed and git-ignored.

## Editing the Max for Live device

`.amxd` is a **binary file** — it cannot be merged by git and cannot be edited as
text. Do not attempt to patch it with search-and-replace.

Rule for collaboration: **only one person edits the `.amxd` at a time.** Say so
before you start. Logic that can live in `codecell_receive2.js` instead should
live there, because that file *is* diffable and mergeable.

## Hardware notes (CodeCell C6)

ESP32-C6, Wi-Fi 6 + BLE 5 + Zigbee, 8 MB flash, USB-C + LiPo charging.

Two sensors, both on I²C (SDA=GPIO8, SCL=GPIO9, 400 kHz, internal pull-ups):
- **VCNL4040** (`0x60`) — ambient light, white light, proximity (~20 cm)
- **BNO085** (`0x4A`) — 9-axis IMU with onboard sensor fusion

The firmware enables everything at once via a single `Init()` call and runs the
service loop at 50 Hz (`myCodeCell.Run(50)`).

Things worth knowing:
- The IMU needs **10–30 seconds after power-on** to stabilise its fused outputs.
  Early readings are unreliable — don't treat them as a bug.
- The onboard RGB LED doubles as a power indicator and `Run()` can override
  manual `LED()` calls. `LED_SetBrightness(0)` disables the idle animation but
  the low-battery red blink always survives.
- Below 3.3 V the board blinks red and sleeps. For an installation running for
  hours, watch the `_bat` channel.
- ADC pins read 0–4095 against a **2.5 V** reference, not 3.3 V.

Useful CodeCell API, for reference:

```cpp
myCodeCell.Init(LIGHT + MOTION_ROTATION + ...);   // combine modes with +
myCodeCell.Run(50);                                // true at the given Hz
myCodeCell.Light_ProximityRead();                  // uint16_t
myCodeCell.Light_AmbientRead();                    // uint16_t
myCodeCell.Motion_AccelerometerRead(x, y, z);      // m/s², floats by reference
myCodeCell.Motion_GyroRead(x, y, z);               // °/s
myCodeCell.Motion_MagnetometerRead(x, y, z);
myCodeCell.Motion_RotationRead(roll, pitch, yaw);  // degrees
myCodeCell.Motion_GravityRead(x, y, z);
myCodeCell.Motion_LinearAccRead(x, y, z);
myCodeCell.Motion_StateRead();                     // 0–4
myCodeCell.Motion_ActivityRead();                  // 1–8
myCodeCell.Motion_StepCounterRead();               // uint16_t
myCodeCell.BatteryLevelRead();                     // 1–100, 101, 102
myCodeCell.LED(r, g, b);
myCodeCell.pinPWM(pin, freq_hz, duty_0_100);       // any of the 6 GPIOs
```

Full documentation: <https://microbots.io/blogs/learn>

## Roadmap

1. **Now:** one working `stickB` device end-to-end in Ableton.
2. **Next:** multiple pendulums at once — each with its own `DEVICE_NAME`, all
   sending to the same port 9999. The JS already filters by device name, so a
   second device means a second `.amxd` instance with a different `setdevice`.
3. **Later:** MIDI out from Max for Live to hardware synths.

## When adding a new sensor channel

Four places, in order — miss one and the channel is silently dead:

1. `main.cpp` — declare the tag `String` and a `lastX` cache variable
2. `main.cpp` `setup()` — build the tag from `DEVICE_NAME`
3. `main.cpp` `loop()` — read, scale to 0–127, send only if changed
4. `codecell_receive2.js` — add the suffix to `channelVar` and raise the
   `outlets` count
5. The `.amxd`, inside Max — add a `setvar <channel> <suffix>` message box and
   wire up the new outlet. **This step overrides step 4**, so the suffix must be
   spelled identically in both.
