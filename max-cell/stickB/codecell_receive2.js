autowatch = 1;

// ============================================================================
// codecell_receive2.js
//
// Decodes the plain-text UDP packets from the CodeCell firmware AND normalises
// them so every channel produces smooth, organic 0-127 output regardless of how
// badly the firmware's fixed min/max ranges are calibrated.
//
// WHY THIS EXISTS
//   The firmware scales each sensor to 0-127 using fixed, hand-guessed ranges
//   (gyroMax=250, gravMax=12, ...). Those ranges are wrong for juggling motion:
//     - some channels barely move (grav sits at ~102 forever, gyro squashed near 0)
//     - nothing is smoothed, so noisy channels (comp) arrive as hard steps.
//   Reflashing every board with better ranges is slow and per-environment. So we
//   fix it here instead, in the one file that is diffable and shared by all
//   devices. No firmware changes, no .amxd surgery.
//
// WHAT IT DOES, per continuous channel:
//   1. AUTO-RANGE  - learn the actual min/max this channel really produces and
//                    stretch that span to fill 0-127 (self-calibrating AGC).
//   2. NOISE FLOOR - if the real span is tiny (a "dead" channel like grav), keep
//                    it quiet near 0 instead of amplifying sensor noise.
//   3. SMOOTH      - a one-pole low-pass so steps become organic glides.
//
//   Categorical channels (state, act, steps, bat) pass through untouched - it
//   makes no sense to smooth or auto-range an integer state code or step count.
//
// LIVE TUNING (send these as messages into the js object; defaults are sane):
//   smoothing <0..1>   glide amount. Lower = smoother/slower, higher = snappier.
//   minspan   <1..127> how big a channel's real swing must be to reach full
//                      output. Lower = more sensitive (and more noise); higher =
//                      only big motions reach 127.
//   rangedecay <0..1>  how fast the learned min/max relax back (small, e.g. .002)
//   autorange <0|1>    turn the AGC on/off (off = just smooth the raw value)
//   clocked   <0|1>    0 = smooth+output on each packet (default, needs no metro)
//                      1 = packets only set targets; a metro must bang() the js,
//                          giving framerate-independent glide even when packets
//                          are sparse. Wire a [metro 20]->[js] if you use this.
//   resetrange         forget all learned ranges (e.g. after moving location)
//   printparams        post current settings to the Max window
//   printrange         post each channel's learned min/max to the Max window
// ============================================================================

// NOTE: VARLIST is currently unused - channelVar is what drives the parsing.
// 0-10: the 11 sensor variables | 11: battery | 12: sleep state | 13: idle timeout
var channelVar = ["inertia","prox","state","act","gyro","comp","rot","steps","light","grav","lin","bat","sleep","timeout"];

var NUM_CHANNELS = channelVar.length;
outlets = NUM_CHANNELS;
var device = "";
var channelTag = [];
for (var initCh = 0; initCh < NUM_CHANNELS; initCh++) channelTag[initCh] = "";

// Channels whose value is a code/count, not a continuous magnitude. These skip
// all normalisation and pass straight through. Keyed by variable NAME so the
// behaviour follows the variable even if setvar remaps it to another outlet.
var PASSTHROUGH = { state: true, act: true, steps: true, bat: true, sleep: true, timeout: true };

// ---- tunable parameters ----
var smoothing  = 0.25;   // one-pole low-pass coefficient (0..1)
var minSpan    = 8.0;    // noise-floor: minimum span before a channel reaches full scale
var rangeDecay = 0.002;  // how fast learned min/max relax back toward the signal
var autorange  = 1;      // 1 = adaptive normalisation on, 0 = smooth raw only
var clocked    = 0;      // 1 = output only on bang() (metro-driven), 0 = per packet

// ---- per-channel adaptive state (index 0..11) ----
var sInit    = [];  // has this channel seen its first value yet
var sMin     = [];  // learned minimum (raw 0-127 domain)
var sMax     = [];  // learned maximum
var sSmooth  = [];  // current smoothed OUTPUT value (0-127)
var sTarget  = [];  // most recent normalisation target (used by clocked mode)
var sLastOut = [];  // last integer emitted, to suppress duplicate outputs
resetRange();

function resetRange() {
    for (var ch = 0; ch < 12; ch++) resetChannel(ch);
}

function resetChannel(ch) {
    sInit[ch]    = false;
    sMin[ch]     = 0;
    sMax[ch]     = 0;
    sSmooth[ch]  = 0;
    sTarget[ch]  = 0;
    sLastOut[ch] = -1;
}

// ---------------------------------------------------------------------------
// UDP intake
// ---------------------------------------------------------------------------
function list() {
    var bytes = arrayfromargs(arguments);
    var str = "";
    for (var i = 0; i < bytes.length; i++) {
        str += String.fromCharCode(bytes[i]);
    }
    parseAndOutput(str);
}

function rawbytes() {
    list.apply(this, arguments);
}

function msg_int(b) {
    parseAndOutput(String.fromCharCode(b));
}

function parseAndOutput(str) {
    str = str.replace(/\0/g, "").trim();
    var messages = str.split(";");
    for (var i = 0; i < messages.length; i++) {
        var m = messages[i].trim();
        if (m.length === 0) continue;
        var parts = m.split(/\s+/);
        if (parts.length < 2) continue;

        var tag = parts[0];
        var value = parseFloat(parts[1]);
        if (isNaN(value)) continue;

        for (var ch = 0; ch < NUM_CHANNELS; ch++) {
            if (channelTag[ch] && tag === channelTag[ch]) {
                handleChannel(ch, value);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Normalisation
// ---------------------------------------------------------------------------
function handleChannel(ch, v) {
    // Categorical channels: pass straight through, no smoothing/auto-range.
    if (PASSTHROUGH[channelVar[ch]]) {
        outlet(ch, v);
        sLastOut[ch] = v;
        return;
    }

    if (!sInit[ch]) {
        // First value seeds the range and the smoother so we start from reality
        // rather than gliding up from 0.
        sMin[ch]    = v;
        sMax[ch]    = v;
        sSmooth[ch] = v;
        sTarget[ch] = v;
        sInit[ch]   = true;
        emit(ch, Math.round(v));
        return;
    }

    var target;
    if (autorange) {
        // Learn the range: expand instantly toward new extremes, relax slowly
        // back so a single spike doesn't lock the range wide open forever.
        if (v < sMin[ch]) sMin[ch] = v;
        else              sMin[ch] += (v - sMin[ch]) * rangeDecay;
        if (v > sMax[ch]) sMax[ch] = v;
        else              sMax[ch] += (v - sMax[ch]) * rangeDecay;

        // effSpan pins the denominator at >= minSpan. When the real swing is
        // smaller than minSpan (a "dead"/quiet channel) the output stays
        // compressed near the low end instead of amplifying noise to full scale.
        var span    = sMax[ch] - sMin[ch];
        var effSpan = span > minSpan ? span : minSpan;

        var norm = (v - sMin[ch]) / effSpan;
        if (norm < 0) norm = 0; else if (norm > 1) norm = 1;
        target = norm * 127.0;
    } else {
        target = v; // auto-range off: just smooth the raw value
    }

    sTarget[ch] = target;

    // In clocked mode, bang() does the smoothing+output so glide is driven by a
    // metro and stays smooth even when packets are sparse.
    if (!clocked) {
        sSmooth[ch] += (target - sSmooth[ch]) * smoothing;
        emit(ch, Math.round(sSmooth[ch]));
    }
}

// Advance every continuous channel one smoothing step toward its target and
// output. Only used in clocked mode - wire a [metro 20] (or similar) into the
// js object so this fires at a steady rate.
function bang() {
    if (!clocked) return;
    for (var ch = 0; ch < 12; ch++) {
        if (!sInit[ch]) continue;
        if (PASSTHROUGH[channelVar[ch]]) continue;
        sSmooth[ch] += (sTarget[ch] - sSmooth[ch]) * smoothing;
        emit(ch, Math.round(sSmooth[ch]));
    }
}

// Emit only when the integer output actually changed, to keep traffic down.
function emit(ch, iv) {
    if (iv !== sLastOut[ch]) {
        outlet(ch, iv);
        sLastOut[ch] = iv;
    }
}

// ---------------------------------------------------------------------------
// Live tuning messages
// ---------------------------------------------------------------------------
// "minspan <x>", "rangedecay <x>" get their own functions (no name clash).
// "smoothing", "autorange" and "clocked" clash with variable names, so they are
// handled in anything() below instead.
function minspan(x)    { minSpan = x < 1 ? 1 : x; }
function rangedecay(x) { rangeDecay = clamp01(x); }

function clamp01(x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

function resetrange() { resetRange(); post("Ranges reset\n"); }

function printparams() {
    post("smoothing=" + smoothing + " minspan=" + minSpan +
         " rangedecay=" + rangeDecay + " autorange=" + autorange +
         " clocked=" + clocked + "\n");
}

function printrange() {
    for (var ch = 0; ch < 12; ch++) {
        if (!sInit[ch]) continue;
        post("ch" + ch + " " + channelVar[ch] +
             " min=" + round1(sMin[ch]) + " max=" + round1(sMax[ch]) + "\n");
    }
}

function round1(x) { return Math.round(x * 10) / 10; }

// Max maps the message "smoothing 0.3" to a function named "smoothing", but we
// already use "smoothing" as a variable. Bridge the two so both work.
function anything() {
    var a = arrayfromargs(messagename, arguments);
    var name = a[0];
    var arg = a.length > 1 ? a[1] : 0;
    if (name === "smoothing")      smoothing = clamp01(arg);
    else if (name === "autorange") autorange = arg ? 1 : 0;
    else if (name === "clocked")   clocked = arg ? 1 : 0;
    else post("unknown message: " + name + "\n");
}

// ---------------------------------------------------------------------------
// Device / channel wiring (unchanged contract with the .amxd)
// ---------------------------------------------------------------------------
function updateAllTags() {
    for (var ch = 0; ch < NUM_CHANNELS; ch++) {   // include sleep (12) and timeout (13)
        if (device && channelVar[ch]) {
            channelTag[ch] = device + "_" + channelVar[ch];
        }
    }
    post("Device -> " + device + "\n");
}

// "setdevice <stickB>" - a single device name for the whole plugin.
// Drop the leading "text" atom that Max's textedit prepends to a typed name,
// so "setdevice text stickB" resolves to stickB (not "text").
function setdevice() {
    var a = arrayfromargs(arguments);
    if (a.length && String(a[0]) === "text") a = a.slice(1);
    device = a.length ? String(a[0]) : "";
    resetRange(); // new device => the learned ranges no longer apply
    updateAllTags();
}

// "setvar <channel 0-13> <inertia/prox/state/act/gyro/comp/rot/steps/light/grav/lin/bat/sleep/timeout>"
// // Normally fixed per channel (one variable per row), but can be rewired if needed.
function setvar(ch, name) {
    ch = Math.floor(ch);
    if (ch >= 0 && ch < NUM_CHANNELS) {
        channelVar[ch] = String(name);
        resetChannel(ch); // meaning of this channel changed => forget its range
        updateAllTags();
    }
}
