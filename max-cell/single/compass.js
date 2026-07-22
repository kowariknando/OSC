autowatch = 1;
outlets = 2; // 0 = compass value (0-127, smoothed) | 1 = battery (0-100, 101=charging, 102=USB)

// ============================================================================
// compass.js — single-variable CodeCell receiver + normaliser (COMPASS / MAG)
//
// One plugin, one job: decode <device>_comp (magnetometer magnitude) into a
// smooth, self-calibrating 0-127 value.
//
// NOTE: the magnetometer is the noisiest of the four channels and its magnitude
// does not map cleanly to pendulum motion (it reacts to orientation vs. the
// Earth's field and to nearby metal). Defaults here are therefore heavier:
// more smoothing (lower coefficient) and a higher noise floor to reject jitter.
// If it still feels twitchy, lower `smoothing` further (e.g. 0.08) and/or raise
// `minspan`.
//
// PIPELINE (see handle()):
//   1. AUTO-RANGE  - learn real min/max, stretch to 0-127.
//   2. NOISE FLOOR - reject jitter when the real swing is small (minSpan).
//   3. SMOOTH      - one-pole low-pass for organic glides.
//
// WIRING (in the .amxd):
//   [udpreceive 9004 @outputformat rawbytes] -> [js compass.js] -> rest of device
//   [textedit] -> [route text] -> [prepend setdevice] -> [js compass.js]
//   Use a DIFFERENT udp port per plugin (see max/single/README.md).
//
// LIVE TUNING (send as messages):
//   smoothing <0..1>   glide amount     minspan <1..127> swing-to-full-scale
//   rangedecay <0..1>  range relax rate  autorange <0|1>  AGC on/off
//   clocked <0|1>      metro-driven glide (drive [js] with [metro 20])
//   resetrange | printparams | printrange
// ============================================================================

var SUFFIX = "comp"; // <device>_comp

var device = "";
var tag    = "";     // <device>_comp  (the sensor value)
var batTag = "";     // <device>_bat   (battery, always sent every ~5s)

// ---- tunables (defaults tuned for the noisy magnetometer: heavier smoothing) ----
var smoothing  = 0.15;
var minSpan    = 12.0;
var rangeDecay = 0.002;
var autorange  = 1;
var clocked    = 0;

// ---- adaptive state ----
var inited  = false;
var rMin    = 0;
var rMax    = 0;
var smooth  = 0;
var target  = 0;
var lastOut = -1;

function resetState() {
    inited = false; rMin = 0; rMax = 0; smooth = 0; target = 0; lastOut = -1;
}

// ---- UDP intake ----
function list() {
    var bytes = arrayfromargs(arguments);
    var str = "";
    for (var i = 0; i < bytes.length; i++) str += String.fromCharCode(bytes[i]);
    parse(str);
}
function rawbytes() { list.apply(this, arguments); }
function msg_int(b) { parse(String.fromCharCode(b)); }

function parse(str) {
    str = str.replace(/\0/g, "").trim();
    var messages = str.split(";");
    for (var i = 0; i < messages.length; i++) {
        var m = messages[i].trim();
        if (m.length === 0) continue;
        var parts = m.split(/\s+/);
        if (parts.length < 2) continue;
        var value = parseFloat(parts[1]);
        if (isNaN(value)) continue;
        if (tag && parts[0] === tag) handle(value);              // compass value -> outlet 0
        else if (batTag && parts[0] === batTag) outlet(1, value); // battery -> outlet 1 (raw)
    }
}

// ---- normalisation ----
function handle(v) {
    if (!inited) {
        rMin = v; rMax = v; smooth = v; target = v; inited = true;
        emit(Math.round(v));
        return;
    }
    var t;
    if (autorange) {
        if (v < rMin) rMin = v; else rMin += (v - rMin) * rangeDecay;
        if (v > rMax) rMax = v; else rMax += (v - rMax) * rangeDecay;
        var span = rMax - rMin;
        var effSpan = span > minSpan ? span : minSpan;
        var norm = (v - rMin) / effSpan;
        if (norm < 0) norm = 0; else if (norm > 1) norm = 1;
        t = norm * 127.0;
    } else {
        t = v;
    }
    target = t;
    if (!clocked) {
        smooth += (target - smooth) * smoothing;
        emit(Math.round(smooth));
    }
}

function bang() {
    if (!clocked || !inited) return;
    smooth += (target - smooth) * smoothing;
    emit(Math.round(smooth));
}

function emit(iv) {
    if (iv !== lastOut) { outlet(0, iv); lastOut = iv; }
}

// ---- device selection ----
// Drop the leading "text" atom that Max's textedit prepends to a typed name,
// so "setdevice text stickA" resolves to stickA (not "text").
function setdevice() {
    var a = arrayfromargs(arguments);
    if (a.length && String(a[0]) === "text") a = a.slice(1);
    device = a.length ? String(a[0]) : "";
    tag    = device + "_" + SUFFIX;
    batTag = device + "_bat";
    resetState();
    post("[compass] Device -> " + device + "   listening for: " + tag + " (+" + batTag + ")\n");
}

// ---- live tuning ----
function minspan(x)    { minSpan = x < 1 ? 1 : x; }
function rangedecay(x) { rangeDecay = clamp01(x); }

// "smoothamount 0..100" - the on-screen Smooth knob. 0 = raw/snappy,
// 100 = very smooth. Maps to the internal one-pole coefficient (higher
// amount = lower coefficient = gentler, more organic glide).
function smoothamount(x) {
    if (x < 0) x = 0; else if (x > 100) x = 100;
    smoothing = 0.6 * Math.pow(0.04, x / 100.0); // 0 ->0.6, 50 ->~0.12, 100 ->~0.024
}
function resetrange()  { resetState(); post("[compass] Range reset\n"); }
function printparams() {
    post("[compass] smoothing=" + smoothing + " minspan=" + minSpan +
         " rangedecay=" + rangeDecay + " autorange=" + autorange +
         " clocked=" + clocked + "\n");
}
function printrange() {
    if (!inited) { post("[compass] no data yet\n"); return; }
    post("[compass] min=" + round1(rMin) + " max=" + round1(rMax) + "\n");
}
function round1(x) { return Math.round(x * 10) / 10; }
function clamp01(x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

function anything() {
    var a = arrayfromargs(messagename, arguments);
    var name = a[0];
    var arg = a.length > 1 ? a[1] : 0;
    if (name === "smoothing")      smoothing = clamp01(arg);
    else if (name === "autorange") autorange = arg ? 1 : 0;
    else if (name === "clocked")   clocked = arg ? 1 : 0;
    else post("[compass] unknown message: " + name + "\n");
}
