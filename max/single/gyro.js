autowatch = 1;
outlets = 2; // 0 = gyro value (0-127, smoothed) | 1 = battery (0-100, 101=charging, 102=USB)

// ============================================================================
// gyro.js — single-variable CodeCell receiver + normaliser (GYRO)
//
// One plugin, one job: decode <device>_gyro from the UDP stream and turn it
// into a smooth, self-calibrating 0-127 value on outlet 0. Outlet 1 passes the
// battery through untouched — the firmware always sends <device>_bat every ~5s,
// regardless of the sensor profile, so a single-variable prop still reports it.
//
// PIPELINE for the value (see handle()):
//   1. AUTO-RANGE  - learn the real min/max this device's gyro produces and
//                    stretch it to fill 0-127. Fixes the firmware squashing gyro
//                    near 0 (it divides by gyroMax=250, far bigger than reality).
//   2. NOISE FLOOR - if the real swing is tiny, stay quiet near 0 instead of
//                    amplifying sensor noise to full scale (minSpan).
//   3. SMOOTH      - one-pole low-pass so steps become organic glides.
//
// WIRING (in the .amxd):
//   [udpreceive 9999 @outputformat rawbytes] -> [js gyro.js]
//   outlet 0 -> your value mapping    outlet 1 -> battery readout
//   [textedit] -> [prepend setdevice] -> [js gyro.js]   (name box)
//
// LIVE TUNING (send as messages; defaults tuned for gyro):
//   smoothing <0..1>   minspan <1..127>   rangedecay <0..1>
//   autorange <0|1>    clocked <0|1>      resetrange | printparams | printrange
// ============================================================================

var SUFFIX = "gyro"; // <device>_gyro

var device = "";
var tag    = "";     // <device>_gyro  (the sensor value)
var batTag = "";     // <device>_bat   (battery, always sent every ~5s)

// ---- tunables (defaults tuned for gyro) ----
var smoothing  = 0.25;
var minSpan    = 8.0;
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
        if (tag && parts[0] === tag) handle(value);              // gyro value -> outlet 0
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
// "setdevice stickA" -> listen for stickA_gyro (+ stickA_bat).
// Max's textedit prepends the word "text", so a typed name arrives as
// "setdevice text stickA"; we drop a leading "text" atom so the real name wins.
function setdevice() {
    var a = arrayfromargs(arguments);
    if (a.length && String(a[0]) === "text") a = a.slice(1);
    device = a.length ? String(a[0]) : "";
    tag    = device + "_" + SUFFIX;
    batTag = device + "_bat";
    resetState();
    post("[gyro] Device -> " + device + "   listening for: " + tag + " (+" + batTag + ")\n");
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
function resetrange()  { resetState(); post("[gyro] Range reset\n"); }
function printparams() {
    post("[gyro] smoothing=" + smoothing + " minspan=" + minSpan +
         " rangedecay=" + rangeDecay + " autorange=" + autorange +
         " clocked=" + clocked + "\n");
}
function printrange() {
    if (!inited) { post("[gyro] no data yet\n"); return; }
    post("[gyro] min=" + round1(rMin) + " max=" + round1(rMax) + "\n");
}
function round1(x) { return Math.round(x * 10) / 10; }
function clamp01(x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

// "smoothing"/"autorange"/"clocked" clash with variable names, so route them here.
function anything() {
    var a = arrayfromargs(messagename, arguments);
    var name = a[0];
    var arg = a.length > 1 ? a[1] : 0;
    if (name === "smoothing")      smoothing = clamp01(arg);
    else if (name === "autorange") autorange = arg ? 1 : 0;
    else if (name === "clocked")   clocked = arg ? 1 : 0;
    else post("[gyro] unknown message: " + name + "\n");
}
