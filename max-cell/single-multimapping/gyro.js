autowatch = 1;
outlets = 8; // 0-4 = five independently-smoothed lanes of the gyro value | 5 = battery | 6 = sleep | 7 = idle timeout

// ============================================================================
// gyro.js — multi-mapping CodeCell receiver + normaliser (GYRO)
//
// Decodes <device>_gyro and sends it to FIVE independent mapping lanes, each
// with its own smoothing so one prop can drive five Ableton parameters, each
// reacting with a different amount of "organic" glide.
//
//   raw gyro -> shared AUTO-RANGE + NOISE FLOOR -> target (0-127)
//            -> lane 0 smoother -> outlet 0
//            -> lane 1 smoother -> outlet 1
//            ...                   ...
//            -> lane 4 smoother -> outlet 4
//   battery  -> outlet 5 (raw: 0-100, 101=charging, 102=USB)
//
// Each lane's Smooth knob sends "smoothamount <lane> <0..100>" (0 = raw/snappy,
// 100 = very smooth). Auto-range is SHARED (it is a property of the sensor); only
// the smoothing is per-lane.
//
// LIVE TUNING (messages): minspan <1..127> | rangedecay <0..1> | autorange <0|1>
//   | resetrange | printparams
// ============================================================================

var SUFFIX = "gyro";
var NLANES = 5;

var device = "";
var tag        = "";   // <device>_gyro
var batTag     = "";   // <device>_bat
var sleepTag   = "";   // <device>_sleep    (1 = going to sleep, 0 = awake / just booted)
var timeoutTag = "";   // <device>_timeout  (current idle timeout in minutes, 0 = off)

// ---- shared auto-range tunables ----
var minSpan    = 8.0;
var rangeDecay = 0.002;
var autorange  = 1;

// ---- shared adaptive range ----
var inited = false;
var rMin = 0, rMax = 0, target = 0;

// ---- per-lane smoothing state ----
var laneCoeff   = [];
var laneSmooth  = [];
var laneLastOut = [];
for (var i = 0; i < NLANES; i++) { laneCoeff[i] = 0.12; laneSmooth[i] = 0; laneLastOut[i] = -1; }

function resetState() {
    inited = false; rMin = 0; rMax = 0; target = 0;
    for (var i = 0; i < NLANES; i++) { laneSmooth[i] = 0; laneLastOut[i] = -1; }
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
        if (tag && parts[0] === tag) handle(value);                       // gyro value
        else if (batTag && parts[0] === batTag) outlet(5, value);         // battery      -> outlet 5
        else if (sleepTag && parts[0] === sleepTag) outlet(6, value);     // sleep state  -> outlet 6
        else if (timeoutTag && parts[0] === timeoutTag) outlet(7, value); // idle timeout -> outlet 7
    }
}

// ---- shared auto-range -> per-lane smoothing ----
function handle(v) {
    if (!inited) {
        rMin = v; rMax = v; target = v; inited = true;
        for (var i = 0; i < NLANES; i++) laneSmooth[i] = v;
        emitAll();
        return;
    }
    if (autorange) {
        if (v < rMin) rMin = v; else rMin += (v - rMin) * rangeDecay;
        if (v > rMax) rMax = v; else rMax += (v - rMax) * rangeDecay;
        var span = rMax - rMin;
        var effSpan = span > minSpan ? span : minSpan;
        var norm = (v - rMin) / effSpan;
        if (norm < 0) norm = 0; else if (norm > 1) norm = 1;
        target = norm * 127.0;
    } else {
        target = v;
    }
    for (var j = 0; j < NLANES; j++) {
        laneSmooth[j] += (target - laneSmooth[j]) * laneCoeff[j];
        emitLane(j);
    }
}

function emitLane(i) {
    var iv = Math.round(laneSmooth[i]);
    if (iv !== laneLastOut[i]) { outlet(i, iv); laneLastOut[i] = iv; }
}
function emitAll() { for (var i = 0; i < NLANES; i++) emitLane(i); }

// ---- per-lane Smooth knob ----
// "smoothamount <lane 0..4> <0..100>"  (0 = raw/snappy, 100 = very smooth)
function smoothamount(lane, x) {
    lane = Math.floor(lane);
    if (lane < 0 || lane >= NLANES) return;
    if (x < 0) x = 0; else if (x > 100) x = 100;
    laneCoeff[lane] = 0.6 * Math.pow(0.04, x / 100.0); // 0 ->0.6, 50 ->~0.12, 100 ->~0.024
}

// ---- device selection ----
function setdevice() {
    var a = arrayfromargs(arguments);
    if (a.length && String(a[0]) === "text") a = a.slice(1);
    device = a.length ? String(a[0]) : "";
    tag        = device + "_" + SUFFIX;
    batTag     = device + "_bat";
    sleepTag   = device + "_sleep";
    timeoutTag = device + "_timeout";
    resetState();
    post("[" + SUFFIX + "] Device -> " + device + "   (" + tag + " x" + NLANES + " lanes  +" + batTag + ")\n");
}

// ---- shared tuning ----
function minspan(x)    { minSpan = x < 1 ? 1 : x; }
function rangedecay(x) { rangeDecay = clamp01(x); }
function resetrange()  { resetState(); post("[" + SUFFIX + "] range reset\n"); }
function printparams() {
    post("[" + SUFFIX + "] minspan=" + minSpan + " rangedecay=" + rangeDecay +
         " autorange=" + autorange + " laneCoeff=" + laneCoeff.join(",") + "\n");
}
function clamp01(x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

// "autorange" clashes with the variable name, so route it through anything().
function anything() {
    var a = arrayfromargs(messagename, arguments);
    if (a[0] === "autorange") autorange = (a.length > 1 && a[1]) ? 1 : 0;
    else post("[" + SUFFIX + "] unknown message: " + a[0] + "\n");
}
