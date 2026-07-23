autowatch = 1;
outlets = 8; // 0-4 = five independently-smoothed lanes of the PAN | 5 = battery | 6 = sleep | 7 = idle timeout

// ============================================================================
// infinitecompass.js (multi-mapping) — compass HEADING folded into a jump-free PAN
//
// A raw compass heading wraps 127 -> 0 at North, which makes panning jump from
// hard-right to hard-left. This folds the circle onto the East-West axis so the
// value is CONTINUOUS and never jumps:
//
//     pan = 63.5 * (1 - sin(heading))
//
//   Heading   Direction   Pan
//   North     0 / 360     63.5  (centre)   <- the wrap point sits at centre,
//   East      90          0     (left)        so both sides agree: no jump
//   South     180         63.5  (centre)
//   West      270         127   (right)
//
// Reads <device>_head (firmware atan2 heading, 0-127). No firmware change.
// Five lanes (outlets 0-4), each with its own Smooth knob:
//   "smoothamount <lane 0..4> <0..100>"   (0 = raw, 100 = very smooth)
// Battery on outlet 5. The pan has no wrap, so smoothing is LINEAR per lane.
// ============================================================================

var SUFFIX = "head"; // <device>_head
var NLANES = 5;

var device = "";
var tag        = "";
var batTag     = "";
var sleepTag   = "";   // <device>_sleep    (1 = going to sleep, 0 = awake / just booted)
var timeoutTag = "";   // <device>_timeout  (current idle timeout in minutes, 0 = off)

var inited = false;
var laneCoeff   = [];
var laneSmooth  = [];
var laneLastOut = [];
for (var i = 0; i < NLANES; i++) { laneCoeff[i] = 0.12; laneSmooth[i] = 63.5; laneLastOut[i] = -1; }

function resetState() {
    inited = false;
    for (var i = 0; i < NLANES; i++) { laneSmooth[i] = 63.5; laneLastOut[i] = -1; }
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
        if (tag && parts[0] === tag) handle(value);                       // heading -> pan
        else if (batTag && parts[0] === batTag) outlet(5, value);         // battery      -> outlet 5
        else if (sleepTag && parts[0] === sleepTag) outlet(6, value);     // sleep state  -> outlet 6
        else if (timeoutTag && parts[0] === timeoutTag) outlet(7, value); // idle timeout -> outlet 7
    }
}

// heading (0-127 = 0-360 deg) -> continuous pan (0-127)
function headingToPan(v) {
    var rad = v * (2 * Math.PI / 128.0);   // 0-127 -> 0..2pi
    return 63.5 * (1 - Math.sin(rad));     // N/S = 63.5, E = 0, W = 127
}

function handle(v) {
    var pan = headingToPan(v);
    if (!inited) {
        inited = true;
        for (var i = 0; i < NLANES; i++) laneSmooth[i] = pan;
        emitAll();
        return;
    }
    for (var j = 0; j < NLANES; j++) {
        laneSmooth[j] += (pan - laneSmooth[j]) * laneCoeff[j]; // linear: pan never wraps
        emitLane(j);
    }
}

function emitLane(i) {
    var iv = Math.round(laneSmooth[i]);
    if (iv < 0) iv = 0; else if (iv > 127) iv = 127;
    if (iv !== laneLastOut[i]) { outlet(i, iv); laneLastOut[i] = iv; }
}
function emitAll() { for (var i = 0; i < NLANES; i++) emitLane(i); }

// ---- per-lane Smooth knob ----
function smoothamount(lane, x) {
    lane = Math.floor(lane);
    if (lane < 0 || lane >= NLANES) return;
    if (x < 0) x = 0; else if (x > 100) x = 100;
    laneCoeff[lane] = 0.6 * Math.pow(0.04, x / 100.0);
}

// ---- device selection ----
function setdevice() {
    var a = arrayfromargs(arguments);
    if (a.length && String(a[0]) === "text") a = a.slice(1);
    device = a.length ? String(a[0]) : "";
    tag        = device + "_" + SUFFIX;   // <device>_head
    batTag     = device + "_bat";
    sleepTag   = device + "_sleep";
    timeoutTag = device + "_timeout";
    resetState();
    post("[infpan] Device -> " + device + "   (pan from " + tag + " x" + NLANES + " lanes  +" + batTag + ")\n");
}

function resetrange()  { resetState(); post("[infpan] reset\n"); }
function printparams() { post("[infpan] laneCoeff=" + laneCoeff.join(",") + "\n"); }

function anything() {
    var a = arrayfromargs(messagename, arguments);
    post("[infpan] unknown message: " + a[0] + "\n");
}
