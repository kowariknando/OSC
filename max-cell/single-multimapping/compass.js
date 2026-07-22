autowatch = 1;
outlets = 6; // 0-4 = five independently-smoothed lanes of the compass HEADING | 5 = battery

// ============================================================================
// compass.js (multi-mapping) — DIGITAL COMPASS HEADING
//
// Unlike the other plugins, this one is a real compass: it decodes
// <device>_head, the heading the firmware computes with atan2(y, x) (see the
// CodeCell "Digital Compass" docs), already scaled 0-360 deg -> 0-127:
//     0 = North,  ~32 = East,  ~64 = South,  ~96 = West   (127 wraps back to 0)
//
// A heading has a FIXED absolute range, so there is NO auto-range here (that
// would destroy the absolute direction). It is mapped straight through and
// smoothed CIRCULARLY, so turning past North (127 -> 0) glides the short way
// round instead of sweeping all the way back through South.
//
// Five lanes (outlets 0-4), each with its own Smooth knob:
//   "smoothamount <lane 0..4> <0..100>"   (0 = raw, 100 = very smooth)
// Battery on outlet 5.
//
// NOTE: requires firmware that sends <device>_head (magnetometer profile).
// ============================================================================

var SUFFIX = "head";     // <device>_head  (the compass heading, 0-127)
var NLANES = 5;
var PERIOD = 128;        // 0..127 is circular; 127 is adjacent to 0
var HALF   = 64;

var device = "";
var tag    = "";
var batTag = "";

// ---- per-lane smoothing state ----
var target = 0;
var inited = false;
var laneCoeff   = [];
var laneSmooth  = [];
var laneLastOut = [];
for (var i = 0; i < NLANES; i++) { laneCoeff[i] = 0.12; laneSmooth[i] = 0; laneLastOut[i] = -1; }

function resetState() {
    inited = false; target = 0;
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
        if (tag && parts[0] === tag) handle(value);              // heading
        else if (batTag && parts[0] === batTag) outlet(5, value); // battery -> outlet 5
    }
}

// ---- circular smoothing (no auto-range: a heading is absolute) ----
function handle(v) {
    if (!inited) {
        target = v; inited = true;
        for (var i = 0; i < NLANES; i++) laneSmooth[i] = v;
        emitAll();
        return;
    }
    target = v;
    for (var j = 0; j < NLANES; j++) {
        var d = target - laneSmooth[j];
        if (d > HALF) d -= PERIOD; else if (d < -HALF) d += PERIOD; // shortest way round
        laneSmooth[j] += d * laneCoeff[j];
        if (laneSmooth[j] < 0) laneSmooth[j] += PERIOD;
        else if (laneSmooth[j] >= PERIOD) laneSmooth[j] -= PERIOD;
        emitLane(j);
    }
}

function emitLane(i) {
    var iv = Math.round(laneSmooth[i]);
    if (iv >= PERIOD) iv -= PERIOD;
    if (iv !== laneLastOut[i]) { outlet(i, iv); laneLastOut[i] = iv; }
}
function emitAll() { for (var i = 0; i < NLANES; i++) emitLane(i); }

// ---- per-lane Smooth knob ----
// "smoothamount <lane 0..4> <0..100>"  (0 = raw/snappy, 100 = very smooth)
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
    tag    = device + "_" + SUFFIX;   // <device>_head
    batTag = device + "_bat";
    resetState();
    post("[compass] Device -> " + device + "   (heading " + tag + " x" + NLANES + " lanes  +" + batTag + ")\n");
}

function resetrange()  { resetState(); post("[compass] smoothers reset\n"); }
function printparams() {
    post("[compass] heading mode  laneCoeff=" + laneCoeff.join(",") + "\n");
}

function anything() {
    var a = arrayfromargs(messagename, arguments);
    post("[compass] unknown message: " + a[0] + "\n");
}
