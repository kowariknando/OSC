autowatch = 1;
outlets = 6; // 0-4 = five independently-smoothed lanes of the inertia value | 5 = battery

// ============================================================================
// inertia.js — multi-mapping CodeCell receiver + normaliser (INERTIA)
// Decodes <device>_inertia into 5 independently-smoothed lanes (outlets 0-4)
// plus battery on outlet 5. Each lane's Smooth knob sends
// "smoothamount <lane> <0..100>". Auto-range is shared; smoothing is per-lane.
// See gyro.js for the full explanation of the pipeline.
// ============================================================================

var SUFFIX = "inertia";
var NLANES = 5;

var device = "";
var tag    = "";
var batTag = "";

var minSpan    = 8.0;
var rangeDecay = 0.002;
var autorange  = 1;

var inited = false;
var rMin = 0, rMax = 0, target = 0;

var laneCoeff   = [];
var laneSmooth  = [];
var laneLastOut = [];
for (var i = 0; i < NLANES; i++) { laneCoeff[i] = 0.12; laneSmooth[i] = 0; laneLastOut[i] = -1; }

function resetState() {
    inited = false; rMin = 0; rMax = 0; target = 0;
    for (var i = 0; i < NLANES; i++) { laneSmooth[i] = 0; laneLastOut[i] = -1; }
}

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
        if (tag && parts[0] === tag) handle(value);
        else if (batTag && parts[0] === batTag) outlet(5, value);
    }
}

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

function smoothamount(lane, x) {
    lane = Math.floor(lane);
    if (lane < 0 || lane >= NLANES) return;
    if (x < 0) x = 0; else if (x > 100) x = 100;
    laneCoeff[lane] = 0.6 * Math.pow(0.04, x / 100.0);
}

function setdevice() {
    var a = arrayfromargs(arguments);
    if (a.length && String(a[0]) === "text") a = a.slice(1);
    device = a.length ? String(a[0]) : "";
    tag    = device + "_" + SUFFIX;
    batTag = device + "_bat";
    resetState();
    post("[" + SUFFIX + "] Device -> " + device + "   (" + tag + " x" + NLANES + " lanes  +" + batTag + ")\n");
}

function minspan(x)    { minSpan = x < 1 ? 1 : x; }
function rangedecay(x) { rangeDecay = clamp01(x); }
function resetrange()  { resetState(); post("[" + SUFFIX + "] range reset\n"); }
function printparams() {
    post("[" + SUFFIX + "] minspan=" + minSpan + " rangedecay=" + rangeDecay +
         " autorange=" + autorange + " laneCoeff=" + laneCoeff.join(",") + "\n");
}
function clamp01(x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

function anything() {
    var a = arrayfromargs(messagename, arguments);
    if (a[0] === "autorange") autorange = (a.length > 1 && a[1]) ? 1 : 0;
    else post("[" + SUFFIX + "] unknown message: " + a[0] + "\n");
}
