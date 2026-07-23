autowatch = 1;
outlets = 13; // 0-11 = 12 mapping points | 12 = battery

// ============================================================================
// roll-xyz.js (multi-mapping) — ROLL / PITCH / YAW + rotation vector total
//
// The firmware (MOTION_ROTATION profile) sends four channels, each 0-127:
//   <device>_roll   (roll,  -180..180 -> 0..127)   = X
//   <device>_pitch  (pitch, -180..180 -> 0..127)   = Y
//   <device>_yaw    (yaw,   -180..180 -> 0..127)   = Z
//   <device>_rvec   (|(roll,pitch,yaw)| -> 0..127) = vector total
//
// Layout: 3 mapping groups, each mapping X, Y, Z and the vector total = 12 points.
//   outlet  0  1  2  3   4  5  6  7   8  9 10 11        12
//   group   1  1  1  1   2  2  2  2   3  3  3  3      battery
//   coord   X  Y  Z  Σ   X  Y  Z  Σ   X  Y  Z  Σ
//
// So the SAME roll goes to outlets 0,4,8 (three destinations), etc. Each of the
// 12 points is smoothed independently:  "smoothamount <0..11> <0..100>".
//
// These are absolute orientation angles, so there is NO auto-range (that would
// destroy the absolute direction); values are mapped straight through and
// smoothed linearly. (Roll and yaw wrap at +/-180 -> 127/0; if that seam bothers
// a mapping, lower its Smooth or tell me and I add circular smoothing.)
// ============================================================================

var NPOINTS = 12;

var device  = "";
var tagRoll = "", tagPitch = "", tagYaw = "", tagRvec = "", batTag = "";

// which outlets each coordinate feeds (the three groups)
var OUT_ROLL  = [0, 4, 8];
var OUT_PITCH = [1, 5, 9];
var OUT_YAW   = [2, 6, 10];
var OUT_TOTAL = [3, 7, 11];

var laneCoeff = [], laneSmooth = [], laneLastOut = [], laneInit = [];
for (var i = 0; i < NPOINTS; i++) { laneCoeff[i] = 0.12; laneSmooth[i] = 0; laneLastOut[i] = -1; laneInit[i] = false; }

function resetState() {
    for (var i = 0; i < NPOINTS; i++) { laneSmooth[i] = 0; laneLastOut[i] = -1; laneInit[i] = false; }
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
        var t0 = parts[0];
        var v = parseFloat(parts[1]);
        if (isNaN(v)) continue;
        if      (t0 === tagRoll)  feed(OUT_ROLL,  v);
        else if (t0 === tagPitch) feed(OUT_PITCH, v);
        else if (t0 === tagYaw)   feed(OUT_YAW,   v);
        else if (t0 === tagRvec)  feed(OUT_TOTAL, v);
        else if (batTag && t0 === batTag) outlet(12, v); // battery
    }
}

// send one coordinate value to its three destination outlets, each smoothed
function feed(outs, v) {
    for (var k = 0; k < outs.length; k++) {
        var i = outs[k];
        if (!laneInit[i]) { laneSmooth[i] = v; laneInit[i] = true; }
        else              { laneSmooth[i] += (v - laneSmooth[i]) * laneCoeff[i]; }
        emit(i);
    }
}

function emit(i) {
    var iv = Math.round(laneSmooth[i]);
    if (iv < 0) iv = 0; else if (iv > 127) iv = 127;
    if (iv !== laneLastOut[i]) { outlet(i, iv); laneLastOut[i] = iv; }
}

// ---- per-point Smooth knob: "smoothamount <point 0..11> <0..100>" ----
function smoothamount(point, x) {
    point = Math.floor(point);
    if (point < 0 || point >= NPOINTS) return;
    if (x < 0) x = 0; else if (x > 100) x = 100;
    laneCoeff[point] = 0.6 * Math.pow(0.04, x / 100.0);
}

// ---- device selection ----
function setdevice() {
    var a = arrayfromargs(arguments);
    if (a.length && String(a[0]) === "text") a = a.slice(1);
    device  = a.length ? String(a[0]) : "";
    tagRoll  = device + "_roll";
    tagPitch = device + "_pitch";
    tagYaw   = device + "_yaw";
    tagRvec  = device + "_rvec";
    batTag   = device + "_bat";
    resetState();
    post("[roll-xyz] Device -> " + device + "   (roll/pitch/yaw/rvec x3 groups  +" + batTag + ")\n");
}

function resetrange()  { resetState(); post("[roll-xyz] reset\n"); }
function printparams() { post("[roll-xyz] laneCoeff=" + laneCoeff.join(",") + "\n"); }

function anything() {
    var a = arrayfromargs(messagename, arguments);
    post("[roll-xyz] unknown message: " + a[0] + "\n");
}
