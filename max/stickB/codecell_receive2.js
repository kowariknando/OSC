autowatch = 1;
outlets = 12; // 0-10: les 11 variables | 11: bateria

var VARLIST = ["inercia","prox","state","act","gyro","comp","rot","steps","llum","grav","lin"];
var device = "";
var channelVar = ["inercia","prox","state","act","gyro","comp","rot","steps","llum","grav","lin","bat"];
var channelTag = ["","","","","","","","","","","",""];

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

        for (var ch = 0; ch < 12; ch++) {
            if (channelTag[ch] && tag === channelTag[ch]) {
                outlet(ch, value);
            }
        }
    }
}

function updateAllTags() {
    for (var ch = 0; ch < 12; ch++) {
        if (device && channelVar[ch]) {
            channelTag[ch] = device + "_" + channelVar[ch];
        }
    }
    post("Dispositiu -> " + device + "\n");
}

// "setdevice <stickB>" - un sol nom de dispositiu per a tot el plugin
function setdevice(name) {
    device = String(name);
    updateAllTags();
}

// "setvar <canal 0-11> <inercia/prox/state/act/gyro/comp/rot/steps/llum/grav/lin/bat>"
// Normalment fix per canal (una variable per fila), pero es pot recablejar si cal.
function setvar(ch, name) {
    ch = Math.floor(ch);
    if (ch >= 0 && ch < 12) {
        channelVar[ch] = String(name);
        updateAllTags();
    }
}
