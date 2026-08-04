function mavlinkCrc16Update(crc, byte) {
    let tmp = byte ^ (crc & 0xFF);
    tmp = (tmp ^ (tmp << 4)) & 0xFF;
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF;
}

const CRC_EXTRA = {
    0: 50,     // HEARTBEAT
    1: 124,    // SYS_STATUS
    20: 214,   // PARAM_REQUEST_READ
    21: 159,   // PARAM_REQUEST_LIST
    22: 220,   // PARAM_VALUE
    23: 168,   // PARAM_SET
    24: 24,    // GPS_RAW_INT
    27: 144,   // RAW_IMU
    30: 39,    // ATTITUDE
    33: 104,   // GLOBAL_POSITION_INT
    43: 132,   // MISSION_REQUEST_LIST
    44: 221,   // MISSION_COUNT
    47: 153,   // MISSION_ACK
    51: 196,   // MISSION_REQUEST_INT
    40: 230,   // MISSION_REQUEST (legacy)
    66: 148,   // REQUEST_DATA_STREAM
    70: 124,   // RC_CHANNELS_OVERRIDE
    73: 38,    // MISSION_ITEM_INT
    74: 20,    // VFR_HUD
    76: 152,   // COMMAND_LONG
    77: 143,   // COMMAND_ACK
    109: 185,  // RADIO_STATUS
    132: 85,   // DISTANCE_SENSOR
    253: 83,   // STATUSTEXT
};

const REQUESTED_STREAMS = [1, 2, 3, 6, 10, 11, 12];
const STREAM_RATE_HZ = 5;

function buildMavlink1Frame(seq, sysid, compid, msgId, payloadBytes, crcExtra) {
    const len = payloadBytes.length;
    const beforeCrc = [len, seq & 0xFF, sysid, compid, msgId, ...payloadBytes];
    let crc = 0xFFFF;
    for (const b of beforeCrc) crc = mavlinkCrc16Update(crc, b);
    crc = mavlinkCrc16Update(crc, crcExtra);
    return new Uint8Array([0xFE, ...beforeCrc, crc & 0xFF, (crc >> 8) & 0xFF]);
}

const PARSE_STATE = {
    IDLE: 0, GOT_STX: 1, GOT_LENGTH: 2, GOT_INCOMPAT: 3, GOT_COMPAT: 4,
    GOT_SEQ: 5, GOT_SYSID: 6, GOT_COMPID: 7, GOT_MSGID: 8, GOT_PAYLOAD: 9,
};

class MavlinkFrameParser {
    constructor(onFrame) {
        this.onFrame = onFrame;
        this._reset();
    }

    _reset() {
        this.state = PARSE_STATE.IDLE;
        this.isV2 = false;
        this.buf = [];
        this.payloadLen = 0;
        this.msgId = 0;
        this.crcAccum = 0xFFFF;
        this.headerLen = 0;
        this._sysid = 0;
        this._compid = 0;
    }

    feedByte(byte) {
        switch (this.state) {
            case PARSE_STATE.IDLE:
                if (byte === 0xFE) { this.isV2 = false; this._startFrame(); }
                else if (byte === 0xFD) { this.isV2 = true; this._startFrame(); }
                break;
            case PARSE_STATE.GOT_STX:
                this.payloadLen = byte;
                this.buf.push(byte);
                this.state = PARSE_STATE.GOT_LENGTH;
                break;
            case PARSE_STATE.GOT_LENGTH:
                if (this.isV2) {
                    this.buf.push(byte);
                    this.state = PARSE_STATE.GOT_INCOMPAT;
                } else {
                    this.buf.push(byte);
                    this.state = PARSE_STATE.GOT_SEQ;
                }
                break;
            case PARSE_STATE.GOT_INCOMPAT:
                this.buf.push(byte);
                this.state = PARSE_STATE.GOT_COMPAT;
                break;
            case PARSE_STATE.GOT_COMPAT:
                this.buf.push(byte);
                this.state = PARSE_STATE.GOT_SEQ;
                break;
            case PARSE_STATE.GOT_SEQ:
                this.buf.push(byte);
                this._sysid = byte;
                this.state = PARSE_STATE.GOT_SYSID;
                break;
            case PARSE_STATE.GOT_SYSID:
                this.buf.push(byte);
                this._compid = byte;
                this.state = PARSE_STATE.GOT_COMPID;
                break;
            case PARSE_STATE.GOT_COMPID:
                this.buf.push(byte);
                if (this.isV2) {
                    this.msgIdBytes = [byte];
                    this._msgIdByteCount = 1;
                    this.state = PARSE_STATE.GOT_MSGID;
                } else {
                    this.msgId = byte;
                    this.state = PARSE_STATE.GOT_MSGID;
                    this._payloadStart = this.buf.length;
                }
                break;
            case PARSE_STATE.GOT_MSGID:
                if (this.isV2 && this._msgIdByteCount < 3) {
                    this.buf.push(byte);
                    this.msgIdBytes.push(byte);
                    this._msgIdByteCount++;
                    if (this._msgIdByteCount === 3) {
                        this.msgId = this.msgIdBytes[0] | (this.msgIdBytes[1] << 8) | (this.msgIdBytes[2] << 16);
                        this._payloadStart = this.buf.length;
                    }
                    break;
                }
                this.buf.push(byte);
                if (this.buf.length - this._payloadStart >= this.payloadLen) {
                    this.state = PARSE_STATE.GOT_PAYLOAD;
                    this._crcBytesNeeded = 2;
                    this._crcBytes = [];
                }
                break;
            case PARSE_STATE.GOT_PAYLOAD:
                this._crcBytes.push(byte);
                if (this._crcBytes.length === 2) {
                    this._finishFrame();
                    this._reset();
                }
                break;
        }
    }

    _startFrame() {
        this.buf = [];
        this.state = PARSE_STATE.GOT_STX;
    }

    _finishFrame() {
        const payload = this.buf.slice(this._payloadStart, this._payloadStart + this.payloadLen);
        const crcExtra = CRC_EXTRA[this.msgId];
        if (crcExtra === undefined) return;
        let crc = 0xFFFF;
        for (const b of this.buf) crc = mavlinkCrc16Update(crc, b);
        crc = mavlinkCrc16Update(crc, crcExtra);
        const receivedCrc = this._crcBytes[0] | (this._crcBytes[1] << 8);
        if (crc !== receivedCrc) return;
        this.onFrame(this.msgId, payload, this._sysid, this._compid);
    }
}

function readF32(dv, off) { return dv.getFloat32(off, true); }
function readU32(dv, off) { return dv.getUint32(off, true); }
function readI32(dv, off) { return dv.getInt32(off, true); }
function readU16(dv, off) { return dv.getUint16(off, true); }

const MAV_TYPE_ROVER = 10;
const MAV_TYPE_COPTER_SET = new Set([2, 13, 14]);
const COPTER_MODE_MAP = { 0:'STABILIZE',2:'ALT_HOLD',3:'AUTO',4:'GUIDED',5:'LOITER',6:'RTL',9:'LAND',16:'POSHOLD',20:'GUIDED_NOGPS' };
const ROVER_MODE_MAP = { 0:'MANUAL',3:'STEERING',4:'HOLD',5:'LOITER',10:'AUTO',11:'RTL',15:'GUIDED' };

const DECODERS = {
    0: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        const custom_mode = readU32(dv, 0);
        const type = dv.getUint8(4);
        const base_mode = dv.getUint8(6);
        const armed = (base_mode & 0x80) !== 0;
        const vehicle_type = type === MAV_TYPE_ROVER ? 'ROVER' : (MAV_TYPE_COPTER_SET.has(type) ? 'COPTER' : 'UNKNOWN');
        const modeMap = vehicle_type === 'ROVER' ? ROVER_MODE_MAP : COPTER_MODE_MAP;
        return { _kind:'telemetry', armed, vehicle_type, mode: modeMap[custom_mode] || `MODE_${custom_mode}` };
    },
    1: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return { _kind:'telemetry', battery_voltage: readU16(dv,14)/1000, battery_current: dv.getInt16(16,true)/100, battery_remaining: dv.getInt8(30) };
    },
    22: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        const value = dv.getFloat32(0, true);
        const paramCount = dv.getUint16(4, true);
        const paramIndex = dv.getUint16(6, true);
        let name = '';
        for (let i = 8; i < 24; i++) { const c = dv.getUint8(i); if (c === 0) break; name += String.fromCharCode(c); }
        const paramType = dv.getUint8(24);
        return { _kind:'param_value', name, value, paramIndex, paramCount, paramType };
    },
    27: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return {
            _kind: 'telemetry',
            accel_x: dv.getInt16(8, true) / 1000, accel_y: dv.getInt16(10, true) / 1000, accel_z: dv.getInt16(12, true) / 1000,
            gyro_x: dv.getInt16(14, true) / 1000, gyro_y: dv.getInt16(16, true) / 1000, gyro_z: dv.getInt16(18, true) / 1000,
            mag_field: Math.sqrt(dv.getInt16(20, true)**2 + dv.getInt16(22, true)**2 + dv.getInt16(24, true)**2) / 1000
        };
    },
    30: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return { _kind:'telemetry', attitude: { roll: readF32(dv,4), pitch: readF32(dv,8), yaw: readF32(dv,12) } };
    },
    33: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return { _kind:'telemetry', latitude: readI32(dv,4)/1e7, longitude: readI32(dv,8)/1e7, altitude: readI32(dv,16)/1000, heading: readU16(dv,26)/100 };
    },
    74: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return { _kind:'telemetry', speed: readF32(dv,4), climb: readF32(dv,12), throttle: dv.getUint16(18, true) };
    },
    24: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return { _kind:'telemetry', gps_fix_type: dv.getUint8(28), satellites_visible: dv.getUint8(29) };
    },
    40: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return { _kind:'mission_request', seq: dv.getUint16(0, true) };
    },
    51: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return { _kind:'mission_request', seq: dv.getUint16(0, true) };
    },
    44: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return { _kind:'mission_count', count: dv.getUint16(0, true) };
    },
    47: (bytes) => {
        return { _kind:'mission_ack', result: bytes[2] };
    },
    73: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return {
            _kind: 'mission_item',
            param1: dv.getFloat32(0, true), param2: dv.getFloat32(4, true),
            param3: dv.getFloat32(8, true), param4: dv.getFloat32(12, true),
            x: dv.getInt32(16, true), y: dv.getInt32(20, true), z: dv.getFloat32(24, true),
            seq: dv.getUint16(28, true), command: dv.getUint16(30, true),
            frame: dv.getUint8(34), current: dv.getUint8(35), autocontinue: dv.getUint8(36),
        };
    },
    77: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return { _kind:'command_ack', command: readU16(dv,0), result: dv.getUint8(2) };
    },
    132: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return {
            _kind: 'distance_sensor',
            min_distance: dv.getUint16(4, true),
            max_distance: dv.getUint16(6, true),
            current_distance: dv.getUint16(8, true),
            type: dv.getUint8(10),
            signal_quality: dv.getUint8(38),
        };
    },
    109: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return {
            _kind: 'radio_status',
            rxerrors: dv.getUint16(0, true),
            fixedErr: dv.getUint16(2, true),
            rssi: dv.getUint8(4),
            remrssi: dv.getUint8(5),
            txbuf: dv.getUint8(6),
            noise: dv.getUint8(7),
            remnoise: dv.getUint8(8),
        };
    },
    253: (bytes) => {
        const severity = bytes[0];
        let end = 1;
        while (end < 51 && bytes[end] !== 0) end++;
        const text = new TextDecoder('utf-8').decode(new Uint8Array(bytes.slice(1, end)));
        const msgId = bytes.length >= 53 ? (bytes[51] | (bytes[52] << 8)) : 0;
        const chunkSeq = bytes.length >= 54 ? bytes[53] : 0;
        return { _kind:'statustext', severity, text, msgId, chunkSeq };
    },
};

console.log('[MavlinkProtocol] mavlink-protocol.js loaded (shared parser/decoder)');