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
    45: 251,   // MISSION_CLEAR_ALL
    47: 153,   // MISSION_ACK
    242: 104,  // HOME_POSITION
    51: 196,   // MISSION_REQUEST_INT
    40: 230,   // MISSION_REQUEST (legacy)
    65: 118,
    66: 148,   // REQUEST_DATA_STREAM
    70: 124,   // RC_CHANNELS_OVERRIDE
    73: 38,    // MISSION_ITEM_INT
    74: 20,    // VFR_HUD
    75: 158,   // COMMAND_INT
    76: 152,   // COMMAND_LONG
    77: 143,   // COMMAND_ACK
    109: 185,  // RADIO_STATUS
    132: 85,   // DISTANCE_SENSOR
    191: 92,   // MAG_CAL_PROGRESS
    192: 36,   // MAG_CAL_REPORT
    253: 83,   // STATUSTEXT
    160: 78,   // FENCE_POINT
    161: 68,   // FENCE_FETCH_POINT
};

const MSG_LENGTHS = {
    0: 9, 1: 31, 20: 20, 21: 2, 22: 25, 23: 23, 24: 30, 27: 29, 30: 28, 
    33: 28, 40: 4, 43: 3, 44: 5, 47: 4, 51: 5, 65: 42, 66: 6, 70: 18, 
    73: 38, 74: 20, 75: 35, 76: 33, 77: 3, 109: 9, 132: 14, 191: 27, 
    192: 44, 253: 54, 160: 12, 161: 3, 45: 2, 242: 60
};

const REQUESTED_STREAMS = [1, 2, 3, 6, 10, 11, 12];
const STREAM_RATE_HZ = 8;

function buildMavlink1Frame(seq, sysid, compid, msgId, payloadBytes, crcExtra) {
    const len = payloadBytes.length;
    const beforeCrc = [len, seq & 0xFF, sysid, compid, msgId, ...payloadBytes];
    let crc = 0xFFFF;
    for (const b of beforeCrc) crc = mavlinkCrc16Update(crc, b);
    crc = mavlinkCrc16Update(crc, crcExtra);
    return new Uint8Array([0xFE, ...beforeCrc, crc & 0xFF, (crc >> 8) & 0xFF]);
}

function buildMavlink2Frame(seq, sysid, compid, msgId, payloadBytes, crcExtra) {
    const len = payloadBytes.length;
    const incompatFlags = 0x00;
    const compatFlags = 0x00;
    const msgIdLo = msgId & 0xFF;
    const msgIdMid = (msgId >> 8) & 0xFF;
    const msgIdHi = (msgId >> 16) & 0xFF;
    const beforeCrc = [
        len, incompatFlags, compatFlags, seq & 0xFF, sysid, compid,
        msgIdLo, msgIdMid, msgIdHi, ...payloadBytes
    ];
    let crc = 0xFFFF;
    for (const b of beforeCrc) crc = mavlinkCrc16Update(crc, b);
    crc = mavlinkCrc16Update(crc, crcExtra);
    return new Uint8Array([0xFD, ...beforeCrc, crc & 0xFF, (crc >> 8) & 0xFF]);
}

const MAV_MISSION_TYPE_MISSION = 0;
const MAV_MISSION_TYPE_FENCE = 1;
const MAV_CMD_NAV_FENCE_RETURN_POINT = 5000;
const MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001;
const MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002;
const MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION = 5003;
const MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = 5004;

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
        if (crcExtra === undefined) {
            if (typeof window !== 'undefined' && window.MAVLINK_DEBUG_DROPS) {
                console.warn(`[MavlinkParser] DROP msgId=${this.msgId}: unknown CRC_EXTRA (belum ada di map)`);
            }
            return;
        }
        let crc = 0xFFFF;
        for (const b of this.buf) crc = mavlinkCrc16Update(crc, b);
        crc = mavlinkCrc16Update(crc, crcExtra);
        const receivedCrc = this._crcBytes[0] | (this._crcBytes[1] << 8);
        if (crc !== receivedCrc) {
            if (typeof window !== 'undefined' && window.MAVLINK_DEBUG_DROPS) {
                console.warn(`[MavlinkParser] DROP msgId=${this.msgId}: CRC mismatch (dihitung=0x${crc.toString(16)}, diterima=0x${receivedCrc.toString(16)}, payloadLen=${this.payloadLen}, v2=${this.isV2})`);
            }
            return;
        }
        this.onFrame(this.msgId, payload, this._sysid, this._compid);
    }
}

function readF32(dv, off) { return dv.getFloat32(off, true); }
function readU32(dv, off) { return dv.getUint32(off, true); }
function readI32(dv, off) { return dv.getInt32(off, true); }
function readU16(dv, off) { return dv.getUint16(off, true); }
function readI16(dv, off) { return dv.getInt16(off, true); }

const MAV_TYPE_ROVER = 10;
const MAV_TYPE_COPTER_SET = new Set([2, 13, 14, 19, 20, 21]);
const COPTER_MODE_MAP = { 0:'STABILIZE',2:'ALT_HOLD',3:'AUTO',4:'GUIDED',5:'LOITER',6:'RTL',9:'LAND',16:'POSHOLD',20:'GUIDED_NOGPS' };
const ROVER_MODE_MAP = { 0:'MANUAL',3:'STEERING',4:'HOLD',5:'LOITER',10:'AUTO',11:'RTL',15:'GUIDED' };
const MAV_CMD_NAV_TAKEOFF = 22;

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
        const lat = readI32(dv, 4) / 1e7;
        const lon = readI32(dv, 8) / 1e7;
        const alt = readI32(dv, 12) / 1000;        
        const relativeAlt = readI32(dv, 16) / 1000;
        const heading = readI16(dv, 26) / 100;
        return { _kind: 'telemetry', latitude: lat, longitude: lon, altitude: alt, relativeAlt, heading };
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
    242: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return {
            _kind: 'telemetry',
            home_lat: readI32(dv, 0) / 1e7,
            home_lon: readI32(dv, 4) / 1e7,
            home_alt: readI32(dv, 8) / 1000,
        };
    },
    51: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        const missionType = bytes.length > 4 ? bytes[4] : 0;
        return { _kind:'mission_request', seq: dv.getUint16(0, true), missionType };
    },
    44: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        const missionType = bytes.length > 4 ? bytes[4] : 0;
        return { _kind:'mission_count', count: dv.getUint16(0, true), missionType };
    },
    47: (bytes) => {
        const missionType = bytes.length > 3 ? bytes[3] : 0;
        return { _kind:'mission_ack', result: bytes[2], missionType };
    },

    65: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return {
            _kind: 'telemetry',
            rc1_in: dv.getUint16(4, true),
            rc2_in: dv.getUint16(6, true),
            rc3_in: dv.getUint16(8, true),
            rc4_in: dv.getUint16(10, true),
            rc5_in: dv.getUint16(12, true),
            rc6_in: dv.getUint16(14, true),
            rc7_in: dv.getUint16(16, true),
            rc8_in: dv.getUint16(18, true),
        }
    },

    73: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        const missionType = bytes.length > 37 ? bytes[37] : 0;
        return {
            _kind: 'mission_item',
            param1: dv.getFloat32(0, true), param2: dv.getFloat32(4, true),
            param3: dv.getFloat32(8, true), param4: dv.getFloat32(12, true),
            x: dv.getInt32(16, true), y: dv.getInt32(20, true), z: dv.getFloat32(24, true),
            seq: dv.getUint16(28, true), command: dv.getUint16(30, true),
            frame: dv.getUint8(34), current: dv.getUint8(35), autocontinue: dv.getUint8(36),
            missionType,
        };
    },
    77: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return { _kind:'command_ack', command: readU16(dv,0), result: dv.getUint8(2) };
    },
    76: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return {
            _kind: 'command_long',
            param1: dv.getFloat32(0, true), param2: dv.getFloat32(4, true),
            param3: dv.getFloat32(8, true), param4: dv.getFloat32(12, true),
            param5: dv.getFloat32(16, true), param6: dv.getFloat32(20, true),
            param7: dv.getFloat32(24, true),
            command: readU16(dv, 28),
            targetSystem: dv.getUint8(30), targetComponent: dv.getUint8(31),
            confirmation: dv.getUint8(32),
        };
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
    191: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return {
            _kind: 'mag_cal_progress',
            compass_id: dv.getUint8(12),
            completion_pct: dv.getUint8(16)
        };
    },
    192: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return {
            _kind: 'mag_cal_report',
            fitness: dv.getFloat32(0, true),
            compass_id: dv.getUint8(40),
            cal_status: dv.getUint8(42)
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
    160: (bytes) => {
        const dv = new DataView(new Uint8Array(bytes).buffer);
        return {
            _kind: 'fence_point',
            lat: dv.getInt32(0, true),
            lon: dv.getInt32(4, true),
            target_system: dv.getUint8(8),
            target_component: dv.getUint8(9),
            idx: dv.getUint8(10),
            count: dv.getUint8(11),
        };
    },
};

console.log('[MavlinkProtocol] mavlink-protocol.js loaded (shared parser/decoder)');