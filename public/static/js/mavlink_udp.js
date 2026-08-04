class UdpMavlink {
    constructor() {
        this.parser = new MavlinkFrameParser((msgId, payload, sysid, compid) => this._handleFrame(msgId, payload, sysid, compid));

        this.telemetry = { connected: false, source: 'udp' };
        this.logSeq = 0;
        this._statustextChunks = {};
        this.lastHeartbeatTime = 0;
        this._watchdogTimer = null;
        this._linkLossTimeoutMs = 5000;

        this._txSeq = 0;
        this._gcsSysId = 255;
        this._gcsCompId = 190;
        this.targetSystem = 1;
        this.targetComponent = 1;
        this._streamsRequested = false;
        this.rcChannels = [1500, 1500, 1000, 1500, 65535, 65535, 65535, 65535];
        this._rcOverrideTimer = null;

        this._remoteIp = null;
        this._remotePort = null;
        this._connected = false;
        this._gcsHeartbeatTimer = null;
        this._dataUnsubscribe = null;

        this._ackWaiters = new Map();
        this._paramWaiters = new Map();
        this._paramListWaiters = [];
        this._missionUpload = null;
        this._missionCountWaiter = null;
        this._missionItemWaiter = null;

        this.onTelemetry = null;
        this.onMavlog = null;
        this.onConnect = null;
        this.onDisconnect = null;
    }

    isLive() {
        return this.telemetry.connected && (Date.now() - this.lastHeartbeatTime) < this._linkLossTimeoutMs;
    }

    async connect(ip, port) {
        if (!window.udpBridge) throw new Error('udpBridge tidak tersedia -- pastikan preload.js sudah dimuat.');
        if (!ip || !port) throw new Error('IP dan Port wajib diisi.');

        const result = await window.udpBridge.connect(ip, parseInt(port, 10));
        if (!result || !result.ok) {
            throw new Error(result?.message || 'Gagal membuka UDP socket di main process.');
        }

        this._remoteIp = ip;
        this._remotePort = parseInt(port, 10);
        this._connected = true;
        this.telemetry.connected = true;
        this.lastHeartbeatTime = Date.now(); // grace period awal, sama seperti RadioMavlink

        // Dengarkan data masuk dari main process
        this._dataUnsubscribe = window.udpBridge.onData((bytes) => {
            for (const b of bytes) this.parser.feedByte(b);
        });

        this._startGcsHeartbeat();
        this._startWatchdog();
        this._startRcOverrideLoop();
        if (this.onConnect) this.onConnect();
    }

    async disconnect() {
        this._stopGcsHeartbeat();
        this._stopWatchdog();
        this._stopRcOverrideLoop();
        if (this._dataUnsubscribe) { this._dataUnsubscribe(); this._dataUnsubscribe = null; }
        if (window.udpBridge) {
            try { await window.udpBridge.disconnect(); } catch (e) {}
        }
        this._connected = false;
        this.telemetry.connected = false;
        this._streamsRequested = false;
        if (this.onDisconnect) this.onDisconnect();
    }

    _startGcsHeartbeat() {
        this._stopGcsHeartbeat();
        const sendGcsHeartbeat = () => {
            const buf = new ArrayBuffer(9);
            const dv = new DataView(buf);
            dv.setUint32(0, 0, true);      // custom_mode
            dv.setUint8(4, 6);              // type = MAV_TYPE_GCS
            dv.setUint8(5, 8);              // autopilot = MAV_AUTOPILOT_INVALID
            dv.setUint8(6, 0);              // base_mode
            dv.setUint8(7, 4);              // system_status = MAV_STATE_ACTIVE
            dv.setUint8(8, 3);               // mavlink_version
            this._sendFrame(0, Array.from(new Uint8Array(buf)), CRC_EXTRA[0]);
        };
        sendGcsHeartbeat(); // langsung kirim satu begitu connect, tidak nunggu 1 detik pertama
        this._gcsHeartbeatTimer = setInterval(sendGcsHeartbeat, 1000);
    }

    _stopGcsHeartbeat() {
        if (this._gcsHeartbeatTimer) { clearInterval(this._gcsHeartbeatTimer); this._gcsHeartbeatTimer = null; }
    }

    _startWatchdog() {
        this._stopWatchdog();
        this._watchdogTimer = setInterval(() => {
            if (this.telemetry.connected && !this.isLive()) {
                console.warn('[UdpMavlink] Heartbeat FC timeout — link dianggap putus.');
                this.telemetry.connected = false;
                if (this.onDisconnect) this.onDisconnect();
            }
        }, 1000);
    }

    _stopWatchdog() {
        if (this._watchdogTimer) { clearInterval(this._watchdogTimer); this._watchdogTimer = null; }
    }

    _startRcOverrideLoop() {
        this._stopRcOverrideLoop();
        this._rcOverrideTimer = setInterval(() => {
            if (this.telemetry.connected && this.rcChannels.some(ch => ch !== 65535)) {
                const payload = [this.targetSystem, this.targetComponent];
                for (let i = 0; i < 8; i++) {
                    const ch = this.rcChannels[i];
                    payload.push(ch & 0xFF, (ch >> 8) & 0xFF);
                }
                this._sendFrame(70, payload, CRC_EXTRA[70]).catch(()=>{});
            }
        }, 500);
    }

    _stopRcOverrideLoop() {
        if (this._rcOverrideTimer) { clearInterval(this._rcOverrideTimer); this._rcOverrideTimer = null; }
    }

    async _sendFrame(msgId, payloadBytes, crcExtra) {
        if (!this._connected || !window.udpBridge) return;
        const frame = buildMavlink1Frame(this._txSeq++, this._gcsSysId, this._gcsCompId, msgId, payloadBytes, crcExtra);
        try { window.udpBridge.send(frame); } catch (e) {}
    }

    async requestDataStreams(rateHz = STREAM_RATE_HZ) {
        for (const streamId of REQUESTED_STREAMS) {
            const rateBytes = [rateHz & 0xFF, (rateHz >> 8) & 0xFF];
            const payload = [...rateBytes, this.targetSystem, this.targetComponent, streamId, 1];
            await this._sendFrame(66, payload, CRC_EXTRA[66]);
            await new Promise(r => setTimeout(r, 50));
        }
        console.log(`[UdpMavlink] Streams requested @ ${rateHz}Hz`);
    }

    async sendCommandAck(command, result = 1) {
        const buf = new ArrayBuffer(3);
        const dv = new DataView(buf);
        dv.setUint16(0, command, true);
        dv.setUint8(2, result);
        await this._sendFrame(77, Array.from(new Uint8Array(buf)), CRC_EXTRA[77]);
    }

    async sendCommandLong(command, params = {}, waitAck = true, timeout = 3000) {
        const p = { p1:0, p2:0, p3:0, p4:0, p5:0, p6:0, p7:0, ...params };
        const payload = new Uint8Array(33);
        const dv = new DataView(payload.buffer);
        [p.p1, p.p2, p.p3, p.p4, p.p5, p.p6, p.p7].forEach((v,i) => dv.setFloat32(i*4, v, true));
        dv.setUint16(28, command, true);
        dv.setUint8(30, this.targetSystem);
        dv.setUint8(31, this.targetComponent);
        dv.setUint8(32, 0);

        if (!waitAck) {
            await this._sendFrame(76, Array.from(payload), CRC_EXTRA[76]);
            return { ok: true, result: null };
        }
        const ackPromise = new Promise((resolve) => {
            const timer = setTimeout(() => {
                this._ackWaiters.delete(command);
                resolve({ ok: false, result: 'timeout' });
            }, timeout);
            this._ackWaiters.set(command, { resolve, timer });
        });
        await this._sendFrame(76, Array.from(payload), CRC_EXTRA[76]);
        return ackPromise;
    }

    async sendCommandInt(command, params = {}, waitAck = true, timeout = 3000) {
        const p = { param1:0, param2:0, param3:0, param4:0, x:0, y:0, z:0, frame:0, ...params };
        const buf = new ArrayBuffer(1+1+1+2+1+1+4*4+4+4+4);
        const dv = new DataView(buf);
        let off=0;
        dv.setUint8(off, this.targetSystem); off++;
        dv.setUint8(off, this.targetComponent); off++;
        dv.setUint8(off, p.frame); off++;
        dv.setUint16(off, command, true); off+=2;
        dv.setUint8(off, 0); off++;
        dv.setUint8(off, 0); off++;
        dv.setFloat32(off, p.param1, true); off+=4;
        dv.setFloat32(off, p.param2, true); off+=4;
        dv.setFloat32(off, p.param3, true); off+=4;
        dv.setFloat32(off, p.param4, true); off+=4;
        dv.setInt32(off, p.x, true); off+=4;
        dv.setInt32(off, p.y, true); off+=4;
        dv.setFloat32(off, p.z, true); off+=4;
        const payload = Array.from(new Uint8Array(buf));
        if (!waitAck) {
            await this._sendFrame(75, payload, CRC_EXTRA[75]);
            return { ok: true, result: null };
        }
        const ackPromise = new Promise((resolve) => {
            const timer = setTimeout(() => {
                this._ackWaiters.delete(command);
                resolve({ ok: false, result: 'timeout' });
            }, timeout);
            this._ackWaiters.set(command, { resolve, timer });
        });
        await this._sendFrame(75, payload, CRC_EXTRA[75]);
        return ackPromise;
    }

    _encodeParamId(name) {
        const bytes = new Uint8Array(16);
        bytes.set(new TextEncoder().encode(name).slice(0, 16));
        return Array.from(bytes);
    }

    async getParam(name, timeout = 3000) {
        const payload = [0xFF, 0xFF, this.targetSystem, this.targetComponent, ...this._encodeParamId(name)];
        const promise = new Promise((resolve, reject) => {
            const timer = setTimeout(() => {
                this._paramWaiters.delete(name);
                reject(new Error(`Timeout getParam ${name}`));
            }, timeout);
            this._paramWaiters.set(name, { resolve: (v) => { clearTimeout(timer); resolve(v); }, timer });
        });
        await this._sendFrame(20, payload, CRC_EXTRA[20]);
        return promise;
    }

    async setParam(name, value, paramType = 9, timeout = 3000) {
        const buf = new ArrayBuffer(23);
        const dv = new DataView(buf);
        dv.setFloat32(0, value, true);
        dv.setUint8(4, this.targetSystem);
        dv.setUint8(5, this.targetComponent);
        this._encodeParamId(name).forEach((b, i) => dv.setUint8(6 + i, b));
        dv.setUint8(22, paramType);
        const payload = Array.from(new Uint8Array(buf));

        const promise = new Promise((resolve) => {
            const timer = setTimeout(() => {
                this._paramWaiters.delete(name);
                resolve({ ok: false, reason: 'timeout' });
            }, timeout);
            this._paramWaiters.set(name, {
                resolve: (v) => { clearTimeout(timer); resolve({ ok: true, value: v }); },
                timer
            });
        });
        await this._sendFrame(23, payload, CRC_EXTRA[23]);
        return promise;
    }

    async getAllParams(onProgress, timeout = 3000) {
        return new Promise((resolve, reject) => {
            const waiter = { resolve, reject, params: {}, total: null, count: 0, onProgress, timer: null };
            waiter._resetTimer = () => {
                if (waiter.timer) clearTimeout(waiter.timer);
                waiter.timer = setTimeout(() => {
                    const idx = this._paramListWaiters.indexOf(waiter);
                    if (idx !== -1) this._paramListWaiters.splice(idx, 1);
                    const receivedCount = Object.keys(waiter.params).length;
                    if (receivedCount > 0) {
                        console.warn(`[UdpMavlink] Timeout getAllParams, mengembalikan ${receivedCount} parameter yang diterima`);
                        resolve(waiter.params);
                    } else {
                        reject(new Error(`Timeout getAllParams (${waiter.count}/${waiter.total ?? '?'} diterima)`));
                    }
                }, timeout);
            };
            waiter._resetTimer();
            this._paramListWaiters.push(waiter);
            this._sendFrame(21, [this.targetSystem, this.targetComponent], CRC_EXTRA[21]);
        });
    }

    async uploadMission(items, timeout = 10000) {
        const count = items.length;
        if (count === 0) throw new Error('Mission kosong, tidak ada yang di-upload');
        if (this._missionUpload) throw new Error('Upload mission sedang berlangsung, tunggu selesai');

        return new Promise((resolve, reject) => {
            const state = { items, resolve, reject, timer: null };
            state.resetTimer = () => {
                if (state.timer) clearTimeout(state.timer);
                state.timer = setTimeout(() => {
                    this._missionUpload = null;
                    reject(new Error('Upload mission timeout -- FC berhenti meminta item'));
                }, timeout);
            };
            state.resetTimer();
            this._missionUpload = state;

            const countPayload = [count & 0xFF, (count >> 8) & 0xFF, this.targetSystem, this.targetComponent];
            this._sendFrame(44, countPayload, CRC_EXTRA[44]);
        });
    }

    _buildMissionItemPayload(seq, item) {
        const buf = new ArrayBuffer(37);
        const dv = new DataView(buf);
        dv.setFloat32(0, item.param1 || 0, true);
        dv.setFloat32(4, item.param2 || 0, true);
        dv.setFloat32(8, item.param3 || 0, true);
        dv.setFloat32(12, item.param4 || 0, true);
        dv.setInt32(16, item.x || 0, true);
        dv.setInt32(20, item.y || 0, true);
        dv.setFloat32(24, item.z || 0, true);
        dv.setUint16(28, seq, true);
        dv.setUint16(30, item.command, true);
        dv.setUint8(32, this.targetSystem);
        dv.setUint8(33, this.targetComponent);
        dv.setUint8(34, item.frame ?? 3);
        dv.setUint8(35, item.current || 0);
        dv.setUint8(36, item.autocontinue ?? 1);
        return Array.from(new Uint8Array(buf));
    }

    async downloadMission(timeout = 5000, maxRetries = 3) {
        let count;
        try {
            count = await new Promise((resolve, reject) => {
                const timer = setTimeout(() => {
                    console.warn('[UdpMavlink] Timeout MISSION_REQUEST_LIST, asumsikan tidak ada misi');
                    resolve(0);
                }, timeout);
                this._missionCountWaiter = { resolve: (c) => { clearTimeout(timer); resolve(c); } };
                this._sendFrame(43, [this.targetSystem, this.targetComponent], CRC_EXTRA[43]);
            });
        } catch (e) {
            console.warn('[UdpMavlink] Gagal mendapatkan MISSION_COUNT:', e);
            return [];
        }

        if (count === 0) return [];

        const items = [];
        for (let seq = 0; seq < count; seq++) {
            let item = null;
            for (let attempt = 0; attempt < maxRetries && !item; attempt++) {
                try {
                    item = await new Promise((resolve, reject) => {
                        const timer = setTimeout(() => reject(new Error('timeout')), timeout);
                        this._missionItemWaiter = { seq, resolve: (it) => { clearTimeout(timer); resolve(it); } };
                        const payload = [seq & 0xFF, (seq >> 8) & 0xFF, this.targetSystem, this.targetComponent];
                        this._sendFrame(51, payload, CRC_EXTRA[51]);
                    });
                } catch (e) {
                    console.warn(`[UdpMavlink] Item seq=${seq} timeout, percobaan ${attempt + 1}/${maxRetries}`);
                }
            }
            if (!item) throw new Error(`Gagal ambil mission item seq=${seq} setelah ${maxRetries} percobaan`);
            items.push(item);
        }

        await this._sendFrame(47, [this.targetSystem, this.targetComponent, 0], CRC_EXTRA[47]);
        return items;
    }

    async armDisarm(arm) { return this.sendCommandLong(400, { p1: arm ? 1 : 0 }); }
    async takeoff(alt = 10) { return this.sendCommandLong(22, { p7: alt }); }
    async rtl() { return this.sendCommandLong(20); }
    async land() { return this.sendCommandLong(21); }
    async setMode(modeCode) {
        const MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1;
        return this.sendCommandLong(176, { p1: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, p2: modeCode });
    }
    async reboot() { return this.sendCommandLong(246, { p1: 1 }); }
    async setHome(lat, lon, alt = 0) { return this.sendCommandLong(179, { p1: 0, p5: lat, p6: lon, p7: alt }); }
    async setServo(channel, pwm) { return this.sendCommandLong(183, { p1: channel, p2: pwm }); }
    async sendRcOverride(chIndex, pwmValue) {
        if (chIndex < 0 || chIndex > 7) {
            throw new Error(`Channel index ${chIndex} tidak valid. Harus antara 0-7.`);
        }
        this.rcChannels[chIndex] = pwmValue;

        const payload = [this.targetSystem, this.targetComponent];
        for (let i = 0; i < 8; i++) {
            const ch = this.rcChannels[i];
            payload.push(ch & 0xFF, (ch >> 8) & 0xFF);
        }
        await this._sendFrame(70, payload, CRC_EXTRA[70]);
    }

    async calibrateMag(compassId = -1) { return this.sendCommandLong(241, { p3: 1, p4: compassId >= 0 ? compassId : 0 }); }
    async calibrateAccel(retries = 0) { return this.sendCommandLong(241, { p1: 1, p2: retries }); }
    async calibrateGyro() { return this.sendCommandLong(241, { p5: 1 }); }
    async calibrateLevel() { return this.sendCommandLong(241, { p6: 1 }); }

    async getPID(axis) {
        const axisMap = { roll:'ROLL', pitch:'PITCH', yaw:'YAW' };
        const prefix = axisMap[axis.toLowerCase()];
        if (!prefix) throw new Error(`Axis ${axis} tidak didukung.`);
        const P = await this.getParam(`ATC_RAT_${prefix}_P`);
        const I = await this.getParam(`ATC_RAT_${prefix}_I`);
        const D = await this.getParam(`ATC_RAT_${prefix}_D`);
        return { P, I, D };
    }
    async setPID(axis, pid) {
        const axisMap = { roll:'ROLL', pitch:'PITCH', yaw:'YAW' };
        const prefix = axisMap[axis.toLowerCase()];
        if (!prefix) throw new Error(`Axis ${axis} tidak didukung.`);
        await this.setParam(`ATC_RAT_${prefix}_P`, pid.P);
        await this.setParam(`ATC_RAT_${prefix}_I`, pid.I);
        await this.setParam(`ATC_RAT_${prefix}_D`, pid.D);
        return true;
    }

    _handleFrame(msgId, payload, sysid, compid) {
        if (msgId === 0 && sysid !== this._gcsSysId) {
            this.lastHeartbeatTime = Date.now();
            if (!this._streamsRequested) {
                this.targetSystem = sysid;
                this.targetComponent = compid;
                this._streamsRequested = true;
                this.requestDataStreams().catch(e => console.warn('[UdpMavlink] requestDataStreams gagal:', e));
            }
        }

        const decoder = DECODERS[msgId];
        if (!decoder) return;

        const paddedPayload = new Uint8Array(255);
        paddedPayload.set(payload, 0);

        let result;
        try {
            result = decoder(paddedPayload);
        } catch (e) {
            console.error(`[UdpMavlink] Gagal memproses pesan MAVLink ID ${msgId}:`, e);
            return;
        }

        if (result._kind === 'telemetry') {
            delete result._kind;
            Object.assign(this.telemetry, result);
            this.telemetry.connected = true;
            if (this.onTelemetry) this.onTelemetry({ ...this.telemetry });
        } else if (result._kind === 'statustext') {
            this._handleStatustext(result);
        } else if (result._kind === 'command_ack') {
            this._handleCommandAck(result.command, result.result);
        } else if (result._kind === 'param_value') {
            this._handleParamValue(result);
        } else if (result._kind === 'mission_request') {
            this._handleMissionRequest(result.seq);
        } else if (result._kind === 'mission_count') {
            this._handleMissionCount(result.count);
        } else if (result._kind === 'mission_item') {
            this._handleMissionItem(result);
        } else if (result._kind === 'mission_ack') {
            this._handleMissionAck(result.result);
        } else if (result._kind === 'distance_sensor') {
            const RANGEFINDER_TYPE_MAP = { 0: 'LASER', 1: 'ULTRASOUND', 2: 'INFRARED', 3: 'RADAR', 4: 'UNKNOWN' };
            const isHealthy = result.signal_quality !== 1
                && result.current_distance > result.min_distance
                && result.current_distance < result.max_distance;
            this.telemetry.rangefinder_distance = result.current_distance / 100;
            this.telemetry.rangefinder_healthy = isHealthy;
            this.telemetry.rangefinder_type = RANGEFINDER_TYPE_MAP[result.type] || 'UNKNOWN';
            if (this.onTelemetry) this.onTelemetry({ ...this.telemetry });
        } else if (result._kind === 'radio_status') {
            this.telemetry.rssi = Math.round((result.rssi / 254) * 100);
            this.telemetry.remrssi = Math.round((result.remrssi / 254) * 100);
            this.telemetry.radio_noise = result.noise;
            this.telemetry.radio_txbuf = result.txbuf;
            if (this.onTelemetry) this.onTelemetry({ ...this.telemetry });
        }
    }

    _handleCommandAck(command, result) {
        const waiter = this._ackWaiters.get(command);
        if (waiter) {
            this._ackWaiters.delete(command);
            clearTimeout(waiter.timer);
            waiter.resolve({ ok: result === 0, result });
        }
    }

    _handleParamValue({ name, value, paramIndex, paramCount }) {
        const waiter = this._paramWaiters.get(name);
        if (waiter) {
            this._paramWaiters.delete(name);
            waiter.resolve(value);
            return;
        }
        if (this._paramListWaiters.length > 0) {
            const w = this._paramListWaiters[0];
            w.params[name] = value;
            w.count = Object.keys(w.params).length;
            w.total = paramCount;
            w._resetTimer();
            if (w.onProgress) w.onProgress(w.count, w.total, name);
            if (w.count >= w.total) {
                clearTimeout(w.timer);
                this._paramListWaiters.shift();
                w.resolve(w.params);
            }
        }
    }

    _handleMissionRequest(seq) {
        const state = this._missionUpload;
        if (!state) return;
        const item = state.items[seq];
        if (!item) {
            console.warn(`[UdpMavlink] FC minta item seq=${seq} tapi mission cuma punya ${state.items.length} item`);
            return;
        }
        state.resetTimer();
        const payload = this._buildMissionItemPayload(seq, item);
        this._sendFrame(73, payload, CRC_EXTRA[73]);
    }

    _handleMissionCount(count) {
        if (this._missionCountWaiter) {
            const w = this._missionCountWaiter;
            this._missionCountWaiter = null;
            w.resolve(count);
        }
    }

    _handleMissionItem(item) {
        if (this._missionItemWaiter && this._missionItemWaiter.seq === item.seq) {
            const w = this._missionItemWaiter;
            this._missionItemWaiter = null;
            w.resolve(item);
        }
    }

    _handleMissionAck(result) {
        const state = this._missionUpload;
        if (!state) return;
        clearTimeout(state.timer);
        this._missionUpload = null;
        if (result === 0) {
            state.resolve({ ok: true });
        } else {
            state.resolve({ ok: false, message: `Mission ditolak FC, kode error MAV_MISSION_RESULT=${result}` });
        }
    }

    _handleStatustext({ severity, text, msgId, chunkSeq }) {
        let fullText = text;
        if (!(msgId === 0 && chunkSeq === 0)) {
            const buf = this._statustextChunks[msgId] || {};
            buf[chunkSeq] = text;
            this._statustextChunks[msgId] = buf;
            if (text.length < 50) {
                const keys = Object.keys(buf).map(Number).sort((a,b)=>a-b);
                fullText = keys.map(k => buf[k]).join('');
                delete this._statustextChunks[msgId];
            } else {
                return;
            }
        }
        this.logSeq++;
        const logItem = { seq: this.logSeq, text: fullText, severity, server_time: Date.now()/1000, source: 'udp' };
        if (this.onMavlog) this.onMavlog(logItem);
    }
}

window.UdpMavlink = UdpMavlink;
console.log('[UdpMavlink] udp-mavlink.js loaded');