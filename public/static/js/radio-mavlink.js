class RadioMavlink {
    constructor() {
        this.port = null;
        this.reader = null;
        this.keepReading = false;
        this.parser = new MavlinkFrameParser((msgId, payload, sysid, compid) => this._handleFrame(msgId, payload, sysid, compid));

        this.telemetry = { connected: false, source: 'radio' };
        this.logSeq = 0;
        this._statustextChunks = {};
        this.lastHearbeatTime = null;
        this._watchdogTimer = null;
        this._linkLossTimeoutMs = 5000;

        this.writer = null;
        this._txSeq = 0;
        this._gcsSysId = 255;
        this._gcsCompId = 190;
        this.targetSystem = 1;
        this.targetComponent = 1;
        this._streamsRequested = false;
        this._lastBaudRate = 57600;

        // Waiters untuk command/param/mission
        this._ackWaiters = new Map();
        this._paramWaiters = new Map();      // untuk getParam/setParam (key: nama param)
        this._paramListWaiters = [];         // untuk getAllParams
        this._missionUpload = null;          // state aktif uploadMission() (FC yang menarik item)
        this._missionCountWaiter = null;     // waiter MISSION_COUNT saat downloadMission()
        this._missionItemWaiter = null;      // waiter MISSION_ITEM_INT per-seq saat downloadMission()

        this._fenceUpload = null;
        this._fenceCountWaiter = null;
        this._fenceItemWaiter = null;

        // Callback publik
        this.onTelemetry = null;
        this.onMavlog = null;
        this.onConnect = null;
        this.onDisconnect = null;

        this.rcChannels = new Array(8).fill(65535);
        this._rcOverrideTimer = null;

        this._intentionalDisconnect = false;
        this._awaitingReconnect = false;
        this._reconnecting = false;
        this._reconnectTimer = null;
        this._isRebooting = false;

        this._setupAutoReconnectListeners();
    }

    async tryAutoReconnect(baudRate = 57600) {
        if (!('serial' in navigator)) return false;
        try {
            const ports = await navigator.serial.getPorts();
            if (ports.length === 0) return false;
            this.port = ports[0];
            await this.connect(baudRate);
            console.log('[RadioMavlink] Auto-reconnect berhasil.');
            return true;
        } catch (e) {
            console.warn('[RadioMavlink] Auto-reconnect gagal:', e.message);
            return false;
        }
    }

    _setupAutoReconnectListeners() {
        if (!('serial' in navigator)) return;
        navigator.serial.addEventListener('disconnect', (event) => {
            if (this.port && event.target === this.port) {
                console.warn('[RadioMavlink] Radio ter-unplug (fisik).');
                this._handleUnexpectedDrop();
            }
        });
        navigator.serial.addEventListener('connect', async (event) => {
            if (this.port && event.target === this.port
                && this._awaitingReconnect && !this._intentionalDisconnect) {
                console.log('[RadioMavlink] Radio ter-plug kembali (fisik), mencoba reconnect...');
                clearTimeout(this._reconnectTimer);
                this._attemptReconnect(1);
            }
        });
    }

    async _teardownTransport() {
        this.keepReading = false;
        this._stopWatchdog();
        this._stopRcOverrideLoop();
        this.rcChannels = new Array(8).fill(65535);
        if (this.reader) {
            try { await this.reader.cancel().catch(()=>{}); } catch (e) {}
        }
        
        // 2. Tutup Writer
        if (this.writer) {
            try { 
                await this.writer.close().catch(()=>{}); 
                this.writer.releaseLock();
            } catch (e) {}
            this.writer = null;
        }

        // 3. TUNGGU readLoop selesai melepas lock (Max 1 detik)
        let maxWait = 20;
        while (this._isReading && maxWait > 0) {
            await new Promise(r => setTimeout(r, 50));
            maxWait--;
        }

        // 4. Tutup Port secara aman di OS
        if (this.port) {
            try { await this.port.close(); } catch (e) {}
        }
        this.telemetry.connected = false;
        this._streamsRequested = false;
    }

    async _handleUnexpectedDrop() {
        if (this._intentionalDisconnect) return; // safety guard, harusnya tak pernah kena
        this._rejectPendingTransfers('Link radio terputus (unexpected drop) -- upload/download dibatalkan.');
        await this._teardownTransport();
        this._awaitingReconnect = true;
        if (this.onDisconnect) this.onDisconnect();
        this._scheduleReconnect(1);
    }

    _rejectPendingTransfers(message) {
        if (this._missionUpload) {
            const state = this._missionUpload;
            this._missionUpload = null;
            if (state.resendTimer) clearInterval(state.resendTimer);
            clearTimeout(state.timer);
            state.reject(new Error(message));
        }
        if (this._fenceUpload) {
            const state = this._fenceUpload;
            this._fenceUpload = null;
            if (state.resendTimer) clearInterval(state.resendTimer);
            clearTimeout(state.timer);
            state.reject(new Error(message));
        }
        if (this._missionCountWaiter) {
            this._missionCountWaiter.resolve(0);
            this._missionCountWaiter = null;
        }
        if (this._missionItemWaiter) {
            this._missionItemWaiter = null; // biar promise-nya reject lewat timeout internal downloadMission
        }
        if (this._fenceCountWaiter) {
            this._fenceCountWaiter.resolve(0);
            this._fenceCountWaiter = null;
        }
        if (this._fenceItemWaiter) {
            this._fenceItemWaiter = null;
        }
    }

    _scheduleReconnect(attempt) {
        if (this._intentionalDisconnect || !this._awaitingReconnect) return;
        clearTimeout(this._reconnectTimer);
        const delayMs = Math.min(1500 * attempt, 10000);
        console.log(`[RadioMavlink] Reconnect attempt ${attempt} dijadwalkan dalam ${delayMs}ms...`);
        this._reconnectTimer = setTimeout(() => this._attemptReconnect(attempt), delayMs);
    }

    async _attemptReconnect(attempt) {
        if (this._intentionalDisconnect || !this._awaitingReconnect || this._reconnecting) return;
        
        this._reconnecting = true;
        try {
            if ('serial' in navigator) {
                const ports = await navigator.serial.getPorts();
                if (ports.length > 0) {
                    this.port = ports[0];
                } else {
                    throw new Error("Perangkat belum terdeteksi kembali oleh OS.");
                }
            }

            if (!this.port) throw new Error("Port tidak valid.");

            await this.connect(this._lastBaudRate);
            console.log('[RadioMavlink] Auto-reconnect berhasil setelah link drop / reboot.');
            
            this._isRebooting = false; 
        } catch (e) {
            console.warn(`[RadioMavlink] Auto-reconnect attempt ${attempt} gagal:`, e.message);
            this._reconnecting = false;
            this._scheduleReconnect(attempt + 1);
            return;
        }
        this._reconnecting = false;
    }

    async requestPort() {
        this.port = await navigator.serial.requestPort();
        return this.port;
    }

    isLive() {
        const timeout = this.isRebooting ? 15000 : this._linkLossTimeoutMs;
        return this.telemetry.connected && (Date.now() - this.lastHeartbeatTime) < timeout;
    }

    async connect(baudRate = 57600) {
        if (!this.port) throw new Error('Belum ada port dipilih.');
        
        this._intentionalDisconnect = false;
        this._awaitingReconnect = false;

        await this.port.open({ baudRate });
        this._lastBaudRate = baudRate;
        this.writer = this.port.writable.getWriter();
        this.keepReading = true;
        this._streamsRequested = false;
        this.telemetry.connected = true;
        this.lastHeartbeatTime = Date.now();
        if (this.onConnect) this.onConnect();
        this._readLoop();
        this._startWatchdog();
        this._startRcOverrideLoop();
    }

    _startWatchdog() {
        this._stopWatchdog();
        this._watchdogTimer = setInterval(() => {
            if (this.telemetry.connected && !this.isLive()) {
                console.warn('[RadioMavlink] Heartbeat timeout — link dianggap putus.');
                this._isRebooting = false;
                this._handleUnexpectedDrop().catch(() => {});
                // this.disconnect().catch(() => {});
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
                const payload = [];
                for (let i = 0; i < 8; i++) {
                    const ch = this.rcChannels[i];
                    payload.push(ch & 0xFF, (ch >> 8) & 0xFF);
                }
                payload.push(this.targetSystem, this.targetComponent);
                this._sendFrame(70, payload, CRC_EXTRA[70]).catch(()=>{});
            }
        }, 150);
    }

    _stopRcOverrideLoop() {
        if (this._rcOverrideTimer) {
            clearInterval(this._rcOverrideTimer);
            this._rcOverrideTimer = null;
        }
    }

    async disconnect() {
       if (this._intentionalDisconnect) return;
        
        console.log('[RadioMavlink] Memutuskan koneksi secara manual...');
        
        this._intentionalDisconnect = true;
        this._awaitingReconnect = false;
        
        this._rejectPendingTransfers('Koneksi radio ditutup manual -- upload/download dibatalkan.');
        
        await this._teardownTransport();
        
        if (this.onDisconnect) this.onDisconnect();
    }

    async _sendFrame(msgId, payloadBytes, crcExtra, useV2 = false) {
        if (!this.writer) return;
        const builder = useV2 ? buildMavlink2Frame : buildMavlink1Frame;
        const frame = builder(this._txSeq++, this._gcsSysId, this._gcsCompId, msgId, payloadBytes, crcExtra);
        try { await this.writer.write(frame); } catch (e) {}
    }

    async requestDataStreams(rateHz = STREAM_RATE_HZ) {
        for (const streamId of REQUESTED_STREAMS) {
            const rateBytes = [rateHz & 0xFF, (rateHz >> 8) & 0xFF];
            const payload = [...rateBytes, this.targetSystem, this.targetComponent, streamId, 1];
            await this._sendFrame(66, payload, CRC_EXTRA[66]);
            await new Promise(r => setTimeout(r, 50));
        }
        console.log(`[RadioMavlink] Streams requested @ ${rateHz}Hz`);
    }

    async sendCommandAck(command, result = 1) {
        const buf = new ArrayBuffer(3);
        const dv = new DataView(buf);
        dv.setUint16(0, command, true); // MAV_CMD yang di-acknowledge
        dv.setUint8(2, result);         // 1 = MAV_RESULT_ACCEPTED
        
        const payload = Array.from(new Uint8Array(buf));
        await this._sendFrame(77, payload, CRC_EXTRA[77]);
        console.log(`[RadioMavlink] COMMAND_ACK sent for CMD ${command}`);
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

    // COMMAND_INT
    async sendCommandInt(command, params = {}, waitAck = true, timeout = 3000) {
        const p = { param1:0, param2:0, param3:0, param4:0, x:0, y:0, z:0, frame:0, ...params };
        const buf = new ArrayBuffer(35); // 4*4 (param1-4) + 4+4 (x,y) + 4 (z) + 2 (command) + 5*1 (target_system,target_component,frame,current,autocontinue)
        const dv = new DataView(buf);
        dv.setFloat32(0, p.param1, true);
        dv.setFloat32(4, p.param2, true);
        dv.setFloat32(8, p.param3, true);
        dv.setFloat32(12, p.param4, true);
        dv.setInt32(16, p.x, true);
        dv.setInt32(20, p.y, true);
        dv.setFloat32(24, p.z, true);
        dv.setUint16(28, command, true);
        dv.setUint8(30, this.targetSystem);
        dv.setUint8(31, this.targetComponent);
        dv.setUint8(32, p.frame);
        dv.setUint8(33, 0);  // current
        dv.setUint8(34, 0);  // autocontinue
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

    // PARAMETER
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
                        console.warn(`[RadioMavlink] Timeout getAllParams, mengembalikan ${receivedCount} parameter yang diterima`);
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
            const state = { items, resolve, reject, timer: null, resendTimer: null };
            state.resetTimer = () => {
                if (state.resendTimer) { clearInterval(state.resendTimer); state.resendTimer = null; }
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

            state.resendTimer = setInterval(() => {
                if (!this._missionUpload) return;
                console.warn('[RadioMavlink] uploadMission: belum ada respons FC, resend MISSION_COUNT...');
                this._sendFrame(44, countPayload, CRC_EXTRA[44]);
            }, 1500);
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
        console.log('[RadioMavlink] downloadMission: mengirim MISSION_REQUEST_LIST...');
        let count;
        try {
            count = await new Promise((resolve, reject) => {
                const listPayload = [this.targetSystem, this.targetComponent];
                // Sama seperti uploadMission -- MISSION_REQUEST_LIST adalah paket pembuka.
                // Resend periodik sampai FC balas MISSION_COUNT, supaya tahan paket hilang.
                const resendTimer = setInterval(() => {
                    console.warn('[RadioMavlink] downloadMission: belum ada respons FC, resend MISSION_REQUEST_LIST...');
                    this._sendFrame(43, listPayload, CRC_EXTRA[43]);
                }, 1500);
                const timer = setTimeout(() => {
                    clearInterval(resendTimer);
                    console.warn('[RadioMavlink] Timeout MISSION_REQUEST_LIST, asumsikan tidak ada misi');
                    resolve(0);
                }, timeout);
                this._missionCountWaiter = { resolve: (c) => { clearInterval(resendTimer); clearTimeout(timer); resolve(c); } };
                this._sendFrame(43, listPayload, CRC_EXTRA[43]);
            });
        } catch (e) {
            console.warn('[RadioMavlink] Gagal mendapatkan MISSION_COUNT:', e);
            return [];
        }

        if (count === 0) {
            console.log('[RadioMavlink] Tidak ada misi di FC.');
            return [];
        }

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
                    console.warn(`[RadioMavlink] Item seq=${seq} timeout, percobaan ${attempt + 1}/${maxRetries}`);
                }
            }
            if (!item) throw new Error(`Gagal ambil mission item seq=${seq} setelah ${maxRetries} percobaan`);
            items.push(item);
        }

        await this._sendFrame(47, [this.targetSystem, this.targetComponent, 0], CRC_EXTRA[47]);
        return items;
    }

    _buildFenceItemPayload(seq, item) {
        const base = this._buildMissionItemPayload(seq, item);
        return [...base, MAV_MISSION_TYPE_FENCE];
    }

    async uploadFence(points, timeout = 10000) {
        if (!Array.isArray(points) || points.length < 3) {
            throw new Error('Fence polygon butuh minimal 3 titik.');
        }
        if (this._fenceUpload) throw new Error('Upload fence sedang berlangsung, tunggu selesai.');

        // STANDAR ARDUPILOT: Polygon harus ditutup (Titik awal = Titik akhir)
        let uploadPoints = [...points];
        const first = uploadPoints[0];
        const last = uploadPoints[uploadPoints.length - 1];
        if (first.lat !== last.lat || first.lon !== last.lon) {
            uploadPoints.push({ lat: first.lat, lon: first.lon });
        }

        const items = [];
        // STANDAR ARDUPILOT: Wajib mengirim Return Point di urutan pertama (seq 0)
        items.push({
            command: 5000, // MAV_CMD_NAV_FENCE_RETURN_POINT
            param1: 0,
            x: Math.round(uploadPoints[0].lat * 1e7),
            y: Math.round(uploadPoints[0].lon * 1e7),
            z: 0, frame: 0, current: 0, autocontinue: 1,
        });

        // Masukkan titik-titik poligonnya
        const vertexCount = uploadPoints.length;
        uploadPoints.forEach((p) => {
            items.push({
                command: 5001, // MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
                param1: vertexCount,
                x: Math.round(p.lat * 1e7),
                y: Math.round(p.lon * 1e7),
                z: 0, frame: 0, current: 0, autocontinue: 1,
            });
        });

        return new Promise((resolve, reject) => {
            const state = { items, resolve, reject, timer: null, resendTimer: null };
            state.resetTimer = () => {
                if (state.resendTimer) { clearInterval(state.resendTimer); state.resendTimer = null; }
                if (state.timer) clearTimeout(state.timer);
                state.timer = setTimeout(() => {
                    this._fenceUpload = null;
                    reject(new Error('Upload fence timeout.'));
                }, timeout);
            };
            state.resetTimer();
            this._fenceUpload = state;

            const countPayload = [
                items.length & 0xFF, (items.length >> 8) & 0xFF,
                this.targetSystem, this.targetComponent, 1 // 1 = MAV_MISSION_TYPE_FENCE
            ];
            this._sendFrame(44, countPayload, CRC_EXTRA[44], true);

            // Sama seperti uploadMission -- MISSION_COUNT(mission_type=FENCE) adalah paket
            // pembuka. Resend periodik sampai FC merespons, supaya tahan paket hilang.
            state.resendTimer = setInterval(() => {
                if (!this._fenceUpload) return;
                console.warn('[RadioMavlink] uploadFence: belum ada respons FC, resend MISSION_COUNT...');
                this._sendFrame(44, countPayload, CRC_EXTRA[44], true);
            }, 1500);
        });
    }

    async downloadFence(timeout = 5000, maxRetries = 3) {
        let count;
        try {
            count = await new Promise((resolve, reject) => {
                const listPayload = [this.targetSystem, this.targetComponent, 1];
                const resendTimer = setInterval(() => {
                    console.warn('[RadioMavlink] downloadFence: belum ada respons FC, resend MISSION_REQUEST_LIST...');
                    this._sendFrame(43, listPayload, CRC_EXTRA[43], true);
                }, 1500);
                const timer = setTimeout(() => { clearInterval(resendTimer); resolve(0); }, timeout);
                this._fenceCountWaiter = { resolve: (c) => { clearInterval(resendTimer); clearTimeout(timer); resolve(c); } };
                this._sendFrame(43, listPayload, CRC_EXTRA[43], true);
            });
        } catch (e) { return []; }

        if (count === 0) return [];

        const items = [];
        for (let seq = 0; seq < count; seq++) {
            let item = null;
            for (let attempt = 0; attempt < maxRetries && !item; attempt++) {
                try {
                    item = await new Promise((resolve, reject) => {
                        const timer = setTimeout(() => reject(new Error('timeout')), timeout);
                        this._fenceItemWaiter = { seq, resolve: (it) => { clearTimeout(timer); resolve(it); } };
                        this._sendFrame(51, [seq & 0xFF, (seq >> 8) & 0xFF, this.targetSystem, this.targetComponent, 1], CRC_EXTRA[51], true);
                    });
                } catch (e) {}
            }
            if (!item) throw new Error(`Gagal ambil fence seq=${seq}`);
            items.push(item);
        }

        await this._sendFrame(47, [this.targetSystem, this.targetComponent, 0, 1], CRC_EXTRA[47], true);
        
        return items
            .sort((a, b) => a.seq - b.seq)
            .filter(it => it.command === 5001)
            .map(it => ({ lat: it.x / 1e7, lon: it.y / 1e7 }));
    }

    async clearFence(timeout = 5000) {
        if (this._fenceUpload) throw new Error('Operasi fence lain sedang berlangsung, tunggu selesai.');
        return new Promise((resolve, reject) => {
            const state = { items: [], resolve, reject, timer: null, resendTimer: null };
            state.resetTimer = () => {
                if (state.resendTimer) { clearInterval(state.resendTimer); state.resendTimer = null; }
                if (state.timer) clearTimeout(state.timer);
                state.timer = setTimeout(() => {
                    this._fenceUpload = null;
                    reject(new Error('Clear fence timeout.'));
                }, timeout);
            };
            state.resetTimer();
            this._fenceUpload = state;
            const countPayload = [0, 0, this.targetSystem, this.targetComponent, MAV_MISSION_TYPE_FENCE];
            this._sendFrame(44, countPayload, CRC_EXTRA[44], true);

            state.resendTimer = setInterval(() => {
                if (!this._fenceUpload) return;
                console.warn('[RadioMavlink] clearFence: belum ada respons FC, resend MISSION_COUNT(0)...');
                this._sendFrame(44, countPayload, CRC_EXTRA[44], true);
            }, 1500);
        });
    }

    _handleFenceRequest(seq) {
        const state = this._fenceUpload;
        if (!state) return;
        const item = state.items[seq];
        if (!item) {
            console.warn(`[RadioMavlink] FC minta fence item seq=${seq} tapi fence cuma punya ${state.items.length} titik`);
            return;
        }
        state.resetTimer();
        const payload = this._buildFenceItemPayload(seq, item);
        this._sendFrame(73, payload, CRC_EXTRA[73], true);
    }

    _handleFenceAck(result) {
        const state = this._fenceUpload;
        if (!state) return;
        if (state.resendTimer) clearInterval(state.resendTimer);
        clearTimeout(state.timer);
        this._fenceUpload = null;
        if (result === 0) { // MAV_MISSION_ACCEPTED
            state.resolve({ ok: true });
        } else {
            state.resolve({ ok: false, message: `Fence ditolak FC, kode error MAV_MISSION_RESULT=${result}` });
        }
    }

    async setFenceEnabled(enabled) {
        return this.setParam('FENCE_ENABLE', enabled ? 1 : 0); // MAV_PARAM_TYPE_UINT8=1
    }
    async getFenceEnabled() {
        const v = await this.getParam('FENCE_ENABLE');
        return v !== 0;
    }
    async setFenceAction(actionCode) {
        return this.setParam('FENCE_ACTION', actionCode);
    }

    async armDisarm(arm) {
        return this.sendCommandLong(400, { p1: arm ? 1 : 0 });
    }
    async takeoff(alt = 10) {
        return this.sendCommandLong(22, { p7: alt });
    }
    async rtl() {
        return this.sendCommandLong(20);
    }
    async land() {
        return this.sendCommandLong(21);
    }
    async setMode(modeCode) {
        const MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1;
        return this.sendCommandLong(176, { p1: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, p2: modeCode });
    }
    async reboot() {
        console.log('[Mavlink] reboot');
        this._isRebooting = true;
        if (this.telemetry) {
            this.telemetry.system_status = 'rebooting';
            if (this.onTelemetry) this.onTelemetry({ ...this.telemetry });
        }
        return this.sendCommandLong(246, { p1: 1 });
    }
    async setHome(lat, lon, alt = 0) {
        return this.sendCommandLong(179, { p1: 0, p5: lat, p6: lon, p7: alt });
    }
    async reposition(lat, lon, alt = null) {
        const targetAlt = (alt !== null && !Number.isNaN(alt)) ? alt : (Number(this.telemetry?.alt) || 0);
        return this.sendCommandLong(192, { p1: -1, p4: NaN, p5: lat, p6: lon, p7: targetAlt });
    }
    async setServo(channel, pwm) {
        return this.sendCommandLong(183, { p1: channel, p2: pwm });
    }

    async sendRcOverride(chIndex, pwmValue) {
        if (chIndex < 0 || chIndex > 7) {
            throw new Error('sendRcOverride hanya mendukung channel 1-8 (index 0-7) -- base RC_CHANNELS_OVERRIDE MAVLink1');
        }
        this.rcChannels[chIndex] = pwmValue;

        console.log(`[RC_OVERRIDE] ch${chIndex + 1} -> ${pwmValue}us | terkirim:`, [...this.rcChannels]);

        const buildPayload = () => {
            const payload = [];
            for (let i = 0; i < 8; i++) {
                const ch = this.rcChannels[i];
                payload.push(
                    ch & 0xFF, (ch >> 8) & 0xFF);
            }
            payload.push(this.targetSystem, this.targetComponent);
            return payload;
        };
        await this._sendFrame(70, buildPayload(), CRC_EXTRA[70]);
        for (let i = 0; i < 3; i++) {
            await new Promise(r => setTimeout(r, 50));
            this._sendFrame(70, buildPayload(), CRC_EXTRA[70]).catch(()=>{});
        }    
    }

    async calibrateMag(compassId = -1) {
        return this.sendCommandLong(241, { p3: 1, p4: compassId >= 0 ? compassId : 0 });
    }
    async calibrateAccel(retries = 0) {
        return this.sendCommandLong(241, { p1: 1, p2: retries });
    }
    async calibrateGyro() {
        return this.sendCommandLong(241, { p5: 1 });
    }
    async calibrateLevel() {
        return this.sendCommandLong(241, { p6: 1 });
    }

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

    async _readLoop() {
        this._isReading = true;
        
        while (this.port && this.port.readable && this.keepReading) {
            this.reader = this.port.readable.getReader();
            try {
                while (true) {
                    const { value, done } = await this.reader.read();
                    if (done) break;
                    for (const byte of value) this.parser.feedByte(byte);
                }
            } catch (err) {
                console.error('[RadioMavlink] Read error (kemungkinan USB terputus):', err.message);
                break;
            } finally {
                if (this.reader) {
                    this.reader.releaseLock();
                    this.reader = null;
                }
            }
        }
        
        this._isReading = false;
    }

    _handleFrame(msgId, payload, sysid, compid) {
        const isSelf = (sysid === this._gcsSysId && compid === this._gcsCompId);

        if (msgId === 0 && !isSelf) {
            if (this._isRebooting) {
                console.log(`[RadioMavlink] Heartbeat diterima dari sysid=${sysid}, compid=${compid} -- FC sudah reboot.`);
                this._isRebooting = false;
                this._streamsRequested = false;
                if (this.telemetry) {
                    this.telemetry.system_status = 'connected';
                }
            }
            if (!this._streamsRequested) {
                console.log(`[RadioMavlink] Heartbeat pertama diterima: sysid=${sysid}, compid=${compid} -- dipakai sebagai target.`);
                this.targetSystem = sysid;
                this.targetComponent = compid;
                this._streamsRequested = true;
                this.requestDataStreams().catch(e => console.warn('[RadioMavlink] requestDataStreams gagal:', e));
            }
            if (sysid === this.targetSystem && compid === this.targetComponent) {
                this.lastHeartbeatTime = Date.now();
            } else {
                console.warn(`[RadioMavlink] Heartbeat dari sysid=${sysid}/compid=${compid} diabaikan (target saat ini: ${this.targetSystem}/${this.targetComponent}) -- kemungkinan ada device lain di link yang sama.`);
            }
        }

        if (isSelf) return;
        if (this._streamsRequested && sysid !== this.targetSystem) return;

        const decoder = DECODERS[msgId];
        if (!decoder) {
            console.warn(`[RadioMavlink] Msg ID ${msgId} diterima tapi tidak ada decoder -- frame dibuang`);
            return;
        }

        const paddedPayload = new Uint8Array(255);
        paddedPayload.set(payload, 0);

        let result;
        try { 
            result = decoder(paddedPayload); // <--- Gunakan data yang sudah diganjal
        } catch (e) { 
            console.error(`Gagal memproses pesan MAVLink ID ${msgId}:`, e);
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
        } else if (result._kind === 'command_long') {
            this._handleCommandLong(result);
        } else if (result._kind === 'param_value') {
            this._handleParamValue(result);
        } else if (result._kind === 'mission_request') {
            if (result.missionType === MAV_MISSION_TYPE_FENCE) this._handleFenceRequest(result.seq);
            else this._handleMissionRequest(result.seq);
        } else if (result._kind === 'mission_count') {
            if (result.missionType === MAV_MISSION_TYPE_FENCE) {
                if (this._fenceCountWaiter) {
                    const w = this._fenceCountWaiter;
                    this._fenceCountWaiter = null;
                    w.resolve(result.count);
                }
            } else {
                this._handleMissionCount(result.count);
            }
        } else if (result._kind === 'mission_item') {
            if (result.missionType === MAV_MISSION_TYPE_FENCE) {
                if (this._fenceItemWaiter && this._fenceItemWaiter.seq === result.seq) {
                    const w = this._fenceItemWaiter;
                    this._fenceItemWaiter = null;
                    w.resolve(result);
                }
            } else {
                this._handleMissionItem(result);
            }
        } else if (result._kind === 'mission_ack') {
            if (result.missionType === MAV_MISSION_TYPE_FENCE) this._handleFenceAck(result.result);
            else this._handleMissionAck(result.result);
        } else if (result._kind === 'distance_sensor') {
            const RANGEFINDER_TYPE_MAP = { 0: 'LASER', 1: 'ULTRASOUND', 2: 'INFRARED', 3: 'RADAR', 4: 'UNKNOWN' };
            const isHealthy = result.signal_quality !== 1
                && result.current_distance > result.min_distance
                && result.current_distance < result.max_distance;

            this.telemetry.rangefinder_distance = result.current_distance / 100;   // cm -> m
            this.telemetry.rangefinder_healthy = isHealthy;
            this.telemetry.rangefinder_type = RANGEFINDER_TYPE_MAP[result.type] || 'UNKNOWN';
            if (this.onTelemetry) this.onTelemetry({ ...this.telemetry });
        } else if (result._kind === 'mag_cal_progress') {
            this.telemetry.mag_cal_progress = { id: result.compass_id, pct: result.completion_pct };
            if (this.onTelemetry) this.onTelemetry({ ...this.telemetry });
        } else if (result._kind === 'mag_cal_report') {
            this.telemetry.mag_cal_report = { id: result.compass_id, status: result.cal_status, fitness: result.fitness };
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

    _handleCommandLong({ command, param1 }) {
        const MAV_CMD_ACCELCAL_VEHICLE_POS = 42429;
        if (command === MAV_CMD_ACCELCAL_VEHICLE_POS) {
            console.log(`[RadioMavlink] FC minta konfirmasi posisi accel cal #${param1}`);
            if (this.onAccelCalPosRequest) this.onAccelCalPosRequest(param1);
        } else {
            console.log(`[RadioMavlink] COMMAND_LONG diterima dari FC (command=${command}), belum ada handler spesifik -- diabaikan.`);
        }
    }

    _handleParamValue({ name, value, paramIndex, paramCount }) {
        // Waiter tunggal (getParam/setParam)
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
            w._resetTimer(); // ada progress -- perpanjang timeout, jangan flat
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
        if (!state) {
            console.warn(`[RadioMavlink] FC minta mission item seq=${seq} tapi tidak ada upload aktif (mungkin sudah timeout/reconnect duluan)`);
            return;
        }
        const item = state.items[seq];
        if (!item) {
            console.warn(`[RadioMavlink] FC minta item seq=${seq} tapi mission cuma punya ${state.items.length} item`);
            return;
        }
        console.log(`[RadioMavlink] uploadMission: mengirim item seq=${seq}/${state.items.length - 1} (command=${item.command})`);
        state.resetTimer(); // ada progress -- perpanjang timeout
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
        if (!state) {
            console.warn(`[RadioMavlink] Terima MISSION_ACK (result=${result}) tapi tidak ada upload aktif -- kemungkinan ACK ini datang terlambat setelah upload sebelumnya sudah timeout/reconnect.`);
            return;
        }
        console.log(`[RadioMavlink] uploadMission: MISSION_ACK diterima, result=${result} (0=ACCEPTED)`);
        if (state.resendTimer) clearInterval(state.resendTimer);
        clearTimeout(state.timer);
        this._missionUpload = null;
        if (result === 0) { // MAV_MISSION_ACCEPTED
            state.resolve({ok: true});
        } else {
            state.resolve({ ok: false, message: `Mission ditolak FC, kode error MAV_MISSION_RESULT=${result}` });
        }
    }

    _handleStatustext({ severity, text, msgId, chunkSeq }) {
        if (msgId === 0 && chunkSeq === 0) {
            // Pesan non-chunked biasa
            this._emitStatustext(text, severity);
            return;
        }

        // Pesan chunked: text.length < 50 = sinyal akhir sequence (sesuai konvensi
        // MAVLink STATUSTEXT chunking). Tapi ini bisa gagal kalau chunk terakhir
        // pas 50 char, atau ada chunk yang hilang di link radio -> pakai idle-timeout
        // sebagai safety net supaya buffer tidak nyangkut selamanya.
        let entry = this._statustextChunks[msgId];
        if (!entry) {
            entry = { chunks: {}, severity, timer: null };
            this._statustextChunks[msgId] = entry;
        }
        entry.chunks[chunkSeq] = text;
        entry.severity = severity;

        if (entry.timer) clearTimeout(entry.timer);

        const flush = () => {
            const e = this._statustextChunks[msgId];
            if (!e) return;
            const keys = Object.keys(e.chunks).map(Number).sort((a, b) => a - b);
            const fullText = keys.map(k => e.chunks[k]).join('');
            delete this._statustextChunks[msgId];
            this._emitStatustext(fullText, e.severity);
        };

        if (text.length < 50) {
            flush();
        } else {
            // Safety net: kalau tidak ada chunk berikutnya dalam 800ms, paksa flush
            // apa adanya (mencegah pesan SAR hilang total kalau chunk terakhir hilang).
            entry.timer = setTimeout(flush, 800);
        }
    }

    _emitStatustext(text, severity) {
        this.logSeq++;
        const logItem = { seq: this.logSeq, text, severity, server_time: Date.now() / 1000, source: 'radio' };
        if (this.onMavlog) this.onMavlog(logItem);
    }
}

window.RadioMavlink = RadioMavlink;
console.log('[RadioMavlink] radio-mavlink.js loaded (full version)');