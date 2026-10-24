const MAV_RESULT_MESSAGES = {
    0: 'ACCEPTED',
    1: 'TEMPORARILY_REJECTED',
    2: 'DENIED',
    3: 'UNSUPPORTED',
    4: 'FAILED',
    5: 'IN_PROGRESS',
    6: 'CANCELLED'
};

function mavResultMessage(result) {
    return MAV_RESULT_MESSAGES[result] ?? `UNKNOWN (${result})`;
}

// harus match FCU_COMP_ID di config.h firmware ESP32
const MAV_COMP_ID_AUTOPILOT1 = 1;

class RadioMavlink {
    constructor() {
        this._useWebSerial = !window.serialBridge && ('serial' in navigator);
        this._webSerialCache = new Map();
        this._webSerialWriter = null;

        this.port = null;
        this.reader = null;
        this.keepReading = false;
        this.parser = new MavlinkFrameParser((msgId, payload, sysid, compid) => this._handleFrame(msgId, payload, sysid, compid));

        this.telemetry = { connected: false, source: 'radio' };
        this.logSeq = 0;
        this._statustextChunks = {};
        this.lastHeartbeatTime = null;
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
        this._physicalRc = new Array(8).fill(65535);
        this._physicalRc5 = 65535;
        this._physicalRc5Seq = 0;
        this._physicalRc5LastUpdate = 0;
        // this._hybridOverride = {active: false, channel: 4, targetPwm: 65535, targetState: null, startedAt: 0, initialRemoteState: null, matchStableCount: 0, changeStableCount: 0, requiredStableMatches: 3, matchedOnce: false, lastPhysicalState: null, maxHoldMs: 30000, releaseTimer: null};
        this._hybridOverride = {active: false, channel: 4, targetPwm: 65535, targetState: null};

        this._intentionalDisconnect = false;
        this._awaitingReconnect = false;
        this._reconnecting = false;
        this._isRebooting = false;
        this._missionBusy = false; // true selama uploadMission/downloadMission -- FC bisa stall EEPROM write & telat kirim heartbeat, watchdog jangan false-trigger disconnect
        this._transportOpen = false;
        this._connectStartedAt = 0;
        this._magCalTelemetryTimer = null;
        this._magCalTelemetryPending = false;
        this._removeSerialDataListener = null;
        this._removeSerialClosedListener = null;
        this._removeSerialErrorListener = null;
        this._handlingUnexpectedDrop = false;
        this._setupAutoReconnectListeners();
    }
    // JANGAN DIHAPUS
    // ini yang bisa di electron
    async tryAutoReconnect(baudRate = 57600) {
        if (this._useWebSerial) {
            try {
                const ports = await navigator.serial.getPorts();
                if (!ports || ports.length === 0) return false;
                this.port = ports[0];
                const info = await this.port.getInfo();
                const pathName = `WebSerial_${info.usbVendorId || 'Auto'}_${info.usbProductId || 'Port'}`;
                this._webSerialCache.set(pathName, this.port);
                console.log('[RadioMavlink] Mencoba auto-reconnect ke:', pathName);
                await this.connect(baudRate, pathName);
                return true;
            } catch (e) { return false; }
        }
         if (!window.serialBridge || typeof window.serialBridge.list !== 'function') {
            console.warn('[RadioMavlink] serialBridge tidak tersedia.');
            return false;
        }
        try {
            const ports = await window.serialBridge.list();
            if (!port || ports.length === 0) {
                console.warn('[RadioMavlink] Tidak ada port serial yang tersedia'); 
                return false;
            };
            let found = null
            if (this._lastPortPath) {
                found = ports.find(
                    p => p.path === this._lastPortPath
                );
            }
            if (!found && this._lastVendorId && this._lastProductId) {
                found = ports.find(
                    p =>
                        p.vendorId === this._lastVendorId &&
                        p.productId === this._lastProductId
                );
            }
            if (!found) {
                if (!this._lastPortPath && ports.length === 1) {
                    found = ports[0];
                } else {
                    console.warn(
                        '[RadioMavlink] Port FC sebelumnya tidak ditemukan.'
                    );

                    return false;
                }
            }
            console.log('[RadioMavlink] Mencoba auto-reconnect ke:', found.path);
            await this.connect(baudRate,found.path);
            console.log('[RadioMavlink] Auto-reconnect berhasil.');
            return true;

        } catch (e) {
            console.warn('[RadioMavlink] Auto-reconnect gagal:', e.message);
            return false;
        }
    }

    // async tryAutoReconnect(baudRate = 57600) {
    //     if (this._useWebSerial) {
    //         try {
    //             const ports = await navigator.serial.getPorts();
    //             if (!ports || ports.length === 0) return false;
                
    //             let found = null;
    //             if (this.port && ports.includes(this.port)) {
    //                 found = this.port;
    //             } else if (ports.length > 0) {
    //                 found = ports[0];
    //             }
                
    //             if (found) {
    //                 this.port = found;
    //                 const info = this.port.getInfo();
    //                 const pathName = `WebSerial_${info.usbVendorId || 'Auto'}_${info.usbProductId || 'Port'}`;
    //                 this._webSerialCache.set(pathName, this.port);
    //                 console.log('[RadioMavlink] Mencoba auto-reconnect ke Web Serial:', pathName);
    //                 await this.connect(baudRate, pathName);
    //                 return true;
    //             }
    //             return false;
    //         } catch (e) { return false; }
    //     }

    //     if (!window.serialBridge || typeof window.serialBridge.list !== 'function') {
    //         console.warn('[RadioMavlink] serialBridge tidak tersedia.');
    //         return false;
    //     }

    //     try {
    //         const ports = await window.serialBridge.list();
    //         if (!ports || ports.length === 0) {
    //             console.warn('[RadioMavlink] Tidak ada port serial yang tersedia'); 
    //             return false;
    //         };
    //         let found = null
    //         if (this._lastPortPath) {
    //             found = ports.find(p => p.path === this._lastPortPath);
    //         }
    //         if (!found && this._lastVendorId && this._lastProductId) {
    //             found = ports.find(p => p.vendorId === this._lastVendorId && p.productId === this._lastProductId);
    //         }
    //         if (!found) {
    //             if (!this._lastPortPath && ports.length === 1) {
    //                 found = ports[0];
    //             } else {
    //                 console.warn('[RadioMavlink] Port FC sebelumnya tidak ditemukan.');
    //                 return false;
    //             }
    //         }
    //         console.log('[RadioMavlink] Mencoba auto-reconnect ke:', found.path);
    //         await this.connect(baudRate, found.path);
    //         console.log('[RadioMavlink] Auto-reconnect berhasil.');
    //         return true;

    //     } catch (e) {
    //         console.warn('[RadioMavlink] Auto-reconnect gagal:', e.message);
    //         return false;
    //     }
    // }

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
        this._stopWatchdog();
        this._stopRcOverrideLoop();
        if (this._useWebSerial) {
            this.keepReading = false;
            if (this.reader) {
                try { await this.reader.cancel(); } catch (e) {}
            }
            if (this._webSerialWriter) {
                try { await this._webSerialWriter.releaseLock(); } catch (e) {}
                this._webSerialWriter = null;
            }
            if (this.port) {
                try { await this.port.close(); } catch (e) {console.warn('[RadioMavlink] Peringatan saat close port:', e);}
            }
        } else {
            if (this._removeSerialDataListener) {
                this._removeSerialDataListener();
                this._removeSerialDataListener = null;
            }
            if (this._removeSerialClosedListener) {
                this._removeSerialClosedListener();
                this._removeSerialClosedListener = null;
            }
            if (this._removeSerialErrorListener) {
                this._removeSerialErrorListener();
                this._removeSerialErrorListener = null;
            }
            try { await window.serialBridge.disconnect(); } catch (e) {}
        }
        this.rcChannels = new Array(8).fill(65535);
        this._transportOpen = false;
        this.telemetry.connected = false;
    }

    async _handleUnexpectedDrop() {
        if (this._intentionalDisconnect) return;
        if (this._handlingUnexpectedDrop) {console.warn('[RadioMavlink] Unexpected Drop dipanggil berulang, abaikan'); return;}
        this._handlingUnexpectedDrop = true;
        this._awaitingReconnect = true;
        try {
            this._rejectPendingTransfers('Link radio terputus (unexpected drop) -- upload/download dibatalkan.');
            if (this.onDisconnect) this.onDisconnect();
            await this._teardownTransport();
            this._scheduleReconnect(1);
        } catch (e) {
            console.error('[RadioMavlink] Error saat handleUnexpectedDrop:', e);
        } finally {
            this._handlingUnexpectedDrop = false;
        }        
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
            this._missionItemWaiter = null;
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

    // JANGAN DIHAPUS
    // ini yang bisa di electron
    async _attemptReconnect(attempt) {
        if (this._intentionalDisconnect || !this._awaitingReconnect || this._reconnecting) return;

        this._reconnecting = true;
        try {
            if (!window.serialBridge || typeof window.serialBridge.list !== 'function') {
                throw new Error('serialBridge tidak tersedia.');
            }
            if (!this._lastPortPath) {
                throw new Error('Tidak ada portPath tersimpan dari koneksi sebelumnya.');
            }

            const ports = await window.serialBridge.list();
            let found = ports.find(p => p.path === this._lastPortPath);
            if (!found && this._lastVendorId && this._lastProductId) {
                found = ports.find(p => p.vendorId === this._lastVendorId && p.productId === this._lastProductId);
            }
            if (!found) {
                throw new Error('Perangkat belum terdeteksi kembali oleh OS.');
            }

            await this.connect(this._lastBaudRate, found.path);
            console.log('[RadioMavlink] Auto-reconnect berhasil setelah link drop / reboot.');

        } catch (e) {
            console.warn(`[RadioMavlink] Auto-reconnect attempt ${attempt} gagal:`, e.message);
            this._reconnecting = false;
            this._scheduleReconnect(attempt + 1);
            return;
        }
        this._reconnecting = false;
    }

    // ini untuk web
    // async _attemptReconnect(attempt) {
    //     if (this._intentionalDisconnect || !this._awaitingReconnect || this._reconnecting) return;

    //     this._reconnecting = true;

    //     // --- Perbaikan: Pastikan jika ini Web Serial, maka JANGAN panggil serialBridge ---
    //     if (this._useWebSerial) {
    //         try {
    //             const ports = await navigator.serial.getPorts();
    //             let found = null;
    //             if (this.port && ports.includes(this.port)) found = this.port;
    //             else if (ports.length > 0) found = ports[0];
                
    //             if (!found) throw new Error('Perangkat Web Serial belum terdeteksi kembali.');
    //             this.port = found;
    //             await this.connect(this._lastBaudRate, this._lastPortPath);
    //             console.log('[RadioMavlink] Auto-reconnect Web Serial berhasil.');
    //             this._reconnecting = false;
    //             return; // Berhenti di sini, jangan lanjutkan ke blok Electron!
    //         } catch(e) {
    //             console.warn(`[RadioMavlink] Auto-reconnect Web Serial attempt ${attempt} gagal:`, e.message);
    //             this._reconnecting = false;
    //             this._scheduleReconnect(attempt + 1);
    //             return;
    //         }
    //     }

    //     try {
    //         if (!window.serialBridge || typeof window.serialBridge.list !== 'function') {
    //             throw new Error('serialBridge tidak tersedia.');
    //         }
    //         if (!this._lastPortPath) {
    //             throw new Error('Tidak ada portPath tersimpan dari koneksi sebelumnya.');
    //         }

    //         const ports = await window.serialBridge.list();
    //         let found = ports.find(p => p.path === this._lastPortPath);
    //         if (!found && this._lastVendorId && this._lastProductId) {
    //             found = ports.find(p => p.vendorId === this._lastVendorId && p.productId === this._lastProductId);
    //         }
    //         if (!found) {
    //             throw new Error('Perangkat belum terdeteksi kembali oleh OS.');
    //         }

    //         await this.connect(this._lastBaudRate, found.path);
    //         console.log('[RadioMavlink] Auto-reconnect berhasil setelah link drop / reboot.');

    //     } catch (e) {
    //         console.warn(`[RadioMavlink] Auto-reconnect attempt ${attempt} gagal:`, e.message);
    //         this._reconnecting = false;
    //         this._scheduleReconnect(attempt + 1);
    //         return;
    //     }
    //     this._reconnecting = false;
    // }

    async requestPort() {
        if (this._useWebSerial) {
            try {
                const port = await navigator.serial.requestPort();
                const info = port.getInfo();
                const pathName = `WebSerial_${info.usbVendorId || 'Unknown'}_${info.usbProductId || 'Port'}`;
                this._webSerialCache.set(pathName, port);
                return [{ path: pathName, manufacturer: 'Browser Web API' }];
            } catch (e) {
                console.warn('[RadioMavlink] Web Serial ditolak/dibatalkan user:', e);
                return [];
            }
        }
        if (!window.serialBridge || typeof window.serialBridge.list !== 'function') {
        throw new Error('serialBridge tidak tersedia. Cek apakah preload.js berhasil dimuat.');
        }
        return await window.serialBridge.list();
    }

    isLive() {
        const timeout = (this._isRebooting || this._missionBusy) ? 15000 : this._linkLossTimeoutMs;
        return this.telemetry.connected && (Date.now() - this.lastHeartbeatTime) < timeout;
    }

    async connect(baudRate = 57600, portPath) {
        if (!portPath) throw new Error('Belum ada port dipilih.');
        
        this._intentionalDisconnect = false;
        this._awaitingReconnect = false;
        this._linkReady = false;
        this._transportOpen = false;
        this.lastHeartbeatTime = null;
        this._streamsRequested = false;
        this._lastBaudRate = baudRate;
        this.telemetry.connected = false;
        this._lastPortPath = portPath;

        if (this._useWebSerial) {
            this.port = this._webSerialCache.get(portPath);
            if (!this.port) {
                const ports = await navigator.serial.getPorts();
                if (ports.length > 0) {
                    this.port = ports[0];
                    this._webSerialCache.set(portPath, this.port);
                } else {
                    throw new Error('Web Serial Port tidak ditemukan di memori.');
                }
            }
            try {
                await this.port.open({ baudRate });
            } catch (e) {
                if (e.name !== 'InvalidStateError' && !e.message.includes('already open')) {
                    throw e;
                }
                console.log('[RadioMavlink] Port sudah terbuka, melanjutkan koneksi...');
            }
            if (!this._webSerialWriter) {
                try {
                    this._webSerialWriter = this.port.writable.getWriter();
                } catch (e) {
                    console.warn('[RadioMavlink] Writer terkunci, mencoba mereset...', e);
                }
            }
            if (!this.keepReading) {
                this.keepReading = true;
                this._readLoop();
            }
            this._transportOpen = true;
            this._connectStartedAt = Date.now();
            this._startWatchdog();
            this._startRcOverrideLoop();
            return;
        }

        if (!window.serialBridge) {
            throw new Error("Sistem mendeteksi bukan Web Serial (mungkin karena tidak HTTPS), tetapi modul Electron juga tidak ditemukan.");
        }

        try {
            const ports = await window.serialBridge.list();
            const current = ports.find(p => p.path === portPath);
            if (current) {
                this._lastVendorId = current.vendorId;
                this._lastProductId = current.productId;
            }
        } catch (e) {}

        const res = await window.serialBridge.connect(portPath, baudRate);
        if(!res.ok) throw new Error(`Gagal membuka port serial: ${res.message}`);

        if(this._removeSerialDataListener) {
            this._removeSerialDataListener();
            this._removeSerialDataListener = null;
        }
        if (this._removeSerialClosedListener) {
            this._removeSerialClosedListener();
            this._removeSerialClosedListener = null;
        }

        if (this._removeSerialErrorListener) {
            this._removeSerialErrorListener();
            this._removeSerialErrorListener = null;
        }
        this._removeSerialDataListener =
            window.serialBridge.onData((bytes) => {
                for (const byte of bytes) {
                    this.parser.feedByte(byte);
                }
            });

        this._removeSerialClosedListener =
            window.serialBridge.onClosed(() => {
                if (this._intentionalDisconnect) {
                    return;
                }

                this._handleUnexpectedDrop().catch((err) => {
                    console.error(
                        '[RadioMavlink] Unexpected drop handler error:',
                        err
                    );
                });
            });

        this._removeSerialErrorListener =
            window.serialBridge.onError((msg) => {
                console.error(
                    '[RadioMavlink] Serial error:',
                    msg
                );
            });

        this._transportOpen = true;
        this._connectStartedAt = Date.now();

        console.log(
            '[RadioMavlink] Serial transport terbuka, menunggu HEARTBEAT FC...'
        );

        this._startWatchdog();
        this._startRcOverrideLoop();
    }

    // JANGAN HAPUS async _readLoop()
    // FUNCTION INI KETIKA DINYALAKAN APP TIDAK BISA CONNECT, BEGITU SEBALIKNYA KETIKA INI DIMATIKAN WEBSITE TIDAK BISA CONNECT
    // JIKA INGIN MELAKUKAN PERUBAHAN PADA WEBSITE DAN HARUS DEPLOY MAKA NYALAKAN DULU FUNCTION INI AGAR BISA CONNECT WEBSITENYA
    // JIKA INGIN MELAKUKAN BUILD APLIKASI MAKA FUNCTION INI HARUS DIMATIKAN AGAR APLIKASI BISA CONNECT KE SERIAL PORT
    // async _readLoop() {
    //     if (!this.port || !this.port.readable) {
    //         console.warn('[RadioMavlink] Web Serial readable stream tidak tersedia.');
    //         this.keepReading = false;
    //         return;
    //     }

    //     while (this.keepReading && this.port && this.port.readable) {
    //         try {
    //             this.reader = this.port.readable.getReader();

    //             while (this.keepReading) {
    //                 const { value, done } = await this.reader.read();

    //                 if (done) {
    //                     break;
    //                 }

    //                 if (value) {
    //                     for (const byte of value) {
    //                         this.parser.feedByte(byte);
    //                     }
    //                 }
    //             }

    //         } catch (e) {
    //             if (this.keepReading) {
    //                 console.error('[RadioMavlink] Web Serial read error:', e);
    //             }
    //         } finally {
    //             if (this.reader) {
    //                 try {
    //                     this.reader.releaseLock();
    //                 } catch (e) {}

    //                 this.reader = null;
    //             }
    //         }
    //     }
    // }

    _startWatchdog() {
        this._stopWatchdog();

        this._watchdogTimer = setInterval(() => {
            if (!this._transportOpen) return;
            if (!this._linkReady) {
                const elapsed = Date.now() - this._connectStartedAt;
                if (elapsed > 15000) {
                    console.warn(
                        '[RadioMavlink] Tidak ada HEARTBEAT selama 15 detik setelah serial open. Memulai reconnect transport.'
                    );
                    this._isRebooting = false;
                    this._handleUnexpectedDrop().catch(() => {});
                }
                return;
            }

            if (!this.isLive()) {
                console.warn(
                    '[RadioMavlink] Heartbeat timeout — link dianggap putus.'
                );
                this._isRebooting = false;
                this._handleUnexpectedDrop().catch(() => {});
            }
        }, 1000);
    }

    _stopWatchdog() {
        if (this._watchdogTimer) { clearInterval(this._watchdogTimer); this._watchdogTimer = null; }
    }

    // before
    // _startRcOverrideLoop() {
    //     this._stopRcOverrideLoop();
    //     this._rcOverrideTimer = setInterval(() => {
    //         if (this.telemetry.connected && this.rcChannels.some(ch => ch !== 65535)) {
    //             const payload = [];
    //             for (let i = 0; i < 8; i++) {
    //                 const ch = this.rcChannels[i];
    //                 payload.push(ch & 0xFF, (ch >> 8) & 0xFF);
    //             }
    //             payload.push(this.targetSystem, this.targetComponent);
    //             this._sendFrame(70, payload, CRC_EXTRA[70]).catch(()=>{});
    //         }
    //     }, 150);
    // }

    // after
    // 6) TAMBAHKAN di _startRcOverrideLoop() agar saat latch aktif CH5 dipaksa terus
    _startRcOverrideLoop() {
        this._stopRcOverrideLoop();
        this._rcOverrideTimer = setInterval(() => {
            if (!this.telemetry.connected) return;

            // paksa CH5 kalau latch aktif
            if (this._hybridOverride?.active) {
                this.rcChannels[this._hybridOverride.channel] = this._hybridOverride.targetPwm;
            }

            if (this.rcChannels.some(ch => ch !== 65535)) {
                const payload = [];
                for (let i = 0; i < 8; i++) {
                    const ch = this.rcChannels[i];
                    payload.push(ch & 0xFF, (ch >> 8) & 0xFF);
                }
                payload.push(this.targetSystem, this.targetComponent);
                this._sendFrame(70, payload, CRC_EXTRA[70]).catch(() => {});
            }
        }, 150);
    }

    _stopRcOverrideLoop() {
        if (this._rcOverrideTimer) {
            clearInterval(this._rcOverrideTimer);
            this._rcOverrideTimer = null;
        }
    }

    pauseRcOverride() {this._stopRcOverrideLoop();}
    resumeRcOverride() {this._startRcOverrideLoop();}

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
        const builder = useV2 ? buildMavlink2Frame : buildMavlink1Frame;
        const frame = builder(this._txSeq++, this._gcsSysId, this._gcsCompId, msgId, payloadBytes, crcExtra);
        // try { await window.serialBridge.send(Array.from(frame)); } catch (e) {}
        try {
            if (this._useWebSerial) {
                if (!this._webSerialWriter) {
                    throw new Error('Web Serial writer belum tersedia.');
                }

                await this._webSerialWriter.write(frame);
                return;
            }

            await window.serialBridge.send(Array.from(frame));

        } catch (e) {
            console.error('[RadioMavlink] Gagal mengirim MAVLink frame:', e);
        }
    }

    async requestDataStreams(rateHz = STREAM_RATE_HZ) {
        for (const streamId of REQUESTED_STREAMS) {
            const rateBytes = [rateHz & 0xFF, (rateHz >> 8) & 0xFF];
            const payload = [...rateBytes, this.targetSystem, this.targetComponent, streamId, 1];
            await this._sendFrame(66, payload, CRC_EXTRA[66]);
            await new Promise(r => setTimeout(r, 50));
        }
        try {
            await this.requestRawRcChannels(10);
        } catch (e) {
            console.warn('[RadioMavlink] Gagal request RC_CHANNELS_RAW:', e);
        }
        try {
            await this.requestServoOutput(10);
        } catch (e) {
            console.warn('[RadioMavlink] Gagal request SERVO_OUTPUT_RAW:', e);
        }
        console.log(`[RadioMavlink] Streams requested @ ${rateHz}Hz`);
    }

    async requestRawRcChannels(rateHz = 10) {
        const rate = Math.max(1, Math.min(20, Number(rateHz) || 10));
        const intervalUs = Math.round(1_000_000 / rate);
        console.log(`[RadioMavlink] Request RC_CHANNELS_RAW(35) @ ${rate}Hz`);
        // ID 65
        await this.sendCommandLong(511, {p1: 65, p2: intervalUs}, false);
        // ID 35
        await this.sendCommandLong(511, {p1: 35, p2: intervalUs}, false);
    }    

    async requestServoOutput(rateHz = 10) {
        const rate = Math.max(1, Math.min(20, Number(rateHz) || 10));
        const intervalUs = Math.round(1_000_000 / rate);
        console.log(`[RadioMavlink] Request SERVO_OUTPUT_RAW(36) @ ${rate}Hz`);
        // MAV_CMD_SET_MESSAGE_INTERVAL (511): p1 = message id, p2 = interval (us)
        await this.sendCommandLong(511, {p1: 36, p2: intervalUs}, false);
    }

    async requestHomePosition() {
        try {
            await this.sendCommandLong(410, {}, false);
            console.log('[RadioMavlink] MAV_CMD_GET_HOME_POSITION dikirim');
        } catch (e) {
            console.warn('[RadioMavlink] Gagal minta HOME_POSITION:', e);
        }
    }

    async sendCommandAck(command, result = 0) {
        const buf = new ArrayBuffer(10);
        const dv = new DataView(buf);
        dv.setUint16(0, command, true);            // command
        dv.setUint8(2, result);                    // result (0 = MAV_RESULT_ACCEPTED)
        dv.setUint8(3, 255);                       // progress -- 255 = N/A
        dv.setInt32(4, 0, true);                   // result_param2 -- tidak dipakai
        dv.setUint8(8, this.targetSystem);         // target_system -- WAJIB, bukan 0
        dv.setUint8(9, this.targetComponent);      // target_component -- WAJIB, bukan 0

        const payload = Array.from(new Uint8Array(buf));
        await this._sendFrame(77, payload, CRC_EXTRA[77], true);
        console.log(`[RadioMavlink] COMMAND_ACK sent for CMD ${command} -> target_system=${this.targetSystem}, target_component=${this.targetComponent}`);
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

            const stale = this._ackWaiters.get(command);      // BARU
            if (stale) {                                       // BARU
                clearTimeout(stale.timer);                      // BARU
                stale.resolve({ ok: false, result: 'superseded' }); // BARU
            }                                                    // BARU

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
            const waiter = { resolve, reject, params: {}, total: null, count: 0, onProgress, timer: null, resendTimer: null };
            
            waiter._resetTimer = () => {
                if (waiter.timer) clearTimeout(waiter.timer);
                // Matikan resend jika paket parameter pertama sudah masuk
                if (waiter.resendTimer && waiter.count > 0) {
                    clearInterval(waiter.resendTimer);
                    waiter.resendTimer = null;
                }
                
                waiter.timer = setTimeout(() => {
                    const idx = this._paramListWaiters.indexOf(waiter);
                    if (idx !== -1) this._paramListWaiters.splice(idx, 1);
                    if (waiter.resendTimer) clearInterval(waiter.resendTimer);

                    const receivedCount = Object.keys(waiter.params).length;
                    if (receivedCount > 0) {
                        resolve(waiter.params);
                    } else {
                        reject(new Error(`Timeout getAllParams. FC tidak merespons.`));
                    }
                }, timeout);
            };

            waiter._resetTimer();
            this._paramListWaiters.push(waiter);

            // Paksa komponen 1 (Autopilot Utama)
            const requestPayload = [this.targetSystem, 1]; 
            this._sendFrame(21, requestPayload, CRC_EXTRA[21]);

            // Resend otomatis jika FC drop paket request pertama
            waiter.resendTimer = setInterval(() => {
                if (waiter.count === 0) {
                    console.warn('[Mavlink] Resending PARAM_REQUEST_LIST...');
                    this._sendFrame(21, requestPayload, CRC_EXTRA[21]);
                }
            }, 1000);
        });
    }

    async uploadMission(items, timeout = 10000) {
        const count = items.length;
        if (this._missionUpload) throw new Error('Upload mission sedang berlangsung, tunggu selesai');

        this._missionBusy = true;
        this.pauseRcOverride();

        await this._sendFrame(45, [this.targetSystem, this.targetComponent], CRC_EXTRA[45]);

        const p = new Promise((resolve, reject) => {
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

        return p.finally(() => {
            this._missionBusy = false;
            this.resumeRcOverride();
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
        this._missionBusy = true;
        this.pauseRcOverride();
        try {
            return await this._downloadMissionInner(timeout, maxRetries);
        } finally {
            this._missionBusy = false;
            this.resumeRcOverride();
        }
    }

    async _downloadMissionInner(timeout, maxRetries) {
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
            this._emitTelemetrySafe();
        }
        await this.sendCommandLong(
            246,
            { p1: 1 },
            false
        );
        return {
            ok: true,
            result: null
        };
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

    _normalizeHybridState(state) {
        const value = String(state ?? '').toLowerCase();

        if (['rover', 'low'].includes(value)) {
            return 'rover';
        }

        if (['copter_safe', 'mid', 'middle'].includes(value)) {
            return 'copter_safe';
        }

        if (['copter_unsafe', 'high', 'ready'].includes(value)) {
            return 'copter_unsafe';
        }

        return null;
    }

    _getHybridStateFromRc(pwm) {
        const value = Number(pwm);

        if (!Number.isFinite(value)) {
            return null;
        }

        // Mapping CH5 transmitter
        if (value <= 1250) {
            return 'rover';
        }

        if (value >= 1750) {
            return 'copter_unsafe';
        }

        return 'copter_safe';
    }

    _isHybridVehicleTypeCompatible(targetState) {
        const vehicleType = String(
            this.telemetry?.vehicle_type ?? ''
        ).toUpperCase();

        // Kalau vehicle_type belum tersedia,
        // jangan blokir proses release.
        if (!vehicleType) {
            return true;
        }

        if (targetState === 'rover') {
            return vehicleType === 'ROVER';
        }

        if (
            targetState === 'copter_safe' ||
            targetState === 'copter_unsafe'
        ) {
            return vehicleType === 'COPTER';
        }

        return true;
    }

    // before
    // _resetHybridOverride() {
    //     if (!this._hybridOverride) return;

    //     if (this._hybridOverride.releaseTimer) {
    //         clearTimeout(this._hybridOverride.releaseTimer);
    //         this._hybridOverride.releaseTimer = null;
    //     }

    //     this._hybridOverride.active = false;
    //     this._hybridOverride.targetPwm = 65535;
    //     this._hybridOverride.targetState = null;
    //     this._hybridOverride.startedAt = 0;

    //     this._hybridOverride.initialRemoteState = null;
    //     this._hybridOverride.matchStableCount = 0;
    //     this._hybridOverride.changeStableCount = 0;
    //     this._hybridOverride.matchedOnce = false;
    //     this._hybridOverride.lastPhysicalState = null;
    // }
    // after
    _resetHybridOverride() {
        if (!this._hybridOverride) return;
        this._hybridOverride.active = false;
        this._hybridOverride.channel = 4;
        this._hybridOverride.targetPwm = 65535;
        this._hybridOverride.targetState = null;
    }

    // before
    // async setHybridModeTemporary(targetState, targetPwm, channel = 4) {
    //     const normalized = this._normalizeHybridState(targetState);

    //     if (!normalized) {
    //         throw new Error(
    //             `Hybrid target state tidak valid: ${targetState}`
    //         );
    //     }

    //     if (!Number.isFinite(Number(targetPwm))) {
    //         throw new Error(
    //             `Hybrid target PWM tidak valid: ${targetPwm}`
    //         );
    //     }

    //     if (channel < 0 || channel > 7) {
    //         throw new Error(`Channel ${channel} tidak valid.`);
    //     }

    //     const remotePwm = Number(
    //         this.telemetry?.[`rc${channel + 1}_raw`]
    //     );

    //     const remoteState = this._getHybridStateFromRc(remotePwm);

    //     this._hybridOverride.active = true;
    //     this._hybridOverride.channel = channel;
    //     this._hybridOverride.targetPwm = Number(targetPwm);
    //     this._hybridOverride.targetState = normalized;
    //     this._hybridOverride.startedAt = Date.now();
    //     this._hybridOverride.minHoldMs = 800;
    //     this._hybridOverride.stableMatches = 0;
    //     this._hybridOverride.lastRemoteState = remoteState;
    //     this._hybridOverride.initialRemoteState = remoteState;

    //     console.log(
    //         `[HybridOverride] START target=${normalized} pwm=${targetPwm} ` +
    //         `| remote_rc${channel + 1}=` +
    //         `${Number.isFinite(remotePwm) ? remotePwm : 'N/A'} ` +
    //         `(${remoteState ?? 'unknown'})`
    //     );

    //     // Ambil alih CH5 sekarang.
    //     // RC override loop akan mempertahankan nilai ini.
    //     await this.sendRcOverride(
    //         channel,
    //         Number(targetPwm)
    //     );

    //     this._checkHybridOverrideRelease();

    //     return {
    //         ok: true,
    //         temporary: true,
    //         targetState: normalized,
    //         targetPwm: Number(targetPwm),
    //         remoteState
    //     };
    // }


    // after
    // async setHybridModeTemporary(targetState, targetPwm, channel = 4) {
    //     const normalized = this._normalizeHybridState(targetState);

    //     if (!normalized) {
    //         throw new Error(`Hybrid target state tidak valid: ${targetState}`);
    //     }

    //     if (!Number.isFinite(Number(targetPwm))) {
    //         throw new Error(`Hybrid target PWM tidak valid: ${targetPwm}`);
    //     }

    //     if (channel < 0 || channel > 7) {
    //         throw new Error(`Channel ${channel} tidak valid.`);
    //     }

    //     const remotePwm = Number(
    //         this.telemetry?.[`rc${channel + 1}_raw`] ?? this.telemetry?.rc5_physical
    //     );
    //     const remoteState = this._getHybridStateFromRc(remotePwm);

    //     // Start/refresh latch
    //     this._hybridOverride.active = true;
    //     this._hybridOverride.channel = channel;
    //     this._hybridOverride.targetPwm = Number(targetPwm);
    //     this._hybridOverride.targetState = normalized;
    //     this._hybridOverride.startedAt = Date.now();

    //     this._hybridOverride.initialRemoteState = remoteState;
    //     this._hybridOverride.matchStableCount = 0;
    //     this._hybridOverride.changeStableCount = 0;
    //     this._hybridOverride.matchedOnce = false;
    //     this._hybridOverride.lastPhysicalState = remoteState;

    //     // Failsafe timer: jangan lock selamanya kalau RC hilang total
    //     if (this._hybridOverride.releaseTimer) {
    //         clearTimeout(this._hybridOverride.releaseTimer);
    //     }
    //     this._hybridOverride.releaseTimer = setTimeout(() => {
    //         if (this._hybridOverride?.active) {
    //             console.warn('[HybridOverride] Failsafe release (maxHoldMs reached)');
    //             this._releaseHybridOverride().catch(() => {});
    //         }
    //     }, this._hybridOverride.maxHoldMs);

    //     console.log(
    //         `[HybridOverride] START target=${normalized} pwm=${targetPwm} ` +
    //         `| remote_rc${channel + 1}=${Number.isFinite(remotePwm) ? remotePwm : 'N/A'} ` +
    //         `(${remoteState ?? 'unknown'})`
    //     );

    //     // Ambil alih CH5 sekarang; loop override akan mempertahankan
    //     await this.sendRcOverride(channel, Number(targetPwm));

    //     return {
    //         ok: true,
    //         temporary: true,
    //         targetState: normalized,
    //         targetPwm: Number(targetPwm),
    //         remoteState
    //     };
    // }
    // after after
    // 3) GANTI setHybridModeTemporary() -> jadi pure latch (tanpa auto-release)
    async setHybridModeTemporary(targetState, targetPwm, channel = 4) {
        const normalized = this._normalizeHybridState(targetState);
        if (!normalized) throw new Error(`Hybrid target state tidak valid: ${targetState}`);
        if (!Number.isFinite(Number(targetPwm))) throw new Error(`Hybrid target PWM tidak valid: ${targetPwm}`);
        if (channel < 0 || channel > 7) throw new Error(`Channel ${channel} tidak valid.`);

        this._hybridOverride.active = true;
        this._hybridOverride.channel = channel;
        this._hybridOverride.targetPwm = Number(targetPwm);
        this._hybridOverride.targetState = normalized;

        // kirim segera
        await this.sendRcOverride(channel, Number(targetPwm));

        console.log(`[HybridOverride] LATCH ON CH${channel + 1} -> ${targetPwm} (${normalized})`);

        return {
            ok: true,
            latched: true,
            targetState: normalized,
            targetPwm: Number(targetPwm),
            channel: channel + 1
        };
    }

    // before
    // _checkHybridOverrideRelease() {
    //     const state = this._hybridOverride;
    //     if (!state?.active) return false;

    //     const remotePwm = Number(
    //         this.telemetry?.[`rc${state.channel + 1}_raw`]
    //     );
    //     const remoteState = this._getHybridStateFromRc(remotePwm);

    //     const matchesTarget =
    //         remoteState === state.targetState &&
    //         this._isHybridVehicleTypeCompatible(state.targetState);

    //     const operatorMovedAway =
    //         remoteState !== null &&
    //         remoteState !== state.initialRemoteState;

    //     const shouldTrack = matchesTarget || operatorMovedAway;

    //     if (shouldTrack) {
    //         state.stableMatches =
    //             state.lastRemoteState === remoteState
    //                 ? state.stableMatches + 1
    //                 : 1;
    //     } else {
    //         state.stableMatches = 0;
    //     }
    //     state.lastRemoteState = remoteState;

    //     const heldLongEnough =
    //         Date.now() - state.startedAt >= state.minHoldMs;
    //     const stableEnough =
    //         state.stableMatches >= state.requiredStableMatches;

    //     if (heldLongEnough && stableEnough) {
    //         this._releaseHybridOverride();
    //         return true;
    //     }

    //     return false;
    // }

    // after
    _checkHybridOverrideRelease() {
        // const state = this._hybridOverride;
        // if (!state?.active) return false;

        // const physicalPwm = Number(
        //     this.telemetry?.[`rc${state.channel + 1}_raw`] ??
        //     this.telemetry?.rc5_physical
        // );
        // const physicalState = this._getHybridStateFromRc(physicalPwm);

        // // Tidak ada data RC valid -> tetap latch (biar mode tidak mental balik)
        // if (physicalState === null) {
        //     state.matchStableCount = 0;
        //     state.changeStableCount = 0;
        //     return false;
        // }

        // // Stage 1: operator harus samakan dulu switch fisik ke mode yang sedang dilatch GCS
        // if (!state.matchedOnce) {
        //     if (physicalState === state.targetState) {
        //         state.matchStableCount =
        //             (state.lastPhysicalState === physicalState)
        //                 ? state.matchStableCount + 1
        //                 : 1;

        //         if (state.matchStableCount >= state.requiredStableMatches) {
        //             state.matchedOnce = true;
        //             state.changeStableCount = 0;
        //             console.log(
        //                 `[HybridOverride] Catch-up OK: remote sudah match target (${state.targetState}). ` +
        //                 `Silakan pindah switch untuk takeover.`
        //             );
        //         }
        //     } else {
        //         state.matchStableCount = 0;
        //     }

        //     state.lastPhysicalState = physicalState;
        //     return false;
        // }

        // // Stage 2: setelah match, tunggu operator ubah ke state lain (intentional takeover)
        // if (physicalState !== state.targetState) {
        //     state.changeStableCount =
        //         (state.lastPhysicalState === physicalState)
        //             ? state.changeStableCount + 1
        //             : 1;

        //     if (state.changeStableCount >= state.requiredStableMatches) {
        //         this._releaseHybridOverride().catch(() => {});
        //         state.lastPhysicalState = physicalState;
        //         return true;
        //     }
        // } else {
        //     state.changeStableCount = 0;
        // }

        // state.lastPhysicalState = physicalState;
        return false;
    }

    // before
    // async _releaseHybridOverride() {
    //     const state = this._hybridOverride;

    //     if (!state?.active) {
    //         return {
    //             ok: true,
    //             alreadyReleased: true
    //         };
    //     }

    //     const channel = state.channel;
    //     const targetState = state.targetState;

    //     console.log(
    //         `[HybridOverride] RELEASE -> remote CH${channel + 1} ` +
    //         `sudah berada di ${targetState}`
    //     );

    //     // Matikan state terlebih dahulu agar loop
    //     // berhenti mempertahankan override.
    //     state.active = false;
    //     state.targetState = null;
    //     state.targetPwm = 65535;
    //     state.stableMatches = 0;
    //     state.lastRemoteState = null;

    //     // 65535 = release / no override
    //     await this.sendRcOverride(
    //         channel,
    //         65535
    //     );

    //     return {
    //         ok: true,
    //         released: true,
    //         channel,
    //         targetState
    //     };
    // }

    // after
    async _releaseHybridOverride() {
        const state = this._hybridOverride;

        if (!state?.active) {
            return { ok: true, alreadyReleased: true };
        }

        const channel = state.channel;
        const targetState = state.targetState;

        console.log(
            `[HybridOverride] RELEASE CH${channel + 1} ` +
            `(latch target sebelumnya: ${targetState}) -> kembali ke RC fisik`
        );

        this._resetHybridOverride();

        // 65535 = release / no override
        await this.sendRcOverride(channel, 65535);

        return {
            ok: true,
            released: true,
            channel,
            targetState
        };
    }

    // before
    // async cancelHybridOverride() {
    //     const state = this._hybridOverride;

    //     const channel = state?.channel ?? 4;
    //     const wasActive = !!state?.active;

    //     this._resetHybridOverride();

    //     // Pastikan FC menerima release override.
    //     await this.sendRcOverride(
    //         channel,
    //         65535
    //     );

    //     console.log(
    //         `[HybridOverride] CANCEL/RELEASE CH${channel + 1} ` +
    //         `-> remote kembali menjadi sumber kontrol`
    //     );

    //     return {
    //         ok: true,
    //         cancelled: wasActive,
    //         channel
    //     };
    // }

    // after
    // async cancelHybridOverride() {
    //     const state = this._hybridOverride;
    //     const channel = state?.channel ?? 4;
    //     const wasActive = !!state?.active;

    //     this._resetHybridOverride();

    //     await this.sendRcOverride(channel, 65535);

    //     console.log(
    //         `[HybridOverride] CANCEL/RELEASE CH${channel + 1} -> remote kembali jadi sumber kontrol`
    //     );

    //     return { ok: true, cancelled: wasActive, channel };
    // }

    // async sendRcOverride(chIndex, pwmValue) {
    //     if (chIndex < 0 || chIndex > 7) {
    //         throw new Error('sendRcOverride hanya mendukung channel 1-8 (index 0-7) -- base RC_CHANNELS_OVERRIDE MAVLink1');
    //     }
    //     this.rcChannels[chIndex] = pwmValue;

    //     console.log(`[RC_OVERRIDE] ch${chIndex + 1} -> ${pwmValue}us | terkirim:`, [...this.rcChannels]);

    //     const buildPayload = () => {
    //         const payload = [];
    //         for (let i = 0; i < 8; i++) {
    //             const ch = this.rcChannels[i];
    //             payload.push(
    //                 ch & 0xFF, (ch >> 8) & 0xFF);
    //         }
    //         payload.push(this.targetSystem, this.targetComponent);
    //         return payload;
    //     };
    //     await this._sendFrame(70, buildPayload(), CRC_EXTRA[70]);
    //     for (let i = 0; i < 3; i++) {
    //         await new Promise(r => setTimeout(r, 50));
    //         this._sendFrame(70, buildPayload(), CRC_EXTRA[70]).catch(()=>{});
    //     }    
    // }

    async sendRcOverride(chIndex, pwmValue) {
        if (chIndex < 0 || chIndex > 7) {
            throw new Error('sendRcOverride hanya mendukung channel 1-8');
        }
        this.rcChannels[chIndex] = pwmValue;
        console.log(`[RC_OVERRIDE] ch${chIndex + 1} -> ${pwmValue}us`);
    }

    // after
    // 5) GANTI cancelHybridOverride()
    async cancelHybridOverride() {
        const ch = this._hybridOverride?.channel ?? 4;
        const wasActive = !!this._hybridOverride?.active;

        this._resetHybridOverride();
        await this.sendRcOverride(ch, 65535); // release

        console.log(`[HybridOverride] RELEASE CH${ch + 1} -> RC fisik aktif`);

        return {
            ok: true,
            released: true,
            wasActive,
            channel: ch + 1
        };
    }

    async calibrateMag(compassId = -1) {
        return this.sendCommandLong(241, { p3: 1, p4: compassId >= 0 ? compassId : 0 });
    }
    clearMagCalState() {
        this.telemetry.mag_cal_progress = {};
        this.telemetry.mag_cal_report   = {};
        window.compassSuccessMask = 0;
        this._emitTelemetrySafe();
    }
    async sendCommandLongRetry(command, params = {}, opts = {}) {
        const { retries = 3, timeout = 3000, gapMs = 250 } = opts;
        let last = null;
        for (let i = 0; i <= retries; i++) {
            last = await this.sendCommandLong(command, params, true, timeout);
            if (last && last.ok) return last;
            const r = last ? last.result : 'timeout';
            const retryable = (r === 'timeout' || r === 1 || r === 5);
            if (!retryable) return last;
            if (i < retries) await new Promise(res => setTimeout(res, gapMs));
        }
        return last;
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

    _emitTelemetrySafe() {
        if (!this.onTelemetry) return;

        try {
            this.onTelemetry({
                ...this.telemetry,
                link: 'radio'   // penanda sumber, dipakai routeTelemetry() di index.html
            });
        } catch (err) {
            console.error(
                '[RadioMavlink] onTelemetry callback error:',
                err
            );
        }
    }

    _scheduleMagCalTelemetry() {
        this._magCalTelemetryPending = true;

        if (this._magCalTelemetryTimer) {
            return;
        }

        this._magCalTelemetryTimer = setTimeout(() => {
            this._magCalTelemetryTimer = null;

            if (!this._magCalTelemetryPending) {
                return;
            }

            this._magCalTelemetryPending = false;
            this._emitTelemetrySafe();
        }, 150);
    }

    _handleRawRcChannels(payload) {
        if (!payload || payload.length < 20) {
            return;
        }

        try {
            const bytes = Uint8Array.from(payload);
            const dv = new DataView(
                bytes.buffer,
                bytes.byteOffset,
                bytes.byteLength
            );
            for (let i = 0; i < 8; i++) {
                this._physicalRc[i] =
                    dv.getUint16(
                        4 + (i * 2),
                        true
                    );
            }

            // FIX: sebelumnya cuma channel 5 yang diteruskan ke this.telemetry,
            // channel 1-4/6-8 dibaca ke _physicalRc[] lalu dibuang. Sekarang
            // semua 8 channel diekspos dengan nama field yang sama persis
            // dengan yang dicari poller Radio Calibration di config.html
            // (data['rc' + ch + '_raw']).
            for (let i = 0; i < 8; i++) {
                const raw = this._physicalRc[i];
                if (Number.isFinite(raw) && raw !== 65535) {
                    this.telemetry[`rc${i + 1}_raw`] = raw;
                }
            }

            const rc5 = this._physicalRc[4];
            if (!Number.isFinite(rc5) || rc5 === 65535) {
                this._emitTelemetrySafe();
                return;
            }
            this._physicalRc5 = rc5;
            this._physicalRc5Seq++;
            this._physicalRc5LastUpdate = Date.now();
            this.telemetry.rc5_physical = rc5;
            this.telemetry.hybrid_remote_state = this._getHybridStateFromRc(rc5);
            this._checkHybridOverrideRelease();
            // FIX: tanpa ini, update rc*_raw & rc5_physical cuma "menumpang"
            // terkirim ke UI kalau kebetulan ada message lain yang trigger
            // emit -- tidak reliable dan bikin bar kalibrasi terasa macet.
            this._emitTelemetrySafe();
        } catch (error) {
            console.error(
                '[RadioMavlink] Gagal decode RC_CHANNELS_RAW:',
                error
            );
        }
    }

    _handleFrame(msgId, payload, sysid, compid) {
        const isSelf = (sysid === this._gcsSysId && compid === this._gcsCompId);

        if (msgId === 0 && !isSelf) {
            const firstHeartbeat = !this._streamsRequested;

            if (firstHeartbeat) {
                console.log(
                    `[RadioMavlink] HEARTBEAT diterima: sysid=${sysid}, compid=${compid} -- target dikunci ke compid=${MAV_COMP_ID_AUTOPILOT1} (autopilot).`
                );

                this.targetSystem = sysid;
                // FIX: paksa compid ke autopilot asli (FCU_COMP_ID di firmware), jangan ikut
                // race "siapa duluan" -- ESP32 companion juga kirim HEARTBEAT sendiri
                // (compid MAV_COMP_ID_ONBOARD_COMPUTER) dan bisa menang race ini.
                this.targetComponent = MAV_COMP_ID_AUTOPILOT1;
                this._streamsRequested = true;
                this.requestDataStreams()
                .then(() => {
                    console.log('[RadioMavlink] requestDataStreams done');
                })
                .catch(e => {
                    console.warn(
                        '[RadioMavlink] requestDataStreams gagal:',
                        e
                    );
                });
                this.requestHomePosition();
            }

            if (
                sysid === this.targetSystem &&
                Number(compid) === MAV_COMP_ID_AUTOPILOT1
            ) {
                const wasReady = this._linkReady;
                this.lastHeartbeatTime = Date.now();
                this._linkReady = true;
                this._transportOpen = true;
                this.telemetry.connected = true;
                this.telemetry.system_status = 'connected';
                if (payload && payload.length >= 9) {
                    const bytes = Uint8Array.from(payload);
                    const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);

                    const customMode = dv.getUint32(0, true);
                    const vehicleTypeCode = bytes[4];
                    const baseMode = bytes[6];
                    const mavState = bytes[7];

                    const MAV_MODE_FLAG_SAFETY_ARMED = 0x80;
                    const armedNow = (baseMode & MAV_MODE_FLAG_SAFETY_ARMED) !== 0;

                    this.telemetry.armed = armedNow;
                    this.telemetry.base_mode = baseMode;
                    this.telemetry.custom_mode = customMode;
                    this.telemetry.vehicle_type_code = vehicleTypeCode;
                    this.telemetry.mav_state = mavState;

                    this._emitTelemetrySafe();
                }

                if (this._isRebooting) {
                    console.log(
                        `[RadioMavlink] FC kembali setelah reboot: sysid=${sysid}, compid=${compid}`
                    );
                    this._isRebooting = false;
                }

                if (!wasReady && this.onConnect) {
                    try {
                        this.onConnect();
                    } catch (err) {
                        console.error(
                            '[RadioMavlink] onConnect callback error:',
                            err
                        );
                    }
                }
            } else if (sysid === this.targetSystem) {
                // Heartbeat dari companion (ESP32, compid != MAV_COMP_ID_AUTOPILOT1)
                // -- dipakai buat deteksi "ESP32 masih hidup", TIDAK dipakai untuk arm/mode
                this.telemetry.esp32_alive = true;
                this.telemetry.esp32_last_heartbeat = Date.now();
            }
            return;
        }

        if (isSelf) return;
        // if (this._streamsRequested && sysid !== this.targetSystem) return;
        // if (this._streamsRequested && (sysid !== this.targetSystem || compid !== this.targetComponent)) return;
        if (this._streamsRequested) {
            if (sysid !== this.targetSystem) return;
            if (compid !== this.targetComponent && msgId !== 253) return;
        }

        if (msgId === 35) {
            this._handleRawRcChannels(payload);
            return;
        }

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
            this._checkHybridOverrideRelease();
            this._emitTelemetrySafe();
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
            if (this.onTelemetry) this.onTelemetry({ ...this.telemetry, link: 'radio' });
        } else if (result._kind === 'mag_cal_progress') {
            if (!this.telemetry.mag_cal_progress) this.telemetry.mag_cal_progress = {};
            const id      = Number(result.compass_id);
            const attempt = Number(result.attempt) || 1;
            const prev    = this.telemetry.mag_cal_progress[id];

            if (prev && attempt > (prev.attempt || 1)) {
                if (this.telemetry.mag_cal_report) delete this.telemetry.mag_cal_report[id];
                window.compassSuccessMask = (window.compassSuccessMask || 0) & ~(1 << id);
            }

            this.telemetry.mag_cal_progress[id] = {
                id,
                pct:        Number(result.completion_pct) || 0,
                cal_status: Number(result.cal_status) || 0,
                attempt,
                done:       false
            };
            this._scheduleMagCalTelemetry();

        } else if (result._kind === 'mag_cal_report') {
            const MAG_CAL_SUCCESS = 4;
            const id     = Number(result.compass_id);
            const status = Number(result.cal_status);

            if (!this.telemetry.mag_cal_report)   this.telemetry.mag_cal_report   = {};
            if (!this.telemetry.mag_cal_progress) this.telemetry.mag_cal_progress = {};

            this.telemetry.mag_cal_report[id] = {
                id, status,
                fitness:   Number(result.fitness),
                autosaved: Number(result.autosaved) || 0,
                ts:        Date.now()
            };

            const attempt = this.telemetry.mag_cal_progress[id]?.attempt || 1;

            if (status === MAG_CAL_SUCCESS) {
                window.compassSuccessMask = (window.compassSuccessMask || 0) | (1 << id);
                this.telemetry.mag_cal_progress[id] = { id, pct: 100, cal_status: status, attempt, done: true };
            } else {
                window.compassSuccessMask = (window.compassSuccessMask || 0) & ~(1 << id);
                this.telemetry.mag_cal_progress[id] = { id, pct: 0, cal_status: status, attempt, done: true };
            }
            this._emitTelemetrySafe();
        }
    }

    _handleCommandAck(command, result) {
        const waiter = this._ackWaiters.get(command);
        if (waiter) {
            this._ackWaiters.delete(command);
            clearTimeout(waiter.timer);
            waiter.resolve({ ok: result === 0, result, message: mavResultMessage(result) });
        }
    }

    _handleCommandLong({ command, param1 }) {
        const MAV_CMD_ACCELCAL_VEHICLE_POS = 42429;
        if (command === MAV_CMD_ACCELCAL_VEHICLE_POS) {
            console.log(`[RadioMavlink] COMMAND_LONG accelcal diterima dari FC (param1=${param1})`);
            if (this.onAccelCalPosSignal) this.onAccelCalPosSignal(param1);
        } else {
            console.log(`[RadioMavlink] COMMAND_LONG diterima dari FC (command=${command}, param1=${param1})`);
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
            entry.timer = setTimeout(flush, 800);
        }
    }cd 

    _emitStatustext(text, severity) {
        this.logSeq++;
        const logItem = { seq: this.logSeq, text, severity, server_time: Date.now() / 1000, source: 'radio' };
        if (this.onMavlog) this.onMavlog(logItem);
    }
}

window.RadioMavlink = RadioMavlink;
console.log('[RadioMavlink] radio-mavlink.js loaded (full version)');