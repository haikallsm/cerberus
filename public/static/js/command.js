/**
 * commands.js — API tingkat tinggi untuk perintah ke FC.
 * Semua perintah di-routing ke link aktif (radio serial atau UDP).
 */
const commands = {
    // ------------------------------------------------------------------
    // Link selection
    // ------------------------------------------------------------------
    _radioActive() {
        return !!(window.radioMavlink && window.radioMavlink.isLive());
    },

    _udpActive() {
        return !!(window.udpMavlink && window.udpMavlink.isLive());
    },

    _getActiveLink() {
        if (this._radioActive()) return window.radioMavlink;
        if (this._udpActive()) return window.udpMavlink;
        throw new Error('Tidak ada koneksi aktif. Silakan hubungkan Radio atau UDP terlebih dahulu.');
    },

    _retrySender(link) {
        return (typeof link.sendCommandLongRetry === 'function')
            ? link.sendCommandLongRetry.bind(link)
            : (cmd, params) => link.sendCommandLong(cmd, params);
    },

    // ------------------------------------------------------------------
    // Flight commands
    // ------------------------------------------------------------------
    async armDisarm(arm) {
        const ack = await this._getActiveLink().armDisarm(arm);
        if (ack && ack.ok && window.gcsState) {
            window.gcsState.armed = !!arm;
        }
        return ack;
    },

    async takeoff(alt = 1) {
        return this._getActiveLink().takeoff(alt);
    },

    async setFlightMode(mode) {
        return this._getActiveLink().setMode(mode);
    },

    async reboot() {
        return this._getActiveLink().reboot();
    },

    // ------------------------------------------------------------------
    // Safety switch (protokol kustom FC: CMD 176)
    // ------------------------------------------------------------------
    async setSafetySwitch(turnOn) {
        const baseMode = 128.0;
        const customMode = turnOn ? 1.0 : 0.0;

        console.log("[Command] Safety Switch:", turnOn ? "HARDWARE SAFETY" : "UNSAFETY");

        const ack = await this._getActiveLink().sendCommandLong(176, {
            p1: baseMode,
            p2: customMode,
            p3: 0.0,
            p4: 0.0
        });

        if (!ack || !ack.ok) {
            console.error("[Command] Safety Switch Failed", ack);
            return ack;
        }

        if (window.gcsState?.safety) {
            window.gcsState.safety.off = !turnOn;
        }

        console.log("[Command] Safety Switch:", window.gcsState?.safety?.off ? "OFF" : "ON");
        return ack;
    },

    // ------------------------------------------------------------------
    // Hybrid mode (transisi UGV <-> UAV via CH5 override)
    // Menargetkan switch mid/safe saja; mode high/unsafe tidak dipakai GCS.
    // ------------------------------------------------------------------
    async setHybridMode(state) {
        const stateStr = String(state).toLowerCase().trim();
        const link = this._getActiveLink();

        let targetState;
        let pwm;

        if (['low', 'rover'].includes(stateStr)) {
            targetState = 'rover';
            pwm = 999;
        } else if (
            ['mid', 'middle', 'copter', 'copter_safe', 'high', 'copter_unsafe', 'ready'].includes(stateStr)
        ) {
            // copter selalu SAFE (MID)
            targetState = 'copter_safe';
            pwm = 1500;
        } else if (['release', 'off'].includes(stateStr)) {
            if (typeof link.cancelHybridOverride === 'function') {
                return link.cancelHybridOverride();
            }
            return link.sendRcOverride(4, 65535);
        } else {
            throw new Error(`Invalid hybrid mode state: ${state}`);
        }

        if (typeof link.setHybridModeTemporary === 'function') {
            return link.setHybridModeTemporary(targetState, pwm, 4);
        }
        return link.sendRcOverride(4, pwm);
    },

    async setServo(channel, pwm) {
        return this._getActiveLink().setServo(channel, pwm);
    },

    // ------------------------------------------------------------------
    // Parameters & PID
    // ------------------------------------------------------------------
    async getAllParams(onProgress) {
        return this._getActiveLink().getAllParams(onProgress);
    },

    async setParam(name, value, paramType = 9) {
        return this._getActiveLink().setParam(name, value, paramType);
    },

    async getPID(axis) {
        return this._getActiveLink().getPID(axis);
    },

    async setPID(axis, pid) {
        return this._getActiveLink().setPID(axis, pid);
    },

    // ------------------------------------------------------------------
    // Mission & fence
    // ------------------------------------------------------------------
    async uploadMission(items) {
        return this._getActiveLink().uploadMission(items);
    },

    async downloadMission() {
        return this._getActiveLink().downloadMission();
    },

    async setHome(lat, lon, alt = 0) {
        return this._getActiveLink().setHome(lat, lon, alt);
    },

    async uploadFence(points) {
        return this._getActiveLink().uploadFence(points);
    },

    async downloadFence() {
        return this._getActiveLink().downloadFence();
    },

    async clearFence() {
        return this._getActiveLink().clearFence();
    },

    // ------------------------------------------------------------------
    // Calibration — accelerometer (custom CMD 241 / 42429)
    // ------------------------------------------------------------------
    async startAccel3Axis() {
        window.acceptCalStep = 0;
        return this._getActiveLink().sendCommandLong(241, { p5: 1.0 });
    },

    async startAccelLevel() {
        return this._getActiveLink().sendCommandLong(241, { p5: 2.0 });
    },

    async startAccelSimple() {
        return this._getActiveLink().sendCommandLong(241, { p5: 4.0 });
    },

    async nextAccelStep(posValue) {
        return this._getActiveLink().sendCommandLong(42429, { p1: posValue });
    },

    // ------------------------------------------------------------------
    // Calibration — compass (custom CMD 42424/42425/42426)
    // RC override di-pause selama kalibrasi dan SELALU di-resume
    // setelah selesai/gagal supaya remote tidak terkunci.
    // ------------------------------------------------------------------
    async startCompassCal() {
        const link = this._getActiveLink();

        if (typeof link.clearMagCalState === 'function') {
            link.clearMagCalState();
        } else {
            window.compassSuccessMask = 0;
        }

        link.pauseRcOverride();
        link.rcChannels = new Array(8).fill(65535); // release semua channel

        const send = this._retrySender(link);

        // p2: retry=1, p3: autosave=0
        let ack;
        try {
            ack = await send(42424, { p1: 0, p2: 1, p3: 0, p4: 0, p5: 0 },
                             { retries: 2, timeout: 3000 });
        } catch (e) {
            // Command gagal/timeout: kalibrasi tidak berjalan,
            // jangan biarkan override mengunci RC selamanya.
            link.resumeRcOverride();
            throw e;
        }
        if (!ack || !ack.ok) {
            link.resumeRcOverride();
        }
        return ack;
    },

    async cancelCompassCal() {
        const link = this._getActiveLink();
        const send = this._retrySender(link);

        const ack = await send(42426, {}, { retries: 2, timeout: 3000 });
        if (typeof link.clearMagCalState === 'function') link.clearMagCalState();
        else window.compassSuccessMask = 0;
        link.resumeRcOverride();
        return ack;
    },

    async acceptCompassCal() {
        const link = this._getActiveLink();
        const send = this._retrySender(link);

        const mask = window.compassSuccessMask || 0;
        if (mask === 0) throw new Error("Tidak ada compass yang SUCCESS. Silakan kalibrasi ulang.");

        let ack;
        try {
            // resumeRcOverride() dipanggil di finally, setelah ACK diterima
            ack = await send(42425, { p1: mask }, { retries: 3, timeout: 5000 });
        } finally {
            link.resumeRcOverride();
        }

        if (ack && ack.ok) window.compassSuccessMask = 0;
        return ack;
    },
};

window.compassSuccessMask = 0;
window.commands = commands;
