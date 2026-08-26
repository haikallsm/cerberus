const commands = {
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

    async armDisarm(arm) {
        const ack = await this._getActiveLink().armDisarm(arm);
        if(ack && ack.ok) {
            if (window.gcsState) {
                window.gcsState.armed = !!arm;
            }
        } return ack;
    },

    async reboot() {
        return this._getActiveLink().reboot();
    },

    async setFlightMode(mode) {
        return this._getActiveLink().setMode(mode);
    },

    async setSafetySwitch(turnOn) {
        const baseMode = 128.0;
        const customMode = turnOn ? 1.0 : 0.0;

        if (turnOn) {
            console.log("[Command] HARDWARE SAFETY");
        } else {
            console.log("[Command] UNSAFETY");
        }

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

        if(window.gcsState?.safety) {
            window.gcsState.safety.off = !turnOn;
        }

        console.log("[Command] Safety Switch:", window.gcsState?.safety?.off ? "OFF" : "ON");
        return ack;
    },

    async setServo(channel, pwm) {
        return this._getActiveLink().setServo(channel, pwm);
    },

    async setHybridMode(state) {
        const stateStr = String(state).toLowerCase();
        const CH5_INDEX = 4;

        let pwm;
        if(['low', 'rover'].includes(stateStr)) {
            pwm = 1100;
            console.log("[COMMAND] HYBRID MODE: ROVER (LOW)");
        } else if(['mid', 'middle', 'copter_safe'].includes(stateStr)) {
            pwm = 1000;
            console.log("[COMMAND] HYBRID MODE: COPTER & SAFETY (MID)");
        } else if (['high', 'copter_unsafe', 'ready'].includes(stateStr)) {
            pwm = 2000;
            console.log("[COMMAND] HYBRID MODE: COPTER & UNSAFETY (HIGH)");
        } else if (['release', 'off'].includes(stateStr)) {
            pwm = 65535;
            console.log("[COMMAND] HYBRID MODE: RELEASED");
        } else {
            throw new Error(`Invalid hybrid mode state: ${state}`);
        }
        return this._getActiveLink().sendRcOverride(CH5_INDEX, pwm);
    },

    async setHome(lat, lon, alt = 0) {
        return this._getActiveLink().setHome(lat, lon, alt);
    },

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

    async uploadMission(items) {
        return this._getActiveLink().uploadMission(items);
    },

    async downloadMission() {
        return this._getActiveLink().downloadMission();
    },

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

    async startCompassCal() {
        window.compassSuccessMask = 0;
        const link = this._getActiveLink();
        link.pauseRcOverride();
        link.rcChannels = new Array(8).fill(65535);
        return link.sendCommandLong(42424, { p3: 1.0 }); // 42424 = START_MAG_CAL
    },
    
    async cancelCompassCal() {
        const link = this._getActiveLink();
        link.resumeRcOverride();
        return link.sendCommandLong(42426, {}); // 42426 = CANCEL_MAG_CAL
    },
    
    async acceptCompassCal() {
        const mask = window.compassSuccessMask || 0;
        if (mask === 0) throw new Error("Tidak ada compass yang SUCCESS. Silakan kalibrasi ulang.");
        const link = this._getActiveLink();
        link.resumeRcOverride();
        return link.sendCommandLong(42425, { p1: mask }); // 42425 = ACCEPT_MAG_CAL
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
};

window.compassSuccessMask = 0;
window.commands = commands;