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

    async takeoff(alt = 1) {
        return this._getActiveLink().takeoff(alt);
    },

    async reboot() {
        return this._getActiveLink().reboot();
    },

    async setFlightMode(mode) {
        return this._getActiveLink().setMode(mode);
    },

    // before
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

    // after
    // async setSafetySwitch(turnOn) {
    //     const MAV_CMD_DO_SET_SAFETY_SWITCH_STATE = 5300;
    //     const safetyState = turnOn ? 0 : 1;

    //     if (turnOn) {
    //         console.log("[Command] HARDWARE SAFETY");
    //     } else {
    //         console.log("[Command] UNSAFETY");
    //     }

    //     const ack = await this._getActiveLink().sendCommandLong(MAV_CMD_DO_SET_SAFETY_SWITCH_STATE, { 
    //         p1: safetyState, 
    //         p2: 0.0,
    //         p3: 0.0,
    //         p4: 0.0,
    //         p5: 0.0,
    //         p6: 0.0,
    //         p7: 0.0
    //     });

    //     console.log("[Command] Safety ACK:", ack);
    //     if (!ack || !ack.ok) {
    //         console.error("[Command] Safety Switch Failed", ack);
    //         return ack;
    //     }

    //     if(window.gcsState?.safety) {
    //         window.gcsState.safety.off = !turnOn;
    //     }

    //     console.log("[Command] Safety Switch:", window.gcsState?.safety?.off ? "OFF" : "ON");
    //     return ack;
    // },

    async setServo(channel, pwm) {
        return this._getActiveLink().setServo(channel, pwm);
    },

    // before
    // async setHybridMode(state) {
    //     const stateStr = String(state).toLowerCase();
    //     let targetState;
    //     let pwm;

    //     if (['low', 'rover'].includes(stateStr)) {
    //         targetState = 'rover';
    //         pwm = 999;
    //         console.log(
    //             '[COMMAND] HYBRID MODE: ROVER (LOW)'
    //         );

    //     } else if (['mid', 'middle', 'copter_safe'].includes(stateStr)) {
    //         targetState = 'copter_safe';
    //         pwm = 1500;
    //         console.log(
    //             '[COMMAND] HYBRID MODE: COPTER & SAFETY (MID)'
    //         );

    //     } else if (['high', 'copter_unsafe', 'ready'].includes(stateStr)) {
    //         targetState = 'copter_unsafe';
    //         pwm = 1998;
    //         console.log(
    //             '[COMMAND] HYBRID MODE: COPTER & UNSAFETY (HIGH)'
    //         );

    //     } else if (['release', 'off'].includes(stateStr)) {
    //         console.log('[COMMAND] HYBRID MODE: RELEASED');
    //         const link = this._getActiveLink();
    //         if (typeof link.cancelHybridOverride === 'function') {
    //             return link.cancelHybridOverride();
    //         }
    //         return link.sendRcOverride(4, 65535);
    //     } else {
    //         throw new Error(`Invalid hybrid mode state: ${state}`);
    //     }
    //     const link = this._getActiveLink();
    //     if (typeof link.setHybridModeTemporary === 'function') {
    //         return link.setHybridModeTemporary(
    //             targetState,
    //             pwm,
    //             4
    //         );
    //     }
    //     return link.sendRcOverride(4, pwm);
    // },

    // after
    // async setHybridMode(state) {
    //     const stateStr = String(state).toLowerCase().trim();
    //     let targetState;
    //     let pwm;
    //     let label;

    //     if (['low', 'rover'].includes(stateStr)) {
    //         targetState = 'rover';
    //         pwm = 999;
    //         label = 'ROVER (LOW)';
    //     } else if (['mid', 'middle', 'copter_safe'].includes(stateStr)) {
    //         targetState = 'copter_safe';
    //         pwm = 1500;
    //         label = 'COPTER + SAFETY (MID)';
    //     } else if (['high', 'copter_unsafe', 'ready'].includes(stateStr)) {
    //         targetState = 'copter_unsafe';
    //         pwm = 1998;
    //         label = 'COPTER + UNSAFETY (HIGH)';
    //     } else if (['release', 'off'].includes(stateStr)) {
    //         const link = this._getActiveLink();
    //         console.log('[COMMAND] HYBRID MODE: RELEASE OVERRIDE (CH5 -> RC FISIK)');

    //         let res;
    //         if (typeof link.cancelHybridOverride === 'function') {
    //             res = await link.cancelHybridOverride();
    //         } else {
    //             res = await link.sendRcOverride(4, 65535);
    //         }

    //         return {
    //             ok: true,
    //             action: 'release',
    //             mode: 'released',
    //             channel: 5,
    //             message: 'Hybrid override released. RC fisik sekarang jadi sumber CH5.',
    //             result: res
    //         };
    //     } else {
    //         throw new Error(`Invalid hybrid mode state: ${state}`);
    //     }

    //     const link = this._getActiveLink();

    //     console.log(`[COMMAND] HYBRID MODE REQUEST: ${label}`);
    //     console.log('[COMMAND] CH5 akan di-latch oleh GCS sampai operator catch-up lalu takeover.');

    //     let res;
    //     if (typeof link.setHybridModeTemporary === 'function') {
    //         res = await link.setHybridModeTemporary(targetState, pwm, 4);
    //     } else {
    //         // fallback lama (kurang ideal, tapi tetap jalan)
    //         res = await link.sendRcOverride(4, pwm);
    //     }

    //     return {
    //         ok: true,
    //         action: 'set',
    //         mode: targetState,
    //         pwm,
    //         channel: 5,
    //         message:
    //             `Hybrid mode ${targetState} aktif via GCS latch. ` +
    //             `Untuk ambil alih dari remote: samakan switch ke mode aktif dulu, lalu pindah ke mode tujuan.`,
    //         result: res
    //     };
    // },

    // after after
    // GANTI setHybridMode() dengan ini
    // GANTI setHybridMode() dengan ini
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
            // FORCE copter selalu SAFE (MID)
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