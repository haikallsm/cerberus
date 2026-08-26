class GCSTelemetry {
    constructor() {
        this.data = {
            lat: 0, lon: 0, alt: 0, roll: 0, pitch: 0, yaw: 0,
            speed: 0, heading: 0, climb: 0, system_status: 'disconnected',
            satellites_visible: 0, gps_fix_type: 0
        };
        this.onUpdate = null;
    }

    updateFromApi(apiData) {
        if (!apiData) return;
        this.data = { ...this.data, ...apiData };
        if (this.onUpdate) this.onUpdate(this.data);
    }
}

function adaptRadioTelemetry(raw) {
    const RAD2DEG = 180 / Math.PI;

    const out = { source: raw.source || 'radio' };

    if (raw.vehicle_type !== undefined) out.vehicle_type = raw.vehicle_type;
    if (raw.armed !== undefined) {
        out.is_armed = raw.armed;
        out.arm_mode = raw.armed ? 'ARMED' : 'DISARMED';
    }
    if (raw.mode !== undefined) out.flight_mode = raw.mode;
    if (raw.connected !== undefined) {
        out.status = raw.connected ? 'connected' : 'disconnected';
        out.system_status = out.status;
    }

    if (raw.attitude) {
        out.roll = raw.attitude.roll * RAD2DEG;
        out.pitch = raw.attitude.pitch * RAD2DEG;
        out.yaw = raw.attitude.yaw * RAD2DEG;
    }

    if (raw.latitude !== undefined) out.lat = raw.latitude;
    if (raw.longitude !== undefined) out.lon = raw.longitude;
    if (raw.altitude !== undefined) out.alt = raw.altitude;
    if (raw.heading !== undefined) out.heading = raw.heading;

    // HOME_POSITION (msg 242) -- dikirim FC saat home berubah (EKF origin set,
    // arm pertama, dsb) atau saat diminta via MAV_CMD_GET_HOME_POSITION.
    if (raw.home_lat !== undefined) out.home_lat = raw.home_lat;
    if (raw.home_lon !== undefined) out.home_lon = raw.home_lon;
    if (raw.home_alt !== undefined) out.home_alt = raw.home_alt;

    // Data Kecepatan HUD (Berasal dari VFR_HUD)
    if (raw.speed !== undefined) out.speed = raw.speed;
    if (raw.climb !== undefined) out.climb = raw.climb;
    if (raw.throttle !== undefined) out.throttle = raw.throttle;

    // Data GPS (Berasal dari GPS_RAW_INT)
    if (raw.satellites_visible !== undefined) out.satellites_visible = raw.satellites_visible;
    if (raw.gps_fix_type !== undefined) out.gps_fix_type = raw.gps_fix_type;

    // Data Baterai (Berasal dari SYS_STATUS)
    if (raw.battery_voltage !== undefined) out.battery_voltage = raw.battery_voltage;
    if (raw.battery_current !== undefined) out.battery_current = raw.battery_current;
    if (raw.battery_remaining !== undefined) out.battery_remaining = raw.battery_remaining;

    // Data Sensor Mentah untuk Preflight (Berasal dari RAW_IMU)
    if (raw.accel_x !== undefined) {
        out.accel_x = raw.accel_x; out.accel_y = raw.accel_y; out.accel_z = raw.accel_z;
        out.gyro_x = raw.gyro_x; out.gyro_y = raw.gyro_y; out.gyro_z = raw.gyro_z;
        out.mag_field = raw.mag_field;
    }

    // Rangefinder
    if (raw.rangefinder_distance !== undefined) out.rangefinder_distance = raw.rangefinder_distance;
    if (raw.rangefinder_healthy !== undefined) out.rangefinder_healthy = raw.rangefinder_healthy;
    if (raw.rangefinder_type !== undefined) out.rangefinder_type = raw.rangefinder_type;

    if (raw.rc5_in !== undefined) out.rc5_in = raw.rc5_in;

    if (raw.mag_cal_progress !== undefined) out.mag_cal_progress = raw.mag_cal_progress;
    if (raw.mag_cal_report !== undefined) out.mag_cal_report = raw.mag_cal_report;

    return out;
}
window.adaptRadioTelemetry = adaptRadioTelemetry;
window.gcsConnection = { telemetry: new GCSTelemetry(), logHistory: [] };