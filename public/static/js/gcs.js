class GCSTelemetry {
    constructor() {
        this.data = { 
            lat: 0, lon: 0, alt: 0, roll: 0, pitch: 0, yaw: 0, 
            speed: 0, heading: 0, climb: 0, battery_voltage: 0, 
            battery_current: 0, battery_remaining: 0, system_status: 'disconnected', 
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
    
    // Siapkan wadah kosong, pastikan selalu ada penanda sumber data
    const out = { source: 'radio' };

    if (raw.vehicle_type !== undefined) out.vehicle_type = raw.vehicle_type;
    if (raw.armed !== undefined) {
        out.is_armed = raw.armed;
        out.arm_mode = raw.armed ? 'ARMED' : 'DISARMED';
    }
    if (raw.mode !== undefined) out.flight_mode = raw.mode;
    if (raw.connected !== undefined) out.status = raw.connected ? 'connected' : 'disconnected';

    if (raw.attitude) {
        out.roll = raw.attitude.roll * RAD2DEG;
        out.pitch = raw.attitude.pitch * RAD2DEG;
        out.yaw = raw.attitude.yaw * RAD2DEG;
    }

    if (raw.latitude !== undefined) out.lat = raw.latitude;
    if (raw.longitude !== undefined) out.lon = raw.longitude;
    if (raw.altitude !== undefined) out.alt = raw.altitude;
    if (raw.heading !== undefined) out.heading = raw.heading;

    // 4. Data Kecepatan HUD (Berasal dari VFR_HUD)
    if (raw.speed !== undefined) out.speed = raw.speed;
    if (raw.climb !== undefined) out.climb = raw.climb;
    if (raw.throttle !== undefined) out.throttle = raw.throttle;

    // 5. Data GPS (Berasal dari GPS_RAW_INT)
    if (raw.satellites_visible !== undefined) out.satellites_visible = raw.satellites_visible;
    if (raw.gps_fix_type !== undefined) out.gps_fix_type = raw.gps_fix_type;

    // 6. Data Baterai (Berasal dari SYS_STATUS)
    if (raw.battery_voltage !== undefined) out.battery_voltage = raw.battery_voltage;
    if (raw.battery_current !== undefined) out.battery_current = raw.battery_current;
    if (raw.battery_remaining !== undefined) out.battery_remaining = raw.battery_remaining;

    // 7. Data Sensor Mentah untuk Preflight (Berasal dari RAW_IMU)
    if (raw.accel_x !== undefined) {
        out.accel_x = raw.accel_x; out.accel_y = raw.accel_y; out.accel_z = raw.accel_z;
        out.gyro_x = raw.gyro_x; out.gyro_y = raw.gyro_y; out.gyro_z = raw.gyro_z;
        out.mag_field = raw.mag_field;
    }
    // 8. RF
    if (raw.rangefinder_distance !== undefined) out.rangefinder_distance = raw.rangefinder_distance;
    if (raw.rangefinder_healthy !== undefined) out.rangefinder_healthy = raw.rangefinder_healthy;
    if (raw.rangefinder_type !== undefined) out.rangefinder_type = raw.rangefinder_type;
    
    if (raw.rc5_in !== undefined) out.rc5_in = raw.rc5_in;
    
    return out;
}
window.adaptRadioTelemetry = adaptRadioTelemetry;
// ===== API Connection Manager =====
const DEFAULT_BACKEND_URL = 'http://127.0.0.1:5000'

class GCSApiClient {
    constructor() {
        this.isConnected = false;
        this.telemetry = new GCSTelemetry();
        this.ws = null;
        this.baseUrl = DEFAULT_BACKEND_URL;
        this.onConnect = null;
        this.onDisconnect = null;
        this.onTelemetry = null;
        this.onMavlog = null;
        this.onStatusChange = null;
        this.remoteBackendMode = false;
        this.lastLogSeq = 0;
        this.isFetchingLogs = false;
        this.logHistory = [];
        this.maxLogHistory = 200;
        this.localSerialPort = null;
        this.serialReader = null;
        this.serialWriter = null;
    }

    startWebSocket() {
        if (this.ws) this.ws.disconnect();

        this.ws = io(this.baseUrl);

        this.ws.on('connect', () => {
            console.log('Socket.io Telemetry Terhubung');
        });

        this.ws.on('telemetry', (data) => {
            this.telemetry.updateFromApi(data);
            this.messagesReceived++;
            if (this.onTelemetry) this.onTelemetry(data);
        });

        this.ws.on('mavlog', (logItem) => {
            this.handleIncomingLog(logItem);
        });

        // 3. Pengecekan Gap saat baru konek
        this.ws.on('mavlog_meta', (meta) => {
            if (meta.latest_seq > this.lastLogSeq) {
                this.fetchRecentLogs();
            }
        });

        this.ws.on('disconnect', () => {
            console.log('Socket.io Terputus');
        });
    }

    handleIncomingLog(logItem) {
        if (this.lastLogSeq > 0 && logItem.seq > this.lastLogSeq + 1) {
            this.fetchRecentLogs();
            return; 
        }
        
        if (logItem.seq <= this.lastLogSeq) return;

        this.lastLogSeq = logItem.seq;
        this.logHistory.push(logItem);
        if(this.logHistory.length > this.maxLogHistory) this.logHistory.shift();
        if(this.onMavlog) this.onMavlog(logItem);
    }

    // Gap Recovery: Mengambil log yang tertinggal via REST API
    async fetchRecentLogs() {
        if (this.isFetchingLogs) return;
        this.isFetchingLogs = true;

        try {
            const response = await fetch(`${this.baseUrl}/logs/recent?after=${this.lastLogSeq}&limit=100`);
            const data = await response.json();
            
            if (data.status === 'success' && data.items) {
                data.items.forEach(item => {
                    if (item.seq > this.lastLogSeq) {
                        this.lastLogSeq = item.seq;
                        this.logHistory.push(item);
                        if (this.logHistory.length > this.maxLogHistory) this.logHistory.shift();
                        if (this.onMavlog) this.onMavlog(item);
                    }
                });
            }
        } catch (err) {
            console.error("Gagal menarik log yang tertinggal:", err);
        } finally {
            this.isFetchingLogs = false;
        }
    }

    stopWebSocket() {
        if (this.ws) {
            this.ws.disconnect();
            this.ws = null;
        }
    }

    updateUI(connected, failed = false) {
        const btn = document.getElementById('connectBtn'); // Sesuaikan dengan id di navbar baru
        const statusText = document.getElementById('statusText');
        
        if (btn) {
            if (connected) {
                btn.innerText = 'Disconnect';
            } else {
                btn.innerText = 'Connect';
            }
        }
        
        if (statusText) {
            if (connected) {
                statusText.innerText = 'Connected';
                statusText.style.color = '#4caf50';
            } else {
                statusText.innerText = failed ? 'Failed' : 'Disconnected';
                statusText.style.color = failed ? '#f44336' : '#888';
            }
        }
        
        if (this.onStatusChange) this.onStatusChange(connected);
    }

    restoreNavbarStatus() {
        try {
            const stored = sessionStorage.getItem('hawkeye_navbar_status');
            if (stored) {
                const status = JSON.parse(stored);
                this.updateUI(status.connected, status.failed);
            }
        } catch (e) {
            // Silently ignore
        }
    }
}

window.gcsConnection = new GCSApiClient();

document.addEventListener('DOMContentLoaded', async () => {
    console.log('Initializing HAWKEYE GCS Master...');
    
    // Persiapan awal UI
    window.gcsConnection.loadPorts();
    if (typeof window.gcsConnection.switchMode === 'function') {
        window.gcsConnection.switchMode();
    }
    window.gcsConnection.restoreNavbarStatus();
    
    // PERBAIKAN 1: Routing data telemetri ke dalam Iframe
    window.gcsConnection.onTelemetry = (data) => {
        const frame = document.getElementById('main-frame');
        if (frame && frame.contentWindow && typeof frame.contentWindow.updateTelemetryUI === 'function') {
            frame.contentWindow.updateTelemetryUI(data);
        }
    };

    window.gcsConnection.onMavlog = (logItem) => {
        const frame = document.getElementById('main-frame');
        if (frame && frame.contentWindow && typeof frame.contentWindow.updateMavlogUI === 'function') {
            frame.contentWindow.updateMavlogUI(logItem);
        }
    };
    try {
        const res = await fetch(window.gcsConnection.baseUrl + '/status');
        const data = await res.json();
        
        if (data.connected) {
            window.gcsConnection.isConnected = true;
            window.gcsConnection.updateUI(true);
            
            // PERBAIKAN 2: Pastikan WebSocket dipanggil saat auto-reconnect
            window.gcsConnection.startWebSocket(); 
            
            console.log("FC terdeteksi sudah terkoneksi sebelumnya. Melanjutkan sesi...");
        } else {
            window.gcsConnection.updateUI(false);
            console.log("Menunggu instruksi koneksi manual dari pengguna...");
        }
    } catch (err) {
        console.error("Gagal memeriksa status awal dari backend:", err);
    }
});