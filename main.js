// main.js (proses utama Electron)
const { app, BrowserWindow, session, ipcMain } = require('electron');
const path = require('path');

const fs = require('fs');
const dgram = require('dgram'); // Wajib ditambahkan untuk membuka socket UDP

app.commandLine.appendSwitch('disable-features', 'WebRtcHideLocalIpsWithMdns');

let mainWindow;
let udpClient = null; // Menyimpan status socket UDP

app.whenReady().then(() => {
    // === DIAGNOSTIC: pastikan preload.js benar-benar ada di path yang dituju ===
   const preloadPath = path.join(__dirname, 'public', 'static', 'js', 'preload.js');
    console.log('[DEBUG] __dirname     =', __dirname);
    console.log('[DEBUG] preloadPath   =', preloadPath);
    console.log('[DEBUG] file exists?  =', fs.existsSync(preloadPath));
    // ============================================================================

    mainWindow = new BrowserWindow({
        width: 1280,
        height: 800,
        icon: path.join(__dirname, "assets", "main-logo.png"),
        webPreferences: {
            contextIsolation: true,
            nodeIntegration: false,
            // Wajib ditambahkan agar file preload dieksekusi sebelum HTML
            preload: preloadPath
        }
    });

    // === DIAGNOSTIC: tangkap error preload yang biasanya "ditelan diam-diam" ===
    mainWindow.webContents.on('preload-error', (event, preloadPath, error) => {
        console.error('[Electron] PRELOAD GAGAL DIMUAT!');
        console.error('[Electron] Path:', preloadPath);
        console.error('[Electron] Error:', error);
    });
    // ============================================================================

    // Wajib: Electron butuh handler manual buat serial port picker
    mainWindow.webContents.session.on('select-serial-port', (event, portList, webContents, callback) => {
        event.preventDefault();
        if (portList.length === 0) {
            callback('');
        } else {
            callback(portList[0].portId);
        }
    });

    mainWindow.webContents.session.setPermissionCheckHandler((webContents, permission) => {
        return permission === 'serial';
    });

    mainWindow.webContents.session.setDevicePermissionHandler((details) => {
        return details.deviceType === 'serial';
    });

    mainWindow.loadFile(path.join(__dirname, 'public/index.html'))
        .catch(err => {
            console.error('[Electron] Gagal load index.html:', err);
        });

    mainWindow.webContents.on('did-fail-load', (event, errorCode, errorDescription) => {
        console.error(`[Electron] did-fail-load: ${errorCode} - ${errorDescription}`);
    });
});

// ==========================================================
// BACKEND UDP HANDLERS (Dikendalikan oleh dgram Node.js)
// ==========================================================

// 1. Menangani permintaan buka koneksi UDP dari Frontend
// 1. Menangani permintaan buka koneksi UDP dari Frontend
ipcMain.handle('udp-connect', async (event, { ip, port }) => {
    if (udpClient) {
        udpClient.close();
        udpClient = null;
    }

    return new Promise((resolve) => {
        try {
            udpClient = dgram.createSocket('udp4');

            // 2. Mendengarkan data masuk dari FC/Drone
            udpClient.on('message', (msg, rinfo) => {
                // AUTO-TARGETING: Begitu drone membalas, kita kunci IP drone aslinya!
                // Ini meniru cara kerja Mission Planner
                udpClient.targetIp = rinfo.address;
                udpClient.targetPort = rinfo.port;

                if (mainWindow) {
                    // Wajib gunakan Array.from agar data aman saat dikirim ke UI
                    mainWindow.webContents.send('udp-data', Array.from(msg));
                }
            });

            udpClient.on('error', (err) => {
                console.error('[UDP Backend] Error:', err);
                if (udpClient) udpClient.close();
                udpClient = null;
            });

            udpClient.targetIp = ip;
            udpClient.targetPort = port;

            // 3. Bind ke 0.0.0.0
            udpClient.bind(port, '0.0.0.0', () => {
                // WAJIB AKTIF agar bisa mencari drone ke seluruh jaringan (Broadcast)
                udpClient.setBroadcast(true); 
                console.log(`[UDP Backend] Socket terbuka di 0.0.0.0:${port}`);
                resolve({ ok: true });
            });

        } catch (err) {
            console.error('[UDP Backend] Gagal membuat socket UDP:', err);
            resolve({ ok: false, message: err.message });
        }
    });
});

// 3. Menangani permintaan putus koneksi UDP dari Frontend
ipcMain.handle('udp-disconnect', async () => {
    if (udpClient) {
        udpClient.close();
        udpClient = null;
        console.log('[UDP Backend] Socket UDP ditutup');
    }
    return { ok: true };
});

// 4. Menangani pengiriman perintah (byte) dari Frontend ke Drone
ipcMain.on('udp-send', (event, bytes) => {
    if (udpClient && udpClient.targetIp && udpClient.targetPort) {
        const buf = Buffer.from(bytes);
        udpClient.send(buf, udpClient.targetPort, udpClient.targetIp, (err) => {
            if (err) console.error('[UDP Backend] Gagal mengirim paket:', err);
        });
    }
});

// ==========================================================

app.on('window-all-closed', () => {
    if (process.platform !== 'darwin') {
        app.quit();
    }
});
app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) {

        const preloadPath = path.join(
            __dirname,
            'public',
            'static',
            'js',
            'preload.js'
        );

        mainWindow = new BrowserWindow({
            width: 1280,
            height: 800,
            icon: path.join(__dirname, 'assets', 'main-logo.png'),

            webPreferences: {
                contextIsolation: true,
                nodeIntegration: false,
                preload: preloadPath
            }
        });

        mainWindow.loadFile(path.join(__dirname, 'public', 'index.html'));
    }
});