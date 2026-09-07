// main.js (proses utama Electron)
const { app, BrowserWindow, session, ipcMain } = require('electron');
const path = require('path');
const { SerialPort } = require('serialport');
let activeSerialPort = null; 
const fs = require('fs');
const dgram = require('dgram'); // Wajib ditambahkan untuk membuka socket UDP

app.commandLine.appendSwitch('disable-features', 'WebRtcHideLocalIpsWithMdns');

let mainWindow;
let udpClient = null; // Menyimpan status socket UDP

app.whenReady().then(() => {
    console.log('[DEBUG] Electron:', process.versions.electron, '| Chromium:', process.versions.chrome);
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
            devTools: !app.isPackaged,
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
    mainWindow.loadFile(path.join(__dirname, 'public/index.html'))
        .catch(err => {
            console.error('[Electron] Gagal load index.html:', err);
        });

    mainWindow.webContents.on('did-fail-load', (event, errorCode, errorDescription) => {
        console.error(`[Electron] did-fail-load: ${errorCode} - ${errorDescription}`);
    });
});

ipcMain.handle('serial-list', async () => {
    try {
        const ports = await SerialPort.list();
        return ports.map(p => ({ path: p.path, manufacturer: p.manufacturer, vendorId: p.vendorId, productId: p.productId }));
    } catch (err) {
        console.error('[Serial Backend] Gagal list port:', err);
        return [];
    }
});

ipcMain.handle('serial-connect', async (event, { path: portPath, baudRate }) => {
    return new Promise((resolve) => {
        if (activeSerialPort?.isOpen) {
            activeSerialPort.close();
            activeSerialPort = null;
        }
        try {
            activeSerialPort = new SerialPort({ path: portPath, baudRate, autoOpen: false });
            activeSerialPort.open((err) => {
                if (err) {
                    console.error('[Serial Backend] Gagal buka port:', err.message);
                    return resolve({ ok: false, message: err.message });
                }

                activeSerialPort.on('data', (chunk) => {
                    if (mainWindow) mainWindow.webContents.send('serial-data', Array.from(chunk));
                });
                activeSerialPort.on('close', () => {
                    console.log('[Serial Backend] Port tertutup.');
                    if (mainWindow) mainWindow.webContents.send('serial-closed');
                });
                activeSerialPort.on('error', (err) => {
                    console.error('[Serial Backend] Error:', err.message);
                    if (mainWindow) mainWindow.webContents.send('serial-error', err.message);
                });

                console.log(`[Serial Backend] Terbuka di ${portPath} @ ${baudRate}`);
                resolve({ ok: true });
            });
        } catch (err) {
            resolve({ ok: false, message: err.message });
        }
    });
});

ipcMain.handle('serial-disconnect', async () => {
    if (activeSerialPort?.isOpen) {
        return new Promise((resolve) => {
            activeSerialPort.close(() => { activeSerialPort = null; resolve({ ok: true }); });
        });
    }
    return { ok: true };
});

ipcMain.on('serial-send', (event, bytes) => {
    if (activeSerialPort?.isOpen) {
        activeSerialPort.write(Buffer.from(bytes), (err) => {
            if (err) console.error('[Serial Backend] Gagal kirim:', err);
        });
    }
});
// ==========================================================
// BACKEND UDP HANDLERS (Dikendalikan oleh dgram Node.js)
// ==========================================================

// ini yang kemaren
// 1. Menangani permintaan buka koneksi UDP dari Frontend
// ipcMain.handle('udp-connect', async (event, { ip, port }) => {
//     if (udpClient) {
//         udpClient.close();
//         udpClient = null;
//     }

//     return new Promise((resolve) => {
//         try {
//             udpClient = dgram.createSocket('udp4');

//             // 2. Mendengarkan data masuk dari FC/Drone
//             udpClient.on('message', (msg, rinfo) => {
//                 // AUTO-TARGETING: Begitu drone membalas, kita kunci IP drone aslinya!
//                 // Ini meniru cara kerja Mission Planner
//                 udpClient.targetIp = rinfo.address;
//                 udpClient.targetPort = rinfo.port;

//                 if (mainWindow) {
//                     // Wajib gunakan Array.from agar data aman saat dikirim ke UI
//                     mainWindow.webContents.send('udp-data', Array.from(msg));
//                 }
//             });

//             udpClient.on('error', (err) => {
//                 console.error('[UDP Backend] Error:', err);
//                 if (udpClient) {
//                     udpClient.close();
//                     udpClient = null;
//                 } 
//                 resolve({ ok: false, message: err.message });
//             });

//             udpClient.targetIp = ip;
//             udpClient.targetPort = port;

//             // 3. Bind ke 0.0.0.0
//             udpClient.bind(port, '0.0.0.0', () => {
//                 // WAJIB AKTIF agar bisa mencari drone ke seluruh jaringan (Broadcast)
//                 udpClient.setBroadcast(true); 
//                 console.log(`[UDP Backend] Socket terbuka di 0.0.0.0:${port}`);
//                 resolve({ ok: true });
//             });

//         } catch (err) {
//             console.error('[UDP Backend] Gagal membuat socket UDP:', err);
//             resolve({ ok: false, message: err.message });
//         }
//     });
// });

// ini yang tadi bisa
function pickDefaultLocalPort(remotePort) {
    // Hindari bind ke port remote yang mungkin dipakai app lain
    if (Number(remotePort) === 14550) return 14560;
    return 14550;
}

ipcMain.handle('udp-connect', async (_event, { ip, port, localPort }) => {
    if (udpClient) {
        try { udpClient.close(); } catch {}
        udpClient = null;
    }

    return new Promise((resolve) => {
        try {
            const remoteIp = ip;
            const remotePort = Number(port);
            const bindPort = Number(localPort) || pickDefaultLocalPort(remotePort);

            udpClient = dgram.createSocket('udp4');

            udpClient.on('message', (msg, rinfo) => {
                // lock endpoint ke sumber yang benar
                udpClient.targetIp = rinfo.address;
                udpClient.targetPort = rinfo.port;

                if (mainWindow) {
                    mainWindow.webContents.send('udp-data', Array.from(msg));
                }
            });

            udpClient.on('error', (err) => {
                console.error('[UDP Backend] Error:', err);
                if (udpClient) {
                    try { udpClient.close(); } catch {}
                    udpClient = null;
                }
                resolve({ ok: false, message: err.message });
            });

            udpClient.targetIp = remoteIp;
            udpClient.targetPort = remotePort;

            udpClient.bind(bindPort, '0.0.0.0', () => {
                try { udpClient.setBroadcast(true); } catch {}
                console.log(`[UDP Backend] bind 0.0.0.0:${bindPort} -> remote ${remoteIp}:${remotePort}`);
                resolve({ ok: true, bindPort, targetIp: remoteIp, targetPort: remotePort });
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
                devTools: !app.isPackaged,
                preload: preloadPath
            }
        });

        mainWindow.loadFile(path.join(__dirname, 'public', 'index.html'));
    }
});