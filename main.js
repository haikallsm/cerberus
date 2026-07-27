// main.js (proses utama Electron)
const { app, BrowserWindow, session } = require('electron');
const path = require('path');

app.commandLine.appendSwitch('disable-features', 'WebRtcHideLocalIpsWithMdns');

let mainWindow;

app.whenReady().then(() => {
    mainWindow = new BrowserWindow({
        width: 1280,
        height: 800,
        webPreferences: {
            contextIsolation: true,
            nodeIntegration: false,
        }
    });

    // Wajib: Electron butuh handler manual buat serial port picker,
    // karena tidak ada UI bawaan seperti di Chrome
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

app.on('window-all-closed', () => {
    if (process.platform !== 'darwin') {
        app.quit();
    }
});

app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) {
        mainWindow = new BrowserWindow({ width: 1280, height: 800 });
        mainWindow.loadFile(path.join(__dirname, 'public/index.html'));
    }
});