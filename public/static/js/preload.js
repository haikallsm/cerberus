// preload.js
// Jembatan aman renderer <-> main process. Renderer TIDAK dikasih akses
// langsung ke ipcRenderer atau Node API -- cuma fungsi spesifik ini saja
// yang diekspos, sesuai prinsip contextIsolation.
//
// Kalau kamu SUDAH punya preload.js sendiri (misal untuk serial port picker),
// gabungkan blok "udpBridge" ini ke situ -- jangan bikin 2 file preload
// terpisah untuk window yang sama.

const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('udpBridge', {
    connect: (ip, port) => ipcRenderer.invoke('udp-connect', { ip, port }),
    disconnect: () => ipcRenderer.invoke('udp-disconnect'),
    send: (bytes) => ipcRenderer.send('udp-send', bytes),

    // onData mengembalikan fungsi "unsubscribe" -- WAJIB dipanggil saat disconnect()
    // supaya listener lama tidak menumpuk kalau connect() dipanggil berkali-kali.
    onData: (callback) => {
        const listener = (_event, bytes) => callback(bytes);
        ipcRenderer.on('udp-data', listener);
        return () => ipcRenderer.removeListener('udp-data', listener);
    },
});