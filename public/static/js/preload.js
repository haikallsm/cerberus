// preload.js — env loop kecil
// Jembatan IPC dari renderer ke Electron main process
const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('serialBridge', {
    list: () => ipcRenderer.invoke('serial-list'),
    connect: (path, baudRate) => ipcRenderer.invoke('serial-connect', { path, baudRate }),
    disconnect: () => ipcRenderer.invoke('serial-disconnect'),
    send: (bytes) => ipcRenderer.send('serial-send', bytes),

    onData: (callback) => {
        const listener = (_event, bytes) => callback(bytes);
        ipcRenderer.on('serial-data', listener);
        return () => ipcRenderer.removeListener('serial-data', listener);
    },
    onClosed: (callback) => {
        const listener = () => callback();
        ipcRenderer.on('serial-closed', listener);
        return () => ipcRenderer.removeListener('serial-closed', listener);
    },
    onError: (callback) => {
        const listener = (_e, msg) => callback(msg);
        ipcRenderer.on('serial-error', listener);
        return () => ipcRenderer.removeListener('serial-error', listener);
    },
});

contextBridge.exposeInMainWorld('udpBridge', {
    // localPort opsional (bind ke port tertentu)
    connect: (ip, port, localPort) => ipcRenderer.invoke('udp-connect', { ip, port, localPort }),
    disconnect: () => ipcRenderer.invoke('udp-disconnect'),
    send: (bytes) => ipcRenderer.send('udp-send', bytes),

    onData: (callback) => {
        const listener = (_event, bytes) => callback(bytes);
        ipcRenderer.on('udp-data', listener);
        return () => ipcRenderer.removeListener('udp-data', listener);
    },
});