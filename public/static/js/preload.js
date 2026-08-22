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
    onClosed: (callback) => ipcRenderer.on('serial-closed', callback),
    onError: (callback) => ipcRenderer.on('serial-error', (_e, msg) => callback(msg)),
});

contextBridge.exposeInMainWorld('udpBridge', {
    connect: (ip, port) => ipcRenderer.invoke('udp-connect', { ip, port }),
    disconnect: () => ipcRenderer.invoke('udp-disconnect'),
    send: (bytes) => ipcRenderer.send('udp-send', bytes),

    onData: (callback) => {
        const listener = (_event, bytes) => callback(bytes);
        ipcRenderer.on('udp-data', listener);
        return () => ipcRenderer.removeListener('udp-data', listener);
    },
});