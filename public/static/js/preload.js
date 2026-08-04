
const { contextBridge, ipcRenderer } = require('electron');

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