import time
import serial
from flask import Flask
from flask_socketio import SocketIO

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='gevent', max_http_buffer_size=10000000)

# Konfigurasi Serial USB ACM
SERIAL_PORT = '/dev/ttyACM0'  # Ganti sesuai port perangkat, misal /dev/ttyACM1 atau /dev/ttyUSB0
BAUD_RATE = 115200            # Sesuaikan baud rate (umumnya 57600 atau 115200)

ser = None
last_packet_time = 0
is_drone_connected = False
TIMEOUT_THRESHOLD = 3.0  # Waktu toleransi sebelum dianggap terputus (detik)

def serial_reader_loop():
    """Thread pembaca data raw Serial dari Flight Controller USB ACM"""
    global ser, last_packet_time, is_drone_connected
    
    while True:
        try:
            # Buka port serial jika belum tersambung
            if ser is None or not ser.is_open:
                print(f"[*] Mencoba menghubungkan ke {SERIAL_PORT} @ {BAUD_RATE} baud...")
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
                print(f"[✅ OPEN] Berhasil membuka port {SERIAL_PORT}")

            # Baca data jika tersedia di buffer
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                if data:
                    last_packet_time = time.time()

                    # Deteksi koneksi aktif
                    if not is_drone_connected:
                        is_drone_connected = True
                        print(f"[✅ CONNECTED] Drone terhubung melalui {SERIAL_PORT}")
                        socketio.emit('drone-status', {
                            'connected': True, 
                            'message': f'Drone Terhubung ({SERIAL_PORT})', 
                            'port': SERIAL_PORT
                        })

                    # Kirim data byte stream ke WebSocket frontend
                    socketio.emit('udp-data', list(data))
            else:
                socketio.sleep(0.005)  # Beri jeda kecil untuk gevent loop

        except serial.SerialException as e:
            if is_drone_connected:
                is_drone_connected = False
                print(f"[⚠️ DISCONNECTED] Koneksi Serial terputus: {e}")
                socketio.emit('drone-status', {'connected': False, 'message': 'Port Serial Terputus'})
            
            if ser and ser.is_open:
                ser.close()
            ser = None
            socketio.sleep(1.0)  # Coba koneksi ulang tiap 1 detik

        except Exception as e:
            print(f"[!] Error Serial read: {e}")
            socketio.sleep(0.01)

def connection_monitor_loop():
    """Thread pemantau heartbeat / timeout data serial"""
    global is_drone_connected
    while True:
        socketio.sleep(1)
        if is_drone_connected and (time.time() - last_packet_time > TIMEOUT_THRESHOLD):
            is_drone_connected = False
            print(f"[⚠️ TIMEOUT] Tidak ada data dari {SERIAL_PORT} > {TIMEOUT_THRESHOLD}s")
            socketio.emit('drone-status', {'connected': False, 'message': 'Koneksi Drone Timeout'})

@socketio.on('connect')
def handle_connect():
    print("[+] Browser GCS terhubung ke WebSocket!")
    socketio.emit('drone-status', {
        'connected': is_drone_connected,
        'message': f'Drone Terhubung ({SERIAL_PORT})' if is_drone_connected else 'Menunggu koneksi Serial...'
    })

@socketio.on('udp-send')
def handle_frontend_send(data):
    """Kirim paket MAVLink dari web frontend ke Flight Controller via USB Serial"""
    global ser
    try:
        if isinstance(data, list):
            byte_payload = bytes(data)
        elif isinstance(data, (bytes, bytearray)):
            byte_payload = data
        else:
            byte_payload = bytearray(data)

        if ser is not None and ser.is_open:
            ser.write(byte_payload)
    except Exception as e:
        print(f"[!] Error forward data ke Serial: {e}")

if __name__ == '__main__':
    # Jalankan reader dan monitor sebagai background task gevent/socketio
    socketio.start_background_task(serial_reader_loop)
    socketio.start_background_task(connection_monitor_loop)

    print("[*] WebSocket Server siap di http://127.0.0.1:5000")
    socketio.run(app, host='0.0.0.0', port=5000)