# Cerberus — HAWKEYE Ground Control Station

Cerberus adalah aplikasi **Ground Control Station (GCS)** berbasis Electron untuk mengoperasikan dan memantau kendaraan udara maupun darat melalui protokol **MAVLink**. Project ini dikembangkan sebagai bagian dari sistem HAWKEYE dan mendukung koneksi melalui radio serial serta UDP.

> Status: project aktif dalam pengembangan. Beberapa fitur backend computer vision dan ROS 2 bersifat opsional serta membutuhkan konfigurasi perangkat keras dan software tambahan.

## Fitur

- Komunikasi MAVLink melalui:
  - Serial/radio telemetry
  - UDP
  - Auto-reconnect saat koneksi terputus
- Flight View dengan:
  - Peta berbasis Leaflet
  - Posisi, altitude, speed, heading, climb rate, GPS, dan jumlah satelit
  - Artificial horizon dan compass
  - Status arm/disarm dan flight mode
  - Message log MAVLink
- Kendali kendaraan:
  - Arm/disarm
  - Takeoff, land, RTL, dan perubahan flight mode
  - Dukungan konfigurasi kendaraan copter dan rover
- Mission Planning:
  - Home, waypoint, loiter, RTL, dan land
  - Pengaturan altitude dan radius waypoint
  - Perhitungan jumlah waypoint dan jarak misi
  - Upload/download mission melalui MAVLink
- Preflight dan setup:
  - Pemeriksaan GPS, compass, RC, dan gyro
  - Kalibrasi accelerometer, compass, dan radio
  - Pengaturan koneksi video
- Konfigurasi autopilot:
  - Parameter dan pencarian parameter
  - PID
  - Flight mode
  - Servo output
  - Radio calibration
- Video dan computer vision opsional:
  - WebRTC/MJPEG video stream
  - Modul deteksi objek/victim detection
  - Backend Python/OpenCV dan opsi integrasi ROS 2

## Arsitektur

```text
┌──────────────────────────────┐
│ Electron Main Process        │
│ main.js                      │
│ - SerialPort                 │
│ - UDP socket                 │
│ - IPC bridge                 │
└──────────────┬───────────────┘
               │ IPC
┌──────────────▼───────────────┐
│ Renderer / HAWKEYE UI        │
│ public/                      │
│ - Flight View                │
│ - Mission Planning           │
│ - Setup & Preflight          │
│ - Configuration              │
│ - MAVLink parser             │
└──────────────┬───────────────┘
               │ MAVLink / video
      ┌────────▼────────┐
      │ Vehicle / FC    │
      │ Camera backend   │
      └─────────────────┘
```

## Persyaratan

### Aplikasi desktop

- Node.js 18 atau lebih baru
- npm
- Sistem operasi yang didukung Electron: Windows, Linux, atau macOS
- Flight controller/vehicle yang menyediakan koneksi MAVLink jika ingin menggunakan telemetry nyata

### Backend vision/video (opsional)

- Python 3.10+ direkomendasikan
- Dependensi pada [`backend/requirements.txt`](backend/requirements.txt)
- Untuk build ROS 2/C++: ROS 2, OpenCV, GStreamer, `libmicrohttpd`, `nlohmann-json`, dan ONNX Runtime

## Instalasi

Clone repository lalu install dependency Node.js:

```bash
git clone https://github.com/haikallsm/cerberus.git
cd cerberus
npm install
```

## Menjalankan aplikasi

Jalankan mode development:

```bash
npm start
```

Saat aplikasi terbuka, pilih salah satu jalur koneksi yang tersedia:

- **Radio/Serial**: pilih port serial dan baud rate yang sesuai.
- **UDP**: masukkan alamat IP dan port remote, serta local port bila diperlukan.

Pastikan hanya satu aplikasi yang menggunakan port serial atau UDP yang sama.

## Build installer

Build default menggunakan Electron Builder:

```bash
npm run build
```

Build Linux:

```bash
npm run build:linux
```

Hasil build tersedia di direktori `dist/` dan dapat berupa installer/portable package sesuai konfigurasi platform pada `package.json`.

## Backend Python

Install dependency backend bila fitur video atau computer vision diperlukan:

```bash
python -m venv .venv

# Linux/macOS
source .venv/bin/activate

# Windows PowerShell
# .venv\Scripts\Activate.ps1

pip install -r backend/requirements.txt
```

Backend menggunakan Flask, WebRTC melalui `aiortc`, dan OpenCV. Konfigurasi alamat IP kamera/Raspberry Pi dilakukan dari halaman Setup aplikasi.

## ROS 2 dan ONNX Runtime (opsional)

`backend/CMakeLists.txt` berisi konfigurasi build node ROS 2 `hawkeyev2` dengan OpenCV, GStreamer, dan ONNX Runtime. Dependensi tersebut tidak diperlukan untuk menjalankan UI Electron dasar.

Sebelum menjalankan build ROS 2, pastikan environment ROS 2 telah di-source dan ONNX Runtime tersedia pada lokasi yang dikonfigurasi, default:

```text
/opt/onnxruntime
```

## Struktur project

```text
.
├── main.js                 # Electron main process, serial, UDP, dan IPC
├── package.json             # Script dan dependency Electron
├── public/                  # UI, halaman GCS, CSS, dan JavaScript
│   ├── flightview.html      # Flight View dan telemetry
│   ├── mission.html         # Mission Planning
│   ├── setup.html           # Preflight dan kalibrasi
│   ├── config.html          # Parameter, PID, servo, dan radio
│   └── static/js/           # MAVLink parser, koneksi, dan UI logic
├── backend/                 # Video, vision, dan integrasi ROS 2
├── assets/                  # Logo dan gambar kendaraan
└── build/                   # Resource installer Electron
```

## Catatan keamanan dan operasional

- Uji semua perintah arm, takeoff, land, dan mission di lingkungan aman sebelum digunakan pada kendaraan nyata.
- Periksa kembali system ID, component ID, mode penerbangan, geofence, dan failsafe autopilot.
- Jangan menjalankan perintah penerbangan tanpa koneksi telemetry dan kondisi GPS yang tervalidasi.
- Jangan membagikan konfigurasi jaringan, kredensial, atau data telemetry sensitif ke repository publik.

## Lisensi

Repository ini belum mencantumkan file lisensi. Tambahkan file `LICENSE` jika project akan didistribusikan secara publik dengan ketentuan lisensi tertentu.

## Kontribusi

Perubahan dapat diajukan melalui issue atau pull request. Untuk perubahan pada protokol MAVLink, koneksi kendaraan, atau fitur keselamatan penerbangan, sertakan langkah pengujian dan kondisi perangkat keras yang digunakan.
