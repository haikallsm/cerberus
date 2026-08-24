; installer.nsh
; Dijalankan otomatis oleh electron-builder (target NSIS) saat build.
; customInstall  -> jalan setelah file di-copy ke $INSTDIR, saat instalasi
; customUnInstall -> jalan saat uninstall, sebelum file dihapus
;
; CATATAN: ini HANYA berlaku untuk target "nsis" (installer), bukan "portable".
; Build "portable" tidak punya tahap install sama sekali, jadi rule firewall
; tidak akan ter-provision otomatis untuk build itu -- user portable tetap akan
; kena prompt Windows Firewall manual pertama kali connect UDP.

!macro customInstall
  DetailPrint "Menambahkan Windows Firewall rule untuk Hawkeye GCS (UDP telemetry)..."

  ; Inbound UDP -- wajib, supaya balasan telemetry MAVLink dari FC/drone
  ; (port acak sesuai konfigurasi radio companion / SITL) tidak di-drop firewall
  nsExec::ExecToLog 'netsh advfirewall firewall add rule name="Hawkeye GCS (UDP In)" dir=in action=allow program="$INSTDIR\Hawkeye.exe" protocol=UDP enable=yes profile=any'

  ; Outbound UDP -- biasanya sudah default-allow di Windows, tapi didaftarkan
  ; eksplisit supaya konsisten dan gampang diaudit/dicabut satu rule sekaligus
  nsExec::ExecToLog 'netsh advfirewall firewall add rule name="Hawkeye GCS (UDP Out)" dir=out action=allow program="$INSTDIR\Hawkeye.exe" protocol=UDP enable=yes profile=any'
!macroend

!macro customUnInstall
  DetailPrint "Menghapus Windows Firewall rule Hawkeye GCS..."
  nsExec::ExecToLog 'netsh advfirewall firewall delete rule name="Hawkeye GCS (UDP In)"'
  nsExec::ExecToLog 'netsh advfirewall firewall delete rule name="Hawkeye GCS (UDP Out)"'
!macroend
