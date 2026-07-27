# Asumsi fungsi ini dipanggil setiap kali GCS menerima pesan STATUSTEXT dari telemetri
def on_statustext_received(mavlink_message):
    teks = mavlink_message.text
    
    # 1. CEK PREFIX UNIK
    if teks.startswith("[SAR]"):
        # Ini adalah laporan korban, JANGAN tampilkan di log biasa!
        parsing_data_korban(teks)
    else:
        # Ini pesan FCU biasa (contoh: "PreArm: GPS Glitch")
        tampilkan_di_log_fcu(teks)

def parsing_data_korban(teks):
    # Hilangkan spasi berlebih dan pisahkan berdasarkan ';'
    data = teks.strip().split(";")
    
    # Validasi panjang data
    if len(data) == 5:
        header = data[0]        # "[SAR]"
        status = data[1]        # "H" atau "D"
        jumlah = int(data[2])   # "2"
        lat = float(data[3])    # "-7.275614"
        lon = float(data[4])    # "112.793739"
        
        # Penentuan Warna / Ikon Marker di Peta GCS
        warna_titik = "Merah" if status == "H" else "Hitam"
        info_teks = "Korban Hidup" if status == "H" else "Korban Tewas"
        
        # Eksekusi fungsi GCS Anda untuk menggambar titik di Peta (Maps)
        tambah_marker_ke_peta(lat, lon, warna_titik, f"{info_teks}: {jumlah} Orang")
