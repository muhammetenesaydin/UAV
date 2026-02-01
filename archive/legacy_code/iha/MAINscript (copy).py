import socket
import subprocess
import threading
import time
import json
from datetime import datetime

# MAVLink için kütüphane ekle
from pymavlink import mavutil

# UDP dinleme ayarları
UDP_IP = "0.0.0.0"
UDP_PORT = 14755

# Telemetri gönderme aralığı (saniye)
TELEMETRY_INTERVAL = 0.2

# Çalıştırılacak script yolu
MISSION_SCRIPT = "/home/ysf/Downloads/iha2/iha_mission.py"

# MAVLink bağlantı ayarları
MAVLINK_CONNECTION = "udpin:localhost:14550"  # ArduPilot bağlantısı için, gerçek değerle değiştirilmeli

# UDP soketi başlat
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# MAVLink bağlantısı
mav_connection = None

# Mission script process
mission_process = None

# En son alınan telemetri verilerini saklayacak değişken
last_telemetry_data = {
    "altitude": 0.0,
    "speed": 0.0,
    "battery": 100,
    "latitude": 0.0,
    "longitude": 0.0,
    "heading": 0.0,
    "flight_mode": "UNKNOWN",
    "armed": False,
    "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
}

# ArduPilot mod numaraları - isim eşleştirmeleri
# ArduPilot'un versiyonuna göre değişebilir
# Bu değerler plane (sabit kanat) için
ARDUPILOT_PLANE_MODE_MAP = {
    0: "MANUAL",
    1: "CIRCLE",
    2: "STABILIZE",
    3: "TRAINING",
    4: "ACRO",
    5: "FBWA",
    6: "FBWB",
    7: "CRUISE",
    8: "AUTOTUNE",
    10: "AUTO",
    11: "RTL",
    12: "LOITER",
    13: "TAKEOFF",
    14: "AVOID_ADSB",
    15: "GUIDED",
    16: "INITIALIZING",
    17: "QSTABILIZE",
    18: "QHOVER",
    19: "QLOITER",
    20: "QLAND",
    21: "QRTL",
    22: "QAUTOTUNE",
    23: "QACRO",
    24: "THERMAL",
    25: "LOITER_ALT_UNKOWN"
}



# Arayüz mod adları -> ArduPilot mod numaraları (Plane/Sabit Kanat)
PLANE_UI_TO_AP_MODE_MAP = {
    "MANUAL": 0,
    "STABILIZE": 2,
    "GUIDED": 15,
    "AUTO": 10,
    "RTL": 11,
    "LAND": 9,  # Genellikle copter için - plane'de olmayabilir
    "LOITER": 12,
    "CIRCLE": 1,
    "FBWA": 5,
    "FBWB": 6,
    "CRUISE": 7,
    "TRAINING": 3,
    "ACRO": 4
}

# Arayüz mod adları -> ArduPilot mod numaraları (Copter/Multicopter)
COPTER_UI_TO_AP_MODE_MAP = {
    "MANUAL": 0,  # Aslında STABILIZE
    "STABILIZE": 0,
    "GUIDED": 4,
    "AUTO": 3,
    "RTL": 6,
    "LAND": 9,
    "LOITER": 5,
    "ALT_HOLD": 2,
    "ACRO": 1,
    "DRIFT": 11,
    "SPORT": 13,
    "FLIP": 14,
    "AUTOTUNE": 15,
    "POSHOLD": 16
}

# ArduPilot mod eşleşme seçimi (Plane veya Copter)
# İHA tipinize göre değiştirin
is_plane = True  # Sabit kanat için True, multicopter için False

print(f"[INFO] Dinleyici başlatıldı: {UDP_IP}:{UDP_PORT}")
print("[INFO] Komut bekleniyor...")

def init_mavlink():
    """MAVLink bağlantısını başlatır"""
    global mav_connection, is_plane
    try:
        print(f"[INFO] MAVLink bağlantısı başlatılıyor: {MAVLINK_CONNECTION}")
        mav_connection = mavutil.mavlink_connection(MAVLINK_CONNECTION)
        print(f"[INFO] MAVLink bağlantısı için heartbeat bekleniyor...")
        mav_connection.wait_heartbeat()
        
        # ArduPilot tipini tespit et
        # type=mavutil.mavlink.MAV_TYPE_QUADROTOR veya benzeri ise multicopter
        # type=mavutil.mavlink.MAV_TYPE_FIXED_WING ise sabit kanat
        vehicle_type = mav_connection.field('HEARTBEAT', 'type', 0)
        
        if vehicle_type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            is_plane = True
            print("[INFO] Araç tipi: Sabit Kanat (Plane)")
        elif vehicle_type in [mavutil.mavlink.MAV_TYPE_QUADROTOR, 
                              mavutil.mavlink.MAV_TYPE_HELICOPTER,
                              mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                              mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                              mavutil.mavlink.MAV_TYPE_TRICOPTER]:
            is_plane = False
            print("[INFO] Araç tipi: Multicopter (Copter)")
        else:
            print(f"[WARN] Bilinmeyen araç tipi: {vehicle_type}, varsayılan olarak Plane kullanılıyor")
        
        print(f"[INFO] MAVLink bağlantısı kuruldu. Sistem ID: {mav_connection.target_system}, Bileşen ID: {mav_connection.target_component}")
        
        # Mevcut modu al
        msg = mav_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            custom_mode = msg.custom_mode
            mode_name = get_mode_name_from_number(custom_mode)
            print(f"[INFO] ArduPilot mevcut mod: {custom_mode} ({mode_name})")
        
        return True
    except Exception as e:
        print(f"[ERROR] MAVLink bağlantı hatası: {e}")
        return False

def get_mode_name_from_number(mode_number):
    """ArduPilot mod numarasını okunabilir isme çevirir"""
    global is_plane
    if is_plane:
        return ARDUPILOT_PLANE_MODE_MAP.get(mode_number, f"UNKNOWN({mode_number})")
    

def get_mode_number_from_name(mode_name):
    """Mod adını ArduPilot mod numarasına çevirir"""
    global is_plane
    mode_name = mode_name.upper()
    
    if is_plane:
        return PLANE_UI_TO_AP_MODE_MAP.get(mode_name, None)
    else:
        return COPTER_UI_TO_AP_MODE_MAP.get(mode_name, None)

def run_mission_script():
    """Görev scriptini çalıştırır"""
    global mission_process
    print(f"[ACTION] {MISSION_SCRIPT} başlatılıyor...")
    mission_process = subprocess.Popen(
        ["/bin/python3", MISSION_SCRIPT],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
    )

    # Çıktıları anlık olarak yazdır
    for line in mission_process.stdout:
        print(f"[SCRIPT] {line}", end='')

def collect_telemetry():
    """MAVLink'ten telemetri verilerini toplar"""
    global mav_connection, last_telemetry_data
    
    # Sürekli olarak MAVLink mesajlarını al ve verileri güncelle
    while True:
        try:
            if mav_connection:
                # HEARTBEAT mesajı
                msg = mav_connection.recv_match(type='HEARTBEAT', blocking=False)
                if msg and msg.get_type() == 'HEARTBEAT':
                    # Mod numarasını mod adına çevir
                    custom_mode = msg.custom_mode
                    mode_name = get_mode_name_from_number(custom_mode)
                    
                    last_telemetry_data["flight_mode"] = mode_name
                    last_telemetry_data["armed"] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    print(f"[TELEMETRY] Mod: {last_telemetry_data['flight_mode']} (ID: {custom_mode}), Armed: {last_telemetry_data['armed']}")
                
                # GLOBAL_POSITION_INT mesajı
                msg = mav_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                if msg and msg.get_type() == 'GLOBAL_POSITION_INT':
                    last_telemetry_data["altitude"] = msg.relative_alt / 1000.0  # mm'den m'ye çevir
                    last_telemetry_data["latitude"] = msg.lat / 1e7  # Düzeltme faktörü
                    last_telemetry_data["longitude"] = msg.lon / 1e7  # Düzeltme faktörü
                    if msg.hdg != 65535:  # Geçersiz değer kontrolü
                        last_telemetry_data["heading"] = msg.hdg / 100.0
                
                # VFR_HUD mesajı
                msg = mav_connection.recv_match(type='VFR_HUD', blocking=False)
                if msg and msg.get_type() == 'VFR_HUD':
                    last_telemetry_data["speed"] = msg.groundspeed
                    if last_telemetry_data["heading"] == 0.0:
                        last_telemetry_data["heading"] = msg.heading
                
                # SYS_STATUS mesajı
                msg = mav_connection.recv_match(type='SYS_STATUS', blocking=False)
                if msg and msg.get_type() == 'SYS_STATUS':
                    last_telemetry_data["battery"] = msg.battery_remaining if msg.battery_remaining > 0 else 0
                
                # Zaman bilgisini güncelle
                last_telemetry_data["timestamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                
            time.sleep(0.1)  # 100ms aralıklarla kontrol et (çok sık)
        except Exception as e:
            print(f"[ERROR] Telemetri toplama hatası: {e}")
            time.sleep(1)  # Hata durumunda biraz bekle

def send_telemetry(client_addr):
    """Telemetri verilerini gönderir"""
    global last_telemetry_data
    
    while True:
        try:
            # Telemetri verisini hazırla
            telemetry_data = {
                "type": "TELEMETRY",
                "data": last_telemetry_data.copy()  # Mevcut telemetri verilerinin kopyasını kullan
            }
            
            # JSON formatında veriyi gönder
            telemetry_json = json.dumps(telemetry_data)
            sock.sendto(telemetry_json.encode(), client_addr)
            
            # Debug için
            print(f"[TELEMETRY] Veri gönderildi: Alt:{last_telemetry_data['altitude']:.1f}m, "
                  f"Hız:{last_telemetry_data['speed']:.1f}m/s, Mod:{last_telemetry_data['flight_mode']}, "
                  f"Armed:{last_telemetry_data['armed']}")
            
            time.sleep(TELEMETRY_INTERVAL)  # Belirli aralıklarla gönder
        except Exception as e:
            print(f"[ERROR] Telemetri gönderme hatası: {e}")
            time.sleep(1)  # Hata durumunda biraz bekle

def handle_command(command, addr):
    """Gelen komutları işler"""
    global mission_process, mav_connection
    
    # Komut tipini ayır (örn: MODE:GUIDED -> command_type=MODE, params=GUIDED)
    parts = command.split(":", 1)
    command_type = parts[0]
    params = parts[1] if len(parts) > 1 else ""
    
    print(f"[RECV] {addr} → {command_type} {params}")
    
    # Komut işleme
    if command_type == "START":
        if mission_process is None or mission_process.poll() is not None:
            # Misyon scripti çalışmıyorsa başlat
            thread = threading.Thread(target=run_mission_script)
            thread.start()
            return "OK:Görev başlatıldı"
        else:
            return "ERROR:Görev zaten çalışıyor"
    
    elif command_type == "ABORT":
        if mission_process and mission_process.poll() is None:
            mission_process.terminate()
            return "OK:Görev sonlandırıldı"
        return "WARN:Çalışan görev yok"
    
    elif command_type == "ARM":
        if mav_connection:
            try:
                # Arm komutu gönder
                mav_connection.mav.command_long_send(
                    mav_connection.target_system,
                    mav_connection.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 0, 0, 0, 0, 0, 0)
                
                # Komutun alındığını doğrula
                ack = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
                if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print("[INFO] Arm komutu başarıyla kabul edildi")
                        return "OK:ARM komutu kabul edildi"
                    else:
                        print(f"[WARN] Arm komutu reddedildi. Sonuç kodu: {ack.result}")
                        return f"ERROR:Arm komutu reddedildi. Kod: {ack.result}"
                
                # Timeout durumunda
                print("[WARN] Arm komutu onayı alınamadı (timeout)")
                return "OK:ARM komutu gönderildi, onay bekleniyor"
            except Exception as e:
                print(f"[ERROR] ARM komutu gönderilemedi: {e}")
                return f"ERROR:ARM komutu gönderilemedi: {e}"
        return "ERROR:MAVLink bağlantısı yok"
    
    elif command_type == "DISARM":
        if mav_connection:
            try:
                # Disarm komutu gönder
                mav_connection.mav.command_long_send(
                    mav_connection.target_system,
                    mav_connection.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 0, 0, 0, 0, 0, 0)
                
                # Komutun alındığını doğrula
                ack = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
                if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print("[INFO] Disarm komutu başarıyla kabul edildi")
                        return "OK:DISARM komutu kabul edildi"
                    else:
                        print(f"[WARN] Disarm komutu reddedildi. Sonuç kodu: {ack.result}")
                        return f"ERROR:Disarm komutu reddedildi. Kod: {ack.result}"
                
                # Timeout durumunda
                print("[WARN] Disarm komutu onayı alınamadı (timeout)")
                return "OK:DISARM komutu gönderildi, onay bekleniyor"
            except Exception as e:
                print(f"[ERROR] DISARM komutu gönderilemedi: {e}")
                return f"ERROR:DISARM komutu gönderilemedi: {e}"
        return "ERROR:MAVLink bağlantısı yok"
    
    elif command_type == "MODE":
        if mav_connection:
            try:
                # Mod adını ArduPilot mod numarasına çevir
                mode_id = get_mode_number_from_name(params)
                
                if mode_id is not None:
                    print(f"[INFO] Mod değiştiriliyor: {params} -> Mod ID: {mode_id}")
                    
                    # ArduPilot'ta mod numarasını ayarla
                    # Birinci yöntem: set_mode_send
                    mav_connection.mav.set_mode_send(
                        mav_connection.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        mode_id
                    )
                    
                    time.sleep(0.1)  # Kısa bir bekleme
                    
                    # İkinci yöntem: MAV_CMD_DO_SET_MODE
                    mav_connection.mav.command_long_send(
                        mav_connection.target_system,
                        mav_connection.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                        0,  # Confirmation
                        1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                        mode_id,  # Custom mode
                        0, 0, 0, 0, 0  # Param 3-7 kullanılmıyor
                    )
                    
                    # Debug için mevcut mod durumunu kontrol et
                    for _ in range(5):  # 5 deneme
                        msg = mav_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
                        if msg:
                            current_mode = msg.custom_mode
                            current_mode_name = get_mode_name_from_number(current_mode)
                            print(f"[INFO] Mevcut mod: {current_mode} ({current_mode_name})")
                            if current_mode == mode_id:
                                print(f"[INFO] Mod başarıyla değiştirildi: {params}")
                                return f"OK:Mod değiştirildi: {params}"
                        time.sleep(0.1)
                    
                    # Mod değiştirildi mi kontrol et
                    ack = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
                    if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            print(f"[INFO] Mod değiştirme komutu başarıyla kabul edildi: {params}")
                            return f"OK:Mod değiştirildi: {params}"
                    
                    # Timeout veya red durumunda, yine de OK dönelim (belki başarılı olmuştur)
                    print(f"[WARN] Mod değiştirme komutu için onay alınamadı: {params}")
                    return f"OK:Mod değiştirme komutu gönderildi: {params}"
                    
                elif params == "KAMIKAZE":
                    # Kamikaze modu özel bir durum, START komutunu da tetiklesin
                    thread = threading.Thread(target=run_mission_script)
                    thread.start()
                    return "OK:KAMIKAZE modu başlatıldı"
                else:
                    print(f"[ERROR] Bilinmeyen mod: {params}")
                    return f"ERROR:Bilinmeyen mod: {params} (Bu mod bu araç için desteklenmiyor)"
            except Exception as e:
                print(f"[ERROR] Mod değiştirilemedi: {e}")
                return f"ERROR:Mod değiştirilemedi: {e}"
        return "ERROR:MAVLink bağlantısı yok"
    
    elif command_type == "CAMERA":
        if params == "TOGGLE":
            # Kamera açma/kapama işlemi
            if mav_connection:
                try:
                    # MAVLink kamera kontrolü
                    mav_connection.mav.command_long_send(
                        mav_connection.target_system,
                        mav_connection.target_component,
                        mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
                        0, 0, 0, 0, 1, 0, 0, 0)  # Session control = 1 (başlat/durdur)
                    return "OK:Kamera durumu değiştirildi"
                except Exception as e:
                    print(f"[ERROR] Kamera kontrolü hatası: {e}")
                    return f"ERROR:Kamera durumu değiştirilemedi: {e}"
            return "OK:Kamera durumu değiştirildi (MAVLink bağlantısı olmadan)"
            
        elif params == "SNAPSHOT":
            # Fotoğraf çekme işlemi
            if mav_connection:
                try:
                    # MAVLink kamera kontrolü - fotoğraf çek
                    mav_connection.mav.command_long_send(
                        mav_connection.target_system,
                        mav_connection.target_component,
                        mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
                        0, 0, 0, 0, 0, 0, 1, 0)  # Single shoot = 1
                    return "OK:Fotoğraf çekildi"
                except Exception as e:
                    print(f"[ERROR] Fotoğraf çekme hatası: {e}")
                    return f"ERROR:Fotoğraf çekilemedi: {e}"
            return "OK:Fotoğraf çekildi (MAVLink bağlantısı olmadan)"
            
        else:
            return f"ERROR:Bilinmeyen kamera komutu: {params}"
    
    else:
        return f"ERROR:Bilinmeyen komut: {command_type}"

# MAVLink bağlantısını başlat
init_mavlink()

# Telemetri toplama thread'ini başlat
telemetry_collector_thread = threading.Thread(target=collect_telemetry)
telemetry_collector_thread.daemon = True
telemetry_collector_thread.start()
print("[INFO] Telemetri toplama başlatıldı")

# Son bağlanan istemci adresi
last_client_addr = None
telemetry_thread = None

# Ana döngü
while True:
    try:
        data, addr = sock.recvfrom(1024)
        command = data.decode().strip().upper()
        
        # İstemci adresi değiştiyse veya ilk bağlantıysa telemetri thread'i başlat
        if addr != last_client_addr:
            last_client_addr = addr
            if telemetry_thread is None or not telemetry_thread.is_alive():
                telemetry_thread = threading.Thread(target=send_telemetry, args=(addr,))
                telemetry_thread.daemon = True
                telemetry_thread.start()
                print(f"[INFO] Telemetri gönderimi başlatıldı: {addr}")
        
        # Komutu işle ve yanıt gönder
        response = handle_command(command, addr)
        sock.sendto(response.encode(), addr)
        
    except Exception as e:
        print(f"[ERROR] Komut işleme hatası: {e}")
