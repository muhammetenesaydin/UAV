#!/usr/bin/env python3
"""
İHA Kamikaze Dalış Kontrolcüsü
- Belirli bir hedefe hızlı şekilde yaklaşır
- Hedefi görünce durmadan ani kamikaze dalış yapar
- QR kodu tarayarak hedefi tespit eder
"""
from pymavlink import mavutil
import time
import math
import cv2
from pyzbar.pyzbar import decode
import sys
import json
import os
import numpy as np

class KamikazeUAVController:
    def __init__(self, config_file="config.py", connection_string=None):
        # Konfigürasyon dosyasını yükle
        self.config = self.load_config(config_file)
        
        # Konfigürasyon dosyasından parametreleri al
        # Hedef koordinatlar
        self.target_lat = self.config["hedef"]["lat"]
        self.target_lon = self.config["hedef"]["lon"]
        
        # Uçuş parametreleri
        self.target_altitude = self.config["ucus_parametreleri"]["target_altitude"]
        self.approach_threshold = self.config["ucus_parametreleri"]["approach_threshold"]
        self.camera_url = self.config["ucus_parametreleri"]["camera_url"]
        
        # İHA kalkış koordinatları (global)
        self.home_lat = -35.36328477
        self.home_lon = 149.16512247
        
        # Eski hedef hesaplamaları yerine direkt hedef koordinatları kullanılacak
        # Ancak LOCAL_NED değerlerini hesaplamamız gerekiyor hedefe mesafe hesabı için
        # Kuzey-Doğu yönünde yaklaşık değerler
        self.target_x = (self.target_lon - self.home_lon) * 111111 * math.cos(math.radians(self.home_lat))
        self.target_y = (self.target_lat - self.home_lat) * 111111
        self.target_z = 0.0            # QR kodun z değeri (yükseklik)
        
        # Bağlantı kurulumu
        print("MAVLink bağlantısı kuruluyor...")
        
        # Bağlantı string'i verilmişse onu kullan, yoksa varsayılanı kullan
        if connection_string:
            print(f"Bağlantı string'i kullanılıyor: {connection_string}")
            try:
                self.mav = mavutil.mavlink_connection(connection_string, source_system=255, source_component=0, verbose=True)
            except Exception as e:
                print(f"Bağlantı hatası: {str(e)}")
                print("\nHATA: Simülasyona bağlanılamadı!")
                print("Lütfen şunları kontrol edin:")
                print("1. Ubuntu'da simülasyon çalışıyor mu? (ps aux | grep sim_vehicle.py)")
                print("2. Broadcast portu (14750) açık mı? (sudo netstat -tulpn | grep 14750)")
                print("3. IP adresi doğru mu? (ip addr show)")
                print("4. Firewall ayarları doğru mu? (sudo ufw status)")
                print("\nÖrnek kullanım: python iha_mission.py --connection udp:10.192.55.181:14750")
                sys.exit(1)
        else:
            # Varsayılan bağlantı - broadcast portuna bağlan (simülasyon yayını)
            try:
                print("Simülasyon broadcast portuna bağlanılıyor (14750)...")
                self.mav = mavutil.mavlink_connection('udp:10.192.55.181:14750', source_system=255, source_component=0, verbose=True)
            except Exception as e:
                print(f"Broadcast port bağlantı hatası: {str(e)}")
                print("\nHATA: Simülasyona bağlanılamadı!")
                print("Lütfen şunları kontrol edin:")
                print("1. Ubuntu'da simülasyon çalışıyor mu? (ps aux | grep sim_vehicle.py)")
                print("2. Broadcast portu (14750) açık mı? (sudo netstat -tulpn | grep 14750)")
                print("3. IP adresi doğru mu? (ip addr show)")
                print("4. Firewall ayarları doğru mu? (sudo ufw status)")
                print("\nÖrnek kullanım: python iha_mission.py --connection udp:10.192.55.181:14750")
                sys.exit(1)
        
        self.mav.wait_heartbeat()
        self.system_id = self.mav.target_system
        self.component_id = self.mav.target_component
        print(f"Bağlantı kuruldu! (SYS:{self.system_id}, COMP:{self.component_id})")
        
        # İHA durumu
        self.vehicle_state = {
            'armed': False,
            'mode': None,
            'altitude': 0,
            'airspeed': 0,
            'groundspeed': 0,
            'is_flying': False,
            'lat': 0,
            'lon': 0,
            'x': 0,       # LOCAL_NED X pozisyonu
            'y': 0,       # LOCAL_NED Y pozisyonu
            'z': 0,       # LOCAL_NED Z pozisyonu (-Z yukarı yönü)
            'roll': 0,
            'pitch': 0,
            'yaw': 0
        }
        
        # Hemen durumu kontrol et
        self.update_vehicle_state()
        
    def load_config(self, config_file):
        """Konfigürasyon dosyasını yükler"""
        try:
            # Dosya var mı kontrol et
            if not os.path.exists(config_file):
                print(f"UYARI: {config_file} bulunamadı! Varsayılan değerler kullanılacak.")
                # Varsayılan değerleri döndür
                return self.get_default_config()
            
            # Python dosyası mı yoksa JSON dosyası mı kontrol et
            if config_file.endswith('.py'):
                try:
                    # Python dosyasını modül olarak import et
                    import importlib.util
                    spec = importlib.util.spec_from_file_location("config_module", config_file)
                    config_module = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(config_module)
                    
                    # mission_config değişkenini al
                    if hasattr(config_module, 'mission_config'):
                        config = config_module.mission_config
                        print(f"Python konfigürasyon dosyası başarıyla yüklendi: {config_file}")
                        return config
                    else:
                        print(f"HATA: {config_file} dosyasında 'mission_config' değişkeni bulunamadı!")
                        return self.get_default_config()
                except Exception as e:
                    print(f"Python modül yükleme hatası: {str(e)}")
                    print("Varsayılan değerler kullanılacak.")
                    return self.get_default_config()
            else:
                # JSON dosyasını aç ve JSON olarak yükle
                with open(config_file, 'r') as f:
                    config = json.load(f)
                    print(f"JSON konfigürasyon dosyası başarıyla yüklendi: {config_file}")
                    return config
        except Exception as e:
            print(f"Konfigürasyon dosyası yükleme hatası: {str(e)}")
            print("Varsayılan değerler kullanılacak.")
            return self.get_default_config()
    
    def get_default_config(self):
        """Varsayılan konfigürasyon değerlerini döndürür"""
        return {
            "hedef": {
                "lat": -35.36267270,
                "lon": 149.16521811
            },
            "ucus_parametreleri": {
                "target_altitude": 50,
                "approach_threshold": 25,
                "camera_url": "http://0.0.0.0:8080/stream?topic=/cessna/image_raw"
            },
            "dalis_parametreleri": {
                "TECS_SINK_MAX": 14.0,
                "TECS_SINK_MIN": 6.0,
                "LIM_PITCH_MAX": 4500,
                "LIM_PITCH_MIN": -6000,
                "LAND_PITCH_CD": -4500,
                "TECS_TIME_CONST": 3.0,
                "TECS_SPDWEIGHT": 1.0,
                "THR_MAX": 100.0,
                "WP_RADIUS": 10.0,
                "NAVL1_PERIOD": 15.0
            },
            "gorev_parametreleri": {
                "waypoint_kabul_yaricapi": {
                    "normal": 5,
                    "dalis": 2
                },
                "yaklasim_mesafeleri": {
                    "qr_scan_baslat": 90,
                    "dalis_baslat": 60,
                    "zorla_dalis": 40
                },
                "dalis_hedefi": {
                    "irtifa": 3
                },
                "guvenli_irtifa": 40,
                "kurtarma": {
                    "pitch_up": 1100,
                    "tekrar_sayisi": 3,
                    "throttle": 1300,
                    "beklemeler": {
                        "pitch_up": 0.5,
                        "tam_gaz": 1.0,
                        "qr_tolerans": 1.0
                    },
                    "normal_deger": {
                        "pitch_min": -4500,
                        "trim": -1500
                    }
                }
            },
            "kalkis_parametreleri": {
                "throttle_degerleri": [1600, 1700, 1800, 1900],
                "pitch_up": 1200,
                "throttle_kalkis": 1900
            },
            "qr_tarama": {
                "tarama_siklik": {
                    "normal": 1.5,
                    "dalis": 0.5
                },
                "tarama_suresi": {
                    "normal": 1,
                    "dalis": 2
                },
                "alternatif_kamera_urls": [
                    "http://localhost:8080/stream?topic=/cessna/image_raw",
                    "http://127.0.0.1:8080/stream?topic=/cessna/camera/image",
                    "http://127.0.0.1:8080/?action=stream",
                    0
                ]
            },
            "pitch_override": {
                "normal": 1900,
                "agresif": 1950,
                "tekrarlama": {
                    "normal": 5,
                    "agresif": 10
                }
            }
        }
    
    def update_vehicle_state(self, verbose=True):
        """İHA durumunu günceller"""
        # Veri akışı hızını artır
        self.mav.mav.request_data_stream_send(
            self.system_id, self.component_id,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        
        # Heartbeat - Mod ve ARM durumu
        msg = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            self.vehicle_state['armed'] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            self.vehicle_state['mode'] = self.mav.flightmode
        
        # Altitude ve hız
        msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            self.vehicle_state['altitude'] = msg.relative_alt / 1000  # mm -> m
            self.vehicle_state['lat'] = msg.lat / 1e7
            self.vehicle_state['lon'] = msg.lon / 1e7
            self.vehicle_state['groundspeed'] = math.sqrt(msg.vx**2 + msg.vy**2) / 100  # cm/s -> m/s
        
        # VFR_HUD - Airspeed
        msg = self.mav.recv_match(type='VFR_HUD', blocking=False)
        if msg:
            self.vehicle_state['airspeed'] = msg.airspeed
        
        # LOCAL_POSITION_NED - LOCAL NED pozisyonu
        msg = self.mav.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            self.vehicle_state['x'] = msg.x
            self.vehicle_state['y'] = msg.y
            self.vehicle_state['z'] = msg.z
        
        # ATTITUDE - Attitude
        msg = self.mav.recv_match(type='ATTITUDE', blocking=False)
        if msg:
            self.vehicle_state['roll'] = math.degrees(msg.roll)
            self.vehicle_state['pitch'] = math.degrees(msg.pitch)
            self.vehicle_state['yaw'] = math.degrees(msg.yaw)
        
        # Uçuş durumunu belirle
        altitude = self.vehicle_state['altitude']
        airspeed = self.vehicle_state['airspeed']
        armed = self.vehicle_state['armed']
        groundspeed = self.vehicle_state['groundspeed']
        
        # İHA havada mı?
        self.vehicle_state['is_flying'] = armed and (altitude > 5 or airspeed > 5 or groundspeed > 5)
        
        if verbose:
            print("\n--- İHA DURUM BİLGİSİ ---")
            print(f"Mod: {self.vehicle_state['mode']}")
            print(f"ARM Durumu: {'ARM' if self.vehicle_state['armed'] else 'DISARM'}")
            print(f"İrtifa: {self.vehicle_state['altitude']:.1f}m")
            print(f"Hız: {self.vehicle_state['airspeed']:.1f}m/s (Yer: {self.vehicle_state['groundspeed']:.1f}m/s)")
            print(f"Global Konum: LAT={self.vehicle_state['lat']:.6f}, LON={self.vehicle_state['lon']:.6f}")
            print(f"Local Konum: X={self.vehicle_state['x']:.2f}, Y={self.vehicle_state['y']:.2f}, Z={self.vehicle_state['z']:.2f}")
            print(f"Yuvarlanma: {self.vehicle_state['roll']:.1f}°, Yunuslama: {self.vehicle_state['pitch']:.1f}°")
            print(f"Uçuş Durumu: {'HAVADA' if self.vehicle_state['is_flying'] else 'YERDE'}")
            print(f"Hedef: LAT={self.target_lat:.6f}, LON={self.target_lon:.6f}")
        
        return self.vehicle_state
    
    def change_mode(self, mode, max_attempts=3):
        """Güvenli mod değişimi yapar"""
        current_mode = self.vehicle_state['mode']
        if current_mode == mode:
            print(f"İHA zaten {mode} modunda.")
            return True
            
        print(f"{current_mode} -> {mode} moduna geçiliyor...")
        
        for i in range(max_attempts):
            try:
                mode_id = self.mav.mode_mapping()[mode]
                self.mav.mav.set_mode_send(
                    self.system_id,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id)
                print(f"Mod değişim komutu gönderildi (Deneme {i+1}/{max_attempts})")
                
                # Mod değişimini kontrol et
                time.sleep(1)
                self.update_vehicle_state(verbose=False)
                
                if self.vehicle_state['mode'] == mode:
                    print(f"{mode} moduna başarıyla geçildi!")
                    return True
            except:
                print(f"Mod değişimi hatası ({mode})")
            
            time.sleep(1)
        
        print(f"UYARI: {mode} moduna geçilemedi!")
        return False
    
    def arm_vehicle(self, should_arm=True):
        """İHA'yı güvenli şekilde arm/disarm eder"""
        if self.vehicle_state['armed'] == should_arm:
            print(f"İHA zaten {'ARM' if should_arm else 'DISARM'} durumunda.")
            return True
            
        # Havada disarm etmeye çalışıyorsak engelle
        if not should_arm and self.vehicle_state['is_flying']:
            print("UYARI: Uçuş sırasında disarm işlemi engellendi!")
            return False
            
        print(f"İHA {'ARM' if should_arm else 'DISARM'} ediliyor...")
        
        for i in range(3):
            self.mav.mav.command_long_send(
                self.system_id,
                self.component_id,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1 if should_arm else 0, 0, 0, 0, 0, 0, 0)
            print(f"{'ARM' if should_arm else 'DISARM'} komutu gönderildi. (Deneme {i+1}/3)")
            
            time.sleep(1)
            self.update_vehicle_state(verbose=False)
            
            if self.vehicle_state['armed'] == should_arm:
                print(f"İHA başarıyla {'ARM' if should_arm else 'DISARM'} edildi!")
                return True
                
            time.sleep(1)
        
        print(f"UYARI: {'ARM' if should_arm else 'DISARM'} işlemi başarısız!")
        return False
    
    def detect_qr_code(self, timeout=10):
        """QR kodu algılar"""
        print("QR kod taranıyor...")
        
        try:
            cap = cv2.VideoCapture(self.camera_url)
            if not cap.isOpened():
                print("Kamera bağlantı hatası! URL kontrol ediliyor...")
                # Alternatif kamera URL'leri dene (konfigürasyondan)
                alt_urls = self.config["qr_tarama"]["alternatif_kamera_urls"]
                
                for url in alt_urls:
                    print(f"Alternatif kamera URL deneniyor: {url}")
                    cap = cv2.VideoCapture(url)
                    if cap.isOpened():
                        print(f"Alternatif kamera bağlantısı başarılı: {url}")
                        self.camera_url = url  # Çalışan URL'yi kaydet
                        break
                
                if not cap.isOpened():
                    print("Tüm kamera bağlantı denemeleri başarısız!")
                    return None
            
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                ret, frame = cap.read()
                if not ret:
                    print("Kamera görüntüsü alınamadı!")
                    time.sleep(0.1)
                    continue
                
                # QR kod işleme hatasını önlemek için görüntüyü küçült ve griye çevir
                try:
                    # Görüntüyü düzenli aralıklarla kaydet (debug için)
                    if (time.time() - start_time) % 3 < 0.1:
                        debug_filename = f"qr_scan_{int(time.time())}.jpg"
                        cv2.imwrite(debug_filename, frame)
                        print(f"Kamera görüntüsü kaydedildi: {debug_filename}")
                    
                    # Görüntüyü daha kolay işlenmesi için dönüştür
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    # Görüntüyü düzeltme
                    gray = cv2.GaussianBlur(gray, (5, 5), 0)
                    
                    # QR kodunu bul
                    decoded = decode(gray)
                    if not decoded:
                        # Orijinal görüntü üzerinde de dene
                        decoded = decode(frame)
                    
                    if decoded:
                        qr_data = decoded[0].data.decode()
                        print(f"QR KOD OKUNDU: {qr_data}")
                        
                        # Tespit edilen QR kodun konumunu çiz
                        points = decoded[0].polygon
                        if points:
                            pts = np.array([list(point) for point in points], np.int32)
                            pts = pts.reshape((-1, 1, 2))
                            cv2.polylines(frame, [pts], True, (0, 255, 0), 3)
                        
                        # QR kodun görüntüsünü kaydet
                        cv2.imwrite("qr_detected.jpg", frame)
                        print("QR kod görüntüsü kaydedildi!")
                        
                        cap.release()
                        return qr_data
                except Exception as e:
                    print(f"QR kod işleme hatası: {str(e)}")
                
                cv2.waitKey(1)
                
            print(f"QR kod tarama zaman aşımı ({timeout}s)!")
            return None
        
        except Exception as e:
            print(f"Genel QR tarama hatası: {str(e)}")
            return None
        
        finally:
            if cap is not None and cap.isOpened():
                cap.release()
    
    def set_kamikaze_parameters(self):
        """İHA'yı kamikaze dalış için ayarlar - süper agresif parametreler"""
        print("\n=== KAMIKAZE PARAMETRELER AYARLANIYOR ===")
        
        # Konfigürasyondan dalış parametrelerini al
        dalis_params = self.config["dalis_parametreleri"]
        
        # 0. Kontrol parametreleri koruması (güvenlik)
        # ArduPlane kontrolcüsünün SKIP_SERVO_TEST parametresini devre dışı bırak (test olmadan servolar çalışsın)
        self.mav.mav.param_set_send(
            self.system_id, self.component_id,
            b'ARMING_REQUIRE', 0,  # Hiçbir şey gerektirme
            mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        print("Arming koruma devre dışı")
        
        # 1. Maksimum alçalma hızı - çok agresif
        self.mav.mav.param_set_send(
            self.system_id, self.component_id,
            b'TECS_SINK_MAX', dalis_params["TECS_SINK_MAX"],  # Konfigürasyondan al
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        print(f"Maksimum alçalma hızı {dalis_params['TECS_SINK_MAX']:.1f} m/s olarak ayarlandı")
        
        # 2. Minimum alçalma hızı - agresif
        self.mav.mav.param_set_send(
            self.system_id, self.component_id,
            b'TECS_SINK_MIN', dalis_params["TECS_SINK_MIN"],  # Konfigürasyondan al
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        print(f"Minimum alçalma hızı {dalis_params['TECS_SINK_MIN']:.1f} m/s olarak ayarlandı")
        
        # 3. Pitch açısı limitleri - çok agresif
        self.mav.mav.param_set_send(
            self.system_id, self.component_id,
            b'LIM_PITCH_MAX', dalis_params["LIM_PITCH_MAX"],  # Konfigürasyondan al
            mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            
        self.mav.mav.param_set_send(
            self.system_id, self.component_id,
            b'LIM_PITCH_MIN', dalis_params["LIM_PITCH_MIN"],  # Konfigürasyondan al
            mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        print(f"Pitch açısı limitleri {dalis_params['LIM_PITCH_MIN']/100:.0f}/{dalis_params['LIM_PITCH_MAX']/100:.0f} derece olarak ayarlandı")
        
        # 4. İniş yaklaşma açısı - dik
        self.mav.mav.param_set_send(
            self.system_id, self.component_id, 
            b'LAND_PITCH_CD', dalis_params["LAND_PITCH_CD"],  # Konfigürasyondan al
            mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        print(f"İniş pitch açısı {dalis_params['LAND_PITCH_CD']/100:.0f} derece olarak ayarlandı")
        
        # 5. TECS tepki süresi - kontrollü
        self.mav.mav.param_set_send(
            self.system_id, self.component_id,
            b'TECS_TIME_CONST', dalis_params["TECS_TIME_CONST"],  # Konfigürasyondan al
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        print(f"TECS tepki süresi {dalis_params['TECS_TIME_CONST']:.1f} olarak ayarlandı")
        
        # 6. Hız kontrolünü dengeli tut
        self.mav.mav.param_set_send(
            self.system_id, self.component_id,
            b'TECS_SPDWEIGHT', dalis_params["TECS_SPDWEIGHT"],  # Konfigürasyondan al
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        print(f"Hız kontrol ağırlığı {dalis_params['TECS_SPDWEIGHT']:.1f} olarak ayarlandı")
        
        # 7. Throttle max
        self.mav.mav.param_set_send(
            self.system_id, self.component_id,
            b'THR_MAX', dalis_params["THR_MAX"],  # Konfigürasyondan al
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        print(f"Throttle limiti %{dalis_params['THR_MAX']:.0f} olarak ayarlandı")
        
        # 8. Navigasyon hata kontrolü - düzgün yaklaşım için
        self.mav.mav.param_set_send(
            self.system_id, self.component_id,
            b'WP_RADIUS', dalis_params["WP_RADIUS"],  # Konfigürasyondan al
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        print(f"Waypoint yarıçapı {dalis_params['WP_RADIUS']:.1f}m olarak ayarlandı")
        
        # 9. L1 kontrolör parametreleri - düzgün yörünge takibi için
        self.mav.mav.param_set_send(
            self.system_id, self.component_id,
            b'NAVL1_PERIOD', dalis_params["NAVL1_PERIOD"],  # Konfigürasyondan al
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        print(f"L1 periyodu {dalis_params['NAVL1_PERIOD']:.1f} olarak ayarlandı")
        
        # 10. Agresif dalış için ek parametreler
        self.mav.mav.param_set_send(
            self.system_id, self.component_id,
            b'PTCH2SRV_TCONST', 0.3,  # Pitch servo zaman sabiti (daha hızlı tepki)
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            
        self.mav.mav.param_set_send(
            self.system_id, self.component_id,
            b'PTCH2SRV_RLL', 1.2,  # Roll to pitch etkisi (daha az karşı tepki)
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            
        self.mav.mav.param_set_send(
            self.system_id, self.component_id,
            b'TRIM_PITCH_CD', -1500,  # Negatif pitch trim (burun aşağı eğilim)
            mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        
        print("Agresif dalış parametreleri ayarlandı!")
        
        return True
    
    def create_kamikaze_mission(self):
        """Hedef konuma ani dalış misyonu oluşturur - düz yaklaşım ve direkt dalış"""
        print("\n=== KAMIKAZE DALIŞ MİSYONU OLUŞTURULUYOR ===")
        
        # Görev parametrelerini konfigürasyondan al
        gorev_params = self.config["gorev_parametreleri"]
        waypoint_radius_normal = gorev_params["waypoint_kabul_yaricapi"]["normal"]
        waypoint_radius_dalis = gorev_params["waypoint_kabul_yaricapi"]["dalis"]
        dalis_hedef_irtifa = gorev_params["dalis_hedefi"]["irtifa"]
        guvenli_irtifa = gorev_params["guvenli_irtifa"]
        
        # İHA'nın mevcut durumunu güncelle
        self.update_vehicle_state()
        current_lat = self.vehicle_state['lat']
        current_lon = self.vehicle_state['lon']
        current_alt = self.vehicle_state['altitude']
        
        # Uçuş irtifası (havada değilse konfigürasyondaki değer, havadaysa mevcut irtifa)
        flight_alt = current_alt if self.vehicle_state['is_flying'] and current_alt > 30 else self.target_altitude
        
        # Misyonu temizle
        self.mav.mav.mission_clear_all_send(self.system_id, self.component_id)
        time.sleep(1)
        
        # Misyon öğelerini oluştur - DÜZ YAKLAŞIM ve DALIŞLI
        mission_items = []
        
        # 0. ITEM: HOME
        mission_items.append({
            'seq': 0,
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            'current': 0,
            'autocontinue': 1,
            'param1': 0,  # Hold time
            'param2': 0,  # Accept radius
            'param3': 0,  # Pass radius
            'param4': 0,  # Yaw
            'x': int(current_lat * 1e7),
            'y': int(current_lon * 1e7),
            'z': float(0)  # Göreceli yükseklik
        })
        
        # 1. ITEM: HIZ AYARLA - ÇOK HIZLI
        mission_items.append({
            'seq': 1,
            'frame': mavutil.mavlink.MAV_FRAME_MISSION,
            'command': mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            'current': 1,  # İlk aktif komut
            'autocontinue': 1,
            'param1': 0,   # 0=hava hızı
            'param2': 30,  # Hız (m/s) - ÇOK HIZLI
            'param3': 100, # Throttle - TAM GAZ
            'param4': 0,
            'x': 0,
            'y': 0,
            'z': 0
        })
        
        # 2. ITEM: HEDEFE DOĞRU İLERLEME - BELİRLİ İRTİFADA (eski 3. waypoint)
        mission_items.append({
            'seq': 2, 
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            'current': 0,
            'autocontinue': 1,
            'param1': 0,    # Beklemeden geç
            'param2': waypoint_radius_normal,  # Accept radius (konfigürasyondan)
            'param3': 0,    # Pass radius
            'param4': 0,    # Yaw (hedefe bakacak şekilde otomatik)
            'x': int(self.target_lat * 1e7),
            'y': int(self.target_lon * 1e7), 
            'z': float(flight_alt)  # Belirlenen uçuş irtifası
        })
        
        # 3. ITEM: KAMIKAZE DALİŞ - DÜŞEY İNİŞ (eski 4. waypoint)
        mission_items.append({
            'seq': 3,
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            'current': 0, 
            'autocontinue': 1,
            'param1': 0,   # Beklemeden geç
            'param2': waypoint_radius_dalis,  # Accept radius (konfigürasyondan)
            'param3': 0,   # Pass radius
            'param4': 0,   # Yaw
            'x': int(self.target_lat * 1e7),
            'y': int(self.target_lon * 1e7),
            'z': float(dalis_hedef_irtifa)  # Dalış hedef irtifası (konfigürasyondan)
        })
        
        # 4. ITEM: GÜVENLİ İRTİFAYA DÖNÜŞ (eski 5. waypoint)
        mission_items.append({
            'seq': 4,
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            'current': 0,
            'autocontinue': 1,
            'param1': 0,   # Bekleme yok
            'param2': 15,  # Accept radius
            'param3': 0,   # Pass radius
            'param4': 0,   # Yaw
            'x': int(self.target_lat * 1e7),
            'y': int(self.target_lon * 1e7),
            'z': float(guvenli_irtifa)  # Güvenli irtifa (konfigürasyondan)
        })
        
        # 5. ITEM: RTL - Eve dönüş (eski 6. waypoint)
        mission_items.append({
            'seq': 5,
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            'command': mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            'current': 0,
            'autocontinue': 1,
            'param1': 0,
            'param2': 0,
            'param3': 0,
            'param4': 0,
            'x': 0,
            'y': 0,
            'z': 0
        })
        
        # Misyon sayısını bildir
        self.mav.mav.mission_count_send(
            self.system_id,
            self.component_id,
            len(mission_items)
        )
        
        # Misyon öğesi isteklerini bekle ve gönder
        for i in range(len(mission_items)):
            msg = self.mav.recv_match(type=['MISSION_REQUEST'], blocking=True, timeout=3)
            if not msg:
                print(f"MISSION_REQUEST alınamadı (item {i})!")
                return False
            
            print(f"Dalış misyonu öğesi {msg.seq} istendi.")
            item = mission_items[msg.seq]
            
            # MISSION_ITEM_INT kullan
            self.mav.mav.mission_item_int_send(
                self.system_id,
                self.component_id,
                item['seq'],
                item['frame'],
                item['command'],
                item['current'],
                item['autocontinue'],
                item['param1'],
                item['param2'],
                item['param3'],
                item['param4'],
                item['x'],      # Int32 olarak x*1e7
                item['y'],      # Int32 olarak y*1e7
                item['z']       # Float32 olarak z
            )
            
            print(f"Dalış misyonu öğesi {item['seq']} gönderildi")
        
        # Mission kabul mesajını bekle
        msg = self.mav.recv_match(type=['MISSION_ACK'], blocking=True, timeout=3)
        if not msg:
            print("Dalış misyonu ACK alınamadı!")
            return False
            
        if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print("Kamikaze dalış misyonu başarıyla yüklendi!")
            
            # İlave: Başlangıç öğesini belirle (hız değişikliği)
            self.mav.mav.command_long_send(
                self.system_id, self.component_id,
                mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                0, 1, 0, 0, 0, 0, 0, 0)
            
            return True
        else:
            print(f"Dalış misyonu hatası: {msg.type}")
            return False
    
    def perform_kamikaze_mission(self):
        """Kamikaze dalış misyonunu gerçekleştirir - düz yaklaşım ve direkt dalış"""
        print("\n=== KAMIKAZE DALIŞI BAŞLATILIYOR ===")
        
        # Konfigürasyondan yaklaşma mesafelerini al
        yaklasim_mesafeleri = self.config["gorev_parametreleri"]["yaklasim_mesafeleri"]
        qr_scan_baslat = yaklasim_mesafeleri["qr_scan_baslat"]
        dalis_baslat = yaklasim_mesafeleri["dalis_baslat"]
        zorla_dalis = yaklasim_mesafeleri["zorla_dalis"]
        
        # Dalış süresi parametrelerini al
        dalis_suresi = self.config["gorev_parametreleri"]["dalis_suresi"]
        min_dalis_suresi = dalis_suresi["min_sure"]
        ekstra_bekleme = dalis_suresi["ekstra_bekleme"]
        max_dalis_suresi = dalis_suresi["max_sure"]
        rtl_gecikme = dalis_suresi["rtl_gecikme"]
        
        # QR tarama sıklıklarını al
        qr_tarama = self.config["qr_tarama"]
        normal_tarama_siklik = qr_tarama["tarama_siklik"]["normal"]
        dalis_tarama_siklik = qr_tarama["tarama_siklik"]["dalis"]
        normal_tarama_suresi = qr_tarama["tarama_suresi"]["normal"]
        dalis_tarama_suresi = qr_tarama["tarama_suresi"]["dalis"]
        
        # Pitch override değerlerini al
        pitch_override = self.config["pitch_override"]
        normal_pitch = pitch_override["normal"]
        agresif_pitch = pitch_override["agresif"]
        normal_tekrar = pitch_override["tekrarlama"]["normal"]
        agresif_tekrar = pitch_override["tekrarlama"]["agresif"]

        # Kurtarma parametrelerini al
        kurtarma_params = self.config["gorev_parametreleri"]["kurtarma"]
        pitch_up = kurtarma_params["pitch_up"]
        tekrar_sayisi = kurtarma_params["tekrar_sayisi"]
        throttle = kurtarma_params["throttle"]
        pitch_bekleme = kurtarma_params["beklemeler"]["pitch_up"]
        tam_gaz_bekleme = kurtarma_params["beklemeler"]["tam_gaz"]
        normal_pitch_min = kurtarma_params["normal_deger"]["pitch_min"]
        normal_trim = kurtarma_params["normal_deger"]["trim"]
        
        # Kamikaze parametrelerini ayarla
        self.set_kamikaze_parameters()
        
        # Kamikaze misyonu oluştur
        if not self.create_kamikaze_mission():
            print("Kamikaze misyonu oluşturulamadı!")
            return False
        
        # AUTO moda geç
        if not self.change_mode('AUTO'):
            print("AUTO moda geçilemedi! Manuel kamikaze deneniyor...")
            
            # Manuel yardım: RC override ile burnu aşağı zorla
            for pitch in [1600, 1700, 1800]:
                self.mav.mav.rc_channels_override_send(
                    self.system_id, self.component_id,
                    0, pitch, 0, 0, 0, 0, 0, 0)  # Kanal 2 (pitch) - burnu aşağı
                time.sleep(0.2)
            
            if not self.change_mode('AUTO'):
                print("Kamikaze dalış başlatılamadı!")
                return False
                
        # Dalış takibi
        print("Kamikaze dalış aktif! Hedefe düz bir şekilde ilerliyor...")
        
        mission_start_time = time.time()
        mission_timeout = 180  # 3 dakika timeout - daha uzun süre ver (kalkış+yaklaşma+dalış)
        qr_found = False
        min_altitude = self.vehicle_state['altitude']
        last_waypoint = -1
        dalish_begin = False
        last_distance_check = 0  # Son mesafe kontrolü zamanı
        closest_distance = 9999  # En yakın mesafe takibi
        last_qr_scan = 0         # Son QR tarama zamanı
        continuous_qr_scan = False  # Sürekli QR tarama modu
        active_diving = False    # Aktif dalış durumu
        dalis_baslama_zamani = 0 # Dalış başlama zamanı
        dalis_min_irtifa_zamani = 0 # Dalış minimum irtifa ulaşma zamanı
        hedef_irtifaya_ulasti = False # Hedef irtifaya ulaştı mı?
        dalis_geciktirme_basladi = False # Dalış sonrası geciktirme başladı mı?
        kurtarma_yapildi = False  # Kurtarma yapıldı mı?
        
        # Dalış ve yaklaşımı takip et
        while time.time() - mission_start_time < mission_timeout:
            self.update_vehicle_state(verbose=False)
            current_alt = self.vehicle_state['altitude']
            current_mode = self.vehicle_state['mode']
            
            # RTL modunu tespit et - normal görevin parçası, durdurmaya çalışma
            if current_mode == 'RTL':
                if not kurtarma_yapildi:
                    print("RTL MODUNA ERİŞİLDİ FAKAT KURTARMA YAPILMADI! ACİL BURUN YUKARI KOMUTLARI BAŞLATILIYOR")
                    
                    # ACİL KURTARMA MODU - DAHA HIZLI VE ETKİLİ
                    for i in range(tekrar_sayisi * 4):  # Daha fazla tekrar 
                        self.mav.mav.rc_channels_override_send(
                            self.system_id, self.component_id,
                            0, 1000, 0, 0, 0, 0, 0, 0)  # Pitch up en yukarı (1000 değeri)
                        time.sleep(0.05)  # Daha hızlı komut gönderimi
                    
                    # Tam motor gücü ve burun yukarı - DAHA AGRESİF
                    for i in range(10):  # Daha fazla tekrar
                        self.mav.mav.rc_channels_override_send(
                            self.system_id, self.component_id,
                            0, 1000, 2000, 0, 0, 0, 0, 0)  # Maksimum yukarı + tam gaz
                        time.sleep(0.1)  # Hızlı komut tekrarı
                    
                    # Normal kontrole geri dön
                    self.mav.mav.rc_channels_override_send(
                        self.system_id, self.component_id,
                        0, 0, 0, 0, 0, 0, 0, 0)
                    
                    kurtarma_yapildi = True
                
                print("RTL moduna geçildi - Dalış tamamlandı")
                # Loglama için yakınlık bilgisini ekle
                print(f"En yakın mesafe: {closest_distance:.1f}m, Minimum irtifa: {min_altitude:.1f}m")
                break  # Görevi sonlandır, RTL'ye izin ver
            
            # Minimum irtifayı takip et
            if current_alt < min_altitude:
                min_altitude = current_alt
                # Eğer aktif dalış sırasında minimum irtifaya ulaşmışsa zamanı kaydet
                if active_diving and not hedef_irtifaya_ulasti and current_alt < 5:  # 5 metre irtifada hedefin yakınındayız kabul et
                    hedef_irtifaya_ulasti = True
                    dalis_min_irtifa_zamani = time.time()
                    print(f"HEDEF İRTİFAYA (yaklaşık) ULAŞILDI: {current_alt:.1f}m")
                    
                    # Hedef irtifaya ulaşıldığında burnu dikme - acil kurtarma
                    if not qr_found and not kurtarma_yapildi:  # QR bulunmadıysa hemen
                        print("HEDEF İRTİFADA ACİL KURTARMA - BURUN YUKARI")
                        
                        # Biraz daha bekleme - QR tarama şansı
                        time.sleep(0.5)
                        
                        # Burnu hızla yukarı çevir - maksimum burun yukarı komutu
                        for i in range(tekrar_sayisi + 2):  # Çokça tekrar et
                            self.mav.mav.rc_channels_override_send(
                                self.system_id, self.component_id,
                                0, pitch_up, 0, 0, 0, 0, 0, 0)  # Pitch up
                            time.sleep(pitch_bekleme)
                        
                        # Tam gaz ve burun yukarı
                        self.mav.mav.rc_channels_override_send(
                            self.system_id, self.component_id,
                            0, pitch_up, throttle, 0, 0, 0, 0, 0)  # Pitch up + tam gaz
                            
                        time.sleep(tam_gaz_bekleme)  # Tam gaz süresince bekle
                        
                        # Parametreleri normale döndür
                        self.mav.mav.param_set_send(
                            self.system_id, self.component_id,
                            b'LIM_PITCH_MIN', normal_pitch_min,  # Normal burun aşağı limit
                            mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                        
                        self.mav.mav.param_set_send(
                            self.system_id, self.component_id,
                            b'TRIM_PITCH_CD', normal_trim,  # Normal trim
                            mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                        
                        # Güvenli irtifaya dönüş (waypoint 4)
                        for _ in range(3):
                            self.mav.mav.command_long_send(
                                self.system_id, self.component_id,
                                mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                                0, 4, 0, 0, 0, 0, 0, 0)
                            time.sleep(0.3)
                            
                        kurtarma_yapildi = True
            
            # Mesafeyi hesapla
            lat1, lon1 = self.vehicle_state['lat'], self.vehicle_state['lon'] 
            lat2, lon2 = self.target_lat, self.target_lon
            
            # Dünya yarıçapı (metre)
            R = 6371000.0
            
            # Radyana çevirme
            phi1 = math.radians(lat1)
            phi2 = math.radians(lat2)
            delta_phi = math.radians(lat2 - lat1)
            delta_lambda = math.radians(lon2 - lon1)
            
            a = math.sin(delta_phi/2.0) * math.sin(delta_phi/2.0) + \
                math.cos(phi1) * math.cos(phi2) * \
                math.sin(delta_lambda/2.0) * math.sin(delta_lambda/2.0)
                
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
            distance_to_target = R * c  # metre cinsinden mesafe
            
            # En yakın mesafeyi güncelle
            if distance_to_target < closest_distance:
                closest_distance = distance_to_target
                print(f"En yakın mesafe güncellendi: {closest_distance:.1f}m")
            
            # Düzenli aralıklarla mesafe kontrolü (2 saniyede bir)
            if time.time() - last_distance_check > 2:
                last_distance_check = time.time()
                print(f"Anlık mesafe: {distance_to_target:.1f}m, En yakın: {closest_distance:.1f}m, İrtifa: {current_alt:.1f}m")
                
                # Dalış süresi bilgisi (aktif dalışta)
                if active_diving:
                    dalis_gecen_sure = time.time() - dalis_baslama_zamani
                    print(f"Dalış süresi: {dalis_gecen_sure:.1f}s / Min: {min_dalis_suresi}s")
                    
                    # Dalış sonrası geciktirme kontrolü
                    if hedef_irtifaya_ulasti:
                        irtifa_sonrasi_sure = time.time() - dalis_min_irtifa_zamani
                        print(f"Hedef irtifada geçen süre: {irtifa_sonrasi_sure:.1f}s / Hedef: {ekstra_bekleme}s")
                        
                        # Minimum dalış süresi sağlandı ve ekstra bekleme süresi doldu ise - bir sonraki waypoint'e geç
                        if irtifa_sonrasi_sure > ekstra_bekleme and dalis_gecen_sure > min_dalis_suresi and not dalis_geciktirme_basladi:
                            print(f"YETERLİ DALIŞ SÜRESİ TAMAMLANDI - GÜVENLİ İRTİFAYA DÖNÜŞ BAŞLIYOR ({dalis_gecen_sure:.1f}s)")
                            dalis_geciktirme_basladi = True
                            
                            # Güvenli irtifaya dönüş (waypoint 4)
                            for _ in range(3):  # Garanti olması için 3 kez dene
                                self.mav.mav.command_long_send(
                                    self.system_id, self.component_id,
                                    mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                                    0, 4, 0, 0, 0, 0, 0, 0)
                                time.sleep(0.3)
                
                # Hedefe yaklaşıldığında sürekli QR tarama modunu etkinleştir
                if distance_to_target < qr_scan_baslat and current_alt < 60 and not continuous_qr_scan:
                    print("HEDEF YAKINDA - SÜREKLİ QR TARAMA BAŞLIYOR")
                    continuous_qr_scan = True
                
                # Hedefe yaklaşıldığında dalış modu aktifleştir
                if distance_to_target < dalis_baslat and not dalish_begin and not kurtarma_yapildi:
                    print("!!! HEDEF YAKINDA - DALIŞ BAŞLATILIYOR (MESAFE: {:.1f}m) !!!".format(distance_to_target))
                    print("SEBEBI: HEDEFE YAKLAŞMA MESAFESI KRITERINE GÖRE DALIŞ BAŞLATILIYOR")
                    dalish_begin = True
                    active_diving = True
                    dalis_baslama_zamani = time.time()  # Dalış başlama zamanını kaydet
                    print("DALIŞ SÜRESİ SAYACI BAŞLATILDI - MİNİMUM: {}s, MAKSİMUM: {}s".format(min_dalis_suresi, max_dalis_suresi))
                    
                    # Waypoint 3'e atla (kamikaze dalış noktası)
                    for _ in range(3):  # Garanti olması için 3 kez dene
                        self.mav.mav.command_long_send(
                            self.system_id, self.component_id,
                            mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                            0, 3, 0, 0, 0, 0, 0, 0)
                        time.sleep(0.3)
                    
                    # Kamikaze parametrelerini maksimum agresifliğe ayarla
                    self.mav.mav.param_set_send(
                        self.system_id, self.component_id,
                        b'TECS_SINK_MAX', 15.0,  # Daha hızlı alçalma için zorla
                        mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    
                    self.mav.mav.param_set_send(
                        self.system_id, self.component_id,
                        b'LIM_PITCH_MIN', -30000,  # -300 derece (çok dik)
                        mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                    
                    self.mav.mav.param_set_send(
                        self.system_id, self.component_id,
                        b'LAND_PITCH_CD', -30000,  # -300 derece (çok dik iniş açısı)
                        mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                        
                    self.mav.mav.param_set_send(
                        self.system_id, self.component_id,
                        b'TRIM_PITCH_CD', -2000,  # Negatif pitch trim (burun aşağıda)
                        mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                    
                    # Arduino'nun dalış parametrelerini hızlı ayarlaması için MAVLINK protokol hızını artır
                    self.mav.mav.request_data_stream_send(
                        self.system_id, self.component_id,
                        mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)
                    
                    print("!!! SÜPERAGRESİF ACİL DALIŞ BAŞLADI !!!")
                    
                    # Pitch kontrolü - aşırı agresif burun aşağı
                    pitch_override_value = self.config["pitch_override"]["agresif"]
                    tekrar_sayisi = self.config["pitch_override"]["tekrarlama"]["agresif"]
                    
                    # Aşırı agresif burun aşağı komutlarını tekrarla (daha fazla tekrar)
                    for i in range(tekrar_sayisi + 5): # Acil durumda daha fazla tekrarla
                        self.mav.mav.rc_channels_override_send(
                            self.system_id, self.component_id,
                            0, pitch_override_value, 0, 0, 0, 0, 0, 0)
                        time.sleep(0.1) # Daha hızlı tekrarla
                    
                    # Throttle'ı düşür (motor gücü azalt)
                    self.mav.mav.rc_channels_override_send(
                        self.system_id, self.component_id,
                        0, pitch_override_value, 1300, 0, 0, 0, 0, 0) # Düşük throttle
                    time.sleep(0.5)
                    
                    # Kontrol serbest bırak ve aviyoniklerin dalışı ele almasını bekle
                    self.mav.mav.rc_channels_override_send(
                        self.system_id, self.component_id,
                        0, 0, 0, 0, 0, 0, 0, 0)
                        
                    
                    time.sleep(0.5)  # Kısa bekle
            
            # Mevcut waypoint'i öğren
            try:
                msg = self.mav.recv_match(type='MISSION_CURRENT', blocking=False)
                if msg and msg.seq != last_waypoint:
                    last_waypoint = msg.seq
                    print(f"Yeni waypoint: {last_waypoint}")
                    
                    # Yaklaşma başladığında (2. waypoint) - eski: 3
                    if last_waypoint == 2:
                        print("HEDEFE DOĞRU DÜZ YAKLAŞIM BAŞLADI")
                        
                        # Düz yönelimi güçlendir
                        self.mav.mav.param_set_send(
                            self.system_id, self.component_id,
                            b'NAVL1_PERIOD', 15.0,
                            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                    
                    # Dalış başladığında (3. waypoint) - eski: 4
                    elif last_waypoint == 3:
                        if not dalish_begin:  # Eğer dalış henüz başlamadıysa
                            print("!!! KAMİKAZE DALIŞ BAŞLATILDI !!!")
                         
                            dalish_begin = True
                            active_diving = True
                            continuous_qr_scan = True  # Dalış başladığında sürekli QR tarama başlat
                            
                            # Dalış başlama zamanını kaydet (waypoint'ten)
                            if dalis_baslama_zamani == 0:
                                dalis_baslama_zamani = time.time()
                                print("DALIŞ SÜRESİ SAYACI BAŞLATILDI - MİNİMUM: {}s, MAKSİMUM: {}s".format(min_dalis_suresi, max_dalis_suresi))
                            
                            # Pitch aşağı komutunu güçlendir
                            for i in range(normal_tekrar):
                                self.mav.mav.rc_channels_override_send(
                                    self.system_id, self.component_id,
                                    0, normal_pitch, 0, 0, 0, 0, 0, 0)  # Pitch down
                                time.sleep(0.2)
                            
                            # Kontrolü tekrar AUTO'ya bırak
                            self.mav.mav.rc_channels_override_send(
                                self.system_id, self.component_id,
                                0, 0, 0, 0, 0, 0, 0, 0)
                        else:
                            print("Dalış zaten başlatıldı, tekrar dalış komutları gönderilmeyecek.")
                    # Güvenli irtifaya yükselme (waypoint 4) - eski: 5
                    elif last_waypoint == 4:
                        print("DALIŞTAN SONRA GÜVENLİ İRTİFAYA YÜKSELİYOR")
                        active_diving = False  # Dalış tamamlandı
                        
                        # Konfigürasyondan kurtarma parametrelerini al
                        kurtarma_params = self.config["gorev_parametreleri"]["kurtarma"]
                        pitch_up = kurtarma_params["pitch_up"]
                        tekrar_sayisi = kurtarma_params["tekrar_sayisi"]
                        throttle = kurtarma_params["throttle"]
                        pitch_bekleme = kurtarma_params["beklemeler"]["pitch_up"]
                        tam_gaz_bekleme = kurtarma_params["beklemeler"]["tam_gaz"]
                        normal_pitch_min = kurtarma_params["normal_deger"]["pitch_min"]
                        normal_trim = kurtarma_params["normal_deger"]["trim"]
                        
                        # Kurtarma rutini - burnu yukarı dikme
                        print("KURTARMA MODU - BURUN YUKARI KOMUTLARI BAŞLADI")
                        
                        # Burnu hızla yukarı çevir - maksimum burun yukarı komutu
                        for i in range(tekrar_sayisi):  # Konfigürasyondan alınan sayı kadar
                            self.mav.mav.rc_channels_override_send(
                                self.system_id, self.component_id,
                                0, pitch_up, 0, 0, 0, 0, 0, 0)  # Pitch up
                            time.sleep(pitch_bekleme)  # Konfigürasyondan alınan bekleme süresi
                        
                        # Tam gaz ve burun yukarı - hızlı tırmanış
                        self.mav.mav.rc_channels_override_send(
                            self.system_id, self.component_id,
                            0, pitch_up, throttle, 0, 0, 0, 0, 0)  # Pitch up + tam gaz
                        time.sleep(tam_gaz_bekleme)  # Konfigürasyondan alınan bekleme süresi
                        
                        # Pitch yukarı tutmaya devam et
                        for i in range(5):
                            self.mav.mav.rc_channels_override_send(
                                self.system_id, self.component_id,
                                0, pitch_up, throttle, 0, 0, 0, 0, 0)  # Pitch up + tam gaz
                            time.sleep(0.2)
                            
                        # Normal kontrole geri dön
                        self.mav.mav.rc_channels_override_send(
                            self.system_id, self.component_id,
                            0, 0, 0, 0, 0, 0, 0, 0)
                            
                        # Parametreleri normale döndür
                        self.mav.mav.param_set_send(
                            self.system_id, self.component_id,
                            b'LIM_PITCH_MIN', normal_pitch_min,  # Normal burun aşağı limit (konfigden)
                            mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                        
                        
                        self.mav.mav.param_set_send(
                            self.system_id, self.component_id,
                            b'TRIM_PITCH_CD', 0,  # Normal trim
                            mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                            
                        self.mav.mav.param_set_send(
                            self.system_id, self.component_id,
                            b'TECS_SINK_MAX', 5.0,  # Normal alçalma hızı
                            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                        
                        # Son güvenlik kontrolü - minimum dalış süresini kontrol et
                        dalis_gecen_sure = time.time() - dalis_baslama_zamani
                        if dalis_gecen_sure < min_dalis_suresi:
                            print(f"UYARI: Dalış çok kısa sürdü ({dalis_gecen_sure:.1f}s < {min_dalis_suresi}s)")
                        
                        # RTL'e geçmeden önce belirtilen süre kadar bekle
                        print(f"RTL'e geçmeden önce {rtl_gecikme}s beklenecek...")
                        time.sleep(rtl_gecikme)
                        
                        # RTL'e geçmeden önce çok daha az bekle - hızlı RTL geçişi
                        print("HIZLI RTL'E GEÇİŞ YAPILIYOR...")
                        time.sleep(1.0)  # Sadece 1 saniye bekle
                        
                        # RTL'e geçmeden önce hızlı yükseliş
                        print("RTL ÖNCESİ SON BURUN YUKARI KOMUTLARI...")
                        for i in range(10):
                            self.mav.mav.rc_channels_override_send(
                                self.system_id, self.component_id,
                                0, 1000, 2000, 0, 0, 0, 0, 0)  # Maksimum yukarı + tam gaz
                            time.sleep(0.05)  # Hızlı komut gönderimi
                    
                    # RTL moduna geçiş (waypoint 5) - eski: 6
                    elif last_waypoint == 5:
                        print("GÖREV TAMAMLANDI - RTL MODUNA GEÇECEK")
                        break  # Döngüden çık, RTL'ye izin ver
            except:
                pass
                
            # QR taraması yap - sürekli tarama modunda ya da dalış sırasında
            # Dalış sırasında daha agresif tarama yap (sık ve uzun)
            if active_diving and (last_waypoint == 3 or dalish_begin):  # eski: 4
                # Dalış sırasında sürekli QR tarama yapma sıklığını artır
                if time.time() - last_qr_scan > dalis_tarama_siklik:  # Dalış QR tarama sıklığı
                    last_qr_scan = time.time()
                    print(f"DALIŞ ANINDA QR TARAMA: İrtifa={current_alt:.1f}m, Mesafe={distance_to_target:.1f}m")
                    qr_data = self.detect_qr_code(timeout=dalis_tarama_suresi)  # Dalış QR tarama süresi
                    
                    if qr_data:
                        print(f"QR KOD BULUNDU: {qr_data}")
                        qr_found = True
                        
                        # QR tespit edildiğinde hızlı kurtarma - burun yukarı
                        if current_alt < 15.0:  # Düşük irtifada kurtarma
                            print("QR BULUNDU - ACİL KURTARMA MODU - BURUN YUKARI")
                            
                            # Konfigürasyondan kurtarma parametrelerini al
                            kurtarma_params = self.config["gorev_parametreleri"]["kurtarma"]
                            pitch_up = kurtarma_params["pitch_up"]
                            tekrar_sayisi = kurtarma_params["tekrar_sayisi"]
                            throttle = kurtarma_params["throttle"]
                            pitch_bekleme = kurtarma_params["beklemeler"]["pitch_up"]
                            qr_tolerans = kurtarma_params["beklemeler"]["qr_tolerans"]
                            normal_pitch_min = kurtarma_params["normal_deger"]["pitch_min"]
                            normal_trim = kurtarma_params["normal_deger"]["trim"]
                            
                            # QR kod görüldüğünde ek kayıt süresi - kayıt yapılabilsin
                            time.sleep(qr_tolerans)
                            
                            # Burnu hızla yukarı çevir - maksimum burun yukarı komutu
                            for i in range(tekrar_sayisi):  # Konfigürasyondan alınan sayı kadar
                                self.mav.mav.rc_channels_override_send(
                                    self.system_id, self.component_id,
                                    0, pitch_up, 0, 0, 0, 0, 0, 0)  # Pitch up
                                time.sleep(pitch_bekleme)  # Konfigürasyondan alınan bekleme süresi
                            
                            # Tam gaz ve burun yukarı
                            self.mav.mav.rc_channels_override_send(
                                self.system_id, self.component_id,
                                0, pitch_up, throttle, 0, 0, 0, 0, 0)  # Pitch up + tam gaz
                            
                            # Güvenli irtifaya dönüş (waypoint 4)
                            for _ in range(3):  # Garanti olması için 3 kez dene
                                self.mav.mav.command_long_send(
                                    self.system_id, self.component_id,
                                    mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                                    0, 4, 0, 0, 0, 0, 0, 0)
                                time.sleep(0.3)
                                
                            # Parametreleri normale döndür
                            self.mav.mav.param_set_send(
                                self.system_id, self.component_id,
                                b'LIM_PITCH_MIN', normal_pitch_min,  # Normal burun aşağı limit
                                mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                                
                            self.mav.mav.param_set_send(
                                self.system_id, self.component_id,
                                b'TRIM_PITCH_CD', normal_trim,  # Normal trim
                                mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                                
                            time.sleep(0.5)  # Kısa bekle
                            self.mav.mav.rc_channels_override_send(
                                self.system_id, self.component_id,
                                0, 0, 0, 0, 0, 0, 0, 0)  # Kontrolü serbest bırak
            
            # Normal sürekli tarama (dalış değilse)
            elif continuous_qr_scan and time.time() - last_qr_scan > normal_tarama_siklik:
                last_qr_scan = time.time()
                print(f"QR TARAMA: İrtifa={current_alt:.1f}m, Mesafe={distance_to_target:.1f}m")
                qr_data = self.detect_qr_code(timeout=normal_tarama_suresi)
                
                if qr_data:
                    print(f"QR KOD BULUNDU: {qr_data}")
                    qr_found = True
            
            # Duruma göre bilgilendirme
            if active_diving:
                # Ek dalış süresi kontrolü - maksimum dalış süresini aşarsa güvenli irtifaya çık
                dalis_gecen_sure = time.time() - dalis_baslama_zamani
                if dalis_gecen_sure > max_dalis_suresi and last_waypoint == 3 and not dalis_geciktirme_basladi:
                    print(f"MAKSİMUM DALIŞ SÜRESİNE ULAŞILDI: {dalis_gecen_sure:.1f}s > {max_dalis_suresi}s")
                    print("GÜVENLİ İRTİFAYA DÖNÜŞ BAŞLATILIYOR...")
                    dalis_geciktirme_basladi = True
                    
                    # Güvenli irtifaya dönüş (waypoint 4)
                    for _ in range(3):
                        self.mav.mav.command_long_send(
                            self.system_id, self.component_id,
                            mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                            0, 4, 0, 0, 0, 0, 0, 0)
                        time.sleep(0.3)
                
                # Dalış durum bilgisini ekrana yaz
                print(f"DALIŞ DURUMU: İrtifa={current_alt:.1f}m, Mesafe={distance_to_target:.1f}m, Waypoint={last_waypoint}, Süre={dalis_gecen_sure:.1f}s")
            elif last_waypoint == 2:  # eski: 3
                # Yaklaşma durumunu sadece değişikliklerde bildir
                if int(time.time()) % 3 == 0:
                    print(f"YAKLAŞMA: İrtifa={current_alt:.1f}m, Mesafe={distance_to_target:.1f}m")
            
            # Eğer çok yakına geldik ve hala dalış yapmadıysa zorla dalış başlat
            if distance_to_target < zorla_dalis and not dalish_begin and not kurtarma_yapildi:
                print("!!! KRİTİK MESAFE - ZORLA DALIŞ BAŞLATILIYOR (MESAFE: {:.1f}m) !!!".format(distance_to_target))
                print("SEBEBI: ACİL ZORLA DALIŞ MESAFESINE ({:.1f}m) ULAŞILDI".format(zorla_dalis))
                dalish_begin = True
                active_diving = True
                continuous_qr_scan = True  # Sürekli QR tarama modunu aktif et
                dalis_baslama_zamani = time.time()  # Dalış başlama zamanını kaydet
                print("DALIŞ SÜRESİ SAYACI BAŞLATILDI - MİNİMUM: {}s, MAKSİMUM: {}s".format(min_dalis_suresi, max_dalis_suresi))
                
                # Waypoint 3'e atla (kamikaze dalış noktası)
                for _ in range(3):
                    self.mav.mav.command_long_send(
                        self.system_id, self.component_id,
                        mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT,
                        0, 3, 0, 0, 0, 0, 0, 0)
                    time.sleep(0.3)
                    
                # Kamikaze parametrelerini maksimum agresifliğe ayarla
                self.mav.mav.param_set_send(
                    self.system_id, self.component_id,
                    b'TECS_SINK_MAX', 15.0,  # Daha hızlı alçalma için zorla
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                
                self.mav.mav.param_set_send(
                    self.system_id, self.component_id,
                    b'LIM_PITCH_MIN', -30000,  # -300 derece (çok dik)
                    mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                
                self.mav.mav.param_set_send(
                    self.system_id, self.component_id,
                    b'LAND_PITCH_CD', -30000,  # -300 derece (çok dik iniş açısı)
                    mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                    
                self.mav.mav.param_set_send(
                    self.system_id, self.component_id,
                    b'TRIM_PITCH_CD', -2000,  # Negatif pitch trim (burun aşağıda)
                    mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                
                # Arduino'nun dalış parametrelerini hızlı ayarlaması için MAVLINK protokol hızını artır
                self.mav.mav.request_data_stream_send(
                    self.system_id, self.component_id,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)
                
                print("!!! SÜPERAGRESİF ACİL DALIŞ BAŞLADI !!!")
                
                # Pitch kontrolü - aşırı agresif burun aşağı
                pitch_override_value = self.config["pitch_override"]["agresif"]
                tekrar_sayisi = self.config["pitch_override"]["tekrarlama"]["agresif"]
                
                # Aşırı agresif burun aşağı komutlarını tekrarla (daha fazla tekrar)
                for i in range(tekrar_sayisi + 5): # Acil durumda daha fazla tekrarla
                    self.mav.mav.rc_channels_override_send(
                        self.system_id, self.component_id,
                        0, pitch_override_value, 0, 0, 0, 0, 0, 0)
                    time.sleep(0.1) # Daha hızlı tekrarla
                
                # Throttle'ı düşür (motor gücü azalt)
                self.mav.mav.rc_channels_override_send(
                    self.system_id, self.component_id,
                    0, pitch_override_value, 1300, 0, 0, 0, 0, 0) # Düşük throttle
                time.sleep(0.5)
                
                # Kontrol serbest bırak ve aviyoniklerin dalışı ele almasını bekle
                self.mav.mav.rc_channels_override_send(
                    self.system_id, self.component_id,
                    0, 0, 0, 0, 0, 0, 0, 0)
                
                # Kontrol serbest bırak ve aviyoniklerin dalışı ele almasını bekle - İHA artık tek seferde dalacak
            
            time.sleep(0.2)  # Hızlı döngü
        
        # RTL'e dönüş - görevin sonlandığı bilgisi
        print(f"Kamikaze dalış tamamlandı. En düşük irtifa: {min_altitude:.1f}m, En yakın mesafe: {closest_distance:.1f}m")
        if active_diving:
            dalis_gecen_sure = time.time() - dalis_baslama_zamani
            print(f"Toplam dalış süresi: {dalis_gecen_sure:.1f}s")
            
        print("RTL modunda eve dönüş yapılıyor...")
        
        return qr_found
    
    def execute_mission(self):
        """Ana görev akışı"""
        try:
            # 1. İHA durumunu kontrol et
            self.update_vehicle_state()
            
            if self.vehicle_state['is_flying']:
                print("\n*** İHA ZATEN HAVADA DURUMU TESPİT EDİLDİ ***")
                
                # Mevcut mod durumu
                current_mode = self.vehicle_state['mode']
                print(f"Mevcut mod: {current_mode}, İrtifa: {self.vehicle_state['altitude']:.1f}m")
                
                # RTL modundaysa güvenli şekilde görevi yükle
                if current_mode == 'RTL':
                    print("RTL MODUNDA TESPİT EDİLDİ - DOĞRUDAN KAMIKAZE MODU BAŞLATILIYOR!")
                    # Kamikaze için parametreleri ayarla
                    self.set_kamikaze_parameters()
                    
                    # Önce kamikaze misyonunu yükle
                    if not self.create_kamikaze_mission():
                        print("Kamikaze misyonu oluşturulamadı!")
                        return False
                    
                    # AUTO moda geç
                    if self.change_mode('AUTO'):
                        print("AUTO moduna başarıyla geçildi - Kamikaze görevi başlatılıyor!")
                        return self.perform_kamikaze_mission()
                    else:
                        print("AUTO moda geçiş başarısız - İHA kontrol edilemiyor!")
                        return False
                else:
                    print(f"İHA {current_mode} modunda - Kamikaze modu başlatılıyor")
                
                # Mesafeyi hesapla
                lat1, lon1 = self.vehicle_state['lat'], self.vehicle_state['lon']
                lat2, lon2 = self.target_lat, self.target_lon
                
                # Dünya yarıçapı (metre)
                R = 6371000.0
                
                # Radyana çevirme
                phi1 = math.radians(lat1)
                phi2 = math.radians(lat2)
                delta_phi = math.radians(lat2 - lat1)
                delta_lambda = math.radians(lon2 - lon1)
                
                a = math.sin(delta_phi/2.0) * math.sin(delta_phi/2.0) + \
                    math.cos(phi1) * math.cos(phi2) * \
                    math.sin(delta_lambda/2.0) * math.sin(delta_lambda/2.0)
                    
                c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
                distance_to_target = R * c  # metre cinsinden mesafe
                
                print(f"Hedefe mesafe: {distance_to_target:.1f}m (GLOBAL)")
                
                # Direkt kamikaze dalış misyonu başlat
                print("KAMIKAZE GÖREV BAŞLATILIYOR!")
                success = self.perform_kamikaze_mission()
                print(f"Kamikaze görevi {'başarılı' if success else 'başarısız'}")
                return success
            else:
                print("\n*** İHA YERDE - KALKIŞ PROSEDÜRÜ ***")
                
                # Mode MANUAL'e geç
                if not self.change_mode('MANUAL'):
                    print("MANUAL moda geçilemedi")
                    return False
                
                # ARM et
                if not self.arm_vehicle():
                    print("ARM başarısız")
                    return False
                
                # Kamikaze parametrelerini hemen ayarla
                self.set_kamikaze_parameters()
                
                # Kamikaze misyonunu yükle
                if not self.create_kamikaze_mission():
                    print("Kamikaze misyonu oluşturulamadı")
                    return False
                
                # Kalkış için manuel yardım
                print("Motor gücü artırılıyor...")
                for throttle in self.config["kalkis_parametreleri"]["throttle_degerleri"]:
                    self.mav.mav.rc_channels_override_send(
                        self.system_id, self.component_id,
                        0, 0, throttle, 0, 0, 0, 0, 0)  # Kanal 3: Throttle
                    time.sleep(1)
                    print(f"Throttle: {throttle}")
                
                # FBWA moduna geç
                if self.change_mode('FBWA'):
                    print("FBWA moduna geçildi - Kalkış için burun yukarı dikiliyor")
                    
                    # Yunus hareketi parametrelerini al
                    yunus_params = self.config["kalkis_parametreleri"]["yunus_hareketi"]
                    yunus_suresi = yunus_params["sure"]
                    pitch_up = yunus_params["pitch_up"]
                    throttle_kalkis = self.config["kalkis_parametreleri"]["throttle_kalkis"]
                    
                    print(f"Burun yukarı tutma işlemi başlatılıyor (Süre: {yunus_suresi}s)")
                    yunus_baslangic = time.time()
                    
                    # Burun yukarı ve tam gaz
                    print("Burun YUKARI dikiliyor")
                    self.mav.mav.rc_channels_override_send(
                        self.system_id, self.component_id,
                        0, pitch_up, throttle_kalkis, 0, 0, 0, 0, 0)  # Pitch up + tam gaz
                    
                    # Belirtilen süre boyunca burnunu yukarıda tut
                    while time.time() - yunus_baslangic < yunus_suresi:
                        # İHA durumunu kontrol et
                        self.update_vehicle_state(verbose=False)
                        print(f"İHA İrtifa: {self.vehicle_state['altitude']:.1f}m, Hız: {self.vehicle_state['airspeed']:.1f}m/s")
                        
                        # Burnu tekrar yukarı doğrult (her saniye tekrar gönder)
                        if int(time.time()) % 2 == 0:  # Her 2 saniyede bir
                            self.mav.mav.rc_channels_override_send(
                                self.system_id, self.component_id,
                                0, pitch_up, throttle_kalkis, 0, 0, 0, 0, 0)  # Pitch up tekrarla
                            
                        # Eğer İHA havadaysa ve yeterli irtifaya ulaştıysa, döngüden çık
                        if self.vehicle_state['is_flying'] and self.vehicle_state['altitude'] > 20:
                            print("İHA havada ve yeterli irtifaya ulaştı!")
                            break
                            
                        time.sleep(0.5)  # Yarım saniyede bir kontrol et
                    
                    print(f"Burun yukarı tutma işlemi tamamlandı (Süre: {time.time() - yunus_baslangic:.1f}s)")
                
                # AUTO moduna geç
                if self.change_mode('AUTO'):
                    print("AUTO moduna geçildi - Misyon başlatıldı")
                    
                    # RC override'ı temizle
                    self.mav.mav.rc_channels_override_send(
                        self.system_id, self.component_id,
                        0, 0, 0, 0, 0, 0, 0, 0)
                    
                    print("İHA kamikaze görevi için hazır, misyon devam ediyor...")
                    
                    # İHA'nın havaya kalktığını kontrol et
                    start_time = time.time()
                    while time.time() - start_time < 60:  # 1 dakika bekle
                        self.update_vehicle_state(verbose=False)
                        
                        if self.vehicle_state['is_flying']:
                            print(f"İHA havada! İrtifa: {self.vehicle_state['altitude']:.1f}m")
                            print("Kamikaze dalış görevi devam ediyor...")
                            # Kalkış sonrası beklemeye gerek yok, misyon zaten devam ediyor
                            # Sadece takip et
                            return self.perform_kamikaze_mission()
                        
                        time.sleep(2)
                    
                    print("Kalkış başarısız, işlem iptal edildi")
                    return False
                else:
                    print("AUTO moda geçilemedi, işlem iptal edildi")
                    return False
        
        except Exception as e:
            print(f"Kritik hata: {str(e)}")
            
            # Acil durum: İHA havadaysa RTL moduna geç
            if self.vehicle_state['is_flying']:
                print("ACİL DURUM: RTL MODUNA GEÇİLİYOR!")
                self.change_mode('RTL')
            
            sys.exit(1)

if __name__ == "__main__":
    # Argümanları kontrol et
    connection_string = None
    config_file = "config.py"
    
    # Argümanları parse et
    i = 1
    while i < len(sys.argv):
        if sys.argv[i] == "--connection" and i + 1 < len(sys.argv):
            connection_string = sys.argv[i + 1]
            i += 2
        elif sys.argv[i] == "--config" and i + 1 < len(sys.argv):
            config_file = sys.argv[i + 1]
            i += 2
    else:
            i += 1
    
    print(f"Konfigürasyon dosyası: {config_file}")
    if connection_string:
        print(f"Bağlantı: {connection_string}")
    
    controller = KamikazeUAVController(config_file, connection_string)
    controller.execute_mission()

