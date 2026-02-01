"""
Görev yönetimi ve durum makinesi mantığını içeren sınıf.
"""
import cv2
import time
import math
import numpy as np
from enum import Enum, auto
from perception.detector import Detector
from planning.planner import Planner
from control.controller import Controller

class MissionState(Enum):
    """Görev durumlarını tanımlayan enum sınıfı."""
    IDLE = auto()
    APPROACH = auto()    # Yaklaşma durumu
    DIVE = auto()        # Dalış durumu
    RECOVERY = auto()    # Yükselme durumu
    COMPLETE = auto()    # Tamamlanma durumu

class MissionFSM:
    def __init__(self, mavlink_connection, model_path, camera_id=0):
        """
        MissionFSM sınıfını başlatır.
        
        Args:
            mavlink_connection (MavlinkConnection): MAVLink bağlantı nesnesi
            model_path (str): YOLO model dosyasının yolu
            camera_id (int): Kamera ID'si
        """
        self.state = MissionState.IDLE
        self.detector = Detector(model_path)
        self.planner = Planner(mavlink_connection)
        self.controller = Controller(mavlink_connection)
        self.camera = cv2.VideoCapture(camera_id)
        
        # Kamikaze görevi parametreleri
        self.approach_angle = 30.0    # Yaklaşma açısı
        self.min_altitude = 5.0       # En düşük yükseklik
        self.dive_distance = 30.0     # 30° dalış mesafesi
        self.cruise_altitude = 50.0   # Seyir irtifası
        
        # Gelişmiş dalış parametreleri
        self.max_g_force = 3.0        # Maksimum G kuvveti
        self.safe_approach_distance = 2.0  # Güvenli yaklaşma mesafesi (m)
        self.max_dive_angle = 60.0    # Maksimum dalış açısı (derece)
        self.terminal_phase_altitude = 10.0  # Terminal fazın başlangıç irtifası
        
        # QR takip parametreleri
        self.last_qr_detection = None  # Son QR tespiti
        self.qr_detection_count = 0    # QR tespit sayısı
        self.min_detection_count = 3   # Minimum tespit sayısı
        
    def run(self):
        """Ana görev döngüsünü çalıştırır."""
        print("Kamikaze görevi başlatıldı.")
        
        # QR Kilitlenme Algoritmasını Başlat
        while True:
            # Kamera görüntüsünü al
            ret, frame = self.camera.read()
            if not ret:
                print("Kamera görüntüsü alınamadı!")
                continue
                
            # Duruma göre işlem yap
            if self.state == MissionState.IDLE:
                self._handle_idle_state()
                cv2.putText(frame, "DURUM: HAZIR", (10, 30), 
                         cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            elif self.state == MissionState.APPROACH:
                self._handle_approach_state(frame)
                cv2.putText(frame, "DURUM: YAKLAŞMA", (10, 30), 
                         cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            elif self.state == MissionState.DIVE:
                # Formül tabanlı dalış algoritmalarını kullan
                self._handle_dive_state_improved(frame)
                
                # Dinamik dalış manevrası - formül tabanlı
                dive_success = self._execute_formula_based_dive(frame)
                
                # Durum bilgisini göster
                cv2.putText(frame, "DURUM: DALIŞ", (10, 30), 
                         cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Dalış açısı ve hızı göster
                current_alt = self.controller.get_altitude()
                current_speed = self.controller.mavlink.get_groundspeed()
                
                cv2.putText(frame, f"Yükseklik: {current_alt:.1f}m", (10, 60), 
                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(frame, f"Hız: {current_speed:.1f}m/s", (10, 90), 
                         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Terminal faza yaklaşıldığında uyarı
                if current_alt <= self.terminal_phase_altitude + 5:
                    cv2.putText(frame, "TERMINAL FAZ!", (frame.shape[1]//2 - 100, frame.shape[0]//2), 
                             cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            elif self.state == MissionState.RECOVERY:
                self._handle_recovery_state()
                cv2.putText(frame, "DURUM: KURTARMA", (10, 30), 
                         cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            elif self.state == MissionState.COMPLETE:
                print("Görev tamamlandı.")
                cv2.putText(frame, "DURUM: TAMAMLANDI", (10, 30), 
                         cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.waitKey(2000)  # 2 saniye bekle
                break
                
            # Kamera görüntüsüne ek bilgiler ekle
            # Frame sayısı, zaman ve durum bilgisi
            current_time = cv2.getTickCount() / cv2.getTickFrequency()
            cv2.putText(frame, f"Zaman: {current_time:.1f}s", (10, frame.shape[0] - 10), 
                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Görüntü merkezini belirt
            height, width = frame.shape[:2]
            cv2.circle(frame, (width//2, height//2), 10, (0, 0, 255), 2)
            cv2.line(frame, (width//2 - 20, height//2), (width//2 + 20, height//2), (0, 0, 255), 2)
            cv2.line(frame, (width//2, height//2 - 20), (width//2, height//2 + 20), (0, 0, 255), 2)
            
            # Kamera parametrelerini göster
            camera_fov = 60.0  # Kamera FOV değeri
            cv2.putText(frame, f"FOV: {camera_fov}°", (width - 120, 30), 
                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Formül parametrelerini göster (yalnızca DIVE durumunda)
            if self.state == MissionState.DIVE:
                # Kamera parametreleri
                camera_params = {
                    'focal_length': 3.6,  # mm
                    'fov': 60.0,          # derece
                    'sensor_width': 4.8,  # mm
                    'qr_size': 0.1        # 10 cm QR kod
                }
                
                # Footprint hesapla (yerden görülen alan)
                # F = 2 * h * tan(θ/2)
                fov_rad = np.radians(camera_params['fov'])
                footprint = 2 * current_alt * np.tan(fov_rad / 2)
                
                # Piksel/metre oranı
                # Piksel/metre = resolution_width / footprint
                pixel_per_meter = width / footprint
                
                # Minimum tespit mesafesi
                # d_min = (qr_width * focal_length) / min_pixel_width
                min_pixel_width = 30
                min_distance = (camera_params['qr_size'] * camera_params['focal_length']) / min_pixel_width
                
                # Maksimum tespit mesafesi
                # d = (2 * tan(θ/2) * min_pixel_width * qr_width) / resolution_width
                max_distance = (2 * np.tan(fov_rad / 2) * min_pixel_width * camera_params['qr_size']) / width
                
                # Dalış başlangıç mesafesi
                # d_start = 3 * d_min
                dive_start_distance = 3 * min_distance
                
                # Parametre bilgilerini göster
                cv2.putText(frame, f"Footprint: {footprint:.1f}m", (width - 180, 50), 
                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(frame, f"Min Dist: {min_distance:.2f}m", (width - 180, 70), 
                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(frame, f"Max Dist: {max_distance:.2f}m", (width - 180, 90), 
                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(frame, f"Dive Start: {dive_start_distance:.2f}m", (width - 180, 110), 
                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
            # Görüntüyü göster
            cv2.imshow('Mission View', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
        self.camera.release()
        cv2.destroyAllWindows()
        
    def _handle_idle_state(self):
        """Başlangıç durumunu işler."""
        # Yaklaşma düzlemine giriş kontrolü
        if self._check_approach_parameters():
            print("Yaklaşma düzlemine girildi.")
            self.state = MissionState.APPROACH
            
    def _handle_approach_state(self, frame):
        """Yaklaşma durumunu işler."""
        # Yaklaşma parametrelerini kontrol et
        if not self._maintain_approach_parameters():
            return
            
        # 30° dalış mesafesine ulaşıldı mı?
        if self._check_dive_distance():
            print("Dalış mesafesine ulaşıldı.")
            self.state = MissionState.DIVE
            
    def _handle_dive_state(self, frame):
        """Dalış durumunu işler."""
        # QR kod tespiti
        qr_data = self.detector.detect_qr(frame)
        
        if qr_data:
            print("QR kod okundu:", qr_data)
            # İletişim sistemini bilgilendir
            self._notify_communication_system(qr_data)
            self.state = MissionState.RECOVERY
    
    def _handle_dive_state_improved(self, frame):
        """Gelişmiş dalış durumunu işler."""
        # Gelişmiş QR kod tespiti ve takibi
        qr_result = self.detector.track_and_read_qr(frame, self.last_qr_detection)
        
        # Sonuçları işle
        if qr_result and qr_result['success']:
            self.last_qr_detection = qr_result['qr_data'][0] if qr_result['qr_data'] else None
            self.qr_detection_count += 1
            
            # QR kodu çerçeve üzerine çiz
            if self.last_qr_detection and 'rect' in self.last_qr_detection:
                x, y, w, h = self.last_qr_detection['rect']
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                
                # QR içeriğini göster
                if 'data' in self.last_qr_detection:
                    data = self.last_qr_detection['data']
                    cv2.putText(frame, f"QR: {data}", (x, y - 10), 
                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Yeterli sayıda tespit olduysa, veriyi işle
            if self.qr_detection_count >= self.min_detection_count:
                print(f"QR kod doğrulandı: {self.last_qr_detection['data']}")
                
                # İletişim sistemini bilgilendir
                self._notify_communication_system(self.last_qr_detection)
                
                # Kurtarma durumuna geç
                self.state = MissionState.RECOVERY
        else:
            # QR tespit edilmediyse, terminal faz kontrolü
            current_alt = self.controller.get_altitude()
            
            # Terminal fazda mıyız?
            if current_alt <= self.terminal_phase_altitude:
                # QR tespit edilemedi, kurtarma durumuna geç
                print("Terminal fazda QR tespit edilemedi, kurtarma durumuna geçiliyor...")
                self.state = MissionState.RECOVERY
    
    def _handle_recovery_state(self):
        """Yükselme durumunu işler."""
        # Yükselme algoritmasını başlat
        if self._execute_recovery():
            # Uçuş yüksekliğine ulaşıldı mı?
            if self._check_cruise_altitude():
                print("Seyir irtifasına ulaşıldı.")
                self.state = MissionState.COMPLETE
                
    def _check_approach_parameters(self):
        """Yaklaşma parametrelerini kontrol eder."""
        current_alt = self.controller.get_altitude()
        current_angle = self.controller.get_approach_angle()
        
        return (current_alt >= self.min_altitude and 
                abs(current_angle - self.approach_angle) < 5.0)
                
    def _maintain_approach_parameters(self):
        """Yaklaşma parametrelerini korur."""
        current_alt = self.controller.get_altitude()
        if current_alt < self.min_altitude:
            self.controller.adjust_altitude(self.min_altitude)
            return False
            
        current_angle = self.controller.get_approach_angle()
        if abs(current_angle - self.approach_angle) > 5.0:
            self.controller.adjust_angle(self.approach_angle)
            return False
            
        return True
        
    def _check_dive_distance(self):
        """Dalış mesafesini kontrol eder."""
        current_distance = self.controller.get_distance_to_target()
        return current_distance <= self.dive_distance
        
    def _notify_communication_system(self, qr_data):
        """İletişim sistemini bilgilendirir."""
        # QR kod verisini işle ve gönder
        self.controller.send_mission_data(qr_data)
        
    def _execute_recovery(self):
        """Yükselme manevrasını gerçekleştirir."""
        current_alt = self.controller.get_altitude()
        if current_alt < self.cruise_altitude:
            self.controller.adjust_altitude(self.cruise_altitude)
            return False
        return True
        
    def _check_cruise_altitude(self):
        """Seyir irtifasını kontrol eder."""
        current_alt = self.controller.get_altitude()
        return abs(current_alt - self.cruise_altitude) < 2.0  # 2 metre tolerans 

    def _execute_formula_based_dive(self, frame):
        """
        Kamera ve QR formüllerini kullanarak optimum dalış manevrası gerçekleştirir.
        Bu metot dalış ve QR kod tespiti için verilen formülleri kullanır.
        
        Args:
            frame: Kamera görüntüsü
            
        Returns:
            bool: Dalış başarıyla tamamlandı mı?
        """
        # Pozisyon bilgilerini güncelle
        self.planner.update_position()
        
        # Kamera parametreleri
        camera_params = {
            'focal_length': 3.6,      # mm
            'fov': 60.0,              # derece
            'sensor_width': 4.8,      # mm (1/3" sensör)
            'qr_size': 0.1,           # 10 cm QR kodu
            'resolution': frame.shape[1::-1]  # (width, height)
        }
        
        # Gelişmiş QR kod tespiti (formülleri kullanan)
        qr_result = self.detector.advanced_qr_detection_using_formulas(frame, camera_params)
        
        # QR kod tespiti başarılı mı?
        if qr_result and qr_result['success'] and 'best_qr' in qr_result:
            best_qr = qr_result['best_qr']
            
            # QR bilgilerini çerçeve üzerine çiz
            if 'rect' in best_qr:
                rect = best_qr['rect']
                x, y, w, h = rect
                
                # QR kodunu işaretle
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # QR merkezi
                center_x, center_y = best_qr['center']
                cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                
                # QR bilgilerini göster
                estimated_distance = best_qr.get('estimated_distance', 0)
                qr_data = best_qr.get('data', 'Unknown')
                
                cv2.putText(frame, f"QR: {qr_data}", (x, y - 30), 
                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f"Dist: {estimated_distance:.2f}m", (x, y - 10), 
                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # QR kodun tahmini 3D pozisyonu
            current_pos = self.planner.current_position
            norm_center_x, norm_center_y = best_qr['normalized_center']
            estimated_distance = best_qr['estimated_distance']
            
            # QR kodun dünya koordinatlarını hesapla
            # Bu hesaplama basitleştirilmiş bir yaklaşımdır, gerçek senaryolarda
            # kamera kalibrasyonu ve geometrik dönüşümler kullanılmalıdır
            qr_world_x = current_pos[0] + norm_center_x * estimated_distance * 0.5
            qr_world_y = current_pos[1] + norm_center_y * estimated_distance * 0.5
            qr_world_z = current_pos[2] - estimated_distance * 0.8  # QR'ın altımızda olduğunu varsay
            
            qr_position = (qr_world_x, qr_world_y, qr_world_z)
            
            # Gelişmiş yaklaşma stratejisi hesapla
            approach_strategy = self.planner.calculate_advanced_qr_approach(qr_position, camera_params)
            
            if approach_strategy:
                # Yaklaşma stratejisi bilgilerini göster
                min_dist = approach_strategy.get('min_detection_distance', 0)
                max_dist = approach_strategy.get('max_detection_distance', 0)
                current_dist = approach_strategy.get('current_distance', 0)
                action = approach_strategy.get('action', 'unknown')
                
                # Bilgileri ekrana yaz
                info_text = f"Min: {min_dist:.2f}m, Max: {max_dist:.2f}m, Curr: {current_dist:.2f}m"
                cv2.putText(frame, info_text, (10, frame.shape[0] - 30), 
                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(frame, f"Action: {action.upper()}", (10, frame.shape[0] - 10), 
                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Eyleme göre hareket et
                if action == 'approach':
                    # Yaklaşma (hedef çok uzakta)
                    if 'approach_point' in approach_strategy:
                        approach_point = approach_strategy['approach_point']
                        
                        # Optimum hız hesapla (Mesafeye göre dinamik)
                        optimal_speed = min(5.0, current_dist * 0.3)  # Mesafenin %30'u kadar hız
                        
                        # Yaklaşma komutunu gönder
                        self.controller.move_to_position(
                            approach_point[0], 
                            approach_point[1], 
                            approach_point[2], 
                            max_g_force=2.0  # Daha yumuşak bir yaklaşma için
                        )
                        
                        print(f"QR'a yaklaşılıyor: Mesafe={current_dist:.2f}m, Hız={optimal_speed:.2f}m/s")
                        return True
                        
                elif action == 'maintain':
                    # İdeal mesafede, dalış için hazırlan
                    target_altitude = approach_strategy.get('target_altitude', current_pos[2])
                    
                    # İdeal irtifaya ulaş
                    if abs(current_pos[2] - target_altitude) > 2.0:
                        # İrtifayı ayarla
                        self.controller.move_to_position(
                            current_pos[0], 
                            current_pos[1], 
                            target_altitude
                        )
                        print(f"İdeal irtifaya ayarlanıyor: {target_altitude:.2f}m")
                    else:
                        # Dalışa başla
                        dive_start_distance = approach_strategy.get('dive_start_distance', current_dist * 1.5)
                        
                        # QR'ın hesaplanan konumunu kullanarak dalış parametrelerini hesapla
                        # v^2 = 2*a*s formülünü kullanarak optimum hızı bul
                        max_g_force = 2.5  # Daha kontrollü bir dalış için
                        max_acceleration = max_g_force * 9.81
                        
                        # Dalış mesafesi
                        dive_distance = current_dist * 0.9  # %90'ı kadar mesafeye kadar yaklaş
                        
                        # Optimum dalış hızı
                        max_speed = min(10.0, np.sqrt(2 * max_acceleration * dive_distance * 0.5))
                        
                        # Dalış açısı
                        vertical_dist = current_pos[2] - qr_position[2]
                        horizontal_dist = np.sqrt((qr_position[0] - current_pos[0])**2 + 
                                             (qr_position[1] - current_pos[1])**2)
                                             
                        # Güvenli dalış açısı (15° - 60° arasında sınırlandır)
                        dive_angle = np.degrees(np.arctan2(vertical_dist, horizontal_dist))
                        dive_angle = max(15.0, min(60.0, dive_angle))
                        
                        # Dalış komutunu gönder
                        dive_params = self.controller.execute_dive_maneuver(
                            qr_position,  # QR pozisyonu
                            max_speed,    # Optimum hız
                            max_g_force   # G-kuvveti sınırı
                        )
                        
                        print(f"Dalış başlatıldı: Açı={dive_angle:.1f}°, "
                             f"Hız={max_speed:.1f}m/s, Mesafe={dive_distance:.1f}m")
                        
                        # QR kod verisi alındı, kurtarma durumuna geç
                        if 'data' in best_qr:
                            self._notify_communication_system(best_qr)
                            
                            # Belirli bir süre sonra kurtarma fazına geç
                            if self.qr_detection_count > self.min_detection_count:
                                print(f"QR kodu doğrulandı: {best_qr['data']}")
                                self.state = MissionState.RECOVERY
                            else:
                                self.qr_detection_count += 1
                                
                        return True
                        
                elif action == 'move_back':
                    # Çok yakınsın, geri çekil
                    target_distance = approach_strategy.get('target_distance', min_dist * 1.2)
                    
                    # QR kodundan güvenli bir mesafeye geri çekil
                    # Mevcut pozisyon vektörü ile QR pozisyon vektörü arasındaki farkı normalleştir
                    direction_vector = [
                        current_pos[0] - qr_position[0],
                        current_pos[1] - qr_position[1],
                        current_pos[2] - qr_position[2]
                    ]
                    
                    # Vektörü normalleştir
                    vector_length = np.sqrt(sum(x**2 for x in direction_vector))
                    
                    if vector_length > 0:
                        normalized_vector = [x / vector_length for x in direction_vector]
                        
                        # Hedef pozisyonu hesapla
                        target_position = [
                            qr_position[0] + normalized_vector[0] * target_distance,
                            qr_position[1] + normalized_vector[1] * target_distance,
                            qr_position[2] + normalized_vector[2] * target_distance
                        ]
                        
                        # Geri çekilme komutunu gönder
                        self.controller.move_to_position(
                            target_position[0],
                            target_position[1],
                            target_position[2],
                            max_g_force=1.5  # Yumuşak bir geri çekilme
                        )
                        
                        print(f"QR'dan geri çekiliniyor: Hedef mesafe={target_distance:.2f}m")
                        return True
            
            # Son QR tespitini güncelle
            self.last_qr_detection = best_qr
            return True
            
        # QR tespit edilemedi
        return False 