"""
Görev planlaması ve hedef belirleme işlemlerini yapan sınıf.
"""
import numpy as np
from utils.mavlink_utils import MavlinkConnection

class Planner:
    def __init__(self, mavlink_connection):
        """
        Planner sınıfını başlatır.
        
        Args:
            mavlink_connection (MavlinkConnection): MAVLink bağlantı nesnesi
        """
        self.mavlink = mavlink_connection
        self.current_position = None
        self.target_position = None
        
    def update_position(self):
        """Güncel pozisyon bilgisini alır."""
        self.current_position = self.mavlink.get_position()
        
    def set_target(self, target_coords):
        """
        Hedef koordinatlarını belirler.
        
        Args:
            target_coords (tuple): Hedef koordinatları (x, y, z)
        """
        self.target_position = target_coords
        
    def calculate_path(self):
        """
        Mevcut pozisyondan hedefe giden yolu hesaplar.
        
        Returns:
            tuple: Hedef pozisyon (x, y, z)
        """
        if self.current_position is None or self.target_position is None:
            return None
            
        # Basit bir doğrusal yol hesaplama
        return self.target_position
        
    def calculate_dive_point(self, qr_position, dive_angle=45):
        """
        Dalış noktasını hesaplar.
        
        Args:
            qr_position (tuple): QR kodun pozisyonu (x, y, z)
            dive_angle (float): Dalış açısı (derece)
            
        Returns:
            tuple: Dalış noktası koordinatları (x, y, z)
        """
        if self.current_position is None:
            return None
            
        # Dalış noktasını hesapla
        current_x, current_y, current_z = self.current_position
        qr_x, qr_y, qr_z = qr_position
        
        # Dalış mesafesini ve açısını hesapla
        distance = np.sqrt((qr_x - current_x)**2 + (qr_y - current_y)**2)
        height = distance * np.tan(np.radians(dive_angle))
        
        # Dalış noktası koordinatları
        dive_x = qr_x
        dive_y = qr_y
        dive_z = current_z + height
        
        return (dive_x, dive_y, dive_z)
    
    def calculate_advanced_dive_trajectory(self, qr_position, current_speed, max_g_force=3.0, 
                                         min_altitude=5.0, max_dive_angle=60):
        """
        Gelişmiş dalış yörüngesi hesaplar.
        
        Args:
            qr_position (tuple): QR kodun pozisyonu (x, y, z)
            current_speed (float): Mevcut hız (m/s)
            max_g_force (float): Maksimum G kuvveti
            min_altitude (float): Minimum güvenli irtifa (m)
            max_dive_angle (float): Maksimum dalış açısı (derece)
            
        Returns:
            dict: Dalış yörüngesi parametreleri
        """
        if self.current_position is None:
            return None
            
        # Mevcut pozisyon ve hedef
        current_x, current_y, current_z = self.current_position
        qr_x, qr_y, qr_z = qr_position
        
        # Yatay ve dikey mesafeler
        horizontal_dist = np.sqrt((qr_x - current_x)**2 + (qr_y - current_y)**2)
        
        # Kinematik hesaplamalar için sabitler
        g = 9.81  # Yerçekimi (m/s^2)
        max_acceleration = max_g_force * g
        
        # Güvenli mesafe - irtifa kontrolü
        safe_altitude = max(min_altitude, qr_z + 2.0)  # QR kodun en az 2m üstünde ol
        
        # Dalışın başlangıcı için dikey mesafe (yükseklik farkı)
        vertical_dist = current_z - safe_altitude
        
        # Eğer hedefe yeterince yakın veya altındaysak, minimum güvenli irtifaya çık
        if current_z <= safe_altitude:
            return {
                'action': 'climb',
                'target_altitude': safe_altitude + 5,  # 5m ekstra marj
                'reason': 'Hedefin altında veya çok yakın irtifadayız'
            }
        
        # Optimum dalış açısını hesapla (dikey ve yatay mesafe oranına göre)
        # Fakat max_dive_angle ile sınırla
        dive_angle_rad = np.arctan2(vertical_dist, horizontal_dist)
        dive_angle = np.degrees(dive_angle_rad)
        dive_angle = min(dive_angle, max_dive_angle)  # Maksimum dalış açısı sınırlaması
        
        # Toplam dalış mesafesi
        dive_distance = np.sqrt(horizontal_dist**2 + vertical_dist**2)
        
        # Maksimum güvenli hız - duruş mesafesi hesabı
        # v^2 = 2as formülünden yola çıkarak
        stopping_distance = current_speed**2 / (2 * max_acceleration)
        
        # Maksimum güvenli hız
        # Duruş mesafesi toplam mesafenin yarısını geçmesin
        max_speed = np.sqrt(max_acceleration * dive_distance * 0.5)
        
        # Hız vektörü bileşenleri
        speed_x = max_speed * np.cos(dive_angle_rad)
        speed_z = max_speed * np.sin(dive_angle_rad)
        
        # Hedef yaklaşım noktası - QR kodun üzerinde
        approach_point = (qr_x, qr_y, safe_altitude)
        
        # Dalış yörüngesi parametreleri
        result = {
            'action': 'dive',
            'dive_angle': dive_angle,
            'max_speed': max_speed,
            'approach_point': approach_point,
            'horizontal_distance': horizontal_dist,
            'vertical_distance': vertical_dist,
            'total_distance': dive_distance,
            'stopping_distance': stopping_distance,
            'safe_altitude': safe_altitude,
            'speed_vector': (speed_x, 0, speed_z)  # X, Y, Z hız bileşenleri
        }
        
        return result
        
    def calculate_qr_approach_strategy(self, qr_center_point, camera_fov=60, min_distance=2.0):
        """
        QR koda optimum yaklaşma stratejisini hesaplar.
        
        Args:
            qr_center_point (tuple): QR merkez noktası (x, y, z)
            camera_fov (float): Kamera görüş açısı (derece)
            min_distance (float): Minimum güvenli mesafe (m)
            
        Returns:
            dict: Yaklaşma stratejisi parametreleri
        """
        if self.current_position is None:
            return None
            
        # Mevcut pozisyon
        current_x, current_y, current_z = self.current_position
        qr_x, qr_y, qr_z = qr_center_point
        
        # QR koda olan mesafe
        distance_to_qr = np.sqrt((qr_x - current_x)**2 + 
                               (qr_y - current_y)**2 + 
                               (qr_z - current_z)**2)
        
        # QR boyutuna bağlı optimal okuma mesafesi 
        # (QR kodun yaklaşık 20-30% FOV kapladığı mesafe ideal)
        # QR kodun yaklaşık 10 cm olduğunu varsayalım
        qr_size = 0.1  # metre
        
        # QR kodun kamera FOV'unun %25'ini kaplaması için gereken mesafe
        # FOV = 2 * arctan(size/(2*distance))
        optimal_distance = qr_size / (2 * np.tan(np.radians(camera_fov * 0.25 / 2)))
        
        # Güvenli mesafe ile karşılaştır
        approach_distance = max(optimal_distance, min_distance)
        
        # QR'ın üzerinde durma noktası
        hover_x = qr_x
        hover_y = qr_y
        hover_z = qr_z + approach_distance  # QR'ın üzerinde
        
        # Yaklaşma açısı (dikeye yakın, örn. 75 derece)
        approach_angle = 75  # derece
        
        # Yaklaşma hızı (yavaş, kontrollü)
        approach_speed = 1.0  # m/s
        
        return {
            'optimal_distance': approach_distance,
            'current_distance': distance_to_qr,
            'hover_point': (hover_x, hover_y, hover_z),
            'approach_angle': approach_angle,
            'approach_speed': approach_speed
        }
    
    def calculate_advanced_qr_approach(self, qr_position, camera_params=None):
        """
        QR koda yaklaşma için gelişmiş hesaplamalar yapar. Kamera parametrelerini
        ve QR formüllerini kullanarak optimum yaklaşma stratejisini belirler.
        
        Args:
            qr_position (tuple): QR merkez noktası (x, y, z)
            camera_params (dict): Kamera parametreleri
                - focal_length: Odak uzaklığı (mm)
                - fov: Görüş açısı (derece)
                - resolution: Çözünürlük (width, height)
                - qr_size: QR kod boyutu (metre)
            
        Returns:
            dict: Gelişmiş yaklaşma stratejisi
        """
        if self.current_position is None:
            return None
            
        # Kamera parametreleri
        if camera_params is None:
            camera_params = {
                'focal_length': 3.6,  # mm
                'fov': 60,           # derece
                'resolution': (1920, 1080),
                'qr_size': 0.1       # 10 cm QR kod
            }
            
        # Mevcut pozisyon ve mesafe
        current_x, current_y, current_z = self.current_position
        qr_x, qr_y, qr_z = qr_position
        
        # QR koda olan yatay ve dikey mesafe
        horizontal_dist = np.sqrt((qr_x - current_x)**2 + (qr_y - current_y)**2)
        vertical_dist = current_z - qr_z
        total_dist = np.sqrt(horizontal_dist**2 + vertical_dist**2)
        
        # Kamera parametrelerini ayıkla
        focal_length = camera_params['focal_length']
        fov = camera_params['fov']
        resolution_width, resolution_height = camera_params['resolution']
        qr_size = camera_params['qr_size']
        
        # Minimum tespit mesafesi (QR kodu en az 30 piksel genişliğinde olmalı)
        min_pixel_width = 30
        min_distance = (qr_size * focal_length) / min_pixel_width
        
        # Maksimum tespit mesafesi 
        # d = (2 * tan(θ/2) * min_pixel_width * qr_width) / resolution_width
        fov_rad = np.radians(fov)
        max_distance = (2 * np.tan(fov_rad / 2) * min_pixel_width * qr_size) / resolution_width
        
        # Dalış başlangıç mesafesi
        # d_start = 3 * d_min
        dive_start_distance = 3 * min_distance
        
        # İdeal görüntüleme yüksekliği - QR kodun FOV'un %25'ini kapladığı durumda
        # h = F / (2 * tan(θ/2)) where F = qr_size / 0.25
        ideal_footprint = qr_size / 0.25  # QR kod FOV'un %25'i
        ideal_altitude = ideal_footprint / (2 * np.tan(fov_rad / 2))
        
        # QR kodun ekrandaki piksel genişliği
        # w_screen = (focal_length * qr_width) / distance
        qr_screen_width = (focal_length * qr_size) / total_dist
        
        # Dinamik eşik değeri (güvenli mesafede QR piksel boyutu)
        # w_threshold = (safe_distance * qr_width) / focal_length
        safe_distance = min_distance * 1.5  # Güvenli mesafe
        dynamic_threshold = (safe_distance * qr_size) / focal_length
        
        # Yerden görülen alan (footprint)
        # F = 2 * h * tan(θ/2)
        current_footprint = 2 * current_z * np.tan(fov_rad / 2)
        
        # Piksel/metre oranı
        # Piksel/metre = resolution_width / footprint
        pixel_per_meter = resolution_width / current_footprint
        
        # Yaklaşma stratejisi
        result = {
            'min_detection_distance': min_distance,
            'max_detection_distance': max_distance,
            'dive_start_distance': dive_start_distance,
            'ideal_altitude': ideal_altitude,
            'current_distance': total_dist,
            'horizontal_distance': horizontal_dist,
            'vertical_distance': vertical_dist,
            'qr_screen_width_px': qr_screen_width,
            'dynamic_threshold_px': dynamic_threshold,
            'current_footprint': current_footprint,
            'pixel_per_meter': pixel_per_meter,
            'is_in_detection_range': min_distance <= total_dist <= max_distance,
        }
        
        # Eylem tavsiyesi
        if total_dist < min_distance:
            result['action'] = 'move_back'
            result['target_distance'] = min_distance * 1.2
        elif total_dist > max_distance:
            result['action'] = 'approach'
            result['target_distance'] = max_distance * 0.8
        else:
            result['action'] = 'maintain'
            result['target_altitude'] = ideal_altitude
        
        # Dalış yörüngesi için önerilen nokta
        if result['action'] != 'move_back':
            approach_angle = np.degrees(np.arctan2(vertical_dist, horizontal_dist))
            # Yeni yaklaşma noktası
            approach_x = qr_x
            approach_y = qr_y
            approach_z = qr_z + ideal_altitude
            
            result['approach_point'] = (approach_x, approach_y, approach_z)
            result['approach_angle'] = approach_angle
        
        return result
        
    def check_altitude(self, min_altitude):
        """
        Minimum irtifa kontrolü yapar.
        
        Args:
            min_altitude (float): Minimum irtifa değeri
            
        Returns:
            bool: Minimum irtifanın altında mı?
        """
        if self.current_position is None:
            return False
            
        _, _, current_alt = self.current_position
        return current_alt < min_altitude 