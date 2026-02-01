"""
Uçuş kontrolü ve manevra komutlarını yöneten sınıf.
"""
import math
from utils.mavlink_utils import MavlinkConnection

class Controller:
    def __init__(self, mavlink_connection):
        """
        Controller sınıfını başlatır.
        
        Args:
            mavlink_connection (MavlinkConnection): MAVLink bağlantı nesnesi
        """
        self.mavlink = mavlink_connection
        
    def takeoff(self, altitude):
        """
        Kalkış manevrası gerçekleştirir.
        
        Args:
            altitude (float): Hedef irtifa (metre)
        """
        self.mavlink.set_mode("GUIDED")
        self.mavlink.arm()
        self.mavlink.set_position_target_local_ned(0, 0, -altitude)
        
    def land(self):
        """İniş manevrası gerçekleştirir."""
        self.mavlink.set_mode("LAND")
        
    def move_to_position(self, x, y, z, max_g_force=3.0):
        """
        Belirtilen pozisyona hareket eder.
        
        Args:
            x, y, z (float): Hedef pozisyon koordinatları
            max_g_force (float): Maksimum G kuvveti
        """
        # G kuvveti sınırlaması
        current_pos = self.mavlink.get_position()
        distance = math.sqrt((x - current_pos[0])**2 + 
                           (y - current_pos[1])**2 + 
                           (z - current_pos[2])**2)
                           
        # Hızı G kuvveti sınırına göre ayarla
        max_speed = math.sqrt(max_g_force * 9.81 * distance)
        
        self.mavlink.set_position_target_local_ned(
            x, y, -z,
            max_speed, max_speed, max_speed
        )
        
    def execute_dive_maneuver(self, dive_point, max_speed, max_g_force):
        """
        Dalış manevrası gerçekleştirir.
        
        Args:
            dive_point (tuple): Dalış noktası koordinatları
            max_speed (float): Maksimum dalış hızı (m/s)
            max_g_force (float): Maksimum G kuvveti
        """
        x, y, z = dive_point
        
        # Dalış açısını hesapla
        current_pos = self.mavlink.get_position()
        horizontal_dist = math.sqrt((x - current_pos[0])**2 + 
                                  (y - current_pos[1])**2)
        vertical_dist = abs(z - current_pos[2])
        
        # Dalış açısını yeniden hesapla (optimum açı formülü)
        # Maksimum hız ve G kuvvetine bağlı optimum dalış açısı
        # V^2 = 2*a*s formülünden türetilmiştir
        g_value = 9.81  # m/s^2
        max_acceleration = max_g_force * g_value
        
        # Güvenli mesafede durabilmek için gerekli dalış açısı
        total_dist = math.sqrt(horizontal_dist**2 + vertical_dist**2)
        
        # Hız, ivme ve mesafe arasındaki ilişkiden optimum açıyı hesapla
        # s = v^2 / (2a) formülünü kullanarak minimum mesafeyi hesapla
        min_stopping_dist = (max_speed**2) / (2 * max_acceleration)
        
        # Eğer duruş mesafesi toplam mesafeden büyükse, max_speed'i azalt
        if min_stopping_dist > total_dist:
            max_speed = math.sqrt(2 * max_acceleration * total_dist * 0.8)  # %80'ini kullan
            
        # Optimize edilmiş dalış açısını hesapla
        dive_angle = math.degrees(math.atan2(vertical_dist, horizontal_dist))
        
        # Dalış açısını güvenli bir aralıkta tut
        dive_angle = max(15, min(60, dive_angle))  # 15° - 60° arasında sınırla
        
        # Yatış açısını hesapla (optimum yatış açısı)
        roll_angle = dive_angle * 0.6  # Dalış açısının %60'ı kadar yatış
        
        # Hız bileşenlerini hesapla
        vx = max_speed * math.cos(math.radians(dive_angle))
        vy = 0  # Yan hız bileşeni sıfır
        vz = max_speed * math.sin(math.radians(dive_angle))
        
        # Dalış komutunu gönder
        self.mavlink.set_position_target_local_ned(
            x, y, -z,
            vx, vy, vz,
            roll_angle, dive_angle, 0  # roll, pitch, yaw
        )
        
        return {
            'dive_angle': dive_angle,
            'roll_angle': roll_angle,
            'max_speed': max_speed,
            'horizontal_dist': horizontal_dist,
            'vertical_dist': vertical_dist,
            'total_dist': total_dist
        }
        
    def execute_recovery_maneuver(self, recovery_altitude, max_g_force):
        """
        Kurtarma manevrası gerçekleştirir.
        
        Args:
            recovery_altitude (float): Kurtarma irtifası
            max_g_force (float): Maksimum G kuvveti
        """
        # Yatış açısını sıfırla
        self.mavlink.set_position_target_local_ned(
            0, 0, -recovery_altitude,
            0, 0, 0,  # hız bileşenleri
            0, 0, 0   # açılar
        )
        
    def execute_target_lock(self, target_position):
        """
        Hedef kilitleme manevrası gerçekleştirir.
        
        Args:
            target_position (tuple): Hedef pozisyon koordinatları
        """
        x, y, z = target_position
        self.move_to_position(x, y, z)
        
    def check_rc_switch(self, channel, threshold=1500):
        """
        RC kumanda anahtarını kontrol eder.
        
        Args:
            channel (int): Kontrol edilecek kanal
            threshold (int): Eşik değeri
            
        Returns:
            bool: Anahtar aktif mi?
        """
        try:
            rc_value = self.mavlink.connection.recv_match(
                type='RC_CHANNELS',
                blocking=True
            ).channels[channel]
            
            return rc_value > threshold
        except:
            return False
            
    def get_rc_channels(self):
        """
        Tüm RC kanal değerlerini döndürür.
        
        Returns:
            dict: Kanal değerleri
        """
        try:
            channels = self.mavlink.connection.recv_match(
                type='RC_CHANNELS',
                blocking=True
            ).channels
            
            return {
                'roll': channels[0],
                'pitch': channels[1],
                'throttle': channels[2],
                'yaw': channels[3],
                'mode': channels[4],
                'arm': channels[7],
                'mission': channels[8],
                'abort': channels[9]
            }
        except:
            return None

    def calculate_optimal_dive_parameters(self, target_position, current_altitude, target_altitude,
                                        min_dive_angle=20, max_dive_angle=60, 
                                        safety_factor=0.8, max_g_force=3.0):
        """
        Optimum dalış parametrelerini hesaplar.
        
        Args:
            target_position (tuple): Hedef X,Y konumu (x, y)
            current_altitude (float): Mevcut irtifa (m)
            target_altitude (float): Hedef irtifa (m)
            min_dive_angle (float): Minimum dalış açısı (derece)
            max_dive_angle (float): Maksimum dalış açısı (derece)
            safety_factor (float): Güvenlik faktörü (0-1 arası)
            max_g_force (float): Maksimum G kuvveti
            
        Returns:
            dict: Optimum dalış parametreleri
        """
        current_pos = self.mavlink.get_position()
        target_x, target_y = target_position
        current_x, current_y = current_pos[0], current_pos[1]
        
        # Yatay ve dikey mesafeleri hesapla
        horizontal_dist = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        vertical_dist = current_altitude - target_altitude
        
        # Maksimum ivme sınırı (G kuvvetinden)
        max_acceleration = max_g_force * 9.81  # m/s^2
        
        # Optimum dalış açısını bul
        # Yatay ve dikey mesafelerin oranına göre tanjant hesapla
        raw_dive_angle = math.degrees(math.atan2(vertical_dist, horizontal_dist))
        
        # Açıyı belirlenen sınırlar içerisinde tut
        dive_angle = max(min_dive_angle, min(max_dive_angle, raw_dive_angle))
        
        # Toplam dalış mesafesi
        dive_path_distance = math.sqrt(horizontal_dist**2 + vertical_dist**2)
        
        # Maksimum hızı güvenli bir şekilde hesapla
        # v^2 = 2*a*s formülünden, s=distance/2 (yarı yolda yavaşlamaya başla)
        # Güvenlik faktörü ile çarp
        max_speed = math.sqrt(max_acceleration * dive_path_distance * safety_factor)
        
        # Güvenli hızı kontrol et (makul bir üst sınır koy, örn. 30 m/s)
        max_speed = min(max_speed, 30.0)  # 30 m/s (~108 km/h) üst sınır
        
        # Sonuçları döndür
        return {
            'dive_angle': dive_angle,
            'max_speed': max_speed,
            'horizontal_distance': horizontal_dist,
            'vertical_distance': vertical_dist,
            'total_distance': dive_path_distance,
            'time_to_target': dive_path_distance / max_speed if max_speed > 0 else 0
        } 