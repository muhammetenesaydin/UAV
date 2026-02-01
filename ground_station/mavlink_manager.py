"""
Pixhawk ile MAVLink iletişimi için yönetici sınıfı.
"""
from pymavlink import mavutil
from dataclasses import dataclass
from datetime import datetime
import time
import threading

class MAVLinkManager:
    def __init__(self, connection_string="tcp:localhost:5760"):
        """
        MAVLink bağlantı yöneticisini başlatır.
        
        Args:
            connection_string: Bağlantı dizesi (örn: "tcp:localhost:5760" veya "COM3")
        """
        self.connection_string = connection_string
        self.vehicle = None
        self.is_connected = False
        self.is_running = False
        self.heartbeat_thread = None

    def connect(self):
        """Pixhawk'a bağlanır."""
        try:
            print(f"Pixhawk'a bağlanılıyor: {self.connection_string}")
            self.vehicle = mavutil.mavlink_connection(self.connection_string)
            
            # Heartbeat mesajını bekle
            self.vehicle.wait_heartbeat()
            print("Pixhawk'a bağlantı başarılı!")
            
            self.is_connected = True
            self.is_running = True
            
            # Heartbeat gönderme thread'ini başlat
            self.heartbeat_thread = threading.Thread(target=self._send_heartbeat)
            self.heartbeat_thread.daemon = True
            self.heartbeat_thread.start()
            
            return True
        except Exception as e:
            print(f"Bağlantı hatası: {e}")
            return False

    def disconnect(self):
        """Pixhawk bağlantısını kapatır."""
        self.is_running = False
        if self.heartbeat_thread:
            self.heartbeat_thread.join()
        if self.vehicle:
            self.vehicle.close()
        self.is_connected = False

    def _send_heartbeat(self):
        """Düzenli olarak heartbeat mesajı gönderir."""
        while self.is_running:
            if self.is_connected:
                self.vehicle.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
            time.sleep(1)

    def set_flight_mode(self, mode):
        """Uçuş modunu değiştirir."""
        if not self.is_connected:
            return False
            
        # Mode mapping
        mode_mapping = {
            'STABILIZE': 0,
            'ALT_HOLD': 2,
            'LOITER': 5,
            'AUTO': 3,
            'GUIDED': 4,
            'RTL': 6,
            'LAND': 9,
            'KAMIKAZE': 7  # Custom mode
        }
        
        if mode not in mode_mapping:
            print(f"Geçersiz mod: {mode}")
            return False
            
        try:
            self.vehicle.mav.set_mode_send(
                self.vehicle.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_mapping[mode]
            )
            return True
        except Exception as e:
            print(f"Mod değiştirme hatası: {e}")
            return False

    def arm(self, arm=True):
        """Aracı arm/disarm eder."""
        if not self.is_connected:
            return False
            
        try:
            self.vehicle.mav.command_long_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1 if arm else 0, 0, 0, 0, 0, 0, 0
            )
            return True
        except Exception as e:
            print(f"Arm/disarm hatası: {e}")
            return False

    def get_telemetry(self):
        """Telemetri verilerini alır."""
        if not self.is_connected:
            return None
            
        try:
            # Telemetri mesajlarını al
            msg = self.vehicle.recv_match(type=['GLOBAL_POSITION_INT', 'VFR_HUD', 'BATTERY_STATUS'],
                                        blocking=True, timeout=1)
            if msg is None:
                return None
                
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                altitude = msg.relative_alt / 1000.0  # mm -> m
                latitude = msg.lat / 1e7
                longitude = msg.lon / 1e7
                heading = msg.hdg / 100.0  # cdeg -> deg
            elif msg.get_type() == 'VFR_HUD':
                speed = msg.groundspeed
            elif msg.get_type() == 'BATTERY_STATUS':
                battery = msg.battery_remaining
                
            return {
                'altitude': altitude,
                'speed': speed,
                'latitude': latitude,
                'longitude': longitude,
                'heading': heading,
                'battery': battery
            }
        except Exception as e:
            print(f"Telemetri alma hatası: {e}")
            return None

    def start_mission(self):
        """Görevi başlatır."""
        if not self.is_connected:
            return False
            
        try:
            self.vehicle.mav.command_long_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_MISSION_START,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            return True
        except Exception as e:
            print(f"Görev başlatma hatası: {e}")
            return False

    def abort_mission(self):
        """Görevi iptal eder."""
        if not self.is_connected:
            return False
            
        try:
            # RTL moduna geç
            self.set_flight_mode('RTL')
            return True
        except Exception as e:
            print(f"Görev iptal hatası: {e}")
            return False 