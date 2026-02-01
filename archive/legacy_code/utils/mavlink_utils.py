"""
MAVLink iletişimi için yardımcı fonksiyonlar içeren modül.
"""
from pymavlink import mavutil
import time

class MavlinkConnection:
    def __init__(self, connection_string="udp:127.0.0.1:14550"):
        """
        MAVLink bağlantısını başlatır.
        
        Args:
            connection_string (str): Bağlantı string'i (udp, serial vb.)
        """
        self.connection = mavutil.mavlink_connection(connection_string)
        self.wait_heartbeat()
        
    def wait_heartbeat(self):
        """Pixhawk'tan heartbeat mesajı bekler."""
        print("Heartbeat bekleniyor...")
        self.connection.wait_heartbeat()
        print("Heartbeat alındı!")
        
    def set_mode(self, mode):
        """
        Uçuş modunu değiştirir.
        
        Args:
            mode (str): Hedef uçuş modu (GUIDED, AUTO, vb.)
        """
        mode_id = self.connection.mode_mapping()[mode]
        self.connection.mav.set_mode_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        
    def arm(self):
        """İHA'yı silahlandırır."""
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        
    def disarm(self):
        """İHA'yı silahsızlandırır."""
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
    def set_position_target_local_ned(self, x, y, z, vx=0, vy=0, vz=0):
        """
        Yerel NED koordinat sisteminde pozisyon hedefi belirler.
        
        Args:
            x, y, z (float): Hedef pozisyon (metre)
            vx, vy, vz (float): Hedef hız (m/s)
        """
        self.connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10,  # time_boot_ms
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b110111111000,  # type_mask
            x, y, z,
            vx, vy, vz,
            0, 0, 0,  # acceleration
            0, 0  # yaw, yaw_rate
        ))
        
    def get_position(self):
        """Güncel pozisyon bilgisini döndürür."""
        msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        return msg.x, msg.y, msg.z 