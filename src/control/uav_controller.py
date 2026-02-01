from src.core.interfaces import IController
from pymavlink import mavutil

class UAVController(IController):
    def __init__(self, mavlink_conn):
        self.mavlink_conn = mavlink_conn
        self.mav = mavlink_conn.mav

    def takeoff(self, altitude):
        print(f"Kalkış emri: {altitude}m")
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude)

    def land(self):
        print("İniş emri verildi.")
        self.mavlink_conn.set_mode("LAND")

    def move_to_ned(self, x, y, z):
        # SET_POSITION_TARGET_LOCAL_NED
        self.mav.mav.set_position_target_local_ned_send(
            0, self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,
            x, y, z, 0, 0, 0, 0, 0, 0, 0, 0)
