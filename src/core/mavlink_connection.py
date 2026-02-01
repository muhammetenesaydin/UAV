import time
import math
from pymavlink import mavutil
from .interfaces import IConnection

class MavlinkConnection(IConnection):
    def __init__(self, connection_string):
        self.connection_string = connection_string
        self.mav = None
        self.state = {}

    def connect(self):
        print(f"Bağlantı kuruluyor: {self.connection_string}")
        self.mav = mavutil.mavlink_connection(self.connection_string)
        self.mav.wait_heartbeat()
        print("Heartbeat alındı!")

    def get_telemetry(self):
        msg = self.mav.recv_match(type=['GLOBAL_POSITION_INT', 'VFR_HUD', 'ATTITUDE'], blocking=False)
        if msg:
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                self.state['lat'] = msg.lat / 1e7
                self.state['lon'] = msg.lon / 1e7
                self.state['alt'] = msg.relative_alt / 1000.0
            elif msg.get_type() == 'VFR_HUD':
                self.state['airspeed'] = msg.airspeed
                self.state['groundspeed'] = msg.groundspeed
            elif msg.get_type() == 'ATTITUDE':
                self.state['roll'] = math.degrees(msg.roll)
                self.state['pitch'] = math.degrees(msg.pitch)
                self.state['yaw'] = math.degrees(msg.yaw)
        return self.state

    def set_mode(self, mode):
        if mode not in self.mav.mode_mapping():
            return False
        mode_id = self.mav.mode_mapping()[mode]
        self.mav.mav.set_mode_send(
            self.mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        return True

    def arm(self):
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)

    def disarm(self):
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0)
