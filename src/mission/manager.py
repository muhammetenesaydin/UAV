from enum import Enum, auto

class MissionState(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    SEARCHING = auto()
    LOCKED_AIR = auto()
    DIVE_GROUND = auto()
    RECOVERY = auto()
    RTL = auto()

class MissionManager:
    def __init__(self, connection, detector, controller):
        self.connection = connection
        self.detector = detector
        self.controller = controller
        self.state = MissionState.IDLE
        self.active_mission = "SAVASAN"

    def run(self):
        while True:
            telemetry = self.connection.get_telemetry()
            
            if self.state == MissionState.IDLE:
                self._handle_idle()
            elif self.state == MissionState.TAKEOFF:
                self._handle_takeoff(telemetry)
            elif self.state == MissionState.SEARCHING:
                self._handle_searching()
            # ... diğer durumlar ...
            
            time.sleep(0.1)

    def _handle_idle(self):
        # Başlama komutu bekle
        pass

    def _handle_takeoff(self, telemetry):
        # Kalkış yüksekliğine ulaşıldı mı kontrol et
        pass
