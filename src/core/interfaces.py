from abc import ABC, abstractmethod
import numpy as np

class IConnection(ABC):
    @abstractmethod
    def connect(self): pass
    
    @abstractmethod
    def get_telemetry(self): pass
    
    @abstractmethod
    def set_mode(self, mode: str): pass
    
    @abstractmethod
    def arm(self): pass
    
    @abstractmethod
    def disarm(self): pass

class IDetector(ABC):
    @abstractmethod
    def detect(self, frame: np.ndarray): pass

class IController(ABC):
    @abstractmethod
    def takeoff(self, altitude: float): pass
    
    @abstractmethod
    def land(self): pass
    
    @abstractmethod
    def move_to_ned(self, x: float, y: float, z: float): pass

class IPlanner(ABC):
    @abstractmethod
    def get_next_waypoint(self, current_pos, target_pos): pass
