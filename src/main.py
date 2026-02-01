import signal
import sys
from src.common.config import Config
from src.core.mavlink_connection import MavlinkConnection
from src.perception.detectors import YOLODetector, QRDetector
from src.control.uav_controller import UAVController
from src.mission.manager import MissionManager

def main():
    # 1. Konfigürasyonu yükle
    Config.load_from_env()
    
    # 2. Bileşenleri başlat (Dependency Injection)
    connection = MavlinkConnection(Config.CONNECTION_STRING)
    yolo_detector = YOLODetector(Config.YOLO_MODEL_PATH)
    qr_detector = QRDetector()
    
    # Başlangıçta MAVLink'e bağlan
    connection.connect()
    
    controller = UAVController(connection)
    
    # 3. Görev Yöneticisini Başlat
    manager = MissionManager(connection, yolo_detector, controller)
    
    print("Sistem Hazır. Görev başlatılıyor...")
    
    try:
        manager.run()
    except KeyboardInterrupt:
        print("\nKullanıcı durdurdu. Güvenli çıkış yapılıyor...")
        connection.disarm()
        sys.exit(0)

if __name__ == "__main__":
    main()
