import os

class Config:
    """Merkezi yapılandırma yönetimi."""
    
    # MAVLink Bağlantı Ayarları
    CONNECTION_STRING = "udp:127.0.0.1:14550"
    
    # Görev Modları
    MISSION_MODE_SAVASAN = "SAVASAN"
    MISSION_MODE_KAMIKAZE = "KAMIKAZE"
    
    # Kamu / Yarışma Parametreleri
    HEARTBEAT_TIMEOUT = 5
    TAKEOFF_ALTITUDE = 120.0
    MIN_DIVE_ALTITUDE = 100.0
    QR_SIZE_METERS = 2.0
    
    # Kamera Ayarları
    CAMERA_URL = "http://127.0.0.1:8080/stream?topic=/cessna/image_raw"
    
    # YOLO Ayarları
    YOLO_MODEL_PATH = "models/yolov8.pt"
    CONFIDENCE_THRESHOLD = 0.5
    
    # PID / Kontrol Parametreleri (Örnek)
    DIVE_PITCH_CD = -4500
    RECOVERY_PITCH_CD = 1500

    @classmethod
    def load_from_env(cls):
        """Çevresel değişkenlerden yükleme yapma özelliği (opsiyonel)."""
        cls.CONNECTION_STRING = os.getenv("MAVLINK_CONN", cls.CONNECTION_STRING)
        cls.CAMERA_URL = os.getenv("CAMERA_URL", cls.CAMERA_URL)
