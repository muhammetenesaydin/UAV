"""
Yer istasyonu iletişim yöneticisi sınıfı.
"""
import json
import threading
import cv2
import numpy as np
import requests
from queue import Queue, Empty
from typing import Dict, Any, Optional
from dataclasses import dataclass
from datetime import datetime

@dataclass
class TelemetryData:
    """Telemetri verisi veri sınıfı."""
    altitude: float = 0.0
    speed: float = 0.0
    battery: int = 100
    latitude: float = 0.0
    longitude: float = 0.0
    heading: float = 0.0
    flight_mode: str = "MANUAL"
    armed: bool = False
    timestamp: datetime = datetime.now()

@dataclass
class SystemDecision:
    """Sistem kararları veri sınıfı."""
    decision_id: str
    timestamp: datetime
    decision_type: str
    parameters: Dict[str, Any]
    priority: int

class CommunicationManager:
    def __init__(self):
        """İletişim yöneticisi sınıfını başlatır."""
        # Görüntü akışı ayarları
        self.stream_url = "http://0.0.0.0:8080/stream?topic=/cessna/image_raw"
        self.cap = None
        
        # Veri kuyrukları
        self.telemetry_queue = Queue()
        self.video_queue = Queue()
        self.mission_queue = Queue()
        self.decision_queue = Queue()
        self.user_input_queue = Queue()
        self.command_queue = Queue()
        
        # İletişim durumu
        self.is_running = False
        self.connected_clients = set()
        
        # İş parçacıkları
        self.threads = []
        
    def start(self):
        """İletişim yöneticisini başlatır."""
        self.is_running = True
        
        # Görüntü akışını başlat
        try:
            self.cap = cv2.VideoCapture(self.stream_url)
            if not self.cap.isOpened():
                print("Görüntü akışı başlatılamadı!")
            else:
                print("Görüntü akışı başlatıldı")
        except Exception as e:
            print(f"Görüntü akışı hatası: {e}")
        
        # İş parçacıklarını başlat
        self.threads.extend([
            threading.Thread(target=self._handle_video_stream),
            threading.Thread(target=self._handle_mission_data),
            threading.Thread(target=self._handle_system_decisions),
            threading.Thread(target=self._handle_user_input),
            threading.Thread(target=self._run)
        ])
        
        # Thread'leri başlat
        for thread in self.threads:
            thread.daemon = True
            thread.start()
            
        return True

    def stop(self):
        """İletişim yöneticisini durdurur."""
        self.is_running = False
        
        # Görüntü akışını kapat
        if self.cap is not None:
            self.cap.release()
        
        # Thread'leri bekle
        for thread in self.threads:
            thread.join()

    def _handle_video_stream(self):
        """Video akışı işleme döngüsü."""
        while self.is_running:
            try:
                if self.cap is not None and self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if ret:
                        # BGR'den RGB'ye dönüştür
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        self.video_queue.put(frame)
                    else:
                        print("Görüntü okunamadı")
                        # Bağlantıyı yeniden dene
                        self.cap.release()
                        self.cap = cv2.VideoCapture(self.stream_url)
                else:
                    # Bağlantıyı yeniden dene
                    self.cap = cv2.VideoCapture(self.stream_url)
                    if not self.cap.isOpened():
                        print("Görüntü akışı yeniden başlatılamadı!")
            except Exception as e:
                print(f"Video akışı hatası: {e}")
                # Hata durumunda kısa bir bekleme
                threading.Event().wait(1.0)

    def _handle_mission_data(self):
        """Görev verisi işleme döngüsü."""
        while self.is_running:
            try:
                mission_data = self.mission_queue.get(timeout=1.0)
                # Görev verisini tüm bağlı istemcilere gönder
                self._broadcast_data('mission', mission_data)
            except Empty:
                continue

    def _handle_system_decisions(self):
        """Sistem kararları işleme döngüsü."""
        while self.is_running:
            try:
                decision = self.decision_queue.get(timeout=1.0)
                # Sistem kararını tüm bağlı istemcilere gönder
                self._broadcast_data('decision', decision)
            except Empty:
                continue

    def _handle_user_input(self):
        """Kullanıcı girdisi işleme döngüsü."""
        while self.is_running:
            try:
                user_input = self.user_input_queue.get(timeout=1.0)
                # Kullanıcı girdisini işle ve gerekli birimlere ilet
                self._process_user_command(user_input)
            except Empty:
                continue

    def _broadcast_data(self, data_type: str, data: Any):
        """Veriyi tüm bağlı istemcilere gönderir."""
        message = {
            'type': data_type,
            'timestamp': datetime.now().isoformat(),
            'data': data
        }
        
        # JSON formatına çevir
        try:
            json_data = json.dumps(message)
            # Tüm bağlı istemcilere gönder
            for client in self.connected_clients:
                client.send(json_data)
        except Exception as e:
            print(f"Veri gönderme hatası: {e}")

    def _run(self):
        """Ana iletişim döngüsü."""
        while self.is_running:
            # Komut kuyruğunu kontrol et
            try:
                command = self.command_queue.get_nowait()
                self._process_command(command)
            except Empty:
                pass

    def _process_command(self, command: str):
        """Komutları işler."""
        parts = command.split()
        cmd_type = parts[0]

        if cmd_type == "SET_MODE":
            print(f"Uçuş modu değiştirildi: {parts[1]}")
        elif cmd_type == "START_MISSION":
            print("Görev başlatıldı")
        elif cmd_type == "ABORT_MISSION":
            print("Görev iptal edildi")
        elif cmd_type == "TOGGLE_ARM":
            print("Arm durumu değiştirildi")

    def send_command(self, command: str):
        """Komut gönderir."""
        self.command_queue.put(command)
        self._process_command(command)

    def get_telemetry(self) -> Optional[TelemetryData]:
        """En son telemetri verilerini döndürür."""
        try:
            return self.telemetry_queue.get_nowait()
        except Empty:
            return TelemetryData()  # Varsayılan değerlerle döndür

    def get_video_frame(self):
        """En son video karesini döndürür."""
        try:
            return self.video_queue.get_nowait()
        except Empty:
            return None 