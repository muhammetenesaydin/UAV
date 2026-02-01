"""
İHA iletişim modülü.
"""
import socket
import json
import threading
from typing import Optional, Dict, Any
from dataclasses import dataclass
from datetime import datetime

@dataclass
class TelemetryData:
    """Telemetri verisi sınıfı."""
    altitude: float = 0.0
    speed: float = 0.0
    battery: int = 100
    latitude: float = 0.0
    longitude: float = 0.0
    heading: float = 0.0
    flight_mode: str = "MANUAL"
    armed: bool = False
    timestamp: datetime = datetime.now()

class IHACommunication:
    def __init__(self, ip: str = "10.192.55.181", port: int = 14755):
        """İHA iletişim sınıfını başlatır."""
        self.UDP_IP = ip
        self.UDP_PORT = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.is_running = False
        self.telemetry_callback = None
        
    def start(self):
        """İletişimi başlatır."""
        self.is_running = True
        # Telemetri dinleme thread'ini başlat
        self.thread = threading.Thread(target=self._listen_telemetry)
        self.thread.daemon = True
        self.thread.start()
        
    def stop(self):
        """İletişimi durdurur."""
        self.is_running = False
        if hasattr(self, 'thread'):
            self.thread.join()
        self.sock.close()
        
    def _listen_telemetry(self):
        """Telemetri verilerini dinler."""
        while self.is_running:
            try:
                data, addr = self.sock.recvfrom(1024)
                message = data.decode()
                
                # JSON formatındaki telemetri verisini işle
                if message.startswith('{'):
                    try:
                        telemetry = json.loads(message)
                        if telemetry.get('type') == 'TELEMETRY':
                            if self.telemetry_callback:
                                self.telemetry_callback(TelemetryData(**telemetry['data']))
                    except json.JSONDecodeError:
                        print(f"Geçersiz JSON verisi: {message}")
                else:
                    # Durum mesajlarını işle
                    print(f"Durum mesajı: {message}")
            except Exception as e:
                print(f"Telemetri dinleme hatası: {e}")
                
    def send_command(self, command: str):
        """Komut gönderir."""
        try:
            self.sock.sendto(command.encode(), (self.UDP_IP, self.UDP_PORT))
            print(f"Gönderildi: {command}")
            return True
        except Exception as e:
            print(f"Komut gönderme hatası: {e}")
            return False
            
    def arm(self):
        """İHA'yı arm eder."""
        return self.send_command("ARM")
        
    def disarm(self):
        """İHA'yı disarm eder."""
        return self.send_command("DISARM")
        
    def start_mission(self):
        """Görevi başlatır."""
        return self.send_command("START")
        
    def abort_mission(self):
        """Görevi iptal eder."""
        return self.send_command("ABORT")
        
    def set_flight_mode(self, mode: str):
        """Uçuş modunu değiştirir."""
        return self.send_command(f"MODE:{mode}")
        
    def toggle_camera(self):
        """Kamera modunu değiştirir."""
        return self.send_command("CAMERA:TOGGLE")
        
    def take_snapshot(self):
        """Ekran görüntüsü alır."""
        return self.send_command("CAMERA:SNAPSHOT")
        
    def set_telemetry_callback(self, callback):
        """Telemetri verisi alındığında çağrılacak fonksiyonu ayarlar."""
        self.telemetry_callback = callback

# Örnek kullanım
if __name__ == "__main__":
    iha = IHACommunication()
    iha.start()
    
    # Telemetri verisi alındığında çağrılacak fonksiyon
    def on_telemetry(telemetry: TelemetryData):
        print(f"Telemetri: {telemetry}")
    
    iha.set_telemetry_callback(on_telemetry)
    
    # Örnek komutlar
    iha.arm()
    iha.set_flight_mode("GUIDED")
    iha.start_mission()
    
    try:
        # Programı çalışır durumda tut
        while True:
            pass
    except KeyboardInterrupt:
        iha.stop()