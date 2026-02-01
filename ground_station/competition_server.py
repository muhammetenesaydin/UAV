"""
Yarışma sunucusu ile iletişimi sağlayan sınıf.
"""
import json
import socket
import threading
from typing import Dict, Any
from datetime import datetime
from dataclasses import dataclass

@dataclass
class ServerConfig:
    """Sunucu yapılandırma sınıfı."""
    host: str
    port: int
    team_id: str
    api_key: str

class CompetitionServer:
    def __init__(self, config: ServerConfig):
        """
        CompetitionServer sınıfını başlatır.
        
        Args:
            config (ServerConfig): Sunucu yapılandırması
        """
        self.config = config
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.is_connected = False
        self.is_running = False
        self.receive_thread = None
        
    def connect(self):
        """Sunucuya bağlanır."""
        try:
            self.socket.connect((self.config.host, self.config.port))
            self.is_connected = True
            
            # Kimlik doğrulama
            auth_data = {
                'team_id': self.config.team_id,
                'api_key': self.config.api_key,
                'timestamp': datetime.now().isoformat()
            }
            self.send_data('auth', auth_data)
            
            # Veri alma iş parçacığını başlat
            self.is_running = True
            self.receive_thread = threading.Thread(target=self._receive_data)
            self.receive_thread.start()
            
            return True
        except Exception as e:
            print(f"Sunucu bağlantı hatası: {e}")
            return False
            
    def disconnect(self):
        """Sunucu bağlantısını kapatır."""
        self.is_running = False
        if self.receive_thread:
            self.receive_thread.join()
        if self.is_connected:
            self.socket.close()
            self.is_connected = False
            
    def send_data(self, data_type: str, data: Dict[str, Any]):
        """
        Sunucuya veri gönderir.
        
        Args:
            data_type (str): Veri tipi
            data (Dict): Gönderilecek veri
        """
        if not self.is_connected:
            return
            
        message = {
            'type': data_type,
            'team_id': self.config.team_id,
            'timestamp': datetime.now().isoformat(),
            'data': data
        }
        
        try:
            json_data = json.dumps(message)
            self.socket.send(json_data.encode('utf-8'))
        except Exception as e:
            print(f"Veri gönderme hatası: {e}")
            self.disconnect()
            
    def send_telemetry(self, telemetry_data: Dict):
        """Telemetri verisi gönderir."""
        self.send_data('telemetry', telemetry_data)
        
    def send_mission_status(self, status_data: Dict):
        """Görev durumu gönderir."""
        self.send_data('mission_status', status_data)
        
    def send_qr_data(self, qr_data: Dict):
        """QR kod verisi gönderir."""
        self.send_data('qr_data', qr_data)
        
    def _receive_data(self):
        """Sunucudan gelen verileri alır."""
        buffer = ""
        while self.is_running:
            try:
                data = self.socket.recv(4096).decode('utf-8')
                if not data:
                    break
                    
                buffer += data
                
                # Tam JSON mesajları işle
                while True:
                    try:
                        # JSON mesajının sonunu bul
                        json_end = buffer.find('}') + 1
                        if json_end == 0:
                            break
                            
                        # JSON mesajını parse et
                        message = json.loads(buffer[:json_end])
                        self._handle_message(message)
                        
                        # İşlenen mesajı bufferdan çıkar
                        buffer = buffer[json_end:]
                    except json.JSONDecodeError:
                        break
                        
            except Exception as e:
                print(f"Veri alma hatası: {e}")
                break
                
        self.disconnect()
        
    def _handle_message(self, message: Dict):
        """
        Gelen mesajı işler.
        
        Args:
            message (Dict): İşlenecek mesaj
        """
        message_type = message.get('type')
        
        if message_type == 'server_time':
            # Sunucu zamanını senkronize et
            self._sync_time(message['data'])
        elif message_type == 'mission_command':
            # Görev komutunu işle
            self._handle_mission_command(message['data'])
        elif message_type == 'system_status':
            # Sistem durumunu güncelle
            self._handle_system_status(message['data'])
            
    def _sync_time(self, time_data: Dict):
        """Sunucu zamanını senkronize eder."""
        server_time = datetime.fromisoformat(time_data['server_time'])
        time_diff = datetime.now() - server_time
        print(f"Sunucu zaman farkı: {time_diff.total_seconds():.3f} saniye")
        
    def _handle_mission_command(self, command_data: Dict):
        """Görev komutunu işler."""
        command = command_data.get('command')
        if command == 'start':
            print("Sunucu: Görev başlatma komutu alındı")
        elif command == 'abort':
            print("Sunucu: Görev iptal komutu alındı")
            
    def _handle_system_status(self, status_data: Dict):
        """Sistem durumunu işler."""
        print(f"Sunucu durumu: {status_data.get('status')}")
        # Sistem durumuna göre gerekli işlemleri yap 