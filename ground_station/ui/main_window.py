"""
Yer kontrol istasyonu ana pencere sınıfı.
"""
import sys
import cv2
import numpy as np
from datetime import datetime
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                           QPushButton, QLabel, QGroupBox, QTextEdit, QApplication, QComboBox)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from ...iha_communication import IHACommunication, TelemetryData

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # İHA iletişimini başlat
        self.iha = IHACommunication()
        self.iha.set_telemetry_callback(self.on_telemetry)
        self.iha.start()
        
        self.init_ui()
        
        # UI güncelleme zamanlayıcısı
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)  # 100ms'de bir güncelle

    def on_telemetry(self, telemetry: TelemetryData):
        """Telemetri verisi alındığında çağrılır."""
        self.current_telemetry = telemetry

    def init_ui(self):
        """Kullanıcı arayüzünü başlatır."""
        self.setWindowTitle("İHA Yer Kontrol İstasyonu")
        self.setGeometry(100, 100, 1200, 800)

        # Varsayılan telemetri verisi
        self.current_telemetry = TelemetryData()

        # Ana widget ve layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Sol panel (Video ve telemetri)
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        
        # Video görüntüleme
        self.video_label = QLabel()
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(self.video_label)
        
        # Telemetri verileri
        telemetry_group = QGroupBox("Telemetri Verileri")
        telemetry_layout = QVBoxLayout()
        self.altitude_label = QLabel("İrtifa: -- m")
        self.speed_label = QLabel("Hız: -- m/s")
        self.battery_label = QLabel("Batarya: --%")
        self.gps_label = QLabel("GPS: --, --")
        self.heading_label = QLabel("Yön: -- derece")
        
        telemetry_layout.addWidget(self.altitude_label)
        telemetry_layout.addWidget(self.speed_label)
        telemetry_layout.addWidget(self.battery_label)
        telemetry_layout.addWidget(self.gps_label)
        telemetry_layout.addWidget(self.heading_label)
        telemetry_group.setLayout(telemetry_layout)
        left_layout.addWidget(telemetry_group)
        
        main_layout.addWidget(left_panel)
        
        # Sağ panel (Kontroller ve log)
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        
        # Uçuş Modları
        flight_mode_group = QGroupBox("Uçuş Modu Kontrolleri")
        flight_mode_layout = QVBoxLayout()
        
        self.mode_combo = QComboBox()
        self.mode_combo.addItems([
            "STABILIZE",
            "ALT_HOLD",
            "LOITER",
            "AUTO",
            "GUIDED",
            "RTL",
            "LAND",
            "KAMIKAZE"
        ])
        
        self.mode_button = QPushButton("Modu Değiştir")
        self.mode_button.clicked.connect(self.change_flight_mode)
        
        flight_mode_layout.addWidget(QLabel("Uçuş Modu Seçin:"))
        flight_mode_layout.addWidget(self.mode_combo)
        flight_mode_layout.addWidget(self.mode_button)
        flight_mode_group.setLayout(flight_mode_layout)
        right_layout.addWidget(flight_mode_group)
        
        # Görev Kontrolleri
        mission_group = QGroupBox("Görev Kontrolleri")
        mission_layout = QVBoxLayout()
        
        self.start_button = QPushButton("Görevi Başlat")
        self.abort_button = QPushButton("Görevi İptal Et")
        self.arm_button = QPushButton("Arm")
        
        mission_layout.addWidget(self.start_button)
        mission_layout.addWidget(self.abort_button)
        mission_layout.addWidget(self.arm_button)
        mission_group.setLayout(mission_layout)
        right_layout.addWidget(mission_group)
        
        # Kamera Kontrolleri
        camera_group = QGroupBox("Kamera Kontrolleri")
        camera_layout = QVBoxLayout()
        
        self.camera_mode_button = QPushButton("Kamera Modu Değiştir")
        self.snapshot_button = QPushButton("Ekran Görüntüsü Al")
        
        camera_layout.addWidget(self.camera_mode_button)
        camera_layout.addWidget(self.snapshot_button)
        camera_group.setLayout(camera_layout)
        right_layout.addWidget(camera_group)
        
        # Sistem log'u
        log_group = QGroupBox("Sistem Logu")
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        
        log_group.setLayout(log_layout)
        right_layout.addWidget(log_group)
        
        main_layout.addWidget(right_panel)
        
        # Buton bağlantıları
        self.start_button.clicked.connect(self.start_mission)
        self.abort_button.clicked.connect(self.abort_mission)
        self.arm_button.clicked.connect(self.toggle_arm)
        self.camera_mode_button.clicked.connect(self.toggle_camera_mode)
        self.snapshot_button.clicked.connect(self.take_snapshot)
        
    def update_ui(self):
        """Arayüzü günceller."""
        # Telemetri verilerini güncelle
        self.altitude_label.setText(f"İrtifa: {self.current_telemetry.altitude:.1f} m")
        self.speed_label.setText(f"Hız: {self.current_telemetry.speed:.1f} m/s")
        self.battery_label.setText(f"Batarya: {self.current_telemetry.battery}%")
        self.gps_label.setText(f"GPS: {self.current_telemetry.latitude:.6f}, {self.current_telemetry.longitude:.6f}")
        self.heading_label.setText(f"Yön: {self.current_telemetry.heading:.1f}°")
            
    def change_flight_mode(self):
        """Uçuş modunu değiştirir."""
        mode = self.mode_combo.currentText()
        if self.iha.set_flight_mode(mode):
            self.log_text.append(f"Uçuş modu değiştiriliyor: {mode}")
        
    def start_mission(self):
        """Görevi başlatır."""
        if self.iha.start_mission():
            self.log_text.append("Görev başlatıldı")
        
    def abort_mission(self):
        """Görevi iptal eder."""
        if self.iha.abort_mission():
            self.log_text.append("Görev iptal edildi")
        
    def toggle_arm(self):
        """Arm/Disarm durumunu değiştirir."""
        if self.current_telemetry.armed:
            if self.iha.disarm():
                self.arm_button.setText("Arm")
                self.log_text.append("Disarm komutu gönderildi")
        else:
            if self.iha.arm():
                self.arm_button.setText("Disarm")
                self.log_text.append("Arm komutu gönderildi")
        
    def toggle_camera_mode(self):
        """Kamera modunu değiştirir."""
        if self.iha.toggle_camera():
            self.log_text.append("Kamera modu değiştirildi")
        
    def take_snapshot(self):
        """Ekran görüntüsü alır."""
        if self.iha.take_snapshot():
            self.log_text.append("Ekran görüntüsü alındı")
            
    def closeEvent(self, event):
        """Pencere kapatıldığında çağrılır."""
        self.iha.stop()
        event.accept()

def main():
    """Ana uygulama döngüsü."""
    app = QApplication(sys.argv)
    
    # Ana pencereyi oluştur
    window = MainWindow()
    window.show()
    
    # Uygulama döngüsünü başlat
    sys.exit(app.exec_()) 