import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QTextEdit, QFrame)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont, QColor
from src.common.config import Config
from src.core.mavlink_connection import MavlinkConnection

class ModernGCS(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TEKNOFEST 2026 - Savaşan İHA GCS")
        self.resize(1000, 700)
        self.setStyleSheet("""
            QMainWindow { background-color: #121212; }
            QLabel { color: #e0e0e0; font-family: 'Segoe UI'; }
            QPushButton { 
                background-color: #1e88e5; 
                color: white; 
                border-radius: 5px; 
                padding: 10px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #1565c0; }
            QFrame#StatusPanel { 
                background-color: #1e1e1e; 
                border: 1px solid #333; 
                border-radius: 10px;
            }
            QTextEdit { 
                background-color: #000; 
                color: #00ff00; 
                font-family: 'Consolas';
                border: 1px solid #333;
            }
        """)

        # Main Layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)

        # Header
        header = QLabel("SAVAŞAN İHA KONTROL PANELİ")
        header.setFont(QFont("Segoe UI", 20, QFont.Bold))
        header.setAlignment(Qt.AlignCenter)
        layout.addWidget(header)

        # Top Section: Telemetry & Status
        top_layout = QHBoxLayout()
        
        self.telemetry_panel = QFrame()
        self.telemetry_panel.setObjectName("StatusPanel")
        tel_layout = QVBoxLayout(self.telemetry_panel)
        
        self.lbl_alt = QLabel("İrtifa: -- m")
        self.lbl_speed = QLabel("Hız: -- m/s")
        self.lbl_mode = QLabel("Mod: --")
        self.lbl_arm = QLabel("ARM: --")
        
        for lbl in [self.lbl_alt, self.lbl_speed, self.lbl_mode, self.lbl_arm]:
            lbl.setFont(QFont("Segoe UI", 12))
            tel_layout.addWidget(lbl)
            
        top_layout.addWidget(self.telemetry_panel, 1)

        # Controls Section
        ctrl_panel = QFrame()
        ctrl_panel.setObjectName("StatusPanel")
        ctrl_layout = QVBoxLayout(ctrl_panel)
        
        self.btn_connect = QPushButton("BAĞLAN")
        self.btn_takeoff = QPushButton("KALKIŞ (AUTO)")
        self.btn_mission = QPushButton("GÖREVİ BAŞLAT")
        self.btn_rtl = QPushButton("EVE DÖN (RTL)")
        
        ctrl_layout.addWidget(self.btn_connect)
        ctrl_layout.addWidget(self.btn_takeoff)
        ctrl_layout.addWidget(self.btn_mission)
        ctrl_layout.addWidget(self.btn_rtl)
        
        top_layout.addWidget(ctrl_panel, 1)
        layout.addLayout(top_layout)

        # Console Output
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.append(">>> Sistem hazır. Bağımsız katmanlar yüklendi.")
        layout.addWidget(self.console)

        # Connection Logic
        self.conn = None
        self.btn_connect.clicked.connect(self.toggle_connection)
        
        # Timer for UI Update
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)

    def toggle_connection(self):
        try:
            self.conn = MavlinkConnection(Config.CONNECTION_STRING)
            self.conn.connect()
            self.console.append(">>> MAVLink bağlantısı kuruldu.")
            self.btn_connect.setText("BAĞLI")
            self.btn_connect.setEnabled(False)
        except Exception as e:
            self.console.append(f">>> HATA: {str(e)}")

    def update_ui(self):
        if self.conn:
            tel = self.conn.get_telemetry()
            self.lbl_alt.setText(f"İrtifa: {tel.get('alt', 0):.1f} m")
            self.lbl_speed.setText(f"Hız: {tel.get('airspeed', 0):.1f} m/s")
            self.lbl_mode.setText(f"Mod: {tel.get('mode', '---')}")
            # Basit kilitlenme simülasyonu veya gerçek veri buraya
            
def start_ui():
    app = QApplication(sys.argv)
    window = ModernGCS()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    start_ui()
