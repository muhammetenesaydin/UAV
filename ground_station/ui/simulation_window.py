from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                           QPushButton, QLabel, QComboBox, QStatusBar, QMessageBox,
                           QLineEdit, QGroupBox, QFormLayout)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
import sys
import os
from pymavlink import mavutil
import subprocess
import socket

class SimulationWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("İHA ROS Kontrol Paneli")
        self.setGeometry(100, 100, 800, 600)
        
        # MAVLink bağlantısı
        self.mavlink = None
        self.connected = False
        self.armed = False
        self.mission_running = False
        
        # Ana widget ve layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)
        
        # ROS Bağlantı Ayarları
        connection_group = QGroupBox("ROS Bağlantı Ayarları")
        connection_layout = QFormLayout()
        
        # IP Adresi
        self.ip_input = QLineEdit()
        self.ip_input.setPlaceholderText("Örn: 192.168.1.100")
        connection_layout.addRow("ROS Bilgisayarı IP:", self.ip_input)
        
        # Port
        self.port_input = QLineEdit()
        self.port_input.setText("14550")  # Varsayılan MAVLink portu
        connection_layout.addRow("MAVLink Port:", self.port_input)
        
        # ROS Master Port
        self.ros_port_input = QLineEdit()
        self.ros_port_input.setText("11311")  # Varsayılan ROS portu
        connection_layout.addRow("ROS Master Port:", self.ros_port_input)
        
        connection_group.setLayout(connection_layout)
        layout.addWidget(connection_group)
        
        # Bağlantı durumu
        self.status_label = QLabel("Bağlantı Durumu: Bağlı Değil")
        self.status_label.setFont(QFont('Arial', 12))
        layout.addWidget(self.status_label)
        
        # Bağlantı test butonu
        self.test_btn = QPushButton("Bağlantıyı Test Et")
        self.test_btn.clicked.connect(self.test_connection)
        layout.addWidget(self.test_btn)
        
        # Bağlantı butonu
        self.connect_btn = QPushButton("ROS'a Bağlan")
        self.connect_btn.clicked.connect(self.connect_to_ros)
        layout.addWidget(self.connect_btn)
        
        # Kontrol butonları
        control_group = QGroupBox("İHA Kontrolleri")
        control_layout = QVBoxLayout()
        
        button_layout = QHBoxLayout()
        self.arm_btn = QPushButton("ARM")
        self.arm_btn.setEnabled(False)
        self.arm_btn.clicked.connect(self.toggle_arm)
        button_layout.addWidget(self.arm_btn)
        
        self.mission_btn = QPushButton("Görevi Başlat")
        self.mission_btn.setEnabled(False)
        self.mission_btn.clicked.connect(self.start_mission)
        button_layout.addWidget(self.mission_btn)
        
        control_layout.addLayout(button_layout)
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)
        
        # Durum bilgisi
        self.info_label = QLabel("İHA Durumu: Beklemede")
        self.info_label.setFont(QFont('Arial', 10))
        layout.addWidget(self.info_label)
        
        # Status bar
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage("Hazır")
        
        # Timer for status updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(1000)  # Her saniye güncelle
        
    def test_connection(self):
        """ROS bilgisayarına bağlantıyı test et"""
        ip = self.ip_input.text().strip()
        ros_port = self.ros_port_input.text().strip()
        
        if not ip or not ros_port:
            QMessageBox.warning(self, "Hata", "IP adresi ve port gerekli!")
            return
            
        try:
            # ROS Master'a bağlantıyı test et
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2)
            result = sock.connect_ex((ip, int(ros_port)))
            sock.close()
            
            if result == 0:
                QMessageBox.information(self, "Başarılı", 
                    f"ROS Master'a bağlantı başarılı!\nIP: {ip}\nPort: {ros_port}")
            else:
                QMessageBox.warning(self, "Hata", 
                    f"ROS Master'a bağlanılamadı!\nIP: {ip}\nPort: {ros_port}")
                
        except Exception as e:
            QMessageBox.critical(self, "Bağlantı Hatası", 
                               f"Test sırasında hata: {str(e)}")
    
    def connect_to_ros(self):
        try:
            ip = self.ip_input.text().strip()
            port = self.port_input.text().strip()
            
            if not ip or not port:
                QMessageBox.warning(self, "Hata", "IP adresi ve port gerekli!")
                return
            
            # MAVLink bağlantı string'ini oluştur
            connection_string = f"udp:{ip}:{port}"
            
            # MAVLink bağlantısını kur
            self.mavlink = mavutil.mavlink_connection(connection_string)
            self.mavlink.wait_heartbeat()
            
            # ROS_MASTER_URI ve ROS_IP ortam değişkenlerini ayarla
            os.environ['ROS_MASTER_URI'] = f"http://{ip}:{self.ros_port_input.text()}"
            os.environ['ROS_IP'] = socket.gethostbyname(socket.gethostname())
            
            self.connected = True
            self.status_label.setText(f"Bağlantı Durumu: {ip} adresine bağlı")
            self.status_label.setStyleSheet("color: green")
            self.connect_btn.setText("Bağlantıyı Kes")
            self.arm_btn.setEnabled(True)
            self.statusBar.showMessage(f"ROS bilgisayarına bağlantı kuruldu: {ip}")
            
        except Exception as e:
            QMessageBox.critical(self, "Bağlantı Hatası", 
                               f"ROS bilgisayarına bağlanılamadı: {str(e)}")
            self.statusBar.showMessage("Bağlantı hatası!")
    
    def toggle_arm(self):
        if not self.connected:
            return
            
        try:
            if not self.armed:
                # ARM komutu gönder
                self.mavlink.mav.command_long_send(
                    self.mavlink.target_system,
                    self.mavlink.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 0, 0, 0, 0, 0, 0)
                self.armed = True
                self.arm_btn.setText("DISARM")
                self.mission_btn.setEnabled(True)
                self.statusBar.showMessage("İHA ARM edildi")
            else:
                # DISARM komutu gönder
                self.mavlink.mav.command_long_send(
                    self.mavlink.target_system,
                    self.mavlink.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 0, 0, 0, 0, 0, 0)
                self.armed = False
                self.arm_btn.setText("ARM")
                self.mission_btn.setEnabled(False)
                self.statusBar.showMessage("İHA DISARM edildi")
                
        except Exception as e:
            QMessageBox.warning(self, "ARM/DISARM Hatası", 
                              f"İşlem başarısız: {str(e)}")
    
    def start_mission(self):
        if not self.connected or not self.armed:
            return
            
        try:
            # İHA mission dosyasını çalıştır
            mission_script = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 
                                        "iha_mission.py")
            
            # Bağlantı parametrelerini ekle
            connection_string = f"udp:{self.ip_input.text()}:{self.port_input.text()}"
            
            # Python process'i başlat
            self.mission_process = subprocess.Popen([
                sys.executable, 
                mission_script,
                "--connection", connection_string
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
            self.mission_running = True
            self.mission_btn.setText("Görevi Durdur")
            self.statusBar.showMessage("Görev çalışıyor...")
            
        except Exception as e:
            QMessageBox.critical(self, "Görev Hatası", 
                               f"Görev başlatılamadı: {str(e)}")
    
    def update_status(self):
        if not self.connected or not self.mavlink:
            return
            
        try:
            # İHA durumunu güncelle
            msg = self.mavlink.recv_match(type='HEARTBEAT', blocking=False)
            if msg:
                mode = self.mavlink.flightmode
                armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                
                # Altitude bilgisini al
                alt_msg = self.mavlink.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                altitude = alt_msg.relative_alt / 1000.0 if alt_msg else 0.0
                
                status_text = f"Mod: {mode}, ARM: {'Evet' if armed else 'Hayır'}, İrtifa: {altitude:.1f}m"
                self.info_label.setText(status_text)
                
        except Exception as e:
            self.statusBar.showMessage(f"Durum güncelleme hatası: {str(e)}")
    
    def closeEvent(self, event):
        # Program kapatılırken temizlik
        if hasattr(self, 'mission_process') and self.mission_process:
            self.mission_process.terminate()
        
        if self.mavlink:
            try:
                if self.armed:
                    self.toggle_arm()  # DISARM
                self.mavlink.close()
            except:
                pass
        
        event.accept() 