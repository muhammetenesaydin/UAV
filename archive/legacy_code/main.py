"""
Savaşan İHA yarışması için ana program.
"""
import os
from utils.mavlink_utils import MavlinkConnection
from mission.mission_fsm import MissionFSM

def main():
    # MAVLink bağlantısını başlat
    mavlink = MavlinkConnection("udp:127.0.0.1:14550")
    
    # YOLO model dosyasının yolunu belirt
    model_path = "models/yolov7.pt"
    
    # MissionFSM nesnesini oluştur
    mission = MissionFSM(mavlink, model_path)
    
    try:
        # Görevi başlat
        mission.run()
    except KeyboardInterrupt:
        print("Program kullanıcı tarafından durduruldu.")
    except Exception as e:
        print(f"Bir hata oluştu: {str(e)}")
    finally:
        # Temizlik işlemleri
        mavlink.disarm()
        print("Program sonlandırıldı.")

if __name__ == "__main__":
    main() 