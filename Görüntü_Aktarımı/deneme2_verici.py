import cv2
import socket
import numpy as np
import time

# UDP soketi oluştur
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_host = '0.0.0.0'  # Tüm arayüzlerden gelen bağlantıları kabul et
udp_port = 5005
server_address = (udp_host, udp_port)

# Kamera ayarları
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

# Paket boyutu (UDP için genellikle 65507 byte maksimum)
PACKET_SIZE = 65500

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
            
        # Frame'i sıkıştır
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        _, buffer = cv2.imencode('.jpg', frame, encode_param)
        
        # Byte dizisine çevir
        data = buffer.tobytes()
        data_size = len(data)
        
        # Paketleri gönder
        for i in range(0, data_size, PACKET_SIZE):
            chunk = data[i:i+PACKET_SIZE]
            sock.sendto(chunk, server_address)
            
        # Küçük bir bekleme (CPU yükünü azaltmak için)
        time.sleep(0.001)
        
finally:
    cap.release()
    sock.close()