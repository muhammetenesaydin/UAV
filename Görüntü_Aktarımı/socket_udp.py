import socket
import cv2
import numpy as np

# UDP Ayarları
udp_ip = "192.168.221.57"  # Alıcı IP
udp_port = 5005
max_packet_size = 1400  # Güvenli bir UDP paketi boyutu10.192.176.180

# Kamera başlat
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Kamera açılamadı!")
    exit()

# UDP soketi oluştur
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"Görüntü akışı başlatılıyor... Hedef: {udp_ip}:{udp_port}")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Kamera görüntüsü alınamadı!")
            break
        
        # Frame boyutunu küçült
        frame = cv2.resize(frame, (640, 480))

        # JPEG sıkıştırma kalitesi
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        _, buffer = cv2.imencode('.jpg', frame, encode_param)

        # Paketleri böl ve gönder
        data = buffer.tobytes()
        for i in range(0, len(data), max_packet_size):
            chunk = data[i:i+max_packet_size]
            sock.sendto(chunk, (udp_ip, udp_port))
        print(f"Gönderilen paket sayısı: {len(data) // max_packet_size + 1}, Toplam boyut: {len(data)} byte")
        # ESC ile çıkış
        if cv2.waitKey(1) == 27:
            break
finally:
    cap.release()
    print("Kamera serbest bırakıldı.")