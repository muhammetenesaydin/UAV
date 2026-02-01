import cv2
import socket
import struct
import numpy as np

# Kullanıcıdan sender (PC) IP adresini alma
sender_ip = '192.168.0.53'
sender_port = 9999  # Sabit port

# ALICI (RECEIVER) AYARLARI
receiver_ip = '192.168.0.52'  # Otomatik olarak kullanılacak (tüm arayüzler)
receiver_port = 9999

# UDP socket oluştur
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((receiver_ip, receiver_port))

print(f"{receiver_port} portunda kamera yayını bekleniyor...")

while True:
    try:
        # Önce veri boyutunu al
        size_data, addr = sock.recvfrom(4)
        if addr[0] != sender_ip:
            continue  # Sadece belirlenen sender'dan geleni al
        total_size = struct.unpack("!I", size_data)[0]
        print(f"Veri alınıyor... Toplam boyut: {total_size} bytes")
        # Veriyi topla
        received_data = b""
        while len(received_data) < total_size:
            try:
                chunk, addr2 = sock.recvfrom(512)
                if addr2[0] != sender_ip:
                    continue
                received_data += chunk
            except Exception as e:
                print("Parça alma hatası:", e)
                break
        if len(received_data) != total_size:
            print("Eksik veri, frame atlandı.")
            continue
        # Görüntüyü çöz
        frame_buffer = np.frombuffer(received_data, np.uint8)
        frame = cv2.imdecode(frame_buffer, cv2.IMREAD_COLOR)
        if frame is not None:
            cv2.imshow('UDP Receiver - Alınan Görüntü', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print("Görüntü çözülemedi!")
    except Exception as e:
        print(f"Veri alma hatası: {e}")
        continue

sock.close()
cv2.destroyAllWindows()