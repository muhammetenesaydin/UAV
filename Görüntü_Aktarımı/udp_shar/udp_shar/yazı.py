import cv2
import socket
import struct
import numpy as np
import time

def receive_data_in_chunks(socket_obj, timeout=5.0):
    """Parçalar halinde gelen veriyi toplar"""
    try:
        socket_obj.settimeout(timeout)
        
        # İlk olarak toplam veri boyutunu al
        size_data, addr = socket_obj.recvfrom(4)
        total_size = struct.unpack("!I", size_data)[0]
        
        print(f"Veri alınıyor... Toplam boyut: {total_size} bytes")
        
        # Veriyi topla
        received_data = b""
        while len(received_data) < total_size:
            chunk, addr = socket_obj.recvfrom(1024)
            received_data += chunk
            
            # İlerleme göster
            progress = (len(received_data) / total_size) * 100
            print(f"\rİlerleme: {progress:.1f}%", end="", flush=True)
        
        print()  # Yeni satır
        return received_data
        
    except socket.timeout:
        print("Veri alma zaman aşımı!")
        return None
    except Exception as e:
        print(f"Veri alma hatası: {e}")
        return None

def main():
    print("UDP Receiver - Görüntü Alıcısı")
    print("="*40)
    
    # Kullanıcıdan sender (PC) IP adresini alma
    while True:
        try:
            sender_ip = input("Sender (PC) IP adresini girin: ").strip()
            sender_port = 9999  # Sabit port
            break
        except KeyboardInterrupt:
            print("\nProgram kapatılıyor...")
            return
    
    # ALICI (RECEIVER) AYARLARI
    receiver_ip = ''  # Otomatik olarak kullanılacak (tüm arayüzler)
    receiver_port = 5000
    
    # UDP socket oluştur
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((receiver_ip, receiver_port))
    
    print(f"{receiver_port} portunda mesaj bekleniyor...")

    while True:
        data, addr = sock.recvfrom(1024)
        print(f"{addr[0]}:{addr[1]} -> {data.decode('utf-8')}")
        break

    sock.close()

if __name__ == "__main__":
    main()