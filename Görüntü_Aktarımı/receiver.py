import cv2
import socket
import pickle
import struct
import numpy as np

def main():
    print("UDP Receiver - Görüntü Alıcısı")
    print("="*40)
    
    # Kullanıcıdan IP ve port bilgisini al
    while True:
        try:
            sender_ip = input("Sender IP adresini girin: ").strip()
            sender_port = int(input("Sender Port numarasını girin: ").strip())
            break
        except ValueError:
            print("Geçersiz port numarası! Tekrar deneyin.")
        except KeyboardInterrupt:
            print("\nProgram kapatılıyor...")
            return
    
    # UDP socket oluştur
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print(f"\n{sender_ip}:{sender_port} adresine bağlanılıyor...")
    print("Görüntü alınmaya başlanıyor...")
    print("Çıkmak için 'q' tuşuna basın\n")
    
    # Sender'a bağlantı isteği gönder
    try:
        client_socket.sendto(b"REQUEST", (sender_ip, sender_port))
    except Exception as e:
        print(f"Bağlantı hatası: {e}")
        client_socket.close()
        return
    
    # Veri alma buffer'ı
    data = b""
    payload_size = struct.calcsize("L")
    
    while True:
        try:
            # Veri boyutunu al
            while len(data) < payload_size:
                packet, addr = client_socket.recvfrom(4096)
                if not packet:
                    break
                data += packet
            
            if len(data) < payload_size:
                print("Bağlantı kesildi!")
                break
            
            # Veri boyutunu çıkar
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("L", packed_msg_size)[0]
            
            # Tüm veriyi al
            while len(data) < msg_size:
                packet, addr = client_socket.recvfrom(4096)
                if not packet:
                    break
                data += packet
            
            if len(data) < msg_size:
                print("Veri alma hatası!")
                break
            
            # Görüntü verisini al
            frame_data = data[:msg_size]
            data = data[msg_size:]
            
            # Görüntüyü çöz
            frame_buffer = pickle.loads(frame_data)
            frame = cv2.imdecode(frame_buffer, cv2.IMREAD_COLOR)
            
            # Alıcı bilgisini görüntü üzerine yaz
            cv2.putText(frame, f"Alici: {sender_ip}:{sender_port}", (10, frame.shape[0] - 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(frame, "UDP Receiver - Alinan Goruntu", (10, frame.shape[0] - 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # Görüntüyü göster
            cv2.imshow('UDP Receiver - Alınan Görüntü', frame)
            
            # 'q' tuşuna basılırsa çık
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
        except socket.timeout:
            print("Zaman aşımı! Sender'dan veri gelmiyor.")
            continue
        except Exception as e:
            print(f"Hata: {e}")
            break
    
    # Temizleme
    cv2.destroyAllWindows()
    client_socket.close()
    print("UDP Receiver kapatıldı.")

if __name__ == "__main__":
    main() 